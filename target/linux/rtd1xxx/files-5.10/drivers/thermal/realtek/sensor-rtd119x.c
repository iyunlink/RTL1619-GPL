// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2018 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/delay.h>
#include <linux/nvmem-consumer.h>
#include <linux/slab.h>
#include "sensor.h"

struct sensor_data {
	int calibration_data;
};

static int get_calibration_data(struct thermal_sensor_device *tdev, int index,
				int *cal)
{
	struct device *dev = tdev->dev;
	char cell_name[20];
	struct nvmem_cell *cell;
	unsigned char *buf;
	size_t buf_size;

	*cal = 0;

	sprintf(cell_name, "calibration%d", index);
	cell = nvmem_cell_get(dev, cell_name);
	if (IS_ERR(cell)) {
		dev_warn(dev, "nvmem_cell_get() returns %ld\n", PTR_ERR(cell));
		goto done;
	}
	buf = nvmem_cell_read(cell, &buf_size);
	nvmem_cell_put(cell);
	dev_info(dev, "sensor%d: calibration=%d\n", index, (int)buf[0]);
	*cal = (int)buf[0] * 1000;
	kfree(buf);
done:
	return 0;
}

static int rtd119x_sensor_init(struct thermal_sensor_device *tdev)
{
	struct sensor_data *data;

	data = kcalloc(2, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	get_calibration_data(tdev, 0, &data[0].calibration_data);
	get_calibration_data(tdev, 1, &data[1].calibration_data);
	tdev->priv_data = data;
	return 0;
}

static void rtd119x_sensor_exit(struct thermal_sensor_device *tdev)
{
	kfree(tdev->priv_data);
}

static void rtd119x_sensor_get_temp_index(struct thermal_sensor_device *tdev,
					  int index, int *temp)
{
	int i;
	u64 t;
	u32 val, cmp_data;
	u32 off = index ? 0x4 : 0x0;
	struct sensor_data *data = tdev->priv_data;
	u32 cal = data[index].calibration_data;

	/* enable */
	thermal_sensor_device_reg_write(tdev, off, 0x0000c80c);
	usleep_range(15, 25);

	/* thermal detect */
	for (i = 6; i >= 0; i--) {
		thermal_sensor_device_reg_read(tdev, off, &val);
		cmp_data = BIT(i + 5);
		thermal_sensor_device_reg_write(tdev, off, cmp_data);
		usleep_range(15, 25);

		thermal_sensor_device_reg_read(tdev, off, &val);
		if ((val & BIT(16)) == 0)
			thermal_sensor_device_reg_write(tdev, off, ~cmp_data & val);
	}

	/* get temperture */
	thermal_sensor_device_reg_read(tdev, off, &val);
	t = (val >> 5) & 0x7f;
	t *= 1000;
	t -= cal;
	t = (t * 165) / 128;
	if (cal)
		t += 25 * 1000;
	else
		t -= 47 * 1000;

	dev_dbg(tdev->dev, "sensor%d: reg=%05x, t=%lld, calibration_data=%d\n",
		index, val, t, cal);
	*temp = (int)t;
}

static
int rtd119x_sensor_get_temp(struct thermal_sensor_device *tdev, int *temp)
{
	int t0, t1;

	rtd119x_sensor_get_temp_index(tdev, 0, &t0);
	rtd119x_sensor_get_temp_index(tdev, 1, &t1);

	*temp = t0 > t1 ? t0 : t1;
	return 0;
}

static const struct thermal_sensor_hw_ops rtd119x_hw_ops = {
	.get_temp = rtd119x_sensor_get_temp,
	.init     = rtd119x_sensor_init,
	.exit     = rtd119x_sensor_exit,
};

const struct thermal_sensor_desc rtd119x_sensor_desc = {
	.hw_ops = &rtd119x_hw_ops,
};
