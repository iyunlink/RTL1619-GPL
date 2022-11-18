// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2018 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include "sensor.h"

/*
 * thermal sensor offset
 */
#define TM_SENSOR_CTRL0    0x00
#define TM_SENSOR_CTRL1    0x04
#define TM_SENSOR_CTRL2    0x08
#define TM_SENSOR_STATUS0  0x40
#define TM_SENSOR_STATUS1  0x44

/* external control reg offset */
#define TM2_SENSOR_CTRL3  0x604

static void rtd1619_sensor_reset(struct thermal_sensor_device *tdev)
{
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL0, 0x07ce7ae1);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL1, 0x00378228);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL2, 0x00011114);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL2, 0x00031114);
}

static void sc_wrap_disable_sensor_latch(struct thermal_sensor_device *tdev)
{
	regmap_write(tdev->regmap, TM2_SENSOR_CTRL3, 0xfffffffe);
}

static void rtd1319_sensor_reset(struct thermal_sensor_device *tdev)
{
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL0, 0x08130000);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL1, 0x003723ff);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL2, 0x00011114);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL2, 0x00031114);

	sc_wrap_disable_sensor_latch(tdev);
}

static void rtd1619b_sensor_reset(struct thermal_sensor_device *tdev)
{
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL0, 0x09000000);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL1, 0x00364400);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL2, 0x00011194);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL2, 0x00031194);

	sc_wrap_disable_sensor_latch(tdev);
}

static void rtd1312c_sensor_reset(struct thermal_sensor_device *tdev)
{
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL0, 0x080A0000);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL1, 0x00372732);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL2, 0x00011114);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL2, 0x00031114);

	sc_wrap_disable_sensor_latch(tdev);
}

static int rtd1619_sensor_init(struct thermal_sensor_device *tdev)
{
	tdev->desc->hw_ops->reset(tdev);
	return 0;
}

static inline int rtd1619_sensor_get_temp(struct thermal_sensor_device *tdev,
	int *temp)
{
	unsigned int val = 0;

	thermal_sensor_device_reg_read(tdev, TM_SENSOR_STATUS0, &val);
	*temp = sign_extend32(val, 18) * 1000 / 1024;
	return 0;
}

static const struct thermal_sensor_hw_ops rtd1619_sensor_hw_ops = {
	.get_temp = rtd1619_sensor_get_temp,
	.reset    = rtd1619_sensor_reset,
	.init     = rtd1619_sensor_init,
};

const struct thermal_sensor_desc rtd1619_sensor_desc = {
	.hw_ops = &rtd1619_sensor_hw_ops,
	.reset_time_ms = 12,
	.status_ofs = TM_SENSOR_STATUS0,
	.status_cnt = 2,
};

static const struct thermal_sensor_hw_ops rtd1319_sensor_hw_ops = {
	.get_temp = rtd1619_sensor_get_temp,
	.reset    = rtd1319_sensor_reset,
	.init     = rtd1619_sensor_init,
};

const struct thermal_sensor_desc rtd1319_sensor_desc = {
	.hw_ops = &rtd1319_sensor_hw_ops,
	.reset_time_ms = 12,
	.status_ofs = TM_SENSOR_STATUS0,
	.status_cnt = 2,
};

static const struct thermal_sensor_hw_ops rtd1619b_sensor_hw_ops = {
	.get_temp = rtd1619_sensor_get_temp,
	.reset    = rtd1619b_sensor_reset,
	.init     = rtd1619_sensor_init,
};

const struct thermal_sensor_desc rtd1619b_sensor_desc = {
	.hw_ops = &rtd1619b_sensor_hw_ops,
	.reset_time_ms = 12,
	.status_ofs = TM_SENSOR_STATUS0,
	.status_cnt = 2,
};

static const struct thermal_sensor_hw_ops rtd1312c_sensor_hw_ops = {
	.get_temp = rtd1619_sensor_get_temp,
	.reset    = rtd1312c_sensor_reset,
	.init     = rtd1619_sensor_init,
};

const struct thermal_sensor_desc rtd1312c_sensor_desc = {
	.hw_ops = &rtd1312c_sensor_hw_ops,
	.reset_time_ms = 12,
	.status_ofs = TM_SENSOR_STATUS0,
	.status_cnt = 2,
};

static void rtd1319d_sensor_reset(struct thermal_sensor_device *tdev)
{
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL0, 0x08e80000);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL1, 0x00365199);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL2, 0x00011194);
	thermal_sensor_device_reg_write(tdev, TM_SENSOR_CTRL2, 0x00031194);

	regmap_write(tdev->regmap, TM2_SENSOR_CTRL3, 0);
}

static const struct thermal_sensor_hw_ops rtd1319d_sensor_hw_ops = {
	.get_temp = rtd1619_sensor_get_temp,
	.reset    = rtd1319d_sensor_reset,
	.init     = rtd1619_sensor_init,
};

const struct thermal_sensor_desc rtd1319d_sensor_desc = {
	.hw_ops = &rtd1319d_sensor_hw_ops,
	.reset_time_ms = 12,
	.status_ofs = TM_SENSOR_STATUS0,
	.status_cnt = 2,
};
