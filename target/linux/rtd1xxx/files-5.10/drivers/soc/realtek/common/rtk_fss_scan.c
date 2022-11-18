// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek FSS Scan Driver
 *
 * Copyright (c) 2020-2021 Realtek Semiconductor Corp.
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/iopoll.h>
#include <linux/workqueue.h>
#include <soc/realtek/rtk_fss.h>

struct fss_scan_device {
	struct device             *dev;
	struct fss_control        *fss_ctl;
	struct clk                *clk;
	struct regulator          *supply;

	unsigned long             saved_freq;
	int                       saved_volt;

	int                       target;
	int                       num_freqs;
	int                       scan_num;
	int                       *freqs;
	int                       volt_max;
	int                       volt_min;

	int                       *volts;

	struct work_struct        work;
	int                       freeze_task;
};

static int fss_scan_set_freq_volt(struct fss_scan_device *fdev, int freq, int volt)
{
	long old_freq = clk_get_rate(fdev->clk);
	int ret;

	/* always set the voltage */
	if (freq >= old_freq) {
		ret = regulator_set_voltage(fdev->supply, volt, volt);
		if (ret)
			return ret;
	}

	ret = clk_set_rate(fdev->clk, freq);
	if (ret)
		return ret;

	if (freq < old_freq) {
		ret = regulator_set_voltage(fdev->supply, volt, volt);
		if (ret)
			return ret;
	}
	return 0;
}

static void fss_scan_save_state(struct fss_scan_device *fdev)
{
	fdev->saved_volt = regulator_get_voltage(fdev->supply);
	fdev->saved_freq = clk_get_rate(fdev->clk);
}

static void fss_scan_restore_state(struct fss_scan_device *fdev)
{
	fss_scan_set_freq_volt(fdev, fdev->saved_freq, fdev->saved_volt);
}

static int check_calibration_data(unsigned int bitmask, unsigned int val, unsigned int target)
{
	int i;

	for (i = ffs(bitmask); i ; i = ffs(bitmask)) {
		i -= 1;
		bitmask &= ~BIT(i);

		if (((val >> (i * 4)) & 0xf) < target)
			return 0;
	}

	return 1;
}

static int fss_scan_calibrate_and_check(struct fss_scan_device *fdev, struct fss_calibration_data *data)
{
	int target = fdev->target;
	int ret;

	ret = fss_control_get_calibration_data(fdev->fss_ctl, data);
	if (ret) {
		dev_err(fdev->dev, "failed to get calibration_data: %d\n", ret);
		return 0;
	}

	return check_calibration_data(data->sensor_bitmask, data->result, target) &&
		check_calibration_data(data->dsu_sensor_bitmask, data->dsu_result, target);
}

static int fss_scan_iterate_voltage(struct fss_scan_device *fdev, int volt_init)
{
	int ret;
	int best = 0;
	int volt_min = fdev->volt_min;
	int target_volt_min;
	int target_volt_max;

	if (!volt_init)
		return 0;

	target_volt_max = target_volt_min = volt_init;

	while (target_volt_min >= volt_min) {
		struct fss_calibration_data data = { 0 };

		dev_dbg(fdev->dev, "request volt=(%d-%d)\n", target_volt_min, target_volt_max);

		ret = regulator_set_voltage(fdev->supply, target_volt_min, target_volt_max);
		if (ret) {
			dev_err(fdev->dev, "failed to set volt: %d\n", ret);
			return 0;
		}


		/* must wait more time, when lowering voltage */
		msleep(20);

		ret = fss_scan_calibrate_and_check(fdev, &data);
		if (!ret)
			return best;
		dev_info(fdev->dev, "set volt=%d, bitmap=(%x,%x), result=(%x,%x)\n",
			regulator_get_voltage(fdev->supply),
			data.sensor_bitmask, data.dsu_sensor_bitmask,
			data.result, data.dsu_result);


		best = regulator_get_voltage(fdev->supply);
		target_volt_max = best - 1;
		target_volt_min = best - 12500;
	}

	return best;
}

static int fss_scan(struct fss_scan_device *fdev, int freq, int volt_init)
{
	int ret;

	ret = fss_scan_set_freq_volt(fdev, freq, volt_init);
	if (ret) {
		dev_err(fdev->dev, "failed to set freq=%dMHz, volt=%d\n",
			freq / 1000000, volt_init);
		return 0;
	}

	msleep(200);

	ret = fss_scan_iterate_voltage(fdev, volt_init);

	dev_info(fdev->dev, "get result freq=%d, target=0x%x, input=(%d,%d), output=%d\n",
		freq, fdev->target, volt_init, fdev->volt_min, ret);

	return ret;
}

static void fss_scan_work(struct work_struct *work)
{
	struct fss_scan_device *fdev = container_of(work, struct fss_scan_device, work);
	int i, j;
	int volt;
	int error;
	int freeze_task = fdev->freeze_task;

	fdev->supply = regulator_get(fdev->dev, "cpu");
	if (IS_ERR(fdev->supply)) {
		dev_err(fdev->dev, "failed to get regulator: %ld\n",
				PTR_ERR(fdev->supply));
		return;
	}

	fss_scan_save_state(fdev);

	if (freeze_task) {
		error = freeze_processes();
		if (error) {
			dev_err(fdev->dev, "failed to freeze processes: %d\n", error);
			return;
		}
	}

	volt = fdev->volt_max;

	for (i = 0; i < fdev->num_freqs; i++) {
		int best = 0;
		int res;

		for (j = 0; j < fdev->scan_num; j++) {
			res = fss_scan(fdev, fdev->freqs[i], volt);
			if (res == 0)
				continue;
			if (best == 0 || best > res)
				best = res;
		}

		if (!best) {
			dev_err(fdev->dev, "no voltage found\n");
			break;
		}
		volt = fdev->volts[i] = best;
	}

	if (freeze_task)
		thaw_processes();

	fss_scan_restore_state(fdev);

	regulator_put(fdev->supply);
}

static void fss_scan_wait(struct fss_scan_device *fdev)
{
	flush_work(&fdev->work);
}

static int fss_scan_start(struct fss_scan_device *fdev)
{
	if (!queue_work_on(0, system_highpri_wq, &fdev->work))
		return -EBUSY;
	return 0;
}

static ssize_t freeze_task_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct fss_scan_device *fdev = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = strtobool(buf, &enable);
	if (ret)
		return ret;

	fdev->freeze_task = enable;

	return count;
}

static ssize_t freeze_task_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct fss_scan_device *fdev = dev_get_drvdata(dev);

	return  snprintf(buf, PAGE_SIZE, "%d\n", fdev->freeze_task);
}
DEVICE_ATTR_RW(freeze_task);

static ssize_t control_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct fss_scan_device *fdev = dev_get_drvdata(dev);
	int ret = 0;

	if (!strncmp("start", buf, 5))
		ret = fss_scan_start(fdev);
	else if (!strncmp("wait", buf, 4))
		fss_scan_wait(fdev);
	else
		ret = -EINVAL;

	return ret ?: count;
}
DEVICE_ATTR_WO(control);

static
ssize_t frequencies_mhz_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	int len = 0;
	int i;
	struct fss_scan_device *fdev = dev_get_drvdata(dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "%d", fdev->freqs[0] / 1000000);
	for (i = 1; i < fdev->num_freqs; i++)
		len += snprintf(buf + len, PAGE_SIZE - len, " %d", fdev->freqs[i] / 1000000);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	return len;
}
DEVICE_ATTR_RO(frequencies_mhz);

static ssize_t voltages_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct fss_scan_device *fdev = dev_get_drvdata(dev);
	int len = 0;
	int i;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d", fdev->volts[0]);
	for (i = 1; i < fdev->num_freqs; i++)
		len += snprintf(buf + len, PAGE_SIZE - len, " %d",
				fdev->volts[i]);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	return len;
}
DEVICE_ATTR_RO(voltages);

static ssize_t target_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct fss_scan_device *fdev = dev_get_drvdata(dev);
	int len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%08x\n", fdev->target);
	return len;
}
DEVICE_ATTR_RO(target);

static struct attribute *fss_scan_attrs[] = {
	&dev_attr_control.attr,
	&dev_attr_voltages.attr,
	&dev_attr_frequencies_mhz.attr,
	&dev_attr_freeze_task.attr,
	NULL
};

static struct attribute_group fss_scan_attr_group = {
	.name = "scan",
	.attrs = fss_scan_attrs,
};

static int of_parse_config(struct fss_scan_device *fdev, struct device_node *np)
{
	int ret;

	ret = of_property_read_u32(np, "fss-target", &fdev->target);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "fss-max-voltage", &fdev->volt_max);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "fss-min-voltage", &fdev->volt_min);
	if (ret)
		return ret;

	if (of_property_read_u32(np, "fss-scan-num", &fdev->scan_num))
		fdev->scan_num = 3;

	fdev->num_freqs = of_property_count_u32_elems(np, "fss-frequencies");
	if (fdev->num_freqs < 0)
		return fdev->num_freqs;

	fdev->freqs = devm_kcalloc(fdev->dev, fdev->num_freqs, sizeof(*fdev->freqs), GFP_KERNEL);
	if (!fdev->freqs)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "fss-frequencies", fdev->freqs, fdev->num_freqs);
	if (ret)
		return ret;
	return 0;
}

static int fss_scan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct fss_scan_device *fdev;
	int ret;

	fdev = devm_kzalloc(dev, sizeof(*fdev), GFP_KERNEL);
	if (!fdev)
		return -ENOMEM;
	fdev->dev = dev;

	fdev->fss_ctl = of_fss_control_get(np);
	if (IS_ERR(fdev->fss_ctl)) {
		ret = PTR_ERR(fdev->fss_ctl);
		dev_err(dev, "failed to get fss control: %d\n", ret);
		return ret;
	};

	ret = of_parse_config(fdev, np);
	if (ret) {
		dev_info(dev, "failed to get config from dt: %d\n", ret);
		return ret;
	}

	fdev->volts = devm_kcalloc(dev, fdev->num_freqs, sizeof(*fdev->volts), GFP_KERNEL);
	if (!fdev->volts)
		return -ENOMEM;

	fdev->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(fdev->clk)) {
		ret = PTR_ERR(fdev->clk);
		if (ret == -EPROBE_DEFER)
			dev_dbg(dev, "clk is not ready, retry\n");
		else
			dev_err(dev, "failed to get clk: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, fdev);
	INIT_WORK(&fdev->work, fss_scan_work);

	ret = sysfs_create_group(&dev->kobj, &fss_scan_attr_group);
	if (ret) {
		dev_err(dev, "failed to create sysfs group: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id fss_scan_ids[] = {
	{ .compatible = "realtek,fss-scan" },
	{}
};

static struct platform_driver fss_scan_drv = {
	.driver = {
		.name           = "rtk-fss-scan",
		.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(fss_scan_ids),
	},
	.probe    = fss_scan_probe,
};
module_platform_driver(fss_scan_drv);

MODULE_DESCRIPTION("Realtek FSS Scan driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:rtk-fss-scan");

