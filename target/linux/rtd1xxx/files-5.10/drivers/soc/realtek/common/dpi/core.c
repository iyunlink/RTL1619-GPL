// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#include "dpi.h"

#define MIS_DUMMY2  0xe4

int dpi_register_notifier(struct dpi_device *dpi, struct notifier_block *nb)
{
	return raw_notifier_chain_register(&dpi->notifier_list, nb);
}
EXPORT_SYMBOL(dpi_register_notifier);

int dpi_unregister_notifier(struct dpi_device *dpi, struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&dpi->notifier_list, nb);
}
EXPORT_SYMBOL(dpi_unregister_notifier);

static int dpi_notify_event(struct dpi_device *dpi, unsigned long event, void *v)
{
	return raw_notifier_call_chain(&dpi->notifier_list, event, v);
}

static void dpi_restart_checking(struct dpi_device *dpi)
{
	mod_delayed_work(system_freezable_wq, &dpi->delayed_work,
			 round_jiffies(msecs_to_jiffies(2000)));
}

static void dpi_handle_thermal_status(struct dpi_device *dpi, int temp)
{
	struct dpi_event_temp_status_data d;
	unsigned long event;

	d.temp = temp;

	if (temp > dpi->hot_temp && dpi->hot_temp > 0)
		event = DPI_EVENT_TEMP_STATUS_HOT;
	else if (temp < dpi->cold_temp && dpi->cold_temp > 0)
		event = DPI_EVENT_TEMP_STATUS_COLD;
	else
		event = DPI_EVENT_TEMP_STATUS_NORMAL;
	dpi_notify_event(dpi, event, &d);
}

static void dpi_handle_base_temp_delta(struct dpi_device *dpi, int temp)
{
	int dt;
	struct dpi_event_base_temp_data d;

	dt = temp - dpi->base_temp;

	if (dpi->base_temp && dt < dpi->delta_temp && dt > -dpi->delta_temp) {
		return;
	}

	d.old_temp = dpi->base_temp;
	d.new_temp = temp;
	dpi_notify_event(dpi, DPI_EVENT_BASE_TEMP_DIFF_OVER_THERSHOLD, &d);

	dpi->base_temp = temp;
	dpi_notify_event(dpi, DPI_EVENT_BASE_TEMP_UPDATED, &d);
}

static void dpi_temp_check(struct work_struct *work)
{
	struct dpi_device *dpi = container_of(work, struct dpi_device,
		delayed_work.work);
	int t;
	int ret;

	ret = thermal_zone_get_temp(dpi->tzdev, &t);
	if (ret) {
		dev_warn(dpi->dev, "failed to read temp: %d\n", ret);
		goto restart;
	}

	dpi_handle_thermal_status(dpi, t);
	dpi_handle_base_temp_delta(dpi, t);

restart:
	dpi_restart_checking(dpi);
}

enum dpi_dram_type dpi_get_dram_type(struct dpi_device *dpi)
{
	uint32_t val;

	regmap_read(dpi->misc, MIS_DUMMY2, &val);
	val = val & 0xf;

	switch (val) {
	case 0x4:
		return DPI_DRAM_TYPE_DDR4;
	case 0x3:
		return DPI_DRAM_TYPE_DDR3;
	case 0xc:
		return DPI_DRAM_TYPE_LPDDR4;
	}
	return DPI_DRAM_TYPE_UNKNOWN;
}
EXPORT_SYMBOL(dpi_get_dram_type);

static inline int of_dpi_read_temp_range(struct dpi_device *dpi)
{
	struct device *dev = dpi->dev;
	int len, ret;
	int val[2];

	if (!of_find_property(dev->of_node, "temp-range", &len)){
		dev_err(dev, "[DPI] of_find_property error!\n");

		return -EINVAL;
	}

	len /= 4;

	if((ret = of_property_read_u32_array(dev->of_node, "temp-range", val, len)))
		return ret;

	dpi->cold_temp = val[0];
	dpi->hot_temp = val[1];

	if (!of_find_property(dev->of_node, "delta-temp", &len))
		return -EINVAL;

	if (len > sizeof(dpi->delta_temp))
		return -EINVAL;

	if((ret = of_property_read_u32_array(dev->of_node, "delta-temp", &dpi->delta_temp, len/4)))
		return ret;

	// pr_err("%-20s: hot_temp = %d\n", __func__, dpi->hot_temp);
	// pr_err("%-20s: cold_temp = %d\n", __func__, dpi->cold_temp);

	return 0;
}

static int dpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dpi_device *dpi;
	int ret;

	dpi = devm_kzalloc(dev, sizeof(*dpi), GFP_KERNEL);
	if (!dpi)
		return -ENOMEM;

	dpi->misc = syscon_regmap_lookup_by_phandle(dev->of_node,
						    "realtek,misc");
	if (IS_ERR(dpi->misc)) {
		ret = PTR_ERR(dpi->misc);
		dev_err(dev, "failed to get syscon: %d\n", ret);
		return ret;
	}

	dpi->tzdev = thermal_zone_get_zone_by_name("cpu-thermal");
	if (IS_ERR(dpi->tzdev)) {
		ret = PTR_ERR(dpi->tzdev);
		dev_err(dev, "failed to get thermal zone: %d\n", ret);
		return ret;
	}

	dpi->dev = dev;
	if((ret = dpi_check_perm(dpi)))
		return ret;

	if((ret = of_dpi_read_temp_range(dpi)))
		return ret;

	INIT_DELAYED_WORK(&dpi->delayed_work, dpi_temp_check);
	RAW_INIT_NOTIFIER_HEAD(&dpi->notifier_list);
	platform_set_drvdata(pdev, dpi);

	dpi_restart_checking(dpi);

	of_platform_populate(dev->of_node, NULL, NULL, dev);

	return 0;
}

static const struct of_device_id dpi_of_match[] = {
	{ .compatible = "realtek,ddr-phy" },
	{}
};

static struct platform_driver dpi_driver = {
	.driver = {
		.name           = "rtk-dpi",
		.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(dpi_of_match),
	},
	.probe    = dpi_probe,
};
module_platform_driver(dpi_driver);

MODULE_LICENSE("GPL v2");
