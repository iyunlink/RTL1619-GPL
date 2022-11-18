// SPDX-License-Identifier: GPL-2.0-only
/*
 * Regmap Poweroff Driver
 * Copyright (C) 2020 Realtek Semiconductor Corporation
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

static struct regmap *map;
static u32 offset;
static u32 mask;
static u32 val;

static void regmap_poweroff(void)
{
	regmap_update_bits(map, offset, mask, val);
	mdelay(1000);

	pr_emerg("Unable to poweroff system\n");
}

static int regmap_poweroff_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	if (map)
		return -EINVAL;

	map = dev_get_regmap(dev->parent, NULL);
	if (!map)
		return -ENODEV;

	if (of_property_read_u32(np, "poweroff-offset", &offset)) {
		dev_err(dev, "unable to read 'poweroff-offset'");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "poweroff-mask", &mask)) {
		dev_err(dev, "unable to read 'poweroff-mask'");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "poweroff-value", &val)) {
		dev_err(dev, "unable to read 'poweroff-value'");
		return -EINVAL;
	}

	if (!of_device_is_system_power_controller(dev->of_node)) {
		dev_info(dev, "not a system power controller\n");
		return 0;
	}

	WARN(pm_power_off, "pm_power_off is already assigned with %pf\n",
		pm_power_off);
	pm_power_off = regmap_poweroff;

	return 0;
}

static int regmap_poweroff_remove(struct platform_device *pdev)
{
	if (pm_power_off == regmap_poweroff)
		pm_power_off = NULL;

	return 0;
}

static const struct of_device_id regmap_poweroff_of_match[] = {
	{ .compatible = "regmap-poweroff" },
	{}
};

static struct platform_driver regmap_poweroff_driver = {
	.probe = regmap_poweroff_probe,
	.remove = regmap_poweroff_remove,
	.driver = {
		.name = "regmap-poweroff",
		.of_match_table = regmap_poweroff_of_match,
	},
};
module_platform_driver(regmap_poweroff_driver);
