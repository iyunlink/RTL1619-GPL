// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Realtek Semiconductor Corp.
 */

#include <linux/clk/clk-conf.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/printk.h>
#include <linux/sysfs.h>

static int rtk_acpu_ve1_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	pm_runtime_enable(dev);

	pm_runtime_get_sync(dev);
	return 0;
}

static void rtk_acpu_ve1_restore_clk_config(struct device *dev)
{
	struct device_node *np = of_get_child_by_name(dev->of_node, "clk-config");

	if (!np) {
		dev_info(dev, "no clk config\n");
		return;
	}

	pm_runtime_force_suspend(dev);

	of_clk_set_defaults(np, false);
	of_node_put(np);

	pm_runtime_force_resume(dev);
}

static int rtk_acpu_ve1_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	rtk_acpu_ve1_restore_clk_config(dev);

	pm_runtime_put(dev);

	pm_runtime_disable(dev);
	return 0;
}

static const struct of_device_id rtk_acpu_ve1_match[] = {
	{ .compatible = "realtek,acpu-ve1", },
	{}
};

static struct platform_driver rtk_acpu_ve1_driver = {
	.probe    = rtk_acpu_ve1_probe,
	.remove   = rtk_acpu_ve1_remove,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-acpu-ve1",
		.of_match_table = of_match_ptr(rtk_acpu_ve1_match),
	},
};

static __init int rtk_acpu_ve1_init(void)
{
        return platform_driver_register(&rtk_acpu_ve1_driver);
}
rootfs_initcall(rtk_acpu_ve1_init);

MODULE_DESCRIPTION("Realtek Audio CPU Codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-acpu-ve1");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");

int rtk_acpu_release_ve1(void)
{
	struct device_node *np;
	struct platform_device *pdev;

	for_each_compatible_node(np, NULL, rtk_acpu_ve1_match[0].compatible) {
		pdev = of_find_device_by_node(np);
		if (!pdev)
			continue;

		dev_info(&pdev->dev, "remove platform device\n");
		of_platform_device_destroy(&pdev->dev, NULL);
	}
	return 0;
}
