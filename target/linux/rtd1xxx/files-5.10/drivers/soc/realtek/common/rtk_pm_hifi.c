// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Realtek Semiconductor Corp.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include "internal.h" // regulator/internal.h
#include <soc/realtek/rtk_pm.h>

struct rtk_pm_hifi_data {
	struct device *dev;
	struct pm_private *pm_dev;
	struct regulator *supp;
};

static int rtk_pm_hifi_set_suspend_state(struct rtk_pm_hifi_data *data, int en)
{
	int ret;

	if (en)
		ret = regulator_suspend_enable(data->supp->rdev, PM_SUSPEND_MEM);
	else
		ret = regulator_suspend_disable(data->supp->rdev, PM_SUSPEND_MEM);
	return ret;
}

static int rtk_pm_hifi_prepare(struct device *dev)
{
	struct rtk_pm_hifi_data *data = dev_get_drvdata(dev);
	int ret;

	if (data->pm_dev->pcpu_param->wakeup_source & BIT(HIFI_EVENT))
		ret = rtk_pm_hifi_set_suspend_state(data, 1);
	else
		ret = rtk_pm_hifi_set_suspend_state(data, 0);
	if (ret)
		dev_err(data->dev, "%s: rtk_pm_hifi_set_suspend_state return %d\n", __func__, ret);
	return 0;
}

static const struct dev_pm_ops rtk_pm_hifi_dev_pm_ops = {
	.prepare = rtk_pm_hifi_prepare,
};

static int rtk_pm_hifi_get_pm_dev(struct rtk_pm_hifi_data  *data)
{
	struct device_node *np;
	struct platform_device *pdev;

	np = of_parse_phandle(data->dev->of_node, "realtek,pm-device", 0);
	if (!np)
		return -EINVAL;

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -ENODEV;

	data->pm_dev = platform_get_drvdata(pdev);
	if (!data->pm_dev)
		return -EPROBE_DEFER;

	return 0;
}

static int rtk_pm_hifi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_pm_hifi_data  *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->supp = devm_regulator_get(dev, "hifi");
	if (IS_ERR(data->supp)) {
		ret = PTR_ERR(data->supp);
		if (ret == -EPROBE_DEFER)
			dev_dbg(dev, "hifi supply not ready, retry\n");
		else
			dev_err(dev, "failed to get hifi supply: %d\n", ret);
		return ret;
	}

	ret = rtk_pm_hifi_get_pm_dev(data);
	if (ret) {
		if (ret == -EPROBE_DEFER)
			dev_dbg(dev, "pm_dev not ready, retry\n");
		else
			dev_err(dev, "failed to get pm_dev: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, data);

	return 0;
}

static int rtk_pm_hifi_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id rtk_pm_hifi_match[] = {
	{ .compatible = "realtek,pm-hifi", },
	{}
};

static struct platform_driver rtk_pm_hifi_driver = {
	.probe    = rtk_pm_hifi_probe,
	.remove   = rtk_pm_hifi_remove,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-pm-hifi",
		.of_match_table = of_match_ptr(rtk_pm_hifi_match),
		.pm             = &rtk_pm_hifi_dev_pm_ops,
	},
};
module_platform_driver(rtk_pm_hifi_driver);

MODULE_DESCRIPTION("Realtek PM Hifi driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-pm-hifi");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
