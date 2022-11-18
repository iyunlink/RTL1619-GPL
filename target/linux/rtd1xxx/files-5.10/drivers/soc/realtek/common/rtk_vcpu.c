// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021 Realtek Semiconductor Corp.
 */

#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/printk.h>
#include <linux/reset.h>
#include <linux/slab.h>

struct rtk_vcpu_data {
	struct device *dev;
	struct clk *clk;
	struct reset_control *rstc;
};

static int rtk_vcpu_power_on(struct rtk_vcpu_data *vcpu_data)
{
	reset_control_deassert(vcpu_data->rstc);
	clk_prepare_enable(vcpu_data->clk);
	return 0;
}

static int rtk_vcpu_power_off(struct rtk_vcpu_data *vcpu_data)
{
	clk_disable_unprepare(vcpu_data->clk);
	reset_control_assert(vcpu_data->rstc);
	return 0;
}

static int rtk_vcpu_runtime_suspend(struct device *dev)
{
	struct rtk_vcpu_data *vcpu_data = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	rtk_vcpu_power_off(vcpu_data);
	return 0;
}

static int rtk_vcpu_runtime_resume(struct device *dev)
{
	struct rtk_vcpu_data *vcpu_data = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	rtk_vcpu_power_on(vcpu_data);
	return 0;
}

static const struct dev_pm_ops rtk_vcpu_dev_pm_ops = {
	.runtime_resume = rtk_vcpu_runtime_resume,
	.runtime_suspend = rtk_vcpu_runtime_suspend,
};

static int rtk_vcpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_vcpu_data *vcpu_data;
	int ret;

	vcpu_data = devm_kzalloc(dev, sizeof(*vcpu_data), GFP_KERNEL);
	if (!vcpu_data)
		return -ENOMEM;
	vcpu_data->dev = dev;

	vcpu_data->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(vcpu_data->clk)) {
		ret = PTR_ERR(vcpu_data->clk);
		if (ret == -ENOENT)
			vcpu_data->clk = NULL;
		else {
			dev_err(dev, "failed to get clk: %d\n", ret);
			return ret;
		}
	}

	vcpu_data->rstc = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(vcpu_data->rstc)) {
		ret = PTR_ERR(vcpu_data->rstc);
		dev_err(dev, "failed to get reset control: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, vcpu_data);
	dev_pm_syscore_device(dev, true);

	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);

	pm_runtime_get_sync(dev);
	return 0;
}

static int rtk_vcpu_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void rtk_vcpu_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id rtk_vcpu_match[] = {
	{ .compatible = "realtek,rtd1319-vcpu", },
	{}
};

static struct platform_driver rtk_vcpu_driver = {
	.probe    = rtk_vcpu_probe,
	.remove   = rtk_vcpu_remove,
	.shutdown = rtk_vcpu_shutdown,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-vcpu",
		.of_match_table = of_match_ptr(rtk_vcpu_match),
		.pm             = &rtk_vcpu_dev_pm_ops,
	},
};
module_platform_driver(rtk_vcpu_driver);

MODULE_DESCRIPTION("Realtek Video CPU driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-vcpu");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
