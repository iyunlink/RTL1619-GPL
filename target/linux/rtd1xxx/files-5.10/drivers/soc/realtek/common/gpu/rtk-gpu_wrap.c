// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Realtek Semiconductor Corp.
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/printk.h>
#include <linux/reset.h>
#include <linux/slab.h>

struct rtk_gpu_wrap_data {
	struct device *dev;
	struct clk *clk;
	struct reset_control *rstc;
	struct reset_control *rstc_bist;
	void __iomem *reg_bist;
	int auto_bist;
	struct regmap *dbgprot;
	int clk_cnt;
};

static int bist_test_is_normal(struct rtk_gpu_wrap_data *data)
{
	unsigned int val;

	if (!data->dbgprot)
		return 0;
	regmap_read(data->dbgprot, 0, &val);
	return (val >> 4) == 7;
}

static void rtd1319d_gpu_bisr(struct rtk_gpu_wrap_data *data)
{
	ktime_t start;
	s64 delta_us;
	unsigned int regval;
	int ret;

	start = ktime_get();
	if (data->auto_bist)
		goto polling;

	writel(0x00000031, data->reg_bist + 0x10);
polling:
	ret = readl_poll_timeout(data->reg_bist + 0x40, regval, (regval & 0x1) == 0x1, 0, 1000000);
	delta_us = ktime_to_us(ktime_sub(ktime_get(), start));
	if (ret)
		dev_warn(data->dev, "error %pe: bisr failed: status=%08x, time=%lldus\n",
			ERR_PTR(ret), regval, delta_us);
}

static int rtk_gpu_wrap_power_on(struct rtk_gpu_wrap_data *data)
{
	reset_control_deassert(data->rstc);
	reset_control_deassert(data->rstc_bist);
	clk_prepare_enable(data->clk);

	dmb(sy);

	rtd1319d_gpu_bisr(data);

	return 0;
}

static int rtk_gpu_wrap_power_off(struct rtk_gpu_wrap_data *data)
{
	dmb(sy);

	clk_disable_unprepare(data->clk);
	reset_control_assert(data->rstc);
	reset_control_assert(data->rstc_bist);
	return 0;
}

static int rtk_gpu_wrap_runtime_suspend(struct device *dev)
{
	struct rtk_gpu_wrap_data *data = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	rtk_gpu_wrap_power_off(data);
	return 0;
}

static int rtk_gpu_wrap_runtime_resume(struct device *dev)
{
	struct rtk_gpu_wrap_data *data = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	rtk_gpu_wrap_power_on(data);
	return 0;
}

static int rtk_gpu_wrap_suspend(struct device *dev)
{
	struct rtk_gpu_wrap_data *data = dev_get_drvdata(dev);
	int ret;

	dev_info(dev, "enter %s\n", __func__);

	ret = pm_runtime_force_suspend(dev);

	/* gpu may enter suspend, with clk enabled, disable it */
	data->clk_cnt = 0;
	if (__clk_is_enabled(data->clk) && data->clk_cnt < 10) {
		data->clk_cnt++;
		clk_disable_unprepare(data->clk);
	}
	if (data->clk_cnt)
		dev_err(dev, "do clk_disable_unprepare %d times\n", data->clk_cnt);

	dev_info(dev, "exit %s\n", __func__);
	return ret;
}

static int rtk_gpu_wrap_resume(struct device *dev)
{
	struct rtk_gpu_wrap_data *data = dev_get_drvdata(dev);
	int i;

	dev_info(dev, "enter %s\n", __func__);

	pm_runtime_force_resume(dev);

	for (i = 0; i < data->clk_cnt; i++)
		clk_prepare_enable(data->clk);

	dev_info(dev, "exit %s\n", __func__);
	return 0;
}

static const struct dev_pm_ops rtk_gpu_wrap_dev_pm_ops = {
	.runtime_resume  = rtk_gpu_wrap_runtime_resume,
	.runtime_suspend = rtk_gpu_wrap_runtime_suspend,
	.suspend         = rtk_gpu_wrap_suspend,
	.resume          = rtk_gpu_wrap_resume,
};

static int rtk_gpu_wrap_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_gpu_wrap_data *data;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->dev = dev;

	data->reg_bist = devm_platform_ioremap_resource_byname(pdev, "bist");
	if (IS_ERR(data->reg_bist))
		return PTR_ERR(data->reg_bist);

	data->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(data->clk))
		return dev_err_probe(dev, PTR_ERR(data->clk), "failed to get clk\n");

	data->rstc = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(data->rstc))
		return dev_err_probe(dev, PTR_ERR(data->rstc), "failed to get reset\n");

	data->rstc_bist = devm_reset_control_get_exclusive(dev, "bist");
	if (IS_ERR(data->rstc_bist))
		return dev_err_probe(dev, PTR_ERR(data->rstc_bist), "failed to get bist reset\n");

	data->dbgprot = syscon_regmap_lookup_by_phandle(dev->of_node, "realtek,dbgprot");
	if (IS_ERR(data->dbgprot)) {
		dev_warn(dev, "error %pe: failed to get dbgprot syscon\n", data->dbgprot);
		data->dbgprot = NULL;
	}

	if (data->dbgprot)
		data->auto_bist = bist_test_is_normal(data);

	platform_set_drvdata(pdev, data);

	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);

	return devm_of_platform_populate(dev);
}

static int rtk_gpu_wrap_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void rtk_gpu_wrap_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id rtk_gpu_wrap_match[] = {
	{ .compatible = "realtek,rtd1319d-gpu-wrap", },
	{}
};

static struct platform_driver rtk_gpu_wrap_driver = {
	.probe    = rtk_gpu_wrap_probe,
	.remove   = rtk_gpu_wrap_remove,
	.shutdown = rtk_gpu_wrap_shutdown,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-gpu_wrap",
		.of_match_table = of_match_ptr(rtk_gpu_wrap_match),
		.pm             = &rtk_gpu_wrap_dev_pm_ops,
	},
};
module_platform_driver(rtk_gpu_wrap_driver);

MODULE_DESCRIPTION("Realtek GPU Wrapper driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-gpu_wrap");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
