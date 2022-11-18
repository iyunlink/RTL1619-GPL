// SPDX-License-Identifier: GPL-2.0-only
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include "hse.h"

static int hse_hw_init(struct hse_device *hse_dev)
{
	unsigned int val;

	val = hse_should_disable_bypass_en(hse_dev) ? 0 : 1;
	hse_write(hse_dev, HSE_REG_BYPASS, val);
	return 0;
}

static int hse_runtime_suspend(struct device *dev)
{
	struct hse_device *hse_dev = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	clk_disable_unprepare(hse_dev->clk);
	reset_control_assert(hse_dev->rstc_bist);
	reset_control_assert(hse_dev->rstc);

	return 0;
}

static int hse_runtime_resume(struct device *dev)
{
	struct hse_device *hse_dev = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	reset_control_deassert(hse_dev->rstc);
	reset_control_deassert(hse_dev->rstc_bist);
	clk_prepare_enable(hse_dev->clk);
	hse_hw_init(hse_dev);

	return 0;
}

static const struct dev_pm_ops hse_pm_ops = {
	.runtime_suspend = hse_runtime_suspend,
	.runtime_resume  = hse_runtime_resume,
};

static irqreturn_t hse_interrupt(int irq, void *dev_id)
{
	struct hse_device *hse_dev = dev_id;

	hse_engine_handle_interrupt(&hse_dev->eng);
	return IRQ_HANDLED;
}

static int hse_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct hse_device *hse_dev;
	struct resource res;
	int ret;

	hse_dev = devm_kzalloc(dev, sizeof(*hse_dev), GFP_KERNEL);
	if (!hse_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, hse_dev);
	hse_dev->dev = dev;
	hse_dev->quirks = of_device_get_match_data(dev);

	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(dev, "failed to get recource: %d\n", ret);
		return -ENODEV;
	}

	hse_dev->base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!hse_dev->base)
		return -ENOMEM;

	hse_dev->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(hse_dev->clk)) {
		ret = PTR_ERR(hse_dev->clk);
		dev_err(dev, "failed to get clk: %d\n", ret);
		return ret;
	}

	hse_dev->rstc = devm_reset_control_get_optional(dev, "core");
	if (IS_ERR(hse_dev->rstc)) {
		ret = PTR_ERR(hse_dev->rstc);
		dev_dbg(dev, "failed to get reset control: %d\n", ret);
		hse_dev->rstc = NULL;
	}

	hse_dev->rstc_bist = devm_reset_control_get_optional(dev, "bist");
	if (IS_ERR(hse_dev->rstc_bist)) {
		ret = PTR_ERR(hse_dev->rstc_bist);
		dev_dbg(dev, "failed to get reset control: %d\n", ret);
		hse_dev->rstc_bist = NULL;
	}

	hse_dev->irq = platform_get_irq(pdev, 0);
	if (hse_dev->irq < 0) {
		ret = hse_dev->irq;
		dev_err(dev, "failed to get irq: %d\n", ret);
		return ret;
	}

	ret = devm_request_irq(dev, hse_dev->irq, hse_interrupt, IRQF_SHARED,
			       dev_name(dev), hse_dev);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		return ret;
	}

	hse_dev->pool = dma_pool_create(dev_name(dev), dev, PAGE_SIZE, 0, 0);
	if (!hse_dev->pool) {
		dev_err(dev, "failed to allocate dma pool\n");
		return -ENOMEM;
	}

	hse_setup_miscdevice(hse_dev);
	if (ret)
		dev_err(dev, "error %pe: failed to register miscdevice\n", ERR_PTR(ret));
	else
		hse_dev->miscdevice_ready = 1;

	ret = hse_setup_dmaengine(hse_dev);
	if (ret)
		dev_err(dev, "error %pe: failed to setup dmaengine\n", ERR_PTR(ret));
	else
		hse_dev->dmaengine_ready = 1;

	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);

	pm_runtime_get_sync(dev);
	hse_engine_init(hse_dev, &hse_dev->eng, 1);

	return 0;
}

static int hse_remove(struct platform_device *pdev)
{
	struct hse_device *hse_dev = platform_get_drvdata(pdev);

	if (hse_dev->dmaengine_ready)
		hse_teardown_dmaengine(hse_dev);
	if (hse_dev->miscdevice_ready)
		hse_teardown_miscdevice(hse_dev);
	dma_pool_destroy(hse_dev->pool);
	pm_runtime_disable(hse_dev->dev);
	reset_control_assert(hse_dev->rstc);
	return 0;
}

static const struct hse_quirks rtd1619b_quirks = {
	.bypass_en_disable = 1,
	.xor_copy_v2 = 1,
};

static const struct of_device_id hse_ids[] = {
	{ .compatible = "realtek,rtd1319-hse" },
	{ .compatible = "realtek,rtd1619b-hse", .data =  &rtd1619b_quirks, },
	{}
};

static struct platform_driver hse_driver = {
	.probe  = hse_probe,
	.remove = hse_remove,
	.driver = {
		.name           = "rtk-hse",
		.owner          = THIS_MODULE,
		.of_match_table = hse_ids,
		.pm             = &hse_pm_ops,
		.probe_type     = PROBE_PREFER_ASYNCHRONOUS,
	},
};
module_platform_driver(hse_driver);

MODULE_LICENSE("GPL");
