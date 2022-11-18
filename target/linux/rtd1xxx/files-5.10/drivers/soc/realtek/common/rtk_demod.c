// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek Demod Driver
 *
 * Copyright (C) 2018-2019,2022 Realtek Semiconductor Corp.
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>

#define DISP_CABLERX_MISC 0x264

struct rtk_demod_quirks {
	int dptmx_cfg: 1;
};

struct rtk_demod_device {
	struct miscdevice     mdev;
	struct device         *dev;
	struct clk            *pll;
	struct clk            *clk;
	struct reset_control  *rstc;
	const char            *name;
	phys_addr_t           addr;
	resource_size_t       size;

	struct regmap         *dptmx;
	int                   dptmx_idx;
	const struct rtk_demod_quirks *quirks;
};

struct rtk_demod_ctx {
	struct rtk_demod_device *dmdev;
};

static int rtk_demod_open(struct inode *inode, struct file *filp)
{
	struct rtk_demod_device *dmdev = container_of(filp->private_data,
						      struct rtk_demod_device,
						      mdev);
	struct rtk_demod_ctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dmdev = dmdev;
	filp->private_data = ctx;

	pm_runtime_get_sync(dmdev->dev);

	return 0;
}

static int rtk_demod_release(struct inode *inode, struct file *filp)
{
	struct rtk_demod_ctx *ctx = filp->private_data;
	struct rtk_demod_device *dmdev = ctx->dmdev;

	pm_runtime_put_sync(dmdev->dev);
	kfree(ctx);
	return 0;
}

static const struct vm_operations_struct rtk_demod_vm_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys,
#endif
};

static int rtk_demod_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct rtk_demod_ctx *ctx = filp->private_data;
	struct rtk_demod_device *dmdev = ctx->dmdev;

	if (vma->vm_end < vma->vm_start)
		return -EINVAL;

	if (dmdev->addr & ~PAGE_MASK)
		return -EINVAL;

	if (vma->vm_end - vma->vm_start > dmdev->size)
		return -EINVAL;

	if (vma->vm_pgoff > (dmdev->size >> PAGE_SHIFT))
		return -EINVAL;

	vma->vm_ops  = &rtk_demod_vm_ops;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return remap_pfn_range(vma,
			       vma->vm_start,
			       dmdev->addr >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot);
}


static const struct file_operations rtk_demod_fops = {
	.owner         = THIS_MODULE,
	.open          = rtk_demod_open,
	.release       = rtk_demod_release,
	.mmap          = rtk_demod_mmap,
};

static void rtk_demod_power_on(struct rtk_demod_device *dmdev)
{
	clk_prepare_enable(dmdev->pll);
	clk_prepare_enable(dmdev->clk);
	reset_control_deassert(dmdev->rstc);

	if (dmdev->quirks->dptmx_cfg) {
		unsigned int mask = dmdev->dptmx_idx == 0 ? 0x075: 0x70a;
		unsigned int val = dmdev->dptmx_idx == 0 ? 0x035: 0x30a;

		regmap_update_bits(dmdev->dptmx, DISP_CABLERX_MISC, mask, val);
	}
}

static void rtk_demod_power_off(struct rtk_demod_device *dmdev)
{
	 reset_control_assert(dmdev->rstc);
	 clk_disable_unprepare(dmdev->clk);
	 clk_disable_unprepare(dmdev->pll);
}

static int rtk_demod_runtime_resume(struct device *dev)
{
	struct rtk_demod_device *dmdev = dev_get_drvdata(dev);

	dev_dbg(dev, "enter %s\n", __func__);
	rtk_demod_power_on(dmdev);
	dev_dbg(dev, "exit %s\n", __func__);
	return 0;
}

static int rtk_demod_runtime_suspend(struct device *dev)
{
	struct rtk_demod_device *dmdev = dev_get_drvdata(dev);

	dev_dbg(dev, "enter %s\n", __func__);
	rtk_demod_power_off(dmdev);
	dev_dbg(dev, "exit %s\n", __func__);
	return 0;
}

static const struct dev_pm_ops rtk_demod_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(rtk_demod_runtime_suspend, rtk_demod_runtime_resume, NULL)
};

static int rtk_demod_probe(struct platform_device *pdev)
{
	struct rtk_demod_device *dmdev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;
	struct resource res;

	dmdev = devm_kzalloc(dev, sizeof(*dmdev), GFP_KERNEL);
	if (!dmdev)
		return -ENOMEM;

	dmdev->dev = dev;
	ret = of_property_read_string(dev->of_node, "label", &dmdev->name);
	if (ret)
		dmdev->name = "demod";

	dmdev->quirks = of_device_get_match_data(dev);
	if (!dmdev->quirks) {
		dev_err(dev, "failed to get match data\n");
		return -EINVAL;
	}

	if (dmdev->quirks->dptmx_cfg) {
		dmdev->dptmx = syscon_regmap_lookup_by_phandle_args(np,	"realtek,dptmx", 1, &dmdev->dptmx_idx);
		if (IS_ERR(dmdev->dptmx)) {
			ret = PTR_ERR(dmdev->dptmx);
			dev_err(dev, "failed to get dptmx: %d\n", ret);
			return ret;
		}
	}

	dmdev->clk = devm_clk_get(dev, "cablerx");
	if (IS_ERR(dmdev->clk)) {
		ret = PTR_ERR(dmdev->clk);
		dev_err(dev, "cablerx: failed to get clk: %d\n", ret);
		return ret;
	}

	dmdev->pll = devm_clk_get(dev, "pll");
	if (IS_ERR(dmdev->pll)) {
		ret = PTR_ERR(dmdev->pll);
		dev_err(dev, "pll: failed to get clk: %d\n", ret);
		return ret;
	}

	dmdev->rstc = devm_reset_control_get(dev, NULL);
	if (IS_ERR(dmdev->rstc)) {
		ret = PTR_ERR(dmdev->rstc);
		dev_err(dev, "failed to get reset_control: %d\n", ret);
		return ret;
	}

	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(dev, "failed to get address: %d\n", ret);
		return ret;
	}

	dmdev->addr = res.start;
	dmdev->size = ALIGN(resource_size(&res), PAGE_SIZE);

	dmdev->mdev.minor  = MISC_DYNAMIC_MINOR;
	dmdev->mdev.name   = dmdev->name;
	dmdev->mdev.fops   = &rtk_demod_fops;
	dmdev->mdev.parent = dev;
	ret = misc_register(&dmdev->mdev);
	if (ret) {
		dev_err(dev, "failed to register misc device: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, dmdev);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	return 0;
}

static int rtk_demod_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_demod_device *dmdev = platform_get_drvdata(pdev);

	pm_runtime_disable(dev);
	platform_set_drvdata(pdev, NULL);
	misc_deregister(&dmdev->mdev);
	return 0;
}

static const struct rtk_demod_quirks default_quirks;
static const struct rtk_demod_quirks rtd1319d_quirks = { .dptmx_cfg = 1 };

static const struct of_device_id rtk_demod_ids[] = {
	{ .compatible = "realtek,demod", .data = &default_quirks, },
	{ .compatible = "realtek,rtd1319d-demod", .data = &rtd1319d_quirks, },
	{}
};
MODULE_DEVICE_TABLE(of, rtk_demod_ids);

static struct platform_driver rtk_demod_driver = {
	.probe = rtk_demod_probe,
	.remove = rtk_demod_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "rtk-demod",
		.of_match_table = rtk_demod_ids,
		.pm = &rtk_demod_pm_ops,
	},
};
module_platform_driver(rtk_demod_driver);

MODULE_DESCRIPTION("Reatek Demod Driver");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");
