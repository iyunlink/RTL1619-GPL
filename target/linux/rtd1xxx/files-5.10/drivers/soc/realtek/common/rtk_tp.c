// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek TP Driver
 *
 * Copyright (C) 2019-2020 Realtek Semiconductor Corp.
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/list.h>
#include <linux/pm_runtime.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#define TP_MODULE_NUM_MAX    3

struct tp_module {
	struct clk            *clk;
	phys_addr_t           addr;
	resource_size_t       size;
};

struct rtk_tp_device {
	struct miscdevice     mdev;
	struct device         *dev;

	/* map */
	struct tp_module      tpm[TP_MODULE_NUM_MAX];
	int num_tpm;

	/* cp */
	struct regmap        *cp;
};

struct rtk_tp_ctx {
	struct rtk_tp_device *tpdev;
};

static int rtk_tp_open(struct inode *inode, struct file *filp)
{
	struct rtk_tp_device *tpdev = container_of(filp->private_data,
						      struct rtk_tp_device,
						      mdev);
	struct rtk_tp_ctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->tpdev = tpdev;
	filp->private_data = ctx;

	pm_runtime_get_sync(tpdev->dev);

	return 0;
}

static int rtk_tp_release(struct inode *inode, struct file *filp)
{
	struct rtk_tp_ctx *ctx = filp->private_data;
	struct rtk_tp_device *tpdev = ctx->tpdev;

	pm_runtime_put_sync(tpdev->dev);
	kfree(ctx);
	return 0;
}

static const struct vm_operations_struct rtk_tp_vm_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys,
#endif
};

static int rtk_tp_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct rtk_tp_ctx *ctx = filp->private_data;
	struct rtk_tp_device *tpdev = ctx->tpdev;
	phys_addr_t addr;
	resource_size_t size;


	if (vma->vm_end < vma->vm_start)
		return -EINVAL;

	if (vma->vm_pgoff >= tpdev->num_tpm)
		return -E2BIG;
	addr = tpdev->tpm[vma->vm_pgoff].addr;
	size = tpdev->tpm[vma->vm_pgoff].size;

	if (addr & ~PAGE_MASK)
		return -EINVAL;

	if ((vma->vm_end - vma->vm_start) > size)
		return -EINVAL;

	vma->vm_ops = &rtk_tp_vm_ops;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return remap_pfn_range(vma,
			       vma->vm_start,
			       addr >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot);
}

#define RTK_TP_IOCTL_GET_MOUDLE_NUM       _IOR('C', 0x01, unsigned int)
#define RTK_TP_IOCTL_CONFIGURE_CP_REG     _IOW('C', 0x04, struct rtk_tp_config_reg)

struct rtk_tp_config_reg {
	uint32_t offset;
	uint32_t mask;
	uint32_t value;
};

static int rtk_tp_ioctl_configure_cp_reg(struct rtk_tp_device *tpdev, unsigned long arg)
{
	struct rtk_tp_config_reg conf;
	int ret;

	if (!tpdev->cp)
		return -EINVAL;

	ret = copy_from_user(&conf, (void __user *)arg, sizeof(conf));
	if (ret)
		return ret;

	if (conf.offset != 0x198 && conf.offset != 0x830)
		return -EINVAL;
	dev_info(tpdev->dev, "%s: %03x %08x %08x\n", __func__, conf.offset, conf.mask, conf.value);

	return regmap_update_bits(tpdev->cp, conf.offset, conf.mask, conf.value);
}

static long rtk_tp_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	struct rtk_tp_ctx *ctx = filp->private_data;
	struct rtk_tp_device *tpdev = ctx->tpdev;
	unsigned int num_tpm;
	int ret = 0;

	switch (cmd) {
	case RTK_TP_IOCTL_GET_MOUDLE_NUM:
		num_tpm = tpdev->num_tpm;
		ret = copy_to_user((unsigned int __user *)arg,
				&num_tpm, sizeof(unsigned int));
		break;

	case RTK_TP_IOCTL_CONFIGURE_CP_REG:
		return rtk_tp_ioctl_configure_cp_reg(tpdev, arg);

	default:
		return -EFAULT;
	}
	return ret;
}

static const struct file_operations rtk_tp_fops = {
	.owner          = THIS_MODULE,
	.open           = rtk_tp_open,
	.release        = rtk_tp_release,
	.mmap           = rtk_tp_mmap,
	.unlocked_ioctl = rtk_tp_ioctl,
	.compat_ioctl   = compat_ptr_ioctl,
};

static void rtk_tp_power_on(struct rtk_tp_device *tpdev)
{
	struct tp_module *tpm;
	int i;

	for (i = 0; i < tpdev->num_tpm; i++) {
		tpm = &tpdev->tpm[i];
		clk_prepare_enable(tpm->clk);
	}
}

static void rtk_tp_power_off(struct rtk_tp_device *tpdev)
{
	struct tp_module *tpm;
	int i;

	for (i = tpdev->num_tpm - 1; i >= 0; i--) {
		tpm = &tpdev->tpm[i];
		clk_disable_unprepare(tpm->clk);
	}
}

static int rtk_tp_runtime_resume(struct device *dev)
{
	struct rtk_tp_device *tpdev = dev_get_drvdata(dev);

	dev_dbg(dev, "enter %s\n", __func__);
	rtk_tp_power_on(tpdev);
	dev_dbg(dev, "exit %s\n", __func__);
	return 0;
}

static int rtk_tp_runtime_suspend(struct device *dev)
{
	struct rtk_tp_device *tpdev = dev_get_drvdata(dev);

	dev_dbg(dev, "enter %s\n", __func__);
	rtk_tp_power_off(tpdev);
	dev_dbg(dev, "exit %s\n", __func__);
	return 0;
}

static const struct dev_pm_ops rtk_tp_pm_ops = {
	.runtime_suspend = rtk_tp_runtime_suspend,
	.runtime_resume  = rtk_tp_runtime_resume,
	.suspend         = pm_runtime_force_suspend,
	.resume          = pm_runtime_force_resume,
};

static int rtk_tp_init_tpm(struct rtk_tp_device *tpdev, int i)
{
	struct tp_module *tpm = &tpdev->tpm[i];
	struct device_node *np = tpdev->dev->of_node;
	struct resource res;
	int ret;

	ret = of_address_to_resource(np, i, &res);
	if (ret)
		return ret;

	tpm->clk = of_clk_get(np, i);
	if (IS_ERR(tpm->clk))
		return PTR_ERR(tpm->clk);

	tpm->addr = res.start;
	tpm->size = ALIGN(resource_size(&res), PAGE_SIZE);
	return 0;
}

static void rtk_tp_fini_tpm(struct rtk_tp_device *tpdev, int i)
{
	struct tp_module *tpm = &tpdev->tpm[i];

	clk_put(tpm->clk);
}

static int of_count_tpm_num(struct device_node *np)
{
	int num_reg = of_property_count_u32_elems(np, "reg");
	int num_a_cells = of_n_addr_cells(np);
	int num_n_cells = of_n_addr_cells(np);

	if (!num_reg || !num_a_cells || !num_n_cells)
		return -EINVAL;

	return num_reg / (num_a_cells + num_n_cells);
}
static int rtk_tp_probe(struct platform_device *pdev)
{
	struct rtk_tp_device *tpdev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;
	int i;

	tpdev = devm_kzalloc(dev, sizeof(*tpdev), GFP_KERNEL);
	if (!tpdev)
		return -ENOMEM;
	tpdev->dev = dev;

	tpdev->num_tpm = of_count_tpm_num(np);
	if (tpdev->num_tpm < 0)
		return tpdev->num_tpm;
	if (tpdev->num_tpm > TP_MODULE_NUM_MAX)
		return -EINVAL;

	for (i = 0; i < tpdev->num_tpm; i++) {
		ret = rtk_tp_init_tpm(tpdev, i);
		if (ret)
			goto error;
	}

	tpdev->cp = syscon_regmap_lookup_by_phandle(np, "realtek,cp");
	if (IS_ERR(tpdev->cp)) {
		dev_warn(dev, "failed to get cp syscon: %ld\n", PTR_ERR(tpdev->cp));
		tpdev->cp = NULL;
	}

	tpdev->mdev.minor  = MISC_DYNAMIC_MINOR;
	tpdev->mdev.name   = "tp";
	tpdev->mdev.fops   = &rtk_tp_fops;
	tpdev->mdev.parent = dev;
	ret = misc_register(&tpdev->mdev);
	if (ret) {
		dev_err(dev, "failed to register misc device: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, tpdev);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	dev_info(dev, "initialized\n");
	return 0;
error:
	dev_err(dev, "failed to init tpm%d: %d\n", i, ret);
	for (; i >= 0; i--)
		rtk_tp_fini_tpm(tpdev, i);
	return ret;
}

static int rtk_tp_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_tp_device *tpdev = platform_get_drvdata(pdev);
	int i;

	pm_runtime_disable(dev);
	platform_set_drvdata(pdev, NULL);
	misc_deregister(&tpdev->mdev);
	for (i = 0; i < tpdev->num_tpm; i++)
		rtk_tp_fini_tpm(tpdev, i);
	return 0;
}

static const struct of_device_id rtk_tp_ids[] = {
	{ .compatible = "realtek,tp" },
	{}
};
MODULE_DEVICE_TABLE(of, rtk_tp_ids);

static struct platform_driver rtk_tp_driver = {
	.probe = rtk_tp_probe,
	.remove = rtk_tp_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "rtk-tp",
		.of_match_table = rtk_tp_ids,
		.pm = &rtk_tp_pm_ops,

	},
};
module_platform_driver(rtk_tp_driver);

MODULE_DESCRIPTION("Realtek TP Driver");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");
