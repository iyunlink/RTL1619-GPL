// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 */

#include <linux/init.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "rtk_sb2.h"

#define SB2_ICG_REG_MAX    4

struct sb2_icg_desc {
	int offset;
	int num_icg_reg;
	unsigned int icg_val[SB2_ICG_REG_MAX];

};

static void sb2_enable_icg(struct sb2_device *sb2_dev)
{
	const struct sb2_icg_desc *desc = sb2_dev->icg_desc;

	regmap_bulk_write(sb2_dev->regmap,  desc->offset, desc->icg_val,
		desc->num_icg_reg);
}

static const struct sb2_icg_desc sb2_icg_desc_rtd1295 = {
	.offset = 0x308,
	.num_icg_reg = 3,
	.icg_val = { 0xffffffff, 0x0000ffff, 0x00000007 },
};

static const struct sb2_icg_desc sb2_icg_desc_rtd1395 = {
	.offset = 0x308,
	.num_icg_reg = 3,
	.icg_val = { 0xffffffff, 0x000fe7ff, 0x03ff003f },
};

static const struct sb2_icg_desc sb2_icg_desc_rtd1619 = {
	.offset = 0x308,
	.num_icg_reg = 3,
	.icg_val = { 0xffffffff, 0xffffffff, 0xf73f007f },
};

static const struct sb2_icg_desc sb2_icg_desc_rtd1319 = {
	.offset = 0xff0,
	.num_icg_reg = 4,
	.icg_val = { 0xffffffff, 0xffffffff, 0x00000007, 0xff7f00ff },
};


static int rtk_sb2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct sb2_device *sb2_dev;
	int ret;

	const struct sb2_icg_desc *desc;

	desc = of_device_get_match_data(dev);
	if (!desc)
		return -EINVAL;

	sb2_dev = devm_kzalloc(dev, sizeof(*sb2_dev), GFP_KERNEL);
	if (!sb2_dev)
		return -ENOMEM;

	sb2_dev->regmap = syscon_node_to_regmap(np);
	if (IS_ERR(sb2_dev->regmap)) {
		ret = PTR_ERR(sb2_dev->regmap);
		dev_err(dev, "failed to get syscon: %d\n", ret);
		return ret;
	}

	sb2_dev->icg_desc = desc;

	platform_set_drvdata(pdev, sb2_dev);
	sb2_enable_icg(sb2_dev);
	return 0;
}

static int rtk_sb2_resume(struct device *dev)
{
	struct sb2_device *sb2_dev = dev_get_drvdata(dev);

	dev_info(dev, "enter %s\n", __func__);
	sb2_enable_icg(sb2_dev);
	dev_info(dev, "exit %s\n", __func__);
	return 0;
}

static const struct dev_pm_ops rtk_sb2_pm_ops = {
	.resume  = rtk_sb2_resume,
};

static const struct of_device_id rtk_sb2_match[] = {
	{
		.compatible = "realtek,rtd1295-sysbrg2",
		.data = &sb2_icg_desc_rtd1295,
	},
	{
		.compatible = "realtek,rtd1395-sysbrg2",
		.data = &sb2_icg_desc_rtd1395,
	},
	{
		.compatible = "realtek,rtd1619-sysbrg2",
		.data = &sb2_icg_desc_rtd1619,
	},
	{
		.compatible = "realtek,rtd1319-sysbrg2",
		.data = &sb2_icg_desc_rtd1319,
	},
	{}
};
MODULE_DEVICE_TABLE(of, rtk_sb2_match);

static struct platform_driver rtk_sb2_driver = {
	.probe  = rtk_sb2_probe,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-sb2",
		.pm             = &rtk_sb2_pm_ops,
		.of_match_table = of_match_ptr(rtk_sb2_match),
	},
};

static int __init rtk_sb2_init(void)
{
	return platform_driver_register(&rtk_sb2_driver);
}
subsys_initcall_sync(rtk_sb2_init);

MODULE_DESCRIPTION("Realtek SB2 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-sb2");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
