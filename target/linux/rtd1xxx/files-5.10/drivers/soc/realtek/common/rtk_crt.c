// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 */

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>

struct rtk_crt_pm_data {
	int ofs;
	uint32_t ignore_bits;
	uint32_t write_en_bits;
	uint32_t val;
};

struct rtk_crt_platform_desc {
	int pm_data_num;
	struct rtk_crt_pm_data *pm_data;
};

struct rtk_crt_data {
	struct regmap *regmap;
	const struct rtk_crt_platform_desc *desc;
};

static struct rtk_crt_pm_data rtd1295_pm_data[] = {
	/* SOFT_RESET */
	{ .ofs = 0x000, .ignore_bits = 0x00001000, },
	{ .ofs = 0x004, },
	/* CLK_EN */
	{ .ofs = 0x00C, .ignore_bits = 0x0001C100, },
	{ .ofs = 0x010, },
	{ .ofs = 0x450, .ignore_bits = 0xFFFFFFFB, },
	/* PLL_GPU */
	{ .ofs = 0x1C4, },
	{ .ofs = 0x5A4, .ignore_bits = ~(0x7FFFF), },
	/* PLL_VE1 */
	{ .ofs = 0x118, },
	{ .ofs = 0x114, .ignore_bits = ~(0x63FF0), },
	/* PLL_VE2 */
	{ .ofs = 0x1D4, },
	{ .ofs = 0x1D0, .ignore_bits = ~(0x63FF0), },
};

static const struct rtk_crt_platform_desc rtd1295_crt_desc = {
	.pm_data_num = ARRAY_SIZE(rtd1295_pm_data),
	.pm_data = rtd1295_pm_data,
};

static struct rtk_crt_pm_data rtd1395_pm_data[] = {
	/* SOFT_RESET */
	{ .ofs = 0x00, .ignore_bits = 0x00001000, },
	{ .ofs = 0x04, },
	{ .ofs = 0x50, },
	/* CLK_EN */
	{ .ofs = 0x0C, .ignore_bits = 0x0004c524, },
	{ .ofs = 0x10, .ignore_bits = 0xc0fe0030, },
	/* PLL_GPU */
	{ .ofs = 0x1C4, },
	{ .ofs = 0x5A4, .ignore_bits = ~(0x7FFFF), },
	/* PLL_VE1 */
	{ .ofs = 0x118, },
	{ .ofs = 0x114, .ignore_bits = ~(0x63FF0), },
	/* PLL_VE2 */
	{ .ofs = 0x1D4, },
	{ .ofs = 0x1D0, .ignore_bits = ~(0x63FF0), },
};

static const struct rtk_crt_platform_desc rtd1395_crt_desc = {
	.pm_data_num = ARRAY_SIZE(rtd1395_pm_data),
	.pm_data = rtd1395_pm_data,
};

static struct rtk_crt_pm_data rtd1619_pm_data[] = {
	/* SOFT_RESET */
	{ .ofs = 0x00, .write_en_bits = 0xAAAAAAAA, },
	{ .ofs = 0x04, .write_en_bits = 0xAAAAAAAA, },
	{ .ofs = 0x08, .write_en_bits = 0x0A80AAAA, },
	{ .ofs = 0x0C, .write_en_bits = 0x2AAAAAAA, },
	{ .ofs = 0x68, .write_en_bits = 0xAAAAAAAA, },
	/* CLK_EN */
	{ .ofs = 0x50, .write_en_bits = 0xA0A8288A, },
	{ .ofs = 0x54, .write_en_bits = 0xAAAAAAA8, },
	{ .ofs = 0x58, .write_en_bits = 0xA800082A, },
	{ .ofs = 0x5C, .write_en_bits = 0xAAAAAA0A, },
	/* PLL_GPU */
	{ .ofs = 0x1C4, },
	{ .ofs = 0x5A4, .ignore_bits = ~(0x7FFFF), },
	/* PLL_VE1 */
	{ .ofs = 0x118, },
	{ .ofs = 0x114, .ignore_bits = ~(0x63FF0), },
	/* PLL_VE2 */
	{ .ofs = 0x1D4, },
	{ .ofs = 0x1D0, .ignore_bits = ~(0x63FF0), },
};

static const struct rtk_crt_platform_desc rtd1619_crt_desc = {
	.pm_data_num = ARRAY_SIZE(rtd1619_pm_data),
	.pm_data = rtd1619_pm_data,
};

static struct rtk_crt_pm_data rtd1319_pm_data[] = {
	/* SOFT_RESET */
	{ .ofs = 0x000, .write_en_bits = 0xAAAAAAAA, },
	{ .ofs = 0x004, .write_en_bits = 0xAAAAAAAA, },
	{ .ofs = 0x008, .write_en_bits = 0x0A80AAAA, },
	{ .ofs = 0x00C, .write_en_bits = 0x2AAAAAAA, },
	{ .ofs = 0x068, .write_en_bits = 0xAAAAAAAA, },
	{ .ofs = 0x090, .write_en_bits = 0xAAAAAAAA, },
	{ .ofs = 0x454, .ignore_bits = 0xFFFFFFFE, },
	{ .ofs = 0x458, .ignore_bits = 0xFFFFFFFE, },
	{ .ofs = 0x464, .ignore_bits = 0xFFFFFFFE, },
	/* CLK_EN */
	{ .ofs = 0x050, .write_en_bits = 0xA0A8288A, },
	{ .ofs = 0x054, .write_en_bits = 0xAAAAAAA8, },
	{ .ofs = 0x058, .write_en_bits = 0xA800082A, },
	{ .ofs = 0x05C, .write_en_bits = 0xAAAAAA0A, },
	{ .ofs = 0x08C, .write_en_bits = 0xAAAAAAAA, },
	/* PLL_GPU */
	{ .ofs = 0x1C4, },
	{ .ofs = 0x5A4, .ignore_bits = ~(0x7FFFF), },
	/* PLL_VE1 */
	{ .ofs = 0x118, },
	{ .ofs = 0x114, .ignore_bits = ~(0x63FF0), },
	/* PLL_VE2 */
	{ .ofs = 0x1D4, },
	{ .ofs = 0x1D0, .ignore_bits = ~(0x63FF0), },
};

static const struct rtk_crt_platform_desc rtd1319_crt_desc = {
	.pm_data_num = ARRAY_SIZE(rtd1319_pm_data),
	.pm_data = rtd1319_pm_data,
};

static struct rtk_crt_pm_data rtd1619b_pm_data[] = {
	/* SOFT_RESET */
	{ .ofs = 0x000, .write_en_bits = 0xAAAAAAAA, },
	{ .ofs = 0x004, .write_en_bits = 0xAAAAAAAA, },
	{ .ofs = 0x008, .write_en_bits = 0x0A80AAAA, },
	{ .ofs = 0x00C, .write_en_bits = 0x2AAAAAAA, },
	{ .ofs = 0x068, .write_en_bits = 0xAAAAAAAA, },
	{ .ofs = 0x090, .write_en_bits = 0xAA2AAAAA, },
	{ .ofs = 0x454, .ignore_bits = 0xFFFFFFFE, },
	{ .ofs = 0x458, .ignore_bits = 0xFFFFFFFE, },
	{ .ofs = 0x464, .ignore_bits = 0xFFFFFFFE, },
	/* CLK_EN */
	{ .ofs = 0x050, .write_en_bits = 0xA0A8288A, },
	{ .ofs = 0x054, .write_en_bits = 0xAAAAAAA8, },
	{ .ofs = 0x058, .write_en_bits = 0xA800082A, },
	{ .ofs = 0x05C, .write_en_bits = 0xAAAAAA0A, },
	{ .ofs = 0x08C, .write_en_bits = 0xAAAA2AAA, },
	/* PLL_GPU */
	{ .ofs = 0x1C4, },
	{ .ofs = 0x5A4, .ignore_bits = ~(0x7FFFF), },
	/* PLL_VE1 */
	{ .ofs = 0x118, },
	{ .ofs = 0x114, .ignore_bits = ~(0x63FF0), },
	/* PLL_VE2 */
	{ .ofs = 0x1D4, },
	{ .ofs = 0x1D0, .ignore_bits = ~(0x63FF0), },
};

static const struct rtk_crt_platform_desc rtd1619b_crt_desc = {
	.pm_data_num = ARRAY_SIZE(rtd1619b_pm_data),
	.pm_data = rtd1619b_pm_data,
};


static int rtk_crt_suspend(struct device *dev)
{
	struct rtk_crt_data *crt = dev_get_drvdata(dev);
	const struct rtk_crt_platform_desc *desc = crt->desc;
	int i;

	for (i = 0; i < desc->pm_data_num; i++) {
		struct rtk_crt_pm_data *pm_data = &desc->pm_data[i];

		regmap_read(crt->regmap, pm_data->ofs, &pm_data->val);
	}
	return 0;
}

static int rtk_crt_resume(struct device *dev)
{
	struct rtk_crt_data *crt = dev_get_drvdata(dev);
	const struct rtk_crt_platform_desc *desc = crt->desc;
	int i;

	for (i = desc->pm_data_num - 1; i >= 0; i--) {
		struct rtk_crt_pm_data *pm_data = &desc->pm_data[i];
		uint32_t val = pm_data->val;
		uint32_t mask = ~0x0;

		if (pm_data->write_en_bits)
			val |= pm_data->write_en_bits;
		if (pm_data->ignore_bits)
			mask &= ~pm_data->ignore_bits;
		dev_info(dev, "resuming: ofs=%03x, vs=%08x vr=%08x m=%08x\n",
			pm_data->ofs, pm_data->val, val, mask);
		regmap_update_bits(crt->regmap, pm_data->ofs, mask, val);
	}
	return 0;
}

const struct dev_pm_ops rtk_crt_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(rtk_crt_suspend, rtk_crt_resume)
};

static int rtk_crt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rtk_crt_data *crt;
	int ret;

	crt = devm_kzalloc(dev, sizeof(*crt), GFP_KERNEL);
	if (!crt)
		return -ENOMEM;

	crt->desc = of_device_get_match_data(dev);
	if (!crt->desc)
		return -EINVAL;

	crt->regmap = syscon_node_to_regmap(np);
	if (IS_ERR(crt->regmap)) {
		ret = PTR_ERR(crt->regmap);
		dev_err(dev, "failed to get syscon: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, crt);
	return 0;
}

static const struct of_device_id rtk_crt_match[] = {
	{ .compatible = "realtek,rtd1295-crt", .data = &rtd1295_crt_desc, },
	{ .compatible = "realtek,rtd1395-crt", .data = &rtd1395_crt_desc, },
	{ .compatible = "realtek,rtd1619-crt", .data = &rtd1619_crt_desc, },
	{ .compatible = "realtek,rtd1319-crt", .data = &rtd1319_crt_desc, },
	{ .compatible = "realtek,rtd1619b-crt", .data = &rtd1619b_crt_desc, },
	{}
};
MODULE_DEVICE_TABLE(of, rtk_crt_match);

static struct platform_driver rtk_crt_driver = {
	.probe  = rtk_crt_probe,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-crt",
		.pm             = &rtk_crt_pm_ops,
		.of_match_table = of_match_ptr(of_match_ptr(rtk_crt_match)),
	},
};
module_platform_driver(rtk_crt_driver);

MODULE_DESCRIPTION("Realtek CRT driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-crt");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
