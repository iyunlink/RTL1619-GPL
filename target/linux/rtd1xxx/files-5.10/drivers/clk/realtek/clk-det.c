// SPDX-License-Identifier: GPL-2.0-only

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include "clk-det.h"

struct clk_det_desc {
	uint32_t ctrl_rstn_bit;
	uint32_t ctrl_cnten_bit;
	uint32_t stat_ofs;
	uint32_t stat_done_bit;
	uint32_t cnt_mask;
	uint32_t cnt_shift;
};

static const struct clk_det_desc clk_det_descs[2] = {
	[CLK_DET_TYPE_CRT] = {
		0, 1, 0, 30, 0x3FFFC000, 13
	},
	[CLK_DET_TYPE_SC_WRAP] = {
		17, 16, 8, 0, 0x0001FFFE, 1
	},
};

static DEFINE_MUTEX(clk_det_lock);

static unsigned long clk_det_get_freq(struct clk_det *clkd)
{
	const struct clk_det_desc *desc = &clk_det_descs[clkd->type];
	uint32_t ctrl_mask;
	uint32_t val;
	unsigned long freq = 0;
	int ret;

	mutex_lock(&clk_det_lock);

	ctrl_mask = BIT(desc->ctrl_rstn_bit) | BIT(desc->ctrl_cnten_bit);
	regmap_update_bits(clkd->regmap, clkd->ofs, ctrl_mask, 0);
	regmap_update_bits(clkd->regmap, clkd->ofs, ctrl_mask, BIT(desc->ctrl_rstn_bit));
	regmap_update_bits(clkd->regmap, clkd->ofs, ctrl_mask, ctrl_mask);

	ret = regmap_read_poll_timeout(clkd->regmap, clkd->ofs + desc->stat_ofs, val,
			val & BIT(desc->stat_done_bit), 0, 100);
	if (!ret) {
		regmap_read(clkd->regmap, clkd->ofs + desc->stat_ofs, &val);
		freq = ((val & desc->cnt_mask) >> desc->cnt_shift) * 100000;
	}

	regmap_update_bits(clkd->regmap, clkd->ofs, ctrl_mask, 0);

	mutex_unlock(&clk_det_lock);

	return freq;
}

static unsigned long clk_det_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct clk_det *clkd = to_clk_det(hw);

	if (clkd->ref && !__clk_is_enabled(clkd->ref))
		return 0;

	return clk_det_get_freq(clkd);
}

const struct clk_ops clk_det_ops = {
	.recalc_rate = clk_det_recalc_rate,
};
EXPORT_SYMBOL_GPL(clk_det_ops);

struct clk_det_initdata {
	const char *name;
	struct regmap *regmap;
	uint32_t ofs;
	uint32_t type;
	struct clk *ref;
};

static struct regmap *clk_det_get_regmap(struct device_node *np)
{
	if (of_find_property(np, "syscon", NULL))
		return syscon_regmap_lookup_by_phandle(np, "syscon");

	return device_node_to_regmap(np);
}

static int clk_det_get_initdata(struct device *dev, struct clk_det_initdata *data)
{
	struct device_node *np = dev->of_node;
	int ret;

	data->regmap = clk_det_get_regmap(np);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	data->ref = clk_get(dev, 0);
	if (IS_ERR(data->ref))
		data->ref = NULL;

	ret = of_property_read_string_index(np, "clock-output-names", 0, &data->name);
	if (ret)
		return ret;

	of_property_read_u32(np, "realtek,clk-det-offset", &data->ofs);
	of_property_read_u32(np, "realtek,clk-det-type", &data->type);

	return 0;
}

static int clk_det_create_clk(struct device *dev, struct clk_det_initdata *data)
{
	struct clk_det *clkd;
	struct clk_init_data initdata = { .name = data->name, .ops = &clk_det_ops,
		.flags = CLK_GET_RATE_NOCACHE };
	int ret;

	clkd = devm_kzalloc(dev, sizeof(*clkd), GFP_KERNEL);
	if (!clkd)
		return -ENOMEM;

	clkd->hw.init = &initdata;
	clkd->ofs     = data->ofs;
	clkd->type    = data->type;
	clkd->regmap  = data->regmap;
	clkd->ref     = data->ref;

	ret = devm_clk_hw_register(dev, &clkd->hw);
	if (ret)
		return ret;

	ret = of_clk_add_hw_provider(dev->of_node, of_clk_hw_onecell_get, &clkd->hw);
	if (ret)
		dev_warn(dev, "failed to add clk provider: %d\n", ret);
	return 0;
}


static int clk_det_plat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_det_initdata clkd_initdata = { 0 };
	int ret;

	ret = clk_det_get_initdata(dev, &clkd_initdata);
	if (ret) {
		dev_err(dev, "failed to parse initdata: %d\n", ret);
		return ret;
	}

	ret = clk_det_create_clk(dev, &clkd_initdata);
	if (ret) {
		dev_err(dev, "failed to create clk_det: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id clk_det_match[] = {
	{ .compatible = "realtek,clk-det", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, clk_det_match);

static struct platform_driver clk_det_driver = {
	.probe = clk_det_plat_probe,
	.driver = {
		.name = "rtk-clk-det",
		.of_match_table = of_match_ptr(clk_det_match),
	},
};
module_platform_driver(clk_det_driver);

MODULE_LICENSE("GPL v2");
