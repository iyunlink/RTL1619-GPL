// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019-2020 Realtek Semiconductor Corporatio
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <dt-bindings/reset/rtd1319-reset.h>

#define M2TMX_VE_AXI_ARB_CTRL_0   0x0B0

struct m2tmx_reset_controller_dev {
	struct device *dev;
	struct reset_controller_dev rcdev;
	spinlock_t lock;
	struct regmap *regmap;
	struct clk *clk[RTD1319_M2TMX_RSTN_MAX];
};

static
int rtk_m2tmx_reset_update_bits(struct m2tmx_reset_controller_dev *m2rcdev,
				unsigned int offset, unsigned int mask,
				unsigned int val)
{
	return regmap_update_bits(m2rcdev->regmap, offset, mask, val);
}

static int rtk_m2tmx_reset_reset(struct reset_controller_dev *rcdev,
				 unsigned long idx)
{
	struct m2tmx_reset_controller_dev *m2rcdev = container_of(rcdev,
		struct m2tmx_reset_controller_dev, rcdev);
	unsigned int val = BIT(8 + idx * 2);
	unsigned int en  = BIT(8 + idx * 2 + 1);
	unsigned int mask = val | en;
	unsigned long flags;

	dev_dbg(m2rcdev->dev, "reset idx=%lu, en_mask=%x, val_mask=%x\n", idx,
		en, val);

	clk_prepare_enable(m2rcdev->clk[idx]);

	spin_lock_irqsave(&m2rcdev->lock, flags);

	rtk_m2tmx_reset_update_bits(m2rcdev, M2TMX_VE_AXI_ARB_CTRL_0, mask, en);
	udelay(10);
	rtk_m2tmx_reset_update_bits(m2rcdev, M2TMX_VE_AXI_ARB_CTRL_0, mask, val);

	spin_unlock_irqrestore(&m2rcdev->lock, flags);

	clk_disable_unprepare(m2rcdev->clk[idx]);
	return 0;
}

static const struct reset_control_ops rtk_m2tmx_reset_ops = {
	.reset = rtk_m2tmx_reset_reset,
};

static int rtk_m2tmx_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct m2tmx_reset_controller_dev *m2rcdev;
	int ret;
	int i;
	const char *name;

	m2rcdev = devm_kzalloc(dev, sizeof(*m2rcdev), GFP_KERNEL);
	if (!m2rcdev)
		return -EINVAL;

	m2rcdev->regmap = syscon_regmap_lookup_by_phandle(dev->of_node,
							  "realtek,m2tmx");
	if (IS_ERR(m2rcdev->regmap)) {
		ret = PTR_ERR(m2rcdev->regmap);
		dev_err(dev, "failed to get syscon: %d\n", ret);
		return ret;
	}

	for (i = 0; i < RTD1319_M2TMX_RSTN_MAX; i++) {
		name = NULL;
		of_property_read_string_index(np, "clock-names", i, &name);
		m2rcdev->clk[i] = devm_clk_get(dev, name);
		if (IS_ERR(m2rcdev->clk[i])) {
			dev_warn(dev, "invalid clocks at index %d: %ld\n", i, PTR_ERR(m2rcdev->clk[i]));
			m2rcdev->clk[i] = NULL;
		}
	}

	m2rcdev->dev = dev;

	spin_lock_init(&m2rcdev->lock);
	m2rcdev->rcdev.owner     = THIS_MODULE;
	m2rcdev->rcdev.nr_resets = RTD1319_M2TMX_RSTN_MAX;
	m2rcdev->rcdev.ops       = &rtk_m2tmx_reset_ops;
	m2rcdev->rcdev.of_node   = dev->of_node;
	return devm_reset_controller_register(dev, &m2rcdev->rcdev);
}


static const struct of_device_id rtk_m2tmx_reset_dt_ids[] = {
	{ .compatible = "realtek,rtd1319-m2tmx-reset", },
	{ /* sentinel */ }
};

static struct platform_driver rtk_m2tmx_reset_driver = {
	.probe	= rtk_m2tmx_reset_probe,
	.driver = {
		.name = "rtk-m2tmx-reset",
		.of_match_table	= rtk_m2tmx_reset_dt_ids,
	},
};
static int __init rtk_m2tmx_reset_init(void)
{
	return platform_driver_register(&rtk_m2tmx_reset_driver);
}
arch_initcall(rtk_m2tmx_reset_init);

MODULE_DESCRIPTION("Realtek M2TMX Reset Driver");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");
