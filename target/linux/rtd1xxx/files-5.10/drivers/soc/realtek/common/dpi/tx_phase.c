// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include "dpi.h"

#define DPI_DLL_CRT_PLL_PI0 0x10
#define DPI_DLL_CRT_PLL_PI1 0x14
#define DPI_DLL_CRT_PLL_PI2 0x98
#define DPI_DLL_CRT_PLL_PI3 0xA0
#define DPI_DLL_CRT_SSC2    0x24
#define DPI_DLL_CRT_SSC3    0x28

#define DPI_TPT_STATE_0  1
#define DPI_TPT_STATE_1  2

struct dpi_tx_phase_priv {
	struct device *dev;
	struct dpi_device *dpi;
	struct notifier_block nb;
	int state;
	int ck_phase_default_ofs;
	int ck_phase_ofs;
};

struct dpi_tx_phase_data {
	int ofs;
	int sft;
};

#define DPI_PHASE(_ofs, _sft) { .ofs = _ofs, .sft = _sft, }

static struct dpi_tx_phase_data dpi_tx_phase[]= {
	DPI_PHASE(DPI_DLL_CRT_PLL_PI0,  0),		// CK0
	DPI_PHASE(DPI_DLL_CRT_PLL_PI3, 24),		// CK1
	DPI_PHASE(DPI_DLL_CRT_PLL_PI2, 24),		// CS0
	DPI_PHASE(DPI_DLL_CRT_PLL_PI3,  0),		// CS1
	DPI_PHASE(DPI_DLL_CRT_PLL_PI3,  8),		// CS2
	DPI_PHASE(DPI_DLL_CRT_PLL_PI3, 16),		// CS3
	DPI_PHASE(DPI_DLL_CRT_PLL_PI0, 16),		// DQS0
	DPI_PHASE(DPI_DLL_CRT_PLL_PI0, 24),		// DQS1
	DPI_PHASE(DPI_DLL_CRT_PLL_PI1,  0),		// DQS2
	DPI_PHASE(DPI_DLL_CRT_PLL_PI1,  8),		// DQS3
	DPI_PHASE(DPI_DLL_CRT_PLL_PI1, 16),		// DQ0
	DPI_PHASE(DPI_DLL_CRT_PLL_PI2,  0),		// DQ1
	DPI_PHASE(DPI_DLL_CRT_PLL_PI2,  8),		// DQ2
	DPI_PHASE(DPI_DLL_CRT_PLL_PI2, 16),		// DQ3
};

static void dpi_tx_phase_tune_one(struct dpi_tx_phase_priv *priv,
				  struct dpi_tx_phase_data *p, int v)
{
	uint32_t val_ori, val;

	dpi_reg_read(priv->dpi, p->ofs, &val_ori);
	val = (val_ori >> p->sft) & 0x1f;
	val = (val + v) & 0x1f;
	val_ori = (val_ori & ~((0x1f << p->sft))) | (val << p->sft);
	dpi_reg_write(priv->dpi, p->ofs, val_ori);
}

static void dpi_tx_phase_tune(struct dpi_tx_phase_priv *priv, int ofs, int val)
{
	int i, j;

	dpi_clock_gating_disable(priv->dpi);

	for (i = 0; i < ofs; i++)
		for (j = 0; j < ARRAY_SIZE(dpi_tx_phase); j++)
			dpi_tx_phase_tune_one(priv, &dpi_tx_phase[j], val);

	dpi_clock_gating_enable(priv->dpi);

	pr_info("%s: CRT_PLL_PI0 = %08x\n", __func__, readl(priv->dpi->base + DPI_DLL_CRT_PLL_PI0));
	pr_info("%s: CRT_PLL_PI1 = %08x\n", __func__, readl(priv->dpi->base + DPI_DLL_CRT_PLL_PI1));
	pr_info("%s: CRT_PLL_PI2 = %08x\n", __func__, readl(priv->dpi->base + DPI_DLL_CRT_PLL_PI2));
	pr_info("%s: CRT_PLL_PI3 = %08x\n", __func__, readl(priv->dpi->base + DPI_DLL_CRT_PLL_PI3));
}

static int dpi_tx_phase_cb(struct notifier_block *nb, unsigned long event,
			   void *data)
{
	struct dpi_tx_phase_priv *priv = container_of(nb, struct dpi_tx_phase_priv, nb);
	int new_state, ofs, dir, val;

	switch (event) {
	case DPI_EVENT_TEMP_STATUS_HOT:
	case DPI_EVENT_TEMP_STATUS_COLD:
		new_state = DPI_TPT_STATE_1;
		break;
	case DPI_EVENT_TEMP_STATUS_NORMAL:
		new_state = DPI_TPT_STATE_0;
		break;
	default:
		return NOTIFY_DONE;
	}

	// pr_info("%s: event = %d, priv->state = %d, new_state = %d\n", __func__, event, priv->state, new_state);

	if (new_state != priv->state) {
		ofs = (priv->ck_phase_ofs < 0) ? -priv->ck_phase_ofs : priv->ck_phase_ofs;
		dir = (priv->ck_phase_ofs < 0) ? -1 : 1;
		val = (new_state == DPI_TPT_STATE_1) ? dir : -dir;
		pr_info("%s: state = %d, val= %d, ofs = %d\n", __func__, new_state, val, ofs);

		dpi_tx_phase_tune(priv, ofs, val);
	}
	priv->state = new_state;
	return NOTIFY_OK;
}

static int of_dpi_tx_phase_parse_data(struct device_node *np, struct dpi_tx_phase_priv *priv)
{
	int len, ret;
	const char *name[2] = {"ck-phase-default-ofs", "ck-phase-ofs"};

	if (!of_find_property(np, name[0], &len))
		return -EINVAL;

	if (len > sizeof(priv->ck_phase_default_ofs))
		return -EINVAL;

	len /= 4;

	if((ret = of_property_read_u32_array(np, name[0], &priv->ck_phase_default_ofs, len)))
		return ret;

	if (!of_find_property(np, name[1], &len))
		return -EINVAL;

	if (len > sizeof(priv->ck_phase_ofs))
		return -EINVAL;

	len /= 4;

	if((ret = of_property_read_u32_array(np, name[1], &priv->ck_phase_ofs, len)))
		return ret;

	priv->state = DPI_TPT_STATE_0;

	return ret;
}

static int dpi_tx_phase_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dpi_device *dpi = dev_get_drvdata(dev->parent);
	struct dpi_tx_phase_priv *priv;
	int ret, ofs, val;

	if (!dpi)
		return -EPROBE_DEFER;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = of_dpi_tx_phase_parse_data(dev->of_node, priv);
	if (ret) {
		dev_err(dev, "failed to parse data from dt: %d\n", ret);
		return ret;
	}

	priv->dpi = dpi;
	priv->dev = dev;
	priv->nb.notifier_call = dpi_tx_phase_cb;

	ofs = (priv->ck_phase_default_ofs < 0) ? -priv->ck_phase_default_ofs : priv->ck_phase_default_ofs;
	val = (priv->ck_phase_default_ofs < 0) ? -1 : 1;
	pr_info("%s: state = %d, val= %d, ofs = %d\n", __func__, priv->state, val, ofs);

	dpi_tx_phase_tune(priv, ofs, val);

	return dpi_register_notifier(dpi, &priv->nb);
}

static const struct of_device_id dpi_tx_phase_of_match[] = {
	{ .compatible = "realtek,dpi-tx-phase" },
	{}
};

static struct platform_driver dpi_tx_phase_driver = {
	.driver = {
		.name           = "rtk-dpi-tx-phase",
		.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(dpi_tx_phase_of_match),
	},
	.probe    = dpi_tx_phase_probe,
};
module_platform_driver(dpi_tx_phase_driver);

MODULE_LICENSE("GPL v2");
