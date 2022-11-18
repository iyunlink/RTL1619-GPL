// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Realtek Semiconductor Corporation
 * Author: Cheng-Chih Tsai <wesley_tsai@realtek.com>
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include "dpi.h"

#define DPI_DLL_CRT_CKE_OCD_SEL		0x1c0
#define DPI_DLL_CRT_DQ_OCD_SEL		0x1d8
#define DPI_DLL_CRT_DQS_OCD_SEL_0	0x1e4
#define DPI_DLL_CRT_DQS_OCD_SEL_1	0x1e8
#define DPI_DLL_CRT_CS_OCD_SEL		0x1ec
#define DPI_DLL_CRT_CK_OCD_SEL		0x1f0
#define DPI_DLL_CRT_OCD_SEL_0		0x3c4
#define DPI_DLL_CRT_OCD_SEL_1		0x3c8
#define DPI_DLL_CRT_OCD_SEL_2		0x3cc
#define DPI_DLL_CRT_OCD_SEL_3		0x3d0
#define DPI_DLL_CRT_OCD_SEL_4		0x3d4
#define DPI_DLL_CRT_OCD_SEL_5		0x3d8
#define DPI_DLL_CRT_OCD_SEL_6		0x3dc
#define DPI_DLL_CRT_OCD_SEL_7		0x3e0
#define DPI_DLL_CRT_OCD_SEL_8		0x3e4

#define DPI_RT_OCD_SEL_STATE_DEFAULT  1
#define DPI_RT_OCD_SEL_STATE_1  2

#define DPI_CK_OCD_IDX		0
#define DPI_DQS_OCD_IDX		1
#define DPI_DQ_OCD_IDX  	2
#define DPI_CA_OCD_IDX  	3

static const uint32_t reg_ofs_tbl[] = {
	DPI_DLL_CRT_CK_OCD_SEL,
	DPI_DLL_CRT_DQS_OCD_SEL_0,
	DPI_DLL_CRT_DQS_OCD_SEL_1,
	DPI_DLL_CRT_DQ_OCD_SEL,
	DPI_DLL_CRT_CS_OCD_SEL,
	DPI_DLL_CRT_CKE_OCD_SEL,
	DPI_DLL_CRT_OCD_SEL_0,
	DPI_DLL_CRT_OCD_SEL_1,
	DPI_DLL_CRT_OCD_SEL_2,
	DPI_DLL_CRT_OCD_SEL_3,
	DPI_DLL_CRT_OCD_SEL_4,
	DPI_DLL_CRT_OCD_SEL_5,
	DPI_DLL_CRT_OCD_SEL_6,
	DPI_DLL_CRT_OCD_SEL_7,
	DPI_DLL_CRT_OCD_SEL_8,
};

struct dpi_tx_ocd_sel_priv {
	struct device *dev;
	struct dpi_device *dpi;
	struct notifier_block nb;
	int state;
	enum dpi_dram_type type;
	uint32_t *ddr_type_q;
	uint32_t *ocd_idx_q;
	uint32_t *ocd_val_q_0;
	uint32_t *ocd_val_q_1;
	uint32_t num_type;
	uint32_t num_idx;
	uint32_t num_val_0;
	uint32_t num_val_1;
};

static void dpi_tx_ocd_sel_update_state(struct dpi_tx_ocd_sel_priv *priv, int state)
{
	int i, j;
	uint32_t *val= state == DPI_RT_OCD_SEL_STATE_1 ? priv->ocd_val_q_1 : priv->ocd_val_q_0;

	for (i = 0; i < priv->num_idx; i++) {
		if(priv->ddr_type_q[i] == priv->type) {
			switch (priv->ocd_idx_q[i]) {
				case DPI_CK_OCD_IDX:
					dpi_reg_write(priv->dpi, reg_ofs_tbl[0], val[i]);
					break;
				case DPI_DQS_OCD_IDX:
					dpi_reg_write(priv->dpi, reg_ofs_tbl[1], val[i]);
					dpi_reg_write(priv->dpi, reg_ofs_tbl[2], val[i]);
					break;
				case DPI_DQ_OCD_IDX:
					dpi_reg_write(priv->dpi, reg_ofs_tbl[3], val[i]);
					break;
				case DPI_CA_OCD_IDX:
					for (j = 4; j < 15; ++j)
						dpi_reg_write(priv->dpi, reg_ofs_tbl[j], val[i]);
					break;
				default:
					dpi_reg_write(priv->dpi, reg_ofs_tbl[priv->ocd_idx_q[i]-4], val[i]);
			}

			pr_info("%s: idx=%x, val=%08x\n", __func__, priv->ocd_idx_q[i], val[i]);
		}
	}
}

static int dpi_tx_ocd_sel_cb(struct notifier_block *nb, unsigned long event,
			   void *data)
{
	struct dpi_tx_ocd_sel_priv *priv = container_of(nb,	struct dpi_tx_ocd_sel_priv, nb);
	int new_state ;

	switch (event) {
	case DPI_EVENT_TEMP_STATUS_HOT:
	case DPI_EVENT_TEMP_STATUS_COLD:
		new_state = DPI_RT_OCD_SEL_STATE_1;
		break;
	case DPI_EVENT_TEMP_STATUS_NORMAL:
		new_state = DPI_RT_OCD_SEL_STATE_DEFAULT;
		break;
	default:
		return NOTIFY_DONE;
	}

	if (new_state != priv->state)
		dpi_tx_ocd_sel_update_state(priv, new_state);
	priv->state = new_state;
	return NOTIFY_OK;
}

static int dpi_tx_ocd_sel_read_reg_values(struct device *dev,
					  const char *name,
					  uint32_t **val, uint32_t *num)

{
	struct device_node *np = dev->of_node;
	int len;

	if (!of_find_property(np, name, &len))
		return -EINVAL;

	len /= 4;

	*val = devm_kcalloc(dev, len, sizeof(*val), GFP_KERNEL);
	if(!(*val)) {
		dev_err(dev, "%s %s(): memory allocation failure\n", __FILE__, __func__);
		return -ENOMEM;
	}

	*num = len;

	return of_property_read_u32_array(np, name, *val, len);
}

static int dpi_tx_ocd_sel_of_init(struct device *dev,
					struct dpi_tx_ocd_sel_priv *priv)
{
	int ret;

	if ((ret = dpi_tx_ocd_sel_read_reg_values(dev, "ddr-type", &priv->ddr_type_q, &priv->num_type)))
		return ret;

	if ((ret = dpi_tx_ocd_sel_read_reg_values(dev, "ocd-sel-idx", &priv->ocd_idx_q, &priv->num_idx)))
		return ret;

	if ((ret = dpi_tx_ocd_sel_read_reg_values(dev, "state-default-value", &priv->ocd_val_q_0, &priv->num_val_0)))
		return ret;

	ret = dpi_tx_ocd_sel_read_reg_values(dev, "state-1-value", &priv->ocd_val_q_1, &priv->num_val_1);

	return ret;
}

static int dpi_tx_ocd_sel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dpi_device *dpi = dev_get_drvdata(dev->parent);
	struct dpi_tx_ocd_sel_priv *priv;
	int ret;

	if (!dpi)
		return -EPROBE_DEFER;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->type = dpi_get_dram_type(dpi);

	ret = dpi_tx_ocd_sel_of_init(dev, priv);
	if (ret) {
		dev_err(dev, "failed to parse data from dt: %d\n", ret);
		return ret;
	}

	priv->dpi = dpi;
	priv->dev = dev;
	priv->nb.notifier_call = dpi_tx_ocd_sel_cb;

	return dpi_register_notifier(dpi, &priv->nb);
}

static const struct of_device_id dpi_tx_ocd_sel_of_match[] = {
	{ .compatible = "realtek,dpi-tx-ocd-sel" },
	{}
};

static struct platform_driver dpi_tx_ocd_sel_driver = {
	.driver = {
		.name           = "rtk-dpi-tx-ocd-sel",
		.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(dpi_tx_ocd_sel_of_match),
	},
	.probe    = dpi_tx_ocd_sel_probe,
};
module_platform_driver(dpi_tx_ocd_sel_driver);

MODULE_LICENSE("GPL v2");
