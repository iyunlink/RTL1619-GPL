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

#define DPI_DLL_CRT_DQ_ODT_SEL    0x1d4
#define DPI_DLL_CRT_DQS_P_ODT_SEL 0x1dc
#define DPI_DLL_CRT_DQS_N_ODT_SEL 0x1e0
#define DPI_DLL_CRT_CLK_ODT_SEL   0x418
#define DPI_DLL_CRT_ADR_ODT_SEL   0x474
#define DPI_DLL_CRT_CKE_ODT_SEL   0x478
#define DPI_DLL_CRT_CS_ODT_SEL    0x47c

#define DPI_RT_ODT_SEL_STATE_DEFAULT  1
#define DPI_RT_ODT_SEL_STATE_1  2

static const uint32_t reg_ofs_tbl[] = {
	DPI_DLL_CRT_CLK_ODT_SEL,
	DPI_DLL_CRT_ADR_ODT_SEL,
	DPI_DLL_CRT_CKE_ODT_SEL,
	DPI_DLL_CRT_CS_ODT_SEL,
	DPI_DLL_CRT_DQ_ODT_SEL,
	DPI_DLL_CRT_DQS_P_ODT_SEL,
	DPI_DLL_CRT_DQS_N_ODT_SEL,
};

struct dpi_rx_odt_sel_priv {
	struct device *dev;
	struct dpi_device *dpi;
	struct notifier_block nb;
	int state;
	enum dpi_dram_type type;
	uint32_t *ddr_type_q;
	uint32_t *odt_idx_q;
	uint32_t *odt_val_q_0;
	uint32_t *odt_val_q_1;
	uint32_t num_type;
	uint32_t num_idx;
	uint32_t num_val_0;
	uint32_t num_val_1;
	// uint32_t reg_val_0[ARRAY_SIZE(reg_ofs_tbl)];
	// uint32_t reg_val_1[ARRAY_SIZE(reg_ofs_tbl)];
};

static void dpi_rx_odt_sel_update_state(struct dpi_rx_odt_sel_priv *priv, int state)
{
	int i;
	uint32_t *val= state == DPI_RT_ODT_SEL_STATE_1 ? priv->odt_val_q_1 : priv->odt_val_q_0;

	for (i = 0; i < priv->num_idx; i++) {
		if(priv->ddr_type_q[i] == priv->type) {
			dpi_reg_write(priv->dpi, reg_ofs_tbl[priv->odt_idx_q[i]], val[i]);
			pr_info("%s: ofs=%03x, val=%08x\n", __func__, reg_ofs_tbl[priv->odt_idx_q[i]], val[i]);
		}
	}
}

static int dpi_rx_odt_sel_cb(struct notifier_block *nb, unsigned long event,
			   void *data)
{
	struct dpi_rx_odt_sel_priv *priv = container_of(nb,
		struct dpi_rx_odt_sel_priv, nb);
	int new_state ;

	switch (event) {
	case DPI_EVENT_TEMP_STATUS_HOT:
	case DPI_EVENT_TEMP_STATUS_COLD:
		new_state = DPI_RT_ODT_SEL_STATE_1;
		break;
	case DPI_EVENT_TEMP_STATUS_NORMAL:
		new_state = DPI_RT_ODT_SEL_STATE_DEFAULT;
		break;
	default:
		return NOTIFY_DONE;
	}

	if (new_state != priv->state)
		dpi_rx_odt_sel_update_state(priv, new_state);
	priv->state = new_state;
	return NOTIFY_OK;
}

static int dpi_rx_odt_sel_read_reg_values(struct device *dev,
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
		dev_err(dev, "%s %s(): memory allocation failure\n",
		       __FILE__, __func__);
		return -ENOMEM;
	}

	*num = len;

	return of_property_read_u32_array(np, name, *val, len);
}

static int dpi_rx_odt_sel_of_init(struct device *dev,
					struct dpi_rx_odt_sel_priv *priv)
{
	int ret;

	if ((ret = dpi_rx_odt_sel_read_reg_values(dev, "ddr-type", &priv->ddr_type_q, &priv->num_type)))
		return ret;

	if ((ret = dpi_rx_odt_sel_read_reg_values(dev, "odt-sel-idx", &priv->odt_idx_q, &priv->num_idx)))
		return ret;

	if ((ret = dpi_rx_odt_sel_read_reg_values(dev, "state-default-value", &priv->odt_val_q_0, &priv->num_val_0)))
		return ret;

	ret = dpi_rx_odt_sel_read_reg_values(dev, "state-1-value", &priv->odt_val_q_1, &priv->num_val_1);

	return ret;
}

static int dpi_rx_odt_sel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dpi_device *dpi = dev_get_drvdata(dev->parent);
	struct dpi_rx_odt_sel_priv *priv;
	int ret;

	if (!dpi)
		return -EPROBE_DEFER;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->type = dpi_get_dram_type(dpi);

	ret = dpi_rx_odt_sel_of_init(dev, priv);
	if (ret) {
		dev_err(dev, "failed to parse data from dt: %d\n", ret);
		return ret;
	}

	priv->dpi = dpi;
	priv->dev = dev;
	priv->nb.notifier_call = dpi_rx_odt_sel_cb;

	return dpi_register_notifier(dpi, &priv->nb);
}

static const struct of_device_id dpi_rx_odt_sel_of_match[] = {
	{ .compatible = "realtek,dpi-rx-odt-sel" },
	{}
};

static struct platform_driver dpi_rx_odt_sel_driver = {
	.driver = {
		.name           = "rtk-dpi-rx-odt-sel",
		.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(dpi_rx_odt_sel_of_match),
	},
	.probe    = dpi_rx_odt_sel_probe,
};
module_platform_driver(dpi_rx_odt_sel_driver);

MODULE_LICENSE("GPL v2");
