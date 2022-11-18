// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define SCW_CTRL           0x000
#define SCW_CTRL1          0x024
#define SC_ACP_CRT_CTRL    0x030
#define SC_CRT_CTRL        0x100
#define SC_DUMMY_2         0x608

struct sc_wrap_data;

struct sc_wrap_ops {
	void (*setup_icg)(struct sc_wrap_data *data);
	void (*setup_pmu)(struct sc_wrap_data *data);
};

struct sc_wrap_data {
	struct device *dev;
	void *base;
	const struct sc_wrap_ops *ops;
};

static void rtd1319_sc_wrap_setup_icg(struct sc_wrap_data *data)
{
	uint32_t val;

	val = readl(data->base + SCW_CTRL);
	val |= 0xff3c0000;
	writel(val, data->base + SCW_CTRL);

	val = readl(data->base + SC_ACP_CRT_CTRL);
	val |= 0x10000000;
	writel(val, data->base + SC_ACP_CRT_CTRL);

	val = readl(data->base + SC_CRT_CTRL);
	val |= 0xc0000000;
	writel(val, data->base + SC_CRT_CTRL);
}

static const struct sc_wrap_ops rtd1319_sc_wrap_ops = {
	.setup_icg = rtd1319_sc_wrap_setup_icg,
};

static void rtd1619b_sc_wrap_setup_icg(struct sc_wrap_data *data)
{
	uint32_t val;

	val = readl(data->base + SCW_CTRL);
	val |= 0xff3c0000;
	writel(val, data->base + SCW_CTRL);

	val = readl(data->base + SCW_CTRL1);
	val |= 0x00000101;
	writel(val, data->base + SCW_CTRL1);

	val = readl(data->base + SC_ACP_CRT_CTRL);
	val |= 0x10000000;
	writel(val, data->base + SC_ACP_CRT_CTRL);

	val = readl(data->base + SC_CRT_CTRL);
	val |= 0xc0000000;
	writel(val, data->base + SC_CRT_CTRL);
}

static const struct sc_wrap_ops rtd1619b_sc_wrap_ops = {
	.setup_icg = rtd1619b_sc_wrap_setup_icg,
};

static void sc_wrap_init(struct sc_wrap_data *data)
{
	if (!data->ops)
		return;

	if (data->ops->setup_icg)
		data->ops->setup_icg(data);

	if (data->ops->setup_pmu)
		data->ops->setup_pmu(data);
}

static int sc_wrap_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct sc_wrap_data *data;
	struct resource res;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->ops = of_device_get_match_data(dev);

	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(dev, "failed to get recource: %d\n", ret);
		return -ENODEV;
	}

	data->base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!data->base)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	sc_wrap_init(data);

	return 0;
}

static const struct of_device_id sc_wrap_ids[] = {
	{ .compatible = "realtek,rtd1319-scpu-wrapper", .data = &rtd1319_sc_wrap_ops, },
	{ .compatible = "realtek,rtd1619b-scpu-wrapper", .data = &rtd1619b_sc_wrap_ops, },
	{}
};

static struct platform_driver sc_wrap_driver = {
	.probe  = sc_wrap_probe,
	.driver = {
		.name           = "rtk-sc-wrap",
		.owner          = THIS_MODULE,
		.of_match_table = sc_wrap_ids,
	},
};
module_platform_driver(sc_wrap_driver);

MODULE_DESCRIPTION("Reatek SCPU Wrapper Driver");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");

