// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Realtek Semiconductor Corp.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

struct dco_cal_data {
	struct regmap *regmap;
	struct device *dev;
	spinlock_t lock;
};

#define ISO_DCO_0        0x79C
#define ISO_DCO_1        0x0F4
#define ISO_PLL_ETN_OSC  0x7A4

static void dco_set_cal_en(struct dco_cal_data *data, uint32_t val)
{
	regmap_update_bits(data->regmap, ISO_DCO_0, 0x1, val);
}

static void dco_set_dco_sel(struct dco_cal_data *data, uint32_t val)
{
	regmap_update_bits(data->regmap, ISO_DCO_0, 0x2, val << 1);
}

static void dco_set_xtal_off(struct dco_cal_data *data, uint32_t val)
{
	regmap_update_bits(data->regmap, ISO_DCO_0, 0x4, val << 2);
}

static void dco_set_osc_count_limit(struct dco_cal_data *data, uint32_t val)
{
	regmap_update_bits(data->regmap, ISO_DCO_0, 0x3ffc00, val << 10);
}

static void dco_clear_cal_data(struct dco_cal_data *data)
{
	regmap_write(data->regmap, ISO_DCO_1, 0x00000000);
}

static uint32_t dco_get_cal_done(struct dco_cal_data *data)
{
	uint32_t val;

	regmap_read(data->regmap, ISO_DCO_1, &val);
	return val & 0x1;
}

static uint32_t dco_get_dco_count_latch(struct dco_cal_data *data)
{
	uint32_t val;

	regmap_read(data->regmap, ISO_DCO_1, &val);
	return (val & 0x1ffe000) >> 13;
}

static void dco_set_osc_rstb(struct dco_cal_data *data)
{
	regmap_update_bits(data->regmap, ISO_PLL_ETN_OSC, 1, 1);
}

static uint32_t dco_get_osc_sig(struct dco_cal_data *data)
{
	uint32_t val;

	regmap_read(data->regmap, ISO_PLL_ETN_OSC, &val);
	return (val >> 1) & 0x7f;
}

static void dco_set_osc_sig(struct dco_cal_data *data, uint32_t val)
{
	regmap_update_bits(data->regmap, ISO_PLL_ETN_OSC, 0xfe, val << 1);
}

static int __calibrate_dco(struct dco_cal_data *data, uint32_t *dco_count)
{
	dco_set_dco_sel(data, 0);
	dco_set_xtal_off(data, 0);
	dco_set_osc_count_limit(data, 0xc00);
	dco_clear_cal_data(data);

	dco_set_cal_en(data, 0);
	dco_set_cal_en(data, 1);

	udelay(1000);

	if (!dco_get_cal_done(data))
		return -EBUSY;

	*dco_count = dco_get_dco_count_latch(data);

	return 0;
}

static int dco_calibrate(struct dco_cal_data *data, uint32_t sig, uint32_t *dco_count)
{
	int retry_left = 30;
	uint32_t v;

	dco_set_osc_sig(data, sig);

	while (retry_left-- > 0 && __calibrate_dco(data, &v))
		dco_set_osc_rstb(data);

	dev_dbg(data->dev, "%s: sig=%02x, dco_count=%03x\n", __func__, sig, v);

	if (dco_count)
		*dco_count = v;

	return retry_left < 0 ? -ETIMEDOUT : 0;
}

static int evaluate_sig_delta(int delta)
{
	int ret;
	int neg = 0;

	if (delta < 0) {
		neg = 1;
		delta = -delta;
	}

	if (delta > 0x180)
		ret = 8;
	else if (delta > 0xc0)
		ret = 4;
	else if (delta > 0x60)
		ret = 2;
	else if (delta > 0x10)
		ret = 1;
	else
		ret = 0;
	return neg ? -ret : ret;
}

static int __dco_recalibrate(struct dco_cal_data *data)
{
	uint32_t dco_count_target = 0xc00, dco_count = 0;
	int retry_left = 30;
	int ret;
	uint32_t sig = 0x40;
	int delta = 0;

	do {
		ret = dco_calibrate(data, sig, &dco_count);
		if (ret)
			return ret;

		delta = evaluate_sig_delta(dco_count_target - dco_count);
		if (delta == 0)
			break;

		sig += delta;

	} while (retry_left-- > 0);

	if (retry_left < 0) {
		dco_calibrate(data, 0x40, NULL);
		return -EINVAL;
	}

	return 0;
}

int dco_recalibrate(struct dco_cal_data *data)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	ret = __dco_recalibrate(data);
	spin_unlock_irqrestore(&data->lock, flags);

	dev_dbg(data->dev, "sig=%02x\n", dco_get_osc_sig(data));

	return ret;
}
EXPORT_SYMBOL_GPL(dco_recalibrate);

static int dco_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *parent = of_get_parent(np);
	struct dco_cal_data *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->dev = dev;
	spin_lock_init(&data->lock);

	data->regmap = syscon_node_to_regmap(parent);
	of_node_put(parent);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(dev, "failed to get syscon regmap\n");
	}

	platform_set_drvdata(pdev, data);
	return 0;
}

static int dco_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id dco_match[] = {
	{ .compatible = "realtek,dco" },
	{}
};
MODULE_DEVICE_TABLE(of, dco_match);

static struct platform_driver dco_driver = {
	.probe  = dco_probe,
	.remove = dco_remove,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-dco",
		.of_match_table = of_match_ptr(dco_match),
	},
};
module_platform_driver(dco_driver);

/**
 * of_dco_get - lookup and obtain a dco device.
 * @np: device node to request
 * @index: index of the dco
 *
 * Returns the dco device on success, and returns -EINVAL if failed to
 * get the dco, and returns -EPROBE_DERFER if dco device is not ready.
 */
struct dco_cal_data *of_dco_get(struct device_node *np, int index)
{
	struct dco_cal_data *data = ERR_PTR(-EINVAL);
	struct device_node *dco_np;
	struct device *dev;

	dco_np = of_parse_phandle(np, "realtek,dco", index);

	if (!dco_np)
		return data;

	dev = driver_find_device_by_of_node(&dco_driver.driver, dco_np);
	of_node_put(dco_np);
	if (!dev)
		return data;

	data = dev_get_drvdata(dev);
	if (!data)
		return ERR_PTR(-EPROBE_DEFER);

	get_device(dev);
	return data;
}
EXPORT_SYMBOL_GPL(of_dco_get);

/**
 * dco_put - free the dco device
 * @dco: dco device
 */
void dco_put(struct dco_cal_data *data)
{
	if (data)
		put_device(data->dev);
}
EXPORT_SYMBOL_GPL(dco_put);

MODULE_DESCRIPTION("Realtek DCO driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-dco");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
