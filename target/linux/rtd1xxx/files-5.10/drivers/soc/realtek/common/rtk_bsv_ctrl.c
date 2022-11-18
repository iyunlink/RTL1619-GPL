// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Realtek Semiconductor Corp.
 */

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/nvmem-consumer.h>

#define BSV_CTRL_MAX_OPP       2

struct bsv_opp {
	unsigned int freq_mhz;
	unsigned int volt_uv;
};

struct bsv_config {
	unsigned int volt_max;
	unsigned int volt_min;
	unsigned int volt_step;
	unsigned int volt_round;
	unsigned int volt_correct[BSV_CTRL_MAX_OPP];
	unsigned int num_volt_correct;
};

struct bsv_ctrl_data {
	struct device            *dev;
	struct clk_hw            hw;
	struct clk               *clk;
	struct regulator         *supply;
	unsigned int             cur_volt;
	struct bsv_opp           opps[BSV_CTRL_MAX_OPP];
	int                      num_opps;
	const struct bsv_config  *config;
};

static int bsv_ctrl_should_bypass(struct bsv_ctrl_data *data)
{
	return data->num_opps == 0;
}

static unsigned int bsv_ctrl_volt(struct bsv_ctrl_data *data, int i)
{
	const struct bsv_config *cfg = data->config;

	return data->opps[i].volt_uv + (cfg->num_volt_correct == 1 ? cfg->volt_correct[0] : cfg->volt_correct[i]);
}

static int bsv_ctrl_freq_to_volt(struct bsv_ctrl_data *data, int freq_mhz)
{
	const struct bsv_config *cfg = data->config;
	int i;
	int va, fa, vb, fb;

	for (i = 0; i < data->num_opps; i++)
		if (freq_mhz < data->opps[i].freq_mhz)
			break;

	if (i == data->num_opps)
		return 0;

	if (i == 0)
		return bsv_ctrl_volt(data, i) - (data->opps[i].freq_mhz - freq_mhz) / 100 * cfg->volt_step;

	va = bsv_ctrl_volt(data, i - 1);
	fa = data->opps[i - 1].freq_mhz;
	vb = bsv_ctrl_volt(data, i);
	fb = data->opps[i].freq_mhz;

	return (vb - va) * (freq_mhz - fa) / (fb - fa) + va;
}

static int bsv_ctrl_freq_to_volt_with_limit(struct bsv_ctrl_data *data, int freq_mhz)
{
	const struct bsv_config *cfg = data->config;
	int target_volt = bsv_ctrl_freq_to_volt(data, freq_mhz);

	if (!target_volt)
		return 0;

	if (target_volt < cfg->volt_min)
		return cfg->volt_min;

	if (target_volt > cfg->volt_max)
		return 0;

	return DIV_ROUND_UP(target_volt - cfg->volt_min, cfg->volt_round) * cfg->volt_round
		+ cfg->volt_min;
}

static int bsv_ctrl_clk_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct bsv_ctrl_data *data = container_of(hw, struct bsv_ctrl_data, hw);
	int ret;
	int target_volt;

	if (bsv_ctrl_should_bypass(data))
		return clk_set_rate(data->clk, rate);

	target_volt = bsv_ctrl_freq_to_volt_with_limit(data, rate / 1000000);
	if (!target_volt)
		return -EINVAL;

	if (data->cur_volt < target_volt) {
		ret = regulator_set_voltage(data->supply, target_volt, target_volt);
		if (ret)
			dev_warn(data->dev, "failed to set voltage: %d\n", ret);
	}

	dev_dbg(data->dev, "%s: freq=%lu, volt=%d\n", __func__, rate, target_volt);
	ret = clk_set_rate(data->clk, rate);
	if (ret)
		dev_warn(data->dev, "failed to set frequency: %d\n", ret);

	if (data->cur_volt > target_volt) {
		ret = regulator_set_voltage(data->supply, target_volt, target_volt);
		if (ret)
			dev_warn(data->dev, "failed to set voltage: %d\n", ret);
	}

	data->cur_volt = target_volt;
	return 0;
}

static unsigned long bsv_ctrl_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct bsv_ctrl_data *data = container_of(hw, struct bsv_ctrl_data, hw);

	return clk_get_rate(data->clk);
}

static long bsv_ctrl_clk_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *parent_rate)
{
	struct bsv_ctrl_data *data = container_of(hw, struct bsv_ctrl_data, hw);

	return clk_round_rate(data->clk, rate);
}

static const struct clk_ops bsv_ctrl_clk_ops = {
	.round_rate  = bsv_ctrl_clk_round_rate,
	.recalc_rate = bsv_ctrl_clk_recalc_rate,
	.set_rate    = bsv_ctrl_clk_set_rate,
};

static void bsv_ctrl_remove_of_clk_provider(void *d)
{
	struct bsv_ctrl_data *data = d;

	of_clk_del_provider(data->dev->of_node);
}

static int bsv_ctrl_add_clk(struct bsv_ctrl_data *data)
{
	struct device *dev = data->dev;
	struct device_node *np = dev->of_node;
	struct clk_init_data init_data = {
		.name         = "bsv_vclk",
		.ops          = &bsv_ctrl_clk_ops,
		.num_parents  = 0,
		.flags        = CLK_GET_RATE_NOCACHE,
	};
	int ret;

	data->hw.init = &init_data;
	ret = devm_clk_hw_register(dev, &data->hw);
	if (ret)
		return ret;

	ret = of_clk_add_provider(np, of_clk_src_simple_get, data->hw.clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, bsv_ctrl_remove_of_clk_provider, data);
	if (ret)
		return ret;

	return 0;
}

static int bsv_ctrl_regulator_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	struct bsv_ctrl_data *data = rdev_get_drvdata(rdev);

	return regulator_list_voltage(data->supply, selector);
}

static int bsv_ctrl_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct bsv_ctrl_data *data = rdev_get_drvdata(rdev);

	return regulator_get_voltage(data->supply);
}

static int bsv_ctrl_regulator_set_voltage(struct regulator_dev *rdev,
	int min_uV, int max_uV, unsigned *selector)
{
	struct bsv_ctrl_data *data = rdev_get_drvdata(rdev);

	if (bsv_ctrl_should_bypass(data))
		return regulator_set_voltage(data->supply, min_uV, max_uV);

	dev_dbg(data->dev, "%s: regulator_set_voltage() ignored\n", __func__);
	return 0;
}

static const struct regulator_ops bsv_ctrl_regulator_ops = {
	.get_voltage  = bsv_ctrl_regulator_get_voltage,
	.set_voltage  = bsv_ctrl_regulator_set_voltage,
	.list_voltage = bsv_ctrl_regulator_list_voltage,
};

static struct regulator_desc bsv_ctrl_supply_desc = {
	 .owner          = THIS_MODULE,
	 .ops            = &bsv_ctrl_regulator_ops,
	 .type           = REGULATOR_VOLTAGE,
	 .name           = "bsv_cpudvs",
};

static int bsv_ctrl_add_supply(struct bsv_ctrl_data *data)
{
	struct regulator_config config = {
		.dev         = data->dev,
		.of_node     = data->dev->of_node,
		.driver_data = data,
	};

	bsv_ctrl_supply_desc.n_voltages = regulator_count_voltages(data->supply);

	config.init_data = of_get_regulator_init_data(data->dev, data->dev->of_node,
			&bsv_ctrl_supply_desc);

	if (!config.init_data)
		return -ENOMEM;

	return PTR_ERR_OR_ZERO(devm_regulator_register(data->dev, &bsv_ctrl_supply_desc, &config));
}

static int bsv_ctrl_add_suppliers(struct bsv_ctrl_data *data)
{
	int ret;

	ret = bsv_ctrl_add_clk(data);
	if (ret) {
		dev_err(data->dev, "failed to add clk: %d\n", ret);
		return ret;
	}

	ret = bsv_ctrl_add_supply(data);
	if (ret) {
		dev_err(data->dev, "failed to add supply: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct bsv_config default_config = {
	.volt_max = 1050000,
	.volt_min = 800000,
	.volt_step = 25000,
	.volt_round = 12500,
	.volt_correct = {150000, 175000},
	.num_volt_correct = 2,
};

static int bsv_ctrl_get_otp_val(struct bsv_ctrl_data *data, unsigned int *val)
{
	struct device *dev = data->dev;
	struct nvmem_cell *cell;
	int ret = -EINVAL;
	unsigned char *buf;
	size_t buf_size;

	cell = nvmem_cell_get(dev, "bsv");
	if (IS_ERR(cell)) {
		ret = PTR_ERR(cell);
		dev_warn(dev, "invalid nvmem cell: %d\n", ret);
		return ret;
	}

	buf = nvmem_cell_read(cell, &buf_size);

	val[0] = val[1] = 0;
	if (buf_size == 4) {
		dev_info(dev, "otp_val=%d,%d\n", buf[1], buf[3]);
		val[0] = buf[1];
		val[1] = buf[3];
	}

	kfree(buf);
	nvmem_cell_put(cell);
	return val[0] && val[1] ? 0 : -EINVAL;
}

static int bsv_ctrl_setup_opps(struct bsv_ctrl_data *data)
{
	unsigned int val[2];

	if (bsv_ctrl_get_otp_val(data, val))
		return 0;

	data->opps[0].freq_mhz = 1400;
	data->opps[0].volt_uv = val[0] * 10000;

	data->opps[1].freq_mhz = 1800;
	data->opps[1].volt_uv = val[1] * 10000;

	data->num_opps = 2;

	return 0;
}

static int bsv_ctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bsv_ctrl_data *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->dev = dev;

	data->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(data->clk)) {
		ret = PTR_ERR(data->clk);
		dev_err(dev, "failed to get clk: %d\n", ret);
		return ret;
	}

	data->supply = devm_regulator_get(dev, "cpu");
	if (IS_ERR(data->supply)) {
		ret = PTR_ERR(data->supply);
		dev_err(dev, "failed to get supply: %d\n", ret);
		return ret;
	}

	data->config = &default_config;
	data->cur_volt = regulator_get_voltage(data->supply);
	bsv_ctrl_setup_opps(data);

	return bsv_ctrl_add_suppliers(data);
}

static const struct of_device_id bsv_ctrl_match[] = {
	{ .compatible = "realtek,bsv-controller", },
	{}
};

static struct platform_driver bsv_ctrl_driver = {
	.probe    = bsv_ctrl_probe,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-bsv-ctrl",
		.of_match_table = of_match_ptr(bsv_ctrl_match),
	},
};
module_platform_driver(bsv_ctrl_driver);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:rtk-bsv-ctrl");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_DESCRIPTION("Realtek BSV Supply");
