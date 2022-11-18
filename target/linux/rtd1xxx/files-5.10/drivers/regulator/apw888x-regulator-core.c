// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/module.h>
#include <linux/list.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/mfd/apw888x.h>
#include <dt-bindings/regulator/anpec,apw888x.h>
#include "apw888x-regulator.h"


#define APW888X_SLEEP_MODE_NRM_VOLT   0x0
#define APW888X_SLEEP_MODE_SLP_VOLT   0x2
#define APW888X_SLEEP_MODE_SHUTDOWN   0x3
static int apw888x_set_sleep_mode(struct regulator_dev *rdev, int mode);

static int
apw888x_regulator_of_parse_data(struct device_node *np,
				const struct regulator_desc *desc,
				struct apw888x_regulator_data *data)
{
	unsigned int val;
	struct device_node *child;
	struct regulator_state *rstate = &data->state_shutdown;

	child = of_get_child_by_name(np, "regulator-state-shutdown");
	if (!child)
		child = of_get_child_by_name(np, "regulator-state-mem");
	if (!child) {
		rstate->enabled = ENABLE_IN_SUSPEND;
	} else {
		if (of_property_read_bool(child, "regulator-on-in-suspend"))
			rstate->enabled = ENABLE_IN_SUSPEND;

		if (of_property_read_bool(child, "regulator-off-in-suspend"))
			rstate->enabled = DISABLE_IN_SUSPEND;

		if (!of_property_read_u32(child, "regulator-suspend-microvolt", &val))
			rstate->uV = val;

		of_node_put(child);
	}

	if (desc->n_voltages == 1 && desc->fixed_uV == 0) {
		u32 min = 0, max = 0;

		of_property_read_u32(np, "regulator-min-microvolt", &min);
		of_property_read_u32(np, "regulator-max-microvolt", &max);
		WARN_ON(min != max);
		data->fixed_uV = max;
	}

	data->dischg_dis = of_property_read_bool(np, "apw888x-discharge-disable");
	data->use_nrm_in_slp = of_property_read_bool(np, "apw888x-use-nrm-volt-in-slp");

	return 0;
}

unsigned int apw888x_regulator_dc_of_map_mode(unsigned int mode)
{
	switch (mode) {
	case APW8889_DC_MODE_FORCE_PWM:
		return REGULATOR_MODE_FAST;
	default:
		break;
	}
	return REGULATOR_MODE_NORMAL;
}
EXPORT_SYMBOL_GPL(apw888x_regulator_dc_of_map_mode);

unsigned int apw888x_regulator_ldo_of_map_mode(unsigned int mode)
{
	return REGULATOR_MODE_NORMAL;
}
EXPORT_SYMBOL_GPL(apw888x_regulator_ldo_of_map_mode);

/* regulator_ops */
static int apw888x_regulator_set_mode_regmap(struct regulator_dev *rdev,
					     unsigned int mode)
{
	struct apw888x_regulator_data *data = rdev_get_drvdata(rdev);
	struct apw888x_regulator_desc *desc = data->desc;
	unsigned int val = 0;

	dev_dbg(rdev_get_dev(rdev), "%s\n", __func__);
	if (!data->nmode)
		return -EINVAL;

	if (apw888x_regulator_type_is_ldo(desc))
		val = 0;
	else
		val = (mode & REGULATOR_MODE_FAST) ? 2 : 0;

	return regmap_field_write(data->nmode, val);
}

static
unsigned int apw888x_regulator_get_mode_regmap(struct regulator_dev *rdev)
{
	struct apw888x_regulator_data *data = rdev_get_drvdata(rdev);
	struct apw888x_regulator_desc *desc = data->desc;
	unsigned int val;
	int ret;

	dev_dbg(rdev_get_dev(rdev), "%s\n", __func__);
	if (!data->nmode)
		return -EINVAL;

	ret = regmap_field_read(data->nmode, &val);
	if (ret)
		return 0;

	if (apw888x_regulator_type_is_ldo(desc) && val == 2)
		return REGULATOR_MODE_IDLE;
	else if (val == 2)
		return REGULATOR_MODE_FAST;
	return REGULATOR_MODE_NORMAL;
}

static int apw888x_set_sleep_mode(struct regulator_dev *rdev, int mode)
{
	struct apw888x_regulator_data *data = rdev_get_drvdata(rdev);

	if (!data->smode)
		return -EINVAL;

	dev_dbg(rdev_get_dev(rdev), "%s to %d\n", __func__, mode);
	return regmap_field_write(data->smode, mode);
}

static int apw888x_regulator_set_suspend_enable(struct regulator_dev *rdev)
{
	struct apw888x_regulator_data *data = rdev_get_drvdata(rdev);
	int val = data->use_nrm_in_slp ? APW888X_SLEEP_MODE_NRM_VOLT : APW888X_SLEEP_MODE_SLP_VOLT;

	dev_dbg(rdev_get_dev(rdev), "%s\n", __func__);
	return apw888x_set_sleep_mode(rdev, val);
}

static int apw888x_regulator_set_suspend_disable(struct regulator_dev *rdev)
{
	struct apw888x_regulator_data *data = rdev_get_drvdata(rdev);

	dev_dbg(rdev_get_dev(rdev), "%s\n", __func__);
	apw888x_set_sleep_mode(rdev, APW888X_SLEEP_MODE_SHUTDOWN);

	/* dischg only vaild when shutdown */
	if (data->dischg && data->dischg_dis) {
		dev_info(rdev_get_dev(rdev), "%s: set dischg disabled\n", __func__);
		regmap_field_write(data->dischg, 0x0);
	}
	return 0;
}

static int apw888x_regulator_set_suspend_voltage(struct regulator_dev *rdev, int uV)
{
	struct apw888x_regulator_data *data = rdev_get_drvdata(rdev);
	int vsel;

	dev_dbg(rdev_get_dev(rdev), "%s: uV=%d\n", __func__, uV);
	vsel = regulator_map_voltage_iterate(rdev, uV, uV);
	if (vsel < 0)
		return -EINVAL;

	if (!data->smode || !data->svsel)
		return -EINVAL;

	apw888x_set_sleep_mode(rdev, APW888X_SLEEP_MODE_SLP_VOLT);
	regmap_field_write(data->svsel, vsel);
	return 0;
}

static int apw888x_regulator_resume(struct regulator_dev *rdev)
{
	dev_dbg(rdev_get_dev(rdev), "%s\n", __func__);
	return apw888x_set_sleep_mode(rdev, APW888X_SLEEP_MODE_NRM_VOLT);
}

static int apw888x_regulator_get_voltage_fixed(struct regulator_dev *rdev)
{
	struct apw888x_regulator_data *data = rdev_get_drvdata(rdev);

	if (data->fixed_uV)
		return data->fixed_uV;
	return -EINVAL;
}

const struct regulator_ops apw888x_regulator_ops = {
	.list_voltage         = regulator_list_voltage_linear,
	.map_voltage          = regulator_map_voltage_linear,
	.set_voltage_sel      = regulator_set_voltage_sel_regmap,
	.get_voltage_sel      = regulator_get_voltage_sel_regmap,
	.enable               = regulator_enable_regmap,
	.disable              = regulator_disable_regmap,
	.is_enabled           = regulator_is_enabled_regmap,
	.get_mode             = apw888x_regulator_get_mode_regmap,
	.set_mode             = apw888x_regulator_set_mode_regmap,
	.set_suspend_voltage  = apw888x_regulator_set_suspend_voltage,
	.set_suspend_enable   = apw888x_regulator_set_suspend_enable,
	.set_suspend_disable  = apw888x_regulator_set_suspend_disable,
	.resume               = apw888x_regulator_resume,
};
EXPORT_SYMBOL_GPL(apw888x_regulator_ops);

const struct regulator_ops apw888x_regulator_fixed_uV_ops = {
	.enable               = regulator_enable_regmap,
	.disable              = regulator_disable_regmap,
	.is_enabled           = regulator_is_enabled_regmap,
	.get_mode             = apw888x_regulator_get_mode_regmap,
	.set_mode             = apw888x_regulator_set_mode_regmap,
	.set_suspend_voltage  = apw888x_regulator_set_suspend_voltage,
	.set_suspend_enable   = apw888x_regulator_set_suspend_enable,
	.set_suspend_disable  = apw888x_regulator_set_suspend_disable,
	.get_voltage          = apw888x_regulator_get_voltage_fixed,
	.resume               = apw888x_regulator_resume,
};
EXPORT_SYMBOL_GPL(apw888x_regulator_fixed_uV_ops);

const struct regulator_ops apw888x_regulator_vfb5_ops = {
	.list_voltage         = regulator_list_voltage_linear,
	.map_voltage          = regulator_map_voltage_linear,
	.set_voltage_sel      = regulator_set_voltage_sel_regmap,
	.get_voltage_sel      = regulator_get_voltage_sel_regmap,
};
EXPORT_SYMBOL_GPL(apw888x_regulator_vfb5_ops);

static struct regmap_field *
create_regmap_field(struct apw888x_regulator_device *regdev, u32 reg, u32 mask)
{
	u32 msb = fls(mask) - 1;
	u32 lsb = ffs(mask) - 1;
	struct reg_field map = REG_FIELD(reg, lsb, msb);
	struct regmap_field *rmap;

	if (reg == 0 && mask == 0)
		return NULL;

	dev_dbg(regdev->dev, "reg=%02x, mask=%02x, lsb=%d, msb=%d\n",
		reg, mask, lsb, msb);
	rmap = devm_regmap_field_alloc(regdev->dev, regdev->regmap, map);
	if (IS_ERR(rmap)) {
		dev_err(regdev->dev, "failed to alloc regmap field for (reg=%02x, mask=%02x): %ld\n",
			reg, mask, PTR_ERR(rmap));
	}
	return rmap;
}


#define APW888X_DC_MODE_MASK    (REGULATOR_MODE_NORMAL | REGULATOR_MODE_FAST)
#define APW888X_LDO_MODE_MASK   (REGULATOR_MODE_NORMAL)

int apw888x_regulator_register(struct apw888x_regulator_device *regdev,
			       struct apw888x_regulator_desc *desc)
{
	struct device *dev = regdev->dev;
	struct apw888x_regulator_data *data;
	struct regulator_config config = {};
	struct regulator_init_data *init_data = NULL;
	struct regulation_constraints *c;
	struct device_node *child;

	for_each_child_of_node(dev->of_node, child) {
		if (of_node_cmp(child->name, desc->desc.name))
			continue;

		init_data = of_get_regulator_init_data(dev, child, &desc->desc);
		break;
	}
	if (!init_data)
		return -EINVAL;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->desc = desc;
	list_add_tail(&data->list, &regdev->list);

	c = &init_data->constraints;
	c->initial_state = 0;
	c->valid_ops_mask |= REGULATOR_CHANGE_MODE;
	c->valid_modes_mask |= apw888x_regulator_type_is_ldo(data->desc) ?
		APW888X_LDO_MODE_MASK : APW888X_DC_MODE_MASK;
	apw888x_regulator_of_parse_data(child, &desc->desc, data);

	data->nmode = create_regmap_field(regdev, desc->nmode_reg,
					  desc->nmode_mask);
	if (IS_ERR(data->nmode))
		return PTR_ERR(data->nmode);
	data->smode = create_regmap_field(regdev, desc->smode_reg,
					  desc->smode_mask);
	if (IS_ERR(data->smode))
		return PTR_ERR(data->smode);
	data->svsel = create_regmap_field(regdev, desc->svsel_reg,
					  desc->svsel_mask);
	if (IS_ERR(data->svsel))
		return PTR_ERR(data->svsel);
	data->dischg = create_regmap_field(regdev, desc->dischg_reg,
					   desc->dischg_mask);
	if (IS_ERR(data->dischg))
		return PTR_ERR(data->dischg);

	config.dev         = regdev->dev;
	config.regmap      = regdev->regmap;
	config.driver_data = data;
	config.of_node     = child;
	config.init_data   = init_data;

	data->rdev = devm_regulator_register(dev, &desc->desc, &config);
	if (IS_ERR(data->rdev))
		return PTR_ERR(data->rdev);

	apw888x_set_sleep_mode(data->rdev, APW888X_SLEEP_MODE_NRM_VOLT);

	return 0;
}
EXPORT_SYMBOL_GPL(apw888x_regulator_register);

static int apw888x_regulator_set_sleep_state(struct regulator_dev *rdev, int is_shutdown)
{
	struct apw888x_regulator_data *data = rdev_get_drvdata(rdev);
	struct regulator_state *rstate;

       if (!data->smode)
               return -EINVAL;

       rstate = is_shutdown ? &data->state_shutdown : &rdev->constraints->state_mem;

       if (rstate->enabled == ENABLE_IN_SUSPEND)
	       apw888x_regulator_set_suspend_enable(rdev);

       if (rstate->enabled == ENABLE_IN_SUSPEND && rstate->uV)
	       apw888x_regulator_set_suspend_voltage(rdev, rstate->uV);

       if (rstate->enabled == DISABLE_IN_SUSPEND)
	       apw888x_regulator_set_suspend_disable(rdev);

       return -EINVAL;
}

int apw888x_regulator_device_generic_suspend(struct apw888x_regulator_device *regdev)
{
	return 0;
}
EXPORT_SYMBOL_GPL(apw888x_regulator_device_generic_suspend);

int apw888x_regulator_device_generic_resume(struct apw888x_regulator_device *regdev)
{
	return 0;
}
EXPORT_SYMBOL_GPL(apw888x_regulator_device_generic_resume);

int apw888x_regulator_device_generic_shutdown(struct apw888x_regulator_device *regdev)
{
	struct apw888x_regulator_data *pos;

	list_for_each_entry(pos, &regdev->list, list)
		apw888x_regulator_set_sleep_state(pos->rdev, 1);
	return 0;
}
EXPORT_SYMBOL_GPL(apw888x_regulator_device_generic_shutdown);

MODULE_LICENSE("GPL v2");
