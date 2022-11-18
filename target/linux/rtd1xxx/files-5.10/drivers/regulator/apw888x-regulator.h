/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2019 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#ifndef __APW888X_REGULATOR_H
#define __APW888X_REGULATOR_H

#include <linux/list.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/mfd/apw888x.h>

struct apw888x_regulator_desc {
	struct regulator_desc desc;
	u8 nmode_reg;
	u8 nmode_mask;
	u8 smode_reg;
	u8 smode_mask;
	u8 svsel_reg;
	u8 svsel_mask;
	u8 dischg_reg;
	u8 dischg_mask;
};

struct apw888x_regulator_device {
	struct device *dev;
	struct regmap *regmap;
	struct list_head list;
};

struct apw888x_regulator_data {
	struct list_head list;
	struct regulator_dev *rdev;
	struct apw888x_regulator_desc *desc;

	struct regmap_field *svsel;
	struct regmap_field *smode;
	struct regmap_field *nmode;
	struct regmap_field *clamp;
	struct regmap_field *dischg;

	struct regulator_state state_shutdown;
	u32 dischg_dis;
	int use_nrm_in_slp;

	u32 fixed_uV;
};

int apw888x_regulator_of_parse_cb(struct device_node *np,
				  const struct regulator_desc *desc,
				  struct regulator_config *config);
unsigned int apw888x_regulator_dc_of_map_mode(unsigned int mode);
unsigned int apw888x_regulator_ldo_of_map_mode(unsigned int mode);

static inline int apw888x_regulator_type_is_ldo(struct apw888x_regulator_desc *gd)
{
	return gd->desc.of_map_mode == apw888x_regulator_ldo_of_map_mode;
}

extern const struct regulator_ops apw888x_regulator_ops;
extern const struct regulator_ops apw888x_regulator_fixed_uV_ops;
extern const struct regulator_ops apw888x_regulator_vfb5_ops;
/*
 * apw888x_regulator_set_state - set apw888x suspend state
 *
 * @rdev:  regulator device
 * @state: state selection
 */
int apw888x_regulator_set_state(struct regulator_dev *rdev, int state);

#define APW888X_DC_MODE_MASK    (REGULATOR_MODE_NORMAL | REGULATOR_MODE_FAST)
#define APW888X_LDO_MODE_MASK   (REGULATOR_MODE_NORMAL)

int apw888x_regulator_register(struct apw888x_regulator_device *regdev,
			       struct apw888x_regulator_desc *desc);

int apw888x_regulator_device_generic_suspend(struct apw888x_regulator_device *regdev);
int apw888x_regulator_device_generic_resume(struct apw888x_regulator_device *regdev);
int apw888x_regulator_device_generic_shutdown(struct apw888x_regulator_device *regdev);

#endif /* __APW888X_REGULATOR_H */
