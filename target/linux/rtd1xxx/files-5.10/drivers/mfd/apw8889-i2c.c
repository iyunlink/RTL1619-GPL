// SPDX-License-Identifier: GPL-2.0-only
/*
 * apw8889-i2c.c - Anpec APW8889 PMIC I2C driver
 *
 * Copyright (C) 2018-2019 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/mfd/core.h>
#include <linux/mfd/apw888x.h>
#include <linux/mfd/apw8889.h>

static bool apw8889_regmap_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case APW8889_REG_INTR ... APW8889_REG_PWRKEY:
	case APW8889_REG_FAULT_STATUS:
	case APW8889_REG_SYS_CONTROL ... APW8889_REG_LDO1_SLPVOLT:
	case APW8889_REG_CLAMP:
	case APW8889_REG_CHIP_ID:
	case APW8889_REG_VERSION:
		return true;
	}
	return false;
}

static bool apw8889_regmap_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case APW8889_REG_INTR_MASK ... APW8889_REG_PWRKEY:
	case APW8889_REG_SYS_CONTROL ... APW8889_REG_LDO1_SLPVOLT:
	case APW8889_REG_CLAMP:
		return true;
	}
	return false;
}

static bool apw8889_regmap_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case APW8889_REG_INTR_MASK ... APW8889_REG_PWRKEY:
	case APW8889_REG_FAULT_STATUS:
	case APW8889_REG_SYS_CONTROL:
	case APW8889_REG_CLAMP:
	case APW8889_REG_CHIP_ID:
	case APW8889_REG_VERSION:
	case APW8889_REG_DC1DC2_MODE ... APW8889_REG_DC5DC6_MODE:
		return true;
	}
	return false;
}

static const struct regmap_config apw8889_regmap_config = {
	.reg_bits         = 8,
	.val_bits         = 8,
	.max_register     = 0x1F,
	.cache_type       = REGCACHE_RBTREE,
	.readable_reg     = apw8889_regmap_readable_reg,
	.writeable_reg    = apw8889_regmap_writeable_reg,
	.volatile_reg     = apw8889_regmap_volatile_reg,
};

static struct mfd_cell apw8889_devs[] = {
	{
		.name = "apw8889-regulator",
		.of_compatible = "anpec,apw8889-regulator",
	},
};

static int apw8889_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct apw888x_device *adev;
	int ret;
	u32 chip_id, rev;

	adev = devm_kzalloc(dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->regmap = devm_regmap_init_i2c(client, &apw8889_regmap_config);
	if (IS_ERR(adev->regmap))
		return PTR_ERR(adev->regmap);

	/* show chip info */
	ret = regmap_read(adev->regmap, APW8889_REG_CHIP_ID, &chip_id);
	if (ret) {
		dev_err(dev, "failed to read chip_id: %d\n", ret);
		return ret;
	}
	if (chip_id != 0x5a) {
		dev_err(dev, "chip_id(%02x) not match\n", chip_id);
		return -EINVAL;
	}
	regmap_read(adev->regmap, APW8889_REG_VERSION, &rev);
	dev_info(dev, "apw8889(%02x) rev%d\n", chip_id, rev);


	adev->chip_id = APW888X_DEVICE_ID_APW8889;
	adev->chip_rev = rev;
	adev->dev = dev;
	i2c_set_clientdata(client, adev);

	ret = devm_mfd_add_devices(adev->dev, PLATFORM_DEVID_NONE,
		apw8889_devs, ARRAY_SIZE(apw8889_devs), 0, 0, 0);
	if (ret) {
		dev_err(dev, "failed to add sub-devices: %d\n", ret);
		return ret;
	}
	return 0;
}

static int apw8889_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id apw8889_of_match[] = {
	{ .compatible = "anpec,apw8889", },
	{}
};
MODULE_DEVICE_TABLE(of, apw8889_of_match);

static const struct i2c_device_id apw8889_i2c_ids[] = {
	{"apw8889", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, apw8889_i2c_ids);

static struct i2c_driver apw8889_i2c_driver = {
	.driver = {
		.name = "apw8889",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(apw8889_of_match),
	},
	.id_table = apw8889_i2c_ids,
	.probe    = apw8889_i2c_probe,
	.remove   = apw8889_i2c_remove,
};
module_i2c_driver(apw8889_i2c_driver);

MODULE_DESCRIPTION("Anpec APW8889 PMIC MFD Driver");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");
