// SPDX-License-Identifier: GPL-2.0-only
/*
 * apw8886-i2c.c - Anpec APW8886 PMIC I2C driver
 *
 * Copyright (C) 2018-2020 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/mfd/core.h>
#include <linux/mfd/apw888x.h>
#include <linux/mfd/apw8886.h>

static bool apw8886_regmap_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case APW8886_REG_INTR ... APW8886_REG_PWRKEY:
	case APW8886_REG_FAULT_STATUS:
	case APW8886_REG_SYS_CONTROL ... APW8886_REG_LDO1_SLPVOLT:
	case APW8886_REG_CLAMP:
	case APW8886_REG_VFB5_REF_VOLT_DAC:
	case APW8886_REG_CHIP_ID:
	case APW8886_REG_VERSION:
		return true;
	}
	return false;
}

static bool apw8886_regmap_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case APW8886_REG_INTR_MASK ... APW8886_REG_PWRKEY:
	case APW8886_REG_SYS_CONTROL ... APW8886_REG_LDO1_SLPVOLT:
	case APW8886_REG_VFB5_REF_VOLT_DAC:
	case APW8886_REG_CLAMP:
		return true;
	}
	return false;
}

static bool apw8886_regmap_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case APW8886_REG_INTR_MASK ... APW8886_REG_PWRKEY:
	case APW8886_REG_FAULT_STATUS:
	case APW8886_REG_SYS_CONTROL:
	case APW8886_REG_CLAMP:
	case APW8886_REG_CHIP_ID:
	case APW8886_REG_VERSION:
		return true;
	}
	return false;
}

static bool apw8886_regmap_precious_reg(struct device *dev, unsigned int reg)
{
        return reg == APW8886_REG_INTR;
}

static const struct regmap_config apw8886_regmap_config = {
	.reg_bits         = 8,
	.val_bits         = 8,
	.max_register     = 0x1F,
	.cache_type       = REGCACHE_RBTREE,
	.readable_reg     = apw8886_regmap_readable_reg,
	.writeable_reg    = apw8886_regmap_writeable_reg,
	.volatile_reg     = apw8886_regmap_volatile_reg,
	.precious_reg     = apw8886_regmap_precious_reg,
};

static inline unsigned long apw8886_i2c_get_driver_data(struct i2c_client *client, const struct i2c_device_id *id)
{
	if (IS_ENABLED(CONFIG_OF) && client->dev.of_node)
		return (unsigned long)of_device_get_match_data(&client->dev);
	return id->driver_data;
}

static int apw8886_chip_id_valid(u32 chip_id)
{
	return chip_id == 0x5a || chip_id == 0x9a || chip_id == 0xda;
}

static struct mfd_cell apw8886_devs[] = {
	{
		.name = "apw8886-regulator",
		.of_compatible = "anpec,apw8886-regulator",
	},
};

static int apw8886_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct apw888x_device *adev;
	int ret;
	u32 chip_id, rev;
	unsigned long pmic_id = apw8886_i2c_get_driver_data(client, id);

	adev = devm_kzalloc(dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->regmap = devm_regmap_init_i2c(client, &apw8886_regmap_config);
	if (IS_ERR(adev->regmap))
		return PTR_ERR(adev->regmap);

	/* show chip info */
	ret = regmap_read(adev->regmap, APW8886_REG_CHIP_ID, &chip_id);
	if (ret) {
		dev_err(dev, "failed to read chip_id: %d\n", ret);
		return ret;
	}

	if (!apw8886_chip_id_valid(chip_id)) {
		dev_err(dev, "chip_id(%02x) not match\n", chip_id);
		return -EINVAL;
	}

	regmap_read(adev->regmap, APW8886_REG_VERSION, &rev);
	dev_info(dev, "(%x) rev%d\n", chip_id, rev);

	adev->chip_id = pmic_id;
	adev->chip_rev = rev;
	adev->dev = dev;
	i2c_set_clientdata(client, adev);

	ret = devm_mfd_add_devices(adev->dev, PLATFORM_DEVID_NONE,
		apw8886_devs, ARRAY_SIZE(apw8886_devs), 0, 0, 0);
	if (ret) {
		dev_err(dev, "failed to add sub-devices: %d\n", ret);
		return ret;
	}
	return 0;
}

static int apw8886_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static void apw8886_i2c_shutdown(struct i2c_client *client)
{
	struct apw888x_device *adev = i2c_get_clientdata(client);
	unsigned int val = adev->chip_id == APW888X_DEVICE_ID_APW8886 ? 0x24 : 0x28;

	dev_info(&client->dev, "reset dc3 nrmvolt\n");
	regmap_write(adev->regmap, APW8886_REG_DC3_NRMVOLT, val);
}

static const struct of_device_id apw8886_of_match[] = {
	{ .compatible = "anpec,apw8886", .data = (void *)APW888X_DEVICE_ID_APW8886, },
	{ .compatible = "anpec,apw7899", .data = (void *)APW888X_DEVICE_ID_APW7899, },
	{}
};
MODULE_DEVICE_TABLE(of, apw8886_of_match);

static const struct i2c_device_id apw8886_i2c_ids[] = {
	{"apw8886", APW888X_DEVICE_ID_APW8886},
	{}
};
MODULE_DEVICE_TABLE(i2c, apw8886_i2c_ids);

static struct i2c_driver apw8886_i2c_driver = {
	.driver = {
		.name = "apw8886",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(apw8886_of_match),
	},
	.id_table = apw8886_i2c_ids,
	.probe    = apw8886_i2c_probe,
	.remove   = apw8886_i2c_remove,
	.shutdown = apw8886_i2c_shutdown,
};
module_i2c_driver(apw8886_i2c_driver);

MODULE_DESCRIPTION("Anpec APW8886 PMIC MFD Driver");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");
