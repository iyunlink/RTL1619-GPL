/*
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/gpio/consumer.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <soc/realtek/rtk_chip.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#define LSADC0_PAD0 0x900
#define ISO_PFUNC21 0x74

#define ISO_DBG_STATUS 0x190
#define ISO_PFUNC0 0x2c
#define ISO_PFUNC1 0x30

static struct rfkill *bt_rfk;
static const char bt_name[] = "bluetooth";
static struct gpio_desc *bt_reset;

static const struct soc_device_attribute rtk_soc_stark[] = {
	{
		.family = "Realtek Stark",
	},
	{
		/* empty */
	}
};

static const struct soc_device_attribute rtk_soc_parker[] = {
	{
		.family = "Realtek Parker",
	},
	{
		/* empty */
	}
};

static int bluetooth_set_power(void *data, bool blocked)
{
	pr_info("%s: block=%d\n", __func__, blocked);

	if (!blocked)
		gpiod_direction_output(bt_reset, 1);
	else
		gpiod_direction_output(bt_reset, 0);

	return 0;
}

static struct rfkill_ops rfkill_bluetooth_ops = {
	.set_block = bluetooth_set_power,
};

static int rfkill_gpio_init(struct device *dev)
{
	/*initial gpios*/
	/* get gpio number from device tree*/
	bt_reset = devm_gpiod_get(dev, "rfkill",
				GPIOD_OUT_LOW);
	if (IS_ERR(bt_reset)) {
		pr_err("[%s ] could not request gpio\n", __func__);
		return -1;
	}
	return 0;
}

static void rfkill_gpio_deinit(void)
{
	gpiod_put(bt_reset);
}

static int rfkill_bluetooth_probe(struct platform_device *pdev)
{

	int rc = 0;
	bool default_state = true;
	struct device *rtk119x_bt_dev;
	struct device_node *rtk119x_bt_node = NULL;

	rtk119x_bt_dev = &pdev->dev;
	rtk119x_bt_node = pdev->dev.of_node;

	pr_info("-->%s\n", __func__);

	rc = rfkill_gpio_init(rtk119x_bt_dev);
	if (rc)
		goto deferred;

	if(soc_device_match(rtk_soc_stark)) {
		struct regmap           *iso_base;
		struct regmap           *pad_base;
		u32 iso_tmp=0, pad_tmp=0;

		iso_base = syscon_regmap_lookup_by_phandle(rtk119x_bt_node, "realtek,iso");
		pad_base = syscon_regmap_lookup_by_phandle(rtk119x_bt_node, "realtek,pinctrl");

		if (IS_ERR_OR_NULL(iso_base) || IS_ERR_OR_NULL(pad_base)) {
			printk(KERN_ERR "iso_base = NULL or pad_base = NULL !!!\n");
			return -ENOMEM;
		}

		regmap_read(iso_base, LSADC0_PAD0, &iso_tmp);

		if((iso_tmp&0xff)>=0x94) {
			pad_tmp = BIT(11)|BIT(16)|BIT(21)|BIT(26);
			regmap_update_bits_base(pad_base, ISO_PFUNC21,
						pad_tmp, pad_tmp, NULL, false, true);
		}
		else  {
			pad_tmp = BIT(11)|BIT(16)|BIT(21)|BIT(26);
			regmap_update_bits_base(pad_base, ISO_PFUNC21,
						pad_tmp, 0, NULL, false, true);
		}
	}
	else if(soc_device_match(rtk_soc_parker)) {
		struct regmap           *pad_base;
		u32 dbg_tmp=0, dbg_status=0, pad_tmp=0;

		pad_base = syscon_regmap_lookup_by_phandle(rtk119x_bt_node, "realtek,pinctrl");

		if (IS_ERR_OR_NULL(pad_base)) {
			printk(KERN_ERR "iso_base = NULL or pad_base = NULL !!!\n");
			return -ENOMEM;
		}
		dbg_tmp = BIT(6);
		regmap_update_bits_base(pad_base, ISO_DBG_STATUS,
						dbg_tmp, dbg_tmp, NULL, false, true);
		mdelay(100);
		regmap_read(pad_base, ISO_DBG_STATUS, &dbg_status);

		if(dbg_status & BIT(3)) {
			pad_tmp = BIT(27);
			regmap_update_bits_base(pad_base, ISO_PFUNC0,
				pad_tmp, pad_tmp, NULL, false, true);
			pad_tmp = BIT(4)|BIT(9)|BIT(14);
			regmap_update_bits_base(pad_base, ISO_PFUNC1,
				pad_tmp, pad_tmp, NULL, false, true);
		}
		else {
			pad_tmp = BIT(27);
			regmap_update_bits_base(pad_base, ISO_PFUNC0,
				pad_tmp, 0, NULL, false, true);
			pad_tmp = BIT(4)|BIT(9)|BIT(14);
			regmap_update_bits_base(pad_base, ISO_PFUNC1,
				pad_tmp, 0, NULL, false, true);
		}
	}

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
	   &rfkill_bluetooth_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_alloc;
	}

	/* userspace cannot take exclusive control */
	rfkill_init_sw_state(bt_rfk, false);
	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	rfkill_set_sw_state(bt_rfk, true);
	bluetooth_set_power(NULL, default_state);

	pr_info("<--%s\n", __func__);
	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
	return rc;
deferred:
	return -EPROBE_DEFER;
}

static int rfkill_bluetooth_remove(struct platform_device *dev)
{
	pr_info("-->%s\n", __func__);
	rfkill_gpio_deinit();
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
	pr_info("<--%s\n", __func__);
	return 0;
}
static const struct of_device_id rtk119x_bt_ids[] = {
	{ .compatible = "realtek,rfkill" },
	{ /* Sentinel */ },
};
static struct platform_driver rfkill_bluetooth_driver = {
	.probe  = rfkill_bluetooth_probe,
	.remove = rfkill_bluetooth_remove,
	.driver = {
		.name = "rfkill",
		.owner = THIS_MODULE,
		.of_match_table = rtk119x_bt_ids,
	},
};

static int __init rfkill_bluetooth_init(void)
{
	pr_info("-->%s\n", __func__);
	return platform_driver_register(&rfkill_bluetooth_driver);
}

static void __exit rfkill_bluetooth_exit(void)
{
	pr_info("-->%s\n", __func__);
	platform_driver_unregister(&rfkill_bluetooth_driver);
}

late_initcall(rfkill_bluetooth_init);
module_exit(rfkill_bluetooth_exit);
MODULE_DESCRIPTION("bluetooth rfkill");
MODULE_AUTHOR("rs <wn@realsil.com.cn>");
MODULE_LICENSE("GPL");

