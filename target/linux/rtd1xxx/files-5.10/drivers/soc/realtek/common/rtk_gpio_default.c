// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Realtek DHC gpio default init driver
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>



struct rtd_gpio_default_state {
	struct gpio_descs *gpio;
	int *values;
};



static int rtd_gpio_default_probe(struct platform_device *pdev)
{
	struct gpio_descs *rtd_gpios;
	int num = 0;
	int value = 0;
	int i;
	int ret = 0;

	rtd_gpios = gpiod_get_array(&pdev->dev, NULL, GPIOD_ASIS);
	if (IS_ERR(rtd_gpios)) {
		dev_err(&pdev->dev, "No default gpios need to be set\n");
		return 0;
	} else
		dev_err(&pdev->dev, "set %d gpios to default value\n", rtd_gpios->ndescs);


	num = of_property_count_u32_elems(pdev->dev.of_node, "gpio-default");
	if (num < 0 || num != rtd_gpios-> ndescs) {
		dev_err(&pdev->dev, "number of gpio-default is not match to number of gpios\n");
		goto out;
	}
	for (i = 0; i < num; i++) {
		of_property_read_u32_index(pdev->dev.of_node, "gpio-default", i, &value);
		switch (value) {
		case 0:
			ret = gpiod_direction_output(rtd_gpios->desc[i], 0);
			if (ret)
				dev_err(&pdev->dev, "cannot set gpio %d", desc_to_gpio(rtd_gpios->desc[i]));
			break;
		case 1:
			ret = gpiod_direction_output(rtd_gpios->desc[i], 1);
			if (ret)
				dev_err(&pdev->dev, "cannot set gpio %d", desc_to_gpio(rtd_gpios->desc[i]));
			break;
		case 2:
			ret = gpiod_direction_input(rtd_gpios->desc[i]);
			if (ret)
				dev_err(&pdev->dev, "cannot set gpio %d", desc_to_gpio(rtd_gpios->desc[i]));
			break;
		default:
			dev_err(&pdev->dev, "default value is not support for gpio %d", desc_to_gpio(rtd_gpios->desc[i]));

		}
	}

out:
	gpiod_put_array(rtd_gpios);

	return 0;
}

static const struct of_device_id rtd_gpio_default_of_matches[] = {
	{ .compatible = "realtek,gpio-set-default",},
	{}
};


static struct platform_driver rtd_gpio_default_driver = {
	.driver = {
		.name = "gpio-rtd-default",
		.of_match_table = rtd_gpio_default_of_matches,
	},
	.probe = rtd_gpio_default_probe,
};


static int rtd_gpio_default(void)
{
	return platform_driver_register(&rtd_gpio_default_driver);
}

postcore_initcall(rtd_gpio_default);

MODULE_LICENSE("GPL");
