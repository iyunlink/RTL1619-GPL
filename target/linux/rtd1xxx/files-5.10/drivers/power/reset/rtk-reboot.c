// SPDX-License-Identifier: GPL-2.0-only
/*
 * rtk-reboot.c
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/delay.h>

#define ISO_TCWCR             0x00
#define ISO_TCWTR             0x04
#define ISO_TCWNMI            0x08
#define ISO_TCWOV             0x0c
#define ISO_TCWOV_RSTB_CNT    0x40
#define ISO_TCWOV_RSTB_PAD    0x44

#define OE_INVALID            (-1)

struct rtk_reboot_data {
	struct device *dev;
	struct notifier_block reboot_nb;
	struct notifier_block restart_nb;
	int oe;
	int oe_init;
	void *base;
};

static void rtk_reboot_set_oe(struct rtk_reboot_data *d, int oe)
{
	u32 val;

	val = readl(d->base + ISO_TCWOV_RSTB_PAD);
	val &= ~0x1;
	val |= oe & 0x1;
	writel(val, d->base + ISO_TCWOV_RSTB_PAD);
}

static int rtk_reboot_cb(struct notifier_block *nb, unsigned long mode,
			 void *cmd)
{
	struct rtk_reboot_data *d;

	d = container_of(nb, struct rtk_reboot_data, reboot_nb);

	if (d->oe < 0)
		return NOTIFY_DONE;

	dev_info(d->dev, "set oe to %d\n", d->oe);
	rtk_reboot_set_oe(d, d->oe);
	return NOTIFY_OK;
}

static int rtk_reboot_restart_handler(struct notifier_block *nb,
				      unsigned long mode, void *cmd)
{
	struct rtk_reboot_data *d;

	d = container_of(nb, struct rtk_reboot_data, restart_nb);

	dev_emerg(d->dev, "Restarting...\n");

	writel(0xA5, d->base + ISO_TCWCR);
	writel(0x01, d->base + ISO_TCWTR);
	writel(0x04, d->base + ISO_TCWOV);
	writel(0x00, d->base + ISO_TCWCR);

	mdelay(10000);
	dev_emerg(d->dev, "Unable to restart system\n");

	return NOTIFY_DONE;
}

static int rtk_reboot_parse_args(struct rtk_reboot_data *d)
{
	struct device_node *np = d->dev->of_node;

	d->oe_init = OE_INVALID;
	d->oe      = OE_INVALID;

	of_property_read_u32(np, "rst-oe-for-init", &d->oe_init);
	of_property_read_u32(np, "rst-oe", &d->oe);
	return 0;
}

static int rtk_reboot_init_tcw(struct rtk_reboot_data *d)
{
	if (d->oe_init == OE_INVALID)
		return 0;

	dev_info(d->dev, "set oe to %d\n", d->oe_init);
	writel(0x00800000, d->base + ISO_TCWOV_RSTB_CNT);
	rtk_reboot_set_oe(d, d->oe_init);
	return 0;
};

static int rtk_reboot_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_reboot_data *d;
	struct resource *res;
	int ret;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	d->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(d->base))
		return PTR_ERR(d->base);

	d->reboot_nb.notifier_call = rtk_reboot_cb;
	ret = register_reboot_notifier(&d->reboot_nb);
	if (ret)
		return ret;

	d->restart_nb.notifier_call = rtk_reboot_restart_handler;
	ret = register_restart_handler(&d->restart_nb);
	if (ret) {
		dev_err(dev, "failed to register restart_handler: %d\n", ret);
		unregister_reboot_notifier(&d->reboot_nb);
		return ret;
	}

	rtk_reboot_parse_args(d);
	rtk_reboot_init_tcw(d);
	return 0;
}

static const struct of_device_id rtk_reboot_match[] = {
	{ .compatible = "realtek,reboot", },
	{}
};
MODULE_DEVICE_TABLE(of, rtk_reboot_match);

static struct platform_driver rtk_reboot_driver = {
	.probe = rtk_reboot_probe,
	.driver = {
		.name = "rtk-reboot",
		.of_match_table = of_match_ptr(rtk_reboot_match),
	},
};

static int __init rtk_reboot_init(void)
{
	return platform_driver_register(&rtk_reboot_driver);
}
module_init(rtk_reboot_init);

static void __exit rtk_reboot_exit(void)
{
	platform_driver_unregister(&rtk_reboot_driver);
}
module_exit(rtk_reboot_exit);

MODULE_DESCRIPTION("Reatek Reboot Driver");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");

