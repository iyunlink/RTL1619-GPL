// SPDX-License-Identifier: GPL-2.0-only
/*
 * rtk-reboot-mode.c
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <soc/realtek/rtk_rstctl.h>

struct rtk_reboot_mode_data {
	struct device *dev;
	struct notifier_block reboot_nb;
	void *base;
};

static int rtk_reboot_mode_cb(struct notifier_block *nb, unsigned long mode,
			      void *cmd)
{
	struct rtk_reboot_mode_data *d;
	u32 act = RESET_ACTION_NO_ACTION;

	d = container_of(nb, struct rtk_reboot_mode_data, reboot_nb);
	if (cmd) {
		if (!strncmp("bootloader", cmd, 11))
			act = RESET_ACTION_FASTBOOT;
		else if (!strncmp("recovery", cmd, 9))
			act = RESET_ACTION_RECOVERY;
		else if (!strncmp("quiescent", cmd, 10))
			act = RESET_ACTION_QUIESCENT;
#ifdef CONFIG_RTK_VMX_ULTRA
		else if (!strncmp("dm-verity device corrupted", cmd, 27))
			act = RESET_ACTION_RESCUE;
#endif
		dev_info(d->dev, "reboot-mode is %s(%x)\n", (const char *)cmd, act);
	}

	writel(act | RESET_MAGIC, d->base);
	return NOTIFY_OK;
}

static int rtk_reboot_mode_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_reboot_mode_data *d;
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

	d->reboot_nb.notifier_call = rtk_reboot_mode_cb;
	ret = register_reboot_notifier(&d->reboot_nb);
	if (ret)
		return ret;

	writel(RESET_ACTION_ABNORMAL | RESET_MAGIC, d->base);
	return 0;
}

static struct of_device_id rtk_reboot_mode_match[] = {
	{ .compatible = "realtek,reboot-mode", },
	{}
};
MODULE_DEVICE_TABLE(of, rtk_reboot_mode_match);

static struct platform_driver rtk_reboot_mode_driver = {
	.probe = rtk_reboot_mode_probe,
	.driver = {
		.name = "rtk-reboot-mode",
		.of_match_table = of_match_ptr(rtk_reboot_mode_match),
	},
};

static int __init rtk_reboot_mode_init(void)
{
	return platform_driver_register(&rtk_reboot_mode_driver);
}
module_init(rtk_reboot_mode_init);

static void __exit rtk_reboot_mode_exit(void)
{
	platform_driver_unregister(&rtk_reboot_mode_driver);
}
module_exit(rtk_reboot_mode_exit);


MODULE_DESCRIPTION("Reatek Reboot Mode Driver");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");

