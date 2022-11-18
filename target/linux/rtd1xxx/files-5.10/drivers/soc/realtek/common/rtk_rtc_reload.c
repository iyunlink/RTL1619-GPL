// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/rtc.h>
#include <linux/slab.h>

#define RTC_TIMESTAMP_BEGIN_2000	946684800LL /* 2000-01-01 00:00:00 */

struct rtc_reload_device {
	struct device *dev;
	void *base;
	struct rtc_device *rtc;
	struct class_interface class_intf;
};

#define RTC_RELOAD_MAGIC           BIT(31)
#define RTC_RELOAD_PARITY_BIT      30

static int calculate_parity(uint64_t val)
{
	return bitmap_weight((void *)&val, 30) & 1;
}

static int reg_to_parity(uint32_t val)
{
	return (val >> RTC_RELOAD_PARITY_BIT) & 1;
}

static uint64_t reg_to_time(uint32_t val)
{
	return (val & ~0xc0000000) + RTC_TIMESTAMP_BEGIN_2000;
}

static uint32_t time_to_reg(uint64_t time)
{
	uint32_t val = time - RTC_TIMESTAMP_BEGIN_2000;

	val |= calculate_parity(val) << RTC_RELOAD_PARITY_BIT;
	val |= RTC_RELOAD_MAGIC;
	return val;
}

static void rtc_reload_restore_time(struct rtc_reload_device *rtc_reload)
{
	struct device *dev = rtc_reload->dev;
	struct rtc_time tm;
	uint32_t val;
	uint32_t boot_time = DIV_ROUND_UP((uint32_t)ktime_to_us(ktime_get()),
					  USEC_PER_SEC) + 1;
	uint64_t reboot_time;

	val = readl(rtc_reload->base);

	if (!(RTC_RELOAD_MAGIC & val)) {
		dev_warn(dev, "invaid magic bit, raw=%08x\n", val);
		return;
	}

	if (reg_to_parity(val) != calculate_parity(val)) {
		dev_warn(dev, "invalid parity, raw=%08x\n", val);
		return;
	}

	reboot_time = reg_to_time(val);
	rtc_time64_to_tm(reboot_time + boot_time, &tm);
	dev_info(rtc_reload->dev,
		"restore system clock to %d-%02d-%02d %02d:%02d:%02d UTC (%lld)\n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
		tm.tm_min, tm.tm_sec, reboot_time);

	rtc_set_time(rtc_reload->rtc, &tm);
}

static void rtc_reload_save_time(struct rtc_reload_device *rtc_reload)
{
	struct rtc_time tm;
	uint64_t val;

	if (!rtc_reload->rtc) {
		dev_warn(rtc_reload->dev, "rtc0 not ready, ignored\n");
		return;
	}

	rtc_read_time(rtc_reload->rtc, &tm);
	dev_info(rtc_reload->dev,
		"save current system clock: %d-%02d-%02d %02d:%02d:%02d UTC\n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
		tm.tm_min, tm.tm_sec);

	val = time_to_reg(rtc_tm_to_time64(&tm));
	writel(val, rtc_reload->base);
}

static int rtc_reload_add_dev(struct device *dev, struct class_interface *class_intf)
{
	struct rtc_reload_device *rtc_reload = container_of(class_intf, struct rtc_reload_device, class_intf);

	if (!device_match_name(dev, "rtc0"))
		return 0;

	if (rtc_reload->rtc)
		return 0;

	rtc_reload->rtc = rtc_class_open("rtc0");
	if (!rtc_reload->rtc) {
		dev_err(rtc_reload->dev, "rtc0 is not a rtc device\n");
		return -EINVAL;
	}
	rtc_reload_restore_time(rtc_reload);
	return 0;
}

static void rtc_reload_remove_dev(struct device *dev, struct class_interface *class_intf)
{
	struct rtc_reload_device *rtc_reload = container_of(class_intf, struct rtc_reload_device, class_intf);

	if (!device_match_name(dev, "rtc0"))
		return;

	if (!rtc_reload->rtc)
		return;

	rtc_class_close(rtc_reload->rtc);
}

static int rtc_reload_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rtc_reload_device *rtc_reload;
	int ret;

	rtc_reload = devm_kzalloc(dev, sizeof(*rtc_reload), GFP_KERNEL);
	if (!rtc_reload)
		return -ENOMEM;
	rtc_reload->dev = dev;

	rtc_reload->base = of_iomap(np, 0);
	if (!rtc_reload->base)
		return -ENOMEM;

	platform_set_drvdata(pdev, rtc_reload);
	rtc_reload->class_intf.class = rtc_class;
	rtc_reload->class_intf.add_dev = rtc_reload_add_dev;
	rtc_reload->class_intf.remove_dev = rtc_reload_remove_dev;

	ret = class_interface_register(&rtc_reload->class_intf);
	if (ret) {
		dev_err(dev, "failed to register class interface: %d\n", ret);
		platform_set_drvdata(pdev, NULL);
		iounmap(rtc_reload->base);
		return ret;
	}

	return 0;
}

static int rtc_reload_remove(struct platform_device *pdev)
{
	struct rtc_reload_device *rtc_reload = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	iounmap(rtc_reload->base);
	class_interface_unregister(&rtc_reload->class_intf);
	return 0;
}

static void rtc_reload_shutdown(struct platform_device *pdev)
{
	struct rtc_reload_device *rtc_reload = platform_get_drvdata(pdev);

	rtc_reload_save_time(rtc_reload);
}

static const struct of_device_id rtc_reload_match[] = {
	{ .compatible = "realtek,rtc-reload" },
	{}
};

static struct platform_driver rtc_reload_driver = {
	.probe    = rtc_reload_probe,
	.remove   = rtc_reload_remove,
	.shutdown = rtc_reload_shutdown,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-rtc-reload",
		.of_match_table = of_match_ptr(rtc_reload_match),
	},
};
module_platform_driver(rtc_reload_driver);

MODULE_DESCRIPTION("Realtek RTC Reload driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-rtc-reload");
