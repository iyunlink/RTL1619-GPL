// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017,2019,2020 Realtek Semiconductor Corp.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/reset.h>

#define MIS_RTCSEC                      0x00
#define MIS_RTCMIN                      0x04
#define MIS_RTCHR                       0x08
#define MIS_RTCDATE1                    0x0c
#define MIS_RTCDATE2                    0x10
#define MIS_ALMMIN                      0x14
#define MIS_ALMHR                       0x18
#define MIS_ALMDATE1                    0x1c
#define MIS_ALMDATE2                    0x20
#define MIS_RTCSTOP                     0x24
#define MIS_RTCACR                      0x28
#define MIS_RTCEN                       0x2c
#define MIS_RTCCR                       0x30
#define MIS_RTCACR2                     0x34

#define ISO_RTC                         0x34
#define ISO_RTC_ALARM_INT_EN            0x00000001
#define ISO_RTC_HSEC_INT_EN             0x00000002
#define ISO_RTC_HSEC_SYNC               0x00000004

#define MIS_DRTCCR_ARAINTE              0x00000010
#define MIS_DRTCCR_HSUINTE              0x00000001

enum {
	RTK_RTC_TIME,
	RTK_RTC_ALARM
};

struct rtk_rtc_config {
	int has_acr2 : 1;
	int is_drtc : 1;
};

struct rtk_rtc_device {
	struct rtc_device *rtc;
	struct device *dev;
	struct regmap *regmap;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rstc;
	time64_t base_time;
	int bias;
	spinlock_t lock;
	const struct rtk_rtc_config *cfg;
};

static inline unsigned int rtk_rtc_reg_read(struct rtk_rtc_device *rdev, unsigned int offset)
{
	unsigned int val = readl(rdev->base + offset);

	dev_dbg(rdev->dev, "%s: rtc+%02x: %08x\n", __func__, offset, val);
	return val;
}

static inline void rtk_rtc_reg_write(struct rtk_rtc_device *rdev, unsigned int offset, unsigned int val)
{
	dev_dbg(rdev->dev, "%s: rtc+%02x: %08x\n", __func__, offset, val);
	writel(val, rdev->base + offset);
}

static inline void rtk_rtc_show_time(struct rtk_rtc_device *rdev, const char *log, struct rtc_time *tm)
{
	dev_dbg(rdev->dev, "%s: %04d.%02d.%02d %02d:%02d:%02d", log,
		1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour,
		tm->tm_min, tm->tm_sec);
}

static void rtk_rtc_reg_to_time(struct rtk_rtc_device *rdev, int type, time64_t *time)
{
	unsigned long flags;
	unsigned int day, hour, minute, second = 0;

	spin_lock_irqsave(&rdev->lock, flags);

	switch (type) {
	case RTK_RTC_TIME:
		second = rtk_rtc_reg_read(rdev, MIS_RTCSEC) >> 1;
		minute = rtk_rtc_reg_read(rdev, MIS_RTCMIN);
		hour   = rtk_rtc_reg_read(rdev, MIS_RTCHR);
		day    = (rtk_rtc_reg_read(rdev, MIS_RTCDATE1) & 0xff) | (rtk_rtc_reg_read(rdev, MIS_RTCDATE2) << 8);
		break;

	case RTK_RTC_ALARM:
		minute = rtk_rtc_reg_read(rdev, MIS_ALMMIN);
		hour   = rtk_rtc_reg_read(rdev, MIS_ALMHR);
		day    = (rtk_rtc_reg_read(rdev, MIS_ALMDATE1) & 0xff) | (rtk_rtc_reg_read(rdev, MIS_ALMDATE2) << 8);
		break;
	}

	spin_unlock_irqrestore(&rdev->lock, flags);

	*time = rdev->base_time + day * 86400 + hour * 3600 + minute * 60 + second;
}

static int rtk_rtc_time_to_reg(struct rtk_rtc_device *rdev, int type, time64_t time)
{
	unsigned long flags;
	unsigned int day, hour, minute, second;

	if (time < rdev->base_time)
		return -EINVAL;

	time -= rdev->base_time;
	second = (time % 60);
	minute = (time / 60) % 60;
	hour   = (time / 3600) % 24;
	day    = (time / 86400);

	if (day > 0x3fff)
		return -EINVAL;

	spin_lock_irqsave(&rdev->lock, flags);

	switch (type) {
	case RTK_RTC_TIME:
		rtk_rtc_reg_write(rdev, MIS_RTCSEC,   second << 1);
		rtk_rtc_reg_write(rdev, MIS_RTCMIN,   minute);
		rtk_rtc_reg_write(rdev, MIS_RTCHR,    hour);
		rtk_rtc_reg_write(rdev, MIS_RTCDATE1, day & 0xff);
		rtk_rtc_reg_write(rdev, MIS_RTCDATE2, day >> 8);
		break;

	case RTK_RTC_ALARM:
		rtk_rtc_reg_write(rdev, MIS_ALMMIN,   minute);
		rtk_rtc_reg_write(rdev, MIS_ALMHR,    hour);
		rtk_rtc_reg_write(rdev, MIS_ALMDATE1, day & 0xff);
		rtk_rtc_reg_write(rdev, MIS_ALMDATE2, day >> 8);
		break;
	}

	spin_unlock_irqrestore(&rdev->lock, flags);

	return 0;
}

static void __rtk_rtc_enable(struct rtk_rtc_device *rdev)
{
	rtk_rtc_reg_write(rdev, MIS_RTCEN, 0x5a);
}

static void rtk_rtc_init(struct rtk_rtc_device *rdev)
{
	unsigned long flags;
	unsigned int val;

	spin_lock_irqsave(&rdev->lock, flags);

	if (rdev->cfg->is_drtc) {
		val = rtk_rtc_reg_read(rdev, MIS_RTCEN);
		if (val == 0xa5)
			goto done;
	} else {
		val = rtk_rtc_reg_read(rdev, MIS_RTCACR);
		if ((val & 0x80) == 0x80)
			goto done;
		rtk_rtc_reg_write(rdev, MIS_RTCACR, 0x80);
	}

	rtk_rtc_reg_write(rdev, MIS_RTCCR, 0x40);
	rtk_rtc_reg_write(rdev, MIS_RTCCR, 0x00);

	rtk_rtc_reg_write(rdev, MIS_RTCSEC,   0);
	rtk_rtc_reg_write(rdev, MIS_RTCMIN,   0);
	rtk_rtc_reg_write(rdev, MIS_RTCHR,    0);
	rtk_rtc_reg_write(rdev, MIS_RTCDATE1, 0);
	rtk_rtc_reg_write(rdev, MIS_RTCDATE2, 0);

	if (rdev->cfg->has_acr2)
		rtk_rtc_reg_write(rdev, MIS_RTCACR2, rdev->bias);

	__rtk_rtc_enable(rdev);
done:
	spin_unlock_irqrestore(&rdev->lock, flags);
}

static int rtk_rtc_get_drtc_alarm_inte(struct rtk_rtc_device *rdev)
{
	unsigned long flags;
	unsigned int val;

	spin_lock_irqsave(&rdev->lock, flags);
	val = rtk_rtc_reg_read(rdev, MIS_RTCCR);
	spin_unlock_irqrestore(&rdev->lock, flags);

	return !!(val & MIS_DRTCCR_ARAINTE);
}

static void rtk_rtc_set_drtc_alarm_inte(struct rtk_rtc_device *rdev, int enabled)
{
	unsigned long flags;
	unsigned int val;

	spin_lock_irqsave(&rdev->lock, flags);
	val = rtk_rtc_reg_read(rdev, MIS_RTCCR);
	if (enabled)
		val |= MIS_DRTCCR_ARAINTE;
	else
		val &= ~MIS_DRTCCR_ARAINTE;
	rtk_rtc_reg_write(rdev, MIS_RTCCR, val);
	spin_unlock_irqrestore(&rdev->lock, flags);
}

static int rtk_rtc_get_alarm_inte(struct rtk_rtc_device *rdev)
{
	unsigned int val;

	regmap_read(rdev->regmap, ISO_RTC, &val);
	return !!(val & ISO_RTC_ALARM_INT_EN);
}

static void rtk_rtc_set_alarm_inte(struct rtk_rtc_device *rdev, int enabled)
{
	unsigned int mask = ISO_RTC_ALARM_INT_EN;

	regmap_update_bits(rdev->regmap, ISO_RTC, mask, enabled ? mask : 0);
}

static int rtk_rtc_alarm_interrupt_enabled(struct rtk_rtc_device *rdev)
{
	if (rdev->cfg->is_drtc)
		return rtk_rtc_get_drtc_alarm_inte(rdev);

	return rtk_rtc_get_alarm_inte(rdev);
}

static void rtk_rtc_alarm_interrupt_enable(struct rtk_rtc_device *rdev)
{
	if (rdev->cfg->is_drtc)
		rtk_rtc_set_drtc_alarm_inte(rdev, 1);
	else
		rtk_rtc_set_alarm_inte(rdev, 1);
}

static void rtk_rtc_alarm_interrupt_disable(struct rtk_rtc_device *rdev)
{
	if (rdev->cfg->is_drtc)
		rtk_rtc_set_drtc_alarm_inte(rdev, 0);
	else
		rtk_rtc_set_alarm_inte(rdev, 0);
}

static int rtk_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtk_rtc_device *rdev = dev_get_drvdata(dev);
	time64_t time;

	rtk_rtc_reg_to_time(rdev, RTK_RTC_TIME, &time);

	rtc_time64_to_tm(time, tm);

	rtk_rtc_show_time(rdev, __func__, tm);

	return rtc_valid_tm(tm);
}

static int rtk_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct rtk_rtc_device *rdev = dev_get_drvdata(dev);

	rtk_rtc_show_time(rdev, __func__, tm);

	return rtk_rtc_time_to_reg(rdev, RTK_RTC_TIME, rtc_tm_to_time64(tm));
}

static int rtk_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtk_rtc_device *rdev = dev_get_drvdata(dev);
	time64_t time;

	rtk_rtc_reg_to_time(rdev, RTK_RTC_ALARM, &time);

	rtc_time64_to_tm(time, &alrm->time);

	alrm->enabled = rtk_rtc_alarm_interrupt_enabled(rdev);

	rtk_rtc_show_time(rdev, __func__, &alrm->time);

	return 0;
}

static int rtk_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtk_rtc_device *rdev = dev_get_drvdata(dev);
	int ret = 0;
	time64_t now, alarm_time;

	rtk_rtc_alarm_interrupt_disable(rdev);

	alarm_time = rtc_tm_to_time64(&alrm->time);
	if (alrm->time.tm_sec) {
		alarm_time += 60 - alrm->time.tm_sec;
	}

	rtk_rtc_reg_to_time(rdev, RTK_RTC_TIME, &now);
	if (now > alarm_time)
		return -EINVAL;

	ret = rtk_rtc_time_to_reg(rdev, RTK_RTC_ALARM, alarm_time);
	if (!ret && alrm->enabled)
		rtk_rtc_alarm_interrupt_enable(rdev);

	rtk_rtc_show_time(rdev, __func__, &alrm->time);

	return ret;
}

static int rtk_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct rtk_rtc_device *rdev = dev_get_drvdata(dev);

	if (enabled)
		rtk_rtc_alarm_interrupt_enable(rdev);
	else
		rtk_rtc_alarm_interrupt_disable(rdev);
	return 0;
}

static const struct rtc_class_ops rtk_rtc_alarm_ops = {
	.read_time        = rtk_rtc_read_time,
	.set_time         = rtk_rtc_set_time,
	.read_alarm       = rtk_rtc_read_alarm,
	.set_alarm        = rtk_rtc_set_alarm,
	.alarm_irq_enable = rtk_rtc_alarm_irq_enable,
};

static const struct rtc_class_ops rtk_rtc_ops = {
	.read_time = rtk_rtc_read_time,
	.set_time  = rtk_rtc_set_time,
};

static irqreturn_t rtk_rtc_alarm_irq_handler(int irq, void *data)
{
	struct platform_device *pdev = data;
	struct rtk_rtc_device *rdev = platform_get_drvdata(pdev);

	dev_dbg(rdev->dev, "%s\n", __func__);
	rtc_update_irq(rdev->rtc, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static int rtk_rtc_parse_dt_config(struct device_node *np, struct rtk_rtc_device *rdev)
{
	unsigned int base_year = 1900;

	of_property_read_u32(np, "rtc-base-year", &base_year);
	dev_info(rdev->dev, "use %d as base year\n", base_year);

	rdev->base_time = mktime64(base_year, 1, 1, 0, 0, 0);

	if (!rdev->cfg->has_acr2)
		return 0;

	rdev->bias = 2;
	of_property_read_u32(np, "rtc-bias", &rdev->bias);
	return 0;
}

static int rtk_rtc_probe(struct platform_device *pdev)
{
	struct rtk_rtc_device *rdev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct rtc_class_ops *ops = &rtk_rtc_ops;
	int ret;
	struct resource *res;
	int irq;

	rdev = devm_kzalloc(dev, sizeof(*rdev), GFP_KERNEL);
	if (!rdev)
		return -ENOMEM;

	rdev->cfg = of_device_get_match_data(dev);
	if (!rdev->cfg) {
		dev_err(dev, "failed to get match data\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "invalid resource\n");
		return -EINVAL;
	}

	rdev->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!rdev->base)
		return -ENOMEM;

	if (!rdev->cfg->is_drtc) {
		rdev->regmap = syscon_regmap_lookup_by_phandle(np, "realtek,iso");
		if (IS_ERR(rdev->regmap)) {
			ret = PTR_ERR(rdev->regmap);
			dev_err(dev, "failed to get syscon: %d\n", ret);
			return ret;
		}
	}

	rdev->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(rdev->clk)) {
		ret = PTR_ERR(rdev->clk);
		dev_err(dev, "failed to get clk: %d\n", ret);
		return ret;
	}

	rdev->rstc = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(rdev->rstc)) {
		ret = PTR_ERR(rdev->rstc);
		dev_err(dev, "failed to get reset control: %d\n", ret);
		return ret;
	}

	rdev->dev = dev;
	rtk_rtc_parse_dt_config(np, rdev);
	spin_lock_init(&rdev->lock);

	irq = platform_get_irq(pdev, 0);
	if (irq > 0) {
		ret = devm_request_threaded_irq(dev, irq, NULL,	rtk_rtc_alarm_irq_handler,
						IRQF_ONESHOT, dev_name(dev), pdev);
		if (ret) {
			dev_err(dev, "failed to request irq%d: %d\n", irq, ret);
			return ret;
		}

		device_init_wakeup(&pdev->dev, true);
		ops = &rtk_rtc_alarm_ops;
	}

	platform_set_drvdata(pdev, rdev);
	reset_control_deassert(rdev->rstc);
	clk_prepare_enable(rdev->clk);
	rtk_rtc_init(rdev);

	rdev->rtc = devm_rtc_device_register(&pdev->dev, "rtk-rtc", ops, THIS_MODULE);
	if (IS_ERR(rdev->rtc)) {
		ret = PTR_ERR(rdev->rtc);
		dev_err(dev, "cannot attach rtc: %d\n", ret);
		goto err_nortc;
	}

	rdev->rtc->uie_unsupported = 1;
	return 0;

err_nortc:
	clk_disable_unprepare(rdev->clk);
	reset_control_assert(rdev->rstc);
	return ret;
}

static int rtk_rtc_remove(struct platform_device *pdev)
{
	struct rtk_rtc_device *rdev = platform_get_drvdata(pdev);

	clk_disable_unprepare(rdev->clk);
	reset_control_assert(rdev->rstc);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct rtk_rtc_config rtd1195_cfg = { 0, 0 };
static const struct rtk_rtc_config rtd1619_cfg = { 1, 0 };
static const struct rtk_rtc_config rtd1319d_cfg = { 0, 1 };

static const struct of_device_id rtk_rtc_ids[] = {
	{ .compatible = "realtek,rtd1195-rtc", .data = &rtd1195_cfg, },
	{ .compatible = "realtek,rtd1295-rtc", .data = &rtd1195_cfg, },
	{ .compatible = "realtek,rtd1619-rtc", .data = &rtd1619_cfg, },
	{ .compatible = "realtek,rtd1319-rtc", .data = &rtd1619_cfg, },
	{ .compatible = "realtek,rtd1319d-rtc", .data = &rtd1319d_cfg, },
	{}
};
MODULE_DEVICE_TABLE(of, rtk_rtc_ids);

static struct platform_driver rtk_rtc_driver = {
	.probe  = rtk_rtc_probe,
	.remove = rtk_rtc_remove,
	.driver = {
		.name = "rtk-rtc",
		.of_match_table = rtk_rtc_ids,
	},
};
module_platform_driver(rtk_rtc_driver);

MODULE_DESCRIPTION("Realtek Real-time Clock Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
