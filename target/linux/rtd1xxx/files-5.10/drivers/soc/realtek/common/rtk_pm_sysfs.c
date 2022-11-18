// SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause)
/*
 * Realtek DHC SoC family power management
 * Copyright (c) 2020-2021 Realtek Semiconductor Corp.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/tick.h>
#include <linux/vmalloc.h>
#include <linux/regmap.h>
#include <soc/realtek/memory.h>
#include <soc/realtek/rtk_pm.h>

#include <asm/suspend.h>
#include <asm/system_misc.h>

const char *const rtk_pm_event_states[MAX_EVENT + 1] = {
	[LAN_EVENT]    = "lan",
	[IR_EVENT]     = "irda",
	[GPIO_EVENT]   = "gpio",
	[ALARM_EVENT]  = "alarm",
	[TIMER_EVENT]  = "timer",
	[CEC_EVENT]    = "cec",
	[USB_EVENT]    = "usb",
	[HIFI_EVENT]   = "hifi",
	[VTC_EVENT]    = "vtc",
	[PON_EVENT]    = "pon",
	[MAX_EVENT]    = "invalid",
};

#define RTK_PM_ATTR(_name) \
{ \
	.attr = {.name = #_name, .mode = 0644}, \
	.show =  rtk_pm_##_name##_show, \
	.store = rtk_pm_##_name##_store, \
}

static enum rtk_wakeup_event rtk_pm_decode_states(const char *buf, size_t n)
{
	const char *const *s;
	char *p;
	int len = 0;
	int i = 0;

	p = memchr(buf, '\n', n);
	len = p ? p - buf : n;

	for (i = 0; i < MAX_EVENT; i++) {
		s = &rtk_pm_event_states[i];
		if (*s && len == strlen(*s) && !strncmp(buf, *s, len))
			return i;
	}

	return MAX_EVENT;
}

static ssize_t rtk_pm_dco_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	struct pm_dev_param *pm_param = rtk_pm_get_param(PM);
	struct pm_private *dev_pm = dev_get_drvdata(pm_param->dev);
	char *str;

	if (dev_pm->pcpu_param->wakeup_source & DCO_ENABLE)
		str = "on";
	else
		str = "off";

	return sprintf(buf, "%s\n", str);
}

static ssize_t rtk_pm_dco_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	char *p;
	int len = 0;
	struct pm_dev_param *pm_param = rtk_pm_get_param(PM);
	struct pm_dev_param *lan_node = rtk_pm_get_param(LAN);
	struct pm_private *dev_pm = dev_get_drvdata(pm_param->dev);

	if (lan_node == NULL)
		goto err;

	p = memchr(buf, '\n', count);
	len = p ? p - buf : count;

	if (!strncmp(buf, "on", len)) {
		 *(int *)lan_node->data = DCO_ENABLE;
		dev_pm->pcpu_param->wakeup_source |= 0x1000;
	} else if (!strncmp(buf, "off", len)) {
		*(int *)lan_node->data = 0;
		dev_pm->pcpu_param->wakeup_source &= 0xffffefff;
		dev_pm->pcpu_param->wakeup_source |= 0x1;
	}

err:
	return count;
}

static ssize_t rtk_pm_wakeup_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	int i = 0;
	struct pm_dev_param *pm_param = rtk_pm_get_param(PM);
	struct pm_private *dev_pm = dev_get_drvdata(pm_param->dev);
	unsigned int n = 0;

	for (i = 0; i < MAX_EVENT; i++) {
		if (dev_pm->pcpu_param->wakeup_source & BIT(i))
			n += sprintf(buf + n, " * ");
		else
			n += sprintf(buf + n, "   ");

		n += sprintf(buf + n, "%s\n", rtk_pm_event_states[i]);
	}

	n += sprintf(buf + n, "\n");

	return n;
}

static ssize_t rtk_pm_wakeup_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	enum rtk_wakeup_event wakeup = rtk_pm_decode_states(buf, count);
	struct pm_dev_param *pm_param = rtk_pm_get_param(PM);
	struct pm_private *dev_pm = dev_get_drvdata(pm_param->dev);

	if (wakeup < MAX_EVENT) {
		dev_pm->pcpu_param->wakeup_source ^= BIT(wakeup);
		return count;
	}

	return -ENOMEM;
}

static ssize_t rtk_pm_timer_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct pm_dev_param *pm_param = rtk_pm_get_param(PM);
	struct pm_private *dev_pm = dev_get_drvdata(pm_param->dev);

	return sprintf(buf, " %d sec (reciprocal timer)\n",
		dev_pm->pcpu_param->timerout_val);
}

static ssize_t rtk_pm_timer_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	long val = 0;
	int ret = kstrtol(buf, 10, &val);
	struct pm_dev_param *pm_param = rtk_pm_get_param(PM);
	struct pm_private *dev_pm = dev_get_drvdata(pm_param->dev);

	if (ret < 0)
		return -ENOMEM;

	dev_pm->pcpu_param->timerout_val = val;
	return count;
}

static ssize_t rtk_pm_context_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	struct pm_dev_param *pm_param = rtk_pm_get_param(PM);
	struct pm_private *dev_pm = dev_get_drvdata(pm_param->dev);

	return sprintf(buf, "%d\n", dev_pm->suspend_context);
}

static ssize_t rtk_pm_context_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	long val;
	int ret = kstrtol(buf, 10, &val);
	struct pm_dev_param *pm_param = rtk_pm_get_param(PM);
	struct pm_private *dev_pm = dev_get_drvdata(pm_param->dev);

	if (ret < 0)
		return -ENOMEM;

	dev_pm->suspend_context = val;
	return count;
}

static ssize_t rtk_pm_reasons_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	struct pm_dev_param *pm_param = rtk_pm_get_param(PM);
	struct pm_private *dev_pm = dev_get_drvdata(pm_param->dev);

	return sprintf(buf, "%s\n", rtk_pm_event_states[dev_pm->wakeup_reason]);
}

static ssize_t rtk_pm_reasons_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	return count;
}

static ssize_t rtk_pm_gpio_en_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	struct pm_dev_param *pm_param = rtk_pm_get_param(PM);
	struct pm_private *dev_pm = dev_get_drvdata(pm_param->dev);
	int i = 0;
	int n = 0;

	n += sprintf(buf + n, "EN  | GPIO\n");
	n += sprintf(buf + n, "----+--------\n");

	for (i = 0 ; i < GPIO_MAX_SIZE ; i++) {
		if (dev_pm->pcpu_param->wu_gpio_en[i])
			n += sprintf(buf + n, "  * | %d\n", i);
		else
			n += sprintf(buf + n, "    | %d\n", i);
	}

	n += sprintf(buf + n, "\n");

	return n;
}

static ssize_t rtk_pm_gpio_en_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	return count;
}

static ssize_t rtk_pm_gpio_act_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	struct pm_dev_param *pm_param = rtk_pm_get_param(PM);
	struct pm_private *dev_pm = dev_get_drvdata(pm_param->dev);
	int i = 0;
	int n = 0;

	n += sprintf(buf + n, "ACT | GPIO\n");
	n += sprintf(buf + n, "----+--------\n");

	for (i = 0 ; i < GPIO_MAX_SIZE ; i++) {
		if (!dev_pm->pcpu_param->wu_gpio_en[i])
			n += sprintf(buf + n, "    | %d\n", i);
		else if (dev_pm->pcpu_param->wu_gpio_act[i])
			n += sprintf(buf + n, "  H | %d\n", i);
		else
			n += sprintf(buf + n, "  L | %d\n", i);
	}

	n += sprintf(buf + n, "\n");

	return n;
}

static ssize_t rtk_pm_gpio_act_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	return count;
}

static ssize_t rtk_pm_ir_key_show(struct kobject *kobj,
                                struct kobj_attribute *attr, char *buf)
{
       struct pm_dev_param *pm_param = rtk_pm_get_param(PM);
       struct pm_private *dev_pm = dev_get_drvdata(pm_param->dev);
       unsigned int ir_key;

       regmap_read(dev_pm->syscon_iso, 0x65c, &ir_key);

       return sprintf(buf, "%x\n", ir_key);
}

static ssize_t rtk_pm_ir_key_store(struct kobject *kobj,
                                 struct kobj_attribute *attr,
                                 const char *buf, size_t count)
{
       return count;
}


static struct kobj_attribute rtk_pm_dco_attr =
	RTK_PM_ATTR(dco);
static struct kobj_attribute rtk_pm_wakeup_attr =
	RTK_PM_ATTR(wakeup);
static struct kobj_attribute rtk_pm_timer_attr =
	RTK_PM_ATTR(timer);
static struct kobj_attribute rtk_pm_context_attr =
	RTK_PM_ATTR(context);
static struct kobj_attribute rtk_pm_reasons_attr =
	RTK_PM_ATTR(reasons);
static struct kobj_attribute rtk_pm_gpio_en_attr =
	RTK_PM_ATTR(gpio_en);
static struct kobj_attribute rtk_pm_gpio_act_attr =
	RTK_PM_ATTR(gpio_act);
static struct kobj_attribute rtk_pm_ir_key_attr =
       RTK_PM_ATTR(ir_key);

static struct attribute *rtk_pm_attrs[] = {
	&rtk_pm_dco_attr.attr,
	&rtk_pm_wakeup_attr.attr,
	&rtk_pm_timer_attr.attr,
	&rtk_pm_context_attr.attr,
	&rtk_pm_reasons_attr.attr,
	&rtk_pm_gpio_en_attr.attr,
	&rtk_pm_gpio_act_attr.attr,
	&rtk_pm_ir_key_attr.attr,
	NULL,
};

static struct attribute_group rtk_pm_attr_group = {
	.attrs = rtk_pm_attrs,
};

int rtk_pm_create_sysfs(void)
{
	int ret = 0;
	struct kobject *rtk_pm_kobj;

	rtk_pm_kobj = kobject_create_and_add("suspend", kernel_kobj);
	if (!rtk_pm_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(rtk_pm_kobj, &rtk_pm_attr_group);
	if (ret)
		kobject_put(rtk_pm_kobj);

	return ret;
}
EXPORT_SYMBOL(rtk_pm_create_sysfs);

MODULE_AUTHOR("James Tai <james.tai@realtek.com>");
MODULE_DESCRIPTION("Realtek suspend sysfs");
MODULE_LICENSE("GPL v2");
