// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Realtek Semiconductor Corp.
 */

#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

enum {
	REBOOT_REASON_INVALID = 0,
	REBOOT_REASON_SOFTWARE,
	REBOOT_REASON_WATCHDOG,
	REBOOT_REASON_SUSPEND_ABNORMAL,
	REBOOT_REASON_SHUTDOWN_ABNORMAL,
};

static const char * const reboot_reason_strings[] = {
	"invalid", "software", "watchdog", "suspend_error", "shutdown_error",
};

enum {
	BOOT_TYPE_EMMC,
	BOOT_TYPE_USB_DEVICE,
	BOOT_TYPE_NOR,
	BOOT_TYPE_NAND,
	BOOT_TYPE_NAND_SERIAL,
	BOOT_TYPE_NAND_SERIAL_3V3,
	BOOT_TYPE_NAND_SERIAL_1V8,
	BOOT_TYPE_NAND_PARALLEL,
};

static const char * const boot_type_strings[] = {
	[BOOT_TYPE_EMMC]            = "emmc",
	[BOOT_TYPE_USB_DEVICE]      = "usb-device",
	[BOOT_TYPE_NOR]             = "nor",
	[BOOT_TYPE_NAND]            = "nand",
	[BOOT_TYPE_NAND_SERIAL]     = "nand-serial",
	[BOOT_TYPE_NAND_SERIAL_3V3] = "nand-serial-3v3",
	[BOOT_TYPE_NAND_SERIAL_1V8] = "nand-serial-1v8",
	[BOOT_TYPE_NAND_PARALLEL]   = "nand-parellel",
};

struct bootstatus_plat_desc {
	const char *(*boot_type_string)(int opt);
};

struct bootstatus_data {
	const struct bootstatus_plat_desc *desc;
	struct regmap *regmap;
};

static const char *bootstatus_get_boot_type(struct bootstatus_data *data)
{
	unsigned int val;

	regmap_read(data->regmap, 0x678, &val);

	if (!data->desc || !data->desc->boot_type_string)
		return "invalid";

	return data->desc->boot_type_string(val);
}

static const unsigned int rtd1619_boot_type_map[] = {
	BOOT_TYPE_NAND, BOOT_TYPE_NOR, BOOT_TYPE_USB_DEVICE, BOOT_TYPE_EMMC
};

static const char *rtd1619_boot_type_string(int opt)
{
	int sel = (opt >> 29) & 0x3;

	return boot_type_strings[rtd1619_boot_type_map[sel]];
}

static const struct bootstatus_plat_desc rtd1619_desc = {
	.boot_type_string = rtd1619_boot_type_string,
};

static const unsigned int rtd1319_boot_type_map[] = {
	BOOT_TYPE_NAND_SERIAL, BOOT_TYPE_NAND_PARALLEL,
	BOOT_TYPE_NOR,         BOOT_TYPE_NOR,
	BOOT_TYPE_USB_DEVICE,  BOOT_TYPE_USB_DEVICE,
	BOOT_TYPE_EMMC,        BOOT_TYPE_EMMC
};

static const char *rtd1319_boot_type_string(int opt)
{
	int sel = (opt >> 28) & 0x7;

	return boot_type_strings[rtd1319_boot_type_map[sel]];
}

static const struct bootstatus_plat_desc rtd1319_desc = {
	.boot_type_string = rtd1319_boot_type_string,
};

static const unsigned int rtd1619b_boot_type_map[] = {
	BOOT_TYPE_NAND_SERIAL_1V8, BOOT_TYPE_NAND_SERIAL_3V3,
	BOOT_TYPE_NOR,             BOOT_TYPE_NAND_PARALLEL,
	BOOT_TYPE_USB_DEVICE,      BOOT_TYPE_USB_DEVICE,
	BOOT_TYPE_EMMC,            BOOT_TYPE_EMMC
};

static const char *rtd1619b_boot_type_string(int opt)
{
	int sel = (opt >> 28) & 0x7;

	return boot_type_strings[rtd1619b_boot_type_map[sel]];
}

static const struct bootstatus_plat_desc rtd1619b_desc = {
	.boot_type_string = rtd1619b_boot_type_string,
};

static struct bootstatus_data *bootstatus_data;
static long reboot_reason;

static int reboot_reason_config(char *str)
{
	if (!str)
		return -EINVAL;

	if (strcmp(str, "hardware") == 0)  // off  -> on
		reboot_reason = REBOOT_REASON_INVALID;
	else if (strcmp(str, "str_warm") == 0) // suspend_to_ram -> on
		reboot_reason = REBOOT_REASON_SUSPEND_ABNORMAL;
	else if (strcmp(str, "str_cold") == 0) // shutdown -> on
		reboot_reason = REBOOT_REASON_SHUTDOWN_ABNORMAL;
	else if (strcmp(str, "software") == 0) // reboot -> on
		reboot_reason = REBOOT_REASON_SOFTWARE;
	else if (strcmp(str, "watchdog") == 0) // reboot_trigger_by_watchdog -> on
		reboot_reason = REBOOT_REASON_WATCHDOG;
	else
		pr_err("%s: invalid wakeupreason value \"%s\"\n", __func__, str);
	return 0;
}
__setup("wakeupreason=", reboot_reason_config);

static ssize_t reboot_reason_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", reboot_reason_strings[reboot_reason]);
}

static struct kobj_attribute reboot_reason_attr = __ATTR_RO(reboot_reason);

static ssize_t boot_type_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", bootstatus_get_boot_type(bootstatus_data));
}

static struct kobj_attribute boot_type_attr = __ATTR_RO(boot_type);

static ssize_t sw_cold_boot_counter_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unsigned int val;

	regmap_read(bootstatus_data->regmap, 0x644, &val);
	return sprintf(buf, "%u\n", val);
}

static ssize_t sw_cold_boot_counter_store(struct kobject *kobj, struct kobj_attribute *attr,
					  const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = kstrtouint(buf, 0, &val);
	if (!ret)
		regmap_write(bootstatus_data->regmap, 0x644, val);

	return ret ?: count;
}

static struct kobj_attribute sw_cold_boot_counter_attr = __ATTR_RW(sw_cold_boot_counter);

static struct attribute *rtk_bootstatus_attrs[] = {
	&reboot_reason_attr.attr,
	&boot_type_attr.attr,
	&sw_cold_boot_counter_attr.attr,
	NULL,
};

static umode_t rtk_bootstatus_attr_is_visible(struct kobject *kobj, struct attribute *attr, int unused)
{
	if (attr == &boot_type_attr.attr || attr == &sw_cold_boot_counter_attr.attr)
		return bootstatus_data ? attr->mode : 0;
	return attr->mode;
}

static struct attribute_group rtk_bootstatus_attr_group = {
	.attrs      = rtk_bootstatus_attrs,
	.is_visible = rtk_bootstatus_attr_is_visible,
};

static const struct of_device_id machines[] __initconst = {
	{ .compatible = "realtek,rtd1619", .data = &rtd1619_desc, },
	{ .compatible = "realtek,rtd1319", .data = &rtd1319_desc, },
	{ .compatible = "realtek,rtd1312c", .data = &rtd1319_desc, },
	{ .compatible = "realtek,rtd1619b", .data = &rtd1619b_desc, },
	{ .compatible = "realtek,rtd1315c", .data = &rtd1619b_desc, },
	{ .compatible = "realtek,rtd1319d", .data = &rtd1619b_desc, },
	{}
};

static int platform_init(struct bootstatus_data *data)
{
	struct device_node *np = of_find_node_by_path("/");
	const struct of_device_id *match;

	if (!np)
		return -ENODEV;

	match = of_match_node(machines, np);
	of_node_put(np);
	if (!match)
		return -ENODEV;

	data->desc = match->data;
	return 0;
}

static int syscon_init(struct bootstatus_data *data)
{
	struct device_node *np;

	np = of_find_node_by_path("/iso@98007000");
	if (!np)
		np = of_find_node_by_path("/soc@0/rbus@98000000/syscon@7000");
	if (!np)
		return -ENODEV;

	data->regmap = syscon_node_to_regmap(np);
	of_node_put(np);

	return PTR_ERR_OR_ZERO(data->regmap);
}

static int bootstatus_data_init(void)
{
	int ret;

	bootstatus_data = kzalloc(sizeof(*bootstatus_data), GFP_KERNEL);
	if (!bootstatus_data)
		return -ENOMEM;

	ret = platform_init(bootstatus_data);
	if (ret)
		goto free_mem;

	ret = syscon_init(bootstatus_data);
	if (ret)
		goto free_mem;

	return 0;

free_mem:
	kfree(bootstatus_data);
	bootstatus_data = NULL;
	return ret;
}

static void bootstatus_data_exit(void)
{
	kfree(bootstatus_data);
}

struct kobject *rtk_bootstatus_kobj;

static int __init rtk_bootstatus_init(void)
{
	int ret = 0;

	ret = bootstatus_data_init();
	WARN_ON(ret);

	rtk_bootstatus_kobj = kobject_create_and_add("bootstatus", kernel_kobj);
	if (!rtk_bootstatus_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(rtk_bootstatus_kobj, &rtk_bootstatus_attr_group);
	if (ret) {
		kobject_put(rtk_bootstatus_kobj);
		bootstatus_data_exit();
	}

	return ret;
}
module_init(rtk_bootstatus_init);

static void __exit rtk_bootstatus_exit(void)
{
	sysfs_remove_group(rtk_bootstatus_kobj, &rtk_bootstatus_attr_group);
	kobject_put(rtk_bootstatus_kobj);
	bootstatus_data_exit();
}
module_exit(rtk_bootstatus_exit);
MODULE_DESCRIPTION("Realtek Bootstatus Sysfs");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
