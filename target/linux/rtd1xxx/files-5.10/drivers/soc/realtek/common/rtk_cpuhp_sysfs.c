// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Realtek Semiconductor Corp.
 */

#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <soc/realtek/rtk_cpuhp.h>

static struct kobject *rtk_cpuhp_sysfs_kobj;
static struct rtk_cpuhp_qos_request rtk_cpuhp_sysfs_request;
static unsigned int rtk_cpuhp_sysfs_request_val;

static int update_request(unsigned int val)
{
	int ret;

	ret = rtk_cpuhp_qos_update_request(&rtk_cpuhp_sysfs_request, val);;
	if (!ret)
		rtk_cpuhp_sysfs_request_val = val;
	return ret;
}

static ssize_t request_num_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", rtk_cpuhp_sysfs_request_val);
}

static ssize_t request_num_store(struct kobject *kobj, struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = kstrtouint(buf, 0, &val);
	if (!ret)
		ret = update_request(val);
	return ret ?: count;
}

static struct kobj_attribute request_num_attr = __ATTR_RW(request_num);

static struct attribute *rtk_cpuhp_sysfs_attrs[] = {
	&request_num_attr.attr,
	NULL,
};

static struct attribute_group rtk_cpuhp_sysfs_attr_group = {
	.attrs      = rtk_cpuhp_sysfs_attrs,
};

static int __init rtk_cpuhp_sysfs_init(void)
{
	int ret;

	rtk_cpuhp_sysfs_kobj = kobject_create_and_add("rtk_cpuhp", kernel_kobj);
	if (!rtk_cpuhp_sysfs_kobj)
		return -ENOMEM;

	ret = rtk_cpuhp_qos_add_request(&rtk_cpuhp_sysfs_request, 0);
	if (ret)
		goto put_kobj;

	ret = sysfs_create_group(rtk_cpuhp_sysfs_kobj, &rtk_cpuhp_sysfs_attr_group);
	if (ret)
		goto remove_qos_request;

	return 0;

remove_qos_request:
	rtk_cpuhp_qos_remove_request(&rtk_cpuhp_sysfs_request);
put_kobj:
	kobject_put(rtk_cpuhp_sysfs_kobj);
	return ret;

}
module_init(rtk_cpuhp_sysfs_init);


static void __exit rtk_cpuhp_sysfs_exit(void)
{
	sysfs_remove_group(rtk_cpuhp_sysfs_kobj, &rtk_cpuhp_sysfs_attr_group);
	rtk_cpuhp_qos_remove_request(&rtk_cpuhp_sysfs_request);
	kobject_put(rtk_cpuhp_sysfs_kobj);
}
module_exit(rtk_cpuhp_sysfs_exit);

MODULE_DESCRIPTION("Realtek CPUHP Sysfs");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
