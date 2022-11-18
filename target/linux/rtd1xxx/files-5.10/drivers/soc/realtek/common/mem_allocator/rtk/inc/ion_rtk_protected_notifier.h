/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef _LINUX_ION_RTK_PROTECTED_NOTIFIER_H
#define _LINUX_ION_RTK_PROTECTED_NOTIFIER_H

#include <linux/types.h>
#include <linux/notifier.h>

enum E_ION_NOTIFIER_PROTECTED_TYPE {
	ION_NOTIFIER_PROTECTED_TYPE_NONE = 0,
	ION_NOTIFIER_PROTECTED_TYPE_1,
	ION_NOTIFIER_PROTECTED_TYPE_2,
	ION_NOTIFIER_PROTECTED_TYPE_3,
	ION_NOTIFIER_PROTECTED_TYPE_4,
	ION_NOTIFIER_PROTECTED_TYPE_5,
	ION_NOTIFIER_PROTECTED_TYPE_6,
	ION_NOTIFIER_PROTECTED_TYPE_7,
	ION_NOTIFIER_PROTECTED_TYPE_8,
	ION_NOTIFIER_PROTECTED_TYPE_9,
	ION_NOTIFIER_PROTECTED_TYPE_10,
	ION_NOTIFIER_PROTECTED_TYPE_11,
	ION_NOTIFIER_PROTECTED_TYPE_12,
	ION_NOTIFIER_PROTECTED_TYPE_13,
	ION_NOTIFIER_PROTECTED_TYPE_14,
	ION_NOTIFIER_PROTECTED_TYPE_15,
	ION_NOTIFIER_PROTECTED_TYPE_MAX,
};

enum E_ION_NOTIFIER_PROTECTED_EXT {
	ION_NOTIFIER_PROTECTED_EXT_NONE = 0,
	ION_NOTIFIER_PROTECTED_EXT_1,
	ION_NOTIFIER_PROTECTED_EXT_2,
	ION_NOTIFIER_PROTECTED_EXT_3,
	ION_NOTIFIER_PROTECTED_EXT_4,
	ION_NOTIFIER_PROTECTED_EXT_5,
	ION_NOTIFIER_PROTECTED_EXT_6,
	ION_NOTIFIER_PROTECTED_EXT_7,
	ION_NOTIFIER_PROTECTED_EXT_MAX,
};

struct protected_region {
	enum E_ION_NOTIFIER_PROTECTED_TYPE type;
	unsigned long base;
	size_t size;
};

struct protected_ext_region {
	enum E_ION_NOTIFIER_PROTECTED_EXT ext;
	unsigned long base;
	size_t size;
	void *parent_priv;
};

struct ion_rtk_protected_create_info {
	struct protected_region mem;	// info to TEE
	void *priv_virt;	// write by TEE
};

struct ion_rtk_protected_change_info {
	struct protected_region mem;
	void *priv_virt;
};

struct ion_rtk_protected_destroy_info {
	void *priv_virt;
};

struct ion_rtk_protected_ext_set {
	struct protected_ext_region mem;
	void *priv_virt;
};

struct ion_rtk_protected_ext_unset {
	void *priv_virt;
};

enum ion_rtk_protected_notify_cmd {
	PROTECTED_REGION_CREATE,
	PROTECTED_REGION_CHANGE,
	PROTECTED_REGION_DESTROY,
	PROTECTED_REGION_EXTENSION_SET,
	PROTECTED_REGION_EXTENSION_UNSET,
};

int ion_rtk_protected_notifier_register(struct notifier_block *nb);
void ion_rtk_protected_notifier_unregister(struct notifier_block *nb);
#endif /* _LINUX_ION_RTK_PROTECTED_NOTIFIER_H */
