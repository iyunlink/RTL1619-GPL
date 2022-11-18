/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include "protected.h"
#include "linux/slab.h"

static RAW_NOTIFIER_HEAD(protected_notifier_list);

unsigned long get_protected_base(const struct RTK_PROTECTED_INFO *info)
{
	return info->mem.base;
}

unsigned long get_protected_limit(const struct RTK_PROTECTED_INFO *info)
{
	return info->mem.base + info->mem.size;
}

size_t get_protected_size(const struct RTK_PROTECTED_INFO * info)
{
	return info->mem.size;
}

void *get_protected_priv(const struct RTK_PROTECTED_INFO *info)
{
	return info->priv_virt;
}

void set_protected_range(struct RTK_PROTECTED_INFO *info, unsigned long base,
			 unsigned long limit)
{
	info->mem.base = base;
	info->mem.size = limit - base;
}

void set_protected_priv(struct RTK_PROTECTED_INFO *info, void *priv_virt)
{
	info->priv_virt = priv_virt;
}

void init_protected_info(struct RTK_PROTECTED_INFO *info)
{
	memset((void *)info, 0, sizeof(struct RTK_PROTECTED_INFO));
	INIT_LIST_HEAD(&info->list);
}

enum E_ION_NOTIFIER_PROTECTED_TYPE get_protected_type(const struct
						      RTK_PROTECTED_INFO *info)
{
	return info->mem.type;
}

void set_protected_type(struct RTK_PROTECTED_INFO *info,
			enum E_ION_NOTIFIER_PROTECTED_TYPE type)
{
	info->mem.type = type;
}

void init_protected_ext_info(struct RTK_PROTECTED_EXT_INFO *info)
{
	memset((void *)info, 0, sizeof(struct RTK_PROTECTED_EXT_INFO));
}

void set_protected_ext_priv(struct RTK_PROTECTED_EXT_INFO *info, void *priv_virt)
{
	info->priv_virt = priv_virt;
}

void *get_protected_ext_priv(struct RTK_PROTECTED_EXT_INFO *info)
{
	return info->priv_virt;
}

void set_protected_ext_info(struct RTK_PROTECTED_EXT_INFO *info, enum E_ION_NOTIFIER_PROTECTED_EXT ext,
		unsigned long base, unsigned long size, void * parent_priv)
{
	info->mem.ext   = ext;
	info->mem.base  = base;
	info->mem.size  = size;
	info->mem.parent_priv = parent_priv;
}

struct RTK_PROTECTED_EXT_INFO * create_protected_ext_info(enum E_ION_NOTIFIER_PROTECTED_EXT ext,
		unsigned long base, unsigned long size, struct RTK_PROTECTED_INFO * parent)
{
	struct RTK_PROTECTED_EXT_INFO * ret = NULL;
	do {
		struct ion_rtk_protected_ext_set config;
		struct RTK_PROTECTED_EXT_INFO * ext_info = NULL;
		ext_info = (struct RTK_PROTECTED_EXT_INFO *) kzalloc(sizeof(struct RTK_PROTECTED_EXT_INFO), GFP_KERNEL);
		if (!ext_info)
			break;
		init_protected_ext_info(ext_info);
		set_protected_ext_info(ext_info, ext, base, size, get_protected_priv(parent));
		memcpy(&config.mem, &ext_info->mem, sizeof(ext_info->mem));
		if (ion_rtk_protected_ext_set_notify(&config) != NOTIFY_OK) {
			kfree(ext_info);
			break;
		}
		set_protected_ext_priv(ext_info, config.priv_virt);
		ret = ext_info;
	} while (0);
	return ret;
}

int destroy_protected_ext_info(struct RTK_PROTECTED_EXT_INFO * info)
{
	int ret = -1;
	do {
		struct ion_rtk_protected_ext_unset config;

		if (!info)
			break;

		config.priv_virt = get_protected_ext_priv(info);
		if (ion_rtk_protected_ext_unset_notify(&config) != NOTIFY_OK) {
			kfree(info);
			break;
		}

		kfree(info);
		ret = 0;
	} while(0);
	return ret;
}

int ion_rtk_protected_notifier_register(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&protected_notifier_list, nb);
}

EXPORT_SYMBOL(ion_rtk_protected_notifier_register);

void ion_rtk_protected_notifier_unregister(struct notifier_block *nb)
{
	raw_notifier_chain_unregister(&protected_notifier_list, nb);
}

EXPORT_SYMBOL(ion_rtk_protected_notifier_unregister);

int ion_rtk_protected_create_notify(struct ion_rtk_protected_create_info *info)
{
	int ret = raw_notifier_call_chain(&protected_notifier_list,
					     PROTECTED_REGION_CREATE, info);
	return ret;
}

int ion_rtk_protected_change_notify(struct ion_rtk_protected_change_info *info)
{
	int ret = raw_notifier_call_chain(&protected_notifier_list,
					     PROTECTED_REGION_CHANGE, info);
	return ret;
}

int ion_rtk_protected_destroy_notify(struct ion_rtk_protected_destroy_info
				     *info)
{
	int ret = raw_notifier_call_chain(&protected_notifier_list,
					     PROTECTED_REGION_DESTROY, info);
	return ret;
}

int ion_rtk_protected_ext_set_notify(struct ion_rtk_protected_ext_set * config)
{
	int ret = raw_notifier_call_chain(&protected_notifier_list,
			PROTECTED_REGION_EXTENSION_SET, config);
	return ret;
}

int ion_rtk_protected_ext_unset_notify(struct ion_rtk_protected_ext_unset * config)
{
	int ret = raw_notifier_call_chain(&protected_notifier_list,
			PROTECTED_REGION_EXTENSION_UNSET, config);
	return ret;
}
