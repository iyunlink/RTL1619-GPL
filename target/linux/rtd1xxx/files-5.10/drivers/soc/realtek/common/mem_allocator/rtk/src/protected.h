/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef _LINUX_ION_RTK_CARVEOUT_HEAP_PROTECTED_H
#define _LINUX_ION_RTK_CARVEOUT_HEAP_PROTECTED_H

#include <linux/types.h>
#include "../inc/ion_rtk_protected_notifier.h"

struct RTK_PROTECTED_INFO {
	struct protected_region mem;
	struct list_head list;
	void *priv_virt;
};

struct RTK_PROTECTED_EXT_INFO {
	struct protected_ext_region mem;
	void * priv_virt;
};

unsigned long get_protected_base(const struct RTK_PROTECTED_INFO *info);
unsigned long get_protected_limit(const struct RTK_PROTECTED_INFO *info);
size_t get_protected_size(const struct RTK_PROTECTED_INFO *info);
void *get_protected_priv(const struct RTK_PROTECTED_INFO *info);
void set_protected_range(struct RTK_PROTECTED_INFO *info, unsigned long base,
			 unsigned long limit);
void set_protected_priv(struct RTK_PROTECTED_INFO *info, void *priv_virt);
void init_protected_info(struct RTK_PROTECTED_INFO *info);

enum E_ION_NOTIFIER_PROTECTED_TYPE get_protected_type(const struct
						      RTK_PROTECTED_INFO *info);
void set_protected_type(struct RTK_PROTECTED_INFO *info,
			enum E_ION_NOTIFIER_PROTECTED_TYPE type);

void init_protected_ext_info(struct RTK_PROTECTED_EXT_INFO *info);
void set_protected_ext_priv(struct RTK_PROTECTED_EXT_INFO *info, void *priv_virt);
void *get_protected_ext_priv(struct RTK_PROTECTED_EXT_INFO *info);
void set_protected_ext_info(struct RTK_PROTECTED_EXT_INFO *info, enum E_ION_NOTIFIER_PROTECTED_EXT ext,
		unsigned long base, unsigned long size, void * parent_priv);

struct RTK_PROTECTED_EXT_INFO * create_protected_ext_info(enum E_ION_NOTIFIER_PROTECTED_EXT ext,
		unsigned long base, unsigned long size, struct RTK_PROTECTED_INFO * parent);

int destroy_protected_ext_info(struct RTK_PROTECTED_EXT_INFO * info);

int ion_rtk_protected_create_notify(struct ion_rtk_protected_create_info *info);
int ion_rtk_protected_change_notify(struct ion_rtk_protected_change_info *info);
int ion_rtk_protected_destroy_notify(struct ion_rtk_protected_destroy_info
				     *info);
int ion_rtk_protected_ext_set_notify(struct ion_rtk_protected_ext_set * config);
int ion_rtk_protected_ext_unset_notify(struct ion_rtk_protected_ext_unset * config);

#endif /* _LINUX_ION_RTK_CARVEOUT_HEAP_PROTECTED_H */
