/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include "ion_rtk_protected_notifier.h"

struct protected_info {
	unsigned long base;
	size_t size;
};

static int protected_handler(struct notifier_block *self,
			     unsigned long val, void *data)
{
	switch (val) {
	case PROTECTED_REGION_CREATE:
		{
			struct protected_info *info =
			    kzalloc(sizeof(struct protected_info), GFP_KERNEL);
			struct ion_rtk_protected_create_info *create_info =
			    (struct ion_rtk_protected_create_info *)data;
			info->base = create_info->mem.base;
			info->size = create_info->mem.size;
			pr_info("PROTECTED_REGION_CREATE: 0x%08lx ~ 0x%08lx\n",
				info->base, info->base + info->size);
			create_info->priv_virt = (void *)info;
		}
		break;
	case PROTECTED_REGION_CHANGE:
		{
			struct ion_rtk_protected_change_info *change_info =
			    (struct ion_rtk_protected_change_info *)data;
			struct protected_info *info =
			    (struct protected_info *)change_info->priv_virt;
			pr_info
			    ("PROTECTED_REGION_CHANGE: 0x%08lx ~ 0x%08lx => 0x%08lx ~ 0x%08lx\n",
			     info->base, info->base + info->size,
			     change_info->mem.base,
			     change_info->mem.base + change_info->mem.size);
			info->base = change_info->mem.base;
			info->size = change_info->mem.size;
		}
		break;
	case PROTECTED_REGION_DESTROY:
		{
			struct ion_rtk_protected_destroy_info *destroy_info =
			    (struct ion_rtk_protected_destroy_info *)data;
			struct protected_info *info =
			    (struct protected_info *)destroy_info->priv_virt;
			pr_info("PROTECTED_REGION_DESTROY: 0x%08lx ~ 0x%08lx\n",
				info->base, info->base + info->size);
			kfree(info);
		}
		break;
	default:
		return NOTIFY_BAD;
	}
	return NOTIFY_OK;
}

static struct notifier_block protected_notifier = {
	.notifier_call = protected_handler,
};

static int __init ion_rtk_notifier_init(void)
{
	return ion_rtk_protected_notifier_register(&protected_notifier);
}

static void ion_rtk_notifier_exit(void)
{
	ion_rtk_protected_notifier_unregister(&protected_notifier);
}

module_init(ion_rtk_notifier_init);
module_exit(ion_rtk_notifier_exit);
MODULE_AUTHOR("Otto Chen <ottochen@realtek.com>");
MODULE_DESCRIPTION("test of protected memory");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
