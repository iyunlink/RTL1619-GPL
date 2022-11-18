/* carveout_heap.h
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#ifndef _LINUX_ION_RTK_CARVEOUT_HEAP_H
#define _LINUX_ION_RTK_CARVEOUT_HEAP_H

#include <linux/ion.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <soc/realtek/memory.h>

#include "private.h"

enum ion_rtk_carveout_pool_type {
	RTK_CARVEOUT_GEN_POOL_TYPE = 0,
	RTK_CARVEOUT_CMA_POOL_TYPE,
};

struct ion_rtk_priv_pool {
	enum ion_rtk_carveout_pool_type type;
	phys_addr_t base;
	struct cma *cma_pool;
	size_t size;
	unsigned long flags;
	struct list_head list;
	bool from_reserved_memory;
};

struct ion_flag_replace {
	unsigned long condition;
	unsigned long replace;
	struct list_head list;
};

struct ion_rtk_pre_alloc {
	unsigned long rtk_flags;
	unsigned long size;
	struct list_head list;
};

struct ion_rtk_heap_create_priv {
	struct list_head pools;
	struct list_head rtk_flag_replace;
	struct list_head pre_alloc;
};

struct ion_rtk_carveout_meminfo {
	unsigned long usedSize;
	unsigned long freeSize;
};

struct ion_rtk_carveout_ops {
	unsigned int (*getVersion) (struct ion_heap * heap);
	int (*getMemInfo) (struct ion_heap * heap, unsigned int flags,
			   struct ion_rtk_carveout_meminfo * info);
	int (*traceDump) (struct ion_heap * heap, struct seq_file * s);
};

struct ion_heap *ion_rtk_carveout_heap_create(struct device *dev, struct ion_platform_heap *heap_data);
void ion_rtk_carveout_heap_destroy(struct ion_heap *);

struct ion_rtk_carveout_ops *get_rtk_carveout_ops(struct ion_heap *heap);

#endif /* End of _LINUX_ION_RTK_CARVEOUT_HEAP_H */
