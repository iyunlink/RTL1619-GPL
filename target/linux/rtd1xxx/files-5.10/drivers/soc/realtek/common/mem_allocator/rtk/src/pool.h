/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef _LINUX_ION_RTK_CARVEOUT_HEAP_POOL_H
#define _LINUX_ION_RTK_CARVEOUT_HEAP_POOL_H

#include <linux/types.h>
#include <linux/seq_buf.h>
#include <linux/dma-buf.h>
#include <linux/scatterlist.h>
#include "carveout_heap.h"
#include "flags.h"

struct page_info;
struct pool_info;

enum pool_type {
	GEN_POOL,
	CMA_POOL,
	PREALLOC_POOL,
};

struct pool_info {
	enum pool_type type;
	unsigned long base;
	size_t size;
	unsigned long rtk_flags;
	struct list_head list;

	struct page_info *(*alloc) (struct pool_info * pool_info,
				    unsigned long size, unsigned long align,
				    unsigned long flags);
	 size_t(*getSpace) (struct pool_info * pool_info, unsigned long flags);
	 size_t(*getUsage) (struct pool_info * pool_info, unsigned long flags);
	int (*debug_show) (struct pool_info * pool_info, struct seq_buf *,
			   char *);
	void (*destroy) (struct pool_info * pool_info);
};

struct page_info {
	unsigned long base;
	size_t size;
	unsigned long flags;
	char task_comm[TASK_COMM_LEN];
	pid_t pid;
	pid_t tgid;

	struct sg_table *(*get_sg_table) (struct page_info * page_info);
	void (*sync) (struct device *dev, struct page_info * page_info, enum dma_data_direction dir);
	void (*sync_range) (struct device *dev, struct page_info * page_info,
			    enum dma_data_direction dir, unsigned long addr,
			    size_t size);
	void (*destroy) (struct page_info * page_info);
};

struct pool_info *ion_gen_pool_create(enum pool_type type, unsigned long base, size_t size,
				      unsigned long rtk_flags,
				      bool from_reserved_memory,
				      struct device *dev);
struct pool_info *ion_cma_pool_create(struct cma *cma, unsigned long rtk_flags);

void page_common_init(struct page_info *info, unsigned long base, size_t size,
		      unsigned long flags);
void page_common_sync(struct device *dev, struct page_info *page_info, enum dma_data_direction dir);
void page_common_sync_range(struct device *dev, struct page_info *page_info,
			    enum dma_data_direction dir, unsigned long addr,
			    size_t size);

void pool_common_init(struct pool_info *info, enum pool_type type,
		      unsigned long base, size_t size, unsigned long rtk_flags);

void seq_buf_printf_ion_flags(struct seq_buf *s, unsigned long flags);

void page_common_debug_show(struct page_info *page_info,
			    struct seq_buf *s, char *prefix);
void pool_common_debug_show(struct pool_info *pool_info,
			    struct seq_buf *s, char *prefix);
#endif /* End of  _LINUX_ION_RTK_CARVEOUT_HEAP_POOL_H */
