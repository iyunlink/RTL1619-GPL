/* ion_rtk_sys_heap.c
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <asm/page.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>

#include "ion_old/ion.h"
#include "uapi_old/ion.h"
#include "uapi_old/ion_rtk.h"

static const unsigned int orders[] = { 8, 4, 0 };

#define NUM_ORDERS ARRAY_SIZE(orders)

static gfp_t high_order_gfp_flags = (GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN |
				     __GFP_NORETRY) & ~__GFP_RECLAIM;
static gfp_t low_order_gfp_flags = GFP_HIGHUSER | __GFP_ZERO;

static int order_to_index(unsigned int order)
{
	int i;

	for (i = 0; i < NUM_ORDERS; i++)
		if (order == orders[i])
			return i;
	BUG();
	return -1;
}

static inline unsigned int order_to_size(int order)
{
	return PAGE_SIZE << order;
}

struct rtk_ion_sys_heap {
	struct ion_heap heap;
	struct ion_page_pool *uncached_pools[NUM_ORDERS];
	struct ion_page_pool *cached_pools[NUM_ORDERS];
};

/**
 * The page from page-pool are all zeroed before. We need do cache
 * clean for cached buffer. The uncached buffer are always non-cached
 * since it's allocated. So no need for non-cached pages.
 */
static struct page *alloc_buffer_page(struct rtk_ion_sys_heap *heap,
				      struct ion_buffer *buffer,
				      unsigned long order)
{
	bool cached = ion_buffer_cached(buffer);
	struct ion_page_pool *pool;
	struct page *page;

	if (!cached)
		pool = heap->uncached_pools[order_to_index(order)];
	else
		pool = heap->cached_pools[order_to_index(order)];

	page = ion_page_pool_alloc(pool);

	return page;
}

static void free_buffer_page(struct rtk_ion_sys_heap *heap,
			     struct ion_buffer *buffer, struct page *page)
{
	struct ion_page_pool *pool;
	unsigned int order = compound_order(page);
	bool cached = ion_buffer_cached(buffer);

	/* go to system */
	if (buffer->private_flags & ION_PRIV_FLAG_SHRINKER_FREE) {
		__free_pages(page, order);
		return;
	}

	if (!cached)
		pool = heap->uncached_pools[order_to_index(order)];
	else
		pool = heap->cached_pools[order_to_index(order)];

	ion_page_pool_free(pool, page);
}

static struct page *alloc_largest_available(struct rtk_ion_sys_heap *heap,
					    struct ion_buffer *buffer,
					    unsigned long size,
					    unsigned int max_order)
{
	struct page *page;
	int i;

	for (i = 0; i < NUM_ORDERS; i++) {
		if (size < order_to_size(orders[i]))
			continue;
		if (max_order < orders[i])
			continue;

		page = alloc_buffer_page(heap, buffer, orders[i]);
		if (!page)
			continue;

		return page;
	}

	return NULL;
}

static int rtk_ion_sys_heap_allocate(struct ion_heap *heap,
				     struct ion_buffer *buffer,
				     unsigned long size, unsigned long flags)
{
	struct rtk_ion_sys_heap *sys_heap = container_of(heap,
							 struct
							 rtk_ion_sys_heap,
							 heap);
	struct sg_table *table;
	struct scatterlist *sg;
	struct list_head pages;
	struct page *page, *tmp_page;
	int i = 0;
	unsigned long size_remaining = PAGE_ALIGN(size);
	unsigned int max_order = orders[0];

	if (size / PAGE_SIZE > totalram_pages / 2)
		return -ENOMEM;

	buffer->flags |= ION_FLAG_CACHED;

	INIT_LIST_HEAD(&pages);
	while (size_remaining > 0) {
		page = alloc_largest_available(sys_heap, buffer, size_remaining,
					       max_order);
		if (!page)
			goto free_pages;
		list_add_tail(&page->lru, &pages);
		size_remaining -= PAGE_SIZE << compound_order(page);
		max_order = compound_order(page);
		i++;
	}
	table = kmalloc(sizeof(*table), GFP_KERNEL);
	if (!table)
		goto free_pages;

	if (sg_alloc_table(table, i, GFP_KERNEL))
		goto free_table;

	sg = table->sgl;
	list_for_each_entry_safe(page, tmp_page, &pages, lru) {
		sg_set_page(sg, page, PAGE_SIZE << compound_order(page), 0);
		sg = sg_next(sg);
		list_del(&page->lru);
	}

	buffer->sg_table = table;
	return 0;

free_table:
	kfree(table);
free_pages:
	list_for_each_entry_safe(page, tmp_page, &pages, lru)
	    free_buffer_page(sys_heap, buffer, page);
	return -ENOMEM;
}

static void rtk_ion_sys_heap_free(struct ion_buffer *buffer)
{
	struct rtk_ion_sys_heap *sys_heap = container_of(buffer->heap,
							 struct
							 rtk_ion_sys_heap,
							 heap);
	struct sg_table *table = buffer->sg_table;
	struct scatterlist *sg;
	int i;

	/* zero the buffer before goto page pool */
	if (!(buffer->private_flags & ION_PRIV_FLAG_SHRINKER_FREE))
		ion_heap_buffer_zero(buffer);

	for_each_sg(table->sgl, sg, table->nents, i)
	    free_buffer_page(sys_heap, buffer, sg_page(sg));
	sg_free_table(table);
	kfree(table);
}

static int rtk_ion_sys_heap_shrink(struct ion_heap *heap, gfp_t gfp_mask,
				   int nr_to_scan)
{
	struct ion_page_pool *uncached_pool;
	struct ion_page_pool *cached_pool;
	struct rtk_ion_sys_heap *sys_heap;
	int nr_total = 0;
	int i, nr_freed;
	int only_scan = 0;

	sys_heap = container_of(heap, struct rtk_ion_sys_heap, heap);

	if (!nr_to_scan)
		only_scan = 1;

	for (i = 0; i < NUM_ORDERS; i++) {
		uncached_pool = sys_heap->uncached_pools[i];
		cached_pool = sys_heap->cached_pools[i];

		if (only_scan) {
			nr_total += ion_page_pool_shrink(uncached_pool,
							 gfp_mask, nr_to_scan);

			nr_total += ion_page_pool_shrink(cached_pool,
							 gfp_mask, nr_to_scan);
		} else {
			nr_freed = ion_page_pool_shrink(uncached_pool,
							gfp_mask, nr_to_scan);
			nr_to_scan -= nr_freed;
			nr_total += nr_freed;
			if (nr_to_scan <= 0)
				break;
			nr_freed = ion_page_pool_shrink(cached_pool,
							gfp_mask, nr_to_scan);
			nr_to_scan -= nr_freed;
			nr_total += nr_freed;
			if (nr_to_scan <= 0)
				break;
		}
	}
	return nr_total;
}

void *rtk_ion_heap_map_kernel(struct ion_heap *heap, struct ion_buffer *buffer)
{
	struct scatterlist *sg;
	int i, j;
	void *vaddr;
	pgprot_t pgprot;
	struct sg_table *table = buffer->sg_table;
	int npages = PAGE_ALIGN(buffer->size) / PAGE_SIZE;
	struct page **pages = vmalloc(sizeof(struct page *) * npages);
	struct page **tmp = pages;

	if (!pages)
		return ERR_PTR(-ENOMEM);

	if (buffer->flags & ION_FLAG_CACHED)
		pgprot = PAGE_KERNEL;
	else
		pgprot = pgprot_noncached(PAGE_KERNEL);

	for_each_sg(table->sgl, sg, table->nents, i) {
		int npages_this_entry = PAGE_ALIGN(sg->length) / PAGE_SIZE;
		struct page *page = sg_page(sg);

		BUG_ON(i >= npages);
		for (j = 0; j < npages_this_entry; j++)
			*(tmp++) = page++;
	}
	vaddr = vmap(pages, npages, VM_MAP, pgprot);
	vfree(pages);

	if (!vaddr)
		return ERR_PTR(-ENOMEM);

	return vaddr;
}

static struct ion_heap_ops system_heap_ops = {
	.allocate = rtk_ion_sys_heap_allocate,
	.free = rtk_ion_sys_heap_free,
	.map_kernel = rtk_ion_heap_map_kernel,
	.unmap_kernel = ion_heap_unmap_kernel,
	.map_user = ion_heap_map_user,
	.shrink = rtk_ion_sys_heap_shrink,
};

static void ion_system_heap_destroy_pools(struct ion_page_pool **pools)
{
	int i;

	for (i = 0; i < NUM_ORDERS; i++)
		if (pools[i])
			ion_page_pool_destroy(pools[i]);
}

static int rtk_ion_sys_heap_create_pools(struct ion_page_pool **pools,
					 bool cached)
{
	int i;

	for (i = 0; i < NUM_ORDERS; i++) {
		struct ion_page_pool *pool;
		gfp_t gfp_flags = low_order_gfp_flags;

		if (orders[i] > 4)
			gfp_flags = high_order_gfp_flags;

		pool = ion_page_pool_create(gfp_flags, orders[i], cached);
		if (!pool)
			goto err_create_pool;
		pools[i] = pool;
	}
	return 0;

err_create_pool:
	ion_system_heap_destroy_pools(pools);
	return -ENOMEM;
}

static int ion_system_heap_debug_show(struct ion_heap *heap, struct seq_file *s,
				      void *unused)
{
	struct rtk_ion_sys_heap *sys_heap = container_of(heap,
							 struct
							 rtk_ion_sys_heap,
							 heap);
	int i;
	struct ion_page_pool *pool;

	for (i = 0; i < NUM_ORDERS; i++) {
		pool = sys_heap->uncached_pools[i];

		seq_printf(s, "%d order %u highmem pages uncached %lu total\n",
			   pool->high_count, pool->order,
			   (PAGE_SIZE << pool->order) * pool->high_count);
		seq_printf(s, "%d order %u lowmem pages uncached %lu total\n",
			   pool->low_count, pool->order,
			   (PAGE_SIZE << pool->order) * pool->low_count);
	}

	for (i = 0; i < NUM_ORDERS; i++) {
		pool = sys_heap->cached_pools[i];

		seq_printf(s, "%d order %u highmem pages cached %lu total\n",
			   pool->high_count, pool->order,
			   (PAGE_SIZE << pool->order) * pool->high_count);
		seq_printf(s, "%d order %u lowmem pages cached %lu total\n",
			   pool->low_count, pool->order,
			   (PAGE_SIZE << pool->order) * pool->low_count);
	}
	return 0;
}

static void rtk_ion_sys_heap_destroy_pools(struct ion_page_pool **pools)
{
	int i;

	for (i = 0; i < NUM_ORDERS; i++)
		if (pools[i])
			ion_page_pool_destroy(pools[i]);
}

static struct ion_heap *__rtk_ion_sys_heap_create(void)
{
	struct rtk_ion_sys_heap *heap;

	heap = kzalloc(sizeof(*heap), GFP_KERNEL);
	if (!heap)
		return ERR_PTR(-ENOMEM);
	heap->heap.ops = &system_heap_ops;
	heap->heap.type = ION_HEAP_TYPE_CUSTOM;

	if (rtk_ion_sys_heap_create_pools(heap->uncached_pools, false))
		goto free_heap;

	if (rtk_ion_sys_heap_create_pools(heap->cached_pools, true))
		goto destroy_uncached_pools;

	heap->heap.debug_show = ion_system_heap_debug_show;
	return &heap->heap;

destroy_uncached_pools:
	rtk_ion_sys_heap_destroy_pools(heap->uncached_pools);

free_heap:
	kfree(heap);
	return ERR_PTR(-ENOMEM);
}

static int rtk_ion_sys_heap_create(void)
{
	struct ion_heap *heap;

	heap = __rtk_ion_sys_heap_create();
	if (IS_ERR(heap))
		return PTR_ERR(heap);
	heap->name = "rtk_ion_system_heap";

	ion_device_add_heap(heap);
	return 0;
}

late_initcall_sync(rtk_ion_sys_heap_create);
