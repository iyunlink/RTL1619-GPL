/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include <linux/cma.h>
#include <linux/highmem.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/dma-map-ops.h>

#include "cma.h"
#include "pool.h"
#include "protected.h"

struct cma_pool_info {
	struct pool_info common;
	struct mutex lock;
	struct list_head pages;
	struct cma *cpool;
	struct list_head protected_list;
	unsigned long *protected_reserved_bitmap;
	struct page *protected_static_page;
};

struct cma_page_info {
	struct page_info common;
	struct cma_pool_info *parent;
	struct page *page;
	struct list_head list;
	struct sg_table *sg_table;
	struct RTK_PROTECTED_EXT_INFO * protected_ext_info;
};

/* public api */
struct pool_info *ion_cma_pool_create(struct cma *cma, unsigned long rtk_flags);
static struct page_info *ion_cma_pool_alloc(struct pool_info *pool_info,
					    unsigned long size,
					    unsigned long align,
					    unsigned long flags);
static size_t ion_cma_pool_getSpace(struct pool_info *pool_info,
				    unsigned long flags);
static size_t ion_cma_pool_getUsage(struct pool_info *pool_info,
				    unsigned long flags);
static int ion_cma_pool_debug_show(struct pool_info *pool_info,
				   struct seq_buf *, char *prefix);
static void ion_cma_pool_destroy(struct pool_info *pool_info);
static void ion_cma_page_destroy(struct page_info *page_info);
static struct sg_table *ion_cma_page_get_sg_table(struct page_info *page_info);

/* private func */
static struct cma_page_info *ion_cma_page_alloc(void);
static void ion_cma_page_free(struct cma_page_info *page);
static void ion_cma_page_info_init(struct cma_page_info *page, size_t count,
				   unsigned long flags,
				   unsigned long rtk_flags);
static struct page *ion_cma_pool_alloc_taile(struct cma_pool_info *pool,
					     size_t count, unsigned int align, bool no_warn);
static struct page *ion_cma_pool_alloc_from_protected(struct cma_pool_info
						      *pool, size_t count,
						      unsigned int align);
struct RTK_PROTECTED_INFO * ion_cma_pool_find_protected_info(struct cma_pool_info *pool, struct page *page);
static bool ion_cma_pool_check_region_is_safe(struct cma_pool_info *pool,
					      enum E_ION_NOTIFIER_PROTECTED_TYPE
					      protected_type, struct page *page,
					      size_t count);
static void ion_cma_pool_refresh_protected_range(struct cma_pool_info *pool, struct cma_page_info
						 *cma_page);
static size_t ion_cma_free_pages(struct cma_pool_info *pool);

static size_t bitmap_seq_printf(struct seq_buf *s, unsigned long *bitmap,
			      int nbits)
{
	/* current bit is 'cur', most recently seen range is [rbot, rtop] */
	int cur, rbot;
	bool first = true;

	cur = find_first_bit(bitmap, nbits);
	while (cur < nbits) {
		rbot = cur;
		cur = find_next_zero_bit(bitmap, nbits, cur + 1);

		if (!first)
			seq_buf_printf(s, ",");
		first = false;

		seq_buf_printf(s, "%d", rbot);

		seq_buf_printf(s, "-%d", cur - 1);

		cur = find_next_bit(bitmap, nbits, cur + 1);
	}
	return 0;
}



struct pool_info *ion_cma_pool_create(struct cma *cma, unsigned long rtk_flags)
{
	struct pool_info *ret = NULL;

	do {
		struct cma_pool_info *pool = (struct cma_pool_info *)
		    kzalloc(sizeof(struct cma_pool_info), GFP_KERNEL);
		struct pool_info *common;
		unsigned long base;
		size_t size;

		if (!pool)
			break;

		common = &pool->common;
		base = PFN_PHYS(cma->base_pfn);
		size = (cma->count << PAGE_SHIFT);

		pool_common_init(common, CMA_POOL, base, size, rtk_flags);
		common->alloc = ion_cma_pool_alloc;
		common->getSpace = ion_cma_pool_getSpace;
		common->getUsage = ion_cma_pool_getUsage;
		common->debug_show = ion_cma_pool_debug_show;
		common->destroy = ion_cma_pool_destroy;

		pool->cpool = cma;
		INIT_LIST_HEAD(&pool->pages);
		INIT_LIST_HEAD(&pool->protected_list);
		if (rtk_flag_is_protected_dynamic(rtk_flags)) {
			int bitmap_count = cma_bitmap_maxno(pool->cpool);
			int bitmap_size =
			    BITS_TO_LONGS(bitmap_count) * sizeof(long);
			pool->protected_reserved_bitmap =
			    kzalloc(bitmap_size, GFP_KERNEL);
			bitmap_set(pool->protected_reserved_bitmap,
				   0 /*start */, bitmap_count /*len */);
		} else if (rtk_flag_is_protected_static(rtk_flags)) {
			do {
				int bitmap_count =
				    cma_bitmap_maxno(pool->cpool);
				int bitmap_size =
				    BITS_TO_LONGS(bitmap_count) * sizeof(long);

				pool->protected_static_page =
				    cma_alloc(pool->cpool, bitmap_count, 0,
					      GFP_KERNEL | GFP_DMA);

				if (!pool->protected_static_page) {
					pr_err
					    ("%s : cma area creation failed! (cma = %p, rtk_flags = 0x%lx)\n",
					     __func__, pool->cpool, rtk_flags);
					break;
				}

				if (!ion_cma_pool_check_region_is_safe
				    (pool,
				     rtk_flags_to_notifier_protected_type
				     (rtk_flags), pool->protected_static_page,
				     bitmap_count)) {
					cma_release(pool->cpool,
						    pool->protected_static_page,
						    bitmap_count);
					pool->protected_static_page = NULL;
					pr_err
					    ("%s : Protected area creation failed! (cma = %p, rtk_flags = 0x%lx)\n",
					     __func__, pool->cpool, rtk_flags);
					break;
				}

				pool->protected_reserved_bitmap =
				    kzalloc(bitmap_size, GFP_KERNEL);

				bitmap_clear(pool->protected_reserved_bitmap,
					     0 /*start */,
					     bitmap_count /*len */);
				break;
			} while (0);

			if (!pool->protected_reserved_bitmap) {
				pool->common.rtk_flags &=
				    ~(RTK_FLAG_PROTECTED_DYNAMIC |
				      RTK_FLAG_PROTECTED_MASK);
			}
		}

		mutex_init(&pool->lock);
		ret = &pool->common;
		break;
	} while (0);
	return ret;
}

void ion_cma_pool_destroy(struct pool_info *pool_info)
{
	struct cma_pool_info *pool =
	    container_of(pool_info, struct cma_pool_info, common);
	mutex_lock(&pool->lock);
	//TODO
	if (pool->protected_reserved_bitmap)
		kfree(pool->protected_reserved_bitmap);
	mutex_unlock(&pool->lock);
	kfree(pool);
}

static void ion_cma_page_zero(struct page *page, size_t count)
{
	if (PageHighMem(page)) {
		unsigned long nr_clear_pages = count;
		struct page *ppage = page;

		while (nr_clear_pages > 0) {
			void *vaddr = kmap_atomic(ppage);

			memset(vaddr, 0, PAGE_SIZE);
#ifdef CONFIG_ARM64
			arch_dma_prep_coherent(ppage, PAGE_SIZE);
#elif defined(CONFIG_ARM)
			dmac_flush_range(vaddr, vaddr + PAGE_SIZE);
#endif
			kunmap_atomic(vaddr);
			ppage++;
			nr_clear_pages--;
		}
	} else {
		void *ptr = page_address(page);
		size_t flush_size = PAGE_SIZE * count;

		memset(ptr, 0, flush_size);
#ifdef CONFIG_ARM64
		arch_dma_prep_coherent(page, flush_size);
#elif defined(CONFIG_ARM)
		dmac_flush_range(ptr, ptr + flush_size);
		outer_flush_range(__pa(ptr), __pa(ptr) + flush_size);
#endif
	}
}

static struct page_info *ion_cma_pool_alloc(struct pool_info *pool_info,
					    unsigned long size,
					    unsigned long align,
					    unsigned long flags)
{
	struct page_info *ret = NULL;
	struct cma_pool_info *pool =
	    container_of(pool_info, struct cma_pool_info, common);
	struct pool_info *common = &pool->common;
	bool no_warn;

	if (IS_ENABLED(CONFIG_CMA_DEBUG))
		no_warn = false;
	else
		no_warn = true;

	mutex_lock(&pool->lock);
	do {
		struct cma_page_info *page = NULL;
		size_t cma_count = size >> PAGE_SHIFT;
		unsigned int cma_align = 0;	/* closely spaced */

		if (ion_flag_mismatch_pool_condition(flags, common->rtk_flags))
			break;

		page = ion_cma_page_alloc();
		if (!page)
			break;

		if (rtk_flag_is_protected(common->rtk_flags) &&
		    ion_flag_is_protected(flags)) {
			if (rtk_flag_is_protected_dynamic(common->rtk_flags)) {
				const size_t protected_align_size =
				    2 * 1024 * 1024;
				cma_align = protected_align_size >> PAGE_SHIFT;
				cma_count = ALIGN(cma_count, cma_align);
			}
			page->page =
			    ion_cma_pool_alloc_from_protected(pool, cma_count,
							      cma_align);
			if (!page->page) {
				page->page =
				    ion_cma_pool_alloc_taile(pool, cma_count,
							     cma_align, no_warn);
				if (!page->page) {
					ion_cma_page_free(page);
					break;
				}

				if (!ion_cma_pool_check_region_is_safe
				    (pool,
				     ion_flags_to_notifier_protected_type
				     (flags), page->page, cma_count)) {
					cma_release(pool->cpool, page->page,
						    cma_count);
					ion_cma_page_free(page);
					break;
				}
			}
		} else {
			cma_align = get_order(size);
			if (cma_align > CONFIG_CMA_ALIGNMENT)
				cma_align = CONFIG_CMA_ALIGNMENT;

			page->page =
			    cma_alloc(pool->cpool, cma_count, cma_align, no_warn);
		}

		if (!page->page) {
			ion_cma_page_free(page);
			break;
		}

		if (rtk_flag_is_protected(common->rtk_flags) &&
				!rtk_flag_has_protected_ext(common->rtk_flags) &&
				ion_flag_has_protected_ext(flags)) {

			page->protected_ext_info = create_protected_ext_info(
					ion_flag_to_notifier_protected_ext(flags),
					page_to_phys(page->page), cma_count << PAGE_SHIFT,
					ion_cma_pool_find_protected_info(pool, page->page));

			if (!page->protected_ext_info) {
				cma_release(pool->cpool, page->page, cma_count);
				ion_cma_page_free(page);
				break;
			}
		}

		if (!ion_flag_is_protected(flags))
			ion_cma_page_zero(page->page, cma_count);

		ion_cma_page_info_init(page, cma_count, flags,
				       common->rtk_flags);

		list_add(&page->list, &pool->pages);
		page->parent = pool;

		ret = &page->common;
	} while (0);
	mutex_unlock(&pool->lock);

	return ret;
}

static struct page *ion_cma_pool_alloc_from_protected(struct cma_pool_info
						      *pool, size_t count,
						      unsigned int align)
{
	struct cma *cma = pool->cpool;
	unsigned long mask, offset;
	unsigned long pfn = -1;
	unsigned long start = 0;
	unsigned long bitmap_maxno, bitmap_no, bitmap_count;
	struct page *page = NULL;

	if (!cma || !cma->count)
		return NULL;

	if (!count)
		return NULL;

	mask =
	    (align <=
	     cma->order_per_bit) ? 0 : (1UL << (align - cma->order_per_bit)) -
	    1;
	offset = (cma->base_pfn & ((1UL << align) - 1)) >> cma->order_per_bit;
	bitmap_maxno = cma_bitmap_maxno(cma);
	bitmap_count =
	    ALIGN(count, 1UL << cma->order_per_bit) >> cma->order_per_bit;

	if (bitmap_count > bitmap_maxno)
		return NULL;

	do {
		bitmap_no =
		    bitmap_find_next_zero_area_off
		    (pool->protected_reserved_bitmap, bitmap_maxno, start,
		     bitmap_count, mask, offset);

		if (bitmap_no >= bitmap_maxno) {
			break;
		}

		pfn = cma->base_pfn + (bitmap_no << cma->order_per_bit);

		page = pfn_to_page(pfn);

		bitmap_set(pool->protected_reserved_bitmap, bitmap_no,
			   bitmap_count);
	} while (0);
	return page;
}

#if defined(CONFIG_CMA_DEBUG)
static void ion_cma_debug_show_areas(struct cma *cma)
{
	unsigned long next_zero_bit, next_set_bit, nr_zero;
	unsigned long start = 0;
	unsigned long nr_part, nr_total = 0;
	unsigned long nbits = cma_bitmap_maxno(cma);

	mutex_lock(&cma->lock);
	pr_info("number of available pages: ");
	for (;;) {
		next_zero_bit = find_next_zero_bit(cma->bitmap, nbits, start);
		if (next_zero_bit >= nbits)
			break;
		next_set_bit = find_next_bit(cma->bitmap, nbits, next_zero_bit);
		nr_zero = next_set_bit - next_zero_bit;
		nr_part = nr_zero << cma->order_per_bit;
		pr_cont("%s%lu@%lu", nr_total ? "+" : "", nr_part,
			next_zero_bit);
		nr_total += nr_part;
		start = next_zero_bit + nr_zero;
	}
	pr_cont("=> %lu free of %lu total pages\n", nr_total, cma->count);
	mutex_unlock(&cma->lock);
}
#else
static inline void ion_cma_debug_show_areas(struct cma *cma) { }
#endif

static struct page *ion_cma_pool_alloc_taile(struct cma_pool_info *pool,
					     size_t count, unsigned int align, bool no_warn)
{
	struct cma *cma = pool->cpool;
	unsigned long mask, offset;
	unsigned long pfn = -1;
	unsigned long start = 0;
	unsigned long bitmap_maxno, bitmap_no, bitmap_count;
	struct page *page = NULL;
	int ret = -ENOMEM;

	if (!cma || !cma->count)
		return NULL;

	pr_debug("%s(cma %p, count %zu, align %d)\n", __func__, (void *)cma,
		 count, align);

	if (!count)
		return NULL;

	mask =
	    (align <=
	     cma->order_per_bit) ? 0 : (1UL << (align - cma->order_per_bit)) -
	    1;
	offset = (cma->base_pfn & ((1UL << align) - 1)) >> cma->order_per_bit;
	bitmap_maxno = cma_bitmap_maxno(cma);
	bitmap_count =
	    ALIGN(count, 1UL << cma->order_per_bit) >> cma->order_per_bit;

	if (bitmap_count > bitmap_maxno)
		return NULL;

	start = bitmap_maxno;

	while (start >= bitmap_count) {
		bitmap_maxno = start;
		start -= bitmap_count;
		mutex_lock(&cma->lock);
		bitmap_no = bitmap_find_next_zero_area_off(cma->bitmap,
							   bitmap_maxno, start,
							   bitmap_count, mask,
							   offset);

		if (bitmap_no >= bitmap_maxno) {
			start = find_next_bit(cma->bitmap, bitmap_maxno, start);
			mutex_unlock(&cma->lock);
			continue;
		}
		bitmap_set(cma->bitmap, bitmap_no, bitmap_count);
		mutex_unlock(&cma->lock);

		pfn = cma->base_pfn + (bitmap_no << cma->order_per_bit);
		ret =
		    alloc_contig_range(pfn, pfn + count, MIGRATE_CMA,
				       GFP_KERNEL | (no_warn ? __GFP_NOWARN : 0));

		if (ret == 0) {
			page = pfn_to_page(pfn);
			break;
		}

		mutex_lock(&cma->lock);
		bitmap_clear(cma->bitmap, bitmap_no, bitmap_count);
		mutex_unlock(&cma->lock);

		if (ret != -EBUSY)
			break;
	};

	if (ret && !no_warn) {
		pr_err("%s: alloc failed, req-size: %zu pages, ret: %d\n",
			__func__, count, ret);
		ion_cma_debug_show_areas(cma);
	}

	return page;
}

struct RTK_PROTECTED_INFO * ion_cma_pool_find_protected_info(struct cma_pool_info *pool, struct page *page)
{
	struct RTK_PROTECTED_INFO * ret = NULL;
	struct RTK_PROTECTED_INFO *protected_info;
	unsigned long offset = page_to_phys(page);
	list_for_each_entry(protected_info, &pool->protected_list, list) {
		if (offset >= get_protected_base(protected_info) &&
				offset < get_protected_limit(protected_info)) {
			ret = protected_info;
			break;
		}

	}
	return ret;
}

static bool ion_cma_pool_check_region_is_safe(struct cma_pool_info *pool,
					      enum E_ION_NOTIFIER_PROTECTED_TYPE
					      protected_type, struct page *page,
					      size_t count)
{
	bool safe = false;
	do {
		struct RTK_PROTECTED_INFO *protected_info;
		struct cma *cma = pool->cpool;
		unsigned long bitmap_count = ALIGN(count,
						   1UL << cma->order_per_bit) >>
		    cma->order_per_bit;
		unsigned long offset = page_to_phys(page);
		size_t size = bitmap_count << PAGE_SHIFT;
		unsigned long limit = offset + size;
		list_for_each_entry(protected_info, &pool->protected_list, list) {
			if (get_protected_base(protected_info) == limit) {
				unsigned long next_protected_base = offset;
				unsigned long next_protected_limit =
				    get_protected_limit(protected_info);
				struct ion_rtk_protected_change_info info;

				info.mem.base = next_protected_base;
				info.mem.size =
				    next_protected_limit - next_protected_base;
				info.mem.type =
				    rtk_flags_to_notifier_protected_type
				    (pool->common.rtk_flags);
				info.priv_virt =
				    get_protected_priv(protected_info);
				if (ion_rtk_protected_change_notify(&info) !=
				    NOTIFY_OK) {
					pr_err
					    ("%s:%d ion_rtk_protected_change_notify return ERROR! (priv_virt=%p)\n",
					     __func__, __LINE__,
					     get_protected_priv
					     (protected_info));
					continue;
				}
				set_protected_range(protected_info,
						    next_protected_base,
						    next_protected_limit);
				safe = true;
				break;
			} else if (get_protected_limit(protected_info) ==
				   offset) {
				unsigned long next_protected_base =
				    get_protected_base(protected_info);
				unsigned long next_protected_limit =
				    offset + size;
				struct ion_rtk_protected_change_info info;

				info.mem.base = next_protected_base;
				info.mem.size =
				    next_protected_limit - next_protected_base;
				info.mem.type =
				    rtk_flags_to_notifier_protected_type
				    (pool->common.rtk_flags);
				info.priv_virt =
				    get_protected_priv(protected_info);
				if (ion_rtk_protected_change_notify(&info) !=
				    NOTIFY_OK) {
					pr_err
					    ("%s:%d ion_rtk_protected_change_notify return ERROR! (priv_virt=%p)\n",
					     __func__, __LINE__,
					     get_protected_priv
					     (protected_info));
					continue;
				}
				set_protected_range(protected_info,
						    next_protected_base,
						    next_protected_limit);
				safe = true;
				break;
			}
		}

		if (!safe) {
			struct RTK_PROTECTED_INFO *protected_info =
			    kzalloc(sizeof(struct RTK_PROTECTED_INFO),
				    GFP_KERNEL);
			struct ion_rtk_protected_create_info info;

			info.mem.base = offset;
			info.mem.size = size;
			info.mem.type = protected_type;

			if (protected_info == NULL) {
				pr_err("%s:%d ERROR!\n", __func__, __LINE__);
				break;
			}

			if (ion_rtk_protected_create_notify(&info) != NOTIFY_OK) {
				kfree(protected_info);
				pr_err
				    ("%s:%d ion_rtk_protected_create_notify return ERROR! (priv_virt=%p)\n",
				     __func__, __LINE__,
				     get_protected_priv(protected_info));
				break;
			}

			init_protected_info(protected_info);
			set_protected_range(protected_info, offset,
					    offset + size);
			set_protected_type(protected_info, protected_type);
			set_protected_priv(protected_info, info.priv_virt);
			list_add(&protected_info->list, &pool->protected_list);
			safe = true;
			break;
		}

		if (!safe) {
			pr_err("%s:%d ERROR!\n", __func__, __LINE__);
		}

		break;
	} while (0);

	return safe;
}

struct cma_page_info *ion_cma_page_alloc(void)
{
	struct cma_page_info *ret = NULL;
	do {
		struct cma_page_info *page = (struct cma_page_info *)
		    kzalloc(sizeof(struct cma_page_info), GFP_KERNEL);

		if (!page) {
			pr_err("%s  kzalloc return NULL!", __func__);
			break;
		}

		page->sg_table = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
		if (!page->sg_table) {
			kfree(page);
			break;
		}

		if (sg_alloc_table(page->sg_table, 1, GFP_KERNEL) != 0) {
			kfree(page->sg_table);
			kfree(page);
			break;
		}

		ret = page;
	} while (0);
	return ret;
}

void ion_cma_page_free(struct cma_page_info *page)
{
	sg_free_table(page->sg_table);
	kfree(page->sg_table);
	kfree(page);
}

void ion_cma_page_info_init(struct cma_page_info *page, size_t count,
			    unsigned long flags, unsigned long rtk_flags)
{
	unsigned long base = page_to_phys(page->page);
	size_t size = count << PAGE_SHIFT;

	page_common_init(&page->common, base, size,
			 flags | ion_flag_from_rtk(rtk_flags));

	sg_set_page(page->sg_table->sgl, page->page, size, 0);

	{
		size_t i;
		struct scatterlist *sg;
		for_each_sg(page->sg_table->sgl, sg, page->sg_table->nents, i) {
			sg_dma_address(sg) = sg_phys(sg);
			sg_dma_len(sg) = sg->length;
		}
	}
	page->common.destroy = ion_cma_page_destroy;
	page->common.get_sg_table = ion_cma_page_get_sg_table;
	page->common.sync_range = page_common_sync_range;
	page->common.sync = page_common_sync;
	INIT_LIST_HEAD(&page->list);
}

void ion_cma_page_destroy(struct page_info *page_info)
{
	struct cma_page_info *page =
	    container_of(page_info, struct cma_page_info, common);
	struct cma_pool_info *pool = page->parent;
	mutex_lock(&pool->lock);
	do {

		list_del(&page->list);

		if (page->protected_ext_info) {
			if (destroy_protected_ext_info(page->protected_ext_info) != 0)
				pr_err("[%s] destroy_protected_ext_info return ERROR! (base=0x%08lx, size=0x%lx, flags=0x%lx)\n", __func__,
						page->common.base, page->common.size, page->common.flags);
			page->protected_ext_info = NULL;
		}

		if (rtk_flag_is_protected(pool->common.rtk_flags)
		    && ion_flag_is_protected(page->common.flags))
			ion_cma_pool_refresh_protected_range(pool, page);
		else
			cma_release(pool->cpool, page->page,
				    page->common.size >> PAGE_SHIFT);

		ion_cma_page_free(page);
		break;
	} while (0);
	mutex_unlock(&pool->lock);
}

static struct sg_table *ion_cma_page_get_sg_table(struct page_info *page_info)
{
	struct cma_page_info *page =
	    container_of(page_info, struct cma_page_info, common);
	return page->sg_table;
}

static void ion_cma_pool_refresh_protected_range(struct cma_pool_info *pool,
						 struct cma_page_info *cma_page)
{
	struct cma *cma = pool->cpool;
	{
		/* release to protected_reserved_bitmap */
		unsigned long pfn = page_to_pfn(cma_page->page);
		unsigned long bitmap_no =
		    (pfn - cma->base_pfn) >> cma->order_per_bit;
		unsigned long bitmap_count =
		    ALIGN(cma_page->common.size >> PAGE_SHIFT,
			  1UL << cma->order_per_bit) >> cma->order_per_bit;
		bitmap_clear(pool->protected_reserved_bitmap, bitmap_no,
			     bitmap_count);
	}

	if (rtk_flag_is_protected_dynamic(pool->common.rtk_flags)) {
		int bitmap_count = cma_bitmap_maxno(cma);
		unsigned long bitmap_offset =
		    find_first_zero_bit(pool->protected_reserved_bitmap,
					bitmap_count);

		while (bitmap_offset < bitmap_count) {
			struct RTK_PROTECTED_INFO *protected_info,
			    *tmp_protected_info;
			unsigned long bitmap_limit =
			    find_next_bit(pool->protected_reserved_bitmap,
					  bitmap_count, bitmap_offset + 1);
			unsigned long bitmap_size =
			    bitmap_limit - bitmap_offset;
			unsigned long pfn =
			    (bitmap_offset << cma->order_per_bit) +
			    cma->base_pfn;
			struct page *page = pfn_to_page(pfn);
			unsigned long phyAddr = page_to_phys(page);
			size_t size = bitmap_size << PAGE_SHIFT;
			list_for_each_entry_safe(protected_info,
						 tmp_protected_info,
						 &pool->protected_list, list) {
				if (phyAddr ==
				    get_protected_base(protected_info)) {
					if (size ==
					    get_protected_size(protected_info))
					{
						struct
						ion_rtk_protected_destroy_info
						    info;
						info.priv_virt =
						    get_protected_priv
						    (protected_info);
						if (ion_rtk_protected_destroy_notify(&info) != NOTIFY_OK) {
							pr_err
							    ("%s:%d ion_rtk_protected_destroy_notify return ERROR! (priv_virt=%p)\n",
							     __func__, __LINE__,
							     get_protected_priv
							     (protected_info));
							continue;
						}
						bitmap_set
						    (pool->
						     protected_reserved_bitmap,
						     bitmap_offset,
						     bitmap_size);
						cma_release(cma, page,
							    bitmap_size);
						list_del(&protected_info->list);
						kfree(protected_info);
					} else {
						unsigned long
						    next_protected_base =
						    get_protected_base
						    (protected_info) + size;
						unsigned long
						    next_protected_limit =
						    get_protected_limit
						    (protected_info);
						struct
						ion_rtk_protected_change_info
						    info;
						info.mem.base =
						    next_protected_base;
						info.mem.size =
						    next_protected_limit -
						    next_protected_base;
						info.mem.type =
						    rtk_flags_to_notifier_protected_type
						    (pool->common.rtk_flags);
						info.priv_virt =
						    get_protected_priv
						    (protected_info);
						if (ion_rtk_protected_change_notify(&info) != NOTIFY_OK) {
							pr_err
							    ("%s:%d ion_rtk_protected_change_notify return ERROR! (priv_virt=%p)\n",
							     __func__, __LINE__,
							     get_protected_priv
							     (protected_info));
							continue;
						}
						bitmap_set
						    (pool->
						     protected_reserved_bitmap,
						     bitmap_offset,
						     bitmap_size);
						cma_release(cma, page,
							    bitmap_size);
						set_protected_range
						    (protected_info,
						     next_protected_base,
						     next_protected_limit);
					}
				} else if ((phyAddr + size) ==
					   get_protected_limit(protected_info))
				{
					unsigned long next_protected_base =
					    get_protected_base(protected_info);
					unsigned long next_protected_limit =
					    get_protected_limit(protected_info)
					    - size;
					struct ion_rtk_protected_change_info
					    info;
					info.mem.base = next_protected_base;
					info.mem.size =
					    next_protected_limit -
					    next_protected_base;
					info.mem.type =
					    rtk_flags_to_notifier_protected_type
					    (pool->common.rtk_flags);
					info.priv_virt =
					    get_protected_priv(protected_info);
					if (ion_rtk_protected_change_notify
					    (&info) != NOTIFY_OK) {
						pr_err
						    ("%s:%d ion_rtk_protected_change_notify return ERROR! (priv_virt=%p)\n",
						     __func__, __LINE__,
						     get_protected_priv
						     (protected_info));
						continue;
					}
					bitmap_set
					    (pool->protected_reserved_bitmap,
					     bitmap_offset, bitmap_size);
					cma_release(cma, page, bitmap_size);
					set_protected_range(protected_info,
							    next_protected_base,
							    next_protected_limit);
				}
			}	/* End of list_for_each_entry_safe(protected_info... */
			bitmap_offset =
			    find_next_zero_bit(pool->protected_reserved_bitmap,
					       bitmap_count, bitmap_limit + 1);
		}		/* End of while (bitmap_offset < bitmap_count) { */
	}
}

static size_t ion_cma_free_pages(struct cma_pool_info *pool)
{
	size_t ret = 0;
	struct cma *cma = pool->cpool;
	mutex_lock(&cma->lock);
	do {
		unsigned long start = 0;
		unsigned long nr_total = 0;
		unsigned long nbits = cma_bitmap_maxno(cma);
		for (;;) {
			unsigned long next_zero_bit, next_set_bit, nr_zero,
			    nr_part;
			next_zero_bit =
			    find_next_zero_bit(cma->bitmap, nbits, start);
			if (next_zero_bit >= nbits)
				break;
			next_set_bit =
			    find_next_bit(cma->bitmap, nbits, next_zero_bit);
			nr_zero = next_set_bit - next_zero_bit;
			nr_part = nr_zero << cma->order_per_bit;
			nr_total += nr_part;
			start = next_zero_bit + nr_zero;
		}

		ret = nr_total;
	} while (0);
	mutex_unlock(&cma->lock);
	return ret;
}

static size_t ion_protected_free_pages(struct cma_pool_info *pool)
{
	size_t ret = 0;
	struct cma *cma = pool->cpool;
	mutex_lock(&cma->lock);
	do {
		unsigned long start = 0;
		unsigned long nr_total = 0;
		unsigned long nbits = cma_bitmap_maxno(cma);

		if (!rtk_flag_is_protected(pool->common.rtk_flags))
			break;

		for (;;) {
			unsigned long next_zero_bit, next_set_bit, nr_zero,
			    nr_part;
			next_zero_bit =
			    find_next_zero_bit(pool->protected_reserved_bitmap,
					       nbits, start);
			if (next_zero_bit >= nbits)
				break;
			next_set_bit =
			    find_next_bit(pool->protected_reserved_bitmap,
					  nbits, next_zero_bit);
			nr_zero = next_set_bit - next_zero_bit;
			nr_part = nr_zero << cma->order_per_bit;
			nr_total += nr_part;
			start = next_zero_bit + nr_zero;
		}

		ret = nr_total;
	} while (0);
	mutex_unlock(&cma->lock);
	return ret;
}

size_t ion_cma_pool_getSpace(struct pool_info * pool_info, unsigned long flags)
{
	size_t ret = 0;
	struct cma_pool_info *pool =
	    container_of(pool_info, struct cma_pool_info, common);
	mutex_lock(&pool->lock);
	do {
		if (ion_flag_mismatch_pool_condition
		    (flags, pool->common.rtk_flags))
			break;

		ret = ion_cma_free_pages(pool) << PAGE_SHIFT;

		if (ion_flag_is_protected(flags))
			ret += ion_protected_free_pages(pool) << PAGE_SHIFT;
	} while (0);
	mutex_unlock(&pool->lock);
	return ret;
}

size_t ion_cma_pool_getUsage(struct pool_info * pool_info, unsigned long flags)
{
	size_t ret = 0;
	struct cma_pool_info *pool =
	    container_of(pool_info, struct cma_pool_info, common);
	mutex_lock(&pool->lock);
	do {
		if (ion_flag_mismatch_pool_condition
		    (flags, pool->common.rtk_flags))
			break;

		ret =
		    (pool->cpool->count -
		     ion_cma_free_pages(pool)) << PAGE_SHIFT;

		ret -= ion_protected_free_pages(pool) << PAGE_SHIFT;
	} while (0);
	mutex_unlock(&pool->lock);
	return ret;
}

int ion_cma_pool_debug_show(struct pool_info *pool_info, struct seq_buf *s,
			    char *prefix)
{
	int ret = -1;
	struct cma_pool_info *pool =
	    container_of(pool_info, struct cma_pool_info, common);
	struct cma *cma = pool->cpool;
	if (!prefix)
		prefix = "";
	mutex_lock(&pool->lock);
	pool_common_debug_show(pool_info, s, prefix);
	do {
		char *space_prefix = "    ";
		char *sub_prefix =
		    kzalloc(strlen(prefix) + strlen(space_prefix) + 1, GFP_KERNEL);
		if (!sub_prefix)
			break;
		strcpy(sub_prefix, prefix);
		strcat(sub_prefix, space_prefix);
		{
			struct cma_page_info *info;
			list_for_each_entry(info, &pool->pages, list) {
				if (info)
					page_common_debug_show(&info->common, s,
							       sub_prefix);
			}
		}
		{
			seq_buf_printf(s, "%sFree pages : %zu\n", sub_prefix,
				   ion_cma_free_pages(pool));
			if (rtk_flag_is_protected(pool->common.rtk_flags)) {
				seq_buf_printf(s, "%sProtected Free pages : %zu\n",
					   sub_prefix,
					   ion_protected_free_pages(pool));
			}
			mutex_lock(&cma->lock);
			seq_buf_printf(s, "%sBitmap : ", sub_prefix);
			bitmap_seq_printf(s,
						    cma->bitmap, (int)
						    cma_bitmap_maxno
						    (pool->cpool));
			seq_buf_printf(s, "\n");
			mutex_unlock(&cma->lock);

			if (rtk_flag_is_protected(pool_info->rtk_flags)) {
				int protected_count = 0;
				struct RTK_PROTECTED_INFO *protected_info;
				list_for_each_entry(protected_info,
						    &pool->protected_list,
						    list) {
					if (protected_info)
						protected_count++;
				}
				seq_buf_printf(s,
					   "%sNumber of protection areas : %d\n",
					   sub_prefix, protected_count);
				seq_buf_printf(s,
					   "%sUnreserved protection bitmap : ",
					   sub_prefix);
				bitmap_seq_printf(s,
						    pool->protected_reserved_bitmap,
						    (int)
						    cma_bitmap_maxno
						    (pool->cpool));
				seq_buf_printf(s, "\n");
				protected_count = 0;
				list_for_each_entry(protected_info,
						    &pool->protected_list,
						    list) {
					if (protected_info) {
						seq_buf_printf(s,
							   "%sprotected[%d] : 0x%08lX~0x%08lX\n",
							   sub_prefix,
							   protected_count,
							   get_protected_base
							   (protected_info),
							   get_protected_limit
							   (protected_info));
						protected_count++;
					}
				}
			}
		}
		kfree(sub_prefix);
		ret = 0;
	} while (0);
	mutex_unlock(&pool->lock);
	return ret;
}

#ifdef CONFIG_VIRTUAL_PMU

/*
 * ref. from int ion_cma_pool_debug_show(..)
 */
unsigned long long ion_cma_pool_get_size(struct pool_info *pool_info)
{
	size_t pool_pages_size;
	struct cma_pool_info *pool =
	    container_of(pool_info, struct cma_pool_info, common);
	struct cma_page_info *info;

	pool_pages_size = 0;

	info = NULL;

	mutex_lock(&pool->lock);

	list_for_each_entry(info, &pool->pages, list) {
		if (info) {
			//page_common_debug_show(&info->common, s,
			//                     sub_prefix);
			pool_pages_size += info->common.size;
		}
	}

	mutex_unlock(&pool->lock);

	return (unsigned long long)pool_pages_size;
}

#endif
