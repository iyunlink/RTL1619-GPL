/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include "pool.h"
#include "protected.h"
#include <linux/slab.h>
#include <linux/genalloc.h>
#include <linux/string.h>

static u64 dma_mask;

enum rtk_algo_mode {
	ALGO_FIRST_FIT = 0,
	ALGO_LAST_FIT,
};

struct gen_pool_info {
	struct pool_info common;
	struct gen_pool *gpool;
	enum rtk_algo_mode algo_mode;
	struct mutex lock;
	struct list_head pages;
	struct RTK_PROTECTED_INFO protected_info;
};

struct gen_page_info {
	struct page_info common;
	struct gen_pool_info *parent;
	struct list_head list;
	struct sg_table *sg_table;
	struct RTK_PROTECTED_EXT_INFO * protected_ext_info;
};

/* public api */
struct pool_info *ion_gen_pool_create(enum pool_type type,
				      unsigned long base,
				      size_t size,
				      unsigned long rtk_flags,
				      bool from_reserved_memory,
				      struct device *dev);
static struct page_info *ion_gen_pool_alloc(struct pool_info *pool_info,
					    unsigned long size,
					    unsigned long align,
					    unsigned long flags);
static size_t ion_gen_pool_getSpace(struct pool_info *pool_info,
				    unsigned long flags);
static size_t ion_gen_pool_getUsage(struct pool_info *pool_info,
				    unsigned long flags);
static int ion_gen_pool_debug_show(struct pool_info *pool_info,
				   struct seq_buf *, char *);
static void ion_gen_pool_destroy(struct pool_info *pool_info);
static void ion_gen_page_destroy(struct page_info *page_info);
static struct sg_table *ion_gen_page_get_sg_table(struct page_info *page_info);

/* private func */
static struct gen_page_info *ion_gen_page_alloc(void);
static void ion_gen_page_free(struct gen_page_info *page);
static void ion_gen_page_info_init(struct gen_page_info *page,
				   unsigned long base, size_t size,
				   unsigned long flags,
				   unsigned long rtk_flags);
static bool ion_gen_pool_check_region_is_safe(struct gen_pool_info *pool,
					      bool protected,
					      unsigned long base, size_t size);
static void ion_gen_pool_refresh_protected_range(struct gen_pool_info *pool);
static unsigned long ion_gen_pool_algo_last_fit(unsigned long *map,
						unsigned long size,
						unsigned long start,
						unsigned int nr, void *data);
static unsigned long ion_gen_pool_algo(unsigned long *map, unsigned long size,
				       unsigned long start, unsigned int nr,
				       void *data, struct gen_pool *pool,
				       unsigned long start_addr);
static void ion_pages_sync_for_device(struct device *dev, struct page *page,
				      size_t size, enum dma_data_direction dir);
static size_t ion_gen_pool_pages_size(struct gen_pool_info *pool,
				      unsigned long flags);

static inline bool mem_is_within_range(unsigned long offset, size_t size,
				       unsigned long base, unsigned long limit)
{
	if (offset >= base && (offset + size) <= limit)
		return true;
	return false;
}

inline bool mem_is_some_overlap(unsigned long offset, size_t size,
				unsigned long base, unsigned long limit)
{
	if (offset <= base && (offset + size) > base)
		return true;
	if (offset < limit && (offset + size) > limit)
		return true;
	return false;
}

inline bool mem_is_overlap_range(unsigned long offset, size_t size,
				 unsigned long base, unsigned long limit)
{
	if (mem_is_within_range(offset, size, base, limit))
		return true;
	if (mem_is_some_overlap(offset, size, base, limit))
		return true;
	return false;
}

static int clear_pages(struct page **pages, int num, pgprot_t pgprot)
{
	void *addr = vmap(pages, num, VM_MAP, pgprot);

	if (!addr)
		return -ENOMEM;
	memset(addr, 0, PAGE_SIZE * num);
	vunmap(addr);

	return 0;
}

static int sglist_zero(struct scatterlist *sgl, unsigned int nents,
			   pgprot_t pgprot)
{
	int p = 0;
	int ret = 0;
	struct sg_page_iter piter;
	struct page *pages[32];

	for_each_sg_page(sgl, &piter, nents, 0) {
		pages[p++] = sg_page_iter_page(&piter);
		if (p == ARRAY_SIZE(pages)) {
			ret = clear_pages(pages, p, pgprot);
			if (ret)
				return ret;
			p = 0;
		}
	}
	if (p)
		ret = clear_pages(pages, p, pgprot);

	return ret;
}

int pages_zero(struct page *page, size_t size, pgprot_t pgprot)
{
	struct scatterlist sg;

	sg_init_table(&sg, 1);
	sg_set_page(&sg, page, size, 0);
	return sglist_zero(&sg, 1, pgprot);
}

struct pool_info *ion_gen_pool_create(enum pool_type type,
				      unsigned long base,
				      size_t size,
				      unsigned long rtk_flags,
				      bool from_reserved_memory,
				      struct device *dev)
{
	struct pool_info *ret = NULL;

	do {
		struct gen_pool_info *pool;
		struct pool_info *common;

		pool = (struct gen_pool_info *) kzalloc(sizeof(struct gen_pool_info), GFP_KERNEL);

		if (!pool)
			break;

		common = &pool->common;

		init_protected_info(&pool->protected_info);
		INIT_LIST_HEAD(&pool->pages);
		mutex_init(&pool->lock);

		if (rtk_flag_is_protected_static(rtk_flags)) {
			if (from_reserved_memory) {
				struct ion_rtk_protected_create_info info;
				info.mem.base = base;
				info.mem.size = size;
				info.mem.type =
					rtk_flags_to_notifier_protected_type(rtk_flags);
				if (ion_rtk_protected_create_notify(&info) != NOTIFY_OK) {
					pr_err("%s : Protected area creation failed! (rtk_flags = 0x%lx)\n",
						 __func__, rtk_flags);
					rtk_flags &=
						~(RTK_FLAG_PROTECTED_DYNAMIC |
								RTK_FLAG_PROTECTED_MASK);
				}
			}
		}

		if (rtk_flag_is_protected_static(rtk_flags)) {
			set_protected_range(&pool->protected_info,
					base, base+size);
		}

		pool_common_init(common, type, base, size, rtk_flags);
		common->alloc = ion_gen_pool_alloc;
		common->getSpace = ion_gen_pool_getSpace;
		common->getUsage = ion_gen_pool_getUsage;
		common->debug_show = ion_gen_pool_debug_show;
		common->destroy = ion_gen_pool_destroy;

		pool->gpool = gen_pool_create(PAGE_SHIFT, -1);
		if (!pool->gpool) {
			kfree(pool);
			break;
		}

		if (rtk_flag_canAccess(rtk_flags)) {
			struct page *page = pfn_to_page(PFN_DOWN(base));
			ion_pages_sync_for_device(dev, page, size, DMA_BIDIRECTIONAL);
			pages_zero(page, size, pgprot_writecombine(PAGE_KERNEL));
		}

		gen_pool_set_algo(pool->gpool, ion_gen_pool_algo, (void *)pool);
		gen_pool_add(pool->gpool, base, size, -1);

		ret = common;
		break;
	} while (0);
	return ret;
}

static void ion_gen_pool_destroy(struct pool_info *pool_info)
{
	struct gen_pool_info *pool =
	    container_of(pool_info, struct gen_pool_info, common);
	if (!pool)
		return;

	mutex_lock(&pool->lock);
	do {
		struct gen_page_info *page_info, *tmp_page_info;
		list_for_each_entry_safe(page_info, tmp_page_info, &pool->pages,
					 list) {
			// TODO ADD LOG
			list_del(&page_info->list);
			ion_gen_page_free(page_info);
		}

		gen_pool_destroy(pool->gpool);

		ion_gen_pool_refresh_protected_range(pool);
	} while (0);
	mutex_unlock(&pool->lock);
	mutex_destroy(&pool->lock);
	kfree(pool);
}

static struct page_info *ion_gen_pool_alloc(struct pool_info *pool_info,
					    unsigned long size,
					    unsigned long align,
					    unsigned long flags)
{
	struct page_info *ret = NULL;
	struct gen_pool_info *pool =
	    container_of(pool_info, struct gen_pool_info, common);
	struct pool_info *common = &pool->common;
	mutex_lock(&pool->lock);
	do {
		struct gen_page_info *page = NULL;
		unsigned long offset = 0;

		if (ion_flag_mismatch_pool_condition(flags, common->rtk_flags))
			break;

		page = ion_gen_page_alloc();
		if (!page)
			break;

		if (rtk_flag_is_protected_dynamic(common->rtk_flags)) {
			if (ion_flag_is_protected(flags))
				pool->algo_mode = ALGO_LAST_FIT;
			else
				pool->algo_mode = ALGO_FIRST_FIT;
		} else {
			if (ion_flag_is_algo_last_fit(flags))
				pool->algo_mode = ALGO_LAST_FIT;
			else
				pool->algo_mode = ALGO_FIRST_FIT;
		}

		offset = gen_pool_alloc(pool->gpool, size);
		if (offset == 0) {
			ion_gen_page_free(page);
			break;
		}

		if (rtk_flag_is_protected_dynamic(common->rtk_flags)) {
			if (!ion_gen_pool_check_region_is_safe
			    (pool, ion_flag_is_protected(flags), offset,
			     size)) {
				gen_pool_free(pool->gpool, offset, size);
				ion_gen_page_free(page);
				break;
			}
		}

		if (rtk_flag_is_protected(common->rtk_flags) &&
				!rtk_flag_has_protected_ext(common->rtk_flags) &&
				ion_flag_has_protected_ext(flags)) {

			page->protected_ext_info = create_protected_ext_info(
					ion_flag_to_notifier_protected_ext(flags),
					offset, size,
					&pool->protected_info);

			if (!page->protected_ext_info) {
				gen_pool_free(pool->gpool, offset, size);
				ion_gen_page_free(page);
				break;
			}
		}

		ion_gen_page_info_init(page, offset, size, flags,
				       common->rtk_flags);

		list_add(&page->list, &pool->pages);
		page->parent = pool;

		ret = &page->common;
	} while (0);
	mutex_unlock(&pool->lock);
	return ret;
}

static unsigned long ion_gen_pool_algo_last_fit(unsigned long *map,
						unsigned long size,
						unsigned long start,
						unsigned int nr, void *data)
{
	unsigned long start_bit = size;
	unsigned long index = size;

	if (start_bit <= nr)
		return index;

	do {
		if (start_bit > nr) {
			unsigned long start_bit_next = start_bit;
			start_bit_next -= nr;
			start_bit_next =
			    find_next_bit(map, size, start_bit_next) - nr;
			if (start_bit_next >= start_bit) {
				pr_err
				    ("[%s:%d] Warning! start_bit=%ld start_bit_next=%ld size=%ld nr=%d\n",
				     __func__, __LINE__, start_bit,
				     start_bit_next, size, nr);
				break;
			}
			start_bit = start_bit_next;
		} else
			start_bit -= 1;

		index = bitmap_find_next_zero_area(map, size, start_bit, nr, 0);

		if (start_bit <= 0)
			break;

	} while (index >= size);

	return index;
}

static unsigned long ion_gen_pool_algo(unsigned long *map,
				unsigned long size,
				unsigned long start,
				unsigned int nr,
				void *data,
				struct gen_pool *pool,
				unsigned long start_addr)
{
	struct gen_pool_info *pool_t = (struct gen_pool_info *)data;

	if (pool_t) {
		switch (pool_t->algo_mode) {
		case ALGO_FIRST_FIT:
			return gen_pool_first_fit(map, size, start, nr, data,
						  NULL, 0);
		case ALGO_LAST_FIT:
			return ion_gen_pool_algo_last_fit(map, size, start, nr,
							  data);
		default:
			break;
		}
	}

	pr_err("[%s] Warning: pool_t=%p", __func__, pool_t);
	if (pool_t != NULL)
		pr_err(" algo_mode=0x%x", pool_t->algo_mode);
	pr_err(" size=%ld start=%ld, nr=%d\n", size, start, nr);
	return gen_pool_first_fit(map, size, start, nr, data, NULL, 0);
}

static void ion_gen_page_destroy(struct page_info *page_info)
{
	struct gen_page_info *page =
	    container_of(page_info, struct gen_page_info, common);
	struct gen_pool_info *pool = page->parent;
	mutex_lock(&pool->lock);
	do {
		list_del(&page->list);

		if (page->protected_ext_info) {
			if (destroy_protected_ext_info(page->protected_ext_info) != 0)
				pr_err("[%s] ion_rtk_protected_ext_unset_notify return ERROR! (base=0x%08lx, size=0x%lx, flags=0x%lx)\n", __func__,
						page->common.base, page->common.size, page->common.flags);
			page->protected_ext_info = NULL;
		}

		if (rtk_flag_is_protected_dynamic(pool->common.rtk_flags) &&
		    ion_flag_is_protected(page->common.flags))
			ion_gen_pool_refresh_protected_range(pool);

		gen_pool_free(pool->gpool, page->common.base,
			      page->common.size);

		ion_gen_page_free(page);
		break;
	} while (0);
	mutex_unlock(&pool->lock);
}

static struct sg_table *ion_gen_page_get_sg_table(struct page_info *page_info)
{
	struct gen_page_info *page =
	    container_of(page_info, struct gen_page_info, common);
	return page->sg_table;
}

static size_t ion_gen_pool_pages_size(struct gen_pool_info *pool,
				      unsigned long flags)
{
	size_t ret = 0;
	struct gen_page_info *info;
	list_for_each_entry(info, &pool->pages, list) {
		if (info && ion_flag_match_condition(flags, info->common.flags))
			ret += info->common.size;
	}
	return ret;
}

static size_t ion_gen_pool_getSpace(struct pool_info *pool_info,
				    unsigned long flags)
{
	size_t ret = 0;
	struct gen_pool_info *pool =
	    container_of(pool_info, struct gen_pool_info, common);
	mutex_lock(&pool->lock);
	do {
		if (ion_flag_mismatch_pool_condition
		    (flags, pool->common.rtk_flags))
			break;

		ret = pool->common.size;
		ret -= ion_gen_pool_pages_size(pool, 0);
		break;
	} while (0);
	mutex_unlock(&pool->lock);
	return ret;
}

static size_t ion_gen_pool_getUsage(struct pool_info *pool_info,
				    unsigned long flags)
{
	size_t ret = 0;
	struct gen_pool_info *pool =
	    container_of(pool_info, struct gen_pool_info, common);
	mutex_lock(&pool->lock);
	do {
		if (ion_flag_mismatch_pool_condition
		    (flags, pool->common.rtk_flags))
			break;

		ret = ion_gen_pool_pages_size(pool, flags);
		break;
	} while (0);
	mutex_unlock(&pool->lock);
	return ret;
}

static int ion_gen_pool_debug_show(struct pool_info *pool_info,
				   struct seq_buf *s, char *prefix)
{
	int ret = 0;
	struct gen_pool_info *pool =
	    container_of(pool_info, struct gen_pool_info, common);
	if (!prefix)
		prefix = "";
	mutex_lock(&pool->lock);
	pool_common_debug_show(pool_info, s, prefix);
	seq_buf_printf(s, "%s    Usage : %zu\n", prefix,
		   ion_gen_pool_pages_size(pool, 0));
	do {
		char *space_prefix = "    ";
		char *sub_prefix =
		    kzalloc(strlen(prefix) + strlen(space_prefix) + 1, GFP_KERNEL);
		if (!sub_prefix)
			break;
		strcpy(sub_prefix, prefix);
		strcat(sub_prefix, space_prefix);
		{
			struct gen_page_info *info;
			list_for_each_entry(info, &pool->pages, list) {
				if (info)
					page_common_debug_show(&info->common, s,
							       sub_prefix);
			}
		}
		{
			struct gen_page_info *info;
			unsigned long slot_offset = pool->common.base;
			size_t slot_size = 0;
			unsigned long next_offset, close_offet;
			seq_buf_printf(s, "%s+---------------------+\n",
				   sub_prefix);
FIND_NEXD:
			next_offset = slot_offset + slot_size;
			close_offet = pool->common.base + pool->common.size;
			list_for_each_entry(info, &pool->pages, list) {
				if (!info)
					continue;
				if (info->common.base == next_offset) {
					slot_size += info->common.size;
					goto FIND_NEXD;
				}
				if (info->common.base > next_offset
				    && info->common.base < close_offet)
					close_offet = info->common.base;
			}

			{
				unsigned long next_search_offset;
				if (slot_size) {
					seq_buf_printf(s,
						   "%s| %08lx ~ %08lx | used %10zu\n",
						   sub_prefix, slot_offset,
						   slot_offset + slot_size - 1,
						   slot_size);
					next_search_offset =
					    slot_offset + slot_size;
				} else {
					seq_buf_printf(s,
						   "%s| %08lx ~ %08lx | free %10zu\n",
						   sub_prefix, slot_offset,
						   close_offet - 1,
						   (size_t) close_offet -
						   slot_offset);
					next_search_offset = close_offet;
				}

				if (next_search_offset <
				    (pool->common.base + pool->common.size)) {
					slot_offset = next_search_offset;
					slot_size = 0;
					goto FIND_NEXD;
				}
			}
			seq_buf_printf(s, "%s+---------------------+\n",
				   sub_prefix);
		}

		kfree(sub_prefix);
		break;
	} while (0);
	mutex_unlock(&pool->lock);
	return ret;
}

static struct gen_page_info *ion_gen_page_alloc(void)
{
	struct gen_page_info *ret = NULL;
	do {
		struct gen_page_info *page = (struct gen_page_info *)
		    kzalloc(sizeof(struct gen_page_info), GFP_KERNEL);
		if (!page)
			break;

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

static void ion_gen_page_free(struct gen_page_info *page)
{
	sg_free_table(page->sg_table);
	kfree(page->sg_table);
	kfree(page);
}

static void ion_gen_page_info_init(struct gen_page_info *page,
				   unsigned long base, size_t size,
				   unsigned long flags, unsigned long rtk_flags)
{
	page_common_init(&page->common, base, size,
			 flags | ion_flag_from_rtk(rtk_flags));
	sg_set_page(page->sg_table->sgl, pfn_to_page(PFN_DOWN(base)), size, 0);

	{
		size_t i;
		struct scatterlist *sg;
		for_each_sg(page->sg_table->sgl, sg, page->sg_table->nents, i) {
			sg_dma_address(sg) = sg_phys(sg);
			sg_dma_len(sg) = sg->length;
		}
	}

	page->common.destroy = ion_gen_page_destroy;
	page->common.get_sg_table = ion_gen_page_get_sg_table;
	page->common.sync_range = page_common_sync_range;
	page->common.sync = page_common_sync;
	INIT_LIST_HEAD(&page->list);

}

static bool ion_gen_pool_check_region_is_safe(struct gen_pool_info *pool,
					      bool protected,
					      unsigned long offset, size_t size)
{
	bool safe = false;
	do {
		struct RTK_PROTECTED_INFO *protected_info =
		    &pool->protected_info;
		unsigned long protected_base =
		    get_protected_base(protected_info);
		unsigned long protected_limit =
		    get_protected_limit(protected_info);
		size_t protected_size = get_protected_size(protected_info);

		if (protected) {
			if (mem_is_within_range
			    (offset, size, protected_base, protected_limit)) {
				safe = true;
				break;
			}

			if (protected_size == 0) {
				struct ion_rtk_protected_create_info info;
				info.mem.base = offset;
				info.mem.size = size;
				info.mem.type =
				    rtk_flags_to_notifier_protected_type
				    (pool->common.rtk_flags);
				if (ion_rtk_protected_create_notify(&info) !=
				    NOTIFY_OK) {
					pr_err
					    ("%s:%d ion_rtk_protected_create_notify return ERROR! (priv_virt=%p)\n",
					     __func__, __LINE__,
					     get_protected_priv
					     (protected_info));
					break;
				}
				set_protected_range(protected_info, offset,
						    offset + size);
				set_protected_priv(protected_info,
						   info.priv_virt);
				safe = true;
				break;
			} else if (offset == protected_limit) {
				unsigned long next_protected_limit =
				    offset + size;
				struct ion_rtk_protected_change_info info;
				info.mem.base = protected_base;
				info.mem.size =
				    next_protected_limit - protected_base;
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
					break;
				}
				set_protected_range(protected_info,
						    protected_base,
						    next_protected_limit);
				safe = true;
				break;
			} else if ((offset + size) == protected_base) {
				unsigned long next_protected_base = offset;
				struct ion_rtk_protected_change_info info;
				info.mem.base = next_protected_base;
				info.mem.size =
				    protected_limit - next_protected_base;
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
					break;
				}
				set_protected_range(protected_info,
						    next_protected_base,
						    protected_limit);
				safe = true;
				break;
			}

			break;
		} else {
			if (!mem_is_overlap_range
			    (offset, size, protected_base, protected_limit)) {
				safe = true;
				break;
			}
			break;
		}

		break;
	} while (0);
	return safe;
}

static void ion_gen_pool_refresh_protected_range(struct gen_pool_info *pool)
{
	struct pool_info *pool_info = &pool->common;
	do {
		struct RTK_PROTECTED_INFO *protected_info =
		    &pool->protected_info;
		unsigned long protected_base =
		    get_protected_base(protected_info);
		unsigned long protected_limit =
		    get_protected_limit(protected_info);
		size_t protected_size = get_protected_size(protected_info);
		unsigned long next_protected_base =
		    pool_info->base + pool_info->size;
		unsigned long next_protected_limit = pool_info->base;
		int protected_count = 0;

		struct gen_page_info *page, *tmp_page;

		if (protected_size == 0)
			break;

		list_for_each_entry_safe(page, tmp_page, &pool->pages, list) {
			struct page_info *page_info = &page->common;
			if (ion_flag_is_protected(page_info->flags)) {
				unsigned long base = page_info->base;
				unsigned long limit =
				    page_info->base + page_info->size;
				if (next_protected_limit < limit)
					next_protected_limit = limit;
				if (next_protected_base > base)
					next_protected_base = base;
				protected_count++;
			}
		}

		if (protected_count == 0) {
			struct ion_rtk_protected_destroy_info info;
			info.priv_virt = get_protected_priv(protected_info);
			if (ion_rtk_protected_destroy_notify(&info) !=
			    NOTIFY_OK) {
				pr_err
				    ("%s:%d ion_rtk_protected_destroy_notify return ERROR! (priv_virt=%p)\n",
				     __func__, __LINE__,
				     get_protected_priv(protected_info));
			}
			init_protected_info(protected_info);
			break;
		}

		if (next_protected_base != protected_base
		    || next_protected_limit != protected_limit) {
			struct ion_rtk_protected_change_info info;
			info.mem.base = next_protected_base;
			info.mem.size =
			    next_protected_limit - next_protected_base;
			info.mem.type =
			    rtk_flags_to_notifier_protected_type(pool->common.
								 rtk_flags);
			info.priv_virt = get_protected_priv(protected_info);
			if (ion_rtk_protected_change_notify(&info) != NOTIFY_OK) {
				pr_err
				    ("%s:%d ion_rtk_protected_change_notify return ERROR! (priv_virt=%p)\n",
				     __func__, __LINE__,
				     get_protected_priv(protected_info));
				break;
			}
			set_protected_range(protected_info, next_protected_base,
					    next_protected_limit);
		}
	} while (0);
}

void ion_pages_sync_for_device(struct device *dev, struct page *page,
			       size_t size, enum dma_data_direction dir)
{
	struct scatterlist sg;
	int nents;

	sg_init_table(&sg, 1);
	sg_set_page(&sg, page, size, 0);
	/*
	 * This is not correct - sg_dma_address needs a dma_addr_t that is valid
	 * for the targeted device, but this works on the currently targeted
	 * hardware.
	 */
	sg_dma_address(&sg) = page_to_phys(page);
	dma_mask = 0xffffffff;
	dev->dma_mask = &dma_mask;
	nents = dma_map_sg(dev, &sg, 1, dir);
	dma_sync_sg_for_device(dev, &sg, 1, dir);
	if (nents > 0)
		dma_unmap_sg(dev, &sg, 1, dir);
}

#ifdef CONFIG_VIRTUAL_PMU

/*
 * ref. from static int ion_gen_pool_debug_show(
 */
unsigned long long ion_gen_pool_get_size(struct pool_info *pool_info)
{
	size_t pool_pages_size;
	struct gen_pool_info *pool =
	    container_of(pool_info, struct gen_pool_info, common);
	struct gen_page_info *info;

	pool_pages_size = 0;

	info = NULL;

	mutex_lock(&pool->lock);

	//pool_pages_size =
	//    (unsigned long long) (ion_gen_pool_pages_size(pool, 0));
	list_for_each_entry(info, &pool->pages, list) {
		//if (info && ion_flag_match_condition(0, info->common.flags))
		if (info) {
			pool_pages_size += info->common.size;
		}
	}

	mutex_unlock(&pool->lock);

	return (unsigned long long)pool_pages_size;
}

#endif
