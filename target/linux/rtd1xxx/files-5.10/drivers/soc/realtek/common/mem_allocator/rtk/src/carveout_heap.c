/* ion_rtk_carveout_heap.c
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-map-ops.h>
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/sort.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/seq_buf.h>
#include <linux/sched/clock.h>
#include <linux/sched/task.h>

#include <soc/realtek/uapi/ion_rtk.h>

#include "carveout_heap.h"
#include "pool.h"

#include "../../../../staging/android/ion/ion_private.h"

extern struct device *idev;

struct prealloc_pool {
	struct ion_buffer buffer;
	struct pool_info * pool;
	struct list_head list;
};

struct ion_rtk_carveout_heap {
	struct ion_heap heap;
	unsigned long version;
	struct ion_rtk_carveout_ops *priv_ops;
	struct mutex lock;
	struct list_head pools;
	struct list_head prealloc_pools;
	struct list_head ion_flag_replace;
	struct seq_buf trace_seq_buf;
	char *trace_message;
};

struct pool_score_slot {
	struct pool_info *pool_info;
	int score;
};

unsigned int retry_count_value = 5;
unsigned int retry_delay_value = 200;
EXPORT_SYMBOL(retry_count_value);
EXPORT_SYMBOL(retry_delay_value);

static size_t heap_pool_count(struct ion_rtk_carveout_heap *rtk_carveout_heap);
static int pool_score_cmp(const void *r, const void *c);
static int pool_score(struct pool_info *pool_info,
		      unsigned long size, unsigned long flags);

static unsigned int ion_rtk_carveout_get_version(struct ion_heap *heap);
static int ion_rtk_carveout_get_meminfo(struct ion_heap *heap,
					unsigned int flags,
					struct ion_rtk_carveout_meminfo *info);
static int ion_rtk_carveout_trace_dump(struct ion_heap *heap,
				       struct seq_file *s);

static struct ion_rtk_carveout_ops rtk_carveout_heap_priv_ops = {
	.getVersion = ion_rtk_carveout_get_version,
	.getMemInfo = ion_rtk_carveout_get_meminfo,
	.traceDump = ion_rtk_carveout_trace_dump,
};

struct ion_rtk_carveout_ops *get_rtk_carveout_ops(struct ion_heap *heap);

static int ion_carveout_heap_dump(struct ion_rtk_carveout_heap
				  *rtk_carveout_heap, struct seq_buf *s,
				  void *prefix);
int ion_carveout_heap_debug_show(struct ion_heap *heap,
				 struct seq_file *s, void *unused);
static int ion_rtk_carveout_heap_allocate(struct ion_heap *heap,
					  struct ion_buffer *buffer,
					  unsigned long size,
					  unsigned long flags);
static void ion_rtk_carveout_heap_free(struct ion_buffer *buffer);
static struct ion_heap_ops rtk_carveout_heap_ops = {
	.allocate = ion_rtk_carveout_heap_allocate,
	.free = ion_rtk_carveout_heap_free,
};

struct ion_heap *ion_rtk_carveout_heap_create(struct device *dev,
					      struct ion_platform_heap *heap_data);
void ion_rtk_carveout_heap_destroy(struct ion_heap *heap);

/*
 *
 */
#ifdef CONFIG_VIRTUAL_PMU

extern int rtk_vpmu_api_add_to_available(unsigned long addr);
extern int rtk_vpmu_api_is_enable(int vpmu_reg_idx);
extern void rtk_vpmu_api_update_count(unsigned long long count, int vpmu_reg_idx);

static int vpmu_reg_idx_media_gen;
static int vpmu_reg_idx_media_cma;
static int vpmu_reg_idx_audio_gen;
static int vpmu_reg_idx_audio_cma;

extern unsigned long long ion_gen_pool_get_size(struct pool_info *pool_info);
extern unsigned long long ion_cma_pool_get_size(struct pool_info *pool_info);
/*
 *
 */
unsigned long long
ion_get_heap_size_info(struct ion_heap *heap, enum pool_type type)
{
	char symname[KSYM_NAME_LEN];
	unsigned long long allocSize;
	struct ion_rtk_carveout_heap *rtk_carveout_heap =
	    container_of(heap, struct ion_rtk_carveout_heap, heap);

	allocSize = 0;
	//printk(KERN_ERR "\033[1;33m" "|   %s %d: pool type %d" "\033[m\n", __FUNCTION__, __LINE__, type);

	mutex_lock(&rtk_carveout_heap->lock);

	do {
		struct pool_info *pool_info = NULL;

		list_for_each_entry(pool_info, &rtk_carveout_heap->pools, list) {
			if (!pool_info) {
				continue;
			}
			//pool_info->debug_show(pool_info, s, prefix);
			//ion_gen_pool_debug_show ==> ion_gen_pool_get_size
			//ion_cma_pool_debug_show ==> ion_cma_pool_get_size
			if( pool_info->type == type &&
				type == GEN_POOL ) {
				//printk(KERN_ERR "\033[1;33m" "|    %s %d: pool_info->type %d, pool type %d, get GEN pool used size" "\033[m\n",
				//	__FUNCTION__, __LINE__, pool_info->type, type);
				allocSize += ion_gen_pool_get_size(pool_info);
			}
			else if( pool_info->type == type &&
				type == CMA_POOL ) {
				//printk(KERN_ERR "\033[1;33m" "|    %s %d: pool_info->type %d, pool type %d, get CMA pool used size" "\033[m\n",
				//	__FUNCTION__, __LINE__, pool_info->type, type);
				allocSize += ion_cma_pool_get_size(pool_info);
			}

			//if( lookup_symbol_name(pool_info->debug_show,symname) == 0 )
			//	printk(KERN_ERR "\033[1;33m" "|   %s %d: %p [%s]" "\033[m\n", __FUNCTION__, __LINE__, pool_info->debug_show, symname);
			//else
			//	printk(KERN_ERR "\033[1;33m" "|   %s %d: %p" "\033[m\n", __FUNCTION__, __LINE__, pool_info->debug_show);
			//allocSize += 1000000;
		}
	}
	while (0);

	mutex_unlock(&rtk_carveout_heap->lock);

	return allocSize;
}

void ion_update_media_heap_size_info_in_gen(struct ion_heap *heap, int vpmu_reg_idx)
{
	//struct ion_rtk_carveout_heap *rtk_carveout_heap =
	//	container_of(heap, struct ion_rtk_carveout_heap, heap);
	//struct ION_RTK_CARVEOUT_PAGE_INFO * page_info;
	//struct ION_RTK_CARVEOUT_POOL_INFO * phandler;
	//unsigned long offset;
	unsigned long long allocSize;

	if (heap &&
	    (int)heap->type == 7 /* (int)RTK_PHOENIX_ION_HEAP_TYPE_MEDIA */) {
		allocSize = ion_get_heap_size_info(heap, GEN_POOL);
		rtk_vpmu_api_update_count(allocSize, vpmu_reg_idx);
	}
}

void ion_update_media_heap_size_info_in_cma(struct ion_heap *heap, int vpmu_reg_idx)
{
	//struct ion_rtk_carveout_heap *rtk_carveout_heap =
	//	container_of(heap, struct ion_rtk_carveout_heap, heap);
	//struct ION_RTK_CARVEOUT_PAGE_INFO * page_info;
	//struct ION_RTK_CARVEOUT_POOL_INFO * phandler;
	//unsigned long offset;
	unsigned long long allocSize;

	if (heap &&
	    (int)heap->type == 7 /* (int)RTK_PHOENIX_ION_HEAP_TYPE_MEDIA */) {
		allocSize = ion_get_heap_size_info(heap, CMA_POOL);
		rtk_vpmu_api_update_count(allocSize, vpmu_reg_idx);
	}
}

void ion_update_audio_heap_size_info_in_gen(struct ion_heap *heap, int vpmu_reg_idx)
{
	//struct ion_rtk_carveout_heap *rtk_carveout_heap =
	//	container_of(heap, struct ion_rtk_carveout_heap, heap);
	//struct ION_RTK_CARVEOUT_PAGE_INFO * page_info;
	//struct ION_RTK_CARVEOUT_POOL_INFO * phandler;
	//unsigned long offset;
	unsigned long long allocSize;

	if (heap &&
	    (int)heap->type == 8 /* (int)RTK_PHOENIX_ION_HEAP_TYPE_AUDIO */) {
		allocSize = ion_get_heap_size_info(heap, GEN_POOL);
		rtk_vpmu_api_update_count(allocSize, vpmu_reg_idx);
	}
}

void ion_update_audio_heap_size_info_in_cma(struct ion_heap *heap, int vpmu_reg_idx)
{
	//struct ion_rtk_carveout_heap *rtk_carveout_heap =
	//	container_of(heap, struct ion_rtk_carveout_heap, heap);
	//struct ION_RTK_CARVEOUT_PAGE_INFO * page_info;
	//struct ION_RTK_CARVEOUT_POOL_INFO * phandler;
	//unsigned long offset;
	unsigned long long allocSize;

	if (heap &&
	    (int)heap->type == 8 /* (int)RTK_PHOENIX_ION_HEAP_TYPE_AUDIO */) {

		allocSize = ion_get_heap_size_info(heap, CMA_POOL);

		rtk_vpmu_api_update_count(allocSize, vpmu_reg_idx);
	}
}

void ion_update_heap_size_info(struct ion_heap *heap)
{
	// only update if function is added to /proc/rtk_vpmu/filter
	if( rtk_vpmu_api_is_enable(vpmu_reg_idx_media_gen) )
	{
		ion_update_media_heap_size_info_in_gen(heap,vpmu_reg_idx_media_gen);
	}
	if( rtk_vpmu_api_is_enable(vpmu_reg_idx_media_cma) )
	{
		ion_update_media_heap_size_info_in_cma(heap,vpmu_reg_idx_media_cma);
	}
	if( rtk_vpmu_api_is_enable(vpmu_reg_idx_audio_gen) )
	{
		ion_update_audio_heap_size_info_in_gen(heap,vpmu_reg_idx_audio_gen);
	}
	if( rtk_vpmu_api_is_enable(vpmu_reg_idx_audio_cma) )
	{
		ion_update_audio_heap_size_info_in_cma(heap,vpmu_reg_idx_audio_cma);
	}
}
#endif // end of CONFIG_VIRTUAL_PMU

int trace_seq_buf_print_seq(struct seq_file *m, struct seq_buf *s)
{
	unsigned int len = seq_buf_used(s);

	return seq_write(m, s->buffer, len);
}

static size_t print_time(u64 ts, char *buf)
{
	unsigned long rem_nsec;
	rem_nsec = do_div(ts, 1000000000);
	if (!buf)
		return snprintf(NULL, 0, "[%5lu.000000] ", (unsigned long)ts);

	return sprintf(buf, "[%5lu.%06lu] ",
			(unsigned long)ts, rem_nsec / 1000);
}

#ifdef CONFIG_DEBUG_FS
static int debugfs_retry_count_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%d\n", retry_count_value);
	return 0;
}
static int debugfs_retry_delay_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%d\n", retry_delay_value);
	return 0;
}

static int debugfs_retry_count_open(struct inode *inode, struct file *file)
{
	return single_open(file, debugfs_retry_count_show, inode->i_private);
}

static int debugfs_retry_delay_open(struct inode *inode, struct file *file)
{
	return single_open(file, debugfs_retry_delay_show, inode->i_private);
}

static ssize_t debugfs_retry_count_write(struct file *file, const char __user *userbuf,
					size_t count, loff_t *ppos)
{
	char retry_buf[10]= {0};
	int ret = 0;

	if (*ppos > 10)
		return -EINVAL;
	if (copy_from_user(retry_buf, userbuf, min(count, sizeof(retry_buf))))
		return -EFAULT;
	ret = kstrtouint(retry_buf, 10, &retry_count_value);
	return  count;
}

static ssize_t debugfs_retry_delay_write(struct file *file, const char __user *userbuf,
					size_t count, loff_t *ppos)
{
	char retry_buf[10] = {0};
	int ret = 0;

	if (*ppos > 10)
		return -EINVAL;
	if (copy_from_user(retry_buf, userbuf, min(count, sizeof(retry_buf))))
		return -EFAULT;
	ret = kstrtouint(retry_buf, 10, &retry_delay_value);
	return  count;
}


static const struct file_operations debug_retry_delay_fops = {
	.open = debugfs_retry_delay_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = debugfs_retry_delay_write,
};


static const struct file_operations debug_retry_count_fops = {
	.open = debugfs_retry_count_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = debugfs_retry_count_write,
};
#endif
static int pool_score_cmp(const void *r, const void *c)
{
	struct pool_score_slot *slot_r = (struct pool_score_slot *)r;
	struct pool_score_slot *slot_c = (struct pool_score_slot *)c;
	return slot_c->score - slot_r->score;
}

static int pool_score(struct pool_info *pool_info,
		      unsigned long size, unsigned long flags)
{
	const int step = 100;
	const int half_step = step / 2;
	int score = -1;
	do {
		size_t space;

		if (ion_flag_mismatch_pool_condition
		    (flags, pool_info->rtk_flags))
			break;

		space = pool_info->getSpace(pool_info, flags);

		if (space < size || space <= 0)
			break;

		score =
		    (sizeof(flags) * 8 * step) + half_step /* propected */  +
		    half_step /* cma */  + 1;

		if (rtk_flag_is_protected(pool_info->rtk_flags))
			score -= half_step;

		score -=
		    hweight_long((pool_info->rtk_flags &
				  ~(RTK_FLAG_PROTECTED_MASK | RTK_FLAG_PROTECTED_EXT_MASK))) * step;

		if (pool_info->type == CMA_POOL)
			score -= half_step;

		if (pool_info->type == PREALLOC_POOL)
			score += half_step;

		break;
	} while (0);
	return score;
}

static struct pool_info * ion_rtk_carveout_heap_find_pool_info(struct ion_rtk_carveout_heap *rtk_carveout_heap, unsigned long base)
{
	struct pool_info * ret = NULL;
	struct pool_info * pool_info;
	list_for_each_entry(pool_info, &rtk_carveout_heap->pools, list) {
		if (pool_info && base >= pool_info->base && base < (pool_info->base + pool_info->size)) {
			ret = pool_info;
			break;
		}
	}
	return ret;
}

static bool ion_buffer_is_protected(struct ion_buffer *buffer)
{
	unsigned int prot_flags;

	/* buffer->flags[BIT24 BIT25 BIT26]: 3b'001 ~ 3b'111 (6 types) */
	prot_flags = ION_PROTECTED_TYPE_GET(buffer->flags);

	/* add your code here for more protected types support */
	if( prot_flags /*== 2*/ )
		return true;

	return false;
}

static struct sg_table *rtk_ion_map_dma_buf(struct dma_buf_attachment *attachment,
					enum dma_data_direction direction)
{
	struct ion_buffer *buffer = attachment->dmabuf->priv;
	struct ion_dma_buf_attachment *a;
	struct sg_table *table;
	unsigned long attrs = attachment->dma_map_attrs;

	a = attachment->priv;
	table = a->table;

	if (attachment && attachment->priv) {
		if( buffer && ion_buffer_is_protected(buffer)) {
			if (!dma_map_sg_attrs(attachment->dev, table->sgl, table->nents,
				direction, DMA_ATTR_SKIP_CPU_SYNC))
				return ERR_PTR(-ENOMEM);
			return table;
		}
	}

	if (!(buffer->flags & ION_FLAG_CACHED))
		attrs |= DMA_ATTR_SKIP_CPU_SYNC;

	if (!dma_map_sg_attrs(attachment->dev, table->sgl, table->nents,
			      direction, attrs))
		return ERR_PTR(-ENOMEM);

	a->mapped = true;

	return table;
}

static void rtk_ion_unmap_dma_buf(struct dma_buf_attachment *attachment,
				  struct sg_table *table,
				  enum dma_data_direction direction)
{
	struct ion_buffer *buffer = attachment->dmabuf->priv;
	struct ion_dma_buf_attachment *a = attachment->priv;
	unsigned long attrs = attachment->dma_map_attrs;

	if (attachment && attachment->priv) {
		if (buffer && ion_buffer_is_protected(buffer)) {
			dma_unmap_sg_attrs(attachment->dev, table->sgl, table->nents,
				direction, DMA_ATTR_SKIP_CPU_SYNC);
			return;
		}
	}

	a->mapped = false;

	if (!(buffer->flags & ION_FLAG_CACHED))
		attrs |= DMA_ATTR_SKIP_CPU_SYNC;

	dma_unmap_sg_attrs(attachment->dev, table->sgl, table->nents,
			   direction, attrs);
}

static const struct dma_buf_ops rtk_dma_buf_ops = {
	.map_dma_buf = rtk_ion_map_dma_buf,
	.unmap_dma_buf = rtk_ion_unmap_dma_buf,
};

struct ion_heap *ion_rtk_carveout_heap_create(struct device *dev,
					      struct ion_platform_heap *heap_data)
{
	struct ion_rtk_carveout_heap *rtk_carveout_heap;
	struct ion_rtk_heap_create_priv * priv_data = (struct ion_rtk_heap_create_priv *)heap_data->priv;
	struct list_head *pool_list = &priv_data->pools;
	struct ion_rtk_priv_pool *pool;
	struct dentry __maybe_unused *debug_root;

	if (!pool_list) {
		pr_err("[%s] pool_list is NULL!!!!\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	rtk_carveout_heap =
	    kzalloc(sizeof(struct ion_rtk_carveout_heap), GFP_KERNEL);

	if (!rtk_carveout_heap)
		return ERR_PTR(-ENOMEM);

	{
		const size_t seq_buf_size = PAGE_SIZE * 10;
		rtk_carveout_heap->trace_message =
		    kzalloc(seq_buf_size, GFP_KERNEL);
		if (!rtk_carveout_heap->trace_message) {
			kfree(rtk_carveout_heap);
			return ERR_PTR(-ENOMEM);
		}
		seq_buf_init(&rtk_carveout_heap->trace_seq_buf,
			     rtk_carveout_heap->trace_message, seq_buf_size);
	}

	mutex_init(&rtk_carveout_heap->lock);
	INIT_LIST_HEAD(&rtk_carveout_heap->pools);
	INIT_LIST_HEAD(&rtk_carveout_heap->prealloc_pools);
	INIT_LIST_HEAD(&rtk_carveout_heap->ion_flag_replace);

	list_for_each_entry(pool, pool_list, list) {
		struct pool_info *sub_pool = NULL;
		if (pool->type == RTK_CARVEOUT_GEN_POOL_TYPE) {
			sub_pool =
			    ion_gen_pool_create(GEN_POOL,
						pool->base,
						pool->size,
						pool->flags,
						pool->from_reserved_memory,
						dev);
		} else if (pool->type == RTK_CARVEOUT_CMA_POOL_TYPE) {
			sub_pool =
			    ion_cma_pool_create(pool->cma_pool, pool->flags);
		}
		if (sub_pool)
			list_add(&sub_pool->list, &rtk_carveout_heap->pools);
	}

	{
		struct ion_flag_replace *rtk_replace, *tmp_rtk_replace;
		list_for_each_entry_safe(rtk_replace, tmp_rtk_replace,
				&priv_data->rtk_flag_replace, list) {
			if (rtk_replace) {
				do {
					struct ion_flag_replace * ion_replace = kzalloc(sizeof(struct ion_flag_replace), GFP_KERNEL);
					if (!ion_replace)
						break;
					INIT_LIST_HEAD(&ion_replace->list);
					ion_replace->condition = ion_flag_mapping_from_rtk(rtk_replace->condition);
					ion_replace->replace = ion_flag_mapping_from_rtk(rtk_replace->replace);
					list_add_tail(&ion_replace->list, &rtk_carveout_heap->ion_flag_replace);
				} while(0);
			}
		}
	}

	rtk_carveout_heap->heap.buf_ops = rtk_dma_buf_ops;
	rtk_carveout_heap->heap.ops = &rtk_carveout_heap_ops;
	rtk_carveout_heap->heap.flags = 0;
	rtk_carveout_heap->priv_ops = &rtk_carveout_heap_priv_ops;
	rtk_carveout_heap->version = sizeof(*rtk_carveout_heap);
#ifdef CONFIG_DEBUG_FS
	debug_root = debugfs_create_dir("ion_retry", NULL);
	debugfs_create_file("retry_count", 0644, debug_root, pool, &debug_retry_count_fops);
	debugfs_create_file("retry_delay", 0644, debug_root, pool, &debug_retry_delay_fops);
#endif

	{
		struct ion_rtk_pre_alloc * pre_alloc, *tmp_pre_alloc;
		list_for_each_entry_safe(pre_alloc, tmp_pre_alloc,
				&priv_data->pre_alloc, list) {
			if (pre_alloc) {
				do {
					unsigned long rtk_flags = pre_alloc->rtk_flags;
					struct prealloc_pool * prealloc_pool = (struct prealloc_pool *) kzalloc(sizeof(struct prealloc_pool), GFP_KERNEL);
					struct pool_info * parent;
					struct pool_info * pool;
					struct page_info *page_info = NULL;
					struct ion_buffer *buffer = NULL;
					if (!prealloc_pool)
						break;
					INIT_LIST_HEAD(&prealloc_pool->list);
					buffer = &prealloc_pool->buffer;
					buffer->heap = &rtk_carveout_heap->heap;
					buffer->flags = ion_flag_mapping_from_rtk(rtk_flags);
					if (ion_rtk_carveout_heap_allocate(buffer->heap, buffer,
								pre_alloc->size,
								buffer->flags) != 0) {
						kfree(prealloc_pool);
						break;
					}
					buffer->size = pre_alloc->size;
					page_info = (struct page_info *)buffer->priv_virt;
					parent = ion_rtk_carveout_heap_find_pool_info(rtk_carveout_heap, page_info->base);
					if (!parent) {
						ion_rtk_carveout_heap_free(buffer);
						kfree(prealloc_pool);
						break;

					}
					rtk_flags |= parent->rtk_flags & ~RTK_FLAG_PROTECTED_DYNAMIC;
					if (!ion_flag_is_protected(buffer->flags)) {
						rtk_flags &= ~(RTK_FLAG_PROTECTED_MASK | RTK_FLAG_PROTECTED_EXT_MASK);
					}
					pool = ion_gen_pool_create(PREALLOC_POOL, page_info->base, page_info->size,
							rtk_flags, false /*from_reserved_memory*/, dev);
					if (!pool) {
						ion_rtk_carveout_heap_free(buffer);
						kfree(prealloc_pool);
						break;
					}
					list_add(&pool->list, &rtk_carveout_heap->pools);
					prealloc_pool->pool = pool;

					list_add(&prealloc_pool->list, &rtk_carveout_heap->prealloc_pools);
				} while(0);
			}
		}
	}
#ifdef CONFIG_VIRTUAL_PMU
	printk(KERN_ERR "\033[1;33m" "%s %d: name %s, type %d, id %d" "\033[m\n",
	__FUNCTION__, __LINE__, heap_data->name, heap_data->type, heap_data->id);
	if( (int)heap_data->type == 7 /* (int)RTK_PHOENIX_ION_HEAP_TYPE_MEDIA */) {
		vpmu_reg_idx_media_gen = rtk_vpmu_api_add_to_available(
		    (unsigned long)ion_update_media_heap_size_info_in_gen);
		printk(KERN_ERR "\033[1;33m" "%s %d: vpmu_reg_idx_media_gen 0x%08x" "\033[m\n",
		__FUNCTION__, __LINE__, vpmu_reg_idx_media_gen);

		vpmu_reg_idx_media_cma = rtk_vpmu_api_add_to_available(
		    (unsigned long)ion_update_media_heap_size_info_in_cma);
		printk(KERN_ERR "\033[1;33m" "%s %d: vpmu_reg_idx_media_cma 0x%08x" "\033[m\n",
		__FUNCTION__, __LINE__, vpmu_reg_idx_media_cma);
	}

	if( (int)heap_data->type == 8 /* (int)RTK_PHOENIX_ION_HEAP_TYPE_AUDIO */) {
		vpmu_reg_idx_audio_gen = rtk_vpmu_api_add_to_available(
		    (unsigned long)ion_update_audio_heap_size_info_in_gen);
		printk(KERN_ERR "\033[1;33m" "%s %d: vpmu_reg_idx_audio_gen 0x%08x" "\033[m\n",
		__FUNCTION__, __LINE__, vpmu_reg_idx_audio_gen);

		vpmu_reg_idx_audio_cma = rtk_vpmu_api_add_to_available(
		    (unsigned long)ion_update_audio_heap_size_info_in_cma);
		printk(KERN_ERR "\033[1;33m" "%s %d: vpmu_reg_idx_audio_cma 0x%08x" "\033[m\n",
		__FUNCTION__, __LINE__, vpmu_reg_idx_audio_cma);
	}
#endif
	return &rtk_carveout_heap->heap;
}

void ion_rtk_carveout_heap_destroy(struct ion_heap *heap)
{
	struct ion_rtk_carveout_heap *rtk_carveout_heap =
	    container_of(heap, struct ion_rtk_carveout_heap, heap);
	struct pool_info *pool_info, *tmp_pool_info;
	struct prealloc_pool *prealloc_pool, *tmp_prealloc_pool;
	struct ion_flag_replace * ion_replace, *tmp_ion_replace;

	mutex_lock(&rtk_carveout_heap->lock);
	list_for_each_entry_safe(ion_replace, tmp_ion_replace,
			&rtk_carveout_heap->ion_flag_replace, list) {
		if (ion_replace) {
			list_del(&ion_replace->list);
			kfree(ion_replace);
		}
	}
	list_for_each_entry_safe(prealloc_pool, tmp_prealloc_pool,
			&rtk_carveout_heap->prealloc_pools, list) {
		if (prealloc_pool) {
			pool_info = prealloc_pool->pool;
			list_del(&prealloc_pool->list);
			ion_rtk_carveout_heap_free(&prealloc_pool->buffer);
			list_del(&pool_info->list);
			pool_info->destroy(pool_info);
			kfree(prealloc_pool);
		}
	}
	list_for_each_entry_safe(pool_info, tmp_pool_info,
				 &rtk_carveout_heap->pools, list) {
		if (pool_info) {
			list_del(&pool_info->list);
			pool_info->destroy(pool_info);
		}
	}
	mutex_unlock(&rtk_carveout_heap->lock);
	kfree(rtk_carveout_heap->trace_message);
	kfree(rtk_carveout_heap);
}

static unsigned int ion_rtk_carveout_get_version(struct ion_heap *heap)
{
	struct ion_rtk_carveout_heap *rtk_carveout_heap =
	    container_of(heap, struct ion_rtk_carveout_heap, heap);

	if (rtk_carveout_heap->version != sizeof(struct ion_rtk_carveout_heap))
		goto err;

	return (unsigned int)rtk_carveout_heap->version & -1U;

err:
	return 0;
}

static size_t heap_pool_count(struct ion_rtk_carveout_heap *rtk_carveout_heap)
{
	size_t ret = 0;
	struct pool_info *pool_info = NULL;
	list_for_each_entry(pool_info, &rtk_carveout_heap->pools, list) {
		if (pool_info)
			ret++;
	}
	return ret;
}

static int ion_rtk_carveout_get_meminfo(struct ion_heap *heap,
					unsigned int flags,
					struct ion_rtk_carveout_meminfo *info)
{
	int ret = -1;
	struct ion_rtk_carveout_heap *rtk_carveout_heap =
	    container_of(heap, struct ion_rtk_carveout_heap, heap);
	mutex_lock(&rtk_carveout_heap->lock);
	do {
		struct pool_info *pool_info = NULL;
		unsigned long usedSize = 0, freeSize = 0;

		if (!info)
			break;

		list_for_each_entry(pool_info, &rtk_carveout_heap->pools, list) {
			size_t sUsed, sSpace;
			if (!pool_info)
				continue;
			sUsed = pool_info->getUsage(pool_info, flags);
			sSpace = pool_info->getSpace(pool_info, flags);
			usedSize += sUsed;
			freeSize += sSpace - sUsed;
		}
		info->usedSize = usedSize;
		info->freeSize = freeSize;
		ret = 0;
	}
	while (0);
	mutex_unlock(&rtk_carveout_heap->lock);
	return ret;
}

static int ion_rtk_carveout_trace_dump(struct ion_heap *heap,
				       struct seq_file *s)
{
	int ret = -ENOMEM;
	struct ion_rtk_carveout_heap *rtk_carveout_heap =
	    container_of(heap, struct ion_rtk_carveout_heap, heap);
	mutex_lock(&rtk_carveout_heap->lock);
	ret = trace_seq_buf_print_seq(s, &rtk_carveout_heap->trace_seq_buf);
	mutex_unlock(&rtk_carveout_heap->lock);
	return ret;

}

struct ion_rtk_carveout_ops *get_rtk_carveout_ops(struct ion_heap *heap)
{
	struct ion_rtk_carveout_heap *rtk_carveout_heap =
	    container_of(heap, struct ion_rtk_carveout_heap, heap);

	if (rtk_carveout_heap->version != sizeof(struct ion_rtk_carveout_heap))
		return NULL;

	return rtk_carveout_heap->priv_ops;
}

static void carveout_heap_allocate_fail_trace(struct ion_rtk_carveout_heap *rtk_carveout_heap,
					  unsigned long size,
					  unsigned long flags)
{
	struct task_struct *task;
	pid_t pid, tgid;
	char task_comm[TASK_COMM_LEN];
	struct seq_buf *s = &rtk_carveout_heap->trace_seq_buf;
	char prefix [100];
	print_time(local_clock(), &prefix[0]);

	get_task_struct(current->group_leader);
	{
		task_lock(current->group_leader);
		pid = task_pid_nr(current->group_leader);
		tgid = task_tgid_nr(current->group_leader);
		if (current->group_leader->flags & PF_KTHREAD)
			task = NULL;
		else
			task = current->group_leader;
		task_unlock(current->group_leader);

		if (task)
			get_task_comm(task_comm, task);
		else
			strncpy(task_comm, "KTHREAD", sizeof(task_comm));
	}
	put_task_struct(current->group_leader);

    seq_buf_clear(s);

	seq_buf_printf(s, "%s(%s PID:%d TID:%d) Alloc failed! size:%lu flags:0x%08lx ",
			&prefix[0], task_comm, pid, tgid, size, flags);

	seq_buf_printf(s, "(");
	seq_buf_printf_ion_flags(s, flags);
	seq_buf_printf(s, ")\n");

	ion_carveout_heap_dump(rtk_carveout_heap, s, &prefix[0]);
}

static void ion_rtk_carveout_heap_flags_preprocess(struct ion_rtk_carveout_heap *rtk_carveout_heap, unsigned long * pflags)
{
	struct ion_flag_replace *ion_replace;
	unsigned long flags = *pflags;
	if ((flags & RTK_ION_FLAG_POOL_CONDITION) == 0) {
		pr_err
			(" Warning: flags is 0x%lx!! The default value is set (ION_FLAG_ACPUACC | ION_FLAG_SCPUACC | ION_FLAG_HWIPACC | ION_USAGE_MMAP_NONCACHED) \n",
			 flags);
		flags =
			ION_FLAG_ACPUACC | ION_FLAG_SCPUACC |
			ION_FLAG_HWIPACC | ION_USAGE_MMAP_NONCACHED;
	}

	list_for_each_entry(ion_replace, &rtk_carveout_heap->ion_flag_replace, list) {
		if (ion_replace) {
			unsigned long condition = ion_replace->condition | ION_FLAG_PROTECTED_MASK | ION_FLAG_PROTECTED_EXT_MASK;
			if ((flags & condition) == ion_replace->condition) {
				flags &= ~condition;
				flags |= ion_replace->replace;
			}
		}
	}

	*pflags = flags;
}

static int ion_rtk_carveout_heap_allocate(struct ion_heap *heap,
					  struct ion_buffer *buffer,
					  unsigned long size,
					  unsigned long flags)
{
	int ret = -ENOMEM;
	unsigned int retry_count = 0;
	struct ion_rtk_carveout_heap *rtk_carveout_heap =
	    container_of(heap, struct ion_rtk_carveout_heap, heap);
	mutex_lock(&rtk_carveout_heap->lock);
	do {
		size_t pool_count = heap_pool_count(rtk_carveout_heap);
		struct pool_score_slot *pool_score_array = NULL;
		struct page_info *page_info = NULL;

		ion_rtk_carveout_heap_flags_preprocess(rtk_carveout_heap, &flags);

		if (pool_count <= 0) {
			break;
		}

		pool_score_array =
		    kzalloc(sizeof(struct pool_score_slot) * pool_count,
			    GFP_KERNEL);

		if (!pool_score_array) {
			break;
		}

		do {
			size_t i;
			struct pool_info *pool_info = NULL;
			struct pool_score_slot *pool_score_slot =
			    pool_score_array;

			list_for_each_entry(pool_info,
					    &rtk_carveout_heap->pools, list) {
				pool_score_slot->pool_info = pool_info;
				pool_score_slot->score =
				    pool_score(pool_info, PAGE_ALIGN(size),
					       flags);
				pool_score_slot++;
			}

			sort((void *)pool_score_array, pool_count,
			     sizeof(struct pool_score_slot), pool_score_cmp,
			     NULL);

			for (i = 0; i < pool_count; i++) {
				pool_score_slot = &pool_score_array[i];
				pool_info = pool_score_slot->pool_info;
				pr_debug
				    ("alloc[size=%-10ld flags=%08lx]  score[%2zu]=%-5d pool[base=%08lx size=%-10zu flags=%08lx space=%zu]\n",
				     size, flags, i, pool_score_slot->score,
				     pool_info->base, pool_info->size,
				     pool_info->rtk_flags,
				     pool_info->getSpace(pool_info, flags));
			}

			for (i = 0; i < pool_count; i++) {
				pool_score_slot = &pool_score_array[i];
				pool_info = pool_score_slot->pool_info;

				if (!pool_info || pool_score_slot->score <= 0)
					continue;

				page_info =
				    pool_info->alloc(pool_info, size,
						     PAGE_ALIGN(size), flags);

				if (page_info)
					break;
			}
		} while (0);

		kfree(pool_score_array);

		if (!page_info)
			break;

		buffer->flags |= page_info->flags;

		if (buffer->flags & ION_USAGE_MMAP_NONCACHED) {
			/* NONCACHED */
			buffer->flags |= ION_FLAG_CACHED;
			buffer->flags |= ION_FLAG_NONCACHED;
			buffer->flags &= ~ION_FLAG_CACHED_NEEDS_SYNC;
		} else if (buffer->flags & ION_USAGE_MMAP_WRITECOMBINE) {
			/* WRITECOMBINE */
			buffer->flags &= ~ION_FLAG_CACHED;
			buffer->flags &= ~ION_FLAG_NONCACHED;
			buffer->flags &= ~ION_FLAG_CACHED_NEEDS_SYNC;
		} else if ((buffer->flags & ION_USAGE_MMAP_CACHED)
			   || (!(buffer->flags & ION_FLAG_NONCACHED))) {
			/* CACHED */
			buffer->flags |= ION_FLAG_CACHED;
			buffer->flags &= ~ION_FLAG_NONCACHED;
			buffer->flags |= ION_FLAG_CACHED_NEEDS_SYNC;
		}

		if (!(buffer->flags & ION_FLAG_SCPUACC)
		    || (buffer->flags & ION_USAGE_PROTECTED)) {
			buffer->flags |= ION_FLAG_CACHED;
			buffer->flags |= ION_FLAG_NONCACHED;
			buffer->flags &= ~ION_FLAG_CACHED_NEEDS_SYNC;
		}

		buffer->sg_table = page_info->get_sg_table(page_info);
		buffer->priv_virt = page_info;
		ret = 0;
	}
	while (0);

	if (ret != 0) {
		carveout_heap_allocate_fail_trace(rtk_carveout_heap, size, flags);
	}

	mutex_unlock(&rtk_carveout_heap->lock);
#ifdef CONFIG_VIRTUAL_PMU
	ion_update_heap_size_info(heap);
#endif
	return ret;
}

static void ion_rtk_carveout_heap_free(struct ion_buffer *buffer)
{
	struct page_info *page_info = NULL;
	struct device *dev = idev;

	if (buffer)
		page_info = (struct page_info *)buffer->priv_virt;

	if (page_info) {
		if (ion_flag_canAccess(page_info->flags)) {
			ion_buffer_zero(buffer);
			if(!!(buffer->flags & ION_FLAG_CACHED))
				page_info->sync(dev, page_info, DMA_TO_DEVICE);
		}
		page_info->destroy(page_info);
#ifdef CONFIG_VIRTUAL_PMU
		if (buffer->heap) {
			ion_update_heap_size_info(buffer->heap);
		}
#endif
	}
}

int ion_carveout_heap_debug_show(struct ion_heap *heap,
				 struct seq_file *s, void *prefix)
{
	int ret = -1;
	struct ion_rtk_carveout_heap *rtk_heap;

	rtk_heap = container_of(heap, struct ion_rtk_carveout_heap, heap);

	mutex_lock(&rtk_heap->lock);
	do {
		const size_t seq_buf_size = PAGE_SIZE * 10;
		struct seq_buf seq_buf;
		char *seq_buf_message = kzalloc(seq_buf_size, GFP_KERNEL);

		if (!seq_buf_message)
			break;

		seq_buf_init(&seq_buf, seq_buf_message, seq_buf_size);

		ret = ion_carveout_heap_dump(rtk_heap, &seq_buf, prefix);

		trace_seq_buf_print_seq(s, &seq_buf);

		kfree(seq_buf_message);

	} while (0);
	mutex_unlock(&rtk_heap->lock);

	return ret;
}
EXPORT_SYMBOL(ion_carveout_heap_debug_show);

static int ion_carveout_heap_dump(struct ion_rtk_carveout_heap
				  *rtk_carveout_heap, struct seq_buf *s,
				  void *prefix)
{
	int ret = -1;

	do {
		struct ion_flag_replace *ion_replace;
		struct pool_info *pool_info = NULL;

		if (!prefix)
			prefix = "";

		list_for_each_entry(ion_replace, &rtk_carveout_heap->ion_flag_replace, list) {
			if (!ion_replace)
				continue;
			seq_buf_printf(s, "replace-table[%pK] 0x%08lx :",
					prefix, ion_replace->condition);
			seq_buf_printf_ion_flags(s, ion_replace->condition);
			seq_buf_printf(s, "\n");
			seq_buf_printf(s, "replace-table[%pK] 0x%08lx :",
					prefix, ion_replace->replace);
			seq_buf_printf_ion_flags(s, ion_replace->replace);
			seq_buf_printf(s, "\n");
		}

		list_for_each_entry(pool_info, &rtk_carveout_heap->pools, list) {
			if (!pool_info)
				continue;
			pool_info->debug_show(pool_info, s, prefix);
		}
		ret = 0;
	}

	while (0);

	return ret;
}
