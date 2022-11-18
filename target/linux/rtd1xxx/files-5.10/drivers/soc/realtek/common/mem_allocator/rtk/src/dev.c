/* rtk_ion_of.c
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/buffer_head.h>
#include <linux/cma.h>
#include <linux/dma-map-ops.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ion.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/version.h>

#include <asm/uaccess.h>
#include <soc/realtek/memory.h>
#include <soc/realtek/uapi/ion_rtk.h>
#include <soc/realtek/rtk_tee.h>
#include <uapi/linux/ion.h>

#include "../inc/ion_rtk_alloc.h"
#include "carveout_heap.h"
#include "cma.h"

#ifdef CONFIG_DEBUG_FS
#include "debugfs.h"
#endif

#include "ioctl.h"
#include "private.h"

struct device *idev;
unsigned int cma_area_count_t;
struct cma_init_priv {
	bool skip_dma_default_array;
	int cma_pool_flags;
	struct device_node *node;
	struct list_head *pools;
};

struct ion_platform_data {
	int nr;
	struct ion_platform_heap *heaps;
};
struct ion_heap_desc {
	unsigned int id;
	enum ion_heap_type type;
	const char *name;
	unsigned int permission_type;
};

enum ion_heap_ids {
	INVALID_HEAP_ID = -1,
	RTK_ION_HEAP_TYPE_MEDIA_ID = 7,
	RTK_ION_HEAP_TYPE_AUDIO_ID = 8,
	RTK_ION_HEAP_TYPE_SECURE_ID = 9,
};
struct gen_reserved {
	struct resource r;
	void __iomem *vaddr;
	unsigned int flags;
	struct list_head list;
};
static LIST_HEAD(gen_reserved_list);

#define COLUMN 3

static struct ion_heap_desc ion_heap_meta[] = {
	{
	 .type = RTK_ION_HEAP_TYPE_MEDIA,
	 .id = RTK_ION_HEAP_TYPE_MEDIA_ID,
	 .name = "Media",
	 },
	{
	 .type = RTK_ION_HEAP_TYPE_AUDIO,
	 .id = RTK_ION_HEAP_TYPE_AUDIO_ID,
	 .name = "Audio",
	 },
	{
	 .type = RTK_ION_HEAP_TYPE_SECURE,
	 .id = RTK_ION_HEAP_TYPE_SECURE_ID,
	 .name = "Secure",
	 },
};

static int rtk_ion_populate_heap(struct ion_platform_heap *heap)
{
	unsigned int i;
	int ret = -EINVAL;
	unsigned int len = ARRAY_SIZE(ion_heap_meta);

	for (i = 0; i < len; i++) {
		if (ion_heap_meta[i].id == heap->id) {
			heap->name = ion_heap_meta[i].name;
			heap->type = ion_heap_meta[i].type;
			ret = 0;
			break;
		}
	}

	if (ret)
		dev_err(idev, "unable to populate heap, error:%d", ret);

	return ret;
}

struct gen_reserved *create_gen_reserved(struct device_node *node)
{
	struct gen_reserved *ret = NULL;
	do {
		struct resource r;
		void __iomem *vaddr;
		int gen_flags = 0;

		struct device_node *dt_node =
		    of_parse_phandle(node, "memory-region", 0);
		if (!dt_node)
			break;

		if (of_property_read_u32(node, "rtk,gen-pool-Flags", &gen_flags)
		    != 0) {
			break;
		}

		if (gen_flags == 0) {
			pr_err("%s : Error! rtk,gen-pool-Flags is zero\n",
			       __func__);
			break;
		}

		if (of_address_to_resource(dt_node, 0, &r)) {
			pr_err
			    ("%s : No memory address assigned to the region\n",
			     __func__);
			break;
		}

		{
			phys_addr_t paddr = r.start;
			size_t size = resource_size(&r);
			vaddr = memremap(paddr, size, MEMREMAP_WB);
			if (!vaddr) {
				pr_err
				    ("%s : Could not map the region. (paddr: 0x%08llx, size: 0x%08lx)\n",
				     __func__, paddr, size);
				break;
			}

		}

		ret =
		    (struct gen_reserved *)kzalloc(sizeof(struct gen_reserved),
						   GFP_KERNEL);
		if (!ret) {
			pr_err("%s : kzalloc gen_reserved failed!\n", __func__);
			break;
		}

		memcpy(&ret->r, &r, sizeof(struct resource));
		ret->vaddr = vaddr;
		ret->flags = gen_flags;
		break;
	} while (0);
	return ret;
}

phys_addr_t gen_reserved_base(struct gen_reserved * gen_reserved)
{
	return gen_reserved->r.start;
}

size_t gen_reserved_size(struct gen_reserved * gen_reserved)
{
	return resource_size(&gen_reserved->r);
}

unsigned int gen_reserved_flags(struct gen_reserved *gen_reserved)
{
	return gen_reserved->flags;
}

struct cma *find_reserved_cma(struct device_node *node)
{
	struct cma *ret = NULL;

	do {
		if (of_reserved_mem_device_init_by_idx(idev, node, 0) == 0) {
			ret = idev->cma_area;
			of_reserved_mem_device_release(idev);
		}
	} while (0);

	return ret;
}

static unsigned int rtk_ion_cma_extFlags(struct device_node *node,
					 struct cma *cma)
{
	unsigned long pool_base = PFN_PHYS(cma->base_pfn);
	size_t pool_size = (cma->count << PAGE_SHIFT);
	unsigned int ret = 0;
	const u32 *prop;
	int size = 0;
	int i;
	prop = of_get_property(node, "rtk,cma-pool-extFlags-info", &size);
	if (prop && (size % (sizeof(u32) * 3)) == 0) {
		for (i = 0; i < (size / (sizeof(u32) * 3)); i += 1) {
			u32 extFlag = (u32) of_read_number(prop, 1 + (i * 3));
			u32 extFlag_base =
			    (u32) of_read_number(prop, 2 + (i * 3));
			u32 extFlag_size =
			    (u32) of_read_number(prop, 3 + (i * 3));
			if (((u32) pool_base & -1U) >= extFlag_base
			    && (((u32) (pool_base + pool_size) & -1U) <=
				(extFlag_base + extFlag_size)))
				ret |= extFlag;
		}
	}

	{
		struct device_node *sub_node;
		for_each_child_of_node(node, sub_node) {
			struct cma *tmp = find_reserved_cma(sub_node);
			if (tmp && tmp == cma) {
				int cma_flags = 0;
				if (of_property_read_u32
				    (sub_node, "rtk,cma-pool-extFlags",
				     &cma_flags) == 0) {
					ret |= cma_flags;
				}
			}
		}
	}
	return ret;
}

static int rtk_get_cma_area_count(struct cma *cma, void *data)
{
	cma_area_count_t++;

	return 0;
}

static int rtk_cma_init(struct cma *cma, void *data)
{
	struct ion_rtk_priv_pool *pool;
	struct cma_init_priv *cma_init_data = (struct cma_init_priv *) data;
	struct list_head *pools = cma_init_data->pools;
	int cma_pool_flags = cma_init_data->cma_pool_flags;
	bool skip_dma_default_array = cma_init_data->skip_dma_default_array;
	struct device_node *node = cma_init_data->node;

	if (skip_dma_default_array && cma == dma_contiguous_default_area) {
		dev_err(idev, "do not add dma_contiguous_default_area to the pool list.\n");
		return 0;
	}

	if ((cma->count << PAGE_SHIFT) == 0)
		return 0;

	pool = kzalloc(sizeof(struct ion_rtk_priv_pool), GFP_KERNEL);
	pool->type = RTK_CARVEOUT_CMA_POOL_TYPE;
	pool->cma_pool = cma;
	pool->base = PFN_PHYS(cma->base_pfn);
	pool->size = (pool->cma_pool->count << PAGE_SHIFT);
	pool->flags = cma_pool_flags;
	pool->flags |= rtk_ion_cma_extFlags(node, cma);
	list_add(&pool->list, pools);

	return 0;
}

static struct ion_platform_data *rtk_ion_parse_dt(const struct device_node
						  *dt_node)
{
	struct ion_platform_data *pdata;
	struct ion_platform_heap *heap_data;
	struct device_node *node;

	uint32_t val = 0;
	int ret = 0;
	uint32_t num_heaps = 0;
	const u32 *prop;
	int err = 0;
	int size = 0;
	int i = 0;
	struct list_head *pools;
	bool use_cma_pools = 0;
	int cma_pool_flags = 0;
	bool skip_dma_default_array = 0;
	struct device_node *sub_node;
	int counter = 0;
	struct cma_init_priv cma_init_data;

	cma_area_count_t = 0;

	cma_for_each_area(rtk_get_cma_area_count, NULL);

	for_each_child_of_node(dt_node, node)
	    num_heaps++;

	if (!num_heaps)
		return ERR_PTR(-EINVAL);

	pdata = kzalloc(sizeof(struct ion_platform_data), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->nr = num_heaps;

	pdata->heaps = kcalloc(num_heaps, sizeof(struct ion_platform_heap), GFP_KERNEL);
	if (!pdata->heaps) {
		ret = -ENOMEM;
		goto free_pdata;
	}

	heap_data = pdata->heaps;
	for_each_child_of_node(dt_node, node) {
		struct ion_rtk_heap_create_priv * priv_data = NULL;
		struct list_head * rtk_flag_replace;
		struct list_head * pre_alloc;

		ret = of_property_read_u32(node, "reg", &val);
		if (ret) {
			dev_err(idev, "unable to find reg key");
			goto free_heaps;
		}

		heap_data->id = val;
		ret = rtk_ion_populate_heap(heap_data);
		if (ret) {
			pdata->nr--;
			continue;
			//goto free_heaps;
		}

		heap_data->base = 0;
		heap_data->size = 0;
		heap_data->priv = NULL;

		priv_data = (struct ion_rtk_heap_create_priv *) kzalloc(sizeof(struct ion_rtk_heap_create_priv), GFP_KERNEL);
		pools = &priv_data->pools;
		rtk_flag_replace = &priv_data->rtk_flag_replace;
		pre_alloc = &priv_data->pre_alloc;
		use_cma_pools =
		    of_find_property(node, "rtk,use-cma-pools",
				     NULL) ? true : false;
		cma_pool_flags = RTK_FLAG_SCPUACC | RTK_FLAG_ACPUACC | RTK_FLAG_HWIPACC;	/* defaule */
		skip_dma_default_array =
		    of_find_property(node,
				     "rtk,cma-pools-skip-dma-default-array",
				     NULL) ? true : false;

		INIT_LIST_HEAD(pools);
		INIT_LIST_HEAD(rtk_flag_replace);
		INIT_LIST_HEAD(pre_alloc);

		prop = of_get_property(node, "rtk,memory-reserve", &size);
		err = size % (sizeof(u32) * COLUMN);
		if ((prop) && (!err)) {
			counter = size / (sizeof(u32) * COLUMN);
			for (i = 0; i < counter; i += 1) {
				struct ion_rtk_priv_pool *pool =
				    kzalloc(sizeof(struct ion_rtk_priv_pool),
					    GFP_KERNEL);
				pool->type = RTK_CARVEOUT_GEN_POOL_TYPE;
				pool->base = (phys_addr_t) ((u32)
							    of_read_number(prop,
									   1 +
									   (i *
									    COLUMN)));
				pool->size = (size_t) ((u32)
						       of_read_number(prop,
								      2 +
								      (i *
								       COLUMN)));
				pool->flags = (unsigned long)((u32)
							      of_read_number
							      (prop,
							       3 +
							       (i * COLUMN)));

				list_add(&pool->list, pools);
			}
			heap_data->priv = (void *)priv_data;
		}

		struct device_node *sub_node;
		for_each_child_of_node(node, sub_node) {
			struct gen_reserved *tmp =
				create_gen_reserved(sub_node);
			if (tmp) {
				struct ion_rtk_priv_pool *pool =
					kzalloc(sizeof
						(struct ion_rtk_priv_pool),
						GFP_KERNEL);
				pool->type = RTK_CARVEOUT_GEN_POOL_TYPE;
				pool->base = gen_reserved_base(tmp);
				pool->size = gen_reserved_size(tmp);
				pool->flags = gen_reserved_flags(tmp);
				pool->from_reserved_memory = true;
				list_add(&pool->list, pools);
				list_add(&tmp->list,
						&gen_reserved_list);
			}
		}
		heap_data->priv = (void *)priv_data;

		if (use_cma_pools
		    && of_find_property(node, "rtk,cma-pool-flags", NULL)) {
			unsigned int flags;
			if (of_property_read_u32
			    (node, "rtk,cma-pool-flags", &flags) == 0
			    && flags != 0) {
				cma_pool_flags = flags;
			}
		}

		/*** legacy ***/
		if (heap_data->id == RTK_ION_HEAP_TYPE_MEDIA_ID &&
		    !of_find_property(node, "rtk,use-cma-pools", NULL)) {
			use_cma_pools = true;
			if (cma_area_count_t == 1) {
				cma_pool_flags =
				    RTK_FLAG_SCPUACC | RTK_FLAG_NONCACHED |
				    RTK_FLAG_HWIPACC;
			} else {
				skip_dma_default_array = true;
			}
		}

		if (use_cma_pools) {
			cma_init_data.cma_pool_flags = cma_pool_flags;
			cma_init_data.skip_dma_default_array = skip_dma_default_array;
			cma_init_data.pools = pools;
			cma_init_data.node = node;
			cma_for_each_area(rtk_cma_init, &cma_init_data);
			heap_data->priv = (void *)priv_data;
		}

		if (heap_data->priv == NULL) {
			kfree(priv_data);
		} else {
			prop = of_get_property(node, "rtk,flag-replace-table", &size);
			if (prop && (size % (sizeof(u32) * 2)) == 0) {
				for (i = 0; i < size/sizeof(u32) ; i += 2) {
					struct ion_flag_replace * replace = (struct ion_flag_replace *) kzalloc(sizeof(struct ion_flag_replace), GFP_KERNEL);
					if (!replace)
						continue;
					INIT_LIST_HEAD(&replace->list);
					replace->condition      = (unsigned long) ((u32)of_read_number(prop, i + 1));
					replace->replace        = (unsigned long) ((u32)of_read_number(prop, i + 2));
					list_add_tail(&replace->list, &priv_data->rtk_flag_replace);
				}
			}

			prop = of_get_property(node, "rtk,pre-alloc", &size);
			if (prop && (size % (sizeof(u32) * 2)) == 0) {
				for (i = 0; i < size/sizeof(u32) ; i += 2) {
					struct ion_rtk_pre_alloc * pre_alloc = (struct ion_rtk_pre_alloc *) kzalloc(sizeof(struct ion_rtk_pre_alloc), GFP_KERNEL);
					if (!pre_alloc)
						continue;
					INIT_LIST_HEAD(&pre_alloc->list);
					pre_alloc->rtk_flags    = (unsigned long) ((u32)of_read_number(prop, i + 1));
					pre_alloc->size         = (unsigned long) ((u32)of_read_number(prop, i + 2));
					list_add(&pre_alloc->list, &priv_data->pre_alloc);
				}
			}
		}

		heap_data++;
	}
	return pdata;

free_heaps:
	kfree(pdata->heaps);
free_pdata:
	kfree(pdata);
	return ERR_PTR(ret);
}

struct ion_heap *ion_heap_create(struct device *dev,
				 struct ion_platform_heap *heap_data)
{
	struct ion_heap *heap = NULL;

	switch ((int)heap_data->type) {
	case RTK_ION_HEAP_TYPE_MEDIA:
	case RTK_ION_HEAP_TYPE_AUDIO:
	case RTK_ION_HEAP_TYPE_SECURE:
		heap = ion_rtk_carveout_heap_create(dev, heap_data);
		if (!IS_ERR_OR_NULL(heap))
			heap->type = heap_data->type;
		break;
	default:
		dev_warn(idev, "invalid heap type %d\n", heap_data->type);
		return ERR_PTR(-EINVAL);
	}

	if (IS_ERR_OR_NULL(heap)) {
		dev_err(idev,
			"error creating heap %s type %d base %llx size %zu\n",
			heap_data->name, heap_data->type, heap_data->base,
			heap_data->size);
		return ERR_PTR(-EINVAL);
	}

	heap->name = heap_data->name;

#ifdef CONFIG_DEBUG_FS
	if (IS_ENABLED(CONFIG_DEBUG_FS))
		debugfs_add_heap(heap);
#endif

	return heap;
}

static const struct file_operations ion_rtk_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ion_rtk_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ion_rtk_ioctl,
#endif
};

int ion_rtk_device_create(void)
{
	struct miscdevice *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->minor = MISC_DYNAMIC_MINOR;
	dev->name = "ion_rtk";
	dev->fops = &ion_rtk_fops;
	dev->parent = NULL;

	ret = misc_register(dev);
	if (ret) {
		pr_err("ion_rtk: failed to register misc device.\n");
		kfree(dev);
		return ret;
	}

	return 0;
}

unsigned int rtk_ion_heap_id_mask_tr(unsigned int heap_type_mask)
{
	int i;
	size_t heap_cnt = 0;
	struct ion_heap_data buffer[32];
	unsigned int heap_id_mask = 0;

	do {
		heap_cnt = ion_query_heaps_kernel(buffer, 32);

		if (buffer == NULL || heap_cnt == 0)
			break;

		for (i = 0; i < heap_cnt; i++) {
			if (((1U << buffer[i].type) & heap_type_mask) == 0)
				continue;
			heap_id_mask |= 1U << buffer[i].heap_id;
		}

		break;
	} while (0);

	return heap_id_mask;
}


struct dma_buf *ext_rtk_ion_alloc(size_t len, unsigned int heap_type_mask,
		      unsigned int flags)
{
	int fd;
	struct dma_buf *dmabuf;

	unsigned int heap_id_mask = rtk_ion_heap_id_mask_tr(heap_type_mask);

	dmabuf = ion_alloc(len, heap_id_mask, flags);

	return dmabuf;
}
EXPORT_SYMBOL(ext_rtk_ion_alloc);

int ext_rtk_ion_close_fd(struct files_struct *files, unsigned fd)
{
	return __close_fd(files, fd);
}
EXPORT_SYMBOL(ext_rtk_ion_close_fd);

static int rtk_ion_probe(struct platform_device *pdev)
{
	int err = -1;
	int i = 0;
	struct ion_heap **heaps;
	struct ion_platform_data *pdata;
	struct ion_platform_heap *heap_data;
	struct tee_mem_api_module *tee_module;
	unsigned int pdata_needs_freed;

	tee_module = tee_mem_protected_get_gmodule();
	if (tee_module != NULL)
		ion_rtk_protected_notifier_register(&tee_module->protected_notifier);
	else
		return -EPROBE_DEFER;

	idev = &pdev->dev;
	if (pdev->dev.of_node) {
		pdata = rtk_ion_parse_dt(pdev->dev.of_node);
		if (IS_ERR(pdata)) {
			err = PTR_ERR(pdata);
			goto out;
		}
		pdata_needs_freed = 1;
	} else {
		pdata = pdev->dev.platform_data;
		pdata_needs_freed = 0;
	}

	dev_info(idev, "number of_heaps: %d\n", pdata->nr);

	heaps = devm_kzalloc(&pdev->dev, sizeof(struct ion_heap *) * pdata->nr, GFP_KERNEL);
	if (!heaps) {
		err = -ENOMEM;
		goto out;
	}

	/* create the heaps as specified in the board file */
	heap_data = pdata->heaps;

	for (i = 0; i < pdata->nr; i++) {

		heaps[i] = ion_heap_create(idev, heap_data);
		if (IS_ERR_OR_NULL(heaps[i])) {
			heaps[i] = 0;
			heap_data++;
			continue;
		}

		ion_device_add_heap(heaps[i]);

		if (heap_data->priv) {
			struct ion_rtk_heap_create_priv * priv_data = (struct ion_rtk_heap_create_priv *) heap_data->priv;
			struct ion_rtk_priv_pool *pool, *tmp_pool;
			struct ion_flag_replace * replace, *tmp_replace;
			struct ion_rtk_pre_alloc *pre_alloc, *tmp_pre_alloc;
			list_for_each_entry_safe(pool, tmp_pool, &priv_data->pools, list) {
				dev_err(idev,
					"adding heap %s of type %d with %llx@%lx (type:%s)",
					heap_data->name, heap_data->type,
					pool->base, pool->size,
					(pool->type ==
					 RTK_CARVEOUT_GEN_POOL_TYPE) ?
					"GEN_POOL" : (pool->type ==
						      RTK_CARVEOUT_CMA_POOL_TYPE)
					? "CMA_POOL" : "Unknown");

				list_del(&pool->list);
				kfree(pool);
			}
			list_for_each_entry_safe(replace, tmp_replace, &priv_data->rtk_flag_replace, list) {
				list_del(&replace->list);
				kfree(replace);
			}
			list_for_each_entry_safe(pre_alloc, tmp_pre_alloc,  &priv_data->pre_alloc, list) {
				list_del(&pre_alloc->list);
				kfree(pre_alloc);
			}
			kfree(priv_data);
		}
		heap_data++;
	}

	ion_rtk_device_create();

	if (pdata_needs_freed) {
		kfree(pdata->heaps);
		kfree(pdata);
	}

	return 0;

out:
	return err;
}

static int rtk_ion_remove(struct platform_device *pdev)
{
	/* [TODO]: destroy heap
	   for (int i = 0; i < num_heaps; i++)
	   ion_heap_destroy(heaps[i]);

	   kfree(heaps);
	 */
	return 0;
}
static struct of_device_id rtk_ion_ids[] = {
	{.compatible = "realtek,rtk-ion"},
	{ /* Sentinel */ },
};
static struct platform_driver rtk_ion_driver = {
	.probe = rtk_ion_probe,
	.remove = rtk_ion_remove,
	.driver = {
		   .name = "realtek-ion",
		   .of_match_table = rtk_ion_ids,
		   },
};

static int __init rtk_ion_init(void)
{
	return platform_driver_register(&rtk_ion_driver);
}

static void __exit rtk_ion_exit(void)
{
	platform_driver_unregister(&rtk_ion_driver);
}

fs_initcall(rtk_ion_init);
module_exit(rtk_ion_exit);

MODULE_LICENSE("GPL v2");
