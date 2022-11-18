// SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause)
/*
 * rpc memory remapping
 *
 * Copyright (c) 2017-2020 Realtek Semiconductor Corp.
 */
#include <linux/atomic.h>
#include <linux/dmi.h>
#include <linux/efi.h>
#include <linux/export.h>
#include <linux/memblock.h>
#include <linux/mm_types.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/preempt.h>
#include <linux/rbtree.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mmu_context.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/mmu.h>
#include <asm/pgtable.h>

#include <soc/realtek/memory.h>
#include <soc/realtek/rtk_ipc_shm.h>

#define VT100_NONE	"\033[m\n"
#define VT100_LIGHT_RED	"\033[1;31m"

void __iomem *rpc_common_base;
EXPORT_SYMBOL(rpc_common_base);

void __iomem *rpc_ringbuf_base;
EXPORT_SYMBOL(rpc_ringbuf_base);

static __init int rtk_rpc_memory_init(void)
{
	int i = 0;
	int npages = 0;
	int ret = -1;
	struct page **pages;
	struct page **tmp;
	struct device_node *node;
	struct resource res;

	node = of_find_compatible_node(NULL, NULL, "comm");
	if (!node) {
		pr_err("%s: Unable to find comm node", __func__);
		goto err;
	}

	ret = of_address_to_resource(node, 0, &res);
	if (ret) {
		pr_err("%s: Unable to get resource", __func__);
		goto err;
	}

	npages = resource_size(&res) >> PAGE_SHIFT;
	pages = vmalloc(sizeof(struct page *) * npages);
	if (!pages) {
		pr_info(VT100_LIGHT_RED "%s, fail to allocate memory"
			VT100_NONE, __func__);
		ret = -1;
		goto err;
	}

	for (i = 0, tmp = pages; i < npages; i++)
		*(tmp++) = phys_to_page(res.start + (PAGE_SIZE * i));

	rpc_common_base = vmap(pages,
			       npages,
			       VM_MAP,
			       pgprot_noncached(PAGE_KERNEL));
	vfree(pages);

	node = of_find_compatible_node(NULL, NULL, "ringbuf");
	if (!node) {
		pr_err("%s: Unable to find ringbuf node", __func__);
		goto err;
	}

	ret = of_address_to_resource(node, 0, &res);
	if (ret) {
		pr_err("%s: Unable to get resource", __func__);
		goto err;
	}

	npages = resource_size(&res) >> PAGE_SHIFT;
	pages = vmalloc(sizeof(struct page *) * npages);
	if (!pages) {
		pr_info(VT100_LIGHT_RED "%s, fail to allocate memory"
			VT100_NONE, __func__);
		ret = -1;
		goto err;
	}

	for (i = 0, tmp = pages; i < npages; i++)
		*(tmp++) = phys_to_page(res.start + (PAGE_SIZE * i));

	rpc_ringbuf_base = vmap(pages,
			       npages,
			       VM_MAP,
			       pgprot_noncached(PAGE_KERNEL));
	vfree(pages);
err:
	return ret;
}
arch_initcall(rtk_rpc_memory_init);

MODULE_LICENSE("GPL v2");
