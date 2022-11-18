// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek framebuffer driver
 *
 * Copyright (c) 2019-2020 Realtek Semiconductor Corp.
 */
#include "bootlogo.h"

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/memblock.h>

static struct platform_device *bootlogo_pdev = NULL;

struct bootlogo_drvdata {
	struct resource r;
	void __iomem *vaddr;
	enum {
		NORMAL,
		TAKE_UP,
	} st;
};

unsigned long bootlogo_free_reserved_area(void *start, void *end, int poison, const char *s)
{
	void *pos;
	unsigned long pages = 0;

	start = (void *)PAGE_ALIGN((unsigned long)start);
	end = (void *)((unsigned long)end & PAGE_MASK);
	for (pos = start; pos < end; pos += PAGE_SIZE, pages++) {
		struct page *page = virt_to_page(pos);
		void *direct_map_addr;

		/*
		 * 'direct_map_addr' might be different from 'pos'
		 * because some architectures' virt_to_page()
		 * work with aliases.  Getting the direct map
		 * address ensures that we get a _writeable_
		 * alias for the memset().
		 */
		direct_map_addr = page_address(page);
		if ((unsigned int)poison <= 0xFF)
			memset(direct_map_addr, poison, PAGE_SIZE);

		free_reserved_page(page);
	}

	if (pages && s)
		pr_info("Freeing %s memory: %ldK\n",
			s, pages << (PAGE_SHIFT - 10));

	return pages;
}

#ifdef CONFIG_HIGHMEM		/* copy form arch/arm/mm/init.c */
static inline void free_area_high(unsigned long pfn, unsigned long end)
{
	for (; pfn < end; pfn++)
		free_highmem_page(pfn_to_page(pfn));
}
#else
static inline void free_area_high(unsigned long pfn, unsigned long end)
{
}
#endif

void bootlogo_free_memory(struct device *dev,
	phys_addr_t paddr,
	void __iomem * vaddr,
	size_t size)
{
	struct page *page;

	if( vaddr ) {
		dev_info(dev, "\033[1;32m" "memunmap 0x%08x" "\033[m\n", (unsigned long)vaddr);
		memunmap(vaddr);
	}

	memblock_free(paddr, size); // remove rsv. info from memblock list

#if !defined(CONFIG_ARM64) || !defined(CONFIG_64BIT)
	page = pfn_to_page(paddr >> PAGE_SHIFT);
	if (PageHighMem(page)) {
		dev_info(dev, "\033[1;32m" "free high memory addr 0x%08x, size %d"
			"\033[m\n",
			paddr, size);
		//memblock_free(paddr, size);
		free_area_high((paddr >> PAGE_SHIFT),
			(paddr+size) >> PAGE_SHIFT);
	}
	else { /* low memory in linear mapping */
		dev_info(dev, "\033[1;32m" "free low memory addr 0x%08x(va 0x%08x)"
			" , size %d" "\033[m\n",
			paddr, __va(paddr), size);
		bootlogo_free_reserved_area(__va(paddr),
			__va(paddr+size), 0,
			"free logo area");
	}
#else /* low memory in linear mapping */
	dev_info(dev, "\033[1;32m" "free memory addr 0x%llx(0x%llx)"
		" , size %d" "\033[m\n",
		paddr, __va(paddr), size);
	bootlogo_free_reserved_area(__va(paddr),
	   __va(paddr+size), 0,
	   "free logo area");
#endif
}

void bootlogo_release(void)
{
	do {
		struct platform_device *pdev;
		struct bootlogo_drvdata *pdata;

		pdev = bootlogo_pdev;
		if (!pdev)
			break;

		pdata = (struct bootlogo_drvdata *)platform_get_drvdata(pdev);
		if (!pdata)
			break;

		switch (pdata->st) {
		case TAKE_UP:
			{
				phys_addr_t paddr = pdata->r.start;
				size_t size = resource_size(&pdata->r);
				unsigned long vaddr_limite =
				    ((unsigned long)pdata->vaddr) + size;
				dev_info(&pdev->dev,
					 "(release) base=0x%08x size=0x%zx(%zu) v=0x%08lx high_memory=0x%08lx\n",
					 paddr, size, size, (long)pdata->vaddr,
					 (long)high_memory);
#if 1
					bootlogo_free_memory(&pdev->dev, paddr, pdata->vaddr, size);
#else
#if !defined(CONFIG_ARM64) || !defined(CONFIG_64BIT)
				if (vaddr_limite >= (unsigned long)high_memory) {
					memblock_free(paddr, size);
					free_area_high((paddr >> PAGE_SHIFT),
						       (paddr +
							size) >> PAGE_SHIFT);
				} else {
					bootlogo_free_reserved_area(pdata->vaddr,
							   pdata->vaddr + size,
							   0, "free logo area");
				}
#else
				bootlogo_free_reserved_area(pdata->vaddr,
						   pdata->vaddr + size, 0,
						   "free logo area");
#endif
#endif
				pdata->st = NORMAL;
			}
			break;
		case NORMAL:
		default:
			break;
		}
	} while (0);
}
EXPORT_SYMBOL(bootlogo_release);

static int bootlogo_probe(struct platform_device *pdev)
{
	int ret = -1;

	do {
		struct device *dev = &pdev->dev;
		struct device_node *dt_node =
		    of_parse_phandle(dev->of_node, "memory-region", 0);
		struct bootlogo_drvdata *pdata = (struct bootlogo_drvdata *)
		    kzalloc(sizeof(struct bootlogo_drvdata), GFP_KERNEL);
		phys_addr_t paddr;
		size_t size;

		if (!pdata) {
			dev_err(dev, "Alloc drvdata failed.\n");
			break;
		}

		if (of_address_to_resource(dt_node, 0, &pdata->r)) {
			dev_err(dev,
				"No memory address assigned to the region\n");
			kfree(pdata);
			break;
		}

		paddr = pdata->r.start;
		size = resource_size(&pdata->r);

		pdata->vaddr = memremap(paddr, size, MEMREMAP_WB);

		if (!pdata->vaddr) {
			dev_err(dev,
				"Could not map the region. (paddr: 0x%08x, size: 0x%08x)\n",
				paddr, size);
			kfree(pdata);
			break;
		}

		pdata->st = TAKE_UP;
		platform_set_drvdata(pdev, (void *)pdata);

		bootlogo_pdev = pdev;
		dev_info(dev, "base=0x%08x size=0x%lx(%lu) v=0x%08lx\n",
			 paddr, (long)size, (long)size, (long)pdata->vaddr);

		ret = 0;
		break;
	} while (0);
	return ret;
}

static int bootlogo_remove(struct platform_device *pdev)
{
	struct bootlogo_drvdata *pdata =
	    (struct bootlogo_drvdata *)platform_get_drvdata(pdev);
	if (pdata) {
		bootlogo_release();
		kfree(pdata);
	}
	bootlogo_pdev = NULL;
	return 0;
}

static struct of_device_id bootlogo_ids[] = {
	{.compatible = "realtek,bootlogo"},
	{ /* Sentinel */ },
};

static struct platform_driver bootlogo_driver = {
	.probe = bootlogo_probe,
	.remove = bootlogo_remove,
	.driver = {
		   .name = "bootlogo",
		   .of_match_table = bootlogo_ids,
		   },
};

static int __init bootlogo_init(void)
{
	return platform_driver_register(&bootlogo_driver);
}

static void __exit bootlogo_exit(void)
{
	platform_driver_unregister(&bootlogo_driver);
}

fs_initcall(bootlogo_init);
module_exit(bootlogo_exit);

MODULE_LICENSE("GPL");
