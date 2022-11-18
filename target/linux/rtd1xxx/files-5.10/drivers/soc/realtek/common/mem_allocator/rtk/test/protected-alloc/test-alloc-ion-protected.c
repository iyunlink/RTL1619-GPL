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
#include <linux/delay.h>	/* usleep_range */
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/ktime.h>
#include <linux/random.h>
#include <linux/dma-buf.h>
#include <linux/fdtable.h>
#include <soc/realtek/uapi/ion_rtk.h>
#include <ion_rtk_alloc.h>

extern struct ion_device *rtk_phoenix_ion_device;

struct protected_mem {
	struct dma_buf *dmabuf;
	size_t size;
	struct list_head list;	// list of pool_handlers
};

struct alloc_protected_module {
	struct mutex protected_lock;
	struct list_head protected_list;
	size_t alloc_size;
	struct task_struct *alloc_thread;
	struct task_struct *free_thread;
};

struct alloc_protected_module *gModule = NULL;

#define TEST_ALLOC_THRESHOLD_MB (128)
#define TEST_ALLOC_MAX_MB       (32)

static int alloc_work(void *data)
{
	struct alloc_protected_module *module =
	    (struct alloc_protected_module *)data;
	while (!kthread_should_stop()) {
		uint32_t ion_heap_mask = RTK_ION_HEAP_MEDIA_MASK;
		uint32_t ion_heap_flag = ION_FLAG_PROTECTED_IO;
		size_t alloc_size =
		    ((get_random_int() +
		      1) % TEST_ALLOC_MAX_MB) * (1024 * 1024);
		mutex_lock(&module->protected_lock);
		do {
			struct protected_mem *mem;
			int handle;

			if (module->alloc_size >
			    (TEST_ALLOC_THRESHOLD_MB * 1024 * 1024))
				break;

			mem = kzalloc(sizeof(struct protected_mem), GFP_KERNEL);
			if (!mem)
				break;
			INIT_LIST_HEAD(&mem->list);
			mem->dmabuf = ext_rtk_ion_alloc(alloc_size,
						        ion_heap_mask,
						        ion_heap_flag);

			if (IS_ERR(mem->dmabuf)) {
				break;
			}

			mem->size = alloc_size;
			module->alloc_size += alloc_size;
			list_add_tail(&mem->list, &module->protected_list);
		} while (0);
		mutex_unlock(&module->protected_lock);
		msleep(get_random_int() % 16);
	}
	return 0;
}

static int free_work(void *data)
{
	struct alloc_protected_module *module =
	    (struct alloc_protected_module *)data;
	while (!kthread_should_stop()) {
		mutex_lock(&module->protected_lock);
		do {
			struct protected_mem *mem;
			if (list_empty(&module->protected_list))
				break;
#if 0
			mem =
			    list_first_entry(&module->protected_list,
					     struct protected_mem, list);
#else
			{
				int slot, total = 0;
				list_for_each_entry(mem,
						    &module->protected_list,
						    list)
				    total++;
				slot = get_random_int() % total;
				list_for_each_entry(mem,
						    &module->protected_list,
						    list) {
					if (slot == 0)
						break;
					slot--;
				}
			}
#endif
			list_del(&mem->list);
			dma_buf_put(mem->dmabuf);
			module->alloc_size -= mem->size;
			kfree(mem);
			break;
		} while (0);
		mutex_unlock(&module->protected_lock);
		msleep(get_random_int() % 16);
	}
	return 0;
}

static int alloc_protected_module_init(void)
{
	int ret = -1;

	pr_err("[DEBUG] ******** %s\n", __func__);

	do {
		struct alloc_protected_module *module = NULL;
		gModule =
		    kzalloc(sizeof(struct alloc_protected_module), GFP_KERNEL);
		if (!gModule)
			break;

		module = gModule;

		mutex_init(&module->protected_lock);
		INIT_LIST_HEAD(&module->protected_list);
		module->alloc_thread =
		    kthread_create(alloc_work, module,
				   "test-ion-protected-alloc");
		module->free_thread =
		    kthread_create(free_work, module,
				   "test-ion-protected-free");
		wake_up_process(module->alloc_thread);
		wake_up_process(module->free_thread);
		ret = 0;
	} while (0);
	return ret;
}

static void alloc_protected_module_release_all(struct alloc_protected_module
					       *module)
{
	struct protected_mem *protected_mem, *tmp_protected_mem;
	mutex_lock(&module->protected_lock);
	list_for_each_entry_safe(protected_mem, tmp_protected_mem,
				 &module->protected_list, list) {
		list_del(&protected_mem->list);
		dma_buf_put(protected_mem->dmabuf);
		module->alloc_size -= protected_mem->size;
		kfree(protected_mem);
	}
	mutex_unlock(&module->protected_lock);
}

static void alloc_protected_module_exit(void)
{
	struct alloc_protected_module *module = gModule;
	kthread_stop(module->alloc_thread);
	kthread_stop(module->free_thread);
	alloc_protected_module_release_all(module);
	kfree(module);
}

module_init(alloc_protected_module_init);
module_exit(alloc_protected_module_exit);
MODULE_AUTHOR("Otto Chen <ottochen@realtek.com>");
MODULE_DESCRIPTION("pressure test of protected memory");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
