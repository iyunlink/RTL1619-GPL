// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2020 Realtek Semiconductor Corporation
 */

#include <linux/kref.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/fdtable.h>
#include <linux/sched/task.h>
#include <linux/ion.h>
#include <soc/realtek/uapi/ion_rtk.h>

#include "buflock.h"

#include <ion_rtk_alloc.h>
#define ion_alloc ext_rtk_ion_alloc

#define OFFSET_FROM_ID(id) (id * 4)

#define MODULE_TAG "buflock :"

struct buflock_module; /* main module */
struct buflock_client; /* fd */
struct buflock_handle; /* slot */

struct buflock_handle {
    struct buflock_module * module;
    struct kref             ref;
    struct list_head        list; /* module->handles */
    struct mutex            mutex;

    size_t                  id;
    unsigned int            fwid;
    buflock_status_t *      pStatus;

    struct {
        ktime_t             create;
        ktime_t             lastSet;
        ktime_t             destroy;
    } sTime;

    void            (*destroy)      (struct kref *kref);
    unsigned int    (*getFWID)      (struct buflock_handle * handle);
    void            (*get)          (struct buflock_handle * handle);
    void            (*put)          (struct buflock_handle * handle);
    unsigned int    (*refCount)     (struct buflock_handle * handle);
    int             (*setStatus)    (struct buflock_handle * handle, buflock_status_t status);
    buflock_status_t(*getStatus)    (struct buflock_handle * handle);
};

struct buflock_slot {
    struct buflock_handle * handle;
    struct list_head        list; /* client->list */
    struct mutex            mutex;
};

struct buflock_client {
    struct buflock_module * module;
    struct mutex            mutex;
    struct list_head        slots; /* buflock_slot */
    struct list_head        list; /* module->clients */

    /*  client info. */
    pid_t                   pid; 
    pid_t                   tgid;
    char                    task_comm[TASK_COMM_LEN];
};

struct buflock_module {
    struct mutex            handles_lock;
    struct list_head        handles;
    struct list_head        free_handles;
    wait_queue_head_t       free_queue;
    size_t                  next_id;

    struct mutex            clients_lock;
    struct list_head        clients; /* buflock_client */

    struct task_struct      *kthread;

    dev_t                   dev_num;
    struct device *         device;
    struct cdev             dev;
    struct class *          dev_class;

    buflock_status_t *      pStatusMemory;
    unsigned int            uStatusPhyAddr;

    unsigned int            release_loop_ms;
    unsigned int            destroy_timeout_ms;
    size_t                  max_handles;
    struct dma_buf          *dmabuf;
    int                     ion_handle;
};

static struct buflock_handle *  buflock_handle_create               (struct buflock_module * module, size_t id, buflock_fwid_t fwid, buflock_status_t * pStatus);
static void                     buflock_handle_destroy              (struct kref *kref);
static unsigned int             buflock_handle_get_fwid             (struct buflock_handle * handle);
static void                     buflock_handle_get                  (struct buflock_handle * handle);
static void                     buflock_handle_put                  (struct buflock_handle * handle);
static unsigned int             buflock_handle_refCount             (struct buflock_handle * handle);
static int                      buflock_handle_setStatus            (struct buflock_handle * handle, buflock_status_t status);
static buflock_status_t         buflock_handle_getStatus            (struct buflock_handle * handle);
static size_t                   buflock_module_get_unused_id_unlock (struct buflock_module * module);
static struct buflock_handle *  buflock_module_handle_create        (struct buflock_module * module);
static struct buflock_handle *  buflock_module_handle_get_by_fwid   (struct buflock_module * module, buflock_fwid_t fwid);
static int                      buflock_module_independent          (struct buflock_module * module, buflock_fwid_t fwid, struct buflock_client * pClient);
static void                     buflock_module_handle_destroy       (struct kref *kref);
static int                      buflock_fops_open                   (struct inode *inode, struct file *file);
static int                      buflock_fops_close                  (struct inode *inode, struct file *file);
static ssize_t                  buflock_fops_read                   (struct file *file, char *buf, size_t size, loff_t *f_pos);
static ssize_t                  buflock_fops_write                  (struct file *file, const char *buf, size_t size, loff_t *f_pos);
static long                     buflock_fops_ioctl                  (struct file *file, unsigned int cmd, unsigned long arg);
static int                      buflock_client_release_get_fwid     (struct buflock_client * client, buflock_fwid_t fwid);

static struct file_operations buflock_fops = {
    .owner                  = THIS_MODULE,
    .open                   = buflock_fops_open,
    .release                = buflock_fops_close,
    .read                   = buflock_fops_read,
    .write                  = buflock_fops_write,
    .unlocked_ioctl         = buflock_fops_ioctl,
    .compat_ioctl           = buflock_fops_ioctl,
};

static char * getStatusStr (buflock_status_t st) {
    switch (st) {
        case E_BUFLOCK_ST_LOCK      : return "LOCK" ;
        case E_BUFLOCK_ST_UNLOCK    : return "UNLOCK" ;
        case E_BUFLOCK_ST_RELEASE   : return "RELEASE" ;
        case E_BUFLOCK_ST_NORMAL    : return "NORMAL" ;
        case E_BUFLOCK_ST_TOUCH     : return "TOUCH" ;
        case E_BUFLOCK_ST_ERROR     : return "ERROR" ;
        default: return "ERROR2";
    };
}

static struct buflock_handle * buflock_handle_create(struct buflock_module * module,
        size_t id, buflock_fwid_t fwid, buflock_status_t * pStatus)
{
    struct buflock_handle * ret = NULL;
    do {
        ret = (struct buflock_handle *) kzalloc(sizeof(struct buflock_handle), GFP_KERNEL);

        kref_init       (&ret->ref);
        mutex_init      (&ret->mutex);
        INIT_LIST_HEAD  (&ret->list);

        ret->module     = module;
        ret->id         = id;
        ret->fwid       = fwid;
        ret->pStatus    = pStatus;

        ret->destroy    = buflock_handle_destroy;
        ret->getFWID    = buflock_handle_get_fwid;
        ret->get        = buflock_handle_get;
        ret->put        = buflock_handle_put;
        ret->refCount   = buflock_handle_refCount;
        ret->setStatus  = buflock_handle_setStatus;
        ret->getStatus  = buflock_handle_getStatus;

    } while (0);
    return ret;
}

static void buflock_handle_destroy(struct kref *kref)
{
    struct buflock_handle * handle = container_of(kref, struct buflock_handle, ref);
    mutex_destroy(&handle->mutex);
    kfree(handle);
}

static unsigned int buflock_handle_get_fwid (struct buflock_handle * handle)
{
    unsigned int ret;
    mutex_lock(&handle->mutex);
    ret = handle->fwid;
    mutex_unlock(&handle->mutex);
    return ret;
}

static void buflock_handle_get(struct buflock_handle * handle)
{
    kref_get(&handle->ref);
}

static void buflock_handle_put(struct buflock_handle * handle)
{
    kref_put(&handle->ref, handle->destroy);
}

static unsigned int buflock_handle_refCount(struct buflock_handle * handle)
{
    return kref_read(&handle->ref);
}

static int buflock_handle_setStatus (struct buflock_handle * handle, buflock_status_t status)
{
    int ret = -1;
    mutex_lock(&handle->mutex);
    if (handle->pStatus) {
        writel(status, handle->pStatus);
        ret = 0;
    }

    handle->sTime.lastSet = ktime_get();

    mutex_unlock(&handle->mutex);
    return ret;
}

static buflock_status_t buflock_handle_getStatus(struct buflock_handle * handle)
{
    buflock_status_t ret = E_BUFLOCK_ST_ERROR;
    mutex_lock(&handle->mutex);
    if (handle->pStatus)
        ret = readl(handle->pStatus);
    mutex_unlock(&handle->mutex);
    return ret;
}

/****************************************************************/

static struct buflock_handle * buflock_module_handle_create        (struct buflock_module * module)
{
    struct buflock_handle * ret = NULL;
    mutex_lock(&module->handles_lock);
    do {
        size_t              id      = buflock_module_get_unused_id_unlock(module);
        buflock_fwid_t      fwid    = module->uStatusPhyAddr + OFFSET_FROM_ID(id);
        buflock_status_t *  pStatus = &module->pStatusMemory[OFFSET_FROM_ID(id)];
        struct buflock_handle * handle;

        if (id >= module->max_handles)
            break;

        module->next_id = id + 1;
        if (module->next_id >= module->max_handles)
            module->next_id = 0;

        handle = buflock_handle_create(module, id, fwid, pStatus);

        if (!handle)
            break;

        handle->sTime.create = ktime_get();

        handle->module  = module;
        handle->destroy = buflock_module_handle_destroy;
        handle->setStatus(handle, E_BUFLOCK_ST_NORMAL);
        list_add(&handle->list, &module->handles);

        pr_debug("%s [+] handle=%p (id=%zu fwid=%u, status=%s)\n", MODULE_TAG, handle,
                id, fwid, getStatusStr(handle->getStatus(handle)));

        ret = handle;
    } while(0);
    mutex_unlock(&module->handles_lock);
    return ret;
}

static size_t buflock_module_get_unused_id_unlock (struct buflock_module * module)
{
    size_t ret = module->next_id;
    struct buflock_handle * handle, * tmp_handle;
REDISCOVER:
    list_for_each_entry_safe(handle,  tmp_handle, &module->handles, list) {
        if (handle && handle->id == ret) {
            ret++;
            goto REDISCOVER;
        }
    }
    list_for_each_entry_safe(handle,  tmp_handle, &module->free_handles, list) {
        if (handle && handle->id == ret) {
            ret++;
            goto REDISCOVER;
        }
    }

    if (ret == module->max_handles && module->next_id != 0) {
        module->next_id = 0;
        ret = 0;
        goto REDISCOVER;
    }

    return ret;
}

static void buflock_module_handle_destroy (struct kref *kref)
{
    struct buflock_handle * handle = container_of(kref, struct buflock_handle, ref);
    struct buflock_module * module = handle->module;
    bool delayed_release = true;

    mutex_lock(&module->handles_lock);
    list_del(&handle->list);

    pr_debug("%s [-] handle=%p (id=%zu fwid=%u, status=%s)\n", MODULE_TAG, handle,
            handle->id, handle->getFWID(handle), getStatusStr(handle->getStatus(handle)));

    handle->sTime.destroy = ktime_get();

    switch (handle->getStatus(handle)) {
        case E_BUFLOCK_ST_NORMAL:
        case E_BUFLOCK_ST_RELEASE:
            break;
        case E_BUFLOCK_ST_LOCK:
            handle->setStatus(handle, E_BUFLOCK_ST_UNLOCK);
            delayed_release = true;
            break;
        case E_BUFLOCK_ST_UNLOCK:
            delayed_release = true;
            break;
        default:
            pr_debug("%s [DEBUG STATE] handle=%p (id=%zu fwid=%u, status=%s)\n", MODULE_TAG, handle,
                    handle->id, handle->getFWID(handle), getStatusStr(handle->getStatus(handle)));
            break;
    }

    if (delayed_release) {
        list_add(&handle->list, &module->free_handles);
    } else {
        handle->setStatus(handle, E_BUFLOCK_ST_ERROR);
        buflock_handle_destroy(&handle->ref);
    }

    wake_up(&module->free_queue);
    mutex_unlock(&module->handles_lock);
}

static size_t buflock_module_free_count (struct buflock_module * module)
{
    size_t ret = 0;
    mutex_lock(&module->handles_lock);
    {
        struct buflock_handle * handle, * tmp_handle;
        list_for_each_entry_safe(handle,  tmp_handle, &module->free_handles, list) {
            if (handle)
                ret++;
        }
    }
    mutex_unlock(&module->handles_lock);
    return ret;
}

static int buflock_module_free_thread(void *data)
{
    struct buflock_module * module = (struct buflock_module *)data;
    size_t free_cnt = 0;
    for (;;) {
        if (kthread_should_stop())
            break;

        wait_event_interruptible_timeout(module->free_queue,
                (free_cnt != buflock_module_free_count(module)),
                msecs_to_jiffies(module->release_loop_ms));

        mutex_lock(&module->handles_lock);
        {
            struct buflock_handle * handle, * tmp_handle;
            list_for_each_entry_safe(handle,  tmp_handle, &module->free_handles, list) {
                bool remove = false;
                if (!handle)
                    continue;
                switch (handle->getStatus(handle)) {
                    case E_BUFLOCK_ST_NORMAL:
                    case E_BUFLOCK_ST_RELEASE:
                        remove = true;
                        break;
                    default:
                        {
                            ktime_t now = ktime_get();
                            if (ktime_ms_delta(now, handle->sTime.destroy) > module->destroy_timeout_ms) {
                                remove = true;
                                pr_err("%s Destroy handle(%p) timeout! ***FORCED RELEASE*** (id=%zu fwid=%u, status=%s)\n",
                                        MODULE_TAG, handle,
                                        handle->id, handle->getFWID(handle), getStatusStr(handle->getStatus(handle)));
                            }
                        }
                        break;
                }

                if (remove) {
                    handle->setStatus(handle, E_BUFLOCK_ST_ERROR);
                    list_del(&handle->list);
                    buflock_handle_destroy(&handle->ref);
                }
            }
        }
        mutex_unlock(&module->handles_lock);

        free_cnt = buflock_module_free_count(module);
    }

    return 0;
}

static struct buflock_handle * buflock_module_handle_get_by_fwid (struct buflock_module * module, buflock_fwid_t fwid)
{
    struct buflock_handle * ret = NULL;

    mutex_lock(&module->handles_lock);
    {
        struct buflock_handle * handle, * tmp_handle;
        list_for_each_entry_safe(handle,  tmp_handle, &module->handles, list) {
            if (handle && buflock_handle_get_fwid(handle) == fwid) {
                buflock_handle_get(handle);
                ret = handle;
                break;
            }
        }
    }
    mutex_unlock(&module->handles_lock);
    return ret;
}

static int buflock_module_independent (struct buflock_module * module,
        buflock_fwid_t fwid, struct buflock_client * pClient)
{
    int ret = -1;

    mutex_lock(&module->clients_lock);
    {
        struct buflock_client * client, * tmp_client;
        list_for_each_entry_safe(client,  tmp_client, &module->clients, list) {
            if (!client || pClient == client)
                continue;
            if (buflock_client_release_get_fwid(client, fwid) == 0)
                ret = 0;
        }
    }
    mutex_unlock(&module->clients_lock);
    return ret;
}

static phys_addr_t buflock_mem_pa(struct ion_buffer *buffer)
{
	unsigned long ret = -1UL;
	struct sg_table *table;
	struct page *page;
	phys_addr_t paddr;

	mutex_lock(&buffer->lock);
	table = buffer->sg_table;
	page = sg_page(table->sgl);
	paddr = PFN_PHYS(page_to_pfn(page));
	mutex_unlock(&buffer->lock);

	ret = paddr;

	return ret;
}

static void *buflock_mem_va(struct ion_buffer *buffer)
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

	if (buffer->flags & ION_FLAG_NONCACHED)
		pgprot = pgprot_noncached(PAGE_KERNEL);
	else if (buffer->flags & ION_FLAG_CACHED)
		pgprot = PAGE_KERNEL;
	else
		pgprot = pgprot_writecombine(PAGE_KERNEL);

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

static struct buflock_module * gModule = NULL;
static int __init buflock_module_init (void)
{
    int ret = -1;
    do {
        struct buflock_module * module;

        if (gModule) {
            ret = 0;
            break;
        }

        gModule = (struct buflock_module *) kzalloc(sizeof(struct buflock_module), GFP_KERNEL);

        module = gModule;

        module->release_loop_ms = 5;
        module->destroy_timeout_ms = 5000;
        module->max_handles = 4096;
        module->next_id = 0;
        INIT_LIST_HEAD(&module->handles);
        INIT_LIST_HEAD(&module->free_handles);
        mutex_init(&module->handles_lock);

        INIT_LIST_HEAD(&module->clients);
        mutex_init(&module->clients_lock);

        do {
            phys_addr_t         ion_phyAddr;
            size_t                  ion_size;
            unsigned int ion_flag_mask = ION_FLAG_NONCACHED | ION_USAGE_MMAP_NONCACHED;
            ion_flag_mask |= ION_FLAG_SCPUACC | ION_FLAG_ACPUACC | ION_FLAG_VCPU_FWACC;

            module->dmabuf = ion_alloc(sizeof(unsigned int) * module->max_handles,
                    RTK_ION_HEAP_AUDIO_MASK, ion_flag_mask);
            if (IS_ERR(module->dmabuf))
                module->dmabuf = ion_alloc(sizeof(unsigned int) * module->max_handles,
                    RTK_ION_HEAP_MEDIA_MASK, ion_flag_mask);

            if (IS_ERR(module->dmabuf)) {
                ret = PTR_ERR(module->dmabuf);
                break;
            }

            ion_phyAddr = buflock_mem_pa(module->dmabuf->priv);

            if (ion_phyAddr == 0) {
            	dma_buf_put(module->dmabuf);
                module->ion_handle = 0;
                break;
            }

            module->uStatusPhyAddr = (unsigned int) ion_phyAddr;
            module->pStatusMemory = buflock_mem_va(module->dmabuf->priv);
            break;
        } while (0);

        if (!module->pStatusMemory) {
            module->pStatusMemory = (buflock_status_t *) kzalloc(sizeof(unsigned int) * module->max_handles, GFP_KERNEL);
            module->uStatusPhyAddr = (unsigned int) virt_to_phys(module->pStatusMemory);
        }

        {
            cdev_init(&module->dev, &buflock_fops);
            if (alloc_chrdev_region(&module->dev_num, 0, 1, MODULE_NAME)!=0) {
                cdev_del(&module->dev);
                kfree(module);
                gModule = NULL;
                break;
            }

            if (cdev_add(&module->dev, module->dev_num, 1)<0) {
                cdev_del(&module->dev);
                kfree(module);
                gModule = NULL;
                break;
            }

            module->dev_class = class_create(THIS_MODULE, MODULE_NAME);

            module->device = device_create(
                    module->dev_class,     // class
                    NULL,                  // parent
                    module->dev_num,       // dev number
                    NULL,                  // driver data
                    MODULE_NAME);          // device name
        }

        init_waitqueue_head(&module->free_queue);
        module->kthread = kthread_create(buflock_module_free_thread, module, "buflock_release_thread");
        wake_up_process(module->kthread);

        ret = 0;
    } while (0);
    return ret;
}

static void __exit buflock_module_exit(void)
{
    struct buflock_module * module = gModule;
    if (!module)
        return;

    kthread_stop(module->kthread);

    {
        struct buflock_handle * handle, * tmp_handle;
        list_for_each_entry_safe(handle,  tmp_handle, &module->free_handles, list) {
            if (handle) {
                list_del(&handle->list);
                buflock_handle_destroy(&handle->ref);
            }
        }

        list_for_each_entry_safe(handle,  tmp_handle, &module->handles, list) {
            if (handle) {
                list_del(&handle->list);
                buflock_handle_destroy(&handle->ref);
            }
        }
    }

    {
        device_destroy(module->dev_class, module->device->devt);
        class_destroy(module->dev_class);
        cdev_del(&module->dev);
        unregister_chrdev_region(module->dev_num, 1);
    }

    if (module->ion_handle) {
    	dma_buf_put(module->dmabuf);
    } else if (module->pStatusMemory)
        kfree(module->pStatusMemory);

    mutex_destroy(&module->handles_lock);
    mutex_destroy(&module->clients_lock);

    kfree(module);
    gModule = NULL;
}

static struct buflock_slot * buflock_slot_create(struct buflock_handle * handle)
{
    struct buflock_slot * slot = NULL;
    do {
        slot = (struct buflock_slot *)
            kzalloc(sizeof(struct buflock_slot), GFP_KERNEL);

        mutex_init (&slot->mutex);
        INIT_LIST_HEAD (&slot->list);
        slot->handle = handle;
    } while (0);
    return slot;
}

static void buflock_slot_destroy(struct buflock_slot * slot)
{
    if (slot->handle)
        slot->handle->put(slot->handle);
    kfree(slot);
}

static struct buflock_client * buflock_client_create(struct buflock_module * module)
{
    struct buflock_client * ret = NULL;
    do {
        ret = (struct buflock_client *) kzalloc(sizeof(struct buflock_client), GFP_KERNEL);
        mutex_init      (&ret->mutex);
        INIT_LIST_HEAD  (&ret->slots);
        INIT_LIST_HEAD  (&ret->list);
        ret->module = module;

        {
            struct task_struct *task;
            get_task_struct(current->group_leader);

            task_lock(current->group_leader);
            ret->pid = task_pid_nr(current->group_leader);
            ret->tgid = task_tgid_nr(current->group_leader);
            if (current->group_leader->flags & PF_KTHREAD)
                task = NULL;
            else
                task = current->group_leader;
            task_unlock(current->group_leader);

            if (task)
                get_task_comm(ret->task_comm, task);
            else
                strncpy(ret->task_comm, "KTHREAD", sizeof(ret->task_comm));

            put_task_struct(current->group_leader);
        }
    } while (0);
    return ret;
}

static void buflock_client_destroy(struct buflock_client * client)
{
    mutex_lock(&client->mutex);
    {
        struct buflock_slot * slot, * tmp_slot;
        list_for_each_entry_safe(slot,  tmp_slot, &client->slots, list) {
            list_del(&slot->list);
            buflock_slot_destroy(slot);
        }
    }
    mutex_unlock(&client->mutex);
    mutex_destroy(&client->mutex);
    kfree(client);
}

static int buflock_client_alloc_get_fwid (struct buflock_client * client, buflock_fwid_t * fwid)
{
    int ret = -1;
    mutex_lock(&client->mutex);
    do {
        struct buflock_handle * handle;
        struct buflock_slot * slot;

        if (!fwid)
            break;

        handle = buflock_module_handle_create(client->module);

        if (!handle)
            break;

        slot = buflock_slot_create(handle);

        if (!slot) {
            handle->put(handle);
            break;
        }

        list_add (&slot->list, &client->slots);

        *fwid = handle->getFWID(handle);

        ret = 0;
    } while (0);
    mutex_unlock(&client->mutex);
    return ret;
}

static int buflock_client_import_get_fwid (struct buflock_client * client, buflock_fwid_t fwid)
{
    int ret = -1;
    mutex_lock(&client->mutex);
    do {
        struct buflock_handle * handle = NULL;
        struct buflock_slot * slot, * tmp_slot;

        list_for_each_entry_safe(slot,  tmp_slot, &client->slots, list) {
            if (slot && slot->handle && slot->handle->getFWID(slot->handle) == fwid) {
                handle = slot->handle;
                break;
            }
        }

        if (handle) {
            ret = 0;
            break;
        }

        handle = buflock_module_handle_get_by_fwid(client->module, fwid);

        if (!handle)
            break;

        slot = buflock_slot_create(handle);

        if (!slot) {
            handle->put(handle);
            break;
        }

        list_add (&slot->list, &client->slots);

        ret = 0;
    } while (0);
    mutex_unlock(&client->mutex);
    return ret;
}

static int buflock_client_release_get_fwid (struct buflock_client * client, buflock_fwid_t fwid)
{
    int ret = -1;
    mutex_lock(&client->mutex);
    {
        struct buflock_slot * slot, * tmp_slot;
        list_for_each_entry_safe(slot,  tmp_slot, &client->slots, list) {
            if (slot && slot->handle && slot->handle->getFWID(slot->handle) == fwid) {
                list_del(&slot->list);
                buflock_slot_destroy(slot);
                ret = 0;
                break;
            }
        }
    }
    mutex_unlock(&client->mutex);
    return ret;
}

static int buflock_client_set_status(struct buflock_client * client,
        buflock_fwid_t fwid, buflock_status_t status)
{
    int ret = -1;
    mutex_lock(&client->mutex);
    {
        struct buflock_slot * slot, * tmp_slot;
        list_for_each_entry_safe(slot,  tmp_slot, &client->slots, list) {
            if (slot && slot->handle && slot->handle->getFWID(slot->handle) == fwid) {
                struct buflock_handle * handle = slot->handle;
                ret = handle->setStatus(handle, status);
                break;
            }
        }
    }
    mutex_unlock(&client->mutex);
    return ret;
}

static int buflock_client_get_status(struct buflock_client * client,
        buflock_fwid_t fwid, buflock_status_t * status)
{
    int ret = -1;
    mutex_lock(&client->mutex);
    {
        struct buflock_slot * slot, * tmp_slot;
        list_for_each_entry_safe(slot,  tmp_slot, &client->slots, list) {
            if (slot && slot->handle && slot->handle->getFWID(slot->handle) == fwid) {
                struct buflock_handle * handle = slot->handle;
                *status = handle->getStatus(handle);
                ret = 0;
                break;
            }
        }
    }
    mutex_unlock(&client->mutex);
    return ret;
}

static int buflock_client_get_refCount(struct buflock_client * client,
        buflock_fwid_t fwid, unsigned int * pCount)
{
    int ret = -1;
    mutex_lock(&client->mutex);
    {
        struct buflock_slot * slot, * tmp_slot;
        list_for_each_entry_safe(slot,  tmp_slot, &client->slots, list) {
            if (slot && slot->handle && slot->handle->getFWID(slot->handle) == fwid) {
                struct buflock_handle * handle = slot->handle;
                *pCount = handle->refCount(handle);
                ret = 0;
                break;
            }
        }
    }
    mutex_unlock(&client->mutex);
    return ret;
}

static int buflock_client_independent(struct buflock_client * client,
        buflock_fwid_t fwid)
{
    int ret = -1;
    mutex_lock(&client->mutex);
    {
        struct buflock_slot * slot, * tmp_slot;
        list_for_each_entry_safe(slot,  tmp_slot, &client->slots, list) {
            if (slot && slot->handle && slot->handle->getFWID(slot->handle) == fwid) {
                ret = buflock_module_independent(client->module, fwid, client);
                break;
            }
        }
    }
    mutex_unlock(&client->mutex);
    return ret;
}

static int buflock_fops_open (struct inode *inode, struct file *file)
{
    int ret = -1;
    do {
        struct cdev * dev  = inode->i_cdev;
        struct buflock_module * module = container_of(dev, struct buflock_module, dev);
        struct buflock_client * client = buflock_client_create(module);
        if (!client)
            break;

        mutex_lock(&module->clients_lock);
        list_add(&client->list, &module->clients);
        mutex_unlock(&module->clients_lock);

        file->private_data = client;
        ret = 0;
    } while (0);
    return ret;
}

static int buflock_fops_close (struct inode *inode, struct file *file)
{
    struct buflock_client * client = file->private_data;
    struct buflock_module * module = client->module;
    if (module) {
        mutex_lock(&module->clients_lock);
        list_del(&client->list);
        mutex_unlock(&module->clients_lock);
    }
    buflock_client_destroy(client);
    return 0;
}

static ssize_t buflock_fops_read (struct file *file, char *buf, size_t size, loff_t *f_pos)
{
    ssize_t ret = 0;
    struct buflock_client * f_client = file->private_data;
    struct buflock_module * module = f_client->module;
    do  {
        ktime_t now = ktime_get();
        char * str = (char *) kzalloc(size, GFP_KERNEL);
        char * s = str;
        if (!s)
            break;

        s += sprintf(s, "#%s ion(%p) phy=%08zx max=%zu checkloop_ms=%d destroy_timeout_ms=%d\n", MODULE_NAME,
                module->dmabuf, module->uStatusPhyAddr, module->max_handles, module->release_loop_ms, module->destroy_timeout_ms);

        s += sprintf(s, "    clients:\n");
        mutex_lock(&module->clients_lock);
        {
            struct buflock_client * client, * tmp_client;
            list_for_each_entry_safe(client,  tmp_client, &module->clients, list) {
                if (!client)
                    continue;
                s += sprintf(s, "        %u %u [%s]\n", client->pid, client->tgid, client->task_comm);
                mutex_lock(&client->mutex);
                {
                    struct buflock_slot * slot, * tmp_slot;
                    list_for_each_entry_safe(slot,  tmp_slot, &client->slots, list) {
                        if (!slot)
                            continue;
                        s += sprintf(s, "            %p\n", slot->handle);
                    }
                }
                mutex_unlock(&client->mutex);
            }
        }
        mutex_unlock(&module->clients_lock);

        mutex_lock(&module->handles_lock);

        s += sprintf(s, "    active handle:\n");
        {
            struct buflock_handle * handle, * tmp_handle;
            list_for_each_entry_safe(handle,  tmp_handle, &module->handles, list) {
                unsigned int lastSet_delta_us;
                if (!handle)
                    continue;
                lastSet_delta_us = ktime_us_delta(now, handle->sTime.lastSet) & -1U;
                s += sprintf(s, "        %p (id=%08x, %.8s) lastSet(%5d.%03d ms)\n", handle,
                        handle->getFWID(handle), getStatusStr(handle->getStatus(handle)),
                        lastSet_delta_us/1000, lastSet_delta_us%1000
                        );
            }
        }

        s += sprintf(s, "    inactive handle:\n");
        {
            struct buflock_handle * handle, * tmp_handle;
            list_for_each_entry_safe(handle,  tmp_handle, &module->free_handles, list) {
                unsigned int lastSet_delta_us;
                unsigned int destroy_delta_us;
                if (!handle)
                    continue;
                lastSet_delta_us = ktime_us_delta(now, handle->sTime.lastSet) & -1U;
                destroy_delta_us = ktime_us_delta(now, handle->sTime.destroy) & -1U;
                s += sprintf(s, "        %p (id=%08x, %.8s) lastSet(%5d.%03d ms), Destroy(%5d.%03d ms)\n", handle,
                        handle->getFWID(handle), getStatusStr(handle->getStatus(handle)),
                        lastSet_delta_us/1000, lastSet_delta_us%1000,
                        destroy_delta_us/1000, destroy_delta_us%1000
                        );
            }
        }

        mutex_unlock(&module->handles_lock);

        ret = simple_read_from_buffer(buf, size, f_pos, str, s-str);
        kfree(str);
    } while (0);
    return ret;
}

static ssize_t buflock_fops_write (struct file *file, const char *buf, size_t size, loff_t *f_pos)
{
    return 0;
}

static long buflock_fops_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
    long ret = -1L;
    struct buflock_client * client = (struct buflock_client *) file->private_data;
    switch (cmd) {
        case BUFLOCK_IOC_ALLOC:
            {
                struct buflock_alloc_data data;

                if (buflock_client_alloc_get_fwid(client, &data.fwid) != 0)
                    break;

                if (copy_to_user((void __user *) arg, &data, sizeof(data))) {
                    buflock_client_release_get_fwid(client, data.fwid);
                    break;
                }

                ret = 0;
            }
            break;
        case BUFLOCK_IOC_FREE:
            {
                struct buflock_import_data data;
                if (copy_from_user(&data, (void __user *) arg, sizeof(data)))
                    break;

                if (buflock_client_release_get_fwid(client, data.fwid) != 0)
                    break;
                ret = 0;
            }
            break;
        case BUFLOCK_IOC_IMPORT:
            {
                struct buflock_import_data data;
                if (copy_from_user(&data, (void __user *) arg, sizeof(data)))
                    break;

                if (buflock_client_import_get_fwid(client, data.fwid) != 0)
                    break;
                ret = 0;
            }
            break;
        case BUFLOCK_IOC_SET_STATUS:
            {
                struct buflock_status_data data;
                if (copy_from_user(&data, (void __user *) arg, sizeof(data)))
                    break;

                if (buflock_client_set_status(client, data.fwid, data.status) != 0)
                    break;
                ret = 0;
            }
            break;
        case BUFLOCK_IOC_GET_STATUS:
            {
                struct buflock_status_data data;
                if (copy_from_user(&data, (void __user *) arg, sizeof(data)))
                    break;

                if (buflock_client_get_status(client, data.fwid, &data.status) != 0)
                    break;

                if (copy_to_user((void __user *) arg, &data, sizeof(data))) {
                    buflock_client_release_get_fwid(client, data.fwid);
                    break;
                }
                ret = 0;
            }
            break;
        case BUFLOCK_IOC_GET_REF:
            {
                struct buflock_reference_data data;
                if (copy_from_user(&data, (void __user *) arg, sizeof(data)))
                    break;

                if (buflock_client_get_refCount(client, data.fwid, &data.count) != 0)
                    break;

                if (copy_to_user((void __user *) arg, &data, sizeof(data))) {
                    buflock_client_release_get_fwid(client, data.fwid);
                    break;
                }
                ret = 0;
            }
            break;
        case BUFLOCK_IOC_SET_INDIE:
            {
                struct buflock_independent_data data;
                if (copy_from_user(&data, (void __user *) arg, sizeof(data)))
                    break;

                if (buflock_client_independent(client, data.fwid) != 0)
                    break;

                if (copy_to_user((void __user *) arg, &data, sizeof(data))) {
                    break;
                }
                ret = 0;
            }
            break;
        default:
            break;
    }
    return ret;
}

module_init(buflock_module_init);
module_exit(buflock_module_exit);
MODULE_LICENSE("GPL");
