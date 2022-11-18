/*
 * Realtek RPC driver
 *
 * Copyright (c) 2017 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dma-buf.h>
#include <linux/uaccess.h>
#include <linux/fdtable.h>
#include <linux/kmemleak.h>
#include <linux/ion.h>
#include <soc/realtek/rtk_chip.h>
#include <uapi/linux/ion.h>
#include <soc/realtek/uapi/ion_rtk.h>

#include <trace/events/rtk_rpc.h>
#include "rtk_rpc.h"

#include <ion_rtk_alloc.h>
#define ion_alloc ext_rtk_ion_alloc

volatile RPC_DEV *rpc_poll_devices;

#ifdef CONFIG_REALTEK_RPC_KCPU
volatile RPC_DEV *rpc_poll_kcpu_devices;
#endif

int rpc_poll_is_paused;
int rpc_poll_is_suspend;

extern int chip_id;

struct rpc_debug_flag {
	unsigned int acpu;
	unsigned int reserve_acpu[127];
	unsigned int vcpu;
	unsigned int reserve_vcpu[127];
};

struct rpc_debug_flag_memory {
	int handle;
	struct rpc_debug_flag *debug_flag;
	phys_addr_t debug_phys;
	size_t debug_size;
	struct dma_buf *dmabuf;
};

struct rpc_debug_print_memory {
	int handle;
	void *debug_hdr;
	phys_addr_t debug_phys;
	size_t debug_size;
	struct dma_buf *dmabuf_orig;
	struct dma_buf *dmabuf;
	int32_t fd;
};

static struct rpc_debug_flag_memory *mDebugFlagMemory = NULL;
static struct rpc_debug_print_memory * mDebugPrintMemory = NULL;

static phys_addr_t rpc_mem_pa(struct ion_buffer *buffer)
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

static void *rpc_mem_va(struct ion_buffer *buffer)
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

static struct rpc_debug_flag_memory * get_debug_flag_memory(void)
{
	do {
		if (mDebugFlagMemory == NULL) {
			struct rpc_debug_flag_memory * tmp = (struct rpc_debug_flag_memory *) kzalloc(
			sizeof(struct rpc_debug_flag_memory), GFP_KERNEL);
			unsigned int ion_flag_mask = ION_FLAG_NONCACHED | ION_USAGE_MMAP_NONCACHED;

			if (tmp == NULL)
				break;

			ion_flag_mask |= ION_FLAG_SCPUACC | ION_FLAG_ACPUACC;

			ion_flag_mask |= ION_FLAG_VCPU_FWACC;

			tmp->dmabuf = ion_alloc(sizeof(struct rpc_debug_flag) ,
							RTK_ION_HEAP_AUDIO_MASK, ion_flag_mask);
			if (IS_ERR(tmp->dmabuf))
				tmp->dmabuf = ion_alloc(sizeof(struct rpc_debug_flag),
								RTK_ION_HEAP_MEDIA_MASK, ion_flag_mask);

			if (IS_ERR(tmp->dmabuf)) {
				dma_buf_put(tmp->dmabuf);
				kfree(tmp);
				break;
			}

			tmp->debug_phys = rpc_mem_pa(tmp->dmabuf->priv);

			if (tmp->debug_phys == 0) {
				dma_buf_put(tmp->dmabuf);
				kfree(tmp);
				break;
			}

			tmp->debug_flag = (struct rpc_debug_flag *) rpc_mem_va(tmp->dmabuf->priv);

			mDebugFlagMemory = tmp;
		}
	} while (0);

	return mDebugFlagMemory;
}

static struct rpc_debug_print_memory * get_debug_print_memory(void)
{
	do {
		if (mDebugPrintMemory == NULL) {
			struct rpc_debug_print_memory * tmp = (struct rpc_debug_print_memory *) kzalloc(
			sizeof(struct rpc_debug_print_memory), GFP_KERNEL);
			unsigned int ion_flag_mask = ION_FLAG_NONCACHED | ION_USAGE_MMAP_NONCACHED;

			if (tmp == NULL)
				break;

			ion_flag_mask |= ION_FLAG_SCPUACC | ION_FLAG_ACPUACC;

			ion_flag_mask |= ION_FLAG_VCPU_FWACC;

			tmp->dmabuf = dma_buf_get(tmp->handle);
			if (IS_ERR(tmp->dmabuf))
				break;

			__close_fd(current->files, tmp->handle);

			if (0 > tmp->handle) {
				dma_buf_put(tmp->dmabuf);
				kfree(tmp);
				break;
			}
			
			tmp->debug_phys = rpc_mem_pa(tmp->dmabuf->priv);

			if (tmp->debug_phys == 0) {
				dma_buf_put(tmp->dmabuf);
				kfree(tmp);
				break;
			}

			tmp->debug_hdr = (void *) rpc_mem_va(tmp->dmabuf->priv);

			mDebugPrintMemory = tmp;
		}
	} while (0);

	return mDebugPrintMemory;
}

static struct rpc_debug_flag *get_debug_flag(void)
{
	struct rpc_debug_flag_memory * debug_memory = get_debug_flag_memory();
	return (debug_memory) ? debug_memory->debug_flag : NULL;
}

static phys_addr_t get_debug_flag_phyAddr(void)
{
	struct rpc_debug_flag_memory * debug_memory = get_debug_flag_memory();
	return (debug_memory) ? debug_memory->debug_phys : -1UL;
}

static struct rpc_debug_print_memory *get_debug_print(void)
{
	struct rpc_debug_print_memory * debug_print = get_debug_print_memory();
	return (debug_print) ? debug_print : NULL;
}

static phys_addr_t get_debug_print_phyAddr(void)
{
	struct rpc_debug_print_memory * debug_print = get_debug_print_memory();
	return (debug_print) ? debug_print->debug_phys : -1UL;
}

RPC_DEV_EXTRA rpc_poll_extra[RPC_NR_DEVS/RPC_NR_PAIR];

int rpc_poll_init(void)
{
	static int is_init;
	int result = 0, i;

	is_init = 0;

	/* Create corresponding structures for each device. */
	rpc_poll_devices = (RPC_DEV *)AVCPU2SCPU(RPC_POLL_RECORD_ADDR);

	for (i = 0; i < RPC_INTR_DEV_TOTAL; i++) {
		pr_debug("rpc_poll_device %d addr: %p\n", i, &rpc_poll_devices[i]);
		rpc_poll_devices[i].ringBuf = RPC_POLL_DEV_ADDR + i*RPC_RING_SIZE*2;

		/* Initialize pointers... */
		rpc_poll_devices[i].ringStart = rpc_poll_devices[i].ringBuf;
		rpc_poll_devices[i].ringEnd   = rpc_poll_devices[i].ringBuf + RPC_RING_SIZE;
		rpc_poll_devices[i].ringIn	= rpc_poll_devices[i].ringBuf;
		rpc_poll_devices[i].ringOut   = rpc_poll_devices[i].ringBuf;

		pr_debug("The %dth poll dev:\n", i);
		pr_debug("RPC ringStart: %p\n", AVCPU2SCPU(rpc_poll_devices[i].ringStart));
		pr_debug("RPC ringEnd:   %p\n", AVCPU2SCPU(rpc_poll_devices[i].ringEnd));
		pr_debug("RPC ringIn:	%p\n", AVCPU2SCPU(rpc_poll_devices[i].ringIn));
		pr_debug("RPC ringOut:   %p\n", AVCPU2SCPU(rpc_poll_devices[i].ringOut));
		pr_debug("\n");

		rpc_poll_extra[i].nextRpc = rpc_poll_devices[i].ringOut;
		rpc_poll_extra[i].currProc = NULL;

		if (!is_init) {
			rpc_poll_devices[i].ptrSync = kmalloc(sizeof(RPC_SYNC_Struct), GFP_KERNEL);
			kmemleak_not_leak(rpc_poll_devices[i].ptrSync);

			/* Initialize wait queue... */
			//init_waitqueue_head(&(rpc_poll_devices[i].ptrSync->waitQueue));

			/* Initialize sempahores... */
			init_rwsem(&rpc_poll_devices[i].ptrSync->readSem);
			init_rwsem(&rpc_poll_devices[i].ptrSync->writeSem);

			rpc_poll_extra[i].dev = (void *)&rpc_poll_devices[i];
			INIT_LIST_HEAD(&rpc_poll_extra[i].tasks);
			//tasklet_init(&rpc_poll_extra[i].tasklet, rpc_dispatch, (unsigned long)&rpc_poll_extra[i]);
			spin_lock_init(&rpc_poll_extra[i].lock);
			switch (i) {
			case 0:
				rpc_poll_extra[i].name = "AudioPollWrite";
				break;
			case 1:
				rpc_poll_extra[i].name = "AudioPollRead";
				break;
			case 2:
				rpc_poll_extra[i].name = "Video1PollWrite";
				break;
			case 3:
				rpc_poll_extra[i].name = "Video1PollRead";
				break;
			case 4:
				rpc_poll_extra[i].name = "Video2PollWrite";
				break;
			case 5:
				rpc_poll_extra[i].name = "Video2PollRead";
				break;
			}
		}
	}

	is_init = 1;
	rpc_poll_is_paused = 0;
	rpc_poll_is_suspend = 0;

	return result;
}

int rpc_poll_pause(void)
{
	rpc_poll_is_paused = 1;
	return 0;
}

int rpc_poll_suspend(void)
{
	rpc_poll_is_suspend = 1;
	return 0;
}

int rpc_poll_resume(void)
{
	rpc_poll_is_suspend = 0;
	return 0;
}

void rpc_poll_cleanup(void)
{
	int num = RPC_NR_DEVS/RPC_NR_PAIR, i;

	/* Clean corresponding structures for each device. */
	if (rpc_poll_devices) {
		/* Clean ring buffers. */
		for (i = 0; i < num; i++) {
			//if (rpc_poll_devices[i].ringBuf)
			//kfree(rpc_poll_devices[i].ringBuf);
		}
		//kfree(rpc_poll_devices);
	}

	return;
}

int rpc_poll_open(struct inode *inode, struct file *filp)
{
	int minor = MINOR(inode->i_rdev);

	if (minor == 100) {
		filp->f_op = &rpc_ctrl_fops;
		return filp->f_op->open(inode, filp); /* dispatch to specific open */
	}

	/*
	 * If private data is not valid, we are not using devfs
	 * so use the minor number to select a new f_op
	 */
	if (!filp->private_data && (minor >= RPC_NR_DEVS)) {
		filp->f_op = rpc_fop_array[1];
		return filp->f_op->open(inode, filp);
	}
	if (!filp->private_data && (minor%RPC_NR_PAIR != 0)) {
		filp->f_op = rpc_fop_array[minor%RPC_NR_PAIR];
		return filp->f_op->open(inode, filp); /* dispatch to specific open */
	}

	pr_debug("RPC poll open with minor number: %d\n", minor);

		if (!filp->private_data) {
			RPC_PROCESS *proc = kmalloc(sizeof(RPC_PROCESS), GFP_KERNEL | __GFP_ZERO);

			if (proc == NULL) {
				pr_err("%s: failed to allocate RPC_PROCESS", __func__);
				return -ENOMEM;
			}

#ifdef CONFIG_REALTEK_RPC_KCPU
			if (minor >= RPC_NR_DEVS)
				proc->dev = &rpc_poll_kcpu_devices[(minor-RPC_NR_DEVS)/RPC_NR_PAIR];
			else
#endif
				proc->dev = (RPC_DEV *)&rpc_poll_devices[minor/RPC_NR_PAIR];

			proc->extra = &rpc_poll_extra[minor/RPC_NR_PAIR];
			/* current->tgid = process id, current->pid = thread id */
			proc->pid = current->tgid;
            proc->bStayActive = false;

			init_waitqueue_head(&proc->waitQueue);
			INIT_LIST_HEAD(&proc->threads);
#ifdef CONFIG_REALTEK_RPC_PROGRAM_REGISTER
			INIT_LIST_HEAD(&proc->handlers);
#endif
			spin_lock_bh(&proc->extra->lock);
			list_add(&proc->list, &proc->extra->tasks);
			spin_unlock_bh(&proc->extra->lock);
			pr_debug("%s: Current process pid:%d tgid:%d => %d(%p) for %s(%p)\n", __func__, current->pid, current->tgid, proc->pid, &proc->waitQueue, proc->extra->name, proc->dev);

			filp->private_data = proc;
		}

	//MOD_INC_USE_COUNT; /* Before we maybe sleep */

	return 0;
}

int rpc_poll_release(struct inode *inode, struct file *filp)
{
#ifdef CONFIG_REALTEK_RPC_PROGRAM_REGISTER
	RPC_HANDLER *handler, *hdltmp;
#endif
	RPC_THREAD *th, *thtmp;
	int minor = MINOR(inode->i_rdev);

	RPC_PROCESS *proc = filp->private_data;
	RPC_DEV *dev = proc->dev; /* the first listitem */
	RPC_DEV_EXTRA *extra = proc->extra;

	if (extra->currProc == proc) {
		pr_debug("%s: clear %s(%p) current process\n", __func__, proc->extra->name, dev);
		update_currProc(extra, NULL);
		if (minor == 2 || minor == 6 || minor == 10) { /* poll read device (ugly code) */
			if (!rpc_done(extra)) {
				pr_err("%s: previous rpc hasn't finished, force clear!! ringOut %p => %p\n", __func__,
				AVCPU2SCPU(dev->ringOut), AVCPU2SCPU(extra->nextRpc));
				down_write(&dev->ptrSync->readSem);
				dev->ringOut = extra->nextRpc;
				up_write(&dev->ptrSync->readSem);
			}
		}
	}

	spin_lock_bh(&extra->lock);

#ifdef CONFIG_REALTEK_RPC_PROGRAM_REGISTER
	//unregister myself from handler list
	list_for_each_entry_safe(handler, hdltmp, &proc->handlers, list) {
		list_del(&handler->list);
		kfree(handler);
	}
#endif

	list_for_each_entry_safe(th, thtmp, &proc->threads, list) {
		list_del(&th->list);
		kfree(th);
	}

	/* remove myself from task list */
	list_del(&proc->list);
	kfree(proc);

	spin_unlock_bh(&extra->lock);

	pr_debug("RPC poll close with minor number: %d\n", minor);

//	MOD_DEC_USE_COUNT;

	return 0;
}

/* We don't need parameter f_pos here... */
ssize_t rpc_poll_read(struct file *filp, char *buf, size_t count,
	loff_t *f_pos)
{
	RPC_PROCESS *proc = filp->private_data;
	RPC_DEV *dev = proc->dev; /* the first listitem */
	RPC_DEV_EXTRA *extra = proc->extra;
	int temp, size;
	size_t r;
	ssize_t ret = 0;
	uint32_t ptmp;
	int rpc_ring_size = dev->ringEnd - dev->ringStart;

	//pr_debug("%s:%d thread:%s pid:%d tgid:%d device:%s\n", __func__, __LINE__, current->comm, current->pid, current->tgid, extra->name);
	if (rpc_poll_is_paused) {
		pr_err("RPCpoll: someone access rpc poll during the pause...\n");
		pr_err("%s:%d buf:%p count:%lu EAGAIN\n", __func__, __LINE__, buf, count);
		msleep(1000);
		return -EAGAIN;
	}

	while (rpc_poll_is_suspend) {
		pr_warn("RPCpoll: someone access rpc poll during the suspend!!!...\n");
		msleep(1000);
	}

	wmb();
	down_write(&dev->ptrSync->readSem);

	if (need_dispatch(extra)) {
		tasklet_schedule(&(extra->tasklet));
	}

	//pr_debug("%s: dev:%s(%p) currProc:%p\n", __func__, extra->name, dev, extra->currProc);
	if ((extra->currProc != proc) || (ring_empty(extra))) {
		if (unlikely(!(filp->f_flags & O_NONBLOCK))) {
			//pr_warn("%s:%d:%s Warning: pid:%d use blocking mode with poll buffer!\n", __func__, __LINE__, extra->name, current->pid);
		}
		goto out; //return anyway
	}

	if (dev->ringIn > dev->ringOut)
		size = dev->ringIn - dev->ringOut;
	else
		size = rpc_ring_size + dev->ringIn - dev->ringOut;

	//pr_debug("%s: %s: count:%lu size:%d\n", __func__, extra->name, count, size);
	//peek_rpc_struct(__func__, dev, f_pos);
	if (count > size)
			count = size;

	temp = dev->ringEnd - dev->ringOut;
	if (temp >= count) {
#ifdef MY_COPY
		r = my_copy_to_user((int *)buf, (int *)AVCPU2SCPU(dev->ringOut), count);
#else
		r = copy_to_user((int *)buf, (int *)AVCPU2SCPU(dev->ringOut), count);
#endif
		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n", __func__, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}

		ret += count;
		ptmp = dev->ringOut + ((count+3) & 0xfffffffc);

		if (ptmp == dev->ringEnd)
			dev->ringOut = dev->ringStart;
		else
			dev->ringOut = ptmp;

		//pr_debug("RPC Read is in 1st kind...\n");
	} else {
#ifdef MY_COYP
		r = my_copy_to_user((int *)buf, (int *)AVCPU2SCPU(dev->ringOut), temp);
#else
		r = copy_to_user((int *)buf, (int *)AVCPU2SCPU(dev->ringOut), temp);
#endif
		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n", __func__, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}

		count -= temp;

#ifdef MY_COPY
		r = my_copy_to_user((int *)(buf+temp), (int *)AVCPU2SCPU(dev->ringStart), count);
#else
		r = copy_to_user((int *)(buf+temp), (int *)AVCPU2SCPU(dev->ringStart), count);
#endif
		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n", __func__, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}

		ret += (temp + count);
		dev->ringOut = dev->ringStart+((count+3) & 0xfffffffc);

		//pr_debug("RPC Read is in 2nd kind...\n");
	}

	spin_lock_bh(&extra->lock);
	if (rpc_done(extra)) {
		pr_debug("%s: Previous RPC is done, unregister myself\n", __func__);
		update_currProc(extra, NULL);
	}
	spin_unlock_bh(&extra->lock);

	//process next rpc command if any
	if (need_dispatch(extra))
		tasklet_schedule(&(extra->tasklet));

	pr_debug("%s:%d buf:%p count:%lu actual:%lu\n", __func__, __LINE__, buf, count, ret);

out:
	up_write(&dev->ptrSync->readSem);
	wmb();
	//pr_debug("RPC poll ringOut pointer is : 0x%8x\n", (int)AVCPU2SCPU(dev->ringOut));
	//pr_debug("%s:%d pid:%d reads %d bytes\n", extra->name, __LINE__, current->pid, ret);
	return ret;
}

/* We don't need parameter f_pos here... */
ssize_t rpc_poll_write(struct file *filp, const char *buf, size_t count,
	loff_t *f_pos)
{
	RPC_PROCESS *proc = filp->private_data;
	RPC_DEV *dev = proc->dev; /* the first listitem */
	RPC_DEV_EXTRA *extra = proc->extra;
	RPC_DEV_EXTRA *rextra = extra + 1;
	int temp, size;
	size_t r;
	ssize_t ret = 0;
	uint32_t ptmp;
	int rpc_ring_size = dev->ringEnd - dev->ringStart;

	if (rpc_poll_is_paused) {
		pr_warn("RPCpoll: someone access rpc poll during the pause...\n");
		msleep(1000);
		return -EAGAIN;
	}

	while (rpc_poll_is_suspend) {
		pr_warn("RPCpoll: someone access rpc poll during the suspend!!!...\n");
		msleep(1000);
	}

	wmb();
	down_write(&dev->ptrSync->writeSem);

#if 1
	/* Threads that share the same file descriptor should have the same tgid
	 * However, with uClibc pthread library, pthread_create() creates threads with pid == tgid
	 * So the tgid is not real tgid, we have to maintain the thread list that we can lookup later
	 */
	if (current->pid != proc->pid)
		update_thread_list(rextra, proc->pid);
#endif

	if (ring_empty(extra))
		size = 0;   // the ring is empty
	else if (dev->ringIn > dev->ringOut)
		size = dev->ringIn - dev->ringOut;
	else
		size = rpc_ring_size + dev->ringIn - dev->ringOut;

	pr_debug("%s: count:%lu space:%d\n", extra->name, count, rpc_ring_size - size - 1);
	pr_debug("%s: pid:%d tgid:%d\n", extra->name, current->pid, current->tgid);

	if (count > (rpc_ring_size - size - 1))
		goto out;

	temp = dev->ringEnd - dev->ringIn;

	if (temp >= count) {
#ifdef MY_COPY
		r = my_copy_from_user((int *)AVCPU2SCPU(dev->ringIn), (int *)buf, count);
#else
		r = copy_from_user((int *)AVCPU2SCPU(dev->ringIn), (int *)buf, count);
#endif
		if (r) {
			ret = -EFAULT;
			goto out;
		}

		ret += count;
		ptmp = dev->ringIn + ((count+3) & 0xfffffffc);

		//asm("DSB");
		mb();

		if (ptmp == dev->ringEnd)
			dev->ringIn = dev->ringStart;
		else
			dev->ringIn = ptmp;

		//pr_debug("RPC Write is in 1st kind...\n");
	} else {
#ifdef MY_COPY
		r = my_copy_from_user((int *)AVCPU2SCPU(dev->ringIn), (int *)buf, temp);
#else
		r = copy_from_user((int *)AVCPU2SCPU(dev->ringIn), (int *)buf, temp);
#endif
		if (r) {
			ret = -EFAULT;
			goto out;
		}

		count -= temp;

#ifdef MY_COPY
		r = my_copy_from_user((int *)AVCPU2SCPU(dev->ringStart), (int *)(buf+temp), count);
#else
		r = copy_from_user((int *)AVCPU2SCPU(dev->ringStart), (int *)(buf+temp), count);
#endif
		if (r) {
			ret = -EFAULT;
			goto out;
		}

		ret += (temp + count);

		//asm("DSB");
		mb();

		dev->ringIn = dev->ringStart+((count+3) & 0xfffffffc);
		//pr_debug("RPC Write is in 2nd kind...\n");
	}

	//peek_rpc_struct(extra->name, dev, f_pos);

out:
	pr_debug("RPC poll ringIn pointer is : %p\n", AVCPU2SCPU(dev->ringIn));
	up_write(&dev->ptrSync->writeSem);
	wmb();

	return ret;
}

long rpc_poll_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int found;
	RPC_PROCESS *proc = filp->private_data;
	RPC_DEV_EXTRA *extra = proc->extra;
	RPC_HANDLER *handler;

	while (rpc_poll_is_suspend) {
		pr_warn("RPCpoll: someone access rpc poll during the suspend!!!...\n");
		msleep(1000);
	}

	switch (cmd) {
#ifdef CONFIG_REALTEK_RPC_PROGRAM_REGISTER
	case RPC_IOCTHANDLER:

		pr_debug("%s:%d : Register handler for programID:%lu\n", __func__, __LINE__, arg);
		found = 0;
		list_for_each_entry(handler, &proc->handlers, list) {
			if (handler->programID == arg) {
				found = 1;
				break;
			}
		}

		if (found)
			break;

		/* not found, add to handler list */
		handler = kmalloc(sizeof(RPC_HANDLER), GFP_KERNEL);
		if (handler == NULL) {
			pr_err("%s: failed to allocate RPC_HANDLER\n", __func__);
			return -ENOMEM;
		}

		handler->programID = arg;
		spin_lock_bh(&extra->lock);
		list_add(&handler->list, &proc->handlers);
		spin_unlock_bh(&extra->lock);
		pr_debug("%s:%d %s: Add handler pid:%d for programID:%lu\n", __func__, __LINE__, proc->extra->name, proc->pid, arg);
		break;
#endif
	case RPC_IOC_PROCESS_CONFIG_0:
	{
		struct S_RPC_IOC_PROCESS_CONFIG_0 config;

		if (copy_from_user(&config, (void __user *)arg, sizeof(struct S_RPC_IOC_PROCESS_CONFIG_0))) {
			pr_err("ERROR! %s cmd:RPC_IOC_PROCESS_CONFIG_0 copy_from_user failed\n", __func__);
			return -ENOMEM;
		}

		if (proc == NULL) {
			pr_err("ERROR! %s cmd:RPC_IOC_PROCESS_CONFIG_0 proc:%p\n", __func__, proc);
			return -ENOMEM;
		}

		proc->bStayActive = (config.bStayActive > 0) ? true : false;
		break;
	}
	default:
		pr_warn("%s:%d unsupported ioctl cmd:%x arg:%lx\n", __func__, __LINE__, cmd, arg);
		return -ENOTTY;
	}

	return ret;
}

long rpc_ctrl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	RPC_DBG_FLAG dFlag;

	while (rpc_poll_is_suspend) {
		pr_warn("RPCpoll: someone access rpc poll during the suspend!!!...\n");
		msleep(1000);
	}

	switch (cmd) {
	case RPC_IOCTRESET:

		pr_info("[RPC]start reset...\n");
		rpc_poll_init();
		rpc_intr_init();
		rpc_kern_init();

		/* clear the inter-processor interrupts */

		writel_relaxed(1 << 1, rpc_int_base+RPC_SB2_INT);

		writel(RPC_INT_SA, rpc_int_base+RPC_SB2_INT);

		rpc_set_flag(RPC_AUDIO, 0xffffffff);

		pr_info("[RPC]done...\n");
		break;

	case RPC_IOCTRGETDBGREG_A:
	case RPC_IOCTRGETDBGREG_V:

		if (copy_from_user(&dFlag, (void __user *)arg, sizeof(dFlag)))
			return -EFAULT;
	else {
		unsigned int *puDebugFlag = NULL;
		struct rpc_debug_flag * debug_flag = get_debug_flag();
		phys_addr_t debug_flag_phyAddr = get_debug_flag_phyAddr();

		if (debug_flag == NULL || -1 == debug_flag_phyAddr)
			return -EFAULT;

		if (cmd == RPC_IOCTRGETDBGREG_V) {
			puDebugFlag = &debug_flag->vcpu;
			debug_flag_phyAddr = debug_flag_phyAddr + offsetof(struct rpc_debug_flag, vcpu);
		} else {
			puDebugFlag = &debug_flag->acpu;
			debug_flag_phyAddr = debug_flag_phyAddr + offsetof(struct rpc_debug_flag, acpu);
		}

		if (dFlag.op == RPC_DBGREG_SET)
			*puDebugFlag = dFlag.flagValue;
		else {
			dFlag.flagValue = (unsigned int)*puDebugFlag;
			dFlag.flagAddr = (uint32_t) debug_flag_phyAddr & -1U;
			if (copy_to_user((void __user *)arg, &dFlag, sizeof(dFlag)))
				return -EFAULT;
		}

		pr_debug("RPC_DEBUG cmd=%s op=%s phyAddr=0x%08llx flag=0x%08x",
			(cmd == RPC_IOCTRGETDBGREG_V) ? "RPC_IOCTRGETDBGREG_V" : "RPC_IOCTRGETDBGREG_A",
			(dFlag.op == RPC_DBGREG_SET) ? "SET" : "GET",
			debug_flag_phyAddr, *puDebugFlag);
	}

		break;
	case RPC_IOCTRGETDBGPRINT_V:

		if (copy_from_user(&dFlag, (void __user *)arg, sizeof(dFlag)))
			return -EFAULT;
        else {
            struct rpc_debug_print_memory * debug_print = get_debug_print();
            phys_addr_t debug_print_phyAddr = get_debug_print_phyAddr();
            struct dma_buf * dmabuf;

            if (debug_print == NULL || -1 == debug_print_phyAddr)
                return -EFAULT;

            dFlag.flagAddr = (uint32_t) debug_print_phyAddr & -1U;
//            dFlag.flagValue = dma_buf_fd(debug_print->dmabuf, debug_print->handle);
            debug_print->fd = dma_buf_fd(debug_print->dmabuf, O_CLOEXEC);
            dmabuf = dma_buf_get(debug_print->fd);
            if (IS_ERR(dmabuf)) {
                pr_err("%s : ERROR! dma_buf_get return %p", __func__, dmabuf);
                return -EFAULT;
            }

            debug_print->dmabuf = dmabuf;
//            dFlag.flagValue = dma_buf_fd(debug_print->dmabuf, O_CLOEXEC);
            dFlag.flagValue = debug_print->fd;
            
            if (copy_to_user((void __user *)arg, &dFlag, sizeof(dFlag)))
                return -EFAULT;

        }
        break;
	default:
		pr_warn("[RPC]: error ioctl command...\n");
		break;
	}

	return ret;
}

int rpc_ctrl_open(struct inode *inode, struct file *filp)
{
	pr_info("[RPC]open for RPC ioctl...\n");

	return 0;
}

struct file_operations rpc_poll_fops = {
	//llseek:scull_llseek,
	.unlocked_ioctl = rpc_poll_ioctl,
	.compat_ioctl = rpc_poll_ioctl,
	.read = rpc_poll_read,
	.write = rpc_poll_write,
	.open = rpc_poll_open,
	.release = rpc_poll_release,
};

struct file_operations rpc_ctrl_fops = {
	.unlocked_ioctl = rpc_ctrl_ioctl,
	.compat_ioctl = rpc_ctrl_ioctl,
	.open = rpc_ctrl_open,
};

MODULE_LICENSE("GPL v2");
