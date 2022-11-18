/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/ion.h>
#include <linux/miscdevice.h>
#include <uapi/linux/ion.h>
#include <soc/realtek/uapi/ion_rtk.h>

#include <ion_rtk_alloc.h>
#include "rpc_mem.h"
#include "rpc_mem_uapi.h"
#include "compat_rpc_mem.h"

static DEFINE_SPINLOCK(rpc_alloc_lock);
static r_program_entry_t *r_program_head = NULL;
static int r_program_count = 0;

void r_program_add(r_program_entry_t * entry)
{
	spin_lock_bh(&rpc_alloc_lock);
	entry->next = r_program_head;
	r_program_head = entry;
	r_program_count++;
	spin_unlock_bh(&rpc_alloc_lock);
}

r_program_entry_t *r_program_remove(unsigned long phys_addr)
{
	r_program_entry_t *prev = NULL;
	r_program_entry_t *curr = NULL;

	spin_lock_bh(&rpc_alloc_lock);
	curr = r_program_head;
	while (curr != NULL) {
		if (curr->phys_addr != phys_addr) {
			prev = curr;
			curr = curr->next;
			continue;
		}

		if (prev == NULL) {
			r_program_head = curr->next;
		} else {
			prev->next = curr->next;
		}
		r_program_count--;
		spin_unlock_bh(&rpc_alloc_lock);

		return curr;
	}
	spin_unlock_bh(&rpc_alloc_lock);
	return NULL;
}

int r_program_fd(unsigned long phys_addr, unsigned long *offset,
		 unsigned long *size)
{
	int ret_fd = -1;
	spin_lock_bh(&rpc_alloc_lock);
	do {
		r_program_entry_t *curr = r_program_head;
		while (curr != NULL) {
			if (phys_addr >= curr->phys_addr &&
			    phys_addr < (curr->phys_addr + curr->size)) {

				ret_fd =
				    dma_buf_fd(curr->rpc_dmabuf, O_CLOEXEC);

				if (ret_fd >= 0) {
					struct dma_buf *dmabuf =
					    dma_buf_get(ret_fd);
					if (IS_ERR(dmabuf))
						pr_err
						    ("%s : dma=%p (phy=0x%08lx) fd=%d\n",
						     __func__, dmabuf,
						     phys_addr, ret_fd);
					else
						curr->rpc_dmabuf = dmabuf;
				}

				if (offset)
					*offset = phys_addr - curr->phys_addr;

				if (size)
					*size = curr->size;

				break;
			} else {
				curr = curr->next;
			}
		}
	} while (0);
	spin_unlock_bh(&rpc_alloc_lock);
	return ret_fd;
}

static long rpc_mem_ioctl(struct file *filp, unsigned int cmd,
			  unsigned long arg)
{
	long ret = -ENOTTY;
	switch (cmd) {
	case RPC_MEM_IOC_EXPORT:
		{
			struct rpc_mem_fd_data data;
			if (copy_from_user
			    (&data, (void __user *)arg, sizeof(data))) {
				pr_err("%s: copy_from_user ERROR!\n", __func__);
				break;
			}

			data.ret_fd =
			    r_program_fd(data.phyAddr, &data.ret_offset,
					 &data.ret_size);

			if (data.ret_fd < 0) {
				pr_err("%s : ret_fd = %d\n", __func__,
				       data.ret_fd);
				break;
			}

			if (copy_to_user
			    ((void __user *)arg, &data, sizeof(data))) {
				ext_rtk_ion_close_fd(current->files,
						     data.ret_fd);
				pr_err
				    ("%s : copy_to_user failed! (phyAddr=0x%08lx)\n",
				     __func__, data.phyAddr);
				break;
			}
			ret = 0;
			break;
		}
	default:
		pr_err("%s: Unknown ioctl (cmd=0x%08x)\n", __func__, cmd);
		ret = -ENOTTY;
		break;
	}
	return ret;
}

static const struct file_operations rpc_mem_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = rpc_mem_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_rpc_mem_ioctl,
#endif
};

int rtk_rpc_mem_int(void)
{
	struct miscdevice *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->minor = MISC_DYNAMIC_MINOR;
	dev->name = "rpc_mem";
	dev->fops = &rpc_mem_fops;
	dev->parent = NULL;

	ret = misc_register(dev);
	if (ret) {
		pr_err("rpc_mem: failed to register misc device.\n");
		kfree(dev);
		return ret;
	}

	return 0;
}

MODULE_LICENSE("GPL v2");
