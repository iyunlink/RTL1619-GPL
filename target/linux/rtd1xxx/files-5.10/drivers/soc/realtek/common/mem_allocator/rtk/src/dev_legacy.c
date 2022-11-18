/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/file.h>

#include "ion_old/ion.h"
#include "uapi_old/ion_rtk.h"
#include "carveout_heap.h"
#include "flags.h"
#include "dev_legacy.h"
#include "../inc/ion_rtk_alloc.h"

static u64 dma_mask;

static bool is_dma_buf_fd(int fd)
{
	struct dma_buf *dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return false;

	dma_buf_put(dmabuf);
	return true;
}

static int dma_buf_fd_dup(int fd)
{
	int ret = -1;
	do {
		struct dma_buf *dmabuf = dma_buf_get(fd);
		if (IS_ERR(dmabuf)) {
			pr_err("%s dma_buf_get error! (fd:%d)\n", __func__, fd);
			break;
		}

		ret = dma_buf_fd(dmabuf, O_CLOEXEC);
		if (ret < 0) {
			dma_buf_put(dmabuf);
			pr_err
			    ("%s dma_buf_fd error! (fd:%d dmabuf=%p)\n",
			     __func__, fd, dmabuf);
			break;
		}

		break;
	} while (0);
	return ret;
}

static int legacy_ion_get_phys(int handle, phys_addr_t * addr, size_t * len)
{
	struct ion_buffer *buffer;
	struct dma_buf *dmabuf;
	struct sg_table *table;
	struct page *page;
	phys_addr_t paddr;

	dmabuf = dma_buf_get(handle);
	if (IS_ERR(dmabuf))
		return -ENODEV;

	buffer = dmabuf->priv;
	table = buffer->sg_table;
	page = sg_page(table->sgl);
	paddr = PFN_PHYS(page_to_pfn(page));
	*addr = paddr;
	*len = buffer->size;

	dma_buf_put(dmabuf);
	return 0;
}

static int legacy_ion_sync(int fd, enum dma_data_direction dir)
{
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	buffer = dmabuf->priv;
	if (!ion_flag_is_protected(buffer->flags)) {
		dma_mask = 0xffffffff;
		buffer->dev->dev.this_device->dma_mask = &dma_mask;
		int nents = dma_map_sg(buffer->dev->dev.this_device, buffer->sg_table->sgl,
				       buffer->sg_table->nents, dir);
		dma_sync_sg_for_device(buffer->dev->dev.this_device, buffer->sg_table->sgl,
				       buffer->sg_table->nents, dir);
		if (nents > 0)
			dma_unmap_sg(buffer->dev->dev.this_device, buffer->sg_table->sgl,
				     buffer->sg_table->nents, dir);
	}

	dma_buf_put(dmabuf);
	return 0;
}

static int legacy_ion_sync_range(int fd, enum dma_data_direction dir,
				 unsigned long phyAddr, size_t len)
{
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	buffer = dmabuf->priv;
	dma_mask = 0xffffffff;
	buffer->dev->dev.this_device->dma_mask = &dma_mask;
	if (!ion_flag_is_protected(buffer->flags)) {
		phys_addr_t addr;
		size_t len;
		if (legacy_ion_get_phys(fd, &addr, &len) == 0) {
			size_t offset = phyAddr - addr;
			dma_addr_t paddr = dma_map_single(buffer->dev->dev.this_device,
							  sg_virt
							  (buffer->sg_table->
							   sgl)
							  + offset,
							  len, dir);
			dma_sync_single_for_device(buffer->dev->dev.this_device, phyAddr, len, dir);
			if (paddr)
				dma_unmap_single(buffer->dev->dev.this_device, paddr, len, dir);
		}
	}

	dma_buf_put(dmabuf);
	return 0;
}

static long legacy_ion_rtk_custom_ioctl(struct file *filp, unsigned int cmd,
					unsigned long arg)
{
	long ret = -ENOTTY;
	switch (cmd) {
	case LEGACY_RTK_ION_IOC_INVALIDATE:
		{
			int fd = arg;
			if (legacy_ion_sync(fd, DMA_FROM_DEVICE)) {
				pr_err("%s: legacy_ion_sync failed!(fd=%d)\n",
				       __func__, fd);
				break;
			}
			ret = 0;
			break;
		}
	case LEGACY_RTK_ION_IOC_FLUSH:
		{
			int fd = arg;
			if (legacy_ion_sync(fd, DMA_TO_DEVICE)) {
				pr_err("%s: legacy_ion_sync failed!(fd=%d)\n",
				       __func__, fd);
				break;
			}
			ret = 0;
			break;
		}
	case LEGACY_RTK_ION_IOC_INVALIDATE_RANGE:
		{
			struct legacy_rtk_ion_ioc_sync_rane data;
			if (copy_from_user
			    (&data, (void __user *)arg, sizeof(data))) {
				pr_err("%s: copy_from_user ERROR!\n", __func__);
				break;
			}
			if (legacy_ion_sync_range
			    (data.handle, DMA_FROM_DEVICE, data.phyAddr,
			     data.len)) {
				pr_err
				    ("%s: legacy_ion_sync_range failed!(fd=%d)\n",
				     __func__, data.handle);
				break;
			}
			ret = 0;
			break;
		}
	case LEGACY_RTK_ION_IOC_FLUSH_RANGE:
		{
			struct legacy_rtk_ion_ioc_sync_rane data;
			if (copy_from_user
			    (&data, (void __user *)arg, sizeof(data))) {
				pr_err("%s: copy_from_user ERROR!\n", __func__);
				break;
			}
			if (legacy_ion_sync_range
			    (data.handle, DMA_TO_DEVICE, data.phyAddr,
			     data.len)) {
				pr_err
				    ("%s: legacy_ion_sync_range failed!(fd=%d)\n",
				     __func__, data.handle);
				break;
			}
			ret = 0;
			break;
		}
	case LEGACY_RTK_ION_IOC_GET_PHYINFO:
		{
			struct legacy_rtk_ion_ioc_phy_info data;
			phys_addr_t addr;
			size_t len;
			if (copy_from_user
			    (&data, (void __user *)arg, sizeof(data))) {
				pr_err("%s: copy_from_user ERROR!\n", __func__);
				break;
			}

			if (data.handle < 0) {
				pr_err("%s : invalid handle=%d\n", __func__,
				       data.handle);
				break;
			}

			if (legacy_ion_get_phys(data.handle, &addr, &len)) {
				pr_err
				    ("%s: legacy_ion_get_phys failed!(fd=%d)\n",
				     __func__, data.handle);
				break;
			}

			data.addr = addr;
			data.len = len;

			if (copy_to_user
			    ((void __user *)arg, &data, sizeof(data))) {
				pr_err
				    ("%s : copy_to_user failed! (fd=%d, phyAddr=0x%08x, len=%zu)\n",
				     __func__, data.handle, addr, len);
				break;
			}

			ret = 0;
			break;
		}
	default:
		pr_err("%s: Unknown custom ioctl (cmd=0x%08x)\n", __func__,
		       cmd);
		break;
	}
	return ret;
}

static long legacy_ion_rtk_ioctl(struct file *filp, unsigned int cmd,
				 unsigned long arg)
{
	long ret = -ENOTTY;
	switch (cmd) {
	case LEGACY_ION_IOC_ALLOC:
		{
			struct legacy_ion_allocation_data data;
			if (copy_from_user
			    (&data, (void __user *)arg, sizeof(data))) {
				pr_err("%s: copy_from_user ERROR!\n", __func__);
				break;
			}

			data.handle =
			    ext_rtk_ion_alloc(data.len, data.heap_id_mask,
					      data.flags);
			if (data.handle < 0) {
				pr_err
				    ("%s : alloc failed! (len:%zu heaps=0x%x flags=0x%x)\n",
				     __func__, data.len, data.heap_id_mask,
				     data.flags);
				break;
			}

			if (copy_to_user
			    ((void __user *)arg, &data, sizeof(data))) {
				ext_rtk_ion_close_fd(current->files,
						     data.handle);
				pr_err
				    ("%s : copy_to_user failed! (len:%zu heaps=0x%x flags=0x%x)\n",
				     __func__, data.len, data.heap_id_mask,
				     data.flags);
				break;
			}
			ret = 0;
			break;
		}
	case LEGACY_ION_IOC_FREE:
		{
			struct legacy_ion_handle_data data;
			if (copy_from_user
			    (&data, (void __user *)arg, sizeof(data))) {
				pr_err("%s: copy_from_user ERROR!\n", __func__);
				break;
			}

			if (data.handle < 0 || !is_dma_buf_fd(data.handle)) {
				pr_err
				    ("%s : (cmd=0x%08x) invalid handle=%d\n",
				     __func__, cmd, data.handle);
				break;
			}

			ext_rtk_ion_close_fd(current->files, data.handle);
			ret = 0;
			break;
		}
	case LEGACY_ION_IOC_SHARE:
	case LEGACY_ION_IOC_MAP:
		{
			struct legacy_ion_fd_data data;
			if (copy_from_user
			    (&data, (void __user *)arg, sizeof(data))) {
				pr_err("%s: copy_from_user ERROR!\n", __func__);
				break;
			}

			if (data.handle < 0 || !is_dma_buf_fd(data.handle)) {
				pr_err
				    ("%s : (cmd=0x%08x) invalid handle=%d\n",
				     __func__, cmd, data.handle);
				break;
			}

			data.fd = dma_buf_fd_dup(data.handle);
			if (data.fd < 0) {
				pr_err
				    ("%s (cmd=0x%08x) dma_buf_fd_dup error! (fd:%d)\n",
				     __func__, cmd, data.handle);
				break;
			}

			if (copy_to_user
			    ((void __user *)arg, &data, sizeof(data))) {
				ext_rtk_ion_close_fd(current->files, data.fd);
				pr_err
				    ("%s : copy_to_user failed! (handle=%d)\n",
				     __func__, data.handle);
				break;
			}
			ret = 0;
			break;
		}
	case LEGACY_ION_IOC_IMPORT:
		{
			struct legacy_ion_fd_data data;
			if (copy_from_user
			    (&data, (void __user *)arg, sizeof(data))) {
				pr_err("%s: copy_from_user ERROR!\n", __func__);
				break;
			}

			if (data.fd < 0 || !is_dma_buf_fd(data.fd)) {
				pr_err
				    ("%s : (cmd=0x%08x) invalid fd=%d\n",
				     __func__, cmd, data.fd);
				break;
			}

			data.handle = dma_buf_fd_dup(data.fd);
			if (data.handle < 0) {
				pr_err
				    ("%s (cmd=0x%08x) dma_buf_fd_dup error! (fd:%d)\n",
				     __func__, cmd, data.fd);
				break;
			}

			if (copy_to_user
			    ((void __user *)arg, &data, sizeof(data))) {
				ext_rtk_ion_close_fd(current->files,
						     data.handle);
				pr_err
				    ("%s : copy_to_user failed! (handle=%d)\n",
				     __func__, data.handle);
				break;
			}
			ret = 0;
			break;
		}
	case LEGACY_ION_IOC_SYNC:
		{
			struct legacy_ion_fd_data data;
			if (copy_from_user
			    (&data, (void __user *)arg, sizeof(data))) {
				pr_err("%s: copy_from_user ERROR!\n", __func__);
				break;
			}

			if (data.fd < 0 || !is_dma_buf_fd(data.fd)) {
				pr_err
				    ("%s : (cmd=0x%08x) invalid fd=%d\n",
				     __func__, cmd, data.fd);
				break;
			}

			if (legacy_ion_sync(data.fd, DMA_BIDIRECTIONAL)) {
				pr_err("%s : error sync (fd=%d)\n", __func__,
				       data.fd);
				break;
			}
			ret = 0;
			break;
		}
	case LEGACY_ION_IOC_CUSTOM:
		{
			struct legacy_ion_custom_data data;
			if (copy_from_user
			    (&data, (void __user *)arg, sizeof(data))) {
				pr_err("%s: copy_from_user ERROR!\n", __func__);
				break;
			}
			ret =
			    legacy_ion_rtk_custom_ioctl(filp, data.cmd,
							data.arg);
			break;
		}
	case LEGACY_ION_IOC_PHYS:
		{
			struct legacy_ion_phys_data data;
			if (copy_from_user
			    (&data, (void __user *)arg, sizeof(data))) {
				pr_err("%s: copy_from_user ERROR!\n", __func__);
				break;
			}

			if (data.handle < 0 || !is_dma_buf_fd(data.handle)) {
				pr_err
				    ("%s : (cmd=0x%08x) invalid fd=%d\n",
				     __func__, cmd, data.handle);
				break;
			}

			{
				phys_addr_t addr;
				size_t len;
				if (legacy_ion_get_phys
				    (data.handle, &addr, &len)) {
					pr_err
					    ("%s: (cmd=0x%08x) legacy_ion_get_phys failed!(fd=%d)\n",
					     __func__, cmd, data.handle);
					break;
				}
				data.addr = addr;
				data.len = len;
			}

			if (copy_to_user
			    ((void __user *)arg, &data, sizeof(data))) {
				pr_err
				    ("%s : (cmd=0x%08x) copy_to_user failed! (handle=%d)\n",
				     __func__, cmd, data.handle);
				break;
			}

			ret = 0;
			break;
		}
	default:
		pr_err("%s: Unknown custom ioctl (cmd=0x%08x)\n", __func__,
		       cmd);
		ret = -ENOTTY;
		break;
	}
	return ret;
}

static const struct file_operations legacy_ion_rtk_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = legacy_ion_rtk_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = legacy_ion_rtk_ioctl,
#endif
};

static int legacy_ion_device_create(void)
{
	struct miscdevice *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->minor = MISC_DYNAMIC_MINOR;
	dev->name = "ion_legacy";
	dev->fops = &legacy_ion_rtk_fops;
	dev->parent = NULL;

	ret = misc_register(dev);
	if (ret) {
		pr_err("ion_legacy: failed to register misc device.\n");
		kfree(dev);
		return ret;
	}

	return 0;
}

fs_initcall(legacy_ion_device_create);
