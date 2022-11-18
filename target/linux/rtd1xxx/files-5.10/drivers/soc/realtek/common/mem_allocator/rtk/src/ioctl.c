/* ion_rtk_ioctl.c
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/ion.h>
#include "../../../../staging/android/ion/ion_private.h"

#include <soc/realtek/uapi/ion_rtk.h>
#include "carveout_heap.h"
#include "flags.h"

#define ALIGNTO 32U
#define ION_ALIGN(len) (((len) + ALIGNTO - 1) & ~(ALIGNTO - 1))

static u64 dma_mask;

extern struct device *idev;
extern struct ion_device *ion_get_ion_device(void);

int ion_rtk_get_phys(int handle, phys_addr_t * addr, size_t * len)
{
	struct ion_buffer *buffer;
	struct dma_buf *dmabuf;
	struct sg_table *table;
	struct page *page;
	phys_addr_t paddr;
	int ret = 0;

	dmabuf = dma_buf_get(handle);
	if (IS_ERR(dmabuf)) {
		ret = -ENODEV;
		goto err;
	}

	buffer = dmabuf->priv;
	table = buffer->sg_table;
	page = sg_page(table->sgl);
	paddr = PFN_PHYS(page_to_pfn(page));
	*addr = paddr;
	*len = buffer->size;

err:
	dma_buf_put(dmabuf);
	return ret;
}
struct ion_heap *ion_get_client_heap_by_mask(struct ion_device *dev,
					     unsigned int heap_id_mask)
{
	struct ion_heap *heap = NULL;

	down_read(&dev->lock);
	plist_for_each_entry(heap, &dev->heaps, node) {
		/* if the caller didn't specify this heap id */
		if (!((1 << heap->id) & heap_id_mask))
			continue;
		break;
	}
	up_read(&dev->lock);

	return heap;
}

static int rtk_ion_sync_for_device(int fd, int cmd)
{
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer;
	enum dma_data_direction dir = (cmd == RTK_ION_IOC_INVALIDATE)
	    ? DMA_FROM_DEVICE : DMA_TO_DEVICE;

	switch (cmd) {
	case RTK_ION_IOC_INVALIDATE:
	case RTK_ION_IOC_FLUSH:
		break;
	default:
		return -EINVAL;
	}

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	buffer = dmabuf->priv;
	dma_mask = 0xffffffff;
	idev->dma_mask = &dma_mask;

	if (!ion_flag_is_protected(buffer->flags) &&
	    !ion_flag_is_noncached(buffer->flags)) {
		int nents = dma_map_sg(idev, buffer->sg_table->sgl,
				       buffer->sg_table->nents, dir);
		dma_sync_sg_for_device(idev, buffer->sg_table->sgl,
				       buffer->sg_table->nents, dir);
		if (nents > 0)
			dma_unmap_sg(idev, buffer->sg_table->sgl,
				     buffer->sg_table->nents, dir);
	}

	dma_buf_put(dmabuf);
	return 0;
}

static int rtk_ion_sync_range_for_device(int handle, int cmd, struct rtk_ion_ioc_sync_rane
					 *range_data)
{
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer;
	enum dma_data_direction dir =
	    (cmd ==
	     RTK_ION_IOC_INVALIDATE_RANGE) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	int fd;

	switch (cmd) {
	case RTK_ION_IOC_INVALIDATE_RANGE:
	case RTK_ION_IOC_FLUSH_RANGE:
		break;
	default:
		return -EINVAL;
	}

	fd = (int)range_data->handle & -1U;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	buffer = dmabuf->priv;
	dma_mask = 0xffffffff;
	idev->dma_mask = &dma_mask;

	if (!ion_flag_is_protected(buffer->flags) &&
	    !ion_flag_is_noncached(buffer->flags)) {
		phys_addr_t addr;
		size_t len;
		if (ion_rtk_get_phys(handle, &addr, &len) == 0) {
			size_t offset = range_data->phyAddr - addr;
			dma_addr_t paddr = dma_map_page(idev,
							sg_page(buffer->sg_table->sgl),
							offset,
							range_data->len, dir);
			dma_sync_single_for_device(idev, range_data->phyAddr,
						   range_data->len, dir);
			dma_unmap_page(idev, paddr, range_data->len, dir);
		}
	}

	dma_buf_put(dmabuf);
	return 0;
}

static int rtk_ion_get_memory_info(int handle, unsigned int heapMask,
				   unsigned int flags,
				   struct ion_rtk_carveout_meminfo *info)
{
	int i;
	int ret;
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer;
	struct ion_heap *heap;
	struct ion_rtk_carveout_meminfo tmp_info;
	struct ion_rtk_carveout_ops *ops;
	struct ion_device *ion_dev = ion_get_ion_device();

	if (info == NULL)
		goto err;

	dmabuf = dma_buf_get(handle);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto err;
	}

	buffer = dmabuf->priv;

	info->usedSize = 0;
	info->freeSize = 0;

	for (i = 0; i < 32; i++) {
		struct ion_heap *heap;
		struct ion_rtk_carveout_ops *ops;
		struct ion_rtk_carveout_meminfo tmp_info;
		unsigned int target_heap_id_mask = 0x1U << i;

		if ((target_heap_id_mask & heapMask) == 0)
			continue;

		heap = ion_get_client_heap_by_mask(ion_dev, target_heap_id_mask);
		if (heap == NULL)
			continue;

		ops = get_rtk_carveout_ops(heap);
		if (ops == NULL)
			continue;

		if (ops->getMemInfo(heap, flags, &tmp_info) == 0) {
			info->usedSize += tmp_info.usedSize;
			info->freeSize += tmp_info.freeSize;
		}
	}

	dma_buf_put(dmabuf);

	return 0;
err:
	return -1;
}

static int rtk_ion_get_phy_info(struct rtk_ion_ioc_phy_info *phyInfo)
{
	int ret = 0;
	phys_addr_t addr;
	size_t len;

	if (phyInfo->handle < 0) {
		ret = -EINVAL;
		goto err;
	}

	ret = ion_rtk_get_phys(phyInfo->handle, &addr, &len);

	phyInfo->addr = addr;
	phyInfo->len = len;

err:
	return ret;
}

long ion_rtk_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = -1;
	switch (cmd) {
	case RTK_ION_GET_LAST_ALLOC_ADDR:{
			pr_err
			    ("%s: Outdated ioctl : RTK_PHOENIX_ION_GET_LAST_ALLOC_ADDR\n",
			     __func__);
			ret = -EFAULT;
			break;
		}
	case RTK_ION_IOC_INVALIDATE:
	case RTK_ION_IOC_FLUSH:{
			int fd = -1;
			ret =
			    copy_from_user(&fd, (void __user *)arg, sizeof(fd));
			if (rtk_ion_sync_for_device(fd, cmd) != 0) {
				pr_err
				    ("%s: rtk_ion_sync_for_device failed! (cmd:%x fd:%d)\n",
				     __func__, cmd, fd);
				ret = -EFAULT;
			}

			break;
		}
	case RTK_ION_IOC_FLUSH_RANGE:
	case RTK_ION_IOC_INVALIDATE_RANGE:{

			struct rtk_ion_ioc_sync_rane range_data;
			ret =
			    copy_from_user(&range_data, (void __user *)arg,
					   sizeof(range_data));
			if (ret) {
				pr_err
				    ("%s:%d copy_from_user failed! (ret = %d)\n",
				     __func__, __LINE__, ret);
				ret = -EFAULT;
			}

			if (rtk_ion_sync_range_for_device
			    (range_data.handle, cmd, &range_data) != 0) {
				pr_err
				    ("%s: rtk_ion_sync_range_for_device failed! (cmd:%d handle:%d)\n",
				     __func__, cmd, (int)range_data.handle);
				ret = -EFAULT;
			}

			break;
		}

	case RTK_ION_IOC_GET_PHYINFO:{
			struct rtk_ion_ioc_phy_info phyInfo;
			ret =
			    copy_from_user(&phyInfo, (void __user *)arg,
					   sizeof(phyInfo));
			if (ret) {
				pr_err
				    ("%s:%d copy_from_user failed! (ret = %d)\n",
				     __func__, __LINE__, ret);
				ret = -EFAULT;
				break;
			}

			ret = rtk_ion_get_phy_info(&phyInfo);
			if (ret != 0) {
				pr_err
				    ("%s:%d rtk_ion_get_phy_info failed! (ret = %d)\n",
				     __func__, __LINE__, ret);
				ret = -EFAULT;
				break;
			}

			ret =
			    copy_to_user((void __user *)arg, &phyInfo,
					 sizeof(phyInfo));
			if (ret) {
				pr_err
				    ("%s:%d copy_to_user failed! (ret = %d)\n",
				     __func__, __LINE__, ret);
				return -EFAULT;
			}

			break;
		}
	case RTK_ION_IOC_GET_MEMORY_INFO:{
			struct rtk_ion_ioc_get_memory_info_s user_info;
			struct ion_rtk_carveout_meminfo ion_info;

			ret =
			    copy_from_user(&user_info, (void __user *)arg,
					   sizeof(user_info));
			if (ret) {
				pr_err
				    ("%s:%d copy_from_user failed! (ret = %d)\n",
				     __func__, __LINE__, ret);
				ret = -EFAULT;
				break;
			}

			ret =
			    rtk_ion_get_memory_info(user_info.handle,
						    user_info.heapMask,
						    user_info.flags, &ion_info);
			if (ret) {
				pr_err
				    ("%s:%d rtk_ion_get_memory_info failed! (ret = %d)\n",
				     __func__, __LINE__, ret);
				ret = -EFAULT;
				break;
			}

			user_info.usedSize =
			    (unsigned int)ion_info.usedSize & -1U;
			user_info.freeSize =
			    (unsigned int)ion_info.freeSize & -1U;

			ret =
			    copy_to_user((void __user *)arg, &user_info,
					 sizeof(user_info));
			if (ret) {
				pr_err
				    ("%s:%d copy_to_user failed! (ret = %d)\n",
				     __func__, __LINE__, ret);
				ret = -EFAULT;
			}

			break;
		}

	default:
		pr_err("%s: Unknown custom ioctl\n", __func__);
		ret = -ENOTTY;
		break;
	}
	return ret;
}
