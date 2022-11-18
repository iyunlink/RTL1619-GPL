/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/list.h>
#include <linux/pm_runtime.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include "tpdemux_common.h"
#include "tpdemux_core.h"
#include "tpdemux_reg.h"

#define MM_BUFFER_SIZE (188*400)
#define TP_BUFFER_ALIGNMENT (376)

#define FLUSH_USER_MTP

/**
 * tp_ring_params - Describe a ring buffer entry
 *
 * @limit: top address of the ring buffer
 * @base: bottom address of the ring buffer
 * @rp: read pointer for the ring buffer
 * @wp: write pointer for the ring buffer
 */
struct tp_ring_params {
	unsigned long limit;
	unsigned long base;
	unsigned long rp;
	unsigned long wp;
};

/**
 * _tp_mmbuf_get_regs() - Get TP_M2M_RING registers
 */
void _tp_mmbuf_get_regs(struct rtk_tp_reg *regs, struct tp_ring_params *pparam)
{
	pparam->limit = read_reg32(regs->reg_TP_M2M_RING_LIMIT);
	pparam->base = read_reg32(regs->reg_TP_M2M_RING_BASE);
	pparam->rp = read_reg32(regs->reg_TP_M2M_RING_RP);
	pparam->wp = read_reg32(regs->reg_TP_M2M_RING_WP);
}

/**
 * _tp_mmbuf_dump_status() - Dump status of M2M ring buffer
 */
void _tp_mmbuf_dump_status(struct rtk_tp_reg *regs)
{
	pr_debug("MMBUF : BASE=%08x, LIMIT=%08x, RP=%08x, WP=%08x\n",
	       read_reg32(regs->reg_TP_M2M_RING_BASE),
	       read_reg32(regs->reg_TP_M2M_RING_LIMIT),
	       read_reg32(regs->reg_TP_M2M_RING_RP),
	       read_reg32(regs->reg_TP_M2M_RING_WP));
}

/**
 * _tp_mmbuf_reset_buffer() - Reset M2M ring buffer
 */
static int _tp_mmbuf_reset_buffer(struct rtk_tp_reg *regs)
{
	write_reg32(regs->reg_TP_M2M_RING_CTRL,
		    TP_M2M_RING_CTRL_GO_BIT | TP_M2M_RING_CTRL_WRITE_DATA(0));

	write_reg32(regs->reg_TP_M2M_RING_LIMIT, 0);
	write_reg32(regs->reg_TP_M2M_RING_BASE, 0);
	write_reg32(regs->reg_TP_M2M_RING_RP, 0);
	write_reg32(regs->reg_TP_M2M_RING_WP, 0);
	write_reg32(regs->reg_TP_M2M_RING_CTRL, 0);

	return 0;
}

/**
 * _tp_mmbuf_set_boundary() - Set boundrary of M2M ring buffer
 */
static int _tp_mmbuf_set_boundary(struct rtk_tp_reg *regs, unsigned long base,
				  unsigned long length)
{
	write_reg32(regs->reg_TP_M2M_RING_LIMIT, base + length);
	write_reg32(regs->reg_TP_M2M_RING_BASE, base);
	write_reg32(regs->reg_TP_M2M_RING_RP, base);
	write_reg32(regs->reg_TP_M2M_RING_WP, base);

	return 0;
}

/**
 * rtk_tp_set_mmbuffer() - Set M2M ring buffer
 */
int rtk_tp_set_mmbuffer(struct demux_info *dmx)
{
	struct rtk_tp_reg *regs = &dmx->regs;
#if 0
	struct rtktpfei *fei = dmx->fei;
	dmx->mmbuf_hdl = ion_alloc(fei->tp_buf_ion_client,
				   MM_BUFFER_SIZE, 4096,
				   RTK_PHOENIX_ION_HEAP_MEDIA_MASK,
				   ION_FLAG_NONCACHED | ION_FLAG_SCPUACC |
				   ION_FLAG_ACPUACC);

	if (ion_phys(fei->tp_buf_ion_client, dmx->mmbuf_hdl,
		     &dmx->mmbuf_phy_addr, &dmx->mmbuf_len) != 0)
		pr_err("alloc M2M buffer fail\n");

	dmx->mmbuf_virt_addr = ion_map_kernel(fei->tp_buf_ion_client,
					      dmx->mmbuf_hdl);

	/* data ring buffer should be aligned */
	dmx->mmbuf_len =
	    (dmx->mmbuf_len / TP_BUFFER_ALIGNMENT) * TP_BUFFER_ALIGNMENT;

#endif
	_tp_mmbuf_reset_buffer(regs);
	_tp_mmbuf_set_boundary(regs, dmx->mmbuf_phy_addr, dmx->mmbuf_len);

	return 0;
}

#ifdef FLUSH_USER_MTP
extern struct device *idev;
extern phys_addr_t rtk_tp_ion_pa(struct ion_buffer *buffer);
#define TP_FLUSH_ION_RANGE(b, p, l) rtk_tp_ion_sync_range(b, p, l)
static int rtk_tp_ion_sync_range(struct dma_buf *dmabuf,
				 unsigned long phys, size_t len)
{
	u64 dma_mask = 0xffffffff;
	struct ion_buffer *buffer;
	if (IS_ERR_OR_NULL(dmabuf))
		return PTR_ERR(dmabuf);

	buffer = dmabuf->priv;
	idev->dma_mask = &dma_mask;
	if (!(buffer->flags & ION_FLAG_NONCACHED)) {
		phys_addr_t addr;
		if ((addr = rtk_tp_ion_pa(buffer)) != 0) {
			size_t offset = phys - addr;
			dma_addr_t paddr = dma_map_page(idev,
						sg_page(buffer->sg_table->sgl),
						offset,
						len, DMA_TO_DEVICE);
			dma_sync_single_for_device(idev, phys, len, DMA_TO_DEVICE);
			if (paddr)
				dma_unmap_page(idev, paddr, len, DMA_TO_DEVICE);
		}
	}

	return 0;
}

#else // ifdef FLUSH_USER_MTP

// empty function
#define TP_FLUSH_ION_RANGE(b, p, l) (void)(p)

#endif // ifdef FLUSH_USER_MTP

/**
 * _tp_mmbuf_get_free() - Get the number of free bytes in M2M ring buffer
 */
static ssize_t _tp_mmbuf_get_free(struct rtk_tp_reg *regs)
{
	struct tp_ring_params param;
	ssize_t free;

	_tp_mmbuf_get_regs(regs, &param);

	free = param.rp - param.wp;
	if (free <= 0)
		free += (param.limit - param.base);

	return free - 1;
}

/**
 * _tp_mmbuf_update() - Move write pointer of M2M ring buffer
 */
static int _tp_mmbuf_update(struct rtk_tp_reg *regs, unsigned long wp)
{
	write_reg32(regs->reg_TP_M2M_RING_WP, wp);
	write_reg32(regs->reg_TP_M2M_RING_CTRL,
		    TP_M2M_RING_CTRL_GO_BIT | TP_M2M_RING_CTRL_WRITE_DATA(1));

	return 0;
}

/**
 * _tp_mmbuf_write() - Copy data from user space to M2M ring buffer
 */
static ssize_t _tp_mmbuf_write(struct demux_info *dmx, const void __user *buf,
			       size_t len)
{
	struct rtk_tp_reg *regs = &dmx->regs;
	struct tp_ring_params param;
	unsigned long offset;
	size_t todo = len;
	size_t split;
	int status;
	unsigned long phys = dmx->mmbuf_phy_addr;

	_tp_mmbuf_get_regs(regs, &param);

	split = (param.wp + len > param.limit) ? param.limit - param.wp : 0;
	if (split > 0) {
		offset = param.wp - param.base;
		status =
		    copy_from_user(dmx->mmbuf_virt_addr + offset, buf, split);
		if (status)
			return len - todo;

		TP_FLUSH_ION_RANGE(dmx->mmbuf_dmabuf, phys + offset, split);

		buf += split;
		todo -= split;

		param.wp = param.base;
		_tp_mmbuf_update(regs, param.wp);
	}

	offset = param.wp - param.base;
	status = copy_from_user(dmx->mmbuf_virt_addr + offset, buf, todo);
	if (status)
		return len - todo;

	TP_FLUSH_ION_RANGE(dmx->mmbuf_dmabuf, phys + offset, todo);

	param.wp += todo;
	if (param.wp >= param.limit)
		param.wp = param.base;
	_tp_mmbuf_update(regs, param.wp);

	return len;
}

/**
 * rtk_tp_write_mmbuffer() - Write user space data to M2M ring buffer
 */
ssize_t rtk_tp_write_mmbuffer(struct demux_info *dmx, const void __user *buf,
			      size_t len)
{
	struct rtk_tp_reg *regs = &dmx->regs;
	ssize_t free;

	free = _tp_mmbuf_get_free(regs);
	if (len > free)
		len = free;

	return _tp_mmbuf_write(dmx, buf, len);
}
