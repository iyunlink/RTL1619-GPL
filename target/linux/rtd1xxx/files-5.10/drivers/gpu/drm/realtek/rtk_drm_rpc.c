// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Realtek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/io.h>
//#include <asm/io.h>
#include "rtk_drm_rpc.h"

#ifdef USE_ION

unsigned int rtk_ion_flags(unsigned int dumb_flags)
{
	unsigned int ret = 0;

	if (dumb_flags & BUFFER_NONCACHED)
		ret |= ION_FLAG_NONCACHED;
	if (dumb_flags & BUFFER_SCPUACC)
		ret |= ION_FLAG_SCPUACC;
	if (dumb_flags & BUFFER_ACPUACC)
		ret |= ION_FLAG_ACPUACC;
	if (dumb_flags & BUFFER_HWIPACC)
		ret |= ION_FLAG_HWIPACC;
	if (dumb_flags & BUFFER_VE_SPEC)
		ret |= ION_FLAG_VE_SPEC;
	if (dumb_flags & BUFFER_ALGO_LAST_FIT)
		ret |= ION_USAGE_ALGO_LAST_FIT;
	if (dumb_flags & BUFFER_PROTECTED)
		ret |= ION_USAGE_PROTECTED;
	if (dumb_flags & BUFFER_SECURE_AUDIO)
		ret |= ION_FLAG_SECURE_AUDIO;

	if (ret == 0)
		ret |= ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_HWIPACC;

	return ret;
}

unsigned int rtk_ion_heaps(unsigned int dumb_flags)
{
	unsigned int ret = 0;

	if (dumb_flags & BUFFER_HEAP_MEDIA)
		ret |= RTK_ION_HEAP_MEDIA_MASK;
	if (dumb_flags & BUFFER_HEAP_AUDIO)
		ret |= RTK_ION_HEAP_AUDIO_MASK;
//	if (dumb_flags & BUFFER_HEAP_SYSTEM)
//		ret |= ION_HEAP_SYSTEM_MASK;
//	if (dumb_flags & BUFFER_HEAP_DMA)
//		ret |= ION_HEAP_TYPE_DMA_MASK;
	if (dumb_flags & BUFFER_HEAP_SECURE)
		ret |= RTK_ION_HEAP_SECURE_MASK;

	if (ret == 0)
		ret |= RTK_ION_HEAP_MEDIA_MASK;

	return ret;
}

phys_addr_t rtk_ion_mem_phy(struct ion_buffer *buffer)
{
	struct sg_table *table;
	struct page *page;
	phys_addr_t paddr;

	mutex_lock(&buffer->lock);
	table = buffer->sg_table;
	page = sg_page(table->sgl);
	if (!page)
		paddr = 0;
	else
		paddr = PFN_PHYS(page_to_pfn(page));
	mutex_unlock(&buffer->lock);

	return paddr;
}

void *rtk_ion_kernel_map(struct ion_buffer *buffer)
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

void rtk_rpc_free_ion(struct dma_buf *dmabuf)
{
	dma_buf_end_cpu_access(dmabuf, DMA_BIDIRECTIONAL);

	dma_buf_put(dmabuf);
}
#endif

unsigned int ipcReadULONG(u8 *src)
{
	return __be32_to_cpu(readl(src));
}

void ipcCopyMemory(void *p_des, void *p_src, unsigned long len)
{
	unsigned char *des = (unsigned char *)p_des;
	unsigned char *src = (unsigned char *)p_src;
	int i;

	for (i = 0; i < len; i += 4)
		writel(__cpu_to_be32(readl(&src[i])), &des[i]);
}

int rpc_create_video_agent(struct rtk_rpc_info *rpc_info, u32 *videoId, u32 pinId)
{
	struct rpc_create_video_agent *rpc = NULL;
	unsigned int offset;
	int ret = -1;

	rpc = (struct rpc_create_video_agent *)rpc_info->vaddr;
	offset = get_rpc_alignment_offset(sizeof(rpc->instance));

	memset_io(rpc, 0, sizeof(*rpc));

	rpc->instance = htonl(pinId);
	if (send_rpc_command(RPC_AUDIO, ENUM_VIDEO_KERNEL_RPC_CREATE,
			     rpc_info->paddr, rpc_info->paddr + offset,
			     &rpc->ret))
		goto exit;

	if (ntohl(rpc->result) != S_OK || rpc->ret != S_OK)
		goto exit;

	*videoId = ntohl(rpc->data);
	ret = 0;
exit:
	return ret;
}

int rpc_video_display(struct rtk_rpc_info *rpc_info,
		      struct rpc_vo_filter_display *argp)
{
	struct rpc_vo_filter_display_t *rpc = NULL;
	unsigned int offset;
	int ret = -1;

	rpc = (struct rpc_vo_filter_display_t *)rpc_info->vaddr;
	offset = get_rpc_alignment_offset(sizeof(*argp));

	memset_io(rpc, 0, sizeof(*rpc));

	rpc->instance = htonl(argp->instance);
	rpc->videoPlane = htonl(argp->videoPlane);
	rpc->zeroBuffer = argp->zeroBuffer;
	rpc->realTimeSrc = argp->realTimeSrc;

	if (send_rpc_command(RPC_AUDIO, ENUM_VIDEO_KERNEL_RPC_DISPLAY,
			     rpc_info->paddr, rpc_info->paddr + offset,
			     &rpc->ret))
		goto exit;

	if (ntohl(rpc->result) != S_OK || rpc->ret != S_OK)
		goto exit;
	ret = 0;
exit:
	return ret;
}

int rpc_video_config_disp_win(struct rtk_rpc_info *rpc_info,
			      struct rpc_config_disp_win *argp)
{
	struct rpc_config_disp_win_t *rpc = NULL;
	unsigned int offset;
	int ret = -1;

	rpc = (struct rpc_config_disp_win_t *)rpc_info->vaddr;
	offset = get_rpc_alignment_offset(sizeof(*argp));

	memset_io(rpc, 0, sizeof(*rpc));

	rpc->videoPlane = htonl(argp->videoPlane);
	rpc->videoWin.x = htons(argp->videoWin.x);
	rpc->videoWin.y = htons(argp->videoWin.y);
	rpc->videoWin.width = htons(argp->videoWin.width);
	rpc->videoWin.height = htons(argp->videoWin.height);
	rpc->borderWin.x = htons(argp->borderWin.x);
	rpc->borderWin.y = htons(argp->borderWin.y);
	rpc->borderWin.width = htons(argp->borderWin.width);
	rpc->borderWin.height = htons(argp->borderWin.height);
	rpc->borderColor.c1 = htons(argp->borderColor.c1);
	rpc->borderColor.c2 = htons(argp->borderColor.c2);
	rpc->borderColor.c3 = htons(argp->borderColor.c3);
	rpc->borderColor.isRGB = htons(argp->borderColor.isRGB);
	rpc->enBorder = argp->enBorder;

	if (send_rpc_command(RPC_AUDIO,
			     ENUM_VIDEO_KERNEL_RPC_CONFIGUREDISPLAYWINDOW,
			     rpc_info->paddr, rpc_info->paddr + offset,
			     &rpc->ret))
		goto exit;

	if (ntohl(rpc->result) != S_OK || rpc->ret != S_OK)
		goto exit;
	ret = 0;
exit:
	return ret;
}

int rpc_video_set_refclock(struct rtk_rpc_info *rpc_info,
			   struct rpc_refclock *argp)
{
	struct rpc_refclock_t *rpc = NULL;
	unsigned int offset;
	int ret = -1;

	rpc = (struct rpc_refclock_t *)rpc_info->vaddr;
	offset = get_rpc_alignment_offset(sizeof(*argp));

	memset_io(rpc, 0, sizeof(*rpc));

	rpc->instance = htonl(argp->instance);
	rpc->pRefClock = htonl(argp->pRefClock);

	if (send_rpc_command(RPC_AUDIO, ENUM_VIDEO_KERNEL_RPC_SETREFCLOCK,
			     rpc_info->paddr, rpc_info->paddr + offset,
			     &rpc->ret))
		goto exit;

	if (ntohl(rpc->result) != S_OK || rpc->ret != S_OK)
		goto exit;
	ret = 0;
exit:
	return ret;
}

int rpc_video_init_ringbuffer(struct rtk_rpc_info *rpc_info,
			      struct rpc_ringbuffer *argp)
{
	struct rpc_ringbuffer_t *rpc = NULL;
	unsigned int offset;
	int ret = -1;

	rpc = (struct rpc_ringbuffer_t *)rpc_info->vaddr;
	offset = get_rpc_alignment_offset(sizeof(unsigned int)*3);

	memset_io(rpc, 0, sizeof(*rpc));

	rpc->instance = htonl(argp->instance);
	rpc->readPtrIndex = htonl(argp->readPtrIndex);
	rpc->pinID = htonl(argp->pinID);
	rpc->pRINGBUFF_HEADER = htonl(argp->pRINGBUFF_HEADER);

	if (send_rpc_command(RPC_AUDIO, ENUM_VIDEO_KERNEL_RPC_INITRINGBUFFER,
			     rpc_info->paddr + offset, rpc_info->paddr,
			     &rpc->ret))
		goto exit;

	if (ntohl(rpc->result) != S_OK || rpc->ret != S_OK)
		goto exit;
	ret = 0;
exit:
	return ret;
}

int rpc_video_run(struct rtk_rpc_info *rpc_info, unsigned int instance)
{
	struct rpc_video_run_t *rpc = NULL;
	unsigned int offset;
	int ret = -1;

	rpc = (struct rpc_video_run_t *)rpc_info->vaddr;
	offset = get_rpc_alignment_offset(sizeof(unsigned int));

	memset_io(rpc, 0, sizeof(*rpc));

	rpc->instance = htonl(instance);

	if (send_rpc_command(RPC_AUDIO, ENUM_VIDEO_KERNEL_RPC_RUN,
			     rpc_info->paddr, rpc_info->paddr + offset,
			     &rpc->ret))
		goto exit;

	if (ntohl(rpc->result) != S_OK || rpc->ret != S_OK)
		goto exit;
	ret = 0;
exit:
	return ret;
}

int rpc_send_vout_edid_data(struct rtk_rpc_info *rpc_info,
			 struct rpc_vout_edid_data *arg)
{
	struct rpc_vout_edid_data *rpc = NULL;
	unsigned int offset;
	unsigned int i;
	unsigned int rpc_ret;
	int ret = -1;

	rpc = (struct rpc_vout_edid_data *)rpc_info->vaddr;
	offset = get_rpc_alignment_offset(sizeof(struct rpc_vout_edid_data));

	for (i = 0; i < sizeof(struct rpc_vout_edid_data); i++)
		((char *)rpc)[i] = ((char *)arg)[i];

	if (send_rpc_command(RPC_AUDIO,
			ENUM_VIDEO_KERNEL_RPC_VOUT_EDID_DATA,
			rpc_info->paddr, rpc_info->paddr + offset,
			&rpc_ret))
		goto exit;

	ret = 0;
exit:
	return ret;
}

int rpc_send_audio_edid_data(struct rtk_rpc_info *rpc_info,
			 struct rpc_audio_edid_data *arg)
{
	struct rpc_audio_edid_data *rpc;
	uint32_t offset;
	uint32_t rpc_ret;
	int ret = -1;

	rpc = (struct rpc_audio_edid_data *)rpc_info->vaddr;
	offset = get_rpc_alignment_offset(sizeof(struct rpc_audio_edid_data));

	memset_io(rpc, 0, sizeof(*rpc));

	ipcCopyMemory((unsigned char *)rpc, (unsigned char *)arg,
			sizeof(struct rpc_audio_edid_data));

	if (send_rpc_command(RPC_AUDIO,
			ENUM_KERNEL_RPC_HDMI_OUT_EDID2,
			rpc_info->paddr, rpc_info->paddr + offset,
			&rpc_ret))
		goto exit;

	ret = 0;
exit:
	return ret;
}

int rpc_send_hdmi_freq(struct rtk_rpc_info *rpc_info,
			 struct rpc_audio_hdmi_freq *arg)
{
	struct rpc_audio_hdmi_freq *rpc;
	uint32_t offset;
	uint32_t rpc_ret;
	int ret = -1;

	rpc = (struct rpc_audio_hdmi_freq *)rpc_info->vaddr;
	offset = get_rpc_alignment_offset(sizeof(struct rpc_audio_hdmi_freq));

	memset_io(rpc, 0, sizeof(*rpc));

	ipcCopyMemory((unsigned char *)rpc, (unsigned char *)arg,
			sizeof(struct rpc_audio_hdmi_freq));

	if (send_rpc_command(RPC_AUDIO,
			ENUM_KERNEL_RPC_HDMI_SET,
			rpc_info->paddr, rpc_info->paddr + offset,
			&rpc_ret))
		goto exit;

	ret = 0;
exit:
	return ret;
}

int rpc_set_vrr(struct rtk_rpc_info *rpc_info,
			struct rpc_vout_hdmi_vrr *arg)
{
	struct rpc_vout_hdmi_vrr *rpc;
	unsigned int offset;
	unsigned int rpc_ret;
	int ret = -EIO, i;

	rpc = (struct rpc_vout_hdmi_vrr *)rpc_info->vaddr;
	offset = get_rpc_alignment_offset(sizeof(struct rpc_vout_hdmi_vrr));

	rpc->vrr_function = htonl(arg->vrr_function);
	rpc->vrr_act = htonl(arg->vrr_act);

	for (i = 0; i < 15; i++)
		rpc->reserved[i] = htonl(arg->reserved[i]);

	if (send_rpc_command(RPC_AUDIO,
			ENUM_VIDEO_KERNEL_RPC_SET_HDMI_VRR,
			rpc_info->paddr, rpc_info->paddr + offset,
			&rpc_ret))
		goto exit;

	ret = 0;
exit:
	return ret;
}

int rpc_query_tv_system(struct rtk_rpc_info *rpc_info,
			struct rpc_config_tv_system *arg)
{
	struct rpc_config_tv_system *i_rpc = NULL;
	struct rpc_config_tv_system *o_rpc = NULL;
	unsigned int offset;
	unsigned int rpc_ret;
	int ret = -1, i;

	i_rpc = (struct rpc_config_tv_system *)rpc_info->vaddr;
	offset = get_rpc_alignment_offset(sizeof(struct rpc_config_tv_system));
	o_rpc = (struct rpc_config_tv_system *)((unsigned long)i_rpc + offset);

	if (send_rpc_command(RPC_AUDIO,
			     ENUM_VIDEO_KERNEL_RPC_QUERY_CONFIG_TV_SYSTEM,
			     rpc_info->paddr, rpc_info->paddr + offset,
			     &rpc_ret))
		goto exit;

	for (i = 0; i < sizeof(struct rpc_config_tv_system); i++)
		((char *)arg)[i] = ((char *)o_rpc)[i];

	arg->interfaceType = htonl(arg->interfaceType);
	arg->videoInfo.standard = htonl(arg->videoInfo.standard);
	arg->videoInfo.pedType  = htonl(arg->videoInfo.pedType);
	arg->videoInfo.dataInt0 = htonl(arg->videoInfo.dataInt0);
	arg->videoInfo.dataInt1 = htonl(arg->videoInfo.dataInt1);

	arg->info_frame.hdmiMode  = htonl(arg->info_frame.hdmiMode);
	arg->info_frame.audioSampleFreq = htonl(arg->info_frame.audioSampleFreq);
	arg->info_frame.dataInt0  = htonl(arg->info_frame.dataInt0);
	arg->info_frame.hdmi2px_feature = htonl(arg->info_frame.hdmi2px_feature);
	arg->info_frame.hdmi_off_mode = htonl(arg->info_frame.hdmi_off_mode);
	arg->info_frame.hdr_ctrl_mode = htonl(arg->info_frame.hdr_ctrl_mode);
	arg->info_frame.reserved4 = htonl(arg->info_frame.reserved4);

	ret = 0;
exit:
	return ret;
}

int rpc_config_tv_system(struct rtk_rpc_info *rpc_info,
			 struct rpc_config_tv_system *arg)
{
	struct rpc_config_tv_system *rpc = NULL;
	unsigned int offset;
	unsigned int rpc_ret;
	int ret = -1, i;

	rpc = (struct rpc_config_tv_system *)rpc_info->vaddr;
	offset = get_rpc_alignment_offset(sizeof(struct rpc_config_tv_system));

	for (i = 0; i < sizeof(struct rpc_config_tv_system); i++)
		((char *)rpc)[i] = ((char *)arg)[i];

	rpc->interfaceType = htonl(arg->interfaceType);
	rpc->videoInfo.standard = htonl(arg->videoInfo.standard);
	rpc->videoInfo.pedType  = htonl(arg->videoInfo.pedType);
	rpc->videoInfo.dataInt0 = htonl(arg->videoInfo.dataInt0);
	rpc->videoInfo.dataInt1 = htonl(arg->videoInfo.dataInt1);

	rpc->info_frame.hdmiMode  = htonl(arg->info_frame.hdmiMode);
	rpc->info_frame.audioSampleFreq = htonl(arg->info_frame.audioSampleFreq);
	rpc->info_frame.dataInt0  = htonl(arg->info_frame.dataInt0);
	rpc->info_frame.hdmi2px_feature = htonl(arg->info_frame.hdmi2px_feature);
	rpc->info_frame.hdmi_off_mode = htonl(arg->info_frame.hdmi_off_mode);
	rpc->info_frame.hdr_ctrl_mode = htonl(arg->info_frame.hdr_ctrl_mode);
	rpc->info_frame.reserved4 = htonl(arg->info_frame.reserved4);

	if (send_rpc_command(RPC_AUDIO,
			     ENUM_VIDEO_KERNEL_RPC_CONFIG_TV_SYSTEM,
			     rpc_info->paddr, rpc_info->paddr + offset,
			     &rpc_ret))
		goto exit;

	ret = 0;
exit:
	return ret;
}

int rtk_rpc_init(struct device *dev, struct rtk_rpc_info *rpc_info)
{
	struct rtk_ipc_shm __iomem *ipc = (void __iomem *)IPC_SHM_VIRT;

	rpc_info->vo_sync_flag = &ipc->vo_int_sync;
#ifdef USE_ION
	rpc_info->dmabuf = ion_alloc(RPC_CMD_BUFFER_SIZE, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(rpc_info->dmabuf)) {
		pr_err("%s dma buf get fail\n", __func__);
		return -1;
	}
	rpc_info->paddr = rtk_ion_mem_phy(rpc_info->dmabuf->priv);
	if (rpc_info->paddr == 0) {
		pr_err("%s get phy addr fail\n", __func__);
		return -1;
	}
	rpc_info->vaddr = rtk_ion_kernel_map(rpc_info->dmabuf->priv);
	if (rpc_info->vaddr  == NULL) {
		pr_err("%s get vaddr fail\n", __func__);
		return -1;
	}
#else
	rpc_info->vaddr = dma_alloc_attrs(dev, RPC_CMD_BUFFER_SIZE,
					&rpc_info->paddr,
					GFP_KERNEL | __GFP_NOWARN,
					DMA_ATTR_WRITE_COMBINE);
#endif
	if (!rpc_info->vaddr) {
		pr_err("%s failed to allocate rpc buffer\n", __func__);
		return -1;
	}

	return 0;
}
