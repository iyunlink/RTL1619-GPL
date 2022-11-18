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

#include <drm/drm_plane_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fourcc.h>

#include "rtk_drm_drv.h"
#include "rtk_drm_fb.h"
#include "rtk_drm_gem.h"
#include "rtk_drm_crtc.h"

#define to_rtk_plane(s) container_of(s, struct rtk_drm_plane, plane)
//#define VO1_FOR_PRIMARY

static const unsigned int formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YUV422,
	DRM_FORMAT_YUV420,
	DRM_FORMAT_YVU420,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
};

#define ulPhyAddrFilter(x) ((x) & ~0xe0000000)

static uint64_t htonll(long long val)
{
	return (((long long) htonl(val)) << 32) + htonl(val >> 32);
}

static int write_cmd_to_ringbuffer(struct rtk_drm_plane *rtk_plane, void *cmd)
{
	void *base_iomap = rtk_plane->ringbase;
	struct tag_ringbuffer_header *rbHeader = rtk_plane->ringheader;
	unsigned int size = ((struct inband_cmd_pkg_header *)cmd)->size;
	unsigned int read = ipcReadULONG((u8 *)&rbHeader->readPtr[0]);
	unsigned int write = ipcReadULONG((u8 *)&(rbHeader->writePtr));
	unsigned int base = ipcReadULONG((u8 *)&(rbHeader->beginAddr));
	unsigned int b_size = ipcReadULONG((u8 *)&(rbHeader->size));
	unsigned int limit = base + b_size;

	if (read + (read > write ? 0 : limit - base) - write > size) {
		unsigned long offset = write - base;
		void *write_io = (void *)((unsigned long)base_iomap + offset);

		if (write + size <= limit) {
			ipcCopyMemory((void *)write_io, cmd, size);
		} else {
			ipcCopyMemory((void *)write_io, cmd, limit - write);
			ipcCopyMemory((void *)base_iomap, (void *)((unsigned long)cmd + limit - write), size - (limit - write));
		}
		write += size;
		write = write < limit ? write : write - (limit - base);

		rbHeader->writePtr = ipcReadULONG((u8 *)&write);
	} else {
		DRM_ERROR("errQ r:%x w:%x size:%u base:%u limit:%u\n",
			  read, write, size, base, limit);
		goto err;
	}

	return 0;
err:
	return -1;
}

static void init_video_object(struct video_object *obj)
{
	memset(obj, 0, sizeof(struct video_object));

	obj->lumaOffTblAddr = 0xffffffff;
	obj->chromaOffTblAddr = 0xffffffff;
	obj->lumaOffTblAddrR = 0xffffffff;
	obj->chromaOffTblAddrR = 0xffffffff;
	obj->bufBitDepth = 8;
	obj->matrix_coefficients = 1;
	obj->tch_hdr_metadata.specVersion = -1;

	obj->Y_addr_Right = 0xffffffff;
	obj->U_addr_Right = 0xffffffff;
	obj->pLock_Right = 0xffffffff;
}

static int queue_ring_buffer(struct drm_plane *plane)
{
	struct rtk_drm_plane *rtk_plane = to_rtk_plane(plane);
	struct rtk_rpc_info *rpc_info = rtk_plane->rpc_info;
	struct drm_framebuffer *fb = plane->state->fb;
	struct drm_gem_object *gem;
	struct rtk_gem_object *rtk_gem;
	enum drm_plane_type type = plane->type;
	struct vo_rectangle *p_rect, rect;
	struct vo_color blueBorder = {0, 0, 255, 1};
	unsigned int videoplane;

	if (type == DRM_PLANE_TYPE_PRIMARY)
#ifdef VO1_FOR_PRIMARY
		videoplane = VO_VIDEO_PLANE_V1;
#else
		videoplane = VO_VIDEO_PLANE_OSD1;
#endif
	else if (type == DRM_PLANE_TYPE_CURSOR)
		videoplane = VO_VIDEO_PLANE_SUB1;
	else
#ifdef VO1_FOR_PRIMARY
		videoplane = VO_VIDEO_PLANE_OSD1;
#else
		videoplane = VO_VIDEO_PLANE_V1;
#endif

	gem = rtk_fb_get_gem_obj(fb, 0);
	rtk_gem = to_rtk_gem_obj(gem);

#ifdef VO1_FOR_PRIMARY
	if (type != DRM_PLANE_TYPE_OVERLAY) {
#else
	if (type == DRM_PLANE_TYPE_OVERLAY) {
#endif
		struct video_object obj;

		init_video_object(&obj);

		obj.header.type = VIDEO_VO_INBAND_CMD_TYPE_OBJ_PIC;
		obj.header.size = sizeof(struct video_object);
		obj.version = 0x306B7472;	// now use rtk1, fix me ....
		obj.width = fb->width;
		obj.height = fb->height;
		obj.Y_pitch = fb->width;
		obj.mode = CONSECUTIVE_FRAME;
		obj.Y_addr = rtk_gem->paddr;
		obj.U_addr = rtk_gem->paddr + (obj.Y_pitch * 64);

		write_cmd_to_ringbuffer(rtk_plane, &obj);
	} else {
		struct graphic_object obj;
		unsigned int flags = 0;

		memset(&obj, 0, sizeof(struct graphic_object));
		obj.header.type = VIDEO_GRAPHIC_INBAND_CMD_TYPE_PICTURE_OBJECT;
		obj.header.size = sizeof(struct graphic_object);
		obj.colorkey = -1;
		if (fb->format->format == DRM_FORMAT_XRGB8888) {
			flags |= eBuffer_USE_GLOBAL_ALPHA;
			obj.alpha = 0xff;
		}
		obj.format = INBAND_CMD_GRAPHIC_FORMAT_ARGB8888_LITTLE;
		obj.width = fb->width;
		obj.height = fb->height;
		obj.pitch = fb->pitches[0];
		obj.address = rtk_gem->paddr;
		obj.picLayout = INBAND_CMD_GRAPHIC_2D_MODE;
		obj.afbc = (flags & eBuffer_AFBC_Enable)?1:0;
		obj.afbc_block_split = (flags & eBuffer_AFBC_Split)?1:0;
		obj.afbc_yuv_transform = (flags & eBuffer_AFBC_YUV_Transform)?1:0;

		write_cmd_to_ringbuffer(rtk_plane, &obj);
	}

	rect.x = plane->state->crtc_x;
	rect.y = plane->state->crtc_y;
	rect.width = fb->width;
	rect.height = fb->height;

	p_rect = &rtk_plane->disp_win.videoWin;
	if (p_rect->width != fb->width || p_rect->height != fb->height) {
		rtk_plane->disp_win.videoPlane = videoplane;
		rtk_plane->disp_win.videoWin = rect;
		rtk_plane->disp_win.borderWin = rect;
		rtk_plane->disp_win.borderColor = blueBorder;
		rtk_plane->disp_win.enBorder = 0;
		if (rpc_video_config_disp_win(rpc_info, &rtk_plane->disp_win)) {
			DRM_ERROR("rpc_video_config_disp_win RPC fail\n");
			return -1;
		}
	}

	return 0;
}

static int rtk_plane_rpc_init(struct rtk_drm_plane *rtk_plane)
{
//	struct drm_device *drm = rtk_plane->plane.dev;
	struct rtk_rpc_info *rpc_info = rtk_plane->rpc_info;
	void *vaddr;
	dma_addr_t paddr;
	struct rpc_refclock refclock;
	struct rpc_ringbuffer ringbuffer;
	unsigned int id;

	enum drm_plane_type type = rtk_plane->plane.type;
	enum VO_VIDEO_PLANE videoplane;

	struct vo_rectangle rect;
	struct vo_color blueBorder = {0, 0, 255, 1};

	if (type == DRM_PLANE_TYPE_PRIMARY)
#ifdef VO1_FOR_PRIMARY
		videoplane = VO_VIDEO_PLANE_V1;
#else
		videoplane = VO_VIDEO_PLANE_OSD1;
#endif
	else if (type == DRM_PLANE_TYPE_CURSOR)
		videoplane = VO_VIDEO_PLANE_SUB1;
	else
#ifdef VO1_FOR_PRIMARY
		videoplane = VO_VIDEO_PLANE_OSD1;
#else
		videoplane = VO_VIDEO_PLANE_V1;
#endif

#ifdef USE_ION
	rtk_plane->dmabuf = ion_alloc(67*1024, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (rtk_plane->handle < 0) {
		pr_err("%s ring buffer init fail\n", __func__);
		return -1;
	}

	paddr = rtk_ion_mem_phy(rtk_plane->dmabuf->priv);
	vaddr = rtk_ion_kernel_map(rtk_plane->dmabuf->priv);
#else
	vaddr = dma_alloc_attrs(drm->dev, 67*1024, &paddr,
				GFP_KERNEL | __GFP_NOWARN,
				DMA_ATTR_WRITE_COMBINE);
#endif
	rtk_plane->ringbase = (void *)((unsigned long)(vaddr));
	rtk_plane->refclock = (struct tag_refclock *)((unsigned long)(vaddr)+(65*1024));
	rtk_plane->ringheader = (struct tag_ringbuffer_header *)
			((unsigned long)(vaddr)+(64*1024));

	if (rpc_create_video_agent(rpc_info, &id, VF_TYPE_VIDEO_OUT)) {
		DRM_ERROR("rpc_create_video_agent RPC fail\n");
		return -1;
	}

	rtk_plane->info.instance = id;
	rtk_plane->info.videoPlane = videoplane;
	rtk_plane->info.zeroBuffer = 0;
	rtk_plane->info.realTimeSrc = 0;

	if (rpc_video_display(rpc_info, &rtk_plane->info)) {
		DRM_ERROR("rpc_video_display RPC fail\n");
		return -1;
	}

	rect.x = 0;
	rect.y = 0;
	rect.width = 1920;
	rect.height = 1080;

	rtk_plane->disp_win.videoPlane = videoplane;
	rtk_plane->disp_win.videoWin = rect;
	rtk_plane->disp_win.borderWin = rect;
	rtk_plane->disp_win.borderColor = blueBorder;
	rtk_plane->disp_win.enBorder = 0;

	if (rpc_video_config_disp_win(rpc_info, &rtk_plane->disp_win)) {
		DRM_ERROR("rpc_video_config_disp_win RPC fail\n");
		return -1;
	}

	rtk_plane->refclock->RCD = htonll(-1LL);
	rtk_plane->refclock->RCD_ext = htonl(-1L);
	rtk_plane->refclock->masterGPTS = htonll(-1LL);
	rtk_plane->refclock->GPTSTimeout = htonll(0LL);
	rtk_plane->refclock->videoSystemPTS = htonll(-1LL);
	rtk_plane->refclock->audioSystemPTS = htonll(-1LL);
	rtk_plane->refclock->videoRPTS = htonll(-1LL);
	rtk_plane->refclock->audioRPTS = htonll(-1LL);
	rtk_plane->refclock->videoContext = htonl(-1);
	rtk_plane->refclock->audioContext = htonl(-1);
	rtk_plane->refclock->videoEndOfSegment = htonl(-1);
	rtk_plane->refclock->videoFreeRunThreshold = htonl(0x7FFFFFFF);
	rtk_plane->refclock->audioFreeRunThreshold = htonl(0x7FFFFFFF);
	rtk_plane->refclock->VO_Underflow = htonl(0);
	rtk_plane->refclock->AO_Underflow = htonl(0);
	rtk_plane->refclock->mastership.systemMode = (unsigned char)AVSYNC_FORCED_SLAVE;
	rtk_plane->refclock->mastership.videoMode = (unsigned char)AVSYNC_FORCED_MASTER;
	rtk_plane->refclock->mastership.audioMode = (unsigned char)AVSYNC_FORCED_MASTER;
	rtk_plane->refclock->mastership.masterState = (unsigned char)AUTOMASTER_NOT_MASTER;
	refclock.instance = id;
	refclock.pRefClock = (long)(0xffffffff&paddr)+65*1024;
	if (rpc_video_set_refclock(rpc_info, &refclock)) {
		DRM_ERROR("rpc_video_set_refclock RPC fail\n");
		return -1;
	}

	rtk_plane->ringheader->beginAddr = htonl((long)(0xffffffff&paddr));
	rtk_plane->ringheader->size = htonl(64*1024);
	rtk_plane->ringheader->writePtr = rtk_plane->ringheader->beginAddr;
	rtk_plane->ringheader->readPtr[0] = rtk_plane->ringheader->beginAddr;
	rtk_plane->ringheader->bufferID = htonl(1);
	memset(&ringbuffer, 0, sizeof(ringbuffer));
	ringbuffer.instance = id;
	ringbuffer.readPtrIndex = 0;
	ringbuffer.pinID = 0;
	ringbuffer.pRINGBUFF_HEADER = (long)(0xffffffff&paddr)+64*1024;

	if (rpc_video_init_ringbuffer(rpc_info, &ringbuffer)) {
		DRM_ERROR("rpc_video_int_ringbuffer RPC fail\n");
		return -1;
	}

	if (rpc_video_run(rpc_info, id)) {
		DRM_ERROR("rpc_video_run RPC fail\n");
		return -1;
	}

	rtk_plane->flags |= RPC_READY;

	rtk_plane->info.instance = id;
	rtk_plane->info.videoPlane = videoplane;
	rtk_plane->info.zeroBuffer = 1;
	rtk_plane->info.realTimeSrc = 0;

	if (rpc_video_display(rpc_info, &rtk_plane->info)) {
		DRM_ERROR("rpc_video_display RPC fail\n");
		return -1;
	}

	return 0;
}

static void rtk_plane_destroy(struct drm_plane *plane)
{
	struct rtk_drm_plane *rtk_plane = to_rtk_plane(plane);

	dma_buf_put(rtk_plane->dmabuf);
	drm_plane_cleanup(plane);
}

static const struct drm_plane_funcs rtk_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.destroy = rtk_plane_destroy,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static int rtk_plane_atomic_check(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	DRM_DEBUG_KMS("%d\n", __LINE__);

	return 0;
}

static void rtk_plane_atomic_update(struct drm_plane *plane,
				    struct drm_plane_state *old_state)
{
	struct drm_crtc *crtc = plane->state->crtc;
	struct drm_framebuffer *fb = plane->state->fb;

	DRM_DEBUG_KMS("%s, width=%d, height=%d\n", __func__,
			fb->width, fb->height);

	if (!crtc || WARN_ON(!fb))
		return;

	queue_ring_buffer(plane);
}

static void rtk_plane_atomic_disable(struct drm_plane *plane,
				struct drm_plane_state *old_state)
{
	struct rtk_drm_plane *rtk_plane = to_rtk_plane(plane);
	struct rtk_rpc_info *rpc_info = rtk_plane->rpc_info;
	struct vo_rectangle rect;

	DRM_DEBUG_KMS("%d\n", __LINE__);

	rect.x = 0;
	rect.y = 0;
	rect.width = 0;
	rect.height = 0;

	rtk_plane->disp_win.videoWin = rect;
	rtk_plane->disp_win.borderWin = rect;

	if (rpc_video_config_disp_win(rpc_info, &rtk_plane->disp_win))
		DRM_ERROR("rpc_video_config_disp_win RPC fail\n");
}

static const struct drm_plane_helper_funcs rtk_plane_helper_funcs = {
	.atomic_check = rtk_plane_atomic_check,
	.atomic_update = rtk_plane_atomic_update,
	.atomic_disable = rtk_plane_atomic_disable,
};

int rtk_plane_init(struct drm_device *drm, struct rtk_drm_plane *rtk_plane,
		   unsigned long possible_crtcs, enum drm_plane_type type)
{
	struct drm_plane *plane = &rtk_plane->plane;
	struct rtk_drm_private *priv = drm->dev_private;
	int err;

	err = drm_universal_plane_init(drm, plane, possible_crtcs,
				       &rtk_plane_funcs, formats,
				       ARRAY_SIZE(formats), NULL, type, NULL);
	if (err) {
		DRM_ERROR("failed to initialize plane\n");
		return err;
	}

	drm_plane_helper_add(plane, &rtk_plane_helper_funcs);

	rtk_plane->rpc_info = &priv->rpc_info;
	rtk_plane->gAlpha = 0;
	rtk_plane->flags &= ~BG_SWAP;
	rtk_plane->flags |= VSYNC_FORCE_LOCK;

	rtk_plane_rpc_init(rtk_plane);

	return 0;
}
