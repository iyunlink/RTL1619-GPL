/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019 RealTek Inc.
 */

#ifndef _RTK_DRM_CRTC_H
#define _RTK_DRM_CRTC_H

#include "rtk_drm_rpc.h"

#define VO_LAYER_NR	3

enum {
	RPC_READY = (1U << 0),
	ISR_INIT = (1U << 2),
	WAIT_VSYNC = (1U << 3),
	CHANGE_RES = (1U << 4),
	BG_SWAP = (1U << 5),
	SUSPEND = (1U << 6),
	VSYNC_FORCE_LOCK = (1U << 7),
};

struct rtk_drm_plane {
	struct drm_plane plane;

#ifdef USE_ION
	int handle;
	struct dma_buf *dmabuf;
#endif
	struct rtk_rpc_info *rpc_info;
	struct rpc_vo_filter_display info;
	struct rpc_config_disp_win disp_win;

	void *ringbase;
	struct tag_refclock *refclock;
	struct tag_ringbuffer_header *ringheader;

	unsigned int flags;
	unsigned int gAlpha; /* [0]:Pixel Alpha	[0x01 ~ 0xFF]:Global Alpha */
};

struct rtk_drm_crtc {
	struct drm_crtc crtc;
	struct drm_pending_vblank_event *event;

	struct rtk_drm_plane planes[VO_LAYER_NR];
	struct rtk_rpc_info *rpc_info;

	void *vo_vsync_flag; /* VSync enable and notify. */
	unsigned int irq;
};

struct rtk_crtc_state {
	struct drm_crtc_state base;
};

int rtk_plane_init(struct drm_device *drm, struct rtk_drm_plane *rtk_plane,
		   unsigned long possible_crtcs, enum drm_plane_type type);

#endif  /* _RTK_DRM_CRTC_H_ */
