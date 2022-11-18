/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019 RealTek Inc.
 */

#ifndef _RTK_DRM_DRV_H
#define _RTK_DRM_DRV_H

#include "rtk_drm_rpc.h"

#define RTK_MAX_CRTC		1
#define RTK_MAX_FB_BUFFER	3
#define RTK_GEM_INFO_MAX	128

struct rtk_gem_object_info {
	const char *name;
	dma_addr_t paddr[RTK_GEM_INFO_MAX];
	u32 num_allocated;
	u32 size_allocated;
};

struct rtk_drm_private {
	struct rtk_gem_object_info *obj_info;
	struct mutex obj_lock;
	struct rtk_rpc_info rpc_info;
	int obj_info_num;
};

extern struct platform_driver rtk_crtc_platform_driver;
extern struct platform_driver rtk_vo_platform_driver;
extern struct platform_driver rtk_hdmi_driver;
extern struct platform_driver rtk_hdmi_legacy_driver;
extern struct platform_driver rtk_dptx_driver;

#endif /* _RTK_DRM_DRV_H_ */
