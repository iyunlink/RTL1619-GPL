/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019 RealTek Inc.
 */

#ifndef _RTK_DRM_FB_H
#define _RTK_DRM_FB_H

struct drm_gem_object *rtk_fb_get_gem_obj(struct drm_framebuffer *fb,
					  unsigned int plane);
void rtk_drm_mode_config_init(struct drm_device *dev);

#endif  /* _RTK_DRM_FB_H_ */
