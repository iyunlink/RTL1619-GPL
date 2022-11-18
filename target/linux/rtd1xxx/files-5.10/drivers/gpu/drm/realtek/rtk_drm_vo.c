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

#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>
#include <drm/drm_fourcc.h>

#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/kthread.h>
#include <linux/dma-buf.h>
#include <linux/shmem_fs.h>

#include "rtk_drm_fb.h"
#include "rtk_drm_gem.h"

#include "../../../drivers/dma-buf/sync_debug.h"
#include "../../../video/fbdev/rtk/dc2vo.h"

#define VO_LAYER_NR	1
#define RTK_CRTC_SWAP_SHORT_FENCE_TIMEOUT_MS    (60)
#define RTK_CRTC_SWAP_LONG_FENCE_TIMEOUT_MS     (-1)

static const unsigned int formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

struct rtk_drm_plane {
	struct drm_plane plane;
};

struct rtk_drm_vo {
	struct drm_crtc crtc;
	struct drm_pending_vblank_event *event;

	struct rtk_drm_plane planes[VO_LAYER_NR];

	struct list_head swap_list;
	struct mutex swap_lock;
	struct task_struct *swap_thread;
	struct kthread_work swap_work;
	struct kthread_worker swap_worker;

	struct task_struct *vblank_task;
};

struct rtk_vo_swap_post {
	struct list_head head;
	struct sync_file *fence;
	struct drm_framebuffer *fb;
	struct rtk_drm_vo *rtk_vo;
	struct drm_pending_vblank_event *event;
};

struct rtk_vo_state {
	struct drm_crtc_state base;
};

#define to_rtk_vo(s) container_of(s, struct rtk_drm_vo, crtc)
#define to_rtk_vo_state(s) container_of(s, struct rtk_vo_state, base)

static void rtk_vo_swap_post_cleanup(struct rtk_vo_swap_post *cfg)
{
	if (cfg == NULL)
		return;
	if (cfg->fence)
		fput(cfg->fence->file);
	kfree(cfg);
}

static void rtk_vo_swap_post_release(struct rtk_vo_swap_post *cfg)
{
        struct rtk_drm_vo *rtk_vo = cfg->rtk_vo;
        struct drm_device *dev = rtk_vo->crtc.dev;
        unsigned long flags;
	int err;

	if (cfg == NULL)
		return;

	if (cfg->fence) {
		err = dma_fence_wait_timeout(cfg->fence->fence, 1, RTK_CRTC_SWAP_SHORT_FENCE_TIMEOUT_MS);
		if (err == -ETIME) {
			DRM_ERROR("DRM %s:%d fence %p\n", __FUNCTION__, __LINE__, cfg->fence);
			err = dma_fence_wait_timeout(cfg->fence->fence, 1, RTK_CRTC_SWAP_SHORT_FENCE_TIMEOUT_MS);
		}
		if (err < 0)
			DRM_ERROR("DRM %s:%d fence %p\n", __FUNCTION__, __LINE__, cfg->fence);
	}

	if (cfg->event) {
		spin_lock_irqsave(&dev->event_lock, flags);
		drm_crtc_vblank_get(&rtk_vo->crtc);
		drm_crtc_send_vblank_event(&rtk_vo->crtc, cfg->event);
		drm_crtc_vblank_put(&rtk_vo->crtc);
		spin_unlock_irqrestore(&dev->event_lock, flags);
		cfg->event = NULL;
	}

	return rtk_vo_swap_post_cleanup(cfg);
}

struct rtk_vo_swap_post *rtk_vo_swap_post_create(struct drm_framebuffer *fb,
						struct sync_file *fence,
						struct rtk_drm_vo *rtk_vo,
						struct drm_pending_vblank_event *event)
{
	struct rtk_vo_swap_post *cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);

	INIT_LIST_HEAD(&cfg->head);
	cfg->fence = fence;
	cfg->fb = fb;
	cfg->rtk_vo = rtk_vo;
	cfg->event = event;

	return cfg;
}

static void rtk_vo_swap_work_func(struct kthread_work *work)
{
	struct rtk_drm_vo *rtk_vo = container_of(work, struct rtk_drm_vo, swap_work);
	struct rtk_vo_swap_post *post, *next;
	struct list_head saved_list;

	mutex_lock(&rtk_vo->swap_lock);
	memcpy(&saved_list, &rtk_vo->swap_list, sizeof(saved_list));
	list_replace_init(&rtk_vo->swap_list, &saved_list);
	mutex_unlock(&rtk_vo->swap_lock);

	list_for_each_entry_safe(post, next, &saved_list, head) {
		list_del(&post->head);
		rtk_vo_swap_post_release(post);
	}
}

static struct drm_crtc_state *rtk_vo_duplicate_state(struct drm_crtc *crtc)
{
	struct rtk_vo_state *state;

	if (WARN_ON(!crtc->state))
		return NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &state->base);

	WARN_ON(state->base.crtc != crtc);
	state->base.crtc = crtc;

	return &state->base;
}

static void rtk_vo_destroy_state(struct drm_crtc *crtc,
				   struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(state);
	kfree(to_rtk_vo_state(state));
}

static int rtk_vo_enable_vblank(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("%d\n", __LINE__);

	return 0;
}

static void rtk_vo_disable_vblank(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("%d\n", __LINE__);
}

static void rtk_vo_destroy(struct drm_crtc *crtc)
{
	drm_crtc_cleanup(crtc);
}

static void rtk_vo_finish_page_flip(struct drm_crtc *crtc)
{
	struct rtk_drm_vo *rtk_vo = to_rtk_vo(crtc);
	struct drm_device *drm = crtc->dev;
	unsigned long flags;

	spin_lock_irqsave(&drm->event_lock, flags);
	if (rtk_vo->event) {
		drm_crtc_send_vblank_event(crtc, rtk_vo->event);
		drm_crtc_vblank_put(crtc);
		rtk_vo->event = NULL;
	}
	spin_unlock_irqrestore(&drm->event_lock, flags);
}

static int vblank_thread(void *data)
{
	struct rtk_drm_vo *rtk_vo = data;
        struct drm_device *dev = rtk_vo->crtc.dev;
	unsigned long long nsecs = 0;

	do {
		if (drm_dev_has_vblank(dev)) {
			drm_crtc_handle_vblank(&rtk_vo->crtc);
			rtk_vo_finish_page_flip(&rtk_vo->crtc);
		}
		DC_VsyncWait(&nsecs);
	} while(!kthread_should_stop());

	return 0;
}

int rtk_vo_page_flip(struct drm_crtc *crtc,
		     struct drm_framebuffer *fb,
		     struct drm_pending_vblank_event *event)
//		     uint32_t flags,
//		     struct drm_modeset_acquire_ctx *ctx)
{
	struct rtk_drm_vo *rtk_vo = to_rtk_vo(crtc);
	struct drm_gem_object *gem;
	struct rtk_gem_object *rtk_gem;
        struct dc_buffer buf;
	struct rtk_vo_swap_post *cfg = NULL;
	struct sync_file *fence = NULL;

	DRM_DEBUG_KMS("%s, width=%d, height=%d\n", __func__,
			fb->width, fb->height);

	gem = rtk_fb_get_gem_obj(fb, 0);
	rtk_gem = to_rtk_gem_obj(gem);

	memset(&buf, 0, sizeof(buf));
	buf.id = eUserBuffer;
	buf.overlay_engine = eEngine_VO;
	buf.phyAddr = rtk_gem->paddr;
	buf.width = fb->width;
	buf.height = fb->height;
	buf.stride = fb->pitches[0];
	buf.acquire.fence = (struct sync_file *) 0;

//	rtk_store_fb_info(rtk_crtc, &buf);	//??
	if (fb->format->format == DRM_FORMAT_XRGB8888) {
		buf.flags |= eBuffer_USE_GLOBAL_ALPHA;
		buf.alpha = 0xff;
        }

/*	if (fb->modifier != 0) {
		DRM_INFO_ONCE("AFBC modifier: 0x%llx\n", fb->modifier);
		buf.flags |= eBuffer_AFBC_Enable;
		buf.flags |= eBuffer_AFBC_YUV_Transform;
		buf.format = INBAND_CMD_GRAPHIC_FORMAT_ARGB8888_LITTLE | (1 << 29);
	}*/

        fence = DC_QueueBuffer(&buf);
	if (fence == NULL)
		return -1;

	cfg = rtk_vo_swap_post_create(fb, fence, rtk_vo, event);
	if (cfg == NULL) {
		fput(fence->file);
		return -1;
	}
/*
	if (fb) {
		struct rtk_gem_ion_object * ion_obj = drm_fb_ion_get_gem_obj(fb,0);
		if (ion_obj)
			drm_gem_object_reference(&ion_obj->base);
		drm_framebuffer_reference(fb);
	}
*/
	mutex_lock(&rtk_vo->swap_lock);
	list_add_tail(&cfg->head, &rtk_vo->swap_list);
	kthread_queue_work(&rtk_vo->swap_worker, &rtk_vo->swap_work);
	mutex_unlock(&rtk_vo->swap_lock);

	return 0;
}

static const struct drm_plane_funcs rtk_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static const struct drm_crtc_funcs rtk_vo_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.destroy = rtk_vo_destroy,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = rtk_vo_duplicate_state,
	.atomic_destroy_state = rtk_vo_destroy_state,
	.enable_vblank = rtk_vo_enable_vblank,
	.disable_vblank = rtk_vo_disable_vblank,
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
	struct drm_pending_vblank_event *event;

	DRM_DEBUG_KMS("%d\n", __LINE__);

	if (!crtc || WARN_ON(!fb))
		return;

	spin_lock_irq(&crtc->dev->event_lock);
	event = crtc->state->event;
	spin_unlock_irq(&crtc->dev->event_lock);

	rtk_vo_page_flip(crtc, fb, event);
}

static void rtk_plane_atomic_disable(struct drm_plane *plane,
				struct drm_plane_state *old_state)
{
	DRM_DEBUG_KMS("%d\n", __LINE__);
}

static bool rtk_vo_mode_fixup(struct drm_crtc *crtc,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	DRM_DEBUG_KMS("%d\n", __LINE__);
	return true;
}

static void rtk_vo_atomic_flush(struct drm_crtc *crtc,
				struct drm_crtc_state *old_crtc_state)
{
	DRM_DEBUG_KMS("%d\n", __LINE__);
}


static void rtk_vo_atomic_begin(struct drm_crtc *crtc,
				struct drm_crtc_state *old_crtc_state)
{
	struct rtk_drm_vo *rtk_vo = to_rtk_vo(crtc);
	struct rtk_vo_state *state = to_rtk_vo_state(crtc->state);

	DRM_DEBUG_KMS("%d\n", __LINE__);

	if (rtk_vo->event && state->base.event)
		DRM_ERROR("new event while there is still a pending event\n");

	if (state->base.event) {
		state->base.event->pipe = drm_crtc_index(crtc);
		WARN_ON(drm_crtc_vblank_get(crtc) != 0);
		rtk_vo->event = state->base.event;
		state->base.event = NULL;
	}
}

static void rtk_vo_atomic_enable(struct drm_crtc *crtc,
				struct drm_crtc_state *old_crtc_state)
{
	DRM_DEBUG_KMS("%d\n", __LINE__);

	drm_crtc_vblank_on(crtc);
}

static void rtk_vo_atomic_disable(struct drm_crtc *crtc,
				struct drm_crtc_state *old_crtc_state)
{
	DRM_DEBUG_KMS("%d\n", __LINE__);

	drm_crtc_wait_one_vblank(crtc);
	drm_crtc_vblank_off(crtc);

	if (crtc->state->event) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static const struct drm_plane_helper_funcs rtk_plane_helper_funcs = {
	.atomic_check = rtk_plane_atomic_check,
	.atomic_update = rtk_plane_atomic_update,
	.atomic_disable = rtk_plane_atomic_disable,
};

static const struct drm_crtc_helper_funcs rtk_vo_helper_funcs = {
	.mode_fixup = rtk_vo_mode_fixup,
	.atomic_flush = rtk_vo_atomic_flush,
	.atomic_begin = rtk_vo_atomic_begin,
	.atomic_enable = rtk_vo_atomic_enable,
	.atomic_disable = rtk_vo_atomic_disable,
};
/*
irqreturn_t rtk_vo_isr(int irq, void *dev_id)
{
	struct rtk_drm_vo *rtk_vo = (struct rtk_drm_vo *)dev_id;
	struct drm_crtc *crtc = &rtk_vo->crtc;

	drm_crtc_handle_vblank(crtc);
	rtk_vo_finish_page_flip(crtc);

	return IRQ_HANDLED;
}
*/
static int rtk_plane_init(struct drm_device *drm, struct rtk_drm_plane *rtk_plane,
		   unsigned long possible_crtcs, enum drm_plane_type type)
{
	struct drm_plane *plane = &rtk_plane->plane;
	int err;

	err = drm_universal_plane_init(drm, plane, possible_crtcs,
				       &rtk_plane_funcs, formats,
				       ARRAY_SIZE(formats), NULL, type, NULL);
	if (err) {
		DRM_ERROR("failed to initialize plane\n");
		return err;
	}

	drm_plane_helper_add(plane, &rtk_plane_helper_funcs);

	return 0;
}

static int rtk_vo_bind(struct device *dev, struct device *master, void *data)
{
//	struct device_node *np = dev->of_node;
	struct drm_device *drm = data;
	struct device_node *port;
	enum drm_plane_type type;
	struct rtk_drm_vo *rtk_vo;
	int i;

	rtk_vo = devm_kzalloc(dev, sizeof(*rtk_vo), GFP_KERNEL);

	dev_set_drvdata(dev, rtk_vo);

	for (i = 0; i < VO_LAYER_NR; i++) {
		type = (i == 0) ? DRM_PLANE_TYPE_PRIMARY :
				(i == 1) ? DRM_PLANE_TYPE_CURSOR :
						DRM_PLANE_TYPE_OVERLAY;
		if (type != DRM_PLANE_TYPE_OVERLAY)
			rtk_plane_init(drm, &rtk_vo->planes[i], 0, type);
	}

	drm_crtc_init_with_planes(drm, &rtk_vo->crtc,
				  &rtk_vo->planes[0].plane,
				  NULL,
				  &rtk_vo_funcs, NULL);

	drm_crtc_helper_add(&rtk_vo->crtc, &rtk_vo_helper_funcs);

	port = of_get_child_by_name(dev->of_node, "port");
	if (!port)
		DRM_ERROR("no connect port node found\n");

	rtk_vo->crtc.port = port;
	INIT_LIST_HEAD(&rtk_vo->swap_list);
	mutex_init(&rtk_vo->swap_lock);
	kthread_init_worker(&rtk_vo->swap_worker);
	kthread_init_work(&rtk_vo->swap_work, rtk_vo_swap_work_func);
	rtk_vo->swap_thread = kthread_run(kthread_worker_fn,
					&rtk_vo->swap_worker,
					"rtk_crtc swap_worker");
	rtk_vo->vblank_task = kthread_create(vblank_thread, rtk_vo, "vblank thread");
	wake_up_process(rtk_vo->vblank_task);
/*	rtk_vo->irq = irq_of_parse_and_map(np, 0);
	if (!rtk_vo->irq) {
		DRM_ERROR("no irq for crtc\n");
		return -1;
	}

	if (request_irq(rtk_vo->irq, rtk_vo_isr, IRQF_SHARED | IRQF_NO_SUSPEND,
			  "crtc_irq", rtk_vo)) {
		DRM_ERROR("can't request crtc irq\n");
		return -1;
	}
*/
	return 0;
}

static void
rtk_vo_unbind(struct device *dev, struct device *master, void *data)
{
	struct rtk_drm_vo *rtk_vo = dev_get_drvdata(dev);

	drm_crtc_cleanup(&rtk_vo->crtc);
}

const struct component_ops rtk_vo_component_ops = {
	.bind = rtk_vo_bind,
	.unbind = rtk_vo_unbind,
};

static int rtk_vo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	return component_add(dev, &rtk_vo_component_ops);
}

static int rtk_vo_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &rtk_vo_component_ops);
	return 0;
}

static const struct of_device_id rtk_vo_of_ids[] = {
	{ .compatible = "realtek,vo" },
	{},
};

struct platform_driver rtk_vo_platform_driver = {
	.probe = rtk_vo_probe,
	.remove = rtk_vo_remove,
	.driver = {
		.name = "realtek-vo",
		.of_match_table = rtk_vo_of_ids,
	},
};
