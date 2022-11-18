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

#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>

#include "rtk_drm_drv.h"
#include "rtk_drm_crtc.h"

#define to_rtk_crtc(s) container_of(s, struct rtk_drm_crtc, crtc)
#define to_rtk_crtc_state(s) container_of(s, struct rtk_crtc_state, base)

static struct drm_crtc_state *rtk_crtc_duplicate_state(struct drm_crtc *crtc)
{
	struct rtk_crtc_state *state;

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

static void rtk_crtc_destroy_state(struct drm_crtc *crtc,
				   struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(state);
	kfree(to_rtk_crtc_state(state));
}

static int rtk_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct rtk_drm_crtc *rtk_crtc = to_rtk_crtc(crtc);
	struct rtk_rpc_info *rpc_info = rtk_crtc->rpc_info;
	unsigned int val;

	DRM_DEBUG_KMS("%d\n", __LINE__);

	val = readl(rpc_info->vo_sync_flag);
	val |= DC_VO_SET_NOTIFY;
	writel(val, rpc_info->vo_sync_flag);

	return 0;
}

static void rtk_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct rtk_drm_crtc *rtk_crtc = to_rtk_crtc(crtc);
	struct rtk_rpc_info *rpc_info = rtk_crtc->rpc_info;
	unsigned int val;

	DRM_DEBUG_KMS("%d\n", __LINE__);

	val = readl(rpc_info->vo_sync_flag);
	val &= ~DC_VO_SET_NOTIFY;
	writel(val, rpc_info->vo_sync_flag);
}

static void rtk_crtc_destroy(struct drm_crtc *crtc)
{
	drm_crtc_cleanup(crtc);
}

static void rtk_crtc_finish_page_flip(struct drm_crtc *crtc)
{
	struct rtk_drm_crtc *rtk_crtc = to_rtk_crtc(crtc);
	struct drm_device *drm = crtc->dev;
	unsigned long flags;

	spin_lock_irqsave(&drm->event_lock, flags);
	if (rtk_crtc->event) {
		drm_crtc_send_vblank_event(crtc, rtk_crtc->event);
		drm_crtc_vblank_put(crtc);
		rtk_crtc->event = NULL;
	}
	spin_unlock_irqrestore(&drm->event_lock, flags);
}

static const struct drm_crtc_funcs rtk_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.destroy = rtk_crtc_destroy,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = rtk_crtc_duplicate_state,
	.atomic_destroy_state = rtk_crtc_destroy_state,
	.enable_vblank = rtk_crtc_enable_vblank,
	.disable_vblank = rtk_crtc_disable_vblank,
};

static bool rtk_crtc_mode_fixup(struct drm_crtc *crtc,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	DRM_DEBUG_KMS("%d\n", __LINE__);
	return true;
}

static void rtk_crtc_atomic_flush(struct drm_crtc *crtc,
				struct drm_crtc_state *old_crtc_state)
{
	DRM_DEBUG_KMS("%d\n", __LINE__);
}

static void rtk_crtc_atomic_begin(struct drm_crtc *crtc,
				struct drm_crtc_state *old_crtc_state)
{
	struct rtk_drm_crtc *rtk_crtc = to_rtk_crtc(crtc);
	struct rtk_crtc_state *state = to_rtk_crtc_state(crtc->state);

	DRM_DEBUG_KMS("%d\n", __LINE__);

	if (rtk_crtc->event && state->base.event)
		DRM_ERROR("new event while there is still a pending event\n");

	if (state->base.event) {
		WARN_ON(drm_crtc_vblank_get(crtc) != 0);
		rtk_crtc->event = state->base.event;
		state->base.event = NULL;
	}
}

static void rtk_crtc_atomic_enable(struct drm_crtc *crtc,
				struct drm_crtc_state *old_crtc_state)
{
	DRM_DEBUG_KMS("%d\n", __LINE__);
	drm_crtc_vblank_on(crtc);
}

static void rtk_crtc_atomic_disable(struct drm_crtc *crtc,
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

static const struct drm_crtc_helper_funcs rtk_crtc_helper_funcs = {
	.mode_fixup = rtk_crtc_mode_fixup,
	.atomic_flush = rtk_crtc_atomic_flush,
	.atomic_begin = rtk_crtc_atomic_begin,
	.atomic_enable = rtk_crtc_atomic_enable,
	.atomic_disable = rtk_crtc_atomic_disable,
};

irqreturn_t rtk_crtc_isr(int irq, void *dev_id)
{
	struct rtk_drm_crtc *rtk_crtc = (struct rtk_drm_crtc *)dev_id;
	struct drm_crtc *crtc = &rtk_crtc->crtc;

	drm_crtc_handle_vblank(crtc);
	rtk_crtc_finish_page_flip(crtc);

	return IRQ_HANDLED;
}

static int rtk_crtc_bind(struct device *dev, struct device *master, void *data)
{
	struct device_node *np = dev->of_node;
	struct drm_device *drm = data;
	struct rtk_drm_private *priv = drm->dev_private;
	struct device_node *port;
	enum drm_plane_type type;
	struct rtk_drm_crtc *rtk_crtc;
	int i;

	rtk_crtc = devm_kzalloc(dev, sizeof(*rtk_crtc), GFP_KERNEL);

	rtk_crtc->rpc_info = &priv->rpc_info;
	dev_set_drvdata(dev, rtk_crtc);

	for (i = 0; i < VO_LAYER_NR; i++) {
		type = (i == 0) ? DRM_PLANE_TYPE_PRIMARY :
				(i == 1) ? DRM_PLANE_TYPE_CURSOR :
						DRM_PLANE_TYPE_OVERLAY;
		if (type != DRM_PLANE_TYPE_OVERLAY)
			rtk_plane_init(drm, &rtk_crtc->planes[i], 0, type);
	}

	drm_crtc_init_with_planes(drm, &rtk_crtc->crtc,
				  &rtk_crtc->planes[0].plane,
				  &rtk_crtc->planes[1].plane,
				  &rtk_crtc_funcs, NULL);

	drm_crtc_helper_add(&rtk_crtc->crtc, &rtk_crtc_helper_funcs);

	for (i = 0; i < VO_LAYER_NR; i++) {
		type = (i == 0) ? DRM_PLANE_TYPE_PRIMARY :
				(i == 1) ? DRM_PLANE_TYPE_CURSOR :
						DRM_PLANE_TYPE_OVERLAY;
		if (type == DRM_PLANE_TYPE_OVERLAY)
			rtk_plane_init(drm, &rtk_crtc->planes[i],
					1 << drm_crtc_index(&rtk_crtc->crtc),
					type);
	}

	port = of_get_child_by_name(dev->of_node, "port");
	if (!port)
		DRM_ERROR("no connect port node found\n");

	rtk_crtc->crtc.port = port;

	rtk_crtc->irq = irq_of_parse_and_map(np, 0);
	if (!rtk_crtc->irq) {
		DRM_ERROR("no irq for crtc\n");
		return -1;
	}

	if (devm_request_irq(dev, rtk_crtc->irq, rtk_crtc_isr, IRQF_SHARED | IRQF_NO_SUSPEND,
			  "crtc_irq", rtk_crtc)) {
		DRM_ERROR("can't request crtc irq\n");
		return -1;
	}

	return 0;
}

static void
rtk_crtc_unbind(struct device *dev, struct device *master, void *data)
{
	struct rtk_drm_crtc *rtk_crtc = dev_get_drvdata(dev);

	drm_crtc_cleanup(&rtk_crtc->crtc);
}

const struct component_ops rtk_crtc_component_ops = {
	.bind = rtk_crtc_bind,
	.unbind = rtk_crtc_unbind,
};

static int rtk_crtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	return component_add(dev, &rtk_crtc_component_ops);
}

static int rtk_crtc_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &rtk_crtc_component_ops);
	return 0;
}

static const struct of_device_id rtk_crtc_of_ids[] = {
	{ .compatible = "realtek,crtc" },
	{},
};

struct platform_driver rtk_crtc_platform_driver = {
	.probe = rtk_crtc_probe,
	.remove = rtk_crtc_remove,
	.driver = {
		.name = "realtek-crtc",
		.of_match_table = rtk_crtc_of_ids,
	},
};
