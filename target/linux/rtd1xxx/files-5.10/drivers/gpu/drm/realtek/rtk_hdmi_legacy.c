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

#include <drm/drm_of.h>

#include <linux/platform_device.h>
#include <linux/component.h>

#include <drm/drm_print.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_probe_helper.h>

#include "../../../media/platform/rtk_hdmitx/hdmitx_dev.h"

struct rtk_hdmitx {
	struct device *dev;
	struct drm_device *drm_dev;
	struct drm_connector connector;
	struct drm_encoder encoder;
};

#define to_rtk_hdmitx(x) container_of(x, struct rtk_hdmitx, x)

static bool rtk_hdmitx_enc_mode_fixup(struct drm_encoder *encoder,
				      const struct drm_display_mode *mode,
				      struct drm_display_mode *adj_mode)
{
	DRM_DEBUG_KMS("%s\n", __func__);
	return true;
}

static void rtk_hdmitx_enc_mode_set(struct drm_encoder *encoder,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adj_mode)
{
	DRM_DEBUG_KMS("%s\n", __func__);
}

static void rtk_hdmitx_enc_enable(struct drm_encoder *encoder)
{
	DRM_DEBUG_KMS("%s\n", __func__);
}

static void rtk_hdmitx_enc_disable(struct drm_encoder *encoder)
{
	DRM_DEBUG_KMS("%s\n", __func__);
}

static int rtk_hdmitx_enc_atomic_check(struct drm_encoder *encoder,
				       struct drm_crtc_state *crtc_state,
				       struct drm_connector_state *conn_state)
{
	DRM_DEBUG_KMS("%s\n", __func__);
	return 0;
}

static const struct drm_encoder_funcs rtk_hdmitx_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_encoder_helper_funcs rtk_hdmitx_encoder_helper_funcs = {
	.mode_fixup = rtk_hdmitx_enc_mode_fixup,
	.mode_set   = rtk_hdmitx_enc_mode_set,
	.enable     = rtk_hdmitx_enc_enable,
	.disable    = rtk_hdmitx_enc_disable,
	.atomic_check = rtk_hdmitx_enc_atomic_check,
};

static enum drm_connector_status
rtk_hdmitx_conn_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void rtk_hdmitx_conn_destroy(struct drm_connector *connector)
{
	DRM_DEBUG_KMS("%s\n", __func__);
}

static int rtk_hdmitx_edid_callback(struct device *dev, void *data)
{
	return hdmitx_get_raw_edid(dev, data);
}

static int rtk_hdmitx_conn_get_modes(struct drm_connector *connector)
{
//	struct rtk_hdmitx *hdmitx = to_rtk_hdmitx(connector);
	struct device_driver *drv = driver_find("rtk_hdmi", &platform_bus_type);
	unsigned char *hdmitx_edid;
	int err, count = 0;

	hdmitx_edid = kmalloc(2 * EDID_LENGTH, GFP_KERNEL);
	if (hdmitx_edid != NULL) {
		err = driver_for_each_device(drv, NULL, hdmitx_edid, rtk_hdmitx_edid_callback);
		if (!err) {
			drm_connector_update_edid_property(connector, (struct edid *)hdmitx_edid);
			count = drm_add_edid_modes(connector, (struct edid *)hdmitx_edid);
			kfree(hdmitx_edid);
		} else {
			DRM_ERROR("hdmitx_get_raw_edid failed\n");
		}
	} else {
		DRM_ERROR("alloc hdmitx_edid block failed\n");
	}
	return count;
}

static enum drm_mode_status
rtk_hdmitx_conn_mode_valid(struct drm_connector *connector,
			   struct drm_display_mode *mode)
{
	return MODE_OK;
}

static const struct drm_connector_funcs rtk_hdmitx_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = rtk_hdmitx_conn_detect,
	.destroy = rtk_hdmitx_conn_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_connector_helper_funcs rtk_hdmitx_connector_helper_funcs = {
	.get_modes = rtk_hdmitx_conn_get_modes,
	.mode_valid = rtk_hdmitx_conn_mode_valid,
};

static int rtk_hdmitx_bind(struct device *dev, struct device *master,
			   void *data)
{
	struct drm_device *drm = data;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct rtk_hdmitx *hdmitx;
	int ret;

	hdmitx = devm_kzalloc(dev, sizeof(*hdmitx), GFP_KERNEL);
	if (!hdmitx)
		return -ENOMEM;

	hdmitx->drm_dev = drm;
	hdmitx->dev = dev;

	encoder = &hdmitx->encoder;
	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	drm_encoder_init(drm, encoder, &rtk_hdmitx_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);

	drm_encoder_helper_add(encoder, &rtk_hdmitx_encoder_helper_funcs);

	connector = &hdmitx->connector;
	connector->polled = DRM_CONNECTOR_POLL_HPD;
	ret = drm_connector_init(drm, connector, &rtk_hdmitx_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		dev_err(dev, "connector_init failed");
		goto err_exit;
	}
	drm_connector_helper_add(connector, &rtk_hdmitx_connector_helper_funcs);

	drm_connector_attach_encoder(connector, encoder);

	return 0;

err_exit:
	return ret;
}

static void rtk_hdmitx_unbind(struct device *dev, struct device *master,
			      void *data)
{

}

static const struct component_ops rtk_hdmitx_ops = {
	.bind	= rtk_hdmitx_bind,
	.unbind	= rtk_hdmitx_unbind,
};

static int rtk_hdmitx_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &rtk_hdmitx_ops);
}

static int rtk_hdmitx_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &rtk_hdmitx_ops);
	return 0;
}

static const struct of_device_id rtk_hdmitx_dt_ids[] = {
	{ .compatible = "realtek,rtk-hdmi-legacy",
	},
	{},
};
MODULE_DEVICE_TABLE(of, rtk_hdmitx_dt_ids);

struct platform_driver rtk_hdmi_legacy_driver = {
	.probe  = rtk_hdmitx_probe,
	.remove = rtk_hdmitx_remove,
	.driver = {
		.name = "rtk-hdmi-legacy",
		.of_match_table = rtk_hdmitx_dt_ids,
	},
};
