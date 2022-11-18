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
#include <drm/drm_print.h>

#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

//#include <linux/mfd/syscon.h>

#include <linux/component.h>
#include <linux/platform_device.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_probe_helper.h>

#include "rtk_dptx_core.h"

#define to_rtk_dptx(x) container_of(x, struct rtk_dptx, x)

static int rtk_dptx_link_start(struct rtk_dptx *dptx)
{
	int lane, lane_count, ret;
	u8 buf[4];

	lane_count = dptx->lt_info.lane_count;

	dptx->lt_info.lt_state = CLOCK_RECOVERY;
	dptx->lt_info.eq_loop = 0;
	for (lane = 0; lane < lane_count; lane++)
		dptx->lt_info.cr_loop[lane] = 0;

	buf[0] = dptx->lt_info.link_rate;
	buf[1] = dptx->lt_info.lane_count | 0x80;

	ret = drm_dp_dpcd_write(&dptx->aux, DP_LINK_BW_SET, buf, 2);
	if (ret < 0)
		return ret;

	dptx_set_training_pattern(dptx, TRAINING_PTN1);

	ret = drm_dp_dpcd_writeb(&dptx->aux, DP_TRAINING_PATTERN_SET,
				 DP_LINK_SCRAMBLING_DISABLE |
				 DP_TRAINING_PATTERN_1);
	if (ret < 0)
		return ret;

	for (lane = 0; lane < lane_count; lane++)
		buf[lane] = DP_TRAIN_PRE_EMPH_LEVEL_0 |
			    DP_TRAIN_VOLTAGE_SWING_LEVEL_0;

	ret = drm_dp_dpcd_write(&dptx->aux, DP_TRAINING_LANE0_SET, buf,
				   lane_count);
	if (ret < 0)
		return ret;

	return 0;
}

static void rtk_dptx_get_adjust_data(struct rtk_dptx *dptx, u8 *request)
{
	int lane, lane_count;
	int shift;
	u8 swing, emphasis, adjust;

	lane_count = dptx->lt_info.lane_count;
	for (lane = 0; lane < lane_count; lane++) {
		shift = (lane & 1) * 4;
		swing = (request[lane >> 1] >> shift) & 0x3;
		emphasis = ((request[lane >> 1] >> shift) & 0xc) >> 2;

		dptx->lt_info.swing[lane] = swing;
		dptx->lt_info.emphasis[lane] = emphasis;

		adjust = VOLTAGE_SWING_SET(swing) |
			 PRE_EMPHASIS_SET(emphasis);
		if (swing == VOLTAGE_LEVEL_3)
			adjust |= DP_TRAIN_MAX_SWING_REACHED;
		if (emphasis == PRE_EMPHASIS_LEVEL_3)
			adjust |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;

		dptx->lt_info.adjust[lane] = adjust;
	}
}

static int rtk_dptx_check_clock_recovery(u8 *data, int lane_count)
{
	int lane, shift;
	u8 status;

	for (lane = 0; lane < lane_count; lane++) {
		shift = (lane & 1) * 4;
		status = (data[lane >> 1] >> shift) & 0xf;
		if ((status & DP_LANE_CR_DONE) == 0)
			return -EINVAL;
	}
	return 0;
}

static int rtk_dptx_check_eqaulizer(u8 *data, u8 align, int lane_count)
{
	int lane, shift;
	u8 status;

	if ((align & DP_INTERLANE_ALIGN_DONE) == 0)
		return -EINVAL;

	for (lane = 0; lane < lane_count; lane++) {
		shift = (lane & 1) * 4;
		status = (data[lane >> 1] >> shift) & 0xf;
		if (status != DP_CHANNEL_EQ_BITS)
			return -EINVAL;
	}
	return 0;
}

static int rtk_dptx_clock_recovery(struct rtk_dptx *dptx)
{
	int lane, lane_count, ret;
	int shift;
	u8 status[2], request[2];
	u8 swing, emphasis;

	mdelay(1);

	lane_count = dptx->lt_info.lane_count;

	ret = drm_dp_dpcd_read(&dptx->aux, DP_LANE0_1_STATUS, status, 2);
	if (ret < 0)
		return ret;

	ret = drm_dp_dpcd_read(&dptx->aux, DP_ADJUST_REQUEST_LANE0_1,
			       request, 2);
	if (ret < 0)
		return ret;

	dev_info(dptx->dev, "dptx - TP1: 0x%x, 0x%x, 0x%x, 0x%x\n",
		status[0], status[1], request[0], request[1]);

	if (rtk_dptx_check_clock_recovery(status, lane_count) == 0) {
		dptx_set_training_pattern(dptx, TRAINING_PTN2);
		ret = drm_dp_dpcd_writeb(&dptx->aux, DP_TRAINING_PATTERN_SET,
					 DP_LINK_SCRAMBLING_DISABLE |
					 DP_TRAINING_PATTERN_2);
		if (ret < 0)
			return ret;
		dptx->lt_info.lt_state = EQUALIZER_TRAINING;
	} else {
		for (lane = 0; lane < lane_count; lane++) {
			shift = (lane & 1) * 4;

			swing = (request[lane >> 1] >> shift) & 0x3;
			emphasis = ((request[lane >> 1] >> shift) & 0xc) >> 2;

			if (dptx->lt_info.swing[lane] == swing &&
			    dptx->lt_info.emphasis[lane] == emphasis)
				dptx->lt_info.cr_loop[lane]++;

			if (dptx->lt_info.cr_loop[lane] == MAX_CR_LOOP) {
				dev_err(dptx->dev, "CR Max reached(%d,%d,%d)\n",
					dptx->lt_info.cr_loop[lane],
					swing, emphasis);
				dptx->lt_info.lt_state = FAILED;
				return -EIO;
			}
		}
	}

	rtk_dptx_get_adjust_data(dptx, request);

	dptx_set_training_lane(dptx);

	ret = drm_dp_dpcd_write(&dptx->aux, DP_TRAINING_LANE0_SET,
				dptx->lt_info.adjust, lane_count);
	if (ret < 0)
		return ret;

	return 0;
}

static int rtk_dptx_equalizer_training(struct rtk_dptx *dptx)
{
	int lane_count, ret;
	u8 align, status[2], request[2];

	mdelay(1);

	lane_count = dptx->lt_info.lane_count;

	ret = drm_dp_dpcd_read(&dptx->aux, DP_LANE0_1_STATUS, status, 2);
	if (ret < 0)
		return ret;

	if (rtk_dptx_check_clock_recovery(status, lane_count)) {
		dptx->lt_info.lt_state = FAILED;
		return -EIO;
	}

	dev_info(dptx->dev, "dptx - TP2: 0x%x, 0x%x, 0x%x, 0x%x\n",
		status[0], status[1], request[0], request[1]);

	ret = drm_dp_dpcd_read(&dptx->aux, DP_ADJUST_REQUEST_LANE0_1,
			       request, 2);
	if (ret < 0)
		return ret;

	ret = drm_dp_dpcd_readb(&dptx->aux, DP_LANE_ALIGN_STATUS_UPDATED,
				&align);
	if (ret < 0)
		return ret;

	rtk_dptx_get_adjust_data(dptx, request);

	if (!rtk_dptx_check_eqaulizer(status, align, lane_count)) {
		dev_info(dptx->dev, "Link Training success!\n");
		drm_dp_dpcd_writeb(&dptx->aux, DP_TRAINING_PATTERN_SET,
				   DP_TRAINING_PATTERN_DISABLE);
		dptx->lt_info.lt_state = FINISHED;
		return 0;
	}

	dptx->lt_info.eq_loop++;

	if (dptx->lt_info.eq_loop > MAX_EQ_LOOP) {
		dev_err(dptx->dev, "EQ MAX loop\n");
		return -EIO;
	}

	dptx_set_training_lane(dptx);

	ret = drm_dp_dpcd_write(&dptx->aux, DP_TRAINING_LANE0_SET,
				dptx->lt_info.adjust, lane_count);
	if (ret < 0)
		return ret;

	return 0;
}

static int rtk_dptx_link_train(struct rtk_dptx *dptx)
{
	int ret = 0, finished = 0;
	u8 data;

	ret = drm_dp_dpcd_readb(&dptx->aux, DP_MAX_LINK_RATE, &data);
	if (ret < 0)
		return ret;

	dptx->lt_info.link_rate = data;

	ret = drm_dp_dpcd_readb(&dptx->aux, DP_MAX_LANE_COUNT, &data);
	if (ret < 0)
		return ret;

	dptx->lt_info.lane_count = data & 0x1f;

	if ((dptx->lt_info.link_rate != DP_LINK_BW_1_62) &&
	    (dptx->lt_info.link_rate != DP_LINK_BW_2_7) &&
	    (dptx->lt_info.link_rate != DP_LINK_BW_5_4)) {
		dev_err(dptx->dev, "Rx Max Link Rate is abnormal :%x !\n",
			dptx->lt_info.link_rate);
		dptx->lt_info.link_rate = DP_LINK_BW_1_62;
	}

	if (dptx->lt_info.lane_count == 0) {
		dev_err(dptx->dev, "Rx Max Lane count is abnormal :%x !\n",
			dptx->lt_info.lane_count);
		dptx->lt_info.lane_count = (u8)LANE_COUNT1;
	}

	if (dptx->lt_info.link_rate > dptx->enc_info.max_link_rate)
		dptx->lt_info.link_rate = dptx->enc_info.max_link_rate;
	if (dptx->lt_info.lane_count > dptx->enc_info.max_lane_count)
		dptx->lt_info.lane_count = dptx->enc_info.max_lane_count;

	ret = 0;
	dptx->lt_info.lt_state = START;
	while (!ret && !finished) {
		switch (dptx->lt_info.lt_state) {
		case START:
			ret = rtk_dptx_link_start(dptx);
			if (ret)
				dev_err(dptx->dev, "LT link start failed\n");
			break;
		case CLOCK_RECOVERY:
			ret = rtk_dptx_clock_recovery(dptx);
			if (ret)
				dev_err(dptx->dev, "LT CR failed\n");
			break;
		case EQUALIZER_TRAINING:
			ret = rtk_dptx_equalizer_training(dptx);
			if (ret)
				dev_err(dptx->dev, "LT EQ failed!\n");
			break;
		case FINISHED:
			finished = 1;
			break;
		case FAILED:
			return -EREMOTEIO;
		}
	}
	if (ret)
		dev_err(dptx->dev, "eDP link training failed (%d)\n", ret);

	return ret;
}

static bool rtk_dptx_enc_mode_fixup(struct drm_encoder *encoder,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adj_mode)
{
	DRM_DEBUG_KMS("%s\n", __func__);
	return true;
}

static void rtk_dptx_enc_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adj_mode)
{
	struct rtk_dptx *dptx = to_rtk_dptx(encoder);
	int vic;

	vic = drm_match_cea_mode(mode);
	DRM_DEBUG_KMS("vid = %d\n", vic);

	dptx_set_video_timing(dptx, adj_mode);
	dptx_set_sst_setting(dptx);
}

static void rtk_dptx_enc_enable(struct drm_encoder *encoder)
{
	struct rtk_dptx *dptx = to_rtk_dptx(encoder);
	struct rpc_config_tv_system arg;
	int ret;

	DRM_DEBUG_KMS("%s\n", __func__);

	ret = rtk_dptx_link_train(dptx);

	rpc_query_tv_system(dptx->rpc_info, &arg);
	arg.videoInfo.pedType = VO_STANDARD_DP_FORMAT_1920_1080P_60;
	arg.interfaceType = VO_DISPLAY_PORT_ONLY;
	arg.videoInfo.standard = VO_STANDARD_NTSC_J;
	rpc_config_tv_system(dptx->rpc_info, &arg);
}

static void rtk_dptx_enc_disable(struct drm_encoder *encoder)
{
	DRM_DEBUG_KMS("%s\n", __func__);
}

static int rtk_dptx_enc_atomic_check(struct drm_encoder *encoder,
				struct drm_crtc_state *crtc_state,
				struct drm_connector_state *conn_state)
{
	DRM_DEBUG_KMS("%s\n", __func__);
	return 0;
}

static const struct drm_encoder_funcs rtk_dptx_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_encoder_helper_funcs rtk_dptx_encoder_helper_funcs = {
	.mode_fixup = rtk_dptx_enc_mode_fixup,
	.mode_set   = rtk_dptx_enc_mode_set,
	.enable     = rtk_dptx_enc_enable,
	.disable    = rtk_dptx_enc_disable,
	.atomic_check = rtk_dptx_enc_atomic_check,
};

static enum drm_connector_status
rtk_dptx_conn_detect(struct drm_connector *connector, bool force)
{
	struct rtk_dptx *dptx = to_rtk_dptx(connector);
	int state;

	state = gpio_get_value(dptx->hpd_gpio);

	return state ?
	       connector_status_connected : connector_status_disconnected;
}

static void rtk_dptx_conn_destroy(struct drm_connector *connector)
{
	DRM_DEBUG_KMS("%s\n", __func__);
}

static int rtk_dptx_conn_get_modes(struct drm_connector *connector)
{
	struct rtk_dptx *dptx = to_rtk_dptx(connector);
	struct edid *edid;
	int num_modes = 0;

	DRM_DEBUG_KMS("%s\n", __func__);

	edid = drm_get_edid(connector, &dptx->aux.ddc);
	if (edid) {
		drm_connector_update_edid_property(&dptx->connector, edid);
		num_modes += drm_add_edid_modes(&dptx->connector, edid);
		kfree(edid);
	}
	return num_modes;
}

static enum drm_mode_status
rtk_dptx_conn_mode_valid(struct drm_connector *connector,
			 struct drm_display_mode *mode)
{
	DRM_DEBUG_KMS("%s\n", __func__);
	return MODE_OK;
}

static const struct drm_connector_funcs rtk_dptx_connector_funcs = {
//	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = rtk_dptx_conn_detect,
	.destroy = rtk_dptx_conn_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_connector_helper_funcs rtk_dptx_connector_helper_funcs = {
	.get_modes = rtk_dptx_conn_get_modes,
	.mode_valid = rtk_dptx_conn_mode_valid,
};

static irqreturn_t rtk_dptx_hpd_irq(int irq, void *dev_id)
{
	struct rtk_dptx *dptx = dev_id;

	drm_helper_hpd_irq_event(dptx->connector.dev);

	return IRQ_HANDLED;
}

static irqreturn_t rtk_dptx_aux_irq(int irq, void *dev_id)
{
	struct rtk_dptx *dptx = dev_id;
	int ret;

	ret = dptx_aux_isr(dptx);
	if (!ret)
		up(&dptx->sem);

	return IRQ_HANDLED;
}

static ssize_t rtk_dptx_aux_transfer(struct drm_dp_aux *aux,
				     struct drm_dp_aux_msg *msg)
{
	struct rtk_dptx *dptx = to_rtk_dptx(aux);

	if (!msg->size && (msg->request & DP_AUX_I2C_READ))
		return 0;

	if (WARN_ON(msg->size > 16))
		return -E2BIG;

	dptx_aux_transfer(dptx, msg);

	if (down_timeout(&dptx->sem, 100)) {
		dev_err(dptx->dev, "aux transfer error\n");
		return -ETIMEDOUT;
	}

	if (msg->request & DP_AUX_I2C_READ)
		dptx_aux_get_data(dptx, msg);

	if ((msg->request & ~DP_AUX_I2C_MOT) == DP_AUX_I2C_WRITE ||
	    (msg->request & ~DP_AUX_I2C_MOT) == DP_AUX_I2C_READ)
		msg->reply = DP_AUX_I2C_REPLY_ACK;
	else if ((msg->request & ~DP_AUX_I2C_MOT) == DP_AUX_NATIVE_WRITE ||
		 (msg->request & ~DP_AUX_I2C_MOT) == DP_AUX_NATIVE_READ)
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;

	return msg->size;
}

static int rtk_dptx_register(struct drm_device *drm, struct rtk_dptx *dptx)
{
	struct drm_encoder *encoder = &dptx->encoder;
	struct drm_connector *connector = &dptx->connector;
	struct device *dev = dptx->dev;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	drm_encoder_init(drm, encoder, &rtk_dptx_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);

	drm_encoder_helper_add(encoder, &rtk_dptx_encoder_helper_funcs);

	connector->polled = DRM_CONNECTOR_POLL_HPD;
	drm_connector_init(drm, connector, &rtk_dptx_connector_funcs,
			   DRM_MODE_CONNECTOR_DisplayPort);
	drm_connector_helper_add(connector, &rtk_dptx_connector_helper_funcs);

	drm_connector_attach_encoder(connector, encoder);

	return 0;
}

static int rtk_dptx_bind(struct device *dev, struct device *master,
				 void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm = data;
	struct rtk_drm_private *priv = drm->dev_private;
	struct rtk_dptx *dptx;
	struct resource *iores;
	struct clk *clk;
	int ret;
	int irq;

	dptx = devm_kzalloc(dev, sizeof(*dptx), GFP_KERNEL);
	if (!dptx)
		return -ENOMEM;

	dptx->drm_dev = drm;
	dptx->dev = dev;

	clk = devm_clk_get(dev, "clk_en_edp");
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get clock\n");
		return PTR_ERR(clk);
	}
	clk_prepare_enable(clk);

	dptx->pllpixel = devm_clk_get(dev, "pllpixel");
	if (IS_ERR(dptx->pllpixel)) {
		dev_err(dev, "failed to get pixel pll\n");
		return PTR_ERR(dptx->pllpixel);
	}
	clk_prepare_enable(dptx->pllpixel);

	dptx->plledp = devm_clk_get(dev, "plledp");
	if (IS_ERR(dptx->plledp)) {
		dev_err(dev, "failed to get edp pll\n");
		return PTR_ERR(dptx->plledp);
	}
	clk_prepare_enable(dptx->plledp);

	dptx->rstc = devm_reset_control_get(dev, "edp");
	if (IS_ERR(dptx->rstc)) {
		dev_err(dev, "failed to get reset controller\n");
		return PTR_ERR(dptx->rstc);
	}
	reset_control_deassert(dptx->rstc);

//	dptx->crt = syscon_regmap_lookup_by_phandle(dev->of_node, "crt");

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dptx->edpreg = devm_ioremap_resource(dev, iores);
	if (IS_ERR(dptx->edpreg))
		return PTR_ERR(dptx->edpreg);

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	dptx->lvdsreg = devm_ioremap_resource(dev, iores);
	if (IS_ERR(dptx->lvdsreg))
		return PTR_ERR(dptx->lvdsreg);

	dptx->hpd_gpio = of_get_named_gpio(dev->of_node, "hpd-gpio", 0);
	if (!gpio_is_valid(dptx->hpd_gpio)) {
		dev_err(dev, "can't get hot plug gpio\n");
		return -ENODEV;
	}
	ret = devm_gpio_request_one(dev, dptx->hpd_gpio, GPIOF_IN, "hpd_gpio");
	if (ret) {
		dev_err(dev, "can't request hot plug gpio\n");
		return -ENODEV;
	}
	gpio_set_debounce(dptx->hpd_gpio, 30*1000);

	irq = gpio_to_irq(dptx->hpd_gpio);
	irq_set_irq_type(irq, IRQ_TYPE_EDGE_BOTH);
	ret = devm_request_irq(dev, irq, rtk_dptx_hpd_irq, IRQF_SHARED,
			       dev_name(dev), dptx);
	if (ret) {
		dev_err(dev, "can't request hpd gpio irq\n");
		return ret;
	}

	sema_init(&dptx->sem, 0);

	dptx->irq = platform_get_irq(pdev, 0);
	if (dptx->irq < 0) {
		dev_err(dev, "can't get aux irq resource\n");
		return -ENODEV;
	}
	ret = devm_request_irq(dev, dptx->irq, rtk_dptx_aux_irq,
			       IRQF_SHARED, dev_name(dev), dptx);
	if (ret) {
		dev_err(dev, "can't request aux irq resource\n");
		return ret;
	}

	dptx->rpc_info = &priv->rpc_info;
	dptx->aux.name = "DP-AUX";
	dptx->aux.transfer = rtk_dptx_aux_transfer;
	dptx->aux.dev = dev;
	ret = drm_dp_aux_register(&dptx->aux);
	if (ret)
		return ret;

	ret = rtk_dptx_register(drm, dptx);
	if (ret)
		return ret;

	dev_set_drvdata(dev, dptx);

	/* fix me */
	dptx->enc_info.max_link_rate = 0x0A;
	dptx->enc_info.max_lane_count = 0x02;

	dptx_init(dptx);

	return 0;
}

static void rtk_dptx_unbind(struct device *dev, struct device *master,
			     void *data)
{

}

static const struct component_ops rtk_dptx_ops = {
	.bind	= rtk_dptx_bind,
	.unbind	= rtk_dptx_unbind,
};

static int rtk_dptx_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &rtk_dptx_ops);
}

static int rtk_dptx_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &rtk_dptx_ops);
	return 0;
}

static const struct of_device_id rtk_dptx_dt_ids[] = {
	{ .compatible = "realtek,rtk-dptx",
	},
	{},
};
MODULE_DEVICE_TABLE(of, rtk_dptx_dt_ids);

struct platform_driver rtk_dptx_driver = {
	.probe  = rtk_dptx_probe,
	.remove = rtk_dptx_remove,
	.driver = {
		.name = "rtk-dptx",
		.of_match_table = rtk_dptx_dt_ids,
	},
};

MODULE_AUTHOR("Simon Hsu <simon_hsu@realtek.com>");
MODULE_DESCRIPTION("Realtek DP Core Driver");
MODULE_LICENSE("GPL v2");
