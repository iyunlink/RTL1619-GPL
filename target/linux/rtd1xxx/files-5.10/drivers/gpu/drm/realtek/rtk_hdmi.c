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

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/component.h>
#include <linux/gpio/consumer.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#include "rtk_hdmi.h"
#include "rtk_hdmi_reg.h"
#include "rtk_crt_reg.h"

#define to_rtk_hdmi(x) container_of(x, struct rtk_hdmi, x)
#define I2C_BUS_ID 1

enum VIDEO_ID_CODE {
	VIC_720X480P60 = 2,
	VIC_1280X720P60 = 4,
	VIC_1920X1080I60 = 5,
	VIC_720X480I60 = 6,
	VIC_1920X1080P60 = 16,
	VIC_720X576P50 = 17,
	VIC_1280X720P50 = 19,
	VIC_1920X1080I50 = 20,
	VIC_720X576I50 = 21,
	VIC_1920X1080P50 = 31,
	VIC_1920X1080P24 = 32,
	VIC_1920X1080P25 = 33,
	VIC_1920X1080P30 = 34,
	VIC_1280X720P24 = 60,
	VIC_1280X720P25 = 61,
	VIC_1280X720P30 = 62,
	VIC_1920X1080P120 = 63,
	VIC_3840X2160P24 = 93,
	VIC_3840X2160P25 = 94,
	VIC_3840X2160P30 = 95,
	VIC_3840X2160P50 = 96,
	VIC_3840X2160P60 = 97,
	VIC_4096X2160P24 = 98,
	VIC_4096X2160P25 = 99,
	VIC_4096X2160P30 = 100,
	VIC_4096X2160P50 = 101,
	VIC_4096X2160P60 = 102,
};

static const struct drm_prop_enum_list colorspace_mode_list[] = {
	{ HDMI_COLORSPACE_RGB, "RGB" },
	{ HDMI_COLORSPACE_YUV422, "Y422" },
	{ HDMI_COLORSPACE_YUV444, "Y444" },
	{ HDMI_COLORSPACE_YUV420, "Y420" },
};

static const struct drm_prop_enum_list hdr_mode_list[] = {
	{ HDR_CTRL_AUTO, "AUTO" },
	{ HDR_CTRL_DV_ON, "DV_ON" },
	{ HDR_CTRL_SDR, "SDR" },
	{ HDR_CTRL_HDR_GAMMA, "HDR_GAMMA" },
	{ HDR_CTRL_PQHDR, "PQHDR" },
	{ HDR_CTRL_FUTURE, "FUTURE" },
	{ HDR_CTRL_INPUT, "INPUT" },
	{ HDR_CTRL_DV_LOW_LATENCY_12b_YUV422, "DV_LL_12b_Y422" },
	{ HDR_CTRL_DV_LOW_LATENCY_10b_YUV444, "DV_LL_10b_Y444" },
	{ HDR_CTRL_DV_LOW_LATENCY_10b_RGB444, "DV_LL_10b_RGB" },
	{ HDR_CTRL_DV_LOW_LATENCY_12b_YUV444, "DV_LL_12b_Y444" },
	{ HDR_CTRL_DV_LOW_LATENCY_12b_RGB444, "DV_LL_12b_RGB" },
	{ HDR_CTRL_DV_ON_INPUT, "DV_ON_INPUT" },
	{ HDR_CTRL_DV_ON_LOW_LATENCY_12b422_INPUT, "DV_LL_12b422_INPUT" },
	{ HDR_CTRL_INPUT_BT2020, "INPUT_BT2020" },
};

static const struct drm_prop_enum_list fractional_fps_status_list[] = {
	{ FRACTIONAL_FPS_DISABLE, "Disable" },
	{ FRACTIONAL_FPS_ENABLE, "Enable" },
};

static const struct drm_prop_enum_list allm_status_list[] = {
	{ ALLM_UNSUPPORTED, "Unsupported" },
	{ ALLM_DISABLE, "Disable" },
	{ ALLM_ENABLE, "Enable" },
};

static const struct drm_prop_enum_list qms_status_list[] = {
	{ QMS_VRR_UNSUPPORTED, "Unsupported" },
	{ QMS_VRR_DISABLE, "Disable" },
	{ QMS_VRR_EN_VRR, "Enable_VRR" },
	{ QMS_VRR_EN_QMS, "Enable_QMS" },
};

static const struct drm_prop_enum_list vrr_rate_list[] = {
	{ RATE_UNSPECIFIED, "UNSPECIFIED" },
	{ RATE_23HZ, "RATE_23HZ" },
	{ RATE_24HZ, "RATE_24HZ" },
	{ RATE_25HZ, "RATE_25HZ" },
	{ RATE_29HZ, "RATE_29HZ" },
	{ RATE_30HZ, "RATE_30HZ" },
	{ RATE_47HZ, "RATE_47HZ" },
	{ RATE_48HZ, "RATE_48HZ" },
	{ RATE_50HZ, "RATE_50HZ" },
	{ RATE_59HZ, "RATE_59HZ" },
	{ RATE_60HZ, "RATE_60HZ" },
	{ RATE_BASE, "RATE_BASE" },
};

static bool rtk_hdmi_valid_vic(u8 vic)
{
	bool valid;

	switch (vic) {
	case VIC_720X480P60:
	case VIC_1280X720P60:
	case VIC_1920X1080I60:
	case VIC_720X480I60:
	case VIC_1920X1080P60:
	case VIC_720X576P50:
	case VIC_1280X720P50:
	case VIC_1920X1080I50:
	case VIC_720X576I50:
	case VIC_1920X1080P50:
	case VIC_1920X1080P24:
	case VIC_1920X1080P25:
	case VIC_1920X1080P30:
	case VIC_1280X720P24:
	case VIC_1280X720P25:
	case VIC_1280X720P30:
	case VIC_1920X1080P120:
	case VIC_3840X2160P24:
	case VIC_3840X2160P25:
	case VIC_3840X2160P30:
	case VIC_3840X2160P50:
	case VIC_3840X2160P60:
	case VIC_4096X2160P24:
	case VIC_4096X2160P25:
	case VIC_4096X2160P30:
	case VIC_4096X2160P50:
	case VIC_4096X2160P60:
		valid = true;
		break;
	default:
		valid = false;
	}

	return valid;
}

static unsigned int
vic_to_vo_standard(unsigned char vic, struct rtk_hdmi *hdmi)
{
	unsigned int ret_val;
	enum hdmi_colorspace rgb_or_yuv;
	enum FRACTIONAL_FPS_STATUS en_fractional_fps;
	u8 requested_bpc;

	requested_bpc = hdmi->connector.state->max_requested_bpc;
	rgb_or_yuv = hdmi->rgb_or_yuv;
	en_fractional_fps = hdmi->en_fractional_fps;

	switch (vic) {
	case VIC_720X480P60:
		ret_val = VO_STANDARD_NTSC_J;
		break;
	case VIC_1280X720P60:
		if (en_fractional_fps)
			ret_val = VO_STANDARD_HDTV_720P_59;
		else
			ret_val = VO_STANDARD_HDTV_720P_60;
		break;
	case VIC_1920X1080I60:
		if (en_fractional_fps)
			ret_val = VO_STANDARD_HDTV_1080I_59;
		else
			ret_val = VO_STANDARD_HDTV_1080I_60;
		break;
	case VIC_720X480I60:
		ret_val = VO_STANDARD_NTSC_J;
		break;
	case VIC_1920X1080P60:
		if (en_fractional_fps)
			ret_val = VO_STANDARD_HDTV_1080P_59;
		else
			ret_val = VO_STANDARD_HDTV_1080P_60;
		break;
	case VIC_720X576P50:
		ret_val = VO_STANDARD_PAL_I;
		break;
	case VIC_1280X720P50:
		ret_val = VO_STANDARD_HDTV_720P_50;
		break;
	case VIC_1920X1080I50:
		ret_val = VO_STANDARD_HDTV_1080I_50;
		break;
	case VIC_720X576I50:
		ret_val = VO_STANDARD_PAL_I;
		break;
	case VIC_1920X1080P50:
		ret_val = VO_STANDARD_HDTV_1080P_50;
		break;
	case VIC_1920X1080P24:
		if (en_fractional_fps)
			ret_val = VO_STANDARD_HDTV_1080P_23;
		else
			ret_val = VO_STANDARD_HDTV_1080P_24;
		break;
	case VIC_1920X1080P25:
		ret_val = VO_STANDARD_HDTV_1080P_25;
		break;
	case VIC_1920X1080P30:
		if (en_fractional_fps)
			ret_val = VO_STANDARD_HDTV_1080P_29;
		else
			ret_val = VO_STANDARD_HDTV_1080P_30;
		break;
	case VIC_1280X720P24:
		if (en_fractional_fps)
			ret_val = VO_STANDARD_HDTV_720P_P23;
		else
			ret_val = VO_STANDARD_HDTV_720P_P24;
		break;
	case VIC_1280X720P25:
		ret_val = VO_STANDARD_HDTV_720P_P25;
		break;
	case VIC_1280X720P30:
		if (en_fractional_fps)
			ret_val = VO_STANDARD_HDTV_720P_P29;
		else
			ret_val = VO_STANDARD_HDTV_720P_P30;
		break;
	case VIC_1920X1080P120:
		ret_val = VO_STANDARD_HDTV_1080P_120;
		break;
	case VIC_3840X2160P24:
		if (en_fractional_fps)
			ret_val = VO_STANDARD_HDTV_2160P_23;
		else
			ret_val = VO_STANDARD_HDTV_2160P_24;
		break;
	case VIC_3840X2160P25:
		ret_val = VO_STANDARD_HDTV_2160P_25;
		break;
	case VIC_3840X2160P30:
		if (en_fractional_fps)
			ret_val = VO_STANDARD_HDTV_2160P_29;
		else
			ret_val = VO_STANDARD_HDTV_2160P_30;
		break;
	case VIC_3840X2160P50:
		if (rgb_or_yuv == HDMI_COLORSPACE_YUV420)
			ret_val = VO_STANDARD_HDTV_2160P_50_420;
		else if ((rgb_or_yuv == HDMI_COLORSPACE_YUV422) &&
					(requested_bpc >= 10))
			ret_val = VO_STANDARD_HDTV_2160P_50_422_12bit;
		else
			ret_val = VO_STANDARD_HDTV_2160P_50;
		break;
	case VIC_3840X2160P60:
		if (rgb_or_yuv == HDMI_COLORSPACE_YUV420) {
			if (en_fractional_fps)
				ret_val = VO_STANDARD_HDTV_2160P_59_420;
			else
				ret_val = VO_STANDARD_HDTV_2160P_60_420;
		} else if ((rgb_or_yuv == HDMI_COLORSPACE_YUV422)
					&& (requested_bpc >= 10)) {
			if (en_fractional_fps)
				ret_val = VO_STANDARD_HDTV_2160P_59_422_12bit;
			else
				ret_val = VO_STANDARD_HDTV_2160P_60_422_12bit;
		} else {
			if (en_fractional_fps)
				ret_val = VO_STANDARD_HDTV_2160P_59;
			else
				ret_val = VO_STANDARD_HDTV_2160P_60;
		}
		break;
	case VIC_4096X2160P24:
		ret_val = VO_STANDARD_HDTV_4096_2160P_24;
		break;
	case VIC_4096X2160P25:
		ret_val = VO_STANDARD_HDTV_4096_2160P_25;
		break;
	case VIC_4096X2160P30:
		ret_val = VO_STANDARD_HDTV_4096_2160P_30;
		break;
	case VIC_4096X2160P50:
		ret_val = VO_STANDARD_HDTV_4096_2160P_50;
		break;
	case VIC_4096X2160P60:
		ret_val = VO_STANDARD_HDTV_4096_2160P_60;
		break;
	default:
		ret_val = VO_STANDARD_HDTV_1080P_60;
		break;
	} /* end of switch (vic) */

	return ret_val;
}

static bool is_hdmi_clock_on(struct rtk_hdmi *hdmi)
{
	unsigned int pll_hdmi;
	bool clk_on;

	clk_on = false;

	if (__clk_is_enabled(hdmi->clk_hdmi) &&
		(!reset_control_status(hdmi->reset_hdmi))) {
		regmap_read(hdmi->crtreg, SYS_PLL_HDMI, &pll_hdmi);
		if ((pll_hdmi&0xBF) == 0xBF)
			clk_on = true;
	}

	return clk_on;
}

static bool send_scdc_tmdsconfig(struct rtk_hdmi *hdmi,
	struct drm_display_mode *mode, u8 bpc)
{
	struct drm_scdc *scdc = &hdmi->connector.display_info.hdmi.scdc;
	int clock;
	bool clock_ratio = false;
	bool en_scramb = false;

	clock = mode->clock;

	if (hdmi->rgb_or_yuv == HDMI_COLORSPACE_YUV420)
		clock = clock/2;

	if (hdmi->rgb_or_yuv != HDMI_COLORSPACE_YUV422) {
		if (bpc == 12)
			clock += clock/4;
		else if (bpc == 10)
			clock += clock/2;
	}

	if (clock > 340000) {
		clock_ratio = true;
		en_scramb = true;
	} else if (scdc->scrambling.low_rates &&
			(hdmi->rgb_or_yuv == HDMI_COLORSPACE_YUV420)) {
		en_scramb = true;
	}

	dev_info(hdmi->dev, "clock=%d, edid_scdc=%u, clock_ratio=%u, en_scramb=%u",
		clock, scdc->supported, clock_ratio, en_scramb);

	if (en_scramb || scdc->supported) {
		drm_scdc_set_high_tmds_clock_ratio(hdmi->ddc, clock_ratio);
		drm_scdc_set_scrambling(hdmi->ddc, en_scramb);
	}

	return en_scramb;
}

static void rtk_hdmi_send_avmute(struct rtk_hdmi *hdmi, unsigned char mute)
{
	if (is_hdmi_clock_on(hdmi)) {
		regmap_write(hdmi->hdmireg, HDMI_GCPCR,
			HDMI_GCPCR_enablegcp(1) |
			HDMI_GCPCR_gcp_clearavmute(1) |
			HDMI_GCPCR_gcp_setavmute(1) |
			HDMI_GCPCR_write_data(0));

		regmap_write(hdmi->hdmireg, HDMI_GCPCR,
			HDMI_GCPCR_enablegcp(1) |
			HDMI_GCPCR_gcp_clearavmute(!mute) |
			HDMI_GCPCR_gcp_setavmute(mute) |
			HDMI_GCPCR_write_data(1));
	}
	dev_info(hdmi->dev, "%s AVmute", mute ? "Set":"Clear");
}

static int rtk_hdmi_get_rxsense(struct rtk_hdmi *hdmi)
{
	unsigned int reg_val;
	int rxsense;

	switch(hdmi->rxsense_mode) {
	case RXSENSE_PASSIVE_MODE:
		if (is_hdmi_clock_on(hdmi)) {
			regmap_read(hdmi->hdmireg, HDMI_PHY_STATUS, &reg_val);
			rxsense = HDMI_PHY_STATUS_get_Rxstatus(reg_val);
		} else {
			rxsense = HDMI_RXSENSE_UNKNOWN;
		}
		break;
	case RXSENSE_TIMER_MODE:
	case RXSENSE_INTERRUPT_MODE:
		regmap_read(hdmi->topreg, RXST, &reg_val);
		rxsense = RXST_get_Rxstatus(reg_val);
		break;
	default:
		rxsense = HDMI_RXSENSE_UNKNOWN;
	}

	return rxsense;
}

static void rtk_hdmi_fill_hdr_data(struct rtk_hdmi *hdmi, struct edid *edid)
{
	struct rpc_vout_edid_data data;
	struct hdr_static_metadata *p_metadata;

	memset(&data, 0, sizeof(data));

	p_metadata = &hdmi->connector.hdr_sink_metadata.hdmi_type1;

	data.et = p_metadata->eotf;
	data.sm = p_metadata->metadata_type;
	data.max_luminace = p_metadata->max_cll;
	data.max_frame_avg = p_metadata->max_fall;
	data.min_luminace = p_metadata->min_cll;

	if (data.min_luminace != 0)
		data.metadata_len = 5;
	else if (data.max_frame_avg != 0)
		data.metadata_len = 4;
	else if (data.max_luminace != 0)
		data.metadata_len = 3;
	else
		data.metadata_len = 2;

	data.red_green_lo = edid->red_green_lo;
	data.black_white_lo = edid->black_white_lo;
	data.red_x = edid->red_x;
	data.red_y = edid->red_y;
	data.green_x = edid->green_x;
	data.green_y = edid->green_y;
	data.blue_x = edid->blue_x;
	data.blue_y = edid->blue_y;
	data.white_x = edid->white_x;
	data.white_y = edid->white_y;

	data.color_space = hdmi->edid_info.colorimetry;
	data.vcdb = hdmi->edid_info.vcdb;
	data.max_frl_rate = hdmi->edid_info.max_frl_rate;

	if (hdmi->edid_info.scds_pb5 & SCDS_QMS)
		data.vrr_feature |= FEATURE_QMS;

	if (hdmi->edid_info.scds_pb8& SCDS_QMS_TFR_MIN)
		data.vrr_feature |= FEATURE_QMS_TFRMIN;

	if (hdmi->edid_info.scds_pb8& SCDS_QMS_TFR_MAX)
		data.vrr_feature |= FEATURE_QMS_TFRMAX;

	data.vrr_max98_min = ((hdmi->edid_info.vrr_max >> 2) & 0xC0) |
		hdmi->edid_info.vrr_min;
	data.vrr_max = hdmi->edid_info.vrr_max & 0xFF;

	rpc_send_vout_edid_data(hdmi->rpc_info, &data);
}

static void
rtk_hdmi_fill_audio_data(struct rtk_hdmi *hdmi, struct edid *edid)
{
	struct rpc_audio_edid_data rpc_data;
	struct audio_edid_data *audio_db;
	int i;
	int sad_count;
	struct cea_sad *sads;

	rpc_data.version = 2;

	if (IS_ERR_OR_NULL(edid)) {
		rpc_data.hdmi_en_state = 0;
		goto send_rpc;
	}

	rpc_data.hdmi_en_state = 1;

	audio_db = (struct audio_edid_data *)hdmi->audio_edid_buf.vaddr;

	sad_count = drm_edid_to_sad(edid, &sads);
	if (sad_count <= 0) {
		audio_db->data_size = 1;
	} else {
		if (sad_count > MAX_SAD_SIZE)
			sad_count = MAX_SAD_SIZE;

		audio_db->data_size = sad_count*3 + 1;

		for (i = 0; i < sad_count; i++) {
			audio_db->sad[i].byte0 = (sads[i].format << 3) | sads[i].channels;
			audio_db->sad[i].byte1 = sads[i].freq;
			audio_db->sad[i].byte2 = sads[i].byte2;
		}
		kfree(sads);
	}

	rpc_data.edid_data_addr = (uint32_t)hdmi->audio_edid_buf.paddr;

send_rpc:
	rpc_send_audio_edid_data(hdmi->rpc_info, &rpc_data);
}

static u8 rtk_hdmi_get_bpc(struct rtk_hdmi *hdmi,
			struct drm_display_mode *mode)
{
	u8 requested_bpc;
	struct drm_display_info *info;

	info = &hdmi->connector.display_info;

	requested_bpc = hdmi->connector.state->max_requested_bpc;

	if ((mode->clock >= 594000) &&
		(hdmi->rgb_or_yuv != HDMI_COLORSPACE_YUV422) &&
		(hdmi->rgb_or_yuv != HDMI_COLORSPACE_YUV420))
		return 8;

	// TODO: Consider max TMDS clock of EDID and frequency doubling of deep color

	if (requested_bpc >= 12) {
		if (hdmi->rgb_or_yuv == HDMI_COLORSPACE_YUV422)
			return 12;

		if ((hdmi->rgb_or_yuv == HDMI_COLORSPACE_YUV420) &&
			(info->hdmi.y420_dc_modes & DRM_EDID_YCBCR420_DC_36))
			return 12;

		if (info->edid_hdmi_dc_modes & DRM_EDID_HDMI_DC_36)
			return 12;
	}

	if (requested_bpc >= 10) {
		if (hdmi->rgb_or_yuv == HDMI_COLORSPACE_YUV422)
			return 10;

		if ((hdmi->rgb_or_yuv == HDMI_COLORSPACE_YUV420) &&
			(info->hdmi.y420_dc_modes & DRM_EDID_YCBCR420_DC_30))
			return 10;

		if (info->edid_hdmi_dc_modes & DRM_EDID_HDMI_DC_30)
			return 10;
	}

	return 8;
}

static int rtk_hdmi_set_allm(struct rtk_hdmi *hdmi, enum ALLM_STATUS en_allm)
{
	int ret;
	struct rpc_vout_hdmi_vrr arg;

	memset(&arg, 0, sizeof(arg));
	arg.vrr_function = HDMI_ALLM_ON_OFF;

	if (en_allm == ALLM_ENABLE)
		arg.vrr_act = HDMI_ALLM_ENABLE;
	else
		arg.vrr_act = HDMI_ALLM_DISABLE;

	ret = rpc_set_vrr(hdmi->rpc_info, &arg);

	return ret;
}

static int rtk_hdmi_set_qms(struct rtk_hdmi *hdmi,
	struct drm_connector_state *state, enum QMS_VRR_STATUS en_qms)
{
	int ret;
	int vrefresh;
	int vtotal;
	struct rpc_vout_hdmi_vrr arg;

	ret = -ENOEXEC;

	if (state->crtc == NULL)
		goto exit;

	if ((en_qms == QMS_VRR_EN_VRR) || (en_qms == QMS_VRR_EN_QMS)) {
		vrefresh = (state->crtc->mode.clock * 1000) /
				(state->crtc->mode.vtotal * state->crtc->mode.htotal);
		vtotal = state->crtc->mode.vtotal;
		if ((vrefresh != 60) || (vtotal < 1080))
			goto exit;
	}

	memset(&arg, 0, sizeof(arg));
	arg.vrr_function = HDMI_VRR_ON_OFF;
	arg.vrr_act = en_qms;

	ret = rpc_set_vrr(hdmi->rpc_info, &arg);

	if (!ret) {
		if ((en_qms == QMS_VRR_EN_VRR) || (en_qms == QMS_VRR_EN_QMS))
			hdmi->vrr_rate = RATE_BASE;
		else
			hdmi->vrr_rate = RATE_UNSPECIFIED;
	}
exit:
	return ret;
}

static int rtk_hdmi_set_vrr_rate(struct rtk_hdmi *hdmi,
			enum QMS_VRR_RATE vrr_rate)
{
	int ret;
	struct rpc_vout_hdmi_vrr arg;

	ret = -ENOEXEC;

	if ((hdmi->en_qms_vrr != QMS_VRR_EN_VRR) &&
		(hdmi->en_qms_vrr != QMS_VRR_EN_QMS))
		goto exit;

	memset(&arg, 0, sizeof(arg));
	arg.vrr_function = HDMI_VRR_TARGET_RATE;
	arg.vrr_act = vrr_rate;

	ret = rpc_set_vrr(hdmi->rpc_info, &arg);
exit:
	return ret;
}

static void hdmitx_set_video_timing(struct drm_encoder *encoder,
	struct drm_display_mode *mode)
{
	int err;
	u8 vic;
	struct rtk_hdmi *hdmi = to_rtk_hdmi(encoder);
	struct hdmi_avi_infoframe infoframe;
	struct rpc_config_tv_system arg;
	struct rpc_audio_hdmi_freq audio_arg;
	unsigned int dataint0;
	u8 bpc;
	u8 avi_buffer[HDMI_INFOFRAME_SIZE(AVI)];
	bool en_scramb;

	rpc_query_tv_system(hdmi->rpc_info, &arg);

	vic = drm_match_cea_mode(mode);
	bpc = rtk_hdmi_get_bpc(hdmi, mode);

	dev_info(hdmi->dev, "vic=%u %s-%ubit %s",
		vic,
		colorspace_mode_list[hdmi->rgb_or_yuv].name,
		bpc,
		hdr_mode_list[hdmi->hdr_mode].name);

	if (!rtk_hdmi_valid_vic(vic)) {
		dev_err(hdmi->dev, "Invalid vic %u", vic);
		return;
	}

	err = drm_hdmi_avi_infoframe_from_display_mode(&infoframe,
			&hdmi->connector, mode);
	if (err < 0) {
		dev_err(hdmi->dev, "Failed to fill avi infoframe");
		return;
	}

	infoframe.colorspace = hdmi->rgb_or_yuv;
	err = hdmi_avi_infoframe_pack(&infoframe, avi_buffer,
					HDMI_INFOFRAME_SIZE(AVI));
	if (err < 0) {
		dev_err(hdmi->dev, "Failed to pack avi infoframe");
		return;
	}

	dataint0 = arg.info_frame.dataInt0&(~RPC_BPC_MASK);
	if (bpc >= 12)
		dataint0 |= 0x1A;
	else if (bpc >= 10)
		dataint0 |= 0x16;
	else
		dataint0 |= 0x10;
	arg.info_frame.dataInt0 = dataint0;

	arg.videoInfo.standard = vic_to_vo_standard(vic, hdmi);
	arg.videoInfo.enProg = !(mode->flags & DRM_MODE_FLAG_INTERLACE);
	arg.videoInfo.enDIF = 0x1;
	arg.videoInfo.enCompRGB = 0;
	arg.videoInfo.pedType = VO_PEDESTAL_TYPE_300_700_OFF;

	if (hdmi->edid_info.sink_is_hdmi)
		arg.info_frame.hdmiMode = VO_HDMI_ON;
	else
		arg.info_frame.hdmiMode = VO_DVI_ON;

	arg.info_frame.audioSampleFreq = VO_HDMI_AUDIO_48K;
	arg.info_frame.audioChannelCount = 0x1;

	arg.info_frame.dataByte1 = avi_buffer[HDMI_INFOFRAME_HEADER_SIZE];
	arg.info_frame.dataByte2 = avi_buffer[HDMI_INFOFRAME_HEADER_SIZE+1];
	arg.info_frame.dataByte3 = avi_buffer[HDMI_INFOFRAME_HEADER_SIZE+2];
	arg.info_frame.dataByte4 = avi_buffer[HDMI_INFOFRAME_HEADER_SIZE+3];
	arg.info_frame.dataByte5 = avi_buffer[HDMI_INFOFRAME_HEADER_SIZE+4];

	arg.info_frame.hdr_ctrl_mode = hdmi->hdr_mode;

	en_scramb = send_scdc_tmdsconfig(hdmi, mode, bpc);
	if (en_scramb)
		arg.info_frame.hdmi2px_feature |= HDMI2PX_SCRAMBLE;

	rpc_config_tv_system(hdmi->rpc_info, &arg);

	audio_arg.tmds_freq = mode->clock/1000;
	rpc_send_hdmi_freq(hdmi->rpc_info, &audio_arg);
}

static int rtk_hdmi_off(struct rtk_hdmi *hdmi)
{
	int ret;
	struct rpc_config_tv_system arg;

	memset(&arg, 0, sizeof(arg));

	arg.info_frame.hdmiMode = VO_HDMI_OFF;
	arg.videoInfo.standard = VO_STANDARD_NTSC_J;
	arg.videoInfo.enProg = 0;
	arg.videoInfo.pedType = 0x1;
	arg.videoInfo.dataInt0 = 0x4;

	ret = rpc_config_tv_system(hdmi->rpc_info, &arg);

	if (hdmi->edid_info.scds_pb5 & SCDS_ALLM)
		hdmi->en_allm = ALLM_DISABLE;
	else
		hdmi->en_allm = ALLM_UNSUPPORTED;

	if ((hdmi->edid_info.vrr_min != 0) ||
		(hdmi->edid_info.scds_pb5 & SCDS_QMS))
		hdmi->en_qms_vrr = QMS_VRR_DISABLE;
	else
		hdmi->en_qms_vrr = QMS_VRR_UNSUPPORTED;

	hdmi->vrr_rate = RATE_UNSPECIFIED;

	return ret;
}

static int rtk_hdmi_init_properties(struct rtk_hdmi *hdmi)
{
	int ret;
	struct drm_property *prop;

	if (hdmi->connector.funcs->reset)
			hdmi->connector.funcs->reset(&hdmi->connector);

	ret = -ENOMEM;

	hdmi->rgb_or_yuv = HDMI_COLORSPACE_RGB;
	prop = drm_property_create_enum(hdmi->drm_dev, 0, "RGB or YCbCr",
				colorspace_mode_list,
				ARRAY_SIZE(colorspace_mode_list));
	if (!prop) {
		dev_err(hdmi->dev, "create colorspace enum property failed");
		goto exit;
	}
	hdmi->rgb_or_yuv_property = prop;
	drm_object_attach_property(&hdmi->connector.base, prop, hdmi->rgb_or_yuv);

	hdmi->hdr_mode = HDR_CTRL_SDR;
	prop = drm_property_create_enum(hdmi->drm_dev, 0, "HDR mode",
				hdr_mode_list,
				ARRAY_SIZE(hdr_mode_list));
	if (!prop) {
		dev_err(hdmi->dev, "create hdr_mode enum property failed");
		goto exit;
	}
	hdmi->hdr_mode_property = prop;
	drm_object_attach_property(&hdmi->connector.base, prop, hdmi->hdr_mode);

	hdmi->en_fractional_fps = FRACTIONAL_FPS_DISABLE;
	prop = drm_property_create_enum(hdmi->drm_dev, 0, "fractional fps",
				fractional_fps_status_list,
				ARRAY_SIZE(fractional_fps_status_list));
	if (!prop) {
		dev_err(hdmi->dev, "create en_fractional_fps enum property failed");
		goto exit;
	}
	hdmi->fractional_fps_property = prop;
	drm_object_attach_property(&hdmi->connector.base, prop,
		hdmi->en_fractional_fps);

	hdmi->en_allm = ALLM_UNSUPPORTED;
	prop = drm_property_create_enum(hdmi->drm_dev, 0, "allm",
				allm_status_list,
				ARRAY_SIZE(allm_status_list));
	if (!prop) {
		dev_err(hdmi->dev, "create en_allm enum property failed");
		goto exit;
	}
	hdmi->allm_property = prop;
	drm_object_attach_property(&hdmi->connector.base, prop,
		hdmi->en_allm);

	hdmi->en_qms_vrr = QMS_VRR_UNSUPPORTED;
	prop = drm_property_create_enum(hdmi->drm_dev, 0, "qms_vrr",
				qms_status_list,
				ARRAY_SIZE(qms_status_list));
	if (!prop) {
		dev_err(hdmi->dev, "create qms_vrr enum property failed");
		goto exit;
	}
	hdmi->qms_property = prop;
	drm_object_attach_property(&hdmi->connector.base, prop,
		hdmi->en_qms_vrr);

	hdmi->vrr_rate = RATE_UNSPECIFIED;
	prop = drm_property_create_enum(hdmi->drm_dev, 0, "vrr_rate",
				vrr_rate_list,
				ARRAY_SIZE(vrr_rate_list));
	if (!prop) {
		dev_err(hdmi->dev, "create vrr_rate enum property failed");
		goto exit;
	}
	hdmi->vrr_rate_property = prop;
	drm_object_attach_property(&hdmi->connector.base, prop,
		hdmi->vrr_rate);

	ret = drm_connector_attach_max_bpc_property(&hdmi->connector, 8, 12);

exit:
	return ret;
}

int rtk_hdmi_conn_set_property(struct drm_connector *connector,
				struct drm_connector_state *state,
				struct drm_property *property,
				uint64_t val)
{
	int ret;
	struct rtk_hdmi *hdmi = to_rtk_hdmi(connector);

	ret = -EINVAL;
	if (property == hdmi->rgb_or_yuv_property) {
		hdmi->rgb_or_yuv = val;
		ret = 0;
	} else if (property == hdmi->hdr_mode_property) {
		hdmi->hdr_mode = val;
		ret = 0;
	} else if (property == hdmi->fractional_fps_property) {
		hdmi->en_fractional_fps = val;
		ret = 0;
	} else if ((property == hdmi->allm_property) &&
				(val != ALLM_UNSUPPORTED)) {
		ret = rtk_hdmi_set_allm(hdmi, val);
		DRM_DEBUG_KMS("allm_property val=%llu ret=%d\n", val, ret);
		if (!ret)
			hdmi->en_allm = val;
	} else if ((property == hdmi->qms_property) &&
				(val != hdmi->en_qms_vrr) &&
				(val != QMS_VRR_UNSUPPORTED)) {
		ret = rtk_hdmi_set_qms(hdmi, state, val);
		DRM_DEBUG_KMS("qms_property val=%llu ret=%d\n", val, ret);
		if (!ret)
			hdmi->en_qms_vrr = val;
	} else if ((property == hdmi->vrr_rate_property) &&
				(val != hdmi->vrr_rate) &&
				(val != RATE_UNSPECIFIED) && (val != RATE_BASE)) {
		ret = rtk_hdmi_set_vrr_rate(hdmi, val);
		DRM_DEBUG_KMS("vrr_rate_property val=%llu ret=%d\n", val, ret);
		if (!ret)
			hdmi->vrr_rate = val;
	} else if (property == hdmi->hdcp.hdcp14_timeout_property) {
		hdmi->hdcp.hdcp14_timeout = val;
		ret = 0;
	}

	return ret;
}

int rtk_hdmi_conn_get_property(struct drm_connector *connector,
				const struct drm_connector_state *state,
				struct drm_property *property,
				uint64_t *val)
{
	struct rtk_hdmi *hdmi = to_rtk_hdmi(connector);

	if (property == hdmi->rgb_or_yuv_property) {
		*val = hdmi->rgb_or_yuv;
		return 0;
	} else if (property == hdmi->hdr_mode_property) {
		*val = hdmi->hdr_mode;
		return 0;
	} else if (property == hdmi->fractional_fps_property) {
		*val = hdmi->en_fractional_fps;
		return 0;
	} else if (property == hdmi->allm_property) {
		*val = hdmi->en_allm;
		return 0;
	} else if (property == hdmi->qms_property) {
		*val = hdmi->en_qms_vrr;
		return 0;
	} else if (property == hdmi->vrr_rate_property) {
		*val = hdmi->vrr_rate;
		return 0;
	} else if (property == hdmi->hdcp.hdcp14_timeout_property) {
		*val = hdmi->hdcp.hdcp14_timeout;
		return 0;
	}

	return -EINVAL;
}

static bool rtk_hdmi_enc_mode_fixup(struct drm_encoder *encoder,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adj_mode)
{
	DRM_DEBUG_KMS("%s\n", __func__);
	return true;
}

static void rtk_hdmi_enc_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adj_mode)
{
	struct rtk_hdmi *hdmi = to_rtk_hdmi(encoder);

	dev_dbg(hdmi->dev, "%s\n", __func__);

	hdmitx_set_video_timing(encoder, adj_mode);
}

static void rtk_hdmi_enc_enable(struct drm_encoder *encoder)
{
	struct rtk_hdmi *hdmi = to_rtk_hdmi(encoder);
	int ret;

	DRM_DEBUG_KMS("%s\n", __func__);

	rtk_hdmi_send_avmute(hdmi, 0);

	if(hdmi->hdcp_support) {
		msleep(50);
		ret = rtk_hdcp_enable(hdmi, hdmi->connector.state->hdcp_content_type);
		if (ret)
			DRM_DEBUG_KMS("%s: hdcp enable failed.\n", __func__);
	}
}

static void rtk_hdmi_enc_disable(struct drm_encoder *encoder)
{
	struct rtk_hdmi *hdmi = to_rtk_hdmi(encoder);

	DRM_DEBUG_KMS("%s\n", __func__);

	if(hdmi->hdcp_support)
		rtk_hdcp_disable(hdmi);

	rtk_hdmi_send_avmute(hdmi, 1);
	msleep(50);
	rtk_hdmi_off(hdmi);
}

static int rtk_hdmi_enc_atomic_check(struct drm_encoder *encoder,
				struct drm_crtc_state *crtc_state,
				struct drm_connector_state *conn_state)
{
	DRM_DEBUG_KMS("%s\n", __func__);
	return 0;
}

static const struct drm_encoder_funcs rtk_hdmi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_encoder_helper_funcs rtk_hdmi_encoder_helper_funcs = {
	.mode_fixup = rtk_hdmi_enc_mode_fixup,
	.mode_set   = rtk_hdmi_enc_mode_set,
	.enable     = rtk_hdmi_enc_enable,
	.disable    = rtk_hdmi_enc_disable,
	.atomic_check = rtk_hdmi_enc_atomic_check,
};

static enum drm_connector_status
rtk_hdmi_conn_detect(struct drm_connector *connector, bool force)
{
	struct rtk_hdmi *hdmi;
	int hpd;
	int rxsense;

	hdmi = to_rtk_hdmi(connector);

	hpd = gpiod_get_value(hdmi->hpd_gpio);
	if (hdmi->rxsense_mode) {
		rxsense = rtk_hdmi_get_rxsense(hdmi);
		DRM_DEBUG_KMS("HPD(%d) RxSense(%d)\n", hpd, rxsense);
	} else {
		rxsense = hpd;
		DRM_DEBUG_KMS("HPD(%d)\n", hpd);
	}

	hdmi->hpd_state = hpd;

	return (hpd && rxsense) ?
		connector_status_connected : connector_status_disconnected;
}

static void rtk_hdmi_conn_destroy(struct drm_connector *connector)
{
	DRM_DEBUG_KMS("%s\n", __func__);
}

static int rtk_hdmi_conn_get_modes(struct drm_connector *connector)
{
	struct rtk_hdmi *hdmi = to_rtk_hdmi(connector);
	struct edid *edid;
	int ret = 0;

	DRM_DEBUG_KMS("%s\n", __func__);

	if (!hdmi->ddc)
		return 0;

	edid = drm_get_edid(connector, hdmi->ddc);
	if (edid) {
		memset(&hdmi->edid_info, 0, sizeof(hdmi->edid_info));
		hdmi->edid_info.sink_is_hdmi = drm_detect_hdmi_monitor(edid);
		hdmi->edid_info.sink_has_audio = drm_detect_monitor_audio(edid);
		drm_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		rtk_parse_cea_ext(hdmi, edid);
		rtk_hdmi_fill_hdr_data(hdmi, edid);
		// TODO: move rtk_hdmi_fill_audio_data to after enabling hdmi
		rtk_hdmi_fill_audio_data(hdmi, edid);

		if ((hdmi->edid_info.scds_pb5 & SCDS_ALLM) &&
				(hdmi->en_allm == ALLM_UNSUPPORTED))
			hdmi->en_allm = ALLM_DISABLE;
		else if (!(hdmi->edid_info.scds_pb5 & SCDS_ALLM) &&
				(hdmi->en_allm == ALLM_DISABLE))
			hdmi->en_allm = ALLM_UNSUPPORTED;

		if (((hdmi->edid_info.vrr_min != 0) ||
			(hdmi->edid_info.scds_pb5 & SCDS_QMS)) &&
			(hdmi->en_qms_vrr == QMS_VRR_UNSUPPORTED))
			hdmi->en_qms_vrr = QMS_VRR_DISABLE;
		else if ((hdmi->edid_info.vrr_min == 0) &&
			!(hdmi->edid_info.scds_pb5 & SCDS_QMS) &&
			(hdmi->en_qms_vrr == QMS_VRR_DISABLE))
			hdmi->en_qms_vrr = QMS_VRR_UNSUPPORTED;

		kfree(edid);
	}

	return ret;
}

static enum drm_mode_status
rtk_hdmi_conn_mode_valid(struct drm_connector *connector,
			 struct drm_display_mode *mode)
{
	u8 vic;

	vic = drm_match_cea_mode(mode);

	DRM_DEBUG_KMS("vic=%u\n", vic);

	if (rtk_hdmi_valid_vic(vic))
		return MODE_OK;

	return MODE_ERROR;
}

static const struct drm_connector_funcs rtk_hdmi_connector_funcs = {
//	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = rtk_hdmi_conn_detect,
	.destroy = rtk_hdmi_conn_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_set_property = rtk_hdmi_conn_set_property,
	.atomic_get_property = rtk_hdmi_conn_get_property,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_connector_helper_funcs rtk_hdmi_connector_helper_funcs = {
	.get_modes = rtk_hdmi_conn_get_modes,
	.mode_valid = rtk_hdmi_conn_mode_valid,
};

static void rtk_hdmi_rxsense_timer_cb(struct timer_list *rxsense_timer)
{
	struct rtk_hdmi *hdmi;
	int curr_rxsense;

	hdmi = to_rtk_hdmi(rxsense_timer);

	curr_rxsense = rtk_hdmi_get_rxsense(hdmi);

	if (curr_rxsense != hdmi->rxsense_state) {
		hdmi->rxsense_state = curr_rxsense;
		schedule_work(&hdmi->hpd_work);
	}

	if (hdmi->hpd_state)
		mod_timer(rxsense_timer, jiffies + msecs_to_jiffies(30));
}

static void rtk_hdmi_hpd_worker(struct work_struct *hpd_work)
{
	struct rtk_hdmi *hdmi;

	hdmi = to_rtk_hdmi(hpd_work);

	drm_helper_hpd_irq_event(hdmi->connector.dev);

	if(hdmi->hdcp_support &&
	   hdmi->connector.status == connector_status_connected) {
		rtk_hdcp_update_mCap(hdmi, WV_HDCP_NONE);
	} else if(hdmi->hdcp_support &&
		hdmi->connector.status == connector_status_disconnected) {
		rtk_hdcp_disable(hdmi);
		rtk_hdcp_update_mCap(hdmi, WV_HDCP_NO_DIGITAL_OUTPUT);
	}

	if (hdmi->hpd_state && hdmi->rxsense_mode == RXSENSE_TIMER_MODE)
		mod_timer(&hdmi->rxsense_timer, jiffies + msecs_to_jiffies(30));
}

static irqreturn_t rtk_hdmi_hpd_irq(int irq, void *dev_id)
{
	struct rtk_hdmi *hdmi = dev_id;

	schedule_work(&hdmi->hpd_work);

	return IRQ_HANDLED;
}

static int rtk_hdmi_bind(struct device *dev, struct device *master,
				 void *data)
{
	struct drm_device *drm = data;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct device_node *syscon_np;
	struct rtk_drm_private *priv = drm->dev_private;
	struct rtk_hdmi *hdmi;
	int ret;
	int size;
	const u32 *prop;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->drm_dev = drm;
	hdmi->dev = dev;

	syscon_np = of_parse_phandle(dev->of_node, "syscon", 0);
	if (IS_ERR_OR_NULL(syscon_np)) {
		dev_err(dev, "Parse syscon phandle 0 fail");
		ret = -EPROBE_DEFER;
		goto err_exit;
	}

	hdmi->hdmireg = syscon_node_to_regmap(syscon_np);
	if (IS_ERR(hdmi->hdmireg)) {
		dev_err(dev, "Remap syscon 0 to hdmireg fail");
		of_node_put(syscon_np);
		ret = PTR_ERR(hdmi->hdmireg);
		goto err_exit;
	}

	syscon_np = of_parse_phandle(dev->of_node, "syscon", 1);
	if (IS_ERR_OR_NULL(syscon_np)) {
		dev_err(dev, "Parse syscon phandle 1 fail");
		ret = -EPROBE_DEFER;
		goto err_exit;
	}

	hdmi->crtreg = syscon_node_to_regmap(syscon_np);
	if (IS_ERR(hdmi->crtreg)) {
		dev_err(dev, "Remap syscon 1 to crtreg fail");
		of_node_put(syscon_np);
		ret = PTR_ERR(hdmi->crtreg);
		goto err_exit;
	}

	ret = of_property_read_u32(dev->of_node, "rxsense-mode",
				&hdmi->rxsense_mode);

	if (ret < 0 || hdmi->rxsense_mode > RXSENSE_INTERRUPT_MODE)
		hdmi->rxsense_mode = RXSENSE_PASSIVE_MODE;

	if (hdmi->rxsense_mode == RXSENSE_PASSIVE_MODE)
		goto skip_topreg;

	syscon_np = of_parse_phandle(dev->of_node, "syscon", 2);
	if (IS_ERR_OR_NULL(syscon_np)) {
		dev_err(dev, "Parse syscon phandle 2 fail");
		ret = -EPROBE_DEFER;
		goto err_exit;
	}

	hdmi->topreg = syscon_node_to_regmap(syscon_np);
	if (IS_ERR(hdmi->topreg)) {
		dev_err(dev, "Remap syscon 2 to topreg fail");
		of_node_put(syscon_np);
		ret = PTR_ERR(hdmi->topreg);
		goto err_exit;
	}

skip_topreg:

	hdmi->reset_hdmi = devm_reset_control_get_optional_exclusive(dev, "rstn_hdmi");
	if (IS_ERR(hdmi->reset_hdmi)) {
		dev_err(dev, "Can't get reset_control reset_hdmi");
		return -EPROBE_DEFER;
	}

	hdmi->clk_hdmi = devm_clk_get(dev, "clk_en_hdmi");
	if (IS_ERR(hdmi->clk_hdmi)) {
		dev_err(dev, "Can't get clk clk_hdmi");
		return -EPROBE_DEFER;
	}

	prop = of_get_property(dev->of_node, "hdcp", &size);
	if (prop) {
		hdmi->hdcp_support = of_read_number(prop, 1);
		dev_info(dev, "get hdcp on/off setting: 0x%x\n",
			 hdmi->hdcp_support);
	} else {
		dev_err(dev, "no hdcp on/off setting.\n");
		hdmi->hdcp_support = 0;
	}

	hdmi->hpd_gpio = devm_gpiod_get(dev, "hpd", GPIOD_IN);
	if (IS_ERR(hdmi->hpd_gpio)) {
		dev_err(dev, "Could not get gpio from of\n");
		return -EPROBE_DEFER;
	}

	dev_info(dev, "hotplug gpio(%d)\n", desc_to_gpio(hdmi->hpd_gpio));

	gpiod_direction_input(hdmi->hpd_gpio);
	gpiod_set_debounce(hdmi->hpd_gpio, 30*1000); /* 30ms */

	encoder = &hdmi->encoder;
	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	drm_encoder_init(drm, encoder, &rtk_hdmi_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);

	drm_encoder_helper_add(encoder, &rtk_hdmi_encoder_helper_funcs);

	connector = &hdmi->connector;
	connector->polled = DRM_CONNECTOR_POLL_HPD;
	ret = drm_connector_init(drm, connector, &rtk_hdmi_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		dev_err(dev, "connector_init failed");
		goto err_exit;
	}
	drm_connector_helper_add(connector, &rtk_hdmi_connector_helper_funcs);

	drm_connector_attach_encoder(connector, encoder);

	rtk_hdmi_init_properties(hdmi);

	hdmi->ddc = i2c_get_adapter(I2C_BUS_ID);
	if (IS_ERR(hdmi->ddc)) {
		dev_err(dev, "Get i2c adapter fail\n");
		ret = PTR_ERR(hdmi->ddc);
		hdmi->ddc = NULL;
		goto err_exit;
	}

	if(hdmi->hdcp_support) {
		ret = rtk_hdcp_init(hdmi, hdmi->hdcp_support);
		if (ret)
			dev_err(dev, "RealTek HDCP init failed, skipping.\n");
	}

	if (hdmi->rxsense_mode == RXSENSE_TIMER_MODE) {
		hdmi->rxsense_state= rtk_hdmi_get_rxsense(hdmi);
		timer_setup(&hdmi->rxsense_timer, rtk_hdmi_rxsense_timer_cb, 0);
	}

	INIT_WORK(&hdmi->hpd_work, rtk_hdmi_hpd_worker);

	hdmi->hpd_irq = gpiod_to_irq(hdmi->hpd_gpio);
	if (hdmi->hpd_irq < 0) {
		dev_err(dev, "Fail to get hpd_irq");
		ret = hdmi->hpd_irq;
		goto err_exit;
	}

	hdmi->hpd_state = gpiod_get_value(hdmi->hpd_gpio);
	if (hdmi->hpd_state)
		schedule_work(&hdmi->hpd_work);

	irq_set_irq_type(hdmi->hpd_irq, IRQ_TYPE_EDGE_BOTH);
	ret = devm_request_irq(dev, hdmi->hpd_irq, rtk_hdmi_hpd_irq,
				IRQF_SHARED, dev_name(dev), hdmi);

	if (ret) {
		dev_err(dev, "can't request hpd gpio irq\n");
		goto err_exit;
	}

	hdmi->rpc_info = &priv->rpc_info;

	ret = rtk_rpc_init(dev, &hdmi->audio_edid_buf);
	if (ret < 0)
		goto err_exit;

	dev_set_drvdata(dev, hdmi);

	return 0;

err_exit:
	return ret;
}

static void rtk_hdmi_unbind(struct device *dev, struct device *master,
			     void *data)
{

}

static const struct component_ops rtk_hdmi_ops = {
	.bind	= rtk_hdmi_bind,
	.unbind	= rtk_hdmi_unbind,
};

static int rtk_hdmi_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &rtk_hdmi_ops);
}

static int rtk_hdmi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &rtk_hdmi_ops);
	return 0;
}

static const struct of_device_id rtk_hdmi_dt_ids[] = {
	{ .compatible = "realtek,rtk-hdmi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, rtk_hdmi_dt_ids);

struct platform_driver rtk_hdmi_driver = {
	.probe  = rtk_hdmi_probe,
	.remove = rtk_hdmi_remove,
	.driver = {
		.name = "rtk-hdmi",
		.of_match_table = rtk_hdmi_dt_ids,
	},
};
