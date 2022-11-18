/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 RealTek Inc.
 */

#ifndef RTK_HDMI_H_
#define RTK_HDMI_H_

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_scdc_helper.h>

#include "rtk_drm_drv.h"
#include "rtk_hdcp.h"

/* SCDS PB5 Flags */
#define SCDS_FAPA_END    (1 << 7)
#define SCDS_QMS         (1 << 6)
#define SCDS_M_DELTA     (1 << 5)
#define SCDS_CINEMA_VRR  (1 << 4)
#define SCDS_NEG_MVRR    (1 << 3)
#define SCDS_FVA         (1 << 2)
#define SCDS_ALLM        (1 << 1)
#define SCDS_FAPA_START  (1 << 0)

/* SCDS PB8 Flags */
#define SCDS_DSC_1P2     (1 << 7)
#define SCDS_DSC_NAT420  (1 << 6)
#define SCDS_QMS_TFR_MAX (1 << 5)
#define SCDS_QMS_TFR_MIN (1 << 4)
#define SCDS_DSC_ALL_BPP (1 << 3)
#define SCDS_DSC_16BPC   (1 << 2)
#define SCDS_DSC_12BPC   (1 << 1)
#define SCDS_DSC_10BPC   (1 << 0)

struct hdmi_edid_info {
	bool sink_is_hdmi;
	bool sink_has_audio;
	u32 max_tmds_char_rate;
	u8 scdc_capable;
	u8 dc_420;
	u8 colorimetry;
	u8 vcdb;
	u8 max_frl_rate;
	u8 scds_pb5;
	u8 scds_pb8;
	u8 vrr_min;
	u16 vrr_max;
};

enum FRACTIONAL_FPS_STATUS {
	FRACTIONAL_FPS_DISABLE,
	FRACTIONAL_FPS_ENABLE,
};

enum ALLM_STATUS {
	ALLM_UNSUPPORTED,
	ALLM_DISABLE,
	ALLM_ENABLE,
};

enum QMS_VRR_STATUS {
	QMS_VRR_DISABLE = HDMI_VRR_DISABLE,
	QMS_VRR_EN_VRR = HDMI_VRR_ENABLE,
	QMS_VRR_EN_QMS = HDMI_QMS_ENABLE,
	QMS_VRR_UNSUPPORTED = 0xFF,
};

enum QMS_VRR_RATE {
	RATE_23HZ = HDMI_VRR_23HZ,
	RATE_24HZ = HDMI_VRR_24HZ,
	RATE_25HZ = HDMI_VRR_25HZ,
	RATE_29HZ = HDMI_VRR_29HZ,
	RATE_30HZ = HDMI_VRR_30HZ,
	RATE_47HZ = HDMI_VRR_47HZ,
	RATE_48HZ = HDMI_VRR_48HZ,
	RATE_50HZ = HDMI_VRR_50HZ,
	RATE_59HZ = HDMI_VRR_59HZ,
	RATE_60HZ = HDMI_VRR_60HZ,
	RATE_BASE = 0xFE,
	RATE_UNSPECIFIED = 0xFF,
};

enum HDMI_RXSENSE_MODE {
	RXSENSE_PASSIVE_MODE = 0,
	RXSENSE_TIMER_MODE,
	// TODO: Implement RxSense interrupt mode
	RXSENSE_INTERRUPT_MODE,
};

enum HDMI_RXSENSE_STATUS {
	HDMI_RXSENSE_OFF = 0,
	HDMI_RXSENSE_ON,
	HDMI_RXSENSE_UNKNOWN,
};

struct rtk_hdmi {
	struct device *dev;
	struct drm_device *drm_dev;
	struct drm_connector connector;
	struct drm_encoder encoder;

	struct reset_control *reset_hdmi;
	struct clk *clk_hdmi;
	struct regmap *crtreg;
	struct regmap *hdmireg;
	struct regmap *topreg;
	int hpd_irq;
	struct work_struct hpd_work;
	enum HDMI_RXSENSE_MODE rxsense_mode;
	struct timer_list rxsense_timer;

	struct gpio_desc *hpd_gpio;
	struct i2c_adapter *ddc;
	struct hdmi_edid_info edid_info;
	int hpd_state;
	int rxsense_state;

	enum hdmi_colorspace rgb_or_yuv;
	struct drm_property *rgb_or_yuv_property;

	enum rtk_hdr_mode hdr_mode;
	struct drm_property *hdr_mode_property;

	enum FRACTIONAL_FPS_STATUS en_fractional_fps;
	struct drm_property *fractional_fps_property;

	enum ALLM_STATUS en_allm;
	struct drm_property *allm_property;

	enum QMS_VRR_STATUS en_qms_vrr;
	struct drm_property *qms_property;

	enum QMS_VRR_RATE vrr_rate;
	struct drm_property *vrr_rate_property;

	struct rtk_rpc_info *rpc_info;
	struct rtk_rpc_info audio_edid_buf;

	struct rtk_hdcp hdcp;
	u32 hdcp_support;
};

extern void rtk_parse_cea_ext(struct rtk_hdmi *hdmi, struct edid *edid);

#endif /* RTK_HDMI_H_ */
