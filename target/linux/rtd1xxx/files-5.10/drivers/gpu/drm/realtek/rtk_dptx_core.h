/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019 RealTek Inc.
 */

#ifndef _RTK_DPTX_CORE_H
#define _RTK_DPTX_CORE_H

#include <linux/reset.h>

#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>

#include "rtk_drm_drv.h"
//#include "rtk_dptx_rpc.h"

#define MAX_LANE	4
#define MAX_CR_LOOP	5
#define MAX_EQ_LOOP	5

/* DP_TRAINING_LANE0_SET */
#define PRE_EMPHASIS_SET(x)		(((x) & 0x3) << 3)
#define PRE_EMPHASIS_GET(x)		(((x) >> 3) & 0x3)
#define VOLTAGE_SWING_SET(x)		(((x) & 0x3) << 0)
#define VOLTAGE_SWING_GET(x)		(((x) >> 0) & 0x3)

enum link_lane_count_type {
	LANE_COUNT1 = 1,
	LANE_COUNT2 = 2,
	LANE_COUNT4 = 4
};

enum link_training_state {
	START,
	CLOCK_RECOVERY,
	EQUALIZER_TRAINING,
	FINISHED,
	FAILED
};

enum voltage_swing_level {
	VOLTAGE_LEVEL_0,
	VOLTAGE_LEVEL_1,
	VOLTAGE_LEVEL_2,
	VOLTAGE_LEVEL_3,
};

enum pre_emphasis_level {
	PRE_EMPHASIS_LEVEL_0,
	PRE_EMPHASIS_LEVEL_1,
	PRE_EMPHASIS_LEVEL_2,
	PRE_EMPHASIS_LEVEL_3,
};

enum pattern_set {
	IDLE_PTN = 0,
	TRAINING_PTN1,
	TRAINING_PTN2,
	TRAINING_PTN3,
	VIDEO_PTN,
	PRBS7_PTN
};

struct enc_info {
	int max_link_rate;


	enum link_lane_count_type max_lane_count;
};

struct link_train {
	int eq_loop;
	int cr_loop[MAX_LANE];

	u8 emphasis[MAX_LANE];
	u8 swing[MAX_LANE];
	u8 adjust[MAX_LANE];

	u8 link_rate;
	u8 lane_count;

	enum link_training_state lt_state;
};

struct rtk_dptx {
	struct device *dev;
	struct drm_device *drm_dev;
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct drm_dp_aux aux;

	struct semaphore sem;

	struct clk *plledp;
	struct clk *pllpixel;
	struct reset_control *rstc;
	void __iomem *edpreg;
	void __iomem *lvdsreg;

	struct rtk_rpc_info *rpc_info;

	struct enc_info enc_info;
	struct link_train lt_info;
	int hpd_gpio;
	int irq;
};

void dptx_set_sst_setting(struct rtk_dptx *dptx);
void dptx_set_video_timing(struct rtk_dptx *dptx,
			   struct drm_display_mode *mode);
void dptx_set_training_lane(struct rtk_dptx *dptx);
void dptx_set_training_pattern(struct rtk_dptx *dptx,
			       enum pattern_set pattern);

int dptx_aux_isr(struct rtk_dptx *dptx);
void dptx_aux_get_data(struct rtk_dptx *dptx, struct drm_dp_aux_msg *msg);
void dptx_aux_transfer(struct rtk_dptx *dptx, struct drm_dp_aux_msg *msg);
void dptx_init(struct rtk_dptx *dptx);

#endif /* _RTK_DPTX_CORE_H */
