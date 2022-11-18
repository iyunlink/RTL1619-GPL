/*
 * frl_link_training.c - HDMI2.1 FRL link training procedure
 *
 * Copyright (C) 2019 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include "hdmitx.h"
#include "hdmitx_dev.h"
#include "hdmitx_scdc.h"
#include "hdmitx_reg.h"
#include "crt_reg.h"

#define HPD_TIMEOUT 500
#define LTS2_TIMEOUT 3000
#define LTS3_TIMEOUT 3000
#define LTSP_TIMEOUT 300

enum FRL_RATES {
	FRL_3G3LANES = 1,
	FRL_6G3LANES,
	FRL_6G4LANES,
};

enum LINK_TRAINING_STATE {
	FSM_LTS_1,
	FSM_LTS_2,
	FSM_LTS_3,
	FSM_LTS_4,
	FSM_LTS_L,
	FSM_LTS_P,
	FSM_LTS_EXIT
};

enum FFE_CHANNEL {
	TXFFE_0 = 0,
	TXFFE_1,
	TXFFE_2,
	TXFFE_3,
};

enum FFE_LEVELS {
	FFE_LEVEL0 = 0,
	FFE_LEVEL1,
	FFE_LEVEL2,
	FFE_LEVEL3,
};

enum FFE_MODE {
	FFE_NORMAL = 0,
	FFE_PRESHOOT,
	FFE_DEEMPHASIS,
};

static unsigned char test_conf;
static unsigned char d0_ffe_counter;
static unsigned char d1_ffe_counter;
static unsigned char d2_ffe_counter;
static unsigned char d3_ffe_counter;

/**
 * set_frl_txffe - LT TxFFE
 * @channel: enum FFE_CHANNEL
 * @level: enum FFE_LEVELS
 * @mode: enum FFE_MODE
 */
static void set_frl_txffe(struct device *dev,
	unsigned char channel, unsigned char level, unsigned char mode)
{
	unsigned int em_mask;
	unsigned int em_val;
	unsigned int iem_mask;
	unsigned int iem_val;
	unsigned int idrv_mask;
	unsigned int idrv_val;
	unsigned int empre_mask;
	unsigned int empre_val;
	unsigned int iem;
	unsigned int idrv;
	unsigned int iempre;

	switch (level) {
	case FFE_LEVEL0:
		iem = 0;
		idrv = 14;
		iempre = 0;
		break;
	case FFE_LEVEL1:
		iem = 1;
		idrv = 16;
		iempre = 1;
		break;
	case FFE_LEVEL2:
		iem = 2;
		idrv = 18;
		iempre = 2;
		break;
	case FFE_LEVEL3:
		iem = 3;
		idrv = 20;
		iempre = 3;
		break;
	default:
		dev_err(dev, "%s unknown level", __func__);
		return;
	}

	if (mode == FFE_NORMAL)
		idrv = 14;
	else if (mode == FFE_PRESHOOT)
		iem = 0;
	else if (mode == FFE_DEEMPHASIS)
		iempre = 0;

	switch (channel) {
	case TXFFE_0:
		em_mask = SYS_PLL_HDMI_LDO2_REG_TMDS_EMA_mask;
		em_val = SYS_PLL_HDMI_LDO2_REG_TMDS_EMA(1);
		iem_mask = SYS_PLL_HDMI_LDO4_REG_TMDS_IEMA_mask;
		iem_val = SYS_PLL_HDMI_LDO4_REG_TMDS_IEMA(iem);
		idrv_mask = SYS_PLL_HDMI_LDO3_REG_TMDS_IDRVA_mask;
		idrv_val = SYS_PLL_HDMI_LDO3_REG_TMDS_IDRVA(idrv);
		empre_mask =
			SYS_PLL_HDMI_LDO7_REG_TMDS_EMPREA_mask |
			SYS_PLL_HDMI_LDO7_REG_TMDS_IEMPREA_mask;
		empre_val =
			SYS_PLL_HDMI_LDO7_REG_TMDS_EMPREA(1) |
			SYS_PLL_HDMI_LDO7_REG_TMDS_IEMPREA(iempre);
		break;
	case TXFFE_1:
		em_mask = SYS_PLL_HDMI_LDO2_REG_TMDS_EMB_mask;
		em_val = SYS_PLL_HDMI_LDO2_REG_TMDS_EMB(1);
		iem_mask = SYS_PLL_HDMI_LDO4_REG_TMDS_IEMB_mask;
		iem_val = SYS_PLL_HDMI_LDO4_REG_TMDS_IEMB(iem);
		idrv_mask = SYS_PLL_HDMI_LDO3_REG_TMDS_IDRVB_mask;
		idrv_val = SYS_PLL_HDMI_LDO3_REG_TMDS_IDRVB(idrv);
		empre_mask =
			SYS_PLL_HDMI_LDO7_REG_TMDS_EMPREB_mask |
			SYS_PLL_HDMI_LDO7_REG_TMDS_IEMPREB_mask;
		empre_val =
			SYS_PLL_HDMI_LDO7_REG_TMDS_EMPREB(1) |
			SYS_PLL_HDMI_LDO7_REG_TMDS_IEMPREB(iempre);
		break;
	case TXFFE_2:
		em_mask = SYS_PLL_HDMI_LDO2_REG_TMDS_EMC_mask;
		em_val = SYS_PLL_HDMI_LDO2_REG_TMDS_EMC(1);
		iem_mask = SYS_PLL_HDMI_LDO4_REG_TMDS_IEMC_mask;
		iem_val = SYS_PLL_HDMI_LDO4_REG_TMDS_IEMC(iem);
		idrv_mask = SYS_PLL_HDMI_LDO3_REG_TMDS_IDRVC_mask;
		idrv_val = SYS_PLL_HDMI_LDO3_REG_TMDS_IDRVC(idrv);
		empre_mask =
			SYS_PLL_HDMI_LDO7_REG_TMDS_EMPREC_mask |
			SYS_PLL_HDMI_LDO7_REG_TMDS_IEMPREC_mask;
		empre_val =
			SYS_PLL_HDMI_LDO7_REG_TMDS_EMPREC(1) |
			SYS_PLL_HDMI_LDO7_REG_TMDS_IEMPREC(iempre);
		break;
	case TXFFE_3:
		em_mask = SYS_PLL_HDMI_LDO2_REG_TMDS_EMCK_mask;
		em_val = SYS_PLL_HDMI_LDO2_REG_TMDS_EMCK(1);
		iem_mask = SYS_PLL_HDMI_LDO4_REG_TMDS_IEMCK_mask;
		iem_val = SYS_PLL_HDMI_LDO4_REG_TMDS_IEMCK(iem);
		idrv_mask = SYS_PLL_HDMI_LDO3_REG_TMDS_IDRVCK_mask;
		idrv_val = SYS_PLL_HDMI_LDO3_REG_TMDS_IDRVCK(idrv);
		empre_mask =
			SYS_PLL_HDMI_LDO7_REG_TMDS_EMPRECK_mask |
			SYS_PLL_HDMI_LDO7_REG_TMDS_IEMPRECK_mask;
		empre_val =
			SYS_PLL_HDMI_LDO7_REG_TMDS_EMPRECK(1) |
			SYS_PLL_HDMI_LDO7_REG_TMDS_IEMPRECK(iempre);
		break;
	default:
		dev_err(dev, "%s unknown channel", __func__);
		return;
	}

	/* en_IEM _0234[0] */
	hdmitx_mask32(dev, SYS_PLL_HDMI_LDO2, ~em_mask, em_val);

	/* IEM _023C[4:0] */
	hdmitx_mask32(dev, SYS_PLL_HDMI_LDO4, ~iem_mask, iem_val);

	/* IDRV _0238[12:8] */
	hdmitx_mask32(dev, SYS_PLL_HDMI_LDO3, ~idrv_mask, idrv_val);

	/* IEMPRE _0248[15:12] */
	hdmitx_mask32(dev, SYS_PLL_HDMI_LDO7, ~empre_mask, empre_val);
}

/*
 * set_ck_lane_as_data -
 * When FRL mode is activated, Clock lane should act as data lane
 */
static void set_ck_lane_as_data(struct device *dev)
{
	/* _0248[29] */
	hdmipll_mask32(dev, SYS_PLL_HDMI_LDO7,
		~SYS_PLL_HDMI_LDO7_REG_TMDS_SELCK_mask,
		SYS_PLL_HDMI_LDO7_REG_TMDS_SELCK(0));
}

/*
 * send_super_block - Super Block in LTS:P
 */
static void send_super_block(struct device *dev,
	unsigned char dpc_enable, unsigned char color_depth)
{
	unsigned char deep_color_enable;
	unsigned char deep_color_depth;

	HDMI_INFO("Send super block, dpc_enable=%u depth=%u",
		dpc_enable, color_depth);

	if (dpc_enable)
		deep_color_enable = 1;
	else
		deep_color_enable = 0;

	switch (color_depth) {
	case 8:
		deep_color_depth = 4;
		break;
	case 10:
		deep_color_depth = 5;
		break;
	case 12:
		deep_color_depth = 6;
		break;
	default:
		dev_err(dev, "%s Invalid color_depth %u", __func__, color_depth);
		deep_color_depth = 4;
		break;
	}

	/* 0x9800d42C */
	hdmitx_mask32(dev, HDMI_FRL_ST,
		~HDMI_FRL_ST_FRL_en_mask,
		HDMI_FRL_ST_get_FRL_en(1));

	/* 0x9800d154, need setting advanced */
	hdmitx_write32(dev, HDMI_DPC,
		HDMI_DPC_write_en2(1) | HDMI_DPC_color_depth(deep_color_depth) |
		HDMI_DPC_write_en1(1) | HDMI_DPC_dpc_enable(deep_color_enable));

	/* FRL mode can't set 1 */
	hdmitx_write32(dev, HDMI_DPC1,
		HDMI_DPC1_write_en4(1) | HDMI_DPC1_hdmi21vip_hsync_clr(0) |
		HDMI_DPC1_write_en2(1) | HDMI_DPC1_dp_vfch(0));

	/* 0x9800d034 */
	hdmitx_write32(dev, HDMI_CR,
		HDMI_CR_write_en3(1) | HDMI_CR_tmds_encen(1) |
		HDMI_CR_write_en1(1) | HDMI_CR_enablehdmi(1));

}

static void enable_frl_mode(struct device *dev, unsigned char frl_rate)
{
	unsigned int lane_mode = 0;
	unsigned int RFD_sel;
	unsigned int FRL_en;

	switch (frl_rate) {
	case FRL_3G3LANES:
	case FRL_6G3LANES:
		lane_mode = 0; /* 0 for 3Lane mode */
		break;
	case FRL_6G4LANES:
		lane_mode = 1; /* 1 for 4Lane mode */
		break;
	default:
		dev_err(dev, "Unknown frl_rate=%u", frl_rate);
		break;
	}

	RFD_sel = 1; /* 0: RFD=-3 ,1:RFD=-1 ,2:RDR=1 ,3:RFD=3 */
	FRL_en = (frl_rate > 0) ? 1:0;

	/* Hank B for debugging usage d414~d41C */
	/* d420 */
	hdmitx_mask32(dev, HDMI_FRL_FLOW_CTRL1,
		~(HDMI_FRL_FLOW_CTRL1_UnderFlow_thold_mask |
		HDMI_FRL_FLOW_CTRL1_Blank_TriByte_lower_bound_mask |
		HDMI_FRL_FLOW_CTRL1_Video_TriByte_lower_bound_mask),
		HDMI_FRL_FLOW_CTRL1_UnderFlow_thold(492) |
		HDMI_FRL_FLOW_CTRL1_Blank_TriByte_lower_bound(4) |
		HDMI_FRL_FLOW_CTRL1_Video_TriByte_lower_bound(4));

	/* d424 */
	hdmitx_mask32(dev, HDMI_FRL_FLOW_CTRL2,
		~(HDMI_FRL_FLOW_CTRL2_Ideal_sub_value_int_val_mask |
		HDMI_FRL_FLOW_CTRL2_OverFlow_thold_mask),
		HDMI_FRL_FLOW_CTRL2_Ideal_sub_value_int_val(5) |
		HDMI_FRL_FLOW_CTRL2_OverFlow_thold(492));

	/*
	 * d42C - HDMI_FRL_ST
	 * vrrfva_en=0 for disable VRR
	 * FRL_clk_gate_dis=0 for saving power cousumption use
	 * prefetch_blank_char_num=0, should always be 0
	 */
	hdmitx_write32(dev, HDMI_FRL_ST,
		HDMI_FRL_ST_tmds_win_cnt(96) |
		HDMI_FRL_ST_vrrfva_en(0) |
		HDMI_FRL_ST_fix18b_en(1) |
		HDMI_FRL_ST_FRL_clk_gate_dis(0) |
		HDMI_FRL_ST_prefetch_blank_char_num(0) |
		HDMI_FRL_ST_lane_mode(lane_mode) |
		HDMI_FRL_ST_RFD_sel(RFD_sel) |
		HDMI_FRL_ST_FRL_en(FRL_en));
}

/*
 * Link_Training_LTS1 - Source reads EDID
 */
static unsigned char link_training_s1(struct device *dev, unsigned char max_rate)
{
	unsigned char frl_rate;
	unsigned char sink_ver;
	int ret;

	frl_rate = 0;

	if (max_rate >= FRL_6G4LANES)
		frl_rate = FRL_6G4LANES;
	else if (max_rate == FRL_6G3LANES)
		frl_rate = FRL_6G3LANES;
	else if (max_rate == FRL_3G3LANES)
		frl_rate = FRL_3G3LANES;

	if (frl_rate > 0) {
		ret = hdmitx_read_scdc_port(SCDCS_Sink_Version, &sink_ver, 1);
		if (ret == SCDC_I2C_FAIL) {
			frl_rate = 0;
			goto exit;
		}

		if (sink_ver == 0) {
			frl_rate = 0;
			dev_err(dev, "SCDC Sink Version=%u", sink_ver);
                        goto exit;
		}

		if (sink_ver != 1)
			dev_info(dev, "SCDC Sink Version=%u", sink_ver);

		ret = hdmitx_write_scdc_port(SCDCS_Source_Version, 1);
		if (ret == SCDC_I2C_FAIL) {
			frl_rate = 0;
			goto exit;
		}
	}

exit:
	return frl_rate;
}

/*
 * Link_Training_LTS2 - Prepares for FRL Link Training
 * Return 
 * 	0 : OK 
 *	-ETIMEDOUT : timeout.
 *	-EIO : i2c error

 */
static int link_training_s2(struct device *dev, unsigned char frl_rate)
{
	unsigned char status_flag0;
	unsigned char w_sink_config1;
	unsigned int counter;
	int ret;

 	ret = 0;
	counter = 0;
	dev_info(dev, "Wait FLT_ready");

	while (1) {
		if (counter++ > LTS2_TIMEOUT) {
			dev_err(dev, "Wait FLT_ready time out, %dms", counter);
			ret = -ETIMEDOUT;
			goto exit;
		}

		/* Polls FLT_ready */
		ret = hdmitx_read_scdc_port(SCDCS_Status_Flag_0, &status_flag0, 1);
		if (ret == SCDC_I2C_FAIL) {
			ret = -EIO;
			goto exit;
		}


		if (status_flag0 & STATUS_FLT_READY) {
			/* Sink is ready for FRL Training */
			dev_info(dev, "STATUS_FLT_READY, set FRL_Rate=%u",
				frl_rate);

			switch (frl_rate) {
			case FRL_3G3LANES:
				/* 3L3G, FRL_rate=1 FFE_Levels=3 */
				w_sink_config1 = 0x31;
				break;
			case FRL_6G3LANES:
				/* 3L6G, FRL_rate=2  FFE_Levels=3  */
				w_sink_config1 = 0x32;
				break;
			case FRL_6G4LANES:
				/* 4L6G, FRL_rate=3, FFE_Levels=3 */
				w_sink_config1 = 0x33;
				break;
			default:
				w_sink_config1 = 0x0;
				ret = -EINVAL;
				goto exit;
			}

			/* Set FRL_Rate */
			ret = hdmitx_write_scdc_port(SCDCS_Config_1,
				w_sink_config1 & 0x0F);
			if (ret == SCDC_I2C_FAIL) {
				ret = -EIO;
				goto exit;
			}

			/* Set TxFFE=TxFFE0 for all active lanes */
			pr_info("Set TxFFE=TxFFE0 for all active lanes");
			d0_ffe_counter = 0;
			d1_ffe_counter = 0;
			d2_ffe_counter = 0;
			d3_ffe_counter = 0;

			/* Set FFE_Levels */
			ret = hdmitx_write_scdc_port(SCDCS_Config_1, w_sink_config1);
			if (ret == SCDC_I2C_FAIL) {
				ret = -EIO;
				goto exit;
			}

			/* CTS HFR1-10 */
			ret = hdmitx_read_scdc_port(SCDCS_Source_Test, &test_conf, 1);
			if (ret == SCDC_I2C_FAIL) {
				ret = -EIO;
				goto exit;
			}


			dev_info(dev, "SCDCS_Source_Test=0x%x", test_conf);

			set_ck_lane_as_data(dev);
			goto exit;
		}
		usleep_range(1000, 1050);
	}
exit:
	dev_info(dev, "EXIT LTS:2");

	return ret;
}

/*
 * Link_Training_LTS3 -
 * Conduct Link Training for the specified FRL_Rate
 * Return 
 * 	0 : OK 
 *	1 : test at different rate.
 *	-EREMOTEIO : hotplug pin down.
 *	-ETIMEDOUT : timeout.
 *	-EIO : i2c error
 */
static int link_training_s3(struct device *dev)
{
	unsigned char update0;
	unsigned char status_flag1;
	unsigned char status_flag2;
	unsigned char ln0_req;
	unsigned char ln1_req;
	unsigned char ln2_req;
	unsigned char ln3_req;
	unsigned int reg_val;
	unsigned int ch3_sel;
	unsigned int ch2_sel;
	unsigned int ch1_sel;
	unsigned int ch0_sel;
	unsigned int counter;
	int hpd_state;
	int ret;

	ret = 0;
	counter = 0;
	dev_info(dev, "Wait FLT_update");
	while (1) {

		if (counter++ > LTS3_TIMEOUT) {
			dev_err(dev, "FLT_UPDATE time out, reaches %dms",
				counter);
			ret = -ETIMEDOUT;
			goto exit;
		}

		ret = hdmitx_read_scdc_port(SCDCS_Update_0, &update0, 1);
		if (ret == SCDC_I2C_FAIL) {
			hpd_state = show_hpd_status(dev, true);
			if (hpd_state)
				ret = -EIO;
			else
				ret = -EREMOTEIO;
			goto exit;
		}

		if (update0 & UPDATE0_FLT_UPDATE) {
			/* Reads Ln(x)_LTP_req */
			ret = hdmitx_read_scdc_port(SCDCS_Status_Flag_1,
				&status_flag1, 1);
			if (ret == SCDC_I2C_FAIL) {
				hpd_state = show_hpd_status(dev, true);
				if (hpd_state)
					ret = -EIO;
				else
					ret = -EREMOTEIO;
				goto exit;
			}

			ret = hdmitx_read_scdc_port(SCDCS_Status_Flag_2,
				&status_flag2, 1);
			if (ret == SCDC_I2C_FAIL) {
				hpd_state = show_hpd_status(dev, true);
				if (hpd_state)
					ret = -EIO;
				else
					ret = -EREMOTEIO;
				goto exit;
			}

			dev_dbg(dev, "Status_Flag_1=0x%08x Status_Flag_2=0x%08x",
				status_flag1, status_flag2);

			ln0_req = status_flag1 & 0xF;
			ln1_req = (status_flag1 & 0xF0) >> 4;
			ln2_req = status_flag2 & 0xF;
			ln3_req = (status_flag2 & 0xF0) >> 4;

			dev_info(dev, "LTP_req 0/1/2/3 = %u, %u, %u, %u",
				ln0_req, ln1_req, ln2_req, ln3_req);

			if ((ln0_req == 0x0) && (ln1_req == 0x0) &&
				(ln2_req == 0x0) && (ln3_req == 0x0)) {
				/* Training finished */
				dev_info(dev, "Training finishe");
				ret = 0;
				goto clear_exit;
			}

			if ((ln0_req == 0xf) && (ln1_req == 0xf) &&
				(ln2_req == 0xf) && (ln3_req == 0xf)) {
				/* Test at a different Link rate */
				dev_info(dev, "Test at a different Link rate");
				ret = 1;
				goto clear_exit;
			}

			reg_val = hdmitx_read32(dev, HDMI_FRL_TRAINING);
			ch3_sel = HDMI_FRL_TRAINING_get_ch3_LTP_sel(reg_val);
			ch2_sel = HDMI_FRL_TRAINING_get_ch2_LTP_sel(reg_val);
			ch1_sel = HDMI_FRL_TRAINING_get_ch1_LTP_sel(reg_val);
			ch0_sel = HDMI_FRL_TRAINING_get_ch0_LTP_sel(reg_val);

			if (ln0_req == 0xe) {
				d0_ffe_counter++;
				set_frl_txffe(dev, TXFFE_0,
					d0_ffe_counter, FFE_NORMAL);
				dev_info(dev, "Increase D0_FFE to Tx0FFE%u",
					d0_ffe_counter);
			} else if ((ln0_req == 0x3) &
					!(test_conf & FLT_NO_TIMEOUT)) {
				dev_dbg(dev, "No change on ch0_sel");
			} else if ((ln0_req >= 1) && (ln0_req <= 8)) {
				ch0_sel = ln0_req;
				dev_dbg(dev, "Change ch0_sel to LTP%u", ch0_sel);
			}

			if (ln1_req == 0xe) {
				d1_ffe_counter++;
				set_frl_txffe(dev, TXFFE_1,
					d1_ffe_counter, FFE_NORMAL);
				dev_info(dev, "Increase D1_FFE to Tx1FFE%u",
					d1_ffe_counter);
			} else if ((ln1_req == 0x3) &
					!(test_conf & FLT_NO_TIMEOUT)) {
				dev_dbg(dev, "No change on ch1_sel");
			} else if ((ln1_req >= 1) && (ln1_req <= 8)) {
				ch1_sel = ln1_req;
				dev_dbg(dev, "Change ch1_sel to LTP%u", ch1_sel);
			}

			if (ln2_req == 0xe) {
				d2_ffe_counter++;
				set_frl_txffe(dev, TXFFE_2,
					d2_ffe_counter, FFE_NORMAL);
				dev_info(dev, "Increase D2_FFE to Tx2FFE%u",
					d2_ffe_counter);
			} else if ((ln2_req == 0x3) &
					!(test_conf & FLT_NO_TIMEOUT)) {
				dev_dbg(dev, "No change on ch2_sel");
			} else if ((ln2_req >= 1) && (ln2_req <= 8)) {
				ch2_sel = ln2_req;
				dev_dbg(dev, "Change ch2_sel to LTP%u", ch2_sel);
			}

			if (ln3_req == 0xe) {
				d3_ffe_counter++;
				set_frl_txffe(dev, TXFFE_3,
					d3_ffe_counter, FFE_NORMAL);
				dev_info(dev, "Increase D3_FFE to Tx3FFE%u",
					d3_ffe_counter);
			} else if ((ln3_req == 0x3) &
					!(test_conf & FLT_NO_TIMEOUT)) {
				dev_dbg(dev, "No change on ch3_sel");
			} else if ((ln3_req >= 1) && (ln3_req <= 8)) {
				ch3_sel = ln3_req;
				dev_dbg(dev, "Change ch3_sel to LTP%u", ch3_sel);
			}

			hdmitx_write32(dev, HDMI_FRL_TRAINING,
				HDMI_FRL_TRAINING_LTP_sw_clr(1) |
				HDMI_FRL_TRAINING_ch3_LTP_sel(ch3_sel) |
				HDMI_FRL_TRAINING_ch2_LTP_sel(ch2_sel) |
				HDMI_FRL_TRAINING_ch1_LTP_sel(ch1_sel) |
				HDMI_FRL_TRAINING_ch0_LTP_sel(ch0_sel) |
				HDMI_FRL_TRAINING_FRL_training_en(1));

			/* Clear FLT_update by writing 1 */
			ret = hdmitx_write_scdc_port(SCDCS_Update_0,
				UPDATE0_FLT_UPDATE);

			if (ret == SCDC_I2C_FAIL) {
				ret = -EIO;
				goto exit;
			}

		}

		if (update0 & UPDATE0_STATUS) {
			ret = hdmitx_read_scdc_port(SCDCS_Status_Flag_0,
				&status_flag1, 1);
			if (ret == SCDC_I2C_FAIL) {
				ret = -EIO;
				goto exit;
			}
			dev_info(dev, "CLK_DET(%u) Locked L0(%u)L1(%u)L2(%u)L3(%u)",
				(status_flag1 & STATUS_CLK_DET) ? 1:0,
				(status_flag1 & STATUS_L0_LOCK) ? 1:0,
				(status_flag1 & STATUS_L1_LOCK) ? 1:0,
				(status_flag1 & STATUS_L2_LOCK) ? 1:0,
				(status_flag1 & STATUS_L3_LOCK) ? 1:0);
			ret = hdmitx_write_scdc_port(SCDCS_Update_0, UPDATE0_STATUS);
			if (ret == SCDC_I2C_FAIL) {
				ret = -EIO;
				goto exit;
			}
		}
		usleep_range(1000, 1050);
	}

clear_exit:
	ret = hdmitx_write_scdc_port(SCDCS_Update_0, UPDATE0_FLT_UPDATE);
	if (ret == SCDC_I2C_FAIL) 
		ret = -EIO;
exit:
	dev_info(dev, "EXIT LTS:3");
	return ret;
}

/*
 * Link_Training_LTS4 -
 * FRL_Rate is changed to start Link Training for a new rate
 */
unsigned char link_training_s4(struct device *dev, unsigned char frl_rate)
{
	unsigned char new_rate;
	unsigned char w_sink_config1;
	int ret;

	/* Stop transmitting the Link Training Pattern */
	hdmitx_mask32(dev, HDMI_FRL_TRAINING,
		~HDMI_FRL_TRAINING_FRL_training_en_mask,
		HDMI_FRL_TRAINING_FRL_training_en(0));

	/* Set TxFFE=TxFFE0 for all active Lanes */
	dev_info(dev, "Sets TxFFE=TxFFE0 for all active Lanes");
	set_frl_txffe(dev, TXFFE_0, FFE_LEVEL0, FFE_NORMAL);
	set_frl_txffe(dev, TXFFE_1, FFE_LEVEL0, FFE_NORMAL);
	set_frl_txffe(dev, TXFFE_2, FFE_LEVEL0, FFE_NORMAL);
	set_frl_txffe(dev, TXFFE_3, FFE_LEVEL0, FFE_NORMAL);
	d0_ffe_counter = 0;
	d1_ffe_counter = 0;
	d2_ffe_counter = 0;
	d3_ffe_counter = 0;

	new_rate = 0;

	if (frl_rate == FRL_6G4LANES) {
		ret = hdmitx_write_scdc_port(SCDCS_Update_0, UPDATE0_FLT_UPDATE);
		if (ret == SCDC_I2C_FAIL) {
			goto exit;
		}
		
		new_rate = FRL_6G3LANES;
		/* 3L6G, FRL_rate=2  FFE_Levels=3  */
		w_sink_config1 = 0x32;
		/* Set FFE_Levels */
		ret = hdmitx_write_scdc_port(SCDCS_Config_1, w_sink_config1);
		if (ret == SCDC_I2C_FAIL) {
			new_rate = 0;
			goto exit;
		}
		/* Clear FLT_update by writing 1 */
		ret = hdmitx_write_scdc_port(SCDCS_Update_0,
			UPDATE0_FLT_UPDATE);

		if (ret == SCDC_I2C_FAIL) {
			new_rate = 0;
			goto exit;
		}
		
	}
exit:
	return new_rate;
}

/*
 * Link_Training_LTSP -
 * FRL training has passed
 * Return 0 : frl starting
 *        1 : frl update
 *	  -ETIMEDOUT : timeout.
 *	  -EIO : i2c error
 */
static int link_training_sp(struct device *dev,
	unsigned char dpc_enable, unsigned char color_depth)
{
	unsigned char update0;
	unsigned int counter;
	int ret; 	

	/* Clear FLT_update by writing 1 */
	ret = hdmitx_write_scdc_port(SCDCS_Update_0, UPDATE0_FLT_UPDATE);
	if (ret == SCDC_I2C_FAIL) {
		ret = -EIO;
		goto exit;
	}

	send_super_block(dev, dpc_enable, color_depth);

	counter = 0;
	dev_info(dev, "Wait FRL_start");
	while (1) {
		if (counter++ > LTSP_TIMEOUT) {
			pr_err("ERROR: FRL_start time out, reaches %dms",
				counter);
			ret = -ETIMEDOUT;
			goto exit;
		}

		ret = hdmitx_read_scdc_port(SCDCS_Update_0, &update0, 1);
		if (ret == SCDC_I2C_FAIL) {
			ret = -EIO;
			goto exit;
		}
		if (update0 & UPDATE0_FRL_START) {
			/* FRL_start = 1 */
			ret = hdmitx_write_scdc_port(SCDCS_Update_0,
				UPDATE0_FRL_START);
			if (ret == SCDC_I2C_FAIL) {
				ret = -EIO;
				goto exit;
			}

			dev_info(dev, "Got FRL_start, count=%u", counter);
			ret = 0;
			goto exit;
		} else if (update0 & UPDATE0_FLT_UPDATE) {
			/* FRL_update = 1 */
			ret = hdmitx_write_scdc_port(SCDCS_Update_0,
				UPDATE0_FLT_UPDATE);
			if (ret == SCDC_I2C_FAIL) {
				ret = -EIO;
				goto exit;
			}

			/* EXIT to LTS:3 */
			dev_info(dev, "Got FRL_update, count=%u", counter);
			ret = 1;
			goto exit;
		}
		usleep_range(1000, 1050);
	}
exit:
	dev_info(dev, "EXIT LTS:P");
	return ret;
}

unsigned int
frl_link_training(struct device *dev, unsigned char max_rate,
	struct hdmi_format_setting *hdmi_format)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);
	struct rtk_hdmitx_frl *frl;
	unsigned char frl_fsm_state;
	unsigned char frl_rate;
	unsigned char dpc_enable;
	unsigned char color_depth;
	int hpd_state;
	int i;
	int ret;

	frl = tx_dev->frl;
	if (!frl || !frl->is_support_frl)
		return 0;

	frl->in_training = 1;

	if (hdmi_format->color != COLOR_YUV422) {
		dpc_enable = (hdmi_format->color_depth > 8) ? 1:0;
		color_depth = hdmi_format->color_depth;
	} else {
		dpc_enable = 0;
		color_depth = 8;
	}

	frl_rate = 0;
	test_conf = 0;
	d0_ffe_counter = 0;
	d1_ffe_counter = 0;
	d2_ffe_counter = 0;
	d3_ffe_counter = 0;
	frl_fsm_state = FSM_LTS_1;

	/* Set debounce to 1ms for CTS HFR1-10 */
	hdmitx_hpd_debounce(dev, 1);

NEXT_FSM:
	switch (frl_fsm_state) {
	case FSM_LTS_1:
		pr_info("Link training State LTS:1");

		frl_rate = link_training_s1(dev, max_rate);

		if (frl_rate != 0)
			frl_fsm_state = FSM_LTS_2;
		else
			frl_fsm_state = FSM_LTS_L;

		pr_info("frl_rate=%u", frl_rate);
		goto NEXT_FSM;
	case FSM_LTS_2:
		pr_info("Link training State LTS:2");

		hdmitx_reset_clk(dev);

		frl_set_phy(dev, frl_rate, hdmi_format);

		if (link_training_s2(dev, frl_rate))
			frl_fsm_state = FSM_LTS_L;
		else
			frl_fsm_state = FSM_LTS_3;

		goto NEXT_FSM;
	case FSM_LTS_3:
		pr_info("Link training State LTS:3");
		ret = link_training_s3(dev);
		pr_info("Disable and clear pattern");

		hdmitx_mask32(dev, HDMI_FRL_TRAINING,
			~(HDMI_FRL_TRAINING_LTP_sw_clr_mask |
			HDMI_FRL_TRAINING_FRL_training_en_mask),
			HDMI_FRL_TRAINING_LTP_sw_clr(0) |
			HDMI_FRL_TRAINING_FRL_training_en(0));
		hdmitx_mask32(dev, HDMI_FRL_TRAINING,
			~HDMI_FRL_TRAINING_LTP_sw_clr_mask,
			HDMI_FRL_TRAINING_LTP_sw_clr(1));

		switch (ret) {
		case 0:
			frl_fsm_state = FSM_LTS_P;
			break;	
		case 1:
			frl_fsm_state = FSM_LTS_4;
			break;
		case -EREMOTEIO:
			i = 0;
			do {
				hpd_state = show_hpd_status(dev, true);
				if (hpd_state)
					break;
				usleep_range(1000, 1050);
			} while (i++ < HPD_TIMEOUT);

			if (hpd_state)
				frl_fsm_state = FSM_LTS_1;
			else
				frl_fsm_state = FSM_LTS_L;
			break;
		case -ETIMEDOUT:
		case -EIO:
		default:
			frl_fsm_state = FSM_LTS_L;
			break;
		}

		goto NEXT_FSM;
	case FSM_LTS_4:
		pr_info("Link training State LTS:4");
		frl_rate = link_training_s4(dev, frl_rate);

		if (frl_rate != 0)
			frl_fsm_state = FSM_LTS_3;
		else
			frl_fsm_state = FSM_LTS_L;

		goto NEXT_FSM;
	case FSM_LTS_L:
		pr_info("Link training State LTS:L");
		/* no matter what below is ok or not , go exit */
		hdmitx_write_scdc_port(SCDCS_Config_1, 0);
		goto EXIT_FSM;
	case FSM_LTS_P:
		pr_info("Link training State LTS:P");
		enable_frl_mode(dev, frl_rate);

		if (IS_ENABLED(CONFIG_RTK_HDCP_1x)) {
			ta_hdcp14_init();
			ta_hdcp_set_keepout_win();
		}

		ret = link_training_sp(dev, dpc_enable, color_depth);

		if (ret < 0)
			frl_fsm_state = FSM_LTS_L;
		else if (ret == 0)
			frl_fsm_state = FSM_LTS_EXIT;
		else if (ret == 1)
			frl_fsm_state = FSM_LTS_3;

		goto NEXT_FSM;
	case FSM_LTS_EXIT:
	default:
		break;
	}

EXIT_FSM:

	hdmitx_hpd_debounce(dev, 30);

	hdmitx_mask32(dev, HDMI_FRL_TRAINING,
		~HDMI_FRL_TRAINING_FRL_training_en_mask,
		HDMI_FRL_TRAINING_FRL_training_en(0));

	if (frl_fsm_state == FSM_LTS_L)
		frl_rate = 0;

	frl->in_training = 0;
	frl->current_frl_rate = frl_rate;

	return frl_rate;
}

