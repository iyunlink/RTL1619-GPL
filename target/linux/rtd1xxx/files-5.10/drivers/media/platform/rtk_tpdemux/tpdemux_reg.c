/*
 * tpdemux_reg.c
 *
 * Copyright (C) 2020 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
 #include "tpdemux_reg.h"

void tp_reg_mapping(int tp_index, struct rtk_tp_reg *reg)
{
	switch (tp_index) {
	case DMX_TP_A_0:
		reg->reg_TF_CNTL = reg->base + TP_TF0_CNTL;
		reg->reg_TF_CNT = reg->base + TP_TF0_CNT;
		reg->reg_TF_CNTL = reg->base + TP_TF0_CNTL;
		reg->reg_TF_CNT = reg->base + TP_TF0_CNT;
		reg->reg_TF_DRP_CNT = reg->base + TP_TF0_DRP_CNT;
		reg->reg_TF_ERR_CNT = reg->base + TP_TF0_ERR_CNT;
		reg->reg_TF_FRMCFG = reg->base + TP_TF0_FRMCFG;
		reg->reg_TF_INT = reg->base + TP_TF0_INT;
		reg->reg_TF_INT_EN = reg->base + TP_TF0_INT_EN;
		reg->reg_TF_STRM_ID_0 = reg->base + TP_TF0_STRM_ID_0;
		reg->reg_TF_STRM_ID_1 = reg->base + TP_TF0_STRM_ID_1;
		reg->reg_TF_STRM_ID_2 = reg->base + TP_TF0_STRM_ID_2;
		reg->reg_TF_STRM_ID_3 = reg->base + TP_TF0_STRM_ID_3;
		reg->reg_TF_STRM_ID_VAL = reg->base + TP_TF0_STRM_ID_VAL;
		reg->reg_TP_DES_CNTL = reg->base + TP_TP0_DES_CNTL;

		reg->reg_TP_TF_CTRL_SWC = reg->base + TP_TF0_CTRL_SWC;

		reg->reg_TP_KEY_INFO_SWC_0 = reg->base + TP_KEY_INFO_SWC_0;
		reg->reg_TP_KEY_INFO_SWC_1 = reg->base + TP_KEY_INFO_SWC_1;
		reg->reg_TP_KEY_INFO_SWC_2 = reg->base + TP_KEY_INFO_SWC_2;
		reg->reg_TP_KEY_INFO_SWC_3 = reg->base + TP_KEY_INFO_SWC_3;
		reg->reg_TP_KEY_INFO_SWC_4 = reg->base + TP_KEY_INFO_SWC_4;
		reg->reg_TP_KEY_INFO_SWC_5 = reg->base + TP_KEY_INFO_SWC_5;
		reg->reg_TP_KEY_INFO_SWC_6 = reg->base + TP_KEY_INFO_SWC_6;
		reg->reg_TP_KEY_INFO_SWC_7 = reg->base + TP_KEY_INFO_SWC_7;
		reg->reg_TP_KEY_INFO_SWC_8 = reg->base + TP_KEY_INFO_SWC_8;
		reg->reg_TP_KEY_INFO_SWC_9 = reg->base + TP_KEY_INFO_SWC_9;
		reg->reg_TP_KEY_INFO_SWC_A = reg->base + TP_KEY_INFO_SWC_A;
		reg->reg_TP_KEY_INFO_SWC_B = reg->base + TP_KEY_INFO_SWC_B;

		reg->reg_TP_KEY_CTRL_SWC = reg->base + TP_KEY_CTRL_SWC;
		reg->reg_TP_KEY_HEADER_SWC = reg->base + TP_KEY_HEADER_SWC;
		reg->reg_TP_KEY_MASK_SWC = reg->base + TP_KEY_MASK_SWC;
		reg->reg_TP_SWC_DMY_A = reg->base + TP_SWC_DMY_A;

		reg->reg_TP_PID_PART = reg->base + TP_PID_PART;
		reg->reg_TP_CRC_INIT = reg->base + TP_CRC_INIT;

		reg->reg_TP_EXT_PID_CNTL = reg->base + TP_0_EXT_PID_CNTL;

		reg->reg_TP_M2M_RING_LIMIT = reg->base + TP0_M2M_RING_LIMIT;
		reg->reg_TP_M2M_RING_BASE = reg->base + TP0_M2M_RING_BASE;
		reg->reg_TP_M2M_RING_RP	= reg->base + TP0_M2M_RING_RP;
		reg->reg_TP_M2M_RING_WP	= reg->base + TP0_M2M_RING_WP;
		reg->reg_TP_M2M_RING_CTRL = reg->base + TP0_M2M_RING_CTRL;

		reg->reg_TP_RING_CTRL = reg->base + TP_RING_CTRL;
		reg->reg_TP_RING_LIMIT = reg->base + TP_RING_LIMIT;
		reg->reg_TP_RING_BASE = reg->base + TP_RING_BASE;
		reg->reg_TP_RING_RP = reg->base + TP_RING_RP;
		reg->reg_TP_RING_WP = reg->base + TP_RING_WP;
		reg->reg_TP_FULLNESS = reg->base + TP_FULLNESS;
		reg->reg_TP_THRESHOLD = reg->base + TP_THRESHOLD;
		reg->reg_TP_RING_FULL_INT_0 = reg->base + TP_RING_FULL_INT_0;
		reg->reg_TP_RING_FULL_INT_1 = reg->base + TP_RING_FULL_INT_1;
		reg->reg_TP_RING_FULL_INT_2 = reg->base + TP_RING_FULL_INT_2;
		reg->reg_TP_RING_FULL_INT_3 = reg->base + TP_RING_FULL_INT_3;
		reg->reg_TP_RING_AVAIL_INT_0 = reg->base +
						TP_RING_AVAIL_INT_0;
		reg->reg_TP_RING_AVAIL_INT_1 = reg->base +
						TP_RING_AVAIL_INT_1;
		reg->reg_TP_RING_AVAIL_INT_2 = reg->base +
						TP_RING_AVAIL_INT_2;
		reg->reg_TP_RING_AVAIL_INT_3 = reg->base +
						TP_RING_AVAIL_INT_3;
		reg->reg_TP_RING_AVAIL_INT_EN_0 = reg->base +
						TP_RING_AVAIL_INT_EN_0;
		reg->reg_TP_RING_AVAIL_INT_EN_1 = reg->base +
						TP_RING_AVAIL_INT_EN_1;
		reg->reg_TP_RING_AVAIL_INT_EN_2 = reg->base +
						TP_RING_AVAIL_INT_EN_2;
		reg->reg_TP_RING_AVAIL_INT_EN_3 = reg->base +
						TP_RING_AVAIL_INT_EN_3;
		reg->reg_TP_RING_FULL_INT_EN_0 = reg->base +
						TP_RING_FULL_INT_EN_0;
		reg->reg_TP_RING_FULL_INT_EN_1 = reg->base +
						TP_RING_FULL_INT_EN_1;
		reg->reg_TP_RING_FULL_INT_EN_2 = reg->base +
						TP_RING_FULL_INT_EN_2;
		reg->reg_TP_RING_FULL_INT_EN_3 = reg->base +
						TP_RING_FULL_INT_EN_3;

		reg->reg_TP_PID_CTRL	= reg->base + TP_PID_CTRL;
		reg->reg_TP_PID_DATA	= reg->base + TP_PID_DATA;
		reg->reg_TP_PID_DATA2	= reg->base + TP_PID_DATA2;
		reg->reg_TP_PID_DATA3	= reg->base + TP_PID_DATA3;
		reg->reg_TP_PCRCtrlReg = reg->base + TP0_PCR_CTL;
		reg->reg_TP_PCR_LATCH	= reg->base + TP_PCR_LATCH;
		reg->reg_TP_PCR_SYSTEM = reg->base + TP_PCR_SYSTEM;
		reg->reg_TP_PCR_BASE	= reg->base + TP_PCR_BASE;
		reg->reg_TP_PCR_EXT	= reg->base + TP_PCR_EXT;

		reg->reg_TP_SEC_CTRL  = reg->base + TP_SEC_CTRL;
		reg->reg_TP_SEC_DATA0 = reg->base + TP_SEC_DATA0;
		reg->reg_TP_SEC_DATA1 = reg->base + TP_SEC_DATA1;
		reg->reg_TP_SEC_DATA2 = reg->base + TP_SEC_DATA2;
		reg->reg_TP_SEC_DATA3 = reg->base + TP_SEC_DATA3;
		reg->reg_TP_SEC_DATA4 = reg->base + TP_SEC_DATA4;
		reg->reg_TP_SEC_DATA5 = reg->base + TP_SEC_DATA5;
		reg->reg_TP_SEC_DATA6 = reg->base + TP_SEC_DATA6;
		reg->reg_TP_SEC_DATA7 = reg->base + TP_SEC_DATA7;
		reg->reg_TP_SEC_DATA8 = reg->base + TP_SEC_DATA8;
		reg->reg_TP_SEC_DATA9 = reg->base + TP_SEC_DATA9;
		reg->reg_TP_SEC_DATA10 = reg->base + TP_SEC_DATA10;
		reg->reg_TP_SEC_DATA11 = reg->base + TP_SEC_DATA11;
		break;
	case DMX_TP_A_1:
		reg->reg_TF_CNTL = reg->base + TP_TF1_CNTL;
		reg->reg_TF_CNT = reg->base + TP_TF1_CNT;
		reg->reg_TF_DRP_CNT = reg->base + TP_TF1_DRP_CNT;
		reg->reg_TF_ERR_CNT = reg->base + TP_TF1_ERR_CNT;
		reg->reg_TF_FRMCFG = reg->base + TP_TF1_FRMCFG;
		reg->reg_TF_INT = reg->base + TP_TF1_INT;
		reg->reg_TF_INT_EN = reg->base + TP_TF1_INT_EN;
		reg->reg_TF_STRM_ID_0 = reg->base + TP_TF1_STRM_ID_0;
		reg->reg_TF_STRM_ID_1 = reg->base + TP_TF1_STRM_ID_1;
		reg->reg_TF_STRM_ID_2 = reg->base + TP_TF1_STRM_ID_2;
		reg->reg_TF_STRM_ID_3 = reg->base + TP_TF1_STRM_ID_3;
		reg->reg_TF_STRM_ID_VAL = reg->base + TP_TF1_STRM_ID_VAL;
		reg->reg_TP_DES_CNTL = reg->base + TP_TP1_DES_CNTL;

		reg->reg_TP_TF_CTRL_SWC = reg->base + TP_TF1_CTRL_SWC;

		reg->reg_TP_KEY_INFO_SWC_0 = reg->base + TP_KEY_INFO_SWC_0;
		reg->reg_TP_KEY_INFO_SWC_1 = reg->base + TP_KEY_INFO_SWC_1;
		reg->reg_TP_KEY_INFO_SWC_2 = reg->base + TP_KEY_INFO_SWC_2;
		reg->reg_TP_KEY_INFO_SWC_3 = reg->base + TP_KEY_INFO_SWC_3;
		reg->reg_TP_KEY_INFO_SWC_4 = reg->base + TP_KEY_INFO_SWC_4;
		reg->reg_TP_KEY_INFO_SWC_5 = reg->base + TP_KEY_INFO_SWC_5;
		reg->reg_TP_KEY_INFO_SWC_6 = reg->base + TP_KEY_INFO_SWC_6;
		reg->reg_TP_KEY_INFO_SWC_7 = reg->base + TP_KEY_INFO_SWC_7;
		reg->reg_TP_KEY_INFO_SWC_8 = reg->base + TP_KEY_INFO_SWC_8;
		reg->reg_TP_KEY_INFO_SWC_9 = reg->base + TP_KEY_INFO_SWC_9;
		reg->reg_TP_KEY_INFO_SWC_A = reg->base + TP_KEY_INFO_SWC_A;
		reg->reg_TP_KEY_INFO_SWC_B = reg->base + TP_KEY_INFO_SWC_B;
		reg->reg_TP_KEY_CTRL_SWC = reg->base + TP_KEY_CTRL_SWC;
		reg->reg_TP_KEY_HEADER_SWC = reg->base + TP_KEY_HEADER_SWC;
		reg->reg_TP_KEY_MASK_SWC = reg->base + TP_KEY_MASK_SWC;
		reg->reg_TP_SWC_DMY_A = reg->base + TP_SWC_DMY_A;

		reg->reg_TP_PID_PART = reg->base + TP_PID_PART;
		reg->reg_TP_CRC_INIT = reg->base + TP_CRC_INIT;

		reg->reg_TP_EXT_PID_CNTL = reg->base + TP_1_EXT_PID_CNTL;

		reg->reg_TP_M2M_RING_LIMIT = reg->base + TP1_M2M_RING_LIMIT;
		reg->reg_TP_M2M_RING_BASE = reg->base + TP1_M2M_RING_BASE;
		reg->reg_TP_M2M_RING_RP	= reg->base + TP1_M2M_RING_RP;
		reg->reg_TP_M2M_RING_WP	= reg->base + TP1_M2M_RING_WP;
		reg->reg_TP_M2M_RING_CTRL = reg->base + TP1_M2M_RING_CTRL;

		reg->reg_TP_RING_CTRL = reg->base + TP_RING_CTRL;
		reg->reg_TP_RING_LIMIT = reg->base + TP_RING_LIMIT;
		reg->reg_TP_RING_BASE = reg->base + TP_RING_BASE;
		reg->reg_TP_RING_RP = reg->base + TP_RING_RP;
		reg->reg_TP_RING_WP = reg->base + TP_RING_WP;
		reg->reg_TP_FULLNESS = reg->base + TP_FULLNESS;
		reg->reg_TP_THRESHOLD = reg->base + TP_THRESHOLD;
		reg->reg_TP_RING_FULL_INT_0 = reg->base + TP_RING_FULL_INT_0;
		reg->reg_TP_RING_FULL_INT_1 = reg->base + TP_RING_FULL_INT_1;
		reg->reg_TP_RING_FULL_INT_2 = reg->base + TP_RING_FULL_INT_2;
		reg->reg_TP_RING_FULL_INT_3 = reg->base + TP_RING_FULL_INT_3;
		reg->reg_TP_RING_AVAIL_INT_0 = reg->base +
						TP_RING_AVAIL_INT_0;
		reg->reg_TP_RING_AVAIL_INT_1 = reg->base +
						TP_RING_AVAIL_INT_1;
		reg->reg_TP_RING_AVAIL_INT_2 = reg->base +
						TP_RING_AVAIL_INT_2;
		reg->reg_TP_RING_AVAIL_INT_3 = reg->base +
						TP_RING_AVAIL_INT_3;
		reg->reg_TP_RING_AVAIL_INT_EN_0 = reg->base +
						TP_RING_AVAIL_INT_EN_0;
		reg->reg_TP_RING_AVAIL_INT_EN_1 = reg->base +
						TP_RING_AVAIL_INT_EN_1;
		reg->reg_TP_RING_AVAIL_INT_EN_2 = reg->base +
						TP_RING_AVAIL_INT_EN_2;
		reg->reg_TP_RING_AVAIL_INT_EN_3 = reg->base +
						TP_RING_AVAIL_INT_EN_3;
		reg->reg_TP_RING_FULL_INT_EN_0 = reg->base +
						TP_RING_FULL_INT_EN_0;
		reg->reg_TP_RING_FULL_INT_EN_1 = reg->base +
						TP_RING_FULL_INT_EN_1;
		reg->reg_TP_RING_FULL_INT_EN_2 = reg->base +
						TP_RING_FULL_INT_EN_2;
		reg->reg_TP_RING_FULL_INT_EN_3 = reg->base +
						TP_RING_FULL_INT_EN_3;

		reg->reg_TP_PID_CTRL	= reg->base + TP_PID_CTRL;
		reg->reg_TP_PID_DATA	= reg->base + TP_PID_DATA;
		reg->reg_TP_PID_DATA2	= reg->base + TP_PID_DATA2;
		reg->reg_TP_PID_DATA3	= reg->base + TP_PID_DATA3;
		reg->reg_TP_PCRCtrlReg = reg->base + TP1_PCR_CTL;
		reg->reg_TP_PCR_LATCH	= reg->base + TP_PCR_LATCH;
		reg->reg_TP_PCR_SYSTEM = reg->base + TP_PCR_SYSTEM;
		reg->reg_TP_PCR_BASE	= reg->base + TP_PCR_BASE;
		reg->reg_TP_PCR_EXT	= reg->base + TP_PCR_EXT;

		reg->reg_TP_SEC_CTRL  = reg->base + TP_SEC_CTRL;
		reg->reg_TP_SEC_DATA0 = reg->base + TP_SEC_DATA0;
		reg->reg_TP_SEC_DATA1 = reg->base + TP_SEC_DATA1;
		reg->reg_TP_SEC_DATA2 = reg->base + TP_SEC_DATA2;
		reg->reg_TP_SEC_DATA3 = reg->base + TP_SEC_DATA3;
		reg->reg_TP_SEC_DATA4 = reg->base + TP_SEC_DATA4;
		reg->reg_TP_SEC_DATA5 = reg->base + TP_SEC_DATA5;
		reg->reg_TP_SEC_DATA6 = reg->base + TP_SEC_DATA6;
		reg->reg_TP_SEC_DATA7 = reg->base + TP_SEC_DATA7;
		reg->reg_TP_SEC_DATA8 = reg->base + TP_SEC_DATA8;
		reg->reg_TP_SEC_DATA9 = reg->base + TP_SEC_DATA9;
		reg->reg_TP_SEC_DATA10 = reg->base + TP_SEC_DATA10;
		reg->reg_TP_SEC_DATA11 = reg->base + TP_SEC_DATA11;
		break;

	case DMX_TP_B_0:
		reg->reg_TF_CNTL = reg->base + TPB_TF0_CNTL;
		reg->reg_TF_CNT = reg->base + TPB_TF0_CNT;
		reg->reg_TF_DRP_CNT = reg->base + TPB_TF0_DRP_CNT;
		reg->reg_TF_ERR_CNT = reg->base + TPB_TF0_ERR_CNT;
		reg->reg_TF_FRMCFG = reg->base + TPB_TF0_FRMCFG;
		reg->reg_TF_INT = reg->base + TPB_TF0_INT;
		reg->reg_TF_INT_EN = reg->base + TPB_TF0_INT_EN;
		reg->reg_TF_STRM_ID_0 = reg->base + TPB_TF0_STRM_ID_0;
		reg->reg_TF_STRM_ID_1 = reg->base + TPB_TF0_STRM_ID_1;
		reg->reg_TF_STRM_ID_2 = reg->base + TPB_TF0_STRM_ID_2;
		reg->reg_TF_STRM_ID_3 = reg->base + TPB_TF0_STRM_ID_3;
		reg->reg_TF_STRM_ID_VAL = reg->base + TPB_TF0_STRM_ID_VAL;
		reg->reg_TP_DES_CNTL = reg->base + TPB_TP0_DES_CNTL;

		reg->reg_TP_TF_CTRL_SWC = reg->base + TPB_TF0_CTRL_SWC;

		reg->reg_TP_KEY_INFO_SWC_0 = reg->base + TP_KEY_INFO_SWC_0;
		reg->reg_TP_KEY_INFO_SWC_1 = reg->base + TP_KEY_INFO_SWC_1;
		reg->reg_TP_KEY_INFO_SWC_2 = reg->base + TP_KEY_INFO_SWC_2;
		reg->reg_TP_KEY_INFO_SWC_3 = reg->base + TP_KEY_INFO_SWC_3;
		reg->reg_TP_KEY_INFO_SWC_4 = reg->base + TP_KEY_INFO_SWC_4;
		reg->reg_TP_KEY_INFO_SWC_5 = reg->base + TP_KEY_INFO_SWC_5;
		reg->reg_TP_KEY_INFO_SWC_6 = reg->base + TP_KEY_INFO_SWC_6;
		reg->reg_TP_KEY_INFO_SWC_7 = reg->base + TP_KEY_INFO_SWC_7;
		reg->reg_TP_KEY_INFO_SWC_8 = reg->base + TP_KEY_INFO_SWC_8;
		reg->reg_TP_KEY_INFO_SWC_9 = reg->base + TP_KEY_INFO_SWC_9;
		reg->reg_TP_KEY_INFO_SWC_A = reg->base + TP_KEY_INFO_SWC_A;
		reg->reg_TP_KEY_INFO_SWC_B = reg->base + TP_KEY_INFO_SWC_B;

		reg->reg_TP_KEY_CTRL_SWC = reg->base + TP_KEY_CTRL_SWC;
		reg->reg_TP_KEY_HEADER_SWC = reg->base + TP_KEY_HEADER_SWC;
		reg->reg_TP_KEY_MASK_SWC = reg->base + TP_KEY_MASK_SWC;
		reg->reg_TP_SWC_DMY_A = reg->base + TPB_SWC_DMY_A;

		reg->reg_TP_PID_PART = reg->base + TPB_PID_PART;
		reg->reg_TP_CRC_INIT = reg->base + TPB_CRC_INIT;

		reg->reg_TP_EXT_PID_CNTL = reg->base + TPB_0_EXT_PID_CNTL;

		reg->reg_TP_M2M_RING_LIMIT = reg->base +
						TPB_TP0_M2M_RING_LIMIT;
		reg->reg_TP_M2M_RING_BASE = reg->base + TPB_TP0_M2M_RING_BASE;
		reg->reg_TP_M2M_RING_RP	= reg->base + TPB_TP0_M2M_RING_RP;
		reg->reg_TP_M2M_RING_WP	= reg->base + TPB_TP0_M2M_RING_WP;
		reg->reg_TP_M2M_RING_CTRL = reg->base + TPB_TP0_M2M_RING_CTRL;

		reg->reg_TP_RING_CTRL = reg->base + TPB_RING_CTRL;
		reg->reg_TP_RING_LIMIT = reg->base + TPB_RING_LIMIT;
		reg->reg_TP_RING_BASE = reg->base + TPB_RING_BASE;
		reg->reg_TP_RING_RP = reg->base + TPB_RING_RP;
		reg->reg_TP_RING_WP = reg->base + TPB_RING_WP;
		reg->reg_TP_FULLNESS = reg->base + TPB_FULLNESS;
		reg->reg_TP_THRESHOLD = reg->base + TPB_THRESHOLD;
		reg->reg_TP_RING_FULL_INT_0 = reg->base + TPB_RING_FULL_INT_0;
		reg->reg_TP_RING_FULL_INT_1 = reg->base + TPB_RING_FULL_INT_1;
		reg->reg_TP_RING_FULL_INT_2 = reg->base + TPB_RING_FULL_INT_2;
		reg->reg_TP_RING_FULL_INT_3 = reg->base + TPB_RING_FULL_INT_3;
		reg->reg_TP_RING_AVAIL_INT_0 = reg->base +
						TPB_RING_AVAIL_INT_0;
		reg->reg_TP_RING_AVAIL_INT_1 = reg->base +
						TPB_RING_AVAIL_INT_1;
		reg->reg_TP_RING_AVAIL_INT_2 = reg->base +
						TPB_RING_AVAIL_INT_2;
		reg->reg_TP_RING_AVAIL_INT_3 = reg->base +
						TPB_RING_AVAIL_INT_3;
		reg->reg_TP_RING_AVAIL_INT_EN_0 = reg->base +
						TPB_RING_AVAIL_INT_EN_0;
		reg->reg_TP_RING_AVAIL_INT_EN_1 = reg->base +
						TPB_RING_AVAIL_INT_EN_1;
		reg->reg_TP_RING_AVAIL_INT_EN_2 = reg->base +
						TPB_RING_AVAIL_INT_EN_2;
		reg->reg_TP_RING_AVAIL_INT_EN_3 = reg->base +
						TPB_RING_AVAIL_INT_EN_3;
		reg->reg_TP_RING_FULL_INT_EN_0 = reg->base +
						TPB_RING_FULL_INT_EN_0;
		reg->reg_TP_RING_FULL_INT_EN_1 = reg->base +
						TPB_RING_FULL_INT_EN_1;
		reg->reg_TP_RING_FULL_INT_EN_2 = reg->base +
						TPB_RING_FULL_INT_EN_2;
		reg->reg_TP_RING_FULL_INT_EN_3 = reg->base +
						TPB_RING_FULL_INT_EN_3;

		reg->reg_TP_PID_CTRL = reg->base + TPB_PID_CTRL;
		reg->reg_TP_PID_DATA = reg->base + TPB_PID_DATA;
		reg->reg_TP_PID_DATA2 = reg->base + TPB_PID_DATA2;
		reg->reg_TP_PID_DATA3 = reg->base + TPB_PID_DATA3;
		reg->reg_TP_PCRCtrlReg = reg->base + TPB0_PCR_CTL;
		reg->reg_TP_PCR_LATCH = reg->base + TPB_PCR_LATCH;
		reg->reg_TP_PCR_SYSTEM = reg->base + TPB_PCR_SYSTEM;
		reg->reg_TP_PCR_BASE = reg->base + TPB_PCR_BASE;
		reg->reg_TP_PCR_EXT = reg->base + TPB_PCR_EXT;

		reg->reg_TP_SEC_CTRL = reg->base + TPB_SEC_CTRL;
		reg->reg_TP_SEC_DATA0 = reg->base + TPB_SEC_DATA0;
		reg->reg_TP_SEC_DATA1 = reg->base + TPB_SEC_DATA1;
		reg->reg_TP_SEC_DATA2 = reg->base + TPB_SEC_DATA2;
		reg->reg_TP_SEC_DATA3 = reg->base + TPB_SEC_DATA3;
		reg->reg_TP_SEC_DATA4 = reg->base + TPB_SEC_DATA4;
		reg->reg_TP_SEC_DATA5 = reg->base + TPB_SEC_DATA5;
		reg->reg_TP_SEC_DATA6 = reg->base + TPB_SEC_DATA6;
		reg->reg_TP_SEC_DATA7 = reg->base + TPB_SEC_DATA7;
		reg->reg_TP_SEC_DATA8 = reg->base + TPB_SEC_DATA8;
		reg->reg_TP_SEC_DATA9 = reg->base + TPB_SEC_DATA9;
		reg->reg_TP_SEC_DATA10 = reg->base + TPB_SEC_DATA10;
		reg->reg_TP_SEC_DATA11 = reg->base + TPB_SEC_DATA11;
		break;

	case DMX_TP_B_1:
		reg->reg_TF_CNTL = reg->base + TPB_TF1_CNTL;
		reg->reg_TF_CNT = reg->base + TPB_TF1_CNT;
		reg->reg_TF_DRP_CNT = reg->base + TPB_TF1_DRP_CNT;
		reg->reg_TF_ERR_CNT = reg->base + TPB_TF1_ERR_CNT;
		reg->reg_TF_FRMCFG = reg->base + TPB_TF1_FRMCFG;
		reg->reg_TF_INT = reg->base + TPB_TF1_INT;
		reg->reg_TF_INT_EN = reg->base + TPB_TF1_INT_EN;
		reg->reg_TF_STRM_ID_0 = reg->base + TPB_TF1_STRM_ID_0;
		reg->reg_TF_STRM_ID_1 = reg->base + TPB_TF1_STRM_ID_1;
		reg->reg_TF_STRM_ID_2 = reg->base + TPB_TF1_STRM_ID_2;
		reg->reg_TF_STRM_ID_3 = reg->base + TPB_TF1_STRM_ID_3;
		reg->reg_TF_STRM_ID_VAL = reg->base + TPB_TF1_STRM_ID_VAL;
		reg->reg_TP_DES_CNTL = reg->base + TPB_TP1_DES_CNTL;

		reg->reg_TP_TF_CTRL_SWC = reg->base + TPB_TF1_CTRL_SWC;

		reg->reg_TP_KEY_INFO_SWC_0 = reg->base + TP_KEY_INFO_SWC_0;
		reg->reg_TP_KEY_INFO_SWC_1 = reg->base + TP_KEY_INFO_SWC_1;
		reg->reg_TP_KEY_INFO_SWC_2 = reg->base + TP_KEY_INFO_SWC_2;
		reg->reg_TP_KEY_INFO_SWC_3 = reg->base + TP_KEY_INFO_SWC_3;
		reg->reg_TP_KEY_INFO_SWC_4 = reg->base + TP_KEY_INFO_SWC_4;
		reg->reg_TP_KEY_INFO_SWC_5 = reg->base + TP_KEY_INFO_SWC_5;
		reg->reg_TP_KEY_INFO_SWC_6 = reg->base + TP_KEY_INFO_SWC_6;
		reg->reg_TP_KEY_INFO_SWC_7 = reg->base + TP_KEY_INFO_SWC_7;
		reg->reg_TP_KEY_INFO_SWC_8 = reg->base + TP_KEY_INFO_SWC_8;
		reg->reg_TP_KEY_INFO_SWC_9 = reg->base + TP_KEY_INFO_SWC_9;
		reg->reg_TP_KEY_INFO_SWC_A = reg->base + TP_KEY_INFO_SWC_A;
		reg->reg_TP_KEY_INFO_SWC_B = reg->base + TP_KEY_INFO_SWC_B;

		reg->reg_TP_KEY_CTRL_SWC = reg->base + TP_KEY_CTRL_SWC;
		reg->reg_TP_KEY_HEADER_SWC = reg->base + TP_KEY_HEADER_SWC;
		reg->reg_TP_KEY_MASK_SWC = reg->base + TP_KEY_MASK_SWC;
		reg->reg_TP_SWC_DMY_A = reg->base + TPB_SWC_DMY_A;

		reg->reg_TP_PID_PART = reg->base + TPB_PID_PART;
		reg->reg_TP_CRC_INIT = reg->base + TPB_CRC_INIT;

		reg->reg_TP_EXT_PID_CNTL = reg->base + TPB_1_EXT_PID_CNTL;

		reg->reg_TP_M2M_RING_LIMIT = reg->base +
						TPB_TP1_M2M_RING_LIMIT;
		reg->reg_TP_M2M_RING_BASE = reg->base + TPB_TP1_M2M_RING_BASE;
		reg->reg_TP_M2M_RING_RP	= reg->base + TPB_TP1_M2M_RING_RP;
		reg->reg_TP_M2M_RING_WP	= reg->base + TPB_TP1_M2M_RING_WP;
		reg->reg_TP_M2M_RING_CTRL = reg->base + TPB_TP1_M2M_RING_CTRL;

		reg->reg_TP_RING_CTRL = reg->base + TPB_RING_CTRL;
		reg->reg_TP_RING_LIMIT = reg->base + TPB_RING_LIMIT;
		reg->reg_TP_RING_BASE = reg->base + TPB_RING_BASE;
		reg->reg_TP_RING_RP = reg->base + TPB_RING_RP;
		reg->reg_TP_RING_WP = reg->base + TPB_RING_WP;
		reg->reg_TP_FULLNESS = reg->base + TPB_FULLNESS;
		reg->reg_TP_THRESHOLD = reg->base + TPB_THRESHOLD;
		reg->reg_TP_RING_FULL_INT_0 = reg->base + TPB_RING_FULL_INT_0;
		reg->reg_TP_RING_FULL_INT_1 = reg->base + TPB_RING_FULL_INT_1;
		reg->reg_TP_RING_FULL_INT_2 = reg->base + TPB_RING_FULL_INT_2;
		reg->reg_TP_RING_FULL_INT_3 = reg->base + TPB_RING_FULL_INT_3;
		reg->reg_TP_RING_AVAIL_INT_0 = reg->base +
						TPB_RING_AVAIL_INT_0;
		reg->reg_TP_RING_AVAIL_INT_1 = reg->base +
						TPB_RING_AVAIL_INT_1;
		reg->reg_TP_RING_AVAIL_INT_2 = reg->base +
						TPB_RING_AVAIL_INT_2;
		reg->reg_TP_RING_AVAIL_INT_3 = reg->base +
						TPB_RING_AVAIL_INT_3;
		reg->reg_TP_RING_AVAIL_INT_EN_0 = reg->base +
						TPB_RING_AVAIL_INT_EN_0;
		reg->reg_TP_RING_AVAIL_INT_EN_1 = reg->base +
						TPB_RING_AVAIL_INT_EN_1;
		reg->reg_TP_RING_AVAIL_INT_EN_2 = reg->base +
						TPB_RING_AVAIL_INT_EN_2;
		reg->reg_TP_RING_AVAIL_INT_EN_3 = reg->base +
						TPB_RING_AVAIL_INT_EN_3;
		reg->reg_TP_RING_FULL_INT_EN_0 = reg->base +
						TPB_RING_FULL_INT_EN_0;
		reg->reg_TP_RING_FULL_INT_EN_1 = reg->base +
						TPB_RING_FULL_INT_EN_1;
		reg->reg_TP_RING_FULL_INT_EN_2 = reg->base +
						TPB_RING_FULL_INT_EN_2;
		reg->reg_TP_RING_FULL_INT_EN_3 = reg->base +
						TPB_RING_FULL_INT_EN_3;

		reg->reg_TP_PID_CTRL	= reg->base + TPB_PID_CTRL;
		reg->reg_TP_PID_DATA	= reg->base + TPB_PID_DATA;
		reg->reg_TP_PID_DATA2	= reg->base + TPB_PID_DATA2;
		reg->reg_TP_PID_DATA3	= reg->base + TPB_PID_DATA3;
		reg->reg_TP_PCRCtrlReg = reg->base + TPB1_PCR_CTL;
		reg->reg_TP_PCR_LATCH	= reg->base + TPB_PCR_LATCH;
		reg->reg_TP_PCR_SYSTEM = reg->base + TPB_PCR_SYSTEM;
		reg->reg_TP_PCR_BASE	= reg->base + TPB_PCR_BASE;
		reg->reg_TP_PCR_EXT	= reg->base + TPB_PCR_EXT;

		reg->reg_TP_SEC_CTRL  = reg->base + TPB_SEC_CTRL;
		reg->reg_TP_SEC_DATA0 = reg->base + TPB_SEC_DATA0;
		reg->reg_TP_SEC_DATA1 = reg->base + TPB_SEC_DATA1;
		reg->reg_TP_SEC_DATA2 = reg->base + TPB_SEC_DATA2;
		reg->reg_TP_SEC_DATA3 = reg->base + TPB_SEC_DATA3;
		reg->reg_TP_SEC_DATA4 = reg->base + TPB_SEC_DATA4;
		reg->reg_TP_SEC_DATA5 = reg->base + TPB_SEC_DATA5;
		reg->reg_TP_SEC_DATA6 = reg->base + TPB_SEC_DATA6;
		reg->reg_TP_SEC_DATA7 = reg->base + TPB_SEC_DATA7;
		reg->reg_TP_SEC_DATA8 = reg->base + TPB_SEC_DATA8;
		reg->reg_TP_SEC_DATA9 = reg->base + TPB_SEC_DATA9;
		reg->reg_TP_SEC_DATA10 = reg->base + TPB_SEC_DATA10;
		reg->reg_TP_SEC_DATA11 = reg->base + TPB_SEC_DATA11;
		break;
#if defined(CONFIG_ARCH_RTD13xx)
	case DMX_TP_C_0:
		reg->reg_TF_CNTL = reg->base + TPC_TF0_CNTL;
		reg->reg_TF_CNT = reg->base + TPC_TF0_CNT;
		reg->reg_TF_DRP_CNT = reg->base + TPC_TF0_DRP_CNT;
		reg->reg_TF_ERR_CNT = reg->base + TPC_TF0_ERR_CNT;
		reg->reg_TF_FRMCFG = reg->base + TPC_TF0_FRMCFG;
		reg->reg_TF_INT = reg->base + TPC_TF0_INT;
		reg->reg_TF_INT_EN = reg->base + TPC_TF0_INT_EN;
		reg->reg_TF_STRM_ID_0 = reg->base + TPC_TF0_STRM_ID_0;
		reg->reg_TF_STRM_ID_1 = reg->base + TPC_TF0_STRM_ID_1;
		reg->reg_TF_STRM_ID_2 = reg->base + TPC_TF0_STRM_ID_2;
		reg->reg_TF_STRM_ID_3 = reg->base + TPC_TF0_STRM_ID_3;
		reg->reg_TF_STRM_ID_VAL = reg->base + TPC_TF0_STRM_ID_VAL;

		reg->reg_TP_TF_CTRL_SWC = reg->base + TPC_TF0_CTRL_SWC;

		reg->reg_TP_PID_PART = reg->base + TPC_PID_PART;
		reg->reg_TP_CRC_INIT = reg->base + TPC_CRC_INIT;

		reg->reg_TP_EXT_PID_CNTL = reg->base + TPC_0_EXT_PID_CNTL;

		reg->reg_TP_M2M_RING_LIMIT = reg->base +
						TPC_TP0_M2M_RING_LIMIT;
		reg->reg_TP_M2M_RING_BASE = reg->base + TPC_TP0_M2M_RING_BASE;
		reg->reg_TP_M2M_RING_RP	= reg->base + TPC_TP0_M2M_RING_RP;
		reg->reg_TP_M2M_RING_WP	= reg->base + TPC_TP0_M2M_RING_WP;
		reg->reg_TP_M2M_RING_CTRL = reg->base + TPC_TP0_M2M_RING_CTRL;

		reg->reg_TP_RING_CTRL = reg->base + TPC_RING_CTRL;
		reg->reg_TP_RING_LIMIT	= reg->base + TPC_RING_LIMIT;
		reg->reg_TP_RING_BASE		= reg->base + TPC_RING_BASE;
		reg->reg_TP_RING_RP		= reg->base + TPC_RING_RP;
		reg->reg_TP_RING_WP		= reg->base + TPC_RING_WP;
		reg->reg_TP_FULLNESS		= reg->base + TPC_FULLNESS;
		reg->reg_TP_THRESHOLD		= reg->base + TPC_THRESHOLD;
		reg->reg_TP_RING_FULL_INT_0 = reg->base + TPC_RING_FULL_INT_0;
		reg->reg_TP_RING_FULL_INT_1 = reg->base + TPC_RING_FULL_INT_1;
		reg->reg_TP_RING_FULL_INT_2 = reg->base + TPC_RING_FULL_INT_2;
		reg->reg_TP_RING_FULL_INT_3 = reg->base + TPC_RING_FULL_INT_3;
		reg->reg_TP_RING_AVAIL_INT_0 = reg->base +
						TPC_RING_AVAIL_INT_0;
		reg->reg_TP_RING_AVAIL_INT_1 = reg->base +
						TPC_RING_AVAIL_INT_1;
		reg->reg_TP_RING_AVAIL_INT_2 = reg->base +
						TPC_RING_AVAIL_INT_2;
		reg->reg_TP_RING_AVAIL_INT_3 = reg->base +
						TPC_RING_AVAIL_INT_3;
		reg->reg_TP_RING_AVAIL_INT_EN_0 = reg->base +
						TPC_RING_AVAIL_INT_EN_0;
		reg->reg_TP_RING_AVAIL_INT_EN_1 = reg->base +
						TPC_RING_AVAIL_INT_EN_1;
		reg->reg_TP_RING_AVAIL_INT_EN_2 = reg->base +
						TPC_RING_AVAIL_INT_EN_2;
		reg->reg_TP_RING_AVAIL_INT_EN_3 = reg->base +
						TPC_RING_AVAIL_INT_EN_3;
		reg->reg_TP_RING_FULL_INT_EN_0 = reg->base +
						TPC_RING_FULL_INT_EN_0;
		reg->reg_TP_RING_FULL_INT_EN_1 = reg->base +
						TPC_RING_FULL_INT_EN_1;
		reg->reg_TP_RING_FULL_INT_EN_2 = reg->base +
						TPC_RING_FULL_INT_EN_2;
		reg->reg_TP_RING_FULL_INT_EN_3 = reg->base +
						TPC_RING_FULL_INT_EN_3;

		reg->reg_TP_PID_CTRL	= reg->base + TPC_PID_CTRL;
		reg->reg_TP_PID_DATA	= reg->base + TPC_PID_DATA;
		reg->reg_TP_PID_DATA2	= reg->base + TPC_PID_DATA2;
		reg->reg_TP_PID_DATA3	= reg->base + TPC_PID_DATA3;
		reg->reg_TP_PCRCtrlReg = reg->base + TPC0_PCR_CTL;
		reg->reg_TP_PCR_LATCH	= reg->base + TPC_PCR_LATCH;
		reg->reg_TP_PCR_SYSTEM = reg->base + TPC_PCR_SYSTEM;
		reg->reg_TP_PCR_BASE	= reg->base + TPC_PCR_BASE;
		reg->reg_TP_PCR_EXT	= reg->base + TPC_PCR_EXT;

		reg->reg_TP_SEC_CTRL  = reg->base + TPC_SEC_CTRL;
		reg->reg_TP_SEC_DATA0 = reg->base + TPC_SEC_DATA0;
		reg->reg_TP_SEC_DATA1 = reg->base + TPC_SEC_DATA1;
		reg->reg_TP_SEC_DATA2 = reg->base + TPC_SEC_DATA2;
		reg->reg_TP_SEC_DATA3 = reg->base + TPC_SEC_DATA3;
		reg->reg_TP_SEC_DATA4 = reg->base + TPC_SEC_DATA4;
		reg->reg_TP_SEC_DATA5 = reg->base + TPC_SEC_DATA5;
		reg->reg_TP_SEC_DATA6 = reg->base + TPC_SEC_DATA6;
		reg->reg_TP_SEC_DATA7 = reg->base + TPC_SEC_DATA7;
		reg->reg_TP_SEC_DATA8 = reg->base + TPC_SEC_DATA8;
		reg->reg_TP_SEC_DATA9 = reg->base + TPC_SEC_DATA9;
		reg->reg_TP_SEC_DATA10 = reg->base + TPC_SEC_DATA10;
		reg->reg_TP_SEC_DATA11 = reg->base + TPC_SEC_DATA11;
		break;
#endif
	default:
		break;
	}
}

