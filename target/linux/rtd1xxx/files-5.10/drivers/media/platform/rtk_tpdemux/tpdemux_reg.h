/*
 * tpdemux_reg.h
 *
 * Copyright (C) 2020 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _TPDEMUX_REG_H_
#define _TPDEMUX_REG_H_

#include <linux/io.h>
#include <linux/mutex.h>

#include "inc/hank_tp_reg.h"

enum TP_ID {
	TP_A = 0,
	TP_B,
	TP_C,
	TP_MAX,
};


enum DMX_TP_ID {
	DMX_TP_A_0 = 0,
	DMX_TP_A_1,
	DMX_TP_B_0,
	DMX_TP_B_1,
	DMX_TP_C_0,
	DMX_TP_MAX,
};

struct rtk_tp_reg {
	struct mutex buf_ctrl_mutex;
	struct mutex pid_ctrl_mutex;

	void __iomem *base;

	void __iomem *reg_TF_CNTL;
	void __iomem *reg_TF_CNT;
	void __iomem *reg_TF_DRP_CNT;
	void __iomem *reg_TF_ERR_CNT;
	void __iomem *reg_TF_FRMCFG;
	void __iomem *reg_TF_INT;
	void __iomem *reg_TF_INT_EN;

	void __iomem *reg_TP_DES_CNTL;
	void __iomem *reg_TP_KEY_INFO_H;
	void __iomem *reg_TP_KEY_INFO_L;
	void __iomem *reg_TP_KEY_CTRL;
	void __iomem *reg_TP_KEYA_MSB_0;
	void __iomem *reg_TP_KEYA_MSB_1;
	void __iomem *reg_TP_KEYA_LSB_0;
	void __iomem *reg_TP_KEYA_LSB_1;
	void __iomem *reg_TP_KEYB_MSB_0;
	void __iomem *reg_TP_KEYB_MSB_1;
	void __iomem *reg_TP_KEYB_LSB_0;
	void __iomem *reg_TP_KEYB_LSB_1;
	void __iomem *reg_TP_KEYC_MSB_0;
	void __iomem *reg_TP_KEYC_MSB_1;
	void __iomem *reg_TP_KEYC_LSB_0;
	void __iomem *reg_TP_KEYC_LSB_1;
	void __iomem *reg_TP_IV_MSB_0;
	void __iomem *reg_TP_IV_MSB_1;
	void __iomem *reg_TP_IV_LSB_0;
	void __iomem *reg_TP_IV_LSB_1;

	void __iomem *reg_TP_TF_CTRL_SWC;

	void __iomem *reg_TP_KEY_MASK_SWC;
	void __iomem *reg_TP_KEY_HEADER_SWC;
	void __iomem *reg_TP_KEY_CTRL_SWC;

	void __iomem *reg_TP_KEY_INFO_SWC_0;
	void __iomem *reg_TP_KEY_INFO_SWC_1;
	void __iomem *reg_TP_KEY_INFO_SWC_2;
	void __iomem *reg_TP_KEY_INFO_SWC_3;
	void __iomem *reg_TP_KEY_INFO_SWC_4;
	void __iomem *reg_TP_KEY_INFO_SWC_5;
	void __iomem *reg_TP_KEY_INFO_SWC_6;
	void __iomem *reg_TP_KEY_INFO_SWC_7;
	void __iomem *reg_TP_KEY_INFO_SWC_8;
	void __iomem *reg_TP_KEY_INFO_SWC_9;
	void __iomem *reg_TP_KEY_INFO_SWC_A;
	void __iomem *reg_TP_KEY_INFO_SWC_B;

	void __iomem *reg_TP_SWC_DMY_A;

	void __iomem *reg_TF_CHSEL;
	void __iomem *reg_TF_STRM_ID_0;
	void __iomem *reg_TF_STRM_ID_1;
	void __iomem *reg_TF_STRM_ID_2;
	void __iomem *reg_TF_STRM_ID_3;
	void __iomem *reg_TF_STRM_ID_VAL;

	void __iomem *reg_TP_PID_PART;
	void __iomem *reg_TP_CRC_INIT;

	void __iomem *reg_TP_EXT_PID_CNTL;

	void __iomem *reg_TP_M2M_RING_LIMIT;
	void __iomem *reg_TP_M2M_RING_BASE;
	void __iomem *reg_TP_M2M_RING_RP;
	void __iomem *reg_TP_M2M_RING_WP;
	void __iomem *reg_TP_M2M_RING_CTRL;

	void __iomem *reg_TP_RING_CTRL;
	void __iomem *reg_TP_RING_LIMIT;
	void __iomem *reg_TP_RING_BASE;
	void __iomem *reg_TP_RING_RP;
	void __iomem *reg_TP_RING_WP;
	void __iomem *reg_TP_FULLNESS;
	void __iomem *reg_TP_THRESHOLD;

	void __iomem *reg_TP_RING_FULL_INT_0;
	void __iomem *reg_TP_RING_FULL_INT_1;
	void __iomem *reg_TP_RING_FULL_INT_2;
	void __iomem *reg_TP_RING_FULL_INT_3;

	void __iomem *reg_TP_RING_AVAIL_INT_0;
	void __iomem *reg_TP_RING_AVAIL_INT_1;
	void __iomem *reg_TP_RING_AVAIL_INT_2;
	void __iomem *reg_TP_RING_AVAIL_INT_3;

	void __iomem *reg_TP_RING_AVAIL_INT_EN_0;
	void __iomem *reg_TP_RING_AVAIL_INT_EN_1;
	void __iomem *reg_TP_RING_AVAIL_INT_EN_2;
	void __iomem *reg_TP_RING_AVAIL_INT_EN_3;

	void __iomem *reg_TP_RING_FULL_INT_EN_0;
	void __iomem *reg_TP_RING_FULL_INT_EN_1;
	void __iomem *reg_TP_RING_FULL_INT_EN_2;
	void __iomem *reg_TP_RING_FULL_INT_EN_3;

	void __iomem *reg_TP_PID_CTRL;
	void __iomem *reg_TP_PID_DATA;
	void __iomem *reg_TP_PID_DATA2;
	void __iomem *reg_TP_PID_DATA3;

	void __iomem *reg_TP_PCRCtrlReg;

	void __iomem *reg_TP_PCR_LATCH;
	void __iomem *reg_TP_PCR_SYSTEM;
	void __iomem *reg_TP_PCR_BASE;
	void __iomem *reg_TP_PCR_EXT;

	void __iomem *reg_TP_SEC_CTRL;
	void __iomem *reg_TP_SEC_DATA0;
	void __iomem *reg_TP_SEC_DATA1;
	void __iomem *reg_TP_SEC_DATA2;
	void __iomem *reg_TP_SEC_DATA3;
	void __iomem *reg_TP_SEC_DATA4;
	void __iomem *reg_TP_SEC_DATA5;
	void __iomem *reg_TP_SEC_DATA6;
	void __iomem *reg_TP_SEC_DATA7;
	void __iomem *reg_TP_SEC_DATA8;
	void __iomem *reg_TP_SEC_DATA9;
	void __iomem *reg_TP_SEC_DATA10;
	void __iomem *reg_TP_SEC_DATA11;
};

#define write_reg32(addr, val) writel(val, (unsigned int *)(addr))
#define read_reg32(addr) readl((unsigned int *)(addr))

void tp_reg_mapping(int tp_index, struct rtk_tp_reg *reg);

#endif /* _TPDEMUX_REG_H_ */
