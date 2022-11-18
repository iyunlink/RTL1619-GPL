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
#include <linux/regmap.h>

#include "rtk_dptx_core.h"
#include "rtk_dptx_reg.h"

const unsigned char DPTX_DRV_TABLE[48] = {
//Emphasis->  0                 1                 2                 3
//Voltage
/*0*/  0x00, 0x00, 0x03, 0x00, 0x07, 0x05, 0x00, 0x0C, 0x05, 0x01, 0x09, 0x06,
/*1*/  0x00, 0x00, 0x05, 0x00, 0x07, 0x05, 0x00, 0x0C, 0x05, 0x00, 0x0C, 0x05,
/*2*/  0x00, 0x00, 0x09, 0x00, 0x0C, 0x09, 0x00, 0x0C, 0x09, 0x00, 0x0C, 0x09,
/*3*/  0x00, 0x00, 0x0E, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x0E,
};

static void writebit(void __iomem *base, u32 offset, u32 mask, u32 value)
{
	u32 val;

	val = readl(base + offset);
	val &= ~mask;
	val |= value & mask;
	writel(val, base + offset);
}

void dptx_set_sst_setting(struct rtk_dptx *dptx)
{
	void __iomem *base = dptx->edpreg;
	void __iomem *base2 = dptx->lvdsreg;

	bool hwmode = true;
	unsigned char hwmeacnt = 0;
	unsigned short loop = 0;
	unsigned short measuretime = 0;
	unsigned int hwmin = 0xFFFFFFFF;
	unsigned int hwmax = 0x00000000;
	unsigned int data[6];
	int i;
	int pixelclk = 148;

	data[0] = 0;
	data[1] = 0x8000;
	data[2] = 0;
	data[3] = 0;

	data[0] = pixelclk * data[1];
	data[0] /= 270;

	measuretime = (unsigned short)(data[1] / 27 / 5);
	measuretime *= 2;

	for (i = 0; i < 10; i++) {
		// Enable HW Mvid measure
		writebit(base, MN_VID_AUTO_EN_1, BIT7, BIT7);
		for (loop = 0; loop < measuretime; loop++)
			udelay(5);

		writebit(base, MN_VID_AUTO_EN_1, BIT7, 0x00);

		data[2] = ((readl(base + MVID_AUTO_H) << 16)
			| (readl(base + MVID_AUTO_M) << 8)
			| readl(base + MVID_AUTO_L));

		if (data[2] == 0) {
			writel(*((unsigned char *)data+2), base + MN_M_VID_H);
			writel(*((unsigned char *)data+1), base + MN_M_VID_M);
			writel(*((unsigned char *)data), base + MN_M_VID_L);

			hwmode = false;
			break;
		}
		if (abs(data[0] - data[2]) > data[0] >> 2)
			continue;
		if (data[3] == 0) {
			if (data[2] < hwmin)
				hwmin = data[2];
			if (data[2] > hwmax)
				hwmax = data[2];
			data[3] = data[2];
			hwmeacnt++;
			continue;
		}
		if (abs(((data[3] + (hwmeacnt / 2)) /
			hwmeacnt) - data[2]) < 0x50) {
			if (data[2] < hwmin)
				hwmin = data[2];
			if (data[2] > hwmax)
				hwmax = data[2];
			data[3] += data[2];
			hwmeacnt++;
		}
	}
	if (hwmode == true) {
		if (hwmeacnt > 2) {
			data[3] -= hwmax + hwmin;
			hwmeacnt -= 2;
			data[3] = ((data[3] + (hwmeacnt / 2))
					/ hwmeacnt);
			if (abs(data[0] - data[3]) > (data[0] >> 1))
				data[3] = data[0];
		} else
			data[3] = data[0];

		writel(*((unsigned char *)data+14), base + MN_M_VID_H);
		writel(*((unsigned char *)data+13), base + MN_M_VID_M);
		writel(*((unsigned char *)data+12), base + MN_M_VID_L);
	}
	writel(*((unsigned char *)data+6), base + MN_N_VID_H);
	writel(*((unsigned char *)data+5), base + MN_N_VID_M);
	writel(*((unsigned char *)data+4), base + MN_N_VID_L);
	writebit(base, MSA_CTRL, BIT6, 0x00);
	writebit(base, MN_VID_AUTO_EN_1, (BIT7 | BIT6), BIT6);

	// set sstmsa
	writebit(base, DP_RESET_CTRL, BIT6, BIT6);
	writebit(base, DP_RESET_CTRL, BIT6, 0x00);

	writel(0x8, base + MN_STRM_ATTR_HTT_M);
	writel(0x98, base + MN_STRM_ATTR_HTT_L);
	writel(0x00, base + MN_STRM_ATTR_HST_M);
	writel(0xC1, base + MN_STRM_ATTR_HST_L);
	writel(0x7, base + MN_STRM_ATTR_HWD_M);
	writel(0x80, base + MN_STRM_ATTR_HWD_L);
	writel(0x0, base + MN_STRM_ATTR_HSW_M);
	writel(0x2C, base + MN_STRM_ATTR_HSW_L);
	writel(0x4, base + MN_STRM_ATTR_VTTE_M);
	writel(0x65, base + MN_STRM_ATTR_VTTE_L);
	writel(0x2A, base + MN_STRM_ATTR_VST_L);
	writel(0x4, base + MN_STRM_ATTR_VHT_M);
	writel(0x38, base + MN_STRM_ATTR_VHT_L);
	writel(0x0, base + MN_STRM_ATTR_VSW_M);
	writel(0x5, base + MN_STRM_ATTR_VSW_L);
	writel(0x20, base + MSA_MISC0);
	writebit(base, MSA_CTRL, BIT7, BIT7);

	writebit(base, LFIFO_WL_SET, BIT7, 0x00);
	writebit(base, DP_RESET_CTRL, BIT7, BIT7);
	writebit(base, DP_RESET_CTRL, BIT7, 0);

	writel(0x35, base + TU_DATA_SIZE0);
	writel(0x0, base + TU_DATA_SIZE1);
	writel(0xb, base + V_DATA_PER_LINE0);
	writel(0x40, base + V_DATA_PER_LINE1);
	writel(0xc0, base + LFIFO_WL_SET);

	writel(0xff, base + PG_FIFO_CTRL);
	writel(0x4, base + VBID);
	writel(0, base + ARBITER_SEC_END_CNT_HB);

	writel(0x38, base + DP_PHY_CTRL);
	writel(0x6, base + DP_MAC_CTRL);
	writel(0x3, base2 + AIF_EDP_1);

	writel(0x80, base + DPTX_SFIFO_CTRL0);
	writel(0x15, base + DPTX_PHY_CTRL);
}

void dptx_set_video_timing(struct rtk_dptx *dptx, struct drm_display_mode *mode)
{
	void __iomem *base2 = dptx->edpreg;
	void __iomem *base = dptx->lvdsreg;

	writel(0x1, base2 + VBID_FW_CTL);

	writel(0x0, base + CT_CTRL);
	writel(0x2C, base + DH_WIDTH);
	writel(0x8980898, base + DH_TOTAL);
	writel(0xb70837, base + DH_DEN_START_END);
	writel(0x2A0462, base + DV_DEN_START_END_F1);
	writel(0x465, base + DV_TOTAL);
	writel(0x10006, base + DV_VS_START_END_F1);
//	writel(0x80000463, base + DV_SYNC_INT);
	writel(0x00000463, base + DV_SYNC_INT);
}

void dptx_set_training_lane(struct rtk_dptx *dptx)
{
	void __iomem *base = dptx->lvdsreg;
	int swing, emphasis;
	int lane, lane_count;
	u32 val;
	u8 *tbl, idx;
	u8 *adjust;

	tbl = (u8 *)&DPTX_DRV_TABLE;
	adjust = dptx->lt_info.adjust;
	lane_count = dptx->lt_info.lane_count;

	for (lane = 0; lane < lane_count; lane++) {
		swing = VOLTAGE_SWING_GET(adjust[lane]);
		emphasis = PRE_EMPHASIS_GET(adjust[lane]);

		if ((swing + emphasis) > 3)
			emphasis = 3 - swing;

		idx = (swing * 4 + emphasis) * 3;

		val = readl(base + AIF_EDP_2);
		val &= ~((LANE0_EMPHASIS_MASK1 << (lane * 4)) |
			(LANE0_EMPHASIS_MASK2) << (lane));
		val |= (tbl[idx] << (LANE0_EMPHASIS_2X + lane)) |
			(tbl[idx + 1] << (lane * 4)) |
			(LANE0_EMPHASIS_EN << lane);
		writel(val, base + AIF_EDP_2);

		val = readl(base + AIF_EDP_3);
		val &= ~((LANE0_DRV_MASK << (lane * 4)) | SEL_IBX);
		val |= tbl[idx + 2] << (lane * 4) | (LANE0_DRV_2X << lane);
		writel(val, base + AIF_EDP_3);
	}
}

void dptx_set_training_pattern(struct rtk_dptx *dptx, enum pattern_set pattern)
{
	void __iomem *base = dptx->edpreg;

	writel(pattern << PAT_SEL_SHIFT, base + DPTX_ML_PAT_SEL);
	writebit(base, DPTX_ML_PAT_SEL, PAT_CHANGE, PAT_CHANGE);
}

int dptx_aux_isr(struct rtk_dptx *dptx)
{
	void __iomem *base = dptx->edpreg;
	u32 val;
	int ret;

	val = readl(base + AUX_IRQ_EVENT);
	if (val & AUXDONE)
		ret = 0;
	else
		ret = -1;

	writel(AUX_ALL_IRQ, base + AUX_IRQ_EVENT);
	writebit(base, AUX_FIFO_CTRL, NA_FIFO_RST | I2C_FIFO_RST,
		 NA_FIFO_RST | I2C_FIFO_RST);

	return ret;
}

void dptx_aux_get_data(struct rtk_dptx *dptx, struct drm_dp_aux_msg *msg)
{
	void __iomem *base = dptx->edpreg;
	size_t size = msg->size;
	u8 *buffer = msg->buffer;
	int i;

	for (i = 0; i < size; i++)
		buffer[i] = readl(base + AUX_REPLY_DATA);
}

void dptx_aux_transfer(struct rtk_dptx *dptx, struct drm_dp_aux_msg *msg)
{
	void __iomem *base = dptx->edpreg;
	size_t size = msg->size;
	u32 addr = msg->address;
	u8 *buffer = msg->buffer;
	u8 data;
	int i;

	switch (msg->request & ~DP_AUX_I2C_MOT) {
	case DP_AUX_I2C_WRITE:
		if ((msg->request & DP_AUX_I2C_MOT) && size != 0)
			addr |= (0x1 << 6) << 16;
		if (size == 0) {
			size = 1;
			data = 0;
			buffer = &data;
		}
		break;
	case DP_AUX_I2C_READ:
		if (msg->request & DP_AUX_I2C_MOT)
			addr |= (0x10 | (0x1 << 6)) << 16;
		else
			addr |= 0x10 << 16;
		break;
	case DP_AUX_NATIVE_WRITE:
		addr |= 0x80 << 16;
		break;
	case DP_AUX_NATIVE_READ:
		addr |= 0x90 << 16;
		break;
	default:
		pr_err("transfer command not support !!!\n");
		return;
	}

	writebit(base, AUX_FIFO_CTRL, NA_FIFO_RST | I2C_FIFO_RST,
		 NA_FIFO_RST | I2C_FIFO_RST);
	writel(AUX_ALL_IRQ, base + AUX_IRQ_EVENT);

	writel((addr >> 16) & 0xFF, base + AUXTX_REQ_CMD);
	writel((addr >> 8) & 0xFF, base + AUXTX_REQ_ADDR_M);
	writel(addr & 0xFF, base + AUXTX_REQ_ADDR_L);
	writel((size > 0) ? (size - 1) : 0, base + AUXTX_REQ_LEN);

	if (!(msg->request & DP_AUX_I2C_READ))
		for (i = 0; i < size; i++)
			writel(buffer[i], base + AUXTX_REQ_DATA);

	writebit(base, AUXTX_TRAN_CTRL, TX_START, TX_START);
}

void dptx_init(struct rtk_dptx *dptx)
{
	void __iomem *base = dptx->edpreg;
	/* enable aux channel */
	writel(AUX_EN, dptx->edpreg + AUX_TX_CTRL);
	writel(AUX_ALL_IRQ, dptx->edpreg + AUX_IRQ_EN);

	writebit(base, ARBITER_CTRL, BIT0, BIT0);
	writel(DPTX_IRQ_EN, dptx->edpreg + DPTX_IRQ_CTRL);

	writebit(base, AUX_DIG_PHY8, BIT1, BIT1);
}
