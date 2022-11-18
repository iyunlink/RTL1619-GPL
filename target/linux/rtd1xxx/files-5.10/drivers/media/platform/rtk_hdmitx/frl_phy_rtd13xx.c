/*
 * frl_phy_setting.c - HDMI2.1 FRL TMDS/Pixel Clock setting
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
#include "hdmitx_reg.h"
#include "crt_reg.h"

#define FRL_PLL_DELAY 20

enum FREQ_SHIFT_MODE {
	FREQ_SHIFT_OFF,
	FREQ_SHIFT_ON,
};

enum FRL_CLK_SELECT {
	FRL_3G_27MHZ,
	FRL_3G_74MHZ,
	FRL_3G_148MHZ,
	FRL_6G_297MHZ,
	FRL_594MHZ_420,
	FRL_6G_594MHZ
};

enum FRL_CLK_MODE {
	FRL_3G_MODE,
	FRL_6G_MODE,
};

enum FRL_LANE_MODE {
	FRL_3LANE,
	FRL_4LANE,
};

enum FRL_RATES {
	FRL_3G3LANES = 1,
	FRL_6G3LANES,
	FRL_6G4LANES,
};

enum FFE_LEVELS {
	FFE_LEVEL0 = 0,
	FFE_LEVEL1,
	FFE_LEVEL2,
	FFE_LEVEL3,
};

struct PLL_HDMI_SD_T {
	unsigned int sd1;
	unsigned int sd2;
	unsigned int sd4;
	unsigned int sd5;
};

struct PLL_HDMI_LDO_T {
	unsigned int ldo1;
	unsigned int ldo2;
	unsigned int ldo3;
	unsigned int ldo4;
	unsigned int ldo5;
	unsigned int pll_hdmi2;
	unsigned int ldo6;
	unsigned int ldo7;
	unsigned int ldo8;
	unsigned int ldo9;
};

/*
 * enum PLL_MODE - Index for struct PLL_HDMI_SD_T pll_param[]
 *    and struct PLL_HDMI_LDO_T ldo_param[]
 */
enum PLL_MODE {
	PLL_27MHz = 0,
	PLL_27x1p25,
	PLL_27x1p5,
	PLL_54MHz,
	PLL_54x1p25,
	PLL_54x1p5,
	PLL_74p25MHz,
	PLL_74p25x1p25,
	PLL_74p25x1p5,
	PLL_148p5MHz,
	PLL_148p5x1p25,
	PLL_148p5x1p5,
	PLL_297MHz,
	PLL_297x1p25,
	PLL_297x1p5,
	PLL_594MHz_420,
	PLL_594MHz_420x1p25,
	PLL_594MHz_420x1p5,
	PLL_594MHz,
};

static const struct PLL_HDMI_SD_T frl_sd[] = {
	/* 27MHz */
	{.sd1 = 0x9368000d,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 27MHz*1.25 */
	{.sd1 = 0x93680025,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 27MHz*1.5 */
	{.sd1 = 0x93680015,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 54MHz */
	{.sd1 = 0x9368000d,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 54MHz*1.25 */
	{.sd1 = 0x93680025,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 54MHz*1.5 */
	{.sd1 = 0x93680015,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 74.25MHz */
	{.sd1 = 0x93680013,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 74.25MHz*1.25 */
	{.sd1 = 0x93680034,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 74.25MHz*1.5 */
	{.sd1 = 0x9368001e,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 148.5MHz */
	{.sd1 = 0x93680013,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 148.5MHz*1.25 */
	{.sd1 = 0x93680034,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 148.5MHz*1.5 */
	{.sd1 = 0x9368001e,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 297MHz */
	{.sd1 = 0x93680013,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 297MHz*1.25  */
	{.sd1 = 0x93680034,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 297MHz*1.5 */
	{.sd1 = 0x9368001e,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 594MHz_420 */
	{.sd1 = 0x93680013,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 594MHz_420*1.25 */
	{.sd1 = 0x93680034,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 594MHz_420*1.5 */
	{.sd1 = 0x9368001E,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
	/* 594MHz */
	{.sd1 = 0x93680013,
	 .sd2 = 0xd0201ffe,
	 .sd4 = 0x00000000,
	 .sd5 = 0x00800000},
};

static const struct PLL_HDMI_LDO_T frl_ldo[] = {
	/* 27MHz */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x0005070F,
	 .pll_hdmi2 = 0x0020C880,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 27MHz*1.25 */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x00000002,
	 .pll_hdmi2 = 0x0021CA00,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 27MHz*1.5 */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x00000002,
	 .pll_hdmi2 = 0x0020CC40,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 54MHz */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x00000002,
	 .pll_hdmi2 = 0x002808C0,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 54MHz*1.25 */  
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x00000002,
	 .pll_hdmi2 = 0x00290A40,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 54MHz*1.5 */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x00000002,
	 .pll_hdmi2 = 0x00280C80,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 74.25MHz */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x00000002,
	 .pll_hdmi2 = 0x002808C0,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 74.25MHz*1.25 */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x00000000,
	 .pll_hdmi2 = 0x00290A60,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 74.25MHz*1.5 */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x00000002,
	 .pll_hdmi2 = 0x00210C40,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 148.5MHz */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x0000040B,
	 .pll_hdmi2 = 0x00200880,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 148.5MHz*1.25 */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x00050400,
	 .pll_hdmi2 = 0x00210A00,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 148.5MHz*1.5 */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x3774A8F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x00050526,
	 .pll_hdmi2 = 0x00110C00,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 297MHz */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377454F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x0005072C,
	 .pll_hdmi2 = 0x00100840,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 297MHz*1.25 */
	{.ldo1 = 0x22860988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x0005010C,
	 .pll_hdmi2 = 0x00113800,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 297MHz*1.5 */
	{.ldo1 = 0x22860988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x00050429,
	 .pll_hdmi2 = 0x00080400,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 594MHz_420 */
	{.ldo1 = 0x22060988,
	 .ldo2 = 0x377454F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x0005070C,
	 .pll_hdmi2 = 0x00100800,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 594MHz_420*1.25 */
	{.ldo1 = 0x22860988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x0005010E,
	 .pll_hdmi2 = 0x00111800,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 594MHz_420*1.5 */
	{.ldo1 = 0x22860988,
	 .ldo2 = 0x377400F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x00050526,
	 .pll_hdmi2 = 0x00102030,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
	/* 594MHz */
	{.ldo1 = 0x228E0988,
	 .ldo2 = 0x377454F0,
	 .ldo3 = 0x6739CE00,
	 .ldo4 = 0x00010842,
	 .ldo5 = 0x0005070F,
	 .pll_hdmi2 = 0x00000804,
	 .ldo6 = 0x14540001,
	 .ldo7 = 0x04222030,
	 .ldo8 = 0x0AAAA777,
	 .ldo9 = 0x0D030000},
};

void enable_pll_power(struct device *dev)
{
	hdmipll_write32(dev, SYS_PLL_HDMI,
		SYS_PLL_HDMI_PLLDISP_OEB(0) |
		SYS_PLL_HDMI_PLLDISP_VCORSTB(1) |
		SYS_PLL_HDMI_REG_PLL_MHL3_DIV_EN(1) |
		SYS_PLL_HDMI_REG_PLLDISP_RSTB(0) |
		SYS_PLL_HDMI_REG_PLLDISP_POW(1) |
		SYS_PLL_HDMI_REG_TMDS_POW(0) |
		SYS_PLL_HDMI_REG_PLL_RSTB(0) |
		SYS_PLL_HDMI_REG_PLL_POW(1) |
		SYS_PLL_HDMI_REG_HDMI_CK_EN(1));

	usleep_range(FRL_PLL_DELAY, FRL_PLL_DELAY+1);
}

void active_pll_control(struct device *dev)
{
	hdmipll_write32(dev, SYS_PLL_HDMI,
			SYS_PLL_HDMI_PLLDISP_OEB(0) |
			SYS_PLL_HDMI_PLLDISP_VCORSTB(1) |
			SYS_PLL_HDMI_REG_PLL_MHL3_DIV_EN(1) |
			SYS_PLL_HDMI_REG_PLLDISP_RSTB(1) |
			SYS_PLL_HDMI_REG_PLLDISP_POW(1) |
			SYS_PLL_HDMI_REG_TMDS_POW(0) |
			SYS_PLL_HDMI_REG_PLL_RSTB(0) |
			SYS_PLL_HDMI_REG_PLL_POW(1) |
			SYS_PLL_HDMI_REG_HDMI_CK_EN(1));

	usleep_range(FRL_PLL_DELAY, FRL_PLL_DELAY+1);

	hdmipll_write32(dev, SYS_PLL_HDMI,
			SYS_PLL_HDMI_PLLDISP_OEB(0) |
			SYS_PLL_HDMI_PLLDISP_VCORSTB(1) |
			SYS_PLL_HDMI_REG_PLL_MHL3_DIV_EN(1) |
			SYS_PLL_HDMI_REG_PLLDISP_RSTB(1) |
			SYS_PLL_HDMI_REG_PLLDISP_POW(1) |
			SYS_PLL_HDMI_REG_TMDS_POW(1) |
			SYS_PLL_HDMI_REG_PLL_RSTB(0) |
			SYS_PLL_HDMI_REG_PLL_POW(1) |
			SYS_PLL_HDMI_REG_HDMI_CK_EN(1));

	usleep_range(FRL_PLL_DELAY, FRL_PLL_DELAY+1);

	hdmipll_write32(dev, SYS_PLL_HDMI,
			SYS_PLL_HDMI_PLLDISP_OEB(0) |
			SYS_PLL_HDMI_PLLDISP_VCORSTB(1) |
			SYS_PLL_HDMI_REG_PLL_MHL3_DIV_EN(1) |
			SYS_PLL_HDMI_REG_PLLDISP_RSTB(1) |
			SYS_PLL_HDMI_REG_PLLDISP_POW(1) |
			SYS_PLL_HDMI_REG_TMDS_POW(1) |
			SYS_PLL_HDMI_REG_PLL_RSTB(1) |
			SYS_PLL_HDMI_REG_PLL_POW(1) |
			SYS_PLL_HDMI_REG_HDMI_CK_EN(1));

	usleep_range(FRL_PLL_DELAY, FRL_PLL_DELAY+1);
}

void reset_mac_frl_pll(struct device *dev)
{
	hdmipll_write32(dev, SYS_PLL_HDMITX2P1_SD5, 0x0);
}

/**
 * set_frl_pll - CRT & PHY config
 * @mode: enum PLL_MODE
 * @freq_shift: freq shift
 * @lane: enum FRL_LANE_MODE
 */
static void set_frl_pll(struct device *dev,
	unsigned char mode, unsigned char freq_shift, unsigned char lane)
{
	unsigned int sd1;
	unsigned int sd2;
	unsigned int reg_ldo2;

	sd1 = frl_sd[mode].sd1;
	sd2 = frl_sd[mode].sd2;
	if (freq_shift) {
		unsigned int fcode;
		unsigned int ncode;

		switch (mode) {
		case PLL_74p25MHz:
		case PLL_148p5MHz:
		case PLL_297MHz:
		case PLL_594MHz_420:
		case PLL_594MHz:
			fcode = 2003;
			ncode = 18;
			break;
		case PLL_74p25x1p25:
		case PLL_148p5x1p25:
		case PLL_297x1p25:
		case PLL_594MHz_420x1p25:
			fcode = 1936;
			ncode = 51;
			break;
		case PLL_74p25x1p5:
		case PLL_148p5x1p5:
		case PLL_297x1p5:
		case PLL_594MHz_420x1p5:
			fcode = 1980;
			ncode = 29;
			break;
		default:
			fcode = SYS_PLL_HDMI_SD1_get_fcode(sd1);
			ncode = SYS_PLL_HDMI_SD1_get_ncode(sd1);
			dev_err(dev, "%s Unknown mode=%u", __func__, mode);
			break;
		}

		sd1 = sd1 & ~(SYS_PLL_HDMI_SD1_fcode_mask | SYS_PLL_HDMI_SD1_ncode_mask);
		sd1 = sd1 | SYS_PLL_HDMI_SD1_fcode(fcode) | SYS_PLL_HDMI_SD1_ncode(ncode);

		sd2 = sd2 & ~SYS_PLL_HDMI_SD2_bypass_pi_mask;
		sd2 = sd2 | SYS_PLL_HDMI_SD2_bypass_pi(0);
	}

	hdmipll_write32(dev, SYS_PLL_HDMI_SD1, sd1);
	hdmipll_write32(dev, SYS_PLL_HDMI_SD2, sd2);
	hdmipll_write32(dev, SYS_PLL_HDMI_SD4, frl_sd[mode].sd4);
	hdmipll_write32(dev, SYS_PLL_HDMI_SD5, frl_sd[mode].sd5);

	/* Enable PLL OC */
	if (freq_shift)
		hdmipll_mask32(dev, SYS_PLL_HDMI_SD2,
			~(SYS_PLL_HDMI_SD2_bypass_pi_mask | SYS_PLL_HDMI_SD2_oc_en_mask),
			SYS_PLL_HDMI_SD2_bypass_pi(0) | SYS_PLL_HDMI_SD2_oc_en(1));
	else
		hdmipll_mask32(dev, SYS_PLL_HDMI_SD2,
			~SYS_PLL_HDMI_SD2_oc_en_mask, SYS_PLL_HDMI_SD2_oc_en(1));

	usleep_range(FRL_PLL_DELAY, FRL_PLL_DELAY+1);

	/* Disable CK output in 3 lane mode */
	reg_ldo2 = frl_ldo[mode].ldo2 & (~SYS_PLL_HDMI_LDO2_REG_TMDS_POWCK_mask);
	if (lane == FRL_4LANE)
		reg_ldo2 |= SYS_PLL_HDMI_LDO2_REG_TMDS_POWCK_mask;

	hdmipll_write32(dev, SYS_PLL_HDMI_LDO1, frl_ldo[mode].ldo1);
	hdmipll_write32(dev, SYS_PLL_HDMI_LDO2, reg_ldo2);
	hdmipll_write32(dev, SYS_PLL_HDMI_LDO3, frl_ldo[mode].ldo3);
	hdmipll_write32(dev, SYS_PLL_HDMI_LDO4, frl_ldo[mode].ldo4);
	hdmipll_write32(dev, SYS_PLL_HDMI_LDO5, frl_ldo[mode].ldo5);
	hdmipll_write32(dev, SYS_PLL_HDMI2, frl_ldo[mode].pll_hdmi2);

	usleep_range(FRL_PLL_DELAY, FRL_PLL_DELAY+1);

	hdmipll_write32(dev, SYS_PLL_HDMI_LDO6, frl_ldo[mode].ldo6);
	hdmipll_write32(dev, SYS_PLL_HDMI_LDO7, frl_ldo[mode].ldo7);
	hdmipll_write32(dev, SYS_PLL_HDMI_LDO8, frl_ldo[mode].ldo8);
	hdmipll_write32(dev, SYS_PLL_HDMI_LDO9, frl_ldo[mode].ldo9);

	usleep_range(FRL_PLL_DELAY, FRL_PLL_DELAY+1);
}

void set_frl_mode(struct device *dev, unsigned char mode)
{
	switch (mode) {
	case FRL_3G_MODE:
		/* after528 PLL */
		/* 0194_M1 =0 */
		hdmipll_mask32(dev, SYS_PLL_HDMI2,
			~SYS_PLL_HDMI2_REG_PLL_M1_mask, SYS_PLL_HDMI2_REG_PLL_M1(0));

		/* 0268[16]=1 */
		hdmipll_mask32(dev, SYS_PLL_HDMI_LDO9,
			~SYS_PLL_HDMI_LDO9_REG_P2S_CLK_SELECT_mask,
			SYS_PLL_HDMI_LDO9_REG_P2S_CLK_SELECT(1));

		/* 3Gbps mode  023C[25:20]=5*/
		hdmipll_mask32(dev, SYS_PLL_HDMI_LDO4,
			~SYS_PLL_HDMI_LDO4_REG_PLL_IP_mask,
			SYS_PLL_HDMI_LDO4_REG_PLL_IP(5));

		hdmipll_write32(dev, SYS_PLL_HDMITX2P1_SD1, 0x9368E46C);
		hdmipll_write32(dev, SYS_PLL_HDMITX2P1_SD2, 0x90203ffe);
		hdmipll_write32(dev, SYS_PLL_HDMITX2P1_SD5, 0x00800000);
		/* Enable PLL OC */
		hdmipll_mask32(dev, SYS_PLL_HDMITX2P1_SD2,
			~SYS_PLL_HDMITX2P1_SD2_oc_en_mask,
			SYS_PLL_HDMITX2P1_SD2_oc_en_mask);
		break;
	case FRL_6G_MODE:
		/* 98000230[8] high gain */
		hdmipll_mask32(dev, SYS_PLL_HDMI_LDO1,
			~SYS_PLL_HDMI_LDO1_REG_PLL_VCOGAIN_mask,
			SYS_PLL_HDMI_LDO1_REG_PLL_VCOGAIN(1));
		/* 98000194_M1 =0 */
		hdmipll_mask32(dev, SYS_PLL_HDMI2,
			~SYS_PLL_HDMI2_REG_PLL_M1_mask,
			SYS_PLL_HDMI2_get_REG_PLL_M1(0));

		/* 98000268[16]=1 */
		hdmipll_mask32(dev, SYS_PLL_HDMI_LDO9,
			~SYS_PLL_HDMI_LDO9_REG_P2S_CLK_SELECT_mask,
			SYS_PLL_HDMI_LDO9_REG_P2S_CLK_SELECT(1));

		/* 6Gbps mode  023C[25:20]=10*/
		hdmipll_mask32(dev, SYS_PLL_HDMI_LDO4,
			~SYS_PLL_HDMI_LDO4_REG_PLL_IP_mask,
			SYS_PLL_HDMI_LDO4_REG_PLL_IP(10));

		hdmipll_write32(dev, SYS_PLL_HDMITX2P1_SD1, 0x9369C7DB);
		hdmipll_write32(dev, SYS_PLL_HDMITX2P1_SD2, 0x90203ffe);
		hdmipll_write32(dev, SYS_PLL_HDMITX2P1_SD5, 0x00800000);
		/* Enable PLL OC */
		hdmipll_mask32(dev, SYS_PLL_HDMITX2P1_SD2,
			~SYS_PLL_HDMITX2P1_SD2_oc_en_mask,
			SYS_PLL_HDMITX2P1_SD2_oc_en_mask);
		break;
	default:
		printk("ERROR: Unknown frl mode %u\n", mode);
		break;
	}

	usleep_range(FRL_PLL_DELAY, FRL_PLL_DELAY+1);
}

/*
 * frl_set_phy - PHY setting when FRL mode is actived
 */
void frl_set_phy(struct device *dev,
	unsigned char link_rate, struct hdmi_format_setting *hdmi_format)
{
	unsigned char frl_mode;
	unsigned char lane;
	unsigned char vic;
	unsigned char freq_shift;
	unsigned char color_depth;
	unsigned char _3d_format;
	unsigned char pll_mode;

	vic = hdmi_format->vic;
	freq_shift = hdmi_format->freq_shift;

	if (hdmi_format->color != COLOR_YUV422)
		color_depth = hdmi_format->color_depth;
	else
		color_depth = 8;

	_3d_format = hdmi_format->_3d_format;

	switch (link_rate) {
	case FRL_3G3LANES:
		frl_mode = FRL_3G_MODE;
		lane = FRL_3LANE;
		break;
	case FRL_6G3LANES:
	case FRL_6G4LANES:
		frl_mode = FRL_6G_MODE;
		lane = FRL_4LANE;
		break;
	default:
		frl_mode = FRL_6G_MODE;
		lane = FRL_4LANE;
		dev_err(dev, "%s Wrong FRL_LinkRate=%u", __func__, link_rate);
	}

	reset_mac_frl_pll(dev);

	enable_pll_power(dev);

	switch (vic) {
	case VIC_1280X720P60:
	case VIC_1920X1080I60:
	case VIC_1280X720P50:
	case VIC_1920X1080I50:
	case VIC_1920X1080P24:
	case VIC_1920X1080P25:
	case VIC_1920X1080P30:
		/* FRL 74MHz PHY */
		if (color_depth == 10) {
			if (_3d_format == FORMAT_RTK_3D_FP)
				pll_mode = PLL_148p5x1p25;
			else
				pll_mode = PLL_74p25x1p25;
		} else if (color_depth == 12) {
			if (_3d_format == FORMAT_RTK_3D_FP)
				pll_mode = PLL_148p5x1p5;
			else
				pll_mode = PLL_74p25x1p5;
		} else {
			if (_3d_format == FORMAT_RTK_3D_FP)
				pll_mode = PLL_148p5MHz;
			else
				pll_mode = PLL_74p25MHz;
		}
		break;

	case VIC_720X480P60:
	case VIC_720X480I60:
	case VIC_720X576P50:
	case VIC_720X576I50:
		/* FRL 27MHz PHY */
		if (color_depth == 10) {
			if (_3d_format == FORMAT_RTK_3D_FP)
				pll_mode = PLL_54x1p25;
			else
				pll_mode = PLL_27x1p25;
		} else if (color_depth == 12) {
			if (_3d_format == FORMAT_RTK_3D_FP)
				pll_mode = PLL_54x1p5;
			else
				pll_mode = PLL_27x1p5;
		} else {
			if (_3d_format == FORMAT_RTK_3D_FP)
				pll_mode = PLL_54MHz;
			else
				pll_mode = PLL_27MHz;
		}
		break;

	case VIC_1920X1080P60:
	case VIC_1920X1080P50:
		/* FRL 148MHz PHY */
		if (color_depth == 10) {
			if (_3d_format == FORMAT_RTK_3D_FP)
				pll_mode = PLL_297x1p25;
			else
				pll_mode = PLL_148p5x1p25;
		} else if (color_depth == 12) {
			if (_3d_format == FORMAT_RTK_3D_FP)
				pll_mode = PLL_297x1p5;
			else
				pll_mode = PLL_148p5x1p5;
		} else {
			if (_3d_format == FORMAT_RTK_3D_FP)
				pll_mode = PLL_297MHz;
			else
				pll_mode = PLL_148p5MHz;
		}
		break;

	case VIC_3840X2160P24:
	case VIC_3840X2160P25:
	case VIC_3840X2160P30:
	case VIC_4096X2160P24:
	case VIC_4096X2160P25:
	case VIC_4096X2160P30:
		/* FRL 297MHz PHY */
		if (color_depth == 10)
			pll_mode = PLL_297x1p25;
		else if (color_depth == 12)
			pll_mode = PLL_297x1p5;
		else
			pll_mode = PLL_297MHz;
		break;

	case VIC_3840X2160P50:
	case VIC_3840X2160P60:
	case VIC_4096X2160P50:
	case VIC_4096X2160P60:
		if (hdmi_format->color != COLOR_YUV420) {
			/* FRL 594MHz 444 PHY */
			pll_mode = PLL_594MHz;
			lane = FRL_4LANE;
			break;
		}

		/* FRL 594MHz Y420 PHY */
		if (color_depth == 10)
			pll_mode = PLL_594MHz_420x1p25;
		else if (color_depth == 12)

			pll_mode = PLL_594MHz_420x1p5;
		else
			pll_mode = PLL_594MHz_420;

		break;

	default:
		break;
	}

	set_frl_pll(dev, pll_mode, freq_shift, lane);

	set_frl_mode(dev, frl_mode);

	active_pll_control(dev);
}
