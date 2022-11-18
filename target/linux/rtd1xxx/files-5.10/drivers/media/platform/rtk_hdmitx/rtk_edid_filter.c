/*
 * rtk_edid_filter.c - RTK hdmitx driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <drm/drm_edid.h>
#include <drm/drm_crtc.h>
#include <linux/types.h>
#include <soc/realtek/rtk_chip.h>

#include "hdmitx.h"
#include "rtk_edid.h"


struct edid_product_info_t {
	unsigned char mfg_name[2];
	unsigned char prod_code[2];
	unsigned int serial;
	__u64 vic_mask;/* VIC 1~64, BIT0=VIC1 */
	__u64 vic2_mask;/* VIC 65~128, BIT0=VIC65 */
	unsigned char extended_vic_mask;
	__u64 vic2_420_mask;/* YCbCr420 format, VIC 65~128, BIT0=VIC65 */
};


static const struct edid_product_info_t quirks_info[] = {
	{.mfg_name[0] = 0x2E, .mfg_name[1] = 0x83,
	.prod_code[0] = 0x00, .prod_code[1] = 0x55,
	.serial = 1,
	.vic_mask = 0xFFFFFFFFFFFFFFFFULL,
	.vic2_mask = 0x0,
	.extended_vic_mask = 0x0,
	.vic2_420_mask = 0x0},
	/* DENON_AVR-2312CI */
	{.mfg_name[0] = 0x11, .mfg_name[1] = 0xee,
	.prod_code[0] = 0x28, .prod_code[1] = 0x00,
	.serial = 0x01010101,
	.vic_mask = 0xFFFFFFFFFFFFFFFFULL,
	.vic2_mask = 0x0,
	.extended_vic_mask = 0x0,
	.vic2_420_mask = 0x0},
	/* DENON_AVR-1610 */
	{.mfg_name[0] = 0x11, .mfg_name[1] = 0xee,
	.prod_code[0] = 0x14, .prod_code[1] = 0x00,
	.serial = 0x01010101,
	.vic_mask = 0xFFFFFFFFFFFFFFFF,
	.vic2_mask = 0x0,
	.extended_vic_mask = 0x0,
	.vic2_420_mask = 0x0},
};

int rtk_filter_quirks_vic(struct edid *edid, struct sink_capabilities_t *sink_cap)
{
	int i;
	int chip_id;

	chip_id = get_rtd_chip_id();
	if ((chip_id == CHIP_ID_RTD1392) || (chip_id == CHIP_ID_RTD1312C)) {
		/* Filter 4K VIC */
		sink_cap->vic2 = 0;
		sink_cap->extended_vic = 0;
		sink_cap->vic2_420 = 0;
	} else if ((chip_id == CHIP_ID_RTD1395) && (get_rtd_chip_revision() == RTD_CHIP_A02)) {
		sink_cap->vic2 = 0;
		sink_cap->extended_vic = 0;
		sink_cap->vic2_420 = 0;
	}

	for (i = 0; i < ARRAY_SIZE(quirks_info); i++) {
		if ((edid->mfg_id[0] == quirks_info[i].mfg_name[0]) &&
			(edid->mfg_id[1] == quirks_info[i].mfg_name[1]) &&
			(edid->prod_code[0] == quirks_info[i].prod_code[0]) &&
			(edid->prod_code[1] == quirks_info[i].prod_code[1]) &&
			(edid->serial == quirks_info[i].serial)) {

			sink_cap->vic = sink_cap->vic & quirks_info[i].vic_mask;
			sink_cap->vic2 = sink_cap->vic2 & quirks_info[i].vic2_mask;
			sink_cap->extended_vic = sink_cap->extended_vic & quirks_info[i].extended_vic_mask;
			sink_cap->vic2_420 = sink_cap->vic2_420 & quirks_info[i].vic2_420_mask;

			pr_info("Filter vic_mask=0x%llx", quirks_info[i].vic_mask);
			pr_info("Filter extended_vic_mask=%u", quirks_info[i].extended_vic_mask);
			pr_info("Filter vic2_mask=0x%llx", quirks_info[i].vic2_mask);
			pr_info("Filter vic2_420_mask=0x%llx", quirks_info[i].vic2_420_mask);

			return 1;
		}
	}
	return 0;
}

