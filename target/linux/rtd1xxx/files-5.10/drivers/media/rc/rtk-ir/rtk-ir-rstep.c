// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * RTK IR Decoder setup for R-STEP protocol.
 *
 * Copyright (C) 2020 Simon Hsu <simon_hsu@realtek.com>
 */

#include "rtk-ir.h"

static int rtk_ir_rstep_scancode(struct rtk_ir_scancode_req *request, unsigned int raw)
{
	request->scancode = raw;
	request->protocol = RC_PROTO_OTHER;

	return 1;
}

static int rtk_ir_rstep_wakeinfo(struct rtk_ir_wakeinfo *info, unsigned int scancode)
{
	unsigned int addr_m, data_m;

	addr_m = 0xff00;
	data_m = 0x00ff;

	info->addr = (scancode & addr_m) >> 8;
	info->addr_msk = addr_m;
	info->scancode = scancode & data_m;
	info->scancode_msk = data_m;

	return 0;
}

struct rtk_ir_hw_decoder rtk_ir_rstep = {
	.type = RC_PROTO_BIT_OTHER,
	.tolerance = 80,
	.unit = 300,	/* 300 us */
	.sr = 40,	/* sample rate = 40 us */
	.timings = {
		.s00 = {	/* 0 symbol */
			.pulse = 1,	/* 562.5 us */
			.space = 1,	/* 562.5 us */
		},
		.s01 = {	/* 1 symbol */
			.pulse = 1,	/* 562.5 us */
			.space = 1,	/* 1687.5 us */
		},
		.ft_min = 50000,	/* 10000 us */
	},
	.hw_set = IRIE | IRRES | IRUE | IRDPM | 0x10,
	.scancode = rtk_ir_rstep_scancode,
	.wakeinfo = rtk_ir_rstep_wakeinfo,
};
