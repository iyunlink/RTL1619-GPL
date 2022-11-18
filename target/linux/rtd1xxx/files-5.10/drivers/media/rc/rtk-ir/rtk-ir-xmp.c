// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * RTK IR Decoder setup for XMP protocol.
 *
 * Copyright (C) 2020 Simon Hsu <simon_hsu@realtek.com>
 */

#include "rtk-ir.h"
#include <linux/bitrev.h>

static int rtk_ir_xmp_scancode(struct rtk_ir_scancode_req *request, unsigned int raw)
{
	int i, temp = 0;
	int bits;
	int repeat = 0;

	for(i=0; i<8; i++) {
		bits = (raw & (0xF<<(4*i)))>>(4*i);
		if (i==4 && bits == 0xf)
			return 0;
		if (i==6) {
			if (!(bits & 0x8))
				repeat = 1;
			continue;
		}
		temp = temp + bits;
	}
	temp = (~temp+1) & 0xF;
	if(temp != (raw & 0xF<<24)>>24)
		return 0;

	request->scancode = (raw >> 8) & 0xff;
	request->protocol = RC_PROTO_XMP;
	if (request->scancode != request->last_scancode)
		repeat = 0;
	request->repeat = repeat;

	return 1;
}

static int rtk_ir_xmp_wakeinfo(struct rtk_ir_wakeinfo *info, unsigned int scancode)
{
	info->addr = 0;
	info->addr_msk = 0;
	info->scancode = scancode;
	info->scancode_msk = 0xff00;

	return 0;
}

struct rtk_ir_hw_decoder rtk_ir_xmp = {
	.type = RC_PROTO_BIT_XMP,
	.hw_set = COMCAST_EN | IRUE | IRRES | IRIE | 0x1f,
	.scancode = rtk_ir_xmp_scancode,
	.wakeinfo = rtk_ir_xmp_wakeinfo,
};
