// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * RTK IR Decoder setup for NEC protocol.
 *
 * Copyright (C) 2020 Simon Hsu <simon_hsu@realtek.com>
 */

#include "rtk-ir.h"
#include <linux/bitrev.h>

static int rtk_ir_nec_scancode(struct rtk_ir_scancode_req *request, unsigned int raw)
{
	unsigned int addr, addr_inv, data, data_inv;

	addr     = (raw >>  0) & 0xff;
	addr_inv = (raw >>  8) & 0xff;
	data     = (raw >> 16) & 0xff;
	data_inv = (raw >> 24) & 0xff;

	if ((data_inv ^ data) != 0xff) {
		request->scancode = bitrev8(addr)     << 24 |
				bitrev8(addr_inv) << 16 |
				bitrev8(data)     <<  8 |
				bitrev8(data_inv);
		request->protocol = RC_PROTO_NEC32;
	} else if ((addr_inv ^ addr) != 0xff) {
		request->scancode = addr     << 16 |
				addr_inv <<  8 |
				data;
		request->protocol = RC_PROTO_NECX;
	} else {
		request->scancode = addr << 8 |
				data;
		request->protocol = RC_PROTO_NEC;
	}

	return 1;
}

static int rtk_ir_nec_wakeinfo(struct rtk_ir_wakeinfo *info, unsigned int scancode)
{
	unsigned int mask;
	enum rc_proto protocol;
	unsigned int addr, addr_inv, data, data_inv;
	unsigned int addr_m, addr_inv_m, data_m, data_inv_m;

	if (scancode & 0xff000000) {
		protocol = RC_PROTO_NEC32;
		mask = 0xffffffff;
	} else if (scancode & 0x00ff0000) {
		protocol = RC_PROTO_NECX;
		mask = 0x00ffffff;
	} else {
		protocol = RC_PROTO_NEC;
		mask = 0x0000ffff;
	}

	data = scancode & 0xff;
	data_m = mask & 0xff;

	if (protocol == RC_PROTO_NEC32) {
		addr       = bitrev8(scancode >> 24);
		addr_m     = bitrev8(mask >> 24);
		addr_inv   = bitrev8(scancode >> 16);
		addr_inv_m = bitrev8(mask >> 16);
		data       = bitrev8(scancode >>  8);
		data_m     = bitrev8(mask >>  8);
		data_inv   = bitrev8(scancode >>  0);
		data_inv_m = bitrev8(mask >>  0);
	} else if (protocol == RC_PROTO_NECX) {
		addr       = (scancode >> 16) & 0xff;
		addr_m     = (mask >> 16) & 0xff;
		addr_inv   = (scancode >>  8) & 0xff;
		addr_inv_m = (mask >>  8) & 0xff;
		data_inv   = data ^ 0xff;
		data_inv_m = data_m;
	} else {
		addr       = (scancode >>  8) & 0xff;
		addr_m     = (mask >>  8) & 0xff;
		addr_inv   = addr ^ 0xff;
		addr_inv_m = addr_m;
		data_inv   = data ^ 0xff;
		data_inv_m = data_m;
	}

	info->addr = addr_inv << 8 | addr;
	info->addr_msk = addr_inv_m <<  8 | addr_m;
	info->scancode = data_inv << 8 | data;
	info->scancode_msk = data_inv_m << 24 | data_m << 16;
	return 0;
}

struct rtk_ir_hw_decoder rtk_ir_nec = {
	.type = RC_PROTO_BIT_NEC | RC_PROTO_BIT_NECX | RC_PROTO_BIT_NEC32,
	.tolerance = 70,
	.unit = 562,	/* 562.5 us */
	.sr = 40,	/* sample rate = 40 us */
	.timings = {
		.ldr = {	/* leader symbol */
			.pulse = 16,	/* 9ms */
			.space = 8,	/* 4.5ms */
		},
		.s00 = {	/* 0 symbol */
			.pulse = 1,	/* 562.5 us */
			.space = 1,	/* 562.5 us */
		},
		.s01 = {	/* 1 symbol */
			.pulse = 1,	/* 562.5 us */
			.space = 3,	/* 1687.5 us */
		},
		.ft_min = 10000,	/* 10000 us */
	},
	.rtimings = {
		.ldr = {	/* leader symbol */
			.space = 4,	/* 2.25 ms */
		},
	},
	.hw_set = IRBME | IRCM | IRUE | IRIE | REPEAT_EN | 0x1f,
	.scancode = rtk_ir_nec_scancode,
	.wakeinfo = rtk_ir_nec_wakeinfo,
};

