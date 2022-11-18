/*
 * rtk_sb2_dbg.h - Realtek SB2 Debug API
 *
 * Copyright (C) 2019 Realtek Semiconductor Corporation
 * Copyright (C) 2019 Cheng-Yu Lee <cylee12@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SOC_REALTEK_SB2_DGB_H
#define __SOC_REALTEK_SB2_DGB_H

enum {
	SB2_DBG_SOURCE_SCPU,
	SB2_DBG_SOURCE_ACPU,
	SB2_DBG_SOURCE_PCPU,
	SB2_DBG_SOURCE_VCPU,
};

enum {
	SB2_DBG_ACCESS_DATA,
	SB2_DBG_ACCESS_INST,
	SB2_DBG_ACCESS_READ,
	SB2_DBG_ACCESS_WRITE,
};

enum {
	SB2_DBG_MONITOR_DATA  = 0x04,
	SB2_DBG_MONITOR_INST  = 0x08,
	SB2_DBG_MONITOR_READ  = 0x20,
	SB2_DBG_MONITOR_WRITE = 0x40,
};

struct sb2_dbg_event_data {
	u32 raw_ints;
	u32 source;
	u32 rw;
	u32 di;
	u32 addr;
};

enum {
	SB2_INV_UNKNOWN = 0,
	SB2_INV_SCPU = 1,
	SB2_INV_PCPU = 2,
	SB2_INV_ACPU = 3,
	SB2_INV_SCPU_SWC = 4,
	SB2_INV_PCPU_2 = 5,
	SB2_INV_VCPU = 6,
};

struct sb2_inv_event_data {
	u32 raw_ints;
	u32 addr;
	u32 inv_cpu;
	u32 timeout_th;
};

#endif
