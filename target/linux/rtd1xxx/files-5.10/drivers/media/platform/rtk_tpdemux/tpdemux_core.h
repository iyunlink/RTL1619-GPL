/*
 * tpdemux_core.h - Realtek TP demux driver
 *
 * Copyright (C) 2020 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _TPDEMUX_CORE_H_
#define _TPDEMUX_CORE_H_

#include "tpdemux_reg.h"

#define TP_MODULE_NUM_MAX    3

extern struct ion_device *rtk_phoenix_ion_device;

enum {
	MSB_D7          = 0,
	MSB_D0          = 1
};

enum {
	MSB_FIRST       = 0,
	LSB_FIRST       = 1
};

enum {
	ACTIVE_HIGH     = 0,
	ACTIVE_LOW      = 1
};

enum {
	RISING_EDGE,
	FALLING_EDGE
};

struct tp_module {
	struct clk            *clk;
	phys_addr_t           addr;
	void __iomem *reg_base;
	resource_size_t       size;
	int active;
};

struct rtktpfei {
	struct device         *dev;
	/* pm */
	atomic_t              open_cnt;

	/* map */
	struct tp_module       tpm[TP_MODULE_NUM_MAX];
	int num_tpm;


	struct rtktpfe *rtktpfe[RTKTPFEI_MAXADAPTER];
	struct demux_info *demux_data[DMX_TP_MAX];
	struct dvb_demux_feed *dvbdmxfeed;
	int num_dmx;
	atomic_t tp_init;
	struct mutex lock;

	struct timer_list timer;	/* timer interrupts for outputs */
	int global_feed_count;

	struct filter_info fi[TP_PID_FILTER_COUNT];

	struct workqueue_struct *wq;
};

#endif /* _TPDEMUX_CORE_H_ */
