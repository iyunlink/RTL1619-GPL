/*
 * tpdemux_common.h - Realtek TP demux driver
 *
 * Copyright (C) 2020 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _TPDEMUX_COMMON_H_
#define _TPDEMUX_COMMON_H_

#include <media/dmxdev.h>
#include <media/dvb_demux.h>
#include <media/dvb_frontend.h>

#include <linux/ion.h>
#include <uapi/linux/ion.h>
#include <soc/realtek/uapi/ion_rtk.h>

#include <ion_rtk_alloc.h>

#include "tpdemux_reg.h"

#define RTKTPFEI_MAXADAPTER 1
#define RTKTPFE_MAXCHANNEL 64

enum TS_IN_SEL {
	TS_IN_TS0_PAD = 0,
	TS_IN_INTERNAL_DEMOD,
	TS_IN_TSIO,
	TS_IN_TS1_PAD
};

enum TP_STREAMING_STATUS {
	TP_STREAMING_START = 0,
	TP_STREAMING_STOP,
	TP_STREAMING_FLUSH,
	TP_STREAMING_MUTE,
	TP_STREAMING_MAX,
};

enum TP_BUFFER_MODE {
	DATA_BUFFER = 0,
	INFO_PACK_BUFFER,
};

enum TP_FRAMER_MODE {
	TP_FRAMER_NORMAL = 0,
	TP_FRAMER_MEM,
};

/**
 * struct my_struct
 * @serial        : 0 : Parallel, 1: Serial
 * @data_order: 0 : bit 7 is MSB, 1 : bit 7 is LSB (Parallel)
 * @datapin     : 0 : MSB First,   1 : LSB First (Serial)
 * @err_pol      : 0 : 0 : Active High, 1 : Active Low
 * @sync_pol   : 0 : 0 : Active High, 1 : Active Low
 * @val_pol     : 0 : Active High, 1 : Active Low
 * @clk_pol     : 0 : Latch On Rising Edge, 1 : Latch On Falling Edge
 *
 */
struct ts_param {
	unsigned int	serial;
	unsigned int	data_order;
	unsigned int	datapin;
	unsigned int	err_pol;
	unsigned int	sync_pol;
	unsigned int	val_pol;
	unsigned int	clk_pol;
};

struct stdemux {
	struct dvb_demux	dvb_demux;
	struct dmxdev		dmxdev;
	struct dmx_frontend	hw_frontend;
	struct dmx_frontend	mem_frontend;
	int			tsin_index;
	int			running_feed_count;
	struct			rtktpfei *rtktpfei;
};


struct rtktpfe {
	struct stdemux demux[DMX_TP_MAX];
	struct mutex lock;
	struct dvb_adapter adapter;
	struct device *device;
	int mapping;
	int num_feeds;
};

struct filter_info {

	int pid_table_id;
	int ring_int_page_id;
	int ring_int_sub_id;

	int ddr_q_id;
	int info_q_id;

	unsigned char is_streaming;
	unsigned char is_buf_full;
	unsigned char block_size;

	struct dma_buf *ddr_q_dmabuf;
	phys_addr_t ddr_q_phy_addr;
	void *ddr_q_virt_addr;
	size_t ddr_q_len;

	struct demux_info *dmx;

};

struct pid_table_info {
	unsigned char used;
	int ddr_q_idx;
	int info_q_idx;
};


struct demux_info {
	unsigned int tp_id;
	int tp_mapping;
	int active;

	enum TS_IN_SEL input_sel;
	struct rtktpfei *fei;
	struct ts_param hw_info;
	struct rtk_tp_reg regs;

	struct filter_info *filter_data[TP_PID_FILTER_COUNT];
	struct pid_table_info pid_tbl_info[TP_PID_FILTER_COUNT];
	unsigned char ring_buf_tbl[TP_FILTER_COUNT];

	struct work_struct work;

	struct dma_buf *mmbuf_dmabuf;
	phys_addr_t mmbuf_phy_addr;
	void *mmbuf_virt_addr;
	size_t mmbuf_len;
};

void rtk_tp_init(struct demux_info *dmx);
int rtk_tp_register(struct rtktpfe **rtktpfe,
				struct rtktpfei *fei,
				void *start_feed,
				void *stop_feed);
int rtk_tp_unregister(struct rtktpfe *rtktpfe,
				struct rtktpfei *fei);
void rtk_demux_init(struct demux_info *dmx);
struct filter_info *rtk_filter_init(struct demux_info *dmx, int index);
void rtk_filter_uninit(struct filter_info *filter);
void rtk_set_ts_input_select(struct demux_info *dmx);
void rtk_tp_set_pid_filter(struct filter_info *ch, u16 pid);
int rtk_is_tp_enable(struct demux_info *dmx);
void rtk_tp_deliver_data(struct dvb_demux_feed *feed);
void rtk_tp_stream_control(struct demux_info *dmx, enum TP_STREAMING_STATUS st);

#endif /* _TPDEMUX_COMMON_H_ */
