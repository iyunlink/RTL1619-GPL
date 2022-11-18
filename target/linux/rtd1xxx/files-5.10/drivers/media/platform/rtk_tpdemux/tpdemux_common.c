/*
 * Realtek TP Demux Driver
 *
 * Copyright (C) 2020 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/list.h>
#include <linux/pm_runtime.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include "tpdemux_common.h"
#include "tpdemux_core.h"
#include "tpdemux_buffer.h"

#define INFO_QUEUE_SIZE (376*16)
#if defined(CONFIG_RTK_TPDEMUX_QSIZE_256KB)
#define DDR_QUEUE_SIZE (256 * 1024)
#elif defined(CONFIG_RTK_TPDEMUX_QSIZE_512KB)
#define DDR_QUEUE_SIZE (512 * 1024)
#elif defined(CONFIG_RTK_TPDEMUX_QSIZE_128KB)
#define DDR_QUEUE_SIZE (128 * 1024)
#elif defined(CONFIG_RTK_TPDEMUX_QSIZE_64KB)
#define DDR_QUEUE_SIZE (64 * 1024)
#endif

#define TP_BUFFER_ALIGNMENT     (376)

#define MM_BUFFER_SIZE (188*4096)
#define ion_alloc ext_rtk_ion_alloc
#define ion_phys_addr_t u32

struct tp_pid_param {
	unsigned char v        : 1;
	unsigned char si_en    : 1;
	unsigned char ai_en    : 1;
	unsigned char ti_en    : 1;        /* Enable TP Info Pack or Not */
	unsigned char sec_en   : 1;      /* Enable Section Filter or Not */
	unsigned char cc_en    : 1;       /* Continuity Counter check enable */
	unsigned char reserved : 2;

	unsigned char sec_idx;
	unsigned char ddr_q;
	unsigned char info_q;
	unsigned char key;                /* Jupiter can supports */
	unsigned short pid;
	unsigned char ext_pid;
	unsigned char pb;

	unsigned char enc_key_odd;
	unsigned char enc;
	unsigned char dec;
	unsigned char clear_head;
	unsigned char ddr_q2_v;
	unsigned char ddr_q2;

	unsigned int data2_val;
	int idx;
};

struct tp_buf_param {
	ion_phys_addr_t           limit;
	ion_phys_addr_t           base;
	ion_phys_addr_t           rptr;
	ion_phys_addr_t           wptr;
};




static void _tp_hardware_env_setup(struct rtk_tp_reg *regs)
{
	if (TP_PID_FILTER_COUNT == MAX_PID_COUNT)
		write_reg32(regs->reg_TP_PID_PART, 0x7C);
	else
		write_reg32(regs->reg_TP_PID_PART,
			((((TP_PID_FILTER_COUNT / 8) * 2) & 0x1F) << 2));

	write_reg32(regs->reg_TP_PID_PART,
		read_reg32(regs->reg_TP_PID_PART) & ~PID_PART_CRC_ADDR_SEL_BIT);
	write_reg32(regs->reg_TP_CRC_INIT, 0xFFFFFFFF);
}

static void _tp_buffer_set_mode(struct filter_info *filter,
					enum TP_BUFFER_MODE mode)
{
	if (mode == INFO_PACK_BUFFER)
		filter->block_size = 8;
	else
		filter->block_size = 188;
}

static void _tp_buffer_set_param(struct rtk_tp_reg *regs,
					int idx, struct tp_buf_param param)
{
	mutex_lock(&regs->buf_ctrl_mutex);

	write_reg32(regs->reg_TP_RING_LIMIT, param.limit);
	write_reg32(regs->reg_TP_RING_BASE, param.base);
	write_reg32(regs->reg_TP_RING_RP, param.rptr);
	write_reg32(regs->reg_TP_RING_WP, param.wptr);

	write_reg32(regs->reg_TP_RING_CTRL, TP_RING_CTRL_WM(0x00) |
			TP_RING_CTRL_R_W(1)   |
			TP_RING_CTRL_IDX(idx));

	write_reg32(regs->reg_TP_THRESHOLD, TP_BUFFER_ALIGNMENT);

	mutex_unlock(&regs->buf_ctrl_mutex);
}

void _tp_buffer_get_param(struct rtk_tp_reg *regs,
				int idx, struct tp_buf_param *pParam)
{
	mutex_lock(&regs->buf_ctrl_mutex);

	write_reg32(regs->reg_TP_RING_CTRL, TP_RING_CTRL_WM(0x00) |
						TP_RING_CTRL_R_W(0)   |
						TP_RING_CTRL_IDX(idx));

	pParam->limit     = read_reg32(regs->reg_TP_RING_LIMIT);
	pParam->base      = read_reg32(regs->reg_TP_RING_BASE);
	pParam->rptr        = read_reg32(regs->reg_TP_RING_RP);
	pParam->wptr        = read_reg32(regs->reg_TP_RING_WP);

	mutex_unlock(&regs->buf_ctrl_mutex);
}

void _tp_buffer_get_status(struct rtk_tp_reg *regs,
				int idx, unsigned long *pBufferSize,
				unsigned long *pDataSize)
{
	struct tp_buf_param param;

	_tp_buffer_get_param(regs, idx, &param);
	*pBufferSize = param.limit-param.base;
	*pDataSize = (param.wptr >= param.rptr) ? (param.wptr - param.rptr) :
				(*pBufferSize - (param.rptr - param.wptr));
}



static void _tp_buffer_rst_ring_full(struct filter_info *filter)
{
	struct demux_info *dmx = filter->dmx;
	struct rtk_tp_reg *regs = &dmx->regs;

	switch (filter->ring_int_page_id) {
	case 0:
		write_reg32(regs->reg_TP_RING_FULL_INT_0,
			    TP_RING_INT_MASK(filter->ring_int_sub_id) |
			    TP_RING_FULL_INT_WRITE_DATA(0));
		break;
	case 1:
		write_reg32(regs->reg_TP_RING_FULL_INT_1,
			    TP_RING_INT_MASK(filter->ring_int_sub_id) |
			    TP_RING_FULL_INT_WRITE_DATA(0));
		break;
	case 2:
		write_reg32(regs->reg_TP_RING_FULL_INT_2,
			    TP_RING_INT_MASK(filter->ring_int_sub_id) |
			    TP_RING_FULL_INT_WRITE_DATA(0));
		break;
	case 3:
		write_reg32(regs->reg_TP_RING_FULL_INT_3,
			    TP_RING_INT_MASK(filter->ring_int_sub_id) |
			    TP_RING_FULL_INT_WRITE_DATA(0));
		break;
	default:
		break;
	}
}

static void _tp_buffer_rst_ring_avail(struct filter_info *filter)
{
	struct demux_info *dmx = filter->dmx;
	struct rtk_tp_reg *regs = &dmx->regs;

	switch (filter->ring_int_page_id) {
	case 0:
		write_reg32(regs->reg_TP_RING_AVAIL_INT_0,
			    TP_RING_INT_MASK(filter->ring_int_sub_id) |
			    TP_RING_FULL_INT_WRITE_DATA(0));
		break;
	case 1:
		write_reg32(regs->reg_TP_RING_AVAIL_INT_1,
			    TP_RING_INT_MASK(filter->ring_int_sub_id) |
			    TP_RING_FULL_INT_WRITE_DATA(0));
		break;
	case 2:
		write_reg32(regs->reg_TP_RING_AVAIL_INT_2,
			    TP_RING_INT_MASK(filter->ring_int_sub_id) |
			    TP_RING_FULL_INT_WRITE_DATA(0));
		break;
	case 3:
		write_reg32(regs->reg_TP_RING_AVAIL_INT_3,
			    TP_RING_INT_MASK(filter->ring_int_sub_id) |
			    TP_RING_FULL_INT_WRITE_DATA(0));
		break;
	default:
		break;
	}
}


static void _tp_buffer_flush(struct filter_info *filter)
{
	struct tp_buf_param param;
	struct rtk_tp_reg *regs = &filter->dmx->regs;

	_tp_buffer_get_param(regs, filter->ddr_q_id, &param);
	param.rptr = param.base;
	param.wptr = param.base;
	_tp_buffer_set_param(regs, filter->ddr_q_id, param);
	_tp_buffer_rst_ring_full(filter);
	_tp_buffer_rst_ring_avail(filter);

}

static int _tp_buffer_is_full(struct filter_info *filter)
{
	struct rtk_tp_reg *regs = &filter->dmx->regs;
	unsigned long FullInt;

	switch (filter->ring_int_page_id) {
	case 0:
		FullInt = read_reg32(regs->reg_TP_RING_FULL_INT_0);
		break;
	case 1:
		FullInt = read_reg32(regs->reg_TP_RING_FULL_INT_1);
		break;
	case 2:
		FullInt = read_reg32(regs->reg_TP_RING_FULL_INT_2);
		break;
	case 3:
		FullInt = read_reg32(regs->reg_TP_RING_FULL_INT_3);
		break;
	default:
		break;
	}

	return (FullInt & TP_RING_INT_MASK(filter->ring_int_sub_id)) ? 1 : 0;
}

static void _tp_buffer_update_rp(struct rtk_tp_reg *regs,
					int idx, struct tp_buf_param param)
{
	const unsigned long mask = TP_RING_CTRL_WM_LIMIT_BIT |
				   TP_RING_CTRL_WM_BASE_BIT  |
				   TP_RING_CTRL_WM_WP_BIT;

	mutex_lock(&regs->buf_ctrl_mutex);
	write_reg32(regs->reg_TP_RING_RP, param.rptr);
	write_reg32(regs->reg_TP_RING_CTRL, TP_RING_CTRL_WM(mask) |
						TP_RING_CTRL_R_W(1)   |
						TP_RING_CTRL_IDX(idx));

	mutex_unlock(&regs->buf_ctrl_mutex);

}

static void _tp_buffer_release_data(struct filter_info *filter,
					struct tp_buf_param param)
{
	struct rtk_tp_reg *regs = &filter->dmx->regs;

	_tp_buffer_update_rp(regs, filter->ddr_q_id, param);

	_tp_buffer_rst_ring_full(filter);
	_tp_buffer_rst_ring_avail(filter);
}

static void _tp_framer_set_cntl(struct rtk_tp_reg *regs,
					unsigned int cntl)
{
	cntl |= TP_TF_CNTL_WRITE_DATA(1);

	write_reg32(regs->reg_TF_CNTL,  cntl);
	write_reg32(regs->reg_TF_CNTL, ~cntl);
}

static void _tp_framer_get_cntl(struct rtk_tp_reg *regs,
					unsigned int *pCntl)
{
	*pCntl = read_reg32(regs->reg_TF_CNTL) & (~0x01);
}


static void _tp_framer_set_frm_cfg(struct rtk_tp_reg *regs,
					unsigned int cfg)
{
	write_reg32(regs->reg_TF_FRMCFG, cfg);
}

static void _tp_framer_get_frm_cfg(struct rtk_tp_reg *regs,
					unsigned int *pCfg)
{
	*pCfg = read_reg32(regs->reg_TF_FRMCFG);
}

static void _tp_framer_set_mode(struct rtk_tp_reg *regs,
					enum TP_FRAMER_MODE mode)
{
	unsigned int ctrl = 0;
	unsigned int frm_cfg = 0;

	_tp_framer_get_cntl(regs, &ctrl);
	_tp_framer_get_frm_cfg(regs, &frm_cfg);
	frm_cfg &= ~TP_TF_FRMCFG_LOCKNO(0xF);

	if (mode == TP_FRAMER_MEM) {
		ctrl   |= TP_TF_CNTL_MODE_BIT;
		frm_cfg |= TP_TF_FRMCFG_LOCKNO(1);
	} else {
		ctrl   &= ~TP_TF_CNTL_MODE_BIT;
		frm_cfg |= TP_TF_FRMCFG_LOCKNO(3);
	}

	_tp_framer_set_cntl(regs, ctrl);
	_tp_framer_set_frm_cfg(regs, frm_cfg);
}

static void _tp_framer_set_tsif(struct rtk_tp_reg *regs,
					struct ts_param hw_info)
{
	unsigned int val;

	_tp_framer_get_frm_cfg(regs, &val);

	val &= ~(TP_TF_FRMCFG_SERIAL(1)     |
		 TP_TF_FRMCFG_DATA_ORDER(1) |
		 TP_TF_FRMCFG_DATAPIN(1)    |
		 TP_TF_FRMCFG_ERR_POL(1)    |
		 TP_TF_FRMCFG_SYNC_POL(1)   |
		 TP_TF_FRMCFG_VAL_POL(1)    |
		 TP_TF_FRMCFG_CLK_POL(1));

	val |=  TP_TF_FRMCFG_SERIAL(hw_info.serial);
	val |=  TP_TF_FRMCFG_DATA_ORDER(hw_info.data_order);
	val |=  TP_TF_FRMCFG_DATAPIN(hw_info.datapin);
	val |=  TP_TF_FRMCFG_ERR_POL(hw_info.err_pol);
	val |=  TP_TF_FRMCFG_SYNC_POL(hw_info.sync_pol);
	val |=  TP_TF_FRMCFG_VAL_POL(hw_info.val_pol);
	val |=  TP_TF_FRMCFG_CLK_POL(hw_info.clk_pol);

	_tp_framer_set_frm_cfg(regs, val);

}

static void _tp_framer_set_int_enable(struct rtk_tp_reg *regs,
					unsigned int enable)
{
	enable &= ~TP_TF_INT_WRITE_DATA(1);
	write_reg32(regs->reg_TF_INT_EN, TP_TF_INT_MASK_HANK(enable)  |
						TP_TF_INT_WRITE_DATA(1));
	write_reg32(regs->reg_TF_INT_EN, TP_TF_INT_MASK_HANK(~enable) |
						TP_TF_INT_WRITE_DATA(0));
}

static void _tp_framer_clear_int_value(struct rtk_tp_reg *regs,
					unsigned int val)
{
	write_reg32(regs->reg_TF_INT, TP_TF_INT_VALUE_MASK_HANK(val) |
						TP_TF_INT_WRITE_DATA(0));
}



static void _tp_framer_reset(struct rtk_tp_reg *regs)
{
	unsigned int cnt_init = 0;
	unsigned int frm_cfg_init = 0;

	cnt_init =  TP_TF_CNTL_RAW_MODE_EN(0) |
		    TP_TF_CNTL_AF_FORCE_00_PKT_EN(0)  |
		    TP_TF_CNTL_AF_FORCE_ALL_PKT_EN(0) |
		    TP_TF_CNTL_PID_DEC_CHK_EN(0) |
		    TP_TF_CNTL_AF_TSC_EN(0) |
		    TP_TF_CNTL_PROTECT_MODE_EN(1) |
		    TP_TF_CNTL_DES_NX_MODE_EN(0)  |
		    TP_TF_CNTL_DES_RT_MODE_EN(0)  |
		    TP_TF_CNTL_TS_INPUT_SEL_EN(3) |
		    TP_TF_CNTL_SEC_BODY_INFO_EN(0)|
		    TP_TF_CNTL_STRM_ID_EN(0)  |
		    TP_TF_CNTL_BUF_RDY_CTL(0) |
		    TP_TF_CNTL_PSC_EN(0)      |
		    TP_TF_CNTL_PES_EN(0)      |
		    TP_TF_CNTL_TSC_EN(0)      |
		    TP_TF_CNTL_TB(0)          |
		    TP_TF_CNTL_BUSY(0)        |
		    TP_TF_CNTL_MODE(0)        |
		    TP_TF_CNTL_DU_EN(0)       |
		    TP_TF_CNTL_DE_EN(0)       |
		    TP_TF_CNTL_XT_EN(0)       |
		    TP_TF_CNTL_PID_EN(1)      |
		    TP_TF_CNTL_NULL_EN(1)     |
		    TP_TF_CNTL_TRERR_EN(0)    |
		    TP_TF_CNTL_SYNC_EN(1)     |
		    TP_TF_CNTL_RST_EN(0);

	frm_cfg_init =  TP_TF_FRMCFG_SYNC_BYTE(0x47)|
		    TP_TF_FRMCFG_DROPNO(1)      |
		    TP_TF_FRMCFG_LOCKNO(1)      |
		    TP_TF_FRMCFG_PACKET_SIZE(0) |
		    TP_TF_FRMCFG_DATA_ORDER(0)  |
		    TP_TF_FRMCFG_FRM_EN(0)      |
		    TP_TF_FRMCFG_FORCEDROP(0)   |
		    TP_TF_FRMCFG_SYNCMODE(0x1F) |
		    TP_TF_FRMCFG_SERIAL(1)      |
		    TP_TF_FRMCFG_DATAPIN(0)     |
		    TP_TF_FRMCFG_ERR_POL(0)     |
		    TP_TF_FRMCFG_SYNC_POL(0)    |
		    TP_TF_FRMCFG_VAL_POL(0)     |
		    TP_TF_FRMCFG_CLK_POL(0);

	_tp_framer_set_cntl(regs, cnt_init | TP_TF_CNTL_ERR_FIX_EN_BIT);
	_tp_framer_set_frm_cfg(regs, frm_cfg_init);
	_tp_framer_set_int_enable(regs, TP_TF_INT_CW_HANK);
	_tp_framer_clear_int_value(regs, TP_TF_INT_ALL_HANK);
}

static void _tp_framer_enable(struct rtk_tp_reg *regs,
				unsigned char en)
{
	if (en)
		write_reg32(regs->reg_TF_FRMCFG,
					read_reg32(regs->reg_TF_FRMCFG) |
					TP_TF_FRMCFG_FRM_EN_BIT);
	else
		write_reg32(regs->reg_TF_FRMCFG,
					read_reg32(regs->reg_TF_FRMCFG) &
					~TP_TF_FRMCFG_FRM_EN_BIT);
}

static void _tp_framer_reset_FSM(struct rtk_tp_reg *regs)
{
	write_reg32(regs->reg_TF_CNTL, TP_TF_CNTL_RST_EN_BIT |
						TP_TF_CNTL_WRITE_DATA(1));
	udelay(500);
	write_reg32(regs->reg_TF_CNTL, TP_TF_CNTL_RST_EN_BIT |
						TP_TF_CNTL_WRITE_DATA(0));
}

static void _tp_framer_rst_counter(struct rtk_tp_reg *regs)
{
	write_reg32(regs->reg_TF_CNT, 0);
	write_reg32(regs->reg_TF_DRP_CNT, 0);
	write_reg32(regs->reg_TF_ERR_CNT, 0);
}

static void _tp_pid_set_param(struct rtk_tp_reg *regs,
				struct tp_pid_param param)
{
	mutex_lock(&regs->pid_ctrl_mutex);
	write_reg32(regs->reg_TP_PID_DATA,
		TP_PID_DATA_SI_EN(param.si_en)    |
		TP_PID_DATA_PID_INI(1)            |
		TP_PID_DATA_AI_EN(param.ai_en)    |
		TP_PID_DATA_TI_EN(param.ti_en)    |
		TP_PID_DATA_SEC_IDX(param.sec_idx)|
		TP_PID_DATA_SEC_EN(param.sec_en)  |
		TP_PID_DATA_DDR_Q(param.ddr_q)    |
		TP_PID_DATA_CC_EN(param.cc_en)    |
		TP_PID_DATA_V(param.v)            |
		TP_PID_DATA_PID(param.pid)
	);

	write_reg32(regs->reg_TP_PID_DATA2, TP_PID_DATA2_INFO_Q(param.info_q) |
				TP_PID_DATA2_KEY(param.key)       |
				TP_PID_DATA2_KEY_INDEX(param.key) |
				TP_PID_DATA2_PB(param.pb)         |
				param.data2_val);

	write_reg32(regs->reg_TP_PID_DATA3,
				TP_PID_DATA3_ENC_KEY_ODD(param.enc_key_odd) |
				TP_PID_DATA3_ENC(param.enc) |
				TP_PID_DATA3_DEC(param.dec) |
				TP_PID_DATA3_CLEAR_HEAD(param.clear_head) |
				TP_PID_DATA3_DDR_Q2_V(param.ddr_q2_v) |
				TP_PID_DATA3_DDR_Q2(param.ddr_q2) |
				TP_PID_DATA3_EXT_PID(param.ext_pid));

	write_reg32(regs->reg_TP_PID_CTRL, TP_PID_CTRL_R_W(1) |
						TP_PID_CTRL_IDX(param.idx));
	mutex_unlock(&regs->pid_ctrl_mutex);
}


static void _tp_pid_get_param(struct rtk_tp_reg *regs,
				struct tp_pid_param *pParam)
{
	unsigned long data;

	mutex_lock(&regs->pid_ctrl_mutex);
	write_reg32(regs->reg_TP_PID_CTRL, TP_PID_CTRL_R_W(0) |
						TP_PID_CTRL_IDX(pParam->idx));

	data = read_reg32(regs->reg_TP_PID_DATA);

	pParam->v      = TP_PID_DATA_PARAM_V(data);
	pParam->si_en  = TP_PID_DATA_PARAM_SI_EN(data);
	pParam->ai_en  = TP_PID_DATA_PARAM_AI_EN(data);
	pParam->ti_en  = TP_PID_DATA_PARAM_TI_EN(data);
	pParam->sec_idx = TP_PID_DATA_PARAM_SEC_IDX(data);
	pParam->sec_en = TP_PID_DATA_PARAM_SEC_EN(data);
	pParam->cc_en  = TP_PID_DATA_PARAM_CC_EN(data);
	pParam->pid    = TP_PID_DATA_PARAM_PID(data);
	pParam->ddr_q  = TP_PID_DATA_PARAM_DDR_Q(data);

	data = read_reg32(regs->reg_TP_PID_DATA2);
	pParam->key    = (data >> 6) & 0x3F;
	pParam->info_q = (data & 0x3F);
	pParam->pb     = (data >> 13) & 0x01;

	data = read_reg32(regs->reg_TP_PID_DATA3);
	pParam->ext_pid = data & 0x3F;
	mutex_unlock(&regs->pid_ctrl_mutex);
}

static void _tp_pid_enable_PCR_tracking(struct rtk_tp_reg *regs,
					unsigned char en,
					int pid_table_id)
{
	unsigned long val = 0;

	mutex_lock(&regs->pid_ctrl_mutex);
	if (en) {
		val = TP_PCR_CTL_STC_EXTRA_PID_ADDR(pid_table_id);
		val |= TP_PCR_CTL_STC_EXTRA_FUNC_ENA;
	}

	if ((read_reg32(regs->reg_TP_PCRCtrlReg) & 0x1FF) != val) {
		write_reg32(regs->reg_TP_PCRCtrlReg,
			(read_reg32(regs->reg_TP_PCRCtrlReg) & ~0x1FF) | val);
		/* todo */
		/* ResetPCRTrackingStatus(); */
	}
	mutex_unlock(&regs->pid_ctrl_mutex);
}

static void _tp_pid_reset(struct rtk_tp_reg *regs,
				int pid_table_id)
{
	struct tp_pid_param param;

	/* todo */
	/* EnableSectionAssembler(0); */
	_tp_pid_enable_PCR_tracking(regs, 0, pid_table_id);
	memset(&param, 0, sizeof(param));
	param.idx = pid_table_id;
	param.data2_val = TP_PID_DATA2_DIS_DESCRAMBLE;	/* todo */
	_tp_pid_set_param(regs, param);
}

static void _tp_pid_init(struct rtk_tp_reg *regs, int idx)
{
	int i = 0;

	write_reg32(regs->reg_TP_PCRCtrlReg, 0x0);
	write_reg32(regs->reg_TP_EXT_PID_CNTL, 0);

	/* In PID table, TP_0 uses index 0 ~ 63, TP_1 uses index 64 ~ 127 */
	for (i = ((idx & 1) * TP_PID_FILTER_COUNT);
	     i < (((idx & 1) * TP_PID_FILTER_COUNT) + TP_PID_FILTER_COUNT); i++)
		_tp_pid_reset(regs, i);
}

static void _tp_read_data(struct filter_info *filter,
				struct tp_buf_param *param,
				unsigned long *length,
				unsigned long *offset)
{
	struct rtk_tp_reg *regs = &filter->dmx->regs;

	/* todo */
	/* UpdatePCRTrackingStatus */
	if (filter->is_buf_full) {
		unsigned long buf_size;
		unsigned long data_size;

		_tp_buffer_get_status(regs, filter->ddr_q_id,
						&buf_size, &data_size);

		if (data_size == 0) {
			_tp_framer_enable(regs, 1);
			_tp_framer_reset_FSM(regs);
			_tp_buffer_flush(filter);
			filter->is_buf_full = 0;
		}
	}

	_tp_buffer_get_param(regs, filter->ddr_q_id, param);
	*length = (param->rptr > param->wptr) ? (param->limit - param->rptr) :
						(param->wptr - param->rptr);
	*offset = param->rptr - param->base;

	if (_tp_buffer_is_full(filter)) {
		if (!filter->is_buf_full) {
			_tp_framer_enable(regs, 0);
			_tp_framer_reset_FSM(regs);
			filter->is_buf_full = 1;
		}
	}
}

void rtk_tp_stream_control(struct demux_info *dmx, enum TP_STREAMING_STATUS st)
{
	struct rtk_tp_reg *regs = &dmx->regs;

	mutex_lock(&regs->pid_ctrl_mutex);
	switch (st) {
	case TP_STREAMING_STOP:
		_tp_framer_enable(regs, 0);
		_tp_framer_reset_FSM(regs);
		break;

	case TP_STREAMING_START:
		_tp_framer_rst_counter(regs);
		_tp_framer_enable(regs, 0);
		_tp_framer_reset_FSM(regs);
		_tp_framer_enable(regs, 1);
		break;

	default:
		pr_err("%s, %d, %s: WARNING, illegal Streaming Control Command (%d)\n",
			__FILE__, __LINE__, __func__, st);
	}
	mutex_unlock(&regs->pid_ctrl_mutex);
}



void rtk_set_ts_input_select(struct demux_info *dmx)
{
	struct rtk_tp_reg *regs = &dmx->regs;

	switch (dmx->input_sel) {
	case TS_IN_TS0_PAD:
		if (dmx->tp_id == DMX_TP_A_0 ||
			dmx->tp_id == DMX_TP_B_0 ||
			dmx->tp_id == DMX_TP_C_0) {
			write_reg32(regs->reg_TF_CNTL,
					TP_TF_CNTL_TS_INPUT_SEL_H_BIT |
					TP_TF_CNTL_WRITE_DATA(0));
			write_reg32(regs->reg_TF_CNTL,
					TP_TF_CNTL_TS_INPUT_SEL_L_BIT |
					TP_TF_CNTL_WRITE_DATA(0));
			if (dmx->hw_info.serial == 0)
				write_reg32(regs->reg_TF_FRMCFG,
					(read_reg32(regs->reg_TF_FRMCFG) &
					~TP_TF_FRMCFG_SERIAL_EN_BIT) |
					TP_TF_FRMCFG_SERIAL(0));
			else
				write_reg32(regs->reg_TF_FRMCFG,
					(read_reg32(regs->reg_TF_FRMCFG) &
					~TP_TF_FRMCFG_SERIAL_EN_BIT) |
					TP_TF_FRMCFG_SERIAL(1));
		} else {
			write_reg32(regs->reg_TF_CNTL,
					TP_TF_CNTL_TS_INPUT_SEL_H_BIT |
					TP_TF_CNTL_WRITE_DATA(1));
			write_reg32(regs->reg_TF_CNTL,
					TP_TF_CNTL_TS_INPUT_SEL_L_BIT |
					TP_TF_CNTL_WRITE_DATA(1));
			if (dmx->hw_info.serial == 0)
				write_reg32(regs->reg_TF_FRMCFG,
					(read_reg32(regs->reg_TF_FRMCFG) &
					~TP_TF_FRMCFG_SERIAL_EN_BIT) |
					TP_TF_FRMCFG_SERIAL(0));
			else
				write_reg32(regs->reg_TF_FRMCFG,
					(read_reg32(regs->reg_TF_FRMCFG) &
					~TP_TF_FRMCFG_SERIAL_EN_BIT) |
					TP_TF_FRMCFG_SERIAL(1));
		}
		break;
	case TS_IN_INTERNAL_DEMOD:
		write_reg32(regs->reg_TF_CNTL, TP_TF_CNTL_TS_INPUT_SEL_H_BIT |
					TP_TF_CNTL_WRITE_DATA(0));
		write_reg32(regs->reg_TF_CNTL, TP_TF_CNTL_TS_INPUT_SEL_L_BIT |
					TP_TF_CNTL_WRITE_DATA(1));
		write_reg32(regs->reg_TF_FRMCFG,
					(read_reg32(regs->reg_TF_FRMCFG) &
					~TP_TF_FRMCFG_SERIAL_EN_BIT) |
					TP_TF_FRMCFG_SERIAL(0));
		break;
	case TS_IN_TSIO:
		write_reg32(regs->reg_TF_CNTL, TP_TF_CNTL_TS_INPUT_SEL_H_BIT |
					TP_TF_CNTL_WRITE_DATA(1));
		write_reg32(regs->reg_TF_CNTL, TP_TF_CNTL_TS_INPUT_SEL_L_BIT |
					TP_TF_CNTL_WRITE_DATA(0));
		write_reg32(regs->reg_TF_FRMCFG,
					(read_reg32(regs->reg_TF_FRMCFG) &
					~TP_TF_FRMCFG_SERIAL_EN_BIT) |
					TP_TF_FRMCFG_SERIAL(0));
		break;
	case TS_IN_TS1_PAD:
		if (dmx->tp_id == DMX_TP_A_0 ||
			dmx->tp_id == DMX_TP_B_0 ||
			dmx->tp_id == DMX_TP_C_0) {
			write_reg32(regs->reg_TF_CNTL,
					TP_TF_CNTL_TS_INPUT_SEL_H_BIT |
					TP_TF_CNTL_WRITE_DATA(1));
			write_reg32(regs->reg_TF_CNTL,
					TP_TF_CNTL_TS_INPUT_SEL_L_BIT |
					TP_TF_CNTL_WRITE_DATA(1));
			write_reg32(regs->reg_TF_FRMCFG,
					(read_reg32(regs->reg_TF_FRMCFG) &
					~TP_TF_FRMCFG_SERIAL_EN_BIT) |
					TP_TF_FRMCFG_SERIAL(1));
	} else {
			write_reg32(regs->reg_TF_CNTL,
					TP_TF_CNTL_TS_INPUT_SEL_H_BIT |
					TP_TF_CNTL_WRITE_DATA(0));
			write_reg32(regs->reg_TF_CNTL,
					TP_TF_CNTL_TS_INPUT_SEL_L_BIT |
					TP_TF_CNTL_WRITE_DATA(0));
			write_reg32(regs->reg_TF_FRMCFG,
					(read_reg32(regs->reg_TF_FRMCFG) &
					~TP_TF_FRMCFG_SERIAL_EN_BIT) |
					TP_TF_FRMCFG_SERIAL(1));
	}
		break;
	default:
		pr_err("%s, ERROR, input_sel %d is an invalid value\n",
						__func__, dmx->input_sel);
	}

	if (dmx->input_sel == TS_IN_TSIO) {
		write_reg32(regs->reg_TF_FRMCFG,
					(read_reg32(regs->reg_TF_FRMCFG) &
					~TP_TF_FRMCFG_LOCKNO(0xF)) |
					TP_TF_FRMCFG_LOCKNO(1));
	} else {
		if (((read_reg32(regs->reg_TF_CNTL) & (~0x01)) &
					TP_TF_CNTL_MODE_BIT) == 1) {
			/* MTP mode */
			write_reg32(regs->reg_TF_FRMCFG,
					(read_reg32(regs->reg_TF_FRMCFG) &
					~TP_TF_FRMCFG_LOCKNO(0xF)) |
					TP_TF_FRMCFG_LOCKNO(1));
		} else {
			/* normal mode */
			write_reg32(regs->reg_TF_FRMCFG,
					(read_reg32(regs->reg_TF_FRMCFG) &
					~TP_TF_FRMCFG_LOCKNO(0xF)) |
					TP_TF_FRMCFG_LOCKNO(3));
		}
	}
}

/**
 * rtk_tp_write()
 * @buf: source address in user space
 * @count: number of bytes to write
 *
 * Return: number of bytes written or <0 on error.
 */
static int rtk_tp_write(struct dmx_demux *demux, const char __user *buf,
			size_t count)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;
	struct stdemux *stdemux = (struct stdemux *)dvbdemux->priv;
	struct rtktpfei *fei = stdemux->rtktpfei;
	struct demux_info *dmx = fei->demux_data[stdemux->tsin_index];
	int status = 0;

	/* M2MTP */
	status = rtk_tp_write_mmbuffer(dmx, buf, count);

	return status;
}

void rtk_tp_init(struct demux_info *dmx)
{
	struct rtk_tp_reg *regs = &dmx->regs;

	_tp_hardware_env_setup(regs);
}

void rtk_demux_init(struct demux_info *dmx)
{
	struct rtk_tp_reg *regs = &dmx->regs;

	_tp_framer_reset(regs);
	_tp_pid_init(regs, dmx->tp_id);
}


struct filter_info *rtk_filter_init(struct demux_info *dmx, int i)
{
	struct rtktpfei *fei = dmx->fei;
	struct rtk_tp_reg *regs = &dmx->regs;
	struct filter_info *filter;
	struct tp_buf_param param;
	int j, buf_idx = -1;

	/* Find available index from 32 ring buffers belonging to this TP */
	for (j = 0; j < TP_FILTER_COUNT; j++) {
		if (dmx->ring_buf_tbl[j] == 0) {
			dmx->ring_buf_tbl[j] = 1;
			buf_idx = j;
			break;
		}
	}
	if (buf_idx == -1)
		return NULL;

	if (!dmx->filter_data[i]) {
		dmx->filter_data[i] = devm_kzalloc(fei->dev,
				sizeof(struct filter_info),
				GFP_KERNEL);
		memset(dmx->filter_data[i], 0, sizeof(struct filter_info));
	}

	filter = dmx->filter_data[i];

	/* In PID table, TP_0 uses index 0 ~ 63, TP_1 uses index 64 ~ 127 */
	filter->pid_table_id = ((dmx->tp_id & 1) * TP_PID_FILTER_COUNT) + i;

	/* TP_0 uses ring buffer index 0 ~ 31, TP_1 uses index 32 ~ 63 */
	dmx->pid_tbl_info[i].ddr_q_idx =
	    ((dmx->tp_id & 1) * TP_FILTER_COUNT) + buf_idx;
	filter->ddr_q_id = dmx->pid_tbl_info[i].ddr_q_idx;

	/* Set associated bit of ring buffer interrupt register sets, set [0]
	 * for ring buffer index 0 ~ 15, set [1] for index 16 ~ 31, etc.
	 */
	filter->ring_int_page_id = (filter->ddr_q_id >> 4) & 0x03;
	switch (filter->ring_int_page_id) {
	case 0:
		filter->ring_int_sub_id = (filter->ddr_q_id & 0x0F);
		break;
	case 1:
		filter->ring_int_sub_id = ((filter->ddr_q_id % 16) & 0x0F);
		break;
	case 2:
		filter->ring_int_sub_id = ((filter->ddr_q_id % 32) & 0x0F);
		break;
	case 3:
		filter->ring_int_sub_id = ((filter->ddr_q_id % 48) & 0x0F);
		break;
	default:
		break;
	}

	filter->dmx = dmx;

	_tp_buffer_set_mode(filter, DATA_BUFFER);

	filter->ddr_q_dmabuf = fei->fi[i].ddr_q_dmabuf;
	filter->ddr_q_phy_addr = fei->fi[i].ddr_q_phy_addr;
	filter->ddr_q_virt_addr = fei->fi[i].ddr_q_virt_addr;
	filter->ddr_q_len = fei->fi[i].ddr_q_len;

	param.limit = filter->ddr_q_phy_addr + filter->ddr_q_len;
	param.base  = filter->ddr_q_phy_addr;
	param.rptr    = filter->ddr_q_phy_addr;
	param.wptr    = filter->ddr_q_phy_addr;
	_tp_buffer_set_param(regs, filter->ddr_q_id, param);

	_tp_framer_set_mode(regs, TP_FRAMER_NORMAL);

	/* FIXME: set M2MTP mode */
	if (dmx->tp_id == DMX_TP_A_1)
		_tp_framer_set_mode(regs, TP_FRAMER_MEM);

	_tp_framer_set_tsif(regs, dmx->hw_info);

	_tp_buffer_flush(filter);
	filter->is_streaming = 1;
	filter->is_buf_full = 0;

	return filter;
}

void rtk_filter_uninit(struct filter_info *filter)
{
	struct demux_info *dmx = filter->dmx;
	struct rtk_tp_reg *regs = &dmx->regs;
	struct rtktpfei *fei = dmx->fei;
	int i = 0;

	for (i = 0; i < TP_PID_FILTER_COUNT; i++) {
		if ((((dmx->tp_id & 1) * TP_PID_FILTER_COUNT) + i) ==
			filter->pid_table_id &&
			dmx->pid_tbl_info[i].used == 1) {
			dmx->pid_tbl_info[i].used = 0;
			_tp_pid_reset(regs, filter->pid_table_id);
			break;
		}
	}

	/* TP_0: ddr_q_id 0 ~ 31 to ring_buf_tbl [0] ~ [31]
	 * TP_1: ddr_q_id 32 ~ 63 to ring_buf_tbl [0] ~ [31]
	 */
	i = filter->ddr_q_id - ((dmx->tp_id & 1) * TP_FILTER_COUNT);
	dmx->ring_buf_tbl[i] = 0;

	filter->is_streaming = 0;
	filter->is_buf_full = 0;
	_tp_buffer_flush(filter);

	/* TP_0: pid_table_id 0 ~ 63 to filter_data [0] ~ [63]
	 * TP_1: pid_table_id 64 ~ 127 to filter_data [0] ~ [63]
	 */
	i = filter->pid_table_id - ((dmx->tp_id & 1) * TP_PID_FILTER_COUNT);
	if (dmx->filter_data[i]) {
		devm_kfree(fei->dev, dmx->filter_data[i]);
		dmx->filter_data[i] = NULL;
	}
}


static int _register_dmx(struct stdemux *demux,
				struct dvb_adapter *adap,
				void *start_feed, void *stop_feed,
				struct rtktpfei *fei)
{
	int result;

	demux->dvb_demux.dmx.capabilities = DMX_TS_FILTERING |
					DMX_SECTION_FILTERING |
					DMX_MEMORY_BASED_FILTERING;

	demux->dvb_demux.priv = demux;
	demux->dvb_demux.filternum = RTKTPFE_MAXCHANNEL;
	demux->dvb_demux.feednum = RTKTPFE_MAXCHANNEL;

	demux->dvb_demux.start_feed = start_feed;
	demux->dvb_demux.stop_feed = stop_feed;
	demux->dvb_demux.write_to_decoder = NULL;
	result = dvb_dmx_init(&demux->dvb_demux);
	if (result < 0) {
		dev_err(fei->dev, "dvb_dmx_init failed (errno = %d)\n",
			result);
		goto err_dmx;
	}
	demux->dmxdev.filternum = demux->dvb_demux.filternum;
	demux->dmxdev.demux = &demux->dvb_demux.dmx;
	demux->dmxdev.capabilities = 0;
	result = dvb_dmxdev_init(&demux->dmxdev, adap);
	if (result < 0) {
		dev_err(fei->dev, "dvb_dmxdev_init failed (errno = %d)\n",
				result);

		goto err_dmxdev;
	}

	demux->hw_frontend.source = DMX_FRONTEND_0 + demux->tsin_index;

	result = demux->dvb_demux.dmx.add_frontend(&demux->dvb_demux.dmx,
						&demux->hw_frontend);
	if (result < 0) {
		dev_err(fei->dev, "add_frontend failed (errno = %d)\n",
					result);
		goto err_fe_hw;
	}

	demux->mem_frontend.source = DMX_MEMORY_FE;
	result = demux->dvb_demux.dmx.add_frontend(&demux->dvb_demux.dmx,
						&demux->mem_frontend);
	if (result < 0) {
		dev_err(fei->dev, "add_frontend failed (%d)\n", result);
		goto err_fe_mem;
	}

	result = demux->dvb_demux.dmx.connect_frontend(&demux->dvb_demux.dmx,
							&demux->hw_frontend);
	if (result < 0) {
		dev_err(fei->dev, "connect_frontend (%d)\n", result);
		goto err_fe_con;
	}
	return 0;

err_fe_con:
	demux->dvb_demux.dmx.remove_frontend(&demux->dvb_demux.dmx,
						     &demux->mem_frontend);
err_fe_mem:
	demux->dvb_demux.dmx.remove_frontend(&demux->dvb_demux.dmx,
						     &demux->hw_frontend);
err_fe_hw:
	dvb_dmxdev_release(&demux->dmxdev);
err_dmxdev:
	dvb_dmx_release(&demux->dvb_demux);
err_dmx:
	return result;

}

static void _unregister_dmx(struct stdemux *demux)
{

	demux->dvb_demux.dmx.remove_frontend(&demux->dvb_demux.dmx,
						     &demux->mem_frontend);
	demux->dvb_demux.dmx.remove_frontend(&demux->dvb_demux.dmx,
						     &demux->hw_frontend);
	dvb_dmxdev_release(&demux->dmxdev);
	dvb_dmx_release(&demux->dvb_demux);
}

phys_addr_t rtk_tp_ion_pa(struct ion_buffer *buffer)
{
	struct sg_table *table;
	struct page *page;
	phys_addr_t paddr;

	mutex_lock(&buffer->lock);

	table = buffer->sg_table;
	page = sg_page(table->sgl);
	if (!page)
		paddr = 0;
	else
		paddr = PFN_PHYS(page_to_pfn(page));

	mutex_unlock(&buffer->lock);

	return paddr;
}

//void *rtk_tp_ion_va(struct dma_buf *dmabuf)
void *rtk_tp_ion_va(struct ion_buffer *buffer)
{
	//struct ion_buffer *buffer
	struct scatterlist *sg;
	int i, j;
	void *vaddr;
	pgprot_t pgprot;
	struct sg_table *table = buffer->sg_table;
	int npages = PAGE_ALIGN(buffer->size) / PAGE_SIZE;
	struct page **pages = vmalloc(sizeof(struct page *) * npages);
	struct page **tmp = pages;

	if (!pages)
		return ERR_PTR(-ENOMEM);

	if (buffer->flags & ION_FLAG_NONCACHED)
		pgprot = pgprot_noncached(PAGE_KERNEL);
	else if (buffer->flags & ION_FLAG_CACHED)
		pgprot = PAGE_KERNEL;
	else
		pgprot = pgprot_writecombine(PAGE_KERNEL);

	for_each_sg(table->sgl, sg, table->nents, i) {
		int npages_this_entry = PAGE_ALIGN(sg->length) / PAGE_SIZE;
		struct page *page = sg_page(sg);

		BUG_ON(i >= npages);
		for (j = 0; j < npages_this_entry; j++)
			*(tmp++) = page++;
	}
	vaddr = vmap(pages, npages, VM_MAP, pgprot);
	vfree(pages);

	if (!vaddr)
		return ERR_PTR(-ENOMEM);

	return vaddr;
}

static void rtktpfe_alloc_static_ddr_q(struct rtktpfei *fei)
{
	struct filter_info *filter;
	int i;

	for (i = 0; i < TP_PID_FILTER_COUNT; i++) {

		filter = &fei->fi[i];
		filter->ddr_q_dmabuf = ion_alloc(DDR_QUEUE_SIZE,
					RTK_ION_HEAP_MEDIA_MASK,
					ION_FLAG_NONCACHED | ION_FLAG_SCPUACC |
					ION_FLAG_ACPUACC);

		if (IS_ERR_OR_NULL(filter->ddr_q_dmabuf)) {
			pr_err("ion_alloc ddr queue fail\n");
			return;
		}

		filter->ddr_q_phy_addr = rtk_tp_ion_pa(filter->ddr_q_dmabuf->priv);
		if (filter->ddr_q_phy_addr == 0) {
			pr_err("[%s rtk_tp_ion_pa fail]\n", __func__);
			goto dma_put;
		}

#if 1
		filter->ddr_q_virt_addr = rtk_tp_ion_va(filter->ddr_q_dmabuf->priv);
#else
		filter->ddr_q_virt_addr = rtk_tp_ion_va(filter->ddr_q_dmabuf);
#endif
		if (IS_ERR_OR_NULL(filter->ddr_q_virt_addr)) {
			pr_err("[%s rtk_tp_ion_va fail]\n", __func__);
			goto dma_put;
		}

		filter->ddr_q_len = (DDR_QUEUE_SIZE/TP_BUFFER_ALIGNMENT) *
					TP_BUFFER_ALIGNMENT;

	}
	return;

dma_put:
	if (filter->ddr_q_dmabuf) {
		dma_buf_put(filter->ddr_q_dmabuf);
		filter->ddr_q_dmabuf = NULL;
	}
}

static void rtktpfe_free_static_ddr_q(struct rtktpfei *fei)
{
	struct filter_info *filter;
	int i;

	for (i = 0; i < TP_PID_FILTER_COUNT; i++) {

		filter = &fei->fi[i];
		filter->ddr_q_len = 0;

		if (!IS_ERR_OR_NULL(filter->ddr_q_dmabuf)) {
			if (filter->ddr_q_virt_addr != NULL) {
				 vunmap((void*)filter->ddr_q_virt_addr);
			}
			dma_buf_put(filter->ddr_q_dmabuf);
			filter->ddr_q_dmabuf = NULL;
		}

		filter->ddr_q_virt_addr = NULL;
		filter->ddr_q_phy_addr = 0;
	}
}

static void rtktpfe_alloc_static_TPA1_mmbuf(struct rtktpfei *fei)
{
	struct demux_info *dmx = fei->demux_data[DMX_TP_A_1];
	dmx->mmbuf_dmabuf = ion_alloc(MM_BUFFER_SIZE, RTK_ION_HEAP_MEDIA_MASK,
				   //ION_FLAG_NONCACHED | ION_FLAG_SCPUACC |
				   ION_FLAG_CACHED | ION_FLAG_SCPUACC |
				   ION_FLAG_ACPUACC);

	if (IS_ERR_OR_NULL(dmx->mmbuf_dmabuf)) {
		pr_err("ion_alloc static mm buffer fail\n");
		return;
	}

	dmx->mmbuf_phy_addr = rtk_tp_ion_pa(dmx->mmbuf_dmabuf->priv);
	if (dmx->mmbuf_phy_addr == 0) {
		pr_err("[%s rtk_tp_ion_pa fail]\n", __func__);
		dmx->mmbuf_len = 0;
		goto dma_put;
	}

#if 1
	dmx->mmbuf_virt_addr = rtk_tp_ion_va(dmx->mmbuf_dmabuf->priv);
#else
	dmx->mmbuf_virt_addr = rtk_tp_ion_va(dmx->mmbuf_dmabuf);
#endif
	if (IS_ERR_OR_NULL(dmx->mmbuf_virt_addr)) {
		pr_err("[%s rtk_tp_ion_va fail]\n", __func__);
		dmx->mmbuf_len = 0;
		goto dma_put;
	}

	dmx->mmbuf_len = (MM_BUFFER_SIZE/TP_BUFFER_ALIGNMENT) *
					TP_BUFFER_ALIGNMENT;

	return;
dma_put:
	if (dmx->mmbuf_dmabuf) {
		dma_buf_put(dmx->mmbuf_dmabuf);
		dmx->mmbuf_dmabuf = NULL;
	}
}

static void rtktpfe_free_static_TPA1_mmbuf(struct rtktpfei *fei)
{
	struct demux_info *dmx = fei->demux_data[DMX_TP_A_1];

	if (!IS_ERR_OR_NULL(dmx->mmbuf_dmabuf)) {
		if (dmx->mmbuf_virt_addr != NULL) {
			//dma_buf_vunmap(dmx->mmbuf_dmabuf,
			 vunmap((void *)dmx->mmbuf_virt_addr);
		}
		//dma_buf_end_cpu_access(dmx->mmbuf_dmabuf, 0);
		dma_buf_put(dmx->mmbuf_dmabuf);
		dmx->mmbuf_dmabuf = NULL;
	}

	dmx->mmbuf_virt_addr = NULL;
	dmx->mmbuf_phy_addr = 0;
	dmx->mmbuf_len = 0;
}

static struct rtktpfe *rtktpfe_create(struct rtktpfei *fei,
				void *start_feed,
				void *stop_feed)
{
	struct rtktpfe *rtktpfe;
	int result;
	short int ids[] = { -1 };
	int i, j;

	rtktpfe = kzalloc(sizeof(struct rtktpfe), GFP_KERNEL);
	if (!rtktpfe)
		goto err1;

	mutex_init(&rtktpfe->lock);

	rtktpfe->device = fei->dev;

	result = dvb_register_adapter(&rtktpfe->adapter, "rtk tpfe",
					THIS_MODULE, fei->dev, ids);
	if (result < 0) {
		dev_err(fei->dev, "dvb_register_adapter failed (errno = %d)\n",
			result);
		goto err2;
	}

	rtktpfe->adapter.priv = fei;

	for (i = 0; i < fei->num_dmx; i++) {

		rtktpfe->demux[i].tsin_index = i;
		rtktpfe->demux[i].rtktpfei = fei;

		result = _register_dmx(&rtktpfe->demux[i], &rtktpfe->adapter,
				start_feed, stop_feed, fei);
		if (result < 0) {
			dev_err(fei->dev,
				"register_dvb feed=%d failed (errno = %d)\n",
				result, i);

			/* we take a all or nothing approach */
			for (j = 0; j < i; j++)
				_unregister_dmx(&rtktpfe->demux[j]);
			goto err3;
		}

		/* FIXME: set write callback for M2MTP */
		if (rtktpfe->demux[i].tsin_index == DMX_TP_A_1)
			rtktpfe->demux[i].dvb_demux.dmx.write = rtk_tp_write;
	}

	rtktpfe->num_feeds = fei->num_dmx;

	return rtktpfe;
err3:
	dvb_unregister_adapter(&rtktpfe->adapter);
err2:
	kfree(rtktpfe);
err1:
	return NULL;
};

static void rtktpfe_delete(struct rtktpfe *rtktpfe)
{
	int i;

	if (!rtktpfe)
		return;

	for (i = 0; i < rtktpfe->num_feeds; i++)
		_unregister_dmx(&rtktpfe->demux[i]);

	dvb_unregister_adapter(&rtktpfe->adapter);

	kfree(rtktpfe);
};

/**
 * _tp_framer_enable_pid() - Enable PID filter
 * @regs: register.
 * @onoff: 0 if PID filtering is disabled.
 *
 * Set PID filtering enable (pid_en) in register (e.g. TP_TF0_CNTL).
 *
 * Return: 0 always.
 */
int _tp_framer_enable_pid(struct rtk_tp_reg *regs, u8 onoff)
{
	write_reg32(regs->reg_TF_CNTL, TP_TF_CNTL_PID_EN_BIT |
		    TP_TF_CNTL_WRITE_DATA((onoff) ? 1 : 0));

	return 0;
}

/**
 * _rtk_set_ddrq() - Set ddr_q index
 */
void _rtk_set_ddrq(struct filter_info *filter)
{
	struct rtk_tp_reg *regs = &filter->dmx->regs;
	struct tp_pid_param param;

	memset(&param, 0, sizeof(param));
	param.idx = filter->pid_table_id;
	_tp_pid_get_param(regs, &param);

	param.ddr_q = filter->ddr_q_id;
	_tp_pid_set_param(regs, param);
}

void _rtk_request_pid_filter(struct filter_info *filter, u16 pid)
{
	struct rtk_tp_reg *regs = &filter->dmx->regs;
	struct tp_pid_param param;

	memset(&param, 0, sizeof(param));
	param.idx = filter->pid_table_id;
	_tp_pid_get_param(regs, &param);

	param.pid = pid;
	param.ddr_q = filter->ddr_q_id;
	param.v = 1;
	param.idx = filter->pid_table_id;
	param.data2_val = TP_PID_DATA2_DIS_DESCRAMBLE;
	_tp_pid_set_param(regs, param);
}

void rtk_tp_set_pid_filter(struct filter_info *filter, u16 pid)
{
	struct rtk_tp_reg *regs = &filter->dmx->regs;

	/* 8192 is a "dummy PID" which means the entire TS */
	if (pid == 8192) {
		_rtk_set_ddrq(filter);
		_tp_framer_enable_pid(regs, 0);
	} else {
		/* Remove unused PID */
		/* to do */

		/* Add new PID Filter */
		{
			/* skip pid that already exists... */

			_rtk_request_pid_filter(filter, pid);
			_tp_framer_enable_pid(regs, 1);
		}
	}

}

int rtk_is_tp_enable(struct demux_info *dmx)
{
	struct rtk_tp_reg *regs = &dmx->regs;

	return (read_reg32(regs->reg_TF_FRMCFG) &
				TP_TF_FRMCFG_FRM_EN_BIT);
}

void rtk_tp_deliver_data(struct dvb_demux_feed *feed)
{
	struct dvb_demux *demux = feed->demux;
	struct stdemux *stdemux = (struct stdemux *)demux->priv;
	struct rtktpfei *fei = stdemux->rtktpfei;
	struct demux_info *dmx = fei->demux_data[stdemux->tsin_index];
	struct filter_info *filter;
	unsigned long length = 0;
	unsigned long offset = 0;
	int i = 0;

	for (i = 0; i < TP_PID_FILTER_COUNT; i++) {
		mutex_lock(&stdemux->dmxdev.mutex);
		if (dmx->pid_tbl_info[i].used == 1) {
			struct tp_buf_param param = {0};

			filter = dmx->filter_data[i];
			if (filter == NULL || filter->ddr_q_dmabuf == NULL) {
				mutex_unlock(&stdemux->dmxdev.mutex);
				continue;
			}

			_tp_read_data(filter, &param, &length, &offset);

			if (feed) {
				struct dmx_ts_feed *core_feed = &feed->feed.ts;
				if (core_feed && core_feed->priv) {
					if (IS_ERR_OR_NULL(
					     filter->ddr_q_virt_addr + offset)) {
						dev_dbg(fei->dev,
						 "invalid q_virt_addr skip it\n");
					} else {
						u32 buf_flags = 0;
						feed->cb.ts(
						 filter->ddr_q_virt_addr + offset,
						 length, NULL, 0, &feed->feed.ts,
						 &buf_flags);
					}
				} else {
					pr_err("%s:feed->feed.ts NULL\n", __func__);
				}
			} else {
				pr_err("%s:feed NULL\n", __func__);
			}

			param.rptr = (param.rptr + length >= param.limit) ?
					param.base : (param.rptr + length);
			_tp_buffer_release_data(filter, param);
		}
		mutex_unlock(&stdemux->dmxdev.mutex);
	}
}

int rtk_tp_register(struct rtktpfe **rtktpfe,
						struct rtktpfei *fei,
						void *start_feed,
						void *stop_feed)
{
	rtktpfe_alloc_static_ddr_q(fei);
	rtktpfe_alloc_static_TPA1_mmbuf(fei);

	*rtktpfe = rtktpfe_create(fei, start_feed, stop_feed);
	if (!*rtktpfe) {
		rtktpfe_free_static_ddr_q(fei);
		rtktpfe_free_static_TPA1_mmbuf(fei);
		return -ENOMEM;
	}

	return 0;
}

int rtk_tp_unregister(struct rtktpfe *rtktpfe,
				struct rtktpfei *fei)
{
	rtktpfe_delete(rtktpfe);
	return 0;
}


