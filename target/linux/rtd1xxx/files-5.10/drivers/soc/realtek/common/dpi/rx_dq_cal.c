/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#include <asm/tlb.h>

#include "dpi.h"

struct dpi_rx_dq_cal_priv {
	struct device *dev;
	struct dpi_device *dpi;
	struct notifier_block nb;
	uint32_t delta_width;
};

#define DPI_DLL_TEST_CTRL0				0x168
#define DPI_DLL_TEST_CTRL1				0x16c
#define DPI_DLL_READ_CTRL_1				0x174
#define DPI_DLL_DBG_READ_1				0x184
#define DPI_DLL_DPI_CTRL_0				0x208
#define DPI_DLL_DPI_CTRL_1				0x20c
#define DPI_DLL_DPI_CTRL_2				0x210
#define DPI_DLL_CAL_VREF_CTR			0x368
#define DPI_DLL_CAL_MODE_CTRL			0x36c
#define DPI_RW_DQS_IN_DLY_0(x)			0x520 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_DQS_IN_DLY_1(x)			0x528 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_DQS_IN_DLY_2(x)			0x538 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_DQS_IN_DLY_3(x)			0x540 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_DQS_IN_DLY_1_DBI(x)		0x530 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_DQS_IN_DLY_3_DBI(x)		0x548 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_CAL_OUT_0(x)				0x570 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_CAL_OUT_1(x)				0x578 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_READ_DBG_CTRL(x)			0x5e0 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_DQ_DEL_PSEL(x)			0x620 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_DQ_DEL_NSEL(x)			0x628 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_DM_DEL_PNSEL(x)			0x640 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_READ_CTRL_4(x)			0x6c8 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_VALID_WIN_DET_PFIFO(x)	0x6d0 + (x & 1) * 4 + (x >> 1) * 0x300
#define DPI_RW_VALID_WIN_DET_NFIFO(x)	0x6d8 + (x & 1) * 4 + (x >> 1) * 0x300

static void dpi_rx_dq_cal_clk_check(struct dpi_rx_dq_cal_priv *priv)
{
	uint32_t dpi_dbg_dll;
	struct dpi_device *dpi = priv->dpi;

	/* rd_dbg_sel0 = rd_dbg_sel1 = 0x17 */
	dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(0), 0x1f1f0000, (0x17 << 16) | (0x17 << 24));
	dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(1), 0x1f1f0000, (0x17 << 16) | (0x17 << 24));
	dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(2), 0x1f1f0000, (0x17 << 16) | (0x17 << 24));
	dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(3), 0x1f1f0000, (0x17 << 16) | (0x17 << 24));

	/* rw_dbg_sel0 = rw_dbg_sel0 = top_dbg_sel = 0x1, dpi_dbg_dll_sel = 0x0 */
	dpi_reg_update_bits(dpi, DPI_DLL_TEST_CTRL0, 0x0e300000, (0x1 << 20) | (0x1 << 25));
	dpi_reg_update_bits(dpi, DPI_DLL_TEST_CTRL1, 0x01300000, (0x1 << 20));

	dpi_reg_read(dpi, DPI_DLL_DBG_READ_1, &dpi_dbg_dll);
	pr_info("%s: dpi_dbg_dll = 0x%08x\n", __func__, dpi_dbg_dll);
	dpi_reg_write(dpi, DPI_DLL_DBG_READ_1, 0x0);
	pr_info("%s: dpi_dbg_dll = 0x%08x\n", __func__, dpi_dbg_dll);
}

static void dpi_rx_dq_cal_clk_enable(struct dpi_rx_dq_cal_priv *priv)
{
	struct dpi_device *dpi = priv->dpi;
	uint32_t src[1024], dst[1024];
	uint32_t size = 1024;

	// dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_0, 0xc, 0xc);					 // rst_fifo_mode: force reset FIFO pointer
	dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_0, 0xc, 0x0);					 // rst_fifo_mode: during no read operation
	dpi_reg_update_bits(dpi, DPI_DLL_READ_CTRL_1, 0x40000000, 0x0);			 // rx_cal_dis = 0
	dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_2, 0x0c000000, 0x04000000);	   // rst_3point_mode = force reset 3 point
	dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_0, 0x30, 0x0);					// enable rx 3 point K
	// dpi_reg_update_bits(dpi, DPI_DLL_CAL_VREF_CTRL, 0x3f00, 0x0);			 // Disable RX Vref auto calibration
	// dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_0, 0x3, 0x1);					 // fw_set_mode: during refresh
	dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_1, 0xc, 0xc);					 // fw_set_rd_dly
	mdelay(1);
	memcpy(dst, src, size);	 // ddr read
}

static void dpi_rx_dq_cal_clk_disable(struct dpi_rx_dq_cal_priv *priv)
{
	struct dpi_device *dpi = priv->dpi;

	// dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_0, 0xc, 0x1);					 // rst_fifo_mode: during refresh
	dpi_reg_update_bits(dpi, DPI_DLL_READ_CTRL_1, 0x40000000, 0x40000000);	 // rx_cal_dis = 1
	// dpi_reg_update_bits(dpi, DPI_DLL_CAL_VREF_CTRL, 0x3f00, 0x0);			// Disable RX Vref auto calibration
	dpi_reg_write(dpi, DPI_RW_READ_CTRL_4(0), 0x0);
	dpi_reg_write(dpi, DPI_RW_READ_CTRL_4(1), 0x0);
	dpi_reg_write(dpi, DPI_RW_READ_CTRL_4(2), 0x0);
	dpi_reg_write(dpi, DPI_RW_READ_CTRL_4(3), 0x0);
	// dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_0, 0x3, 0x1);			 // fw_set_mode: during refresh
	dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_1, 0xc, 0xc);			 // fw_set_rd_dly
	mdelay(1);
}

 static void dpi_rx_rd_dly_out(struct dpi_rx_dq_cal_priv *priv)
{
	struct dpi_device *dpi = priv->dpi;
	uint32_t slice, dpi_rw_cal_out_0, dpi_rw_cal_out_1;

	for(slice = 0; slice < 4; ++slice)
	{
		pr_info("%s: Slice %d:\n", __func__, slice);
		dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(slice), 0xf000, (0x0 << 12));
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_0(slice), &dpi_rw_cal_out_0);
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_1(slice), &dpi_rw_cal_out_1);
		pr_info("%s: TE	   = {dq1_p, dq1_n, dq0_p, dq0_n} = 0x%08x\n", __func__, dpi_rw_cal_out_0);
		pr_info("%s: Delta = {dq1_p, dq1_n, dq0_p, dq0_n} = 0x%08x\n", __func__, dpi_rw_cal_out_1);

		dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(slice), 0xf000, (0x1 << 12));
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_0(slice), &dpi_rw_cal_out_0);
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_1(slice), &dpi_rw_cal_out_1);
		pr_info("%s: TE	   = {dq3_p, dq3_n, dq2_p, dq2_n} = 0x%08x\n", __func__, dpi_rw_cal_out_0);
		pr_info("%s: Delta = {dq3_p, dq3_n, dq2_p, dq2_n} = 0x%08x\n", __func__, dpi_rw_cal_out_1);

		dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(slice), 0xf000, (0x2 << 12));
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_0(slice), &dpi_rw_cal_out_0);
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_1(slice), &dpi_rw_cal_out_1);
		pr_info("%s: TE	   = {dq5_p, dq5_n, dq4_p, dq4_n} = 0x%08x\n", __func__, dpi_rw_cal_out_0);
		pr_info("%s: Delta = {dq5_p, dq5_n, dq4_p, dq4_n} = 0x%08x\n", __func__, dpi_rw_cal_out_1);

		dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(slice), 0xf000, (0x3 << 12));
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_0(slice), &dpi_rw_cal_out_0);
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_1(slice), &dpi_rw_cal_out_1);
		pr_info("%s: TE	   = {dq7_p, dq7_n, dq6_p, dq6_n} = 0x%08x\n", __func__, dpi_rw_cal_out_0);
		pr_info("%s: Delta = {dq7_p, dq7_n, dq6_p, dq6_n} = 0x%08x\n", __func__, dpi_rw_cal_out_1);

		dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(slice), 0xf000, (0x4 << 12));
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_0(slice), &dpi_rw_cal_out_0);
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_1(slice), &dpi_rw_cal_out_1);
		pr_info("%s: TE	   = {dm0_p, dm0_n} = 0x%08x\n", __func__, dpi_rw_cal_out_0);
		pr_info("%s: Delta = {dm0_p, dm0_n} = 0x%08x\n", __func__, dpi_rw_cal_out_1);
	}
}

static void dpi_rx_dq_cal_run(struct dpi_rx_dq_cal_priv *priv)
{
	struct dpi_device *dpi = priv->dpi;
	uint32_t i, slice, dpi_rw_cal_out_0, rx_lead_det_pfifo, rx_lead_det_nfifo;
	uint32_t TE_P[9], TE_N[9];
	uint32_t src[1024], dst[1024];
	uint32_t size = 1024;
	uint32_t delta = 0;

	for(i = 0; i < 32; i+=4) delta |= (priv->delta_width << i);

	dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_0, 0x30, (0x3 << 4));	 // disable 3 point K
	dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_2, 0x0c000000, 0x0);	  // rst_3point_mode = disable reset 3 point
	// dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_0, 0x3, 0x1);			 // fw_set_mode: during refresh
	dpi_reg_update_bits(dpi, DPI_DLL_CAL_MODE_CTRL, 0x20, 0x20);		// fw_delta_force = 1
	dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_PFIFO(0), 0x0);
	dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_PFIFO(1), 0x0);
	dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_PFIFO(2), 0x0);
	dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_PFIFO(3), 0x0);
	dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_NFIFO(0), 0x0);
	dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_NFIFO(1), 0x0);
	dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_NFIFO(2), 0x0);
	dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_NFIFO(3), 0x0);

	// fw_rd_delta_upd = 1
	dpi_reg_update_bits(dpi, DPI_RW_READ_CTRL_4(0), 0x2, 0x2);
	dpi_reg_update_bits(dpi, DPI_RW_READ_CTRL_4(1), 0x2, 0x2);
	dpi_reg_update_bits(dpi, DPI_RW_READ_CTRL_4(2), 0x2, 0x2);
	dpi_reg_update_bits(dpi, DPI_RW_READ_CTRL_4(3), 0x2, 0x2);

	dpi_reg_write(dpi, DPI_RW_DQ_DEL_PSEL(0), delta);				// rw_blk_01 DQ rising  delta width
	dpi_reg_write(dpi, DPI_RW_DQ_DEL_PSEL(1), delta);				// rw_blk_01 DQ rising  delta width
	dpi_reg_write(dpi, DPI_RW_DQ_DEL_PSEL(2), delta);				// rw_blk_23 DQ rising  delta width
	dpi_reg_write(dpi, DPI_RW_DQ_DEL_PSEL(3), delta);				// rw_blk_23 DQ rising  delta width
	dpi_reg_write(dpi, DPI_RW_DQ_DEL_NSEL(0), delta);				// rw_blk_01 DQ falling  delta width
	dpi_reg_write(dpi, DPI_RW_DQ_DEL_NSEL(1), delta);				// rw_blk_01 DQ falling  delta width
	dpi_reg_write(dpi, DPI_RW_DQ_DEL_NSEL(2), delta);				// rw_blk_23 DQ falling  delta width
	dpi_reg_write(dpi, DPI_RW_DQ_DEL_NSEL(3), delta);				// rw_blk_23 DQ falling  delta width
	dpi_reg_write(dpi, DPI_RW_DM_DEL_PNSEL(0), delta & 0x00ff00ff);	// rw_blk_0 DM rising/falling delta width
	dpi_reg_write(dpi, DPI_RW_DM_DEL_PNSEL(1), delta & 0x00ff00ff);	// rw_blk_1 DM rising/falling delta width
	dpi_reg_write(dpi, DPI_RW_DM_DEL_PNSEL(2), delta & 0x00ff00ff);	// rw_blk_2 DM rising/falling delta width
	dpi_reg_write(dpi, DPI_RW_DM_DEL_PNSEL(3), delta & 0x00ff00ff);	// rw_blk_3 DM rising/falling delta width
	dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_1, 0xf, 0xf);			// fw_set_rd_dly/fw_set_wr_dly

	mdelay(1);
	memcpy(dst, src, size);	 // ddr read

	for(slice = 0; slice < 4; ++slice)
	{
		dpi_reg_read(dpi, DPI_RW_VALID_WIN_DET_PFIFO(slice), &rx_lead_det_pfifo);
		dpi_reg_read(dpi, DPI_RW_VALID_WIN_DET_NFIFO(slice), &rx_lead_det_nfifo);
		pr_info("%s: Slice %d:\n", __func__, slice);
		pr_info("%s: rx_lead_det_pfifo = 0x%08x\n", __func__, rx_lead_det_pfifo);
		pr_info("%s: rx_lead_det_nfifo = 0x%08x\n", __func__, rx_lead_det_nfifo);

		dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(slice), 0xf000, (0x0 << 12));
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_0(slice), &dpi_rw_cal_out_0);
		TE_P[0] = ((rx_lead_det_pfifo >> 0) & 0x1) ? ((dpi_rw_cal_out_0 >> 8) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 8) & 0xff) - 1;
		TE_P[1] = ((rx_lead_det_pfifo >> 1) & 0x1) ? ((dpi_rw_cal_out_0 >> 24) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 24) & 0xff) - 1;
		TE_N[0] = ((rx_lead_det_nfifo >> 0) & 0x1) ? ((dpi_rw_cal_out_0 >> 0) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 0) & 0xff) - 1;
		TE_N[1] = ((rx_lead_det_nfifo >> 1) & 0x1) ? ((dpi_rw_cal_out_0 >> 16) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 16) & 0xff) - 1;

		dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(slice), 0xf000, (0x1 << 12));
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_0(slice), &dpi_rw_cal_out_0);
		TE_P[2] = ((rx_lead_det_pfifo >> 2) & 0x1) ? ((dpi_rw_cal_out_0 >> 8) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 8) & 0xff) - 1;
		TE_P[3] = ((rx_lead_det_pfifo >> 3) & 0x1) ? ((dpi_rw_cal_out_0 >> 24) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 24) & 0xff) - 1;
		TE_N[2] = ((rx_lead_det_nfifo >> 2) & 0x1) ? ((dpi_rw_cal_out_0 >> 0) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 0) & 0xff) - 1;
		TE_N[3] = ((rx_lead_det_nfifo >> 3) & 0x1) ? ((dpi_rw_cal_out_0 >> 16) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 16) & 0xff) - 1;

		dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(slice), 0xf000, (0x2 << 12));
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_0(slice), &dpi_rw_cal_out_0);
		TE_P[4] = ((rx_lead_det_pfifo >> 4) & 0x1) ? ((dpi_rw_cal_out_0 >> 8) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 8) & 0xff) - 1;
		TE_P[5] = ((rx_lead_det_pfifo >> 5) & 0x1) ? ((dpi_rw_cal_out_0 >> 24) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 24) & 0xff) - 1;
		TE_N[4] = ((rx_lead_det_nfifo >> 4) & 0x1) ? ((dpi_rw_cal_out_0 >> 0) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 0) & 0xff) - 1;
		TE_N[5] = ((rx_lead_det_nfifo >> 5) & 0x1) ? ((dpi_rw_cal_out_0 >> 16) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 16) & 0xff) - 1;

		dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(slice), 0xf000, (0x3 << 12));
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_0(slice), &dpi_rw_cal_out_0);
		TE_P[6] = ((rx_lead_det_pfifo >> 6) & 0x1) ? ((dpi_rw_cal_out_0 >> 8) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 8) & 0xff) - 1;
		TE_P[7] = ((rx_lead_det_pfifo >> 7) & 0x1) ? ((dpi_rw_cal_out_0 >> 24) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 24) & 0xff) - 1;
		TE_N[6] = ((rx_lead_det_nfifo >> 6) & 0x1) ? ((dpi_rw_cal_out_0 >> 0) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 0) & 0xff) - 1;
		TE_N[7] = ((rx_lead_det_nfifo >> 7) & 0x1) ? ((dpi_rw_cal_out_0 >> 16) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 16) & 0xff) - 1;

		dpi_reg_update_bits(dpi, DPI_RW_READ_DBG_CTRL(slice), 0xf000, (0x4 << 12));
		dpi_reg_read(dpi, DPI_RW_CAL_OUT_0(slice), &dpi_rw_cal_out_0);
		TE_P[8] = ((rx_lead_det_pfifo >> 8) & 0x1) ? ((dpi_rw_cal_out_0 >> 8) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 8) & 0xff) - 1;
		TE_N[8] = ((rx_lead_det_nfifo >> 8) & 0x1) ? ((dpi_rw_cal_out_0 >> 0) & 0xff) + 1 : ((dpi_rw_cal_out_0 >> 0) & 0xff) - 1;

		dpi_reg_write(dpi, DPI_RW_DQS_IN_DLY_0(slice), (TE_P[3] << 24) | (TE_P[2] << 16) | (TE_P[1] << 8) | (TE_P[0] << 0));
		dpi_reg_write(dpi, DPI_RW_DQS_IN_DLY_1(slice), (TE_P[7] << 24) | (TE_P[6] << 16) | (TE_P[5] << 8) | (TE_P[4] << 0));
		dpi_reg_write(dpi, DPI_RW_DQS_IN_DLY_2(slice), (TE_N[3] << 24) | (TE_N[2] << 16) | (TE_N[1] << 8) | (TE_N[0] << 0));
		dpi_reg_write(dpi, DPI_RW_DQS_IN_DLY_3(slice), (TE_N[7] << 24) | (TE_N[6] << 16) | (TE_N[5] << 8) | (TE_N[4] << 0));
		dpi_reg_write(dpi, DPI_RW_DQS_IN_DLY_1_DBI(slice), TE_P[8]);
		dpi_reg_write(dpi, DPI_RW_DQS_IN_DLY_1_DBI(slice), TE_N[8]);
	}

	// dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_0, 0x3, 0x1);			 // rst_fifo_mode: during refresh

	/* fw_rd_te_upd = 1 */
	dpi_reg_update_bits(dpi, DPI_RW_READ_CTRL_4(0), 0x1, 0x1);
	dpi_reg_update_bits(dpi, DPI_RW_READ_CTRL_4(1), 0x1, 0x1);
	dpi_reg_update_bits(dpi, DPI_RW_READ_CTRL_4(2), 0x1, 0x1);
	dpi_reg_update_bits(dpi, DPI_RW_READ_CTRL_4(3), 0x1, 0x1);
	dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_1, 0xc, 0xc);			 // fw_set_rd_dly
	mdelay(1);

	// dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_0, 0xc, 0x0);			 // rst_fifo_mode: no read
	// dpi_reg_update_bits(dpi, DPI_DLL_CAL_MODE_CTRL, 0x20, 0x20);		// fw_delta_force = 1

	/* fw_rd_te_upd = 0 */
	// dpi_reg_update_bits(dpi, DPI_RW_READ_CTRL_4(0), 0x1, 0x0);
	// dpi_reg_update_bits(dpi, DPI_RW_READ_CTRL_4(1), 0x1, 0x0);
	// dpi_reg_update_bits(dpi, DPI_RW_READ_CTRL_4(2), 0x1, 0x0);
	// dpi_reg_update_bits(dpi, DPI_RW_READ_CTRL_4(3), 0x1, 0x0);
	// dpi_reg_update_bits(dpi, DPI_DLL_DPI_CTRL_1, 0xc, 0xc);			 // fw_set_rd_dly
	// mdelay(1);

	// dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_PFIFO(0), 0x0);
	// dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_PFIFO(1), 0x0);
	// dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_PFIFO(2), 0x0);
	// dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_PFIFO(3), 0x0);
	// dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_NFIFO(0), 0x0);
	// dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_NFIFO(1), 0x0);
	// dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_NFIFO(2), 0x0);
	// dpi_reg_write(dpi, DPI_RW_VALID_WIN_DET_NFIFO(3), 0x0);
}

static int dpi_rx_dq_cal_cb(struct notifier_block *nb, unsigned long event, void *data)
{
	struct dpi_rx_dq_cal_priv *priv = container_of(nb, struct dpi_rx_dq_cal_priv, nb);
	struct dpi_device *dpi = priv->dpi;
	struct dpi_event_base_temp_data *d = (struct dpi_event_base_temp_data *)data;
	int dt = d->new_temp - d->old_temp;

	// pr_info("%s: dt = %d, delta_temp = &d\n", __func__, dt, priv->delta_temp);

	// if (dt < priv->delta_temp && dt > -priv->delta_temp)
	if (event != DPI_EVENT_BASE_TEMP_DIFF_OVER_THERSHOLD)
		return NOTIFY_DONE;

	dpi_clock_gating_disable(dpi);
	// dpi_rx_dq_cal_clk_check(priv);
	dpi_rx_dq_cal_clk_enable(priv);
	// dpi_rx_dq_cal_clk_check(priv);
	// dpi_rx_rd_dly_out(priv);
	dpi_rx_dq_cal_run(priv);
	dpi_rx_rd_dly_out(priv);
	// dpi_rx_dq_cal_clk_check(priv);
	dpi_rx_dq_cal_clk_disable(priv);
	// dpi_rx_dq_cal_clk_check(priv);
	dpi_clock_gating_enable(dpi);

	return NOTIFY_OK;
}

static int of_dpi_rx_dq_cal_parse_data(struct device_node *np, struct dpi_rx_dq_cal_priv *priv)
{
	int len, ret;

	if (!of_find_property(np, "delta-width", &len))
		return -EINVAL;

	if (len > sizeof(priv->delta_width))
		return -EINVAL;

	if((ret = of_property_read_u32_array(np, "delta-width", &priv->delta_width, len/4)))
		return ret;

	return ret;
}

static int dpi_rx_dq_cal_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct dpi_device *dpi = dev_get_drvdata(dev->parent);
	struct dpi_rx_dq_cal_priv *priv;

	if (!dpi)
		return -EPROBE_DEFER;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = of_dpi_rx_dq_cal_parse_data(dev->of_node, priv);
	if (ret) {
		dev_err(dev, "failed to parse data from dt: %d\n", ret);
		return ret;
	}

	priv->dpi = dpi;
	priv->dev = dev;
	priv->nb.notifier_call = dpi_rx_dq_cal_cb;

	return dpi_register_notifier(dpi, &priv->nb);
}

static const struct of_device_id dpi_rx_dq_cal_of_match[] = {
	{ .compatible = "realtek,dpi-rx-dq-cal" },
	{}
};

static struct platform_driver dpi_rx_dq_cal_driver = {
	.driver = {
		.name		   = "rtk-dpi-rx-dq-cal",
		.owner		  = THIS_MODULE,
		.of_match_table = of_match_ptr(dpi_rx_dq_cal_of_match),
	},
	.probe	= dpi_rx_dq_cal_probe,
};
module_platform_driver(dpi_rx_dq_cal_driver);

MODULE_LICENSE("GPL v2");
