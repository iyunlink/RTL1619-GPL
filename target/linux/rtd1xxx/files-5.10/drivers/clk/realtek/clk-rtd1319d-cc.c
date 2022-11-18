// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include "common.h"
#include "clk-pll.h"
#include "clk-regmap-gate.h"
#include "clk-regmap-mux.h"
#include "clk-det.h"
#include "reset.h"
#include <dt-bindings/clock/rtd1319d-clk.h>
#include <dt-bindings/reset/rtd1319d-reset.h>
#include "of-conf.h"

#define DIV_DV(_r, _d, _v)    { .rate = _r, .div = _d, .val = _v, }
#define FREQ_NF_MASK          (0x7FFFF)
#define FREQ_NF(_r, _nf)   { .rate = _r, .val = (_nf), }

static const struct freq_table scpu_tbl[] = {
	FREQ_NF(800000000, 0x0cd09),
	FREQ_NF(900000000, 0x0eaaa),
	FREQ_NF(1000000000, 0x1084b),
	FREQ_NF(1100000000, 0x125ed),
	FREQ_NF(1200000000, 0x1438e),
	FREQ_NF(1300000000, 0x1612f),
	FREQ_NF(1400000000, 0x17ed0),
	FREQ_NF(1500000000, 0x19c71),
	FREQ_NF(1600000000, 0x1ba12),
	FREQ_NF(1700000000, 0x1d7b4),
	FREQ_NF(1800000000, 0x1f555),
	FREQ_NF(1900000000, 0x212f6),
	FREQ_TABLE_END
};

static const struct freq_table bus_tbl[] = {
	FREQ_NF(499500000, 0x11000),
	FREQ_TABLE_END
};

static const struct freq_table dcsb_tbl[] = {
	FREQ_NF(472500000, 0x10000),
	FREQ_TABLE_END
};

static const struct freq_table gpu_tbl[] = {
	FREQ_NF(300000000, 0x099c7),
	FREQ_NF(400000000, 0x0d509),
	FREQ_NF(500000000, 0x1104b),
	FREQ_NF(600000000, 0x14b8e),
	FREQ_NF(634500000, 0x16000),
	FREQ_NF(661500000, 0x17000),
	FREQ_NF(769500000, 0x1b000),
	FREQ_TABLE_END
};

static const struct freq_table ve_tbl[] = {
	FREQ_NF(526500000, 0x12000),
	FREQ_NF(553500000, 0x13000),
	FREQ_NF(661500000, 0x17000),
	FREQ_NF(756000000, 0x1a800),
	FREQ_TABLE_END
};

static const struct freq_table hifi_tbl[] = {
	FREQ_NF(486000000, 0x10800),
	FREQ_NF(904500000, 0x20000),
	FREQ_TABLE_END
};

static const struct div_table scpu_div_tbl[] = {
	DIV_DV(1,  1, 0x00),
	DIV_DV(1,  2, 0x88),
	DIV_DV(1,  4, 0x90),
	DIV_DV(1,  8, 0xa0),
	DIV_DV(1, 10, 0xa8),
	DIV_TABLE_END
};

static const struct div_table bus_div_tbl[] = {
	DIV_DV(1, 1, 0),
	DIV_DV(1, 2, 2),
	DIV_DV(1, 4, 3),
	DIV_TABLE_END
};

static const struct div_table dcsb_div_tbl[] = {
	DIV_DV(1, 1, 0),
	DIV_DV(1, 2, 2),
	DIV_DV(1, 4, 3),
	DIV_TABLE_END
};

static struct clk_pll_div pll_scpu = {
	.div_ofs    = 0x108,
	.div_shift  = 8,
	.div_width  = 8,
	.div_tbl    = scpu_div_tbl,
	.clkp       = {
		.ssc_ofs   = 0x500,
		.pll_ofs   = CLK_OFS_INVALID,
		.pll_type  = CLK_PLL_TYPE_NF_SSC,
		.freq_tbl  = scpu_tbl,
		.freq_mask = FREQ_NF_MASK,
		.clkr.hw.init = CLK_HW_INIT("pll_scpu", "osc27m", &clk_pll_div_ops,
			CLK_IGNORE_UNUSED | CLK_GET_RATE_NOCACHE),
	},
};

static struct clk_pll_div pll_bus = {
	.div_ofs    = 0x030,
	.div_shift  = 0,
	.div_width  = 2,
	.div_tbl    = bus_div_tbl,
	.clkp       = {
		.ssc_ofs   = 0x520,
		.pll_ofs   = CLK_OFS_INVALID,
		.pll_type  = CLK_PLL_TYPE_NF_SSC,
		.freq_tbl  = bus_tbl,
		.freq_mask = FREQ_NF_MASK,
		.clkr.hw.init = CLK_HW_INIT("pll_bus", "osc27m", &clk_pll_div_ops,
			CLK_IGNORE_UNUSED | CLK_GET_RATE_NOCACHE),
		.clkr.shared = 1,
	},
};

static struct clk_pll_div pll_dcsb = {
	.div_ofs = 0x030,
	.div_shift  = 2,
	.div_width  = 2,
	.div_tbl    = dcsb_div_tbl,
	.clkp       = {
		.ssc_ofs   = 0x540,
		.pll_ofs   = CLK_OFS_INVALID,
		.pll_type  = CLK_PLL_TYPE_NF_SSC,
		.freq_tbl  = dcsb_tbl,
		.freq_mask = FREQ_NF_MASK,
		.clkr.hw.init = CLK_HW_INIT("pll_dcsb", "osc27m", &clk_pll_div_ops,
			CLK_IGNORE_UNUSED | CLK_GET_RATE_NOCACHE),
		.clkr.shared = 1,
	},
};

static CLK_FIXED_FACTOR(clk_sys, "clk_sys", "pll_bus", 1, 1, CLK_SET_RATE_PARENT);
static CLK_FIXED_FACTOR(clk_sysh, "clk_sysh", "pll_dcsb", 1, 1, CLK_SET_RATE_PARENT);

static const struct freq_table ddsx_tbl[] = {
	FREQ_NF(432000000, 0x0e800),
	FREQ_TABLE_END
};

static struct clk_pll pll_ddsa = {
	.ssc_ofs   = 0x560,
	.pll_ofs   = 0x120,
	.pow_loc   = CLK_PLL_CONF_POW_LOC_CTL3,
	.pll_type  = CLK_PLL_TYPE_NF_SSC,
	.freq_tbl  = ddsx_tbl,
	.freq_mask = FREQ_NF_MASK,
	.clkr.hw.init = CLK_HW_INIT("pll_ddsa", "osc27m", &clk_pll_ops,
		CLK_IGNORE_UNUSED | CLK_GET_RATE_NOCACHE),
};

static struct clk_pll pll_gpu = {
	.ssc_ofs   = 0x5a0,
	.pll_ofs   = 0x1C0,
	.pow_loc   = CLK_PLL_CONF_POW_LOC_CTL2,
	.pll_type  = CLK_PLL_TYPE_NF_SSC,
	.freq_tbl  = gpu_tbl,
	.freq_mask = FREQ_NF_MASK,
	.clkr.hw.init = CLK_HW_INIT("pll_gpu", "osc27m", &clk_pll_ops, CLK_GET_RATE_NOCACHE),
	CLK_PLL_CONF_RS(0x0003c000, 0x00014000),
	CLK_PLL_CONF_PI_BPS(),
};

static struct clk_pll pll_ve1 = {
	.pll_ofs   = 0x114,
	.ssc_ofs   = 0x580,
	.pow_loc   = CLK_PLL_CONF_POW_LOC_CTL2,
	.pll_type  = CLK_PLL_TYPE_NF_SSC,
	.freq_tbl  = ve_tbl,
	.freq_mask = FREQ_NF_MASK,
	.clkr.hw.init = CLK_HW_INIT("pll_ve1", "osc27m", &clk_pll_ops, CLK_GET_RATE_NOCACHE),
	CLK_PLL_CONF_RS(0x0003c000, 0x00014000),
	CLK_PLL_CONF_PI_BPS(),
};

static struct clk_pll pll_ve2 = {
	.ssc_ofs   = 0x5e0,
	.pll_ofs   = 0x1d0,
	.pow_loc   = CLK_PLL_CONF_POW_LOC_CTL2,
	.pll_type  = CLK_PLL_TYPE_NF_SSC,
	.freq_tbl  = ve_tbl,
	.freq_mask = FREQ_NF_MASK,
	.clkr.hw.init = CLK_HW_INIT("pll_ve2", "osc27m", &clk_pll_ops, CLK_GET_RATE_NOCACHE),
	CLK_PLL_CONF_RS(0x0003c000, 0x00014000),
	CLK_PLL_CONF_PI_BPS(),
};

static struct clk_pll_dif pll_dif = {
	.ssc_ofs   = 0x634,
	.pll_ofs   = 0x624,
	.clkr.hw.init = CLK_HW_INIT("pll_dif", "osc27m", &clk_pll_dif_v2_ops, CLK_GET_RATE_NOCACHE),
};

static struct clk_pll_psaud pll_psaud1a = {
	.reg = 0x130,
	.id  = CLK_PLL_PSAUD1A,
	.clkr.hw.init = CLK_HW_INIT("pll_psaud1a", "osc27m", &clk_pll_psaud_ops,
		CLK_IGNORE_UNUSED | CLK_SET_RATE_UNGATE),
};

static struct clk_pll_psaud pll_psaud2a = {
	.reg = 0x130,
	.id  = CLK_PLL_PSAUD2A,
	.clkr.hw.init = CLK_HW_INIT("pll_psaud2a", "osc27m", &clk_pll_psaud_ops,
		CLK_IGNORE_UNUSED | CLK_SET_RATE_UNGATE),
};

static struct clk_pll pll_hifi = {
	.ssc_ofs   = 0x6e0,
	.pll_ofs   = 0x1d8,
	.pll_type  = CLK_PLL_TYPE_NF_SSC,
	.pow_loc   = CLK_PLL_CONF_POW_LOC_CTL2,
	.freq_tbl  = hifi_tbl,
	.freq_mask = FREQ_NF_MASK,
	.clkr.hw.init = CLK_HW_INIT("pll_hifi", "osc27m", &clk_pll_ops, CLK_GET_RATE_NOCACHE),
	CLK_PLL_CONF_RS(0x0003c000, 0x00014000),
	CLK_PLL_CONF_PI_BPS(),
};

static const char * const clk_sc_parents[] = { "osc27m", "clk216m" };

static struct clk_regmap_mux clk_sc0 = {
	.mux_ofs = 0x038,
	.shift = 28,
	.mask = 0x1,
	.clkr.hw.init = CLK_HW_INIT_PARENTS("clk_sc0",  clk_sc_parents, &clk_regmap_mux_ops, 0),
};

static struct clk_regmap_mux clk_sc1 = {
	.mux_ofs = 0x038,
	.shift = 29,
	.mask = 0x1,
	.clkr.hw.init = CLK_HW_INIT_PARENTS("clk_sc1",  clk_sc_parents, &clk_regmap_mux_ops, 0),
};

static struct clk_det clk_det_pll_bus = {
	.hw.init = CLK_HW_INIT_NO_PARENT("ref_pll_bus", &clk_det_ops, CLK_GET_RATE_NOCACHE),
	.type = CLK_DET_TYPE_CRT,
	.ofs  = 0x424,
};

static struct clk_det clk_det_pll_dcsb = {
	.hw.init = CLK_HW_INIT_NO_PARENT("ref_pll_dcsb", &clk_det_ops, CLK_GET_RATE_NOCACHE),
	.type = CLK_DET_TYPE_CRT,
	.ofs  = 0x428,
};

static struct clk_det clk_det_pll_acpu = {
	.hw.init = CLK_HW_INIT_NO_PARENT("ref_pll_acpu", &clk_det_ops, CLK_GET_RATE_NOCACHE),
	.type = CLK_DET_TYPE_CRT,
	.ofs  = 0x42c,
};

static struct clk_det clk_det_pll_ddsa = {
	.hw.init = CLK_HW_INIT_NO_PARENT("ref_pll_ddsa", &clk_det_ops, CLK_GET_RATE_NOCACHE),
	.type = CLK_DET_TYPE_CRT,
	.ofs  = 0x430,
};

static struct clk_det clk_det_pll_gpu = {
	.hw.init = CLK_HW_INIT_NO_PARENT("ref_pll_gpu", &clk_det_ops, CLK_GET_RATE_NOCACHE),
	.type = CLK_DET_TYPE_CRT,
	.ofs  = 0x438,
};

static struct clk_det clk_det_pll_ve1 = {
	.hw.init = CLK_HW_INIT_NO_PARENT("ref_pll_ve1", &clk_det_ops, CLK_GET_RATE_NOCACHE),
	.type = CLK_DET_TYPE_CRT,
	.ofs  = 0x43c,
};

static struct clk_det clk_det_pll_ve2 = {
	.hw.init = CLK_HW_INIT_NO_PARENT("ref_pll_ve2", &clk_det_ops, CLK_GET_RATE_NOCACHE),
	.type = CLK_DET_TYPE_CRT,
	.ofs  = 0x440,
};

static struct clk_det clk_det_pll_hifi = {
	.hw.init = CLK_HW_INIT_NO_PARENT("ref_pll_hifi", &clk_det_ops, CLK_GET_RATE_NOCACHE),
	.type = CLK_DET_TYPE_CRT,
	.ofs  = 0x448,
};

static struct clk_fixed_factor pll_emmc_ref = {
	.div = 6,
	.mult = 1,
	.hw.init = CLK_HW_INIT("pll_emmc_ref", "osc27m", &clk_fixed_factor_ops, 0),
};

static struct clk_pll_mmc pll_emmc = {
	.pll_ofs = 0x1f0,
	.clkr.hw.init = CLK_HW_INIT("pll_emmc", "pll_emmc_ref", &clk_pll_mmc_ops, 0),
	.phase0_hw.init = CLK_HW_INIT("pll_emmc_vp0", "pll_emmc", &clk_pll_mmc_phase_ops, 0),
	.phase1_hw.init = CLK_HW_INIT("pll_emmc_vp1", "pll_emmc", &clk_pll_mmc_phase_ops, 0),
};

static struct clk_hw *cc_hws[] = {
	[RTD1319D_CRT_PLL_SCPU]    = &__clk_pll_div_hw(&pll_scpu),
	[RTD1319D_CRT_PLL_BUS]     = &__clk_pll_div_hw(&pll_bus),
	[RTD1319D_CRT_PLL_DCSB]    = &__clk_pll_div_hw(&pll_dcsb),
	[RTD1319D_CRT_CLK_SYS]     = &clk_sys.hw,
	[RTD1319D_CRT_CLK_SYSH]    = &clk_sysh.hw,
	[RTD1319D_CRT_PLL_DDSA]    = &__clk_pll_hw(&pll_ddsa),
	[RTD1319D_CRT_PLL_GPU]     = &__clk_pll_hw(&pll_gpu),
	[RTD1319D_CRT_PLL_VE1]     = &__clk_pll_hw(&pll_ve1),
	[RTD1319D_CRT_PLL_VE2]     = &__clk_pll_hw(&pll_ve2),
	[RTD1319D_CRT_PLL_DIF]     = &__clk_pll_dif_hw(&pll_dif),
	[RTD1319D_CRT_PLL_PSAUD1A] = &__clk_pll_psaud_hw(&pll_psaud1a),
	[RTD1319D_CRT_PLL_PSAUD2A] = &__clk_pll_psaud_hw(&pll_psaud2a),
	[RTD1319D_CRT_PLL_HIFI]    = &__clk_pll_hw(&pll_hifi),

	[RTD1319D_CRT_PLL_EMMC_REF] = &pll_emmc_ref.hw,
	[RTD1319D_CRT_PLL_EMMC]     = &__clk_pll_mmc_hw(&pll_emmc),
	[RTD1319D_CRT_PLL_EMMC_VP0] = &pll_emmc.phase0_hw,
	[RTD1319D_CRT_PLL_EMMC_VP1] = &pll_emmc.phase1_hw,
	[RTD1319D_CRT_CLK_SC0]      = &__clk_regmap_mux_hw(&clk_sc0),
	[RTD1319D_CRT_CLK_SC1]      = &__clk_regmap_mux_hw(&clk_sc1),

	[RTD1319D_CRT_CLK_DET_PLL_BUS]  = &__clk_det_hw(&clk_det_pll_bus),
	[RTD1319D_CRT_CLK_DET_PLL_DCSB] = &__clk_det_hw(&clk_det_pll_dcsb),
	[RTD1319D_CRT_CLK_DET_PLL_ACPU] = &__clk_det_hw(&clk_det_pll_acpu),
	[RTD1319D_CRT_CLK_DET_PLL_DDSA] = &__clk_det_hw(&clk_det_pll_ddsa),
	[RTD1319D_CRT_CLK_DET_PLL_GPU]  = &__clk_det_hw(&clk_det_pll_gpu),
	[RTD1319D_CRT_CLK_DET_PLL_VE1]  = &__clk_det_hw(&clk_det_pll_ve1),
	[RTD1319D_CRT_CLK_DET_PLL_VE2]  = &__clk_det_hw(&clk_det_pll_ve2),
	[RTD1319D_CRT_CLK_DET_PLL_HIFI] = &__clk_det_hw(&clk_det_pll_hifi),
};

static struct clk_hw *cc_n_hws[] = {
	[RTD1319D_CRT_PLL_BUS]     = &__clk_pll_div_hw(&pll_bus),
	[RTD1319D_CRT_PLL_DCSB]    = &__clk_pll_div_hw(&pll_dcsb),
	[RTD1319D_CRT_CLK_SYS]     = &clk_sys.hw,
	[RTD1319D_CRT_CLK_SYSH]    = &clk_sysh.hw,
	[RTD1319D_CRT_PLL_DDSA]    = &__clk_pll_hw(&pll_ddsa),
	[RTD1319D_CRT_PLL_GPU]     = &__clk_pll_hw(&pll_gpu),
	[RTD1319D_CRT_PLL_VE1]     = &__clk_pll_hw(&pll_ve1),
	[RTD1319D_CRT_PLL_VE2]     = &__clk_pll_hw(&pll_ve2),
	[RTD1319D_CRT_PLL_DIF]     = &__clk_pll_dif_hw(&pll_dif),
	[RTD1319D_CRT_PLL_PSAUD1A] = &__clk_pll_psaud_hw(&pll_psaud1a),
	[RTD1319D_CRT_PLL_PSAUD2A] = &__clk_pll_psaud_hw(&pll_psaud2a),
	[RTD1319D_CRT_PLL_HIFI]    = &__clk_pll_hw(&pll_hifi),

	[RTD1319D_CRT_PLL_EMMC_REF] = &pll_emmc_ref.hw,
	[RTD1319D_CRT_PLL_EMMC]     = &__clk_pll_mmc_hw(&pll_emmc),
	[RTD1319D_CRT_PLL_EMMC_VP0] = &pll_emmc.phase0_hw,
	[RTD1319D_CRT_PLL_EMMC_VP1] = &pll_emmc.phase1_hw,
	[RTD1319D_CRT_CLK_SC0]      = &__clk_regmap_mux_hw(&clk_sc0),
	[RTD1319D_CRT_CLK_SC1]      = &__clk_regmap_mux_hw(&clk_sc1),

	[RTD1319D_CRT_CLK_DET_PLL_BUS]  = &__clk_det_hw(&clk_det_pll_bus),
	[RTD1319D_CRT_CLK_DET_PLL_DCSB] = &__clk_det_hw(&clk_det_pll_dcsb),
	[RTD1319D_CRT_CLK_DET_PLL_ACPU] = &__clk_det_hw(&clk_det_pll_acpu),
	[RTD1319D_CRT_CLK_DET_PLL_DDSA] = &__clk_det_hw(&clk_det_pll_ddsa),
	[RTD1319D_CRT_CLK_DET_PLL_GPU]  = &__clk_det_hw(&clk_det_pll_gpu),
	[RTD1319D_CRT_CLK_DET_PLL_VE1]  = &__clk_det_hw(&clk_det_pll_ve1),
	[RTD1319D_CRT_CLK_DET_PLL_VE2]  = &__clk_det_hw(&clk_det_pll_ve2),
	[RTD1319D_CRT_CLK_DET_PLL_HIFI] = &__clk_det_hw(&clk_det_pll_hifi),
};

static const char * const clk_ve_parents[] = { "pll_vodma", "clk_sysh", "pll_ve1", "pll_ve2" };
static const char * const clk_hifi_parents[] = { "pll_hifi", "pll_hifi", "pll_gpu", "pll_dcsb" };
static const char * const clk_hifi_iso_parents[] = { "pll_hifi", "osc27m", "clk_sys", "clk_sys" };

static struct clk_composite_data cc_composites[] = {
	{
		.id            = RTD1319D_CRT_CLK_GPU,
		.mux_ofs       = CLK_OFS_INVALID,
		.gate_ofs      = 0x050,
		.gate_shift    = 18,
		.gate_write_en = 1,
		.parent_names  = (const char *[]){ "pll_gpu" },
		.num_parents   = 1,
		.name          = "clk_gpu",
		.flags         = CLK_SET_RATE_PARENT,
	},
	{
		.id            = RTD1319D_CRT_CLK_VE1,
		.gate_ofs      = 0x050,
		.gate_shift    = 20,
		.gate_write_en = 1,
		.mux_ofs       = 0x04c,
		.mux_width     = 3,
		.mux_shift     = 0,
		.parent_names  = clk_ve_parents,
		.num_parents   = ARRAY_SIZE(clk_ve_parents),
		.name          = "clk_ve1",
		.flags         = CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT,
	},
	{
		.id            = RTD1319D_CRT_CLK_VE2,
		.gate_ofs      = 0x050,
		.gate_shift    = 22,
		.gate_write_en = 1,
		.mux_ofs       = 0x04c,
		.mux_width     = 3,
		.mux_shift     = 3,
		.parent_names  = clk_ve_parents,
		.num_parents   = ARRAY_SIZE(clk_ve_parents),
		.name          = "clk_ve2",
		.flags         = CLK_SET_RATE_PARENT |
			 CLK_SET_RATE_NO_REPARENT | CLK_IGNORE_UNUSED,
	},
	{
		.id            = RTD1319D_CRT_CLK_VE3,
		.gate_ofs      = 0x05c,
		.gate_shift    = 26,
		.gate_write_en = 1,
		.mux_ofs       = 0x04c,
		.mux_width     = 3,
		.mux_shift     = 6,
		.parent_names  = clk_ve_parents,
		.num_parents   = ARRAY_SIZE(clk_ve_parents),
		.name          = "clk_ve3",
		.flags         = CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT,
	},
	{
		.id            = RTD1319D_CRT_CLK_VE3_BPU,
		.gate_ofs      = CLK_OFS_INVALID,
		.mux_ofs       = 0x04c,
		.mux_width     = 3,
		.mux_shift     = 9,
		.parent_names  = clk_ve_parents,
		.num_parents   = ARRAY_SIZE(clk_ve_parents),
		.name          = "clk_ve3_bpu",
		.flags         = CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT,
	},
	{
		.id            = RTD1319D_CRT_CLK_HIFI,
		.gate_ofs      = CLK_OFS_INVALID,
		.mux_ofs       = 0x028,
		.mux_width     = 2,
		.mux_shift     = 0,
		.parent_names  = clk_hifi_parents,
		.num_parents   = ARRAY_SIZE(clk_hifi_parents),
		.name          = "clk_hifi",
		.flags         = CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT,
	},
	{
		.id            = RTD1319D_CRT_CLK_HIFI_ISO,
		.gate_ofs      = CLK_OFS_INVALID,
		.mux_ofs       = 0x028,
		.mux_width     = 2,
		.mux_shift     = 8,
		.parent_names  = clk_hifi_iso_parents,
		.num_parents   = ARRAY_SIZE(clk_hifi_iso_parents),
		.name          = "clk_hifi_iso",
	},
};


#define GATE_COMMON(_id, _name, _parent, _flags, _ofs, _shift) \
	CLK_GATE_DATA(_id, _name, _parent, _flags, _ofs, _shift, 1, 0)

#define GATE_NORMAL(_id, _name, _parent, _ofs, _shift) \
	GATE_COMMON(_id, _name, _parent, 0, _ofs, _shift)

#define GATE_IGNORED(_id, _name, _parent, _ofs, _shift) \
	GATE_COMMON(_id, _name, _parent, CLK_IGNORE_UNUSED, _ofs, _shift)

#define GATE_CRITICAL(_id, _name, _parent, _ofs, _shift) \
	GATE_COMMON(_id, _name, _parent, CLK_IS_CRITICAL, _ofs, _shift)

#define GATE_SET_RATE_PARENT(_id, _name, _parent, _ofs, _shift) \
	GATE_COMMON(_id, _name, _parent, CLK_SET_RATE_PARENT,  _ofs, _shift)

static struct clk_gate_data cc_gates[] = {
	GATE_CRITICAL(RTD1319D_CRT_CLK_EN_MISC, "misc", NULL, 0x50, 0),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_PCIE0, "pcie0", NULL, 0x50, 2),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_GSPI, "gspi", "misc", 0x50, 6),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_SDS, "sds", NULL, 0x50, 12),
	GATE_IGNORED(RTD1319D_CRT_CLK_EN_HDMI, "hdmi", NULL, 0x50, 14),
	GATE_IGNORED(RTD1319D_CRT_CLK_EN_CP, "cp", NULL, 0x54, 2),
	GATE_CRITICAL(RTD1319D_CRT_CLK_EN_TP, "tp", NULL, 0x54, 6),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_NF, "nf", NULL, 0x54, 10),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_EMMC, "emmc", NULL, 0x54, 12),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_SD, "sd", NULL, 0x54, 14),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_SDIO_IP, "sdio_ip", NULL, 0x54, 16),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_MIPI, "mipi", NULL, 0x54, 18),
	GATE_SET_RATE_PARENT(RTD1319D_CRT_CLK_EN_EMMC_IP, "emmc_ip", "pll_emmc", 0x54, 20),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_SDIO, "sdio", NULL, 0x54, 22),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_SD_IP, "sd_ip", NULL, 0x54, 24),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_CABLERX, "cablerx", NULL, 0x54, 26),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_TPB, "tpb", NULL, 0x54, 28),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_MISC_SC1, "misc_sc1", "misc", 0x54, 30),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_MISC_I2C_3, "misc_i2c_3", "misc", 0x58, 0),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_JPEG, "jpeg", NULL, 0x58, 4),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_MISC_SC0, "misc_sc0", "misc", 0x58, 10),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_HDMIRX, "hdmirx", NULL, 0x58, 26),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_HSE, "hse", NULL, 0x58, 28),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_UR2, "ur2", "misc", 0x58, 30),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_UR1, "ur1", "misc", 0x5c, 0),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_FAN, "fan", "misc", 0x5c, 2),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_SATA_WRAP_SYS, "sata_wrap_sys", NULL, 0x5c, 8),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_SATA_WRAP_SYSH, "sata_wrap_sysh", NULL, 0x5c, 10),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_SATA_MAC_SYSH, "sata_mac_sysh", NULL, 0x5c, 12),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_R2RDSC, "r2rdsc", NULL, 0x5c, 14),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_TPC, "tpc", NULL, 0x5c, 16),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_PCIE1, "pcie1", NULL, 0x5c, 18),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_MISC_I2C_4, "misc_i2c_4", "misc", 0x5c, 20),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_MISC_I2C_5, "misc_i2c_5", "misc", 0x5c, 22),
	GATE_IGNORED(RTD1319D_CRT_CLK_EN_TSIO, "tsio", NULL, 0x5c, 24),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_EDP, "edp", NULL, 0x5c, 28),
	GATE_IGNORED(RTD1319D_CRT_CLK_EN_TSIO_TRX, "tsio_trx", NULL, 0x5c, 30),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_PCIE2, "pcie2", NULL, 0x8c, 0),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_LITE, "lite", NULL, 0x8c, 6),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_MIPI_DSI, "mipi_dsi", NULL, 0x8c, 8),
	GATE_NORMAL(RTD1319D_CRT_CLK_EN_CABLERX_Q, "cablerx_q", NULL, 0x8c, 16),
};

static struct rtk_reset_bank cc_reset_banks[] = {
	{ .ofs = 0x000, .write_en = 1, },
	{ .ofs = 0x004, .write_en = 1, },
	{ .ofs = 0x008, .write_en = 1, },
	{ .ofs = 0x00c, .write_en = 1, },
	{ .ofs = 0x068, .write_en = 1, },
	{ .ofs = 0x090, .write_en = 1, },
	{ .ofs = 0x454, },
	{ .ofs = 0x458, },
	{ .ofs = 0x464, },
	{ .ofs = 0x0b8, .write_en = 1, }
};

static struct rtk_reset_initdata cc_reset_initdata = {
	.banks     = cc_reset_banks,
	.num_banks = ARRAY_SIZE(cc_reset_banks),
};

static struct clk_hw_map rtd1319d_sys_map = {
	.group = &(struct clk_hw_group) {
		.hws = cc_hws,
		.num_hws = ARRAY_SIZE(cc_hws),
	},
	.start_index = 0,
};

static struct clk_hw_map rtd1319d_sys_n_map = {
	.group = &(struct clk_hw_group) {
		.hws = cc_n_hws,
		.num_hws = ARRAY_SIZE(cc_n_hws),
	},
	.start_index = 0,
};

static int rtd1319d_cc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rtk_clk_data *data;
	int ret;
	const struct clk_hw_map *map;

	map = of_device_get_match_data(dev);
	if (!map)
		return -EINVAL;

	data = rtk_clk_alloc_data(RTD1319D_CRT_CLK_MAX);
	if (!data)
		return -ENOMEM;

	ret = rtk_clk_of_init_data(np, data);
	if (ret) {
		rtk_clk_free_data(data);
		return ret;
	}

	of_rtk_clk_setup_crt(np, data->regmap);

	platform_set_drvdata(pdev, data);

	rtk_clk_add_hw_map(dev, data, map);

	rtk_clk_add_composites(dev, data, cc_composites,
			       ARRAY_SIZE(cc_composites));
	rtk_clk_add_gates(dev, data, cc_gates, ARRAY_SIZE(cc_gates));

	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &data->clk_data);
	if (ret)
		dev_err(dev, "failed to add clk provider: %d\n", ret);

	cc_reset_initdata.lock = data->lock;
	cc_reset_initdata.regmap = data->regmap;
	rtk_reset_controller_add(dev, &cc_reset_initdata);

	return 0;
}

static const struct of_device_id rtd1319d_cc_match[] = {
	{ .compatible = "realtek,rtd1319d-crt-clk", .data = &rtd1319d_sys_map,},
	{ .compatible = "realtek,rtd1319d-crt-clk-n", .data = &rtd1319d_sys_n_map,},
	{ /* sentinel */ }
};

static struct platform_driver rtd1319d_cc_driver = {
	.probe = rtd1319d_cc_probe,
	.driver = {
		.name = "rtk-rtd1319d-crt-clk",
		.of_match_table = rtd1319d_cc_match,
	},
};

static int __init rtd1319d_cc_init(void)
{
	return platform_driver_register(&rtd1319d_cc_driver);
}
core_initcall(rtd1319d_cc_init);

MODULE_DESCRIPTION("Reatek RTD1319D CRT Controller Driver");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");
