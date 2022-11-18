// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2019 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include "common.h"
#include "clk-pll.h"
#include "clk-regmap-gate.h"
#include "clk-regmap-mux.h"
#include "reset.h"
#include <dt-bindings/clock/rtd1195-clk.h>

#define DIV_DV(_r, _div, _val) { .rate = _r, .div = _div, .val = _val, }
#define FREQ_NF_MASK    0x1FFFFC00
#define FREQ_NF(_r, _n, _f) { .rate = _r, .val = ((_n) << 21) | ((_f) << 10), }
#define FREQ_MNO_MASK   0x030FF800
#define FREQ_MNO(_r, _m, _n, _o) \
	{ .rate = _r, .val = ((_m) << 11) | ((_n) << 18) | ((_o) << 24), }
#define FREQ_MN_MASK    0x07FC0000
#define FREQ_MN(_r, _m, _n) { .rate = _r, .val = ((_m) << 18) | ((_n) << 25), }


static const struct div_table scpu_div_tbl[] = {
	DIV_DV(700000000, 1, 0),
	DIV_DV(350000000, 2, 2),
	DIV_DV(290000000, 4, 3),
	DIV_TABLE_END
};

static const struct freq_table scpu_tbl[] = {
	FREQ_NF(720000000,  25, 1364),
	FREQ_NF(780000000,  27, 1820),
	FREQ_NF(800000000,  28, 1290),
	FREQ_NF(1000000000, 36,   75),
	FREQ_NF(1160000000, 41, 1159),
	FREQ_NF(1200000000, 43,  910),
	FREQ_TABLE_END
};

static struct clk_pll_div pll_scpu = {
	.div_ofs    = 0x030,
	.div_shift  = 7,
	.div_width  = 2,
	.div_tbl    = scpu_div_tbl,
	.clkp       = {
		.pll_type     = CLK_PLL_TYPE_NF,
		.freq_mask    = FREQ_NF_MASK,
		.freq_tbl     = scpu_tbl,
		.pll_ofs      = 0x100,
		.clkr.hw.init = CLK_HW_INIT("pll_scpu", "osc27m", &clk_pll_div_ops, CLK_IGNORE_UNUSED | CLK_GET_RATE_NOCACHE),
	},
};

static const struct div_table bus_div_tbl[] = {
	DIV_DV(250000000, 2, 0),
	DIV_DV(1,         4, 1),
	DIV_TABLE_END
};

static const struct freq_table bus_tbl[] = {
	FREQ_MN(459000000, 15, 0),
	FREQ_TABLE_END
};

static struct clk_pll_div pll_bus = {
	.div_ofs    = 0x030,
	.div_shift  = 0,
	.div_width  = 1,
	.div_tbl    = bus_div_tbl,
	.clkp       = {
		.pll_type     = CLK_PLL_TYPE_MNO,
		.freq_mask    = FREQ_MN_MASK,
		.freq_tbl     = bus_tbl,
		.pll_ofs      = 0x164,
		.pow_loc      = CLK_PLL_CONF_POW_LOC_CTL3,
		.clkr.hw.init = CLK_HW_INIT("pll_bus", "osc27m", &clk_pll_div_ops, CLK_IGNORE_UNUSED | CLK_GET_RATE_NOCACHE),
	},
};

static struct clk_fixed_factor clk_sys = {
	.div     = 1,
	.mult    = 1,
	.hw.init = CLK_HW_INIT("clk_sys", "pll_bus", &clk_fixed_factor_ops, 0),
};

static const struct freq_table dcsb_tbl[] = {
	FREQ_MN(351000000, 11, 0),
	FREQ_TABLE_END
};

static struct clk_pll pll_dcsb = {
	.pll_type  = CLK_PLL_TYPE_MNO,
	.freq_mask = FREQ_MN_MASK,
	.freq_tbl  = dcsb_tbl,
	.pll_ofs   = 0x1B4,
	.pow_loc   = CLK_PLL_CONF_POW_LOC_CTL3,
	.clkr.hw.init = CLK_HW_INIT("pll_dcsb", "osc27m", &clk_pll_ops, CLK_IGNORE_UNUSED | CLK_GET_RATE_NOCACHE),
};

static struct clk_fixed_factor clk_sysh = {
	.div     = 1,
	.mult    = 1,
	.hw.init = CLK_HW_INIT("clk_sysh", "pll_dscb", &clk_fixed_factor_ops, CLK_SET_RATE_PARENT),
};

static const struct freq_table gpu_tbl[] = {
	FREQ_MNO(378000000, 12, 0, 0),
	FREQ_TABLE_END
};

static struct clk_pll pll_gpu = {
	.pll_type  = CLK_PLL_TYPE_MNO,
	.freq_mask = FREQ_MNO_MASK,
	.freq_tbl  = gpu_tbl,
	.pll_ofs   = 0x1C0,
	.pow_loc   = CLK_PLL_CONF_POW_LOC_CTL2,
	.clkr.hw.init = CLK_HW_INIT("pll_gpu", "osc27m", &clk_pll_ops, CLK_GET_RATE_NOCACHE),
};

static const struct freq_table vcpu_tbl[] = {
	FREQ_MNO(243000000,  7, 0, 0),
	FREQ_MNO(324000000, 10, 0, 0),
	FREQ_MNO(405000000, 13, 0, 0),
	FREQ_TABLE_END
};

static struct clk_pll pll_vcpu = {
	.pll_type  = CLK_PLL_TYPE_MNO,
	.freq_mask = FREQ_MNO_MASK,
	.freq_tbl  = vcpu_tbl,
	.pll_ofs   = 0x114,
	.pow_loc   = CLK_PLL_CONF_POW_LOC_CTL2,
	.clkr.hw.init = CLK_HW_INIT("pll_vcpu", "osc27m", &clk_pll_ops, CLK_GET_RATE_NOCACHE),
};

static struct clk_hw *cc_hws[] = {
	[RTD1195_CRT_PLL_SCPU]   = &__clk_pll_div_hw(&pll_scpu),
	[RTD1195_CRT_PLL_BUS]    = &__clk_pll_div_hw(&pll_bus),
	[RTD1195_CRT_PLL_DCSB]   = &__clk_pll_hw(&pll_dcsb),
	[RTD1195_CRT_PLL_GPU]    = &__clk_pll_hw(&pll_gpu),
	[RTD1195_CRT_PLL_VCPU]   = &__clk_pll_hw(&pll_vcpu),
	[RTD1195_CRT_CLK_SYS]    = &clk_sys.hw,
	[RTD1195_CRT_CLK_SYSH]   = &clk_sysh.hw,
};

static struct clk_composite_data cc_composites[] = {
	{
		.id           = RTD1195_CRT_CLK_GPU,
		.gate_ofs     = 0x00C,
		.gate_shift   = 11,
		.mux_ofs      = CLK_OFS_INVALID,
		.parent_names = (const char *[]){ "pll_gpu", },
		.num_parents  = 1,
		.name         = "clk_gpu",
		.flags        = CLK_SET_RATE_PARENT,
	},
	{
		.id           = RTD1195_CRT_CLK_VE,
		.mux_ofs      = CLK_OFS_INVALID,
		.gate_ofs     = 0x010,
		.gate_shift   = 5,
		.parent_names = (const char *[]){ "pll_vcpu", },
		.num_parents  = 1,
		.name         = "clk_ve",
		.flags        = CLK_IGNORE_UNUSED | CLK_SET_RATE_PARENT |
				CLK_SET_RATE_NO_REPARENT,
	},
	{
		.id           = RTD1195_CRT_CLK_VE1,
		.mux_ofs      = 0x04C,
		.mux_width    = 2,
		.mux_shift    = 0,
		.gate_ofs     = 0x00C,
		.gate_shift   = 12,
		.parent_names = (const char *[]){
			"clk_sys",
			"clk_sysh",
			"clk_ve",
		},
		.num_parents  = 3,
		.name         = "clk_ve1",
		.flags        = CLK_IGNORE_UNUSED | CLK_SET_RATE_PARENT |
				CLK_SET_RATE_NO_REPARENT,
	},
	{
		.id           = RTD1195_CRT_CLK_VE2,
		.mux_ofs      = 0x04C,
		.mux_width    = 2,
		.mux_shift    = 2,
		.gate_ofs     = 0x00C,
		.gate_shift   = 29,
		.parent_names = (const char *[]){
			"clk_sys",
			"clk_sysh",
			"clk_ve",
		},
		.num_parents  = 3,
		.name         = "clk_ve2",
		.flags        = CLK_IGNORE_UNUSED | CLK_SET_RATE_PARENT |
				CLK_SET_RATE_NO_REPARENT,
	},
	{
		.id           = RTD1195_CRT_CLK_VE2_BPU,
		.mux_ofs      = 0x04C,
		.mux_width    = 2,
		.mux_shift    = 6,
		.gate_ofs     = CLK_OFS_INVALID,
		.parent_names = (const char *[]){
			"clk_sys",
			"clk_sysh",
			"clk_gpu",
		},
		.num_parents  = 3,
		.name         = "clk_ve2_bpu",
		.flags        = CLK_IGNORE_UNUSED | CLK_SET_RATE_PARENT |
				CLK_SET_RATE_NO_REPARENT,
	},
};

#define GATE_COMMON(_id, _name, _parent, _flags, _ofs, _shift) \
	CLK_GATE_DATA(_id, _name, _parent, _flags, _ofs, _shift, 0, 0)
#define GATE_CRITICAL(_id, _name, _parent, _ofs, _shift) \
	GATE_COMMON(_id, _name, _parent, CLK_IS_CRITICAL, _ofs, _shift)
#define GATE(_id, _name, _parent, _ofs, _shift) \
	GATE_COMMON(_id, _name, _parent, 0, _ofs, _shift)
#define GATE_IGNORE(_id, _name, _parent, _ofs, _shift) \
	GATE_COMMON(_id, _name, _parent, CLK_IGNORE_UNUSED, _ofs, _shift)

static struct clk_gate_data cc_gates[] = {
	GATE_CRITICAL(RTD1195_CRT_CLK_EN_MISC, "misc", NULL,  0x0C, 0),
	GATE(RTD1195_CRT_CLK_EN_HDMIRX, "hdmirx", NULL, 0x0C, 1),
	GATE(RTD1195_CRT_CLK_EN_GSPI, "gspi", "misc", 0x0C, 3),
	GATE(RTD1195_CRT_CLK_EN_USB, "usb", NULL, 0x0C, 4),
	GATE(RTD1195_CRT_CLK_EN_HDMI, "hdmi", NULL, 0x0C, 8),
	GATE(RTD1195_CRT_CLK_EN_ETN, "etn", NULL, 0x0C, 9),
	GATE(RTD1195_CRT_CLK_EN_VE_JPEG, "jpeg", NULL, 0x0C, 13),
	GATE(RTD1195_CRT_CLK_EN_SE, "se", NULL, 0x0C, 17),
	GATE(RTD1195_CRT_CLK_EN_CP, "cp", NULL, 0x0C, 19),
	GATE(RTD1195_CRT_CLK_EN_MD, "md", NULL, 0x0C, 20),
	GATE(RTD1195_CRT_CLK_EN_TP, "tp", NULL, 0x0C, 21),
	GATE(RTD1195_CRT_CLK_EN_NF, "nf", NULL, 0x0C, 23),
	GATE(RTD1195_CRT_CLK_EN_EMMC, "emmc", NULL, 0x0C, 24),
	GATE(RTD1195_CRT_CLK_EN_CR, "cr", NULL, 0x0C, 25),
	GATE(RTD1195_CRT_CLK_EN_SDIO_IP, "sdio_ip", NULL, 0x0C, 26),
	GATE(RTD1195_CRT_CLK_EN_MIPI, "mipi", NULL, 0x0C, 27),
	GATE(RTD1195_CRT_CLK_EN_EMMC_IP, "emmc_ip", NULL, 0x0C, 28),
	GATE(RTD1195_CRT_CLK_EN_SDIO, "sdio", NULL, 0x0C, 30),
	GATE(RTD1195_CRT_CLK_EN_SD_IP, "sd_ip", NULL, 0x0C, 31),
	GATE(RTD1195_CRT_CLK_EN_MISC_I2C_5, "i2c5", "misc", 0x10, 2),
	GATE(RTD1195_CRT_CLK_EN_MISC_RTC, "rtc", "misc", 0x10, 9),
	GATE(RTD1195_CRT_CLK_EN_MISC_I2C_4, "i2c4", "misc", 0x10, 13),
	GATE(RTD1195_CRT_CLK_EN_MISC_I2C_3, "i2c3", "misc", 0x10, 14),
	GATE(RTD1195_CRT_CLK_EN_MISC_I2C_2, "i2c2", "misc", 0x10, 15),
	GATE(RTD1195_CRT_CLK_EN_MISC_I2C_1, "i2c1", "misc", 0x10, 16),
	GATE(RTD1195_CRT_CLK_EN_UR1, "ur1", "misc", 0x10, 28),
};

static struct rtk_reset_bank cc_reset_banks[] = {
	{ .ofs = 0x00, },
	{ .ofs = 0x04, },
};

static struct rtk_reset_initdata cc_reset_initdata = {
	.banks     = cc_reset_banks,
	.num_banks = ARRAY_SIZE(cc_reset_banks),
};

static int rtd1195_cc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rtk_clk_data *data;
	int ret;

	data = rtk_clk_alloc_data(RTD1195_CRT_CLK_MAX);
	if (!data)
		return -ENOMEM;

	ret = rtk_clk_of_init_data(np, data);
	if (ret) {
		rtk_clk_free_data(data);
		return ret;
	}

	rtk_clk_add_hws(dev, data, cc_hws, ARRAY_SIZE(cc_hws));
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

static const struct of_device_id rtd1195_cc_match[] = {
	{ .compatible = "realtek,rtd1195-crt-clk", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtd1195_cc_match);

static struct platform_driver rtd1195_cc_driver = {
	.probe = rtd1195_cc_probe,
	.driver = {
		.name = "rtk-rtd1195-crt-clk",
		.of_match_table = of_match_ptr(rtd1195_cc_match),
	},
};

static int __init rtd1195_cc_init(void)
{
	return platform_driver_register(&rtd1195_cc_driver);
}
core_initcall(rtd1195_cc_init);
MODULE_DESCRIPTION("Reatek RTD1195 CRT Controller Driver");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");
