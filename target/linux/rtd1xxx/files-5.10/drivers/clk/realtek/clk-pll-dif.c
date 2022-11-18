// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include "common.h"
#include "clk-pll.h"

#define PLL_ADTV1   0x0
#define PLL_ADTV2   0x4
#define PLL_ADTV3   0x8
#define PLL_ADTV4   0xc
#define PLL_ADTV5   (-0xc)
#define PLL_ADTV6   (-0x8)

#define PLL_SSC_DIG_PLLDIF0   0x0
#define PLL_SSC_DIG_PLLDIF1   0x4
#define PLL_SSC_DIG_PLLDIF2   0x8
#define PLL_SSC_DIG_PLLDIF3   0xc
#define PLL_SSC_DIG_PLLDIF4   0x10
#define PLL_SSC_DIG_PLLDIF5   0x14
#define PLL_SSC_DIG_PLLDIF_DBG1   (0x18)
#define PLL_SSC_DIG_PLLDIF_DBG2   (0x1c)

static int clk_pll_dif_enable(struct clk_hw *hw)
{
	struct clk_pll_dif *pll = to_clk_pll_dif(hw);

	pr_debug("%pC: %s\n", hw->clk, __func__);
	pll->status = 1;

	clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV4, pll->adtv_conf[0]);
	clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV3, pll->adtv_conf[1]);
	clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV2, pll->adtv_conf[2]);
	clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV1, pll->adtv_conf[3]);
	udelay(100);

	clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV3, pll->adtv_conf[4]);
	udelay(50);

	clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV3, pll->adtv_conf[5]);
	udelay(200);

	clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV4, pll->adtv_conf[6]);
	udelay(100);

	clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV2, pll->adtv_conf[7]);

	/* ssc control */
	clk_regmap_write(&pll->clkr, pll->ssc_ofs + PLL_SSC_DIG_PLLDIF0, 0x00000004);
	clk_regmap_write(&pll->clkr, pll->ssc_ofs + PLL_SSC_DIG_PLLDIF1, 0x00006800);
	clk_regmap_write(&pll->clkr, pll->ssc_ofs + PLL_SSC_DIG_PLLDIF3, 0x00000000);
	clk_regmap_write(&pll->clkr, pll->ssc_ofs + PLL_SSC_DIG_PLLDIF4, 0x00000000);
	clk_regmap_write(&pll->clkr, pll->ssc_ofs + PLL_SSC_DIG_PLLDIF2, 0x001e1f98);
	clk_regmap_write(&pll->clkr, pll->ssc_ofs + PLL_SSC_DIG_PLLDIF0, 0x00000005);

	return 0;
}

static void clk_pll_dif_disable(struct clk_hw *hw)
{
	struct clk_pll_dif *pll = to_clk_pll_dif(hw);

	pr_debug("%pC: %s\n", hw->clk, __func__);

	clk_regmap_update(&pll->clkr, pll->pll_ofs + PLL_ADTV2, 0x00080000, 0x0);
	clk_regmap_update(&pll->clkr, pll->pll_ofs + PLL_ADTV3, 0x00400C03, 0x0);
	clk_regmap_update(&pll->clkr, pll->pll_ofs + PLL_ADTV4, 0x00000038, 0x0);

	clk_regmap_write(&pll->clkr, pll->ssc_ofs + PLL_SSC_DIG_PLLDIF0, 0x00000004);
	pll->status = 0;
}

static int clk_pll_dif_is_enabled(struct clk_hw *hw)
{
	struct clk_pll_dif *pll = to_clk_pll_dif(hw);

	return pll->status;
}

static void clk_pll_dif_disable_unused(struct clk_hw *hw)
{
	pr_info("%pC: %s\n", hw->clk, __func__);
	clk_pll_dif_disable(hw);
}

const struct clk_ops clk_pll_dif_ops = {
	.enable           = clk_pll_dif_enable,
	.disable          = clk_pll_dif_disable,
	.disable_unused   = clk_pll_dif_disable_unused,
	.is_enabled       = clk_pll_dif_is_enabled,
};
EXPORT_SYMBOL_GPL(clk_pll_dif_ops);

static int clk_pll_dif_v2_enable(struct clk_hw *hw)
{
	struct clk_pll_dif *pll = to_clk_pll_dif(hw);

	pr_debug("%pC: %s\n", hw->clk, __func__);

	pll->status = 1;

	clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV6, 0x000000b0);
	udelay(100);

	clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV6, 0x000200b0);
	udelay(100);

	clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV6, 0x000300b0);
	udelay(100);

	switch (pll->freq) {
	case 216000000:
		clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV6, 0x000305bf);
		clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV3, 0xd40a0903);
		clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV2, 0x00100868);
		clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV1, 0x03002076);
		break;

	case 186600000:
	default:
		clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV6, 0x000300bf);
		clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV3, 0xd4410780);
		clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV2, 0x00100828);
		clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV1, 0x41002076);
		pll->freq = 186600000;
		break;
	}

	/* improve ifadc */
	clk_regmap_update(&pll->clkr, pll->pll_ofs + PLL_ADTV1, 0x3800, 0x2800);
	clk_regmap_update(&pll->clkr, pll->pll_ofs + PLL_ADTV5, 0x70000, 0x50000);

	return 0;
}

static void clk_pll_dif_v2_disable(struct clk_hw *hw)
{
	struct clk_pll_dif *pll = to_clk_pll_dif(hw);

	pr_debug("%pC: %s\n", hw->clk, __func__);

	clk_regmap_write(&pll->clkr, pll->pll_ofs + PLL_ADTV6, 0);

	pll->status = 0;
}

static void clk_pll_dif_v2_disable_unused(struct clk_hw *hw)
{
	pr_info("%pC: %s\n", hw->clk, __func__);
	clk_pll_dif_v2_disable(hw);
}

static long clk_pll_dif_v2_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	return (rate < 216000000 ? 186600000 : 216000000);
}

static unsigned long clk_pll_dif_v2_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_pll_dif *pll = to_clk_pll_dif(hw);

	return pll->freq;
}

static int clk_pll_dif_v2_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct clk_pll_dif *pll = to_clk_pll_dif(hw);

	pll->freq = rate;
	return 0;
}

const struct clk_ops clk_pll_dif_v2_ops = {
	.set_rate         = clk_pll_dif_v2_set_rate,
	.recalc_rate      = clk_pll_dif_v2_recalc_rate,
	.round_rate       = clk_pll_dif_v2_round_rate,
	.enable           = clk_pll_dif_v2_enable,
	.disable          = clk_pll_dif_v2_disable,
	.disable_unused   = clk_pll_dif_v2_disable_unused,
	.is_enabled       = clk_pll_dif_is_enabled,
};
EXPORT_SYMBOL_GPL(clk_pll_dif_v2_ops);
