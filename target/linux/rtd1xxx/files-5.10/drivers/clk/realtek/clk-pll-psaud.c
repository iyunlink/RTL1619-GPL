// SPDX-License-Identifier: GPL-2.0-only
/*
 * clk-pll-psaud.c - PLL_PSAUDXA
 *
 * Copyright (c) 2019 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/clk-provider.h>
#include "common.h"
#include "clk-pll.h"

static int clk_pll_psaud_enable(struct clk_hw *hw)
{
	struct clk_pll_psaud *pll = to_clk_pll_psaud(hw);
	u32 mask = 0, val = 0;

	if (pll->id == CLK_PLL_PSAUD1A) {
		mask = 0x3;
		val = 0x1;
	} else {
		mask = 0xc;
		val = 0x4;
	}
	clk_regmap_update(&pll->clkr, pll->reg + 4, mask, val);
	return 0;
}

static void clk_pll_psaud_disable(struct clk_hw *hw)
{
	struct clk_pll_psaud *pll = to_clk_pll_psaud(hw);
	u32 mask = 0, val = 0;

	if (pll->id == CLK_PLL_PSAUD1A) {
		mask = 0x3;
		val = 0x3;
	} else {
		mask = 0xc;
		val = 0xc;
	}
	clk_regmap_update(&pll->clkr, pll->reg + 4, mask, val);
}

static void clk_pll_psaud_disable_unused(struct clk_hw *hw)
{
	pr_info("%pC: %s\n", hw->clk, __func__);
	clk_pll_psaud_disable(hw);
}

static int clk_pll_psaud_is_enabled(struct clk_hw *hw)
{
	struct clk_pll_psaud *pll = to_clk_pll_psaud(hw);
	u32 val;

	val = clk_regmap_read(&pll->clkr, pll->reg + 4);
	if (pll->id == CLK_PLL_PSAUD1A)
		val &= 0x3;
	else
		val >>= 2;
	return val == 0x1;
}

static long clk_pll_psaud_round_rate(struct clk_hw *hw,
				     unsigned long rate,
				     unsigned long *parent_rate)
{
	if (45158400 == rate)
		return rate;
	return 49192000;
}

static int clk_pll_psaud_set_rate(struct clk_hw *hw,
				  unsigned long rate,
				  unsigned long parent_rate)
{
	struct clk_pll_psaud *pll = to_clk_pll_psaud(hw);
	u32 rsel = 0;
	u32 mask = 0, val = 0;

	if (WARN_ON_ONCE(rate != 45158400 && rate != 49192000))
		return -EINVAL;

	if (rate == 45158400)
		rsel = 1;
	if (pll->id == CLK_PLL_PSAUD1A) {
		val  = 0x6a0 | (rsel << 8);
		mask = 0x7e0;
	} else {
		val  = 0x19 | (rsel << 2);
		mask = 0x1f;
	}
	clk_regmap_update(&pll->clkr, pll->reg, mask, val);
	return 0;
}

static unsigned long clk_pll_psaud_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct clk_pll_psaud *pll = to_clk_pll_psaud(hw);
	u32 val;
	u32 rsel = 0;

	val = clk_regmap_read(&pll->clkr, pll->reg);

	if (pll->id == CLK_PLL_PSAUD1A)
		rsel = !!(val & BIT(8));
	else
		rsel = !!(val & BIT(2));

	return rsel ? 45158400 : 49192000;
}

const struct clk_ops clk_pll_psaud_ops = {
	.enable         = clk_pll_psaud_enable,
	.disable        = clk_pll_psaud_disable,
	.disable_unused = clk_pll_psaud_disable_unused,
	.is_enabled     = clk_pll_psaud_is_enabled,
	.set_rate       = clk_pll_psaud_set_rate,
	.round_rate     = clk_pll_psaud_round_rate,
	.recalc_rate    = clk_pll_psaud_recalc_rate,
};
EXPORT_SYMBOL_GPL(clk_pll_psaud_ops);
