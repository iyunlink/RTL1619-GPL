/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Copyright (C) 2021 Realtek Semiconductor Corporation
 *  Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#ifndef __CLK_REALTEK_CLK_DET_H
#define __CLK_REALTEK_CLK_DET_H

#include <linux/clk-provider.h>

struct clk_det {
	struct clk_hw hw;
	struct regmap *regmap;
	int ofs;
	int type;
	struct clk *ref;
};

#define to_clk_det(_hw) container_of(_hw, struct clk_det, hw)
#define __clk_det_hw(_ptr)  ((_ptr)->hw)

#define CLK_DET_TYPE_CRT      0
#define CLK_DET_TYPE_SC_WRAP  1

extern const struct clk_ops clk_det_ops;

static inline int is_clk_det_ops(const struct clk_ops *ops)
{
	return ops == &clk_det_ops;
}

#endif
