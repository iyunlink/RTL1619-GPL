/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Realtek IR Raw Decoder Controller.
 *
 * Copyright (C) 2020 Simon Hsu <simon_hsu@realtek.com>
 */

#ifndef _RTK_IR_RAW_H_
#define _RTK_IR_RAW_H_

struct rtk_ir;

struct rtk_ir_raw {
	struct rc_dev *rc;

};

#ifdef CONFIG_IR_RTK_RAW
void rtk_ir_isr_raw(struct rtk_ir *ir);
int rtk_ir_raw_probe(struct rtk_ir *ir);
#else
#define rtk_ir_isr_raw(x) (void)0
#define rtk_ir_raw_probe(x) -1
#endif


#endif /* _RTK_IR_RAW_H_ */
