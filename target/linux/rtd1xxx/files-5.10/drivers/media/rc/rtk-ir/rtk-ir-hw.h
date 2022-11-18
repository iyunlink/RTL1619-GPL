/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Realtek IR Hardware Decoder Controller.
 *
 * Copyright (C) 2020 Simon Hsu <simon_hsu@realtek.com>
 */

#ifndef _RTK_IR_HW_H_
#define _RTK_IR_HW_H_

struct rtk_ir;

struct ir_symbol_timing {
	unsigned short pulse;
	unsigned short space;
};

struct ir_timings {
	struct ir_symbol_timing ldr, s00, s01, s10, s11;
	unsigned int ft_min;
};

struct rtk_ir_scancode_req {
	enum rc_proto protocol;
	unsigned int scancode;
	unsigned int last_scancode;
	unsigned char toggle;
	unsigned char repeat;
};

struct rtk_ir_wakeinfo {
	unsigned int addr;
	unsigned int addr_msk;
	unsigned int scancode;
	unsigned int scancode_msk;
};

struct rtk_ir_hw_decoder {
	u64		type;
	unsigned int	tolerance;
	unsigned int	unit;
	unsigned int	sr;
	unsigned int	hw_set;

	struct ir_timings timings;
	struct ir_timings rtimings;

	int (*scancode)(struct rtk_ir_scancode_req *reqest, unsigned int raw);
	int (*wakeinfo)(struct rtk_ir_wakeinfo *info, unsigned int scancode);
};

struct rtk_ir_hw {
	struct rc_dev *rc;
	struct rtk_ir_hw_decoder *dec;
};

extern struct rtk_ir_hw_decoder rtk_ir_nec;
extern struct rtk_ir_hw_decoder rtk_ir_rstep;
extern struct rtk_ir_hw_decoder rtk_ir_xmp;
extern struct rtk_ir_hw_decoder rtk_ir_sky;

#ifdef CONFIG_IR_RTK_HW
void rtk_ir_hw_suspend(struct rtk_ir *ir);
void rtk_ir_hw_resume(struct rtk_ir *ir);
void rtk_ir_isr_hw(struct rtk_ir *ir);
int rtk_ir_hw_probe(struct rtk_ir *ir);
#else
#define rtk_ir_hw_suspend(x) (void)0
#define rtk_ir_hw_resume(x) (void)0
#define rtk_ir_isr_hw(x) (void)0
#define rtk_ir_hw_probe(x) -1
#endif

#ifdef CONFIG_IR_RTK_HW1
void rtk_ir_hw1_suspend(struct rtk_ir *ir);
void rtk_ir_hw1_resume(struct rtk_ir *ir);
void rtk_ir_isr_hw1(struct rtk_ir *ir);
int rtk_ir_hw1_probe(struct rtk_ir *ir);
#else
#define rtk_ir_hw1_suspend(x) (void)0
#define rtk_ir_hw1_resume(x) (void)0
#define rtk_ir_isr_hw1(x) (void)0
#define rtk_ir_hw1_probe(x) -1
#endif

#endif /* _RTK_IR_HW_H_ */
