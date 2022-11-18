// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Realtek IR HW Receiver Controller
 *
 * Copyright (C) 2020 Simon Hsu <simon_hsu@realtek.com>
 */

#include <linux/of_platform.h>
#include "rtk-ir.h"

#define RTK_IR_HW "rtk-ir-hw"

static struct rtk_ir_hw_decoder *rtk_ir_hw_decoders = {
#ifdef CONFIG_IR_RTK_HW_NEC
	&rtk_ir_nec,
#endif
#ifdef CONFIG_IR_RTK_HW_RSTEP
	&rtk_ir_rstep,
#endif
#ifdef CONFIG_IR_RTK_HW_XMP
	&rtk_ir_xmp,
#endif
#ifdef CONFIG_IR_RTK_HW_SKY
	&rtk_ir_sky,
#endif
};

static void rtk_ir_hw_init(struct rtk_ir *ir)
{
	struct rtk_ir_hw_decoder *dec = ir->hw.dec;
	struct rc_dev *rc = ir->hw.rc;
	void __iomem *base = ir->base;
	unsigned int ldr_p, ldr_s, mod, one, zero, repeat;
	unsigned int val;

	if (rc->rc_map.rc_proto == RC_PROTO_XMP) {
		writel(0x1a, base + IR_SF);
		writel(0x10001, base + IR_PER);
		writel(0x2ee0, base + IR_DPIR);
		writel(0x11d89385, base + IRCOMCAST_TIMING);
	} else {
		writel((dec->sr * 27 - 1), base + IR_SF);

		ldr_p = (dec->timings.ldr.pulse * dec->unit) / (4 * dec->sr);
		ldr_p = ldr_p * dec->tolerance / 100;
		ldr_s = (dec->timings.ldr.space * dec->unit) / (4 * dec->sr);
		ldr_s = ldr_s * dec->tolerance / 100;
		mod = (dec->unit / dec->sr) * dec->tolerance / 100;
		zero = (dec->timings.s00.space * dec->unit) / dec->sr;
		zero = zero * dec->tolerance / 100;
		one = (dec->timings.s01.space * dec->unit) / dec->sr;
		one = one * dec->tolerance / 100;
		repeat = (dec->rtimings.ldr.space * dec->unit) / (4 * dec->sr);
		repeat = repeat * dec->tolerance / 100;

		val = (ldr_p & 0xff) << 24 | (mod & 0xff) << 16 |
			(zero & 0xff) << 8 | (one & 0xff);

		writel(val, base + IR_PSR);

		val = 1 << 16 | (repeat & 0xff) << 8 | (ldr_s & 0xff);
		writel(val, base + IR_PER);

		writel(dec->timings.ft_min / dec->sr, base + IR_DPIR);
		if (rc->rc_map.rc_proto == RC_PROTO_RC6_0)
			writel(0x123, base + IR_CTRL_RC6);
	}
	val = readl(base + IR_CR) & RAW_MASK;
	val |= dec->hw_set;
	writel(val, base + IR_CR);
}

void rtk_ir_hw_suspend(struct rtk_ir *ir)
{
	void __iomem *base = ir->base;

	writel((readl(base + IR_CR) & ~IRIE), base + IR_CR);

	while (readl(base + IR_SR) & IR_DVF) {
		writel((IR_DVF | IR_REPEAT), base + IR_SR);
		readl(base + IR_RP);
	}
}

void rtk_ir_hw_resume(struct rtk_ir *ir)
{
	void __iomem *base = ir->base;

	while (readl(base + IR_SR) & IR_DVF) {
		writel((IR_DVF | IR_REPEAT), base + IR_SR);
		readl(base + IR_RP);
	}

	writel((readl(base + IR_CR) | IRIE), base + IR_CR);
}

void rtk_ir_isr_hw(struct rtk_ir *ir)
{
	struct rtk_ir_hw_decoder *dec = ir->hw.dec;
	struct rc_dev *rc = ir->hw.rc;
	struct rtk_ir_scancode_req request;
	void __iomem *base = ir->base;
	unsigned int repeat = 0, data;

	data = readl(base + IR_SR);
	if (!(data & IR_DVF))
		return;
	if (data & IR_REPEAT)
		repeat = 1;

	writel((IR_DVF | IR_REPEAT), base + IR_SR);
	data = readl(base + IR_RP);

	if (repeat) {
		rc_repeat(ir->hw.rc);
		return;
	}

	request.protocol = RC_PROTO_UNKNOWN;
	request.toggle = 0;
	if (!dec->scancode)
		return;

	request.last_scancode = rc->last_scancode;
	request.repeat = 0;
	if(dec->scancode(&request, data)) {
		if (request.repeat)
			rc_repeat(ir->hw.rc);
		else
			rc_keydown(ir->hw.rc, request.protocol, request.scancode,
				   request.toggle);
	}
}

int rtk_ir_hw_probe(struct rtk_ir *ir)
{
	struct device *dev = ir->dev;
	struct device_node *dn = dev->of_node;
	struct rc_dev *rc;
	const char *map_name;
	int ret;

	rc = devm_rc_allocate_device(dev, RC_DRIVER_SCANCODE);
	if (!rc) {
		dev_err(dev, "failed to allocate device\n");
		return -ENOMEM;
	}

	rc->priv = ir;
	rc->device_name = RTK_IR_HW;
	rc->input_phys = RTK_IR_HW "/input0";
	rc->dev.parent = dev;
	rc->driver_name = RTK_IR_HW;
	rc->timeout = MS_TO_US(10);

	map_name = of_get_property(dn, "hw,rc-map-name", NULL);
	rc->map_name = map_name ?: RC_MAP_EMPTY;

	ret = devm_rc_register_device(dev, rc);
	if (ret) {
		dev_err(dev, "failed to register rc device\n");
		return ret;
	}

	ir->hw.rc = rc;
	ir->hw.dec = rtk_ir_hw_decoders;

	rtk_ir_hw_init(ir);

	return 0;
}
