// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Realtek IR RAW Receiver Controller
 *
 * Copyright (C) 2020 Simon Hsu <simon_hsu@realtek.com>
 */

#include <linux/of_platform.h>
#include "rtk-ir.h"

#define RTK_IR_RAW "rtk-ir-raw"

/* Maximum count of samples */
#define RTK_MAX_SAMPLES		0xffff
/* Sample period in us */
#define RAW_IR_SAMPLE		40

#define STOP_TIME		0x1388
#define FIFO_THRESHOLD		0x10
#define RXFIFO			128

static void rtk_process_ir_raw_data(struct rtk_ir *ir, unsigned int *fifo)
{
	struct rc_dev *rc = ir->raw.rc;
	struct ir_raw_event rawir = {};
	int bitptr, pulse, cnt = 0;

	bitptr = sizeof(unsigned int)*8 - 1;
	pulse = (*fifo >> bitptr) & 1 ? true : false;

	for ( ; bitptr >= 0; bitptr--) {
		if (((*fifo >> bitptr) & 1) == pulse) {
			cnt++;
		} else {
			rawir.pulse = !pulse;
			rawir.duration = US_TO_NS(cnt * RAW_IR_SAMPLE);
			ir_raw_event_store_with_filter(rc, &rawir);

			pulse = !pulse;
			cnt = 1;
		}
	}

	rawir.pulse = !pulse;
	rawir.duration = US_TO_NS(cnt * RAW_IR_SAMPLE);
	ir_raw_event_store_with_filter(rc, &rawir);
}

void rtk_ir_isr_raw(struct rtk_ir *ir)
{
	void __iomem *base = ir->base;
	unsigned int fifoval[RXFIFO], fifolv = 0, val;
	int i;

	val = readl(base + IR_SR);
	if (!(val & (IR_RAWDOV | IR_RAWDVF)))
		return;

	writel((val & (~0xf)), base + IR_SR);
	fifolv = readl(base + IR_RAW_WL) & 0x3f;
	for (i = 0; i < fifolv; i++) {
		fifoval[i] = readl(base + IR_RAW_FF);
		rtk_process_ir_raw_data(ir, &fifoval[i]);
	}

	ir_raw_event_handle(ir->raw.rc);
}

static int rtk_ir_raw_init(struct rtk_ir *ir)
{
	void __iomem *reg = ir->base;
	unsigned int val;

	writel((RAW_IR_SAMPLE * 27) - 1, reg + IR_RAW_SF);

	val = FIFO_THRESHOLD | WRITE_EN1 | (STOP_TIME << STOP_TIME_SHIFT) |
		STOP_SAMPLE | WRITE_EN2;
	writel(val, reg + IR_RAW_CTRL);

	val = readl(reg + IR_CR) | RAWEN | RAWIE | RAWOV;
	writel(val, reg + IR_CR);

	return 0;
}

int rtk_ir_raw_probe(struct rtk_ir *ir)
{
	struct device *dev = ir->dev;
	struct device_node *dn = dev->of_node;
	const char *map_name;
	struct rc_dev *rc;
	int ret;

	rc = rc_allocate_device(RC_DRIVER_IR_RAW);
	if (!rc) {
		dev_err(dev, "failed to allocate device\n");
		return -ENOMEM;
	}

	rc->priv = ir;
	map_name = of_get_property(dn, "raw,rc-map-name", NULL);
	rc->map_name = map_name ?: RC_MAP_EMPTY;
	rc->device_name = "RTK Infrared Decoder Raw";
	rc->allowed_protocols = RC_PROTO_BIT_ALL_IR_DECODER;
	rc->rx_resolution = US_TO_NS(RAW_IR_SAMPLE);
	rc->timeout = RTK_MAX_SAMPLES * (rc->rx_resolution + 1);

	ret = devm_rc_register_device(dev, rc);
	if (ret) {
		dev_err(dev, "failed to register rc device\n");
		return ret;
	}
	ir->raw.rc = rc;

	rtk_ir_raw_init(ir);

	return 0;
}
