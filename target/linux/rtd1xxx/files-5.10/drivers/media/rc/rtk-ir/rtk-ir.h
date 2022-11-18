/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ImgTec IR Decoder found in PowerDown Controller.
 *
 * Copyright 2010-2014 Imagination Technologies Ltd.
 */

#ifndef _RTK_IR_H_
#define _RTK_IR_H_

#include <media/rc-core.h>

#include <soc/realtek/rtk_pm.h>

#include "rtk-ir-hw.h"
#include "rtk-ir-raw.h"

#define IR_PSR			(0x00)
#define IR_PER			(0x04)
#define IR_SF			(0x08)
#define IR_DPIR			(0x0c)
#define IR_CR			(0x10)
#define IR_RP			(0x14)
#define IR_SR			(0x18)
#define IR_RAW_CTRL		(0x1c)
#define IR_RAW_FF		(0x20)
#define IR_RAW_SAMPLE_TIME	(0x24)
#define IR_RAW_WL		(0x28)
#define IR_RAW_DEB		(0x2c)
#define IR_PSR_UP		(0x30)
#define IR_PER_UP		(0x34)
#define IR_CTRL_RC6		(0x38)
#define IR_RP2			(0x3C)
#define IRTX_CFG		(0x40)
#define IRTX_TIM		(0x44)
#define IRCOMCAST1_TIMING	(0x44)
#define IRTX_PWM_SETTING	(0x48)
#define IRTX_INT_EN		(0x4c)
#define IRTX_INT_ST		(0x50)
#define IRTX_FIFO_ST		(0x54)
#define IRTX_FIFO		(0x58)
#define IRRCMM_TIMING		(0x60)
#define IR_CR1			(0x64)
#define IRRCMM_APKB		(0x68)
#define IRRXRCLFIFO		(0x6C)
#define IRDIRECTV_TIMING1	(0x70)
#define IRDIRECTV_TIMING2	(0x74)
#define IRCOMCAST_TIMING	(0x78)
#define IR_RAW_SF		(0xC8)

/* define for register IR_CR */
#define IRDN_MSK		0x1F
#define	IRDPM			(1<<5)
#define IRBME			(1<<6)
#define	IRCM			(1<<7)
#define	IRUE			(1<<8)
#define	IRRES			(1<<9)
#define	IRIE			(1<<10)
#define MLAE			(1<<11)
#define	RAWEN			(1<<12)
#define	RAWIE			(1<<13)
#define	RAWOV			(1<<14)
#define RAW_MASK		(0x7<<12)
#define	IREDN_MSK		(0x3F<<16)
#define	IREDN_EN		(1<<23)
#define TOSHIBA_EN		(1<<24)
#define	RCMM_EN			(1<<25)
#define COMCAST_EN		(1<<27)
#define REPEAT_EN		(1<<28)
#define	IRSR			(1<<31)

/* define for register IR_SR */
#define IR_DVF			(1<<0)
#define IR_REPEAT		(1<<1)
#define IR_RAWDVF		(1<<2)
#define IR_RAWDOV		(1<<3)
#define IR_REPEAT_EN		(1<<28)

/* define for register IR_RAW_CTRL */
#define STOP_TIME_SHIFT		8
#define WRITE_EN1		(1<<6)
#define STOP_SAMPLE		(1<<24)
#define	WRITE_EN2		(1<<25)

struct rtk_ir {
	struct device *dev;
	struct rtk_ir_hw hw;
	struct rtk_ir_hw hw1;
	struct rtk_ir_raw raw;

	struct pm_dev_param pm_param;
	struct notifier_block pm_notifier;
	struct ipc_shm_irda pcpu_data;

	void __iomem *base;
	struct regmap *iso;
	struct clk *clk;
	struct reset_control *rsts;
	int irq;
};

#endif /* _RTK_IR_H_ */
