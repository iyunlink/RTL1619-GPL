/*
 * hdmitx_scdc.h - RTK hdmitx driver header file
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include "hdmitx_dev.h"

#ifndef _HDMITX_SCDC_H_
#define _HDMITX_SCDC_H_

#define SCDC_I2C_ADDR	0x54

#define SCDC_I2C_OK		0
#define SCDC_I2C_FAIL	1

/* Status and Control Data Channel Structure */
#define SCDCS_Sink_Version    0x01
#define SCDCS_Source_Version  0x02
#define SCDCS_Update_0      0x10
#define SCDCS_Update_1      0x11
#define SCDCS_TMDS_Config   0x20
#define SCDCS_Config_0      0x30
#define SCDCS_Config_1      0x31
#define SCDCS_Source_Test   0x35
#define SCDCS_Status_Flag_0 0x40
#define SCDCS_Status_Flag_1	0x41
#define SCDCS_Status_Flag_2 0x42
#define SCDCS_Err_Det_0_L   0x50
#define SCDCS_Err_Det_0_H   0x51
#define SCDCS_Err_Det_1_L   0x52
#define SCDCS_Err_Det_1_H   0x53
#define SCDCS_Err_Det_2_L   0x54
#define SCDCS_Err_Det_2_H   0x55
#define SCDCS_Err_Det_sum   0x56
#define SCDCS_Test_Config_0 0xC0

/* SCDCS_Source_Test */
#define FLT_NO_TIMEOUT      0x20

/* SCDCS_Update */
#define UPDATE0_STATUS      0x01
#define UPDATE0_FRL_START   0x10
#define UPDATE0_FLT_UPDATE  0x20

/* SCDCS_Status_Flag  */
#define STATUS_CLK_DET      0x01
#define STATUS_L0_LOCK      0x02
#define STATUS_L1_LOCK      0x04
#define STATUS_L2_LOCK      0x08
#define STATUS_L3_LOCK      0x10
#define STATUS_FLT_READY    0x40

/* I2C1 read request register offset */
#define I2C1_REQ_CTRL 0x200

/**
 * struct hdmitx_scdc_data
 * @*dev_nd: scdc_rr node
 * @enable_rr: flag of enable SCDC read request function
 * @iso_reg: I2C1 read request control registers
 * @i2c1_irq: I2C1 read request interrupt number
 * @rr_work: struct work_struct
 */
struct hdmitx_scdc_data {
	struct device_node *dev_nd;
	unsigned int enable_rr;
	void __iomem *iso_reg;
	unsigned int i2c1_irq;
	struct work_struct rr_work;
};

unsigned char hdmitx_send_scdc_TmdsConfig(struct edid_information *hdmitx_edid_info,
		 unsigned int standard, unsigned int dataInt0, unsigned char data_byte1);
void enable_hdmitx_scdcrr(hdmitx_device_t *tx_dev, unsigned char enable);
int register_hdmitx_scdcrr(struct device *dev);
void hdmitx_scdcrr_suspend(hdmitx_device_t *tx_dev);
void hdmitx_scdcrr_resume(hdmitx_device_t *tx_dev);

#endif /* _HDMITX_SCDC_H_ */
