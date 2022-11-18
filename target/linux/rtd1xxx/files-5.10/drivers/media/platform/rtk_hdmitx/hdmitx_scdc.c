/*
 * hdmitx_scdc.c - RTK hdmitx driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/i2c.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>

#include "hdmitx_scdc.h"
#include "hdmitx.h"
#include "hdmitx_dev.h"
#include "hdmitx_rpc.h"
#include "rtk_edid.h"



void hdmitx_scdc_work_func(struct work_struct *work);

int hdmitx_write_scdc_port(unsigned char offset, unsigned char value)
{
	struct i2c_adapter *p_adap;
	unsigned char bus_id = 1;
	unsigned char data[2];
	struct i2c_msg msgs;

	data[0] = offset;
	data[1] = value;

	msgs.addr = SCDC_I2C_ADDR;
	msgs.flags = 0;
	msgs.len = 2;
	msgs.buf = data;

	p_adap = i2c_get_adapter(bus_id);
	if (p_adap == NULL) {
		pr_err("[%s] get i2c adapter %d failed\n", __func__, bus_id);
		goto scdc_write_fal;
	}

	if (i2c_transfer(p_adap, &msgs, 1) != 1) {
		pr_err("Write SCDC port fail, offset(0x%02x) value(0x%02x)", offset, value);
		goto scdc_write_fal;
	} else {
		pr_devel("Write SCDC port, offset(0x%02x) value(0x%02x)", offset, value);
	}

	return SCDC_I2C_OK;
scdc_write_fal:
	return SCDC_I2C_FAIL;
}

int hdmitx_read_scdc_port(unsigned char offset, unsigned char *buf, unsigned char len)
{
	struct i2c_adapter *p_adap;
	unsigned char bus_id = 1;
	unsigned char data;
	struct i2c_msg msgs[2];

	data = offset;

	msgs[0].addr = SCDC_I2C_ADDR;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &data;

	msgs[1].addr = SCDC_I2C_ADDR;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;


	p_adap = i2c_get_adapter(bus_id);
	if (p_adap == NULL) {
		pr_err("[%s] get i2c adapter %d failed\n", __func__, bus_id);
		goto scdc_read_fail;
	}

	if (i2c_transfer(p_adap, &msgs[0], 2) != 2) {
		pr_err("Read SCDC port fail, offset(0x%02x) len(%u)",
			offset, len);
		goto scdc_read_fail;
	} else {
		pr_devel("Read SCDC port, offset(0x%02x) len(%u)",
			offset, len);
	}

	return SCDC_I2C_OK;
scdc_read_fail:
	return SCDC_I2C_FAIL;
}

/**
 * hdmitx_send_scdc_TmdsConfig - SCDC TMDS Configuration
 *
 * The maximum time period between write SCDC Scrambling_Enable
 * and output scrambled video is 100ms in spec.
 *
 * @hdmitx_edid_info: struct edid_information
 * @standard: VO_STANDARD
 * @dataInt0:  VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME dataInt0
 * @data_byte1: VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME dataByte1
 *
 * Return: SCDC TMDS_Config value
 */
unsigned char hdmitx_send_scdc_TmdsConfig(
	struct edid_information *hdmitx_edid_info, unsigned int standard,
	unsigned int dataInt0, unsigned char data_byte1)
{
	unsigned char config_data;
	unsigned char color;
	unsigned char deep_color;
	unsigned char depp_depth;
	unsigned char format_3D;
	int ret_val;

	color = data_byte1>>5;

	deep_color = (dataInt0>>1)&0x1;
	depp_depth = (dataInt0>>2)&0xF;
	format_3D = (dataInt0>>8)&0xF;

	if ((depp_depth != 5) && (depp_depth != 6))
		deep_color = 0;

	switch (standard) {
	/* Scramble, 1/40 data rate */
	case VO_STANDARD_HDTV_2160P_50:
	case VO_STANDARD_HDTV_2160P_50_422_12bit:
	case VO_STANDARD_HDTV_2160P_59:
	case VO_STANDARD_HDTV_2160P_60:
	case VO_STANDARD_HDTV_2160P_60_422_12bit:
	case VO_STANDARD_HDTV_4096_2160P_50:
	case VO_STANDARD_HDTV_4096_2160P_60:
	case VO_STANDARD_HDTV_2160P_59_422_12bit:
		config_data = 0x3;

		break;
	/* Scramble, 1/40 data rate */
	case VO_STANDARD_HDTV_2160P_23:
	case VO_STANDARD_HDTV_2160P_24:
	case VO_STANDARD_HDTV_2160P_25:
	case VO_STANDARD_HDTV_2160P_29:
	case VO_STANDARD_HDTV_2160P_30:
	case VO_STANDARD_HDTV_4096_2160P_24:
	case VO_STANDARD_HDTV_4096_2160P_25:
	case VO_STANDARD_HDTV_4096_2160P_30:
	case VO_STANDARD_HDTV_1080P_120:
		if (color == COLOR_YUV422)
			config_data = 0x0;
		else if (deep_color)
			config_data = 0x3;
		else
			config_data = 0x0;

		break;
	/* Depends on 340M_SCRAMBLE support */
	case VO_STANDARD_HDTV_2160P_50_420:
	case VO_STANDARD_HDTV_2160P_59_420:
	case VO_STANDARD_HDTV_2160P_60_420:
	case VO_STANDARD_HDTV_4096_2160P_50_420:
	case VO_STANDARD_HDTV_4096_2160P_60_420:
		if (deep_color)
			config_data = 0x3;
		else if (hdmitx_edid_info->scdc_capable&SCDC_340M_SCRAMBLE)
			config_data = 0x1;// Scramble, 1/10
		else
			config_data = 0x0;

		break;
	/* 3D frame packing, pixel freq*2 */
	case VO_STANDARD_HDTV_2160P_23_3D:
	case VO_STANDARD_HDTV_2160P_24_3D:
	case VO_STANDARD_HDTV_2160P_25_3D:
	case VO_STANDARD_HDTV_2160P_30_3D:
	case VO_STANDARD_HDTV_2160P_50_3D:
	case VO_STANDARD_HDTV_2160P_60_3D:
	case VO_STANDARD_HDTV_4096_2160P_24_3D:
	case VO_STANDARD_HDTV_4096_2160P_25_3D:
	case VO_STANDARD_HDTV_4096_2160P_30_3D:
	case VO_STANDARD_HDTV_4096_2160P_50_3D:
	case VO_STANDARD_HDTV_4096_2160P_60_3D:
		switch (format_3D) {
		case 0:/* Auto(Frame packing) */
		case 1:/* Frame packing */
			if ((standard == VO_STANDARD_HDTV_2160P_50_3D) ||
				(standard == VO_STANDARD_HDTV_2160P_60_3D) ||
				(standard == VO_STANDARD_HDTV_4096_2160P_50_3D) ||
				(standard == VO_STANDARD_HDTV_4096_2160P_60_3D))
				HDMI_ERROR("4KP50/60 NOT support 3D Frame packing");

			format_3D = 1;
			config_data = 0x3;
			break;
		case 2:/* Side by side half */
		case 3:/* Top and buttom */
			if (deep_color)
				config_data = 0x3;
			else
				config_data = 0x0;
			break;
		default:
			config_data = 0x0;
			pr_err("Unknow 3D format");
		} /* end of switch (format_3D) */
		break;
	/* No Scramble, 1/10 data rate */
	default:
		config_data = 0x0;
	} /* end of switch (standard)*/

	if (hdmitx_edid_info->scdc_capable&SCDC_PRESENT) {
		hdmitx_write_scdc_port(SCDCS_TMDS_Config, config_data);
		pr_info("Send SCDC TMDS_Config(0x%02x), deep_color(%u) 3D(%u)",
			config_data, deep_color, format_3D);
	} else if (config_data != 0) {
		pr_err("Sink not support SCDC_PRESENT, but need scramble");
		ret_val = hdmitx_write_scdc_port(SCDCS_TMDS_Config, config_data);
		if (ret_val == SCDC_I2C_OK)
			hdmitx_edid_info->scdc_capable |= SCDC_PRESENT;
		pr_info("Send SCDC TMDS_Config(0x%02x), deep_color(%u) 3D(%u)",
			config_data, deep_color, format_3D);
	} else if ((standard >= VO_STANDARD_HDTV_2160P_60_420) &&
		(standard <= VO_STANDARD_HDTV_4096_2160P_50_420)) {
		/* 4K YUV420 */
		pr_err("Sink not support SCDC_PRESENT, but YUV420 format should need");
		ret_val = hdmitx_write_scdc_port(SCDCS_TMDS_Config, config_data);
		if (ret_val == SCDC_I2C_OK)
			hdmitx_edid_info->scdc_capable |= SCDC_PRESENT;
		pr_info("Send SCDC TMDS_Config(0x%02x), deep_color(%u) 3D(%u)",
			config_data, deep_color, format_3D);
	}

	return config_data;
}

void hdmitx_scdc_work_func(struct work_struct *work)
{
	unsigned char update_flags[2], status_flags[2], err_det[7];
	unsigned int i;
	unsigned int sum = 0;

	if (hdmitx_read_scdc_port(SCDCS_Update_0, update_flags, 2))
		return;

	pr_devel("SCDC: Update(0x%02x)", update_flags[0]);

	/* Status_Update */
	if (update_flags[0]&0x1) {
		hdmitx_read_scdc_port(SCDCS_Status_Flag_0, status_flags, 2);
		pr_devel("SCDC: Status(0x%02x)", status_flags[0]);
	}

	/* CED_Update */
	if (update_flags[0]&0x2) {
		hdmitx_read_scdc_port(SCDCS_Err_Det_0_L, err_det, 7);

		for (i = 0; i < 6; i++)
			sum += err_det[i];

		sum = (0x100-(sum&0xFF))&0xFF;

		if (sum != err_det[6])
			pr_err("Wrong CED checksum");
		else
			pr_devel("SCDC: CED checksum ok");
	}

	/* Write clear */
	hdmitx_write_scdc_port(SCDCS_Update_0, update_flags[0]);
}


static irqreturn_t hdmitx_scdc_isr(int irq, void *dev_id)
{
	struct hdmitx_scdc_data *tx_scdc = (struct hdmitx_scdc_data *)dev_id;
	pr_devel("Get SCDC read request");
	schedule_work(&tx_scdc->rr_work);
	return IRQ_HANDLED;
}

void enable_hdmitx_scdcrr(hdmitx_device_t *tx_dev, unsigned char enable)
{
	struct hdmitx_scdc_data *tx_scdc = tx_dev->tx_scdc;

	if (!tx_scdc->enable_rr)
		return;

	pr_info("%s SCDC read request", enable?"Enable":"Disable");

	if ((!tx_scdc->iso_reg) || (!tx_scdc->dev_nd)) {
		pr_err("[%s] No control reg or no device node\n", __func__);
		return;
	}

	if (enable) {
		regmap_write(tx_scdc->iso_reg, I2C1_REQ_CTRL, 0x3);
		hdmitx_write_scdc_port(SCDCS_Config_0, 0x1);/* Send RR_Enable */
	} else {
		regmap_write(tx_scdc->iso_reg, I2C1_REQ_CTRL, 0x0);
	}
}

int register_hdmitx_scdcrr(struct device *dev)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);
	struct hdmitx_scdc_data *tx_scdc;
	int ret_value;
	struct device_node *syscon_np;

	dev_info(dev, "[%s]", __func__);

	tx_scdc = devm_kzalloc(dev, sizeof(struct hdmitx_scdc_data), GFP_KERNEL);
	tx_dev->tx_scdc = tx_scdc;

	syscon_np = of_parse_phandle(dev->of_node, "syscon", 1);
	if (IS_ERR_OR_NULL(syscon_np)) {
		dev_err(dev, "Parse syscon phandle 1 fail");
		return -EFAULT;
	}

	tx_scdc->iso_reg = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(tx_scdc->iso_reg)) {
		dev_err(dev, "Remap syscon 1 to iso_reg fail");
		of_node_put(syscon_np);
		return -EFAULT;
	}

	/* Default disable SCDC read request function for CTS 8-9 */
	regmap_write(tx_scdc->iso_reg, I2C1_REQ_CTRL, 0x0);

	/* Get SCDC read request device node */
	tx_scdc->dev_nd = of_get_child_by_name(dev->of_node, "scdc_rr");
	if (!tx_scdc->dev_nd) {
		dev_err(dev, "[%s] Get SCDC read request node fail!\n", __func__);
		return 0;
	}

	ret_value = of_property_read_u32(tx_scdc->dev_nd, "enable-scdc-rr",
		&tx_scdc->enable_rr);
	if (ret_value)
		tx_scdc->enable_rr = 0;

	if (!tx_scdc->enable_rr) {
		dev_info(dev, "Skip SCDC RR init");
		return 0;
	}

	/* Init interrupt work function */
	INIT_WORK(&tx_scdc->rr_work, hdmitx_scdc_work_func);

	/* Register read request interrupt */
	tx_scdc->i2c1_irq = irq_of_parse_and_map(tx_scdc->dev_nd, 0);
	ret_value = request_irq(tx_scdc->i2c1_irq, hdmitx_scdc_isr,
		IRQF_SHARED, "hdmitx_scdc", tx_scdc);
	if (ret_value) {
		dev_err(dev, "[%s] Request irq fail\n", __func__);
		return -EFAULT;
	}
	return 0;
}

void hdmitx_scdcrr_suspend(hdmitx_device_t *tx_dev)
{
	struct hdmitx_scdc_data *tx_scdc = tx_dev->tx_scdc;

	if (!tx_scdc->enable_rr)
		return;

	enable_hdmitx_scdcrr(tx_dev, 0);
	free_irq(tx_scdc->i2c1_irq, tx_scdc->dev_nd);
	cancel_work_sync(&tx_scdc->rr_work);
}

void hdmitx_scdcrr_resume(hdmitx_device_t *tx_dev)
{
	struct hdmitx_scdc_data *tx_scdc = tx_dev->tx_scdc;
	int ret_value;

	if (!tx_scdc->enable_rr)
		return;

	/* Register read request interrupt */
	ret_value = request_irq(tx_scdc->i2c1_irq, hdmitx_scdc_isr,
		IRQF_SHARED, "hdmitx_scdc", tx_scdc->dev_nd);
	if (ret_value)
		pr_err("[%s] Request irq fail\n", __func__);

}

