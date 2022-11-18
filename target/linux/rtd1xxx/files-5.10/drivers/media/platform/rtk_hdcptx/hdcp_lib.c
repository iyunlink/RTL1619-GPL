/*
 * hdcp_lib.c - RTK hdcp tx driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "hdcp_top.h"
#include "hdcp_lib_hw.h"
#include "hdcp.h"
#include "hdcp_ddc.h"

#include "hdmitx_reg.h"

static int hdcp_lib_read_aksv(uint8_t *ksv_data);
static int hdcp_lib_initiate_step1(void);
static int hdcp_lib_check_ksv(uint8_t ksv[5]);

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_read_aksv
 *-----------------------------------------------------------------------------
 */
static int hdcp_lib_read_aksv(uint8_t *ksv_data)
{
	if (hdcp.hdcp_enabled == 0)
		return -HDCP_CANCELLED_AUTH;

	*(ksv_data+0) = hdcp.en_ctrl->Aksv[0];
	*(ksv_data+1) = hdcp.en_ctrl->Aksv[1];
	*(ksv_data+2) = hdcp.en_ctrl->Aksv[2];
	*(ksv_data+3) = hdcp.en_ctrl->Aksv[3];
	*(ksv_data+4) = hdcp.en_ctrl->Aksv[4];

	return HDCP_OK;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_check_ksv
 *-----------------------------------------------------------------------------
 */
static int hdcp_lib_check_ksv(uint8_t ksv[5])
{
	int i;
	int j;
	int zero;
	int one;

	zero = 0;
	one = 0;
	for (i = 0; i < 5; i++) {
		/* Count number of zero / one */
		for (j = 0; j < 8; j++) {
			if (ksv[i] & (0x01 << j))
				one++;
			else
				zero++;
		}
	}

	if (one == zero)
		return HDCP_OK;
	else
		return -HDCP_AKSV_ERROR;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_initiate_step1
 *-----------------------------------------------------------------------------
 */
static int hdcp_lib_initiate_step1(void)
{
	/*
	 * HDCP authentication steps:
	 *   1) Read Bksv - check validity (is HDMI Rx supporting HDCP ?)
	 *   2) Initializes HDCP (CP reset release)
	 *   3) Read Bcaps - is HDMI Rx a repeater ?
	 *   *** First part authentication ***
	 *   4) Read Bksv - check validity (is HDMI Rx supporting HDCP ?)
	 *   5) Generates An
	 *   6) DDC: Writes An, Aksv
	 *   7) DDC: Write Bksv
	 */
	int status = HDCP_OK;
	uint8_t an_ksv_data[5];
	uint8_t an_bksv_data[5];
	uint8_t rx_type;
	uint8_t genAn[8];

	/* HDCP_DEBUG("[%s] %s  %d :%u\n", __FILE__, __func__, __LINE__, jiffies_to_msecs(jiffies)); */
	/* Generate An */
	status = hdcp_lib_generate_an(genAn);

	if (status != HDCP_OK)
		return status;

	HDCP_DEBUG("AN: %x %x %x %x %x %x %x %x",
		genAn[0], genAn[1], genAn[2], genAn[3],
		genAn[4], genAn[5], genAn[6], genAn[7]);

	status = hdcp_lib_read_aksv(an_ksv_data);
	if (status != HDCP_OK)
		return status;

	HDCP_INFO("AKSV: %02x %02x %02x %02x %02x",
		an_ksv_data[0], an_ksv_data[1], an_ksv_data[2],
		an_ksv_data[3], an_ksv_data[4]);

	status = hdcp_lib_check_ksv(an_ksv_data);
	if (status != HDCP_OK) {
		HDCP_ERROR("AKSV error (number of,0 and 1)\n");
		HDCP_ERROR("AKSV: %02x %02x %02x %02x %02x",
			an_ksv_data[0], an_ksv_data[1], an_ksv_data[2],
			an_ksv_data[3], an_ksv_data[4]);
		return status;
	}
	memcpy(ksvlist_info.Aksv, an_ksv_data, sizeof(an_ksv_data));

	/* DDC: Write An */
	if (ddc_write(DDC_AN_LEN, DDC_AN_ADDR, genAn))
		return -DDC_ERROR;

	/* DDC: Write AKSV */
	if (ddc_write(DDC_AKSV_LEN, DDC_AKSV_ADDR, an_ksv_data))
		return -DDC_ERROR;

	/* Read BCAPS to determine if HDCP RX is a repeater */
	if (ddc_read(DDC_BCAPS_LEN, DDC_BCAPS_ADDR, &rx_type))
		return -DDC_ERROR;

	rx_type = FLD_GET(rx_type, DDC_BIT_REPEATER, DDC_BIT_REPEATER);

	/* Set repeater bit in HDCP CTRL */
	if (rx_type == 1) {

		hdcp_lib_set_repeater_bit_in_tx(HDCP_REPEATER);

		HDCP_DEBUG("HDCP RX is a repeater");
	} else {

		hdcp_lib_set_repeater_bit_in_tx(HDCP_RECEIVER);

		HDCP_DEBUG("HDCP RX is a receiver");
	}

	/* DDC: Read BKSV from RX */
	if (ddc_read(DDC_BKSV_LEN, DDC_BKSV_ADDR, an_bksv_data))
		return -DDC_ERROR;

	if (hdcp_lib_check_ksv(an_bksv_data)) {
		HDCP_ERROR("Invalid BKSV: %02x %02x %02x %02x %02x",
			an_bksv_data[0], an_bksv_data[1], an_bksv_data[2],
			an_bksv_data[3], an_bksv_data[4]);
		return -HDCP_BKSV_ERROR;
	}

	memcpy(ksvlist_info.Bksv, an_bksv_data, sizeof(an_bksv_data));

	HDCP_DEBUG("BKSV: %02x %02x %02x %02x %02x",
		an_bksv_data[0], an_bksv_data[1], an_bksv_data[2],
		an_bksv_data[3], an_bksv_data[4]);

	HDCP_DEBUG("PK: %02x %02x %02x %02x %02x",
		hdcp.en_ctrl->PK[0], hdcp.en_ctrl->PK[1], hdcp.en_ctrl->PK[2],
		hdcp.en_ctrl->PK[3], hdcp.en_ctrl->PK[4]);

	/* Authentication 1st step initiated HERE */
	status = hdcp_lib_write_bksv(an_bksv_data);

	return status;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_step1_start
 *-----------------------------------------------------------------------------
 */
int hdcp_lib_step1_start(void)
{
	int status;

	status = hdcp_lib_initiate_step1();

	return status;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_r0_check
 *----------------------------------------------------------------------------
 */
int hdcp_lib_r0_check(void)
{
	uint8_t r0_rx[2] = {0, 0};
	int result;

	/* DDC: Read Ri' from RX */
	if (ddc_read(DDC_Ri_LEN, DDC_Ri_ADDR, r0_rx))
		return -DDC_ERROR;

	result = hdcp_lib_check_r0(r0_rx);

	if (result != HDCP_OK)
		return -HDCP_R0_ERROR;

	return HDCP_OK;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_auto_ri_check
 *-----------------------------------------------------------------------------
 */
void hdcp_lib_auto_ri_check(bool state)
{
	void *irq_dev_id = (void *)&hdcp.hdcp_irq_num;

	if (state == true) {

		hdcp_lib_set_wider_window();

		if (request_irq(hdcp.hdcp_irq_num, HDCP_interrupt_handler, IRQF_SHARED, "rtk_hdcp", irq_dev_id)) {
			HDCP_ERROR("request HDCP irq failed");
		} else {
			HDCP_DEBUG("request HDCP irq %d", hdcp.hdcp_irq_num);

			hdcp_lib_set_ri_interrupt(HDCP_RI_ON);
		}
	} else {
		hdcp_lib_set_ri_interrupt(HDCP_RI_OFF);
	}

	HDCP_DEBUG("[%s] %s  %d :%u ,state=%s\n", __FILE__, __func__, __LINE__,
		jiffies_to_msecs(jiffies), state == true ? "ON" : "OFF");
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_polling_bcaps_rdy_check
 *-----------------------------------------------------------------------------
 */
int hdcp_lib_polling_bcaps_rdy_check(void)
{
	uint8_t ready_bit;
	int i;
	int ret_val;

	ret_val = HDCP_OK;
	/*
	 * State A8:Wait for Ready.
	 * sets up 5 secs timer and polls the Receiver's READY bit.
	 */
	for (i = 0; i < 50; i++) {
		ready_bit = 0;
		if (ddc_read(DDC_BCAPS_LEN, DDC_BCAPS_ADDR, &ready_bit)) {
			ret_val = -HDCP_DDC_ERROR;
			HDCP_ERROR("I2c error, read Bcaps failed");
		}

		if (FLD_GET(ready_bit, DDC_BIT_READY, DDC_BIT_READY)) {
			HDCP_DEBUG("Bcaps ready bit asserted");
			goto exit;
		} else {
			usleep_range(100000, 110000);
		}
	}

	/* Timeout occurred */
	ret_val = -HDCP_BCAPS_RDY_ERROR;

exit:
	return ret_val;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_step1_r0_check
 *-----------------------------------------------------------------------------
 */
int hdcp_lib_step1_r0_check(void)
{
	int status;
	uint8_t rx_type;

	/*
	 * HDCP authentication steps:
	 * 1) DDC: Read M0'
	 * 2) Compare M0 and M0'
	 * if Rx is a receiver: switch to authentication step 3
	 * 3) Enable encryption / auto Ri check / disable AV mute
	 * if Rx is a repeater: switch to authentication step 2
	 * 3) Get M0 from HDMI IP and store it for further processing (V)
	 * 4) Enable encryption / auto Ri check / auto BCAPS RDY polling
	 *     Disable AV mute
	 */

	/* HDCP_DEBUG("[%s] %s  %d :%u\n", __FILE__,__FUNCTION__,__LINE__,jiffies_to_msecs(jiffies)); */

	status = hdcp_lib_r0_check();
	if (status < 0)
		return status;

	/* Authentication 1st step done */
	HDCP_INFO("R0 check success");

	/*
	 * Now prepare 2nd step authentication in case of RX repeater and
	 * enable encryption / Ri check
	 */

	/* Read BCAPS to determine if HDCP RX is a repeater */
	if (ddc_read(DDC_BCAPS_LEN, DDC_BCAPS_ADDR, &rx_type))
		return -DDC_ERROR;

	rx_type = FLD_GET(rx_type, DDC_BIT_REPEATER, DDC_BIT_REPEATER);

	if (rx_type == 0) {
		/* Receiver: enable encryption and auto Ri check */
		hdcp_lib_set_encryption(HDCP_ENC_ON);

		hdcp_lib_auto_ri_check(true);
	}

	return status;
}


/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_step2
 *-----------------------------------------------------------------------------
 */
int hdcp_lib_step2(void)
{
	/*
	 * HDCP authentication steps:
	 *   1) Disable auto Ri check
	 *   2) DDC: read BStatus (nb of devices, MAX_DEV
	 */
	uint8_t bstatus[2] = {0};
	int status = HDCP_OK;
	int i;

	HDCP_DEBUG("[%s] %s  %d :%u", __FILE__, __func__, __LINE__, jiffies_to_msecs(jiffies));

	status = hdcp_lib_polling_bcaps_rdy_check();
	if (status != HDCP_OK) {
		HDCP_ERROR("polling Bcaps ready failed\n");
		return status;
	}

	/* DDC: Read Bstatus (1st byte) from Rx */
	if (ddc_read(DDC_BSTATUS_LEN, DDC_BSTATUS_ADDR, bstatus))
		return -DDC_ERROR;

	HDCP_DEBUG("bstatus=0x%x,0x%x", bstatus[0], bstatus[1]);

	/* Check BStatus topology errors */
	if (bstatus[0] & DDC_BSTATUS0_MAX_DEVS) {
		HDCP_ERROR("MAX_DEV_EXCEEDED set");
		return -HDCP_AUTH_FAILURE;
	}

	if (bstatus[1] & DDC_BSTATUS1_MAX_CASC) {
		HDCP_ERROR("MAX_CASCADE_EXCEEDED set");
		return -HDCP_AUTH_FAILURE;
	}

	HDCP_DEBUG("Retrieving KSV list...");
	/* Get KSV list size */
	sha_input.byte_counter = (bstatus[0] & DDC_BSTATUS0_DEV_COUNT) * 5;

	/* Clear all SHA input data */
	/* TODO: should be done earlier at HDCP init */
	memset(sha_input.data, 0, MAX_SHA_DATA_SIZE);

	/* DDC: read KSV list */
	if (sha_input.byte_counter) {
		if (ddc_read(sha_input.byte_counter, DDC_KSV_FIFO_ADDR, (uint8_t *)&sha_input.data)) {
			return -DDC_ERROR;
		} else {
			ksvlist_info.device_count = (bstatus[0] & DDC_BSTATUS0_DEV_COUNT);
			memcpy(ksvlist_info.ksvlist, sha_input.data, sizeof(ksvlist_info.ksvlist));
			memcpy(ksvlist_info.bstatus, bstatus, sizeof(ksvlist_info.bstatus));
		}
	}

	/* Read and add Bstatus */
	hdcp_lib_SHA_append_bstatus_M0(&sha_input, bstatus);

	/* Read V' */
	if (ddc_read(DDC_V_LEN, DDC_V_ADDR, sha_input.vprime))
		return -DDC_ERROR;

	for (i = 0; i < 5; i++) {
		HDCP_DEBUG("sha_input.vprime[%d]=%x,%x,%x,%x\n", 4*i,
			sha_input.vprime[4*i+0], sha_input.vprime[4*i+1],
			sha_input.vprime[4*i+2], sha_input.vprime[4*i+3]);
	}
	/* hdcp_lib_dump_sha(&sha_input); */

	status = hdcp_lib_compute_V(&sha_input);
	if (status != HDCP_OK)
		goto sha_err;

	status = hdcp_lib_verify_V(&sha_input);

	if (status == HDCP_OK) {
		/* Re-enable Ri check */
		/* Receiver: enable encryption and auto Ri check */
		hdcp_lib_set_encryption(HDCP_ENC_ON);

		hdcp_lib_auto_ri_check(true);
	} else {
		status = -HDCP_SHA1_ERROR;
	}

sha_err:
	return status;
}

/**
 * hdcp_lib_query_sink_hdcp_capable - query hdcp version
 *
 * Return: enum sink_hdcp_capable
 */
int hdcp_lib_query_sink_hdcp_capable(void)
{
	int err_no;
	int ret_val;
	uint8_t hdcp2ver;
	uint8_t bcaps;
	uint8_t bstatus[2];

	ret_val = HDCP_INCAPABLE;

	err_no = ddc_read(DDC_HDCP2Ver_LEN, DDC_HDCP2Ver_ADDR, &hdcp2ver);
	if (!IS_ERR(err_no)) {
		if (hdcp2ver & BIT(2)) {
			ret_val = HDCP2x_CAPABLE;
			goto exit;
		}
	}

	err_no = ddc_read(DDC_BCAPS_LEN, DDC_BCAPS_ADDR, &bcaps);
	if (IS_ERR(err_no)) {
		hdcp.err_code = HDCP_DDC_ERROR;
		goto exit;
	}

	err_no = ddc_read(DDC_BSTATUS_LEN, DDC_BSTATUS_ADDR, bstatus);
	if (IS_ERR(err_no)) {
		hdcp.err_code = HDCP_DDC_ERROR;
		goto exit;
	}

	ret_val = HDCP1x_CAPABLE;

exit:
	return ret_val;
}

int hdcp_lib_get_info(unsigned char *buf)
{
	int ret_count = 0;
	unsigned char ksv[5] = {0};
	unsigned char valid;
	unsigned char hdcp2ver = 0;
	unsigned char hdcp_ver = 0;
	unsigned char bcaps;
	unsigned char bstatus[2] = {0};

	if (ddc_read(DDC_BCAPS_LEN, DDC_BCAPS_ADDR, &bcaps)) {
		HDCP_ERROR("Read Bcaps fail");
		hdcp_ver = 0;
	} else {
		hdcp_ver = 1;
	}

	if (ddc_read(DDC_HDCP2Ver_LEN, DDC_HDCP2Ver_ADDR, &hdcp2ver))
		HDCP_ERROR("Read HDCP2Version fail");

	if (hdcp2ver & BIT(2))
		hdcp_ver = 2;

	if (hdcp_ver == 0) {
		ret_count += sprintf(buf+ret_count,
			"hdcp_version: Unsupported\n");
		goto exit;
	}

	ret_count += sprintf(buf+ret_count, "hdcp_version: %s\n",
		(hdcp_ver == 2) ? "2.x":"1.4");

	ret_count += sprintf(buf+ret_count, "sink_type: %s\n",
		(bcaps & BIT(6)) ? "Repeater":"Receiver");

	ret_count += sprintf(buf+ret_count, "FAST(400KHz): %s\n",
		(bcaps & BIT(4)) ? "Yes":"No");

	ret_count += sprintf(buf+ret_count, "1.1_FEATURES: %s\n",
		(bcaps & BIT(1)) ? "Yes":"No");

	if (ddc_read(DDC_BKSV_LEN, DDC_BKSV_ADDR, ksv))
		HDCP_ERROR("Read BKSV fail");

	valid = !hdcp_lib_check_ksv(ksv);

	ret_count += sprintf(buf+ret_count,
			"BKSV: %02x%02x%02x%02x%02x %s\n",
			ksv[0], ksv[1], ksv[2], ksv[3], ksv[4],
			valid ? "Valid":"Invalid");

	if (ddc_read(DDC_BSTATUS_LEN, DDC_BSTATUS_ADDR, bstatus))
		HDCP_ERROR("Read Bstatus fail");

	ret_count += sprintf(buf+ret_count, "MAX_CASCADE_EXCEEDED: %u\n",
		(bstatus[1] & DDC_BSTATUS1_MAX_CASC) ? 1:0);

	ret_count += sprintf(buf+ret_count, "DEPTH: %u\n",
		bstatus[1] & DDC_BSTATUS1_DEPTH);

	ret_count += sprintf(buf+ret_count, "MAX_DEVS_EXCEEDED: %u\n",
		(bstatus[0] & DDC_BSTATUS0_MAX_DEVS) ? 1:0);

	ret_count += sprintf(buf+ret_count, "DEVICE_COUNT: %u\n",
		bstatus[0] & DDC_BSTATUS0_DEV_COUNT);

exit:
	return ret_count;
}
