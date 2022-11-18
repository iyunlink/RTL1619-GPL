/*
 * RealTek HDCP 1.4 and HDCP 2.2 driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/component.h>
#include <linux/i2c.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <drm/drm_print.h>
#include <linux/gpio/consumer.h>

#include "rtk_hdmi.h"

#define HDCP2_LC_RETRY_CNT	1024

static int rtk_hdmi_hdcp_read(struct rtk_hdmi *hdmi,
				unsigned int offset, void *buffer, size_t size)
{
	struct i2c_adapter *adapter = hdmi->ddc;
	int ret;
	u8 start = offset & 0xff;
	int hpd;

	struct i2c_msg msgs[] = {
		{
			.addr = DRM_HDCP_DDC_ADDR,
			.flags = 0,
			.len = 1,
			.buf = &start,
		},
		{
			.addr = DRM_HDCP_DDC_ADDR,
			.flags = I2C_M_RD,
			.len = size,
			.buf = buffer
		}
	};
	ret = i2c_transfer(adapter, msgs, ARRAY_SIZE(msgs));
	if (ret == ARRAY_SIZE(msgs))
		return 0;

	hpd = gpiod_get_value(hdmi->hpd_gpio);
	if(!hpd)
		return -HDCP_PLUGOUT_EVENT;

	return ret >= 0 ? -EIO : ret;
}

static int rtk_hdmi_hdcp_write(struct rtk_hdmi *hdmi,
				unsigned int offset, void *buffer, size_t size)
{
	struct i2c_adapter *adapter = hdmi->ddc;
	int ret;
	u8 *write_buf;
	struct i2c_msg msg;
	int hpd;

	write_buf = kzalloc(size + 1, GFP_KERNEL);
	if (!write_buf)
		return -ENOMEM;

	write_buf[0] = offset & 0xff;
	memcpy(&write_buf[1], buffer, size);

	msg.addr = DRM_HDCP_DDC_ADDR;
	msg.flags = 0,
	msg.len = size + 1,
	msg.buf = write_buf;

	ret = i2c_transfer(adapter, &msg, 1);
	if (ret == 1)
		ret = 0;
	else if (ret >= 0)
		ret = -EIO;

	hpd = gpiod_get_value(hdmi->hpd_gpio);
	if(!hpd)
		ret = -HDCP_PLUGOUT_EVENT;

	kfree(write_buf);
	return ret;
}

static
int rtk_hdmi_hdcp_write_an_aksv(struct rtk_hdmi *hdmi,
				u8 *an, u8 *aksv)
{
	int ret;

	ret = rtk_hdmi_hdcp_write(hdmi, DRM_HDCP_DDC_AN, an,
					DRM_HDCP_AN_LEN);
	if (ret) {
		DRM_DEBUG_KMS("Write An over DDC failed (%d)\n", ret);
		return ret;
	}

	ret = rtk_hdmi_hdcp_write(hdmi, DRM_HDCP_DDC_AKSV,
				  aksv, DRM_HDCP_KSV_LEN);
	if (ret) {
		DRM_DEBUG_KMS("Write Aksv over DDC failed (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int rtk_hdmi_hdcp_read_bksv(struct rtk_hdmi *hdmi, u8 *bksv)
{
	int ret;

	ret = rtk_hdmi_hdcp_read(hdmi, DRM_HDCP_DDC_BKSV, bksv,
					DRM_HDCP_KSV_LEN);
	if (ret)
		DRM_DEBUG_KMS("Read Bksv over DDC failed (%d)\n", ret);
	return ret;
}

static
int rtk_hdmi_hdcp_read_bstatus(struct rtk_hdmi *hdmi, u8 *bstatus)
{
	int ret;

	ret = rtk_hdmi_hdcp_read(hdmi, DRM_HDCP_DDC_BSTATUS,
				bstatus, DRM_HDCP_BSTATUS_LEN);
	if (ret)
		DRM_DEBUG_KMS("Read bstatus over DDC failed (%d)\n", ret);
	return ret;
}

static
int rtk_hdmi_hdcp_repeater_present(struct rtk_hdmi *hdmi, bool *repeater_present)
{
	int ret;
	u8 val;

	ret = rtk_hdmi_hdcp_read(hdmi, DRM_HDCP_DDC_BCAPS, &val, 1);
	if (ret) {
		DRM_DEBUG_KMS("Read bcaps over DDC failed (%d)\n", ret);
		return ret;
	}
	*repeater_present = val & DRM_HDCP_DDC_BCAPS_REPEATER_PRESENT;
	return 0;
}

static
int rtk_hdmi_hdcp_read_ri_prime(struct rtk_hdmi *hdmi, u8 *ri_prime)
{
	int ret;

	ret = rtk_hdmi_hdcp_read(hdmi, DRM_HDCP_DDC_RI_PRIME,
					ri_prime, DRM_HDCP_RI_LEN);
	if (ret)
		DRM_DEBUG_KMS("Read Ri' over DDC failed (%d)\n", ret);
	return ret;
}

static
int rtk_hdmi_hdcp_read_ksv_ready(struct rtk_hdmi *hdmi, bool *ksv_ready)
{
	int ret;
	u8 val;

	ret = rtk_hdmi_hdcp_read(hdmi, DRM_HDCP_DDC_BCAPS, &val, 1);
	if (ret) {
		DRM_DEBUG_KMS("Read bcaps over DDC failed (%d)\n", ret);
		return ret;
	}
	*ksv_ready = val & DRM_HDCP_DDC_BCAPS_KSV_FIFO_READY;
	return 0;
}

static
int rtk_hdmi_hdcp_read_ksv_fifo(struct rtk_hdmi *hdmi,
				int num_downstream, u8 *ksv_fifo)
{
	int ret;

	ret = rtk_hdmi_hdcp_read(hdmi, DRM_HDCP_DDC_KSV_FIFO,
				ksv_fifo, num_downstream * DRM_HDCP_KSV_LEN);
	if (ret) {
		DRM_DEBUG_KMS("Read ksv fifo over DDC failed (%d)\n", ret);
		return ret;
	}
	return 0;
}

static
int rtk_hdmi_hdcp_read_v_prime_part(struct rtk_hdmi *hdmi, int i, u32 *part)
{
	int ret;

	if (i >= DRM_HDCP_V_PRIME_NUM_PARTS)
		return -EINVAL;

	ret = rtk_hdmi_hdcp_read(hdmi, DRM_HDCP_DDC_V_PRIME(i),
				part, DRM_HDCP_V_PRIME_PART_LEN);
	if (ret)
		DRM_DEBUG_KMS("Read V'[%d] over DDC failed (%d)\n", i, ret);
	return ret;
}

static int
rtk_hdcp_check_ri_prime(struct rtk_hdcp *hdcp, u8 *ri_prime)
{
	struct rtk_hdcp1_tee *hdcp1_tee = &hdcp->hdcp1_tee;
	int ret = HDCP1_R0_ERROR;

	if(hdcp1_tee->hdcp1_tee_ops &&
	   hdcp1_tee->hdcp1_tee_ops->check_ri_prime)
		ret = hdcp1_tee->hdcp1_tee_ops->check_ri_prime(hdcp1_tee,
							       ri_prime);
	return ret;
}

static
int rtk_hdmi_hdcp_check_link(struct rtk_hdmi *hdmi)
{
	int ret = 0;
	u8 ri[DRM_HDCP_RI_LEN];
	struct rtk_hdcp *hdcp = &hdmi->hdcp;

	ret = rtk_hdmi_hdcp_read_ri_prime(hdmi, ri);
	if (ret)
		return ret;

	ret = rtk_hdcp_check_ri_prime(hdcp, ri);

	return ret;
}

struct hdcp2_message_data {
	u8 msg_id;
	u32 timeout;
	u32 timeout2;
};

static const struct hdcp2_message_data hdcp2_msg_data[] = {
	{ HDCP_2_2_AKE_INIT, 0, 0 },
	{ HDCP_2_2_AKE_SEND_CERT, HDCP_2_2_CERT_TIMEOUT_MS, 0 },
	{ HDCP_2_2_AKE_NO_STORED_KM, 0, 0 },
	{ HDCP_2_2_AKE_STORED_KM, 0, 0 },
	{ HDCP_2_2_AKE_SEND_HPRIME, HDCP_2_2_HPRIME_PAIRED_TIMEOUT_MS,
		HDCP_2_2_HPRIME_NO_PAIRED_TIMEOUT_MS },
	{ HDCP_2_2_AKE_SEND_PAIRING_INFO, HDCP_2_2_PAIRING_TIMEOUT_MS, 0 },
	{ HDCP_2_2_LC_INIT, 0, 0 },
	{ HDCP_2_2_LC_SEND_LPRIME, HDCP_2_2_HDMI_LPRIME_TIMEOUT_MS, 0 },
	{ HDCP_2_2_SKE_SEND_EKS, 0, 0 },
	{ HDCP_2_2_REP_SEND_RECVID_LIST, HDCP_2_2_RECVID_LIST_TIMEOUT_MS, 0 },
	{ HDCP_2_2_REP_SEND_ACK, 0, 0 },
	{ HDCP_2_2_REP_STREAM_MANAGE, 0, 0 },
	{ HDCP_2_2_REP_STREAM_READY, HDCP_2_2_STREAM_READY_TIMEOUT_MS, 0 },
};

static
int rtk_hdmi_hdcp2_capable(struct rtk_hdmi *hdmi, bool *capable)
{
	u8 hdcp2_version;
	int ret;

	*capable = false;
	ret = rtk_hdmi_hdcp_read(hdmi, HDCP_2_2_HDMI_REG_VER_OFFSET,
				&hdcp2_version, sizeof(hdcp2_version));
	if (!ret && hdcp2_version & HDCP_2_2_HDMI_SUPPORT_MASK)
		*capable = true;

	return ret;
}

static
int rtk_hdmi_hdcp2_read_rx_status(struct rtk_hdmi *hdmi, u8 *rx_status)
{
	return rtk_hdmi_hdcp_read(hdmi,
			     HDCP_2_2_HDMI_REG_RXSTATUS_OFFSET,
			     rx_status,
			     HDCP_2_2_HDMI_RXSTATUS_LEN);
}

static inline
int hdcp2_detect_msg_availability(struct rtk_hdmi *hdmi,
				u8 msg_id, bool *msg_ready,
				ssize_t *msg_sz)
{
	u8 rx_status[HDCP_2_2_HDMI_RXSTATUS_LEN];
	int ret;

	ret = rtk_hdmi_hdcp2_read_rx_status(hdmi, rx_status);
	if (ret < 0) {
		DRM_DEBUG_KMS("rx_status read failed. Err %d\n", ret);
		return ret;
	}

	*msg_sz = ((HDCP_2_2_HDMI_RXSTATUS_MSG_SZ_HI(rx_status[1]) << 8) |
		rx_status[0]);

	if (msg_id == HDCP_2_2_REP_SEND_RECVID_LIST)
		*msg_ready = (HDCP_2_2_HDMI_RXSTATUS_READY(rx_status[1]) &&
				*msg_sz);
	else
		*msg_ready = *msg_sz;

	return 0;
}

static int get_hdcp2_msg_timeout(u8 msg_id, bool is_paired)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hdcp2_msg_data); i++) {
		if (hdcp2_msg_data[i].msg_id == msg_id &&
			(msg_id != HDCP_2_2_AKE_SEND_HPRIME || is_paired))
			return hdcp2_msg_data[i].timeout;
		else if (hdcp2_msg_data[i].msg_id == msg_id)
			return hdcp2_msg_data[i].timeout2;
	}

	return -EINVAL;
}

static ssize_t
rtk_hdmi_hdcp2_wait_for_msg(struct rtk_hdmi *hdmi,
			u8 msg_id, bool paired)
{
	bool msg_ready = false;
	int timeout, ret;
	ssize_t msg_sz = 0;

	timeout = get_hdcp2_msg_timeout(msg_id, paired);
	if (timeout < 0)
		return timeout;

	ret = __wait_for(ret = hdcp2_detect_msg_availability(hdmi,
							     msg_id, &msg_ready,
							     &msg_sz),
			ret == -HDCP_PLUGOUT_EVENT,
			!ret && msg_ready && msg_sz, timeout * 1000,
			1000, 5 * 1000);
	if (ret)
		DRM_DEBUG_KMS("msg_id: %d, ret: %d, timeout: %d, msg_ready: %d, msg_sz: %d\n",
			msg_id, ret, timeout, msg_ready, msg_sz);

	return ret ? ret : msg_sz;
}

static
int rtk_hdmi_hdcp2_write_msg(struct rtk_hdmi *hdmi,
				void *buf, size_t size)
{
	unsigned int offset;

	offset = HDCP_2_2_HDMI_REG_WR_MSG_OFFSET;
	return rtk_hdmi_hdcp_write(hdmi, offset, buf, size);
}

static
int rtk_hdmi_hdcp2_read_msg(struct rtk_hdmi *hdmi,
			u8 msg_id, void *buf, size_t size)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	unsigned int offset;
	ssize_t ret;

	ret = rtk_hdmi_hdcp2_wait_for_msg(hdmi, msg_id,
					  hdcp->is_paired);
	if (ret < 0)
		return ret;

	/*
	 * Available msg size should be equal to or lesser than the
	 * available buffer.
	 */
	if (ret > size) {
		DRM_DEBUG_KMS("msg_sz(%zd) is more than exp size(%zu)\n",
				ret, size);
		return -1;
	}

	offset = HDCP_2_2_HDMI_REG_RD_MSG_OFFSET;
	ret = rtk_hdmi_hdcp_read(hdmi, offset, buf, ret);
	if (ret)
		DRM_DEBUG_KMS("Failed to read msg_id: %d(%zd)\n", msg_id, ret);

	return ret;
}

static
int rtk_hdmi_hdcp2_check_link(struct rtk_hdmi *hdmi)
{
	u8 rx_status[HDCP_2_2_HDMI_RXSTATUS_LEN];
	int ret;

	ret = rtk_hdmi_hdcp2_read_rx_status(hdmi, rx_status);
	if (ret)
		return ret;

	/*
	 * Re-auth request and Link Integrity Failures are represented by
	 * same bit. i.e reauth_req.
	 */
	if (HDCP_2_2_HDMI_RXSTATUS_REAUTH_REQ(rx_status[1]))
		ret = HDCP_REAUTH_REQUEST;
	else if (HDCP_2_2_HDMI_RXSTATUS_READY(rx_status[1]))
		ret = HDCP_TOPOLOGY_CHANGE;

	return ret;
}

static const struct rtk_hdcp_ops rtk_hdmi_hdcp_ops = {
	.write_an_aksv = rtk_hdmi_hdcp_write_an_aksv,
	.read_bksv = rtk_hdmi_hdcp_read_bksv,
	.read_bstatus = rtk_hdmi_hdcp_read_bstatus,
	.repeater_present = rtk_hdmi_hdcp_repeater_present,
	.read_ri_prime = rtk_hdmi_hdcp_read_ri_prime,
	.read_ksv_ready = rtk_hdmi_hdcp_read_ksv_ready,
	.read_ksv_fifo = rtk_hdmi_hdcp_read_ksv_fifo,
	.read_v_prime_part = rtk_hdmi_hdcp_read_v_prime_part,
	.check_link = rtk_hdmi_hdcp_check_link,
	.hdcp2_capable = rtk_hdmi_hdcp2_capable,
	.write_2_2_msg = rtk_hdmi_hdcp2_write_msg,
	.read_2_2_msg = rtk_hdmi_hdcp2_read_msg,
	.check_2_2_link = rtk_hdmi_hdcp2_check_link,
};

/* Is HDCP2.2 capable on Platform and Sink */
bool rtk_hdcp2_capable(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	bool capable = false;

	if(!hdcp->hdcp2_supported)
		return false;

	hdcp->hdcp_ops->hdcp2_capable(hdmi, &capable);

	return capable;
}

static
int rtk_hdcp_is_ksv_valid(u8 *ksv)
{
	int i, ones = 0;
	/* KSV has 20 1's and 20 0's */
	for (i = 0; i < DRM_HDCP_KSV_LEN; i++)
		ones += hweight8(ksv[i]);
	if (ones != 20)
		return HDCP1_AKSV_ERROR;

	return HDCP1_SUCCESS;
}

static
int rtk_hdcp_read_valid_bksv(struct rtk_hdmi *hdmi,
			     const struct rtk_hdcp_ops *hdcp_ops,
			     u8 *bksv)
{
	int ret = HDCP1_BKSV_ERROR, i, tries = 2;

	/* HDCP spec states that we must retry the bksv if it is invalid */
	for (i = 0; i < tries; i++) {
		ret = hdcp_ops->read_bksv(hdmi, bksv);
		if (ret)
			return ret;
		if (rtk_hdcp_is_ksv_valid(bksv) == HDCP1_SUCCESS)
			break;
	}
	if (i == tries) {
		DRM_DEBUG_KMS("Bksv is invalid\n");
		return HDCP1_BKSV_ERROR;
	}

	return HDCP1_SUCCESS;
}

/* Is HDCP1.4 capable on Platform and Sink */
int rtk_hdcp_capable(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	int ret;
	bool capable = false;
	u8 bksv[DRM_HDCP_KSV_LEN];

	if (!hdcp || !hdcp->hdcp_ops)
		return capable;

	if (hdcp->hdcp_ops->hdcp_capable) {
		hdcp->hdcp_ops->hdcp_capable(hdmi, &capable);
		ret = capable;
	} else {
		ret = rtk_hdcp_read_valid_bksv(hdmi, hdcp->hdcp_ops, bksv);
		if (ret == HDCP1_SUCCESS)
			ret = 1;
	}

	return ret;
}

static void
hdcp2_prepare_ake_init(struct rtk_hdcp *hdcp,
		       struct hdcp2_ake_init *ake_data)
{
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;

	ake_data->msg_id = HDCP_2_2_AKE_INIT;
	/*According to the HDCP 2.2 Spec.,
	  version must be 0x2 and tx_cap_mask read as 0*/
	ake_data->tx_caps.version = 0x2;
	ake_data->tx_caps.tx_cap_mask[0] = 0x0;
	ake_data->tx_caps.tx_cap_mask[1] = 0x0;
	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->generate_random_rtx)
		hdcp2_tee->hdcp2_tee_ops->generate_random_rtx(hdcp2_tee, ake_data);
}

/*We always use the no stored km data structure
  to hold the store_km and nostore_km data*/
static int
hdcp2_verify_rx_cert_prepare_km(struct rtk_hdcp *hdcp,
				struct hdcp2_ake_send_cert *rx_cert,
				bool *km_stored,
				struct hdcp2_ake_no_stored_km *ek_pub_km,
				size_t *msg_sz)
{
	int ret = 0;
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;
	u8 receiver_id[HDCP_2_2_RECEIVER_ID_LEN];

	if (!rx_cert || !km_stored || !ek_pub_km || !msg_sz)
		return -EINVAL;

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->verify_rx_cert)
		ret = hdcp2_tee->hdcp2_tee_ops->verify_rx_cert(hdcp2_tee, rx_cert);

	if(ret != HDCP2_SUCCESS) {
		DRM_DEBUG_KMS("Verify rx_cert failed. %d\n", ret);
		return ret;
	}

	memcpy(receiver_id, rx_cert->cert_rx.receiver_id,
	       HDCP_2_2_RECEIVER_ID_LEN);

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->check_stored_km)
		hdcp2_tee->hdcp2_tee_ops->check_stored_km(hdcp2_tee, receiver_id,
						    ek_pub_km, km_stored);

	else
		*km_stored = false;

	if(*km_stored) {
		DRM_DEBUG_KMS("[HDCP22] Stored Km.\n");
		ek_pub_km->msg_id = HDCP_2_2_AKE_STORED_KM;
		*msg_sz = sizeof(struct hdcp2_ake_stored_km);
		/*We do not need to setup stored km data because
		 that data has been already prepared in check_stored_km if success,
		 However, we add a callback function here*/
		if(hdcp2_tee->hdcp2_tee_ops &&
		   hdcp2_tee->hdcp2_tee_ops->prepare_stored_km)
			hdcp2_tee->hdcp2_tee_ops->prepare_stored_km(hdcp2_tee, ek_pub_km);
	}
	else {
		DRM_DEBUG_KMS("[HDCP22] non-Stored Km.\n");
		ek_pub_km->msg_id = HDCP_2_2_AKE_NO_STORED_KM;
		*msg_sz = sizeof(struct hdcp2_ake_no_stored_km);
		if(hdcp2_tee->hdcp2_tee_ops &&
		   hdcp2_tee->hdcp2_tee_ops->prepare_no_stored_km)
			hdcp2_tee->hdcp2_tee_ops->prepare_no_stored_km(hdcp2_tee, ek_pub_km);
	}

	return 0;
}

static int hdcp2_verify_hprime(struct rtk_hdcp *hdcp,
			       struct hdcp2_ake_send_hprime *rx_hprime)
{
	int ret = 0;
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;
	/*Sum of HDCP_2_2_RRX_LEN, HDCP_2_2_RTX_LEN,
	  HDCP_2_2_RXCAPS_LEN, and sizeof(struct hdcp2_tx_caps)*/
	u8 verified_src[22];
	u8 h[HDCP_2_2_H_PRIME_LEN];
	int offset=0;

	memcpy(verified_src, hdcp->r_rx, HDCP_2_2_RRX_LEN);
	offset += HDCP_2_2_RRX_LEN;
	memcpy(verified_src+offset, hdcp->r_tx, HDCP_2_2_RTX_LEN);
	offset += HDCP_2_2_RTX_LEN;
	memcpy(verified_src+offset, hdcp->rx_caps, HDCP_2_2_RXCAPS_LEN);
	offset += HDCP_2_2_RXCAPS_LEN;
	memcpy(verified_src+offset, &hdcp->tx_caps, sizeof(struct hdcp2_tx_caps));

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->verify_hprime)
		ret = hdcp2_tee->hdcp2_tee_ops->verify_hprime(hdcp2_tee, rx_hprime,
							verified_src, h);

	if (ret != HDCP2_SUCCESS)
		DRM_DEBUG_KMS("Verify hprime failed. %d\n", ret);

	return ret;
}

static int
hdcp2_store_pairing_info(struct rtk_hdcp *hdcp,
			 struct hdcp2_ake_send_pairing_info *pairing_info)
{
	int ret = HDCP2_READPAIRING_ERROR ;
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->store_pairing_info)
		ret = hdcp2_tee->hdcp2_tee_ops->store_pairing_info(hdcp2_tee, pairing_info);

	if (ret != HDCP2_SUCCESS)
		DRM_DEBUG_KMS("Store pairing info failed. %d\n", ret);

	return ret;
}

static void
hdcp2_prepare_lc_init(struct rtk_hdcp *hdcp,
		      struct hdcp2_lc_init *lc_init)
{
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;

	lc_init->msg_id = HDCP_2_2_LC_INIT;

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->initiate_locality_check)
		hdcp2_tee->hdcp2_tee_ops->initiate_locality_check(hdcp2_tee, lc_init);
}

static int
hdcp2_verify_lprime(struct rtk_hdcp *hdcp,
		    struct hdcp2_lc_send_lprime *rx_lprime)
{
	int ret = HDCP2_COMPARE_L_ERROR;
	u8 lprime[HDCP_2_2_L_PRIME_LEN];
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->verify_lprime)
		ret = hdcp2_tee->hdcp2_tee_ops->verify_lprime(hdcp2_tee, rx_lprime, lprime);

	if (ret != HDCP2_SUCCESS)
		DRM_DEBUG_KMS("Verify L_Prime failed. %d\n", ret);

	return ret;
}

static void hdcp2_prepare_skey(struct rtk_hdcp *hdcp,
			       struct hdcp2_ske_send_eks *ske_data)
{
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;

	ske_data->msg_id = HDCP_2_2_SKE_SEND_EKS;

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->get_session_key)
		hdcp2_tee->hdcp2_tee_ops->get_session_key(hdcp2_tee, ske_data);
}

static int
hdcp2_verify_rep_topology_prepare_ack(struct rtk_hdcp *hdcp,
				      u8 *buf, int msg_size,
				      u8 *mV)
{
	int ret = HDCP2_COMPARE_V_ERROR;
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->repeater_check_flow_prepare_ack)
		ret = hdcp2_tee->hdcp2_tee_ops->repeater_check_flow_prepare_ack(hdcp2_tee,
										buf,
										msg_size,
										mV);
	if (ret != HDCP2_SUCCESS)
		DRM_DEBUG_KMS("Verify rep topology failed. %d\n", ret);

	return ret;
}

static int hdcp2_verify_mprime(struct rtk_hdcp *hdcp,
		    struct hdcp2_rep_stream_ready *stream_ready,
		    u8 *input, int input_size)
{
	int ret = HDCP2_COMPARE_M_ERROR;
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->verify_mprime)
		ret = hdcp2_tee->hdcp2_tee_ops->verify_mprime(hdcp2_tee, stream_ready,
							      input, input_size);
	if (ret != HDCP2_SUCCESS)
		DRM_DEBUG_KMS("Verify mprime failed. %d\n", ret);

	return ret;
}

static int hdcp2_authentication_key_exchange(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	struct drm_device *dev = hdmi->connector.dev;
	union {
		struct hdcp2_ake_init ake_init;
		struct hdcp2_ake_send_cert send_cert;
		struct hdcp2_ake_no_stored_km no_stored_km;
		struct hdcp2_ake_send_hprime send_hprime;
		struct hdcp2_ake_send_pairing_info pairing_info;
	} msgs;
	const struct rtk_hdcp_ops *hdcp_ops = hdcp->hdcp_ops;
	size_t size;
	int ret;

	hdcp->hdcp1_device_downstream = 0;
	hdcp->seq_num_m = 0;

	hdcp2_prepare_ake_init(hdcp, &msgs.ake_init);

	memcpy(&hdcp->tx_caps, &msgs.ake_init.tx_caps, sizeof(struct hdcp2_tx_caps));
	memcpy(hdcp->r_tx, msgs.ake_init.r_tx, HDCP_2_2_RTX_LEN);

	ret = hdcp_ops->write_2_2_msg(hdmi, &msgs.ake_init,
				      sizeof(msgs.ake_init));
	if (ret < 0)
		return ret;

	ret = hdcp_ops->read_2_2_msg(hdmi, HDCP_2_2_AKE_SEND_CERT,
				     &msgs.send_cert, sizeof(msgs.send_cert));

	if(ret == -HDCP_PLUGOUT_EVENT)
		return -HDCP_PLUGOUT_EVENT;
	else if (ret < 0) {
		ret = HDCP2_READCERT_TIMEOUT;
		return ret;
	}

	if (msgs.send_cert.rx_caps[0] != HDCP_2_2_RX_CAPS_VERSION_VAL) {
		DRM_ERROR("cert.rx_caps dont claim HDCP2.2\n");
		return -EINVAL;
	}

	memcpy(hdcp->r_rx, msgs.send_cert.r_rx, HDCP_2_2_RRX_LEN);
	memcpy(hdcp->rx_caps, msgs.send_cert.rx_caps, HDCP_2_2_RXCAPS_LEN);
	hdcp->is_repeater = HDCP_2_2_RX_REPEATER(msgs.send_cert.rx_caps[2]);

	if (drm_hdcp_check_ksvs_revoked(dev, msgs.send_cert.cert_rx.receiver_id, 1)) {
		DRM_ERROR("Receiver ID is revoked\n");
		return -EPERM;
	}

	/*
	 * Here msgs.no_stored_km will hold msgs corresponding to the km
	 * stored also.
	 */
	ret = hdcp2_verify_rx_cert_prepare_km(hdcp, &msgs.send_cert,
					      &hdcp->is_paired,
					      &msgs.no_stored_km, &size);
	if (ret != 0)
		return ret;

	ret = hdcp_ops->write_2_2_msg(hdmi, &msgs.no_stored_km, size);
	if (ret < 0)
		return ret;

	ret = hdcp_ops->read_2_2_msg(hdmi, HDCP_2_2_AKE_SEND_HPRIME,
				     &msgs.send_hprime, sizeof(msgs.send_hprime));

	if(ret == -HDCP_PLUGOUT_EVENT)
		return -HDCP_PLUGOUT_EVENT;
	else if (ret < 0) {
		ret = HDCP2_READHPRIME_TIMEOUT;
		return ret;
	}

	ret = hdcp2_verify_hprime(hdcp, &msgs.send_hprime);
	if (ret != 0)
		return ret;

	if (!hdcp->is_paired) {
		/* Pairing is required */
		ret = hdcp_ops->read_2_2_msg(hdmi,
					     HDCP_2_2_AKE_SEND_PAIRING_INFO,
					     &msgs.pairing_info,
					     sizeof(msgs.pairing_info));
		if(ret == -HDCP_PLUGOUT_EVENT)
			return -HDCP_PLUGOUT_EVENT;
		else if (ret < 0) {
			ret = HDCP2_READPAIRING_TIMEOUT;
			return ret;
		}

		ret = hdcp2_store_pairing_info(hdcp, &msgs.pairing_info);
		if (ret != HDCP2_SUCCESS)
			return ret;
		hdcp->is_paired = true;
	}

	return HDCP2_SUCCESS;
}

static int hdcp2_locality_check(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	union {
		struct hdcp2_lc_init lc_init;
		struct hdcp2_lc_send_lprime send_lprime;
	} msgs;
	const struct rtk_hdcp_ops *hdcp_ops = hdcp->hdcp_ops;
	int tries = HDCP2_LC_RETRY_CNT, ret, i;

	for (i = 0; i < tries; i++) {
		hdcp2_prepare_lc_init(hdcp, &msgs.lc_init);

		ret = hdcp_ops->write_2_2_msg(hdmi, &msgs.lc_init,
					      sizeof(msgs.lc_init));
		if (ret < 0)
			break;

		ret = hdcp_ops->read_2_2_msg(hdmi,
					     HDCP_2_2_LC_SEND_LPRIME,
					     &msgs.send_lprime,
					     sizeof(msgs.send_lprime));
		if(ret == -HDCP_PLUGOUT_EVENT)
			return -HDCP_PLUGOUT_EVENT;
		else if (ret < 0) {
			ret = HDCP2_READLPRIME_TIMEOUT;
			continue;
		}

		ret = hdcp2_verify_lprime(hdcp, &msgs.send_lprime);
		if (!ret)
			break;
	}

	return ret;
}

static int hdcp2_session_key_exchange(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	struct hdcp2_ske_send_eks send_eks;
	const struct rtk_hdcp_ops *hdcp_ops = hdcp->hdcp_ops;
	int ret;

	hdcp2_prepare_skey(hdcp, &send_eks);

	ret = hdcp_ops->write_2_2_msg(hdmi, &send_eks,
				      sizeof(send_eks));
	if (ret < 0)
		return ret;

	return 0;
}

static
int hdcp2_authenticate_repeater_topology(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	struct drm_device *dev = hdmi->connector.dev;
	union {
		struct hdcp2_rep_send_receiverid_list recvid_list;
		struct hdcp2_rep_send_ack rep_ack;
	} msgs;
	const struct rtk_hdcp_ops *hdcp_ops = hdcp->hdcp_ops;
	u32 device_cnt;
	u8 *rx_info, *buf;
	u8 mV[HDCP_2_2_V_PRIME_HALF_LEN];
	int ret;
	int msg_sz;

	msg_sz = rtk_hdmi_hdcp2_wait_for_msg(hdmi,
					     HDCP_2_2_REP_SEND_RECVID_LIST,
					     hdcp->is_paired);

	DRM_DEBUG_KMS("wait for ReceiverList ready and msg_sz: %d.\n",
			msg_sz);

	buf = kzalloc(msg_sz, GFP_KERNEL);
	if(!buf) {
		DRM_ERROR("%s: buf alloacated failed\n", __func__);
		return -ENOMEM;
	}

	ret = hdcp_ops->read_2_2_msg(hdmi, HDCP_2_2_REP_SEND_RECVID_LIST,
				     buf, msg_sz);
	if (ret == -HDCP_PLUGOUT_EVENT)
		return -HDCP_PLUGOUT_EVENT;
	else if (ret < 0) {
		ret = HDCP2_READRECEIVEIDLIST_TIMEOUT;
		kfree(buf);
		return ret;
	}

	ret = hdcp2_verify_rep_topology_prepare_ack(hdcp,
						    buf,
						    msg_sz,
						    mV);
	if (ret != HDCP2_SUCCESS) {
		kfree(buf);
		return ret;
	}

	memcpy(&msgs.recvid_list, buf, msg_sz);

	kfree(buf);

	rx_info = msgs.recvid_list.rx_info;

	if (HDCP_2_2_MAX_CASCADE_EXCEEDED(rx_info[1]) ||
	    HDCP_2_2_MAX_DEVS_EXCEEDED(rx_info[1])) {
		DRM_ERROR("Topology Max Size Exceeded\n");
		return -EINVAL;
	}

	device_cnt = (HDCP_2_2_DEV_COUNT_HI(rx_info[0]) << 4 |
		      HDCP_2_2_DEV_COUNT_LO(rx_info[1]));
	if (drm_hdcp_check_ksvs_revoked(dev, msgs.recvid_list.receiver_ids,
					device_cnt)) {
		DRM_ERROR("Revoked receiver ID(s) is in list\n");
		return -EPERM;
	}

	hdcp->hdcp1_device_downstream = (rx_info[1] >> 0) & 0x1;

	msgs.rep_ack.msg_id = HDCP_2_2_REP_SEND_ACK;
	memcpy(msgs.rep_ack.v, mV, HDCP_2_2_V_PRIME_HALF_LEN);

	ret = hdcp_ops->write_2_2_msg(hdmi, &msgs.rep_ack,
				      sizeof(msgs.rep_ack));
	if (ret < 0)
		return ret;

	return 0;
}

static int
hdcp2_propagate_stream_management_info(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	union {
		struct hdcp2_rep_stream_manage stream_manage;
		struct hdcp2_rep_stream_ready stream_ready;
	} msgs;
	const struct rtk_hdcp_ops *hdcp_ops = hdcp->hdcp_ops;
	int ret;
	u8 seq_num_m[HDCP_2_2_SEQ_NUM_LEN];
	u8 streamIDtype[2];
	u8 input[5];

	/* Prepare RepeaterAuth_Stream_Manage msg */
	msgs.stream_manage.msg_id = HDCP_2_2_REP_STREAM_MANAGE;
	drm_hdcp_cpu_to_be24(msgs.stream_manage.seq_num_m, hdcp->seq_num_m);
	memcpy(seq_num_m, msgs.stream_manage.seq_num_m, HDCP_2_2_SEQ_NUM_LEN);

	/* K no of streams is fixed as 1. Stored as big-endian. */
	msgs.stream_manage.k = cpu_to_be16(1);

	/* For HDMI this is forced to be 0x0. For DP SST also this is 0x0. */
	msgs.stream_manage.streams[0].stream_id = 0;

	/*We set type 0 content if there is an HDCP 1.4 downstream device*/
	if(hdcp->hdcp1_device_downstream == 1)
		msgs.stream_manage.streams[0].stream_type =
					DRM_MODE_HDCP_CONTENT_TYPE0;
	else
		msgs.stream_manage.streams[0].stream_type =
					DRM_MODE_HDCP_CONTENT_TYPE1;

	streamIDtype[0] = msgs.stream_manage.streams[0].stream_id;
	streamIDtype[1] = msgs.stream_manage.streams[0].stream_type;

	memcpy(input, streamIDtype, 2);
	memcpy(input+2, seq_num_m, HDCP_2_2_SEQ_NUM_LEN);

	/* Send it to Repeater */
	ret = hdcp_ops->write_2_2_msg(hdmi, &msgs.stream_manage,
				      sizeof(msgs.stream_manage));
	if (ret < 0)
		return ret;

	hdcp->seq_num_m++;

	ret = hdcp_ops->read_2_2_msg(hdmi, HDCP_2_2_REP_STREAM_READY,
				     &msgs.stream_ready, sizeof(msgs.stream_ready));
	if (ret == -HDCP_PLUGOUT_EVENT)
		return -HDCP_PLUGOUT_EVENT;
	else if (ret < 0) {
		ret = HDCP2_READSTREAMREADY_TIMEOUT;
		return ret;
	}

	ret = hdcp2_verify_mprime(hdcp, &msgs.stream_ready,
				  input, 2+HDCP_2_2_SEQ_NUM_LEN);
	if (ret != HDCP2_SUCCESS)
		return ret;

	if (hdcp->seq_num_m > HDCP_2_2_SEQ_NUM_MAX) {
		DRM_DEBUG_KMS("seq_num_m roll over.\n");
		return -1;
	}

	return 0;
}

static int hdcp2_authenticate_repeater(struct rtk_hdmi *hdmi)
{
	int ret;
	int i = 0, tries = 10;

	ret = hdcp2_authenticate_repeater_topology(hdmi);
	if (ret != HDCP2_SUCCESS)
		return ret;

	for(i=0; i<tries; i++) {
		ret = hdcp2_propagate_stream_management_info(hdmi);
		if(!ret)
			break;
	}

	return ret;
}

static int hdcp2_authenticate_sink(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	int ret;
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->hdcp2_tee_api_init)
		hdcp2_tee->hdcp2_tee_ops->hdcp2_tee_api_init(hdcp2_tee);

	hdcp->mCap[0] = WV_HDCP_NONE;

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->read_hdcp2_key) {
		ret = hdcp2_tee->hdcp2_tee_ops->read_hdcp2_key(hdcp2_tee);
		if (ret != HDCP2_SUCCESS) {
			DRM_DEBUG_KMS("Read HDCP 2.2 key Failed. Err : %d\n", ret);
			return ret;
		}
	}

	DRM_DEBUG_KMS("[HDCP22] AKE Init.\n");
	ret = hdcp2_authentication_key_exchange(hdmi);
	if (ret != HDCP2_SUCCESS) {
		DRM_DEBUG_KMS("AKE Failed. Err : %d\n", ret);
		return ret;
	}

	DRM_DEBUG_KMS("[HDCP22] Locality Check.\n");
	ret = hdcp2_locality_check(hdmi);
	if (ret != HDCP2_SUCCESS) {
		DRM_DEBUG_KMS("Locality Check failed. Err : %d\n", ret);
		return ret;
	}

	DRM_DEBUG_KMS("[HDCP22] SKE Init.\n");
	ret = hdcp2_session_key_exchange(hdmi);
	if (ret < 0) {
		DRM_DEBUG_KMS("SKE Failed. Err : %d\n", ret);
		return ret;
	}

	if (hdcp->is_repeater) {
		DRM_DEBUG_KMS("[HDCP22] Auth Repeater.\n");
		ret = hdcp2_authenticate_repeater(hdmi);
		if (ret != HDCP2_SUCCESS) {
			DRM_DEBUG_KMS("Repeater Auth Failed. Err: %d\n", ret);
			return ret;
		}

		if(hdcp->hdcp1_device_downstream == 1)
			hdcp->mCap[0]  = WV_HDCP_V1;
		else
			hdcp->mCap[0] = WV_HDCP_V2_2;
	}
	else
		hdcp->mCap[0] = WV_HDCP_V2_2;

	return ret;
}

static void hdcp2_enable_encryption(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->enable_hdcp2_cipher)
		hdcp2_tee->hdcp2_tee_ops->enable_hdcp2_cipher(hdcp2_tee,
							      hdcp->mCap);
}

static void hdcp2_disable_encryption(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->disable_hdcp2_cipher)
		hdcp2_tee->hdcp2_tee_ops->disable_hdcp2_cipher(hdcp2_tee);

	/*Disable hdcp 2.2 cipher encryption will take effect in the next frame,
	  However, clear HDCP 2.2 cipher setting will take effect immediately,
	  The result is that use the incorrect cipher parameters to encrypt the current frame*/

	/*Make sure clearing hdcp 2.2 cipher setting is in the next frame*/
	msleep(50);

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->clear_hdcp2_cipher_setting)
		hdcp2_tee->hdcp2_tee_ops->clear_hdcp2_cipher_setting(hdcp2_tee);

	hdcp->mCap[0] = WV_HDCP_NONE;

	if(hdcp2_tee->hdcp2_tee_ops &&
	   hdcp2_tee->hdcp2_tee_ops->update_mCap)
		hdcp2_tee->hdcp2_tee_ops->update_mCap(hdcp2_tee,
						      hdcp->mCap);
}

static int hdcp2_authenticate_and_encrypt(struct rtk_hdmi *hdmi)
{
	int ret, i, tries = 3;
	int hpd;

	for (i = 0; i < tries; i++) {
		ret = hdcp2_authenticate_sink(hdmi);
		if (!ret)
			break;

		/* Clearing the mei hdcp session */
		DRM_DEBUG_KMS("HDCP2.2 Auth %d of %d Failed.(%d)\n",
				i + 1, tries, ret);

		if(ret == -HDCP_PLUGOUT_EVENT)
			return ret;
	}

	if (i != tries) {
		/*
		 * Ensuring the required 200mSec min time interval between
		 * Session Key Exchange and encryption. During this period,
		 * we also detect if there is any plugout events.
		 */
		for(i=0; i < 20; i++) {
			hpd = gpiod_get_value(hdmi->hpd_gpio);
			if(!hpd)
				return -HDCP_PLUGOUT_EVENT;
			msleep(HDCP_2_2_DELAY_BEFORE_ENCRYPTION_EN/20);
		}

		hdcp2_enable_encryption(hdmi);
	}

	return ret;
}

static int rtk_hdcp_generate_an(struct rtk_hdcp *hdcp, u8 *an)
{
	struct rtk_hdcp1_tee *hdcp1_tee = &hdcp->hdcp1_tee;
	int ret = HDCP1_GEN_AN_ERROR;

	if(hdcp1_tee->hdcp1_tee_ops &&
	   hdcp1_tee->hdcp1_tee_ops->generate_an)
		ret = hdcp1_tee->hdcp1_tee_ops->generate_an(hdcp1_tee, an);

	if (ret != HDCP1_SUCCESS)
		DRM_DEBUG_KMS("Generate an failed, ret=%d.\n", ret);

	return ret;
}

static int rtk_hdcp_read_valid_aksv(struct rtk_hdcp *hdcp, u8 *aksv)
{
	struct rtk_hdcp1_tee *hdcp1_tee = &hdcp->hdcp1_tee;
	int ret = HDCP1_AKSV_ERROR;

	if(hdcp1_tee->hdcp1_tee_ops &&
	   hdcp1_tee->hdcp1_tee_ops->read_aksv)
		ret = hdcp1_tee->hdcp1_tee_ops->read_aksv(hdcp1_tee,
							  aksv);

	if (ret != HDCP1_SUCCESS)
		DRM_DEBUG_KMS("Read aksv failed, ret=%d.\n", ret);
	else
		ret = rtk_hdcp_is_ksv_valid(aksv);

	return ret;
}

static void
rtk_hdcp_set_repeater_bit(struct rtk_hdcp *hdcp, bool is_repeater)
{
	struct rtk_hdcp1_tee *hdcp1_tee = &hdcp->hdcp1_tee;

	if(hdcp1_tee->hdcp1_tee_ops &&
	   hdcp1_tee->hdcp1_tee_ops->set_hdcp1_repeater_bit) {
		if(is_repeater)
			hdcp1_tee->hdcp1_tee_ops->set_hdcp1_repeater_bit(hdcp1_tee, 1);
		else
			hdcp1_tee->hdcp1_tee_ops->set_hdcp1_repeater_bit(hdcp1_tee, 0);
	}
}

static int
rtk_hdcp_write_bksv(struct rtk_hdcp *hdcp, u8 *bksv)
{
	struct rtk_hdcp1_tee *hdcp1_tee = &hdcp->hdcp1_tee;
	int ret = HDCP1_BKSV_ERROR;

	if(hdcp1_tee->hdcp1_tee_ops &&
	   hdcp1_tee->hdcp1_tee_ops->write_bksv)
		ret = hdcp1_tee->hdcp1_tee_ops->write_bksv(hdcp1_tee, bksv);

	return ret;
}

static int rtk_hdcp_poll_ksv_fifo(struct rtk_hdmi *hdmi,
				  const struct rtk_hdcp_ops *hdcp_ops)
{
	int err_ret = HDCP1_BCAPS_RDY_ERROR;
	int ret, read_ret;
	bool ksv_ready;

	if(hdcp_ops && hdcp_ops->read_ksv_ready) {
		/* Poll for ksv list ready (spec says max time allowed is 5s) */
		ret = __wait_for(read_ret = hdcp_ops->read_ksv_ready(hdmi,
								     &ksv_ready),
				 read_ret == -HDCP_PLUGOUT_EVENT,
				 read_ret || ksv_ready, 5 * 1000 * 1000, 1000,
				 100 * 1000);
		if (ret)
			return ret;
		if (read_ret)
			return read_ret;
		if (!ksv_ready)
			goto exit;

		err_ret = HDCP1_SUCCESS;
	}
exit:
	return err_ret;
}

static void
rtk_hdcp_sha_append_bstatus_m0(struct rtk_hdcp *hdcp, u8 *ksv_fifo,
			       int *byte_cnt, u8 *bstatus)
{
	struct rtk_hdcp1_tee *hdcp1_tee = &hdcp->hdcp1_tee;

	if(hdcp1_tee->hdcp1_tee_ops &&
	   hdcp1_tee->hdcp1_tee_ops->sha_append_bstatus_m0)
		hdcp1_tee->hdcp1_tee_ops->sha_append_bstatus_m0(hdcp1_tee,
								ksv_fifo,
								byte_cnt,
								bstatus);
}

static int
rtk_hdcp_compute_and_validate_V(struct rtk_hdcp *hdcp, u8 *ksv_fifo,
				int *byte_cnt, u8 *vprime)
{
	int ret = HDCP1_SHA1_ERROR;
	struct rtk_hdcp1_tee *hdcp1_tee = &hdcp->hdcp1_tee;

	DRM_DEBUG_KMS("[HDCP14] Compute V.\n");
	if(hdcp1_tee->hdcp1_tee_ops &&
	   hdcp1_tee->hdcp1_tee_ops->compute_V) {
		ret = hdcp1_tee->hdcp1_tee_ops->compute_V(hdcp1_tee,
							  ksv_fifo,
							  byte_cnt);
		if(ret)
			return ret;
	}

	DRM_DEBUG_KMS("[HDCP14] Verify V.\n");
	if(hdcp1_tee->hdcp1_tee_ops &&
	   hdcp1_tee->hdcp1_tee_ops->verify_V) {
		ret = hdcp1_tee->hdcp1_tee_ops->verify_V(hdcp1_tee,
							 vprime);

		if(ret)
			return ret;
	}

	DRM_DEBUG_KMS("[HDCP14] Verify V Pass.\n");

	return ret;
}

static int rtk_hdcp_auth_downstream(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	struct drm_device *dev = hdmi->connector.dev;
	const struct rtk_hdcp_ops *hdcp_ops = hdcp->hdcp_ops;
	u8 bstatus[DRM_HDCP_BSTATUS_LEN];
	int num_downstream, byte_cnt;
	u8 ksv_fifo[MAX_SHA_DATA_SIZE];
	u32 vprime_part;
	u8 vprime[MAX_SHA_VPRIME_SIZE];
	int ret = -1, i;

	DRM_DEBUG_KMS("[HDCP14] Wait KSV list ready bit.\n");
	ret = rtk_hdcp_poll_ksv_fifo(hdmi, hdcp_ops);
	if(ret) {
		DRM_DEBUG_KMS("KSV list failed to become ready (%d)\n", ret);
		return ret;
	}

	if(hdcp_ops && hdcp_ops->read_bstatus) {
		ret = hdcp_ops->read_bstatus(hdmi, bstatus);
		if (ret)
			return ret;
	}

	if (DRM_HDCP_MAX_DEVICE_EXCEEDED(bstatus[0]) ||
	    DRM_HDCP_MAX_CASCADE_EXCEEDED(bstatus[1])) {
		DRM_DEBUG_KMS("Max Topology Limit Exceeded, bstatus[0]=0x%x, bstatus[1]=0x%x\n",
				bstatus[0], bstatus[1]);
		return HDCP1_AUTH_FAILURE;
	}

	DRM_DEBUG_KMS("[HDCP14] Wait KSV list.\n");

	memset(ksv_fifo, 0, MAX_SHA_DATA_SIZE);

	num_downstream = DRM_HDCP_NUM_DOWNSTREAM(bstatus[0]);
	byte_cnt = num_downstream * DRM_HDCP_KSV_LEN;

#if 0
	/*
	 * When repeater reports 0 device count, HDCP1.4 spec allows disabling
	 * the HDCP encryption. That implies that repeater can't have its own
	 * display. As there is no consumption of encrypted content in the
	 * repeater with 0 downstream devices, we are failing the
	 * authentication.
	 */

	if (num_downstream == 0) {
		DRM_DEBUG_KMS("Repeater with zero downstream devices\n");

		return -EINVAL;
	}
#endif
	if(num_downstream &&
	   hdcp_ops && hdcp_ops->read_ksv_fifo) {
		ret = hdcp_ops->read_ksv_fifo(hdmi, num_downstream, ksv_fifo);
		if (ret)
			return ret;;
	}

	if (num_downstream &&
	    drm_hdcp_check_ksvs_revoked(dev, ksv_fifo, num_downstream)) {
		DRM_ERROR("Revoked Ksv(s) in ksv_fifo\n");
		ret = -EPERM;
		return ret;
	}

	/* Read and add Bstatus */
	rtk_hdcp_sha_append_bstatus_m0(hdcp, ksv_fifo,
				       &byte_cnt,
				       bstatus);

	DRM_DEBUG_KMS("[HDCP14] Read V prime.\n");
	memset(vprime, 0, MAX_SHA_VPRIME_SIZE);
	if(hdcp_ops && hdcp_ops->read_v_prime_part) {
		for (i = 0; i < DRM_HDCP_V_PRIME_NUM_PARTS; i++) {
			ret = hdcp_ops->read_v_prime_part(hdmi, i, &vprime_part);
			if (ret)
				return ret;
			memcpy(vprime + i*DRM_HDCP_V_PRIME_PART_LEN,
				&vprime_part, DRM_HDCP_V_PRIME_PART_LEN);
		}
	}

	ret = rtk_hdcp_compute_and_validate_V(hdcp, ksv_fifo,
					      &byte_cnt, vprime);

	DRM_DEBUG_KMS("[HDCP14] %d downstream devices is verified.\n",
			num_downstream);
	return ret;
}

static void rtk_hdcp_set_encryption(struct rtk_hdcp *hdcp, u8 enc_state)
{
	struct rtk_hdcp1_tee *hdcp1_tee = &hdcp->hdcp1_tee;

	if(hdcp1_tee->hdcp1_tee_ops &&
	   hdcp1_tee->hdcp1_tee_ops->hdcp1_set_encryption)
		hdcp1_tee->hdcp1_tee_ops->hdcp1_set_encryption(hdcp1_tee,
							       enc_state);
}

static void rtk_hdcp_set_wider_window(struct rtk_hdcp *hdcp)
{
	struct rtk_hdcp1_tee *hdcp1_tee = &hdcp->hdcp1_tee;

	if(hdcp1_tee->hdcp1_tee_ops &&
	   hdcp1_tee->hdcp1_tee_ops->set_wider_window)
		hdcp1_tee->hdcp1_tee_ops->set_wider_window(hdcp1_tee);
}

/* Implements Part 1 of the HDCP authorization procedure */
static int rtk_hdcp_auth(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	struct drm_device *dev = hdmi->connector.dev;
	const struct rtk_hdcp_ops *hdcp_ops = hdcp->hdcp_ops;
	int ret = -1;
	u8 an[DRM_HDCP_AN_LEN];
	u8 aksv[DRM_HDCP_KSV_LEN];
	u8 bksv[DRM_HDCP_KSV_LEN];
	u8 ri_prime[DRM_HDCP_RI_LEN];
	unsigned long r0_prime_gen_start;
	int i, hpd;

	/* Generate An */
	DRM_DEBUG_KMS("[HDCP14] Generate An.\n");
	ret = rtk_hdcp_generate_an(hdcp, an);
	if(ret)
		return ret;

	DRM_DEBUG_KMS("[HDCP14] Read Aksv.\n");
	ret = rtk_hdcp_read_valid_aksv(hdcp, aksv);
	if(ret)
		return ret;

	if(hdcp_ops && hdcp_ops->write_an_aksv)
		ret = hdcp_ops->write_an_aksv(hdmi, an, aksv);
	if(ret)
		return ret;

	r0_prime_gen_start = jiffies;

	memset(bksv, 0, DRM_HDCP_KSV_LEN);
	DRM_DEBUG_KMS("[HDCP14] Read Bksv.\n");
	ret = rtk_hdcp_read_valid_bksv(hdmi, hdcp_ops, bksv);
	if (ret)
		return ret;

	if (drm_hdcp_check_ksvs_revoked(dev, bksv, 1)) {
		DRM_ERROR("BKSV is revoked\n");
		return -EPERM;
	}

	if(hdcp_ops && hdcp_ops->repeater_present)
		ret = hdcp_ops->repeater_present(hdmi, &hdcp->is_repeater);

	if (ret)
		return ret;

	rtk_hdcp_set_repeater_bit(hdcp, hdcp->is_repeater);

	DRM_DEBUG_KMS("[HDCP14] Generate Km.\n");
	ret = rtk_hdcp_write_bksv(hdcp, bksv);
	if(ret)
		return ret;

	/*
	 * Wait for R0' to become available. The spec says 100ms from Aksv, but
	 * some monitors can take longer than this. We'll set the timeout at
	 * 300ms just to be sure.
	 *
	 */
	wait_remaining_ms_from_jiffies(r0_prime_gen_start, 300);

	DRM_DEBUG_KMS("[HDCP14] Read r0'.\n");
	if(hdcp_ops && hdcp_ops->read_ri_prime)
		ret = hdcp_ops->read_ri_prime(hdmi, ri_prime);
	if (ret)
		return ret;

	DRM_DEBUG_KMS("[HDCP14] Check r0'.\n");
	ret = rtk_hdcp_check_ri_prime(hdcp, ri_prime);
	/* we add plugout event detection in each i2c read/write stage,
	 * However, checking ri' stage is not i2c read/write
	 * so that's why we add plugout event detection here.
	 * It's also for HDCP 1.4 CTS 1B-02 test item.
	 */
	for(i=0; i < 20; i++) {
		hpd = gpiod_get_value(hdmi->hpd_gpio);
		if(!hpd)
			return -HDCP_PLUGOUT_EVENT;
		msleep(10);
	}

	if (ret)
		return ret;

	if(hdcp->is_repeater) {
		DRM_DEBUG_KMS("[HDCP14] Auth Repeater.\n");
		ret = rtk_hdcp_auth_downstream(hdmi);
		if (ret)
			return ret;
	}

	DRM_DEBUG_KMS("[HDCP14] Enable hdcp 1.4 cipher.\n");
	rtk_hdcp_set_encryption(hdcp, HDCP_ENC_ON);
	rtk_hdcp_set_wider_window(hdcp);

	return ret;
}

static void _rtk_hdcp_disable(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;

	DRM_DEBUG_KMS("%s: HDCP1.4 is being Disabled\n", __func__);

	rtk_hdcp_set_encryption(hdcp, HDCP_ENC_OFF);

	DRM_DEBUG_KMS("%s: HDCP1.4 is Disabled\n", __func__);

	hdcp->hdcp_encrypted = false;
}

static int _rtk_hdcp_enable(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	struct rtk_hdcp1_tee *hdcp1_tee = &hdcp->hdcp1_tee;
	int retry;
	int ret;
	ktime_t end_t;
	bool is_expired;

	DRM_DEBUG_KMS("[%s] HDCP1.4 is being enabled.\n", __func__);

	if(hdcp1_tee->hdcp1_tee_ops &&
	   hdcp1_tee->hdcp1_tee_ops->hdcp1_tee_api_init)
		hdcp1_tee->hdcp1_tee_ops->hdcp1_tee_api_init(hdcp1_tee);

	retry = 0;
	end_t = ktime_add_ms(ktime_get_raw(), HDCP14_EN_TIMEOUT_MS);
	/* Incase of authentication failures, HDCP spec expects reauth. */
	for (;;) {
		ret = rtk_hdcp_auth(hdmi);
		if (!ret) {
			hdcp->hdcp_encrypted = true;
			DRM_DEBUG_KMS("[%s] HDCP1.4 is enabled.\n", __func__);
			return 0;
		}

		DRM_DEBUG_KMS("HDCP Auth failure (%d)\n", ret);

		/* Ensuring HDCP encryption and signalling are stopped. */
		_rtk_hdcp_disable(hdmi);

		if (ret == -HDCP_PLUGOUT_EVENT)
			break;

		is_expired = ktime_after(ktime_get_raw(), end_t);
		if (is_expired && retry > 0)
			break;

		retry++;
		msleep(30);
	}

	DRM_DEBUG_KMS("HDCP1.4 authentication failed, ret=%d\n", ret);

	return ret;
}

static int _rtk_hdcp2_enable(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	int ret;

	DRM_DEBUG_KMS("HDCP2.2 is being enabled. Type: %d\n",
			hdcp->content_type);

	ret = hdcp2_authenticate_and_encrypt(hdmi);
	if (ret) {
		DRM_DEBUG_KMS("HDCP2 Type%d  Enabling Failed. (%d)\n",
				hdcp->content_type, ret);
		return ret;
	}

	DRM_DEBUG_KMS("HDCP2.2 is enabled. Type %d\n",
			hdcp->content_type);

	hdcp->hdcp2_encrypted = true;
	return 0;
}

static void _rtk_hdcp2_disable(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;

	DRM_DEBUG_KMS("%s: HDCP2.2 is being Disabled\n", __func__);

	hdcp2_disable_encryption(hdmi);

	DRM_DEBUG_KMS("%s: HDCP2.2 is Disabled\n", __func__);

	hdcp->hdcp2_encrypted = false;
}

void rtk_hdcp_update_mCap(struct rtk_hdmi *hdmi, u8 mCap)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;

	/*mCap is only available for HDCP 2.2*/
	if(hdcp->hdcp2_supported) {
		mutex_lock(&hdcp->mutex);

		hdcp->mCap[0] = mCap;

		if(hdcp2_tee->hdcp2_tee_ops &&
		   hdcp2_tee->hdcp2_tee_ops->update_mCap)
			hdcp2_tee->hdcp2_tee_ops->update_mCap(hdcp2_tee,
							      hdcp->mCap);

		mutex_unlock(&hdcp->mutex);
	}
}

static int rtk_hdcp_check_link(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	const struct rtk_hdcp_ops *hdcp_ops = hdcp->hdcp_ops;
	int ret = -1;

	mutex_lock(&hdcp->mutex);

	/* Check_link valid only when HDCP1.4 is enabled */
	if (hdcp->value != DRM_MODE_CONTENT_PROTECTION_ENABLED ||
	    !hdcp->hdcp_encrypted) {
		ret = -EINVAL;
		goto out;
	}

	if (hdcp_ops && hdcp_ops->check_link)
		ret = hdcp_ops->check_link(hdmi);

	if (ret == HDCP_LINK_PROTECTED) {
		DRM_DEBUG_KMS("[%s] HDCP1.4 integirty check ri = ri'.\n",
				__func__);

		if (hdcp->value != DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
			hdcp->value = DRM_MODE_CONTENT_PROTECTION_ENABLED;
			schedule_work(&hdcp->prop_work);
		}
		goto out;
	}

	DRM_ERROR("[%s] HDCP1.4 link failed, retrying authentication\n",
			__func__);

	_rtk_hdcp_disable(hdmi);

	ret = _rtk_hdcp_enable(hdmi);
	if (ret) {
		DRM_ERROR("[%s] Failed to enable hdcp 1.4 (%d)\n",
				__func__, ret);
		hdcp->value = DRM_MODE_CONTENT_PROTECTION_DESIRED;
		schedule_work(&hdcp->prop_work);
		goto out;
	}

out:
	mutex_unlock(&hdcp->mutex);
	return ret;
}

/* Implements the Link Integrity Check for HDCP2.2 */
static int rtk_hdcp2_check_link(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	const struct rtk_hdcp_ops *hdcp_ops = hdcp->hdcp_ops;
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;
	int ret = 0;

	mutex_lock(&hdcp->mutex);

	/* hdcp2_check_link is expected only when HDCP2.2 is Enabled */
	if (hdcp->value != DRM_MODE_CONTENT_PROTECTION_ENABLED ||
	    !hdcp->hdcp2_encrypted) {
		ret = -EINVAL;
		goto out;
	}

	if(hdcp_ops && hdcp_ops->check_2_2_link)
		ret = hdcp_ops->check_2_2_link(hdmi);

	if (ret == HDCP_LINK_PROTECTED) {
		DRM_DEBUG_KMS("%s: HDCP 2.2 polling rx status.\n", __func__);

		if (hdcp->value != DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
			hdcp->value = DRM_MODE_CONTENT_PROTECTION_ENABLED;
			schedule_work(&hdcp->prop_work);
		}
		goto out;
	}
	if (ret == HDCP_TOPOLOGY_CHANGE) {
		if (hdcp->value == DRM_MODE_CONTENT_PROTECTION_UNDESIRED)
			goto out;

		DRM_DEBUG_KMS("HDCP2.2 Downstream topology change\n");
		ret = hdcp2_authenticate_repeater_topology(hdmi);
		if (!ret) {
			if(hdcp->hdcp1_device_downstream == 1)
				hdcp->mCap[0]  = WV_HDCP_V1;
			else
				hdcp->mCap[0] = WV_HDCP_V2_2;

			if(hdcp2_tee->hdcp2_tee_ops &&
			   hdcp2_tee->hdcp2_tee_ops->update_mCap)
				hdcp2_tee->hdcp2_tee_ops->update_mCap(hdcp2_tee,
								      hdcp->mCap);
			hdcp->value = DRM_MODE_CONTENT_PROTECTION_ENABLED;
			schedule_work(&hdcp->prop_work);
			goto out;
		}
		DRM_DEBUG_KMS("%s: HDCP 2.2 Repeater topology auth failed.(%d)\n",
				__func__, ret);
	}
	else {
		DRM_DEBUG_KMS("%s: HDCP2.2 link failed, retrying auth (%d)\n",
				__func__, ret);
	}

	_rtk_hdcp2_disable(hdmi);

	ret = _rtk_hdcp2_enable(hdmi);
	if (ret) {
		DRM_DEBUG_KMS("%s: Failed to enable hdcp2.2 (%d)\n",
				__func__, ret);
		hdcp->value = DRM_MODE_CONTENT_PROTECTION_DESIRED;
		schedule_work(&hdcp->prop_work);
		goto out;
	}

out:
	mutex_unlock(&hdcp->mutex);
	return ret;
}

int rtk_hdcp_enable(struct rtk_hdmi *hdmi, u8 content_type)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	unsigned long check_link_interval = DRM_HDCP_CHECK_PERIOD_MS;
	int ret = -EINVAL;
	int capable14;

	if (!hdcp || !hdcp->hdcp_ops)
		return -ENOENT;

	mutex_lock(&hdcp->mutex);
	WARN_ON(hdcp->value == DRM_MODE_CONTENT_PROTECTION_ENABLED);
	hdcp->content_type = content_type;

	/*
	 * Considering that HDCP2.2 is more secure than HDCP1.4, If the setup
	 * is capable of HDCP2.2, it is preferred to use HDCP2.2.
	 */
	if(rtk_hdcp2_capable(hdmi)) {
		ret = _rtk_hdcp2_enable(hdmi);
		if (ret == -HDCP_PLUGOUT_EVENT)
			goto unlock;

		if (!ret) {
			check_link_interval = DRM_HDCP2_CHECK_PERIOD_MS;
			goto check_link;
		}
	}
	DRM_DEBUG_KMS("%s hdcp2 ret=%d\n", __func__, ret);
	/*
	 * When HDCP2.2 fails and Content Type is not Type1, HDCP1.4 will
	 * be attempted.
	 */
	ret = __wait_for(capable14 = rtk_hdcp_capable(hdmi),
				capable14 == -HDCP_PLUGOUT_EVENT,
				capable14 == 1,
				hdcp->hdcp14_timeout * 1000 * 1000,
				50 * 1000, 60 * 1000);
	if (ret == 0 && hdcp->content_type != DRM_MODE_HDCP_CONTENT_TYPE1)
		ret = _rtk_hdcp_enable(hdmi);
	else
		ret = -HDCP1_BKSV_ERROR;

	DRM_DEBUG_KMS("%s hdcp14 ret=%d\n", __func__, ret);
check_link:
	if (!ret) {
		schedule_delayed_work(&hdcp->check_work,
			msecs_to_jiffies(check_link_interval));
		hdcp->value = DRM_MODE_CONTENT_PROTECTION_ENABLED;
		schedule_work(&hdcp->prop_work);
	}
unlock:
	mutex_unlock(&hdcp->mutex);
	return ret;
}

int rtk_hdcp_disable(struct rtk_hdmi *hdmi)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;

	if (!hdcp || !hdcp->hdcp_ops)
		return -ENOENT;

	mutex_lock(&hdcp->mutex);

	if (hdcp->value != DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
		hdcp->value = DRM_MODE_CONTENT_PROTECTION_UNDESIRED;
		if (hdcp->hdcp2_encrypted)
			_rtk_hdcp2_disable(hdmi);
		else if (hdcp->hdcp_encrypted)
			_rtk_hdcp_disable(hdmi);
	}

	mutex_unlock(&hdcp->mutex);
	cancel_delayed_work_sync(&hdcp->check_work);

	return 0;
}

static void rtk_hdcp_check_work(struct work_struct *work)
{
	struct rtk_hdcp *hdcp = container_of(to_delayed_work(work),
					     struct rtk_hdcp,
					     check_work);
	struct rtk_hdmi *hdmi = container_of(hdcp, struct rtk_hdmi,
					     hdcp);

	if (!rtk_hdcp2_check_link(hdmi))
		schedule_delayed_work(&hdcp->check_work,
				      msecs_to_jiffies(DRM_HDCP2_CHECK_PERIOD_MS));
	else if (!rtk_hdcp_check_link(hdmi))
		schedule_delayed_work(&hdcp->check_work,
				      msecs_to_jiffies(DRM_HDCP_CHECK_PERIOD_MS));
}

static void rtk_hdcp_prop_work(struct work_struct *work)
{
	struct rtk_hdcp *hdcp = container_of(work, struct rtk_hdcp,
					     prop_work);
	struct rtk_hdmi *hdmi = container_of(hdcp, struct rtk_hdmi,
					     hdcp);
	struct drm_device *dev = hdmi->connector.dev;

	drm_modeset_lock(&dev->mode_config.connection_mutex, NULL);
	mutex_lock(&hdcp->mutex);

	/*
	 * This worker is only used to flip between ENABLED/DESIRED. Either of
	 * those to UNDESIRED is handled by core. If value == UNDESIRED,
	 * we're running just after hdcp has been disabled, so just exit
	 */
	if (hdcp->value != DRM_MODE_CONTENT_PROTECTION_UNDESIRED)
		drm_hdcp_update_content_protection(&hdmi->connector,
						   hdcp->value);

	mutex_unlock(&hdcp->mutex);
	drm_modeset_unlock(&dev->mode_config.connection_mutex);
}

int rtk_hdcp_init(struct rtk_hdmi *hdmi, u32 hdcp_support)
{
	struct rtk_hdcp *hdcp = &hdmi->hdcp;
	struct rtk_hdcp2_tee *hdcp2_tee = &hdcp->hdcp2_tee;
	struct rtk_hdcp1_tee *hdcp1_tee = &hdcp->hdcp1_tee;
	int ret;

	if(hdcp_support >= 2)
		hdcp->hdcp2_supported = true;
	else
		hdcp->hdcp2_supported = false;

	ret = drm_connector_attach_content_protection_property(&hdmi->connector,
							       hdcp->hdcp2_supported);
	if (ret) {
		hdcp->hdcp2_supported = false;
		return ret;
	}

	hdcp->hdcp14_timeout = 5;
	hdcp->hdcp14_timeout_property =
		drm_property_create_range(hdmi->drm_dev, 0, "hdcp14_timout", 0, 300);
	if (!hdcp->hdcp14_timeout_property) {
		dev_err(hdmi->dev, "create hdcp14_timeout_property failed");
		return -ENOMEM;
	}
	drm_object_attach_property(&hdmi->connector.base,
		hdcp->hdcp14_timeout_property, hdcp->hdcp14_timeout);

	hdcp->hdcp_encrypted = false;
	hdcp->hdcp2_encrypted = false;
	hdcp->hdcp_ops = &rtk_hdmi_hdcp_ops;
	rtk_hdcp1_tee_init(hdcp1_tee);
	rtk_hdcp2_tee_init(hdcp2_tee);
	mutex_init(&hdcp->mutex);
	INIT_DELAYED_WORK(&hdcp->check_work, rtk_hdcp_check_work);
	INIT_WORK(&hdcp->prop_work, rtk_hdcp_prop_work);

	return 0;
}

