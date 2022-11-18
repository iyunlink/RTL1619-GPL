/*
 * RealTek HDCP 1.4 CA include file
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _HDCP1_TEE_H_INCLUDED_
#define _HDCP1_TEE_H_INCLUDED_


#include <linux/tee_drv.h>
#include <uapi/linux/tee.h>
#include <drm/drm_hdcp.h>

/*hdcp 1.4 error code*/
#define HDCP1_SUCCESS		0
#define HDCP1_DDC_ERROR		1
#define HDCP1_AUTH_FAILURE	2
#define HDCP1_AKSV_ERROR	3
#define HDCP1_3DES_ERROR	4
#define HDCP1_SHA1_ERROR	5
#define HDCP1_DRIVER_ERROR	6
#define HDCP1_CANCELLED_AUTH	7
#define HDCP1_GEN_AN_ERROR	8
#define HDCP1_BKSV_ERROR	9
#define HDCP1_R0_ERROR		10
#define HDCP1_BCAPS_RDY_ERROR	11
#define HDCP1_RI_ERROR		12
#define HDCP1_PASSKEY_ERROR	13
#define HDCP1_UNKNOWN_STATE	0xFF

#define HDCP14_KEY_SIZE		288

struct rtk_hdcp1_tee;

struct rtk_hdcp1_tee_ops {
	void (*hdcp1_tee_api_init)(struct rtk_hdcp1_tee *hdcp1_tee);
	void (*hdcp1_tee_api_deinit)(struct rtk_hdcp1_tee *hdcp1_tee);
	int (*generate_an)(struct rtk_hdcp1_tee *hdcp1_tee, u8 *an);
	int (*read_aksv)(struct rtk_hdcp1_tee *hdcp1_tee, u8 *aksv);
	void (*set_hdcp1_repeater_bit)(struct rtk_hdcp1_tee *hdcp1_tee,
				       u8 is_repeater);
	int (*write_bksv)(struct rtk_hdcp1_tee *hdcp1_tee, u8 *bksv);
	int (*check_ri_prime)(struct rtk_hdcp1_tee *hdcp1_tee, u8 *ri);
	void (*hdcp1_set_encryption)(struct rtk_hdcp1_tee *hdcp1_tee,
				    u8 enc_state);
	void (*sha_append_bstatus_m0)(struct rtk_hdcp1_tee *hdcp1_tee,
				      u8 *ksv_fifo,
				      int *byte_cnt,
				      u8 *bstatus);
	int (*compute_V)(struct rtk_hdcp1_tee *hdcp1_tee,
			 u8 *ksv_fifo,
			 int *byte_cnt);
	int (*verify_V)(struct rtk_hdcp1_tee *hdcp1_tee, u8 *vprime);
	void (*set_wider_window)(struct rtk_hdcp1_tee *hdcp1_tee);
	int (*write_hdcp1_key)(struct rtk_hdcp1_tee *hdcp1_tee,
			       unsigned char *key);
	int (*fix480p)(struct rtk_hdcp1_tee *hdcp1_tee);
	void (*set_keepout_win)(struct rtk_hdcp1_tee *hdcp1_tee);
};

struct rtk_hdcp1_tee {
	const struct rtk_hdcp1_tee_ops *hdcp1_tee_ops;

	int init_hdcp1_ta_flag;
	struct tee_ioctl_open_session_arg hdcp1_arg;
	struct tee_context *hdcp1_ctx;
};

void rtk_hdcp1_tee_init(struct rtk_hdcp1_tee *hdcp1_tee);

#endif
