/*
 * RealTek HDCP 2.2 CA include file
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _HDCP2_TEE_H_INCLUDED_
#define _HDCP2_TEE_H_INCLUDED_


#include <linux/tee_drv.h>
#include <uapi/linux/tee.h>
#include <drm/drm_hdcp.h>

/*hdcp 2.2 error code*/
#define HDCP2_SUCCESS                                  ( 0)
#define HDCP2_PASSKEY_ERROR                            ( 1)
#define HDCP2_READKEY_ERROR                            ( 2)
#define HDCP2_READCERT_TIMEOUT                         ( 3)
#define HDCP2_READCERT_ID_ERROR                        ( 4)
#define HDCP2_LLC_SIGNATURE_ERROR                      ( 5)
#define HDCP2_READCERT_ERROR                           ( 6)
#define HDCP2_READHPRIME_TIMEOUT                       ( 7)
#define HDCP2_READHPRIME_ID_ERROR                      ( 8)
#define HDCP2_COMPARE_H_ERROR                          ( 9)
#define HDCP2_READPAIRING_TIMEOUT                      (10)
#define HDCP2_READPAIRING_ID_ERROR                     (11)
#define HDCP2_READPAIRING_ERROR                        (12)
#define HDCP2_READ_HPRIME_ERROR                        (13)
#define HDCP2_READLPRIME_TIMEOUT                       (14)
#define HDCP2_READLPRIME_ID_ERROR                      (15)
#define HDCP2_COMPARE_L_ERROR                          (16)
#define HDCP2_READ_LPRIME_ERROR                        (17)
#define HDCP2_SENDEKS_ERROR                            (18)
#define HDCP2_READRECEIVEIDLIST_TIMEOUT                (19)
#define HDCP2_READRECEIVEIDLIST_ID_ERROR               (20)
#define HDCP2_COMPARE_V_ERROR                          (21)
#define HDCP2_READRECEIVEIDLIST_ERROR                  (22)
#define HDCP2_SENDSTREAM_MANAGE_ERROR                  (23)
#define HDCP2_READSTREAMREADY_TIMEOUT                  (24)
#define HDCP2_COMPARE_M_ERROR                          (25)
#define HDCP2_POLLING_NO_READRXSTATUS                  (26)
#define HDCP2_POLLING_READRECEIVEIDLIST_ID_ERROR       (27)
#define HDCP2_POLLING_COMPARE_V_ERROR                  (28)
#define HDCP2_POLLING_READRECEIVEIDLIST_ERROR          (29)
#define HDCP2_POLLING_STATE_EXIT                       (30)
#define HDCP2_POLLING_I2C_ERROR                        (31)
#define HDCP2_REAUTH_ERROR                             (32)
#define HDCP2_REPEATER_REAUTH_ERROR                    (33)
#define HDCP2_READ_M_ID_ERROR                          (34)

struct rtk_hdcp2_tee;

struct rtk_hdcp2_tee_ops {
	void (*hdcp2_tee_api_init)(struct rtk_hdcp2_tee *hdcp2_tee);
	void (*hdcp2_tee_api_deinit)(struct rtk_hdcp2_tee *hdcp2_tee);
	void (*generate_random_rtx)(struct rtk_hdcp2_tee *hdcp2_tee,
				    struct hdcp2_ake_init *ake_data);
	int (*verify_rx_cert)(struct rtk_hdcp2_tee *hdcp2_tee,
			      struct hdcp2_ake_send_cert *rx_cert);
	void (*prepare_stored_km)(struct rtk_hdcp2_tee *hdcp2_tee,
				  struct hdcp2_ake_no_stored_km *ek_pub_km);
	void (*prepare_no_stored_km)(struct rtk_hdcp2_tee *hdcp2_tee,
				     struct hdcp2_ake_no_stored_km *ek_pub_km);
	int (*verify_hprime)(struct rtk_hdcp2_tee *hdcp2_tee,
			     struct hdcp2_ake_send_hprime *rx_hprime,
			     u8 *verified_src, u8 *h);
	void (*check_stored_km)(struct rtk_hdcp2_tee *hdcp2_tee,
				u8 *receiver_id,
				struct hdcp2_ake_no_stored_km *e_kh_km_m,
				bool *km_stored);
	int (*store_pairing_info)(struct rtk_hdcp2_tee *hdcp2_tee,
				  struct hdcp2_ake_send_pairing_info *pairing_info);
	void (*initiate_locality_check)(struct rtk_hdcp2_tee *hdcp2_tee,
					struct hdcp2_lc_init *lc_init);
	int (*verify_lprime)(struct rtk_hdcp2_tee *hdcp2_tee,
			     struct hdcp2_lc_send_lprime *rx_lprime,
			     u8 *lprime);
	void (*get_session_key)(struct rtk_hdcp2_tee *hdcp2_tee,
				struct hdcp2_ske_send_eks *ske_data);
	int (*repeater_check_flow_prepare_ack)(struct rtk_hdcp2_tee *hdcp2_tee,
					       u8 *buf,
					       int msg_size,
					       u8 *mV);
	int (*verify_mprime)(struct rtk_hdcp2_tee *hdcp2_tee,
			     struct hdcp2_rep_stream_ready *stream_ready,
			     u8 *input, int input_size);
	/* Enables HDCP signalling on the port */
	void (*enable_hdcp2_cipher)(struct rtk_hdcp2_tee *hdcp2_tee,
				    u8 *mCap);
	void (*disable_hdcp2_cipher)(struct rtk_hdcp2_tee *hdcp2_tee);
	void (*update_mCap)(struct rtk_hdcp2_tee *hdcp2_tee,
			    u8 *mCap);
	int (*read_hdcp2_key)(struct rtk_hdcp2_tee *hdcp2_tee);
	int (*write_hdcp2_key)(struct rtk_hdcp2_tee *hdcp2_tee,
			       char *key, unsigned int keyLength);
	void (*clear_hdcp2_cipher_setting)(struct rtk_hdcp2_tee *hdcp2_tee);
};

struct rtk_hdcp2_tee {
	const struct rtk_hdcp2_tee_ops *hdcp2_tee_ops;

	int init_hdcp2_ta_flag;
	struct tee_ioctl_open_session_arg hdcp2_arg;
	struct tee_context *hdcp2_ctx;
};

void rtk_hdcp2_tee_init(struct rtk_hdcp2_tee *hdcp2_tee);
#endif
