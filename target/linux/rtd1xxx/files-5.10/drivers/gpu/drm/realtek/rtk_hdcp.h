/*
 * RealTek HDCP 1.4 and HDCP 2.2 include file
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _HDCP_H_INCLUDED_
#define _HDCP_H_INCLUDED_

#include "rtk_hdcp1_tee.h"
#include "rtk_hdcp2_tee.h"

/*No HDCP supported, no secure data path*/
#define WV_HDCP_NONE			0
/*HDCP version 1.0*/
#define WV_HDCP_V1			1
/*HDCP version 2.0 Type 1*/
#define WV_HDCP_V2			2
/*HDCP version 2.1 Type 1.*/
#define WV_HDCP_V2_1			3
/*HDCP version 2.2 Type 1.*/
#define WV_HDCP_V2_2			4
/*HDCP version 2.3 Type 1.*/
#define WV_HDCP_V2_3			5
/*No digital output*/
#define WV_HDCP_NO_DIGITAL_OUTPUT	0xff

#define HDCP_LINK_PROTECTED		0
#define HDCP_TOPOLOGY_CHANGE		1
#define HDCP_LINK_INTEGRITY_FAILURE	2
#define HDCP_REAUTH_REQUEST		3

#define HDCP_ENC_OFF 0
#define HDCP_ENC_ON 1

#define MAX_SHA_DATA_SIZE       645
#define MAX_SHA_VPRIME_SIZE     20

#define HDCP_PLUGOUT_EVENT      999

/* HDCP CTS 1A-07a, 4 seconds */
#define HDCP14_EN_TIMEOUT_MS 4500

#define __wait_for(OP, COND1, COND2, US, Wmin, Wmax) ({ \
	const ktime_t end__ = ktime_add_ns(ktime_get_raw(), 1000ll * (US)); \
	long wait__ = (Wmin); /* recommended min for usleep is 10 us */ \
	int ret__;                                                      \
	might_sleep();                                                  \
	for (;;) {                                                      \
		const bool expired__ = ktime_after(ktime_get_raw(), end__); \
		OP;                                                     \
		/* Guarantee COND check prior to timeout */             \
		barrier();                                              \
		if (COND1) {						\
			ret__ = -HDCP_PLUGOUT_EVENT;			\
			break;						\
		}							\
		if (COND2) {                                             \
			ret__ = 0;                                      \
			break;                                          \
		}                                                       \
		if (expired__) {                                        \
			ret__ = -ETIMEDOUT;                             \
			break;                                          \
		}                                                       \
		usleep_range(wait__, wait__ * 2);                       \
		if (wait__ < (Wmax))                                    \
			wait__ <<= 1;                                   \
	}                                                               \
	ret__;                                                          \
})

static inline unsigned long msecs_to_jiffies_timeout(const unsigned int m)
{
	unsigned long j = msecs_to_jiffies(m);

	return min_t(unsigned long, MAX_JIFFY_OFFSET, j + 1);
}

static inline void
wait_remaining_ms_from_jiffies(unsigned long timestamp_jiffies, int to_wait_ms)
{
	unsigned long target_jiffies, tmp_jiffies, remaining_jiffies;

	/*
	 * Don't re-read the value of "jiffies" every time since it may change
	 * behind our back and break the math.
	 */
	tmp_jiffies = jiffies;
	target_jiffies = timestamp_jiffies +
			 msecs_to_jiffies_timeout(to_wait_ms);

	if (time_after(target_jiffies, tmp_jiffies)) {
		remaining_jiffies = target_jiffies - tmp_jiffies;
		while (remaining_jiffies)
			remaining_jiffies =
			    schedule_timeout_uninterruptible(remaining_jiffies);
	}
}

struct rtk_hdmi;

struct rtk_hdcp_ops {
	 /* Outputs the transmitter's An and Aksv values to the receiver. */
	int (*write_an_aksv)(struct rtk_hdmi *hdmi,
			     u8 *an, u8 *aksv);

	/* Reads the receiver's key selection vector */
	int (*read_bksv)(struct rtk_hdmi *hdmi, u8 *bksv);

	/*
	 * Reads BINFO from DP receivers and BSTATUS from HDMI receivers. The
	 * definitions are the same in the respective specs, but the names are
	 * different. Call it BSTATUS since that's the name the HDMI spec
	 * uses and it was there first.
	 */
	int (*read_bstatus)(struct rtk_hdmi *hdmi,
				u8 *bstatus);

	/* Determines whether a repeater is present downstream */
	int (*repeater_present)(struct rtk_hdmi *hdmi,
				bool *repeater_present);

	/* Reads the receiver's Ri' value */
	int (*read_ri_prime)(struct rtk_hdmi *hdmi, u8 *ri);

	/* Determines if the receiver's KSV FIFO is ready for consumption */
	int (*read_ksv_ready)(struct rtk_hdmi *hdmi,
				bool *ksv_ready);

	/* Reads the ksv fifo for num_downstream devices */
	int (*read_ksv_fifo)(struct rtk_hdmi *hdmi,
				int num_downstream, u8 *ksv_fifo);

	/* Reads a 32-bit part of V' from the receiver */
	int (*read_v_prime_part)(struct rtk_hdmi *hdmi,
				int i, u32 *part);

	/* Ensures the link is still protected */
	int (*check_link)(struct rtk_hdmi *hdmi);

	/* Detects panel's hdcp capability. This is optional for HDMI. */
	int (*hdcp_capable)(struct rtk_hdmi *hdmi,
				bool *hdcp_capable);

	/* Detects panel's hdcp capability. This is optional for HDMI. */
	int (*hdcp2_capable)(struct rtk_hdmi *hdmi,
				bool *hdcp2_capable);

	/* Write HDCP2.2 messages */
	int (*write_2_2_msg)(struct rtk_hdmi *hdmi,
				void *buf, size_t size);

	/* Read HDCP2.2 messages */
	int (*read_2_2_msg)(struct rtk_hdmi *hdmi,
				u8 msg_id, void *buf, size_t size);

	/* HDCP2.2 Link Integrity Check */
	int (*check_2_2_link)(struct rtk_hdmi *hdmi);
};

struct rtk_hdcp {
	const struct rtk_hdcp_ops *hdcp_ops;
	/* Mutex for hdcp state of the connector */
	struct mutex mutex;
	u64 value;
	struct delayed_work check_work;
	struct work_struct prop_work;

	/* HDCP1.4 Encryption status */
	bool hdcp_encrypted;

	/* HDCP2.2 related definitions */
	/* Flag indicates whether this connector supports HDCP2.2 or not. */
	bool hdcp2_supported;

	/* HDCP2.2 Encryption status */
	bool hdcp2_encrypted;

	/*
	 * Content Stream Type defined by content owner. TYPE0(0x0) content can
	 * flow in the link protected by HDCP2.2 or HDCP1.4, where as TYPE1(0x1)
	 * content can flow only through a link protected by HDCP2.2.
	 */

	u8 content_type;
	bool is_paired;
	bool is_repeater;

	/*
	 * Count of RepeaterAuth_Stream_Manage msg propagated.
	 * Initialized to 0 on AKE_INIT. Incremented after every successful
	 * transmission of RepeaterAuth_Stream_Manage message. When it rolls
	 * over re-Auth has to be triggered.
	 */
	u32 seq_num_m;
	u8 hdcp1_device_downstream;
	u8 mCap[1];
	u8 r_rx[HDCP_2_2_RRX_LEN];
	u8 r_tx[HDCP_2_2_RTX_LEN];
	u8 rx_caps[HDCP_2_2_RXCAPS_LEN];


	struct hdcp2_tx_caps    tx_caps;

	/* timeout value of HDCP1.4 retry */
	u32 hdcp14_timeout;
	struct drm_property *hdcp14_timeout_property;

	struct rtk_hdcp1_tee hdcp1_tee;
	struct rtk_hdcp2_tee hdcp2_tee;
};

int rtk_hdcp_init(struct rtk_hdmi *hdmi, u32 hdcp_support);
int rtk_hdcp_enable(struct rtk_hdmi *hdmi, u8 content_type);
int rtk_hdcp_disable(struct rtk_hdmi *hdmi);
void rtk_hdcp_update_mCap(struct rtk_hdmi *hdmi, u8 mCap);

#endif
