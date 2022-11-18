// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 */

#ifndef __RTK_CEC__H__
#define __RTK_CEC__H__

#define MAX_OSD_NAME 32

struct ipc_shm_cec {
	unsigned int  standby_config;
	unsigned char  standby_logical_addr;
	unsigned short standby_physical_addr;
	unsigned char  standby_cec_version;
	unsigned int  standby_vendor_id;
	unsigned short standby_rx_mask;
	unsigned char  standby_cec_wakeup_off;
	unsigned char  standby_osd_name[MAX_OSD_NAME];
};

#endif
