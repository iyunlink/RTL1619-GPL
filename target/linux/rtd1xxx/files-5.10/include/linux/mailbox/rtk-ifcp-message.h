/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_REALTEK_IFCP_MESSAGE_H_
#define _LINUX_REALTEK_IFCP_MESSAGE_H_

#define IFCP_MAX_DATA    0x40

struct rtk_ifcp_message {
	unsigned int len;
	unsigned int data[IFCP_MAX_DATA];
};

#endif
