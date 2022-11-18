/*
 * compat_hdmitx.h - RTK hdmitx driver header file
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _LINUX_COMPAT_HDMITX_H
#define _LINUX_COMPAT_HDMITX_H

#include "hdmitx.h"

#if IS_ENABLED(CONFIG_COMPAT)

struct block_edid32 {
	unsigned char block_index;
	unsigned char block_size;
	compat_caddr_t edid_ptr;
} __attribute__ ((packed));

struct fake_edid32 {
	compat_caddr_t data_ptr;
	unsigned int size;
};

#define HDMI_GET_EDID_BLOCK32           _IOWR(HDMI_IOCTL_MAGIC, 23, struct block_edid32)
#define HDMI_SET_FAKE_EDID32            _IOWR(HDMI_IOCTL_MAGIC, 26, struct fake_edid32)

long rtk_compat_hdmitx_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

#endif /* CONFIG_COMPAT */
#endif /* _LINUX_COMPAT_HDMITX_H */
