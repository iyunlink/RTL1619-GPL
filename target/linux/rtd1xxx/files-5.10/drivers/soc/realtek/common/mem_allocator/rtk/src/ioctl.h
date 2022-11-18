/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef _LINUX_ION_RTK_IOCTL_H
#define _LINUX_ION_RTK_IOCTL_H

#include <linux/file.h>

long ion_rtk_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#endif /* _LINUX_ION_RTK_IOCTL_H */
