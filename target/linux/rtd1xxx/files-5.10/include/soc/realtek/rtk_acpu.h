/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef __SOC_REALTEK_ACPU_H
#define __SOC_REALTEK_ACPU_H

#ifdef CONFIG_RTK_ACPU_VE1
int rtk_acpu_release_ve1(void);
#else

static inline int rtk_acpu_release_ve1(void)
{
	return -EINVAL;
}

#endif

#endif
