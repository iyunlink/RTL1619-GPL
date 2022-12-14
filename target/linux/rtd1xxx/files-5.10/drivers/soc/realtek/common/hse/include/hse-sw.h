/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef __HSE_SW_H__
#define __HSE_SW_H__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

void __hse_sw_yuyv2nv12(void *dst0,
			void *dst1,
			void *src,
			uint32_t width,
			uint32_t height,
			uint32_t src_pitch,
			uint32_t dst_pitch);

void __hse_sw_rotate(void *src,
		     void *dst,
		     uint32_t mode,
		     uint32_t width,
		     uint32_t height,
		     uint32_t src_pitch,
		     uint32_t dst_pitch);

#endif /*__HSE_SW_H__ */
