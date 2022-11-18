// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek framebuffer driver
 *
 * Copyright (c) 2019-2020 Realtek Semiconductor Corp.
 */

#ifndef RTK_FB_RPC_H
#define RTK_FB_RPC_H

#include <linux/dma-buf.h>
#if defined(CONFIG_ARCH_RTD129x) || defined(CONFIG_ARCH_RTD119X)
#define CONVERT_FOR_AVCPU(x)	((unsigned int)(x) | 0xA0000000)
#else
#define CONVERT_FOR_AVCPU(x)	(x)
#endif

#define AUDIO_ION_FLAG		(ION_FLAG_NONCACHED |ION_FLAG_SCPUACC | ION_FLAG_ACPUACC)

struct rtk_osd_priv {
	struct dma_buf *osd_dma_buf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	void *ion_virt;
	struct device *dev;
};

int RTK_FB_RPC_OSD_init(struct rtk_fb *fb);
void RTK_FB_RPC_OSD_uninit(void);

#endif
