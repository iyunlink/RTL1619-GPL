// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek framebuffer driver
 *
 * Copyright (c) 2019-2020 Realtek Semiconductor Corp.
 */

#ifndef __RTK_FB_H__
#define __RTK_FB_H__

#include <linux/fb.h>

struct rtk_dc_info {
	void *dc_info;
	struct regmap *acpu_int_base;
};

struct fb_dmabuf_export {
	__u32 fd;
	__u32 flags;
};
#define FBIOGET_DMABUF  _IOR('F', 0x21, struct fb_dmabuf_export)

struct fb_resize {
	__u32 width;
	__u32 height;
	__u32 count;
	__u32 pixel;
	__u32 flags;
    __u32 reserve[16-5];
};
#define FBIOSET_RESIZE _IOWR('F', 0x22, struct fb_resize)

#define DEFAULT_FPS 60
#define DEFAULT_WIDTH 1280
#define DEFAULT_HEIGHT 768
#define DEFAULT_COUNT 3
#define DEFAULT_OSD_INIT 0

#define BYTES_PER_PIXEL 4

#define MEMORY_ALIGN(value, base) (((value) + ((base) - 1)) & ~((base) - 1))

struct rtk_fb_mem {
	int handle;
	int width;
	int height;
	int align;
	int count;
	int pixel;
	int flags;
	size_t size;
	phys_addr_t pa_addr;
	void *va_addr;
};

struct rtk_fb {
	struct device *dev;
	struct fb_info fb;
	struct rtk_fb_mem *mem;
	struct rtk_dc_info video_info;
	struct dma_buf *fb_dmabuf;
	struct sg_table *sgt;
	struct dma_buf_attachment *attachment;
	u32 cmap[16];
	int rotation;
	int fps;
	int irq;
};

enum rtk_fb_mem_flags {
	RTK_FB_MEM_ALLOC_ALGO_LAST_FIT = 0x1 << 0,
	RTK_FB_MEM_FREE_FB = 0x1 << 1,
};

int rtk_fb_ion_fd(struct rtk_fb *fb);
int rtk_fb_config(struct rtk_fb *fb);

extern int dc_ioctl(struct fb_info *fb,
	struct rtk_dc_info *video_info, unsigned int cmd, unsigned long arg);
extern int dc_init(struct rtk_dc_info *video_info,
	struct fb_info *fbi, int irq);
extern int dc_swap_buffer(struct fb_info *fbi,
	struct rtk_dc_info *video_info);
extern void dc_deinit(struct rtk_dc_info *video_info);
extern int dc_suspend(struct rtk_dc_info *video_info);
extern int dc_resume(struct rtk_dc_info *video_info);
extern int dc_avcpu_event_notify(unsigned long action,
	struct rtk_dc_info *video_info);
extern int dc_vsync_wait(unsigned long long *nsecs);
extern int dc_set_skip (struct rtk_dc_info *video_info, int cnt);
extern int dc2vo_sysfs_init(void);

#endif
