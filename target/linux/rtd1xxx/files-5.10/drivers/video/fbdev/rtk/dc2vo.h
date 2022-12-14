// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek framebuffer driver
 *
 * Copyright (c) 2019-2020 Realtek Semiconductor Corp.
 */

#ifndef __DC_HW_H__
#define __DC_HW_H__

#include <linux/fb.h>
#include "dc_rpc.h"
#include "rtk_fb.h"

#include <trace/events/rtk_rpc.h>

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct DCRT_PARAM_BUF_ADDR
{
	unsigned int buf_Vaddr;
	unsigned int buf_Paddr;
	int buf_id;
	unsigned int buf_size;
	int format;
	unsigned int width;
	unsigned int height;
	int param_size;	/* fix to [7] position */
} DCRT_PARAM_BUF_ADDR;

typedef struct DCRT_PARAM_OSD_ADDR
{
	int param_size;
	unsigned int buf_Paddr;
	int go;
} DCRT_PARAM_OSD_ADDR;

typedef struct DCRT_PARAM_RPC_VIRT_ADDR
{
    void* ringheaderbaseVirtAddr;
    void* ringVirtAddr;
    void* refclockVirtAddr;
    unsigned int vo_instance_id;
} DCRT_PARAM_RPC_VIRT_ADDR;

typedef struct DCRT_PARAM_RPC_ADDR
{
	unsigned int ringPhyAddr;
	unsigned int refclockAddr;
	int param_size;	/* [2]-th pos */

} DCRT_PARAM_RPC_ADDR;

typedef struct 
{
	int param_size;
	unsigned int pts_gap;
	unsigned int hz; /* 50000, 60000 HZ ? */
	unsigned int clockAddrLow;
	unsigned int clockAddrHi;

} DCRT_PARAM_RATE_INFO;

typedef struct 
{
	int param_size;
	unsigned int interval;
	unsigned int rateMode;

} DCRT_PARAM_RATE_MODE;

typedef struct 
{
	int param_size;
	unsigned int mode;

} DCRT_PARAM_DISABLE_MODE;

typedef struct
{
	int ver;
	int param_size;
	unsigned int buf_Vaddr;
	unsigned int buf_Paddr;
	unsigned int buf_size;
	int width;
	int height;
	int x;
	int y;
} DCRT_PARAM_SURFACE;

typedef enum
{
	DC_RT_FRAME_DISP_SYNC= 0, /* show last pic only */
	DC_RT_FRAME_DISP_ALL,
} DC_RT_FRAME_DISP_MODE; 

typedef enum
{
	GLDC_MODIFY_RESET = 1 << 1,
	GLDC_MODIFY_ALPHA = 1 << 2,
	GLDC_MODIFY_PLANE = 1 << 3,
	GLDC_MODIFY_MODE_3D = 1 << 4,
	GLDC_MODIFY_DISP_WIN = 1 << 5,
	GLDC_MODIFY_BLENDING = 1 << 6
} GLDC_CONFIG_MASK;

typedef enum
{
	GLDC_BLEND_MODE_OVERLAY = 0,
	GLDC_BLEND_MODE_BLENDING = 1 /* default */

} GLDC_OSD_BLEND_MODE;

enum VO_GRAPHIC_PLANE {
	VO_GRAPHIC_OSD = 0,
	VO_GRAPHIC_SUB1 = 1,
	VO_GRAPHIC_SUB2 = 2,
};
typedef enum VO_GRAPHIC_PLANE VO_GRAPHIC_PLANE;

struct VO_RECTANGLE {
	u_short x;
	u_short y;
	u_short width;
	u_short height;
};
typedef struct VO_RECTANGLE VO_RECTANGLE;

typedef struct 
{
	unsigned int cfg;
	VO_GRAPHIC_PLANE plane;
	VO_RECTANGLE dispWin;
	unsigned int alpha; /* valid value: 0~254 */
	int mode3D; /* 0: disable (INBAND_CMD_GRAPHIC_3D_MODE) */
	char blendMode; /* 1: blending, 0: overlay */
	char rev[31]; /* reserve */

} GLDC_MODIFY_COMMAND;

typedef struct{
	REFCLOCK RefClock;
	unsigned long long PTS;
	unsigned int CTX;
	unsigned int OSD2VO_Delay_us;
	unsigned int FPS;
	unsigned int RATE_GAP;
	unsigned long long RefClockAddr;
	unsigned long long ClockAddr_HI;
	unsigned long long ClockAddr_LOW;
	long long RTK90KClock;
	int WAIT_ISR;
	unsigned int FlipTimeTemp;
} DC_SYSTEM_TIME_INFO;

typedef struct{
	unsigned int HiOffset;
	unsigned int LowOffset;
} DC_CLOCK_MAP_INFO;

typedef enum {
	ResetOption_videoSystemPTS = 0x1 << 0,
	ResetOption_audioSystemPTS = 0x1 << 1,
	ResetOption_videoRPTS = 0x1 << 2,
	ResetOption_audioRPTS = 0x1 << 3,
	ResetOption_videoContext = 0x1 << 4,
	ResetOption_audioContext = 0x1 << 5,
	ResetOption_videoEndOfSegment = 0x1 << 6,
	ResetOption_RCD = 0x1 << 7,
}ResetOption;

typedef enum
{ 
	INBAND_CMD_GRAPHIC_2D_MODE = 0,
	INBAND_CMD_GRAPHIC_SIDE_BY_SIDE = 1,
	INBAND_CMD_GRAPHIC_TOP_AND_BOTTOM = 2,
	INBAND_CMD_GRAPHIC_FRAME_PACKING = 3
} INBAND_CMD_GRAPHIC_3D_MODE ;

typedef struct
{
	INBAND_CMD_PKT_HEADER header;
	INBAND_CMD_GRAPHIC_FORMAT format;
	unsigned int PTSH;
	unsigned int PTSL;
	unsigned int context; /* system can check it to know which picture displaying on VO */
	int colorkey; /* -1: disable colorkey */
	int alpha; /* 0: disable constant alpha */
	unsigned int x;
	unsigned int y;
	unsigned int width;
	unsigned int height;
	unsigned int address;
	unsigned int pitch;
	unsigned int address_right;
	unsigned int pitch_right;
	INBAND_CMD_GRAPHIC_3D_MODE picLayout;
	unsigned int afbc;
	unsigned int afbc_block_split;
	unsigned int afbc_yuv_transform;

} VIDEO_GRAPHIC_PICTURE_OBJECT;

typedef struct
{
	INBAND_CMD_PKT_HEADER header;
	unsigned int version;
	INBAND_CMD_GRAPHIC_FORMAT format;
	unsigned int PTSH;
	unsigned int PTSL;
	unsigned int context; /* system can check it to know which picture displaying on VO */
	int colorkey; /* -1: disable colorkey */
	int alpha; /*  0: disable constant alpha */
	unsigned int x;
	unsigned int y;
	unsigned int width;
	unsigned int height;
	unsigned int address;
	unsigned int pitch;
	unsigned int address_right;
	unsigned int pitch_right;
	INBAND_CMD_GRAPHIC_3D_MODE picLayout;
	unsigned int afbc;
	unsigned int afbc_block_split;
	unsigned int afbc_yuv_transform;
	unsigned int partialSrcWin_x;
	unsigned int partialSrcWin_y;
	unsigned int partialSrcWin_w;
	unsigned int partialSrcWin_h;
} VIDEO_GRAPHIC_PICTURE_OBJECT_VERSION;

#define RTK_VERSION_0 0x72746B30

typedef struct {
	int sfd_refclk;
	int sfd_rbHeader;
	int sfd_rbBase;
	long long vo_instance_id;
	unsigned int reserve[16-5];
} DC_ION_SHARE_MEMORY ;

typedef struct {
	long long vo_instance_id;
	unsigned int reserve[16-2];
} DC_VO_INSTANCE_INFO;

typedef struct {
	unsigned int enable ;
	unsigned int width ;
	unsigned int height ;
	unsigned int reserve[16-3];
} DC_BUFFER_INFO ;

#define DC_SHORT_FENCE_TIMEOUT (60)
#define DC_LONG_FENCE_TIMEOUT (120)

enum dc_buffer_id {
	eFrameBuffer = 0x1U << 0,
	eIONBuffer = 0x1U << 1,
	eUserBuffer = 0x1U << 2,
	eFrameBufferTarget = 0x1U << 3,
	eFrameBufferPartial = 0x1U << 4,
	eFrameBufferSkip = 0x1U << 5,
};

enum dc_overlay_engine {
	eEngine_VO = 0x1U << 0,
	eEngine_SE = 0x1U << 1,
	eEngine_DMA = 0x1U << 2,
	eEngine_MAX = 0x1U << 3,
};

struct dc_buffer_rect {
	u32 left;
	u32 top;
	u32 right;
	u32 bottom;
};

union _fence_data{
	int64_t fence_fd; /* user   space : fenceFd (input) */
	struct sync_file *fence; /* kernel space : sync_file to wait fence sync */
};

enum dc_buffer_flags {
	eBuffer_AFBC_Enable = 0x1U << 16,
	eBuffer_AFBC_Split  = 0x1U << 17,
	eBuffer_AFBC_YUV_Transform = 0x1U << 18,
	eBuffer_USE_GLOBAL_ALPHA = 0x1U << 19,
};

struct dc_buffer {
	u32 id; /* enum dc_buffer_id */
	u32 overlay_engine; /* enum dc_overlay_engine */
	struct dc_buffer_rect sourceCrop;
	struct dc_buffer_rect displayFrame; /* base on framebuffer */

	u32 format;
	u32 offset;
	u32 phyAddr;
	u32 width;
	u32 height;
	u32 stride;
	u32 context;
	int64_t pts;
	u32 flags;
	u32 alpha; /* need to enable eBuffer_USE_GLOBAL_ALPHA, 0 : Pixel Alpha */
	u32 partial_x;
	u32 partial_y;
	u32 partial_w;
	u32 partial_h;
	u32 reserve[27];

	union _fence_data acquire;
};

#if 0
struct dc_buffer_config {
	struct dc_buffer buffer;
};
#endif

struct dc_post {
	u32 n_bufs;
	struct dc_buffer *bufs;
};

struct dc_pending_post {
	struct list_head head;
	struct dc_post config;
	union _fence_data release;
};

//TODO::
#if 0
struct dc_post_config {
	size_t n_bufs;
	struct dc_buffer_config __user *bufs;
	__s64 complete_fence;
};
#endif

struct dc_simple_post_config {
	struct dc_buffer buf;
	int complete_fence_fd;
};

/*
 * Ioctl
 */
#define DC2VO_IOC_MAGIC 'd'
#define DC2VO_IOC_MAXNR 32
#define DC2VO_WAIT_FOR_VSYNC _IO (DC2VO_IOC_MAGIC, 0)
#define DC2VO_SET_BUFFER_ADDR _IO (DC2VO_IOC_MAGIC, 1)
#define DC2VO_GET_BUFFER_ADDR _IO (DC2VO_IOC_MAGIC, 2)
#define DC2VO_SET_RING_INFO _IO	(DC2VO_IOC_MAGIC, 3)
#define DC2VO_SET_OUT_RATE_INFO _IO (DC2VO_IOC_MAGIC, 4)
#define DC2VO_SET_DISABLE _IO (DC2VO_IOC_MAGIC, 5)
#define DC2VO_GET_SURFACE _IO (DC2VO_IOC_MAGIC, 6)
#define DC2VO_SET_MODIFY _IO (DC2VO_IOC_MAGIC, 7)
#define DC2VO_GET_MAX_FRAME_BUFFER _IO (DC2VO_IOC_MAGIC, 8)
#define DC2VO_GET_SYSTEM_TIME_INFO _IO (DC2VO_IOC_MAGIC, 9)
#define DC2VO_SET_SYSTEM_TIME_INFO _IO (DC2VO_IOC_MAGIC, 10)
#define DC2VO_GET_CLOCK_MAP_INFO _IO (DC2VO_IOC_MAGIC, 11)
#define DC2VO_GET_CLOCK_INFO _IO (DC2VO_IOC_MAGIC, 12)
#define DC2VO_RESET_CLOCK_TABLE _IO (DC2VO_IOC_MAGIC, 13)
#define DC2VO_SET_ION_SHARE_MEMORY _IO (DC2VO_IOC_MAGIC, 17)
#define DC2VO_SET_BUFFER_INFO _IO (DC2VO_IOC_MAGIC, 18)
#define DC2VO_SET_BG_SWAP _IO (DC2VO_IOC_MAGIC, 19)
#define DC2VO_SET_GLOBAL_ALPHA _IO (DC2VO_IOC_MAGIC, 20)
#define DC2VO_SET_VSYNC_FORCE_LOCK _IO(DC2VO_IOC_MAGIC, 21)
#define DC2VO_SIMPLE_POST_CONFIG _IO (DC2VO_IOC_MAGIC, 22)
#define DC2VO_GET_VO_INSTANCE_INFO _IO (DC2VO_IOC_MAGIC, 23)

/*
 * legacy : DCRT_IRQ, DC_NOHW_MAX_BACKBUFFERS
 */
#define DCRT_IRQ (65) /* AVCPU_NWC : AVCPU => SCPU */
#define DC_NOHW_MAX_BACKBUFFERS (4)

#define DC_VO_SET_NOTIFY (__cpu_to_be32(1U << 0)) /* SCPU write */
#define DC_VO_FEEDBACK_NOTIFY (__cpu_to_be32(1U << 1))
#define RPC_AUDIO_SET_NOTIFY (__cpu_to_be32(1U << 8)) /* SCPU write */
#define RPC_AUDIO_FEEDBACK_NOTIFY (__cpu_to_be32(1U << 9))

#define VO_DC_SET_NOTIFY (__cpu_to_be32(1U << 16)) /* ACPU write */
#define VO_DC_FEEDBACK_NOTIFY (__cpu_to_be32(1U << 17))
#define AUDIO_RPC_SET_NOTIFY (__cpu_to_be32(1U << 24)) /* ACPU write */
#define AUDIO_RPC_FEEDBACK_NOTIFY (__cpu_to_be32(1U << 25))

#define DC_HAS_BIT(addr, bit) (readl(addr) & bit)
#define DC_SET_BIT(addr,bit) (writel((readl(addr)|bit), addr))
#define DC_RESET_BIT(addr,bit) (writel((readl(addr)&~bit), addr))

#define ACPU_INT_SA (1 << 1)
#define ACPU_INT_WRITE 0x1
#define ACPU_INT_BASE 0x0


int dc_ioctl (struct fb_info *fbi, struct rtk_dc_info *video_info,unsigned int cmd, unsigned long arg);
int dc_swap_buffer (struct fb_info *fbi, struct rtk_dc_info *video_info);
int dc_init (struct rtk_dc_info *video_info, struct fb_info *fbi, int irq);
void dc_deinit (struct rtk_dc_info *video_info);
int dc_suspend (struct rtk_dc_info *video_info);
int dc_resume (struct rtk_dc_info *video_info);
int dc_avcpu_event_notify (unsigned long action, struct rtk_dc_info *video_info);
struct sync_file *DC_QueueBuffer(struct dc_buffer *buf);
int DC_VsyncWait(unsigned long long *nsecs);
int dc_set_skip (struct rtk_dc_info *video_info, int cnt);

/* Offer function to rtk_fb.c */
int DC_Set_RateInfo (struct fb_info *fb, struct rtk_dc_info * video_info,DCRT_PARAM_RATE_INFO* param);
int DC_Set_ION_Share_Memory(struct fb_info *fb, struct rtk_dc_info *video_info,DC_ION_SHARE_MEMORY *param);
int DC_Set_RPCAddr_Virt(struct fb_info *fb, struct rtk_dc_info * video_info, DCRT_PARAM_RPC_VIRT_ADDR* param);
int DC_Set_RPCAddr(struct fb_info *fb, struct rtk_dc_info * video_info, DCRT_PARAM_RPC_ADDR* param);
int dc_do_static_post_config(struct rtk_dc_info * video_info);

#if defined(__cplusplus)
}
#endif

#endif
