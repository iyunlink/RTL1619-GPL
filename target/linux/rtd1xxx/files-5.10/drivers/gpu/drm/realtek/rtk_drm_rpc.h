/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019 RealTek Inc.
 */

#ifndef _RTK_DRM_RPC_H
#define _RTK_DRM_RPC_H

#include <linux/kernel.h>
#include <linux/dma-buf.h>
#include <soc/realtek/rtk_ipc_shm.h>
#include <soc/realtek/kernel-rpc.h>

#define S_OK 0x10000000

#define RPC_CMD_BUFFER_SIZE 4096
#define DC_VO_SET_NOTIFY (__cpu_to_be32(1U << 0))

#define USE_ION

#ifdef USE_ION
#include <linux/ion.h>
#include <soc/realtek/uapi/ion_rtk.h>
#include <ion_rtk_alloc.h>
#define ion_alloc ext_rtk_ion_alloc
#define AUDIO_ION_FLAG (ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_ACPUACC)

#define BUFFER_NONCACHED	(0x1)
#define BUFFER_SCPUACC		(0x1 << 1)
#define BUFFER_ACPUACC		(0x1 << 2)
#define BUFFER_HWIPACC		(0x1 << 3)
#define BUFFER_VE_SPEC		(0x1 << 4)
#define BUFFER_ALGO_LAST_FIT	(0x1 << 5)
#define BUFFER_PROTECTED	(0x1 << 6)
#define BUFFER_SECURE_AUDIO	(0x1 << 7)
#define BUFFER_HEAP_MEDIA	(0x1 << 8)
#define BUFFER_HEAP_AUDIO	(0x1 << 9)
#define BUFFER_HEAP_SYSTEM	(0x1 << 10)
#define BUFFER_HEAP_DMA		(0x1 << 11)
#define BUFFER_HEAP_SECURE	(0x1 << 12)
#define BUFFER_MASK		((0x1 << 13) - 1)

extern unsigned int rtk_ion_flags(unsigned int dumb_flags);
extern unsigned int rtk_ion_heaps(unsigned int dumb_flags);
extern phys_addr_t rtk_ion_mem_phy(struct ion_buffer *buffer);
extern void *rtk_ion_kernel_map(struct ion_buffer *buffer);
#endif

enum {
	ENUM_KERNEL_RPC_CREATE_AGENT,
	ENUM_KERNEL_RPC_INIT_RINGBUF,
	ENUM_KERNEL_RPC_PRIVATEINFO,
	ENUM_KERNEL_RPC_RUN,
	ENUM_KERNEL_RPC_PAUSE,
	ENUM_KERNEL_RPC_SWITCH_FOCUS,
	ENUM_KERNEL_RPC_MALLOC_ADDR,
	ENUM_KERNEL_RPC_VOLUME_CONTROL,
	ENUM_KERNEL_RPC_FLUSH,
	ENUM_KERNEL_RPC_CONNECT,
	ENUM_KERNEL_RPC_SETREFCLOCK,
	ENUM_KERNEL_RPC_DAC_I2S_CONFIG,
	ENUM_KERNEL_RPC_DAC_SPDIF_CONFIG,
	ENUM_KERNEL_RPC_HDMI_OUT_EDID,
	ENUM_KERNEL_RPC_HDMI_OUT_EDID2,
	ENUM_KERNEL_RPC_HDMI_SET,
	ENUM_KERNEL_RPC_HDMI_MUTE,
	ENUM_KERNEL_RPC_ASK_DBG_MEM_ADDR,
	ENUM_KERNEL_RPC_DESTROY,
	ENUM_KERNEL_RPC_STOP,
	ENUM_KERNEL_RPC_CHECK_READY,
	ENUM_KERNEL_RPC_GET_MUTE_N_VOLUME,
	ENUM_KERNEL_RPC_EOS,
	ENUM_KERNEL_RPC_ADC0_CONFIG,
	ENUM_KERNEL_RPC_ADC1_CONFIG,
	ENUM_KERNEL_RPC_ADC2_CONFIG,
	ENUM_KERNEL_RPC_HDMI_OUT_VSDB,
	ENUM_VIDEO_KERNEL_RPC_CONFIG_TV_SYSTEM,
	ENUM_VIDEO_KERNEL_RPC_CONFIG_HDMI_INFO_FRAME,
	ENUM_VIDEO_KERNEL_RPC_QUERY_DISPLAY_WIN,
	ENUM_VIDEO_KERNEL_RPC_PP_INIT_PIN,
	ENUM_KERNEL_RPC_INIT_RINGBUF_AO,
	ENUM_VIDEO_KERNEL_RPC_VOUT_EDID_DATA,
	ENUM_KERNEL_RPC_AUDIO_POWER_SET,
	ENUM_VIDEO_KERNEL_RPC_VOUT_VDAC_SET,
	ENUM_VIDEO_KERNEL_RPC_QUERY_CONFIG_TV_SYSTEM,
	ENUM_KERNEL_RPC_AUDIO_CONFIG,
	ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
	ENUM_KERNEL_RPC_QUERY_FW_DEBUG_INFO,
	ENUM_KERNEL_RPC_HDMI_RX_LATENCY_MEM,
	ENUM_KERNEL_RPC_EQ_CONFIG,
	ENUM_VIDEO_KERNEL_RPC_CREATE,
	ENUM_VIDEO_KERNEL_RPC_DISPLAY,
	ENUM_VIDEO_KERNEL_RPC_CONFIGUREDISPLAYWINDOW,
	ENUM_VIDEO_KERNEL_RPC_SETREFCLOCK,
	ENUM_VIDEO_KERNEL_RPC_RUN,
	ENUM_VIDEO_KERNEL_RPC_INITRINGBUFFER,
	ENUM_VIDEO_KERNEL_RPC_SETRESCALEMODE,
	ENUM_VIDEO_KERNEL_RPC_SET_HDMI_VRR,
};

enum VO_VIDEO_PLANE {
	VO_VIDEO_PLANE_V1,
	VO_VIDEO_PLANE_V2,
	VO_VIDEO_PLANE_SUB1,
	VO_VIDEO_PLANE_OSD1,
	VO_VIDEO_PLANE_OSD2,
	VO_VIDEO_PLANE_WIN1,
	VO_VIDEO_PLANE_WIN2,
	VO_VIDEO_PLANE_WIN3,
	VO_VIDEO_PLANE_WIN4,
	VO_VIDEO_PLANE_WIN5,
	VO_VIDEO_PLANE_WIN6,
	VO_VIDEO_PLANE_WIN7,
	VO_VIDEO_PLANE_WIN8,
	VO_VIDEO_PLANE_NONE = 0xFF,
};

enum VIDEO_VF_TYPE {
	VF_TYPE_VIDEO_MPEG2_DECODER,
	VF_TYPE_VIDEO_MPEG4_DECODER,
	VF_TYPE_VIDEO_DIVX_DECODER,
	VF_TYPE_VIDEO_H263_DECODER,
	VF_TYPE_VIDEO_H264_DECODER,
	VF_TYPE_VIDEO_VC1_DECODER,
	VF_TYPE_VIDEO_REAL_DECODER,
	VF_TYPE_VIDEO_JPEG_DECODER,
	VF_TYPE_VIDEO_MJPEG_DECODER,
	VF_TYPE_SPU_DECODER,
	VF_TYPE_VIDEO_OUT,
	VF_TYPE_TRANSITION,
	VF_TYPE_THUMBNAIL,
	VF_TYPE_VIDEO_VP6_DECODER,
	VF_TYPE_VIDEO_IMAGE_DECODER,
	VF_TYPE_FLASH,
	VF_TYPE_VIDEO_AVS_DECODER,
	VF_TYPE_MIXER,
	VF_TYPE_VIDEO_VP8_DECODER,
	VF_TYPE_VIDEO_WMV7_DECODER,
	VF_TYPE_VIDEO_WMV8_DECODER,
	VF_TYPE_VIDEO_RAW_DECODER,
	VF_TYPE_VIDEO_THEORA_DECODER,
	VF_TYPE_VIDEO_FJPEG_DECODER,
	VF_TYPE_VIDEO_H265_DECODER,
	VF_TYPE_VIDEO_VP9_DECODER,
};

enum INBAND_CMD_TYPE {
	INBAND_CMD_TYPE_PTS = 0,
	INBAND_CMD_TYPE_PTS_SKIP,
	INBAND_CMD_TYPE_NEW_SEG,
	INBAND_CMD_TYPE_SEQ_END,
	INBAND_CMD_TYPE_EOS,
	INBAND_CMD_TYPE_CONTEXT,
	INBAND_CMD_TYPE_DECODE,

	/* Video Decoder In-band Command */
	VIDEO_DEC_INBAND_CMD_TYPE_VOBU,
	VIDEO_DEC_INBAND_CMD_TYPE_DVDVR_DCI_CCI,
	VIDEO_DEC_INBAND_CMD_TYPE_DVDV_VATR,

	/* MSG Type for parse mode */
	VIDEO_DEC_INBAND_CMD_TYPE_SEG_INFO,
	VIDEO_DEC_INBAND_CMD_TYPE_PIC_INFO,

	/* Sub-picture Decoder In-band Command */
	VIDEO_SUBP_INBAND_CMD_TYPE_SET_PALETTE,
	VIDEO_SUBP_INBAND_CMD_TYPE_SET_HIGHLIGHT,

	/* Video Mixer In-band Command */
	VIDEO_MIXER_INBAND_CMD_TYPE_SET_BG_COLOR,
	VIDEO_MIXER_INBAND_CMD_TYPE_SET_MIXER_RPTS,
	VIDEO_MIXER_INBAND_CMD_TYPE_BLEND,

	/* Video Scaler In-band Command */
	VIDEO_SCALER_INBAND_CMD_TYPE_OUTPUT_FMT,

	/*DivX3 resolution In-band Command*/
	VIDEO_DIVX3_INBAND_CMD_TYPE_RESOLUTION,

	/*MPEG4 DivX4 detected In-band command*/
	VIDEO_MPEG4_INBAND_CMD_TYPE_DIVX4,
	/* Audio In-band Commands Start Here */

	/* DV In-band Commands */
	VIDEO_DV_INBAND_CMD_TYPE_VAUX,
	VIDEO_DV_INBAND_CMD_TYPE_FF,	//fast forward

	/* Transport Demux In-band command */
	VIDEO_TRANSPORT_DEMUX_INBAND_CMD_TYPE_PID,
	VIDEO_TRANSPORT_DEMUX_INBAND_CMD_TYPE_PTS_OFFSET,
	VIDEO_TRANSPORT_DEMUX_INBAND_CMD_TYPE_PACKET_SIZE,

	/* Real Video In-band command */
	VIDEO_RV_INBAND_CMD_TYPE_FRAME_INFO,
	VIDEO_RV_INBAND_CMD_TYPE_FORMAT_INFO,
	VIDEO_RV_INBAND_CMD_TYPE_SEGMENT_INFO,

	/*VC1 video In-band command*/
	VIDEO_VC1_INBAND_CMD_TYPE_SEQ_INFO,

	/* general video properties */
	VIDEO_INBAND_CMD_TYPE_VIDEO_USABILITY_INFO,
	VIDEO_INBAND_CMD_TYPE_VIDEO_MPEG4_USABILITY_INFO,

	/*MJPEG resolution In-band Command*/
	VIDEO_MJPEG_INBAND_CMD_TYPE_RESOLUTION,

	/* picture object for graphic */
	VIDEO_GRAPHIC_INBAND_CMD_TYPE_PICTURE_OBJECT,
	VIDEO_GRAPHIC_INBAND_CMD_TYPE_DISPLAY_INFO,

	/* subtitle offset sequence id for 3D video */
	VIDEO_DEC_INBAND_CMD_TYPE_SUBP_OFFSET_SEQUENCE_ID,

	VIDEO_H264_INBAND_CMD_TYPE_DPBBYPASS,

	/* Clear back frame to black color and send it to VO */
	VIDEO_FJPEG_INBAND_CMD_TYPE_CLEAR_SCREEN,

	/* each picture info of MJPEG */
	VIDEO_FJPEG_INBAND_CMD_TYPE_PIC_INFO,

	/*FJPEG resolution In-band Command*/
	VIDEO_FJPEG_INBAND_CMD_TYPE_RESOLUTION,

	/*VO receive VP_OBJ_PICTURE_TYPE In-band Command*/
	VIDEO_VO_INBAND_CMD_TYPE_OBJ_PIC,
	VIDEO_VO_INBAND_CMD_TYPE_OBJ_DVD_SP,
	VIDEO_VO_INBAND_CMD_TYPE_OBJ_DVB_SP,
	VIDEO_VO_INBAND_CMD_TYPE_OBJ_BD_SP,
	VIDEO_VO_INBAND_CMD_TYPE_OBJ_SP_FLUSH,
	VIDEO_VO_INBAND_CMD_TYPE_OBJ_SP_RESOLUTION,

	/* VO receive writeback buffers In-band Command */
	VIDEO_VO_INBAND_CMD_TYPE_WRITEBACK_BUFFER,

	/* for VO debug, VO can dump picture */
	VIDEO_VO_INBAND_CMD_TYPE_DUMP_PIC,
	VIDEO_CURSOR_INBAND_CMD_TYPE_PICTURE_OBJECT,
	VIDEO_CURSOR_INBAND_CMD_TYPE_COORDINATE_OBJECT,
	VIDEO_TRANSCODE_INBAND_CMD_TYPE_PICTURE_OBJECT,
	VIDEO_WRITEBACK_INBAND_CMD_TYPE_PICTURE_OBJECT,

	VIDEO_VO_INBAND_CMD_TYPE_OBJ_BD_SCALE_RGB_SP,

	/* TV code */
	VIDEO_INBAND_CMD_TYPE_DIVX_CERTIFY,

	/* M_DOMAIN resolution In-band Command */
	VIDEO_INBAND_CMD_TYPE_M_DOMAIN_RESOLUTION,

	/* DTV source In-band Command */
	VIDEO_INBAND_CMD_TYPE_SOURCE_DTV,

	/* Din source copy mode In-band Command */
	VIDEO_DIN_INBAND_CMD_TYPE_COPY_MODE,

	/* Video Decoder AU In-band command */
	VIDEO_DEC_INBAND_CMD_TYPE_AU,

	/* Video Decoder parse frame In-band command */
	VIDEO_DEC_INBAND_CMD_TYPE_PARSE_FRAME_IN,
	VIDEO_DEC_INBAND_CMD_TYPE_PARSE_FRAME_OUT,

	/* Set video decode mode In-band command */
	VIDEO_DEC_INBAND_CMD_TYPE_NEW_DECODE_MODE,

	/* Secure buffer protection */
	VIDEO_INBAND_CMD_TYPE_SECURE_PROTECTION,

	/* Dolby HDR inband command */
	VIDEO_DEC_INBAND_CMD_TYPE_DV_PROFILE,

	/* VP9 HDR10 In-band command */
	VIDEO_VP9_INBAND_CMD_TYPE_HDR10_METADATA,

	/* AV1 HDR10 In-band command */
	VIDEO_AV1_INBAND_CMD_TYPE_HDR10_METADATA,

	/* DvdPlayer tell RVSD video BS ring buffer is full */
	VIDEO_DEC_INBAND_CMD_TYPE_BS_RINGBUF_FULL,
};

enum INBAND_CMD_GRAPHIC_FORMAT {
	INBAND_CMD_GRAPHIC_FORMAT_RGB565 = 4,
	INBAND_CMD_GRAPHIC_FORMAT_ARGB1555 = 5,
	INBAND_CMD_GRAPHIC_FORMAT_ARGB4444 = 6,
	INBAND_CMD_GRAPHIC_FORMAT_ARGB8888 = 7,
	INBAND_CMD_GRAPHIC_FORMAT_YCBCRA4444 = 11,
	INBAND_CMD_GRAPHIC_FORMAT_YCBCRA8888 = 12,
	INBAND_CMD_GRAPHIC_FORMAT_RGBA5551 = 13,
	INBAND_CMD_GRAPHIC_FORMAT_RGBA4444 = 14,
	INBAND_CMD_GRAPHIC_FORMAT_RGBA8888 = 15,
	INBAND_CMD_GRAPHIC_FORMAT_RGB556 = 20,
	INBAND_CMD_GRAPHIC_FORMAT_RGB655 = 21,
	INBAND_CMD_GRAPHIC_FORMAT_RGB888 = 22,
	INBAND_CMD_GRAPHIC_FORMAT_RGB565_LITTLE  = 36,
	INBAND_CMD_GRAPHIC_FORMAT_ARGB1555_LITTLE = 37,
	INBAND_CMD_GRAPHIC_FORMAT_ARGB4444_LITTLE = 38,
	INBAND_CMD_GRAPHIC_FORMAT_ARGB8888_LITTLE = 39,
	INBAND_CMD_GRAPHIC_FORMAT_YCBCRA4444_LITTLE = 43,
	INBAND_CMD_GRAPHIC_FORMAT_YCBCRA8888_LITTLE = 44,
	INBAND_CMD_GRAPHIC_FORMAT_RGBA5551_LITTLE = 45,
	INBAND_CMD_GRAPHIC_FORMAT_RGBA4444_LITTLE = 46,
	INBAND_CMD_GRAPHIC_FORMAT_RGBA8888_LITTLE = 47,
	INBAND_CMD_GRAPHIC_FORMAT_RGB556_LITTLE = 52,
	INBAND_CMD_GRAPHIC_FORMAT_RGB655_LITTLE = 53,
	INBAND_CMD_GRAPHIC_FORMAT_RGB888_LITTLE = 54,
};

enum INBAND_CMD_GRAPHIC_3D_MODE {
	INBAND_CMD_GRAPHIC_2D_MODE,
	INBAND_CMD_GRAPHIC_SIDE_BY_SIDE,
	INBAND_CMD_GRAPHIC_TOP_AND_BOTTOM,
	INBAND_CMD_GRAPHIC_FRAME_PACKING,
};

enum {
	INTERLEAVED_TOP_FIELD = 0,  /* top	field data stored in even lines of a frame buffer */
	INTERLEAVED_BOT_FIELD,	  /* bottom field data stored in odd  lines of a frame buffer */
	CONSECUTIVE_TOP_FIELD,	  /* top	field data stored consecutlively in all lines of a field buffer */
	CONSECUTIVE_BOT_FIELD,	  /* bottom field data stored consecutlively in all lines of a field buffer */
	CONSECUTIVE_FRAME,		   /* progressive frame data stored consecutlively in all lines of a frame buffer */
	INTERLEAVED_TOP_FIELD_422,  /* top	field data stored in even lines of a frame buffer */
	INTERLEAVED_BOT_FIELD_422,	  /* bottom field data stored in odd  lines of a frame buffer */
	CONSECUTIVE_TOP_FIELD_422,	  /* top	field data stored consecutlively in all lines of a field buffer */
	CONSECUTIVE_BOT_FIELD_422,	  /* bottom field data stored consecutlively in all lines of a field buffer */
	CONSECUTIVE_FRAME_422,		/* progressive frame with 4:2:2 chroma */
	TOP_BOTTOM_FRAME,			/* top field in the 0~height/2-1, bottom field in the height/2~height-1 in the frame */
	INTERLEAVED_TOP_BOT_FIELD,   /* one frame buffer contains one top and one bot field, top field first */
	INTERLEAVED_BOT_TOP_FIELD,   /* one frame buffer contains one bot and one top field, bot field first */

	MPEG2_PIC_MODE_NOT_PROG	  /*yllin: for MPEG2 check pic mode usage */
};

enum AVSYNC_MODE {
	AVSYNC_FORCED_SLAVE,
	AVSYNC_FORCED_MASTER,
	AVSYNC_AUTO_SLAVE,
	AVSYNC_AUTO_MASTER,
	AVSYNC_AUTO_MASTER_NO_SKIP,
	AVSYNC_AUTO_MASTER_CONSTANT_DELAY,
};

enum AUTOMASTER_STATE {
	AUTOMASTER_NOT_MASTER,
	AUTOMASTER_IS_MASTER,
};

enum VO_HDMI_MODE {
	VO_DVI_ON,
	VO_HDMI_ON,
	VO_HDMI_OFF,
};

enum VO_HDMI_AUDIO_SAMPLE_FREQ {
	VO_HDMI_AUDIO_NULL,
	VO_HDMI_AUDIO_32K,
	VO_HDMI_AUDIO_44_1K,
	VO_HDMI_AUDIO_48K,
	VO_HDMI_AUDIO_88_2K,
	VO_HDMI_AUDIO_96K,
	VO_HDMI_AUDIO_176_4K,
	VO_HDMI_AUDIO_192K,
};

enum VO_INTERFACE_TYPE {
	VO_ANALOG_AND_DIGITAL,
	VO_ANALOG_ONLY,
	VO_DIGITAL_ONLY,
	VO_DISPLAY_PORT_ONLY,
	VO_HDMI_AND_DISPLAY_PORT_SAME_SOURCE,
	VO_HDMI_AND_DISPLAY_PORT_DIFF_SOURCE,
	VO_DISPLAY_PORT_AND_CVBS_SAME_SOURCE,
	VO_HDMI_AND_DP_DIFF_SOURCE_WITH_CVBS,
	VO_FORCE_DP_OFF,
};

enum VO_PEDESTAL_TYPE {
	VO_PEDESTAL_TYPE_300_700_ON,
	VO_PEDESTAL_TYPE_300_700_OFF,
	VO_PEDESTAL_TYPE_286_714_ON,
	VO_PEDESTAL_TYPE_286_714_OFF,
};

enum VO_STANDARD {
	VO_STANDARD_NTSC_Mi,
	VO_STANDARD_NTSC_J,
	VO_STANDARD_NTSC_443,
	VO_STANDARD_PAL_B,
	VO_STANDARD_PAL_D,
	VO_STANDARD_PAL_G,
	VO_STANDARD_PAL_H,
	VO_STANDARD_PAL_I,
	VO_STANDARD_PAL_N,
	VO_STANDARD_PAL_NC,
	VO_STANDARD_PAL_M,
	VO_STANDARD_PAL_60,
	VO_STANDARD_SECAM,
	VO_STANDARD_HDTV_720P_60,
	VO_STANDARD_HDTV_720P_50,
	VO_STANDARD_HDTV_720P_30,
	VO_STANDARD_HDTV_720P_25,
	VO_STANDARD_HDTV_720P_24,
	VO_STANDARD_HDTV_1080I_60,
	VO_STANDARD_HDTV_1080I_50,
	VO_STANDARD_HDTV_1080P_30,
	VO_STANDARD_HDTV_1080P_25,
	VO_STANDARD_HDTV_1080P_24,
	VO_STANDARD_VGA,
	VO_STANDARD_SVGA,
	VO_STANDARD_HDTV_1080P_60,
	VO_STANDARD_HDTV_1080P_50,
	VO_STANDARD_HDTV_1080I_59,
	VO_STANDARD_HDTV_720P_59,
	VO_STANDARD_HDTV_1080P_23,
	VO_STANDARD_HDTV_1080P_59,
	VO_STANDARD_HDTV_1080P_60_3D,
	VO_STANDARD_HDTV_1080P_50_3D,
	VO_STANDARD_HDTV_1080P_30_3D,
	VO_STANDARD_HDTV_1080P_24_3D,
	VO_STANDARD_HDTV_720P_60_3D,
	VO_STANDARD_HDTV_720P_50_3D,
	VO_STANDARD_HDTV_720P_30_3D,
	VO_STANDARD_HDTV_720P_24_3D,
	VO_STANDARD_HDTV_720P_59_3D,
	VO_STANDARD_HDTV_1080I_60_3D,
	VO_STANDARD_HDTV_1080I_59_3D,
	VO_STANDARD_HDTV_1080I_50_3D,
	VO_STANDARD_HDTV_1080P_23_3D,
	VO_STANDARD_HDTV_2160P_30,
	VO_STANDARD_HDTV_2160P_29,
	VO_STANDARD_HDTV_2160P_25,
	VO_STANDARD_HDTV_2160P_24,
	VO_STANDARD_HDTV_2160P_23,
	VO_STANDARD_HDTV_4096_2160P_24,
	VO_STANDARD_HDTV_2160P_60,
	VO_STANDARD_HDTV_2160P_50,
	VO_STANDARD_HDTV_4096_2160P_25,
	VO_STANDARD_HDTV_4096_2160P_30,
	VO_STANDARD_HDTV_4096_2160P_50,
	VO_STANDARD_HDTV_4096_2160P_60,
	VO_STANDARD_HDTV_2160P_60_420,
	VO_STANDARD_HDTV_2160P_50_420,
	VO_STANDARD_HDTV_4096_2160P_60_420,
	VO_STANDARD_HDTV_4096_2160P_50_420,
	VO_STANDARD_DP_FORMAT_1920_1080P_60,
	VO_STANDARD_DP_FORMAT_2160P_30,
	VO_STANDARD_HDTV_2160P_24_3D,
	VO_STANDARD_HDTV_2160P_23_3D,
	VO_STANDARD_HDTV_2160P_59,
	VO_STANDARD_HDTV_2160P_59_420,
	VO_STANDARD_HDTV_2160P_25_3D,
	VO_STANDARD_HDTV_2160P_30_3D,
	VO_STANDARD_HDTV_2160P_50_3D,
	VO_STANDARD_HDTV_2160P_60_3D,
	VO_STANDARD_HDTV_4096_2160P_24_3D,
	VO_STANDARD_HDTV_4096_2160P_25_3D,
	VO_STANDARD_HDTV_4096_2160P_30_3D,
	VO_STANDARD_HDTV_4096_2160P_50_3D,
	VO_STANDARD_HDTV_4096_2160P_60_3D,
	VO_STANDARD_DP_FORMAT_1280_720P_60,
	VO_STANDARD_DP_FORMAT_3840_2160P_60,
	VO_STANDARD_DP_FORMAT_1024_768P_60,
	VO_STANDARD_HDTV_2160P_50_422_12bit,
	VO_STANDARD_HDTV_2160P_60_422_12bit,
	VO_STANDARD_DP_FORMAT_1280_800P_60,
	VO_STANDARD_DP_FORMAT_1440_900P_60,
	VO_STANDARD_DP_FORMAT_1440_768P_60,
	VO_STANDARD_DP_FORMAT_960_544P_60,
	VO_STANDARD_HDTV_720P_120_3D,
	VO_STANDARD_DP_FORMAT_800_480P_60,
	VO_STANDARD_HDTV_2160P_59_422_12bit,
	VO_STANDARD_DP_FORMAT_800_1280P_60,
	VO_STANDARD_DP_FORMAT_1280_720P_50,
	VO_STANDARD_HDTV_1080P_144,
	VO_STANDARD_HDTV_1024_768P_70,
	VO_STANDARD_DP_FORMAT_1024_600P_60,
	VO_STANDARD_DP_FORMAT_600_1024P_60,
	VO_STANDARD_DSIMIPI_FORMAT_1200_1920P_60,
	VO_STANDARD_HDTV_1080P_120,
	VO_STANDARD_HDTV_720P_P24,
	VO_STANDARD_HDTV_720P_P25,
	VO_STANDARD_HDTV_720P_P30,
	VO_STANDARD_HDTV_720P_P23,
	VO_STANDARD_HDTV_720P_P29,
	VO_STANDARD_DP_FORMAT_1366_768P_60,
	VO_STANDARD_HDTV_1080P_29,
	VO_STANDARD_ERROR,
};

enum VO_HDMI_OFF_MODE {
	VO_HDMI_OFF_CLOCK_OFF,
	VO_HDMI_OFF_CLOCK_ON,
};

enum rtk_hdr_mode {
	HDR_CTRL_AUTO,
	HDR_CTRL_DV_ON,
	HDR_CTRL_SDR,
	HDR_CTRL_HDR_GAMMA,
	HDR_CTRL_PQHDR,
	HDR_CTRL_FUTURE,
	HDR_CTRL_INPUT,
	HDR_CTRL_DV_LOW_LATENCY_12b_YUV422,
	HDR_CTRL_DV_LOW_LATENCY_10b_YUV444,
	HDR_CTRL_DV_LOW_LATENCY_10b_RGB444,
	HDR_CTRL_DV_LOW_LATENCY_12b_YUV444,
	HDR_CTRL_DV_LOW_LATENCY_12b_RGB444,
	HDR_CTRL_DV_ON_INPUT,
	HDR_CTRL_DV_ON_LOW_LATENCY_12b422_INPUT,
	HDR_CTRL_INPUT_BT2020,
};

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

enum dc_buffer_flags {
	eBuffer_AFBC_Enable = 0x1U << 16,
	eBuffer_AFBC_Split = 0x1U << 17,
	eBuffer_AFBC_YUV_Transform = 0x1U << 18,
	eBuffer_USE_GLOBAL_ALPHA = 0x1U << 19,
};

struct inband_cmd_pkg_header {
	enum INBAND_CMD_TYPE type;
	unsigned int size;
};

struct graphic_object {
	struct inband_cmd_pkg_header header;
	enum INBAND_CMD_GRAPHIC_FORMAT format;
	unsigned int PTSH;
	unsigned int PTSL;
	unsigned int context;
	int colorkey;
	int alpha;
	unsigned int x;
	unsigned int y;
	unsigned int width;
	unsigned int height;
	unsigned int address;
	unsigned int pitch;
	unsigned int address_right;
	unsigned int pitch_right;
	enum INBAND_CMD_GRAPHIC_3D_MODE picLayout;
	unsigned int afbc;
	unsigned int afbc_block_split;
	unsigned int afbc_yuv_transform;
};

struct rpc_create_video_agent {
	unsigned int instance;
	unsigned int result;
	unsigned int data;
	unsigned int ret;
};

struct rpc_vo_filter_display {
	unsigned int instance;
	enum VO_VIDEO_PLANE videoPlane;
	unsigned char zeroBuffer;
	unsigned char realTimeSrc;
};

struct rpc_vo_filter_display_t {
	unsigned int instance;
	enum VO_VIDEO_PLANE videoPlane;
	unsigned char zeroBuffer;
	unsigned char realTimeSrc;
	unsigned int result;
	unsigned int data;
	unsigned int ret;
};

struct vo_rectangle {
	short x;
	short y;
	unsigned short width;
	unsigned short height;
};

struct vo_color {
	unsigned short c1;
	unsigned short c2;
	unsigned short c3;
	unsigned short isRGB;
};

struct rpc_config_disp_win {
	enum VO_VIDEO_PLANE videoPlane;
	struct vo_rectangle videoWin;
	struct vo_rectangle borderWin;
	struct vo_color borderColor;
	unsigned char enBorder;
};

struct rpc_config_disp_win_t {
	enum VO_VIDEO_PLANE videoPlane;
	struct vo_rectangle videoWin;
	struct vo_rectangle borderWin;
	struct vo_color borderColor;
	unsigned char enBorder;
	unsigned int result;
	unsigned int data;
	unsigned int ret;
};

struct rpc_refclock {
	int instance;
	int pRefClock;
};

struct rpc_refclock_t {
	int instance;
	int pRefClock;
	unsigned int result;
	unsigned int data;
	unsigned int ret;
};

struct rpc_ringbuffer {
	unsigned int instance;
	unsigned int pinID;
	unsigned int readPtrIndex;
	unsigned int pRINGBUFF_HEADER;
};

struct rpc_ringbuffer_t {
	unsigned int result;
	unsigned int data;
	unsigned int ret;
	unsigned int instance;
	unsigned int pinID;
	unsigned int readPtrIndex;
	unsigned int pRINGBUFF_HEADER;
};

struct rpc_video_run_t {
	unsigned int instance;
	unsigned int result;
	unsigned int data;
	unsigned int ret;
};

struct rpc_config_video_standard {
	enum VO_STANDARD standard;
	unsigned char enProg;
	unsigned char enDIF;
	unsigned char enCompRGB;
	enum VO_PEDESTAL_TYPE pedType;
	unsigned int dataInt0;
	unsigned int dataInt1;
};

#define RPC_BPC_MASK  0x3E
#define RPC_BPC_8     0x10
#define RPC_BPC_10    0x16
#define RPC_BPC_12    0x1A

/* hdmi2px_feature */
#define HDMI2PX_2P0_MASK      0x1
#define HDMI2PX_2P0           0x1
#define HDMI2PX_SCRAMBLE_MASK 0x2
#define HDMI2PX_SCRAMBLE      0x2
#define HDMI2PX_FRLRATE_MASK  0x3C
#define HDMI2PX_FRL_3G3L      0x4
#define HDMI2PX_FRL_6G3L      0x8
#define HDMI2PX_FRL_6G4L      0xC

struct rpc_config_info_frame {
	enum VO_HDMI_MODE hdmiMode;
	enum VO_HDMI_AUDIO_SAMPLE_FREQ audioSampleFreq;
	unsigned char audioChannelCount;
	unsigned char dataByte1;
	unsigned char dataByte2;
	unsigned char dataByte3;
	unsigned char dataByte4;
	unsigned char dataByte5;
	unsigned int dataInt0;
	/* hdmi2px_feature [Bit5:2] FRL Rate [Bit1]Scramble [Bit0]HDMI 2.x */
	unsigned int hdmi2px_feature;
	enum VO_HDMI_OFF_MODE hdmi_off_mode;
	enum rtk_hdr_mode hdr_ctrl_mode;
	unsigned int reserved4;
};

struct rpc_config_tv_system {
	enum VO_INTERFACE_TYPE interfaceType;
	struct rpc_config_video_standard videoInfo;
	struct rpc_config_info_frame info_frame;
};

#define FEATURE_QMS        (1 << 0)
#define FEATURE_QMS_TFRMIN (1 << 1)
#define FEATURE_QMS_TFRMAX (1 << 2)
struct rpc_vout_edid_data {
	/* HDR metadata */
	u_char et;
	u_char sm;
	u_char max_luminace;
	u_char max_frame_avg;
	u_char min_luminace;
	/* Color characteristics */
	u_char red_green_lo;
	u_char black_white_lo;
	u_char red_x;
	u_char red_y;
	u_char green_x;
	u_char green_y;
	u_char blue_x;
	u_char blue_y;
	u_char white_x;
	u_char white_y;
	/* Colorimetry Data */
	u_char color_space;
	/* Video Capability Data Block */
	u_char vcdb;/* Bit7:QY,Bit6:QS */
	/* HDR metadata length */
	u_char metadata_len;
	/* Dolby Vision data length */
	u_char dolby_len;
	/* Dolby Vision VSVDB additional payload */
	u_char dolby_data[22];
	/* HDR10+ support */
	u_char hdr10_plus;
	/* Max_FRL_Rate */
	u_char max_frl_rate;
	/* VRR feature [Bit2]QMS-TFRmax; [Bit1]QMS-TFRmin; [Bit0]QMS */
	u_char vrr_feature;
	/* VRR VRRmax[9:8] VRRmin */
	u_char vrr_max98_min;
	/* VRR VRRmax[7:0] */
	u_char vrr_max;
	/* Reserved */
	u_char reserved[2];
};

enum ENUM_HDMI_VRR_FUNC_TYPE {
	HDMI_VRR_ON_OFF = 0,
	HDMI_VRR_TARGET_RATE = 1,
	HDMI_ALLM_ON_OFF = 2,
};

enum ENUM_HDMI_VRR_ACT {
	/* When vrr_function = HDMI_VRR_ON_OFF */
	HDMI_VRR_DISABLE = 0,
	HDMI_VRR_ENABLE = 1,
	HDMI_QMS_ENABLE = 2,
	/* When vrr_function = HDMI_VRR_TARGET_RATE */
	HDMI_VRR_60HZ = 0,
	HDMI_VRR_50HZ = 1,
	HDMI_VRR_48HZ = 2,
	HDMI_VRR_24HZ = 3,
	HDMI_VRR_59HZ = 4,
	HDMI_VRR_47HZ = 5,
	HDMI_VRR_30HZ = 6,
	HDMI_VRR_29HZ = 7,
	HDMI_VRR_25HZ = 8,
	HDMI_VRR_23HZ = 9,
	/* When vrr_function = HDMI_ALLM_ON_OFF */
	HDMI_ALLM_DISABLE = 0,
	HDMI_ALLM_ENABLE = 1,
};

/**
 * struct rpc_vout_hdmi_vrr -
 *   Parameter of RPC ENUM_VIDEO_KERNEL_RPC_SET_HDMI_VRR
 *
 * @vrr_function: enum ENUM_HDMI_VRR_FUNC_TYPE
 * @vrr_act: enum ENUM_HDMI_VRR_ACT
 */
struct rpc_vout_hdmi_vrr {
	enum ENUM_HDMI_VRR_FUNC_TYPE vrr_function;
	enum ENUM_HDMI_VRR_ACT vrr_act;
	int reserved[15];
};

/**
 * struct audio_sad -Short Audio Descriptors
 * @byte0: Bit[6:3] Audio Format Code, Bit[2:0] Max Number of channels -1
 * @byte1: Sampling rates
 * @byte2: depends on format
 */
struct audio_sad {
	u8 byte0;
	u8 byte1;
	u8 byte2;
} __attribute__ ((packed));

#define MAX_SAD_SIZE  10
struct audio_edid_data {
	u8 data_size;
	struct audio_sad sad[MAX_SAD_SIZE];
} __attribute__ ((packed));

struct rpc_audio_edid_data {
	uint32_t version;
	uint32_t hdmi_en_state;
	uint32_t edid_data_addr;
};

/**
 * struct rpc_audio_hdmi_freq - hdmi frequency for AO
 * @hdmi_frequency: [Bit23:Bit16] FRL Rate [Bit15:Bit0] TMDS freq(MHz)
 */
#define AUDIO_FRL_RATE1 0x00010000
#define AUDIO_FRL_RATE2 0x00020000
#define AUDIO_FRL_RATE3 0x00030000
struct rpc_audio_hdmi_freq {
	uint32_t tmds_freq;
};

struct tag_ringbuffer_header {
	unsigned int magic;	//Magic number
	unsigned int beginAddr;
	unsigned int size;
	unsigned int bufferID;	// RINGBUFFER_TYPE, choose a type from RINGBUFFER_TYPE

	unsigned int writePtr;
	unsigned int numOfReadPtr;
	unsigned int reserve2;	//Reserve for Red Zone
	unsigned int reserve3;	//Reserve for Red Zone

	unsigned int readPtr[4];

	int fileOffset;
	int requestedFileOffset;
	int fileSize;

	int bSeekable;		//Can't be sought if data is streamed by HTTP
};

struct tag_master_ship {
	unsigned char systemMode;	/* enum AVSYNC_MODE */
	unsigned char videoMode;	/* enum AVSYNC_MODE */
	unsigned char audioMode;	/* enum AVSYNC_MODE */
	unsigned char masterState;	/* enum AUTOMASTER_STATE */
};

struct tag_refclock {
	long long RCD;
	unsigned int RCD_ext;
	long long GPTSTimeout;
	long long videoSystemPTS;
	long long audioSystemPTS;
	long long videoRPTS;
	long long audioRPTS;
	unsigned int videoContext;
	unsigned int audioContext;

	struct tag_master_ship mastership;
	unsigned int videoFreeRunThreshold;
	unsigned int audioFreeRunThreshold;
	long long masterGPTS;	// this is the value of GPTS (hardware PTS) when master set the reference clock
	int audioFullness;	// This flag will be turned on when AE's output buffer is almost full.
				// VE which is monitoring this flag will issue auto-pause then.
				// (0: still have enough output space for encoding.   1: AE request pause)
	int audioPauseFlag;	// This flag will be turned on when VE decides to auto-pause.
				// AE which is monitoring this flag will auto-pause itself then.
				// (0: ignore.  1: AE have to pauseEncode immediately)
	int VO_Underflow;	// (0: VO is working properly; otherwise, VO underflow)
	int AO_Underflow;	// (0: AO is working properly; otherwise, AO underflow)
	int videoEndOfSegment;	// set to the flow EOS.eventID by VO after presenting the EOS sample
	int audioEndOfSegment;	// set to the flow EOS.eventID by AO after presenting the EOS sample
#ifdef DC2VO_SUPPORT_MEMORY_TRASH
	unsigned int memorytrashAddr;
	unsigned int memorytrashContext;
	unsigned char reserved[8];
#else /* DC2VO_SUPPORT_MEMORY_TRASH */
	unsigned char  reserved[16];
#endif /* End of DC2VO_SUPPORT_MEMORY_TRASH */
};

struct tch_metadata_variables {
	int tmInputSignalBlackLevelOffset;
	int tmInputSignalWhiteLevelOffset;
	int shadowGain;
	int highlightGain;
	int midToneWidthAdjFactor;
	int tmOutputFineTuningNumVal;
	int tmOutputFineTuningX[15];
	int tmOutputFineTuningY[15];
	int saturationGainNumVal;
	int saturationGainX[15];
	int saturationGainY[15];
};

struct tch_metadata_tables {
	int luminanceMappingNumVal;
	int luminanceMappingX[33];
	int luminanceMappingY[33];
	int colourCorrectionNumVal;
	int colourCorrectionX[33];
	int colourCorrectionY[33];
	int chromaToLumaInjectionMuA;
	int chromaToLumaInjectionMuB;
};

struct tch_metadata {
	int specVersion;
	int payloadMode;
	int hdrPicColourSpace;
	int hdrMasterDisplayColourSpace;
	int hdrMasterDisplayMaxLuminance;
	int hdrMasterDisplayMinLuminance;
	int sdrPicColourSpace;
	int sdrMasterDisplayColourSpace;
	union {
		struct tch_metadata_variables variables;
		struct tch_metadata_tables tables;
	} u;
};

struct video_object {
	struct inband_cmd_pkg_header header;
	unsigned int version;
	unsigned int mode;
	unsigned int Y_addr;
	unsigned int U_addr;
	unsigned int pLock;
	unsigned int width;
	unsigned int height;
	unsigned int Y_pitch;
	unsigned int C_pitch;
	unsigned int RPTSH;
	unsigned int RPTSL;
	unsigned int PTSH;
	unsigned int PTSL;

	/* for send two interlaced fields in the same packet, */
	/* valid only when mode is INTERLEAVED_TOP_BOT_FIELD or INTERLEAVED_BOT_TOP_FIELD */
	unsigned int RPTSH2;
	unsigned int RPTSL2;
	unsigned int PTSH2;
	unsigned int PTSL2;

	unsigned int context;
	unsigned int pRefClock;		/* not used now */

	unsigned int pixelAR_hor;	/* pixel aspect ratio hor, not used now */
	unsigned int pixelAR_ver;	/* pixel aspect ratio ver, not used now */

	unsigned int Y_addr_Right;	/* for mvc */
	unsigned int U_addr_Right;	/* for mvc */
	unsigned int pLock_Right;	/* for mvc */
	unsigned int mvc;		/* 1: mvc */
	unsigned int subPicOffset;	/* 3D Blu-ray dependent-view sub-picture plane offset metadata as defined in BD spec sec. 9.3.3.6. */
					/* Valid only when Y_BufId_Right and C_BufId_Right are both valid */
	unsigned int pReceived;		// fix bug 44329 by version 0x72746B30 'rtk0'
	unsigned int pReceived_Right;	// fix bug 44329 by version 0x72746B30 'rtk0'

	unsigned int fps;		// 'rtk1'

	unsigned int IsForceDIBobMode;	// force vo use bob mode to do deinterlace, 'rtk2'.
	unsigned int lumaOffTblAddr;	// 'rtk3'
	unsigned int chromaOffTblAddr;	// 'rtk3'
	unsigned int lumaOffTblAddrR;	// for mvc, 'rtk3'
	unsigned int chromaOffTblAddrR;	// for mvc, 'rtk3'

	unsigned int bufBitDepth;	// 'rtk3'
	unsigned int bufFormat;		// 'rtk3', according to VO spec: 10bits Pixel Packing mode selection,
					// "0": use 2 bytes to store 1 components. MSB justified.
					// "1": use 4 bytes to store 3 components. LSB justified.

	// VUI (Video Usability Information)
	unsigned int transferCharacteristics;	// 0:SDR, 1:HDR, 2:ST2084, 'rtk3'
	// Mastering display colour volume SEI, 'rtk3'
	unsigned int display_primaries_x0;
	unsigned int display_primaries_y0;
	unsigned int display_primaries_x1;
	unsigned int display_primaries_y1;
	unsigned int display_primaries_x2;
	unsigned int display_primaries_y2;
	unsigned int white_point_x;
	unsigned int white_point_y;
	unsigned int max_display_mastering_luminance;
	unsigned int min_display_mastering_luminance;

	// for transcode interlaced feild use.	//'rtk4'
	unsigned int Y_addr_prev;		//'rtk4'
	unsigned int U_addr_prev;		//'rtk4'
	unsigned int Y_addr_next;		//'rtk4'
	unsigned int U_addr_next;		//'rtk4'
	unsigned int video_full_range_flag;	//'rtk4' default= 1
	unsigned int matrix_coefficients;	//'rtk4' default= 1

	// for transcode interlaced feild use.	//'rtk5'
	unsigned int pLock_prev;
	unsigned int pReceived_prev;
	unsigned int pLock_next;
	unsigned int pReceived_next;

	unsigned int is_tch_video;		//'rtk6'
	struct tch_metadata tch_hdr_metadata;	//'rtk6'
#if 0
	unsigned int pFrameBufferDbg;		//'rtk7'
	unsigned int pFrameBufferDbg_Right;
	unsigned int Y_addr_EL;			//'rtk8' for dolby vision
	unsigned int U_addr_EL;
	unsigned int width_EL;
	unsigned int height_EL;
	unsigned int Y_pitch_EL;
	unsigned int C_pitch_EL;
	unsigned int lumaOffTblAddr_EL;
	unsigned int chromaOffTblAddr_EL;

	unsigned int dm_reg1_addr;
	unsigned int dm_reg1_size;
	unsigned int dm_reg2_addr;
	unsigned int dm_reg2_size;
	unsigned int dm_reg3_addr;
	unsigned int dm_reg3_size;
	unsigned int dv_lut1_addr;
	unsigned int dv_lut1_size;
	unsigned int dv_lut2_addr;
	unsigned int dv_lut2_size;

	unsigned int slice_height;		//'rtk8'

	unsigned int hdr_metadata_addr;		//'rtk9'
	unsigned int hdr_metadata_size;		//'rtk9'
	unsigned int tch_metadata_addr;		//'rtk9'
	unsigned int tch_metadata_size;		//'rtk9'
	unsigned int is_dolby_video;		//'rtk10'

	unsigned int lumaOffTblSize;		//'rtk11'
	unsigned int chromaOffTblSize;		//'rtk11'
	// 'rtk12'
	unsigned int Combine_Y_Addr;
	unsigned int Combine_U_Addr;
	unsigned int Combine_Width;
	unsigned int Combine_Height;
	unsigned int Combine_Y_Pitch;
	unsigned int secure_flag;

	// 'rtk13'
	unsigned int tvve_picture_width;
	unsigned int tvve_lossy_en;
	unsigned int tvve_bypass_en;
	unsigned int tvve_qlevel_sel_y;
	unsigned int tvve_qlevel_sel_c;
	unsigned int is_ve_tile_mode;
	unsigned int film_grain_metadat_addr;
	unsigned int film_grain_metadat_size;
#endif
};

struct rtk_rpc_info {
	volatile void *vo_sync_flag;
#ifdef USE_ION
	int handle;
	struct dma_buf *dmabuf;
#endif
	void *vaddr;
	dma_addr_t paddr;
};

unsigned int ipcReadULONG(u8 *src);
void ipcCopyMemory(void *p_des, void *p_src, unsigned long len);

int rtk_rpc_init(struct device *dev, struct rtk_rpc_info *rpc_info);
int rpc_create_video_agent(struct rtk_rpc_info *rpc_info, unsigned int *videoId,
			   unsigned int pinId);

int rpc_video_display(struct rtk_rpc_info *rpc_info,
		      struct rpc_vo_filter_display *argp);
int rpc_video_config_disp_win(struct rtk_rpc_info *rpc_info,
			      struct rpc_config_disp_win *argp);
int rpc_video_set_refclock(struct rtk_rpc_info *rpc_info,
			   struct rpc_refclock *argp);
int rpc_video_init_ringbuffer(struct rtk_rpc_info *rpc_info,
			      struct rpc_ringbuffer *argp);
int rpc_video_run(struct rtk_rpc_info *rpc_info, unsigned int instance);
int rpc_send_vout_edid_data(struct rtk_rpc_info *rpc_info,
			 struct rpc_vout_edid_data *arg);
int rpc_send_audio_edid_data(struct rtk_rpc_info *rpc_info,
			 struct rpc_audio_edid_data *arg);
int rpc_send_hdmi_freq(struct rtk_rpc_info *rpc_info,
			 struct rpc_audio_hdmi_freq *arg);
int rpc_set_vrr(struct rtk_rpc_info *rpc_info,
			struct rpc_vout_hdmi_vrr *arg);
int rpc_query_tv_system(struct rtk_rpc_info *rpc_info,
			struct rpc_config_tv_system *arg);
int rpc_config_tv_system(struct rtk_rpc_info *rpc_info,
			 struct rpc_config_tv_system *arg);

#endif
