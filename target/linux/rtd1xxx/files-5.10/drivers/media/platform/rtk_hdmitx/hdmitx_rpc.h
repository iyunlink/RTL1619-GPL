/*
 * hdmitx_rpc.h - RTK hdmitx driver header file
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _HDMITX_RPC_H_
#define _HDMITX_RPC_H_

#define S_OK        0x10000000

typedef struct RPCRES_LONG {
	u_int result;
	u_int data;
} RPCRES_LONG;

typedef struct {
	u_int  info;
	RPCRES_LONG retval;
	u_int ret;
} RPC_DEFAULT_INPUT_T;

#define AUDIO_FRL_RATE1 0x00010000
#define AUDIO_FRL_RATE2 0x00020000
#define AUDIO_FRL_RATE3 0x00030000
struct AUDIO_HDMI_SET {
	u_int HDMI_Frequency;
};

struct AUDIO_HDMI_MUTE_INFO {
	u_int instanceID;
	char hdmi_mute;
};

struct AUDIO_HDMI_OUT_VSDB_DATA {
	u_int HDMI_VSDB_delay;
};

struct HDMI_INFO {
	u_int video_type;
};

/* From FW amw_kernel_rpc.h */
typedef enum {
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
	ENUM_KERNEL_RPC_CHECK_READY,   // check if Audio get memory pool from AP
	ENUM_KERNEL_RPC_GET_MUTE_N_VOLUME,	 // get mute and volume level
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
	ENUM_VIDEO_KERNEL_RPC_SET_HDMI_VRR
} ENUM_AUDIO_KERNEL_RPC_CMD;


/* Video */
enum VO_HDMI_MODE {
	VO_DVI_ON = 0,
	VO_HDMI_ON = 1,
	VO_HDMI_OFF = 2,
	VO_MHL_ON = 3,
	VO_MHL_OFF = 4,
};

enum VO_HDMI_OFF_MODE {
	VO_HDMI_OFF_CLOCK_OFF = 0,
	VO_HDMI_OFF_CLOCK_ON = 1,
};

enum VO_HDMI_AUDIO_SAMPLE_FREQ {
	VO_HDMI_AUDIO_NULL = 0,
	VO_HDMI_AUDIO_32K = 1,
	VO_HDMI_AUDIO_44_1K = 2,
	VO_HDMI_AUDIO_48K = 3,
	VO_HDMI_AUDIO_88_2K = 4,
	VO_HDMI_AUDIO_96K = 5,
	VO_HDMI_AUDIO_176_4K = 6,
	VO_HDMI_AUDIO_192K = 7,
};

enum VO_HDR_CTRL_MODE {
	VO_HDR_CTRL_AUTO = 0,
	VO_HDR_CTRL_DV_ON = 1,
	VO_HDR_CTRL_SDR = 2,
	VO_HDR_CTRL_HDR_GAMMA = 3,
	VO_HDR_CTRL_PQHDR = 4,
	VO_HDR_CTRL_FUTURE = 5,
	VO_HDR_CTRL_INPUT = 6,
	VO_HDR_CTRL_DV_LOW_LATENCY_12b_YUV422 = 7,
	VO_HDR_CTRL_DV_LOW_LATENCY_10b_YUV444 = 8,
	VO_HDR_CTRL_DV_LOW_LATENCY_10b_RGB444 = 9,
	VO_HDR_CTRL_DV_LOW_LATENCY_12b_YUV444 = 10,
	VO_HDR_CTRL_DV_LOW_LATENCY_12b_RGB444 = 11,
	VO_HDR_CTRL_DV_ON_INPUT = 12,
	VO_HDR_CTRL_DV_ON_LOW_LATENCY_12b422_INPUT = 13,
	VO_HDR_CTRL_INPUT_BT2020 = 14,
};

/* hdmi2px_feature */
#define HDMI2PX_2P0_MASK      0x1
#define HDMI2PX_2P0           0x1
#define HDMI2PX_SCRAMBLE_MASK 0x2
#define HDMI2PX_SCRAMBLE      0x2
#define HDMI2PX_FRLRATE_MASK  0x3C
#define HDMI2PX_FRL_3G3L      0x4
#define HDMI2PX_FRL_6G3L      0x8
#define HDMI2PX_FRL_6G4L      0xC

struct VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME {
	enum VO_HDMI_MODE hdmiMode;
	enum VO_HDMI_AUDIO_SAMPLE_FREQ audioSampleFreq;
	u_char audioChannelCount;
	u_char dataByte1;
	u_char dataByte2;
	u_char dataByte3;
	u_char dataByte4;
	u_char dataByte5;
	u_int dataInt0;
	/* hdmi2px_feature [Bit5:2] FRL Rate [Bit1]Scramble [Bit0]HDMI 2.x */
	u_int hdmi2px_feature;
	enum VO_HDMI_OFF_MODE hdmi_off_mode;
	enum VO_HDR_CTRL_MODE hdr_ctrl_mode;
	u_int reserved4;
};

/**
 * VO_INTERFACE_TYPE
 * --------------------
 *     | CVBS  HDMI  DP
 * --------------------
 * 0  |    O       O       X
 * 1  |    O       X       X
 * 2  |    X       O       X
 * 3  |    X       X       O
 * 4  |    X       O       O
 * 5  |    U       O       O
 * 6  |    O       X       O
 * 7  |    O       O       O
 * 8  |    U       O       X
 * --------------------
 * O: Enable, X: Disable, U: Enable or Disable
 */
enum VO_INTERFACE_TYPE {
	VO_ANALOG_AND_DIGITAL = 0,
	VO_ANALOG_ONLY = 1,
	VO_DIGITAL_ONLY = 2,
	VO_DISPLAY_PORT_ONLY = 3,
	VO_HDMI_AND_DISPLAY_PORT_SAME_SOURCE = 4,
	VO_HDMI_AND_DISPLAY_PORT_DIFF_SOURCE = 5,
	VO_DISPLAY_PORT_AND_CVBS_SAME_SOURCE = 6,
	VO_HDMI_AND_DP_DIFF_SOURCE_WITH_CVBS = 7,
	VO_FORCE_DP_OFF = 8,
};


enum VO_PEDESTAL_TYPE {
	VO_PEDESTAL_TYPE_300_700_ON = 0,
	VO_PEDESTAL_TYPE_300_700_OFF = 1,
	VO_PEDESTAL_TYPE_286_714_ON = 2,
	VO_PEDESTAL_TYPE_286_714_OFF = 3,
};


enum VO_STANDARD {
	VO_STANDARD_NTSC_M = 0,
	VO_STANDARD_NTSC_J = 1,
	VO_STANDARD_NTSC_443 = 2,
	VO_STANDARD_PAL_B = 3,
	VO_STANDARD_PAL_D = 4,
	VO_STANDARD_PAL_G = 5,
	VO_STANDARD_PAL_H = 6,
	VO_STANDARD_PAL_I = 7,
	VO_STANDARD_PAL_N = 8,
	VO_STANDARD_PAL_NC = 9,
	VO_STANDARD_PAL_M = 10,
	VO_STANDARD_PAL_60 = 11,
	VO_STANDARD_SECAM = 12,
	VO_STANDARD_HDTV_720P_60 = 13,
	VO_STANDARD_HDTV_720P_50 = 14,
	VO_STANDARD_HDTV_720P_30 = 15,
	VO_STANDARD_HDTV_720P_25 = 16,
	VO_STANDARD_HDTV_720P_24 = 17,
	VO_STANDARD_HDTV_1080I_60 = 18,
	VO_STANDARD_HDTV_1080I_50 = 19,
	VO_STANDARD_HDTV_1080P_30 = 20,
	VO_STANDARD_HDTV_1080P_25 = 21,
	VO_STANDARD_HDTV_1080P_24 = 22,
	VO_STANDARD_VGA = 23,
	VO_STANDARD_SVGA = 24,
	VO_STANDARD_HDTV_1080P_60 = 25,
	VO_STANDARD_HDTV_1080P_50 = 26,
	VO_STANDARD_HDTV_1080I_59 = 27,
	VO_STANDARD_HDTV_720P_59 = 28,
	VO_STANDARD_HDTV_1080P_23 = 29,
	VO_STANDARD_HDTV_1080P_59 = 30,
	VO_STANDARD_HDTV_1080P_60_3D = 31,
	VO_STANDARD_HDTV_1080P_50_3D = 32,
	VO_STANDARD_HDTV_1080P_30_3D = 33,
	VO_STANDARD_HDTV_1080P_24_3D = 34,
	VO_STANDARD_HDTV_720P_60_3D = 35,
	VO_STANDARD_HDTV_720P_50_3D = 36,
	VO_STANDARD_HDTV_720P_30_3D = 37,
	VO_STANDARD_HDTV_720P_24_3D = 38,
	VO_STANDARD_HDTV_720P_59_3D = 39,
	VO_STANDARD_HDTV_1080I_60_3D = 40,
	VO_STANDARD_HDTV_1080I_59_3D = 41,
	VO_STANDARD_HDTV_1080I_50_3D = 42,
	VO_STANDARD_HDTV_1080P_23_3D = 43,
	VO_STANDARD_HDTV_2160P_30 = 44,
	VO_STANDARD_HDTV_2160P_29 = 45,
	VO_STANDARD_HDTV_2160P_25 = 46,
	VO_STANDARD_HDTV_2160P_24 = 47,
	VO_STANDARD_HDTV_2160P_23 = 48,
	VO_STANDARD_HDTV_4096_2160P_24 = 49,
	VO_STANDARD_HDTV_2160P_60 = 50,
	VO_STANDARD_HDTV_2160P_50 = 51,
	VO_STANDARD_HDTV_4096_2160P_25 = 52,
	VO_STANDARD_HDTV_4096_2160P_30 = 53,
	VO_STANDARD_HDTV_4096_2160P_50 = 54,
	VO_STANDARD_HDTV_4096_2160P_60 = 55,
	VO_STANDARD_HDTV_2160P_60_420 = 56,
	VO_STANDARD_HDTV_2160P_50_420 = 57,
	VO_STANDARD_HDTV_4096_2160P_60_420 = 58,
	VO_STANDARD_HDTV_4096_2160P_50_420 = 59,
	VO_STANDARD_DP_FORMAT_1920_1080P_60 = 60,
	VO_STANDARD_DP_FORMAT_2160P_30 = 61,
	VO_STANDARD_HDTV_2160P_24_3D = 62,
	VO_STANDARD_HDTV_2160P_23_3D = 63,
	VO_STANDARD_HDTV_2160P_59 = 64,
	VO_STANDARD_HDTV_2160P_59_420 = 65,
	VO_STANDARD_HDTV_2160P_25_3D = 66,
	VO_STANDARD_HDTV_2160P_30_3D = 67,
	VO_STANDARD_HDTV_2160P_50_3D = 68,
	VO_STANDARD_HDTV_2160P_60_3D = 69,
	VO_STANDARD_HDTV_4096_2160P_24_3D = 70,
	VO_STANDARD_HDTV_4096_2160P_25_3D = 71,
	VO_STANDARD_HDTV_4096_2160P_30_3D = 72,
	VO_STANDARD_HDTV_4096_2160P_50_3D = 73,
	VO_STANDARD_HDTV_4096_2160P_60_3D = 74,
	VO_STANDARD_DP_FORMAT_1280_720P_60 = 75,
	VO_STANDARD_DP_FORMAT_3840_2160P_60 = 76,
	VO_STANDARD_DP_FORMAT_1024_768P_60 = 77,
	VO_STANDARD_HDTV_2160P_50_422_12bit = 78,
	VO_STANDARD_HDTV_2160P_60_422_12bit = 79,
	VO_STANDARD_DP_FORMAT_1280_800P_60 = 80,
	VO_STANDARD_DP_FORMAT_1440_900P_60 = 81,
	VO_STANDARD_DP_FORMAT_1440_768P_60 = 82,
	VO_STANDARD_DP_FORMAT_960_544P_60 = 83,
	VO_STANDARD_HDTV_720P_120_3D = 84,
	VO_STANDARD_DP_FORMAT_800_480P_60 = 85,
	VO_STANDARD_HDTV_2160P_59_422_12bit = 86,
	VO_STANDARD_DP_FORMAT_800_1280P_60 = 87,
	VO_STANDARD_DP_FORMAT_1280_720P_50 = 88,
	VO_STANDARD_HDTV_1080P_144 = 89,
	VO_STANDARD_HDTV_1024_768P_70 = 90,
	VO_STANDARD_DP_FORMAT_1024_600P_60 = 91,
	VO_STANDARD_DP_FORMAT_600_1024P_60 = 92,
	VO_STANDARD_DSIMIPI_FORMAT_1200_1920P_60 = 93,
	VO_STANDARD_HDTV_1080P_120 = 94,
	VO_STANDARD_HDTV_720P_P24 = 95,
	VO_STANDARD_HDTV_720P_P25 = 96,
	VO_STANDARD_HDTV_720P_P30 = 97,
	VO_STANDARD_HDTV_720P_P23 = 98,
	VO_STANDARD_HDTV_720P_P29 = 99,
	VO_STANDARD_DP_FORMAT_1366_768P_60 = 100,
	VO_STANDARD_HDTV_1080P_29 = 101,
	VO_STANDARD_ERROR = 102,
};


struct VIDEO_RPC_VOUT_CONFIG_VIDEO_STANDARD {
	enum VO_STANDARD standard;
	u_char enProg;
	u_char enDIF;
	u_char enCompRGB;
	enum VO_PEDESTAL_TYPE pedType;
	u_int dataInt0;
	u_int dataInt1;
};


struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM {
	enum VO_INTERFACE_TYPE interfaceType;
	struct VIDEO_RPC_VOUT_CONFIG_VIDEO_STANDARD videoInfo;
	struct VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME hdmiInfo;
};


struct AUDIO_HDMI_OUT_EDID_DATA2 {
	u_int Version;
	u_int HDMI_output_enable;
	u_int EDID_DATA_addr;
};


enum VO_VIDEO_PLANE {
	VO_VIDEO_PLANE_V1 = 0,
	VO_VIDEO_PLANE_V2 = 1,
	VO_VIDEO_PLANE_SUB1 = 2,
	VO_VIDEO_PLANE_OSD1 = 3,
	VO_VIDEO_PLANE_OSD2 = 4,
	VO_VIDEO_PLANE_WIN1 = 5,
	VO_VIDEO_PLANE_WIN2 = 6,
	VO_VIDEO_PLANE_WIN3 = 7,
	VO_VIDEO_PLANE_WIN4 = 8,
	VO_VIDEO_PLANE_WIN5 = 9,
	VO_VIDEO_PLANE_WIN6 = 10,
	VO_VIDEO_PLANE_WIN7 = 11,
	VO_VIDEO_PLANE_WIN8 = 12,
	VO_VIDEO_PLANE_NONE = 255,
};


enum VO_OSD_COLOR_FORMAT {
	VO_OSD_COLOR_FORMAT_2BIT = 0,
	VO_OSD_COLOR_FORMAT_4BIT = 1,
	VO_OSD_COLOR_FORMAT_8BIT = 2,
	VO_OSD_COLOR_FORMAT_RGB332 = 3,
	VO_OSD_COLOR_FORMAT_RGB565 = 4,
	VO_OSD_COLOR_FORMAT_ARGB1555 = 5,
	VO_OSD_COLOR_FORMAT_ARGB4444 = 6,
	VO_OSD_COLOR_FORMAT_ARGB8888 = 7,
	VO_OSD_COLOR_FORMAT_Reserved0 = 8,
	VO_OSD_COLOR_FORMAT_Reserved1 = 9,
	VO_OSD_COLOR_FORMAT_Reserved2 = 10,
	VO_OSD_COLOR_FORMAT_YCBCRA4444 = 11,
	VO_OSD_COLOR_FORMAT_YCBCRA8888 = 12,
	VO_OSD_COLOR_FORMAT_RGBA5551 = 13,
	VO_OSD_COLOR_FORMAT_RGBA4444 = 14,
	VO_OSD_COLOR_FORMAT_RGBA8888 = 15,
	VO_OSD_COLOR_FORMAT_420 = 16,
	VO_OSD_COLOR_FORMAT_422 = 17,
	VO_OSD_COLOR_FORMAT_RGB323 = 18,
	VO_OSD_COLOR_FORMAT_RGB233 = 19,
	VO_OSD_COLOR_FORMAT_RGB556 = 20,
	VO_OSD_COLOR_FORMAT_RGB655 = 21,
	VO_OSD_COLOR_FORMAT_RGB888 = 22,
	VO_OSD_COLOR_FORMAT_RGB565_LITTLE = 36,
	VO_OSD_COLOR_FORMAT_ARGB1555_LITTLE = 37,
	VO_OSD_COLOR_FORMAT_ARGB4444_LITTLE = 38,
	VO_OSD_COLOR_FORMAT_ARGB8888_LITTLE = 39,
	VO_OSD_COLOR_FORMAT_YCBCRA4444_LITTLE = 43,
	VO_OSD_COLOR_FORMAT_YCBCRA8888_LITTLE = 44,
	VO_OSD_COLOR_FORMAT_RGBA5551_LITTLE = 45,
	VO_OSD_COLOR_FORMAT_RGBA4444_LITTLE = 46,
	VO_OSD_COLOR_FORMAT_RGBA8888_LITTLE = 47,
	VO_OSD_COLOR_FORMAT_RGB556_LITTLE = 52,
	VO_OSD_COLOR_FORMAT_RGB655_LITTLE = 53,
	VO_OSD_COLOR_FORMAT_RGB888_LITTLE = 54,
};


enum VO_OSD_RGB_ORDER {
	VO_OSD_COLOR_RGB = 0,
	VO_OSD_COLOR_BGR = 1,
	VO_OSD_COLOR_GRB = 2,
	VO_OSD_COLOR_GBR = 3,
	VO_OSD_COLOR_RBG = 4,
	VO_OSD_COLOR_BRG = 5,
};


enum VO_3D_MODE_TYPE {
	VO_2D_MODE = 0,
	VO_3D_SIDE_BY_SIDE_HALF = 1,
	VO_3D_TOP_AND_BOTTOM = 2,
	VO_3D_FRAME_PACKING = 3,
};


struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_IN {
	enum VO_VIDEO_PLANE plane;
};


struct VO_RECTANGLE {
	short x;
	short y;
	u_short width;
	u_short height;
};


struct VO_SIZE {
	u_short w;
	u_short h;
};


struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_OUT {
	short result;
	enum VO_VIDEO_PLANE plane;
	u_short numWin;
	u_short zOrder;
	struct VO_RECTANGLE configWin;
	struct VO_RECTANGLE contentWin;
	short deintMode;
	u_short pitch;
	enum VO_OSD_COLOR_FORMAT colorType;
	enum VO_OSD_RGB_ORDER RGBOrder;
	enum VO_3D_MODE_TYPE format3D;
	struct VO_SIZE mix1_size;
	enum VO_STANDARD standard;
	u_char enProg;
	u_char reserved1;
	u_short reserved2;
	u_int reserved3;
};

struct VIDEO_RPC_VOUT_EDID_DATA {
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
	HDMI_QMS_MC0_COUNT = 3,
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
	/* When vrr_function = HDMI_QMS_MC0_COUNT */
	HDMI_QMS_MC0_1 = 1,
	HDMI_QMS_MC0_2 = 2,
	HDMI_QMS_MC0_3 = 3,
	HDMI_QMS_MC0_4 = 4,
	HDMI_QMS_MC0_5 = 5,
	HDMI_QMS_MC0_6 = 6,
	HDMI_QMS_MC0_7 = 7,
	HDMI_QMS_MC0_8 = 8,
	HDMI_QMS_MC0_9 = 9,
	HDMI_QMS_MC0_10 = 10,
	HDMI_QMS_MC0_11 = 11,
	HDMI_QMS_MC0_12 = 12,
	HDMI_QMS_MC0_13 = 13,
	HDMI_QMS_MC0_14 = 14,
	HDMI_QMS_MC0_15 = 15,
	HDMI_QMS_MC0_16 = 16,
	HDMI_QMS_MC0_17 = 17,
	HDMI_QMS_MC0_18 = 18,
	HDMI_QMS_MC0_19 = 19,
	HDMI_QMS_MC0_20 = 20,
};

/**
 * struct VIDEO_RPC_VOUT_HDMI_VRR -
 *   Parameter of RPC ENUM_VIDEO_KERNEL_RPC_SET_HDMI_VRR
 *
 * @vrr_function: enum ENUM_HDMI_VRR_FUNC_TYPE
 * @vrr_act: enum ENUM_HDMI_VRR_ACT
 */
struct VIDEO_RPC_VOUT_HDMI_VRR {
	enum ENUM_HDMI_VRR_FUNC_TYPE vrr_function;
	enum ENUM_HDMI_VRR_ACT vrr_act;
	int reserved[15];
};

#endif//_HDMITX_RPC_H_
