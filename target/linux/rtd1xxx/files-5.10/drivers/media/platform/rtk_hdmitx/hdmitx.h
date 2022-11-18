/*
 * hdmitx.h - RTK hdmitx driver header file
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _HDMITX_H_
#define _HDMITX_H_

#include <drm/drm_edid.h>
#include <drm/drm_crtc.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "hdmitx_rpc.h"

#define __RTK_HDMI_GENERIC_DEBUG__  0

#if __RTK_HDMI_GENERIC_DEBUG__
#define HDMI_DEBUG(format, ...) printk("[HDMITx_DBG] " format "\n", ## __VA_ARGS__)
#else
#define HDMI_DEBUG(format, ...)
#endif

#define HDMI_ERROR(format, ...) printk(KERN_ERR "[HDMITx_ERR] " format "\n", ## __VA_ARGS__)
#define HDMI_INFO(format, ...) printk(KERN_WARNING "[HDMITx] " format "\n", ## __VA_ARGS__)

#define wr_reg(x, y)                    writel(y, (volatile unsigned int*)(x))
#define rd_reg(x)                       readl((volatile unsigned int*)(x))

#define field_get(val, start, end) 		(((val) & field_mask(start, end)) >> (end))
#define field_mask(start, end)          (((1 << (start - end + 1)) - 1) << (end))
#define setbits(base, offset, Mask)     wr_reg((base+offset), (rd_reg(base+offset) | Mask))
#define readbits(value, Mask)           ((value >> (Mask)) & 0x1)
#define clearbits(base, offset, Mask)	wr_reg((base+offset), (rd_reg(base+offset) & ~(Mask)))

#define WR_REG_32(base, offset, val)    writel(val, (volatile unsigned int*)(base + offset))
#define RD_REG_32(base, offset)         readl((volatile unsigned int*)(base + offset))

#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MASK(start, end)    (((1 << (start - end + 1)) - 1) << (end))
#define SETBITS(base, offset, Mask) WR_REG_32(base, offset, (RD_REG_32(base, offset) | Mask))
#define READBITS(value, Mask) ((value >> (Mask)) & 0x1)
#define CLEARBITS(base, offset, Mask) WR_REG_32(base, offset, ((RD_REG_32(base, offset) & ~(Mask))))
#define MASK_REG_32(base, offset, andMask, orMask) WR_REG_32(base, offset, ((RD_REG_32(base, offset) & (andMask)) | (orMask)))


#define MAX_ELD_BYTES	128

#define MAX_3D_VIC	16
#define MAX_SPEC_3D	18


enum HDMI_MODE {
	HDMI_MODE_UNDEF = 0,
	HDMI_MODE_HDMI = 1,
	HDMI_MODE_DVI = 2,
	HDMI_MODE_FRL = 3,
	HDMI_MODE_MAX
};

enum TMDS_MODE {
	TMDS_MODE_UNKNOW = 0,
	TMDS_HDMI_DISABLED = 1,
	TMDS_HDMI_ENABLED = 2,
	TMDS_MHL_ENABLED = 3,
	TMDS_MODE_MAX
};

enum HDMI_EXTENDED_MODE {
	EXTENDED_MODE_RESERVED = 0,
	EXTENDED_MODE_3840_2160_30HZ = 1,
	EXTENDED_MODE_3840_2160_25HZ = 2,
	EXTENDED_MODE_3840_2160_24HZ = 3,
	EXTENDED_MODE_4096_2160_24HZ = 4,
	EXTENDED_MODE_MAX
};

/* HDMI Forum Vendor Specific Data Block Byte6 */
enum SCDC_CAPABLES {
	SCDC_PRESENT = 0x80, /* Bit7 */
	SCDC_RR_CAPABLE = 0x40, /* Bit6*/
	SCDC_340M_SCRAMBLE = 0x08, /* Bit3 */
};

#define RTK_EDID_DIGITAL_DEPTH_6       (1 << 0)
#define RTK_EDID_DIGITAL_DEPTH_8       (1 << 1)
#define RTK_EDID_DIGITAL_DEPTH_10      (1 << 2)
#define RTK_EDID_DIGITAL_DEPTH_12      (1 << 3)
#define RTK_EDID_DIGITAL_DEPTH_14      (1 << 4)
#define RTK_EDID_DIGITAL_DEPTH_16      (1 << 5)

struct raw_edid {
	unsigned char edid_data[256];
};

/**
 * struct Audio_desc - Audio descriptor for audio format
 * @coding_type: Audio format code
 * @channel_count: Max number of channels
 * @sample_freq_all: Bit6[192KHz]/[176.4KHz]/[96KHz]/[88.2KHz]/[48KHz]/[44.1KHz]/Bit0[32KHz]
 * @sample_size_all: For Audio Format Code 1(L-PCM)
 * @max_bit_rate_divided_by_8KHz: For Audio Format Code 2 to 8
 */
struct Audio_desc {
	unsigned char	coding_type;
	unsigned char	channel_count;
	unsigned char	sample_freq_all;
	unsigned char	sample_size_all;
	unsigned char	max_bit_rate_divided_by_8KHz;
} __attribute__ ((packed));

struct Audio_Data {
	/* Audio Data Block */
	char ADB_length;
	struct Audio_desc ADB[10];
	/* Speaker Allocation Data Block */
	char SADB_length;
	unsigned char SADB[3];
} __attribute__ ((packed));

struct Specific_3D {
	unsigned char vic;
	unsigned char format;// 0:Frame packing, 6:Top-and-Bottom, 8:Side-by-Side(Half)
} __attribute__ ((packed));

struct Video_Display_Info {
	/* Physical size */
	unsigned int width_mm;
	unsigned int height_mm;
	unsigned int pixel_clock;
	unsigned int bpc;
	u32 color_formats;
	u8 cea_rev;
} __attribute__ ((packed));

struct sink_capabilities_t {
	unsigned int hdmi_mode;
	unsigned int est_modes;

	/* Audio */
	unsigned char eld[MAX_ELD_BYTES];
	struct Audio_Data audio_data;

	/* Vendor-Specific Data Block(VSDB) */
	unsigned char cec_phy_addr[2];
	bool support_AI; /* needs info from ACP or ISRC packets */
	bool DC_Y444; /* 4:4:4 in deep color modes */
	unsigned char color_space;
	bool dvi_dual; /* DVI Dual Link Operation */
	int max_tmds_clock;	/* in MHz */
	bool latency_present[2];
	unsigned char video_latency[2];	/* [0]: progressive, [1]: interlaced */
	unsigned char audio_latency[2];

	/* 3D */
	bool _3D_present;
	__u16 structure_all;
	unsigned char _3D_vic[MAX_3D_VIC];
	struct Specific_3D spec_3d[MAX_SPEC_3D];

	/* Video */
	struct Video_Display_Info display_info;
	__u64 vic;/* VIC 1~64, BIT0=VIC1 */
	unsigned char extended_vic;
	__u64 vic2;//VIC 65~128, BIT0=VIC65
	__u64 vic2_420;//YCbCr420 format, VIC 65~128, BIT0=VIC65
	struct VIDEO_RPC_VOUT_EDID_DATA vout_edid_data;
} __attribute__ ((packed));

struct edid_information {
	struct edid *raw_edid;
	/* ----- Base Block 128 Bytes End ---- */
	struct Audio_Data audio_data;
	u8 monitor_name[16];
	u32 hdmi_id;
	u32 max_tmds_char_rate;
	u8 colorimetry;
	u8 cn_type; /* Content Type */
	u8 scdc_capable;
	u8 dc_420;/* Deep Color420, [Bit2]DC_48bit_420;[Bit1]DC_36bit_420;[Bit0]:DC_30bit_420 */
	u8 max_frl_rate;
	u8 vrr_feature;/* [Bit2]QMS-TFRmax; [Bit1]QMS-TFRmin; [Bit0]QMS */
	u8 vrr_min;
	u16 vrr_max;
	u8 allm;
	u8 override_extdb_count;
} __attribute__((packed));

/**
 * struct hdmi_format_support - Color format and 3D support for specific VIC
 * @vic: Video ID Code
 * @rgb444: b'0-Support RGB444, b'1-RGB444/8bit, b'2-RGB444/10bit, b'3-RGB444/12bit
 * @yuv422: b'0-Support YUV422, b'1-YUV422/8bit, b'2-YUV422/10bit, b'3-YUV422/12bit
 * @yuv444: b'0-Support YUV444, b'1-YUV444/8bit, b'2-YUV444/10bit, b'3-YUV444/12bit
 * @yuv420: b'0-Support YUV420, b'1-YUV420/8bit, b'2-YUV420/10bit, b'3-YUV420/12bit
 * @support_3d: b'0-Support 3D, b'1-FramePacking, b'2-TopAndBottom, b'3-SideBySideHalf
 * @support_fs: b'0-Support Frequency-Shift, b'1-Support FS 3D
 */
struct hdmi_format_support {
	unsigned char vic;
	unsigned char rgb444;
	unsigned char yuv422;
	unsigned char yuv444;
	unsigned char yuv420;
	unsigned char support_3d;
	unsigned char support_fs;
	unsigned char reserved;
};

#define TXSUPPORT_SIZE 30
struct hdmi_support_list {
	struct hdmi_format_support tx_support[TXSUPPORT_SIZE];
	unsigned int tx_support_size;
};

enum HDMI_FORMAT_MODE {
	FORMAT_MODE_OFF = 0,
	FORMAT_MODE_DVI = 1,
	FORMAT_MODE_HDMI = 2,
	FORMAT_MODE_FRL = 3,
};

enum HDMI_VIDEO_ID_CODE {
	VIC_720X480P60 = 2,
	VIC_1280X720P60 = 4,
	VIC_1920X1080I60 = 5,
	VIC_720X480I60 = 6,
	VIC_1920X1080P60 = 16,
	VIC_720X576P50 = 17,
	VIC_1280X720P50 = 19,
	VIC_1920X1080I50 = 20,
	VIC_720X576I50 = 21,
	VIC_1920X1080P50 = 31,
	VIC_1920X1080P24 = 32,
	VIC_1920X1080P25 = 33,
	VIC_1920X1080P30 = 34,
	VIC_1280X720P24 = 60,
	VIC_1280X720P25 = 61,
	VIC_1280X720P30 = 62,
	VIC_1920X1080P120 = 63,
	VIC_3840X2160P24 = 93,
	VIC_3840X2160P25 = 94,
	VIC_3840X2160P30 = 95,
	VIC_3840X2160P50 = 96,
	VIC_3840X2160P60 = 97,
	VIC_4096X2160P24 = 98,
	VIC_4096X2160P25 = 99,
	VIC_4096X2160P30 = 100,
	VIC_4096X2160P50 = 101,
	VIC_4096X2160P60 = 102,
	/* Use forbidden VIC for non-CEA format */
	VIC_1920X1080P144 = 128,
};

enum HDMI_COLOR_MODE {
	COLOR_RGB = 0,
	COLOR_YUV422 = 1,
	COLOR_YUV444 = 2,
	COLOR_YUV420 =3,
};

enum HDMI_3D_MODE {
	FORMAT_3D_OFF = 0,
	/* RTK player Frame Packing */
	FORMAT_RTK_3D_FP = 1,
	/* RTK player Side by side half */
	FORMAT_RTK_3D_SS = 2,
	/* RTK player Top and Buttom */
	FORMAT_RTK_3D_TB = 3,
	/* Frame Packing */
	FORMAT_3D_FP = 4,
	/* Side by side half */
	FORMAT_3D_SS = 5,
	/* Top and Buttom */
	FORMAT_3D_TB = 6,
};

/**
 * enum HDMI_HDR_MODE - Same as enum VO_HDR_CTRL_MODE in hdmitx_rpc.h
 */
enum HDMI_HDR_MODE {
	/* Control by AudioFW */
	HDR_MODE_AUTO = 0,
	/* Dolby Vision */
	HDR_MODE_DV = 1,
	/* HDR OFF */
	HDR_MODE_SDR = 2,
	HDR_MODE_GAMMA = 3,
	/* HDR10 */
	HDR_MODE_HDR10 = 4,
	/* reserved future */
	HDR_MODE_FUTURE = 5,
	/* SDR/HDR by video */
	HDR_MODE_INPUT = 6,
	/* Dolby Vision, low latency */
	HDR_MODE_DV_12B_YUV422 = 7,
	HDR_MODE_DV_10B_YUV444 = 8,
	HDR_MODE_DV_10B_RGB444 = 9,
	HDR_MODE_DV_12B_YUV444 = 10,
	HDR_MODE_DV_12B_RGB444 = 11,
	/* Dolby Vision, RGB 8bit */
	HDR_MODE_DV_ON_INPUT = 12,
	HDR_MODE_DV_ON_LOW_LATENCY_12b422_INPUT = 13,
	/* INPUT_SDR2020 mode */
	HDR_MODE_HDR_CTRL_INPUT_BT2020 = 14,
	/* Output SDR/HDR10 via DV HW */
	HDR_MODE_DV_ON_HDR10_VS10 = 15,
	HDR_MODE_DV_ON_SDR_VS10 = 16,
};

/**
 * enum HDMI_CONTENT_TYPE - IT Contents Type of AVI Info Frame
 */
enum HDMI_CONTENT_TYPE {
	CONTENT_NONE = 0x0,
	CONTENT_GRAPHICS = 0x4,
	CONTENT_PHOTO = 0x5,
	CONTENT_CINEMA = 0x6,
	CONTENT_GAME = 0x7,
};

/**
 * struct hdmi_format_setting - HDMI output format setting
 * @mode: enum HDMI_FORMAT_MODE
 * @vic: Video ID code
 * @freq_shift: 0-No freq shift, 1-set freq shift
 * @color: enum HDMI_COLOR_MODE
 * @color_depth: 8-8bit, 10-10bit, 12-12bit
 * @_3d_format: enum HDMI_3D_MODE
 * @hdr: enum HDMI_HDR_MODE
 * @misc: Bit[2:0] enum hdmi_content_type
 */
struct hdmi_format_setting {
	unsigned char mode;
	unsigned char vic;
	unsigned char freq_shift;
	unsigned char color;
	unsigned char color_depth;
	unsigned char _3d_format;
	unsigned char hdr;
	unsigned char misc;
};

typedef struct {
	struct sink_capabilities_t sink_cap;
	bool en_default_edid;
	bool sink_cap_available;
	bool fake;
	struct edid_information hdmitx_edid_info;
	struct hdmi_support_list support_list;	
} asoc_hdmi_t;

struct ext_edid {
	int	extension;
	int current_blk;
	unsigned char data[2*EDID_LENGTH];
};

struct block_edid {
	unsigned char block_index;
	unsigned char block_size;
	unsigned char *edid_ptr;
} __attribute__ ((packed));

struct fake_edid {
	unsigned char *data_ptr;
	unsigned int size;
};

typedef struct rtk_hdmitx_frl {
        unsigned char is_support_frl;
        unsigned char current_frl_rate;
        unsigned char in_training;
} rtk_hdmitx_frl_t;

/**
 * enum rtk_ctrl5v_cmd -For ioctl HDMI_CONTROL_5V
 */
enum rtk_ctrl5v_cmd {
	CMD_GET_5V_STATE = 0,
	CMD_SET_5V_ON,
	CMD_SET_5V_OFF
};

#define HDMI_IOCTL_MAGIC 0xf1
#define HDMI_CHECK_LINK_STATUS			_IOWR(HDMI_IOCTL_MAGIC, 2, int)
#define HDMI_CHECK_Rx_Sense				_IOR(HDMI_IOCTL_MAGIC, 11, int)
#define HDMI_GET_EXT_BLK_COUNT			_IOR(HDMI_IOCTL_MAGIC, 12, int)
#define HDMI_GET_EXTENDED_EDID			_IOWR(HDMI_IOCTL_MAGIC, 13, struct ext_edid)
#define HDMI_QUERY_DISPLAY_STANDARD		_IOR(HDMI_IOCTL_MAGIC, 14, int)
#define HDMI_SEND_VOUT_EDID_DATA		_IOWR(HDMI_IOCTL_MAGIC, 15, struct VIDEO_RPC_VOUT_EDID_DATA)
#define HDMI_GET_EDID_SUPPORT_LIST		_IOWR(HDMI_IOCTL_MAGIC, 16, struct hdmi_support_list)
#define HDMI_SET_OUTPUT_FORMAT			_IOWR(HDMI_IOCTL_MAGIC, 17, struct hdmi_format_setting)
#define HDMI_GET_OUTPUT_FORMAT			_IOWR(HDMI_IOCTL_MAGIC, 18, struct hdmi_format_setting)
#define HDMI_SET_VO_INTERFACE_TYPE		_IOW(HDMI_IOCTL_MAGIC, 19, int)
#define HDMI_GET_CONFIG_TV_SYSTEM		_IOR(HDMI_IOCTL_MAGIC, 20, struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM)
#define HDMI_HOTPLUG_DETECTION			_IOWR(HDMI_IOCTL_MAGIC,21, int)
#define HDMI_WAIT_HOTPLUG				_IOWR(HDMI_IOCTL_MAGIC,22, int)
#define HDMI_GET_EDID_BLOCK             _IOWR(HDMI_IOCTL_MAGIC, 23, struct block_edid)
#define HDMI_SET_VRR                    _IOWR(HDMI_IOCTL_MAGIC, 24, struct VIDEO_RPC_VOUT_HDMI_VRR)
#define HDMI_CONTROL_5V                 _IOWR(HDMI_IOCTL_MAGIC, 25, int)
#define HDMI_SET_FAKE_EDID              _IOWR(HDMI_IOCTL_MAGIC, 26, struct fake_edid)


/* HDMI ioctl */
enum {
	HDMI_GET_SINK_CAPABILITY,
	HDMI_GET_RAW_EDID,
	HDMI_GET_LINK_STATUS,
	HDMI_GET_VIDEO_CONFIG,
	HDMI_SEND_AVMUTE,
	HDMI_CONFIG_TV_SYSTEM,
	HDMI_CONFIG_AVI_INFO,
	HDMI_SET_FREQUNCY,
	HDMI_SEND_AUDIO_MUTE,
	HDMI_SEND_AUDIO_VSDB_DATA,
	HDMI_SEND_AUDIO_EDID2,
};

enum HDMI_ERROR_CODE {
	HDMI_ERROR_RESERVED = 0,
	HDMI_ERROR_NO_MEMORY = 1,
	HDMI_ERROR_I2C_ERROR = 2,
	HDMI_ERROR_HPD = 3,
	HDMI_ERROR_INVALID_EDID = 4,
	HDMI_ERROR_MAX
};

enum HDMI_RXSENSE_MODE {
	RXSENSE_PASSIVE_MODE = 0,
	RXSENSE_TIMER_MODE,
	RXSENSE_INTERRUPT_MODE,
};

enum HDMI_RXSENSE_STATUS {
	HDMI_RXSENSE_OFF = 0,
	HDMI_RXSENSE_ON,
};

/* hdmitx_core */
extern void hdmitx_reset_clk(struct device *dev);
extern unsigned int hdmipll_read32(struct device *dev, unsigned int offset);
extern void hdmipll_write32(struct device *dev, unsigned int offset, unsigned int value);
extern void hdmipll_mask32(struct device *dev, unsigned int offset, unsigned int andMask,
	unsigned int orMask);
extern unsigned int hdmitx_read32(struct device *dev, unsigned int offset);
extern void hdmitx_write32(struct device *dev, unsigned int offset, unsigned int value);
extern void hdmitx_mask32(struct device *dev, unsigned int offset, unsigned int andMask,
	unsigned int orMask);
extern unsigned int hdmitop_read32(struct device *dev, unsigned int offset);
extern void hdmitop_write32(struct device *dev, unsigned int offset, unsigned int value);
extern void hdmitop_mask32(struct device *dev, unsigned int offset, unsigned int andMask,
	unsigned int orMask);

extern int hdmitx_send_AVmute(struct device *dev, int flag);

/* hdmitx_scdc */
extern int hdmitx_write_scdc_port(unsigned char offset, unsigned char value);
extern int hdmitx_read_scdc_port(unsigned char offset, unsigned char *buf, unsigned char len);

/* frl_phy_setting */
extern void frl_set_phy(struct device *dev,
	unsigned char link_rate, struct hdmi_format_setting *hdmi_format);

/* frl_link_training */
extern void set_frl_rate(unsigned char rate);
extern unsigned char get_frl_rate(void);
extern unsigned char is_in_training(void);
extern unsigned int frl_link_training(struct device *dev,
	unsigned char max_rate, struct hdmi_format_setting *hdmi_format);

/* switch_hdmitx */
extern int show_hpd_status(struct device *dev, bool real_time);
extern void hdmitx_hpd_debounce(struct device *dev, unsigned char ms);
int hdmitx_switch_get_state(struct device *dev);

/* hdmitx_info */
ssize_t show_hdmitx_info(struct device *cd, struct device_attribute *attr, char *buf);
ssize_t show_edid_info(struct device *cd, struct device_attribute *attr, char *buf);
void register_hdmitx_sysfs(struct device *dev);

/* rtk_mute_gpio */
void set_mute_gpio_pulse(struct device *dev);
#define I2S_OUT_ON 1
#define I2S_OUT_OFF 0
void set_i2s_output(struct device *dev, int gpio_value);
void setup_mute_gpio(struct device *dev);

/* hdmitx_config */
unsigned int gen_hdmi_format_support(struct hdmi_format_support *tx_support,
	struct sink_capabilities_t *sink_cap,
	struct edid_information *info);
unsigned int is_forbidden_vic(unsigned int vic);
int tv_system_to_hdmi_format(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *tv_system,
	struct hdmi_format_setting *format);
int set_hdmitx_format(struct device *dev, struct hdmi_format_setting *format);
int get_hdmitx_format(struct hdmi_format_setting *format);
void set_vo_interface_type(int type);
int ctrl_hdmi_5v(struct device *dev, enum rtk_ctrl5v_cmd cmd);
void register_support_list_sysfs(struct device *dev);

#if IS_ENABLED(CONFIG_RTK_HDCP_1x)
extern void ta_hdcp14_init(void);
extern int ta_hdcp_fix480p(void);
extern void ta_hdcp_lib_set_encryption(int enc_state);
extern void ta_hdcp_set_keepout_win(void);
#else
static inline void ta_hdcp14_init(void)
{
}

static inline int ta_hdcp_fix480p(void)
{
	return 0;
}

static inline void ta_hdcp_lib_set_encryption(int enc_state)
{
}

static inline void ta_hdcp_set_keepout_win(void)
{
}
#endif

#endif /* _HDMITX_H_ */
