/*
 * hdmitx_info.c - RTK hdmitx driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/platform_device.h>

#include <drm/drm_edid.h>
#include <drm/drm_crtc.h>

#include "hdmitx.h"
#include "hdmitx_api.h"
#include "hdmitx_rpc.h"
#include "rtk_edid.h"

struct vic_format_info {
	unsigned char vic;
	unsigned int width;
	unsigned int height;
	unsigned char fps;
	unsigned char interlace;
};

const struct vic_format_info vic_format_table[] = {
	{ 0,                 0,    0,    0,   0},
	{ VIC_720X480P60,    720,  480,  60,  0},
	{ VIC_1280X720P60,   1280, 720,  60,  0},
	{ VIC_1920X1080I60,  1920, 1080, 60,  1},
	{ VIC_720X480I60,    720,  480,  60,  1},
	{ VIC_1920X1080P60,  1920, 1080, 60,  0},
	{ VIC_720X576P50,    720,  576,  50,  0},
	{ VIC_1280X720P50,   1280, 720,  50,  0},
	{ VIC_1920X1080I50,  1920, 1080, 50,  1},
	{ VIC_720X576I50,    720,  576,  50,  1},
	{ VIC_1920X1080P50,  1920, 1080, 50,  0},
	{ VIC_1920X1080P24,  1920, 1080, 24,  0},
	{ VIC_1920X1080P25,  1920, 1080, 25,  0},
	{ VIC_1920X1080P30,  1920, 1080, 30,  0},
	{ VIC_1280X720P24,   1280, 720,  24,  0},
	{ VIC_1280X720P25,   1280, 720,  25,  0},
	{ VIC_1280X720P30,   1280, 720,  30,  0},
	{ VIC_1920X1080P120, 1920, 1080, 120, 0},
	{ VIC_3840X2160P24,  3840, 2160, 24,  0},
	{ VIC_3840X2160P25,  3840, 2160, 25,  0},
	{ VIC_3840X2160P30,  3840, 2160, 30,  0},
	{ VIC_3840X2160P50,  3840, 2160, 50,  0},
	{ VIC_3840X2160P60,  3840, 2160, 60,  0},
	{ VIC_4096X2160P24,  4096, 2160, 24,  0},
	{ VIC_4096X2160P25,  4096, 2160, 25,  0},
	{ VIC_4096X2160P30,  4096, 2160, 30,  0},
	{ VIC_4096X2160P50,  4096, 2160, 50,  0},
	{ VIC_4096X2160P60,  4096, 2160, 60,  0},
	{ VIC_1920X1080P144, 1920, 1080, 144, 0},
};
const unsigned int vic_format_number = ARRAY_SIZE(vic_format_table);

/* EISA ID is based upon compressed ASCII */
const unsigned char eisa_id[] = {
	' ', 'A', 'B', 'C', 'D', 'E', 'F', 'G',
	'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
	'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W',
	'X', 'Y', 'Z', ' ', ' ', ' ', ' ', ' '
};

static const char * const mode[] = {
	"OFF", "DVI", "HDMI", "FRL"
};

static const char * const pixel_format[] = {
	"RGB", "YUV422", "YUV444", "YUV420"
};

static const char * const colorimetry[] = {
	"No Data", "SMPTE 170M", "ITU 709", "Extend"
};

static const char * const extended_colorimetry[] = {
	"xvYCC601", "xvYCC709", "sYCC601", "AdobeYCC601",
	"AdobeRGB", "BT2020YcCbcCrx", "BT2020RGB", "Reserved"
};

static const char * const format3d[] = {
	"N/A", "FramePacking", "Side-by-Side", "Top-and-Bottom",
	"FramePacking", "Side-by-Side", "Top-and-Bottom", "N/A"
};

static const char * const hdr_mode[] = {
	"AUTO", "DV", "SDR", "GAMMA",
	"HDR10", "FUTURE", "INPUT", "12B_YUV422",
	"10B_YUV444", "10B_RGB444", "12B_YUV444", "12B_RGB444",
	"DV_ON_INPUT", "DV_ON_LOW_LATENCY_12B422", "INPUT_BT2020", "DV_ON_HDR10_VS10",
	"DV_ON_SDR_VS10"
};

static const char * const it_cn_type[] = {
	"Graphics", "Photo",
	"Cinema", "Game"
};

static const char * const vo_type[] = {
	"ANALOG_AND_DIGITAL",
	"ANALOG_ONLY",
	"DIGITAL_ONLY",
	"DISPLAY_PORT_ONLY",
	"HDMI_AND_DISPLAY_PORT_SAME_SOURCE",
	"HDMI_AND_DISPLAY_PORT_DIFF_SOURCE",
	"DISPLAY_PORT_AND_CVBS_SAME_SOURCE",
	"HDMI_AND_DP_DIFF_SOURCE_WITH_CVBS",
	"FORCE_DP_OFF"
};

static const char * const frl_rate[] = {
	"Not supported", "3Gbps @ 3Lanes",
	"6Gbps @ 3Lanes", "6Gbps @ 4Lanes",
	"8Gbps @ 4Lanes", "10Gbps @ 4Lanes",
	"12Gbps @ 4Lanes", "Reserved"
};

static const char * const audio_format[] = {
	"StreamHeader", "L-PCM", "AC3", "MPEG-1",
	"MP3", "MPEG2", "AAC LC", "DTS",
	"ATRAC", "One Bit Audio", "Enhanced AC3", "DTS-HD",
	"MAT", "DST", "WMA Pro", "CXT"
};

static const char * const yes_no[] = {
	"No", "Yes"
};

unsigned char get_vic_format_table_index(unsigned char vic)
{
	int i;
	unsigned char index;

	index = 0;
	for (i = 0; i < vic_format_number; i++) {
		if (vic == vic_format_table[i].vic) {
			index = i;
			break;
		}
	}

	return index;
}

ssize_t show_hdmitx_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int ret_count;
	unsigned int vo_standard;
	unsigned char support_frl;
	unsigned char vic_index;
	unsigned char fps;
	unsigned char color_index;
	unsigned char vo_type_index;
	unsigned int hdmi2px_feature;
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM hdmitx_tv_system;
	struct hdmi_format_setting format;
	hdmitx_device_t *tx_dev;

	RPC_ToAgent_QueryConfigTvSystem(&hdmitx_tv_system);

	tv_system_to_hdmi_format(&hdmitx_tv_system, &format);

	ret_count = sprintf(buf, "------ HDMI Info -----\n");

	vo_standard = hdmitx_tv_system.videoInfo.standard;

	/* Support FRL: Yes/No */
	tx_dev = dev_get_drvdata(dev);
	if (tx_dev->frl)
		support_frl = tx_dev->frl->is_support_frl;
	else
		support_frl = 0;

	ret_count += sprintf(buf+ret_count, "Support FRL: %s\n",
		yes_no[support_frl]);

	/* Mode:DVI/HDMI/OFF */
	ret_count += sprintf(buf+ret_count, "Mode: %s\n",
		mode[format.mode&0x3]);

	/* RTK VO_STANDARD */
	ret_count += sprintf(buf+ret_count, "RTK VO_STANDARD: %u\n",
		vo_standard);

	/* VO interface type */
	vo_type_index = (unsigned char)hdmitx_tv_system.interfaceType;
	if (vo_type_index < 9)
		ret_count += sprintf(buf+ret_count, "VO type: %s\n",
			vo_type[vo_type_index]);
	else
		ret_count += sprintf(buf+ret_count, "VO type: Unknow\n");

	/* VIC */
	if (is_forbidden_vic(format.vic))
		ret_count += sprintf(buf+ret_count, "VIC: 0\n");
	else
		ret_count += sprintf(buf+ret_count, "VIC: %u\n",
			format.vic);

	/* Resolution: Width x Height I/P fps */
	vic_index = get_vic_format_table_index(format.vic);

	if (format.freq_shift)
		fps = vic_format_table[vic_index].fps - 1;
	else
		fps = vic_format_table[vic_index].fps;

	ret_count += sprintf(buf+ret_count, "Resolution: %ux%u%s @ %uHz\n",
		vic_format_table[vic_index].width,
		vic_format_table[vic_index].height,
		(hdmitx_tv_system.videoInfo.enProg&0x1)?"P":"I",
		fps);

	if (format.mode == FORMAT_MODE_OFF)
		return ret_count;

	/* Pixel format:RGB/YUV422/YUV444/YUV420 */
	ret_count += sprintf(buf+ret_count, "Pixel format: %s\n",
		pixel_format[format.color&0x3]);

	/* Color depth:8 Bits/10 Bits/12 Bits */
	ret_count += sprintf(buf+ret_count, "Color depth: %u Bits\n",
		format.color_depth);

	/* Colorimetry */
	color_index = (hdmitx_tv_system.hdmiInfo.dataByte2>>6)&0x3;
	if (color_index != 0x3) {
		/* Colorimetry C1:C0 */
		ret_count += sprintf(buf+ret_count, "Colorimetry: %s\n",
				colorimetry[color_index]);
	} else {
		/* Extended Colorimetry EC2:EC1:EC0 */
		color_index = (hdmitx_tv_system.hdmiInfo.dataByte3>>4)&0x7;

		ret_count += sprintf(buf+ret_count, "Colorimetry: %s\n",
				extended_colorimetry[color_index]);
	}

	/* 3D:"N/A"/FramePacking/Side-by-Side/Top-and-Bottom */
	ret_count += sprintf(buf+ret_count, "3D: %s\n",
		format3d[format._3d_format&0x7]);

	/* HDR mode */
	if (format.hdr <= HDR_MODE_DV_ON_SDR_VS10)
		ret_count += sprintf(buf+ret_count, "HDR mode: %s\n",
			hdr_mode[format.hdr]);
	else
		ret_count += sprintf(buf+ret_count, "HDR mode: %u\n",
			format.hdr);

	/* Content Type */
	if (format.misc & BIT(2))
		ret_count += sprintf(buf+ret_count, "IT content: %s\n",
			it_cn_type[format.misc&0x3]);
	else
		ret_count += sprintf(buf+ret_count, "IT content: No data\n");

	hdmi2px_feature = hdmitx_tv_system.hdmiInfo.hdmi2px_feature;
	/* HDMI2.0:YES/NO */
	ret_count += sprintf(buf+ret_count, "HDMI2.x: %s\n",
		yes_no[hdmi2px_feature&HDMI2PX_2P0_MASK ? 1:0]);

	/* Scramble:YES/NO */
	ret_count += sprintf(buf+ret_count, "Scramble: %s\n",
		yes_no[hdmi2px_feature&HDMI2PX_SCRAMBLE_MASK ? 1:0]);

	return ret_count;
}

ssize_t show_edid_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int ret_count;
	unsigned int vendor_id;
	unsigned char id_index[3];
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);
	asoc_hdmi_t *data = tx_dev->hdmi_data;
	struct edid_information *hdmitx_edid_info =
				 &tx_dev->hdmi_data->hdmitx_edid_info;

	ret_count = sprintf(buf, "------ EDID Info -----\n");

	if (!(data->sink_cap_available))
		return ret_count;

	vendor_id = hdmitx_edid_info->raw_edid->mfg_id[0];
	vendor_id = (vendor_id<<8) | hdmitx_edid_info->raw_edid->mfg_id[1];
	ret_count += sprintf(buf+ret_count, "Vendor ID:%04x\n", vendor_id);

	if (hdmitx_edid_info->hdmi_id == HDMI_2P1_IDENTIFIER)
		ret_count += sprintf(buf+ret_count, "HDMI version:2.1\n");
	else if (hdmitx_edid_info->hdmi_id == HDMI_2P0_IDENTIFIER)
		ret_count += sprintf(buf+ret_count, "HDMI version:2.0\n");
	else if (hdmitx_edid_info->hdmi_id != 0)
		ret_count += sprintf(buf+ret_count, "HDMI version:1.x\n");
	else
		ret_count += sprintf(buf+ret_count, "HDMI version:DVI\n");

	/* EISA ID for manufacturer name, using five-bit codes */
	id_index[0] = (vendor_id>>10)&0x1F;
	id_index[1] = (vendor_id>>5)&0x1F;
	id_index[2] = vendor_id&0x1F;
	ret_count += sprintf(buf+ret_count, "Manufacture name:%c%c%c\n",
					eisa_id[id_index[0]],
					eisa_id[id_index[1]],
					eisa_id[id_index[2]]);

	ret_count += sprintf(buf+ret_count, "ProductCode:%02x%02x\n",
					hdmitx_edid_info->raw_edid->prod_code[1],
					hdmitx_edid_info->raw_edid->prod_code[0]);

	ret_count += sprintf(buf+ret_count, "SerialNumber:%08x\n",
					hdmitx_edid_info->raw_edid->serial);

	ret_count += sprintf(buf+ret_count, "ManufactureYear:%u\n",
					1990+hdmitx_edid_info->raw_edid->mfg_year);

	ret_count += sprintf(buf+ret_count, "ManufactureWeek:%u\n",
					hdmitx_edid_info->raw_edid->mfg_week);

	ret_count += sprintf(buf+ret_count, "Monitor name:%s\n",
					hdmitx_edid_info->monitor_name);

	if (hdmitx_edid_info->cn_type & 0xf)
		ret_count += sprintf(buf+ret_count, "Content Type:%s%s%s%s\n",
			hdmitx_edid_info->cn_type&0x1 ? "Graphics,":"",
			hdmitx_edid_info->cn_type&0x2 ? "Photo,":"",
			hdmitx_edid_info->cn_type&0x4 ? "Cinema,":"",
			hdmitx_edid_info->cn_type&0x8 ? "Game":"");
	else
		ret_count += sprintf(buf+ret_count, "Content Type: None\n");

	if ((hdmitx_edid_info->hdmi_id == HDMI_2P0_IDENTIFIER) ||
		(hdmitx_edid_info->hdmi_id == HDMI_2P1_IDENTIFIER)) {
		ret_count += sprintf(buf+ret_count,
			"Max TMDS character rate:%u\n",
			hdmitx_edid_info->max_tmds_char_rate);

		ret_count += sprintf(buf+ret_count, "Deep Color 420:0x%x\n",
			hdmitx_edid_info->dc_420);

		if (hdmitx_edid_info->max_frl_rate <= 6)
			ret_count += sprintf(buf+ret_count, "Max FRL Rate:%s\n",
				frl_rate[hdmitx_edid_info->max_frl_rate]);
		else
			ret_count += sprintf(buf+ret_count, "Max FRL Rate:%u\n",
				hdmitx_edid_info->max_frl_rate);

		ret_count += sprintf(buf+ret_count, "ALLM:%s\n",
			yes_no[hdmitx_edid_info->allm&0x1]);

		ret_count += sprintf(buf+ret_count, "QMS:%s\n",
			yes_no[hdmitx_edid_info->vrr_feature&0x1 ? 1:0]);

		ret_count += sprintf(buf+ret_count, "QMS-TFRmin:%s\n",
			yes_no[hdmitx_edid_info->vrr_feature&0x2 ? 1:0]);

		ret_count += sprintf(buf+ret_count, "QMS-TFRmax:%s\n",
			yes_no[hdmitx_edid_info->vrr_feature&0x4 ? 1:0]);

		ret_count += sprintf(buf+ret_count, "VRR min:%u\n",
			hdmitx_edid_info->vrr_min);

		ret_count += sprintf(buf+ret_count, "VRR max:%u\n",
			hdmitx_edid_info->vrr_max);
	}
	return ret_count;
}

ssize_t show_audio_block(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret_count = 0;
	struct Audio_Data *data;
	unsigned int i;
	unsigned char coding_type;
	unsigned char rates;
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);
        struct edid_information *hdmitx_edid_info = 
				 &tx_dev->hdmi_data->hdmitx_edid_info;

	data = &hdmitx_edid_info->audio_data;

	ret_count += sprintf(buf+ret_count,
		"CodingType MaxChannels SamplingFreq SampleSize\n");

	if (data->ADB_length > 30)
		goto exit;

	for (i = 0; i < data->ADB_length/3; i++) {
		coding_type = data->ADB[i].coding_type;
		rates = data->ADB[i].sample_freq_all;

		ret_count += sprintf(buf+ret_count, "%s, %uch, ",
			audio_format[coding_type], data->ADB[i].channel_count);

		ret_count += sprintf(buf+ret_count, "%s%s%s%s%s%s%s kHz, ",
			rates&0x01 ? "32":"",
			rates&0x02 ? "/44.1":"",
			rates&0x04 ? "/48":"",
			rates&0x08 ? "/88.2":"",
			rates&0x10 ? "/96":"",
			rates&0x20 ? "/176.4":"",
			rates&0x40 ? "/192":"");

		if (coding_type == 1)
			ret_count += sprintf(buf+ret_count, "%s%s%s bit\n",
				data->ADB[i].sample_size_all & 0x1 ? "16":"",
				data->ADB[i].sample_size_all & 0x2 ? "/20":"",
				data->ADB[i].sample_size_all & 0x4 ? "/24":"");
		else
			ret_count += sprintf(buf+ret_count, "\n");
	}

exit:
	return ret_count;
}

ssize_t show_raw_edid(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret_count = 0;
	int ret_val;
	int i;
	unsigned char hpd_status;
	unsigned char edid[EDID_LENGTH*4];

	hpd_status = show_hpd_status(dev, false);
	if (!hpd_status)
		goto exit;

	ret_val = rtk_get_edid_block(dev, 0, 1, &edid[0]);
	if ((ret_val != 0) || (edid[0x7e] > 3))
		goto exit;

	ret_val = rtk_get_edid_block(dev, 1, edid[0x7e], &edid[128]);

	for (i = 0; i < (edid[0x7e]+1)*EDID_LENGTH; i += 16) {
		ret_count += sprintf(buf+ret_count,
			"%02x%02x%02x%02x%02x%02x%02x%02x",
			edid[i], edid[i+1], edid[i+2], edid[i+3],
			edid[i+4], edid[i+5], edid[i+6], edid[i+7]);
		ret_count += sprintf(buf+ret_count,
			"%02x%02x%02x%02x%02x%02x%02x%02x\n",
			edid[i+8], edid[i+9], edid[i+10], edid[i+11],
			edid[i+12], edid[i+13], edid[i+14], edid[i+15]);
	}

exit:
	return ret_count;
}

/* /sys/devices/platform/9800d000.hdmitx/hdmitx_info */
static DEVICE_ATTR(hdmitx_info, 0444, show_hdmitx_info, NULL);
/* /sys/devices/platform/9800d000.hdmitx/edid_info */
static DEVICE_ATTR(edid_info, 0444, show_edid_info, NULL);
/* /sys/devices/platform/9800d000.hdmitx/audio_block */
static DEVICE_ATTR(audio_block, 0444, show_audio_block, NULL);
/* /sys/devices/platform/9800d000.hdmitx/raw_edid */
static DEVICE_ATTR(raw_edid, 0444, show_raw_edid, NULL);

void register_hdmitx_sysfs(struct device *dev)
{
	device_create_file(dev, &dev_attr_hdmitx_info);
	device_create_file(dev, &dev_attr_edid_info);
	device_create_file(dev, &dev_attr_audio_block);
	device_create_file(dev, &dev_attr_raw_edid);
}

