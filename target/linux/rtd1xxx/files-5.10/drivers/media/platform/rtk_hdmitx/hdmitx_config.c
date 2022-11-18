/*
 * hdmitx_config.c - RTK hdmitx driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "hdmitx.h"
#include "hdmitx_dev.h"
#include "hdmitx_api.h"
#include "hdmitx_rpc.h"
#include "rtk_edid.h"
#include "hdmitx_scdc.h"
#include <linux/delay.h>
#include <linux/gpio/consumer.h>

#define xvYCC601 0x01
#define xvYCC709 0x02
#define BT2020_YCC 0x40
#define BT2020_RGB 0x80

int vo_interface_type = -1;

static const char * const string_list[] = {
	"0  - HDMI OFF",
	"6  - 720x480i/59.94Hz 4:3",
	"2  - 720x480p/59.94Hz 4:3",
	"21 - 720x576i/50Hz 4:3",
	"17 - 720x576p/50Hz 4:3",
	"19 - 1280x720p/50Hz 16:9",
	"4  - 1280x720p/59.94Hz/60Hz 16:9",
	"20 - 1920x1080i/50Hz 16:9",
	"5  - 1920x1080i/59.94Hz/60Hz 16:9",
	"32 - 1920x1080p/23.98Hz/24Hz 16:9",
	"33 - 1920x1080p/25Hz 16:9",
	"34 - 1920x1080p/30Hz 16:9",
	"31 - 1920x1080p/50Hz 16:9",
	"16 - 1920x1080p/59.94Hz/60Hz 16:9",
	"93 - 3840x2160p/23.98Hz/24Hz",
	"94 - 3840x2160p/25Hz",
	"95 - 3840x2160p/29.97Hz/30Hz",
	"96 - 3840x2160p/50Hz",
	"97 - 3840x2160p/59.94Hz/60Hz",
	"98 - 4096x2160p/24Hz",
	"99 - 4096x2160p/25Hz",
	"100- 4096x2160p/30Hz",
	"101- 4096x2160p/50Hz",
	"102- 4096x2160p/60Hz"
};

#define SUPPORT_VIC_SIZE 27
const char support_vic[SUPPORT_VIC_SIZE] = {
	VIC_720X480P60,
	VIC_1280X720P60,
	VIC_1920X1080I60,
	VIC_720X480I60,
	VIC_1920X1080P60,
	VIC_720X576P50,
	VIC_1280X720P50,
	VIC_1920X1080I50,
	VIC_720X576I50,
	VIC_1920X1080P50,
	VIC_1920X1080P24,
	VIC_1920X1080P25,
	VIC_1920X1080P30,
	VIC_1280X720P24,
	VIC_1280X720P25,
	VIC_1280X720P30,
	VIC_1920X1080P120,
	VIC_3840X2160P24,
	VIC_3840X2160P25,
	VIC_3840X2160P30,
	VIC_3840X2160P50,
	VIC_3840X2160P60,
	VIC_4096X2160P24,
	VIC_4096X2160P25,
	VIC_4096X2160P30,
	VIC_4096X2160P50,
	VIC_4096X2160P60,
};

static const char * const string_mode[] = {
	"OFF",
	"DVI",
	"HDMI",
	"FRL",
};

static unsigned char get_rgb444_support(unsigned char vic,
	struct sink_capabilities_t *sink_cap,
	struct edid_information *info,
	unsigned int max_tmds)
{
	unsigned int depth;
	unsigned char rgb444_support;
	unsigned char need_check_tmds;

	rgb444_support = BIT(1)|BIT(0);

	if ((vic == VIC_3840X2160P50) || (vic == VIC_3840X2160P60) ||
		(vic == VIC_4096X2160P50) || (vic == VIC_4096X2160P60))
		goto exit;

	if ((vic == VIC_3840X2160P24) || (vic == VIC_3840X2160P25) ||
		(vic == VIC_3840X2160P30) || (vic == VIC_4096X2160P24) ||
		(vic == VIC_4096X2160P25) || (vic == VIC_4096X2160P30) ||
		(vic == VIC_1920X1080P120))
		need_check_tmds = 1;
	else
		need_check_tmds = 0;

	/* Check deep color */
	depth = sink_cap->display_info.bpc;
	if (depth & RTK_EDID_DIGITAL_DEPTH_10) {
		if ((!need_check_tmds) || (need_check_tmds && (max_tmds > 371)))
			rgb444_support |= BIT(2);
	}

	if (depth & RTK_EDID_DIGITAL_DEPTH_12) {
		if ((!need_check_tmds) || (need_check_tmds && (max_tmds > 445)))
			rgb444_support |= BIT(3);
	}
exit:
	return rgb444_support;
}

static unsigned char get_yuv422_support(unsigned char vic,
	struct sink_capabilities_t *sink_cap,
	struct edid_information *info,
	unsigned int max_tmds)
{
	unsigned int color_formats;
	unsigned char yuv422_support;

	yuv422_support = 0;
	color_formats = sink_cap->display_info.color_formats;

	if (color_formats & DRM_COLOR_FORMAT_YCRCB422)
		yuv422_support |= (BIT(1)|BIT(0));
	else
		goto exit;

	/* Check deep color */
	if ((vic != VIC_4096X2160P50) && (vic != VIC_4096X2160P60)) {
			yuv422_support |= BIT(2);
			yuv422_support |= BIT(3);
	}

exit:
	return yuv422_support;
}

static unsigned char get_yuv444_support(unsigned char vic,
	struct sink_capabilities_t *sink_cap,
	struct edid_information *info,
	unsigned int max_tmds)
{
	unsigned int color_formats;
	unsigned int depth;
	unsigned char yuv444_support;
	unsigned char need_check_tmds;

	yuv444_support = 0;
	color_formats = sink_cap->display_info.color_formats;

	if (color_formats & DRM_COLOR_FORMAT_YCRCB444)
		yuv444_support |= (BIT(1)|BIT(0));
	else
		goto exit;

	if ((vic == VIC_3840X2160P50) || (vic == VIC_3840X2160P60) ||
		(vic == VIC_4096X2160P50) || (vic == VIC_4096X2160P60) ||
		(sink_cap->DC_Y444 == 0))
		goto exit;

	if ((vic == VIC_3840X2160P24) || (vic == VIC_3840X2160P25) ||
		(vic == VIC_3840X2160P30) || (vic == VIC_4096X2160P24) ||
		(vic == VIC_4096X2160P25) || (vic == VIC_4096X2160P30) ||
		(vic == VIC_1920X1080P120))
		need_check_tmds = 1;
	else
		need_check_tmds = 0;

	/* Check deep color */
	depth = sink_cap->display_info.bpc;
	if (depth & RTK_EDID_DIGITAL_DEPTH_10) {
		if ((!need_check_tmds) || (need_check_tmds && (max_tmds > 371)))
			yuv444_support |= BIT(2);
	}

	if (depth & RTK_EDID_DIGITAL_DEPTH_12) {
		if ((!need_check_tmds) || (need_check_tmds && (max_tmds > 445)))
			yuv444_support |= BIT(3);
	}
exit:
	return yuv444_support;
}

static unsigned char get_yuv420_support(unsigned char vic,
	struct sink_capabilities_t *sink_cap,
	struct edid_information *info,
	unsigned int max_tmds)
{
	unsigned char yuv420_support;

	yuv420_support = 0;
	if ((vic != VIC_3840X2160P50) && (vic != VIC_3840X2160P60) &&
		(vic != VIC_4096X2160P50) && (vic != VIC_4096X2160P60))
		goto exit;

	yuv420_support = BIT(1)|BIT(0);

	/* Check deep color */
	if ((info->dc_420&BIT(0)) && (max_tmds > 371))
		yuv420_support |= BIT(2);

	if ((info->dc_420&BIT(1)) && (max_tmds > 445))
		yuv420_support |= BIT(3);

exit:
	return yuv420_support;
}

static unsigned char get_3d_support(unsigned char vic,
	struct sink_capabilities_t *sink_cap,
	struct edid_information *info,
	unsigned int max_tmds)
{
	unsigned char _3d_support;
	unsigned char i;

	_3d_support = 0;
	if (sink_cap->_3D_present == 0)
		goto exit;

	for (i = 0; i < MAX_3D_VIC; i++) {
		if (sink_cap->_3D_vic[i] == vic) {
			/* frame packing */
			if ((sink_cap->structure_all & BIT(0)) &&
				(vic != 96) && (vic != 97) &&
				(vic != 101) && (vic != 102))
				_3d_support |= BIT(0)|BIT(1);

			/* top-and-bottom */
			if (sink_cap->structure_all & BIT(6))
				_3d_support |= BIT(0)|BIT(2);

			/* side-by-side half*/
			if (sink_cap->structure_all & BIT(8))
				_3d_support |= BIT(0)|BIT(3);

			break;
		}
	}

	for (i = 0; i < MAX_SPEC_3D; i++) {
		if (sink_cap->spec_3d[i].vic == vic) {
			switch (sink_cap->spec_3d[i].format) {
			case 0:
				_3d_support |= BIT(0)|BIT(1);
				break;
			case 6:
				_3d_support |= BIT(0)|BIT(2);
				break;
			case 8:
				_3d_support |= BIT(0)|BIT(3);
				break;
			default:
				break;
			}
			break;
		}
	}

exit:
	return _3d_support;
}

static unsigned char get_fs_support(unsigned char vic, unsigned char support_3d)
{
	unsigned char fs_support;

	switch (vic) {
	case VIC_1280X720P24:
	case VIC_1280X720P30:
	case VIC_1280X720P60:
	case VIC_1920X1080I60:
	case VIC_1920X1080P24:
	case VIC_1920X1080P30:
	case VIC_1920X1080P60:
	case VIC_3840X2160P24:
	case VIC_3840X2160P30:
	case VIC_3840X2160P60:
		fs_support = BIT(0);
		break;
	default:
		fs_support = 0;
		break;
	}

	if (support_3d == 0)
		goto exit;

	/* freq shift 3d */
	switch (vic) {
	case VIC_1280X720P60:
	case VIC_1920X1080I60:
	case VIC_1920X1080P24:
	case VIC_3840X2160P24:
		fs_support |= BIT(1);
		break;
	default:
		break;
	}

exit:
	return fs_support;
}

unsigned int gen_hdmi_format_support(struct hdmi_format_support *tx_support,
	struct sink_capabilities_t *sink_cap,
	struct edid_information *info)
{
	__u64 vic;
	unsigned char extended_vic;
	__u64 vic2;
	__u64 vic2_420;
	unsigned int max_tmds;
	unsigned int offset;
	unsigned int i;

	memset(tx_support, 0, sizeof(struct hdmi_format_support)*TXSUPPORT_SIZE);

	vic = sink_cap->vic;
	extended_vic = sink_cap->extended_vic;
	vic2 = sink_cap->vic2;
	vic2_420 = sink_cap->vic2_420;

	/* SD 16:9->4:3 */
	if (vic & BIT(2))
		vic |= BIT(1);
	if (vic & BIT(6))
		vic |= BIT(5);
	if (vic & BIT(17))
		vic |= BIT(16);
	if (vic & BIT(21))
		vic |= BIT(20);

	/* Merge extended_vic to vic2 */
	if (extended_vic & BIT(1))
		vic2 |= BIT(30);
	if (extended_vic & BIT(2))
		vic2 |= BIT(29);
	if (extended_vic & BIT(3))
		vic2 |= BIT(28);
	if (extended_vic & BIT(4))
		vic2 |= BIT_ULL(33);

	max_tmds = sink_cap->max_tmds_clock;
	if (info->max_tmds_char_rate > max_tmds)
		max_tmds = info->max_tmds_char_rate;

	if (max_tmds < 594)
		vic2 &= ~(BIT_ULL(31) | BIT_ULL(32) | BIT_ULL(36) | BIT_ULL(37));

	if (max_tmds < 297)
		vic2_420 = 0;

	offset = 0;
	for (i = 0; i < SUPPORT_VIC_SIZE; i++) {
		if (support_vic[i] <= 64) {

			if (vic & BIT_ULL(support_vic[i]-1)) {
				tx_support[offset].vic = support_vic[i];
				tx_support[offset].rgb444 = get_rgb444_support(support_vic[i], sink_cap, info, max_tmds);
				tx_support[offset].yuv422 = get_yuv422_support(support_vic[i], sink_cap, info, max_tmds);
				tx_support[offset].yuv444 = get_yuv444_support(support_vic[i], sink_cap, info, max_tmds);
				tx_support[offset].support_3d = get_3d_support(support_vic[i], sink_cap, info, max_tmds);
				tx_support[offset].support_fs = get_fs_support(support_vic[i], tx_support[offset].support_3d);
				offset++;
			}

		} else {

			if (vic2 & BIT_ULL(support_vic[i]-65)) {
				tx_support[offset].vic = support_vic[i];
				tx_support[offset].rgb444 = get_rgb444_support(support_vic[i], sink_cap, info, max_tmds);
				tx_support[offset].yuv422 = get_yuv422_support(support_vic[i], sink_cap, info, max_tmds);
				tx_support[offset].yuv444 = get_yuv444_support(support_vic[i], sink_cap, info, max_tmds);
				tx_support[offset].support_3d = get_3d_support(support_vic[i], sink_cap, info, max_tmds);
				tx_support[offset].support_fs = get_fs_support(support_vic[i], tx_support[offset].support_3d);

				if (vic2_420 & BIT_ULL(support_vic[i]-65))
					tx_support[offset].yuv420 = get_yuv420_support(support_vic[i], sink_cap, info, max_tmds);

				offset++;
			} else if (vic2_420 & BIT_ULL(support_vic[i]-65)) {
				/* Only support 420*/
				tx_support[offset].vic = support_vic[i];
				tx_support[offset].yuv420 = get_yuv420_support(support_vic[i], sink_cap, info, max_tmds);
				tx_support[offset].support_fs = get_fs_support(support_vic[i], 0);
				offset++;
			}

		}
	}

	return offset;
}

static unsigned int
vic_to_vo_standard(unsigned char vic, unsigned char freq_shift,
	unsigned char color, unsigned char color_depth, unsigned char en_3d)
{
	unsigned int ret_val;

	switch (vic) {
	case VIC_720X480P60:
		if (en_3d)
			ret_val = VO_STANDARD_ERROR;
		else
			ret_val = VO_STANDARD_NTSC_J;
		break;
	case VIC_1280X720P60:
		if (freq_shift) {
			if (en_3d)
				ret_val = VO_STANDARD_HDTV_720P_59_3D;
			else
				ret_val = VO_STANDARD_HDTV_720P_59;
		} else {
			if (en_3d)
				ret_val = VO_STANDARD_HDTV_720P_60_3D;
			else
				ret_val = VO_STANDARD_HDTV_720P_60;
		}
		break;
	case VIC_1920X1080I60:
		if (freq_shift) {
			if (en_3d)
				ret_val = VO_STANDARD_HDTV_1080I_59_3D;
			else
				ret_val = VO_STANDARD_HDTV_1080I_59;
		} else {
			if (en_3d)
				ret_val = VO_STANDARD_HDTV_1080I_60_3D;
			else
				ret_val = VO_STANDARD_HDTV_1080I_60;
		}
		break;
	case VIC_720X480I60:
		if (en_3d)
			ret_val = VO_STANDARD_ERROR;
		else
			ret_val = VO_STANDARD_NTSC_J;
		break;
	case VIC_1920X1080P60:
		if (freq_shift) {
			if (en_3d)
				ret_val = VO_STANDARD_ERROR;
			else
				ret_val = VO_STANDARD_HDTV_1080P_59;
		} else {
			if (en_3d)
				ret_val = VO_STANDARD_HDTV_1080P_60_3D;
			else
				ret_val = VO_STANDARD_HDTV_1080P_60;
		}
		break;
	case VIC_720X576P50:
		if (en_3d)
			ret_val = VO_STANDARD_ERROR;
		else
			ret_val = VO_STANDARD_PAL_I;
		break;
	case VIC_1280X720P50:
		if (freq_shift) {
			ret_val = VO_STANDARD_ERROR;
		} else {
			if (en_3d)
				ret_val = VO_STANDARD_HDTV_720P_50_3D;
			else
				ret_val = VO_STANDARD_HDTV_720P_50;
		}
		break;
	case VIC_1920X1080I50:
		if (freq_shift)
			ret_val = VO_STANDARD_ERROR;
		else if (en_3d)
			ret_val = VO_STANDARD_HDTV_1080I_50_3D;
		else
			ret_val = VO_STANDARD_HDTV_1080I_50;
		break;
	case VIC_720X576I50:
		if (en_3d)
			ret_val = VO_STANDARD_ERROR;
		else
			ret_val = VO_STANDARD_PAL_I;
		break;
	case VIC_1920X1080P50:
		if (freq_shift)
			ret_val = VO_STANDARD_ERROR;
		else if (en_3d)
			ret_val = VO_STANDARD_HDTV_1080P_50_3D;
		else
			ret_val = VO_STANDARD_HDTV_1080P_50;
		break;
	case VIC_1920X1080P24:
		if (freq_shift) {
			if (en_3d)
				ret_val = VO_STANDARD_HDTV_1080P_23_3D;
			else
				ret_val = VO_STANDARD_HDTV_1080P_23;
		} else {
			if (en_3d)
				ret_val = VO_STANDARD_HDTV_1080P_24_3D;
			else
				ret_val = VO_STANDARD_HDTV_1080P_24;
		}
		break;
	case VIC_1920X1080P25:
		if (freq_shift || en_3d)
			ret_val = VO_STANDARD_ERROR;
		else
			ret_val = VO_STANDARD_HDTV_1080P_25;
		break;
	case VIC_1920X1080P30:
		if (freq_shift)
			ret_val = VO_STANDARD_HDTV_1080P_29;
		else if (en_3d)
			ret_val = VO_STANDARD_HDTV_1080P_30_3D;
		else
			ret_val = VO_STANDARD_HDTV_1080P_30;
		break;
	case VIC_1280X720P24:
		if (en_3d)
			ret_val = VO_STANDARD_ERROR;
		else if (freq_shift)
			ret_val = VO_STANDARD_HDTV_720P_P23;
		else
			ret_val = VO_STANDARD_HDTV_720P_P24;
		break;
	case VIC_1280X720P25:
		if (freq_shift || en_3d)
			ret_val = VO_STANDARD_ERROR;
		else
			ret_val = VO_STANDARD_HDTV_720P_P25;
		break;
	case VIC_1280X720P30:
		if (en_3d)
			ret_val = VO_STANDARD_ERROR;
		else if (freq_shift)
			ret_val = VO_STANDARD_HDTV_720P_P29;
		else
			ret_val = VO_STANDARD_HDTV_720P_P30;
		break;
	case VIC_3840X2160P24:
		if (freq_shift) {
			if (en_3d)
				ret_val = VO_STANDARD_HDTV_2160P_23_3D;
			else
				ret_val = VO_STANDARD_HDTV_2160P_23;
		} else {
			if (en_3d)
				ret_val = VO_STANDARD_HDTV_2160P_24_3D;
			else
				ret_val = VO_STANDARD_HDTV_2160P_24;
		}
		break;
	case VIC_3840X2160P25:
		if (freq_shift)
			ret_val = VO_STANDARD_ERROR;
		else if (en_3d)
			ret_val = VO_STANDARD_HDTV_2160P_25_3D;
		else
			ret_val = VO_STANDARD_HDTV_2160P_25;
		break;
	case VIC_3840X2160P30:
		if (freq_shift) {
			if (en_3d)
				ret_val = VO_STANDARD_ERROR;
			else
				ret_val = VO_STANDARD_HDTV_2160P_29;
		} else {
			if (en_3d)
				ret_val = VO_STANDARD_HDTV_2160P_30_3D;
			else
				ret_val = VO_STANDARD_HDTV_2160P_30;
		}
		break;
	case VIC_3840X2160P50:
		if (color == COLOR_YUV420) {
			if (freq_shift || en_3d)
				ret_val = VO_STANDARD_ERROR;
			else
				ret_val = VO_STANDARD_HDTV_2160P_50_420;
		} else if ((color == COLOR_YUV422) &&
			((color_depth == 10) || (color_depth == 12))) {
			ret_val = VO_STANDARD_HDTV_2160P_50_422_12bit;
		} else {
			if (freq_shift) {
				ret_val = VO_STANDARD_ERROR;
			} else {
				if (en_3d)
					ret_val = VO_STANDARD_HDTV_2160P_50_3D;
				else
					ret_val = VO_STANDARD_HDTV_2160P_50;
			}
		}
		break;
	case VIC_3840X2160P60:
		if (color == COLOR_YUV420) {
			if (en_3d)
				ret_val = VO_STANDARD_ERROR;
			else if (freq_shift)
				ret_val = VO_STANDARD_HDTV_2160P_59_420;
			else
				ret_val = VO_STANDARD_HDTV_2160P_60_420;
		} else if ((color == COLOR_YUV422) &&
			((color_depth == 10) || (color_depth == 12))) {
			if (freq_shift)
				ret_val = VO_STANDARD_HDTV_2160P_59_422_12bit;
			else
				ret_val = VO_STANDARD_HDTV_2160P_60_422_12bit;
		} else {
			if (freq_shift && en_3d)
				ret_val = VO_STANDARD_ERROR;
			else if (freq_shift)
				ret_val = VO_STANDARD_HDTV_2160P_59;
			else if (en_3d)
				ret_val = VO_STANDARD_HDTV_2160P_60_3D;
			else
				ret_val = VO_STANDARD_HDTV_2160P_60;
		}
		break;
	case VIC_4096X2160P24:
		if (freq_shift)
			ret_val = VO_STANDARD_ERROR;
		else if (en_3d)
			ret_val = VO_STANDARD_HDTV_4096_2160P_24_3D;
		else
			ret_val = VO_STANDARD_HDTV_4096_2160P_24;
		break;
	case VIC_4096X2160P25:
		if (freq_shift)
			ret_val = VO_STANDARD_ERROR;
		else if (en_3d)
			ret_val = VO_STANDARD_HDTV_4096_2160P_25_3D;
		else
			ret_val = VO_STANDARD_HDTV_4096_2160P_25;
		break;
	case VIC_4096X2160P30:
		if (freq_shift)
			ret_val = VO_STANDARD_ERROR;
		else if (en_3d)
			ret_val = VO_STANDARD_HDTV_4096_2160P_30_3D;
		else
			ret_val = VO_STANDARD_HDTV_4096_2160P_30;
		break;
	case VIC_4096X2160P50:
		if (color == COLOR_YUV420) {
			if (freq_shift || en_3d)
				ret_val = VO_STANDARD_ERROR;
			else
				ret_val = VO_STANDARD_HDTV_4096_2160P_50_420;
		} else {
			if (freq_shift)
				ret_val = VO_STANDARD_ERROR;
			else if (en_3d)
				ret_val = VO_STANDARD_HDTV_4096_2160P_50_3D;
			else
				ret_val = VO_STANDARD_HDTV_4096_2160P_50;
		}
		break;
	case VIC_4096X2160P60:
		if (color == COLOR_YUV420) {
			if (freq_shift || en_3d)
				ret_val = VO_STANDARD_ERROR;
			else
				ret_val = VO_STANDARD_HDTV_4096_2160P_60_420;
		} else {
			if (freq_shift)
				ret_val = VO_STANDARD_ERROR;
			else if (en_3d)
				ret_val = VO_STANDARD_HDTV_4096_2160P_60_3D;
			else
				ret_val = VO_STANDARD_HDTV_4096_2160P_60;
		}
		break;
	case VIC_1920X1080P120:
		ret_val = VO_STANDARD_HDTV_1080P_120;
		break;
	case VIC_1920X1080P144:
		ret_val = VO_STANDARD_HDTV_1080P_144;
		break;
	default:
		ret_val = VO_STANDARD_ERROR;
		break;
	} /* end of switch (vic) */

	if (ret_val >= VO_STANDARD_ERROR)
		pr_err("Convert VO_STANDARD fail, vic(%u) freq_shift(%u) color(%u) 3d(%u)",
			vic, freq_shift, color, en_3d);

	return ret_val;
}

static unsigned char vo_standard_to_vic(unsigned int vo_standard, unsigned char is_interlace)
{
	unsigned char vic;

	switch (vo_standard) {
	case VO_STANDARD_NTSC_J:
		if (is_interlace)
			vic = VIC_720X480I60;
		else
			vic = VIC_720X480P60;
		break;
	case VO_STANDARD_PAL_I:
		if (is_interlace)
			vic = VIC_720X576I50;
		else
			vic = VIC_720X576P50;
		break;
	case VO_STANDARD_HDTV_720P_P23:
	case VO_STANDARD_HDTV_720P_P24:
		vic = VIC_1280X720P24;
		break;
	case VO_STANDARD_HDTV_720P_P25:
		vic = VIC_1280X720P25;
		break;
	case VO_STANDARD_HDTV_720P_P29:
	case VO_STANDARD_HDTV_720P_P30:
		vic = VIC_1280X720P30;
		break;
	case VO_STANDARD_HDTV_720P_50:
	case VO_STANDARD_HDTV_720P_50_3D:
		vic = VIC_1280X720P50;
		break;
	case VO_STANDARD_HDTV_720P_59:
	case VO_STANDARD_HDTV_720P_60:
	case VO_STANDARD_HDTV_720P_59_3D:
	case VO_STANDARD_HDTV_720P_60_3D:
		vic = VIC_1280X720P60;
		break;
	case VO_STANDARD_HDTV_1080I_50:
	case VO_STANDARD_HDTV_1080I_50_3D:
		vic = VIC_1920X1080I50;
		break;
	case VO_STANDARD_HDTV_1080P_23:
	case VO_STANDARD_HDTV_1080P_24:
	case VO_STANDARD_HDTV_1080P_23_3D:
	case VO_STANDARD_HDTV_1080P_24_3D:
		vic = VIC_1920X1080P24;
		break;
	case VO_STANDARD_HDTV_1080P_25:
		vic = VIC_1920X1080P25;
		break;
	case VO_STANDARD_HDTV_1080P_29:
	case VO_STANDARD_HDTV_1080P_30:
	case VO_STANDARD_HDTV_1080P_30_3D:
		vic = VIC_1920X1080P30;
		break;
	case VO_STANDARD_HDTV_1080P_50:
	case VO_STANDARD_HDTV_1080P_50_3D:
		vic = VIC_1920X1080P50;
		break;
	case VO_STANDARD_HDTV_1080I_59:
	case VO_STANDARD_HDTV_1080I_60:
	case VO_STANDARD_HDTV_1080I_59_3D:
	case VO_STANDARD_HDTV_1080I_60_3D:
		vic = VIC_1920X1080I60;
		break;
	case VO_STANDARD_HDTV_1080P_59:
	case VO_STANDARD_HDTV_1080P_60:
	case VO_STANDARD_HDTV_1080P_60_3D:
		vic = VIC_1920X1080P60;
		break;
	case VO_STANDARD_HDTV_2160P_23:
	case VO_STANDARD_HDTV_2160P_24:
	case VO_STANDARD_HDTV_2160P_23_3D:
	case VO_STANDARD_HDTV_2160P_24_3D:
		vic = VIC_3840X2160P24;
		break;
	case VO_STANDARD_HDTV_2160P_25:
	case VO_STANDARD_HDTV_2160P_25_3D:
		vic = VIC_3840X2160P25;
		break;
	case VO_STANDARD_HDTV_2160P_29:
	case VO_STANDARD_HDTV_2160P_30:
	case VO_STANDARD_HDTV_2160P_30_3D:
		vic = VIC_3840X2160P30;
		break;
	case VO_STANDARD_HDTV_2160P_50:
	case VO_STANDARD_HDTV_2160P_50_420:
	case VO_STANDARD_HDTV_2160P_50_3D:
	case VO_STANDARD_HDTV_2160P_50_422_12bit:
		vic = VIC_3840X2160P50;
		break;
	case VO_STANDARD_HDTV_2160P_59:
	case VO_STANDARD_HDTV_2160P_60:
	case VO_STANDARD_HDTV_2160P_59_420:
	case VO_STANDARD_HDTV_2160P_60_420:
	case VO_STANDARD_HDTV_2160P_60_3D:
	case VO_STANDARD_HDTV_2160P_60_422_12bit:
	case VO_STANDARD_HDTV_2160P_59_422_12bit:
		vic = VIC_3840X2160P60;
		break;
	case VO_STANDARD_HDTV_4096_2160P_24:
	case VO_STANDARD_HDTV_4096_2160P_24_3D:
		vic = VIC_4096X2160P24;
		break;
	case VO_STANDARD_HDTV_4096_2160P_25:
	case VO_STANDARD_HDTV_4096_2160P_25_3D:
		vic = VIC_4096X2160P25;
		break;
	case VO_STANDARD_HDTV_4096_2160P_30:
	case VO_STANDARD_HDTV_4096_2160P_30_3D:
		vic = VIC_4096X2160P30;
		break;
	case VO_STANDARD_HDTV_4096_2160P_50:
	case VO_STANDARD_HDTV_4096_2160P_50_420:
	case VO_STANDARD_HDTV_4096_2160P_50_3D:
		vic = VIC_4096X2160P50;
		break;
	case VO_STANDARD_HDTV_4096_2160P_60:
	case VO_STANDARD_HDTV_4096_2160P_60_420:
	case VO_STANDARD_HDTV_4096_2160P_60_3D:
		vic = VIC_4096X2160P60;
		break;
	case VO_STANDARD_HDTV_1080P_120:
		vic = VIC_1920X1080P120;
		break;
	case VO_STANDARD_HDTV_1080P_144:
		vic = VIC_1920X1080P144;
		break;
	default:
		vic = 0;
		pr_err("%s fail", __func__);
	}

	return vic;
}

static unsigned char is_freq_shift(unsigned int vo_standard)
{
	unsigned int ret_val;

	switch (vo_standard) {
	case VO_STANDARD_HDTV_720P_P23:
	case VO_STANDARD_HDTV_720P_P29:
	case VO_STANDARD_HDTV_720P_59:
	case VO_STANDARD_HDTV_720P_59_3D:
	case VO_STANDARD_HDTV_1080I_59:
	case VO_STANDARD_HDTV_1080I_59_3D:
	case VO_STANDARD_HDTV_1080P_23:
	case VO_STANDARD_HDTV_1080P_23_3D:
	case VO_STANDARD_HDTV_1080P_29:
	case VO_STANDARD_HDTV_1080P_59:
	case VO_STANDARD_HDTV_2160P_23:
	case VO_STANDARD_HDTV_2160P_23_3D:
	case VO_STANDARD_HDTV_2160P_29:
	case VO_STANDARD_HDTV_2160P_59:
	case VO_STANDARD_HDTV_2160P_59_420:
	case VO_STANDARD_HDTV_2160P_59_422_12bit:
		ret_val = 1;
		break;
	default:
		ret_val = 0;
	}

	return ret_val;
}

unsigned int is_forbidden_vic(unsigned int vic)
{
	if ((vic > 255) || ((vic >= 128) && (vic <= 192)))
		return 1;
	else
		return 0;
}

static unsigned int is_interlace_mode(unsigned int vic)
{
	unsigned int ret_val;

	switch (vic) {
	case VIC_1920X1080I60:
	case VIC_720X480I60:
	case VIC_1920X1080I50:
	case VIC_720X576I50:
		ret_val = 1;
		break;
	default:
		ret_val = 0;
		break;
	}

	return ret_val;
}

static void set_colorimetry(struct edid_information *hdmitx_edid_info,
			unsigned int color_mode, 
			struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *tv_system)
{
	unsigned int standard;
	unsigned char data_byte2;
	unsigned char data_byte3;
	unsigned char colorimetry;
	

	standard = tv_system->videoInfo.standard;
	/* mask out C1:C0 */
	data_byte2 = tv_system->hdmiInfo.dataByte2&0x3F;
	/* mask out EC2:EC1:EC0 */
	data_byte3 = tv_system->hdmiInfo.dataByte3&0x8F;

	colorimetry = hdmitx_edid_info->colorimetry;

	if (color_mode == COLOR_RGB) {

		if (colorimetry&BT2020_RGB) {
			data_byte2 |= 0xC0;
			data_byte3 |= 0x60;
		} else if (standard >= VO_STANDARD_HDTV_720P_60) {
			data_byte2 |= 0x80;/* BT.709 */
		} else {
			data_byte2 |= 0x40;/* SMPTE170M */
		}

	} else {

		if (colorimetry&BT2020_YCC) {
			data_byte2 |= 0xC0;
			data_byte3 |= 0x60;
		} else if ((colorimetry&xvYCC709) &&
			(standard >= VO_STANDARD_HDTV_720P_60)) {
			data_byte2 |= 0xC0;
			data_byte3 |= 0x10;
		} else if ((colorimetry&xvYCC601) &&
			(standard < VO_STANDARD_HDTV_720P_60)) {
			data_byte2 |= 0xC0;
			data_byte3 |= 0x00;
		} else if (standard >= VO_STANDARD_HDTV_720P_60) {
			data_byte2 |= 0x80;
		} else {
			data_byte2 |= 0x40;
		}

	}

	tv_system->hdmiInfo.dataByte2 = data_byte2;
	tv_system->hdmiInfo.dataByte3 = data_byte3;

}

static void set_aspect_ratio(unsigned int vic, struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *tv_system)
{
	unsigned char data_byte2;

	/* mask out M1:R0 */
	data_byte2 = tv_system->hdmiInfo.dataByte2&0xC0;

	switch (vic) {
	case 2:
	case 6:
	case 17:
	case 21:
		data_byte2 |= 0x18;
		break;
	default:
		data_byte2 |= 0x28;
		break;
	}

	tv_system->hdmiInfo.dataByte2 = data_byte2;
}

static void config_avi_infoframe(struct edid_information *hdmitx_edid_info,
			struct hdmi_format_setting *format,
			struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *tv_system)
{
	unsigned char data_byte1;
	unsigned char data_byte4;
	unsigned char cn_valid;

	/* Data Byte1 */
	switch (format->color) {
	case COLOR_RGB:
		data_byte1 = 0x0;
		break;
	case COLOR_YUV422:
		data_byte1 = 0x20;
		break;
	case COLOR_YUV444:
		data_byte1 = 0x40;
		break;
	case COLOR_YUV420:
		data_byte1 = 0x60;
		break;
	default:
		data_byte1 = 0x0;
		pr_err("Unknown color_mode");
		break;
	}

	tv_system->hdmiInfo.dataByte1 = data_byte1;

	/* Data Byte2, Data Byte3 */
	set_colorimetry(hdmitx_edid_info, format->color, tv_system);
	set_aspect_ratio(format->vic, tv_system);

	/* Data Byte4 */
	if (is_forbidden_vic(format->vic))
		data_byte4 = 0;
	else
		data_byte4 = (unsigned char)format->vic;

	tv_system->hdmiInfo.dataByte4 = data_byte4;

	/* IT content */
	cn_valid = format->misc & 0x4;
	if (cn_valid) {
		unsigned char data_byte5;
		unsigned char cn;

		data_byte5 = tv_system->hdmiInfo.dataByte5 & ~(0x3 << 4);
		cn = format->misc & 0x3;
		data_byte5 |= cn << 4;

		tv_system->hdmiInfo.dataByte3 |= BIT(7);
		tv_system->hdmiInfo.dataByte5 = data_byte5;
	}

}

static void config_video_dataint0(unsigned char _3d_format, unsigned int vic,
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *tv_system)
{
	unsigned int video_dataint0;

	video_dataint0 = 0;

	/*
	 * videoInfo.dataInt0: [BIT15:12]=3d_src [BIT11:8]=3d_dest
	 * RTK DVD player, 3d_dest always set to 1
	 * Other player, 3d_src always set to 0
	 */
	switch (_3d_format) {
	case FORMAT_RTK_3D_FP:
		video_dataint0 |= 0x1100;
		break;
	case FORMAT_RTK_3D_SS:
		video_dataint0 |= 0x2100;
		break;
	case FORMAT_RTK_3D_TB:
		video_dataint0 |= 0x3100;
		break;
	case FORMAT_3D_FP:
		video_dataint0 |= 0x0100;
		break;
	case FORMAT_3D_SS:
		video_dataint0 |= 0x0200;
		break;
	case FORMAT_3D_TB:
		video_dataint0 |= 0x0300;
		break;
	default:
		break;
	}

	switch (vic) {
	case VIC_720X576P50:
	case VIC_1280X720P50:
	case VIC_1920X1080I50:
	case VIC_720X576I50:
	case VIC_1920X1080P50:
	case VIC_1920X1080P25:
	case VIC_3840X2160P25:
	case VIC_3840X2160P50:
	case VIC_4096X2160P25:
	case VIC_4096X2160P50:
		/* CVBS PAL */
		video_dataint0 |= 0x2;
		break;
	default:
		/* CVBS NTSC */
		video_dataint0 |= 0x4;
		break;
	}

	tv_system->videoInfo.dataInt0 = video_dataint0;
}

static void config_interface_type(struct device *dev,
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *tv_system)
{
	int ret;
	unsigned int hdmiMode;
	unsigned int interfaceType;
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM cur_tv_system;

	if (!dp_or_dsi_exist)
		goto exit;

	ret = RPC_ToAgent_QueryConfigTvSystem(&cur_tv_system);

	if (ret != 0)
		goto fail;

	if (vo_interface_type >= 0) {
		interfaceType = vo_interface_type;
		goto config;
	}

	dev_info(dev, "Auto select interface type");
	hdmiMode = tv_system->hdmiInfo.hdmiMode;

	switch (cur_tv_system.interfaceType) {
	case VO_ANALOG_AND_DIGITAL:
	case VO_ANALOG_ONLY:
	case VO_DIGITAL_ONLY:
		interfaceType = cur_tv_system.interfaceType;
		break;
	case VO_DISPLAY_PORT_ONLY:
		if (hdmiMode != VO_HDMI_OFF)
			interfaceType = VO_HDMI_AND_DISPLAY_PORT_SAME_SOURCE;
		else
			interfaceType = VO_DISPLAY_PORT_ONLY;
		break;
	case VO_HDMI_AND_DISPLAY_PORT_SAME_SOURCE:
		if (hdmiMode != VO_HDMI_OFF)
			interfaceType = VO_HDMI_AND_DISPLAY_PORT_SAME_SOURCE;
		else
			interfaceType = VO_DISPLAY_PORT_ONLY;
		break;
	case VO_HDMI_AND_DISPLAY_PORT_DIFF_SOURCE:
			interfaceType = VO_HDMI_AND_DISPLAY_PORT_DIFF_SOURCE;
		break;
	case VO_DISPLAY_PORT_AND_CVBS_SAME_SOURCE:
		if (hdmiMode != VO_HDMI_OFF)
			interfaceType = VO_HDMI_AND_DISPLAY_PORT_SAME_SOURCE;
		else
			interfaceType = VO_DISPLAY_PORT_AND_CVBS_SAME_SOURCE;
		break;
	case VO_HDMI_AND_DP_DIFF_SOURCE_WITH_CVBS:
	case VO_FORCE_DP_OFF:
		interfaceType = cur_tv_system.interfaceType;
		break;
	default:
		interfaceType = cur_tv_system.interfaceType;
		dev_info(dev, "Unknown interfaceType %u",
			cur_tv_system.interfaceType);
		break;
	}

config:
	tv_system->interfaceType = interfaceType;
	tv_system->videoInfo.pedType = cur_tv_system.videoInfo.pedType;

	dev_info(dev, "interfaceType %u -> %u, pedType %u",
		cur_tv_system.interfaceType, tv_system->interfaceType,
		tv_system->videoInfo.pedType);
exit:
	return;
fail:
	dev_info(dev, "%s fail", __func__);
}

/**
 * tv_system_to_hdmi_format - Convert struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM to
 *                                             struct hdmi_format_setting
 * @tv_system: Input struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM
 * @format: Output struct hdmi_format_setting
 *
 * Return: 0 on success, other on failure
 */
int tv_system_to_hdmi_format(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *tv_system,
	struct hdmi_format_setting *format)
{
	int ret;
	unsigned char is_interlace;
	unsigned char dataInt0_depth;
	unsigned char dataInt0_3d;
	unsigned char content_type;
	unsigned int hdmi2px_feature;

	ret = 0;
	memset(format, 0, sizeof(struct hdmi_format_setting));

	switch (tv_system->hdmiInfo.hdmiMode) {
	case VO_HDMI_OFF:
		format->mode = FORMAT_MODE_OFF;
		break;
	case VO_DVI_ON:
		format->mode = FORMAT_MODE_DVI;
		break;
	case VO_HDMI_ON:
		hdmi2px_feature = tv_system->hdmiInfo.hdmi2px_feature;
		if ((hdmi2px_feature & HDMI2PX_FRLRATE_MASK) != 0)
			format->mode = FORMAT_MODE_FRL;
		else
			format->mode = FORMAT_MODE_HDMI;
		break;
	default:
		ret = -1;
		pr_err("Convert fail, unknown hdmiMode\n");
	}

	if (tv_system->videoInfo.enProg)
		is_interlace = 0;
	else
		is_interlace = 1;

	format->vic = vo_standard_to_vic(tv_system->videoInfo.standard, is_interlace);

	format->freq_shift = is_freq_shift(tv_system->videoInfo.standard);

	format->color = (tv_system->hdmiInfo.dataByte1>>5)&0x3;

	dataInt0_depth = (tv_system->hdmiInfo.dataInt0>>2)&0xF;
	switch (dataInt0_depth) {
	case 5:
		format->color_depth = 10;
		break;
	case 6:
		format->color_depth = 12;
		break;
	default:
		format->color_depth = 8;
	}

	dataInt0_3d = (tv_system->videoInfo.dataInt0>>8)&0xFF;
	switch (dataInt0_3d) {
	case 0x0:
		format->_3d_format = FORMAT_3D_OFF;
		break;
	case 0x11:
		format->_3d_format = FORMAT_RTK_3D_FP;
		break;
	case 0x21:
		format->_3d_format = FORMAT_RTK_3D_SS;
		break;
	case 0x31:
		format->_3d_format = FORMAT_RTK_3D_TB;
		break;
	case 0x01:
		format->_3d_format = FORMAT_3D_FP;
		break;
	case 0x02:
		format->_3d_format = FORMAT_3D_SS;
		break;
	case 0x03:
		format->_3d_format = FORMAT_3D_TB;
		break;
	default:
		format->_3d_format = FORMAT_3D_OFF;
		pr_err("Unknown 3D mode, dataInt0_3d=0x%02x\n", dataInt0_3d);
	}

	format->hdr = tv_system->hdmiInfo.hdr_ctrl_mode;

	if (tv_system->hdmiInfo.dataByte3 & BIT(7)) {
		unsigned char cn;

		cn = (tv_system->hdmiInfo.dataByte5 >> 4) & 0x3;
		switch (cn) {
		case 0:
			content_type = CONTENT_GRAPHICS;
			break;
		case 1:
			content_type = CONTENT_PHOTO;
			break;
		case 2:
			content_type = CONTENT_CINEMA;
			break;
		case 3:
			content_type = CONTENT_GAME;
			break;
		default:
			content_type = CONTENT_NONE;
			pr_err("Unknown content type %u\n", cn);
		}

	} else {
		content_type = CONTENT_NONE;
	}
	format->misc = content_type;

	return ret;
}


int set_hdmitx_format(struct device *dev, struct hdmi_format_setting *format)
{
	int ret = 0;
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM tv_system;
	unsigned int standard;
	unsigned int interlace;
	unsigned int dataint0;
	unsigned char en_3d;
	unsigned int frl_rate;
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);
	struct edid_information *hdmitx_edid_info;

	hdmitx_edid_info = &tx_dev->hdmi_data->hdmitx_edid_info;

	dev_info(dev, "Set format: mode=%u vic=%u freq_shift=%u "
		"color=%u depth=%u 3D=%u HDR=%u misc=0x%x",
		format->mode, format->vic, format->freq_shift,
		format->color, format->color_depth,
		format->_3d_format, format->hdr, format->misc);

	memset(&tv_system, 0, sizeof(tv_system));

	/* HDMI OFF */
	if ((format->vic == 0) || (format->mode == FORMAT_MODE_OFF)) {
		/* cvbs format */
		if (format->vic != 0) {
			standard =  vic_to_vo_standard(format->vic, format->freq_shift,
				format->color, format->color_depth, 0);
			if (standard == VO_STANDARD_ERROR) {
				ret = -EFAULT;
				goto exit;
			}
				
			interlace = is_interlace_mode(format->vic);
		} else {
			standard = VO_STANDARD_NTSC_J;
			interlace = 0;
		}

		tv_system.videoInfo.standard = standard;
		tv_system.videoInfo.enProg = interlace?0:1;
		tv_system.videoInfo.pedType = 0x1;
		tv_system.videoInfo.dataInt0 = 0x4;
		tv_system.hdmiInfo.hdmiMode = VO_HDMI_OFF;

		if (hdmi_clk_always_on) {
			dev_info(dev, "Keep clock always on");
			tv_system.hdmiInfo.hdmi_off_mode = VO_HDMI_OFF_CLOCK_ON;
		}
		goto config;
	}

	if (format->_3d_format != FORMAT_3D_OFF)
		en_3d = 1;
	else
		en_3d = 0;

	standard =  vic_to_vo_standard(format->vic, format->freq_shift,
		format->color, format->color_depth, en_3d);

	if (standard == VO_STANDARD_ERROR) {
		ret = -EFAULT;
		goto exit;
	}

	if ((format->mode == FORMAT_MODE_FRL) &&
		(hdmitx_edid_info->max_frl_rate != 0)) {
		/* Clear AVmute before turn off HDMI clock */
		hdmitx_send_AVmute(dev, 0);

		tv_system.videoInfo.standard = VO_STANDARD_NTSC_J;
		tv_system.videoInfo.enProg = 0x1;
		tv_system.videoInfo.pedType = 0x1;
		tv_system.videoInfo.dataInt0 = 0x4;
		tv_system.hdmiInfo.hdmiMode = VO_HDMI_OFF;
		tv_system.hdmiInfo.hdmi_off_mode = VO_HDMI_OFF_CLOCK_OFF;
		config_interface_type(dev, &tv_system);
		RPC_TOAGENT_HDMI_Config_TV_System(hdmitx_edid_info, &tv_system);
		memset(&tv_system, 0, sizeof(tv_system));
	}

	tv_system.videoInfo.standard = standard;

	if (format->mode == FORMAT_MODE_DVI)
		tv_system.hdmiInfo.hdmiMode = VO_DVI_ON;
	else
		tv_system.hdmiInfo.hdmiMode = VO_HDMI_ON;

	tv_system.videoInfo.enDIF = 0x1;
	tv_system.videoInfo.enCompRGB = 0;
	tv_system.videoInfo.pedType = 0x1;
	tv_system.hdmiInfo.audioSampleFreq = 0x3;
	tv_system.hdmiInfo.audioChannelCount = 0x1;

	interlace = is_interlace_mode(format->vic);
	if (interlace != 0)
		tv_system.videoInfo.enProg = 0;
	else
		tv_system.videoInfo.enProg = 1;

	config_avi_infoframe(hdmitx_edid_info, format, &tv_system);

	/* mask out  color_depth */
	dataint0 = tv_system.hdmiInfo.dataInt0&(~0x3E);

	switch (format->color_depth) {
	case 8:
		dataint0 |= 0x10;
		break;
	case 10:
		dataint0 |= 0x16;
		break;
	case 12:
		dataint0 |= 0x1A;
		break;
	default:
		pr_err("Unknown color_depth=%u", format->color_depth);
		break;
	}

	tv_system.hdmiInfo.dataInt0 = dataint0;

	/* 3D format, CVBS format */
	config_video_dataint0(format->_3d_format, format->vic, &tv_system);

	/* HDR mode */
	tv_system.hdmiInfo.hdr_ctrl_mode = format->hdr;

	/* FRL Link Training */
	frl_rate = 0;
	if ((format->mode == FORMAT_MODE_FRL) &&
		(hdmitx_edid_info->max_frl_rate != 0)) {

		frl_rate = frl_link_training(dev,
					hdmitx_edid_info->max_frl_rate, format);

		switch (frl_rate) {
			case 1:
				tv_system.hdmiInfo.hdmi2px_feature |= HDMI2PX_FRL_3G3L;
				break;
			case 2:
				tv_system.hdmiInfo.hdmi2px_feature |= HDMI2PX_FRL_6G3L;
				break;
			case 3:
				tv_system.hdmiInfo.hdmi2px_feature |= HDMI2PX_FRL_6G4L;
				break;
			default:
				break;
		}
	}

	if ((hdmitx_edid_info->hdmi_id == HDMI_2P0_IDENTIFIER) ||
		(hdmitx_edid_info->hdmi_id == HDMI_2P1_IDENTIFIER))
		tv_system.hdmiInfo.hdmi2px_feature |= HDMI2PX_2P0;

config:
	/* For DP and DSI */
	config_interface_type(dev, &tv_system);

	/* Send RPC */
	RPC_TOAGENT_HDMI_Config_TV_System(hdmitx_edid_info, &tv_system);
exit:
	return ret;
}


int get_hdmitx_format(struct hdmi_format_setting *format)
{
	int ret;

	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM hdmitx_tv_system;

	ret = RPC_ToAgent_QueryConfigTvSystem(&hdmitx_tv_system);

	tv_system_to_hdmi_format(&hdmitx_tv_system, format);

	return ret;
}
/**
 * set_vo_interfaceType - store service dp setting
 * @type: enum VO_INTERFACE_TYPE
 */
void set_vo_interface_type(int type)
{
	vo_interface_type = type;
}

/**
 * ctrl_hdmi_5v - Control HDMI_5V gpio
 * @cmd: enum rtk_ctrl5v_cmd
 *
 * Return: 5V state, 0-OFF 1-ON
 *             Negative value on failure
 */
int ctrl_hdmi_5v(struct device *dev, enum rtk_ctrl5v_cmd cmd)
{
	int ret = -ENODEV;
	int gpio_val;
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);

	if (tx_dev->ctrl_5v_gpio < 0)
		goto exit;

	switch (cmd) {
	case CMD_SET_5V_ON:
	case CMD_SET_5V_OFF:
		/* Active Low */
		gpiod_set_value(tx_dev->ctrl_5v_gpio,
			(cmd == CMD_SET_5V_ON) ? 0:1);
	case CMD_GET_5V_STATE:
		gpio_val = gpiod_get_value(tx_dev->ctrl_5v_gpio);
		ret = (gpio_val == 0) ? 1:0;
		break;
	default:
		goto exit;
	}

exit:
	return ret;
}

ssize_t edid_support_list_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret_count;
	int i;
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);
	asoc_hdmi_t *hdmi_data= tx_dev->hdmi_data;
			
	ret_count = sprintf(buf, "VIC: RGB444  YUV422  YUV444  YUV420  3D FS\n");

	for (i = 0; i <  hdmi_data->support_list.tx_support_size; i++) {
		ret_count += sprintf(buf + ret_count,
			"%03u  %s/%s/%s %s/%s/%s %s/%s/%s %s/%s/%s  %s  %s\n",
			hdmi_data->support_list.tx_support[i].vic,
			hdmi_data->support_list.tx_support[i].rgb444&BIT(1)?"8":"-",
			hdmi_data->support_list.tx_support[i].rgb444&BIT(2)?"10":"--",
			hdmi_data->support_list.tx_support[i].rgb444&BIT(3)?"12":"--",
			hdmi_data->support_list.tx_support[i].yuv422&BIT(1)?"8":"-",
			hdmi_data->support_list.tx_support[i].yuv422&BIT(2)?"10":"--",
			hdmi_data->support_list.tx_support[i].yuv422&BIT(3)?"12":"--",
			hdmi_data->support_list.tx_support[i].yuv444&BIT(1)?"8":"-",
			hdmi_data->support_list.tx_support[i].yuv444&BIT(2)?"10":"--",
			hdmi_data->support_list.tx_support[i].yuv444&BIT(3)?"12":"--",
			hdmi_data->support_list.tx_support[i].yuv420&BIT(1)?"8":"-",
			hdmi_data->support_list.tx_support[i].yuv420&BIT(2)?"10":"--",
			hdmi_data->support_list.tx_support[i].yuv420&BIT(3)?"12":"--",
			hdmi_data->support_list.tx_support[i].support_3d&BIT(0)?"Y":"N",
			hdmi_data->support_list.tx_support[i].support_fs&BIT(0)?"Y":"N");
	}

	return ret_count;
}

/* /sys/devices/platform/9800d000.hdmitx/edid_support_list */
DEVICE_ATTR(edid_support_list, 0444, edid_support_list_show, NULL);

void register_support_list_sysfs(struct device *dev)
{
	device_create_file(dev, &dev_attr_edid_support_list);
}
