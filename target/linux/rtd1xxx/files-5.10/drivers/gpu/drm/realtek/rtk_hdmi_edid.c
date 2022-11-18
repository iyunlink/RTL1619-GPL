// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 RealTek Inc.
 */
#include "rtk_hdmi.h"

/* Tag Code */
#define AUDIO_BLOCK	0x01
#define VIDEO_BLOCK     0x02
#define VENDOR_BLOCK    0x03
#define SPEAKER_BLOCK	0x04
#define VESA_DISPLAY_TRANSFER_BLOCK	0x05
#define USE_EXTENDED_TAG	0x07

/* Extended Tag Codes */
#define VIDEO_CAPABILITY_DATA_BLOCK			0x00
#define VENDOR_SPECIFIC_VIDEO_DATA_BLOCK	0x01
#define VESA_DISPLAY_DEVICE_DATA_BLOCK		0x02
#define VESA_VIDEO_TIMING_BLOCK_EXTENSION	0x03
#define COLORIMETRY_DATA_BLOCK				0x05
#define HDR_STATIC_METADATA_DATA_BLOCK		0x06
#define HDR_DYNAMIC_METADATA_DATA_BLOCK		0x07
#define VIDEO_FORMAT_PERFERENCE_DATA_BLOCK	0x0D
#define YCBCR420_VIDEO_DATA_BLOCK			0x0E
#define YCBCR420_CAPABILITY_MAP_DATA_BLOCK	0x0F
#define VENDOR_SPECIFIC_AUDIO_DATA_BLOCK	0x11
#define ROOM_CONFIGURATION_DATA_BLOCK		0x13
#define SPEAKER_LOCATION_DATA_BLOCK			0x14
#define INFOFRAME_DATA_BLOCK				0x20
#define HF_EXTENSION_OVERRIDE_DATA_BLOCK    0x78
#define HF_SINK_CAPABILITY_DATA_BLOCK       0x79

static int cea_db_payload_len(const u8 *db)
{
	return db[0] & 0x1f;
}

static int cea_db_tag(const u8 *db)
{
	return db[0] >> 5;
}

static int cea_db_offsets(const u8 *cea, int *start, int *end)
{
	/* Data block offset in CEA extension block */
	*start = 4;
	*end = cea[2];
	if (*end == 0)
		*end = 127;
	if (*end < 4 || *end > 127)
		return -ERANGE;
	return 0;
}

#define for_each_cea_db(cea, i, start, end) \
	for ((i) = (start); (i) < (end) && (i) + cea_db_payload_len(&(cea)[(i)]) < (end); \
				(i) += cea_db_payload_len(&(cea)[(i)]) + 1)

/**
 * parse_hdmi_colorimetry_db - parse colorimetry data block
 * @hdmi: struct rtk_hdmi
 * @db: colorimetry data block
 *
 * bit         7                   6                    5                4                   3                2             1             0
 *     BT2020_RGB/BT2020_YCC/BT2020_cYCC/AdobeRGB/AdobeYCC601/sYCC601/xvYCC709/xvYCC601
 */
static void parse_hdmi_colorimetry_db(struct rtk_hdmi *hdmi, const u8 *db)
{
	hdmi->edid_info.colorimetry = db[2];
}

/**
 * parse_hdmi_VideoCapability_db - parse video capability data block
 * @hdmi: struct rtk_hdmi
 * @db: video capability data block
 *
 * bit   7   6      5         4        3         2         1          0
 *      QY/QS/S_PT1/S_PT0/S_IT1/S_IT0/S_CE1/S_CE0
 */
static void
parse_hdmi_video_capability_db(struct rtk_hdmi *hdmi, const u8 *db)
{
	hdmi->edid_info.vcdb = db[2];
}

/**
 * parse_hdmi_scds - parse Sink Capability  Data Structure(SCDS)
 *   The minimun length of the SCDS is 4, and the maximum length is 28
 *   SCDS might be contained in HF-VSDB or HF-SCDB
 * @hdmi: struct rtk_hdmit
 * @db: Data block, should be HF-VSDB or HF-SCDB
 */
static void parse_hdmi_scds(struct rtk_hdmi *hdmi, const u8 *db)
{
	unsigned char len;

	len = cea_db_payload_len(db);

	if (len < 7)
		return;

	hdmi->edid_info.max_tmds_char_rate = db[5]*5;
	hdmi->edid_info.scdc_capable = db[6];
	hdmi->edid_info.dc_420 = db[7]&0x7;
	hdmi->edid_info.max_frl_rate = (db[7]>>4)&0xF;

	if (len >= 8)
		hdmi->edid_info.scds_pb5 = db[8];

	if (len >= 9)
		hdmi->edid_info.vrr_min = db[9]&0x3F;

	if (len >= 10)
		hdmi->edid_info.vrr_max = ((db[9]&0xC0)<<2) | db[10];

	if (len >= 11)
		hdmi->edid_info.scds_pb8 = db[11];

}

static void rtk_parse_hdmi_extdb(struct rtk_hdmi *hdmi, const u8 *db)
{
	int dbl;

	if (cea_db_tag(db) != USE_EXTENDED_TAG)
		return;

	dbl = cea_db_payload_len(db);

	switch (*(db+1)) {
	case VIDEO_CAPABILITY_DATA_BLOCK:
		dev_dbg(hdmi->dev, "VIDEO_CAPABILITY_DATA_BLOCK (%u bytes)", dbl);
		parse_hdmi_video_capability_db(hdmi, db);
		break;
	case VENDOR_SPECIFIC_VIDEO_DATA_BLOCK:
		dev_dbg(hdmi->dev,
			"VENDOR_SPECIFIC_VIDEO_DATA_BLOCK (%u bytes)", dbl);
		break;
	case VESA_DISPLAY_DEVICE_DATA_BLOCK:
		dev_dbg(hdmi->dev, "VESA_DISPLAY_DEVICE_DATA_BLOCK (%u bytes)", dbl);
		break;
	case VESA_VIDEO_TIMING_BLOCK_EXTENSION:
		dev_dbg(hdmi->dev,
			"VESA_VIDEO_TIMING_BLOCK_EXTENSION (%u bytes)", dbl);
		break;
	case COLORIMETRY_DATA_BLOCK:
		dev_dbg(hdmi->dev, "COLORIMETRY_DATA_BLOCK (%u bytes)", dbl);
		parse_hdmi_colorimetry_db(hdmi, db);
		break;
	case HDR_STATIC_METADATA_DATA_BLOCK:
		dev_dbg(hdmi->dev, "HDR_STATIC_METADATA_DATA_BLOCK (%u bytes)", dbl);
		break;
	case YCBCR420_VIDEO_DATA_BLOCK:
		dev_dbg(hdmi->dev, "YCBCR420_VIDEO_DATA_BLOCK (%u bytes)", dbl);
		break;
	case YCBCR420_CAPABILITY_MAP_DATA_BLOCK:
		dev_dbg(hdmi->dev,
			"YCBCR420_CAPABILITY_MAP_DATA_BLOCK (%u bytes)", dbl);
		break;
	case VENDOR_SPECIFIC_AUDIO_DATA_BLOCK:
		dev_dbg(hdmi->dev, "VENDOR_SPECIFIC_AUDIO_DATA_BLOCK (%u bytes)", dbl);
		// TODO: Parse vendor specific audio_db
		break;
	case INFOFRAME_DATA_BLOCK:
		dev_dbg(hdmi->dev, "INFOFRAME_DATA_BLOCK (%u bytes)", dbl);
		break;
	case HF_EXTENSION_OVERRIDE_DATA_BLOCK:
		dev_dbg(hdmi->dev, "HF_EXTENSION_OVERRIDE_DATA_BLOCK (%u bytes)", dbl);
		// TODO: Parse HF-EEODB
		break;
	case HF_SINK_CAPABILITY_DATA_BLOCK:
		dev_dbg(hdmi->dev, "HF_SINK_CAPABILITY_DATA_BLOCK (%u bytes)", dbl);
		parse_hdmi_scds(hdmi, db);
		break;
	default:
		dev_dbg(hdmi->dev, "Unknow Extend Tag(%u) (%u bytes)", *(db+1), dbl);
		break;
	} /* end of switch (*(db+1)) */

}

void rtk_parse_cea_ext(struct rtk_hdmi *hdmi, struct edid *edid)
{
	bool valid;
	int start;
	int end;
	int i;
	int j;
	u8 *block;
	u8 *db;
	u8 dbl;
	unsigned int oui;

	valid = drm_edid_is_valid(edid);
	if (!valid) {
		dev_err(hdmi->dev, "edid is invalid");
		return;
	}

	if (edid->extensions == 0)
		return;

	for (i = 1; i <= edid->extensions; i++) {
		block = (u8 *)edid + i*EDID_LENGTH;

		if (cea_db_offsets(block, &start, &end))
			return;

		for_each_cea_db(block, j, start, end) {
			db = &block[j];
			dbl = cea_db_payload_len(db);

			switch (cea_db_tag(db)) {
			case AUDIO_BLOCK:
				break;
			case VIDEO_BLOCK:
				break;
			case SPEAKER_BLOCK:
				break;
			case VENDOR_BLOCK:
				if (dbl < 7)
					break;

				oui = db[3] << 16 | db[2] << 8 | db[1];
				if (oui == HDMI_FORUM_IEEE_OUI)
					parse_hdmi_scds(hdmi, db);
				break;
			case VESA_DISPLAY_TRANSFER_BLOCK:
				break;
			case USE_EXTENDED_TAG:
				rtk_parse_hdmi_extdb(hdmi, db);
				break;
			default:
				break;
			}
		}
	}

}
