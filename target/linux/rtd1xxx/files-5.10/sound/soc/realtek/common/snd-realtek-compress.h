/* SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2017-2020 Realtek Semiconductor Corp.
 */

#ifndef SND_COMPRESS_RTK
#define SND_COMPRESS_RTK

#include <sound/compress_params.h>
#include <sound/compress_offload.h>
#include <sound/compress_driver.h>

#include "snd-realtek.h"

enum {
	OBJT_TYPE_NULL = 0,
	OBJT_TYPE_MAIN = 1,
	OBJT_TYPE_LC   = 2,
	OBJT_TYPE_SSR  = 3,
	OBJT_TYPE_SBR  = 5
};

struct AUDIO_PCM_FORMAT {
	int chnum;
	int samplebit;
	int samplerate;
	u_int dynamicRange;
	u_char emphasis;
	u_char mute;
};

enum ENUM_AUDIO_INFO_TYPE {
	ENUM_AUDIO_PCM_FORMAT,                              //0
	ENUM_AUDIO_FRAME_SIZE,
	ENUM_AUDIO_FRAME_BOUNDARY,
	ENUM_AUDIO_DEC_EXT_BS,
	ENUM_AUDIO_DEC_CFG,
	ENUM_AUDIO_CS_DATA,
	ENUM_AUDIO_AI_STATE,
	ENUM_AUDIO_SPDIF_BS_INFO,
	ENUM_AUDIO_DEMUX_BUFFER_INIT,
	ENUM_AUDIO_CHANNEL_INDEX,                           //9
	ENUM_AUDIO_ECHO_INFO,
	// add by ny /////////////////////////////////
	ENUM_AUDIO_ENC_CFG,
	// add end ///////////////////////////////////
	ENUM_AUDIO_REFRESH_RBUFPTR,
	ENUM_AUDIO_SWITCH_FOCUS,                            //d
	ENUM_AIN_SWITCH_FOCUS,
	ENUM_AUDIO_CHANNEL_SOURCE,      //obsolete
	ENUM_AUDIO_SPDIF_NONPCM_CFG,
	ENUM_AOUT_MUTE_INFO,
	ENUM_AOUT_VOLUME_LEVEL,
	ENUM_AOUT_SPDIF_SOURCE_SELECT,
	ENUM_AOUT_KARAOKE_CONTROL,
	ENUM_AOUT_SPDIF_DIFF,
	ENUM_AUDIO_CHANGE_PP_OUT_SAMPLERATE,
	ENUM_APP_NIGHT_MODE_CFG,  // old version
	ENUM_APP_REINIT_PIN,
	ENUM_AUDIO_BS_ERR,
	ENUM_AUDIO_PTS_TEST,
	ENUM_AOUT_DROP_PTS,
	ENUM_APP_NIGHT_MODE_INFO, // for night mode global var
	ENUM_AUDIO_MPEG_TYPE,
	ENUM_AUDIO_NOISE_INFO,
	ENUM_AUDIO_SOUND_EVENT,
	ENUM_AUDIO_PRIVATE_CGMS_INFO,                       //21
	// add by taro for inband /////////////////////////////////
	ENUM_AUDIO_INBAND_EOS_INFO,
	ENUM_AUDIO_INBAND_PTS_INFO,
	//    ENUM_AUDIO_INBAND_PRIVATE_INFO,
	ENUM_AUDIO_INBAND_NEWFORMAT_INFO,
	// end add by taro for inband /////////////////////////////////
	ENUM_AUDIO_DEC_GET_FORMAT,
	// for trans-encoder, add by isometry
	ENUM_AUDIO_TRANSENC_BS_INFO,
	ENUM_AUDIO_TRANSENC_BUFFER_INIT,
	ENUM_AUDIO_TRANSENC_NONPCM_CFG,
	ENUM_AUDIO_SRC_CHANGE,
	// end for trans-encoder
	// send aac raw out info
	ENUM_AUDIO_AAC_RAW_OUT_INFO,
	ENUM_AUDIO_FLASH_AO,
	ENUM_AUDIO_IN_DATA_MEASURE,
	ENUM_AUDIO_DECODER_NEWFORMAT,
	ENUM_AUDIO_PASSTHROUGH_MODE,
	ENUM_AUDIO_DTS_DOWNMIX,
	ENUM_AUDIO_AAC_DOWNMIX,
	ENUM_AUDIO_KARAOKE_INFO, // for Karaoke mixer, add by Chris
#ifdef AUDIO_RAW_OUT_MORE_BYTES
	ENUM_AUDIO_DECODER_EOS,      // for raw out, decoder inform AO to raw out more 64 bytes.
#endif
	ENUM_AUDIO_GET_FLASH_PINID,
	ENUM_AUDIO_RELEASE_FLASH_PINID,
	ENUM_AUDIO_CONTROL_FLASH_VOLUME,
	ENUM_AUDIO_DEC_SUPPORT_RATE,
#ifdef AUDIO_TV_PLATFORM
	ENUM_AUDIO_SPDIF_OUT_CS_INFO,
	ENUM_AUDIO_DEC_DELAY_RP,
	ENUM_AUDIO_ANALOG_INPUT_CLOCK_INFO,
	ENUM_AIN_ATV_SET_CLOCK,
	ENUM_AIN_INOUT_CH_SELECT,
	ENUM_AIN_SET_FS_CLOCKSRC,
	ENUM_AIN_SET_PATH_DATA_OUT,
	ENUM_AIN_SET_INTERLEAVE_OUT,
#ifdef SUPPORT_GLOBAL_AI
	ENUM_AIN_SET_OUT_FMT,
#endif // SUPPORT_GLOBAL_AI
#endif // AUDIO_TV_PLATFORM
	ENUM_APP_AGC_MODE_CONFIG,
	ENUM_AO_LOW_DELAY_PARAMETERS,
	ENUM_AIN_CONNECT_ALSA
};

struct AUDIO_INFO_PCM_FORMAT {
	enum ENUM_AUDIO_INFO_TYPE infoType;
	struct AUDIO_PCM_FORMAT pcmFormat;
	unsigned int            start_addr;
	int                     max_bit_rate;
};

struct AUDIO_INFO_CHANNEL_INDEX_OLD {
	enum ENUM_AUDIO_INFO_TYPE    infoType;
	char channel_index[8];
	unsigned int start_addr;
	char dual_mono_flag;
};

struct AUDIO_INFO_CHANNEL_INDEX_NEW {
	enum ENUM_AUDIO_INFO_TYPE infoType;
	unsigned int channel_index[8];
	unsigned int start_addr;
	unsigned int dual_mono_flag;
};

// for use dwnstrmInBandQ to deliver info
enum ENUM_AUDIO_INBAND_PRIVATE_INFO {
	AUDIO_INBAND_CMD_PRIVATE_INVALID    = 0,
	AUDIO_INBAND_CMD_PRIVATE_PCM_FMT    = 1,
	AUDIO_INBAND_CMD_PRIVATE_CH_IDX     = 2
};

//following this packet, has private_size number of bytes is the privateinfo to deliver out
struct AUDIO_INBAND_PRIVATE_INFO {
	struct AUDIO_INBAND_CMD_PKT_HEADER header;
	enum ENUM_AUDIO_INBAND_PRIVATE_INFO infoType;
	unsigned int infoSize;
};

struct ALSA_RAW_LATENCY {
	unsigned int latency;
	unsigned int ptsL;
	unsigned int ptsH;
	unsigned int sum; // latency+PTSL
	unsigned int decin_wp;
	unsigned int sync;
	unsigned int decfrm_smpl;
	int rvd[8];
};

#define INBAND_PTS_INFO_SIZE sizeof(struct AUDIO_DEC_PTS_INFO)
#define INBAND_PCM_SIZE      sizeof(struct AUDIO_INFO_PCM_FORMAT)
#define INBAND_EOS_SIZE      sizeof(struct AUDIO_DEC_EOS)
#define INBAND_INDEX_SIZE    sizeof(struct AUDIO_INFO_CHANNEL_INDEX_NEW)
#define INBAND_INFO_SIZE     sizeof(struct AUDIO_INBAND_PRIVATE_INFO)
#define AUDIO_START_THRES    (INBAND_PCM_SIZE + INBAND_INDEX_SIZE + (2 * INBAND_INFO_SIZE))

#define NEW_CHANNEL_INDEX_INFO_SIZE  sizeof(struct AUDIO_INFO_CHANNEL_INDEX_NEW)
#define OLD_CHANNEL_INDEX_INFO_SIZE  sizeof(struct AUDIO_INFO_CHANNEL_INDEX_OLD)

int snd_card_create_compress_instance(struct RTK_snd_card *pSnd, int instance_idx);
int snd_monitor_raw_data_queue(void);

#endif
