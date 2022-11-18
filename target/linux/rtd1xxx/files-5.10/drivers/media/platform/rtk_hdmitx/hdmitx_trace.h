// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek HDMITX Trace
 *
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM rtk_hdmitx

#if !defined(_HDMITX_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _HDMITX_TRACE_H

#include <linux/tracepoint.h>
#include "hdmitx.h"

#define HDMI_IOCTL_NAME_LIST					\
			hdmi_ioctl_name(GET_SINK_CAPABILITY)	\
			hdmi_ioctl_name(GET_RAW_EDID)		\
			hdmi_ioctl_name(CHECK_LINK_STATUS)	\
			hdmi_ioctl_name(GET_VIDEO_CONFIG)	\
			hdmi_ioctl_name(SEND_AVMUTE)		\
			hdmi_ioctl_name(CONFIG_TV_SYSTEM)	\
			hdmi_ioctl_name(CONFIG_AVI_INFO)	\
			hdmi_ioctl_name(SET_FREQUNCY)		\
			hdmi_ioctl_name(SEND_AUDIO_MUTE)	\
			hdmi_ioctl_name(SEND_AUDIO_VSDB_DATA)	\
			hdmi_ioctl_name(SEND_AUDIO_EDID2)	\
			hdmi_ioctl_name(CHECK_Rx_Sense)		\
			hdmi_ioctl_name(GET_EXT_BLK_COUNT)	\
			hdmi_ioctl_name(GET_EXTENDED_EDID)	\
			hdmi_ioctl_name(QUERY_DISPLAY_STANDARD)	\
			hdmi_ioctl_name(SEND_VOUT_EDID_DATA)	\
			hdmi_ioctl_name(GET_EDID_SUPPORT_LIST)	\
			hdmi_ioctl_name(SET_OUTPUT_FORMAT)	\
			hdmi_ioctl_name(GET_OUTPUT_FORMAT)	\
			hdmi_ioctl_name(SET_VO_INTERFACE_TYPE)	\
			hdmi_ioctl_name(GET_CONFIG_TV_SYSTEM)	\
			hdmi_ioctl_name(HOTPLUG_DETECTION)	\
			hdmi_ioctl_name(WAIT_HOTPLUG)		\
			hdmi_ioctl_name(GET_EDID_BLOCK)		\
			hdmi_ioctl_name_end(SET_VRR)

#undef hdmi_ioctl_name
#undef hdmi_ioctl_name_end

#define hdmi_ioctl_name(cmd) TRACE_DEFINE_ENUM(HDMI_##cmd);
#define hdmi_ioctl_name_end(cmd)  TRACE_DEFINE_ENUM(HDMI_##cmd);

HDMI_IOCTL_NAME_LIST

#undef hdmi_ioctl_name
#undef hdmi_ioctl_name_end

#define hdmi_ioctl_name(cmd) { HDMI_##cmd, #cmd },
#define hdmi_ioctl_name_end(cmd) { HDMI_##cmd, #cmd }

#define show_hdmi_ioctl_name(val)				\
	__print_symbolic(val, HDMI_IOCTL_NAME_LIST)

TRACE_EVENT(hdmitx_ioctl,

	TP_PROTO(int cmd),

	TP_ARGS(cmd),

	TP_STRUCT__entry(
		__field(	int,	cmd	)
	),

	TP_fast_assign(
		__entry->cmd = cmd;
	),

	TP_printk("[HDMI_%s]", show_hdmi_ioctl_name(__entry->cmd))
);

DECLARE_EVENT_CLASS(hdmitx_ioctl_simple_state,

	TP_PROTO(unsigned state),

	TP_ARGS(state),

	TP_STRUCT__entry(
		__field(	unsigned,	state	)
	),

	TP_fast_assign(
		__entry->state = state;
	),

	TP_printk("--> 0x%x",  __entry->state)
);

DEFINE_EVENT(hdmitx_ioctl_simple_state, hdmitx_send_AVmute,

	TP_PROTO(unsigned flag),

	TP_ARGS(flag)
);

DEFINE_EVENT(hdmitx_ioctl_simple_state, hdmitx_set_frequency,

	TP_PROTO(unsigned freq),

	TP_ARGS(freq)
);

DEFINE_EVENT(hdmitx_ioctl_simple_state, hdmitx_get_extension_blk_count,

	TP_PROTO(unsigned count),

	TP_ARGS(count)
);

#define T_DEBOUNCE_MS  10000
DECLARE_EVENT_CLASS(hdmitx_check_state,

	TP_PROTO(unsigned state, unsigned count),

	TP_ARGS(state, count),

	TP_STRUCT__entry(
		__field(	unsigned,	state	)
		__field(	unsigned,	count	)
	),

	TP_fast_assign(
		__entry->state = state;
		__entry->count = count;
	),

	TP_printk("--> state=%u, was called %u times",
		__entry->state, __entry->count)
);

DEFINE_EVENT(hdmitx_check_state, hdmitx_check_rx_sense,

	TP_PROTO(unsigned state, unsigned count),

	TP_ARGS(state, count)
);

DEFINE_EVENT(hdmitx_check_state, hdmitx_get_link_status,

	TP_PROTO(unsigned status, unsigned count),

	TP_ARGS(status, count)
);

TRACE_EVENT(hdmitx_get_video_config,

	TP_PROTO(u64 vic),

	TP_ARGS(vic),

	TP_STRUCT__entry(
		__field(	u64,	vic	)
	),

	TP_fast_assign(
		__entry->vic = vic;
	),

	TP_printk("--> 0x%llx",  (unsigned long long)__entry->vic)
);

#define HDMI_VO_INTERFACE_TYPE_LIST						\
		hdmi_vo_interface_name(ANALOG_AND_DIGITAL)			\
		hdmi_vo_interface_name(ANALOG_ONLY)				\
		hdmi_vo_interface_name(DIGITAL_ONLY)				\
		hdmi_vo_interface_name(DISPLAY_PORT_ONLY)			\
		hdmi_vo_interface_name(HDMI_AND_DISPLAY_PORT_SAME_SOURCE)	\
		hdmi_vo_interface_name(HDMI_AND_DISPLAY_PORT_DIFF_SOURCE)	\
		hdmi_vo_interface_name(DISPLAY_PORT_AND_CVBS_SAME_SOURCE)	\
		hdmi_vo_interface_name(HDMI_AND_DP_DIFF_SOURCE_WITH_CVBS)	\
		hdmi_vo_interface_name_end(FORCE_DP_OFF)

#undef hdmi_vo_interface_name
#undef hdmi_vo_interface_name_end

#define hdmi_vo_interface_name(type) TRACE_DEFINE_ENUM(VO_##type);
#define hdmi_vo_interface_name_end(type) TRACE_DEFINE_ENUM(VO_##type);

HDMI_VO_INTERFACE_TYPE_LIST

#undef hdmi_vo_interface_name
#undef hdmi_vo_interface_name_end

#define hdmi_vo_interface_name(type) { VO_##type, #type },
#define hdmi_vo_interface_name_end(type) {VO_##type, #type }

#define show_hdmi_vo_interface_name(type)				\
	__print_symbolic(type, HDMI_VO_INTERFACE_TYPE_LIST)

TRACE_EVENT(hdmitx_set_interface_type,

	TP_PROTO(int type),

	TP_ARGS(type),

	TP_STRUCT__entry(
		__field(	int,	type	)
	),

	TP_fast_assign(
		__entry->type = type;
	),

	TP_printk("--> %s", show_hdmi_vo_interface_name(__entry->type))
);

DECLARE_EVENT_CLASS(hdmitx_output_format,

	TP_PROTO(struct hdmi_format_setting *format),

	TP_ARGS(format),

	TP_STRUCT__entry(
		__field_struct(	struct hdmi_format_setting,	format	)
	),

	TP_fast_assign(
		memcpy(&__entry->format, format, sizeof(__entry->format));
	),

	TP_printk("--> mod:%u vic:%u f_shif:%u col:%u col_dep:%u"
		" 3d:%u hdr:%u", __entry->format.mode,
		__entry->format.vic, __entry->format.freq_shift,
		__entry->format.color, __entry->format.color_depth,
		__entry->format._3d_format,  __entry->format.hdr
	)
);

DEFINE_EVENT(hdmitx_output_format, hdmitx_get_output_format,

	TP_PROTO(struct hdmi_format_setting *format),

	TP_ARGS(format)
);

DEFINE_EVENT(hdmitx_output_format, hdmitx_set_output_format,

	TP_PROTO(struct hdmi_format_setting *format),

	TP_ARGS(format)
);


TRACE_EVENT(hdmitx_get_edid_support_list,

	TP_PROTO(struct hdmi_format_support *info, int idx),

	TP_ARGS(info, idx),

	TP_STRUCT__entry(
		__field_struct(	struct hdmi_format_support,	info	)
		__field(	int,				idx	)
	),

	TP_fast_assign(
		memcpy(&__entry->info, info, sizeof(__entry->info));
		__entry->idx = idx;
	),

	TP_printk("--> [%d] vic:%u rgb:0x%x 422:0x%x 444:0x%x 420:0x%x"
		" 3d:0x%x fs:0x%x", __entry->idx,  __entry->info.vic,
		__entry->info.rgb444, __entry->info.yuv422,
		__entry->info.yuv444, __entry->info.yuv420,
		__entry->info.support_3d, __entry->info.support_fs
	)
);

TRACE_EVENT(hdmitx_hotplug_state,

	TP_PROTO(int state, int old_state, int hpd, int rx_sense),

	TP_ARGS(state, old_state, hpd, rx_sense),

	TP_STRUCT__entry(
		__field(	int,	state		)
		__field(	int,	old_state	)
		__field(	int,	hpd			)
		__field(	int,	rx_sense	)
	),

	TP_fast_assign(
		__entry->state = state;
		__entry->old_state = old_state;
		__entry->hpd = hpd;
		__entry->rx_sense = rx_sense;
	),

	TP_printk("state:%d, HPD:%d, RxSense:%d, orig:%d%s", __entry->state,
		__entry->hpd, __entry->rx_sense, __entry->old_state,
		__entry->state == __entry->old_state ? "": "[TOGGLED]")
);

#endif /* _HDMITX_TRACE_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE hdmitx_trace
#include <trace/define_trace.h>
