/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#undef TRACE_SYSTEM
#define TRACE_SYSTEM rtk_rpc

#if !defined(_TRACE_RTK_RPC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_RTK_RPC_H

#include <linux/tracepoint.h>
#include <soc/realtek/kernel-rpc.h>

#define TP_STORE_RPC(__entry, big_endian)				\
	do {							\
		if (big_endian) {			\
			__entry->programID	= ntohl(rpc_arg->programID);			\
			__entry->versionID	= ntohl(rpc_arg->versionID);			\
			__entry->taskID	= ntohl(taskid);			\
			__entry->parameterSize	= ntohl(rpc_arg->parameterSize);			\
		} else {					\
			__entry->programID	= rpc_arg->programID;			\
			__entry->versionID	= rpc_arg->versionID;			\
			__entry->taskID	= taskid;			\
			__entry->parameterSize	= rpc_arg->parameterSize;			\
		}						\
	} while (0)

DECLARE_EVENT_CLASS(rtk_rpc_struct_class,

	TP_PROTO(struct rpc_struct_tp *rpc_arg, u32 refclk,
			loff_t num, u32 taskid, bool big_endian),

	TP_ARGS(rpc_arg, refclk, num, taskid, big_endian),

	TP_STRUCT__entry(
		__field(u32,	programID)
		__field(u32,	versionID)
		__field(u32,	procedureID)
		__field(u32,	taskID)
		__field(u32,	parameterSize)
		__field(u32,	mycontext)
		__field(u32,	refclk)
		__field(loff_t,	num)
	),

	TP_fast_assign(
		TP_STORE_RPC(__entry, big_endian);
		__entry->procedureID	= rpc_arg->procedureID;
		__entry->mycontext		= rpc_arg->mycontext;
		__entry->refclk			= refclk;
		__entry->num			= num;
	),

	TP_printk("program=%u version=%u procedure=%u task=%u size=%u context=0x%x 90k=%u rpc_num=%lld",
		__entry->programID, __entry->versionID,
		__entry->procedureID, __entry->taskID,
		__entry->parameterSize, __entry->mycontext,
		__entry->refclk, __entry->num)
);

/**
 * rtk_rpc_peek_rpc_request - called immediately after peeking rpc struct
 */
DEFINE_EVENT(rtk_rpc_struct_class, rtk_rpc_peek_rpc_request,

	TP_PROTO(struct rpc_struct_tp *rpc_arg, u32 refclk,
			loff_t num, u32 taskid, bool big_endian),

	TP_ARGS(rpc_arg, refclk, num, taskid, big_endian)
);

/**
 * rtk_rpc_peek_rpc_reply - called immediately after peeking rpc struct
 */
DEFINE_EVENT(rtk_rpc_struct_class, rtk_rpc_peek_rpc_reply,

	TP_PROTO(struct rpc_struct_tp *rpc_arg, u32 refclk,
			loff_t num, u32 taskid, bool big_endian),

	TP_ARGS(rpc_arg, refclk, num, taskid, big_endian)
);
#endif /*  _TRACE_RTK_RPC_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

