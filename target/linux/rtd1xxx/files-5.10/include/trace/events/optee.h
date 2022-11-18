/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#undef TRACE_SYSTEM
#define TRACE_SYSTEM optee

#if !defined(_TRACE_OPTEE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_OPTEE_H

#include <linux/tracepoint.h>
#include <linux/tee_drv.h>

/**
 * optee_open_session_entry - called immediately before open_session in tee
 * @session_arg:		pointer to struct tee_ioctl_open_session_arg
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
TRACE_EVENT(optee_open_session_entry,

	TP_PROTO(struct tee_ioctl_open_session_arg *session_arg,
		struct optee_msg_arg *msg_arg),

	TP_ARGS(session_arg, msg_arg),

	TP_STRUCT__entry(
		__field(u32,	timeLow)
		__field(u16,	timeMid)
		__field(u16,	timeHiAndVersion)
		__field(u32,	session)
		__field(u32,	ret)
	),

	TP_fast_assign(
		__entry->timeLow		= (session_arg->uuid[0] << 24) |
						(session_arg->uuid[1] << 16) |
						(session_arg->uuid[2] << 8) |
						session_arg->uuid[3];
		__entry->timeMid		= (session_arg->uuid[4] << 8) |
						session_arg->uuid[5];
		__entry->timeHiAndVersion	= (session_arg->uuid[6] << 8) |
						session_arg->uuid[7];
		__entry->session		= msg_arg->session;
		__entry->ret			= msg_arg->ret;
	),

	TP_printk("timeLow=0x%x timeMid=0x%x timeHiAndVersion=0x%x "
		"session=0x%x", __entry->timeLow, __entry->timeMid,
		__entry->timeHiAndVersion, __entry->session)
);

/**
 * optee_open_session_exit - called immediately after open_session in tee
 * @session_arg:		pointer to struct tee_ioctl_open_session_arg
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
TRACE_EVENT(optee_open_session_exit,

	TP_PROTO(struct tee_ioctl_open_session_arg *session_arg,
		struct optee_msg_arg *msg_arg),

	TP_ARGS(session_arg, msg_arg),

	TP_STRUCT__entry(
		__field(u32,   timeLow)
		__field(u16,   timeMid)
		__field(u16,   timeHiAndVersion)
		__field(u32,   session)
		__field(u32,   ret)
	),

	TP_fast_assign(
		__entry->timeLow		= (session_arg->uuid[0] << 24) |
						(session_arg->uuid[1] << 16) |
						(session_arg->uuid[2] << 8) |
						session_arg->uuid[3];
		__entry->timeMid		= (session_arg->uuid[4] << 8) |
						session_arg->uuid[5];
		__entry->timeHiAndVersion	= (session_arg->uuid[6] << 8) |
						session_arg->uuid[7];
		__entry->session		= msg_arg->session;
		__entry->ret			= msg_arg->ret;
	),

	TP_printk("timeLow=0x%x timeMid=0x%x timeHiAndVersion=0x%x "
		"session=0x%x ret=0x%x", __entry->timeLow, __entry->timeMid,
		__entry->timeHiAndVersion, __entry->session, __entry->ret)
);

DECLARE_EVENT_CLASS(optee_entry_class,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg),

	TP_STRUCT__entry(
		__field(u32,   func)
		__field(u32,   session)
		__field(u32,   cancel_id)
	),

	TP_fast_assign(
		__entry->func			= msg_arg->func;
		__entry->session		= msg_arg->session;
		__entry->cancel_id		= msg_arg->cancel_id;
	),

	TP_printk("func=0x%x session=0x%x cancel_id=0x%x",
		__entry->func, __entry->session, __entry->cancel_id)
);

DECLARE_EVENT_CLASS(optee_exit_class,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg),

	TP_STRUCT__entry(
		__field(u32,   func)
		__field(u32,   session)
		__field(u32,   cancel_id)
		__field(u32,   ret)
	),

	TP_fast_assign(
		__entry->func			= msg_arg->func;
		__entry->session		= msg_arg->session;
		__entry->cancel_id		= msg_arg->cancel_id;
		__entry->ret			= msg_arg->ret;
	),

	TP_printk("func=0x%x session=0x%x cancel_id=0x%x ret=0x%x",
		__entry->func, __entry->session,
		__entry->cancel_id, __entry->ret)
);

/**
 * optee_close_session_entry - called immediately before close_session in tee
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
DEFINE_EVENT(optee_entry_class, optee_close_session_entry,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg)
);

/**
 * optee_close_session_exit - called immediately after close_session in tee
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
DEFINE_EVENT(optee_exit_class, optee_close_session_exit,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg)
);

/**
 * optee_invoke_func_entry - called immediately before invoke_func in tee
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
DEFINE_EVENT(optee_entry_class, optee_invoke_func_entry,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg)
);

/**
 * optee_invoke_func_exit - called immediately after invoke_func in tee
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
DEFINE_EVENT(optee_entry_class, optee_invoke_func_exit,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg)
);

/**
 * optee_cancel_req_entry - called immediately before cancel_req in tee
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
DEFINE_EVENT(optee_entry_class, optee_cancel_req_entry,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg)
);

/**
 * optee_cancel_req_exit - called immediately after cancel_req in tee
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
DEFINE_EVENT(optee_entry_class, optee_cancel_req_exit,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg)
);

/**
 * optee_release_entry - called immediately before release in tee
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
DEFINE_EVENT(optee_entry_class, optee_release_entry,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg)
);

/**
 * optee_release_exit - called immediately after release in tee
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
DEFINE_EVENT(optee_entry_class, optee_release_exit,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg)
);

/**
 * optee_shm_register_entry - called immediately before shm_register in tee
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
TRACE_EVENT(optee_shm_register_entry,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg),

	TP_STRUCT__entry(
		__field(u64,   attr)
		__field(u64,   shm_ref)
		__field(u64,   size)
		__field(u64,   buf_ptr)
	),

	TP_fast_assign(
		__entry->attr		= msg_arg->params->attr;
		__entry->shm_ref	= msg_arg->params->u.tmem.shm_ref;
		__entry->size		= msg_arg->params->u.tmem.size;
		__entry->buf_ptr	= msg_arg->params->u.tmem.buf_ptr;
	),

	TP_printk("attr=0x%llx shm_ref=0x%llx size=%llu buf_ptr=0x%llx",
		__entry->attr, __entry->shm_ref,
		__entry->size, __entry->buf_ptr)
);

/**
 * optee_shm_register_exit - called immediately after shm_register in tee
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
TRACE_EVENT(optee_shm_register_exit,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg),

	TP_STRUCT__entry(
		__field(u64,   attr)
		__field(u64,   shm_ref)
		__field(u64,   size)
		__field(u64,   buf_ptr)
		__field(u32,   ret)
	),

	TP_fast_assign(
		__entry->attr		= msg_arg->params->attr;
		__entry->shm_ref	= msg_arg->params->u.tmem.shm_ref;
		__entry->size		= msg_arg->params->u.tmem.size;
		__entry->buf_ptr	= msg_arg->params->u.tmem.buf_ptr;
		__entry->ret		= msg_arg->ret;
	),

	TP_printk("attr=0x%llx shm_ref=0x%llx size=%llu buf_ptr=0x%llx ret=0x%x",
		__entry->attr, __entry->shm_ref, __entry->size,
		__entry->buf_ptr, __entry->ret)
);

/**
 * optee_shm_unregister_entry - called immediately before shm_unregister in tee
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
TRACE_EVENT(optee_shm_unregister_entry,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg),

	TP_STRUCT__entry(
		__field(u64,   attr)
		__field(u64,   shm_ref)
	),

	TP_fast_assign(
		__entry->attr		= msg_arg->params[0].attr;
		__entry->shm_ref	= msg_arg->params[0].u.rmem.shm_ref;
	),

	TP_printk("attr=0x%llx shm_ref=0x%llx", __entry->attr, __entry->shm_ref)
);

/**
 * optee_shm_unregister_exit - called immediately after shm_unregister in tee
 * @msg_arg:			pointer to struct optee_msg_arg
 *
 * Allows to determine the tee latency roughly.
 */
TRACE_EVENT(optee_shm_unregister_exit,

	TP_PROTO(struct optee_msg_arg *msg_arg),

	TP_ARGS(msg_arg),

	TP_STRUCT__entry(
		__field(u64,   attr)
		__field(u64,   shm_ref)
		__field(u32,   ret)
	),

	TP_fast_assign(
		__entry->attr		= msg_arg->params[0].attr;
		__entry->shm_ref	= msg_arg->params[0].u.rmem.shm_ref;
		__entry->ret		= msg_arg->ret;
	),

	TP_printk("attr=0x%llx shm_ref=0x%llx ret=0x%x",
		__entry->attr, __entry->shm_ref, __entry->ret)
);
#endif /*  _TRACE_OPTEE_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

