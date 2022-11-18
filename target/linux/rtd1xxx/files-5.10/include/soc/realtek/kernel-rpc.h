// SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause)
/*
 * kernel-rpc.h
 *
 * Copyright (c) 2017-2020 Realtek Semiconductor Corp.
 */

#ifndef _KERNEL_RPC_H_
#define _KERNEL_RPC_H_

#define RPC_AUDIO	0x0
#define RPC_VIDEO	0x1
#define RPC_VIDEO2	0x2
#define RPC_HIFI	0x3

#define RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID
/*
 *struct rpc_struct_tp
 *@uint32_t programID: program ID defined in IDL file
 *@uint32_t versionID: version ID defined in IDL file
 *@uint32_t procedureID: function ID defined in IDL file
 *@uint32_t taskID: the caller's task ID, assign 0 if NONBLOCK_MODE
 *@uint32_t sysTID: N/A
 *@uint32_t sysPID: the callee's task ID
 *@uint32_t parameterSize: packet's body size
 *@uint32_t mycontext: return address of reply value
 */
struct rpc_struct_tp {
	uint32_t programID;
	uint32_t versionID;
	uint32_t procedureID;
	uint32_t taskID;
#ifdef RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID
	uint32_t sysTID;
#endif  /* RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID */
	uint32_t sysPID;
	uint32_t parameterSize;
	uint32_t mycontext;
};

int send_rpc_command(int opt, uint32_t command, uint32_t param1, uint32_t param2, uint32_t *retvalue);

static inline uint32_t get_rpc_alignment_offset(uint32_t offset)
{
	if ((offset % 4) == 0)
		return offset;
	else
		return (offset + (4 - (offset % 4)));
}

extern void __iomem *rpc_ringbuf_base;
extern void __iomem *rpc_common_base;
extern void __iomem *rpc_int_base;
extern void __iomem *rpc_acpu_int_flag;
extern void __iomem *rpc_vcpu_int_flag;

#endif
