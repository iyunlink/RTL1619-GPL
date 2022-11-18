/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef _RTK_RPC_MEM_UAPI_H
#define _RTK_RPC_MEM_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

struct rpc_mem_fd_data {
	unsigned long phyAddr;
	unsigned long ret_offset;
	unsigned long ret_size;
	int ret_fd;
};

#define RPC_MEM_IOC_MAGIC		'R'
#define RPC_MEM_IOC_EXPORT		_IOWR(RPC_MEM_IOC_MAGIC, 0, struct rpc_mem_fd_data)

#endif /* _RTK_RPC_MEM_UAPI_H */
