/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef _LINUX_COMPAT_RPC_MEM_H
#define _LINUX_COMPAT_RPC_MEM_H

#if IS_ENABLED(CONFIG_COMPAT)

long compat_rpc_mem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#else

#define compat_rpc_mem_ioctl  NULL

#endif /* CONFIG_COMPAT */
#endif /* _LINUX_COMPAT_RPC_MEM_H */
