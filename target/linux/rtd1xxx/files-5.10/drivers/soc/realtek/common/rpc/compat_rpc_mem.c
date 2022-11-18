/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/module.h>

#include "rpc_mem_uapi.h"
#include "compat_rpc_mem.h"

struct compat_rpc_mem_fd_data {
	compat_ulong_t phyAddr;
	compat_ulong_t ret_offset;
	compat_ulong_t ret_size;
	compat_int_t ret_fd;
};

#define COMPAT_RPC_MEM_IOC_EXPORT		_IOWR(RPC_MEM_IOC_MAGIC, 0, struct compat_rpc_mem_fd_data)

static int compat_get_rpc_mem_fd_data(
	struct compat_rpc_mem_fd_data __user *data32,
	struct rpc_mem_fd_data __user *data)
{
	compat_ulong_t p;
	compat_ulong_t o;
	compat_ulong_t s;
	compat_int_t f;
	int err;

	err = get_user(p, &data32->phyAddr);
	err |= put_user(p, &data->phyAddr);
	err |= get_user(o, &data32->ret_offset);
	err |= put_user(o, &data->ret_offset);
	err |= get_user(s, &data32->ret_size);
	err |= put_user(s, &data->ret_size);
	err |= get_user(f, &data32->ret_fd);
	err |= put_user(f, &data->ret_fd);

	return err;
}

static int compat_put_rpc_mem_fd_data(
	struct compat_rpc_mem_fd_data __user *data32,
	struct rpc_mem_fd_data __user *data)
{
	compat_ulong_t p;
	compat_ulong_t o;
	compat_ulong_t s;
	compat_int_t f;
	int err;

	err = get_user(p, &data->phyAddr);
	err |= put_user(p, &data32->phyAddr);
	err |= get_user(o, &data->ret_offset);
	err |= put_user(o, &data32->ret_offset);
	err |= get_user(s, &data->ret_size);
	err |= put_user(s, &data32->ret_size);
	err |= get_user(f, &data->ret_fd);
	err |= put_user(f, &data32->ret_fd);

	return err;
}

long compat_rpc_mem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret = 0;

	if (!filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_RPC_MEM_IOC_EXPORT:
	{
		struct compat_rpc_mem_fd_data __user *data32;
		struct rpc_mem_fd_data __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_rpc_mem_fd_data(data32, data);
		if (err)
			return err;

		ret = filp->f_op->unlocked_ioctl(filp, RPC_MEM_IOC_EXPORT, (unsigned long)data);
		err = compat_put_rpc_mem_fd_data(data32, data);
		return ret ? ret : err;
	}

	default:
	{
		printk(KERN_ERR "[COMPAT_RPC_MEM] No such IOCTL, cmd is %d\n", cmd);
		return -ENOIOCTLCMD;
	}
	}
}

MODULE_LICENSE("GPL v2");
