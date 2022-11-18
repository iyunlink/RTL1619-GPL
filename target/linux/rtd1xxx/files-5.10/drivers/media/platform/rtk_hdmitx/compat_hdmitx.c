/*
 * compat_hdmitx.c - RTK hdmitx driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/compat.h>
#include <linux/fs.h>

#include "compat_hdmitx.h"

#if defined(CONFIG_CPU_V7)
/**
 * rtk_compat_hdmitx_ioctl - ioctl function of hdmitx miscdev
 * @file: hdmitx miscdev to be registered
 * @cmd: control command
 * @arg: arguments
 *
 * Return: 0 on success, -E* on failure
 */
long rtk_compat_hdmitx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret_value = -EFAULT;

	if (!file->f_op->unlocked_ioctl) {
		ret_value = -ENOTTY;
		goto exit;
	}

	ret_value = file->f_op->unlocked_ioctl(file, cmd, arg);

exit:
	return ret_value;
}

#else

static int get_edid_block(struct block_edid __user *kp,
			   struct block_edid32 __user *up)
{
	int err;
	unsigned char block_index;
	unsigned char block_size;
	compat_uptr_t tmp;

	err = get_user(block_index, &up->block_index);
	err |= put_user(block_index, &kp->block_index);
	err |= get_user(block_size, &up->block_size);
	err |= put_user(block_size, &kp->block_size);
	err |= get_user(tmp, &up->edid_ptr);
	err |= put_user(compat_ptr(tmp), &kp->edid_ptr);

	return err;
}

static int put_edid_block(struct block_edid __user *kp,
			   struct block_edid32 __user *up)
{
	int err;
	void *edid;

	err = get_user(edid, &kp->edid_ptr);
	err |= put_user(ptr_to_compat(edid), &up->edid_ptr);

	return err;
}

static int get_fake_edid(struct fake_edid __user *kp,
			   struct fake_edid32 __user *up)
{
	int err;
	compat_uptr_t tmp;
	unsigned int size;

	err = get_user(tmp, &up->data_ptr);
	err |= put_user(compat_ptr(tmp), &kp->data_ptr);
	err |= get_user(size, &up->size);
	err |= put_user(size, &kp->size);

	return err;
}

/**
 * rtk_compat_hdmitx_ioctl - ioctl function of hdmitx miscdev
 * @file: hdmitx miscdev to be registered
 * @cmd: control command
 * @arg: arguments
 *
 * Return: 0 on success, -E* on failure
 */
long rtk_compat_hdmitx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	void __user *kup = NULL;
	long err = -EFAULT;

	if (!file->f_op->unlocked_ioctl) {
		err = -ENOTTY;
		goto exit;
	}

	switch (cmd) {
	case HDMI_GET_EDID_BLOCK32:
		kup = compat_alloc_user_space(sizeof(struct block_edid));
		err = get_edid_block(kup, up);
		if (err)
			goto exit;
		err = file->f_op->unlocked_ioctl(file, HDMI_GET_EDID_BLOCK,
					(unsigned long)kup);
		if (err)
			goto exit;
		err = put_edid_block(kup, up);
	break;
	case HDMI_SET_FAKE_EDID32:
		kup = compat_alloc_user_space(sizeof(struct fake_edid));
		err = get_fake_edid(kup, up);
		if (err)
			goto exit;
		err = file->f_op->unlocked_ioctl(file, HDMI_SET_FAKE_EDID,
					(unsigned long)kup);
	break;
	default:
		err = file->f_op->unlocked_ioctl(file, cmd,
					(unsigned long)up);
	}

exit:
	return err;
}
#endif /* CONFIG_CPU_V7 */

