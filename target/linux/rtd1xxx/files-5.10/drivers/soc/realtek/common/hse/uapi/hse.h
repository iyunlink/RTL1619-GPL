/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef __UAPI_HSE_H
#define __UAPI_HSE_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

struct hse_cmd {
	uint32_t size;
	uint32_t cmds[32];
};

#define HSE_IOCTL_VERSION                _IOR('H', 0x00, unsigned int)
#define HSE_IOCTL_ADD_CMD                _IOW('H', 0x01, struct hse_cmd)
#define HSE_IOCTL_START                  _IO('H', 0x02)
#define HSE_IOCTL_HW_RESET               _IO('H', 0x04)
#define HSE_IOCTL_HW_RESET_FORCE         _IO('H', 0x05)


#define HSE_ROTATE_MODE_90              0
#define HSE_ROTATE_MODE_180             1
#define HSE_ROTATE_MODE_270             2

struct hse_cmd_copy {
	uint32_t size;
	int      dst_fd;
	uint32_t dst_offset;
	int      src_fd;
	uint32_t src_offset;
};

#define HSE_XOR_NUM   5

struct hse_cmd_xor {
	uint32_t size;
	int      dst_fd;
	uint32_t dst_offset;
	int      src_fd[HSE_XOR_NUM];
	uint32_t src_offset[HSE_XOR_NUM];
	uint32_t src_num;
};

struct hse_cmd_constant_fill {
	uint32_t size;
	int      dst_fd;
	uint32_t dst_offset;
	uint32_t val;
};

#define HSE_IOCTL_CMD_COPY             _IOW('H', 0x10, struct hse_cmd_copy)
#define HSE_IOCTL_CMD_XOR              _IOW('H', 0x11, struct hse_cmd_xor)
#define HSE_IOCTL_CMD_CONSTANT_FILL    _IOW('H', 0x12, struct hse_cmd_constant_fill)

#endif /* __UAPI_HSE_H */
