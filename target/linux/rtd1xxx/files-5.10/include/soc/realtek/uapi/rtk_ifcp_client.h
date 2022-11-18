/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef _UAPI_ICFP_MBOX_H
#define _UAPI_ICFP_MBOX_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define MBOX_ID                 0xCF
#define MBOX_GET_CW             _IOWR(MBOX_ID, 0x0, CMD_MBOX_GET_CW)
#define MBOX_SET_BIV            _IOWR(MBOX_ID, 0x1, CMD_MBOX_SET_BIV)
#define MBOX_GET_BIV            _IOWR(MBOX_ID, 0x2, CMD_MBOX_GET_BIV)
#define MBOX_RESET_IFCP         _IO(MBOX_ID, 0x3)
#define MBOX_WRITE_OTP          _IOWR(MBOX_ID, 0x4, CMD_MBOX_WRITE_OTP)
#define MBOX_READ_OTP           _IOWR(MBOX_ID, 0x5, CMD_MBOX_READ_OTP)
#define MBOX_RESET_OTP          _IO (MBOX_ID, 0x6)
#define MBOX_DESCRAMBLE_PACKET  _IOWR(MBOX_ID, 0x7, CMD_MBOX_DESCRAMBLE)

#define MBOX_RESET_RFIFO        _IO(MBOX_ID, 0x11)


typedef struct _CMD_MBOX_GET_CW
{
	uint32_t slot_id;
	uint32_t key_size;
	uint8_t key[16];
} CMD_MBOX_GET_CW;

typedef struct _CMD_MBOX_BIV
{
	uint32_t data[4];
	uint32_t size;
} CMD_MBOX_SET_BIV, CMD_MBOX_GET_BIV;

typedef struct _CMD_MBOX_OTP
{
	uint32_t addr;
	uint8_t value;
} CMD_MBOX_WRITE_OTP, CMD_MBOX_READ_OTP;

typedef struct _CMD_MBOX_DESCRAMBLE
{
	uint8_t scrambled_packet[188];
	uint8_t clear_packet[188];
	uint32_t packet_size;
} CMD_MBOX_DESCRAMBLE;

#define MBOX_SUCCESS            0
#define MBOX_ERROR              (-100)
#define MBOX_READ_SIZE_ILLEGAL  (-101)
#define MBOX_WRITE_SIZE_ILLEGAL (-102)
#define MBOX_WRITE_OTP_ERROR    (-103)
#define MBOX_READ_OTP_ERROR     (-104)
#define MBOX_RESET_OTP_ERROR    (-105)
#define MBOX_SET_BIV_ERROR      (-106)
#define MBOX_GET_BIV_ERROR      (-107)
#define MBOX_RESET_IFCP_ERROR   (-108)

#endif /* _UAPI_ICFP_MBOX_H */
