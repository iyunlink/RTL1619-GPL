/*
 * rtk_smcc.h - RTK SMCC driver
 *
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/ioctl.h>

#define RTK_SMCC_IOC_MAGIC 'k'
#define RTK_OTP_WRITE 0x8400ff28
#define RTK_OTP_READ 0x8400ff27

enum OTP_WRITE_CASE {
	DATA_SECTION_CASE = 0x0,
	DRITECT_PARAM_CASE = 0x1,
};

typedef struct {
	unsigned int typeID;
	unsigned long long ret_value;
	unsigned long long ret_value_h;
} OTP_Info_T;

typedef struct {
	unsigned int typeID;
	unsigned int perform_case;
	unsigned long long burning_value;
	unsigned long long burning_data[32];
} OTP_Write_Info_T;

typedef struct {
	unsigned long long ret_value;
	unsigned long long reserve1[7];
	unsigned long long ret_value_h;
	unsigned long long reserve2[7];
	unsigned long long burning_data[32];
} OPT_Data_T;

#define RTK_SMCC_OTP_READ	_IOWR(RTK_SMCC_IOC_MAGIC, 1, OTP_Info_T)
#define RTK_SMCC_OTP_WRITE      _IOWR(RTK_SMCC_IOC_MAGIC, 2, OTP_Write_Info_T)
