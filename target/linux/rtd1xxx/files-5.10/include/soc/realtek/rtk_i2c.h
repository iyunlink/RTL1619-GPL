// SPDX-License-Identifier: GPL-2.0+
/*
 * Realtek I2C host driver
 *
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 */

#ifndef _SOC_REALTEK_RTK_I2C_H
#define _SOC_REALTEK_RTK_I2C_H

#define I2C_M_NO_GUARD_TIME		0x0020 /* disable guard time*/
#define I2C_GPIO_RW			0x0080
#define I2C_M_NORMAL_SPEED		0x0000 /* Standard Speed Transmission: 100Kbps */
#define I2C_M_FAST_SPEED		0x0002 /* Fast Speed Transmission: 400Kbps */
#define I2C_M_HIGH_SPEED		0x0004 /* High Speed Transmission: > 400Kbps to max 3.4 Mbps */
#define I2C_M_LOW_SPEED			0x0006 /* Low Speed Transmission: 50Kbps */
#define I2C_M_LOW_SPEED_80		0x0008 /* Low Speed Transmission: 80Kbps */
#define I2C_M_LOW_SPEED_66		0x000a /* Low Speed Transmission: 66Kbps */
#define I2C_M_LOW_SPEED_33		0x000c /* Low Speed Transmission: 33Kbps */
#define I2C_M_LOW_SPEED_10		0x000e /* Low Speed Transmission: 10Kbps */
#define I2C_M_SPEED_MASK		0x000e /* speed control*/

#endif /* _SOC_REALTEK_RTK_I2C_H */
