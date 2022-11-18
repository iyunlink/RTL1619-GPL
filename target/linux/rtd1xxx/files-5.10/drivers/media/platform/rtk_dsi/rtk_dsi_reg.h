/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019 RealTek Inc.
 */

#ifndef _RTK_DSI_REG_H
#define _RTK_DSI_REG_H

//-------------------------------------
//	DSI Control Register
//-------------------------------------
#define CTRL_REG	0x000
#define INTE		0x010
#define TC0		0x100
#define TC1		0x104
#define	TC2		0x108
#define	TC3		0x10C
#define	TC4		0x110
#define	TC5		0x114
#define PAT_GEN		0x610

//-------------------------------------
//	MIPI_DPHY_REG
//-------------------------------------
#define CLOCK_GEN	0x800
#define TX_DATA0	0x808
#define TX_DATA1	0x80C
#define TX_DATA2	0x810
#define TX_DATA3	0x814
#define SSC0		0x840
#define SSC1		0x844
#define SSC2		0x848
#define SSC3		0x84C
#define WATCHDOG	0x850
#define MPLL		0xC00
#define DF		0xC0C 

#endif /* _RTK_DSI_REG_H */
