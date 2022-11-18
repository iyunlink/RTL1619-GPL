/*
 * hank_tp_reg.h
 *
 * Copyright (C) 2020 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _HANK_TP_REG_H_
#define _HANK_TP_REG_H_

#define SOFT_RESET6_SECURE   ((0x98000014))
#define SOFT_RESET7   ((0x98000068))
#define CLOCK_ENABLE2 ((0x98000054))
#define CLOCK_ENABLE4 ((0x9800005C))

#define RESET_TP(x)         ((x) << 26)
#define RESET_TPB(x)        ((x) << 28)
#define RESET_TPC(x)        ((x) << 30)
#define RESET_TSIO(x)       ((x) << 6)
#define ENABLE_TP_CLOCK(x)  ((x) << 6)
#define ENABLE_TPB_CLOCK(x) ((x) << 28)
#define ENABLE_TPC_CLOCK(x) ((x) << 16)
#define ENABLE_TSIO_CLOCK(x)  ((x) << 24)

#define  MAX_PID_COUNT          128
#define  MAX_DDR_BUFFER_COUNT   64
#define  MAX_SEC_COUNT          64

#define  TP_PID_FILTER_COUNT   (MAX_PID_COUNT / 2)
#define  TP_DDR_BUFFER_COUNT   (MAX_DDR_BUFFER_COUNT / 2)
#define  TP_FILTER_COUNT   (MAX_DDR_BUFFER_COUNT / 2)
#define  TP_SEC_FILTER_COUNT   (MAX_SEC_COUNT >> 1)


/* TP CONTROL */
#define TP_TF0_CNTL					0x000
#define TP_TF0_STRM_ID_0			0x208
#define TP_TF0_STRM_ID_1			0x20c
#define TP_TF0_STRM_ID_2			0x210
#define TP_TF0_STRM_ID_3			0x214
#define TP_TF0_STRM_ID_VAL			0x218
#define TP_TF0_CNT					0x004
#define TP_TF0_DRP_CNT				0x008
#define TP_TF0_ERR_CNT				0x00c
#define TP_TF0_FRMCFG				0x824
#define TP_TF0_INT					0x828
#define TP_TF0_INT_EN				0x82c

#define TP_TF1_CNTL					0x010
#define TP_TF1_STRM_ID_0			0x21c
#define TP_TF1_STRM_ID_1			0x220
#define TP_TF1_STRM_ID_2			0x224
#define TP_TF1_STRM_ID_3			0x228
#define TP_TF1_STRM_ID_VAL			0x22c
#define TP_TF1_CNT					0x014
#define TP_TF1_DRP_CNT				0x018
#define TP_TF1_ERR_CNT				0x01C
#define TP_TF1_FRMCFG				0x830
#define TP_TF1_INT					0x834
#define TP_TF1_INT_EN				0x83c

#define TPB_TF0_CNTL					0x000
#define TPB_TF0_STRM_ID_0			0x208
#define TPB_TF0_STRM_ID_1			0x20c
#define TPB_TF0_STRM_ID_2			0x210
#define TPB_TF0_STRM_ID_3			0x214
#define TPB_TF0_STRM_ID_VAL			0x218
#define TPB_TF0_CNT					0x004
#define TPB_TF0_DRP_CNT				0x008
#define TPB_TF0_ERR_CNT				0x00C
#define TPB_TF0_FRMCFG				0x824
#define TPB_TF0_INT				0x828
#define TPB_TF0_INT_EN				0x82C

#define TPB_TF1_CNTL				0x010
#define TPB_TF1_STRM_ID_0			0x21c
#define TPB_TF1_STRM_ID_1			0x220
#define TPB_TF1_STRM_ID_2			0x224
#define TPB_TF1_STRM_ID_3			0x228
#define TPB_TF1_STRM_ID_VAL			0x22c
#define TPB_TF1_CNT				0x014
#define TPB_TF1_DRP_CNT				0x018
#define TPB_TF1_ERR_CNT				0x01C
#define TPB_TF1_FRMCFG				0x830
#define TPB_TF1_INT				0x834
#define TPB_TF1_INT_EN				0x83C

#define TP_PESE_TO0			0x880
#define TP_PESE_TO1			0x884
#define TPB_PESE_TO0		0x880
#define TPB_PESE_TO1		0x884

#define TP_TF_CNTL_AF_TSC_ENABLE_BIT    (0x00000001 << 25)
#define TP_TF_CNTL_PROTECT_MODE_BIT     (0x00000001 << 24)
#define TP_TF_CNTL_DES_NX_MODE_BIT      (0x00000001 << 23)
#define TP_TF_CNTL_DES_RT_MODE_BIT      (0x00000001 << 22)
#define TP_TF_CNTL_TS_INPUT_SEL_H_BIT   (0x00000001 << 21)
#define TP_TF_CNTL_TS_INPUT_SEL_L_BIT   (0x00000001 << 20)
#define TP_TF_CNTL_SEC_BODY_INFO_EN_BIT (0x00000001 << 18)
#define TP_TF_CNTL_ERR_FIX_EN_BIT		(0x00000001 << 17)
#define TP_TF_CNTL_STRM_ID_EN_BIT       (0x00000001 << 16)
#define TP_TF_CNTL_BUF_RDY_CTL_BIT	(0x00000001 << 15)
#define TP_TF_CNTL_PSC_EN_BIT           (0x00000001 << 14)
#define TP_TF_CNTL_PES_EN_BIT           (0x00000001 << 13)
#define TP_TF_CNTL_TSC_EN_BIT		(0x00000001 << 12)
#define TP_TF_CNTL_TB_BIT		(0x00000001 << 11)
#define TP_TF_CNTL_BUSY_BIT		(0x00000001 << 10)
#define TP_TF_CNTL_MODE_BIT		(0x00000001 << 9)
#define TP_TF_CNTL_DU_EN_BIT		(0x00000001 << 8)
#define TP_TF_CNTL_DE_EN_BIT		(0x00000001 << 7)
#define TP_TF_CNTL_XT_EN_BIT		(0x00000001 << 6)
#define TP_TF_CNTL_PID_EN_BIT		(0x00000001 << 5)
#define TP_TF_CNTL_NULL_EN_BIT	(0x00000001 << 4)
#define TP_TF_CNTL_TRERR_EN_BIT	(0x00000001 << 3)
#define TP_TF_CNTL_SYNC_EN_BIT	(0x00000001 << 2)
#define TP_TF_CNTL_RST_EN_BIT		(0x00000001 << 1)
#define TP_TF_CNTL_WRITE_DATA_BIT	(0x00000001)

#define TP_TF_CNTL_AF_TSC_EN(x)        ((x & 0x01) << 25)
#define TP_TF_CNTL_PROTECT_MODE_EN(x)  ((x & 0x01) << 24)
#define TP_TF_CNTL_DES_NX_MODE_EN(x)   ((x & 0x01) << 23)
#define TP_TF_CNTL_DES_RT_MODE_EN(x)   ((x & 0x01) << 22)
#define TP_TF_CNTL_TS_INPUT_SEL_EN(x)  ((x & 0x03) << 20)
#define TP_TF_CNTL_SEC_BODY_INFO_EN(x) ((x & 0x01) << 18)
#define TP_TF_CNTL_ERR_FIX_EN(x)		((x & 0x01) << 17)
#define TP_TF_CNTL_STRM_ID_EN(x)		((x & 0x01) << 16)
#define TP_TF_CNTL_BUF_RDY_CTL(x)           ((x & 0x01) << 15)
#define TP_TF_CNTL_PSC_EN(x)		((x & 0x01) << 14)
#define TP_TF_CNTL_PES_EN(x)		((x & 0x01) << 13)
#define TP_TF_CNTL_TSC_EN(x)			((x & 0x01) << 12)
#define TP_TF_CNTL_TB(x)			((x & 0x01) << 11)
#define TP_TF_CNTL_BUSY(x)			((x & 0x01) << 10)
#define TP_TF_CNTL_MODE(x)			((x & 0x01) << 9)
#define TP_TF_CNTL_DU_EN(x)			((x & 0x01) << 8)
#define TP_TF_CNTL_DE_EN(x)			((x & 0x01) << 7)
#define TP_TF_CNTL_XT_EN(x)			((x & 0x01) << 6)
#define TP_TF_CNTL_PID_EN(x)			((x & 0x01) << 5)
#define TP_TF_CNTL_NULL_EN(x)		((x & 0x01) << 4)
#define TP_TF_CNTL_TRERR_EN(x)		((x & 0x01) << 3)
#define TP_TF_CNTL_SYNC_EN(x)		((x & 0x01) << 2)
#define TP_TF_CNTL_RST_EN(x)			((x & 0x01) << 1)
#define TP_TF_CNTL_WRITE_DATA(x)	    (x & 0x01)

#define TP_TF_FRMCFG_FRM_EN_BIT		(0x00000001 << 12)
#define TP_TF_FRMCFG_PACKET_SIZE_EN_BIT		(0x00000003 << 14)

#define TP_TF_FRMCFG_SYNC_BYTE(x)	    ((x & 0xFF) << 24)
#define TP_TF_FRMCFG_DROPNO(x)		((x & 0x0F) << 20)
#define TP_TF_FRMCFG_LOCKNO(x)		((x & 0x0F) << 16)
#define TP_TF_FRMCFG_PACKET_SIZE(x)	    ((x & 0x03) << 14)
#define TP_TF_FRMCFG_DATA_ORDER(x)	    ((x & 0x01) << 13)
#define TP_TF_FRMCFG_FRM_EN(x)		((x & 0x01) << 12)
#define TP_TF_FRMCFG_FORCEDROP(x)	    ((x & 0x01) << 11)
#define TP_TF_FRMCFG_SYNCMODE(x)		((x & 0x1F) << 06)
#define TP_TF_FRMCFG_SERIAL(x)		((x & 0x01) << 05)
#define TP_TF_FRMCFG_DATAPIN(x)		((x & 0x01) << 04)
#define TP_TF_FRMCFG_ERR_POL(x)		((x & 0x01) << 03)
#define TP_TF_FRMCFG_SYNC_POL(x)		((x & 0x01) << 02)
#define TP_TF_FRMCFG_VAL_POL(x)		((x & 0x01) << 01)
#define TP_TF_FRMCFG_CLK_POL(x)		((x & 0x01) << 00)

#define TP_TF_FRMCFG_SERIAL_EN_BIT          (0x00000001 << 5)

#define TP_TF_INT_OVER_FLOW_BIT		(0x01 << 3)
#define TP_TF_INT_DROP_BIT			(0x01 << 2)
#define TP_TF_INT_SYNC_BIT			(0x01 << 1)
#define TP_TF_INT_NONE			(0)
#define TP_TF_INT_ALL			(TP_TF_INT_OVER_FLOW_BIT | \
					TP_TF_INT_DROP_BIT | \
					TP_TF_INT_SYNC_BIT)

#define TP_TF_INT_MASK(x)			(x & 0x0E)
#define TP_TF_INT_WRITE_DATA(x)		(x & 0x01)

/* PID & SECTION FILTER */
#define TP_PID_PART			0x024
#define TP_PID_CTRL			0x028
#define TP_PID_DATA			0x02c
#define TP_PID_DATA2		0x020
#define TP_PID_DATA3		0x070

#define TPB_PID_PART		0x024
#define TPB_PID_CTRL		0x028
#define TPB_PID_DATA		0x02C
#define TPB_PID_DATA2		0x020
#define TPB_PID_DATA3		0x070

/* TP_PID_CTRL */
#define TP_PID_CTRL_R_W(x)     ((x & 0x01) << 7)
#define TP_PID_CTRL_IDX(x)     ((x & 0x7F))

/* TP_PID_DATA */
#define TP_PID_DATA_SI_EN(x)			((x & 0x01) << 31)
#define TP_PID_DATA_PID_INI(x)	        ((x & 0x01) << 30)
#define TP_PID_DATA_AI_EN(x)		    ((x & 0x01) << 29)
#define TP_PID_DATA_TI_EN(x)			((x & 0x01) << 28)
#define TP_PID_DATA_SEC_IDX(x)		    ((x & 0x3F) << 22)
#define TP_PID_DATA_SEC_EN(x)		    ((x & 0x01) << 21)
#define TP_PID_DATA_DDR_Q(x)			((x & 0x3F) << 15)
#define TP_PID_DATA_CC_EN(x)			((x & 0x01) << 14)
#define TP_PID_DATA_V(x)				((x & 0x01) << 13)
#define TP_PID_DATA_PID(x)				((x & 0x1FFF))
#define TP_PID_DATA_SEC_EN_BIT		    TP_PID_DATA_SEC_EN(1)
#define TP_PID_DATA_SEC_MASK			TP_PID_DATA_SEC_IDX(0x3F)

#define TP_PID_DATA_PARAM_SI_EN(x)	((x >> 31) & 0x01)
#define TP_PID_DATA_PARAM_PID_INI(x)	((x >> 30) & 0x01)
#define TP_PID_DATA_PARAM_AI_EN(x)	((x >> 29) & 0x01)
#define TP_PID_DATA_PARAM_TI_EN(x)	((x >> 28) & 0x01)
#define TP_PID_DATA_PARAM_SEC_IDX(x)	((x >> 22) & 0x3F)
#define TP_PID_DATA_PARAM_SEC_EN(x)	((x >> 21) & 0x01)
#define TP_PID_DATA_PARAM_DDR_Q(x)	((x >> 15) & 0x3F)
#define TP_PID_DATA_PARAM_CC_EN(x)	((x >> 14) & 0x01)
#define TP_PID_DATA_PARAM_V(x)		((x >> 13) & 0x01)
#define TP_PID_DATA_PARAM_PID(x)		(x & 0x1FFF)

/* TP_PID_DATA2 */
#define TP_PID_DATA2_INFO_Q(x)		    (x & 0x3F)
#define TP_PID_DATA2_KEY(x)                 ((x & 0x3F)<<6)
#define TP_PID_DATA2_KEY_INDEX(x)           ((x & 0x3F)<<16)
#define TP_PID_DATA2_PRE_DES(x)			((x & 0x1) << 12)
#define TP_PID_DATA2_PB(x)			    ((x & 0x1) << 13)
#define TP_PID_DATA2_DIS_DESCRAMBLE         (1<<22)

/* TP_PID_DATA */
#define TP_PID_DATA3_EXT_PID(x)			((x & 0x3F))

/* TP_SEC_CTRL */
#define TP_SEC_CTRL					0x030
#define TP_SEC_DATA0					0x034
#define TP_SEC_DATA1					0x038
#define TP_SEC_DATA2					0x03c
#define TP_SEC_DATA3					0x040
#define TP_SEC_DATA4					0x044
#define TP_SEC_DATA5					0x080
#define TP_SEC_DATA6					0x084
#define TP_SEC_DATA7					0x088
#define TP_SEC_DATA8					0x08c
#define TP_SEC_DATA9					0x090
#define TP_SEC_DATA10					0x094
#define TP_SEC_DATA11					0x098

#define TPB_SEC_CTRL					0x030
#define TPB_SEC_DATA0					0x034
#define TPB_SEC_DATA1					0x038
#define TPB_SEC_DATA2					0x03C
#define TPB_SEC_DATA3					0x040
#define TPB_SEC_DATA4					0x044
#define TPB_SEC_DATA5					0x080
#define TPB_SEC_DATA6					0x084
#define TPB_SEC_DATA7					0x088
#define TPB_SEC_DATA8					0x08C
#define TPB_SEC_DATA9					0x090
#define TPB_SEC_DATA10					0x094
#define TPB_SEC_DATA11					0x098

#define TP_SEC_CTRL_R_W(x)		((x & 0x01)<<6)
#define TP_SEC_CTRL_IDX(x)		((x & 0x3F))

#define SET_TP_SEC_DATA4_SP_FILTER(x)	((x & 0x03)<<12)
#define SET_TP_SEC_DATA4_SP_MASK(x)	((x & 0x03)<<10)
#define SET_TP_SEC_DATA4_CRC_EN(x)	((x & 0x01)<<9)
#define SET_TP_SEC_DATA4_P_D(x)	((x & 0x01)<<8)
#define SET_TP_SEC_DATA4_P_N(x)	((x & 0x01)<<7)
#define SET_TP_SEC_DATA4_LAST(x)	((x & 0x01)<<6)
#define SET_TP_SEC_DATA4_NEXT_SEC(x)	((x & 0x3F))

/* TP_DES_CTRL */
#define TP_TP0_DES_CNTL		0x050
#define TP_TP1_DES_CNTL		0x0a0

#define TPB_TP0_DES_CNTL	0x050
#define TPB_TP1_DES_CNTL	0x0A0

#define TP_KEY_INFO_H		0x058
#define TP_KEY_INFO_L		0x05c
#define TP_KEY_INFO_0		0x300
#define TP_KEY_INFO_1		0x304
#define TP_KEY_INFO_2		0x308
#define TP_KEY_INFO_3		0x30C
#define TP_KEY_INFO_4		0x310
#define TP_KEY_INFO_5		0x314
#define TP_KEY_INFO_6		0x318
#define TP_KEY_INFO_7		0x31C
#define TP_KEY_CTRL			0x060

#define TPB_KEY_INFO_H		0x058
#define TPB_KEY_INFO_L		0x05c
#define TPB_KEY_CTRL		0x060

#define TP_CTRL_RW(x)		((x & 1) << 7)
#define TP_CTRL_IDX(x)		((x & 0x7F))

#define TP_TP0_IV_MSB_0     0x088
#define TP_TP0_IV_MSB_1     0x08C
#define TP_TP0_IV_LSB_0     0x090
#define TP_TP0_IV_LSB_1     0x094

#define TP_KEY_MASK         0x320

/* TP_TPx_DES_CNTL */
#define TDES_ABC_MODE(x)	((x & 0x01)<<20)
#define TDES_ABC_MODE_BIT	(0x00000001 << 20)

#define OFB_MODE(x)			((x & 3)<<18)
    #define OFB_MARS		0x00
    #define KEEP_CLEAR		0x01
    #define OFB_ON			0x02

#define CSA_MODE(x)			((x & 3)<<16)
    #define CSA_MANUAL_MODE	0x00
    #define CSA_MODE1		0x01
    #define CSA_MODE2		0x02

#define MULTI2_ROUND(x)		((x & 0xFF)<<8)
#define MULTI2_MODE(x)		((x & 1)<<7)

#define MAP_11(x)			((x & 1)<<6)
#define MAP_10(x)			((x & 1)<<5)
#define MAP_01(x)			((x & 1)<<4)
#define DES_MODE(x)			((x & 1)<<3)
#define CBC						0
#define ECB						1
#define MODE(x)				((x & 7))
#define DES						0
#define TDES					1
#define CSA						2
#define MULTI2					3
#define AES_128_CBC				4
#define AES_128_RCBC_CS			5

/* CRC */
#define TP_CRC_INIT			0x0EC
#define TPB_CRC_INIT		0x0EC

/* DDR RING BUFFER */
#define TP_THRESHOLD					0x0F0
#define TP_FULLNESS					0x0F4
#define TP_RING_CTRL					0x0FC
#define TP_RING_LIMIT				0x100
#define TP_RING_BASE					0x104
#define TP_RING_RP					0x108
#define TP_RING_WP					0x10C

#define TPB_THRESHOLD					0x0F0
#define TPB_FULLNESS					0x0F4
#define TPB_RING_CTRL					0x0FC
#define TPB_RING_LIMIT				0x100
#define TPB_RING_BASE					0x104
#define TPB_RING_RP					0x108
#define TPB_RING_WP					0x10c

#define TP_RING_CTRL_WM_LIMIT_BIT		    (0x00000001 << 3)
#define TP_RING_CTRL_WM_BASE_BIT			(0x00000001 << 2)
#define TP_RING_CTRL_WM_RP_BIT			(0x00000001 << 1)
#define TP_RING_CTRL_WM_WP_BIT			(0x00000001 << 0)

#define TP_RING_CTRL_WM(x)				((x & 0x0F) << 7)
#define TP_RING_CTRL_R_W(x)				((x & 0x01) << 6)
#define TP_RING_CTRL_IDX(x)				(x & 0x3F)

#define TP_RING_AVAIL_INT_0				0x600
#define TP_RING_AVAIL_INT_1				0x604
#define TP_RING_AVAIL_INT_2				0x608
#define TP_RING_AVAIL_INT_3				0x60c

#define TP_RING_FULL_INT_0				0x610
#define TP_RING_FULL_INT_1				0x614
#define TP_RING_FULL_INT_2				0x618
#define TP_RING_FULL_INT_3				0x61c

#define TP_RING_AVAIL_INT_EN_0		    0x620
#define TP_RING_AVAIL_INT_EN_1		    0x624
#define TP_RING_AVAIL_INT_EN_2		    0x628
#define TP_RING_AVAIL_INT_EN_3		    0x62c

#define TP_RING_FULL_INT_EN_0		    0x630
#define TP_RING_FULL_INT_EN_1		    0x634
#define TP_RING_FULL_INT_EN_2		    0x638
#define TP_RING_FULL_INT_EN_3		    0x63c

#define TP0_M2M_RING_LIMIT		0x640
#define TP0_M2M_RING_BASE		0x644
#define TP0_M2M_RING_RP			0x648
#define TP0_M2M_RING_WP			0x64c
#define TP0_M2M_RING_CTRL		0x650

#define TP1_M2M_RING_LIMIT		0x6a0
#define TP1_M2M_RING_BASE		0x6a4
#define TP1_M2M_RING_RP			0x6a8
#define TP1_M2M_RING_WP			0x6ac
#define TP1_M2M_RING_CTRL		0x6b0

#define TPB_TP0_M2M_RING_LIMIT		0x640
#define TPB_TP0_M2M_RING_BASE		0x644
#define TPB_TP0_M2M_RING_RP			0x648
#define TPB_TP0_M2M_RING_WP			0x64c
#define TPB_TP0_M2M_RING_CTRL		0x650

#define TPB_TP1_M2M_RING_LIMIT		0x6a0
#define TPB_TP1_M2M_RING_BASE		0x6a4
#define TPB_TP1_M2M_RING_RP			0x6a8
#define TPB_TP1_M2M_RING_WP			0x6ac
#define TPB_TP1_M2M_RING_CTRL		0x6b0

#define TP_DEBUG					0x690
#define TPB_DEBUG					0x690

#define TPB_RING_AVAIL_INT_0				0x600
#define TPB_RING_AVAIL_INT_1				0x604
#define TPB_RING_AVAIL_INT_2				0x608
#define TPB_RING_AVAIL_INT_3				0x60C

#define TPB_RING_FULL_INT_0				0x610
#define TPB_RING_FULL_INT_1				0x614
#define TPB_RING_FULL_INT_2				0x618
#define TPB_RING_FULL_INT_3				0x61c

#define TPB_RING_AVAIL_INT_EN_0		    0x620
#define TPB_RING_AVAIL_INT_EN_1		    0x624
#define TPB_RING_AVAIL_INT_EN_2		    0x628
#define TPB_RING_AVAIL_INT_EN_3		    0x62c

#define TPB_RING_FULL_INT_EN_0		    0x630
#define TPB_RING_FULL_INT_EN_1		    0x634
#define TPB_RING_FULL_INT_EN_2		    0x638
#define TPB_RING_FULL_INT_EN_3		    0x63c

#define TP_RING_INT_MASK(x)             (0x00000001 << (x+1))
#define TP_RING_FULL_INT_WRITE_DATA(x)	(x & 0x01)

#define TP_M2M_RING_CTRL_GO_BIT         (0x00000001<<1)
#define TP_M2M_RING_CTRL_STOP_BIT	(0x00000001<<2)
#define TP_M2M_RING_CTRL_EMPYT_EN_BIT	(0x00000001<<3)
#define TP_M2M_RING_CTRL_EMPYT_BIT	(0x00000001<<4)
#define TP_M2M_RING_CTRL_WRITE_DATA(x)	(x & 0x01)

#define CW_LENGTH           2  /* one keyset using 2 entries */

/* PCR */
#define TP_PCR_90K_CNT		            0x244
#define TPB_PCR_90K_CNT                         0x244

#define TP0_PCR_CTL                             0x248
#define TP0_PCR_CTL_1                           0x264
#define TP0_PCR_CTL_2                           0x268
#define TP0_PCR_CTL_3                           0x26C

#define TP1_PCR_CTL                             0x24C
#define TP1_PCR_CTL_1                           0x270
#define TP1_PCR_CTL_2                           0x274
#define TP1_PCR_CTL_3                           0x278

#define TPB0_PCR_CTL                            0x248
#define TPB0_PCR_CTL_1                          0x264
#define TPB0_PCR_CTL_2                          0x268
#define TPB0_PCR_CTL_3                          0x26C

#define TPB1_PCR_CTL                            0x24C
#define TPB1_PCR_CTL_1                          0x270
#define TPB1_PCR_CTL_2                          0x274
#define TPB1_PCR_CTL_3                          0x278

    #define TP_PCR_CTL_STC_EXTRA_FUNC_ENA         (1<<8)
    #define TP_PCR_CTL_STC_EXTRA_PID_ADDR(x)      (x & 0x7F)
    #define TP_PCR_CTL_STC_EXTRA_FUNC_ENA_CH(x)   ((1<<8) << x)
    #define TP_PCR_CTL_STC_EXTRA_PID_ADDR_CH(x, y) ((x & 0x7F) << y)
    #define TP_PCR_CTL_LATCH_8CH_SEL(x)           ((x & 0x07) << 4)

#define TP_PCR_LATCH                            0x254
#define TPB_PCR_LATCH                           0x254
    #define TP_PCR_LATCH_LATCH_ENABLE           (1<<8)
    #define TP_PCR_LATCH_LATCH_SEL_TP0          0
    #define TP_PCR_LATCH_LATCH_SEL_TP1          1
    #define TP_PCR_LATCH_LATCH_SEL_TP2          2

#define TP_PCR_BASE                             0x258
#define TP_PCR_EXT                              0x25C
#define TPB_PCR_BASE                            0x258
#define TPB_PCR_EXT                             0x25C
  #define TP_PCR_EXT_PCR_BASE_HIGH(x)           ((x >> 16) & 0x1)

#define TP_PCR_SYSTEM                           0x260
#define TPB_PCR_SYSTEM                          0x260

/* Ext PID Control*/
#define TP_0_EXT_PID_CNTL	0x500
#define TP_1_EXT_PID_CNTL	0x504
#define TPB_0_EXT_PID_CNTL	0x500
#define TPB_1_EXT_PID_CNTL	0x504
  #define EXT_PID_EXTRACT_BYTE(x)	(x & 0x03)
  #define EXT_HEADER_MODE(x)		((x & 0x01) << 4)

#define TP_DMY_A  0x65C
#define TPB_DMY_A 0x65C
  #define TP_TF_AF01_FORCE_EN(x)  (0x00000001 << (4+x))
  #define TP_TF_RAW_MODE_EN(x)    (0x00000001 << (6+x))

/* TP CONTROL */
#define TPC_TF0_CNTL         0x000
#define TPC_TF0_STRM_ID_0    0x208
#define TPC_TF0_STRM_ID_1    0x20C
#define TPC_TF0_STRM_ID_2    0x210
#define TPC_TF0_STRM_ID_3    0x214
#define TPC_TF0_STRM_ID_VAL  0x218
#define TPC_TF0_CNT          0x004
#define TPC_TF0_DRP_CNT      0x008
#define TPC_TF0_ERR_CNT      0x00C
#define TPC_TF0_FRMCFG       0x824
#define TPC_TF0_INT          0x828
#define TPC_TF0_INT_EN       0x82C

/* PCR */
#define TPC_PCR_90K_CNT  0x244

#define TPC0_PCR_CTL     0x248
#define TPC0_PCR_CTL_1   0x264
#define TPC0_PCR_CTL_2   0x268
#define TPC0_PCR_CTL_3   0x26C

#define TPC_PCR_LATCH    0x254
#define TPC_PCR_BASE     0x258
#define TPC_PCR_EXT      0x25C
#define TPC_PCR_SYSTEM   0x260

/* TF_CTRL_SWC */
#define TP_TF0_CTRL_SWC   0x840
#define TP_TF1_CTRL_SWC   0x844

#define TPB_TF0_CTRL_SWC  0x840
#define TPB_TF1_CTRL_SWC  0x844

#define TPC_TF0_CTRL_SWC  0x840

/* PID Filter */
#define TPC_PID_PART   0x024
#define TPC_PID_CTRL   0x028
#define TPC_PID_DATA   0x02C
#define TPC_PID_DATA2  0x020
#define TPC_PID_DATA3  0x070

/* TP_SEC_CTRL */
#define TPC_SEC_CTRL    0x030
#define TPC_SEC_DATA0   0x034
#define TPC_SEC_DATA1   0x038
#define TPC_SEC_DATA2   0x03C
#define TPC_SEC_DATA3   0x040
#define TPC_SEC_DATA4   0x044
#define TPC_SEC_DATA5   0x080
#define TPC_SEC_DATA6   0x084
#define TPC_SEC_DATA7   0x088
#define TPC_SEC_DATA8   0x08C
#define TPC_SEC_DATA9   0x090
#define TPC_SEC_DATA10  0x094
#define TPC_SEC_DATA11  0x098

/* TP_DES_CTRL */
#define TPC_TP0_DES_CNTL	0x050

#define TP_KEY_INFO_SWC_0   0x300
#define TP_KEY_INFO_SWC_1   0x304
#define TP_KEY_INFO_SWC_2   0x308
#define TP_KEY_INFO_SWC_3   0x30C
#define TP_KEY_INFO_SWC_4   0x310
#define TP_KEY_INFO_SWC_5   0x314
#define TP_KEY_INFO_SWC_6   0x318
#define TP_KEY_INFO_SWC_7   0x31C
#define TP_KEY_INFO_SWC_8   0x320
#define TP_KEY_INFO_SWC_9   0x324
#define TP_KEY_INFO_SWC_A   0x328
#define TP_KEY_INFO_SWC_B   0x32C

#define TP_KEY_HEADER_SWC   0x330
#define TP_KEY_MASK_SWC     0x334
#define TP_KEY_CTRL_SWC     0x060

/* define register but these registers are invalid */
#define TPB_KEY_INFO_SWC_0   0x300
#define TPB_KEY_INFO_SWC_1   0x304
#define TPB_KEY_INFO_SWC_2   0x308
#define TPB_KEY_INFO_SWC_3   0x30C
#define TPB_KEY_INFO_SWC_4   0x310
#define TPB_KEY_INFO_SWC_5   0x314
#define TPB_KEY_INFO_SWC_6   0x318
#define TPB_KEY_INFO_SWC_7   0x31C
#define TPB_KEY_INFO_SWC_8   0x320
#define TPB_KEY_INFO_SWC_9   0x324
#define TPB_KEY_INFO_SWC_A   0x328
#define TPB_KEY_INFO_SWC_B   0x32C

#define TPB_KEY_HEADER_SWC   0x330
#define TPB_KEY_MASK_SWC     0x334
#define TPB_KEY_CTRL_SWC     0x060

/* CRC */
#define TPC_CRC_INIT  0x0EC

/* DDR RING BUFFER */
#define TPC_THRESHOLD            0x0F0
#define TPC_FULLNESS             0x0F4
#define TPC_RING_CTRL            0x0FC
#define TPC_RING_LIMIT           0x100
#define TPC_RING_BASE            0x104
#define TPC_RING_RP              0x108
#define TPC_RING_WP              0x10C

#define TPC_RING_AVAIL_INT_0     0x600
#define TPC_RING_AVAIL_INT_1     0x604
#define TPC_RING_AVAIL_INT_2     0x608
#define TPC_RING_AVAIL_INT_3     0x60C

#define TPC_RING_FULL_INT_0      0x610
#define TPC_RING_FULL_INT_1      0x614
#define TPC_RING_FULL_INT_2      0x618
#define TPC_RING_FULL_INT_3      0x61C

#define TPC_RING_AVAIL_INT_EN_0  0x620
#define TPC_RING_AVAIL_INT_EN_1  0x624
#define TPC_RING_AVAIL_INT_EN_2  0x628
#define TPC_RING_AVAIL_INT_EN_3  0x62C

#define TPC_RING_FULL_INT_EN_0   0x630
#define TPC_RING_FULL_INT_EN_1   0x634
#define TPC_RING_FULL_INT_EN_2   0x638
#define TPC_RING_FULL_INT_EN_3   0x63C

#define TPC_TP0_M2M_RING_LIMIT   0x640
#define TPC_TP0_M2M_RING_BASE    0x644
#define TPC_TP0_M2M_RING_RP      0x648
#define TPC_TP0_M2M_RING_WP      0x64C
#define TPC_TP0_M2M_RING_CTRL    0x650

/* Dummy*/
#define TP_DMY_A     0x65C
#define TP_DMY_B     0x660
#define TP_SWC_DMY_A 0x848
#define TP_SWC_DMY_B 0x84C

#define TPB_DMY_A     0x65C
#define TPB_DMY_B     0x660
#define TPB_SWC_DMY_A 0x848
#define TPB_SWC_DMY_B 0x84C

#define TPC_DMY_A     0x65C
#define TPC_DMY_B     0x660
#define TPC_SWC_DMY_A 0x848
#define TPC_SWC_DMY_B 0x84C

#define TPC_DEBUG     0x690

/* PESE Timeout */
#define TPC_PESE_TO0  0x880
#define TPC_PESE_TO1  0x884

/* Ext PID Control*/
#define TPC_0_EXT_PID_CNTL 0x500

/* TF_CNTL */
#define TP_TF_CNTL_RAW_MODE_BIT         (0x00000001 << 29)
#define TP_TF_CNTL_AF_FORCE_00_PKT_BIT  (0x00000001 << 28)
#define TP_TF_CNTL_AF_FORCE_ALL_PKT_BIT (0x00000001 << 27)
#define TP_TF_CNTL_PID_DEC_CHK_BIT      (0x00000001 << 26)

#define TP_TF_CNTL_RAW_MODE_EN(x)         ((x & 0x01) << 29)
#define TP_TF_CNTL_AF_FORCE_00_PKT_EN(x)  ((x & 0x01) << 28)
#define TP_TF_CNTL_AF_FORCE_ALL_PKT_EN(x) ((x & 0x01) << 27)
#define TP_TF_CNTL_PID_DEC_CHK_EN(x)      ((x & 0x01) << 26)

/* TF_INT */
#define TF_INT_TPC(x)                ((x & 0x01) << 18)
#define TF_INT_TPB(x)                ((x & 0x01) << 17)
#define TF_INT_TP(x)                 ((x & 0x01) << 16)
#define TF_INT_CW_ILLEGAL_W(x)       ((x & 0x01) << 9)
#define TF_INT_CW_ILLEGAL_USAGE(x)   ((x & 0x01) << 8)
#define TF_INT_CW_ILLEGAL_ALGO(x)    ((x & 0x01) << 7)
#define TF_INT_CW_ILLEGAL_ENC_DEC(x) ((x & 0x01) << 6)
#define TF_INT_CW_ILLEGAL_ACTV(x)    ((x & 0x01) << 5)

#define TF_INT_TPC_BIT                (0x01 << 18)
#define TF_INT_TPB_BIT                (0x01 << 17)
#define TF_INT_TP_BIT                 (0x01 << 16)
#define TF_INT_CW_ILLEGAL_W_BIT       (0x01 << 9)
#define TF_INT_CW_ILLEGAL_USAGE_BIT   (0x01 << 8)
#define TF_INT_CW_ILLEGAL_ALGO_BIT    (0x01 << 7)
#define TF_INT_CW_ILLEGAL_ENC_DEC_BIT (0x01 << 6)
#define TF_INT_CW_ILLEGAL_ACTV_BIT    (0x01 << 5)

#define TP_TF_INT_ALL_HANK     (0x703FE)
#define TP_TF_INT_CW_HANK      (TF_INT_CW_ILLEGAL_ACTV_BIT | \
				TF_INT_CW_ILLEGAL_ENC_DEC_BIT | \
				TF_INT_CW_ILLEGAL_ALGO_BIT | \
				TF_INT_CW_ILLEGAL_USAGE_BIT| \
				TF_INT_CW_ILLEGAL_W_BIT)

#define TP_TF_INT_MASK_HANK(x) (x & 0x3FE)
#define TP_TF_INT_VALUE_MASK_HANK(x) (x & 0x703FE)

/* TF_INT_EN */
#define TF_INT_CW_ILLEGAL_W_EN(x)       ((x & 0x01) << 9)
#define TF_INT_CW_ILLEGAL_USAGE_EN(x)   ((x & 0x01) << 8)
#define TF_INT_CW_ILLEGAL_ALGO_EN(x)    ((x & 0x01) << 7)
#define TF_INT_CW_ILLEGAL_ENC_DEC_EN(x) ((x & 0x01) << 6)
#define TF_INT_CW_ILLEGAL_ACTV_EN(x)    ((x & 0x01) << 5)

/* TF_CTRL_SWC */
#define TF_CTRL_SWC_SOLITARY_KEEP_CLEAR(x) ((x & 0x01) << 2)
#define TF_CTRL_SWC_PACKET_SIZE(x)         (x & 0x03)

/* PID_PART */
#define PID_PART_CRC_ADDR_SEL(x)  ((x & 0x01) << 16)
#define PID_PART_CRC_ADDR_SEL_BIT (0x01 << 16)


/* TP_PID_DATA3 */
#define TP_PID_DATA3_ENC_KEY_ODD(x) ((x & 0x01) << 18)
#define TP_PID_DATA3_ENC(x)         ((x & 0x01) << 17)
#define TP_PID_DATA3_DEC(x)         ((x & 0x01) << 16)
#define TP_PID_DATA3_CLEAR_HEAD(x)  ((x & 0x01) << 15)
#define TP_PID_DATA3_DDR_Q2_V(x)    ((x & 0x01) << 14)
#define TP_PID_DATA3_DDR_Q2(x)      ((x & 0x3F) << 8)

/* DES CNTL */
#define CSAv3 6

#define KEY_HEADER_ALGO_CW1_DES      (0x1)
#define KEY_HEADER_ALGO_CW1_TDES_ABA (0x2)
#define KEY_HEADER_ALGO_CW1_AES_128  (0x4)
#define KEY_HEADER_ALGO_CW1_CSA2     (0xD)
#define KEY_HEADER_ALGO_CW1_CSA3     (0xE)

#define KEY_HEADER_ALGO_CW2_TDES_ABC (0x1)
#define KEY_HEADER_ALGO_CW2_AES_192  (0x2)
#define KEY_HEADER_ALGO_CW2_AES_256  (0x4)
#define KEY_HEADER_ALGO_CW2_MULTI2   (0x8)

#define KEY_HEADER_USAGE_CW1_TP      (0x1)
#define KEY_HEADER_USAGE_CW1_CP_ONLY (0x2)
#define KEY_HEADER_USAGE_CW1_KL_ONLY (0x4)

#define KEY_HEADER_USAGE_CW2_TP      (0x1)
#define KEY_HEADER_USAGE_CW2_CP_ONLY (0x2)

#define KEY_DEACTIVATE (0x1)
#define KEY_ACTIVATE (0x2)

#define ENGINE_DEC (0x1)
#define ENGINE_ENC (0x2)

#define TP_KEY_HEADER_ACTIVATE(x)  ((x & 0x03) << 9)
#define TP_KEY_HEADER_ENC_DEC(x)   ((x & 0x03) << 7)
#define TP_KEY_HEADER_ALGORITHM(x) ((x & 0x0F) << 3)
#define TP_KEY_HEADER_USAGE(x)     (x & 0x07)

/* TP Scramble */
#define TP_SWC_DMY_A_SCRAMBLE(x) (x & 0x0F)

/* NX Mask enc*/
#define ENABLE_TP_NX_MASK_ENC(x) (1 << x)
#endif
