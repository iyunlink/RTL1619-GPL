/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019 RealTek Inc.
 */

#ifndef _RTK_DPTX_REG_H
#define _RTK_DPTX_REG_H

#define BIT0	0x00000001
#define BIT1	0x00000002
#define BIT2	0x00000004
#define BIT3	0x00000008
#define BIT4	0x00000010
#define BIT5	0x00000020
#define BIT6	0x00000040
#define BIT7	0x00000080
#define BIT8	0x00000100
#define BIT9	0x00000200
#define BIT10	0x00000400
#define BIT11	0x00000800
#define BIT12	0x00001000
#define BIT13	0x00002000
#define BIT14	0x00004000
#define BIT15	0x00008000
#define BIT16	0x00010000
#define BIT17	0x00020000
#define BIT18	0x00040000
#define BIT19	0x00080000
#define BIT20	0x00100000
#define BIT21	0x00200000
#define BIT22	0x00400000
#define BIT23	0x00800000
#define BIT24	0x01000000
#define BIT25	0x02000000
#define BIT26	0x04000000
#define BIT27	0x08000000
#define BIT28	0x10000000
#define BIT29	0x20000000
#define BIT30	0x40000000
#define BIT31	0x80000000

//--------------------------------------
//       LVDS and EDP PLL Control Register
//--------------------------------------
#define DISP_PLL_DIV2	0x24
#define PLL_HDMI	0x190
#define PLL_HDMI_LDO1	0x230
#define PLL_EDP1	0x248
#define PLL_EDP2	0x24c
#define PLL_PIXEL1	0x250
#define PLL_PIXEL2	0x254

/* PLL_PIXEL1 */
#define RESET_PLL	BIT14

//--------------------------------------
//       EDP PLL_SSC_DIG Control Register
//--------------------------------------
#define PLL_SSC_DIG_EDP0	0x5e0
#define PLL_SSC_DIG_EDP1	0x5e4
#define PLL_SSC_DIG_EDP2	0x5e8
#define PLL_SSC_DIG_EDP3	0x5ec
#define PLL_SSC_DIG_EDP4	0x5f0
#define PLL_SSC_DIG_EDP5	0x5f4
#define PLL_SSC_DIG_EDP_DBG1	0x5f8
#define PLL_SSC_DIG_EDP_DBG2	0x5fc

//--------------------------------------
//       PIXEL PLL_SSC_DIG Control Register
//--------------------------------------
#define PLL_SSC_DIG_PIXEL0	0x600
#define PLL_SSC_DIG_PIXEL1	0x604
#define PLL_SSC_DIG_PIXEL2	0x608

//--------------------------------------
//       LVDS control register
//--------------------------------------
#define AIF_EDP_1		0x40
#define AIF_EDP_2		0x44
#define AIF_EDP_3		0x48
#define CT_CTRL			0x100
#define CT_COE_1		0x108
#define CT_COE_2		0x10c
#define CT_COE_3		0x110
#define CT_COE_4		0x114
#define CT_COE_5		0x118
#define CT_COE_6		0x11c
#define DH_WIDTH		0x404
#define DH_TOTAL		0x408
#define DH_DEN_START_END	0x40c
#define DV_DEN_START_END_F1	0x410
#define DV_TOTAL		0x418
#define DV_VS_START_END_F1	0x41c
#define DV_SYNC_INT		0x42c

/* AIF_EDP_2 */
#define LANE0_EMPHASIS_EN	(1 << 20)
#define LANE0_EMPHASIS_2X	(16)
#define LANE0_EMPHASIS_MASK2	(1 << LANE0_EMPHASIS_2X)
#define LANE0_EMPHASIS_MASK1	(0xF)

/* AIF_EDP_3 */
#define SEL_IBX			(1 << 31)
#define LANE0_DRV_2X		(1 << 24)
#define LANE0_DRV_MASK		(0xF)

//--------------------------------------
//        DPTX Digital PHY CTRL
//--------------------------------------
#define DP_PHY_CTRL		0x000
#define DPTX_ML_PAT_SEL		0x004
#define CUSTOM_PATTERN_0	0x008
#define CUSTOM_PATTERN_1	0x00C
#define CUSTOM_PATTERN_2	0x010
#define CUSTOM_PATTERN_3	0x014
#define CUSTOM_PATTERN_4	0x018
#define CUSTOM_PATTERN_5	0x01C
#define CUSTOM_PATTERN_6	0x020
#define CUSTOM_PATTERN_7	0x024
#define CUSTOM_PATTERN_8	0x028
#define CUSTOM_PATTERN_9	0x02C
#define COMPLIANCE_EYE_PATTERN	0x030
#define DPTX_PHY_CTRL		0x034
#define DPTX_LANE_SWAP		0x038
#define DPTX_8B10B_TST		0x03C
#define DPTX_PHY_DUMMY		0x040
#define RIV0			0x044
#define RIV1			0x048
#define RIV2			0x04C
#define RIV3			0x050
#define RIV4			0x054
#define RIV5			0x058
#define RIV6			0x05C
#define RIV7			0x060
#define DPTX_HDCP_CTRL1		0x0C0
#define DPTX_HDCP_CTRL2		0x0C4
#define DP_HDCP_KEY_DL_PORT	0x0C8
#define DP_HDCP_KEY_OUTPUT	0x0CC
#define AN_BYTE_7		0x0D0
#define AN_BYTE_6		0x0D4
#define AN_BYTE_5		0x0D8
#define AN_BYTE_4		0x0DC
#define AN_BYTE_3		0x0E0
#define AN_BYTE_2		0x0E4
#define AN_BYTE_1		0x0E8
#define AN_BYTE_0		0x0EC
#define M0_BYTE_7		0x0F0
#define M0_BYTE_6		0x0F4
#define M0_BYTE_5		0x0F8
#define M0_BYTE_4		0x0FC
#define M0_BYTE_3		0x100
#define M0_BYTE_2		0x104
#define M0_BYTE_1		0x108
#define M0_BYTE_0		0x10C
#define KM_BYTE_6		0x110
#define KM_BYTE_5		0x114
#define KM_BYTE_4		0x118
#define KM_BYTE_3		0x11C
#define KM_BYTE_2		0x120
#define KM_BYTE_1		0x124
#define KM_BYTE_0		0x128
#define R0_BYTE_MSB		0x12C
#define R0_BYTE_LSB		0x130
#define RI_BYTE_MSB		0x134
#define RI_BYTE_LSB		0x138
#define BKSV_0			0x13C
#define BKSV_1			0x140
#define BKSV_2			0x144
#define BKSV_3			0x148
#define BKSV_4			0x14C
#define HDCP_TX_LIP_H		0x150
#define HDCP_TX_LIP_L		0x154
#define DP_HDCP_TX		0x158
#define DP_HDCP_BIST		0x15C
#define DP_HDCP_AN_SEED		0x160
#define HDCP_IRQ_EVENT		0x164
#define DPTX_HDCP_TST		0x168
#define DP_HDCP_TX_SHA_CTRL	0x16C
#define DP_HDCP_TX_SHA_DATA_3	0x170
#define DP_HDCP_TX_SHA_DATA_2	0x174
#define DP_HDCP_TX_SHA_DATA_1	0x178
#define DP_HDCP_TX_SHA_DATA_0	0x17C
#define DP_HDCP_TX_SHA_OUT_3	0x180
#define DP_HDCP_TX_SHA_OUT_2	0x184
#define DP_HDCP_TX_SHA_OUT_1	0x188
#define DP_HDCP_TX_SHA_OUT_0	0x18C
#define HDCP_ECF_BYTE0		0x190
#define HDCP_ECF_BYTE1		0x194
#define HDCP_ECF_BYTE2		0x198
#define HDCP_ECF_BYTE3		0x19C
#define HDCP_ECF_BYTE4		0x1A0
#define HDCP_ECF_BYTE5		0x1A4
#define HDCP_ECF_BYTE6		0x1A8
#define HDCP_ECF_BYTE7		0x1AC
#define DP_STREAM_1_PBN		0x1B0
#define DP_STREAM_2_PBN		0x1B4
#define DP_STREAM_3_PBN		0x1B8
#define DP_STREAM_4_PBN		0x1BC
#define HDCP_AES_CIPHER_KEY_15	0x200
#define HDCP_AES_CIPHER_KEY_14	0x204
#define HDCP_AES_CIPHER_KEY_13	0x208
#define HDCP_AES_CIPHER_KEY_12	0x20C
#define HDCP_AES_CIPHER_KEY_11	0x210
#define HDCP_AES_CIPHER_KEY_10	0x214
#define HDCP_AES_CIPHER_KEY_9	0x218
#define HDCP_AES_CIPHER_KEY_8	0x21C
#define HDCP_AES_CIPHER_KEY_7	0x220
#define HDCP_AES_CIPHER_KEY_6	0x224
#define HDCP_AES_CIPHER_KEY_5	0x228
#define HDCP_AES_CIPHER_KEY_4	0x22C
#define HDCP_AES_CIPHER_KEY_3	0x230
#define HDCP_AES_CIPHER_KEY_2	0x234
#define HDCP_AES_CIPHER_KEY_1	0x238
#define HDCP_AES_CIPHER_KEY_0	0x23C
#define HDCP22_CTRL		0x240
#define HDCP22_TYPE_AES_0	0x244
#define HDCP22_TYPE_AES_1	0x248
#define DUMMY_1			0x268
#define DUMMY_2			0x26C
#define DUMMY_3			0x270

/* DPTX_ML_PAT_SEL */
#define PAT_SEL_SHIFT		(0x4)
#define PAT_CHANGE		(0x1)

//--------------------------------------
//        DPTX link MAC
//--------------------------------------
#define DP_MAC_CTRL			0x280
#define DP_RESET_CTRL			0x284
#define DP_DEBUG_CTRL			0x288
#define DPTX_IRQ_CTRL			0x28C
#define PG_FIFO_CTRL			0x290
#define MAX_WL				0x294
#define LFIFO_WL			0x298
#define PG_INTERRUPT_CTRL		0x29C
#define MN_VID_AUTO_EN_1		0x2A0
#define MN_M_VID_H			0x2A4
#define MN_M_VID_M			0x2A8
#define MN_M_VID_L			0x2AC
#define MN_N_VID_H			0x2B0
#define MN_N_VID_M			0x2B4
#define MN_N_VID_L			0x2B8
#define MVID_AUTO_H			0x2BC
#define MVID_AUTO_M			0x2C0
#define MVID_AUTO_L			0x2C4
#define NVID_ASYNC_M			0x2C8
#define NVID_ASYNC_L			0x2CC
#define MSA_CTRL			0x2D0
#define MSA_MISC0			0x2D4
#define MN_STRM_ATTR_MISC1		0x2D8
#define MN_STRM_ATTR_HTT_M		0x2DC
#define MN_STRM_ATTR_HTT_L		0x2E0
#define MN_STRM_ATTR_HST_M		0x2E4
#define MN_STRM_ATTR_HST_L		0x2E8
#define MN_STRM_ATTR_HWD_M		0x2EC
#define MN_STRM_ATTR_HWD_L		0x2F0
#define MN_STRM_ATTR_HSW_M		0x2F4
#define MN_STRM_ATTR_HSW_L		0x2F8
#define MN_STRM_ATTR_VTTE_M		0x2FC
#define MN_STRM_ATTR_VTTE_L		0x300
#define MN_STRM_ATTR_VST_M		0x304
#define MN_STRM_ATTR_VST_L		0x308
#define MN_STRM_ATTR_VHT_M		0x30C
#define MN_STRM_ATTR_VHT_L		0x310
#define MN_STRM_ATTR_VSW_M		0x314
#define MN_STRM_ATTR_VSW_L		0x318
#define VBID				0x31C
#define VBID_FW_CTL			0x320
#define ARBITER_CTRL			0x324
#define V_DATA_PER_LINE0		0x328
#define V_DATA_PER_LINE1		0x32C
#define TU_SIZE				0x330
#define TU_DATA_SIZE0			0x334
#define TU_DATA_SIZE1			0x338
#define HDEALY0				0x33C
#define HDEALY1				0x340
#define AUTO_HDEALY0			0x344
#define AUTO_HDEALY1			0x348
#define LFIFO_WL_SET			0x34C
#define ARBITER_SEC_END_CNT_HB		0x350
#define ARBITER_SEC_END_CNT_LB		0x354
#define ARBITER_DEBUG			0x358
#define DPTX_CTSFIFO_CTRL		0x35C
#define DPTX_CTSFIFO_RSV1		0x360
#define DPTX_TOP_CTL			0x364
#define DPTX_TOP_RSV1			0x368
#define DPTX_TOP_RSV2			0x36C
#define ARBITER_MIN_H_BLANK_WIDTH_HB	0x370
#define ARBITER_MIN_H_BLANK_WIDTH_LB	0x374
#define ARBITER_INTERRUPT_CTRL		0x378
#define VESA_FMT_REGEN			0x37C
#define DPTX_CLK_GEN			0x380
#define PG_MBIST_CTRL			0x384
#define PG_DRF_MBIST_CTRL		0x388
#define ARBITER_SEC_IDLE_END_CNT	0x3C0
#define DPTX_TOP_RSV3			0x3E0
#define DPTX_TOP_RSV4			0x3E4
#define DPTX_TOP_RSV5			0x3E8
#define DPTX_TOP_RSV6			0x3EC
#define DPTX_TOP_RSV7			0x3F0
#define DPTX_TOP_RSV8			0x3F4
#define DPTX_TOP_RSV9			0x3F8
#define DPTX_TOP_RSV10			0x3FC

/* DPTX_IRQ_CTRL */
#define DPTX_IRQ_EN			BIT7

//--------------------------------------
//        DPTX MAC CTRL1
//--------------------------------------
#define SEC_FUNCTION_CTRL		0x400
#define SEC_RESERVED_0			0x404
#define SEC_DBUF_CTRL			0x408
#define SEC_DEBUG			0x40C
#define SEC_PSR_DB0			0x410
#define SEC_PSR_DB1			0x414
#define SEC_PSR_DB2			0x418
#define SEC_PSR_DB3			0x41C
#define SEC_PSR_DB4			0x420
#define SEC_PSR_DB5			0x424
#define SEC_PSR_DB6			0x428
#define SEC_PSR_DB7			0x42C
#define SEC_PSR_DB8			0x430
#define SEC_PSR_DB9			0x434
#define SEC_PSR_DB10			0x438
#define SEC_PSR_DB11			0x43C
#define SEC_PSR_DB12			0x440
#define SEC_PSR_DB13			0x444
#define SEC_PSR_DB14			0x448
#define SEC_PSR_DB15			0x44C
#define SEC_RESERVED_1			0x450
#define SEC_RESERVED_2			0x454
#define SEC_RESERVED_3			0x458
#define SEC_RESERVED_4			0x45C
#define AUD_FUNCTION_CTRL1		0x460
#define AUD_PAYLOAD_B3			0x464
#define DP_AUD_ID			0x468
#define SEC_RESERVED_6			0x46C
#define AUD_FIFO_CTRL			0x470
#define AUD_LFIFO_MAX_WL		0x474
#define AUD_LFIFO_WL			0x478
#define AUD_INTERRUPT_CTRL		0x47C
#define AUD_TS_MAUD_H			0x480
#define AUD_TS_MAUD_M			0x484
#define AUD_TS_MAUD_L			0x488
#define AUD_TS_NAUD_H			0x48C
#define AUD_TS_NAUD_M			0x490
#define AUD_TS_NAUD_L			0x494
#define SEC_INFO_AUD_DB0		0x498
#define SEC_INFO_AUD_DB1		0x49C
#define SEC_INFO_AUD_DB2		0x4A0
#define SEC_INFO_AUD_DB3		0x4A4
#define SEC_INFO_AUD_DB4		0x4A8
#define SEC_INFO_AUD_DB5		0x4AC
#define SEC_INFO_AUD_DB6		0x4B0
#define SEC_INFO_AUD_DB7		0x4B4
#define SEC_INFO_AUD_DB8		0x4B8
#define SEC_INFO_AUD_DB9		0x4BC
#define SEC_INFO_AVI_DB0		0x4C0
#define SEC_INFO_AVI_DB1		0x4C4
#define SEC_INFO_AVI_DB2		0x4C8
#define SEC_INFO_AVI_DB3		0x4CC
#define SEC_INFO_AVI_DB4		0x4D0
#define SEC_INFO_AVI_DB5		0x4D4
#define SEC_INFO_AVI_DB6		0x4D8
#define SEC_INFO_AVI_DB7		0x4DC
#define SEC_INFO_AVI_DB8		0x4E0
#define SEC_INFO_AVI_DB9		0x4E4
#define SEC_INFO_AVI_DB10		0x4E8
#define SEC_INFO_AVI_DB11		0x4EC
#define SEC_INFO_AVI_DB12		0x4F0
#define SEC_INFO_MPEG_DB0		0x4F4
#define SEC_INFO_MPEG_DB1		0x4F8
#define SEC_INFO_MPEG_DB2		0x4FC
#define SEC_INFO_MPEG_DB3		0x500
#define SEC_INFO_MPEG_DB4		0x504
#define SEC_INFO_MPEG_DB5		0x508
#define SEC_INFO_MPEG_DB6		0x50C
#define SEC_INFO_MPEG_DB7		0x510
#define SEC_INFO_MPEG_DB8		0x514
#define SEC_INFO_MPEG_DB9		0x518
#define AUD_FUNCTION_CTRL2		0x51C
#define AUD_FUNCTION_CTRL3		0x520
#define MST_SEC_PKT_ID			0x524
#define MST_SEC_PKT_HB3			0x528
#define SEC_RS_DECODE_CTRL		0x52C
#define SEC_AUD_FREQDET_CTRL		0x530
#define SEC_AUD_XCLK_DIV		0x534
#define SEC_AUD_FREQ_TIME		0x538
#define SEC_AUD_SAMPLE_CNT_HB		0x53C
#define SEC_AUD_SAMPLE_CNT_LB		0x540
#define AUD_MBIST_CTRL			0x544
#define AUD_DRF_MBIST_CTRL		0x548
#define DP_INFO_FM_RSV0			0x54C
#define DP_INFO_FM_RSV1			0x550
#define DP_INFO_FM_ADR			0x554
#define DP_INFO_FM_DAT			0x558
#define RESERVED_57			0x55C
#define VSC_RX_DB0			0x560
#define VSC_RX_DB1			0x564
#define VSC_RX_DB2			0x568
#define VSC_RX_DB3			0x56C
#define VSC_RX_DB4			0x570
#define VSC_RX_DB5			0x574
#define VSC_RX_DB6			0x578
#define VSC_RX_DB7			0x57C
#define AUD_FREQ_TH_0			0x580
#define AUD_FREQ_TH_1			0x584
#define AUD_FREQ_TH_2			0x588
#define AUD_FREQ_TH_3			0x58C
#define AUD_FREQ_TH_4			0x590
#define AUD_FREQ_TH_5			0x594
#define AUD_FREQ_TH_6			0x598
#define RESERVED_67			0x59C
#define RESERVED_68			0x5A0
#define RESERVED_69			0x5A4
#define VSC_CTRL_0			0x5A8
#define VSC_RX_HB0			0x5AC
#define VSC_RX_HB1			0x5B0
#define VSC_RX_HB2			0x5B4
#define VSC_RX_HB3			0x5B8
#define SEC_AWD_CTRL			0x5C0
#define SEC_AWD_STATUS_0		0x5C4
#define SEC_IRQ_CTRL0			0x5C8
#define SEC_PH_HB0			0x5CC
#define SEC_PH_HB1			0x5D0
#define SEC_PH_HB2			0x5D4
#define SEC_PH_HB3			0x5D8
#define SEC_PH_PB0			0x5DC
#define SEC_PH_PB1			0x5E0
#define SEC_PH_PB2			0x5E4
#define SEC_PH_PB3			0x5E8
#define SEC_PH_PACKET_TYPE		0x5EC
#define SEC_AWD_STATUS_1		0x600
#define DP_INFO_VAR_EN_M		0x604
#define DP_INFO_VAR_EN_L		0x608
#define DP_INFO_VAR_ST_M		0x60C
#define DP_INFO_VAR_ST_L		0x610
#define CH_STATUS_0			0x614
#define CH_STATUS_1			0x618
#define CH_STATUS_2			0x61C
#define CH_STATUS_3			0x620
#define CH_STATUS_4			0x624
#define DP_AVWD_CTRL0			0x628
#define RESERVED_8B			0x62C
#define RESERVED_8C			0x630
#define RESERVED_8D			0x634
#define RESERVED_8E			0x638
#define DPTX_I2S_CTRL			0x63C
#define SEC_IRQ_CTRL1			0x640
#define DUMMY_95			0x654
#define DUMMY_96			0x658
#define DUMMY_97			0x65C
#define DUMMY_98			0x660
#define DUMMY_99			0x664
#define DUMMY_9A			0x668
#define DUMMY_9B			0x66C
#define DUMMY_9C			0x670

//--------------------------------------
//        DPTX link MAC
//--------------------------------------
#define DPTX_PHY_CTRL0			0x680
#define DPTX_PHY_CTRL1			0x684
#define DPTX_RCV_DET0			0x688
#define DPTX_RCV_DET1			0x68C
#define DPTX_PN_SWAP			0x690
#define DPTX_SFIFO_CTRL0		0x69C
#define DPTX_SFIFO_CTRL1		0x6A0
#define DPTX_SFIFO_LANE_SWAP0		0x6A4
#define DPTX_SFIFO_LANE_SWAP1		0x6A8
#define DPTX_SFIFO_INT_EN		0x6AC

//--------------------------------------
//        DPTX AUX Analog PHY CTRL
//--------------------------------------
#define DIG_TX_04	0x980
#define AUX_1		0x984
#define AUX_2		0x988
#define AUX_3		0x98C
#define AUX_4		0x990
#define AUX_5		0x994
#define AUX_6		0x998
#define DIG_TX_03	0x99C

//--------------------------------------
//        DPTX AUX MAC CTRL
//--------------------------------------
#define AUX_TX_CTRL		0xA80
#define AUX_TIMEOUT		0xA84
#define AUX_FSM_STATUS		0xA88
#define AUXTX_TRAN_CTRL		0xA8C
#define AUXTX_REQ_CMD		0xA90
#define AUXTX_REQ_ADDR_M	0xA94
#define AUXTX_REQ_ADDR_L	0xA98
#define AUXTX_REQ_LEN		0xA9C
#define AUXTX_REQ_DATA		0xAA0
#define AUX_REPLY_CMD		0xAA4
#define AUX_REPLY_DATA		0xAA8
#define AUX_FIFO_CTRL		0xAAC
#define AUX_TX_FIFO_STATUS	0xAB0
#define AUX_FIFO_RD_PTR		0xAB4
#define AUX_FIFO_WR_PTR		0xAB8
#define AUX_RETRY_1		0xABC
#define AUX_RETRY_2		0xAC0
#define AUX_IRQ_EVENT		0xAC4
#define AUX_IRQ_EN		0xAC8
#define AUX_DIG_PHY2		0xBC0
#define AUX_DIG_PHY3		0xBC4
#define AUX_DIG_PHY4		0xBC8
#define AUX_DIG_PHY5		0xBCC
#define AUX_DIG_PHY6		0xBD0
#define AUX_DIG_PHY7		0xBD4
#define AUX_DIG_PHY8		0xBD8
#define AUX_DIG_PHY9		0xBDC
#define AUX_DEBUG		0xBE0

/* AUX_TX_CTRL */
#define AUX_EN		BIT0

/* AUXTX_TRAN_CTRL */
#define TX_START	BIT0

/* AUX_IRQ_EN */
#define TIMEOUT		BIT0
#define RETRY		BIT1
#define NACK		BIT2
#define READFAIL	BIT3
#define RXERROR		BIT4
#define AUXDONE		BIT5
#define ALPM		BIT6
#define AUX_ALL_IRQ	(TIMEOUT | RETRY | NACK | READFAIL | \
			RXERROR | AUXDONE | ALPM)

/* AUX_FIFO_CTRL */
#define NA_FIFO_RST	BIT0
#define I2C_FIFO_RST	BIT1

#endif /* _RTK_DPTX_REG_H */
