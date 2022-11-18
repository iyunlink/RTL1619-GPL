/*
 *  Copyright (C) 2013 Realtek Semiconductors, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DW_RTK_MMC_CQE_H
#define __DW_RTK_MMC_CQE_H

#define SDMMC_CP                         0x41c
#define SDMMC_OTHER1                     0x420
#define SDMMC_DUMMY_SYS                  0x42c
#define SDMMC_AHB                        0x430
#define SDMMC_CKGEN_CTL                  0x478
#define SDMMC_DQS_CTRL1                  0x498
#define SDMMC_IP_DESC0                   0x4a0
#define SDMMC_IP_DESC1                   0x4a4
#define SDMMC_IP_DESC2                   0x4a8
#define SDMMC_IP_DESC3                   0x4ac
#define SDMMC_DQ_CTRL_SET                0x50c
#define SDMMC_WDQ_CTRL0                  0x510
#define SDMMC_WDQ_CTRL1                  0x514
#define SDMMC_WDQ_CTRL2                  0x518
#define SDMMC_WDQ_CTRL3                  0x51c
#define SDMMC_WDQ_CTRL4                  0x520
#define SDMMC_WDQ_CTRL5                  0x524
#define SDMMC_WDQ_CTRL6                  0x528
#define SDMMC_WDQ_CTRL7                  0x52c
#define SDMMC_RDQ_CTRL0                  0x530
#define SDMMC_RDQ_CTRL1                  0x534
#define SDMMC_RDQ_CTRL2                  0x538
#define SDMMC_RDQ_CTRL3                  0x53c
#define SDMMC_RDQ_CTRL4                  0x540
#define SDMMC_RDQ_CTRL5                  0x544
#define SDMMC_RDQ_CTRL6                  0x548
#define SDMMC_RDQ_CTRL7                  0x54c
#define SDMMC_CMD_CTRL_SET               0x550
#define SDMMC_WCMD_CTRL                  0x554
#define SDMMC_RCMD_CTRL                  0x558
#define SDMMC_PLL_STATUS                 0x55c

#define SDMMC_NAND_DMA_SEL               0x54
#define SDMMC_SRAM_DMA_SEL               BIT(0)

#define SDMMC_CLK_O_ICG_EN               BIT(3)
#define SDMMC_CARD_STOP_ENABLE           BIT(23)
#define SDMMC_STARK_CARD_STOP_ENABLE     BIT(11)
#define SDMMC_TOP_RST_N_FIFO             BIT(3)
#define SDMMC_L4_GATED_DIS1              BIT(2)
#define SDMMC_L4_GATED_DIS               BIT(0)

#define SDMMC_FW_SET                     BIT(7)
#define SDMMC_FW_SET_CMD_W               BIT(0)
#define SDMMC_FW_SET_RW                  0xff00

#define SDMMC_CRC_CLK_CHANGE_SHIFT       (16)
#define SDMMC_CLK4M                      ((BIT(0)|BIT(1)|BIT(2))<<EMMC_CRC_CLK_CHANGE_SHIFT)
#define SDMMC_CRC_CLK_DIV                (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7))
#define SDMMC_CRC_CLK_DIV_MASK           (~SDMMC_CRC_CLK_DIV)
#define SDMMC_CRC_CLK_DIV_EN             BIT(8)
#define SDMMC_CLK_INV_DIV_SEL            BIT(9)
#define SDMMC_SSC_CLK_DIV_SEL            (BIT(19)|BIT(20))

#define SDMMC_CLK_DIV512                 (0x200)
#define SDMMC_CLK_DIV4                   (0x4)
#define SDMMC_CLK_DIV1                   (0x1)

struct backupRegs {
	u32                     sdmasa_r;       	/*0x000*/
	u16                     blocksize_r;    	/*0x004*/
	u16                     blockcount_r;   	/*0x006*/
	u16                     xfer_mode_r;    	/*0x00c*/
	u8                      host_ctrl1_r;   	/*0x028*/
	u8                      pwr_ctrl_r;     	/*0x029*/
	u8                      bgap_ctrl_r;    	/*0x02a*/
	u16                     clk_ctrl_r;     	/*0x02c*/
	u8                      tout_ctrl_r;    	/*0x02e*/

	u16                     normal_int_stat_en_r;   /*0x034*/
	u16                     error_int_stat_en_r;    /*0x036*/
	u16                     normal_int_signal_en_r; /*0x038*/
	u16                     error_int_signal_en_r;  /*0x03a*/
	u16                     auto_cmd_stat_r;        /*0x03c*/
	u16                     host_ctrl2_r;           /*0x03e*/
	u32                     adma_sa_low_r;          /*0x058*/
	u8                      mshc_ctrl_r;            /*0x208*/
	u8                      ctrl_r;                 /*0x22c*/
	u32                     other1;                 /*0x420*/
	u32                     dummy_sys;              /*0x42c*/
	u32                     dqs_ctrl1;              /*0x498*/
	u32                     wcmd_ctrl;              /*0x554*/

	u32                     rdq_ctrl0;              /*0x530*/
	u32                     rdq_ctrl1;              /*0x534*/
	u32                     rdq_ctrl2;              /*0x538*/
	u32                     rdq_ctrl3;              /*0x53c*/
	u32                     rdq_ctrl4;              /*0x540*/
	u32                     rdq_ctrl5;              /*0x544*/
	u32                     rdq_ctrl6;              /*0x548*/
	u32                     rdq_ctrl7;              /*0x54c*/
	u32                     dq_ctrl_set;            /*0x50c*/
	u32                     ahb;
	u32                     ckgen_ctl;
};

struct dw_mci_rtkemmc_host {
	struct pinctrl          *pinctrl;
	struct pinctrl_state    *pins_default;
	struct pinctrl_state    *pins_sdr50;
	struct pinctrl_state    *pins_ddr50;
	struct pinctrl_state    *pins_hs200;
	struct pinctrl_state    *pins_hs400;
	struct pinctrl_state    *pins_tune0;
	struct pinctrl_state    *pins_tune1;
	struct pinctrl_state    *pins_tune2;
	struct pinctrl_state    *pins_tune3;
	struct pinctrl_state    *pins_tune4;
	struct regmap		*m2tmx;
	struct clk              *vp0;
	struct clk              *vp1;
	struct backupRegs       gRegTbl;
	int                     emmc_mode;
	unsigned int		rdq_ctrl;
	unsigned int		is_cqe;
	unsigned int		protect_start;
	unsigned int		protect_unit;
};
#endif
