/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Realtek DHC SoC Secure Digital Host Controller Interface.
 *
 * Copyright (c) 2022 Realtek Semiconductor Corp.
 */
#ifndef _DRIVERS_MMC_SDHCI_OF_RTKSTB_H
#define _DRIVERS_MMC_SDHCI_OF_RTKSTB_H

#define MAX_PHASE		32
#define TUNING_CNT		3
#define MINIMUM_CONTINUE_LENGTH	16
#define PAD_PWR_CHK_CNT		3
/* Controller clock has jitter when SSC enable */
#define RTKQUIRK_SSC_CLK_JITTER	BIT(0)
/* CRT register */
#define SYS_PLL_SD1		0x1E0
#define  PHRT0			BIT(1)
#define	 PHSEL0_MASK		GENMASK(7, 3)
#define  PHSEL0_SHIFT		3
#define	 PHSEL1_MASK		GENMASK(12, 8)
#define  PHSEL1_SHIFT		8
#define	 REG_SEL3318_MASK	GENMASK(14, 13)
#define	 REG_SEL3318_3V3	0x1
#define	 REG_SEL3318_0V		0x2
#define	 REG_SEL3318_1V8	0x3
#define SYS_PLL_SD2		0x1E4
#define  REG_TUNE11		GENMASK(2, 1)
#define  REG_TUNE11_1V9		0x2
#define  SSCPLL_CS1		GENMASK(4, 3)
#define  SSCPLL_CS1_INIT_VALUE	0x1
#define  SSCPLL_ICP		GENMASK(9, 5)
#define  SSCPLL_ICP_CURRENT	0x01
#define  SSCPLL_RS		GENMASK(12, 10)
#define  SSCPLL_RS_13K		0x5
#define  SSC_DEPTH		GENMASK(15, 13)
#define  SSC_DEPTH_1_N		0x3
#define  SSC_8X_EN		BIT(16)
#define  SSC_DIV_EXT_F		GENMASK(25, 18)
#define  SSC_DIV_EXT_F_INIT_VAL	0xE3
#define  EN_CPNEW		BIT(26)
#define SYS_PLL_SD3		0x1E8
#define  SSC_TBASE		GENMASK(7, 0)
#define  SSC_TBASE_INIT_VALUE	0x88
#define  SSC_STEP_IN		GENMASK(14, 8)
#define  SSC_STEP_IN_INIT_VALUE	0x43
#define  SSC_DIV_N		GENMASK(25, 16)
#define  SSC_DIV_N_50M		0x28
#define  SSC_DIV_N_100M		0x56
#define  SSC_DIV_N_200M		0xAF
#define  SSC_DIV_N_208M		0xB6
#define SYS_PLL_SD4		0x1EC
#define  SSC_RSTB		BIT(0)
#define  SSC_PLL_RSTB		BIT(1)
#define  SSC_PLL_POW		BIT(2)
/* ISO register */
#define ISO_CTRL_1		0x70C4
#define  SD3_SSCLDO_EN		BIT(2)
#define  SD3_BIAS_EN		BIT(3)
#define ISO_GPDIR_1		0x7118
#define  GPDIR_1_53		BIT(21)
#define ISO_GPDATO_1		0x711C
#define  GPDATO_1_53		BIT(21)
/* ISO testmux register */
#define ISO_MUXPAD6		0x4E018
#define  GPIO_32_PADMUX		GENMASK(3, 0)
#define  MUX_TO_SD_CMD		0x1
#define  GPIO_33_PADMUX		GENMASK(7, 4)
#define  MUX_TO_SD_CLK		0x1
#define  GPIO_34_PADMUX		GENMASK(11, 8)
#define  MUX_TO_SD_WP		0x1
#define  GPIO_35_PADMUX		GENMASK(15, 12)
#define  MUX_TO_SD_CD		0x1
#define  HIF_DATA_PADMUX	GENMASK(19, 16)
#define  MUX_TO_SD_DAT0		0x1
#define  HIF_EN_PADMUX		GENMASK(23, 20)
#define  MUX_TO_SD_DAT1		0x1
#define  HIF_RDY_PADMUX		GENMASK(27, 24)
#define  MUX_TO_SD_DAT2		0x1
#define  HIF_CLK_PADMUX		GENMASK(31, 28)
#define  MUX_TO_SD_DAT3		0x1
#define ISO_PFUNC18		0x4E074
#define  HIF_EN_PAD_N_MASK	GENMASK(8, 6)
#define  HIF_EN_PAD_N2E		BIT(6)
#define  HIF_EN_PAD_N3E		BIT(7)
#define  HIF_EN_PAD_N4E		BIT(8)
#define  HIF_EN_PAD_P_MASK	GENMASK(11, 9)
#define  HIF_EN_PAD_P2E		BIT(9)
#define  HIF_EN_PAD_P3E		BIT(10)
#define  HIF_EN_PAD_P4E		BIT(11)
#define  HIF_EN_H3L1		BIT(12)
#define  HIF_DATA_PAD_N_MASK	GENMASK(21, 19)
#define  HIF_DATA_PAD_N2E	BIT(19)
#define  HIF_DATA_PAD_N3E	BIT(20)
#define  HIF_DATA_PAD_N4E	BIT(21)
#define  HIF_DATA_PAD_P_MASK	GENMASK(24, 22)
#define  HIF_DATA_PAD_P2E	BIT(22)
#define  HIF_DATA_PAD_P3E	BIT(23)
#define  HIF_DATA_PAD_P4E	BIT(24)
#define  HIF_DATA_H3L1		BIT(25)
#define ISO_PFUNC19		0x4E078
#define  GPIO33_PUD_EN		BIT(0)
#define  GPIO33_PUD_SEL		BIT(1)
#define  GPIO33_PAD_N_MASK	GENMASK(8, 6)
#define  GPIO33_PAD_N2E		BIT(6)
#define  GPIO33_PAD_N3E		BIT(7)
#define  GPIO33_PAD_N4E		BIT(8)
#define  GPIO33_PAD_P_MASK	GENMASK(11, 9)
#define  GPIO33_PAD_P2E		BIT(9)
#define  GPIO33_PAD_P3E		BIT(10)
#define  GPIO33_PAD_P4E		BIT(11)
#define  GPIO33_H3L1		BIT(12)
#define  GPIO32_PAD_N_MASK	GENMASK(21, 19)
#define  GPIO32_PAD_N2E		BIT(19)
#define  GPIO32_PAD_N3E		BIT(20)
#define  GPIO32_PAD_N4E		BIT(21)
#define  GPIO32_PAD_P_MASK	GENMASK(24, 22)
#define  GPIO32_PAD_P2E		BIT(22)
#define  GPIO32_PAD_P3E		BIT(23)
#define  GPIO32_PAD_P4E		BIT(24)
#define  GPIO32_H3L1		BIT(25)
#define ISO_PFUNC20		0x4E07C
#define  HIF_CLK_PAD_N_MASK	GENMASK(8, 6)
#define  HIF_CLK_PAD_N2E	BIT(6)
#define  HIF_CLK_PAD_N3E	BIT(7)
#define  HIF_CLK_PAD_N4E	BIT(8)
#define  HIF_CLK_PAD_P_MASK	GENMASK(11, 9)
#define  HIF_CLK_PAD_P2E	BIT(9)
#define  HIF_CLK_PAD_P3E	BIT(10)
#define  HIF_CLK_PAD_P4E	BIT(11)
#define  HIF_CLK_H3L1		BIT(12)
#define  HIF_RDY_PAD_N_MASK	GENMASK(21, 19)
#define  HIF_RDY_PAD_N2E	BIT(19)
#define  HIF_RDY_PAD_N3E	BIT(20)
#define  HIF_RDY_PAD_N4E	BIT(21)
#define  HIF_RDY_PAD_P_MASK	GENMASK(24, 22)
#define  HIF_RDY_PAD_P2E	BIT(22)
#define  HIF_RDY_PAD_P3E	BIT(23)
#define  HIF_RDY_PAD_P4E	BIT(24)
#define  HIF_RDY_H3L1		BIT(25)
#define ISO_DBG_STATUS		0x4E190
#define  SDIO0_H3L1_STATUS_HI	BIT(1)
#define  SDIO0_H3L1_DETECT_EN	BIT(4)
/* wrapper control register */
#define SDHCI_RTK_CTL		0x120A10
#define  SUSPEND_N		BIT(0)
#define  L4_GATE_DISABLE	BIT(1)
#define  DBUS_ENDIAN_SEL	BIT(2)
#define  L4_GATE_DISABLE_IP	BIT(3)
#define  IP_ENDIAN_SEL		BIT(4)
#define SDHCI_RTK_ISREN		0x120A34
#define  ISREN_WRITE_DATA	BIT(0)
#define  INT1_EN		BIT(1)
#define  INT3_EN		BIT(3)
#define  INT4_EN		BIT(4)

#endif /* _DRIVERS_MMC_SDHCI_OF_RTKSTB_H */
