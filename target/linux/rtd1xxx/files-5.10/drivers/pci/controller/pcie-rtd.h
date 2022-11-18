/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Realtek PCIe host controller driver
 *
 * Copyright (c) 2017 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#ifndef RTK_PCIE_13xx_H_
#define RTK_PCIE_13xx_H_

#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/msi.h>
#include <linux/gpio/consumer.h>

#include "../pci.h"


#define PCIE_IO_2K_MASK  0xFFFFF800
#define PCIE_IO_4K_MASK  0xFFFFF000
#define PCIE_IO_64K_MASK 0xFFFF0000


#define RTK_MAX_MSI_IRQS			256
#define RTK_MAX_MSI_IRQS_PER_CTRL		32
#define RTK_MAX_MSI_CTRLS			(RTK_MAX_MSI_IRQS / RTK_MAX_MSI_IRQS_PER_CTRL)
#define RTK_MSI_REG_CTRL_BLOCK_SIZE		12
#define RTK_MSI_DEF_NUM_VECTORS		32

#define RTK_MSIX_DEF_NUM_VECTORS		64
#define RTK_MAX_MSIX_IRQS_PER_CTRL 32
#define MAX_RTK_MSIX_CTRLS	(MAX_RTK_MSI_IRQS / 32)



struct rtd_pcie_port {
	struct rtd_pcie_ops *ops;
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *ctrl_base;
	void __iomem *cfg_base;
	struct regmap *main2_misc_base;
	struct regmap *pinmux_base;
	struct clk *pcie_clk;
	struct reset_control *rst;
	struct reset_control *rst_stitch;
	struct reset_control *rst_core;
	struct reset_control *rst_power;
	struct reset_control *rst_nonstitch;
	struct reset_control *rst_phy;
	struct reset_control *rst_phy_mdio;
	struct reset_control *rst_sgmii_mdio;
	struct phy *pcie_phy;
	int speed_mode;
	int debug_mode;
	int workaround;
	struct gpio_desc *perst_gpio;
	int	msi_irq;
	struct irq_domain *irq_domain;
	struct irq_domain *msi_domain;
	dma_addr_t msi_data;
	u32 *msi_data_virt;
	struct page *msi_page;
	u32 msi_max_vector;
	u32 irq_mask[RTK_MAX_MSI_CTRLS];
	struct irq_chip *msi_irq_chip;
	raw_spinlock_t lock;
	DECLARE_BITMAP(irq_bitmap, RTK_MAX_MSI_IRQS);
	struct msi_controller msi_chip;
};

struct rtd_pcie_ops {
	char name[10];
	const struct rtd_msi_ops *msi_ops;
	int (*get_resource)(struct rtd_pcie_port *pp);
	int (*init)(struct rtd_pcie_port *pp);
	int (*deinit)(struct rtd_pcie_port *pp);
	int (*hwinit)(struct rtd_pcie_port *pp);
};

struct rtd_msi_ops {
	int (*msi_init)(struct rtd_pcie_port *pp);
	void (*msi_host_init)(struct rtd_pcie_port *pp);
	irqreturn_t (*irq_handle)(struct rtd_pcie_port *pp);
};

/*Unsupported Request*/
#define UR 0x1
/*Configuration Request Retry Status*/
#define CRS 0x2
/*Completer Abort*/
#define CA 0x4

#define ISO_POWERCUT_ETN 0x5c
#define PCIE0_USB_SEL_OFFSET BIT(6)

#define MISC_PHY_CTRL 0x50
#define PCIE1_SATA_SEL_OFFSET BIT(8)
#define PCIE2_SATA_SEL_OFFSET BIT(9)

/* SCPU Wrapper PCIe MMIO REG*/
#define PCIE1_START 0x630
#define PCIE1_END 0x634
#define PCIE1_CTRL 0x638
#define PCIE1_STAT 0x63C
#define PCIE2_START 0x640
#define PCIE2_END 0x644
#define PCIE2_CTRL 0x648
#define PCIE2_STAT 0x64C
#define INTERFACE_EN 0x124
#define SC_CRT_CTRL 0x100
#define SC_ACP_CTRL 0x800
#define SC_ACP_CRT_CTRL 0x030

#define PCIE_MSIX_TRAN 0xD68
#define PCIE_MSIX_0 0xD60
#define PCIE_MSIX_1 0xD64
#define PCIE_MSIX_CTRL 0xD6C



/* PCIE_MAC_REG */
#define LINK_CONTROL2_LINK_STATUS2_REG 0xa0
#define PORT_LINK_CTRL_OFF 0x710
#define MEM_LIMIT_MEM_BASE_REG 0x20
#define PREF_MEM_LIMIT_PREF_MEM_BASE_REG 0x24
#define TYPE1_STATUS_COMMAND_REG 0x4
#define PCIE_MSI_ADDR_LO 0x820
#define PCIE_MSI_ADDR_HI 0x824
#define PCIE_MSI_INTR0_ENABLE 0x828
#define PCIE_MSI_INTR0_MASK 0x82C
#define PCIE_MSI_INTR0_STATUS 0x830


/*
 * PCI-DVR Bridge
 */
#define PCIE_SYS_CTR 0x00000C00U
#define PCIE_BASE_0 0xCFC
#define PCIE_MASK_0 0xD00
#define PCIE_TRANS_0 0xD04
#define PCIE_DIR_EN 0xC78
#define PCIE_RCPL_ST 0xC7C
#define PCIE_MDIO_CTR 0xC1C
#define PCIE_SCTCH 0xCBC
#define PCIE_SERVICE_REGION 0xD30
#define PCIE_MSI_TRAN 0xCD0
#define PCIE_INT_CTR 0xC04
#define PCIE_MSI_DATA 0xCD4
#define TRANS_EN (1 << 20)
#define PHY_MMIO_TYPE (1 << 19)
#define PHY_MDIO_OE (1 << 18)
#define PHY_MDIO_RSTN (1 << 17)
#define APP_INIT_RST (1 << 16)
#define LOOPBACK_EN (1 << 9)
#define DIR_REQ_INFO_EN (1 << 8)
#define RX_LANE_REVERSAL_EN (1 << 7)
#define TX_LANE_REVERSAL_EN (1 << 6)
#define INDOR_CFG_EN (1 << 5)
#define DIR_CFG_EN (1 << 4)
#define RCV_ADDR0_EN (1 << 3)
#define RCV_ADDR1_EN (1 << 2)
#define APP_LTSSM_EN (1 << 1)
#define RCV_TRANS_EN (1)
#define PCIE_DDR_START 0xD70
#define PCIE_DDR_END 0xD74
#define PCI_BASE_2 0xD5C
#define PCIE_ACP_CTRL 0xD40

#define DVR_GNR_INT 0x00000C08U
#define S_INTP_PCIE_LEGACY_MSI_INT 14
#define S_INTP_PM_TO_ACK_INT 13
#define S_INTP_CFG_SYS_ERR_RC_INT 12
#define S_INTP_PCIE_LEGACY_INT 11
#define S_INTP_CFG_RSDM_VENDOR_MSG_INT 10
#define S_INTP_CFG_PME_MSI 9
#define S_INTP_CFG_PME_INT 8
#define S_INTP_CFG_AER_RC_MSI 7
#define S_INTP_CFG_AER_RC_ERR 6
#define S_INTP_RTGT 5 /* Slave Receiver Interrupt */
#define S_INTP_RCPL 4 /* Master Receiver Interrupt */
#define S_INTP_DIR_CFG 3 /* Direct CFG Interrupt */
#define S_INTP_DIR_MIO 2 /* Direct MIO Interrupt */
#define S_INTP_CFG 1 /* Indirect CFG Interrupt */
#define S_INTP_MIO 0 /* Indirect MIO Interrupt */
#define INTP_PCIE_LEGACY_MSI_INT (1 << INTP_PCIE_LEGACY_MSI_INT)
#define INTP_PM_TO_ACK_INT (1 << INTP_PM_TO_ACK_INT)
#define INTP_CFG_SYS_ERR_RC_INT (1 << INTP_CFG_SYS_ERR_RC_INT)
#define INTP_PCIE_LEGACY_INT (1 << INTP_PCIE_LEGACY_INT)
#define INTP_CFG_RSDM_VENDOR_MSG_INT (1 << INTP_CFG_RSDM_VENDOR_MSG_INT)
#define INTP_CFG_PME_MSI (1 << INTP_CFG_PME_MSI)
#define INTP_CFG_PME_INT (1 << INTP_CFG_PME_INT)
#define INTP_CFG_AER_RC_MSI (1 << INTP_CFG_AER_RC_MSI)
#define INTP_CFG_AER_RC_ERR (1 << INTP_CFG_AER_RC_ERR)
#define INTP_RTGT (1 << INTP_RTGT)
#define INTP_RCPL (1 << INTP_RCPL)
#define INTP_DIR_CFG (1 << INTP_DIR_CFG)
#define INTP_DIR_MIO (1 << INTP_DIR_MIO)
#define INTP_CFG (1 << INTP_CFG)
#define INTP_MIO (1 << INTP_MIO)

#define DVR_PCIE_INT 0x00000C0CU
#define PCIE_INTP_INTD (0x1U << 3)
#define PCIE_INTP_INTC (0x1U << 2)
#define PCIE_INTP_INTB (0x1U << 1)
#define PCIE_INTP_INTA (0x1U)

#define PCIE_DBI_CTR 0x00000C10U
#define DBI_IO_ACCESS (0x1U << 9)
#define DBI_ROM_ACCESS (0x1U << 8)
#define DBI_BAR_NUM(x) ((x & 0x7) << 5)
#define DBI_FUNC_NUM(x) ((x & 0x7) << 2)
#define DBI_SC2_ACCESS (0x1U << 1)
#define DBI_CMD_ACCESS (0x1U)

#define PCIE_INDIR_CTR 0x00000C14
#define PCIE_DIR_CTR 0x00000018U
#define REQ_INFO_ALIGN (0x1U << 13)
#define REQ_INFO_ATTR(x) ((x & 0x3) << 11)
#define REQ_INFO_EP (0x1U << 10)
#define REQ_INFO_TC(x) ((x & 0x7) << 7)
#define REQ_INFO_TYPE(x) ((x & 0x1F) << 2)
#define REQ_INFO_FMT(x) (x & 0x3)

#define MDIO_DATA(x) ((x & 0xFFFF) << 16) /* MDIO Data */
#define MDIO_PHY_ADDR(x) ((x & 0x7) << 13) /* MDIO PHY Address */
#define MDIO_REG_ADDR(x) ((x & 0x1F) << 8) /* MDIO Register Address */
#define MDIO_BUSY (0x1U << 7)
#define MDIO_ST(x) ((x & 0x03) << 5) /* MDIO Host Control State Monitor */
#define MDIO_RDY (0x1U << 4) /* MDIO PREAMBLE Signal Monitor */
#define MDIO_RATE(x) ((x & 0x3) << 2) /* MDIO Clock Rate */
#define CLK_SYS_1_32(x) 0 /* MDIO Clock Rate = 1/32 */
#define CLK_SYS_1_16(x) 1 /* MDIO Clock Rate = 1/16 */
#define CLK_SYS_1_8(x) 2 /* MDIO Clock Rate = 1/8 */
#define CLK_SYS_1_4(x) 3 /* MDIO Clock Rate = 1/4 */
#define MDIO_SRST (0x1U << 1)
#define MDIO_RDWR(x) (x & 0x1) /* MDIO Read/Write : 0 read, 1 write */

/*
 * Address Translation
 * PCI-E Bridge provides 2 set of control registers
 * for in-bound address translation
 */
#define PCIE_BASE0 0x00000C20U
#define PCIE_BASE1 0x00000C24U
#define PCIE_MASK0 0x00000C28U
#define PCIE_MASK1 0x00000C2CU
#define PCIE_TRAN0 0x00000C30U
#define PCIE_TRAN1 0x00000C34U

/*
 * Configuration Access
 * the following registers are used to access configuration
 * space
 */
#define PCIE_CFG_CT 0x00000C38U
#define GO_CT (0x1U)

#define PCIE_CFG_EN 0x00000C3CU
#define BUS_NUM(x) ((x & 0xFF) << 16) /* Bus number */
#define DEV_NUM(x) ((x & 0xF) << 11) /* Device number */
#define FUN_NUM(x) ((x & 0x7) << 8) /* Function number */
#define BYTE_CNT(x) ((x & 0xF) << 4) /* Byte enable bits */
#define ERROR_EN(x) ((x & 0x1) << 2) /* Enable error timeout counter */
#define BYTE_EN (0x1 << 1) /* Byte enable bits enable */
#define WRRD_EN(x) (x & 0x01)
#define WRITE_CFG WRRD_EN(1)
#define READ_CFG WRRD_EN(0)

#define PCIE_CFG_ST 0x00000C40U
#define CFG_ST_ERROR (0x1U << 1)
#define CFG_ST_DONE 0x1U

#define PCIE_CFG_ADDR 0x00000C44U
#define PCIE_CFG_WDATA 0x00000C48U
#define PCIE_CFG_RDATA 0x00000C4CU

/*
 * Memory / IO Access
 * the following registers are used to access configuration
 * space
 */
#define PCIE_MIO_CT 0x00000C50U
#define PCIE_MIO_EN 0x00000C54U

#define TIMEOUT_CNT_VAL(x) ((x & 0xFFFFFFF) << 8)

#define PCIE_MIO_ST 0x00000C58U
#define PCIE_MIO_ADDR 0x00000C5CU
#define PCIE_MIO_WDATA 0x00000C60U
#define PCIE_MIO_RDATA 0x00000C64U

/*
 * MISC
 */
#define PCIE_CTR 0x00000C68U
#define PCIE_PWR_CTR 0x00000C6CU
#define PCIE_DBG 0x00000C70U
#define PCIE_DIR_ST 0x00000C74U /* Status for Direct Access */
#define CFG_RERROR_ST (0x1U << 1)
#define MIO_RERROR_ST (0x1U)
#define TIMEOUT_EN (0x1U)

#define PCIE_MAC_ST 0x00000CB4
#define RDLH_LINK_UP (0x1U << 14)
#define PM_XTLH_BLOCK_TLP (0x1U << 13)
#define CFG_BUS_MASTER_EN (0x1U << 12)
#define CFG_PM_NO_SOFT_RST (0x1U << 11)
#define XMLH_LINK_UP (0x1U << 10)
#define LINK_REQ_RST_NOT (0x1U << 9)
#define XMLH_LTSSM_STATE(x) ((x & 0xF) << 4)
#define PM_CURNT_STATE(x) ((x & 0x7) << 1)
#define CFG_CLK_REQ_N (0x1U)

/*
 * PCI-E RC registers
 */
#define PCI_CFG_REG 0x00000000U /* PCI compatible registers */
#define PCIE_DEV_REG 0x00000040U /* PCI-E Device Configuration Space */
#define PCIE_EXT_REG 0x00000100U /* PCI-E Extend Configuration Space */
#define PCIE_DVR_REG 0x00000C00U /* DVR space register */

#define CFG(addr) (PCI_CFG_REG + (addr))
#define BAR1 0x10
#define BAR2 0x14
#define BAR3 0x18
#define BAR4 0x1C

#define CFG_ST_DETEC_PAR_ERROR (1 << 31)
#define CFG_ST_SIGNAL_SYS_ERROR (1 << 30)
#define CFG_ST_REC_MASTER_ABORT (1 << 29)
#define CFG_ST_REC_TARGET_ABORT (1 << 28)
#define CFG_ST_SIG_TAR_ABORT (1 << 27)

#define PCIE_CONNECT_TIMEOUT 60
#define ADDR_TO_DEVICE_NO(addr) ((addr >> 19) & 0x1F)
#endif

