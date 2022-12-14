/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __DT_BINDINGS_RTK_RESET_RTD1319D_H
#define __DT_BINDINGS_RTK_RESET_RTD1319D_H

#define RTD1319D_RSTN_REG_ID_SOFT_RESET1  0x0000
#define RTD1319D_RSTN_REG_ID_SOFT_RESET2  0x0100
#define RTD1319D_RSTN_REG_ID_SOFT_RESET3  0x0200
#define RTD1319D_RSTN_REG_ID_SOFT_RESET4  0x0300
#define RTD1319D_RSTN_REG_ID_SOFT_RESET7  0x0400
#define RTD1319D_RSTN_REG_ID_SOFT_RESET9  0x0500
#define RTD1319D_RSTN_REG_ID_DUMMY0       0x0600
#define RTD1319D_RSTN_REG_ID_DUMMY1       0x0700
#define RTD1319D_RSTN_REG_ID_DUMMY4       0x0800
#define RTD1319D_RSTN_REG_ID_SOFT_RESET_BIST  0x900

#define RTD1319D_CRT_RSTN_MISC              (RTD1319D_RSTN_REG_ID_SOFT_RESET1|0x00)
#define RTD1319D_CRT_RSTN_DIP               (RTD1319D_RSTN_REG_ID_SOFT_RESET1|0x02)
#define RTD1319D_CRT_RSTN_GSPI              (RTD1319D_RSTN_REG_ID_SOFT_RESET1|0x04)
#define RTD1319D_CRT_RSTN_SDS               (RTD1319D_RSTN_REG_ID_SOFT_RESET1|0x06)
#define RTD1319D_CRT_RSTN_SDS_REG           (RTD1319D_RSTN_REG_ID_SOFT_RESET1|0x08)
#define RTD1319D_CRT_RSTN_SDS_PHY           (RTD1319D_RSTN_REG_ID_SOFT_RESET1|0x0a)
#define RTD1319D_CRT_RSTN_DC_PHY            (RTD1319D_RSTN_REG_ID_SOFT_RESET1|0x16)
#define RTD1319D_CRT_RSTN_DCPHY_CRT         (RTD1319D_RSTN_REG_ID_SOFT_RESET1|0x18)
#define RTD1319D_CRT_RSTN_LSADC             (RTD1319D_RSTN_REG_ID_SOFT_RESET1|0x1a)
#define RTD1319D_CRT_RSTN_SE                (RTD1319D_RSTN_REG_ID_SOFT_RESET1|0x1c)

#define RTD1319D_CRT_RSTN_JPEG              (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x00)
#define RTD1319D_CRT_RSTN_SD                (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x02)
#define RTD1319D_CRT_RSTN_SDIO              (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x06)
#define RTD1319D_CRT_RSTN_PCR_CNT           (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x08)
#define RTD1319D_CRT_RSTN_PCIE0_STITCH      (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x0a)
#define RTD1319D_CRT_RSTN_PCIE0_PHY         (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x0c)
#define RTD1319D_CRT_RSTN_PCIE0             (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x0e)
#define RTD1319D_CRT_RSTN_PCIE0_CORE        (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x10)
#define RTD1319D_CRT_RSTN_PCIE0_POWER       (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x12)
#define RTD1319D_CRT_RSTN_PCIE0_NONSTICH    (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x14)
#define RTD1319D_CRT_RSTN_PCIE0_PHY_MDIO    (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x16)
#define RTD1319D_CRT_RSTN_PCIE0_SGMII_MDIO  (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x18)
#define RTD1319D_CRT_RSTN_UR2               (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x1a)
#define RTD1319D_CRT_RSTN_UR1               (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x1c)
#define RTD1319D_CRT_RSTN_MISC_SC0          (RTD1319D_RSTN_REG_ID_SOFT_RESET2|0x1e)

#define RTD1319D_CRT_RSTN_AE                (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x00)
#define RTD1319D_CRT_RSTN_CABLERX           (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x02)
#define RTD1319D_CRT_RSTN_MD                (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x04)
#define RTD1319D_CRT_RSTN_MISC_SC1          (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x0a)
#define RTD1319D_CRT_RSTN_I2C_3             (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x0c)
#define RTD1319D_CRT_RSTN_FAN               (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x0e)
#define RTD1319D_CRT_RSTN_TVE               (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x10)
#define RTD1319D_CRT_RSTN_AIO               (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x12)
#define RTD1319D_CRT_RSTN_VO                (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x14)
#define RTD1319D_CRT_RSTN_MIPI              (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x16)
#define RTD1319D_CRT_RSTN_HDMIRX            (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x18)
#define RTD1319D_CRT_RSTN_HDMIRX_WRAP       (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x1a)
#define RTD1319D_CRT_RSTN_HDMI              (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x1c)
#define RTD1319D_CRT_RSTN_DISP              (RTD1319D_RSTN_REG_ID_SOFT_RESET3|0x1e)

#define RTD1319D_CRT_RSTN_SATA_PHY_POW1     (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x00)
#define RTD1319D_CRT_RSTN_SATA_PHY_POW0     (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x02)
#define RTD1319D_CRT_RSTN_SATA_MDIO1        (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x04)
#define RTD1319D_CRT_RSTN_SATA_MDIO0        (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x06)
#define RTD1319D_CRT_RSTN_SATA_WRAP         (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x08)
#define RTD1319D_CRT_RSTN_SATA_MAC_P1       (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x0a)
#define RTD1319D_CRT_RSTN_SATA_MAC_P0       (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x0c)
#define RTD1319D_CRT_RSTN_SATA_MAC_COM      (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x0e)
#define RTD1319D_CRT_RSTN_PCIE1_STITCH      (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x10)
#define RTD1319D_CRT_RSTN_PCIE1_PHY         (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x12)
#define RTD1319D_CRT_RSTN_PCIE1             (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x14)
#define RTD1319D_CRT_RSTN_PCIE1_CORE        (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x16)
#define RTD1319D_CRT_RSTN_PCIE1_POWER       (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x18)
#define RTD1319D_CRT_RSTN_PCIE1_NONSTICH    (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x1a)
#define RTD1319D_CRT_RSTN_PCIE1_PHY_MDIO    (RTD1319D_RSTN_REG_ID_SOFT_RESET4|0x1c)

#define RTD1319D_CRT_RSTN_CABLERX_Q         (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x00)
#define RTD1319D_CRT_RSTN_I2C_4             (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x02)
#define RTD1319D_CRT_RSTN_I2C_5             (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x04)
#define RTD1319D_CRT_RSTN_TSIO              (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x06)
#define RTD1319D_CRT_RSTN_VE3_BIST          (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x08)
#define RTD1319D_CRT_RSTN_EDP               (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x0a)
#define RTD1319D_CRT_RSTN_VE1_MMU           (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x0c)
#define RTD1319D_CRT_RSTN_VE1_MMU_FUNC      (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x0e)
#define RTD1319D_CRT_RSTN_HSE_MMU           (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x10)
#define RTD1319D_CRT_RSTN_HSE_MMU_FUNC      (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x12)
#define RTD1319D_CRT_RSTN_MDLM2M            (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x14)
#define RTD1319D_CRT_RSTN_ISO_GSPI          (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x16)
#define RTD1319D_CRT_RSTN_SPI2EMMC          (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x1a)
#define RTD1319D_CRT_RSTN_EARC              (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x1c)
#define RTD1319D_CRT_RSTN_VE1               (RTD1319D_RSTN_REG_ID_SOFT_RESET7|0x1e)

#define RTD1319D_CRT_RSTN_PCIE2_STITCH      (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x00)
#define RTD1319D_CRT_RSTN_PCIE2_PHY         (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x02)
#define RTD1319D_CRT_RSTN_PCIE2             (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x04)
#define RTD1319D_CRT_RSTN_PCIE2_CORE        (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x06)
#define RTD1319D_CRT_RSTN_PCIE2_POWER       (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x08)
#define RTD1319D_CRT_RSTN_PCIE2_NONSTICH    (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x0a)
#define RTD1319D_CRT_RSTN_PCIE2_PHY_MDIO    (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x0c)
#define RTD1319D_CRT_RSTN_DCPHY_UMCTL2      (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x0e)
#define RTD1319D_CRT_RSTN_MIPI_DSI          (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x10)
#define RTD1319D_CRT_RSTN_HIFM              (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x12)
#define RTD1319D_CRT_RSTN_NSRAM             (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x14)
#define RTD1319D_CRT_RSTN_AUCPU0_REG        (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x16)
#define RTD1319D_CRT_RSTN_MDL_GENPW         (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x18)
#define RTD1319D_CRT_RSTN_MDL_CHIP          (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x1a)
#define RTD1319D_CRT_RSTN_MDL_IP            (RTD1319D_RSTN_REG_ID_SOFT_RESET9|0x1c)

#define RTD1319D_CRT_RSTN_EMMC              (RTD1319D_RSTN_REG_ID_DUMMY0|0x00)
#define RTD1319D_CRT_RSTN_GPU               (RTD1319D_RSTN_REG_ID_DUMMY1|0x00)
#define RTD1319D_CRT_RSTN_VE2               (RTD1319D_RSTN_REG_ID_DUMMY4|0x00)

#define RTD1319D_CRT_RSTN_ISO_BIST          (RTD1319D_RSTN_REG_ID_SOFT_RESET_BIST|0x00)
#define RTD1319D_CRT_RSTN_MAIN_BIST         (RTD1319D_RSTN_REG_ID_SOFT_RESET_BIST|0x02)
#define RTD1319D_CRT_RSTN_MAIN2_BIST        (RTD1319D_RSTN_REG_ID_SOFT_RESET_BIST|0x04)
#define RTD1319D_CRT_RSTN_VE1_BIST          (RTD1319D_RSTN_REG_ID_SOFT_RESET_BIST|0x06)
#define RTD1319D_CRT_RSTN_VE2_BIST          (RTD1319D_RSTN_REG_ID_SOFT_RESET_BIST|0x08)
#define RTD1319D_CRT_RSTN_DCPHY_BIST        (RTD1319D_RSTN_REG_ID_SOFT_RESET_BIST|0x0a)
#define RTD1319D_CRT_RSTN_GPU_BIST          (RTD1319D_RSTN_REG_ID_SOFT_RESET_BIST|0x0c)
#define RTD1319D_CRT_RSTN_DISP_BIST         (RTD1319D_RSTN_REG_ID_SOFT_RESET_BIST|0x0e)
#define RTD1319D_CRT_RSTN_CAS_BIST          (RTD1319D_RSTN_REG_ID_SOFT_RESET_BIST|0x12)

#define RTD1319D_ISO_RSTN_VFD               (0x00)
#define RTD1319D_ISO_RSTN_IR                (0x01)
#define RTD1319D_ISO_RSTN_CEC0              (0x02)
#define RTD1319D_ISO_RSTN_CEC1              (0x03)
#define RTD1319D_ISO_RSTN_ISO_GSPI          (0x04)
#define RTD1319D_ISO_RSTN_CBUSTX            (0x05)
#define RTD1319D_ISO_RSTN_CBUSRX            (0x06)
#define RTD1319D_ISO_RSTN_LSADC             (0x07)
#define RTD1319D_ISO_RSTN_UR0               (0x08)
#define RTD1319D_ISO_RSTN_GMAC              (0x09)
#define RTD1319D_ISO_RSTN_GPHY              (0x0a)
#define RTD1319D_ISO_RSTN_I2C_0             (0x0b)
#define RTD1319D_ISO_RSTN_I2C_1             (0x0c)
#define RTD1319D_ISO_RSTN_CBUS              (0x0d)
#define RTD1319D_ISO_RSTN_USB_DRD           (0x0e)
#define RTD1319D_ISO_RSTN_USB_HOST          (0x0f)
#define RTD1319D_ISO_RSTN_USB_PHY_0         (0x10)
#define RTD1319D_ISO_RSTN_USB_PHY_1         (0x11)
#define RTD1319D_ISO_RSTN_USB_PHY_2         (0x12)
#define RTD1319D_ISO_RSTN_USB               (0x13)
#define RTD1319D_ISO_RSTN_TYPE_C            (0x14)
#define RTD1319D_ISO_RSTN_USB_U3_HOST       (0x15)
#define RTD1319D_ISO_RSTN_USB3_PHY0_POW     (0x16)
#define RTD1319D_ISO_RSTN_USB3_P0_MDIO      (0x17)
#define RTD1319D_ISO_RSTN_USB3_PHY1_POW     (0x18)
#define RTD1319D_ISO_RSTN_USB3_P1_MDIO      (0x19)
#define RTD1319D_ISO_RSTN_VTC               (0x1a)

#endif /* __DT_BINDINGS_RTK_RESET_RTD1319D_H */
