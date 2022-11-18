/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __DT_BINDINGS_RTK_CLOCK_RTD1319D_H
#define __DT_BINDINGS_RTK_CLOCK_RTD1319D_H

#define RTD1319D_CRT_CLK_EN_MISC            0
#define RTD1319D_CRT_CLK_EN_PCIE0           1
#define RTD1319D_CRT_CLK_EN_DIP             2
#define RTD1319D_CRT_CLK_EN_GSPI            3
#define RTD1319D_CRT_CLK_EN_PCR             4
#define RTD1319D_CRT_CLK_EN_ISO_MISC        5
#define RTD1319D_CRT_CLK_EN_SDS             6
#define RTD1319D_CRT_CLK_EN_HDMI            7
#define RTD1319D_CRT_CLK_EN_AIO             8
#define RTD1319D_CRT_CLK_EN_GPU             9
#define RTD1319D_CRT_CLK_EN_VE1             10
#define RTD1319D_CRT_CLK_EN_VE2             11
#define RTD1319D_CRT_CLK_EN_TVE             12
#define RTD1319D_CRT_CLK_EN_VO              13
#define RTD1319D_CRT_CLK_EN_LSADC           14
#define RTD1319D_CRT_CLK_EN_SE              15

#define RTD1319D_CRT_CLK_EN_DCU             16
#define RTD1319D_CRT_CLK_EN_CP              17
#define RTD1319D_CRT_CLK_EN_MD              18
#define RTD1319D_CRT_CLK_EN_TP              19
#define RTD1319D_CRT_CLK_EN_RCIC            20
#define RTD1319D_CRT_CLK_EN_NF              21
#define RTD1319D_CRT_CLK_EN_EMMC            22
#define RTD1319D_CRT_CLK_EN_SD              23
#define RTD1319D_CRT_CLK_EN_SDIO_IP         24
#define RTD1319D_CRT_CLK_EN_MIPI            25
#define RTD1319D_CRT_CLK_EN_EMMC_IP         26
#define RTD1319D_CRT_CLK_EN_SDIO            27
#define RTD1319D_CRT_CLK_EN_SD_IP           28
#define RTD1319D_CRT_CLK_EN_CABLERX         29
#define RTD1319D_CRT_CLK_EN_TPB             30
#define RTD1319D_CRT_CLK_EN_MISC_SC1        31

#define RTD1319D_CRT_CLK_EN_MISC_I2C_3      32
#define RTD1319D_CRT_CLK_EN_SCPU            33
#define RTD1319D_CRT_CLK_EN_JPEG            34
#define RTD1319D_CRT_CLK_EN_ACPU            35
#define RTD1319D_CRT_CLK_EN_AE              36
#define RTD1319D_CRT_CLK_EN_MISC_SC0        37
#define RTD1319D_CRT_CLK_EN_AIO_AU_CODEC    38
#define RTD1319D_CRT_CLK_EN_AIO_MOD         39
#define RTD1319D_CRT_CLK_EN_AIO_DA          40
#define RTD1319D_CRT_CLK_EN_AIO_HDMI        41
#define RTD1319D_CRT_CLK_EN_AIO_SPDIF       42
#define RTD1319D_CRT_CLK_EN_AIO_I2S         43
#define RTD1319D_CRT_CLK_EN_AIO_MCLK        44
#define RTD1319D_CRT_CLK_EN_HDMIRX          45
#define RTD1319D_CRT_CLK_EN_HSE             46
#define RTD1319D_CRT_CLK_EN_UR2             47

#define RTD1319D_CRT_CLK_EN_UR1             48
#define RTD1319D_CRT_CLK_EN_FAN             49
#define RTD1319D_CRT_CLK_EN_DCPHY_0         50
#define RTD1319D_CRT_CLK_EN_DCPHY_1         51
#define RTD1319D_CRT_CLK_EN_SATA_WRAP_SYS   52
#define RTD1319D_CRT_CLK_EN_SATA_WRAP_SYSH  53
#define RTD1319D_CRT_CLK_EN_SATA_MAC_SYSH   54
#define RTD1319D_CRT_CLK_EN_R2RDSC          55
#define RTD1319D_CRT_CLK_EN_TPC             56
#define RTD1319D_CRT_CLK_EN_PCIE1           57
#define RTD1319D_CRT_CLK_EN_MISC_I2C_4      58
#define RTD1319D_CRT_CLK_EN_MISC_I2C_5      59
#define RTD1319D_CRT_CLK_EN_TSIO            60
#define RTD1319D_CRT_CLK_EN_VE3             61
#define RTD1319D_CRT_CLK_EN_EDP             62
#define RTD1319D_CRT_CLK_EN_TSIO_TRX        63

#define RTD1319D_CRT_CLK_EN_PCIE2           64
#define RTD1319D_CRT_CLK_EN_ISO_GSPI        65
#define RTD1319D_CRT_CLK_EN_EARC            66
#define RTD1319D_CRT_CLK_EN_LITE            67
#define RTD1319D_CRT_CLK_EN_MIPI_DSI        68
#define RTD1319D_CRT_CLK_EN_AUCPU0          71
#define RTD1319D_CRT_CLK_EN_CABLERX_Q       72
#define RTD1319D_CRT_CLK_EN_NSRAM           73
#define RTD1319D_CRT_CLK_EN_AUCPU_SRAM      75
#define RTD1319D_CRT_CLK_EN_KEYLADDER       77
#define RTD1319D_CRT_CLK_EN_IFCP_KLM        78
#define RTD1319D_CRT_CLK_EN_IFCP            79

#define RTD1319D_CRT_CLK_EN_MDL_GENPW       80
#define RTD1319D_CRT_CLK_EN_MDL_CHIP        81
#define RTD1319D_CRT_CLK_EN_MDL_IP          82
#define RTD1319D_CRT_CLK_EN_MDLM2M          83
#define RTD1319D_CRT_CLK_EN_MDL_XTAL        84

#define RTD1319D_CRT_PLL_SCPU               96
#define RTD1319D_CRT_PLL_BUS                97
#define RTD1319D_CRT_PLL_DCSB               98
#define RTD1319D_CRT_CLK_SYS                99
#define RTD1319D_CRT_CLK_SYSH              100
#define RTD1319D_CRT_PLL_DDSA              101
#define RTD1319D_CRT_PLL_GPU               102
#define RTD1319D_CRT_CLK_GPU               103
#define RTD1319D_CRT_PLL_VE1               104
#define RTD1319D_CRT_PLL_VE2               105
#define RTD1319D_CRT_CLK_VE1               106
#define RTD1319D_CRT_CLK_VE2               107
#define RTD1319D_CRT_CLK_VE3               108
#define RTD1319D_CRT_CLK_VE3_BPU           109
#define RTD1319D_CRT_PLL_DIF               110
#define RTD1319D_CRT_PLL_PSAUD1A           111
#define RTD1319D_CRT_PLL_PSAUD2A           112
#define RTD1319D_CRT_PLL_HIFI              115
#define RTD1319D_CRT_CLK_HIFI              116
#define RTD1319D_CRT_CLK_HIFI_ISO          118
#define RTD1319D_CRT_PLL_EMMC_REF          119
#define RTD1319D_CRT_PLL_EMMC              120
#define RTD1319D_CRT_PLL_EMMC_VP0          121
#define RTD1319D_CRT_PLL_EMMC_VP1          122
#define RTD1319D_CRT_CLK_SC0               123
#define RTD1319D_CRT_CLK_SC1               124

#define RTD1319D_CRT_CLK_DET_PLL_BUS       144
#define RTD1319D_CRT_CLK_DET_PLL_DCSB      145
#define RTD1319D_CRT_CLK_DET_PLL_ACPU      146
#define RTD1319D_CRT_CLK_DET_PLL_DDSA      147
#define RTD1319D_CRT_CLK_DET_PLL_GPU       148
#define RTD1319D_CRT_CLK_DET_PLL_VE1       149
#define RTD1319D_CRT_CLK_DET_PLL_VE2       150
#define RTD1319D_CRT_CLK_DET_PLL_HIFI      152

#define RTD1319D_CRT_CLK_MAX               153

#define RTD1319D_ISO_CLK_EN_LSADC            0
#define RTD1319D_ISO_CLK_EN_ISO_GSPI         1
#define RTD1319D_ISO_CLK_EN_MISC_CEC0        2
#define RTD1319D_ISO_CLK_EN_CBUSRX_SYS       3
#define RTD1319D_ISO_CLK_EN_CBUSTX_SYS       4
#define RTD1319D_ISO_CLK_EN_CBUS_SYS         5
#define RTD1319D_ISO_CLK_EN_CBUS_OSC         6
#define RTD1319D_ISO_CLK_EN_MISC_IR          7
#define RTD1319D_ISO_CLK_EN_MISC_UR0         8
#define RTD1319D_ISO_CLK_EN_I2C0             9
#define RTD1319D_ISO_CLK_EN_I2C1            10
#define RTD1319D_ISO_CLK_EN_ETN_250M        11
#define RTD1319D_ISO_CLK_EN_ETN_SYS         12
#define RTD1319D_ISO_CLK_EN_USB_DRD         13
#define RTD1319D_ISO_CLK_EN_USB_HOST        14
#define RTD1319D_ISO_CLK_EN_USB_U3_HOST     15
#define RTD1319D_ISO_CLK_EN_USB             16
#define RTD1319D_ISO_CLK_EN_VTC             17
#define RTD1319D_ISO_CLK_EN_MISC_VFD        18

#define RTD1319D_ISO_CLK_MAX                19

#endif /* __DT_BINDINGS_RTK_CLOCK_RTD1319D_H */
