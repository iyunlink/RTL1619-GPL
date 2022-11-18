/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2019 Realtek Inc.
 */

#ifndef _SYS_REG_H_INCLUDED_
#define _SYS_REG_H_INCLUDED_

#define SYS_PLL_HDMI                                                                 0x190
#define SYS_PLL_HDMI_PLLDISP_OEB_shift                                               (8)
#define SYS_PLL_HDMI_PLLDISP_OEB_mask                                                (0x00000100)
#define SYS_PLL_HDMI_PLLDISP_OEB(data)                                               (0x00000100&((data)<<8))
#define SYS_PLL_HDMI_get_PLLDISP_OEB(data)                                           ((0x00000100&(data))>>8)
#define SYS_PLL_HDMI_PLLDISP_VCORSTB_shift                                           (7)
#define SYS_PLL_HDMI_PLLDISP_VCORSTB_mask                                            (0x00000080)
#define SYS_PLL_HDMI_PLLDISP_VCORSTB(data)                                           (0x00000080&((data)<<7))
#define SYS_PLL_HDMI_get_PLLDISP_VCORSTB(data)                                       ((0x00000080&(data))>>7)
#define SYS_PLL_HDMI_REG_PLL_MHL3_DIV_EN_shift                                       (6)
#define SYS_PLL_HDMI_REG_PLL_MHL3_DIV_EN_mask                                        (0x00000040)
#define SYS_PLL_HDMI_REG_PLL_MHL3_DIV_EN(data)                                       (0x00000040&((data)<<6))
#define SYS_PLL_HDMI_get_REG_PLL_MHL3_DIV_EN(data)                                   ((0x00000040&(data))>>6)
#define SYS_PLL_HDMI_REG_PLLDISP_RSTB_shift                                          (5)
#define SYS_PLL_HDMI_REG_PLLDISP_RSTB_mask                                           (0x00000020)
#define SYS_PLL_HDMI_REG_PLLDISP_RSTB(data)                                          (0x00000020&((data)<<5))
#define SYS_PLL_HDMI_get_REG_PLLDISP_RSTB(data)                                      ((0x00000020&(data))>>5)
#define SYS_PLL_HDMI_REG_PLLDISP_POW_shift                                           (4)
#define SYS_PLL_HDMI_REG_PLLDISP_POW_mask                                            (0x00000010)
#define SYS_PLL_HDMI_REG_PLLDISP_POW(data)                                           (0x00000010&((data)<<4))
#define SYS_PLL_HDMI_get_REG_PLLDISP_POW(data)                                       ((0x00000010&(data))>>4)
#define SYS_PLL_HDMI_REG_TMDS_POW_shift                                              (3)
#define SYS_PLL_HDMI_REG_TMDS_POW_mask                                               (0x00000008)
#define SYS_PLL_HDMI_REG_TMDS_POW(data)                                              (0x00000008&((data)<<3))
#define SYS_PLL_HDMI_get_REG_TMDS_POW(data)                                          ((0x00000008&(data))>>3)
#define SYS_PLL_HDMI_REG_PLL_RSTB_shift                                              (2)
#define SYS_PLL_HDMI_REG_PLL_RSTB_mask                                               (0x00000004)
#define SYS_PLL_HDMI_REG_PLL_RSTB(data)                                              (0x00000004&((data)<<2))
#define SYS_PLL_HDMI_get_REG_PLL_RSTB(data)                                          ((0x00000004&(data))>>2)
#define SYS_PLL_HDMI_REG_PLL_POW_shift                                               (1)
#define SYS_PLL_HDMI_REG_PLL_POW_mask                                                (0x00000002)
#define SYS_PLL_HDMI_REG_PLL_POW(data)                                               (0x00000002&((data)<<1))
#define SYS_PLL_HDMI_get_REG_PLL_POW(data)                                           ((0x00000002&(data))>>1)
#define SYS_PLL_HDMI_REG_HDMI_CK_EN_shift                                            (0)
#define SYS_PLL_HDMI_REG_HDMI_CK_EN_mask                                             (0x00000001)
#define SYS_PLL_HDMI_REG_HDMI_CK_EN(data)                                            (0x00000001&((data)<<0))
#define SYS_PLL_HDMI_get_REG_HDMI_CK_EN(data)                                        ((0x00000001&(data))>>0)
#endif /* _SYS_REG_H_INCLUDED_ */