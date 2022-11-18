/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2019 Realtek Inc.
 */

#ifndef _HDMI_REG_H_INCLUDED_
#define _HDMI_REG_H_INCLUDED_

#define HDMI_GCPCR                                                                   0x078
#define HDMI_GCPCR_enablegcp_shift                                                   (3)
#define HDMI_GCPCR_enablegcp_mask                                                    (0x00000008)
#define HDMI_GCPCR_enablegcp(data)                                                   (0x00000008&((data)<<3))
#define HDMI_GCPCR_get_enablegcp(data)                                               ((0x00000008&(data))>>3)
#define HDMI_GCPCR_gcp_clearavmute_shift                                             (2)
#define HDMI_GCPCR_gcp_clearavmute_mask                                              (0x00000004)
#define HDMI_GCPCR_gcp_clearavmute(data)                                             (0x00000004&((data)<<2))
#define HDMI_GCPCR_get_gcp_clearavmute(data)                                         ((0x00000004&(data))>>2)
#define HDMI_GCPCR_gcp_setavmute_shift                                               (1)
#define HDMI_GCPCR_gcp_setavmute_mask                                                (0x00000002)
#define HDMI_GCPCR_gcp_setavmute(data)                                               (0x00000002&((data)<<1))
#define HDMI_GCPCR_get_gcp_setavmute(data)                                           ((0x00000002&(data))>>1)
#define HDMI_GCPCR_write_data_shift                                                  (0)
#define HDMI_GCPCR_write_data_mask                                                   (0x00000001)
#define HDMI_GCPCR_write_data(data)                                                  (0x00000001&((data)<<0))
#define HDMI_GCPCR_get_write_data(data)                                              ((0x00000001&(data))>>0)

#define HDMI_PHY_STATUS                                                              0x15c
#define HDMI_PHY_STATUS_wdout_shift                                                  (1)
#define HDMI_PHY_STATUS_wdout_mask                                                   (0x00000002)
#define HDMI_PHY_STATUS_wdout(data)                                                  (0x00000002&((data)<<1))
#define HDMI_PHY_STATUS_wdout_src(data)                                              ((0x00000002&(data))>>1)
#define HDMI_PHY_STATUS_get_wdout(data)                                              ((0x00000002&(data))>>1)
#define HDMI_PHY_STATUS_Rxstatus_shift                                               (0)
#define HDMI_PHY_STATUS_Rxstatus_mask                                                (0x00000001)
#define HDMI_PHY_STATUS_Rxstatus(data)                                               (0x00000001&((data)<<0))
#define HDMI_PHY_STATUS_Rxstatus_src(data)                                           ((0x00000001&(data))>>0)
#define HDMI_PHY_STATUS_get_Rxstatus(data)                                           ((0x00000001&(data))>>0)

/* HDMI_TOP */
#define RXST                                                                         0x40
#define RXST_rxsenseint_shift                                                        (2)
#define RXST_rxsenseint_mask                                                         (0x4)
#define RXST_rxsenseint(data)                                                        (0x4&((data)<<2))
#define RXST_rxsenseint_src(data)                                                    ((0x4&(data))>>2)
#define RXST_get_rxsenseint(data)                                                    ((0x4&(data))>>2)
#define RXST_Rxstatus_shift                                                          (1)
#define RXST_Rxstatus_mask                                                           (0x2)
#define RXST_Rxstatus(data)                                                          (0x2&((data)<<1))
#define RXST_Rxstatus_src(data)                                                      ((0x2&(data))>>1)
#define RXST_get_Rxstatus(data)                                                      ((0x2&(data))>>1)
#define RXST_rxupdated_shift                                                         (0)
#define RXST_rxupdated_mask                                                          (0x1)
#define RXST_rxupdated(data)                                                         (0x1&((data)<<0))
#define RXST_rxupdated_src(data)                                                     ((0x1&(data))>>0)
#define RXST_get_rxupdated(data)                                                     ((0x1&(data))>>0)

#endif /* _HDMI_REG_H_INCLUDED_ */
