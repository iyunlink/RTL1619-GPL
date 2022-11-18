/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SOC_REALTEK_ISO_H
#define __SOC_REALTEK_ISO_H

#define ISO_ISR                                                       0x00
#define ISO_ISR_I2C1_REQ_INT                                          0x80000000
#define ISO_ISR_PORB_AV_CEN_INT                                       0x40000000
#define ISO_ISR_PORB_DV_CEN_INT                                       0x20000000
#define ISO_ISR_PORB_HV_CEN_INT                                       0x10000000
#define ISO_ISR_USB_INT_U2_DRD                                        0x04000000
#define ISO_ISR_USB_INT_U3_DRD                                        0x02000000
#define ISO_ISR_USB_INT_HOST                                          0x01000000
#define ISO_ISR_ETN_INT                                               0x00800000
#define ISO_ISR_CBUS_INT                                              0x00400000
#define ISO_ISR_ISO_MISC_INT                                          0x00200000
#define ISO_ISR_GPIODA_INT                                            0x00100000
#define ISO_ISR_GPIOA_INT                                             0x00080000
#define ISO_ISR_RTC_ALARM_INT                                         0x00002000
#define ISO_ISR_RTC_HSEC_INT                                          0x00001000
#define ISO_ISR_I2C1_INT                                              0x00000800
#define ISO_ISR_TC7_INT                                               0x00000400
#define ISO_ISR_TC4_INT                                               0x00000200
#define ISO_ISR_I2C0_INT                                              0x00000100
#define ISO_ISR_WDOG_NMI_INT                                          0x00000080
#define ISO_ISR_SPI1_INT                                              0x00000040
#define ISO_ISR_IRDA_INT                                              0x00000020
#define ISO_ISR_LSADC0_INT                                            0x00000008
#define ISO_ISR_UR0_INT                                               0x00000004
#define ISO_ISR_TC3_INT                                               0x00000002
#define ISO_ISR_WRITE_DATA                                            0x00000001
#define ISO_ISR_CLEAR_DATA                                            0x00000000


#define ISO_UMSK_ISR                                                  0x04
#define ISO_UMSK_ISR_I2C1_REQ_INT                                     0x80000000
#define ISO_UMSK_ISR_PORB_AV_CEN_INT                                  0x40000000
#define ISO_UMSK_ISR_PORB_DV_CEN_INT                                  0x20000000
#define ISO_UMSK_ISR_PORB_HV_CEN_INT                                  0x10000000
#define ISO_UMSK_ISR_ETN_PHY_INTR                                     0x08000000
#define ISO_UMSK_ISR_GPIODA_INT                                       0x00100000
#define ISO_UMSK_ISR_GPIOA_INT                                        0x00080000
#define ISO_UMSK_ISR_RTC_ALARM_INT                                    0x00002000
#define ISO_UMSK_ISR_RTC_HSEC_INT                                     0x00001000
#define ISO_UMSK_ISR_TC7_INT                                          0x00000400
#define ISO_UMSK_ISR_TC4_INT                                          0x00000200
#define ISO_UMSK_ISR_WDOG_NMI_INT                                     0x00000080
#define ISO_UMSK_ISR_SPI1_INT                                         0x00000040
#define ISO_UMSK_ISR_IRDA_INT                                         0x00000020
#define ISO_UMSK_ISR_TC3_INT                                          0x00000002
#define ISO_UMSK_ISR_WRITE_DATA                                       0x00000001
#define ISO_UMSK_ISR_CLEAR_DATA                                       0x00000000

#define ISO_RTC                                                       0x34
#define ISO_RTC_HSEC_SYNC                                             0x00000004
#define ISO_RTC_HSEC_INT_EN                                           0x00000002
#define ISO_RTC_ALARM_INT_EN                                          0x00000001

#endif
