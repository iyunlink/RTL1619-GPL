/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SOC_REALTEK_MISC_H
#define __SOC_REALTEK_MISC_H

#define MIS_ISR                                                       0x0C

#define MIS_ISR_FAN_INT                                               0x20000000
#define MIS_ISR_SPI_INT                                               0x08000000
#define MIS_ISR_SC1_INT                                               0x02000000
#define MIS_ISR_SC0_INT                                               0x01000000
#define MIS_ISR_I2C3_INT                                              0x00800000
#define MIS_ISR_I2C5_INT                                              0x00004000
#define MIS_ISR_UR2_TO_INT                                            0x00002000
#define MIS_ISR_RTC_DATE_INT                                          0x00001000
#define MIS_ISR_RTC_HOUR_INT                                          0x00000800
#define MIS_ISR_RTC_MIN_INT                                           0x00000400
#define MIS_ISR_RTC_HSEC_INT                                          0x00000200
#define MIS_ISR_UR2_INT                                               0x00000100
#define MIS_ISR_TC1_INT                                               0x00000080
#define MIS_ISR_TC0_INT                                               0x00000040
#define MIS_ISR_UR1_TO_INT                                            0x00000020
#define MIS_ISR_TC5_INT                                               0x00000010
#define MIS_ISR_UR1_INT                                               0x00000008
#define MIS_ISR_WDOG_NMI_INT                                          0x00000004
#define MIS_ISR_WRITE_DATA                                            0x00000001
#define MIS_ISR_CLEAR_DATA                                            0x00000000

#endif
