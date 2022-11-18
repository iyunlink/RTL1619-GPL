/*
 * hdmitop_reg.h - RTK hdmitx driver header file
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#ifndef _HDMITOP_REG_H_INCLUDED_
#define _HDMITOP_REG_H_INCLUDED_

#define DBG0                                                                         0x800
#define DBG0_reg_addr                                                                "0x9804D800"
#define DBG0_reg                                                                     0x9804D800
#define set_DBG0_reg(data)   (*((volatile unsigned int*) DBG0_reg)=data)
#define get_DBG0_reg   (*((volatile unsigned int*) DBG0_reg))
#define DBG0_inst_adr                                                                "0x0000"
#define DBG0_inst                                                                    0x0000
#define DBG0_rb_dbg_sel1_shift                                                       (12)
#define DBG0_rb_dbg_sel1_mask                                                        (0x00007000)
#define DBG0_rb_dbg_sel1(data)                                                       (0x00007000&((data)<<12))
#define DBG0_rb_dbg_sel1_src(data)                                                   ((0x00007000&(data))>>12)
#define DBG0_get_rb_dbg_sel1(data)                                                   ((0x00007000&(data))>>12)
#define DBG0_rb_dbg_sel0_shift                                                       (9)
#define DBG0_rb_dbg_sel0_mask                                                        (0x00000E00)
#define DBG0_rb_dbg_sel0(data)                                                       (0x00000E00&((data)<<9))
#define DBG0_rb_dbg_sel0_src(data)                                                   ((0x00000E00&(data))>>9)
#define DBG0_get_rb_dbg_sel0(data)                                                   ((0x00000E00&(data))>>9)
#define DBG0_rb_dbg_enable_shift                                                     (8)
#define DBG0_rb_dbg_enable_mask                                                      (0x00000100)
#define DBG0_rb_dbg_enable(data)                                                     (0x00000100&((data)<<8))
#define DBG0_rb_dbg_enable_src(data)                                                 ((0x00000100&(data))>>8)
#define DBG0_get_rb_dbg_enable(data)                                                 ((0x00000100&(data))>>8)
#define DBG0_hdmi_dbg_out1_sel_shift                                                 (4)
#define DBG0_hdmi_dbg_out1_sel_mask                                                  (0x000000F0)
#define DBG0_hdmi_dbg_out1_sel(data)                                                 (0x000000F0&((data)<<4))
#define DBG0_hdmi_dbg_out1_sel_src(data)                                             ((0x000000F0&(data))>>4)
#define DBG0_get_hdmi_dbg_out1_sel(data)                                             ((0x000000F0&(data))>>4)
#define DBG0_hdmi_dbg_out0_sel_shift                                                 (0)
#define DBG0_hdmi_dbg_out0_sel_mask                                                  (0x0000000F)
#define DBG0_hdmi_dbg_out0_sel(data)                                                 (0x0000000F&((data)<<0))
#define DBG0_hdmi_dbg_out0_sel_src(data)                                             ((0x0000000F&(data))>>0)
#define DBG0_get_hdmi_dbg_out0_sel(data)                                             ((0x0000000F&(data))>>0)


#define MISC0                                                                        0x804
#define MISC0_reg_addr                                                               "0x9804D804"
#define MISC0_reg                                                                    0x9804D804
#define set_MISC0_reg(data)   (*((volatile unsigned int*) MISC0_reg)=data)
#define get_MISC0_reg   (*((volatile unsigned int*) MISC0_reg))
#define MISC0_inst_adr                                                               "0x0001"
#define MISC0_inst                                                                   0x0001
#define MISC0_pg_test_en_shift                                                       (0)
#define MISC0_pg_test_en_mask                                                        (0x00000001)
#define MISC0_pg_test_en(data)                                                       (0x00000001&((data)<<0))
#define MISC0_pg_test_en_src(data)                                                   ((0x00000001&(data))>>0)
#define MISC0_get_pg_test_en(data)                                                   ((0x00000001&(data))>>0)


#define MISC1                                                                        0x808
#define MISC1_reg_addr                                                               "0x9804D808"
#define MISC1_reg                                                                    0x9804D808
#define set_MISC1_reg(data)   (*((volatile unsigned int*) MISC1_reg)=data)
#define get_MISC1_reg   (*((volatile unsigned int*) MISC1_reg))
#define MISC1_inst_adr                                                               "0x0002"
#define MISC1_inst                                                                   0x0002
#define MISC1_pg_test1_fail_shift                                                    (3)
#define MISC1_pg_test1_fail_mask                                                     (0x00000008)
#define MISC1_pg_test1_fail(data)                                                    (0x00000008&((data)<<3))
#define MISC1_pg_test1_fail_src(data)                                                ((0x00000008&(data))>>3)
#define MISC1_get_pg_test1_fail(data)                                                ((0x00000008&(data))>>3)
#define MISC1_pg_test0_fail_shift                                                    (2)
#define MISC1_pg_test0_fail_mask                                                     (0x00000004)
#define MISC1_pg_test0_fail(data)                                                    (0x00000004&((data)<<2))
#define MISC1_pg_test0_fail_src(data)                                                ((0x00000004&(data))>>2)
#define MISC1_get_pg_test0_fail(data)                                                ((0x00000004&(data))>>2)
#define MISC1_pg_test1_done_shift                                                    (1)
#define MISC1_pg_test1_done_mask                                                     (0x00000002)
#define MISC1_pg_test1_done(data)                                                    (0x00000002&((data)<<1))
#define MISC1_pg_test1_done_src(data)                                                ((0x00000002&(data))>>1)
#define MISC1_get_pg_test1_done(data)                                                ((0x00000002&(data))>>1)
#define MISC1_pg_test0_done_shift                                                    (0)
#define MISC1_pg_test0_done_mask                                                     (0x00000001)
#define MISC1_pg_test0_done(data)                                                    (0x00000001&((data)<<0))
#define MISC1_pg_test0_done_src(data)                                                ((0x00000001&(data))>>0)
#define MISC1_get_pg_test0_done(data)                                                ((0x00000001&(data))>>0)


#define PLLDIV0                                                                      0x810
#define PLLDIV0_reg_addr                                                             "0x9804D810"
#define PLLDIV0_reg                                                                  0x9804D810
#define set_PLLDIV0_reg(data)   (*((volatile unsigned int*) PLLDIV0_reg)=data)
#define get_PLLDIV0_reg   (*((volatile unsigned int*) PLLDIV0_reg))
#define PLLDIV0_inst_adr                                                             "0x0004"
#define PLLDIV0_inst                                                                 0x0004
#define PLLDIV0_hdmitx_plltmds_clkdet_done_shift                                     (30)
#define PLLDIV0_hdmitx_plltmds_clkdet_done_mask                                      (0x40000000)
#define PLLDIV0_hdmitx_plltmds_clkdet_done(data)                                     (0x40000000&((data)<<30))
#define PLLDIV0_hdmitx_plltmds_clkdet_done_src(data)                                 ((0x40000000&(data))>>30)
#define PLLDIV0_get_hdmitx_plltmds_clkdet_done(data)                                 ((0x40000000&(data))>>30)
#define PLLDIV0_hdmitx_plltmds_clk_count_shift                                       (13)
#define PLLDIV0_hdmitx_plltmds_clk_count_mask                                        (0x3FFFE000)
#define PLLDIV0_hdmitx_plltmds_clk_count(data)                                       (0x3FFFE000&((data)<<13))
#define PLLDIV0_hdmitx_plltmds_clk_count_src(data)                                   ((0x3FFFE000&(data))>>13)
#define PLLDIV0_get_hdmitx_plltmds_clk_count(data)                                   ((0x3FFFE000&(data))>>13)
#define PLLDIV0_hdmitx_plltmds_refclk_count_shift                                    (2)
#define PLLDIV0_hdmitx_plltmds_refclk_count_mask                                     (0x00001FFC)
#define PLLDIV0_hdmitx_plltmds_refclk_count(data)                                    (0x00001FFC&((data)<<2))
#define PLLDIV0_hdmitx_plltmds_refclk_count_src(data)                                ((0x00001FFC&(data))>>2)
#define PLLDIV0_get_hdmitx_plltmds_refclk_count(data)                                ((0x00001FFC&(data))>>2)
#define PLLDIV0_hdmitx_plltmds_count_en_shift                                        (1)
#define PLLDIV0_hdmitx_plltmds_count_en_mask                                         (0x00000002)
#define PLLDIV0_hdmitx_plltmds_count_en(data)                                        (0x00000002&((data)<<1))
#define PLLDIV0_hdmitx_plltmds_count_en_src(data)                                    ((0x00000002&(data))>>1)
#define PLLDIV0_get_hdmitx_plltmds_count_en(data)                                    ((0x00000002&(data))>>1)
#define PLLDIV0_hdmitx_plltmds_rstn_shift                                            (0)
#define PLLDIV0_hdmitx_plltmds_rstn_mask                                             (0x00000001)
#define PLLDIV0_hdmitx_plltmds_rstn(data)                                            (0x00000001&((data)<<0))
#define PLLDIV0_hdmitx_plltmds_rstn_src(data)                                        ((0x00000001&(data))>>0)
#define PLLDIV0_get_hdmitx_plltmds_rstn(data)                                        ((0x00000001&(data))>>0)


#define PLLDIV1                                                                      0x814
#define PLLDIV1_reg_addr                                                             "0x9804D814"
#define PLLDIV1_reg                                                                  0x9804D814
#define set_PLLDIV1_reg(data)   (*((volatile unsigned int*) PLLDIV1_reg)=data)
#define get_PLLDIV1_reg   (*((volatile unsigned int*) PLLDIV1_reg))
#define PLLDIV1_inst_adr                                                             "0x0005"
#define PLLDIV1_inst                                                                 0x0005
#define PLLDIV1_hdmitx_pllpixel_clkdet_done_shift                                    (30)
#define PLLDIV1_hdmitx_pllpixel_clkdet_done_mask                                     (0x40000000)
#define PLLDIV1_hdmitx_pllpixel_clkdet_done(data)                                    (0x40000000&((data)<<30))
#define PLLDIV1_hdmitx_pllpixel_clkdet_done_src(data)                                ((0x40000000&(data))>>30)
#define PLLDIV1_get_hdmitx_pllpixel_clkdet_done(data)                                ((0x40000000&(data))>>30)
#define PLLDIV1_hdmitx_pllpixel_clk_count_shift                                      (13)
#define PLLDIV1_hdmitx_pllpixel_clk_count_mask                                       (0x3FFFE000)
#define PLLDIV1_hdmitx_pllpixel_clk_count(data)                                      (0x3FFFE000&((data)<<13))
#define PLLDIV1_hdmitx_pllpixel_clk_count_src(data)                                  ((0x3FFFE000&(data))>>13)
#define PLLDIV1_get_hdmitx_pllpixel_clk_count(data)                                  ((0x3FFFE000&(data))>>13)
#define PLLDIV1_hdmitx_pllpixel_refclk_count_shift                                   (2)
#define PLLDIV1_hdmitx_pllpixel_refclk_count_mask                                    (0x00001FFC)
#define PLLDIV1_hdmitx_pllpixel_refclk_count(data)                                   (0x00001FFC&((data)<<2))
#define PLLDIV1_hdmitx_pllpixel_refclk_count_src(data)                               ((0x00001FFC&(data))>>2)
#define PLLDIV1_get_hdmitx_pllpixel_refclk_count(data)                               ((0x00001FFC&(data))>>2)
#define PLLDIV1_hdmitx_pllpixel_count_en_shift                                       (1)
#define PLLDIV1_hdmitx_pllpixel_count_en_mask                                        (0x00000002)
#define PLLDIV1_hdmitx_pllpixel_count_en(data)                                       (0x00000002&((data)<<1))
#define PLLDIV1_hdmitx_pllpixel_count_en_src(data)                                   ((0x00000002&(data))>>1)
#define PLLDIV1_get_hdmitx_pllpixel_count_en(data)                                   ((0x00000002&(data))>>1)
#define PLLDIV1_hdmitx_pllpixel_rstn_shift                                           (0)
#define PLLDIV1_hdmitx_pllpixel_rstn_mask                                            (0x00000001)
#define PLLDIV1_hdmitx_pllpixel_rstn(data)                                           (0x00000001&((data)<<0))
#define PLLDIV1_hdmitx_pllpixel_rstn_src(data)                                       ((0x00000001&(data))>>0)
#define PLLDIV1_get_hdmitx_pllpixel_rstn(data)                                       ((0x00000001&(data))>>0)


#define PLLDIV2                                                                      0x818
#define PLLDIV2_reg_addr                                                             "0x9804D818"
#define PLLDIV2_reg                                                                  0x9804D818
#define set_PLLDIV2_reg(data)   (*((volatile unsigned int*) PLLDIV2_reg)=data)
#define get_PLLDIV2_reg   (*((volatile unsigned int*) PLLDIV2_reg))
#define PLLDIV2_inst_adr                                                             "0x0006"
#define PLLDIV2_inst                                                                 0x0006
#define PLLDIV2_hdmirx_vpll_clkdet_done_shift                                        (30)
#define PLLDIV2_hdmirx_vpll_clkdet_done_mask                                         (0x40000000)
#define PLLDIV2_hdmirx_vpll_clkdet_done(data)                                        (0x40000000&((data)<<30))
#define PLLDIV2_hdmirx_vpll_clkdet_done_src(data)                                    ((0x40000000&(data))>>30)
#define PLLDIV2_get_hdmirx_vpll_clkdet_done(data)                                    ((0x40000000&(data))>>30)
#define PLLDIV2_hdmirx_vpll_clk_count_shift                                          (13)
#define PLLDIV2_hdmirx_vpll_clk_count_mask                                           (0x3FFFE000)
#define PLLDIV2_hdmirx_vpll_clk_count(data)                                          (0x3FFFE000&((data)<<13))
#define PLLDIV2_hdmirx_vpll_clk_count_src(data)                                      ((0x3FFFE000&(data))>>13)
#define PLLDIV2_get_hdmirx_vpll_clk_count(data)                                      ((0x3FFFE000&(data))>>13)
#define PLLDIV2_hdmirx_vpll_refclk_count_shift                                       (2)
#define PLLDIV2_hdmirx_vpll_refclk_count_mask                                        (0x00001FFC)
#define PLLDIV2_hdmirx_vpll_refclk_count(data)                                       (0x00001FFC&((data)<<2))
#define PLLDIV2_hdmirx_vpll_refclk_count_src(data)                                   ((0x00001FFC&(data))>>2)
#define PLLDIV2_get_hdmirx_vpll_refclk_count(data)                                   ((0x00001FFC&(data))>>2)
#define PLLDIV2_hdmirx_vpll_count_en_shift                                           (1)
#define PLLDIV2_hdmirx_vpll_count_en_mask                                            (0x00000002)
#define PLLDIV2_hdmirx_vpll_count_en(data)                                           (0x00000002&((data)<<1))
#define PLLDIV2_hdmirx_vpll_count_en_src(data)                                       ((0x00000002&(data))>>1)
#define PLLDIV2_get_hdmirx_vpll_count_en(data)                                       ((0x00000002&(data))>>1)
#define PLLDIV2_hdmirx_vpll_rstn_shift                                               (0)
#define PLLDIV2_hdmirx_vpll_rstn_mask                                                (0x00000001)
#define PLLDIV2_hdmirx_vpll_rstn(data)                                               (0x00000001&((data)<<0))
#define PLLDIV2_hdmirx_vpll_rstn_src(data)                                           ((0x00000001&(data))>>0)
#define PLLDIV2_get_hdmirx_vpll_rstn(data)                                           ((0x00000001&(data))>>0)


#define PLLDIV3                                                                      0x81C
#define PLLDIV3_reg_addr                                                             "0x9804D81C"
#define PLLDIV3_reg                                                                  0x9804D81C
#define set_PLLDIV3_reg(data)   (*((volatile unsigned int*) PLLDIV3_reg)=data)
#define get_PLLDIV3_reg   (*((volatile unsigned int*) PLLDIV3_reg))
#define PLLDIV3_inst_adr                                                             "0x0007"
#define PLLDIV3_inst                                                                 0x0007
#define PLLDIV3_hdmirx_apll_clkdet_done_shift                                        (30)
#define PLLDIV3_hdmirx_apll_clkdet_done_mask                                         (0x40000000)
#define PLLDIV3_hdmirx_apll_clkdet_done(data)                                        (0x40000000&((data)<<30))
#define PLLDIV3_hdmirx_apll_clkdet_done_src(data)                                    ((0x40000000&(data))>>30)
#define PLLDIV3_get_hdmirx_apll_clkdet_done(data)                                    ((0x40000000&(data))>>30)
#define PLLDIV3_hdmirx_apll_clk_count_shift                                          (13)
#define PLLDIV3_hdmirx_apll_clk_count_mask                                           (0x3FFFE000)
#define PLLDIV3_hdmirx_apll_clk_count(data)                                          (0x3FFFE000&((data)<<13))
#define PLLDIV3_hdmirx_apll_clk_count_src(data)                                      ((0x3FFFE000&(data))>>13)
#define PLLDIV3_get_hdmirx_apll_clk_count(data)                                      ((0x3FFFE000&(data))>>13)
#define PLLDIV3_hdmirx_apll_refclk_count_shift                                       (2)
#define PLLDIV3_hdmirx_apll_refclk_count_mask                                        (0x00001FFC)
#define PLLDIV3_hdmirx_apll_refclk_count(data)                                       (0x00001FFC&((data)<<2))
#define PLLDIV3_hdmirx_apll_refclk_count_src(data)                                   ((0x00001FFC&(data))>>2)
#define PLLDIV3_get_hdmirx_apll_refclk_count(data)                                   ((0x00001FFC&(data))>>2)
#define PLLDIV3_hdmirx_apll_count_en_shift                                           (1)
#define PLLDIV3_hdmirx_apll_count_en_mask                                            (0x00000002)
#define PLLDIV3_hdmirx_apll_count_en(data)                                           (0x00000002&((data)<<1))
#define PLLDIV3_hdmirx_apll_count_en_src(data)                                       ((0x00000002&(data))>>1)
#define PLLDIV3_get_hdmirx_apll_count_en(data)                                       ((0x00000002&(data))>>1)
#define PLLDIV3_hdmirx_apll_rstn_shift                                               (0)
#define PLLDIV3_hdmirx_apll_rstn_mask                                                (0x00000001)
#define PLLDIV3_hdmirx_apll_rstn(data)                                               (0x00000001&((data)<<0))
#define PLLDIV3_hdmirx_apll_rstn_src(data)                                           ((0x00000001&(data))>>0)
#define PLLDIV3_get_hdmirx_apll_rstn(data)                                           ((0x00000001&(data))>>0)


#define PLLDIV4                                                                      0x820
#define PLLDIV4_reg_addr                                                             "0x9804D820"
#define PLLDIV4_reg                                                                  0x9804D820
#define set_PLLDIV4_reg(data)   (*((volatile unsigned int*) PLLDIV4_reg)=data)
#define get_PLLDIV4_reg   (*((volatile unsigned int*) PLLDIV4_reg))
#define PLLDIV4_inst_adr                                                             "0x0008"
#define PLLDIV4_inst                                                                 0x0008
#define PLLDIV4_hdmirx_fdds_clkdet_done_shift                                        (30)
#define PLLDIV4_hdmirx_fdds_clkdet_done_mask                                         (0x40000000)
#define PLLDIV4_hdmirx_fdds_clkdet_done(data)                                        (0x40000000&((data)<<30))
#define PLLDIV4_hdmirx_fdds_clkdet_done_src(data)                                    ((0x40000000&(data))>>30)
#define PLLDIV4_get_hdmirx_fdds_clkdet_done(data)                                    ((0x40000000&(data))>>30)
#define PLLDIV4_hdmirx_fdds_clk_count_shift                                          (13)
#define PLLDIV4_hdmirx_fdds_clk_count_mask                                           (0x3FFFE000)
#define PLLDIV4_hdmirx_fdds_clk_count(data)                                          (0x3FFFE000&((data)<<13))
#define PLLDIV4_hdmirx_fdds_clk_count_src(data)                                      ((0x3FFFE000&(data))>>13)
#define PLLDIV4_get_hdmirx_fdds_clk_count(data)                                      ((0x3FFFE000&(data))>>13)
#define PLLDIV4_hdmirx_fdds_refclk_count_shift                                       (2)
#define PLLDIV4_hdmirx_fdds_refclk_count_mask                                        (0x00001FFC)
#define PLLDIV4_hdmirx_fdds_refclk_count(data)                                       (0x00001FFC&((data)<<2))
#define PLLDIV4_hdmirx_fdds_refclk_count_src(data)                                   ((0x00001FFC&(data))>>2)
#define PLLDIV4_get_hdmirx_fdds_refclk_count(data)                                   ((0x00001FFC&(data))>>2)
#define PLLDIV4_hdmirx_fdds_count_en_shift                                           (1)
#define PLLDIV4_hdmirx_fdds_count_en_mask                                            (0x00000002)
#define PLLDIV4_hdmirx_fdds_count_en(data)                                           (0x00000002&((data)<<1))
#define PLLDIV4_hdmirx_fdds_count_en_src(data)                                       ((0x00000002&(data))>>1)
#define PLLDIV4_get_hdmirx_fdds_count_en(data)                                       ((0x00000002&(data))>>1)
#define PLLDIV4_hdmirx_fdds_rstn_shift                                               (0)
#define PLLDIV4_hdmirx_fdds_rstn_mask                                                (0x00000001)
#define PLLDIV4_hdmirx_fdds_rstn(data)                                               (0x00000001&((data)<<0))
#define PLLDIV4_hdmirx_fdds_rstn_src(data)                                           ((0x00000001&(data))>>0)
#define PLLDIV4_get_hdmirx_fdds_rstn(data)                                           ((0x00000001&(data))>>0)


#endif
