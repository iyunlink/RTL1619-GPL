/*
 * rtk_nand.h - nand driver
 *
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef __NAND_REG_H
#define __NAND_REG_H


/* NAND flash register definition */
#define REG_ND_PA0		(0x0)
#define	REG_ND_PA1		(0x4)
#define	REG_ND_PA2		(0x8)
#define	REG_ND_CA0		(0xc)
#define	REG_ND_CMD		(0x10)
#define	REG_ND_DAT		(0x14)
#define	REG_ND_CTL		(0x18)
#define REG_BLANK_ZERO_NUM	(0x1c)
#define REG_DEC_CLKMUX		(0x20)
#define	REG_CMD3		(0x28)
#define	REG_ND_PA3		(0x2c)
#define REG_POLL_FSTS		(0x30)
#define REG_BLANK_CHK		(0x34)
#define REG_ND_ECC		(0x38)
#define	REG_ND_CA1		(0x3c)
#define	REG_DATA_TL0		(0x100)
#define	REG_DATA_TL1		(0x104)
#define REG_DMA1_CTL		(0x10c)
#define REG_PP_CTL0		(0x110)
#define REG_ECC_SEL		(0x128)
#define REG_PP_CTL1		(0x12c)
#define	REG_PD			(0x130)
#define REG_NF_LOW_PWR		(0x13c)
#define REG_DUMMY_NF1		(0x158)
#define REG_DUMMY_NF2		(0x15c)
#define REG_DUMMY_NF3		(0x160)
#define REG_DUMMY_NF4		(0x164)
#define REG_AUTO_TRIG		(0x200)
#define REG_RSECC_NUM		(0x204)
#define REG_READ_BY_PP		(0x228)
#define REG_MAX_ECC_NUM		(0x22c)
#define REG_TIME_PARA3		(0x234)
#define REG_TIME_PARA2		(0x238)
#define REG_TIME_PARA1		(0x23c)
#define REG_PIN_MUX_STOP	(0x25c)
#define REG_DELAY_CTL		(0x260)
#define	REG_ECC_STOP		(0x264)
#define	REG_ECC_PAGE		(0x268)
#define	REG_PAGE_CNT		(0x26c)
#define	REG_PAGE_LEN		(0x270)
#define	REG_CMD2		(0x274)
#define	REG_MULTI_CHNL_MODE	(0x27c)
#define REG_SRAM_CTL		(0x300)
#define	REG_DMA_CTL1		(0x304)
#define	REG_DMA_CTL2		(0x308)
#define	REG_DMA_CTL3		(0x30c)
#define REG_READ		(0x31c)
#define	REG_NAND_ISR		(0x324)
#define	REG_NAND_ISREN		(0x328)
#define	REG_DUMMY_SYS		(0x32c)
#define	REG_DBG			(0x344)	
#define	REG_SPR_DDR_CTL		(0x348)	
#define	REG_CP_LEN		(0x34C)	
#define	REG_RND_CMD1		(0x208)
#define	REG_RND_CMD2		(0x20c)
#define	REG_RND_DATA_STR_COL_H	(0x210)
#define	REG_RND_SPR_STR_COL_H	(0x214)
#define REG_RND_SPR_STR_COL_L	(0x218)
#define	REG_RND_EN		(0x21c)
#define REG_RMZ_CTRL		(0x240)
#define REG_RMZ_SEED_L		(0x244)
#define REG_RMZ_SEED_H		(0x248)


#define NF_ND_PA0_page_addr0(value)		(0x000000FF&((value)<<0))
#define NF_ND_PA1_page_addr1(value)		(0x000000FF&((value)<<0))
#define NF_ND_PA2_addr_mode(value)		(0x000000E0&((value)<<5))
#define NF_ND_PA2_page_addr2(value)		(0x0000001F&((value)<<0))
#define NF_ND_CA0_col_addr0(value)		(0x000000FF&((value)<<0))
#define NF_ND_CMD_cmd(value)			(0x000000FF&((value)<<0))
#define NF_ND_DAT_dat(value)			(0x000000FF&((value)<<0))
#define NF_ND_CTL_xfer(value)			(0x00000080&((value)<<7))
#define NF_ND_CTL_ready_busy(value)		(0x00000040&((value)<<6))
#define NF_ND_CTL_ecc_tran(value)		(0x00000020&((value)<<5))
#define NF_ND_CTL_ecc_enable(value)		(0x00000008&((value)<<3))
#define NF_ND_CTL_tran_mode(value)		(0x00000007&((value)<<0))
#define NF_CMD3_cmd3(value)			(0x000000FF&((value)<<0))
#define NF_ND_PA3_page_addr3(value)		(0x000000E0&((value)<<5))
#define NF_POLL_FSTS_bit_sel(value)		(0x0000000E&((value)<<1))
#define NF_POLL_FSTS_trig_poll(value)		(0x00000001&((value)<<0))
#define NF_BLANK_CHK_read_ecc_xnor_ena(value)	(0x00000004&((value)<<2))
#define NF_BLANK_CHK_blank_all_one(value)	(0x00000002&((value)<<1))
#define NF_BLANK_CHK_blank_ena(value)		(0x00000001&((value)<<0))
#define NF_ND_ECC_ecc_not_clr(value)		(0x00000008&((value)<<3))
#define NF_ND_ECC_ecc_err(value)		(0x00000004&((value)<<2))
#define NF_ND_CA1_col_addr1(value)		(0x000000FF&((value)<<0))
#define NF_DATA_TL0_length0(value)		(0x000000FF&((value)<<0))
#define NF_DATA_TL1_access_mode(value)		(0x00000080&((value)<<7))
#define NF_DATA_TL1_sram_path(value)		(0x00000040&((value)<<6))
#define NF_DATA_TL1_length1(value)		(0x0000003F&((value)<<0))
#define NF_DMA_CTL1_dram_sa(value)              (0x0FFFFFFF&((value)<<0))
#define NF_DMA_CTL2_dma_len(value)              (0x0000FFFF&((value)<<0))
#define NF_DMA1_CTL_eot_gen(value)		(0x00000008&((value)<<3))
#define NF_DMA1_CTL_dma_max_pkt(value)		(0x00000004&((value)<<2))
#define NF_DMA1_CTL_spec_pkt(value)		(0x00000002&((value)<<1))
#define NF_PP_CTL0_pp_start_addr(value)		(0x00000030&((value)<<4))
#define NF_PP_CTL0_pp_busy(value)		(0x00000004&((value)<<2))
#define NF_PP_CTL0_pp_reset(value)		(0x00000002&((value)<<1))
#define NF_PP_CTL0_pp_enable(value)		(0x00000001&((value)<<0))
#define NF_ECC_SEL_ecc_bch_24b_ena(value)       (0x00000002&((value)<<1))
#define NF_ECC_SEL_ecc_bch_12b_ena(value)       (0x00000001&((value)<<0))
#define NF_PP_CTL1_pp_start_addr(value)		(0x000000FF&((value)<<0))
#define NF_PD_pd(value)				(0x0000001F&((value)<<0))
#define NF_DUMMY_NF1_Dmy1(value)		(0x000000FF&((value)<<0))
#define NF_DUMMY_NF2_Dmy2(value)		(0x000000FF&((value)<<0))
#define NF_DUMMY_NF3_Dmy3(value)		(0x000000FF&((value)<<0))
#define NF_DUMMY_NF4_Dmy4(value)		(0x000000FF&((value)<<0))
#define NF_AUTO_TRIG_auto_trig(value)		(0x00000080&((value)<<7))
#define NF_AUTO_TRIG_addr_fbd(value)		(0x00000040&((value)<<6))
#define NF_AUTO_TRIG_pp_empty(value)		(0x00000020&((value)<<5))
#define NF_AUTO_TRIG_spec_auto_case(value)	(0x00000018&((value)<<3))
#define NF_AUTO_TRIG_auto_case(value)		(0x00000007&((value)<<0))
#define NF_RSECC_NUM_ecc_num(value)		(0x00000001F&((value)<<0))
#define NF_READ_BY_PP_read_by_pp(value)		(0x00000080&((value)<<7))
#define REG_MAX_ECC_NUM_max_ecc_num(value)	(0x00000001F&((value)<<0))
#define NF_TIME_PARA3_T3(value)			(0x000000FF&((value)<<0))
#define NF_TIME_PARA2_T2(value)			(0x000000FF&((value)<<0))
#define NF_TIME_PARA1_T1(value)			(0x000000FF&((value)<<0))
#define NF_PIN_MUX_STOP_pin_stop(value)         (0x00000004&((value)<<2))
#define NF_PIN_MUX_STOP_intlev_pin_ena(value)   (0x00000002&((value)<<1))
#define NF_PIN_MUX_STOP_pin_mux_enae(value)     (0x00000001&((value)<<0))
#define NF_DELAY_CTL_T_swait_busy(value)	(0x000000C0&((value)<<6))
#define NF_DELAY_CTL_T_WHR_ADL(value)		(0x0000003F&((value)<<0))
#define NF_ECC_STOP_ecc_n_stop(value)		(0x00000080&((value)<<7))
#define NF_ECC_PAGE_ecc_page(value)		(0x000000FF&((value)<<0))
#define NF_PAGE_CNT_page_cnt(value)		(0x000000FF&((value)<<0))
#define NF_PAGE_LEN_page_len(value)		(0x000000FF&((value)<<0))
#define NF_CMD2_cmd2(value)			(0x000000FF&((value)<<0))
#define NF_MULTI_CHNL_MODE_ecc_pass(value)      (0x00000080&((value)<<7))
#define NF_MULTI_CHNL_MODE_ecc_no_check(value)  (0x00000040&((value)<<6))
#define NF_MULTI_CHNL_MODE_edo(value)           (0x00000020&((value)<<5))
#define NF_MULTI_CHNL_MODE_no_wait_busy(value)  (0x00000010&((value)<<4))
#define NF_MULTI_CHNL_MODE_wait_ready(value)    (0x0000000C&((value)<<2))
#define NF_MULTI_CHNL_MODE_multi_chnl_md(value) (0x00000003&((value)<<0))
#define NF_SRAM_CTL_map_sel(value)		(0x00000020&((value)<<5))
#define NF_SRAM_CTL_access_en(value)		(0x00000010&((value)<<4))
#define NF_SRAM_CTL_mem_region(value)		(0x0000000F&((value)<<0))
#define NF_DMA_CTL3_cp_first(value)		(0x00000008&((value)<<3))
#define NF_DMA_CTL3_cp_enable(value)		(0x00000004&((value)<<2))	
#define NF_DMA_CTL3_ddr_wr(value)		(0x00000002&((value)<<1))
#define NF_DMA_CTL3_dma_xfer(value)		(0x00000001&((value)<<0))
#define NF_NAND_ISR_Int5(value)			(0x00000020&((value)<<5))
#define NF_NAND_ISR_Int4(value)			(0x00000010&((value)<<4))
#define NF_NAND_ISR_Int3(value)			(0x00000008&((value)<<3))
#define NF_NAND_ISR_Int2(value)			(0x00000004&((value)<<2))
#define NF_NAND_ISR_Int1(value)			(0x00000002&((value)<<1))
#define NF_NAND_ISR_write_data(value)		(0x00000001&((value)<<0))
#define NF_NAND_ISREN_Int5En(value)		(0x00000020&((value)<<5))
#define NF_NAND_ISREN_Int4En(value)		(0x00000010&((value)<<4))
#define NF_NAND_ISREN_Int3En(value)		(0x00000008&((value)<<3))
#define NF_NAND_ISREN_Int2En(value)		(0x00000004&((value)<<2))
#define NF_NAND_ISREN_Int1En(value)		(0x00000002&((value)<<1))
#define NF_NAND_ISREN_write_data(value)		(0x00000001&((value)<<0))
#define NF_DUMMY_SYS_dmy(value)			(0xFFFFFFFF&((value)<<0))
#define NF_SPR_DDR_CTL_cr_nf_hw_pinmux_ena(value)	(0x40000000&((value)<<30))
#define NF_SPR_DDR_CTL_spare_ddr_ena(value)	(0x20000000&((value)<<29))
#define NF_SPR_DDR_CTL_per_2k_spr_ena(value)	(0x10000000&((value)<<28))
#define NF_SPR_DDR_CTL_spare_dram_sa(value)	(0x1FFFFFFF&((value)<<0))
#define NF_CP_LEN_cp_length(value)		(0x01FFFE00&((value)<<9))


/* Reserve Block Area usage */
#define BB_INIT                 0xFFFE
#define RB_INIT                 0xFFFD
#define BBT_TAG                 0xBB
#define SBT_TAG			0xAA
#define TAG_FACTORY_PARAM       (0x82)
#define BB_DIE_INIT             0xEEEE
#define RB_DIE_INIT             BB_DIE_INIT
#define BAD_RESERVED            0x4444
#define SB_INIT			0xFFAA

#define RTKNF_PROC_DIR_NAME           "rtknf"
#define RTKNF_PROC_INFO_DIR_NAME      "rtknf/info"
#define RTKNF_PROC_TEST_DIR_NAME      "rtknf/test"

enum access_mode {
	ECC,
	RAW
};

enum access_type {
	NFWRITE,
	NFREAD,
	NFERASE
};

enum table_type {
	BBTABLE,
	SBTABLE
};

struct sb_table {
        u16 chipnum;
        u16 block;
        u16 shift;
};

struct bb_table {
        u16 BB_die;
        u16 bad_block;
        u16 RB_die;
        u16 remap_block;
};

struct rtk_buffer {
        unsigned char *dataBuf;
        dma_addr_t dataPhys;
        unsigned int index_r;
        unsigned int index_w;
	unsigned char *tmpBuf;
	unsigned char *oobtmp;
};

struct rtk_nf {
        struct nand_chip chip;
        struct nand_controller controller;
	struct clk* clk_nand;
        struct device *dev;
        void __iomem *regs;
        void __iomem *pll_regs;
#if defined(CONFIG_MTD_RTK_NAND_HW_SEMAPHORE)
	void __iomem *hwsem_base;
#endif
	struct rtk_buffer nandbuf;
	unsigned char cmdbuf[128];
	u64 chipsize;
	u32 ppb;	/* page per block */
	struct nand_flash_dev *nf_ids;
	unsigned char flashname[16];
	unsigned char t1;
	unsigned char t2;
	unsigned char t3;
	unsigned char ecc;
#if defined(CONFIG_MTD_NAND_RTK_BBM)
	struct bb_table *bbt;
	struct sb_table *sbt;
	u32 bbtcrc;
	u32 sbtcrc;
	u32 RBA;
	u32 RBASTART;
	u32 SHIFTBLK;   /* count in shift table */
	u32 bootarea;
	u32 bootblk;
	u32 bootareashift;
#endif
	struct proc_dir_entry *rtknf_proc_dir;
};

int rtk_nf_get_real_page(struct mtd_info *mtd, int page, int type);
int rtk_nf_scan_table(struct mtd_info *mtd);
int rtk_nf_bb_handle(struct mtd_info *mtd, int o_blk, int page,
			int backup, int mode);
void rtk_nf_read_proc_bbtinfo(struct seq_file *m, void *v);
#endif
