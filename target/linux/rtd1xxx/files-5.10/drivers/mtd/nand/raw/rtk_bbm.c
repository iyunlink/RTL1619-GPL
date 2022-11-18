/*
 * RTK NAND Flash controller driver.
 * Copyright (C) 2020 Realtek Inc.
 * Authors : PK Chuang	<pk.chuang@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/module.h>
#include <linux/iopoll.h>
#include <linux/crc32.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/reset.h>
#include "rtk_nand.h"

#ifdef CONFIG_MTD_NAND_RTK_LEGACY
#define CRCLEN		0
#define TAGOFFSET	0
#define BLOCKINFO	0
#else
#define CRCLEN		4
#define TAGOFFSET	4
#define BLOCKINFO	2
#endif

#define	BBT1		1
#define	BBT2		2
#define SBT1		4
#define SBT2		5
#define SBTCNT		16
#define BOOTBLKSTART	6
#define NOTBOOTAREA	0
#define BOOTAREA	1

static unsigned int crc_table[256];
static int crc_table_computed = 0;

void rtk_nf_read_proc_bbtinfo(struct seq_file *m, void *v)
{
        struct rtk_nf *nf = (struct rtk_nf *)m->private;
        struct nand_chip *chip = &nf->chip;
        struct mtd_info *mtd = nand_to_mtd(chip);
        struct bb_table *bbt = nf->bbt;
        struct sb_table *sbt = nf->sbt;
        int i;

        seq_printf(m, "RBA          : %u\n", nf->RBA);
        seq_puts(m, "\n");

        if (sbt) {
                seq_puts(m, "Shift block table :\n");
                for (i = 0; i < 16; i++) {
                        if (sbt[i].block == SB_INIT)
                                seq_printf(m, "[%d] (SB_INIT, %d)\n",
                                        i, sbt[i].shift);
                        else
                                seq_printf(m, "[%d] (%d, %d)\n",
                                        i, sbt[i].block, sbt[i].shift);
                }
                seq_puts(m, "\n");
        }

        if (bbt) {
                seq_puts(m, "Bad block table :\n");
                for (i = 0; i < nf->RBA; i++) {
                        if ((bbt[i].BB_die == BB_DIE_INIT) &&
                                (bbt[i].bad_block == BB_INIT)) {
                                seq_printf(m, "[%d] (BB_DIE_INIT, BB_INIT, %d, %d)\n",
                                                i,
                                                bbt[i].RB_die,
                                                bbt[i].remap_block);

                        } else if (bbt[i].bad_block == BAD_RESERVED) {
                                seq_printf(m, "[%d] (%d, 0x%x, %d, %d\n",
                                                i,
                                                bbt[i].BB_die,
                                                bbt[i].bad_block,
                                                bbt[i].RB_die,
                                                bbt[i].remap_block);
                        } else {
                                seq_printf(m, "[%d] (%d, %d, %d, %d)\n",
                                                i,
                                                bbt[i].BB_die,
                                                bbt[i].bad_block,
                                                bbt[i].RB_die,
                                                bbt[i].remap_block);
                        }
                }
                seq_puts(m, "\n");
        }
}

static unsigned int nf_Reflect(unsigned int ref, char ch)
{
        unsigned int value = 0;
        int i;
        /* Swap bit 0 for bit 7 */
        /* bit 1 for bit 6, etc. */
        for (i = 1; i < (ch + 1); i++) {
                if (ref & 1)
                        value |= 1 << (ch - i);
                ref >>= 1;
        }

        return value;
}

static void nf_make_crc_table(void)
{
        unsigned int polynomial = 0x04C11DB7;
        int i, j;

        for (i = 0; i <= 0xFF; i++) {
                crc_table[i] = nf_Reflect(i, 8) << 24;
                for (j = 0; j < 8; j++)
                        crc_table[i] = (crc_table[i] << 1) ^ (crc_table[i] &
						(1 << 31) ? polynomial : 0);
		crc_table[i] = nf_Reflect(crc_table[i],  32);
        }

        crc_table_computed = 1;
}

static unsigned int nf_crc32(unsigned char *p, int len, unsigned int *crc)
{
        int cnt = len;
        unsigned int value;

        if (!crc_table_computed)
                nf_make_crc_table();

        value = 0xFFFFFFFF;
        while (cnt--) {
                value = (value >> 8) ^ crc_table[(value & 0xFF) ^ *p++];
        }

        *crc = value ^ 0xFFFFFFFF;

        return 0;
}

static int rtk_nf_table_crc_calculate(struct rtk_nf *nf, int type, void *t,
					int *cnt)
{
	unsigned int hash_value = 0;
	struct bb_table *bbt;
        struct sb_table *sbt;
	int i;

	if (type == BBTABLE) {
                bbt = (struct bb_table *)t;
                if (cnt)
                        *cnt = 0;

                for (i = 0; i < nf->RBA; i++) {
                        if ((bbt[i].BB_die != BB_DIE_INIT) &&
                                (bbt[i].bad_block != BB_INIT)) {
                                if (cnt)
                                        *cnt = *cnt + 1;
                        }
                }

		nf_crc32((unsigned char *)bbt, sizeof(struct bb_table)*nf->RBA,
					&hash_value);
        } else if (type == SBTABLE) {
                sbt = (struct sb_table *)t;
		nf_crc32((unsigned char *)sbt, sizeof(struct sb_table)*16,
					&hash_value);
        }

        return hash_value;
}

static unsigned int rtk_nf_page_to_block(struct rtk_nf *nf, int page)
{
        return page / nf->ppb;
}

static unsigned int rtk_nf_page_offset_in_block(struct rtk_nf *nf, int page)
{
        return page % nf->ppb;
}

static void rtk_nf_dump_sbt(struct rtk_nf *nf)
{
	struct device *dev = nf->dev;
        int i;

        for (i=0; i<SBTCNT; i++) {
                if (nf->sbt[i].block == SB_INIT)
                        break;

                dev_info(dev, "[%d](%d, %d)\n", i, nf->sbt[i].block,
						nf->sbt[i].shift);
        }

        nf->SHIFTBLK = i;

        return;
}

static void rtk_nf_dump_bbt(struct rtk_nf *nf)
{
	struct device *dev = nf->dev;
	int i;

	for (i=0; i<nf->RBA; i++) {
		if (nf->bbt[i].bad_block != BB_INIT)
			dev_info(dev, "[%d](%d, %d)\n",  i,
				nf->bbt[i].bad_block, nf->bbt[i].remap_block);
	}
	nf->RBASTART = nf->bbt[i-1].remap_block;

	return;
}

static int rtk_nf_check_shift_table(struct rtk_nf *nf, unsigned int blk)
{
        int r_blk = blk;
        int i;

	if (!nf->sbt)
		return r_blk;

        for (i=0; i<SBTCNT; i++) {
                if (nf->sbt[i].chipnum != SB_INIT) {
                        if ((r_blk >= nf->sbt[i].block))
                                r_blk = blk + nf->sbt[i].shift;
                } else
                        break;
        }

        return r_blk;
}

static int rtk_nf_check_badblock_table(struct rtk_nf *nf, unsigned int blk)
{
        int r_blk = blk;
        int i;

        for (i=0; i<nf->RBA; i++) {
                if (r_blk == nf->bbt[i].bad_block)
                        r_blk = nf->bbt[i].remap_block;
        }

        return r_blk;
}
#if 0
static int rtk_nf_bootarea_blk(struct mtd_info *mtd, unsigned int blk)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);

#ifndef CONFIG_MTD_NAND_RTK_LEGACY
	if (blk >= BOOTBLKSTART + nf->bootblk) {
		nf->bootarea = NOTBOOTAREA;
                return -1;
	}
#endif
	/* write bootarea start */
	if (blk == BOOTBLKSTART) {
		nf->bootarea = BOOTAREA;
		nf->bootareashift = 0;
	}

	return blk + nf->bootareashift;
}
#endif
static int rtk_nf_ignore_table(struct mtd_info *mtd, unsigned int blk)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);

	if (blk < BOOTBLKSTART || blk >= nf->RBASTART)
		return 1;

	return 0;
}

int rtk_nf_get_real_page(struct mtd_info *mtd, int page, int mode)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	int offset, block; /* bootblk; */

	offset = rtk_nf_page_offset_in_block(nf, page);
	block = rtk_nf_page_to_block(nf, page);

	if (rtk_nf_ignore_table(mtd, block))
		return (block * nf->ppb) + offset;
#if 0
	if ((bootblk = rtk_nf_bootarea_blk(mtd, block)) >= 0)
		return (bootblk * nf->ppb) + offset;
#endif
	if (mode == SBTABLE) {
		if (nf->sbt)
			block = rtk_nf_check_shift_table(nf, block);
	} else {
		if (nf->bbt)
			block = rtk_nf_check_badblock_table(nf, block);
	}

	return (block * nf->ppb) + offset;
}

static int rtk_nf_find_available_reserved_block(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct device *dev = nf->dev;
        int i;

        for (i = 0; i < nf->RBA; i++) {
                if (nf->bbt[i].bad_block == BB_INIT) {
			return nf->bbt[i].remap_block;
                }
        }

	dev_err(dev, "RTK %s(%d) No available reserved block.\n",
		__func__, __LINE__);

        return -1;
}

static void rtk_nf_update_BBT(struct mtd_info *mtd, int o_blk,
			      int s_blk, int m_blk)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	unsigned int blk = ((unsigned int)nf->chipsize / mtd->erasesize) - BLOCKINFO;

	if (s_blk == 0) {
		nf->bbt[(blk-1) - m_blk].bad_block = BAD_RESERVED;
		nf->bbt[(blk-1) - m_blk].BB_die = 0;
	}
	else {
		if (o_blk != s_blk) {
			nf->bbt[(blk-1) - s_blk].bad_block = BAD_RESERVED;
			nf->bbt[(blk-1) - s_blk].BB_die = 0;
		}

		nf->bbt[(blk-1) - m_blk].bad_block = o_blk;
		nf->bbt[(blk-1) - m_blk].BB_die = 0;
	}

        return;
}

static int rtk_nf_write_bbt_to_flash(struct mtd_info *mtd, unsigned int page)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct rtk_buffer *buffer = &nf->nandbuf;
	struct device *dev = nf->dev;
	u32 crc;
	int ret;

	chip->legacy.select_chip(chip, 0);
        chip->legacy.cmdfunc(chip, NAND_CMD_ERASE1, -1, page);
	memset(buffer->dataBuf, 0xff, mtd->writesize + mtd->oobsize);

	if (CRCLEN) {
		crc = rtk_nf_table_crc_calculate(nf, BBTABLE,
					(unsigned char *)nf->bbt, NULL);
		memcpy(buffer->dataBuf, &crc, CRCLEN);
	}
	memcpy(buffer->dataBuf + CRCLEN, nf->bbt,
				sizeof(struct bb_table) * nf->RBA);
	memset(chip->oob_poi, 0xFF, mtd->oobsize);
	*(chip->oob_poi + TAGOFFSET) = BBT_TAG;

	ret = chip->ecc.write_page(chip, buffer->dataBuf, 0, page);
	if (ret) {
		dev_info(dev, "RTK %s(%d) write bbt%d fail.(%d)\n",
			__func__, __LINE__,
			(page == 64) ? 1 : 2, ret);
		goto rtk_nf_write_bbt_to_flash_exit;
	}

rtk_nf_write_bbt_to_flash_exit:
	return ret;
}

static int rtk_nf_write_bbt(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	int ret1, ret2;

	ret1 = rtk_nf_write_bbt_to_flash(mtd, BBT1 * nf->ppb);

	ret2 = rtk_nf_write_bbt_to_flash(mtd, BBT2 * nf->ppb);

	return ((ret1 < 0) && (ret2 < 0)) ? -1 : 0;
}

static int rtk_nf_backup_block(struct mtd_info *mtd, int src_b, int map_b,
			       int offset, int mode)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct device *dev = nf->dev;
	struct rtk_buffer *buffer = &nf->nandbuf;
	int s_page = src_b * nf->ppb;
	int b_page = map_b * nf->ppb;
	char *buf;
	int ret;
	int i;

	dev_info(dev, "RTK %s(%d) backup %d to %d.\n",
			__func__, __LINE__, src_b, map_b);

	buf = kmalloc(mtd->writesize, GFP_KERNEL);
	if (!buf) {
		ret = -2;
		goto rtk_nf_backup_block_exit;
	}

	chip->legacy.select_chip(chip, 0);

        chip->legacy.cmdfunc(chip, NAND_CMD_ERASE1, -1, b_page);

	for (i=0; i<nf->ppb; i++) {
		ret = chip->ecc.read_oob(chip, s_page + i);
		if (ret == -1)
			goto rtk_nf_backup_block_exit;
		else if ((ret == 1) || ((i == offset) && (mode == NFWRITE)))
			continue;

		memcpy(buf, buffer->dataBuf, mtd->writesize);

		ret = chip->ecc.write_page(chip, buf, 0, b_page + i);
		if (ret < 0)
			goto rtk_nf_backup_block_exit;
	}
	
rtk_nf_backup_block_exit:
	if (buf)
		kfree(buf);

	return ret;
}

static int rtk_nf_bb_handle_in_bootarea(struct mtd_info *mtd, int o_blk,
					int page, int mode)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	int offset = rtk_nf_page_offset_in_block(nf, page);
	int ret = 0;
        int i = 0;

        do {
		i++;
                if (o_blk + i >= (BOOTBLKSTART + nf->bootblk))
                        return -1;

                ret = rtk_nf_backup_block(mtd, o_blk, o_blk + i, offset, mode);
                nf->bootareashift++;
        } while (ret < 0);

	return ret;
}

int rtk_nf_bb_handle(struct mtd_info *mtd, int o_blk, int page,
		     int backup, int mode)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	int s_blk = rtk_nf_page_to_block(nf, page);
	int m_blk = 0;
	int offset = rtk_nf_page_offset_in_block(nf, page);
	int ret = 0;

#ifndef CONFIG_MTD_NAND_RTK_LEGACY
	if (rtk_nf_ignore_table(mtd, o_blk))
                return 0;

	if (nf->bootarea == BOOTAREA)
		return rtk_nf_bb_handle_in_bootarea(mtd, o_blk, page, mode);
#endif
rtk_nf_bb_handle_redo:
	m_blk = rtk_nf_find_available_reserved_block(mtd);
	if (m_blk <= 0)
		return -1;

	if (backup) {
		ret = rtk_nf_backup_block(mtd, s_blk, m_blk, offset, mode);
		if (ret < 0) {
			if (ret == -1) {
				rtk_nf_update_BBT(mtd, 0, 0, m_blk);
				goto rtk_nf_bb_handle_redo;
			} else {
				return ret;
			}
		}
	}

	rtk_nf_update_BBT(mtd, o_blk, s_blk, m_blk);

	return rtk_nf_write_bbt(mtd);
}

static int rtk_nf_table_crc_check(struct mtd_info *mtd, int type, int *cnt)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
        struct rtk_buffer *buffer = &nf->nandbuf;
        unsigned int crc_c = 0;
        unsigned int crc_r = 0;

        memcpy(&crc_r, buffer->dataBuf, sizeof(unsigned int));

        crc_c = rtk_nf_table_crc_calculate(nf, type, buffer->dataBuf + 4, cnt);

        return (crc_c == crc_r) ? 0 : -1;
}

#ifdef CONFIG_MTD_NAND_RTK_LEGACY
static int rtk_nf_get_bbt_count(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
        struct rtk_buffer *buffer = &nf->nandbuf;
	struct bb_table *bbt = (struct bb_table *)buffer->dataBuf;
	int cnt = 0;
        int i;

        for (i = 0; i < nf->RBA; i++) {
		if ((bbt[i].BB_die != BB_DIE_INIT) && 
			(bbt[i].bad_block != BB_INIT)) {
			cnt++;
                }
        }

	return cnt;
}
#endif

static int rtk_nf_scan_tag(struct mtd_info *mtd, unsigned int page,
                           const unsigned int magic, int *cnt)
{
        int mode = (magic == BBT_TAG) ? BBTABLE : SBTABLE;
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct rtk_buffer *buffer = &nf->nandbuf;
	int ret;

	chip->legacy.select_chip(chip, 0);

        ret = chip->ecc.read_oob(chip, page);
        if (ret < 0)
                return -1;

        if (*(chip->oob_poi + TAGOFFSET) != magic)
                return -1;

	memcpy(buffer->dataBuf, chip->data_buf, mtd->writesize);
#ifdef CONFIG_MTD_NAND_RTK_LEGACY
	*cnt = rtk_nf_get_bbt_count(mtd);
	return 0;
#else
	return rtk_nf_table_crc_check(mtd, mode, cnt);
#endif
}

static int rtk_nf_scan_shift_table(struct mtd_info *mtd)
{
	int ret = 0;
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
#ifdef CONFIG_MTD_NAND_RTK_LEGACY
	nf->SHIFTBLK = 0;
        nf->sbt = NULL;
#else
	struct rtk_buffer *buffer = &nf->nandbuf;
        struct device *dev = nf->dev;

	nf->sbt = kmalloc(sizeof(struct sb_table) * SBTCNT, GFP_KERNEL);
        if (!nf->sbt) {
                dev_err(dev, "RTK %s(%d) alloc sbt fail.\n",
                        __func__, __LINE__);
                return -ENOMEM;
        }
        memset(nf->sbt, 0xff, sizeof(struct sb_table) * nf->RBA);

	ret = rtk_nf_scan_tag(mtd, SBT1 * nf->ppb, SBT_TAG, NULL);
        if (ret == 0) {
                memcpy(nf->sbt, buffer->dataBuf + CRCLEN, 
					SBTCNT * sizeof(struct sb_table));
                rtk_nf_dump_sbt(nf);
        } else {
                ret = rtk_nf_scan_tag(mtd, SBT2 * nf->ppb, SBT_TAG, NULL);
                if (ret == 0) {
                        memcpy(nf->sbt, buffer->dataBuf + CRCLEN, 
					SBTCNT * sizeof(struct sb_table));
                        rtk_nf_dump_sbt(nf);
                } else {
			dev_err(dev, "RTK %s(%d) NO SBT1 & SBT2.\n",
						__func__, __LINE__);
                        kfree(nf->sbt);
                        nf->sbt = NULL;
                }
        }
#endif
	return ret;
}

static void rtk_nf_bbt_sync(struct mtd_info *mtd, u32 bbtid)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	int ret;
	
	ret = rtk_nf_write_bbt_to_flash(mtd, bbtid * nf->ppb);
}

int rtk_nf_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct rtk_buffer *buffer = &nf->nandbuf;
	struct device *dev = nf->dev;
	int cnt1, cnt2;
        int sync = 0;
	u8 *bbt = NULL;
	u32 b;
	int ret = 0;

	/* calculate RBA */
	b = (unsigned int)nf->chipsize / mtd->erasesize;
	nf->RBA = ((b * 5) / 100) - BLOCKINFO - nf->SHIFTBLK;

	nf->bbt = kmalloc(sizeof(struct bb_table) * nf->RBA, GFP_KERNEL);
	if (!nf->bbt) {
		dev_err(dev, "RTK %s(%d) alloc bbt fail.\n", 
			__func__, __LINE__);
		return -ENOMEM;
	} 
	memset(nf->bbt, 0xff, sizeof(struct bb_table) * nf->RBA);

	bbt = kmalloc(sizeof(struct bb_table) * nf->RBA, GFP_KERNEL);
	if (!bbt) {
		dev_err(dev, "RTK %s(%d) alloc bbt temp fail.\n", 
			__func__, __LINE__);
		return -ENOMEM;
	}
	memset(bbt, 0xff, sizeof(struct bb_table) * nf->RBA);

	ret = rtk_nf_scan_tag(mtd, BBT1 * nf->ppb, BBT_TAG, &cnt1);
        if (ret < 0) {
                dev_err(dev, "RTK %s(%d) read bbt1 fail.\n",
                        __func__, __LINE__);

		ret = rtk_nf_scan_tag(mtd, BBT2 * nf->ppb, BBT_TAG, &cnt2);
		if (ret < 0) {
			dev_err(dev, "RTK %s(%d) read bbt2 fail.\n",
                        __func__, __LINE__);
			return -1;
		} else {
			dev_info(dev, "RTK %s(%d) Load BBT from BBT2.\n",
						__func__, __LINE__);
			memcpy(nf->bbt, buffer->dataBuf + CRCLEN,
					sizeof(struct bb_table) * nf->RBA);
			sync = BBT1;
		}
		
	} else {
		memcpy(bbt, buffer->dataBuf + CRCLEN, 
					sizeof(struct bb_table) * nf->RBA);

		ret = rtk_nf_scan_tag(mtd, BBT2 * nf->ppb, BBT_TAG, &cnt2);
                if (ret < 0) {
                        dev_err(dev, "RTK %s(%d) read bbt2 fail.\n",
                        __func__, __LINE__);
			dev_info(dev, "RTK %s(%d) Load BBT from BBT1.\n",
                                                __func__, __LINE__);
			memcpy(nf->bbt, bbt, sizeof(struct bb_table) * nf->RBA);
			sync = BBT2;
                } else {
			if (cnt1 == cnt2) {
				dev_info(dev, "RTK %s(%d) Load BBT from BBT1.\n",
						__func__, __LINE__);
                                memcpy(nf->bbt, bbt,
					nf->RBA * sizeof(struct bb_table));
                        }
                        else if (cnt1 > cnt2) {
				dev_info(dev, "RTK %s(%d) Load BBT from BBT1.\n",
						__func__, __LINE__);
                                memcpy(nf->bbt, bbt,
					nf->RBA * sizeof(struct bb_table));
                                sync = BBT2;
                        }
                        else if (cnt1 < cnt2) {
				dev_info(dev, "RTK %s(%d) Load BBT from BBT2.\n",
						__func__, __LINE__);
				memcpy(nf->bbt, buffer->dataBuf + CRCLEN,
					sizeof(struct bb_table) * nf->RBA);
                                sync = BBT1;
                        }
                }
		
	}
	
	if (sync)
		rtk_nf_bbt_sync(mtd, sync);

	rtk_nf_dump_bbt(nf);

	return 0;
}

static void rtk_nf_set_bootarea_blk_count(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
#ifdef CONFIG_MTD_NAND_RTK_LEGACY
	nf->bootblk = 0;	
#else
	if (mtd->writesize == 0x800)
		nf->bootblk = 84;
        else if (mtd->writesize == 0x1000)
		nf->bootblk = 54;
#endif
}

int rtk_nf_scan_table(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	int ret = 0;

	rtk_nf_set_bootarea_blk_count(mtd);

	ret = rtk_nf_scan_shift_table(mtd);
	if (ret < 0) { 
		nf->SHIFTBLK = 0;
		nf->sbt = NULL;
	}

	return rtk_nf_scan_bbt(mtd);
}
