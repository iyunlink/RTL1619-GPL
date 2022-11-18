// SPDX-License-Identifier: GPL-2.0 OR MIT
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
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/reset.h>
#include "rtk_nand.h"

#define RTK_NAME                "rtk_nand"
#define RTK_TIMEOUT		(500000)
#define RTK_RESET_TIMEOUT	(1000000)

#define NF_CTL_ENABLE		1
#define NF_CTL_DISABLE		0
#define OOBOFF	0
#define OOBON	1
#define OOBONLY	1

static unsigned char g_rtk_nandinfo_line[128];

static void rtk_nf_set_nand_info(struct rtk_nf *nf, char *item)
{
	struct nand_flash_dev *nf_ids = nf->nf_ids;
	int ret = 0;
	char *s = NULL;
	unsigned int temp = 0;

	if ((s = strstr(item, "ds:")) != NULL) {
		ret = kstrtouint(s+3, 10, &nf_ids->chipsize);
	}
	else if ((s = strstr(item, "id:")) != NULL) {
		ret = kstrtouint(s+3, 16, &temp);
		nf_ids->mfr_id = temp & 0xFF;
		nf_ids->dev_id = (temp >> 8) & 0xFF;
		nf_ids->id[2] = (temp >> 16) & 0xFF;
		nf_ids->id[3] = (temp >> 24) & 0xFF;
	}
	else if ((s = strstr(item, "ps:")) != NULL) {
		ret = kstrtouint(s+3, 10, &nf_ids->pagesize);
	}
	else if ((s = strstr(item, "bs:")) != NULL) {
		ret = kstrtouint(s+3, 10, &nf_ids->erasesize);
	}
	else if ((s = strstr(item, "os:")) != NULL) {
		ret = kstrtou16(s+3, 10, &nf_ids->oobsize);
	}
	else if ((s = strstr(item, "t1:")) != NULL) {
		ret = kstrtouint(s+3, 10, &temp);
		nf->t1 = (unsigned char)temp;
	}
	else if ((s = strstr(item, "t2:")) != NULL) {
		ret = kstrtouint(s+3, 10, &temp);
		nf->t2 = (unsigned char)temp;
	}
	else if ((s = strstr(item, "t3:")) != NULL) {
		ret = kstrtouint(s+3, 10, &temp);
		nf->t3 = (unsigned char)temp;
	}
	else if ((s = strstr(item, "eb:")) != NULL) {
		ret = kstrtouint(s+3, 10, &temp);
		nf->ecc = (unsigned short)temp;
	}
}

static int rtk_nf_get_nand_info_from_bootcode(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct device *dev = nf->dev;
	const char * const delim = ",";
	char *sepstr = g_rtk_nandinfo_line;
	char *substr = NULL;

	if (strlen(g_rtk_nandinfo_line) == 0) {
		dev_err(dev, "No nand info got from lk!!!!\n");
		return -1;
	}

	dev_info(dev, "g_rtk_nandinfo_line:[%s]\n", sepstr);

	nf->nf_ids = kmalloc(sizeof(struct nand_flash_dev), GFP_KERNEL);
	if (!nf->nf_ids)
		return -1;

	substr = strsep(&sepstr, delim);
	do {
		rtk_nf_set_nand_info(nf, substr);
		substr = strsep(&sepstr, delim);
	} while (substr);

	sprintf(nf->flashname, "%s", "RTKNANDFLASH");
	nf->nf_ids->name = nf->flashname;
	nf->nf_ids->options = 0;
	nf->nf_ids->id_len = 4;
	if (nf->ecc == 0x1)
		nf->nf_ids->ecc.strength_ds = 12;
	else
		nf->nf_ids->ecc.strength_ds = 6;
	nf->nf_ids->ecc.step_ds = 512;
	//nf->nf_ids->onfi_timing_mode_default = 2;

	return 0;
}

static int rtk_nf_wait_down(void __iomem *regs, u64 mask, unsigned int value)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(regs, val, (val & mask) == value, 10,
					RTK_TIMEOUT);
	if (ret)
		return -EIO;

	return 0;
}

int rtk_nf_get_mapping_page(struct mtd_info *mtd, int page)
{
#if defined(CONFIG_MTD_NAND_RTK_BBM)
	return rtk_nf_get_real_page(mtd, page, BBTABLE);
#else
	return page;
#endif
}

int rtk_nf_get_physical_page(struct mtd_info *mtd, int page)
{
#if defined(CONFIG_MTD_NAND_RTK_BBM)
	return rtk_nf_get_real_page(mtd, page, SBTABLE);
#else
	return page;
#endif
}

static void rtk_nf_update_indexw(struct rtk_nf *nf, unsigned int count)
{
	struct rtk_buffer *buf = &nf->nandbuf;

	buf->index_w = buf->index_w + count;
}

static void rtk_nf_update_indexr(struct rtk_nf *nf, unsigned int count)
{
	struct rtk_buffer *buf = &nf->nandbuf;

	buf->index_r = buf->index_r + count;
}

static void rtk_nf_putbyte(struct rtk_nf *nf, unsigned char val)
{
	struct rtk_buffer *buf = &nf->nandbuf;

	nf->cmdbuf[buf->index_w] = val;
	rtk_nf_update_indexw(nf, 1);
}

static void rtk_nf_reset_cmdbuf(struct rtk_nf *nf)
{
	struct rtk_buffer *buf = &nf->nandbuf;

	memset(nf->cmdbuf, 0x0, sizeof(nf->cmdbuf));

	buf->index_r = 0;
	buf->index_w = 0;
}

static int rtk_nf_init_nandbuf(struct nand_chip *chip)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct mtd_info *mtd = mtd = nand_to_mtd(chip);
	struct rtk_buffer *buf = &nf->nandbuf;
	int ret = 0;

	buf->dataBuf = (unsigned char *)dma_alloc_coherent(nf->dev,
				mtd->writesize + mtd->oobsize, &buf->dataPhys,
				GFP_DMA | GFP_KERNEL);
	if (!buf->dataBuf) {
		dev_err(nf->dev, "alloc buf->dataBuf fail.\n");
		ret = -ENOMEM;
		goto rtk_nf_init_nandbuf_exit;
	}

	buf->tmpBuf = kmalloc(mtd->writesize + mtd->oobsize, GFP_KERNEL);
	if (!buf->tmpBuf) {
		dev_err(nf->dev, "alloc buf->tmpBuf fail.\n");
		ret = -ENOMEM;
		goto rtk_nf_init_nandbuf_exit;
	}

	buf->oobtmp = kmalloc(mtd->oobsize, GFP_KERNEL);
	if (!buf->oobtmp) {
		dev_err(nf->dev, "alloc buf->oobtmp fail.\n");
		ret = -ENOMEM;
		goto rtk_nf_init_nandbuf_exit;
	}

	buf->index_r = 0;
	buf->index_w = 0;

rtk_nf_init_nandbuf_exit:
	return ret;
}

static u8 rtk_nf_read_byte(struct nand_chip *chip)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct rtk_buffer *buf = &nf->nandbuf;
	u8 byte;

	byte = nf->cmdbuf[buf->index_r];

	rtk_nf_update_indexr(nf, 1);

	return byte;
}

static void rtk_nf_read_buf(struct nand_chip *chip, u8 *buf, int len)
{
	int i;

	for (i = 0; i < len; i++)
		buf[i] = rtk_nf_read_byte(chip);
}

static int rtk_nf_do_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				    const u8 *buffer, int page, int oobonly)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);
	void __iomem *map_base = nf->regs;
	struct device *dev = nf->dev;
	unsigned int remain_size;
	unsigned int col_addr;
	unsigned int size;
	unsigned int value;
	u8 *buf = (u8 *)buffer;
	int loop = 0;
	int ret = 0;
	int i;

	remain_size = (oobonly) ? mtd->oobsize : mtd->writesize + mtd->oobsize;
	col_addr = (oobonly) ? mtd->writesize : 0x0;

	writel(0x0, map_base + REG_RND_EN);

	/* Set command */
	writel(NF_ND_CMD_cmd(NAND_CMD_SEQIN), map_base + REG_ND_CMD);
	writel(NF_CMD2_cmd2(NAND_CMD_PAGEPROG), map_base + REG_CMD2);
	writel(NF_CMD3_cmd3(NAND_CMD_STATUS), map_base + REG_CMD3);

	/* set address */
	writel(NF_ND_PA0_page_addr0(page), map_base + REG_ND_PA0);
	writel(NF_ND_PA1_page_addr1(page >> 8), map_base + REG_ND_PA1);
	writel(NF_ND_PA2_addr_mode(1) | NF_ND_PA2_page_addr2(page >> 16),
		map_base + REG_ND_PA2);
	writel(NF_ND_PA3_page_addr3((page >> 21) & 0x7), map_base + REG_ND_PA3);
	writel(0xff & (col_addr >> 0), map_base + REG_ND_CA0);
	writel(0xff & (col_addr >> 8), map_base + REG_ND_CA1);

	writel(NF_AUTO_TRIG_auto_trig(1) | NF_AUTO_TRIG_spec_auto_case(1) |
		NF_AUTO_TRIG_auto_case(1),
		map_base + REG_AUTO_TRIG);
	if ((ret = rtk_nf_wait_down(map_base + REG_AUTO_TRIG, 0x80, 0x0)) != 0)
		return -1;
	udelay(1000);

	while (remain_size > 0) {
		size = (remain_size >= 0x200) ? 0x200 : remain_size;
		if (size != 0x200) {
			buf = chip->oob_poi;
			loop = 0;
		}

		writel(0x30, map_base + REG_SRAM_CTL);
		for (i = 0; i < (size/4); i++) {
			value = buf[loop*0x200 + i*4] |
				buf[loop*0x200 + i*4 + 1] << 8 |
				buf[loop*0x200 + i*4 + 2] << 16 |
				buf[loop*0x200 + i*4 + 3] << 24;

			writel(value, map_base + i*4);
		}
		writel(0x0, map_base + REG_SRAM_CTL);

		writel(size & 0xff, map_base + REG_DATA_TL0);
		writel(0x80 | ((size >> 8) & 0x3f), map_base + REG_DATA_TL1);

		/* Set PP */
		writel(NF_READ_BY_PP_read_by_pp(0), map_base + REG_READ_BY_PP);
		writel(NF_PP_CTL1_pp_start_addr(0) | NF_PP_CTL0_pp_enable(1),
			map_base + REG_PP_CTL0);
		writel(0x0, map_base + REG_PP_CTL1);

		/* Enable XFER mode */
		writel(0x80 | 0x3, map_base + REG_ND_CTL);
		ret = readl_poll_timeout_atomic(map_base + REG_ND_CTL, value,
						(value & 0x80) == 0x0, 10,
						RTK_TIMEOUT);
		if (ret) {
			dev_err(dev, "RTK %s(%d) timeout.\n", __func__, __LINE__);
			return -1;
		}

		remain_size -= size;
		loop++;
	}

	writel(NF_ND_CMD_cmd(NAND_CMD_PAGEPROG), map_base + REG_ND_CMD);
	writel(0x80, map_base + REG_ND_CTL);
	ret = readl_poll_timeout_atomic(map_base + REG_ND_CTL, value,
					(value & 0x80) == 0x0, 10,
					RTK_TIMEOUT);
	if (ret) {
		dev_err(dev, "RTK %s(%d) timeout.\n", __func__, __LINE__);
		return -1;
	}

	if ((ret = rtk_nf_wait_down(map_base + REG_ND_CTL,
				0x40, 0x40)) != 0)
		return -1;

	writel(0xd, map_base + REG_POLL_FSTS);
	if ((ret = rtk_nf_wait_down(map_base + REG_POLL_FSTS,
				0x1, 0x0)) != 0)
		return -1;

	if (readl(map_base + REG_ND_DAT) & 0x1) {
		dev_err(dev, "RTK %s(%d) write page %d fail.\n",
			__func__, __LINE__, page);
		return -1;
	}
	return 0;
}

static int rtk_nf_do_write_page_ecc(struct mtd_info *mtd, struct nand_chip *chip,
				    const u8 *buf, int page, int mode)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);
	void __iomem *base = nf->regs;
	unsigned int dram_sa, dma_len, spare_dram_sa;
	struct rtk_buffer *buffer = &nf->nandbuf;
	dma_addr_t dataPhy = buffer->dataPhys;
	dma_addr_t oobPhy = buffer->dataPhys + mtd->writesize;
	int ret = 0;
	int i;

	chip->pagecache.page = -1;

	if (buf != buffer->dataBuf && !((u32)buf & 0xF)) {
		dataPhy = dma_map_single(nf->dev, (u8 *)buf, mtd->writesize,
					 DMA_TO_DEVICE);
		ret = dma_mapping_error(nf->dev, dataPhy);
		if (ret) {
			dev_err(nf->dev, "data dma mapping error\n");
			return -EINVAL;
		}
		dma_sync_single_for_device(nf->dev, dataPhy, mtd->writesize,
					   DMA_TO_DEVICE);
	}
	else {
		if (buf != buffer->dataBuf) {
			memcpy(buffer->dataBuf, buf, mtd->writesize);
		}
	}

	oobPhy = dma_map_single(nf->dev, (u8 *)chip->oob_poi,
				mtd->oobsize, DMA_TO_DEVICE);
	ret = dma_mapping_error(nf->dev, oobPhy);
	if (ret) {
		dev_err(nf->dev, "oob dma mapping error\n");
		return -EINVAL;
	}

	dma_sync_single_for_device(nf->dev, oobPhy, mtd->oobsize,
				   DMA_TO_DEVICE);


	//writel(0x0, base + REG_READ);
	writel(NF_DATA_TL0_length0(0), base + REG_DATA_TL0);

	writel(0x1, base + REG_RND_EN);
	writel(0x85, base + REG_RND_CMD1);
	writel(0x0, base + REG_RND_DATA_STR_COL_H);
	writel((mtd->writesize >> 8), base + REG_RND_SPR_STR_COL_H);
	writel((mtd->writesize & 0xff), base + REG_RND_SPR_STR_COL_L);
	writel(NF_DATA_TL1_length1(2), base + REG_DATA_TL1);

	writel(NF_PAGE_LEN_page_len(mtd->writesize >> 9), base + REG_PAGE_LEN);

	/* Set PP */
	writel(NF_READ_BY_PP_read_by_pp(0), base + REG_READ_BY_PP);
	writel(NF_PP_CTL1_pp_start_addr(0), base + REG_PP_CTL1);
	writel(0x0, base + REG_PP_CTL0);

	/* Set command */
	writel(NF_ND_CMD_cmd(NAND_CMD_SEQIN), base + REG_ND_CMD);
	writel(NF_CMD2_cmd2(NAND_CMD_PAGEPROG), base + REG_CMD2);
	writel(NF_CMD3_cmd3(NAND_CMD_STATUS), base + REG_CMD3);

	/* set address */
	writel(NF_ND_PA0_page_addr0(page), base + REG_ND_PA0);
	writel(NF_ND_PA1_page_addr1(page >> 8), base + REG_ND_PA1);
	writel(NF_ND_PA2_addr_mode(1) | NF_ND_PA2_page_addr2(page >> 16),
		base + REG_ND_PA2);
	writel(NF_ND_PA3_page_addr3((page >> 21) & 0x7), base + REG_ND_PA3);
	writel(0x0, base + REG_ND_CA0);
	writel(0x0, base + REG_ND_CA1);

	/* set ECC */
	if (mode == RAW)
		writel(NF_MULTI_CHNL_MODE_edo(1) | NF_MULTI_CHNL_MODE_ecc_pass(1) |
		       NF_MULTI_CHNL_MODE_ecc_no_check(0),
		       base + REG_MULTI_CHNL_MODE);
	else
		writel(NF_MULTI_CHNL_MODE_edo(1), base + REG_MULTI_CHNL_MODE);

	writel(NF_ECC_STOP_ecc_n_stop(1), base + REG_ECC_STOP);
	writel(nf->ecc, base + REG_ECC_SEL);

	dram_sa = ((uintptr_t)dataPhy >> 3);
	dma_len = mtd->writesize >> 9;
	writel(NF_DMA_CTL1_dram_sa(dram_sa), base + REG_DMA_CTL1);
	writel(NF_DMA_CTL2_dma_len(dma_len), base + REG_DMA_CTL2);

	if (mode == RAW)
		writel(0x0, base + REG_SPR_DDR_CTL);
	else {
		spare_dram_sa = ((uintptr_t)oobPhy >> 3);
		writel(0x60000000 | NF_SPR_DDR_CTL_spare_dram_sa(spare_dram_sa),
			base + REG_SPR_DDR_CTL);
	}
	writel(NF_DMA_CTL3_ddr_wr(0)|NF_DMA_CTL3_dma_xfer(1),
		base + REG_DMA_CTL3);

	if (mode == RAW) {
		/* fill SRAM = 0xff */
		writel(0x32, base + REG_SRAM_CTL);
		for (i = 0; i < (mtd->oobsize/4); i++) {
			 writel(0xFFFFFFFF, base + i*4);
		}
		writel(0x0, base + REG_SRAM_CTL);
	}

	writel(NF_AUTO_TRIG_auto_trig(1) | NF_AUTO_TRIG_spec_auto_case(0) |
		NF_AUTO_TRIG_auto_case(1),
		base + REG_AUTO_TRIG);

	if ((ret = rtk_nf_wait_down(base + REG_AUTO_TRIG, 0x80, 0x0)) != 0)
		goto rtk_nf_do_write_page_exit;

	if ((ret = rtk_nf_wait_down(base + REG_DMA_CTL3, 0x1, 0x0)) != 0)
		goto rtk_nf_do_write_page_exit;

	writel(NF_POLL_FSTS_bit_sel(6) | NF_POLL_FSTS_trig_poll(1),
			base + REG_POLL_FSTS);
	if ((ret = rtk_nf_wait_down(base + REG_POLL_FSTS, 0x1, 0x0)) != 0)
		goto rtk_nf_do_write_page_exit;

	if ((ret = rtk_nf_wait_down(base + REG_ND_CTL, 0x40, 0x40)) != 0)
		goto rtk_nf_do_write_page_exit;

	if (readl(base + REG_ND_DAT) & 0x1)
		ret = -1;

rtk_nf_do_write_page_exit:
	if (buf != buffer->dataBuf && !((u32)buf & 0xF)) {
		dma_sync_single_for_cpu(nf->dev, dataPhy, mtd->writesize,
					   DMA_TO_DEVICE);

		dma_unmap_single(nf->dev, dataPhy, mtd->writesize,
				 DMA_TO_DEVICE);
	}

	dma_sync_single_for_cpu(nf->dev, oobPhy, mtd->oobsize, DMA_TO_DEVICE);
	dma_unmap_single(nf->dev, oobPhy, mtd->oobsize, DMA_TO_DEVICE);

	return ret;
}

static int rtk_nf_do_write_page(struct mtd_info *mtd, struct nand_chip *chip,
				int page, const u8 *buf, int oob_on,
				int access_mode)
{
	int ret = 0;

	ret = rtk_nf_do_write_page_ecc(mtd, chip, buf, page, access_mode);
	if (ret)
		return ret;

	if (access_mode == RAW)
		ret = rtk_nf_do_write_page_raw(mtd, chip, buf, page, OOBONLY);

	return ret;
}

static int rtk_nf_do_read_page(struct mtd_info *mtd, int page, u8 *p, int oob_on, int raw);
static int rtk_nf_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			const u8 *buf, int page, int oob_on, int access_mode)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct rtk_buffer *buffer = &nf->nandbuf;
	int phy_page = rtk_nf_get_physical_page(mtd, page);
	int real_page;
	int ret;
	int src_blk;

	chip->pagecache.page = -1;

#if defined(CONFIG_MTD_NAND_RTK_BBM)
	src_blk = phy_page / nf->ppb;
rtk_nf_write_page_retry:
#endif
	real_page = rtk_nf_get_mapping_page(mtd, phy_page);

	memcpy(buffer->dataBuf, buf, mtd->writesize);
	memcpy(buffer->dataBuf + mtd->writesize, chip->oob_poi, mtd->oobsize);
	ret = rtk_nf_do_write_page(mtd, chip, real_page, buf, oob_on, access_mode);
#if defined(CONFIG_MTD_NAND_RTK_BBM)
	if (ret < 0) {
		ret = rtk_nf_bb_handle(mtd, src_blk, real_page, 1, NFWRITE);
		if (ret == 0)
			goto rtk_nf_write_page_retry;
	}
#endif
	return ret;
}

static int rtk_nf_write_page_raw(struct nand_chip *chip, const u8 *buf,
				 int oob_on, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	return rtk_nf_write_page(mtd, chip, buf, page, oob_on, RAW);
}

static int rtk_nf_write_page_hwecc(struct nand_chip *chip, const u8 *buf,
				   int oob_on, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	return rtk_nf_write_page(mtd, chip, buf, page, oob_on, ECC);
}

static int rtk_nf_write_oob_page(struct mtd_info *mtd, struct nand_chip *chip,
				int page, int access_mode)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct rtk_buffer *buffer = &nf->nandbuf;
	int phy_page = rtk_nf_get_physical_page(mtd, page);
	int real_page = rtk_nf_get_mapping_page(mtd, phy_page);
	int ret;

	chip->pagecache.page = -1;

	memset(buffer->tmpBuf, 0xff, mtd->writesize);
	memcpy(buffer->tmpBuf, chip->oob_poi, mtd->oobsize);

	ret = rtk_nf_do_read_page(mtd, real_page, buffer->dataBuf, OOBON,
				  access_mode);
	if (ret >= 0){
		memcpy(chip->oob_poi, buffer->tmpBuf, mtd->oobsize);
		memcpy(buffer->tmpBuf, buffer->dataBuf, mtd->writesize);
	}
	return rtk_nf_write_page(mtd, chip, buffer->tmpBuf, page, OOBON,
			access_mode);
}

static int rtk_nf_write_oob_raw(struct nand_chip *chip, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	return rtk_nf_write_oob_page(mtd, chip, page, RAW);
}

static int rtk_nf_write_oob_hwecc(struct nand_chip *chip, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	return rtk_nf_write_oob_page(mtd, chip, page, ECC);
}

static void rtk_read_oob_from_SRAM(struct mtd_info *mtd, int phase)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct rtk_buffer *buffer = &nf->nandbuf;
	unsigned char *oobbuf = buffer->dataBuf + mtd->writesize;
	void __iomem *map_base = nf->regs;
	unsigned char buf[256];
	unsigned int val = 0x0;
	int eccstep_size = (nf->ecc == 0x1) ? 32 : 16;
	int ecc_size = (nf->ecc == 0x1) ? 20 : 10;
	int eccstep = (phase == 1) ? 4 : (mtd->writesize / SZ_512);
	int i;

	writel(0x0, map_base + REG_READ_BY_PP);
	writel(0x30 | 0x02, map_base + REG_SRAM_CTL);

	memset(oobbuf, 0xFF, mtd->oobsize);
	memset(buf, 0xFF, mtd->oobsize);

	for (i = 0; i < (mtd->oobsize / 4); i++) {
		val = readl(map_base + (i*4));

		buf[(i*4)] = val & 0xff;
		buf[(i*4)+1] = (val >> 8) & 0xff;
		buf[(i*4)+2] = (val >> 16) & 0xff;
		buf[(i*4)+3] = (val >> 24) & 0xff;
	}

	for (i = 0; i < eccstep; i++)
		memcpy(oobbuf+(i*(ecc_size + 6)), buf+(i*eccstep_size),
			(ecc_size + 6));

	writel(0x0, map_base + REG_SRAM_CTL);
	writel(0x80, map_base + REG_READ_BY_PP);

	return;
}

static void rtk_nf_enable_io_mode(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	void __iomem *map_base = nf->regs;

	writel(readl(map_base + REG_READ) | 0x1, map_base + REG_READ);

	writel(0x0, map_base + REG_TIME_PARA1);
	writel(0x0, map_base + REG_TIME_PARA2);
	writel(0x0, map_base + REG_TIME_PARA3);

	return;
}

static void rtk_nf_disable_io_mode(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	void __iomem *map_base = nf->regs;

	writel(nf->t1, map_base + REG_TIME_PARA1);
	writel(nf->t2, map_base + REG_TIME_PARA2);
	writel(nf->t3, map_base + REG_TIME_PARA3);

	return;
}


static int rtk_nf_do_read_page_raw(struct mtd_info *mtd, int page, u8 *buffer,
				   int oobonly)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct device *dev = nf->dev;
	void __iomem *map_base = nf->regs;
	unsigned int remain_size;
	unsigned int col_addr;
	unsigned int size = 0;
	u8 *buf = buffer;
	int loop = 0;
	u32 val;
	int ret;
	int i;

	remain_size = (oobonly) ? mtd->oobsize : mtd->writesize + mtd->oobsize;
	col_addr = (oobonly) ? mtd->writesize : 0x0;

	writel(0x0, map_base + REG_RND_EN);
	writel(0x0, map_base + REG_ND_CMD);
	writel(0x30, map_base + REG_CMD2);
	writel(0x70, map_base + REG_CMD3);

	/* set PA and CA */
	writel(0xff & page, map_base + REG_ND_PA0);
	writel(0xff & (page >> 8), map_base + REG_ND_PA1);
	writel((0x1 << 5) | (0x1f & (page >> 16)), map_base + REG_ND_PA2);
	writel(((0x7 & (page >> 21)) << 5) & 0x000000E0, map_base + REG_ND_PA3);
	writel(0xff & (col_addr >> 0), map_base + REG_ND_CA0);
	writel(0xff & (col_addr >> 8), map_base + REG_ND_CA1);

	/* Enable Auto mode */
	writel(0x80 | 0x8 | (0x7 & 2), map_base + REG_AUTO_TRIG);

	if ((ret = rtk_nf_wait_down(map_base + REG_AUTO_TRIG, 0x80, 0x0)) != 0)
		return -1;

	if ((ret = rtk_nf_wait_down(map_base + REG_ND_CTL, 0x40, 0x40)) != 0)
		return -1;

	while (remain_size > 0) {
		size = (remain_size >= 0x200) ? 0x200 : remain_size;
		if (size != 0x200) {
			buf = chip->oob_poi;
			loop = 0;
		}

		writel(0xff & size, map_base + REG_DATA_TL0);
		writel(0x80 | ((size >> 8) & 0x3f), map_base + REG_DATA_TL1);
		writel(0x1, map_base + REG_PAGE_LEN);

		/* set PP */
		writel(0x0, map_base + REG_READ_BY_PP);
		writel(0x1, map_base + REG_PP_CTL0);
		writel(0x0, map_base + REG_PP_CTL1);

		/* Enable XFER mode */
		writel(0x80 | 0x4, map_base + REG_ND_CTL);
		ret = readl_poll_timeout_atomic(map_base + REG_ND_CTL, val,
						(val & 0x80) == 0x0, 10,
						RTK_TIMEOUT);
		if (ret) {
			dev_err(dev, "RTK %s(%d) timeout.\n", __func__, __LINE__);
			return -1;
		}

		writel(0x30, map_base + REG_SRAM_CTL);
		for (i = 0; i < (size/4); i++) {
			val = readl(map_base + i*4);
			buf[(loop*0x200) + i*4] = val & 0xff;
			buf[(loop*0x200) + i*4 + 1] = (val>>8) & 0xff;
			buf[(loop*0x200) + i*4 + 2] = (val>>16) & 0xff;
			buf[(loop*0x200) + i*4 + 3] = (val>>24) & 0xff;
		}
		writel(0x0, map_base + REG_SRAM_CTL);

		remain_size -= size;
		loop++;
	}

	return 0;
}

static int rtk_nf_do_read_page_ecc(struct mtd_info *mtd, int page, u8 *buf,
				   int oob_on, int mode, int phase)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	void __iomem *map_base = nf->regs;
	struct rtk_buffer *buffer = &nf->nandbuf;
	dma_addr_t dataPhy = buffer->dataPhys;
	unsigned int ecc_threshold = 0;
	unsigned int eccNum = 0;
	unsigned int blank_check = 0;
	unsigned int access_page_len = 0;
	int ret;

	access_page_len = (phase == 1) ? SZ_2K : mtd->writesize;
	ecc_threshold = (nf->ecc == 0x1) ? 10 : 4;

	if (buf != buffer->dataBuf && !((u32)buf & 0xF)) {
		dataPhy = dma_map_single(nf->dev, (u8 *)buf, access_page_len, DMA_FROM_DEVICE);
		ret = dma_mapping_error(nf->dev, dataPhy);
		if (ret) {
			dev_err(nf->dev, "dma mapping error\n");
			return -EINVAL;
		}

		dma_sync_single_for_device(nf->dev, dataPhy, access_page_len,
					   DMA_FROM_DEVICE);
	}

	rtk_nf_enable_io_mode(mtd);

	writel(0x1, map_base + REG_RND_EN);
	writel(0x5, map_base + REG_RND_CMD1);
	writel(0xe0, map_base + REG_RND_CMD2);
	writel(0x0, map_base + REG_RND_DATA_STR_COL_H);
	writel((mtd->writesize >> 8), map_base + REG_RND_SPR_STR_COL_H);
	writel((mtd->writesize & 0xff), map_base + REG_RND_SPR_STR_COL_L);

	/* set PA and CA */
	writel(0xff & page, map_base + REG_ND_PA0);
	writel(0xff & (page >> 8), map_base + REG_ND_PA1);
	writel((0x1 << 5) | (0x1f & (page >> 16)), map_base + REG_ND_PA2);
	writel(((0x7 & (page >> 21)) << 5) & 0x000000E0, map_base + REG_ND_PA3);
	writel(0x0, map_base + REG_ND_CA0);
	writel(0x0, map_base + REG_ND_CA1);

	/* set transfer size */
	//writel(0xff & 512, map_base + REG_DATA_TL0);
	writel(0x0, map_base + REG_DATA_TL0);
	writel(0x80 | (2 & 0x3f), map_base + REG_DATA_TL1);
	writel(access_page_len >> 9, map_base + REG_PAGE_LEN);

	writel(((uintptr_t)dataPhy >> 3) & 0x0FFFFFFF, map_base + REG_DMA_CTL1);
	writel((access_page_len >> 9) & 0x0000FFFF, map_base + REG_DMA_CTL2);

#if 0
	writel(0x60000000 | ((uintptr_t)(dataPhy + mtd->writesize) >> 3),
		map_base + REG_SPR_DDR_CTL);
#endif
	writel(0x0, map_base + REG_SPR_DDR_CTL);

	/* set PP */
	writel(0x80, map_base + REG_READ_BY_PP);
	writel(0x0, map_base + REG_PP_CTL1);
	writel(0x0, map_base + REG_PP_CTL0);

	/* enable blank check */
	writel(ecc_threshold & 0xF, map_base + REG_BLANK_ZERO_NUM);
	writel(NF_BLANK_CHK_blank_ena(1) | NF_BLANK_CHK_read_ecc_xnor_ena(0),
						map_base + REG_BLANK_CHK);

	/* set command */
	writel(NAND_CMD_READ0, map_base + REG_ND_CMD);
	writel(NAND_CMD_READSTART, map_base + REG_CMD2);
	writel(NAND_CMD_STATUS, map_base + REG_CMD3);

	/* Set ECC */
	if (mode == RAW)
		writel(NF_MULTI_CHNL_MODE_edo(1) |
			NF_MULTI_CHNL_MODE_ecc_pass(1) |
			NF_MULTI_CHNL_MODE_ecc_no_check(1),
			map_base + REG_MULTI_CHNL_MODE);
	else
		writel(0x20, map_base + REG_MULTI_CHNL_MODE);

	writel(0x80, map_base + REG_ECC_STOP);
	writel(nf->ecc, map_base + REG_ECC_SEL);

	writel(0x2 | 0x1, map_base + REG_DMA_CTL3);

	/* Enable Auto mode */
	writel(0x80 | (0x7 & 2), map_base + REG_AUTO_TRIG);

	if ((ret = rtk_nf_wait_down(map_base + REG_AUTO_TRIG, 0x80, 0x0)) != 0)
		goto rtk_nf_do_read_page_ecc_exit;

	if ((ret = rtk_nf_wait_down(map_base + REG_DMA_CTL3, 0x1, 0x0)) != 0)
		goto rtk_nf_do_read_page_ecc_exit;

	if (oob_on && (mode == ECC))
		rtk_read_oob_from_SRAM(mtd, phase);

	blank_check = readl(map_base + REG_BLANK_CHK);
	if (blank_check & 0x2) {
		ret = 1;
	} else if (readl(map_base + REG_ND_ECC) & 0x8) {
		if (blank_check & 0x8) {
			dev_err(nf->dev, "RTK %s(%d) un-correct ecc error ... page:[%d]\n",
							__func__, __LINE__, page);
			ret = -1;
		} else {
			memset(buf, 0xFF, mtd->writesize);
			memset(buffer->dataBuf + mtd->writesize, 0xFF,
				mtd->oobsize);
			ret = 1;
		}
	} else {
		if (readl(map_base + REG_ND_ECC) & 0x04) {
			eccNum = readl(map_base + REG_MAX_ECC_NUM) & 0xff;
			if (eccNum > ecc_threshold) {
				dev_warn(nf->dev, "RTK %s(%d) ecc over threshold.\n",
						__func__, __LINE__);

				ret = 2;
			} else
				ret = 0;
		}
	}

rtk_nf_do_read_page_ecc_exit:
	writel(NF_BLANK_CHK_blank_ena(1) | NF_BLANK_CHK_read_ecc_xnor_ena(0),
		map_base + REG_BLANK_CHK);

	rtk_nf_disable_io_mode(mtd);

	if (buf != buffer->dataBuf && !((u32)buf & 0xF)) {
		dma_sync_single_for_cpu(nf->dev, dataPhy, access_page_len,
					DMA_FROM_DEVICE);
		dma_unmap_single(nf->dev, dataPhy, access_page_len,
				 DMA_FROM_DEVICE);
		memcpy(chip->oob_poi, buffer->dataBuf + mtd->writesize,
		       mtd->oobsize);
	} else {
		if (buf != buffer->dataBuf)
			memcpy(buf, buffer->dataBuf, access_page_len);
	}

	return ret;
}

static int rtk_nf_do_read_page(struct mtd_info *mtd, int page, u8 *p, int oob_on,
			       int mode)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct rtk_buffer *buffer = &nf->nandbuf;
	int oobtmp_len = (nf->ecc == 0x1) ? (26*4) : (16*4);
	int ret = 0;

	if (oob_on && mtd->writesize == SZ_4K) {
		memset(buffer->oobtmp, 0xFF, mtd->oobsize);

		ret = rtk_nf_do_read_page_ecc(mtd, page, p, oob_on, mode, 1);
		if (ret < 0)
			goto rtk_nf_do_read_page_exit;

		memcpy(buffer->oobtmp, buffer->dataBuf + mtd->writesize,
			oobtmp_len);

		ret = rtk_nf_do_read_page_ecc(mtd, page, p, oob_on, mode, 2);
		if (ret < 0)
			goto rtk_nf_do_read_page_exit;

		memcpy(buffer->oobtmp + oobtmp_len,
			buffer->dataBuf + mtd->writesize, oobtmp_len);
		memcpy(buffer->dataBuf + mtd->writesize,
			buffer->oobtmp, mtd->oobsize);
	} else {
		ret = rtk_nf_do_read_page_ecc(mtd, page, p, oob_on, mode, 0);
		if (ret < 0)
			goto rtk_nf_do_read_page_exit;
	}

	if (mode == RAW)
		ret = rtk_nf_do_read_page_raw(mtd, page, p, OOBONLY);

rtk_nf_do_read_page_exit:
	return ret;
}

static int rtk_nf_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				u8 *p, int oob_on, int page, int access_mode)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);
	int phy_page = rtk_nf_get_physical_page(mtd, page);
	int real_page = rtk_nf_get_mapping_page(mtd, phy_page);
	int ret;
#if defined(CONFIG_MTD_NAND_RTK_BBM)
	int src_blk = phy_page / (int)nf->ppb;
#endif /* CONFIG_MTD_NAND_RTK_BBM */

	ret = rtk_nf_do_read_page(mtd, real_page, p, oob_on, access_mode);
#if defined(CONFIG_MTD_NAND_RTK_BBM)
	src_blk = phy_page / (int)nf->ppb;
	if (ret < 0)
		ret = rtk_nf_bb_handle(mtd, src_blk, real_page, 0, NFREAD);
	else if (ret == 2) /* ecc bit over threshold */
		ret = rtk_nf_bb_handle(mtd, src_blk, real_page, 1, NFREAD);
#endif /* CONFIG_MTD_NAND_RTK_BBM */
	return ret;
}

static int rtk_nf_read_page_raw(struct nand_chip *chip, u8 *p,
				int oob_on, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	return rtk_nf_read_page(mtd, chip, p, oob_on, page, RAW);
}

static int rtk_nf_read_page_hwecc(struct nand_chip *chip, u8 *p,
				  int oob_on, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	return rtk_nf_read_page(mtd, chip, p, oob_on, page, ECC);
}

static int rtk_nf_read_oob_page(struct mtd_info *mtd, struct nand_chip *chip,
				int page, int access_mode)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);
	int phy_page = rtk_nf_get_physical_page(mtd, page);
	int real_page = rtk_nf_get_mapping_page(mtd, phy_page);
	int ret;
#if defined(CONFIG_MTD_NAND_RTK_BBM)
	int src_blk = phy_page / (int)nf->ppb;
#endif /* CONFIG_MTD_NAND_RTK_BBM */

	ret = rtk_nf_do_read_page(mtd, real_page, chip->data_buf, OOBON,
				  access_mode);
#if defined(CONFIG_MTD_NAND_RTK_BBM)
	if (ret < 0)
		ret = rtk_nf_bb_handle(mtd, src_blk, real_page, 0, NFREAD);
	else if (ret == 2) /* ecc bit over threshold */
		ret = rtk_nf_bb_handle(mtd, src_blk, real_page, 1, NFREAD);
#endif /* CONFIG_MTD_NAND_RTK_BBM */
	return ret;
}

static int rtk_nf_read_oob_raw(struct nand_chip *chip, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	return rtk_nf_read_oob_page(mtd, chip, page, RAW);
}

static int rtk_nf_read_oob_hwecc(struct nand_chip *chip, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	return rtk_nf_read_oob_page(mtd, chip, page, ECC);
}

static int rtk_nf_do_erase_block(struct nand_chip *chip, int page)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct device *dev = nf->dev;
	void __iomem *base = nf->regs;
	int ret = 0;

	writel(NF_MULTI_CHNL_MODE_no_wait_busy(1) | NF_MULTI_CHNL_MODE_edo(1),
		base + REG_MULTI_CHNL_MODE);

	writel(NF_ND_CMD_cmd(NAND_CMD_ERASE1), base + REG_ND_CMD);
	writel(NF_CMD2_cmd2(NAND_CMD_ERASE2), base + REG_CMD2);
	writel(NF_CMD3_cmd3(NAND_CMD_STATUS), base + REG_CMD3);

	writel(NF_ND_PA0_page_addr0(page), base + REG_ND_PA0);
	writel(NF_ND_PA1_page_addr1(page>>8), base + REG_ND_PA1);
	writel(NF_ND_PA2_addr_mode(0x04) | NF_ND_PA2_page_addr2(page >> 16),
		base + REG_ND_PA2);
	writel(NF_ND_PA3_page_addr3((page >> 21) & 0x7), base + REG_ND_PA3);

	writel(NF_AUTO_TRIG_auto_trig(1) | NF_AUTO_TRIG_spec_auto_case(1)|
		NF_AUTO_TRIG_auto_case(2), base + REG_AUTO_TRIG);
	if ((ret = rtk_nf_wait_down(base + REG_AUTO_TRIG, 0x80, 0x0)) != 0)
		goto rtk_nf_do_erase_block_exit;

	writel(NF_POLL_FSTS_bit_sel(6) | NF_POLL_FSTS_trig_poll(1), 
		base + REG_POLL_FSTS);
	if ((ret = rtk_nf_wait_down(base + REG_POLL_FSTS, 0x1, 0x0)) != 0)
		goto rtk_nf_do_erase_block_exit;

	if ((ret = rtk_nf_wait_down(base + REG_ND_CTL, 0x40, 0x40)) != 0)
		goto rtk_nf_do_erase_block_exit;

	if (readl(base + REG_ND_DAT) & 0x1) {
		dev_err(dev, "RTK %s(%d) erase fail.\n", __func__, __LINE__);
		ret = -1;
	}

rtk_nf_do_erase_block_exit:
	return ret;
}

static int rtk_nf_erase_block(struct nand_chip *chip, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	int phy_page = rtk_nf_get_physical_page(mtd, page);
	int real_page;
	int src_blk;
	int ret;

#if defined(CONFIG_MTD_NAND_RTK_BBM)
	src_blk = phy_page / (int)nf->ppb;
rtk_nf_erase_block_retry:
#endif /*CONFIG_MTD_NAND_RTK_BBM*/
	real_page = rtk_nf_get_mapping_page(mtd, phy_page);

	ret = rtk_nf_do_erase_block(chip, real_page);
#if defined(CONFIG_MTD_NAND_RTK_BBM)
	if (ret < 0) {
		ret = rtk_nf_bb_handle(mtd, src_blk, real_page, 0, NFERASE);
		if (ret == 0)
			goto rtk_nf_erase_block_retry;
	}
#endif /* CONFIG_MTD_NAND_RTK_BBM */
	return ret;
}

static int rtk_nf_block_bad(struct nand_chip *chip, loff_t ofs)
{
#ifndef CONFIG_MTD_NAND_RTK_BBM
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct nand_memory_organization *memorg = &chip->base.memorg;
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct rtk_buffer *buffer = &nf->nandbuf;
	u32 ppb = memorg->pages_per_eraseblock;
	u64 blk = (u64)ofs;
	int i;

	do_div(blk, mtd->erasesize);

	/* check 1st & 2nd page */
	for (i = 0; i < 2; i++) {
		if (chip->ecc.read_oob(chip, blk*ppb + i) < 0)
			return -1;

		if (*(buffer->dataBuf + mtd->writesize) != 0xff)
			return -1;
	}
#endif /*CONFIG_MTD_NAND_RTK_BBM*/
	return 0;
}

static int rtk_nf_block_markbad(struct nand_chip *chip, loff_t ofs)
{
#ifndef CONFIG_MTD_NAND_RTK_BBM
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct rtk_buffer *buffer = &nf->nandbuf;
	unsigned int page;
	int i;

	page = (unsigned int)(ofs >> chip->page_shift) & chip->pagemask;

	memset(buffer->dataBuf, 0x00, mtd->writesize + mtd->oobsize);

	/* mark 1st & 2nd page */
	for (i = 0; i < 2; i++)
		rtk_nf_do_write_page(mtd, chip, page + i, buffer->dataBuf,
				     OOBON, ECC);
#endif /* CONFIG_MTD_NAND_RTK_BBM */
	return 0;
}

static void rtk_nf_read_id(struct nand_chip *chip)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);
	void __iomem *base = nf->regs;
	int ret, i;
	int id_chain;

	writel(6, base + REG_DATA_TL0);
	writel(0x80, base + REG_DATA_TL1);

	/* Set PP */
	writel(0x0, base + REG_READ_BY_PP);
	writel(0x01, base + REG_PP_CTL0);
	writel(0x0, base + REG_PP_CTL1);

	/* Set command */
	writel(0x90, base + REG_ND_CMD);
	writel(0x80, base + REG_ND_CTL);
	if ((ret = rtk_nf_wait_down(base + REG_ND_CTL, 0x80, 0x0)) != 0)
		return;

	/* Set address */
	writel(0x0, base + REG_ND_PA0);
	writel(0x0, base + REG_ND_PA1);
	writel(0x7 << 5, base + REG_ND_PA2);
	writel(0x81, base + REG_ND_CTL);
	if ((ret = rtk_nf_wait_down(base + REG_ND_CTL, 0x80, 0x0)) != 0)
		return;

	/* Enable XFER mode */
	writel(0x84, base + REG_ND_CTL);
	if ((ret = rtk_nf_wait_down(base + REG_ND_CTL, 0x80, 0x0)) != 0)
		return;

	/* reset PP */
	writel(0x2, base + REG_PP_CTL0);

	/* Move data to DRAM from SRAM */
	writel(0x30, base + REG_SRAM_CTL);

	id_chain = readl(base + REG_ND_PA0);
	for (i = 0; i < 4; i++)
		rtk_nf_putbyte(nf, (id_chain >> (8*i)) & 0xff);

	id_chain = readl(base + REG_ND_PA1);
	for (i = 0; i < 4; i++)
		rtk_nf_putbyte(nf, (id_chain >> (8*i)) & 0xff);

	writel(0x0, base + REG_SRAM_CTL);
}

static void rtk_nf_status(struct nand_chip *chip)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);
	struct rtk_buffer *buf = &nf->nandbuf;
	u32 val = 0x0;

	val |= NAND_STATUS_WP;
	val |= NAND_STATUS_READY;

	nf->cmdbuf[buf->index_w] = val;
	rtk_nf_update_indexw(nf, 1);
}

static int rtk_nf_ooblayout_ecc(struct mtd_info *mtd, int section,
				struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->offset = 6;
	oobregion->length = mtd->oobsize - oobregion->offset;

	return 0;
}

static int rtk_nf_ooblayout_free(struct mtd_info *mtd, int section,
				struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->offset = 0;
	oobregion->length = 6;

	return 0;
}

static const struct mtd_ooblayout_ops rtk_nf_ooblayout_ops = {
	.ecc = rtk_nf_ooblayout_ecc,
	.free = rtk_nf_ooblayout_free,
};

static void rtk_nf_update_ecc_info(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	if (chip->ecc.engine_type != NAND_ECC_ENGINE_TYPE_ON_HOST)
		return;

	if (mtd->writesize == SZ_2K)
		mtd->ecc_strength = 6;
	else if (mtd->writesize == SZ_4K)
		mtd->ecc_strength = 12;

	chip->ecc.steps	= 4;
	chip->ecc.size	= 512;
	chip->ecc.bytes	= (mtd->writesize == SZ_2K) ? 10 : 20;
	chip->ecc.strength = mtd->ecc_strength;

	mtd_set_ooblayout(mtd, &rtk_nf_ooblayout_ops);
}

static int rtk_nf_wait(struct nand_chip *chip)
{
	/* nop */
	return 0;
}

static void rtk_nf_command(struct nand_chip *chip, unsigned int command,
				int column, int page_addr)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);

	rtk_nf_reset_cmdbuf(nf);

	switch (command) {
	case NAND_CMD_RESET:
		break;

	case NAND_CMD_READID:
		rtk_nf_read_id(chip);
		break;

	case NAND_CMD_READ0:
		break;

	case NAND_CMD_STATUS:
		rtk_nf_status(chip);
		break;

	case NAND_CMD_ERASE1:
		rtk_nf_erase_block(chip, page_addr);
		break;

	default:
		break;
	}
}

static void rtk_nf_select_chip(struct nand_chip *chip, int cs)
{
	struct rtk_nf *nf = nand_get_controller_data(chip);
	void __iomem *base = nf->regs;
	unsigned long value = 0x0;

	switch (cs) {
	case -1:
		value = 0xff;
		break;
	case 0:
	case 1:
	case 2:
	case 3:
		value = ~(BIT(cs));
		break;
	default:
		value = ~(BIT(0));
	}

	writel(value, base + REG_PD);
}

static int rtk_nf_hw_init(struct rtk_nf *nf)
{
	void __iomem *base = nf->regs;
	int ret;

	/* init controller */
	writel(0x1E, base + REG_PD);
	writel(0x2, base + REG_TIME_PARA3);
	writel(0x5, base + REG_TIME_PARA2);
	writel(0x2, base + REG_TIME_PARA1);

	writel(0x0, base + REG_MULTI_CHNL_MODE);
	writel(0x0, base + REG_READ_BY_PP);

	/* reset nand */
	writel(0xff, base + REG_ND_CMD);
	writel(0x80, base + REG_ND_CTL);

	if ((ret = rtk_nf_wait_down(base + REG_ND_CTL, 0x80, 0x0)) != 0)
		return ret;

	if ((ret = rtk_nf_wait_down(base + REG_ND_CTL, 0x40, 0x40)) != 0)
		return ret;

	writel(readl(base + REG_NF_LOW_PWR) & ~0x10, base + REG_NF_LOW_PWR);
	writel(NF_SPR_DDR_CTL_spare_ddr_ena(1) |
		NF_SPR_DDR_CTL_per_2k_spr_ena(1) |
		NF_SPR_DDR_CTL_spare_dram_sa(0), base + REG_SPR_DDR_CTL);

	return 0;
}

static void rtk_nf_ecc_init(struct nand_chip *chip)
{
	rtk_nf_update_ecc_info(chip);
}

static int rtk_nf_attach_chip(struct nand_chip *chip)
{
	int ret;

	rtk_nf_ecc_init(chip);

	ret = rtk_nf_init_nandbuf(chip);
	if (ret)
		return  -ENOMEM;

	return 0;
}

static const struct nand_controller_ops rtk_nf_controller_ops = {
	.attach_chip = rtk_nf_attach_chip,
};

static int rtk_nf_nand_chip_init(struct device *dev, struct rtk_nf *nf,
				  struct device_node *np)
{
	struct nand_chip *chip = &nf->chip;
	struct mtd_info *mtd;
	int ret;

	chip->controller = &nf->controller;

	nand_set_flash_node(chip, np);
	nand_set_controller_data(chip, nf);

	mtd = nand_to_mtd(chip);
	mtd->owner = THIS_MODULE;
	mtd->dev.parent = dev;
	mtd->name = RTK_NAME;

	chip->options = NAND_SKIP_BBTSCAN | NAND_NO_SUBPAGE_WRITE |
			NAND_USES_DMA;

	chip->legacy.select_chip = rtk_nf_select_chip;
	chip->legacy.read_byte = rtk_nf_read_byte;
	chip->legacy.read_buf = rtk_nf_read_buf;
	chip->legacy.cmdfunc = rtk_nf_command;
	chip->legacy.waitfunc = rtk_nf_wait;
	chip->legacy.block_bad = rtk_nf_block_bad;
	chip->legacy.block_markbad = rtk_nf_block_markbad;
	chip->ecc.engine_type = NAND_ECC_ENGINE_TYPE_ON_HOST;

	chip->ecc.write_page_raw = rtk_nf_write_page_raw;
	chip->ecc.write_page = rtk_nf_write_page_hwecc;
	chip->ecc.write_oob_raw = rtk_nf_write_oob_raw;
	chip->ecc.write_oob = rtk_nf_write_oob_hwecc;

	chip->ecc.read_page_raw = rtk_nf_read_page_raw;
	chip->ecc.read_page = rtk_nf_read_page_hwecc;
	chip->ecc.read_oob_raw = rtk_nf_read_oob_raw;
	chip->ecc.read_oob = rtk_nf_read_oob_hwecc;
	chip->bbt = NULL;

	rtk_nf_hw_init(nf);

	ret = rtk_nf_get_nand_info_from_bootcode(mtd);
	if (ret < 0) {
		ret = nand_scan_with_ids(chip, 1, NULL);
		if (ret)
			return -ENODEV;
	} else {
		dev_err(dev, "Get nand info from bootcode successfully.\n");
		ret = nand_scan_with_ids(chip, 1, nf->nf_ids);
		if (ret)
			return -ENODEV;
	}

	nf->ppb = mtd->erasesize / mtd->writesize;
	nf->chipsize = mtd->size;

#if defined(CONFIG_MTD_NAND_RTK_BBM)
	ret = rtk_nf_scan_table(mtd);
	if (ret)
		return -ENODEV;
#endif

	ret = mtd_device_register(mtd, NULL, 0);
	if (ret) {
		dev_err(dev, "mtd parse partition error\n");
		nand_cleanup(chip);
		return -ENODEV;
	}

	return 0;
}

static void rtk_nf_pll_setup(struct rtk_nf *nf)
{
	void __iomem *pll_base = nf->pll_regs;

	writel(0x3, pll_base+0x0);
	writel(0x434388, pll_base+0x8);
	writel(0x7, pll_base+0xc);
	udelay(200);
}

static void rtk_nf_gating(struct rtk_nf *nf)
{
	void __iomem *base = nf->regs;

	writel(readl(0x168 + base) | BIT(0), 0x168 + base);
	writel(readl(0x168 + base) | BIT(1), 0x168 + base);
	writel(readl(0x314 + base) | BIT(0), 0x314 + base);
	writel(readl(0x314 + base) | BIT(1), 0x314 + base);
	writel(readl(0x13c + base) | BIT(3), 0x13c + base);
	writel(readl(0x13c + base) | BIT(4), 0x13c + base);
	writel(readl(0x310 + base) | BIT(5), 0x310 + base);
	writel(readl(0x310 + base) | BIT(6), 0x310 + base);
	writel(readl(0x318 + base) | BIT(0), 0x318 + base);
}

static int rtk_nf_read_proc_nandinfo(struct seq_file *m, void *v)
{
	struct rtk_nf *nf = (struct rtk_nf *)m->private;
	struct nand_chip *chip = &nf->chip;
	struct mtd_info *mtd = nand_to_mtd(chip);

	seq_printf(m, "chip size    : %llu\n", nf->chipsize);
	seq_printf(m, "block size   : %u\n", mtd->erasesize);
	seq_printf(m, "page size    : %u\n", mtd->writesize);
	seq_printf(m, "oob size     : %u\n", mtd->oobsize);
	seq_printf(m, "ecc strength : %u\n", mtd->ecc_strength);
#if defined(CONFIG_MTD_NAND_RTK_BBM)
	rtk_nf_read_proc_bbtinfo(m, v);
#endif /* CONFIG_MTD_NAND_RTK_BBM */
	return 0;
}

static int rtk_nf_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, rtk_nf_read_proc_nandinfo, PDE_DATA(inode));
}

static const struct proc_ops info_proc_fops = {
	//.owner = THIS_MODULE,
	.proc_open = rtk_nf_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static void rtk_nf_init_procfs(struct rtk_nf *nf)
{
	nf->rtknf_proc_dir = proc_mkdir(RTKNF_PROC_DIR_NAME, NULL);
	if (nf->rtknf_proc_dir) {
		proc_create_data("info", 0644, nf->rtknf_proc_dir,
					&info_proc_fops, nf);
	}
}

static int rtk_nf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rtk_nf *nf;
	struct resource *res;
	int ret;

	nf = devm_kzalloc(dev, sizeof(*nf), GFP_KERNEL);
	if (!nf)
		return -ENOMEM;

	nand_controller_init(&nf->controller);
	nf->controller.ops = &rtk_nf_controller_ops;

#if defined(CONFIG_MTD_RTK_NAND_HW_SEMAPHORE)
	unsigned int addr;

	if (of_property_read_u32(np, "hw-semaphore", &addr)) {
		addr = 0x9801a63c;
		dev_err(dev, "NAND:can't find hw semaphore in dtb,use default - 0x%x\n",
			addr);
	} else {
		dev_info(dev, "NAND : find hw semaphore in dtb, 0x%x\n", addr);
	}
	nf->hwsem_base = ioremap(addr, 1);
#endif /* CONFIG_MTD_RTK_NAND_HW_SEMAPHORE */

	nf->clk_nand = devm_clk_get(&pdev->dev, "nand");
	if (IS_ERR(nf->clk_nand)) {
		dev_err(dev, "%s: clk_get() returns %ld\n", __func__,
			PTR_ERR(nf->clk_nand));
		goto rtk_nf_probe_exit;
	}

	clk_prepare_enable(nf->clk_nand);

	nf->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nf->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(nf->regs)) {
		ret = PTR_ERR(nf->regs);
		dev_err(dev, "no reg base\n");
		goto rtk_nf_probe_disable_clk;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	nf->pll_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(nf->pll_regs)) {
		ret = PTR_ERR(nf->pll_regs);
		dev_err(dev, "no pll reg base\n");
		goto rtk_nf_probe_disable_clk;
	}

	rtk_nf_pll_setup(nf);

	rtk_nf_gating(nf);

	platform_set_drvdata(pdev, nf);

	ret = rtk_nf_nand_chip_init(dev, nf, np);
	if (ret) {
		dev_err(dev, "failed to init nand chip\n");
		goto rtk_nf_probe_disable_clk;
	}

	rtk_nf_init_procfs(nf);

	return 0;

rtk_nf_probe_disable_clk:
	clk_disable_unprepare(nf->clk_nand);

rtk_nf_probe_exit:
	return ret;
}

static int rtk_nf_remove(struct platform_device *pdev)
{
	struct rtk_nf *nf = platform_get_drvdata(pdev);

	nand_cleanup(&nf->chip);
	clk_disable_unprepare(nf->clk_nand);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rtk_nf_suspend(struct device *dev)
{
	struct rtk_nf *nf = dev_get_drvdata(dev);

	clk_disable_unprepare(nf->clk_nand);

	return 0;
}

static int rtk_nf_resume(struct device *dev)
{
	struct rtk_nf *nf = dev_get_drvdata(dev);

	clk_prepare_enable(nf->clk_nand);
	rtk_nf_hw_init(nf);

	return 0;
}

static SIMPLE_DEV_PM_OPS(rtk_nf_pm_ops, rtk_nf_suspend, rtk_nf_resume);
#endif

static const struct of_device_id rtk_nf_id_table[] = {
	{ .compatible = "realtek,rtd13xx-nf" },
	{ .compatible = "realtek,rtd16xxb-nf" },
	{ .compatible = "realtek,rtd13xxd-nf" },
	{}
};
MODULE_DEVICE_TABLE(of, rtk_nf_id_table);

static struct platform_driver rtk_nf_driver = {
	.probe  = rtk_nf_probe,
	.remove = rtk_nf_remove,
	.driver = {
		.name  = RTK_NAME,
		.of_match_table = rtk_nf_id_table,
#ifdef CONFIG_PM_SLEEP
		.pm = &rtk_nf_pm_ops,
#endif
	},
};

module_platform_driver(rtk_nf_driver);

static int __init rtk_nf_nand_info_setup(char *line)
{
	strlcpy(g_rtk_nandinfo_line, line, sizeof(g_rtk_nandinfo_line));
	return 1;
}
__setup("nandinfo=", rtk_nf_nand_info_setup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("PK Chuang <pk.chuang@realtek.com>");
MODULE_DESCRIPTION("RTK Parallel Nand Flash Controller Driver");
