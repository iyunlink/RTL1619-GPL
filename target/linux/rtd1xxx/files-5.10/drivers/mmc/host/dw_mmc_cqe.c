/*
 * Synopsys DesignWare Multimedia Card Interface driver
 *  (Based on NXP driver for lpc 31xx)
 *
 * Copyright (C) 2009 NXP Semiconductors
 * Copyright (C) 2009, 2010 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/blkdev.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/bitops.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/pm_runtime.h>

#include "../core/card.h"
#include "../core/host.h"
#include "dw_mmc_cqe.h"
#include "cqhci.h"

#define DW_MCI_FREQ_MAX 	200000000       /* unit: HZ */
#define DW_MCI_FREQ_MIN 	100000          /* unit: HZ */
#define DW_MCI_CMDQ_DISABLED	0x30f0001
#define DW_MCI_CMDQ_ENABLED	0x30f0101
#define DW_MCI_POWEROFF		0x3220301
#define DW_MCI_DESC_LEN		0x100000
#define DW_MCI_MAX_SCRIPT_BLK	128
#define DW_MCI_TIMEOUT_MS	3000
#define TUNING_ERR		531
#define DW_MCI_NOT_READY	9999

DECLARE_COMPLETION(dw_mci_wait);

static void dw_mci_request(struct mmc_host *mmc, struct mmc_request *mrq);
static void dw_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios);
irqreturn_t cqhci_irq(struct mmc_host *mmc, u32 intmask, int cmd_error, int data_error);
static void cmd_complete(struct dw_mci *host, u16 interrupt, int *cmd_error);
static void data_complete(struct dw_mci *host, u16 interrupt, int *data_error);

#if 0
static void print_reg(struct dw_mci *host)
{
	int i = 0;
	for(i=0; i<= 0x58 ; i+=4)
		printk(KERN_ERR "0x%x = 0x%x\n", i, readl(host->regs+i));

	printk(KERN_ERR "0x208 = 0x%x\n", readl(host->regs+0x208));
	printk(KERN_ERR "0x22c = 0x%x\n", readl(host->regs+0x22c));
	printk(KERN_ERR "0x420 = 0x%x\n", readl(host->regs+0x420));
	printk(KERN_ERR "0x42c = 0x%x\n", readl(host->regs+0x42c));
	printk(KERN_ERR "0x430 = 0x%x\n", readl(host->regs+0x430));
	printk(KERN_ERR "0x478 = 0x%x\n", readl(host->regs+0x478));
}
#endif

static void dw_mci_cqhci_dumpregs(struct mmc_host *mmc)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;

	pr_err("%s: cmd idx 0x%08x\n", __func__, mci_readw(host, CMD_R));
}

static void dw_mci_cqhci_set_tran_desc(u8 *desc,
					dma_addr_t addr,
					int len,
					bool end,
					bool dma64)
{
	__le32 *attr = (__le32 __force *)desc;

	*attr = (CQHCI_VALID(1) |
		 CQHCI_END(end ? 1 : 0) |
		 CQHCI_INT(0) |
		 CQHCI_ACT(0x4) |
		 CQHCI_DAT_LENGTH(len));

	if (dma64) {
		__le64 *dataddr = (__le64 __force *)(desc + 4);
		dataddr[0] = cpu_to_le64(addr);
	}
	else {
		__le32 *dataddr = (__le32 __force *)(desc + 4);
		dataddr[0] = cpu_to_le32(addr);
	}
}

static void dw_mci_cqhci_setup_tran_desc(struct mmc_data *data,
				      struct cqhci_host *cq_host,
				      u8 *desc,
				      int sg_count)
{
	int i, len;
	bool last = false;
	bool dma64 = cq_host->dma64;
	dma_addr_t addr;
	struct scatterlist *sg;
	u32 cur_blk_cnt, remain_blk_cnt;
	unsigned int begin, end;

	for_each_sg(data->sg, sg, sg_count, i) {
		addr = sg_dma_address(sg);
		len = sg_dma_len(sg);
		remain_blk_cnt  = len >> 9;

		while(remain_blk_cnt) {
			/*DW_MCI_MAX_SCRIPT_BLK is tha max for each descriptor record*/
			if(remain_blk_cnt > DW_MCI_MAX_SCRIPT_BLK)
				cur_blk_cnt = DW_MCI_MAX_SCRIPT_BLK;
			else
				cur_blk_cnt = remain_blk_cnt;

			/* In Synopsys DesignWare Databook Page 84,
			 * They mentioned the DMA 128MB restriction
			 */
			begin = addr / 0x8000000;
			end = (addr + cur_blk_cnt * 512) / 0x8000000;

			if(begin != end)
				cur_blk_cnt = (end * 0x8000000 - addr) / 512;

			if ((i+1) == sg_count && (remain_blk_cnt == cur_blk_cnt))
				last = true;

			dw_mci_cqhci_set_tran_desc(desc, addr, (cur_blk_cnt << 9), last, dma64);

			addr = addr + (cur_blk_cnt << 9);
			remain_blk_cnt -= cur_blk_cnt;
			desc += cq_host->trans_desc_len;
		}
	}
}

static void dw_mci_cqe_enable(struct mmc_host *mmc)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;

	/*clear data path SW_RST_R.SW_RST_DAT = 1*/
	mci_writeb(host, SW_RST_R, SDMMC_SW_RST_DAT);
	/*0x9801200c*/
	mci_writew(host, XFER_MODE_R,
		((1 << SDMMC_MULTI_BLK_SEL) | SDMMC_BLOCK_COUNT_ENABLE | SDMMC_DMA_ENABLE));

	/*Set DMA_SEL to ADMA2 only mode in the HOST_CTRL1_R*/
	mci_writeb(host, HOST_CTRL1_R,
		(mci_readb(host, HOST_CTRL1_R) & 0xe7) | (SDMMC_ADMA2_32 << SDMMC_DMA_SEL));
	mci_writew(host, BLOCKSIZE_R, 0x200);
	mci_writew(host, BLOCKCOUNT_R, 0);

	/*Set SDMASA_R (while using 32 bits) to 0*/
	mci_writel(host, SDMASA_R, 0);
	/*we set this register additionally to enhance the IO perofrmance*/

	cqhci_writel(host->cqe, 0x10, CQHCI_SSC1);
	cqhci_writel(host->cqe, 0, CQHCI_CTL);

	if (cqhci_readl(host->cqe, CQHCI_CTL) && CQHCI_HALT) {
		pr_err("%s: cqhci: CQE failed to exit halt state\n",
			mmc_hostname(mmc));
	}

	/*cmdq interrupt mode*/
	dw_mci_clr_signal_int(host);
	dw_mci_en_cqe_int(host);
}

static const struct cqhci_host_ops dw_mci_cqhci_host_ops = {
	.enable = dw_mci_cqe_enable,
	.dumpregs = dw_mci_cqhci_dumpregs,
	.setup_tran_desc = dw_mci_cqhci_setup_tran_desc,
};

#ifdef CONFIG_OF
static struct dw_mci_board *dw_mci_parse_dt(struct dw_mci *host)
{
	struct dw_mci_board *pdata;
	struct device *dev = host->dev;
	const struct dw_mci_drv_data *drv_data = host->drv_data;
	int ret;
	u32 clock_frequency;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	/* find reset controller when exist */
	pdata->rstc = devm_reset_control_get_optional(dev, "reset");
	if (IS_ERR(pdata->rstc)) {
		if (PTR_ERR(pdata->rstc) == -EPROBE_DEFER)
			return ERR_PTR(-EPROBE_DEFER);
	}

	device_property_read_u32(dev, "card-detect-delay",
		&pdata->detect_delay_ms);

	if (!device_property_read_u32(dev, "clock-frequency", &clock_frequency))
		pdata->bus_hz = clock_frequency;

	if (drv_data && drv_data->parse_dt) {
		ret = drv_data->parse_dt(host);
		if (ret)
			return ERR_PTR(ret);
	}

	return pdata;
}

#else /* CONFIG_OF */
static struct dw_mci_board *dw_mci_parse_dt(struct dw_mci *host)
{
	return ERR_PTR(-EINVAL);
}
#endif /* CONFIG_OF */

static void dw_mci_cmd_timer(struct timer_list *t)
{
	struct dw_mci *host = from_timer(host, t, timer);;

	if(host->int_waiting) {
		pr_err("%s fired, opcode=%d, arg=0x%x, irq status=0x%x, err irq status=0x%x, auto err irq status=0x%x\n",
			__func__, host->opcode, host->arg,
			host->normal_interrupt, host->error_interrupt, host->auto_error_interrupt);
		dw_mci_clr_signal_int(host);
		dw_mci_get_int(host);

		complete(host->int_waiting);
	}
}

static void dw_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	const struct dw_mci_drv_data *drv_data = slot->host->drv_data;

#if 0
	/*Set this regs if using SD card*/
	mci_writeb(host, HOST_CTRL1_R,
		(mci_readb(host, HOST_CTRL1_R) & SDMMC_HIGH_SPEED_MASK) | SDMMC_HIGH_SPEED_EN);
#endif

	switch(ios->timing)
	{
		case MMC_TIMING_MMC_HS400:
			mci_writew(host, HOST_CTRL2_R,
				(mci_readw(host, HOST_CTRL2_R) & SDMMC_UHS_MODE_SEL_MASK) | SDMMC_HS400);
			break;
		case MMC_TIMING_MMC_HS200:
			mci_writew(host, HOST_CTRL2_R,
				(mci_readw(host, HOST_CTRL2_R) & SDMMC_UHS_MODE_SEL_MASK) | SDMMC_HS200);
			break;
		case MMC_TIMING_MMC_DDR52:
			mci_writew(host, HOST_CTRL2_R,
				(mci_readw(host, HOST_CTRL2_R) & SDMMC_UHS_MODE_SEL_MASK) | SDMMC_DDR);
		case MMC_TIMING_MMC_HS:
			mci_writew(host, HOST_CTRL2_R,
				(mci_readw(host, HOST_CTRL2_R) & SDMMC_UHS_MODE_SEL_MASK) | SDMMC_SDR);
			break;
		default:
			/*MMC_TIMING_LEGACY case*/
			mci_writew(host, HOST_CTRL2_R,
				(mci_readw(host, HOST_CTRL2_R) & SDMMC_UHS_MODE_SEL_MASK) | SDMMC_LEGACY);
	}

	slot->clock = ios->clock;

	if (drv_data && drv_data->set_ios)
		drv_data->set_ios(slot, ios);

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_4:
		mci_writeb(host, HOST_CTRL1_R,
			(mci_readb(host, HOST_CTRL1_R) &
			(SDMMC_EXT_DAT_XFER_MASK & SDMMC_DAT_XFER_WIDTH_MASK))|SDMMC_BUS_WIDTH_4);
		break;
	case MMC_BUS_WIDTH_8:
		mci_writeb(host, HOST_CTRL1_R,
			(mci_readb(host, HOST_CTRL1_R) & SDMMC_EXT_DAT_XFER_MASK) | SDMMC_BUS_WIDTH_8);
		break;
	default:
		/* set default 1 bit mode */
		mci_writeb(host, HOST_CTRL1_R,
			(mci_readb(host, HOST_CTRL1_R) &
				(SDMMC_EXT_DAT_XFER_MASK & SDMMC_DAT_XFER_WIDTH_MASK)) | SDMMC_BUS_WIDTH_1);
	}
}

static int dw_mci_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	const struct dw_mci_drv_data *drv_data = host->drv_data;
	int err = -EINVAL;

	if (drv_data && drv_data->execute_tuning)
		err = drv_data->execute_tuning(slot, opcode);
	return err;
}

static void dw_mci_tasklet_func(unsigned long priv)
{
	struct dw_mci *host = (struct dw_mci *)priv;
	struct mmc_host *prev_mmc = host->slot->mmc;
	struct mmc_request* mrq;
	unsigned long flags;

	spin_lock_irqsave(&host->irq_lock, flags);

	host->cmd = NULL;
	host->data = NULL;
	mrq = host->mrq;
	host->slot->mrq = NULL;
	host->mrq = NULL;

	spin_unlock_irqrestore(&host->irq_lock, flags);

	mmc_request_done(prev_mmc, mrq);
}

static irqreturn_t dw_mci_interrupt(int irq, void *dev_id)
{
	struct dw_mci *host = dev_id;
	struct mmc_host *mmc = host->slot->mmc;
	struct cqhci_host *cq_host = NULL;
	int cmd_error=0, data_error=0;

	if(host->pdata && (host->pdata->caps2 & MMC_CAP2_CQE))
		cq_host = mmc->cqe_private;

	dw_mci_get_int(host);

	if(host->pdata && (host->pdata->caps2 & MMC_CAP2_CQE)) {
		if(mmc->cqe_on==false && cq_host->activated==false)
			dw_mci_clr_signal_int(host);
	}
	else
		dw_mci_clr_signal_int(host);

	/*if run the cmdq mode*/
	if(host->pdata && (host->pdata->caps2 & MMC_CAP2_CQE) &&
		mmc->cqe_on==true && cq_host->activated==true) {
		if(host->normal_interrupt & SDMMC_ERR_INTERRUPT) {
			pr_err("%s: cmdq error: interrupt status=%08x, error interrupt=0x%08x, CQIS=0x%x, CQTCN=0x%x\n",
				__func__, host->normal_interrupt, host->error_interrupt,
				readl(host->cqe->mmio + CQHCI_IS), readl(host->cqe->mmio + CQHCI_TCN));

			cmd_complete(host, host->error_interrupt, &cmd_error);
			data_complete(host, host->error_interrupt, &data_error);
		}
		cqhci_irq(mmc, (u32)(host->normal_interrupt), cmd_error, data_error);
		dw_mci_clr_int(host);

		return IRQ_HANDLED;
	}

	if(host->int_waiting) {
		del_timer(&host->timer);
		complete(host->int_waiting);
	}

	return IRQ_HANDLED;
}

static void dw_mci_setup(struct dw_mci *host)
{
	/*This IP regiter setting should be more flexible accordding to the SD, EMMC, or UHS2 in the future*/
	mci_writeb(host, SW_RST_R, (SDMMC_SW_RST_ALL|SDMMC_SW_RST_CMD|SDMMC_SW_RST_DAT));
	mci_writeb(host, TOUT_CTRL_R, 0xe);
	mci_writew(host, HOST_CTRL2_R, SDMMC_HOST_VER4_ENABLE|SDMMC_SIGNALING_EN);
	mci_writew(host, NORMAL_INT_STAT_EN_R, 0xffff);
	mci_writew(host, ERROR_INT_STAT_EN_R, SDMMC_ALL_ERR_STAT_EN);
	/*Card detect will be enabled in the last*/
	mci_writew(host, NORMAL_INT_SIGNAL_EN_R, (~(BIT(6)|BIT(7) | BIT(8)) & 0xffff) );
	mci_writew(host, ERROR_INT_SIGNAL_EN_R, SDMMC_ALL_ERR_SIGNAL_EN);
	mci_writeb(host, CTRL_R, SDMMC_RST_N_OE|SDMMC_RST_N|SDMMC_CARD_IS_EMMC);
	mci_writeb(host, HOST_CTRL1_R,
		(mci_readb(host, HOST_CTRL1_R)&0xe7) | (SDMMC_ADMA2_32 << SDMMC_DMA_SEL));
	mci_writeb(host, MSHC_CTRL_R, mci_readb(host, MSHC_CTRL_R) & (~SDMMC_CMD_CONFLICT_CHECK));
	mci_writew(host, CLK_CTRL_R,
		mci_readw(host, CLK_CTRL_R)|SDMMC_INTERNAL_CLK_EN);
}

static int dw_mci_init_slot_caps(struct dw_mci_slot *slot)
{
	struct dw_mci *host = slot->host;
	const struct dw_mci_drv_data *drv_data = host->drv_data;
	struct mmc_host *mmc = slot->mmc;
	int ctrl_id;

	if (host->pdata->caps)
		mmc->caps = host->pdata->caps;

	if (host->pdata->pm_caps)
		mmc->pm_caps = host->pdata->pm_caps;

	if (host->dev->of_node) {
		ctrl_id = of_alias_get_id(host->dev->of_node, "mshc");
		if (ctrl_id < 0)
			ctrl_id = 0;
	} else {
		ctrl_id = to_platform_device(host->dev)->id;
	}

	if (drv_data && drv_data->caps) {
		if (ctrl_id >= drv_data->num_caps) {
			dev_err(host->dev, "invalid controller id %d\n",
				ctrl_id);
			return -EINVAL;
		}
		mmc->caps |= drv_data->caps[ctrl_id];
	}

	if (host->pdata->caps2)
		mmc->caps2 = host->pdata->caps2;

	mmc->f_min = DW_MCI_FREQ_MIN;
	if (!mmc->f_max)
		mmc->f_max = DW_MCI_FREQ_MAX;

	/* Process SDIO IRQs through the sdio_irq_work. */
	if (mmc->caps & MMC_CAP_SDIO_IRQ)
		mmc->caps2 |= MMC_CAP2_SDIO_IRQ_NOTHREAD;

	return 0;
}

static int dw_mci_get_ro(struct mmc_host *mmc)
{
	int read_only = -1;
#if 0
	struct dw_mci_slot *slot = mmc_priv(mmc);
#endif
	int gpio_ro = mmc_gpio_get_ro(mmc);

	/* Use platform get_ro function, else try on board write protect */
	if (gpio_ro >= 0)
		read_only = gpio_ro;
	else {
		/*Need to read the IP register to judge if ro*/
		pr_err("IP get_ro feature is not implemented currently.\n");
	}
	dev_dbg(&mmc->class_dev, "card is %s\n",
		read_only ? "read-only" : "read-write");

	return read_only;
}

static int dw_mci_get_cd(struct mmc_host *mmc)
{
	int present = -1;
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	int gpio_cd = mmc_gpio_get_cd(mmc);

	/* Use platform get_cd function, else try onboard card detect */
	if (((mmc->caps & MMC_CAP_NEEDS_POLL)
		|| !mmc_card_is_removable(mmc))) {
		present = 1;

		if (!test_bit(DW_MMC_CARD_PRESENT, &slot->flags)) {
			if (mmc->caps & MMC_CAP_NEEDS_POLL) {
				dev_info(&mmc->class_dev,
					"card is polling.\n");
			} else {
				dev_info(&mmc->class_dev,
					"card is non-removable.\n");
			}
			set_bit(DW_MMC_CARD_PRESENT, &slot->flags);
		}

		return present;
	}
	else if (gpio_cd >= 0)
		present = gpio_cd;
	else {
		/*SD card card detect using IP regs is todo*/
		pr_err("SD card card detect using IP regs is ToDo.\n");
	}

	spin_lock_bh(&host->lock);
	if (present && !test_and_set_bit(DW_MMC_CARD_PRESENT, &slot->flags))
		dev_dbg(&mmc->class_dev, "card is present\n");
	else if (!present &&
		!test_and_clear_bit(DW_MMC_CARD_PRESENT, &slot->flags))
		dev_dbg(&mmc->class_dev, "card is not present\n");
	spin_unlock_bh(&host->lock);

	return present;
}

static int dw_mci_switch_voltage(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	const struct dw_mci_drv_data *drv_data = host->drv_data;

	if (drv_data && drv_data->switch_voltage)
		return drv_data->switch_voltage(mmc, ios);

	return 0;
}

static int dw_mci_prepare_hs400_tuning(struct mmc_host *mmc,
				       struct mmc_ios *ios)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	const struct dw_mci_drv_data *drv_data = host->drv_data;

	if (drv_data && drv_data->prepare_hs400_tuning)
		return drv_data->prepare_hs400_tuning(host, ios);

	return 0;
}

static void dw_mci_hs400_complete(struct mmc_host *mmc)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	const struct dw_mci_drv_data *drv_data = host->drv_data;

	if (drv_data && drv_data->hs400_complete)
		drv_data->hs400_complete(mmc);
}

static void dw_mci_init_card(struct mmc_host *mmc, struct mmc_card *card)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	const struct dw_mci_drv_data *drv_data = host->drv_data;

	/*
	 Add any quirks for this synopsys IP here or
	 deal with something special for some specific
	 vendors' SOC platform by calling drv_data->init_card().
	 */
	if (drv_data && drv_data->init_card)
		drv_data->init_card(mmc, card);
}

static const struct mmc_host_ops dw_mci_ops = {
	.request                	= dw_mci_request,
	.set_ios                	= dw_mci_set_ios,
	.get_ro                 	= dw_mci_get_ro,
	.get_cd                 	= dw_mci_get_cd,
	//.hw_reset               	= dw_mci_hw_reset,
	//.enable_sdio_irq        	= dw_mci_enable_sdio_irq,
	//.ack_sdio_irq           	= dw_mci_ack_sdio_irq,
	.execute_tuning         	= dw_mci_execute_tuning,
	//.card_busy             	= dw_mci_card_busy,
	.start_signal_voltage_switch 	= dw_mci_switch_voltage,
	.prepare_hs400_tuning   	= dw_mci_prepare_hs400_tuning,
	.hs400_complete         	= dw_mci_hs400_complete,
	.init_card 			= dw_mci_init_card,
};

static int dw_mci_init_slot(struct dw_mci *host)
{
	struct mmc_host *mmc;
	struct dw_mci_slot *slot;
	int ret;

	mmc = mmc_alloc_host(sizeof(struct dw_mci_slot), host->dev);
	if (!mmc)
		return -ENOMEM;

	slot = mmc_priv(mmc);
	slot->id = 0;
	slot->sdio_id = host->sdio_id0 + slot->id;
	slot->mmc = mmc;
	slot->switch_partition = 0;
	slot->host = host;
	host->slot = slot;

	mmc->ops = &dw_mci_ops;

	/*if there are external regulators, get them*/
	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		goto err_host_allocated;

	if (!mmc->ocr_avail)
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	printk(KERN_ERR "%s: regulator support volage ocr_avail=0x%x\n",
			__func__, mmc->ocr_avail);

	ret = mmc_of_parse(mmc);
	if (ret)
		goto err_host_allocated;

	ret = dw_mci_init_slot_caps(slot);
	if (ret)
		goto err_host_allocated;

	/* Useful defaults if platform data is unset. */
	if (host->use_dma == TRANS_MODE_DMA) {
		mmc->max_segs = 256;
		mmc->max_blk_size = 512;
		mmc->max_seg_size = 0x1000;
		mmc->max_req_size = mmc->max_seg_size * mmc->max_segs;
		mmc->max_blk_count = mmc->max_req_size / 512;
	} else {
		pr_err("dw-mmc-cqe pio mode is ToDo.\n");
		/* To DO, TRANS_MODE_PIO */
	}

	dw_mci_get_cd(mmc);

	ret = mmc_add_host(mmc);
	if (ret)
		goto err_host_allocated;

	return 0;

err_host_allocated:
	mmc_free_host(mmc);
	return ret;
}

static void dw_mci_cleanup_slot(struct dw_mci_slot *slot)
{
	/* Debugfs stuff is cleaned up by mmc core */
	mmc_remove_host(slot->mmc);
	slot->host->slot = NULL;
	mmc_free_host(slot->mmc);
}

static u32 dw_mci_prepare_data_flags(struct mmc_command *cmd)
{
	u32 dataflags;
	int read_flag=1;
	int mul_blk_flag=0;
	int auto_stop_flag=0;

	if(cmd->opcode == MMC_WRITE_BLOCK ||
	   cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK ||
	   cmd->opcode == MMC_LOCK_UNLOCK ||
	   (cmd->opcode == MMC_GEN_CMD && cmd->arg == 0))
		read_flag=0;

	if(cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK ||
	   cmd->opcode == MMC_READ_MULTIPLE_BLOCK) {
		mul_blk_flag=1;
		auto_stop_flag=1;
	}

	dataflags = (mul_blk_flag << SDMMC_MULTI_BLK_SEL) |
		    (read_flag << SDMMC_DATA_XFER_DIR) |
		    (auto_stop_flag << SDMMC_AUTO_CMD_ENABLE) |
		    (SDMMC_BLOCK_COUNT_ENABLE) |
		    (SDMMC_DMA_ENABLE);

	return dataflags;
}

static u32 dw_mci_prepare_command(struct mmc_host *mmc, struct mmc_command *cmd)
{
	u32 cmdr;

	cmd->error = -EINPROGRESS;
	cmdr = (cmd->opcode << 8);

	if(cmd->flags & MMC_RSP_PRESENT) {
		if(cmd->flags & MMC_RSP_136)
			cmdr |= SDMMC_RESP_LEN_136;
		else {
			if(cmd->flags & MMC_RSP_BUSY)
				cmdr |= SDMMC_RESP_LEN_48B;
			else
				cmdr |= SDMMC_RESP_LEN_48;
		}
	}

	cmdr |= SDMMC_CMD_CHK_RESP_CRC;
	if(cmd->opcode == MMC_GO_IDLE_STATE ||
	   cmd->opcode == MMC_SEND_OP_COND ||
	   (cmd->opcode == MMC_SELECT_CARD && cmd->flags == (MMC_RSP_NONE | MMC_CMD_AC)))
		cmdr &= ~SDMMC_CMD_CHK_RESP_CRC;

	cmdr |= SDMMC_CMD_IDX_CHK_ENABLE;
	if(cmd->opcode == MMC_GO_IDLE_STATE ||
	   cmd->opcode == MMC_SEND_OP_COND ||
	   cmd->opcode == MMC_SEND_CSD ||
	   cmd->opcode == MMC_SEND_CID ||
	   cmd->opcode == MMC_ALL_SEND_CID ||
	   (cmd->opcode == MMC_SELECT_CARD && cmd->flags == (MMC_RSP_NONE | MMC_CMD_AC)))
		cmdr &= ~SDMMC_CMD_IDX_CHK_ENABLE;

	if(cmd->data)
		cmdr |= SDMMC_DATA;

	if(cmd->opcode == MMC_STOP_TRANSMISSION)
		cmdr |= (SDMMC_ABORT_CMD << 6);

	return cmdr;
}

static void dw_mci_prepare_desc64(struct dw_mci *host,
				 struct mmc_data *data,
				 struct scatterlist *sg)
{
	/*ToDo*/
	pr_err("Currently, the 64bit DMA mode is not implemented yet.\n");
}

static void dw_mci_prepare_desc32(struct dw_mci *host,
				 struct mmc_data *data,
				 struct scatterlist *sg)
{
	u32  blk_cnt, cur_blk_cnt, remain_blk_cnt;
	u32  tmp_val;
	u32* desc_base = host->desc_vaddr ;
	u32  dma_len = 0;
	u32  dma_addr;
	u32  i;
	unsigned int begin, end;

	for(i=0; i<host->dma_nents; i++, sg++) {
		dma_len = sg_dma_len(sg);

		/*blk_cnt must be the multiple of 512(0x200)*/
		if(dma_len < 512)
			blk_cnt = 1;
		else
			blk_cnt  = dma_len >> 9;

		remain_blk_cnt  = blk_cnt;
		dma_addr = sg_dma_address(sg);

		while(remain_blk_cnt) {
			/*DW_MCI_MAX_SCRIPT_BLK is tha max for each descriptor record*/
			if(remain_blk_cnt > DW_MCI_MAX_SCRIPT_BLK)
				cur_blk_cnt = DW_MCI_MAX_SCRIPT_BLK;
			else
				cur_blk_cnt = remain_blk_cnt;

			/* In Synopsys DesignWare Databook Page 84,
			 * They mentioned the DMA 128MB restriction
			 */
			begin = dma_addr / 0x8000000;
			end = (dma_addr + cur_blk_cnt * 512) / 0x8000000;

			/*If begin and end in the different 128MB memory zone*/
			if(begin != end)
				cur_blk_cnt = (end * 0x8000000 - dma_addr) / 512;

			if(dma_len < 512)
				tmp_val = ((dma_len) << 16) | BIT(0) | BIT(5);
			else
				tmp_val = ((cur_blk_cnt & 0x7f) << 25) | BIT(0) | BIT(5);

			/*Last descriptor*/
			if(i == host->dma_nents - 1 && remain_blk_cnt == cur_blk_cnt)
				tmp_val |= BIT(1);

			desc_base[0] =  tmp_val;
			desc_base[1] =  dma_addr;

			dma_addr = dma_addr + (cur_blk_cnt << 9);
			remain_blk_cnt -= cur_blk_cnt;
			desc_base += 2;
		}
	}
}

static void dw_mci_submit_data(struct dw_mci *host, struct mmc_data *data)
{
	u32 dir = 0;

	host->sg = NULL;
	host->data = data;

	if(data->flags & MMC_DATA_READ)
		dir = DMA_FROM_DEVICE;
	else
		dir = DMA_TO_DEVICE;

	host->dma_nents = dma_map_sg(mmc_dev(host->slot->mmc), data->sg, data->sg_len, dir);
	host->sg = data->sg;

	if (host->dma_64bit_address == 1)
		dw_mci_prepare_desc64(host, host->data, host->sg);
	else
		dw_mci_prepare_desc32(host, host->data, host->sg);
}

static void dw_mci_endup_data(struct dw_mci *host, struct mmc_data *data)
{
	u32 dir = 0;

	if(data->flags & MMC_DATA_READ)
		dir = DMA_FROM_DEVICE;
	else
		dir = DMA_TO_DEVICE;

	dma_unmap_sg(mmc_dev(host->slot->mmc), data->sg, data->sg_len, dir);
	host->sg = NULL;
}

void wait_done(struct dw_mci *host, volatile u32 *addr,
		      u32 mask, u32 value)
{
	int n = 0;

	while(1)
	{
		if (((*addr) & mask) == value)
			break;

		/*error interrupt detected*/
		if((mci_readw(host, NORMAL_INT_STAT_R) & SDMMC_ERR_INTERRUPT)!=0)
			break;

		if(n++ > 3000000) {
			pr_err("%s: addr=%px, *addr=0x%x, mask=0x%x, value=0x%x\n",
				__func__, addr, readl(addr), mask, value);
			break;
		}
		udelay(1);
	}
}
EXPORT_SYMBOL(wait_done);

static void dw_mci_regs_show(struct dw_mci *host,
			    struct mmc_command *cmd, u32 cmd_flags)
{
	pr_err("opcode = %d, arg = 0x%x, cmdflags = 0x%x\n", cmd->opcode, cmd->arg, cmd_flags);
	pr_err("status_interrupt = 0x%x, error_interrupt = 0x%x, auto_error_interrupt = 0x%x\n",
		host->normal_interrupt, host->error_interrupt, host->auto_error_interrupt);
	pr_err("0x24 = 0x%x\n", mci_readl(host, PSTATE_REG));
	pr_err("0x28 = 0x%x\n", mci_readb(host, HOST_CTRL1_R));
	pr_err("0x0c = 0x%x\n", mci_readw(host, XFER_MODE_R));
}

static int dw_mci_start_command(struct dw_mci *host,
				struct mmc_command *cmd, u32 cmd_flags)
{
	int err = 0;
	unsigned long end=0;
	unsigned long flags;
	u8 xfer_flag=0;

	host->cmd = cmd;

	switch(cmd->opcode) {
		case MMC_READ_SINGLE_BLOCK:
		case MMC_READ_MULTIPLE_BLOCK:
		case MMC_WRITE_BLOCK:
		case MMC_WRITE_MULTIPLE_BLOCK:
		case MMC_SEND_EXT_CSD:
		case MMC_GEN_CMD:
		case MMC_SLEEP_AWAKE:
		case MMC_SWITCH:
		case MMC_SET_WRITE_PROT:
		case MMC_CLR_WRITE_PROT:
		case MMC_SEND_WRITE_PROT:
		case MMC_ERASE:
		case MMC_SEND_TUNING_BLOCK_HS200:
			xfer_flag=1;
			break;
		default:
			xfer_flag=0;
	}

	host->int_waiting = &dw_mci_wait;
	end = jiffies + msecs_to_jiffies(DW_MCI_TIMEOUT_MS);
	mod_timer(&host->timer, end);

	if (host->int_waiting) {
		dw_mci_clr_signal_int(host);
		dw_mci_clr_int(host);

		/*command with data, r1b case*/
		if (xfer_flag==1)
			dw_mci_en_xfer_int(host);
		else
			dw_mci_en_cd_int(host);

		/*If we use cmd23, we cannot send auto stop command*/
		if (cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK ||
		    cmd->opcode == MMC_READ_MULTIPLE_BLOCK) {
			if (host->is_sbc) {
				mci_writew(host, XFER_MODE_R,
					mci_readw(host, XFER_MODE_R) & ~BIT(SDMMC_AUTO_CMD_ENABLE));
					host->is_sbc = 0;
			}
		}

		host->opcode = cmd->opcode;
		host->arg = cmd->arg;

		spin_lock_irqsave(&host->irq_lock,flags);
		mci_writew(host, CMD_R, cmd_flags);
		spin_unlock_irqrestore(&host->irq_lock,flags);

		wait_for_completion(host->int_waiting);

		if(xfer_flag == 1)
			wait_done(host, (u32*)(host->regs + SDMMC_NORMAL_INT_STAT_R), BIT(1), BIT(1));
		else
			wait_done(host, (u32*)(host->regs + SDMMC_NORMAL_INT_STAT_R), BIT(0), BIT(0));

		if (host->normal_interrupt & SDMMC_ERR_INTERRUPT) {
			if(host->tuning == 1) {
				/*pr_err("Tuning error and just ignore.\n");*/
			}
			else {
				dw_mci_regs_show(host, cmd, cmd_flags);
			}
			err = -1;
		}
	}

	return err;
}

static void dw_mci_read_rsp(struct dw_mci *host, struct mmc_command *cmd, u32 *rsp)
{
	if (cmd->flags & MMC_RSP_PRESENT) {
		if(cmd->flags & MMC_RSP_136) {
			/*R2 long response*/
			u32 rsp_tmp[4];
			rsp_tmp[3] = mci_readl(host, RESP01_R);
			rsp_tmp[2] = mci_readl(host, RESP23_R);
			rsp_tmp[1] = mci_readl(host, RESP45_R);
			rsp_tmp[0] = mci_readl(host, RESP67_R);

			rsp[3] = (rsp_tmp[3] & 0x00ffffff) << 8;
			rsp[2] = ((rsp_tmp[2] & 0x00ffffff) << 8) | ((rsp_tmp[3] & 0xff000000) >> 24);
			rsp[1] = ((rsp_tmp[1] & 0x00ffffff) << 8) | ((rsp_tmp[2] & 0xff000000) >> 24);
			rsp[0] = ((rsp_tmp[0] & 0x00ffffff) << 8) | ((rsp_tmp[1] & 0xff000000) >> 24);
		}
		else {
			/*Short response*/
			rsp[0] = rsp[1] = rsp[2] = rsp[3] = 0;
			rsp[0] = mci_readl(host, RESP01_R);
		}
	}
}

static void cmd_complete(struct dw_mci *host, u16 interrupt, int *cmd_error)
{
	if (interrupt & (SDMMC_CMD_IDX_ERR | SDMMC_CMD_END_BIT_ERR | SDMMC_CMD_CRC_ERR))
	{
		if(host->tuning)
			*cmd_error = -TUNING_ERR;
		else
			*cmd_error = -EILSEQ;
	}
	else if (interrupt & SDMMC_CMD_TOUT_ERR) {
		if(host->tuning)
			*cmd_error = -TUNING_ERR;
		else
			*cmd_error = -ETIMEDOUT;
	}
	else
		*cmd_error = 0;
}

static void data_complete(struct dw_mci *host, u16 interrupt, int *data_error)
{
	if (interrupt & (SDMMC_DATA_END_BIT_ERR | SDMMC_DATA_CRC_ERR)) {
		if(host->tuning)
			*data_error = -TUNING_ERR;
		else
			*data_error = -EILSEQ;
	}
	else if (interrupt & SDMMC_DATA_TOUT_ERR) {
		if(host->tuning)
			*data_error = -TUNING_ERR;
		else
			*data_error = -ETIMEDOUT;
	}
	else if (interrupt & SDMMC_ADMA_ERR)
		*data_error = -EIO;
	else
		*data_error = 0;
}

static void dw_mci_send_stop_command(struct dw_mci *host, struct mmc_command *cmd)
{
	struct mmc_command stop;
	u32 cmdr;

	/*Stop command only use after data command*/
	if (!cmd->data)
		return;

	memset(&stop, 0, sizeof(struct mmc_command));

	if (cmd->opcode == MMC_READ_SINGLE_BLOCK ||
	    cmd->opcode == MMC_READ_MULTIPLE_BLOCK ||
	    cmd->opcode == MMC_WRITE_BLOCK ||
	    cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK ||
	    cmd->opcode == MMC_SEND_TUNING_BLOCK ||
	    cmd->opcode == MMC_SEND_TUNING_BLOCK_HS200) {
		stop.opcode = MMC_STOP_TRANSMISSION;
		stop.arg = 0;
		stop.flags = MMC_RSP_R1 | MMC_CMD_AC;
	} else if (cmd->opcode == SD_IO_RW_EXTENDED) {
		stop.opcode = SD_IO_RW_DIRECT;
		stop.arg |= (1 << 31) | (0 << 28) | (SDIO_CCCR_ABORT << 9) |
			    ((cmd->arg >> 28) & 0x7);
		stop.flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_AC;
	} else {
		return;
	}

	cmdr = (stop.opcode << 8) | SDMMC_RESP_LEN_48 |
		SDMMC_CMD_CHK_RESP_CRC | SDMMC_CMD_IDX_CHK_ENABLE;

	cmdr |= (SDMMC_ABORT_CMD << 6);

	mci_writew(host, XFER_MODE_R, 0);
	mci_writel(host, ARGUMENT_R, stop.arg);

	dw_mci_start_command(host, &stop, cmdr);
}

static void dw_mci_reset(struct dw_mci *host)
{
	/*check the cmd line*/
	if(mci_readw(host, ERROR_INT_STAT_R) & SDMMC_CMD_ERR) {
		/*Perform a software reset*/
		mci_writeb(host, SW_RST_R, BIT(1));
		wait_done(host, (u32*)(host->regs + SDMMC_CLK_CTRL_R), BIT(25), 0);
	}
	/*check data line*/
	if(mci_readw(host, ERROR_INT_STAT_R) & SDMMC_DATA_ERR) {
		mci_writeb(host, SW_RST_R, BIT(2));
		wait_done(host, (u32*)(host->regs + SDMMC_CLK_CTRL_R), BIT(26), 0);
	}
}

static int dw_mci_wait_status(struct dw_mci *host, u32 *status)
{
	int err = 0;
	struct mmc_command cmd;
	u32 cmdr;
	unsigned long timeend;
	u8 cur_state;

	memset(&cmd, 0, sizeof(struct mmc_command));

	timeend = jiffies + msecs_to_jiffies(600);

	do {
		cmd.opcode = MMC_SEND_STATUS;
		cmd.arg = 1 << 16;
		cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
		cmd.data = NULL;

		cmdr = (cmd.opcode << 8) | SDMMC_RESP_LEN_48 |
			SDMMC_CMD_CHK_RESP_CRC | SDMMC_CMD_IDX_CHK_ENABLE;

		mci_writew(host, XFER_MODE_R, 0);
		mci_writel(host, ARGUMENT_R, cmd.arg);

		err = dw_mci_start_command(host, &cmd, cmdr);
		if (err) {
			dw_mci_reset(host);
			break;
		}
		dw_mci_read_rsp(host, &cmd, cmd.resp);

		*status = cmd.resp[0];
		cur_state = R1_CURRENT_STATE(cmd.resp[0]);
		err = -DW_MCI_NOT_READY;
		if(cur_state == R1_STATE_TRAN) {
			if(cmd.resp[0] & R1_READY_FOR_DATA){
				err = 0;
				break;
			}
		}
	}while(time_before(jiffies, timeend));

	return err;
}

static void dw_mci_err_handle(struct dw_mci *host,
			      struct dw_mci_slot *slot,
			      struct mmc_command *cmd)
{
	int err = 0;
	int rty_cnt = 0;
	int pstat_rty=0;
	u32 status = 0;

	dw_mci_reset(host);

	if(cmd->data) {
		do {
			mci_writew(host, ERROR_INT_STAT_R, mci_readw(host, ERROR_INT_STAT_R) & 0xffff);
			/*synchronous abort: stop host dma*/
			mci_writeb(host, BGAP_CTRL_R, BIT(0));
			wait_done(host, (u32*)(host->regs + SDMMC_NORMAL_INT_STAT_R), BIT(1), BIT(1));

			mci_writew(host, NORMAL_INT_STAT_R, BIT(1));

			do {
				if(cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200) {
					dw_mci_send_stop_command(host, cmd);
					mdelay(1);

					err = dw_mci_wait_status(host, &status);

					rty_cnt++;
					if(rty_cnt > 100) {
						if(err==-DW_MCI_NOT_READY)
							pr_err("%s: status check failed, err = %d, status = 0x%x\n",
								__func__, err, status);
						break;
					}
				}
				else
					break;
			}while(err);

			mci_writeb(host, SW_RST_R, BIT(1)|BIT(2));
			wait_done(host, (u32*)(host->regs + SDMMC_CLK_CTRL_R), (BIT(25)|BIT(26)), 0);
			wait_done(host, (u32*)(host->regs + SDMMC_PSTATE_REG), (BIT(0)|BIT(1)), 0);
			udelay(40);

			pstat_rty++;
			if(pstat_rty > 5000)
			{
				pr_err("%s: wait pstate register data line ready timeout...\n", __func__);
				break;
			}
		}while((mci_readl(host, PSTATE_REG) & 0xf00000) != 0xf00000 ||
			(mci_readl(host, PSTATE_REG) & 0xf0) != 0xf0);
	}
}

static void __dw_mci_start_request(struct dw_mci *host,
				   struct dw_mci_slot *slot,
				   struct mmc_command *cmd)
{
	struct mmc_request *mrq;
	struct mmc_data *data;
	u32 cmdflags;
	u32 dataflags;
	int ret = 0;

	mrq = slot->mrq;
	host->mrq = mrq;

	data = cmd->data;

	if (data) {
		mci_writew(host, BLOCKCOUNT_R, data->blocks);
		mci_writel(host, BLOCKSIZE_R, data->blksz);
		mci_writel(host, ADMA_SA_LOW_R, host->desc_paddr);

		dataflags = dw_mci_prepare_data_flags(cmd);

		mci_writew(host, XFER_MODE_R, dataflags);
	}
	else {
		if (cmd->opcode == MMC_SET_BLOCK_COUNT)
			host->is_sbc = 1;
		else
			host->is_sbc = 0;

		mci_writew(host, XFER_MODE_R, 0);
	}

	mci_writel(host, ARGUMENT_R, cmd->arg);

	cmdflags = dw_mci_prepare_command(slot->mmc, cmd);

	if (data) {
		data->bytes_xfered = 0;
		if(host->use_dma == TRANS_MODE_DMA) {
			dw_mci_submit_data(host, data);
			wmb(); /* drain writebuffer */
		}
		else {
			/*Using PIO mode*/
			pr_err("pio mode is not supported currently\n");
		}
	}

	ret = dw_mci_start_command(host, cmd, cmdflags);

	if(ret == 0) {
		dw_mci_read_rsp(host, cmd, cmd->resp);

		if (data)
			data->bytes_xfered += (data->blocks * data->blksz);
	}

	cmd_complete(host, host->error_interrupt, &cmd->error);
	if (data) {
		data_complete(host, host->error_interrupt, &data->error);
		if(host->use_dma == TRANS_MODE_DMA)
			dw_mci_endup_data(host, data);
		else {
			/*Using PIO mode*/
			pr_err("pio mode is not supported currently\n");
		}
	}

	if(ret !=0)
		dw_mci_err_handle(host, slot, cmd);

	if (cmd->opcode == SD_SWITCH_VOLTAGE) {
		/*
		 * If cmd11 needs to be dealt with specially, put in here.
		 */
	}
}

static void dw_mci_start_request(struct dw_mci *host, struct dw_mci_slot *slot)
{
	struct mmc_request *mrq = slot->mrq;

	if(mrq->sbc)
		__dw_mci_start_request(host, slot, mrq->sbc);

	if(mrq->cmd)
		__dw_mci_start_request(host, slot, mrq->cmd);
}

static int dw_mci_switch(struct mmc_host *mmc,
			 u8 set,
			 u8 index,
			 u8 value,
			 unsigned int timeout_ms)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	struct mmc_command cmd;
	int err=0;
	u32 cmdr;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode         = MMC_SWITCH;
	cmd.arg            = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
			      (index << 16) |
			      (value << 8) |
			      set;
	cmd.flags          = MMC_CMD_AC|MMC_RSP_SPI_R1B | MMC_RSP_R1B;
	cmd.data 	   = NULL;

	cmdr = (cmd.opcode << 8) | SDMMC_RESP_LEN_48B |
		SDMMC_CMD_CHK_RESP_CRC | SDMMC_CMD_IDX_CHK_ENABLE;

	mci_writew(host, XFER_MODE_R, 0);
	mci_writel(host, ARGUMENT_R, cmd.arg);

	err = dw_mci_start_command(host, &cmd, cmdr);

	if(err) {
		pr_err("%s interrupt status reg : 0x%x, interrupt error reg : 0x%x\n",
			__func__, host->normal_interrupt, host->error_interrupt);
	}

	return err;
}

static int dw_mci_cqe_switch(struct mmc_host *mmc, bool enable)
{
	int ret = 0;
	struct mmc_card* card = mmc->card;
	u8 val = enable ? EXT_CSD_CMDQ_MODE_ENABLED : 0;

	if (!card->ext_csd.cmdq_support)
	{
		pr_err("The device card does not support cqe mode\n");
		return 0;
	}

	ret = dw_mci_switch(mmc, EXT_CSD_CMD_SET_NORMAL,
			    EXT_CSD_CMDQ_MODE_EN, val,
			    card->ext_csd.generic_cmd6_time);
	if (ret) {
		pr_err("%s: cmdq mode %sable failed %d\n", __func__, enable ? "en" : "dis", ret);
		goto out;
	}
	else
		card->ext_csd.cmdq_en = enable;
out:
	return ret;
}

static void dw_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	int ret;
	u32 status = 0;

	WARN_ON(slot->mrq);

	/*
	 * The check for card presence and queueing of the request must be
	 * atomic, otherwise the card could be removed in between and the
	 * request wouldn't fail until another card was inserted.
	 */

	if (!dw_mci_get_cd(mmc)) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	down_write(&host->cr_rw_sem);

	/*cmdq case needs extra check*/
	if(host->pdata && (host->pdata->caps2 & MMC_CAP2_CQE))
	{
		if(mmc->cqe_on==false && host->cqe->activated==true
			&& slot->switch_partition==0)
			cqhci_deactivate(mmc);

		if(mrq->cmd->opcode==MMC_SWITCH && mrq->cmd->arg == DW_MCI_CMDQ_DISABLED)
			slot->switch_partition = 1;

		/* we do not need to disable cmdq if it is rpmb request
		 * because rpmb has been changed to rpmb partition in block.c
		 * Also, we do not need to disable cmdq if this command is disable/enable cmdq
		 */
		if(mmc->card && mmc->card->ext_csd.cmdq_en==1
			&& slot->switch_partition == 0
			&& host->cmd_atomic == false) {
			ret = dw_mci_cqe_switch(mmc, false);

			if(mrq->cmd->opcode==MMC_SLEEP_AWAKE ||
			   (mrq->cmd->opcode == MMC_SELECT_CARD && mrq->cmd->arg == 0) ||
			   (mrq->cmd->opcode == MMC_SWITCH && mrq->cmd->arg == DW_MCI_POWEROFF))
				host->cqe_reenable = 0;
			else
				host->cqe_reenable=1;
			if (ret) {
				pr_err("%s: disable cmdq failed !\n", __func__);
			}
			dw_mci_wait_status(host, &status);
		}

		if(mrq->cmd->opcode==MMC_SWITCH && mrq->cmd->arg == DW_MCI_CMDQ_ENABLED)
			slot->switch_partition = 0;

		if(mrq->cmd->opcode==MMC_ERASE_GROUP_START) {
			host->cmd_atomic = true;
			host->cqe_reenable=0;
		}

		if(host->cmd_atomic == true && mrq->cmd->opcode==MMC_SEND_STATUS) {
			host->cmd_atomic = false;
			host->cqe_reenable=1;
		}
	}

	slot->mrq = mrq;

	dw_mci_start_request(host, slot);

	tasklet_schedule(&host->tasklet);

	/*cmdq case needs extra check*/
	if(host->pdata && (host->pdata->caps2 & MMC_CAP2_CQE) &&
		host->cqe_reenable == 1) {
		if(mmc->card && mmc->card->ext_csd.cmdq_en==0) {
			ret = dw_mci_cqe_switch(mmc, true);
			host->cqe_reenable=0;
			if (ret)
				pr_err("%s: switch cmdq failed !\n", __func__);
			dw_mci_wait_status(host, &status);
		}
	}

	up_write(&host->cr_rw_sem);
}

static void dw_mci_enable_cd(struct dw_mci *host)
{
#if 0
	u32 temp;
#endif
	/*
	 * No need for CD if all slots have a non-error GPIO
	 * as well as broken card detection is found.
	 */
	if (host->slot->mmc->caps & MMC_CAP_NEEDS_POLL
		|| !mmc_card_is_removable(host->slot->mmc))
		return;

	if (mmc_gpio_get_cd(host->slot->mmc) < 0) {
#if 0
		/*Card Detect using IP regs is ToDo*/
		temp = mci_readl(host, NORMAL_INT_SIGNAL_EN_R);
		temp  |= (SDMMC_CARD_INSERTION_SIGNAL_EN | SDMMC_CARD_REMOVAL_SIGNAL_EN);
		mci_writel(host, NORMAL_INT_SIGNAL_EN_R, temp);
#endif
	}
}

static void dw_mci_cqhci_init(struct dw_mci *host)
{
	if(host->pdata && (host->pdata->caps2 & MMC_CAP2_CQE)) {
		host->cqe = cqhci_pltfm_init(host->pdev);
		if(PTR_ERR(host->cqe) == -EINVAL ||
		   PTR_ERR(host->cqe) == -ENOMEM ||
		   PTR_ERR(host->cqe)==-EBUSY) {
			pr_err("Unable to get the cmdq related attribute, err=%ld\n", PTR_ERR(host->cqe));
			host->cqe = 0;
			host->pdata->caps2 &= ~(MMC_CAP2_CQE|MMC_CAP2_CQE_DCMD);
		}
		else {
			host->cqe->ops = &dw_mci_cqhci_host_ops;
			cqhci_init(host->cqe, host->slot->mmc, 0);
		}
	}
}

int dw_mci_probe(struct dw_mci *host)
{
	const struct dw_mci_drv_data *drv_data = host->drv_data;
	int ret = 0;

	if (!host->pdata) {
		host->pdata = dw_mci_parse_dt(host);
		if (PTR_ERR(host->pdata) == -EPROBE_DEFER) {
			return -EPROBE_DEFER;
		} else if (IS_ERR(host->pdata)) {
			dev_err(host->dev, "platform data not available\n");
			return -EINVAL;
		}
	}

	host->biu_clk = devm_clk_get(host->dev, "biu");
	if (IS_ERR(host->biu_clk)) {
		dev_dbg(host->dev, "biu clock not available\n");
	} else {
		ret = clk_prepare_enable(host->biu_clk);
		if (ret) {
			dev_err(host->dev, "failed to enable biu clock\n");
			return ret;
		}
	}

	host->ciu_clk = devm_clk_get(host->dev, "ciu");
	if (IS_ERR(host->ciu_clk)) {
		dev_dbg(host->dev, "ciu clock not available\n");
		host->bus_hz = host->pdata->bus_hz;
	} else {
		ret = clk_prepare_enable(host->ciu_clk);
		if (ret) {
			dev_err(host->dev, "failed to enable ciu clock\n");
			goto err_clk_biu;
		}

		if (host->pdata->bus_hz) {
			ret = clk_set_rate(host->ciu_clk, host->pdata->bus_hz);
			if (ret)
				dev_warn(host->dev,
					"Unable to set bus rate to %uHz\n",
					 host->pdata->bus_hz);
		}
		host->bus_hz = clk_get_rate(host->ciu_clk);
	}

	if (!host->bus_hz) {
		dev_err(host->dev,
			"Platform data must supply bus speed\n");
		ret = -ENODEV;
		goto err_clk_ciu;
	}

	if (!IS_ERR(host->pdata->rstc)) {
		reset_control_assert(host->pdata->rstc);
		usleep_range(10, 50);
		reset_control_deassert(host->pdata->rstc);
	}

	timer_setup(&host->timer, dw_mci_cmd_timer, 0);

	spin_lock_init(&host->lock);
	spin_lock_init(&host->irq_lock);
	init_rwsem(&host->cr_rw_sem);
	tasklet_init(&host->tasklet, dw_mci_tasklet_func, (unsigned long)host);

	host->cqe_reenable=0;
	host->cmd_atomic=false;

	/*Allocate the descriptor memory*/
	host->desc_vaddr = dma_alloc_coherent(host->dev,
					      DW_MCI_DESC_LEN,
					      &host->desc_paddr,
					      GFP_KERNEL);
#if 0
	/*pio mode's parameters should be initialized here*/
#endif
	/*Initialize the eMMC IP related attribute*/
	dw_mci_setup(host);

	/* Do not use PIO mode by default,
	 * user can modify this setting by drv_data->init()
	 */
	host->use_dma = TRANS_MODE_DMA;

	/* using 32bit DMA by default,
	 * user can modify this setting by drv_data->init()
	 */
	host->dma_64bit_address = 0;

	/* This flag will be set 1 when doing tuning,
	 * we add this flag because
	 * some vendors might use other cmd instead of 21
	 * to tune phase under high speed interface.
	 * we use this flag to recognize if the system is under tuning stage.
	 */
	host->tuning = 0;

	/*Timing_setting is to avoid sending command
	 *before setting phase in hs200, hs400
	*/
	host->current_speed = 0;

	/*Do the rest of init for specific*/
	if (drv_data && drv_data->init) {
		ret = drv_data->init(host);
		if (ret) {
			dev_err(host->dev,
				"implementation specific init failed\n");
			goto err_dmaunmap;
		}
	}

	ret = dw_mci_init_slot(host);
	if (ret) {
		dev_err(host->dev, "slot 0 init failed\n");
		goto err_dmaunmap;
	}

	ret = devm_request_irq(host->dev, host->irq, dw_mci_interrupt,
				host->irq_flags, "dw-mci-cqe", host);
	if (ret)
		goto err_dmaunmap;

	/*After the slot intialization,
	 *now we have mmc data and can initialize cmdq if user enabled
	 */
	dw_mci_cqhci_init(host);

	/* Now that slots are all setup, we can enable card detect */
	dw_mci_enable_cd(host);

	return 0;

err_dmaunmap:
	if (!IS_ERR(host->pdata->rstc))
		reset_control_assert(host->pdata->rstc);
err_clk_ciu:
	clk_disable_unprepare(host->ciu_clk);

err_clk_biu:
	clk_disable_unprepare(host->biu_clk);

	return ret;
}
EXPORT_SYMBOL(dw_mci_probe);

void dw_mci_remove(struct dw_mci *host)
{
	dev_dbg(host->dev, "remove slot\n");
	if (host->slot)
		dw_mci_cleanup_slot(host->slot);

	if (!IS_ERR(host->pdata->rstc))
		reset_control_assert(host->pdata->rstc);

	clk_disable_unprepare(host->ciu_clk);
	clk_disable_unprepare(host->biu_clk);
}
EXPORT_SYMBOL(dw_mci_remove);

#ifdef CONFIG_PM
int dw_mci_cqe_runtime_suspend(struct device *dev)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	int ret = 0;

	if (host->pdata && (host->pdata->caps2 & MMC_CAP2_CQE)) {
		if(host->slot) {
			printk(KERN_ERR "[%s] cqe suspend\n", __func__);
			ret = cqhci_suspend(host->slot->mmc);
			if (ret) {
				pr_err("%s: cqe suspend failed\n", __func__);
				return ret;
			}
		}
	}

	ret = pm_runtime_force_suspend(dev);

	return ret;
}
EXPORT_SYMBOL(dw_mci_cqe_runtime_suspend);

int dw_mci_cqe_runtime_resume(struct device *dev)
{
	int ret = 0;
	struct dw_mci *host = dev_get_drvdata(dev);
	const struct dw_mci_drv_data *drv_data = host->drv_data;

	ret = pm_runtime_force_resume(dev);
	if (ret) {
		pr_err("%s: pm_runtime_force_resume failed.\n", __func__);
		return ret;
	}

	dw_mci_setup(host);
	if (drv_data && drv_data->init) {
		ret = drv_data->init(host);
		if (ret)
			pr_err("%s: implementation specific init failed\n", __func__);
	}

	init_completion(host->int_waiting);

	if (host->pdata && (host->pdata->caps2 & MMC_CAP2_CQE)) {
		if(host->slot) {
			printk(KERN_ERR "[%s] cqe resume\n", __func__);
			ret = cqhci_resume(host->slot->mmc);
			if (ret)
				pr_err("%s: cqe resume failed\n", __func__);
		}
	}

	return ret;
}
EXPORT_SYMBOL(dw_mci_cqe_runtime_resume);
#else
int dw_mci_cqe_runtime_suspend(struct device *dev)
{
	pr_err("user should enable CONFIG_PM\n");
	return 0;
}
EXPORT_SYMBOL(dw_mci_cqe_runtime_suspend);
int dw_mci_cqe_runtime_resume(struct device *dev)
{
	pr_err("user should enable CONFIG_PM\n");
	return 0;
}
EXPORT_SYMBOL(dw_mci_cqe_runtime_resume);
#endif /*CONFIG_PM*/

static int __init dw_mci_init(void)
{
	pr_info("Synopsys Designware Multimedia Card Interface Driver\n");
	return 0;
}

static void __exit dw_mci_exit(void)
{
}

module_init(dw_mci_init);
module_exit(dw_mci_exit);

MODULE_DESCRIPTION("DW Multimedia Card Interface driver");
MODULE_AUTHOR("NXP Semiconductor VietNam");
MODULE_AUTHOR("Imagination Technologies Ltd");
MODULE_LICENSE("GPL v2");
