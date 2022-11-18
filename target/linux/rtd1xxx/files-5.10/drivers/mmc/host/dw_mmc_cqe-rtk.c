/*
 * Copyright (c) 2014, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/mmc/mmc.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/pinctrl/consumer.h>
#include <linux/nvmem-consumer.h>

#include "dw_mmc_cqe.h"
#include "dw_mmc-pltfm.h"
#include "dw_mmc_cqe-rtk.h"
#include "../core/core.h"

static void dqs_delay_tap_setting(struct dw_mci *host, u32 dqs_dly);
static void cmd_delay_tap_setting(struct dw_mci *host, u32 cmd_dly_tape);
static void data_delay_tap_setting(struct dw_mci *host);
int mmc_cmdq_enable(struct mmc_card *card);
int mmc_cmdq_disable(struct mmc_card *card);
int mmc_switch(struct mmc_card *card, u8 set, u8 index, u8 value,
	       unsigned int timeout_ms);
int mmc_hw_reset(struct mmc_host *host);

int dw_mci_rtk_write_protect_cmd(struct mmc_host *mmc, u32 args, bool is_wrtie_protect)
{
	struct mmc_request mrq = {};
	struct mmc_command cmd = {};
	int err = 0;

	if (is_wrtie_protect)
		cmd.opcode         = MMC_SET_WRITE_PROT;
	else
		cmd.opcode         = MMC_CLR_WRITE_PROT;

	cmd.arg            = args;
	cmd.flags          = MMC_CMD_AC|MMC_RSP_SPI_R1B | MMC_RSP_R1B;
	mrq.cmd = &cmd;
	mrq.data = NULL;

	mmc_wait_for_req(mmc, &mrq);

	if (cmd.error) {
		err = cmd.error;
		goto out;
	}

out:
	mdelay(1);
	return err;
}

u32 swap_endian(u32 input)
{
	u32 output;
	output = (input & 0xff000000)>>24|
	(input & 0x00ff0000)>>8|
	(input & 0x0000ff00)<<8|
	(input & 0x000000ff)<<24;
	return output;
}

int dw_mci_rtk_query_protect_cmd(struct mmc_host *mmc, unsigned long addr, u32 *src)
{
	struct mmc_request mrq = {};
	struct mmc_command cmd = {};
	struct mmc_data data = {};
	struct scatterlist sg;
	int err = 0;
	int size = 4;
	u8 *data_buf;

	cmd.arg = addr;
	cmd.opcode = MMC_SEND_WRITE_PROT;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	cmd.data = &data;
	mrq.cmd = &cmd;

	data_buf = kzalloc(size, GFP_KERNEL);

	mrq.data = &data;
	data.flags = MMC_DATA_READ;
	data.blksz = size;
	data.blocks = 1;
	data.timeout_ns = 150 * NSEC_PER_MSEC;

	data.sg = &sg;
	data.sg_len = 1;
	sg_init_one(&sg, data_buf, size);

	mmc_wait_for_req(mmc, &mrq);

	if (cmd.error) {
		err = cmd.error;
		goto out;
	}
	if (data.error) {
		err = data.error;
		goto out;
	}

	memcpy(src, data_buf, size);
	*src = swap_endian(*src);
out:

	kfree(data_buf);
	return err;
}

static int dw_mci_rtk13xx_switch_to_uda(struct mmc_host *mmc)
{
	u8 part_config = mmc->card->ext_csd.part_config;
	int ret = 0;

	/* if current partition is not uda */
	if ((mmc->card->ext_csd.part_config&0x7) != 0) {
		if (mmc->card->ext_csd.cmdq_en) {
			ret = mmc_cmdq_disable(mmc->card);
			if (ret)
				return ret;
		}
		pr_info("%s: switch to uda partition...\n", __func__);
		part_config &= ~EXT_CSD_PART_CONFIG_ACC_MASK;
		part_config |= 0x0;     //uda partition

		ret = mmc_switch(mmc->card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_PART_CONFIG, part_config,
				 mmc->card->ext_csd.part_time);

		if (mmc->card->reenable_cmdq && !mmc->card->ext_csd.cmdq_en)
			ret = mmc_cmdq_enable(mmc->card);
	}
	return ret;
}

static int dw_mci_rtk13xx_restore_from_uda(struct mmc_host *mmc)
{
	int ret = 0;

	if ((mmc->card->ext_csd.part_config&0x7) != 0) {
		if (mmc->card->ext_csd.cmdq_en) {
			ret = mmc_cmdq_disable(mmc->card);
			if (ret)
				return ret;
		}
		pr_info("%s: switch back to original partition...\n", __func__);

		ret = mmc_switch(mmc->card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_PART_CONFIG, mmc->card->ext_csd.part_config,
				 mmc->card->ext_csd.part_time);
		if (mmc->card->reenable_cmdq && !mmc->card->ext_csd.cmdq_en)
			ret = mmc_cmdq_enable(mmc->card);
	}

	return ret;
}

static ssize_t protect_region_setting_dev_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	struct mmc_host *mmc = host->slot->mmc;
	struct dw_mci_rtkemmc_host *priv = host->priv;
	unsigned int wpg_unit = 0;
	int ret;
	u32 src;

	wpg_unit = mmc->card->ext_csd.raw_hc_erase_grp_size *
			mmc->card->ext_csd.raw_hc_erase_gap_size * 1024;

	pr_err("echo Q > protect_region_setting to show all eMMC protect group regions\n");
	pr_err("echo C > protect_region_setting to clear all eMMC protect group region\n");
	pr_err("echo S > protect_region_setting to set eMMC protect the N-th group region\n");
	pr_err("write protect unit = 0x%x blocks\n", wpg_unit);
	pr_err("start from Nth protect group, the following 32 protect group bitmap =\n");
	mmc_claim_host(mmc);
	ret = dw_mci_rtk13xx_switch_to_uda(mmc);

	dw_mci_rtk_query_protect_cmd(mmc, priv->protect_start, &src);

	ret = dw_mci_rtk13xx_restore_from_uda(mmc);
	mmc_release_host(mmc);

	return sprintf(buf, "0x%x\n", src);
}

static ssize_t protect_region_setting_dev_store(struct device *dev,
	    struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	struct mmc_host *mmc = host->slot->mmc;
	struct dw_mci_rtkemmc_host *priv = host->priv;
	unsigned long args = 0;
	unsigned int wpg_unit = 0;
	int ret;
	u32 src;
	int max_sectors;

	mmc_claim_host(mmc);

	ret = dw_mci_rtk13xx_switch_to_uda(mmc);

	wpg_unit = mmc->card->ext_csd.raw_hc_erase_grp_size *
		    mmc->card->ext_csd.raw_hc_erase_gap_size * 1024;

	if (buf[0]=='Q') {
		/*wpg_unit is write protect grpoup size (sector unit)*/
		dev_info(dev, "%s curret eMMC protect region status:\n", __func__);
		max_sectors = mmc->card->ext_csd.sectors;
		for (args = 0; args < max_sectors; args += (wpg_unit * 0x20)) {
			dw_mci_rtk_query_protect_cmd(mmc, args, &src);
			pr_err("from sector 0x%x = 0x%x\n", args, src);
		}
	} else if (buf[0]=='C') {
		dev_info(dev, "%s clear all eMMC protect group regions...\n", __func__);
		max_sectors = mmc->card->ext_csd.sectors;
		for (args = 0; args < max_sectors; args += wpg_unit) {
			dw_mci_rtk_write_protect_cmd(mmc, args, 0);
		}
	} else if(buf[0]=='S') {
		dev_info(dev, "%s set the eMMC protect group  regions...\n", __func__);
		dw_mci_rtk_write_protect_cmd(mmc, priv->protect_start, 1);
	}

	ret = dw_mci_rtk13xx_restore_from_uda(mmc);

	mmc_release_host(mmc);

	return count;
}
DEVICE_ATTR(protect_region_setting, S_IRUGO | S_IWUSR,
		protect_region_setting_dev_show, protect_region_setting_dev_store);

static ssize_t protect_region_start_dev_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	struct dw_mci_rtkemmc_host *priv = host->priv;

	return sprintf(buf, "0x%x\n", priv->protect_start);
}

static ssize_t protect_region_start_dev_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	struct dw_mci_rtkemmc_host *priv = host->priv;
	int i = 0, sz = 0;
	u32 ch;
	char *substring = NULL;

	for(sz=0; sz < strlen(buf); sz++) {
		ch = (u32) buf[sz];
		if(ch < 0x30 || ch > 0x39) {
			break;
		}
	}

	substring = kzalloc(sz, GFP_KERNEL);
	for(i = 0; i < sz; i++)
		substring[i] = buf[i];

	priv->protect_start = simple_strtoul(substring, NULL, 16);

	if(substring)
		kfree(substring);

	pr_err("protect_start = 0x%x\n", priv->protect_start);

	return count;
}

DEVICE_ATTR(protect_region_start, S_IRUGO | S_IWUSR,
		protect_region_start_dev_show, protect_region_start_dev_store);

static ssize_t protect_region_unit_dev_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	struct mmc_host *mmc = host->slot->mmc;
	struct dw_mci_rtkemmc_host *priv = host->priv;

	priv->protect_unit = mmc->card->ext_csd.raw_hc_erase_grp_size *
				mmc->card->ext_csd.raw_hc_erase_gap_size * 1024;

	pr_err("the protect group unit(blocks)=\n");

	return sprintf(buf, "0x%x\n", priv->protect_unit);
}

static ssize_t protect_region_unit_dev_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

DEVICE_ATTR(protect_region_unit, S_IRUGO | S_IWUSR,
		protect_region_unit_dev_show, protect_region_unit_dev_store);

static void reset_fifo(struct dw_mci *host)
{
	mci_writel(host, OTHER1, mci_readl(host, OTHER1) & (~SDMMC_TOP_RST_N_FIFO));
	udelay(1);
	mci_writel(host, OTHER1, mci_readl(host, OTHER1) | SDMMC_TOP_RST_N_FIFO);
}

static int dw_mci_rtk_set_pinstates(struct dw_mci_rtkemmc_host *priv,
                                        unsigned char timing)
{
	switch (timing) {
	case MMC_TIMING_UHS_SDR50:
		return pinctrl_select_state(priv->pinctrl,
					    priv->pins_sdr50);

	case MMC_TIMING_UHS_DDR50:
		return pinctrl_select_state(priv->pinctrl,
					    priv->pins_ddr50);
	case MMC_TIMING_MMC_HS200:
		return pinctrl_select_state(priv->pinctrl,
					    priv->pins_hs200);

	case MMC_TIMING_MMC_HS400:
		return pinctrl_select_state(priv->pinctrl,
					    priv->pins_hs400);
	default:
		return pinctrl_select_state(priv->pinctrl,
					    priv->pins_default);
	}
}

static void setup_clk_div(struct dw_mci_slot *slot)
{
	struct dw_mci *host = slot->host;
	unsigned int clock = slot->clock;
	u32 div = 0;

	slot->mmc->actual_clock = 0;

	if (clock != host->current_speed) {
		div = host->bus_hz / clock;
		if (host->bus_hz % clock)
			div += 1;

		if (clock != slot->__clk_old) {
			/* Silent the verbose log if calling from PM context */
			printk(KERN_ERR "Bus speed (slot %d) = %dHz (slot req %dHz, actual %dHZ div = %d)\n",
				slot->id, host->bus_hz, clock,
				host->bus_hz / div, div);
		}

		slot->__clk_old = clock;
		slot->mmc->actual_clock = host->bus_hz / div;

		/*In RealTek Stark platform, they only support div 1,4, and 512*/
		if(div > 4) div =  512;

		switch(div) {
		case SDMMC_CLK_DIV1:
			mci_writel(host, CKGEN_CTL,
				mci_readl(host, CKGEN_CTL) & (~SDMMC_CRC_CLK_DIV_EN));
			break;
		case SDMMC_CLK_DIV4:
			mci_writel(host, CKGEN_CTL,
				mci_readl(host, CKGEN_CTL) | SDMMC_CLK_INV_DIV_SEL);
			mci_writel(host, CKGEN_CTL,
				mci_readl(host, CKGEN_CTL) | SDMMC_CRC_CLK_DIV_EN);
			break;
		case SDMMC_CLK_DIV512:
			mci_writel(host, CKGEN_CTL,
				mci_readl(host, CKGEN_CTL) & (~SDMMC_CLK_INV_DIV_SEL));
			mci_writel(host, CKGEN_CTL,
				mci_readl(host, CKGEN_CTL) | SDMMC_CRC_CLK_DIV_EN);
			break;
		default:
			printk(KERN_ERR "%s: default case div = %d\n", __func__, div);
			if(div != 0)
				mci_writel(host, CKGEN_CTL,
					mci_readl(host, CKGEN_CTL) & (~SDMMC_CRC_CLK_DIV_EN));
			break;
		}
	}

	host->current_speed = clock;
}

static void dw_mci_rtk_phase_tuning(struct dw_mci *host,
					u32 tx_phase, u32 rx_phase)
{
	struct dw_mci_rtkemmc_host *priv = host->priv;
	u32 t1=10;
	u32 t2=3;

	clk_disable(host->ciu_clk);
	udelay(t1);

	mci_writel(host, OTHER1, mci_readl(host, OTHER1) | BIT(10));

        if (!IS_ERR(priv->vp0) && !IS_ERR(priv->vp1)) {
                if (tx_phase != 0xff) {
                        clk_set_phase(priv->vp0,  (tx_phase*360)/32);

                }
                if (rx_phase != 0xff) {
                        clk_set_phase(priv->vp1, (rx_phase*360)/32);
                }
        }

	udelay(t2);

	mci_writel(host, OTHER1, mci_readl(host, OTHER1) & ~BIT(10));

        clk_enable(host->ciu_clk);

	wait_done(host, (u32*)(host->regs + SDMMC_PLL_STATUS), BIT(0), BIT(0));
	wait_done(host, (u32*)(host->regs + SDMMC_CLK_CTRL_R), (BIT(0)|BIT(1)), 0x3);
	mci_writeb(host, SW_RST_R, BIT(1)|BIT(2));
	wait_done(host, (u32*)(host->regs + SDMMC_CLK_CTRL_R), (BIT(25)|BIT(26)), 0);

	reset_fifo(host);
}

static void adjust_window(struct dw_mci_rtkemmc_host *priv,
                          const char *w_type,
                          unsigned int loop_cnt,
                          unsigned int window)
{
	switch(loop_cnt) {
	case 10:
		printk(KERN_ERR "try pad driving 3: %s = 0x%08x\n", w_type, window);
		pinctrl_select_state(priv->pinctrl, priv->pins_tune3);
		break;
	case 20:
		printk(KERN_ERR "try pad driving 2: %s = 0x%08x\n", w_type, window);
		pinctrl_select_state(priv->pinctrl, priv->pins_tune2);
                break;
	case 30:
		printk(KERN_ERR "try pad driving 1: %s = 0x%08x\n", w_type, window);
		pinctrl_select_state(priv->pinctrl, priv->pins_tune1);
		break;
	case 40:
		printk(KERN_ERR "try pad driving 0: %s = 0x%08x\n", w_type, window);
		pinctrl_select_state(priv->pinctrl, priv->pins_tune0);
		break;
	case 60:
		printk(KERN_ERR "loop cnt %d: %s = 0x%08x, cannot find a proper phase\n",
			loop_cnt, w_type, window);
	default:
		break;
	}
}

static int search_best(u32 window, u32 range)
{
	int i = 0, j = 1, k = 0, max = 0;
	int window_temp[32];
	int window_start[32];
	int window_end[32];
	int window_max = 0;
	int window_best = 0;
	int parse_end = 1;

	for (i = 0; i < 0x20; i++) {
		window_temp[i] = 0;
		window_start[i] = 0;
		window_end[i] = -1;
	}

	i=0;

	while (( i < (range-1)) && ( k < (range-1))) {
		parse_end=0;
		for (i = window_end[j-1]+1; i < range; i++){
			if (((window >> i) & 1) == 1 ) {
				window_start[j] = i;
				break;
			}
		}
		if (i == range)
			break;

		for (k = window_start[j] + 1; k < range; k++) {
			if(((window >> k) & 1) == 0) {
				window_end[j] = k-1;
				parse_end = 1;
				break;
			}
		}
		if (parse_end == 0) {
			window_end[j] = range - 1;
		}
		j++;
	}
	for (i = 1; i < j; i++)
		window_temp[i] = window_end[i] - window_start[i] + 1;

	if ((((window) & 1) == 1)&&(((window >> (range - 1)) & 1) == 1)) {
		window_temp[1] = window_temp[1] + window_temp[j - 1];
		window_start[1] = window_start[j - 1];
	}
	for (i = 1; i < j; i++) {
		if (window_temp[i] > window_max) {
			window_max = window_temp[i];
			max=i;
		}
	}

	if (window == ~0UL) {
		window_best = 0x10;
	}
	else if ((window == 0xffff) && (range == 0x10)) {
		window_best = 0x8;
	}
	else if ((((window & 1) == 1)&&(((window >> (range-1)) & 1) == 1))
		&& (max == 1)) {
		window_best = (((window_start[max] + window_end[max] +
				range) / 2) &(range - 1));
	}
	else {
		window_best = ((window_start[max] + window_end[max]) / 2) & 0x1f;
	}

	/*We set a criteria for 6 continuous points*/
	if(window_max > 6)
		return window_best;
	else
		return 0xff;
}

static int dw_mci_rtk_send_tuning(struct mmc_host *mmc, u32 opcode, u32 arg, int size)
{
	struct mmc_request mrq = {};
	struct mmc_command cmd = {};
	struct mmc_data data = {};
	struct scatterlist sg;
	int i, err = 0;
	u8 *data_buf = NULL;;

	cmd.opcode = opcode;
	cmd.arg = arg;
	if(opcode == MMC_SEND_STATUS) {
		cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	}
	else
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	mrq.cmd = &cmd;
	mrq.data = NULL;

	if(opcode == MMC_WRITE_MULTIPLE_BLOCK || opcode == MMC_READ_MULTIPLE_BLOCK) {
		data_buf = kzalloc(size, GFP_KERNEL);
		if (!data_buf)
			return -ENOMEM;

		mrq.data = &data;

		if (opcode == MMC_WRITE_MULTIPLE_BLOCK) {
			for (i = 0; i < size/4; i++)
				*(u32 *)(data_buf + i*4) = i;
			data.flags = MMC_DATA_WRITE;
		}
		else
			data.flags = MMC_DATA_READ;

		data.blksz = 512;
		data.blocks = size / data.blksz;

		data.timeout_ns = 150 * NSEC_PER_MSEC;

		data.sg = &sg;
		data.sg_len = 1;
		sg_init_one(&sg, data_buf, size);
	}

	mmc_wait_for_req(mmc, &mrq);

	if (cmd.error) {
		err = cmd.error;
		goto out;
	}

	if (data.error) {
		err = data.error;
		goto out;
	}

out:
	if(data_buf)
		kfree(data_buf);
	return err;
}

static int dw_mci_rtk_execute_tuning(struct dw_mci_slot *slot, u32 opcode)
{
	struct dw_mci *host = slot->host;
	struct dw_mci_rtkemmc_host *priv = host->priv;
	struct mmc_host *mmc = slot->mmc;
	u32 tx_window = 0, rx_window = 0;
	u32 tx_best = 0, rx_best = 0;
	u32 tx_range, rx_range = 0x20;
	unsigned int bitmap=0;
	unsigned int max=0;
	int i,j, ret = 0, rsp=0;
	unsigned int loop_cnt = 0;
	unsigned int reg;
	bool fail = false;
	bool dqs_retry = false;
	u32 src = 0x0;

	if(mmc->doing_retune == 1) {
		dev_err(mmc_dev(mmc), "%s: retune case.\n", __func__);
		goto out;
	}

	ret = dw_mci_rtk_set_pinstates(priv, mmc->ios.timing);
	if (ret) {
		dev_err(mmc_dev(mmc),
			"Failed to set pinstate err=%d\n", ret);
		ret = -ENODEV;
		goto out;
	}

	host->tuning = 1;

	do {
		if (mmc->ios.timing == MMC_TIMING_MMC_HS400)
			loop_cnt=0;
		else if(mmc->ios.timing == MMC_TIMING_MMC_HS200)
			loop_cnt=20;
		else
			loop_cnt=30;

		/* rx tuning */
		do {
			for(i = 0; i < rx_range; i++) {
				dw_mci_rtk_phase_tuning(host, 0xff, i);

				if(mmc->ios.timing == MMC_TIMING_MMC_HS200)
					rsp = mmc_send_tuning(mmc, MMC_SEND_TUNING_BLOCK_HS200, NULL);
				else
					rsp = dw_mci_rtk_send_tuning(mmc, MMC_SEND_STATUS, 0x10000, 0);
				if (rsp)
					rx_window = rx_window & ~BIT(i);
				else
					rx_window = rx_window | BIT(i);
			}

			if (rx_window == 0) {
				dev_err(host->dev, "rx_window = 0, cannot find a proper"
					   " rx phase \n");
				ret = -EFAULT;
				goto out;
			}
			else if(rx_window == 0xffffffff) {
				loop_cnt++;
				adjust_window(priv, "rx_window", loop_cnt, rx_window);
			}
		} while(rx_window == 0xffffffff && loop_cnt < 60);

		rx_best = search_best(rx_window, rx_range);
		if(rx_best == 0xff) {
			dev_err(host->dev, "no 6 continuous points for rx phase\n");
			ret = -EFAULT;
			goto out;
		}
		dw_mci_rtk_phase_tuning(host, 0xff, rx_best);
		printk(KERN_ERR "rx_window=0x%x, rx_best=0x%x\n", rx_window, rx_best);


		/*dqs tuning in HS400 mode*/
		if(mmc->ios.timing == MMC_TIMING_MMC_HS400) {
			if(fail == true)
				pinctrl_select_state(priv->pinctrl, priv->pins_tune4);
			else
				pinctrl_select_state(priv->pinctrl, priv->pins_hs400);

			/*reset the bitmap, max, and j for each wcmd_rty*/
			bitmap=0;
			max=0;
			j=0;

			for(i=0; i<0x20; i++) {
				/*criteria is more than 5 continuous tap sample point*/
				if(j>=5)
					max = j;
				/*find the max tap length*/
				if(j==0 && max!=0)
					break;

				dqs_delay_tap_setting(host, (i<<1));

				if(dw_mci_rtk_send_tuning(mmc, MMC_READ_MULTIPLE_BLOCK, 0x100, 1024) != 0) {
					j = 0;
				}
				else {
					j++;
					bitmap |= BIT(i);
				}
			}

			if(max==0) {
				if(fail == false) {
					pr_err("DQS_RETRY: dqs tap bitmap= 0x%x\n", bitmap);
					fail = true;
					dqs_retry = true;
					pr_err("try pad driving 7\n");
					pinctrl_select_state(priv->pinctrl, priv->pins_tune4);
				}
				else {
					ret = -EFAULT;
					dev_err(mmc_dev(mmc),
						"Cannot find a proper dqs window, dqs tap bitmap=0x%x !\n", bitmap);
					goto out;
				}
			}
			else {
				reg = mci_readl(host, DQS_CTRL1)-2-((max/2)*2);
				/*set the dqs dly tape*/
				dqs_delay_tap_setting(host, reg);
				dqs_retry = false;
				printk(KERN_ERR "max sample point=%d, bitmap=0x%x, DQS=0x%x\n",
					max, bitmap, mci_readl(host, DQS_CTRL1));
			}
		}
	} while(dqs_retry == true);

	/*tx tuning */
	if (mmc->ios.timing == MMC_TIMING_MMC_HS400) {
		tx_range = 0x10;
		loop_cnt=0;
		if(fail == true)
			pinctrl_select_state(priv->pinctrl, priv->pins_tune4);
		else
			pinctrl_select_state(priv->pinctrl, priv->pins_hs400);
	}
	else if(mmc->ios.timing == MMC_TIMING_MMC_HS200){
		tx_range = 0x20;
		loop_cnt=20;
		if(fail == true)
			pinctrl_select_state(priv->pinctrl, priv->pins_tune4);
		else
			pinctrl_select_state(priv->pinctrl, priv->pins_hs200);
	}
	else {
		tx_range = 0x10;
		loop_cnt=30;
	}

	do {
		dw_mci_rtk_query_protect_cmd(mmc, 0, &src);
		pr_err("check write protect region 0 during tuning: 0x%x\n", src);
		if(src & BIT(0))
			dw_mci_rtk_write_protect_cmd(mmc, 0, 0);

		for(i = 0; i < tx_range; i++) {
			dw_mci_rtk_phase_tuning(host, i, 0xff);
			if (dw_mci_rtk_send_tuning(mmc,
						       MMC_WRITE_MULTIPLE_BLOCK, 0xfe, 1024))
				tx_window = tx_window & ~BIT(i);
			else
				tx_window = tx_window | BIT(i);
		}

		if(src & BIT(0))
			dw_mci_rtk_write_protect_cmd(mmc, 0, 1);

		if (tx_window == 0) {
			dev_err(host->dev, "tx_window = 0, cannot find a proper "
					   "tx phase\n");
			ret = -EFAULT;
			goto out;
		}
		else if((tx_window == 0xffff && mmc->ios.timing == MMC_TIMING_MMC_HS400) ||
			(tx_window == 0xffffffff && mmc->ios.timing == MMC_TIMING_MMC_HS200)) {
			loop_cnt++;
			adjust_window(priv, "tx_window", loop_cnt, tx_window);
		}
	} while(((tx_window == 0xffff && mmc->ios.timing == MMC_TIMING_MMC_HS400) ||
		(tx_window == 0xffffffff && mmc->ios.timing == MMC_TIMING_MMC_HS200)) && loop_cnt < 60);

	if(fail == true)
		pinctrl_select_state(priv->pinctrl, priv->pins_tune4);
	else if (mmc->ios.timing == MMC_TIMING_MMC_HS400)
		pinctrl_select_state(priv->pinctrl, priv->pins_hs400);
	else if (mmc->ios.timing == MMC_TIMING_MMC_HS200)
		pinctrl_select_state(priv->pinctrl, priv->pins_hs200);

	tx_best = search_best(tx_window, tx_range);
	if(tx_best == 0xff) {
		dev_err(host->dev, "no 6 continuous points for tx phase\n");
		ret = -EFAULT;
		goto out;
	}
	dw_mci_rtk_phase_tuning(host, tx_best, 0xff);
	printk(KERN_ERR "tx_window=0x%x, tx_best=0x%x\n", tx_window, tx_best);

out:
	/*We send cmd 13 again because the eMMC handling might send command 12 more than twice.
	  After kernel 5.4, system might send cmd13 first before issuing any command,
	  user will see the illegal command status because the emmc tuning error handling*/
	dw_mci_rtk_send_tuning(mmc, MMC_SEND_STATUS, 0x10000, 0);
	host->tuning = 0;

	return ret;
}

static void dw_mci_rtk_set_ios(struct dw_mci_slot *slot, struct mmc_ios *ios)
{
	struct dw_mci *host = slot->host;
	int ret;

	clk_disable(host->ciu_clk);
	udelay(10);

	ret = clk_set_rate(host->ciu_clk, ios->clock);
	if (ret)
		dev_warn(host->dev, "failed to set rate %uHz\n", ios->clock);

	host->bus_hz = clk_get_rate(host->ciu_clk);

	udelay(60);
	clk_enable(host->ciu_clk);

	wait_done(host, (u32*)(host->regs + SDMMC_PLL_STATUS), BIT(0), BIT(0));
	wait_done(host, (u32*)(host->regs + SDMMC_CLK_CTRL_R), (BIT(0)|BIT(1)), 0x3);
	mci_writeb(host, SW_RST_R, BIT(1)|BIT(2));
	wait_done(host, (u32*)(host->regs + SDMMC_CLK_CTRL_R), (BIT(25)|BIT(26)), 0);

	clk_disable(host->ciu_clk);
	udelay(6);

	setup_clk_div(slot);

	mci_writel(host, CKGEN_CTL, mci_readl(host, CKGEN_CTL) & ~BIT(20));
	udelay(6);

	clk_enable(host->ciu_clk);
	udelay(6);

	wait_done(host, (u32*)(host->regs + SDMMC_PLL_STATUS), BIT(0), BIT(0));
	wait_done(host, (u32*)(host->regs + SDMMC_CLK_CTRL_R), (BIT(0)|BIT(1)), 0x3);
	mci_writeb(host, SW_RST_R, BIT(1)|BIT(2));
	wait_done(host, (u32*)(host->regs + SDMMC_CLK_CTRL_R), (BIT(25)|BIT(26)), 0);

	mci_writel(host, OTHER1, mci_readl(host, OTHER1) & (~SDMMC_TOP_RST_N_FIFO));
	udelay(10);
	mci_writel(host, OTHER1, mci_readl(host, OTHER1) | SDMMC_TOP_RST_N_FIFO);
	udelay(10);
}

static int dw_mci_rtk_switch_voltage(struct mmc_host *mmc, struct mmc_ios *ios)
{
	/*In RealTek Platform, they do not use any 1.8V related settings*/
        return 0;
}

static void dqs_delay_tap_setting(struct dw_mci *host,
				  u32 dqs_dly)
{
	u32 regs;

	regs = mci_readl(host, DQS_CTRL1) & ~BIT(8);
	mci_writel(host, DQS_CTRL1, regs);
	mci_writel(host, DQS_CTRL1, dqs_dly);

	regs = dqs_dly | BIT(8);
	mci_writel(host, DQS_CTRL1, regs);
}

static void data_delay_tap_setting(struct dw_mci *host)
{
	u32 regs;
	struct dw_mci_rtkemmc_host *priv = host->priv;

	regs = mci_readl(host, RDQ_CTRL0) & (~SDMMC_FW_SET);
	mci_writel(host, RDQ_CTRL0, regs);

	if(priv->rdq_ctrl == 0)
		return;

	mci_writel(host, RDQ_CTRL0, priv->rdq_ctrl);
	mci_writel(host, RDQ_CTRL1, priv->rdq_ctrl);
	mci_writel(host, RDQ_CTRL2, priv->rdq_ctrl);
	mci_writel(host, RDQ_CTRL3, priv->rdq_ctrl);
	mci_writel(host, RDQ_CTRL4, priv->rdq_ctrl);
	mci_writel(host, RDQ_CTRL5, priv->rdq_ctrl);
	mci_writel(host, RDQ_CTRL6, priv->rdq_ctrl);
	mci_writel(host, RDQ_CTRL7, priv->rdq_ctrl);

	regs = mci_readl(host, RDQ_CTRL0) | SDMMC_FW_SET;
	mci_writel(host, RDQ_CTRL0, regs);
}

static void cmd_delay_tap_setting(struct dw_mci *host,
				  u32 cmd_dly_tape)
{
	u32 regs;

	regs = mci_readl(host, WCMD_CTRL) & ~BIT(7);
	mci_writel(host, WCMD_CTRL, regs);

	if(cmd_dly_tape==0)
		return;

	mci_writel(host, WCMD_CTRL, cmd_dly_tape);

	regs = cmd_dly_tape | BIT(7);
	mci_writel(host, WCMD_CTRL, regs);
}

static int dw_mci_rtk_prepare_hs400_tuning(struct dw_mci *host, struct mmc_ios *ios)
{
	dqs_delay_tap_setting(host, 0x88);

	data_delay_tap_setting(host);

	cmd_delay_tap_setting(host, 0);

	return 0;
}

static void dw_mci_rtk_hs400_complete(struct mmc_host *mmc)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	const struct dw_mci_drv_data *drv_data = host->drv_data;
	struct dw_mci_rtkemmc_host *priv = host->priv;
	int err = -EINVAL;

	if (drv_data && drv_data->execute_tuning)
		err = drv_data->execute_tuning(slot,
			MMC_SEND_TUNING_BLOCK_HS200);

	if(err!=0) {
		dev_err(host->dev, "HS400 tuning failed and down speed to hs200, err=%d\n",
			err);
		host->pdata->caps &= ~MMC_CAP_1_8V_DDR;
		host->pdata->caps2 &= ~MMC_CAP2_HS400_1_8V;
		mmc->card->mmc_avail_type &= ~EXT_CSD_CARD_TYPE_HS400_1_8V;
		priv->emmc_mode = 2;
		mmc_hw_reset(mmc);
	}
}

static void dw_mci_rtk_init_card(struct mmc_host *host, struct mmc_card *card)
{
	/*In Realtek Platform, we need to attach eMMC card onto mmc host
	  during eMMC initialization because of the following reason:
	  When system cannot run the hs400, we need to down speed to hs200
	  and call mmc_hw_reset and modify the mmc card attribute through mmc host.
	  At this moment, system will show errors if host->card = NULL.
	 */
	host->card = card;
}

static int dw_mci_rtk_parse_dt(struct dw_mci *host)
{
	struct dw_mci_rtkemmc_host *priv;
	const u32 *prop;
	int size;

	priv = devm_kzalloc(host->dev, sizeof(struct dw_mci_rtkemmc_host), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pinctrl = devm_pinctrl_get(host->dev);
	if (IS_ERR(priv->pinctrl))
		dev_dbg(host->dev, "no pinctrl\n");

	priv->pins_default = pinctrl_lookup_state(priv->pinctrl,
                                                PINCTRL_STATE_DEFAULT);
	if (IS_ERR(priv->pins_default))
		dev_warn(host->dev, "could not get default state\n");

	priv->pins_sdr50 = pinctrl_lookup_state(priv->pinctrl,
						"sdr50");
	if (IS_ERR(priv->pins_sdr50))
		dev_warn(host->dev, "could not get sdr50 state\n");

	priv->pins_hs200 = pinctrl_lookup_state(priv->pinctrl,
						"hs200");
        if (IS_ERR(priv->pins_hs200))
                dev_warn(host->dev, "could not get hs200 state\n");

	priv->pins_hs400 = pinctrl_lookup_state(priv->pinctrl,
						"hs400");
	if (IS_ERR(priv->pins_hs400))
		dev_warn(host->dev, "could not get hs400 state\n");

	priv->pins_tune0 = pinctrl_lookup_state(priv->pinctrl,
						"tune0");
	if (IS_ERR(priv->pins_tune0))
		dev_warn(host->dev, "could not get tune0 state\n");

	priv->pins_tune1 = pinctrl_lookup_state(priv->pinctrl,
						"tune1");
	if (IS_ERR(priv->pins_tune1))
		dev_warn(host->dev, "could not get tune1 state\n");

	priv->pins_tune2 = pinctrl_lookup_state(priv->pinctrl,
						"tune2");
	if (IS_ERR(priv->pins_tune2))
		dev_warn(host->dev, "could not get tune2 state\n");

	priv->pins_tune3 = pinctrl_lookup_state(priv->pinctrl,
						"tune3");
	if (IS_ERR(priv->pins_tune3))
		dev_warn(host->dev, "could not get tune3 state\n");

	priv->pins_tune4 = pinctrl_lookup_state(priv->pinctrl,
						"tune4");

	if (IS_ERR(priv->pins_tune4))
		dev_warn(host->dev, "could not get tune4 state\n");

	priv->vp0 = devm_clk_get(host->dev, "vp0");
	if (IS_ERR(priv->vp0)) {
		dev_err(host->dev, "could not get vp0 clk\n");
	}

	priv->vp1 = devm_clk_get(host->dev, "vp1");
	if (IS_ERR(priv->vp1)) {
		dev_err(host->dev, "could not get vp1 clk\n");
	}

	priv->emmc_mode = 0;
	prop = of_get_property(host->dev->of_node, "speed-step", &size);
	if (prop) {
		priv->emmc_mode = of_read_number(prop, 1);
		printk(KERN_ERR "[%s] emmc mode : %d\n", __func__, priv->emmc_mode);
	} else {
		printk(KERN_ERR "[%s] use default emmc sdr50 mode !\n", __func__);
	}

	/*In Parker platform, only PKG SOC can run HS400 mode*/
	if(priv->emmc_mode == 3 &&
		of_device_is_compatible(host->dev->of_node, "rtd13xxd-dw-cqe-emmc")) {
		struct nvmem_cell	*cell;
		unsigned int *buf;
		size_t buf_size;

		cell = nvmem_cell_get(host->dev, "emmc_pkg_cal");
		if (IS_ERR(cell)) {
			pr_err("cannot get the uuid info !\n");
		}
		buf = nvmem_cell_read(cell, &buf_size);
		printk(KERN_ERR "%s: otp_emmc_pkg_cal = 0x%x\n", __func__, buf[0]);
		if((buf[0] & BIT(0)) == 0)
			 priv->emmc_mode = 2;

		kfree(buf);
		nvmem_cell_put(cell);
	}

	priv->is_cqe = 0;
	prop = of_get_property(host->dev->of_node, "cqe", &size);
	if (prop) {
		priv->is_cqe = of_read_number(prop, 1);
		printk(KERN_ERR "[%s] cmdq mode : %d\n", __func__, priv->is_cqe);
	} else {
		printk(KERN_ERR "[%s] use default eMMC legacy mode !\n", __func__);
	}
	prop = of_get_property(host->dev->of_node, "rdq-ctrl", &size);
	if (prop) {
		priv->rdq_ctrl = of_read_number(prop, 1);
		printk(KERN_ERR "[%s] get rdq-ctrl : %u\n",__func__, priv->rdq_ctrl);
	} else {
		priv->rdq_ctrl = 0;
		printk(KERN_ERR "[%s] no dqs_dly_tape switch node, use default 0x0 !! \n",__func__);
	}

	priv->m2tmx = syscon_regmap_lookup_by_phandle(host->dev->of_node, "realtek,m2tmx");
	if (IS_ERR_OR_NULL(priv->m2tmx))
		printk(KERN_ERR "%s: can not get m2mtx node.\n", __func__);

	host->priv = priv;

	return 0;
}

static int dw_mci_rtk_init(struct dw_mci *host)
{
	struct dw_mci_rtkemmc_host *priv = host->priv;

	host->pdata->caps2 = MMC_CAP2_NO_SDIO | MMC_CAP2_NO_SD;

	if(priv->emmc_mode >= 2)
		host->pdata->caps2 |= MMC_CAP2_HS200_1_8V_SDR;
	if(priv->emmc_mode >= 3) {
		host->pdata->caps |= MMC_CAP_1_8V_DDR;
		host->pdata->caps2 |= MMC_CAP2_HS400_1_8V;
	}
	if(priv->is_cqe>0)
		host->pdata->caps2 |= (MMC_CAP2_CQE|MMC_CAP2_CQE_DCMD);

	priv->protect_start = 0;
	priv->protect_unit = 0;

	/*In Realtek Platform, only using 32bit DMA*/
	host->dma_64bit_address = 0;

	/*In Realtek Platform, do not use PIO mode by default*/
	host->use_dma = TRANS_MODE_DMA;

	host->irq_flags = IRQF_SHARED;

	mci_writel(host, CP, 0x0);

	/*Enable L4 gated*/
	mci_writel(host, OTHER1, mci_readl(host, OTHER1) & ~(BIT(0)|BIT(2)));

	mci_writel(host, OTHER1, mci_readl(host, OTHER1) & (~(BIT(12)|BIT(13))));

	mci_writel(host, DUMMY_SYS,
		mci_readl(host, DUMMY_SYS) | (SDMMC_CLK_O_ICG_EN | SDMMC_CARD_STOP_ENABLE));

	/*Set the eMMC wrapper little Endian*/
	mci_writel(host, AHB, mci_readl(host, AHB)|BIT(2));

	mci_writel(host, OTHER1,
		mci_readl(host, OTHER1) | SDMMC_STARK_CARD_STOP_ENABLE);

	/*set eMMC instead of nand*/
	regmap_update_bits_base(priv->m2tmx, SDMMC_NAND_DMA_SEL,
				SDMMC_SRAM_DMA_SEL, SDMMC_SRAM_DMA_SEL, NULL, false, true);

	/*Set the clk initial phase*/
	dw_mci_rtk_phase_tuning(host, 0, 0);

        return 0;
}

#ifdef CONFIG_PM
static int dw_mci_rtk_suspend(struct device *dev)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	int ret = 0;

	ret = dw_mci_cqe_runtime_suspend(dev);

	mci_writel(host, AHB, 0);
	printk(KERN_ERR "[%s] AHB=0x%x, dw_mci_cqe_suspend ret=%d\n",
		__func__, mci_readl(host, AHB), ret);

	return ret;
}

static int dw_mci_rtk_resume(struct device *dev)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	int ret = 0;

	mci_writel(host, AHB, mci_readl(host, AHB)|BIT(2));

	ret = dw_mci_cqe_runtime_resume(dev);

	printk(KERN_ERR "[%s] AHB=0x%x, dw_mci_cqe_resume ret=%d\n",
		__func__, mci_readl(host, AHB), ret);

	return ret;
}
#else
static int dw_mci_rtk_suspend(struct device *dev)
{
	pr_err("User should enable CONFIG_PM kernel config\n");

	return 0;
}
static int dw_mci_rtk_resume(struct device *dev)
{
	pr_err("User should enable CONFIG_PM kernel config\n");

	return 0;
}
#endif /*CONFIG_PM*/
static const struct dev_pm_ops rtk_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dw_mci_rtk_suspend,
				dw_mci_rtk_resume)
	SET_RUNTIME_PM_OPS(dw_mci_cqe_runtime_suspend,
			   dw_mci_cqe_runtime_resume,
			   NULL)
};

static void dw_mci_rtk_shutdown(struct platform_device *pdev)
{
	printk(KERN_ERR "[eMMC] Shutdown\n");
	dw_mci_cqe_runtime_resume(&pdev->dev);
}

static unsigned long dw_mci_rtk_dwmmc_caps[1] = {
	MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
	MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED |
	MMC_CAP_NONREMOVABLE | MMC_CAP_CMD23,
};


static const struct dw_mci_drv_data rtk_drv_data = {
	.caps                   = dw_mci_rtk_dwmmc_caps,
	.num_caps               = ARRAY_SIZE(dw_mci_rtk_dwmmc_caps),
	.set_ios                = dw_mci_rtk_set_ios,
	.execute_tuning         = dw_mci_rtk_execute_tuning,
	.parse_dt               = dw_mci_rtk_parse_dt,
	.init                   = dw_mci_rtk_init,
	.switch_voltage         = dw_mci_rtk_switch_voltage,
	.prepare_hs400_tuning	= dw_mci_rtk_prepare_hs400_tuning,
	.hs400_complete         = dw_mci_rtk_hs400_complete,
	.init_card		= dw_mci_rtk_init_card,
};

static const struct of_device_id dw_mci_rtk_match[] = {
	{ .compatible = "rtd161xb-dw-cqe-emmc",
		.data = &rtk_drv_data },
	{ .compatible = "rtd1312c-dw-cqe-emmc",
		.data = &rtk_drv_data },
	{ .compatible = "rtd13xxd-dw-cqe-emmc",
		.data = &rtk_drv_data },
	{},
};
MODULE_DEVICE_TABLE(of, dw_mci_rtk_match);

static int dw_mci_rtk_probe(struct platform_device *pdev)
{

	const struct dw_mci_drv_data *drv_data;
	const struct of_device_id *match;
	int ret = -1;

	if (!pdev->dev.of_node)
		return -ENODEV;

	match = of_match_node(dw_mci_rtk_match, pdev->dev.of_node);
	drv_data = match->data;

	ret = device_create_file(&pdev->dev, &dev_attr_protect_region_setting);
	if (ret < 0) {
		dev_err(&pdev->dev, "device_create_setting_file fail (ret=%d)\n",
			    ret);
	}
	ret = device_create_file(&pdev->dev, &dev_attr_protect_region_start);
	if (ret < 0) {
		dev_err(&pdev->dev, "device_create_start_file fail (ret=%d)\n",
			ret);
	}
	ret = device_create_file(&pdev->dev, &dev_attr_protect_region_unit);
	if (ret < 0) {
		dev_err(&pdev->dev, "device_create_len_file fail (ret=%d)\n",
			ret);
	}
	return dw_mci_pltfm_register(pdev, drv_data);
}

int dw_mci_rtk_remove(struct platform_device *pdev)
{
	struct dw_mci *host = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_protect_region_setting);
	device_remove_file(&pdev->dev, &dev_attr_protect_region_start);
	device_remove_file(&pdev->dev, &dev_attr_protect_region_unit);

	dw_mci_remove(host);
	return 0;
}

static struct platform_driver dw_mci_rtk_pltfm_driver = {
	.probe          = dw_mci_rtk_probe,
	.remove         = dw_mci_rtk_remove,
	.driver         = {
		.name           = "dwmmc_cqe_rtk",
		.of_match_table = dw_mci_rtk_match,
		.pm             = &rtk_dev_pm_ops,
	},
	.shutdown   = dw_mci_rtk_shutdown,
};

module_platform_driver(dw_mci_rtk_pltfm_driver);

MODULE_AUTHOR("<@.com>");
MODULE_DESCRIPTION(" Specific Driver Extension");
MODULE_ALIAS("platform:dwmmc_cqe_rtk");
MODULE_LICENSE("GPL v2");
