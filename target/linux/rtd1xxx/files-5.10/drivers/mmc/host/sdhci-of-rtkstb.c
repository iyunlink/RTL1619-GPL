// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Realtek DHC SoCs Secure Digital Host Controller Interface
 *
 * Copyright (c) 2022 Realtek Semiconductor Corp.
 */

#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/bitfield.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <soc/realtek/rtk_chip.h>

#include "sdhci-pltfm.h"
#include "sdhci-of-rtkstb.h"

struct sdhci_rtkstb_soc_data {
	const struct sdhci_pltfm_data *pdata;
	enum rtd_chip_id chip_id;
	u32 rtkquirks;
};

struct timing_phase_path {
	int start;
	int end;
	int mid;
	int len;
};

struct sdhci_rtkstb {
	const struct sdhci_rtkstb_soc_data *soc_data;
	struct gpio_desc *sd_pwr_gpio;
	struct clk *sd_clk_en;
	struct clk *sd_ip_clk_en;
	struct reset_control *sd_rstn;
	void __iomem *crt_base;
};

static int rtkstb_sdhci_pad_power_check(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_rtkstb *rtk_host = sdhci_pltfm_priv(pltfm_host);
	struct device *dev = mmc_dev(host->mmc);
	void __iomem *crt_base =  rtk_host->crt_base;
	int pad_pwr_chk = 0, pad_pwr_3v3 = 0;
	u32 reg;
	u16 ctrl;

	do {
		reg = readl(crt_base + ISO_DBG_STATUS);
		ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
		if (reg & SDIO0_H3L1_STATUS_HI) {
			ctrl &= ~SDHCI_CTRL_VDD_180;
			sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
			pad_pwr_3v3 = 1;
		} else {
			dev_dbg(dev, "pad power switch to 1.8v, retry %d times", pad_pwr_chk);
			ctrl |= SDHCI_CTRL_VDD_180;
			sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
			pad_pwr_3v3 = 0;
		}

		pad_pwr_chk += pad_pwr_chk;
	} while (pad_pwr_3v3 && pad_pwr_chk < PAD_PWR_CHK_CNT);

	return pad_pwr_3v3;
}

static int rtkstb_sdhci_pad_power_ctrl(struct sdhci_host *host, int voltage)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_rtkstb *rtk_host = sdhci_pltfm_priv(pltfm_host);
	struct device *dev = mmc_dev(host->mmc);
	void __iomem *crt_base =  rtk_host->crt_base;
	int err = 0;
	u32 reg;

	switch (voltage) {
	case MMC_SIGNAL_VOLTAGE_180:
		reg = readl(crt_base + SYS_PLL_SD1);
		reg &= ~REG_SEL3318_MASK;
		writel(reg, crt_base + SYS_PLL_SD1);
		mdelay(1);

		reg = readl(crt_base + SYS_PLL_SD1);
		reg |= FIELD_PREP(REG_SEL3318_MASK, REG_SEL3318_0V);
		writel(reg, crt_base + SYS_PLL_SD1);
		mdelay(1);

		reg = readl(crt_base + SYS_PLL_SD1);
		reg |= FIELD_PREP(REG_SEL3318_MASK, REG_SEL3318_1V8);
		writel(reg, crt_base + SYS_PLL_SD1);
		mdelay(1);

		err = rtkstb_sdhci_pad_power_check(host);
		if (err) {
			dev_err(dev, "switch LDO to 1.8v fail\n");
			goto exit;
		}

		reg = readl(crt_base + ISO_PFUNC18);
		reg &= ~(HIF_EN_H3L1 | HIF_DATA_H3L1);
		writel(reg, crt_base + ISO_PFUNC18);

		reg = readl(crt_base + ISO_PFUNC19);
		reg &= ~(GPIO33_H3L1 | GPIO32_H3L1);
		writel(reg, crt_base + ISO_PFUNC19);

		reg = readl(crt_base + ISO_PFUNC20);
		reg &= ~(HIF_CLK_H3L1 | HIF_RDY_H3L1);
		writel(reg, crt_base + ISO_PFUNC20);
		break;
	case MMC_SIGNAL_VOLTAGE_330:
		reg = readl(crt_base + SYS_PLL_SD1);
		if (FIELD_GET(REG_SEL3318_MASK, reg) == REG_SEL3318_3V3)
			goto exit;

		reg &= ~FIELD_PREP(REG_SEL3318_MASK, REG_SEL3318_3V3);
		writel(reg, crt_base + SYS_PLL_SD1);
		mdelay(1);

		reg = readl(crt_base + ISO_PFUNC18);
		reg |= HIF_EN_H3L1 | HIF_DATA_H3L1;
		writel(reg, crt_base + ISO_PFUNC18);

		reg = readl(crt_base + ISO_PFUNC19);
		reg |= GPIO33_H3L1 | GPIO32_H3L1;
		writel(reg, crt_base + ISO_PFUNC19);

		reg = readl(crt_base + ISO_PFUNC20);
		reg |= HIF_CLK_H3L1 | HIF_RDY_H3L1;
		writel(reg, crt_base + ISO_PFUNC20);
		mdelay(1);

		reg = readl(crt_base + SYS_PLL_SD1);
		reg &= ~REG_SEL3318_MASK;
		writel(reg, crt_base + SYS_PLL_SD1);
		mdelay(1);

		reg = readl(crt_base + SYS_PLL_SD1);
		reg |= FIELD_PREP(REG_SEL3318_MASK, REG_SEL3318_3V3);
		writel(reg, crt_base + SYS_PLL_SD1);
		break;
	}
exit:
	return err;
}

static void rtkstb_sdhci_pad_driving_configure(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_rtkstb *rtk_host = sdhci_pltfm_priv(pltfm_host);
	void __iomem *crt_base =  rtk_host->crt_base;
	u32 reg;

	reg = readl(crt_base + ISO_PFUNC18);
	reg &= ~(HIF_EN_PAD_N_MASK | HIF_EN_PAD_P_MASK |
		 HIF_DATA_PAD_N_MASK | HIF_DATA_PAD_P_MASK);

	if (host->timing > MMC_TIMING_SD_HS)
		reg |= HIF_EN_PAD_N3E | HIF_EN_PAD_N2E | HIF_EN_PAD_P3E | HIF_EN_PAD_P2E |
		       HIF_DATA_PAD_N3E | HIF_DATA_PAD_N2E | HIF_DATA_PAD_P3E | HIF_DATA_PAD_P2E;

	writel(reg, crt_base + ISO_PFUNC18);

	reg = readl(crt_base + ISO_PFUNC19);
	reg &= ~(GPIO33_PAD_N_MASK | GPIO33_PAD_P_MASK |
		 GPIO32_PAD_N_MASK | GPIO32_PAD_P_MASK);

	if (host->timing > MMC_TIMING_SD_HS)
		reg |= GPIO33_PAD_N3E | GPIO33_PAD_N2E | GPIO33_PAD_P3E | GPIO33_PAD_P2E |
		       GPIO32_PAD_N3E | GPIO32_PAD_N2E | GPIO32_PAD_P3E | GPIO32_PAD_P2E;

	writel(reg, crt_base + ISO_PFUNC19);

	reg = readl(crt_base + ISO_PFUNC20);
	reg &= ~(HIF_CLK_PAD_N_MASK | HIF_CLK_PAD_P_MASK |
		 HIF_RDY_PAD_N_MASK | HIF_RDY_PAD_P_MASK);

	if (host->timing > MMC_TIMING_SD_HS)
		reg |= HIF_CLK_PAD_N3E | HIF_CLK_PAD_N2E | HIF_CLK_PAD_P3E | HIF_CLK_PAD_P2E |
		       HIF_RDY_PAD_N3E | HIF_RDY_PAD_N2E | HIF_RDY_PAD_P3E | HIF_RDY_PAD_P2E;

	writel(reg, crt_base + ISO_PFUNC20);
}

static void rtkstb_sdhci_pll_configure(struct sdhci_host *host, u32 ssc, int down_clk_tuning)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_rtkstb *rtk_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_rtkstb_soc_data *soc_data = rtk_host->soc_data;
	void __iomem *crt_base =  rtk_host->crt_base;
	u32 rtkquirks = soc_data->rtkquirks;
	u32 reg, sscpll;

	reg = readl(crt_base + SYS_PLL_SD4);
	reg &= ~SSC_RSTB;
	reg |= SSC_PLL_RSTB | SSC_PLL_POW;
	writel(reg, crt_base + SYS_PLL_SD4);

	sscpll = FIELD_PREP(REG_TUNE11, REG_TUNE11_1V9) |
		 FIELD_PREP(SSCPLL_CS1, SSCPLL_CS1_INIT_VALUE) |
		 FIELD_PREP(SSCPLL_ICP, SSCPLL_ICP_CURRENT) |
		 FIELD_PREP(SSCPLL_RS, SSCPLL_RS_13K) |
		 FIELD_PREP(SSC_DEPTH, SSC_DEPTH_1_N) |
		 SSC_8X_EN |
		 FIELD_PREP(SSC_DIV_EXT_F, SSC_DIV_EXT_F_INIT_VAL) |
		 EN_CPNEW;

	if (down_clk_tuning || (rtkquirks & RTKQUIRK_SSC_CLK_JITTER))
		sscpll &= ~SSC_DEPTH;

	writel(sscpll, crt_base + SYS_PLL_SD2);
	writel(ssc, crt_base + SYS_PLL_SD3);
	mdelay(2);

	reg |= SSC_RSTB;
	writel(reg, crt_base + SYS_PLL_SD4);
	udelay(200);
}

static void sdhci_rtkstb_hw_init(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_rtkstb *rtk_host = sdhci_pltfm_priv(pltfm_host);
	void __iomem *crt_base =  rtk_host->crt_base;
	u32 reg;

	reg = readl(crt_base + ISO_CTRL_1);
	reg |= SD3_SSCLDO_EN | SD3_BIAS_EN;
	writel(reg, crt_base + ISO_CTRL_1);
	udelay(200);

	reg = FIELD_PREP(SSC_TBASE, SSC_TBASE_INIT_VALUE) |
	      FIELD_PREP(SSC_STEP_IN, SSC_STEP_IN_INIT_VALUE) |
	      FIELD_PREP(SSC_DIV_N, SSC_DIV_N_200M);

	rtkstb_sdhci_pll_configure(host, reg, 0);

	reg = readl(crt_base + SDHCI_RTK_ISREN);
	reg &= ~(INT1_EN | INT3_EN);
	reg |= ISREN_WRITE_DATA | INT4_EN;
	writel(reg, crt_base + SDHCI_RTK_ISREN);

	reg = readl(crt_base + SDHCI_RTK_CTL);
	reg &= ~(L4_GATE_DISABLE | DBUS_ENDIAN_SEL |
		 L4_GATE_DISABLE_IP | IP_ENDIAN_SEL);
	reg |= SUSPEND_N;
	writel(reg, crt_base + SDHCI_RTK_CTL);

	reg = readl(crt_base + ISO_GPDIR_1);
	reg |= GPDIR_1_53;
	writel(reg, crt_base + ISO_GPDIR_1);

	reg = readl(crt_base + ISO_GPDATO_1);
	reg &= ~GPDATO_1_53;
	writel(reg, crt_base + ISO_GPDATO_1);
	mdelay(10);

	writel(0x0, crt_base + ISO_MUXPAD6);
	reg = FIELD_PREP(GPIO_32_PADMUX, MUX_TO_SD_CMD) |
	      FIELD_PREP(GPIO_33_PADMUX, MUX_TO_SD_CLK) |
	      FIELD_PREP(HIF_DATA_PADMUX, MUX_TO_SD_DAT0) |
	      FIELD_PREP(HIF_EN_PADMUX, MUX_TO_SD_DAT1) |
	      FIELD_PREP(HIF_RDY_PADMUX, MUX_TO_SD_DAT2) |
	      FIELD_PREP(HIF_CLK_PADMUX, MUX_TO_SD_DAT3);
	writel(reg, crt_base + ISO_MUXPAD6);
	/*
	 * To disable the pull function when the pad switch to
	 * SD clock, ensure the clock starts at low amplitude.
	 */
	reg = readl(crt_base + ISO_PFUNC19);
	reg &= ~(GPIO33_PUD_EN | GPIO33_PUD_SEL);
	writel(reg, crt_base + ISO_PFUNC19);

	reg = readl(crt_base + ISO_DBG_STATUS);
	reg |= SDIO0_H3L1_DETECT_EN;
	writel(reg, crt_base + ISO_DBG_STATUS);
}

static void rtkstb_sdhci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_rtkstb *rtk_host = sdhci_pltfm_priv(pltfm_host);
	void __iomem *crt_base =  rtk_host->crt_base;
	u32 ssc;

	sdhci_set_ios(mmc, ios);
	rtkstb_sdhci_pad_driving_configure(host);

	ssc = readl(crt_base + SYS_PLL_SD3);
	ssc &= ~SSC_DIV_N;

	switch (host->timing) {
	case MMC_TIMING_SD_HS:
		dev_err(mmc_dev(host->mmc), "high speed mode\n");
		ssc |= FIELD_PREP(SSC_DIV_N, SSC_DIV_N_200M);
		rtkstb_sdhci_pll_configure(host, ssc, 0);
		break;
	case MMC_TIMING_UHS_SDR12:
	case MMC_TIMING_UHS_SDR25:
		dev_err(mmc_dev(host->mmc), "SDR25 mode\n");
		ssc |= FIELD_PREP(SSC_DIV_N, SSC_DIV_N_200M);
		rtkstb_sdhci_pll_configure(host, ssc, 0);
		break;
	case MMC_TIMING_UHS_SDR50:
		dev_err(mmc_dev(host->mmc), "SDR50 mode\n");
		ssc |= FIELD_PREP(SSC_DIV_N, SSC_DIV_N_200M);
		rtkstb_sdhci_pll_configure(host, ssc, 0);
		break;
	case MMC_TIMING_UHS_SDR104:
		dev_err(mmc_dev(host->mmc), "SDR104 mode\n");
		ssc |= FIELD_PREP(SSC_DIV_N, SSC_DIV_N_208M);
		rtkstb_sdhci_pll_configure(host, ssc, 0);
		break;
	}
}

static int rtkstb_mmc_send_tuning(struct mmc_host *mmc, u32 opcode, int tx)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct mmc_command cmd = {0};
	u32 reg, mask;
	int err;

	if (tx) {
		mask = ~(SDHCI_INT_CRC | SDHCI_INT_END_BIT | SDHCI_INT_INDEX |
			 SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_DATA_CRC |
			 SDHCI_INT_DATA_END_BIT | SDHCI_INT_BUS_POWER |
			 SDHCI_INT_AUTO_CMD_ERR | SDHCI_INT_ADMA_ERROR);

		reg = sdhci_readl(host, SDHCI_INT_ENABLE);
		reg &= mask;
		sdhci_writel(host, reg, SDHCI_INT_ENABLE);

		reg = sdhci_readl(host, SDHCI_SIGNAL_ENABLE);
		reg &= mask;
		sdhci_writel(host, reg, SDHCI_SIGNAL_ENABLE);

		cmd.opcode = opcode;
		if (mmc_card_sdio(mmc->card)) {
			cmd.arg = 0x2000;
			cmd.flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_AC;
		} else {
			cmd.arg = mmc->card->rca << 16;
			cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
		}

		err = mmc_wait_for_cmd(mmc, &cmd, 0);
		if (err)
			return err;
	} else {
		return mmc_send_tuning(mmc, opcode, NULL);
	}

	return 0;
}

static int rtkstb_sdhci_change_phase(struct sdhci_host *host, u8 sample_point, int tx)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_rtkstb *rtk_host = sdhci_pltfm_priv(pltfm_host);
	void __iomem *crt_base =  rtk_host->crt_base;
	u32 reg = 0;
	u16 clk = 0;

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk &= ~SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
	/* To disable reset signal */
	reg = readl(crt_base + SYS_PLL_SD1);
	reg &= ~PHRT0;
	writel(reg, crt_base + SYS_PLL_SD1);

	reg = readl(crt_base + SYS_PLL_SD1);

	if (tx) {
		reg &= ~PHSEL0_MASK;
		reg |= (sample_point << PHSEL0_SHIFT);
	} else {
		reg &= ~PHSEL1_MASK;
		reg |= (sample_point << PHSEL1_SHIFT);
	}

	writel(reg, crt_base + SYS_PLL_SD1);
	/* re-enable reset signal */
	reg = readl(crt_base + SYS_PLL_SD1);
	reg |= PHRT0;
	writel(reg, crt_base + SYS_PLL_SD1);
	udelay(100);

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	return 0;
}

static inline u32 test_phase_bit(u32 phase_map, unsigned int bit)
{
	bit %= MAX_PHASE;
	return phase_map & (1 << bit);
}

static int sd_get_phase_len(u32 phase_map, unsigned int start_bit)
{
	int i;

	for (i = 0; i < MAX_PHASE; i++) {
		if (test_phase_bit(phase_map, start_bit + i) == 0)
			return i;
	}
	return MAX_PHASE;
}

static u8 rtkstb_sdhci_search_final_phase(struct sdhci_host *host, u32 phase_map, int tx)
{
	int start = 0, len = 0;
	int start_final = 0, len_final = 0;
	u8 final_phase = 0xFF;

	if (phase_map == 0) {
		dev_err(mmc_dev(host->mmc), "phase error: [map:%08x]\n", phase_map);
		return final_phase;
	}

	while (start < MAX_PHASE) {
		len = sd_get_phase_len(phase_map, start);
		if (len_final < len) {
			start_final = start;
			len_final = len;
		}
		start += len ? len : 1;
	}

	if (len_final > MINIMUM_CONTINUE_LENGTH)
		final_phase = (start_final + len_final / 2) % MAX_PHASE;
	else
		final_phase = 0xFF;

	dev_err(mmc_dev(host->mmc), "%s phase: [map:%08x] [maxlen:%d] [final:%d]\n",
		tx ? "tx" : "rx", phase_map, len_final, final_phase);

	return final_phase;
}

static int rtkstb_sdhci_tuning(struct sdhci_host *host, u32 opcode, int tx)
{
	struct mmc_host *mmc = host->mmc;
	int err, i, sample_point;
	u32 raw_phase_map[TUNING_CNT] = {0}, phase_map;
	u8 final_phase = 0;

	for (sample_point = 0; sample_point < MAX_PHASE; sample_point++) {
		for (i = 0; i < TUNING_CNT; i++) {
			rtkstb_sdhci_change_phase(host, (u8) sample_point, tx);
			err = rtkstb_mmc_send_tuning(mmc, opcode, tx);
			if (err == 0)
				raw_phase_map[i] |= (1 << sample_point);
		}
	}

	phase_map = 0xFFFFFFFF;
	for (i = 0; i < TUNING_CNT; i++) {
		dev_dbg(mmc_dev(host->mmc), "%s raw_phase_map[%d] = 0x%08x\n",
			tx ? "tx" : "rx", i, raw_phase_map[i]);
		phase_map &= raw_phase_map[i];
	}

	if (phase_map) {
		final_phase = rtkstb_sdhci_search_final_phase(host, phase_map, tx);
		if (final_phase == 0xFF) {
			dev_err(mmc_dev(host->mmc), "final phase invalid\n");
			return -EINVAL;
		}

		err = rtkstb_sdhci_change_phase(host, final_phase, tx);
		if (err < 0)
			return err;
	} else {
		dev_err(mmc_dev(host->mmc), "tuning fail, phase map unavailable\n");
		return -EINVAL;
	}

	return 0;
}

static void rtkstb_sdhci_down_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_rtkstb *rtk_host = sdhci_pltfm_priv(pltfm_host);
	void __iomem *crt_base =  rtk_host->crt_base;
	u32 ssc;

	ssc = readl(crt_base + SYS_PLL_SD3);

	switch (FIELD_GET(SSC_DIV_N, ssc)) {
	case SSC_DIV_N_208M:
		ssc &= ~SSC_DIV_N;
		ssc |= FIELD_PREP(SSC_DIV_N, SSC_DIV_N_200M);
		break;
	case SSC_DIV_N_200M:
		ssc &= ~SSC_DIV_N;
		ssc |= FIELD_PREP(SSC_DIV_N, SSC_DIV_N_100M);
		break;
	case SSC_DIV_N_100M:
		ssc &= ~SSC_DIV_N;
		ssc |= FIELD_PREP(SSC_DIV_N, SSC_DIV_N_50M);
		break;
	}

	rtkstb_sdhci_pll_configure(host, ssc, 1);
}

static int rtkstb_sdhci_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_rtkstb *rtk_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_rtkstb_soc_data *soc_data = rtk_host->soc_data;
	void __iomem *crt_base =  rtk_host->crt_base;
	u32 rtkquirks = soc_data->rtkquirks;
	u32 reg, tx_opcode;
	int ret = 0;

	dev_info(mmc_dev(host->mmc), "execute clock phase tuning\n");
	/* To disable the SSC during the phase tuning process. */
	reg = readl(crt_base + SYS_PLL_SD2);
	reg &= ~SSC_DEPTH;
	writel(reg, crt_base + SYS_PLL_SD2);

	if (mmc_card_sdio(host->mmc->card))
		tx_opcode = SD_IO_RW_DIRECT;
	else
		tx_opcode = MMC_SEND_STATUS;

	ret = rtkstb_sdhci_tuning(host, tx_opcode, 1);
	if (ret)
		dev_err(mmc_dev(host->mmc), "tx tuning fail\n");

	do {
		ret = rtkstb_sdhci_tuning(host, MMC_SEND_TUNING_BLOCK, 0);
		if (ret) {
			reg = readl(crt_base + SYS_PLL_SD3);
			if (FIELD_GET(SSC_DIV_N, reg) == SSC_DIV_N_50M) {
				dev_err(mmc_dev(host->mmc), "rx tuning fail\n");
				return ret;
			}

			dev_err(mmc_dev(host->mmc), "down clock, and retuning\n");
			rtkstb_sdhci_down_clock(host);
		}
	} while (ret);

	if (!(rtkquirks & RTKQUIRK_SSC_CLK_JITTER)) {
		reg = readl(crt_base + SYS_PLL_SD2);
		reg |= FIELD_PREP(SSC_DEPTH, SSC_DEPTH_1_N);
		writel(reg, crt_base + SYS_PLL_SD2);
	}

	reg = readl(crt_base + SYS_PLL_SD3);

	dev_info(mmc_dev(host->mmc), "after tuning, current pll = %x\n",
		 FIELD_GET(SSC_DIV_N, reg));

	return 0;
}

static void rtkstb_sdhci_card_event(struct sdhci_host *host)
{
	int err = 0;

	dev_err(mmc_dev(host->mmc), "card event detected\n");

	err = rtkstb_sdhci_pad_power_ctrl(host, MMC_SIGNAL_VOLTAGE_330);
	if (err)
		dev_err(mmc_dev(host->mmc), "reset voltage to 3.3v fail\n");

	if (mmc_gpio_get_cd(host->mmc))
		dev_err(mmc_dev(host->mmc), "card insert\n");
	else
		dev_err(mmc_dev(host->mmc), "card remove\n");
}

static void rtkstb_sdhci_voltage_switch(struct sdhci_host *host)
{
	int err = 0;

	err = rtkstb_sdhci_pad_power_ctrl(host, MMC_SIGNAL_VOLTAGE_180);
	if (!err)
		dev_err(mmc_dev(host->mmc), "voltage switch to 1.8v\n");
}

/* Update card information to determine SD/SDIO tx tuning function */
static void rtkstb_init_card(struct mmc_host *mmc, struct mmc_card *card)
{
	mmc->card = card;
}

static void rtkstb_replace_mmc_host_ops(struct sdhci_host *host)
{
	host->mmc_host_ops.set_ios	= rtkstb_sdhci_set_ios;
	host->mmc_host_ops.init_card	= rtkstb_init_card;
}

static const struct sdhci_ops rtkstb_sdhci_ops = {
	.set_clock	= sdhci_set_clock,
	.set_bus_width	= sdhci_set_bus_width,
	.reset		= sdhci_reset,
	.platform_execute_tuning = rtkstb_sdhci_execute_tuning,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.card_event	= rtkstb_sdhci_card_event,
	.voltage_switch = rtkstb_sdhci_voltage_switch,
};

static const struct sdhci_pltfm_data sdhci_rtkstb_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12 |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC,
	.quirks2 = SDHCI_QUIRK2_BROKEN_DDR50 |
		   SDHCI_QUIRK2_ACMD23_BROKEN |
		   SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
	.ops = &rtkstb_sdhci_ops,
};

static struct sdhci_rtkstb_soc_data rtd1319d_soc_data = {
	.pdata = &sdhci_rtkstb_pdata,
	.chip_id = CHIP_ID_RTD1319D,
	.rtkquirks = RTKQUIRK_SSC_CLK_JITTER,
};

static const struct of_device_id sdhci_rtkstb_dt_match[] = {
	{.compatible = "realtek,rtd1319d-sdmmc", .data = &rtd1319d_soc_data},
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_rtkstb_dt_match);

static int sdhci_rtkstb_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	const struct sdhci_rtkstb_soc_data *soc_data;
	struct device_node *node = pdev->dev.of_node;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_rtkstb *rtk_host;
	int ret;

	dev_info(&pdev->dev, " build at : %s\n", utsname()->version);

	match = of_match_device(sdhci_rtkstb_dt_match, &pdev->dev);
	if (!match)
		return -EINVAL;

	soc_data = match->data;

	host = sdhci_pltfm_init(pdev, soc_data->pdata, sizeof(*rtk_host));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	rtk_host = sdhci_pltfm_priv(pltfm_host);
	rtk_host->soc_data = soc_data;

	rtkstb_replace_mmc_host_ops(host);

	rtk_host->crt_base = of_iomap(node, 1);
	if (!rtk_host->crt_base) {
		dev_err(&pdev->dev, "%s failed to map CRT space\n", __func__);
		ret = -ENOMEM;
		goto err_free_pltfm;
	}

	rtk_host->sd_pwr_gpio = devm_gpiod_get(&pdev->dev, "sd-power", GPIOD_OUT_LOW);
	if (IS_ERR(rtk_host->sd_pwr_gpio)) {
		dev_err(&pdev->dev, "%s can't request power gpio\n", __func__);
		ret = PTR_ERR(rtk_host->sd_pwr_gpio);
		goto err_iomap;
	}

	rtk_host->sd_rstn = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(rtk_host->sd_rstn)) {
		dev_err(&pdev->dev, "%s can't request reset control\n", __func__);
		ret = PTR_ERR(rtk_host->sd_rstn);
		goto err_iomap;
	}

	rtk_host->sd_clk_en = devm_clk_get(&pdev->dev, "sd");
	if (IS_ERR(rtk_host->sd_clk_en)) {
		dev_err(&pdev->dev, "%s can't request clock\n", __func__);
		ret = PTR_ERR(rtk_host->sd_clk_en);
		goto err_iomap;
	}

	rtk_host->sd_ip_clk_en = devm_clk_get(&pdev->dev, "sd_ip");
	if (IS_ERR(rtk_host->sd_ip_clk_en)) {
		dev_err(&pdev->dev, "%s can't request IP clock\n", __func__);
		ret = PTR_ERR(rtk_host->sd_clk_en);
		goto err_iomap;
	}

	ret = reset_control_deassert(rtk_host->sd_rstn);
	if (ret) {
		dev_err(&pdev->dev, " %s can't deassert reset\n", __func__);
		goto err_iomap;
	}

	ret = clk_prepare_enable(rtk_host->sd_clk_en);
	if (ret) {
		dev_err(&pdev->dev, " %s can't enable clock\n", __func__);
		goto err_deassert_rst;
	}

	ret = clk_prepare_enable(rtk_host->sd_ip_clk_en);
	if (ret) {
		dev_err(&pdev->dev, " %s can't enable IP clock\n", __func__);
		goto err_enable_clk;
	}

	sdhci_rtkstb_hw_init(host);

	ret = mmc_of_parse(host->mmc);
	if (ret) {
		dev_err(&pdev->dev, "%s parsing dt failed\n", __func__);
		goto err_enable_ip_clk;
	}

	ret = sdhci_add_host(host);
	if (ret)
		goto err_enable_ip_clk;

	return 0;

err_enable_ip_clk:
	clk_disable_unprepare(rtk_host->sd_ip_clk_en);
err_enable_clk:
	clk_disable_unprepare(rtk_host->sd_clk_en);
err_deassert_rst:
	reset_control_assert(rtk_host->sd_rstn);
err_iomap:
	iounmap(rtk_host->crt_base);
err_free_pltfm:
	sdhci_pltfm_free(pdev);

	return ret;
}

static int sdhci_rtkstb_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	int dead = (readl(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);

	dev_warn(&pdev->dev, "%s\n", __func__);

	sdhci_remove_host(host, dead);
	sdhci_pltfm_free(pdev);

	return 0;
}

static void sdhci_rtkstb_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sdhci_host *host = dev_get_drvdata(dev);

	dev_warn(dev, "%s\n", __func__);

	sdhci_suspend_host(host);
}

static int __maybe_unused sdhci_rtkstb_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_rtkstb *rtk_host = sdhci_pltfm_priv(pltfm_host);

	dev_warn(dev, "%s start\n", __func__);

	sdhci_suspend_host(host);

	reset_control_assert(rtk_host->sd_rstn);

	if (!IS_ERR(rtk_host->sd_clk_en))
		clk_disable_unprepare(rtk_host->sd_clk_en);

	if (!IS_ERR(rtk_host->sd_ip_clk_en))
		clk_disable_unprepare(rtk_host->sd_ip_clk_en);

	dev_warn(dev, "%s finish\n", __func__);

	return 0;
}

static int __maybe_unused sdhci_rtkstb_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_rtkstb *rtk_host = sdhci_pltfm_priv(pltfm_host);
	u8 reg;
	int ret;

	host->clock = 0;

	dev_warn(dev, "%s start\n", __func__);

	reset_control_deassert(rtk_host->sd_rstn);

	ret = clk_prepare_enable(rtk_host->sd_clk_en);
	if (ret) {
		dev_err(dev, " %s can't enable clock\n", __func__);
		goto err_enable_clk;
	}

	ret = clk_prepare_enable(rtk_host->sd_ip_clk_en);
	if (ret) {
		dev_err(dev, " %s can't enable IP clock\n", __func__);
		goto err_enable_ip_clk;
	}

	sdhci_rtkstb_hw_init(host);

	reg = sdhci_readb(host, SDHCI_POWER_CONTROL);
	reg |= SDHCI_POWER_ON | SDHCI_POWER_330;
	sdhci_writeb(host, reg, SDHCI_POWER_CONTROL);

	sdhci_resume_host(host);

	dev_warn(dev, "%s finish\n", __func__);

	return 0;

err_enable_ip_clk:
	clk_disable_unprepare(rtk_host->sd_ip_clk_en);
err_enable_clk:
	clk_disable_unprepare(rtk_host->sd_clk_en);

	return ret;
}

static SIMPLE_DEV_PM_OPS(sdhci_rtkstb_dev_pm_ops, sdhci_rtkstb_suspend,
			 sdhci_rtkstb_resume);

static struct platform_driver sdhci_rtkstb_driver = {
	.driver		= {
		.name	= "sdhci-rtkstb",
		.of_match_table = sdhci_rtkstb_dt_match,
		.pm	= &sdhci_rtkstb_dev_pm_ops,
	},
	.probe		= sdhci_rtkstb_probe,
	.remove		= sdhci_rtkstb_remove,
	.shutdown	= sdhci_rtkstb_shutdown,
};

module_platform_driver(sdhci_rtkstb_driver);

MODULE_DESCRIPTION("SDHCI platform driver for Realtek DHC STB SoC");
MODULE_LICENSE("GPL");
