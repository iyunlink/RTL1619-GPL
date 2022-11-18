// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Realtek SPI driver based on DW SPI Core on RTD139x,
 * RTD16xx, or RTD13xx platform
 *
 * Copyright (c) 2020 Realtek Corporation.
 */

#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>

#include "spi-dw.h"

#define DRIVER_NAME "spi-dw-rtk"
#define SPI_Wra_CTRL 0

struct rtk_spi {
	struct dw_spi dws;

	void __iomem *spi_wrapper;
	struct clk *clk;
	struct reset_control *rstc;
	struct device *dev;
	u32 clk_div;
	#if IS_ENABLED(CONFIG_SPI_SPIDEV)
	struct spi_board_info info;
	struct spi_device *spidev;
	#endif /* CONFIG_SPI_SPIDEV */
};

static inline struct rtk_spi *rtk_spi_to_hw(struct spi_device *spi)
{
	struct dw_spi *dws = spi_controller_get_devdata(spi->master);
	struct rtk_spi *hw = container_of(dws, struct rtk_spi, dws);

	return hw;
}

static inline void rtk_spi_wrap_ctrl(struct spi_device *spi,  int val)
{
	struct rtk_spi *hw = rtk_spi_to_hw(spi);

	__raw_writel(val, hw->spi_wrapper + SPI_Wra_CTRL);
}

static void rtk_spi_set_cs(struct spi_device *spi, bool enable)
{
	rtk_spi_wrap_ctrl(spi, enable ? 0x17 : 0x13);
	dw_spi_set_cs(spi, enable);
}

static int rtk_spi_probe(struct platform_device *pdev)
{
	struct rtk_spi *hw;
	struct dw_spi *dws;
	struct clk *clk = clk_get(&pdev->dev, NULL);
	struct reset_control *rstc =
		reset_control_get_exclusive(&pdev->dev, NULL);
	u32 val;
	int err = -ENODEV;

	if (WARN_ON(!(pdev->dev.of_node))) {
		pr_err("[SPI] Error: No node\n");
		return err;
	}

	hw = devm_kzalloc(&pdev->dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	dws = &hw->dws;
	/* Enable clk and release reset module */
	reset_control_deassert(rstc);	/* release reset */
	clk_prepare_enable(clk);	/* enable clk */

	dev_set_drvdata(&pdev->dev, hw);
	dws->set_cs = rtk_spi_set_cs;

	/* Find and map resources */
	dws->regs = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(dws->regs)) {
		dev_err(&pdev->dev, "[SPI] DW SPI region map failed, addr 0x%p\n", dws->regs);
		goto exit;
	}
	hw->spi_wrapper = of_iomap(pdev->dev.of_node, 1);

	if (IS_ERR(dws->regs)) {
		dev_err(&pdev->dev, "[SPI] SPI wrapper region map failed, addr 0x%p\n", hw->spi_wrapper);
		goto exit;
	}

	dws->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (dws->irq < 0) {
		dev_err(&pdev->dev, "[SPI] no irq resource?\n");
		err = -ENXIO;
		goto exit;
	}

	if (!of_property_read_u32(pdev->dev.of_node, "num-chipselect", &val))
		dws->num_cs = val;
	if (!of_property_read_u32(pdev->dev.of_node, "bus-num", &val))
		dws->bus_num = val;
	if (!of_property_read_u32(pdev->dev.of_node, "clock-frequency", &val))
		dws->max_freq = val;

	hw->clk = clk;
	hw->rstc = rstc;
	hw->dev = &pdev->dev;

	pm_runtime_enable(&pdev->dev);

	/* call setup function */
	err = dw_spi_add_host(&pdev->dev, &hw->dws);
	if (err) {
		pr_err("[SPI] Init failed, ret = %d\n", err);
		goto dw_err_exit;
	}
	dev_info(&pdev->dev, "[SPI] num_cs %d, bus_num %d, max_freq %d, irq %d\n",
		 hw->dws.num_cs, hw->dws.bus_num, hw->dws.max_freq,
		 hw->dws.irq);

	#if IS_ENABLED(CONFIG_SPI_SPIDEV)
	/* add spidev */
	strcpy(hw->info.modalias, "spidev");
	hw->info.max_speed_hz = 16 * 1000 * 1000;
	hw->info.platform_data = hw;
	hw->info.mode = SPI_MODE_0;

	hw->spidev = spi_new_device(hw->dws.master, &hw->info);
	if (hw->spidev) {
		dev_info(&pdev->dev, "[SPI] add spidev for %s\n",
			dev_name(&hw->spidev->dev));
	} else {
		dev_err(&pdev->dev, "[SPI] spi_new_device failed\n");
	}
	#endif /* CONFIG_SPI_SPIDEV */

	return 0;

dw_err_exit:
	pm_runtime_disable(&pdev->dev);

exit:
	/* Disable clk and reset module */
	reset_control_assert(hw->rstc);	/* reset */
	clk_disable_unprepare(hw->clk);	/* disable clk */
	reset_control_put(rstc);
	dev_set_drvdata(&pdev->dev, NULL);
	return err;
}

static int rtk_spi_remove(struct platform_device *dev)
{
	struct rtk_spi *hw = platform_get_drvdata(dev);
	struct dw_spi *dws = &hw->dws;

	dw_spi_remove_host(dws);

	pm_runtime_disable(&dev->dev);

	/* Disable clk and reset module */
	reset_control_assert(hw->rstc);	/* reset */
	clk_disable_unprepare(hw->clk);	/* disable clk */
	reset_control_put(hw->rstc);

	dev_set_drvdata(&dev->dev, NULL);
	return 0;
}


#ifdef CONFIG_PM_SLEEP

static int rtk_spi_suspend(struct device *dev)
{
	struct rtk_spi *hw = dev_get_drvdata(dev);
	struct dw_spi *dws = &hw->dws;
	int ret;

	dev_info(dev, "[SPI] Enter %s\n", __func__);

	/* save SPI baud rate */
	hw->clk_div = dw_readl(dws, DW_SPI_BAUDR);

	ret = dw_spi_suspend_host(dws);

	dev_info(dev, "[SPI] Exit %s\n", __func__);

	return ret;
}

static int rtk_spi_resume(struct device *dev)
{
	struct rtk_spi *hw = dev_get_drvdata(dev);
	struct dw_spi *dws = &hw->dws;
	int ret;

	dev_info(dev, "[SPI] Enter %s\n", __func__);

	ret = dw_spi_resume_host(dws);

	/* restore SPI baud rate */
	spi_enable_chip(dws, 0);
	spi_set_clk(dws, hw->clk_div);
	spi_enable_chip(dws, 1);

	dev_info(dev, "[SPI] Exit %s\n", __func__);

	return ret;
}

static SIMPLE_DEV_PM_OPS(rtk_spi_pm_ops, rtk_spi_suspend, rtk_spi_resume);

#define RTK_SPI_PM_OPS	(&rtk_spi_pm_ops)

#else /* CONFIG_PM_SLEEP */
#define RTK_SPI_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */


static const struct of_device_id rtk_spi_match[] = {
	{ .compatible = "realtek,rtk-dw-apb-ssi", },
	{},
};
MODULE_DEVICE_TABLE(of, rtk_spi_match);

static struct platform_driver rtk_spi_driver = {
	.probe = rtk_spi_probe,
	.remove = rtk_spi_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = RTK_SPI_PM_OPS,
		.of_match_table = of_match_ptr(rtk_spi_match),
	},
};
module_platform_driver(rtk_spi_driver);

MODULE_AUTHOR("Eric Wang <ericwang@realtek.com>");
MODULE_DESCRIPTION("SPI driver based on DW SPI Core on RTK platform");
MODULE_LICENSE("GPL v2");
