// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Realtek PCIe MMIO Translate driver
 *
 * Copyright (c) 2017 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#define PCIE_IO_64K_MASK 0xFFFF0000

struct regmap *pcie0_base;
struct regmap *pcie1_base;
struct regmap *pcie2_base;

struct pcie_info {
	struct regmap *base;
	spinlock_t spinlock;
	void __iomem *mmio_base;
	struct device *dev;
};

struct pcie_info *pcie0_info;
struct pcie_info *pcie1_info;
struct pcie_info *pcie2_info;



u32 rtk_pcie_ctrl_read(struct pcie_info *info, u32 offset)
{
	u32 val;

	regmap_read(info->base, offset, &val);

	return val;
}

void rtk_pcie_ctrl_write(struct pcie_info *info, u32 offset, u32 val)
{
	regmap_write(info->base, offset, val);
}

u32 rtk_pcie_mmio_read(struct pcie_info *info, u32 offset, u32 size)
{
	u32 val;

	switch (size) {
	case 1:
		val = readb(info->mmio_base + offset);
		break;
	case 2:
		val = readw(info->mmio_base + offset);
		break;
	case 4:
		val = readl(info->mmio_base + offset);
		break;
	default:
		dev_err(info->dev, "RTD13xx: %s: wrong size %d\n", __func__, size);
		val = 0;
		break;
	}

	return val;
}

void rtk_pcie_mmio_write(struct pcie_info *info, u32 offset, u32 val, u32 size)
{
	switch (size) {
	case 1:
		writeb(val, info->mmio_base + offset);
		break;
	case 2:
		writew(val, info->mmio_base + offset);
		break;
	case 4:
		writel(val, info->mmio_base + offset);
		break;
	default:
		dev_err(info->dev, "RTD13xx: %s: wrong size %d\n", __func__, size);
		break;
	}
}


u32 rtk_pcie_read(struct pcie_info *info, u32 addr, u8 size)
{
	u32 rval = 0;
	u32 mask;
	u32 translate_val = 0;
	unsigned long irqL;
	u32 pci_error_status = 0;
	int retry_cnt = 0;
	u8 retry = 5;

	spin_lock_irqsave(&info->spinlock, irqL);

	if (addr >= 0x10000) {
		mask = PCIE_IO_64K_MASK;
		translate_val = rtk_pcie_ctrl_read(info, 0xD04);
		rtk_pcie_ctrl_write(info, 0xD04, translate_val | (addr & mask));
	} else
		mask = 0x0;

pci_read_13xx_retry:

	rval = rtk_pcie_mmio_read(info, addr & ~mask, size);


	//DLLP error patch
	pci_error_status = rtk_pcie_ctrl_read(info, 0xc7c);
	if (pci_error_status & 0x1F) {
		rtk_pcie_ctrl_write(info, 0xc7c, pci_error_status);
		dev_err(info->dev, "RTD13xx: %s: DLLP(#%d) 0x%x reg=0x%x val=0x%x\n",
			__func__, retry_cnt, pci_error_status, addr, rval);

		if (retry_cnt < retry) {
			retry_cnt++;
			goto pci_read_13xx_retry;
		}
	}

	if (addr >= 0x10000)
		rtk_pcie_ctrl_write(info, 0xD04, translate_val);


	spin_unlock_irqrestore(&info->spinlock, irqL);

	return rval;
}


void rtk_pcie_write(struct pcie_info *info, u32 addr, u8 size, u32 wval)
{
	u32 mask;
	u32 translate_val = 0;
	unsigned long irqL;

	spin_lock_irqsave(&info->spinlock, irqL);

	if (addr >= 0x10000) {
		mask = PCIE_IO_64K_MASK;
		translate_val = rtk_pcie_ctrl_read(info, 0xD04);
		rtk_pcie_ctrl_write(info, 0xD04, translate_val | (addr & mask));
	} else
		mask = 0x0;

	rtk_pcie_mmio_write(info, addr & ~mask, wval, size);

	if (addr >= 0x10000)
		rtk_pcie_ctrl_write(info, 0xD04, translate_val);


	spin_unlock_irqrestore(&info->spinlock, irqL);
}

u32 rtk_pcie_13xx_read(u32 addr, u8 size)
{
	return rtk_pcie_read(pcie0_info, addr, size);
}
EXPORT_SYMBOL(rtk_pcie_13xx_read);


void rtk_pcie_13xx_write(u32 addr, u8 size, u32 wval)
{
	rtk_pcie_write(pcie0_info, addr, size, wval);
}
EXPORT_SYMBOL(rtk_pcie_13xx_write);


u32 rtk_pcie2_13xx_read(u32 addr, u8 size)
{
	return rtk_pcie_read(pcie1_info, addr, size);
}
EXPORT_SYMBOL(rtk_pcie2_13xx_read);


void rtk_pcie2_13xx_write(u32 addr, u8 size, u32 wval)
{
	rtk_pcie_write(pcie1_info, addr, size, wval);
}
EXPORT_SYMBOL(rtk_pcie2_13xx_write);


u32 rtk_pcie3_13xx_read(u32 addr, u8 size)
{
	return rtk_pcie_read(pcie2_info, addr, size);
}
EXPORT_SYMBOL(rtk_pcie3_13xx_read);


void rtk_pcie3_13xx_write(u32 addr, u8 size, u32 wval)
{
	rtk_pcie_write(pcie2_info, addr, size, wval);
}
EXPORT_SYMBOL(rtk_pcie3_13xx_write);


static int rtd_pcie_trans_probe(struct platform_device *pdev)
{
	struct device_node *syscon_np;

	syscon_np = of_parse_phandle(pdev->dev.of_node, "syscon", 0);
	if (IS_ERR_OR_NULL(syscon_np))
		return -ENODEV;

	pcie0_info = devm_kzalloc(&pdev->dev, sizeof(*pcie0_info), GFP_KERNEL);
	pcie0_info->dev = &pdev->dev;
	pcie0_info->base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(pcie0_info->base)) {
		of_node_put(syscon_np);
		return -EINVAL;
	}

	pcie0_info->mmio_base = of_iomap(pdev->dev.of_node, 0);
	if (!pcie0_info->mmio_base) {
		dev_err(&pdev->dev, "failed to get pcie0 mmio address\n");
		return -EINVAL;
	}

	spin_lock_init(&pcie0_info->spinlock);

	syscon_np = of_parse_phandle(pdev->dev.of_node, "syscon", 1);
	if (IS_ERR_OR_NULL(syscon_np))
		return -ENODEV;

	pcie1_info = devm_kzalloc(&pdev->dev, sizeof(*pcie1_info), GFP_KERNEL);
	pcie1_info->dev = &pdev->dev;
	pcie1_info->base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(pcie1_info->base)) {
		of_node_put(syscon_np);
		return -EINVAL;
	}
	pcie1_info->mmio_base = of_iomap(pdev->dev.of_node, 1);
	if (!pcie1_info->mmio_base) {
		dev_err(&pdev->dev, "failed to get pcie1 mmio address\n");
		return -EINVAL;
	}

	spin_lock_init(&pcie1_info->spinlock);

	syscon_np = of_parse_phandle(pdev->dev.of_node, "syscon", 2);
	if (IS_ERR_OR_NULL(syscon_np))
		return -ENODEV;

	pcie2_info = devm_kzalloc(&pdev->dev, sizeof(*pcie2_info), GFP_KERNEL);
	pcie2_info->dev = &pdev->dev;
	pcie2_info->base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(pcie2_info->base)) {
		of_node_put(syscon_np);
		return -EINVAL;
	}

	pcie2_info->mmio_base = of_iomap(pdev->dev.of_node, 2);
	if (!pcie2_info->mmio_base) {
		dev_err(&pdev->dev, "failed to get pcie2 mmio address\n");
		return -EINVAL;
	}
	spin_lock_init(&pcie2_info->spinlock);

	dev_info(&pdev->dev, "\n===========rtd13xx pcie mmio translate ready\n");

	return 0;
}

static const struct of_device_id rtd_pcie_trans_match_table[] = {
	{.compatible = "realtek,rtd13xx-pcie-trans"},
	{},
};

static struct platform_driver rtd_pcie_trans_driver = {
	.driver = {
		.name = "Realtek DHC PCIe mmio translate",
		.of_match_table = of_match_ptr(rtd_pcie_trans_match_table),
	},
	.probe = rtd_pcie_trans_probe,
};
module_platform_driver(rtd_pcie_trans_driver);

MODULE_AUTHOR("TYChang <tychang@realtek.com>");
MODULE_DESCRIPTION("Realtek PCIe MMIO translate driver");
MODULE_LICENSE("GPL v2");

