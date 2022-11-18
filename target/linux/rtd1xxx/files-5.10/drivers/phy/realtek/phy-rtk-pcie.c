// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek PCIe Controller PHY Driver
 *
 * Copyright (C) 2020 Realtek
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/phy/phy.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>



#define DRV_NAME "rtd-pcie-phy"

#define PCIE_MDIO_CTR 0xC1C
#define PCIE_SYS_CTR 0xC00
#define MDIO_BUSY		BIT(7)
#define MDIO_RDY		BIT(4)
#define MDIO_WRITE		BIT(0)
#define MDIO_REG_SHIFT		8
#define MDIO_DATA_SHIFT		16

struct rtd_pcie_phy {
	struct device *dev;
	struct reset_control *rst_phy;
	struct reset_control *rst_phy_mdio;
	struct regmap *pcie_base;
};

static int mdio_wait_busy(struct rtd_pcie_phy *rtd_phy)
{
	unsigned int val;
	int cnt = 0;

	regmap_read(rtd_phy->pcie_base, PCIE_MDIO_CTR, &val);
	while ((val & MDIO_BUSY) && cnt < 10) {
		udelay(10);
		regmap_read(rtd_phy->pcie_base, PCIE_MDIO_CTR, &val);
		cnt++;
	}

	if (val & MDIO_BUSY)
		return -EBUSY;

	return 0;
}

static int write_mdio_reg(struct rtd_pcie_phy *rtd_phy, u8 reg, u16 data)
{
	unsigned int val;

	if (mdio_wait_busy(rtd_phy))
		goto mdio_busy;

	val = (reg << MDIO_REG_SHIFT) |
			(data << MDIO_DATA_SHIFT) | MDIO_WRITE;
	regmap_write(rtd_phy->pcie_base, PCIE_MDIO_CTR, val);

	return 0;
mdio_busy:
	dev_err(rtd_phy->dev, "%s - mdio is busy\n", __func__);
	return -EBUSY;
}

static int read_mdio_reg(struct rtd_pcie_phy *rtd_phy, u8 reg)
{
	unsigned int addr;
	unsigned int val;

	if (mdio_wait_busy(rtd_phy))
		goto mdio_busy;

	addr = reg << MDIO_REG_SHIFT;
	regmap_write(rtd_phy->pcie_base, PCIE_MDIO_CTR, addr);

	if (mdio_wait_busy(rtd_phy))
		goto mdio_busy;

	regmap_read(rtd_phy->pcie_base, PCIE_MDIO_CTR, &val);
	return val >> MDIO_DATA_SHIFT;
mdio_busy:
	dev_err(rtd_phy->dev, "%s - mdio is busy\n", __func__);
	return -EBUSY;
}

static int rtd13xx_pcie_phy_power_on(struct phy *phy)
{
	struct rtd_pcie_phy *rtd_phy = phy_get_drvdata(phy);

	reset_control_deassert(rtd_phy->rst_phy);
	reset_control_deassert(rtd_phy->rst_phy_mdio);

	return 0;
}

static int rtd13xx_pcie_phy_power_off(struct phy *phy)
{
	struct rtd_pcie_phy *rtd_phy = phy_get_drvdata(phy);

	reset_control_assert(rtd_phy->rst_phy);
	reset_control_assert(rtd_phy->rst_phy_mdio);

	return 0;
}

static int rtd13xx_pcie_phy_init(struct phy *phy)
{
	struct rtd_pcie_phy *rtd_phy = phy_get_drvdata(phy);

	write_mdio_reg(rtd_phy, 0x06, 0x000C);
	write_mdio_reg(rtd_phy, 0x04, 0x52F5);
	write_mdio_reg(rtd_phy, 0x06, 0x000C);
	write_mdio_reg(rtd_phy, 0x0A, 0xC210);
	write_mdio_reg(rtd_phy, 0x29, 0xFF00);
	write_mdio_reg(rtd_phy, 0x01, 0xA852);
	write_mdio_reg(rtd_phy, 0x0B, 0xB905);
	write_mdio_reg(rtd_phy, 0x09, 0x620C);
	write_mdio_reg(rtd_phy, 0x24, 0x4F08);
	write_mdio_reg(rtd_phy, 0x0D, 0xF712);
	write_mdio_reg(rtd_phy, 0x23, 0xCB66);
	write_mdio_reg(rtd_phy, 0x20, 0xC466);
	write_mdio_reg(rtd_phy, 0x21, 0x5577);
	write_mdio_reg(rtd_phy, 0x22, 0x0033);
	write_mdio_reg(rtd_phy, 0x2F, 0x61BD);
	write_mdio_reg(rtd_phy, 0x0E, 0x1000);
	write_mdio_reg(rtd_phy, 0x2B, 0xB801);
	write_mdio_reg(rtd_phy, 0x1B, 0x8EA1);
	write_mdio_reg(rtd_phy, 0x09, 0x600C);
	write_mdio_reg(rtd_phy, 0x09, 0x620C);
	write_mdio_reg(rtd_phy, 0x46, 0x000C);
	write_mdio_reg(rtd_phy, 0x44, 0x52F5);
	write_mdio_reg(rtd_phy, 0x4A, 0xC210);
	write_mdio_reg(rtd_phy, 0x69, 0xFF00);
	write_mdio_reg(rtd_phy, 0x41, 0xA84A);
	write_mdio_reg(rtd_phy, 0x4B, 0xB905);
	write_mdio_reg(rtd_phy, 0x49, 0x620C);
	write_mdio_reg(rtd_phy, 0x64, 0x4F0C);
	write_mdio_reg(rtd_phy, 0x4D, 0xF712);
	write_mdio_reg(rtd_phy, 0x63, 0xCB66);
	write_mdio_reg(rtd_phy, 0x60, 0xC466);
	write_mdio_reg(rtd_phy, 0x61, 0x8866);
	write_mdio_reg(rtd_phy, 0x62, 0x0033);
	write_mdio_reg(rtd_phy, 0x6F, 0x91BD);
	write_mdio_reg(rtd_phy, 0x4E, 0x1000);
	write_mdio_reg(rtd_phy, 0x6B, 0xB801);
	write_mdio_reg(rtd_phy, 0x49, 0x600C);
	write_mdio_reg(rtd_phy, 0x49, 0x620C);

	return 0;
}

static int rtd16xxb_pcie_phy_init(struct phy *phy)
{
	struct rtd_pcie_phy *rtd_phy = phy_get_drvdata(phy);
	int val;
	int cnt;
	int tmp;


	/*Gen1*/
	write_mdio_reg(rtd_phy, 0x29, 0xFF13);
	write_mdio_reg(rtd_phy, 0x2A, 0x3D60);
	write_mdio_reg(rtd_phy, 0x05, 0xFAD3);
	write_mdio_reg(rtd_phy, 0x06, 0x0013);
	write_mdio_reg(rtd_phy, 0x01, 0xA852);
	write_mdio_reg(rtd_phy, 0x0A, 0xB650);
	write_mdio_reg(rtd_phy, 0x28, 0xF802);
	write_mdio_reg(rtd_phy, 0x0A, 0xB670);
	write_mdio_reg(rtd_phy, 0x24, 0x4F10);
	write_mdio_reg(rtd_phy, 0x23, 0xCB66);
	write_mdio_reg(rtd_phy, 0x20, 0xC4CC);
	write_mdio_reg(rtd_phy, 0x22, 0x0013);
	write_mdio_reg(rtd_phy, 0x21, 0x55AA);
	write_mdio_reg(rtd_phy, 0x2B, 0xA801);
	write_mdio_reg(rtd_phy, 0x2F, 0xA008);
	write_mdio_reg(rtd_phy, 0x0B, 0x9905);
	write_mdio_reg(rtd_phy, 0x09, 0x720C);
	write_mdio_reg(rtd_phy, 0x29, 0xFF13);
	/*Gen2*/
	write_mdio_reg(rtd_phy, 0x69, 0xFF13);
	write_mdio_reg(rtd_phy, 0x6A, 0x3D60);
	write_mdio_reg(rtd_phy, 0x45, 0xFAD3);
	write_mdio_reg(rtd_phy, 0x5E, 0x6EEB);
	write_mdio_reg(rtd_phy, 0x46, 0x0013);
	write_mdio_reg(rtd_phy, 0x41, 0xA84A);
	write_mdio_reg(rtd_phy, 0x4A, 0xB650);
	write_mdio_reg(rtd_phy, 0x68, 0xF802);
	write_mdio_reg(rtd_phy, 0x63, 0xCB66);
	write_mdio_reg(rtd_phy, 0x60, 0xC4EE);
	write_mdio_reg(rtd_phy, 0x62, 0x0013);
	write_mdio_reg(rtd_phy, 0x61, 0x55AA);
	write_mdio_reg(rtd_phy, 0x6B, 0xA801);
	write_mdio_reg(rtd_phy, 0x6F, 0xA008);
	write_mdio_reg(rtd_phy, 0x4B, 0x9905);
	write_mdio_reg(rtd_phy, 0x49, 0x720C);
	write_mdio_reg(rtd_phy, 0x69, 0xFF13);
	/*OOBS*/
	write_mdio_reg(rtd_phy, 0x09, 0x720C);
	write_mdio_reg(rtd_phy, 0x49, 0x720C);
	write_mdio_reg(rtd_phy, 0x09, 0x700C);
	write_mdio_reg(rtd_phy, 0x49, 0x700C);
	write_mdio_reg(rtd_phy, 0x09, 0x720C);
	write_mdio_reg(rtd_phy, 0x49, 0x720C);
	val = read_mdio_reg(rtd_phy, 0x0D);
	val |= BIT(6);
	write_mdio_reg(rtd_phy, 0x0D, val);
	val = read_mdio_reg(rtd_phy, 0x4D);
	val |= BIT(6);
	write_mdio_reg(rtd_phy, 0x4D, val);
	val = read_mdio_reg(rtd_phy, 0x19);
	val |= BIT(2);
	write_mdio_reg(rtd_phy, 0x19, val);
	val = read_mdio_reg(rtd_phy, 0x59);
	val |= BIT(2);
	write_mdio_reg(rtd_phy, 0x59, val);
	write_mdio_reg(rtd_phy, 0x10, 0x03C4);
	write_mdio_reg(rtd_phy, 0x50, 0x03C4);

	cnt = 0;
	val = read_mdio_reg(rtd_phy, 0x1f);
	while ((val & BIT(6)) != 0 && cnt < 10) {
		udelay(10);
		val = read_mdio_reg(rtd_phy, 0x1f);
		cnt++;
	}
	if (cnt == 10) {
		dev_err(rtd_phy->dev, "wait mdio reg(0x1f) bit6 == 0 timeout\n");
		return -EBUSY;
	}
	cnt = 0;
	val = read_mdio_reg(rtd_phy, 0x5f);
	while ((val & BIT(6)) != 0 && cnt < 10) {
		udelay(10);
		val = read_mdio_reg(rtd_phy, 0x5f);
		cnt++;
	}
	if (cnt == 10) {
		dev_err(rtd_phy->dev, "wait mdio reg(0x5f) bit6 == 0 timeout\n");
		return -EBUSY;
	}
	mdelay(1);
	val = read_mdio_reg(rtd_phy, 0x19);
	val |= BIT(2);
	write_mdio_reg(rtd_phy, 0x19, val);
	val = read_mdio_reg(rtd_phy, 0x59);
	val |= BIT(2);
	write_mdio_reg(rtd_phy, 0x59, val);
	write_mdio_reg(rtd_phy, 0x10, 0x03C4);
	write_mdio_reg(rtd_phy, 0x50, 0x03C4);
	tmp = read_mdio_reg(rtd_phy, 0x1f);
	tmp = (tmp & GENMASK(12, 8)) >> 8;
	val = read_mdio_reg(rtd_phy, 0x03);
	val = (val & ~ GENMASK(5, 1)) | (tmp << 1);
	write_mdio_reg(rtd_phy, 0x03, val);
	tmp = read_mdio_reg(rtd_phy, 0x5f);
	tmp = (tmp & GENMASK(12, 8)) >> 8;

	val = read_mdio_reg(rtd_phy, 0x43);
	val = (val & ~ GENMASK(5, 1)) | (tmp << 1);
	write_mdio_reg(rtd_phy, 0x43, val);
	write_mdio_reg(rtd_phy, 0x09, 0x721C);
	write_mdio_reg(rtd_phy, 0x49, 0x721C);
	cnt = 0;
	val = read_mdio_reg(rtd_phy, 0x1f);
	while (!(val & BIT(15)) && cnt < 10) {
		udelay(10);
		val = read_mdio_reg(rtd_phy, 0x1f);
		cnt++;
	}
	if (cnt == 10) {
		dev_err(rtd_phy->dev, "wait mdio reg(0x1f) bit15 == 1 timeout\n");
		return -EBUSY;
	}
	cnt = 0;
	val = read_mdio_reg(rtd_phy, 0x5f);
	while (!(val & BIT(15)) && cnt < 10) {
		udelay(10);
		val = read_mdio_reg(rtd_phy, 0x5f);
		cnt++;
	}
	if (cnt == 10) {
		dev_err(rtd_phy->dev, "wait mdio reg(0x5f) bit15 == 1  timeout\n");
		return -EBUSY;
	}

	regmap_write(rtd_phy->pcie_base, PCIE_SYS_CTR, 0x00140012);

	mdelay(1);
	val = read_mdio_reg(rtd_phy, 0x19);
	val &= ~BIT(2);
	write_mdio_reg(rtd_phy, 0x19, val);
	val = read_mdio_reg(rtd_phy, 0x59);
	val &= ~BIT(2);
	write_mdio_reg(rtd_phy, 0x59, val);
	write_mdio_reg(rtd_phy, 0x10, 0x000C);
	write_mdio_reg(rtd_phy, 0x50, 0x000C);
	val = read_mdio_reg(rtd_phy, 0x0D);
	val &= ~BIT(6);
	write_mdio_reg(rtd_phy, 0x0D, val);
	val = read_mdio_reg(rtd_phy, 0x4D);
	val &= ~BIT(6);
	write_mdio_reg(rtd_phy, 0x4D, val);

	return 0;

}

static int rtd13xxd_pcie_phy_init(struct phy *phy)
{
	//struct rtd_pcie_phy *rtd_phy = phy_get_drvdata(phy);

	/*TODO*/

	return 0;
}


static const struct phy_ops rtd13xx_pcie_phy_ops = {
	.init		= rtd13xx_pcie_phy_init,
	.power_on	= rtd13xx_pcie_phy_power_on,
	.power_off	= rtd13xx_pcie_phy_power_off,
	.owner		= THIS_MODULE,
};

static const struct phy_ops rtd16xxb_pcie_phy_ops = {
	.init		= rtd16xxb_pcie_phy_init,
	.power_on	= rtd13xx_pcie_phy_power_on,
	.power_off	= rtd13xx_pcie_phy_power_off,
	.owner		= THIS_MODULE,
};

static const struct phy_ops rtd13xxd_pcie_phy_ops = {
	.init		= rtd13xxd_pcie_phy_init,
	.power_on	= rtd13xx_pcie_phy_power_on,
	.power_off	= rtd13xx_pcie_phy_power_off,
	.owner		= THIS_MODULE,
};


static int rtd_pcie_phy_probe(struct platform_device *pdev)
{
	struct device_node *syscon_np;
	struct rtd_pcie_phy *rtd_phy;
	struct phy_ops *ops;
	struct phy_provider *phy_provider;
	struct phy *phy;

	rtd_phy = devm_kzalloc(&pdev->dev, sizeof(*rtd_phy), GFP_KERNEL);
	if (!rtd_phy)
		return -ENOMEM;

	rtd_phy->dev = &pdev->dev;

	ops = (struct phy_ops *)of_device_get_match_data(rtd_phy->dev);

	syscon_np = of_parse_phandle(rtd_phy->dev->of_node, "syscon", 0);
	if (IS_ERR_OR_NULL(syscon_np))
		return -ENODEV;

	rtd_phy->pcie_base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(rtd_phy->pcie_base)) {
		of_node_put(syscon_np);
		return -EINVAL;
	}

	rtd_phy->rst_phy = devm_reset_control_get(rtd_phy->dev, "phy");
	if (rtd_phy->rst_phy == NULL) {
		dev_err(rtd_phy->dev, "phy source missing or invalid\n");
		return -EINVAL;
	}

	rtd_phy->rst_phy_mdio = devm_reset_control_get(rtd_phy->dev, "phy_mdio");
	if (rtd_phy->rst_phy_mdio == NULL) {
		dev_err(rtd_phy->dev, "phy_mdio source missing. or invalid\n");
		return -EINVAL;
	}

	phy = devm_phy_create(rtd_phy->dev, rtd_phy->dev->of_node, ops);
	if (IS_ERR(phy)) {
		dev_err(rtd_phy->dev, "failed to create phy\n");
		return PTR_ERR(phy);
	}

	phy_set_drvdata(phy, rtd_phy);

	phy_provider = devm_of_phy_provider_register(rtd_phy->dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		dev_err(rtd_phy->dev, "failed to register phy provider\n");

	dev_info(rtd_phy->dev, "init done\n");
	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id rtd_pcie_phy_of_match[] = {
	{ .compatible = "realtek,rtd13xx-pcie-slot0-phy", .data = &rtd13xx_pcie_phy_ops},
	{ .compatible = "realtek,rtd13xx-pcie-slot1-phy", .data = &rtd13xx_pcie_phy_ops},
	{ .compatible = "realtek,rtd13xx-pcie-slot2-phy", .data = &rtd13xx_pcie_phy_ops},
	{ .compatible = "realtek,rtd16xxb-pcie-slot1-phy", .data = &rtd16xxb_pcie_phy_ops},
	{ .compatible = "realtek,rtd16xxb-pcie-slot2-phy", .data = &rtd16xxb_pcie_phy_ops},
	{ .compatible = "realtek,rtd13xxd-pcie-slot1-phy", .data = &rtd13xxd_pcie_phy_ops},
	{ },
};

static struct platform_driver rtd_pcie_phy_driver = {
	.probe	= rtd_pcie_phy_probe,
	.driver	= {
		.name = DRV_NAME,
		.of_match_table	= rtd_pcie_phy_of_match,
	},
};

module_platform_driver(rtd_pcie_phy_driver);

MODULE_DESCRIPTION("Realtek PCIe PHY driver");
MODULE_AUTHOR("TYChang <tychang@realtek.com>");
MODULE_LICENSE("GPL v2");
