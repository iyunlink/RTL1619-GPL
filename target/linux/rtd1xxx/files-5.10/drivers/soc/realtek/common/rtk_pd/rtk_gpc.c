// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek Generic Power Controller
 *
 * Copyright (C) 2021 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <linux/reset.h>
#include <linux/slab.h>

struct rtk_gpc_cfg {
	const char  *name;
	unsigned int iso_bit;
	unsigned long flags;
};

#define GPC_FLAGS_HAS_VE_TOP_CTL    0x1
#define GPC_FLAGS_HAS_ISO_CTL2      0x2
#define GPC_FLAGS_PWRSEQ_V2         0x4
#define GPC_FLAGS_SETUP_DELAYS      0x8
#define GPC_FLAGS_IGNORE_ISO_PWR    0x10

struct rtk_gpc_soc_data {
	unsigned int              iso_offset;
	int                       num_cfgs;
	const struct rtk_gpc_cfg  *cfgs;
};

struct rtk_gpc_data {
	const char                   *name;
	struct device                *dev;
	void __iomem                 *base;
	void __iomem                 *ctl5;
	void __iomem                 *ctl8;
	struct generic_pm_domain     genpd;
	struct reset_controller_dev  rcdev;

	struct regmap                *regmap;
	struct clk_bulk_data         *clks;
	int                          num_clks;
	struct reset_control         *rstn;
	struct reset_control         *rstn_bist;
	struct reset_control         *rstn_auto;
	const struct rtk_gpc_cfg     *cfg;
	unsigned int                 iso_offset;
	int                          power_state_should_sync;
};

static inline int has_ve_top_ctl(struct rtk_gpc_data *data)
{
	return !!(data->cfg->flags & GPC_FLAGS_HAS_VE_TOP_CTL);
}

static inline int has_iso_ctl2(struct rtk_gpc_data *data)
{
	return !!(data->cfg->flags & GPC_FLAGS_HAS_ISO_CTL2);
}

static inline int is_pwrseq_v2(struct rtk_gpc_data *data)
{
	return !!(data->cfg->flags & GPC_FLAGS_PWRSEQ_V2);
}

static inline int should_setup_delay(struct rtk_gpc_data *data)
{
	return !!(data->cfg->flags & GPC_FLAGS_SETUP_DELAYS);
}

static inline int iso_power_ignored(struct rtk_gpc_data *data)
{
	return !!(data->cfg->flags & GPC_FLAGS_IGNORE_ISO_PWR);
}

#define SRAM_PWR0 0x0
#define SRAM_PWR1 0x4
#define SRAM_PWR2 0x8
#define SRAM_PWR3 0xC
#define SRAM_PWR4 0x10
#define SRAM_PWR5 0x14
#define SRAM_PWR6 0x18
#define SRAM_PWR7 0x1c

static void rtk_gpc_iso_set_ctl2(struct rtk_gpc_data *data, int is_on)
{
	unsigned int mask, val;

	if (!has_iso_ctl2(data))
		return;

	mask = BIT(1);
	val = is_on ? 0 : mask;

	regmap_update_bits(data->regmap, 0x078, mask, val);
}

static void rtk_gpc_iso_set_power(struct rtk_gpc_data *data, int is_on)
{
	const struct rtk_gpc_cfg *cfg = data->cfg;
	unsigned int mask, val;

	if (iso_power_ignored(data))
		return;

	mask = BIT(cfg->iso_bit);
	val = is_on ? 0 : mask;

	regmap_update_bits(data->regmap, data->iso_offset, mask, val);

	rtk_gpc_iso_set_ctl2(data, is_on);
}

static void rtk_gpc_iso_power_on(struct rtk_gpc_data *data)
{
	rtk_gpc_iso_set_power(data, 1);
}

static void rtk_gpc_iso_power_off(struct rtk_gpc_data *data)
{
	rtk_gpc_iso_set_power(data, 0);
}

static void rtk_gpc_sram_power_delay_set(struct rtk_gpc_data *data,
		unsigned int l2h, unsigned int hl2, unsigned int std)
{
	if (l2h)
		writel(l2h, data->base + SRAM_PWR0);

	if (hl2)
		writel(hl2, data->base + SRAM_PWR1);

	if (std && data->ctl8)
		writel(std, data->ctl8);
}

static void rtk_gpc_sram_power_manual_mask_set(struct rtk_gpc_data *data, unsigned int mask)
{
	writel(mask, data->base + SRAM_PWR3);
}

static void rtk_gpc_sram_power_setup_config(struct rtk_gpc_data *data)
{
	if (has_ve_top_ctl(data))
		rtk_gpc_sram_power_manual_mask_set(data, 0x00008000);

	if (should_setup_delay(data))
		rtk_gpc_sram_power_delay_set(data, 0xf, 0xf, 0x32);
}

static int rtk_gpc_sram_power_ready(struct rtk_gpc_data *data)
{
	unsigned int pollval = 0;

	return readl_poll_timeout(data->ctl5, pollval, pollval == 0x4, 0, 500);
}

static void rtk_gpc_sram_power_clear_ints(struct rtk_gpc_data *data)
{
	writel(0x4, data->ctl5);
}

static int rtk_gpc_sram_power_set(struct rtk_gpc_data *data, int on_off)
{
	unsigned int val = on_off ? 0 : 1;
	unsigned int reg;
	int ret;

	rtk_gpc_sram_power_setup_config(data);

	reg = readl(data->base + SRAM_PWR4);
	if ((reg & 0xff) == val)
		return 1;
	val |= 0xf << 8;

	writel(val, data->base + SRAM_PWR4);

	ret = rtk_gpc_sram_power_ready(data);

	rtk_gpc_sram_power_clear_ints(data);

	return ret;
}

static int rtk_gpc_sram_power_on(struct rtk_gpc_data *data)
{
	return rtk_gpc_sram_power_set(data, 1);
}

static int rtk_gpc_sram_power_off(struct rtk_gpc_data *data)
{
	return rtk_gpc_sram_power_set(data, 0);
}

static int rtk_gpc_get_sram_power_state(struct rtk_gpc_data *data)
{
	unsigned int val;

	val = readl(data->base + SRAM_PWR4);
	return (val & 0xff) == 0;
}

static void rtk_gpc_enable_clocks(struct rtk_gpc_data *data)
{
	int ret;

	ret = clk_bulk_prepare_enable(data->num_clks, data->clks);
	WARN_ON_ONCE(ret);
}

static void rtk_gpc_disable_clocks(struct rtk_gpc_data *data)
{
	clk_bulk_disable_unprepare(data->num_clks, data->clks);
}

static int rtk_gpc_get_clocks(struct rtk_gpc_data *data)
{
	struct device *dev = data->dev;
	int ret;

	ret = devm_clk_bulk_get_all(dev, &data->clks);
	if (ret < 0) {
		dev_err(dev, "failed to get clk: %d\n", ret);
		return ret;
	}
	data->num_clks = ret;
	return 0;
}

static int rtk_gpc_get_resets(struct rtk_gpc_data *data)
{
	struct device *dev = data->dev;
	int ret;

	data->rstn = devm_reset_control_get_optional_exclusive(dev, "reset");
	if (IS_ERR(data->rstn)) {
		ret = PTR_ERR(data->rstn);
		dev_err(dev, "failed to get rstn: %d\n", ret);
		return ret;
	}

	data->rstn_bist = devm_reset_control_get_optional_exclusive(dev, "bist");
	if (IS_ERR(data->rstn_bist)) {
		ret = PTR_ERR(data->rstn_bist);
		dev_err(dev, "failed to get rstn_bist: %d\n", ret);
		return ret;
	}

	data->rstn_auto = devm_reset_control_get_optional_exclusive(dev, "auto");
	if (IS_ERR(data->rstn_auto)) {
		ret = PTR_ERR(data->rstn_auto);
		dev_err(dev, "failed to get rstn_auto: %d\n", ret);
		return ret;
	}
	return 0;
}

static void rtk_gpc_prepare_power_on(struct rtk_gpc_data *data, int already_power_on)
{
	if (!is_pwrseq_v2(data))
		return;

	rtk_gpc_enable_clocks(data);
}

static void rtk_gpc_setup_suppliers(struct rtk_gpc_data *data, int already_power_on)
{
	if (!is_pwrseq_v2(data))
		rtk_gpc_enable_clocks(data);

	reset_control_deassert(data->rstn);
	if (!already_power_on)
		reset_control_reset(data->rstn_auto);
	reset_control_deassert(data->rstn_bist);
}

static void rtk_gpc_teardown_suppliers(struct rtk_gpc_data *data)
{
	reset_control_assert(data->rstn);
	reset_control_assert(data->rstn_bist);

	/*
	 * if power off is called with power_state_should_sync,
	 * No device is attached, and clk is nerver enabled.
	 */
	if (is_pwrseq_v2(data) || data->power_state_should_sync)
		return;

	rtk_gpc_disable_clocks(data);
}

static void rtk_gpc_complete_power_off(struct rtk_gpc_data *data)
{
	/*
	 * if power off is called with power_state_should_sync,
	 * No device is attached, and clk is nerver enabled.
	 */
	if (!is_pwrseq_v2(data) || data->power_state_should_sync)
		return;

	rtk_gpc_disable_clocks(data);
}

static int rtk_gpc_is_on(struct rtk_gpc_data *data)
{
	return rtk_gpc_get_sram_power_state(data);
}

static int rtk_gpc_genpd_power_on(struct generic_pm_domain *genpd)
{
	struct rtk_gpc_data *data = container_of(genpd, struct rtk_gpc_data, genpd);
	int ret;

	dev_dbg(data->dev, "%s\n", __func__);

	rtk_gpc_prepare_power_on(data, 0);

	ret = rtk_gpc_sram_power_on(data);

	rtk_gpc_setup_suppliers(data, ret > 0);

	rtk_gpc_iso_power_on(data);

	return 0;
}

static int rtk_gpc_genpd_power_off(struct generic_pm_domain *genpd)
{
	struct rtk_gpc_data *data = container_of(genpd, struct rtk_gpc_data, genpd);

	dev_dbg(data->dev, "%s\n", __func__);

	rtk_gpc_iso_power_off(data);

	rtk_gpc_teardown_suppliers(data);

	rtk_gpc_sram_power_off(data);

	rtk_gpc_complete_power_off(data);

	data->power_state_should_sync = 0;
	return 0;
}

static int rtk_gpc_genpd_attach_dev(struct generic_pm_domain *genpd, struct device *dev)
{
	struct rtk_gpc_data *data = container_of(genpd, struct rtk_gpc_data, genpd);

	pr_debug("%s: %s %s %s\n", genpd->name, __func__, dev_driver_string(dev), dev_name(dev));

	/* sync power on device attached */
	if (data->power_state_should_sync) {
		rtk_gpc_prepare_power_on(data, 1);

		rtk_gpc_setup_suppliers(data, 1);

		rtk_gpc_iso_power_on(data);

		data->power_state_should_sync = 0;
	}
	return 0;
}

static void rtk_gpc_genpd_detach_dev(struct generic_pm_domain *genpd, struct device *dev)
{
	pr_debug("%s: %s %s %s\n", genpd->name, __func__, dev_driver_string(dev), dev_name(dev));
}

static int rtk_gpc_reset_reset(struct reset_controller_dev *rcdev, unsigned long idx)
{
	struct rtk_gpc_data *data = container_of(rcdev, struct rtk_gpc_data, rcdev);
	int ret;

	if (!data->rstn && !data->rstn_auto)
		return -EINVAL;

	ret = reset_control_reset(data->rstn);
	if (ret)
		return ret;
	return reset_control_reset(data->rstn_auto);
}

static const struct reset_control_ops rtk_gpc_reset_ops = {
	.reset = rtk_gpc_reset_reset,
};

static int rtk_gpc_reset_of_xlate(struct reset_controller_dev *rcdev,
				    const struct of_phandle_args *reset_spec)
{
	if (WARN_ON(reset_spec->args_count != 0))
		return -EINVAL;

	return 0;
}

static const struct rtk_gpc_cfg *rtk_gpc_find_cfg_by_name(const struct rtk_gpc_soc_data *soc_data,
	const char *name)
{
	int i;

	for (i = 0; i < soc_data->num_cfgs; i++)
		if (!strcmp(name, soc_data->cfgs[i].name))
			return &soc_data->cfgs[i];

	return NULL;
}

static int rtk_gpc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rtk_gpc_data *data;
	struct resource *res;
	int ret;
	int power_off;
	const struct rtk_gpc_soc_data *soc_data;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = of_property_read_string(np, "label", &data->name);
	if (ret) {
		dev_err(dev, "failed to get label: %d\n", ret);
		return ret;
	}

	soc_data = of_device_get_match_data(dev);
	if (!soc_data) {
		dev_err(dev, "no match data\n");
		return -EINVAL;
	}

	data->dev = dev;
	data->iso_offset = soc_data->iso_offset;

	data->cfg = rtk_gpc_find_cfg_by_name(soc_data, data->name);
	if (!data->cfg) {
		dev_err(dev, "invalid config for %s\n", data->name);
		return -EINVAL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ctl");
	data->base = devm_ioremap_resource(dev, res);
	if (!data->base)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ctl5");
	if (!res)
		data->ctl5 = data->base + SRAM_PWR5;
	else
		data->ctl5 = devm_ioremap_resource(dev, res);
	if (!data->ctl5)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ctl8");
	data->ctl8 = devm_ioremap_resource(dev, res);

	data->regmap = syscon_node_to_regmap(np->parent);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(dev, "failed to get syscon regmap from parent: %d\n", ret);
		return ret;
	}

	ret = rtk_gpc_get_clocks(data);
	if (ret)
		return ret;

	ret = rtk_gpc_get_resets(data);
	if (ret)
		return ret;

	if (data->rstn || data->rstn_auto) {
		data->rcdev.owner            = THIS_MODULE;
		data->rcdev.ops              = &rtk_gpc_reset_ops;
		data->rcdev.nr_resets        = 1;
		data->rcdev.of_node          = dev->of_node;
		data->rcdev.of_reset_n_cells = 0;
		data->rcdev.of_xlate         = rtk_gpc_reset_of_xlate;

		ret = devm_reset_controller_register(dev, &data->rcdev);
		if (ret) {
			dev_err(dev, "failed to register reset_controller: %d\n", ret);
			return ret;
		}
	}

	power_off = !rtk_gpc_is_on(data);
	if (!power_off)
		data->power_state_should_sync = 1;

	dev_set_drvdata(dev, data);

	data->genpd.name       = data->name;
	data->genpd.power_on   = rtk_gpc_genpd_power_on;
	data->genpd.power_off  = rtk_gpc_genpd_power_off;
	data->genpd.attach_dev = rtk_gpc_genpd_attach_dev;
	data->genpd.detach_dev = rtk_gpc_genpd_detach_dev;
	ret = pm_genpd_init(&data->genpd, NULL, power_off);
	if (ret) {
		dev_err(dev, "failed to init genpd: %d\n", ret);
		return ret;
	}

	ret = of_genpd_add_provider_simple(np, &data->genpd);
	if (ret) {
		dev_err(dev, "failed to add genpd of provider: %d\n", ret);
		pm_genpd_remove(&data->genpd);
	}
	return ret;
}


#define GPC_SOC_DATA(_offset, _cfgs) \
{ \
	.iso_offset = _offset, \
	.cfgs = _cfgs, \
	.num_cfgs = ARRAY_SIZE(_cfgs), \
}

#define DEFINE_GPC_SOC_DATA(_name, _offset, _cfgs) \
struct rtk_gpc_soc_data _name = GPC_SOC_DATA(_offset, _cfgs)

#define GPC_CFG(_name, _bit, _flags) \
{ \
	.name = _name, \
	.iso_bit = _bit, \
	.flags = _flags, \
}

static const struct rtk_gpc_cfg rtd1295_gpc_cfgs[] = {
	GPC_CFG("ve1", 0, 0),
	GPC_CFG("gpu", 1, 0),
	GPC_CFG("ve2", 4, 0),
	GPC_CFG("ve3", 6, 0),
	GPC_CFG("nat", 18, 0),
};
static const DEFINE_GPC_SOC_DATA(rtd1295_soc_data, 0x400, rtd1295_gpc_cfgs);

static const struct rtk_gpc_cfg rtd1395_gpc_cfgs[] = {
	GPC_CFG("ve1", 0, GPC_FLAGS_HAS_VE_TOP_CTL),
	GPC_CFG("ve2", 1, 0),
	GPC_CFG("gpu", 3, 0),
	GPC_CFG("ve3", 10, 0),
};
static const DEFINE_GPC_SOC_DATA(rtd1395_soc_data, 0xfd0, rtd1395_gpc_cfgs);

static const struct rtk_gpc_cfg rtd1619_gpc_cfgs[] = {
	GPC_CFG("ve1", 0, GPC_FLAGS_HAS_VE_TOP_CTL),
	GPC_CFG("ve2", 1, 0),
	GPC_CFG("gpu", 3, GPC_FLAGS_HAS_ISO_CTL2),
	GPC_CFG("hdmirx", 9, 0),
	GPC_CFG("ve3", 10, 0),
};
static const DEFINE_GPC_SOC_DATA(rtd1619_soc_data, 0xfd0, rtd1619_gpc_cfgs);

static const struct rtk_gpc_cfg rtd1319_gpc_cfgs[] = {
	GPC_CFG("ve1", 0, 0),
	GPC_CFG("ve2", 1, GPC_FLAGS_IGNORE_ISO_PWR),
	GPC_CFG("gpu", 3, 0),
	GPC_CFG("ve3", 10, 0),
};
static const DEFINE_GPC_SOC_DATA(rtd1319_soc_data, 0xfd0, rtd1319_gpc_cfgs);

static const struct rtk_gpc_cfg rtd1619b_gpc_cfgs[] = {
	GPC_CFG("ve1", 0, 0),
	GPC_CFG("ve2", 1, GPC_FLAGS_IGNORE_ISO_PWR),
	GPC_CFG("gpu", 3, 0),
	GPC_CFG("ve3", 10, 0),
	GPC_CFG("npu", 12, GPC_FLAGS_PWRSEQ_V2 | GPC_FLAGS_SETUP_DELAYS),
};
static const DEFINE_GPC_SOC_DATA(rtd1619b_soc_data, 0xfd0, rtd1619b_gpc_cfgs);

static const struct rtk_gpc_cfg rtd1319d_gpc_cfgs[] = {
	GPC_CFG("ve1", 0, GPC_FLAGS_PWRSEQ_V2 | GPC_FLAGS_SETUP_DELAYS),
	GPC_CFG("ve2", 1, GPC_FLAGS_IGNORE_ISO_PWR),
	GPC_CFG("gpu", 3, 0),
	GPC_CFG("ve3", 10, 0),
};
static const DEFINE_GPC_SOC_DATA(rtd1319d_soc_data, 0xfd0, rtd1319d_gpc_cfgs);

static const struct of_device_id rtk_gpc_match[] = {
	{ .compatible = "realtek,rtd1295-gpc", .data = &rtd1295_soc_data, },
	{ .compatible = "realtek,rtd1395-gpc", .data = &rtd1395_soc_data, },
	{ .compatible = "realtek,rtd1619-gpc", .data = &rtd1619_soc_data, },
	{ .compatible = "realtek,rtd1319-gpc", .data = &rtd1319_soc_data, },
	{ .compatible = "realtek,rtd1619b-gpc", .data = &rtd1619b_soc_data, },
	{ .compatible = "realtek,rtd1319d-gpc", .data = &rtd1319d_soc_data, },
	{}
};

static struct platform_driver rtk_gpc_driver = {
	.probe = rtk_gpc_probe,
	.driver = {
		.name = "rtk-gpc",
		.of_match_table = of_match_ptr(rtk_gpc_match),
	},
};

static int __init rtk_gpc_init(void)
{
	return platform_driver_register(&rtk_gpc_driver);
}
fs_initcall(rtk_gpc_init);

MODULE_DESCRIPTION("Realtek Generic Power Controller");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");
