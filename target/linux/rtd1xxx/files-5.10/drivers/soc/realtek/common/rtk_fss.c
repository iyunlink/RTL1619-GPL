// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Realtek Semiconductor Corp.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <soc/realtek/rtk_fss.h>

/* fss registers */
#define SC_WRAP_DVFS_FSS_CTRL0    0x00
#define SC_WRAP_DVFS_FSS_CTRL1    0x04
#define SC_WRAP_DVFS_FSS_CTRL2    0x08
#define SC_WRAP_DVFS_FSS_CTRL3    0x0c
#define SC_WRAP_DVFS_FSS_CTRL4    0x10
#define SC_WRAP_DVFS_FSS_ST0      0x14
#define SC_WRAP_DVFS_FSS_ST1      0x18
#define SC_WRAP_DVFS_FSS_ST2      0x1c
#define SC_WRAP_DVFS_FSS_ST3      0x20

/* fss v4 registers */
#define SC_WRAP_DVFS_FSS_CTRL5    0x24
#define SC_WRAP_DVFS_FSS_ST6      0x28

/* sc_wrap registers */
#define SC_WRAP_DVFS_SC_LOW_POWER_CTRL    0x538

/* sc_dsu_wrap fss registers */
#define SC_DSU_WRAP_SC_FSS_0          0x00
#define SC_DSU_WRAP_SC_FSS_1          0x04
#define SC_DSU_WRAP_SC_FSS_2          0x08
#define SC_DSU_WRAP_SC_FSS_3          0x0c
#define SC_DSU_WRAP_SC_FSS_4          0x10
#define SC_DSU_WRAP_SC_FSS_5          0x14
#define SC_DSU_WRAP_SC_FSS_STAT_0     0x18
#define SC_DSU_WRAP_SC_FSS_STAT_1     0x1c
#define SC_DSU_WRAP_SC_FSS_STAT_2     0x20
#define SC_DSU_WRAP_SC_FSS_STAT_3     0x24

struct fss_device;

struct fss_desc {
	int (*get_calibration_data)(struct fss_device *fss, struct fss_calibration_data *data);

	unsigned int sensor_bitmask;
	unsigned int st0_cal_done_mask;
	unsigned int ctrl0_val_prepare;
	unsigned int ctrl0_val_enable;
	unsigned int ctrl1_val_fss_scan;
	unsigned int dvfs_sc_low_power_ctrl_en : 1;

	unsigned int dsu_sensor_bitmask;
};

struct fss_device {
	struct device *dev;
	void *base;
	void *dsu_base;
	struct regmap *sc_wrap;
	const struct fss_desc *desc;
	struct mutex lock;
};

struct fss_control {
	struct fss_device *fss;
};

static void sc_wrap_reg_write(struct fss_device *fss, int offset, unsigned int val)
{
	regmap_write(fss->sc_wrap, offset, val);
}

static unsigned int fss_reg_read(struct fss_device *fss, int offset)
{
	return readl(fss->base + offset);
}

static void fss_reg_write(struct fss_device *fss, int offset, u32 val)
{
	writel(val, fss->base + offset);
}

static int fss_wait_cal_done(struct fss_device *fss, unsigned int mask)
{
	unsigned int cal_done;

	return readl_poll_timeout_atomic(fss->base + SC_WRAP_DVFS_FSS_ST0,
		cal_done, (cal_done & mask) == mask, 0, 100);
}

static unsigned int dsu_fss_reg_read(struct fss_device *fss, int offset)
{
	return readl(fss->dsu_base + offset);
}

static void dsu_fss_reg_write(struct fss_device *fss, int offset, u32 val)
{
	writel(val, fss->dsu_base + offset);
}

static int dsu_fss_wait_cal_done(struct fss_device *fss, unsigned int mask)
{
	unsigned int cal_done;

	return readl_poll_timeout_atomic(fss->dsu_base + SC_DSU_WRAP_SC_FSS_STAT_2,
		cal_done, (cal_done & mask) == mask, 0, 100);
}

static int fss_get_calibration_data(struct fss_device *fss, struct fss_calibration_data *data)
{
	const struct fss_desc *desc = fss->desc;
	int ret;

	if (desc->dvfs_sc_low_power_ctrl_en)
		sc_wrap_reg_write(fss, SC_WRAP_DVFS_SC_LOW_POWER_CTRL, 0x00000f0f);

	fss_reg_write(fss, SC_WRAP_DVFS_FSS_CTRL0, 0x0);
	fss_reg_write(fss, SC_WRAP_DVFS_FSS_CTRL0, desc->ctrl0_val_prepare);
	fss_reg_write(fss, SC_WRAP_DVFS_FSS_CTRL1, desc->ctrl1_val_fss_scan);
	mdelay(1);

	fss_reg_write(fss, SC_WRAP_DVFS_FSS_CTRL0, desc->ctrl0_val_enable);
	mdelay(1);

	ret = fss_wait_cal_done(fss, fss->desc->st0_cal_done_mask);
	if (ret)
		return ret;

	mdelay(10);

	data->sensor_bitmask = desc->sensor_bitmask;
	data->cdl0 = fss_reg_read(fss, SC_WRAP_DVFS_FSS_ST1);
	data->cdl1_min = fss_reg_read(fss, SC_WRAP_DVFS_FSS_ST3);
	data->result = data->cdl1_min;

	fss_reg_write(fss, SC_WRAP_DVFS_FSS_CTRL0, desc->ctrl0_val_prepare);
	fss_reg_write(fss, SC_WRAP_DVFS_FSS_CTRL0, 0);

	return 0;
}

static int fss_v4_get_calibration_data(struct fss_device *fss, struct fss_calibration_data *data)
{
	const struct fss_desc *desc = fss->desc;
	int ret;

	fss_reg_write(fss, SC_WRAP_DVFS_FSS_CTRL0, 0x0);
	dsu_fss_reg_write(fss, SC_DSU_WRAP_SC_FSS_0, 0x0);

	fss_reg_write(fss, SC_WRAP_DVFS_FSS_CTRL0, 0x03000000);
	dsu_fss_reg_write(fss, SC_DSU_WRAP_SC_FSS_0, 0x0000000f);

	fss_reg_write(fss, SC_WRAP_DVFS_FSS_CTRL0, 0x03000300);
	dsu_fss_reg_write(fss, SC_DSU_WRAP_SC_FSS_0, 0xf000000f);
	mdelay(1);

	ret = fss_wait_cal_done(fss, 0x00030000);
	if (ret)
		return ret;
	ret = dsu_fss_wait_cal_done(fss, 0x000f0000);
	if (ret)
		return ret;

	mdelay(10);

	data->sensor_bitmask = desc->sensor_bitmask;
	data->cdl0 = fss_reg_read(fss, SC_WRAP_DVFS_FSS_ST1);
	data->cdl1_min = fss_reg_read(fss, SC_WRAP_DVFS_FSS_ST3);
	data->result = data->cdl0;

	data->dsu_sensor_bitmask = desc->dsu_sensor_bitmask;
	data->dsu_cdl0 = dsu_fss_reg_read(fss, SC_DSU_WRAP_SC_FSS_STAT_2) & 0xffff;
	data->dsu_cdl1_min = dsu_fss_reg_read(fss, SC_DSU_WRAP_SC_FSS_STAT_3);
	data->dsu_result = data->dsu_cdl0;

	fss_reg_write(fss, SC_WRAP_DVFS_FSS_CTRL0, 0x03000000);
	dsu_fss_reg_write(fss, SC_DSU_WRAP_SC_FSS_0, 0x0000000f);

	fss_reg_write(fss, SC_WRAP_DVFS_FSS_CTRL0, 0x0);
	dsu_fss_reg_write(fss, SC_DSU_WRAP_SC_FSS_0, 0x0);

	return 0;
}

static int fss_get_verion(struct fss_device *fss)
{
	return fss->desc->get_calibration_data == fss_v4_get_calibration_data ? 4 : 0;
}

static int fss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *parent;
	struct fss_device *fss;

	fss = devm_kzalloc(dev, sizeof(*fss), GFP_KERNEL);
	if (!fss)
		return -ENOMEM;
	fss->dev = dev;

	fss->desc = of_device_get_match_data(dev);
	if (!fss->desc)
		return -EINVAL;

	fss->base = devm_platform_ioremap_resource(pdev, 0);
	if (!fss->base)
		return -ENOMEM;

	if (fss_get_verion(fss) == 4) {
		fss->dsu_base = devm_platform_ioremap_resource(pdev, 1);
		if (!fss->dsu_base)
			return -ENOMEM;
	}

	parent = of_get_parent(np);
	fss->sc_wrap = syscon_node_to_regmap(parent);
	of_node_put(parent);

	platform_set_drvdata(pdev, fss);

	return 0;
}

static int fss_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct fss_desc rtd1319_desc = {
	.get_calibration_data = fss_get_calibration_data,
	.sensor_bitmask     = 0x1f,
	.st0_cal_done_mask  = 0x1f000000,
	.ctrl0_val_prepare  = 0x1f000000,
	.ctrl0_val_enable   = 0x1f1f0000,
	.ctrl1_val_fss_scan = 0x00077777,
};

static const struct fss_desc rtd1619b_desc = {
	.get_calibration_data = fss_get_calibration_data,
	.sensor_bitmask     = 0x3f,
	.st0_cal_done_mask  = 0x3f000000,
	.ctrl0_val_prepare  = 0x3f000000,
	.ctrl0_val_enable   = 0x3f3f0000,
	.ctrl1_val_fss_scan = 0x00777777,
	.dvfs_sc_low_power_ctrl_en = 1,
};

static const struct fss_desc rtd1319d_desc = {
	.get_calibration_data = fss_v4_get_calibration_data,
	.sensor_bitmask     = 0x3,
	.dsu_sensor_bitmask = 0xf,
};

static const struct of_device_id fss_match[] = {
	{ .compatible = "realtek,rtd1319-fss",  .data = &rtd1319_desc,  },
	{ .compatible = "realtek,rtd1619b-fss", .data = &rtd1619b_desc, },
	{ .compatible = "realtek,rtd1319d-fss",  .data = &rtd1319d_desc,  },
	{}
};
MODULE_DEVICE_TABLE(of, fss_match);

static struct platform_driver fss_driver = {
	.probe  = fss_probe,
	.remove = fss_remove,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-fss",
		.of_match_table = of_match_ptr(fss_match),
	},
};
module_platform_driver(fss_driver);

MODULE_DESCRIPTION("Realtek FSS driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-fss");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");

struct fss_control *of_fss_control_get(struct device_node *np)
{
	struct fss_control *ctl;
	struct fss_device *fss;
	struct device_node *fss_np;
	struct platform_device *pdev;

	fss_np = of_parse_phandle(np, "realtek,fss", 0);
	if (!fss_np)
		return ERR_PTR(-EINVAL);

	pdev = of_find_device_by_node(fss_np);
	of_node_put(fss_np);
	if (!pdev)
		return ERR_PTR(-ENODEV);

	fss = platform_get_drvdata(pdev);
	if (!fss)
		return ERR_PTR(-EPROBE_DEFER);

	ctl = kzalloc(sizeof(*ctl), GFP_KERNEL);
	if (!ctl)
		return ERR_PTR(-ENOMEM);

	ctl->fss = fss;
	get_device(&pdev->dev);
	return ctl;
}
EXPORT_SYMBOL_GPL(of_fss_control_get);

void fss_control_put(struct fss_control *ctl)
{
	if (!ctl)
		return;

	put_device(ctl->fss->dev);
	kfree(ctl);
}
EXPORT_SYMBOL_GPL(fss_control_put);

int fss_control_get_calibration_data(struct fss_control *ctl, struct fss_calibration_data *data)
{
	int ret;
	const struct fss_desc *desc = ctl->fss->desc;

	mutex_lock(&ctl->fss->lock);
	ret = desc->get_calibration_data(ctl->fss, data);
	mutex_unlock(&ctl->fss->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(fss_control_get_calibration_data);
