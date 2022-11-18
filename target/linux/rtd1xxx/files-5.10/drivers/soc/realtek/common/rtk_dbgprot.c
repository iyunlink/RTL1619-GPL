// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek Debug Proptection Driver
 *
 * Copyright (C) 2021 Realtek Semiconductor Corp.
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define DP_NX_MRK1_CN   0x290

struct rtk_dbgprot_data {
	struct device         *dev;
	void                  *base;
};

static ssize_t checknum_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct rtk_dbgprot_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%08x\n", readl(data->base + DP_NX_MRK1_CN));
}
DEVICE_ATTR_RO(checknum);

static struct attribute *rtk_dbgprot_attrs[] = {
	&dev_attr_checknum.attr,
	NULL
};

static struct attribute_group rtk_dbgprot_attr_group = {
	.name = "dbgprot",
	.attrs = rtk_dbgprot_attrs,
};

static int rtk_dbgprot_probe(struct platform_device *pdev)
{
	struct rtk_dbgprot_data *data;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->base = of_iomap(np, 0);
	if (!data->base)
		return -ENOMEM;

	ret = sysfs_create_group(&dev->kobj, &rtk_dbgprot_attr_group);
	if (ret) {
		dev_err(dev, "failed to create sysfs group: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, data);
	return 0;
}

static int rtk_dbgprot_remove(struct platform_device *pdev)
{
	struct rtk_dbgprot_data *data = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	sysfs_remove_group(&pdev->dev.kobj, &rtk_dbgprot_attr_group);
	iounmap(data->base);
	return 0;
}

static const struct of_device_id rtk_dbgprot_ids[] = {
	{ .compatible = "realtek,dbgprot" },
	{}
};
MODULE_DEVICE_TABLE(of, rtk_dbgprot_ids);

static struct platform_driver rtk_dbgprot_driver = {
	.probe = rtk_dbgprot_probe,
	.remove = rtk_dbgprot_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "rtk-dbgprot",
		.of_match_table = rtk_dbgprot_ids,

	},
};
module_platform_driver(rtk_dbgprot_driver);

MODULE_DESCRIPTION("Realtek Debug Proptection Driver");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");
