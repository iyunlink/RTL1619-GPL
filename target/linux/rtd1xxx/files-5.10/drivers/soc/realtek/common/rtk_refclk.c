// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/atomic.h>

#define MIS_CLK90K_CTRL     0x0
#define MIS_SCPU_CLK90K_LO  0x8
#define MIS_SCPU_CLK90K_HI  0xC

struct refclk_device {
	void *base;
	struct device *dev;
};

static struct platform_driver refclk_driver;

static int refclk_reg_read(struct refclk_device *refclk, int offset, u32 *val)
{
	*val = readl(refclk->base + offset);
	return 0;
}

static int refclk_reg_write(struct refclk_device *refclk, int offset, u32 val)
{
	writel(val, refclk->base + offset);
	return 0;
}

/**
 * of_refclk_get - lookup and obtain a refclk device.
 * @np: device node to request
 * @index: index of the refclk
 *
 * Returns the refclk device on success, and returns -EINVAL if failed to
 * get the refclk, and returns -EPROBE_DERFER if refclk device is not ready.
 */
struct refclk_device *of_refclk_get(struct device_node *np, int index)
{
	struct refclk_device *refclk = ERR_PTR(-EINVAL);
	struct device_node *refclk_np;
	struct platform_device *pdev;

	refclk_np = of_parse_phandle(np, "realtek,refclk", index);

	if (!refclk_np)
		return refclk;

	pdev = of_find_device_by_node(refclk_np);
	of_node_put(refclk_np);
	if (!pdev)
		return refclk;

	refclk = platform_get_drvdata(pdev);
	if (!refclk)
		return ERR_PTR(-EPROBE_DEFER);

	get_device(&pdev->dev);
	return refclk;
}
EXPORT_SYMBOL_GPL(of_refclk_get);

struct platform_match_data {
	struct device_driver *drv;
	const char *name;
};

static int platform_match(struct device *dev, const void *d)
{
	const struct platform_match_data *data = d;

	return dev->driver == data->drv && !strcmp(dev_name(dev), data->name);
}

/**
 * refclk_get_by_name - lookup and obtain a refclk device by name
 * name: device name
 *
 * Returns the refclk device on success, and returns -EINVAL if failed to
 * get the refclk, and returns -EPROBE_DERFER if refclk device is not ready.
 */
struct refclk_device *refclk_get_by_name(const char *name)
{
	struct refclk_device *refclk;
	struct platform_match_data data = {
		.drv = &refclk_driver.driver,
		.name = name,
	};
	struct device *dev = NULL;

	dev = bus_find_device(&platform_bus_type, NULL, &data, platform_match);
	if (!dev)
		return ERR_PTR(-EINVAL);

	refclk = platform_get_drvdata(to_platform_device(dev));
	if (!refclk)
		return ERR_PTR(-EPROBE_DEFER);

	get_device(dev);
	return refclk;
}
EXPORT_SYMBOL_GPL(refclk_get_by_name);

/**
 * refclk_put - free the refclk device
 * @refclk: refclk device
 */
void refclk_put(struct refclk_device *refclk)
{
	put_device(refclk->dev);
}
EXPORT_SYMBOL_GPL(refclk_put);

static void refclk_set_enable(struct refclk_device *refclk, u32 val)
{
	refclk_reg_write(refclk, MIS_CLK90K_CTRL, val);
}

static u32 refclk_get_enable(struct refclk_device *refclk)
{
	u32 val;

	refclk_reg_read(refclk, MIS_CLK90K_CTRL, &val);
	return val;
}

/**
 * refclk_get_counter - get the counter value
 * @refclk: refclk device
 *
 * Return the counter value of the refclk device.
 */
u64 refclk_get_counter(struct refclk_device *refclk)
{
	u32 lo;
	u32 hi0, hi1;

	if (!refclk)
		return 0;
	if (WARN_ON(IS_ERR(refclk)))
		return (u64)-EINVAL;

	refclk_reg_read(refclk, MIS_SCPU_CLK90K_HI, &hi0);
	refclk_reg_read(refclk, MIS_SCPU_CLK90K_LO, &lo);
	refclk_reg_read(refclk, MIS_SCPU_CLK90K_HI, &hi1);
	if (hi0 != hi1)
		refclk_reg_read(refclk, MIS_SCPU_CLK90K_LO, &lo);

	return ((u64)hi1 << 32) | lo;
}
EXPORT_SYMBOL_GPL(refclk_get_counter);

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct refclk_device *refclk = dev_get_drvdata(dev);
	int ret;
	u32 val;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;
	if (val) {
		refclk_set_enable(refclk, 1);
	} else {
		refclk_set_enable(refclk, 0);
	}

	return count;
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct refclk_device *refclk = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", refclk_get_enable(refclk));
}
DEVICE_ATTR_RW(enable);

static ssize_t counter_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct refclk_device *refclk = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%llu\n", refclk_get_counter(refclk));
}
DEVICE_ATTR_RO(counter);

static struct attribute *refclk_attrs[] = {
	&dev_attr_counter.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group refclk_attr_group = {
	.name = "refclk",
	.attrs = refclk_attrs,
};

static int refclk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct refclk_device *refclk;
	int ret;

	refclk = devm_kzalloc(dev, sizeof(*refclk), GFP_KERNEL);
	if (!refclk)
		return -ENOMEM;
	refclk->dev = dev;

	refclk->base = of_iomap(np, 0);
	if (!refclk->base)
		return -ENOMEM;

	ret = sysfs_create_group(&dev->kobj, &refclk_attr_group);
	if (ret) {
		dev_err(dev, "failed to create sysfs group: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, refclk);
	return 0;
}

static int refclk_remove(struct platform_device *pdev)
{
	struct refclk_device *refclk = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	sysfs_remove_group(&pdev->dev.kobj, &refclk_attr_group);
	iounmap(refclk->base);
	return 0;
}

static const struct of_device_id refclk_match[] = {
	{.compatible = "realtek,refclk"},
	{}
};
MODULE_DEVICE_TABLE(of, refclk_match);

static struct platform_driver refclk_driver = {
	.probe  = refclk_probe,
	.remove = refclk_remove,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-refclk",
		.of_match_table = of_match_ptr(refclk_match),
	},
};
module_platform_driver(refclk_driver);

MODULE_DESCRIPTION("Realtek Refclk driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-refclk");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
