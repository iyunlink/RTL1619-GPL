// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek System-on-Chip info
 *
 * Copyright (c) 2017-2019 Andreas Färber
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>
#include <linux/nvmem-consumer.h>

#define REG_CHIP_ID	0x0
#define REG_CHIP_REV	0x4

static int of_rtd_soc_read_efuse(struct device_node *np, const char *name,
				 unsigned char *val, size_t size)
{
	struct nvmem_cell *cell;
	char *buf;
	size_t buf_size;
	int ret = 0;

	cell = of_nvmem_cell_get(np, name);
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	buf = nvmem_cell_read(cell, &buf_size);
	nvmem_cell_put(cell);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	if (buf_size > size)
		ret = -E2BIG;
	else
		memcpy(val, buf, size);
	kfree(buf);
	return ret;
}

static int of_get_bond_id(struct device_node *np, u32 *bond)
{
	return of_rtd_soc_read_efuse(np, "bond_id", (u8 *)bond, sizeof(*bond));
}

static int of_get_package_id(struct device_node *np, u32 *package)
{
	return of_rtd_soc_read_efuse(np, "package_id", (u8 *)package,
		sizeof(*package));
}

struct rtd_soc_revision {
	const char *name;
	u32 chip_rev;
};

static const struct rtd_soc_revision rtd1195_revisions[] = {
	{ "A", 0x00000000 },
	{ "B", 0x00010000 },
	{ "C", 0x00020000 },
	{ "D", 0x00030000 },
	{ }
};

static const struct rtd_soc_revision rtd1295_revisions[] = {
	{ "A00", 0x00000000 },
	{ "A01", 0x00010000 },
	{ "B00", 0x00020000 },
	{ "B01", 0x00030000 },
	{ }
};

static const struct rtd_soc_revision rtd1395_revisions[] = {
	{ "A00", 0x00000000},
	{ "A01", 0x00010000},
	{ "A02", 0x00020000},
	{ }
};

static const struct rtd_soc_revision rtd1619_revisions[] = {
	{ "A00", 0x00000000},
	{ "A01", 0x00010000},
	{ }
};

static const struct rtd_soc_revision rtd1319_revisions[] = {
	{ "A00", 0x00000000},
	{ "B00", 0x00010000},
	{ "B01", 0x00020000},
	{ "B02", 0x00030000},
	{ }
};

static const struct rtd_soc_revision rtd161xb_revisions[] = {
	{ "A00", 0x00000000},
	{ }
};

static const struct rtd_soc_revision rtd1312c_revisions[] = {
	{ "A00", 0x00000000},
	{ }
};

static const struct rtd_soc_revision rtd1319d_revisions[] = {
	{ "A00", 0x00000000},
	{ }
};

struct rtd_soc {
	u32 chip_id;
	const char *(*get_name)(struct device *dev, const struct rtd_soc *s);
	const char *(*get_chip_type)(struct device *dev, const struct rtd_soc *s);
	const struct rtd_soc_revision *revisions;
	const char *codename;
};

static const char *rtd119x_name(struct device *dev, const struct rtd_soc *s)
{
	return "RTD1195";
}

static const char *rtd129x_name(struct device *dev, const struct rtd_soc *s)
{
	void __iomem *base;
	u32 package_id;
	int ret;

	ret = of_get_package_id(dev->of_node, &package_id);
	if (!ret) {
		dev_info(dev, "package_id: 0x%08x\n", package_id);
		if (package_id == 0x1)
			return "RTD1294";
	}

	base = of_iomap(dev->of_node, 1);
	if (base) {
		u32 chipinfo1 = readl_relaxed(base);
		iounmap(base);
		dev_info(dev, "chipinfo1: 0x%08x\n", chipinfo1);
		if (chipinfo1 & BIT(11)) {
			if (chipinfo1 & BIT(4))
				return "RTD1293";

			return "RTD1296";
		}
	}

	return "RTD1295";
}

static const char *rtd139x_name(struct device *dev, const struct rtd_soc *s)
{
	void __iomem *base;

	base = of_iomap(dev->of_node, 1);
	if (base) {
		u32 chipinfo1 = readl_relaxed(base);
		iounmap(base);
		dev_info(dev, "chipinfo1: 0x%08x\n", chipinfo1);
		if (chipinfo1 & BIT(12))
			return "RTD1392";
	}

	return "RTD1395";
}

static const char *rtd161x_name(struct device *dev, const struct rtd_soc *s)
{
	return "RTD1619";
}

static const char *rtd131x_name(struct device *dev, const struct rtd_soc *s)
{
	u32 bond_id;
	int ret;

#define OTP_CHIP_MASK 0x000F0000
#define RTD1319 0x00000000
#define RTD1317 0x00010000
#define RTD1315 0x00020000

	ret = of_get_bond_id(dev->of_node, &bond_id);
	if (!ret) {
		dev_info(dev, "bond_id: 0x%08x\n", bond_id);
		switch (bond_id & OTP_CHIP_MASK) {
		case (RTD1319):
			break;
		case (RTD1317):
			return "RTD1317";
		case (RTD1315):
			return "RTD1315";
		default:
			pr_err("%s: Not define chip id 0x%x\n",
				    __func__, bond_id);
		}
	}

	return "RTD1319";
}

static const char *rtd131x_chip_type(struct device *dev, const struct rtd_soc *s)
{
	u32 bond_id;
	int ret;

#define CHIP_TYPE_1319_EI (0x00000000)
#define CHIP_TYPE_1319_ES (0x00300001)
#define CHIP_TYPE_1319_PB (0x1FB00007)
#define CHIP_TYPE_1319_DV (0x1B30000F)
#define CHIP_TYPE_1319_VS (0x1B30001B)
#define CHIP_TYPE_1311_EI (0x0000F000)
#define CHIP_TYPE_1311_ES (0x0030F001)
#define CHIP_TYPE_1311_PB (0x1FB07007)
#define CHIP_TYPE_1311_DV (0x1B30700F)
#define CHIP_TYPE_1311_VS (0x1B307013)

	ret = of_get_bond_id(dev->of_node, &bond_id);
	if (!ret) {
		dev_info(dev, "bond_id: 0x%08x\n", bond_id);
		switch (bond_id) {
		case (CHIP_TYPE_1319_EI):
			return "1319_EI";
		case (CHIP_TYPE_1319_ES):
			return "1319_ES";
		case (CHIP_TYPE_1319_PB):
			return "1319_PB";
		case (CHIP_TYPE_1319_DV):
			return "1319_DV";
		case (CHIP_TYPE_1319_VS):
			return "1319_VS";
		case (CHIP_TYPE_1311_EI):
			return "1311_EI";
		case (CHIP_TYPE_1311_ES):
			return "1311_ES";
		case (CHIP_TYPE_1311_PB):
			return "1311_PB";
		case (CHIP_TYPE_1311_DV):
			return "1311_DV";
		case (CHIP_TYPE_1311_VS):
			return "1311_VS";
		default:
			pr_err("%s: Not define chip type 0x%x\n",
				    __func__, bond_id);
		}
	}

	return "unknown";
}

static const char *rtd161xb_name(struct device *dev, const struct rtd_soc *s)
{
	u32 bond_id;
	int ret;

#define OTP_STARK_CHIP_MASK 0x00007000
#define RTD1619B 0x00000000
#define RTD1315C 0x00002000

	ret = of_get_bond_id(dev->of_node, &bond_id);
	if (!ret) {
		dev_info(dev, "bond_id: 0x%08x\n", bond_id);
		switch (bond_id & OTP_STARK_CHIP_MASK) {
		case (RTD1619B):
			break;
		case (RTD1315C):
			return "RTD1315C";
		default:
			pr_err("%s: Not define chip id 0x%x\n",
				    __func__, bond_id);
		}
	}

	return "RTD1619B";
}

static const char *rtd161xb_chip_type(struct device *dev, const struct rtd_soc *s)
{
	u32 bond_id;
	int ret;

#define CHIP_TYPE_1619B_EI (0x00000000)

	ret = of_get_bond_id(dev->of_node, &bond_id);
	if (!ret) {
		dev_info(dev, "bond_id: 0x%08x\n", bond_id);
		switch (bond_id) {
		case (CHIP_TYPE_1619B_EI):
			return "1619B_EI";
		default:
			pr_err("%s: Not define chip type 0x%x\n",
				    __func__, bond_id);
		}
	}

	return "unknown";
}

static const char *rtd1312c_name(struct device *dev, const struct rtd_soc *s)
{
	return "RTD1312C";
}

static const char *rtd1319d_name(struct device *dev, const struct rtd_soc *s)
{
	u32 bond_id;
	int ret;

#define OTP_PARKER_CHIP_MASK 0xC0000000
#define RTD1319D 0x00000000
#define RTD1315D 0x40000000

	ret = of_get_bond_id(dev->of_node, &bond_id);
	if (!ret) {
		dev_info(dev, "bond_id: 0x%08x\n", bond_id);
		switch (bond_id & OTP_PARKER_CHIP_MASK) {
		case (RTD1319D):
			break;
		case (RTD1315D):
			return "RTD1315D";
		default:
			pr_err("%s: Not define chip id 0x%x\n",
				    __func__, bond_id);
		}
	}

	return "RTD1319D";
}

static const struct rtd_soc rtd_soc_families[] = {
	{ 0x00006329, rtd119x_name, NULL, rtd1195_revisions, "Phoenix" },
	{ 0x00006421, rtd129x_name, NULL, rtd1295_revisions, "Kylin" },
	{ 0x00006481, rtd139x_name, NULL, rtd1395_revisions, "Hercules" },
	{ 0x00006525, rtd161x_name, NULL, rtd1619_revisions, "Thor" },
	{ 0x00006570, rtd131x_name, rtd131x_chip_type, rtd1319_revisions, "Hank" },
	{ 0x00006698, rtd161xb_name, rtd161xb_chip_type, rtd161xb_revisions, "Stark" },
	{ 0x00006820, rtd1312c_name, NULL, rtd1312c_revisions, "Groot" },
	{ 0x00006756, rtd1319d_name, NULL, rtd1319d_revisions, "Parker" },
};

static const struct rtd_soc *rtd_soc_by_chip_id(u32 chip_id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rtd_soc_families); i++) {
		const struct rtd_soc *family = &rtd_soc_families[i];

		if (family->chip_id == chip_id)
			return family;
	}
	return NULL;
}

static const char *rtd_soc_rev(const struct rtd_soc *family, u32 chip_rev)
{
	if (family) {
		const struct rtd_soc_revision *rev = family->revisions;

		while (rev && rev->name) {
			if (rev->chip_rev == chip_rev) {
				return rev->name;
			}
			rev++;
		}
	}
	return "unknown";
}

struct custom_device_attribute {
	const char *chip_type;
};

static ssize_t custom_info_get(struct device *dev,
			    struct device_attribute *attr,
			    char *buf);

static DEVICE_ATTR(chip_type,  S_IRUGO, custom_info_get,  NULL);

static umode_t custom_attribute_mode(struct kobject *kobj,
				struct attribute *attr,
				int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct soc_device *soc_dev = container_of(dev, struct soc_device, dev);

	if ((attr == &dev_attr_chip_type.attr)
	    && (soc_dev->attr->data)) {
		struct custom_device_attribute *custom_dev_attr;

		custom_dev_attr = (struct custom_device_attribute *)
			(soc_dev->attr->data);
		if (custom_dev_attr->chip_type)
			return attr->mode;
	}
	/* Unknown or unfilled attribute. */
	return 0;
}


static ssize_t custom_info_get(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct soc_device *soc_dev = container_of(dev, struct soc_device, dev);

	if (attr == &dev_attr_chip_type &&
		soc_dev->attr->data) {
		struct custom_device_attribute *custom_dev_attr;

		custom_dev_attr = (struct custom_device_attribute *)
			(soc_dev->attr->data);
		if (custom_dev_attr->chip_type)
			return sprintf(buf, "%s\n", custom_dev_attr->chip_type);
	}

	return -EINVAL;
}

static struct attribute *custom_attr[] = {
	&dev_attr_chip_type.attr,
	NULL,
};

static const struct attribute_group custom_attr_group = {
	.attrs = custom_attr,
	.is_visible = custom_attribute_mode,
};

static int rtd_soc_probe(struct platform_device *pdev)
{
	const struct rtd_soc *s;
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	struct device_node *node;
	struct custom_device_attribute *custom_dev_attr;
	void __iomem *base;
	u32 chip_id, chip_rev;

	base = of_iomap(pdev->dev.of_node, 0);
	if (!base)
		return -ENODEV;

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENOMEM;

	chip_id  = readl_relaxed(base + REG_CHIP_ID);
	chip_rev = readl_relaxed(base + REG_CHIP_REV);

	node = of_find_node_by_path("/");
	of_property_read_string(node, "model", &soc_dev_attr->machine);
	of_node_put(node);

	s = rtd_soc_by_chip_id(chip_id);

	soc_dev_attr->family = kasprintf(GFP_KERNEL, "Realtek %s",
		(s && s->codename) ? s->codename : "Digital Home Center");

	if (likely(s && s->get_name))
		soc_dev_attr->soc_id = s->get_name(&pdev->dev, s);
	else
		soc_dev_attr->soc_id = "unknown";

	soc_dev_attr->revision = rtd_soc_rev(s, chip_rev);

	custom_dev_attr = kzalloc(sizeof(*custom_dev_attr), GFP_KERNEL);
	if (!custom_dev_attr)
		return -ENOMEM;

	if (likely(s && s->get_chip_type))
		custom_dev_attr->chip_type = s->get_chip_type(&pdev->dev, s);
	else
		custom_dev_attr->chip_type = NULL;

	soc_dev_attr->data = (void *)custom_dev_attr;

	soc_dev_attr->custom_attr_group = &custom_attr_group;

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		kfree(soc_dev_attr->family);
		kfree(soc_dev_attr);
		return PTR_ERR(soc_dev);
	}

	platform_set_drvdata(pdev, soc_dev);

	pr_info("%s %s (0x%08x) rev %s (0x%08x) detected\n",
		soc_dev_attr->family, soc_dev_attr->soc_id, chip_id,
		soc_dev_attr->revision, chip_rev);

	return 0;
}

static int rtd_soc_remove(struct platform_device *pdev)
{
	struct soc_device *soc_dev = platform_get_drvdata(pdev);

	soc_device_unregister(soc_dev);

	return 0;
}

static const struct of_device_id rtd_soc_dt_ids[] = {
	 { .compatible = "realtek,soc-chip" },
	 { }
};

static struct platform_driver rtd_soc_driver = {
	.probe = rtd_soc_probe,
	.remove = rtd_soc_remove,
	.driver = {
		.name = "rtk-soc",
		.of_match_table	= rtd_soc_dt_ids,
	},
};

static int __init rtd_soc_driver_init(void)
{
	return platform_driver_register(&(rtd_soc_driver));
}
subsys_initcall_sync(rtd_soc_driver_init);

static void __exit rtd_soc_driver_exit(void)
{
	platform_driver_unregister(&(rtd_soc_driver));
}

MODULE_DESCRIPTION("Realtek SoC identification");
MODULE_LICENSE("GPL");
