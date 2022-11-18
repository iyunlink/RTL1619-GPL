// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2019 Realtek Semiconductor Corp.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/ktime.h>
#include <linux/regmap.h>
#include <linux/nvmem-consumer.h>
#include <linux/bitops.h>


#include <soc/realtek/rtk_sb2_dbg.h>
#include "rtk_sb2.h"

#define SB2_INV_INTEN              0x0
#define SB2_INV_INTSTAT            0x4
#define SB2_INV_ADDR               0x8
#define SB2_DEBUG_REG              0xC

#define SB2_INV_INTEN_VCIVAIRQ_EN                                     0x00000400
#define SB2_INV_INTEN_PCIVAIRQ2_EN                                    0x00000100
#define SB2_INV_INTEN_SWCIVAIRQ_EN                                    0x00000040
#define SB2_INV_INTEN_ACIVAIRQ_EN                                     0x00000008
#define SB2_INV_INTEN_PCIVAIRQ_EN                                     0x00000004
#define SB2_INV_INTEN_SCIVAIRQ_EN                                     0x00000002
#define SB2_INV_INTEN_WRITE_DATA                                      0x00000001
#define SB2_INV_INTEN_CLEAR_DATA                                      0x00000000
#define SB2_INV_INTEN_ENABLE_DEFAULT_CPUS \
	(SB2_INV_INTEN_SWCIVAIRQ_EN | SB2_INV_INTEN_SCIVAIRQ_EN | \
	 SB2_INV_INTEN_PCIVAIRQ_EN | SB2_INV_INTEN_PCIVAIRQ2_EN | \
	 SB2_INV_INTEN_WRITE_DATA)

#define SB2_INV_INTSTAT_VCIVA_INT                                     0x00000040
#define SB2_INV_INTSTAT_PCIVA2_INT                                    0x00000020
#define SB2_INV_INTSTAT_SWCIVA_INT                                    0x00000010
#define SB2_INV_INTSTAT_ACIVA_INT                                     0x00000008
#define SB2_INV_INTSTAT_PCIVA_INT                                     0x00000004
#define SB2_INV_INTSTAT_SCIVA_INT                                     0x00000002
#define SB2_INV_INTSTAT_WRITE_DATA                                    0x00000001
#define SB2_INV_INTSTAT_CLEAR_DATA                                    0x00000000

static const char *inv_cpu_str[] = {
	[SB2_INV_UNKNOWN] = "Unknown CPU",
	[SB2_INV_SCPU]     = "SCPU",
	[SB2_INV_PCPU]     = "PCPU",
	[SB2_INV_ACPU]     = "ACPU",
	[SB2_INV_SCPU_SWC] = "SCPU security world",
	[SB2_INV_PCPU_2]   = "PCPU_2",
	[SB2_INV_VCPU]     = "VCPU",
};

static const char *get_cpu_id(int id)
{
	if (id > SB2_INV_VCPU)
		id = 0;
	return inv_cpu_str[id];
}

static void sb2_inv_print_inv_event(struct sb2_inv_event_data *d)
{
	pr_err("sb2 get int 0x%08x from SB2_INV_INTSTAT\n", d->raw_ints);
	pr_err("\033[0;31mInvalid access issued by %s\033[m\n", get_cpu_id(d->inv_cpu));
	pr_err("\033[0;31mInvalid address is 0x%08x\033[m\n", d->addr);
	pr_err("Timeout threshold(0x%08x)\n", d->timeout_th);
}

static irqreturn_t sb2_inv_int_handler(int irq, void *id)
{
	struct platform_device *pdev = id;
	struct sb2_common_data *sb2 = platform_get_drvdata(pdev);
	u32 ints;

	sb2_reg_read(sb2, SB2_INV_INTSTAT, &ints);
	if (ints) {
		struct sb2_inv_event_data d;

		/* clear ints */
		sb2_reg_write(sb2, SB2_INV_INTSTAT, 0xfffffffe);

		d.raw_ints = ints;
		sb2_reg_read(sb2, SB2_INV_ADDR, &d.addr);
		sb2_reg_read(sb2, SB2_DEBUG_REG, &d.timeout_th);
		d.inv_cpu = ffs(ints) - 1;
		sb2_inv_print_inv_event(&d);

		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static void sb2_inv_enable_inv_int(struct sb2_common_data *sb2)
{
	sb2_reg_write(sb2, SB2_INV_INTSTAT, 0xfffffffe);
	sb2_reg_write(sb2, SB2_INV_INTEN, SB2_INV_INTEN_ENABLE_DEFAULT_CPUS);
}

static int sb2_inv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sb2_common_data *sb2;
	struct device_node *np = dev->of_node;
	struct resource res;
	int ret;

	sb2 = devm_kzalloc(dev, sizeof(*sb2), GFP_KERNEL);
	if (!sb2)
		return -ENOMEM;
	sb2->dev = dev;

	if (sb2_check_perm(sb2) != 0) {
		dev_err(dev, "no permission\n");
		return -ENODEV;
	}

	ret = of_address_to_resource(np, 0, &res);
	if (ret)
		return ret;

	sb2->base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!sb2->base)
		return -ENOMEM;

	platform_set_drvdata(pdev, sb2);

	sb2->irq = irq_of_parse_and_map(np, 0);
	if (!sb2->irq) {
		dev_err(dev, "failed to parse irq: %d\n", sb2->irq);
		return -ENXIO;
	}

	ret = devm_request_irq(dev, sb2->irq, sb2_inv_int_handler, IRQF_SHARED,
			       dev_name(dev), pdev);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		return -ENXIO;
	}

	sb2_inv_enable_inv_int(sb2);
	return 0;
}

static int sb2_inv_resume(struct device *dev)
{
	struct sb2_common_data *sb2 = dev_get_drvdata(dev);

	dev_info(dev, "enter %s\n", __func__);
	sb2_inv_enable_inv_int(sb2);
	dev_info(dev, "exit %s\n", __func__);
	return 0;
}

static struct dev_pm_ops sb2_inv_pm_ops = {
	.resume_noirq  = sb2_inv_resume,
};

static const struct of_device_id sb2_inv_match[] = {
	{.compatible = "realtek,sysbrg2-inv"},
	{},
};
MODULE_DEVICE_TABLE(of, sb2_inv_match);

static struct platform_driver sb2_inv_driver = {
	.probe  = sb2_inv_probe,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-sb2-inv",
		.pm             = &sb2_inv_pm_ops,
		.of_match_table = of_match_ptr(sb2_inv_match),
	},
};

static int __init rtk_sb2_init(void)
{
	return platform_driver_register(&sb2_inv_driver);
}
subsys_initcall_sync(rtk_sb2_init);

MODULE_DESCRIPTION("Realtek SB2 Invaild Access driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-sb2-inv");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
