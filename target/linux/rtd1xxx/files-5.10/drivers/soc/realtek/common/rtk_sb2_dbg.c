// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2019 Realtek Semiconductor Corp.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <soc/realtek/rtk_sb2_dbg.h>
#include "rtk_sb2.h"

#define SB2_DBG_START(i)              (0x0 + (i) * 4)
#define SB2_DBG_END(i)                (0x20 + (i) * 4)
#define SB2_DBG_CTRL(i)               (0x40 + (i) * 4)
#define SB2_DBG_ADDR_ACPU             (0x60)
#define SB2_DBG_ADDR_PCPU             (0x64)
#define SB2_DBG_ADDR_SCPU             (0x68)
#define SB2_DBG_ADDR_VCPU             (0x6C)
#define SB2_DBG_ADDR1                 (0x70)
#define SB2_DBG_INT                   (0x88)

#define SB2_DBG_CTRL_WRITE_EN7                                        0x00008000
#define SB2_DBG_CTRL_DBG_VCPU_CHK_EN                                  0x00004000
#define SB2_DBG_CTRL_WRITE_EN6                                        0x00002000
#define SB2_DBG_CTRL_DBG_ACPU_CHK_EN                                  0x00001000
#define SB2_DBG_CTRL_WRITE_EN5                                        0x00000800
#define SB2_DBG_CTRL_DBG_PCPU_CHK_EN                                  0x00000400
#define SB2_DBG_CTRL_WRITE_EN4                                        0x00000200
#define SB2_DBG_CTRL_DBG_SCPU_CHK_EN                                  0x00000100
#define SB2_DBG_CTRL_WRITE_EN3                                        0x00000080
#define SB2_DBG_CTRL_DBG_WR_CHK                                       0x00000060
#define SB2_DBG_CTRL_WRITE_EN2                                        0x00000010
#define SB2_DBG_CTRL_DBG_ID_CHK                                       0x0000000C
#define SB2_DBG_CTRL_WRITE_EN1                                        0x00000002
#define SB2_DBG_CTRL_DBG_EN                                           0x00000001

#define SB2_DBG_CTRL_DISABLE \
	(SB2_DBG_CTRL_WRITE_EN6 | \
	 SB2_DBG_CTRL_WRITE_EN4 | \
	 SB2_DBG_CTRL_WRITE_EN1)
#define SB2_DBG_CTRL_ENABLE_ACPU \
	(SB2_DBG_CTRL_WRITE_EN6 | SB2_DBG_CTRL_DBG_ACPU_CHK_EN | \
	 SB2_DBG_CTRL_WRITE_EN4 | \
	 SB2_DBG_CTRL_WRITE_EN1 | SB2_DBG_CTRL_DBG_EN)
#define SB2_DBG_CTRL_ENABLE_SCPU \
	(SB2_DBG_CTRL_WRITE_EN6 | \
	 SB2_DBG_CTRL_WRITE_EN4 | SB2_DBG_CTRL_DBG_SCPU_CHK_EN | \
	 SB2_DBG_CTRL_WRITE_EN1 | SB2_DBG_CTRL_DBG_EN)

#define SB2_DBG_ADDR1_VCPU_DBG_WRITE                                  0x00000200
#define SB2_DBG_ADDR1_VCPU_DBG_DACC                                   0x00000100
#define SB2_DBG_ADDR1_PCPU_DBG_WRITE                                  0x00000080
#define SB2_DBG_ADDR1_PCPU_DBG_DACC                                   0x00000040
#define SB2_DBG_ADDR1_ACPU_DBG_WRITE                                  0x00000020
#define SB2_DBG_ADDR1_ACPU_DBG_DACC                                   0x00000010
#define SB2_DBG_ADDR1_SCPU_DBG_WRITE                                  0x00000008
#define SB2_DBG_ADDR1_SCPU_DBG_DACC                                   0x00000004

#define SB2_DBG_INT_VCPU_INT                                          0x00010000
#define SB2_DBG_INT_VCPU_INT_EN                                       0x00008000
#define SB2_DBG_INT_VCPU_NEG_INT                                      0x00004000
#define SB2_DBG_INT_VCPU_NEG_INT_EN                                   0x00002000
#define SB2_DBG_INT_ACPU_INT                                          0x00001000
#define SB2_DBG_INT_PCPU_INT                                          0x00000800
#define SB2_DBG_INT_SCPU_INT                                          0x00000400
#define SB2_DBG_INT_ACPU_INT_EN                                       0x00000200
#define SB2_DBG_INT_PCPU_INT_EN                                       0x00000100
#define SB2_DBG_INT_SCPU_INT_EN                                       0x00000080
#define SB2_DBG_INT_ACPU_NEG_INT                                      0x00000040
#define SB2_DBG_INT_PCPU_NEG_INT                                      0x00000020
#define SB2_DBG_INT_SCPU_NEG_INT                                      0x00000010
#define SB2_DBG_INT_ACPU_NEG_INT_EN                                   0x00000008
#define SB2_DBG_INT_PCPU_NEG_INT_EN                                   0x00000004
#define SB2_DBG_INT_SCPU_NEG_INT_EN                                   0x00000002
#define SB2_DBG_INT_WRITE_DATA                                        0x00000001
#define SB2_DBG_INT_EN_DEFAULT \
	(SB2_DBG_INT_SCPU_INT_EN | SB2_DBG_INT_ACPU_INT_EN | \
	 SB2_DBG_INT_WRITE_DATA)
#define SB2_DBG_INT_INT_STATUS \
	(SB2_DBG_INT_SCPU_INT | SB2_DBG_INT_PCPU_INT | \
	 SB2_DBG_INT_ACPU_INT | SB2_DBG_INT_VCPU_INT)

static struct sb2_common_data *dbg_data;

int sb2_dbg_ready(void)
{
	return dbg_data != NULL;
}
EXPORT_SYMBOL_GPL(sb2_dbg_ready);

int sb2_dbg_disable(int i)
{
	struct sb2_common_data *sb2 = dbg_data;
	if (!sb2_dbg_ready())
		return -ENODEV;

	sb2_reg_write(sb2, SB2_DBG_CTRL(i), SB2_DBG_CTRL_DISABLE);
	return 0;
}
EXPORT_SYMBOL(sb2_dbg_disable);

static int __sb2_dbg_setup(int i, u32 start, u32 end, u32 ctrl)
{
	struct sb2_common_data *sb2 = dbg_data;
	if (!sb2_dbg_ready())
		return -ENODEV;

	/* disable this set first */
	sb2_dbg_disable(i);

	pr_info("%s: dbg%d addr %08x-%08x flag %08x\n", __func__,
		i, start, end, ctrl);
	sb2_reg_write(sb2, SB2_DBG_START(i), start);
	sb2_reg_write(sb2, SB2_DBG_END(i), end);
	sb2_reg_write(sb2, SB2_DBG_CTRL(i), ctrl);
	return 0;
}

int sb2_dbg_monitor_scpu(int i, u32 start, u32 end, u32 d_i, u32 r_w)
{
	unsigned int ctrl = SB2_DBG_CTRL_ENABLE_SCPU |
		 SB2_DBG_CTRL_WRITE_EN3 | r_w |
		 SB2_DBG_CTRL_WRITE_EN2 | d_i;

	return __sb2_dbg_setup(i, start, end, ctrl);
}
EXPORT_SYMBOL(sb2_dbg_monitor_scpu);

int sb2_dbg_monitor_acpu(int i, u32 start, u32 end, u32 d_i, u32 r_w)
{
	unsigned int ctrl = SB2_DBG_CTRL_ENABLE_ACPU |
		 SB2_DBG_CTRL_WRITE_EN3 | r_w |
		 SB2_DBG_CTRL_WRITE_EN2 | d_i;

	return __sb2_dbg_setup(i, start, end, ctrl);
}
EXPORT_SYMBOL(sb2_dbg_monitor_acpu);


static void sb2_dbg_handle_event(struct sb2_dbg_event_data *evd)
{
	pr_err("RAW_STATUS=%08x, SRC=%d, ADDR=%08x, RW=%d, DI=%d\n",
		evd->raw_ints, evd->source, evd->addr, evd->rw, evd->di);
}

static irqreturn_t sb2_dbg_int_handler(int irq, void *id)
{
	struct platform_device *pdev = id;
	struct sb2_common_data *sb2 = platform_get_drvdata(pdev);
	u32 offset = 0;
	u32 acc_shift = 0;
	struct sb2_dbg_event_data evd = { 0 };
	u32 val;
	u32 ints;

	sb2_reg_read(sb2, SB2_DBG_INT, &ints);
	if (!(ints & SB2_DBG_INT_INT_STATUS))
		return IRQ_NONE;

	/* clear ints */
	sb2_reg_write(sb2, SB2_DBG_INT, SB2_DBG_INT_EN_DEFAULT);

	if (ints & SB2_DBG_INT_SCPU_INT) {
		evd.source = SB2_DBG_SOURCE_SCPU;
		offset = SB2_DBG_ADDR_SCPU;
		acc_shift = 0;
	} else if (ints & SB2_DBG_INT_ACPU_INT) {
		evd.source = SB2_DBG_SOURCE_ACPU;
		offset = SB2_DBG_ADDR_ACPU;
		acc_shift = 2;
	} else if (ints & SB2_DBG_INT_PCPU_INT) {
		evd.source = SB2_DBG_SOURCE_PCPU;
		offset = SB2_DBG_ADDR_PCPU;
		acc_shift = 4;
	} else if (ints & SB2_DBG_INT_VCPU_INT) {
		evd.source = SB2_DBG_SOURCE_VCPU;
		offset = SB2_DBG_ADDR_VCPU;
		acc_shift = 8;
	}

	evd.raw_ints = ints;
	sb2_reg_read(sb2, offset, &evd.addr);
	sb2_reg_read(sb2, SB2_DBG_ADDR1, &val);
	val >>= acc_shift;
	evd.rw = (val & 0x2) ? SB2_DBG_ACCESS_WRITE : SB2_DBG_ACCESS_READ;
	evd.di = (val & 0x1) ? SB2_DBG_ACCESS_DATA : SB2_DBG_ACCESS_INST;

	sb2_dbg_handle_event(&evd);

	return IRQ_HANDLED;
}

static void sb2_dbg_enable_interrupt(struct sb2_common_data *sb2)
{
	sb2_reg_write(sb2, SB2_DBG_INT, SB2_DBG_INT_EN_DEFAULT);
}

static void sb2_dbg_disable_interrupt(struct sb2_common_data *sb2)
{
	sb2_reg_write(sb2, SB2_DBG_INT, 0);
}

static int sb2_dbg_probe(struct platform_device *pdev)
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

	sb2->irq = irq_of_parse_and_map(np, 0);
	if (!sb2->irq) {
		dev_err(dev, "failed to parse irq: %d\n", sb2->irq);
		return -ENXIO;
	}

	ret = devm_request_irq(dev, sb2->irq, sb2_dbg_int_handler, IRQF_SHARED,
			       dev_name(dev), pdev);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		return -ENXIO;
	}

	dbg_data = sb2;
	platform_set_drvdata(pdev, sb2);
	sb2_dbg_enable_interrupt(sb2);
	return 0;
}

static int sb2_dbg_remove(struct platform_device *pdev)
{
	struct sb2_common_data *sb2 = platform_get_drvdata(pdev);

	sb2_dbg_disable_interrupt(sb2);
	platform_set_drvdata(pdev, NULL);
	dbg_data = NULL;
	return 0;
}

static int sb2_dbg_suspend(struct device *dev)
{
	struct sb2_common_data *sb2 = dev_get_drvdata(dev);

	dev_info(dev, "enter %s\n", __func__);
	sb2_dbg_disable_interrupt(sb2);
	dev_info(dev, "exit %s\n", __func__);
	return 0;
}

static int sb2_dbg_resume(struct device *dev)
{
	struct sb2_common_data *sb2 = dev_get_drvdata(dev);

	dev_info(dev, "enter %s\n", __func__);
	sb2_dbg_enable_interrupt(sb2);
	dev_info(dev, "exit %s\n", __func__);
	return 0;
}

static struct dev_pm_ops sb2_dbg_pm_ops = {
	.suspend_noirq = sb2_dbg_suspend,
	.resume_noirq  = sb2_dbg_resume,
};

static const struct of_device_id sb2_dbg_match[] = {
	{.compatible = "realtek,sysbrg2-dbg"},
	{},
};
MODULE_DEVICE_TABLE(of, sb2_dbg_match);

static struct platform_driver sb2_dbg_driver = {
	.probe  = sb2_dbg_probe,
	.remove = sb2_dbg_remove,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-sb2-dbg",
		.pm             = &sb2_dbg_pm_ops,
		.of_match_table = of_match_ptr(sb2_dbg_match),
	},
};

static void __exit sb2_dbg_exit(void)
{
	platform_driver_unregister(&sb2_dbg_driver);
}
module_exit(sb2_dbg_exit);

static int __init sb2_dbg_init(void)
{
	return platform_driver_register(&sb2_dbg_driver);
}
subsys_initcall_sync(sb2_dbg_init);

MODULE_DESCRIPTION("Realtek SB2 DBG driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-sb2-dbg");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
