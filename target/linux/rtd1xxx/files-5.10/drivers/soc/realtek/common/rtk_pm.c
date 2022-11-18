// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek DHC SoC family power management driver
 * Copyright (c) 2020-2021 Realtek Semiconductor Corp.
 */

#include <linux/arm-smccc.h>
#include <linux/cpu.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/psci.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/dma-mapping.h>

#include <soc/realtek/rtk_pm.h>
#include <uapi/linux/psci.h>

static u32 dco_mode;

static void rtk_pm_adjust_dco(unsigned int tc_emb, unsigned int s_emb, struct pm_private *dev_pm)
{
	unsigned int tmp = 0;

	regmap_write(dev_pm->syscon_iso, PLL_ETN_OSC, 0);

	tmp = (tc_emb << 9) | (s_emb << 1);
	regmap_write(dev_pm->syscon_iso, PLL_ETN_OSC, tmp);
	tmp |= 0x1;
	regmap_write(dev_pm->syscon_iso, PLL_ETN_OSC, tmp);

	regmap_read(dev_pm->syscon_iso, PLL_ETN_OSC, &tmp);
	dev_info(dev_pm->dev, "PLL_ETN_OSC= %x\n", tmp);
}

static unsigned int rtk_pm_get_dco_cnt(struct pm_private *dev_pm)
{
	unsigned int tmp = 0;
	unsigned int retry_count = 0;

	regmap_write(dev_pm->syscon_iso, DCO0, 0);
	regmap_write(dev_pm->syscon_iso, DCO1, 0);

	tmp = (OSC_COUNT_LIMIT << 10) | 0x1;
	regmap_write(dev_pm->syscon_iso, DCO0, tmp);

	mdelay(1);

	regmap_read(dev_pm->syscon_iso, DCO1, &tmp);
	while (!(0x1 & tmp)) {
		if (retry_count == 500) {
			tmp = 0x0;
			goto error;
		}
		regmap_read(dev_pm->syscon_iso, DCO1, &tmp);
		retry_count++;
	}

	tmp = (tmp >> 13) & 0xfff;
error:
	return tmp;
}

static unsigned int rtk_pm_get_xtal_cnt(struct pm_private *dev_pm)
{
	unsigned int tmp = 0;
	unsigned int retry_count = 0;

	regmap_write(dev_pm->syscon_iso, DCO0, 0);
	regmap_write(dev_pm->syscon_iso, DCO1, 0);

	tmp = (OSC_COUNT_LIMIT << 10) | (0x1 << 1) | 0x1;
	regmap_write(dev_pm->syscon_iso, DCO0, tmp);

	mdelay(1);

	regmap_read(dev_pm->syscon_iso, DCO1, &tmp);
	while (!(0x1 & tmp)) {
		if (retry_count == 500) {
			tmp = 0x0;
			goto error;
		}
		regmap_read(dev_pm->syscon_iso, DCO1, &tmp);
		retry_count++;
	}

	tmp = (tmp >> 1) & 0xfff;
error:
	return tmp;
}

static int rtk_pm_dco(struct pm_private *dev_pm)
{
	unsigned int tc_emb = 4;
	unsigned int s_emb = 0x40;
	unsigned int xtal_cnt = 0;
	unsigned int dco_cnt = 0;
	unsigned int tmp = 0;
	unsigned int retry_count = 0;
	unsigned int ret = 0;

	rtk_pm_adjust_dco(tc_emb, s_emb, dev_pm);
	dco_cnt  = rtk_pm_get_dco_cnt(dev_pm);

	while (1) {
		if (retry_count == 500) {
			dev_err(dev_pm->dev, "The calibration can not be found!\n");
			ret = -1;
			goto error;
		}

		tmp = 2700 *  dco_cnt / OSC_COUNT_LIMIT;
		if ((tmp / 100) == 27) {
			if ((tmp % 100) < 20) {
				xtal_cnt = rtk_pm_get_xtal_cnt(dev_pm);
				ret = 0;
				break;
			}
			s_emb--;
		} else if ((tmp / 100) > 27)
			s_emb--;
		else if ((tmp / 100) < 27)
			s_emb++;

		dev_info(dev_pm->dev, "s_emb = %x\n", s_emb);

		rtk_pm_adjust_dco(tc_emb, s_emb, dev_pm);
		dco_cnt  = rtk_pm_get_dco_cnt(dev_pm);
		retry_count++;
	}

	dev_info(dev_pm->dev, "dco count latch = %x\n", dco_cnt);
	dev_info(dev_pm->dev, "xtal count latch = %x\n", xtal_cnt);

	tmp = 2700 * dco_cnt / OSC_COUNT_LIMIT;
	dev_info(dev_pm->dev, "dco freq = %u.%02uMHz", (tmp / 100), (tmp % 100));

	tmp = 2700 * OSC_COUNT_LIMIT / xtal_cnt;
	dev_info(dev_pm->dev, "xtal freq = %u.%02uMHz\n", (tmp / 100), (tmp % 100));
error:
	return ret;
}

static void rtk_pm_get_gpio_param(struct pm_private *dev_pm)
{
	struct device *dev = dev_pm->dev;
	struct pm_pcpu_param *pcpu_param = dev_pm->pcpu_param;
	int num_row = 0;
	int i = 0;
	u32 gpio_act = 0;
	u32 gpio_en = 0;
	u32 gpio_num = 0;
	char *pname = "wakeup-gpio-list";
	const u32 element = 3;

	num_row = of_property_count_u32_elems(dev->of_node, pname);
	if (num_row < 0) {
		dev_err(dev, "Not found '%s' property\n", pname);
		return;
	}

	num_row /= element;

	for (i = 0; i < num_row; i++) {
		of_property_read_u32_index(dev->of_node, pname, i * element,
					   &gpio_num);
		of_property_read_u32_index(dev->of_node, pname, i * element + 1,
					   &gpio_en);
		of_property_read_u32_index(dev->of_node, pname, i * element + 2,
					   &gpio_act);
		pcpu_param->wu_gpio_en[gpio_num] = (char) gpio_en;
		pcpu_param->wu_gpio_act[gpio_num] = (char) gpio_act;
	}
}

static void rtk_pm_shutdown(struct platform_device *pdev)
{
	struct pm_private *dev_pm = dev_get_drvdata(&pdev->dev);
	int ret = 0;

	if (dco_mode == true)
		ret = rtk_pm_dco(dev_pm);

	rtk_pm_set_pcpu_param(&pdev->dev);
};

static int rtk_pm_prepare(struct device *dev)
{
	struct pm_private *dev_pm = dev_get_drvdata(dev);
	struct pm_dev_param *lan_node = rtk_pm_get_param(LAN);
	int ret = 0;

	if (lan_node == NULL)
		goto skip_set_dco_param;

	if (dev_pm->pcpu_param->wakeup_source & DCO_ENABLE) {
		 *(int *)lan_node->data = DCO_ENABLE;
		dev_pm->pcpu_param->wakeup_source &= 0xfffffffe;
	} else
		*(int *)lan_node->data = 0;

skip_set_dco_param:

	dev_pm->wakeup_reason = MAX_EVENT;
	dev_pm->device_param->data = &dev_pm->wakeup_reason;

	return ret;
}

static int rtk_pm_suspend(struct device *dev)
{
	struct pm_private *dev_pm = dev_get_drvdata(dev);
	int ret = 0;

	if (dco_mode == true)
		ret = rtk_pm_dco(dev_pm);

	dev_pm->wakeup_reason = MAX_EVENT;
	regmap_read(dev_pm->syscon_iso, 0x640, &dev_pm->reboot_reasons);

	rtk_pm_set_pcpu_param(dev);

	return ret;
}

static int rtk_pm_resume(struct device *dev)
{
	struct pm_private *dev_pm = dev_get_drvdata(dev);
	struct pm_pcpu_param *pcpu_param = dev_pm->pcpu_param;
	int ret = 0;

	pcpu_param->wakeup_source = htonl(pcpu_param->wakeup_source);
	pcpu_param->timerout_val = htonl(pcpu_param->timerout_val);

	regmap_read(dev_pm->syscon_iso, 0x640, &dev_pm->wakeup_reason);
	dev_pm->wakeup_reason = (dev_pm->wakeup_reason >> 16) & 0x00ff;

	regmap_write(dev_pm->syscon_iso, 0x640, dev_pm->reboot_reasons);

	dev_pm->device_param->data = &dev_pm->wakeup_reason;
	dev_pm->suspend_context++;

	return ret;
}

static int rtk_pm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *syscon_np;
	struct pm_private *dev_pm;
	struct pm_dev_param *dev_param;
	struct pm_pcpu_param *pcpu_param;
	const u32 *prop;
	u32 timer_value = 0;
	int ret = 0;
	int size = 0;
	unsigned int test = 0;

	dev_pm = devm_kzalloc(dev, sizeof(*dev_pm), GFP_KERNEL);
	if (!dev_pm) {
		ret = -ENOMEM;
		goto error_mem;
	}

	pcpu_param = dma_alloc_coherent(dev, sizeof(*pcpu_param), &dev_pm->pcpu_param_pa, GFP_KERNEL);
	if (!pcpu_param) {
		ret = -ENOMEM;
		goto error_mem;
	}

	dev_param = devm_kzalloc(dev, sizeof(*dev_param), GFP_KERNEL);
	if (!dev_param) {
		ret = -ENOMEM;
		goto error_mem;
	}

	dco_mode = 0;

	dev_pm->pm_dbg = false;
	dev_pm->dev = dev;

	dev_param->dev = dev;
	dev_param->dev_type = PM;

	pcpu_param->wakeup_source = 0;
	pcpu_param->timerout_val = 0;

	dev_pm->pcpu_param = pcpu_param;
	dev_pm->device_param = dev_param;

	syscon_np = of_parse_phandle(dev->of_node, "syscon", 0);
	if (IS_ERR_OR_NULL(syscon_np)) {
		dev_err(dev, "Not found syscon property\n");
		ret = -ENODEV;
		goto error_mem;
	}

	dev_pm->syscon_iso = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(dev_pm->syscon_iso)) {
		dev_err(dev, "Cannot get parent's regmap\n");
		ret = PTR_ERR(dev_pm->syscon_iso);
		goto error_syscon;
	}

	prop = of_get_property(dev->of_node, "wakeup-flags", &size);
	if (of_read_number(prop, 1))
		pcpu_param->wakeup_source = of_read_number(prop, 1);
	else
		dev_err(dev, "Not found wakeup-flags property\n");

	prop = of_get_property(dev->of_node, "pm-dbg", &size);
	if (of_read_number(prop, 1))
		dev_pm->pm_dbg = true;
	else
		dev_pm->pm_dbg = false;

	ret = of_property_read_u32(dev->of_node, "wakeup-timer", &timer_value);
	if (ret)
		dev_err(dev, "Not found wakeup-timer property\n");
	else
		pcpu_param->timerout_val = timer_value;


	ret = of_property_read_u32(dev->of_node, "dco", &dco_mode);
	if (ret)
		dev_err(dev, "Not found dco property\n");

	if (dco_mode == true) {
		pcpu_param->wakeup_source &= 0xfffffffe;
		pcpu_param->wakeup_source |= DCO_ENABLE;
	}

	ret = rtk_pm_create_sysfs();
	if (ret)
		dev_err(dev, "Cannot create sysfs\n");

	regmap_read(dev_pm->syscon_iso, 0x640, &dev_pm->wakeup_reason);
	test = (dev_pm->wakeup_reason >> 16) & 0x00ff;
	if (test > MAX_EVENT ||  test < IR_EVENT)
		dev_pm->wakeup_reason = MAX_EVENT;
	else
		dev_pm->wakeup_reason = test;

	dev_param->data = &dev_pm->wakeup_reason;

	rtk_pm_get_gpio_param(dev_pm);
	rtk_pm_add_list(dev_param);
	platform_set_drvdata(pdev, dev_pm);

error_syscon:
	of_node_put(syscon_np);

error_mem:
	return ret;
};

static int rtk_pm_remove(struct platform_device *pdev)
{
	int ret = 0;

	return ret;
}

static const struct dev_pm_ops rtk_pm_ops = {
	.prepare = rtk_pm_prepare,
	.suspend_noirq = rtk_pm_suspend,
	.resume_noirq = rtk_pm_resume,
};

static struct of_device_id rtk_pm_ids[] = {
	{ .compatible = "realtek,rtd13xx_pm" },
	{ .compatible = "realtek,rtd13xxd_pm" },
	{ .compatible = "realtek,rtd16xxb_pm" },
	{ /* Sentinel */ },
};

static struct platform_driver rtk_pm_driver = {
	.probe = rtk_pm_probe,
	.remove = rtk_pm_remove,
	.shutdown = rtk_pm_shutdown,
	.driver = {
		.name = "realtek-pm",
		.owner = THIS_MODULE,
		.of_match_table = rtk_pm_ids,
		.pm = &rtk_pm_ops,
	},
};
module_platform_driver(rtk_pm_driver);

MODULE_AUTHOR("James Tai <james.tai@realtek.com>");
MODULE_DESCRIPTION("Realtek power management driver");
MODULE_LICENSE("GPL v2");
