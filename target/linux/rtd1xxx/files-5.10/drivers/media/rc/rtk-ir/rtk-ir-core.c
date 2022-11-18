// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Realtek IR Receiver Controller
 *
 * Copyright (C) 2020 Simon Hsu <simon_hsu@realtek.com>
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/interrupt.h>

#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include "rtk-ir.h"

#define RTK_IR_DEV "rtk-ir"

static const struct of_device_id rtk_ir_match[] = {
	{ .compatible = "realtek,rtk-ir" },
	{},
};
MODULE_DEVICE_TABLE(of, rtk_ir_match);

static irqreturn_t rtk_ir_irq(int irq, void *dev_id)
{
	struct rtk_ir __maybe_unused *ir = (struct rtk_ir *)dev_id;

	rtk_ir_isr_hw(ir);
	rtk_ir_isr_hw1(ir);
	rtk_ir_isr_raw(ir);

	return IRQ_HANDLED;
}

static int rtk_ir_key_event(struct notifier_block *notifier,
			    unsigned long pm_event,
			    void *unused)
{
	struct rtk_ir *ir = container_of(notifier, struct rtk_ir, pm_notifier);
	struct device *dev = ir->dev;
	struct rc_dev *rc = ir->hw.rc;
	struct rc_map *map = &rc->rc_map;
	struct pm_dev_param *node;
	bool sendKeyEvent = false;
	unsigned int wakeup_event, i = 0, j = 0, val;

	if (pm_event == PM_SUSPEND_PREPARE)
		return 0;

	node = rtk_pm_get_param(PM);
	wakeup_event = *((unsigned int *) node->data);

	/* error handling */
	if (wakeup_event >= MAX_EVENT) {
		dev_err(dev, "Bad wakeup event value %d !\n", wakeup_event);
		return NOTIFY_DONE;
	}

	/* after resume */
	switch (wakeup_event) {
	case ALARM_EVENT:
	case TIMER_EVENT:
		sendKeyEvent = false;
        break;
	default:
		sendKeyEvent = true;
	}

	if (sendKeyEvent) {
		struct rtk_ir_hw_decoder *dec = ir->hw.dec;
		struct rtk_ir_scancode_req request;

		regmap_read(ir->iso, 0x65c, &val);
		dev_info(dev, "Wakeup entire android src %d, 0x%x\n", wakeup_event, val);
		dec->scancode(&request, val);
		for (i = 0; i < map->len; i++)
			if (request.scancode == (map->scan + i)->scancode)
				break;
		if (i == map->len)
			for (j = 0; j < map->len; j++)
				if (KEY_POWER == (map->scan + j)->keycode){
					request.scancode = (map->scan + j)->scancode;
					break;
				}

		if (i != map->len){
			pr_err("[IRDA_RESUME][%s] Scan code: 0x%x\n", __func__, request.scancode);
			rc_keydown(rc, map->rc_proto, request.scancode, 0);
		} else if (wakeup_event == 7){
			pr_err("[HIFI_RESUME][%s] Wakeup from HIFI", __func__);
			rc_keydown(rc, map->rc_proto, 0x8018, 0);
		}
	}

	return NOTIFY_DONE;
}

static void rtk_ir_get_wakeinfo(struct rtk_ir_hw *hw, struct ipc_shm_irda *data,
			       unsigned int keycode)
{
	struct rc_dev *rc = hw->rc;
	struct rc_map *map = &rc->rc_map;
	struct irda_wake_up_key *tbl;
	struct rtk_ir_wakeinfo info;
	int i, j, idx;

	for (i = 0; i < map->len; i++) {
		if (keycode != (map->scan + i)->keycode && keycode != 0xffffffff)
			continue;

		hw->dec->wakeinfo(&info, (map->scan + i)->scancode);

		for (j = 0; j < MAX_KEY_TBL; j++) {
			tbl = data->key_tbl + j;
			idx = htonl(tbl->wakeup_keynum);
			if (idx != 0 && info.addr != htonl(tbl->cus_code))
				continue;

			if (idx == 0) {
				tbl->cus_code = htonl(info.addr);
				tbl->cus_mask = htonl(info.addr_msk);
			}
			tbl->scancode_mask = htonl(info.scancode_msk);
			tbl->wakeup_scancode[idx++] = htonl(info.scancode);
			tbl->wakeup_keynum = htonl(idx);
			break;
		}
	}
	return;
}

static void rtk_ir_set_wakeup(struct rtk_ir *ir)
{
	struct device *dev = ir->dev;
	struct device_node *np = dev->of_node;
	struct ipc_shm_irda *pcpu_data = &ir->pcpu_data;
	unsigned int keycode[MAX_WAKEUP_CODE];
	int i, num;

	num = of_property_count_u32_elems(np, "wakeup-key");
	if (num < 0) {
		num = 1;
		keycode[0] = KEY_POWER;
	} else {
		if (num > MAX_WAKEUP_CODE)
			num = MAX_WAKEUP_CODE;
		of_property_read_u32_array(np, "wakeup-key", keycode, num);
	}

	if (ir->hw.rc) {
		for (i = 0; i < num; i++)
			rtk_ir_get_wakeinfo(&ir->hw, pcpu_data, keycode[i]);
		pcpu_data->dev_count++;
	}

	if (ir->hw1.rc) {
		for (i = 0; i < num; i++)
			rtk_ir_get_wakeinfo(&ir->hw1, pcpu_data, keycode[i]);
		pcpu_data->dev_count++;
	}

	ir->pm_param.dev = ir->dev;
	ir->pm_param.dev_type = IRDA;
	ir->pm_param.data = pcpu_data;

	rtk_pm_add_list(&ir->pm_param);
}

static int rtk_ir_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct rtk_ir *ir;
	int ret;

	ir = devm_kzalloc(dev, sizeof(*ir), GFP_KERNEL);
	if (!ir)
		return -ENOMEM;

	ir->dev = dev;
	ir->clk = devm_clk_get(dev, "irda");
	if (IS_ERR(ir->clk)) {
		dev_err(dev, "failed to get ir clock.\n");
		return PTR_ERR(ir->clk);
	}
	ir->rsts = devm_reset_control_array_get_optional_shared(dev);
	if (IS_ERR(ir->rsts)) {
		dev_err(dev, "failed to get ir reset.\n");
		return PTR_ERR(ir->rsts);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ir->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ir->base))
		return PTR_ERR(ir->base);

	ir->iso = syscon_regmap_lookup_by_phandle(dev->of_node, "syscon");
	if (IS_ERR(ir->iso)) {
		dev_err(dev, "failed to remap iso regs\n");
		return PTR_ERR(ir->iso);
	}

	platform_set_drvdata(pdev, ir);

	ir->irq = platform_get_irq(pdev, 0);
	if (ir->irq < 0)
		return -ENODEV;

	ret = devm_request_irq(dev, ir->irq, rtk_ir_irq, 0, RTK_IR_DEV, ir);
	if (ret) {
		dev_err(dev, "failed request irq\n");
		return -EINVAL;
	}

	if (clk_prepare_enable(ir->clk)) {
		dev_err(dev, "try to enable ir_clk failed\n");
		return -EINVAL;
	}
	reset_control_deassert(ir->rsts);

	if (__is_defined(CONFIG_IR_RTK_HW) && rtk_ir_hw_probe(ir)) {
		dev_err(dev, "ir hw probe fail");
		goto err_probe_fail;
	}
	if (__is_defined(CONFIG_IR_RTK_HW1) && rtk_ir_hw1_probe(ir)) {
		dev_err(dev, "ir hw1 probe fail");
		goto err_probe_fail;
	}
	if (__is_defined(CONFIG_IR_RTK_RAW) && rtk_ir_raw_probe(ir)) {
		dev_err(dev, "ir raw probe fail");
		goto err_probe_fail;
	}

	ir->pm_notifier.notifier_call = rtk_ir_key_event;
	ret = register_pm_notifier(&ir->pm_notifier);
	if (ret) {
		dev_err(dev, "error registering pm notifier(%d)\n", ret);
		goto err_probe_fail;
	}

	rtk_ir_set_wakeup(ir);

	return 0;

err_probe_fail:
	return -1;
}

static int rtk_ir_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM
static int rtk_ir_suspend(struct device *dev)
{
	struct rtk_ir __maybe_unused *ir = dev_get_drvdata(dev);

	rtk_ir_hw_suspend(ir);
	rtk_ir_hw1_suspend(ir);

	return 0;
}

static int rtk_ir_resume(struct device *dev)
{
	struct rtk_ir __maybe_unused *ir = dev_get_drvdata(dev);

	rtk_ir_hw_resume(ir);
	rtk_ir_hw1_resume(ir);

	return 0;
}

static void rtk_ir_shutdown(struct platform_device *pdev)
{
	rtk_ir_suspend(&pdev->dev);
}

static const struct dev_pm_ops rtk_irda_pm_ops = {
	.suspend = rtk_ir_suspend,
	.resume = rtk_ir_resume,
};

#else

static const struct dev_pm_ops rtk_irda_pm_ops = {};

#endif

static struct platform_driver rtk_ir_driver = {
	.probe          = rtk_ir_probe,
	.remove         = rtk_ir_remove,
	.driver = {
		.name = RTK_IR_DEV,
		.of_match_table = rtk_ir_match,
#ifdef CONFIG_PM
		.pm = &rtk_irda_pm_ops,
#endif /* CONFIG_PM */
	},
#ifdef CONFIG_PM
	.shutdown = rtk_ir_shutdown,
#endif /* CONFIG_PM */
};

module_platform_driver(rtk_ir_driver);

MODULE_DESCRIPTION("Realtek IR Receiver Controller Driver");
MODULE_AUTHOR("Simon Hsu <simon_hsu@realtek.com>");
MODULE_LICENSE("GPL");
