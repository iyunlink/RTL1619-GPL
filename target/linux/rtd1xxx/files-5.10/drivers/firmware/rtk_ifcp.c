// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/arm-smccc.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

static unsigned long __invoke_ifcp_fn_smc(unsigned long func_id, unsigned long arg0,
					  unsigned long arg1, unsigned long arg2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(func_id, arg0, arg1, arg2, 0, 0, 0, 0, &res);

	return res.a0;
}

static unsigned long __invoke_ifcp_fn_hvc(unsigned long func_id, unsigned long arg0,
					  unsigned long arg1, unsigned long arg2)
{
	struct arm_smccc_res res;

	arm_smccc_hvc(func_id, arg0, arg1, arg2, 0, 0, 0, 0, &res);

	return res.a0;
}

typedef unsigned long (ifcp_fn)(unsigned long, unsigned long,
				unsigned long, unsigned long);

static ifcp_fn *invoke_ifcp_fn;

int rtk_ifcp_reset(void)
{
	unsigned long v;

	if (!invoke_ifcp_fn)
		return -ENODEV;

	return invoke_ifcp_fn(0x8400ff35, 0, 0, 0);
}
EXPORT_SYMBOL_GPL(rtk_ifcp_reset);

static void set_conduit(enum arm_smccc_conduit conduit)
{
	switch (conduit) {
	case SMCCC_CONDUIT_HVC:
		invoke_ifcp_fn = __invoke_ifcp_fn_hvc;
		break;
	case SMCCC_CONDUIT_SMC:
		invoke_ifcp_fn = __invoke_ifcp_fn_smc;
		break;
	default:
		WARN(1, "Unexpected conduit %d\n", conduit);
	}
}

static int set_conduit_by_method(struct device_node *np)
{
	const char *method;

	if (of_property_read_string(np, "method", &method))
		return -ENXIO;

	if (!strcmp("hvc", method))
		set_conduit(SMCCC_CONDUIT_HVC);
	else if (!strcmp("smc", method))
		set_conduit(SMCCC_CONDUIT_SMC);
	else
		return -EINVAL;
	return 0;
}

static int rtk_ifcp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	ret = set_conduit_by_method(dev->of_node);
	if (ret) {
		dev_err(dev, "failed to set conduit: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id rtk_ifcp_of_match[] = {
	{ .compatible = "realtek,ifcp" },
	{}
};

static struct platform_driver rtk_ifcp_driver = {
	.driver         = {
		.name           = "rtk-ifcp",
		.of_match_table = of_match_ptr(rtk_ifcp_of_match),
	},
	.probe = rtk_ifcp_probe,
};

static __init int rtk_ifcp_init(void)
{
	return platform_driver_register(&rtk_ifcp_driver);
}
subsys_initcall(rtk_ifcp_init);

MODULE_LICENSE("GPL v2");
