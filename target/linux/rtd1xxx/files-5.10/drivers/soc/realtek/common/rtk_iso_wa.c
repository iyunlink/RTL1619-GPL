/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#define pr_fmt(fmt) "iso_wa: " fmt

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/sys_soc.h>

static int set_gpu_on(void)
{
	struct device_node *np;
	struct platform_device *pdev;
	int ret = 0;

	np = of_find_compatible_node(NULL, NULL, "arm,mali-midgard");
	if (!np)
		return -ENODEV;

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -ENODEV;

	if (!device_is_bound(&pdev->dev))
		return -EINVAL;

	get_device(&pdev->dev);
	if (pm_runtime_enabled(&pdev->dev)) {
		pr_emerg("force gpu on\n");
		pm_runtime_get_sync(&pdev->dev);
	} else
		ret = -EINVAL;
	put_device(&pdev->dev);
	return ret;
}

static int set_gpu_sram_on(void)
{
	struct device_node *np;
	struct regmap *regmap;

	np = of_find_node_by_path("/soc@0/rbus@98000000/syscon@7000");
	if (WARN_ON(!np))
		return -EINVAL;

	regmap = syscon_node_to_regmap(np);
	of_node_put(np);
	if (WARN_ON(IS_ERR_OR_NULL(regmap)))
		return -EINVAL;

	pr_emerg("force gpu sram on\n");

	regmap_update_bits(regmap, 0xfd0, BIT(3), BIT(3));
	regmap_write(regmap, 0xb70, 0xf00);

	return 0;
}


static int reboot_cb(struct notifier_block *nb, unsigned long act, void *cmd)
{
	int ret;

	ret = set_gpu_on();
	if (ret)
		ret = set_gpu_sram_on();
	if (ret)
		return NOTIFY_DONE;
	return NOTIFY_OK;
}

static struct notifier_block reboot_nb = {
	.notifier_call = reboot_cb,
};

static int match_platform(void)
{
	struct soc_device_attribute stark_a00[] = {
		{ .family = "Realtek Stark", .revision = "A00" },
		{ /* sentinel */ }
	};
	return soc_device_match(stark_a00) != NULL;
}

static int __init iso_wa_init(void)
{
	if (match_platform())
		return register_reboot_notifier(&reboot_nb);
	return -EINVAL;
}
module_init(iso_wa_init);

static void __exit iso_wa_exit(void)
{
	unregister_reboot_notifier(&reboot_nb);
}
module_exit(iso_wa_exit);
