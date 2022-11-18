// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017,2020 Realtek Semiconductor Corp.
 */

#include <linux/cpumask.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <soc/realtek/rtk_cpuhp.h>

struct c3dev {
	struct device                 *dev;
	struct thermal_cooling_device *cdev;
	unsigned long                 max_state;
	unsigned long                 cur_state;
	struct rtk_cpuhp_qos_request  req;
};

static int c3dev_get_max_state(struct thermal_cooling_device *cdev,
			       unsigned long *state)
{
	struct c3dev *c = cdev->devdata;

	*state = c->max_state;
	return 0;
}

static int c3dev_get_cur_state(struct thermal_cooling_device *cdev,
			       unsigned long *state)
{
	struct c3dev *c = cdev->devdata;

	*state = c->cur_state;
	return 0;
}

static int c3dev_set_cur_state(struct thermal_cooling_device *cdev,
			       unsigned long state)
{
	struct c3dev *c = cdev->devdata;

	rtk_cpuhp_qos_update_request(&c->req, (int)state);
	return 0;
}

static struct thermal_cooling_device_ops c3dev_cooling_ops = {
	.get_max_state = c3dev_get_max_state,
	.get_cur_state = c3dev_get_cur_state,
	.set_cur_state = c3dev_set_cur_state,
};

static int c3dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct c3dev *c;
	int ret;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	c->dev = dev;
	c->max_state = num_possible_cpus() - 1;
	c->cur_state = 0;
	rtk_cpuhp_qos_add_request(&c->req, 0);

	c->cdev = thermal_of_cooling_device_register(np, "thermal-cpu-core",
						     c, &c3dev_cooling_ops);
	if (IS_ERR(c->cdev)) {
		ret = PTR_ERR(c->cdev);
		dev_err(dev, "failed to register cooling device: %d\n", ret);
		rtk_cpuhp_qos_remove_request(&c->req);
		return ret;
	}

	return 0;
}

static int c3dev_remove(struct platform_device *pdev)
{
	struct c3dev *c = platform_get_drvdata(pdev);

	thermal_cooling_device_unregister(c->cdev);
	rtk_cpuhp_qos_remove_request(&c->req);
	return 0;
}

static const struct of_device_id c3dev_ids[] = {
        { .compatible = "realtek,cpu-core-cooling" },
        {}
};

static struct platform_driver c3dev_driver = {
        .probe = c3dev_probe,
        .remove = c3dev_remove,
        .driver = {
                .owner = THIS_MODULE,
                .name = "rtk-cpu-core-cooling",
                .of_match_table = c3dev_ids,
        },
};
module_platform_driver(c3dev_driver);

MODULE_DESCRIPTION("Realtek CPU core cooling Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
