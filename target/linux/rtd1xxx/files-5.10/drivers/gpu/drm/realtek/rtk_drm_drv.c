// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Realtek Inc.
 * Author: Simon Hsu <simon_hsu@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drm_crtc_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_vblank.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_file.h>
#include <drm/drm_of.h>
#include <linux/component.h>
#include <linux/platform_device.h>

#include "rtk_drm_drv.h"
#include "rtk_drm_fb.h"
#include "rtk_drm_gem.h"
#include "rtk_drm_rpc.h"

#define DRIVER_NAME	"rtk_drm"
#define DRIVER_DESC	"DRM module for RTK"
#define DRIVER_DATE	"20170207"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	5

#define MAX_RTK_SUB_DRIVERS 16

static struct platform_driver *rtk_drm_sub_drivers[MAX_RTK_SUB_DRIVERS];
static int num_sub_drivers;
static struct drm_driver rtk_drm_driver;

#if defined(CONFIG_DEBUG_FS)
static const struct drm_info_list rtk_drm_debugfs_list[] = {
	{"gem_info", rtk_gem_info_debugfs, 0},

};

void rtk_drm_debugfs_init(struct drm_minor *minor)
{
	drm_debugfs_create_files(rtk_drm_debugfs_list,
				ARRAY_SIZE(rtk_drm_debugfs_list),
				minor->debugfs_root, minor);
}

#endif

static int rtk_drm_bind(struct device *dev)
{
	struct drm_device *drm;
	struct rtk_drm_private *priv;
	int ret;

	drm = drm_dev_alloc(&rtk_drm_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	dev_set_drvdata(dev, drm);

	priv = devm_kzalloc(drm->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto err_free_drm;
	}
	drm->dev_private = priv;
	mutex_init(&priv->obj_lock);
	ret = rtk_rpc_init(dev, &priv->rpc_info);
	if (ret < 0)
		return ret;
	drm_mode_config_init(drm);

	rtk_drm_mode_config_init(drm);

	ret = component_bind_all(dev, drm);
	if (ret)
		goto err_bind_device;

	drm_kms_helper_poll_init(drm);

	drm_mode_config_reset(drm);

	drm->irq_enabled = true;
	ret = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (ret)
		goto err_vblank_init;

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_register_drm;

	return 0;

err_register_drm:
err_vblank_init:
	drm_kms_helper_poll_fini(drm);
	component_unbind_all(dev, drm);
err_bind_device:
err_free_drm:
	drm_dev_put(drm);
	return ret;
}

static void rtk_drm_unbind(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	drm_kms_helper_poll_fini(drm);
	component_unbind_all(dev, drm);
	drm_mode_config_cleanup(drm);
	drm->dev_private = NULL;
	drm_dev_unregister(drm);
	drm_dev_put(drm);
	dev_set_drvdata(dev, NULL);
}

static const struct file_operations rtk_drm_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.mmap = rtk_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.unlocked_ioctl = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.release = drm_release,
};

static struct drm_driver rtk_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.dumb_create		= rtk_gem_dumb_create,
	.dumb_map_offset	= rtk_gem_dumb_map_offset,
	.dumb_destroy		= drm_gem_dumb_destroy,
	.prime_handle_to_fd     = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle     = drm_gem_prime_fd_to_handle,
	.gem_free_object_unlocked	= rtk_gem_free_object,
	.gem_vm_ops		= &rtk_gem_vm_ops,
	.gem_prime_vmap		= rtk_gem_prime_vmap,
	.gem_prime_vunmap	= rtk_gem_prime_vunmap,
	.gem_prime_mmap		= rtk_gem_prime_mmap,
	.gem_prime_import       = drm_gem_prime_import,
	.gem_prime_export       = drm_gem_prime_export,
	.gem_prime_get_sg_table = rtk_gem_prime_get_sg_table,
	.gem_prime_import_sg_table = rtk_gem_prime_import_sg_table,
#if defined(CONFIG_DEBUG_FS)
	.debugfs_init		= rtk_drm_debugfs_init,
#endif
	.fops			= &rtk_drm_driver_fops,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= DRIVER_DATE,
	.major			= DRIVER_MAJOR,
	.minor			= DRIVER_MINOR,
};

static const struct component_master_ops rtk_drm_ops = {
	.bind = rtk_drm_bind,
	.unbind = rtk_drm_unbind,
};

static int compare_dev(struct device *dev, void *data)
{
	struct device_node *np = data;

	if (dev->of_node != np && dev->of_node != np->parent)
		return 0;
	return 1;
}

static int rtk_drm_probe(struct platform_device *pdev)
{
	return drm_of_component_probe(&pdev->dev, compare_dev, &rtk_drm_ops);
}

static int rtk_drm_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &rtk_drm_ops);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rtk_drm_suspend(struct device *dev)
{
	return 0;
}

static int rtk_drm_resume(struct device *dev)
{
	return 0;
}
static SIMPLE_DEV_PM_OPS(rtk_drm_pm_ops, rtk_drm_suspend,
			 rtk_drm_resume);
#else
static const struct dev_pm_ops rtk_drm_pm_ops = {};
#endif

static const struct of_device_id rtk_drm_of_ids[] = {
	{ .compatible = "realtek,display-subsystem" },
	{ }
};

static struct platform_driver rtk_drm_platform_driver = {
	.probe	= rtk_drm_probe,
	.remove	= rtk_drm_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.of_match_table = rtk_drm_of_ids,
		.pm     = &rtk_drm_pm_ops,
	},
};

#define ADD_RTK_SUB_DRIVER(drv, CONFIG_cond) { \
	if (IS_ENABLED(CONFIG_cond) && \
	    !WARN_ON(num_sub_drivers >= MAX_RTK_SUB_DRIVERS)) \
		rtk_drm_sub_drivers[num_sub_drivers++] = &drv; \
}

static int __init rtk_drm_init(void)
{
	int ret;

	num_sub_drivers = 0;
	ADD_RTK_SUB_DRIVER(rtk_crtc_platform_driver, CONFIG_DRM_RTK);
	ADD_RTK_SUB_DRIVER(rtk_vo_platform_driver, CONFIG_DRM_RTK);
	ADD_RTK_SUB_DRIVER(rtk_hdmi_driver, CONFIG_DRM_RTK);
	ADD_RTK_SUB_DRIVER(rtk_hdmi_legacy_driver, CONFIG_DRM_RTK);
	ADD_RTK_SUB_DRIVER(rtk_dptx_driver, CONFIG_DRM_RTK);

	ret = platform_register_drivers(rtk_drm_sub_drivers, num_sub_drivers);
	if (ret)
		return ret;

	ret = platform_driver_register(&rtk_drm_platform_driver);
	if (ret)
		goto err_register_drm;

	return 0;

err_register_drm:
	platform_unregister_drivers(rtk_drm_sub_drivers, num_sub_drivers);
	return ret;
}

static void __exit rtk_drm_fini(void)
{
	platform_driver_unregister(&rtk_drm_platform_driver);

	platform_unregister_drivers(rtk_drm_sub_drivers, num_sub_drivers);
}

module_init(rtk_drm_init);
module_exit(rtk_drm_fini);

MODULE_AUTHOR("Simon Hsu <simon_hsu@realtek.com>");
MODULE_DESCRIPTION("REALTEK DRM Driver");
MODULE_LICENSE("GPL v2");
