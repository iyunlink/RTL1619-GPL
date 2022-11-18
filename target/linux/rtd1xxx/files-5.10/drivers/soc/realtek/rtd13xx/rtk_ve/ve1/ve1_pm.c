/*
 * ve1_pm.c - ve1 power management
 *
 * Copyright (c) 2019 Realtek Semiconductor Corporation
 *
 * Author:
 *      Cheng-Yu Lee <cylee12@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/reset.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/soc/realtek/rtk_pd.h>
#include "ve1_pm.h"

#define RTK_ID_VE1 0
#define RTK_ID_VE3 1
#define RTK_ID_MAX 2

struct ve_pm_data {
	struct device *dev;
	struct device *pd_dev;
};

static struct ve_pm_data ve_pm_data[RTK_ID_MAX];

#define get_ve_pm_data_by_id(id)   (&ve_pm_data[(id)])

int ve_pd_power_on(int id)
{
	struct ve_pm_data *data = get_ve_pm_data_by_id(id);

	pr_debug("%s: id=%d\n", __func__, id);

	if (WARN_ON_ONCE(!data->pd_dev))
		return -ENODEV;

	pm_runtime_get_sync(data->pd_dev);
	return 0;
}

int ve_pd_power_off(int id)
{
	struct ve_pm_data *data = get_ve_pm_data_by_id(id);

	pr_debug("%s: id=%d\n", __func__, id);

	if (WARN_ON_ONCE(!data->pd_dev))
		return -ENODEV;

	pm_runtime_mark_last_busy(data->pd_dev);
	pm_runtime_put_autosuspend(data->pd_dev);
	return 0;
}

int ve_pd_reset_control_reset(int id)
{
	return -EINVAL;
}

static void ve_pd_init_pd_dev(struct device *dev, int index)
{
	struct ve_pm_data *data = get_ve_pm_data_by_id(index);

	data->dev = dev;
	data->pd_dev = dev_pm_domain_attach_by_id(dev, index);
	if (IS_ERR_OR_NULL(data->pd_dev)) {
		data->pd_dev = NULL;
		dev_warn(dev, "failed to get pm domain %d\n", index);
		return;
	}

	/* setup */
	pm_runtime_use_autosuspend(data->pd_dev);
	pm_runtime_set_autosuspend_delay(data->pd_dev, 15000);
	pm_runtime_set_active(data->pd_dev);

	/* power off */
	pm_runtime_get_sync(data->pd_dev);
	pm_runtime_mark_last_busy(data->pd_dev);
	pm_runtime_put_autosuspend(data->pd_dev);
}

static void ve_pd_fini_pd_dev(int index)
{
	struct ve_pm_data *data = get_ve_pm_data_by_id(index);

	dev_pm_domain_detach(data->pd_dev, 1);
}

int ve_pd_init(struct device *dev)
{
	ve_pd_init_pd_dev(dev, RTK_ID_VE1);
	ve_pd_init_pd_dev(dev, RTK_ID_VE3);
	return 0;
}

void ve_pd_exit(struct device *dev)
{
	ve_pd_fini_pd_dev(RTK_ID_VE1);
	ve_pd_fini_pd_dev(RTK_ID_VE3);
}

