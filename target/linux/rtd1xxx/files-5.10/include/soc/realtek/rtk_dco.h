/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SOC_REALTEK_DCO_H
#define __SOC_REALTEK_DCO_H

#include <linux/types.h>

struct device_node;
struct dco_cal_data;

#if IS_ENABLED(CONFIG_RTK_DCO)

struct dco_cal_data *of_dco_get(struct device_node *np, int index);
void dco_put(struct dco_cal_data *data);
int dco_recalibrate(struct dco_cal_data *data);

#else

static inline struct dco_cal_data *of_dco_get(struct device_node *np, int index)
{
	return ERR_PTR(-ENODEV);
}

static inline void dco_put(struct dco_cal_data *data)
{
}

static inline int dco_recalibrate(struct dco_cal_data *data)
{
	return -ENODEV;
}

#endif /* CONFIG_RTK_DCO */

#endif /* __SOC_REALTEK_DCO_H */
