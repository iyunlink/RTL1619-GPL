/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SOC_REALTEK_REFCLK_H
#define __SOC_REALTEK_REFCLK_H

#include <linux/types.h>
#include <linux/math64.h>

struct device_node;
struct refclk_device;

#if IS_ENABLED(CONFIG_RTK_REFCLK)

struct refclk_device *of_refclk_get(struct device_node *np, int index);
struct refclk_device *refclk_get_by_name(const char *name);
void refclk_put(struct refclk_device *refclk);
u64 refclk_get_counter(struct refclk_device *refclk);

#else

static inline struct refclk_device *of_refclk_get(struct device_node *np, int index)
{
	return ERR_PTR(-ENODEV);
}

static inline struct refclk_device *refclk_get_by_name(const char *name)
{
	return ERR_PTR(-ENODEV);
}

static inline void refclk_put(struct refclk_device *refclk)
{}

static inline u64 refclk_get_counter(struct refclk_device *refclk)
{
	return 0;
}

#endif

/* convert the counter value to time ms */
static inline u64 refclk_get_time_ms(struct refclk_device *refclk)
{
	u64 val = refclk_get_counter(refclk);

	do_div(val, 90);
	return val;
}

#endif
