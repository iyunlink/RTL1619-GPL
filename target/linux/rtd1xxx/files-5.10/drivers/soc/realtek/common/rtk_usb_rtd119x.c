// SPDX-License-Identifier: GPL-2.0-only
/*
 * rtk_usb.c
 *
 * Copyright (c) 2017 Realtek Semiconductor Corp.
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 *
 */

#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include<linux/module.h>

#include "rtk_usb.h"

struct rtk_usb {
	struct power_ctrl_reg {
	} power_ctrl_reg;

	bool usb_power_cut;
};

static struct rtk_usb *rtk_usb;

static int usb_set_hw_l4icg_on_off(struct rtk_usb *rtk_usb,
	    enum usb_port_num port_num, bool on)
{
	if (!rtk_usb)
		return 0;

	return 0;
}

static int usb_iso_power_ctrl(struct rtk_usb *rtk_usb,
	    bool power_on)
{
	if (!rtk_usb)
		return 0;

	return 0;
}

static int usb_port_suspend_resume(struct rtk_usb *rtk_usb,
	    enum usb_port_num port_num, bool is_suspend)
{
	if (!rtk_usb)
		return 0;

	return 0;
}

static struct rtk_usb *usb_soc_init(struct device_node *sub_node)
{
	if (!rtk_usb)
		rtk_usb = kzalloc(sizeof(struct rtk_usb), GFP_KERNEL);

	return rtk_usb;
}

static int usb_soc_free(struct rtk_usb **rtk_usb)
{
	if (*rtk_usb) {
		kfree(*rtk_usb);
		*rtk_usb = NULL;
	}

	return 0;
}

int rtk_usb_rtd119x_init(struct rtk_usb_ops *ops)
{

	if (ops == NULL)
		return -1;

	ops->usb_soc_init = &usb_soc_init;
	ops->usb_soc_free = &usb_soc_free;

	ops->usb_port_suspend_resume = &usb_port_suspend_resume;
	ops->usb_set_hw_l4icg_on_off = &usb_set_hw_l4icg_on_off;

	ops->usb_iso_power_ctrl = &usb_iso_power_ctrl;

	return 0;
}
EXPORT_SYMBOL(rtk_usb_rtd119x_init);

MODULE_LICENSE("GPL");
