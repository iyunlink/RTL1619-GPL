/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * rtk_usb.h
 *
 * Copyright (c) 2017 Realtek Semiconductor Corp.
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 *
 */

#ifndef __RTK_USB_H_INCLUDED_
#define __RTK_USB_H_INCLUDED_

struct device_node;
struct rtk_usb;

enum usb_port_num {
	USB_PORT_0 = 0,
	USB_PORT_1 = 1,
	USB_PORT_2 = 2,
	USB_PORT_3 = 3,
};

struct rtk_usb_ops {
	struct rtk_usb *rtk_usb;

	struct rtk_usb *(*usb_soc_init)(struct device_node *node);
	int (*usb_soc_free)(struct rtk_usb **rtk_usb);

	int (*usb_port_suspend_resume)(struct rtk_usb *rtk_usb,
		    enum usb_port_num port_num, bool is_suspend);
	int (*usb_set_hw_l4icg_on_off)(struct rtk_usb *rtk_usb,
		    enum usb_port_num port_num, bool on);

	int (*usb_iso_power_ctrl)(struct rtk_usb *rtk_usb,
		    bool power_on);

	void (*usb_set_charger_power)(struct rtk_usb *rtk_usb,
		    unsigned int val);

	/* For Type c */
	int (*type_c_init)(struct rtk_usb *rtk_usb);
	int (*type_c_plug_config)(struct rtk_usb *rtk_usb,
		    int dr_mode, int cc);
};

static inline int rtk_usb_soc_init(struct rtk_usb_ops *ops,
	    struct device_node *node)
{
	if (ops == NULL)
		return -EPERM;
	else if (ops->usb_soc_init == NULL)
		return -EPERM;

	ops->rtk_usb = ops->usb_soc_init(node);

	return 0;
}

static inline int rtk_usb_soc_free(struct rtk_usb_ops *ops)
{
	if (ops == NULL)
		return -EPERM;
	else if (ops->usb_soc_free == NULL)
		return -EPERM;

	ops->usb_soc_free(&(ops->rtk_usb));

	return 0;
}

static inline int rtk_usb_port_suspend_resume(struct rtk_usb_ops *ops,
	    enum usb_port_num port_num, bool is_suspend)
{
	if (ops == NULL)
		return -EPERM;
	if (ops->usb_port_suspend_resume)
		ops->usb_port_suspend_resume(ops->rtk_usb,
			    port_num, is_suspend);

	return 0;
}

static inline int rtk_usb_set_hw_l4icg_on_off(struct rtk_usb_ops *ops,
	    enum usb_port_num port_num, bool on)
{
	if (ops == NULL)
		return -EPERM;
	if (ops->usb_set_hw_l4icg_on_off)
		ops->usb_set_hw_l4icg_on_off(ops->rtk_usb, port_num, on);

	return 0;
}

static inline int rtk_usb_iso_power_ctrl(struct rtk_usb_ops *ops,
	    bool power_on)
{
	if (ops == NULL)
		return -EPERM;
	if (ops->usb_iso_power_ctrl)
		ops->usb_iso_power_ctrl(ops->rtk_usb, power_on);

	return 0;
}

static inline void rtk_usb_set_charger_power(struct rtk_usb_ops *ops,
	    unsigned int val)
{
	if (ops == NULL)
		return;
	if (ops->usb_set_charger_power)
		ops->usb_set_charger_power(ops->rtk_usb, val);
}

/* For Type c */
static inline int rtk_type_c_init(struct rtk_usb_ops *ops)
{
	if (ops == NULL)
		return -EPERM;
	if (ops->type_c_init)
		ops->type_c_init(ops->rtk_usb);

	return 0;
}

static inline int rtk_type_c_plug_config(struct rtk_usb_ops *ops,
	    int dr_mode, int cc)
{
	if (ops == NULL)
		return -EPERM;
	if (ops->type_c_plug_config)
		ops->type_c_plug_config(ops->rtk_usb, dr_mode, cc);

	return 0;
}

/* For platform */
int rtk_usb_rtd119x_init(struct rtk_usb_ops *ops);
int rtk_usb_rtd129x_init(struct rtk_usb_ops *ops);
int rtk_usb_rtd139x_init(struct rtk_usb_ops *ops);
int rtk_usb_rtd16xx_init(struct rtk_usb_ops *ops);
int rtk_usb_rtd13xx_init(struct rtk_usb_ops *ops);
int rtk_usb_rtd16xxb_init(struct rtk_usb_ops *ops);
int rtk_usb_rtd1312c_init(struct rtk_usb_ops *ops);
int rtk_usb_rtd13xxd_init(struct rtk_usb_ops *ops);

#endif // __RTK_USB_H_INCLUDED_
