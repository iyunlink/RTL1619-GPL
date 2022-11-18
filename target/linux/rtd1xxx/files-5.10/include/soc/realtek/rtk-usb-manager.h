/*
 * rtk-usb-manager.h
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#ifndef __RTK_USB_MANAGER_H_INCLUDED_
#define __RTK_USB_MANAGER_H_INCLUDED_


int rtk_usb_manager_is_iso_mode(struct device *usb_dev);
int rtk_usb_manager_schedule_work(struct device *usb_dev,
	    struct work_struct *work);
int rtk_usb_type_c_init(struct device *type_c_dev);
int rtk_usb_type_c_plug_config(struct device *type_c_dev,
	    int dr_mode, int cc);

int rtk_usb_init_port_power_on_off(struct device *usb_dev, bool on);
int rtk_usb_remove_port_power_on_off(struct device *usb_dev, bool on);

int rtk_usb_port_power_on_off(struct device *usb_dev, bool on);

#endif // __RTK_USB_MANAGER_H_INCLUDED_
