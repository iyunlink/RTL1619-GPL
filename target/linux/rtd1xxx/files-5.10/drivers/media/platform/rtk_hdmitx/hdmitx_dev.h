/*
 * hdmitx_dev.h - RTK hdmitx driver header file
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __HDMITX_DEV_H__
#define __HDMITX_DEV_H__

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/switch.h>
#include <linux/miscdevice.h>
#include "hdmitx.h"

typedef struct {
	char *name;
	struct miscdevice miscdev;
	struct switch_dev sdev;
	struct device *dev;
	void __iomem *reg_base;
	void __iomem *pll_base;
	void __iomem *top_base;
	struct reset_control *reset_hdmi;
	struct clk *clk_hdmi;
	struct gpio_desc *hpd_gpio;
	struct gpio_desc *ctrl_5v_gpio;
	unsigned int hpd_irq;
	unsigned int hdmi_irq;
	enum HDMI_RXSENSE_MODE rxsense_mode;
	asoc_hdmi_t *hdmi_data;
	struct hdmitx_scdc_data	*tx_scdc; 
	struct hdmitx_switch_data *s_data;
	struct rtk_hdmitx_frl *frl;
	struct fasync_struct *fasync;
	wait_queue_head_t hpd_wait;
	struct timer_list mute_gpio_timer;
	struct cec_notifier *cec;
} hdmitx_device_t;

struct hdmitx_switch_data {
	int			state;
	int			hpd_state;
	unsigned int irq;
	unsigned int hdmi_irq;
	struct gpio_desc	*pin;
	struct work_struct	work;
	struct timer_list rxsense_timer;
	struct switch_dev   *sdev;
	hdmitx_device_t     *tx_dev;
};

#define to_hdmitx_device(x)  container_of(x, hdmitx_device_t, dev)



extern int register_hdmitx_switchdev(hdmitx_device_t *device);
extern void deregister_hdmitx_switchdev(hdmitx_device_t *device);
extern int show_hpd_status(struct device *dev, bool real_time);
extern int rtk_hdmitx_switch_suspend(hdmitx_device_t *tx_dev);
extern int rtk_hdmitx_switch_resume(hdmitx_device_t *tx_dev);

int hdmitx_get_raw_edid(struct device *dev, unsigned char *edid);

#endif /* __HDMITX_DEV__H__ */

