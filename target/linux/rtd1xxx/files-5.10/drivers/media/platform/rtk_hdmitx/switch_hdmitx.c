/*
 * switch_hdmitx.c - RTK hdmitx driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/switch.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/regmap.h>

#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>

#include <media/cec-notifier.h>
#include "hdmitx_dev.h"
#include "hdmitx_api.h"
#include "rtk_edid.h"
#include "hdmitx_scdc.h"
#include "hdmitx_trace.h"
#include "hdmitx_reg.h"

#define HDMI_SWITCH_NAME "hdmi"

void hdmitx_hpd_debounce(struct device *dev, unsigned char ms)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);
	struct hdmitx_switch_data *s_data; 

	s_data = tx_dev->s_data;
	gpiod_set_debounce(s_data->pin, ms*1000);
}

int hdmitx_switch_get_state(struct device *dev)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);

	return tx_dev->s_data->state;
}

static void hdmitx_switch_work_func(struct work_struct *work)
{
	struct hdmitx_switch_data *s_data = container_of(work, 
					struct hdmitx_switch_data, work);
	struct switch_dev *sdev;
	hdmitx_device_t *tx_dev;
	asoc_hdmi_t *tx_data;
	int hpd;
	int rxsense;
	int state;
	struct edid_information *hdmitx_edid_info;

	sdev = s_data->sdev;
	tx_dev = container_of(sdev, hdmitx_device_t, sdev);
	tx_data = (asoc_hdmi_t *)tx_dev->hdmi_data;
	hdmitx_edid_info = &tx_data->hdmitx_edid_info;

	if (tx_dev->frl) {
		if (tx_dev->frl->in_training) {
			dev_info(tx_dev->dev, "Skip switch_work in link training");
			return;
		}
	}

	hpd = gpiod_get_value(s_data->pin);

	if (tx_dev->rxsense_mode) {
		rxsense = hdmitx_get_rxsense(tx_dev);
		dev_info(sdev->dev, "HPD(%d) RxSense(%u)", hpd, rxsense);
	} else {
		rxsense = hpd;
		dev_info(sdev->dev, "HPD(%d)", hpd);
	}

	state = (rxsense == HDMI_RXSENSE_ON) && hpd;
	trace_hdmitx_hotplug_state(state, switch_get_state(sdev), hpd, rxsense);

	if (state != switch_get_state(sdev)) {

		if (state == 1) {
			hdmitx_get_sink_capability(tx_dev);

			kill_fasync(&tx_dev->fasync, SIGIO, POLL_IN);

			if (hdmitx_edid_info->scdc_capable & SCDC_RR_CAPABLE)
				enable_hdmitx_scdcrr(tx_dev, 1);

			set_i2s_output(tx_dev->dev, I2S_OUT_OFF);
			hdmitx_hpd_debounce(tx_dev->dev, 1);

			cec_notifier_set_phys_addr_from_edid(tx_dev->cec, hdmitx_edid_info->raw_edid);
		} else {
			kill_fasync(&tx_dev->fasync, SIGIO, POLL_HUP);

			enable_hdmitx_scdcrr(tx_dev, 0);
			set_i2s_output(tx_dev->dev, I2S_OUT_ON);
			hdmitx_hpd_debounce(tx_dev->dev, 30);
		}

		s_data->state = state;
		switch_set_state(sdev, s_data->state);
		dev_info(sdev->dev, "Switch state to %u", s_data->state);
	}

	s_data->hpd_state = hpd;

	if (hpd && tx_dev->rxsense_mode == RXSENSE_TIMER_MODE) {
		mod_timer(&s_data->rxsense_timer, jiffies + msecs_to_jiffies(30));
	} else if (!hpd) {
		hdmitx_reset_sink_capability(tx_data);
		cec_notifier_phys_addr_invalidate(tx_dev->cec);
	}

	wake_up_interruptible(&tx_dev->hpd_wait);
}

static void hdmitx_rxsense_timer_cb(struct timer_list *t)
{
	struct hdmitx_switch_data *s_data;
	struct switch_dev *sdev;
	hdmitx_device_t *tx_dev;
	int curr_rxsense;
	static int prev_rxsense = 0;

	s_data = from_timer(s_data, t, rxsense_timer);
	sdev = s_data->sdev;
	tx_dev = container_of(sdev, hdmitx_device_t, sdev);

	curr_rxsense = hdmitx_get_rxsense(tx_dev);

	if (curr_rxsense != prev_rxsense) {
		prev_rxsense = curr_rxsense;
		schedule_work(&s_data->work);
	}

	if (s_data->hpd_state)
		mod_timer(t, jiffies + msecs_to_jiffies(30));
}

static irqreturn_t hdmitx_switch_isr(int irq, void *data)
{
	struct hdmitx_switch_data *s_data = data;

	schedule_work(&s_data->work);
	pr_devel("hdmitx_switch_isr");

	return IRQ_HANDLED;
}

static irqreturn_t hdmitx_rxsense_isr(int irq, void *data)
{
	struct hdmitx_switch_data *s_data = data;
	unsigned int reg_val;
	unsigned int rxupdated;

	regmap_read(s_data->tx_dev->top_base, RXST, &reg_val);
	rxupdated = RXST_get_rxupdated(reg_val);

	if (rxupdated) {
		reg_val = reg_val & (~RXST_rxupdated_mask);
		regmap_write(s_data->tx_dev->top_base, RXST, reg_val);
		schedule_work(&s_data->work);
	}

	return IRQ_HANDLED;
}

int register_hdmitx_switchdev(hdmitx_device_t *tx_dev)
{
	struct switch_dev *sdev;
	struct device *dev = tx_dev->dev; 
	struct hdmitx_switch_data *s_data;
	int ret;
	int hpd_state;

	dev_dbg(dev, "register_hdmitx_switch");

	sdev = &tx_dev->sdev;
	sdev->name = HDMI_SWITCH_NAME;

	ret = switch_dev_register(sdev);
	if (ret < 0) {
		dev_err(dev, "err_register_switch");
		goto err_register_switch;
	}

	s_data = devm_kzalloc(dev, sizeof(struct hdmitx_switch_data), GFP_KERNEL);
	tx_dev->s_data = s_data;
	/* Get hotplug pin state */
	s_data->pin  = tx_dev->hpd_gpio;
	s_data->state = 0;
	s_data->hpd_state = 0;
	s_data->sdev = sdev;
	s_data->tx_dev = tx_dev;

	gpiod_direction_input(s_data->pin);
	/* debounce time as 30ms */
	gpiod_set_debounce(s_data->pin, 30*1000);
	hpd_state = gpiod_get_value(s_data->pin);

	/* Init hotplug work function and ISR */
	INIT_WORK(&s_data->work, hdmitx_switch_work_func);

	if (tx_dev->rxsense_mode == RXSENSE_TIMER_MODE)
		timer_setup(&s_data->rxsense_timer, hdmitx_rxsense_timer_cb, 0);

	s_data->irq = tx_dev->hpd_irq;
	s_data->hdmi_irq = tx_dev->hdmi_irq;

	if (hpd_state)
		schedule_work(&s_data->work);

	irq_set_irq_type(s_data->irq, IRQ_TYPE_EDGE_BOTH);
	ret = request_irq(s_data->irq, hdmitx_switch_isr, IRQF_SHARED, "switch_hdmitx", s_data);
	if (ret)
		dev_err(dev, "Cannot register hpd IRQ %d", s_data->irq);

	if ((tx_dev->rxsense_mode == RXSENSE_INTERRUPT_MODE) &&
			s_data->hdmi_irq) {

		ret = request_irq(s_data->hdmi_irq, hdmitx_rxsense_isr,
				IRQF_SHARED, "rxsense_isr", s_data);
		if (ret) {
			dev_err(dev, "Cannot register rxsense IRQ %d", s_data->hdmi_irq);
			tx_dev->rxsense_mode = RXSENSE_PASSIVE_MODE;
		} else {
			hdmitx_enable_rxsense_int(tx_dev);
			dev_info(tx_dev->dev, "Enable RxSense interrupt");
		}
	}

	goto end;

err_register_switch:
	dev_err(dev, "register_hdmitx_switch failed");
end:
	return ret;
}


void deregister_hdmitx_switchdev(hdmitx_device_t *tx_dev)
{
	return switch_dev_unregister(&tx_dev->sdev);
}


int show_hpd_status(struct device *dev, bool real_time)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);
	struct switch_dev *sdev;
	struct hdmitx_switch_data *s_data;

	sdev = &tx_dev->sdev;
	s_data = tx_dev->s_data; 

	if (real_time)
		return gpiod_get_value(s_data->pin);
	else
		return s_data->state;
}


int rtk_hdmitx_switch_suspend(hdmitx_device_t *tx_dev)
{
	struct switch_dev *sdev;
	struct hdmitx_switch_data *s_data;

	sdev = &tx_dev->sdev;
	s_data = tx_dev->s_data;

	/* hcy : dont need disable first */
	disable_irq(s_data->irq);
	dev_dbg(sdev->dev, "%s free irq=%x ", __func__, s_data->irq);

	/* Cancel work and wait for it to finish */
	cancel_work_sync(&s_data->work);

	if (tx_dev->rxsense_mode == RXSENSE_TIMER_MODE)
		del_timer_sync(&s_data->rxsense_timer);

	return 0;
}

int rtk_hdmitx_switch_resume(hdmitx_device_t *tx_dev)
{
	struct switch_dev *sdev;
	asoc_hdmi_t *tx_data;
	struct hdmitx_switch_data *s_data;
	struct edid_information *hdmitx_edid_info;
	int rxsense;

	sdev = &tx_dev->sdev;
	tx_data = tx_dev->hdmi_data; 
	hdmitx_edid_info = &tx_data->hdmitx_edid_info;
	s_data = tx_dev->s_data;
	/* set debounce time as 30ms */
	gpiod_set_debounce(s_data->pin, 30*1000);

	s_data->hpd_state = gpiod_get_value(s_data->pin);
	if (tx_dev->rxsense_mode) {
		rxsense = hdmitx_get_rxsense(tx_dev);
		s_data->state = s_data->hpd_state && rxsense;
		dev_info(sdev->dev, "HPD(%d) RxSense(%u)",
					s_data->hpd_state, rxsense);
	} else {
		s_data->state = s_data->hpd_state;
		dev_info(sdev->dev, "HPD(%d)", s_data->hpd_state);
	}

	if (s_data->state == 0) {
		hdmitx_reset_sink_capability(tx_data);
		switch_set_state(sdev, 0);
		dev_info(sdev->dev, "Switch state to 0");
	} else {
		if (!hdmitx_check_same_edid(tx_dev)) {
			hdmitx_reset_sink_capability(tx_data);
			hdmitx_get_sink_capability(tx_dev);
		}
		switch_set_state(sdev, 1);
		dev_info(sdev->dev, "Switch state to 1");

		if (hdmitx_edid_info->scdc_capable & SCDC_RR_CAPABLE)
			enable_hdmitx_scdcrr(tx_dev, 1);
	}
	enable_irq(s_data->irq);

	if ((tx_dev->rxsense_mode == RXSENSE_TIMER_MODE) && s_data->hpd_state)
		mod_timer(&s_data->rxsense_timer, jiffies + msecs_to_jiffies(300));

	return 0;
}

