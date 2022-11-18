/*
 * rtk_mute_gpio.c - RTK hdmitx driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/timer.h>

#include "hdmitx_dev.h"

#define AUDIO_MUTE_ON	0
#define AUDIO_MUTE_OFF	1

static struct gpio_desc *audio_mute_gpio;
static struct gpio_desc *i2s_ctrl_gpio;


static int set_mute_gpio(struct device *dev, int gpio_value)
{
	int ret = 0;

	audio_mute_gpio = devm_gpiod_get(dev, "audio-mute", GPIOD_ASIS);
	if (IS_ERR(audio_mute_gpio)) {
		ret = -1;
	} else {
		dev_info(dev, "Set audio mute GPIO (%u)", gpio_value);
		gpiod_direction_output(audio_mute_gpio, gpio_value);
		devm_gpiod_put(dev, audio_mute_gpio);
	}

	return ret;
}

static void mute_gpio_timer_callback(struct timer_list *t)
{
	hdmitx_device_t *tx_dev = from_timer(tx_dev, t, mute_gpio_timer);
	struct device *dev = tx_dev->dev;

	set_mute_gpio(dev, AUDIO_MUTE_OFF);
}

void set_mute_gpio_pulse(struct device *dev)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);

	if (set_mute_gpio(dev, AUDIO_MUTE_ON) < 0)
		return;

	/* Disable mute after 3 sec */
	mod_timer(&tx_dev->mute_gpio_timer, jiffies + msecs_to_jiffies(3000));
}

/**
 * set_i2s_output - Control I2S audio output DAC
 */
void set_i2s_output(struct device *dev, int gpio_value)
{

	i2s_ctrl_gpio = devm_gpiod_get(dev, "i2s-ctrl", GPIOD_ASIS);
	if (!IS_ERR(i2s_ctrl_gpio)) {
		dev_info(dev, "Set i2s_ctrl_gpio=%u", gpio_value);
		gpiod_direction_output(i2s_ctrl_gpio, gpio_value);
		devm_gpiod_put(dev, i2s_ctrl_gpio);
	}
}

/**
 * setup_mute_gpio - Initial audio DAC mute pin control
 */
void setup_mute_gpio(struct device *dev)
{
	struct device_node *hdmitx_dev_node;
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);

	dev_info(dev, "[%s]", __func__);
	
	hdmitx_dev_node = dev->of_node;

	set_mute_gpio(dev, AUDIO_MUTE_OFF);

	timer_setup(&tx_dev->mute_gpio_timer, mute_gpio_timer_callback, 0);

	return;
}
