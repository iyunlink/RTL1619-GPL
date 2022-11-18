// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017-2018,2020 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <soc/realtek/kernel-rpc.h>
#include <sound/asound.h>
#include <trace/events/rtk_rpc.h>
#include <rtk_rpc.h>
#include "snd-realtek.h"
#include <linux/dma-buf.h>
#include <linux/fdtable.h>
#include <ion_rtk_alloc.h>
#include <linux/pm_clock.h>
#include <linux/pm_runtime.h>

enum {
	ENUM_DT_AI_EN         = 0, /* this is used by acpu */
	ENUM_DT_AI_AIN        = 1, /* alway s set this bit if using AI */
	ENUM_DT_AI_ADC        = 2,
	ENUM_DT_AI_ANALOG_IN  = 3,
	ENUM_DT_AI_ADC_AMIC   = 4,
	ENUM_DT_AI_EARC_COMBO = 6,
};

static const char * const audio_in_names[] = {
	"adc", "analog-in", "adc-amic", "unknown", "earc-combo"
};


#define RTK_AUDIO_IN_I2S_STATUS       0x1
#define RTK_AUDIO_IN_I2S_PIN_SHARED   0x2
#define RTK_AUDIO_IN_I2S_MASTER       0x4

enum {
	ENUM_DT_AO_DAC    = 0,
	ENUM_DT_AO_I2S    = 1,
	ENUM_DT_AO_SPDIF  = 2,
	ENUM_DT_AO_HDMI   = 3,
	ENUM_DT_AO_GLOBAL = 4,
};


#define RTK_AUDIO_OUT_I2S_2_CHANNEL    0
#define RTK_AUDIO_OUT_I2S_8_CHANNEL    1

#define RTK_AUDIO_OUT_I2S_MODE_MASTER  0
#define RTK_AUDIO_OUT_I2S_MODE_SLAVE   1

static struct dma_buf *rtk_audio_notifier_alloc_dma_buf(void)
{
	struct dma_buf *dma_buf;

	dma_buf = ext_rtk_ion_alloc(4096, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);

	return dma_buf;
}

static int rtk_audio_notifier_copy_to_buf(void *data, int size, struct dma_buf *dma_buf)
{
	void *virt;

	dma_buf_begin_cpu_access(dma_buf, DMA_BIDIRECTIONAL);
	virt = dma_buf_vmap(dma_buf);
	if (!virt)
		return -ENOMEM;
	memcpy(virt, data, size);
	dma_buf_vunmap(dma_buf, virt);
	dma_buf_end_cpu_access(dma_buf, DMA_BIDIRECTIONAL);

	return 0;
}

static
int rtk_audio_notifier_send_rpc(struct device *dev, unsigned int command, void *data, int size)
{
	uint32_t rpc_ret;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	int off;
	int ret;

	dma_buf = rtk_audio_notifier_alloc_dma_buf();
	if (IS_ERR(dma_buf)) {
		dev_err(dev, "no dma-buf\n");
		return -ENOMEM;
	}

	attachment = dma_buf_attach(dma_buf, dev);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		dev_err(dev, "failed to attach dma-buf: %d\n", ret);
		goto put_dma_buf;
	}

	sgt = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		dev_err(dev, "failed to map attachment: %d\n", ret);
		goto detach_dma_buf;
	}

	if (sgt->nents != 1) {
		ret = -EINVAL;
		dev_err(dev, "scatter list not supportted\n");
		goto unmap_attachment;
	}

	dma_addr = sg_dma_address(sgt->sgl);
	if (!dma_addr) {
		ret = -ENOMEM;
		dev_err(dev, "invalid dma address\n");
		goto unmap_attachment;
	}

	ret = rtk_audio_notifier_copy_to_buf(data, size, dma_buf);
	if (ret) {
		dev_err(dev, "failed to copy data into dma_buf: %d\n", ret);
		goto unmap_attachment;
	}

	off = get_rpc_alignment_offset(size);
	if (send_rpc_command(RPC_AUDIO, command, CONVERT_FOR_AVCPU(dma_addr),
	    CONVERT_FOR_AVCPU(dma_addr + off), &rpc_ret)) {
		ret = -EINVAL;
		goto unmap_attachment;
	}

	ret = rpc_ret != S_OK ? -EINVAL : 0;

unmap_attachment:
	dma_buf_unmap_attachment(attachment, sgt, DMA_TO_DEVICE);
detach_dma_buf:
	dma_buf_detach(dma_buf, attachment);
put_dma_buf:
	dma_buf_put(dma_buf);
	return ret;
}

/* second compatible of ai devices */
static const struct of_device_id audio_in_matches[] = {
	{ .compatible = "adc",        .data = (void *)ENUM_DT_AI_ADC },
	{ .compatible = "analog-in",  .data = (void *)ENUM_DT_AI_ANALOG_IN },
	{ .compatible = "adc-amic",   .data = (void *)ENUM_DT_AI_ADC_AMIC },
	{ .compatible = "earc-combo", .data = (void *)ENUM_DT_AI_EARC_COMBO },
	{}
};

static int rtk_audio_in_init(struct device *dev)
{
	struct device_node *np = dev->of_node;
	unsigned int val;
	const struct of_device_id *match;
	struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS para = { 0 };
	unsigned int conf = 0;

	match = of_match_node(audio_in_matches, np);
	if (!match)
		return -EINVAL;
	val = (unsigned long)(match->data);

	if (of_property_match_string(np, "ai,type", "i2s") >= 0) {
		conf = RTK_AUDIO_IN_I2S_STATUS;

		if (of_find_property(np, "ai,i2s-pin-shared", NULL))
			conf |=	RTK_AUDIO_IN_I2S_PIN_SHARED;
		if (of_find_property(np, "ai,i2s-master", NULL))
			conf |=	RTK_AUDIO_IN_I2S_MASTER;

		dev_info(dev, "i2s mode=%s, pin=%s\n",
			(conf & RTK_AUDIO_IN_I2S_MASTER) ? "master" : "slave",
			(conf & RTK_AUDIO_IN_I2S_PIN_SHARED) ? "shared" : "independent");
	}

	para.type = htonl(ENUM_PRIVATEINFO_AIO_AI_INTERFACE_SWITCH_CONTROL);
	para.argateInfo[0] = htonl(BIT(ENUM_DT_AI_AIN) | BIT(val));
	para.argateInfo[1] = htonl(conf);
	dev_info(dev, "device_type=%s(%d)\n", audio_in_names[val - 2], val);

	return rtk_audio_notifier_send_rpc(dev, ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
			&para, sizeof(para));
}

static const struct of_device_id audio_out_matches[] = {
	{ .compatible = "dac",    .data = (void *)ENUM_DT_AO_DAC },
	{ .compatible = "i2s",    .data = (void *)ENUM_DT_AO_I2S },
	{ .compatible = "spdif",  .data = (void *)ENUM_DT_AO_SPDIF },
	{ .compatible = "hdmi",   .data = (void *)ENUM_DT_AO_HDMI },
	{ .compatible = "global", .data = (void *)ENUM_DT_AO_GLOBAL },
	{}
};

static int rtk_audio_out_init(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct device_node *child;
	unsigned int mask = 0;
	unsigned int channel = 0, mode = 0;
	struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS para = { 0 };

	for_each_child_of_node(np, child) {
		const struct of_device_id *match;
		u32 val;

		match = of_match_node(audio_out_matches, child);
		if (!match)
			continue;

		val = (unsigned long)(match->data);
		mask |= BIT(val);

		if (!of_device_is_available(child))
			continue;

		/* parse config for ao-i2s */
		if (!strcmp(match->compatible, "i2s")) {
			int val = 0;

			channel = RTK_AUDIO_OUT_I2S_2_CHANNEL;
			mode = RTK_AUDIO_OUT_I2S_MODE_MASTER;

			if (of_property_read_u32(child, "channel", &val))
				val = 0;
			if (val == 8)
				channel = RTK_AUDIO_OUT_I2S_8_CHANNEL;

			if (of_find_property(child, "slave-mode", NULL))
				mode = RTK_AUDIO_OUT_I2S_MODE_MASTER;

			dev_info(dev, "i2s: mode=%s, channel=%d\n",
				 mode ? "slave" : "master", channel ? 8 : 2);
		}

		mask &= ~BIT(val);
	}


	para.type = htonl(ENUM_PRIVATEINFO_AIO_AO_INTERFACE_SWITCH_CONTROL);
	para.argateInfo[0] = htonl(mask);
	para.argateInfo[1] = htonl(channel);
	para.argateInfo[2] = htonl(mode);

	if (mask & BIT(ENUM_DT_AO_DAC))
		dev_info(dev, "disabling dac\n");
	if (mask & BIT(ENUM_DT_AO_SPDIF))
		dev_info(dev, "disabling spdif\n");
	if (mask & BIT(ENUM_DT_AO_I2S))
		dev_info(dev, "disabling i2s\n");
	if (mask & BIT(ENUM_DT_AO_HDMI))
		dev_info(dev, "disabling hdmi\n");
	if (mask & BIT(ENUM_DT_AO_GLOBAL))
		dev_info(dev, "disabling global\n");

	return rtk_audio_notifier_send_rpc(dev, ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
			&para, sizeof(para));
}

static int rtk_audio_init(struct device *dev)
{
	return 0;
}

static const struct of_device_id rtk_audio_dev_ids[] = {
	{ .compatible = "realtek,audio-in",  .data = rtk_audio_in_init, },
	{ .compatible = "realtek,audio-out", .data = rtk_audio_out_init, },
	{ .compatible = "realtek,audio",     .data = rtk_audio_init, },
	{}
};

static void rtk_afw_notifier_of_get_gpios(struct device *dev)
{
	struct gpio_desc *gpio_desc;
	int num;
	int i;

	num = gpiod_count(dev, "audio");
	for (i = 0; i < num; i++) {
		gpio_desc = gpiod_get_index(dev, "audio", i, GPIOD_OUT_HIGH);

		if (IS_ERR(gpio_desc)) {
			dev_err(dev, "failed to get gpio[%d]\n", i);
			continue;
		}

		dev_info(dev, "request gpio%d output high\n", desc_to_gpio(gpio_desc));
		gpiod_put(gpio_desc);
	}

}

static int rtk_afw_notifier_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int (*init_func)(struct device *dev);
	int ret;

	init_func = of_device_get_match_data(dev);
	if (!init_func)
		return -EINVAL;

	rtk_afw_notifier_of_get_gpios(dev);

	ret = pm_clk_create(dev);
	if (ret) {
		dev_err(dev, "failed in pm_clk_create(): %d\n", ret);
		return ret;
	}
	of_pm_clk_add_clks(dev);

	dev_info(dev, "init with %pF\n", init_func);
	ret = init_func(dev);
	if (ret) {
		pm_clk_destroy(dev);
		return ret;
	}

	pm_runtime_set_suspended(dev);

	pm_runtime_enable(dev);

	pm_runtime_get_sync(dev);

	return 0;
}

static struct platform_driver rtk_afw_notifier_drv = {
	.driver = {
		.name   = "rtk-afw-notifier",
		.of_match_table = rtk_audio_dev_ids,
	},
	.probe = rtk_afw_notifier_probe,
};

static int __init rtk_afw_notifier_init(void)
{
	return platform_driver_register(&rtk_afw_notifier_drv);
}
late_initcall(rtk_afw_notifier_init);

MODULE_LICENSE("GPL v2");
