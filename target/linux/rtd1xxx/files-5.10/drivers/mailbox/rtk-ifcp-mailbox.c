// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/atomic.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mailbox/rtk-ifcp-message.h>
#include <soc/realtek/rtk_ifcp.h>

struct rtk_ifcp_mb_device {
	struct mbox_controller mbox;
	struct device *dev;
	void *base;
	int irq;
	struct rtk_ifcp_message msg;

	spinlock_t lock;
};

#define CAS_MB_CSR_REE       0x000
#define CAS_MB_REE_DATA      0x004
#define CAS_MB_REE_INTCTL    0x008
#define CAS_MB_MISC          0x200

#define MBOX_POLLING_MS 100

#define  __check_msg_ptr(_p) \
({ \
	BUILD_BUG_ON(!__same_type((_p), (struct rtk_ifcp_message *)(0))); \
	(_p); \
})

static unsigned int rtk_ifcp_mb_read_csr(struct rtk_ifcp_mb_device *mbdev)
{
	return readl(mbdev->base + CAS_MB_CSR_REE);
}

static int mb_tx_is_empty(unsigned int val)
{
	return (val & 0x200) != 0;
}

static int mb_rx_is_full(unsigned int val)
{
	return (val & 0x100) != 0;
}

static int mb_rx_get_length(unsigned int val)
{
	return (val & 0x3f) + 1;
}

static int mb_length_is_valid(int len)
{
	return len > 0 && len <= 0x40;
}

static void rtk_ifcp_mb_tx_set_length(struct rtk_ifcp_mb_device *mbdev, int len)
{
	writel(len - 1, mbdev->base + CAS_MB_CSR_REE);
}

static void rtk_ifcp_mb_tx_send_data(struct rtk_ifcp_mb_device *mbdev, unsigned int data)
{
	writel(data, mbdev->base + CAS_MB_REE_DATA);
}

static unsigned int rtk_ifcp_mb_rx_get_data(struct rtk_ifcp_mb_device *mbdev)
{
	return readl(mbdev->base + CAS_MB_REE_DATA);
}

#define MB_INTC_EN          0x1
#define MB_INTC_TX_EMPTY    0x2
#define MB_INTC_RX_FULL     0x4
#define MB_INTC_MASK        0x6

static void rtk_ifcp_mb_intc_set(struct rtk_ifcp_mb_device *mbdev, unsigned int bits)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&mbdev->lock, flags);

	val = readl(mbdev->base + CAS_MB_REE_INTCTL) & ~MB_INTC_EN;
	val |= bits;
	if (val & MB_INTC_MASK)
		val |= MB_INTC_EN;
	writel(val, mbdev->base + CAS_MB_REE_INTCTL);

	spin_unlock_irqrestore(&mbdev->lock, flags);
}

static void rtk_ifcp_mb_intc_clear(struct rtk_ifcp_mb_device *mbdev, unsigned int bits)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&mbdev->lock, flags);

	val = readl(mbdev->base + CAS_MB_REE_INTCTL) & ~MB_INTC_EN;
	val &= ~bits;
	if (val & MB_INTC_MASK)
		val |= MB_INTC_EN;
	writel(val, mbdev->base + CAS_MB_REE_INTCTL);

	spin_unlock_irqrestore(&mbdev->lock, flags);
}

static irqreturn_t rtk_ifcp_mb_interrupt(int irq, void *dev_id)
{
	struct rtk_ifcp_mb_device *mbdev = dev_id;
	struct mbox_chan *chan = &mbdev->mbox.chans[0];
	int i;
	unsigned int raw;

	raw = rtk_ifcp_mb_read_csr(mbdev);

	if (mb_tx_is_empty(raw)) {
		rtk_ifcp_mb_intc_clear(mbdev, MB_INTC_TX_EMPTY);
		mbox_chan_txdone(chan, 0);
	}

	if (mb_rx_is_full(raw)) {
		struct rtk_ifcp_message msg = { 0 };

		msg.len = mb_rx_get_length(raw);
		if (!msg.len) {
			pr_err("%s: mb_rx_is_full, invalid len=0\n", __func__);
			goto rx_done;
		}
		for (i = 0; i < msg.len; i++)
			msg.data[i] = rtk_ifcp_mb_rx_get_data(mbdev);

		print_hex_dump_debug("rx_data: ", DUMP_PREFIX_NONE, 16, 4, msg.data, msg.len * 4, false);
		mbox_chan_received_data(chan, __check_msg_ptr(&msg));
	}

rx_done:
	return IRQ_HANDLED;
}

static int rtk_ifcp_mb_send_data(struct mbox_chan *chan, void *data)
{
	struct rtk_ifcp_mb_device *mbdev = chan->con_priv;
	struct rtk_ifcp_message *msg = data;
	int i;
	unsigned int raw;

	if (!mb_length_is_valid(msg->len))
		return -EINVAL;

	raw = rtk_ifcp_mb_read_csr(mbdev);
	if (!mb_tx_is_empty(raw)) {
		rtk_ifcp_mb_intc_set(mbdev, MB_INTC_TX_EMPTY);
		return -EBUSY;
	}

	print_hex_dump_debug("tx_data: ", DUMP_PREFIX_NONE, 16, 4, msg->data, msg->len * 4, false);
	rtk_ifcp_mb_tx_set_length(mbdev, msg->len);
	for (i = 0; i < msg->len; i++)
		rtk_ifcp_mb_tx_send_data(mbdev, msg->data[i]);

	rtk_ifcp_mb_intc_set(mbdev, MB_INTC_TX_EMPTY);

	return 0;
}

static int rtk_ifcp_mb_startup(struct mbox_chan *chan)
{
	struct rtk_ifcp_mb_device *mbdev = chan->con_priv;

	rtk_ifcp_mb_intc_set(mbdev, MB_INTC_RX_FULL);

	return 0;
}

static void rtk_ifcp_mb_shutdown(struct mbox_chan *chan)
{
	struct rtk_ifcp_mb_device *mbdev = chan->con_priv;

	rtk_ifcp_mb_intc_clear(mbdev, MB_INTC_TX_EMPTY | MB_INTC_RX_FULL);
}

static const struct mbox_chan_ops rtk_ifcp_mb_chan_ops = {
	.send_data = rtk_ifcp_mb_send_data,
	.startup   = rtk_ifcp_mb_startup,
	.shutdown  = rtk_ifcp_mb_shutdown,
};

static int rtk_ifcp_mb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rtk_ifcp_mb_device *mbdev;
	struct resource res;
	int ret;
	struct mbox_chan *chans;

	mbdev = devm_kzalloc(dev, sizeof(*mbdev), GFP_KERNEL);
	if (!mbdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, mbdev);
	mbdev->dev = dev;

	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(dev, "failed to get recource: %d\n", ret);
		return -ENODEV;
	}

	mbdev->base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!mbdev->base)
		return -ENOMEM;

	chans = devm_kcalloc(dev, 1, sizeof(*chans), GFP_KERNEL);
	if (!chans)
		return -ENOMEM;
	chans[0].con_priv = mbdev;

	mbdev->irq = platform_get_irq(pdev, 0);
	if (mbdev->irq < 0) {
		ret = mbdev->irq;
		dev_err(dev, "failed to get irq: %d\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(dev, mbdev->irq, NULL, rtk_ifcp_mb_interrupt, IRQF_ONESHOT,
			       dev_name(dev), mbdev);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		return ret;
	}

	spin_lock_init(&mbdev->lock);
	mbdev->mbox.dev        = dev;
	mbdev->mbox.txdone_irq = true;
	mbdev->mbox.ops        = &rtk_ifcp_mb_chan_ops;
	mbdev->mbox.chans      = chans;
	mbdev->mbox.num_chans  = 1;

	ret = devm_mbox_controller_register(&pdev->dev, &mbdev->mbox);
	if (ret) {
		dev_err(dev, "failed to register mailbox controller: %d\n", ret);
		return ret;
	}

	rtk_ifcp_reset();

	return 0;
}

static int rtk_ifcp_mb_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}


static const struct of_device_id rtk_ifcp_mb_ids[] = {
	{ .compatible = "realtek,ifcp-mailbox" },
	{}
};

static struct platform_driver rtk_ifcp_mb_driver = {
	.probe  = rtk_ifcp_mb_probe,
	.remove = rtk_ifcp_mb_remove,
	.driver = {
		.name           = "rtk-ifcp-mb",
		.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(rtk_ifcp_mb_ids),
	},
};
module_platform_driver(rtk_ifcp_mb_driver);

MODULE_DESCRIPTION("Realtek IFCP Mailbox");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-ifcp-mailbox");


