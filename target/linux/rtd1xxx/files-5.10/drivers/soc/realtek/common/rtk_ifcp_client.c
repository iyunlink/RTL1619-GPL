// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mailbox_client.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/mailbox/rtk-ifcp-message.h>
#include <soc/realtek/rtk_ifcp.h>
#include <soc/realtek/uapi/rtk_ifcp_client.h>

#define MSG_MAX_SIZE         (IFCP_MAX_DATA)
#define MSG_MAX_SIZE_BYTE    (MSG_MAX_SIZE * 4)
#define RESP_FIFO_SIZE       (MSG_MAX_SIZE * 2)

#define ACK    0x0a
#define NACK   0xba

struct rtk_ifcp_client_data {
	struct device *dev;
	struct mbox_client client;
	struct mbox_chan *chan;
	struct miscdevice mdev;

	struct completion done;
	struct kfifo fifo;
	wait_queue_head_t wait;
};

static void rtk_ifcp_client_rx_callback(struct mbox_client *client, void *message)
{
	struct rtk_ifcp_client_data *mbc = container_of(client, struct rtk_ifcp_client_data, client);
	struct rtk_ifcp_message *msg = message;
	int len;

	kfifo_reset(&mbc->fifo);

	len = kfifo_in(&mbc->fifo, msg->data, msg->len * 4);
	if (len != (msg->len * 4))
		pr_err("%s: excepted len = %d, msg data size %d (bytes)\n", __func__, len, msg->len * 4);
	wake_up(&mbc->wait);
}

static void rtk_ifcp_client_tx_done(struct mbox_client *client, void *message, int r)
{
	struct rtk_ifcp_client_data *mbc = container_of(client, struct rtk_ifcp_client_data, client);

	complete(&mbc->done);
}

static int rtk_ifcp_client_send_msg(struct rtk_ifcp_client_data *mbc, struct rtk_ifcp_message *msg, int blocking)
{
	unsigned long timeout = msecs_to_jiffies(500);
	int ret;

	reinit_completion(&mbc->done);
	ret = mbox_send_message(mbc->chan, msg);
	if (ret < 0) {
		dev_err(mbc->dev, "failed to send message: %d\n", ret);
		return ret;
	}

	if (!blocking)
		return 0;

	timeout = wait_for_completion_timeout(&mbc->done, timeout);
	if (!timeout) {
		dev_err(mbc->dev, "send message timeout\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static ssize_t rtk_ifcp_client_read(struct file *filep, char __user *user_buf,
				 size_t count, loff_t *pos)
{
	struct rtk_ifcp_client_data *mbc = container_of(filep->private_data, struct rtk_ifcp_client_data, mdev);
	int ret;
	unsigned int copied;

	if (!IS_ALIGNED(count, 4) ||  count > MSG_MAX_SIZE_BYTE)
		return -EIO;

	do {
		if (kfifo_is_empty(&mbc->fifo)) {
			if (filep->f_flags & O_NONBLOCK)
				return -EAGAIN;

			ret = wait_event_interruptible(mbc->wait,
					!kfifo_is_empty(&mbc->fifo));
			if (ret)
				return ret;
		}

		ret = kfifo_to_user(&mbc->fifo, user_buf, count, &copied);
		if (ret)
			return ret;
	} while (copied == 0);

	return copied;
}

static ssize_t rtk_ifcp_client_write(struct file *filep, const char __user *user_buf,
				  size_t count, loff_t *pos)
{
	struct rtk_ifcp_client_data *mbc = container_of(filep->private_data, struct rtk_ifcp_client_data, mdev);
	char buf[MSG_MAX_SIZE_BYTE] = {0};
	unsigned int *p = (void *)buf;
	int i;
	int ret;
	struct rtk_ifcp_message msg = {0};

	if (count == 0 || !IS_ALIGNED(count, 4) || count > MSG_MAX_SIZE_BYTE)
		return -EIO;

	ret = copy_from_user(buf, user_buf, count);
	if (ret)
		return ret;

	msg.len = count / 4;
	for (i = 0; i < msg.len; i++)
		msg.data[i] = p[i];
	ret = rtk_ifcp_client_send_msg(mbc, &msg, !(filep->f_flags & O_NONBLOCK));

	return ret ? ret : count;
}

#define MBOX_RESET_RFIFO            _IO(MBOX_ID, 0x11)

static int rtk_ifcp_client_wait_for_ack(struct rtk_ifcp_client_data *mbc)
{
	int ret;
	unsigned int val;

	ret = wait_event_interruptible(mbc->wait, !kfifo_is_empty(&mbc->fifo));
	if (ret)
		return ret;

	ret = kfifo_out(&mbc->fifo, &val, 4);
	if (ret != 4 && val != ACK)
		return -EINVAL;
	return 0;
}

static int rtk_ifcp_client_get_cw(struct rtk_ifcp_client_data *mbc, void __user *user_arg)
{
	CMD_MBOX_GET_CW data;
	struct rtk_ifcp_message msg = {0};
	int ret;

	if (copy_from_user(&data, user_arg, sizeof(data)))
		return -EFAULT;

	kfifo_reset(&mbc->fifo);

	msg.len = 1;
	WARN(1, "not implemented");
	ret = rtk_ifcp_client_send_msg(mbc, &msg, 1);
	if (ret)
		return ret;

	ret = wait_event_interruptible(mbc->wait, !kfifo_is_empty(&mbc->fifo));
	if (ret)
		return ret;

	data.key_size = kfifo_len(&mbc->fifo);
	if (data.key_size > 16)
		return -EINVAL;

	ret = kfifo_out(&mbc->fifo, data.key, data.key_size);
	WARN_ON(ret != data.key_size);

	if (copy_to_user(user_arg, &data, sizeof(data)))
		return -EFAULT;
	return 0;
}

static int rtk_ifcp_client_get_biv(struct rtk_ifcp_client_data *mbc, void __user *user_arg)
{
	CMD_MBOX_GET_BIV data;
	struct rtk_ifcp_message msg = {0};
	int len;
	int ret;

	if (copy_from_user(&data, user_arg, sizeof(data)))
		return -EFAULT;

	kfifo_reset(&mbc->fifo);

	msg.len = 1;
	WARN(1, "not implemented");
	ret = rtk_ifcp_client_send_msg(mbc, &msg, 1);
	if (ret)
		return ret;

	ret = wait_event_interruptible(mbc->wait, !kfifo_is_empty(&mbc->fifo));
	if (ret)
		return ret;

	len = kfifo_len(&mbc->fifo);
	if (len > 16)
		return -EINVAL;

	WARN_ON(!IS_ALIGNED(len, 4));

	data.size = len / 4;
	ret = kfifo_out(&mbc->fifo, data.data, len);
	WARN_ON(len != ret);

	if (copy_to_user(user_arg, &data, sizeof(data)))
		return -EFAULT;
	return 0;
}

static int rtk_ifcp_client_set_biv(struct rtk_ifcp_client_data *mbc, void __user *user_arg)
{
	CMD_MBOX_SET_BIV data;
	struct rtk_ifcp_message msg = {0};
	int ret;

	if (copy_from_user(&data, user_arg, sizeof(data)))
		return -EFAULT;

	kfifo_reset(&mbc->fifo);

	WARN(1, "not implemented");
	ret = rtk_ifcp_client_send_msg(mbc, &msg, 1);
	if (ret)
		return ret;

	ret = wait_event_interruptible(mbc->wait, !kfifo_is_empty(&mbc->fifo));
	if (ret)
		return ret;

	return rtk_ifcp_client_wait_for_ack(mbc);
}


static int rtk_ifcp_client_reset_target(struct rtk_ifcp_client_data *mbc, int target)
{
	struct rtk_ifcp_message msg = {0};
	int ret;

	kfifo_reset(&mbc->fifo);

	WARN(1, "not implemented");
	ret = rtk_ifcp_client_send_msg(mbc, &msg, 1);
	if (ret)
		return ret;

	return rtk_ifcp_client_wait_for_ack(mbc);
}

static int rtk_ifcp_client_write_otp(struct rtk_ifcp_client_data *mbc, void __user *user_arg)
{
	return -ENOIOCTLCMD;
}

static int rtk_ifcp_client_read_otp(struct rtk_ifcp_client_data *mbc, void __user *user_arg)
{
	return -ENOIOCTLCMD;
}

static int rtk_ifcp_client_descramble_packet(struct rtk_ifcp_client_data *mbc, void __user *user_arg)
{
	return -ENOIOCTLCMD;
}

static long rtk_ifcp_client_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct rtk_ifcp_client_data *mbc = container_of(filp->private_data, struct rtk_ifcp_client_data, mdev);
	void __user *user_arg = (void __user *)arg;

	switch (cmd) {
	case MBOX_RESET_RFIFO:
		kfifo_reset(&mbc->fifo);
		break;
	case MBOX_RESET_IFCP:
		rtk_ifcp_reset();
		break;
	case MBOX_GET_CW:
		return rtk_ifcp_client_get_cw(mbc, user_arg);
	case MBOX_GET_BIV:
		return rtk_ifcp_client_get_biv(mbc, user_arg);
	case MBOX_SET_BIV:
		return rtk_ifcp_client_set_biv(mbc, user_arg);
	case MBOX_WRITE_OTP:
		return rtk_ifcp_client_write_otp(mbc, user_arg);
	case MBOX_READ_OTP:
		return rtk_ifcp_client_read_otp(mbc, user_arg);
	case MBOX_RESET_OTP:
		return rtk_ifcp_client_reset_target(mbc, 1);
	case MBOX_DESCRAMBLE_PACKET:
		return rtk_ifcp_client_descramble_packet(mbc, user_arg);
	default:
		return -ENOIOCTLCMD;
	};
	return 0;
}

static const struct file_operations rtk_ifcp_client_fops = {
	.read           = rtk_ifcp_client_read,
	.write          = rtk_ifcp_client_write,
	.unlocked_ioctl = rtk_ifcp_client_ioctl,
	.compat_ioctl   = compat_ptr_ioctl,
};

static int rtk_ifcp_client_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_ifcp_client_data *mbc;
	struct mbox_client *client;
	struct miscdevice *mdev;
	int ret;

	mbc = devm_kzalloc(dev, sizeof(*mbc), GFP_KERNEL);
	if (!mbc)
		return -ENOMEM;
	mbc->dev = dev;
	init_completion(&mbc->done);

	if (kfifo_alloc(&mbc->fifo, RESP_FIFO_SIZE * sizeof(unsigned int), GFP_KERNEL))
		return -ENOMEM;

	init_waitqueue_head(&mbc->wait);

	client = &mbc->client;
	client->dev         = &pdev->dev;
	client->rx_callback = rtk_ifcp_client_rx_callback;
	client->tx_done     = rtk_ifcp_client_tx_done;
	mbc->chan = mbox_request_channel(client, 0);
	if (IS_ERR(mbc->chan)) {
		ret = PTR_ERR(mbc->chan);
		dev_err(dev, "failed to request channel: %d\n", ret);
		return ret;
	}

	mdev = &mbc->mdev;
	mdev->parent = dev;
	mdev->minor  = MISC_DYNAMIC_MINOR;
	mdev->fops   = &rtk_ifcp_client_fops;
	mdev->name   = "ifcp_mailbox";
	ret = misc_register(mdev);
	if (ret) {
		dev_err(dev, "failed to register misc device: %d\n", ret);
		goto error;
	}

	platform_set_drvdata(pdev, mbc);
	return 0;
error:
	mbox_free_channel(mbc->chan);
	return ret;
}

static int rtk_ifcp_client_remove(struct platform_device *pdev)
{
	struct rtk_ifcp_client_data *mbc = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	misc_deregister(&mbc->mdev);
	mbox_free_channel(mbc->chan);
	kfifo_free(&mbc->fifo);
	return 0;
}


static const struct of_device_id rtk_ifcp_client_match[] = {
	{ .compatible = "realtek,ifcp-client", },
	{}
};

static struct platform_driver rtk_ifcp_client_driver = {
	.probe    = rtk_ifcp_client_probe,
	.remove   = rtk_ifcp_client_remove,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "rtk-ifcp-client",
		.of_match_table = of_match_ptr(rtk_ifcp_client_match),
	},
};
module_platform_driver(rtk_ifcp_client_driver);

MODULE_DESCRIPTION("Realtek IFCP Client");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-ifcp-client");

