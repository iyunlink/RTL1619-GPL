/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <soc/realtek/rtk_ipc_shm.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

struct class *ve3_log_class;
int ve3_uart_enable = 0;

#define MODULE_NAME	"ve3_uart"

struct task_struct *ve3_uart_kthread;
wait_queue_head_t ve3_uart_waitQueue;


static void __iomem *hevc_reg;
#define UFIFO_STATUS 0x18
#define UFIFO_RD 0x14
char uart_tx[1024];
int tx_num = 0;


static int ve3_uart_thread(void * p)
{
	int status;
	int cnt;
	char tx_char;

	while (1) {
		if (wait_event_interruptible(ve3_uart_waitQueue, ve3_uart_enable || kthread_should_stop())) {
			pr_notice("%s got signal or should stop...\n", current->comm);
			continue;
		}

		if (kthread_should_stop()) {
			pr_notice("%s exit...\n", current->comm);
			break;
		}
		memset(uart_tx, 0, 1024);
		tx_num = 0;
		while (ve3_uart_enable) {
			status = readl(hevc_reg + UFIFO_STATUS);
			cnt = (status >> 16) & 0x1f;
			while (cnt > 0) {
				tx_char = readl(hevc_reg + UFIFO_RD);
				if (tx_char == '\n' || tx_num == 1023) {
					uart_tx[tx_num] = tx_char;
					pr_err("[VE3 LOG] %s \n", uart_tx);
					memset(uart_tx, 0, 1024);
					tx_num = 0;
				} else {
					uart_tx[tx_num] = tx_char;
					tx_num++;
				}
				cnt--;
			}
			msleep(10);
		}
	}
	return 0;
}


static ssize_t ve3_log_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ve3_uart_enable);
}

ssize_t ve3_log_store(struct class *class, struct class_attribute *attr,
			const char *buf, size_t count)
{
	long val;
	int ret = kstrtol(buf, 10, &val);
	if (ret < 0)
		return -ENOMEM;

	if (val == 0) {
		ve3_uart_enable = 0;
	} else if (val == 1){
		ve3_uart_enable = 1;
		wake_up_interruptible(&ve3_uart_waitQueue);
	}

	return count;
}
static CLASS_ATTR_RW(ve3_log);

static int rtk_ve3_uart_probe(struct platform_device *pdev)
{
	int ret = 0;
	int j = 0;
	static struct clk *ve3_clk;
	const u32 *prop;
	int size = 0;

	ve3_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ve3_clk)) {
		dev_err(&pdev->dev, "ve3 clock source missing or invalid\n");
		return PTR_ERR(ve3_clk);
	}
	if (!(__clk_is_enabled(ve3_clk))) {
		dev_info(&pdev->dev, "ve3 clk is off, do not probe ve3_uart driver\n");
		devm_clk_put(&pdev->dev, ve3_clk);
		return 0;
	}
	hevc_reg = of_iomap(pdev->dev.of_node, 0);
	if (!hevc_reg) {
		pr_err("cannot get hevc reg\n");
		return -EINVAL;
	}

	ve3_log_class = class_create(THIS_MODULE, "ve3_log");
	if (ve3_log_class == NULL)
		pr_err("failed to create ve3_log attribute!\n");
	ret = class_create_file(ve3_log_class, &class_attr_ve3_log);

	prop = of_get_property(pdev->dev.of_node, "log_enable", &size);
	if (prop) {
		ve3_uart_enable = of_read_number(prop, 1);
	} else {
		ve3_uart_enable = 0;
	}

	init_waitqueue_head(&ve3_uart_waitQueue);
	ve3_uart_kthread = kthread_run(ve3_uart_thread, (void *)&j, "ve3_uart_thread");

	clk_prepare_enable(ve3_clk);

	dev_info(&pdev->dev, "probed.\n");

	return ret;
}

static int rtk_ve3_uart_remove(struct platform_device *pdev)
{
	class_destroy(ve3_log_class);

	return 0;
}

static struct of_device_id rtk_ve3_uart_ids[] = {
	{ .compatible = "realtek,rtk-ve3-uart" },
	{ /* Sentinel */ },
};

static struct platform_driver rtk_ve3_uart_driver = {
        .probe = rtk_ve3_uart_probe,
        .remove = rtk_ve3_uart_remove,
        .driver = {
                .name = MODULE_NAME,
                .bus = &platform_bus_type,
                .owner = THIS_MODULE,
                .of_match_table = of_match_ptr(rtk_ve3_uart_ids),
        },
};


static int rtk_ve3_uart_init(void)
{
	return platform_driver_register(&rtk_ve3_uart_driver);
}
subsys_initcall(rtk_ve3_uart_init);

//module_platform_driver(rtk_ve3_uart_driver);

MODULE_DESCRIPTION("Realtek VE3 Uart driver");
MODULE_LICENSE("GPL");

