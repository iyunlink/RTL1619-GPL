/*
 * hdmitx_core.c - RTK hdmitx driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/gpio/consumer.h>
#include <linux/reset.h>
#include <linux/clk.h> /* clk_get */
#include <linux/clk-provider.h>
#include <linux/delay.h>

#include <media/cec-notifier.h>

#include "hdmitx.h"
#include "hdmitx_dev.h"
#include "hdmitx_api.h"
#include "hdmitx_rpc.h"
#include "hdmitx_scdc.h"
#include "crt_reg.h"
#include "hdmitx_reg.h"
#include "hdmitop_reg.h"
#include "compat_hdmitx.h"


#define CREATE_TRACE_POINTS
#include "hdmitx_trace.h"

static int __init rtk_hdmi_init(void);
static void __exit rtk_hdmi_exit(void);


unsigned int hdmipll_read32(struct device *dev, unsigned int offset)
{
	unsigned int reg_val;
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);	

	if (offset > SYS_CLOCK_ENABLE_SECURE_DMY_1_DBG) {
		dev_err(dev, "%s invaild offset", __func__);
		BUG_ON(1);
		return 0;
	}

	regmap_read(tx_dev->pll_base, offset, &reg_val);
	return reg_val;
}
void hdmipll_write32(struct device *dev, unsigned int offset, unsigned int value)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);	

	if (offset > SYS_CLOCK_ENABLE_SECURE_DMY_1_DBG) {
		dev_err(dev, "%s invaild offset", __func__);
		BUG_ON(1);
		return;
	}

	regmap_write(tx_dev->pll_base, offset, value);
}

void hdmipll_mask32(struct device *dev, unsigned int offset, unsigned int andMask,
	unsigned int orMask)
{
	hdmipll_write32(dev, offset, ((hdmipll_read32(dev, offset) & (andMask)) | (orMask)));
}

unsigned int hdmitx_read32(struct device *dev, unsigned int offset)
{
	unsigned int reg_val;
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);	

	if (offset > HDMI_FRL_TRAINING) {
		dev_err(dev, "%s invaild offset", __func__);
		BUG_ON(1);
		return 0;
	}

	regmap_read(tx_dev->reg_base, offset, &reg_val);
	return reg_val;
}
void hdmitx_write32(struct device *dev, unsigned int offset, unsigned int value)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);	

	if (offset > HDMI_FRL_TRAINING) {
		dev_err(dev, "%s invaild offset", __func__);
		BUG_ON(1);
		return;
	}

	regmap_write(tx_dev->reg_base, offset, value);
}

void hdmitx_mask32(struct device *dev, unsigned int offset, unsigned int andMask,
	unsigned int orMask)
{
	hdmitx_write32(dev, offset, ((hdmitx_read32(dev, offset) & (andMask)) | (orMask)));
}

void hdmitx_reset_clk(struct device *dev)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);	

	if (IS_ERR(tx_dev->clk_hdmi) ||
		IS_ERR(tx_dev->reset_hdmi)) {
		dev_err(dev, "%s fail", __func__);
		return;
	}

	clk_disable_unprepare(tx_dev->clk_hdmi);
	reset_control_assert(tx_dev->reset_hdmi);
	usleep_range(1000, 1005);
	reset_control_deassert(tx_dev->reset_hdmi);
	clk_prepare_enable(tx_dev->clk_hdmi);
	usleep_range(1000, 1005);
}

/**
 * hdmitx_get_raw_edid - get raw data of sink device EDID
 * @edid: output data buffer
 *
 * Return:  0 on success, -E* on failure
 */
int hdmitx_get_raw_edid(struct device *dev, unsigned char *edid)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);	
	
	dev_dbg(dev, "%s", __func__);

	if (!(tx_dev->hdmi_data->sink_cap_available)) {
		dev_info(dev, "%s:EDID not ready", __func__);
		return -ENOMSG;
	}

	if (memcpy(edid, (u8 *)tx_dev->hdmi_data->hdmitx_edid_info.raw_edid, sizeof(struct raw_edid)) == NULL) {
		dev_info(dev, "%s:failed to copy EDID", __func__);
		return -EFAULT;
	}

	return 0;
}
EXPORT_SYMBOL(hdmitx_get_raw_edid);

int rtk_hdmitx_open(struct inode *inode, struct file *file)
{
	if (nonseekable_open(inode, file))
		return -ENODEV;

	return 0;
}

/**
 * hdmitx_ioctl -  ioctl function of hdmitx miscdev
 * @file: my argument
 * @cmd: control command
 * @arg: arguments
 *
 * Return: 0 on success, -E* on failure
 */
static long rtk_hdmitx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *mdev = file->private_data;
	hdmitx_device_t *tx_dev = container_of(mdev, hdmitx_device_t, miscdev);
	asoc_hdmi_t *tx_data; 

	if (!tx_dev)
		return -ENODEV;

	tx_data = tx_dev->hdmi_data;

	if ((cmd != HDMI_CHECK_LINK_STATUS) && (cmd != HDMI_CHECK_Rx_Sense))
		trace_hdmitx_ioctl(cmd);

	switch (cmd) {
	case HDMI_GET_SINK_CAPABILITY:
		return ops_get_sink_cap((void __user *)arg, tx_dev);

	case HDMI_GET_RAW_EDID:
		return ops_get_raw_edid((void __user *)arg, tx_dev);

	case HDMI_CHECK_LINK_STATUS:
		return ops_get_link_status((void __user *)arg, tx_dev);

	case HDMI_GET_VIDEO_CONFIG:
		return ops_get_video_config((void __user *)arg, tx_data);

	case HDMI_SEND_AVMUTE:
		return ops_send_AVmute((void __user *)arg, tx_dev);

	case HDMI_CONFIG_TV_SYSTEM:
		return ops_config_tv_system((void __user *)arg, tx_dev);

	case HDMI_CONFIG_AVI_INFO:
		return ops_config_avi_info((void __user *)arg);

	case HDMI_SET_FREQUNCY:
		return ops_set_frequency((void __user *)arg, tx_dev);

	case HDMI_SEND_AUDIO_MUTE:
		return ops_set_audio_mute((void __user *)arg);

	case HDMI_SEND_AUDIO_VSDB_DATA:
		return ops_send_audio_vsdb_data((void __user *)arg);

	case HDMI_SEND_AUDIO_EDID2:
		return ops_send_audio_edid2((void __user *)arg);

	case HDMI_CHECK_Rx_Sense:
		return ops_check_rx_sense((void __user *)arg, tx_dev);

	case HDMI_GET_EXT_BLK_COUNT:
		return ops_get_extension_blk_count((void __user *)arg, tx_dev);

	case HDMI_GET_EXTENDED_EDID:
		return ops_get_extended_edid((void __user *)arg, tx_dev);

	case HDMI_SEND_VOUT_EDID_DATA:
		return ops_send_vout_edid_data((void __user *)arg);

	case HDMI_GET_EDID_SUPPORT_LIST:
		return ops_get_edid_support_list((void __user *)arg, tx_dev);

	case HDMI_SET_OUTPUT_FORMAT:
		return ops_set_output_format((void __user *)arg, tx_dev);

	case HDMI_GET_OUTPUT_FORMAT:
		return ops_get_output_format((void __user *)arg);

	case HDMI_SET_VO_INTERFACE_TYPE:
		return ops_set_interface_type((void __user *)arg);

	case HDMI_GET_CONFIG_TV_SYSTEM:
		return ops_get_config_tv_system((void __user *)arg);

	case HDMI_HOTPLUG_DETECTION:
		return ops_set_hotplug_detection((void __user *)arg, tx_dev);

	case HDMI_WAIT_HOTPLUG:
		return ops_wait_hotplug((void __user *)arg, tx_dev);

	case HDMI_GET_EDID_BLOCK:
		return ops_get_edid_block((void __user *)arg, tx_dev);

	case HDMI_SET_VRR:
		return ops_set_vrr((void __user *)arg, tx_dev);

	case HDMI_CONTROL_5V:
		return ops_ctrl_5v((void __user *)arg, tx_dev);

	case HDMI_SET_FAKE_EDID:
		return ops_set_fake_edid((void __user *)arg, tx_dev);

	default:
		dev_dbg(tx_dev->dev, " Unknown ioctl cmd %08x", cmd);
		return -EFAULT;
	}
}

static int rtk_hdmitx_fasync(int fd, struct file *file, int on)
{
	struct miscdevice *mdev = file->private_data;
	hdmitx_device_t *tx_dev = container_of(mdev, hdmitx_device_t, miscdev);
	
	return fasync_helper(fd, file, on, &tx_dev->fasync);
}


const struct file_operations rtk_hdmitx_fops = {
	.owner			= THIS_MODULE,
	.open			= rtk_hdmitx_open,
	.unlocked_ioctl		= rtk_hdmitx_ioctl,
#if !defined(CONFIG_COMPAT)
	.compat_ioctl		= NULL,
#else
	.compat_ioctl		= rtk_compat_hdmitx_ioctl,
#endif
	.poll			= NULL,
	.fasync 		= rtk_hdmitx_fasync, 
};

static int rtk_hdmitx_suspend(struct device *dev)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);	
	int ret_val;
	int hpd_state;

	dev_info(dev, "Enter %s", __func__);

	ret_val = rtk_hdmitx_switch_suspend(tx_dev);
	hdmitx_scdcrr_suspend(tx_dev);

	/* Send no scramble for prevent HDMI 5V still exist after suspend */
	hpd_state = hdmitx_switch_get_state(dev);
	if (hpd_state == 1)
		hdmitx_send_scdc_TmdsConfig(&tx_dev->hdmi_data->hdmitx_edid_info, 0, 0, 0);

	dev_info(dev, "Exit %s", __func__);
	return ret_val;
}

static int rtk_hdmitx_resume(struct device *dev)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);	
	int ret_val;

	dev_info(dev, "Enter %s", __func__);

	hdmitx_scdcrr_resume(tx_dev);
	ret_val = rtk_hdmitx_switch_resume(tx_dev);

	dev_info(dev, "Exit %s", __func__);
	return ret_val;
}

static void rtk_hdmitx_shutdown(struct platform_device *pdev)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(&pdev->dev);		
	int hpd_state;

	dev_info(&pdev->dev, "Enter %s", __func__);

	rtk_hdmitx_switch_suspend(tx_dev);

	/* Send no scramble for prevent HDMI 5V still exist */
	hpd_state = hdmitx_switch_get_state(&pdev->dev);
	if (hpd_state == 1)
		hdmitx_send_scdc_TmdsConfig(&tx_dev->hdmi_data->hdmitx_edid_info, 0, 0, 0);

	dev_info(&pdev->dev, "Exit %s", __func__);
}

static int rtk_hdmi_probe(struct platform_device *pdev)
{
	struct device_node *dptx_np;
	struct device_node *dsitx_np;
	struct device_node *syscon_np;
	hdmitx_device_t *tx_dev;
	int ret_value;
	unsigned int frl_support;

	dev_info(&pdev->dev, "Driver init");

	tx_dev = devm_kzalloc(&pdev->dev,  sizeof(hdmitx_device_t), GFP_KERNEL);

	tx_dev->dev = &pdev->dev;

	syscon_np = of_parse_phandle(pdev->dev.of_node, "syscon", 0);
	if (IS_ERR_OR_NULL(syscon_np)) {
		dev_err(&pdev->dev, "Parse syscon phandle 0 fail");
		goto end;
	}

	tx_dev->reg_base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(tx_dev->reg_base)) {
		dev_err(&pdev->dev, "Remap syscon 0 to reg_base fail");
		of_node_put(syscon_np);
		goto end;
	}

	syscon_np = of_parse_phandle(pdev->dev.of_node, "syscon", 2);
	if (IS_ERR_OR_NULL(syscon_np)) {
		dev_err(&pdev->dev, "Parse syscon phandle 2 fail");
		goto end;
	}

	tx_dev->pll_base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(tx_dev->pll_base)) {
		dev_err(&pdev->dev, "Remap syscon 2 to pll_base fail");
		of_node_put(syscon_np);
		goto end;
	}

	syscon_np = of_parse_phandle(pdev->dev.of_node, "syscon", 3);
	if (IS_ERR_OR_NULL(syscon_np)) {
		dev_err(&pdev->dev, "Parse syscon phandle 3 fail");
	} else {
		tx_dev->top_base = syscon_node_to_regmap(syscon_np);
		if (IS_ERR_OR_NULL(tx_dev->pll_base)) {
			dev_err(&pdev->dev, "Remap syscon 3 to top_base fail");
			of_node_put(syscon_np);
		}
	}

	ret_value = of_property_read_u32(pdev->dev.of_node, "support-frl",
		&frl_support);

	if (!ret_value) {
		if (frl_support) {
			tx_dev->frl = devm_kzalloc(&pdev->dev,
					 sizeof(rtk_hdmitx_frl_t), GFP_KERNEL);
			tx_dev->frl->is_support_frl = 1;
			dev_info(&pdev->dev, "support-frl property do support");
		} else {
			dev_info(&pdev->dev, "support-frl property doesn't support");
		}
	} else {
		dev_info(&pdev->dev, "support-frl property doesn't exist");
	}

	tx_dev->reset_hdmi = reset_control_get_exclusive(&pdev->dev, "rstn_hdmi");
	if (IS_ERR(tx_dev->reset_hdmi)) {
		dev_err(&pdev->dev, "Can't get reset_control reset_hdmi");
		goto deferred;
	}

	tx_dev->clk_hdmi = clk_get(&pdev->dev, "clk_en_hdmi");
	if (IS_ERR(tx_dev->clk_hdmi)) {
		dev_err(&pdev->dev, "Can't get clk clk_hdmi");
		goto err_clk_hdmi;
	}

	tx_dev->hdmi_data = devm_kzalloc(&pdev->dev, sizeof(asoc_hdmi_t), GFP_KERNEL);

	hdmitx_reset_sink_capability(tx_dev->hdmi_data);
	platform_set_drvdata(pdev, tx_dev);

	/* Initial hotplug gpio */
	tx_dev->hpd_gpio = devm_gpiod_get(&pdev->dev, "hpd-detect", GPIOD_IN);
	if (IS_ERR(tx_dev->hpd_gpio)) {
		dev_err(&pdev->dev, "Could not get gpio from of");
		goto err_hpd_gpio;
	} else {
		dev_info(&pdev->dev, "hotplug gpio(%d)", desc_to_gpio(tx_dev->hpd_gpio));
	}

	/* Initial HDMI_5V ctrl gpio */
	tx_dev->ctrl_5v_gpio = devm_gpiod_get(&pdev->dev, "hdmi-5v", GPIOD_OUT_LOW);
	if (IS_ERR(tx_dev->ctrl_5v_gpio)) {
		dev_err(&pdev->dev, "Request gpio(%d) fail", desc_to_gpio(tx_dev->ctrl_5v_gpio));
		goto err_ctrl_5v_gpio;
	} else {
		dev_info(&pdev->dev, "HDMI_5V gpio(%d)", desc_to_gpio(tx_dev->ctrl_5v_gpio));
	}

	/* Get hotplug gpio irq */
	tx_dev->hpd_irq = gpiod_to_irq(tx_dev->hpd_gpio);
	if (!tx_dev->hpd_irq)
		dev_err(&pdev->dev, "Fail to get hpd_irq");
	else
		dev_dbg(&pdev->dev, "hpd_irq num=%u", tx_dev->hpd_irq);

	/* RxSense  mode */
	ret_value = of_property_read_u32(pdev->dev.of_node, "rxsense-mode",
		&tx_dev->rxsense_mode);

	if (ret_value < 0 || tx_dev->rxsense_mode > RXSENSE_INTERRUPT_MODE)
		tx_dev->rxsense_mode = RXSENSE_PASSIVE_MODE;

	dev_info(&pdev->dev, "rxsense_mode=%u", tx_dev->rxsense_mode);

	tx_dev->hdmi_irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!tx_dev->hdmi_irq)
		dev_info(&pdev->dev, "Not support RxSense interrupt");
	else
		dev_info(&pdev->dev, "hdmi_irq num=%u", tx_dev->hdmi_irq);

	init_waitqueue_head(&tx_dev->hpd_wait);

	setup_mute_gpio(&pdev->dev);

	/* get cec notifier */
	tx_dev->cec = cec_notifier_conn_register(&pdev->dev, NULL, NULL);
	if (!tx_dev->cec)
		goto err_cec_notifier;

	cec_notifier_phys_addr_invalidate(tx_dev->cec);

	/* Initial SCDC read request */
	if (register_hdmitx_scdcrr(&pdev->dev)) {
		dev_err(&pdev->dev, "Fail to register scdcrr");
		goto err_scdcrr;
	}

	tx_dev->miscdev.fops = &rtk_hdmitx_fops;
	tx_dev->miscdev.name = "hdmitx";
	tx_dev->miscdev.mode = 0666;
	tx_dev->miscdev.minor = MISC_DYNAMIC_MINOR;

	if (misc_register(&tx_dev->miscdev)) {
		dev_err(&pdev->dev, "Could not register hdmitx miscdev");
		goto err_misc_register;
	}

	/* Register sysfs */
	register_hdmitx_sysfs(&pdev->dev);
	register_support_list_sysfs(&pdev->dev);

	/* Set interfaceType if DP or DSI exist */
	dptx_np = of_find_compatible_node(NULL, NULL, "Realtek,rtk-dptx");
	if (dptx_np) {
		if (of_device_is_available(dptx_np)) {
			dev_info(&pdev->dev, "Found DP TX node");
			dp_or_dsi_exist = 1;
		}
	}

	dsitx_np = of_find_compatible_node(NULL, NULL, "realtek,rtk-dsi");
	if (dsitx_np) {
		if (of_device_is_available(dsitx_np)) {
			dev_info(&pdev->dev, "Found DSI TX node");
			dp_or_dsi_exist = 1;
		}
	}

	reset_control_deassert(tx_dev->reset_hdmi);
	clk_prepare_enable(tx_dev->clk_hdmi);

	if (register_hdmitx_switchdev(tx_dev)) {
		dev_err(&pdev->dev, "Could not register_hdmitx_switchdev");
		goto err_switch_register;
	}

	dev_info(&pdev->dev, "Driver init done");
	return 0;

err_switch_register:
	misc_deregister(&tx_dev->miscdev);
err_scdcrr:
err_misc_register:
err_cec_notifier:
	devm_gpiod_put(&pdev->dev, tx_dev->ctrl_5v_gpio);
err_ctrl_5v_gpio:
	devm_gpiod_put(&pdev->dev, tx_dev->hpd_gpio);
err_hpd_gpio:
	clk_put(tx_dev->clk_hdmi);
err_clk_hdmi:
	reset_control_release(tx_dev->reset_hdmi);
	reset_control_put(tx_dev->reset_hdmi);
deferred:
	return -EPROBE_DEFER;

end:
	return -EFAULT;
}

static const struct of_device_id rtk_hdmitx_dt_ids[] = {
	{ .compatible = "realtek,rtd119x-hdmitx", },
	{ .compatible = "realtek,rtd129x-hdmitx", },
	{ .compatible = "realtek,rtd161x-hdmitx", },
	{ .compatible = "realtek,rtd13xx-hdmitx", },
	{ .compatible = "realtek,rtd16xxb-hdmitx", },
	{ .compatible = "realtek,rtd13xxd-hdmitx", },
	{},
};
MODULE_DEVICE_TABLE(of, rtk_hdmitx_dt_ids);

static const struct dev_pm_ops rtk_hdmitx_pm_ops = {
	.suspend    = rtk_hdmitx_suspend,
	.resume     = rtk_hdmitx_resume,
	.freeze     = rtk_hdmitx_suspend,
	.thaw       = rtk_hdmitx_resume,
	.restore    = rtk_hdmitx_resume,
};

static struct platform_driver rtk_hdmi_driver = {
	.probe = rtk_hdmi_probe,
	.driver = {
		.name = "rtk_hdmi",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rtk_hdmitx_dt_ids),
#if IS_ENABLED(CONFIG_PM)
		.pm = &rtk_hdmitx_pm_ops,
#endif
	},
	.shutdown = rtk_hdmitx_shutdown,
};

static int __init rtk_hdmi_init(void)
{
	if (platform_driver_register(&rtk_hdmi_driver)) {
		pr_err("Could not add character driver");
		goto err_register;
	}

	return 0;

err_register:
	platform_driver_unregister(&rtk_hdmi_driver);

	return -EFAULT;
}


static int rtk_hdmi_exit_callback(struct device *dev, void *data)
{
	hdmitx_device_t *tx_dev = dev_get_drvdata(dev);	

	deregister_hdmitx_switchdev(tx_dev);
	misc_deregister(&tx_dev->miscdev);

	return 0;
}

static void __exit rtk_hdmi_exit(void)
{
	int err;

	err = driver_for_each_device(&rtk_hdmi_driver.driver,
			NULL, NULL, rtk_hdmi_exit_callback);
	if (err)
		pr_err("%s fail", __func__);

	platform_driver_unregister(&rtk_hdmi_driver);
}

module_init(rtk_hdmi_init);
module_exit(rtk_hdmi_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Realtek HDMI kernel module");
