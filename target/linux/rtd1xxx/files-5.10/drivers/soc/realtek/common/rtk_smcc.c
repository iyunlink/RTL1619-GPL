/*
 * rtk_smcc.c - RTK SMCC driver
 *
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */
//#define DEBUG

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/arm-smccc.h>
#include <linux/dma-map-ops.h>
#include <asm/cacheflush.h>

#include "rtk_smcc.h"

#define RTK_SMCC_DEVICE_NAME "rtk_smcc"
#define SMC_RETURN_OK 0

#define member_size(type, member) (sizeof(((type*)(0))->member))

static dev_t devno;

struct rtk_smcc_file_data {
	struct rtk_smcc_device *device_t;
};

struct rtk_smcc_device {
	struct device *device;
	struct cdev cdev;
	struct mutex lock;
	struct class *class;
	unsigned long long *OTP_PTR_VA;
	dma_addr_t OTP_PTR_PA;
};

static long rtk_smcc_dev_ioctl(struct file* file, unsigned int cmd, unsigned long arg);


/***************************************************************************
 File Operations
****************************************************************************/

/*------------------------------------------------------------------
 * Func : rtk_smcc_dev_open
 * Desc : open function of rtk smcc dev
 * Parm : inode : inode of dev
 *        file  : context of file
 * Retn : 0 : success, others fail
 * ------------------------------------------------------------------*/
static int rtk_smcc_dev_open(struct inode *inode, struct file *file)
{
	struct rtk_smcc_device *device_t = container_of(inode->i_cdev, struct rtk_smcc_device, cdev);

	file->private_data = device_t;
	return 0;
}

/*------------------------------------------------------------------
 * Func : rtk_smcc_dev_release
 * Desc : release function of rtk smcc dev
 * Parm : inode : inode of dev
 *        file  : context of file
 * Retn : 0 : success, others fail
 * ------------------------------------------------------------------*/
static int rtk_smcc_dev_release(struct inode* inode, struct file* file)
{
	if (file->private_data)
		file->private_data = NULL;

	return 0;
}

/*------------------------------------------------------------------
 * Func : compat_rtk_smcc_dev_ioctl
 * Desc : ioctl function of rtk smcc dev
 * Parm : inode : inode of dev
 * file : context o	le
 * cmd : control command
 * arg : arguments
 * Retn : 0 : success, others fail
 *------------------------------------------------------------------*/
static long compat_rtk_smcc_dev_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
	return rtk_smcc_dev_ioctl(file, cmd, arg);
}

/*----------------------------------------------------------------
 * Func : rtk_smcc_dev_ioctl
 * Desc : ioctl function of compat rtk smcc dev
 * Parm : inode : inode of dev
 * file : context of file
 * cmd : control command
 * arg : arguments
 * Retn : 0 : success, others fail
 *------------------------------------------------------------------*/
static long rtk_smcc_dev_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
	struct rtk_smcc_device *data = file->private_data;
	struct device *dev = data->device;
	struct arm_smccc_res res;
	OTP_Info_T otp_info_local = {0};
	OTP_Write_Info_T otp_write_info_local = {0};
	int ret = 0;
	int ret_result = 0;
	OPT_Data_T *opt_data_va;

	switch (cmd) {

		case RTK_SMCC_OTP_READ:
			mutex_lock(&data->lock);

			if (data->OTP_PTR_VA == NULL || data->OTP_PTR_PA == 0) {
				ret_result = -ENOMEM;
				goto fail_out;
			}

			opt_data_va = (OPT_Data_T *)data->OTP_PTR_VA;

			ret = copy_from_user((void *)&otp_info_local, (void *)arg, sizeof(OTP_Info_T));
			if (ret) {
				mutex_unlock(&data->lock);
				dev_err(dev, "copy_from_user %d size of data fail.  [0x%d]\n", sizeof(OTP_Info_T), ret);
				ret_result = -EPERM;
				goto fail_out;
			}

			dev_dbg(dev, "RTK_SMCC_OTP_READ item:[0x%x]\n", otp_info_local.typeID);

			arm_smccc_smc(RTK_OTP_READ, otp_info_local.typeID, data->OTP_PTR_PA, data->OTP_PTR_PA + offsetof(OPT_Data_T, ret_value_h), 0, 0, 0, 0, &res);
			if (res.a0 != SMC_RETURN_OK) {
				mutex_unlock(&data->lock);
				ret_result = -EPERM;
				goto fail_out;
			}

			dev_dbg(dev, "[0x%llx] [0x%llx]\n", opt_data_va->ret_value, opt_data_va->ret_value_h);

			otp_info_local.ret_value = opt_data_va->ret_value;
			otp_info_local.ret_value_h = opt_data_va->ret_value_h;

			if (copy_to_user((void *)arg, (void *)&otp_info_local, sizeof(OTP_Info_T))) {
				mutex_unlock(&data->lock);
				dev_err(dev, "do ioctl command failed - copy otp data to user failed \n");
				ret_result = -EPERM;
				goto fail_out;
			}
			mutex_unlock(&data->lock);

		break;

		case RTK_SMCC_OTP_WRITE:
			mutex_lock(&data->lock);

			if (data->OTP_PTR_VA == NULL || data->OTP_PTR_PA == 0) {
				ret_result = -ENOMEM;
				goto fail_out;
			}

			opt_data_va = (OPT_Data_T *)data->OTP_PTR_VA;

			ret = copy_from_user((void *)&otp_write_info_local, (void *)arg, sizeof(OTP_Write_Info_T));
			if (ret) {
				mutex_unlock(&data->lock);
				dev_err(dev, "copy_from_user %d size of data fail.  [%d]\n", sizeof(OTP_Write_Info_T), ret);
				ret_result = -EPERM;
				goto fail_out;
			}

			dev_dbg(dev, "RTK_SMCC_OTP_WRITE: [0x%d] [0x%llx] [0x%x], [0x%llx]\n",  otp_write_info_local.typeID, otp_write_info_local.burning_value, otp_write_info_local.perform_case, otp_write_info_local.burning_data[0]);

			if (otp_write_info_local.perform_case == DATA_SECTION_CASE) {
				memcpy(opt_data_va->burning_data, otp_write_info_local.burning_data, member_size(OTP_Write_Info_T, burning_data));

				arm_smccc_smc(RTK_OTP_WRITE, otp_write_info_local.typeID, data->OTP_PTR_PA + offsetof(OPT_Data_T, burning_data), 0, 0, 0, 0, 0, &res);
				if (res.a0 != SMC_RETURN_OK) {
					mutex_unlock(&data->lock);
					ret_result = -EPERM;
					goto fail_out;
				}

			} else {
				arm_smccc_smc(RTK_OTP_WRITE, otp_write_info_local.typeID, otp_write_info_local.burning_value, 0, 0, 0, 0, 0, &res);
				if (res.a0 != SMC_RETURN_OK) {
					mutex_unlock(&data->lock);
					ret_result = -EPERM;
					goto fail_out;
				}
			}
			mutex_unlock(&data->lock);
		break;

		default:
			dev_dbg(dev, "do ioctl command failed - unknown ioctl command(%d)\n", cmd);
			ret_result = -EPERM;
			goto fail_out;
		break;
	}

	ret_result = 0;

fail_out:

	return ret_result;
}

static struct file_operations rtk_smcc_ops =
{
	.owner = THIS_MODULE,
	.unlocked_ioctl = rtk_smcc_dev_ioctl,
	.compat_ioctl = compat_rtk_smcc_dev_ioctl,
	.open = rtk_smcc_dev_open,
	.release = rtk_smcc_dev_release,
};

static int rtk_smcc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_smcc_device *device_t = NULL;
	struct device *device = NULL;
	struct class *class = NULL;
	int ret = 0;
	int ret_result = 0;
	unsigned long probe_time = jiffies;

	dev_info(dev, "ENTER %s", __func__);
	device_t = devm_kzalloc(dev, sizeof(*device_t), GFP_KERNEL);
	if (!device_t) {
		ret_result = -ENOMEM;
		goto fail;
	}

	dev_set_drvdata(&pdev->dev, device_t);

	cdev_init(&device_t->cdev, &rtk_smcc_ops);

	device_t->OTP_PTR_VA = dma_alloc_coherent(dev, PAGE_SIZE, &(device_t->OTP_PTR_PA), GFP_KERNEL | GFP_DMA);

	if (device_t->OTP_PTR_VA == NULL) {
		ret_result = -ENOMEM;
		goto fail;
	}

	if (alloc_chrdev_region(&devno, 0, 1, RTK_SMCC_DEVICE_NAME) < 0) {
		dev_err(dev, "%s: fail to reister char-device\n", __func__);
		ret_result = -ENODEV;
		goto fail;
	}

	device_t->cdev.owner = THIS_MODULE;
	ret = cdev_add(&device_t->cdev, devno, 1);
	if (ret) {
		dev_err(dev, "%s: fail to reister char-device\n", __func__);
		ret_result = -ENODEV;
		goto fail_unregister_chrdev;
	}

	class = class_create(THIS_MODULE, RTK_SMCC_DEVICE_NAME);
	if (IS_ERR(class)) {
		ret = PTR_ERR(class);
		dev_err(dev, "%s: class_create with ret %d\n", __func__, ret);
		goto fail_destroy_class;
	}

	device = device_create(class, NULL, devno, NULL, "rtk_smcc_core");
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		dev_err(dev, "%s: class_create with ret %d\n", __func__, ret);
		goto fail_destroy_device;
	}

	device_t->class = class;
	device_t->device = device;

	mutex_init(&device_t->lock);

	dev_info(dev, "%s OK (take %d ms)\n", __func__, jiffies_to_msecs(jiffies - probe_time));

	return ret_result;

fail_destroy_device:
	device_destroy(class, device->devt);

fail_destroy_class:
	class_destroy(class);

fail_unregister_chrdev:
	unregister_chrdev_region(devno, 1);

fail:
	if (device_t->OTP_PTR_VA != NULL)
		dma_free_coherent(dev, PAGE_SIZE, (void *)(device_t->OTP_PTR_VA), device_t->OTP_PTR_PA);

	if (device_t)
		devm_kfree(dev, device_t);

	dev_set_drvdata(&pdev->dev, NULL);

	return ret_result;
}

static int rtk_smcc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_smcc_device *rtk_smcc_device = dev_get_drvdata(dev);

	dev_info(&pdev->dev, "%s\n", __func__);

	device_destroy(rtk_smcc_device->class, rtk_smcc_device->device->devt);
	class_destroy(rtk_smcc_device->class);

	cdev_del(&rtk_smcc_device->cdev);

	unregister_chrdev_region(devno, 1);

	return 0;
}

static int rtk_smcc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct device *dev = &pdev->dev;

	dev_dbg(dev, "Enter %s", __func__);
	dev_dbg(dev, "Exit %s", __func__);
	return 0;
}

static int rtk_smcc_resume(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_dbg(dev, "Enter %s", __func__);
	dev_dbg(dev, "Exit %s", __func__);
	return 0;
}

static struct of_device_id rtk_smcc_ids[] = {
	{ .compatible = "Realtek,rtk-smcc" },
	{ /* Sentinel */ },
};

static struct platform_driver rtk_smcc_driver = {
	.probe = rtk_smcc_probe,
	.remove = rtk_smcc_remove,
	.suspend = rtk_smcc_suspend,
	.resume = rtk_smcc_resume,
	.driver = {
		.name = "RTK_SMCC",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rtk_smcc_ids),
	},
};
module_platform_driver(rtk_smcc_driver);

MODULE_LICENSE("GPL");
