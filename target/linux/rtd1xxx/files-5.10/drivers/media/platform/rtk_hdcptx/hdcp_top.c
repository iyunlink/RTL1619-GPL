/*
 * hdcp_top.c - RTK hdcp tx driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/compat.h>

#include "hdcp.h"

#include "hdmitx_reg.h"
#include "hdcp_top.h"
#include "hdcp_lib_hw.h"

struct hdcp hdcp;
struct hdcp_sha_in sha_input;
struct hdcp_ksvlist_info ksvlist_info;

/* State machine / workqueue */
static void hdcp_wq_start_authentication(void);
static void hdcp_wq_check_r0(void);
static void hdcp_wq_step2_authentication(void);
static void hdcp_wq_authentication_failure(void);
static void hdcp_work_queue(struct work_struct *work);
static struct delayed_work *hdcp_submit_work(int event, int delay);
static void hdcp_cancel_work(struct delayed_work **work);

/* Control */
static long hdcp_enable_ctl(void __user *argp);
static long hdcp_disable_ctl(void);
static long hdcp_query_status_ctl(void __user *argp);


/* Driver */
static int __init hdcp_init(void);
static void __exit hdcp_exit(void);


static DECLARE_WAIT_QUEUE_HEAD(hdcp_up_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD(hdcp_down_wait_queue);

void hdcp_set_state(struct miscdevice *pdev, int state);

struct miscdevice mdev;
/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_start_authentication
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_start_authentication(void)
{
	int status = HDCP_OK;

	hdcp.hdcp_state = HDCP_AUTHENTICATION_START;
	hdcp.auth_state = HDCP_STATE_INIT;
	hdcp_set_state(&mdev, hdcp.auth_state);

	if (hdcp.print_messages)
		HDCP_INFO("Authentication start");

	/* Step 1 part 1 (until R0 calc delay) */
	status = hdcp_lib_step1_start();

	if (status != HDCP_OK)
		hdcp.err_code = -status;

	if (status == -HDCP_AKSV_ERROR) {
		hdcp_wq_authentication_failure();
	} else if (status == -HDCP_CANCELLED_AUTH) {
		hdcp.hdcp_state = HDCP_DISABLED;
		hdcp.auth_state = HDCP_STATE_DISABLED;
		HDCP_ERROR("Authentication step 1 cancelled");
		return;
	} else if (status != HDCP_OK) {
		hdcp_wq_authentication_failure();
	} else {
		hdcp.hdcp_state = HDCP_WAIT_R0_DELAY;
		hdcp.auth_state = HDCP_STATE_AUTH_1ST_STEP;
		hdcp.pending_wq_event = hdcp_submit_work(HDCP_R0_EXP_EVENT, HDCP_R0_DELAY);
	}

}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_check_r0
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_check_r0(void)
{
	int status = hdcp_lib_step1_r0_check();

	if (status != HDCP_OK)
		hdcp.err_code = -status;

	if (status == -HDCP_CANCELLED_AUTH) {
		HDCP_ERROR("Authentication step 1/R0 cancelled.");
		return;
	} else if (status < 0) {
		hdcp_wq_authentication_failure();
	} else {
		hdcp.fail_cnt = 0;
		hdcp.print_messages = 1;
		if (hdcp_lib_check_repeater_bit_in_tx()) {
			/* Repeater */
			HDCP_INFO("authentication step 1 successful - Repeater");

			hdcp.hdcp_state = HDCP_WAIT_KSV_LIST;
			hdcp.auth_state = HDCP_STATE_AUTH_2ND_STEP;
			hdcp.pending_wq_event =
				hdcp_submit_work(HDCP_KSV_LIST_RDY_EVENT, 500);
		} else {
			/* Receiver */
			HDCP_INFO("authentication step 1 successful - Receiver");

			/* hdcp.av_mute_needed = 1; */
			hdcp.hdcp_state = HDCP_LINK_INTEGRITY_CHECK;
			hdcp.auth_state = HDCP_STATE_AUTH_3RD_STEP;
			hdcp_set_state(&mdev, hdcp.auth_state);

			/* Restore retry counter */
			if (hdcp.en_ctrl->nb_retry == 0)
				hdcp.retry_cnt = HDCP_INFINITE_REAUTH;
			else
				hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;
		}
	}

}

static void hdcp_wq_auto_check_r0(void)
{
	int status;

	status = hdcp_lib_r0_check();
	if (status == -HDCP_CANCELLED_AUTH) {
		HDCP_ERROR("Authentication step 3 cancelled.");
		return;
	} else if (status < 0) {
		hdcp.err_code = HDCP_RI_ERROR;
		hdcp_wq_authentication_failure();
	} else {
		hdcp.fail_cnt = 0;
		hdcp.print_messages = 1;

		/* Receiver */
		HDCP_DEBUG("hdcp_wq_auto_check_r0 successful - %s",
			hdcp_lib_check_repeater_bit_in_tx() ? "Repeater" : "Receiver");

		/* hdcp.av_mute_needed = 1; */
		hdcp.hdcp_state = HDCP_LINK_INTEGRITY_CHECK;
		hdcp.auth_state = HDCP_STATE_AUTH_3RD_STEP;
	}

}

irqreturn_t HDCP_interrupt_handler(int irq, void *dev_id)
{
	unsigned int intst;
	unsigned int riupdated;

	if (!__clk_is_enabled(hdcp.clk_hdmi))
		return IRQ_HANDLED;

	regmap_read(hdcp.hdcp_base, HDMI_INTST, &intst);
	riupdated = HDMI_INTST_get_riupdated(intst);

	if (riupdated && (hdcp.hdcp_enabled == 1)) {
		hdcp.hdcp_state = HDCP_LINK_INTEGRITY_CHECK;
		hdcp_submit_work(HDCP_START_FRAME_EVENT, 0);
	} else if (riupdated && (hdcp.hdcp_enabled == 0)) {
		HDCP_INFO("Already disabled, skip ri check");
	}

	regmap_write(hdcp.hdcp_base, HDMI_INTST, 0);

	return IRQ_HANDLED;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_step2_authentication
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_step2_authentication(void)
{
	int status = HDCP_OK;

	HDCP_DEBUG("[%s] %s  %d :%u", __FILE__, __func__, __LINE__, jiffies_to_msecs(jiffies));

	/* KSV list timeout is running and should be canceled */
	hdcp_cancel_work(&hdcp.pending_wq_event);

	status = hdcp_lib_step2();

	if (status != HDCP_OK)
		hdcp.err_code = -status;

	if (status == -HDCP_CANCELLED_AUTH) {
		HDCP_ERROR("Authentication step 2 cancelled.");
		return;
	} else if (status < 0) {
		hdcp_wq_authentication_failure();
	} else {
		HDCP_INFO("Repeater authentication step 2 successful");

		/* hdcp.av_mute_needed = 1; */
		hdcp.hdcp_state = HDCP_LINK_INTEGRITY_CHECK;
		hdcp.auth_state = HDCP_STATE_AUTH_3RD_STEP;
		hdcp_set_state(&mdev, hdcp.auth_state);

		/* Restore retry counter */
		if (hdcp.en_ctrl->nb_retry == 0)
			hdcp.retry_cnt = HDCP_INFINITE_REAUTH;
		else
			hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;
	}

}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_authentication_failure
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_authentication_failure(void)
{
	if (hdcp.hdmi_state == HDMI_STOPPED) {
		hdcp.auth_state = HDCP_STATE_AUTH_FAILURE;
		hdcp_set_state(&mdev, hdcp.auth_state);
		return;
	} else if ((hdcp.hdcp_state == HDCP_DISABLED) && (hdcp.auth_state == HDCP_STATE_DISABLED)) {
		HDCP_INFO("Already disabled, skip failure process");
		return;
	}

	hdcp_lib_auto_ri_check(false);

	if (hdcp.av_mute_needed)
		hdcp_lib_set_av_mute(AV_MUTE_SET);

	hdcp_lib_set_encryption(HDCP_ENC_OFF);

	hdcp_cancel_work(&hdcp.pending_wq_event);

	if (hdcp.retry_cnt && (hdcp.hdmi_state != HDMI_STOPPED)) {

		if (hdcp.retry_cnt < HDCP_INFINITE_REAUTH) {
			hdcp.retry_cnt--;
			HDCP_ERROR("authentication failed - retrying, attempts=%d", hdcp.retry_cnt);
		} else {
			hdcp.fail_cnt++;
			if (hdcp.print_messages) {

				if (hdcp.fail_cnt < HDCP_MAX_FAIL_MESSAGES) {
					HDCP_ERROR("authentication failed - retrying");
				} else {
					hdcp.print_messages = 0;
					HDCP_ERROR(" authentication failed %d consecutive times", hdcp.fail_cnt);
					HDCP_ERROR(" will keep trying but silencing logs until hotplug or success");
				}
			}
		}

		hdcp.hdcp_state = HDCP_AUTHENTICATION_START;
		hdcp.auth_state = HDCP_STATE_AUTH_FAIL_RESTARTING;

		hdcp.pending_wq_event =
			hdcp_submit_work(HDCP_AUTH_REATT_EVENT, hdcp.reauth_delay);

		if (hdcp.reauth_delay == 0)
			hdcp.reauth_delay = 100;
		else if (hdcp.reauth_delay < 1600)
			hdcp.reauth_delay = hdcp.reauth_delay * 2;

	} else {
		HDCP_ERROR("authentication failed -HDCP disabled\n");
		hdcp.hdcp_state = HDCP_ENABLE_PENDING;
		hdcp.auth_state = HDCP_STATE_AUTH_FAILURE;
		hdcp_set_state(&mdev, hdcp.auth_state);
	}

	/* hdcp_set_state(&mdev, hdcp.auth_state); */
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_work_queue
 *-----------------------------------------------------------------------------
 */
static void hdcp_work_queue(struct work_struct *work)
{
	struct hdcp_delayed_work *hdcp_w = container_of(work, struct hdcp_delayed_work, work.work);
	int event = hdcp_w->event;

	mutex_lock(&hdcp.lock);

	HDCP_DEBUG("hdcp_work_queue() - START - %u hdmi=%d hdcp=%d auth=%d evt= %x %d",
		jiffies_to_msecs(jiffies),
		hdcp.hdmi_state,
		hdcp.hdcp_state,
		hdcp.auth_state,
		(event & 0xFF00) >> 8,
		event & 0xFF);

	/*
	 * Clear pending_wq_event
	 * In case a delayed work is scheduled from the state machine
	 * "pending_wq_event" is used to memorize pointer on the event to be
	 * able to cancel any pending work in case HDCP is disabled
	 */
	if (event & HDCP_WORKQUEUE_SRC)
		hdcp.pending_wq_event = 0;

	/* First handle HDMI state */
	if (event == HDCP_START_FRAME_EVENT) {
		hdcp.pending_start = 0;
		hdcp.hdmi_state = HDMI_STARTED;
	}

	if (event == HDCP_DISABLE_CTL)  {
		hdcp.retry_cnt = 0;
		hdcp_wq_authentication_failure();
	}

	/* HDCP state machine */
	switch (hdcp.hdcp_state) {
	case HDCP_DISABLED:
		/* HDCP enable control or re-authentication event */
		if (event == HDCP_ENABLE_CTL) {

			if (hdcp.en_ctrl->nb_retry == 0)
				hdcp.retry_cnt = HDCP_INFINITE_REAUTH;
			else
				hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;

			if (hdcp.hdmi_state == HDMI_STARTED)
				hdcp_wq_start_authentication();
			else
				hdcp.hdcp_state = HDCP_ENABLE_PENDING;
		}
		break;

	case HDCP_ENABLE_PENDING:
		/* HDMI start frame event */
		if (event == HDCP_START_FRAME_EVENT)
			hdcp_wq_start_authentication();
		break;

	case HDCP_AUTHENTICATION_START:
		/* Re-authentication */
		if (event == HDCP_AUTH_REATT_EVENT)
			hdcp_wq_start_authentication();
		break;

	case HDCP_WAIT_R0_DELAY:
		/* R0 timer elapsed */
		if (event == HDCP_R0_EXP_EVENT)
			hdcp_wq_check_r0();
		break;

	case HDCP_WAIT_KSV_LIST:

		if (event == HDCP_RI_FAIL_EVENT) {
			/* Ri failure */
			HDCP_ERROR("Ri check failure");
			hdcp_wq_authentication_failure();
		} else if (event == HDCP_KSV_LIST_RDY_EVENT) {
			/* KSV list ready event */
			hdcp_wq_step2_authentication();
		} else if (event == HDCP_KSV_TIMEOUT_EVENT) {
			/* Timeout */
			HDCP_ERROR("BCAPS polling timeout\n");
			hdcp_wq_authentication_failure();
		}
		break;

	case HDCP_LINK_INTEGRITY_CHECK:
		hdcp_wq_auto_check_r0();
		/* Ri failure */
		if (event == HDCP_RI_FAIL_EVENT) {
			HDCP_ERROR("Ri check failure\n");
			hdcp_wq_authentication_failure();
		}
		break;

	default:
		HDCP_ERROR("error - unknow HDCP state\n");
		break;
	} /* end of switch (hdcp.hdcp_state) */

	kfree(hdcp_w);
	hdcp_w = 0;
	if (event == HDCP_START_FRAME_EVENT)
		hdcp.pending_start = 0;

	if (event == HDCP_KSV_LIST_RDY_EVENT || event == HDCP_R0_EXP_EVENT)
		hdcp.pending_wq_event = 0;

	HDCP_DEBUG("hdcp_work_queue() - END - %u hdmi=%d hdcp=%d auth=%d evt=%x %d ",
		jiffies_to_msecs(jiffies),
		hdcp.hdmi_state,
		hdcp.hdcp_state,
		hdcp.auth_state,
		(event & 0xFF00) >> 8,
		event & 0xFF);

	mutex_unlock(&hdcp.lock);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_submit_work
 *-----------------------------------------------------------------------------
 */
static struct delayed_work *hdcp_submit_work(int event, int delay)
{
	struct hdcp_delayed_work *work;

	if (hdcp.hdcp_enabled == 0) {
		HDCP_INFO("Cancel submit work because hdcp_enabled is 0");
		return 0;
	}

	work = kmalloc(sizeof(struct hdcp_delayed_work), GFP_ATOMIC);

	if (work) {
		INIT_DELAYED_WORK(&work->work, hdcp_work_queue);
		work->event = event;
		queue_delayed_work(hdcp.workqueue,
			&work->work, msecs_to_jiffies(delay));
	} else {
		HDCP_ERROR("Cannot allocate memory to create work");
		return 0;
	}

	return &work->work;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_cancel_work
 *-----------------------------------------------------------------------------
 */
static void hdcp_cancel_work(struct delayed_work **work)
{
	int ret = 0;

	if (*work) {
		ret = cancel_delayed_work(*work);

		if (ret != 1) {
			ret = cancel_work_sync(&((*work)->work));
			HDCP_ERROR("Canceling work failed - cancel_work_sync done %d", ret);
		}

		kfree(*work);
		*work = 0;
	}
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_enable_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_enable_ctl(void __user *argp)
{
	long ret_val;

	HDCP_INFO("%s", __func__);

	mutex_lock(&hdcp.lock);

	ret_val = -EFAULT;

	hdcp.hdcp_enabled = 1;

	if (hdcp.en_ctrl == 0) {
		hdcp.en_ctrl = kmalloc(sizeof(struct hdcp_enable_control), GFP_KERNEL);

		if (hdcp.en_ctrl == 0) {
			HDCP_ERROR("Cannot allocate memory for HDCP enable control struct");
			goto exit;
		}
	}

	if (copy_from_user(hdcp.en_ctrl, argp, sizeof(struct hdcp_enable_control))) {
		HDCP_ERROR("Error copying from user space - enable ioctl");
		goto exit;
	}

	/* Post event to workqueue */
	if (hdcp_submit_work(HDCP_ENABLE_CTL, 0) == 0)
		goto exit;

	ret_val = 0;

exit:
	mutex_unlock(&hdcp.lock);
	return ret_val;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_disable_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_disable_ctl(void)
{
	HDCP_INFO("%s", __func__);

	hdcp_cancel_work(&hdcp.pending_start);
	hdcp_cancel_work(&hdcp.pending_wq_event);

	mutex_lock(&hdcp.lock);

	hdcp.hdcp_enabled = 0;

	hdcp_lib_auto_ri_check(false);

	/* Variable init */
	/* hdcp.en_ctrl  = 0; */
	hdcp.hdcp_state = HDCP_DISABLED;
	hdcp.pending_start = 0;
	hdcp.pending_wq_event = 0;
	hdcp.retry_cnt = 0;
	hdcp.reauth_delay = 0;
	hdcp.auth_state = HDCP_STATE_DISABLED;
	hdcp.err_code = HDCP_UNKNOWN_STATE;
	hdcp.hdcp_up_event = 0;
	hdcp.hdcp_down_event = 0;
	hdcp.hpd_low = 0;
	hdcp.hdmi_state = HDMI_STARTED;

	hdcp_lib_set_encryption(HDCP_ENC_OFF);

	mutex_unlock(&hdcp.lock);

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_query_status_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_query_status_ctl(void __user *argp)
{
	uint32_t status;

	HDCP_DEBUG("[%s] %s  %d :%u", __FILE__, __func__, __LINE__, jiffies_to_msecs(jiffies));

	status = hdcp.auth_state;

	if (copy_to_user(argp, &status, sizeof(status))) {
		HDCP_ERROR("%s:failed to copy to user !", __func__);
		return -EFAULT;
	}

	return 0;
}


static long hdcp_query_sink_hdcp_capable_ctl(void __user *argp)
{
	uint32_t hdcp_capable;

	HDCP_DEBUG("[%s] %s  %d :%u", __FILE__, __func__, __LINE__, jiffies_to_msecs(jiffies));

	hdcp_capable = hdcp_lib_query_sink_hdcp_capable();

	if (hdcp_capable == HDCP_INCAPABLE) {
		hdcp_set_state(&mdev, hdcp.auth_state);
		hdcp_capable = HDCP_INCAPABLE;
		HDCP_INFO("[%s] HDCP_INCAPABLE, auth_state(%d)", __func__, hdcp.auth_state);
	}

	if (copy_to_user(argp, &hdcp_capable, sizeof(hdcp_capable))) {
		HDCP_ERROR("%s:failed to copy to user !", __func__);
		return -EFAULT;
	}

	return 0;
}

static long hdcp_get_downstream_KSVlist_ctl(void __user *argp)
{
	HDCP_DEBUG("[%s] %s  %d :%u", __FILE__, __func__, __LINE__, jiffies_to_msecs(jiffies));

	if (copy_to_user(argp, &ksvlist_info, sizeof(struct hdcp_ksvlist_info))) {
		HDCP_DEBUG("[%s]ksvlist info failed to copy to user !", __func__);
		return -EFAULT;
	}

	return 0;
}

static long hdcp_set_param_key14(void __user *argp)
{
	long ret;
	struct hdcp_key14_param key;

	if (copy_from_user(&key, argp, sizeof(key))) {
		HDCP_DEBUG("[%s]failed to copy from user !\n", __func__);
		return -EFAULT;
	}

	ret = (long)ta_hdcp_set_param_key((unsigned char *)&key);

	return ret;
}

int hdcp_open(struct inode *inode, struct file *filp)
{
	if (nonseekable_open(inode, filp))
		return -ENODEV;

	return 0;
}
/*-----------------------------------------------------------------------------
 * Function: hdcp_ioctl
 *-----------------------------------------------------------------------------
 */
long hdcp_ioctl(struct file *fd, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	HDCP_DEBUG("ioctl TYPE(0x%x) NR(%u) SIZE(%u)",
		_IOC_TYPE(cmd), _IOC_NR(cmd), _IOC_SIZE(cmd));

	ta_hdcp14_init();

	switch (cmd) {
	case HDCP_SET_PARAM_KEY:
		return hdcp_set_param_key14(argp);
	case HDCP_ENABLE:
		return hdcp_enable_ctl(argp);

	case HDCP_DISABLE:
		return hdcp_disable_ctl();

	case HDCP_QUERY_STATUS:
		return hdcp_query_status_ctl(argp);

	case HDCP_QUERY_SINK_HDCP_CAPABLE:
		return hdcp_query_sink_hdcp_capable_ctl(argp);

	case HDCP_GET_DOWNSTREAM_KSVLIST:
		return hdcp_get_downstream_KSVlist_ctl(argp);

	default:
		return -ENOTTY;
	} /* End switch */
}

#ifdef CONFIG_64BIT
long compat_hdcp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	if (!file->f_op->unlocked_ioctl)
		return -ENOTTY;
	else
		return file->f_op->unlocked_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

/******************************************************************************
 * HDCP driver init/exit
 *****************************************************************************/

const struct file_operations hdcp_fops = {
	.owner = THIS_MODULE,
	.open = hdcp_open,
	.unlocked_ioctl = hdcp_ioctl,
#ifdef CONFIG_64BIT
	.compat_ioctl = compat_hdcp_ioctl,
#endif
};

static ssize_t hdcp_state_show(struct device *device, struct device_attribute *attr, char *buffer)
{
	HDCP_DEBUG("%s\n", __func__);
	return sprintf(buffer, "%d", hdcp.auth_state);
}

static ssize_t hdcp_err_show(struct device *device, struct device_attribute *attr, char *buffer)
{
	HDCP_DEBUG("%s\n", __func__);
	return sprintf(buffer, "%d", hdcp.err_code);
}

static ssize_t hdcp_name_show(struct device *device, struct device_attribute *attr, char *buffer)
{
	HDCP_DEBUG("%s\n", __func__);
	return sprintf(buffer, "%s\n", "hdcp");
}

static ssize_t hdcp_hdcp_en_show(struct device *device,
	struct device_attribute *attr, char *buffer)
{
	ssize_t ret_count;
	int hdcp_en;

	HDCP_DEBUG("%s\n", __func__);

	ta_hdcp14_init();
	hdcp_en = ta_hdcp_get_ctrl_state();

	if (hdcp_en == 2)
		ret_count = sprintf(buffer, "2.x\n");
	else if (hdcp_en == 1)
		ret_count = sprintf(buffer, "1.4\n");
	else
		ret_count = sprintf(buffer, "Disable\n");

	return ret_count;
}

static ssize_t hdcp_hdcp_info_show(struct device *device,
	struct device_attribute *attr, char *buffer)
{
	ssize_t ret_count;

	ret_count = (ssize_t)hdcp_lib_get_info(buffer);

	return ret_count;
}

void hdcp_set_state(struct miscdevice *pdev, int state)
{
	char name_buf[120];
	char state_buf[120];
	char *prop_buf;
	char *envp[3];
	int env_offset = 0;
	int length;

	prop_buf = (char *)get_zeroed_page(GFP_KERNEL);
	if (prop_buf) {
		length = hdcp_name_show(pdev->this_device, NULL, prop_buf);
		if (length > 0) {
			if (prop_buf[length - 1] == '\n')
				prop_buf[length - 1] = 0;
			snprintf(name_buf, sizeof(name_buf), "HDCP1x_NAME=%s", prop_buf);
			envp[env_offset++] = name_buf;
		}

		length = hdcp_state_show(pdev->this_device, NULL, prop_buf);
		if (length > 0) {
			if (prop_buf[length - 1] == '\n')
				prop_buf[length - 1] = 0;
			snprintf(state_buf, sizeof(state_buf), "HDCP1x_STATE=%s", prop_buf);
			envp[env_offset++] = state_buf;
		}

		envp[env_offset] = NULL;
		kobject_uevent_env(&pdev->this_device->kobj, KOBJ_CHANGE, envp);
		free_page((unsigned long)prop_buf);
	} else {
		HDCP_ERROR("Out of memory in %s\n", __func__);
		kobject_uevent(&pdev->this_device->kobj, KOBJ_CHANGE);
	}

}

/* /sys/class/misc/hdcp/state */
static DEVICE_ATTR(state, 0444, hdcp_state_show, NULL);
/* /sys/class/misc/hdcp/err_code */
static DEVICE_ATTR(err_code, 0444, hdcp_err_show, NULL);
/* /sys/class/misc/hdcp/name */
static DEVICE_ATTR(name, 0444, hdcp_name_show, NULL);
/* /sys/class/misc/hdcp/hdcp_en */
static DEVICE_ATTR(hdcp_en, 0444, hdcp_hdcp_en_show, NULL);
/* /sys/class/misc/hdcp/hdcp_info */
static DEVICE_ATTR(hdcp_info, 0444, hdcp_hdcp_info_show, NULL);

static int rtk_hdcptx_probe(struct platform_device *pdev)
{
	struct device_node *syscon_np;
	int ret = 0;

	HDCP_INFO("Driver init");

	syscon_np = of_parse_phandle(pdev->dev.of_node, "syscon", 0);
	if (IS_ERR_OR_NULL(syscon_np))
		return -ENODEV;

	hdcp.hdcp_base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(hdcp.hdcp_base)) {
		of_node_put(syscon_np);
		return -EINVAL;
	}

	hdcp.clk_hdmi = clk_get(&pdev->dev, "clk_en_hdmi");
	if (IS_ERR(hdcp.clk_hdmi)) {
		HDCP_ERROR("Can't get clk clk_hdmi");
		return -EINVAL;
	}

	hdcp.hdcp_irq_num = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!hdcp.hdcp_irq_num)
		HDCP_ERROR("map hdcp irq num failed");
	else
		HDCP_INFO("hdcp_irq_num=%d", hdcp.hdcp_irq_num);

	mutex_init(&hdcp.lock);

	mdev.minor = MISC_DYNAMIC_MINOR;
	mdev.name = "hdcp";
	mdev.mode = 0666;
	mdev.fops = &hdcp_fops;

	if (misc_register(&mdev)) {
		HDCP_ERROR("Could not add character driver\n");
		goto err_register;
	}

	mutex_lock(&hdcp.lock);

	/* Variable init */
	hdcp.en_ctrl  = 0;
	hdcp.hdcp_state = HDCP_DISABLED;
	hdcp.pending_start = 0;
	hdcp.pending_wq_event = 0;
	hdcp.retry_cnt = 0;
	hdcp.reauth_delay = 0;
	hdcp.auth_state = HDCP_STATE_DISABLED;
	hdcp.err_code = HDCP_UNKNOWN_STATE;
	hdcp.hdcp_up_event = 0;
	hdcp.hdcp_down_event = 0;
	hdcp.hpd_low = 0;
	hdcp.hdcp_enabled = -1;

	/* Init sysfs */
	ret = device_create_file(mdev.this_device, &dev_attr_state);
	if (ret < 0)
		goto err_add_driver;

	ret = device_create_file(mdev.this_device, &dev_attr_err_code);
	if (ret < 0)
		goto err_add_driver;

	ret = device_create_file(mdev.this_device, &dev_attr_name);
	if (ret < 0)
		goto err_add_driver;

	ret = device_create_file(mdev.this_device, &dev_attr_hdcp_en);
	if (ret < 0)
		goto err_add_driver;

	ret = device_create_file(mdev.this_device, &dev_attr_hdcp_info);
	if (ret < 0)
		goto err_add_driver;

	spin_lock_init(&hdcp.spinlock);

	hdcp.workqueue = create_singlethread_workqueue("hdcp");
	if (hdcp.workqueue == NULL)
		goto err_add_driver;

	hdcp.hdmi_state = HDMI_STARTED;

	memset(&ksvlist_info, 0x0, sizeof(struct hdcp_ksvlist_info));
	mutex_unlock(&hdcp.lock);

	return 0;

err_add_driver:
	misc_deregister(&mdev);

err_register:
	mutex_destroy(&hdcp.lock);

err_map_hdcp:
	return -EFAULT;
}

static int rtk_hdcptx_suspend(struct device *dev)
{
	HDCP_INFO("Enter %s", __func__);

	if (hdcp.hdcp_enabled == 1)
		hdcp_disable_ctl();

	/* tee_client should be closed before enter suspend */
	ta_hdcp14_deinit();

	HDCP_INFO("Exit %s", __func__);

	return 0;
}

static int rtk_hdcptx_resume(struct device *dev)
{
	HDCP_INFO("Enter %s", __func__);
	HDCP_INFO("Exit %s", __func__);

	return 0;
}

static void rtk_hdcptx_shutdown(struct platform_device *pdev)
{
	HDCP_INFO("Enter %s", __func__);

	if (hdcp.hdcp_enabled == 1)
		hdcp_disable_ctl();

	ta_hdcp14_deinit();

	HDCP_INFO("Exit %s", __func__);
}

static const struct of_device_id rtk_hdcptx_dt_ids[] = {
	{ .compatible = "realtek,rtk129x-hdcptx", },
	{ .compatible = "realtek,rtk139x-hdcptx", },
	{ .compatible = "realtek,rtd161x-hdcptx", },
	{ .compatible = "realtek,rtd13xx-hdcptx", },
	{ .compatible = "realtek,rtd16xxb-hdcptx", },
	{ .compatible = "realtek,rtd13xxd-hdcptx", },
	{},
};
MODULE_DEVICE_TABLE(of, rtk_hdcptx_dt_ids);

static const struct dev_pm_ops rtk_hdcptx_pm_ops = {
	.suspend    = rtk_hdcptx_suspend,
	.resume     = rtk_hdcptx_resume,
	.freeze     = rtk_hdcptx_suspend,
	.thaw       = rtk_hdcptx_resume,
};

static struct platform_driver rtk_hdcptx_driver = {
	.probe = rtk_hdcptx_probe,
	.driver = {
		.name = "rtk_hdcptx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rtk_hdcptx_dt_ids),
#ifdef CONFIG_PM
		.pm = &rtk_hdcptx_pm_ops,
#endif
	},
	.shutdown = rtk_hdcptx_shutdown,
};

static int __init hdcp_init(void)
{
	if (platform_driver_register(&rtk_hdcptx_driver)) {
		HDCP_ERROR("Could not register platform driver");
		goto err_register;
	}

	return 0;

err_register:
	platform_driver_unregister(&rtk_hdcptx_driver);

	return -EFAULT;
}

static void __exit hdcp_exit(void)
{
	mutex_lock(&hdcp.lock);

	kfree(hdcp.en_ctrl);

	misc_deregister(&mdev);

	/* Unmap HDCP */

	destroy_workqueue(hdcp.workqueue);

	mutex_unlock(&hdcp.lock);

	mutex_destroy(&hdcp.lock);
}

module_init(hdcp_init);
module_exit(hdcp_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Realtek HDCP kernel module");
