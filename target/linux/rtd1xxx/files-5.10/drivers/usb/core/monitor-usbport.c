// SPDX-License-Identifier: GPL-2.0
/*
 * USB port monitor
 *
 * Copyright (C) 2022 Realtek Semiconductor Corporation
 */

/* #define DEBUG */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/of.h>
#include <linux/kthread.h>
#include "usb.h"

#define USBPORT_MONITOR_CHECK_TIME (1000 * 60) /* 1 minute */

struct usbport_monitor_data {
	struct class *class;

	struct list_head ports;
	struct notifier_block nb;
	int count; /* Amount of connected matching devices */

	struct mutex lock;
	struct kthread_worker *wq;
	struct kthread_delayed_work check_work;

};

struct usbport_monitor_port {
	struct device dev;

	struct usbport_monitor_data *data;
	struct usb_device *hub;
	int portnum;
	char *port_name;
	bool observed;
	struct list_head list;

	struct usb_device *attached_usb;
};

static struct usbport_monitor_data *g_usbport_data;

/***************************************
 * Helpers
 ***************************************/

/**
 * usbport_monitor_usb_dev_observed - Check if dev is connected to observed port
 */
static bool usbport_monitor_usb_dev_observed(struct usbport_monitor_data *usbport_data,
					  struct usb_device *usb_dev)
{
	struct usbport_monitor_port *port;
	bool observed = false;

	if (!usb_dev->parent)
		goto out;

	list_for_each_entry(port, &usbport_data->ports, list) {
		if (usb_dev->parent == port->hub &&
		    usb_dev->portnum == port->portnum) {
			dev_dbg(&usb_dev->dev, "%s: %s on %s port (%s)\n",
				    __func__,
				    dev_name(&usb_dev->dev),
				    port->observed?"observed":"non-observed",
				    port->port_name);
			observed = port->observed;
			goto out;
		}
	}

	dev_dbg(&usb_dev->dev, "%s: No match monitor port\n", __func__);

out:
	return observed;
}

/***************************************
 * Device attr
 ***************************************/

static ssize_t observed_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct usbport_monitor_port *port = container_of(dev,
		     struct usbport_monitor_port, dev);

	return sprintf(buf, "%d\n", port->observed) + 1;
}

static ssize_t observed_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct usbport_monitor_port *port = container_of(dev,
		    struct usbport_monitor_port, dev);

	if (!strcmp(buf, "0") || !strcmp(buf, "0\n"))
		port->observed = 0;
	else if (!strcmp(buf, "1") || !strcmp(buf, "1\n"))
		port->observed = 1;
	else
		return -EINVAL;

	return size;
}
static DEVICE_ATTR_RW(observed);

static ssize_t attached_usb_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct usbport_monitor_port *port = container_of(dev,
		     struct usbport_monitor_port, dev);

	if (port->attached_usb) {
		struct usb_device *usb = port->attached_usb;

		return sprintf(buf, "%s\n", dev_name(&usb->dev)) + 1;
	}
	return sprintf(buf, "no attached usb\n") + 1;
}

static DEVICE_ATTR_RO(attached_usb);

static struct attribute *ports_attrs[] = {
	&dev_attr_observed.attr,
	&dev_attr_attached_usb.attr,
	NULL,
};

static const struct attribute_group ports_group = {
	.attrs = ports_attrs,
};

static const struct attribute_group *ports_groups[] = {
	&ports_group,
	NULL,
};

static void ports_release(struct device *dev)
{
	/* nothing */
}

static const struct device_type port_dev_type = {
	.name = "usbports",
	.groups = ports_groups,
	.release = ports_release,
};

/***************************************
 * Adding & removing ports
 ***************************************/

/**
 * usbport_monitor_port_observed - Check if port should be observed
 */
static bool usbport_monitor_port_observed(
	    struct usbport_monitor_data *usbport_data,
	    struct usb_device *usb_dev, int port)
{
	if (!usb_dev->parent) {
		pr_info("%s: %s is roothub, set default observed\n",
			    __func__, dev_name(&usb_dev->dev));
		return true;
	}

	return false;
}

static int usbport_monitor_add_port(struct usbport_monitor_data *usbport_data,
				 struct usb_device *usb_dev,
				 const char *hub_name, int portnum)
{
	struct usbport_monitor_port *port;
	struct device *port_dev;
	size_t len;
	int err;

	dev_dbg(&usb_dev->dev, "%s: add port for hub %s portnum=%d\n",
		    __func__, hub_name, portnum);

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port) {
		err = -ENOMEM;
		goto err_out;
	}

	port->data = usbport_data;
	port->hub = usb_dev;
	port->portnum = portnum;
	port->observed = usbport_monitor_port_observed(usbport_data, usb_dev,
						    portnum);

	len = strlen(hub_name) + 8;
	port->port_name = kzalloc(len, GFP_KERNEL);
	if (!port->port_name) {
		err = -ENOMEM;
		goto err_free_port;
	}
	snprintf(port->port_name, len, "%s-port%d", hub_name, portnum);

	port_dev = &port->dev;
	port_dev->class = usbport_data->class;
	port_dev->type = &port_dev_type;
	dev_set_name(port_dev, port->port_name);
	err = device_register(port_dev);
	if (err) {
		put_device(&port->dev);
		goto err_free_port_name;
	}

	list_add_tail(&port->list, &usbport_data->ports);

	return 0;

err_free_port_name:
	kfree(port->port_name);
err_free_port:
	kfree(port);
err_out:
	return err;
}

static int usbport_monitor_add_usb_dev_ports(struct usb_device *usb_dev,
					  void *data)
{
	struct usbport_monitor_data *usbport_data = data;
	int i;

	dev_dbg(&usb_dev->dev, "%s: check %s if it is hub\n",
		    __func__, dev_name(&usb_dev->dev));

	for (i = 1; i <= usb_dev->maxchild; i++)
		usbport_monitor_add_port(usbport_data, usb_dev,
				      dev_name(&usb_dev->dev), i);

	return 0;
}

static void usbport_monitor_remove_port(struct usbport_monitor_data *usbport_data,
				     struct usbport_monitor_port *port)
{
	struct device *port_dev;

	if (!port) {
		pr_err("%s: ERROR: remove port is NULL\n", __func__);
		return;
	}

	dev_dbg(&port->dev, "%s: remove port %s\n", __func__, port->port_name);
	port_dev = &port->dev;
	list_del(&port->list);
	device_unregister(port_dev);
	kfree(port->port_name);
	kfree(port);
}

static void usbport_monitor_remove_usb_dev_ports(struct usbport_monitor_data *usbport_data,
					      struct usb_device *usb_dev)
{
	struct usbport_monitor_port *port, *tmp;

	dev_dbg(&usb_dev->dev, "%s: remove usb device %s\n",
		    __func__, dev_name(&usb_dev->dev));
	list_for_each_entry_safe(port, tmp, &usbport_data->ports, list) {
		if (port->hub == usb_dev)
			usbport_monitor_remove_port(usbport_data, port);
	}
}

static void usbport_monitor_attach_usb_dev_to_ports(struct usb_device *usb_dev,
					  void *data)
{
	struct usbport_monitor_data *usbport_data = data;
	struct usbport_monitor_port *port;

	if (!usb_dev->parent) {
		dev_err(&usb_dev->dev, "%s: ERROR: usb_dev=%s no parent!\n",
			     __func__, dev_name(&usb_dev->dev));
		return;
	}

	list_for_each_entry(port, &usbport_data->ports, list) {
		if (usb_dev->parent == port->hub &&
		    usb_dev->portnum == port->portnum) {
			dev_info(&usb_dev->dev, "%s: %s attach to %s\n",
				    __func__,
				    dev_name(&usb_dev->dev), port->port_name);
			port->attached_usb = usb_dev;
			return;
		}
	}

	dev_dbg(&usb_dev->dev, "%s: no port to attach %s\n",
		    __func__, dev_name(&usb_dev->dev));
	return;
}

static void usbport_monitor_detach_usb_dev_to_ports(struct usb_device *usb_dev,
					  void *data)
{
	struct usbport_monitor_data *usbport_data = data;
	struct usbport_monitor_port *port;

	if (!usb_dev->parent) {
		dev_err(&usb_dev->dev, "%s: ERROR: usb_dev=%s no parent!\n",
			     __func__, dev_name(&usb_dev->dev));
		return;
	}

	list_for_each_entry(port, &usbport_data->ports, list) {
		if (usb_dev->parent == port->hub &&
		    usb_dev->portnum == port->portnum) {
			dev_info(&usb_dev->dev, "%s: %s detach from %s\n",
				    __func__,
				    dev_name(&usb_dev->dev), port->port_name);
			port->attached_usb = NULL;
			return;
		}
	}

	dev_dbg(&usb_dev->dev, "%s: no port to detach %s\n",
		    __func__, dev_name(&usb_dev->dev));
	return;
}

static int usbport_monitor_scan_usb_dev_ports(struct usb_device *usb_dev,
					  void *data)
{
	struct usbport_monitor_data *usbport_data = data;
	bool observed;

	dev_dbg(&usb_dev->dev, "%s: start scan %s\n",
		    __func__, dev_name(&usb_dev->dev));

	mutex_lock(&usbport_data->lock);
	usbport_monitor_add_usb_dev_ports(usb_dev, data);
	observed = usbport_monitor_usb_dev_observed(usbport_data, usb_dev);
	if (observed)
		usbport_monitor_attach_usb_dev_to_ports(usb_dev, data);
	mutex_unlock(&usbport_data->lock);

	return 0;
}

static int __get_device_descriptor(struct usb_device *dev, unsigned int size)
{
	struct usb_device_descriptor *desc;
	int ret;

	if (size > sizeof(*desc))
		return -EINVAL;
	desc = kmalloc(sizeof(*desc), GFP_NOIO);
	if (!desc)
		return -ENOMEM;
	ret = usb_get_descriptor(dev, USB_DT_DEVICE, 0, desc, size);

	kfree(desc);
	return ret;
}

static void usbport_monitor_check_work(struct kthread_work *work)
{
	struct usbport_monitor_data *usbport_data = container_of(work,
		    struct usbport_monitor_data, check_work.work);
	struct usbport_monitor_port *port, *tmp;

	mutex_lock(&usbport_data->lock);
	list_for_each_entry_safe(port, tmp, &usbport_data->ports, list) {
		if (port->observed && port->attached_usb) {
			struct usb_device *udev = port->attached_usb;
			int ret;

			usb_autoresume_device(udev);
			ret = __get_device_descriptor(udev, 8);
			if (ret < 8) {
				dev_err(&udev->dev,
					    "%s: get descriptor error %d\n",
					    __func__, ret);
				usb_lock_device(udev);
				usb_reset_device(udev);
				usb_unlock_device(udev);
			} else {
				dev_dbg(&udev->dev, "%s: check device: %s OK\n",
				    __func__, dev_name(&udev->dev));
			}
			usb_autosuspend_device(udev);
		}
	}
	mutex_unlock(&usbport_data->lock);

	kthread_queue_delayed_work(usbport_data->wq, &usbport_data->check_work,
		    msecs_to_jiffies(USBPORT_MONITOR_CHECK_TIME));
}

/***************************************
 * Init, exit, etc.
 ***************************************/

static int usbport_monitor_notify(struct notifier_block *nb, unsigned long action,
			       void *data)
{
	struct usbport_monitor_data *usbport_data =
		container_of(nb, struct usbport_monitor_data, nb);
	struct usb_device *usb_dev = data;
	bool observed;

	if (!usb_dev) {
		pr_err("%s: ERROR: No usb_dev!!\n", __func__);
		return -ENODEV;
	}

	switch (action) {
	case USB_DEVICE_ADD:
		mutex_lock(&usbport_data->lock);
		observed = usbport_monitor_usb_dev_observed(usbport_data,
			    usb_dev);
		usbport_monitor_add_usb_dev_ports(usb_dev, usbport_data);
		if (observed)
			usbport_monitor_attach_usb_dev_to_ports(usb_dev,
				    usbport_data);
		mutex_unlock(&usbport_data->lock);
		return NOTIFY_OK;
	case USB_DEVICE_REMOVE:
		observed = usbport_monitor_usb_dev_observed(usbport_data,
			    usb_dev);
		mutex_lock(&usbport_data->lock);
		if (observed)
			usbport_monitor_detach_usb_dev_to_ports(usb_dev,
				    usbport_data);
		usbport_monitor_remove_usb_dev_ports(usbport_data, usb_dev);
		mutex_unlock(&usbport_data->lock);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static int __init usbport_monitor_init(void)
{
	struct usbport_monitor_data *usbport_data;
	struct class *usbport_monitor_class;
	int err = 0;

	usbport_data = kzalloc(sizeof(*usbport_data), GFP_KERNEL);
	if (!usbport_data) {
		err = -ENOMEM;
		goto err_destroy;
	}

	usbport_monitor_class = class_create(THIS_MODULE, "usbport_monitor");
	if (!usbport_monitor_class) {
		err = -ENOMEM;
		goto err_out;
	}

	usbport_data->class = usbport_monitor_class;

	mutex_init(&usbport_data->lock);

	/* List of ports */
	INIT_LIST_HEAD(&usbport_data->ports);
	usb_for_each_dev(usbport_data, usbport_monitor_scan_usb_dev_ports);

	/* Notifications */
	usbport_data->nb.notifier_call = usbport_monitor_notify;
	usb_register_notify(&usbport_data->nb);

	/* Init kthread worker */
	usbport_data->wq = kthread_create_worker(0, "usbport_monitor");
	if (IS_ERR(usbport_data->wq)) {
		err = PTR_ERR_OR_ZERO(usbport_data->wq);
		goto err_free;
	}
	kthread_init_delayed_work(&usbport_data->check_work, usbport_monitor_check_work);

	kthread_queue_delayed_work(usbport_data->wq, &usbport_data->check_work,
		    msecs_to_jiffies(USBPORT_MONITOR_CHECK_TIME));

	g_usbport_data = usbport_data;
	return 0;

err_destroy:
	class_destroy(usbport_monitor_class);
err_free:
	kfree(usbport_data);
err_out:
	return err;
}

static void __exit usbport_monitor_exit(void)
{
	struct usbport_monitor_data *usbport_data;
	struct class *usbport_monitor_class;
	struct usbport_monitor_port *port, *tmp;

	usbport_data = g_usbport_data;
	if (!usbport_data)
		return;

	usb_unregister_notify(&usbport_data->nb);

	mutex_lock(&usbport_data->lock);
	list_for_each_entry_safe(port, tmp, &usbport_data->ports, list) {
		dev_dbg(&port->dev, "%s: remove port %s\n",
		    __func__, port->port_name);
		usbport_monitor_remove_port(usbport_data, port);
	}
	mutex_unlock(&usbport_data->lock);

	usbport_monitor_class = usbport_data->class;
	class_destroy(usbport_monitor_class);
	kfree(usbport_data);
	g_usbport_data = NULL;
}

module_init(usbport_monitor_init);
module_exit(usbport_monitor_exit);

MODULE_AUTHOR("Stanley Chang <stanley_chang@realtek.com>");
MODULE_DESCRIPTION("USB port monitor");
MODULE_LICENSE("GPL v2");
