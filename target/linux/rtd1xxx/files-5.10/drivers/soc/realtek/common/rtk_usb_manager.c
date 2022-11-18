// SPDX-License-Identifier: GPL-2.0-only
/*
 *  rtk-usb-manager.c RTK Manager Driver for All USB.
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 * Copyright (C) 2020 Realtek Semiconductor Corporation
 *
 */

//#define DEBUG

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/gpio/consumer.h>
#include <linux/of_platform.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/suspend.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/sys_soc.h>

#include "rtk_usb.h"

#define CRT_SOFT_RESET1	0x0
#define CRT_SOFT_RESET2	0x4
#define CRT_CLOCK_ENABLE1	0xc

#define REALTEK_USB_MANAGER_STR "realtek,usb-manager"

#define CLOCK_RESET_NAME_MAX 32
#define CLOCK_NAME_USB "clk_en_usb"
#define CLOCK_NAME_USB_PORT0 "clk_en_phy0_to_host"
#define CLOCK_NAME_USB_PORT1 "clk_en_phy1_to_host"
#define CLOCK_NAME_USB_PORT2 "clk_en_phy2_to_host"
#define CLOCK_NAME_USB_PORT3 "clk_en_phy3_to_host"

#define RESET_NAME_USB "usb"
#define RESET_NAME_TYPEC "type_c"
#define RESET_NAME_U2PHY0 "u2phy0"
#define RESET_NAME_U2PHY1 "u2phy1"
#define RESET_NAME_U2PHY2 "u2phy2"
#define RESET_NAME_U2PHY3 "u2phy3"
#define RESET_NAME_U3PHY0 "u3phy0"
#define RESET_NAME_U3PHY1 "u3phy1"
#define RESET_NAME_U3PHY2 "u3phy2"
#define RESET_NAME_U3PHY3 "u3phy3"
#define RESET_NAME_U3MDIO0 "u3mdio0"
#define RESET_NAME_U3MDIO1 "u3mdio1"
#define RESET_NAME_U3MDIO2 "u3mdio2"
#define RESET_NAME_U3MDIO3 "u3mdio3"
#define RESET_NAME_USB_MAC0 "usb_host0"
#define RESET_NAME_USB_MAC1 "usb_host1"
#define RESET_NAME_USB_MAC2 "usb_host2"
#define RESET_NAME_USB_MAC3 "usb_host3"

struct gpio_data {
	struct gpio_desc *gpio_desc;

	int active_port;
	int active_port_mask; /* bit 0~3 mapping to port 0~3*/
	/*port GPIO*/
	/* port power on off */
	bool power_on;
	bool power_low_active;

	struct device_node *node;
};

struct port_data {
	int port_num;

	struct gpio_data *power_gpio;

	/* port mapping to host */
	struct device_node *mac_node;
	struct device_node *slave_mac_node; /* for ohci */
	bool support_usb3;

	int active;
	int enable_mask; /* bit 0: master; bit 1: slave */
	int enable;

	struct device_node *node;

	/* clock and reset */
	char clock[CLOCK_RESET_NAME_MAX];
	char reset_u2phy[CLOCK_RESET_NAME_MAX];
	char reset_u3phy[CLOCK_RESET_NAME_MAX];
	char reset_u3mdio[CLOCK_RESET_NAME_MAX];
	char reset_usb_mac[CLOCK_RESET_NAME_MAX];
};

struct manager_data {
	void __iomem *crt_base;
	struct device *dev;

	struct port_data port0;
	struct port_data port1;
	struct port_data port2;
	struct port_data port3;

	/* usb power gpio */
	struct gpio_data gpio0;
	struct gpio_data gpio1;
	struct gpio_data gpio2;
	struct gpio_data gpio3;

	bool disable_usb;

	bool usb_iso_mode;
	bool en_usb_storage_reprobe;
	bool rescue_usb;
	bool dis_hub_autosuspend;

	struct rtk_usb_ops *rtk_usb;

	struct workqueue_struct *wq_usb_manager;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debug_dir;
#endif

	struct mutex lock;

	/* clock and reset */
	char clock[CLOCK_RESET_NAME_MAX];
	char reset_usb[CLOCK_RESET_NAME_MAX];
	char reset_type_c[CLOCK_RESET_NAME_MAX];
};

static inline bool port_can_enable(struct port_data *port)
{
	bool can_enable = true;

	if (port->port_num < 0)
		can_enable = false;
	if (port->enable_mask == 0)
		can_enable = false;

	return can_enable;
}

static struct manager_data *get_rtk_usb_manager(void)
{
	struct device_node *node = NULL;
	struct platform_device *pdev = NULL;
	static struct manager_data *data;

	if (data == NULL) {
		node = of_find_compatible_node(NULL, NULL,
		    REALTEK_USB_MANAGER_STR);
		if (node != NULL)
			pdev = of_find_device_by_node(node);
		if (pdev != NULL)
			data = platform_get_drvdata(pdev);
	}
	if (data == NULL) {
		pr_err("%s ERROR no manager_data", __func__);
		return NULL;
	}

	return data;
}

int rtk_usb_manager_is_iso_mode(struct device *usb_dev)
{
	struct manager_data *data = NULL;
	struct device *dev = NULL;

	data = get_rtk_usb_manager();
	if (data == NULL) {
		pr_err("%s ERROR no manager_data", __func__);
		return -ENODEV;
	}
	dev = data->dev;
	if (data->usb_iso_mode) {
		dev_dbg(data->dev, "%s is usb_iso_mode", __func__);
		return 1;
	}
	dev_dbg(data->dev, "%s Not usb_iso_mode", __func__);
	return 0;
}
EXPORT_SYMBOL(rtk_usb_manager_is_iso_mode);

int rtk_usb_manager_schedule_work(struct device *usb_dev,
	    struct work_struct *work)
{
	struct manager_data *data = NULL;

	data = get_rtk_usb_manager();
	if (data == NULL) {
		pr_err("%s ERROR no manager_data", __func__);
		return -ENODEV;
	}
	dev_dbg(data->dev, "%s Enter ..\n", __func__);

	dev_info(data->dev, "%s for %s", __func__, dev_name(usb_dev));

	if (work == NULL) {
		dev_err(data->dev, "%s, work is NULL", __func__);
		return -1;
	}
	mutex_lock(&data->lock);

	queue_work(data->wq_usb_manager, work);

	mutex_unlock(&data->lock);

	dev_dbg(data->dev, "%s Exit ..\n", __func__);
	return 0;
}
EXPORT_SYMBOL(rtk_usb_manager_schedule_work);

/* enable hw_pm (L4 ICG)
 *   The hw_pm function will be reset after doing soft_reset, so
 *   only enable is provided.
 */
static __maybe_unused void __rtk_usb_set_hw_pm_enable(struct manager_data *data)
{
	struct device *dev = data->dev;
	bool on = true;

	if (!data->rtk_usb)
		return;

	dev_dbg(dev, "set usb_hw_pm\n");

	/* for hw_pm, enable is equal to power_off */
	if (port_can_enable(&data->port0))
		rtk_usb_set_hw_l4icg_on_off(data->rtk_usb, USB_PORT_0, on);
	if (port_can_enable(&data->port1))
		rtk_usb_set_hw_l4icg_on_off(data->rtk_usb, USB_PORT_1, on);
	if (port_can_enable(&data->port2))
		rtk_usb_set_hw_l4icg_on_off(data->rtk_usb, USB_PORT_2, on);
	if (port_can_enable(&data->port3))
		rtk_usb_set_hw_l4icg_on_off(data->rtk_usb, USB_PORT_3, on);
}

/* set usb charger power */
static __maybe_unused void __usb_set_charger_power(
	    struct manager_data *data, bool power_on)
{
	struct device *dev = data->dev;
	unsigned int val = 0;

	if (!data->rtk_usb)
		return;

	if (power_on) {
		if (port_can_enable(&data->port0))
			val |= BIT(0);
		if (port_can_enable(&data->port1))
			val |= BIT(1);
		if (port_can_enable(&data->port2))
			val |= BIT(2);
		if (port_can_enable(&data->port3))
			val |= BIT(3);
	}

	dev_dbg(dev, "%s set usb_charger %x\n", __func__, val);

	rtk_usb_set_charger_power(data->rtk_usb, val);
}

/* set usb power domain */
static void __usb_set_pd_power(struct manager_data *data, bool power_on)
{
	//struct device *dev = data->dev;

	if (power_on && !data->disable_usb) {
		if (!data->rtk_usb)
			return;
		rtk_usb_iso_power_ctrl(data->rtk_usb, true);
	} else {
		if (!data->rtk_usb)
			return;
		rtk_usb_iso_power_ctrl(data->rtk_usb, false);
	}
}

static inline struct reset_control *USB_reset_get(struct device *dev,
	    const char *str)
{
	struct reset_control *reset;

	reset = reset_control_get_exclusive(dev, str);
	if (IS_ERR(reset)) {
		dev_dbg(dev, "No controller reset %s\n", str);
		reset = NULL;
	}
	return reset;
}

static inline void USB_reset_put(struct reset_control *reset)
{
	if (reset)
		reset_control_put(reset);
}

static inline int USB_reset_deassert(struct reset_control *reset)
{
	if (!reset)
		return 0;

	return reset_control_deassert(reset);
}

static inline int USB_reset_assert(struct reset_control *reset)
{
	if (!reset)
		return 0;

	return reset_control_assert(reset);
}

static inline struct clk *USB_clk_get(struct device *dev, const char *str)
{
	struct clk *clk;

	clk = clk_get(dev, str);
	if (IS_ERR(clk)) {
		dev_dbg(dev, "No clk %s\n", str);
		clk = NULL;
	}
	return clk;
}

static bool __usb_workaround_port0_enable(struct manager_data *data)
{
	const struct soc_device_attribute *soc_att_match = NULL;
	struct soc_device_attribute rtk_soc[] = {
		{
			.family = "Realtek Phoenix",
		},
		{
			.family = "Realtek Kylin",
		},
		{
			.family = "Realtek Hercules",
		},
		{
			.family = "Realtek Thor",
		},
		{
			.family = "Realtek Hank",
		},
		{
		/* empty */
		}
	};

	if (!data) {
		pr_err("%s ERROR: data is NULL\n", __func__);
		return false;
	}

	soc_att_match = soc_device_match(rtk_soc);
	if (soc_att_match) {
		pr_info("%s Workaround port 0 reset enable match chip: %s\n",
			    __func__,
			    soc_att_match->family);
		return true;
	}
	return false;
}

static int __usb_init_clock_reset(struct manager_data *data)
{
	struct device *dev = data->dev;
	/* GET clock */
	struct clk *clk_usb_phy0_to_host = USB_clk_get(dev, data->port0.clock);
	struct clk *clk_usb_phy1_to_host = USB_clk_get(dev, data->port1.clock);
	struct clk *clk_usb_phy2_to_host = USB_clk_get(dev, data->port2.clock);
	struct clk *clk_usb_phy3_to_host = USB_clk_get(dev, data->port3.clock);
	struct clk *clk_usb = USB_clk_get(dev, data->clock);

	/* GET reset controller */
	struct reset_control *reset_u2phy0 = USB_reset_get(dev,
		    data->port0.reset_u2phy);
	struct reset_control *reset_u2phy1 = USB_reset_get(dev,
		    data->port1.reset_u2phy);
	struct reset_control *reset_u2phy2 = USB_reset_get(dev,
		    data->port2.reset_u2phy);
	struct reset_control *reset_u2phy3 = USB_reset_get(dev,
		    data->port3.reset_u2phy);

	struct reset_control *reset_u3phy0 = USB_reset_get(dev,
		    data->port0.reset_u3phy);
	struct reset_control *reset_u3phy1 = USB_reset_get(dev,
		    data->port1.reset_u3phy);
	struct reset_control *reset_u3phy2 = USB_reset_get(dev,
		    data->port2.reset_u3phy);
	struct reset_control *reset_u3phy3 = USB_reset_get(dev,
		    data->port3.reset_u3phy);
	struct reset_control *reset_u3mdio0 = USB_reset_get(dev,
		    data->port0.reset_u3mdio);
	struct reset_control *reset_u3mdio1 = USB_reset_get(dev,
		    data->port1.reset_u3mdio);
	struct reset_control *reset_u3mdio2 = USB_reset_get(dev,
		    data->port2.reset_u3mdio);
	struct reset_control *reset_u3mdio3 = USB_reset_get(dev,
		    data->port3.reset_u3mdio);

	struct reset_control *reset_usb_mac0 = USB_reset_get(dev,
		    data->port0.reset_usb_mac);
	struct reset_control *reset_usb_mac1 = USB_reset_get(dev,
		    data->port1.reset_usb_mac);
	struct reset_control *reset_usb_mac2 = USB_reset_get(dev,
		    data->port2.reset_usb_mac);
	struct reset_control *reset_usb_mac3 = USB_reset_get(dev,
		    data->port3.reset_usb_mac);
	struct reset_control *reset_usb = USB_reset_get(dev,
		    data->reset_usb);
	struct reset_control *reset_type_c = USB_reset_get(dev,
		    data->reset_type_c);

	struct reset_control *reset_usb_apply = USB_reset_get(dev, "apply");

	dev_dbg(dev, "Realtek USB init\n");

	USB_reset_assert(reset_u2phy0);
	USB_reset_assert(reset_u2phy1);
	USB_reset_assert(reset_u2phy2);
	USB_reset_assert(reset_u2phy3);
	USB_reset_assert(reset_u3phy0);
	USB_reset_assert(reset_u3phy1);
	USB_reset_assert(reset_u3phy2);
	USB_reset_assert(reset_u3phy3);
	USB_reset_assert(reset_u3mdio0);
	USB_reset_assert(reset_u3mdio1);
	USB_reset_assert(reset_u3mdio2);
	USB_reset_assert(reset_u3mdio3);
	USB_reset_assert(reset_usb_mac0);
	USB_reset_assert(reset_usb_mac1);
	USB_reset_assert(reset_usb_mac2);
	USB_reset_assert(reset_usb_mac3);
	USB_reset_assert(reset_usb);
	USB_reset_assert(reset_type_c);
	/* Do wmb */
	wmb();

	// Enable usb phy reset
	/* DEASSERT: set rstn bit to 1 */
	dev_dbg(dev, "Realtek USB init: Set phy reset to 1\n");
	if (port_can_enable(&data->port0)) {
		USB_reset_deassert(reset_u2phy0);
		if (data->port0.support_usb3)
			USB_reset_deassert(reset_u3phy0);
	}
	if (port_can_enable(&data->port1)) {
		USB_reset_deassert(reset_u2phy1);
		if (data->port1.support_usb3)
			USB_reset_deassert(reset_u3phy1);
	}
	if (port_can_enable(&data->port2)) {
		USB_reset_deassert(reset_u2phy2);
		if (data->port2.support_usb3)
			USB_reset_deassert(reset_u3phy2);
	}
	if (port_can_enable(&data->port3)) {
		USB_reset_deassert(reset_u2phy3);
		if (data->port3.support_usb3)
			USB_reset_deassert(reset_u3phy3);
	}

	USB_reset_deassert(reset_usb_apply);

	udelay(300);

	dev_dbg(dev, "Realtek USB init: Trigger usb clk\n");
	// Trigger USB clk (enable -> disable)
	clk_prepare_enable(clk_usb); // = clk_prepare + clk_enable
	clk_disable_unprepare(clk_usb); // = clk_disable + clk_unprepare

	dev_dbg(dev, "Realtek USB init: Set u3phy mdio reset to 1\n");
	// Enable USB reset
	if (port_can_enable(&data->port0) && data->port0.support_usb3)
		USB_reset_deassert(reset_u3mdio0);
	if (port_can_enable(&data->port1) && data->port1.support_usb3)
		USB_reset_deassert(reset_u3mdio1);
	if (port_can_enable(&data->port2) && data->port2.support_usb3)
		USB_reset_deassert(reset_u3mdio2);
	if (port_can_enable(&data->port3) && data->port3.support_usb3)
		USB_reset_deassert(reset_u3mdio3);

	USB_reset_deassert(reset_usb_apply);

	dev_dbg(dev, "Realtek USB init: Set usb reset to 1\n");

	/* port0 mac's reset bit must always enable for some platforms */
	if (port_can_enable(&data->port0) ||
		    __usb_workaround_port0_enable(data))
		USB_reset_deassert(reset_usb_mac0);
	if (port_can_enable(&data->port1))
		USB_reset_deassert(reset_usb_mac1);
	if (port_can_enable(&data->port2))
		USB_reset_deassert(reset_usb_mac2);
	if (port_can_enable(&data->port3))
		USB_reset_deassert(reset_usb_mac3);
	USB_reset_deassert(reset_usb);
	USB_reset_deassert(reset_type_c);
	dev_dbg(dev, "Realtek USB init: enable usb clk\n");

	// Enable USB clk
	clk_prepare_enable(clk_usb); // = clk_prepare + clk_enable
	if (port_can_enable(&data->port0))
		clk_prepare_enable(clk_usb_phy0_to_host);
	if (port_can_enable(&data->port1))
		clk_prepare_enable(clk_usb_phy1_to_host);
	if (port_can_enable(&data->port2))
		clk_prepare_enable(clk_usb_phy2_to_host);
	if (port_can_enable(&data->port3))
		clk_prepare_enable(clk_usb_phy3_to_host);

	dev_info(dev, "Realtek USB init OK\n");

	clk_put(clk_usb);
	clk_put(clk_usb_phy0_to_host);
	clk_put(clk_usb_phy1_to_host);
	clk_put(clk_usb_phy2_to_host);
	clk_put(clk_usb_phy3_to_host);

	USB_reset_put(reset_u2phy0);
	USB_reset_put(reset_u2phy1);
	USB_reset_put(reset_u2phy2);
	USB_reset_put(reset_u2phy3);

	USB_reset_put(reset_u3phy0);
	USB_reset_put(reset_u3phy1);
	USB_reset_put(reset_u3phy2);
	USB_reset_put(reset_u3phy3);
	USB_reset_put(reset_u3mdio0);
	USB_reset_put(reset_u3mdio1);
	USB_reset_put(reset_u3mdio2);
	USB_reset_put(reset_u3mdio3);

	USB_reset_put(reset_usb_mac0);
	USB_reset_put(reset_usb_mac1);
	USB_reset_put(reset_usb_mac2);
	USB_reset_put(reset_usb_mac3);
	USB_reset_put(reset_usb);
	USB_reset_put(reset_type_c);

	USB_reset_put(reset_usb_apply);

	return 0;
}

static int __usb_clear_clock_reset(struct manager_data *data)
{
	struct device *dev = data->dev;
	/* GET clock */
	struct clk *clk_usb_phy0_to_host = USB_clk_get(dev, data->port0.clock);
	struct clk *clk_usb_phy1_to_host = USB_clk_get(dev, data->port1.clock);
	struct clk *clk_usb_phy2_to_host = USB_clk_get(dev, data->port2.clock);
	struct clk *clk_usb_phy3_to_host = USB_clk_get(dev, data->port3.clock);
	struct clk *clk_usb = USB_clk_get(dev, data->clock);

	/* GET reset controller */
	struct reset_control *reset_u2phy0 = USB_reset_get(dev,
		    data->port0.reset_u2phy);
	struct reset_control *reset_u2phy1 = USB_reset_get(dev,
		    data->port1.reset_u2phy);
	struct reset_control *reset_u2phy2 = USB_reset_get(dev,
		    data->port2.reset_u2phy);
	struct reset_control *reset_u2phy3 = USB_reset_get(dev,
		    data->port3.reset_u2phy);

	struct reset_control *reset_u3phy0 = USB_reset_get(dev,
		    data->port0.reset_u3phy);
	struct reset_control *reset_u3phy1 = USB_reset_get(dev,
		    data->port1.reset_u3phy);
	struct reset_control *reset_u3phy2 = USB_reset_get(dev,
		    data->port2.reset_u3phy);
	struct reset_control *reset_u3phy3 = USB_reset_get(dev,
		    data->port3.reset_u3phy);
	struct reset_control *reset_u3mdio0 = USB_reset_get(dev,
		    data->port0.reset_u3mdio);
	struct reset_control *reset_u3mdio1 = USB_reset_get(dev,
		    data->port1.reset_u3mdio);
	struct reset_control *reset_u3mdio2 = USB_reset_get(dev,
		    data->port2.reset_u3mdio);
	struct reset_control *reset_u3mdio3 = USB_reset_get(dev,
		    data->port3.reset_u3mdio);

	struct reset_control *reset_usb_mac0 = USB_reset_get(dev,
		    data->port0.reset_usb_mac);
	struct reset_control *reset_usb_mac1 = USB_reset_get(dev,
		    data->port1.reset_usb_mac);
	struct reset_control *reset_usb_mac2 = USB_reset_get(dev,
		    data->port2.reset_usb_mac);
	struct reset_control *reset_usb_mac3 = USB_reset_get(dev,
		    data->port3.reset_usb_mac);
	struct reset_control *reset_usb = USB_reset_get(dev,
		    data->reset_usb);
	struct reset_control *reset_type_c = USB_reset_get(dev,
		    data->reset_type_c);

	struct reset_control *reset_usb_apply = USB_reset_get(dev, "apply");

	dev_dbg(dev, "Realtek USB clear clock and reset\n");

	USB_reset_assert(reset_u2phy0);
	USB_reset_assert(reset_u2phy1);
	USB_reset_assert(reset_u2phy2);
	USB_reset_assert(reset_u2phy3);
	USB_reset_assert(reset_u3phy0);
	USB_reset_assert(reset_u3phy1);
	USB_reset_assert(reset_u3phy2);
	USB_reset_assert(reset_u3phy3);
	USB_reset_assert(reset_u3mdio0);
	USB_reset_assert(reset_u3mdio1);
	USB_reset_assert(reset_u3mdio2);
	USB_reset_assert(reset_u3mdio3);
	USB_reset_assert(reset_usb_mac0);
	USB_reset_assert(reset_usb_mac1);
	USB_reset_assert(reset_usb_mac2);
	USB_reset_assert(reset_usb_mac3);
	USB_reset_assert(reset_usb);
	USB_reset_assert(reset_type_c);
	/* Do wmb */
	wmb();

	clk_disable_unprepare(clk_usb); // = clk_disable + clk_unprepare
	if (port_can_enable(&data->port0))
		clk_disable_unprepare(clk_usb_phy0_to_host);
	if (port_can_enable(&data->port1))
		clk_disable_unprepare(clk_usb_phy1_to_host);
	if (port_can_enable(&data->port2))
		clk_disable_unprepare(clk_usb_phy2_to_host);
	if (port_can_enable(&data->port3))
		clk_disable_unprepare(clk_usb_phy3_to_host);

	dev_info(dev, "Realtek USB clear clock and reset... OK\n");

	clk_put(clk_usb);
	clk_put(clk_usb_phy0_to_host);
	clk_put(clk_usb_phy1_to_host);
	clk_put(clk_usb_phy2_to_host);
	clk_put(clk_usb_phy3_to_host);

	USB_reset_put(reset_u2phy0);
	USB_reset_put(reset_u2phy1);
	USB_reset_put(reset_u2phy2);
	USB_reset_put(reset_u2phy3);

	USB_reset_put(reset_u3phy0);
	USB_reset_put(reset_u3phy1);
	USB_reset_put(reset_u3phy2);
	USB_reset_put(reset_u3phy3);
	USB_reset_put(reset_u3mdio0);
	USB_reset_put(reset_u3mdio1);
	USB_reset_put(reset_u3mdio2);
	USB_reset_put(reset_u3mdio3);

	USB_reset_put(reset_usb_mac0);
	USB_reset_put(reset_usb_mac1);
	USB_reset_put(reset_usb_mac2);
	USB_reset_put(reset_usb_mac3);
	USB_reset_put(reset_usb);
	USB_reset_put(reset_type_c);

	USB_reset_put(reset_usb_apply);

	return 0;
}

static int __usb_port_clock_reset_enable(struct manager_data *data,
	    struct port_data *port)
{
	struct device *dev = data->dev;
	/* GET clock */
	struct clk *clk_usb_phy_to_host = USB_clk_get(dev, port->clock);

	/* GET reset controller */
	struct reset_control *reset_u2phy = USB_reset_get(dev,
		    port->reset_u2phy);

	struct reset_control *reset_u3phy = USB_reset_get(dev,
		    port->reset_u3phy);
	struct reset_control *reset_u3mdio = USB_reset_get(dev,
		    port->reset_u3mdio);

	struct reset_control *reset_usb_mac = USB_reset_get(dev,
		    port->reset_usb_mac);

	dev_info(dev, "%s: port%d clock/reset enable\n",
		    __func__, port->port_num);

	/* Enable USB port reset */
	if (port_can_enable(port)) {
		USB_reset_deassert(reset_u2phy);
		if (port->support_usb3)
			USB_reset_deassert(reset_u3phy);
	}
	udelay(300);

	if (port_can_enable(port) && port->support_usb3)
		USB_reset_deassert(reset_u3mdio);

	if (port_can_enable(port))
		USB_reset_deassert(reset_usb_mac);

	udelay(300);
	/* Enable USB port clock */
	if (port_can_enable(port))
		clk_prepare_enable(clk_usb_phy_to_host);

	clk_put(clk_usb_phy_to_host);

	USB_reset_put(reset_u2phy);
	USB_reset_put(reset_u3phy);
	USB_reset_put(reset_u3mdio);

	USB_reset_put(reset_usb_mac);

	return 0;
}

static int __usb_port_clock_reset_disable(struct manager_data *data,
	    struct port_data *port)
{
	struct device *dev = data->dev;
	/* GET clock */
	struct clk *clk_usb_phy_to_host = USB_clk_get(dev, port->clock);

	/* GET reset controller */
	struct reset_control *reset_u2phy = USB_reset_get(dev,
		    port->reset_u2phy);

	struct reset_control *reset_u3phy = USB_reset_get(dev,
		    port->reset_u3phy);
	struct reset_control *reset_u3mdio = USB_reset_get(dev,
		    port->reset_u3mdio);

	struct reset_control *reset_usb_mac = USB_reset_get(dev,
		    port->reset_usb_mac);

	dev_info(dev, "%s: port%d clock/reset disable\n",
		    __func__, port->port_num);

	clk_disable_unprepare(clk_usb_phy_to_host);

	/* Do wmb */
	wmb();

	USB_reset_assert(reset_u2phy);
	USB_reset_assert(reset_u3phy);
	USB_reset_assert(reset_u3mdio);
	udelay(300);

	if (port->port_num !=0 ||
		    !__usb_workaround_port0_enable(data))
		USB_reset_assert(reset_usb_mac);

	clk_put(clk_usb_phy_to_host);

	USB_reset_put(reset_u2phy);
	USB_reset_put(reset_u3phy);
	USB_reset_put(reset_u3mdio);

	USB_reset_put(reset_usb_mac);

	return 0;
}

int rtk_usb_type_c_init(struct device *type_c_dev)
{
	struct manager_data *data = NULL;
	struct device *dev = NULL;

	data = get_rtk_usb_manager();
	if (data == NULL) {
		pr_err("%s ERROR no manager_data", __func__);
		return -ENODEV;
	}
	dev = data->dev;
	if (data->rtk_usb)
		rtk_type_c_init(data->rtk_usb);
	else
		dev_info(data->dev, "%s No rtk_usb data", __func__);

	return 0;
}
EXPORT_SYMBOL(rtk_usb_type_c_init);

int rtk_usb_type_c_plug_config(struct device *type_c_dev,
	    int dr_mode, int cc)
{
	struct manager_data *data = NULL;
	struct device *dev = NULL;

	data = get_rtk_usb_manager();
	if (data == NULL) {
		pr_err("%s ERROR no manager_data", __func__);
		return -ENODEV;
	}
	dev = data->dev;
	if (data->rtk_usb)
		rtk_type_c_plug_config(data->rtk_usb, dr_mode, cc);
	else
		dev_info(data->dev, "%s No rtk_usb data", __func__);

	return 0;
}
EXPORT_SYMBOL(rtk_usb_type_c_plug_config);

static int __usb_port_power_on_off(struct device *dev,
	    struct port_data *port, bool on,
	    struct device_node *node);

int rtk_usb_port_power_on_off(struct device *usb_dev, bool on)
{
	struct manager_data *data = NULL;
	int ret;

	data = get_rtk_usb_manager();
	if (data == NULL) {
		pr_err("%s ERROR no manager_data", __func__);
		return -ENODEV;
	}

	dev_dbg(data->dev, "%s power %s for %s", __func__,
			on?"on":"off", dev_name(usb_dev));

	mutex_lock(&data->lock);

	ret = __usb_port_power_on_off(data->dev, &data->port0, on,
		    usb_dev->of_node);
	if (ret)
		ret = __usb_port_power_on_off(data->dev, &data->port1, on,
			    usb_dev->of_node);
	if (ret)
		ret = __usb_port_power_on_off(data->dev, &data->port2, on,
			    usb_dev->of_node);
	if (ret)
		ret = __usb_port_power_on_off(data->dev, &data->port3, on,
			    usb_dev->of_node);

	mutex_unlock(&data->lock);

	return ret;
}
EXPORT_SYMBOL(rtk_usb_port_power_on_off);

static int __gpio_on_off(struct device *dev, int port,
	    struct gpio_data *gpio, bool on)
{
	int ret = 0;
	int active_port_mask = gpio->active_port_mask;
	bool power_low_active = gpio->power_low_active;

	if (IS_ERR(gpio->gpio_desc)) {
		dev_dbg(dev, "%s No gpio config\n", __func__);
		return 0;
	}

	if (on) {
		gpio->active_port |= BIT(port);
		if ((gpio->active_port & active_port_mask) !=
			    active_port_mask) {
			dev_info(dev, "gpio_num=%d active_port=0x%x active_port_mask=0x%x  ==> No turn on\n",
				    desc_to_gpio(gpio->gpio_desc), gpio->active_port,
				    active_port_mask);
			return -1;
		}
	} else {
		gpio->active_port &= ~BIT(port);
		if (gpio->active_port != 0) {
			dev_info(dev, "gpio_num=%d active_port=0x%x active_port_mask=0x%x  ==> No turn off\n",
				    desc_to_gpio(gpio->gpio_desc), gpio->active_port,
				    gpio->active_port_mask);
			return -1;
		}
	}

	if (!IS_ERR(gpio->gpio_desc)) {
		if (gpiod_direction_output(gpio->gpio_desc,
			power_low_active ?  !on:on)) {
			dev_err(dev, "%s ERROR set gpio fail\n",
					__func__);
			ret = -1;
		} else {
			dev_info(dev, "%s to set power%s %s by gpio (id=%d) OK\n",
						__func__,
						power_low_active ?
						" (power_low_active)":"",
						on?"on":"off", desc_to_gpio(gpio->gpio_desc));
			gpio->power_on = on;
		}
		gpiod_put(gpio->gpio_desc);
	}
	return ret;
}

static int __usb_port_power_on_off(struct device *dev,
	    struct port_data *port, bool on,
	    struct device_node *node)
{
	struct device_node *usb_node = node;
	int enable_mask = port->enable_mask;

	if (port->mac_node && usb_node &&
		    port->mac_node->phandle == usb_node->phandle) {
		if (on)
			port->active |= BIT(0);
		else
			port->active &= ~BIT(0);
	} else if (port->slave_mac_node && usb_node &&
		    port->slave_mac_node->phandle == usb_node->phandle) {
		if (on)
			port->active |= BIT(1);
		else
			port->active &= ~BIT(1);
	} else {
		dev_dbg(dev, "%s port%d is not match\n",
			    __func__, port->port_num);
		return -ENODEV;
	}

	dev_info(dev, "%s port%d power=%s active=0x%x enable_mask=0x%x\n",
		    __func__, port->port_num, on?"on":"off",
		    port->active, port->enable_mask);
	if (((port->active & enable_mask) == enable_mask) ||
		    (!port->active && !on)) {
		if (port->power_gpio)
			__gpio_on_off(dev, port->port_num,
				    port->power_gpio, on);
		else
			dev_info(dev, "%s port%d no gpio\n",
				    __func__, port->port_num);
	}
	return 0;
}

static int __usb_port_suspend(struct manager_data *data)
{
	struct device *dev = data->dev;
	bool is_suspend = true;

	if (!data->rtk_usb)
		return 0;

	dev_info(dev, "%s", __func__);
	if (port_can_enable(&data->port0)) {
		dev_dbg(dev, "set port 0 phy suspend\n");
		rtk_usb_port_suspend_resume(data->rtk_usb, USB_PORT_0,
			    is_suspend);
	}
	if (port_can_enable(&data->port1)) {
		dev_dbg(dev, "set port 1 phy suspend\n");
		rtk_usb_port_suspend_resume(data->rtk_usb, USB_PORT_1,
			    is_suspend);
	}
	if (port_can_enable(&data->port2)) {
		dev_dbg(dev, "set port 2 phy suspend\n");
		rtk_usb_port_suspend_resume(data->rtk_usb, USB_PORT_2,
			    is_suspend);
	}
	if (port_can_enable(&data->port3)) {
		dev_dbg(dev, "set port 3 phy suspend\n");
		rtk_usb_port_suspend_resume(data->rtk_usb, USB_PORT_3,
			    is_suspend);
	}
	return 0;
}

static int __usb_port_resume(struct manager_data *data)
{
	struct device *dev = data->dev;
	//void __iomem *reg_u3_port;
	//void __iomem *reg_u2_port;
	bool is_suspend = false;

	if (!data->rtk_usb)
		return 0;

	dev_info(dev, "%s", __func__);
	if (port_can_enable(&data->port0)) {
		dev_dbg(dev, "set port 0 phy resume\n");
		rtk_usb_port_suspend_resume(data->rtk_usb, USB_PORT_0,
			    is_suspend);
	}
	if (port_can_enable(&data->port1)) {
		dev_dbg(dev, "set port 1 phy resume\n");
		rtk_usb_port_suspend_resume(data->rtk_usb, USB_PORT_1,
			    is_suspend);
	}
	if (port_can_enable(&data->port2)) {
		dev_dbg(dev, "set port 2 phy resume\n");
		rtk_usb_port_suspend_resume(data->rtk_usb, USB_PORT_2,
			    is_suspend);
	}
	if (port_can_enable(&data->port3)) {
		dev_dbg(dev, "set port 3 phy resume\n");
		rtk_usb_port_suspend_resume(data->rtk_usb, USB_PORT_3,
			    is_suspend);
	}
	return 0;
}

static int __usb_port_gpio_off(struct manager_data *data)
{
	struct device *dev = data->dev;
	struct device_node *usb_node;
	bool off = false;

	dev_info(dev, "%s", __func__);
	mutex_lock(&data->lock);

	usb_node = data->port0.mac_node;
	__usb_port_power_on_off(data->dev, &data->port0, off, usb_node);
	usb_node = data->port0.slave_mac_node;
	__usb_port_power_on_off(data->dev, &data->port0, off, usb_node);

	usb_node = data->port1.mac_node;
	__usb_port_power_on_off(data->dev, &data->port1, off, usb_node);
	usb_node = data->port1.slave_mac_node;
	__usb_port_power_on_off(data->dev, &data->port1, off, usb_node);

	usb_node = data->port2.mac_node;
	__usb_port_power_on_off(data->dev, &data->port2, off, usb_node);
	usb_node = data->port2.slave_mac_node;
	__usb_port_power_on_off(data->dev, &data->port2, off, usb_node);

	usb_node = data->port3.mac_node;
	__usb_port_power_on_off(data->dev, &data->port3, off, usb_node);
	usb_node = data->port3.slave_mac_node;
	__usb_port_power_on_off(data->dev, &data->port3, off, usb_node);

	mutex_unlock(&data->lock);

	return 0;
}

static int __usb_port_gpio_on(struct manager_data *data)
{
	struct device *dev = data->dev;
	struct device_node *usb_node;
	bool on = true;

	dev_info(dev, "%s", __func__);

	mutex_lock(&data->lock);

	usb_node = data->port0.mac_node;
	__usb_port_power_on_off(data->dev, &data->port0, on, usb_node);
	usb_node = data->port0.slave_mac_node;
	__usb_port_power_on_off(data->dev, &data->port0, on, usb_node);

	usb_node = data->port1.mac_node;
	__usb_port_power_on_off(data->dev, &data->port1, on, usb_node);
	usb_node = data->port1.slave_mac_node;
	__usb_port_power_on_off(data->dev, &data->port1, on, usb_node);

	usb_node = data->port2.mac_node;
	__usb_port_power_on_off(data->dev, &data->port2, on, usb_node);
	usb_node = data->port2.slave_mac_node;
	__usb_port_power_on_off(data->dev, &data->port2, on, usb_node);

	usb_node = data->port3.mac_node;
	__usb_port_power_on_off(data->dev, &data->port3, on, usb_node);
	usb_node = data->port3.slave_mac_node;
	__usb_port_power_on_off(data->dev, &data->port3, on, usb_node);

	mutex_unlock(&data->lock);

	return 0;
}

static int __usb_port_enable(struct manager_data *data, struct port_data *port,
	    struct device_node *usb_node, bool enable)
{
	if (port->mac_node && usb_node &&
		    port->mac_node->phandle == usb_node->phandle) {
		if (enable)
			port->enable |= BIT(0);
		else
			port->enable &= ~BIT(0);
	} else if (port->slave_mac_node && usb_node &&
	    port->slave_mac_node->phandle == usb_node->phandle) {
		if (enable)
			port->enable |= BIT(1);
		else
			port->enable &= ~BIT(1);
	}
	return 0;
}

int rtk_usb_init_port_power_on_off(struct device *usb_dev, bool on)
{
	struct device_node *usb_node;
	static struct manager_data *data;

	data = get_rtk_usb_manager();
	if (data == NULL) {
		pr_err("%s ERROR no manager_data", __func__);
		return -ENODEV;
	}
	if (usb_dev == NULL) {
		dev_err(data->dev, "%s ERROR no usb_dev", __func__);
		return -ENODEV;
	}

	usb_node = usb_dev->of_node;

	dev_info(data->dev, "%s for %s", __func__, dev_name(usb_dev));
	mutex_lock(&data->lock);

	if (!__usb_port_power_on_off(data->dev, &data->port0, on, usb_node))
		__usb_port_enable(data, &data->port0, usb_node, true);
	if (!__usb_port_power_on_off(data->dev, &data->port1, on, usb_node))
		__usb_port_enable(data, &data->port1, usb_node, true);
	if (!__usb_port_power_on_off(data->dev, &data->port2, on, usb_node))
		__usb_port_enable(data, &data->port2, usb_node, true);
	if (!__usb_port_power_on_off(data->dev, &data->port3, on, usb_node))
		__usb_port_enable(data, &data->port3, usb_node, true);

	mutex_unlock(&data->lock);
	return 0;
}
EXPORT_SYMBOL(rtk_usb_init_port_power_on_off);

int rtk_usb_remove_port_power_on_off(struct device *usb_dev, bool on)
{
	struct device_node *usb_node;
	static struct manager_data *data;

	data = get_rtk_usb_manager();
	if (data == NULL) {
		pr_err("%s ERROR no manager_data", __func__);
		return -ENODEV;
	}
	if (usb_dev == NULL) {
		dev_err(data->dev, "%s ERROR no usb_dev", __func__);
		return -ENODEV;
	}

	usb_node = usb_dev->of_node;

	dev_info(data->dev, "%s for %s", __func__, dev_name(usb_dev));
	mutex_lock(&data->lock);

	if (!__usb_port_power_on_off(data->dev, &data->port0, on, usb_node))
		__usb_port_enable(data, &data->port0, usb_node, false);
	if (!__usb_port_power_on_off(data->dev, &data->port1, on, usb_node))
		__usb_port_enable(data, &data->port1, usb_node, false);
	if (!__usb_port_power_on_off(data->dev, &data->port2, on, usb_node))
		__usb_port_enable(data, &data->port2, usb_node, false);
	if (!__usb_port_power_on_off(data->dev, &data->port3, on, usb_node))
		__usb_port_enable(data, &data->port3, usb_node, false);

	mutex_unlock(&data->lock);
	return 0;
}
EXPORT_SYMBOL(rtk_usb_remove_port_power_on_off);

static void __usb_gpio_init(struct device *dev, struct gpio_data *gpio)
{
	bool off = false;

	gpio->active_port = 0;
	__gpio_on_off(dev, -1, gpio, off);

}
static int rtk_usb_gpio_init(struct manager_data *data)
{
	__usb_gpio_init(data->dev, &data->gpio0);
	__usb_gpio_init(data->dev, &data->gpio1);
	__usb_gpio_init(data->dev, &data->gpio2);
	__usb_gpio_init(data->dev, &data->gpio3);

	return 0;
}

static int rtk_usb_manager_init(struct manager_data *data)
{
	struct device *dev = data->dev;

	dev_dbg(dev, "Realtek USB init ....\n");

	__usb_set_pd_power(data, 1);

	if (data->disable_usb) {
		dev_err(dev, "Realtek USB No any usb be enabled ....\n");
		return 0;
	}

	__usb_init_clock_reset(data);

	rtk_usb_gpio_init(data);

	__rtk_usb_set_hw_pm_enable(data);

	dev_dbg(dev, "Realtek USB init Done\n");

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static void __debug_dump_gpio_info(struct seq_file *s, struct gpio_data *gpio)
{
	if (!gpio)
		return;

	seq_puts(s, "    gpio info:\n");
	if (IS_ERR(gpio->gpio_desc)) {
		seq_puts(s, "        No gpio\n");
		return;
	}
	seq_printf(s, "       gpio_num=%d\n", desc_to_gpio(gpio->gpio_desc));
	seq_printf(s, "       active_port=0x%x active_port_mask=0x%x\n",
		    gpio->active_port, gpio->active_port_mask);
	seq_printf(s, "       power %s%s\n",
		    gpio->power_on?"on":"off",
		    gpio->power_low_active?" (power_low_active)":"");
}

static void __debug_dump_port_info(struct seq_file *s, struct port_data *port)
{
	if (port->port_num < 0)
		return;

	seq_printf(s, "port%d info:\n", port->port_num);
	seq_printf(s, "    port_num=%d\n", port->port_num);
	seq_printf(s, "    power_gpio@%p\n", port->power_gpio);
	__debug_dump_gpio_info(s, port->power_gpio);
	seq_printf(s, "    mac_node=%s\n",
		    port->mac_node?port->mac_node->name:"NULL");
	seq_printf(s, "    slave_mac_node=%s\n",
		    port->slave_mac_node?port->slave_mac_node->name:"NULL");
	seq_printf(s, "    active=0x%x (enable_mask=0x%x)\n",
		    port->active, port->enable_mask);
}

static int rtk_usb_debug_show(struct seq_file *s, void *unused)
{
	struct manager_data		*data = s->private;

	seq_puts(s, "rtk usb manager info:\n");
	__debug_dump_port_info(s, &data->port0);
	__debug_dump_port_info(s, &data->port1);
	__debug_dump_port_info(s, &data->port2);
	__debug_dump_port_info(s, &data->port3);

	return 0;
}

static int rtk_usb_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtk_usb_debug_show, inode->i_private);
}

static const struct file_operations rtk_usb_debug_fops = {
	.open			= rtk_usb_debug_open,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static inline void create_debug_files(struct manager_data *data)
{

	dev_dbg(data->dev, "%s", __func__);

	data->debug_dir = debugfs_create_dir("usb_manager", usb_debug_root);
	if (!data->debug_dir) {
		dev_err(data->dev, "%s Error debug_dir is NULL", __func__);
		return;
	}

	if (!debugfs_create_file("debug", 0444, data->debug_dir, data,
		    &rtk_usb_debug_fops))
		goto file_error;

	return;

file_error:
	debugfs_remove_recursive(data->debug_dir);
}

static inline void remove_debug_files(struct manager_data *data)
{
	debugfs_remove_recursive(data->debug_dir);
}
#endif //CONFIG_DEBUG_FS

static ssize_t port0_power_show(struct device *dev,
	    struct device_attribute *attr, char *buf)
{
	struct manager_data *data = dev_get_drvdata(dev);
	char *ptr = buf;
	int count = PAGE_SIZE;
	int n;

	n = scnprintf(ptr, count, "To Control Port 0\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count, "echo \"on\" > port0_power\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count, "echo \"off\" > port0_power\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count, "\n");
	ptr += n;
	count -= n;

	if (data->port0.power_gpio) {
		struct gpio_data *gpio = data->port0.power_gpio;

		n = scnprintf(ptr, count, "Now port0 %s\n",
			    gpio->power_on?"on":"off");
		ptr += n;
		count -= n;
	} else {
		n = scnprintf(ptr, count, "port0 No control gpio\n");
		ptr += n;
		count -= n;
	}

	return ptr - buf;
}

static ssize_t port0_power_store(struct device *dev,
	    struct device_attribute *attr,
	    const char *buf, size_t count)
{
	struct manager_data *data = dev_get_drvdata(dev);
	struct device_node *usb_node;

	mutex_lock(&data->lock);
	if (!strncmp(buf, "on", 2)) {
		usb_node = data->port0.mac_node;
		__usb_port_power_on_off(data->dev, &data->port0, true,
			    usb_node);
		usb_node = data->port0.slave_mac_node;
		__usb_port_power_on_off(data->dev, &data->port0, true,
			    usb_node);
	} else if (!strncmp(buf, "off", 3)) {
		usb_node = data->port0.mac_node;
		__usb_port_power_on_off(data->dev, &data->port0, false,
			    usb_node);
		usb_node = data->port0.slave_mac_node;
		__usb_port_power_on_off(data->dev, &data->port0, false,
			    usb_node);
	} else
		dev_err(data->dev, "UNKNOWN input (%s)", buf);

	mutex_unlock(&data->lock);
	return count;
}
static DEVICE_ATTR_RW(port0_power);

static ssize_t port1_power_show(struct device *dev,
	    struct device_attribute *attr, char *buf)
{
	struct manager_data *data = dev_get_drvdata(dev);
	char *ptr = buf;
	int count = PAGE_SIZE;
	int n;

	n = scnprintf(ptr, count, "To Control Port 1\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count, "echo \"on\" > port1_power\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count, "echo \"off\" > port1_power\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count, "\n");
	ptr += n;
	count -= n;

	if (data->port1.power_gpio) {
		struct gpio_data *gpio = data->port1.power_gpio;

		n = scnprintf(ptr, count, "Now port1 %s\n",
			    gpio->power_on?"on":"off");
		ptr += n;
		count -= n;
	} else {
		n = scnprintf(ptr, count, "port1 No control gpio\n");
		ptr += n;
		count -= n;
	}

	return ptr - buf;
}

static ssize_t port1_power_store(struct device *dev,
	    struct device_attribute *attr,
	    const char *buf, size_t count)
{
	struct manager_data *data = dev_get_drvdata(dev);
	struct device_node *usb_node;

	mutex_lock(&data->lock);
	if (!strncmp(buf, "on", 2)) {
		usb_node = data->port1.mac_node;
		__usb_port_power_on_off(data->dev, &data->port1, true,
			    usb_node);
		usb_node = data->port1.slave_mac_node;
		__usb_port_power_on_off(data->dev, &data->port1, true,
			    usb_node);
	} else if (!strncmp(buf, "off", 3)) {
		usb_node = data->port1.mac_node;
		__usb_port_power_on_off(data->dev, &data->port1, false,
			    usb_node);
		usb_node = data->port1.slave_mac_node;
		__usb_port_power_on_off(data->dev, &data->port1, false,
			    usb_node);
	} else
		dev_err(data->dev, "UNKNOWN input (%s)", buf);

	mutex_unlock(&data->lock);
	return count;
}
static DEVICE_ATTR_RW(port1_power);

static ssize_t port2_power_show(struct device *dev,
	    struct device_attribute *attr, char *buf)
{
	struct manager_data *data = dev_get_drvdata(dev);
	char *ptr = buf;
	int count = PAGE_SIZE;
	int n;

	n = scnprintf(ptr, count, "To Control Port 2\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count, "echo \"on\" > port2_power\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count, "echo \"off\" > port2_power\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count, "\n");
	ptr += n;
	count -= n;

	if (data->port2.power_gpio) {
		struct gpio_data *gpio = data->port2.power_gpio;

		n = scnprintf(ptr, count, "Now port2 %s\n",
			    gpio->power_on?"on":"off");
		ptr += n;
		count -= n;
	} else {
		n = scnprintf(ptr, count, "port2 No control gpio\n");
		ptr += n;
		count -= n;
	}

	return ptr - buf;
}

static ssize_t port2_power_store(struct device *dev,
	    struct device_attribute *attr,
	    const char *buf, size_t count)
{
	struct manager_data *data = dev_get_drvdata(dev);
	struct device_node *usb_node;

	mutex_lock(&data->lock);
	if (!strncmp(buf, "on", 2)) {
		usb_node = data->port2.mac_node;
		__usb_port_power_on_off(data->dev, &data->port2, true,
			    usb_node);
		usb_node = data->port2.slave_mac_node;
		__usb_port_power_on_off(data->dev, &data->port2, true,
			    usb_node);

	} else if (!strncmp(buf, "off", 3)) {
		usb_node = data->port2.mac_node;
		__usb_port_power_on_off(data->dev, &data->port2, false,
			    usb_node);
		usb_node = data->port2.slave_mac_node;
		__usb_port_power_on_off(data->dev, &data->port2, false,
			    usb_node);
	} else
		dev_err(data->dev, "UNKNOWN input (%s)", buf);

	mutex_unlock(&data->lock);
	return count;
}
static DEVICE_ATTR_RW(port2_power);

static ssize_t port3_power_show(struct device *dev,
	    struct device_attribute *attr, char *buf)
{
	struct manager_data *data = dev_get_drvdata(dev);
	char *ptr = buf;
	int count = PAGE_SIZE;
	int n;

	n = scnprintf(ptr, count, "To Control Port 3\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count, "echo \"on\" > port3_power\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count, "echo \"off\" > port3_power\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count, "\n");
	ptr += n;
	count -= n;

	if (data->port3.power_gpio) {
		struct gpio_data *gpio = data->port3.power_gpio;

		n = scnprintf(ptr, count, "Now port3 %s\n",
			    gpio->power_on?"on":"off");
		ptr += n;
		count -= n;
	} else {
		n = scnprintf(ptr, count, "port3 No control gpio\n");
		ptr += n;
		count -= n;
	}

	return ptr - buf;
}

static ssize_t port3_power_store(struct device *dev,
	    struct device_attribute *attr,
	    const char *buf, size_t count)
{
	struct manager_data *data = dev_get_drvdata(dev);
	struct device_node *usb_node;

	mutex_lock(&data->lock);
	if (!strncmp(buf, "on", 2)) {
		usb_node = data->port3.mac_node;
		__usb_port_power_on_off(data->dev, &data->port3, true,
			    usb_node);
		usb_node = data->port3.slave_mac_node;
		__usb_port_power_on_off(data->dev, &data->port3, true,
			    usb_node);
	} else if (!strncmp(buf, "off", 3)) {
		usb_node = data->port3.mac_node;
		__usb_port_power_on_off(data->dev, &data->port3, false,
			    usb_node);
		usb_node = data->port3.slave_mac_node;
		__usb_port_power_on_off(data->dev, &data->port3, false,
			    usb_node);
	} else
		dev_err(data->dev, "UNKNOWN input (%s)", buf);

	mutex_unlock(&data->lock);
	return count;
}
static DEVICE_ATTR_RW(port3_power);

static ssize_t iso_mode_show(struct device *dev,
	    struct device_attribute *attr, char *buf)
{
	struct manager_data *data = dev_get_drvdata(dev);
	char *ptr = buf;
	int count = PAGE_SIZE;
	int n;

	mutex_lock(&data->lock);

	n = scnprintf(ptr, count, "usb_iso_mode %s\n",
		    data->usb_iso_mode?"Enable":"Disable");
	ptr += n;
	count -= n;

	mutex_unlock(&data->lock);

	return ptr - buf;
}

static ssize_t iso_mode_store(struct device *dev,
	    struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct manager_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->lock);
	if (!strncmp(buf, "enable", 6))
		data->usb_iso_mode = true;
	else if (!strncmp(buf, "disable", 7))
		data->usb_iso_mode = false;
	else
		dev_err(data->dev, "UNKNOWN input (%s)", buf);

	mutex_unlock(&data->lock);
	return count;
}
DEVICE_ATTR_RW(iso_mode);

static int __usb_bind_driver(struct manager_data *data,
	    struct port_data *port)
{
	struct device_node *usb_node;

	dev_info(data->dev, "%s: To bind usb port%d\n",
		    __func__, port->port_num);

	usb_node = port->mac_node;
	if (usb_node != NULL) {
		struct platform_device *pdev = NULL;
		struct device *dev = NULL;

		pdev = of_find_device_by_node(usb_node);
		if (pdev != NULL)
			dev = &pdev->dev;

		if (dev && !dev->driver) {
			int ret = 0;

			if (dev->parent && dev->bus->need_parent_lock)
				device_lock(dev->parent);
			ret = device_attach(dev);
			if (dev->parent && dev->bus->need_parent_lock)
				device_unlock(dev->parent);
			if (ret < 0)
				dev_err(data->dev,
					    "%s Error: device_attach fail (ret=%d)",
					    __func__, ret);
			else
				dev_info(data->dev,
					    "%s device %s attach OK\n",
					    __func__, dev_name(dev));
			put_device(dev);
		}
	}
	usb_node = port->slave_mac_node;
	if (usb_node != NULL) {
		struct platform_device *pdev = NULL;
		struct device *dev = NULL;

		pdev = of_find_device_by_node(usb_node);
		if (pdev != NULL)
			dev = &pdev->dev;

		if (dev && !dev->driver) {
			int ret = 0;

			if (dev->parent && dev->bus->need_parent_lock)
				device_lock(dev->parent);
			ret = device_attach(dev);
			if (dev->parent && dev->bus->need_parent_lock)
				device_unlock(dev->parent);
			if (ret < 0)
				dev_err(data->dev,
					    "%s Error: device_attach fail (ret=%d)",
					    __func__, ret);
			else
				dev_info(data->dev,
					    "%s device %s attach OK\n",
					    __func__, dev_name(dev));
			put_device(dev);
		}
	}
	return 0;
}

static int __usb_unbind_driver(struct manager_data *data,
	    struct port_data *port)
{
	struct device_node *usb_node;

	dev_info(data->dev, "%s: To unbind usb port%d\n",
		    __func__, port->port_num);

	usb_node = port->mac_node;
	if (usb_node != NULL) {
		struct platform_device *pdev = NULL;
		struct device *dev = NULL;

		pdev = of_find_device_by_node(usb_node);
		if (pdev != NULL)
			dev = &pdev->dev;

		if (dev && dev->driver) {
			if (dev->parent && dev->bus->need_parent_lock)
				device_lock(dev->parent);
			device_release_driver(dev);
			if (dev->parent && dev->bus->need_parent_lock)
				device_unlock(dev->parent);
			put_device(dev);
		}
	}
	usb_node = port->slave_mac_node;
	if (usb_node != NULL) {
		struct platform_device *pdev = NULL;
		struct device *dev = NULL;

		pdev = of_find_device_by_node(usb_node);
		if (pdev != NULL)
			dev = &pdev->dev;

		if (dev && dev->driver) {
			if (dev->parent && dev->bus->need_parent_lock)
				device_lock(dev->parent);
			device_release_driver(dev);
			if (dev->parent && dev->bus->need_parent_lock)
				device_unlock(dev->parent);
			put_device(dev);
		}
	}
	return 0;
}

static ssize_t port0_show(struct device *dev,
	      struct device_attribute *attr, char *buf)
{
	struct manager_data *data = dev_get_drvdata(dev);
	char *ptr = buf;
	int count = PAGE_SIZE;
	int n;

	n = scnprintf(ptr, count,
		     "Now Port0 is %s\n",
		    data->port0.enable?"enable":"disable");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count,
		     "echo disable > port0 ==> to disable port0\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count,
		     "echo enable > port0 ==> to enable port0\n");
	ptr += n;
	count -= n;

	return ptr - buf;
}

static ssize_t port0_store(struct device *dev,
	       struct device_attribute *attr, const char *buf, size_t count)
{
	struct manager_data *data = dev_get_drvdata(dev);
	struct port_data *port = &data->port0;

	if (!strncmp(buf, "disable", 7)) {
		if ((port->enable & port->enable_mask) == port->enable_mask) {
			__usb_unbind_driver(data, port);
			__usb_port_clock_reset_disable(data, port);
			dev_info(data->dev, "port0 disable OK\n");
		} else {
			dev_info(data->dev, "port0 is disable\n");
		}
	} else if (!strncmp(buf, "enable", 6)) {
		if (!port->enable && port->enable_mask) {
			__usb_port_clock_reset_enable(data, port);
			__usb_bind_driver(data, port);
			dev_info(data->dev, "port0 enable OK\n");
		} else {
			dev_info(data->dev, "port0 is enable\n");
		}
	}

	return count;
}
static DEVICE_ATTR_RW(port0);

static ssize_t port1_show(struct device *dev,
	      struct device_attribute *attr, char *buf)
{
	struct manager_data *data = dev_get_drvdata(dev);
	char *ptr = buf;
	int count = PAGE_SIZE;
	int n;

	n = scnprintf(ptr, count,
		     "Now Port1 is %s\n",
		    data->port1.enable?"enable":"disable");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count,
		     "echo disable > port1 ==> to disable port1\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count,
		     "echo enable > port1 ==> to enable port1\n");
	ptr += n;
	count -= n;

	return ptr - buf;
}

static ssize_t port1_store(struct device *dev,
	       struct device_attribute *attr, const char *buf, size_t count)
{
	struct manager_data *data = dev_get_drvdata(dev);
	struct port_data *port = &data->port1;

	if (!strncmp(buf, "disable", 7)) {
		if ((port->enable & port->enable_mask) == port->enable_mask) {
			__usb_unbind_driver(data, port);
			__usb_port_clock_reset_disable(data, port);
			dev_info(data->dev, "port1 disable OK\n");
		} else {
			dev_info(data->dev, "port1 is disable\n");
		}
	} else if (!strncmp(buf, "enable", 6)) {
		if (!port->enable && port->enable_mask) {
			__usb_port_clock_reset_enable(data, port);
			__usb_bind_driver(data, port);
			dev_info(data->dev, "port1 enable OK\n");
		} else {
			dev_info(data->dev, "port1 is enable\n");
		}
	}

	return count;
}
static DEVICE_ATTR_RW(port1);

static ssize_t port2_show(struct device *dev,
	      struct device_attribute *attr, char *buf)
{
	struct manager_data *data = dev_get_drvdata(dev);
	char *ptr = buf;
	int count = PAGE_SIZE;
	int n;

	n = scnprintf(ptr, count,
		     "Now Port2 is %s\n",
		    data->port2.enable?"enable":"disable");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count,
		     "echo disable > port2 ==> to disable port2\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count,
		     "echo enable > port2 ==> to enable port2\n");
	ptr += n;
	count -= n;

	return ptr - buf;
}

static ssize_t port2_store(struct device *dev,
	       struct device_attribute *attr, const char *buf, size_t count)
{
	struct manager_data *data = dev_get_drvdata(dev);
	struct port_data *port = &data->port2;

	if (!strncmp(buf, "disable", 7)) {
		if ((port->enable & port->enable_mask) == port->enable_mask) {
			__usb_unbind_driver(data, port);
			__usb_port_clock_reset_disable(data, port);
			dev_info(data->dev, "port2 disable OK\n");
		} else {
			dev_info(data->dev, "port2 is disable\n");
		}
	} else if (!strncmp(buf, "enable", 6)) {
		if (!port->enable && port->enable_mask) {
			__usb_port_clock_reset_enable(data, port);
			__usb_bind_driver(data, port);
			dev_info(data->dev, "port2 enable OK\n");
		} else {
			dev_info(data->dev, "port2 is enable\n");
		}
	}

	return count;
}
static DEVICE_ATTR_RW(port2);

static ssize_t port3_show(struct device *dev,
	      struct device_attribute *attr, char *buf)
{
	struct manager_data *data = dev_get_drvdata(dev);
	char *ptr = buf;
	int count = PAGE_SIZE;
	int n;

	n = scnprintf(ptr, count,
		     "Now Port3 is %s\n",
		    data->port3.enable?"enable":"disable");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count,
		     "echo disable > port3 ==> to disable port3\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count,
		     "echo enable > port3 ==> to enable port3\n");
	ptr += n;
	count -= n;

	return ptr - buf;
}

static ssize_t port3_store(struct device *dev,
	       struct device_attribute *attr, const char *buf, size_t count)
{
	struct manager_data *data = dev_get_drvdata(dev);
	struct port_data *port = &data->port3;

	if (!strncmp(buf, "disable", 7)) {
		if ((port->enable & port->enable_mask) == port->enable_mask) {
			__usb_unbind_driver(data, port);
			__usb_port_clock_reset_disable(data, port);
			dev_info(data->dev, "port3 disable OK\n");
		} else {
			dev_info(data->dev, "port3 is disable\n");
		}
	} else if (!strncmp(buf, "enable", 6)) {
		if (!port->enable && port->enable_mask) {
			__usb_port_clock_reset_enable(data, port);
			__usb_bind_driver(data, port);
			dev_info(data->dev, "port3 enable OK\n");
		} else {
			dev_info(data->dev, "port3 is enable\n");
		}
	}

	return count;
}
static DEVICE_ATTR_RW(port3);

static inline void create_device_files(struct manager_data *data)
{
	struct device *dev = data->dev;

	dev_dbg(data->dev, "%s", __func__);

	if (data->port0.port_num >= 0) {
		device_create_file(dev, &dev_attr_port0_power);
		device_create_file(dev, &dev_attr_port0);
	}

	if (data->port1.port_num >= 0) {
		device_create_file(dev, &dev_attr_port1_power);
		device_create_file(dev, &dev_attr_port1);
	}

	if (data->port2.port_num >= 0) {
		device_create_file(dev, &dev_attr_port2_power);
		device_create_file(dev, &dev_attr_port2);
	}

	if (data->port3.port_num >= 0) {
		device_create_file(dev, &dev_attr_port3_power);
		device_create_file(dev, &dev_attr_port3);
	}

	device_create_file(dev, &dev_attr_iso_mode);
}

static inline void remove_device_files(struct manager_data *data)
{
	struct device *dev = data->dev;

	dev_dbg(data->dev, "%s", __func__);

	if (data->port0.port_num >= 0) {
		device_remove_file(dev, &dev_attr_port0_power);
		device_remove_file(dev, &dev_attr_port0);
	}

	if (data->port1.port_num >= 0) {
		device_remove_file(dev, &dev_attr_port1_power);
		device_remove_file(dev, &dev_attr_port1);
	}

	if (data->port2.port_num >= 0) {
		device_remove_file(dev, &dev_attr_port2_power);
		device_remove_file(dev, &dev_attr_port2);
	}

	if (data->port3.port_num >= 0) {
		device_remove_file(dev, &dev_attr_port3_power);
		device_remove_file(dev, &dev_attr_port3);
	}

	device_remove_file(dev, &dev_attr_iso_mode);
}

static inline int __of_parse_gpio_setting(struct manager_data *data,
	    struct gpio_data *gpio,
	    struct device_node *sub_node)
{
	if (!sub_node) {
		gpio->gpio_desc = ERR_PTR(-ENOSYS);
		return -1;
	}

	gpio->node = sub_node;

	gpio->gpio_desc = gpiod_get_from_of_node(sub_node, "realtek,power-gpio",
							0, GPIOD_OUT_HIGH, "usb-power-gpio");

	if (IS_ERR(gpio->gpio_desc)) {
		dev_info(data->dev, "%s power-gpio no found (err=%d)\n",
			     __func__, (int)PTR_ERR(gpio->gpio_desc));
		return (int)PTR_ERR(gpio->gpio_desc);
	}

	if (of_property_read_bool(sub_node, "power_low_active"))
		gpio->power_low_active = true;
	else
		gpio->power_low_active = false;

	return 0;
}

static int parse_gpio_setting(struct manager_data *data,
	    struct device_node *node)
{
	struct device *dev = data->dev;
	struct device_node	*sub_node;
	int ret = 0;

	sub_node = of_get_child_by_name(node, "gpio0");
	ret = __of_parse_gpio_setting(data, &data->gpio0, sub_node);
	if (ret) {
		if (ret == -EPROBE_DEFER) {
			dev_err(dev, "%s probe DEFER for no gpio\n", __func__);
			return ret;
		}
		dev_dbg(dev, "node gpio0 no found in dts\n");
	} else {
		dev_info(dev, "%s get power-gpio (id=%d) OK\n",
			    __func__, desc_to_gpio(data->gpio0.gpio_desc));
	}

	sub_node = of_get_child_by_name(node, "gpio1");
	ret = __of_parse_gpio_setting(data, &data->gpio1, sub_node);
	if (ret) {
		if (ret == -EPROBE_DEFER) {
			dev_err(dev, "%s probe DEFER for no gpio\n", __func__);
			return ret;
		}
		dev_dbg(dev, "node gpio1 no found in dts\n");
	} else {
		dev_info(dev, "%s get power-gpio (id=%d) OK\n",
			    __func__, desc_to_gpio(data->gpio1.gpio_desc));
	}

	sub_node = of_get_child_by_name(node, "gpio2");
	ret = __of_parse_gpio_setting(data, &data->gpio2, sub_node);
	if (ret) {
		if (ret == -EPROBE_DEFER) {
			dev_err(dev, "%s probe DEFER for no gpio\n", __func__);
			return ret;
		}
		dev_dbg(dev, "node gpio2 no found in dts\n");
	} else {
		dev_info(dev, "%s get power-gpio (id=%d) OK\n",
			    __func__, desc_to_gpio(data->gpio2.gpio_desc));
	}

	sub_node = of_get_child_by_name(node, "gpio3");
	ret = __of_parse_gpio_setting(data, &data->gpio3, sub_node);
	if (ret) {
		if (ret == -EPROBE_DEFER) {
			dev_err(dev, "%s probe DEFER for no gpio\n", __func__);
			return ret;
		}
		dev_dbg(dev, "node gpio3 no found in dts\n");
	} else {
		dev_info(dev, "%s get power-gpio (id=%d) OK\n",
			    __func__, desc_to_gpio(data->gpio3.gpio_desc));
	}

	return 0;
}

static inline struct gpio_data *__of_parse_port_gpio_mapping(
	    struct manager_data *data,
	    struct device_node *port_node, int port_num)
{
	struct device_node *gpio_node;
	struct gpio_data *gpio;

	gpio_node = of_parse_phandle(port_node, "usb_gpio", 0);

	gpio = &data->gpio0;
	if (!IS_ERR(gpio->gpio_desc) &&
		    gpio->node &&  gpio_node &&
		    gpio->node->phandle == gpio_node->phandle) {
		gpio->active_port_mask |= BIT(port_num);
		dev_info(data->dev, "%s port%d mapping gpio_num=%d\n",
			    __func__, port_num, desc_to_gpio(gpio->gpio_desc));
		return gpio;
	}

	gpio = &data->gpio1;
	if (!IS_ERR(gpio->gpio_desc) &&
		    gpio->node &&  gpio_node &&
		    gpio->node->phandle == gpio_node->phandle) {
		gpio->active_port_mask |= BIT(port_num);
		dev_info(data->dev, "%s port%d mapping gpio_num=%d\n",
			    __func__, port_num, desc_to_gpio(gpio->gpio_desc));
		return gpio;
	}

	gpio = &data->gpio2;
	if (!IS_ERR(gpio->gpio_desc) &&
		    gpio->node &&  gpio_node &&
		    gpio->node->phandle == gpio_node->phandle) {
		gpio->active_port_mask |= BIT(port_num);
		dev_info(data->dev, "%s port%d mapping gpio_num=%d\n",
			    __func__, port_num, desc_to_gpio(gpio->gpio_desc));
		return gpio;
	}

	gpio = &data->gpio3;
	if (!IS_ERR(gpio->gpio_desc) &&
		    gpio->node &&  gpio_node &&
		    gpio->node->phandle == gpio_node->phandle) {
		gpio->active_port_mask |= BIT(port_num);
		dev_info(data->dev, "%s port%d mapping gpio_num=%d\n",
			    __func__, port_num, desc_to_gpio(gpio->gpio_desc));
		return gpio;
	}

	dev_info(data->dev, "%s port%d No mapping GPIO\n", __func__, port_num);
	return NULL;
}

static inline void __of_parse_port_setting(struct manager_data *data,
	    struct port_data *port,
	    struct device_node *port_node, int port_num)
{
	if (!port_node) {
		port->port_num = -1;
		return;
	}

	port->port_num = port_num;

	port->mac_node = of_parse_phandle(port_node, "usb", 0);
	port->slave_mac_node = of_parse_phandle(port_node, "usb", 1);

	port->active = 0;
	port->enable_mask = 0;
	port->enable = 0;

	if (port->mac_node &&
		    of_device_is_available(port->mac_node)) {
		pr_info("mac_node: %s status is okay\n",
			    port->mac_node->name);
		port->enable_mask |= BIT(0);
	}
	if (port->slave_mac_node &&
		    of_device_is_available(port->slave_mac_node)) {
		pr_info("slave_mac_node: %s status is okay\n",
			    port->slave_mac_node->name);
		port->enable_mask |= BIT(1);
	}

	if (port->enable_mask)
		port->power_gpio = __of_parse_port_gpio_mapping(data,
			    port_node, port->port_num);

	port->node = port_node;

	port->support_usb3 = true;
	if (of_property_read_bool(port_node, "disable_usb3"))
		port->support_usb3 = false;
}

static int parse_port_setting(struct manager_data *data,
	    struct device_node *node)
{
	struct device_node	*sub_node;

	data->disable_usb = true;

	sub_node = of_get_child_by_name(node, "port0");
	__of_parse_port_setting(data, &data->port0, sub_node, 0);
	sub_node = of_get_child_by_name(node, "port1");
	__of_parse_port_setting(data, &data->port1, sub_node, 1);
	sub_node = of_get_child_by_name(node, "port2");
	__of_parse_port_setting(data, &data->port2, sub_node, 2);
	sub_node = of_get_child_by_name(node, "port3");
	__of_parse_port_setting(data, &data->port3, sub_node, 3);

	if (port_can_enable(&data->port0) ||
		    port_can_enable(&data->port1) ||
		    port_can_enable(&data->port2) ||
		    port_can_enable(&data->port3))
		data->disable_usb = false;

	return 0;
}

static int rtk_usb_platform_init(struct manager_data *data)
{
	int (*rtk_usb_init)(struct rtk_usb_ops *ops) = NULL;
	const struct soc_device_attribute *soc_att_match = NULL;
	struct soc_device_attribute rtk_soc[] = {
		{
			.family = "Realtek Phoenix",
			.data = (void *)rtk_usb_rtd119x_init,
		},
		{
			.family = "Realtek Kylin",
			.data = (void *)rtk_usb_rtd129x_init,
		},
		{
			.family = "Realtek Hercules",
			.data = (void *)rtk_usb_rtd139x_init,
		},
		{
			.family = "Realtek Thor",
			.data = (void *)rtk_usb_rtd16xx_init,
		},
		{
			.family = "Realtek Hank",
			.data = (void *)rtk_usb_rtd13xx_init,
		},
		{
			.family = "Realtek Stark",
			.data = (void *)rtk_usb_rtd16xxb_init,
		},
		{
			.family = "Realtek Groot",
			.data = (void *)rtk_usb_rtd1312c_init,
		},
		{
			.family = "Realtek Parker",
			.data = (void *)rtk_usb_rtd13xxd_init,
		},
		{
		/* empty */
		}
	};

	if (!data) {
		pr_err("%s ERROR: data is NULL\n", __func__);
		return -ENODEV;
	}
	data->rtk_usb = kzalloc(sizeof(struct rtk_usb_ops), GFP_KERNEL);
	if (!data->rtk_usb)
		return -ENOMEM;

	soc_att_match = soc_device_match(rtk_soc);
	if (soc_att_match) {
		pr_info("%s match chip: %s\n", __func__,
			    soc_att_match->family);
		rtk_usb_init = (void *)soc_att_match->data;
		if (rtk_usb_init)
			rtk_usb_init(data->rtk_usb);
		else
			pr_err("%s ERROR: rtk_usb_init is NULL\n", __func__);
	} else {
		pr_err("%s ERROR: no match chip\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static void rtk_usb_platform_free(struct manager_data *data)
{
	kfree(data->rtk_usb);
	data->rtk_usb = NULL;
}

static void rtk_usb_set_clock_reset_name(struct manager_data *data)
{
	struct port_data *port;

	/* For All port */
	snprintf(data->clock, CLOCK_RESET_NAME_MAX, "%s", CLOCK_NAME_USB);
	snprintf(data->reset_usb, CLOCK_RESET_NAME_MAX, "%s", RESET_NAME_USB);
	snprintf(data->reset_type_c, CLOCK_RESET_NAME_MAX, "%s",
		    RESET_NAME_TYPEC);
	/* For Port 0 */
	port = &data->port0;
	snprintf(port->clock, CLOCK_RESET_NAME_MAX, "%s", CLOCK_NAME_USB_PORT0);
	snprintf(port->reset_u2phy, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_U2PHY0);
	snprintf(port->reset_u3phy, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_U3PHY0);
	snprintf(port->reset_u3mdio, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_U3MDIO0);
	snprintf(port->reset_usb_mac, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_USB_MAC0);
	/* For Port 1 */
	port = &data->port1;
	snprintf(port->clock, CLOCK_RESET_NAME_MAX, "%s", CLOCK_NAME_USB_PORT1);
	snprintf(port->reset_u2phy, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_U2PHY1);
	snprintf(port->reset_u3phy, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_U3PHY1);
	snprintf(port->reset_u3mdio, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_U3MDIO1);
	snprintf(port->reset_usb_mac, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_USB_MAC1);
	/* For Port 2 */
	port = &data->port2;
	snprintf(port->clock, CLOCK_RESET_NAME_MAX, "%s", CLOCK_NAME_USB_PORT2);
	snprintf(port->reset_u2phy, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_U2PHY2);
	snprintf(port->reset_u3phy, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_U3PHY2);
	snprintf(port->reset_u3mdio, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_U3MDIO2);
	snprintf(port->reset_usb_mac, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_USB_MAC2);
	/* For Port 3 */
	port = &data->port3;
	snprintf(port->clock, CLOCK_RESET_NAME_MAX, "%s", CLOCK_NAME_USB_PORT3);
	snprintf(port->reset_u2phy, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_U2PHY3);
	snprintf(port->reset_u3phy, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_U3PHY3);
	snprintf(port->reset_u3mdio, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_U3MDIO3);
	snprintf(port->reset_usb_mac, CLOCK_RESET_NAME_MAX,
		    "%s", RESET_NAME_USB_MAC3);
}

static int rtk_usb_manager_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct device_node	*node = dev->of_node;
	struct device_node	*sub_node;
	struct manager_data	*data;
	int ret = 0;
	unsigned long probe_time = jiffies;

	dev_info(dev, "ENTER %s", __func__);

	if (!node || !of_device_is_available(node)) {
		dev_err(dev, "%s ERROR: No device_node!!", __func__);
		return -ENODEV;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;

	mutex_init(&data->lock);

	/* platform ops init */
	ret = rtk_usb_platform_init(data);
	if (ret)
		goto err0;

	/* Parse device tree */
	ret = parse_gpio_setting(data, node);
	if (ret)
		goto err1;

	parse_port_setting(data, node);

	if (of_property_read_bool(node, "usb_iso_mode"))
		data->usb_iso_mode = true;
	else
		data->usb_iso_mode = false;

	if (of_property_read_bool(node, "en_usb_storage_reprobe"))
		data->en_usb_storage_reprobe = true;
	else
		data->en_usb_storage_reprobe = false;

	if (of_property_read_bool(node, "rescue_usb"))
		data->rescue_usb = true;
	else
		data->rescue_usb = false;

	if (of_property_read_bool(node, "dis_hub_autosuspend"))
		data->dis_hub_autosuspend = true;
	else
		data->dis_hub_autosuspend = false;

	rtk_usb_set_clock_reset_name(data);

	sub_node = of_get_child_by_name(node, "rtk_usb");
	if (sub_node)
		rtk_usb_soc_init(data->rtk_usb, sub_node);

	rtk_usb_manager_init(data);

	data->wq_usb_manager = create_singlethread_workqueue("rtk_usb_manager");

#ifdef CONFIG_DEBUG_FS
	create_debug_files(data);
#endif

	create_device_files(data);

	platform_set_drvdata(pdev, data);

	dev_info(&pdev->dev, "%s populate subnode device\n", __func__);
	ret = of_platform_populate(node, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "%s failed to add subnode device\n", __func__);
		goto err1;
	}

	dev_info(&pdev->dev, "%s OK (take %d ms)\n", __func__,
		    jiffies_to_msecs(jiffies - probe_time));
	return 0;

err1:
	rtk_usb_platform_free(data);
err0:
	devm_kfree(dev, data);

	dev_info(&pdev->dev, "%s Fail (ret=%d) (take %d ms)\n", __func__,
		    ret, jiffies_to_msecs(jiffies - probe_time));

	return ret;
}

static int rtk_usb_manager_remove(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct manager_data *data = dev_get_drvdata(dev);

	dev_info(&pdev->dev, "%s\n", __func__);

	remove_device_files(data);

#ifdef CONFIG_DEBUG_FS
	remove_debug_files(data);
#endif

	rtk_usb_soc_free(data->rtk_usb);

	rtk_usb_platform_free(data);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rtk_usb_manager_match[] = {
	{ .compatible = "realtek,usb-manager" },
	{},
};
MODULE_DEVICE_TABLE(of, rtk_usb_manager_match);
#endif

#ifdef CONFIG_PM_SLEEP

static int rtk_usb_manager_prepare(struct device *dev)
{
	struct manager_data *data = dev_get_drvdata(dev);
	int ret = 0;

	dev_info(dev, "[USB] Enter %s\n", __func__);
	if (!data) {
		dev_err(dev, "[USB] %s No manager_data!\n", __func__);
		goto out;
	}

out:
	dev_info(dev, "[USB] Exit %s\n", __func__);
	return ret;
}

static void rtk_usb_manager_complete(struct device *dev)
{
	struct manager_data *data = dev_get_drvdata(dev);

	dev_info(dev, "[USB] Enter %s\n", __func__);

	if (!data) {
		dev_err(dev, "[USB] %s No manager_data!\n", __func__);
		goto out;
	}

out:
	dev_info(dev, "[USB] Exit %s\n", __func__);
}

static int rtk_usb_manager_suspend(struct device *dev)
{
	struct manager_data *data = dev_get_drvdata(dev);

	dev_info(dev, "[USB] Enter %s\n", __func__);

	if (!data) {
		dev_err(dev, "[USB] %s No manager_data!\n", __func__);
		goto out;
	}

	if (!data->usb_iso_mode) {
		__usb_port_gpio_off(data);
		__usb_set_pd_power(data, 0);
		__usb_clear_clock_reset(data);
	} else {
		__usb_port_suspend(data);
	}

out:
	dev_info(dev, "[USB] Exit %s\n", __func__);
	return 0;
}

static int rtk_usb_manager_resume(struct device *dev)
{
	struct manager_data *data = dev_get_drvdata(dev);
	struct clk *clk_usb = USB_clk_get(NULL, "clk_en_usb");

	dev_info(dev, "[USB] Enter %s\n", __func__);

	if (!data) {
		dev_err(dev, "[USB] %s No manager_data!\n", __func__);
		goto out;
	}

	if (!data->usb_iso_mode) {
		__usb_set_pd_power(data, 1);

		clk_disable_unprepare(clk_usb);
		__usb_init_clock_reset(data);

		__rtk_usb_set_hw_pm_enable(data);

	} else {
		__usb_port_resume(data);
	}
	__usb_port_gpio_on(data);

out:
	dev_info(dev, "[USB] Exit %s\n", __func__);
	return 0;
}

static const struct dev_pm_ops rtk_usb_manager_pm_ops = {
	.prepare = rtk_usb_manager_prepare,
	.complete = rtk_usb_manager_complete,
	SET_LATE_SYSTEM_SLEEP_PM_OPS(rtk_usb_manager_suspend,
	    rtk_usb_manager_resume)
};

#define DEV_PM_OPS	(&rtk_usb_manager_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static void rtk_usb_manager_shutdown(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct manager_data *data = dev_get_drvdata(dev);

	dev_info(dev, "[USB] Enter %s S5 (shutdown)\n",
		    __func__);

	rtk_usb_gpio_init(data);
	__usb_set_pd_power(data, 0);
	__usb_clear_clock_reset(data);

	dev_info(dev, "[USB] Exit %s\n", __func__);
}

static struct platform_driver rtk_usb_manager_driver = {
	.probe		= rtk_usb_manager_probe,
	.remove		= rtk_usb_manager_remove,
	.driver		= {
		.name	= "rtk-usb-manager",
		.of_match_table = of_match_ptr(rtk_usb_manager_match),
		.pm = DEV_PM_OPS,
	},
	.shutdown = rtk_usb_manager_shutdown,
};

static int __init rtk_usb_manager_driver_init(void)
{
	return platform_driver_register(&(rtk_usb_manager_driver));
}
//subsys_initcall_sync(rtk_usb_manager_driver_init);
module_init(rtk_usb_manager_driver_init);

static void __exit rtk_usb_manager_driver_exit(void)
{
	platform_driver_unregister(&(rtk_usb_manager_driver));
}
module_exit(rtk_usb_manager_driver_exit);

MODULE_ALIAS("platform:rtk-usb-manager");
MODULE_LICENSE("GPL");
