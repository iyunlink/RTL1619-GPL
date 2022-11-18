// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017 Realtek Semiconductor Corp.
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 *
 */

#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/usb/otg.h>
#include <linux/module.h>

#include "rtk_usb.h"

struct rtk_usb {
	struct power_ctrl_reg {
		u32 usb_ctrl;
		/* Port 0~2 */
		u32 usb0_sram_pwr3;
		u32 usb0_sram_pwr4;
		/* l4_icg */
		u32 p0_l4_icg;
		u32 p1_l4_icg;
	} power_ctrl_reg;

	bool usb_power_cut;

	struct type_c {
		struct gpio_desc *connector_switch_gpio;
	} type_c;
};

static struct rtk_usb *rtk_usb;

/* set usb charger power */
static void usb_set_charger_power(struct rtk_usb *rtk_usb, unsigned int val)
{
	if (!rtk_usb)
		return;
}

static int usb_set_hw_l4icg_on_off(struct rtk_usb *rtk_usb,
	    enum usb_port_num port_num, bool on)
{
	void __iomem *reg;

	if (!rtk_usb)
		return 0;

	switch (port_num) {
	case USB_PORT_0:
		if (rtk_usb->power_ctrl_reg.p0_l4_icg) {
			pr_info("%s set l4_icg port %d\n", __func__, port_num);
			reg = ioremap(rtk_usb->power_ctrl_reg.p0_l4_icg, 0x4);
			writel((on&BIT(0)) | readl(reg), reg);
			iounmap(reg);
		}
		break;
	case USB_PORT_1:
	case USB_PORT_2:
		if (rtk_usb->power_ctrl_reg.p1_l4_icg) {
			pr_info("%s set l4_icg port %d\n", __func__, port_num);
			reg = ioremap(rtk_usb->power_ctrl_reg.p1_l4_icg, 0x4);
			writel((on&BIT(0)) | readl(reg), reg);
			iounmap(reg);
		}
		break;
	default:
		pr_err("%s Error Port num %d\n", __func__, port_num);
		break;
	}

	return 0;
}

static int usb_iso_power_ctrl(struct rtk_usb *rtk_usb,
	    bool power_on)
{
	if (!rtk_usb)
		return 0;

	pr_debug("%s START\n", __func__);

	if (rtk_usb->usb_power_cut &&
		    rtk_usb->power_ctrl_reg.usb_ctrl &&
		    rtk_usb->power_ctrl_reg.usb0_sram_pwr3 &&
		    rtk_usb->power_ctrl_reg.usb0_sram_pwr4) {
		void __iomem *usb0_sram_pwr3 =
			    ioremap(rtk_usb->power_ctrl_reg.usb0_sram_pwr3,
			    0x4);
		void __iomem *usb0_sram_pwr4 =
			    ioremap(rtk_usb->power_ctrl_reg.usb0_sram_pwr4,
			    0x4);
		void __iomem *usb_ctrl = ioremap(
			    rtk_usb->power_ctrl_reg.usb_ctrl, 0x4);

		pr_info("%s power_%s ([0x%x=0x%08x)\n", __func__,
			    power_on?"on":"off",
			    rtk_usb->power_ctrl_reg.usb_ctrl, readl(usb_ctrl));
		if (power_on) {
			writel((BIT(0) | BIT(1) | BIT(4) | BIT(5) | BIT(6)) |
				     readl(usb_ctrl), usb_ctrl);
			writel(0x00000f00, usb0_sram_pwr4);
			writel(~(BIT(8) | BIT(9)) & readl(usb_ctrl), usb_ctrl);
		} else {
			writel((BIT(8) | BIT(9)) | readl(usb_ctrl), usb_ctrl);
			// port0-port2 sram
			writel(0, usb0_sram_pwr3);
			writel(0x00000f01, usb0_sram_pwr4);
			writel(~(BIT(0) | BIT(1) | BIT(4) | BIT(5) | BIT(6)) &
				     readl(usb_ctrl), usb_ctrl);
		}
		pr_info("set power_domain %s ([0x%x]=0x%08x)\n",
			    power_on?"on":"off",
			    rtk_usb->power_ctrl_reg.usb_ctrl, readl(usb_ctrl));
		iounmap(usb_ctrl);
		iounmap(usb0_sram_pwr3);
		iounmap(usb0_sram_pwr4);
	}
	pr_debug("%s END\n", __func__);
	return 0;
}

static int usb_port_suspend_resume(struct rtk_usb *rtk_usb,
	    enum usb_port_num port_num, bool is_suspend)
{
	if (!rtk_usb)
		return 0;

	return 0;
}

static int type_c_init(struct rtk_usb *rtk_usb)
{
	if (!rtk_usb)
		return 0;

	return 0;
}

static int type_c_plug_config(struct rtk_usb *rtk_usb, int dr_mode, int cc)
{
	bool high;

	/* host / device */
	if (dr_mode == USB_DR_MODE_PERIPHERAL)
		high = true;
	else if (dr_mode == USB_DR_MODE_HOST)
		high = false;
	else
		goto out;

	if (!IS_ERR(rtk_usb->type_c.connector_switch_gpio)) {
		pr_info("%s Set connector to %s by gpio %d\n",
			    __func__, high?"device":"host",
			    desc_to_gpio(rtk_usb->type_c.connector_switch_gpio));
		if (gpiod_direction_output(
			    rtk_usb->type_c.connector_switch_gpio, high))
			pr_err("%s ERROR connector_switch_ctrl_gpio fail\n",
				    __func__);
	}

out:
	return 0;
}

static struct rtk_usb *usb_soc_init(struct device_node *node)
{
	struct device_node *sub_node;

	if (!rtk_usb)
		rtk_usb = kzalloc(sizeof(struct rtk_usb), GFP_KERNEL);

	pr_info("%s START (%s)\n", __func__, __FILE__);
	sub_node = of_get_child_by_name(node, "power_ctrl_reg");
	if (sub_node) {
		pr_debug("%s sub_node %s\n", __func__, sub_node->name);
		of_property_read_u32(sub_node,
			    "usb_ctrl", &rtk_usb->power_ctrl_reg.usb_ctrl);
		/* Port 0~2 */
		of_property_read_u32(sub_node,
			    "usb0_sram_pwr3",
			    &rtk_usb->power_ctrl_reg.usb0_sram_pwr3);
		of_property_read_u32(sub_node,
			    "usb0_sram_pwr4",
			    &rtk_usb->power_ctrl_reg.usb0_sram_pwr4);

		of_property_read_u32(sub_node,
			    "p0_l4_icg", &rtk_usb->power_ctrl_reg.p0_l4_icg);
		of_property_read_u32(sub_node,
			    "p1_l4_icg", &rtk_usb->power_ctrl_reg.p1_l4_icg);

		if (of_property_read_bool(sub_node, "usb_power_cut"))
			rtk_usb->usb_power_cut = true;
		else
			rtk_usb->usb_power_cut = false;

	}

	sub_node = of_get_child_by_name(node, "type_c");
	if (sub_node) {

		pr_debug("%s sub_node %s\n", __func__, sub_node->name);

		rtk_usb->type_c.connector_switch_gpio = gpiod_get_from_of_node(sub_node, "realtek,connector_switch-gpio",
										0, GPIOD_OUT_HIGH, "usb-connector_switch");
		if (IS_ERR(rtk_usb->type_c.connector_switch_gpio)) {
			pr_info("connector_switch-gpio no found");
		} else {
			pr_info("%s get connector_switch-gpio (id=%d) OK\n",
				    __func__, desc_to_gpio(rtk_usb->type_c.connector_switch_gpio));
		}

	}

	pr_info("%s END\n", __func__);
	return rtk_usb;
}

static int usb_soc_free(struct rtk_usb **rtk_usb)
{
	if (*rtk_usb) {
		kfree(*rtk_usb);
		*rtk_usb = NULL;
	}
	return 0;
}

int rtk_usb_rtd1312c_init(struct rtk_usb_ops *ops)
{
	if (ops == NULL)
		return -1;

	ops->usb_soc_init = &usb_soc_init;
	ops->usb_soc_free = &usb_soc_free;

	ops->usb_port_suspend_resume = &usb_port_suspend_resume;
	ops->usb_set_hw_l4icg_on_off = &usb_set_hw_l4icg_on_off;

	ops->usb_iso_power_ctrl = &usb_iso_power_ctrl;

	ops->usb_set_charger_power = &usb_set_charger_power;

	/* For Type c */
	ops->type_c_init = &type_c_init;
	ops->type_c_plug_config = &type_c_plug_config;

	return 0;
}
EXPORT_SYMBOL(rtk_usb_rtd1312c_init);

MODULE_LICENSE("GPL");
