// SPDX-License-Identifier: GPL-2.0
/**
 *  * dwc3-rtk-type_c.c - Realtek DWC3 Type C driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/sys_soc.h>
#include <linux/nvmem-consumer.h>
#include <linux/usb/otg.h>
#include <soc/realtek/rtk-usb-manager.h>
#ifdef CONFIG_TYPEC
#include <linux/usb/typec.h>
#endif

#include "dwc3-rtk-drd.h"

#ifdef CONFIG_DUAL_ROLE_USB_INTF
#include <linux/usb/class-dual-role.h>
#endif

#ifdef CONFIG_USB_TYPE_C_RTK_RTS5400
extern bool rtk_rts5400_is_UFP_attached(void);
extern bool rtk_rts5400_is_enabled(void);
extern int rtk_rts5400_set_type_c_soft_reset(void);
#endif

static const struct soc_device_attribute rtk_soc_kylin[] = {
	{
		.family = "Realtek Kylin",
	},
	{
		/* empty */
	}
};

static const struct soc_device_attribute rtk_soc_hercules[] = {
	{
		.family = "Realtek Hercules",
	},
	{
		/* empty */
	}
};

static const struct soc_device_attribute rtk_soc_hank[] = {
	{
		.family = "Realtek Hank",
	},
	{
		/* empty */
	}
};

static const struct soc_device_attribute rtk_soc_typc_parameter_v0[] = {
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
		.family = "Realtek Groot",
	},
	{
		/* empty */
	}
};

struct dwc3_rtk;

struct type_c_data {
	void __iomem *type_c_reg_base;
	struct device *dev;

	u32			irq;

	int chip_revision;

	/* rd control GPIO only for rtd129x*/
	unsigned int rd_ctrl_gpio;

	/* Parameters */
	int para_ver; /* Parameter version */
	u32 cc1_rp;
	u32 cc1_rp_code;
	u32 cc1_rd_code;
	u32 cc1_vref_ufp;
	u32 cc1_vref_dfp_usb;
	u32 cc1_vref_dfp_1_5;
	u32 cc1_vref_dfp_3_0;
	u32 cc2_rp;
	u32 cc2_rp_code;
	u32 cc2_rd_code;
	u32 cc2_vref_ufp;
	u32 cc2_vref_dfp_usb;
	u32 cc2_vref_dfp_1_5;
	u32 cc2_vref_dfp_3_0;
	u32 debounce_val; /* 1b,1us 7f,4.7us */
	int cc_dfp_mode;

	bool use_defalut_parameter;

	/* Host/Device mode status */
	struct dwc3_rtk *dwc3_rtk;
	int dr_mode; /* current mode in type c cc status */

	/* type_c state */
	int connect_change;
#define CONNECT_CHANGE 1
#define CONNECT_NO_CHANGE 0
	int cc_mode; /* cc is host or device */
#define IN_HOST_MODE 1
#define IN_DEVICE_MODE 0
	int is_attach;
#define IN_ATTACH 1
#define TO_ATTACH 1
#define IN_DETACH 0
#define TO_DETACH 0
	int at_cc1;
#define AT_CC1 1
#define AT_CC2 0
	bool is_role_swap; /* if in role swap */
#define ROLE_SWAP 1
#define NO_ROLE_SWAP 0
#define TO_SWAP_ROLE 1
#define TO_RESTORE_ROLE 0

	u32 int_status;
	u32 cc_status;
	spinlock_t lock;
	struct delayed_work delayed_work;

	struct work_struct start_work;

	/* boot time check device mode transfer to host mode*/
	bool check_at_boot;
	int boot_check_time;
	struct delayed_work boot_check_work;

	/* force_cc_state_at_device_mode is a workaround
	   to force cc status at device mode */
	bool force_cc_state_at_device_mode;

	bool rd_en_at_first;

	/* A sw debounce to filter cc signal */
	bool filter_config_channel_signal;

	bool debug;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debug_dir;
#endif

#ifdef CONFIG_DUAL_ROLE_USB_INTF
	struct dual_role_phy_instance *drp;
#endif
#ifdef CONFIG_TYPEC
	struct typec_port *port;
#endif /* CONFIG_TYPEC */
};

/* Type C register offset */
#define USB_TYPEC_CTRL_CC1_0 0x0
#define USB_TYPEC_CTRL_CC1_1 0x4
#define USB_TYPEC_CTRL_CC2_0 0x8
#define USB_TYPEC_CTRL_CC2_1 0xC
#define USB_TYPEC_STS        0x10
#define USB_TYPEC_CTRL       0x14
#define USB_DBUS_PWR_CTRL    0x18

#define enable_cc1 0x1
#define enable_cc2 0x2
#define disable_cc 0x0

/* Bit mapping USB_TYPEC_CTRL_CC1_0 and USB_TYPEC_CTRL_CC2_0 */
#define PLR_EN BIT(29)
#define rp4pk_code(val) ((0x1f & (val)) << 22)
#define code_rp4pk(val) (((val) >> 22) & 0x1f)
#define rp36k_code(val) ((0x1f & (val)) << 17)
#define code_rp36k(val) (((val) >> 17) & 0x1f)
#define rp12k_code(val) ((0x1f & (val)) << 12)
#define code_rp12k(val) (((val) >> 12) & 0x1f)
#define rd_code(val) ((0x1f & (val)) << 7)
#define code_rd(val) (((val) >> 7) & 0x1f)
#define cc_mode(val) ((0x3 & (val)) << 5)
#define En_rp4p7k BIT(4)
#define En_rp36k BIT(3)
#define En_rp12k BIT(2)
#define En_rd BIT(1)
#define En_cc_det BIT(0)

#define CC_MODE_UFP 0x0
#define CC_MODE_DFP_USB 0x1
#define CC_MODE_DFP_1_5 0x2
#define CC_MODE_DFP_3_0 0x3

/* Bit mapping USB_TYPEC_CTRL_CC1_1 and USB_TYPEC_CTRL_CC2_1 */
#define V0_vref_2p6v(val) ((0xf & (val)) << 26) /* Bit 29 for groot */
#define V0_vref_1p23v(val) ((0xf & (val)) << 22)
#define V0_vref_0p8v(val) ((0xf & (val)) << 18)
#define V0_vref_0p66v(val) ((0xf & (val)) << 14)
#define V0_vref_0p4v(val) ((0x7 & (val)) << 11)
#define V0_vref_0p2v(val) ((0x7 & (val)) << 8)
#define V0_vref_1_1p6v(val) ((0xf & (val)) << 4)
#define V0_vref_0_1p6v(val) ((0xf & (val)) << 0)

#define V0_decode_2p6v(val) (((val) >> 26) & 0xf) /* Bit 29 for groot */
#define V0_decode_1p23v(val) (((val) >> 22) & 0xf)
#define V0_decode_0p8v(val) (((val) >> 18) & 0xf)
#define V0_decode_0p66v(val) (((val) >> 14) & 0xf)
#define V0_decode_0p4v(val) (((val) >> 11) & 0x7)
#define V0_decode_0p2v(val) (((val) >> 8) & 0x7)
#define V0_decode_1_1p6v(val) (((val) >> 4) & 0xf)
#define V0_decode_0_1p6v(val) (((val) >> 0) & 0xf)

/* new Bit mapping USB_TYPEC_CTRL_CC1_1 and USB_TYPEC_CTRL_CC2_1 */
#define V1_vref_2p6v(val) ((0xf & (val)) << 28)
#define V1_vref_1p23v(val) ((0xf & (val)) << 24)
#define V1_vref_0p8v(val) ((0xf & (val)) << 20)
#define V1_vref_0p66v(val) ((0xf & (val)) << 16)
#define V1_vref_0p4v(val) ((0xf & (val)) << 12)
#define V1_vref_0p2v(val) ((0xf & (val)) << 8)
#define V1_vref_1_1p6v(val) ((0xf & (val)) << 4)
#define V1_vref_0_1p6v(val) ((0xf & (val)) << 0)

#define V1_decode_2p6v(val) (((val) >> 28) & 0xf)
#define V1_decode_1p23v(val) (((val) >> 24) & 0xf)
#define V1_decode_0p8v(val) (((val) >> 20) & 0xf)
#define V1_decode_0p66v(val) (((val) >> 16) & 0xf)
#define V1_decode_0p4v(val) (((val) >> 12) & 0xf)
#define V1_decode_0p2v(val) (((val) >> 8) & 0xf)
#define V1_decode_1_1p6v(val) (((val) >> 4) & 0xf)
#define V1_decode_0_1p6v(val) (((val) >> 0) & 0xf)

/* Bit mapping USB_TYPEC_CTRL_CC1_1 and USB_TYPEC_CTRL_CC2_1 */
#define vref_2p6v(val) (type_c->para_ver?V1_vref_2p6v(val):V0_vref_2p6v(val))
#define vref_1p23v(val) (type_c->para_ver?V1_vref_1p23v(val):V0_vref_1p23v(val))
#define vref_0p8v(val) (type_c->para_ver?V1_vref_0p8v(val):V0_vref_0p8v(val))
#define vref_0p66v(val) (type_c->para_ver?V1_vref_0p66v(val):V0_vref_0p66v(val))
#define vref_0p4v(val) (type_c->para_ver?V1_vref_0p4v(val):V0_vref_0p4v(val))
#define vref_0p2v(val) (type_c->para_ver?V1_vref_0p2v(val):V0_vref_0p2v(val))
#define vref_1_1p6v(val) (type_c->para_ver?V1_vref_1_1p6v(val):V0_vref_1_1p6v(val))
#define vref_0_1p6v(val) (type_c->para_ver?V1_vref_0_1p6v(val):V0_vref_0_1p6v(val))

#define decode_2p6v(val) (type_c->para_ver?V1_decode_2p6v(val):V0_decode_2p6v(val))
#define decode_1p23v(val) (type_c->para_ver?V1_decode_1p23v(val):V0_decode_1p23v(val))
#define decode_0p8v(val) (type_c->para_ver?V1_decode_0p8v(val):V0_decode_0p8v(val))
#define decode_0p66v(val) (type_c->para_ver?V1_decode_0p66v(val):V0_decode_0p66v(val))
#define decode_0p4v(val) (type_c->para_ver?V1_decode_0p4v(val):V0_decode_0p4v(val))
#define decode_0p2v(val) (type_c->para_ver?V1_decode_0p2v(val):V0_decode_0p2v(val))
#define decode_1_1p6v(val) (type_c->para_ver?V1_decode_1_1p6v(val):V0_decode_1_1p6v(val))
#define decode_0_1p6v(val) (type_c->para_ver?V1_decode_0_1p6v(val):V0_decode_0_1p6v(val))

/* Bit mapping USB_TYPEC_STS */
#define det_sts 0x7
#define cc1_det_sts (det_sts)
#define cc2_det_sts (det_sts << 3)
#define det_sts_ra 0x1
#define det_sts_rd 0x3
#define det_sts_rp 0x1
#define cc1_det_sts_ra (det_sts_ra)
#define cc1_det_sts_rd (det_sts_rd)
#define cc1_det_sts_rp (det_sts_rp)
#define cc2_det_sts_ra (det_sts_ra << 3)
#define cc2_det_sts_rd (det_sts_rd << 3)
#define cc2_det_sts_rp (det_sts_rp << 3)

/* Bit mapping USB_TYPEC_CTRL */
#define cc2_int_en BIT(11)
#define cc1_int_en BIT(10)
#define cc2_int_sts BIT(9)
#define cc1_int_sts BIT(8)
#define debounce_time_MASK 0xff
#define ENABLE_TYPE_C_DETECT (cc1_int_en | cc2_int_en)
#define all_cc_int_sts (cc1_int_sts | cc2_int_sts)

/* Parameter */
#define DETECT_TIME 50 /* ms */

static void enable_writel(int value, void __iomem *addr)
{
	writel(value | readl(addr),  addr);
}

static void disable_writel(int value, void __iomem *addr)
{
	writel(~value & readl(addr),  addr);
}

static inline int rtk_type_c_init(struct type_c_data *type_c)
{
	dev_info(type_c->dev, "%s\n", __func__);

	rtk_usb_type_c_init(type_c->dev);

	return 0;
}

static int rtd129x_switch_type_c_plug_config(struct type_c_data *type_c,
	    int dr_mode, int cc)
{
	void __iomem *usb_typec_ctrl_cc1_0;
	int val_cc;

#define TYPE_C_EN_SWITCH BIT(29)
#define TYPE_C_TxRX_sel (BIT(28) | BIT(27))
#define TYPE_C_SWITCH_MASK (TYPE_C_EN_SWITCH | TYPE_C_TxRX_sel)
#define TYPE_C_enable_cc1 TYPE_C_EN_SWITCH
#define TYPE_C_enable_cc2 (TYPE_C_EN_SWITCH | TYPE_C_TxRX_sel)
#define TYPE_C_disable_cc ~TYPE_C_SWITCH_MASK

	usb_typec_ctrl_cc1_0 = type_c->type_c_reg_base + USB_TYPEC_CTRL_CC1_0;
	val_cc = readl(usb_typec_ctrl_cc1_0);
	val_cc &= ~TYPE_C_SWITCH_MASK;

	if (cc == disable_cc) {
		val_cc &= TYPE_C_disable_cc;
	} else if (cc == enable_cc1) {
		val_cc |= TYPE_C_enable_cc1;
	} else if (cc == enable_cc2) {
		val_cc |= TYPE_C_enable_cc2;
	} else {
		pr_err("%s: Error cc setting cc=0x%x\n", __func__, cc);
		return -1;
	}
	writel(val_cc, usb_typec_ctrl_cc1_0);

	mdelay(1);

	pr_info("%s: cc=0x%x val_cc=0x%x usb_typec_ctrl_cc1_0=0x%x\n",
		    __func__, cc, val_cc, readl(usb_typec_ctrl_cc1_0));

	return 0;
}

static inline void switch_type_c_plug_config(struct type_c_data *type_c,
	    int dr_mode, int cc)
{
	int ret = 0;

	dev_info(type_c->dev, "%s dr_mode=%d cc=0x%x\n", __func__, dr_mode, cc);

	if (soc_device_match(rtk_soc_kylin))
		ret = rtd129x_switch_type_c_plug_config(type_c, dr_mode, cc);

	if (ret < 0)
		dev_err(type_c->dev, "%s: Error set type c plug config\n",
			    __func__);

	rtk_usb_type_c_plug_config(type_c->dev, dr_mode, cc);
}

static void switch_type_c_dr_mode(struct type_c_data *type_c,
	    int dr_mode, int cc)
{
	dev_dbg(type_c->dev, "%s START....", __func__);

	switch_type_c_plug_config(type_c, dr_mode, cc);
	if (cc == disable_cc)
		msleep(1000);

	type_c->dr_mode = dwc3_rtk_set_dr_mode(type_c->dwc3_rtk, dr_mode);

#ifdef CONFIG_TYPEC
	if (type_c->port) {
		switch (dr_mode) {
		case USB_DR_MODE_HOST:
			typec_set_data_role(type_c->port, TYPEC_HOST);
			typec_set_pwr_role(type_c->port, TYPEC_SOURCE);
			break;
		case USB_DR_MODE_PERIPHERAL:
			typec_set_data_role(type_c->port,
				    TYPEC_DEVICE);
			typec_set_pwr_role(type_c->port, TYPEC_SINK);
			break;
		default:
			dev_dbg(type_c->dev, "%s unknown dr_mode=%d\n",
				    __func__, dr_mode);
			break;
		}
	}
#endif

	dev_dbg(type_c->dev, "%s END....", __func__);
}

/* device attached/detached */
static int device_attached(struct type_c_data *type_c, u32 enable_cc)
{
	struct device		*dev = type_c->dev;
	void __iomem *type_c_reg_base = type_c->type_c_reg_base;

	dev_info(dev, "%s: a device attach\n", __func__);

	cancel_delayed_work(&type_c->delayed_work);

	switch_type_c_dr_mode(type_c, USB_DR_MODE_HOST, enable_cc);

	enable_writel(ENABLE_TYPE_C_DETECT, type_c_reg_base + USB_TYPEC_CTRL);
	return 0;
}

static int device_detached(struct type_c_data *type_c)
{
	struct device		*dev = type_c->dev;
	void __iomem *type_c_reg_base = type_c->type_c_reg_base;

	dev_info(dev, "%s: a device detach\n", __func__);

	disable_writel(ENABLE_TYPE_C_DETECT, type_c_reg_base + USB_TYPEC_CTRL);

	switch_type_c_dr_mode(type_c, 0, disable_cc);

	schedule_delayed_work(&type_c->delayed_work,
			msecs_to_jiffies(DETECT_TIME));

	return 0;
}

/* host connect/disconnect*/
static int host_connected(struct type_c_data *type_c, u32 enable_cc)
{
	struct device		*dev = type_c->dev;
	void __iomem *type_c_reg_base = type_c->type_c_reg_base;

	dev_info(dev, "%s: a Host connect\n", __func__);

	cancel_delayed_work(&type_c->delayed_work);

	switch_type_c_dr_mode(type_c, USB_DR_MODE_PERIPHERAL, enable_cc);

	enable_writel(ENABLE_TYPE_C_DETECT, type_c_reg_base + USB_TYPEC_CTRL);
	return 0;
}

static int host_disconnected(struct type_c_data *type_c)
{
	struct device		*dev = type_c->dev;
	void __iomem *type_c_reg_base = type_c->type_c_reg_base;

	dev_info(dev, "%s: a Host disconnect\n", __func__);

	disable_writel(ENABLE_TYPE_C_DETECT, type_c_reg_base + USB_TYPEC_CTRL);

	switch_type_c_dr_mode(type_c, 0, disable_cc);

	schedule_delayed_work(&type_c->delayed_work,
		    msecs_to_jiffies(DETECT_TIME));

	return 0;
}

/* detect host device switch */
static int detect_device(struct type_c_data *type_c)
{
	struct device		*dev = type_c->dev;
	void __iomem *type_c_reg_base = type_c->type_c_reg_base;
	unsigned int gpio = type_c->rd_ctrl_gpio;
	u32 cc1_config, cc2_config, default_ctrl;
	int cc_mode_sel = type_c->cc_dfp_mode;

	/* For kylin to disable external rd control gpio */
	if (soc_device_match(rtk_soc_kylin)) {
		if (gpio != -1 && gpio_is_valid(gpio)) {
			if (gpio_direction_output(gpio, 1))
				dev_err(dev, "%s ERROR rd_ctrl_gpio=1 fail\n",
					    __func__);
		}
	}

	default_ctrl = readl(type_c_reg_base + USB_TYPEC_CTRL) &
		    debounce_time_MASK;
	writel(default_ctrl, type_c_reg_base + USB_TYPEC_CTRL);

	disable_writel(En_cc_det, type_c_reg_base + USB_TYPEC_CTRL_CC1_0);
	disable_writel(En_cc_det, type_c_reg_base + USB_TYPEC_CTRL_CC2_0);

	switch (cc_mode_sel) {
	case CC_MODE_DFP_USB:
		writel(type_c->cc1_vref_dfp_usb,
			    type_c_reg_base + USB_TYPEC_CTRL_CC1_1);
		writel(type_c->cc2_vref_dfp_usb,
			    type_c_reg_base + USB_TYPEC_CTRL_CC2_1);
		break;
	case CC_MODE_DFP_1_5:
		writel(type_c->cc1_vref_dfp_1_5,
			    type_c_reg_base + USB_TYPEC_CTRL_CC1_1);
		writel(type_c->cc2_vref_dfp_1_5,
			    type_c_reg_base + USB_TYPEC_CTRL_CC2_1);
		break;
	case CC_MODE_DFP_3_0:
		writel(type_c->cc1_vref_dfp_3_0,
			    type_c_reg_base + USB_TYPEC_CTRL_CC1_1);
		writel(type_c->cc2_vref_dfp_3_0,
			    type_c_reg_base + USB_TYPEC_CTRL_CC2_1);

		/* A workaround for Hank to support verf_2p6v MSB bit */
		if (soc_device_match(rtk_soc_hank)) {
			if (type_c->cc1_vref_dfp_3_0 & BIT(29))
				writel(BIT(12) |
				    readl(type_c_reg_base + USB_TYPEC_CTRL),
				    type_c_reg_base + USB_TYPEC_CTRL);
			if (type_c->cc2_vref_dfp_3_0 & BIT(29))
				writel(BIT(19) |
				    readl(type_c_reg_base + USB_TYPEC_CTRL),
				    type_c_reg_base + USB_TYPEC_CTRL);
		}

		break;
	default:
		dev_err(dev, "%s ERROR cc_mode_sel=%d\n",
			    __func__, cc_mode_sel);
		break;
	}
	cc1_config = type_c->cc1_rp | type_c->cc1_rp_code |
		    cc_mode(cc_mode_sel);
	cc2_config = type_c->cc2_rp | type_c->cc2_rp_code |
		    cc_mode(cc_mode_sel);

	writel(cc1_config, type_c_reg_base + USB_TYPEC_CTRL_CC1_0);
	writel(cc2_config, type_c_reg_base + USB_TYPEC_CTRL_CC2_0);

	/* Do wmb */
	wmb();

	enable_writel(En_cc_det, type_c_reg_base + USB_TYPEC_CTRL_CC1_0);
	enable_writel(En_cc_det, type_c_reg_base + USB_TYPEC_CTRL_CC2_0);

	return 0;
}

static int detect_host(struct type_c_data *type_c)
{
	struct device		*dev = type_c->dev;
	void __iomem *type_c_reg_base = type_c->type_c_reg_base;
	unsigned int gpio = type_c->rd_ctrl_gpio;
	u32 cc1_config, cc2_config, default_ctrl;
	u32 cc_rd = En_rd;

	if (type_c->rd_en_at_first) {
		if (soc_device_match(rtk_soc_kylin)) {
			cc_rd = 0;
			/* use external Rd */
			if (gpio != -1 && gpio_is_valid(gpio)) {
				if (gpio_direction_output(gpio, 0))
					dev_err(dev, "%s ERROR rd_ctrl_gpio=0 fail\n",
						 __func__);
			}
		} else {
			dev_info(dev, "%s set PLR_EN on rd_en_at_first\n",
				    __func__);
			cc_rd = PLR_EN;
		}
		type_c->rd_en_at_first = false;
	} else {
		/* For kylin to disable external rd control gpio */
		if (soc_device_match(rtk_soc_kylin)) {
			/* use internal Rd */
			if (gpio != -1 && gpio_is_valid(gpio)) {
				if (gpio_direction_output(gpio, 1))
					dev_err(dev, "%s ERROR rd_ctrl_gpio=1 fail\n",
				   		 __func__);
			}
		}
	}

	default_ctrl = readl(type_c_reg_base + USB_TYPEC_CTRL) &
		    debounce_time_MASK;
	writel(default_ctrl, type_c_reg_base + USB_TYPEC_CTRL);

	disable_writel(En_cc_det, type_c_reg_base + USB_TYPEC_CTRL_CC1_0);
	disable_writel(En_cc_det, type_c_reg_base + USB_TYPEC_CTRL_CC2_0);

	writel(type_c->cc1_vref_ufp, type_c_reg_base + USB_TYPEC_CTRL_CC1_1);
	writel(type_c->cc2_vref_ufp, type_c_reg_base + USB_TYPEC_CTRL_CC2_1);

	cc1_config = cc_rd | type_c->cc1_rd_code | cc_mode(CC_MODE_UFP);
	cc2_config = cc_rd | type_c->cc2_rd_code | cc_mode(CC_MODE_UFP);

	writel(cc1_config, type_c_reg_base + USB_TYPEC_CTRL_CC1_0);
	writel(cc2_config, type_c_reg_base + USB_TYPEC_CTRL_CC2_0);

	/* Do wmb */
	wmb();

	enable_writel(En_cc_det, type_c_reg_base + USB_TYPEC_CTRL_CC1_0);
	enable_writel(En_cc_det, type_c_reg_base + USB_TYPEC_CTRL_CC2_0);

	return 0;
}

static int host_device_switch_detection(struct type_c_data *type_c)
{
	struct device		*dev = type_c->dev;
	int ret = 0;

	if (type_c->debug)
		dev_dbg(dev, "ENTER %s", __func__);
	if (type_c->cc_mode) {
		type_c->cc_mode = IN_DEVICE_MODE;
		detect_host(type_c);
		if (type_c->debug)
			dev_dbg(dev, "Now device mode $$$$");
	} else {
		type_c->cc_mode = IN_HOST_MODE;
		detect_device(type_c);
		if (type_c->debug)
			dev_dbg(dev, "Now host mode   ####");
	}

	return ret;
}

static int detect_type_c_state(struct type_c_data *type_c)
{
	struct device *dev = type_c->dev;
	u32 int_status, cc_status, cc_status_check;
	unsigned long		flags;

	spin_lock_irqsave(&type_c->lock, flags);

	int_status = readl(type_c->type_c_reg_base + USB_TYPEC_CTRL);
	cc_status = readl(type_c->type_c_reg_base + USB_TYPEC_STS);

	if (type_c->force_cc_state_at_device_mode) {
		/* force_cc_state_at_device_mode is a workaround
		   to force cc status at device mode */
		cc_status = cc1_det_sts_rp;
		dev_info(dev, "force_cc_state_at_device_mode cc_status=0x%x\n",
			    cc_status);
	}

	type_c->connect_change = CONNECT_NO_CHANGE;

	switch (type_c->cc_mode) {
	case IN_HOST_MODE:
		switch (type_c->is_attach) {
		case IN_ATTACH:
			if (((cc_status & cc1_det_sts) == cc1_det_sts) &&
				    (type_c->at_cc1 == AT_CC1)) {
				dev_dbg(dev, "IN host mode and cc1 device detach (cc_status=0x%x)",
					    cc_status);
				type_c->is_attach = TO_DETACH;
				type_c->connect_change = CONNECT_CHANGE;
			} else if (((cc_status & cc2_det_sts) == cc2_det_sts) &&
				    (type_c->at_cc1 == AT_CC2)) {
				dev_dbg(dev, "IN host mode and cc2 device detach (cc_status=0x%x)",
				    cc_status);
				type_c->is_attach = TO_DETACH;
				type_c->connect_change = CONNECT_CHANGE;
			}
		break;
		case IN_DETACH:
			cc_status_check = readl(
				    type_c->type_c_reg_base + USB_TYPEC_STS);
			if (cc_status_check != (cc1_det_sts | cc2_det_sts)) {
				if (in_interrupt()) {
					mdelay(300);
				} else {
					spin_unlock_irqrestore(&type_c->lock,
						    flags);
					msleep(300);
					spin_lock_irqsave(&type_c->lock, flags);
				}
				cc_status_check = readl(
					    type_c->type_c_reg_base +
					       USB_TYPEC_STS);
			}
			if (cc_status != cc_status_check) {
				dev_warn(dev, "IN_HOST_MODE: cc_status (0x%x) != cc_status_check (0x%x)\n",
					    cc_status, cc_status_check);
				cc_status = readl(type_c->type_c_reg_base +
					    USB_TYPEC_STS);
			}

			if ((cc_status & cc1_det_sts) == cc1_det_sts_rd) {
				dev_dbg(dev, "IN host mode and cc1 device attach (cc_status=0x%x)",
					    cc_status);
				type_c->is_attach = TO_ATTACH;
				type_c->at_cc1 = AT_CC1;
				type_c->connect_change = CONNECT_CHANGE;
			} else if ((cc_status & cc2_det_sts) ==
					    cc2_det_sts_rd) {
				dev_dbg(dev, "In host mode and cc2 device attach (cc_status=0x%x)",
					    cc_status);
				type_c->is_attach = TO_ATTACH;
				type_c->at_cc1 = AT_CC2;
				type_c->connect_change = CONNECT_CHANGE;
			}
		break;
		default:
			dev_err(dev, "IN host_mode and error attach state (is_attach=%d)",
				    type_c->is_attach);
		}
	break;
	case IN_DEVICE_MODE:
		switch (type_c->is_attach) {
		case IN_ATTACH:
			if (type_c->filter_config_channel_signal &&
				    ((cc_status & cc1_det_sts) <
				      cc1_det_sts_rp ||
				    (cc_status & cc2_det_sts) <
				      cc2_det_sts_rp)) {
				/* Add a sw debounce to filter cc signal sent
				 * from apple pd adapter
				 */
				if (in_interrupt()) {
					mdelay(5);
				} else {
					spin_unlock_irqrestore(
						    &type_c->lock, flags);
					msleep(5);
					spin_lock_irqsave(&type_c->lock, flags);
				}
				cc_status_check = readl(
					    type_c->type_c_reg_base +
					      USB_TYPEC_STS);

				if (cc_status != cc_status_check) {
					dev_dbg(dev, "IN_DEVICE_MODE: cc_status (0x%x) != cc_status_check (0x%x) maybe use a pd adapter\n",
						    cc_status, cc_status_check);
					cc_status = cc_status_check;
				}
			}

			if ((cc_status & cc1_det_sts) < cc1_det_sts_rp &&
				    type_c->at_cc1 == AT_CC1) {
				dev_dbg(dev, "IN device mode and cc1 host disconnect (cc_status=0x%x)",
					    cc_status);
				type_c->is_attach = TO_DETACH;
				type_c->connect_change = CONNECT_CHANGE;
			} else if ((cc_status & cc2_det_sts) < cc2_det_sts_rp &&
				    type_c->at_cc1 == AT_CC2) {
				dev_dbg(dev, "IN device mode and cc2 host disconnect (cc_status=0x%x)",
					    cc_status);
				type_c->is_attach = TO_DETACH;
				type_c->connect_change = CONNECT_CHANGE;
			}
		break;
		case IN_DETACH:
			cc_status_check = readl(type_c->type_c_reg_base +
				    USB_TYPEC_STS);
			if (cc_status_check != 0x0) {
				if (in_interrupt()) {
					mdelay(300);
				} else {
					spin_unlock_irqrestore(&type_c->lock,
						    flags);
					msleep(300);
					spin_lock_irqsave(&type_c->lock, flags);
				}
				cc_status_check = readl(
					    type_c->type_c_reg_base +
					      USB_TYPEC_STS);
			}

			if (type_c->force_cc_state_at_device_mode) {
				/* force_cc_state_at_device_mode is a workaround
				   to force cc status at device mode */
				cc_status_check = cc1_det_sts_rp;
			}

			if (cc_status != cc_status_check) {
				dev_warn(dev, "IN_DEVICE_MODE: cc_status (0x%x) != cc_status_check (0x%x)\n",
					    cc_status, cc_status_check);
				cc_status = readl(type_c->type_c_reg_base +
					    USB_TYPEC_STS);
			}

			if ((cc_status & cc1_det_sts) >= cc1_det_sts_rp) {
				dev_dbg(dev, "IN device mode and cc1 host connect (cc_status=0x%x)",
					    cc_status);
				type_c->at_cc1 = AT_CC1;
				type_c->is_attach = TO_ATTACH;
				type_c->connect_change = CONNECT_CHANGE;
			} else if ((cc_status & cc2_det_sts) >=
					    cc2_det_sts_rp) {
				dev_dbg(dev, "IN device mode and cc2 host connect (cc_status=0x%x)",
					    cc_status);
				type_c->at_cc1 = AT_CC2;
				type_c->is_attach = TO_ATTACH;
				type_c->connect_change = CONNECT_CHANGE;
			}
		break;
		default:
			dev_err(dev, "IN device_mode and error attach state (is_attach=%d)",
				    type_c->is_attach);
		}
	break;
	default:
		dev_err(dev, "error host or device mode (cc_mode=%d)",
			    type_c->cc_mode);
	}

	type_c->int_status = int_status;
	type_c->cc_status = cc_status;

	spin_unlock_irqrestore(&type_c->lock, flags);
	return 0;
}

static void host_device_switch(struct work_struct *work)
{
	struct type_c_data *type_c = container_of(work,
		    struct type_c_data, delayed_work.work);
	struct device		*dev = type_c->dev;
	unsigned long		flags;
	int connect_change = 0;
	int cc_mode = 0;
	int is_attach = 0;
	int at_cc1 = 0;

	if (type_c->debug)
		dev_dbg(type_c->dev, "ENTER %s", __func__);

	spin_lock_irqsave(&type_c->lock, flags);
	if (type_c->connect_change)
		connect_change = type_c->connect_change;
	spin_unlock_irqrestore(&type_c->lock, flags);

	if (!connect_change)
		detect_type_c_state(type_c);

	spin_lock_irqsave(&type_c->lock, flags);
	if (type_c->connect_change) {
		connect_change = type_c->connect_change;
		cc_mode = type_c->cc_mode;
		is_attach = type_c->is_attach;
		at_cc1 = type_c->at_cc1;
		type_c->connect_change = CONNECT_NO_CHANGE;
		type_c->is_role_swap = NO_ROLE_SWAP;
	} else {
		host_device_switch_detection(type_c);

		schedule_delayed_work(&type_c->delayed_work,
			    msecs_to_jiffies(DETECT_TIME));
	}
	spin_unlock_irqrestore(&type_c->lock, flags);

	if (connect_change) {
		dev_info(dev, "%s: usb cable connection change\n", __func__);
#ifdef CONFIG_USB_TYPE_C_RTK_RTS5400
		if (type_c->check_at_boot
				&& rtk_rts5400_is_enabled()
				&& rtk_rts5400_is_UFP_attached()) {
			u32 enable_cc = at_cc1?enable_cc1:enable_cc2;

			dev_info(dev, "%s: In Device mode, role swap to Host mode\n",
				    __func__);
			switch_type_c_dr_mode(type_c, USB_DR_MODE_HOST,
				    enable_cc);

			rtk_rts5400_set_type_c_soft_reset();

			type_c->check_at_boot = false;
		} else
#endif //CONFIG_USB_TYPE_C_RTK_RTS5400
		if (cc_mode) {
			if (is_attach && at_cc1)
				device_attached(type_c, enable_cc1);
			else if (is_attach && !at_cc1)
				device_attached(type_c, enable_cc2);
			else
				device_detached(type_c);
		} else {
			if (is_attach && at_cc1)
				host_connected(type_c, enable_cc1);
			else if (is_attach && !at_cc1)
				host_connected(type_c, enable_cc2);
			else
				host_disconnected(type_c);
		}
		dev_err(dev, "Connection change OK: IN %s mode to %s %s at %s (cc_status=0x%x)\n",
			    cc_mode?"host":"device",
			    cc_mode ?
			      (is_attach?"attach":"detach") :
			      (is_attach?"connect":"disconnect"),
			    cc_mode?"device":"host",
			    at_cc1?"cc1":"cc2", type_c->cc_status);

	}

	/* For special case, some boards use type c power and
	 * need use host mode.
	 * After 30s, We switch to host mode if in device mode
	 * but no host connect.
	 */
	if (type_c->check_at_boot) {
		if (connect_change &&
			    (cc_mode == IN_DEVICE_MODE) &&
			    is_attach) {
			dev_info(dev, "%s: In Device mode check connection at boot time\n",
				    __func__);
			schedule_delayed_work(&type_c->boot_check_work,
				    msecs_to_jiffies(type_c->boot_check_time));
		}
		type_c->check_at_boot = false;
	}
}

static int host_device_role_swap(struct type_c_data *type_c, bool swap_role)
{
	int ret = 0;
	int swap_to_xx_mode = USB_DR_MODE_UNKNOWN;
	int at_cc1, cc_mode, is_attach;
	unsigned long		flags;

	spin_lock_irqsave(&type_c->lock, flags);

	at_cc1 = type_c->at_cc1;
	cc_mode = type_c->cc_mode;
	is_attach = type_c->is_attach;

	spin_unlock_irqrestore(&type_c->lock, flags);

	if (!is_attach) {
		dev_warn(type_c->dev, "Can not swap role!! It is Not attach\n");
		return -1;
	}

	dev_info(type_c->dev, "%s: In %s Mode and now is%s role swap ",
		    __func__,
		    cc_mode == IN_HOST_MODE?"Host":"Device",
		    type_c->is_role_swap == ROLE_SWAP?"":" Not");

	if (swap_role == TO_RESTORE_ROLE &&
		    type_c->is_role_swap == ROLE_SWAP)
		/* Restore Role */
		if (cc_mode == IN_DEVICE_MODE)
			swap_to_xx_mode = USB_DR_MODE_PERIPHERAL;
		else
			swap_to_xx_mode = USB_DR_MODE_HOST;
	else if (swap_role == TO_SWAP_ROLE &&
		    type_c->is_role_swap == NO_ROLE_SWAP)
		/* Swap Role */
		if (cc_mode == IN_DEVICE_MODE)
			swap_to_xx_mode = USB_DR_MODE_HOST;
		else
			swap_to_xx_mode = USB_DR_MODE_PERIPHERAL;
	else
		dev_info(type_c->dev, "%s: Other Case swap_role=%x is_role_swap=%x\n",
			    __func__, swap_role, type_c->is_role_swap);

	if (swap_to_xx_mode != USB_DR_MODE_UNKNOWN) {
		u32 enable_cc = at_cc1?enable_cc1:enable_cc2;

		dev_info(type_c->dev, "%s: now is%s role swap ==> to %s role (swap_to_xx_mode=%s\n",
			    __func__,
			    type_c->is_role_swap == ROLE_SWAP?"":" No",
			    swap_role == TO_SWAP_ROLE?"swap":"restore",
			    ({ char *tmp;
			switch (swap_to_xx_mode) {
			case USB_DR_MODE_PERIPHERAL:
			    tmp = "USB_DR_MODE_PERIPHERAL"; break;
			case USB_DR_MODE_HOST:
			    tmp = "USB_DR_MODE_HOST"; break;
			default:
			    tmp = "USB_DR_MODE_UNKNOWN"; break;
			    } tmp; }));

		switch_type_c_plug_config(type_c, 0, disable_cc);
		mdelay(100);
		switch_type_c_dr_mode(type_c, swap_to_xx_mode, enable_cc);

		type_c->is_role_swap = (swap_role == TO_SWAP_ROLE ?
			    ROLE_SWAP : NO_ROLE_SWAP);
	} else {
		dev_info(type_c->dev, "%s: No swap and No restore\n", __func__);
	}

	return ret;
}

static void boot_time_check(struct work_struct *work)
{
	struct type_c_data *type_c = container_of(work,
		    struct type_c_data, boot_check_work.work);
	struct device		*dev = type_c->dev;
	int at_cc1, cc_mode, is_attach;
	unsigned long		flags;

	spin_lock_irqsave(&type_c->lock, flags);

	at_cc1 = type_c->at_cc1;
	cc_mode = type_c->cc_mode;
	is_attach = type_c->is_attach;

	spin_unlock_irqrestore(&type_c->lock, flags);

	if ((cc_mode == IN_DEVICE_MODE) && is_attach) {

		if (!dwc3_rtk_is_connected_on_device_mode(
			    type_c->dwc3_rtk)) {
			dev_info(dev, "%s: In Device mode, NO host connect at boot time (After %dms), switch to Host mode\n",
				    __func__, type_c->boot_check_time);

			host_device_role_swap(type_c, TO_SWAP_ROLE);
		}
	}
}

irqreturn_t type_c_detect_irq(int irq, void *__data)
{
	struct type_c_data	*type_c = (struct type_c_data *) __data;
	struct device		*dev = type_c->dev;
	unsigned long		flags;

	detect_type_c_state(type_c);

	spin_lock_irqsave(&type_c->lock, flags);

	if (type_c->connect_change) {
		dev_info(dev, "%s: IN %s mode to %s %s (at %s interrupt) int_status=0x%x, cc_status=0x%x",
			    __func__,
			    type_c->cc_mode?"host":"device",
			    type_c->cc_mode ?
			      (type_c->is_attach?"attach":"detach") :
			      (type_c->is_attach?"connect":"disconnect"),
			    type_c->cc_mode?"device":"host",
			    type_c->at_cc1?"cc1":"cc2",
			    type_c->int_status, type_c->cc_status);

		/* clear interrupt status */
		disable_writel(all_cc_int_sts,
			    type_c->type_c_reg_base + USB_TYPEC_CTRL);

		cancel_delayed_work(&type_c->delayed_work);
		schedule_delayed_work(&type_c->delayed_work,
			    msecs_to_jiffies(0));
#ifdef CONFIG_DUAL_ROLE_USB_INTF
		if (type_c->drp)
			dual_role_instance_changed(type_c->drp);
#endif
	} else {
		static int local_count;

		if (local_count++ > 10) {
			/* clear interrupt status */
			disable_writel(all_cc_int_sts,
			    type_c->type_c_reg_base + USB_TYPEC_CTRL);
			local_count = 0;
		}
		if (type_c->debug)
			dev_dbg(dev, "%s: ###NO change### Status: IN %s mode %s %s (at %s interrupt)\n",
				    __func__,
				    type_c->cc_mode?"host":"device",
				    type_c->cc_mode ?
				     (type_c->is_attach?"attach":"detach") :
				     (type_c->is_attach?"connect":"disconnect"),
				    type_c->cc_mode?"device":"host",
				    type_c->at_cc1?"cc1":"cc2");
			dev_dbg(dev, "%s: int_status=0x%x, cc_status=0x%x\n",
				    __func__,
				    type_c->int_status, type_c->cc_status);
	}

	spin_unlock_irqrestore(&type_c->lock, flags);

	return IRQ_HANDLED;
}

static ssize_t role_swap_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct type_c_data *type_c = dev_get_drvdata(dev);
	char *ptr = buf;
	int count = PAGE_SIZE;
	int n;

	n = scnprintf(ptr, count,
		     "Now cc_mode is %s and cur_mode is %s ==> Now is%s role swap\n",
		    ({ char *tmp;
		switch (type_c->cc_mode) {
		case IN_DEVICE_MODE:
		    tmp = "IN_DEVICE_MODE"; break;
		case IN_HOST_MODE:
		    tmp = "IN_HOST_MODE"; break;
		default:
		    tmp = "UNKNOWN"; break;
		    } tmp; }),
		    ({ char *tmp;
		switch (type_c->dr_mode) {
		case USB_DR_MODE_PERIPHERAL:
		    tmp = "USB_DR_MODE_PERIPHERAL"; break;
		case USB_DR_MODE_HOST:
		    tmp = "USB_DR_MODE_HOST"; break;
		default:
		    tmp = "USB_DR_MODE_UNKNOWN"; break;
		    } tmp; }),
		    type_c->is_role_swap == ROLE_SWAP?"":" not");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count,
		     "write 1 to swap role, ex: echo 1 > role_swap\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count,
		     "write 0 to restore role, ex: echo 0 > role_swap\n");
	ptr += n;
	count -= n;

	return ptr - buf;
}

static ssize_t role_swap_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct type_c_data *type_c = dev_get_drvdata(dev);

	if (!strncmp(buf, "1", 1))
		host_device_role_swap(type_c, TO_SWAP_ROLE);
	else if (!strncmp(buf, "0", 1))
		host_device_role_swap(type_c, TO_RESTORE_ROLE);

	return count;
}
static DEVICE_ATTR_RW(role_swap);

static ssize_t force_set_mode_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct type_c_data *type_c = dev_get_drvdata(dev);
	char *ptr = buf;
	int count = PAGE_SIZE;
	int n;

	n = scnprintf(ptr, count,
		     "Now cc_mode is %s and type c dr_mode is %s (dwc3_rtk dr_mode=%s)\n",
		    ({ char *tmp;
		switch (type_c->cc_mode) {
		case IN_DEVICE_MODE:
		    tmp = "IN_DEVICE_MODE"; break;
		case IN_HOST_MODE:
		    tmp = "IN_HOST_MODE"; break;
		default:
		    tmp = "UNKNOWN"; break;
		    } tmp; }),
		    ({ char *tmp;
		switch (type_c->dr_mode) {
		case USB_DR_MODE_PERIPHERAL:
		    tmp = "USB_DR_MODE_PERIPHERAL"; break;
		case USB_DR_MODE_HOST:
		    tmp = "USB_DR_MODE_HOST"; break;
		default:
		    tmp = "USB_DR_MODE_UNKNOWN"; break;
		    } tmp; }),
		    ({ char *tmp;
		switch (dwc3_rtk_get_dr_mode(type_c->dwc3_rtk)) {
		case USB_DR_MODE_PERIPHERAL:
		    tmp = "USB_DR_MODE_PERIPHERAL"; break;
		case USB_DR_MODE_HOST:
		    tmp = "USB_DR_MODE_HOST"; break;
		default:
		    tmp = "USB_DR_MODE_UNKNOWN"; break;
		    } tmp; }));

	ptr += n;
	count -= n;

	n = scnprintf(ptr, count,
		     "write host -> switch to Host mode\n");
	ptr += n;
	count -= n;

	n = scnprintf(ptr, count,
		     "write device -> switch to Device mode\n");
	ptr += n;
	count -= n;

	return ptr - buf;
}

static ssize_t force_set_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct type_c_data *type_c = dev_get_drvdata(dev);
	unsigned long flags;
	u32 enable_cc;

	spin_lock_irqsave(&type_c->lock, flags);
	enable_cc = type_c->at_cc1?enable_cc1:enable_cc2;
	spin_unlock_irqrestore(&type_c->lock, flags);


	if (!strncmp(buf, "host", 4)) {
		switch_type_c_dr_mode(type_c, USB_DR_MODE_HOST, enable_cc);
	} else if (!strncmp(buf, "device", 6)) {
		switch_type_c_dr_mode(type_c, USB_DR_MODE_PERIPHERAL,
			    enable_cc);
	} else {
		switch_type_c_dr_mode(type_c, 0, disable_cc);
	}
	return count;
}
static DEVICE_ATTR_RW(force_set_mode);

#ifdef CONFIG_DEBUG_FS
static int type_c_parameter_show(struct seq_file *s, void *unused)
{
	struct type_c_data		*type_c = s->private;
	unsigned long		flags;

	spin_lock_irqsave(&type_c->lock, flags);

	seq_printf(s, "cc_dfp_mode %s\n",
		    ({ char *tmp;
		switch (type_c->cc_dfp_mode) {
		case CC_MODE_DFP_USB:
		    tmp = "CC_MODE_DFP_USB"; break;
		case CC_MODE_DFP_1_5:
		    tmp = "CC_MODE_DFP_1_5"; break;
		case CC_MODE_DFP_3_0:
		    tmp = "CC_MODE_DFP_3_0"; break;
		default:
		    tmp = "?"; break;
		     } tmp; }));
	seq_printf(s, "cc1_rp 0x%x\n", type_c->cc1_rp);
	seq_printf(s, "cc1_rp_code 0x%x\n",
		    ({ int tmp;
		switch (type_c->cc_dfp_mode) {
		case CC_MODE_DFP_USB:
		    tmp = code_rp36k(type_c->cc1_rp_code); break;
		case CC_MODE_DFP_1_5:
		    tmp = code_rp12k(type_c->cc1_rp_code); break;
		case CC_MODE_DFP_3_0:
		    tmp = code_rp4pk(type_c->cc1_rp_code); break;
		default:
		    tmp = -1; break;
		    } tmp; }));
	seq_printf(s, "cc1_rd_code 0x%x\n",
		    code_rd(type_c->cc1_rd_code));
	seq_printf(s, "cc1_vref_ufp     vref_1p23v 0x%x vref_0p66v 0x%x vref_0p2v 0x%x\n",
		    decode_1p23v(type_c->cc1_vref_ufp),
		    decode_0p66v(type_c->cc1_vref_ufp),
		    decode_0p2v(type_c->cc1_vref_ufp));
	seq_printf(s, "cc1_vref_dfp_usb vref_0_1p6v 0x%x vref_0p2v 0x%x\n",
		    decode_0_1p6v(type_c->cc1_vref_dfp_usb),
		    decode_0p2v(type_c->cc1_vref_dfp_usb));
	seq_printf(s, "cc1_vref_dfp_1_5 vref_1_1p6v 0x%x vref_0p4v 0x%x vref_0p2v 0x%x\n",
		    decode_1_1p6v(type_c->cc1_vref_dfp_1_5),
		    decode_0p4v(type_c->cc1_vref_dfp_1_5),
		    decode_0p2v(type_c->cc1_vref_dfp_1_5));
	seq_printf(s, "cc1_vref_dfp_3_0 vref_2p6v   0x%x vref_0p8v 0x%x vref_0p2v 0x%x\n",
		    decode_2p6v(type_c->cc1_vref_dfp_3_0),
		    decode_0p8v(type_c->cc1_vref_dfp_3_0),
		    decode_0p2v(type_c->cc1_vref_dfp_3_0));
	seq_printf(s, "cc2_rp 0x%x\n", type_c->cc2_rp);
	seq_printf(s, "cc2_rp_code 0x%x\n",
		    ({ int tmp;
		switch (type_c->cc_dfp_mode) {
		case CC_MODE_DFP_USB:
		    tmp = code_rp36k(type_c->cc2_rp_code); break;
		case CC_MODE_DFP_1_5:
		    tmp = code_rp12k(type_c->cc2_rp_code); break;
		case CC_MODE_DFP_3_0:
		    tmp = code_rp4pk(type_c->cc2_rp_code); break;
		default:
		    tmp = -1; break;
		    } tmp; }));
	seq_printf(s, "cc2_rd_code 0x%x\n",
		    code_rd(type_c->cc2_rd_code));
	seq_printf(s, "cc2_vref_ufp     vref_1p23v 0x%x vref_0p66v 0x%x vref_0p2v 0x%x\n",
		    decode_1p23v(type_c->cc2_vref_ufp),
		    decode_0p66v(type_c->cc2_vref_ufp),
		    decode_0p2v(type_c->cc2_vref_ufp));
	seq_printf(s, "cc2_vref_dfp_usb vref_0_1p6v 0x%x vref_0p2v 0x%x\n",
		    decode_0_1p6v(type_c->cc2_vref_dfp_usb),
		    decode_0p2v(type_c->cc2_vref_dfp_usb));
	seq_printf(s, "cc2_vref_dfp_1_5 vref_1_1p6v 0x%x vref_0p4v 0x%x vref_0p2v 0x%x\n",
		    decode_1_1p6v(type_c->cc2_vref_dfp_1_5),
		    decode_0p4v(type_c->cc2_vref_dfp_1_5),
		    decode_0p2v(type_c->cc2_vref_dfp_1_5));
	seq_printf(s, "cc2_vref_dfp_3_0 vref_2p6v   0x%x vref_0p8v 0x%x vref_0p2v 0x%x\n",
		    decode_2p6v(type_c->cc2_vref_dfp_3_0),
		    decode_0p8v(type_c->cc2_vref_dfp_3_0),
		    decode_0p2v(type_c->cc2_vref_dfp_3_0));
	seq_printf(s, "debounce_val 0x%x\n", type_c->debounce_val);
	seq_puts(s, "\n");

	spin_unlock_irqrestore(&type_c->lock, flags);

	return 0;
}

static int type_c_parameter_open(struct inode *inode, struct file *file)
{
	return single_open(file, type_c_parameter_show, inode->i_private);
}

static const struct file_operations type_c_parameter_fops = {
	.open			= type_c_parameter_open,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int type_c_set_parameter_show(struct seq_file *s, void *unused)
{
	seq_puts(s, "cc_dfp_mode [CC_MODE_DFP_USB|CC_MODE_DFP_1_5|CC_MODE_DFP_3_0]\n");
	seq_puts(s, "cc1_rp_code 0x_value\n");
	seq_puts(s, "cc1_rd_code 0x_value\n");
	seq_puts(s, "cc1_vref_ufp_vref_1p23v 0x_value\n");
	seq_puts(s, "cc1_vref_ufp_vref_0p66v 0x_value\n");
	seq_puts(s, "cc1_vref_ufp_vref_0p2v  0x_value\n");
	seq_puts(s, "cc1_vref_dfp_usb_vref_1p6v 0x_value\n");
	seq_puts(s, "cc1_vref_dfp_usb_vref_0p2v 0x_value\n");
	seq_puts(s, "cc1_vref_dfp_1_5_vref_1p6v 0x_value\n");
	seq_puts(s, "cc1_vref_dfp_1_5_vref_0p4v 0x_value\n");
	seq_puts(s, "cc1_vref_dfp_1_5_vref_0p2v 0x_value\n");
	seq_puts(s, "cc1_vref_dfp_3_0_vref_2p6v 0x_value\n");
	seq_puts(s, "cc1_vref_dfp_3_0_vref_0p8v 0x_value\n");
	seq_puts(s, "cc1_vref_dfp_3_0_vref_0p2v 0x_value\n");
	seq_puts(s, "cc2_rp_code 0x_value\n");
	seq_puts(s, "cc2_rd_code 0x_value\n");
	seq_puts(s, "cc2_vref_ufp_vref_1p23v 0x_value\n");
	seq_puts(s, "cc2_vref_ufp_vref_0p66v 0x_value\n");
	seq_puts(s, "cc2_vref_ufp_vref_0p2v  0x_value\n");
	seq_puts(s, "cc2_vref_dfp_usb_vref_1p6v 0x_value\n");
	seq_puts(s, "cc2_vref_dfp_usb_vref_0p2v 0x_value\n");
	seq_puts(s, "cc2_vref_dfp_1_5_vref_1p6v 0x_value\n");
	seq_puts(s, "cc2_vref_dfp_1_5_vref_0p4v 0x_value\n");
	seq_puts(s, "cc2_vref_dfp_1_5_vref_0p2v 0x_value\n");
	seq_puts(s, "cc2_vref_dfp_3_0_vref_2p6v 0x_value\n");
	seq_puts(s, "cc2_vref_dfp_3_0_vref_0p8v 0x_value\n");
	seq_puts(s, "cc2_vref_dfp_3_0_vref_0p2v 0x_value\n");
	seq_puts(s, "debounce_val value\n");

	return 0;
}

static int type_c_set_parameter_open(struct inode *inode, struct file *file)
{
	return single_open(file, type_c_set_parameter_show, inode->i_private);
}

static ssize_t type_c_set_parameter_write(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file		*s = file->private_data;
	struct type_c_data		*type_c = s->private;
	unsigned long		flags;
	char			buffer[40];
	char *buf = buffer;
	u32 value;
	int ret = 0;

	if (copy_from_user(&buffer, ubuf,
		    min_t(size_t, sizeof(buffer) - 1, count)))
		return -EFAULT;

	spin_lock_irqsave(&type_c->lock, flags);
	if (!strncmp(buf, "cc_dfp_mode", 11)) {
		buf = buf + 11;
		buf = skip_spaces(buf);
		if (!strncmp(buf, "CC_MODE_DFP_USB", 15)) {
			type_c->cc_dfp_mode = CC_MODE_DFP_USB;
			type_c->cc1_rp = En_rp36k;
			type_c->cc2_rp = En_rp36k;
		} else if (!strncmp(buf, "CC_MODE_DFP_1_5", 15)) {
			type_c->cc_dfp_mode = CC_MODE_DFP_1_5;
			type_c->cc1_rp = En_rp12k;
			type_c->cc2_rp = En_rp12k;
		} else if (!strncmp(buf, "CC_MODE_DFP_3_0", 15)) {
			type_c->cc_dfp_mode = CC_MODE_DFP_3_0;
			type_c->cc1_rp = En_rp4p7k;
			type_c->cc2_rp = En_rp4p7k;
		} else {
			dev_err(type_c->dev, "cc_dfp_mode UNKNOWN (%s)", buf);
		}
	} else if (!strncmp(buf, "cc1_rp_code", 11)) {
		buf = buf + 11;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		if (type_c->cc_dfp_mode == CC_MODE_DFP_USB)
			type_c->cc1_rp_code = rp36k_code(value);
		else if (type_c->cc_dfp_mode == CC_MODE_DFP_1_5)
			type_c->cc1_rp_code = rp12k_code(value);
		else if (type_c->cc_dfp_mode == CC_MODE_DFP_3_0)
			type_c->cc1_rp_code = rp4pk_code(value);
	} else if (!strncmp(buf, "cc1_rd_code", 11)) {
		buf = buf + 11;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc1_rd_code = rd_code(value);
	} else if (!strncmp(buf, "cc1_vref_ufp_vref_1p23v", 23)) {
		buf = buf + 23;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc1_vref_ufp = (type_c->cc1_vref_ufp &
			    (~vref_1p23v(0xf))) |
			    vref_1p23v(value);
	} else if (!strncmp(buf, "cc1_vref_ufp_vref_0p66v", 23)) {
		buf = buf + 23;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc1_vref_ufp = (type_c->cc1_vref_ufp &
			    (~vref_0p66v(0xf))) |
			    vref_0p66v(value);
	} else if (!strncmp(buf, "cc1_vref_ufp_vref_0p2v", 22)) {
		buf = buf + 22;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc1_vref_ufp = (type_c->cc1_vref_ufp &
			    (~vref_0p2v(0x7))) |
			    vref_0p2v(value);
	} else if (!strncmp(buf, "cc1_vref_dfp_usb_vref_1p6v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc1_vref_dfp_usb = (type_c->cc1_vref_dfp_usb &
			    (~vref_0_1p6v(0xf))) | vref_0_1p6v(value);
	} else if (!strncmp(buf, "cc1_vref_dfp_usb_vref_0p2v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc1_vref_dfp_usb = (type_c->cc1_vref_dfp_usb &
			    (~vref_0p2v(0x7))) | vref_0p2v(value);
	} else if (!strncmp(buf, "cc1_vref_dfp_1_5_vref_1p6v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc1_vref_dfp_1_5 = (type_c->cc1_vref_dfp_1_5 &
			    (~vref_1_1p6v(0xf))) | vref_1_1p6v(value);
	} else if (!strncmp(buf, "cc1_vref_dfp_1_5_vref_0p4v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc1_vref_dfp_1_5 = (type_c->cc1_vref_dfp_1_5 &
			    (~vref_0p4v(0x7))) | vref_0p4v(value);
	} else if (!strncmp(buf, "cc1_vref_dfp_1_5_vref_0p2v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc1_vref_dfp_1_5 = (type_c->cc1_vref_dfp_1_5 &
			    (~vref_0p2v(0x7))) | vref_0p2v(value);
	} else if (!strncmp(buf, "cc1_vref_dfp_3_0_vref_2p6v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc1_vref_ufp = (type_c->cc1_vref_dfp_1_5 &
			    (~vref_2p6v(0x7))) |
			    vref_2p6v(value);
	} else if (!strncmp(buf, "cc1_vref_dfp_3_0_vref_0p8v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc1_vref_dfp_3_0 = (type_c->cc1_vref_dfp_3_0 &
			    (~vref_0p8v(0xf))) | vref_0p8v(value);
	} else if (!strncmp(buf, "cc1_vref_dfp_3_0_vref_0p2v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc1_vref_dfp_3_0 = (type_c->cc1_vref_dfp_3_0 &
			    (~vref_0p2v(0x7))) | vref_0p2v(value);
	} else if (!strncmp(buf, "cc2_rp_code", 11)) {
		buf = buf + 11;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		if (type_c->cc_dfp_mode == CC_MODE_DFP_USB)
			type_c->cc2_rp_code = rp36k_code(value);
		else if (type_c->cc_dfp_mode == CC_MODE_DFP_1_5)
			type_c->cc2_rp_code = rp12k_code(value);
		else if (type_c->cc_dfp_mode == CC_MODE_DFP_3_0)
			type_c->cc2_rp_code = rp4pk_code(value);
	} else if (!strncmp(buf, "cc2_rd_code", 11)) {
		buf = buf + 11;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc2_rd_code = rd_code(value);
	} else if (!strncmp(buf, "cc2_vref_ufp_vref_1p23v", 23)) {
		buf = buf + 23;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc2_vref_ufp = (type_c->cc2_vref_ufp &
			    (~vref_1p23v(0xf))) |
			    vref_1p23v(value);
	} else if (!strncmp(buf, "cc2_vref_ufp_vref_0p66v", 23)) {
		buf = buf + 23;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc2_vref_ufp = (type_c->cc2_vref_ufp &
			    (~vref_0p66v(0xf))) |
			    vref_0p66v(value);
	} else if (!strncmp(buf, "cc2_vref_ufp_vref_0p2v", 22)) {
		buf = buf + 22;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc2_vref_ufp = (type_c->cc2_vref_ufp &
			    (~vref_0p2v(0x7))) |
			    vref_0p2v(value);
	} else if (!strncmp(buf, "cc2_vref_dfp_usb_vref_1p6v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc2_vref_dfp_usb = (type_c->cc2_vref_dfp_usb &
			    (~vref_0_1p6v(0xf))) | vref_0_1p6v(value);
	} else if (!strncmp(buf, "cc2_vref_dfp_usb_vref_0p2v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc2_vref_dfp_usb = (type_c->cc2_vref_dfp_usb &
			    (~vref_0p2v(0x7))) | vref_0p2v(value);
	} else if (!strncmp(buf, "cc2_vref_dfp_1_5_vref_1p6v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc2_vref_dfp_1_5 = (type_c->cc2_vref_dfp_1_5 &
			    (~vref_1_1p6v(0xf))) | vref_1_1p6v(value);
	} else if (!strncmp(buf, "cc2_vref_dfp_1_5_vref_0p4v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc2_vref_dfp_1_5 = (type_c->cc2_vref_dfp_1_5 &
			    (~vref_0p4v(0x7))) | vref_0p4v(value);
	} else if (!strncmp(buf, "cc2_vref_dfp_1_5_vref_0p2v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc2_vref_dfp_1_5 = (type_c->cc2_vref_dfp_1_5 &
			    (~vref_0p2v(0x7))) | vref_0p2v(value);
	} else if (!strncmp(buf, "cc2_vref_dfp_3_0_vref_2p6v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc2_vref_dfp_3_0 = (type_c->cc2_vref_dfp_3_0 &
			    (~vref_2p6v(0x7))) | vref_2p6v(value);
	} else if (!strncmp(buf, "cc2_vref_dfp_3_0_vref_0p8v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc2_vref_dfp_3_0 = (type_c->cc2_vref_dfp_3_0 &
			    (~vref_0p8v(0xf))) | vref_0p8v(value);
	} else if (!strncmp(buf, "cc2_vref_dfp_3_0_vref_0p2v", 26)) {
		buf = buf + 26;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->cc2_vref_dfp_3_0 = (type_c->cc2_vref_dfp_3_0 &
			    (~vref_0p2v(0x7))) | vref_0p2v(value);
	} else if (!strncmp(buf, "debounce_val", 12)) {
		buf = buf + 12;
		buf = skip_spaces(buf);
		ret = kstrtoint(buf, 0, &value);
		if (ret < 0) {
			dev_err(type_c->dev, "kstrtoint ret error (%d)", ret);
			return -EFAULT;
		}
		type_c->debounce_val = value;
	} else
		dev_err(type_c->dev, "UNKNOWN input (%s)", buf);

	spin_unlock_irqrestore(&type_c->lock, flags);
	return count;
}

static const struct file_operations type_c_set_parameter_fops = {
	.open			= type_c_set_parameter_open,
	.write			= type_c_set_parameter_write,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int type_c_status_show(struct seq_file *s, void *unused)
{
	struct type_c_data		*type_c = s->private;
	unsigned long		flags;

	spin_lock_irqsave(&type_c->lock, flags);

	seq_printf(s, "Now dr_mode is %s (Is%s role swap)\n",
		    ({ char *tmp;
		switch (type_c->dr_mode) {
		case USB_DR_MODE_PERIPHERAL:
		    tmp = "USB_DR_MODE_PERIPHERAL"; break;
		case USB_DR_MODE_HOST:
		    tmp = "USB_DR_MODE_HOST"; break;
		default:
		    tmp = "USB_DR_MODE_UNKNOWN"; break;
		    } tmp; }), type_c->is_role_swap == ROLE_SWAP?"":" not");

	seq_printf(s, "In %s mode %s %s at %s (cc_status=0x%x)\n",
		    type_c->cc_mode?"host":"device",
		    type_c->cc_mode ?
		      (type_c->is_attach?"attach":"detach") :
		      (type_c->is_attach?"connect":"disconnect"),
		    type_c->cc_mode?"device":"host",
		    type_c->at_cc1?"cc1":"cc2", type_c->cc_status);

	seq_printf(s, "Read Register (type_c_ctrl_cc1_0=0x%x)\n",
		    readl(type_c->type_c_reg_base + 0x0));
	seq_printf(s, "Read Register (type_c_ctrl_cc1_1=0x%x)\n",
		    readl(type_c->type_c_reg_base + 0x4));
	seq_printf(s, "Read Register (type_c_ctrl_cc2_0=0x%x)\n",
		    readl(type_c->type_c_reg_base + 0x8));
	seq_printf(s, "Read Register (type_c_ctrl_cc2_1=0x%x)\n",
		    readl(type_c->type_c_reg_base + 0xc));
	seq_printf(s, "Read Register (type_c_status=0x%x)\n",
		    readl(type_c->type_c_reg_base + 0x10));
	seq_printf(s, "Read Register (type_c_ctrl=0x%x)\n",
		    readl(type_c->type_c_reg_base + 0x14));

	spin_unlock_irqrestore(&type_c->lock, flags);

	return 0;
}

static int type_c_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, type_c_status_show, inode->i_private);
}

static const struct file_operations type_c_status_fops = {
	.open			= type_c_status_open,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int type_c_debug_show(struct seq_file *s, void *unused)
{
	struct type_c_data		*type_c = s->private;

	seq_printf(s, "Debug: %s\n",
				type_c->debug?"Enable":"disable");

	return 0;
}

static int type_c_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, type_c_debug_show, inode->i_private);
}

static ssize_t type_c_debug_write(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file		*s = file->private_data;
	struct type_c_data		*type_c = s->private;
	char			buf[32];

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "enable", 6))
		type_c->debug = true;
	else if (!strncmp(buf, "disable", 7))
		type_c->debug = false;

	return count;
}

static const struct file_operations type_c_debug_fops = {
	.open			= type_c_debug_open,
	.write			= type_c_debug_write,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static inline void create_debug_files(struct type_c_data *type_c)
{
	dev_err(type_c->dev, "%s", __func__);

	type_c->debug_dir = debugfs_create_dir("type_c", usb_debug_root);
	if (!type_c->debug_dir) {
		dev_err(type_c->dev, "%s Error debug_dir is NULL", __func__);
		return;
	}

	if (!debugfs_create_file("parameter", 0444, type_c->debug_dir, type_c,
		    &type_c_parameter_fops))
		goto file_error;

	if (!debugfs_create_file("set_parameter", 0644,
		    type_c->debug_dir, type_c,
		    &type_c_set_parameter_fops))
		goto file_error;

	if (!debugfs_create_file("status", 0444, type_c->debug_dir, type_c,
		    &type_c_status_fops))
		goto file_error;

	if (!debugfs_create_file("debug", 0644,
		    type_c->debug_dir, type_c,
		    &type_c_debug_fops))
		goto file_error;

	return;

file_error:
	debugfs_remove_recursive(type_c->debug_dir);
}
#endif //CONFIG_DEBUG_FS

static int __updated_type_c_parameter_by_efuse(struct type_c_data *type_c)
{
	struct nvmem_cell *cell;
	int8_t cc1_4p7k = 0;
	int8_t cc1_12k = 0;
	int8_t cc1_0p2v = 0;
	int8_t cc1_0p8v = 0;
	int8_t cc1_2p6v = 0;
	int8_t cc1_0p66v = 0;
	int8_t cc1_1p23v = 0;
	int8_t cc2_4p7k = 0;
	int8_t cc2_12k = 0;
	int8_t cc2_0p2v = 0;
	int8_t cc2_0p8v = 0;
	int8_t cc2_2p6v = 0;
	int8_t cc2_0p66v = 0;
	int8_t cc2_1p23v = 0;

	cell = nvmem_cell_get(type_c->dev, "usb-cal");
	if (IS_ERR(cell)) {
		dev_warn(type_c->dev, "%s failed to get usb-cal: %ld\n",
			    __func__, PTR_ERR(cell));
	} else {
		unsigned char *buf;
		size_t buf_size;
		int value_size = 4;
		int value_mask = (BIT(value_size) - 1);

		buf = nvmem_cell_read(cell, &buf_size);

#define FIX_VALUE(value) (((int8_t)value&0x8)?(-(int8_t)(0x7&value)):((int8_t)(value)))

		cc1_0p2v = FIX_VALUE((buf[0] >> value_size * 0) & value_mask);
		cc1_0p8v = FIX_VALUE((buf[0] >> value_size * 1) & value_mask);
		cc1_2p6v = FIX_VALUE((buf[1] >> value_size * 0) & value_mask);
		cc1_0p66v = FIX_VALUE((buf[1] >> value_size * 1) & value_mask);
		cc1_1p23v = FIX_VALUE((buf[2] >> value_size * 0) & value_mask);

		cc2_0p2v = FIX_VALUE((buf[3] >> value_size * 0) & value_mask);
		cc2_0p8v = FIX_VALUE((buf[3] >> value_size * 1) & value_mask);
		cc2_2p6v = FIX_VALUE((buf[4] >> value_size * 0) & value_mask);
		cc2_0p66v = FIX_VALUE((buf[4] >> value_size * 1) & value_mask);
		cc2_1p23v = FIX_VALUE((buf[5] >> value_size * 0) & value_mask);

		cc1_4p7k = FIX_VALUE((buf[6] >> value_size * 0) & value_mask);
		cc1_12k = FIX_VALUE((buf[6] >> value_size * 1) & value_mask);
		cc2_4p7k = FIX_VALUE((buf[7] >> value_size * 0) & value_mask);
		cc2_12k = FIX_VALUE((buf[7] >> value_size * 1) & value_mask);

		kfree(buf);
		nvmem_cell_put(cell);
	}

	dev_info(type_c->dev, "check efuse cc1_4p7k=%d cc1_12k=%d cc2_4p7k=%d cc2_12k=%d\n",
		    cc1_4p7k, cc1_12k, cc2_4p7k, cc2_12k);
	dev_info(type_c->dev, "check efuse cc1_0p2v=%d cc1_0p8v=%d cc1_2p6v=%d cc1_0p66v=%d cc1_1p23v=%d\n",
		    cc1_0p2v, cc1_0p8v, cc1_2p6v, cc1_0p66v, cc1_1p23v);
	dev_info(type_c->dev, "check efuse cc2_0p2v=%d cc2_0p8v=%d cc2_2p6v=%d cc2_0p66v=%d cc2_1p23v=%d\n",
		    cc2_0p2v, cc2_0p8v, cc2_2p6v, cc2_0p66v, cc2_1p23v);

	type_c->cc1_rp_code = rp4pk_code(code_rp4pk(type_c->cc1_rp_code) + cc1_4p7k) |
		    rp36k_code(code_rp36k(type_c->cc1_rp_code)) |
		    rp12k_code(code_rp12k(type_c->cc1_rp_code) + cc1_12k);
	type_c->cc2_rp_code = rp4pk_code(code_rp4pk(type_c->cc2_rp_code) + cc2_4p7k) |
		    rp36k_code(code_rp36k(type_c->cc2_rp_code)) |
		    rp12k_code(code_rp12k(type_c->cc2_rp_code) + cc2_12k);
	dev_dbg(type_c->dev, "%s: cc1_rp_code=0x%x, cc1_rd_code=0x%x\n",
		    __func__, type_c->cc1_rp_code, type_c->cc1_rd_code);
	dev_dbg(type_c->dev, "%s: cc2_rp_code=0x%x, cc2_rd_code=0x%x\n",
		    __func__, type_c->cc2_rp_code, type_c->cc2_rd_code);

	type_c->cc1_vref_ufp =
		    vref_1p23v(decode_1p23v(type_c->cc1_vref_ufp) + cc1_1p23v) |
		    vref_0p66v(decode_0p66v(type_c->cc1_vref_ufp) + cc1_0p66v) |
		    vref_0p2v(decode_0p2v(type_c->cc1_vref_ufp) + cc1_0p2v);
	type_c->cc1_vref_dfp_usb =
		    vref_0_1p6v(decode_0_1p6v(type_c->cc1_vref_dfp_usb)) |
		    vref_0p2v(decode_0p2v(type_c->cc1_vref_dfp_usb) + cc1_0p2v);
	type_c->cc1_vref_dfp_1_5 =
		    vref_1_1p6v(decode_1_1p6v(type_c->cc1_vref_dfp_1_5)) |
		    vref_0p4v(decode_0p4v(type_c->cc1_vref_dfp_1_5)) |
		    vref_0p2v(decode_0p2v(type_c->cc1_vref_dfp_1_5) + cc1_0p2v);
	type_c->cc1_vref_dfp_3_0 =
		    vref_2p6v(decode_2p6v(type_c->cc1_vref_dfp_3_0) + cc1_2p6v) |
		    vref_0p8v(decode_0p8v(type_c->cc1_vref_dfp_3_0) + cc1_0p8v) |
		    vref_0p2v(decode_0p2v(type_c->cc1_vref_dfp_3_0) + cc1_0p2v);
	dev_dbg(type_c->dev, "%s: cc1_vref_ufp=0x%x, cc1_vref_dfp_usb=0x%x"
		    " cc1_vref_dfp_1_5=0x%x, cc1_vref_dfp_3_0=0x%x\n",
		    __func__, type_c->cc1_vref_ufp,
		    type_c->cc1_vref_dfp_usb, type_c->cc1_vref_dfp_1_5,
		    type_c->cc1_vref_dfp_3_0);

	type_c->cc2_vref_ufp =
		    vref_1p23v(decode_1p23v(type_c->cc2_vref_ufp) + cc2_1p23v) |
		    vref_0p66v(decode_0p66v(type_c->cc2_vref_ufp) + cc2_0p66v) |
		    vref_0p2v(decode_0p2v(type_c->cc2_vref_ufp) + cc2_0p2v);
	type_c->cc2_vref_dfp_usb =
		    vref_0_1p6v(decode_0_1p6v(type_c->cc2_vref_dfp_usb)) |
		    vref_0p2v(decode_0p2v(type_c->cc2_vref_dfp_usb) + cc2_0p2v);
	type_c->cc2_vref_dfp_1_5 =
		    vref_1_1p6v(decode_1_1p6v(type_c->cc2_vref_dfp_1_5)) |
		    vref_0p4v(decode_0p4v(type_c->cc2_vref_dfp_1_5)) |
		    vref_0p2v(decode_0p2v(type_c->cc2_vref_dfp_1_5) + cc2_0p2v);
	type_c->cc2_vref_dfp_3_0 =
		    vref_2p6v(decode_2p6v(type_c->cc2_vref_dfp_3_0) + cc2_2p6v) |
		    vref_0p8v(decode_0p8v(type_c->cc2_vref_dfp_3_0) + cc2_0p8v) |
		    vref_0p2v(decode_0p2v(type_c->cc2_vref_dfp_3_0) + cc2_0p2v);
	dev_dbg(type_c->dev, "%s: cc2_vref_ufp=0x%x, cc2_vref_dfp_usb=0x%x"
		    " cc1_vref_dfp_1_5=0x%x, cc1_vref_dfp_3_0=0x%x\n",
		    __func__, type_c->cc2_vref_ufp,
		    type_c->cc2_vref_dfp_usb, type_c->cc2_vref_dfp_1_5,
		    type_c->cc2_vref_dfp_3_0);

	return 0;
}

static void get_default_type_c_parameter(struct type_c_data *type_c,
	    struct device *dev)
{
	void *reg;
	int val;

	if (soc_device_match(rtk_soc_typc_parameter_v0))
		type_c->para_ver = 0;
	else
		type_c->para_ver = 1;

	type_c->debounce_val = 0x7f;/* 1b,1us 7f,4.7us */
	type_c->cc_dfp_mode = CC_MODE_DFP_3_0;
	type_c->cc1_rp = En_rp4p7k;
	type_c->cc2_rp = En_rp4p7k;

	reg = type_c->type_c_reg_base + USB_TYPEC_CTRL_CC1_0;
	val = readl(reg);
	type_c->cc1_rp_code = rp4pk_code(code_rp4pk(val)) |
		    rp36k_code(code_rp36k(val)) |
		    rp12k_code(code_rp12k(val));
	type_c->cc1_rd_code = rd_code(code_rd(val));
	dev_dbg(dev, "%s: cc1_0=0x%x, cc1_rp_code=0x%x, cc1_rd_code=0x%x\n",
		    __func__, val, type_c->cc1_rp_code, type_c->cc1_rd_code);

	if (val & PLR_EN)
		type_c->rd_en_at_first = true;
	else
		type_c->rd_en_at_first = false;

	/* Add a workaround to disable rd_en for stark */
	if (!soc_device_match(rtk_soc_typc_parameter_v0))
		type_c->rd_en_at_first = false;

	reg = type_c->type_c_reg_base + USB_TYPEC_CTRL_CC2_0;
	val = readl(reg);
	type_c->cc2_rp_code = rp4pk_code(code_rp4pk(val)) |
		    rp36k_code(code_rp36k(val)) |
		    rp12k_code(code_rp12k(val));
	type_c->cc2_rd_code = rd_code(code_rd(val));
	dev_dbg(dev, "%s: cc2_0=0x%x, cc2_rp_code=0x%x, cc2_rd_code=0x%x\n",
		    __func__, val, type_c->cc2_rp_code, type_c->cc2_rd_code);

	reg = type_c->type_c_reg_base + USB_TYPEC_CTRL_CC1_1;
	val = readl(reg);
	type_c->cc1_vref_ufp = vref_1p23v(decode_1p23v(val)) |
		    vref_0p66v(decode_0p66v(val)) |
		    vref_0p2v(decode_0p2v(val));
	type_c->cc1_vref_dfp_usb = vref_0_1p6v(decode_0_1p6v(val)) |
		    vref_0p2v(decode_0p2v(val));
	type_c->cc1_vref_dfp_1_5 = vref_1_1p6v(decode_1_1p6v(val)) |
		     vref_0p4v(decode_0p4v(val)) |
		     vref_0p2v(decode_0p2v(val));
	type_c->cc1_vref_dfp_3_0 = vref_2p6v(decode_2p6v(val)) |
		    vref_0p8v(decode_0p8v(val)) | vref_0p2v(decode_0p2v(val));
	dev_dbg(dev, "%s: cc1_1=0x%x, cc1_vref_ufp=0x%x, cc1_vref_dfp_usb=0x%x"
		    " cc1_vref_dfp_1_5=0x%x, cc1_vref_dfp_3_0=0x%x\n",
		    __func__, val, type_c->cc1_vref_ufp,
		    type_c->cc1_vref_dfp_usb, type_c->cc1_vref_dfp_1_5,
		    type_c->cc1_vref_dfp_3_0);

	reg = type_c->type_c_reg_base + USB_TYPEC_CTRL_CC2_1;
	val = readl(reg);
	type_c->cc2_vref_ufp = vref_1p23v(decode_1p23v(val)) |
		    vref_0p66v(decode_0p66v(val)) |
		    vref_0p2v(decode_0p2v(val));
	type_c->cc2_vref_dfp_usb = vref_0_1p6v(decode_0_1p6v(val)) |
		    vref_0p2v(decode_0p2v(val));
	type_c->cc2_vref_dfp_1_5 = vref_1_1p6v(decode_1_1p6v(val)) |
		     vref_0p4v(decode_0p4v(val)) |
		     vref_0p2v(decode_0p2v(val));
	type_c->cc2_vref_dfp_3_0 = vref_2p6v(decode_2p6v(val)) |
		    vref_0p8v(decode_0p8v(val)) | vref_0p2v(decode_0p2v(val));
	dev_dbg(dev, "%s: cc2_1=0x%x, cc2_vref_ufp=0x%x, cc2_vref_dfp_usb=0x%x"
		    " cc1_vref_dfp_1_5=0x%x, cc1_vref_dfp_3_0=0x%x\n",
		    __func__, val, type_c->cc2_vref_ufp,
		    type_c->cc2_vref_dfp_usb, type_c->cc2_vref_dfp_1_5,
		    type_c->cc2_vref_dfp_3_0);
}

static int parse_type_c_parameter(struct type_c_data *type_c, struct device *dev)
{
	struct device_node *node = dev->of_node;
	const char *str;
	u32 val, default_val;
	struct device_node *sub_node;
	u8 array_vals[3];
	u32 u32_vals[3];
	u32 default_revision;
	char revision[4] = {0};
	int ret;

	if (!node) {
		dev_err(dev, "%s: No device node!\n", __func__);
		return -ENODEV;
	} else if (!of_device_is_available(node)) {
		dev_err(dev, "%s: device node is unavailable!\n", __func__);
		return -ENODEV;
	}

	if (of_property_read_bool(node, "use_defalut_parameter")) {
		type_c->use_defalut_parameter = true;
	} else {
		type_c->use_defalut_parameter = false;
	}

	get_default_type_c_parameter(type_c, dev);

	if (type_c->use_defalut_parameter) {
		dev_info(dev, "%s: Use default type c parameter\n", __func__);
		return 0;
	}

	ret = of_property_read_u32(node, "default_revision",
		    &default_revision);
	if (ret) {
		default_revision = 0xA00;
		dev_info(dev, "%s: No set default_revision (use %x)\n",
			    __func__, default_revision);
	}

	if (!type_c->chip_revision ||
		    type_c->chip_revision > default_revision)
		snprintf(revision, 4, "%X", default_revision);
	else
		snprintf(revision, 4, "%X", type_c->chip_revision);
	dev_info(dev, "Chip revision is %x (support revision %x) to load %s parameter\n",
		    type_c->chip_revision,
		    default_revision, revision);
	sub_node = of_get_child_by_name(node, revision);

	ret = of_property_read_string(sub_node, "cc_dfp_mode", &str);
	if (ret) {
		dev_err(dev, "%s: cc_dfp_mode error(%d)\n",
			    __func__, ret);
	}
	if (!strcmp(str, "dfp_usb")) {
		type_c->cc_dfp_mode = CC_MODE_DFP_USB;
		type_c->cc1_rp = En_rp36k;
		type_c->cc2_rp = En_rp36k;
	} else if (!strcmp(str, "dfp_1_5")) {
		type_c->cc_dfp_mode = CC_MODE_DFP_1_5;
		type_c->cc1_rp = En_rp12k;
		type_c->cc2_rp = En_rp12k;
	} else if (!strcmp(str, "dfp_3_0")) {
		type_c->cc_dfp_mode = CC_MODE_DFP_3_0;
		type_c->cc1_rp = En_rp4p7k;
		type_c->cc2_rp = En_rp4p7k;
	} else {
		dev_err(dev, "%s: unknown cc_dfp_mode %s\n",
			    __func__, str);
	}

	//cc1 parameters
	ret = of_property_read_u32(sub_node, "cc1_rp_4p7k_code",
		    &u32_vals[0]);
	if (ret) {
		dev_err(dev, "%s: cc1_rp_4p7k_code error(%d)\n",
			    __func__, u32_vals[0]);
	}
	ret = of_property_read_u32(sub_node, "cc1_rp_36k_code",
		    &u32_vals[1]);
	if (ret) {
		dev_err(dev, "%s: cc1_rp_36k_code error(%d)\n",
			    __func__,
			    u32_vals[1]);
	}
	ret = of_property_read_u32(sub_node, "cc1_rp_12k_code",
		    &u32_vals[2]);
	if (ret) {
		dev_err(dev, "%s: cc1_rp_12k_code error(%d)\n",
			    __func__, u32_vals[2]);
	}
	default_val = type_c->cc1_rp_code;
	type_c->cc1_rp_code = rp4pk_code(u32_vals[0])
			| rp36k_code(u32_vals[1])
			| rp12k_code(u32_vals[2]);
	if (default_val ^ type_c->cc1_rp_code)
		dev_dbg(dev, "Set cc1_rp_code 0x%x --> 0x%x\n",
			    default_val, type_c->cc1_rp_code);

	ret = of_property_read_u32(sub_node, "cc1_rd_code", &val);
	if (ret) {
		dev_err(dev, "%s: cc1_rd_code error(%d)\n",
			    __func__, ret);
	}
	default_val = type_c->cc1_rd_code;
	type_c->cc1_rd_code = rd_code(val);
	if (default_val ^ type_c->cc1_rd_code)
		dev_dbg(dev, "Set cc1_rd_code 0x%x --> 0x%x\n",
			    default_val, type_c->cc1_rd_code);

	ret = of_property_read_u8_array(sub_node, "cc1_vref_ufp",
		    array_vals, 3);
	if (ret) {
		dev_err(dev, "%s: cc1_vref_ufp error(%d)\n",
			    __func__, ret);
	}
	default_val = type_c->cc1_vref_ufp;
	type_c->cc1_vref_ufp = vref_1p23v(array_vals[0]) |
		    vref_0p66v(array_vals[1]) |
		    vref_0p2v(array_vals[2]);
	if (default_val ^ type_c->cc1_vref_ufp)
		dev_dbg(dev, "Set cc1_vref_ufp 0x%x --> 0x%x\n",
			    default_val, type_c->cc1_vref_ufp);

	ret = of_property_read_u8_array(sub_node, "cc1_vref_dfp_usb",
		    array_vals, 3);
	if (ret) {
		dev_err(dev, "%s: cc1_vref_dfp_usb error(%d)\n",
			    __func__, ret);
	}
	default_val = type_c->cc1_vref_dfp_usb;
	type_c->cc1_vref_dfp_usb = vref_0_1p6v(array_vals[0]) |
		    vref_0p2v(array_vals[1]);
	if (default_val ^ type_c->cc1_vref_dfp_usb)
		dev_dbg(dev, "Set cc1_vref_dfp_usb 0x%x --> 0x%x\n",
			    default_val, type_c->cc1_vref_dfp_usb);

	ret = of_property_read_u8_array(sub_node, "cc1_vref_dfp_1_5",
		    array_vals, 3);
	if (ret) {
		dev_err(dev, "%s: cc1_vref_dfp_1_5 error(%d)\n",
			    __func__, ret);
	}
	default_val = type_c->cc1_vref_dfp_1_5;
	type_c->cc1_vref_dfp_1_5 = vref_1_1p6v(array_vals[0]) |
		     vref_0p4v(array_vals[1]) |
		     vref_0p2v(array_vals[2]);
	if (default_val ^ type_c->cc1_vref_dfp_1_5)
		dev_dbg(dev, "Set cc1_vref_dfp_1_5 0x%x --> 0x%x\n",
			    default_val, type_c->cc1_vref_dfp_1_5);

	ret = of_property_read_u8_array(sub_node, "cc1_vref_dfp_3_0",
		    array_vals, 3);
	if (ret) {
		dev_err(dev, "%s: cc1_vref_dfp_3_0 error(%d)\n",
			    __func__, ret);
	}
	default_val = type_c->cc1_vref_dfp_3_0;
	type_c->cc1_vref_dfp_3_0 = vref_2p6v(array_vals[0]) |
		    vref_0p8v(array_vals[1]) | vref_0p2v(array_vals[2]);
	if (default_val ^ type_c->cc1_vref_dfp_3_0)
		dev_dbg(dev, "Set cc1_vref_dfp_3_0 0x%x --> 0x%x\n",
			    default_val, type_c->cc1_vref_dfp_3_0);

	//cc2 parameters
	ret = of_property_read_u32(sub_node, "cc2_rp_4p7k_code",
		    &u32_vals[0]);
	if (ret) {
		dev_err(dev, "%s: cc2_rp_4p7k_code error(%d)\n",
			    __func__, ret);
	}
	ret = of_property_read_u32(sub_node, "cc2_rp_36k_code",
		    &u32_vals[1]);
	if (ret) {
		dev_err(dev, "%s: cc2_rp_36k_code error(%d)\n",
			    __func__, ret);
	}
	ret = of_property_read_u32(sub_node, "cc2_rp_12k_code",
		    &u32_vals[2]);
	if (ret) {
		dev_err(dev, "%s: cc2_rp_12k_code error(%d)\n",
			    __func__, ret);
	}
	default_val = type_c->cc2_rp_code;
	type_c->cc2_rp_code = rp4pk_code(u32_vals[0])
			| rp36k_code(u32_vals[1])
			| rp12k_code(u32_vals[2]);
	if (default_val ^ type_c->cc2_rp_code)
		dev_dbg(dev, "Set cc2_rp_code 0x%x --> 0x%x\n",
			    default_val, type_c->cc2_rp_code);

	ret = of_property_read_u32(sub_node, "cc2_rd_code", &val);
	if (ret) {
		dev_err(dev, "%s: cc2_rd_code error(%d)\n",
			    __func__, ret);
	}
	default_val = type_c->cc2_rd_code;
	type_c->cc2_rd_code = rd_code(val);
	if (default_val ^ type_c->cc2_rd_code)
		dev_dbg(dev, "Set cc2_rd_code 0x%x --> 0x%x\n",
			    default_val, type_c->cc2_rd_code);

	ret = of_property_read_u8_array(sub_node, "cc2_vref_ufp",
		    array_vals, 3);
	if (ret) {
		dev_err(dev, "%s: cc2_vref_ufp error(%d)\n",
			    __func__, ret);
	}
	default_val = type_c->cc2_vref_ufp;
	type_c->cc2_vref_ufp = vref_1p23v(array_vals[0]) |
		    vref_0p66v(array_vals[1]) |
		    vref_0p2v(array_vals[2]);
	if (default_val ^ type_c->cc2_vref_ufp)
		dev_dbg(dev, "Set cc2_vref_ufp 0x%x --> 0x%x\n",
			    default_val, type_c->cc2_vref_ufp);

	ret = of_property_read_u8_array(sub_node, "cc2_vref_dfp_usb",
		    array_vals, 3);
	if (ret) {
		dev_err(dev, "%s: cc2_vref_dfp_usb error(%d)\n",
			    __func__, ret);
	}
	default_val = type_c->cc2_vref_dfp_usb;
	type_c->cc2_vref_dfp_usb = vref_0_1p6v(array_vals[0]) |
		    vref_0p2v(array_vals[1]);
	if (default_val ^ type_c->cc2_vref_dfp_usb)
		dev_dbg(dev, "Set cc2_vref_dfp_usb 0x%x --> 0x%x\n",
			    default_val, type_c->cc2_vref_dfp_usb);

	ret = of_property_read_u8_array(sub_node, "cc2_vref_dfp_1_5",
		    array_vals, 3);
	if (ret) {
		dev_err(dev, "%s: cc2_vref_dfp_1_5 error(%d)\n",
			    __func__, ret);
	}
	default_val = type_c->cc2_vref_dfp_1_5;
	type_c->cc2_vref_dfp_1_5 = vref_1_1p6v(array_vals[0]) |
		    vref_0p4v(array_vals[1]) | vref_0p2v(array_vals[2]);
	if (default_val ^ type_c->cc2_vref_dfp_1_5)
		dev_dbg(dev, "Set cc2_vref_dfp_1_5 0x%x --> 0x%x\n",
			    default_val, type_c->cc2_vref_dfp_1_5);

	ret = of_property_read_u8_array(sub_node, "cc2_vref_dfp_3_0",
		    array_vals, 3);
	if (ret) {
		dev_err(dev, "%s: cc2_vref_dfp_3_0 error(%d)\n",
			    __func__, ret);
	}
	default_val = type_c->cc2_vref_dfp_3_0;
	type_c->cc2_vref_dfp_3_0 = vref_2p6v(array_vals[0]) |
		    vref_0p8v(array_vals[1]) | vref_0p2v(array_vals[2]);
	if (default_val ^ type_c->cc2_vref_dfp_3_0)
		dev_dbg(dev, "Set cc2_vref_dfp_3_0 0x%x --> 0x%x\n",
			    default_val, type_c->cc2_vref_dfp_3_0);

	type_c->debounce_val = 0x7f;/* 1b,1us 7f,4.7us */

	__updated_type_c_parameter_by_efuse(type_c);

	return 0;
}

#ifdef CONFIG_DUAL_ROLE_USB_INTF
static enum dual_role_property fusb_drp_properties[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
};

static int dual_role_get_local_prop(struct dual_role_phy_instance *drp,
				enum dual_role_property prop,
				unsigned int *val)
{
	struct type_c_data *type_c = dual_role_get_drvdata(drp);

	if (!type_c) {
		pr_err("driver data not ready\n");
		return -1;
	}

	if (type_c->dr_mode == USB_DR_MODE_PERIPHERAL) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_UFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SNK;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_DEVICE;
		else
			return -EINVAL;
	} else if (type_c->dr_mode == USB_DR_MODE_HOST) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_DFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SRC;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_HOST;
		else
			return -EINVAL;
	} else {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_NONE;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_NONE;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_NONE;
		else
			return -EINVAL;
	}

	return 0;
}

static int dual_role_is_writeable(struct dual_role_phy_instance *drp,
				enum dual_role_property prop)
{
	if (prop == DUAL_ROLE_PROP_MODE)
		return 1;
	else
		return 0;
}

#if 0
static int dual_role_set_mode_prop(struct dual_role_phy_instance *drp,
				enum dual_role_property prop,
				const unsigned int *val)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int dual_role_set_prop(struct dual_role_phy_instance *drp,
				enum dual_role_property prop,
				const unsigned int *val)
{
	if (prop == DUAL_ROLE_PROP_MODE)
		return dual_role_set_mode_prop(drp, prop, val);
	else
		return -EINVAL;
}
#endif
#endif

#ifdef CONFIG_TYPEC
static int type_c_port_dr_set(struct typec_port *port,
	    enum typec_data_role role)
{
	struct type_c_data *type_c = typec_get_drvdata(port);
	int ret = 0;
	u32 enable_cc;
	unsigned long		flags;

	spin_lock_irqsave(&type_c->lock, flags);
	enable_cc = type_c->at_cc1?enable_cc1:enable_cc2;
	spin_unlock_irqrestore(&type_c->lock, flags);

	if (role == TYPEC_HOST) {
		switch_type_c_dr_mode(type_c, USB_DR_MODE_HOST, enable_cc);
	} else if (role == TYPEC_DEVICE) {
		switch_type_c_dr_mode(type_c, USB_DR_MODE_PERIPHERAL,
			    enable_cc);
	} else {
		switch_type_c_dr_mode(type_c, 0, disable_cc);
	}

	return ret < 0 ? ret : 0;
}

static const struct typec_operations type_c_port_ops = {
	.dr_set = type_c_port_dr_set,
};
#endif /* CONFIG_TYPEC */

#define DEFAULT_CHIP_REVISION 0xA00
#define MAX_CHIP_REVISION 0xC00

static int __get_chip_revision(void)
{
	int chip_revision = 0xFFF;
	char revision[] = "FFF";
	struct soc_device_attribute soc_att[] = {{.revision = revision}, {}};
	struct soc_device_attribute *soc_att_match = NULL;

	while (soc_att_match == NULL) {
		chip_revision--;

		if (chip_revision <= DEFAULT_CHIP_REVISION)
			break;
		if (chip_revision > MAX_CHIP_REVISION)
			chip_revision = MAX_CHIP_REVISION;
		else if ((chip_revision & 0xFF) > 0xF)
			chip_revision = (chip_revision & 0xF00) + 0xF;

		snprintf(revision, 4, "%X", chip_revision);

		soc_att_match = (struct soc_device_attribute *)
			    soc_device_match(soc_att);
	}

	if (soc_att_match) {
		pr_debug("%s get chip_revision %x\n", __func__, chip_revision);
		return chip_revision;
	}

	pr_debug("%s usb DEFAULT chip_revision %X\n", __func__,
		    DEFAULT_CHIP_REVISION);
	return DEFAULT_CHIP_REVISION;
}

/* Init and probe */
static int dwc3_rtk_type_c_init(struct type_c_data *type_c)
{
	struct device		*dev = type_c->dev;
	u32 debounce_val = type_c->debounce_val;// 1b,1us 7f,4.7us
	unsigned long		flags;
#ifdef CONFIG_DUAL_ROLE_USB_INTF
	struct dual_role_phy_desc *desc;
	struct dual_role_phy_instance *dual_role;
#endif

	enable_writel(debounce_val<<1,
		    type_c->type_c_reg_base + USB_TYPEC_CTRL);
	dev_info(dev, "%s set debounce = 0x%x (check--> 0x%x)\n",
		    __func__, debounce_val,
		    readl(type_c->type_c_reg_base + USB_TYPEC_CTRL));

	if ((type_c->rd_ctrl_gpio != -1) &&
		    gpio_request(type_c->rd_ctrl_gpio, dev->of_node->name))
		dev_err(dev, "%s ERROR Request rd_ctrl_gpio  (id=%d) fail\n",
			   __func__, type_c->rd_ctrl_gpio);

	rtk_type_c_init(type_c);

	switch_type_c_dr_mode(type_c, 0, disable_cc);

	spin_lock_irqsave(&type_c->lock, flags);

	dev_info(dev, "First check USB_DR_MODE_PERIPHERAL");
	type_c->cc_mode = IN_DEVICE_MODE;
	type_c->is_attach = IN_DETACH;
	type_c->connect_change = CONNECT_NO_CHANGE;
	if (type_c->boot_check_time < 0)
		type_c->check_at_boot = false;
	else
		type_c->check_at_boot = true;
	dev_info(dev, "First time device mode check is %s",
			type_c->check_at_boot?"Enable":"Disable");

	detect_host(type_c);

	spin_unlock_irqrestore(&type_c->lock, flags);

	schedule_delayed_work(&type_c->delayed_work,
		    msecs_to_jiffies(0));

#ifdef CONFIG_DUAL_ROLE_USB_INTF
	if (type_c->drp == NULL) {
		desc = devm_kzalloc(dev, sizeof(struct dual_role_phy_desc),
			    GFP_KERNEL);
		if (!desc)
			return 0;

		desc->name = "dwc3_otg";
		desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
		desc->get_property = dual_role_get_local_prop;
//		desc->set_property = dual_role_set_prop;
		desc->properties = fusb_drp_properties;
		desc->num_properties = ARRAY_SIZE(fusb_drp_properties);
		desc->property_is_writeable = dual_role_is_writeable;
		dual_role = devm_dual_role_instance_register(dev, desc);
		dual_role->drv_data = type_c;
		type_c->drp = dual_role;
	}
#endif

#ifdef CONFIG_TYPEC
	if (type_c->port == NULL) {
		struct typec_capability typec_cap = { };

		typec_cap.revision = USB_TYPEC_REV_1_0;
		typec_cap.pd_revision = 0x200;
		typec_cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;
		typec_cap.driver_data = type_c;
		typec_cap.ops = &type_c_port_ops;

		typec_cap.type = TYPEC_PORT_DRP;
		typec_cap.data = TYPEC_PORT_DRD;

		type_c->port = typec_register_port(type_c->dev, &typec_cap);
		if (IS_ERR(type_c->port))
			return PTR_ERR(type_c->port);
	}
#endif /* CONFIG_TYPEC */
	return 0;
}

static void dwc3_rtk_type_c_probe_work(struct work_struct *work)
{
	struct type_c_data *type_c = container_of(work,
		    struct type_c_data, start_work);
	struct device		*dev = type_c->dev;
	int    ret = 0;

	unsigned long probe_time = jiffies;

	dev_info(dev, "%s Start ...\n", __func__);

	ret = dwc3_rtk_type_c_init(type_c);

	if (ret)
		dev_err(dev, "%s failed to init type_c\n", __func__);

	dev_info(dev, "%s End ... ok! (take %d ms)\n", __func__,
		    jiffies_to_msecs(jiffies - probe_time));
}

static int dwc3_rtk_type_c_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct device_node	*node = dev->of_node;
	struct type_c_data	*type_c;
	unsigned int gpio;
	int ret = 0;
	unsigned long probe_time = jiffies;

	dev_info(dev, "ENTER %s", __func__);
	type_c = devm_kzalloc(dev, sizeof(*type_c), GFP_KERNEL);
	if (!type_c)
		return -ENOMEM;

	type_c->type_c_reg_base = of_iomap(pdev->dev.of_node, 0);
	if (type_c->type_c_reg_base == NULL) {
		dev_err(&pdev->dev, "error mapping memory for reg_base\n");
		ret = -EFAULT;
		goto err1;
	}

	type_c->dev = dev;

	type_c->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (type_c->irq <= 0) {
		dev_err(&pdev->dev,
			    "Type C driver with no IRQ. Check %s setup!\n",
			    dev_name(&pdev->dev));
		ret = -ENODEV;
		goto err1;
	}

	ret = request_irq(type_c->irq, type_c_detect_irq,
			IRQF_SHARED, "type_c_detect", type_c);

	spin_lock_init(&type_c->lock);
	type_c->chip_revision = __get_chip_revision();

	type_c->rd_ctrl_gpio = -1;
	if (node && of_device_is_available(node) &&
		    soc_device_match(rtk_soc_kylin)) {
		gpio = of_get_named_gpio(node, "realtek,rd_ctrl-gpio", 0);
		if (gpio_is_valid(gpio)) {
			type_c->rd_ctrl_gpio = gpio;
			dev_info(dev, "%s get rd_ctrl-gpio (id=%d) OK\n",
				    __func__, gpio);
		} else {
			dev_err(dev, "Error rd_ctrl-gpio no found");
		}
	}

	if (node && of_device_is_available(node)) {
		ret = of_property_read_u32(node, "boot_check_time",
			&type_c->boot_check_time);
		if (ret)
			type_c->boot_check_time = -1;

		dev_info(dev,
			    "Set device mode boot_check_time %d ms ==> (%s to check)\n",
			    type_c->boot_check_time,
			    type_c->boot_check_time < 0?"Disable":"Enable");

		if (of_property_read_bool(node, "filter_config_channel_signal"))
			type_c->filter_config_channel_signal = true;
		else
			type_c->filter_config_channel_signal = false;
		dev_info(dev, "Set filter_config_channel_signal is %s\n",
			    type_c->filter_config_channel_signal ?
			      "True":"False");

		if (of_property_read_bool(node, "force_cc_state_at_device_mode"))
			type_c->force_cc_state_at_device_mode = true;
		else
			type_c->force_cc_state_at_device_mode = false;
		dev_info(dev, "Set force_cc_state_at_device_mode is %s\n",
			    type_c->filter_config_channel_signal?
			      "True":"False");
	}

	if (parse_type_c_parameter(type_c, dev)) {
		dev_err(dev, "ERROR: %s to parse type c parameter!!", __func__);
		ret = -EINVAL;
		goto err1;
	}

	if (node) {
		struct device_node	*parent_node;

		parent_node = of_parse_phandle(dev->of_node, "dwc3_rtk", 0);
		if (!parent_node)
			parent_node = of_get_parent(node);
		if (parent_node) {
			type_c->dwc3_rtk = platform_get_drvdata(
				    of_find_device_by_node(parent_node));
		} else {
			dev_err(dev, "%s No find dwc3_rtk", __func__);
			ret = -ENODEV;
			goto err1;
		}
	}

	type_c->is_attach = IN_DETACH;
	type_c->is_role_swap = NO_ROLE_SWAP;

	if (of_property_read_bool(node, "debug")) {
		dev_info(&pdev->dev, "%s device tree set debug flag\n",
			    __func__);
		type_c->debug = true;
	} else {
		type_c->debug = false;
	}

	INIT_DELAYED_WORK(&type_c->delayed_work, host_device_switch);
	INIT_DELAYED_WORK(&type_c->boot_check_work, boot_time_check);

	if (node) {
		if (of_property_read_bool(node, "delay_probe_work")) {
			INIT_WORK(&type_c->start_work,
				    dwc3_rtk_type_c_probe_work);

			if (of_property_read_bool(node, "ordered_probe"))
				rtk_usb_manager_schedule_work(dev,
					    &type_c->start_work);
			else
				schedule_work(&type_c->start_work);
		} else {
			ret = dwc3_rtk_type_c_init(type_c);
			if (ret) {
				dev_err(dev, "%s failed to init type_c\n",
					    __func__);
				goto err1;
			}
		}
	} else {
		dev_err(dev, "no device node, failed to init type_c\n");
		ret = -ENODEV;
		goto err1;
	}

	platform_set_drvdata(pdev, type_c);

	device_create_file(type_c->dev, &dev_attr_role_swap);
	device_create_file(type_c->dev, &dev_attr_force_set_mode);

#ifdef CONFIG_DEBUG_FS
	create_debug_files(type_c);
#endif

	dev_info(&pdev->dev, "Exit %s OK (take %d ms)\n", __func__,
		    jiffies_to_msecs(jiffies - probe_time));
	return 0;

err1:
	dev_err(&pdev->dev, "%s: Probe fail, %d\n", __func__, ret);

	return ret;
}

static int dwc3_rtk_type_c_remove(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct type_c_data *type_c = dev_get_drvdata(dev);
	u32 default_ctrl;
	unsigned long		flags;

	dev_info(dev, "[USB] Enter %s", __func__);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(type_c->debug_dir);
#endif

#ifdef CONFIG_TYPEC
	if (type_c->port) {
		typec_unregister_port(type_c->port);
		type_c->port = NULL;
	}
#endif

	device_remove_file(type_c->dev, &dev_attr_role_swap);
	device_remove_file(type_c->dev, &dev_attr_force_set_mode);

	cancel_delayed_work_sync(&type_c->delayed_work);
	flush_delayed_work(&type_c->delayed_work);
	BUG_ON(delayed_work_pending(&type_c->delayed_work));

	cancel_delayed_work_sync(&type_c->boot_check_work);
	flush_delayed_work(&type_c->boot_check_work);
	BUG_ON(delayed_work_pending(&type_c->boot_check_work));

	spin_lock_irqsave(&type_c->lock, flags);
	/* disable interrupt */
	default_ctrl = readl(type_c->type_c_reg_base + USB_TYPEC_CTRL) &
		    debounce_time_MASK;
	writel(default_ctrl, type_c->type_c_reg_base + USB_TYPEC_CTRL);

	spin_unlock_irqrestore(&type_c->lock, flags);

	free_irq(type_c->irq, type_c);

	dev_info(&pdev->dev, "[USB] Exit %s\n", __func__);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rtk_dwc3_type_c_match[] = {
	{ .compatible = "realtek,dwc3-type_c" },
	{},
};
MODULE_DEVICE_TABLE(of, rtk_dwc3_type_c_match);
#endif

#ifdef CONFIG_PM_SLEEP
static int dwc3_rtk_type_c_prepare(struct device *dev)
{
	struct type_c_data *type_c = dev_get_drvdata(dev);
	u32 default_ctrl;
	unsigned long		flags;
	int ret = 0;

	dev_info(dev, "[USB] Enter %s\n", __func__);

	if (!type_c) {
		dev_err(dev, "[USB] %s type_c_data is NULL\n", __func__);
		goto out;
	}

	cancel_delayed_work_sync(&type_c->delayed_work);
	flush_delayed_work(&type_c->delayed_work);
	BUG_ON(delayed_work_pending(&type_c->delayed_work));

	cancel_delayed_work_sync(&type_c->boot_check_work);
	flush_delayed_work(&type_c->boot_check_work);
	BUG_ON(delayed_work_pending(&type_c->boot_check_work));

	spin_lock_irqsave(&type_c->lock, flags);
	/* disable interrupt */
	default_ctrl = readl(type_c->type_c_reg_base + USB_TYPEC_CTRL) &
		    debounce_time_MASK;
	writel(default_ctrl, type_c->type_c_reg_base + USB_TYPEC_CTRL);

	spin_unlock_irqrestore(&type_c->lock, flags);

out:
	dev_info(dev, "[USB] Exit %s\n", __func__);
	return ret;
}

static void dwc3_rtk_type_c_complete(struct device *dev)
{
	struct type_c_data *type_c = dev_get_drvdata(dev);

	dev_info(dev, "[USB] Enter %s\n", __func__);

	if (!type_c) {
		dev_err(dev, "[USB] %s type_c_data is NULL\n", __func__);
		goto out;
	}

out:
	dev_info(dev, "[USB] Exit %s\n", __func__);
}

static int dwc3_rtk_type_c_suspend(struct device *dev)
{
	struct type_c_data *type_c = dev_get_drvdata(dev);

	dev_info(dev, "[USB] Enter %s", __func__);

	if (!type_c) {
		dev_err(dev, "[USB] %s type_c_data is NULL\n", __func__);
		goto out;
	}

	if (type_c->rd_ctrl_gpio != -1)
		gpio_free(type_c->rd_ctrl_gpio);

out:
	dev_info(dev, "[USB] Exit %s\n", __func__);
	return 0;
}

static int dwc3_rtk_type_c_resume(struct device *dev)
{
	struct type_c_data *type_c = dev_get_drvdata(dev);
	unsigned long		flags;
	bool reinit = false;

	dev_info(dev, "[USB] Enter %s", __func__);

	if (!type_c) {
		dev_err(dev, "[USB] %s type_c_data is NULL\n", __func__);
		goto out;
	}

	if (rtk_usb_manager_is_iso_mode(type_c->dev)) {
		spin_lock_irqsave(&type_c->lock, flags);
		//enable interrupt
		if (type_c->is_attach == IN_ATTACH)
			enable_writel(ENABLE_TYPE_C_DETECT,
			    type_c->type_c_reg_base + USB_TYPEC_CTRL);
		else
			reinit = true;
		spin_unlock_irqrestore(&type_c->lock, flags);
	} else {
		dev_info(dev, "[USB] %s reinit dwc3_rtk_type_c_init (Not iso mode\n)",
			    __func__);

		if (type_c->is_attach == IN_ATTACH) {
			u32 enable_cc;

			rtk_type_c_init(type_c);
			enable_writel(ENABLE_TYPE_C_DETECT,
			    type_c->type_c_reg_base + USB_TYPEC_CTRL);
			enable_cc = type_c->at_cc1?enable_cc1:enable_cc2;
			switch_type_c_plug_config(type_c,
				    type_c->dr_mode, enable_cc);
		} else {
			reinit = true;
		}
	}

	if (reinit)
		dwc3_rtk_type_c_init(type_c);

out:
	dev_info(dev, "[USB] Exit %s\n", __func__);
	return 0;
}

static const struct dev_pm_ops dwc3_rtk_type_c_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(
	    dwc3_rtk_type_c_suspend, dwc3_rtk_type_c_resume)
	.prepare = dwc3_rtk_type_c_prepare,
	.complete = dwc3_rtk_type_c_complete,
};

#define DEV_PM_OPS	(&dwc3_rtk_type_c_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver dwc3_rtk_type_c_driver = {
	.probe		= dwc3_rtk_type_c_probe,
	.remove		= dwc3_rtk_type_c_remove,
	.driver		= {
		.name	= "rtk-dwc3-type_c",
		.of_match_table = of_match_ptr(rtk_dwc3_type_c_match),
		.pm = DEV_PM_OPS,
	},
};

static int __init dwc3_rtk_type_c_driver_init(void)
{
	return platform_driver_register(&(dwc3_rtk_type_c_driver));
}
module_init(dwc3_rtk_type_c_driver_init);

static void __exit dwc3_rtk_type_c_driver_exit(void)
{
	platform_driver_unregister(&(dwc3_rtk_type_c_driver));
}
module_exit(dwc3_rtk_type_c_driver_exit);

MODULE_ALIAS("platform:rtk-dwc3-type_c");
MODULE_LICENSE("GPL");

