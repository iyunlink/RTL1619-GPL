/* SPDX-License-Identifier: GPL-2.0 */
/**
 * dwc3-rtk.h - Realtek DWC3 Specific Glue layer
 *
 * Copyright (C) 2020 Realtek Semiconductor Corporation
 *
 */

#ifndef __DRIVERS_USB_DWC3_RTK_H
#define __DRIVERS_USB_DWC3_RTK_H

#define WRAP_CTR_reg  0x0
#define DISABLE_MULTI_REQ BIT(1)
#define DESC_R2W_MULTI_DISABLE BIT(9)
#define FORCE_PIPE3_PHY_STATUS_TO_0 BIT(13)

#define WRAP_USB2_PHY_UTMI_reg 0x8
#define TXHSVM_EN BIT(3)

#define WRAP_PHY_PIPE_reg 0xC
#define RESET_DISABLE_PIPE3_P0 BIT(0)
#define CLOCk_ENABLE_FOR_PIPE3_PCLK BIT(1)

#define WRAP_USB_HMAC_CTR0_reg 0x60
#define U3PORT_DIS BIT(8)

#define WRAP_USB2_PHY_reg  0x70
#define USB2_PHY_EN_PHY_PLL_PORT0 BIT(12)
#define USB2_PHY_EN_PHY_PLL_PORT1 BIT(13)
#define USB2_PHY_SWITCH_MASK 0x707
#define USB2_PHY_SWITCH_DEVICE 0x0
#define USB2_PHY_SWITCH_HOST 0x606

#define WRAP_APHY_reg 0x128
#define USB3_MBIAS_ENABLE BIT(1)

#define WRAP_USB_DBUS_PWR_CTRL_reg 0x160
#define DBUS_PWR_CTRL_EN BIT(0)

struct dwc3_rtk {
	struct device		*dev;

	void __iomem		*regs;
	size_t		regs_size;

	struct dwc3 *dwc;

	struct work_struct work;

	int default_dwc3_dr_mode; /* define by dwc3 driver, and it is fixed */
	int cur_dr_mode; /* current dr mode */
	bool support_drd_mode; /* if support Host/device switch */
	struct usb_role_switch	*role_switch;

	bool dis_u3_port;
	bool disable_usb3;

	bool enable_l4icg;

	bool desc_r2w_multi_disable;

	/* For debugfs */
	struct dentry		*debug_dir;

	void *dwc3_regs_dump;
	void *wrap_regs_dump;
};

void dwc3_rtk_debugfs_init(struct dwc3_rtk *dwc3_rtk);
void dwc3_rtk_debugfs_exit(struct dwc3_rtk *dwc3_rtk);

#endif /* __DRIVERS_USB_DWC3_RTK_H */
