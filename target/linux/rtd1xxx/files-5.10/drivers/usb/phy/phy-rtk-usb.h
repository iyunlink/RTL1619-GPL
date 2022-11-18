/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  phy-rtk-usb.h RTK usb phy header file
 *
 * copyright (c) 2017 realtek semiconductor corporation
 *
 */

#ifndef __PHY_RTK_USB_H__
#define __PHY_RTK_USB_H__

struct rtk_usb_phy_s {
	struct usb_phy phy;
	struct device *dev;

	int port_index;
	int phyN;
	void *reg_addr;
	void *phy_data;

#ifdef CONFIG_DEBUG_FS
	struct dentry		*debug_dir;
#endif
};

struct rtk_usb_phy_data_s {
	char addr;
	char data;
};

#define phy_read(addr)			__raw_readl(addr)
#define phy_write(addr, val)	do { \
	/* Do smp_wmb */ \
	smp_wmb(); __raw_writel(val, addr); \
} while (0)
#define PHY_IO_TIMEOUT_MSEC		(50)

static inline int utmi_wait_register(void __iomem *reg, u32 mask, u32 result)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(PHY_IO_TIMEOUT_MSEC);

	while (time_before(jiffies, timeout)) {
		/* Do smp_rmb */
		smp_rmb();
		if ((phy_read(reg) & mask) == result)
			return 0;
		udelay(100);
	}
	pr_err("\033[0;32;31m can't program USB phy \033[m\n");
	return -ETIMEDOUT;
}

#endif /* __PHY_RTK_USB_H__ */
