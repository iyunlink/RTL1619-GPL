// SPDX-License-Identifier: GPL-2.0
/**
 * dwc3-rtk-drd.c - Realtek DWC3 Specific Glue layer
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 */
#include <linux/module.h>

#include "core.h"
#include "gadget.h"
#include "io.h"

static int dwc3_check_drd_mode(struct dwc3 *dwc)
{
	int mode = USB_DR_MODE_UNKNOWN;

	if (dwc->xhci) {
		mode = USB_DR_MODE_HOST;
		dev_dbg(dwc->dev, "%s Now is host\n", __func__);
	} else if (dwc->gadget && dwc->gadget->udc) {
		mode = USB_DR_MODE_PERIPHERAL;
		dev_dbg(dwc->dev, "%s Now is gadget\n", __func__);
	}

	return mode;
}

static int rtk_dwc3_drd_core_soft_reset(struct dwc3 *dwc)
{
	int ret;
	unsigned long timeout;
	u32 reg_gctl, reg_u2phy, reg_u3phy;

	reg_gctl = dwc3_readl(dwc->regs, DWC3_GCTL);
	dwc3_writel(dwc->regs, DWC3_GCTL, reg_gctl | DWC3_GCTL_DSBLCLKGTNG);

	reg_u3phy = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0),
		    reg_u3phy & ~DWC3_GUSB3PIPECTL_SUSPHY);

	reg_u2phy = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0),
		    reg_u2phy & ~DWC3_GUSB2PHYCFG_SUSPHY);

	/* issue device SoftReset too */
	timeout = jiffies + msecs_to_jiffies(500);
	dwc3_writel(dwc->regs, DWC3_DCTL, DWC3_DCTL_CSFTRST);
	do {
		u32 reg;

		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		if (!(reg & DWC3_DCTL_CSFTRST))
			break;

		if (time_after(jiffies, timeout)) {
			dev_err(dwc->dev, "Reset Timed Out\n");
			ret = -ETIMEDOUT;
			goto err0;
		}

		cpu_relax();
	} while (true);

	ret = dwc3_core_soft_reset(dwc);

err0:

	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg_u2phy);
	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg_u3phy);
	dwc3_writel(dwc->regs, DWC3_GCTL, reg_gctl);

	return ret;
}

static int rtk_dwc3_drd_event_buffers_setup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;

	evt = dwc->ev_buf;
	evt->lpos = 0;
	dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(0),
			lower_32_bits(evt->dma));
	dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(0),
			upper_32_bits(evt->dma));
	dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(0),
			DWC3_GEVNTSIZ_SIZE(evt->length));
	dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(0), 0);

	return 0;
}

static void rtk_dwc3_set_mode(struct dwc3 *dwc, u32 mode)
{
	dwc3_set_prtcap(dwc, mode);
}

int dwc3_drd_to_host(struct dwc3 *dwc)
{
	int ret;

	dev_info(dwc->dev, "%s START....", __func__);
	if (dwc3_check_drd_mode(dwc) == USB_DR_MODE_PERIPHERAL) {
		dwc3_gadget_exit(dwc);
	}
	/* Do wmb */
	wmb();

	if (dwc3_check_drd_mode(dwc) == USB_DR_MODE_HOST) {
		dev_info(dwc->dev, "%s Now is host", __func__);
		return 0;
	}

	dev_info(dwc->dev, "%s: call dwc3_core_soft_reset\n", __func__);
	ret = rtk_dwc3_drd_core_soft_reset(dwc);
	if (ret) {
		dev_err(dwc->dev, "soft reset failed\n");
		goto err0;
	}

	ret = rtk_dwc3_drd_event_buffers_setup(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to setup event buffers\n");
		goto err0;
	}

	rtk_dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);

	ret = dwc3_host_init(dwc);
	if (ret)
		dev_err(dwc->dev, "failed to init host\n");

err0:
	dev_info(dwc->dev, "%s END....", __func__);
	return ret;
}
EXPORT_SYMBOL(dwc3_drd_to_host);

int dwc3_drd_to_device(struct dwc3 *dwc)
{
	int ret;
	unsigned long flags = 0;

	dev_info(dwc->dev, "%s START....", __func__);

	if (dwc3_check_drd_mode(dwc) == USB_DR_MODE_HOST) {
		dev_info(dwc->dev, "%s dwc3_host_exit", __func__);
		dwc3_host_exit(dwc);
	}
	/* Do wmb */
	wmb();

	if (dwc3_check_drd_mode(dwc) == USB_DR_MODE_PERIPHERAL) {
		dev_info(dwc->dev, "%s Now is gadget", __func__);
		return 0;
	}

	dev_info(dwc->dev, "%s: call dwc3_core_soft_reset\n", __func__);
	ret = rtk_dwc3_drd_core_soft_reset(dwc);
	if (ret) {
		dev_err(dwc->dev, "soft reset failed\n");
		goto err0;
	}

	spin_lock_irqsave(&dwc->lock, flags);

	ret = rtk_dwc3_drd_event_buffers_setup(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to setup event buffers\n");
		spin_unlock_irqrestore(&dwc->lock, flags);
		goto err0;
	}

	rtk_dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);

	spin_unlock_irqrestore(&dwc->lock, flags);

	ret = dwc3_gadget_init(dwc);
	if (ret)
		dev_err(dwc->dev, "failed to init gadget\n");

err0:
	dev_info(dwc->dev, "%s END....", __func__);
	return ret;
}
EXPORT_SYMBOL(dwc3_drd_to_device);

int dwc3_drd_to_stop_all(struct dwc3 *dwc)
{
	int ret = 0;

	dev_info(dwc->dev, "%s START....", __func__);
	if (dwc3_check_drd_mode(dwc) == USB_DR_MODE_HOST)
		dwc3_host_exit(dwc);
	if (dwc3_check_drd_mode(dwc) == USB_DR_MODE_PERIPHERAL)
		dwc3_gadget_exit(dwc);

	/* Do wmb */
	wmb();
	dev_info(dwc->dev, "%s END....", __func__);
	return ret;
}
EXPORT_SYMBOL(dwc3_drd_to_stop_all);

MODULE_LICENSE("GPL");
