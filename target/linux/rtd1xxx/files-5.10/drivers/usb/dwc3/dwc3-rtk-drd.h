/* SPDX-License-Identifier: GPL-2.0 */
/**
 * dwc3-rtk-drd.h - Realtek DWC3 Specific Glue layer
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 */

#ifndef __DRIVERS_USB_DWC3_RTK_DRD_H
#define __DRIVERS_USB_DWC3_RTK_DRD_H

struct dwc3;

int dwc3_drd_to_host(struct dwc3 *dwc);
int dwc3_drd_to_device(struct dwc3 *dwc);
int dwc3_drd_to_stop_all(struct dwc3 *dwc);

struct dwc3_rtk;

int dwc3_rtk_get_dr_mode(struct dwc3_rtk *rtk);
int dwc3_rtk_set_dr_mode(struct dwc3_rtk *rtk, int dr_mode);
bool dwc3_rtk_is_connected_on_device_mode(struct dwc3_rtk *dwc3_rtk);
bool dwc3_rtk_is_support_drd_mode(struct dwc3_rtk *dwc3_rtk);

#endif /* __DRIVERS_USB_DWC3_RTK_CORE_H */

