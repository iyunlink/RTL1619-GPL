/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2014 Google, Inc.
 * Author: Badhri Jagan Sridharan <badhri@android.com>
 *
 */

extern struct usb_function_instance *alloc_inst_mtp_ptp(bool mtp_config);
extern struct usb_function *function_alloc_mtp_ptp(
			struct usb_function_instance *fi, bool mtp_config);
