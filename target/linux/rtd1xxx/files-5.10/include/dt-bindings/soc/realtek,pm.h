/* SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause) */
/*
 * Realtek DHC SoC family power management driver
 *
 * Copyright (c) 2021 Realtek Semiconductor Corp.
 */

 #ifndef _DT_BINDINGS_REALTEK_PM_H
#define _DT_BINDINGS_REALTEK_PM_H

/* wakeup source */
#define LAN 0x01
#define IR 0x02
#define GPIO 0x04
#define RTC 0x08
#define TIMER 0x10
#define CEC 0x20
#define USB 0x40

/* wakeup mode */
#define HIFI 0x80
#define VTC 0x100
#define PON 0x200

#define NORMAL_MODE (LAN | IR | GPIO | RTC | TIMER | CEC)
#define HIFI_MODE (HIFI | LAN | IR | GPIO | RTC | TIMER | CEC)
#define VTC_MODE (VTC | LAN | IR | GPIO | RTC | TIMER | CEC)
#define PON_MODE (PON | LAN | IR | GPIO | RTC | TIMER | CEC)

/* GPIO mode */
#define GPIO_WAKEUP_ENABLE 1
#define GPIO_WAKEUP_DISABLE 0
#define GPIO_WAKEUP_ACTIVE_LOW 0
#define GPIO_WAKEUP_ACTIVE_HIGH 1

/* DCO mode */
#define DCO_ENABLE 1
#define DCO_DISABLE 0

#endif /* _DT_BINDINGS_REALTEK_PM_H */
