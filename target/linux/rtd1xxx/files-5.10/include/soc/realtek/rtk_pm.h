/* SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause) */
/*
 * Realtek DHC SoC family power management driver
 *
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 */

#ifndef _RTK_PM_H_
#define _RTK_PM_H_

#include <linux/suspend.h>
#include <soc/realtek/rtk_ipc_shm.h>
#include <soc/realtek/rtk_ir.h>
#include <soc/realtek/rtk_cec.h>

#define GPIO_MAX_SIZE 86
#define MEM_VERIFIED_CNT 100

#define OSC_COUNT_LIMIT 0x800
#define PLL_ETN_OSC 0x7a4
#define DCO0 0x79c
#define DCO1 0x0f4
#define DCO_ENABLE 0x1000

enum rtk_pm_driver_id {
	PM = 0,
	LAN,
	IRDA,
	GPIO,
	ALARM_TIMER,
	TIMER,
	CEC,
	USB,
};

enum rtk_cpu_type {
	SCPU = 0x1,
	ACPU,
	VCPU,
};

/* A/V CPU power control */
enum rtk_cpu_ctrl {
	ACPU_OFF = 10,
	VCPU_OFF,
	ACPU_ON,
	VCPU_ON,
};

enum rtk_wakeup_event {
	LAN_EVENT = 0,
	IR_EVENT,
	GPIO_EVENT,
	ALARM_EVENT,
	TIMER_EVENT,
	CEC_EVENT,
	USB_EVENT,
	HIFI_EVENT,
	VTC_EVENT,
	PON_EVENT,
	MAX_EVENT,
};

struct pm_dev_param {
	struct device    *dev;
	struct list_head list;
	unsigned int     dev_type;
	void             *data;
};

struct pm_pcpu_param {
	unsigned int        wakeup_source;
	unsigned int        timerout_val;
	char                wu_gpio_en[GPIO_MAX_SIZE];
	char                wu_gpio_act[GPIO_MAX_SIZE];
	struct ipc_shm_irda irda_info;
	struct ipc_shm_cec  cec_info;
} __packed;

struct pm_private {
	struct device         *dev;
	struct list_head      list;
	struct notifier_block pm_notifier;
	struct pm_dev_param   *device_param;
	struct pm_pcpu_param  *pcpu_param;
	dma_addr_t            pcpu_param_pa;
	struct regmap         *syscon_iso;
	unsigned int          pm_dbg;
	unsigned int          reboot_reasons;
	unsigned int          wakeup_reason;
	unsigned int          suspend_context;
};

struct mem_check {
	unsigned char *mem_addr;
	size_t         mem_byte;
};

unsigned int notify_fw_on(struct device *dev, uint32_t cpu);
unsigned int notify_fw_off(struct device *dev, uint32_t cpu);

extern void rtk_pm_init_list(void);
extern void rtk_pm_add_list(struct pm_dev_param *pm_node);
extern struct pm_dev_param *rtk_pm_get_param(unsigned int id);
extern int rtk_pm_create_sysfs(void);
extern void rtk_pm_set_pcpu_param(struct device *dev);
extern void rtk_pm_del_list(struct pm_dev_param *pm_node);

#endif
