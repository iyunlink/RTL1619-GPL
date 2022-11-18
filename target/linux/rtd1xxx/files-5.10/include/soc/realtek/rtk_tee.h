/* SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause) */
/*
 * Realtek DHC SoC family TEE driver
 *
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 */

#ifndef _RTK_TEE_H_
#define _RTK_TEE_H_

#include <linux/tee_drv.h>

struct tee_mem_api_module {
	struct tee_context *tee_context;
	unsigned int tee_session;
	struct mutex protected_lock;
	struct list_head protected_list;
	struct list_head protected_ext_list;
	struct notifier_block protected_notifier;
	bool bReady;
};

extern struct tee_mem_api_module *tee_mem_protected_get_gmodule(void);

#endif
