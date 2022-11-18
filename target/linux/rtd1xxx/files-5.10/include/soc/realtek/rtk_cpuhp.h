/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef __REALTEK_CPUHP_H
#define __REALTEK_CPUHP_H

#include <linux/pm_qos.h>

struct rtk_cpuhp_qos_request {
	struct plist_node pnode;
	struct pm_qos_constraints *qos;
};

static inline int rtk_cpuhp_qos_request_active(struct rtk_cpuhp_qos_request *req)
{
        return !IS_ERR_OR_NULL(req->qos);
}

s32 rtk_cpuhp_qos_read_value(void);

int rtk_cpuhp_qos_add_request(struct rtk_cpuhp_qos_request *req, s32 value);
int rtk_cpuhp_qos_update_request(struct rtk_cpuhp_qos_request *req, s32 new_value);
int rtk_cpuhp_qos_remove_request(struct rtk_cpuhp_qos_request *req);

int rtk_cpuhp_qos_add_notifier(struct notifier_block *notifier);
int rtk_cpuhp_qos_remove_notifier(struct notifier_block *notifier);

#endif
