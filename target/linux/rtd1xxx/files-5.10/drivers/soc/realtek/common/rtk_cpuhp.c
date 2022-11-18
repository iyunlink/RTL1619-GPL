// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Realtek Semiconductor Corp.
 */

#include <linux/module.h>
#include <linux/pm_qos.h>
#include <soc/realtek/rtk_cpuhp.h>

#define RTK_CPUHP_DEFAULT_VALUE 0

static BLOCKING_NOTIFIER_HEAD(rtk_cpuhp_notifiers);
static struct pm_qos_constraints rtk_cpuhp_constraints = {
	.list                = PLIST_HEAD_INIT(rtk_cpuhp_constraints.list),
	.target_value        = RTK_CPUHP_DEFAULT_VALUE,
	.default_value       = RTK_CPUHP_DEFAULT_VALUE,
	.no_constraint_value = RTK_CPUHP_DEFAULT_VALUE,
	.type                = PM_QOS_MAX,
	.notifiers           = &rtk_cpuhp_notifiers,
};

s32 rtk_cpuhp_qos_read_value(void)
{
	return pm_qos_read_value(&rtk_cpuhp_constraints);
}
EXPORT_SYMBOL_GPL(rtk_cpuhp_qos_read_value);

int rtk_cpuhp_qos_add_request(struct rtk_cpuhp_qos_request *req, s32 value)
{
	int ret;

	if (!req)
		return -EINVAL;

	if (WARN(rtk_cpuhp_qos_request_active(req),
		"%s() called for active request\n", __func__))
		return -EINVAL;

	req->qos = &rtk_cpuhp_constraints;
	ret = pm_qos_update_target(req->qos, &req->pnode, PM_QOS_ADD_REQ, value);
	if (ret < 0)
		req->qos = NULL;
	return ret;
}
EXPORT_SYMBOL_GPL(rtk_cpuhp_qos_add_request);

int rtk_cpuhp_qos_update_request(struct rtk_cpuhp_qos_request *req, s32 new_value)
{
	if (!req)
		return -EINVAL;

	if (WARN(!rtk_cpuhp_qos_request_active(req),
		 "%s() called for unknown object\n", __func__))
		return -EINVAL;

	if (req->pnode.prio == new_value)
		return 0;

	return pm_qos_update_target(req->qos, &req->pnode, PM_QOS_UPDATE_REQ, new_value);
}
EXPORT_SYMBOL_GPL(rtk_cpuhp_qos_update_request);

int rtk_cpuhp_qos_remove_request(struct rtk_cpuhp_qos_request *req)
{
	int ret;

	if (!req)
		return -EINVAL;

	if (WARN(!rtk_cpuhp_qos_request_active(req),
		 "%s() called for unknown object\n", __func__))
		return -EINVAL;

	ret = pm_qos_update_target(req->qos, &req->pnode, PM_QOS_REMOVE_REQ, PM_QOS_DEFAULT_VALUE);
	req->qos = NULL;
	return ret;
}
EXPORT_SYMBOL_GPL(rtk_cpuhp_qos_remove_request);

int rtk_cpuhp_qos_add_notifier(struct notifier_block *notifier)
{
	return blocking_notifier_chain_register(rtk_cpuhp_constraints.notifiers,
		notifier);
}
EXPORT_SYMBOL_GPL(rtk_cpuhp_qos_add_notifier);

int rtk_cpuhp_qos_remove_notifier(struct notifier_block *notifier)
{
	return blocking_notifier_chain_unregister(rtk_cpuhp_constraints.notifiers,
                notifier);
}
EXPORT_SYMBOL_GPL(rtk_cpuhp_qos_remove_notifier);

MODULE_DESCRIPTION("Realtek CPUHP QoS");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
