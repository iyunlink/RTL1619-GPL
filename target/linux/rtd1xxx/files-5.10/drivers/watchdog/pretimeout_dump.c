/*
 * Copyright (C) 2020 Realtek Semiconductor Corp.
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/watchdog.h>
#include <linux/sysrq.h>
#include <linux/reboot.h>

#include "watchdog_pretimeout.h"

static void pretimeout_dump(struct watchdog_device *wdd)
{
	if (wdd->ops->stop)
		wdd->ops->stop(wdd);
	dev_err(wdd->parent, "%s: start dump\n", __func__);

	handle_sysrq('9');
	dump_stack();
	handle_sysrq('w');
	handle_sysrq('l');
	/* dump memory stats */
	handle_sysrq('m');
	handle_sysrq('p');
	handle_sysrq('q');
	/* dump scheduler state */
	handle_sysrq('t');
	handle_sysrq('z');

	machine_restart("watchdog");
}

static struct watchdog_governor watchdog_gov_dump = {
	.name		= "dump",
	.pretimeout	= pretimeout_dump,
};

static int __init watchdog_gov_dump_register(void)
{
	return watchdog_register_governor(&watchdog_gov_dump);
}

static void __exit watchdog_gov_dump_unregister(void)
{
	watchdog_unregister_governor(&watchdog_gov_dump);
}
module_init(watchdog_gov_dump_register);
module_exit(watchdog_gov_dump_unregister);

MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_DESCRIPTION("Realtek dump watchdog pretimeout governor");
MODULE_LICENSE("GPL");
