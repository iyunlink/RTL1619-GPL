/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irqchip.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/memblock.h>
#include <linux/delay.h>
#include <linux/clockchips.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>
#include <asm/system_info.h>
#include <asm/psci.h>

static void __init rtd13xx_init_time(void)
{
	of_clk_init(NULL);
	timer_probe();
	tick_setup_hrtimer_broadcast();
}

static void __init rtd13xx_init_machine(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static const char *const rtd13xx_dt_compat[] __initconst = {
	"realtek,rtd1319",
	NULL
};

DT_MACHINE_START(RTD1319, "Hank")
	.dt_compat =	rtd13xx_dt_compat,
	.init_machine =	rtd13xx_init_machine,
	.init_time = 	rtd13xx_init_time,
	.smp =		smp_ops(psci_smp_ops),
MACHINE_END
