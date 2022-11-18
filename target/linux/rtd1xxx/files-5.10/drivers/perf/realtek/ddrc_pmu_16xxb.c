// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek uncore PMU driver for 16xxb DDRC
 *
 * Copyright (C) 2021-2024 Realtek Semiconductor Corporation
 * Copyright (C) 2021-2024 Ping-Hsiung Chiu <phelic@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#define pr_fmt(fmt)	"[RTK_PMU] " fmt

#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include "rtk_uncore_pmu.h"
#include "rtk_ddrc_pmu.h"


#define _DDRC_CTRL_OFFSET_		0x0300


/* Target ID of DDRC */
#define RT_ID		0x01		/* channel 0, Real-time channel*/
#define NRT_ID		0x02		/* channel 1, Non-real-time channel */
#define NPU_ID		0x03		/* channel 2, NPU */
#define CH3_ID		0x04		/* channel 3, VE2, VCPU */
#define GPU_ID		0x05		/* channel 4, GPU */
#define SCPU_ID		0x06		/* channel 5, SCPU */
#define VCPU_ID		0x0a
#define VE2_ID		0x0b


/* DDRC events */
DDRC_EVENT_GROUP(rt,	RT_ID);
DDRC_EVENT_GROUP(nrt,	NRT_ID);
DDRC_EVENT_GROUP(npu,	NPU_ID);
DDRC_EVENT_GROUP(ch3,	CH3_ID);
DDRC_EVENT_GROUP(gpu,	GPU_ID);
DDRC_EVENT_GROUP(scpu,	SCPU_ID);
DDRC_EVENT_GROUP(vcpu,	VCPU_ID);
DDRC_EVENT_GROUP(ve2,	VE2_ID);

/* Channel total events */
DDRC_TOTAL_EVENT_ATTR(cycles,	DDRC_PMC__TOTAL_CYCLE);
DDRC_TOTAL_EVENT_ATTR(wack,	DDRC_PMC__TOTAL_WACK);
DDRC_TOTAL_EVENT_ATTR(rack,	DDRC_PMC__TOTAL_RACK);

/*
 * DDRC log events
 */
/* SCPU part */
#define STREAM_TRANSIENT		60
#define DATA_PREFETCH			59
#define MEM_TABLE_WALK			58
#define MEM_INST_FETCH			57
#define MEM_READ_DATA			56
#define DEMAND				55
#define PREFETCH			54
#define MEM_READ			53

/* DMC part */
#define HIF_RD_OR_WR			52
#define HIF_WR				51
#define HIF_RD				50
#define HIF_HI_PRI_RD			48
#define DFI_WR_DATA_CYCLES		45
#define DFI_RD_DATA_CYCLES		44
#define HPR_XACT_WHEN_CRITICAL		43
#define WR_XACT_WHEN_CRITICAL		41
#define OP_IS_ACTIVATE			40
#define OP_IS_RD_OR_WR			39
#define OP_IS_RD_ACTIVATE		38
#define OP_IS_RD			37
#define OP_IS_WR			36
#define OP_IS_PRECHARGE			34
#define PRECHARGE_FOR_RDWR		33
#define PRECHARGE_FOR_OTHER		32
#define RDWR_TRANSITIONS		31
#define WAR_HAZARD			29
#define RAW_HAZARD			28
#define WAW_HAZARD			27
#define OP_IS_ENTER_SELFREF		26
#define OP_IS_ENTER_POWERDOWN		25
#define OP_IS_REFRESH			21

DDRC_LOG_CPU_EVENT_ATTR_GROUP(stream_of_transient,	STREAM_TRANSIENT, 4);
DDRC_LOG_CPU_EVENT_ATTR_GROUP(data_prefetch,		DATA_PREFETCH,	4);
DDRC_LOG_CPU_EVENT_ATTR_GROUP(table_walk,		MEM_TABLE_WALK, 4);
DDRC_LOG_CPU_EVENT_ATTR_GROUP(inst_fetch,		MEM_INST_FETCH,	4);
DDRC_LOG_CPU_EVENT_ATTR_GROUP(memory_read_data,		MEM_READ_DATA,	4);
DDRC_LOG_CPU_EVENT_ATTR_GROUP(demand,			DEMAND,		4);
DDRC_LOG_CPU_EVENT_ATTR_GROUP(prefetch,			PREFETCH,	4);
DDRC_LOG_CPU_EVENT_ATTR_GROUP(memory_read,		MEM_READ,	4);

DDRC_LOG_EVENT_ATTR_GROUP(hif_access,		HIF_RD_OR_WR, 2);
DDRC_LOG_EVENT_ATTR_GROUP(hif_write,		HIF_WR, 2);
DDRC_LOG_EVENT_ATTR_GROUP(hif_read,		HIF_RD, 2);
DDRC_LOG_EVENT_ATTR_GROUP(hif_high_prio_read,	HIF_HI_PRI_RD, 2);
DDRC_LOG_EVENT_ATTR_GROUP(dfi_write_data,	DFI_WR_DATA_CYCLES, 2);
DDRC_LOG_EVENT_ATTR_GROUP(dfi_read_data,	DFI_RD_DATA_CYCLES, 2);
DDRC_LOG_EVENT_ATTR_GROUP(hpr_xact_when_critical, HPR_XACT_WHEN_CRITICAL, 2);
DDRC_LOG_EVENT_ATTR_GROUP(write_xact_when_critical, WR_XACT_WHEN_CRITICAL, 2);
DDRC_LOG_EVENT_ATTR_GROUP(op_activate,		OP_IS_ACTIVATE, 2);
DDRC_LOG_EVENT_ATTR_GROUP(op_access,		OP_IS_RD_OR_WR, 2);
DDRC_LOG_EVENT_ATTR_GROUP(op_read_activate,	OP_IS_RD_ACTIVATE, 2);
DDRC_LOG_EVENT_ATTR_GROUP(op_read,		OP_IS_RD, 2);
DDRC_LOG_EVENT_ATTR_GROUP(op_write,		OP_IS_WR, 2);
DDRC_LOG_EVENT_ATTR_GROUP(op_precharge,		OP_IS_PRECHARGE, 2);
DDRC_LOG_EVENT_ATTR_GROUP(precharge_for_access,	PRECHARGE_FOR_RDWR, 2);
DDRC_LOG_EVENT_ATTR_GROUP(precharge_for_other,	PRECHARGE_FOR_OTHER, 2);
DDRC_LOG_EVENT_ATTR_GROUP(access_transitions,	RDWR_TRANSITIONS, 2);
DDRC_LOG_EVENT_ATTR_GROUP(war_hazard,		WAR_HAZARD, 2);
DDRC_LOG_EVENT_ATTR_GROUP(raw_hazard,		RAW_HAZARD, 2);
DDRC_LOG_EVENT_ATTR_GROUP(waw_hazard,		WAW_HAZARD, 2);
DDRC_LOG_EVENT_ATTR_GROUP(op_enter_selfref,	OP_IS_ENTER_SELFREF, 2);
DDRC_LOG_EVENT_ATTR_GROUP(op_enter_powerdown,	OP_IS_ENTER_POWERDOWN, 2);
DDRC_LOG_EVENT_ATTR_GROUP(op_refresh,		OP_IS_REFRESH, 2);

/* Driver statistics events */
DDRC_DRV_EVENT_ATTR(ddrc_overflow,	RTK_DRV_OVERFLOW);
DDRC_DRV_EVENT_ATTR(ddrc_refresh,	DDRC_REFRESH);

static struct attribute *rtk_16xxb_ddrc_event_attrs[] = {
	DDRC_DRV_EVENT_REF(ddrc_refresh),
	DDRC_DRV_EVENT_REF(ddrc_overflow),

	DDRC_TOTAL_EVENT_REF(cycles),
	DDRC_TOTAL_EVENT_REF(wack),
	DDRC_TOTAL_EVENT_REF(rack),

	DDRC_EVENT_REF_GROUP(rt),
	DDRC_EVENT_REF_GROUP(nrt),
	DDRC_EVENT_REF_GROUP(npu),
	DDRC_EVENT_REF_GROUP(ch3),
	DDRC_EVENT_REF_GROUP(gpu),
	DDRC_EVENT_REF_GROUP(scpu),
	DDRC_EVENT_REF_GROUP(vcpu),
	DDRC_EVENT_REF_GROUP(ve2),

	DDRC_LOG_CPU_EVENT_REF_GROUP(stream_of_transient,	4),
	DDRC_LOG_CPU_EVENT_REF_GROUP(data_prefetch,		4),
	DDRC_LOG_CPU_EVENT_REF_GROUP(table_walk,		4),
	DDRC_LOG_CPU_EVENT_REF_GROUP(inst_fetch,		4),
	DDRC_LOG_CPU_EVENT_REF_GROUP(memory_read_data,		4),
	DDRC_LOG_CPU_EVENT_REF_GROUP(demand,			4),
	DDRC_LOG_CPU_EVENT_REF_GROUP(prefetch,			4),
	DDRC_LOG_CPU_EVENT_REF_GROUP(memory_read,		4),

	DDRC_LOG_EVENT_REF_GROUP(hif_access,			2),
	DDRC_LOG_EVENT_REF_GROUP(hif_write,			2),
	DDRC_LOG_EVENT_REF_GROUP(hif_read,			2),
	DDRC_LOG_EVENT_REF_GROUP(hif_high_prio_read,		2),
	DDRC_LOG_EVENT_REF_GROUP(dfi_write_data,		2),
	DDRC_LOG_EVENT_REF_GROUP(dfi_read_data,			2),
	DDRC_LOG_EVENT_REF_GROUP(hpr_xact_when_critical,	2),
	DDRC_LOG_EVENT_REF_GROUP(write_xact_when_critical,	2),
	DDRC_LOG_EVENT_REF_GROUP(op_activate,			2),
	DDRC_LOG_EVENT_REF_GROUP(op_access,			2),
	DDRC_LOG_EVENT_REF_GROUP(op_read_activate,		2),
	DDRC_LOG_EVENT_REF_GROUP(op_read,			2),
	DDRC_LOG_EVENT_REF_GROUP(op_write,			2),
	DDRC_LOG_EVENT_REF_GROUP(op_precharge,			2),
	DDRC_LOG_EVENT_REF_GROUP(precharge_for_access,		2),
	DDRC_LOG_EVENT_REF_GROUP(precharge_for_other,		2),
	DDRC_LOG_EVENT_REF_GROUP(access_transitions,		2),
	DDRC_LOG_EVENT_REF_GROUP(war_hazard,			2),
	DDRC_LOG_EVENT_REF_GROUP(raw_hazard,			2),
	DDRC_LOG_EVENT_REF_GROUP(waw_hazard,			2),
	DDRC_LOG_EVENT_REF_GROUP(op_enter_selfref,		2),
	DDRC_LOG_EVENT_REF_GROUP(op_enter_powerdown,		2),
	DDRC_LOG_EVENT_REF_GROUP(op_refresh,			2),

	NULL
};

static struct attribute_group rtk_16xxb_ddrc_event_attr_group = {
	.name = "events",
	.attrs = rtk_16xxb_ddrc_event_attrs,
};

static const struct attribute_group *rtk_16xxb_ddrc_attr_groups[] = {
	/* must be Null-terminated */
	[RTK_PMU_ATTR_GROUP__COMMON] = &rtk_pmu_common_attr_group,
	[RTK_PMU_ATTR_GROUP__FORMAT] = &rtk_ddrc_format_attr_group,
	[RTK_PMU_ATTR_GROUP__EVENT] = &rtk_16xxb_ddrc_event_attr_group,
	[RTK_PMU_ATTR_GROUP__NUM] = NULL
};

static const unsigned int rtk_16xxb_ddrc_clients[] = {
	RT_ID, NRT_ID, NPU_ID, CH3_ID, GPU_ID, SCPU_ID, VCPU_ID, VE2_ID
};

static const unsigned long rtk_16xxb_ddrc_pmcgs[] = {
	0x0304, 0x031C, 0x0334, 0x034C, 0x0364, 0x037C,
	/* pending counters */
	0x03B4
};

static const unsigned long rtk_16xxb_ddrc_configs[] = {
	0x03B0
};

static const unsigned int rtk_16xxb_ddrc_total_ev[] = {
	DDRC_PMC__TOTAL_CYCLE, DDRC_PMC__TOTAL_WACK, DDRC_PMC__TOTAL_RACK
};

static const unsigned long rtk_16xxb_ddrc_total_pmcgs[] = {
	0x0394, 0x0398, 0x039C
};

static const unsigned int rtk_16xxb_ddrc_log_ev[] = {
	STREAM_TRANSIENT, DATA_PREFETCH, MEM_TABLE_WALK, MEM_INST_FETCH,
	MEM_READ_DATA, DEMAND, PREFETCH, MEM_READ, HIF_RD_OR_WR, HIF_WR, HIF_RD,
	HIF_HI_PRI_RD, DFI_WR_DATA_CYCLES, DFI_RD_DATA_CYCLES,
	HPR_XACT_WHEN_CRITICAL, WR_XACT_WHEN_CRITICAL, OP_IS_ACTIVATE,
	OP_IS_RD_OR_WR, OP_IS_RD_ACTIVATE, OP_IS_RD, OP_IS_WR, OP_IS_PRECHARGE,
	PRECHARGE_FOR_RDWR, PRECHARGE_FOR_OTHER, RDWR_TRANSITIONS, WAR_HAZARD,
	RAW_HAZARD, WAW_HAZARD, OP_IS_ENTER_SELFREF, OP_IS_ENTER_POWERDOWN,
	OP_IS_REFRESH,
};

static const unsigned long rtk_16xxb_ddrc_log_configs[] = {
	0x0300
};

static const unsigned long rtk_16xxb_ddrc_log_pmcgs[] = {
	0x03a0, 0x03a4, 0x03a8, 0x03ac
};

static const unsigned int rtk_16xxb_ddrc_drv_ev[] = {
	RTK_DRV_OVERFLOW, DDRC_REFRESH
};

const static struct rtk_pmc_set_meta rtk_16xxb_ddrc_ps_meta[] = {
	{
		.name = "16xxb DDRC Total",
		.compatible = "ddrc-total",
		.type = PMC_SET__DDRC_TOTAL,
		.init = rtk_ddrc_ps_total_init,
		.group_size = 1,	/* direct map */

		.nr_clients = ARRAY_SIZE(rtk_16xxb_ddrc_total_ev),
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_ddrc_total_pmcgs),
		.nr_configs = 0,
		.clients = rtk_16xxb_ddrc_total_ev,
		.pmcgs = rtk_16xxb_ddrc_total_pmcgs,
		.configs = NULL,
	},
	{
		.name = "16xxb DDRC PMC",
		.compatible = "ddrc",
		.type = PMC_SET__DDRC,
		.init = rtk_ddrc_ps_init,
		.group_size = DDRC_PMC__USAGE_NUM,
		.config_size = 6,
		.config_width = 4,
		.val_mask = (const unsigned int []){
			RTK_PMU_VAL_MASK(32),
			RTK_PMU_VAL_MASK(32),
			RTK_PMU_VAL_MASK(32),
			RTK_PMU_VAL_MASK(32),
			RTK_PMU_VAL_MASK(32),
			RTK_PMU_VAL_MASK(32),
			RTK_PMU_VAL_MASK(8),
			RTK_PMU_VAL_MASK(8),
		},
		.ov_th = (const unsigned int []){
			RTK_PMU_OVERFLOW_TH(32),
			RTK_PMU_OVERFLOW_TH(32),
			RTK_PMU_OVERFLOW_TH(32),
			RTK_PMU_OVERFLOW_TH(32),
			RTK_PMU_OVERFLOW_TH(32),
			RTK_PMU_OVERFLOW_TH(32),
			0, 0,
		},

		.nr_clients = ARRAY_SIZE(rtk_16xxb_ddrc_clients),
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_ddrc_pmcgs) - 1,
		.nr_configs = ARRAY_SIZE(rtk_16xxb_ddrc_configs),
		.clients = rtk_16xxb_ddrc_clients,
		.pmcgs = rtk_16xxb_ddrc_pmcgs,
		.configs = rtk_16xxb_ddrc_configs,
	},
	{
		.name = "16xxb DDRC Log",
		.compatible = "ddrc-log",
		.type = PMC_SET__DDRC_LOG,
		.init = rtk_ddrc_ps_log_init,
		.group_size = 1,	/* direct map */
		.config_size = 4,
		.config_width = 6,

		.nr_clients = ARRAY_SIZE(rtk_16xxb_ddrc_log_ev),
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_ddrc_log_pmcgs),
		.nr_configs = 1,
		.clients = rtk_16xxb_ddrc_log_ev,
		.pmcgs = rtk_16xxb_ddrc_log_pmcgs,
		.configs = rtk_16xxb_ddrc_log_configs,
	},
	{
		.name = "16xxb DDRC DRV",
		.type = PMC_SET__DDRC_DRV,
		.init = rtk_ddrc_ps_drv_init,
		.group_size = 1,	/* direct map */

		.nr_clients = ARRAY_SIZE(rtk_16xxb_ddrc_drv_ev),
		.nr_pmcgs = DDRC_EV_NUM,
		.nr_configs = 0,
		.clients = rtk_16xxb_ddrc_drv_ev,
		.pmcgs = NULL,
		.configs = NULL,
	},
	{},
};

static inline bool
__is_scpu_log(int evid)
{
	return (evid <= STREAM_TRANSIENT && evid >= MEM_READ) ?
		true : false;
}

static int
__log_find_pmc(struct rtk_pmc_set *ps, int hwc, int target)
{
	union rtk_ddrc_event_desc desc = {.val = (int)hwc};
	int cidx = (desc.core << 1);

	if (!test_and_set_bit(cidx, &ps->used_mask))
		goto found;

	cidx++;
	if (!test_and_set_bit(cidx, &ps->used_mask))
		goto found;

	return RTK_INV_PMC_TARGET;

found:
	ps->tracking[cidx].target = target;
	ps->tracking[cidx].config = hwc;

	return cidx;
}

static bool
__migrate_scpu_log_ev(struct rtk_pmc_set *ps, union rtk_ddrc_event_desc ev)
{
	union rtk_pmc_desc pmc;
	struct perf_event *event;
	int cidx;

	/* the destination pmc is occupied by another scpu event */
	if (__is_scpu_log(ps->tracking[ev.core].target))
		return false;

	event = ps->tracking[ev.core].events[ev.usage];
	pmc = rtk_event_pmc_desc(event);
	cidx = __log_find_pmc(ps, ps->tracking[ev.core].config,
			      ps->tracking[ev.core].target);

	if (cidx != RTK_INV_PMC_TARGET) {
		/* update tracking event */
		pmc.idx = cidx;
		if (rtk_ps_track_event(ps, pmc, event))
			goto release;
		pmc.idx = ev.core;
		if (rtk_ps_untrack_event(ps, pmc, event))
			goto untrack;

		/* update hwc for migrated event */
		ps->set_perf_hwc(ps, pmc, event);

		return true;
	}

	return false;

untrack:
	pmc.idx = cidx;
	rtk_ps_untrack_event(ps, pmc, event);
release:
	test_and_clear_bit(cidx, &ps->used_mask);
	pr_err("%s migrate %llx from %d to %d failed\n",
	       ps->name, event->attr.config, ev.core, cidx);

	return false;
}

static int
rtk_16xxb_ddrc_log_find_pmc(struct rtk_pmc_set *ps, int hwc, int target)
{
	union rtk_ddrc_event_desc desc = {.val = (int)hwc};

	/* Each counter is dedicated for certain SCPU */
	if (__is_scpu_log(target)) {
		if (!test_and_set_bit(desc.core, &ps->used_mask)) {
			ps->tracking[desc.core].target = target;
			ps->tracking[desc.core].config = hwc;

			return desc.core;
		}

		/* try to migrate event to another pmc */
		return __migrate_scpu_log_ev(ps, desc) ?
			desc.core : RTK_INV_PMC_TARGET;
	} else {
		/*
		 * DDR controller has dual channel, log counters are dedicated
		 * to specific channel depending its index. Said more precisely,
		 * counter-0 and counter-1 are dedicated for channel-0,
		 * counter-2 and counter-3 are dedicated for channel1.
		 */
		return __log_find_pmc(ps, hwc, target);
	}
}

int rtk_16xxb_ddrc_init(struct rtk_pmu *pmu, struct device_node *dt)
{
	int ret;

	ret = rtk_ddrc_pmu_init(pmu, dt,
				 "rtk_16xxb_ddrc_pmu",
				 _DDRC_CTRL_OFFSET_,
				 rtk_16xxb_ddrc_attr_groups,
				 rtk_16xxb_ddrc_ps_meta,
				 RTK_PMU_META_NR(rtk_16xxb_ddrc_ps_meta));

	/*
	 * override the callback of perf log, since it requires some definitions
	 * depending on platforms
	 */
	if (!ret)
		pmu->pmcss[PMC_SET__DDRC_LOG]->arrange_pmc =
			rtk_16xxb_ddrc_log_find_pmc;

	return ret;
}
EXPORT_SYMBOL(rtk_16xxb_ddrc_init);
MODULE_LICENSE("GPL");
