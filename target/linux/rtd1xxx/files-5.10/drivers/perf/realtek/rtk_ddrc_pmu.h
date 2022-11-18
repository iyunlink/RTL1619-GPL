/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Header of Realtek DDR memory controller PMU
 *
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 * Copyright (C) 2020 Ping-Hsiung Chiu <phelic@realtek.com>
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

#ifndef __RTK_DDRC_H__

#include <linux/perf_event.h>

#include "rtk_uncore_pmu.h"


#define	RTK_DDRC_PMU_PDEV_NAME		"rtk-ddrc-pmu"
#define RTK_DDRC_PMU_CPUHP_NAME		RTK_PMU_CPUHP_NAME("rtk-ddrc")

/*
 * Macros with complex values fail on kernel checking script.
 * This macro is a workaround, wrapping the value as a macro function and
 * expand.
 */
#define _ESCAPE_COMPLEX(...)	__VA_ARGS__


#define __DDRC_EVENT_ATTR_GROUP(_n, _t)				\
_ESCAPE_COMPLEX(						\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_wack, \
				EV_CONFIG(_t, DDRC, WACK, 0)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_wreq, \
				EV_CONFIG(_t, DDRC, WREQ, 0)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_wlat, \
				EV_CONFIG(_t, DDRC, WLAT, 0)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rack, \
				EV_CONFIG(_t, DDRC, RACK, 0)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rreq, \
				EV_CONFIG(_t, DDRC, RREQ, 0)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rlat, \
				EV_CONFIG(_t, DDRC, RLAT, 0)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_wpend, \
				EV_CONFIG(_t, DDRC, WPEND, 0)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rpend, \
				EV_CONFIG(_t, DDRC, RPEND, 0))	\
)

#define __DDRC_EVENT_REF_GROUP(_n)			\
_ESCAPE_COMPLEX(					\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, DDRC_PMC__WACK),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, DDRC_PMC__WREQ),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, DDRC_PMC__WLAT),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, DDRC_PMC__RACK),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, DDRC_PMC__RREQ),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, DDRC_PMC__RLAT),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, DDRC_PMC__WPEND),\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, DDRC_PMC__RPEND)	\
)

#define DDRC_EVENT_GROUP(_n, _t)				\
static struct perf_pmu_events_attr attr_list_##_n[] = {		\
	__DDRC_EVENT_ATTR_GROUP(_n, _t),			\
}
#define DDRC_EVENT_REF_GROUP(_n)	__DDRC_EVENT_REF_GROUP(_n)

#define DDRC_TOTAL_EVENT_ATTR(_n, _ev)		\
	RTK_PMU_EVENT_ATTR(_n, EV_CONFIG(_ev, DDRC_TOTAL, NONE, 0))
#define DDRC_TOTAL_EVENT_REF(_n)		\
	RTK_PMU_EVENT_REF(_n)

#define DDRC_LOG_EVENT_ATTR_GROUP(_n, _ev, _c)	\
static struct perf_pmu_events_attr attr_list_##_n[] = {	\
	__DDRC_LOG_EVENT_ATTR_##_c(_n, _ev),		\
}
#define __DDRC_LOG_EVENT_ATTR_2(_n, _ev)		\
_ESCAPE_COMPLEX(					\
	RTK_PMU_EVENT_ATTR_ITEM(dch0_##_n, EV_CONFIG(_ev, DDRC_LOG, NONE, 0)),\
	RTK_PMU_EVENT_ATTR_ITEM(dch1_##_n, EV_CONFIG(_ev, DDRC_LOG, NONE, 1)) \
)
#define DDRC_LOG_EVENT_REF_GROUP(_n, _c)		\
	__DDRC_LOG_EVENT_REF_##_c(_n)
#define __DDRC_LOG_EVENT_REF_2(_n)			\
_ESCAPE_COMPLEX(					\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, 0),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, 1)	\
)

#define DDRC_LOG_CPU_EVENT_ATTR_GROUP(_n, _ev, _c)	\
static struct perf_pmu_events_attr attr_list_##_n[] = {	\
	__DDRC_LOG_CPU_EVENT_ATTR_##_c(_n, _ev),	\
}
#define __DDRC_LOG_CPU_EVENT_ATTR_4(_n, _ev)		\
_ESCAPE_COMPLEX(					\
	RTK_PMU_EVENT_ATTR_ITEM(core0_##_n, EV_CONFIG(_ev, DDRC_LOG, NONE, 0)),\
	RTK_PMU_EVENT_ATTR_ITEM(core1_##_n, EV_CONFIG(_ev, DDRC_LOG, NONE, 1)),\
	RTK_PMU_EVENT_ATTR_ITEM(core2_##_n, EV_CONFIG(_ev, DDRC_LOG, NONE, 2)),\
	RTK_PMU_EVENT_ATTR_ITEM(core3_##_n, EV_CONFIG(_ev, DDRC_LOG, NONE, 3)) \
)
#define DDRC_LOG_CPU_EVENT_REF_GROUP(_n, _c)		\
	__DDRC_LOG_CPU_EVENT_REF_##_c(_n)
#define __DDRC_LOG_CPU_EVENT_REF_4(_n)			\
_ESCAPE_COMPLEX(					\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, 0),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, 1),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, 2),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, 3)	\
)

#define DDRC_DRV_EVENT_ATTR(_n, _ev)		\
	RTK_PMU_EVENT_ATTR(_n, EV_CONFIG(_ev, DDRC_DRV, NONE, 0))
#define DDRC_DRV_EVENT_REF(_n)			\
	RTK_PMU_EVENT_REF(_n)


/* PMC set of DDRC */
enum {
	PMC_SET__DDRC_TOTAL = 0,	/* total statistics */
	PMC_SET__DDRC,			/* dedicated target */
	PMC_SET__DDRC_LOG,		/* DDRC event statistics */
	PMC_SET__DDRC_DRV,		/* driver event, must be the last set */
};

/* Members of a DDRC channel group */
enum {
	DDRC_PMC__WACK = 0,			/* write ack */
	DDRC_PMC__WREQ,				/* write req */
	DDRC_PMC__WLAT,				/* write accumulated latency */
	DDRC_PMC__RACK,				/* read ack */
	DDRC_PMC__RREQ,				/* read req */
	DDRC_PMC__RLAT,				/* read accumulated latency */
	DDRC_PMC__WPEND,			/* write pending requests */
	__DDRC_PMC__PEND_USAGE_START = DDRC_PMC__WPEND,
	DDRC_PMC__RPEND,			/* read pending requests */
	DDRC_PMC__USAGE_NUM
};

/* Members of DDRC total statistics */
enum {
	DDRC_PMC__TOTAL_CYCLE = 0,		/* monitored cycles  */
	DDRC_PMC__TOTAL_WACK,			/* monitored write ack */
	DDRC_PMC__TOTAL_RACK,			/* monitored read ack */
	DDRC_PMC__TOTAL_USAGE_NUM
};

/* SCPU part of DDRC LOG events has dedicated core number */
enum {
	DDRC_PMC__CORE0 = 0,
	DDRC_PMC__CORE1,
	DDRC_PMC__CORE2,
	DDRC_PMC__CORE3,
};

#define DDRC_PMC__NONE  0

/* Driver events */
enum {
	DDRC_REFRESH	= RTK_DRV_EV_NUM,
	DDRC_EV_NUM
};

/*
 * FIXME: If NR_PEND_REQ_PER_COUNTER is not power of 2, the corresponding
 * caculation get wrong result.
 */
#define NR_PEND_REQ_PER_COUNTER		4

/* Isolated PMCG control */
#define DDRC_PMCG_ENABLE		0x03C0
#define DDRC_PMCG_CLEAR			0x03C4
#define DDRC_RSVD_BIT			31

/* Update control */
/* Latch counter value to shadow register */
#define DDRC_UPDATE_0			0x03D8
#define DDRC_UPDATE_1			0x03DC
#define DDRC_PMCG_UPDATE_WIDTH		8

/* Overflow control */
#define DDRC_OVERFLOW_ENABLE_0		0x03C8
#define DDRC_OVERFLOW_ENABLE_1		0x03CC

/* Overflow status */
#define DDRC_OVERFLOW_STATUS_0		0x03D0
#define DDRC_OVERFLOW_STATUS_1		0x03D4
#define DDRC_OV_STATUS_WIDTH		32	/* overflow status reg width */
#define DDRC_OV_PMCG_WIDTH		8	/* bits for a PMCG */

/* keep counting after overflow */
#define DDRC_CTRL_CNT_FREERUN_BIT	1
#define DDRC_LOG_CONF_OFFSET		4

/* Description of pmc config for a given pmc */
union pmc_config_desc {
	struct {
		int set:8;
		int idx:8;
		int offset:8;
		int width:8;
	};

	int val;
};

union rtk_ddrc_event_desc {
	int val;
	struct {
		unsigned int target:8;	/* target id */
		unsigned int set:3;	/* pmc set */
		unsigned int usage:3;	/* pmc usage */
		unsigned int core:4;	/* core # or ch# */
	};
};

#define SET_OFFSET		8
#define USAGE_OFFSET		11
#define CORE_OFFSET		14
#define TARGET_MASK		0x00ff
#define HWC_MASK		0x3c7ff	/* mask out usage field */

#define EV_CONFIG(ev, set, usage, core)	\
	((ev) | (PMC_SET__##set << SET_OFFSET) | \
	 (DDRC_PMC__##usage << USAGE_OFFSET) | \
	 core << CORE_OFFSET)

static inline union rtk_ddrc_event_desc
__get_event_desc(u64 config)
{
	union rtk_ddrc_event_desc desc = {.val = (int)config};

	return desc;
}

static inline union rtk_ddrc_event_desc
get_event_desc(struct perf_event *event)
{
	return __get_event_desc(event->attr.config);
}

static inline int
rtk_ddrc_pmc_target(u64 config)
{
	return (int)(config & TARGET_MASK);
}

struct rtk_pmc_set *rtk_ddrc_get_pmc_set(struct rtk_pmu *pmu,
					 struct perf_event *event);
int rtk_ddrc_check_event(struct rtk_pmu *pmu, struct perf_event *event);
union rtk_pmc_desc rtk_ddrc_arrange_pmc(struct rtk_pmu *pmu, u64 config);
int rtk_ddrc_ps_init(const struct rtk_pmc_set_meta *meta,
		     struct rtk_pmc_set *ps);
int rtk_ddrc_ps_total_init(const struct rtk_pmc_set_meta *meta,
			   struct rtk_pmc_set *ps);
int rtk_ddrc_ps_log_init(const struct rtk_pmc_set_meta *meta,
			 struct rtk_pmc_set *ps);
int rtk_ddrc_ps_drv_init(const struct rtk_pmc_set_meta *meta,
			 struct rtk_pmc_set *ps);
int rtk_ddrc_pmu_init(struct rtk_pmu *pmu, struct device_node *dt,
		      const char *name, unsigned long ctrl_offset,
		      const struct attribute_group **attr_groups,
		      const struct rtk_pmc_set_meta *meta, int nr_ps);

extern struct attribute_group rtk_ddrc_format_attr_group;

/* Platform specific init functions */
int rtk_16xxb_ddrc_init(struct rtk_pmu *pmu, struct device_node *dt);

#endif /* End of __RTK_DDRC_H__ */

