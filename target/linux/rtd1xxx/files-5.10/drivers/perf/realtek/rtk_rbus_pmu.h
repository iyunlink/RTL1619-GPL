/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Header of Realtek register bus PMU
 *
 * Copyright (C) 2020-2023 Realtek Semiconductor Corporation
 * Copyright (C) 2020-2023 Ping-Hsiung Chiu <phelic@realtek.com>
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

#ifndef __RTK_RBUS_PMU__
#define __RTK_RBUS_PMU__

#include <linux/perf_event.h>

#include "rtk_uncore_pmu.h"


#define	RTK_RBUS_PMU_PDEV_NAME	"rtk-rbus-pmu"
#define RTK_RBUS_PMU_CPUHP_NAME	RTK_PMU_CPUHP_NAME("rtk-rbus")

/*
 * Macros with complex values fail on kernel checking script.
 * This macro is a workaround, wrapping the value as a macro function and
 * expand.
 */
#define _ESCAPE_COMPLEX(...)	__VA_ARGS__

#define RBUS_EVENT_ATTR_GROUP_V1(_n, _s, _t)			\
_ESCAPE_COMPLEX(						\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_acc_lat, \
				RBUS_EV_CONFIG(_t, _s, ACC_LAT)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_max_lat, \
				RBUS_EV_CONFIG(_t, _s, MAX_LAT)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_req_num, \
				RBUS_EV_CONFIG(_t, _s, REQ_NUM))	\
)

#define RBUS_EVENT_REF_GROUP_V1(_n)			\
_ESCAPE_COMPLEX(					\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, RBUS_PMC__ACC_LAT),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, RBUS_PMC__MAX_LAT),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, RBUS_PMC__REQ_NUM)	\
)

#define RBUS_EVENT_GROUP_V1(_n, _t, _s)					\
static struct perf_pmu_events_attr attr_list_##_n[] = {			\
	RBUS_EVENT_ATTR_GROUP_V1(_n, _s, _t)			\
}

#define RBUS_EVENT_ATTR_GROUP_V2(_n, _s, _t)			\
_ESCAPE_COMPLEX(						\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_req_num, \
				RBUS_EV_CONFIG(_t, _s, REQ)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rtg_total, \
				RBUS_EV_CONFIG(_t, _s, RTG_TOTAL)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rta_max, \
				RBUS_EV_CONFIG(_t, _s, RTA_MAX)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rta_total, \
				RBUS_EV_CONFIG(_t, _s, RTA_TOTAL))	\
)

#define RBUS_EVENT_REF_GROUP_V2(_n)			\
_ESCAPE_COMPLEX(					\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, RBUS_PMC__REQ),		\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, RBUS_PMC__RTG_TOTAL),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, RBUS_PMC__RTA_MAX),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, RBUS_PMC__RTA_TOTAL)	\
)

#define RBUS_EVENT_GROUP_V2(_n, _t, _s)					\
static struct perf_pmu_events_attr attr_list_##_n[] = {			\
	RBUS_EVENT_ATTR_GROUP_V2(_n, _s, _t)			\
}

#define RBUS_DRV_EVENT_ATTR(_n, _c)		\
	RTK_PMU_EVENT_ATTR(_n, RBUS_EV_CONFIG(_c, RBUS_DRV, NONE))

#define RBUS_DRV_EVENT_REF(_n)		\
	RTK_PMU_EVENT_REF(_n)


/* PMC set of Rbus */
enum {
	PMC_SET__RBUS		= 0,
	PMC_SET__RBUS_DRV,
};

/* Members of a Rbus PMC group */
enum rbus_ev_enum_v1 {
	RBUS_PMC__ACC_LAT = 0,          /* accumulate latency */
	RBUS_PMC__MAX_LAT,              /* max latency */
	RBUS_PMC__REQ_NUM,              /* # of requests */
	RBUS_PMC__V1_USAGE_NUM
};

/* Members of a Rbus PMC group */
enum rbus_ev_enum_v2 {
	RBUS_PMC__REQ,			/* # of requests */
	RBUS_PMC__RTG_TOTAL,		/* accumulate grant latency */
	RBUS_PMC__RTA_MAX,		/* max latency */
	RBUS_PMC__RTA_TOTAL,		/* accumulate latency */
	RBUS_PMC__V2_USAGE_NUM
};

/* Driver event does not have usage field */
#define RBUS_PMC__NONE			0

/* Driver events */
enum {
	RBUS_REFRESH	= RTK_DRV_EV_NUM,
	RBUS_EV_NUM
};

#define RBUS_OVERFLOW_STATUS		0x0074
#define RBUS_OVERFLOW_ENABLE		0x0078
#define RBUS_OVERFLOW_STATUS_WIDTH	28
#define RBUS_OVERFLOW_PMCG_WIDTH	4
#define RBUS_OVERFLOW_OFFSET		4

union rtk_rbus_event_desc {
	int val;
	struct {
		unsigned target:5;	/* target id */
		unsigned set:3;		/* pmc set */
		unsigned usage:4;	/* pmc usage */
	};
};

#define SET_OFFSET		5
#define USAGE_OFFSET		8
#define TARGET_MASK		0x001f
#define HWC_MASK		0x00ff

#define RBUS_EV_CONFIG(ev, set, usage)	\
	((ev) | (PMC_SET__##set << SET_OFFSET) | \
	 (RBUS_PMC__##usage << USAGE_OFFSET))

static inline union rtk_rbus_event_desc
__get_event_desc(u64 config)
{
	union rtk_rbus_event_desc desc = {.val = (int)config};

	return desc;
}

static inline union rtk_rbus_event_desc
get_event_desc(struct perf_event *event)
{
	return __get_event_desc(event->attr.config);
}

static inline int
rtk_rbus_pmc_target(u64 config)
{
	return (int)(config & TARGET_MASK);
}

int rbus_pmu_ps_init_v1(const struct rtk_pmc_set_meta *meta,
			struct rtk_pmc_set *ps);
int rbus_pmu_ps_init_v2(const struct rtk_pmc_set_meta *meta,
			struct rtk_pmc_set *ps);
int rbus_pmu_ps_drv_init(const struct rtk_pmc_set_meta *meta,
			 struct rtk_pmc_set *ps);
int rtk_rbus_pmu_init(struct rtk_pmu *pmu, struct device_node *dt,
		      const char *name,
		      const struct attribute_group **attr_groups,
		      const struct rtk_pmc_set_meta *meta, int nr_ps);

extern struct attribute_group rtk_rbus_format_attr_group;

/* Platform specific init functions */
int rtk_16xxb_rbus_init(struct rtk_pmu *pmu, struct device_node *dt);


#endif /* End of __RTK_RBUS_PMU__ */
