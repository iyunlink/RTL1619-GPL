/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Header of Realtek Dbus PMU
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 * Copyright (C) 2019 Ping-Hsiung Chiu <phelic@realtek.com>
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

#ifndef __RTK_DBUS_PMU__
#define __RTK_DBUS_PMU__

#include <linux/perf_event.h>

#include "rtk_uncore_pmu.h"



#define	RTK_DBUS_PMU_PDEV_NAME	"rtk-dbus-pmu"
#define RTK_DBUS_PMU_CPUHP_NAME	RTK_PMU_CPUHP_NAME("rtk-dbus")

/*
 * Macros with complex values fail on kernel checking script.
 * This macro is a workaround, wrapping the value as a macro function and
 * expand.
 */
#define _ESCAPE_COMPLEX(...)	__VA_ARGS__

#define DBUS_EVENT_ATTR_GROUP(_n, _t, _s, _m, _c)			\
_ESCAPE_COMPLEX(							\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_acc_lat, \
				EV_CONFIG(_t, _s, ACC_LAT, _m, _c)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_max_lat, \
				EV_CONFIG(_t, _s, MAX_LAT, _m, _c)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_req_num, \
				EV_CONFIG(_t, _s, REQ_NUM, _m, _c)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_ack_num, \
				EV_CONFIG(_t, _s, ACK_NUM, _m, _c))	\
)

#define DBUS_EVENT_REF_GROUP(_n, _i)			\
_ESCAPE_COMPLEX(					\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, (_i) + DBUS_PMC__ACC_LAT),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, (_i) + DBUS_PMC__MAX_LAT),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, (_i) + DBUS_PMC__REQ_NUM),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, (_i) + DBUS_PMC__ACK_NUM)	\
)

#define DBUS_SYSH_EVENT_GROUP(_n, _t, _c)					\
static struct perf_pmu_events_attr attr_list_##_n[] = {				\
	DBUS_EVENT_ATTR_GROUP(_n, _t, DBUS_SYSH, 0, _c),			\
	DBUS_EVENT_ATTR_GROUP(_n##_rd, _t, DBUS_SYSH, DBUS_MON_READ, _c),	\
	DBUS_EVENT_ATTR_GROUP(_n##_wr, _t, DBUS_SYSH, DBUS_MON_WRITE, _c)	\
}

#define DBUS_SYS_EVENT_GROUP_V1(_n, _t)				\
static struct perf_pmu_events_attr attr_list_##_n[] = {		\
	DBUS_EVENT_ATTR_GROUP(_n, _t, DBUS_SYS, 0, 0)		\
}

#define DBUS_SYS_EVENT_GROUP_V2(_n, _t)				\
static struct perf_pmu_events_attr attr_list_##_n[] = {		\
	DBUS_EVENT_ATTR_GROUP(_n, _t, DBUS_SYS, 0, 0),		\
	DBUS_EVENT_ATTR_GROUP(_n##_rd, _t, DBUS_SYS, DBUS_MON_READ, 0),		\
	DBUS_EVENT_ATTR_GROUP(_n##_wr, _t, DBUS_SYS, DBUS_MON_WRITE, 0)		\
}

#define DBUS_SYSH_EVENT_REF_GROUP(_n)				\
_ESCAPE_COMPLEX(						\
	DBUS_EVENT_REF_GROUP(_n, 0),				\
	DBUS_EVENT_REF_GROUP(_n, DBUS_PMC__USAGE_NUM),		\
	DBUS_EVENT_REF_GROUP(_n, DBUS_PMC__USAGE_NUM << 1)	\
)

#define DBUS_SYS_EVENT_REF_GROUP_V1(_n)		DBUS_EVENT_REF_GROUP(_n, 0)
#define DBUS_SYS_EVENT_REF_GROUP_V2(_n)				\
_ESCAPE_COMPLEX(						\
	DBUS_EVENT_REF_GROUP(_n, 0),				\
	DBUS_EVENT_REF_GROUP(_n, DBUS_PMC__USAGE_NUM),		\
	DBUS_EVENT_REF_GROUP(_n, DBUS_PMC__USAGE_NUM << 1)	\
)

#define DBUS_TOTAL_EVENT_ATTR_GROUP(_n, _c)				\
static struct perf_pmu_events_attr attr_list_##_n[] = {			\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_cycle,				\
				EV_CONFIG(_c, DBUS_CH, CYCLE, 0, _c)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_ack,				\
				EV_CONFIG(_c, DBUS_CH, CH_ACK, 0, _c)), \
	RTK_PMU_EVENT_ATTR_ITEM(_n##_idle,				\
				EV_CONFIG(_c, DBUS_CH, CH_IDLE, 0, _c)) \
}

#define DBUS_TOTAL_EVENT_REF_GROUP(_n)					\
_ESCAPE_COMPLEX(							\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, DBUS_PMC__CYCLE),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, DBUS_PMC__CH_ACK),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, DBUS_PMC__CH_IDLE)	\
)

#define DBUS_DRV_EVENT_ATTR(_n, _c)		\
	RTK_PMU_EVENT_ATTR(_n, EV_CONFIG(_c, DBUS_DRV, NONE, 0, 0))

#define DBUS_DRV_EVENT_REF(_n)		\
	RTK_PMU_EVENT_REF(_n)


/* PMC set of DBUS-SYS */
enum {
	PMC_SET__DBUS_SYS		= 0,
	PMC_SET__DBUS_SYSH		= 1,
	PMC_SET__DBUS_CH		= 2,
	PMC_SET__DBUS_DRV		= 3,
};

/* Members of a DBUS-SYS PMC group */
enum {
	DBUS_PMC__ACC_LAT = 0,			/* accumulate latency */
	DBUS_PMC__MAX_LAT,			/* max latency */
	DBUS_PMC__REQ_NUM,			/* # of requests */
	DBUS_PMC__ACK_NUM,			/* # of acks */
	DBUS_PMC__USAGE_NUM
};

/* Overflow status bit offset of PMC group */
enum {
	DBUS_OV__ACC_LAT = 0,			/* accumulate latency */
	DBUS_OV__REQ_NUM = 1,			/* request number */
	DBUS_OV__ACK_NUM = 2,			/* ack number  */
	DBUS_OV__MAX_LAT = 3,			/* max latency  */
};

/* Members of a DBUS channel PMC group */
enum {
	DBUS_PMC__CYCLE = 0,			/* monitored cycles  */
	DBUS_PMC__CH_ACK,			/* monitored ack */
	DBUS_PMC__CH_IDLE,			/* idle cycles */
	DBUS_PMC__CH_USAGE_NUM
};

/* Driver event does not have usage field */
#define DBUS_PMC__NONE                    0

/* Driver events */
enum {
	DBUS_REFRESH	= RTK_DRV_EV_NUM,
	DBUS_EV_NUM
};


#define DBUS_MON_READ			0x02	/* flags to choose SYSH read */
#define DBUS_MON_WRITE			0x03	/* flags to choose SYSH write */

/* Dbus PMC control register offset */
/* keep counting even overflow */
#define DBUS_CTRL_CNT_FREERUN_BIT		4
#define DBUS_SYSH_MODE_CTRL			0x0058	/* SYSH monitor ctrl */
#define DBUS_SYS_MODE_CTRL			0x0C34	/* SYS monitor ctrl */
#define DBUS_MON_MODE_CTRL_MASK			0x03
#define DBUS_MON_MODE_CTRL_WIDTH		3
#define DBUS_CH_SEL_CTRL0			0x0108	/* Channel select reg */
#define DBUS_CH_SEL_CTRL1			0x010C	/* Channel select reg */

/* Isolated PMCG control */
#define DBUS_PMCG_ENABLE			0x004C
#define DBUS_PMCG_CLEAR				0x005C
#define DBUS_RSVD_BIT				31

/* Latch counter value to shadow register */
#define DBUS_UPDATE_SYSH_CTRL			0x0C1C
#define DBUS_UPDATE_SYS_CTRL			0x0C20
#define DBUS_UPDATE_CTRL_WIDTH			4	/* bits for a PMCG */

/* Overflow enable */
#define DBUS_SYSH_OVERFLOW_ENABLE		0x0C04
#define DBUS_SYS_OVERFLOW_ENABLE		0x0C08
#define DBUS_CH_OVERFLOW_ENABLE			0x0C08

/* Overflow status */
#define DBUS_OV_SYSH_STATUS			0x0508	/* SYSH overflow */
#define DBUS_OV_SYS_STATUS			0x050C	/* SYS overflow */
#define DBUS_OV_CH_STATUS			0x050C	/* CH overflow */
#define DBUS_OV_PMCG_WIDTH			4	/* bits for a PMCG */

/* overflow bit to pmcg index */
static inline int ov_bit_to_pmcg_idx(int bit)
{
	return bit >> 2;
}

/* overflow bit to pmc index within a pmcg */
static inline int ov_bit_to_cnt_idx(int bit)
{
	return bit & GENMASK(1, 0);
}

/* Descriptor of pmc config for a given pmc */
union pmc_config_desc {
	struct {
		int set:8;
		int idx:8;
		int offset:8;
		int width:8;
	};

	int val;
};

static inline int
is_valid_pmc_conf(union pmc_config_desc conf)
{
	return conf.val >= 0;
}

/* Descriptor of dbus event */
union rtk_dbus_event_desc {
	int val;
	struct {
		/* monitored target, bridge 6:3, module 2:0 */
		unsigned int target:8;

		unsigned int set:2;	/* pmc set */
		unsigned int mode:2;	/* read/write/all */
		unsigned int ch:2;	/* DDR channel */
		unsigned int usage:2;	/* pmc usage */
	};
};

#define SET_OFFSET		8
#define MODE_OFFSET		10
#define CH_OFFSET		12
#define USAGE_OFFSET		14
#define TARGET_MASK		0xFF
#define HWC_MASK		0x3FFF

#define EV_CONFIG(ev, set, usage, mode, ch)		\
	((ev) | (PMC_SET__##set << SET_OFFSET) |	\
	 ((DBUS_PMC__##usage) << USAGE_OFFSET) |	\
	 ((mode) << MODE_OFFSET) | ((ch) << CH_OFFSET))

enum {
	DBUS_CH_0 = 0,
	DBUS_CH_1,
	DBUS_CH_AUTO,	/* detect the actual channel from control register */
};

static inline union rtk_dbus_event_desc
__get_event_desc(u64 config)
{
	union rtk_dbus_event_desc desc = {.val = (int)config};

	return desc;
}

static inline union rtk_dbus_event_desc
get_event_desc(struct perf_event *event)
{
	return __get_event_desc(event->attr.config);
}

static inline int rtk_dbus_pmc_target(u64 config)
{
	return (int)(config & TARGET_MASK);
}

int rtk_dbus_pmc_config(u64 config);
int rtk_dbus_find_ch_pmc(struct rtk_pmc_set *ps, int hwc, int target);
void rtk_dbus_set_hwc(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
		      struct perf_event *event);
void rtk_dbus_update_mode_reg(struct rtk_pmc_set *ps, struct perf_event *event,
			      int op);
struct rtk_pmc_set *rtk_dbus_get_pmc_set(struct rtk_pmu *pmu,
					 struct perf_event *event);
int rtk_dbus_check_event(struct rtk_pmu *pmu, struct perf_event *event);
int rtk_dbus_ps_init(const struct rtk_pmc_set_meta *meta,
		     struct rtk_pmc_set *ps);
int rtk_dbus_ch_init(const struct rtk_pmc_set_meta *meta,
		     struct rtk_pmc_set *ps);
int rtk_dbus_drv_init(const struct rtk_pmc_set_meta *meta,
		      struct rtk_pmc_set *ps);
int rtk_dbus_pmu_init(struct rtk_pmu *pmu, struct device_node *dt,
		      const char *name, unsigned long ctrl_offset,
		      const struct attribute_group **attr_groups,
		      const struct rtk_pmc_set_meta *meta, int nr_ps);

extern struct attribute_group rtk_dbus_format_attr_group;

/* Platform specific init functions */
int rtk_16xxb_dbus_init(struct rtk_pmu *pmu, struct device_node *dt);


#endif /* End of __RTK_DBUS_PMU__ */
