/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Header of Realtek Traffic Event PMU
 *
 * Copyright (c) 2021 Realtek Semiconductor Corp.
 * Copyright (C) 2021 Ping-Hsiung Chiu <phelic@realtek.com>
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

#ifndef __RTK_TFC_H__

#include <linux/perf_event.h>

#include "rtk_uncore_pmu.h"


#define	RTK_TFC_PMU_PDEV_NAME		"rtk-tfc-pmu"
#define RTK_TFC_PMU_CPUHP_NAME		RTK_PMU_CPUHP_NAME("rtk-tfc")


/*
 * Macros with complex values fail on kernel checking script.
 * This macro is a workaround, wrapping the value as a macro function and
 * expand.
 */
#define _ESCAPE_COMPLEX(...)	__VA_ARGS__

#define __TFC_EVENT_ATTR_GROUP(_n, _set)			\
_ESCAPE_COMPLEX(						\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_idle, \
		EV_CONFIG(TFC_PHASE__IDLE, PHASE, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_ridle, \
		EV_CONFIG(TFC_PHASE__RIDLE, PHASE, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rip, \
		EV_CONFIG(TFC_PHASE__RIP, PHASE, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rcp, \
		EV_CONFIG(TFC_PHASE__RCP, PHASE, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rap, \
		EV_CONFIG(TFC_PHASE__RAP, PHASE, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_widle, \
		EV_CONFIG(TFC_PHASE__WIDLE, PHASE, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_wip, \
		EV_CONFIG(TFC_PHASE__WIP, PHASE, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_wcp, \
		EV_CONFIG(TFC_PHASE__WCP, PHASE, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_wap, \
		EV_CONFIG(TFC_PHASE__WAP, PHASE, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_artl, \
		EV_CONFIG(TFC_TIME__ARTL, TIME, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_arr, \
		EV_CONFIG(TFC_TIME__ARR, TIME, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_riv, \
		EV_CONFIG(TFC_TIME__RIV, TIME, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_iriv, \
		EV_CONFIG(TFC_TIME__IRIV, TIME, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_awtl, \
		EV_CONFIG(TFC_TIME__AWTL, TIME, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_awr, \
		EV_CONFIG(TFC_TIME__AWR, TIME, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_wrb, \
		EV_CONFIG(TFC_TIME__WBR, TIME, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_th_artl, \
		EV_CONFIG(TFC_TH__ARTL, TH, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_th_arr, \
		EV_CONFIG(TFC_TH__ARR, TH, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_th_riv, \
		EV_CONFIG(TFC_TH__RIV, TH, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_th_iriv, \
		EV_CONFIG(TFC_TH__IRIV, TH, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_th_awtl, \
		EV_CONFIG(TFC_TH__AWTL, TH, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_th_awr, \
		EV_CONFIG(TFC_TH__AWR, TH, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_th_wbr, \
		EV_CONFIG(TFC_TH__WBR, TH, _set)),		\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rip_db, \
		EV_CONFIG(TFC_PHASE_DATA__RIP_DB, PHASE_DATA, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rcp_db, \
		EV_CONFIG(TFC_PHASE_DATA__RCP_DB, PHASE_DATA, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rap_db, \
		EV_CONFIG(TFC_PHASE_DATA__RAP_DB, PHASE_DATA, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_wip_db, \
		EV_CONFIG(TFC_PHASE_DATA__WIP_DB, PHASE_DATA, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_wcp_db, \
		EV_CONFIG(TFC_PHASE_DATA__WCP_DB, PHASE_DATA, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_wap_db, \
		EV_CONFIG(TFC_PHASE_DATA__WAP_DB, PHASE_DATA, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_idle2rip, \
		EV_CONFIG(TFC_PHASE_CHG__IDLE_TO_RIP, PHASE_CHG, _set)),\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_rip2rcp, \
		EV_CONFIG(TFC_PHASE_CHG__RIP_TO_RCP, PHASE_CHG, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_p2rap, \
		EV_CONFIG(TFC_PHASE_CHG__P_TO_RAP, PHASE_CHG, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_idle2wip, \
		EV_CONFIG(TFC_PHASE_CHG__IDLE_TO_WIP, PHASE_CHG, _set)),\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_wip2wcp, \
		EV_CONFIG(TFC_PHASE_CHG__WIP_TO_WCP, PHASE_CHG, _set)),	\
	RTK_PMU_EVENT_ATTR_ITEM(_n##_p2wcp, \
		EV_CONFIG(TFC_PHASE_CHG__P_TO_WAP, PHASE_CHG, _set))	\
)

#define __TFC_EVENT_REF_GROUP(_n)			\
_ESCAPE_COMPLEX(					\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, TFC_PHASE__IDLE),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, TFC_PHASE__RIDLE),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, TFC_PHASE__RIP),		\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, TFC_PHASE__RCP),		\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, TFC_PHASE__RAP),		\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, TFC_PHASE__WIDLE),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, TFC_PHASE__WIP),		\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, TFC_PHASE__WCP),		\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, TFC_PHASE__WAP),		\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__ARTL),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__ARR),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__RIV),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__IRIV), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__AWTL), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__AWR),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__WBR),	\
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__NUM + \
			       TFC_TH__ARTL), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__NUM + \
			       TFC_TH__ARR), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__NUM + \
			       TFC_TH__RIV), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__NUM + \
			       TFC_TH__IRIV), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__NUM + \
			       TFC_TH__AWTL), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__NUM + \
			       TFC_TH__AWR), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			       TFC_PHASE__NUM + TFC_TIME__NUM + \
			       TFC_TH__WBR), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			TFC_PHASE__NUM + TFC_TIME__NUM + TFC_TH__NUM +	\
			TFC_PHASE_DATA__RIP_DB), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			TFC_PHASE__NUM + TFC_TIME__NUM + TFC_TH__NUM +	\
			TFC_PHASE_DATA__RCP_DB), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			TFC_PHASE__NUM + TFC_TIME__NUM + TFC_TH__NUM +	\
			TFC_PHASE_DATA__RAP_DB), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			TFC_PHASE__NUM + TFC_TIME__NUM + TFC_TH__NUM +	\
			TFC_PHASE_DATA__WIP_DB), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			TFC_PHASE__NUM + TFC_TIME__NUM + TFC_TH__NUM +	\
			TFC_PHASE_DATA__WCP_DB), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			TFC_PHASE__NUM + TFC_TIME__NUM + TFC_TH__NUM +	\
			TFC_PHASE_DATA__WAP_DB), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			TFC_PHASE__NUM + TFC_TIME__NUM + TFC_TH__NUM +	\
			TFC_PHASE_DATA__NUM + TFC_PHASE_CHG__IDLE_TO_RIP), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			TFC_PHASE__NUM + TFC_TIME__NUM + TFC_TH__NUM +	\
			TFC_PHASE_DATA__NUM + TFC_PHASE_CHG__RIP_TO_RCP), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			TFC_PHASE__NUM + TFC_TIME__NUM + TFC_TH__NUM +	\
			TFC_PHASE_DATA__NUM + TFC_PHASE_CHG__P_TO_RAP), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			TFC_PHASE__NUM + TFC_TIME__NUM + TFC_TH__NUM +	\
			TFC_PHASE_DATA__NUM + TFC_PHASE_CHG__IDLE_TO_WIP), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			TFC_PHASE__NUM + TFC_TIME__NUM + TFC_TH__NUM +	\
			TFC_PHASE_DATA__NUM + TFC_PHASE_CHG__WIP_TO_WCP), \
	RTK_PMU_EVENT_ITEM_REF(attr_list_##_n, \
			TFC_PHASE__NUM + TFC_TIME__NUM + TFC_TH__NUM +	\
			TFC_PHASE_DATA__NUM + TFC_PHASE_CHG__P_TO_WAP)	\
)

#define TFC_EVENT_GROUP(_n, _set)				\
static struct perf_pmu_events_attr attr_list_##_n[] = {		\
	__TFC_EVENT_ATTR_GROUP(_n, _set),			\
}
#define TFC_EVENT_REF_GROUP(_n)	__TFC_EVENT_REF_GROUP(_n)

#define TFC_DRV_EVENT_ATTR(_n, _ev, _set)		\
	RTK_PMU_EVENT_ATTR(_n, EV_CONFIG(_ev, NONE, _set))
#define TFC_DRV_EVENT_REF(_n)			\
	RTK_PMU_EVENT_REF(_n)


/* PMC set of TFC */
enum {
	PMC_SET__TFC = 0,
	PMC_SET__TFC_DRV,	/* driver event */
};

/* traffic event catagory */
enum {
	TFC_CAT__PHASE = 0,	/* phase */
	TFC_CAT__TIME,		/* cycles between signals */
	TFC_CAT__TH,		/* threshold */
	TFC_CAT__PHASE_DATA,	/* data transfered in a phase */
	TFC_CAT__PHASE_CHG,	/* phase change */
	TFC_CAT__NUM
};

#define TFC_CAT__NONE	0

/* events of TFC phase */
enum {
	TFC_PHASE__IDLE = 0,	/* # cycles being idle */
	TFC_PHASE__RIDLE,	/* # cycles read being idle */
	TFC_PHASE__RIP,		/* # cycles read in initial phase */
	TFC_PHASE__RCP,		/* # cycles read in constrained phase */
	TFC_PHASE__RAP,		/* # cycles read in average phase */
	TFC_PHASE__WIDLE,	/* # cycles write being idle */
	TFC_PHASE__WIP,		/* # cycles write in initial phase */
	TFC_PHASE__WCP,		/* # cycles write in constrained phase */
	TFC_PHASE__WAP,		/* # cycles write in average phase */
	TFC_PHASE__NUM
};

/* events of TFC time */
enum {
	TFC_TIME__ARTL = 0,	/* # cycles outstanding read = TxnLimit */
	TFC_TIME__ARR,		/* # cycles waiting for read address */
	TFC_TIME__RIV,		/* # cycles from addr to first data */
	TFC_TIME__IRIV,		/* # cycles from addr to data of first trans */
	TFC_TIME__AWTL,		/* # cycles outstanding write = TxnLimit */
	TFC_TIME__AWR,		/* # cycles waiting for write address */
	TFC_TIME__WBR,		/* # cycles waiting for write data */
	TFC_TIME__NUM
};

/* events of TFC time over threshold */
enum {
	TFC_TH__ARTL = 0,	/* transactions takes ARTL time over threshold */
	TFC_TH__ARR,		/* trns takes ARR time over threshold */
	TFC_TH__RIV,		/* trns takes RIV time over threshold */
	TFC_TH__IRIV,		/* trns takes IRIV time over threshold */
	TFC_TH__AWTL,		/* trns takes AWTL time over threshold */
	TFC_TH__AWR,		/* trns takes AWR time over threshold */
	TFC_TH__WBR,		/* trns takes WBR time over threshold */
	TFC_TH__NUM
};

/* events of TFC phase data beats */
enum {
	TFC_PHASE_DATA__RIP_DB = 0,	/* data in read initial phase */
	TFC_PHASE_DATA__RCP_DB,		/* data in read constrained phase */
	TFC_PHASE_DATA__RAP_DB,		/* data in read average phase */
	TFC_PHASE_DATA__WIP_DB,		/* data in write initial phase */
	TFC_PHASE_DATA__WCP_DB,		/* data in write constrained phase */
	TFC_PHASE_DATA__WAP_DB,		/* data in write average phase */
	TFC_PHASE_DATA__NUM
};

/* events of TFC phase change */
enum {
	TFC_PHASE_CHG__IDLE_TO_RIP = 0,	/* idle to read initial phase */
	TFC_PHASE_CHG__RIP_TO_RCP,	/* read initial to read constrained */
	TFC_PHASE_CHG__P_TO_RAP,	/* any phase to read average phase */
	TFC_PHASE_CHG__IDLE_TO_WIP,	/* idle to write initial phase */
	TFC_PHASE_CHG__WIP_TO_WCP,	/* write initial to write constrained */
	TFC_PHASE_CHG__P_TO_WAP,	/* any phase to write average phase */
	TFC_PHASE_CHG__NUM
};

/* Driver events */
enum {
	TFC_REFRESH	= RTK_DRV_EV_NUM,
	TFC_EV_NUM
};

/* config registers of a TFC PMC set */
enum {
	TFC_CFG_CTRL = 0,	/* controlling monitored event */
	TFC_CFG_TH,		/* ref threshold for TFC_TH events */
	TFC_CFG_UPDATE,		/* update ctrl for reading counters */
	TFC_CFG_CLEAR,		/* clear counters */
	TFC_CFG_INT,		/* enable overflow interrupt*/
	TFC_CFG_OVERFLOW,	/* overflow status */
};

/* TFC use Realtek bus tag or AXI tag to adjudge phase */
enum {
	TFC_MODE__RTK = 0,
	TFC_MODE__AXI
};

#define TFC_CTRL_EN_BIT		0	/* bit of TFC enable */
#define TFC_CTRL_MODE_BIT	28	/* bit of TFC mode control*/
#define TFC_CTRL_PMC_EN_OFFSET	1	/* bit offset of PMC enable */
#define TFC_CTRL_PMC_SEL_OFFSET	4	/* bit offset of first PMC select */

/* bits of a TFC ch control, such as overflow, intterupt, clear and update */
#define TFC_CH_CTRL_BITS	4
#define TFC_OV_RSVD_BIT		31


/* Description of pmc config for a given pmc */
union pmc_config_desc {
	int val;
	struct {
		int set:8;	/* pmc set */
		int idx:8;	/* pmc index */
		int offset:8;	/* corresponding bit offset */
		int width:8;	/* bits for a event config */
	};
};

union rtk_tfc_event_desc {
	unsigned int val;
	/*
	 * Take care of the property order, since we use union to parse the
	 * event desc, the bit-order may lead to wrong translations from
	 * __get_event_desc().
	 */
	struct {
		/*
		 * event category and sub id form a full event id
		 */
		unsigned int sid:4;	/* sub event id in a pmc catogary */
		unsigned int cat:4;	/* event catogary */
		/*
		 * Each TFC channel has its own control and acts as a pmc set,
		 * the pmc set can also be treated as channel index except the
		 * last pmc set, since the pmc set of TFC driver events are
		 * appended as the last pmc set.
		 */
		unsigned int set:4;
	};
};

#define SET_OFFSET		8
#define CAT_OFFSET		4
#define SID_MASK		0x000f
#define EV_MASK			0x00ff
#define HWC_MASK		0x0fff

#define get_ev_set(eid)	(((eid) & HWC_MASL) >> SET_OFFSET)
#define get_ev_cat(eid)	(((eid) & EV_MASK) >> CAT_OFFSET)
#define get_ev_sid(eid)	((eid) & SID_MASK)
#define EV_ID(eid, cat)	((eid) | (TFC_CAT__##cat << CAT_OFFSET))
#define EV_CONFIG(eid, cat, set)	\
	(EV_ID(eid, cat) | (set << SET_OFFSET))

static inline union rtk_tfc_event_desc
__get_event_desc(u64 config)
{
	union rtk_tfc_event_desc desc = {.val = (unsigned int)config};

	return desc;
}

static inline union rtk_tfc_event_desc
get_event_desc(struct perf_event *event)
{
	return __get_event_desc(event->attr.config);
}

static inline int
__pmc_to_ctrl_bit(union rtk_pmc_desc pmc)
{
	return (pmc.set * TFC_CH_CTRL_BITS) + pmc.idx;
}

static inline int
rtk_tfc_pmc_target(u64 config)
{
	return (int)(config & EV_MASK);
}


struct rtk_tfc_sysfs_attr {
	struct device_attribute attr;
	u64 id;
};

#define TFC_SYSFS_ATTR(_name, _id, _show, _store)		\
(&((struct rtk_tfc_sysfs_attr) {				\
	.attr = __ATTR(_name, 0644, _show, _store),	\
	.id = _id,						\
}).attr.attr)

#define _ESCAPE(...)	__VA_ARGS__
#define TFC_SYSFS_ATTR_GROUP(_name, _id)		_ESCAPE(\
	TFC_SYSFS_ATTR(_name##_event_threshold, _id,		\
		       rtk_tfc_thr_read, rtk_tfc_thr_write),	\
	TFC_SYSFS_ATTR(_name##_read_timeout, _id,		\
		       rtk_tfc_rto_read, rtk_tfc_rto_write),	\
	TFC_SYSFS_ATTR(_name##_write_timeout, _id,		\
		       rtk_tfc_wto_read, rtk_tfc_wto_write))	\

/* fileds of threshold register */
#define TFC_TH_SHIFT		0
#define TFC_TH_MASK		0xffff
#define TFC_TH_RTO_SHIFT	24
#define TFC_TH_WTO_SHIFT	16
#define TFC_TH_TIMEOUT_MASK	0xff

#define TFC_SYS_OPS_DECLARE(name, shift, mask)			\
static inline u32 __rtk_tfc_##name##_read(unsigned long addr)	\
{ return (rtk_readl(addr) >> shift) & mask; }			\
static inline void __rtk_tfc_##name##_write(unsigned long addr, u32 val) \
{ rtk_writel(addr, \
	     (rtk_readl(addr) & ~(mask << shift)) | (val & mask) << shift); }

/* access event threshold from sysfs */
TFC_SYS_OPS_DECLARE(thr, TFC_TH_SHIFT, TFC_TH_MASK);
/* access read-req timeout from sysfs */
TFC_SYS_OPS_DECLARE(rto, TFC_TH_RTO_SHIFT, TFC_TH_TIMEOUT_MASK);
/* access write-req timeout from sysfs */
TFC_SYS_OPS_DECLARE(wto, TFC_TH_WTO_SHIFT, TFC_TH_TIMEOUT_MASK);

ssize_t rtk_tfc_thr_read(struct device *dev, struct device_attribute *attr,
			 char *buf);
ssize_t rtk_tfc_thr_write(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count);
ssize_t rtk_tfc_rto_read(struct device *dev, struct device_attribute *attr,
			 char *buf);
ssize_t rtk_tfc_rto_write(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count);
ssize_t rtk_tfc_wto_read(struct device *dev, struct device_attribute *attr,
			 char *buf);
ssize_t rtk_tfc_wto_write(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count);

struct rtk_pmc_set *rtk_tfc_get_pmc_set(struct rtk_pmu *pmu,
					struct perf_event *event);
int rtk_tfc_check_event(struct rtk_pmu *pmu, struct perf_event *event);
union rtk_pmc_desc rtk_tfc_arrange_pmc(struct rtk_pmu *pmu, u64 config);
int rtk_tfc_ps_init(const struct rtk_pmc_set_meta *meta,
		    struct rtk_pmc_set *ps);
int rtk_tfc_ps_total_init(const struct rtk_pmc_set_meta *meta,
			  struct rtk_pmc_set *ps);
int rtk_tfc_ps_drv_init(const struct rtk_pmc_set_meta *meta,
			struct rtk_pmc_set *ps);
int rtk_tfc_pmu_init(struct rtk_pmu *pmu, struct device_node *dt,
		     const char *name, unsigned long ctrl_offset,
		     const struct attribute_group **attr_groups,
		     const struct rtk_pmc_set_meta *meta, int nr_ps);

extern struct attribute_group rtk_tfc_format_attr_group;

/* Platform specific init functions */
int rtk_16xxb_ddrc_tfc_init(struct rtk_pmu *pmu, struct device_node *dt);

#endif /* End of __RTK_TFC_H__ */
