/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Header of Realtek uncore PMU driver
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

#ifndef __RTK_UNCORE_PMU__
#define __RTK_UNCORE_PMU__

#include <linux/io.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>



#ifdef CONFIG_RTK_PMU_DEBUG
#define dmsg(fmt, ...)		pr_info(fmt, ##__VA_ARGS__)
#else
#define dmsg(...)
#endif


#define RTK_PMU_CPUHP_NAME(_n)	"perf/" _n ":online"


#define RTK_PMU_VAL_MASK(_W)	((1ULL << (_W)) - 1)
#define RTK_PMU_OVERFLOW_TH(_W)	(1ULL << ((_W) - 1))

#define REFRESH_TH     0x00040000      /* refresh threshold, default 256us */

#define RTK_UNCORE_HRTIMER_INTERVAL	(40LL * NSEC_PER_MSEC)
#ifdef CONFIG_RTK_PMU_TICK_REFRESH
#define rtk_pmu_kick_tick()	perf_event_task_tick()
#else
#define rtk_pmu_kick_tick()
#endif

#define RTK_PMU_FORMAT_ATTR(name, format)				\
	PMU_FORMAT_ATTR(name, format)

#define RTK_PMU_FORMAT_REF(name)					\
	(&format_attr_##name.attr)

#define __RTK_PMU_EVENT_ATTR(name, config, show)			\
	PMU_EVENT_ATTR(name, event_attr_##name,	config, show)		\

#define RTK_PMU_EVENT_ATTR(name, config)				\
	__RTK_PMU_EVENT_ATTR(name, config, rtk_pmu_event_show)

#define RTK_PMU_EVENT_REF(name)						\
	(&event_attr_##name.attr.attr)

#define __RTK_PMU_EVENT_ATTR_ITEM(_name, _id, _show)			\
{									\
	.attr = __ATTR(_name, 0444, _show, NULL),                       \
	.id   = _id,							\
}

#define RTK_PMU_EVENT_ATTR_ITEM(name, config)				\
	__RTK_PMU_EVENT_ATTR_ITEM(name, config, rtk_pmu_event_show)

#define RTK_PMU_EVENT_ITEM_REF(name, i)					\
	(&name[i].attr.attr)


extern struct attribute_group rtk_pmu_common_attr_group;

enum {
	RTK_PMU_ATTR_GROUP__COMMON = 0,
	RTK_PMU_ATTR_GROUP__FORMAT,
	RTK_PMU_ATTR_GROUP__EVENT,
	RTK_PMU_ATTR_GROUP__NUM,
};

/* Basic driver events */
enum {
	RTK_DRV_OVERFLOW	= 0,
	RTK_DRV_EV_NUM
};

enum {
	RTK_PMC_NORMAL = 0,		/* normal accumulated counter */

	/* the counter only reflect current state */
	RTK_PMC_STATE,
	/* a wide value consists of multiple short counters */
	RTK_PMC_MIX,
};

enum {
	RTK_PMU_CTRL_UNSET = 0,
	RTK_PMU_CTRL_SET = 1,
};


/*
 * Relations between PMC, PMC group and PMC set.
 * - Counters(PMC) monitoring the same target form a PMC group.
 * - PMC groups monitoring targets located at certain layer forms a PMC set.
 *
 * +-----------+    +-----------+    +-----------+
 * |PMC|PMC|PMC|    |PMC|PMC|PMC|    |PMC|PMC|PMC|
 * +---+---+---+    +---+---+---+    +---+---+---+
 * | PMC group |    | PMC group |    | PMC group |
 * +-----------+----+-----------+----+-----------+
 * |			PMC set			 |
 * +---------------------------------------------+
 */
union rtk_pmc_desc {
	struct {
		int set:8;		/* index of pmc set */
		int idx:8;		/* index of pmc group */
		int usage:8;		/* index of counter within a pmcg */
		int ch:8;		/* channel number */
	};

	int val;
};

#define RTK_INV_PMC_TARGET	(-1)

static inline int
is_valid_pmc(union rtk_pmc_desc pmc)
{
	return pmc.val >= 0;
}

/* raise update before reading pmc */
#define RTK_PMU_NEED_UPDATE_CTRL	0x01

/* Track the use of a PMC group */
struct rtk_pmc_tracking {
	int			target;		/* monitored target */

	/* config, including target, mode, channel...etc */
	int			config;
	atomic_t		active;		/* number of active events */
	unsigned long		used_mask;	/* bitmap for tracking events */
	struct perf_event	**events;	/* events under tracking */
};

struct rtk_pmu;
struct rtk_pmc_set;

struct rtk_pmc_set_meta {
	const char	*name;
	const char	*compatible;
	int		(*init)(const struct rtk_pmc_set_meta *psm,
				struct rtk_pmc_set *ps);

	int		nr_clients;	/* # of clients */
	int		nr_pmcgs;	/* # of pmc group */
	int		nr_configs;	/* # of config regs */

	const unsigned int *clients;	/* clients */
	const unsigned long *pmcgs;	/* offset of pmc group */
	const unsigned long *configs;	/* offset of pmc config regsisters */

	int		type;
	int		group_size;	/* size of PMC group */

	/* # of PMC groups controlled by one config reg */
	int		config_size;
	int		config_width;	/* bit width of a PMC in a config reg */

	/*
	 * In some version of Realtek PMU implementation, the counter length is
	 * various, we need some information to extract the value.
	 */
	const unsigned int *val_mask;	/* bitmask array to get counter value */
	const unsigned int *ov_th;	/* overflow array threshold array */
};

#define RTK_PMU_META_NR(_m)		(ARRAY_SIZE(_m) - 1)

struct rtk_pmc_set {
	void __iomem	*base;		/* base addr of counters */
	const char	*name;		/* name of pmc set */

	struct rtk_pmu	*pmu;		/* pointer to pmu */
	const struct rtk_pmc_set_meta *meta;
	int		type;		/* PMC set type */

	raw_spinlock_t	ps_lock;
	unsigned long	used_mask;	/* bitmap for used pmcg */

	u32		nr_defects;	/* # of unusable pmc group */
	u32		*defects;	/* index array of defected pmc */
	u32		has_ext;	/* has extend data */
	u32		ext_info;

	/* Events tracking by a pmc set */
	struct rtk_pmc_tracking	*tracking;

	/* Enable/disable PMU set */
	void			(*enable)(struct rtk_pmc_set *ps,
					  struct perf_event *ev);
	void			(*disable)(struct rtk_pmc_set *ps,
					   struct perf_event *ev);
	void			(*reset)(struct rtk_pmc_set *ps,
					 struct perf_event *ev);

	/* Translate event config to monitored target */
	int			(*pmc_config)(u64 config);

	/* Arrange pmc for event */
	int			(*arrange_pmc)(struct rtk_pmc_set *ps,
					       int config, int target);
	int			(*release_pmc)(struct rtk_pmc_set *ps,
					       union rtk_pmc_desc pmc);
	void			(*set_perf_hwc)(struct rtk_pmc_set *ps,
						union rtk_pmc_desc pmc,
						struct perf_event *ev);
	int			(*start_pmc)(struct rtk_pmc_set *ps,
					     struct perf_event *ev);
	int			(*stop_pmc)(struct rtk_pmc_set *ps,
					    struct perf_event *ev);
	void			(*read_pmc)(struct rtk_pmc_set *ps,
					    struct perf_event *ev);
	void			(*extra_config)(struct rtk_pmc_set *ps,
						struct perf_event *ev,
						int op);
	u32			(*read_counter)(struct rtk_pmc_set *ps,
						struct perf_event *ev);
};

struct rtk_pmu {
	struct platform_device	*pdev;
	struct pmu		pmu;
	cpumask_t		cpus;			/* active cpus */

	/* cpuhp state for updating cpus */
	enum cpuhp_state	cpuhp_state;

	/* cpu responsible for uncore events */
	int			cpu;

	/* hlist node for cpu hot-plug */
	struct hlist_node	cpuhp;

	/* cpuhp display name */
	const char		*cpuhp_name;

	/* base addr of counters */
	void __iomem		*base;
	const char		*name;
	unsigned long		ctrl;			/* control register */
	int			irq;			/* irq number */
	u32			flags;			/* PMU attribute flags */

	/* Must be a Null-terminated array */
	const struct attribute_group **attr_groups;

	struct rtk_pmc_set	**pmcss;		/* pmc set */
	int			nr_pmcss;

	int			nr_active;		/* active events */
	struct hrtimer		hrtimer;
	u64			hrtimer_interval;	/* hrtimer interval */

	atomic_t		*drv_pmc;		/* driver statistics */

	/* Enable/disable PMU set */
	void			(*enable)(struct rtk_pmu *pmu,
					  struct perf_event *event);
	void			(*disable)(struct rtk_pmu *pmu,
					   struct perf_event *event);
	void			(*clear)(struct rtk_pmu *pmu,
					 struct perf_event *event);

	/*
	 * Refresh callback. RTK performance counters may stuck at 0xFFFFFFFF,
	 * it needs to reset the PMU to fix this.
	 */
	void			(*refresh)(struct rtk_pmu *pmu,
					   struct perf_event *event);

	/* Get pmc set from event config */
	struct rtk_pmc_set *	(*get_pmc_set)(struct rtk_pmu *pmu,
					       struct perf_event *event);

	/* Check event validity */
	int			(*is_valid_event)(struct rtk_pmu *pmu,
						  struct perf_event *event);

	union rtk_pmc_desc	(*arrange_pmc)(struct rtk_pmu *pmu, u64 config);
};

#define to_rtk_pmu(_pmu)	(container_of(_pmu, struct rtk_pmu, pmu))


static inline u32 rtk_readl(unsigned long addr)
{
	return readl((void *)addr);
}

static inline void rtk_writel(unsigned long addr, unsigned int val)
{
	writel(val, (void *)addr);
}

ssize_t rtk_pmu_event_show(struct device *dev, struct device_attribute *attr,
			   char *buf);
ssize_t rtk_pmu_ctrl_show(struct device *dev, struct device_attribute *attr,
			  char *buf);
ssize_t rtk_pmu_ctrl_store(struct device *dev, struct device_attribute *attr,
			   char *buf);

/*
 * Generic APIs for accessing hw config data
 */
static inline union rtk_pmc_desc
rtk_event_pmc_desc(struct perf_event *event)
{
	union rtk_pmc_desc pmc = {.val = event->hw.idx};

	return pmc;
}

static inline void
rtk_event_set_pmc_desc(struct hw_perf_event *hwc, int val)
{
	hwc->idx = val;
}

static inline unsigned long
rtk_event_config_addr(struct perf_event *event)
{
	return event->hw.config_base;
}

static inline void
rtk_event_set_config_addr(struct hw_perf_event *hwc, unsigned long addr)
{
	hwc->config_base = addr;
}

static inline unsigned long
rtk_event_pmc_addr(struct perf_event *event)
{
	return event->hw.event_base;
}

static inline void
rtk_event_set_pmc_addr(struct hw_perf_event *hwc, unsigned long addr)
{
	hwc->event_base = addr;
}

static inline u64
rtk_event_config(struct perf_event *event)
{
	return event->hw.config;
}

static inline void
rtk_event_set_config(struct hw_perf_event *hwc, u64 val)
{
	hwc->config = val;
}

static inline int
rtk_event_target_mask(struct perf_event *event)
{
	return event->hw.event_base_rdpmc;
}

static inline void
rtk_event_set_target_mask(struct hw_perf_event *hwc, int mask)
{
	hwc->event_base_rdpmc = mask;
}

static inline unsigned
rtk_event_target(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;

	return (hwc->config & hwc->event_base_rdpmc);
}

static inline int
rtk_event_pmc_threshold(struct perf_event *event)
{
	return event->hw.last_tag;
}

static inline void
rtk_event_set_pmc_threshold(struct hw_perf_event *hwc, int val)
{
	hwc->last_tag = val;
}

static inline unsigned int
rtk_event_pmc_mask(struct perf_event *event)
{
	return event->hw.last_cpu;
}

static inline void
rtk_event_set_pmc_mask(struct hw_perf_event *hwc, int val)
{
	hwc->last_cpu = val;
}

static inline int
rtk_event_pmc_type(struct perf_event *event)
{
	return event->hw.flags;
}

static inline void
rtk_event_set_pmc_type(struct hw_perf_event *hwc, int val)
{
	hwc->flags = val;
}

static inline int
rtk_event_pmc_offset(struct perf_event *event)
{
	return event->hw.extra_reg.idx;
}

static inline void
rtk_event_set_pmc_offset(struct hw_perf_event *hwc, int val)
{
	hwc->extra_reg.idx = val;
}

static inline u64
rtk_event_pmc_ts(struct perf_event *event)
{
	return event->hw.extra_reg.config;
}

static inline void
rtk_event_set_pmc_ts(struct hw_perf_event *hwc, u64 val)
{
	hwc->extra_reg.config = val;
}

static inline int
rtk_event_pmc_sample(struct perf_event *event)
{
	return event->hw.extra_reg.reg;
}

/* tracking the sample count */
static inline void
rtk_event_inc_pmc_sample(struct perf_event *event)
{
	event->hw.extra_reg.reg++;
}

/* the register control PMC update */
static inline void
rtk_event_set_pmc_update_ctrl(struct hw_perf_event *hwc,
			     unsigned long addr)
{
	hwc->branch_reg.config = addr;
}

static inline unsigned long
rtk_event_pmc_update_ctrl(struct perf_event *event)
{
	return event->hw.branch_reg.config;
}

/* the value trigger PMC update */
static inline void
rtk_event_set_pmc_update(struct hw_perf_event *hwc, int val)
{
	hwc->branch_reg.idx = val;
}

static inline int
rtk_event_pmc_update(struct perf_event *event)
{
	return event->hw.branch_reg.idx;
}

static inline struct rtk_pmc_tracking *
get_pmc_tracking(struct rtk_pmu *pmu, union rtk_pmc_desc pmc)
{
	return &pmu->pmcss[pmc.set]->tracking[pmc.idx];
}

#ifdef CONFIG_RTK_PMU_GATOR_COMPATIBLE
static inline int
pre_check_event_validity(struct rtk_pmu *pmu, struct perf_event *event)
{
	return true;
}

static inline int
post_check_event_validity(struct rtk_pmu *pmu, struct perf_event *event)
{
	return pmu->is_valid_event(pmu, event);
}
#else
static inline int
pre_check_event_validity(struct rtk_pmu *pmu, struct perf_event *event)
{
	return pmu->is_valid_event(pmu, event);
}

static inline int
post_check_event_validity(struct rtk_pmu *pmu, struct perf_event *event)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);

	return pmc.val >= 0 &&
		pmc.set < pmu->nr_pmcss &&
		pmc.idx < pmu->pmcss[pmc.set]->meta->nr_pmcgs &&
		pmc.usage < pmu->pmcss[pmc.set]->meta->group_size;
}
#endif

/*
 * Generic APIs for PMC set
 */
void rtk_ps_refresh_pmcg(struct rtk_pmc_set *ps, struct perf_event *event);
void rtk_ps_refresh_pmcg_done(struct rtk_pmc_set *ps, struct perf_event *event);
void rtk_ps_refresh(struct rtk_pmc_set *ps);
void rtk_ps_refresh_done(struct rtk_pmc_set *ps);

int rtk_ps_track_event(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
		       struct perf_event *event);

int rtk_ps_untrack_event(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
			 struct perf_event *event);

int rtk_ps_find_pmc(struct rtk_pmc_set *ps, int hwc, int target);

int rtk_ps_release_pmc(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc);

void rtk_ps_read_pmc(struct rtk_pmc_set *ps, struct perf_event *event);
void rtk_ps_read_pmc_with_update(struct rtk_pmc_set *ps,
				 struct perf_event *event);
int rtk_ps_stop_pmc(struct rtk_pmc_set *ps, struct perf_event *event);
int rtk_ps_start_pmc(struct rtk_pmc_set *ps, struct perf_event *event);

u32 rtk_read_counter(struct rtk_pmc_set *ps, struct perf_event *event);
u32 rtk_read_counter_with_update(struct rtk_pmc_set *ps,
				 struct perf_event *event);

/*
 * APIs for accessing driver SW PMCs
 */
int rtk_find_drv_pmc(struct rtk_pmc_set *ps, int hwc, int target);
void rtk_set_drv_hwc(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
		     struct perf_event *event);
int rtk_start_drv_pmc(struct rtk_pmc_set *ps, struct perf_event *event);
int rtk_stop_drv_pmc(struct rtk_pmc_set *ps, struct perf_event *event);
void rtk_read_drv_pmc(struct rtk_pmc_set *ps, struct perf_event *event);

static inline u64
rtk_pmu_drv_pmc_read(struct rtk_pmu *pmu, u64 idx)
{
	return atomic_read(&pmu->drv_pmc[idx]);
}

static inline void
rtk_pmu_drv_pmc_inc(struct rtk_pmu *pmu, u64 idx)
{
	atomic_inc(&pmu->drv_pmc[idx]);
}

static inline void
compensate_event(struct rtk_pmc_set *ps, struct perf_event *ev)
{
	if (ev) {
		rtk_pmu_drv_pmc_inc(ps->pmu, RTK_DRV_OVERFLOW);
		local64_add((unsigned long)rtk_event_pmc_mask(ev) + 1,
			    &ev->count);
	}
}

/*
 * Generic APIs for PMU
 */
static inline int
rtk_pmu_has_no_overflow(struct rtk_pmu *pmu)
{
	return pmu->pmu.capabilities & PERF_PMU_CAP_NO_INTERRUPT;
}

static inline int
rtk_pmu_need_hrtimer(struct rtk_pmu *pmu)
{
	return IS_ENABLED(CONFIG_RTK_PMU_TICK_REFRESH) &&
		rtk_pmu_has_no_overflow(pmu);
}

static inline int
rtk_pmu_need_update_ctrl(struct rtk_pmu *pmu)
{
	return pmu->flags & RTK_PMU_NEED_UPDATE_CTRL;
}

int rtk_pmu_track_event(struct perf_event *event);
int rtk_pmu_untrack_event(struct perf_event *event);
u32 rtk_read_counter(struct rtk_pmc_set *ps, struct perf_event *event);
u64 rtk_pmu_update_event(struct rtk_pmc_set *ps, struct perf_event *event);
int rtk_pmc_set_init(struct rtk_pmu *pmu, struct device_node *dt,
		     const struct rtk_pmc_set_meta *psm);
int rtk_uncore_pmu_init(struct rtk_pmu *pmu, struct device_node *dt,
			const char *name, const char *cpuhp_name,
			unsigned long ctrl_offset,
			const struct attribute_group **attr_groups,
			const struct rtk_pmc_set_meta *meta, int nr_ps,
			int drv_cnts);

int rtk_pmu_device_probe(struct platform_device *pdev,
			 const struct of_device_id *of_table);
int rtk_pmu_device_remove(struct platform_device *pdev);

#endif /* End of __RTK_UNCORE_PMU__ */
