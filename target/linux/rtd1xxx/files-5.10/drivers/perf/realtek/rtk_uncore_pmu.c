// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek uncore PMU driver
 *
 * Copyright (C) 2019 Realtek Semiconductor Corporation
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

#define pr_fmt(fmt)		"[RTK_PMU] " fmt

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/bitops.h>

#include "rtk_uncore_pmu.h"


struct buf_ctrl {
	char *buf;
	char *tail;
	size_t size;
	size_t left;
};

static inline void
init_buf_ctrl(struct buf_ctrl *bc, char *buf, size_t size)
{
	bc->buf = buf;
	bc->tail = buf;
	bc->size = size;
	bc->left = size;
}

static int
push_buf(struct buf_ctrl *bc, const char *fmt, ...)
{
	va_list ap;
	int used = 0;

	/* the string buffer should be NULL-terminated */
	if (bc->left > 1) {
		va_start(ap, fmt);
		used = vsnprintf(bc->tail, bc->left, fmt, ap);
		if (used >= bc->left)
			used = bc->left - 1;

		bc->left -= used;
		bc->tail += used;
		va_end(ap);
	}

	return used;
}

static int
pop_buf(struct buf_ctrl *bc, size_t size)
{
	if ((bc->left + size) <= bc->size) {
		bc->left += size;
		bc->tail -= size;
	}

	return size;
}

#define SELF(_obj)		(_obj)
#define MEMBER(_obj, _mem)	(_obj._mem)
#define REG(_obj, _base)	(((unsigned long)_base + _obj))
#define GET_REG(_obj, _base)	(rtk_readl((unsigned long)_base + _obj))

#define OUT_INFO(_title, _bc, _size, _data, _p, ...)	\
do { \
	int iter; \
	push_buf(_bc, "\t%s[%d]\t= {", _title, _size); \
	for (iter = 0; iter < _size; iter++) \
		push_buf(_bc, "%#x, ", _p(_data[iter], ##__VA_ARGS__)); \
	pop_buf(_bc, 2); \
	push_buf(_bc, "}\n"); \
} while (0)

static void
__pmcs_show(struct buf_ctrl *bc, const struct rtk_pmc_set *ps)
{
	push_buf(bc, "%s, iobase:%lx\n", ps->name, ps->base);

	if (ps->meta->nr_clients)
		OUT_INFO("clients", bc, ps->meta->nr_clients, ps->meta->clients,
			 SELF);

	if (ps->meta->nr_pmcgs)
		OUT_INFO("tracking", bc, ps->meta->nr_pmcgs, ps->tracking,
			 MEMBER, config);

	if (ps->meta->nr_configs) {
		if (IS_ENABLED(CONFIG_RTK_PMU_DEV))
			OUT_INFO("config_regs", bc, ps->meta->nr_configs,
				 ps->meta->configs, REG, ps->base);
		OUT_INFO("configs", bc, ps->meta->nr_configs, ps->meta->configs,
			 GET_REG, ps->base);
	}

	push_buf(bc, "\n");
}

static ssize_t
info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rtk_pmu *pmu = to_rtk_pmu(dev_get_drvdata(dev));
	struct buf_ctrl ob;
	int i;

	init_buf_ctrl(&ob, buf, PAGE_SIZE);

	for (i = 0; i < pmu->nr_pmcss && pmu->pmcss[i] != NULL; i++)
		__pmcs_show(&ob, pmu->pmcss[i]);

	return ob.size - ob.left;
}

static ssize_t
cpumask_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rtk_pmu *pmu = to_rtk_pmu(dev_get_drvdata(dev));

	return cpumap_print_to_pagebuf(true, buf, &pmu->cpus);
}

static DEVICE_ATTR_RO(cpumask);
static DEVICE_ATTR_RO(info);

static struct attribute *rtk_pmu_common_attrs[] = {
	&dev_attr_cpumask.attr,
	&dev_attr_info.attr,
	NULL,
};

struct attribute_group rtk_pmu_common_attr_group = {
	.attrs = rtk_pmu_common_attrs,
};
EXPORT_SYMBOL(rtk_pmu_common_attr_group);

ssize_t
rtk_pmu_event_show(struct device *dev,
		   struct device_attribute *attr, char *buf)
{
	struct perf_pmu_events_attr *pmu_attr =
		container_of(attr, struct perf_pmu_events_attr, attr);

	return scnprintf(buf, PAGE_SIZE, "event=0x%04lx\n",
			 (unsigned long)pmu_attr->id);
}
EXPORT_SYMBOL(rtk_pmu_event_show);

void
rtk_ps_refresh_pmcg(struct rtk_pmc_set *ps, struct perf_event *event)
{
	struct rtk_pmc_tracking *tracking = ps->tracking;
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	int bit;

	for_each_set_bit(bit,
			 &tracking[pmc.idx].used_mask,
			 ps->meta->group_size) {

		event = tracking[pmc.idx].events[bit];
		rtk_pmu_update_event(ps, event);
		/*
		 * FIXME: If using the state to block the read
		 * operation, should it be atomic?
		 */
		event->hw.state |= (PERF_HES_ARCH | PERF_HES_UPTODATE);

		/*
		 * Reset prev_count since the hw counter is reset after
		 * re-enable PMU.
		 */
		local64_set(&event->hw.prev_count, 0);

		dmsg("%s- refresh pmcg %#x:%#x:%#x, events:%#llx\n",
		     ps->pmu->name, ps->type, pmc.idx, bit,
		     event->attr.config);
	}
}
EXPORT_SYMBOL(rtk_ps_refresh_pmcg);

void
rtk_ps_refresh_pmcg_done(struct rtk_pmc_set *ps, struct perf_event *event)
{
	struct rtk_pmc_tracking *tracking = ps->tracking;
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	int bit;

	for_each_set_bit(bit,
			 &tracking[pmc.idx].used_mask,
			 ps->meta->group_size)
		tracking[pmc.idx].events[bit]->hw.state &=
		~(PERF_HES_ARCH | PERF_HES_UPTODATE);
}
EXPORT_SYMBOL(rtk_ps_refresh_pmcg_done);

void
rtk_ps_refresh(struct rtk_pmc_set *ps)
{
	struct rtk_pmc_tracking *tracking = ps->tracking;
	struct perf_event *event;
	int idx, bit;

	if (ps == NULL)
		return;

	for_each_set_bit(idx, &ps->used_mask, ps->meta->nr_pmcgs) {
		for_each_set_bit(bit,
				 &tracking[idx].used_mask,
				 ps->meta->group_size) {

			event = tracking[idx].events[bit];
			rtk_pmu_update_event(ps, event);
			/*
			 * FIXME: If using the state to block the read
			 * operation, should it be atomic?
			 */
			event->hw.state |= (PERF_HES_ARCH | PERF_HES_UPTODATE);

			/*
			 * Reset prev_count since the hw counter is reset after
			 * re-enable PMU.
			 */
			local64_set(&event->hw.prev_count, 0);

			dmsg("%s- refresh pmc %#x:%#x:%#x, events:%#llx\n",
			     ps->pmu->name, ps->type, idx, bit,
			     event->attr.config);
		}
	}
}
EXPORT_SYMBOL(rtk_ps_refresh);

void
rtk_ps_refresh_done(struct rtk_pmc_set *ps)
{
	struct rtk_pmc_tracking *tracking = ps->tracking;
	int idx, bit;

	for_each_set_bit(idx, &ps->used_mask, ps->meta->nr_pmcgs) {
		for_each_set_bit(bit,
				 &tracking[idx].used_mask,
				 ps->meta->group_size)
			tracking[idx].events[bit]->hw.state &=
			~(PERF_HES_ARCH | PERF_HES_UPTODATE);
	}
}
EXPORT_SYMBOL(rtk_ps_refresh_done);

static inline int
__search_tracking(struct rtk_pmc_set *ps, int hwc, int target)
{
	int idx;

	/*
	 * Since each pmc group can track multiple events, try to track event
	 * using in-used pmc group.
	 */
	for_each_set_bit(idx, &ps->used_mask, ps->meta->nr_pmcgs) {
		if (ps->tracking[idx].config == hwc)
			return idx;
	}

	return -1;
}

static bool
__is_defect(struct rtk_pmc_set *ps, int idx)
{
	unsigned int i;

	if (!ps->defects)
		return false;

	for (i = 0; i < ps->nr_defects; i++) {
		if (ps->defects[i] == idx)
			return true;
	}

	return false;
}

static int
__alloc_tracking(struct rtk_pmc_set *ps, int hwc, int target)
{
	int idx;

	for (idx = 0; idx < ps->meta->nr_pmcgs; idx++) {
		if (!test_and_set_bit(idx, &ps->used_mask) &&
		    !__is_defect(ps, idx)) {
			ps->tracking[idx].target = target;
			ps->tracking[idx].config = hwc;

			return idx;
		}
	}

	return RTK_INV_PMC_TARGET;
}

int
rtk_ps_find_pmc(struct rtk_pmc_set *ps, int hwc, int target)
{
	int idx;

	idx = __search_tracking(ps, hwc, target);
	if (idx < 0)
		idx = __alloc_tracking(ps, hwc, target);

	return idx;
}
EXPORT_SYMBOL(rtk_ps_find_pmc);

void
rtk_ps_read_pmc(struct rtk_pmc_set *ps, struct perf_event *event)
{
	struct rtk_pmu *pmu = to_rtk_pmu(event->pmu);
	u64 curr;

	/*
	 * PMC has been stopped and being refreshing, since the value is
	 * up-to-date, just skip the event updating.
	 */
	if (event->hw.state & PERF_HES_ARCH) {
		if (IS_ENABLED(CONFIG_RTK_PMU_DEV))
			pr_notice("%s- PMC read skip, event state %x\n",
				  pmu->name, event->hw.state);
		return;
	}

	curr = rtk_pmu_update_event(ps, event);

	/*
	 * RTK PMCs stuck at 0xFFFFFFFF instead of recounting from 0 if it is
	 * incapable of firing overflow interrupt. Thus, if a counter has
	 * counted half amount events, update all active events and reset the
	 * PMU.
	 *
	 * FIXME: A PMC group stops counting if the Accumulated latency
	 * overflows. But we nerver know that if the accumulated latency is not
	 * monitored.
	 */
	if (rtk_pmu_has_no_overflow(pmu) &&
	    unlikely(curr & rtk_event_pmc_threshold(event))) {
		if (IS_ENABLED(CONFIG_RTK_PMU_DEV))
			pr_notice("%s- Overflow protect: ev:%#llx, val:%#llx\n",
				  pmu->name, event->attr.config, curr);

		if (pmu->refresh)
			pmu->refresh(pmu, event);
	}
}
EXPORT_SYMBOL(rtk_ps_read_pmc);

int
rtk_ps_stop_pmc(struct rtk_pmc_set *ps, struct perf_event *event)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	struct rtk_pmc_tracking *tracking = get_pmc_tracking(ps->pmu, pmc);
	int target_mask = rtk_event_target_mask(event);
	unsigned int config = 0;

	if (ps->meta->nr_configs) {
		config = rtk_readl(rtk_event_config_addr(event));

		if (IS_ENABLED(CONFIG_RTK_PMU_DEV)) {
			unsigned int target = config & target_mask;

			WARN(target != rtk_event_target(event),
			     "%s- Remove event mis-match %#x:%#x\n",
			     ps->pmu->name, target, rtk_event_target(event));
		}
	}

	/*
	 * Clear the config register only if all events of the corresponding
	 * PMCG are stopped.
	 */
	if (atomic_dec_return(&tracking->active) == 0) {
		if (ps->meta->nr_configs) {
			config &= ~target_mask;

			/*
			 * This is a dirty-hack for support legacy Rbus PMU, in
			 * which a write_enable must be written together with
			 * the config.
			 */
			config |= (rtk_event_config(event) & ~target_mask);

			rtk_writel(rtk_event_config_addr(event), config);
		}

		if (ps->extra_config)
			ps->extra_config(ps, event, RTK_PMU_CTRL_UNSET);
	}

	if (ps->disable)
		ps->disable(ps, event);

	/*
	 * The counter is supposed to be stopped but Realtek uncore PMU does not
	 * support fine-grain control (all counters are enabled and disabled
	 * together).
	 * Since a stopped event is not updated from the counter, in order to
	 * avoid interfering with other counters by restarting the PMU, just let
	 * all PMCs run free.
	 */

	return 0;
}
EXPORT_SYMBOL(rtk_ps_stop_pmc);

int
rtk_ps_start_pmc(struct rtk_pmc_set *ps, struct perf_event *event)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	struct rtk_pmc_tracking *tracking = get_pmc_tracking(ps->pmu, pmc);
	struct hw_perf_event *hwc = &event->hw;
	unsigned int config = 0;

	dmsg("%s- start ev:%#llx, pmc:%#x, count:%ld\n",
	     ps->pmu->name, event->attr.config, event->hw.idx,
	     local64_read(&event->count));

	if (ps->meta->nr_configs)
		config = rtk_readl(rtk_event_config_addr(event));

	/*
	 * FIXME: Simple reset the prev_count may make aggregated events
	 * imprecise. Consider the situation, a PMC was enabled by a task ran
	 * for a while, another task monitors other event of the same event
	 * group. Aggregating the latter event with init prev_count=0 makes
	 * the event behaves like starting with the prior task.
	 */
	local64_set(&hwc->prev_count, 0);

	if (atomic_inc_return(&tracking->active) == 1) {
		dmsg("%s- reset PMC set: %s, pmc config: %llx\n",
		     ps->pmu->name, ps->name, config | rtk_event_config(event));

		if (ps->meta->nr_configs) {
			/* clear origin config */
			config &= ~rtk_event_target_mask(event);
			config |= rtk_event_config(event);
			rtk_writel(rtk_event_config_addr(event), config);
		}

		if (ps->extra_config)
			ps->extra_config(ps, event, RTK_PMU_CTRL_SET);

		/*
		 * uncomment following for initiating event with current counter
		 * value.
		 *
		 * local64_set(&hwc->prev_count, 0);
		 */

		/*
		 * Realtek uncore PMU does not support run-time re-config, the
		 * PMU must be restarted to take new config. Also, all PMCs will
		 * be cleared after reset, leading data lose. Currently, RTK PMU
		 * suffers from event interference, please NOT use RTK PMU with
		 * frequently event insertion;
		 */
		if (ps->reset)
			ps->reset(ps, event);
	} else {
		if (IS_ENABLED(CONFIG_RTK_PMU_DEV))
			/* check validity of event aggregation */
			WARN(ps->meta->nr_configs &&
			     ((config & rtk_event_target_mask(event)) !=
			      rtk_event_target(event)),
			     "%s- aggregated event mis-match %x:%x\n",
			     ps->pmu->name,
			     config & rtk_event_target_mask(event),
			     rtk_event_target(event));

		/*
		 * uncomment following for initiating aggregated event with
		 * current counter value.
		 *
		 * local64_set(&hwc->prev_count, rtk_read_counter(ps, event));
		 */
	}

	if (ps->enable)
		ps->enable(ps, event);

	return 0;
}
EXPORT_SYMBOL(rtk_ps_start_pmc);

int
rtk_ps_release_pmc(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc)
{
	struct rtk_pmc_tracking *tracking = &ps->tracking[pmc.idx];

	if (tracking->used_mask != 0x00)
		return -EBUSY;

	dmsg("%s- release pmc %#x:%#x\n", ps->pmu->name, pmc.set, pmc.idx);

	tracking->target = RTK_INV_PMC_TARGET;
	tracking->config = RTK_INV_PMC_TARGET;

	if (!test_and_clear_bit(pmc.idx, &ps->used_mask)) {
		pr_err("%s- pmc %#x:%#x already released\n",
		       ps->pmu->name, pmc.set, pmc.idx);

		return -EAGAIN;
	}

	return 0;
}
EXPORT_SYMBOL(rtk_ps_release_pmc);

int
rtk_ps_track_event(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
		   struct perf_event *event)
{
	struct rtk_pmc_tracking *tracking = &ps->tracking[pmc.idx];

	/* Is the event already under tracking? */
	if (test_and_set_bit(pmc.usage, &tracking->used_mask))
		return -EBUSY;

	tracking->events[pmc.usage] = event;

	return 0;
}
EXPORT_SYMBOL(rtk_ps_track_event);

int
rtk_ps_untrack_event(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
		     struct perf_event *event)
{
	struct rtk_pmc_tracking *tracking = &ps->tracking[pmc.idx];

	if (WARN_ON(tracking->events[pmc.usage] != event))
		return -EINVAL;

	tracking->events[pmc.usage] = NULL;

	if (!test_and_clear_bit(pmc.usage, &tracking->used_mask)) {
		pr_err("%s- ev:%#llx already been removed\n",
		       ps->pmu->name, event->attr.config);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(rtk_ps_untrack_event);

int
rtk_pmu_track_event(struct perf_event *event)
{
	struct rtk_pmu *pmu = to_rtk_pmu(event->pmu);
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);

	return rtk_ps_track_event(ps, pmc, event);
}

int
rtk_pmu_untrack_event(struct perf_event *event)
{
	struct rtk_pmu *pmu = to_rtk_pmu(event->pmu);
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);

	return rtk_ps_untrack_event(ps, pmc, event);
}

u32 rtk_read_counter(struct rtk_pmc_set *ps, struct perf_event *event)
{
	return rtk_readl(rtk_event_pmc_addr(event));
}
EXPORT_SYMBOL(rtk_read_counter);

u32 rtk_read_counter_with_update(struct rtk_pmc_set *ps,
					struct perf_event *event)
{
	rtk_writel(rtk_event_pmc_update_ctrl(event),
		   rtk_event_pmc_update(event));

	return rtk_readl(rtk_event_pmc_addr(event));
}
EXPORT_SYMBOL(rtk_read_counter_with_update);

u64
rtk_pmu_update_event(struct rtk_pmc_set *ps, struct perf_event *event)
{
	u64 now __maybe_unused = ktime_get_mono_fast_ns();
	struct hw_perf_event *hwc = &event->hw;
	u64 delta = 0, prev_raw_count, new_raw_count;

	if (rtk_pmu_need_update_ctrl(ps->pmu)) {
		prev_raw_count = 0;
		new_raw_count = ps->read_counter(ps, event);
	} else {
		do {
			prev_raw_count = local64_read(&hwc->prev_count);
			new_raw_count = ps->read_counter(ps, event);
		} while (local64_cmpxchg(&hwc->prev_count, prev_raw_count,
					 new_raw_count) != prev_raw_count);
	}

	if (rtk_pmu_has_no_overflow(ps->pmu) &&
	    unlikely(new_raw_count < prev_raw_count) &&
	    rtk_event_pmc_threshold(event) &&
	    rtk_event_pmc_type(event) == RTK_PMC_NORMAL) {
		/*
		 * rollback hwc to avoid value pollution from underflow counters
		 */
		local64_set(&event->hw.prev_count, prev_raw_count);
		rtk_pmu_drv_pmc_inc(ps->pmu, RTK_DRV_OVERFLOW);

		if (IS_ENABLED(CONFIG_RTK_PMU_DEV)) {
			u64 interval = now - rtk_event_pmc_ts(event);

			pr_notice("** %s- ev:%#llx overflow, %dth sample\n",
				  ps->pmu->name, event->attr.config,
				  rtk_event_pmc_sample(event));
			pr_notice("\tinterval:%llu, prev/new:%llu/%llu\n",
				  interval, prev_raw_count, new_raw_count);
		}
	} else {
		switch (rtk_event_pmc_type(event)) {
		case RTK_PMC_NORMAL:
			delta = (new_raw_count - prev_raw_count) &
				rtk_event_pmc_mask(event);
			break;
		case RTK_PMC_STATE:
			delta = (new_raw_count >> rtk_event_pmc_offset(event)) &
				rtk_event_pmc_mask(event);
			break;
		case RTK_PMC_MIX:
			delta = ((new_raw_count >> rtk_event_pmc_offset(event))
				 & rtk_event_pmc_mask(event)) -
				((prev_raw_count >> rtk_event_pmc_offset(event))
				 & rtk_event_pmc_mask(event));
			break;
		}

		local64_add(delta, &event->count);
	}

	if (IS_ENABLED(CONFIG_RTK_PMU_DEV)) {
		rtk_event_set_pmc_ts(hwc, now);
		rtk_event_inc_pmc_sample(event);
	}

	return new_raw_count;
}

static void
__reset_hwc(struct rtk_pmu *pmu, struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;

	/*
	 * We don't assign an index until we actually place the event onto
	 * hardware. Use -1 to signify that we haven't decided where to put it
	 * yet.
	 */
	rtk_event_set_pmc_desc(hwc, -1);	/* index of activated pmc */
	rtk_event_set_config(hwc, 0);		/* config value */
	rtk_event_set_config_addr(hwc, 0);	/* control register address */
	rtk_event_set_pmc_addr(hwc, 0);		/* pmc address */
	rtk_event_set_target_mask(hwc, 0);	/* bit-offset of control register */

	rtk_event_set_pmc_mask(hwc, 0);		/* Bitmask of valid value */
	rtk_event_set_pmc_threshold(hwc, 0);	/* Overflow threshold */
}

static void
rtk_event_destroy(struct perf_event *event)
{
	if (IS_ENABLED(CONFIG_RTK_PMU_DEV) &&
	    IS_ENABLED(CONFIG_RTK_PMU_DEBUG)) {
		u64 count, prev_count;

		count = local64_read(&event->count);
		prev_count = local64_read(&event->hw.prev_count);

		/*
		 * Since we don't have IRQ for an event, nothing should be done
		 * here.
		 */
		dmsg("%s- ev:%#llx is destroyed, count:%lld, hwc:%lld\n",
		     event->pmu->name, event->attr.config, count, prev_count);
	}
}

static void
__ps_update_events(struct rtk_pmc_set *ps)
{
	struct rtk_pmc_tracking *tracking = ps->tracking;
	int idx, bit;

	if (ps == NULL)
		return;

	for_each_set_bit(idx, &ps->used_mask, ps->meta->nr_pmcgs) {
		for_each_set_bit(bit,
				 &tracking[idx].used_mask,
				 ps->meta->group_size) {

			ps->read_pmc(ps, tracking[idx].events[bit]);
		}
	}
}

static inline void
rtk_pmu_start_hrtimer(struct rtk_pmu *pmu)
{
	hrtimer_start(&pmu->hrtimer, ns_to_ktime(pmu->hrtimer_interval),
		      HRTIMER_MODE_REL_PINNED);
}

static inline void
rtk_pmu_stop_hrtimer(struct rtk_pmu *pmu)
{
	hrtimer_cancel(&pmu->hrtimer);
}

static enum hrtimer_restart
rtk_pmu_hrtimer(struct hrtimer *hrtimer)
{
	struct rtk_pmu *pmu;
	unsigned long flags;
	int i;

	pmu = container_of(hrtimer, struct rtk_pmu, hrtimer);
	if (!pmu->nr_active || pmu->cpu != smp_processor_id())
		return HRTIMER_NORESTART;

	/*
	 * disable local interrupt to prevent event_start/stop to interrupt the
	 * update process
	 */
	local_irq_save(flags);

	if (rtk_pmu_has_no_overflow(pmu)) {
		for (i = 0; i < pmu->nr_pmcss; i++)
			__ps_update_events(pmu->pmcss[i]);
	}

	rtk_pmu_kick_tick();
	local_irq_restore(flags);

	hrtimer_forward_now(hrtimer, ns_to_ktime(pmu->hrtimer_interval));

	return HRTIMER_RESTART;
}

static int
rtk_pmu_event_init(struct perf_event *event)
{
	struct rtk_pmu *pmu = to_rtk_pmu(event->pmu);
	int err = 0;

	dmsg("%s- try init ev:%#llx cpu-%d\n",
	     pmu->name, event->attr.config, event->cpu);

	/* only handle own events */
	if (event->attr.type != event->pmu->type)
		return -ENOENT;

	/*
	 * Uncore PMU is shared by all cores. It does not support per-process
	 * and sampling mode.
	 */
	if (is_sampling_event(event) || event->attach_state & PERF_ATTACH_TASK)
		return -EOPNOTSUPP;

	/* not yet support filter */
	if (event->attr.exclude_user	||
	    event->attr.exclude_kernel	||
	    event->attr.exclude_hv	||
	    event->attr.exclude_idle	||
	    event->attr.exclude_host	||
	    event->attr.exclude_guest)
		return -EINVAL;

	/* check event validity */
	if (!pre_check_event_validity(pmu, event)) {
		dmsg("%s- not valid ev:%#llx\n", pmu->name, event->attr.config);
		return -EINVAL;
	}

	/* Unable to provide per-task data */
	if (event->cpu < 0)
		return -EINVAL;

	event->destroy = rtk_event_destroy;
	event->cpu = cpumask_first(&pmu->cpus);
	__reset_hwc(pmu, event);

	if (rtk_pmu_need_hrtimer(pmu)) {
		if (pmu->nr_active == 0)
			pmu->cpu = event->cpu;

		if (event->cpu != pmu->cpu)
			pr_err("%s- ev:%#llx runs on CPU%d instead of CPU%d\n",
			       pmu->name, event->attr.config, event->cpu,
			       pmu->cpu);
	}

	return err;
}

static void
rtk_pmu_start_event(struct perf_event *event, int pmu_flags)
{
	struct rtk_pmu *pmu = to_rtk_pmu(event->pmu);
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	struct hw_perf_event *hwc = &event->hw;
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	unsigned long flags;

	/* PMC already started */
	if (WARN_ON(!(hwc->state & PERF_HES_STOPPED)))
		return;

	if (IS_ENABLED(CONFIG_RTK_PMU_DEV)) {
		struct rtk_pmc_tracking *tracking = get_pmc_tracking(pmu, pmc);

		if (tracking->config != ps->pmc_config(event->attr.config))
			pr_err("%s- tracking event mis-match, %#x-%#llx\n",
			       pmu->name, tracking->config, event->attr.config);
	}

	raw_spin_lock_irqsave(&ps->ps_lock, flags);

	if (pmu->enable)
		pmu->enable(pmu, event);
	ps->start_pmc(ps, event);
	hwc->state = 0;

	if (rtk_pmu_need_hrtimer(pmu) && pmu->nr_active++ == 0)
		rtk_pmu_start_hrtimer(pmu);

	raw_spin_unlock_irqrestore(&ps->ps_lock, flags);
}

static void
rtk_pmu_stop_event(struct perf_event *event, int pmu_flags)
{
	struct rtk_pmu *pmu = to_rtk_pmu(event->pmu);
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	struct hw_perf_event *hwc = &event->hw;
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	unsigned long flags;

	if (hwc->state & PERF_HES_STOPPED)
		return;

	if (IS_ENABLED(CONFIG_RTK_PMU_DEV)) {
		struct rtk_pmc_tracking *tracking = get_pmc_tracking(pmu, pmc);

		if (tracking->config != ps->pmc_config(event->attr.config))
			pr_err("%s- tracking event mis-match, %#x-%#llx\n",
			       pmu->name, tracking->config, event->attr.config);
	}

	/* We always update the event, so ignore PERF_EF_UPDATE. */
	ps->read_pmc(ps, event);

	raw_spin_lock_irqsave(&ps->ps_lock, flags);

	if (rtk_pmu_need_hrtimer(pmu) && --pmu->nr_active == 0)
		rtk_pmu_stop_hrtimer(pmu);

	ps->stop_pmc(ps, event);
	hwc->state = PERF_HES_STOPPED | PERF_HES_UPTODATE;

	raw_spin_unlock_irqrestore(&ps->ps_lock, flags);
}

static inline void
rtk_pmu_set_perf_hwc(struct rtk_pmu *pmu, union rtk_pmc_desc pmc,
		     struct perf_event *event)
{
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);

	ps->set_perf_hwc(ps, pmc, event);
}

static union rtk_pmc_desc
rtk_pmu_get_event_hw(struct rtk_pmu *pmu, struct perf_event *event)
{
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	union rtk_pmc_desc pmc;
	unsigned long flags;
	int ret = 0;

	/*
	 * To make the PMU can be detected by Gator, the is_valid_event callback
	 * which is used to check the validity of event config may return true
	 * for an invalid event. Double-check for such invalid events.
	 */
	if (ps == NULL) {
		pmc.val = -EINVAL;
		return pmc;
	}

	/*
	 * Since the event validity was verified, we can just dispatch pmc
	 * allocation to corresponding pmc set to reserve pmc according to the
	 * event config.
	 */
	raw_spin_lock_irqsave(&ps->ps_lock, flags);

	pmc = pmu->arrange_pmc(pmu, event->attr.config);
	if (is_valid_pmc(pmc))
		ret = rtk_ps_track_event(ps, pmc, event);

	raw_spin_unlock_irqrestore(&ps->ps_lock, flags);

	if (ret)
		pmc.val = ret;

	return pmc;
}

static int
rtk_pmu_release_event_hw(struct rtk_pmu *pmu, struct perf_event *event)
{
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	unsigned long flags;
	int ret = -EINVAL;

	if (is_valid_pmc(pmc)) {
		raw_spin_lock_irqsave(&ps->ps_lock, flags);

		ret = rtk_ps_untrack_event(ps, pmc, event);
		if (!ret)
			ps->release_pmc(ps, pmc);

		raw_spin_unlock_irqrestore(&ps->ps_lock, flags);
	}

	return ret;
}

static int
rtk_pmu_add_event(struct perf_event *event, int flags)
{
	struct rtk_pmu *pmu = to_rtk_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	union rtk_pmc_desc pmc;
	int err = 0;

	dmsg("%s- add ev:%#llx cpu-%d\n",
	     pmu->name, event->attr.config, event->cpu);

	/* already inserted */
	if (hwc->idx != -1) {
		pr_err("%s- ev:%#llx already inserted, hwc:%x\n",
		       pmu->name, event->attr.config, hwc->idx);
		return -EBUSY;
	}

	/*
	 * Gator uses a pre-defined config to test the existence of PMU. To
	 * compatible with Gator, RTK PMU may let such event pass the checking
	 * in pmu::event_init callback, thus it must be rechecked to avoid
	 * unexpected behaviour.
	 */
	if (unlikely(!post_check_event_validity(pmu, event))) {
		pr_err("%s- Invalid event %#llx\n", pmu->name,
		       event->attr.config);
		return -EINVAL;
	}

	pmc = rtk_pmu_get_event_hw(pmu, event);

	/* If we don't have free counter for the event, finish early. */
	if (unlikely(!is_valid_pmc(pmc))) {
		err = pmc.val;
		dmsg("%s- invalid pmc %d\n", pmu->name, err);
		goto out;
	}

	/* save hw event */
	rtk_pmu_set_perf_hwc(pmu, pmc, event);

	hwc->state = PERF_HES_STOPPED | PERF_HES_UPTODATE;
	if (flags & PERF_EF_START)
		rtk_pmu_start_event(event, PERF_EF_RELOAD);

	/* Propagate our changes to the userspace mapping. */
	perf_event_update_userpage(event);

out:
	return err;
}

static void
rtk_pmu_del_event(struct perf_event *event, int flags)
{
	struct rtk_pmu *pmu = to_rtk_pmu(event->pmu);

	dmsg("%s- del ev:%#llx on cpu-%d\n",
	     pmu->name, event->attr.config, event->cpu);

	/*
	 * Gator uses a pre-defined config to test the existence of PMU. To
	 * compatible with Gator, RTK PMU may let such event pass the checking
	 * in pmu::event_init callback, thus it must be rechecked to avoid
	 * unexpected behaviour.
	 */
	if (unlikely(!post_check_event_validity(pmu, event))) {
		pr_err("%s- Invalid event %#llx\n", pmu->name,
		       event->attr.config);
		return;
	}

	rtk_pmu_stop_event(event, PERF_EF_UPDATE);
	rtk_pmu_release_event_hw(pmu, event);
	/* clear hwc info for event reinsertion */
	__reset_hwc(pmu, event);

	/* stop PMU */
	if (pmu->disable)
		pmu->disable(pmu, event);

	perf_event_update_userpage(event);
}

static void
rtk_pmu_read(struct perf_event *event)
{
	struct rtk_pmu *pmu = to_rtk_pmu(event->pmu);
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);

	ps->read_pmc(ps, event);
}

int
rtk_find_drv_pmc(struct rtk_pmc_set *ps, int hwc, int target)
{
	if (target < ps->meta->nr_pmcgs &&
	    !test_and_set_bit(target, &ps->used_mask)) {
		ps->tracking[target].target = target;
		ps->tracking[target].config = hwc;

		return target;
	}

	return RTK_INV_PMC_TARGET;
}
EXPORT_SYMBOL(rtk_find_drv_pmc);

void
rtk_set_drv_hwc(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
		struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;

	hwc->idx		= pmc.val;
	hwc->config		= pmc.idx;
}
EXPORT_SYMBOL(rtk_set_drv_hwc);

int
rtk_start_drv_pmc(struct rtk_pmc_set *ps, struct perf_event *event)
{
	local64_set(&event->hw.prev_count,
		    rtk_pmu_drv_pmc_read(ps->pmu, event->hw.config));
	return 0;
}
EXPORT_SYMBOL(rtk_start_drv_pmc);

int
rtk_stop_drv_pmc(struct rtk_pmc_set *ps, struct perf_event *event)
{
	return 0;
}
EXPORT_SYMBOL(rtk_stop_drv_pmc);

void
rtk_read_drv_pmc(struct rtk_pmc_set *ps, struct perf_event *event)
{
	struct rtk_pmu *pmu = to_rtk_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	u64 delta, prev_raw_count, new_raw_count;

	do {
		prev_raw_count = local64_read(&hwc->prev_count);
		new_raw_count = rtk_pmu_drv_pmc_read(pmu, hwc->config);
	} while (local64_cmpxchg(&hwc->prev_count, prev_raw_count,
				 new_raw_count) != prev_raw_count);

	delta = (new_raw_count - prev_raw_count);

	local64_add(delta, &event->count);
}
EXPORT_SYMBOL(rtk_read_drv_pmc);

static int
__init_ps(struct rtk_pmu *pmu, struct rtk_pmc_set *ps,
	  const struct rtk_pmc_set_meta *psm)
{
	int i;

	ps->pmu = pmu;
	ps->meta = psm;
	ps->name = psm->name;
	ps->type = psm->type;
	psm->init(psm, ps);

	ps->tracking = kcalloc(ps->meta->nr_pmcgs, sizeof(*ps->tracking),
			       GFP_KERNEL);
	if (unlikely(ps->tracking == NULL))
		return -ENOMEM;

	for (i = 0; i < ps->meta->nr_pmcgs; i++) {
		ps->tracking[i].events =
			kcalloc(psm->group_size, sizeof(struct perf_event *),
				GFP_KERNEL);

		if (unlikely(ps->tracking[i].events == NULL))
			return -ENOMEM;

		ps->tracking[i].target = RTK_INV_PMC_TARGET;
		ps->tracking[i].config = RTK_INV_PMC_TARGET;
	}

	raw_spin_lock_init(&ps->ps_lock);
	pr_info("%s:%d - %s init done\n", pmu->name, psm->type, psm->name);

	return 0;
}

static int
rtk_pmc_set_dts_init(struct rtk_pmc_set *ps, struct device_node *dt,
		     const struct rtk_pmc_set_meta *psm)
{
	struct device_node *child;

	for_each_child_of_node(dt, child) {
		if (!of_device_is_compatible(child, psm->compatible))
			continue;

		of_property_read_u32(child, "nr_defects", &ps->nr_defects);
		ps->defects = kmalloc_array(ps->nr_defects,
					    sizeof(*ps->defects), GFP_KERNEL);
		if (ps->defects == NULL)
			return -ENOMEM;

		of_property_read_u32_array(child, "defects", ps->defects,
					   ps->nr_defects);

		of_property_read_u32(child, "has_ext", &ps->has_ext);
		of_property_read_u32(child, "ext_info", &ps->ext_info);

		return 0;
	}

	return 0;
}

int
rtk_pmc_set_init(struct rtk_pmu *pmu, struct device_node *dt,
		 const struct rtk_pmc_set_meta *psm)
{
	struct rtk_pmc_set *ps;
	int ret = 0, i;

	for (i = 0; psm->name; psm++, i++) {
		ps = kzalloc(sizeof(*pmu->pmcss[0]), GFP_KERNEL);
		if (ps == NULL)
			return -ENOMEM;

		pmu->pmcss[i] = ps;
		ps->base = pmu->base;

		ret = rtk_pmc_set_dts_init(ps, dt, psm);
		if (ret)
			return ret;

		ret = __init_ps(pmu, ps, psm);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(rtk_pmc_set_init);

int rtk_uncore_pmu_init(struct rtk_pmu *pmu, struct device_node *dt,
			const char *name, const char *cpuhp_name,
			unsigned long ctrl_offset,
			const struct attribute_group **attr_groups,
			const struct rtk_pmc_set_meta *meta, int nr_ps,
			int drv_cnts)
{
	u32 value;

	pmu->name = name;
	pmu->cpuhp_name = cpuhp_name;
	pmu->attr_groups = attr_groups;

	pmu->nr_pmcss = nr_ps;
	pmu->pmcss = kcalloc(pmu->nr_pmcss, sizeof(*pmu->pmcss), GFP_KERNEL);
	if (pmu->pmcss == NULL)
		return -ENOMEM;

	if (drv_cnts) {
		pmu->drv_pmc = kcalloc(drv_cnts, sizeof(*pmu->drv_pmc),
				       GFP_KERNEL);
		if (pmu->drv_pmc == NULL)
			return -ENOMEM;
	}

	pmu->base = of_iomap(dt, 0);
	pmu->ctrl = (unsigned long) pmu->base + ctrl_offset;

	/*
	 * if interrupt is not forced disabled depends on SoC,
	 * enable PMU interrupt according to config
	 */
	if (!rtk_pmu_has_no_overflow(pmu) &&
	    !of_property_read_u32(dt, "overflow", &value) && value > 0)
		pmu->irq = irq_of_parse_and_map(dt, 0);
	else
		pmu->pmu.capabilities |= PERF_PMU_CAP_NO_INTERRUPT;

	return rtk_pmc_set_init(pmu, dt, meta);
}
EXPORT_SYMBOL(rtk_uncore_pmu_init);

static void
__ps_free(struct rtk_pmc_set *ps)
{
	int i;

	if (ps == NULL || ps->tracking == NULL)
		return;

	for (i = 0; i < ps->meta->nr_pmcgs; i++)
		kfree(ps->tracking[i].events);

	kfree(ps->defects);
	kfree(ps->tracking);
	kfree(ps);
}

static void
__pmu_free(struct rtk_pmu *pmu)
{
	int i;

	if (pmu->pmcss) {
		for (i = 0; i < pmu->nr_pmcss; i++)
			__ps_free(pmu->pmcss[i]);

		kfree(pmu->pmcss);
	}

	kfree(pmu->drv_pmc);
	kfree(pmu);
}

static int
__pre_init(struct rtk_pmu *rtk_pmu, struct platform_device *pdev)
{
	rtk_pmu->pdev = pdev;
	platform_set_drvdata(pdev, rtk_pmu);

	rtk_pmu->pmu = (struct pmu) {

		/* uncore or shared PMU must use perf_invalid_context */
		.task_ctx_nr	= perf_invalid_context,

		/*
		 * If the pmu_enable is set, the perf core sets pmu->{start_txn,
		 * commit_txn, cancel_txn} automatically. The pmu_enable
		 * callback will be invoked during doing start_txn, leading to
		 * PMU re-enabling when performing group_sched_in and
		 * perf_event_read.
		 *
		 * RTK PMU is reset automatically once it is re-enabled, and RTK
		 * pmu contains many subsystems which their control are bound
		 * together. Because of the reasons mentioned above, we do not
		 * provide pmu_enable/disable callback. We just detect the
		 * neccesity of pmu enabling/disabling and do it at proper
		 * moment.
		 *
		 * .pmu_enable	= rtk_pmu_enable,
		 * .pmu_disable	= rtk_pmu_disable,
		 */

		.event_init	= rtk_pmu_event_init,
		.add		= rtk_pmu_add_event,
		.del		= rtk_pmu_del_event,
		.start		= rtk_pmu_start_event,
		.stop		= rtk_pmu_stop_event,
		.read		= rtk_pmu_read,
	};

	return 0;
}

static int
__post_init(struct rtk_pmu *rtk_pmu)
{
	rtk_pmu->pmu.name		= rtk_pmu->name;
	rtk_pmu->pmu.attr_groups	= rtk_pmu->attr_groups;

	/*
	 * Use hrtimer to poll counters to avoid overflow when overflow
	 * interrupt is unavailable.
	 */
	if (rtk_pmu_need_hrtimer(rtk_pmu)) {
		pr_info("%s- init hrtimer\n", rtk_pmu->name);
		hrtimer_init(&rtk_pmu->hrtimer, CLOCK_MONOTONIC,
			     HRTIMER_MODE_REL);
		rtk_pmu->hrtimer.function = rtk_pmu_hrtimer;
		rtk_pmu->hrtimer_interval = RTK_UNCORE_HRTIMER_INTERVAL;
	}

	return 0;
}

static int
__cpu_offline(unsigned int cpu, struct hlist_node *node)
{
	struct rtk_pmu *rtk_pmu = hlist_entry_safe(node, struct rtk_pmu, cpuhp);
	unsigned int target;

	if (!cpumask_test_and_clear_cpu(cpu, &rtk_pmu->cpus))
		return 0;

	target = cpumask_any_but(cpu_online_mask, cpu);
	if (target >= nr_cpu_ids)
		return 0;

	cpumask_set_cpu(target, &rtk_pmu->cpus);
	return 0;
}

static int
__setup_cpuhp(struct rtk_pmu *rtk_pmu, const char *name)
{
	int ret;

	ret = cpuhp_setup_state_multi(CPUHP_AP_ONLINE_DYN, name, NULL,
				      __cpu_offline);

	if (ret < 0) {
		pr_err("Setup cpuhp state for %s failed, errno:%#x\n",
		       name, ret);
		return ret;
	}

	rtk_pmu->cpuhp_state = ret;
	pr_info("%s- registered cpuhp stat=%d\n", name, ret);

	return 0;
}

int
rtk_pmu_device_probe(struct platform_device *pdev,
		     const struct of_device_id *of_table)
{
	const struct of_device_id *of_id = NULL;
	int (*init_fn)(struct rtk_pmu *pmu, struct device_node *dt);
	struct device_node *node = pdev->dev.of_node;
	struct rtk_pmu *pmu;
	int ret = -ENODEV;

	dmsg("Probing PMU:%s\n", pdev->dev.of_node->name);

	if (node)
		of_id = of_match_node(of_table,
				      pdev->dev.of_node);

	if (of_id) {
		init_fn = of_id->data;
	} else {
		pr_err("%s not found\n", pdev->dev.of_node->name);
		return -ENODEV;
	}

	pmu = kzalloc(sizeof(struct rtk_pmu), GFP_KERNEL);
	if (unlikely(pmu == NULL)) {
		pr_err("failed to allocate PMU device for %s!\n",
		       pdev->dev.of_node->name);
		return -ENOMEM;
	}

	ret = __pre_init(pmu, pdev);
	if (ret)
		goto out_free;

	if (init_fn) {
		ret = init_fn(pmu, node);
		if (ret)
			goto out_free;
	}

	ret = __setup_cpuhp(pmu, pmu->cpuhp_name);
	if (ret)
		goto release_cpuhp;

	__post_init(pmu);

	/* init cpumask */
	cpumask_set_cpu(get_cpu(), &pmu->cpus);

	cpuhp_state_add_instance_nocalls(pmu->cpuhp_state, &pmu->cpuhp);

	ret = perf_pmu_register(&pmu->pmu, pmu->name, -1);
	if (ret)
		goto out_cpu;

	put_cpu();

	pr_info("%s- registered type=%d\n", pmu->name, pmu->pmu.type);
	return 0;

out_cpu:
	put_cpu();
	cpuhp_state_remove_instance(pmu->cpuhp_state, &pmu->cpuhp);

release_cpuhp:
	cpuhp_remove_multi_state(pmu->cpuhp_state);

out_free:
	pr_err("failed to register %s, errno=%d!\n",
	       of_node_full_name(node), ret);
	__pmu_free(pmu);
	return ret;
}
EXPORT_SYMBOL(rtk_pmu_device_probe);

int
rtk_pmu_device_remove(struct platform_device *pdev)
{
	struct rtk_pmu *rtk_pmu = platform_get_drvdata(pdev);

	/* clear the hrtimer */
	if (rtk_pmu_need_hrtimer(rtk_pmu) && rtk_pmu->nr_active != 0)
		rtk_pmu_stop_hrtimer(rtk_pmu);

	perf_pmu_unregister(&rtk_pmu->pmu);
	cpuhp_state_remove_instance(rtk_pmu->cpuhp_state, &rtk_pmu->cpuhp);
	cpuhp_remove_multi_state(rtk_pmu->cpuhp_state);
	__pmu_free(rtk_pmu);

	return 0;
}
EXPORT_SYMBOL(rtk_pmu_device_remove);

