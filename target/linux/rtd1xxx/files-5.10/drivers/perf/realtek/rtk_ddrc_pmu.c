// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek uncore PMU driver for DDR memory controller
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

#define pr_fmt(fmt)	"[RTK_PMU] " fmt

#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>

#include "rtk_uncore_pmu.h"
#include "rtk_ddrc_pmu.h"


RTK_PMU_FORMAT_ATTR(event,	"config:0-13");

static struct attribute *rtk_ddrc_format_attrs[] = {
	RTK_PMU_FORMAT_REF(event),
	NULL
};

struct attribute_group rtk_ddrc_format_attr_group = {
	.name = "format",
	.attrs = rtk_ddrc_format_attrs,
};
EXPORT_SYMBOL(rtk_ddrc_format_attr_group);

static inline void
__enable(struct rtk_pmu *pmu)
{
	u32 ctrl = rtk_readl(pmu->ctrl) | BIT(0);

	rtk_writel(pmu->ctrl, ctrl);
}

static inline void
__disable(struct rtk_pmu *pmu)
{
	u32 ctrl = rtk_readl(pmu->ctrl) & ~BIT(0);

	rtk_writel(pmu->ctrl, ctrl);
}

static void
rtk_ddrc_ps_reset(struct rtk_pmc_set *ps, struct perf_event *event)
{
	dmsg("%s- reset\n", ps->pmu->name);
	__disable(ps->pmu);
	if (ps->used_mask)
		__enable(ps->pmu);
}

static inline int
__pmc_to_gr_ctrl_bit(union rtk_pmc_desc pmc)
{
	int offset;

	switch (pmc.set) {
	case PMC_SET__DDRC:
		offset = pmc.idx;
		break;
	case PMC_SET__DDRC_TOTAL:
		offset = 6;
		break;
	case PMC_SET__DDRC_LOG:
		offset = 7 + pmc.idx;
		break;
	default:
		offset = DDRC_RSVD_BIT;
	}

	return offset;
}

static int
__pmc_to_ctrl_bit(struct rtk_pmu *pmu, union rtk_pmc_desc pmc)
{
	if (pmc.set == PMC_SET__DDRC && pmc.idx <= 3)
		return pmc.idx * DDRC_PMCG_UPDATE_WIDTH + pmc.usage;
	else if (pmc.set == PMC_SET__DDRC)
		return (pmc.idx - 4) * DDRC_PMCG_UPDATE_WIDTH + pmc.usage;
	else if (pmc.set == PMC_SET__DDRC_TOTAL)
		return pmc.idx + 16;
	else if (pmc.set == PMC_SET__DDRC_LOG)
		return pmc.idx + 20;

	pr_err("%s do invalid PMC(%x) to bit translation\n",
	       pmu->name, pmc.val);
	return DDRC_RSVD_BIT;
}

static void
rtk_ddrc_ps_reset_with_ov(struct rtk_pmc_set *ps, struct perf_event *event)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	int offset = __pmc_to_gr_ctrl_bit(pmc);

	rtk_writel((unsigned long)ps->base + DDRC_PMCG_CLEAR, BIT(offset));
}

static void
rtk_ddrc_ps_enable_with_ov(struct rtk_pmc_set *ps, struct perf_event *event)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	unsigned long addr;
	u32 ctrl;

	/* enable overflow irq */
	if (ps->type == PMC_SET__DDRC && pmc.idx <= 3)
		addr = (unsigned long)ps->base + DDRC_OVERFLOW_ENABLE_0;
	else
		addr = (unsigned long)ps->base + DDRC_OVERFLOW_ENABLE_1;
	ctrl = rtk_readl(addr) | BIT(__pmc_to_ctrl_bit(ps->pmu, pmc));
	rtk_writel(addr, ctrl);

	/* enable pmcg if active */
	addr = (unsigned long)ps->base + DDRC_PMCG_ENABLE;
	ctrl = rtk_readl(addr) | BIT(__pmc_to_gr_ctrl_bit(pmc));
	rtk_writel(addr, ctrl);
}

static void
rtk_ddrc_ps_disable_with_ov(struct rtk_pmc_set *ps, struct perf_event *event)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	unsigned long addr;
	u32 ctrl;

	/* disable overflow irq */
	if (ps->type == PMC_SET__DDRC && pmc.idx <= 3)
		addr = (unsigned long)ps->base + DDRC_OVERFLOW_ENABLE_0;
	else
		addr = (unsigned long)ps->base + DDRC_OVERFLOW_ENABLE_1;
	ctrl = rtk_readl(addr) & ~BIT(__pmc_to_ctrl_bit(ps->pmu, pmc));
	rtk_writel(addr, ctrl);

	/* disable pmcg if no active */
	addr = (unsigned long)ps->base + DDRC_PMCG_ENABLE;
	ctrl = rtk_readl(addr) & ~BIT(__pmc_to_gr_ctrl_bit(pmc));
	rtk_writel(addr, ctrl);
}

static void
rtk_ddrc_enable(struct rtk_pmu *pmu, struct perf_event *event)
{
	if (pmu->pmcss[PMC_SET__DDRC]->used_mask ||
	    pmu->pmcss[PMC_SET__DDRC_TOTAL]->used_mask ||
	    pmu->pmcss[PMC_SET__DDRC_LOG]->used_mask) {
		dmsg("%s- enable\n", pmu->name);
		__enable(pmu);
	}
}

static void
rtk_ddrc_disable(struct rtk_pmu *pmu, struct perf_event *event)
{
	if (!pmu->pmcss[PMC_SET__DDRC]->used_mask &&
	    !pmu->pmcss[PMC_SET__DDRC_TOTAL]->used_mask &&
	    !pmu->pmcss[PMC_SET__DDRC_LOG]->used_mask) {
		dmsg("%s- disable\n", pmu->name);
		__disable(pmu);
	}
}

static int
rtk_ddrc_pmc_config(u64 config)
{
	return (int)(config & HWC_MASK);
}

struct rtk_pmc_set *
rtk_ddrc_get_pmc_set(struct rtk_pmu *pmu, struct perf_event *event)
{
	union rtk_ddrc_event_desc desc = get_event_desc(event);

	return (desc.set < pmu->nr_pmcss) ? pmu->pmcss[desc.set] : NULL;
}

static void
rtk_ddrc_refresh(struct rtk_pmu *pmu, struct perf_event *event)
{
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	unsigned long flags;
	u64 start, end;

	raw_spin_lock_irqsave(&ps->ps_lock, flags);
	start = ktime_get_mono_fast_ns();
	rtk_pmu_drv_pmc_inc(pmu, DDRC_REFRESH);

	__disable(ps->pmu);

	rtk_ps_refresh(pmu->pmcss[PMC_SET__DDRC]);
	rtk_ps_refresh(pmu->pmcss[PMC_SET__DDRC_TOTAL]);
	rtk_ps_refresh(pmu->pmcss[PMC_SET__DDRC_LOG]);

	__enable(ps->pmu);

	rtk_ps_refresh_done(pmu->pmcss[PMC_SET__DDRC]);
	rtk_ps_refresh_done(pmu->pmcss[PMC_SET__DDRC_TOTAL]);
	rtk_ps_refresh_done(pmu->pmcss[PMC_SET__DDRC_LOG]);

	end = ktime_get_mono_fast_ns();
	raw_spin_unlock_irqrestore(&ps->ps_lock, flags);

	if (unlikely((end - start) > REFRESH_TH))
		pr_err("** %s refresh took long time: %lluns\n",
		       pmu->name, end - start);
}

static struct perf_event *
__find_event_by_ov(struct rtk_pmc_set *ps, int bit)
{
	int usage = bit & GENMASK(2, 0);
	int pmcg = bit / DDRC_OV_PMCG_WIDTH;

	if (pmcg >= ps->meta->nr_pmcgs || usage >= DDRC_PMC__WPEND)
		return NULL;

	return ps->tracking[pmcg].events[usage];
}

static void
update_overflow_event(struct rtk_pmu *pmu, unsigned long ov_status0,
		      unsigned long ov_status1)
{
	struct rtk_pmc_set *ps;
	unsigned long mask, size, offset;
	struct perf_event *ev;
	int i;

	ps = pmu->pmcss[PMC_SET__DDRC];
	size = (ps->meta->nr_pmcgs - 1) * DDRC_OV_PMCG_WIDTH;
	if (size > DDRC_OV_STATUS_WIDTH) {
		offset = size - DDRC_OV_STATUS_WIDTH;
		size = DDRC_OV_STATUS_WIDTH;
	} else {
		offset = 0;
	}
	for_each_set_bit(i, &ov_status0, size) {
		ev = __find_event_by_ov(ps, i);
		compensate_event(ps, ev);
	}
	if (offset) {
		for_each_set_bit(i, &ov_status1, offset) {
			ev = __find_event_by_ov(ps, i);
			compensate_event(ps, ev);
		}
	}

	ps = pmu->pmcss[PMC_SET__DDRC_TOTAL];
	size = DDRC_PMC__TOTAL_USAGE_NUM;
	mask = ov_status1 >> offset;
	for_each_set_bit(i, &mask, size) {
		ev = ps->tracking[i].events[0];
		compensate_event(ps, ev);
	}

	ps = pmu->pmcss[PMC_SET__DDRC_LOG];
	size = ps->meta->nr_pmcgs;
	mask = ov_status1 >> (offset + 4);
	for_each_set_bit(i, &mask, size) {
		ev = ps->tracking[i].events[0];
		compensate_event(ps, ev);
	}
}

static irqreturn_t
overflow_handler(int irq, void *rpmu)
{
#define NOP_THRESHOLD	3

	struct rtk_pmu *pmu = rpmu;
	bool handled = IRQ_NONE;
	u32 ov_status0, ov_status1;
	u32 ov_en0, ov_en1;
	unsigned long addr;
	static unsigned int nop_count;

	ov_en0 = rtk_readl((unsigned long)pmu->base + DDRC_OVERFLOW_ENABLE_0);
	ov_en1 = rtk_readl((unsigned long)pmu->base + DDRC_OVERFLOW_ENABLE_1);
	rtk_writel((unsigned long)pmu->base + DDRC_OVERFLOW_ENABLE_0, 0);
	rtk_writel((unsigned long)pmu->base + DDRC_OVERFLOW_ENABLE_1, 0);

	addr = (unsigned long)pmu->base + DDRC_OVERFLOW_STATUS_0;
	ov_status0 = rtk_readl(addr);
	rtk_writel(addr, 0);

	addr = (unsigned long)pmu->base + DDRC_OVERFLOW_STATUS_1;
	ov_status1 = rtk_readl(addr);
	rtk_writel(addr, 0);

	/*
	 * Interrupts may be fired by memory trash function instead of counter
	 * overflow, reconfirm the necessity of overflow handling.
	 */
	if (!rtk_pmu_has_no_overflow(pmu)) {
		nop_count = (!ov_status0 && !ov_status1) ? nop_count + 1 : 0;

		if (nop_count == 0) {
			update_overflow_event(pmu, ov_status0, ov_status1);
			handled = IRQ_HANDLED;
		} else if (nop_count >= NOP_THRESHOLD) {
			pr_err("interrupted with no overflow %d times",
			       nop_count);
			nop_count = 0;
		}
	}

	rtk_pmu_drv_pmc_inc(pmu, DDRC_REFRESH);
	rtk_writel((unsigned long)pmu->base + DDRC_OVERFLOW_ENABLE_0, ov_en0);
	rtk_writel((unsigned long)pmu->base + DDRC_OVERFLOW_ENABLE_1, ov_en1);

	return IRQ_RETVAL(handled);
}

static union pmc_config_desc
determine_pmc_config(union rtk_pmc_desc pmc, int gran, int width)
{
	union pmc_config_desc conf;

	conf.set = pmc.set;
	conf.width = width;
	conf.idx = pmc.idx / gran;
	conf.offset = (pmc.idx - (conf.idx * gran)) * width;

	/* configs of DDRC PERF_LOG does not start from bit-0 */
	if (pmc.set == PMC_SET__DDRC_LOG)
		conf.offset += DDRC_LOG_CONF_OFFSET;

	return conf;
}

static void
__set_hwc(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
	  struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	union pmc_config_desc conf;

	conf = determine_pmc_config(pmc, ps->meta->config_size,
				    ps->meta->config_width);

	dmsg("%s:%s- set hwc, pmc idx:%d, usage:%d\n", ps->pmu->name, ps->name,
	     pmc.idx, pmc.usage);

	rtk_event_set_pmc_desc(hwc, pmc.val);

	/* control register address */
	rtk_event_set_config_addr(hwc,
				  (unsigned long)ps->base +
				  ps->meta->configs[conf.idx]);

	/* mask of control register */
	rtk_event_set_target_mask(hwc,
				  GENMASK(conf.offset + conf.width - 1,
					  conf.offset));

	/* monitored target */
	rtk_event_set_config(hwc, (rtk_ddrc_pmc_target(event->attr.config) <<
				   conf.offset) & rtk_event_target_mask(event));

	if (IS_ENABLED(CONFIG_RTK_PMU_DEV))
		rtk_event_set_pmc_ts(hwc, ktime_get_mono_fast_ns());

	dmsg("%s- ev:%#llx, cconf:%#llx, cbase:%#lx, ebase:%#lx, rdpmc:%#x",
	     ps->pmu->name, event->attr.config, hwc->config,
	     rtk_event_config_addr(event), rtk_event_pmc_addr(event),
	     rtk_event_target_mask(event));
}

static void
rtk_ddrc_set_hwc(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
		      struct perf_event *event)
{
	unsigned long addr;

	if (pmc.usage < __DDRC_PMC__PEND_USAGE_START) {
		/*
		 * each counter is 32bit width, so the shiftment of each usage
		 * is 4 bytes
		 */
		rtk_event_set_pmc_addr(&event->hw, (unsigned long)ps->base +
				       ps->meta->pmcgs[pmc.idx] +
				       (pmc.usage << 2));
	} else {
		/*
		 * The actual address of pending request counter. The pending
		 * request counter is 8bit width. Each target has two pending
		 * counters for read and write respectively.
		 */
		addr = (unsigned long)ps->base +
			ps->meta->pmcgs[ps->meta->nr_pmcgs - 1] +
			(pmc.idx << 1) +
			(pmc.usage - __DDRC_PMC__PEND_USAGE_START);

		/*
		 * However, the counter can not be accessed using an unaligned
		 * address, we need to read out the full 4 bytes counter then
		 * mask out useless part.
		 *
		 * FIXME: If NR_PEND_REQ_PER_COUNTER is not power of 2, the
		 * corresponding caculation get wrong result.
		 */
		rtk_event_set_pmc_offset(&event->hw,
					 (addr &
					  (NR_PEND_REQ_PER_COUNTER - 1)) << 3);
		addr &= ~(NR_PEND_REQ_PER_COUNTER - 1);
		rtk_event_set_pmc_addr(&event->hw, addr);

		/* set proper type */
		rtk_event_set_pmc_type(&event->hw, RTK_PMC_STATE);
	}

	__set_hwc(ps, pmc, event);

	/* set proper value mask and overflow threshold */
	rtk_event_set_pmc_mask(&event->hw, ps->meta->val_mask[pmc.usage]);
	rtk_event_set_pmc_threshold(&event->hw, ps->meta->ov_th[pmc.usage]);

	if (rtk_pmu_need_update_ctrl(ps->pmu)) {
		addr = (unsigned long)ps->base +
			((pmc.idx <= 3) ?
			 DDRC_UPDATE_0 : DDRC_UPDATE_1);
		rtk_event_set_pmc_update_ctrl(&event->hw, addr);
		rtk_event_set_pmc_update(&event->hw,
					 BIT(__pmc_to_ctrl_bit(ps->pmu,
							       pmc)));

		dmsg("ev:%#llx, pmc=%d:%d:%d update ctrl:%#lx, update:%#x\n",
		     event->attr.config, pmc.set, pmc.idx, pmc.usage,
		     rtk_event_pmc_update_ctrl(event),
		     rtk_event_pmc_update(event));
	}
}

static void
rtk_ddrc_log_set_hwc(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
		      struct perf_event *event)
{
	rtk_event_set_pmc_addr(&event->hw, (unsigned long)ps->base +
			       ps->meta->pmcgs[pmc.idx]);

	__set_hwc(ps, pmc, event);

	/* set proper value mask and overflow threshold */
	rtk_event_set_pmc_mask(&event->hw, RTK_PMU_VAL_MASK(32));
	rtk_event_set_pmc_threshold(&event->hw, RTK_PMU_OVERFLOW_TH(32));

	if (rtk_pmu_need_update_ctrl(ps->pmu)) {
		rtk_event_set_pmc_update_ctrl(&event->hw,
					      (unsigned long)ps->base +
					      DDRC_UPDATE_1);
		rtk_event_set_pmc_update(&event->hw,
					 BIT(__pmc_to_ctrl_bit(ps->pmu, pmc)));

		dmsg("ev:%#llx, pmc=%d:%d:%d update ctrl:%#lx, update:%#x\n",
		     event->attr.config, pmc.set, pmc.idx, pmc.usage,
		     rtk_event_pmc_update_ctrl(event),
		     rtk_event_pmc_update(event));
	}
}

static void
rtk_ddrc_total_set_hwc(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
		       struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;

	rtk_event_set_pmc_desc(hwc, pmc.val);

	/* pmc address */
	rtk_event_set_pmc_addr(hwc, (unsigned long)ps->base +
			       ps->meta->pmcgs[pmc.idx]);

	/* set proper value mask and overflow threshold */
	rtk_event_set_pmc_mask(&event->hw, RTK_PMU_VAL_MASK(32));
	rtk_event_set_pmc_threshold(&event->hw, RTK_PMU_OVERFLOW_TH(32));

	if (rtk_pmu_need_update_ctrl(ps->pmu)) {
		rtk_event_set_pmc_update_ctrl(&event->hw,
					      (unsigned long)ps->base +
					      DDRC_UPDATE_1);
		rtk_event_set_pmc_update(&event->hw,
					 BIT(__pmc_to_ctrl_bit(ps->pmu, pmc)));

		dmsg("ev:%#llx, pmc=%d:%d:%d update ctrl:%#lx, update:%#x\n",
		     event->attr.config, pmc.set, pmc.idx, pmc.usage,
		     rtk_event_pmc_update_ctrl(event),
		     rtk_event_pmc_update(event));
	}

	if (IS_ENABLED(CONFIG_RTK_PMU_DEV))
		rtk_event_set_pmc_ts(hwc, ktime_get_mono_fast_ns());

	dmsg("%s- ev:%#llx, ebase:%#lx\n",
	     ps->pmu->name, event->attr.config, rtk_event_pmc_addr(event));
}

static int
rtk_ddrc_total_find_pmc(struct rtk_pmc_set *ps, int hwc, int target)
{
	if (target < ps->meta->nr_pmcgs &&
	    !test_and_set_bit(target, &ps->used_mask)) {
		ps->tracking[target].target = target;
		ps->tracking[target].config = hwc;

		return target;
	}

	return RTK_INV_PMC_TARGET;
}

union rtk_pmc_desc
rtk_ddrc_arrange_pmc(struct rtk_pmu *pmu, u64 config)
{
	union rtk_pmc_desc pmc;
	union rtk_ddrc_event_desc desc = __get_event_desc(config);
	struct rtk_pmc_set *ps = pmu->pmcss[desc.set];
	int idx = ps->arrange_pmc(ps,
				  rtk_ddrc_pmc_config(config),
				  rtk_ddrc_pmc_target(config));

	if (idx < 0) {
		pmc.val = -EAGAIN;
	} else {
		pmc.set = desc.set;
		pmc.idx = idx;
		pmc.usage = desc.usage;
	}

	dmsg("%s:%s- ev:%#llx, desc %#x:%#x:%#x:%#x\n", pmu->name, ps->name,
	     config, desc.set, desc.target, desc.usage, desc.core);
	dmsg("%s:%s- event pmc set:%#x, idx:%#x, usage:%#x\n", pmu->name,
	     ps->name, pmc.set, pmc.idx, pmc.usage);
	return pmc;
}

int
rtk_ddrc_check_event(struct rtk_pmu *pmu, struct perf_event *event)
{
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	union rtk_ddrc_event_desc desc = get_event_desc(event);
	int i;

	if (ps == NULL || desc.usage >= ps->meta->group_size) {
		dmsg("%s- ev:%llx out of range\n",
		     pmu->name, event->attr.config);
		return false;
	}

	/* check clients for event validity */
	for (i = 0; i < ps->meta->nr_clients; i++) {
		if (desc.target == ps->meta->clients[i])
			return true;
	}

	dmsg("%s- not valid client, config:%#llx, target:%#x, set:%d\n",
	     pmu->name, event->attr.config, desc.target, desc.set);
	return false;
}

static void
__set_op(struct rtk_pmc_set *ps)
{
	if (rtk_pmu_has_no_overflow(ps->pmu)) {
		ps->reset = rtk_ddrc_ps_reset;
		ps->read_counter = rtk_read_counter;
	} else {
		ps->enable = rtk_ddrc_ps_enable_with_ov;
		ps->disable = rtk_ddrc_ps_disable_with_ov;
		ps->reset = rtk_ddrc_ps_reset_with_ov;
		ps->read_counter = rtk_read_counter_with_update;
	}
	ps->pmc_config = rtk_ddrc_pmc_config;
	ps->release_pmc = rtk_ps_release_pmc;

	ps->start_pmc = rtk_ps_start_pmc;
	ps->stop_pmc = rtk_ps_stop_pmc;
	ps->read_pmc = rtk_ps_read_pmc;
}

/* init PMCG set */
int
rtk_ddrc_ps_init(const struct rtk_pmc_set_meta *meta,
		 struct rtk_pmc_set *ps)
{
	/* basic ops */
	__set_op(ps);

	ps->set_perf_hwc = rtk_ddrc_set_hwc;
	ps->arrange_pmc = rtk_ps_find_pmc;

	return 0;
}
EXPORT_SYMBOL(rtk_ddrc_ps_init);

/* init counter set for total statistics */
int
rtk_ddrc_ps_total_init(const struct rtk_pmc_set_meta *meta,
		       struct rtk_pmc_set *ps)
{
	__set_op(ps);

	ps->set_perf_hwc = rtk_ddrc_total_set_hwc;
	ps->arrange_pmc = rtk_ddrc_total_find_pmc;

	return 0;
}
EXPORT_SYMBOL(rtk_ddrc_ps_total_init);

int
rtk_ddrc_ps_log_init(const struct rtk_pmc_set_meta *meta,
		       struct rtk_pmc_set *ps)
{
	/* basic ops */
	__set_op(ps);

	ps->set_perf_hwc = rtk_ddrc_log_set_hwc;

	return 0;
}
EXPORT_SYMBOL(rtk_ddrc_ps_log_init);

/* init driver counter set */
int
rtk_ddrc_ps_drv_init(const struct rtk_pmc_set_meta *meta,
		     struct rtk_pmc_set *ps)
{
	__set_op(ps);

	ps->set_perf_hwc = rtk_set_drv_hwc;
	ps->arrange_pmc = rtk_find_drv_pmc;

	/* override callback */
	ps->start_pmc = rtk_start_drv_pmc;
	ps->stop_pmc = rtk_stop_drv_pmc;
	ps->read_pmc = rtk_read_drv_pmc;

	return 0;
}
EXPORT_SYMBOL(rtk_ddrc_ps_drv_init);

int rtk_ddrc_pmu_init(struct rtk_pmu *pmu, struct device_node *dt,
		      const char *name, unsigned long ctrl_offset,
		      const struct attribute_group **attr_groups,
		      const struct rtk_pmc_set_meta *meta, int nr_ps)
{
	int ret;

	ret = rtk_uncore_pmu_init(pmu, dt, name, RTK_DDRC_PMU_CPUHP_NAME,
				  ctrl_offset, attr_groups,
				  meta, nr_ps, DDRC_EV_NUM);

	if (ret)
		return ret;

	pmu->get_pmc_set = rtk_ddrc_get_pmc_set;
	pmu->is_valid_event = rtk_ddrc_check_event;
	pmu->arrange_pmc = rtk_ddrc_arrange_pmc;
	pmu->enable = rtk_ddrc_enable;
	pmu->disable = rtk_ddrc_disable;

	/* set overflow mode */
	if (rtk_pmu_has_no_overflow(pmu)) {
		pmu->refresh = rtk_ddrc_refresh;
		rtk_writel(pmu->ctrl, rtk_readl(pmu->ctrl) &
			   ~BIT(DDRC_CTRL_CNT_FREERUN_BIT));
	} else {
		/*
		 * DDRC need to raise update signal before reading counter when
		 * under freerun mode
		 */
		pmu->flags |= RTK_PMU_NEED_UPDATE_CTRL;

		ret = request_irq(pmu->irq, overflow_handler, IRQF_SHARED, name,
				  pmu);
		if (ret < 0)
			pr_err("%s request IRQ:%d failed(%d)\n", name, pmu->irq,
			       ret);

		rtk_writel(pmu->ctrl, rtk_readl(pmu->ctrl) |
			   BIT(DDRC_CTRL_CNT_FREERUN_BIT));

		/* clear PMCG enable */
		rtk_writel((unsigned long)pmu->base + DDRC_PMCG_ENABLE, 0);
	}

	return ret;
}
EXPORT_SYMBOL(rtk_ddrc_pmu_init);

static const struct of_device_id rtk_ddrc_of_device_ids[] = {
	{
		.compatible = "realtek,rtk-16xxb-ddrc-pmu",
		.data = rtk_16xxb_ddrc_init
	},
	{},
};

static int
rtk_ddrc_pmu_probe(struct platform_device *pdev)
{
	return rtk_pmu_device_probe(pdev, rtk_ddrc_of_device_ids);
}

static int
rtk_ddrc_pmu_remove(struct platform_device *pdev)
{
	return rtk_pmu_device_remove(pdev);
}

static struct platform_driver rtk_ddrc_pmu_driver = {
	.driver         = {
		.name   = RTK_DDRC_PMU_PDEV_NAME,
		.of_match_table = rtk_ddrc_of_device_ids,
	},
	.probe          = rtk_ddrc_pmu_probe,
	.remove		= rtk_ddrc_pmu_remove
};

module_platform_driver(rtk_ddrc_pmu_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RTK DDRC PMU support");

