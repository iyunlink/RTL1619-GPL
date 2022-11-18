// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek uncore PMU driver for DDR memory controller
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

#define pr_fmt(fmt)	"[RTK_PMU] " fmt

#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>

#include "rtk_uncore_pmu.h"
#include "rtk_rbus_pmu.h"


/* Rbus event format */
RTK_PMU_FORMAT_ATTR(event,	"config:0-12");

static struct attribute *rtk_rbus_format_attrs[] = {
	RTK_PMU_FORMAT_REF(event),
	NULL
};

struct attribute_group rtk_rbus_format_attr_group = {
	.name = "format",
	.attrs = rtk_rbus_format_attrs,
};
EXPORT_SYMBOL(rtk_rbus_format_attr_group);

static inline int
__pmc_to_ctrl_bit(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc)
{
	/*
	 * The sequence of overflow bit vector is differenct from counters in
	 * pmcgs, need remapping to find correct event
	 */
	static const int mapping[] = {
		[RBUS_PMC__REQ] = 0,
		[RBUS_PMC__RTG_TOTAL] = 1,
		[RBUS_PMC__RTA_MAX] = 3,
		[RBUS_PMC__RTA_TOTAL] = 2,
	};

	return ps->meta->clients[pmc.idx] * RBUS_OVERFLOW_PMCG_WIDTH +
		mapping[pmc.usage] +
		RBUS_OVERFLOW_OFFSET;
}

static void
rtk_rbus_ps_enable(struct rtk_pmc_set *ps, struct perf_event *event)
{
	unsigned int val;
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);

	dmsg("%s- enable %s, pmc:%x, config:%llx, mask:%x\n",
	     ps->pmu->name, ps->name, event->hw.idx, event->hw.config,
	     rtk_event_target_mask(event));

	val = rtk_readl(rtk_event_config_addr(event));
	val |= rtk_event_config(event);
	rtk_writel(rtk_event_config_addr(event), val);

	if (!rtk_pmu_has_no_overflow(ps->pmu))
		rtk_writel((unsigned long)ps->base + RBUS_OVERFLOW_ENABLE,
			   BIT(0) | BIT(__pmc_to_ctrl_bit(ps, pmc)));
}

static void
rtk_rbus_ps_disable(struct rtk_pmc_set *ps, struct perf_event *event)
{
	unsigned int val;
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);

	dmsg("%s- disable %s, pmc:%x, config:%llx, mask:%x, val:%llx\n",
	     ps->pmu->name, ps->name, event->hw.idx, event->hw.config,
	     rtk_event_target_mask(event),
	     event->hw.config & ~rtk_event_target_mask(event));

	/*
	 * Here play a bit-wise magic for compatibility of both Rbus PMU version
	 */
	val = rtk_readl(rtk_event_config_addr(event));
	val &= ~rtk_event_target_mask(event);
	val |= event->hw.config & ~rtk_event_target_mask(event);
	rtk_writel(rtk_event_config_addr(event), val);

	if (!rtk_pmu_has_no_overflow(ps->pmu))
		rtk_writel((unsigned long)ps->base + RBUS_OVERFLOW_ENABLE,
			   BIT(__pmc_to_ctrl_bit(ps, pmc)));
}

static int
rtk_rbus_pmc_config(u64 config)
{
	return (int)(config & HWC_MASK);
}

static struct rtk_pmc_set *
rtk_rbus_get_pmc_set(struct rtk_pmu *pmu, struct perf_event *event)
{
	union rtk_rbus_event_desc desc = get_event_desc(event);

	return (desc.set < pmu->nr_pmcss) ? pmu->pmcss[desc.set] : NULL;
}

static void
rtk_rbus_refresh(struct rtk_pmu *pmu, struct perf_event *event)
{
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	unsigned long flags;
	u64 start, end;

	raw_spin_lock_irqsave(&ps->ps_lock, flags);
	start = ktime_get_mono_fast_ns();
	rtk_pmu_drv_pmc_inc(pmu, RBUS_REFRESH);

	rtk_rbus_ps_disable(ps, event);

	rtk_ps_refresh_pmcg(ps, event);

	rtk_rbus_ps_enable(ps, event);

	rtk_ps_refresh_pmcg_done(ps, event);

	end = ktime_get_mono_fast_ns();
	raw_spin_unlock_irqrestore(&ps->ps_lock, flags);

	if (unlikely((end - start) > REFRESH_TH))
		pr_err("** %s refresh took long time: %lluns\n",
		       pmu->name, end - start);
}

static struct perf_event *
__find_event_by_ov(struct rtk_pmc_set *ps, int bit)
{
	/*
	 * overflow status bit from MSB to LSB is:
	 *	RTA_MAX, RTA_TOTAL, RTG_TOTAL, REQ
	 */
	static const int mapping[] = {
		RBUS_PMC__REQ,
		RBUS_PMC__RTG_TOTAL,
		RBUS_PMC__RTA_TOTAL,
		RBUS_PMC__RTA_MAX,
	};
	int usage = bit & GENMASK(1, 0);
	int client = bit / RBUS_OVERFLOW_PMCG_WIDTH;
	int pmcg;

	for (pmcg = 0; pmcg < ps->meta->nr_clients; pmcg++)
		if (ps->meta->clients[pmcg] == client)
			break;
	if (pmcg >= ps->meta->nr_pmcgs)
		return NULL;

	return ps->tracking[pmcg].events[mapping[usage]];
}

static void
update_overflow_event(struct rtk_pmu *pmu, unsigned long status)
{
	struct rtk_pmc_set *ps;
	unsigned long mask;
	struct perf_event *ev;
	int i;

	ps = pmu->pmcss[PMC_SET__RBUS];
	mask = status >> RBUS_OVERFLOW_OFFSET;
	for_each_set_bit(i, &mask, RBUS_OVERFLOW_STATUS_WIDTH) {
		ev = __find_event_by_ov(ps, i);
		compensate_event(ps, ev);
	}
}

static irqreturn_t
overflow_handler(int irq, void *rpmu)
{
#define NOP_THRESHOLD	3

	struct rtk_pmu *pmu = rpmu;
	bool handled = IRQ_NONE;
	u32 status;
	u32 ov_en;
	unsigned long addr;
	static unsigned int nop_count;

	ov_en = rtk_readl((unsigned long)pmu->base + RBUS_OVERFLOW_ENABLE);
	rtk_writel((unsigned long)pmu->base + RBUS_OVERFLOW_ENABLE, ov_en);

	addr = (unsigned long)pmu->base + RBUS_OVERFLOW_STATUS;
	status = rtk_readl(addr);
	rtk_writel(addr, status);

	/*
	 * Interrupts may be fired by memory trash function instead of counter
	 * overflow, reconfirm the necessity of overflow handling.
	 */
	if (!rtk_pmu_has_no_overflow(pmu)) {
		nop_count = (!status) ? nop_count + 1 : 0;

		if (nop_count == 0) {
			update_overflow_event(pmu, status);
			handled = IRQ_HANDLED;
		} else if (nop_count >= NOP_THRESHOLD) {
			pr_err("interrupted with no overflow %d times",
			       nop_count);
			nop_count = 0;
		}
	}

	rtk_pmu_drv_pmc_inc(pmu, RBUS_REFRESH);

	rtk_writel((unsigned long)pmu->base + RBUS_OVERFLOW_ENABLE,
		   ov_en | BIT(0));

	return IRQ_RETVAL(handled);
}

static int
rtk_rbus_check_event(struct rtk_pmu *pmu, struct perf_event *event)
{
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	union rtk_rbus_event_desc desc = get_event_desc(event);
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

static void __maybe_unused
rtk_rbus_set_hwc_v1(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
		    struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	int target = rtk_rbus_pmc_target(event->attr.config);

	rtk_event_set_pmc_desc(hwc, pmc.val);

	/* control register address */
	rtk_event_set_config_addr(hwc,
				  (unsigned long)ps->base +
				  ps->meta->configs[0]);

	/* mask of control register */
	rtk_event_set_target_mask(hwc, GENMASK(target, target));

	/* monitored target */
	rtk_event_set_config(hwc, GENMASK(target + 1, target));

	switch (pmc.usage) {
	case RBUS_PMC__ACC_LAT:
		/* pmc address, including the usage offset */
		rtk_event_set_pmc_addr(hwc,
				       (unsigned long)ps->base +
				       ps->meta->pmcgs[pmc.idx]);

		break;
	case RBUS_PMC__MAX_LAT:
		/* pmc address, including the usage offset */
		rtk_event_set_pmc_addr(hwc,
				       (unsigned long)ps->base +
				       ps->meta->pmcgs[pmc.idx] + 4);

		rtk_event_set_pmc_offset(hwc, 24);
		rtk_event_set_pmc_type(hwc, RTK_PMC_STATE);
		break;
	case RBUS_PMC__REQ_NUM:
		/* pmc address, including the usage offset */
		rtk_event_set_pmc_addr(hwc,
				       (unsigned long)ps->base +
				       ps->meta->pmcgs[pmc.idx] + 4);
		break;
	}

	rtk_event_set_pmc_mask(hwc, ps->meta->val_mask[pmc.usage]);
	rtk_event_set_pmc_threshold(hwc, ps->meta->ov_th[pmc.usage]);

	if (IS_ENABLED(CONFIG_RTK_PMU_DEV))
		rtk_event_set_pmc_ts(&event->hw, ktime_get_mono_fast_ns());

	dmsg("%s- ev:%#llx, cconf:%#llx, cbase:%#lx, ebase:%#lx, emask:%#x",
	     ps->pmu->name, event->attr.config, hwc->config,
	     hwc->config_base, hwc->event_base, hwc->event_base_rdpmc);
}

static void
rtk_rbus_set_hwc_v2(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
		    struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	int target = rtk_rbus_pmc_target(event->attr.config);

	rtk_event_set_pmc_desc(hwc, pmc.val);

	/* control register address */
	rtk_event_set_config_addr(hwc,
				  (unsigned long)ps->base +
				  ps->meta->configs[0]);

	/* mask of control register */
	rtk_event_set_target_mask(hwc, GENMASK(target, target));

	/* monitored target */
	rtk_event_set_config(hwc, GENMASK(target, target));

	rtk_event_set_pmc_addr(hwc,
			       (unsigned long)ps->base +
			       ps->meta->pmcgs[pmc.idx] + (pmc.usage << 2));

	rtk_event_set_pmc_mask(hwc, ps->meta->val_mask[pmc.usage]);
	rtk_event_set_pmc_threshold(hwc, ps->meta->ov_th[pmc.usage]);

	if (pmc.usage == RBUS_PMC__RTA_MAX)
		rtk_event_set_pmc_type(hwc, RTK_PMC_STATE);

	if (IS_ENABLED(CONFIG_RTK_PMU_DEV))
		rtk_event_set_pmc_ts(&event->hw, ktime_get_mono_fast_ns());

	dmsg("%s- ev:%#llx, cconf:%#llx, cbase:%#lx, ebase:%#lx, emask:%#x",
	     ps->pmu->name, event->attr.config, hwc->config,
	     hwc->config_base, hwc->event_base, hwc->event_base_rdpmc);
}

static int
rtk_rbus_find_pmc(struct rtk_pmc_set *ps, int hwc, int target)
{
	int i;

	for (i = 0; i < ps->meta->nr_clients; i++) {
		if (ps->meta->clients[i] == target) {
			test_and_set_bit(i, &ps->used_mask);
			ps->tracking[i].target = target;
			ps->tracking[i].config = hwc;

			return i;
		}
	}

	return RTK_INV_PMC_TARGET;
}

static union rtk_pmc_desc
rtk_rbus_arrange_pmc(struct rtk_pmu *pmu, u64 config)
{
	union rtk_pmc_desc pmc;
	union rtk_rbus_event_desc desc = __get_event_desc(config);
	struct rtk_pmc_set *ps = pmu->pmcss[desc.set];
	int idx = ps->arrange_pmc(ps,
				  rtk_rbus_pmc_config(config),
				  rtk_rbus_pmc_target(config));

	if (idx < 0) {
		pmc.val = -EAGAIN;
	} else {
		pmc.set = desc.set;
		pmc.idx = idx;
		pmc.usage = desc.usage;
	}

	dmsg("%s:%s- ev:%#llx, desc %#x:%#x:%#x\n", pmu->name, ps->name,
	     config, desc.set, desc.target, desc.usage);
	dmsg("%s:%s- event pmc set:%#x, idx:%#x, usage:%#x\n", pmu->name,
	     ps->name, pmc.set, pmc.idx, pmc.usage);

	return pmc;
}

int
rbus_pmu_ps_init_v2(const struct rtk_pmc_set_meta *meta, struct rtk_pmc_set *ps)
{
	ps->pmc_config = rtk_rbus_pmc_config;
	ps->release_pmc = rtk_ps_release_pmc;
	ps->arrange_pmc = rtk_rbus_find_pmc;
	ps->set_perf_hwc = rtk_rbus_set_hwc_v2;
	ps->start_pmc = rtk_ps_start_pmc;
	ps->stop_pmc = rtk_ps_stop_pmc;
	ps->read_pmc = rtk_ps_read_pmc;
	ps->enable = rtk_rbus_ps_enable;
	ps->disable = rtk_rbus_ps_disable;
	ps->read_counter = rtk_read_counter;

	return 0;
}
EXPORT_SYMBOL(rbus_pmu_ps_init_v2);

int
rbus_pmu_ps_drv_init(const struct rtk_pmc_set_meta *meta,
		     struct rtk_pmc_set *ps)
{
	ps->pmc_config = rtk_rbus_pmc_config;
	ps->release_pmc = rtk_ps_release_pmc;
	ps->arrange_pmc = rtk_find_drv_pmc;
	ps->set_perf_hwc = rtk_set_drv_hwc;
	ps->start_pmc = rtk_start_drv_pmc;
	ps->stop_pmc = rtk_stop_drv_pmc;
	ps->read_pmc = rtk_read_drv_pmc;

	return 0;
}
EXPORT_SYMBOL(rbus_pmu_ps_drv_init);

int
rtk_rbus_pmu_init(struct rtk_pmu *pmu, struct device_node *dt,
		  const char *name,
		  const struct attribute_group **attr_groups,
		  const struct rtk_pmc_set_meta *meta, int nr_ps)
{
	int ret;

	ret = rtk_uncore_pmu_init(pmu, dt, name, RTK_RBUS_PMU_CPUHP_NAME,
				  0, attr_groups,
				  meta, nr_ps, RBUS_EV_NUM);
	if (ret)
		return ret;

	pmu->get_pmc_set = rtk_rbus_get_pmc_set;
	pmu->is_valid_event = rtk_rbus_check_event;
	pmu->arrange_pmc = rtk_rbus_arrange_pmc;

	if (rtk_pmu_has_no_overflow(pmu)) {
		pmu->refresh = rtk_rbus_refresh;
	} else {
		ret = request_irq(pmu->irq, overflow_handler, IRQF_SHARED, name,
				  pmu);
		if (ret < 0)
			pr_err("%s request IRQ:%d failed(%d)\n", name, pmu->irq,
			       ret);
	}

	return ret;
}
EXPORT_SYMBOL(rtk_rbus_pmu_init);

static const struct of_device_id rtk_pmu_of_device_ids[] = {
	{
		.compatible = "realtek,rtk-16xxb-rbus-pmu",
		.data = rtk_16xxb_rbus_init
	},
	{},
};

static int
rtk_rbus_pmu_probe(struct platform_device *pdev)
{
	return rtk_pmu_device_probe(pdev, rtk_pmu_of_device_ids);
}

static int
rtk_rbus_pmu_remove(struct platform_device *pdev)
{
	return rtk_pmu_device_remove(pdev);
}

static struct platform_driver rtk_rbus_pmu_driver = {
	.driver         = {
		.name   = RTK_RBUS_PMU_PDEV_NAME,
		.of_match_table = rtk_pmu_of_device_ids,
	},
	.probe          = rtk_rbus_pmu_probe,
	.remove		= rtk_rbus_pmu_remove
};

module_platform_driver(rtk_rbus_pmu_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RTK RBUS PMU support");

