// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek uncore PMU driver for Dbus system
 *
 * Copyright (C) 2019-2022 Realtek Semiconductor Corporation
 * Copyright (C) 2019-2022 Ping-Hsiung Chiu <phelic@realtek.com>
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
#include "rtk_dbus_pmu.h"


RTK_PMU_FORMAT_ATTR(event,	"config:0-15");

static struct attribute *rtk_dbus_format_attrs[] = {
	RTK_PMU_FORMAT_REF(event),
	NULL
};

struct attribute_group rtk_dbus_format_attr_group = {
	.name = "format",
	.attrs = rtk_dbus_format_attrs,
};
EXPORT_SYMBOL(rtk_dbus_format_attr_group);

static inline void
__dbus_enable(struct rtk_pmu *pmu)
{
	u32 ctrl = rtk_readl(pmu->ctrl) | BIT(0);

	rtk_writel(pmu->ctrl, ctrl);
}

static inline void
__dbus_disable(struct rtk_pmu *pmu)
{
	u32 ctrl = rtk_readl(pmu->ctrl) & ~BIT(0);

	rtk_writel(pmu->ctrl, ctrl);
}

static void
rtk_dbus_ps_reset(struct rtk_pmc_set *ps, struct perf_event *event)
{
	dmsg("%s- reset\n", ps->pmu->name);
	__dbus_disable(ps->pmu);
	if (ps->used_mask)
		__dbus_enable(ps->pmu);
}

static inline int
__pmc_to_gr_ctrl_bit(union rtk_pmc_desc pmc)
{
	static const int offset_map[] = {
		[PMC_SET__DBUS_SYS] = 16,
		[PMC_SET__DBUS_SYSH] = 4,
		[PMC_SET__DBUS_CH] = 0,
	};

	return (pmc.set < ARRAY_SIZE(offset_map)) ?
		offset_map[pmc.set] + pmc.idx : DBUS_RSVD_BIT;
}

static inline int
__pmc_to_ctrl_bit(struct rtk_pmu *pmu, union rtk_pmc_desc pmc)
{
	int ret;
	/*
	 * The sequence of bit vector is differenct from counters in pmcgs, need
	 * remapping to find correct controlling bit
	 */
	static const int mapping[3][4] = {
		[PMC_SET__DBUS_SYS] = {0, 3, 1, 2},
		[PMC_SET__DBUS_SYSH] = {0, 3, 1, 2},
		[PMC_SET__DBUS_CH] = {0, 1, 2, -1},
	};

	ret = pmc.idx * DBUS_UPDATE_CTRL_WIDTH + mapping[pmc.set][pmc.usage];
	if (pmc.set == PMC_SET__DBUS_SYS)
		ret += pmu->pmcss[PMC_SET__DBUS_CH]->meta->nr_pmcgs *
			DBUS_UPDATE_CTRL_WIDTH;

	return ret;
}

static void
rtk_dbus_ps_reset_with_ov(struct rtk_pmc_set *ps, struct perf_event *event)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	int offset = __pmc_to_gr_ctrl_bit(pmc);

	rtk_writel((unsigned long)ps->base + DBUS_PMCG_CLEAR, BIT(offset));
}

static void
rtk_dbus_ps_enable_with_ov(struct rtk_pmc_set *ps, struct perf_event *event)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	unsigned long addr;
	u32 ctrl;

	/* enable overflow irq */
	addr = (unsigned long)ps->base + ((ps->type == PMC_SET__DBUS_SYSH) ?
					  DBUS_SYSH_OVERFLOW_ENABLE :
					  DBUS_SYS_OVERFLOW_ENABLE);
	ctrl = rtk_readl(addr) | BIT(__pmc_to_ctrl_bit(ps->pmu, pmc));
	rtk_writel(addr, ctrl);

	/* enable pmcg if active */
	addr = (unsigned long)ps->base + DBUS_PMCG_ENABLE;
	ctrl = rtk_readl(addr) | BIT(__pmc_to_gr_ctrl_bit(pmc));
	rtk_writel(addr, ctrl);
}

static void
rtk_dbus_ps_disable_with_ov(struct rtk_pmc_set *ps, struct perf_event *event)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	unsigned long addr;
	u32 ctrl;

	/* disable overflow irq */
	addr = (unsigned long)ps->base + ((ps->type == PMC_SET__DBUS_SYSH) ?
					  DBUS_SYSH_OVERFLOW_ENABLE :
					  DBUS_SYS_OVERFLOW_ENABLE);
	ctrl = rtk_readl(addr) & ~BIT(__pmc_to_ctrl_bit(ps->pmu, pmc));
	rtk_writel(addr, ctrl);

	/* disable pmcg if no active */
	addr = (unsigned long)ps->base + DBUS_PMCG_ENABLE;
	ctrl = rtk_readl(addr) & ~BIT(__pmc_to_gr_ctrl_bit(pmc));
	rtk_writel(addr, ctrl);
}

static void
rtk_dbus_enable(struct rtk_pmu *pmu, struct perf_event *event)
{
	if (pmu->pmcss[PMC_SET__DBUS_SYSH]->used_mask ||
	    pmu->pmcss[PMC_SET__DBUS_SYS]->used_mask ||
	    pmu->pmcss[PMC_SET__DBUS_CH]->used_mask) {
		dmsg("%s- enable\n", pmu->name);
		__dbus_enable(pmu);
	}
}

static void
rtk_dbus_disable(struct rtk_pmu *pmu, struct perf_event *event)
{
	if (!pmu->pmcss[PMC_SET__DBUS_SYSH]->used_mask &&
	    !pmu->pmcss[PMC_SET__DBUS_SYS]->used_mask &&
	    !pmu->pmcss[PMC_SET__DBUS_CH]->used_mask) {
		dmsg("%s- disable\n", pmu->name);
		__dbus_disable(pmu);
	}
}

static void
rtk_dbus_refresh(struct rtk_pmu *pmu, struct perf_event *event)
{
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	unsigned long flags;
	u64 start, end;

	if (ps == NULL)
		return;

	raw_spin_lock_irqsave(&ps->ps_lock, flags);
	start = ktime_get_mono_fast_ns();
	rtk_pmu_drv_pmc_inc(pmu, DBUS_REFRESH);

	/*
	 * SYS and SYSH are controlled by the same reg, refresh them together.
	 * Force to stop and update the counter.
	 */
	__dbus_disable(ps->pmu);

	rtk_ps_refresh(pmu->pmcss[PMC_SET__DBUS_SYSH]);
	rtk_ps_refresh(pmu->pmcss[PMC_SET__DBUS_SYS]);
	rtk_ps_refresh(pmu->pmcss[PMC_SET__DBUS_CH]);

	__dbus_enable(ps->pmu);

	rtk_ps_refresh_done(pmu->pmcss[PMC_SET__DBUS_SYSH]);
	rtk_ps_refresh_done(pmu->pmcss[PMC_SET__DBUS_SYS]);
	rtk_ps_refresh_done(pmu->pmcss[PMC_SET__DBUS_CH]);

	end = ktime_get_mono_fast_ns();
	raw_spin_unlock_irqrestore(&ps->ps_lock, flags);

	if (unlikely((end - start) > REFRESH_TH))
		pr_err("** %s refresh took long time: %lluns\n",
		       pmu->name, end - start);
}

int
rtk_dbus_pmc_config(u64 config)
{
	return (int)(config & HWC_MASK);
}
EXPORT_SYMBOL(rtk_dbus_pmc_config);

static struct perf_event *
__find_event_by_ov(struct rtk_pmc_set *ps, int bit)
{
	int pmcg = ov_bit_to_pmcg_idx(bit);
	int usage = ov_bit_to_cnt_idx(bit);
	/*
	 * The sequence of overflow bit vector is differenct from counters in
	 * pmcgs, need remapping to find correct event
	 */
	static const int mapping[3][4] = {
		[PMC_SET__DBUS_SYS] = {0, 2, 3, -1},
		[PMC_SET__DBUS_SYSH] = {0, 2, 3, -1},
		[PMC_SET__DBUS_CH] = {0, 1, 2, -1},
	};

	usage = mapping[ps->type][usage];

	if (pmcg >= ps->meta->nr_pmcgs || usage < 0)
		return NULL;

	return ps->tracking[pmcg].events[usage];
}

static void
update_overflow_event(struct rtk_pmu *pmu, unsigned long ov_sysh,
		      unsigned long ov_sys)
{
	struct rtk_pmc_set *ps;
	unsigned long mask, size, offset;
	struct perf_event *ev;
	int i;

	ps = pmu->pmcss[PMC_SET__DBUS_SYSH];
	size = ps->meta->nr_pmcgs * DBUS_OV_PMCG_WIDTH;
	for_each_set_bit(i, &ov_sysh, size) {
		ev = __find_event_by_ov(ps, i);
		if (ev)
			compensate_event(ps, ev);
	}

	ps = pmu->pmcss[PMC_SET__DBUS_SYS];
	size = ps->meta->nr_pmcgs * DBUS_OV_PMCG_WIDTH;
	offset = pmu->pmcss[PMC_SET__DBUS_CH]->meta->nr_pmcgs *
		DBUS_OV_PMCG_WIDTH;
	mask = ov_sys >> offset;
	for_each_set_bit(i, &mask, size) {
		ev = __find_event_by_ov(ps, i);
		if (ev)
			compensate_event(ps, ev);
	}

	ps = pmu->pmcss[PMC_SET__DBUS_CH];
	size = ps->meta->nr_pmcgs * DBUS_OV_PMCG_WIDTH;
	mask = ov_sys & GENMASK(size - 1, 0);
	for_each_set_bit(i, &mask, size) {
		ev = __find_event_by_ov(ps, i);
		if (ev)
			compensate_event(ps, ev);
	}
}

static irqreturn_t
overflow_handler(int irq, void *rpmu)
{
#define NOP_THRESHOLD	3

	struct rtk_pmu *pmu = rpmu;
	bool handled = IRQ_NONE;
	u32 sysh_status, sys_status;
	u32 sysh_ov_en, sys_ov_en;
	unsigned long addr;
	static unsigned int nop_count;

	sysh_ov_en = rtk_readl((unsigned long)pmu->base +
			       DBUS_SYSH_OVERFLOW_ENABLE);
	sys_ov_en = rtk_readl((unsigned long)pmu->base +
			      DBUS_SYS_OVERFLOW_ENABLE);
	rtk_writel((unsigned long)pmu->base + DBUS_SYSH_OVERFLOW_ENABLE, 0);
	rtk_writel((unsigned long)pmu->base + DBUS_SYS_OVERFLOW_ENABLE, 0);

	addr = (unsigned long)pmu->base + DBUS_OV_SYSH_STATUS;
	sysh_status = rtk_readl(addr);
	rtk_writel(addr, 0);

	addr = (unsigned long)pmu->base + DBUS_OV_SYS_STATUS;
	sys_status = rtk_readl(addr);
	rtk_writel(addr, 0);

	/*
	 * Interrupts may be fired by memory trash function instead of counter
	 * overflow, reconfirm the necessity of overflow handling.
	 */
	if (!rtk_pmu_has_no_overflow(pmu)) {
		nop_count = (!sysh_status && !sys_status) ? nop_count + 1 : 0;

		if (nop_count == 0) {
			update_overflow_event(pmu, sysh_status, sys_status);
			handled = IRQ_HANDLED;
		} else if (nop_count >= NOP_THRESHOLD) {
			pr_err("interrupted with no overflow %d times",
			       nop_count);
			nop_count = 0;
		}
	}

	rtk_pmu_drv_pmc_inc(pmu, DBUS_REFRESH);
	rtk_writel((unsigned long)pmu->base + DBUS_SYSH_OVERFLOW_ENABLE,
		   sysh_ov_en);
	rtk_writel((unsigned long)pmu->base + DBUS_SYS_OVERFLOW_ENABLE,
		   sys_ov_en);

	return IRQ_RETVAL(handled);
}

int
rtk_dbus_find_ch_pmc(struct rtk_pmc_set *ps, int hwc, int target)
{
	if (target < ps->meta->nr_pmcgs &&
	    !test_and_set_bit(target, &ps->used_mask)) {
		ps->tracking[target].target = target;
		ps->tracking[target].config = hwc;
	}

	return target;
}
EXPORT_SYMBOL(rtk_dbus_find_ch_pmc);

static union pmc_config_desc
determine_pmc_config(union rtk_pmc_desc pmc, int gran, int width)
{
	union pmc_config_desc conf = {.val = -1};

	if (gran <= 0 || width <= 0)
		return conf;

	conf.set = pmc.set;
	conf.width = width;
	conf.idx = pmc.idx / gran;
	conf.offset = (pmc.idx - (conf.idx * gran)) * width;

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

	dmsg("%s- event config %x:%x:%x:%x\n",
	     ps->pmu->name, conf.set, conf.idx, conf.offset, conf.width);

	rtk_event_set_pmc_desc(hwc, pmc.val);

	/* pmc address, including the usage offset */
	rtk_event_set_pmc_addr(hwc, (unsigned long)ps->base +
			       ps->meta->pmcgs[pmc.idx] + (pmc.usage << 2));

	if (is_valid_pmc_conf(conf)) {
		/* control register address */
		rtk_event_set_config_addr(hwc, (unsigned long)ps->base +
					  ps->meta->configs[conf.idx]);

		/* mask of getting target from control register */
		rtk_event_set_target_mask(hwc,
			GENMASK(conf.offset + conf.width - 1, conf.offset));

		/* monitored target */
		rtk_event_set_config(hwc,
				     (rtk_dbus_pmc_target(event->attr.config) |
				      (pmc.ch << (conf.width - 1))) <<
				     conf.offset);

		rtk_event_set_config(hwc, rtk_event_config(event) &
				     rtk_event_target_mask(event));
	}

	if (rtk_pmu_need_update_ctrl(ps->pmu)) {
		rtk_event_set_pmc_update_ctrl(hwc, (unsigned long)ps->base +
					      ((pmc.set == PMC_SET__DBUS_SYSH) ?
					      DBUS_UPDATE_SYSH_CTRL :
					      DBUS_UPDATE_SYS_CTRL));
		rtk_event_set_pmc_update(hwc,
					 BIT(__pmc_to_ctrl_bit(ps->pmu, pmc)));

		dmsg("ev:%#llx update ctrl:%lx, update:%x\n",
		     event->attr.config, rtk_event_pmc_update_ctrl(event),
		     rtk_event_pmc_update(event));
	}

	if (IS_ENABLED(CONFIG_RTK_PMU_DEV))
		rtk_event_set_pmc_ts(hwc, ktime_get_mono_fast_ns());

	dmsg("%s- ev:%#llx, config:%#llx, cbase:%#lx, ebase:%#lx, rdpmc:%#x\n",
	     ps->pmu->name, event->attr.config, rtk_event_config(event),
	     rtk_event_config_addr(event), rtk_event_pmc_addr(event),
	     rtk_event_target_mask(event));
}

void
rtk_dbus_set_hwc(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
	       struct perf_event *event)
{
	__set_hwc(ps, pmc, event);

	rtk_event_set_pmc_mask(&event->hw, ps->meta->val_mask[pmc.usage]);
	rtk_event_set_pmc_threshold(&event->hw, ps->meta->ov_th[pmc.usage]);
}

static void
__rtk_dbus_update_mode_reg(struct rtk_pmc_set *ps, unsigned long config_addr,
			   union rtk_pmc_desc pmc, unsigned int mode, int op)
{
	u32 mode_reg = rtk_readl(config_addr);

	if (op == RTK_PMU_CTRL_SET) {
		mode_reg |= mode << (pmc.idx * DBUS_MON_MODE_CTRL_WIDTH);
	} else {
		mode_reg &= ~(DBUS_MON_MODE_CTRL_MASK
			      << (pmc.idx * DBUS_MON_MODE_CTRL_WIDTH));
	}

	rtk_writel(config_addr, mode_reg);
}

void
rtk_dbus_update_mode_reg(struct rtk_pmc_set *ps, struct perf_event *event,
			 int op)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	union rtk_dbus_event_desc desc = get_event_desc(event);
	unsigned long config_addr = (unsigned long)ps->base +
		(pmc.set == PMC_SET__DBUS_SYSH ?
		DBUS_SYSH_MODE_CTRL : DBUS_SYS_MODE_CTRL);

	if ((pmc.set == PMC_SET__DBUS_SYSH || pmc.set == PMC_SET__DBUS_SYS) &&
	    desc.mode != 0)
		__rtk_dbus_update_mode_reg(ps, config_addr, pmc, desc.mode, op);
}

struct rtk_pmc_set *
rtk_dbus_get_pmc_set(struct rtk_pmu *pmu, struct perf_event *event)
{
	union rtk_dbus_event_desc desc = get_event_desc(event);

	return (desc.set < pmu->nr_pmcss) ? pmu->pmcss[desc.set] : NULL;
}

int
rtk_dbus_check_event(struct rtk_pmu *pmu, struct perf_event *event)
{
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	union rtk_dbus_event_desc desc = get_event_desc(event);
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
__set_dbus_op(struct rtk_pmc_set *ps)
{
	if (rtk_pmu_has_no_overflow(ps->pmu)) {
		ps->reset = rtk_dbus_ps_reset;
		ps->read_counter = rtk_read_counter;
	} else {
		ps->enable = rtk_dbus_ps_enable_with_ov;
		ps->disable = rtk_dbus_ps_disable_with_ov;
		ps->reset = rtk_dbus_ps_reset_with_ov;
		ps->read_counter = rtk_read_counter_with_update;
	}
	ps->pmc_config = rtk_dbus_pmc_config;
	ps->arrange_pmc = rtk_ps_find_pmc;
	ps->release_pmc = rtk_ps_release_pmc;

	ps->set_perf_hwc = rtk_dbus_set_hwc;
	ps->start_pmc = rtk_ps_start_pmc;
	ps->stop_pmc = rtk_ps_stop_pmc;
	ps->read_pmc = rtk_ps_read_pmc;
}

int
rtk_dbus_ps_init(const struct rtk_pmc_set_meta *meta, struct rtk_pmc_set *ps)
{
	/* basic ops */
	__set_dbus_op(ps);

	/*
	 * Additional ops. DBUS SYSH support read/write mode filter.
	 * Stark also supporots DBUS SYS read/write mode filter.
	 */
	if (!(meta->type == PMC_SET__DBUS_SYS) ||
	    !(IS_ENABLED(CONFIG_ARCH_RTD16xx) ||
	      IS_ENABLED(CONFIG_ARCH_RTD13xx)))
		ps->extra_config = rtk_dbus_update_mode_reg;

	return 0;
}
EXPORT_SYMBOL(rtk_dbus_ps_init);

int
rtk_dbus_ch_init(const struct rtk_pmc_set_meta *meta, struct rtk_pmc_set *ps)
{
	__set_dbus_op(ps);

	/* override arrange_pmc callback */
	ps->arrange_pmc = rtk_dbus_find_ch_pmc;

	return 0;
}
EXPORT_SYMBOL(rtk_dbus_ch_init);

int
rtk_dbus_drv_init(const struct rtk_pmc_set_meta *meta, struct rtk_pmc_set *ps)
{
	ps->pmc_config = rtk_dbus_pmc_config;
	ps->release_pmc = rtk_ps_release_pmc;

	/* override arrange_pmc callback */
	ps->arrange_pmc = rtk_find_drv_pmc;

	ps->set_perf_hwc = rtk_set_drv_hwc;
	ps->start_pmc = rtk_start_drv_pmc;
	ps->stop_pmc = rtk_stop_drv_pmc;
	ps->read_pmc = rtk_read_drv_pmc;

	return 0;
}
EXPORT_SYMBOL(rtk_dbus_drv_init);

int rtk_dbus_pmu_init(struct rtk_pmu *pmu, struct device_node *dt,
		      const char *name, unsigned long ctrl_offset,
		      const struct attribute_group **attr_groups,
		      const struct rtk_pmc_set_meta *meta, int nr_ps)
{
	int ret;

	ret = rtk_uncore_pmu_init(pmu, dt, name, RTK_DBUS_PMU_CPUHP_NAME,
				  ctrl_offset, attr_groups,
				  meta, nr_ps, DBUS_EV_NUM);
	if (ret)
		return ret;

	pmu->get_pmc_set = rtk_dbus_get_pmc_set;
	pmu->is_valid_event = rtk_dbus_check_event;
	pmu->enable = rtk_dbus_enable;
	pmu->disable = rtk_dbus_disable;

	/* set overflow mode */
	if (rtk_pmu_has_no_overflow(pmu)) {
		pmu->refresh = rtk_dbus_refresh;
		rtk_writel(pmu->ctrl, rtk_readl(pmu->ctrl) &
			   ~BIT(DBUS_CTRL_CNT_FREERUN_BIT));
	} else {
		/*
		 * Dbus need to raise update signal before reading counter when
		 * under freerun mode
		 */
		pmu->flags |= RTK_PMU_NEED_UPDATE_CTRL;

		ret = request_irq(pmu->irq, overflow_handler, IRQF_SHARED, name,
				  pmu);
		if (ret < 0)
			pr_err("%s request IRQ:%d failed(%d)\n", name, pmu->irq,
			       ret);

		rtk_writel(pmu->ctrl, rtk_readl(pmu->ctrl) |
			   BIT(DBUS_CTRL_CNT_FREERUN_BIT));

		/* clear PMCG enable */
		rtk_writel((unsigned long)pmu->base + DBUS_PMCG_ENABLE, 0);
	}

	return ret;
}
EXPORT_SYMBOL(rtk_dbus_pmu_init);

static const struct of_device_id rtk_pmu_of_device_ids[] = {
	{
		.compatible = "realtek,rtk-16xxb-dbus-pmu",
		.data = rtk_16xxb_dbus_init
	},
	{},
};

static int
rtk_dbus_pmu_probe(struct platform_device *pdev)
{
	return rtk_pmu_device_probe(pdev, rtk_pmu_of_device_ids);
}

static int
rtk_dbus_pmu_remove(struct platform_device *pdev)
{
	return rtk_pmu_device_remove(pdev);
}

static struct platform_driver rtk_dbus_pmu_driver = {
	.driver         = {
		.name   = RTK_DBUS_PMU_PDEV_NAME,
		.of_match_table = rtk_pmu_of_device_ids,
	},
	.probe          = rtk_dbus_pmu_probe,
	.remove		= rtk_dbus_pmu_remove,
};

module_platform_driver(rtk_dbus_pmu_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RTK DBUS PMU support");
