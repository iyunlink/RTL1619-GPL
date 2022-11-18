// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek uncore PMU driver for traffic event
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

#define pr_fmt(fmt)	"[RTK_PMU] " fmt

#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/module.h>

#include "rtk_uncore_pmu.h"
#include "rtk_tfc_pmu.h"


RTK_PMU_FORMAT_ATTR(event,	"config:0-11");

static struct attribute *rtk_tfc_format_attrs[] = {
	RTK_PMU_FORMAT_REF(event),
	NULL
};

struct attribute_group rtk_tfc_format_attr_group = {
	.name = "format",
	.attrs = rtk_tfc_format_attrs,
};
EXPORT_SYMBOL(rtk_tfc_format_attr_group);

ssize_t rtk_tfc_thr_read(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct rtk_pmu *pmu = to_rtk_pmu(dev_get_drvdata(dev));
	struct rtk_tfc_sysfs_attr *tfc_attr =
		container_of(attr, struct rtk_tfc_sysfs_attr, attr);
	unsigned long addr = (unsigned long)pmu->base +
		pmu->pmcss[tfc_attr->id]->meta->configs[TFC_CFG_TH];

	return sprintf(buf, "0x%x\n", __rtk_tfc_thr_read(addr));
}

ssize_t rtk_tfc_thr_write(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct rtk_pmu *pmu = to_rtk_pmu(dev_get_drvdata(dev));
	struct rtk_tfc_sysfs_attr *tfc_attr =
		container_of(attr, struct rtk_tfc_sysfs_attr, attr);
	unsigned long addr = (unsigned long)pmu->base +
		pmu->pmcss[tfc_attr->id]->meta->configs[TFC_CFG_TH];
	unsigned long val;
	ssize_t ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	__rtk_tfc_thr_write(addr, val);

	return count;
}

ssize_t rtk_tfc_rto_read(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct rtk_pmu *pmu = to_rtk_pmu(dev_get_drvdata(dev));
	struct rtk_tfc_sysfs_attr *tfc_attr =
		container_of(attr, struct rtk_tfc_sysfs_attr, attr);
	unsigned long addr = (unsigned long)pmu->base +
		pmu->pmcss[tfc_attr->id]->meta->configs[TFC_CFG_TH];

	return sprintf(buf, "0x%x\n", __rtk_tfc_rto_read(addr));
}

ssize_t rtk_tfc_rto_write(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct rtk_pmu *pmu = to_rtk_pmu(dev_get_drvdata(dev));
	struct rtk_tfc_sysfs_attr *tfc_attr =
		container_of(attr, struct rtk_tfc_sysfs_attr, attr);
	unsigned long addr = (unsigned long)pmu->base +
		pmu->pmcss[tfc_attr->id]->meta->configs[TFC_CFG_TH];
	unsigned long val;
	ssize_t ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	__rtk_tfc_rto_write(addr, val);

	return count;
}

ssize_t rtk_tfc_wto_read(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct rtk_pmu *pmu = to_rtk_pmu(dev_get_drvdata(dev));
	struct rtk_tfc_sysfs_attr *tfc_attr =
		container_of(attr, struct rtk_tfc_sysfs_attr, attr);
	unsigned long addr = (unsigned long)pmu->base +
		pmu->pmcss[tfc_attr->id]->meta->configs[TFC_CFG_TH];

	return sprintf(buf, "0x%x\n", __rtk_tfc_wto_read(addr));
}

ssize_t rtk_tfc_wto_write(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct rtk_pmu *pmu = to_rtk_pmu(dev_get_drvdata(dev));
	struct rtk_tfc_sysfs_attr *tfc_attr =
		container_of(attr, struct rtk_tfc_sysfs_attr, attr);
	unsigned long addr = (unsigned long)pmu->base +
		pmu->pmcss[tfc_attr->id]->meta->configs[TFC_CFG_TH];
	unsigned long val;
	ssize_t ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	__rtk_tfc_wto_write(addr, val);

	return count;
}

void
rtk_tfc_ps_reset(struct rtk_pmc_set *ps, struct perf_event *event)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);

	rtk_writel((unsigned long)ps->base + ps->meta->configs[TFC_CFG_CLEAR],
		   BIT(__pmc_to_ctrl_bit(pmc)));
}

void
rtk_tfc_ps_enable(struct rtk_pmc_set *ps, struct perf_event *event)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	unsigned long addr;
	u32 ctrl;

	/* enable overflow irq */
	addr = (unsigned long)ps->base + ps->meta->configs[TFC_CFG_INT];
	ctrl = rtk_readl(addr) | BIT(__pmc_to_ctrl_bit(pmc));
	rtk_writel(addr, ctrl);

	/* enable TFC ch and pmc */
	addr = (unsigned long)ps->base + ps->meta->configs[TFC_CFG_CTRL];
	ctrl = rtk_readl(addr) | BIT(TFC_CTRL_EN_BIT) |
		BIT(pmc.idx + TFC_CTRL_PMC_EN_OFFSET);
	rtk_writel(addr, ctrl);
}

void
rtk_tfc_ps_disable(struct rtk_pmc_set *ps, struct perf_event *event)
{
	union rtk_pmc_desc pmc = rtk_event_pmc_desc(event);
	unsigned long addr;
	u32 ctrl;

	/* disable overflow irq */
	addr = (unsigned long)ps->base + ps->meta->configs[TFC_CFG_INT];
	ctrl = rtk_readl(addr) & ~BIT(__pmc_to_ctrl_bit(pmc));
	rtk_writel(addr, ctrl);

	/* disable TFC pmc and disable whole TFC channel if no active */
	addr = (unsigned long)ps->base + ps->meta->configs[TFC_CFG_CTRL];
	ctrl = rtk_readl(addr) & ~BIT(pmc.idx + TFC_CTRL_PMC_EN_OFFSET);
	if (!(ctrl & GENMASK(ps->meta->nr_pmcgs, 1)))
		ctrl &= ~BIT(TFC_CTRL_EN_BIT);
	rtk_writel(addr, ctrl);
}

static int
rtk_tfc_pmc_config(u64 config)
{
	return (int)(config & HWC_MASK);
}

struct rtk_pmc_set *
rtk_tfc_get_pmc_set(struct rtk_pmu *pmu, struct perf_event *event)
{
	union rtk_tfc_event_desc desc = get_event_desc(event);

	return (desc.set < pmu->nr_pmcss) ? pmu->pmcss[desc.set] : NULL;
}

static void
update_overflow_event(struct rtk_pmu *pmu, unsigned long status)
{
	struct rtk_pmc_set *ps;
	struct perf_event *ev;
	int bit, idx, size;

	size = (pmu->nr_pmcss - 1) << TFC_CH_CTRL_BITS;
	for_each_set_bit(bit, &status, size) {
		ps = pmu->pmcss[bit >> TFC_CH_CTRL_BITS];
		idx = bit & GENMASK(TFC_CH_CTRL_BITS - 1, 0);
		ev = ps->tracking[idx].events[0];
		compensate_event(ps, ev);
	}
}

static irqreturn_t
overflow_handler(int irq, void *rpmu)
{
#define NOP_THRESHOLD	3

	struct rtk_pmu *pmu = rpmu;
	struct rtk_pmc_set *ps = pmu->pmcss[0];
	bool handled = IRQ_NONE;
	u32 ov_status;
	u32 ov_en;
	unsigned long addr;
	static unsigned int nop_count;

	/* save overflow interrupt */
	addr = (unsigned long)pmu->base + ps->meta->configs[TFC_CFG_INT];
	ov_en = rtk_readl(addr);
	rtk_writel(addr, 0);

	addr = (unsigned long)pmu->base + ps->meta->configs[TFC_CFG_OVERFLOW];
	ov_status = rtk_readl(addr);
	rtk_writel(addr, 0);

	/*
	 * Interrupts may be fired by memory trash function instead of counter
	 * overflow, reconfirm the necessity of overflow handling.
	 */
	if (!rtk_pmu_has_no_overflow(pmu)) {
		nop_count = !ov_status ? nop_count + 1 : 0;

		if (nop_count == 0) {
			update_overflow_event(pmu, ov_status);
			handled = IRQ_HANDLED;
		} else if (nop_count >= NOP_THRESHOLD) {
			pr_err("interrupted without overflow for %d times",
			       nop_count);
			nop_count = 0;
		}
	}

	rtk_pmu_drv_pmc_inc(pmu, TFC_REFRESH);

	/* restore overflow interrupt */
	rtk_writel((unsigned long)pmu->base + ps->meta->configs[TFC_CFG_INT],
		   ov_en);

	return IRQ_RETVAL(handled);
}

static union pmc_config_desc
determine_pmc_config(union rtk_pmc_desc pmc, int gran, int width)
{
	union pmc_config_desc conf;

	WARN(pmc.idx >= gran,
	     "pmc index(%d) out of TFC set(%d)\n", pmc.idx, gran);
	conf.set = pmc.set;
	conf.width = width;
	conf.idx = TFC_CFG_CTRL;
	conf.offset = pmc.idx * width + TFC_CTRL_PMC_SEL_OFFSET;

	return conf;
}

static void
rtk_tfc_set_hwc(struct rtk_pmc_set *ps, union rtk_pmc_desc pmc,
		struct perf_event *event)
{
	unsigned long addr;
	struct hw_perf_event *hwc = &event->hw;
	union pmc_config_desc conf;

	dmsg("%s:%s- ev:%#llx set hwc, pmc idx:%d, usage:%d\n",
	     ps->pmu->name, ps->name, event->attr.config, pmc.idx, pmc.usage);

	conf = determine_pmc_config(pmc, ps->meta->config_size,
				    ps->meta->config_width);

	rtk_event_set_pmc_desc(hwc, pmc.val);

	/* counter address */
	rtk_event_set_pmc_addr(&event->hw, (unsigned long)ps->base +
			       ps->meta->pmcgs[pmc.idx]);

	/* control register address */
	rtk_event_set_config_addr(hwc,
				  (unsigned long)ps->base +
				  ps->meta->configs[conf.idx]);

	/* mask of control register */
	rtk_event_set_target_mask(hwc,
				  GENMASK(conf.offset + conf.width - 1,
					  conf.offset));

	/* monitored target */
	rtk_event_set_config(hwc, (rtk_tfc_pmc_target(event->attr.config) <<
				   conf.offset) & rtk_event_target_mask(event));

	if (IS_ENABLED(CONFIG_RTK_PMU_DEV))
		rtk_event_set_pmc_ts(hwc, ktime_get_mono_fast_ns());

	dmsg("%s- ev:%#llx, cconf:%#llx, cbase:%#lx, ebase:%#lx, rdpmc:%#x",
	     ps->pmu->name, event->attr.config, hwc->config,
	     rtk_event_config_addr(event), rtk_event_pmc_addr(event),
	     rtk_event_target_mask(event));

	/* set proper value mask and overflow threshold */
	rtk_event_set_pmc_mask(&event->hw, RTK_PMU_VAL_MASK(32));
	rtk_event_set_pmc_threshold(&event->hw, RTK_PMU_OVERFLOW_TH(32));

	/* setup PMC update control */
	addr = (unsigned long)ps->base + ps->meta->configs[TFC_CFG_UPDATE];
	rtk_event_set_pmc_update_ctrl(&event->hw, addr);
	rtk_event_set_pmc_update(&event->hw, BIT(__pmc_to_ctrl_bit(pmc)));

	dmsg("%s- ev:%#llx, pmc=%d:%d:%d update ctrl:%lx, update val:%x\n",
	     ps->pmu->name, event->attr.config, pmc.set, pmc.idx, pmc.usage,
	     rtk_event_pmc_update_ctrl(event),
	     rtk_event_pmc_update(event));
}

union rtk_pmc_desc
rtk_tfc_arrange_pmc(struct rtk_pmu *pmu, u64 config)
{
	union rtk_pmc_desc pmc;
	union rtk_tfc_event_desc desc = __get_event_desc(config);
	struct rtk_pmc_set *ps = pmu->pmcss[desc.set];
	int idx = ps->arrange_pmc(ps,
				  rtk_tfc_pmc_config(config),
				  rtk_tfc_pmc_target(config));

	if (idx < 0) {
		pmc.val = -EAGAIN;
	} else {
		pmc.set = desc.set;
		pmc.idx = idx;
	}

	dmsg("%s:%s- ev:%#llx, desc %#x:%#x:%#x\n",
	     pmu->name, ps->name, config, desc.set, desc.cat, desc.sid);
	dmsg("%s:%s- event pmc set:%#x, idx:%#x, usage:%#x\n",
	     pmu->name, ps->name, pmc.set, pmc.idx, pmc.usage);
	return pmc;
}

static int
__check_by_cat(struct rtk_pmc_set *ps, union rtk_tfc_event_desc desc)
{
	static const unsigned int sid_max[TFC_CAT__NUM] = {
		[TFC_CAT__PHASE]	= TFC_PHASE__NUM,
		[TFC_CAT__TIME]		= TFC_TIME__NUM,
		[TFC_CAT__TH]		= TFC_TH__NUM,
		[TFC_CAT__PHASE_DATA]	= TFC_PHASE_DATA__NUM,
		[TFC_CAT__PHASE_CHG]	= TFC_PHASE_CHG__NUM
	};

	return ((desc.cat < TFC_CAT__NUM) && (desc.sid < sid_max[desc.cat])) ?
		true : false;
}

static int
__check_by_client(struct rtk_pmc_set *ps, union rtk_tfc_event_desc desc)
{
	int i;

	/* check event validity */
	for (i = 0; i < ps->meta->nr_clients; i++) {
		if (desc.sid == ps->meta->clients[i])
			return true;
	}

	return false;
}

int
rtk_tfc_check_event(struct rtk_pmu *pmu, struct perf_event *event)
{
	struct rtk_pmc_set *ps = pmu->get_pmc_set(pmu, event);
	union rtk_tfc_event_desc desc = get_event_desc(event);
	int ret = false;

	if (ps == NULL) {
		dmsg("%s- invalid ev:%llx\n", pmu->name, event->attr.config);
		return false;
	}

	ret = ps->type == PMC_SET__TFC ?
		__check_by_cat(ps, desc) : __check_by_client(ps, desc);

	if (ret == false)
		dmsg("%s- invalid ev:%#llx, sid:%#x, cat:%#x, set:%d\n",
		     pmu->name, event->attr.config, desc.sid, desc.cat,
		     desc.set);

	return ret;
}

static inline void
__set_op(struct rtk_pmc_set *ps)
{
	ps->pmc_config = rtk_tfc_pmc_config;
	ps->release_pmc = rtk_ps_release_pmc;
}

/* init PMCG set */
int
rtk_tfc_ps_init(const struct rtk_pmc_set_meta *meta,
		struct rtk_pmc_set *ps)
{
	/* basic ops */
	__set_op(ps);

	ps->enable = rtk_tfc_ps_enable;
	ps->disable = rtk_tfc_ps_disable;
	ps->reset = rtk_tfc_ps_reset;

	ps->set_perf_hwc = rtk_tfc_set_hwc;
	ps->arrange_pmc = rtk_ps_find_pmc;
	ps->start_pmc = rtk_ps_start_pmc;
	ps->stop_pmc = rtk_ps_stop_pmc;
	ps->read_pmc = rtk_ps_read_pmc;
	ps->read_counter = rtk_read_counter_with_update;

	return 0;
}
EXPORT_SYMBOL(rtk_tfc_ps_init);

/* init driver counter set */
int
rtk_tfc_ps_drv_init(const struct rtk_pmc_set_meta *meta,
		    struct rtk_pmc_set *ps)
{
	__set_op(ps);

	ps->set_perf_hwc = rtk_set_drv_hwc;
	ps->arrange_pmc = rtk_find_drv_pmc;
	ps->start_pmc = rtk_start_drv_pmc;
	ps->stop_pmc = rtk_stop_drv_pmc;
	ps->read_pmc = rtk_read_drv_pmc;

	return 0;
}
EXPORT_SYMBOL(rtk_tfc_ps_drv_init);

int
rtk_tfc_pmu_init(struct rtk_pmu *pmu, struct device_node *dt,
		 const char *name, unsigned long ctrl_offset,
		 const struct attribute_group **attr_groups,
		 const struct rtk_pmc_set_meta *meta, int nr_ps)
{
	int ret;

	ret = rtk_uncore_pmu_init(pmu, dt, name, RTK_TFC_PMU_CPUHP_NAME,
				  ctrl_offset, attr_groups,
				  meta, nr_ps, TFC_EV_NUM);

	if (ret)
		return ret;

	pmu->get_pmc_set	= rtk_tfc_get_pmc_set;
	pmu->is_valid_event	= rtk_tfc_check_event;
	pmu->arrange_pmc	= rtk_tfc_arrange_pmc;

	/*
	 * TFC always uses overflow interrupt to prevent counter overflow and
	 * shadow register to get counter value.
	 * Force the corresponding flags to avoid problems when overflow
	 * property is absent in the dts.
	 */
	pmu->pmu.capabilities &= ~PERF_PMU_CAP_NO_INTERRUPT;
	pmu->flags |= RTK_PMU_NEED_UPDATE_CTRL;

	/* irq number maybe uninitialized due to missing overflow, try again */
	if (pmu->irq == 0)
		pmu->irq = irq_of_parse_and_map(dt, 0);

	ret = request_irq(pmu->irq, overflow_handler, IRQF_SHARED, name, pmu);
	if (ret < 0)
		pr_err("%s request IRQ:%d failed(%d)\n", name, pmu->irq, ret);

	return ret;
}
EXPORT_SYMBOL(rtk_tfc_pmu_init);

static const struct of_device_id rtk_tfc_of_device_ids[] = {
	{
		.compatible = "realtek,rtk-16xxb-ddrc-tfc-pmu",
		.data = rtk_16xxb_ddrc_tfc_init
	},
	{},
};

static int
rtk_tfc_pmu_probe(struct platform_device *pdev)
{
	return rtk_pmu_device_probe(pdev, rtk_tfc_of_device_ids);
}

static int
rtk_tfc_pmu_remove(struct platform_device *pdev)
{
	return rtk_pmu_device_remove(pdev);
}

static struct platform_driver rtk_tfc_pmu_driver = {
	.driver         = {
		.name   = RTK_TFC_PMU_PDEV_NAME,
		.of_match_table = rtk_tfc_of_device_ids,
	},
	.probe          = rtk_tfc_pmu_probe,
	.remove		= rtk_tfc_pmu_remove
};

module_platform_driver(rtk_tfc_pmu_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RTK Traffic PMU support");
