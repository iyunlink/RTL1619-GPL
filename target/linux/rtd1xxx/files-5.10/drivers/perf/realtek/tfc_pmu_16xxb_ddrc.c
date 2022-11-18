// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek uncore PMU driver for 16xxb TFC of DDRC
 *
 * Copyright (C) 2021-2024 Realtek Semiconductor Corporation
 * Copyright (C) 2021-2024 Ping-Hsiung Chiu <phelic@realtek.com>
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

#include "rtk_uncore_pmu.h"
#include "rtk_tfc_pmu.h"

/* TFC does not have a global control, just take control of first channel */
#define _TFC_CTRL_OFFSET_		0x00a0


/* TFC events */
TFC_EVENT_GROUP(rt, 0);
TFC_EVENT_GROUP(nrt, 1);
TFC_EVENT_GROUP(npu, 2);
TFC_EVENT_GROUP(vcpu, 3);
TFC_EVENT_GROUP(gpu, 4);
TFC_EVENT_GROUP(scpu, 5);

/* Driver statistics events */
TFC_DRV_EVENT_ATTR(tfc_overflow,	RTK_DRV_OVERFLOW, 6);
TFC_DRV_EVENT_ATTR(tfc_refresh,		TFC_REFRESH, 6);

static struct attribute *rtk_16xxb_ddrc_tfc_event_attrs[] = {
	TFC_DRV_EVENT_REF(tfc_refresh),
	TFC_DRV_EVENT_REF(tfc_overflow),

	TFC_EVENT_REF_GROUP(rt),
	TFC_EVENT_REF_GROUP(nrt),
	TFC_EVENT_REF_GROUP(npu),
	TFC_EVENT_REF_GROUP(vcpu),
	TFC_EVENT_REF_GROUP(gpu),
	TFC_EVENT_REF_GROUP(scpu),

	NULL
};

static struct attribute_group rtk_16xxb_ddrc_tfc_event_attr_group = {
	.name = "events",
	.attrs = rtk_16xxb_ddrc_tfc_event_attrs,
};

static struct attribute *rtk_tfc_sysfs_attrs[] = {
	TFC_SYSFS_ATTR_GROUP(rt, 0),
	TFC_SYSFS_ATTR_GROUP(nrt, 1),
	TFC_SYSFS_ATTR_GROUP(npu, 2),
	TFC_SYSFS_ATTR_GROUP(vcpu, 3),
	TFC_SYSFS_ATTR_GROUP(gpu, 4),
	TFC_SYSFS_ATTR_GROUP(scpu, 5),
	NULL,
};

static struct attribute_group rtk_tfc_sysfs_attr_group = {
	.name = "settings",
	.attrs = rtk_tfc_sysfs_attrs,
};

static const struct attribute_group *rtk_16xxb_ddrc_tfc_attr_groups[] = {
	/* must be Null-terminated */
	[RTK_PMU_ATTR_GROUP__COMMON] = &rtk_pmu_common_attr_group,
	[RTK_PMU_ATTR_GROUP__FORMAT] = &rtk_tfc_format_attr_group,
	[RTK_PMU_ATTR_GROUP__EVENT] = &rtk_16xxb_ddrc_tfc_event_attr_group,
	[RTK_PMU_ATTR_GROUP__NUM] = &rtk_tfc_sysfs_attr_group,
	[RTK_PMU_ATTR_GROUP__NUM + 1] = NULL
};

static const unsigned long rtk_16xxb_ch0_tfc_pmcgs[] = {
	0xe0, 0xe4, 0xe8
};

static const unsigned long rtk_16xxb_ch0_tfc_configs[] = {
	[TFC_CFG_CTRL]		= 0xa0,
	[TFC_CFG_TH]		= 0xa4,
	[TFC_CFG_UPDATE]	= 0xd0,
	[TFC_CFG_CLEAR]		= 0xd4,
	[TFC_CFG_INT]		= 0xd8,
	[TFC_CFG_OVERFLOW]	= 0xdc
};

static const unsigned long rtk_16xxb_ch1_tfc_pmcgs[] = {
	0xec, 0xf0, 0xf4
};

static const unsigned long rtk_16xxb_ch1_tfc_configs[] = {
	[TFC_CFG_CTRL]		= 0xa8,
	[TFC_CFG_TH]		= 0xac,
	[TFC_CFG_UPDATE]	= 0xd0,
	[TFC_CFG_CLEAR]		= 0xd4,
	[TFC_CFG_INT]		= 0xd8,
	[TFC_CFG_OVERFLOW]	= 0xdc
};

static const unsigned long rtk_16xxb_ch2_tfc_pmcgs[] = {
	0xf8, 0xfc, 0x100
};

static const unsigned long rtk_16xxb_ch2_tfc_configs[] = {
	[TFC_CFG_CTRL]		= 0xb0,
	[TFC_CFG_TH]		= 0xb4,
	[TFC_CFG_UPDATE]	= 0xd0,
	[TFC_CFG_CLEAR]		= 0xd4,
	[TFC_CFG_INT]		= 0xd8,
	[TFC_CFG_OVERFLOW]	= 0xdc
};

static const unsigned long rtk_16xxb_ch3_tfc_pmcgs[] = {
	0x104, 0x108, 0x10c
};

static const unsigned long rtk_16xxb_ch3_tfc_configs[] = {
	[TFC_CFG_CTRL]		= 0xb8,
	[TFC_CFG_TH]		= 0xbc,
	[TFC_CFG_UPDATE]	= 0xd0,
	[TFC_CFG_CLEAR]		= 0xd4,
	[TFC_CFG_INT]		= 0xd8,
	[TFC_CFG_OVERFLOW]	= 0xdc
};

static const unsigned long rtk_16xxb_ch4_tfc_pmcgs[] = {
	0x110, 0x114, 0x118
};

static const unsigned long rtk_16xxb_ch4_tfc_configs[] = {
	[TFC_CFG_CTRL]		= 0xc0,
	[TFC_CFG_TH]		= 0xc4,
	[TFC_CFG_UPDATE]	= 0xd0,
	[TFC_CFG_CLEAR]		= 0xd4,
	[TFC_CFG_INT]		= 0xd8,
	[TFC_CFG_OVERFLOW]	= 0xdc
};

static const unsigned long rtk_16xxb_ch5_tfc_pmcgs[] = {
	0x11c, 0x120, 0x124
};

static const unsigned long rtk_16xxb_ch5_tfc_configs[] = {
	[TFC_CFG_CTRL]		= 0xc8,
	[TFC_CFG_TH]		= 0xcc,
	[TFC_CFG_UPDATE]	= 0xd0,
	[TFC_CFG_CLEAR]		= 0xd4,
	[TFC_CFG_INT]		= 0xd8,
	[TFC_CFG_OVERFLOW]	= 0xdc
};

static const unsigned int rtk_16xxb_ddrc_tfc_drv_ev[] = {
	RTK_DRV_OVERFLOW, TFC_REFRESH
};

#define _TFC_GR_SIZE	1
#define _TFC_CFG_SIZE	3
#define _TFC_CFG_WIDTH	8
const static struct rtk_pmc_set_meta rtk_16xxb_ddrc_tfc_ps_meta[] = {
	{
		.name = "16xxb TFC for DDRC RT",
		.type = PMC_SET__TFC,
		.init = rtk_tfc_ps_init,
		.group_size = _TFC_GR_SIZE,
		.config_size = _TFC_CFG_SIZE,
		.config_width = _TFC_CFG_WIDTH,

		.clients = NULL,
		.nr_clients = 0,
		.pmcgs = rtk_16xxb_ch0_tfc_pmcgs,
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_ch0_tfc_pmcgs),
		.configs = rtk_16xxb_ch0_tfc_configs,
		.nr_configs = ARRAY_SIZE(rtk_16xxb_ch0_tfc_configs),
	},
	{
		.name = "16xxb TFC for DDRC NRT",
		.type = PMC_SET__TFC,
		.init = rtk_tfc_ps_init,
		.group_size = _TFC_GR_SIZE,
		.config_size = _TFC_CFG_SIZE,
		.config_width = _TFC_CFG_WIDTH,

		.clients = NULL,
		.nr_clients = 0,
		.pmcgs = rtk_16xxb_ch1_tfc_pmcgs,
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_ch1_tfc_pmcgs),
		.configs = rtk_16xxb_ch1_tfc_configs,
		.nr_configs = ARRAY_SIZE(rtk_16xxb_ch1_tfc_configs),
	},
	{
		.name = "16xxb TFC for DDRC NPU",
		.type = PMC_SET__TFC,
		.init = rtk_tfc_ps_init,
		.group_size = _TFC_GR_SIZE,
		.config_size = _TFC_CFG_SIZE,
		.config_width = _TFC_CFG_WIDTH,

		.clients = NULL,
		.nr_clients = 0,
		.pmcgs = rtk_16xxb_ch2_tfc_pmcgs,
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_ch2_tfc_pmcgs),
		.configs = rtk_16xxb_ch2_tfc_configs,
		.nr_configs = ARRAY_SIZE(rtk_16xxb_ch2_tfc_configs),
	},
	{
		.name = "16xxb TFC for DDRC VCPU",
		.type = PMC_SET__TFC,
		.init = rtk_tfc_ps_init,
		.group_size = _TFC_GR_SIZE,
		.config_size = _TFC_CFG_SIZE,
		.config_width = _TFC_CFG_WIDTH,

		.clients = NULL,
		.nr_clients = 0,
		.pmcgs = rtk_16xxb_ch3_tfc_pmcgs,
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_ch3_tfc_pmcgs),
		.configs = rtk_16xxb_ch3_tfc_configs,
		.nr_configs = ARRAY_SIZE(rtk_16xxb_ch3_tfc_configs),
	},
	{
		.name = "16xxb TFC for DDRC GPU",
		.type = PMC_SET__TFC,
		.init = rtk_tfc_ps_init,
		.group_size = _TFC_GR_SIZE,
		.config_size = _TFC_CFG_SIZE,
		.config_width = _TFC_CFG_WIDTH,

		.clients = NULL,
		.nr_clients = 0,
		.pmcgs = rtk_16xxb_ch4_tfc_pmcgs,
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_ch4_tfc_pmcgs),
		.configs = rtk_16xxb_ch4_tfc_configs,
		.nr_configs = ARRAY_SIZE(rtk_16xxb_ch4_tfc_configs),
	},
	{
		.name = "16xxb TFC for DDRC SCPU",
		.type = PMC_SET__TFC,
		.init = rtk_tfc_ps_init,
		.group_size = _TFC_GR_SIZE,
		.config_size = _TFC_CFG_SIZE,
		.config_width = _TFC_CFG_WIDTH,

		.clients = NULL,
		.nr_clients = 0,
		.pmcgs = rtk_16xxb_ch5_tfc_pmcgs,
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_ch5_tfc_pmcgs),
		.configs = rtk_16xxb_ch5_tfc_configs,
		.nr_configs = ARRAY_SIZE(rtk_16xxb_ch5_tfc_configs),
	},
	{
		.name = "16xxb TFC DRV",
		.type = PMC_SET__TFC_DRV,
		.init = rtk_tfc_ps_drv_init,
		.group_size = 1,	/* direct map */

		.nr_clients = ARRAY_SIZE(rtk_16xxb_ddrc_tfc_drv_ev),
		.clients = rtk_16xxb_ddrc_tfc_drv_ev,
		.nr_pmcgs = TFC_EV_NUM,
		.pmcgs = NULL,
		.nr_configs = 0,
		.configs = NULL,
	},
	{},
};

static void
__reset_tfc_channel(struct rtk_pmu *pmu)
{
	static const int ch_mode[] = {
		[0] = 0,
		[1] = 0,
		[2] = 0,
		[3] = 0,
		[4] = BIT(TFC_CTRL_MODE_BIT),
		[5] = BIT(TFC_CTRL_MODE_BIT),
	};
	struct rtk_pmc_set *ps;
	int i;

	/* reset tfc of each channel according client type */
	for (i = 0; i < pmu->nr_pmcss && (i < ARRAY_SIZE(ch_mode)); i++) {
		ps = pmu->pmcss[i];
		/* clear PMCG enable and update tag mode */
		rtk_writel((unsigned long)pmu->base +
			   ps->meta->configs[TFC_CFG_CTRL], ch_mode[i]);
	}
}

int rtk_16xxb_ddrc_tfc_init(struct rtk_pmu *pmu, struct device_node *dt)
{
	int ret = rtk_tfc_pmu_init(pmu, dt,
				   "rtk_16xxb_ddrc_tfc_pmu",
				   _TFC_CTRL_OFFSET_,
				   rtk_16xxb_ddrc_tfc_attr_groups,
				   rtk_16xxb_ddrc_tfc_ps_meta,
				   RTK_PMU_META_NR(rtk_16xxb_ddrc_tfc_ps_meta));

	__reset_tfc_channel(pmu);

	return ret;
}
EXPORT_SYMBOL(rtk_16xxb_ddrc_tfc_init);
MODULE_LICENSE("GPL");
