// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek uncore PMU driver for 16xxb register bus
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
#include "rtk_rbus_pmu.h"


#define RBUS_SCPU		0
#define RBUS_ACPU		1
#define RBUS_VCPU		2
#define RBUS_PCPU		3
#define RBUS_AUCPU		4
#define RBUS_SCPU_PERIF		6


/* counter event */
RBUS_EVENT_GROUP_V2(scpu,		RBUS_SCPU, RBUS);
RBUS_EVENT_GROUP_V2(acpu,		RBUS_ACPU, RBUS);
RBUS_EVENT_GROUP_V2(vcpu,		RBUS_VCPU, RBUS);
RBUS_EVENT_GROUP_V2(pcpu,		RBUS_PCPU, RBUS);
RBUS_EVENT_GROUP_V2(aucpu,		RBUS_AUCPU, RBUS);
RBUS_EVENT_GROUP_V2(scpu_perif,		RBUS_SCPU_PERIF, RBUS);


/* driver event */
RBUS_DRV_EVENT_ATTR(rbus_overflow,	RTK_DRV_OVERFLOW);
RBUS_DRV_EVENT_ATTR(rbus_refresh,	RBUS_REFRESH);


static struct attribute *rtk_16xxb_rbus_event_attrs[] = {
	RBUS_DRV_EVENT_REF(rbus_refresh),
	RBUS_DRV_EVENT_REF(rbus_overflow),

	RBUS_EVENT_REF_GROUP_V2(scpu),
	RBUS_EVENT_REF_GROUP_V2(acpu),
	RBUS_EVENT_REF_GROUP_V2(vcpu),
	RBUS_EVENT_REF_GROUP_V2(pcpu),
	RBUS_EVENT_REF_GROUP_V2(aucpu),
	RBUS_EVENT_REF_GROUP_V2(scpu_perif),

	NULL
};

static const unsigned int rtk_16xxb_rbus_clients[] = {
	RBUS_SCPU,
	RBUS_ACPU,
	RBUS_VCPU,
	RBUS_PCPU,
	RBUS_AUCPU,
	RBUS_SCPU_PERIF,
};

static const unsigned long rtk_16xxb_rbus_pmcgs[] = {
	0x0000,	/* Offset of SCPU counters starting address */
	0x0010, /* Offset of ACPU counters starting address */
	0x0020,	/* Offset of VCPU counters starting address */
	0x0030, /* Offset of PCPU counters starting address */
	0x0040, /* Offset of ACPU counters starting address */
	0x0060,	/* Offset of SCPU_PERIF counters starting address */
};

static const unsigned long rtk_rbus_configs[] = {
	0x0070
};

static const unsigned int rtk_rbus_drv_ev[] = {
	RTK_DRV_OVERFLOW, RBUS_REFRESH
};

static struct attribute_group rtk_16xxb_rbus_event_attr_group = {
	.name = "events",
	.attrs = rtk_16xxb_rbus_event_attrs,
};

static const struct attribute_group *rtk_16xxb_rbus_attr_groups[] = {
	/* must be Null-terminated */
	[RTK_PMU_ATTR_GROUP__COMMON] = &rtk_pmu_common_attr_group,
	[RTK_PMU_ATTR_GROUP__FORMAT] = &rtk_rbus_format_attr_group,
	[RTK_PMU_ATTR_GROUP__EVENT] = &rtk_16xxb_rbus_event_attr_group,
	[RTK_PMU_ATTR_GROUP__NUM] = NULL
};

const static struct rtk_pmc_set_meta rtk_16xxb_rbus_ps_meta[] = {
	{
		.name = "Realtek Rbus",
		.type = PMC_SET__RBUS,
		.init = rbus_pmu_ps_init_v2,
		.group_size = RBUS_PMC__V2_USAGE_NUM,
		.config_size = 8,
		.config_width = 1,
		.val_mask = (const unsigned int []){
			RTK_PMU_VAL_MASK(32),
			RTK_PMU_VAL_MASK(32),
			RTK_PMU_VAL_MASK(32),
			RTK_PMU_VAL_MASK(32),
		},
		.ov_th = (const unsigned int []){
			RTK_PMU_OVERFLOW_TH(32),
			RTK_PMU_OVERFLOW_TH(32),
			RTK_PMU_OVERFLOW_TH(32),
			RTK_PMU_OVERFLOW_TH(32),
		},

		.nr_clients = ARRAY_SIZE(rtk_16xxb_rbus_clients),
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_rbus_pmcgs),
		.nr_configs = ARRAY_SIZE(rtk_rbus_configs),
		.clients = rtk_16xxb_rbus_clients,
		.pmcgs = rtk_16xxb_rbus_pmcgs,
		.configs = rtk_rbus_configs,
	},
	{
		.name = "Realtek Rbus Driver",
		.type = PMC_SET__RBUS_DRV,
		.init = rbus_pmu_ps_drv_init,
		.group_size = 1,
		.config_size = 0,
		.config_width = 0,

		.nr_clients = ARRAY_SIZE(rtk_rbus_drv_ev),
		.nr_pmcgs = RBUS_EV_NUM,
		.nr_configs = 0,
		.clients = rtk_rbus_drv_ev,
		.pmcgs = NULL,
		.configs = NULL,
	},
	{},
};

int rtk_16xxb_rbus_init(struct rtk_pmu *pmu, struct device_node *dt)
{
	return rtk_rbus_pmu_init(pmu, dt,
			       "rtk_16xxb_rbus_pmu",
			       rtk_16xxb_rbus_attr_groups,
			       rtk_16xxb_rbus_ps_meta,
			       RTK_PMU_META_NR(rtk_16xxb_rbus_ps_meta));
}
EXPORT_SYMBOL(rtk_16xxb_rbus_init);
MODULE_LICENSE("GPL");

