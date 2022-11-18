// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek Dbus PMU driver for 16xxb
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
#include "rtk_dbus_pmu.h"


#define DBUS_CTRL_OFFSET	0x0030
#define DBUS_SYSH_WR_LAT_CTRL	0x0c38
#define DBUS_SYS_WR_LAT_CTRL	0x0c3c


/* Target ID of 16xxb */
/* SYSH domain */
#define VO_ID				0x00	/* 7b'0000000 */
#define NPU_ID				0x19	/* 7b'0011001 */
#define SCPU_NW_ID			0x20	/* 7b'0100000 */
#define ACPU_ID				0x23	/* 7b'0100011 */
#define SCPU_SW_ID			0x25	/* 7b'0100101 */
#define VE1_ID				0x30	/* 7b'0110000 */
#define VE3_ID				0x31	/* 7b'0110001 */
#define PCIE1_ID			0x38	/* 7b'0111000 */
#define PCIE2_ID			0x39	/* 7b'0111001 */
#define AUCPU_ID			0x40	/* 7b'1000000 */
#define SATA_ID				0x48	/* 7b'1001000 */
#define DIP_ID				0x58	/* 7b'1011000 */
#define HSE_REE_ID			0x60	/* 7b'1100000 */
#define HSE_TEE_ID			0x65	/* 7b'1100101 */

#define SB1_ID				0x10	/* 7b'0010000 */
#define SB3_ID				0x28	/* 7b'0101000 */
#define SB4_ID				0x50	/* 7b'1010000 */

/* SYS domain SB0 */
#define AIO_ID				0x0a	/* 7b'0001010 */

/* SYS domain SB1 */
#define R2RDSC_ID			0x10	/* 7b'0010000 */
#define CP_ID				0x11	/* 7b'0010001 */
#define AEE_ID				0x12	/* 7b'0010010 */
#define MD_ID				0x13	/* 7b'0010011 */
#define ADE_ID				0x14	/* 7b'0010100 */
#define JPEG_ID				0x15	/* 7b'0010101 */
#define TPB_ID				0x16	/* 7b'0010110 */
#define TP_ID				0x17	/* 7b'0010111 */

/* SYS domain SB3 */
#define VTC_ID				0x28	/* 7b'0101000 */
#define USB3_ID				0x29	/* 7b'0101001 */
#define USB_ID				0x2a	/* 7b'0101010 */
#define HIF_ID				0x2b	/* 7b'0101011 */
#define NF_ID				0x2d	/* 7b'0101101 */
#define ETN_ID				0x2e	/* 7b'0101110 */
#define EMMC_ID				0x2f	/* 7b'0101111 */

/* SYS domain SB4 */
#define SD_ID				0x56	/* 7b'1010110 */
#define SDIO_ID				0x57	/* 7b'1010111 */


/* SYSH domain events */
DBUS_SYSH_EVENT_GROUP(vo,	VO_ID,		DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(npu,	NPU_ID,		DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(scpu_nw,	SCPU_NW_ID,	DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(acpu,	ACPU_ID,	DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(scpu_sw,	SCPU_SW_ID,	DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(ve1,	VE1_ID,		DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(ve3,	VE3_ID,		DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(pcie1,	PCIE1_ID,	DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(pcie2,	PCIE2_ID,	DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(aucpu,	AUCPU_ID,	DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(sata,	SATA_ID,	DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(dip,	DIP_ID,		DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(hse_ree,	HSE_REE_ID,	DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(hse_tee,	HSE_TEE_ID,	DBUS_CH_AUTO);
DBUS_SYSH_EVENT_GROUP(sb1_rt,	SB1_ID,		DBUS_CH_0);
DBUS_SYSH_EVENT_GROUP(sb3_rt,	SB3_ID,		DBUS_CH_0);
DBUS_SYSH_EVENT_GROUP(sb4_rt,	SB4_ID,		DBUS_CH_0);
DBUS_SYSH_EVENT_GROUP(sb1_nrt,	SB1_ID,		DBUS_CH_1);
DBUS_SYSH_EVENT_GROUP(sb3_nrt,	SB3_ID,		DBUS_CH_1);
DBUS_SYSH_EVENT_GROUP(sb4_nrt,	SB4_ID,		DBUS_CH_1);

/* SYS domain events */
DBUS_SYS_EVENT_GROUP_V2(aio,	AIO_ID);
DBUS_SYS_EVENT_GROUP_V2(r2rdsc,	R2RDSC_ID);
DBUS_SYS_EVENT_GROUP_V2(cp,	CP_ID);
DBUS_SYS_EVENT_GROUP_V2(aee,	AEE_ID);
DBUS_SYS_EVENT_GROUP_V2(md,	MD_ID);
DBUS_SYS_EVENT_GROUP_V2(ade,	ADE_ID);
DBUS_SYS_EVENT_GROUP_V2(jpeg,	JPEG_ID);
DBUS_SYS_EVENT_GROUP_V2(tpb,	TPB_ID);
DBUS_SYS_EVENT_GROUP_V2(tp,	TP_ID);
DBUS_SYS_EVENT_GROUP_V2(vtc,	VTC_ID);
DBUS_SYS_EVENT_GROUP_V2(usb3,	USB3_ID);
DBUS_SYS_EVENT_GROUP_V2(usb,	USB_ID);
DBUS_SYS_EVENT_GROUP_V2(hif,	HIF_ID);
DBUS_SYS_EVENT_GROUP_V2(nf,	NF_ID);
DBUS_SYS_EVENT_GROUP_V2(etn,	ETN_ID);
DBUS_SYS_EVENT_GROUP_V2(emmc,	EMMC_ID);
DBUS_SYS_EVENT_GROUP_V2(sd,	SD_ID);
DBUS_SYS_EVENT_GROUP_V2(sdio,	SDIO_ID);

/* Channel total events */
DBUS_TOTAL_EVENT_ATTR_GROUP(dbus_ch0,	0);
DBUS_TOTAL_EVENT_ATTR_GROUP(dbus_ch1,	1);

/* Driver statistics events */
DBUS_DRV_EVENT_ATTR(dbus_refresh,		DBUS_REFRESH);
DBUS_DRV_EVENT_ATTR(dbus_overflow,		RTK_DRV_OVERFLOW);


static struct attribute *rtk_16xxb_event_attrs[] = {
	DBUS_DRV_EVENT_REF(dbus_refresh),
	DBUS_DRV_EVENT_REF(dbus_overflow),

	DBUS_TOTAL_EVENT_REF_GROUP(dbus_ch0),
	DBUS_TOTAL_EVENT_REF_GROUP(dbus_ch1),

	DBUS_SYSH_EVENT_REF_GROUP(vo),
	DBUS_SYSH_EVENT_REF_GROUP(npu),
	DBUS_SYSH_EVENT_REF_GROUP(scpu_nw),
	DBUS_SYSH_EVENT_REF_GROUP(acpu),
	DBUS_SYSH_EVENT_REF_GROUP(scpu_sw),
	DBUS_SYSH_EVENT_REF_GROUP(ve1),
	DBUS_SYSH_EVENT_REF_GROUP(ve3),
	DBUS_SYSH_EVENT_REF_GROUP(pcie1),
	DBUS_SYSH_EVENT_REF_GROUP(pcie2),
	DBUS_SYSH_EVENT_REF_GROUP(aucpu),
	DBUS_SYSH_EVENT_REF_GROUP(sata),
	DBUS_SYSH_EVENT_REF_GROUP(dip),
	DBUS_SYSH_EVENT_REF_GROUP(hse_ree),
	DBUS_SYSH_EVENT_REF_GROUP(hse_tee),
	DBUS_SYSH_EVENT_REF_GROUP(sb1_rt),
	DBUS_SYSH_EVENT_REF_GROUP(sb3_rt),
	DBUS_SYSH_EVENT_REF_GROUP(sb4_rt),
	DBUS_SYSH_EVENT_REF_GROUP(sb1_nrt),
	DBUS_SYSH_EVENT_REF_GROUP(sb3_nrt),
	DBUS_SYSH_EVENT_REF_GROUP(sb4_nrt),

	DBUS_SYS_EVENT_REF_GROUP_V2(aio),
	DBUS_SYS_EVENT_REF_GROUP_V2(r2rdsc),
	DBUS_SYS_EVENT_REF_GROUP_V2(cp),
	DBUS_SYS_EVENT_REF_GROUP_V2(aee),
	DBUS_SYS_EVENT_REF_GROUP_V2(md),
	DBUS_SYS_EVENT_REF_GROUP_V2(ade),
	DBUS_SYS_EVENT_REF_GROUP_V2(jpeg),
	DBUS_SYS_EVENT_REF_GROUP_V2(tpb),
	DBUS_SYS_EVENT_REF_GROUP_V2(tp),
	DBUS_SYS_EVENT_REF_GROUP_V2(vtc),
	DBUS_SYS_EVENT_REF_GROUP_V2(usb3),
	DBUS_SYS_EVENT_REF_GROUP_V2(usb),
	DBUS_SYS_EVENT_REF_GROUP_V2(hif),
	DBUS_SYS_EVENT_REF_GROUP_V2(nf),
	DBUS_SYS_EVENT_REF_GROUP_V2(etn),
	DBUS_SYS_EVENT_REF_GROUP_V2(emmc),
	DBUS_SYS_EVENT_REF_GROUP_V2(sd),
	DBUS_SYS_EVENT_REF_GROUP_V2(sdio),
	NULL
};

static struct attribute_group rtk_16xxb_event_attr_group = {
	.name = "events",
	.attrs = rtk_16xxb_event_attrs,
};

static const struct attribute_group *rtk_16xxb_dbus_attr_groups[] = {
	/* must be Null-terminated */
	[RTK_PMU_ATTR_GROUP__COMMON] = &rtk_pmu_common_attr_group,
	[RTK_PMU_ATTR_GROUP__FORMAT] = &rtk_dbus_format_attr_group,
	[RTK_PMU_ATTR_GROUP__EVENT] = &rtk_16xxb_event_attr_group,
	[RTK_PMU_ATTR_GROUP__NUM] = NULL
};

/* channel control bit */
enum {
	ACPU_CH_SEL	= 0,
	VO_CH_SEL	= 1,
	DIP_CH_SEL	= 2,
	VE1_CH_SEL	= 3,
	VE3_CH_SEL	= 4,
	PCIE_CH_SEL	= 5,
	HSE_CH_SEL	= 6,
	AUCPU_CH_SEL	= 7,
	SATA_CH_SEL	= 8,
	SCPU_CH_SEL	= 9,
	NPU_CH_SEL	= 10,
	PCIE2_CH_SEL	= 11,
};

static u8
__get_client_ch(struct rtk_pmc_set *ps, unsigned int target)
{
	unsigned long ch_reg;
	u8 ch = 0;

	if (ps->has_ext)
		/*
		 * using static channel information instead read from control
		 * register.
		 */
		ch_reg = (unsigned long)ps->ext_info;
	else
		ch_reg = rtk_readl((unsigned long)ps->base + DBUS_CH_SEL_CTRL0);

	switch (target) {
	case VO_ID:
		ch = test_bit(VO_CH_SEL, &ch_reg);
		break;
	case SCPU_NW_ID:
	case SCPU_SW_ID:
		ch = test_bit(SCPU_CH_SEL, &ch_reg);
		break;
	case VE1_ID:
		ch = test_bit(VE1_CH_SEL, &ch_reg);
		break;
	case VE3_ID:
		ch = test_bit(VE3_CH_SEL, &ch_reg);
		break;
	case PCIE1_ID:
		ch = test_bit(PCIE_CH_SEL, &ch_reg);
		break;
	case PCIE2_ID:
		ch = test_bit(PCIE2_CH_SEL, &ch_reg);
		break;
	case NPU_ID:
		ch = test_bit(NPU_CH_SEL, &ch_reg);
		break;
	case SATA_ID:
		ch = test_bit(SATA_CH_SEL, &ch_reg);
		break;
	case DIP_ID:
		ch = test_bit(DIP_CH_SEL, &ch_reg);
		break;
	case HSE_REE_ID:
	case HSE_TEE_ID:
		ch = test_bit(HSE_CH_SEL, &ch_reg);
		break;
	case ACPU_ID:
		ch = test_bit(ACPU_CH_SEL, &ch_reg);
		break;
	case AUCPU_ID:
		ch = test_bit(AUCPU_CH_SEL, &ch_reg);
		break;
	default:
		break;
	}

	WARN(ch >= DBUS_CH_AUTO, "target:%#x locates at channel %d\n",
	     target, ch);

	return ch;
}

static union rtk_pmc_desc
rtk_16xxb_arrange_pmc(struct rtk_pmu *pmu, u64 config)
{
	union rtk_pmc_desc pmc;
	union rtk_dbus_event_desc desc = __get_event_desc(config);
	struct rtk_pmc_set *ps = pmu->pmcss[desc.set];
	int idx = ps->arrange_pmc(ps, rtk_dbus_pmc_config(config),
				  rtk_dbus_pmc_target(config));

	if (idx < 0) {
		pmc.val = -EAGAIN;
	} else {
		pmc.set = desc.set;
		pmc.idx = idx;
		pmc.usage = desc.usage;

		if (desc.set == PMC_SET__DBUS_SYSH && desc.ch >= DBUS_CH_AUTO)
			pmc.ch = __get_client_ch(ps, desc.target);
		else
			pmc.ch = desc.ch;
	}

	dmsg("%s- event desc %#x:%#x:%#x\n", pmu->name, desc.set, desc.target,
	     desc.usage);
	dmsg("%s- event pmc %#x:%#x:%#x\n", pmu->name, pmc.set, pmc.idx,
	     pmc.usage);
	return pmc;
}

static const unsigned int rtk_16xxb_dbus_sysh_clients[] = {
	VO_ID, NPU_ID, SCPU_NW_ID, ACPU_ID, SCPU_SW_ID, VE1_ID, VE3_ID,
	PCIE1_ID, PCIE2_ID, AUCPU_ID, SATA_ID, DIP_ID, HSE_REE_ID, HSE_TEE_ID,
	SB1_ID, SB3_ID, SB4_ID
};

/* address offset of Dbus sysh domain PMCG */
static const unsigned long rtk_16xxb_dbus_sysh_pmcgs[] = {
	0x0060, 0x0070, 0x0080, 0x0510, 0x0520, 0x530, 0x540, 0x550
};

/* address offset of configs of Dbus sysh domain PMCG */
static const unsigned long rtk_16xxb_dbus_sysh_configs[] = {
	0x0050, 0x0500, 0x504
};

static const unsigned int rtk_16xxb_dbus_sys_clients[] = {
	AIO_ID, R2RDSC_ID, CP_ID, AEE_ID, MD_ID, ADE_ID, JPEG_ID, TPB_ID, TP_ID,
	VTC_ID, USB3_ID, USB_ID, HIF_ID, NF_ID, ETN_ID, EMMC_ID, SD_ID, SDIO_ID
};

/* address offset of Dbus sys domain(bridge) PMCG */
static const unsigned long rtk_16xxb_dbus_sys_pmcgs[] = {
	0x0090, 0x00a0, 0x00b0
};

/* address offset of configs of Dbus sys domain(bridge) PMCG */
static const unsigned long rtk_16xxb_dbus_sys_configs[] = {
	0x0054
};

static const unsigned int rtk_16xxb_dbus_ch_clients[] = {
	DBUS_CH_0, DBUS_CH_1
};

/* address offset of Dbus output channel statistics PMCG */
static const unsigned long rtk_16xxb_dbus_ch_pmcgs[] = {
	0x0034, 0x0040
};

static const unsigned int rtk_16xxb_dbus_drv_clients[] = {
	RTK_DRV_OVERFLOW, DBUS_REFRESH
};

const static struct rtk_pmc_set_meta rtk_16xxb_ps_meta[] = {
	{
		.name = "16xxb Dbus SYS",
		.compatible = "dbus-sys",
		.type = PMC_SET__DBUS_SYS,
		.init = rtk_dbus_ps_init,
		.group_size = DBUS_PMC__USAGE_NUM,
		.config_size = 3,
		.config_width = 8,
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

		.nr_clients = ARRAY_SIZE(rtk_16xxb_dbus_sys_clients),
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_dbus_sys_pmcgs),
		.nr_configs = ARRAY_SIZE(rtk_16xxb_dbus_sys_configs),
		.clients = rtk_16xxb_dbus_sys_clients,
		.pmcgs = rtk_16xxb_dbus_sys_pmcgs,
		.configs = rtk_16xxb_dbus_sys_configs,
	},
	{
		.name = "16xxb Dbus SYSH",
		.compatible = "dbus-sysh",
		.type = PMC_SET__DBUS_SYSH,
		.init = rtk_dbus_ps_init,
		.group_size = DBUS_PMC__USAGE_NUM,
		.config_size = 3,
		.config_width = 8,
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

		.nr_clients = ARRAY_SIZE(rtk_16xxb_dbus_sysh_clients),
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_dbus_sysh_pmcgs),
		.nr_configs = ARRAY_SIZE(rtk_16xxb_dbus_sysh_configs),
		.clients = rtk_16xxb_dbus_sysh_clients,
		.pmcgs = rtk_16xxb_dbus_sysh_pmcgs,
		.configs = rtk_16xxb_dbus_sysh_configs,
	},
	{
		.name = "16xxb Dbus CH",
		.compatible = "dbus-ch",
		.type = PMC_SET__DBUS_CH,
		.init = rtk_dbus_ch_init,
		.group_size = 3,
		.val_mask = (const unsigned int []){
			RTK_PMU_VAL_MASK(32),
			RTK_PMU_VAL_MASK(32),
			RTK_PMU_VAL_MASK(32),
		},
		.ov_th = (const unsigned int []){
			RTK_PMU_OVERFLOW_TH(32),
			RTK_PMU_OVERFLOW_TH(32),
			RTK_PMU_OVERFLOW_TH(32),
		},

		.nr_clients = ARRAY_SIZE(rtk_16xxb_dbus_ch_clients),
		.nr_pmcgs = ARRAY_SIZE(rtk_16xxb_dbus_ch_pmcgs),
		.nr_configs = 0,
		.clients = rtk_16xxb_dbus_ch_clients,
		.pmcgs = rtk_16xxb_dbus_ch_pmcgs,
		.configs = NULL,
	},
	{
		.name = "16xxb Dbus DRV",
		.type = PMC_SET__DBUS_DRV,
		.init = rtk_dbus_drv_init,
		.group_size = 1,

		.nr_clients = ARRAY_SIZE(rtk_16xxb_dbus_drv_clients),
		.nr_pmcgs = DBUS_EV_NUM,
		.nr_configs = 0,
		.clients = rtk_16xxb_dbus_drv_clients,
		.pmcgs = NULL,
		.configs = NULL,
	},
	{},
};

int rtk_16xxb_dbus_init(struct rtk_pmu *pmu, struct device_node *dt)
{
	int ret;
	unsigned long ctrl;
	u32 val;

	ret = rtk_dbus_pmu_init(pmu, dt,
				"rtk_16xxb_dbus_pmu",
				DBUS_CTRL_OFFSET,
				rtk_16xxb_dbus_attr_groups,
				rtk_16xxb_ps_meta,
				RTK_PMU_META_NR(rtk_16xxb_ps_meta));

	if (!ret) {
		pmu->arrange_pmc = rtk_16xxb_arrange_pmc;

		/* make write latency counts the entire write operation */
		ctrl = (unsigned long)pmu->base + DBUS_SYSH_WR_LAT_CTRL;
		val = BIT(0) | BIT(4) | BIT(8) | BIT(12) |
			BIT(16) | BIT(20) | BIT(24) | BIT(28);
		rtk_writel(ctrl, val);

		ctrl = (unsigned long)pmu->base + DBUS_SYS_WR_LAT_CTRL;
		val = BIT(0) | BIT(4) | BIT(8);
		rtk_writel(ctrl, val);
	}

	return ret;
}
EXPORT_SYMBOL(rtk_16xxb_dbus_init);
MODULE_LICENSE("GPL");
