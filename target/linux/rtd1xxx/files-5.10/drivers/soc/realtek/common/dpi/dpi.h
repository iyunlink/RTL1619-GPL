/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef __SOC_REALTEK_DPI_H
#define __SOC_REALTEK_DPI_H

#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/iopoll.h>
#include <linux/arm-smccc.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <soc/realtek/rtk_chip.h>

struct regmap;
struct thermal_zone_device;

struct dpi_device {
	struct device *dev;
	void *base;
	unsigned long base_addr;
	struct regmap *misc;
	struct thermal_zone_device *tzdev;
	struct delayed_work delayed_work;

	struct raw_notifier_head notifier_list;
	uint32_t chip_id;
	int base_temp;
	int swc_access;
	unsigned int swc_ior;
	unsigned int swc_iow;
	int hot_temp;
	int cold_temp;
	int delta_temp;
};

enum {
	DPI_EVENT_TEMP_STATUS_HOT,
	DPI_EVENT_TEMP_STATUS_COLD,
	DPI_EVENT_TEMP_STATUS_NORMAL,
	DPI_EVENT_BASE_TEMP_DIFF_OVER_THERSHOLD,
	DPI_EVENT_BASE_TEMP_UPDATED,
};

struct dpi_event_temp_status_data {
	int temp;
};

struct dpi_event_base_temp_data {
	int old_temp;
	int new_temp;
};

enum dpi_dram_type {
	DPI_DRAM_TYPE_DDR4 = 0,
	DPI_DRAM_TYPE_DDR3 = 1,
	DPI_DRAM_TYPE_LPDDR4 = 2,
	DPI_DRAM_TYPE_UNKNOWN,
};

#define dpi_reg_poll_timeout_atomic(op, addr, ofs, val, cond, delay_us, timeout_us) \
({ \
	ktime_t timeout = ktime_add_us(ktime_get(), timeout_us); \
	for (;;) { \
		op(addr, ofs, &val); \
		if (cond) \
			break; \
		if (timeout_us && ktime_compare(ktime_get(), timeout) > 0) { \
			op(addr, ofs, &val); \
			break; \
		} \
		if (delay_us) \
			udelay(delay_us);	\
	} \
	(cond) ? 0 : -ETIMEDOUT; \
})

static inline int dpi_get_base_addr(struct device_node *np, unsigned long *addr)
{
	struct resource res;
	int ret;
	ret = of_address_to_resource(np, 0, &res);
	if (ret)
		return ret;
	*addr = (unsigned long)res.start;
	return 0;
}

static inline int dpi_setup_swc_access(struct dpi_device *dpi)
{
	dpi->swc_access = 1;
	dpi->swc_ior = 0x8400fffe;
	dpi->swc_iow = 0x8400ffff;
	return 0;
}

static inline
void dpi_reg_read(struct dpi_device *dpi, uint32_t ofs, uint32_t *val)
{
	if (dpi->swc_access) {
		struct arm_smccc_res res;
		unsigned int addr = dpi->base_addr + ofs;
		arm_smccc_smc(dpi->swc_ior, addr, 0, 0, 0, 0, 0, 0, &res);
		*val = res.a0;
	}
	else
		*val = readl(dpi->base + ofs);
}

static inline
void dpi_reg_write(struct dpi_device *dpi, uint32_t ofs, uint32_t val)
{
	pr_debug("%s: ofs=%03x, val=%08x\n", __func__, ofs, val);
	if (dpi->swc_access) {
		struct arm_smccc_res res;
		unsigned int addr = dpi->base_addr + ofs;
		arm_smccc_smc(dpi->swc_iow, addr, val, 0, 0, 0, 0, 0, &res);
	}
	else
		writel(val, dpi->base + ofs);
}

static inline
void dpi_reg_update_bits(struct dpi_device *dpi, uint32_t ofs, uint32_t mask, uint32_t val)
{
	uint32_t v;

	dpi_reg_read(dpi, ofs, &v);
	v &= ~mask;
	v |= (mask & val);
	dpi_reg_write(dpi, ofs, v);
}

static inline
int dpi_reg_poll_bit(struct dpi_device *dpi, uint32_t ofs, int bit)
{
	uint32_t status;

	pr_debug("%s\n", __func__);
	return dpi_reg_poll_timeout_atomic(dpi_reg_read, dpi, ofs, status, status & BIT(bit), 0, 1000000);
}

static inline
void dpi_clock_gating_disable(struct dpi_device *dpi)
{
	pr_debug("%s\n", __func__);
	dpi_reg_update_bits(dpi, 0x90, 0x3c, 0x0);
	// dpi_reg_update_bits(dpi, 0x4, 0x00800000, 0x0);
	dpi_reg_update_bits(dpi, 0x60, 0x001c0003, 0x0);
}

static inline
void dpi_clock_gating_enable(struct dpi_device *dpi)
{
	pr_debug("%s\n", __func__);
	if(dpi->chip_id == CHIP_ID_RTD1619B) {
		dpi_reg_update_bits(dpi, 0x90, 0x3c, 0x24);
		//dpi_reg_update_bits(dpi, 0x4, 0x00800000, 0x00800000);
		dpi_reg_update_bits(dpi, 0x60, 0x001c0003, 0x001c0003);
	}
	else {
		//dpi_reg_update_bits(dpi, 0x90, 0x3c, 0x24);
		//dpi_reg_update_bits(dpi, 0x4, 0x8800000, 0x8800000);
		dpi_reg_update_bits(dpi, 0x60, 0x7, 0x7);
	}
}

static inline int swc_access_check(struct dpi_device *dpi, uint32_t addr, uint32_t val)
{
	struct arm_smccc_res res;
	arm_smccc_smc(dpi->swc_ior, addr, 0, 0, 0, 0, 0, 0, &res);
	if(val != res.a0)
		return -EACCES;

	return 0;
}

static inline int dpi_check_perm(struct dpi_device *dpi)
{
	struct device *dev = dpi->dev;
	struct device_node *np = dev->of_node;
	struct nvmem_cell *cell;
	unsigned char *buf;
	size_t buf_size;
	unsigned char val = 0;
	int ret;

	if(!(dpi->base = of_iomap(dev->of_node, 0)))
		return -ENOMEM;

	dpi->chip_id = get_rtd_chip_id();

	if(	dpi->chip_id == CHIP_ID_RTD1319 ||
		dpi->chip_id == CHIP_ID_RTD1317 ||
		dpi->chip_id == CHIP_ID_RTD1315 )
	{
		cell = of_nvmem_cell_get(np, "secure_en");
		if (!IS_ERR(cell)) {
			buf = nvmem_cell_read(cell, &buf_size);
			nvmem_cell_put(cell);
			if (!IS_ERR(buf)) {
				val = buf[0];
				kfree(buf);
			}
			if (val == 0 || val == 1)	// EI chip
				return 0;

			ret = dpi_get_base_addr(np, &dpi->base_addr);
			if (ret)
				return ret;
			dpi_setup_swc_access(dpi);

			ret = swc_access_check(dpi, 0x98090ff0, 0x3337302a);	// uMCTL2 version number
			if(ret)
				return ret;

			dev_info(dev, "use smc to access reg with base 0x%lx\n", dpi->base_addr);
		}
	}

	return 0;
}

enum dpi_dram_type dpi_get_dram_type(struct dpi_device *dpi);

int dpi_register_notifier(struct dpi_device *dpi, struct notifier_block *nb);
int dpi_unregister_notifier(struct dpi_device *dpi, struct notifier_block *nb);

#endif /* __SOC_REALTEK_DPI_H */
