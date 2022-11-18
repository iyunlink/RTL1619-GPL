/*
 * Realtek CPU volt sel
 *
 * Copyright (c) 2019, Realtek Semiconductor Corporation
 *
 * Author:
 *        Cheng-Yu Lee <cylee12@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#define pr_fmt(fmt) "cpu-volt-sel: " fmt

#include <linux/cpu.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/module.h>
#include <soc/realtek/rtk_chip.h>
#include <linux/notifier.h>
#include <linux/nvmem-consumer.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>

static char prop_name[40];
static int prop_name_inited;
static struct opp_table *opp_table;

static int prop_from_chip_rev(char *name, size_t size)
{
	unsigned int val;

	val = get_rtd_chip_revision();
	if ((val & 0xFFF) == 0) {
		pr_err("%s: invalid chip revision\n", __func__);
		return -EINVAL;
	}
	snprintf(name, size, "%x", val);
	return 0;
}

static int prop_from_dss(char *name, size_t size)
{
	struct device_node *np;
	unsigned int val;
	struct nvmem_cell *cell;
	int ret = 0;
	unsigned char *buf;
	size_t buf_size;


	np = of_find_node_by_path("/cpu-dvfs");
	if (!np) {
		pr_err("%s: failed to find device node\n", __func__);
		return -ENODEV;
	}

	cell = of_nvmem_cell_get(np, "cpu-dss");
	if (IS_ERR(cell)) {
		ret = PTR_ERR(cell);
		pr_debug("%s: failed to get dss cell: %d\n", __func__, ret);
		return -ENOTSUPP;
	}

	buf = nvmem_cell_read(cell, &buf_size);
	if (IS_ERR(buf)) {
		ret = PTR_ERR(buf);
		pr_err("%s: failed to read dss cell: %d\n", __func__, ret);
		goto done;
	}

	val = (buf[15] << 12) | (buf[16] << 4) | (buf[17] >> 4);

	pr_info("%s: dss val=%d\n", __func__, val);
	if (val > 23000) {
		WARN(val > 26000, "dss value too large\n");
		snprintf(name, size, "ss");
	} else {
		WARN(val && val < 18000, "dss value too small\n");
		snprintf(name, size, "tt");
	}

	kfree(buf);
done:
	nvmem_cell_put(cell);
	return ret;
}

static int prop_from_dt(char *name, size_t size)
{
	struct device_node *np;
	unsigned int val;
	int ret = 0;

	np = of_find_node_by_path("/cpu-dvfs");
	if (!np) {
		pr_err("%s: failed to find device node\n", __func__);
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "bsv-opp-updated", &val);
	if (ret)
		ret = of_property_read_u32(np, "bsv,opp-updated", &val);
	if (!ret && val == 1) {
		snprintf(name, size, "bsv");
		return 0;
	}

	ret = of_property_read_u32(np, "fss-opp-updated", &val);
	if (ret)
		ret = of_property_read_u32(np, "fss,opp-updated", &val);
	if (ret || val == 0)
		return -ENOTSUPP;

	snprintf(name, size, "fss");
	return 0;
}

static int cpu_volt_set_init_prop_name(void)
{
	int ret;

	ret = prop_from_dt(prop_name, sizeof(prop_name));
	if (ret == -ENOTSUPP)
		ret = prop_from_dss(prop_name, sizeof(prop_name));
	if (ret == -ENOTSUPP)
		ret = prop_from_chip_rev(prop_name, sizeof(prop_name));
	if (ret)
		return ret;

	pr_info("prop_name is %s\n", prop_name);
	prop_name_inited = 1;
	return 0;
}

static void cpu_volt_sel_set_prop_name(void)
{
	struct device *dev;
	int ret;

	dev = get_cpu_device(0);
	if (!dev) {
		pr_err("failed to get cpu device\n");
		return;
	}

	if (WARN_ON(opp_table)) {
		pr_err("opp table not clear\n");
		return;
	}

	opp_table = dev_pm_opp_set_prop_name(dev, prop_name);
	if (IS_ERR(opp_table)) {
		ret = PTR_ERR(opp_table);
		if (ret == -EPROBE_DEFER)
			pr_debug("dev_pm_opp_set_prop_name() not ready, retry\n");
		else
			pr_warn("dev_pm_opp_set_prop_name() returns %d\n", ret);
		opp_table = NULL;
	}

	if (opp_table)
		pr_info("dev_pm_opp_set_prop_name: name=%s\n", prop_name);

	return;
}

static void cpu_volt_sel_put_opp_table(void)
{
	if (!opp_table)
		return;

	dev_pm_opp_put_prop_name(opp_table);
	opp_table = NULL;
}

static int cpu_volt_sel_cb(struct notifier_block *nb, unsigned long event,
		                           void *data)
{
	struct device *dev = data;

	if (!prop_name_inited)
		return NOTIFY_DONE;

	if (strcmp("cpufreq-dt", dev_name(dev)))
		return NOTIFY_DONE;

	dev_dbg(dev, "cpufreq-dt receive bus event %ld\n", event);

	switch (event) {
	case BUS_NOTIFY_BIND_DRIVER:
		cpu_volt_sel_set_prop_name();
		return NOTIFY_OK;

	case BUS_NOTIFY_UNBOUND_DRIVER:
	case BUS_NOTIFY_DRIVER_NOT_BOUND:
		cpu_volt_sel_put_opp_table();
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block cpu_volt_sel_nb = {
	.notifier_call = cpu_volt_sel_cb,
};

static __init int cpu_volt_sel_init(void)
{
	int ret;

	ret = cpu_volt_set_init_prop_name();
	if (ret)
		return ret;

	return bus_register_notifier(&platform_bus_type, &cpu_volt_sel_nb);
}
fs_initcall(cpu_volt_sel_init);

MODULE_LICENSE("GPL");
