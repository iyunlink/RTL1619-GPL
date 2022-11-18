// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Realtek PCIe host controller driver
 *
 * Copyright (c) 2017 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/reset.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/phy/phy.h>

#include "pcie-rtd.h"

static void rtd_msi_ack_irq(struct irq_data *d)
{
	irq_chip_ack_parent(d);
}

static void rtd_msi_mask_irq(struct irq_data *d)
{
	pci_msi_mask_irq(d);
	irq_chip_mask_parent(d);
}

static void rtd_msi_unmask_irq(struct irq_data *d)
{
	pci_msi_unmask_irq(d);
	irq_chip_unmask_parent(d);
}

static struct irq_chip rtd_pcie_msi_irq_chip = {
	.name = "PCI-MSI",
	.irq_ack = rtd_msi_ack_irq,
	.irq_mask = rtd_msi_mask_irq,
	.irq_unmask = rtd_msi_unmask_irq,
};

static struct msi_domain_info rtd_pcie_msi_domain_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS | MSI_FLAG_MULTI_PCI_MSI),
	.chip	= &rtd_pcie_msi_irq_chip,
};

/* MSI int handler */
irqreturn_t rtd_handle_mac_msi_irq(struct rtd_pcie_port *pp)
{
	int i, pos, irq;
	unsigned long val;
	u32 status, num_ctrls;
	irqreturn_t ret = IRQ_NONE;

	num_ctrls = RTK_MAX_MSI_IRQS / RTK_MAX_MSI_IRQS_PER_CTRL;

	for (i = 0; i < num_ctrls; i++) {
		status = readl(pp->ctrl_base + PCIE_MSI_INTR0_STATUS +
					(i * RTK_MSI_REG_CTRL_BLOCK_SIZE));
		if (!status)
			continue;

		ret = IRQ_HANDLED;
		val = status;
		pos = 0;
		while ((pos = find_next_bit(&val, RTK_MAX_MSI_IRQS_PER_CTRL,
					    pos)) != RTK_MAX_MSI_IRQS_PER_CTRL) {
			irq = irq_find_mapping(pp->irq_domain,
					       (i * RTK_MAX_MSI_IRQS_PER_CTRL) +
					       pos);
			generic_handle_irq(irq);
			pos++;
		}
	}

	return ret;
}

irqreturn_t rtd_handle_wrapper_msi_irq(struct rtd_pcie_port *pp)
{
	int pos, irq;

	pos = 0;
	irq = irq_find_mapping(pp->irq_domain, pos);
	if (irq != 0) {
		generic_handle_irq(irq);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

irqreturn_t rtd_handle_wrapper_msix_irq(struct rtd_pcie_port *pp)
{
	int i, pos, irq;
	unsigned long val;
	u32 status, num_ctrls;
	irqreturn_t ret = IRQ_NONE;

	num_ctrls = RTK_MSIX_DEF_NUM_VECTORS / RTK_MAX_MSIX_IRQS_PER_CTRL;

	for (i = 0; i < num_ctrls; i++) {
		status = readl(pp->ctrl_base + PCIE_MSIX_0 +
					(i * 0x4));
		if (!status)
			continue;

		ret = IRQ_HANDLED;
		val = status;
		pos = 0;
		while ((pos = find_next_bit(&val, RTK_MAX_MSIX_IRQS_PER_CTRL,
					    pos)) != RTK_MAX_MSIX_IRQS_PER_CTRL) {
			irq = irq_find_mapping(pp->irq_domain,
					       (i * RTK_MAX_MSIX_IRQS_PER_CTRL) +
					       pos);
			generic_handle_irq(irq);
			pos++;
		}
	}

	return ret;
}



/* Chained MSI interrupt service routine */
static void rtd_chained_msi_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct rtd_pcie_port *pp;

	chained_irq_enter(chip, desc);

	pp = irq_desc_get_handler_data(desc);
	pp->ops->msi_ops->irq_handle(pp);

	chained_irq_exit(chip, desc);
}

static void rtd_pci_setup_msi_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct rtd_pcie_port *pp = irq_data_get_irq_chip_data(d);
	u64 msi_target;

	msi_target = (u64)pp->msi_data;

	msg->address_lo = lower_32_bits(msi_target);
	msg->address_hi = upper_32_bits(msi_target);

	msg->data = d->hwirq;

	dev_dbg(pp->dev, "msi#%d address_hi %#x address_lo %#x\n",
		(int)d->hwirq, msg->address_hi, msg->address_lo);
}

static int rtd_pci_msi_set_affinity(struct irq_data *d,
				   const struct cpumask *mask, bool force)
{
	struct rtd_pcie_port *pp = irq_data_get_irq_chip_data(d);
	struct irq_chip *parent_chip = irq_get_chip(pp->msi_irq);
	struct irq_data *parent_data = irq_get_irq_data(pp->msi_irq);

	if (parent_chip && parent_chip->irq_set_affinity)
		return parent_chip->irq_set_affinity(parent_data, mask, force);
	else
		return -EINVAL;
}

static void rtd_pci_mac_bottom_mask(struct irq_data *d)
{
	struct rtd_pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned int res, bit, ctrl;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	ctrl = d->hwirq / RTK_MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * RTK_MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % RTK_MAX_MSI_IRQS_PER_CTRL;

	pp->irq_mask[ctrl] |= BIT(bit);
	writel(pp->irq_mask[ctrl], pp->ctrl_base + PCIE_MSI_INTR0_MASK + res);


	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void rtd_pci_mac_bottom_unmask(struct irq_data *d)
{
	struct rtd_pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned int res, bit, ctrl;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	ctrl = d->hwirq / RTK_MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * RTK_MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % RTK_MAX_MSI_IRQS_PER_CTRL;

	pp->irq_mask[ctrl] &= ~BIT(bit);
	writel(pp->irq_mask[ctrl], pp->ctrl_base + PCIE_MSI_INTR0_MASK + res);

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void rtd_pci_mac_bottom_ack(struct irq_data *d)
{
	struct rtd_pcie_port *pp  = irq_data_get_irq_chip_data(d);
	unsigned int res, bit, ctrl;

	ctrl = d->hwirq / RTK_MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * RTK_MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % RTK_MAX_MSI_IRQS_PER_CTRL;

	writel(BIT(bit), pp->ctrl_base + PCIE_MSI_INTR0_STATUS + res);

}

static struct irq_chip rtd_pci_mac_msi_bottom_irq_chip = {
	.name = "RTD_PCI-MSI",
	.irq_ack = rtd_pci_mac_bottom_ack,
	.irq_compose_msi_msg = rtd_pci_setup_msi_msg,
	.irq_set_affinity = rtd_pci_msi_set_affinity,
	.irq_mask = rtd_pci_mac_bottom_mask,
	.irq_unmask = rtd_pci_mac_bottom_unmask,
};



static void rtd_pci_wrapper_bottom_mask(struct irq_data *d)
{
	struct rtd_pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	writel(readl(pp->ctrl_base + PCIE_INT_CTR) & ~BIT(12), pp->ctrl_base + PCIE_INT_CTR);

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void rtd_pci_wrapper_bottom_unmask(struct irq_data *d)
{
	struct rtd_pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	writel(readl(pp->ctrl_base + PCIE_INT_CTR) | BIT(12), pp->ctrl_base + PCIE_INT_CTR);

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void rtd_pci_wrapper_bottom_ack(struct irq_data *d)
{
	struct rtd_pcie_port *pp  = irq_data_get_irq_chip_data(d);

	writel(0x10000, pp->ctrl_base + PCIE_MSI_DATA);
}

static struct irq_chip rtd_pci_wrapper_msi_bottom_irq_chip = {
	.name = "RTD_PCI-MSI",
	.irq_ack = rtd_pci_wrapper_bottom_ack,
	.irq_compose_msi_msg = rtd_pci_setup_msi_msg,
	.irq_set_affinity = rtd_pci_msi_set_affinity,
	.irq_mask = rtd_pci_wrapper_bottom_mask,
	.irq_unmask = rtd_pci_wrapper_bottom_unmask,
};


static void rtd_pci_setup_msix_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct rtd_pcie_port *pp = irq_data_get_irq_chip_data(d);
	u64 msi_target;

	msi_target = (u64)pp->msi_data + d->hwirq * 0x4;

	msg->address_lo = lower_32_bits(msi_target);
	msg->address_hi = upper_32_bits(msi_target);

	msg->data = d->hwirq;

	dev_dbg(pp->dev, "msix#%d address_hi %#x address_lo %#x\n",
		(int)d->hwirq, msg->address_hi, msg->address_lo);
}


static void rtd_pci_wrapper_msix_bottom_mask(struct irq_data *d)
{
	struct rtd_pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	writel(readl(pp->ctrl_base + PCIE_INT_CTR) & ~BIT(15), pp->ctrl_base + PCIE_INT_CTR);

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void rtd_pci_wrapper_msix_bottom_unmask(struct irq_data *d)
{
	struct rtd_pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	writel(readl(pp->ctrl_base + PCIE_INT_CTR) | BIT(15), pp->ctrl_base + PCIE_INT_CTR);

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void rtd_pci_wrapper_msix_bottom_ack(struct irq_data *d)
{
	struct rtd_pcie_port *pp  = irq_data_get_irq_chip_data(d);
	unsigned int res, bit, ctrl;

	ctrl = d->hwirq / RTK_MAX_MSIX_IRQS_PER_CTRL;
	res = ctrl * 0x4;
	bit = d->hwirq % RTK_MAX_MSIX_IRQS_PER_CTRL;

	writel(BIT(bit), pp->ctrl_base + PCIE_MSIX_0 + res);

}


static struct irq_chip rtd_pci_wrapper_msix_bottom_irq_chip = {
	.name = "RTD_PCI-MSI",
	.irq_ack = rtd_pci_wrapper_msix_bottom_ack,
	.irq_compose_msi_msg = rtd_pci_setup_msix_msg,
	.irq_set_affinity = rtd_pci_msi_set_affinity,
	.irq_mask = rtd_pci_wrapper_msix_bottom_mask,
	.irq_unmask = rtd_pci_wrapper_msix_bottom_unmask,
};


static int rtd_pcie_irq_domain_alloc(struct irq_domain *domain,
				    unsigned int virq, unsigned int nr_irqs,
				    void *args)
{
	struct rtd_pcie_port *pp = domain->host_data;
	unsigned long flags;
	u32 i;
	int bit;

	raw_spin_lock_irqsave(&pp->lock, flags);

	bit = bitmap_find_free_region(pp->irq_bitmap, pp->msi_max_vector,
				      order_base_2(nr_irqs));

	raw_spin_unlock_irqrestore(&pp->lock, flags);

	if (bit < 0)
		return -ENOSPC;

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_info(domain, virq + i, bit + i,
				    pp->msi_irq_chip,
				    pp, handle_edge_irq,
				    NULL, NULL);
	}

	return 0;
}

static void rtd_pcie_irq_domain_free(struct irq_domain *domain,
				    unsigned int virq, unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct rtd_pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	bitmap_release_region(pp->irq_bitmap, d->hwirq,
			      order_base_2(nr_irqs));

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static const struct irq_domain_ops rtd_pcie_msi_domain_ops = {
	.alloc	= rtd_pcie_irq_domain_alloc,
	.free	= rtd_pcie_irq_domain_free,
};

int rtd_pcie_allocate_domains(struct rtd_pcie_port *pp)
{
	struct fwnode_handle *fwnode = of_node_to_fwnode(pp->dev->of_node);

	pp->irq_domain = irq_domain_create_linear(fwnode, pp->msi_max_vector,
					       &rtd_pcie_msi_domain_ops, pp);
	if (!pp->irq_domain) {
		dev_err(pp->dev, "Failed to create IRQ domain\n");
		return -ENOMEM;
	}

	irq_domain_update_bus_token(pp->irq_domain, DOMAIN_BUS_NEXUS);

	pp->msi_domain = pci_msi_create_irq_domain(fwnode,
						   &rtd_pcie_msi_domain_info,
						   pp->irq_domain);
	if (!pp->msi_domain) {
		dev_err(pp->dev, "Failed to create MSI domain\n");
		irq_domain_remove(pp->irq_domain);
		return -ENOMEM;
	}

	return 0;
}

void rtd_pcie_free_msi(struct rtd_pcie_port *pp)
{
	if (pp->msi_irq) {
		irq_set_chained_handler(pp->msi_irq, NULL);
		irq_set_handler_data(pp->msi_irq, NULL);
	}

	irq_domain_remove(pp->msi_domain);
	irq_domain_remove(pp->irq_domain);

	if (pp->msi_page)
		__free_page(pp->msi_page);
}

static void rtd_pcie_mac_msi_host_init(struct rtd_pcie_port *pp)
{
	u64 msi_target;

	msi_target = (u64)pp->msi_data;
	writel(upper_32_bits(msi_target), pp->ctrl_base + PCIE_MSI_ADDR_HI);
	writel(lower_32_bits(msi_target), pp->ctrl_base + PCIE_MSI_ADDR_LO);

	writel(~0x0, pp->ctrl_base + PCIE_MSI_INTR0_ENABLE);
}


static int rtd_pcie_mac_msi_init(struct rtd_pcie_port *pp)
{
	int ret;

	pp->msi_max_vector = RTK_MSI_DEF_NUM_VECTORS;
	pp->msi_page = alloc_page(GFP_KERNEL);
	pp->msi_data = dma_map_page(pp->dev, pp->msi_page, 0, PAGE_SIZE,
				    DMA_FROM_DEVICE);
	if (dma_mapping_error(pp->dev, pp->msi_data)) {
		dev_err(pp->dev, "Failed to map MSI data\n");
		__free_page(pp->msi_page);
		pp->msi_page = NULL;
		return -ENOMEM;
	}
	pp->ops->msi_ops->msi_host_init(pp);
	pp->msi_irq_chip = &rtd_pci_mac_msi_bottom_irq_chip;

	ret = rtd_pcie_allocate_domains(pp);
	if (ret)
		return ret;

	if (pp->msi_irq)
		irq_set_chained_handler_and_data(pp->msi_irq,
				rtd_chained_msi_isr, pp);

	return 0;

}

static void rtd_pcie_wrapper_msi_host_init(struct rtd_pcie_port *pp)
{
	writel(readl(pp->ctrl_base + PCIE_INT_CTR) | BIT(12), pp->ctrl_base + PCIE_INT_CTR);
	writel(pp->msi_data, pp->ctrl_base + PCIE_MSI_TRAN);
	*pp->msi_data_virt = 0x0;
}


static int rtd_pcie_wrapper_msi_init(struct rtd_pcie_port *pp)
{
	int ret;

	pp->msi_max_vector = 1;
	pp->msi_data_virt = dma_alloc_coherent(pp->dev, PAGE_SIZE, &pp->msi_data, GFP_KERNEL);
	if (!pp->msi_data_virt)
		return -ENOMEM;

	pp->ops->msi_ops->msi_host_init(pp);
	pp->msi_irq_chip = &rtd_pci_wrapper_msi_bottom_irq_chip;

	ret = rtd_pcie_allocate_domains(pp);
	if (ret)
		return ret;

	if (pp->msi_irq)
		irq_set_chained_handler_and_data(pp->msi_irq,
				rtd_chained_msi_isr, pp);

	return 0;

}

static void rtd_pcie_wrapper_msix_host_init(struct rtd_pcie_port *pp)
{
	writel(0x1, pp->ctrl_base + PCIE_MSIX_CTRL);
	writel(readl(pp->ctrl_base + PCIE_INT_CTR) | BIT(15), pp->ctrl_base + PCIE_INT_CTR);
	writel(pp->msi_data, pp->ctrl_base + PCIE_MSIX_TRAN);
}


static int rtd_pcie_wrapper_msix_init(struct rtd_pcie_port *pp)
{
	int ret;

	pp->msi_max_vector = RTK_MSI_DEF_NUM_VECTORS;
	pp->msi_data_virt = dma_alloc_coherent(pp->dev, PAGE_SIZE, &pp->msi_data, GFP_KERNEL);
	if (!pp->msi_data_virt)
		return -ENOMEM;

	pp->ops->msi_ops->msi_host_init(pp);
	pp->msi_irq_chip = &rtd_pci_wrapper_msix_bottom_irq_chip;

	ret = rtd_pcie_allocate_domains(pp);
	if (ret)
		return ret;

	if (pp->msi_irq)
		irq_set_chained_handler_and_data(pp->msi_irq,
				rtd_chained_msi_isr, pp);

	return 0;

}

static unsigned long pci_byte_mask(unsigned long addr, unsigned char size)
{
	unsigned char offset = (addr & 0x03);

	switch (size) {
	case 0x01:
		return 0x1 << offset;
	case 0x02:
		if (offset <= 2)
			return 0x3 << offset;
		break;

	case 0x03:
		if (offset <= 1)
			return 0x7 << offset;
		break;

	case 0x04:
		if (offset == 0)
			return 0xF;
		break;

	default:
		break;
	}
	return 0;
}

static unsigned long pci_bit_mask(unsigned char byte_mask)
{
	int i;
	unsigned long mask = 0;

	for (i = 0; i < 4; i++) {
		if ((byte_mask >> i) & 0x1)
			mask |= (0xFF << (i << 3));
	}

	return mask;
}

static unsigned long pci_bit_shift(unsigned long addr)
{
	return ((addr & 0x3) << 3);
}


static int indirect_cfg_read(struct rtd_pcie_port *pp, struct pci_bus *bus, unsigned long addr, u32 *pdata,
						unsigned char size)
{
	unsigned long status;
	unsigned char mask;
	int try_count = 20000;

	if (pci_is_root_bus(bus) && ADDR_TO_DEVICE_NO(addr) != 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	mask = pci_byte_mask(addr, size);

	if (!mask)
		return PCIBIOS_SET_FAILED;

	if (pci_is_root_bus(bus))
		writel(0x10, pp->ctrl_base + PCIE_INDIR_CTR);
	else
		writel(0x14, pp->ctrl_base + PCIE_INDIR_CTR);

	writel(CFG_ST_ERROR|CFG_ST_DONE, pp->ctrl_base + PCIE_CFG_ST);
	writel((addr & ~0x3), pp->ctrl_base + PCIE_CFG_ADDR);
	writel(BYTE_CNT(mask) | BYTE_EN | WRRD_EN(0), pp->ctrl_base + PCIE_CFG_EN);
	writel(GO_CT, pp->ctrl_base + PCIE_CFG_CT);

	do {
		status = readl(pp->ctrl_base + PCIE_CFG_ST);
		udelay(50);
	} while (!(status & CFG_ST_DONE) && try_count--);

	if (try_count < 0) {
		dev_err(pp->dev, "Read config data (%p) failed - timeout\n",
											(void *) addr);
		goto error_occur;
	}

	if (readl(pp->ctrl_base + PCIE_CFG_ST) & CFG_ST_ERROR) {
		dev_err(pp->dev, "Read config data failed - error\n");
		goto error_occur;
	}

	writel(CFG_ST_ERROR|CFG_ST_DONE, pp->ctrl_base + PCIE_CFG_ST);
	*pdata = (readl(pp->ctrl_base + PCIE_CFG_RDATA) & pci_bit_mask(mask))
							>> pci_bit_shift(addr);
	return PCIBIOS_SUCCESSFUL;

error_occur:
	writel(CFG_ST_ERROR|CFG_ST_DONE, pp->ctrl_base + PCIE_CFG_ST);
	return PCIBIOS_SET_FAILED;
}

static unsigned long pci_address_conversion(struct pci_bus *bus,
						unsigned int devfn, int reg)
{
	int busno = bus->number;
	int dev = PCI_SLOT(devfn);
	int func = PCI_FUNC(devfn);

	return (busno << 24) | (dev << 19) | (func << 16) | reg;
}


static int rtd_pcie_rd_conf(struct pci_bus *bus, unsigned int devfn,
			     int reg, int size, u32 *pval)
{
	unsigned long address;
	int ret = PCIBIOS_DEVICE_NOT_FOUND;
	u32 val = 0;
	u32 err_st = 0;
	u8 retry = 5;
	struct rtd_pcie_port *pp = bus->sysdata;

again:

	address = pci_address_conversion(bus, devfn, reg);
	ret = indirect_cfg_read(pp, bus, address, pval, size);

	val = readl(pp->ctrl_base + PCIE_RCPL_ST);
	err_st = (val & 0xE0) >> 5;
	if (err_st && retry) {
		retry--;
		switch (err_st) {
		case UR:
			dev_dbg(&bus->dev, "Unsupported Request(0x%x): bus:0x%x dev:0x%x fn:0x%x reg:0x%x size:%d data:0x%x\n",
						val, bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), reg, size, *pval);
			break;
		case CRS:
			dev_err(&bus->dev, "Configuration Request Retry Status(0x%x): bus:0x%x dev:0x%x fn:0x%x reg:0x%x size:%d data:0x%x\n",
						val, bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), reg, size, *pval);
			break;
		case CA:
			dev_err(&bus->dev, "Completer Abort(0x%x): bus:0x%x dev:0x%x fn:0x%x reg:0x%x size:%d data:0x%x\n",
						val, bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), reg, size, *pval);
			break;
		default:
			dev_err(&bus->dev, "Unexpected Error(0x%x): bus:0x%x dev:0x%x fn:0x%x reg:0x%x size:%d data:0x%x\n",
						val, bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), reg, size, *pval);
			break;
		}

		goto again;
	}

	//dev_info(&bus->dev, "rd_conf  bus:%d reg = 0x%x, devfun:%d, size=%d, *pval = 0x%x\n",
	//						bus->number, reg, devfn, size, *pval);

	return ret;
}

static int indirect_cfg_write(struct rtd_pcie_port *pp,
					struct pci_bus *bus, unsigned long addr,
					unsigned long data, unsigned char size)
{
	unsigned long status;
	unsigned char mask;
	int try_count = 1000;

	if (pci_is_root_bus(bus) && ADDR_TO_DEVICE_NO(addr) != 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	mask = pci_byte_mask(addr, size);

	if (!mask)
		return PCIBIOS_SET_FAILED;

	data = (data << pci_bit_shift(addr)) & pci_bit_mask(mask);

	if (pci_is_root_bus(bus))
		writel(0x12, pp->ctrl_base + PCIE_INDIR_CTR);
	else
		writel(0x16, pp->ctrl_base + PCIE_INDIR_CTR);

	writel(CFG_ST_ERROR|CFG_ST_DONE, pp->ctrl_base + PCIE_CFG_ST);
	writel(addr & ~0x3, pp->ctrl_base + PCIE_CFG_ADDR);
	writel(data, pp->ctrl_base + PCIE_CFG_WDATA);

	if (size == 4)
		writel(0x1, pp->ctrl_base + PCIE_CFG_EN);
	else
		writel(BYTE_CNT(mask) | BYTE_EN | WRRD_EN(1), pp->ctrl_base + PCIE_CFG_EN);

	writel(GO_CT, pp->ctrl_base + PCIE_CFG_CT);

	do {
		status = readl(pp->ctrl_base + PCIE_CFG_ST);
		udelay(50);
	} while (!(status & CFG_ST_DONE) && try_count--);

	if (try_count < 0) {
		dev_err(pp->dev, "Write config data (%p) failed - timeout\n",
							(void *) addr);
		goto error_occur;
	}

	if (readl(pp->ctrl_base + PCIE_CFG_ST) & CFG_ST_ERROR) {
		dev_err(pp->dev, "Write config data (%p) failed - error\n",
							(void *) addr);
		goto error_occur;
	}

	writel(CFG_ST_ERROR|CFG_ST_DONE, pp->ctrl_base + PCIE_CFG_ST);

	return PCIBIOS_SUCCESSFUL;

error_occur:

	writel(CFG_ST_ERROR|CFG_ST_DONE, pp->ctrl_base + PCIE_CFG_ST);

	return PCIBIOS_SET_FAILED;
}

static int rtd_pcie_wr_conf(struct pci_bus *bus, unsigned int devfn,
			     int reg, int size, u32 val)
{
	unsigned long address;
	int ret = PCIBIOS_DEVICE_NOT_FOUND;
	struct rtd_pcie_port *pp = bus->sysdata;

	//dev_info(&bus->dev, "wr_conf bus:%d reg = 0x%x, val = 0x%x\n",
	//						bus->number, reg, val);

	address = pci_address_conversion(bus, devfn, reg);
	ret = indirect_cfg_write(pp, bus, address, val, size);

	return ret;
}

static struct pci_ops rtd_pcie_cfg_ops = {
	.read = rtd_pcie_rd_conf,
	.write = rtd_pcie_wr_conf,
};

static int pcie_link_init(struct rtd_pcie_port *pp)
{
	int ret = 0;
	int timeout;
	bool pci_link_detected;
	u32 tmp;

	ret = gpiod_direction_output(pp->perst_gpio, 0);
	if (ret)
		dev_err(pp->dev, "cannot set gpio\n");

	writel(0x00140010, pp->ctrl_base + PCIE_SYS_CTR);
	if (pp->speed_mode == 0) { /* force gen1*/
		tmp = readl(pp->ctrl_base + LINK_CONTROL2_LINK_STATUS2_REG);
		writel(tmp | 0x1, pp->ctrl_base + LINK_CONTROL2_LINK_STATUS2_REG);
	}

	/*phy mdio setting*/
	phy_init(pp->pcie_phy);

	mdelay(100);

	ret = gpiod_direction_output(pp->perst_gpio, 1);
	if (ret)
		dev_err(pp->dev, "cannot set gpio\n");

	writel(0x001E0022, pp->ctrl_base + PCIE_SYS_CTR);
	writel(0x00010120, pp->ctrl_base + PORT_LINK_CTRL_OFF);

	timeout = PCIE_CONNECT_TIMEOUT;
	do {
		pci_link_detected = readl(pp->ctrl_base + PCIE_MAC_ST) & 0x800;
		if (pci_link_detected) {
			dev_info(pp->dev, "link up with device");
			break;
		}
		mdelay(1);
		timeout--;
	} while (timeout > 0);

	if (timeout == 0) {
		if (!pp->debug_mode) {
			ret = pp->ops->deinit(pp);
			if (ret) {
				dev_err(pp->dev, "deinit failed.\n");
				return ret;
			}
		}
		devm_gpiod_put(pp->dev, pp->perst_gpio);
		dev_info(pp->dev, "link down with device");
		return -ENODEV;
	}

	writel(0x7, pp->ctrl_base + TYPE1_STATUS_COMMAND_REG);

	ret = pp->ops->hwinit(pp);
	if (ret) {
		dev_err(pp->dev, "hw init failed.\n");
		return ret;
	}

	/*pcie timeout 500us*/
	writel(0xF42401, pp->ctrl_base + PCIE_DIR_EN);

	/* set limit and base register */
	writel(0x0000FFF0, pp->ctrl_base + MEM_LIMIT_MEM_BASE_REG);
	writel(0x0000FFF0, pp->ctrl_base + PREF_MEM_LIMIT_PREF_MEM_BASE_REG);

	return 0;
}

int rtd_pci_get_host_bridge_resources(struct rtd_pcie_port *pp,
			unsigned char busno, unsigned char bus_max,
			struct list_head *resources, resource_size_t *io_base)
{
	struct device_node *dev_node = pp->dev->of_node;
	struct resource *res, tmp_res;
	struct resource *bus_range;
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	char range_type[4];
	int err;
	int rlen;

	if (io_base)
		*io_base = (resource_size_t)OF_BAD_ADDR;

	bus_range = devm_kzalloc(pp->dev, sizeof(*bus_range), GFP_KERNEL);
	if (!bus_range)
		return -ENOMEM;

	dev_info(pp->dev, "host bridge %pOF ranges:\n", dev_node);

	err = of_pci_parse_bus_range(dev_node, bus_range);
	if (err) {
		bus_range->start = busno;
		bus_range->end = bus_max;
		bus_range->flags = IORESOURCE_BUS;
		dev_info(pp->dev, "  No bus range found for %pOF, using %pR\n",
			 dev_node, bus_range);
	} else {
		if (bus_range->end > bus_range->start + bus_max)
			bus_range->end = bus_range->start + bus_max;
	}
	pci_add_resource(resources, bus_range);

	/* Check for ranges property */
	err = of_pci_range_parser_init(&parser, dev_node);
	if (err)
		goto failed;

	if (pp->workaround) {
		parser.range = of_get_property(dev_node, "workaround-ranges", &rlen);
		if (parser.range == NULL)
			return -ENOENT;

		parser.end = parser.range + rlen / sizeof(__be32);
	}

	dev_dbg(pp->dev, "Parsing ranges property...\n");
	for_each_of_pci_range(&parser, &range) {
		/* Read next ranges element */
		if ((range.flags & IORESOURCE_TYPE_BITS) == IORESOURCE_IO)
			snprintf(range_type, 4, " IO");
		else if ((range.flags & IORESOURCE_TYPE_BITS) == IORESOURCE_MEM)
			snprintf(range_type, 4, "MEM");
		else
			snprintf(range_type, 4, "err");
		dev_info(pp->dev, "  %s %#010llx..%#010llx -> %#010llx\n",
			 range_type, range.cpu_addr,
			 range.cpu_addr + range.size - 1, range.pci_addr);

		/*
		 * If we failed translation or got a zero-sized region
		 * then skip this range
		 */
		if (range.cpu_addr == OF_BAD_ADDR || range.size == 0)
			continue;

		err = of_pci_range_to_resource(&range, dev_node, &tmp_res);
		if (err)
			continue;

		res = devm_kmemdup(pp->dev, &tmp_res, sizeof(tmp_res), GFP_KERNEL);
		if (!res) {
			err = -ENOMEM;
			goto failed;
		}

		if (resource_type(res) == IORESOURCE_IO) {
			if (!io_base) {
				dev_err(pp->dev, "I/O range found for %pOF. Please provide an io_base pointer to save CPU base address\n",
					dev_node);
				err = -EINVAL;
				goto failed;
			}
			if (*io_base != (resource_size_t)OF_BAD_ADDR)
				dev_warn(pp->dev, "More than one I/O resource converted for %pOF. CPU base address for old range lost!\n",
					 dev_node);
			*io_base = range.cpu_addr;
		}

		pci_add_resource_offset(resources, res,	res->start - range.pci_addr);
	}

	return 0;

failed:
	pci_free_resource_list(resources);
	return err;
}


static int rtd_pcie_probe(struct platform_device *pdev)
{
	struct rtd_pcie_port *pp;
	struct device *dev = &pdev->dev;
	int ret = 0;
	struct pci_host_bridge *bridge;
	struct pci_bus *child;
	static struct pci_bus *bus;

	resource_size_t iobase = 0;

	pp = devm_kzalloc(dev, sizeof(*pp), GFP_KERNEL);
	if (!pp)
		return -ENOMEM;
	pp->dev = dev;
	pp->ops = (struct rtd_pcie_ops *)of_device_get_match_data(dev);

	dev_info(dev, "host driver initial begin.\n");

	ret = pp->ops->get_resource(pp);
	if (ret) {
		if (ret == -EPROBE_DEFER)
			goto defer;
		dev_err(dev, "get resource failed.\n");
		goto failed;
	}
	ret = pp->ops->init(pp);
	if (ret) {
		dev_err(dev, "init failed.\n");
		goto failed;
	}

	ret = pcie_link_init(pp);
	if (ret) {
		dev_err(dev, "link init failed.\n");
		goto failed;
	}

	raw_spin_lock_init(&pp->lock);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		ret = pp->ops->msi_ops->msi_init(pp);
		if (ret) {
			dev_err(dev, "msi init failed.\n");
			goto failed;
		}
	}

	platform_set_drvdata(pdev, pp);

	bridge = pci_alloc_host_bridge(0);
	if (!bridge) {
		dev_err(dev, "alloc host bridge failed.\n");
		ret = -ENOMEM;
		goto failed;
	}
	ret = rtd_pci_get_host_bridge_resources(pp,
					0x1, 0xff, &bridge->windows, &iobase);
	if (ret) {
		dev_err(dev, "get host bridge resources failed.\n");
		goto failed;
	}
	ret = devm_request_pci_bus_resources(pp->dev, &bridge->windows);
	if (ret) {
		dev_err(dev, "cannot request pci bus resources\n");
		goto failed;
	}
	bridge->dev.parent = &pdev->dev;
	bridge->ops = &rtd_pcie_cfg_ops;
	bridge->map_irq = of_irq_parse_and_map_pci;
	bridge->swizzle_irq = pci_common_swizzle;
	bridge->sysdata = pp;

	ret = pci_scan_root_bus_bridge(bridge);
	if (ret) {
		dev_err(pp->dev, "scan root bus failed\n");
		goto failed;
	}
	bus = bridge->bus;

	pci_bus_size_bridges(bus);
	pci_bus_assign_resources(bus);

	list_for_each_entry(child, &bus->children, node)
		pcie_bus_configure_settings(child);
	pci_bus_add_devices(bus);

	dev_info(pp->dev, "host driver initial done.\n");

	return ret;
failed:
	dev_info(pp->dev, "host driver initial failed.\n");
	return ret;

defer:
	dev_info(pp->dev, "host driver needs to be deferred probing.\n");
	return ret;
}

static int rtd13xx_pcie0_get_res(struct rtd_pcie_port *pp)
{
	struct device_node *syscon_np;
	const u32 *prop;
	int size = 0;

	pp->ctrl_base = of_iomap(pp->dev->of_node, 0);
	if (!pp->ctrl_base) {
		dev_err(pp->dev, "failed to get ctrl address\n");
		return -EINVAL;
	}

	syscon_np = of_parse_phandle(pp->dev->of_node, "syscon", 0);
	if (IS_ERR_OR_NULL(syscon_np))
		return -ENODEV;

	pp->pinmux_base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(pp->pinmux_base)) {
		of_node_put(syscon_np);
		return -EINVAL;
	}

	prop = of_get_property(pp->dev->of_node, "speed-mode", &size);
	if (prop) {
		pp->speed_mode = of_read_number(prop, 1);
		if (pp->speed_mode == 0)
			dev_info(pp->dev, "Speed Mode: GEN1\n");
		else if (pp->speed_mode == 1)
			dev_info(pp->dev, "Speed Mode: GEN2\n");
	} else {
		pp->speed_mode = 0;
	}

	prop = of_get_property(pp->dev->of_node, "debug-mode", &size);
	if (prop) {
		pp->debug_mode = of_read_number(prop, 1);
		if (pp->debug_mode == 0)
			dev_info(pp->dev, "PCIE Debug Mode off\n");
		else if (pp->debug_mode == 1)
			dev_info(pp->dev, "PCIE Debug Mode on\n");
	} else {
		pp->debug_mode = 0;
	}

	pp->perst_gpio = devm_gpiod_get(pp->dev, "perst", GPIOD_OUT_LOW);
	if (IS_ERR(pp->perst_gpio)) {
		dev_err(pp->dev, "PERST gpio missing or invalid\n");
		return PTR_ERR(pp->perst_gpio);
	}

	pp->pcie_clk = devm_clk_get(pp->dev, NULL);
	if (IS_ERR(pp->pcie_clk)) {
		dev_err(pp->dev, "pcie clock source missing or invalid\n");
		return PTR_ERR(pp->pcie_clk);
	}

	pp->rst_stitch = devm_reset_control_get(pp->dev, "stitch");
	if (IS_ERR(pp->rst_stitch)) {
		dev_err(pp->dev, "stitch source missing or invalid\n");
		return PTR_ERR(pp->rst_stitch);
	}

	pp->rst = devm_reset_control_get(pp->dev, "rstn");
	if (IS_ERR(pp->rst)) {
		dev_err(pp->dev, "rstn_pcie3 source missing or invalid\n");
		return PTR_ERR(pp->rst);
	}

	pp->rst_core = devm_reset_control_get(pp->dev, "core");
	if (IS_ERR(pp->rst_core)) {
		dev_err(pp->dev, "core source missing or invalid\n");
		return PTR_ERR(pp->rst_core);
	}

	pp->rst_power = devm_reset_control_get(pp->dev, "power");
	if (IS_ERR(pp->rst_power)) {
		dev_err(pp->dev, "power source missing or invalid\n");
		return PTR_ERR(pp->rst_power);
	}

	pp->rst_nonstitch = devm_reset_control_get(pp->dev, "nonstitch");
	if (IS_ERR(pp->rst_nonstitch)) {
		dev_err(pp->dev, "nonstitch source missing or invalid\n");
		return PTR_ERR(pp->rst_nonstitch);
	}

	pp->rst_sgmii_mdio = devm_reset_control_get(pp->dev, "sgmii_mdio");
	if (IS_ERR(pp->rst_sgmii_mdio)) {
		dev_err(pp->dev, "phy_mdio source missing. or invalid\n");
		return PTR_ERR(pp->rst_sgmii_mdio);
	}

	pp->pcie_phy = devm_of_phy_get(pp->dev, pp->dev->of_node, NULL);
	if (IS_ERR(pp->pcie_phy)) {
		dev_err(pp->dev, "pcie phy missing or invalid\n");
		return PTR_ERR(pp->pcie_phy);
	}

	pp->msi_irq = of_irq_get(pp->dev->of_node, 0);
	if (pp->msi_irq < 0) {
		dev_err(pp->dev, "parse irq failed\n");
		return pp->msi_irq;
	}

	return 0;
}


static int rtd13xx_pcie0_init(struct rtd_pcie_port *pp)
{
	u32 tmp;
	int ret = 0;

	regmap_read(pp->pinmux_base, ISO_POWERCUT_ETN, &tmp);
	tmp &= ~PCIE0_USB_SEL_OFFSET;
	regmap_write(pp->pinmux_base, ISO_POWERCUT_ETN, tmp);
	reset_control_deassert(pp->rst_stitch);
	reset_control_deassert(pp->rst);
	reset_control_deassert(pp->rst_core);
	reset_control_deassert(pp->rst_power);
	reset_control_deassert(pp->rst_nonstitch);
	reset_control_deassert(pp->rst_sgmii_mdio);
	phy_power_on(pp->pcie_phy);

	ret = clk_prepare_enable(pp->pcie_clk);

	if (ret) {
		dev_err(pp->dev, "unable to enable pcie clock\n");
		clk_disable_unprepare(pp->pcie_clk);
		return -EINVAL;
	}

	return 0;
}

static int rtd13xx_pcie0_deinit(struct rtd_pcie_port *pp)
{
	clk_disable_unprepare(pp->pcie_clk);
	reset_control_assert(pp->rst_stitch);
	reset_control_assert(pp->rst);
	reset_control_assert(pp->rst_core);
	reset_control_assert(pp->rst_power);
	reset_control_assert(pp->rst_nonstitch);
	reset_control_assert(pp->rst_sgmii_mdio);
	phy_power_off(pp->pcie_phy);

	return 0;
}

static int rtd13xx_pcie0_hw_init(struct rtd_pcie_port *pp)
{
	/* #Base 0 */
	writel(0x98062000, pp->ctrl_base + PCIE_BASE_0);

	/* #Mask 0 */
	writel(0xFFF00000, pp->ctrl_base + PCIE_MASK_0);

	/* #translate for MMIO R/W */
	writel(0x98000000, pp->ctrl_base + PCIE_TRANS_0);

	writel(0x0, pp->ctrl_base + PCIE_SCTCH);

	return 0;
}

static int rtd13xx_pcie1_get_res(struct rtd_pcie_port *pp)
{
	struct device_node *syscon_np;
	const u32 *prop;
	int size = 0;

	pp->ctrl_base = of_iomap(pp->dev->of_node, 0);
	if (!pp->ctrl_base) {
		dev_err(pp->dev, "failed to get ctrl address\n");
		return -EINVAL;
	}

	syscon_np = of_parse_phandle(pp->dev->of_node, "syscon", 0);
	if (IS_ERR_OR_NULL(syscon_np))
		return -ENODEV;

	pp->main2_misc_base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(pp->main2_misc_base)) {
		of_node_put(syscon_np);
		return -EINVAL;
	}

	prop = of_get_property(pp->dev->of_node, "speed-mode", &size);
	if (prop) {
		pp->speed_mode = of_read_number(prop, 1);
		if (pp->speed_mode == 0)
			dev_info(pp->dev, "Speed Mode: GEN1\n");
		else if (pp->speed_mode == 1)
			dev_info(pp->dev, "Speed Mode: GEN2\n");
	} else {
		pp->speed_mode = 0;
	}

	prop = of_get_property(pp->dev->of_node, "debug-mode", &size);
	if (prop) {
		pp->debug_mode = of_read_number(prop, 1);
		if (pp->debug_mode == 0)
			dev_info(pp->dev, "PCIE Debug Mode off\n");
		else if (pp->debug_mode == 1)
			dev_info(pp->dev, "PCIE Debug Mode on\n");
	} else {
		pp->debug_mode = 0;
	}

	prop = of_get_property(pp->dev->of_node, "workaround", &size);
	if (prop) {
		pp->workaround = of_read_number(prop, 1);
		if (pp->workaround == 1)
			dev_info(pp->dev, "PCIE workaround on\n");
	} else {
		pp->workaround = 0;
	}

	pp->perst_gpio = devm_gpiod_get(pp->dev, "perst", GPIOD_OUT_LOW);
	if (IS_ERR(pp->perst_gpio)) {
		dev_err(pp->dev, "PERST gpio missing or invalid\n");
		return PTR_ERR(pp->perst_gpio);
	}

	pp->pcie_clk = devm_clk_get(pp->dev, NULL);
	if (IS_ERR(pp->pcie_clk)) {
		dev_err(pp->dev, "pcie clock source missing or invalid\n");
		return PTR_ERR(pp->pcie_clk);
	}

	pp->rst_stitch = devm_reset_control_get(pp->dev, "stitch");
	if (IS_ERR(pp->rst_stitch)) {
		dev_err(pp->dev, "stitch source missing or invalid\n");
		return PTR_ERR(pp->rst_stitch);
	}

	pp->rst = devm_reset_control_get(pp->dev, "rstn");
	if (IS_ERR(pp->rst)) {
		dev_err(pp->dev, "rstn_pcie3 source missing or invalid\n");
		return PTR_ERR(pp->rst);
	}

	pp->rst_core = devm_reset_control_get(pp->dev, "core");
	if (IS_ERR(pp->rst_core)) {
		dev_err(pp->dev, "core source missing or invalid\n");
		return PTR_ERR(pp->rst_core);
	}

	pp->rst_power = devm_reset_control_get(pp->dev, "power");
	if (IS_ERR(pp->rst_power)) {
		dev_err(pp->dev, "power source missing or invalid\n");
		return PTR_ERR(pp->rst_power);
	}

	pp->rst_nonstitch = devm_reset_control_get(pp->dev, "nonstitch");
	if (IS_ERR(pp->rst_nonstitch)) {
		dev_err(pp->dev, "nonstitch source missing or invalid\n");
		return PTR_ERR(pp->rst_nonstitch);
	}

	pp->pcie_phy = devm_of_phy_get(pp->dev, pp->dev->of_node, NULL);
	if (IS_ERR(pp->pcie_phy)) {
		dev_err(pp->dev, "pcie phy missing or invalid\n");
		return PTR_ERR(pp->pcie_phy);
	}

	pp->msi_irq = of_irq_get(pp->dev->of_node, 0);
	if (pp->msi_irq < 0) {
		dev_err(pp->dev, "parse irq failed\n");
		return pp->msi_irq;
	}

	return 0;
}

static int rtd13xx_pcie1_init(struct rtd_pcie_port *pp)
{
	u32 tmp;
	int ret = 0;

	regmap_read(pp->main2_misc_base, MISC_PHY_CTRL, &tmp);
	tmp &= ~PCIE1_SATA_SEL_OFFSET;
	regmap_write(pp->main2_misc_base, MISC_PHY_CTRL, tmp);
	reset_control_deassert(pp->rst_stitch);
	reset_control_deassert(pp->rst);
	reset_control_deassert(pp->rst_core);
	reset_control_deassert(pp->rst_power);
	reset_control_deassert(pp->rst_nonstitch);
	phy_power_on(pp->pcie_phy);

	ret = clk_prepare_enable(pp->pcie_clk);

	if (ret) {
		dev_err(pp->dev, "unable to enable pcie clock\n");
		clk_disable_unprepare(pp->pcie_clk);
		return -EINVAL;
	}

	return 0;
}

static int rtd13xx_pcie1_deinit(struct rtd_pcie_port *pp)
{
	clk_disable_unprepare(pp->pcie_clk);
	reset_control_assert(pp->rst_stitch);
	reset_control_assert(pp->rst);
	reset_control_assert(pp->rst_core);
	reset_control_assert(pp->rst_power);
	reset_control_assert(pp->rst_nonstitch);
	phy_power_off(pp->pcie_phy);

	return 0;
}

static int rtd13xx_pcie1_hw_init(struct rtd_pcie_port *pp)
{
	if (pp->workaround) {
		/* #Base 0 */
		writel(0x980B0000, pp->ctrl_base + PCIE_BASE_0);

		/* #Mask 0 */
		writel(0xFFFF0000, pp->ctrl_base + PCIE_MASK_0);

		/* #translate for MMIO R/W */
		writel(0xC0000000, pp->ctrl_base + PCIE_TRANS_0);
	} else {
		/* #Base 0 */
		writel(0x980A2000, pp->ctrl_base + PCIE_BASE_0);

		/* #Mask 0 */
		writel(0xFFF00000, pp->ctrl_base + PCIE_MASK_0);

		/* #translate for MMIO R/W */
		writel(0x98000000, pp->ctrl_base + PCIE_TRANS_0);
	}

	writel(0x0, pp->ctrl_base + PCIE_SCTCH);

	return 0;
}

static int rtd13xx_pcie2_get_res(struct rtd_pcie_port *pp)
{
	struct device_node *syscon_np;
	const u32 *prop;
	int size = 0;

	pp->ctrl_base = of_iomap(pp->dev->of_node, 0);
	if (!pp->ctrl_base) {
		dev_err(pp->dev, "failed to get ctrl address\n");
		return -EINVAL;
	}

	syscon_np = of_parse_phandle(pp->dev->of_node, "syscon", 0);
	if (IS_ERR_OR_NULL(syscon_np))
		return -ENODEV;

	pp->main2_misc_base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(pp->main2_misc_base)) {
		of_node_put(syscon_np);
		return -EINVAL;
	}

	prop = of_get_property(pp->dev->of_node, "speed-mode", &size);
	if (prop) {
		pp->speed_mode = of_read_number(prop, 1);
		if (pp->speed_mode == 0)
			dev_info(pp->dev, "Speed Mode: GEN1\n");
		else if (pp->speed_mode == 1)
			dev_info(pp->dev, "Speed Mode: GEN2\n");
	} else {
		pp->speed_mode = 0;
	}

	prop = of_get_property(pp->dev->of_node, "debug-mode", &size);
	if (prop) {
		pp->debug_mode = of_read_number(prop, 1);
		if (pp->debug_mode == 0)
			dev_info(pp->dev, "PCIE Debug Mode off\n");
		else if (pp->debug_mode == 1)
			dev_info(pp->dev, "PCIE Debug Mode on\n");
	} else {
		pp->debug_mode = 0;
	}

	prop = of_get_property(pp->dev->of_node, "workaround", &size);
	if (prop) {
		pp->workaround = of_read_number(prop, 1);
		if (pp->workaround == 1)
			dev_info(pp->dev, "PCIE workaround on\n");
	} else {
		pp->workaround = 0;
	}

	pp->perst_gpio = devm_gpiod_get(pp->dev, "perst", GPIOD_OUT_LOW);
	if (IS_ERR(pp->perst_gpio)) {
		dev_err(pp->dev, "PERST gpio missing or invalid\n");
		return PTR_ERR(pp->perst_gpio);
	}

	pp->pcie_clk = devm_clk_get(pp->dev, NULL);
	if (IS_ERR(pp->pcie_clk)) {
		dev_err(pp->dev, "pcie clock source missing or invalid\n");
		return PTR_ERR(pp->pcie_clk);
	}

	pp->rst_stitch = devm_reset_control_get(pp->dev, "stitch");
	if (IS_ERR(pp->rst_stitch)) {
		dev_err(pp->dev, "stitch source missing or invalid\n");
		return PTR_ERR(pp->rst_stitch);
	}

	pp->rst = devm_reset_control_get(pp->dev, "rstn");
	if (IS_ERR(pp->rst)) {
		dev_err(pp->dev, "rstn_pcie3 source missing or invalid\n");
		return PTR_ERR(pp->rst);
	}

	pp->rst_core = devm_reset_control_get(pp->dev, "core");
	if (IS_ERR(pp->rst_core)) {
		dev_err(pp->dev, "core source missing or invalid\n");
		return PTR_ERR(pp->rst_core);
	}

	pp->rst_power = devm_reset_control_get(pp->dev, "power");
	if (IS_ERR(pp->rst_power)) {
		dev_err(pp->dev, "power source missing or invalid\n");
		return PTR_ERR(pp->rst_power);
	}

	pp->rst_nonstitch = devm_reset_control_get(pp->dev, "nonstitch");
	if (IS_ERR(pp->rst_nonstitch)) {
		dev_err(pp->dev, "nonstitch source missing or invalid\n");
		return PTR_ERR(pp->rst_nonstitch);
	}

	pp->pcie_phy = devm_of_phy_get(pp->dev, pp->dev->of_node, NULL);
	if (IS_ERR(pp->pcie_phy)) {
		dev_err(pp->dev, "pcie phy missing or invalid\n");
		return PTR_ERR(pp->pcie_phy);
	}

	pp->msi_irq = of_irq_get(pp->dev->of_node, 0);
	if (pp->msi_irq < 0) {
		dev_err(pp->dev, "parse irq failed\n");
		return pp->msi_irq;
	}

	return 0;
}

static int rtd13xx_pcie2_init(struct rtd_pcie_port *pp)
{
	u32 tmp;
	int ret = 0;

	regmap_read(pp->main2_misc_base, MISC_PHY_CTRL, &tmp);
	tmp &= ~PCIE2_SATA_SEL_OFFSET;
	regmap_write(pp->main2_misc_base, MISC_PHY_CTRL, tmp);
	reset_control_deassert(pp->rst_stitch);
	reset_control_deassert(pp->rst);
	reset_control_deassert(pp->rst_core);
	reset_control_deassert(pp->rst_power);
	reset_control_deassert(pp->rst_nonstitch);
	phy_power_on(pp->pcie_phy);

	ret = clk_prepare_enable(pp->pcie_clk);

	if (ret) {
		dev_err(pp->dev, "unable to enable pcie clock\n");
		clk_disable_unprepare(pp->pcie_clk);
		return -EINVAL;
	}

	return 0;
}

static int rtd13xx_pcie2_deinit(struct rtd_pcie_port *pp)
{
	clk_disable_unprepare(pp->pcie_clk);
	reset_control_assert(pp->rst_stitch);
	reset_control_assert(pp->rst);
	reset_control_assert(pp->rst_core);
	reset_control_assert(pp->rst_power);
	reset_control_assert(pp->rst_nonstitch);
	phy_power_off(pp->pcie_phy);

	return 0;
}

static int rtd13xx_pcie2_hw_init(struct rtd_pcie_port *pp)
{

	if (pp->workaround) {
		/* #Base 0 */
		writel(0x980D0000, pp->ctrl_base + PCIE_BASE_0);

		/* #Mask 0 */
		writel(0xFFFF0000, pp->ctrl_base + PCIE_MASK_0);

		/* #translate for MMIO R/W */
		writel(0xC1000000, pp->ctrl_base + PCIE_TRANS_0);
	} else {
		/* #Base 0 */
		writel(0x980C2000, pp->ctrl_base + PCIE_BASE_0);

		/* #Mask 0 */
		writel(0xFFF00000, pp->ctrl_base + PCIE_MASK_0);

		/* #translate for MMIO R/W */
		writel(0x98000000, pp->ctrl_base + PCIE_TRANS_0);
	}

	writel(0x0, pp->ctrl_base + PCIE_SCTCH);

	return 0;
}
#ifdef CONFIG_RTD16XXB_PCIE1_ACP
static int acp_init(struct rtd_pcie_port *pp)
{
	struct device_node *syscon_np;
	struct regmap *scpu_wrapper_base;
	u32 tmp;

	syscon_np = of_parse_phandle(pp->dev->of_node, "syscon-scpu-wrapper", 0);
	if (IS_ERR_OR_NULL(syscon_np))
		return -ENODEV;

	scpu_wrapper_base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(pp->main2_misc_base)) {
		of_node_put(syscon_np);
		return -EINVAL;
	}

	/*0x9801D124[0] = 0 SCPU_WRAPPER INTERFACE_EN*/
	regmap_write(scpu_wrapper_base, INTERFACE_EN, 0x0);
	/*0x9801D100 SCPU_WRAPPER SC_CRT_CTRL*/
	regmap_read(scpu_wrapper_base, SC_CRT_CTRL, &tmp);
	tmp |= 0xA0204000;
	regmap_write(scpu_wrapper_base, SC_CRT_CTRL, tmp);
	/*0x9801D800 SCPU_WRAPPER SC_ACP_CTRL*/
	regmap_write(scpu_wrapper_base, SC_ACP_CTRL, 0x100F01FF);
	/*0x9801D030 SCPU_WRAPPER SC_ACP_CRT_CTRL*/
	regmap_write(scpu_wrapper_base, SC_ACP_CRT_CTRL, 0x10010001);
	writel(0x3D820, pp->ctrl_base + PCIE_ACP_CTRL);

	return 0;
}
#endif

static int rtd16xxb_pcie1_hw_init(struct rtd_pcie_port *pp)
{
	struct device_node *syscon_np;
	struct regmap *scpu_wrapper_base;
	u32 regmap_st = 0;
	int timeout;
	int err;
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	u32 mmio_start, mmio_end;

	syscon_np = of_parse_phandle(pp->dev->of_node, "syscon-scpu-wrapper", 0);
	if (IS_ERR_OR_NULL(syscon_np))
		return -ENODEV;

	scpu_wrapper_base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(pp->main2_misc_base)) {
		of_node_put(syscon_np);
		return -EINVAL;
	}

	err = of_pci_range_parser_init(&parser, pp->dev->of_node);
	if (err) {
		dev_err(pp->dev, "parsing range failed\n");
		return err;
	}
	for_each_of_pci_range(&parser, &range) {
		if ((range.flags & IORESOURCE_TYPE_BITS) == IORESOURCE_MEM) {
			mmio_start = range.cpu_addr;
			mmio_end = range.cpu_addr + range.size - 1;
		}
	}

	regmap_write(scpu_wrapper_base, PCIE1_START, mmio_start);
	regmap_write(scpu_wrapper_base, PCIE1_END, mmio_end);
	regmap_write(scpu_wrapper_base, PCIE1_CTRL, 0x1);
	timeout = 500;
	while (regmap_st != 0x1 && timeout > 0) {
		regmap_read(scpu_wrapper_base, PCIE1_STAT, &regmap_st);
		mdelay(1);
		timeout--;
	}
	if (timeout == 0) {
		dev_err(pp->dev, "failed to set mmio start/end\n");
		return -EINVAL;
	}

	writel(mmio_start, pp->ctrl_base + PCIE_DDR_START);
	writel(mmio_end, pp->ctrl_base + PCIE_DDR_END);
	writel(readl(pp->ctrl_base + PCIE_SYS_CTR) | 0x24000000, pp->ctrl_base + PCIE_SYS_CTR);
	writel(mmio_start, pp->ctrl_base + PCI_BASE_2);

	writel(0x0, pp->ctrl_base + PCIE_SERVICE_REGION);

#ifdef CONFIG_RTD16XXB_PCIE1_ACP
	if (acp_init(pp))
		dev_err(pp->dev, "acp init failed.\n");
#endif

	return 0;
}

static int rtd16xxb_pcie2_hw_init(struct rtd_pcie_port *pp)
{
	struct device_node *syscon_np;
	struct regmap *scpu_wrapper_base;
	u32 regmap_st = 0;
	int timeout;
	int err;
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	u32 mmio_start, mmio_end;

	syscon_np = of_parse_phandle(pp->dev->of_node, "syscon-scpu-wrapper", 0);
	if (IS_ERR_OR_NULL(syscon_np))
		return -ENODEV;

	scpu_wrapper_base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(pp->main2_misc_base)) {
		of_node_put(syscon_np);
		return -EINVAL;
	}

	err = of_pci_range_parser_init(&parser, pp->dev->of_node);
	if (err) {
		dev_err(pp->dev, "parsing range failed\n");
		return err;
	}
	for_each_of_pci_range(&parser, &range) {
		if ((range.flags & IORESOURCE_TYPE_BITS) == IORESOURCE_MEM) {
			mmio_start = range.cpu_addr;
			mmio_end = range.cpu_addr + range.size - 1;
		}
	}

	regmap_write(scpu_wrapper_base, PCIE2_START, mmio_start);
	regmap_write(scpu_wrapper_base, PCIE2_END, mmio_end);
	regmap_write(scpu_wrapper_base, PCIE2_CTRL, 0x1);
	timeout = 500;
	while (regmap_st != 0x1 && timeout > 0) {
		regmap_read(scpu_wrapper_base, PCIE2_STAT, &regmap_st);
		mdelay(1);
		timeout--;
	}
	if (timeout == 0) {
		dev_err(pp->dev, "failed to set mmio start/end\n");
		return -EINVAL;
	}

	writel(mmio_start, pp->ctrl_base + PCIE_DDR_START);
	writel(mmio_end, pp->ctrl_base + PCIE_DDR_END);
	writel(readl(pp->ctrl_base + PCIE_SYS_CTR) | 0x24000000, pp->ctrl_base + PCIE_SYS_CTR);
	writel(mmio_start, pp->ctrl_base + PCI_BASE_2);


	writel(0x0, pp->ctrl_base + PCIE_SERVICE_REGION);

	return 0;
}

static const struct rtd_msi_ops rtd_wrapper_msix_ops = {
	.msi_init = rtd_pcie_wrapper_msix_init,
	.msi_host_init = rtd_pcie_wrapper_msix_host_init,
	.irq_handle = rtd_handle_wrapper_msix_irq,
};


static const struct rtd_msi_ops rtd_wrapper_msi_ops = {
	.msi_init = rtd_pcie_wrapper_msi_init,
	.msi_host_init = rtd_pcie_wrapper_msi_host_init,
	.irq_handle = rtd_handle_wrapper_msi_irq,
};

static const struct rtd_msi_ops rtd_mac_msi_ops = {
	.msi_init = rtd_pcie_mac_msi_init,
	.msi_host_init = rtd_pcie_mac_msi_host_init,
	.irq_handle = rtd_handle_mac_msi_irq,
};

static const struct rtd_pcie_ops rtd13xx_pcie0_ops = {
	.name = "pcie0",
	.msi_ops = &rtd_wrapper_msi_ops,
	.get_resource = rtd13xx_pcie0_get_res,
	.init = rtd13xx_pcie0_init,
	.deinit = rtd13xx_pcie0_deinit,
	.hwinit = rtd13xx_pcie0_hw_init,
};

static const struct rtd_pcie_ops rtd13xx_pcie1_ops = {
	.name = "pcie1",
	.msi_ops = &rtd_wrapper_msi_ops,
	.get_resource = rtd13xx_pcie1_get_res,
	.init = rtd13xx_pcie1_init,
	.deinit = rtd13xx_pcie1_deinit,
	.hwinit = rtd13xx_pcie1_hw_init,
};

static const struct rtd_pcie_ops rtd13xx_pcie2_ops = {
	.name = "pcie2",
	.msi_ops = &rtd_wrapper_msi_ops,
	.get_resource = rtd13xx_pcie2_get_res,
	.init = rtd13xx_pcie2_init,
	.deinit = rtd13xx_pcie2_deinit,
	.hwinit = rtd13xx_pcie2_hw_init,
};

static const struct rtd_pcie_ops rtd16xxb_pcie1_ops = {
	.name = "pcie1",
	.msi_ops = &rtd_mac_msi_ops,
	.get_resource = rtd13xx_pcie1_get_res,
	.init = rtd13xx_pcie1_init,
	.deinit = rtd13xx_pcie1_deinit,
	.hwinit = rtd16xxb_pcie1_hw_init,
};

static const struct rtd_pcie_ops rtd16xxb_pcie2_ops = {
	.name = "pcie2",
	.msi_ops = &rtd_mac_msi_ops,
	.get_resource = rtd13xx_pcie2_get_res,
	.init = rtd13xx_pcie2_init,
	.deinit = rtd13xx_pcie2_deinit,
	.hwinit = rtd16xxb_pcie2_hw_init,
};

static const struct rtd_pcie_ops rtd13xxd_pcie1_ops = {
	.name = "pcie1",
	.msi_ops = &rtd_mac_msi_ops,
	.get_resource = rtd13xx_pcie1_get_res,
	.init = rtd13xx_pcie1_init,
	.deinit = rtd13xx_pcie1_deinit,
	.hwinit = rtd16xxb_pcie1_hw_init,
};


static const struct of_device_id rtd_pcie_match_table[] = {
	{.compatible = "realtek,rtd13xx-pcie-slot0", .data = &rtd13xx_pcie0_ops},
	{.compatible = "realtek,rtd13xx-pcie-slot1", .data = &rtd13xx_pcie1_ops},
	{.compatible = "realtek,rtd13xx-pcie-slot2", .data = &rtd13xx_pcie2_ops},
	{.compatible = "realtek,rtd16xxb-pcie-slot1", .data = &rtd16xxb_pcie1_ops},
	{.compatible = "realtek,rtd16xxb-pcie-slot2", .data = &rtd16xxb_pcie2_ops},
	{.compatible = "realtek,rtd13xxd-pcie-slot1", .data = &rtd13xxd_pcie1_ops},
	{},
};

static int rtd_pcie_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtd_pcie_port *pp = platform_get_drvdata(pdev);
	int ret = 0;

	dev_info(dev, "suspend enter ...\n");

	ret = gpiod_direction_output(pp->perst_gpio, 0);
	if (ret)
		dev_err(pp->dev, "cannot set gpio\n");

	ret = pp->ops->deinit(pp);
	if (ret) {
		dev_err(dev, "deinit failed.\n");
		return ret;
	}

	dev_info(dev, "suspend exit ...\n");

	return 0;
}

static int rtd_pcie_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtd_pcie_port *pp = platform_get_drvdata(pdev);
	int ret = 0;

	dev_info(dev, "resume enter ...\n");

	ret = pp->ops->init(pp);
	if (ret) {
		dev_err(dev, "init failed.\n");
		return ret;
	}

	ret = gpiod_direction_output(pp->perst_gpio, 1);
	if (ret)
		dev_err(pp->dev, "cannot set gpio\n");

	ret = pcie_link_init(pp);
	if (ret) {
		dev_err(dev, "link init failed.\n");
		return ret;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI))
		pp->ops->msi_ops->msi_host_init(pp);

	dev_info(dev, "resume exit ...\n");

	return ret;
}

static void rtd_pcie_shutdown(struct platform_device *pdev)
{
	struct rtd_pcie_port *pp = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret = 0;

	dev_info(dev, "shutdown enter ...\n");

	ret = gpiod_direction_output(pp->perst_gpio, 0);
	if (ret)
		dev_err(pp->dev, "cannot set gpio\n");

	ret = pp->ops->deinit(pp);
	if (ret)
		dev_err(dev, "deinit failed.\n");

	dev_info(&pdev->dev, "shutdown exit ...\n");
}

static const struct dev_pm_ops rtd_pcie_pm_ops = {
	.suspend_noirq = rtd_pcie_suspend,
	.resume_noirq = rtd_pcie_resume,
};

static struct platform_driver rtd_pcie_driver = {
	.driver = {
		.name = "Realtek DHC PCIe",
		.of_match_table = of_match_ptr(rtd_pcie_match_table),
#ifdef CONFIG_SUSPEND
		.pm = &rtd_pcie_pm_ops,
#endif
	},
	.probe = rtd_pcie_probe,
	.shutdown = rtd_pcie_shutdown,
};
module_platform_driver(rtd_pcie_driver);

MODULE_AUTHOR("TYChang <tychang@realtek.com>");
MODULE_DESCRIPTION("Realtek PCIe host controller driver");
MODULE_LICENSE("GPL v2");
