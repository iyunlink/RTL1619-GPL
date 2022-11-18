/*
 * rtk_sb2_sem.c - Realtek SB2 HW semaphore API
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 * Copyright (C) 2017 Cheng-Yu Lee <cylee12@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/idr.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <soc/realtek/rtk_sb2_sem.h>


#define SB2_SEM_TRYLOCK_MAX 1024
#define SB2_SEM_RETRY_MAX 10

struct sb2_sem {
	int id;
	struct list_head list;
	struct device_node *np;
	void __iomem *reg;
};

int sb2_sem_try_lock(struct sb2_sem *sem, unsigned int flags)
{
	unsigned int val;

	if (IS_ERR_OR_NULL(sem))
		return -EINVAL;

	val = readl(sem->reg);
	return val;

}
EXPORT_SYMBOL_GPL(sb2_sem_try_lock);

void sb2_sem_lock(struct sb2_sem *sem, unsigned int flags)
{
	int i;
	unsigned int val;
	int cnt  = 0;

	if (IS_ERR_OR_NULL(sem))
		return;

retry:
	for (i = 0; i < SB2_SEM_TRYLOCK_MAX; i++) {
		val = sb2_sem_try_lock(sem, flags);
		if (val)
			return;
	}

	if (flags & SB2_SEM_TIMEOUT_INFINITY)
		goto retry;

	if (!(flags & SB2_SEM_NO_WARNING))
		pr_warn("sb2-sem: trylock fail\n");
	cnt++;
	if (cnt < SB2_SEM_RETRY_MAX)
		goto retry;

	if (!(flags & SB2_SEM_NO_WARNING))
		pr_warn("sb2-sem: ignore lock\n");
}
EXPORT_SYMBOL_GPL(sb2_sem_lock);

#define SB2_SEM_LOCK_WAIT 100

void sb2_sem_lock_interruptible(struct sb2_sem *sem, unsigned int flags)
{
	int i;
	unsigned int val;
	int cnt  = 0;

	if (IS_ERR_OR_NULL(sem)) {
		pr_err("Invalid parameter sem %p\n", sem);
		return;
	}

retry:
	for (i = 0; i < SB2_SEM_TRYLOCK_MAX; i++) {
		val = readl(sem->reg);
		if (val)
			return;
		usleep_range(SB2_SEM_LOCK_WAIT, SB2_SEM_LOCK_WAIT + 1);
	}

	if (flags & SB2_SEM_TIMEOUT_INFINITY)
		goto retry;

	if (!(flags & SB2_SEM_NO_WARNING))
		pr_warn("sb2-sem: trylock fail\n");
	cnt++;
	if (cnt < SB2_SEM_RETRY_MAX)
		goto retry;

	if (!(flags & SB2_SEM_NO_WARNING))
		pr_warn("sb2-sem: ignore lock\n");
}
EXPORT_SYMBOL_GPL(sb2_sem_lock_interruptible);

void sb2_sem_unlock(struct sb2_sem *sem)
{
	if (IS_ERR_OR_NULL(sem))
		return;

	writel(0, sem->reg);
}
EXPORT_SYMBOL_GPL(sb2_sem_unlock);

static LIST_HEAD(sb2_list);
static DEFINE_SPINLOCK(sb2_list_lock);

struct sb2_sem *sb2_sem_get(unsigned int index)
{
	struct sb2_sem *sb2 = ERR_PTR(-EINVAL), *p;

	spin_lock(&sb2_list_lock);
	list_for_each_entry(p, &sb2_list, list)
		if (!index--) {
			sb2 = p;
			break;
		}
	spin_unlock(&sb2_list_lock);

	return sb2;
}
EXPORT_SYMBOL_GPL(sb2_sem_get);

static struct sb2_sem *create_sb2_sem(struct device_node *np)
{
	struct sb2_sem *sb2;

	sb2 = kzalloc(sizeof(*sb2), GFP_KERNEL);
	if (!sb2)
		return ERR_PTR(-ENOMEM);

	sb2->np = np;
	sb2->reg = of_iomap(np, 0);

	spin_lock(&sb2_list_lock);
	list_add_tail(&sb2->list, &sb2_list);
	spin_unlock(&sb2_list_lock);

	return sb2;
}

struct sb2_sem *sb2_sem_node_to_lock(struct device_node *np)
{
	struct sb2_sem *sb2 = NULL, *p;

	spin_lock(&sb2_list_lock);
	list_for_each_entry(p, &sb2_list, list)
		if (p->np == np) {
			sb2 = p;
			break;
		}
	spin_unlock(&sb2_list_lock);

	if (!sb2)
		sb2 = create_sb2_sem(np);

	return sb2;
}
EXPORT_SYMBOL_GPL(sb2_sem_node_to_lock);

static __init int init_sb2_sem(void)
{
	struct device_node *np;

	for_each_compatible_node(np, NULL, "realtek,sb2-sem") {
		sb2_sem_node_to_lock(np);
	}
	return 0;
}
early_initcall(init_sb2_sem);

MODULE_LICENSE("GPL v2");
