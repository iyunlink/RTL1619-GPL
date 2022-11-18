/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include "pool.h"
#include <linux/sched/task.h>
#include <linux/dma-mapping.h>

static u64 dma_mask;

void page_common_init(struct page_info *info, unsigned long base, size_t size,
		      unsigned long flags)
{

	struct task_struct *task;
	pid_t pid, tgid;
	char task_comm[TASK_COMM_LEN];

	get_task_struct(current->group_leader);

	task_lock(current->group_leader);
	pid = task_pid_nr(current->group_leader);
	tgid = task_tgid_nr(current->group_leader);
	if (current->group_leader->flags & PF_KTHREAD)
		task = NULL;
	else
		task = current->group_leader;
	task_unlock(current->group_leader);

	if (task)
		get_task_comm(task_comm, task);
	else
		strncpy(task_comm, "KTHREAD", sizeof(task_comm));

	/* decrease usage count */
	put_task_struct(current->group_leader);

	memset(info, 0, sizeof(struct pool_info));
	info->base = base;
	info->size = size;
	info->flags = flags;
	info->pid = pid;
	info->tgid = tgid;
	strncpy(info->task_comm, task_comm, sizeof(info->task_comm));
}

void page_common_sync(struct device *dev, struct page_info *page_info, enum dma_data_direction dir)
{
	struct sg_table *sg_table = page_info->get_sg_table(page_info);
	int nents;

	if (!ion_flag_canAccess(page_info->flags)) {
		pr_err
		    ("%s : can not access [base=0x%08lx, size=%zu, flags=0x%lx]",
		     __func__, page_info->base, page_info->size,
		     page_info->flags);
		return;
	}

	dma_mask = 0xffffffff;
	dev->dma_mask = &dma_mask;
	nents = dma_map_sg(dev, sg_table->sgl, sg_table->nents, dir);
	dma_sync_sg_for_device(dev, sg_table->sgl, sg_table->nents, dir);
	if (nents > 0)
		dma_unmap_sg(dev, sg_table->sgl, sg_table->nents, dir);
}

void page_common_sync_range(struct device *dev, struct page_info *page_info,
			    enum dma_data_direction dir, unsigned long addr,
			    size_t size)
{
	struct sg_table *sg_table = page_info->get_sg_table(page_info);
	int nents;

	if (!ion_flag_canAccess(page_info->flags)) {
		pr_err
		    ("%s : can not access [base=0x%08lx, size=%zu, flags=0x%lx]",
		     __func__, page_info->base, page_info->size,
		     page_info->flags);
		return;
	}

	if (addr < page_info->base
	    || (addr + size) > (page_info->base + page_info->size)) {
		pr_err
		    ("%s : range(0x%08lx, %zu) is mismatch! [base=0x%08lx, size=%zu, flags=0x%lx]",
		     __func__, addr, size, page_info->base, page_info->size,
		     page_info->flags);
		return;
	}

	dma_mask = 0xffffffff;
	dev->dma_mask = &dma_mask;
	nents = dma_map_sg(dev, sg_table->sgl, sg_table->nents, dir);
	dma_sync_single_for_device(dev, addr, size, dir);
	if (nents > 0)
		dma_unmap_sg(dev, sg_table->sgl, sg_table->nents, dir);
}

void pool_common_init(struct pool_info *info, enum pool_type type,
		      unsigned long base, size_t size, unsigned long rtk_flags)
{
	memset(info, 0, sizeof(struct pool_info));
	INIT_LIST_HEAD(&info->list);
	info->type = type;
	info->base = base;
	info->size = size;
	info->rtk_flags = rtk_flags;
}

void page_common_debug_show(struct page_info *page_info,
			    struct seq_buf *s, char *prefix)
{
	if (!prefix)
		prefix = "";

	if (!s || !page_info)
		return;

	seq_buf_printf(s,
		   "%s%-16s : flags=%08lx | %08lx ~ %08lx | %11zu\t",
		   prefix, &page_info->task_comm[0], page_info->flags,
		   page_info->base, page_info->base + page_info->size - 1,
		   page_info->size);

	seq_buf_printf_ion_flags(s, page_info->flags);

	seq_buf_printf(s, "\n");
}

void seq_buf_printf_ion_flags(struct seq_buf *s, unsigned long flags)
{
	if (flags & ION_FLAG_CACHED)
		seq_buf_printf(s, " CACHED");
	if (flags & ION_FLAG_CACHED_NEEDS_SYNC)
		seq_buf_printf(s, " CACHED_NEEDS_SYNC");
	if (flags & ION_FLAG_NONCACHED)
		seq_buf_printf(s, " NONCACHED");
	if (flags & ION_FLAG_SCPUACC)
		seq_buf_printf(s, " SCPUACC");
	if (flags & ION_FLAG_ACPUACC)
		seq_buf_printf(s, " ACPUACC");
	if (flags & ION_FLAG_HWIPACC)
		seq_buf_printf(s, " HWIPACC");
	if (flags & ION_FLAG_VE_SPEC)
		seq_buf_printf(s, " VE");
	if (flags & ION_FLAG_PROTECTED_MASK)
		seq_buf_printf(s, " PROTECTED(%d)",
			       ION_PROTECTED_TYPE_GET(flags));
	if (flags & ION_FLAG_VCPU_FWACC)
		seq_buf_printf(s, " VFWACC");
	if (flags & ION_FLAG_CMA)
		seq_buf_printf(s, " CMA");
	if (flags & ION_USAGE_PROTECTED)
		seq_buf_printf(s, " USAGE_PROTECTED");
	if (flags & ION_USAGE_MMAP_NONCACHED)
		seq_buf_printf(s, " USAGE_MMAP_NONCACHED");
	if (flags & ION_USAGE_MMAP_CACHED)
		seq_buf_printf(s, " USAGE_MMAP_CACHED");
	if (flags & ION_USAGE_MMAP_WRITECOMBINE)
		seq_buf_printf(s, " USAGE_MMAP_WRITECOMBINE");
	if (flags & ION_USAGE_ALGO_LAST_FIT)
		seq_buf_printf(s, " USAGE_ALGO_LAST_FIT");
	if (flags & ION_FLAG_PROTECTED_EXT_MASK)
		seq_buf_printf(s, " PROTECTED_EXT(%d)",
				ION_PROTECTED_EXT_GET(flags));
}

void pool_common_debug_show(struct pool_info *pool_info,
			    struct seq_buf *s, char *prefix)
{
	if (!prefix)
		prefix = "";

	if (!s || !pool_info)
		return;

	seq_buf_printf(s,
		   "%s[%s] base=0x%08lx size=%zu(0x%zx) rtk_flags=%lx\t",
		   prefix,
		   (pool_info->type == GEN_POOL) ? "GEN" :
		   (pool_info->type == CMA_POOL) ? "CMA" :
		   (pool_info->type == PREALLOC_POOL) ? "PREALLOC" :
		   "ERR",
		   pool_info->base, pool_info->size, pool_info->size,
		   pool_info->rtk_flags);

	if (pool_info->rtk_flags & RTK_FLAG_SCPUACC)
		seq_buf_printf(s, " SCPUACC");
	if (pool_info->rtk_flags & RTK_FLAG_ACPUACC)
		seq_buf_printf(s, " ACPUACC");
	if (pool_info->rtk_flags & RTK_FLAG_HWIPACC)
		seq_buf_printf(s, " HWIPACC");
	if (pool_info->rtk_flags & RTK_FLAG_VE_SPEC)
		seq_buf_printf(s, " VE");
	if (pool_info->rtk_flags & RTK_FLAG_PROTECTED_MASK)
		seq_buf_printf(s, " PROTECTED(%d)",
			   RTK_PROTECTED_TYPE_GET(pool_info->rtk_flags));
	if (pool_info->rtk_flags & RTK_FLAG_PROTECTED_DYNAMIC)
		seq_buf_printf(s, " PROTECTED_DYNAMIC");
	if (pool_info->rtk_flags & RTK_FLAG_VCPU_FWACC)
		seq_buf_printf(s, " VFWACC");
	if (pool_info->rtk_flags & RTK_FLAG_CMA)
		seq_buf_printf(s, " CMA");
	if (pool_info->rtk_flags & RTK_FLAG_PROTECTED_EXT_MASK)
		seq_buf_printf(s, " PROTECTED_EXT(%d)",
				RTK_PROTECTED_EXT_GET(pool_info->rtk_flags));
	seq_buf_printf(s, "\n");
}
