/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include "debugfs.h"
#include <linux/debugfs.h>
#include <linux/slab.h>
#include "carveout_heap.h"

#define DEBUGFS_NAME "ion_rtk"

#ifdef CONFIG_DEBUG_FS
struct debugfs_device {
	struct dentry *debug_root;
};

static struct debugfs_device *debug_dev = NULL;

static struct debugfs_device *get_debug_dev(void)
{
	do {
		struct debugfs_device *tmp = NULL;
		if (debug_dev)
			break;

		tmp = kzalloc(sizeof(struct debugfs_device), GFP_KERNEL);
		if (!tmp)
			break;

		tmp->debug_root = debugfs_create_dir(DEBUGFS_NAME, NULL);
		if (!tmp->debug_root) {
			kfree(tmp);
			break;
		}

		debug_dev = tmp;
		break;
	} while (0);
	return debug_dev;
}

static int debugfs_heap_show(struct seq_file *s, void *unused)
{
	struct ion_heap *heap = s->private;
	struct ion_rtk_carveout_ops *ops = get_rtk_carveout_ops(heap);

	ion_carveout_heap_debug_show(heap, s, "");

	if (ops && ops->traceDump)
		ops->traceDump(heap, s);

	return 0;
}

static int debugfs_heap_open(struct inode *inode, struct file *file)
{
	return single_open(file, debugfs_heap_show, inode->i_private);
}

static const struct file_operations debug_heap_fops = {
	.open = debugfs_heap_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void debugfs_add_heap(struct ion_heap *heap)
{
	do {
		struct debugfs_device *dev = get_debug_dev();
		struct dentry *debug_file;
		if (!dev)
			break;

		debug_file =
		    debugfs_create_file(heap->name, 0644, dev->debug_root, heap,
					&debug_heap_fops);

	} while (0);
}
EXPORT_SYMBOL_GPL(debugfs_add_heap);
#endif
