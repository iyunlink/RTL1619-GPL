/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef _LINUX_ION_RTK_CARVEOUT_HEAP_DEBUGFS_H
#define _LINUX_ION_RTK_CARVEOUT_HEAP_DEBUGFS_H

#include <linux/ion.h>

#ifdef CONFIG_DEBUG_FS
void debugfs_add_heap(struct ion_heap *heap);
#else
void debugfs_add_heap(struct ion_heap *heap)
{
}
#endif

extern int ion_carveout_heap_debug_show(struct ion_heap *heap,
					struct seq_file *s, void *prefix);

#endif /* _LINUX_ION_RTK_CARVEOUT_HEAP_DEBUGFS_H */
