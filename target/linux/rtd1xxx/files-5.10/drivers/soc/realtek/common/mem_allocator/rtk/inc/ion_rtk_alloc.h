/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef _LINUX_ION_RTK_ALLOC_H
#define _LINUX_ION_RTK_ALLOC_H

#include <linux/types.h>
#include <linux/fdtable.h>

struct dma_buf *ext_rtk_ion_alloc(size_t len, unsigned int heap_type_mask,
		      unsigned int flags);
int ext_rtk_ion_close_fd(struct files_struct *files, unsigned fd);

#endif /* _LINUX_ION_RTK_ALLOC_H */
