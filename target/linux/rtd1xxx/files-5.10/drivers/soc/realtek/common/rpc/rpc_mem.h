/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef _RTK_RPC_MEM_H
#define _RTK_RPC_MEM_H

#include <linux/types.h>
#include <linux/dma-buf.h>

typedef struct r_program_entry {
	unsigned long phys_addr;
	unsigned long size;
	struct dma_buf *rpc_dmabuf;
	struct sg_table *table;
	struct dma_buf_attachment *attachment;
	struct r_program_entry *next;
} r_program_entry_t;

extern void r_program_add(r_program_entry_t * entry);
extern r_program_entry_t *r_program_remove(unsigned long phys_addr);

#endif /* _RTK_RPC_MEM_H */
