/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include <linux/ion.h>

#ifndef _RTK_ION_PRIVATE_H
#define _RTK_ION_PRIVATE_H

struct ion_platform_heap {
	enum ion_heap_type type;
	unsigned int id;
	const char *name;
	phys_addr_t base;
	size_t size;
	phys_addr_t align;
	void *priv;
};

extern int ion_rtk_protected_notifier_register(struct notifier_block *nb);

#endif
