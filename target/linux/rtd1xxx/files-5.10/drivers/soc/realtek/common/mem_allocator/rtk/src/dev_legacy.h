/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef _LINUX_ION_RTK_DEV_LEGACY_H
#define _LINUX_ION_RTK_DEV_LEGACY_H

#include <linux/ioctl.h>
#include <linux/types.h>

typedef int legacy_ion_user_handle_t;

enum legacy_ion_heap_type {
	LEGACY_ION_HEAP_TYPE_SYSTEM,
	LEGACY_ION_HEAP_TYPE_SYSTEM_CONTIG,
	LEGACY_ION_HEAP_TYPE_CARVEOUT,
	LEGACY_ION_HEAP_TYPE_CHUNK,
	LEGACY_ION_HEAP_TYPE_DMA,
	LEGACY_ION_HEAP_TYPE_CUSTOM,	/* must be last so device specific heaps always
					   are at the end of this enum */
	LEGACY_ION_NUM_HEAPS = 16,
};

#define LEGACY_ION_HEAP_SYSTEM_MASK			(1 << LEGACY_ION_HEAP_TYPE_SYSTEM)
#define LEGACY_ION_HEAP_SYSTEM_CONTIG_MASK	(1 << LEGACY_ION_HEAP_TYPE_SYSTEM_CONTIG)
#define LEGACY_ION_HEAP_CARVEOUT_MASK		(1 << LEGACY_ION_HEAP_TYPE_CARVEOUT)
#define LEGACY_ION_HEAP_TYPE_DMA_MASK		(1 << LEGACY_ION_HEAP_TYPE_DMA)

/**
 * allocation flags - the lower 16 bits are used by core ion, the upper 16
 * bits are reserved for use by the heaps themselves.
 */
#define LEGACY_ION_FLAG_CACHED 1	/* mappings of this buffer should be
					   cached, ion will do cache
					   maintenance when the buffer is
					   mapped for dma */
#define LEGACY_ION_FLAG_CACHED_NEEDS_SYNC 2	/* mappings of this buffer will created
						   at mmap time, if this is set
						   caches must be managed manually */

/**
 * DOC: Ion Userspace API
 *
 * create a client by opening /dev/ion
 * most operations handled via following ioctls
 *
 */

/**
 * struct legacy_ion_allocation_data - metadata passed from userspace for allocations
 * @len:		size of the allocation
 * @align:		required alignment of the allocation
 * @heap_id_mask:	mask of heap ids to allocate from
 * @flags:		flags passed to heap
 * @handle:		pointer that will be populated with a cookie to use to 
 *			refer to this allocation
 *
 * Provided by userspace as an argument to the ioctl
 */
struct legacy_ion_allocation_data {
	size_t len;
	size_t align;
	unsigned int heap_id_mask;
	unsigned int flags;
	legacy_ion_user_handle_t handle;
};

/**
 * struct legacy_ion_fd_data - metadata passed to/from userspace for a handle/fd pair
 * @handle:	a handle
 * @fd:		a file descriptor representing that handle
 *
 * For LEGACY_ION_IOC_SHARE or LEGACY_ION_IOC_MAP userspace populates the handle field with
 * the handle returned from ion alloc, and the kernel returns the file
 * descriptor to share or map in the fd field.  For LEGACY_ION_IOC_IMPORT, userspace
 * provides the file descriptor and the kernel returns the handle.
 */
struct legacy_ion_fd_data {
	legacy_ion_user_handle_t handle;
	int fd;
};

/**
 * struct legacy_ion_handle_data - a handle passed to/from the kernel
 * @handle:	a handle
 */
struct legacy_ion_handle_data {
	legacy_ion_user_handle_t handle;
};

/**
 * struct legacy_ion_custom_data - metadata passed to/from userspace for a custom ioctl
 * @cmd:	the custom ioctl function to call
 * @arg:	additional data to pass to the custom ioctl, typically a user
 *		pointer to a predefined structure
 *
 * This works just like the regular cmd and arg fields of an ioctl.
 */
struct legacy_ion_custom_data {
	unsigned int cmd;
	unsigned long arg;
};

/**
 * struct legacy_ion_phys_data - metadata passed to/from userspace for a handle pair
 * @handle:	a handle
 * @addr:	the physical address
 * @len:	the allocate size
 *
 * For LEGACY_ION_IOC_PHYS userspace populates the handle field with
 * the handle returned from ion alloc, and the kernel returns the specific info of the handle.
 */
struct legacy_ion_phys_data {
	legacy_ion_user_handle_t handle;
	unsigned long addr;
	unsigned int len;
};

#define LEGACY_ION_IOC_MAGIC		'I'

/**
 * DOC: LEGACY_ION_IOC_ALLOC - allocate memory
 *
 * Takes an legacy_ion_allocation_data struct and returns it with the handle field
 * populated with the opaque handle for the allocation.
 */
#define LEGACY_ION_IOC_ALLOC		_IOWR(LEGACY_ION_IOC_MAGIC, 0, \
				      struct legacy_ion_allocation_data)

/**
 * DOC: LEGACY_ION_IOC_FREE - free memory
 *
 * Takes an legacy_ion_handle_data struct and frees the handle.
 */
#define LEGACY_ION_IOC_FREE		_IOWR(LEGACY_ION_IOC_MAGIC, 1, struct legacy_ion_handle_data)

/**
 * DOC: LEGACY_ION_IOC_MAP - get a file descriptor to mmap
 *
 * Takes an legacy_ion_fd_data struct with the handle field populated with a valid
 * opaque handle.  Returns the struct with the fd field set to a file
 * descriptor open in the current address space.  This file descriptor
 * can then be used as an argument to mmap.
 */
#define LEGACY_ION_IOC_MAP		_IOWR(LEGACY_ION_IOC_MAGIC, 2, struct legacy_ion_fd_data)

/**
 * DOC: LEGACY_ION_IOC_SHARE - creates a file descriptor to use to share an allocation
 *
 * Takes an legacy_ion_fd_data struct with the handle field populated with a valid
 * opaque handle.  Returns the struct with the fd field set to a file
 * descriptor open in the current address space.  This file descriptor
 * can then be passed to another process.  The corresponding opaque handle can
 * be retrieved via LEGACY_ION_IOC_IMPORT.
 */
#define LEGACY_ION_IOC_SHARE		_IOWR(LEGACY_ION_IOC_MAGIC, 4, struct legacy_ion_fd_data)

/**
 * DOC: LEGACY_ION_IOC_IMPORT - imports a shared file descriptor
 *
 * Takes an legacy_ion_fd_data struct with the fd field populated with a valid file
 * descriptor obtained from LEGACY_ION_IOC_SHARE and returns the struct with the handle
 * filed set to the corresponding opaque handle.
 */
#define LEGACY_ION_IOC_IMPORT		_IOWR(LEGACY_ION_IOC_MAGIC, 5, struct legacy_ion_fd_data)

/**
 * DOC: LEGACY_ION_IOC_SYNC - syncs a shared file descriptors to memory
 *
 * Deprecated in favor of using the dma_buf api's correctly (syncing
 * will happend automatically when the buffer is mapped to a device).
 * If necessary should be used after touching a cached buffer from the cpu,
 * this will make the buffer in memory coherent.
 */
#define LEGACY_ION_IOC_SYNC		_IOWR(LEGACY_ION_IOC_MAGIC, 7, struct legacy_ion_fd_data)

/**
 * DOC: LEGACY_ION_IOC_CUSTOM - call architecture specific ion ioctl
 *
 * Takes the argument of the architecture specific ioctl to call and
 * passes appropriate userdata for that ioctl
 */
#define LEGACY_ION_IOC_CUSTOM		_IOWR(LEGACY_ION_IOC_MAGIC, 6, struct legacy_ion_custom_data)

/**
 * DOC: LEGACY_ION_IOC_PHYS - query specific information of the ion handle
 *
 * Takes an legacy_ion_phys_data struct with the handle field populated with a valid
 * opaque handle.  Returns the struct with the phyAddr and len.
 */
#define LEGACY_ION_IOC_PHYS		_IOWR(LEGACY_ION_IOC_MAGIC, 8, struct legacy_ion_phys_data)

/*******************************************/

struct legacy_rtk_ion_ioc_sync_rane {
	legacy_ion_user_handle_t handle;
	unsigned long long phyAddr;
	unsigned int len;
};

struct legacy_rtk_ion_ioc_phy_info {
	legacy_ion_user_handle_t handle;
	unsigned long long addr;
	unsigned long long len;
};

enum {
	LEGACY_RTK_PHOENIX_ION_TILER_ALLOC,
	LEGACY_RTK_PHOENIX_ION_GET_LAST_ALLOC_ADDR,
	LEGACY_RTK_ION_IOC_INVALIDATE = 0x10,
	LEGACY_RTK_ION_IOC_FLUSH,
	LEGACY_RTK_ION_IOC_INVALIDATE_RANGE = 0x13,
	LEGACY_RTK_ION_IOC_FLUSH_RANGE,
	LEGACY_RTK_ION_IOC_GET_PHYINFO,
};

#endif /* End of _LINUX_ION_RTK_DEV_LEGACY_H */
