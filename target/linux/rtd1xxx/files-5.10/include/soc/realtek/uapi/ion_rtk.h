#ifndef __LINUX_ION_RTK_H_
#define __LINUX_ION_RTK_H_
/* ion_rtk.h
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <uapi/linux/ion.h>

#define ION_FLAG_CACHED_NEEDS_SYNC 2

#define ION_FLAG_NONCACHED (1 << 31)
#define ION_FLAG_SCPUACC (1 << 30)
#define ION_FLAG_ACPUACC (1 << 29)
#define ION_FLAG_HWIPACC (1 << 28)
#define ION_FLAG_VE_SPEC (1 << 27)
#define ION_FLAG_PROTECTED_BIT0         (1 << 26)
#define ION_FLAG_PROTECTED_BIT1         (1 << 25)
#define ION_FLAG_PROTECTED_BIT2         (1 << 24)
#define ION_FLAG_PROTECTED_BIT3         (1 << 16)
#define ION_FLAG_PROTECTED_MASK         (ION_FLAG_PROTECTED_BIT3 | ION_FLAG_PROTECTED_BIT2 | ION_FLAG_PROTECTED_BIT1 | ION_FLAG_PROTECTED_BIT0)

#define ION_FLAG_PROTECTED_EXT_BIT0     (1U << 15)
#define ION_FLAG_PROTECTED_EXT_BIT1     (1U << 14)
#define ION_FLAG_PROTECTED_EXT_BIT2     (1U << 13)
#define ION_FLAG_PROTECTED_EXT_MASK     (ION_FLAG_PROTECTED_EXT_BIT0 | ION_FLAG_PROTECTED_EXT_BIT1 | ION_FLAG_PROTECTED_EXT_BIT2)

enum E_ION_PROTECTED_TYPE {
	ION_PROTECTED_TYPE_NONE = 0,
	ION_PROTECTED_TYPE_1,
	ION_PROTECTED_TYPE_2,
	ION_PROTECTED_TYPE_3,
	ION_PROTECTED_TYPE_4,
	ION_PROTECTED_TYPE_5,
	ION_PROTECTED_TYPE_6,
	ION_PROTECTED_TYPE_7,
	ION_PROTECTED_TYPE_8,
	ION_PROTECTED_TYPE_9,
	ION_PROTECTED_TYPE_10,
	ION_PROTECTED_TYPE_11,
	ION_PROTECTED_TYPE_12,
	ION_PROTECTED_TYPE_13,
	ION_PROTECTED_TYPE_14,
	ION_PROTECTED_TYPE_15,
	ION_PROTECTED_TYPE_MAX,
};

enum E_ION_PROTECTED_EXT {
	ION_PROTECTED_EXT_NONE = 0,
	ION_PROTECTED_EXT_1,
	ION_PROTECTED_EXT_2,
	ION_PROTECTED_EXT_3,
	ION_PROTECTED_EXT_4,
	ION_PROTECTED_EXT_5,
	ION_PROTECTED_EXT_6,
	ION_PROTECTED_EXT_7,
	ION_PROTECTED_EXT_MAX,
};

#define ION_FLAG_PROTECTED_BITS(type) ( \
        ((type & (0x1 << 0)) ? ION_FLAG_PROTECTED_BIT0 : 0) |\
        ((type & (0x1 << 1)) ? ION_FLAG_PROTECTED_BIT1 : 0) |\
        ((type & (0x1 << 2)) ? ION_FLAG_PROTECTED_BIT2 : 0) |\
        ((type & (0x1 << 3)) ? ION_FLAG_PROTECTED_BIT3 : 0))

#define ION_PROTECTED_TYPE_GET(flags) (\
        ((flags & ION_FLAG_PROTECTED_BIT0) ? (0x1 << 0) : 0) |\
        ((flags & ION_FLAG_PROTECTED_BIT1) ? (0x1 << 1) : 0) |\
        ((flags & ION_FLAG_PROTECTED_BIT2) ? (0x1 << 2) : 0) |\
        ((flags & ION_FLAG_PROTECTED_BIT3) ? (0x1 << 3) : 0))

#define ION_FLAG_PROTECTED_EXT_BITS(ext) ( \
		((ext & (0x1 << 0)) ? ION_FLAG_PROTECTED_EXT_BIT0 : 0) |\
		((ext & (0x1 << 1)) ? ION_FLAG_PROTECTED_EXT_BIT1 : 0) |\
		((ext & (0x1 << 2)) ? ION_FLAG_PROTECTED_EXT_BIT2 : 0))

#define ION_PROTECTED_EXT_GET(flags) (\
		((flags & ION_FLAG_PROTECTED_EXT_BIT0) ? (0x1 << 0) : 0) |\
		((flags & ION_FLAG_PROTECTED_EXT_BIT1) ? (0x1 << 1) : 0) |\
		((flags & ION_FLAG_PROTECTED_EXT_BIT2) ? (0x1 << 2) : 0))

#define ION_FLAG_PROTECTED_V2_AUDIO_POOL        (ION_FLAG_PROTECTED_BITS(ION_PROTECTED_TYPE_1))
#define ION_FLAG_PROTECTED_V2_TP_POOL           (ION_FLAG_PROTECTED_BITS(ION_PROTECTED_TYPE_2))
#define ION_FLAG_PROTECTED_V2_VO_POOL           (ION_FLAG_PROTECTED_BITS(ION_PROTECTED_TYPE_3))
#define ION_FLAG_PROTECTED_V2_VIDEO_POOL        (ION_FLAG_PROTECTED_BITS(ION_PROTECTED_TYPE_5))
#define ION_FLAG_PROTECTED_V2_AO_POOL           (ION_FLAG_PROTECTED_BITS(ION_PROTECTED_TYPE_6))
#define ION_FLAG_PROTECTED_V2_METADATA_POOL     (ION_FLAG_PROTECTED_BITS(ION_PROTECTED_TYPE_7))
#define ION_FLAG_PROTECTED_V2_OTA_POOL          (ION_FLAG_PROTECTED_BITS(ION_PROTECTED_TYPE_8))
#define ION_FLAG_PROTECTED_V2_FW_STACK          (ION_FLAG_PROTECTED_BITS(ION_PROTECTED_TYPE_4))

#define ION_FLAG_PROTECTED_IO       (ION_FLAG_PROTECTED_V2_AUDIO_POOL)
#define ION_FLAG_PROTECTED_TPACC    (ION_FLAG_PROTECTED_V2_TP_POOL)
#define ION_FLAG_PROTECTED_AFWIO    (ION_FLAG_PROTECTED_V2_VO_POOL)
#define ION_FLAG_PROTECTED_VIDEO    (ION_FLAG_PROTECTED_V2_VIDEO_POOL)

#define ION_FLAG_PROTECTED          ION_FLAG_PROTECTED_IO /* legacy */
#define ION_FLAG_SECURE_AUDIO       ION_FLAG_PROTECTED_IO /* legacy */

#define ION_FLAG_VCPU_FWACC (1 << 18)
#define ION_FLAG_CMA (1 << 17)
#define RTK_ION_FLAG_POOL_CONDITION            (\
	ION_FLAG_ACPUACC | \
	ION_FLAG_SCPUACC | \
	ION_FLAG_HWIPACC | \
	ION_FLAG_VE_SPEC | \
    ION_FLAG_PROTECTED_MASK | \
	ION_FLAG_VCPU_FWACC | \
	ION_FLAG_CMA)

#define ION_USAGE_PROTECTED (1 << 23)
#define ION_USAGE_MMAP_NONCACHED (1 << 22)
#define ION_USAGE_MMAP_CACHED (1 << 21)
#define ION_USAGE_MMAP_WRITECOMBINE (1 << 20)
#define ION_USAGE_ALGO_LAST_FIT (1 << 19) /* 0:first fit(default), 1:last fit */
#define ION_USAGE_MASK (ION_USAGE_PROTECTED | ION_USAGE_MMAP_NONCACHED | ION_USAGE_MMAP_CACHED | ION_USAGE_MMAP_WRITECOMBINE | ION_USAGE_ALGO_LAST_FIT)

enum {
	RTK_ION_HEAP_TYPE_TILER = ION_HEAP_TYPE_CUSTOM + 1,
	RTK_ION_HEAP_TYPE_MEDIA,
	RTK_ION_HEAP_TYPE_AUDIO,
	RTK_ION_HEAP_TYPE_SECURE,
};

#define RTK_ION_HEAP_TILER_MASK (1 << RTK_ION_HEAP_TYPE_TILER)
#define RTK_ION_HEAP_MEDIA_MASK (1 << RTK_ION_HEAP_TYPE_MEDIA)
#define RTK_ION_HEAP_AUDIO_MASK (1 << RTK_ION_HEAP_TYPE_AUDIO)
#define RTK_ION_HEAP_SECURE_MASK (1 << RTK_ION_HEAP_TYPE_SECURE)

struct rtk_ion_ioc_get_memory_info_s {
	int handle;
	unsigned int heapMask; /* request: select the heap to be queried */
	unsigned int flags; /* request: set the conditions to query, 0 is to query all the conditions */
	unsigned int usedSize; /* response */
	unsigned int freeSize; /* response */
};

struct rtk_ion_ioc_sync_rane {
	int handle;
	u32 phyAddr;
	unsigned int len;
};

struct rtk_ion_ioc_phy_info {
	int handle;
	unsigned long long addr;
	unsigned long long len;
};


#define RTK_ION_IOC_MAGIC 'R'
#define RTK_ION_TILER_ALLOC (0x0)
#define RTK_ION_GET_LAST_ALLOC_ADDR (0x1)
#define RTK_ION_IOC_INVALIDATE (0x10)
#define RTK_ION_IOC_FLUSH (0x11)
#define RTK_ION_IOC_GET_MEMORY_INFO _IOWR(RTK_ION_IOC_MAGIC, 0x12, struct rtk_ion_ioc_get_memory_info_s)
#define RTK_ION_IOC_INVALIDATE_RANGE (0x13)
#define RTK_ION_IOC_FLUSH_RANGE (0x14)
#define RTK_ION_IOC_GET_PHYINFO (0x15)

extern unsigned int retry_count_value;
extern unsigned int retry_delay_value;
#endif /* __LINUX_ION_RTK_H_ */
