/* SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause) */
/*
 * Realtek DHC SoC family ION memory flag
 *
 * Copyright (c) 2021 Realtek Semiconductor Corp.
 */

#ifndef _DT_BINDINGS_REALTEK_MEM_H
#define _DT_BINDINGS_REALTEK_MEM_H

#define RTK_FLAG_SCPUACC (1U << 1)
#define RTK_FLAG_ACPUACC (1U << 2)
#define RTK_FLAG_HWIPACC (1U << 3)
#define RTK_FLAG_VE_SPEC (1U << 4)
#define RTK_FLAG_PROTECTED_BIT0 (1U << 5)
#define RTK_FLAG_PROTECTED_BIT1 (1U << 6)
#define RTK_FLAG_VCPU_FWACC (1U << 7) /* Less than 512MB */
#define RTK_FLAG_CMA (1U << 8)
#define RTK_FLAG_PROTECTED_DYNAMIC (1U << 9)
#define RTK_FLAG_PROTECTED_BIT2 (1U << 10)
#define RTK_FLAG_PROTECTED_BIT3 (1U << 11)
#define RTK_FLAG_PROTECTED_MASK                                                \
	(RTK_FLAG_PROTECTED_BIT0 | RTK_FLAG_PROTECTED_BIT1 |                   \
	 RTK_FLAG_PROTECTED_BIT2 | RTK_FLAG_PROTECTED_BIT3)

#define RTK_FLAG_PROTECTED_EXT_BIT0 (1U << 12)
#define RTK_FLAG_PROTECTED_EXT_BIT1 (1U << 13)
#define RTK_FLAG_PROTECTED_EXT_BIT2 (1U << 14)
#define RTK_FLAG_PROTECTED_EXT_MASK                                            \
	(RTK_FLAG_PROTECTED_EXT_BIT0 | RTK_FLAG_PROTECTED_EXT_BIT1 |           \
	 RTK_FLAG_PROTECTED_EXT_BIT2)

#define RTK_PROTECTED_TYPE_NONE (0)
#define RTK_PROTECTED_TYPE_1 (1)
#define RTK_PROTECTED_TYPE_2 (2)
#define RTK_PROTECTED_TYPE_3 (3)
#define RTK_PROTECTED_TYPE_4 (4)
#define RTK_PROTECTED_TYPE_5 (5)
#define RTK_PROTECTED_TYPE_6 (6)
#define RTK_PROTECTED_TYPE_7 (7)
#define RTK_PROTECTED_TYPE_8 (8)
#define RTK_PROTECTED_TYPE_9 (9)
#define RTK_PROTECTED_TYPE_10 (10)
#define RTK_PROTECTED_TYPE_11 (11)
#define RTK_PROTECTED_TYPE_12 (12)
#define RTK_PROTECTED_TYPE_13 (13)
#define RTK_PROTECTED_TYPE_14 (14)
#define RTK_PROTECTED_TYPE_15 (15)

#define RTK_PROTECTED_EXT_NONE (0)
#define RTK_PROTECTED_EXT_1 (1)
#define RTK_PROTECTED_EXT_2 (2)
#define RTK_PROTECTED_EXT_3 (3)
#define RTK_PROTECTED_EXT_4 (4)
#define RTK_PROTECTED_EXT_5 (5)
#define RTK_PROTECTED_EXT_6 (6)
#define RTK_PROTECTED_EXT_7 (7)

#define RTK_PROTECTED_MEM_LIMIT (0x80000000) // 2GB

#define RTK_FLAG_PROTECTED_BITS(TYPE)                                          \
	(((TYPE & (0x1 << 0)) ? RTK_FLAG_PROTECTED_BIT0 : 0) |                 \
	 ((TYPE & (0x1 << 1)) ? RTK_FLAG_PROTECTED_BIT1 : 0) |                 \
	 ((TYPE & (0x1 << 2)) ? RTK_FLAG_PROTECTED_BIT2 : 0) |                 \
	 ((TYPE & (0x1 << 3)) ? RTK_FLAG_PROTECTED_BIT3 : 0))

#define RTK_PROTECTED_TYPE_GET(flags)                                          \
	(((flags & RTK_FLAG_PROTECTED_BIT0) ? (0x1 << 0) : 0) |                \
	 ((flags & RTK_FLAG_PROTECTED_BIT1) ? (0x1 << 1) : 0) |                \
	 ((flags & RTK_FLAG_PROTECTED_BIT2) ? (0x1 << 2) : 0) |                \
	 ((flags & RTK_FLAG_PROTECTED_BIT3) ? (0x1 << 3) : 0))

#define RTK_FLAG_PROTECTED_EXT_BITS(ext)                                       \
	(((ext & (0x1 << 0)) ? RTK_FLAG_PROTECTED_EXT_BIT0 : 0) |              \
	 ((ext & (0x1 << 1)) ? RTK_FLAG_PROTECTED_EXT_BIT1 : 0) |              \
	 ((ext & (0x1 << 2)) ? RTK_FLAG_PROTECTED_EXT_BIT2 : 0))

#define RTK_PROTECTED_EXT_GET(flags)                                           \
	(((flags & RTK_FLAG_PROTECTED_EXT_BIT0) ? (0x1 << 0) : 0) |            \
	 ((flags & RTK_FLAG_PROTECTED_EXT_BIT1) ? (0x1 << 1) : 0) |            \
	 ((flags & RTK_FLAG_PROTECTED_EXT_BIT2) ? (0x1 << 2) : 0))

#define RTK_FLAG_PROTECTED_V2_AUDIO_POOL                                       \
	(RTK_FLAG_PROTECTED_BITS(RTK_PROTECTED_TYPE_1))
#define RTK_FLAG_PROTECTED_V2_TP_POOL                                          \
	(RTK_FLAG_PROTECTED_BITS(RTK_PROTECTED_TYPE_2))
#define RTK_FLAG_PROTECTED_V2_VO_POOL                                          \
	(RTK_FLAG_PROTECTED_BITS(RTK_PROTECTED_TYPE_3))
#define RTK_FLAG_PROTECTED_V2_VIDEO_POOL                                       \
	(RTK_FLAG_PROTECTED_BITS(RTK_PROTECTED_TYPE_5))
#define RTK_FLAG_PROTECTED_V2_AO_POOL                                          \
	(RTK_FLAG_PROTECTED_BITS(RTK_PROTECTED_TYPE_6))
#define RTK_FLAG_PROTECTED_V2_METADATA_POOL                                    \
	(RTK_FLAG_PROTECTED_BITS(RTK_PROTECTED_TYPE_7))
#define RTK_FLAG_PROTECTED_V2_OTA_POOL                                         \
	(RTK_FLAG_PROTECTED_BITS(RTK_PROTECTED_TYPE_8))
#define RTK_FLAG_PROTECTED_V2_FW_STACK                                         \
	(RTK_FLAG_PROTECTED_BITS(RTK_PROTECTED_TYPE_4))

#define RTK_FLAG_PROTECTED_IO (RTK_FLAG_PROTECTED_V2_AUDIO_POOL)
#define RTK_FLAG_PROTECTED_TPACC (RTK_FLAG_PROTECTED_V2_TP_POOL)
#define RTK_FLAG_PROTECTED_AFWIO (RTK_FLAG_PROTECTED_V2_VO_POOL)
#define RTK_FLAG_PROTECTED_VIDEO (RTK_FLAG_PROTECTED_V2_VIDEO_POOL)
#define RTK_FLAG_PROTECTED_DYNAMIC_METADATA                                    \
	(RTK_FLAG_PROTECTED_V2_METADATA_POOL | RTK_FLAG_HWIPACC |              \
	 RTK_FLAG_ACPUACC | RTK_FLAG_VCPU_FWACC)
#define RTK_FLAG_PROTECTED_DYNAMIC_VO                                          \
	(RTK_FLAG_PROTECTED_DYNAMIC | RTK_FLAG_PROTECTED_V2_VO_POOL)
#define RTK_FLAG_PROTECTED_DYNAMIC_OTA                                         \
	(RTK_FLAG_PROTECTED_DYNAMIC | RTK_FLAG_PROTECTED_V2_OTA_POOL)
#define RTK_FLAG_PROTECTED_DYNAMIC_TP                                          \
	(RTK_FLAG_PROTECTED_DYNAMIC | RTK_FLAG_PROTECTED_V2_TP_POOL)

#define RTK_FLAG_CMA_POOL (RTK_FLAG_SCPUACC | RTK_FLAG_HWIPACC | RTK_FLAG_CMA)
#define RTK_FLAG_ION_HEAP                                               \
	(RTK_FLAG_SCPUACC | RTK_FLAG_ACPUACC | RTK_FLAG_VCPU_FWACC |           \
	 RTK_FLAG_HWIPACC)

#define RTK_FLAG_NONCACHED (1U << 0) /* legacy define */

#define RPC_RINGBUF_PHYS (0x040ff000)
#define RPC_RINGBUF_SIZE (0x00004000)

#endif /* _DT_BINDINGS_REALTEK_MEM_H */
