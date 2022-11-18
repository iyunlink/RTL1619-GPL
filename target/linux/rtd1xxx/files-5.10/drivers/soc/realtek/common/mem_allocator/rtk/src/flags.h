/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef _LINUX_ION_RTK_FLAGS_H
#define _LINUX_ION_RTK_FLAGS_H

#include <soc/realtek/memory.h>
#include <soc/realtek/uapi/ion_rtk.h>
#include "../inc/ion_rtk_protected_notifier.h"

static struct protected_mapping_slot {
	int rtk_type;
	enum E_ION_PROTECTED_TYPE ion_type;
	enum E_ION_NOTIFIER_PROTECTED_TYPE notifier_type;
} protected_mapping_table[] = {
	{
	RTK_PROTECTED_TYPE_1, ION_PROTECTED_TYPE_1,
		    ION_NOTIFIER_PROTECTED_TYPE_1}
	, {
	RTK_PROTECTED_TYPE_2, ION_PROTECTED_TYPE_2,
		    ION_NOTIFIER_PROTECTED_TYPE_2}
	, {
	RTK_PROTECTED_TYPE_3, ION_PROTECTED_TYPE_3,
		    ION_NOTIFIER_PROTECTED_TYPE_3}
	, {
	RTK_PROTECTED_TYPE_4, ION_PROTECTED_TYPE_4,
		    ION_NOTIFIER_PROTECTED_TYPE_4}
	, {
	RTK_PROTECTED_TYPE_5, ION_PROTECTED_TYPE_5,
		    ION_NOTIFIER_PROTECTED_TYPE_5}
	, {
	RTK_PROTECTED_TYPE_6, ION_PROTECTED_TYPE_6,
		    ION_NOTIFIER_PROTECTED_TYPE_6}
	, {
	RTK_PROTECTED_TYPE_7, ION_PROTECTED_TYPE_7,
		    ION_NOTIFIER_PROTECTED_TYPE_7}
	, {
	RTK_PROTECTED_TYPE_8, ION_PROTECTED_TYPE_8,
		    ION_NOTIFIER_PROTECTED_TYPE_8}
	, {
	RTK_PROTECTED_TYPE_9, ION_PROTECTED_TYPE_9,
		    ION_NOTIFIER_PROTECTED_TYPE_9}
	, {
	RTK_PROTECTED_TYPE_10, ION_PROTECTED_TYPE_10,
		    ION_NOTIFIER_PROTECTED_TYPE_10}
	, {
	RTK_PROTECTED_TYPE_11, ION_PROTECTED_TYPE_11,
		    ION_NOTIFIER_PROTECTED_TYPE_11}
	, {
	RTK_PROTECTED_TYPE_12, ION_PROTECTED_TYPE_12,
		    ION_NOTIFIER_PROTECTED_TYPE_12}
	, {
	RTK_PROTECTED_TYPE_13, ION_PROTECTED_TYPE_13,
		    ION_NOTIFIER_PROTECTED_TYPE_13}
	, {
	RTK_PROTECTED_TYPE_14, ION_PROTECTED_TYPE_14,
		    ION_NOTIFIER_PROTECTED_TYPE_14}
	, {
	RTK_PROTECTED_TYPE_15, ION_PROTECTED_TYPE_15,
		    ION_NOTIFIER_PROTECTED_TYPE_15}
};

static inline enum E_ION_NOTIFIER_PROTECTED_TYPE
ion_flags_to_notifier_protected_type(unsigned long flags)
{
	enum E_ION_NOTIFIER_PROTECTED_TYPE ret =
	    ION_NOTIFIER_PROTECTED_TYPE_NONE;
	size_t i = 0;
	for (i = 0; i < ARRAY_SIZE(protected_mapping_table); i++) {
		if (protected_mapping_table[i].ion_type ==
		    ION_PROTECTED_TYPE_GET(flags)) {
			ret = protected_mapping_table[i].notifier_type;
			break;
		}
	}
	return ret;
}

static inline enum E_ION_PROTECTED_TYPE rtk_flags_to_ion_protected_type(unsigned
									long
									flags)
{
	enum E_ION_PROTECTED_TYPE ret = ION_PROTECTED_TYPE_NONE;
	size_t i = 0;
	for (i = 0; i < ARRAY_SIZE(protected_mapping_table); i++) {
		if (protected_mapping_table[i].rtk_type ==
		    RTK_PROTECTED_TYPE_GET(flags)) {
			ret = protected_mapping_table[i].ion_type;
			break;
		}
	}
	return ret;
}

static inline enum E_ION_NOTIFIER_PROTECTED_TYPE
rtk_flags_to_notifier_protected_type(unsigned long flags)
{
	enum E_ION_NOTIFIER_PROTECTED_TYPE ret =
	    ION_NOTIFIER_PROTECTED_TYPE_NONE;
	size_t i = 0;
	for (i = 0; i < ARRAY_SIZE(protected_mapping_table); i++) {
		if (protected_mapping_table[i].rtk_type ==
		    RTK_PROTECTED_TYPE_GET(flags)) {
			ret = protected_mapping_table[i].notifier_type;
			break;
		}
	}
	return ret;
}

static inline bool rtk_flag_is_protected_dynamic(unsigned long flags)
{
	return ((flags & RTK_FLAG_PROTECTED_DYNAMIC) &&
		(RTK_PROTECTED_TYPE_GET(flags) !=
		 RTK_PROTECTED_TYPE_NONE)) ? true : false;
}

static inline bool rtk_flag_is_protected_static(unsigned long flags)
{
	return (!(flags & RTK_FLAG_PROTECTED_DYNAMIC) &&
		(RTK_PROTECTED_TYPE_GET(flags) !=
		 RTK_PROTECTED_TYPE_NONE)) ? true : false;
}

static inline bool rtk_flag_is_protected(unsigned long flags)
{
	return (flags & RTK_FLAG_PROTECTED_MASK) ? true : false;
}

static inline bool rtk_flag_has_protected_ext(unsigned long flags)
{
	return (flags & RTK_FLAG_PROTECTED_EXT_MASK) ? true : false;
}

static inline bool rtk_flag_canAccess(unsigned long flags)
{
	bool access = false;
	do {
		if (!(flags & RTK_FLAG_SCPUACC))
			break;

		if (!rtk_flag_is_protected_dynamic(flags)) {
			if (RTK_PROTECTED_TYPE_GET(flags) !=
			    RTK_PROTECTED_TYPE_NONE)
				break;
		}
		access = true;
		break;
	} while (0);
	return access;
}

static inline bool ion_flag_is_protected(unsigned long flags)
{
	bool ret = true;
	do {
		if (flags & ION_FLAG_PROTECTED_MASK)
			break;
		ret = false;
		break;
	} while (0);
	return ret;
}

static inline bool ion_flag_is_noncached(unsigned long flags)
{
	bool ret = true;
	do {
		if (flags & ION_FLAG_NONCACHED)
			break;
		ret = false;
		break;
	} while (0);
	return ret;
}

static inline bool ion_flag_canAccess(unsigned long flags)
{
	bool access = false;
	do {
		if (!(flags & ION_FLAG_SCPUACC))
			break;

		if (ion_flag_is_protected(flags))
			break;

		access = true;
		break;
	} while (0);
	return access;
}

static inline unsigned long ion_flag_pool_condition(unsigned long flags)
{
	return flags & RTK_ION_FLAG_POOL_CONDITION;
}

static inline unsigned long ion_flag_condition_from_rtk(unsigned long rtk_flags)
{
	unsigned long flags = rtk_flags;
	unsigned long out = 0;
	//out |= (flags & RTK_FLAG_NONCACHED)         ? ION_FLAG_NONCACHED      : 0;
	out |= (flags & RTK_FLAG_SCPUACC) ? ION_FLAG_SCPUACC : 0;
	out |= (flags & RTK_FLAG_ACPUACC) ? ION_FLAG_ACPUACC : 0;
	out |= (flags & RTK_FLAG_HWIPACC) ? ION_FLAG_HWIPACC : 0;
	out |= (flags & RTK_FLAG_VE_SPEC) ? ION_FLAG_VE_SPEC : 0;
	out |= (flags & RTK_FLAG_VCPU_FWACC) ? ION_FLAG_VCPU_FWACC : 0;
	out |= (flags & RTK_FLAG_CMA) ? ION_FLAG_CMA : 0;
	out |= ION_FLAG_PROTECTED_BITS(rtk_flags_to_ion_protected_type(flags));
	return out;
}

static inline unsigned long ion_flag_mapping_from_rtk(unsigned long rtk_flags)
{
	unsigned long out = ion_flag_condition_from_rtk(rtk_flags);
	if (rtk_flag_has_protected_ext(rtk_flags)) {
		out |= ION_FLAG_PROTECTED_EXT_BITS(RTK_PROTECTED_EXT_GET(rtk_flags));
	}
	return out;
}

static inline unsigned long ion_flag_from_rtk(unsigned long rtk_flags)
{
	unsigned long flags = rtk_flags;
	unsigned long out = 0;
	//out |= (flags & RTK_FLAG_NONCACHED)         ? ION_FLAG_NONCACHED      : 0;
	out |= (flags & RTK_FLAG_SCPUACC) ? ION_FLAG_SCPUACC : 0;
	out |= (flags & RTK_FLAG_ACPUACC) ? ION_FLAG_ACPUACC : 0;
	out |= (flags & RTK_FLAG_HWIPACC) ? ION_FLAG_HWIPACC : 0;
	out |= (flags & RTK_FLAG_VE_SPEC) ? ION_FLAG_VE_SPEC : 0;
	out |= (flags & RTK_FLAG_VCPU_FWACC) ? ION_FLAG_VCPU_FWACC : 0;
	out |= (flags & RTK_FLAG_CMA) ? ION_FLAG_CMA : 0;
	if (!rtk_flag_is_protected_dynamic(flags)) {
		out |=
		    ION_FLAG_PROTECTED_BITS(rtk_flags_to_ion_protected_type
					    (flags));
	}
	return out;
}

static inline bool ion_flag_match_pool_condition(unsigned long flags,
						 unsigned long rtk_flags)
{
	bool ret = false;
	unsigned long target = ion_flag_pool_condition(flags);
	unsigned long condition = ion_flag_condition_from_rtk(rtk_flags);
	do {
		if ((target & condition) != target)
			break;
		if (ion_flag_is_protected(flags)) {
			if (ION_PROTECTED_TYPE_GET(flags) !=
			    rtk_flags_to_ion_protected_type(rtk_flags))
				break;
		} else if (rtk_flag_is_protected_static(rtk_flags)) {
			break;
		}

		if (rtk_flag_has_protected_ext(rtk_flags) &&
				RTK_PROTECTED_EXT_GET(rtk_flags) !=
				ION_PROTECTED_EXT_GET(flags)) {
			break;
		}

		ret = true;
	} while (0);
	return ret;
}

static inline bool ion_flag_mismatch_pool_condition(unsigned long flags,
						    unsigned long rtk_flags)
{
	return !ion_flag_match_pool_condition(flags, rtk_flags);
}

static inline bool ion_flag_match_condition(unsigned long flags_target,
					    unsigned long flags_condition)
{
	unsigned long target = ion_flag_pool_condition(flags_target);
	unsigned long condition = ion_flag_pool_condition(flags_condition);
	return ((target & condition) == target) ? true : false;
}

static inline bool ion_flag_is_algo_last_fit(unsigned long flags)
{
	return (flags & ION_USAGE_ALGO_LAST_FIT) ? true : false;
}

static inline bool ion_flag_has_protected_ext(unsigned long flags)
{
	return (flags & ION_FLAG_PROTECTED_EXT_MASK) ? true : false;
}

static inline enum E_ION_NOTIFIER_PROTECTED_EXT
ion_flag_to_notifier_protected_ext(unsigned long flags)
{
	enum E_ION_NOTIFIER_PROTECTED_EXT ret =
        (enum E_ION_NOTIFIER_PROTECTED_EXT) ION_PROTECTED_EXT_GET(flags);
    return ret;
}
#endif /* _LINUX_ION_RTK_FLAGS_H */
