/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef __RTK_BUFLOCK_H__
#define __RTK_BUFLOCK_H__

#include <linux/ioctl.h>

enum buflock_status {
    E_BUFLOCK_ST_ERROR,
    E_BUFLOCK_ST_NORMAL,
    E_BUFLOCK_ST_TOUCH,
    E_BUFLOCK_ST_LOCK,
    E_BUFLOCK_ST_UNLOCK,
    E_BUFLOCK_ST_RELEASE,
};

typedef unsigned char   buflock_status_t;
typedef unsigned int    buflock_fwid_t;

struct buflock_alloc_data {
    buflock_fwid_t fwid;
};

struct buflock_import_data {
    buflock_fwid_t fwid;
};

struct buflock_free_data {
    buflock_fwid_t fwid;
};

struct buflock_status_data {
    buflock_fwid_t      fwid;
    buflock_status_t    status;
};

struct buflock_reference_data {
    buflock_fwid_t      fwid;
    unsigned int        count;
};

struct buflock_independent_data {
    buflock_fwid_t      fwid;
};

#define BUFLOCK_IOC_MAGIC       'B'
#define BUFLOCK_IOC_ALLOC       _IOWR(BUFLOCK_IOC_MAGIC, 1, struct buflock_alloc_data)
#define BUFLOCK_IOC_FREE        _IOWR(BUFLOCK_IOC_MAGIC, 2, struct buflock_free_data)
#define BUFLOCK_IOC_IMPORT      _IOWR(BUFLOCK_IOC_MAGIC, 3, struct buflock_import_data)
#define BUFLOCK_IOC_SET_STATUS  _IOWR(BUFLOCK_IOC_MAGIC, 4, struct buflock_status_data)
#define BUFLOCK_IOC_GET_STATUS  _IOWR(BUFLOCK_IOC_MAGIC, 5, struct buflock_status_data)
#define BUFLOCK_IOC_GET_REF     _IOWR(BUFLOCK_IOC_MAGIC, 6, struct buflock_reference_data)
#define BUFLOCK_IOC_SET_INDIE   _IOWR(BUFLOCK_IOC_MAGIC, 7, struct buflock_independent_data)

#endif /* __RTK_BUF_LOCK_H__ */
