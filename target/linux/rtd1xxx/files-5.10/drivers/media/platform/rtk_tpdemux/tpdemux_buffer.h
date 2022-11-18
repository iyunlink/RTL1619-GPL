/*
 * tpdemux_buffer.h - Realtek TP demux driver
 *
 * Copyright (C) 2020 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _TPDEMUX_BUFFER_H_
#define _TPDEMUX_BUFFER_H_

int rtk_tp_set_mmbuffer(struct demux_info *dmx);
size_t rtk_tp_write_mmbuffer(struct demux_info *dmx, const void __user *buf,
			     size_t count);

#endif /* _TPDEMUX_BUFFER_H_ */
