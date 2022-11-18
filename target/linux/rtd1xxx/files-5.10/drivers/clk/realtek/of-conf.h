/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#ifndef __REALTEK_CLK_OF_CONF_H
#define __REALTEK_CLK_OF_CONF_H

struct device_node;
struct regmap;

void of_rtk_clk_setup_crt(struct device_node *np, struct regmap *regmap);

#endif
