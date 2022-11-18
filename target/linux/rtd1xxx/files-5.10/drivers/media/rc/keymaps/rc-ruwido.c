// SPDX-License-Identifier: GPL-2.0+
// Keytable for ruwido Remote Controller
//
// keymap imported from ir-keymaps.c
//
// Copyright (c) 2020 by Simon Hsu

#include <media/rc-map.h>
#include <linux/module.h>

static struct rc_map_table ruwido[] = {

	{ 0x6516, KEY_POWER },
	{ 0x6519, KEY_UP },
	{ 0x651A, KEY_DOWN },
	{ 0x651D, KEY_LEFT },
	{ 0x6521, KEY_RIGHT },
	{ 0x652A, KEY_VOLUMEDOWN },
	{ 0x6526, KEY_VOLUMEUP },
	{ 0x656D, KEY_HOME },
	{ 0x653A, KEY_BACK },
	{ 0x6536, KEY_PLAY },
	{ 0x651e, KEY_OK },
	{ 0x804F, KEY_REWIND },
	{ 0x8016, KEY_FASTFORWARD },
};

static struct rc_map_list ruwido_map = {
	.map = {
		.scan     = ruwido,
		.size     = ARRAY_SIZE(ruwido),
		.rc_proto = RC_PROTO_OTHER,
		.name     = RC_MAP_RUWIDO,
	}
};

static int __init init_rc_map_ruwido(void)
{
	return rc_map_register(&ruwido_map);
}

static void __exit exit_rc_map_ruwido(void)
{
	rc_map_unregister(&ruwido_map);
}

module_init(init_rc_map_ruwido)
module_exit(exit_rc_map_ruwido)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Hsu <simon_hsu@realtek.com>");
