// SPDX-License-Identifier: GPL-2.0+
// Keytable for realtek_dhc Remote Controller
//
// keymap imported from ir-keymaps.c
//
// Copyright (c) 2020 by Simon Hsu

#include <media/rc-map.h>
#include <linux/module.h>

static struct rc_map_table realtek_dhc[] = {
	// scancode	keycode OMNI
	{ 0x8018, KEY_POWER },
	{ 0x8032, KEY_TV },
	{ 0x8005, KEY_RED },
	{ 0x8009, KEY_GREEN },
	{ 0x8042, KEY_YELLOW },
	{ 0x8043, KEY_BLUE },
	{ 0x804F, KEY_REWIND },
	{ 0x8055, KEY_PLAY },
	{ 0x8016, KEY_FASTFORWARD },
	{ 0x804E, KEY_PROG1 },
	{ 0x804A, KEY_PROG2 },
	{ 0x804D, KEY_UP },
	{ 0x8048, KEY_DOWN },
	{ 0x800E, KEY_RIGHT },
	{ 0x800C, KEY_LEFT },
	{ 0x804C, KEY_OK },
	{ 0x8057, KEY_BACK },
	{ 0x8014, KEY_HOME },
	{ 0x8066, KEY_APPSELECT },
	{ 0x804B, KEY_VOLUMEDOWN },
	{ 0x8049, KEY_VOLUMEUP },
	{ 0x8074, KEY_ASSISTANT },
	{ 0x800D, KEY_MUTE },
	{ 0x8059, KEY_CHANNELUP },
	{ 0x805F, KEY_CHANNELDOWN },
	{ 0x8021, KEY_1 },
	{ 0x8022, KEY_2 },
	{ 0x8023, KEY_3 },
	{ 0x8024, KEY_4 },
	{ 0x8025, KEY_5 },
	{ 0x8026, KEY_6 },
	{ 0x8027, KEY_7 },
	{ 0x8028, KEY_8 },
	{ 0x8029, KEY_9 },
	{ 0x8020, KEY_0 },
	{ 0x805A, KEY_SELECT },
	{ 0x8030, KEY_LAST },
	{ 0x8058, KEY_CYCLEWINDOWS },
	{ 0x8056, KEY_OPTION },
	{ 0x8054, KEY_INFO },
	{ 0x8017, KEY_STOP },
	{ 0x8008, KEY_PREVIOUS },
};

static struct rc_map_list realtek_dhc_map = {
	.map = {
		.scan     = realtek_dhc,
		.size     = ARRAY_SIZE(realtek_dhc),
		.rc_proto = RC_PROTO_NEC,
		.name     = RC_MAP_REALTEK_DHC,
	}
};

static int __init init_rc_map_realtek_dhc(void)
{
	return rc_map_register(&realtek_dhc_map);
}

static void __exit exit_rc_map_realtek_dhc(void)
{
	rc_map_unregister(&realtek_dhc_map);
}

module_init(init_rc_map_realtek_dhc)
module_exit(exit_rc_map_realtek_dhc)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Hsu <simon_hsu@realtek.com>");
