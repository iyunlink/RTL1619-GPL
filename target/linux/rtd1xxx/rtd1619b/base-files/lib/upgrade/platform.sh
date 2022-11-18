REQUIRE_IMAGE_METADATA=1

RAMFS_COPY_BIN='echo mdev jffs2mark /bin/mount'

rtk_do_upgrade() {
	tar xf "$1" install_a || return
	./install_a "$1"
}

platform_check_image() {
	return 0
}

platform_do_upgrade() {
	local board=$(board_name)

	case "$board" in
	realtek,bleeding-edge-emmc-router)
		rtk_do_upgrade "$1"
		;;
	*)
		echo "Unsupported device: $board"
		;;
	esac
}

platform_pre_upgrade() {
	local board=$(board_name)

	case "$board" in
	realtek,bleeding-edge-emmc-router)
		[ -z "$UPGRADE_BACKUP" ] && {
			jffs2mark -y
			umount /overlay
		}
		;;
	esac
}

platform_copy_config() {
	local board=$(board_name)

	case "$board" in
	realtek,bleeding-edge-emmc-router)
		#cp -af "$UPGRADE_BACKUP" "/mnt/$BACKUP_FILE"
		;;
	esac
}
