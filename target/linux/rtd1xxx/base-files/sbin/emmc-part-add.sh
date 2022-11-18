#!/bin/sh

[ ! -b /dev/mmcblk0 ] && echo "make sure you have eMMC installed" && exit 1

if [ ! -b /dev/mmcblk0p3 ]; then
/sbin/parted /dev/mmcblk0 --script mkpart primary ext4 1000MB 7000MB
/usr/sbin/mkfs.ext4 -E lazy_itable_init=0,lazy_journal_init=0 /dev/mmcblk0p3
else
echo "/dev/mmcblk0p3 exist, fail to create"
fi
