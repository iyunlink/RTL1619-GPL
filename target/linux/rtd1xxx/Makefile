# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2015-2021 Realtek Semiconductor Corp.

include $(TOPDIR)/rules.mk

ARCH:=aarch64
BOARD:=rtd1xxx
BOARDNAME:=Realtek RTD Media SoC
FEATURES:=squashfs usb pcie
SUBTARGETS:=rtd1619b rtd1319
DEVICE_TYPE:=nas
CPU_TYPE:=cortex-a55
CFLAGS:=-O2 -pipe

KERNEL_PATCHVER:=5.10

define Target/Description
	Build firmware images for Reatek RTD1xxx SoC family for NAS
endef

include $(INCLUDE_DIR)/target.mk

KERNELNAME:=Image dtbs

DEFAULT_PACKAGES +=e2fsprogs ethtool mkf2fs

$(eval $(call BuildTarget))
