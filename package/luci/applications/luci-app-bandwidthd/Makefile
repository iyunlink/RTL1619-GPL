#
# Copyright (C) 2006-2022 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

LUCI_TITLE:=LuCI support for bandwidthd
LUCI_DESCRIPTION:=LuCI support for Setting Bandwidthd
LUCI_DEPENDS:=+bandwidthd
LUCI_PKGARCH:=all

PKG_NAME:=luci-app-bandwidthd
PKG_VERSION:=1.0.0
PKG_RELEASE:=2

PKG_MAINTAINER:=Alex Zhuo <1886090@gmail.com>

include $(TOPDIR)/feeds/luci/luci.mk

# call BuildPackage - OpenWrt buildroot signature
