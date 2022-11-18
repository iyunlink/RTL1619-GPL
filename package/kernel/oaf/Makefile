include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=oaf

PKG_SOURCE_URL:=https://github.com/destan19/OpenAppFilter
PKG_SOURCE_BRANCH:=master
PKG_SOURCE_PROTO:=git
PKG_SOURCE_BRANCH:=master
PKG_SOURCE_VERSION:=7d4b72f
PKG_SOURCE:=$(PKG_NAME)-$(PKG_SOURCE_VERSION).tar.xz
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_SOURCE_VERSION)
PKG_HASH=f87eb98973b1280cdd621d12deff94331a65bbe0508cc6af3f19b0a67f32eb9a

RSTRIP:=:

PKG_MAINTAINER:=derry <destan19@126.com>

include $(INCLUDE_DIR)/package.mk

define KernelPackage/oaf
  TITLE:=Open App Filter Kernel Module
  SECTION:=kernel
  SUBMENU:=Network Support
  DEPENDS:=+kmod-ipt-conntrack
  FILES:=$(PKG_BUILD_DIR)/oaf/src/oaf.ko
  AUTOLOAD:=$(call AutoLoad,0,oaf)
endef

define KernelPackage/oaf/description
  Open Appfilter Kernel Module
endef

include $(INCLUDE_DIR)/kernel-defaults.mk

define Build/Compile
	$(KERNEL_MAKE) M="$(PKG_BUILD_DIR)/oaf/src" \
		MODFLAGS="-DMODULE -mlong-calls" \
		modules
endef

$(eval $(call KernelPackage,oaf))
