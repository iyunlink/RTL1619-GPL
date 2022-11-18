#
# Copyright (C) 2016 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=rtk-bluecore
PKG_RELEASE:=1
PKG_SOURCE_VERSION:=HEAD
PKG_HASH:=skip
PKG_SOURCE_SUBDIR:=$(PKG_NAME)
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_SOURCE_SUBDIR)
PKG_SOURCE:=$(PKG_NAME).tar
PKG_PREPARED_DEPENDS:=CONFIG_UPDATE_AUDIO_FW

ifeq ($(CONFIG_UPDATE_AUDIO_FW),y)
PKG_SOURCE_PROTO:=lcp
PKG_SOURCE_URL:=file://rtk-dl

endif # CONFIG_UPDATE_AUDIO_FW
include $(INCLUDE_DIR)/package.mk
PKG_FW_DIR := $(PKG_BUILD_DIR)

define Package/rtk-bluecore
  SECTION:=multimedia
  CATEGORY:=Multimedia
  TITLE:=Bluecore Libraries
  DEPENDS:=@TARGET_rtd1xxx
  DEFAULT:=y
endef

define Package/rtk-bluecore/description
 Realtek Bluecore libraries
endef

define Package/rtk-bluecore/config
  source "$(SOURCE)/Config.in"
endef

ifeq ($(CONFIG_UPDATE_AUDIO_FW),y)
define Build/Prepare
	$(call Build/Prepare/Default)
	mkdir -p ./files
	$(CP) $(PKG_FW_DIR)/* ./files/
endef

define Build/Clean
	rm -rf ./files
endef
else # CONFIG_UPDATE_AUDIO_FW
define Download/default
endef

define Build/Prepare
endef
endif # CONFIG_UPDATE_AUDIO_FW

define Build/Compile
endef

define Package/rtk-bluecore/install
	#
endef

$(eval $(call BuildPackage,rtk-bluecore))
