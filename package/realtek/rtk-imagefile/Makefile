#
# Copyright (C) 2016 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=rtk-imagefile
PKG_RELEASE:=3

PKG_SOURCE_PROTO:=lcp
PKG_SOURCE_URL:=file://rtk-dl
PKG_SOURCE_VERSION:=1bb079a
PKG_HASH:=c9cd2e2125098bdd5dfed41405f2674f0f1629596b2e5ad3b170f5fa65e77516
PKG_SOURCE:=$(PKG_NAME)-$(PKG_SOURCE_VERSION).tar.xz

IMG_DST_DIR:=$(TOPDIR)/target/linux/$(BOARD)/image/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

ifeq ($(CONFIG_TARGET_rtd1xxx_rtd1319),y)
  SOC=1319
else ifeq ($(CONFIG_TARGET_rtd1xxx_rtd1619b),y)
  SOC=1619b
endif

define Package/rtk-imagefile
  TITLE:=Image file creator for Realtek RTD1319/RTD1619b platform
  DEPENDS:=@(TARGET_rtd1xxx_rtd1619b||TARGET_rtd1xxx_rtd1319)
endef

define Package/rtk-imagefile/description
 Realtek image file creator for Realtek FW layout partition with fwdesc table.
endef

define Build/Configure
endef

define Build/Prepare
	$(call Build/Prepare/Default)
	$(SED) "s/^\(IMGFILE_VER=\).*/\1g$(PKG_SOURCE_VERSION)/" $(PKG_BUILD_DIR)/Makefile.in
	rm -rf $(IMG_DST_DIR)
	mkdir -p $(IMG_DST_DIR)
	cp ./files/rescue.root.emmc.cpio.gz_pad.img $(PKG_BUILD_DIR)/components/packages/generic
	cp ./files/rsa_key_2048.fw_$(SOC).pem $(PKG_BUILD_DIR)/secure_key/rsa_key_2048.fw.pem
	rm $(PKG_BUILD_DIR)/components/packages/generic/rescue.root.emmc.cpio.gz_pad_32bit.img
endef

define Build/Compile
	mv $(PKG_BUILD_DIR)/* $(IMG_DST_DIR)/
endef

define Build/Clean
	rm -rf $(IMG_DST_DIR)
endef

define Package/rtk-imagefile/install
	#
endef

$(eval $(call BuildPackage,rtk-imagefile))
