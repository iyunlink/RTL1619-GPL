# 
# Copyright (C) 2006-2013 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=ecryptfs-utils
PKG_VERSION:=111
PKG_RELEASE:=1

PKG_SOURCE:=$(PKG_NAME)_$(PKG_VERSION).orig.tar.gz
PKG_SOURCE_URL:=https://launchpad.net/ecryptfs/trunk/$(PKG_VERSION)/+download/
PKG_HASH:=112cb3e37e81a1ecd8e39516725dec0ce55c5f3df6284e0f4cc0f118750a987f

PKG_BUILD_DEPENDS:=intltool/host keyutils

PKG_FIXUP:=autoreconf
PKG_FIXUP:=patch-libtool
PKG_INSTALL:=1

include $(INCLUDE_DIR)/package.mk

define Package/ecryptfs-utils
	SECTION:=utils
	CATEGORY:=Utilities
	SUBMENU:=Filesystem
	TITLE:=eCryptfs utilities
	URL:=http://ecryptfs-utils.sourceforge.net/
	DEPENDS:=+openssl-util +libecryptfs +keyutils
endef

define Package/ecryptfs-utils/description
	eCryptfs utilities
endef

define Package/libecryptfs
	SECTION:=libs
	SUBMENU:=Filesystem
	CATEGORY:=Libraries
	TITLE:=eCryptfs Library
	DEPENDS:= +libgcrypt +libkeyutils +libopenssl +kmod-ecryptfs
endef

define Package/libecryptfs/description
	eCryptfs library
endef

TARGET_LDFLAGS += \
        -Wl,-rpath-link,$(STAGING_DIR)/usr/lib \
        -lgcrypt

CONFIGURE_ARGS += \
		--enable-openssl \
		--disable-pywrap \
		--disable-pam \
		--disable-gui \
		--disable-nss \
		--disable-pkcs11-helper \
		--disable-tspi

define Package/libecryptfs/install
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/* $(1)/usr/lib/
endef

define Package/ecryptfs-utils/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/bin/* $(1)/usr/bin/
	$(INSTALL_DIR) $(1)/sbin
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/sbin/* $(1)/sbin
#	$(INSTALL_DIR) $(1)/etc
#	$(CP) ./files/netconfig $(1)/etc/
#	$(INSTALL_DIR) $(1)/etc/init.d
#	$(INSTALL_BIN) ./files/ecryptfs-utils.init $(1)/etc/init.d/ecryptfs-utils
endef

$(eval $(call BuildPackage,ecryptfs-utils))
$(eval $(call BuildPackage,libecryptfs))
