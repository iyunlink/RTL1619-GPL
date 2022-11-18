
include $(TOPDIR)/rules.mk

PKG_NAME:=appfilter
PKG_RELEASE:=1

PKG_SOURCE_URL:=https://github.com/destan19/OpenAppFilter
PKG_SOURCE_BRANCH:=master
PKG_SOURCE_PROTO:=git
PKG_SOURCE_VERSION:=7d4b72f
PKG_SOURCE:=oaf-$(PKG_SOURCE_VERSION).tar.xz
PKG_SOURCE_SUBDIR:=oaf-$(PKG_SOURCE_VERSION)
PKG_HASH=f87eb98973b1280cdd621d12deff94331a65bbe0508cc6af3f19b0a67f32eb9a
PKG_BUILD_DIR=$(BUILD_DIR)/$(PKG_SOURCE_SUBDIR)

include $(INCLUDE_DIR)/package.mk

define Package/appfilter
  SECTION:=net
  CATEGORY:=Network
  DEPENDS:=+libubox +libubus +libuci +libpthread +libjson-c +libblobmsg-json +kmod-oaf
  TITLE:=Open App filter userspace module
  URL:=https://github.com/destan19/OpenAppFilter
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR)/open-app-filter/src \
	CC="$(TARGET_CROSS)gcc" \
	CFLAGS="$(TARGET_CFLAGS)" \
	LIBS="$(TARGET_LDFLAGS) -lm -lpthread -lubox -luci -lubus -ljson-c -lblobmsg_json" \
	all
endef	

define Package/appfilter/description
  Open App filter userspace module
endef

define Package/appfilter/install
	$(INSTALL_DIR) $(1)/usr/bin $(1)/etc/init.d
	$(INSTALL_DIR) $(1)/etc/appfilter
	$(INSTALL_DIR) $(1)/etc/config
	$(CP) $(PKG_BUILD_DIR)/open-app-filter/files/*.cfg $(1)/etc/appfilter/ 
	$(CP) $(PKG_BUILD_DIR)/open-app-filter/files/*.txt $(1)/etc/appfilter/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/open-app-filter/files/appfilter.init $(1)/etc/init.d/appfilter
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/open-app-filter/files/oaf_rule $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/open-app-filter/files/gen_class.sh $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/open-app-filter/files/appfilter.config $(1)/etc/config/appfilter
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/open-app-filter/src/oafd $(1)/usr/bin
	$(CP) ./files/* $(1)/etc/appfilter/
endef

$(eval $(call BuildPackage,appfilter))
