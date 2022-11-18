#
# Copyright (c) 2013-2018, 2020 The Linux Foundation. All rights reserved.
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all copies.
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=shortcut-fe
PKG_RELEASE:=7

PKG_SOURCE_URL:=https://source.codeaurora.org/quic/qsdk/oss/lklm/shortcut-fe
PKG_SOURCE_PROTO:=git
PKG_SOURCE_DATE:=2021-03-17
PKG_SOURCE_VERSION:=697977d8d0ccf0ab596e5692d08608a75dd7f33d
#PKG_MIRROR_HASH:=659fa82a431e15af797a6c7069faeee02810453ad8b576c51c29f95a1761a045
PKG_MIRROR_HASH:=25267cdc26702b770906318016015a93ad25cc1c5281113e5f95b4f809b8c1bd

include $(INCLUDE_DIR)/package.mk

define KernelPackage/shortcut-fe
  SECTION:=kernel
  CATEGORY:=Kernel modules
  SUBMENU:=Network Support
  DEPENDS:=@IPV6 +kmod-nf-conntrack
  TITLE:=Kernel driver for SFE
  FILES:= \
	   $(PKG_BUILD_DIR)/shortcut-fe/shortcut-fe.ko \
	   $(PKG_BUILD_DIR)/shortcut-fe/shortcut-fe-ipv6.ko
  KCONFIG:= \
	   CONFIG_NF_CONNTRACK_EVENTS=y \
	   CONFIG_NF_CONNTRACK_TIMEOUT=y \
	   CONFIG_SHORTCUT_FE=y \
	   CONFIG_XFRM=y
  PROVIDES:=$(PKG_NAME)
  AUTOLOAD:=$(call AutoLoad,09,shortcut-fe shortcut-fe-ipv6)
endef

define KernelPackage/shortcut-fe/Description
Shortcut is an in-Linux-kernel IP packet forwarding engine.
endef

define KernelPackage/shortcut-fe/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) ./files/usr/bin/sfe_dump $(1)/usr/bin
	$(INSTALL_DIR) $(1)/etc
	$(INSTALL_BIN) ./files/etc/firewall.sfe $(1)/etc/
	$(if $(CONFIG_PACKAGE_kmod-fast-classifier),$(SED) "s/shortcut_fe_cm/fast_classifier/" $(1)/etc/firewall.sfe)
endef

define KernelPackage/shortcut-fe-cm
  SECTION:=kernel
  CATEGORY:=Kernel modules
  SUBMENU:=Network Support
  DEPENDS:=+kmod-ipt-conntrack +kmod-shortcut-fe
  TITLE:=Kernel driver for SFE
  FILES:=$(PKG_BUILD_DIR)/shortcut-fe/shortcut-fe-cm.ko
  KCONFIG:= \
	   CONFIG_NF_CONNTRACK_CHAIN_EVENTS=y \
	   CONFIG_NF_CONNTRACK_EVENTS=y \
	   CONFIG_XFRM=y
  CONFLICTS:=kmod-fast-classifier
endef

define KernelPackage/shortcut-fe-cm/Description
Simple connection manager for the Shortcut forwarding engine.
endef

define KernelPackage/fast-classifier/Default
  SECTION:=kernel
  CATEGORY:=Kernel modules
  SUBMENU:=Network Support
  DEPENDS:=+kmod-ipt-conntrack +kmod-shortcut-fe
  TITLE:=Kernel driver for FAST Classifier
  FILES:=$(PKG_BUILD_DIR)/fast-classifier/fast-classifier.ko
  KCONFIG:= \
	   CONFIG_NF_CONNTRACK_EVENTS=y \
	   CONFIG_NF_CONNTRACK_CHAIN_EVENTS=y \
	   CONFIG_NF_CONNTRACK_MARK=y \
	   CONFIG_XFRM=y
endef

define KernelPackage/fast-classifier
  $(call KernelPackage/fast-classifier/Default)
endef

define KernelPackage/fast-classifier/Default/description
FAST Classifier talks to SFE to make decisions about offloading connections
endef

define KernelPackage/fast-classifier/description
$(call KernelPackage/fast-classifier/Default/description)
endef

define Package/fast-classifier-example
  TITLE:=Example user space program for fast-classifier
  DEPENDS:=+libnl +kmod-fast-classifier
endef

define Package/fast-classifier-example/description
Example user space program that communicates with fast
classifier kernel module
endef

define Build/Compile/kmod
	+$(MAKE) $(PKG_JOBS) -C "$(LINUX_DIR)" \
		$(KERNEL_MAKE_FLAGS) \
		$(PKG_MAKE_FLAGS) \
		M="$(PKG_BUILD_DIR)/$1" \
		EXTRA_CFLAGS+="-DSFE_SUPPORT_IPV6" \
		SFE_SUPPORT_IPV6=y \
		$2 \
		modules
endef

define Build/Compile/example
	$(TARGET_CC) -o $(PKG_BUILD_DIR)/userspace_fast_classifier \
		-I $(PKG_BUILD_DIR) \
		-I$(STAGING_DIR)/usr/include/libnl \
		-I$(STAGING_DIR)/usr/include/libnl3 \
		-lnl-genl-3 -lnl-3 \
		$(PKG_BUILD_DIR)/fast-classifier/nl_classifier_test.c
endef

define Build/Compile
	$(call Build/Compile/kmod,shortcut-fe)
	$(if $(CONFIG_PACKAGE_kmod-fast-classifier),$(call Build/Compile/kmod,fast-classifier,CONFIG_FAST_CLASSIFIER=m KBUILD_EXTRA_SYMBOLS="$(PKG_BUILD_DIR)/shortcut-fe/Module.symvers"))
	$(if $(CONFIG_PACKAGE_fast-classifier-example),$(Build/Compile/example))
endef

ifneq ($(CONFIG_PACKAGE_kmod-shortcut-fe),)
define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include/shortcut-fe
	$(CP) -rf $(PKG_BUILD_DIR)/shortcut-fe/sfe.h $(1)/usr/include/shortcut-fe
	$(if $(CONFIG_PACKAGE_kmod-fast-classifier),$(CP) $(PKG_BUILD_DIR)/fast-classifier/fast-classifier.h $(1)/usr/include/)
endef
endif

define Package/fast-classifier-example/install
	$(INSTALL_DIR) $(1)/sbin
	$(CP) $(PKG_BUILD_DIR)/userspace_fast_classifier $(1)/sbin/
endef

$(eval $(call KernelPackage,shortcut-fe))
$(eval $(call KernelPackage,shortcut-fe-cm))
$(eval $(call KernelPackage,fast-classifier))
$(eval $(call BuildPackage,fast-classifier-example))
