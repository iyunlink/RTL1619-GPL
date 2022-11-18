PKG_DRIVERS += \
	rtl8180 rtl8187 \
	rtlwifi rtlwifi-pci rtlwifi-btcoexist rtlwifi-usb rtl8192c-common \
	rtl8192ce rtl8192se rtl8192de rtl8192cu rtl8723bs rtl8821ae \
	rtl8xxxu rtw88

PKG_DRIVERS += \
	rtkwifiu \
	rtkwifiu-rtl8822cs rtkwifiu-rtl8852bs \
	rtkwifiu-rtl8852be

config-$(call config_package,rtl8180) += RTL8180
config-$(call config_package,rtl8187) += RTL8187

config-$(call config_package,rtlwifi) += RTL_CARDS RTLWIFI
config-$(call config_package,rtlwifi-pci) += RTLWIFI_PCI
config-$(call config_package,rtlwifi-btcoexist) += RTLBTCOEXIST
config-$(call config_package,rtlwifi-usb) += RTLWIFI_USB
config-$(call config_package,rtl8192c-common) += RTL8192C_COMMON
config-$(call config_package,rtl8192ce) += RTL8192CE
config-$(call config_package,rtl8192se) += RTL8192SE
config-$(call config_package,rtl8192de) += RTL8192DE
config-$(call config_package,rtl8192cu) += RTL8192CU
config-$(call config_package,rtl8821ae) += RTL8821AE
config-$(CONFIG_PACKAGE_RTLWIFI_DEBUG) += RTLWIFI_DEBUG

config-$(call config_package,rtl8xxxu) += RTL8XXXU
config-y += RTL8XXXU_UNTESTED

config-$(call config_package,rtl8723bs) += RTL8723BS
config-y += STAGING

config-$(call config_package,rtw88) += RTW88 RTW88_CORE RTW88_PCI
config-y += RTW88_8822BE RTW88_8822CE RTW88_8723DE

config-$(call config_package,rtkwifiu) += RTKWIFIU
config-$(call config_package,rtkwifiu-rtl8852be) += RTL8852BE
config-$(call config_package,rtkwifiu-rtl8822cs) += RTL8822CS
config-$(call config_package,rtkwifiu-rtl8852bs) += RTL8852BS

define KernelPackage/rtl818x/Default
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek Drivers for RTL818x devices
  URL:=https://wireless.wiki.kernel.org/en/users/drivers/rtl8187
  DEPENDS+= +kmod-eeprom-93cx6 +kmod-mac80211
endef

define KernelPackage/rtl8180
  $(call KernelPackage/rtl818x/Default)
  DEPENDS+= @PCI_SUPPORT
  TITLE+= (RTL8180 PCI)
  FILES:=$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtl818x/rtl8180/rtl818x_pci.ko
  AUTOLOAD:=$(call AutoProbe,rtl818x_pci)
endef

define KernelPackage/rtl8187
$(call KernelPackage/rtl818x/Default)
  DEPENDS+= @USB_SUPPORT +kmod-usb-core
  TITLE+= (RTL8187 USB)
  FILES:=$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtl818x/rtl8187/rtl8187.ko
  AUTOLOAD:=$(call AutoProbe,rtl8187)
endef

define KernelPackage/rtlwifi/config
	config PACKAGE_RTLWIFI_DEBUG
		bool "Realtek wireless debugging"
		depends on PACKAGE_kmod-rtlwifi
		help
		  Say Y, if you want to debug realtek wireless drivers.

endef

define KernelPackage/rtlwifi
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek common driver part
  DEPENDS+= @(PCI_SUPPORT||USB_SUPPORT) +kmod-mac80211 +@DRIVER_11N_SUPPORT
  FILES:=$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtlwifi/rtlwifi.ko
  HIDDEN:=1
endef

define KernelPackage/rtlwifi-pci
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek common driver part (PCI support)
  DEPENDS+= @PCI_SUPPORT +kmod-rtlwifi
  FILES:=$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtlwifi/rtl_pci.ko
  AUTOLOAD:=$(call AutoProbe,rtl_pci)
  HIDDEN:=1
endef

define KernelPackage/rtlwifi-btcoexist
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek BT coexist support
  DEPENDS+= +kmod-rtlwifi
  FILES:=$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtlwifi/btcoexist/btcoexist.ko
  AUTOLOAD:=$(call AutoProbe,btcoexist)
  HIDDEN:=1
endef

define KernelPackage/rtlwifi-usb
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek common driver part (USB support)
  DEPENDS+= @USB_SUPPORT +kmod-usb-core +kmod-rtlwifi
  FILES:=$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtlwifi/rtl_usb.ko
  AUTOLOAD:=$(call AutoProbe,rtl_usb)
  HIDDEN:=1
endef

define KernelPackage/rtl8192c-common
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek RTL8192CE/RTL8192CU common support module
  DEPENDS+= +kmod-rtlwifi
  FILES:= $(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtlwifi/rtl8192c/rtl8192c-common.ko
  HIDDEN:=1
endef

define KernelPackage/rtl8192ce
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek RTL8192CE/RTL8188CE support
  DEPENDS+= +kmod-rtlwifi-pci +kmod-rtl8192c-common +rtl8192ce-firmware
  FILES:= $(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtlwifi/rtl8192ce/rtl8192ce.ko
  AUTOLOAD:=$(call AutoProbe,rtl8192ce)
endef

define KernelPackage/rtl8192se
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek RTL8192SE/RTL8191SE support
  DEPENDS+= +kmod-rtlwifi-pci +rtl8192se-firmware
  FILES:=$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtlwifi/rtl8192se/rtl8192se.ko
  AUTOLOAD:=$(call AutoProbe,rtl8192se)
endef

define KernelPackage/rtl8192de
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek RTL8192DE/RTL8188DE support
  DEPENDS+= +kmod-rtlwifi-pci +rtl8192de-firmware
  FILES:= $(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtlwifi/rtl8192de/rtl8192de.ko
  AUTOLOAD:=$(call AutoProbe,rtl8192de)
endef

define KernelPackage/rtl8192cu
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek RTL8192CU/RTL8188CU support
  DEPENDS+= +kmod-rtlwifi-usb +kmod-rtl8192c-common +rtl8192cu-firmware
  FILES:= $(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtlwifi/rtl8192cu/rtl8192cu.ko
  AUTOLOAD:=$(call AutoProbe,rtl8192cu)
endef

define KernelPackage/rtl8821ae
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek RTL8821AE support
  DEPENDS+= +kmod-rtlwifi-btcoexist +kmod-rtlwifi-pci +rtl8821ae-firmware
  FILES:= $(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtlwifi/rtl8821ae/rtl8821ae.ko
  AUTOLOAD:=$(call AutoProbe,rtl8821ae)
endef

define KernelPackage/rtl8xxxu
  $(call KernelPackage/mac80211/Default)
  TITLE:=alternative Realtek RTL8XXXU support
  DEPENDS+= @USB_SUPPORT +kmod-usb-core +kmod-mac80211
  FILES:= $(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtl8xxxu/rtl8xxxu.ko
  AUTOLOAD:=$(call AutoProbe,rtl8xxxu)
endef

define KernelPackage/rtl8xxxu/description
  This is an alternative driver for various Realtek RTL8XXX
  parts written to utilize the Linux mac80211 stack.
  The driver is known to work with a number of RTL8723AU,
  RL8188CU, RTL8188RU, RTL8191CU, and RTL8192CU devices

  This driver is under development and has a limited feature
  set. In particular it does not yet support 40MHz channels
  and power management. However it should have a smaller
  memory footprint than the vendor drivers and benetifs
  from the in kernel mac80211 stack.

  It can coexist with drivers from drivers/staging/rtl8723au,
  drivers/staging/rtl8192u, and drivers/net/wireless/rtlwifi,
  but you will need to control which module you wish to load.

  RTL8XXXU_UNTESTED is enabled
  This option enables detection of Realtek 8723/8188/8191/8192 WiFi
  USB devices which have not been tested directly by the driver
  author or reported to be working by third parties.

  Please report your results!
endef

define KernelPackage/rtw88
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek RTL8822BE/RTL8822CE/RTL8723DE
  DEPENDS+= @(PCI_SUPPORT) +kmod-mac80211 +@DRIVER_11AC_SUPPORT +@DRIVER_11N_SUPPORT
  FILES:=\
	$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtw88/rtw88_8822be.ko \
	$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtw88/rtw88_8822b.ko \
	$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtw88/rtw88_8822ce.ko \
	$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtw88/rtw88_8822c.ko \
	$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtw88/rtw88_8723de.ko \
	$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtw88/rtw88_8723d.ko \
	$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtw88/rtw88_core.ko \
	$(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtw88/rtw88_pci.ko
  AUTOLOAD:=$(call AutoProbe,rtw88_8822be rtw88_8822ce rtw88_8723de)
endef

define KernelPackage/rtl8723bs
  $(call KernelPackage/mac80211/Default)
  TITLE:=Realtek RTL8723BS SDIO Wireless LAN NIC driver (staging)
  DEPENDS+=+kmod-mmc +kmod-mac80211
  FILES:=$(PKG_BUILD_DIR)/drivers/staging/rtl8723bs/r8723bs.ko
  AUTOLOAD:=$(call AutoProbe,r8723bs)
endef

define KernelPackage/rtl8723bs/description
 This option enables support for RTL8723BS SDIO drivers, such as the wifi found
 on the 1st gen Intel Compute Stick, the CHIP and many other Intel Atom and ARM
 based devices.
endef

ifneq ($(CONFIG_PACKAGE_kmod-rtkwifiu),)
RTKWIFIU_DIR := $(PKG_BUILD_DIR)/drivers/net/wireless/realtek/rtkwifiu
RTKWIFIU_PKG := rtkwifiu-rtl8822be rtkwifiu-rtl8822ce rtkwifiu-rtl8822cs rtkwifiu-rtl8852be rtkwifiu-rtl8852bs
#PKG_DRIVERS += $(RTKWIFIU_PKG)
BACKPORT_VER_FILES := os_dep/linux/ioctl_cfg80211.h os_dep/linux/ioctl_cfg80211.c os_dep/linux/rtw_cfgvendor.h \
		      os_dep/linux/rtw_cfgvendor.c os_dep/linux/wifi_regd.c core/rtw_ap.c
RESTORE_LINUX_API_FILES := os_dep/linux/ioctl_cfg80211.c
CONFIG_RTKWIFI_COMMON := y
endif

PKG_RTKWIFIU_NAME:=rtkwifiu
PKG_RTKWIFIU_VERSION:=f0fb197
PKG_RTKWIFIU_SOURCE:=$(PKG_RTKWIFIU_NAME)-$(PKG_RTKWIFIU_VERSION).tar.bz2
PKG_RTKWIFIU_SOURCE_URL:=file://rtk-dl
PKG_RTKWIFIU_SUBDIR:=$(PKG_RTKWIFIU_NAME)
PKG_RTKWIFIU_HASH:=af4c88db9c568601faf7d50b0959dd88435feeb9dedfbf226816bf41f2e045ce

define Download/rtkwifiu
  PROTO:=lcp
  VERSION:=$(PKG_RTKWIFIU_VERSION)
  SUBDIR:=$(PKG_RTKWIFIU_SUBDIR)
  FILE:=$(PKG_RTKWIFIU_SOURCE)
  URL:=$(PKG_RTKWIFIU_SOURCE_URL)
  HASH:=$(PKG_RTKWIFIU_HASH)
endef
$(eval $(call Download,rtkwifiu))

define KernelPackage/rtkwifiu/Default
  VERSION:=$(PKG_RTKWIFIU_VERSION)
  TITLE:=Realtek Compat Wifi Drivers
  SUBMENU:=Realtek modules
  DEPENDS+= +kmod-cfg80211 +@DRIVER_11N_SUPPORT +@DRIVER_11AC_SUPPORT #+rtwpriv
endef

define KernelPackage/rtkwifiu
  $(call KernelPackage/rtkwifiu/Default)
  FILES:=
  MENU:=1
endef

define KernelPackage/rtkwifiu/config
  if PACKAGE_kmod-rtkwifiu
        config RTKWIFIU
		default y
		 bool

        config RTKWIFIU_CONCURRENT_MODE
		bool "Enable CONFIG_CONCURRENT_MODE"
		depends on RTKWIFIU
		default y
		help
		Select this to enable CONFIG_CONCURRENT_MODE.
  endif
endef

define KernelPackage/rtkwifiu-rtl8852be
  $(call KernelPackage/rtkwifiu/Default)
  DEPENDS+= @PCI_SUPPORT @PACKAGE_kmod-rtkwifiu
  TITLE+= (RTL8852B PCIe)
  FILES:=$(RTKWIFIU_DIR)/rtl8852be/8852be.ko
  AUTOLOAD:=$(call AutoLoad,61,8852be)
endef

define KernelPackage/rtkwifiu-rtl8822cs
  $(call KernelPackage/rtkwifiu/Default)
  DEPENDS+= @PACKAGE_kmod-rtkwifiu
  TITLE+= (RTL8822C SDIO)
  FILES:=$(RTKWIFIU_DIR)/rtl8822cs/8822cs.ko
  AUTOLOAD:=$(call AutoProbe,8822cs)
endef

define KernelPackage/rtkwifiu-rtl8852bs
  $(call KernelPackage/rtkwifiu/Default)
  DEPENDS+= @PACKAGE_kmod-rtkwifiu
  TITLE+= (RTL8852B SDIO)
  FILES:=$(RTKWIFIU_DIR)/rtl8852bs/8852bs.ko
  AUTOLOAD:=$(call AutoProbe,8852bs)
endef

ifneq ($(CONFIG_PACKAGE_kmod-rtkwifiu),)
RTKWIFIU_OPTS+=CONFIG_PLATFORM_RTK13XX=$(CONFIG_TARGET_rtd1xxx_rtd1319)
RTKWIFIU_OPTS+=CONFIG_PLATFORM_RTK1319=$(CONFIG_TARGET_rtd1xxx_rtd1319)
RTKWIFIU_OPTS+=CONFIG_PLATFORM_RTK16XXB=$(CONFIG_TARGET_rtd1xxx_rtd1619b)
RTKWIFIU_OPTS+=CONFIG_PLATFORM_I386_PC=
RTKWIFIU_OPTS+=TopDIR=$(RTKWIFIU_DIR)
RTKWIFIU_OPTS+=DRV_PATH=$(RTKWIFIU_DIR)/*/

##expr $((5<<16)) + $((10<<8)) + $((110))
BACKPORTS_LINUX_VERSION_CODE:=330350

USER_EXTRA_CFLAGS:=-DBUILD_OPENWRT -Wno-error=date-time -DBACKPORT_VERSION_CODE=$(BACKPORTS_LINUX_VERSION_CODE) -DUSE_DMA_ALLOCATE
ifeq ($(CONFIG_PACKAGE_kmod-rtkwifiu-rtl8852be)$(CONFIG_PACKAGE_kmod-rtkwifiu-rtl8852bs),)
USER_EXTRA_CFLAGS+= -DCONFIG_RTW_HOSTAPD_ACS
endif
RTKWIFIU_OPTS+=USER_EXTRA_CFLAGS="$(USER_EXTRA_CFLAGS)"
endif

define Build/Patch/rtkwifiu
	$(call PatchDir,$(PKG_BUILD_DIR),./rtkwifiu/patches/common/,rtkwifiu/common/)
ifneq ($(CONFIG_PACKAGE_kmod-rtkwifiu-rtl8852be),)
	$(call PatchDir,$(PKG_BUILD_DIR),./rtkwifiu/patches/rtkwifiu-rtl8852be,rtkwifiu/rtkwifiu-rtl8852be/)
endif
ifneq ($(CONFIG_PACKAGE_kmod-rtkwifiu-rtl8822cs),)
	$(call PatchDir,$(PKG_BUILD_DIR),./rtkwifiu/patches/rtkwifiu-rtl8822cs,rtkwifiu/rtkwifiu-rtl8822cs/)
endif
ifneq ($(CONFIG_PACKAGE_kmod-rtkwifiu-rtl8852bs),)
	$(call PatchDir,$(PKG_BUILD_DIR),./rtkwifiu/patches/rtkwifiu-rtl8852bs,rtkwifiu/rtkwifiu-rtl8852bs/)
endif
endef

define Quilt/Refresh/rtkwifiu
	$(call Quilt/RefreshDir,$(PKG_BUILD_DIR),./rtkwifiu/patches/common/,rtkwifiu/common/)
ifneq ($(CONFIG_PACKAGE_kmod-rtkwifiu-rtl8852be),)
	$(call Quilt/RefreshDir,$(PKG_BUILD_DIR),./rtkwifiu/patches/rtkwifiu-rtl8852be,rtkwifiu/rtkwifiu-rtl8852be/)
endif
ifneq ($(CONFIG_PACKAGE_kmod-rtkwifiu-rtl8822cs),)
	$(call Quilt/RefreshDir,$(PKG_BUILD_DIR),./rtkwifiu/patches/rtkwifiu-rtl8822cs,rtkwifiu/rtkwifiu-rtl8822cs/)
endif
ifneq ($(CONFIG_PACKAGE_kmod-rtkwifiu-rtl8852bs),)
	$(call Quilt/RefreshDir,$(PKG_BUILD_DIR),./rtkwifiu/patches/rtkwifiu-rtl8852bs,rtkwifiu/rtkwifiu-rtl8852bs/)
endif
endef
