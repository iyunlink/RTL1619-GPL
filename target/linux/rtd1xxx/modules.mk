RTK_MENU:=Realtek kernel options

define KernelPackage/rtk-spi
  SUBMENU:=$(RTK_MENU)
  TITLE:=Realtek SPI NOR flash driver
  KCONFIG:= \
	CONFIG_MTD_CMDLINE_PARTS=y \
	CONFIG_MTD=y \
	CONFIG_MTD_SPI_NOR=y \
	CONFIG_SPI=y \
	CONFIG_SPI_MASTER=y \
	CONFIG_SPI_MEM=y \
	CONFIG_SPI_RTK_SFC=y

  FILES:=
  AUTOLOAD:=
  DEPENDS:=@TARGET_rtd1xxx @DEFAULT_kmod-rtk-spi
endef

define KernelPackage/rtk-spi/description
  This package contains the Realtek SPI NOR flash driver.
endef

$(eval $(call KernelPackage,rtk-spi))

define KernelPackage/rtk-emmc
  SUBMENU:=$(RTK_MENU)
  TITLE:=Realtek eMMC driver
  KCONFIG:= \
	CONFIG_MMC_DW_CQE=y \
	CONFIG_MMC_DW_CQE_PLTFM=y \

  FILES:=
  AUTOLOAD:=
  DEPENDS:=@TARGET_rtd1xxx @DEFAULT_kmod-rtk-emmc
endef

define KernelPackage/rtk-emmc/description
  This package enables the Realtek eMMC driver.
endef

$(eval $(call KernelPackage,rtk-emmc))

define KernelPackage/rtk-codec
  SUBMENU:=$(RTK_MENU)
  TITLE:=Realtek Codec Driver for HW transcode
  KCONFIG:= \
	CONFIG_RTD16XXB_RTK_CODEC=y \
	CONFIG_RTD16XXB_VE1_CODEC=y \
	CONFIG_RTK_IMAGE_CODEC=y \
	CONFIG_RTK_BUFLOCK=y

  FILES:=
  AUTOLOAD:=
  DEPENDS:=@TARGET_rtd1xxx @DEFAULT_kmod-rtk-codec
endef

define KernelPackage/rtk-codec/description
  This package contains the Realtek Codec driver for HW transcode
endef

$(eval $(call KernelPackage,rtk-codec))

define KernelPackage/rtk-tee
  SUBMENU:=$(RTK_MENU)
  TITLE:=Realtek TEE Driver
  KCONFIG:= \
	CONFIG_COMMON_CLK_REALTEK_TEE=y \
	CONFIG_OPTEE=y \
	CONFIG_OPTEE_SHM_NUM_PRIV_PAGES=1 \
	CONFIG_OPTEE_MEM_API=y \
	CONFIG_OPTEE_MEM_API_DIS=n \
	CONFIG_TEE=y

  FILES:=
  AUTOLOAD:=
  DEPENDS:=@TARGET_rtd1xxx @DEFAULT_kmod-rtk-tee
endef

define KernelPackage/rtk-tee/description
  This package contains the Realtek TEE driver
endef

$(eval $(call KernelPackage,rtk-tee))

define KernelPackage/rtk-hse
  SUBMENU:=$(RTK_MENU)
  TITLE:=Realtek HSE driver
  DEPENDS:=@TARGET_rtd1xxx
  KCONFIG:= \
	CONFIG_ASYNC_TX_DMA=y \
	CONFIG_RTK_HSE \
	CONFIG_RTK_HSE_DMA=y
  FILES:=$(LINUX_DIR)/drivers/soc/realtek/common/hse/rtk-hse.ko
  AUTOLOAD:=$(call AutoProbe,rtk-hse)
endef

define KernelPackage/rtk-hse/description
  This package contains the Realtek HSE driver.
endef

$(eval $(call KernelPackage,rtk-hse))

define KernelPackage/rtl8125
  SUBMENU:=$(NETWORK_DEVICES_MENU)
  TITLE:=Realtek R8125 PCI-E 2.5 Gigabit Ethernet driver
  DEPENDS:=@TARGET_rtd1xxx @PCI_SUPPORT +kmod-mii
  KCONFIG:=CONFIG_R8125
  FILES:=$(LINUX_DIR)/drivers/net/ethernet/realtek/r8125/r8125.ko
  AUTOLOAD:=$(call AutoProbe,r8125)
endef

define KernelPackage/rtl8125/description
  This package contains the Realtek R8125 PCI-E 2.5 Gigibit Ethernet driver
endef

$(eval $(call KernelPackage,rtl8125))

define KernelPackage/rtl8152
  SUBMENU:=$(NETWORK_DEVICES_MENU)
  TITLE:=Realtek R8152 USB 2.5 Gigabit Ethernet driver
  DEPENDS:=@TARGET_rtd1xxx @USB_SUPPORT
  KCONFIG:=CONFIG_USB_RTL8152
  FILES:=$(LINUX_DIR)/drivers/net/usb/r8152/r8152.ko
  AUTOLOAD:=$(call AutoProbe,r8152)
endef

define KernelPackage/rtl8152/description
  This package contains the Realtek R8152 USB 2.5 Gigibit Ethernet driver
endef

$(eval $(call KernelPackage,rtl8152))

define KernelPackage/rtl8168
  SUBMENU:=$(NETWORK_DEVICES_MENU)
  TITLE:=Realtek R8168 PCI-E Gigabit Ethernet driver
  DEPENDS:=@TARGET_rtd1xxx @PCI_SUPPORT +kmod-mii
  KCONFIG:=CONFIG_R8168
  FILES:=$(LINUX_DIR)/drivers/net/ethernet/realtek/r8168/r8168.ko
  AUTOLOAD:=$(call AutoProbe,r8168)
endef

define KernelPackage/rtl8168/description
  This package contains the Realtek R8168 PCI-E Gigibit Ethernet driver
endef

$(eval $(call KernelPackage,rtl8168))

define KernelPackage/ecryptfs
  SECTION:=kernel
  CATEGORY:=Kernel modules
  SUBMENU:=Filesystems
  TITLE:=Ecryptfs kernel module
  KCONFIG:= \
	CONFIG_ECRYPT_FS=y \
	CONFIG_ECRYPT_FS_MESSAGING=n \
	CONFIG_ECRYPT_FS_FAST_INIT=y
  DEPENDS:= +kmod-keys-encrypted
  FILES:=$(LINUX_DIR)/fs/ecryptfs/ecryptfs.ko
  AUTOLOAD:=$(call AutoProbe,ecryptfs)
endef

define KernelPackage/ecryptfs/description
  This package contains the Ecryptfs Kernel Module
endef

$(eval $(call KernelPackage,ecryptfs))

define KernelPackage/ntfs3
  SECTION:=kernel
  CATEGORY:=Kernel modules
  SUBMENU:=Filesystems
  TITLE:=NTFS3 backport from 5.15
  KCONFIG:= \
	CONFIG_NTFS3_FS \
	CONFIG_NTFS3_64BIT_CLUSTER=n \
	CONFIG_NTFS3_LZX_XPRESS=y \
	CONFIG_NTFS3_FS_POSIX_ACL=n
  FILES:=$(LINUX_DIR)/fs/ntfs3/ntfs3.ko
  AUTOLOAD:=$(call AutoProbe,ntfs3)
endef

define KernelPackage/ntfs3/description
  This package contains the ntfs3 Kernel Module
endef

$(eval $(call KernelPackage,ntfs3))

define KernelPackage/rtk-docker
  SUBMENU:=$(RTK_MENU)
  TITLE:=Realtek docker config
  KCONFIG:= \
	CONFIG_PERF_EVENTS=y \
	CONFIG_CGROUPS=y \
	CONFIG_FREEZER=y \
	CONFIG_CGROUP_FREEZER=y \
	CONFIG_CGROUP_DEVICE=y \
	CONFIG_CGROUP_SCHED=y \
	CONFIG_CGROUP_PIDS=y \
	CONFIG_CPUSETS=y \
	CONFIG_CGROUP_CPUACCT=y \
	CONFIG_NAMESPACES=y \
	CONFIG_UTS_NS=y \
	CONFIG_IPC_NS=y \
	CONFIG_USER_NS=y \
	CONFIG_PID_NS=y \
	CONFIG_NET_NS=y \
	CONFIG_SECCOMP=y \
	CONFIG_SECCOMP_FILTER=y \

  FILES:=
  AUTOLOAD:=
  DEPENDS:=@TARGET_rtd1xxx @DEFAULT_kmod-rtk-emmc
  DEPENDS+= +@KERNEL_DEVTMPFS +@KERNEL_DEVTMPFS_MOUNT
  DEPENDS+= +kmod-br-netfilter +kmod-nf-conntrack-netlink \
	    +kmod-nf-ipvs +kmod-nf-nat +kmod-ipt-nat \
	    +kmod-ikconfig +kmod-ipt-extra +kmod-veth \
	    +iptables-mod-extra
endef

define KernelPackage/rtk-docker/description
  This package enables the kernel options for Docker
endef

$(eval $(call KernelPackage,rtk-docker))
