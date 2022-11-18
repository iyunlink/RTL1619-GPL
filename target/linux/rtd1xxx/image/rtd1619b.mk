define Device/rtd1619b
  FILESYSTEMS := squashfs ext4
  KERNEL := kernel-bin
  DEVICE_NAME := realtek
  DEVICE_VENDOR := Realtek
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek
  DEVICE_DTS := rtd1619b-$(1) rtd1619b-rescue
  DEVICE_PACKAGES := nas
  PROFILES := Default $$(DEVICE_NAME)
  IMAGES := install.img
  IMAGE/install.img := rtkimg
  SOC := stark
  VIDEO_FW_NAME := bluecore.video.$$(SOC).zip
  VIDEO_FW3_NAME := bluecore.ve3.$$(SOC).zip
  SYSTEM_MAP_FILE := System.map.audio.$$(SOC)
  BOARD_NAME :=
  AUDIO_FW_NAME :=
  FLASH_SIZE :=
  CLEAR_OVERLAY := n
  IMAGE_FORMAT :=
  NAS_SUFFIX ?= nas
endef
DEVICE_VARS += AUDIO_FW_NAME VIDEO_FW_NAME SYSTEM_MAP_FILE BOARD_NAME FLASH_SIZE CLEAR_OVERLAY IMAGE_FORMAT NAS_SUFFIX
DEVICE_VARS += VIDEO_FW3_NAME

define Device/bleedingedge-emmc
  $(Device/rtd1619b)
  DEVICE_MODEL := BleedingEdge eMMC board
  DEVICE_PACKAGES += kmod-rtk-emmc kmod-rtk-tee
  BOARD_NAME := emmc
  AUDIO_FW_NAME := bluecore.audio.$$(SOC).PB.zip
  FLASH_SIZE := 8gb
  CLEAR_OVERLAY := y
endef

define Device/bleedingedge-emmc-2gb
  $(Device/bleedingedge-emmc)
  DEVICE_PACKAGES += kmod-rtk-codec
  DEVICE_VARIANT := 2GB
endef

TARGET_DEVICES += bleedingedge-emmc-2gb

define Device/bleedingedge-emmc-2gb-router
  $(Device/bleedingedge-emmc)
  DEVICE_MODEL += for Router
  DEVICE_PACKAGES += $(DEFAULT_PACKAGES.router)
  DEVICE_VARIANT := 2GB
  CLEAR_OVERLAY := n
  SUPPORTED_DEVICES := realtek,bleeding-edge-emmc-router
  IMAGE/install.img := rtkimg | append-metadata
endef

TARGET_DEVICES += bleedingedge-emmc-2gb-router

define Device/bleedingedge-spi
  $(Device/rtd1619b)
  KERNEL := kernel-bin | lzma
  DEVICE_MODEL := BleedingEdge SPI board
  DEVICE_PACKAGES += kmod-rtk-spi
  BOARD_NAME := spi
  AUDIO_FW_NAME := bluecore.audio.$$(SOC)_slim.zip
  FLASH_SIZE := 16MB
  IMAGE_FORMAT := .lzma
endef

define Device/bleedingedge-spi-2gb
  $(Device/bleedingedge-spi)
  DEVICE_PACKAGES += kmod-rtk-codec
  DEVICE_VARIANT := 2GB
endef

TARGET_DEVICES += bleedingedge-spi-2gb

define Device/bleedingedge-spi-2gb-purenas
  $(Device/bleedingedge-spi)
  DEVICE_MODEL += for Pure NAS
  DEVICE_VARIANT := 2GB
  NAS_SUFFIX := purenas
endef

TARGET_DEVICES += bleedingedge-spi-2gb-purenas
