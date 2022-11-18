define Device/rtd1319
  FILESYSTEMS := squashfs ext4
  KERNEL := kernel-bin
  DEVICE_NAME := realtek
  DEVICE_VENDOR := Realtek
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek
  DEVICE_DTS := rtd1319-$(1) rtd1319-rescue
  PROFILES := Default $$(DEVICE_NAME)
  IMAGES := install.img
  IMAGE/install.img := rtkimg
  SOC := hank
  VIDEO_FW_NAME := bluecore.video.$$(SOC).zip
  SYSTEM_MAP_FILE := System.map.audio.$$(SOC)
  BOARD_NAME :=
  AUDIO_FW_NAME :=
  FLASH_SIZE :=
  CLEAR_OVERLAY := n
  IMAGE_FORMAT :=
  NAS_SUFFIX ?= nas
endef
DEVICE_VARS += AUDIO_FW_NAME VIDEO_FW_NAME SYSTEM_MAP_FILE BOARD_NAME FLASH_SIZE CLEAR_OVERLAY IMAGE_FORMAT NAS_SUFFIX


define Device/pymparticles-emmc-2gb
  $(Device/rtd1319)
  DEVICE_MODEL := Pymparticles eMMC board
  DEVICE_VARIANT := 2GB
  DEVICE_PACKAGES := nas kmod-rtk-emmc kmod-rtk-tee
  BOARD_NAME := emmc
  AUDIO_FW_NAME := bluecore.audio.$$(SOC)_non_secure.PB.zip
  FLASH_SIZE := 8gb
  CLEAR_OVERLAY := y
endef

TARGET_DEVICES += pymparticles-emmc-2gb
