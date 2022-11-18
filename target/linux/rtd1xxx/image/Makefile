# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2015-2021 Realtek Semiconductor Corp.

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

include $(SUBTARGET).mk
-include $(SUBTARGET)-*.mk

#definitions for Realtek Image Creator

FIRMWARE_DIR:=$(TOPDIR)/package/realtek/rtk-bluecore/files
IMG_NAME:=$(CURDIR)/rtk-imagefile
TARGET_IMAGE_DIR:=$(IMG_NAME)/components/packages/generic
ROOTFS_TYPE:=squash
IMAGE_ROOTFS_FILE:=squashfs1.img

ifdef CONFIG_RTK_3RD_PARTY_ROOTFS
  define Image/mkfs/rootfs-prepare
	$(TAR) -cp --numeric-owner --owner=0 --group=0 --mode=a-s --sort=name \
		-C $(TARGET_DIR)/$(MODULES_SUBDIR) . | gzip -9n > $(TARGET_ROOTFS_DIR)/$(LINUX_UNAME_VERSION).tar.gz
	rm -rf $(TARGET_DIR)
	mkdir -p $(TARGET_DIR)
	$(FAKEROOT) $(TAR) -xf $(CURDIR)/realtek.tar.gz -C $(TARGET_DIR)
	mkdir -p $(TARGET_DIR)/$(MODULES_SUBDIR)
	$(TAR) -xf $(TARGET_ROOTFS_DIR)/$(LINUX_UNAME_VERSION).tar.gz -C $(TARGET_DIR)/$(MODULES_SUBDIR)
  endef
 ifdef CONFIG_RTK_3RD_PARTY_EXT4_ROOTFS
  NAS_SUFFIX:=nas_linux
  ROOTFS_TYPE:=ext4fs
  IMAGE_ROOTFS_FILE:=rootfs.ext4.bin
  DTB_CMD=$(SED) 's,squashfs,ext4 rw ,g' $(TARGET_IMAGE_DIR)/android.$(BOARD_NAME).dtb
 endif
endif

# from rtk-bluecore
FW_CMD= cp $(FIRMWARE_DIR)/$(AUDIO_FW_NAME) $(TARGET_IMAGE_DIR)/bluecore.audio.zip; \
	cp $(FIRMWARE_DIR)/$(SYSTEM_MAP_FILE) $(TARGET_IMAGE_DIR)/System.map.audio; \
	cp $(FIRMWARE_DIR)/$(VIDEO_FW_NAME) $(TARGET_IMAGE_DIR)/bluecore.video.zip;
FW_CMD+=if [ ! -z $(VIDEO_FW3_NAME) ]; then cp $(FIRMWARE_DIR)/$(VIDEO_FW3_NAME) $(TARGET_IMAGE_DIR)/bluecore.ve3.zip; fi

# prebuilt from rtk_tee_prebuilt
LAYOUT_CMD = [ -d $(CURDIR)/files/$(SUBTARGET)/$(DEVICE_NAME) ] && \
	     $(CP) $(CURDIR)/files/$(SUBTARGET)/$(DEVICE_NAME)/* $(IMG_NAME)/components/; \
	     [ ! -d $(CURDIR)/prebuilt/$(SUBTARGET)/$(DEVICE_NAME) ] || \
	     $(CP) $(CURDIR)/prebuilt/$(SUBTARGET)/$(DEVICE_NAME)/* $(IMG_NAME)/components/

#options for image file creators
ifeq ($(CONFIG_RTK_SECURE_BOOT_ENCRYPT),y)
check_fw_encrypt=y
else
check_fw_encrypt=n
endif

ifeq ($(CONFIG_RTK_SECURE_BOOT_EFUSE),y)
check_efuse_key=1
else
check_efuse_key=0
endif

#$(CP) $(KERNEL_BUILD_DIR)/image-$(DEVICE_DTS).dtb $(TARGET_IMAGE_DIR)/android.$(BOARD_NAME).dtb

define Build/rtkimg
	$(CP) $(IMAGE_KERNEL) $(TARGET_IMAGE_DIR)/$(BOARD_NAME).uImage$(IMAGE_FORMAT)
	$(CP) $(IMAGE_ROOTFS) $(TARGET_IMAGE_DIR)/$(IMAGE_ROOTFS_FILE)
	$(CP) $(CURDIR)/files/param.bin $(TARGET_IMAGE_DIR)
	$(foreach dts,$(DEVICE_DTS), $(CP) $(KDIR)/image-$(dts).dtb $(TARGET_IMAGE_DIR)/$(if $(findstring rescue, $(dts)),rescue,android).$(BOARD_NAME).dtb;)
	$(DTB_CMD)
	$(FW_CMD)
	$(LAYOUT_CMD)
	(cd $(IMG_NAME); \
	    make image PACKAGES=generic ROOTFS_TYPE=$(ROOTFS_TYPE) NAS_IMGS=y\
	    layout_type=$(BOARD_NAME) layout_size=$(FLASH_SIZE) \
	    F_EXT=$(IMAGE_FORMAT) \
	    install_avfile_count=0 \
	    enable_check_sha_hash=$(CONFIG_RTK_FWDESC_ENABLE_SHA256) \
	    check_fw_encrypt=$(check_fw_encrypt) \
	    efuse_key=$(check_efuse_key) \
	    force_secure=$(CONFIG_RTK_FORCE_SECURE_BOOT) \
	    clear_overlayfs_signature=$(CLEAR_OVERLAY) \
	    SECURE_BOOT=$(CONFIG_RTK_SECURE_BOOT) \
	    MODELNAME=$(SUBTARGET)-$(PROFILE) \
	    CHIP_ID=$(SOC) \
	    NAS_SUFFIX=$(NAS_SUFFIX) \
	    SQUASH_FS_FILE=$(IMAGE_ROOTFS_FILE) \
	    ; cd -)
	$(CP) $(IMG_NAME)/install.img $@
	(cd offline-gen; \
		./run.sh $(IMG_NAME)/install.img $(BOARD_NAME); \
		$(CP) fw_tbl.bin $(BIN_DIR)/$(basename $(notdir $@))-fw_tbl.bin; \
		cd -)
endef
$(eval $(call BuildImage))
