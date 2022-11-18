DEVICE?=$(call qstrip,$(CONFIG_TARGET_DEVICE))
VENDOR_DIR?=$(wildcard $(PLATFORM_SUBDIR)/vendor/$(DEVICE))
ifneq ($(VENDOR_DIR),)
define Package/base-files/install-target
	if [ -d "$(VENDOR_DIR)" ] && [ "$$$$(ls $(VENDOR_DIR) | wc -l)" -gt 0 ]; then \
	    $(CP) $(VENDOR_DIR)/* $(1)/; \
	fi
endef
endif
