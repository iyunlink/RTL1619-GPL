#
# Copyright (C) 2021 Realtek Realtek Semiconductor Corp. All rights reserved.
#
#
# Redistribution only in binary form is permitted provided that
# the following conditions are met:
#
# 1. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 2. The name of the author may not be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
# SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
# OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
# OF SUCH DAMAGE.

include $(TOPDIR)/rules.mk

PKG_NAME:=rtk_tee_prebuilt
PKG_VERSION:=41152e53
PKG_RELEASE:=2

DEVICE?=$(call qstrip,$(CONFIG_TARGET_DEVICE))

PKG_SOURCE_PROTO:=lcp
PKG_SOURCE_URL:=file://rtk-dl
PKG_SOURCE_VERSION:=41152e53
PKG_HASH:=skip
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION)-$(DEVICE).tar.gz

IMAGE_FILES_DIR:=$(TOPDIR)/vendor/$(DEVICE)/$(SUBTARGET)/image/prebuilt
IMAGE_TEE_DIR:=$(TOPDIR)/vendor/$(DEVICE)/$(SUBTARGET)/uboot-$(BOARD)/prebuilt

include $(INCLUDE_DIR)/package.mk
PKG_DEVICE_DIR := $(PKG_BUILD_DIR)/NAS/$(DEVICE)

define Package/rtk_tee_prebuilt
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Prebuilt Binaries for TEE and CA/TA
  DEPENDS:=@TARGET_rtd1xxx_rtd1619b
endef

define Package/rtk_tee_prebuilt/description
 Realtek prebuilt binaries for TEE, CA and TA usage.
endef

define Build/Configure
endef

define Build/Compile
endef

define Build/InstallDev
endef

ifneq ($(DEVICE),realtek)
define Build/Clean
	rm -rf $(IMAGE_FILES_DIR)
	rm -rf $(IMAGE_TEE_DIR)
endef
endif

define Package/rtk_tee_prebuilt/install-files
	$(CP) $(PKG_DEVICE_DIR)/src/* $(1)/
	[ ! -d $(PKG_DEVICE_DIR)/files ] ||\
	    (\
	    mkdir -p $(IMAGE_FILES_DIR) && \
	    $(CP) $(PKG_DEVICE_DIR)/files/* $(IMAGE_FILES_DIR) \
	    )
	[ ! -d $(PKG_DEVICE_DIR)/tee ] || \
	    (\
	    mkdir -p $(IMAGE_TEE_DIR) && \
	    $(CP) $(PKG_DEVICE_DIR)/tee/* $(IMAGE_TEE_DIR) \
	    )
endef

define Package/rtk_tee_prebuilt/install
	$(Package/rtk_tee_prebuilt/install-files)
endef

$(eval $(call BuildPackage,rtk_tee_prebuilt))
