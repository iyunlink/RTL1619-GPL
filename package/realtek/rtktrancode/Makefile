#
# Copyright (C) 2017-2020 Ian Leonard <antonlacon@gmail.com>
# Copyright (C) 2018 Ted Hess <thess@kitschensync.net>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=rtktranscode
PKG_VERSION:=4.1.8
PKG_RELEASE:=3

PKG_VERSION:=33d1b29
PKG_SOURCE_URL:=https://gitee.com/mpai/easy_porting_ffmpeg_in_mit
PKG_SOURCE_BRANCH:=master

PKG_SOURCE_PROTO:=git
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE_VERSION:=$(PKG_VERSION)
#PKG_MIRROR_HASH:=skip
PKG_MIRROR_HASH:=edd90562cae4f34d83458fc4daa8474558ec67ac42acd050aa079e71cbbfd39f
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.gz

PREBUILT_DIR:=$(SUBTARGET)_files
PATCH_DIR:=./$(SUBTARGET)_patches

include $(INCLUDE_DIR)/package.mk

define Package/rtktranscode
 TITLE:=Realtek Transcode Application
 SECTION:=multimedia
 CATEGORY:=Multimedia
 URL:=https://gitee.com/mpai/easy_porting_ffmpeg_in_mit
 DEPENDS+= +libbz2 +alsa-lib +libgnutls +libpthread +libvpx +zlib  +libstdcpp  @TARGET_rtd1xxx_rtd1619b
endef

define Package/rtktranscode/description
 Realtek Transcode application is a a software package that can convert video in numerous formats.
endef


# Strip off FPU notation
REAL_CPU_TYPE:=$(firstword $(subst +, ,$(CONFIG_CPU_TYPE)))
# Fixup cpu types recogized in configure
REAL_CPU_TYPE:=$(subst octeonplus,octeon+,$(REAL_CPU_TYPE))

TARGET_CFLAGS += -I$(STAGING_DIR)/usr/local/include
TARGET_LDFLAGS += -L$(STAGING_DIR)/usr/local/lib -lRMA -Wl,-rpath-link,$(STAGING_DIR)/usr/local/lib

FFMPEG_CONFIGURE:= \
	CFLAGS="$(TARGET_CFLAGS) $(TARGET_CPPFLAGS) $(FPIC)" \
	LDFLAGS="$(TARGET_LDFLAGS)" \
	./configure \
	--enable-cross-compile \
	--cross-prefix="$(TARGET_CROSS)" \
	--arch="$(ARCH)" \
	$(if $(REAL_CPU_TYPE),--cpu=$(call qstrip,$(REAL_CPU_TYPE)),) \
	--target-os=linux \
	--prefix="/usr" \
	--pkg-config="pkg-config" \
	--enable-shared \
	--enable-static \
	--enable-pthreads \
	--enable-zlib \
	--disable-doc \
	--disable-debug \
	--enable-libvpx \
	--disable-lzma \
	--disable-vaapi \
	--disable-vdpau \
	--disable-outdevs \
	--disable-runtime-cpudetect \
	--enable-lto \
	--enable-neon \
	--enable-vfp


FFMPEG_CONFIGURE+= \
	--enable-omx \
	--enable-rma \
	--enable-decoder=h264_rma \
	--enable-decoder=mpeg4_rma \
	--enable-decoder=hevc_rma \
	--enable-decoder=mpeg1_rma \
	--enable-decoder=mpeg2_rma \
	--enable-decoder=vp8_rma \
	--enable-decoder=vp9_rma \
	--enable-decoder=vc1_rma \
	--enable-decoder=wmv3_rma \
	--enable-decoder=mjpeg_rma \
	--enable-decoder=h263_rma \
	--enable-decoder=avs_rma \
	--enable-decoder=cavs_rma \
	--enable-decoder=flv_rma \
	--enable-decoder=av1_rma \
	--enable-encoder=h265_omx \
	--enable-encoder=h264_omx

FFMPEG_CONFIGURE+= \
	--enable-gnutls \
	--enable-small

define Build/Prepare
	$(Build/Prepare/Default)
	$(INSTALL_DIR) $(STAGING_DIR)/usr/lib
	$(INSTALL_DIR) $(STAGING_DIR)/usr/include
	$(CP) ./$(PREBUILT_DIR)/lib/librma.so $(STAGING_DIR)/usr/lib
	$(CP) ./$(PREBUILT_DIR)/lib/libRMA.so $(STAGING_DIR)/usr/lib
	$(CP) ./$(PREBUILT_DIR)/lib/libhwse*.so $(STAGING_DIR)/usr/lib
	$(CP) ./$(PREBUILT_DIR)/lib/libgeneric_misc.so $(STAGING_DIR)/usr/lib
	$(CP) ./$(PREBUILT_DIR)/include/* $(STAGING_DIR)/usr/include
endef

define Build/Configure
	( cd $(PKG_BUILD_DIR); $(FFMPEG_CONFIGURE) )
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		DESTDIR="$(PKG_INSTALL_DIR)" \
		all install
endef

# Only libx264 is GPL (yes libpostproc); all other builds are lgpl (no libpostproc)
define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include
	$(INSTALL_DIR) $(1)/usr/lib
	$(INSTALL_DIR) $(1)/usr/lib/pkgconfig
	$(CP) $(PKG_INSTALL_DIR)/usr/include/lib{avcodec,avdevice,avfilter,avformat,avutil,swresample,swscale} $(1)/usr/include/
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/lib{avcodec,avdevice,avfilter,avformat,avutil,swresample,swscale}.{a,so*} $(1)/usr/lib/
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/pkgconfig/lib{avcodec,avdevice,avfilter,avformat,avutil,swresample,swscale}.pc $(1)/usr/lib/pkgconfig/
endef

define Package/rtktranscode/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_DIR) $(1)/usr/lib
	$(INSTALL_DIR) $(1)/usr/local
	$(CP) $(PKG_INSTALL_DIR)/usr/bin/ffmpeg $(1)/usr/bin/
	$(CP) $(PKG_INSTALL_DIR)/usr/bin/ffprobe $(1)/usr/bin/
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/lib{avcodec,avdevice,avfilter,avformat,avutil,swresample,swscale}.so.* $(1)/usr/lib/
	$(CP) ./$(PREBUILT_DIR)/* $(1)/usr/local/
endef

$(eval $(call BuildPackage,rtktranscode))
