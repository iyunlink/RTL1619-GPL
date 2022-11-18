# SPDX-License-Identifier: GPL-2.0

ccflags-y += -I$(srctree)/drivers/video/fbdev/rtk
#ccflags-y += -I$(srctree)/drivers/staging/android
#ccflags-y += -I$(srctree)/drivers/staging/android/ion/rtk/inc
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/mem_allocator/rtk/inc

rtk_drm-y := rtk_drm_rpc.o \
			rtk_drm_plane.o \
			rtk_drm_crtc.o \
			rtk_drm_vo.o \
			rtk_drm_gem.o \
			rtk_drm_fb.o \
			rtk_drm_drv.o \
			rtk_hdmi.o \
			rtk_hdmi_legacy.o \
			rtk_hdmi_edid.o \
			rtk_dptx_core.o \
			rtk_dptx_reg.o	\
			rtk_hdcp.o      \
			rtk_hdcp1_tee.o \
			rtk_hdcp2_tee.o

obj-$(CONFIG_DRM_RTK) += rtk_drm.o

