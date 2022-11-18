#
# Makefile for HDMITX linux kernel module.
#
ccflags-y += -I$(srctree)/drivers/video/fbdev/rtk
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/mem_allocator/rtk/inc
ccflags-y += -I$(src) # needed for trace events

hdmitx-$(CONFIG_RTK_HDMITX):= hdmitx_core.o switch_hdmitx.o hdmitx_api.o rtk_edid.o compat_hdmitx.o hdmitx_info.o \
	hdmitx_scdc.o rtk_edid_filter.o rtk_mute_gpio.o frl_link_training.o hdmitx_config.o

hdmitx-$(CONFIG_RTK_HDMI_RTD13XX) += frl_phy_rtd13xx.o
hdmitx-$(CONFIG_RTK_HDMI_RTD16XXB) += frl_phy_rtd16xxb.o

obj-$(CONFIG_RTK_HDMITX)+= hdmitx.o

