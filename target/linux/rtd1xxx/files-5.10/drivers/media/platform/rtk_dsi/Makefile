#
# Makefile for CEC linux kernel module.
#

ccflags-y += -I$(srctree)/drivers/video/fbdev/rtk
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/mem_allocator/rtk/inc
ccflags-y += -I$(src) # needed for trace events

obj-$(CONFIG_RTK_DSI)  += rtk_dsi.o
