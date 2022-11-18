MODULE_NAME := buflock

ccflags-y += -I$(srctree)/drivers/soc/realtek/common/buflock/uapi
ccflags-y += -DMODULE_NAME='"$(MODULE_NAME)"'
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/mem_allocator/rtk/inc

obj-$(CONFIG_RTK_BUFLOCK) += src/buflock.o
