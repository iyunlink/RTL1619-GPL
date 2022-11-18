tpdemux-$(CONFIG_RTK_TPDEMUX):= tpdemux_core.o tpdemux_common.o tpdemux_reg.o tpdemux_buffer.o

obj-$(CONFIG_RTK_TPDEMUX)+= tpdemux.o

ccflags-y += -I$(srctree)/drivers/media/i2c
ccflags-y += -I$(srctree)/drivers/media/common
ccflags-y += -I$(srctree)/drivers/media/dvb-core
ccflags-y += -I$(srctree)/drivers/staging/android
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/mem_allocator/rtk/inc
