ccflags-$(CONFIG_REALTEK_RPC)	+= -I$(srctree)/drivers/soc/realtek/common
ccflags-$(CONFIG_REALTEK_RPC)	+= -I$(srctree)/drivers/soc/realtek/common/include
ccflags-y 			+= -I$(srctree)/drivers/video/fbdev/rtk
ccflags-y 			+= -I$(srctree)/drivers/soc/realtek/common/mem_allocator/rtk/inc

obj-$(CONFIG_REALTEK_RPC)	+= rtk-rpc.o
rtk-rpc-y			+= rtk_rpc_intr.o rtk_rpc_kern.o rtk_rpc_poll.o rtk_rpc.o rpc_mem.o compat_rpc_mem.o
