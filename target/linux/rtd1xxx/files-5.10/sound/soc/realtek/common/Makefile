#
# Copyright (C) 2020 RealTek Inc.
#
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/mem_allocator/rtk/inc
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/rpc

# platform driver
obj-$(CONFIG_SND_SOC_REALTEK) += snd-soc-realtek.o
snd-soc-realtek-y += snd-realtek.o snd-realtek-compress.o snd-realtek_rpc.o

obj-$(CONFIG_SND_SOC_REALTEK_NOTIFY) += snd-realtek-notify.o
