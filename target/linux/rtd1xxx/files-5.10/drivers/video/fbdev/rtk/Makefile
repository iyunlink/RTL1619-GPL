// SPDX-License-Identifier: GPL-2.0-only
ccflags-y += -I$(srctree)/drivers/dma-buf
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/rpc
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/
ccflags-y += -I$(srctree)/drivers/soc/realtek/common/mem_allocator/rtk/inc

obj-$(CONFIG_FB_RTK) += rtk-fb.o bootlogo.o
rtk-fb-y += rtk_fb.o rtk_fb_rpc.o dc2vo.o dc_util.o
