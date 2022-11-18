ccflags-y += -I$(src)/include -I$(srctree)/drivers/dma
obj-$(CONFIG_RTK_HSE) += rtk-hse.o

rtk-hse-y += platform.o
rtk-hse-y += engine.o
rtk-hse-y += cq.o
rtk-hse-y += cmd.o
rtk-hse-y += device.o
rtk-hse-$(CONFIG_RTK_HSE_DMA) += dma.o
