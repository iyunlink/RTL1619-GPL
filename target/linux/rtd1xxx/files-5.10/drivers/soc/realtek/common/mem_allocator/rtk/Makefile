ccflags-y += -I$(srctree)/drivers/soc/realtek/common/mem_allocator
ccflags-y += -I$(srctree)/mm

obj-$(CONFIG_ION_RTK_DHC_HEAP) += ion_rtk_heap.o
ion_rtk_heap-y += src/pool_common.o
ion_rtk_heap-y += src/pool_gen.o
ion_rtk_heap-y += src/pool_cma.o
ion_rtk_heap-y += src/carveout_heap.o
ion_rtk_heap-y += src/dev.o
ion_rtk_heap-y += src/ioctl.o
ion_rtk_heap-y += src/protected.o
ion_rtk_heap-y += src/debugfs.o

obj-$(CONFIG_ION_RTK_LEGACY_DEVICE) += src/dev_legacy.o

obj-$(CONFIG_ION_RTK_DHC_HEAP) += test/
