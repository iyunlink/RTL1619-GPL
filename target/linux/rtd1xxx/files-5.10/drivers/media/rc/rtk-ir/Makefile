# SPDX-License-Identifier: GPL-2.0
rtk-ir-y			:= rtk-ir-core.o \
					rtk-ir-nec.o \
					rtk-ir-rstep.o \
					rtk-ir-xmp.o \
					rtk-ir-sky.o
rtk-ir-$(CONFIG_IR_RTK_RAW)	+= rtk-ir-raw.o
rtk-ir-$(CONFIG_IR_RTK_HW)	+= rtk-ir-hw.o
rtk-ir-$(CONFIG_IR_RTK_HW1)	+= rtk-ir-hw1.o

obj-$(CONFIG_IR_RTK)		+= rtk-ir.o
