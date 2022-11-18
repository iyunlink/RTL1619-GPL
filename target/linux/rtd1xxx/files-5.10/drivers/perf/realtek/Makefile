#
# Makefile for the Realtek bus drivers.
#

obj-$(CONFIG_RTK_PMU)		+= rtk_uncore_pmu.o
obj-$(CONFIG_RTK_DBUS_PMU)	+= dbus_pmu.o
obj-$(CONFIG_RTK_DDRC_PMU)	+= ddrc_pmu.o
obj-$(CONFIG_RTK_RBUS_PMU)	+= rbus_pmu.o
obj-$(CONFIG_RTK_TFC_PMU)	+= tfc_pmu.o
obj-$(CONFIG_RTK_RXI316_PMU) 	+= rtk_rxi316_pmu.o

dbus_pmu-y	:= rtk_dbus_pmu.o dbus_pmu_16xxb.o
ddrc_pmu-y	:= rtk_ddrc_pmu.o ddrc_pmu_16xxb.o
rbus_pmu-y	:= rtk_rbus_pmu.o rbus_pmu_16xxb.o
tfc_pmu-y	:= rtk_tfc_pmu.o tfc_pmu_16xxb_ddrc.o
rtk_rxi316_pmu-y := rxi316_pmu.o
