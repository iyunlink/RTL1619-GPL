obj-$(CONFIG_RTK_MEM_REMAP) += rtk_memory_remap.o
obj-y += rtk_crt.o
obj-$(CONFIG_RTK_REFCLK) += rtk_refclk.o
obj-y += rtk_sb2.o
obj-y += rtk_sb2_dbg.o
obj-y += rtk_sb2_inv.o
obj-y += rtk_sb2_sem.o
obj-y += dvfs/
obj-y += rtk_sc_wrap.o
obj-$(CONFIG_RTK_FSS) += rtk_fss.o
obj-$(CONFIG_RTK_FSS_SCAN) += rtk_fss_scan.o
obj-$(CONFIG_REALTEK_CHIP_INFO) += chip.o
obj-$(CONFIG_REALTEK_RPC) += rpc/
obj-$(CONFIG_RTK_MCP) += rtk_mcp.o rtk_sha1.o
obj-$(CONFIG_RTK_SMCC) += rtk_smcc.o
obj-$(CONFIG_PD_REALTEK) += rtk_pd/
obj-$(CONFIG_RTK_HSE) += hse/
rtk-usb-manager-y += rtk_usb_manager.o
rtk-usb-manager-y += rtk_usb_rtd119x.o
rtk-usb-manager-y += rtk_usb_rtd129x.o
rtk-usb-manager-y += rtk_usb_rtd139x.o
rtk-usb-manager-y += rtk_usb_rtd16xx.o
rtk-usb-manager-y += rtk_usb_rtd13xx.o
rtk-usb-manager-y += rtk_usb_rtd16xxb.o
rtk-usb-manager-y += rtk_usb_rtd1312c.o
rtk-usb-manager-y += rtk_usb_rtd13xxd.o
obj-$(CONFIG_RTK_USB_CTRL_MANAGER) += rtk-usb-manager.o
obj-$(CONFIG_RTK_FAN) += rtk_fan.o
obj-$(CONFIG_ION) += mem_allocator/
obj-$(CONFIG_RTK_BUFLOCK) += buflock/
obj-$(CONFIG_REALTEK_PM)  += rtk_pm_suspend.o
rtk_pm_suspend-y += rtk_pm_common.o rtk_pm.o rtk_pm_sysfs.o
obj-$(CONFIG_RTK_PM_ALARM) += rtk_pm_alarm.o
CFLAGS_rtk_pm_hifi.o = -I$(srctree)/drivers/regulator
obj-$(CONFIG_RTK_PM_HIFI) += rtk_pm_hifi.o
obj-y += rtk_bootstatus.o
obj-$(CONFIG_RTK_TP) += rtk_tp.o
obj-$(CONFIG_RTK_RTC_RELOAD) += rtk_rtc_reload.o
obj-$(CONFIG_RTK_VCPU) += rtk_vcpu.o
obj-$(CONFIG_RTK_VE3_UART) += rtk_ve3_uart.o
obj-$(CONFIG_RTK_GPIO_DEFAULT) += rtk_gpio_default.o
obj-$(CONFIG_RTK_DPI_COORD) += dpi/
obj-$(CONFIG_RTK_VSFC_CTRL) += rtk_vsfc_ctrl.o
obj-$(CONFIG_RTK_CPUHP_QOS) += rtk_cpuhp.o
obj-$(CONFIG_RTK_CPUHP_CONTROL) += rtk_cpuhp_ctrl.o
obj-$(CONFIG_RTK_CPUHP_SYSFS) += rtk_cpuhp_sysfs.o
obj-$(CONFIG_RTK_ACPU_VE1) += rtk_acpu_ve1.o
obj-y += rtk_iso_wa.o
obj-$(CONFIG_RTK_BSV_CTRL) += rtk_bsv_ctrl.o
obj-$(CONFIG_RTK_DBGPROT) += rtk_dbgprot.o
obj-$(CONFIG_RTK_DCO) += rtk_dco.o
obj-$(CONFIG_RTK_AVCPU_LOG) += rtk_avcpulog.o
obj-$(CONFIG_RTK_IFCP_CLIENT) += rtk_ifcp_client.o
obj-$(CONFIG_SMARTCARD) += rtk_scd/
obj-$(CONFIG_RTK_DEMOD) += rtk_demod.o
ifeq ($(CONFIG_RTK_MDL), y)
obj-y += rtk_mdl/
endif
obj-$(CONFIG_RTK_PUWRAP) += puwrap/
obj-$(CONFIG_RTK_IMAGE_CODEC) += jdi/
obj-y += gpu/
