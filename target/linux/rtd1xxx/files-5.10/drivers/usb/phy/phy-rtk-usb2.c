// SPDX-License-Identifier: GPL-2.0
/*
 *  phy-rtk-usb2.c RTK usb2.0 PHY driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/nvmem-consumer.h>
#include <linux/sys_soc.h>
#include <linux/usb/phy.h>
#include <linux/usb/hcd.h>

#include "phy-rtk-usb.h"

#define RTK_USB2PHY_NAME "rtk-usb2phy"

#define OFFEST_PHY_READ	0x20

#define USB_ST_BUSY		BIT(23)
#define MAX_PHY_DATA_SIZE 20

#define EFUS_USB_DC_CAL_RATE 2
#define EFUS_USB_DC_CAL_MAX 7
#define USB_DC_CAL_MASK 0x1F

#define EFUS_USB_DC_DIS_RATE 1
#define EFUS_USB_DC_DIS_MAX 7
#define USB_DC_DIS_MASK 0xF

#define PAGE_START 0xE0
#define PAGE0_0xE4 0xE4
#define PAGE0_0xE7 0xE7
#define PAGE1_0xE0 0xE0
#define PAGE1_0xE2 0xE2

/* mapping 0xE0 to 0 ... 0xE7 to 7, 0xF0 to 8 ,,, 0xF7 to 15 */
#define PAGE_ADDR_MAP_ARRAY_INDEX(addr) \
	(((addr - PAGE_START)&0x7) + \
	(((addr - PAGE_START)&0x10)>>1))
#define ARRAY_INDEX_MAP_PAGE_ADDR(index) \
	(((index + PAGE_START)&0x7) + \
	(((index&0x8)<<1) + PAGE_START))

struct reg_addr {
	void __iomem *REG_WRAP_VStatusOut2;
	void __iomem *REG_GUSB2PHYACC0;
	int vstatus_index;
};

struct phy_parameter {
	u8 addr;
	u8 data;
};

struct phy_data {
	int page0_size;
	struct phy_parameter *page0;
	int page1_size;
	struct phy_parameter *page1;
	int page2_size;
	struct phy_parameter *page2;

	bool check_efuse;
	int8_t efuse_usb_dc_cal;
	int efuse_usb_dc_cal_rate;
	int usb_dc_cal_mask;
	int8_t efuse_usb_dc_dis;
	int efuse_usb_dc_dis_rate;
	int usb_dc_dis_mask;
	bool usb_dc_dis_at_page0;
	bool do_toggle;
	bool do_toggle_driving;
	int disconnect_driving_updated;
	bool use_default_parameter;
	bool is_double_sensitivity_mode;
	bool ldo_force_enable;
	bool ldo_enable;
	s32 ldo_page0_e4_compensate;
};

static char rtk_usb_phy_read(struct reg_addr *regAddr, char addr)
{
	unsigned int regVal;
	void __iomem *REG_GUSB2PHYACC0 = regAddr->REG_GUSB2PHYACC0;
	int ret = 0;

	addr -= OFFEST_PHY_READ;

	// polling until VBusy == 0
	ret = utmi_wait_register(REG_GUSB2PHYACC0, USB_ST_BUSY, 0);
	if (ret)
		return (char)ret;

	// VCtrl = low nibble of addr, VLoadM = 1
	regVal = BIT(25) | // vload
			 ((addr & 0x0f) << 8);	// vcontrol
	phy_write(REG_GUSB2PHYACC0, regVal);
	ret = utmi_wait_register(REG_GUSB2PHYACC0, USB_ST_BUSY, 0);
	if (ret)
		return (char)ret;

	// VCtrl = high nibble of addr, VLoadM = 1
	regVal = BIT(25) | // vload
			 ((addr & 0xf0) << 4);	// vcontrol
	phy_write(REG_GUSB2PHYACC0, regVal);
	ret = utmi_wait_register(REG_GUSB2PHYACC0, USB_ST_BUSY, 0);
	if (ret)
		return (char)ret;

	/* rmb  for reg read */
	smp_rmb();
	regVal = phy_read(REG_GUSB2PHYACC0);

	return (char) (regVal & 0xff);
}

static int rtk_usb_phy_write(struct reg_addr *regAddr, char addr, char data)
{
	unsigned int regVal;
	void __iomem *REG_WRAP_VStatusOut2 = regAddr->REG_WRAP_VStatusOut2;
	void __iomem *REG_GUSB2PHYACC0 = regAddr->REG_GUSB2PHYACC0;
	int shift_bits = regAddr->vstatus_index * 8;
	int ret = 0;

	//write data to VStatusOut2 (data output to phy)
	phy_write(REG_WRAP_VStatusOut2, (u32)data<<shift_bits);
	ret = utmi_wait_register(REG_GUSB2PHYACC0, USB_ST_BUSY, 0);
	if (ret)
		return ret;

	// VCtrl = low nibble of addr, VLoadM = 1
	regVal = BIT(25) |
			 ((addr & 0x0f) << 8);

	phy_write(REG_GUSB2PHYACC0, regVal);
	ret = utmi_wait_register(REG_GUSB2PHYACC0, USB_ST_BUSY, 0);
	if (ret)
		return ret;

	// VCtrl = high nibble of addr, VLoadM = 1
	regVal = BIT(25) |
			 ((addr & 0xf0) << 4);

	phy_write(REG_GUSB2PHYACC0, regVal);
	ret = utmi_wait_register(REG_GUSB2PHYACC0, USB_ST_BUSY, 0);
	if (ret)
		return ret;

	return 0;
}

static int rtk_usb_phy_set_page(struct reg_addr *regAddr, int page)
{
#define SET_PAGE_OFFSET 0xf4
#define SET_PAGE_0 0x9b
#define SET_PAGE_1 0xbb
#define SET_PAGE_2 0xdb

	switch (page) {
	case 0:
		return rtk_usb_phy_write(regAddr, SET_PAGE_OFFSET, SET_PAGE_0);
	case 1:
		return rtk_usb_phy_write(regAddr, SET_PAGE_OFFSET, SET_PAGE_1);
	case 2:
		return rtk_usb_phy_write(regAddr, SET_PAGE_OFFSET, SET_PAGE_2);
	default:
		pr_err("%s error page=%d\n", __func__, page);
	}

	return -1;
}

#ifdef CONFIG_USB_PATCH_ON_RTK

#define USB_CTRL 0x98007FB0
#define ISO_USB_U2PHY_REG_LDO_PW (BIT(20) | BIT(21) | BIT(22) | BIT(23))

static int check_phy_power(struct phy_data *phy_data, struct reg_addr *regAddr)
{
	volatile unsigned int regVal;
	void __iomem *REG_GUSB2PHYACC0 = regAddr->REG_GUSB2PHYACC0;
	u8 addr = 0xF4;
	u8 read_addr = addr - OFFEST_PHY_READ;
	int use_ldo = 0;
	volatile unsigned int val;
	void __iomem *reg_usb_ctrl = ioremap(USB_CTRL, 0x4);

	if (!reg_usb_ctrl) {
		pr_err("%s USB_CTRL ioremap fail\n", __func__);
		return use_ldo;
	}

	val = readl(reg_usb_ctrl);
	if ((val & ISO_USB_U2PHY_REG_LDO_PW) == ISO_USB_U2PHY_REG_LDO_PW) {
		pr_notice("%s phy use ldo power! (USB_CTRL val=0x%x)\n",
			    __func__, val);
		use_ldo = 1;
		goto out;
	}

	// VCtrl = low nibble of addr, VLoadM = 1
	regVal = BIT(25) | // vload
			 ((read_addr & 0x0f) << 8);	// vcontrol
	phy_write(REG_GUSB2PHYACC0, regVal);
	mdelay(1);

	// VCtrl = high nibble of addr, VLoadM = 1
	regVal = BIT(25) | // vload
			 ((read_addr & 0xf0) << 4);	// vcontrol
	phy_write(REG_GUSB2PHYACC0, regVal);
	mdelay(1);

	/* rmb  for reg read */
	smp_rmb();
	regVal = phy_read(REG_GUSB2PHYACC0);

	if ((regVal & 0xf) == 0x0 || phy_data->ldo_force_enable) {
		val = readl(reg_usb_ctrl);
		val |= ISO_USB_U2PHY_REG_LDO_PW;
		writel(val, reg_usb_ctrl);
		use_ldo = 1;

		mdelay(1);
		phy_write(REG_GUSB2PHYACC0, BIT(25));

		pr_notice("%s phy %s then turn on ldo! USB_CTRL val=0x%x\n",
			     __func__,
			    phy_data->ldo_force_enable?
			        " ldo_force_enable":"no power",
			    val);
		pr_notice("check phy regVal=0x%x ==> regVal=0x%x\n",
			    regVal & 0xf, rtk_usb_phy_read(regAddr, addr));
	}

out:
	iounmap(reg_usb_ctrl);
	return use_ldo;
}
#endif // CONFIG_USB_PATCH_ON_RTK

static void rtk_usb2_phy_shutdown(struct usb_phy *phy)
{
	/* Todo */
}

static int __get_phy_parameter_by_efuse(struct rtk_usb_phy_s *rtk_phy,
	    struct phy_data *phy_data, int index)
{
	u8 value = 0;
	struct nvmem_cell *cell;
	int dc_cal_rate = phy_data->efuse_usb_dc_cal_rate;
	int rate;
	struct soc_device_attribute rtk_soc_groot[] = {
			{ .family = "Realtek Groot",},
			{ /* empty */ }
		};

	cell = nvmem_cell_get(rtk_phy->dev, "usb-dc-cal");
	if (IS_ERR(cell)) {
		dev_warn(rtk_phy->dev, "%s failed to get usb-dc-cal: %ld\n",
			    __func__, PTR_ERR(cell));
	} else {
		unsigned char *buf;
		size_t buf_size;

		buf = nvmem_cell_read(cell, &buf_size);

		value = buf[0] & phy_data->usb_dc_cal_mask;

		dev_dbg(rtk_phy->dev,
			    "buf=0x%x buf_size=%d value=0x%x\n",
			    buf[0], (int)buf_size, value);

		kfree(buf);
		nvmem_cell_put(cell);
	}

	if (value <= EFUS_USB_DC_CAL_MAX)
		phy_data->efuse_usb_dc_cal = (int8_t)(value * dc_cal_rate);
	else
		phy_data->efuse_usb_dc_cal = -(int8_t)(
			    (EFUS_USB_DC_CAL_MAX & value) * dc_cal_rate);

	if (soc_device_match(rtk_soc_groot)) {
		dev_info(rtk_phy->dev, "For groot IC we need a workaround to adjust efuse_usb_dc_cal\n");

		/* We don't multiple dc_cal_rate=2 for positive dc cal compensate */
		if (value <= EFUS_USB_DC_CAL_MAX)
			phy_data->efuse_usb_dc_cal = (int8_t)(value);

		/* We set max dc cal compensate is 0x8 if otp is 0x7 */
		if (value == 0x7)
			phy_data->efuse_usb_dc_cal = (int8_t)(value + 1);
	}

	dev_info(rtk_phy->dev, "Get Efuse usb_dc_cal=%d for index=%d value=%x\n",
		    phy_data->efuse_usb_dc_cal, index, value);

	/* Read efuse for usb dc disconnect level */
	value = 0;
	cell = nvmem_cell_get(rtk_phy->dev, "usb-dc-dis");
	if (IS_ERR(cell)) {
		dev_warn(rtk_phy->dev, "%s failed to get usb-dc-dis: %ld\n",
			    __func__, PTR_ERR(cell));
	} else {
		unsigned char *buf;
		size_t buf_size;

		buf = nvmem_cell_read(cell, &buf_size);

		value = buf[0] & phy_data->usb_dc_dis_mask;

		dev_dbg(rtk_phy->dev,
			    "buf=0x%x buf_size=%d value=0x%x\n",
			    buf[0], (int)buf_size, value);

		kfree(buf);
		nvmem_cell_put(cell);
	}

	rate = phy_data->efuse_usb_dc_dis_rate;

	if (value <= EFUS_USB_DC_DIS_MAX)
		phy_data->efuse_usb_dc_dis = (int8_t)(value * rate);
	else
		phy_data->efuse_usb_dc_dis = -(int8_t)(
			    (EFUS_USB_DC_DIS_MAX & value) * rate);

	dev_info(rtk_phy->dev, "Get Efuse usb_dc_dis=%d for index=%d value=%x\n",
		    phy_data->efuse_usb_dc_dis, index, value);

	return 0;
}

/* Get default phy parameter for update by efuse or ldo_page0_e4_compensate*/
static int __get_default_phy_parameter_for_updated(
	    struct rtk_usb_phy_s *rtk_phy, int index)
{
	int i;
	struct reg_addr *regAddr;
	struct phy_data *phy_data;
	struct phy_parameter *phy_page_setting;

	regAddr = &((struct reg_addr *)rtk_phy->reg_addr)[index];
	phy_data = &((struct phy_data *)rtk_phy->phy_data)[index];

	/* Get PAGE0_0xE4 default value */
	if (phy_data->efuse_usb_dc_cal || phy_data->ldo_page0_e4_compensate ||
		    (phy_data->efuse_usb_dc_dis && phy_data->usb_dc_dis_at_page0)) {
		phy_page_setting = phy_data->page0;
		rtk_usb_phy_set_page(regAddr, 0);

		i = PAGE_ADDR_MAP_ARRAY_INDEX(PAGE0_0xE4);
		if (i < phy_data->page0_size) {
			u8 addr = (phy_page_setting + i)->addr;
			u8 data = (phy_page_setting + i)->data;

			if (!addr) {
				addr = ARRAY_INDEX_MAP_PAGE_ADDR(i);
				data = rtk_usb_phy_read(regAddr, addr);

				(phy_page_setting + i)->addr = addr;
				(phy_page_setting + i)->data = data;
				dev_info(rtk_phy->dev,
					    "Get default addr %x value %x\n",
					    (phy_page_setting + i)->addr,
					    (phy_page_setting + i)->data);
			}
		}
	}

	/* Get PAGE1_0xE2 default value */
	if (phy_data->efuse_usb_dc_dis && !phy_data->usb_dc_dis_at_page0) {
		phy_page_setting = phy_data->page1;
		rtk_usb_phy_set_page(regAddr, 1);

		i = PAGE_ADDR_MAP_ARRAY_INDEX(PAGE1_0xE2);
		if (i < phy_data->page1_size) {
			u8 addr = (phy_page_setting + i)->addr;
			u8 data = (phy_page_setting + i)->data;

			if (!addr) {
				addr = ARRAY_INDEX_MAP_PAGE_ADDR(i);
				data = rtk_usb_phy_read(regAddr, addr);

				(phy_page_setting + i)->addr = addr;
				(phy_page_setting + i)->data = data;
				dev_info(rtk_phy->dev,
					    "Get default page1 addr %x value %x\n",
					    (phy_page_setting + i)->addr,
					    (phy_page_setting + i)->data);
			}
		}
	}

	return 0;
}

static u8 __updated_page0_0xe4_parameter(struct phy_data *phy_data, u8 data)
{
	u8 val;
	s32 __val;
	s32 ldo_page0_e4_compensate = 0;
	s32 usb_dc_cal_mask = phy_data->usb_dc_cal_mask;

	if (phy_data->ldo_enable)
		ldo_page0_e4_compensate = phy_data->ldo_page0_e4_compensate;

	__val = (s32)(data & usb_dc_cal_mask) + ldo_page0_e4_compensate
		    + phy_data->efuse_usb_dc_cal;

	if (__val > usb_dc_cal_mask)
		__val = usb_dc_cal_mask;
	else if (__val < 0)
		__val = 0;

	val = (data & (~usb_dc_cal_mask)) | (__val & usb_dc_cal_mask);

	return val;
}

static u8 __updated_dc_disconnect_level_page0_0xe4(struct phy_data *phy_data,
	    u8 data)
{
	u8 val;
	s32 __val;
	s32 usb_dc_dis_mask = phy_data->usb_dc_dis_mask;
	int offset = 4;

	__val = (s32)((data >> offset) & usb_dc_dis_mask)
		     + phy_data->efuse_usb_dc_dis;

	if (__val > usb_dc_dis_mask)
		__val = usb_dc_dis_mask;
	else if (__val < 0)
		__val = 0;

	val = (data & (~(usb_dc_dis_mask << offset))) |
		    (__val & usb_dc_dis_mask) << offset;

	return val;
}

static u8 __updated_dc_disconnect_level_page1_0xe2(struct phy_data *phy_data,
	    u8 data)
{
	u8 val;
	s32 __val;
	s32 usb_dc_dis_mask = phy_data->usb_dc_dis_mask;

	if (phy_data->usb_dc_dis_at_page0)
		return data;

	__val = (s32)(data & usb_dc_dis_mask)
		    + phy_data->efuse_usb_dc_dis;

	if (__val > usb_dc_dis_mask)
		__val = usb_dc_dis_mask;
	else if (__val < 0)
		__val = 0;

	val = (data & (~usb_dc_dis_mask)) | (__val & usb_dc_dis_mask);

	return val;
}

static void update_dc_disconnect_level(struct rtk_usb_phy_s *rtk_phy,
	    struct reg_addr *regAddr,
	    struct phy_data *phy_data, bool isUpdate)
{
	struct phy_parameter *phy_page_setting;
	int i;

	if (!phy_data->usb_dc_dis_at_page0)
		goto update_page1_0xe2;

	/* updated disconnect level at page0 0xe4 */

	/* Set page 0 */
	phy_page_setting = phy_data->page0;
	rtk_usb_phy_set_page(regAddr, 0);

	i = PAGE_ADDR_MAP_ARRAY_INDEX(PAGE0_0xE4);

	if (i < phy_data->page0_size) {
		u8 addr = (phy_page_setting + i)->addr;
		u8 data = (phy_page_setting + i)->data;
		u8 __data;
		int offset = 4;
		s32 usb_dc_dis_mask = phy_data->usb_dc_dis_mask;

		if (!addr) {
			addr = ARRAY_INDEX_MAP_PAGE_ADDR(i);
			data = rtk_usb_phy_read(regAddr, addr);

			(phy_page_setting + i)->addr = addr;
			(phy_page_setting + i)->data = data;
			dev_dbg(rtk_phy->dev,
				    "Get default addr %x value %x\n",
				    (phy_page_setting + i)->addr,
				    (phy_page_setting + i)->data);
		}
		__data = rtk_usb_phy_read(regAddr, addr);

		/* keep default dc dis and real dc cal */
		data = (data & ((usb_dc_dis_mask << offset))) |
			    (__data & (~(usb_dc_dis_mask << offset)));

		if (isUpdate)
			data = __updated_dc_disconnect_level_page0_0xe4(phy_data, data);

		if (rtk_usb_phy_write(regAddr, addr, data)) {
			dev_err(rtk_phy->dev,
				    "[%s:%d] Error page1 addr=0x%x value=0x%x\n",
				    __func__, __LINE__,
				    addr, data);
			return;
		}

		dev_info(rtk_phy->dev,
			    "%s to set Page0 0xE4=%x for dc disconnect level (%s)\n",
			    __func__,
			    rtk_usb_phy_read(regAddr, addr),
			    isUpdate?"Update":"restore");
	} else {
		dev_err(rtk_phy->dev,
			    "ERROR: %s %d index=%d addr Not PAGE0_0xE4\n",
			    __func__, __LINE__, i);
	}

	return;

update_page1_0xe2:

	/* Set page 1 */
	phy_page_setting = phy_data->page1;
	rtk_usb_phy_set_page(regAddr, 1);

	i = PAGE_ADDR_MAP_ARRAY_INDEX(PAGE1_0xE2);

	if (i < phy_data->page1_size) {
		u8 addr = (phy_page_setting + i)->addr;
		u8 data = (phy_page_setting + i)->data;
		u8 __data;
		s32 usb_dc_dis_mask = phy_data->usb_dc_dis_mask;

		if (!addr) {
			addr = ARRAY_INDEX_MAP_PAGE_ADDR(i);
			data = rtk_usb_phy_read(regAddr, addr);

			(phy_page_setting + i)->addr = addr;
			(phy_page_setting + i)->data = data;
			dev_dbg(rtk_phy->dev,
				    "Get default addr %x value %x\n",
				    (phy_page_setting + i)->addr,
				    (phy_page_setting + i)->data);
		}
		__data = rtk_usb_phy_read(regAddr, addr);

		data = (data & usb_dc_dis_mask) | (__data & ~(usb_dc_dis_mask));

		if (isUpdate)
			data = __updated_dc_disconnect_level_page1_0xe2(phy_data, data);

		if (rtk_usb_phy_write(regAddr, addr, data)) {
			dev_err(rtk_phy->dev,
				    "[%s:%d] Error page1 addr=0x%x value=0x%x\n",
				    __func__, __LINE__,
				    addr, data);
			return;
		}

		dev_info(rtk_phy->dev,
			    "%s to set Page1 0xE2=%x for dc disconnect level (%s)\n",
			    __func__,
			    rtk_usb_phy_read(regAddr, addr),
			    isUpdate?"Update":"restore");
	} else {
		dev_err(rtk_phy->dev,
			    "ERROR: %s %d index=%d addr Not PAGE1_0xE2\n",
			    __func__, __LINE__, i);
	}
}

static void do_rtk_usb2_phy_toggle(struct rtk_usb_phy_s *rtk_phy,
	    int index, bool isConnect);

static int do_rtk_usb2_phy_init(struct usb_phy *phy, int index)
{
	int i;
	struct rtk_usb_phy_s *rtk_phy = (struct rtk_usb_phy_s *) phy;
	struct reg_addr *regAddr;
	struct phy_data *phy_data;
	struct phy_parameter *phy_page_setting;

	if (!rtk_phy) {
		pr_err("%s, rtk_phy is NULL\n", __func__);
		return -EINVAL;
	}

	dev_info(phy->dev, "%s Init RTK USB 2.0 PHY phy#%d\n",
		    __func__, index);

	regAddr = &((struct reg_addr *)rtk_phy->reg_addr)[index];
	phy_data = &((struct phy_data *)rtk_phy->phy_data)[index];

	if (!phy_data) {
		pr_err("%s, phy_data is NULL\n", __func__);
		return -EINVAL;
	}

#ifdef CONFIG_USB_PATCH_ON_RTK
	if (check_phy_power(phy_data, regAddr)) {
		phy_data->ldo_enable = true;
		dev_info(phy->dev, "%s USB phy use ldo power compensate phy parameter (%d)\n",
		    __func__, phy_data->ldo_page0_e4_compensate);
	}
#endif // CONFIG_USB_PATCH_ON_RTK

	if (phy_data->use_default_parameter) {
		dev_info(phy->dev, "%s phy#%d use default parameter\n",
			    __func__, index);
		goto do_toggle;
	}

	__get_default_phy_parameter_for_updated(rtk_phy, index);

	/* Set page 0 */
	phy_page_setting = phy_data->page0;
	rtk_usb_phy_set_page(regAddr, 0);

	for (i = 0; i < phy_data->page0_size; i++) {
		u8 addr = (phy_page_setting + i)->addr;
		u8 data = (phy_page_setting + i)->data;

		if (!addr)
			continue;

		if (addr == PAGE0_0xE4)
			data = __updated_page0_0xe4_parameter(phy_data, data);

		if (rtk_usb_phy_write(regAddr, addr, data)) {
			dev_err(phy->dev,
				    "[%s:%d] Error page0 addr=0x%x value=0x%x\n",
				    __func__, __LINE__, addr, data);
			return -1;
		}
		dev_dbg(phy->dev, "[%s:%d] Good page0 addr=0x%x value=0x%x\n",
			    __func__, __LINE__, addr,
			    rtk_usb_phy_read(regAddr, addr));
	}

	/* Set page 1 */
	phy_page_setting = phy_data->page1;
	rtk_usb_phy_set_page(regAddr, 1);

	for (i = 0; i < phy_data->page1_size; i++) {
		u8 addr = (phy_page_setting + i)->addr;
		u8 data = (phy_page_setting + i)->data;

		if (!addr)
			continue;

		if (rtk_usb_phy_write(regAddr, addr, data)) {
			dev_err(phy->dev,
				    "[%s:%d] Error page1 addr=0x%x value=0x%x\n",
				    __func__, __LINE__,
				    addr, data);
			return -1;
		}
		dev_dbg(phy->dev, "[%s:%d] Good page1 addr=0x%x value=0x%x\n",
			    __func__, __LINE__, addr,
			    rtk_usb_phy_read(regAddr, addr));
	}

	if (phy_data->page2_size == 0)
		goto do_toggle;

	/* Set page 2 */
	phy_page_setting = phy_data->page2;
	rtk_usb_phy_set_page(regAddr, 2);

	for (i = 0; i < phy_data->page2_size; i++) {
		u8 addr = (phy_page_setting + i)->addr;
		u8 data = (phy_page_setting + i)->data;

		if (!addr)
			continue;

		if (rtk_usb_phy_write(regAddr, addr, data)) {
			dev_err(phy->dev,
				    "[%s:%d] Error page2 addr=0x%x value=0x%x\n",
				    __func__, __LINE__, addr, data);
			return -1;
		}
		dev_dbg(phy->dev, "[%s:%d] Good page2 addr=0x%x value=0x%x\n",
			    __func__, __LINE__,
			    (phy_page_setting + i)->addr,
			    rtk_usb_phy_read(regAddr,
			      (phy_page_setting + i)->addr));
	}

do_toggle:
	do_rtk_usb2_phy_toggle(rtk_phy, index, false);

	return 0;
}

static int rtk_usb2_phy_init(struct usb_phy *phy)
{
	struct rtk_usb_phy_s *rtk_phy = (struct rtk_usb_phy_s *) phy;
	int i, ret = 0;
	unsigned long phy_init_time = jiffies;

	dev_info(phy->dev, "%s Init RTK USB 2.0 PHY\n", __func__);
	for (i = 0; i < rtk_phy->phyN; i++)
		ret = do_rtk_usb2_phy_init(phy, i);

	dev_info(phy->dev, "%s Initialized RTK USB 2.0 PHY (take %dms)\n",
		    __func__,
		    jiffies_to_msecs(jiffies - phy_init_time));
	return ret;
}

static void do_rtk_usb2_phy_toggle(struct rtk_usb_phy_s *rtk_phy,
	    int index, bool isConnect)
{
	struct reg_addr *regAddr;
	struct phy_data *phy_data;
	struct phy_parameter *phy_page_setting;
	int i;

	if (!rtk_phy) {
		pr_err("%s phy_data is NULL\n", __func__);
		return;
	}

	regAddr = &((struct reg_addr *)rtk_phy->reg_addr)[index];
	phy_data = &((struct phy_data *)rtk_phy->phy_data)[index];

	if (!phy_data) {
		dev_err(rtk_phy->dev, "%s phy_data is NULL\n", __func__);
		return;
	}

	if (!phy_data->do_toggle)
		goto out;

	if (phy_data->is_double_sensitivity_mode)
		goto do_toggle_driving;

	/* Set page 0 */
	phy_page_setting = phy_data->page0;
	rtk_usb_phy_set_page(regAddr, 0);

	i = PAGE_ADDR_MAP_ARRAY_INDEX(PAGE0_0xE7);

	if (i < phy_data->page0_size) {
		u8 addr = (phy_page_setting + i)->addr;
		u8 data = (phy_page_setting + i)->data;

		if (!addr) {
			addr = ARRAY_INDEX_MAP_PAGE_ADDR(i);
			data = rtk_usb_phy_read(regAddr, addr);

			(phy_page_setting + i)->addr = addr;
			(phy_page_setting + i)->data = data;
			dev_dbg(rtk_phy->dev,
				    "Get default addr %x value %x\n",
				    (phy_page_setting + i)->addr,
				    (phy_page_setting + i)->data);
		}

		if (isConnect) {
			rtk_usb_phy_write(regAddr, addr, data &
				    (~(BIT(4) | BIT(5) | BIT(6))));
		} else {
			rtk_usb_phy_write(regAddr, addr, data |
				    (BIT(4) | BIT(5) | BIT(6)));
		}
		dev_info(rtk_phy->dev,
			    "%s %sconnect to set Page0 0xE7=%x\n",
			    __func__,
			    isConnect?"":"dis",
			    rtk_usb_phy_read(regAddr, addr));
	} else {
		dev_err(rtk_phy->dev,
			    "ERROR: %s %d index=%d addr Not PAGE0_0xE7\n",
			    __func__, __LINE__, i);
	}

do_toggle_driving:

	if (!phy_data->do_toggle_driving)
		goto do_toggle;

	/* Page 0 addr 0xE4 driving capability */

	/* Set page 0 */
	phy_page_setting = phy_data->page0;
	rtk_usb_phy_set_page(regAddr, 0);

	i = PAGE_ADDR_MAP_ARRAY_INDEX(PAGE0_0xE4);

	if (i < phy_data->page0_size) {
		u8 addr = (phy_page_setting + i)->addr;
		u8 data = (phy_page_setting + i)->data;

		if (!addr) {
			addr = ARRAY_INDEX_MAP_PAGE_ADDR(i);
			data = rtk_usb_phy_read(regAddr, addr);

			(phy_page_setting + i)->addr = addr;
			(phy_page_setting + i)->data = data;
			dev_dbg(rtk_phy->dev,
				    "Get default addr %x value %x\n",
				    (phy_page_setting + i)->addr,
				    (phy_page_setting + i)->data);
		}

		if (addr == PAGE0_0xE4)
			data = __updated_page0_0xe4_parameter(phy_data, data);

		if (isConnect) {
			rtk_usb_phy_write(regAddr, addr, data);
		} else {
			u8 val;
			s32 __val;
			s32 driving_updated =
				    phy_data->disconnect_driving_updated;
			s32 usb_dc_cal_mask = phy_data->usb_dc_cal_mask;

			__val = (s32)(data & usb_dc_cal_mask) + driving_updated;

			if (__val > usb_dc_cal_mask)
				__val = usb_dc_cal_mask;
			else if (__val < 0)
				__val = 0;

			val = (data & (~usb_dc_cal_mask)) | (__val & usb_dc_cal_mask);

			rtk_usb_phy_write(regAddr, addr, val);
		}
		dev_info(rtk_phy->dev,
			    "%s %sconnect to set Page0 0xE4=%x for driving\n",
			    __func__,
			    isConnect?"":"dis",
			    rtk_usb_phy_read(regAddr, addr));
	} else {
		dev_err(rtk_phy->dev,
			    "ERROR: %s %d index=%d addr Not PAGE0_0xE4\n",
			    __func__, __LINE__, i);
	}

do_toggle:
	/* restore dc disconnect level before toggle */
	update_dc_disconnect_level(rtk_phy, regAddr, phy_data, false);

	/* Set page 1 */
	phy_page_setting = phy_data->page1;
	rtk_usb_phy_set_page(regAddr, 1);

	i = PAGE_ADDR_MAP_ARRAY_INDEX(PAGE1_0xE0);

	if (i < phy_data->page1_size) {
		u8 addr = (phy_page_setting + i)->addr;
		u8 data = (phy_page_setting + i)->data;

		if (!addr) {
			addr = ARRAY_INDEX_MAP_PAGE_ADDR(i);
			data = rtk_usb_phy_read(regAddr, addr);

			(phy_page_setting + i)->addr = addr;
			(phy_page_setting + i)->data = data;
			dev_dbg(rtk_phy->dev,
				    "Get default addr %x value %x\n",
				    (phy_page_setting + i)->addr,
				    (phy_page_setting + i)->data);
		}

		dev_info(rtk_phy->dev,
			    "%s ########## to toggle PAGE1_0xE0 BIT(2)\n",
			    __func__);
		rtk_usb_phy_write(regAddr, addr, data & (~BIT(2)));
		mdelay(1);
		rtk_usb_phy_write(regAddr, addr, data | (BIT(2)));
		mdelay(1);
	} else {
		dev_err(rtk_phy->dev,
			    "ERROR: %s %d index=%d addr Not PAGE1_0xE0\n",
			    __func__, __LINE__, i);
	}

	/* update dc disconnect level after toggle */
	update_dc_disconnect_level(rtk_phy, regAddr, phy_data, true);

out:
	return;
}

void rtk_usb2_phy_toggle(struct usb_phy *usb2_phy, bool isConnect, int port)
{
	int index = port;
	struct rtk_usb_phy_s *rtk_phy = NULL;

	if (usb2_phy != NULL && usb2_phy->dev != NULL)
		rtk_phy = dev_get_drvdata(usb2_phy->dev);

	if (rtk_phy == NULL) {
		pr_err("%s %d ERROR! NO this device\n", __func__, __LINE__);
		return;
	}
	if (index > rtk_phy->phyN) {
		pr_err("%s %d ERROR! port=%d > phyN=%d\n",
			    __func__, __LINE__, index, rtk_phy->phyN);
		return;
	}

	do_rtk_usb2_phy_toggle(rtk_phy, index, isConnect);

}
EXPORT_SYMBOL(rtk_usb2_phy_toggle);

int rtk_usb2_phy_notify_port_change(struct usb_phy *x, int port,
	    u16 portstatus, u16 portchange)
{
	bool isConnect = false;

	pr_debug("%s port=%d portstatus=0x%x portchange=0x%x\n",
		    __func__, port, (int)portstatus, (int)portchange);
	if (portstatus & USB_PORT_STAT_CONNECTION)
		isConnect = true;

	if (portchange & USB_PORT_STAT_C_CONNECTION)
		rtk_usb2_phy_toggle(x, isConnect, port);

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *create_phy_debug_root(void)
{
	struct dentry *phy_debug_root;

	phy_debug_root = debugfs_lookup("phy", usb_debug_root);
	if (!phy_debug_root) {
		phy_debug_root = debugfs_create_dir("phy", usb_debug_root);
		if (!phy_debug_root)
			pr_err("%s Error phy_debug_root is NULL\n", __func__);
		else
			pr_info("%s Create phy_debug_root folder\n", __func__);
	}

	return phy_debug_root;
}

static int rtk_usb2_parameter_show(struct seq_file *s, void *unused)
{
	struct rtk_usb_phy_s		*rtk_phy = s->private;
	int i, index;

	for (index = 0; index < rtk_phy->phyN; index++) {
		struct reg_addr *regAddr =
			    &((struct reg_addr *)rtk_phy->reg_addr)[index];
		struct phy_data *phy_data =
			    &((struct phy_data *)rtk_phy->phy_data)[index];
		struct phy_parameter *phy_page_setting;

		seq_printf(s, "PHY %d:\n", index);

		seq_puts(s, "Page 0:\n");
		/* Set page 0 */
		phy_page_setting = phy_data->page0;
		rtk_usb_phy_set_page(regAddr, 0);

		for (i = 0; i < phy_data->page0_size; i++) {
			u8 addr = ARRAY_INDEX_MAP_PAGE_ADDR(i);
			u8 data = (phy_page_setting + i)->data;
			u8 value = rtk_usb_phy_read(regAddr, addr);

			if ((phy_page_setting + i)->addr)
				seq_printf(s, "Page 0: addr=0x%x data=0x%02x ==> read value=0x%02x\n",
					    addr, data, value);
			else
				seq_printf(s, "Page 0: addr=0x%x data=none ==> read value=0x%02x\n",
					    addr, value);
		}

		seq_puts(s, "Page 1:\n");
		/* Set page 1 */
		phy_page_setting = phy_data->page1;
		rtk_usb_phy_set_page(regAddr, 1);

		for (i = 0; i < phy_data->page1_size; i++) {
			u8 addr = ARRAY_INDEX_MAP_PAGE_ADDR(i);
			u8 data = (phy_page_setting + i)->data;
			u8 value = rtk_usb_phy_read(regAddr, addr);

			if ((phy_page_setting + i)->addr)
				seq_printf(s, "Page 1: addr=0x%x data=0x%02x ==> read value=0x%02x\n",
					    addr, data, value);
			else
				seq_printf(s, "Page 1: addr=0x%x data=none ==> read value=0x%02x\n",
					    addr, value);
		}

		if (phy_data->page2_size == 0)
			goto out;

		seq_puts(s, "Page 2:\n");
		/* Set page 2 */
		phy_page_setting = phy_data->page2;
		rtk_usb_phy_set_page(regAddr, 2);

		for (i = 0; i < phy_data->page2_size; i++) {
			u8 addr = ARRAY_INDEX_MAP_PAGE_ADDR(i);
			u8 data = (phy_page_setting + i)->data;
			u8 value = rtk_usb_phy_read(regAddr, addr);

			if ((phy_page_setting + i)->addr)
				seq_printf(s, "Page 2: addr=0x%x data=0x%02x ==> read value=0x%02x\n",
					    addr, data, value);
			else
				seq_printf(s, "Page 2: addr=0x%x data=none ==> read value=0x%02x\n",
					    addr, value);
		}

		seq_printf(s, "Property:\n");
		seq_printf(s, "check_efuse: %s\n",
			    phy_data->check_efuse?"Enable":"Disable");
		seq_printf(s, "efuse_usb_dc_cal: %d\n",
			    (int)phy_data->efuse_usb_dc_cal);
		seq_printf(s, "efuse_usb_dc_cal_rate: %d\n",
			    phy_data->efuse_usb_dc_cal_rate);
		seq_printf(s, "usb_dc_cal_mask: 0x%x\n",
			    phy_data->usb_dc_cal_mask);
		seq_printf(s, "efuse_usb_dc_dis: %d\n",
			    (int)phy_data->efuse_usb_dc_dis);
		seq_printf(s, "efuse_usb_dc_dis_rate: %d\n",
			    phy_data->efuse_usb_dc_dis_rate);
		seq_printf(s, "usb_dc_dis_mask: 0x%x\n",
			    phy_data->usb_dc_dis_mask);
		seq_printf(s, "usb_dc_dis_at_page0: %s\n",
			    phy_data->usb_dc_dis_at_page0?"true":"false");
		seq_printf(s, "do_toggle: %s\n",
			    phy_data->do_toggle?"Enable":"Disable");
		seq_printf(s, "do_toggle_driving: %s\n",
			    phy_data->do_toggle_driving?"Enable":"Disable");
		seq_printf(s, "disconnect_driving_updated: 0x%x\n",
			    phy_data->disconnect_driving_updated);
		seq_printf(s, "use_default_parameter: %s\n",
			    phy_data->use_default_parameter?"Enable":"Disable");
		seq_printf(s, "is_double_sensitivity_mode: %s\n",
			    phy_data->is_double_sensitivity_mode?"Enable":"Disable");
		seq_printf(s, "ldo_force_enable: %s\n",
			    phy_data->ldo_force_enable?"Enable":"Disable");
		seq_printf(s, "ldo_enable: %s\n",
			    phy_data->ldo_enable?"Enable":"Disable");
		seq_printf(s, "ldo_page0_e4_compensate: %d\n",
			    phy_data->ldo_page0_e4_compensate);
	}

out:
	return 0;
}

static int rtk_usb2_parameter_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtk_usb2_parameter_show, inode->i_private);
}

static const struct file_operations rtk_usb2_parameter_fops = {
	.open			= rtk_usb2_parameter_open,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int rtk_usb2_set_parameter_show(struct seq_file *s, void *unused)
{
	//struct rtk_usb_phy_s	*rtk_phy = s->private;

	seq_puts(s, "Set Phy parameter by following command\n");
	seq_puts(s, "echo \"phy_num page addr value\" > set_parameter\n");
	seq_puts(s, "echo \"0 page0 0xE1 0x30\" > set_parameter\n");
	seq_puts(s, "echo \"0 page1 0xE1 0xEF\" > set_parameter\n");

	return 0;
}

static int rtk_usb2_set_parameter_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtk_usb2_set_parameter_show, inode->i_private);
}

static ssize_t rtk_usb2_set_parameter_write(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file		*s = file->private_data;
	struct rtk_usb_phy_s		*rtk_phy = s->private;
	struct reg_addr *regAddr;
	struct phy_data *phy_data;
	struct phy_parameter *phy_page_setting;
	int page_size = 0;
	int ret = 0;
	char			buffer[40];
	char *buf = buffer;
	int i, index;
	u32 addr;
	u32 value;

	if (copy_from_user(&buffer, ubuf,
		    min_t(size_t, sizeof(buffer) - 1, count)))
		return -EFAULT;

	ret = kstrtoint(buf, 0, &index);
	if (ret < 0)
		return -EFAULT;

	buf = buf + 2;
	regAddr = &((struct reg_addr *)rtk_phy->reg_addr)[index];
	phy_data = &((struct phy_data *)rtk_phy->phy_data)[index];

	if (!strncmp(buf, "page0", 5)) {
		buf = buf + 5;
		buf = skip_spaces(buf);
		ret = sscanf(buf, "%x %x", &addr, &value);
		if (ret < 0)
			return -EFAULT;

		phy_page_setting = phy_data->page0;
		page_size = phy_data->page0_size;
		rtk_usb_phy_set_page(regAddr, 0);
		dev_dbg(rtk_phy->dev, "%s page0 addr = 0x%x, value = 0x%x\n",
			    __func__, addr, value);

	} else if (!strncmp(buf, "page1", 5)) {
		buf = buf + 5;
		buf = skip_spaces(buf);
		ret = sscanf(buf, "%x %x", &addr, &value);
		if (ret < 0)
			return -EFAULT;

		phy_page_setting = phy_data->page1;
		page_size = phy_data->page1_size;
		rtk_usb_phy_set_page(regAddr, 1);
		dev_dbg(rtk_phy->dev, "%s page1 addr = 0x%x, value = 0x%x\n",
			    __func__, addr, value);
	} else if (!strncmp(buf, "page2", 5)) {
		buf = buf + 5;
		buf = skip_spaces(buf);
		ret = sscanf(buf, "%x %x", &addr, &value);
		if (ret < 0)
			return -EFAULT;

		phy_page_setting = phy_data->page2;
		page_size = phy_data->page2_size;
		rtk_usb_phy_set_page(regAddr, 2);
		dev_dbg(rtk_phy->dev, "%s page2 addr = 0x%x, value = 0x%x\n",
			    __func__, addr, value);
	} else {
		dev_err(rtk_phy->dev, "UNKNOWN input (%s)", buf);
	}

	for (i = 0; i < page_size; i++) {
		u8 i2addr = ARRAY_INDEX_MAP_PAGE_ADDR(i);

		if (i2addr == addr) {
			(phy_page_setting + i)->addr = addr;
			(phy_page_setting + i)->data = value;
			if (rtk_usb_phy_write(regAddr, addr, value))
				dev_err(rtk_phy->dev,
					    "[%s:%d] Error: addr=0x%x value=0x%x\n",
					    __func__, __LINE__, addr, value);
			else
				dev_dbg(rtk_phy->dev,
					    "[%s:%d] Good: addr=0x%x value=0x%x\n",
					    __func__, __LINE__, addr,
					    rtk_usb_phy_read(regAddr, addr));
		}
	}

	return count;
}

static const struct file_operations rtk_usb2_set_parameter_fops = {
	.open			= rtk_usb2_set_parameter_open,
	.write			= rtk_usb2_set_parameter_write,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int rtk_usb2_toggle_show(struct seq_file *s, void *unused)
{
	struct rtk_usb_phy_s *rtk_phy = s->private;
	struct phy_data *phy_data;
	int i;

	for (i = 0; i < rtk_phy->phyN; i++) {
		phy_data = &((struct phy_data *)rtk_phy->phy_data)[i];
		seq_printf(s, "Now phy#%d do_toggle is %s.\n",
			    i, phy_data->do_toggle?"Enable":"Disable");
	}
	seq_puts(s, "ehco 1 to enable toggle phy parameter.\n");

	return 0;
}

static int rtk_usb2_toggle_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtk_usb2_toggle_show, inode->i_private);
}

static ssize_t rtk_usb2_toggle_write(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file		*s = file->private_data;
	struct rtk_usb_phy_s		*rtk_phy = s->private;
	char			buf[32];
	struct phy_data *phy_data;
	bool enable = false;
	int i;

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "1", 1))
		enable = true;

	for (i = 0; i < rtk_phy->phyN; i++) {
		phy_data = &((struct phy_data *)rtk_phy->phy_data)[i];
		phy_data->do_toggle = enable;
		dev_info(rtk_phy->dev, "Set phy#%d do_toggle is %s.\n",
			    i, phy_data->do_toggle?"Enable":"Disable");
	}

	return count;
}

static const struct file_operations rtk_usb2_toggle_fops = {
	.open			= rtk_usb2_toggle_open,
	.write			= rtk_usb2_toggle_write,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static inline void create_debug_files(struct rtk_usb_phy_s *rtk_phy)
{
	struct dentry *phy_debug_root = NULL;

	dev_dbg(rtk_phy->dev, "%s", __func__);

	phy_debug_root = create_phy_debug_root();

	if (!phy_debug_root) {
		dev_err(rtk_phy->dev, "%s Error phy_debug_root is NULL",
			    __func__);
		return;
	}

	rtk_phy->debug_dir = debugfs_create_dir(dev_name(rtk_phy->dev),
		    phy_debug_root);
	if (!rtk_phy->debug_dir) {
		dev_err(rtk_phy->dev, "%s Error debug_dir is NULL", __func__);
		return;
	}

	if (!debugfs_create_file("parameter", 0444, rtk_phy->debug_dir, rtk_phy,
		    &rtk_usb2_parameter_fops))
		goto file_error;

	if (!debugfs_create_file("set_parameter", 0644,
		    rtk_phy->debug_dir, rtk_phy, &rtk_usb2_set_parameter_fops))
		goto file_error;

	if (!debugfs_create_file("toggle", 0644,
		    rtk_phy->debug_dir, rtk_phy, &rtk_usb2_toggle_fops))
		goto file_error;

	return;

file_error:
	debugfs_remove_recursive(rtk_phy->debug_dir);
}
#endif //CONFIG_DEBUG_FS

#define DEFAULT_CHIP_REVISION 0xA00
#define MAX_CHIP_REVISION 0xC00

static int __get_chip_revision(void)
{
	int chip_revision = 0xFFF;
	char revision[] = "FFF";
	struct soc_device_attribute soc_att[] = {{.revision = revision}, {}};
	struct soc_device_attribute *soc_att_match = NULL;

	while (soc_att_match == NULL) {
		chip_revision--;

		if (chip_revision <= DEFAULT_CHIP_REVISION)
			break;
		if (chip_revision > MAX_CHIP_REVISION)
			chip_revision = MAX_CHIP_REVISION;
		else if ((chip_revision & 0xFF) > 0xF)
			chip_revision = (chip_revision & 0xF00) + 0xF;

		snprintf(revision, 4, "%X", chip_revision);

		soc_att_match = (struct soc_device_attribute *)
			    soc_device_match(soc_att);
	}

	if (soc_att_match) {
		pr_debug("%s get chip_revision %x\n", __func__, chip_revision);
		return chip_revision;
	}

	pr_debug("%s: Use default chip_revision %x\n", __func__,
		    DEFAULT_CHIP_REVISION);
	return DEFAULT_CHIP_REVISION;
}

static int __get_phy_parameter_v1(struct device *dev, struct phy_data *phy_data,
	    struct device_node *sub_node)
{
	int phy_data_page0_size, phy_data_page1_size;
	int phy_data_page2_size;
	char tmp_addr[MAX_PHY_DATA_SIZE];
	char tmp_data[MAX_PHY_DATA_SIZE];
	int i, chip_revision, revision, ret = 0;

	chip_revision = __get_chip_revision();

	dev_dbg(dev, "%s: Chip revision is %x\n", __func__, chip_revision);

	ret = of_property_read_u32_index(sub_node,
		    "phy_data_page0_size", 0, &phy_data_page0_size);
	if (ret)
		goto err;

	ret = of_property_read_u32_index(sub_node,
		    "phy_data_page1_size", 0, &phy_data_page1_size);
	if (ret)
		goto err;

	dev_dbg(dev, "%s %d phy_data_page0_size=%d, phy_data_page1_size=%d\n",
		    __func__, __LINE__,
		    phy_data_page0_size, phy_data_page1_size);

	if (phy_data_page0_size > MAX_PHY_DATA_SIZE ||
		    phy_data_page1_size > MAX_PHY_DATA_SIZE) {
		dev_err(dev, "%s phy_data size > MAX_PHY_DATA_SIZE\n",
			    __func__);
		goto err;
	}

	ret = of_property_read_u32_index(sub_node,
		    "phy_data_page2_size", 0, &phy_data_page2_size);
	if (ret)
		phy_data_page2_size = 0;
	dev_dbg(dev, "%s %d phy_data_page2_size=%d\n",
		    __func__, __LINE__,
		    phy_data_page2_size);

	if (phy_data_page2_size > MAX_PHY_DATA_SIZE) {
		dev_err(dev, "%s page2 phy_data size=%d > MAX_PHY_DATA_SIZE\n",
			    __func__, phy_data_page2_size);
		goto err;
	}

	phy_data->page0_size = phy_data_page0_size;
	phy_data->page0 = devm_kzalloc(dev,
		    sizeof(struct phy_parameter) *
			phy_data_page0_size,
		    GFP_KERNEL);
	if (!phy_data->page0) {
		ret = -ENOMEM;
		goto err;
	}

	phy_data->page1_size = phy_data_page1_size;
	phy_data->page1 = devm_kzalloc(dev,
		    sizeof(struct phy_parameter) *
			phy_data_page1_size,
		    GFP_KERNEL);
	if (!phy_data->page1) {
		ret = -ENOMEM;
		goto err;
	}

	phy_data->page2_size = phy_data_page2_size;
	if (phy_data->page2_size > 0) {
		phy_data->page2 = devm_kzalloc(dev,
			    sizeof(struct phy_parameter) *
				phy_data->page2_size,
			    GFP_KERNEL);
		if (!phy_data->page2) {
			ret = -ENOMEM;
			goto err;
		}
	}

	ret = of_property_read_u8_array(sub_node, "phy_data_page0_addr",
		    tmp_addr, phy_data_page0_size);
	if (ret)
		goto err;

	revision = chip_revision;
	while (revision >= DEFAULT_CHIP_REVISION) {
		char phy_data_revision[20] = {0};

		snprintf(phy_data_revision, 19, "phy_data_page0_%X", revision);

		ret = of_property_read_u8_array(sub_node, phy_data_revision,
			    tmp_data, phy_data_page0_size);
		if (!ret) {
			dev_dbg(dev, "%s load %s parameter\n",
				    __func__, phy_data_revision);
			break;
		}
		revision--;
		if ((revision & 0xFF) > 0xF)
			revision = (revision & 0xF00) + 0xF;
	}

	/* For old device tree */
	if (ret) {
		ret = of_property_read_u8_array(sub_node, "phy_data_page0_data",
			    tmp_data, phy_data_page0_size);
		if (ret)
			goto err;
		else
			dev_info(dev, "%s load page0 parameter\n",
				    __func__);
	}

	for (i = 0; i < phy_data_page0_size; i++) {
		struct phy_parameter *phy_data_page0 =
			    (phy_data->page0 + i);

		phy_data_page0->addr = tmp_addr[i];
		phy_data_page0->data = tmp_data[i];
	}

	ret = of_property_read_u8_array(sub_node, "phy_data_page1_addr",
		    tmp_addr, phy_data_page1_size);
	if (ret)
		goto err;

	revision = chip_revision;
	while (revision >= DEFAULT_CHIP_REVISION) {
		char phy_data_revision[20] = {0};

		snprintf(phy_data_revision, 19, "phy_data_page1_%X", revision);

		ret = of_property_read_u8_array(sub_node, phy_data_revision,
			    tmp_data, phy_data_page1_size);
		if (!ret) {
			dev_dbg(dev, "%s load %s parameter\n",
				    __func__, phy_data_revision);
			break;
		}
		revision--;
		if ((revision & 0xFF) > 0xF)
			revision = (revision & 0xF00) + 0xF;
	}

	/* For old device tree */
	if (ret) {
		ret = of_property_read_u8_array(sub_node, "phy_data_page1_data",
			    tmp_data, phy_data_page1_size);
		if (ret)
			goto err;
		else
			dev_info(dev, "%s load page1 parameter\n",
				    __func__);
	}

	for (i = 0; i < phy_data_page1_size; i++) {
		struct phy_parameter *phy_data_page1 =
			    (phy_data->page1 + i);

		phy_data_page1->addr = tmp_addr[i];
		phy_data_page1->data = tmp_data[i];
	}

	if (phy_data->page2_size > 0) {
		ret = of_property_read_u8_array(sub_node,
			    "phy_data_page2_addr",
			    tmp_addr, phy_data->page2_size);
		if (ret)
			goto err;

		revision = chip_revision;
		while (revision >= DEFAULT_CHIP_REVISION) {
			char phy_data_revision[20] = {0};

			snprintf(phy_data_revision, 19, "phy_data_page2_%X",
				    revision);

			ret = of_property_read_u8_array(sub_node,
				    phy_data_revision,
				    tmp_data, phy_data_page2_size);
			if (!ret) {
				dev_dbg(dev, "%s load %s parameter\n",
					    __func__, phy_data_revision);
				break;
			}
			revision--;
			if ((revision & 0xFF) > 0xF)
				revision = (revision & 0xF00) + 0xF;
		}

		/* For old device tree */
		if (ret) {
			ret = of_property_read_u8_array(sub_node,
				    "phy_data_page2_data",
				    tmp_data, phy_data->page2_size);
			if (ret)
				goto err;
			else
				dev_info(dev, "%s load page2 parameter\n",
					    __func__);
		}
		for (i = 0; i < phy_data->page2_size; i++) {
			struct phy_parameter *phy_data_page2 =
				    (phy_data->page2 + i);
			phy_data_page2->addr = tmp_addr[i];
			phy_data_page2->data = tmp_data[i];
		}
	}

err:
	return ret;
}

static int __get_phy_parameter_v2(struct device *dev, struct phy_data *phy_data,
	    struct device_node *sub_node)
{
	u32 page_size = 0;
	u32 num_cells = 2; /*< addr value > */
	u32 data_size;
	int i, offset, chip_revision, revision, ret = 0;
	char phy_data_revision[15] = {0};

	chip_revision = __get_chip_revision();

	/* Page 0 */
	ret = of_property_read_u32_index(sub_node, "page0_size", 0, &page_size);
	if (ret) {
		dev_err(dev, "%s No page0_size\n", __func__);
		goto parse_page1;
	}

	phy_data->page0_size = page_size;
	phy_data->page0 = devm_kzalloc(dev,
		    sizeof(struct phy_parameter) * page_size, GFP_KERNEL);
	if (!phy_data->page0) {
		ret = -ENOMEM;
		goto out;
	}

	revision = chip_revision;
	while (revision >= DEFAULT_CHIP_REVISION) {
		snprintf(phy_data_revision, 15, "page0_data_%X", revision);

		if (of_get_property(sub_node, phy_data_revision, &data_size)) {
			dev_dbg(dev, "%s load %s parameter (data_size=%d)\n",
				    __func__, phy_data_revision, data_size);
			break;
		}
		revision--;
		if ((revision & 0xFF) > 0xF)
			revision = (revision & 0xF00) + 0xF;

		data_size = 0;
		ret = 0;
	}
	data_size = data_size / (sizeof(u32) * num_cells);

	for (i = 0; i < data_size; i++) {
		struct phy_parameter *phy_data_page;
		u32 addr, data;
		int index;

		offset = i * num_cells;

		ret = of_property_read_u32_index(sub_node, phy_data_revision,
			    offset, &addr);
		if (ret) {
			dev_err(dev, "ERROR: To get %s i=%d addr=0x%x\n",
				    phy_data_revision, i, addr);
			break;
		}

		ret = of_property_read_u32_index(sub_node, phy_data_revision,
			    offset + 1, &data);
		if (ret) {
			dev_err(dev, "ERROR: To get %s i=%d addr=0x%x\n",
				    phy_data_revision, i, data);
			break;
		}

		index = PAGE_ADDR_MAP_ARRAY_INDEX(addr);
		phy_data_page = (phy_data->page0 + index);
		phy_data_page->addr = (char)addr;
		phy_data_page->data = (char)data;

		dev_dbg(dev, "%s index=%d addr=0x%x data=0x%x\n",
			    phy_data_revision, index,
			    phy_data_page->addr, phy_data_page->data);
	}

parse_page1:
	/* Page 1 */
	ret = of_property_read_u32_index(sub_node, "page1_size", 0, &page_size);
	if (ret) {
		dev_err(dev, "%s No page0_size\n", __func__);
		goto parse_page2;
	}

	phy_data->page1_size = page_size;
	phy_data->page1 = devm_kzalloc(dev,
		    sizeof(struct rtk_usb_phy_data_s) * page_size, GFP_KERNEL);
	if (!phy_data->page1) {
		ret = -ENOMEM;
		goto out;
	}

	revision = chip_revision;
	while (revision >= DEFAULT_CHIP_REVISION) {
		snprintf(phy_data_revision, 15, "page1_data_%X", revision);

		if (of_get_property(sub_node, phy_data_revision, &data_size)) {
			dev_dbg(dev, "%s load %s parameter (data_size=%d)\n",
				    __func__, phy_data_revision, data_size);
			break;
		}
		revision--;
		if ((revision & 0xFF) > 0xF)
			revision = (revision & 0xF00) + 0xF;

		data_size = 0;
		ret = 0;
	}
	data_size = data_size / (sizeof(u32) * num_cells);

	for (i = 0; i < data_size; i++) {
		struct phy_parameter *phy_data_page;
		u32 addr, data;
		int index;

		offset = i * num_cells;

		ret = of_property_read_u32_index(sub_node, phy_data_revision,
			    offset, &addr);
		if (ret) {
			dev_err(dev, "ERROR: To get %s i=%d addr=0x%x\n",
				    phy_data_revision, i, addr);
			break;
		}

		ret = of_property_read_u32_index(sub_node, phy_data_revision,
			    offset + 1, &data);
		if (ret) {
			dev_err(dev, "ERROR: To get %s i=%d addr=0x%x\n",
				    phy_data_revision, i, data);
			break;
		}

		index = PAGE_ADDR_MAP_ARRAY_INDEX(addr);
		phy_data_page = phy_data->page1 + index;
		phy_data_page->addr = (char)addr;
		phy_data_page->data = (char)data;

		dev_dbg(dev, "%s index=%d addr=0x%x data=0x%x\n",
			    phy_data_revision, index,
			    phy_data_page->addr, phy_data_page->data);
	}

parse_page2:
	/* Page 2 */
	ret = of_property_read_u32_index(sub_node, "page2_size", 0, &page_size);
	if (ret) {
		dev_dbg(dev, "%s No page2_size\n", __func__);
		goto out;
	}

	phy_data->page2_size = page_size;
	phy_data->page2 = devm_kzalloc(dev,
		    sizeof(struct rtk_usb_phy_data_s) * page_size, GFP_KERNEL);
	if (!phy_data->page2) {
		ret = -ENOMEM;
		goto out;
	}

	revision = chip_revision;
	while (revision >= DEFAULT_CHIP_REVISION) {
		snprintf(phy_data_revision, 15, "page2_data_%X", revision);

		if (of_get_property(sub_node, phy_data_revision, &data_size)) {
			dev_dbg(dev, "%s load %s parameter (data_size=%d)\n",
				    __func__, phy_data_revision, data_size);
			break;
		}
		revision--;
		if ((revision & 0xFF) > 0xF)
			revision = (revision & 0xF00) + 0xF;

		data_size = 0;
		ret = 0;
	}
	data_size = data_size / (sizeof(u32) * num_cells);

	for (i = 0; i < data_size; i++) {
		struct phy_parameter *phy_data_page;
		u32 addr, data;
		int index;

		offset = i * num_cells;

		ret = of_property_read_u32_index(sub_node, phy_data_revision,
			    offset, &addr);
		if (ret) {
			dev_err(dev, "ERROR: To get %s i=%d addr=0x%x\n",
				    phy_data_revision, i, addr);
			break;
		}

		ret = of_property_read_u32_index(sub_node, phy_data_revision,
			    offset + 1, &data);
		if (ret) {
			dev_err(dev, "ERROR: To get %s i=%d addr=0x%x\n",
				    phy_data_revision, i, data);
			break;
		}

		index = PAGE_ADDR_MAP_ARRAY_INDEX(addr);
		phy_data_page = phy_data->page2 + index;
		phy_data_page->addr = (char)addr;
		phy_data_page->data = (char)data;

		dev_dbg(dev, "%s index=%d addr=0x%x data=0x%x\n",
			    phy_data_revision, index,
			    phy_data_page->addr, phy_data_page->data);
	}

out:
	return ret;
}

static int __get_phy_parameter(struct rtk_usb_phy_s *rtk_usb_phy, int index)
{
	struct device *dev = rtk_usb_phy->dev;
	struct reg_addr *addr =
		    &((struct reg_addr *)rtk_usb_phy->reg_addr)[index];
	struct phy_data *phy_data =
		    &((struct phy_data *)rtk_usb_phy->phy_data)[index];
	char phy_name[5], phy_name_v2[10];
	struct device_node *sub_node;
	int ret = 0;

	addr->REG_WRAP_VStatusOut2 = of_iomap(dev->of_node, 0);
	addr->REG_GUSB2PHYACC0     = of_iomap(dev->of_node, index + 1);
	addr->vstatus_index = index;
	dev_dbg(dev, "%s %d #%d REG_WRAP_VStatusOut2=%p\n",
		    __func__, __LINE__,
		    index, addr->REG_WRAP_VStatusOut2);
	dev_dbg(dev, "%s %d #%d REG_GUSB2PHYACC0=%p\n",
		    __func__, __LINE__,
		    index, addr->REG_GUSB2PHYACC0);

	snprintf(phy_name, 5, "phy%d", index);

	sub_node = of_get_child_by_name(dev->of_node, phy_name);
	if (sub_node) {
		dev_info(dev, "%s %d: #%d Get phy data v1 sub_node for %s\n",
			    __func__, __LINE__, index, phy_name);
		ret = __get_phy_parameter_v1(dev, phy_data, sub_node);
		if (ret)
			goto err;
	} else {
		snprintf(phy_name_v2, 10, "phy%d_data", index);
		sub_node = of_get_child_by_name(dev->of_node, phy_name_v2);
		if (sub_node) {
			dev_info(dev, "%s %d: #%d Get phy data v2 sub_node for %s\n",
			    __func__, __LINE__, index, phy_name_v2);
			ret = __get_phy_parameter_v2(dev, phy_data, sub_node);
			if (ret)
				goto err;
		}
	}

	if (!sub_node)
		goto err;

	if (of_property_read_bool(sub_node, "do_toggle"))
		phy_data->do_toggle = true;
	else
		phy_data->do_toggle = false;

	if (of_property_read_bool(sub_node, "do_toggle_driving"))
		phy_data->do_toggle_driving = true;
	else
		phy_data->do_toggle_driving = false;

	if (of_property_read_s32(sub_node,
		    "disconnect_driving_updated",
		    &phy_data->disconnect_driving_updated))
		phy_data->disconnect_driving_updated = 0xf;

	if (of_property_read_bool(sub_node, "check_efuse"))
		phy_data->check_efuse = true;
	else
		phy_data->check_efuse = false;

	if (of_property_read_bool(sub_node, "use_default_parameter"))
		phy_data->use_default_parameter = true;
	else
		phy_data->use_default_parameter = false;

	if (of_property_read_bool(sub_node,
		    "is_double_sensitivity_mode"))
		phy_data->is_double_sensitivity_mode = true;
	else
		phy_data->is_double_sensitivity_mode = false;

	if (of_property_read_bool(sub_node,
		    "ldo_force_enable"))
		phy_data->ldo_force_enable = true;
	else
		phy_data->ldo_force_enable = false;

	if (of_property_read_s32(sub_node,
		 "ldo_page0_e4_compensate", &phy_data->ldo_page0_e4_compensate))
		phy_data->ldo_page0_e4_compensate = 0;

	if (of_property_read_s32(sub_node,
		 "efuse_usb_dc_cal_rate", &phy_data->efuse_usb_dc_cal_rate))
		phy_data->efuse_usb_dc_cal_rate = EFUS_USB_DC_CAL_RATE;

	if (of_property_read_s32(sub_node,
		 "usb_dc_cal_mask", &phy_data->usb_dc_cal_mask))
		phy_data->usb_dc_cal_mask = USB_DC_CAL_MASK;

	if (of_property_read_s32(sub_node,
		 "efuse_usb_dc_dis_rate", &phy_data->efuse_usb_dc_dis_rate))
		phy_data->efuse_usb_dc_dis_rate = EFUS_USB_DC_DIS_RATE;

	if (of_property_read_s32(sub_node,
		 "usb_dc_dis_mask", &phy_data->usb_dc_dis_mask))
		phy_data->usb_dc_dis_mask = USB_DC_DIS_MASK;

	if (of_property_read_bool(sub_node, "usb_dc_dis_at_page0"))
		phy_data->usb_dc_dis_at_page0 = true;
	else
		phy_data->usb_dc_dis_at_page0 = false;

	if (phy_data->check_efuse)
		__get_phy_parameter_by_efuse(rtk_usb_phy, phy_data, index);

err:
	return ret;
}

static int rtk_usb2phy_probe(struct platform_device *pdev)
{
	struct rtk_usb_phy_s *rtk_usb_phy;
	struct device *dev = &pdev->dev;
	int index, ret = 0;
	int port_index, phyN;

	rtk_usb_phy = devm_kzalloc(dev, sizeof(*rtk_usb_phy), GFP_KERNEL);
	if (!rtk_usb_phy)
		return -ENOMEM;

	rtk_usb_phy->dev			= &pdev->dev;
	rtk_usb_phy->phy.dev		= rtk_usb_phy->dev;
	rtk_usb_phy->phy.label		= RTK_USB2PHY_NAME;
	rtk_usb_phy->phy.init		= rtk_usb2_phy_init;
	rtk_usb_phy->phy.shutdown	= rtk_usb2_phy_shutdown;
	/* notify phy port status change */
	rtk_usb_phy->phy.notify_port_change = rtk_usb2_phy_notify_port_change;

	if (!dev->of_node) {
		dev_err(dev, "%s %d No device node\n", __func__, __LINE__);
		goto err;
	}

	ret = of_property_read_u32_index(dev->of_node, "port_index", 0,
		    &port_index);
	if (ret)
		port_index = -1;

	ret = of_property_read_u32_index(dev->of_node, "phyN", 0,
		    &phyN);
	if (ret)
		goto err;

	dev_dbg(dev, "%s %d port_index=%d phyN=%d\n",
		    __func__, __LINE__, port_index, phyN);

	rtk_usb_phy->port_index = port_index;
	rtk_usb_phy->phyN = phyN;
	rtk_usb_phy->reg_addr = devm_kzalloc(dev,
		    sizeof(struct reg_addr) * phyN, GFP_KERNEL);
	if (!rtk_usb_phy->reg_addr)
		return -ENOMEM;

	rtk_usb_phy->phy_data = devm_kzalloc(dev,
		    sizeof(struct phy_data) * phyN,
		    GFP_KERNEL);

	if (!rtk_usb_phy->phy_data)
		return -ENOMEM;

	for (index = 0; index < phyN; index++) {
		ret = __get_phy_parameter(rtk_usb_phy, index);
		if (ret) {
			dev_err(dev, "%s %d: __get_phy_parameter fail ret=%d\n",
				    __func__, __LINE__, ret);
			goto err;
		}
	}

	platform_set_drvdata(pdev, rtk_usb_phy);

	ret = usb_add_phy_dev(&rtk_usb_phy->phy);
	if (ret)
		goto err;

#ifdef CONFIG_DEBUG_FS
	create_debug_files(rtk_usb_phy);
#endif

	dev_info(&pdev->dev, "%s Probe RTK USB 2.0 PHY\n", __FILE__);
err:
	return ret;
}

static int rtk_usb2phy_remove(struct platform_device *pdev)
{
	struct rtk_usb_phy_s *rtk_usb_phy = platform_get_drvdata(pdev);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(rtk_usb_phy->debug_dir);
#endif

	usb_remove_phy(&rtk_usb_phy->phy);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id usbphy_rtk_dt_match[] = {
	{ .compatible = "realtek,usb2phy", },
	{},
};
MODULE_DEVICE_TABLE(of, usbphy_rtk_dt_match);
#endif

static struct platform_driver rtk_usb2phy_driver = {
	.probe		= rtk_usb2phy_probe,
	.remove		= rtk_usb2phy_remove,
	.driver		= {
		.name	= RTK_USB2PHY_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(usbphy_rtk_dt_match),
	},
};

module_platform_driver(rtk_usb2phy_driver);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" RTK_USB2PHY_NAME);
