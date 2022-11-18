// SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause)
/*
 * Copyright (c) 2008 Kevin Wang <kevin_wang@realtek.com.tw>
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 */

#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/pinctrl/consumer.h>
#include <soc/realtek/rtk_chip.h>
#include "scd_mars_priv.h"
#include "scd_mars_reg.h"

MODULE_LICENSE("GPL");

MARS_DTS_INFO_T dts_info;

/*------------------------------------------------------------------
 * Func : ops_probe
 *
 * Desc : probe a scd device
 *
 * Parm : dev
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_probe(scd_device* dev)
{
	mars_scd* p_scd;

	if ((dev->id & 0xFFFFFFFC)!=0x12830000)
		return -ENODEV;

	p_scd = mars_scd_open((dev->id & 0x3));

	if (p_scd == NULL)
		return -ENOMEM;

	scd_set_drvdata(dev, p_scd);

	return 0;
}

/*------------------------------------------------------------------
 * Func : ops_remove
 *
 * Desc : this function will be invoked when a smart card device which
 *        associated with this driver has been removed
 *
 * Parm : dev : smart card device to be removed
 *
 * Retn : N/A
 *------------------------------------------------------------------*/
static void ops_remove(scd_device* dev)
{
	mars_scd_close((mars_scd*) scd_get_drvdata(dev));
}

/*------------------------------------------------------------------
 * Func : ops_enable
 *
 * Desc : enable IFD
 *
 * Parm : dev    : ifd device
 *        on_off : 0      : disable IFD
 *                 others : enable IFD
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_enable(scd_device* dev, unsigned char on_off)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	return mars_scd_enable(p_this, on_off);
}

/*------------------------------------------------------------------
 * Func : ops_set_param
 *
 * Desc : set param of IFD
 *
 * Parm : dev
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_set_param(scd_device* dev, unsigned long id, unsigned long val)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	int ret = SC_SUCCESS;

	switch(id) {
	case SC_PARAM_VCC_LVL:
		ret = mars_scd_set_vcc_lvl(p_this, (SC_VCC_LVL) val);
		break;

	case SC_PARAM_CLOCK:
		ret = mars_scd_set_clock(p_this, (unsigned long) val);
		break;

	case SC_PARAM_ETU:
		ret = mars_scd_set_etu(p_this, (unsigned long) val);
		break;

	case SC_PARAM_CONV:
		ret = mars_scd_set_convention(p_this, (SC_CONV) val);
		break;

	case SC_PARAM_PARITY:
		ret = mars_scd_set_parity(p_this, (unsigned char) val);
		break;

	case SC_PARAM_GUARD_INTERVAL:
		ret = mars_scd_set_guard_interval(p_this, val);
		// do nothing
		break;

	case SC_PARAM_BWI:
		ret = mars_scd_set_bwi(p_this, val);
		break;

	case SC_PARAM_CWI:
		ret = mars_scd_set_cwi(p_this, val);
		break;

	case SC_PARAM_BGT:
		ret = mars_scd_set_bgt(p_this, val);
		break;

	case SC_PARAM_PIN:
		ret = mars_scd_set_pin(p_this, val);
		break;

	case SC_PARAM_PWR:
		ret = mars_scd_set_pwr(p_this, val);
		break;

	case SC_PARAM_WWI:
		ret = mars_scd_set_wwi(p_this, val);
		break;

	default:
		ret = SC_FAIL;
	}

	return ret;
}

/*------------------------------------------------------------------
 * Func : ops_get_param
 *
 * Desc : get param of IFD
 *
 * Parm : dev
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_get_param(scd_device* dev, unsigned long id, unsigned long* p_val)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	int ret = SC_SUCCESS;

	switch(id) {
	case SC_PARAM_VCC_LVL:
		ret = mars_scd_get_vcc_lvl(p_this, p_val);
		break;

	case SC_PARAM_CLOCK:
		ret = mars_scd_get_clock(p_this,  p_val);
		break;

	case SC_PARAM_ETU:
		ret = mars_scd_get_etu(p_this, p_val);
		break;

	case SC_PARAM_CONV:
		ret = mars_scd_get_convention(p_this, (SC_CONV*) p_val);
		break;

	case SC_PARAM_PARITY:
		ret = mars_scd_get_parity(p_this, (SC_PARITY*) p_val);
		break;

	case SC_PARAM_GUARD_INTERVAL:
		//*p_val = 2;
		ret = mars_scd_get_guard_interval(p_this, p_val);
		break;

	case SC_PARAM_BWI:
		ret = mars_scd_get_bwi(p_this, p_val);
		break;

	case SC_PARAM_CWI:
		ret = mars_scd_get_cwi(p_this, p_val);
		break;

	case SC_PARAM_BGT:
		ret = mars_scd_get_bgt(p_this, p_val);
		break;

	case SC_PARAM_PIN:
		ret = mars_scd_get_pin(p_this, p_val);
		break;

	case SC_PARAM_PWR:
                ret = mars_scd_get_pwr(p_this, p_val);
                break;

	case SC_PARAM_WWI:
                ret = mars_scd_get_wwi(p_this, p_val);
                break;

	default:
		ret = SC_FAIL;
	}

	return ret;
}

/*------------------------------------------------------------------
 * Func : ops_set_protocol
 *
 * Desc : set HW protocl
 *
 * Parm : dev
 *
 * Parm : val
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static
int ops_set_protocol(scd_device* dev, unsigned int val)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	return mars_scd_set_protocol(p_this, val);
}
/*------------------------------------------------------------------
 * Func : ops_get_protocol
 *
 * Desc : activate ICC via reset procedure
 *
 * Parm : dev
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_get_protocol(scd_device* dev, unsigned int* p_val)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	return mars_scd_get_protocol(p_this, p_val);
}

/*------------------------------------------------------------------
 * Func : ops_activate
 *
 * Desc : activate ICC via reset procedure
 *
 * Parm : dev
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_activate(scd_device* dev)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	return mars_scd_activate(p_this);
}

/*------------------------------------------------------------------
 * Func : ops_deactivate
 *
 * Desc : deactivate icc
 *
 * Parm : dev
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_deactivate(scd_device* dev)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	return mars_scd_deactivate(p_this);
}

/*------------------------------------------------------------------
 * Func : ops_reset
 *
 * Desc : reset a scd device
 *
 * Parm : dev
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_reset(scd_device* dev)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	return mars_scd_reset(p_this);
}

/*------------------------------------------------------------------
 * Func : ops_get_atr
 *
 * Desc : get atr from a scd device
 *
 * Parm : dev
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_get_atr(scd_device* dev, scd_atr* p_atr)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	return mars_scd_get_atr(p_this, p_atr);
}

/*------------------------------------------------------------------
 * Func : ops_get_card_status
 *
 * Desc : get card status of IFD
 *
 * Parm : dev
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_get_card_status(scd_device* dev)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	return mars_scd_get_card_status(p_this);
}

/*------------------------------------------------------------------
 * Func : ops_poll_card_status
 *
 * Desc : poll card status
 *
 * Parm : dev  :
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_poll_card_status(scd_device* dev)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	return mars_scd_poll_card_status(p_this);
}

/*------------------------------------------------------------------
 * Func : ops_xmit
 *
 * Desc : xmit data via smart card bus
 *
 * Parm : dev  :
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_xmit(scd_device* dev, unsigned char* p_data, unsigned int len)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	return mars_scd_xmit(p_this, p_data, len);
}

/*------------------------------------------------------------------
 * Func : ops_read
 *
 * Desc : read data via smart card bus
 *
 * Parm : dev  :
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
static int ops_read(scd_device* dev, unsigned char* p_data, unsigned int len)
{
	mars_scd* p_this = (mars_scd*) scd_get_drvdata(dev);
	return mars_scd_read(p_this, p_data, len);
}

static scd_device mars_scd_controller[2] =
{
	{
		.id   = ID_MARS_SCD(0),
		.name = "mars_scd_0"
	},
	{
		.id   = ID_MARS_SCD(1),
		.name = "mars_scd_1"
	},
};

static scd_driver mars_scd_driver =
{
	.name		   = "mars_scd",
	.probe		   = ops_probe,
	.remove		   = ops_remove,
	.suspend	   = NULL,
	.resume		   = NULL,
	.enable		   = ops_enable,
	.set_param	   = ops_set_param,
	.get_param	   = ops_get_param,
	.set_protocol	   = ops_set_protocol,
	.get_protocol	   = ops_get_protocol,
	.activate	   = ops_activate,
	.deactivate	   = ops_deactivate,
	.reset		   = ops_reset,
	.get_atr	   = ops_get_atr,
	.get_card_status   = ops_get_card_status,
	.poll_card_status  = ops_poll_card_status,
	.xmit		   = ops_xmit,
	.read		   = ops_read,
};

/*------------------------------------------------------------------
 * Func : mars_scd_module_init
 *
 * Desc : init mars smart card interface driver
 *
 * Parm : N/A
 *
 * Retn : 0 : success, others fail
 *------------------------------------------------------------------*/
static int mars_scd_module_init(void)
{
	SC_INFO("mars scd module init\n");

	if (register_scd_driver(&mars_scd_driver)!=0)
		return -EFAULT;

	register_scd_device(&mars_scd_controller[0]);          // register scd device
	//register_scd_device(&mars_scd_controller[1]);          // register scd device

	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_module_exit
 *
 * Desc : uninit mars smart card interface driver
 *
 * Parm : N/A
 *
 * Retn : 0 : success, others fail
 *------------------------------------------------------------------*/
static void mars_scd_module_exit(void)
{
	SC_INFO("mars scd module init\n");

	unregister_scd_device(&mars_scd_controller[0]);
	//unregister_scd_device(&mars_scd_controller[1]);
	unregister_scd_driver(&mars_scd_driver);
}

static int rtk_smc_probe(struct platform_device *pdev)
{
	struct device_node *ls_ic_node;
	unsigned int val = 0;
	struct gpio_desc *cmd_vcc_gpio;
	struct gpio_desc *pwr_sel0_gpio;
	struct gpio_desc *pwr_sel1_gpio;

	memset(&dts_info, 0, sizeof(dts_info));

	dts_info.chip_id = get_rtd_chip_id();

	dts_info.misc = of_iomap(pdev->dev.of_node, 0);
	if (!dts_info.misc) {
		printk(KERN_ERR "gpio iomap fail.\n");
		return -EINVAL;
	}

	dts_info.base = of_iomap(pdev->dev.of_node, 1);
	if (!dts_info.base) {
		printk(KERN_ERR "base iomap fail.\n");
		return -EINVAL;
	}

	dts_info.irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (dts_info.irq < 0) {
		printk(KERN_ERR "smc no irq.\n");
		return -EINVAL;
	}

	if(of_property_read_u32(pdev->dev.of_node, "smc-num", &dts_info.id)){
		printk(KERN_ERR "Get SMC ID fail.\n");
		return -EINVAL;
	}

	dts_info.clk = of_clk_get_by_name(pdev->dev.of_node, "clk_en_misc_sc");
	if (IS_ERR(dts_info.clk)){
		dev_err(&pdev->dev, "could not get clk\n");
		return -EINVAL;
	}

	dts_info.clk_sel = of_clk_get_by_name(pdev->dev.of_node, "sel");
	if (IS_ERR(dts_info.clk_sel)){
		dev_err(&pdev->dev, "could not get clk sel\n");
		return -EINVAL;
	}

	dts_info.rstc = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(dts_info.rstc)) {
		dev_err(&pdev->dev, "could not get reset_control\n");
		return -EINVAL;
	}

	/* Get gpio config */
	ls_ic_node = of_get_child_by_name(pdev->dev.of_node, "lsic_control");
	if (!ls_ic_node) {
		dev_err(&pdev->dev, "could not find [ls_ic_node] sub-node\n");
		return -EINVAL;
	}

	if (!of_property_read_u32(ls_ic_node, "cmd_vcc_en", &val))
		dts_info.cmd_vcc_en = val;
	if (!of_property_read_u32(ls_ic_node, "cmd_vcc_polarity", &val))
		dts_info.cmd_vcc_polarity = val;

	dts_info.pin_cmd_vcc = of_get_named_gpio(ls_ic_node, "pin_cmd_vcc", 0);

	if (!of_property_read_u32(ls_ic_node, "pwr_sel_en", &val))
		dts_info.pwr_sel_en = val;
	if (!of_property_read_u32(ls_ic_node, "pwr_sel_polarity", &val))
		dts_info.pwr_sel_polarity = val;

	cmd_vcc_gpio = gpiod_get_from_of_node(ls_ic_node, "pin_cmd_vcc", 0,
						GPIOD_OUT_LOW, "pin_cmd_vcc");
	if (IS_ERR(cmd_vcc_gpio)) {
		dev_err(&pdev->dev, "pin_cmd_vcc gpio missing or invalid\n");
		return -EPROBE_DEFER;
	}

	pwr_sel0_gpio = gpiod_get_from_of_node(ls_ic_node, "pin_pwr_sel0", 0,
						GPIOD_OUT_LOW, "pin_pwr_sel0");
	if (IS_ERR(pwr_sel0_gpio)) {
		dev_err(&pdev->dev, "pin_pwr_sel0 gpio missing or invalid\n");
		return -EPROBE_DEFER;
	}

	pwr_sel1_gpio = gpiod_get_from_of_node(ls_ic_node, "pin_pwr_sel1", 0,
						GPIOD_OUT_LOW, "pin_pwr_sel1");
	if (IS_ERR(pwr_sel1_gpio)) {
		dev_err(&pdev->dev, "pin_pwr_sel1 gpio missing or invalid\n");
		return -EPROBE_DEFER;
	}

	dts_info.pin_cmd_vcc = desc_to_gpio(cmd_vcc_gpio);
	dts_info.pin_pwr_sel0 = desc_to_gpio(pwr_sel0_gpio);
	if (dts_info.chip_id == CHIP_ID_RTD1319D)
		dts_info.pin_pwr_sel1 = desc_to_gpio(pwr_sel1_gpio);

	dts_info.pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(dts_info.pinctrl))
		dev_err(&pdev->dev, "no pinctrl\n");

	dts_info.pins_default = pinctrl_lookup_state(dts_info.pinctrl,
							PINCTRL_STATE_DEFAULT);
	if (IS_ERR(dts_info.pins_default))
		dev_err(&pdev->dev, "could not get default state\n");

	dts_info.pins_data0 = pinctrl_lookup_state(dts_info.pinctrl,
								"sc_data0");
	if (IS_ERR(dts_info.pins_data0))
		dev_err(&pdev->dev, "could not get data0 state\n");

	dts_info.pins_data1 = pinctrl_lookup_state(dts_info.pinctrl,
								"sc_data1");
	if (IS_ERR(dts_info.pins_data1))
		dev_err(&pdev->dev, "could not get data1 state\n");

	dts_info.pins_data2 = pinctrl_lookup_state(dts_info.pinctrl,
								"sc_data2");
	if (IS_ERR(dts_info.pins_data2))
		dev_err(&pdev->dev, "could not get data2 state\n");

	if(mars_scd_module_init() != 0){
		printk(KERN_ERR "register driver fail.\n");
		return -EINVAL;
	}

	return 0;
}

static int rtk_smc_remove(struct platform_device *pdev)
{
	mars_scd_module_exit();

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id rtk_smc_of_match[] = {
	{ .compatible = "Realtek,rtk-smc", },
	{},
};
MODULE_DEVICE_TABLE(of, rtk_smc_of_match);

static struct platform_driver rtk_smc_driver = {
	.probe          = rtk_smc_probe,
	.remove         = rtk_smc_remove,
	.driver         = {
		.name   = "rtk_smc",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(rtk_smc_of_match),
	},
};
module_platform_driver(rtk_smc_driver);

MODULE_DESCRIPTION("Realtek Smart Card Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk_smc");
