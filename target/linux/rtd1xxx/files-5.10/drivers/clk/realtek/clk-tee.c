// SPDX-License-Identifier: GPL-2.0-only
/*
 * cc-tee.c - TEE clock controller
 *
 * Copyright (C) 2019 Realtek Semiconductor Corporation
 * Author: Cheng-Yu Lee <cylee12@realtek.com>
 */

#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/uuid.h>

#define HZ_PER_MHZ            1000000
#define TA_CMD_SCPU_PLL       5

struct ta_clk_data {
	struct clk_hw hw;
	struct device *dev;
	unsigned long freq_mhz;
	struct tee_context *ctx;
	u32 session_id;
};

static const uuid_t uuid = UUID_INIT(0x650b79a1, 0xa79a, 0x43ea,
	0x91, 0x85, 0xf6, 0x67, 0x55, 0x65, 0x64, 0xa7);

static int ta_clk_optee_match(struct tee_ioctl_version_data *vers,
			      const void *data)
{
	if (vers->impl_id == TEE_IMPL_ID_OPTEE)
		return 1;
	return 0;
}

static int ta_clk_init(struct ta_clk_data *tcd)
{
	struct tee_ioctl_version_data vers = {
		.impl_id   = TEE_IMPL_ID_OPTEE,
		.impl_caps = TEE_OPTEE_CAP_TZ,
		.gen_caps  = TEE_GEN_CAP_GP,
	};
	struct tee_ioctl_open_session_arg arg = { 0 };
	int ret;

	tcd->ctx = tee_client_open_context(NULL, ta_clk_optee_match, NULL,
					   &vers);
	if (IS_ERR(tcd->ctx)) {
		ret = PTR_ERR(tcd->ctx);
		dev_err(tcd->dev, "failed to open context: %d\n", ret);
		return ret;
	}

	memcpy(arg.uuid, uuid.b, TEE_IOCTL_UUID_LEN);
	arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	arg.num_params = 0;

	ret = tee_client_open_session(tcd->ctx, &arg, NULL);
	if (ret) {
		dev_err(tcd->dev, "failed to open session: %d\n", ret);
		return ret;
	}

	tcd->session_id = arg.session;
	return 0;
}

static void ta_clk_fini(struct ta_clk_data *tcd)
{
	tee_client_close_session(tcd->ctx, tcd->session_id);
	tee_client_close_context(tcd->ctx);
}

static int ta_clk_set_pll_scpu_freq(struct ta_clk_data *tcd, unsigned int freq)
{
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int ret;
	int retry = 0;

again:
	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = TA_CMD_SCPU_PLL;
	inv_arg.session = tcd->session_id;
	inv_arg.num_params = 4;

	param[0].u.value.a = freq;
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(tcd->ctx, &inv_arg, param);
	if (inv_arg.ret != 0 && retry == 0) {
		dev_warn(tcd->dev, "failed to invoke, retry\n");
		ta_clk_init(tcd);
		retry++;
		goto again;
	}

	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(tcd->dev, "failed to invoke func: %d, %x\n", ret,
			inv_arg.ret);
		return -EINVAL;
	}

	if (param[0].u.value.b) {
		dev_err(tcd->dev, "failed to set freq: %lld\n",
			param[0].u.value.b);
		return -EINVAL;
	}

	return 0;
}


static long ta_clk_round_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long *parent_rate)
{
	return DIV_ROUND_CLOSEST(rate, 100000000) * 100000000;
}

static unsigned long ta_clk_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct ta_clk_data *tcd = container_of(hw, struct ta_clk_data, hw);

	dev_dbg(tcd->dev, "freq = %ld\n", tcd->freq_mhz * HZ_PER_MHZ);
	return tcd->freq_mhz * HZ_PER_MHZ;
}

static int ta_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			   unsigned long parent_rate)
{
	struct ta_clk_data *tcd = container_of(hw, struct ta_clk_data, hw);
	unsigned long freq_mhz = rate / HZ_PER_MHZ;
	int ret;
	ktime_t start;
	s64 delta_us;

	dev_dbg(tcd->dev, "enter %s (freq=%lu)\n", __func__, freq_mhz);

	start = ktime_get();
	ret = ta_clk_set_pll_scpu_freq(tcd, freq_mhz);
	if (ret)
		goto done;
	tcd->freq_mhz = freq_mhz;
done:
	delta_us = ktime_to_us(ktime_sub(ktime_get(), start));
	dev_dbg(tcd->dev, "exit %s (freq=%lu, time=%lld, ret=%d)\n", __func__,
		freq_mhz, delta_us,  ret);
	return ret;
}

static const struct clk_ops ta_clk_ops = {
	.round_rate       = ta_clk_round_rate,
	.recalc_rate      = ta_clk_recalc_rate,
	.set_rate         = ta_clk_set_rate,
};

static struct ta_clk_data pll_scpu = {
	.freq_mhz = 1100,
	.hw.init = CLK_HW_INIT_NO_PARENT("pll_scpu", &ta_clk_ops, 0),
};

static int rtk_tee_cc_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct clk *clk;
	struct ta_clk_data *tcd = &pll_scpu;

	tcd->dev = dev;

	ret = ta_clk_init(tcd);
	if (ret)
		return -EPROBE_DEFER;

	ret = ta_clk_set_pll_scpu_freq(tcd, tcd->freq_mhz);
	if (ret)
		goto error;

	clk = clk_register(NULL, &pll_scpu.hw);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dev, "failed to register clk: %d\n", ret);
		goto error;
	}
	platform_set_drvdata(pdev, clk);

	of_clk_add_provider(dev->of_node, of_clk_src_simple_get, clk);
	clk_register_clkdev(clk, __clk_get_name(clk), NULL);

	dev_info(dev, "ready\n");
	return 0;

error:
	ta_clk_fini(tcd);
	return ret;
}

static const struct of_device_id rtk_tee_cc_match[] = {
	{ .compatible = "realtek,tee-clock-controller", },
	{}
};

static struct platform_driver rtk_tee_cc_driver = {
	.probe = rtk_tee_cc_probe,
	.driver = {
		.name = "rtk-tee-cc",
		.of_match_table = of_match_ptr(rtk_tee_cc_match),
	},
};

static int __init rtk_tee_cc_init(void)
{
	return platform_driver_register(&rtk_tee_cc_driver);
}
module_init(rtk_tee_cc_init);

MODULE_DESCRIPTION("Reatek OPTee Clock Controller");
MODULE_AUTHOR("Cheng-Yu Lee <cylee12@realtek.com>");
MODULE_LICENSE("GPL v2");
