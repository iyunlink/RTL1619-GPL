// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Realtek pulse-width-modulation controller driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

/* PWM V1 */
#define RTK_ADDR_PWM_V1_OCD	(0x0)
#define RTK_ADDR_PWM_V1_CD	(0x4)
#define RTK_ADDR_PWM_V1_CSD	(0x8)

#define RTK_PWM_V1_OCD_SHIFT	(8)
#define RTK_PWM_V1_CD_SHIFT	(8)
#define RTK_PWM_V1_CSD_SHIFT	(4)

#define RTK_PWM_V1_OCD_MASK	(0xff)
#define RTK_PWM_V1_CD_MASK	(0xff)
#define RTK_PWM_V1_CSD_MASK	(0xf)

/* PWM V2 */
#define RTK_ADDR_PWM0_V2	(0x4)
#define RTK_ADDR_PWM1_V2	(0x8)
#define RTK_ADDR_PWM2_V2	(0xc)
#define RTK_ADDR_PWM3_V2	(0x0)

#define RTK_PWM_V2_OCD_SHIFT	(8)
#define RTK_PWM_V2_CD_SHIFT	(0)
#define RTK_PWM_V2_CSD_SHIFT	(16)

#define RTK_PWM_V2_OCD_MASK	(0xff)
#define RTK_PWM_V2_CD_MASK	(0xff)
#define RTK_PWM_V2_CSD_MASK	(0x1f)

/* COMMON */
#define RTK_PWM_OCD_DEFAULT	(0xff)
#define RTK_PWM_CD_DEFAULT	(0x1)
#define RTK_PWM_CSD_DEFAULT	(0x1)

#define NUM_PWM 4
#define DEV_NAME "[RTK_PWM]"
#define RTK_27MHZ		27000000
#define RTK_1SEC_TO_NS		1000000000

#define RTK_PWM_V1_UPD		0x54

struct rtk_pwm_map {
	int duty_rate;
	int ocd_data;
	int cd_data;
};

struct rtk_pwm_chip {
	struct pwm_chip		chip;
	struct device		*dev;
	void __iomem		*mmio_pwm_reg_base;
	struct regmap		*iso_base;
	spinlock_t		lock;
	int			proc_id;
	int			base_freq;
	int			out_freq[NUM_PWM]; /* Hz */
	int			duty_rate[NUM_PWM];
	int			enable[NUM_PWM];
	int			clksrc_div[NUM_PWM];
	int			clkout_div[NUM_PWM];
	int			clk_duty[NUM_PWM];
	int			flags[NUM_PWM];
	void (*pwm_reg_set)(struct rtk_pwm_chip *pc, int hwpwm);
};

#define RTK_PWM_FLAGS_PULSE_LOW_ON_CHANGE         0x1

#define to_rtk_pwm_chip(d) container_of(d, struct rtk_pwm_chip, chip)

int set_real_freq_by_target_freq(struct rtk_pwm_chip *pc, int hwpwm,
				 int target_freq)
{
	int base_freq = pc->base_freq;
	int real_freq;
	int ocd, csd, div, opt_div;
	int min_ocd = 0;
	int max_ocd = 255;
	int min_csd = 0;
	int max_csd = 15;
	int i;

	div = base_freq / target_freq;

	/* give a div to get max ocd and min csd */

	/* find max bit */
	for (i = 0; i < 32; i++) {
		if ((div << i) & BIT(31))
			break;
	}
	csd = (32 - (i + 8)) - 1;
	csd = clamp(csd, min_csd, max_csd);

	ocd = (div >> (csd + 1)) - 1;
	ocd = clamp(ocd, min_ocd, max_ocd);

	opt_div = BIT(csd + 1) * (ocd + 1);

	real_freq = base_freq / opt_div;
	pc->clkout_div[hwpwm] = ocd;
	pc->clksrc_div[hwpwm] = csd;

	pc->out_freq[hwpwm] = real_freq;

	return real_freq;
}

int set_real_period_by_target_period(struct rtk_pwm_chip *pc, int hwpwm,
				     int target_period_ns)
{
	int base_ns = RTK_1SEC_TO_NS;
	int real_period_ns;
	int target_freq, real_freq;

	target_freq = base_ns / target_period_ns;
	real_freq = set_real_freq_by_target_freq(pc, hwpwm, target_freq);
	real_period_ns = base_ns / real_freq;

	return real_period_ns;
}

int set_real_freq_by_target_div(struct rtk_pwm_chip *pc, int hwpwm,
				int clksrc_div, int clkout_div)
{
	int base_freq = pc->base_freq;
	int real_freq, div;

	div = BIT(clksrc_div + 1) * (clkout_div + 1);
	real_freq = base_freq / div;

	pc->clkout_div[hwpwm] = clkout_div;
	pc->clksrc_div[hwpwm] = clksrc_div;

	pc->out_freq[hwpwm] = real_freq;

	return real_freq;
}

int set_clk_duty(struct rtk_pwm_chip *pc, int hwpwm, int duty_rate)
{
	int clkout_div = pc->clkout_div[hwpwm];
	int cd;

	duty_rate = clamp(duty_rate, 0, 100);

	pc->duty_rate[hwpwm] = duty_rate;
	cd = (duty_rate * (clkout_div + 1) / 100) - 1;

	pc->clk_duty[hwpwm] = clamp(cd, 0, clkout_div);

	return 0;
}

static inline void pwm_update_v1(struct rtk_pwm_chip *pc, int hwpwm)
{
	u32 val;

	if (IS_ERR_OR_NULL(pc->iso_base))
		return;

	val = 1 << hwpwm;
	regmap_update_bits_base(pc->iso_base, RTK_PWM_V1_UPD,
				val, val, NULL, false, true);
}

static void pwm_reg_set_v1(struct rtk_pwm_chip *pc, int hwpwm)
{
	u32 value;
	u32 old_val;
	int clkout_div = 0;
	int clk_duty = 0;
	int clksrc_div = 0;
	unsigned long flags;

	if (pc->enable[hwpwm] && pc->duty_rate[hwpwm]) {
		clkout_div = pc->clkout_div[hwpwm];
		clk_duty = pc->clk_duty[hwpwm];
		clksrc_div = pc->clksrc_div[hwpwm];
	}

	spin_lock_irqsave(&pc->lock, flags);

	value = readl(pc->mmio_pwm_reg_base + RTK_ADDR_PWM_V1_OCD);
	old_val = value;
	value &= ~(RTK_PWM_V1_OCD_MASK << (hwpwm * RTK_PWM_V1_OCD_SHIFT));
	value |= clkout_div << (hwpwm * RTK_PWM_V1_OCD_SHIFT);
	if (value != old_val)
		writel(value, pc->mmio_pwm_reg_base + RTK_ADDR_PWM_V1_OCD);

	value = readl(pc->mmio_pwm_reg_base + RTK_ADDR_PWM_V1_CD);
	old_val = value;
	value &= ~(RTK_PWM_V1_CD_MASK << (hwpwm * RTK_PWM_V1_CD_SHIFT));
	value |= clk_duty << (hwpwm * RTK_PWM_V1_CD_SHIFT);
	if (value != old_val)
		writel(value, pc->mmio_pwm_reg_base + RTK_ADDR_PWM_V1_CD);

	value = readl(pc->mmio_pwm_reg_base + RTK_ADDR_PWM_V1_CSD);
	old_val = value;
	value &= ~(RTK_PWM_V1_CSD_MASK << (hwpwm * RTK_PWM_V1_CSD_SHIFT));
	value |= clksrc_div << (hwpwm * RTK_PWM_V1_CSD_SHIFT);
	if (value != old_val)
		writel(value, pc->mmio_pwm_reg_base + RTK_ADDR_PWM_V1_CSD);

	pwm_update_v1(pc, hwpwm);
	spin_unlock_irqrestore(&pc->lock, flags);
}

static void pwm_reg_set_v2(struct rtk_pwm_chip *pc, int hwpwm)
{
	void __iomem *reg;
	u32 value;
	u32 old_val;
	int clkout_div = 0;
	int clk_duty = 0;
	int clksrc_div = 0;
	unsigned long flags;

	if (pc->enable[hwpwm] && pc->duty_rate[hwpwm]) {
		clkout_div = pc->clkout_div[hwpwm];
		clk_duty = pc->clk_duty[hwpwm];
		clksrc_div = pc->clksrc_div[hwpwm];
	}

	switch (hwpwm) {
	case 0:
		reg = pc->mmio_pwm_reg_base + RTK_ADDR_PWM0_V2;
		break;
	case 1:
		reg = pc->mmio_pwm_reg_base + RTK_ADDR_PWM1_V2;
		break;
	case 2:
		reg = pc->mmio_pwm_reg_base + RTK_ADDR_PWM2_V2;
		break;
	case 3:
		reg = pc->mmio_pwm_reg_base + RTK_ADDR_PWM3_V2;
		break;
	default:
		dev_err(pc->dev, "invalid index: PWM %d\n", hwpwm);
		return;
	}

	spin_lock_irqsave(&pc->lock, flags);

	old_val = readl(reg);
	value = ((clksrc_div & RTK_PWM_V2_CSD_MASK) << RTK_PWM_V2_CSD_SHIFT) |
		((clkout_div & RTK_PWM_V2_OCD_MASK) << RTK_PWM_V2_OCD_SHIFT) |
		((clk_duty & RTK_PWM_V2_CD_MASK) << RTK_PWM_V2_CD_SHIFT);
	if (value != old_val)
		writel(value, reg);

	spin_unlock_irqrestore(&pc->lock, flags);
}

static void pwm_set_register(struct rtk_pwm_chip *pc, int hwpwm)
{
	if (pc == NULL) {
		pr_err("%s %s -- parameter error!--\n", DEV_NAME, __func__);
		return;
	}

	pc->pwm_reg_set(pc, hwpwm);
}

static int rtk_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			  int duty_ns, int period_ns)
{
	struct rtk_pwm_chip *pc = to_rtk_pwm_chip(chip);
	int real_period_ns, duty_rate;
	int hwpwm = pwm->hwpwm;
	int clkout_div = pc->clkout_div[hwpwm];
	int cd;

	real_period_ns =
		set_real_period_by_target_period(pc, pwm->hwpwm, period_ns);

	duty_rate = (int)div64_s64((s64)duty_ns * 100, period_ns);

	/* improve accuracy */
	pc->duty_rate[hwpwm] = duty_rate;
	cd = (int)DIV_ROUND_CLOSEST_ULL((unsigned long long)duty_ns * (clkout_div + 1),  period_ns) - 1;
	pc->clk_duty[hwpwm] = clamp(cd, 0, clkout_div);

	pwm_set_register(pc, pwm->hwpwm);

	return 0;
}

static int rtk_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rtk_pwm_chip *pc = to_rtk_pwm_chip(chip);

	pc->enable[pwm->hwpwm] = 1;
	pwm_set_register(pc, pwm->hwpwm);

	return 0;
}

static void rtk_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rtk_pwm_chip *pc = to_rtk_pwm_chip(chip);

	pc->enable[pwm->hwpwm] = 0;
	pwm_set_register(pc, pwm->hwpwm);

}

static void rtk_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			      struct pwm_state *state)
{
	struct rtk_pwm_chip *pc = to_rtk_pwm_chip(chip);
	int id = pwm->hwpwm;
	int base_freq = pc->base_freq;
	int real_freq, div;

	if (pc->enable[id] == 0) {
		state->period     = 0;
		state->duty_cycle = 0;
		state->enabled    = 0;
		return;
	}

	div = BIT(pc->clksrc_div[id] + 1) * (pc->clkout_div[id] + 1);
	real_freq = base_freq / div;

	state->period = DIV_ROUND_UP(RTK_1SEC_TO_NS, real_freq);
	state->duty_cycle = DIV_ROUND_UP_SECTOR_T(state->period * pc->duty_rate[id], 100);
	state->enabled = pc->enable[id];
}

static const struct pwm_ops rtk_pwm_ops = {
	.config = rtk_pwm_config,
	.enable = rtk_pwm_enable,
	.disable = rtk_pwm_disable,
	.get_state = rtk_pwm_get_state,
	.owner = THIS_MODULE,
};

/** define show/store API for each file here **/
static ssize_t duty_rate_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf, int hwpwm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_pwm_chip *pc = platform_get_drvdata(pdev);

	return sprintf(buf, "%d%%\n", pc->duty_rate[hwpwm]);
}

static ssize_t duty_rate_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count, int hwpwm)
{
	int value = 0;
	int ret = -1;
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_pwm_chip *pc = platform_get_drvdata(pdev);

	if (count < 1) {
		pr_err("%s %s , count is too small, return\n",
		       DEV_NAME, __func__);
		return count;
	}
	ret = kstrtoint(buf, 10, &value);
	if (ret != 0) {
		pr_err("%s %s , parse buf error! ret=%d\n",
		       DEV_NAME, __func__, ret);
		return count;
	}
	if (value < 0 || value > 100) {
		pr_err("%s %s  ====  input should between 0 ~ 100\n",
		       DEV_NAME, __func__);
		return count;
	}
	if (pc->duty_rate[hwpwm] == value) {
		pr_info(" %s %s -- duty_rate value is not change, return!\n",
			DEV_NAME, __func__);
		return count;
	}
	pr_info("%s %s -- assign [%d] to duty_rate now!\n",
		DEV_NAME, __func__, value);

	set_clk_duty(pc, hwpwm, value);
	pwm_set_register(pc, hwpwm);

	return count;
}

static ssize_t pwm_enable_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf, int hwpwm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_pwm_chip *pc = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", pc->enable[hwpwm]);
}

static ssize_t pwm_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count, int hwpwm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_pwm_chip *pc = platform_get_drvdata(pdev);
	int value = 0;

	if (buf == NULL) {
		pr_err("%s %s	====  buffer is null, return\n",
		       DEV_NAME, __func__);
		return count;
	}
	sscanf(buf, "%d", &value);

	if (pc->enable[hwpwm] == value) {
		pr_info("%s %s	====  the same, do nothing (hwpwm=%d, value=%d)\n",
			DEV_NAME, __func__, hwpwm, value);
		return count;
	}

	pc->enable[hwpwm] = value;
	pwm_set_register(pc, hwpwm);

	return count;
}

static ssize_t clk_src_div_show(struct device *dev,
				struct device_attribute *attr,
				char *buf, int hwpwm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_pwm_chip *pc = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", pc->clksrc_div[hwpwm]);
}

static ssize_t clk_src_div_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count, int hwpwm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_pwm_chip *pc = platform_get_drvdata(pdev);

	int value = 0;
	int max_val;

	if (buf == NULL) {
		pr_err("%s %s  ====  buffer is null, return\n",
		       DEV_NAME, __func__);
		return count;
	}
	sscanf(buf, "%d", &value);

	if (pc->pwm_reg_set == pwm_reg_set_v1)
		max_val = RTK_PWM_V1_CSD_MASK;
	else
		max_val = RTK_PWM_V2_CSD_MASK;

	if (value < 0 || value > max_val) {
		pr_err("%s %s  ====  input should between 0 ~ %d\n",
		       DEV_NAME, __func__, max_val);
		return count;
	}

	if (pc->clksrc_div[hwpwm] == value) {
		pr_info("%s %s  ====  hwpwm=%d, input is the same=%d, do nothing!\n",
			DEV_NAME, __func__, hwpwm, value);
		return count;
	}

	set_real_freq_by_target_div(pc, hwpwm, value, pc->clkout_div[hwpwm]);

	pwm_set_register(pc, hwpwm);

	return count;
}

static ssize_t clk_out_div_show(struct device *dev,
				struct device_attribute *attr,
				char *buf, int hwpwm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_pwm_chip *pc = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", pc->clkout_div[hwpwm]);
}

static ssize_t clk_out_div_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count, int hwpwm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_pwm_chip *pc = platform_get_drvdata(pdev);

	int value = 0;

	if (buf == NULL) {
		pr_err("%s %s  ====  buffer is null, return\n",
		       DEV_NAME, __func__);
		return count;
	}
	sscanf(buf, "%d", &value);

	if (value < 0 || value > 255) {
		pr_err("%s %s  ====  input should between 0 ~ 255\n",
		       DEV_NAME, __func__);
		return count;
	}

	if (pc->clkout_div[hwpwm] == value) {
		pr_info("%s %s  ====  hwpwm=%d, input is the same=%d, do nothing!\n",
			DEV_NAME, __func__, hwpwm, value);
		return count;
	}

	set_real_freq_by_target_div(pc, hwpwm, pc->clksrc_div[hwpwm], value);

	pwm_set_register(pc, hwpwm);

	return count;
}

static ssize_t out_freq_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf, int hwpwm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_pwm_chip *pc = platform_get_drvdata(pdev);

	return sprintf(buf, "%dHz\n", pc->out_freq[hwpwm]);
}

static ssize_t out_freq_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count, int hwpwm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_pwm_chip *pc = platform_get_drvdata(pdev);

	int value = 0;

	if (buf == NULL) {
		pr_err("%s %s  ====  buffer is null, return\n",
		       DEV_NAME, __func__);
		return count;
	}
	sscanf(buf, "%d", &value);

	if (value < 0 || value > RTK_27MHZ) {
		pr_err("%s %s  ====  input should between 0 ~ 27MHz\n",
		       DEV_NAME, __func__);
		return count;
	}

	if (pc->out_freq[hwpwm] == value) {
		pr_info("%s %s  ====  hwpwm=%d, input is the same=%d, do nothing!\n",
			DEV_NAME, __func__, hwpwm, value);
		return count;
	}

	set_real_freq_by_target_freq(pc, hwpwm, value);

	/* update CD since OCD is changed */
	set_clk_duty(pc, hwpwm, pc->duty_rate[hwpwm]);

	pwm_set_register(pc, hwpwm);

	return count;
}

#define RTK_PWM_HANDLE(name, n, mode)					\
static ssize_t name ## n ## _show(struct device *dev,			\
				  struct device_attribute *attr,	\
				  char *buf)				\
{									\
	return name ## _show(dev, attr, buf, n);			\
}									\
									\
static ssize_t name ## n ## _store(struct device *dev,			\
				   struct device_attribute *attr,	\
				   const char *buf, size_t count)	\
{									\
	return name ## _store(dev, attr, buf, count, n);		\
}									\
									\
static DEVICE_ATTR(name ## n,						\
		   mode,						\
		   name ## n ## _show,					\
		   name ## n ## _store)

RTK_PWM_HANDLE(duty_rate, 0, 0664);
RTK_PWM_HANDLE(pwm_enable, 0, 0664);
RTK_PWM_HANDLE(clk_src_div, 0, 0664);
RTK_PWM_HANDLE(clk_out_div, 0, 0664);
RTK_PWM_HANDLE(out_freq, 0, 0444);

RTK_PWM_HANDLE(duty_rate, 1, 0664);
RTK_PWM_HANDLE(pwm_enable, 1, 0664);
RTK_PWM_HANDLE(clk_src_div, 1, 0664);
RTK_PWM_HANDLE(clk_out_div, 1, 0664);
RTK_PWM_HANDLE(out_freq, 1, 0444);

RTK_PWM_HANDLE(duty_rate, 2, 0664);
RTK_PWM_HANDLE(pwm_enable, 2, 0664);
RTK_PWM_HANDLE(clk_src_div, 2, 0664);
RTK_PWM_HANDLE(clk_out_div, 2, 0664);
RTK_PWM_HANDLE(out_freq, 2, 0444);

RTK_PWM_HANDLE(duty_rate, 3, 0664);
RTK_PWM_HANDLE(pwm_enable, 3, 0664);
RTK_PWM_HANDLE(clk_src_div, 3, 0664);
RTK_PWM_HANDLE(clk_out_div, 3, 0664);
RTK_PWM_HANDLE(out_freq, 3, 0444);

static struct attribute *pwm_dev_attrs[] = {
	&dev_attr_duty_rate0.attr,
	&dev_attr_pwm_enable0.attr,
	&dev_attr_clk_src_div0.attr,
	&dev_attr_clk_out_div0.attr,
	&dev_attr_out_freq0.attr,
	&dev_attr_duty_rate1.attr,
	&dev_attr_pwm_enable1.attr,
	&dev_attr_clk_src_div1.attr,
	&dev_attr_clk_out_div1.attr,
	&dev_attr_out_freq1.attr,
	&dev_attr_duty_rate2.attr,
	&dev_attr_pwm_enable2.attr,
	&dev_attr_clk_src_div2.attr,
	&dev_attr_clk_out_div2.attr,
	&dev_attr_out_freq2.attr,
	&dev_attr_duty_rate3.attr,
	&dev_attr_pwm_enable3.attr,
	&dev_attr_clk_src_div3.attr,
	&dev_attr_clk_out_div3.attr,
	&dev_attr_out_freq3.attr,
	NULL,
};

static struct attribute_group pwm_dev_attr_group = {
	.attrs		= pwm_dev_attrs,
};

static int rtk_pwm_probe(struct platform_device *pdev)
{
	struct rtk_pwm_chip *pwm;
	struct device_node *node = pdev->dev.of_node;
	struct device_node *pwm0, *pwm1, *pwm2, *pwm3;
	int ret = 0;
	u32 val = 0;
	int i;

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm)
		return -ENOMEM;

	pwm->proc_id = pdev->id;
	pwm->dev = &pdev->dev;

	platform_set_drvdata(pdev, pwm);

	pwm->mmio_pwm_reg_base = of_iomap(node, 0);

	pwm->pwm_reg_set = of_device_get_match_data(&pdev->dev);
	if (!pwm->pwm_reg_set) {
		dev_err(&pdev->dev, "no proper pwm_reg_set()\n");
		return -EINVAL;
	}

	spin_lock_init(&pwm->lock);

	pwm->base_freq = RTK_27MHZ;

	pwm->iso_base = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "realtek,iso");

	for (i = 0; i < NUM_PWM; i++) {
		pwm->out_freq[i] = 0;
		pwm->enable[i] = 0;
		pwm->clkout_div[i] = RTK_PWM_OCD_DEFAULT;
		pwm->clksrc_div[i] = RTK_PWM_CD_DEFAULT;
		pwm->clk_duty[i] = RTK_PWM_CSD_DEFAULT;
		pwm->duty_rate[i] = 0;
	}

	pwm0 = of_get_child_by_name(node, "pwm_0");
	if (!pwm0) {
		dev_err(&pdev->dev, "could not find [pwm_0] sub-node\n");
		return -EINVAL;
	}

	pwm1 = of_get_child_by_name(node, "pwm_1");
	if (!pwm1) {
		dev_err(&pdev->dev, "could not find [pwm_1] sub-node\n");
		return -EINVAL;
	}

	pwm2 = of_get_child_by_name(node, "pwm_2");
	if (!pwm2) {
		dev_err(&pdev->dev, "could not find [pwm_2] sub-node\n");
		return -EINVAL;
	}

	pwm3 = of_get_child_by_name(node, "pwm_3");
	if (!pwm3) {
		dev_err(&pdev->dev, "could not find [pwm_3] sub-node\n");
		return -EINVAL;
	}

	/* PWM0 */
	if (!of_property_read_u32(pwm0, "clkout_div", &val))
		pwm->clkout_div[0] = val;

	if (!of_property_read_u32(pwm0, "clksrc_div", &val))
		pwm->clksrc_div[0] = val;

	set_real_freq_by_target_div(pwm, 0, pwm->clksrc_div[0],
				    pwm->clkout_div[0]);

	if (!of_property_read_u32(pwm0, "enable", &val))
		pwm->enable[0] = val;

	if (!of_property_read_u32(pwm0, "duty_rate", &val))
		set_clk_duty(pwm, 0, val);

	if (of_find_property(pwm0, "pulse-low-on-change", NULL))
		pwm->flags[0] |= RTK_PWM_FLAGS_PULSE_LOW_ON_CHANGE;

	pr_info("%s %s - hwpwm=(%d) enable=(%d) duty_rate=(%d) clksrc_div=(%d) clkout_div=(%d)---\n",
		DEV_NAME, __func__, 0, pwm->enable[0], pwm->duty_rate[0],
		pwm->clksrc_div[0], pwm->clkout_div[0]);
	pr_info("%s %s - defualt output frequence = %dHz ---\n",
		DEV_NAME, __func__, pwm->out_freq[0]);

	/* PWM1 */
	if (!of_property_read_u32(pwm1, "clkout_div", &val))
		pwm->clkout_div[1] = val;

	if (!of_property_read_u32(pwm1, "clksrc_div", &val))
		pwm->clksrc_div[1] = val;

	set_real_freq_by_target_div(pwm, 1, pwm->clksrc_div[1],
				    pwm->clkout_div[1]);

	if (!of_property_read_u32(pwm1, "enable", &val))
		pwm->enable[1] = val;

	if (!of_property_read_u32(pwm1, "duty_rate", &val))
		set_clk_duty(pwm, 1, val);

	if (of_find_property(pwm1, "pulse-low-on-change", NULL))
		pwm->flags[1] |= RTK_PWM_FLAGS_PULSE_LOW_ON_CHANGE;

	pr_info("%s %s - hwpwm=(%d) enable=(%d) duty_rate=(%d) clksrc_div=(%d) clkout_div=(%d)---\n",
		DEV_NAME, __func__, 1, pwm->enable[1], pwm->duty_rate[1],
		pwm->clksrc_div[1], pwm->clkout_div[1]);
	pr_info("%s %s - defualt output frequence = %dHz ---\n",
		DEV_NAME, __func__, pwm->out_freq[1]);

	/* PWM2 */
	if (!of_property_read_u32(pwm2, "clkout_div", &val))
		pwm->clkout_div[2] = val;

	if (!of_property_read_u32(pwm2, "clksrc_div", &val))
		pwm->clksrc_div[2] = val;

	set_real_freq_by_target_div(pwm, 2, pwm->clksrc_div[2],
				    pwm->clkout_div[2]);

	if (!of_property_read_u32(pwm2, "enable", &val))
		pwm->enable[2] = val;

	if (!of_property_read_u32(pwm2, "duty_rate", &val))
		set_clk_duty(pwm, 2, val);

	if (of_find_property(pwm2, "pulse-low-on-change", NULL))
		pwm->flags[2] |= RTK_PWM_FLAGS_PULSE_LOW_ON_CHANGE;

	pr_info("%s %s - hwpwm=(%d) enable=(%d) duty_rate=(%d) clksrc_div=(%d) clkout_div=(%d)---\n",
		DEV_NAME, __func__, 2, pwm->enable[2], pwm->duty_rate[2],
		pwm->clksrc_div[2], pwm->clkout_div[2]);
	pr_info("%s %s - defualt output frequence = %dHz ---\n",
		DEV_NAME, __func__, pwm->out_freq[2]);

	/* PWM3 */
	if (!of_property_read_u32(pwm3, "clkout_div", &val))
		pwm->clkout_div[3] = val;

	if (!of_property_read_u32(pwm3, "clksrc_div", &val))
		pwm->clksrc_div[3] = val;

	set_real_freq_by_target_div(pwm, 3, pwm->clksrc_div[3],
				    pwm->clkout_div[3]);

	if (!of_property_read_u32(pwm3, "enable", &val))
		pwm->enable[3] = val;

	if (!of_property_read_u32(pwm3, "duty_rate", &val))
		set_clk_duty(pwm, 3, val);

	if (of_find_property(pwm3, "pulse-low-on-change", NULL))
		pwm->flags[3] |= RTK_PWM_FLAGS_PULSE_LOW_ON_CHANGE;

	pr_info("%s %s - hwpwm=(%d) enable=(%d) duty_rate=(%d) clksrc_div=(%d) clkout_div=(%d)---\n",
		DEV_NAME, __func__, 3, pwm->enable[3], pwm->duty_rate[3],
		pwm->clksrc_div[3], pwm->clkout_div[3]);
	pr_info("%s %s - defualt output frequence = %dHz ---\n",
		DEV_NAME, __func__, pwm->out_freq[3]);

	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &rtk_pwm_ops;
	pwm->chip.base = 0;
	pwm->chip.npwm = NUM_PWM;

	ret = sysfs_create_group(&pdev->dev.kobj, &pwm_dev_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "sysfs_create_group() failed: %d\n", ret);
		return ret;
	}

	ret = pwmchip_add(&pwm->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
		sysfs_remove_group(&pdev->dev.kobj, &pwm_dev_attr_group);
		return ret;
	}

	for (i = 0; i < NUM_PWM; i++)
		pwm_set_register(pwm, i);

	return 0;
}

static int rtk_pwm_remove(struct platform_device *pdev)
{
	struct rtk_pwm_chip *pc = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < NUM_PWM; i++) {
		pc->enable[i] = 0;
		pwm_set_register(pc, i);
	}
	sysfs_remove_group(&pdev->dev.kobj, &pwm_dev_attr_group);

	return pwmchip_remove(&pc->chip);
}

static const struct of_device_id rtk_pwm_of_match[] = {
	{
		.compatible = "realtek,rtk-pwm",
		.data = pwm_reg_set_v1,
	},
	{
		.compatible = "realtek,rtk-pwm-v2",
		.data = pwm_reg_set_v2,
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, rtk_pwm_of_match);

static struct platform_driver rtk_pwm_platform_driver = {
	.driver = {
		.name		= "rtk-pwm",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(rtk_pwm_of_match),
	},
	.probe = rtk_pwm_probe,
	.remove = rtk_pwm_remove,
};

module_platform_driver(rtk_pwm_platform_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("REALTEK Corporation");
MODULE_ALIAS("platform:rtk-pwm");
