// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Realtek Low Speed ADC driver
 *
 * Copyright (C) 2020 Realtek Semiconductor Corporation.
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/reset.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

#include "rtk_lsadc0.h"

#define SA_SHIRQ		IRQF_SHARED
#define LSADC_READL(reg)	readl((void __iomem *)(reg + (uintptr_t)(st->lsadc_addr)))
#define LSADC_WRITEL(val, reg)	writel(val, (void __iomem *)(reg + (uintptr_t)(st->lsadc_addr)))

struct rtk_lsadc_pad_info {
	uint				activate;
	uint				ctrl_mode;
	uint				pad_sw;
	uint				threshold;
	uint				vref_sel;
	uint				adc_val0;
	uint				adc_val_baseline;
	uint				range_evt_cfg;
};

struct rtk_lsadc_info {
	struct rtk_lsadc_pad_info	pad[2];
	uint				debounce_cnt;
	uint				irq;
	uint				clk_gating_en;
};

struct rtk_lsadc_state {
	struct device			*dev;
	struct rtk_lsadc_info		lsadc[1];
	uint				crt_lsadc_pg_val;
	void __iomem			*crt_lsadc_pg_addr;
	void __iomem			*lsadc_addr;
	struct clk			*clk;
	uint				rising_evt_cfg;
	uint				falling_evt_cfg;
	struct mutex			lock;
};

static const struct iio_event_spec rtk_lsadc_event[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	},
};

static const int rtk_mod_map[] = {
	[IIO_MOD_X] = 0,
	[IIO_MOD_Y] = 1,
	[IIO_MOD_Z] = 2,
	[IIO_MOD_LIGHT_RED] = 3,
	[IIO_MOD_LIGHT_GREEN] = 4,
	[IIO_MOD_LIGHT_BLUE] = 5,
};

static const int rtk_level_map[] = {
	IIO_MOD_X, IIO_MOD_Y, IIO_MOD_Z,
	IIO_MOD_LIGHT_RED, IIO_MOD_LIGHT_GREEN, IIO_MOD_LIGHT_BLUE
};

static irqreturn_t lsadc0_interrupt_pad(int irq, void *data)
{
	struct iio_dev *idev = data;
	struct rtk_lsadc_state *st = iio_priv(idev);
	uint status_reg;
	uint pad0_reg, pad1_reg;
	uint pad0_int, pad1_int;
	uint new_adc_val = 0;
	uint reg;
	uint addr;
	int i, j;

	for (i = 0; i < 2; i++) {
		addr = (i ? LSADC0_PAD1_LEVEL_SET0_ADDR : LSADC0_PAD0_LEVEL_SET0_ADDR);
		for (j = 0; j < LSADC0_PAD_LEVEL_SET_NUMBER; j++) {
			if (!(st->lsadc[0].pad[i].range_evt_cfg & BIT(j)))
				continue;
			reg = LSADC_READL(addr + 4 * j);
			dev_info(st->dev, "[LSADC] pad%d level set%d reg = 0x%0x\n", i, j, reg);
			if ((reg & LSADC0_LEVEL_BLK_EN_MASK) &&
			    (reg & LSADC0_LEVEL_INT_EN_MASK) &&
			    (reg & LSADC0_LEVEL_INT_PEND_MASK)) {
				st->lsadc[0].pad[i].range_evt_cfg &= ~BIT(j);
				dev_info(st->dev, "[LSADC] pad%d level set%d INT!\n", i, j);
				iio_push_event(idev,
					       IIO_MOD_EVENT_CODE(IIO_VOLTAGE,
								  i,
								  rtk_level_map[j],
								  IIO_EV_TYPE_THRESH,
								  IIO_EV_DIR_RISING),
					       iio_get_time_ns(idev));
			}
		}
	}
	status_reg = LSADC_READL(LSADC0_STATUS_ADDR);

	pad0_int = LSADC0_STATUS_PAD0_STATUS_MASK & status_reg;
	pad1_int = LSADC0_STATUS_PAD1_STATUS_MASK & status_reg;

	pad0_reg = LSADC_READL(LSADC0_PAD0_ADDR);
	pad1_reg = LSADC_READL(LSADC0_PAD1_ADDR);
	dev_info(st->dev, "[LSADC] %s: pad0_adc=0x%0x, pad1_adc=0x%0x\n",
		 __func__, st->lsadc[0].pad[0].adc_val0, st->lsadc[0].pad[1].adc_val0);

	if (pad0_int) {
		new_adc_val = pad0_reg & LSADC0_PAD_ADC_VAL_MASK;
		dev_info(st->dev, "[LSADC] pad0 interrupt! status_reg=0x%x , adc_val=> from [%d] to [%d]\n",
			 status_reg, st->lsadc[0].pad[0].adc_val0, new_adc_val);
		if (st->lsadc[0].pad[0].adc_val0 > new_adc_val) {
			dev_info(st->dev, "[LSADC] pad0 adc became smaller!\n\n");
			if (st->falling_evt_cfg & BIT(0))
				iio_push_event(idev,
					       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
								    0,
								    IIO_EV_TYPE_THRESH,
								    IIO_EV_DIR_FALLING),
					       iio_get_time_ns(idev));
		} else {
			dev_info(st->dev, "[LSADC] pad0 adc became bigger!\n\n");

			/* set adc_val0 to base line */
			pad0_reg = (pad0_reg & ~LSADC0_PAD_ADC_VAL_MASK) |
				   st->lsadc[0].pad[0].adc_val_baseline;
			LSADC_WRITEL(pad0_reg, LSADC0_PAD0_ADDR);

			if (st->rising_evt_cfg & BIT(0))
				iio_push_event(idev,
					       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
								    0,
								    IIO_EV_TYPE_THRESH,
								    IIO_EV_DIR_RISING),
					       iio_get_time_ns(idev));
		}
		st->lsadc[0].pad[0].adc_val0 = new_adc_val;
	}

	if (pad1_int) {
		new_adc_val = pad1_reg & LSADC0_PAD_ADC_VAL_MASK;
		dev_info(st->dev, "[LSADC] pad1 interrupt! status_reg=0x%x , adc_val=> from [%d] to [%d]\n",
			 status_reg, st->lsadc[0].pad[1].adc_val0, new_adc_val);
		if (st->lsadc[0].pad[1].adc_val0 > new_adc_val) {
			dev_info(st->dev, "[LSADC] pad1 adc became smaller!\n\n");

			if (st->falling_evt_cfg & BIT(1))
				iio_push_event(idev,
					       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
								    1,
								    IIO_EV_TYPE_THRESH,
								    IIO_EV_DIR_FALLING),
					       iio_get_time_ns(idev));
		} else {
			dev_info(st->dev, "[LSADC] pad1 adc became bigger!\n\n");

			/* set adc_val0 to base line */
			pad1_reg = (pad1_reg & ~LSADC0_PAD_ADC_VAL_MASK) |
				   st->lsadc[0].pad[1].adc_val_baseline;
			LSADC_WRITEL(pad1_reg, LSADC0_PAD1_ADDR);

			if (st->rising_evt_cfg & BIT(1))
				iio_push_event(idev,
					       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
								    1,
								    IIO_EV_TYPE_THRESH,
								    IIO_EV_DIR_RISING),
					       iio_get_time_ns(idev));
		}
		st->lsadc[0].pad[1].adc_val0 = new_adc_val;
	}

	/* reset INT flag */
	status_reg |= LSADC0_STATUS_PAD0_STATUS_MASK | LSADC0_STATUS_PAD1_STATUS_MASK;
	LSADC_WRITEL(status_reg, LSADC0_STATUS_ADDR);

	return IRQ_HANDLED;
}

static ssize_t in_info_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct iio_dev *idev = platform_get_drvdata(pdev);
	struct rtk_lsadc_state *st = iio_priv(idev);
	uint ctrl_reg;
	uint analog_ctrl_reg;
	uint pad0_reg;
	uint pad1_reg;
	uint lsadc_status_reg;
	uint pad0_set[LSADC0_PAD_LEVEL_SET_NUMBER];
	uint pad1_set[LSADC0_PAD_LEVEL_SET_NUMBER];
	int i = 0;
	int len = 0;

	mutex_lock(&st->lock);
	ctrl_reg = LSADC_READL(LSADC0_CTRL_ADDR);
	lsadc_status_reg = LSADC_READL(LSADC0_STATUS_ADDR);
	analog_ctrl_reg = LSADC_READL(LSADC0_ANALOG_CTRL_ADDR);

	if ((ctrl_reg & LSADC0_CTRL_ENABLE_MASK) == 0) {
		ctrl_reg = ctrl_reg | LSADC0_CTRL_ENABLE_MASK;
		LSADC_WRITEL(ctrl_reg, LSADC0_CTRL_ADDR);
		dev_info(dev, "[LSADC] write ctrl_enable, ctrl_reg=0x%x\n",
			 ctrl_reg);
	}

	pad0_reg = LSADC_READL(LSADC0_PAD0_ADDR);
	pad1_reg = LSADC_READL(LSADC0_PAD1_ADDR);

	st->lsadc[0].pad[0].adc_val0 = pad0_reg & LSADC0_PAD_ADC_VAL_MASK;
	st->lsadc[0].pad[1].adc_val0 = pad1_reg & LSADC0_PAD_ADC_VAL_MASK;

	len += scnprintf(buf + len, PAGE_SIZE - len, "%s --\n", __func__);
	len += scnprintf(buf + len, PAGE_SIZE - len, "\t ctrl_reg=0x%x\n",
			 ctrl_reg);
	len += scnprintf(buf + len, PAGE_SIZE - len, "\t lsadc_status_reg=0x%x\n",
			 lsadc_status_reg);
	len += scnprintf(buf + len, PAGE_SIZE - len, "\t analog_ctrl_reg=0x%x\n",
			 analog_ctrl_reg);
	len += scnprintf(buf + len, PAGE_SIZE - len, "\t pad0_reg=0x%x\n",
			 pad0_reg);
	len += scnprintf(buf + len, PAGE_SIZE - len, "\t pad1_reg=0x%x\n",
			 pad1_reg);
	len += scnprintf(buf + len, PAGE_SIZE - len, "\t pad0_adc=0x%x\n",
			 st->lsadc[0].pad[0].adc_val0);
	len += scnprintf(buf + len, PAGE_SIZE - len, "\t pad1_adc=0x%x\n",
			 st->lsadc[0].pad[1].adc_val0);

	len += scnprintf(buf + len, PAGE_SIZE - len, "info:\n");
	for (i = 0; i < LSADC0_PAD_LEVEL_SET_NUMBER; i++) {
		pad0_set[i] = LSADC_READL(LSADC0_PAD0_LEVEL_SET0_ADDR + (i * 4));
		pad1_set[i] = LSADC_READL(LSADC0_PAD1_LEVEL_SET0_ADDR + (i * 4));
		len += scnprintf(buf + len, PAGE_SIZE - len, "set_idx[%d]: pad0_set=0x%0x, pad1_set=0x%0x\n",
				 i, pad0_set[i], pad1_set[i]);
	}
	mutex_unlock(&st->lock);

	return len;
}

static IIO_DEVICE_ATTR_RO(in_info, 0);

static struct attribute *rtk_attr_base[] = {
	&iio_dev_attr_in_info.dev_attr.attr,
	NULL
};

static const struct attribute_group rtk_group_base = {
	.attrs = rtk_attr_base,
};

#define RTK_LSADC_CHAN(_idx, _data_reg_addr) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.address = (_data_reg_addr),				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_ENABLE),	\
	.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_DEBOUNCE_COUNT), \
	.scan_index = (_idx),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 8,					\
		.storagebits = 8,				\
		.shift = 0,					\
	},							\
	.event_spec = rtk_lsadc_event,				\
	.num_event_specs = ARRAY_SIZE(rtk_lsadc_event),		\
}

#define RTK_LSADC_CHAN2(_idx, _idx2, _data_reg_addr) {		\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.modified = 1,						\
	.channel2 = (_idx2),					\
	.address = (_data_reg_addr),				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_ENABLE),	\
	.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_DEBOUNCE_COUNT), \
	.scan_index = (_idx),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 8,					\
		.storagebits = 8,				\
		.shift = 0,					\
	},							\
	.event_spec = rtk_lsadc_event,				\
	.num_event_specs = ARRAY_SIZE(rtk_lsadc_event),		\
}

static const struct iio_chan_spec rtk_lsadc_iio_channels[] = {
	RTK_LSADC_CHAN(0, LSADC0_PAD0_ADDR),
	RTK_LSADC_CHAN(1, LSADC0_PAD1_ADDR),
	RTK_LSADC_CHAN2(0, IIO_MOD_X, LSADC0_PAD0_LEVEL_SET0_ADDR),
	RTK_LSADC_CHAN2(0, IIO_MOD_Y, LSADC0_PAD0_LEVEL_SET1_ADDR),
	RTK_LSADC_CHAN2(0, IIO_MOD_Z, LSADC0_PAD0_LEVEL_SET2_ADDR),
	RTK_LSADC_CHAN2(0, IIO_MOD_LIGHT_RED, LSADC0_PAD0_LEVEL_SET3_ADDR),
	RTK_LSADC_CHAN2(0, IIO_MOD_LIGHT_GREEN, LSADC0_PAD0_LEVEL_SET4_ADDR),
	RTK_LSADC_CHAN2(0, IIO_MOD_LIGHT_BLUE, LSADC0_PAD0_LEVEL_SET5_ADDR),
	RTK_LSADC_CHAN2(1, IIO_MOD_X, LSADC0_PAD1_LEVEL_SET0_ADDR),
	RTK_LSADC_CHAN2(1, IIO_MOD_Y, LSADC0_PAD1_LEVEL_SET1_ADDR),
	RTK_LSADC_CHAN2(1, IIO_MOD_Z, LSADC0_PAD1_LEVEL_SET2_ADDR),
	RTK_LSADC_CHAN2(1, IIO_MOD_LIGHT_RED, LSADC0_PAD1_LEVEL_SET3_ADDR),
	RTK_LSADC_CHAN2(1, IIO_MOD_LIGHT_GREEN, LSADC0_PAD1_LEVEL_SET4_ADDR),
	RTK_LSADC_CHAN2(1, IIO_MOD_LIGHT_BLUE, LSADC0_PAD1_LEVEL_SET5_ADDR),
};

static int rtk_lsadc_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct rtk_lsadc_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = st->lsadc[0].pad[chan->channel].adc_val0;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = MICROVOLT_HIGH;
		*val2 = chan->scan_type.realbits;
		ret = IIO_VAL_FRACTIONAL_LOG2;
		break;
	case IIO_CHAN_INFO_ENABLE:
		if (chan->modified) {
			if (chan->channel > 1 || rtk_mod_map[chan->channel2] >= LSADC0_PAD_LEVEL_SET_NUMBER)
				goto err_exit;

			*val = !!(LSADC_READL(chan->address) & LSADC0_LEVEL_BLK_EN_MASK);
			ret = IIO_VAL_INT;
		} else {
			*val = st->lsadc[0].pad[chan->channel].activate;
			ret = IIO_VAL_INT;
		}
		break;
	case IIO_CHAN_INFO_DEBOUNCE_COUNT:
		*val = st->lsadc[0].debounce_cnt;
		ret = IIO_VAL_INT;
		break;
	default:
		break;
	}
err_exit:
	mutex_unlock(&st->lock);
	return ret;
}

static int rtk_lsadc_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct rtk_lsadc_state *st = iio_priv(indio_dev);
	uint reg;
	int activate = 0;
	int addr = 0;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		if (val)
			activate = 1;
		else
			activate = 0;

		addr = chan->address;
		if (chan->modified) {
			if (chan->channel > 1 || rtk_mod_map[chan->channel2] >= LSADC0_PAD_LEVEL_SET_NUMBER)
				return -EINVAL;

			mutex_lock(&st->lock);
			reg = LSADC_READL(addr);
			if ((reg & LSADC0_LEVEL_BLK_EN_MASK) != (activate << LSADC0_LEVEL_BLK_EN_SHIFT)) {
				if (activate)
					reg |= LSADC0_LEVEL_BLK_EN_MASK;
				else
					reg &= ~LSADC0_LEVEL_BLK_EN_MASK;
				reg &= ~LSADC0_LEVEL_INT_PEND_MASK;
				LSADC_WRITEL(reg, addr);
			}
			mutex_unlock(&st->lock);
		} else {
			if (st->lsadc[0].pad[chan->channel].activate == activate)
				return 0; /* ignore the same setting */

			st->lsadc[0].pad[chan->channel].activate = activate;

			mutex_lock(&st->lock);
			reg = LSADC_READL(addr);
			if (activate)
				reg |= LSADC0_PAD_ACTIVE_MASK;
			else
				reg &= ~LSADC0_PAD_ACTIVE_MASK;
			LSADC_WRITEL(reg, addr);
			mutex_unlock(&st->lock);
		}
		return 0;
	case IIO_CHAN_INFO_DEBOUNCE_COUNT:
		if (val > 15)
			val = 15;
		if (val < 0)
			val = 0;

		if (st->lsadc[0].debounce_cnt == val)
			return 0; /* ignore the same setting */

		st->lsadc[0].debounce_cnt = val;
		addr = LSADC0_CTRL_ADDR;

		mutex_lock(&st->lock);
		reg = LSADC_READL(addr);
		reg &= ~LSADC0_CTRL_DEBOUNCE_MASK;
		reg |= val << LSADC0_CTRL_DEBOUNCE_SHIFT;
		LSADC_WRITEL(reg, addr);
		mutex_unlock(&st->lock);
		return 0;
	case IIO_CHAN_INFO_SCALE:
	case IIO_CHAN_INFO_RAW:
		return -EPERM;
	default:
		return -EINVAL;
	}
}

int rtk_lsadc_read_event_config(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir)
{
	struct rtk_lsadc_state *st = iio_priv(indio_dev);
	uint reg;

	switch (dir) {
	case IIO_EV_DIR_RISING:
		if (chan->modified) {
			if (chan->channel > 1 || rtk_mod_map[chan->channel2] >= LSADC0_PAD_LEVEL_SET_NUMBER)
				return -EINVAL;

			mutex_lock(&st->lock);
			reg = LSADC_READL(chan->address);
			mutex_unlock(&st->lock);
			return !!(reg & LSADC0_LEVEL_INT_EN_MASK);
		} else {
			return !!(st->rising_evt_cfg & BIT(chan->channel));
		}
	case IIO_EV_DIR_FALLING:
		if (chan->modified) {
			if (chan->channel > 1 || rtk_mod_map[chan->channel2] >= LSADC0_PAD_LEVEL_SET_NUMBER)
				return -EINVAL;

			mutex_lock(&st->lock);
			reg = LSADC_READL(chan->address);
			mutex_unlock(&st->lock);
			return !!(reg & LSADC0_LEVEL_INT_PEND_MASK);
		} else {
			return !!(st->falling_evt_cfg & BIT(chan->channel));
		}
	default:
		return -EINVAL;
	}

	return 0;
}

int rtk_lsadc_write_event_config(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 enum iio_event_type type,
				 enum iio_event_direction dir,
				 int state)
{
	struct rtk_lsadc_state *st = iio_priv(indio_dev);
	uint reg;
	uint blk_en;
	int ret = -EINVAL;

	mutex_lock(&st->lock);
	switch (dir) {
	case IIO_EV_DIR_RISING:
		if (chan->modified) {
			if (chan->channel > 1 || rtk_mod_map[chan->channel2] >= LSADC0_PAD_LEVEL_SET_NUMBER)
				goto err_exit;

			reg = LSADC_READL(chan->address);
			if (state) {
				reg |= LSADC0_LEVEL_INT_EN_MASK;
				st->lsadc[0].pad[chan->channel].range_evt_cfg |= BIT(rtk_mod_map[chan->channel2]);
			} else {
				reg &= ~LSADC0_LEVEL_INT_EN_MASK;
			}
			reg &= ~LSADC0_LEVEL_INT_PEND_MASK;
			LSADC_WRITEL(reg, chan->address);
		} else {
			if (state)
				st->rising_evt_cfg |= BIT(chan->channel);
			else
				st->rising_evt_cfg &= ~BIT(chan->channel);
		}
		ret = 0;
		break;
	case IIO_EV_DIR_FALLING:
		if (chan->modified) {
			if (chan->channel > 1 || rtk_mod_map[chan->channel2] >= LSADC0_PAD_LEVEL_SET_NUMBER)
				goto err_exit;

			/* disable blk_en */
			reg = LSADC_READL(chan->address);
			blk_en = reg & LSADC0_LEVEL_BLK_EN_MASK;
			reg &= ~LSADC0_LEVEL_BLK_EN_MASK;

			if (state) {
				reg |= LSADC0_LEVEL_INT_PEND_MASK;
				st->lsadc[0].pad[chan->channel].range_evt_cfg |= BIT(rtk_mod_map[chan->channel2]);
			} else {
				reg &= ~LSADC0_LEVEL_INT_PEND_MASK;
			}
			LSADC_WRITEL(reg, chan->address);

			/* restore blk_en */
			if (blk_en) {
				reg |= blk_en;
				reg &= ~LSADC0_LEVEL_INT_PEND_MASK;
				LSADC_WRITEL(reg, chan->address);
			}
			/* reset pad INT status */
			reg = LSADC_READL(LSADC0_STATUS_ADDR);
			reg |= LSADC0_STATUS_PAD0_STATUS_MASK | LSADC0_STATUS_PAD1_STATUS_MASK;
			LSADC_WRITEL(reg, LSADC0_STATUS_ADDR);
		} else {
			if (state)
				st->falling_evt_cfg |= BIT(chan->channel);
			else
				st->falling_evt_cfg &= ~BIT(chan->channel);
		}
		ret = 0;
		break;
	default:
		dev_err(st->dev, "unsupported dir %d\n", dir);
	}

err_exit:
	mutex_unlock(&st->lock);

	return ret;
}

int rtk_lsadc_read_event_value(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan,
			       enum iio_event_type type,
			       enum iio_event_direction dir,
			       enum iio_event_info info,
			       int *val, int *val2)
{
	struct rtk_lsadc_state *st = iio_priv(indio_dev);
	uint reg;

	if (chan->modified) {
		if (chan->channel > 1 || rtk_mod_map[chan->channel2] >= LSADC0_PAD_LEVEL_SET_NUMBER)
			return -EINVAL;

		mutex_lock(&st->lock);
		reg = LSADC_READL(chan->address);
		mutex_unlock(&st->lock);

		switch (dir) {
		case IIO_EV_DIR_RISING:
			*val = (reg & LSADC0_LEVEL_TOP_BOUND_MASK) >> LSADC0_LEVEL_TOP_BOUND_SHIFT;
			break;
		case IIO_EV_DIR_FALLING:
			*val = (reg & LSADC0_LEVEL_LOW_BOUND_MASK) >> LSADC0_LEVEL_LOW_BOUND_SHIFT;
			break;
		default:
			return -EINVAL;
		}
	} else {
		if (chan->channel > 1)
			return -EINVAL;

		mutex_lock(&st->lock);
		reg = LSADC_READL(chan->address);
		mutex_unlock(&st->lock);
		*val = (reg & LSADC0_PAD_THRED_MASK) >> LSADC0_PAD_THRED_SHIFT;
	}

	return IIO_VAL_INT;
}

int rtk_lsadc_write_event_value(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				enum iio_event_info info,
				int val, int val2)
{
	struct rtk_lsadc_state *st = iio_priv(indio_dev);
	uint reg = 0;
	uint old_thresh = 0;
	uint max_thresh;

	max_thresh = GENMASK(chan->scan_type.realbits - 1, 0);
	if (val < 0 || val > max_thresh) {
		dev_info(st->dev, "channel %d: val (%d) out of range (0~%u)\n",
			 chan->channel, val, max_thresh);
		return -EINVAL;
	}


	if (chan->modified) {
		if (chan->channel > 1 || rtk_mod_map[chan->channel2] >= LSADC0_PAD_LEVEL_SET_NUMBER)
			return -EINVAL;

		mutex_lock(&st->lock);
		reg = LSADC_READL(chan->address);

		switch (dir) {
		case IIO_EV_DIR_RISING:
			old_thresh = (reg & LSADC0_LEVEL_TOP_BOUND_MASK) >> LSADC0_LEVEL_TOP_BOUND_SHIFT;
			reg = (reg & ~LSADC0_LEVEL_TOP_BOUND_MASK) | (val << LSADC0_LEVEL_TOP_BOUND_SHIFT);
			break;
		case IIO_EV_DIR_FALLING:
			old_thresh = (reg & LSADC0_LEVEL_LOW_BOUND_MASK) >> LSADC0_LEVEL_LOW_BOUND_SHIFT;
			reg = (reg & ~LSADC0_LEVEL_LOW_BOUND_MASK) | (val << LSADC0_LEVEL_LOW_BOUND_SHIFT);
			break;
		default:
			dev_err(st->dev, "dir %d is not supported\n", dir);
		}
		reg &= ~LSADC0_LEVEL_INT_PEND_MASK;
		LSADC_WRITEL(reg, chan->address);
		mutex_unlock(&st->lock);
		dev_info(st->dev, "channel %d, mapped channel2 %d: write pad_reg = 0x%x, new threshold = %d, old threshold = %d\n",
			 chan->channel, rtk_mod_map[chan->channel2], reg, val, old_thresh);
	} else {
		if (chan->channel > 1)
			return -EINVAL;

		mutex_lock(&st->lock);
		st->lsadc[0].pad[chan->channel].threshold = val;
		reg = LSADC_READL(chan->address);
		old_thresh = (reg & LSADC0_PAD_THRED_MASK) >> LSADC0_PAD_THRED_SHIFT;
		reg = (reg & ~LSADC0_PAD_THRED_MASK) | (val << LSADC0_PAD_THRED_SHIFT);
		LSADC_WRITEL(reg, chan->address);
		mutex_unlock(&st->lock);
		dev_info(st->dev, "channel %d: write pad_reg = 0x%x, new threshold = %d, old threshold = %d\n",
			 chan->channel, reg, val, old_thresh);
	}

	return 0;
}


static int rtk_lsadc_reg_access(struct iio_dev *indio_dev,
				unsigned int addr, unsigned int writeval,
				unsigned int *readval)
{
	struct rtk_lsadc_state *st = iio_priv(indio_dev);
	struct device *dev = st->dev;

	if (addr % 4 || addr > LSADC0_POWER_ADDR)
		return -EINVAL;

	mutex_lock(&st->lock);
	if (readval) {
		*readval = LSADC_READL(addr);
		dev_info(dev, "read reg 0x%X = 0x%X\n", addr, *readval);
	} else {
		LSADC_WRITEL(writeval, addr);
		dev_info(dev, "write reg 0x%X = 0x%X\n", addr, writeval);
	}
	mutex_unlock(&st->lock);

	return 0;
}

static const struct iio_info rtk_lsadc_iio_info = {
	.read_raw = rtk_lsadc_read_raw,
	.write_raw = rtk_lsadc_write_raw,
	.read_event_config = &rtk_lsadc_read_event_config,
	.write_event_config = &rtk_lsadc_write_event_config,
	.read_event_value = &rtk_lsadc_read_event_value,
	.write_event_value = &rtk_lsadc_write_event_value,
	.attrs = &rtk_group_base,
	.debugfs_reg_access = rtk_lsadc_reg_access,
};


static int rtk_lsadc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *idev;
	struct rtk_lsadc_state *st;
	struct device_node *lsadc0_pad0_node;
	struct device_node *lsadc0_pad1_node;
	int ret = -EINVAL;
	uint val;
	uint ctrl_reg;
	uint analog_ctrl_reg;
	uint pad0_reg;
	uint pad1_reg;
	uint lsadc_status_reg;
	uint irq_num;
	uint lsadc_power_reg;
	struct clk *clk;
	struct reset_control *rstc;

	dev_info(dev, "[LSADC] %s : init\n", __func__);

	/* Request IRQ */
	irq_num = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!irq_num)
		return -EPROBE_DEFER;

	idev = devm_iio_device_alloc(dev, sizeof(struct rtk_lsadc_state));
	if (!idev) {
		dev_err(dev, "[LSADC] failed to allocate memory\n");
		return -ENOMEM;
	}

	st = iio_priv(idev);
	mutex_init(&st->lock);

	lsadc0_pad0_node = of_get_child_by_name(pdev->dev.of_node, "lsadc0-pad0");
	if (!lsadc0_pad0_node) {
		dev_err(dev, "[LSADC] could not find [lsadc0-pad0] sub-node\n");
		return -EINVAL;
	}
	lsadc0_pad1_node = of_get_child_by_name(pdev->dev.of_node, "lsadc0-pad1");
	if (!lsadc0_pad1_node) {
		dev_err(dev, "[LSADC] could not find [lsadc0-pad1] sub-node\n");
		return -EINVAL;
	}

	st->dev = dev;
	st->lsadc[0].irq = irq_num;
	st->lsadc_addr = of_iomap(pdev->dev.of_node, 0);
	st->crt_lsadc_pg_addr = of_iomap(pdev->dev.of_node, 1);

	platform_set_drvdata(pdev, idev);

	if (!of_property_read_u32(pdev->dev.of_node, "clk_gating_en", &val)) {
		if (val > 1)
			val = 1;
		st->lsadc[0].clk_gating_en = val;
	}

	if (!of_property_read_u32(pdev->dev.of_node, "debounce0_cnt", &val)) {
		if (val > 15)
			val = 15;
		st->lsadc[0].debounce_cnt = val;
	}

	st->crt_lsadc_pg_val = CRT_LSADC_PG_VALUE;


	/* set LSADC0 pad0 from device tree : lsadc0-pad0 */
	if (!of_property_read_u32(lsadc0_pad0_node, "activate", &val)) {
		if (val > 1)
			val = 1;
		st->lsadc[0].pad[0].activate = val;
	}

	if (!of_property_read_u32(lsadc0_pad0_node, "ctrl_mode", &val)) {
		if (val > 1)
			val = 1;
		st->lsadc[0].pad[0].ctrl_mode = val;
	}

	if (!of_property_read_u32(lsadc0_pad0_node, "sw_idx", &val)) {
		if (val > 1)
			val = 1;
		st->lsadc[0].pad[0].pad_sw = val;
	}

	if (!of_property_read_u32(lsadc0_pad0_node, "voltage_threshold", &val))
		st->lsadc[0].pad[0].threshold = val;

	if (!of_property_read_u32(lsadc0_pad0_node, "adc_val_baseline", &val))
		st->lsadc[0].pad[0].adc_val_baseline = val;
	else
		st->lsadc[0].pad[0].adc_val_baseline = LSADC0_PAD_ADC_VAL_MASK;

	/* set LSADC0 pad1 from device tree : lsadc0-pad1 */
	if (!of_property_read_u32(lsadc0_pad1_node, "activate", &val)) {
		if (val > 1)
			val = 1;
		st->lsadc[0].pad[1].activate = val;
	}

	if (!of_property_read_u32(lsadc0_pad1_node, "ctrl_mode", &val)) {
		if (val > 1)
			val = 1;
		st->lsadc[0].pad[1].ctrl_mode = val;
	}

	if (!of_property_read_u32(lsadc0_pad1_node, "sw_idx", &val)) {
		if (val > 1)
			val = 1;
		st->lsadc[0].pad[1].pad_sw = val;
	}

	if (!of_property_read_u32(lsadc0_pad1_node, "voltage_threshold", &val))
		st->lsadc[0].pad[1].threshold = val;

	if (!of_property_read_u32(lsadc0_pad1_node, "adc_val_baseline", &val))
		st->lsadc[0].pad[1].adc_val_baseline = val & LSADC0_PAD_ADC_VAL_MASK;
	else
		st->lsadc[0].pad[1].adc_val_baseline = LSADC0_PAD_ADC_VAL_MASK;


	clk = clk_get(dev, NULL);
	if (IS_ERR(clk)) {
		clk = NULL;
		dev_err(dev, "[LSADC] can't get LSADC clock\n");
		goto err_get_clk;
	} else {
		clk_prepare_enable(clk);
	}
	st->clk = clk;


	rstc = reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rstc)) {
		dev_err(dev, "[LSADC] can't get LSADC reset\n");
		goto err_get_rstc;
	} else {
		reset_control_deassert(rstc);
		reset_control_put(rstc);
	}

	lsadc_power_reg = LSADC_READL(LSADC0_POWER_ADDR) & ~(LSADC0_CLK_GATING_EN);
	if (st->lsadc[0].clk_gating_en == 1)
		lsadc_power_reg |= LSADC0_CLK_GATING_EN;

	LSADC_WRITEL(lsadc_power_reg, LSADC0_POWER_ADDR);
	dev_info(dev, "[LSADC] write lsadc0_power_reg=0x%x\n", lsadc_power_reg);

	/* Enable JD_power in ananlog ctrl register */
	analog_ctrl_reg = LSADC_READL(LSADC0_ANALOG_CTRL_ADDR);
	analog_ctrl_reg |= LSADC0_ANALOG_CTRL_VALUE;
	LSADC_WRITEL(analog_ctrl_reg, LSADC0_ANALOG_CTRL_ADDR);
	mdelay(100);

	/* LSADC0 */
	ctrl_reg = LSADC_READL(LSADC0_CTRL_ADDR);
	analog_ctrl_reg = LSADC_READL(LSADC0_ANALOG_CTRL_ADDR);
	lsadc_status_reg = LSADC_READL(LSADC0_STATUS_ADDR);
	pad0_reg = LSADC_READL(LSADC0_PAD0_ADDR);
	pad1_reg = LSADC_READL(LSADC0_PAD1_ADDR);

	dev_info(dev, "[LSADC] from device tree: pad0=[activate=%d, ctrl_mode=%d, pad_sw=%d, threshold=%d]\n",
		 st->lsadc[0].pad[0].activate,
		 st->lsadc[0].pad[0].ctrl_mode,
		 st->lsadc[0].pad[0].pad_sw,
		 st->lsadc[0].pad[0].threshold);
	dev_info(dev, "[LSADC] from device tree: pad1=[activate=%d, ctrl_mode=%d, pad_sw=%d, threshold=%d]\n",
		 st->lsadc[0].pad[1].activate,
		 st->lsadc[0].pad[1].ctrl_mode,
		 st->lsadc[0].pad[1].pad_sw,
		 st->lsadc[0].pad[1].threshold);

	dev_info(dev, "[LSADC] current value: ctrl_reg=0x%x, lsadc_status_reg=0x%x, pad0_reg=0x%x, pad1_reg=0x%x\n",
		 ctrl_reg, lsadc_status_reg, pad0_reg, pad1_reg);

	if (st->lsadc[0].pad[0].activate == 1) {
		pad0_reg = pad0_reg | LSADC0_PAD_ACTIVE_MASK;
		if (st->lsadc[0].pad[0].ctrl_mode == 1)
			pad0_reg |= LSADC0_PAD_CTRL_MASK;
		else
			pad0_reg &= ~LSADC0_PAD_CTRL_MASK;

		if (st->lsadc[0].pad[0].pad_sw == 1)
			pad0_reg |= LSADC0_PAD_SW_MASK;
		else
			pad0_reg &= ~LSADC0_PAD_SW_MASK;

		if (st->lsadc[0].pad[0].threshold <= 0xFF &&
		    st->lsadc[0].pad[0].threshold >= 0) {
			pad0_reg |= LSADC0_PAD_THRED_MASK &
				    (st->lsadc[0].pad[0].threshold << LSADC0_PAD_THRED_SHIFT);
		}

		/* set adc_val0 to base line */
		st->lsadc[0].pad[0].adc_val0 = st->lsadc[0].pad[0].adc_val_baseline;
		pad0_reg = (pad0_reg & ~LSADC0_PAD_ADC_VAL_MASK) | st->lsadc[0].pad[0].adc_val0;

		/* enable IIO events */
		st->rising_evt_cfg |= BIT(0);
		st->falling_evt_cfg |= BIT(0);
	} else {
		pad0_reg &= ~LSADC0_PAD_ACTIVE_MASK;

		/* disable IIO events */
		st->rising_evt_cfg &= ~BIT(0);
		st->falling_evt_cfg &= ~BIT(0);
	}
	LSADC_WRITEL(pad0_reg, LSADC0_PAD0_ADDR);
	dev_info(dev, "[LSADC] write pad0_reg=0x%x\n", pad0_reg);

	if (st->lsadc[0].pad[1].activate == 1) {
		pad1_reg |= LSADC0_PAD_ACTIVE_MASK;
		if (st->lsadc[0].pad[1].ctrl_mode == 1)
			pad1_reg |= LSADC0_PAD_CTRL_MASK;
		else
			pad1_reg &= ~LSADC0_PAD_CTRL_MASK;

		if (st->lsadc[0].pad[1].pad_sw == 1)
			pad1_reg |= LSADC0_PAD_SW_MASK;
		else
			pad1_reg &= ~LSADC0_PAD_SW_MASK;

		if (st->lsadc[0].pad[1].threshold <= 0xFF &&
		    st->lsadc[0].pad[1].threshold >= 0) {
			pad1_reg |= LSADC0_PAD_THRED_MASK &
				    (st->lsadc[0].pad[1].threshold << LSADC0_PAD_THRED_SHIFT);
		}

		/* set adc_val0 to base line */
		st->lsadc[0].pad[1].adc_val0 = st->lsadc[0].pad[1].adc_val_baseline;
		pad1_reg = (pad1_reg & ~LSADC0_PAD_ADC_VAL_MASK) | st->lsadc[0].pad[1].adc_val0;

		/* enable IIO events */
		st->rising_evt_cfg |= BIT(1);
		st->falling_evt_cfg |= BIT(1);
	} else {
		pad1_reg &= ~LSADC0_PAD_ACTIVE_MASK;

		/* disable IIO events */
		st->rising_evt_cfg &= ~BIT(1);
		st->falling_evt_cfg &= ~BIT(1);
	}
	LSADC_WRITEL(pad1_reg, LSADC0_PAD1_ADDR);
	dev_info(dev, "[LSADC] write pad1_reg=0x%x\n", pad1_reg);

	ctrl_reg &= ~LSADC0_CTRL_DEBOUNCE_MASK;
	ctrl_reg |= LSADC0_CTRL_ENABLE_MASK | LSADC0_CTRL_SEL_WAIT_DEFAULT;
	ctrl_reg |= st->lsadc[0].debounce_cnt << LSADC0_CTRL_DEBOUNCE_SHIFT;
	LSADC_WRITEL(ctrl_reg, LSADC0_CTRL_ADDR);
	dev_info(dev, "[LSADC] ctrl_reg=0x%x, irq_num=%d\n",
		 ctrl_reg, irq_num);

	if ((lsadc_status_reg & LSADC0_STATUS_IRQ_EN_MASK) != LSADC0_STATUS_IRQ_EN_MASK) {
		lsadc_status_reg = lsadc_status_reg | LSADC0_STATUS_IRQ_EN_MASK;
		LSADC_WRITEL(lsadc_status_reg, LSADC0_STATUS_ADDR);
		dev_info(dev, "[LSADC] write lsadc_status_reg=0x%x\n",
			 lsadc_status_reg);
	}

	if (request_irq(st->lsadc[0].irq, lsadc0_interrupt_pad, IRQF_SHARED, "lsadc0", idev) < 0) {
		dev_err(dev, "[LSADC] unable to request LSADC0 irq#%d\n",
			st->lsadc[0].irq);
		goto err_req_irq;
	}
	/* Enable IRQ for pad0/pad1, set LSADC0_STATUS to 0x0300000 */
	LSADC_WRITEL(LSADC0_STATUS_ENABLE_IRQ, LSADC0_STATUS_ADDR);

	ctrl_reg = LSADC_READL(LSADC0_CTRL_ADDR);
	lsadc_status_reg = LSADC_READL(LSADC0_STATUS_ADDR);
	analog_ctrl_reg = LSADC_READL(LSADC0_ANALOG_CTRL_ADDR);
	pad0_reg = LSADC_READL(LSADC0_PAD0_ADDR);
	pad1_reg = LSADC_READL(LSADC0_PAD1_ADDR);

	st->lsadc[0].pad[0].adc_val0 = pad0_reg & LSADC0_PAD_ADC_VAL_MASK;
	st->lsadc[0].pad[1].adc_val0 = pad1_reg & LSADC0_PAD_ADC_VAL_MASK;

	dev_info(dev, "[LSADC] set ctrl_reg=0x%x, lsadc_status_reg=0x%x, pad0_reg=0x%x, pad1_reg=0x%x\n",
		 ctrl_reg, lsadc_status_reg, pad0_reg, pad1_reg);

	/* Init CRT LSADC PG */
	writel(st->crt_lsadc_pg_val, st->crt_lsadc_pg_addr);
	dev_info(dev, "[LSADC] write crt_lsadc_pg_val=0x%x\n",
		 st->crt_lsadc_pg_val);

	idev->dev.parent = dev;
	idev->dev.of_node = dev->of_node;
	idev->name = dev_name(dev);
	idev->info = &rtk_lsadc_iio_info;
	idev->modes = INDIO_DIRECT_MODE;
	idev->channels = rtk_lsadc_iio_channels;
	idev->num_channels = ARRAY_SIZE(rtk_lsadc_iio_channels);
	ret = iio_device_register(idev);
	if (ret)
		goto err_iio_reg;

	return 0;

err_iio_reg:
	free_irq(st->lsadc[0].irq, idev);
err_req_irq:
err_get_rstc:
	clk_disable_unprepare(st->clk);
err_get_clk:
	return ret;
}

static int rtk_lsadc_remove(struct platform_device *pdev)
{
	struct iio_dev *idev = platform_get_drvdata(pdev);
	struct rtk_lsadc_state *st = iio_priv(idev);

	iio_device_unregister(idev);
	clk_disable_unprepare(st->clk);
	free_irq(st->lsadc[0].irq, idev);
	iio_device_free(idev);
	return 0;
}

static const struct of_device_id rtk_lsadc_of_match[] = {
	{.compatible = "realtek,rtk-lsadc0"},
	{ }
};
MODULE_DEVICE_TABLE(of, rtk_lsadc_of_match);


static struct platform_driver rtk_lsadc_platform_driver = {
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "rtk-lsadc0",
		.of_match_table = rtk_lsadc_of_match,
	},
	.probe		= rtk_lsadc_probe,
	.remove		= rtk_lsadc_remove,
};
module_platform_driver(rtk_lsadc_platform_driver);

MODULE_DESCRIPTION("RTK LSADC0 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk-lsadc0");


