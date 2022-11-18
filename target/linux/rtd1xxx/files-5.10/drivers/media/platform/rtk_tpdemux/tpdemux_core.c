/*
 * Realtek TP Demux Driver
 *
 * Copyright (C) 2020 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/list.h>
#include <linux/pm_runtime.h>
#include <linux/mm.h>
#include <linux/interrupt.h>

#include <media/dmxdev.h>
#include <media/dvb_demux.h>
#include <media/dvb_frontend.h>

#include "tpdemux_common.h"
#include "tpdemux_core.h"
#include "tpdemux_buffer.h"

#define POLL_MSECS 10

static void rtk_tp_power_on(struct rtktpfei *fei)
{
	struct tp_module *tpm;
	int i;

	for (i = 0; i < fei->num_tpm; i++) {
		tpm = &fei->tpm[i];
		clk_prepare_enable(tpm->clk);
	}
}

static void rtk_tp_power_off(struct rtktpfei *fei)
{
	struct tp_module *tpm;
	int i;

	for (i = 0; i < fei->num_tpm; i++) {
		tpm = &fei->tpm[i];
		clk_disable_unprepare(tpm->clk);
	}
}

static int rtk_tp_runtime_resume(struct device *dev)
{
	struct rtktpfei *fei = dev_get_drvdata(dev);

	dev_dbg(dev, "enter %s\n", __func__);
	rtk_tp_power_on(fei);
	dev_dbg(dev, "exit %s\n", __func__);
	return 0;
}

static int rtk_tp_runtime_suspend(struct device *dev)
{
	struct rtktpfei *fei = dev_get_drvdata(dev);

	dev_dbg(dev, "enter %s\n", __func__);
	rtk_tp_power_off(fei);
	dev_dbg(dev, "exit %s\n", __func__);
	return 0;
}

static int rtk_tp_resume(struct device *dev)
{
	struct rtktpfei *fei = dev_get_drvdata(dev);

	if (atomic_read(&fei->open_cnt) == 0)
		return 0;

	dev_info(dev, "enter %s\n", __func__);
	rtk_tp_power_on(fei);
	dev_info(dev, "exit %s\n", __func__);
	return 0;
}

static int rtk_tp_suspend(struct device *dev)
{
	struct rtktpfei *fei = dev_get_drvdata(dev);

	if (atomic_read(&fei->open_cnt) == 0)
		return 0;

	dev_info(dev, "enter %s\n", __func__);
	rtk_tp_power_off(fei);
	dev_info(dev, "exit %s\n", __func__);
	return 0;
}

static const struct dev_pm_ops rtk_tp_pm_ops = {
	.runtime_suspend = rtk_tp_runtime_suspend,
	.runtime_resume  = rtk_tp_runtime_resume,
	.suspend         = rtk_tp_suspend,
	.resume          = rtk_tp_resume,
};

static int rtk_tp_init_tpm(struct rtktpfei *fei, int i)
{
	struct tp_module *tpm = &fei->tpm[i];
	struct device_node *np = fei->dev->of_node;
	struct resource res;
	int ret;

	ret = of_address_to_resource(np, i, &res);
	if (ret)
		return ret;

	tpm->clk = of_clk_get(np, i);
	if (IS_ERR(tpm->clk))
		return PTR_ERR(tpm->clk);

	tpm->addr = res.start;
	tpm->reg_base = of_iomap(np, i);
	tpm->size = ALIGN(resource_size(&res), PAGE_SIZE);
	tpm->active = 0;
	return 0;
}

static void rtk_tp_fini_tpm(struct rtktpfei *fei, int i)
{
	struct tp_module *tpm = &fei->tpm[i];

	clk_put(tpm->clk);
}

static int of_count_tpm_num(struct device_node *np)
{
	int num_reg = of_property_count_u32_elems(np, "reg");
	int num_a_cells = of_n_addr_cells(np);
	int num_n_cells = of_n_addr_cells(np);

	if (!num_reg || !num_a_cells || !num_n_cells)
		return -EINVAL;

	return num_reg / (num_a_cells + num_n_cells);
}

static void output_feedwork(struct work_struct *work)
{
	struct demux_info *dmx = (struct demux_info *)container_of(work,
							struct demux_info, work);
	struct rtktpfei *fei = dmx->fei;
	struct dvb_demux_feed *dvbdmxfeed = fei->dvbdmxfeed;

	if (dvbdmxfeed) {
		rtk_tp_deliver_data(dvbdmxfeed);
	} else {
		pr_err("%s:%d feed NULL\n", __func__, __LINE__);
	}
}

static void rtk_tp_timer_interrupt(struct timer_list *t)
{
	struct rtktpfei *fei = from_timer(fei, t, timer);
	struct demux_info *dmx;
	int dmx_num;

	/* iterate through input block filters */
	for (dmx_num = 0; dmx_num < fei->num_dmx; dmx_num++) {
		dmx = fei->demux_data[dmx_num];
		/* is this descriptor initialised and TP enabled */
		if (dmx->active && dmx->regs.base && rtk_is_tp_enable(dmx)) {
			queue_work(fei->wq, &dmx->work);
		}
	}

	if (fei->global_feed_count > 0)
		mod_timer(&fei->timer, jiffies + msecs_to_jiffies(POLL_MSECS));
}

static int rtk_tp_start_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *demux = dvbdmxfeed->demux;
	struct stdemux *stdemux = (struct stdemux *)demux->priv;
	struct rtktpfei *fei = stdemux->rtktpfei;
	struct demux_info *dmx = fei->demux_data[stdemux->tsin_index];
	struct filter_info *filter;
	int i, index = -1;

	fei->dvbdmxfeed = dvbdmxfeed;

	switch (dvbdmxfeed->type) {
	case DMX_TYPE_TS:
		break;
	default:
		dev_err(fei->dev, "%s:%d Error bailing\n"
			, __func__, __LINE__);
		return -EINVAL;
	}

	if (dvbdmxfeed->type == DMX_TYPE_TS) {
		switch (dvbdmxfeed->pes_type) {
		case DMX_PES_VIDEO:
		case DMX_PES_AUDIO:
		case DMX_PES_TELETEXT:
		case DMX_PES_PCR:
		case DMX_PES_OTHER:
			break;
		default:
			dev_err(fei->dev, "%s:%d Error bailing\n"
				, __func__, __LINE__);
			return -EINVAL;
		}
	}

	if (!atomic_read(&fei->tp_init)) {
		/*reset tp */
		atomic_set(&fei->tp_init, 1);
	}

	mutex_lock(&fei->lock);

	if (fei->tpm[dmx->tp_mapping].active == 0) {
		pm_runtime_get_sync(fei->dev);
		rtk_tp_init(dmx);
		fei->tpm[dmx->tp_mapping].active = 1;
	}

	if (stdemux->running_feed_count == 0) {
		rtk_demux_init(dmx);
		rtk_set_ts_input_select(dmx);
		rtk_tp_stream_control(dmx, TP_STREAMING_START);

		/* FIXME: set ring buffer for M2MTP */
		if (dmx->tp_id == DMX_TP_A_1)
			rtk_tp_set_mmbuffer(dmx);

		INIT_WORK(&dmx->work, output_feedwork);
		dmx->active = 1;
	}

	if (fei->global_feed_count == 0) {
		fei->timer.expires = jiffies +
			msecs_to_jiffies(msecs_to_jiffies(POLL_MSECS));
		mod_timer(&fei->timer, fei->timer.expires);
	}

	if (dvbdmxfeed->pid == 8192) {
		/*
		 * 8192 is a "dummy PID" which means the entire TS, the all TS
		 * are written to the ring buffer specified by PID[0].
		 */
		if (dmx->pid_tbl_info[0].used == 1)
			goto err_inval;

		index = 0;
	} else {
		for (i = 1; i < TP_PID_FILTER_COUNT; i++) {
			if (dmx->pid_tbl_info[i].used == 0) {
				index = i;
				break;
			}
		}
		if (index == -1)
			goto err_busy;
	}

	filter = rtk_filter_init(dmx, index);

	if (!filter)
		goto err_busy;

	dmx->pid_tbl_info[index].used = 1;

	dvbdmxfeed->priv = filter;
	rtk_tp_set_pid_filter(filter, dvbdmxfeed->pid);

	stdemux->running_feed_count++;
	fei->global_feed_count++;
	mutex_unlock(&fei->lock);
	return 0;

err_inval:
	mutex_unlock(&fei->lock);
	return -EINVAL;
err_busy:
	mutex_unlock(&fei->lock);
	return -EBUSY;
}

static int rtk_tp_stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *demux = dvbdmxfeed->demux;
	struct stdemux *stdemux = (struct stdemux *)demux->priv;
	struct rtktpfei *fei = stdemux->rtktpfei;
	struct demux_info *dmx = fei->demux_data[stdemux->tsin_index];
	struct filter_info *filter = dvbdmxfeed->priv;

	mutex_lock(&fei->lock);

	if (--fei->global_feed_count == 0)
		del_timer(&fei->timer);

	if (--stdemux->running_feed_count == 0) {
		rtk_tp_stream_control(dmx, TP_STREAMING_STOP);

		mutex_unlock(&stdemux->dmxdev.mutex);
		cancel_work_sync(&dmx->work);
		mutex_lock(&stdemux->dmxdev.mutex);


		dmx->active = 0;
	}

	if (fei->tpm[dmx->tp_mapping].active == 1 &&
				stdemux->running_feed_count == 0)
		fei->tpm[dmx->tp_mapping].active = 0;

	rtk_filter_uninit(filter);

	mutex_unlock(&fei->lock);

	return 0;
}


static int rtk_tp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rtktpfei *fei;
	struct device_node *child;
	struct demux_info *dmx;
	int ret, index = 0;
	const char *input;
	int i;

	fei = devm_kzalloc(dev, sizeof(struct rtktpfei), GFP_KERNEL);
	if (!fei)
		return -ENOMEM;
	fei->dev = dev;

	fei->num_tpm = of_count_tpm_num(np);
	if (fei->num_tpm < 0)
		return fei->num_tpm;

	for (i = 0; i < fei->num_tpm; i++) {
		ret = rtk_tp_init_tpm(fei, i);
		if (ret)
			goto error;
	}

	atomic_set(&fei->open_cnt, 0);

	platform_set_drvdata(pdev, fei);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);


	for_each_child_of_node(np, child) {
		fei->demux_data[index] = devm_kzalloc(dev,
						sizeof(struct demux_info),
						GFP_KERNEL);
		dmx = fei->demux_data[index];

		dmx->fei = fei;

		ret = of_property_read_u32(child, "tp-index", &dmx->tp_id);
		if (ret) {
			dev_err(&pdev->dev, "[TP] No tp-index found\n");
			goto error;
		}

		if (dmx->tp_id == DMX_TP_A_0 || dmx->tp_id == DMX_TP_A_1)
			dmx->tp_mapping = TP_A;
		else if (dmx->tp_id == DMX_TP_B_0 || dmx->tp_id == DMX_TP_B_1)
			dmx->tp_mapping = TP_B;
		else
			dmx->tp_mapping = TP_C;

		mutex_init(&dmx->regs.buf_ctrl_mutex);
		mutex_init(&dmx->regs.pid_ctrl_mutex);

		dmx->regs.base = fei->tpm[dmx->tp_mapping].reg_base;

		ret = of_property_read_u32(child, "serial",
						&dmx->hw_info.serial);
		if (ret) {
			dev_err(&pdev->dev, "[TP] No serial found\n");
			goto error;
		}

		ret = of_property_read_u32(child, "data_order",
						&dmx->hw_info.data_order);
		if (ret) {
			dev_err(&pdev->dev, "[TP] No data_order found\n");
			goto error;
		}

		ret = of_property_read_u32(child, "datapin",
						&dmx->hw_info.datapin);
		if (ret) {
			dev_err(&pdev->dev, "[TP] No datapin found\n");
			goto error;
		}

		ret = of_property_read_u32(child, "err_pol",
						&dmx->hw_info.err_pol);
		if (ret) {
			dev_err(&pdev->dev, "[TP] No err_pol found\n");
			goto error;
		}

		ret = of_property_read_u32(child, "sync_pol",
						&dmx->hw_info.sync_pol);
		if (ret) {
			dev_err(&pdev->dev, "[TP] No sync_pol found\n");
			goto error;
		}

		ret = of_property_read_u32(child, "val_pol",
						&dmx->hw_info.val_pol);
		if (ret) {
			dev_err(&pdev->dev, "[TP] No val_pol found\n");
			goto error;
		}

		ret = of_property_read_u32(child, "clk_pol",
						&dmx->hw_info.clk_pol);
		if (ret) {
			dev_err(&pdev->dev, "[TP] No clk_pol found\n");
			goto error;
		}

		ret = of_property_read_string(child, "input", &input);
		if (ret) {
			dev_err(&pdev->dev, "[TP] No input found\n");
			goto error;
		}
		if (input) {
			if (strcmp(input, "ts0") == 0)
				dmx->input_sel = TS_IN_TS0_PAD;
			else if (strcmp(input, "internal") == 0)
				dmx->input_sel = TS_IN_INTERNAL_DEMOD;
			else if (strcmp(input, "tsio") == 0)
				dmx->input_sel = TS_IN_TSIO;
			else if (strcmp(input, "ts1") == 0)
				dmx->input_sel = TS_IN_TS1_PAD;
			else
				dmx->input_sel = TS_IN_TS0_PAD;
		}
		tp_reg_mapping(dmx->tp_id, &dmx->regs);
		index++;
	}
	fei->num_dmx = index;
	mutex_init(&fei->lock);

	/* Get the configuration information about the tp */
	ret = rtk_tp_register(&fei->rtktpfe[0],
					(void *)fei,
					rtk_tp_start_feed,
					rtk_tp_stop_feed);
	if (ret) {
		dev_err(dev, "rtktpfe_tuner_register_frontend failed (%d)\n",
			ret);
	}

	fei->wq = create_singlethread_workqueue("tpdemux_fei");
	if (!fei->wq) {
		dev_err(dev, "initialize fei workqueue failed\n");
		return -ENOMEM;
	}

	timer_setup(&fei->timer, rtk_tp_timer_interrupt, 0);

	dev_info(dev, "initialized\n");
	return 0;
error:
	dev_err(dev, "failed to init tpm%d: %d\n", ret, i);
	for (; i >= 0; i--)
		rtk_tp_fini_tpm(fei, i);
	return ret;
}

static int rtk_tp_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtktpfei *fei = platform_get_drvdata(pdev);
	int i;

	del_timer(&fei->timer);

	if (fei->wq) {
		flush_workqueue(fei->wq);
		destroy_workqueue(fei->wq);
	}


	rtk_tp_unregister(fei->rtktpfe[0], fei);

	for (i = 0; i < fei->num_dmx; i++)
		devm_kfree(fei->dev, fei->demux_data[i]);

	pm_runtime_disable(dev);
	platform_set_drvdata(pdev, NULL);
	dev_info(dev, "removed\n");
	for (i = 0; i < fei->num_tpm; i++)
		rtk_tp_fini_tpm(fei, i);
	return 0;
}


static const struct of_device_id rtk_tp_ids[] = {
	{ .compatible = "realtek,tp" },
	{}
};

static struct platform_driver rtk_tp_driver = {
	.probe = rtk_tp_probe,
	.remove = rtk_tp_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "rtk-tp",
		.of_match_table = rtk_tp_ids,
		.pm = &rtk_tp_pm_ops,

	},
};
module_platform_driver(rtk_tp_driver);
MODULE_DESCRIPTION("Realtek TP Demux Driver");
MODULE_LICENSE("GPL");

