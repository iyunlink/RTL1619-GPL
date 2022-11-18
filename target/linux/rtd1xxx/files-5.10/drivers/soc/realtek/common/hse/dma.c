// SPDX-License-Identifier: GPL-2.0-only

#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include "hse.h"

#define MERGE_CQ

struct hse_dma_desc {
	struct dma_async_tx_descriptor tx;
	struct dmaengine_result tx_result;
	struct list_head node;
	struct hse_command_queue *compact_cq;
	struct list_head desc_merged;
};

struct hse_dma_chan {
	struct dma_chan chan;
	struct tasklet_struct task;
	struct hse_dma_desc *desc;
	struct hse_command_queue *cq;
	struct list_head desc_allocated;
	struct list_head desc_submitted;
	struct list_head desc_issued;
	struct list_head desc_completed;
#ifdef MERGE_CQ
	struct list_head desc_pool;
#endif
	spinlock_t lock;
};

static inline struct hse_dma_chan *chan_to_hse_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct hse_dma_chan, chan);
}

static inline struct hse_device *chan_to_hse_device(struct dma_chan *c)
{
	return container_of(c->device, struct hse_device, dma_dev);
}

static struct hse_dma_desc *hse_dma_alloc_desc(struct hse_dma_chan *chan)
{
	struct hse_device *hse_dev = chan_to_hse_device(&chan->chan);
	struct hse_dma_desc *desc;

	desc = kzalloc(sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return NULL;

	desc->compact_cq = hse_cq_alloc_compact(hse_dev);
	if (!desc->compact_cq) {
		kfree(desc);
		return NULL;
	}

	pr_debug("%s: desc=%pK: allocted\n", __func__, desc);

	return desc;
}

static void hse_dma_free_desc(struct hse_dma_desc *desc)
{
	pr_debug("%s: desc=%pK: freed\n", __func__, desc);

	hse_cq_free(desc->compact_cq);
	kfree(desc);
}

static struct hse_dma_desc *hse_dma_get_desc_from_pool(struct hse_dma_chan *chan)
{
	struct hse_dma_desc *desc;
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	desc = list_first_entry_or_null(&chan->desc_pool, struct hse_dma_desc, node);
	if (!desc) {
		spin_unlock_irqrestore(&chan->lock, flags);
		return hse_dma_alloc_desc(chan);
	}
	list_del(&desc->node);
	spin_unlock_irqrestore(&chan->lock, flags);

	hse_cq_reset(desc->compact_cq);
	memset(&desc->tx, 0, sizeof(desc->tx));
	memset(&desc->tx_result, 0, sizeof(desc->tx_result));

	return desc;
}

static void hse_dma_put_desc_to_pool(struct hse_dma_desc *desc)
{
	struct hse_dma_chan *chan = chan_to_hse_dma_chan(desc->tx.chan);

	list_add(&desc->node, &chan->desc_pool);
}

static int hse_dma_desc_free(struct dma_async_tx_descriptor *tx)
{
	struct hse_dma_desc *desc = container_of(tx, struct hse_dma_desc, tx);
	struct hse_dma_chan *chan = chan_to_hse_dma_chan(desc->tx.chan);
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	list_del_init(&desc->node);
	hse_dma_put_desc_to_pool(desc);
	spin_unlock_irqrestore(&chan->lock, flags);
	return 0;
}

static dma_cookie_t hse_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct hse_dma_chan *chan = chan_to_hse_dma_chan(tx->chan);
	struct hse_dma_desc *desc = container_of(tx, struct hse_dma_desc, tx);
	unsigned long flags;
	dma_cookie_t cookie;

	spin_lock_irqsave(&chan->lock, flags);

	cookie = dma_cookie_assign(tx);
	list_move_tail(&desc->node, &chan->desc_submitted);

	spin_unlock_irqrestore(&chan->lock, flags);

	pr_debug("%s: desc=%pK: submitted, cookie=%x\n", __func__, desc, cookie);

	return cookie;
}

static void hse_dma_desc_init(struct hse_dma_chan *chan, struct hse_dma_desc *desc, unsigned long tx_flags)
{
	unsigned long flags;

	dma_async_tx_descriptor_init(&desc->tx, &chan->chan);
	desc->tx.flags = tx_flags;
	desc->tx.tx_submit = hse_dma_tx_submit;
	desc->tx.desc_free = hse_dma_desc_free;

	desc->tx_result.result = DMA_TRANS_NOERROR;
	desc->tx_result.residue = 0;

	spin_lock_irqsave(&chan->lock, flags);
	list_add_tail(&desc->node, &chan->desc_allocated);
	spin_unlock_irqrestore(&chan->lock, flags);
}

static struct dma_async_tx_descriptor *hse_dma_prep_dma_xor(struct dma_chan *c,
	dma_addr_t dst, dma_addr_t *src, unsigned int src_cnt, size_t len,
	unsigned long flags)
{
	struct hse_device *hse_dev = chan_to_hse_device(c);
	struct hse_dma_chan *chan = chan_to_hse_dma_chan(c);
	struct hse_dma_desc *desc;
	int ret;

	desc = hse_dma_get_desc_from_pool(chan);
	if (!desc)
		return NULL;
	hse_dma_desc_init(chan, desc, flags);

	ret = hse_cq_prep_xor(hse_dev, desc->compact_cq, dst, src, src_cnt, len);
	if (ret)
		return NULL;

	pr_debug("%s: desc=%pK\n", __func__, desc);

	return &desc->tx;
}

static struct dma_async_tx_descriptor *hse_dma_prep_dma_memcpy(struct dma_chan *c,
	dma_addr_t dst, dma_addr_t src, size_t len, unsigned long flags)
{
	struct hse_device *hse_dev = chan_to_hse_device(c);
	struct hse_dma_chan *chan = chan_to_hse_dma_chan(c);
	struct hse_dma_desc *desc;
	int ret;

	desc = hse_dma_get_desc_from_pool(chan);
	if (!desc)
		return NULL;
	hse_dma_desc_init(chan, desc, flags);

	ret = hse_cq_prep_copy(hse_dev, desc->compact_cq, dst, src, len);
	if (ret)
		return NULL;

	pr_debug("%s: desc=%pK\n", __func__, desc);

	return &desc->tx;
}

static void hse_dma_complete(struct tasklet_struct *t)
{
	struct hse_dma_chan *chan = from_tasklet(chan, t, task);
	struct hse_dma_desc *desc, *_desc;
	LIST_HEAD(completed);
	LIST_HEAD(processed);
	dma_cookie_t cookie;
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	list_splice_tail_init(&chan->desc_completed, &completed);
	spin_unlock_irqrestore(&chan->lock, flags);

	list_for_each_entry_safe(desc, _desc, &completed, node) {
		cookie = desc->tx.cookie;
		pr_debug("%s: desc=%pK: completed cookie=%x\n", __func__, desc, cookie);

		/*
		 * FIXME: dma_descriptor_unmap must called before dma_cookie_complete and
		 *        dmaengine_desc_get_callback_invoke to prevent raid6test failed.
		 */
		dma_descriptor_unmap(&desc->tx);

		dma_cookie_complete(&desc->tx);
		dmaengine_desc_get_callback_invoke(&desc->tx, &desc->tx_result);

		desc->tx.callback = NULL;
		desc->tx.callback_result = NULL;
		desc->tx.callback_param = NULL;

		list_move_tail(&desc->node, &processed);
	}

	spin_lock_irqsave(&chan->lock, flags);
	list_splice_tail_init(&processed, &chan->desc_pool);
	spin_unlock_irqrestore(&chan->lock, flags);
}

static void hse_dma_chan_start_transfer(struct hse_dma_chan *chan);

static void hse_dma_transfer_complete(void *p)
{
	struct hse_dma_chan *chan = p;
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);

	if (!chan->desc) {
		pr_warn("%s: no desc\n", __func__);
		spin_unlock_irqrestore(&chan->lock, flags);
		return;
	}

	list_add_tail(&chan->desc->node, &chan->desc_completed);
#ifdef MERGE_CQ
	list_splice_tail_init(&chan->desc->desc_merged, &chan->desc_completed);
#endif
	chan->desc = NULL;

	hse_dma_chan_start_transfer(chan);

	spin_unlock_irqrestore(&chan->lock, flags);

	tasklet_schedule(&chan->task);
}

static inline struct hse_dma_desc *hse_dma_next_desc(struct hse_dma_chan *chan)
{
	return list_first_entry_or_null(&chan->desc_issued, struct hse_dma_desc, node);
}

static void hse_dma_chan_start_transfer(struct hse_dma_chan *chan)
{
	struct hse_device *hse_dev = chan_to_hse_device(&chan->chan);
	struct hse_dma_desc *desc;
	int ret;

	chan->desc = hse_dma_next_desc(chan);
	if (!chan->desc)
		return;
	list_del(&chan->desc->node);

	hse_cq_reset(chan->cq);
	hse_cq_append_compact(chan->cq, chan->desc->compact_cq);

#ifdef MERGE_CQ
	INIT_LIST_HEAD(&chan->desc->desc_merged);
	for (desc = hse_dma_next_desc(chan); desc != NULL; desc = hse_dma_next_desc(chan)) {
		ret = hse_cq_append_compact(chan->cq, desc->compact_cq);
		if (ret)
			break;
		list_move_tail(&desc->node, &chan->desc->desc_merged);
	}
#endif
	hse_cq_set_complete_callback(chan->cq, hse_dma_transfer_complete, chan);
	chan->cq->status = 0;

	hse_engine_add_cq(&hse_dev->eng, chan->cq);

	pr_debug("%s: desc=%pK: queued\n", __func__, chan->desc);

	hse_engine_issue_cq(&hse_dev->eng);
}

static inline bool hse_dma_has_issue_pending(struct hse_dma_chan *chan)
{
	list_splice_tail_init(&chan->desc_submitted, &chan->desc_issued);
	return !list_empty(&chan->desc_issued);
}

static void hse_dma_issue_pending(struct dma_chan *c)
{
	struct hse_dma_chan *chan = chan_to_hse_dma_chan(c);
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	if (hse_dma_has_issue_pending(chan) && !chan->desc)
		hse_dma_chan_start_transfer(chan);
	spin_unlock_irqrestore(&chan->lock, flags);
}

static inline void hse_dma_get_all_desc(struct hse_dma_chan *chan, struct list_head *head)
{
	list_splice_tail_init(&chan->desc_allocated, head);
	list_splice_tail_init(&chan->desc_submitted, head);
	list_splice_tail_init(&chan->desc_issued, head);
	list_splice_tail_init(&chan->desc_completed, head);
}

static int hse_dma_terminate_all(struct dma_chan *c)
{
	struct hse_device *hse_dev = chan_to_hse_device(c);
	struct hse_dma_chan *chan = chan_to_hse_dma_chan(c);
	LIST_HEAD(head);
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);

	if (chan->desc) {
		hse_engine_remove_cq(&hse_dev->eng, chan->cq);
		list_move_tail(&chan->desc->node, &chan->desc_pool);
		chan->desc = NULL;
	}

	hse_dma_get_all_desc(chan, &head);
	list_splice_tail_init(&head, &chan->desc_pool);

	spin_unlock_irqrestore(&chan->lock, flags);

	return 0;
}

static void hse_dma_synchronize(struct dma_chan *c)
{
	struct hse_dma_chan *chan = chan_to_hse_dma_chan(c);

	tasklet_kill(&chan->task);
}

static int hse_dma_alloc_chan_resources(struct dma_chan *c)
{
	struct hse_device *hse_dev = chan_to_hse_device(c);
	struct hse_dma_chan *chan = chan_to_hse_dma_chan(c);
	int i;
	unsigned long flags;
	LIST_HEAD(head);

	chan->cq = hse_cq_alloc(hse_dev);
	if (!chan->cq)
		return -ENOMEM;

	for (i = 0; i < HSE_DMA_PREALLOCATED_DESC_NUM; i++) {
		struct hse_dma_desc *desc = hse_dma_alloc_desc(chan);

		if (!desc) {
			dev_warn(hse_dev->dev, "%s: alloc desc failed\n", __func__);
			continue;
		}

		list_add(&desc->node, &head);
	}

	spin_lock_irqsave(&chan->lock, flags);
	list_splice_init(&head, &chan->desc_pool);
	spin_unlock_irqrestore(&chan->lock, flags);

	return 0;
}

static void hse_dma_free_chan_resources(struct dma_chan *c)
{
	struct hse_dma_chan *chan = chan_to_hse_dma_chan(c);
	unsigned long flags;
	struct hse_dma_desc *desc, *_desc;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->lock, flags);
	hse_dma_get_all_desc(chan, &head);
	list_splice_init(&chan->desc_pool, &head);
	spin_unlock_irqrestore(&chan->lock, flags);

	list_for_each_entry_safe(desc, _desc, &head, node)
		hse_dma_free_desc(desc);
	hse_cq_free(chan->cq);
}

static void hse_dma_add_chan(struct dma_device *dma, struct hse_dma_chan *chan)
{
	dma_cookie_init(&chan->chan);

	spin_lock_init(&chan->lock);
	INIT_LIST_HEAD(&chan->desc_pool);
	INIT_LIST_HEAD(&chan->desc_allocated);
	INIT_LIST_HEAD(&chan->desc_submitted);
	INIT_LIST_HEAD(&chan->desc_issued);
	INIT_LIST_HEAD(&chan->desc_completed);

	tasklet_setup(&chan->task, hse_dma_complete);

	chan->chan.device = dma;
	list_add_tail(&chan->chan.device_node, &dma->channels);
}

static void hse_dma_remove_chan(struct hse_dma_chan *chan)
{
	tasklet_kill(&chan->task);
}

int hse_setup_dmaengine(struct hse_device *hse_dev)
{
	struct dma_device *dma = &hse_dev->dma_dev;
	int i;

	hse_dev->chans_num = 1;

	dma_cap_zero(dma->cap_mask);
	dma_cap_set(DMA_MEMCPY, dma->cap_mask);
	dma_cap_set(DMA_XOR, dma->cap_mask);

	dma->dev                    = hse_dev->dev;
	dma->max_xor                = 5;
	dma->device_prep_dma_xor    = hse_dma_prep_dma_xor;
	dma->device_prep_dma_memcpy = hse_dma_prep_dma_memcpy;
	dma->device_tx_status       = dma_cookie_status;
	dma->device_issue_pending   = hse_dma_issue_pending;
	dma->device_terminate_all   = hse_dma_terminate_all;
	dma->device_synchronize     = hse_dma_synchronize;
	dma->device_alloc_chan_resources = hse_dma_alloc_chan_resources;
	dma->device_free_chan_resources  = hse_dma_free_chan_resources;
	INIT_LIST_HEAD(&dma->channels);

	hse_dev->chans = kcalloc(hse_dev->chans_num, sizeof(*hse_dev->chans), GFP_KERNEL);
	if (!hse_dev->chans)
		return -ENOMEM;

	for (i = 0; i < hse_dev->chans_num; i++)
		hse_dma_add_chan(dma, &hse_dev->chans[i]);

	return dmaenginem_async_device_register(dma);
}

void hse_teardown_dmaengine(struct hse_device *hse_dev)
{
	int i;

	dma_async_device_unregister(&hse_dev->dma_dev);
	for (i = 0; i < hse_dev->chans_num; i++)
		hse_dma_remove_chan(&hse_dev->chans[i]);
	kfree(hse_dev->chans);
}
