// SPDX-License-Identifier: GPL-2.0-only
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include "hse.h"

#define USE_DMA_POOL 1

static inline struct device *cq2dev(struct hse_command_queue *cq)
{
	return cq->hse_dev->dev;
}

struct hse_command_queue *hse_cq_alloc(struct hse_device *hse_dev)
{
	struct hse_command_queue *cq = NULL;

	cq = kzalloc(sizeof(*cq), GFP_KERNEL);
	if (!cq)
		return NULL;

	cq->hse_dev = hse_dev;
	cq->size = PAGE_SIZE;
#if USE_DMA_POOL
	cq->virt = dma_pool_alloc(hse_dev->pool, GFP_NOWAIT, &cq->phys);
#else
	cq->virt = dma_alloc_coherent(hse_dev->dev, cq->size, &cq->phys, GFP_NOWAIT);
#endif
	if (!cq->virt)
		goto free_cq;

	dev_dbg(cq2dev(cq), "cq=%pK: alloc_cq: phys=%pad, virt=%pK\n", cq, &cq->phys, cq->virt);
	return cq;

free_cq:
	kfree(cq);
	return NULL;
}

struct hse_command_queue *hse_cq_alloc_compact(struct hse_device *hse_dev)
{
	struct hse_command_queue *cq = NULL;
	void *p;

	p = kzalloc(HSE_CQ_COMPACT_SIZE, GFP_KERNEL);
	if (!p)
		return NULL;
	cq = p;

	cq->hse_dev = hse_dev;
	cq->size = HSE_CQ_COMPACT_SIZE - sizeof(*cq);
	cq->virt = p + sizeof(*cq);
	cq->is_compact = 1;

	dev_dbg(cq2dev(cq), "cq=%pK: alloc_cq_compact: virt=%pK\n", cq, cq->virt);
	return cq;
}

void hse_cq_free(struct hse_command_queue *cq)
{
	if (cq->is_compact) {
		kfree(cq);
		return;
	}

#if USE_DMA_POOL
	dma_pool_free(cq->hse_dev->pool, cq->virt, cq->phys);
#else
	dma_free_coherent(cq->hse_dev->dev, cq->size, cq->virt, cq->phys);
#endif
	kfree(cq);
}

static inline int __hse_cq_add_data_one(struct hse_command_queue *cq, u32 data)
{
	u32 *ptr = (cq->virt + cq->pos);

	if (cq->pos >= cq->size) {
		WARN_ONCE(1, "cq full\n");
		return -EINVAL;
	}

	dev_dbg(cq2dev(cq), "cq=%pK: add data=%08x\n", cq, data);
	*ptr++ = data;
	cq->pos += 4;

	if ((cq->pos % 16) == 0) {
		u32 tmp;

		ptr -= 4;

		tmp    = ptr[0];
		ptr[0] = ptr[3];
		ptr[3] = tmp;

		tmp    = ptr[1];
		ptr[1] = ptr[2];
		ptr[2] = tmp;
	}
	return 0;
}

int hse_cq_append_compact(struct hse_command_queue *cq, struct hse_command_queue *compact)
{
	if ((cq->pos + compact->pos + 16) > cq->size)
		return -EINVAL;

	while ((cq->pos % 16) != 0)
		__hse_cq_add_data_one(cq, 0);

	memcpy(cq->virt + cq->pos, compact->virt, compact->pos);
	cq->pos += compact->pos;
	return 0;
}

int hse_cq_add_data(struct hse_command_queue *cq, u32 *data, size_t size)
{
	int ret;
	int i;

	for (i = 0; i < size; i++) {
		ret = __hse_cq_add_data_one(cq, data[i]);
		if (ret)
			return ret;
	}
	return 0;
}

void hse_cq_pad(struct hse_command_queue *cq)
{
	struct device *dev = cq->hse_dev->dev;

	__hse_cq_add_data_one(cq, 0);
	while (cq->pos % 16)
		__hse_cq_add_data_one(cq, 0);
	dma_sync_single_for_device(dev, cq->phys, cq->size, DMA_TO_DEVICE);
}

void hse_cq_reset(struct hse_command_queue *cq)
{
	cq->pos = 0;
}
