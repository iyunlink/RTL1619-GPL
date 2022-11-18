// SPDX-License-Identifier: GPL-2.0-only
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include "hse.h"

static inline struct device *eng2dev(struct hse_engine *eng)
{
	return eng->hse_dev->dev;
}

static inline int hse_engine_read(struct hse_engine *eng, int offset)
{
	return hse_read(eng->hse_dev, eng->base_offset + offset);
}

static inline void hse_engine_write(struct hse_engine *eng, int offset, unsigned int val)
{
	hse_write(eng->hse_dev, eng->base_offset + offset, val);
}

static void hse_engine_stop(struct hse_engine *eng)
{
	/* stop engine */
	hse_engine_write(eng, HSE_REG_ENGINE_OFFSET_INTC, 0);
	hse_engine_write(eng, HSE_REG_ENGINE_OFFSET_Q,    0);

	/* clear ints */
	hse_engine_write(eng, HSE_REG_ENGINE_OFFSET_INTS, 0x6);
}

static struct hse_command_queue *hse_engine_next_cq(struct hse_engine *eng)
{
	return list_first_entry_or_null(&eng->list, struct hse_command_queue, node);
}

/* run with engine locked */
static void hse_engine_execute_cq(struct hse_engine *eng)
{
	struct hse_command_queue *cq;

	lockdep_assert_held(&eng->lock);

	cq = hse_engine_next_cq(eng);
	if (!cq)
		return;
	list_del(&cq->node);

	hse_cq_pad(cq);
	eng->cq = cq;

	hse_engine_write(eng, HSE_REG_ENGINE_OFFSET_QB, cq->phys);
	hse_engine_write(eng, HSE_REG_ENGINE_OFFSET_QL, cq->phys + cq->size);
	hse_engine_write(eng, HSE_REG_ENGINE_OFFSET_QR, cq->phys);
	hse_engine_write(eng, HSE_REG_ENGINE_OFFSET_QW, cq->phys + cq->pos);

	dev_dbg(eng2dev(eng), "cq=%p,qb=%#x,ql=%#x,qr=%#x,qw=%#x\n", cq,
		hse_engine_read(eng, HSE_REG_ENGINE_OFFSET_QB),
		hse_engine_read(eng, HSE_REG_ENGINE_OFFSET_QL),
		hse_engine_read(eng, HSE_REG_ENGINE_OFFSET_QR),
		hse_engine_read(eng, HSE_REG_ENGINE_OFFSET_QW));

	/* start engine */
	hse_engine_write(eng, HSE_REG_ENGINE_OFFSET_INTC, 0x6);
	hse_engine_write(eng, HSE_REG_ENGINE_OFFSET_Q, 0x1);
}

void hse_engine_add_cq(struct hse_engine *eng, struct hse_command_queue *cq)
{
	unsigned long flags;

	if (cq->is_compact) {
		dev_err(eng2dev(eng), "%s: add a compact cq\n", __func__);
		return;
	}

	spin_lock_irqsave(&eng->lock, flags);
	list_add_tail(&cq->node, &eng->list);
	spin_unlock_irqrestore(&eng->lock, flags);
}

void hse_engine_remove_cq(struct hse_engine *eng, struct hse_command_queue *cq)
{
	unsigned long flags;

	spin_lock_irqsave(&eng->lock, flags);
	if (eng->cq == cq) {
		hse_engine_stop(eng);
		eng->cq = NULL;
		hse_engine_execute_cq(eng);
	} else
		list_del(&cq->node);
	spin_unlock_irqrestore(&eng->lock, flags);
}

void hse_engine_issue_cq(struct hse_engine *eng)
{
	unsigned long flags;

	spin_lock_irqsave(&eng->lock, flags);
	if (!eng->cq)
		hse_engine_execute_cq(eng);
	spin_unlock_irqrestore(&eng->lock, flags);
}

void hse_engine_handle_interrupt(struct hse_engine *eng)
{
	struct hse_command_queue *cq;
	uint32_t raw_ints;

	spin_lock(&eng->lock);
	raw_ints = hse_engine_read(eng, HSE_REG_ENGINE_OFFSET_INTS);
	if (raw_ints == 0) {
		spin_unlock(&eng->lock);
		return;
	}
	hse_engine_stop(eng);
	cq = eng->cq;
	spin_unlock(&eng->lock);

	dev_dbg(eng2dev(eng), "cq=%pK,qb=%#x,ql=%#x,qr=%#x,qw=%#x,ints=%#x\n", cq,
		hse_engine_read(eng, HSE_REG_ENGINE_OFFSET_QB),
		hse_engine_read(eng, HSE_REG_ENGINE_OFFSET_QL),
		hse_engine_read(eng, HSE_REG_ENGINE_OFFSET_QR),
		hse_engine_read(eng, HSE_REG_ENGINE_OFFSET_QW),
		raw_ints);

	if (!cq) {
		dev_warn(eng2dev(eng), "interrupt raised with no cq\n");
		return;
	}

	if (raw_ints & 0x4)
		cq->status |= HSE_STATUS_IRQ_CMD_ERR;
	if (raw_ints & 0x2)
		cq->status |= HSE_STATUS_IRQ_OK;

	if (cq->cb)
		cq->cb(cq->cb_data);

	spin_lock(&eng->lock);
	eng->cq = NULL;
	hse_engine_execute_cq(eng);
	spin_unlock(&eng->lock);
}

int hse_engine_init(struct hse_device *hse_dev, struct hse_engine *eng, int index)
{
	eng->base_offset = HSE_REG_ENGINE_BASE(index);
	eng->hse_dev  = hse_dev;
	spin_lock_init(&eng->lock);
	INIT_LIST_HEAD(&eng->list);

	hse_engine_write(eng, HSE_REG_ENGINE_OFFSET_Q,   0);
	hse_engine_write(eng, HSE_REG_ENGINE_OFFSET_QCL, 0);
	hse_engine_write(eng, HSE_REG_ENGINE_OFFSET_QCH, 0);

	return 0;
}
