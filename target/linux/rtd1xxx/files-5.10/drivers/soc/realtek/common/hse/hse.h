/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SOC_REALTEK_HSE_H__
#define __SOC_REALTEK_HSE_H__

#include <linux/clk.h>
#include <linux/dmapool.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "virt-dma.h"

#define HSE_CQ_COMPACT_SIZE                512
#define HSE_DMA_PREALLOCATED_DESC_NUM      128

#define HSE_MAX_ENGINES                 (1)

#define HSE_REG_ENGINE_BASE(i)          (0x200 + 0x100 * (i))
#define HSE_REG_ENGINE_OFFSET_QB        0x00
#define HSE_REG_ENGINE_OFFSET_QL        0x04
#define HSE_REG_ENGINE_OFFSET_QR        0x08
#define HSE_REG_ENGINE_OFFSET_QW        0x0C
#define HSE_REG_ENGINE_OFFSET_Q         0x10
#define HSE_REG_ENGINE_OFFSET_INTS      0x14
#define HSE_REG_ENGINE_OFFSET_SWAP      0x18
#define HSE_REG_ENGINE_OFFSET_QCL       0x1C
#define HSE_REG_ENGINE_OFFSET_QCH       0x20
#define HSE_REG_ENGINE_OFFSET_INTC      0x24

#define HSE_REG_BYPASS                  0x41c

enum {
	HSE_STATUS_IRQ_OK = 0x1,
	HSE_STATUS_IRQ_CMD_ERR = 0x2,
};

struct hse_device;
struct hse_command_queue;

struct hse_engine {
	struct hse_device *hse_dev;
	int base_offset;
	spinlock_t lock;
	struct list_head list;
	struct hse_command_queue *cq;
};

int hse_engine_init(struct hse_device *hdev, struct hse_engine *eng, int index);
void hse_engine_handle_interrupt(struct hse_engine *eng);
void hse_engine_add_cq(struct hse_engine *eng, struct hse_command_queue *cq);
void hse_engine_remove_cq(struct hse_engine *eng, struct hse_command_queue *cq);
void hse_engine_issue_cq(struct hse_engine *eng);

struct hse_command_queue {
	struct hse_device *hse_dev;
	dma_addr_t phys;
	void *virt;
	size_t size;
	int pos;
	int is_compact;

	int status;
	void (*cb)(void *data);
	void *cb_data;
	struct list_head node;
};

struct hse_command_queue *hse_cq_alloc(struct hse_device *hse_dev);
void hse_cq_free(struct hse_command_queue *cq);
int hse_cq_add_data(struct hse_command_queue *cq, u32 *data, size_t size);
void hse_cq_pad(struct hse_command_queue *cq);
void hse_cq_reset(struct hse_command_queue *cq);
struct hse_command_queue *hse_cq_alloc_compact(struct hse_device *hse_dev);
int hse_cq_append_compact(struct hse_command_queue *cq, struct hse_command_queue *compact_cq);

static inline void hse_cq_set_complete_callback(struct hse_command_queue *cq,
	void (*cb)(void *data), void *cb_data)
{
	cq->cb = cb;
	cq->cb_data = cb_data;
}

struct hse_quirks;
struct hse_dma_chan;

struct hse_device {
	struct miscdevice mdev;
	struct device *dev;
	struct clk *clk;
	struct reset_control *rstc;
	struct reset_control *rstc_bist;
	void *base;
	int irq;
	const struct hse_quirks *quirks;
	struct hse_engine eng;
	struct dma_pool *pool;

	struct dma_device dma_dev;
	int chans_num;
	struct hse_dma_chan *chans;

	int miscdevice_ready : 1;
	int dmaengine_ready : 1;
};

static inline void hse_write(struct hse_device *hse_dev, unsigned int offset,
	u32 val)
{
	dev_dbg(hse_dev->dev, "w: offset=%03x, val=%08x\n", offset, val);
	writel(val, hse_dev->base + offset);
}

static inline int hse_read(struct hse_device *hse_dev, unsigned int offset)
{
	u32 v;

	v = readl(hse_dev->base + offset);
	dev_dbg(hse_dev->dev, "r: offset=%03x, val=%08x\n", offset, v);
	return v;
}

struct hse_quirks {
	unsigned int bypass_en_disable : 1;
	unsigned int xor_copy_v2 : 1;
};

static inline int hse_should_disable_bypass_en(struct hse_device *hse_dev)
{
	if (!hse_dev->quirks)
		return 0;
	return hse_dev->quirks->bypass_en_disable;
}

static inline int hse_should_workaround_copy(struct hse_device *hse_dev)
{
	if (!hse_dev->quirks)
		return 1;

	return hse_dev->quirks->xor_copy_v2;
}


/* cq prep function */
int hse_cq_prep_copy(struct hse_device *hse_dev, struct hse_command_queue *cq,
		     dma_addr_t dst, dma_addr_t src, uint32_t size);
int hse_cq_prep_constant_fill(struct hse_device *hse_dev, struct hse_command_queue *cq,
			      dma_addr_t dst, uint32_t val, uint32_t size);
int hse_cq_prep_xor(struct hse_device *hse_dev, struct hse_command_queue *cq,
		    dma_addr_t dst, dma_addr_t *src, uint32_t src_cnt, uint32_t size);


int hse_setup_miscdevice(struct hse_device *hse_dev);
void hse_teardown_miscdevice(struct hse_device *hse_dev);

#ifdef CONFIG_RTK_HSE_DMA
int hse_setup_dmaengine(struct hse_device *hse_dev);
void hse_teardown_dmaengine(struct hse_device *hse_dev);
#else
static inline int hse_setup_dmaengine(struct hse_device *hse_dev)
{
	return 0;
}

static inline void hse_teardown_dmaengine(struct hse_device *hse_dev)
{
}
#endif /* CONFIG_RTK_HSE_DMA */

#endif /* __SOC_REALTEK_HSE_H__ */
