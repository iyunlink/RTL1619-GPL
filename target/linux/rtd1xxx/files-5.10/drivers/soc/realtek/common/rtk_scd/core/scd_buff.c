// SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause)
/*
 * Copyright (c) 2008 Kevin Wang <kevin_wang@realtek.com.tw>
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include "scd_buff.h"

MODULE_LICENSE("GPL");

void scb_queue_head_init(sc_buff_head *head)
{
	spin_lock_init(&head->lock);
	INIT_LIST_HEAD(&head->list);
	head->qlen = 0;
}

void scb_queue_head(sc_buff_head *head, sc_buff* scb)
{
	unsigned long flags;

	spin_lock_irqsave(&head->lock, flags);
	list_add(&scb->list,&head->list);
	head->qlen++;
	spin_unlock_irqrestore(&head->lock, flags);
}

void scb_queue_tail(sc_buff_head *head, sc_buff* scb)
{
	unsigned long flags;

	spin_lock_irqsave(&head->lock, flags);
	list_add_tail(&scb->list,&head->list);
	head->qlen++;
	spin_unlock_irqrestore(&head->lock, flags);    
}

sc_buff *scb_dequeue(sc_buff_head *head)
{
	sc_buff* scb = NULL;
	unsigned long flags;

	spin_lock_irqsave(&head->lock, flags);
        
	if (!list_empty(&head->list)) {
		scb = list_entry(head->list.next, sc_buff, list);        
		list_del_init(&scb->list);                    
		head->qlen--;
	}

	spin_unlock_irqrestore(&head->lock, flags);        

	return scb;
}

sc_buff *scb_dequeue_tail(sc_buff_head *head)
{
	sc_buff* scb = NULL;
	unsigned long flags;

	spin_lock_irqsave(&head->lock, flags);

	if (!list_empty(&head->list)) {
		scb = list_entry(head->list.prev, sc_buff, list);
		list_del_init(&scb->list);
		head->qlen--;
	}

	spin_unlock_irqrestore(&head->lock, flags);            

	return scb;
}

unsigned int scb_queue_len(const sc_buff_head *head)
{
	return head->qlen;
}

void scb_queue_purge(sc_buff_head *head)
{
	sc_buff *scb;
	unsigned long flags;

	spin_lock_irqsave(&head->lock, flags);

	while ((scb = scb_dequeue(head)) != NULL)
		kfree_scb(scb);

	spin_unlock_irqrestore(&head->lock, flags);                    
}

void init_scb(sc_buff* scb, unsigned char* p_buff, size_t size)
{
	if (scb) {
		INIT_LIST_HEAD(&scb->list);
		init_completion(&scb->complete);
		scb->status = 0;
		scb->flags  = 0;
		scb->head   = ((unsigned char*) scb) + sizeof(sc_buff);
		scb->data   = scb->head;
		scb->tail   = scb->head;
		scb->end    = scb->head + size;
		scb->len    = 0;
	}
}

sc_buff* alloc_scb(size_t size)
{
	sc_buff* scb = (sc_buff*)kmalloc(sizeof(sc_buff) + size, GFP_KERNEL);

	init_scb(scb, ((unsigned char*) scb) + sizeof(sc_buff), size);

	return scb;
}

void kfree_scb(sc_buff* scb)
{
	if (scb)
		kfree(scb);
}

#define CHECK_BOUNDRARY(scb) \
	do { \
		if (scb->tail > scb->end)  \
			printk(KERN_WARNING "scb over panic:  scb=%p scb->tail (%p) > scb->end (%p)\n", scb, scb->tail, scb->end);  \
		if (scb->data < scb->head) \
			printk(KERN_WARNING "scb under panic: scb=%p scb->data (%p) < scb->head (%p)\n", scb, scb->data, scb->head);  \
	} while(0)

int scb_tailroom(const sc_buff *scb)
{
	return scb->end - scb->tail;
}
 
// add data from tail of buffer
void scb_reserve(sc_buff* scb, unsigned int len)
{
	scb->data += len;
	scb->tail += len;
	CHECK_BOUNDRARY(scb);
}

// add data from tail of buffer
unsigned char* scb_put(sc_buff* scb, unsigned int len)
{
	unsigned char* ptr = scb->tail;

	scb->tail += len;
	scb->len  += len;
	CHECK_BOUNDRARY(scb);
	return ptr;
}

// add data from start of buffer
unsigned char* scb_push(sc_buff *scb, unsigned int len)
{
	scb->data -= len;
	scb->len  += len;
	CHECK_BOUNDRARY(scb);
	return scb->data;
}

// remove data from start of buffer
unsigned char* scb_pull(sc_buff *scb, unsigned int len)
{
	if (len <= scb->len) {
		scb->data += len;
		scb->len  -= len;
		return scb->data;
	}

	return NULL;    
}

void scb_trim(sc_buff *scb, unsigned int len)
{
	if (scb->len > len) {
		scb->len  = len;
		scb->tail = scb->data + len;
	}
}

void scb_purge(sc_buff* scb)
{
	scb->data = scb->head;
	scb->tail = scb->head;    
	scb->len  = 0;    
}

EXPORT_SYMBOL(scb_queue_head_init);
EXPORT_SYMBOL(scb_queue_head);
EXPORT_SYMBOL(scb_queue_tail);
EXPORT_SYMBOL(scb_dequeue);
EXPORT_SYMBOL(scb_dequeue_tail);
EXPORT_SYMBOL(scb_queue_purge);
EXPORT_SYMBOL(scb_queue_len);
EXPORT_SYMBOL(alloc_scb);
EXPORT_SYMBOL(kfree_scb);
EXPORT_SYMBOL(scb_reserve);
EXPORT_SYMBOL(scb_put);
EXPORT_SYMBOL(scb_push);
EXPORT_SYMBOL(scb_pull);
EXPORT_SYMBOL(scb_trim);
EXPORT_SYMBOL(scb_purge);
EXPORT_SYMBOL(scb_tailroom);
