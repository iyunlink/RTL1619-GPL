/*
 * Realtek RPC driver
 *
 * Copyright (c) 2017 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/ioctl.h>
#include <linux/sched.h>
#include <linux/freezer.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/kmemleak.h>
#include <soc/realtek/rtk_chip.h>

#include <trace/events/rtk_rpc.h>
#include "rtk_rpc.h"

#define TIMEOUT (5*HZ)

static struct radix_tree_root kernel_rpc_tree = RADIX_TREE_INIT(RPC, GFP_ATOMIC);
//static struct semaphore kernel_rpc_sem;
DECLARE_RWSEM(kernel_rpc_sem);

//static DECLARE_MUTEX(kernel_rpc_sem);
#ifdef CONFIG_REALTEK_RPC_HIFI
volatile RPC_HIFI_DEV *rpc_kern_hifi_devices;
#endif
#ifdef CONFIG_REALTEK_RPC_VE3
volatile RPC_DEV *rpc_kern_ve3_devices;
#endif
volatile RPC_DEV *rpc_kern_devices;
int rpc_kern_is_paused;
int rpc_kern_is_suspend;
#if defined(CONFIG_REALTEK_RPC_HIFI)
struct task_struct *rpc_kthread[(RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS + RPC_NR_KERN_HIFI_DEVS)/RPC_NR_PAIR] = {NULL};
static wait_queue_head_t rpc_wq[(RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS + RPC_NR_KERN_HIFI_DEVS)/RPC_NR_PAIR];
static uint32_t *rpc_retval[(RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS + RPC_NR_KERN_HIFI_DEVS)/RPC_NR_PAIR] = {NULL};
static int complete_condition[(RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS + RPC_NR_KERN_HIFI_DEVS)/RPC_NR_PAIR];
static struct mutex rpc_kern_lock[(RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS + RPC_NR_KERN_HIFI_DEVS)/RPC_NR_PAIR];
#elif defined(CONFIG_REALTEK_RPC_VE3)
struct task_struct *rpc_kthread[(RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS)/RPC_NR_PAIR] = {NULL};
static wait_queue_head_t rpc_wq[(RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS)/RPC_NR_PAIR];
static uint32_t *rpc_retval[(RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS)/RPC_NR_PAIR] = {NULL};
static int complete_condition[(RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS)/RPC_NR_PAIR];
static struct mutex rpc_kern_lock[(RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS)/RPC_NR_PAIR];
#else
struct task_struct *rpc_kthread[RPC_NR_KERN_DEVS/RPC_NR_PAIR] = {NULL};
static wait_queue_head_t rpc_wq[RPC_NR_KERN_DEVS/RPC_NR_PAIR];
static uint32_t *rpc_retval[RPC_NR_KERN_DEVS/RPC_NR_PAIR] = {NULL};
static int complete_condition[RPC_NR_KERN_DEVS/RPC_NR_PAIR];
static struct mutex rpc_kern_lock[RPC_NR_KERN_DEVS/RPC_NR_PAIR];
#endif
static int rpc_kernel_thread(void *p);

extern void rpc_send_interrupt(int type);

int rpc_kern_init(void)
{
	static int is_init;
	int result = 0, num;
	unsigned long i;

	is_init = 0;

	/* Create corresponding structures for each device. */
	rpc_kern_devices = (RPC_DEV *)AVCPU2SCPU(RPC_KERN_RECORD_ADDR);

	num = RPC_NR_KERN_DEVS;
	for (i = 0; i < num; i++) {
		pr_debug("rpc_kern_device %lu addr: %p\n", i, &rpc_kern_devices[i]);
		rpc_kern_devices[i].ringBuf = RPC_KERN_DEV_ADDR + i*RPC_RING_SIZE;

		/* Initialize pointers... */
		rpc_kern_devices[i].ringStart = rpc_kern_devices[i].ringBuf;
		rpc_kern_devices[i].ringEnd =
				rpc_kern_devices[i].ringBuf + RPC_RING_SIZE;
		rpc_kern_devices[i].ringIn = rpc_kern_devices[i].ringBuf;
		rpc_kern_devices[i].ringOut = rpc_kern_devices[i].ringBuf;

		pr_debug("The %luth kern dev:\n", i);
		pr_debug("RPC ringStart: %p\n",
				AVCPU2SCPU(rpc_kern_devices[i].ringStart));
		pr_debug("RPC ringEnd:   %p\n",
				AVCPU2SCPU(rpc_kern_devices[i].ringEnd));
		pr_debug("RPC ringIn:	%p\n",
				AVCPU2SCPU(rpc_kern_devices[i].ringIn));
		pr_debug("RPC ringOut:   %p\n",
				AVCPU2SCPU(rpc_kern_devices[i].ringOut));
		pr_debug("\n");

		if (!is_init) {
			rpc_kern_devices[i].ptrSync =
					kmalloc(sizeof(RPC_SYNC_Struct), GFP_KERNEL);
			kmemleak_not_leak(rpc_kern_devices[i].ptrSync);

			/* Initialize wait queue... */
			init_waitqueue_head(&(rpc_kern_devices[i].ptrSync->waitQueue));

			/* Initialize sempahores... */
			init_rwsem(&rpc_kern_devices[i].ptrSync->readSem);
			init_rwsem(&rpc_kern_devices[i].ptrSync->writeSem);
		}

		if (i%RPC_NR_PAIR == 1) {
			if (rpc_kthread[i/RPC_NR_PAIR] == NULL)
				rpc_kthread[i/RPC_NR_PAIR] =
						kthread_run(rpc_kernel_thread,
								(void *)i, "rpc-%lu", i);
		}
	}
#ifdef CONFIG_REALTEK_RPC_VE3
	/* Create corresponding structures for each device. */
	rpc_kern_ve3_devices = (RPC_DEV *)AVCPU2SCPU(RPC_KERN_VE3_RECORD_ADDR);

	num = RPC_NR_KERN_VE3_DEVS;
	for (i = 0; i < num; i++) {
		pr_debug("rpc_kern_device %lu addr: %p\n", i, &rpc_kern_ve3_devices[i]);
		rpc_kern_ve3_devices[i].ringBuf = RPC_KERN_VE3_DEV_ADDR + i*RPC_RING_SIZE;

		/* Initialize pointers... */
		rpc_kern_ve3_devices[i].ringStart = rpc_kern_ve3_devices[i].ringBuf;
		rpc_kern_ve3_devices[i].ringEnd =
				rpc_kern_ve3_devices[i].ringBuf + RPC_RING_SIZE;
		rpc_kern_ve3_devices[i].ringIn = rpc_kern_ve3_devices[i].ringBuf;
		rpc_kern_ve3_devices[i].ringOut = rpc_kern_ve3_devices[i].ringBuf;

		pr_debug("The %luth kern dev:\n", i + RPC_NR_KERN_DEVS);
		pr_debug("RPC ringStart: %p\n",
			AVCPU2SCPU(rpc_kern_ve3_devices[i].ringStart));
		pr_debug("RPC ringEnd:   %p\n",
			AVCPU2SCPU(rpc_kern_ve3_devices[i].ringEnd));
		pr_debug("RPC ringIn:	%p\n",
			AVCPU2SCPU(rpc_kern_ve3_devices[i].ringIn));
		pr_debug("RPC ringOut:   %p\n",
			AVCPU2SCPU(rpc_kern_ve3_devices[i].ringOut));
		pr_debug("\n");

		if (!is_init) {
			rpc_kern_ve3_devices[i].ptrSync =
					kmalloc(sizeof(RPC_SYNC_Struct), GFP_KERNEL);
			kmemleak_not_leak(rpc_kern_ve3_devices[i].ptrSync);

			/* Initialize wait queue... */
			init_waitqueue_head(&(rpc_kern_ve3_devices[i].ptrSync->waitQueue));

			/* Initialize sempahores... */
			init_rwsem(&rpc_kern_ve3_devices[i].ptrSync->readSem);
			init_rwsem(&rpc_kern_ve3_devices[i].ptrSync->writeSem);
		}

		if (i%RPC_NR_PAIR == 1) {
			if (rpc_kthread[(i + RPC_NR_KERN_DEVS)/RPC_NR_PAIR] == NULL)
				rpc_kthread[(i + RPC_NR_KERN_DEVS)/RPC_NR_PAIR] =
						kthread_run(rpc_kernel_thread,
						(void *)((i + RPC_NR_KERN_DEVS)/RPC_NR_PAIR), "rpc-%lu", (i + RPC_NR_KERN_DEVS)/RPC_NR_PAIR);
		}
	}

#endif
#ifdef CONFIG_REALTEK_RPC_HIFI

	/* Create corresponding structures for each device. */
	rpc_kern_hifi_devices = (RPC_HIFI_DEV *)AVCPU2SCPU(RPC_KERN_HIFI_RECORD_ADDR);

	num = RPC_NR_KERN_HIFI_DEVS;
	for (i = 0; i < num; i++) {
		pr_debug("rpc_kern_device %lu addr: %p\n", i, &rpc_kern_hifi_devices[i]);
		rpc_kern_hifi_devices[i].ringBuf = RPC_KERN_HIFI_DEV_ADDR + i*RPC_RING_SIZE;

		/* Initialize pointers... */
		rpc_kern_hifi_devices[i].ringStart = rpc_kern_hifi_devices[i].ringBuf;
		rpc_kern_hifi_devices[i].ringEnd =
									rpc_kern_hifi_devices[i].ringBuf + RPC_RING_SIZE;
		rpc_kern_hifi_devices[i].ringIn = rpc_kern_hifi_devices[i].ringBuf;
		rpc_kern_hifi_devices[i].ringOut = rpc_kern_hifi_devices[i].ringBuf;

		pr_debug("The %luth kern dev:\n", i + RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS);
		pr_debug("RPC ringStart: %p\n",
									AVCPU2SCPU(rpc_kern_hifi_devices[i].ringStart));
		pr_debug("RPC ringEnd:	 %p\n",
									AVCPU2SCPU(rpc_kern_hifi_devices[i].ringEnd));
		pr_debug("RPC ringIn:	%p\n",
									AVCPU2SCPU(rpc_kern_hifi_devices[i].ringIn));
		pr_debug("RPC ringOut:	 %p\n",
									AVCPU2SCPU(rpc_kern_hifi_devices[i].ringOut));
		pr_debug("\n");

		if (!is_init) {
			rpc_kern_hifi_devices[i].ptrSync =
										kmalloc(sizeof(RPC_SYNC_Struct), GFP_KERNEL);
			kmemleak_not_leak(rpc_kern_hifi_devices[i].ptrSync);

			/* Initialize wait queue... */
			init_waitqueue_head(&(rpc_kern_hifi_devices[i].ptrSync->waitQueue));

			/* Initialize sempahores... */
			init_rwsem(&rpc_kern_hifi_devices[i].ptrSync->readSem);
			init_rwsem(&rpc_kern_hifi_devices[i].ptrSync->writeSem);
		}

		if (i%RPC_NR_PAIR == 1) {
			if (rpc_kthread[(i + RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS)/RPC_NR_PAIR] == NULL)
				rpc_kthread[(i + RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS)/RPC_NR_PAIR] =
				kthread_run(rpc_kernel_thread,
					(void *)(i + RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS), "rpc-%lu", i + RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS);
		}
	}
#endif


	if (!is_init) {
		for (i = 0; i < RPC_NR_KERN_DEVS/RPC_NR_PAIR; i++) {
			init_waitqueue_head(&(rpc_wq[i]));
			mutex_init(&rpc_kern_lock[i]);
		}
#ifdef CONFIG_REALTEK_RPC_VE3
		for (i = RPC_NR_KERN_DEVS/RPC_NR_PAIR; i < (RPC_NR_KERN_DEVS+RPC_NR_KERN_VE3_DEVS)/RPC_NR_PAIR; i++) {
			init_waitqueue_head(&(rpc_wq[i]));
			mutex_init(&rpc_kern_lock[i]);
		}
#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
		for (i = (RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS)/RPC_NR_PAIR; i < (RPC_NR_KERN_DEVS + RPC_NR_KERN_VE3_DEVS + RPC_NR_KERN_HIFI_DEVS)/RPC_NR_PAIR; i++) {
			init_waitqueue_head(&(rpc_wq[i]));
			mutex_init(&rpc_kern_lock[i]);
		}
#endif

	}
	is_init = 1;
	rpc_kern_is_paused = 0;
	rpc_kern_is_suspend = 0;

	return result;
}

int rpc_kern_pause(void)
{
	rpc_kern_is_paused = 1;
	return 0;
}

int rpc_kern_suspend(void)
{
	rpc_kern_is_suspend = 1;
	return 0;
}

int rpc_kern_resume(void)
{
	rpc_kern_is_suspend = 0;
	return 0;
}

ssize_t rpc_kern_read(int opt, char *buf, size_t count)
{
	RPC_DEV *dev;
	int temp, size;
	size_t r;
	ssize_t ret = 0;
	uint32_t ptmp;
	volatile uint32_t *ringStart, *ringEnd, *ringIn, *ringOut;
	RPC_SYNC_Struct *ptrSync;

#ifdef CONFIG_REALTEK_RPC_HIFI
	RPC_HIFI_DEV *hifi_dev;
	if (opt == RPC_HIFI) {
		hifi_dev = (RPC_HIFI_DEV *)&rpc_kern_hifi_devices[1];
		ringStart = &hifi_dev->ringStart;
		ringEnd = &hifi_dev->ringEnd;
		ringIn = &hifi_dev->ringIn;
		ringOut = &hifi_dev->ringOut;
		ptrSync = hifi_dev->ptrSync;
	}
#endif


#ifdef CONFIG_REALTEK_RPC_VE3
	if (opt == RPC_VE3) {
		dev = (RPC_DEV *)&rpc_kern_ve3_devices[1];
		ringStart = &dev->ringStart;
		ringEnd = &dev->ringEnd;
		ringIn = &dev->ringIn;
		ringOut = &dev->ringOut;
		ptrSync = dev->ptrSync;
	}
#endif
	if (opt == RPC_AUDIO || (opt == RPC_VIDEO)) {
		dev = (RPC_DEV *)&rpc_kern_devices[opt*RPC_NR_PAIR+1];
		ringStart = &dev->ringStart;
		ringEnd = &dev->ringEnd;
		ringIn = &dev->ringIn;
		ringOut = &dev->ringOut;
		ptrSync = dev->ptrSync;
	}

	pr_debug("read rpc_kern_device: %p\n", dev);
	down_write(&ptrSync->readSem);

	if (*ringIn == *ringOut)
		goto out;   // the ring is empty...
	else if (*ringIn > *ringOut)
		size = *ringIn - *ringOut;
	else
		size = RPC_RING_SIZE + *ringIn - *ringOut;

	if (count > size)
		count = size;

	temp = *ringEnd - *ringOut;
	if (temp >= count) {
#ifdef MY_COPY
		r = my_copy_user((int *)buf,
				(int *)AVCPU2SCPU(*ringOut), count);
#else
		r = ((int *)buf !=
				memcpy((int *)buf, (int *)AVCPU2SCPU(*ringOut), count));
#endif /* MY_COPY */
		if (r) {
			ret = -EFAULT;
			goto out;
		}
		ret += count;
		ptmp = *ringOut + ((count+3) & 0xfffffffc);
		if (ptmp == *ringEnd)
			*ringOut = *ringStart;
		else
			*ringOut = ptmp;

		pr_debug("RPC Read is in 1st kind...\n");
	} else {
#ifdef MY_COPY
		r = my_copy_user((int *)buf, (int *)AVCPU2SCPU(*ringOut), temp);
#else
		r = ((int *)buf !=
				memcpy((int *)buf, (int *)AVCPU2SCPU(*ringOut), temp));
#endif /* MY_COPY */
		if (r) {
			ret = -EFAULT;
			goto out;
		}
		count -= temp;

#ifdef MY_COPY
		r = my_copy_user((int *)(buf+temp),
				(int *)AVCPU2SCPU(*ringStart), count);
#else
		r = ((int *)(buf+temp) !=
				memcpy((int *)(buf+temp),
						(int *)AVCPU2SCPU(*ringStart), count));
#endif /* MY_COPY */
		if (r) {
			ret = -EFAULT;
			goto out;
		}
		ret += (temp + count);
		*ringOut = *ringStart+((count+3) & 0xfffffffc);

		pr_debug("RPC Read is in 2nd kind...\n");
	}
out:
	pr_debug("RPC kern ringOut pointer is : %p\n", AVCPU2SCPU(*ringOut));
	up_write(&ptrSync->readSem);
	return ret;
}

ssize_t rpc_kern_write(int opt, const char *buf, size_t count)
{
	RPC_DEV *dev;
	int temp, size;
	size_t r;
	ssize_t ret = 0;
	uint32_t ptmp;
	volatile uint32_t *ringStart, *ringEnd, *ringIn, *ringOut;
	RPC_SYNC_Struct *ptrSync;

#ifdef CONFIG_REALTEK_RPC_HIFI
	RPC_HIFI_DEV *hifi_dev;
	if (opt == RPC_HIFI) {
		hifi_dev = (RPC_HIFI_DEV *)&rpc_kern_hifi_devices[0];
		ringStart = &hifi_dev->ringStart;
		ringEnd = &hifi_dev->ringEnd;
		ringIn = &hifi_dev->ringIn;
		ringOut = &hifi_dev->ringOut;
		ptrSync = hifi_dev->ptrSync;
	}
#endif
#ifdef CONFIG_REALTEK_RPC_VE3
	if (opt == RPC_VE3) {
		dev = (RPC_DEV *)&rpc_kern_ve3_devices[0];
		ringStart = &dev->ringStart;
		ringEnd = &dev->ringEnd;
		ringIn = &dev->ringIn;
		ringOut = &dev->ringOut;
		ptrSync = dev->ptrSync;
	}
#endif
	if (opt == RPC_AUDIO || (opt == RPC_VIDEO)) {
		dev = (RPC_DEV *)&rpc_kern_devices[opt*RPC_NR_PAIR];
		ringStart = &dev->ringStart;
		ringEnd = &dev->ringEnd;
		ringIn = &dev->ringIn;
		ringOut = &dev->ringOut;
		ptrSync = dev->ptrSync;
	}

	down_write(&ptrSync->writeSem);

	if (*ringIn == *ringOut)
		size = 0;   // the ring is empty
	else if (*ringIn > *ringOut)
		size = *ringIn - *ringOut;
	else
		size = RPC_RING_SIZE + *ringIn - *ringOut;

	if (count > (RPC_RING_SIZE - size - 1))
		goto out;

	temp = *ringEnd - *ringIn;
	if (temp >= count) {
#ifdef MY_COPY
		r = my_copy_user((int *)AVCPU2SCPU(*ringIn), (int *)buf, count);
#else
		r = ((int *)AVCPU2SCPU(*ringIn) !=
				memcpy((int *)AVCPU2SCPU(*ringIn), (int *)buf, count));
#endif /* MY_COPY */
		if (r) {
			ret = -EFAULT;
			goto out;
		}
		ret += count;
		ptmp = *ringIn + ((count+3) & 0xfffffffc);

		//asm("DSB");
		mb();

		if (ptmp == *ringEnd)
			*ringIn = *ringStart;
		else
			*ringIn = ptmp;

		pr_debug("RPC Write is in 1st kind...\n");
	} else {
#ifdef MY_COPY
		r = my_copy_user((int *)AVCPU2SCPU(*ringIn), (int *)buf, temp);
#else
		r = ((int *)AVCPU2SCPU(*ringIn) !=
				memcpy((int *)AVCPU2SCPU(*ringIn), (int *)buf, temp));
#endif
		if (r) {
			ret = -EFAULT;
			goto out;
		}
		count -= temp;

#ifdef MY_COPY
		r = my_copy_user((int *)AVCPU2SCPU(*ringStart),
				(int *)(buf+temp), count);
#else
		r = ((int *)AVCPU2SCPU(*ringStart) != memcpy((int *)AVCPU2SCPU(*ringStart), (int *)(buf+temp), count));
#endif
		if (r) {
			ret = -EFAULT;
			goto out;
		}
		ret += (temp + count);

		//asm("DSB");
		mb();

		*ringIn = *ringStart+((count+3) & 0xfffffffc);

		pr_debug("RPC Write is in 2nd kind...\n");
	}

	//if (opt == RPC_AUDIO)
	if (opt == RPC_AUDIO) {
		rpc_send_interrupt(RPC_AUDIO);
	} else if (opt == RPC_VIDEO) {
		rpc_send_interrupt(RPC_VIDEO);
#ifdef CONFIG_REALTEK_RPC_VE3
	} else if (opt == RPC_VE3) {
		rpc_send_interrupt(RPC_VE3);
#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
		} else if (opt == RPC_HIFI) {
			rpc_send_interrupt(RPC_HIFI);
#endif

	} else {
		pr_err("error device number...\n");
	}
out:
	pr_debug("RPC kern ringIn pointer is : %p\n", AVCPU2SCPU(*ringIn));
	up_write(&ptrSync->writeSem);
	return ret;
}

#if 0
int register_kernel_rpc(unsigned long command, FUNC_PTR ptr)
{
	int error;

	error = radix_tree_preload(GFP_KERNEL);
	if (error == 0) {
		down_write(&kernel_rpc_sem);
		error = radix_tree_insert(&kernel_rpc_tree, command, (void *)ptr);
		if (error)
			pr_err("RPC: register kernel rpc %ld error...\n", command);
		up_write(&kernel_rpc_sem);
		radix_tree_preload_end();
	}

	return error;
}
#endif

uint32_t handle_command(uint32_t command, uint32_t param1, uint32_t param2)
{
	FUNC_PTR ptr;
	int ret = 0;

	pr_info("Handle command %x, param1: %x, param2: %x...\n",
			command, param1, param2);
	down_write(&kernel_rpc_sem);
	ptr = radix_tree_lookup(&kernel_rpc_tree, command);
	if (ptr)
		ret = ptr(param1, param2);
	else
		pr_err("RPC: lookup kernel rpc %d error...\n", command);
	up_write(&kernel_rpc_sem);

	return ret;
}

static int rpc_kernel_thread(void *p)
{
	char readbuf[sizeof(RPC_STRUCT) + 3*sizeof(uint32_t)];
	RPC_DEV *dev;
	RPC_STRUCT *rpc;
	uint32_t *tmp;
	unsigned long idx = (unsigned long)p;
	unsigned int opt = idx / RPC_NR_PAIR;
	volatile uint32_t *ringStart, *ringEnd, *ringIn, *ringOut;
	RPC_SYNC_Struct *ptrSync;

#ifdef CONFIG_REALTEK_RPC_HIFI
	RPC_HIFI_DEV *hifi_dev;
	if (idx == 7) {
		hifi_dev = (RPC_HIFI_DEV *)&rpc_kern_hifi_devices[1];
		ringStart = &hifi_dev->ringStart;
		ringEnd = &hifi_dev->ringEnd;
		ringIn = &hifi_dev->ringIn;
		ringOut = &hifi_dev->ringOut;
		ptrSync = hifi_dev->ptrSync;
	}
#endif
#ifdef CONFIG_REALTEK_RPC_VE3
	if (idx == 5) {
		dev = (RPC_DEV *)&rpc_kern_ve3_devices[1];
		ringStart = &dev->ringStart;
		ringEnd = &dev->ringEnd;
		ringIn = &dev->ringIn;
		ringOut = &dev->ringOut;
		ptrSync = dev->ptrSync;
	}
#endif
	if (idx < 4) {
		dev = (RPC_DEV *)&rpc_kern_devices[idx];
		ringStart = &dev->ringStart;
		ringEnd = &dev->ringEnd;
		ringIn = &dev->ringIn;
		ringOut = &dev->ringOut;
		ptrSync = dev->ptrSync;
	}
	while (1) {
		//if (current->flags & PF_FREEZE)
		//refrigerator(PF_FREEZE);
		//try_to_freeze();

		//pr_info(" #@# wait %s %x %x \n", current->comm, dev, dev->waitQueue);
		if (wait_event_interruptible(ptrSync->waitQueue, *ringIn !=
				*ringOut || kthread_should_stop())) {
			pr_notice("%s got signal or should stop...\n", current->comm);
			continue;
		}
		//pr_info(" #@# wakeup %s \n", current->comm);

		if (kthread_should_stop()) {
			pr_notice("%s exit...\n", current->comm);
			break;
		}

		/* read the reply data... */
		if (rpc_kern_read(opt, readbuf, sizeof(RPC_STRUCT)) !=
				sizeof(RPC_STRUCT)) {
			pr_err("ERROR in read opt(%d) kernel RPC...\n", opt);
			continue;
		}

		rpc = (RPC_STRUCT *)readbuf;
		tmp = (uint32_t *)(readbuf + sizeof(RPC_STRUCT));
		if (rpc->taskID) {
			/* handle the request... */
			char replybuf[sizeof(RPC_STRUCT) + 2*sizeof(uint32_t)];
			uint32_t ret;
			RPC_STRUCT *rrpc = (RPC_STRUCT *)replybuf;

			/* read the payload... */
			if (rpc_kern_read(opt, readbuf + sizeof(RPC_STRUCT),
					3*sizeof(uint32_t)) != 3*sizeof(uint32_t)) {
				pr_err("ERROR in read payload...\n");
				continue;
			}
#ifdef CONFIG_REALTEK_RPC_HIFI
			if (opt == RPC_HIFI) {
				ret = handle_command(*tmp,
								*(tmp+1), *(tmp+2));
				/* fill the RPC_STRUCT... */
				rrpc->programID = REPLYID;
				rrpc->versionID = REPLYID;
				rrpc->procedureID = 0;
				rrpc->taskID = 0;
#ifdef RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID
				rrpc->sysTID = 0;
#endif /* RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID */
				rrpc->sysPID = 0;
				rrpc->parameterSize = 2*sizeof(uint32_t);
				rrpc->mycontext = rpc->mycontext;

				/* fill the parameters... */
				tmp = (uint32_t *)(replybuf + sizeof(RPC_STRUCT));
				*(tmp+0) = rpc->taskID; /* FIXME: should be 64bit */
				*(tmp+1) = ret;
			} else
#endif
			{
				ret = handle_command(ntohl(*tmp),
							ntohl(*(tmp+1)), ntohl(*(tmp+2)));

				/* fill the RPC_STRUCT... */
				rrpc->programID = htonl(REPLYID);
				rrpc->versionID = htonl(REPLYID);
				rrpc->procedureID = 0;
				rrpc->taskID = 0;
#ifdef RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID
				rrpc->sysTID = 0;
#endif /* RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID */
				rrpc->sysPID = 0;
				rrpc->parameterSize = htonl(2*sizeof(uint32_t));
				rrpc->mycontext = rpc->mycontext;

				/* fill the parameters... */
				tmp = (uint32_t *)(replybuf + sizeof(RPC_STRUCT));
				*(tmp+0) = rpc->taskID; /* FIXME: should be 64bit */
				*(tmp+1) = htonl(ret);
			}

			if (rpc_kern_write(opt, replybuf, sizeof(replybuf)) !=
					sizeof(replybuf)) {
				pr_err("ERROR in send kernel RPC...\n");
				return RPC_FAIL;
			}
		} else {
			/* read the payload... */
			if (rpc_kern_read(opt, readbuf+sizeof(RPC_STRUCT),
					2*sizeof(uint32_t)) != 2*sizeof(uint32_t)) {
				pr_err("ERROR in read payload...\n");
				continue;
			}

			/* parse the reply data... */
#if 0
			/* FIXME: mycontext should be 64bit */
			*((uint32_t *)ntohl(rpc->mycontext)) = ntohl(*(tmp+1));
#else
#ifdef CONFIG_REALTEK_RPC_HIFI
			if (opt == RPC_HIFI)
				*rpc_retval[opt] = *(tmp+1);
			else
				*rpc_retval[opt] = ntohl(*(tmp+1));
#else
			*rpc_retval[opt] = ntohl(*(tmp+1));
#endif
#endif
			//pr_info("tmp %x opt %d\n", ntohl(*tmp), opt);
			complete_condition[opt] = 1;
#if 0
			/* FIXME: wait_queue_head_t * should be 64bit */
			//wake_up((wait_queue_head_t *)ntohl(*tmp)); /* ack the sync... */
#else
			wake_up(&rpc_wq[opt]); // ack the sync...
#endif
		}
	}

	return 0;
}

int dump_kern_rpc(void)
{
	int i, j;
	RPC_DEV *dev;
#ifdef CONFIG_REALTEK_RPC_HIFI
	RPC_HIFI_DEV *hifi_dev;
#endif

	for(j = 0; j < RPC_NR_KERN_DEVS; j++){
		dev = (RPC_DEV *)&rpc_kern_devices[j];
		pr_info("\nname: %sKern%s\n", (j<2) ? "Audio" : "Video", (j % RPC_NR_PAIR == 0) ? "Write" : "Read");
		pr_info("RingBuf: %x\n", dev->ringBuf);
		pr_info("RingStart: %x\n", dev->ringStart);
		pr_info("RingIn: %x\n", dev->ringIn);
		pr_info("RingOut: %x\n", dev->ringOut);
		pr_info("RingEnd: %x\n", dev->ringEnd);

		pr_info("RingBuffer:\n");
		for (i = 0; i < RPC_RING_SIZE; i += 16) {
			uint32_t *addr = (uint32_t *)(AVCPU2SCPU(dev->ringStart) + i);
			pr_info("%x: %08x %08x %08x %08x\n",
				dev->ringStart + i,
				ntohl(*(addr + 0)),
				ntohl(*(addr + 1)),
				ntohl(*(addr + 2)),
				ntohl(*(addr + 3)));
		}
	}

	for(j = 0; j < RPC_INTR_DEV_TOTAL; j++){
		dev = (RPC_DEV *)&rpc_intr_devices[j];
		pr_info("\nname: %sIntr%s\n", (j<2) ? "Audio" : "Video", (j % RPC_NR_PAIR == 0) ? "Write" : "Read");
		pr_info("RingBuf: %x\n", dev->ringBuf);
		pr_info("RingStart: %x\n", dev->ringStart);
		pr_info("RingIn: %x\n", dev->ringIn);
		pr_info("RingOut: %x\n", dev->ringOut);
		pr_info("RingEnd: %x\n", dev->ringEnd);

		pr_info("RingBuffer:\n");
		for (i = 0; i < RPC_RING_SIZE; i += 16) {
			uint32_t *addr = (uint32_t *)(AVCPU2SCPU(dev->ringStart) + i);
			pr_info("%x: %08x %08x %08x %08x\n",
				dev->ringStart + i,
				ntohl(*(addr + 0)),
				ntohl(*(addr + 1)),
				ntohl(*(addr + 2)),
				ntohl(*(addr + 3)));
		}
	}
	if ((get_rtd_chip_id() != CHIP_ID_RTD1619B) && (get_rtd_chip_id() != CHIP_ID_RTD1319D))
		return 0;

#ifdef CONFIG_REALTEK_RPC_VE3
	for(j = 0; j < RPC_NR_KERN_VE3_DEVS; j++){
		dev = (RPC_DEV *)&rpc_kern_ve3_devices[j];
		pr_info("\nname: %sKern%s\n","VE3", (j % RPC_NR_PAIR == 0) ? "Write" : "Read");
		pr_info("RingBuf: %x\n", dev->ringBuf);
		pr_info("RingStart: %x\n", dev->ringStart);
		pr_info("RingIn: %x\n", dev->ringIn);
		pr_info("RingOut: %x\n", dev->ringOut);
		pr_info("RingEnd: %x\n", dev->ringEnd);

		pr_info("RingBuffer:\n");
		for (i = 0; i < RPC_RING_SIZE; i += 16) {
			uint32_t *addr = (uint32_t *)(AVCPU2SCPU(dev->ringStart) + i);
			pr_info("%x: %08x %08x %08x %08x\n",
				dev->ringStart + i,
				ntohl(*(addr + 0)),
				ntohl(*(addr + 1)),
				ntohl(*(addr + 2)),
				ntohl(*(addr + 3)));
		}
	}

	for(j = 0; j < RPC_INTR_VE3_DEV_TOTAL; j++){
        dev = (RPC_DEV *)&rpc_intr_ve3_devices[j];
		pr_info("\nname: %sIntr%s\n", "VE3", (j % RPC_NR_PAIR == 0) ? "Write" : "Read");
		pr_info("RingBuf: %x\n", dev->ringBuf);
		pr_info("RingStart: %x\n", dev->ringStart);
		pr_info("RingIn: %x\n", dev->ringIn);
		pr_info("RingOut: %x\n", dev->ringOut);
		pr_info("RingEnd: %x\n", dev->ringEnd);

		pr_info("RingBuffer:\n");
		for (i = 0; i < RPC_RING_SIZE; i += 16) {
			uint32_t *addr = (uint32_t *)(AVCPU2SCPU(dev->ringStart) + i);
			pr_info("%x: %08x %08x %08x %08x\n",
				dev->ringStart + i,
				ntohl(*(addr + 0)),
				ntohl(*(addr + 1)),
				ntohl(*(addr + 2)),
				ntohl(*(addr + 3)));
		}
	}
#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
	for(j = 0; j < RPC_NR_KERN_HIFI_DEVS; j++){
		hifi_dev = (RPC_HIFI_DEV *)&rpc_kern_hifi_devices[j];
		pr_info("\nname: %sKern%s\n","HIFI", (j % RPC_NR_PAIR == 0) ? "Write" : "Read");
		pr_info("RingBuf: %x\n", hifi_dev->ringBuf);
		pr_info("RingStart: %x\n", hifi_dev->ringStart);
		pr_info("RingIn: %x\n", hifi_dev->ringIn);
		pr_info("RingOut: %x\n", hifi_dev->ringOut);
		pr_info("RingEnd: %x\n", hifi_dev->ringEnd);

		pr_info("RingBuffer:\n");
		for (i = 0; i < RPC_RING_SIZE; i += 16) {
			uint32_t *addr = (uint32_t *)(AVCPU2SCPU(hifi_dev->ringStart) + i);
			pr_info("%x: %08x %08x %08x %08x\n",
				hifi_dev->ringStart + i,
				*(addr + 0),
				*(addr + 1),
				*(addr + 2),
				*(addr + 3));
		}
	}

	for(j = 0; j < RPC_INTR_HIFI_DEV_TOTAL; j++){
        hifi_dev = (RPC_HIFI_DEV *)&rpc_intr_hifi_devices[j];
		pr_info("\nname: %sIntr%s\n", "HIFI", (j % RPC_NR_PAIR == 0) ? "Write" : "Read");
		pr_info("RingBuf: %x\n", hifi_dev->ringBuf);
		pr_info("RingStart: %x\n", hifi_dev->ringStart);
		pr_info("RingIn: %x\n", hifi_dev->ringIn);
		pr_info("RingOut: %x\n", hifi_dev->ringOut);
		pr_info("RingEnd: %x\n", hifi_dev->ringEnd);

		pr_info("RingBuffer:\n");
		for (i = 0; i < RPC_RING_SIZE; i += 16) {
			uint32_t *addr = (uint32_t *)(AVCPU2SCPU(hifi_dev->ringStart) + i);
			pr_info("%x: %08x %08x %08x %08x\n",
				hifi_dev->ringStart + i,
				*(addr + 0),
				*(addr + 1),
				*(addr + 2),
				*(addr + 3));
		}
	}
#endif

	return 0;
}

int send_rpc_command(int opt, uint32_t command, uint32_t param1,
		uint32_t param2, uint32_t *retvalue)
{
	char sendbuf[sizeof(RPC_STRUCT) + 3*sizeof(uint32_t)];
	RPC_STRUCT *rpc = (RPC_STRUCT *)sendbuf;
	uint32_t *tmp;

	if (rpc_kern_is_paused) {
		pr_warn("RPCkern: someone access rpc kern during the pause...\n");
		return RPC_FAIL;
	}

	mutex_lock(&rpc_kern_lock[opt]);

	while (rpc_kern_is_suspend) {
		pr_warn("RPCkern: someone access rpc poll during the suspend!!!...\n");
		msleep(1000);
	}

	if (rpc_kthread[opt] == 0) {
		pr_warn("RPCkern: %s is disabled...\n", rpc_kthread[opt]->comm);
		mutex_unlock(&rpc_kern_lock[opt]);
		return RPC_FAIL;
	}

	//pr_info(" #@# sendbuf: %d cmd %x param1 %x param2 %x\n",
	//		sizeof(sendbuf), command, param1, param2);

	/* fill the RPC_STRUCT... */
#ifdef CONFIG_REALTEK_RPC_HIFI
	if (opt == RPC_HIFI) {
		rpc->programID = KERNELID;
		rpc->versionID = KERNELID;
		rpc->procedureID = 0;
		//rpc->taskID = 0;
		rpc->taskID = current->pid;// 0;

#ifdef RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID
		rpc->sysTID = 0;
#endif /* RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID */
		rpc->sysPID = 0;
		rpc->parameterSize = 3*sizeof(uint32_t);

		rpc->mycontext = 0;
		rpc_retval[opt] = retvalue;

		/* fill the parameters... */
		tmp = (uint32_t *)(sendbuf+sizeof(RPC_STRUCT));
		//pr_info(" aaa: %x bbb: %x \n", sendbuf, tmp);
		*tmp = command;
		*(tmp+1) = param1;
		*(tmp+2) = param2;
		trace_rtk_rpc_peek_rpc_request((struct rpc_struct_tp *)rpc,
			(u32)refclk_get_counter(refclk), 0, rpc->taskID, false);
	} else
#endif
	{
		rpc->programID = htonl(KERNELID);
		rpc->versionID = htonl(KERNELID);
			rpc->procedureID = 0;
		//rpc->taskID = 0;
		rpc->taskID = htonl(current->pid);// 0;

#ifdef RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID
		rpc->sysTID = 0;
#endif /* RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID */
		rpc->sysPID = 0;
		rpc->parameterSize = htonl(3*sizeof(uint32_t));
		rpc->mycontext = 0;
		rpc_retval[opt] = retvalue;

		/* fill the parameters... */
			tmp = (uint32_t *)(sendbuf+sizeof(RPC_STRUCT));
			//pr_info(" aaa: %x bbb: %x \n", sendbuf, tmp);
		*tmp = htonl(command);
		*(tmp+1) = htonl(param1);
		*(tmp+2) = htonl(param2);
		trace_rtk_rpc_peek_rpc_request((struct rpc_struct_tp *)rpc,
			(u32)refclk_get_counter(refclk), 0, rpc->taskID, true);
	}


	complete_condition[opt] = 0;
	if (rpc_kern_write(opt, sendbuf, sizeof(sendbuf)) != sizeof(sendbuf)) {
		pr_err("ERROR in send kernel RPC...\n");
		mutex_unlock(&rpc_kern_lock[opt]);
		return RPC_FAIL;
	}
	/* wait the result... */
	//if (!sleep_on_timeout(&rpc_wq[opt], TIMEOUT)) {
	if (!wait_event_timeout(rpc_wq[opt], complete_condition[opt], TIMEOUT)) {
		pr_err("kernel rpc timeout -> disable %s...\n", rpc_kthread[opt]->comm);
		WARN(1, " #@# sendbuf: size%lu cmd:%x param1:%x param2:%x\n",
				sizeof(sendbuf), command, param1, param2);
		dump_kern_rpc();

		kthread_stop(rpc_kthread[opt]);
		rpc_kthread[opt] = 0;
		mutex_unlock(&rpc_kern_lock[opt]);
		return RPC_FAIL;
	} else {
		pr_debug(" #@# ret: %d \n", *retvalue);
		mutex_unlock(&rpc_kern_lock[opt]);
		return RPC_OK;
	}
}
EXPORT_SYMBOL(send_rpc_command);

MODULE_LICENSE("GPL v2");
