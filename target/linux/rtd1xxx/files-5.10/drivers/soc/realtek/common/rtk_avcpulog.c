// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Realtek Semiconductor Corp.
 */

#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <soc/realtek/rtk_ipc_shm.h>
#include <soc/realtek/rtk_sb2_sem.h>

struct log_device_info_t {
	char name[16];
	unsigned int offset;
};

static const struct log_device_info_t log_device_info_list[] = {
	{"alog", offsetof(struct rtk_ipc_shm, printk_buffer)},
	{"vlog", offsetof(struct rtk_ipc_shm, video_printk_buffer)},
};

#define MODULE_NAME		"avlog"
#define MODULE_NUM		(ARRAY_SIZE(log_device_info_list))
#define HWSEM_COMPAT_NAME	"realtek,sb2-sem-avlog"
#define HWSEM_DEF_TIMEOUT	500 /* msec */
#define WORK_DEF_DELAY_TIME	500 /* msec */

enum LOG_STAT {
	L_NOTAVAIL = 0,
	L_PENDING,
	L_RUNNING,
	L_WAITING,
	L_EXIT,
	L_ERR,
};

struct log_device_t {
	/* static vars */
	const char *name;
	struct device *device;
	struct sb2_sem *avcpu_sem;
	struct avcpu_syslog_struct *syslog_p;
	struct delayed_work log_check_work;
	void __iomem *log_buf;
	/* dynamic vars */
	spinlock_t dev_lock;
	atomic_t cnt;
	enum LOG_STAT stat;
	struct task_struct *tsk;
	uint32_t record_start;
	struct completion completion;
	/* dynamic vars doesn't require lock */
	unsigned int hwsem_timeout;
	unsigned int work_delay;
};

struct avlog_ctl_t {
	/* static vars */
	struct class *class;
	struct cdev cdev;
	/* dynamic vars */
	struct log_device_t log_device[MODULE_NUM];
};

/* return value 0 means success getting semaphore with preempt disabled */
static inline int avlog_hwsem_get(struct sb2_sem *sem, unsigned int timeout)
{
	unsigned long jif_timeout;

	jif_timeout = jiffies + msecs_to_jiffies(timeout);
	while (time_before(jiffies, jif_timeout)) {
		preempt_disable();
		/* 1 means lock acquired */
		if (sb2_sem_try_lock(sem, 0)) {
			/* Need to guarantee memory access sequence after sem held */
			mb();
			return 0;
		}
		preempt_enable();
		usleep_range(5 * USEC_PER_MSEC, 10 * USEC_PER_MSEC);
	}

	return -1;
}

static inline int avlog_hwsem_put(struct sb2_sem *sem)
{
	/* write barrier to ensure share data updated before release hwsem */
	wmb();
	sb2_sem_unlock(sem);
	preempt_enable();
	return 0;
}

static void log_check_work_fn(struct work_struct *work)
{
	struct log_device_t *log_device = container_of(work, struct log_device_t, log_check_work.work);
	struct sb2_sem *sem = log_device->avcpu_sem;
	uint32_t log_start, log_end;

	/* wake up reader if fail to get semaphore */
	if (avlog_hwsem_get(sem, log_device->hwsem_timeout)) {
		pr_err("%s : timeout waiting for hwsem\n", __func__);
		spin_lock(&log_device->dev_lock);
		log_device->stat = L_ERR;
		spin_unlock(&log_device->dev_lock);
		complete_all(&log_device->completion);
		return;
	}

	/* check if log had updated, if yes wake up reader. If not,	*/
	/* schedule another work and keeps reader sleep.		*/
	log_start = log_device->syslog_p->log_start;
	log_end = log_device->syslog_p->log_end;
	avlog_hwsem_put(sem);

	spin_lock(&log_device->dev_lock);

	/* Don't do anything if current reader is not in waiting stat */
	if (log_device->stat != L_WAITING) {
		spin_unlock(&log_device->dev_lock);
		return;
	}

	if (log_start != log_end) {
		complete_all(&log_device->completion);
		log_device->stat = L_RUNNING;
	} else {
		if (!schedule_delayed_work(&log_device->log_check_work,
			msecs_to_jiffies(log_device->work_delay))) {
			/* wake up reader directly or reader may sleep forever */
			pr_err("%s: fail to schedule work\n", __func__);
			log_device->stat = L_ERR;
			complete_all(&log_device->completion);
		}
	}

	spin_unlock(&log_device->dev_lock);
}

static int rtk_avcpu_log_open(struct inode *inode, struct file *filp)
{
	struct avlog_ctl_t *av_ctl = container_of(inode->i_cdev, struct avlog_ctl_t, cdev);
	struct log_device_t *log_device;
	struct sb2_sem *sem;
	uint32_t log_start, idx;

	idx = iminor(inode);
	log_device = &av_ctl->log_device[idx];
	sem = log_device->avcpu_sem;

	/* hold sem first since we want to record reader-pointer(log_start) */
	if (avlog_hwsem_get(sem, log_device->hwsem_timeout)) {
		pr_debug("%s: couldn't get hw-sem for more then %d msec\n",
			__func__, HWSEM_DEF_TIMEOUT);
		return -EBUSY;
	}

	log_start = log_device->syslog_p->log_start;
	avlog_hwsem_put(sem);

	spin_lock(&log_device->dev_lock);

	/* we assume L_ERR might have chance to recover?? */
	if (log_device->stat != L_PENDING) {
		spin_unlock(&log_device->dev_lock);
		return -EBUSY;
	}

	log_device->tsk = current;
	log_device->record_start = log_start;
	log_device->stat = L_RUNNING;
	filp->private_data = log_device;
	atomic_inc(&log_device->cnt);
	spin_unlock(&log_device->dev_lock);

	return 0;
}

static int rtk_avcpu_log_release(struct inode *inode, struct file *filp)
{
	struct avlog_ctl_t *av_ctl = container_of(inode->i_cdev, struct avlog_ctl_t, cdev);
	struct log_device_t *log_device;
	uint32_t idx;

	idx = iminor(inode);
	log_device = &av_ctl->log_device[idx];

	spin_lock(&log_device->dev_lock);

	log_device->tsk = NULL;
	if (log_device->stat != L_ERR)
		log_device->stat = L_PENDING;
	spin_unlock(&log_device->dev_lock);
	atomic_dec(&log_device->cnt);

	if (delayed_work_pending(&log_device->log_check_work))
		cancel_delayed_work_sync(&log_device->log_check_work);

	return 0;
}

static ssize_t rtk_avcpu_log_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct log_device_t *log_device = (struct log_device_t *)filp->private_data;
	struct sb2_sem *sem = log_device->avcpu_sem;
	struct avcpu_syslog_struct *p_syslog = log_device->syslog_p;
	static const char drop_msg[] = "*** LOG DROP ***\n";
	char *iter = NULL, *tmp_buf = NULL;
	int err = 0, rcount = 0, log_count = 0, cp_count = 0;
	unsigned int idx_start, idx_end;

again:
	/* if reader wake up by any signal under sleep stat,		*/
	/* switch back to running stat since it's not cause by error.	*/
	if (signal_pending(current)) {
		spin_lock(&log_device->dev_lock);
		if (log_device->stat == L_WAITING)
			log_device->stat = L_RUNNING;
		spin_unlock(&log_device->dev_lock);
		if (delayed_work_pending(&log_device->log_check_work))
			cancel_delayed_work(&log_device->log_check_work);
		return -EINTR;
	}

	if (avlog_hwsem_get(sem, log_device->hwsem_timeout)) {
		spin_lock(&log_device->dev_lock);
		log_device->stat = L_ERR;
		spin_unlock(&log_device->dev_lock);
		pr_err("%s: fail to get hwsem\n", __func__);
		return -EBUSY;
	}

	spin_lock(&log_device->dev_lock);

	if (log_device->stat != L_RUNNING) {
		spin_unlock(&log_device->dev_lock);
		avlog_hwsem_put(sem);
		return -EFAULT;
	}

	if (p_syslog->log_start < log_device->record_start) {
		pr_debug("%s %s: error detect & recover, log_start:0x%x, log_end:0x%x, record_start:0x%x\n",
			__func__, log_device->name, p_syslog->log_start, p_syslog->log_end, log_device->record_start);
		p_syslog->log_start = log_device->record_start;
	}

	/* log_start & log_end is incremental, need to change policy if overflow happened */
	if (p_syslog->log_end < p_syslog->log_start) {
		pr_err("%s: log_start:0x%x, log_end:0x%x, record_start:0x%x\n",
			__func__, p_syslog->log_start, p_syslog->log_end, log_device->record_start);
		log_device->stat = L_ERR;
		spin_unlock(&log_device->dev_lock);
		avlog_hwsem_put(sem);
		return -ENOSPC;
	}

	/* Log buf empty, go to sleep until new log came in */
	if (p_syslog->log_start == p_syslog->log_end) {
		log_device->stat = L_WAITING;
		reinit_completion(&log_device->completion);
		if (!schedule_delayed_work(&log_device->log_check_work,
			msecs_to_jiffies(log_device->work_delay))) {
			pr_err("%s: fail to schedule work\n", __func__);
			log_device->stat = L_ERR;
			spin_unlock(&log_device->dev_lock);
			avlog_hwsem_put(sem);
			return -EFAULT;
		}
		spin_unlock(&log_device->dev_lock);
		avlog_hwsem_put(sem);
		/* work should switch stat back to L_RUNNING */
		wait_for_completion_interruptible(&log_device->completion);
		goto again;
	}

	/* Only emit log if no error or DROP happened.			*/
	/* Ok, now there is valid log inside log buffer. Try to read	*/
	/* as much as we could.						*/
	log_count = p_syslog->log_end - p_syslog->log_start;
	/* make sure if user buffer is enough, if not, fill the buffer */
	if (log_count > count)
		cp_count = count;
	else
		cp_count = log_count;

	tmp_buf = kzalloc(count, GFP_ATOMIC);
	if (!tmp_buf) {
		err = -ENOMEM;
		pr_err("%s : allocate %d buf fail\n", __func__, (int)count);
	}

	/* Log had been rolled over, log-drop-msg meld into read buffer */
	if (p_syslog->log_start != log_device->record_start) {
		int drop_cp_cnt;

		if (count <= strlen(drop_msg)) {
			pr_info("%s: read buf size %d too small\n",
				__func__, (int)count);
			cp_count = 0;
			rcount += count;
			drop_cp_cnt = count;
		} else {
			cp_count -= strlen(drop_msg);
			rcount += strlen(drop_msg);
			drop_cp_cnt = strlen(drop_msg);
		}

		if (tmp_buf)
			memcpy(tmp_buf, drop_msg, drop_cp_cnt);

		pr_debug("%s: drop, record_start:0x%x, log_start:0x%x\n",
			__func__, log_device->record_start, p_syslog->log_start);
	}

	/* since log_start/end is incremental, need to figure out the	*/
	/* correct position in the buffer				*/
	idx_start = p_syslog->log_start % p_syslog->log_buf_len;
	idx_end = p_syslog->log_end % p_syslog->log_buf_len;

	while (!err && cp_count) {
		int tmp_cnt;
		bool wrap;

		wrap = (cp_count + idx_start >= p_syslog->log_buf_len) ? true : false;
		tmp_cnt = wrap ? (p_syslog->log_buf_len - idx_start) :
				cp_count;

		iter = log_device->log_buf + idx_start;
		pr_debug("%s: tmp_buf:0x%p, rcount:%d, log_buf:0x%p, log_start:0x%x, tmp_cnt:%d\n",
			__func__, tmp_buf, rcount, log_device->log_buf, idx_start, tmp_cnt);
		memcpy(tmp_buf + rcount, iter, tmp_cnt);
		rcount += tmp_cnt;
		p_syslog->log_start += tmp_cnt;
		cp_count -= tmp_cnt;
		idx_start = (idx_start + tmp_cnt) % p_syslog->log_buf_len;
	}

	if ((UINT_MAX - p_syslog->log_start) <= (p_syslog->log_buf_len * 2)) {
		bool buf_empty = p_syslog->log_start == p_syslog->log_end ? true : false;

		p_syslog->log_start %= p_syslog->log_buf_len;
		p_syslog->log_end %= p_syslog->log_buf_len;
		if (p_syslog->log_end <= p_syslog->log_start && !buf_empty)
			p_syslog->log_end += p_syslog->log_buf_len;
	}

	log_device->record_start = p_syslog->log_start;

	spin_unlock(&log_device->dev_lock);
	avlog_hwsem_put(sem);

	if (!err) {
		if (copy_to_user(buf, tmp_buf, rcount)) {
			pr_err("%s: copy fail\n", __func__);
			err = -EFAULT;
		}
	}

	kfree(tmp_buf);
	return err ? err : rcount;
}

static const struct file_operations rtk_avcpu_log_fops = {
	.owner = THIS_MODULE,
	.open = rtk_avcpu_log_open,
	.read = rtk_avcpu_log_read,
	.release = rtk_avcpu_log_release,
};

static int parse_dtb(struct log_device_t *log_dev, const struct device_node *node)
{
	int ret = 0;
	unsigned int val = 0;
	struct device_node *np = NULL;
	struct sb2_sem *tmp_sem = NULL;

	if (!log_dev || !node) {
		pr_err("%s: error input\n", __func__);
		return -EFAULT;
	}

	/* First, we have to get correct HW Semaphoe */
	np = of_parse_phandle(node, "sync_lock", 0);
	if (!np || !of_device_is_compatible(np, HWSEM_COMPAT_NAME)) {
		pr_err("%s: parsing lock fail\n", __func__);
		return -ENODEV;
	}

	tmp_sem = sb2_sem_node_to_lock(np);
	if (IS_ERR(tmp_sem)) {
		ret = PTR_ERR(tmp_sem);
		pr_err("%s: get sb2_sem fail\n", __func__);
		return ret;
	}

	log_dev->avcpu_sem = tmp_sem;

	if (of_property_read_u32(node, "sync_lock_timeout", &val)) {
		pr_info("%s : use default %u for hwsem_timeout\n",
			__func__, HWSEM_DEF_TIMEOUT);
		val = HWSEM_DEF_TIMEOUT;
	}
	log_dev->hwsem_timeout = val;

	if (of_property_read_u32(node, "log_check_period", &val)) {
		pr_info("%s : use default %u for work_delay\n",
			__func__, WORK_DEF_DELAY_TIME);
		val = WORK_DEF_DELAY_TIME;
	}
	log_dev->work_delay = val;

	return 0;
}

static int rtk_avcpu_log_device_init(struct platform_device *pdev,
			const int dev_idx, const struct log_device_info_t *dev_info)
{
	struct avcpu_syslog_struct *tmp_syslog_p =
		(struct avcpu_syslog_struct *)(IPC_SHM_VIRT + dev_info->offset);
	struct device_node *np = NULL;
	struct device *device = NULL;
	struct avlog_ctl_t *avlog_ctl = (struct avlog_ctl_t *)dev_get_drvdata(&pdev->dev);
	struct log_device_t *log_dev = &avlog_ctl->log_device[dev_idx];
	int avlog_major = MAJOR(avlog_ctl->cdev.dev);
	int ret = 0;
	int i, npages = 0;
	struct page **pages, **tmp;

	log_dev->name = dev_info->name;
	log_dev->stat = L_NOTAVAIL;

	/* since alog / vlog is a subnode of rtk_avcpu, check if it exist before parsing */
	np = of_get_child_by_name(pdev->dev.of_node, log_dev->name);
	if (!np) {
		pr_info("%s, %s no config detect\n", __func__, log_dev->name);
		return 0;
	}

	pr_debug("%s: %s addr-0x%x, size-0x%x\n",
		__func__, log_dev->name, tmp_syslog_p->log_buf_addr, tmp_syslog_p->log_buf_len);

	/* Check the IPC_SHM part. Related info should be set by bootcode */
	if (tmp_syslog_p->log_buf_addr == 0 || tmp_syslog_p->log_buf_len == 0)
		return -ENODEV;

	ret = parse_dtb(log_dev, np);
	if (ret) {
		pr_err("%s: parse fail\n", __func__);
		return ret;
	}

	spin_lock_init(&log_dev->dev_lock);
	log_dev->syslog_p = tmp_syslog_p;

	npages = round_up(tmp_syslog_p->log_buf_len, PAGE_SIZE) >> PAGE_SHIFT;
	pages = vmalloc(sizeof(struct page *) * npages);
	if (!pages) {
		pr_info("%s, fail to allocate memory", __func__);
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0, tmp = pages; i < npages; i++)
		*(tmp++) = phys_to_page(tmp_syslog_p->log_buf_addr + (PAGE_SIZE * i));

	log_dev->log_buf = vmap(pages, npages, VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	if (!log_dev->log_buf) {
		pr_err("%s: %s fail to map\n", __func__, log_dev->name);
		ret = -ENOMEM;
		goto out;
	}

	device = device_create(avlog_ctl->class, NULL, MKDEV(avlog_major, dev_idx), log_dev, log_dev->name);
	if (IS_ERR(device)) {
		pr_err("%s: device_create with ret %d\n", __func__, ret);
		ret = PTR_ERR(device);
		goto out;
	}

	log_dev->device = device;
	atomic_set(&log_dev->cnt, 0);
	INIT_DELAYED_WORK(&log_dev->log_check_work, log_check_work_fn);
	init_completion(&log_dev->completion);
	log_dev->stat = L_PENDING;

out:
	if (pages)
		vfree(pages);
	if (ret && log_dev->log_buf)
		vunmap(log_dev->log_buf);

	return ret;
}

static int rtk_avcpu_shm_log_probe(struct platform_device *pdev)
{
	dev_t dev;
	struct avlog_ctl_t *avlog_ctl = NULL;
	struct class *class = NULL;
	int i, tmp, ret = 0;

	avlog_ctl = kzalloc(sizeof(struct avlog_ctl_t), GFP_KERNEL);
	if (!avlog_ctl) {
		pr_err("%s: couldn't allocate memory for alog\n", __func__);
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, avlog_ctl);

	/* initialize char device part */
	if (alloc_chrdev_region(&dev, 0, MODULE_NUM, MODULE_NAME) < 0) {
		pr_err("%s: fail to reister char-device\n", __func__);
		ret = -ENODEV;
		goto fail;
	}

	cdev_init(&avlog_ctl->cdev, &rtk_avcpu_log_fops);
	avlog_ctl->cdev.owner = THIS_MODULE;
	ret = cdev_add(&avlog_ctl->cdev, dev, MODULE_NUM);
	if (ret) {
		pr_err("%s: cdev_add with ret %d\n", __func__, ret);
		goto fail_unregister_chrdev;
	}

	/* Then, create class and device to spawn node under dev */
	class = class_create(THIS_MODULE, MODULE_NAME);
	if (IS_ERR(class)) {
		ret = PTR_ERR(class);
		pr_err("%s: class_create with ret %d\n", __func__, ret);
		goto fail_unregister_chrdev;
	}

	avlog_ctl->cdev.owner = THIS_MODULE;
	avlog_ctl->class = class;

	for (i = 0; i < MODULE_NUM; i++) {
		tmp = rtk_avcpu_log_device_init(pdev, i, &log_device_info_list[i]);
		if (tmp)
			pr_err("%s: log_device init fail %d\n", __func__, tmp);
	}

	return 0;

fail_unregister_chrdev:
	unregister_chrdev_region(dev, MODULE_NUM);
fail:
	kfree(avlog_ctl);
	dev_set_drvdata(&pdev->dev, NULL);

	return ret;
}

static int rtk_avcpu_shm_log_remove(struct platform_device *pdev)
{
	struct avlog_ctl_t *ctl = (struct avlog_ctl_t *)dev_get_drvdata(&pdev->dev);
	bool all_finish;
	unsigned long timeout;
	struct log_device_t *log_dev = NULL;
	int i;

	if (!ctl)
		return -ENODEV;

	for (i = 0; i < MODULE_NUM; i++) {
		log_dev = &ctl->log_device[i];
		all_finish = false;
		if (log_dev->stat == L_NOTAVAIL)
			continue;

		spin_lock(&log_dev->dev_lock);
		log_dev->stat = L_EXIT;
		if (log_dev->tsk)
			do_send_sig_info(SIGKILL, SEND_SIG_PRIV, log_dev->tsk, false);
		spin_unlock(&log_dev->dev_lock);

		/* wait for 1 sec before reader killed */
		timeout = jiffies + msecs_to_jiffies(1000);
		while (time_before(jiffies, timeout)) {
			if (!atomic_read(&log_dev->cnt)) {
				all_finish = true;
				break;
			}
			msleep(20);
		}

		if (!all_finish) {
			pr_err("%s:couldn't finish all reader, release fail\n", __func__);
			return -EBUSY;
		}

		device_destroy(ctl->class, log_dev->device->devt);
	}

	class_destroy(ctl->class);
	cdev_del(&ctl->cdev);
	kfree(ctl);

	return 0;
}

static const struct of_device_id rtk_avcpu_shm_log_ids[] = {
	{ .compatible = "Realtek,rtk-avcpu" },
	{ /* Sentinel */ },
};

static struct platform_driver rtk_avcpu_shm_log_driver = {
	.probe = rtk_avcpu_shm_log_probe,
	.remove = rtk_avcpu_shm_log_remove,
	.driver = {
		.name = MODULE_NAME,
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rtk_avcpu_shm_log_ids),
	},
};

module_platform_driver(rtk_avcpu_shm_log_driver);

MODULE_DESCRIPTION("Realtek AVCPU log driver");
MODULE_LICENSE("GPL");
