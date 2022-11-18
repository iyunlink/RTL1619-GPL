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
#include <linux/sched/signal.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/fdtable.h>
#include <linux/ratelimit.h>
#include <linux/io.h>
#include <linux/ion.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/kmemleak.h>
#include <linux/dma-buf.h>
#include <linux/delay.h>
#include <ion_rtk_alloc.h>
#include <soc/realtek/kernel-rpc.h>
#include <soc/realtek/rtk_chip.h>
#include <soc/realtek/uapi/ion_rtk.h>
#include <trace/events/rtk_rpc.h>
#include <uapi/linux/ion.h>

#include "rpc_mem.h"
#include "rtk_rpc.h"

enum ENUM_AUDIO_KERNEL_RPC_CMD {
	ENUM_KERNEL_RPC_CREATE_AGENT,   // 0
	ENUM_KERNEL_RPC_INIT_RINGBUF,
	ENUM_KERNEL_RPC_PRIVATEINFO,
	ENUM_KERNEL_RPC_RUN,
	ENUM_KERNEL_RPC_PAUSE,
	ENUM_KERNEL_RPC_SWITCH_FOCUS,   // 5
	ENUM_KERNEL_RPC_MALLOC_ADDR,
	ENUM_KERNEL_RPC_VOLUME_CONTROL,      // AUDIO_CONFIG_COMMAND
	ENUM_KERNEL_RPC_FLUSH,               // AUDIO_RPC_SENDIO
	ENUM_KERNEL_RPC_CONNECT,             // AUDIO_RPC_CONNECTION
	ENUM_KERNEL_RPC_SETREFCLOCK,    // 10     // AUDIO_RPC_REFCLOCK
	ENUM_KERNEL_RPC_DAC_I2S_CONFIG,      // AUDIO_CONFIG_DAC_I2S
	ENUM_KERNEL_RPC_DAC_SPDIF_CONFIG,    // AUDIO_CONFIG_DAC_SPDIF
	ENUM_KERNEL_RPC_HDMI_OUT_EDID,       // AUDIO_HDMI_OUT_EDID_DATA
	ENUM_KERNEL_RPC_HDMI_OUT_EDID2,      // AUDIO_HDMI_OUT_EDID_DATA2
	ENUM_KERNEL_RPC_HDMI_SET,       // 15     // AUDIO_HDMI_SET
	ENUM_KERNEL_RPC_HDMI_MUTE,           //AUDIO_HDMI_MUTE_INFO
	ENUM_KERNEL_RPC_ASK_DBG_MEM_ADDR,
	ENUM_KERNEL_RPC_DESTROY,
	ENUM_KERNEL_RPC_STOP,
	ENUM_KERNEL_RPC_CHECK_READY,     // 20    // check if Audio get memory pool from AP
	ENUM_KERNEL_RPC_GET_MUTE_N_VOLUME,   // get mute and volume level
	ENUM_KERNEL_RPC_EOS,
	ENUM_KERNEL_RPC_ADC0_CONFIG,
	ENUM_KERNEL_RPC_ADC1_CONFIG,
	ENUM_KERNEL_RPC_ADC2_CONFIG,    // 25
#if defined(AUDIO_TV_PLATFORM)
	ENUM_KERNEL_RPC_BBADC_CONFIG,
	ENUM_KERNEL_RPC_I2SI_CONFIG,
	ENUM_KERNEL_RPC_SPDIFI_CONFIG,
#endif // AUDIO_TV_PLATFORM
	ENUM_KERNEL_RPC_HDMI_OUT_VSDB,
	ENUM_VIDEO_KERNEL_RPC_CONFIG_TV_SYSTEM,
	ENUM_VIDEO_KERNEL_RPC_CONFIG_HDMI_INFO_FRAME,
	ENUM_VIDEO_KERNEL_RPC_QUERY_DISPLAY_WIN,
	ENUM_VIDEO_KERNEL_RPC_PP_INIT_PIN,
	ENUM_KERNEL_RPC_INIT_RINGBUF_AO, //need check this enum
	ENUM_VIDEO_KERNEL_RPC_VOUT_EDID_DATA,
	ENUM_KERNEL_RPC_AUDIO_POWER_SET,
	ENUM_VIDEO_KERNEL_RPC_VOUT_VDAC_SET,
	ENUM_VIDEO_KERNEL_RPC_QUERY_CONFIG_TV_SYSTEM,
	ENUM_KERNEL_RPC_AUDIO_CONFIG,
	ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
	ENUM_KERNEL_RPC_QUERY_FW_DEBUG_INFO,
	ENUM_KERNEL_RPC_HDMI_RX_LATENCY_MEM,
	ENUM_KERNEL_RPC_EQ_CONFIG,
};


enum AUDIO_ENUM_PRIVAETINFO {
	ENUM_PRIVATEINFO_AUDIO_FORMAT_PARSER_CAPABILITY = 0,
	ENUM_PRIVATEINFO_AUDIO_DECODER_CAPABILITY = 1,
	ENUM_PRIVATEINFO_AUDIO_CONFIG_CMD_BS_INFO = 2,
	ENUM_PRIVATEINFO_AUDIO_CHECK_LPCM_ENDIANESS = 3,
	ENUM_PRIVATEINFO_AUDIO_CONFIG_CMD_AO_DELAY_INFO = 4,
	ENUM_PRIVATEINFO_AUDIO_AO_CHANNEL_VOLUME_LEVEL = 5,
	ENUM_PRIVATEINFO_AUDIO_GET_FLASH_PIN = 6,
	ENUM_PRIVATEINFO_AUDIO_RELEASE_FLASH_PIN = 7,
	ENUM_PRIVATEINFO_AUDIO_GET_MUTE_N_VOLUME = 8,
	ENUM_PRIVATEINFO_AUDIO_AO_MONITOR_FULLNESS = 9,
	ENUM_PRIVATEINFO_AUDIO_CONTROL_FLASH_VOLUME = 10,
	ENUM_PRIVATEINFO_AUDIO_CONTROL_DAC_SWITCH = 11,
	ENUM_PRIVATEINFO_AUDIO_PREPROCESS_CONFIG = 12,
	ENUM_PRIVATEINFO_AUDIO_CHECK_SECURITY_ID = 13,
	ENUM_PRIVATEINFO_AUDIO_LOW_DELAY_PARAMETERS = 14,
	ENUM_PRIVATEINFO_AUDIO_SET_NETWORK_JITTER = 15,
	ENUM_PRIVATEINFO_AUDIO_GET_QUEUE_DATA_SIZE = 16,
	ENUM_PRIVATEINFO_AUDIO_GET_SHARE_MEMORY_FROM_ALSA = 17,
	ENUM_PRIVATEINFO_AUDIO_AI_CONNECT_ALSA = 18,
	ENUM_PRIVATEINFO_AUDIO_SET_PCM_FORMAT = 19,
	ENUM_PRIVATEINFO_AUDIO_DO_SELF_DESTROY_FLOW = 20,
	ENUM_PRIVATEINFO_AUDIO_GET_SAMPLING_RATE = 21,
	ENUM_PRIVATEINFO_AUDIO_SLAVE_TIMEOUT_THRESHOLD = 22,
	ENUM_PRIVATEINFO_AUDIO_GET_GLOBAL_AO_INSTANCEID = 23,
	ENUM_PRIVATEINFO_AUDIO_SET_CEC_PARAMETERS = 24,
	ENUM_PRIVATEINFO_AUDIO_INIT_DBG_DUMP_MEM = 25,
	ENUM_PRIVATEINFO_AUDIO_AI_GET_AO_FLASH_PIN = 26,
	ENUM_PRIVATEINFO_AUDIO_AI_SET_AO_FLASH_PIN = 27,
	ENUM_PRIVATEINFO_AUDIO_GET_PP_FREE_PINID = 28,
	ENUM_PRIVATEINFO_AUDIO_HDMI_RX_CONNECT_TO_BT = 29,
	ENUM_PRIVATEINFO_AUDIO_GET_BS_ERR_RATE = 30,
	ENUM_PRIVATEINFO_AUDIO_SET_RESUME_IR_KEYS = 31,
	ENUM_PRIVATEINFO_SET_GSTREAMER_PTS_ACC_MODE = 32,
	ENUM_PRIVATEINFO_AUDIO_GET_BONDING_TYPE = 33,
	ENUM_PRIVATEINFO_AUDIO_SHARE_MEMORY_FOR_PORTING_FIRMWARE = 34,
	ENUM_PRIVATEINFO_AUDIO_SET_DVDPLAYER_AO_VERSION = 35,
	ENUM_PRIVATEINFO_AUDIO_MS_PP_CERT = 36,
	ENUM_PRIVATEINFO_AUDIO_TRIGGER_EVENT = 37,
	ENUM_PRIVATEINFO_AUDIO_AI_NON_PCM_IN = 38,
	ENUM_PRIVATEINFO_OMX_AUDIO_VERSION = 39,
	ENUM_PRIVATEINFO_AUDIO_AI_PAD_IN = 40,
	ENUM_PRIVATEINFO_AUDIO_MS_MAJOR_DECODER_PIN = 41,
	ENUM_PRIVATEINFO_AUDIO_PROVIDE_RAWOUT_LATENCY = 42,
	ENUM_PRIVATEINFO_AUDIO_MS_MIXER_IGNORE_PIN = 43,
	ENUM_PRIVATEINFO_AUDIO_MS_CERTIFICATION_PLATFORM = 44,
	ENUM_PRIVATEINFO_AUDIO_MS_MIXER_PIN_NEW_SEG = 45,
	ENUM_PRIVATEINFO_AUDIO_MS_DEC_DROP_BY_PTS = 46,
	ENUM_PRIVATEINFO_AUDIO_MS_DEC_INIT_PTS_OFFSET = 47,
	ENUM_PRIVATEINFO_AUDIO_MS_PP_OUTPUT_TYPE = 48,
	ENUM_PRIVATEINFO_AUDIO_DTS_ENCODER_CONFIG = 49,
	ENUM_PRIVATEINFO_AUDIO_GET_FW_VERSION = 50,
	ENUM_PRIVATEINFO_AUDIO_DTS_M8_IN_CONFIG = 51,
	ENUM_PRIVATEINFO_AUDIO_DTS_M8_LA_NUM = 52,
	ENUM_PRIVATEINFO_AUDIO_DTS_M8_SET_OUTPUT_FORMAT = 53,
	ENUM_PRIVATEINFO_AUDIO_SET_DRC_CFG = 54,
	ENUM_PRIVATEINFO_AUDIO_DTS_M8_LA_ERROR_MSG = 55,
	ENUM_PRIVATEINFO_GET_B_VALUE = 56,
	ENUM_PRIVATEINFO_AUDIO_ENTER_SUSPEND = 57,
	ENUM_PRIVATEINFO_AUDIO_MPEGH_IN_CONFIG = 58,
	ENUM_PRIVATEINFO_AUDIO_SET_LOW_WATERLEVEL = 59,
};


struct AUDIO_RPC_PRIVATEINFO_PARAMETERS {
	int instanceID;
	enum AUDIO_ENUM_PRIVAETINFO type;
	volatile int privateInfo[16];
};

struct AUDIO_RPC_PRIVATEINFO_RETURNVAL {
	int instanceID;
	volatile int privateInfo[16];
};

#define S_OK        0x10000000


#define ion_alloc ext_rtk_ion_alloc

/*
 * dump ring buffer rate limiting:
 * not more than 1 ring buffer dumping every 3s
 */
DEFINE_RATELIMIT_STATE(ring_dump_state, 3 * HZ, 1);
static DEFINE_SPINLOCK(rpc_release_lock);
static DEFINE_SPINLOCK(release_proc_lock);

struct release_process_lists
{
	int pid;
	int cnt;
	struct list_head list;
};

struct release_process_lists release_proc_lists;

volatile RPC_DEV *rpc_intr_devices;

#ifdef CONFIG_REALTEK_RPC_VE3
volatile RPC_DEV *rpc_intr_ve3_devices;
#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
volatile RPC_HIFI_DEV *rpc_intr_hifi_devices;
#endif


int rpc_intr_is_paused;
int rpc_intr_is_suspend;

int timeout = HZ; //HZ / 40; /* jiffies */

#if defined(CONFIG_REALTEK_RPC_HIFI)
RPC_DEV_EXTRA rpc_intr_extra[RPC_NR_DEVS/RPC_NR_PAIR + RPC_INTR_VE3_DEV_TOTAL + RPC_INTR_HIFI_DEV_TOTAL];
#elif defined(CONFIG_REALTEK_RPC_VE3)
RPC_DEV_EXTRA rpc_intr_extra[RPC_NR_DEVS/RPC_NR_PAIR + RPC_INTR_VE3_DEV_TOTAL];
#else
RPC_DEV_EXTRA rpc_intr_extra[RPC_NR_DEVS/RPC_NR_PAIR];
#endif

extern struct device *rpc_dev;
extern void rpc_send_interrupt(int type);

#ifdef CONFIG_RPC_KERN_VE2
RPC_PROCESS *ve2_proc;
int ve2_proc_count;
#endif


struct RPC_RELEASE_LIST rpc_release_lists;
RPC_PROCESS *remote_proc;

struct task_struct *acpu_r_program_kthread;
wait_queue_head_t acpu_r_program_waitQueue;
int acpu_r_program_flag = 0;

struct task_struct *vcpu_r_program_kthread;
wait_queue_head_t vcpu_r_program_waitQueue;
int vcpu_r_program_flag = 0;

#ifdef CONFIG_REALTEK_RPC_VE3
struct task_struct *ve3_r_program_kthread;
wait_queue_head_t ve3_r_program_waitQueue;
int ve3_r_program_flag = 0;
#endif

#ifdef CONFIG_REALTEK_RPC_HIFI
struct task_struct *hifi_r_program_kthread;
wait_queue_head_t hifi_r_program_waitQueue;
int hifi_r_program_flag = 0;
#endif

ssize_t r_program_read(RPC_DEV_EXTRA *extra, char *buf, size_t count)
{
	int temp, size;
	size_t r;
	ssize_t ret = 0;
	uint32_t ptmp;
	int rpc_ring_size;
	RPC_DEV *dev = extra->dev;
	volatile uint32_t *ringStart, *ringEnd, *ringIn, *ringOut;
	RPC_SYNC_Struct *ptrSync;

#ifdef CONFIG_REALTEK_RPC_HIFI
	RPC_HIFI_DEV *hifi_dev = extra->hifi_dev;

	if (extra->serialno > 5) {
		ringStart = &hifi_dev->ringStart;
		ringEnd = &hifi_dev->ringEnd;
		ringIn = &hifi_dev->ringIn;
		ringOut = &hifi_dev->ringOut;
		ptrSync = hifi_dev->ptrSync;
		rpc_ring_size = *ringEnd - *ringStart;
	} else {
		ringStart = &dev->ringStart;
		ringEnd = &dev->ringEnd;
		ringIn = &dev->ringIn;
		ringOut = &dev->ringOut;
		ptrSync = dev->ptrSync;
		rpc_ring_size = *ringEnd - *ringStart;
	}
#else
	ringStart = &dev->ringStart;
	ringEnd = &dev->ringEnd;
	ringIn = &dev->ringIn;
	ringOut = &dev->ringOut;
	ptrSync = dev->ptrSync;
	rpc_ring_size = *ringEnd - *ringStart;
#endif


	down_write(&ptrSync->readSem);

	if (*ringIn > *ringOut)
		size = *ringIn - *ringOut;
	else
		size = rpc_ring_size + *ringIn - *ringOut;

	pr_debug("%s:%d ==going read== count:%zu avail:%d "
			"ringOut:%x ringIn:%x nextRPC:%x",
			extra->name, __LINE__, count, size, *ringOut, *ringIn,
			extra->nextRpc);

	if (count > size)
		count = size;
	temp = *ringEnd - *ringOut;
	wmb();

	if (temp >= count) {
#ifdef MY_COPY
		r = my_copy_user((int *)buf,
				(int *)AVCPU2SCPU(*ringOut), count);
#else
		r = copy_to_user((int *)buf, (int *)AVCPU2SCPU(*ringOut), count);
#endif /* MY_COPY */
		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n",
				extra->name, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}

		ret = count;
		ptmp = *ringOut + ((count+3) & 0xfffffffc);
		if (ptmp == *ringEnd)
			*ringOut = *ringStart;
		else
			*ringOut = ptmp;

		//pr_debug("RPC Read is in 1st kind...\n");
	} else {
#ifdef MY_COPY
		r = my_copy_user((int *)buf,
				(int *)AVCPU2SCPU(*ringOut), temp);
#else
		r = copy_to_user((int *)buf,
				(int *)AVCPU2SCPU(*ringOut), temp);
#endif /* MY_COPY */

		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n",
					extra->name, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}

		count -= temp;

#ifdef MY_COPY
		r = my_copy_user((int *)(buf+temp),
				(int *)AVCPU2SCPU(*ringStart), count);
#else
		r = copy_to_user((int *)(buf+temp),
				(int *)AVCPU2SCPU(*ringStart), count);
#endif /* MY_COPY */

		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n",
					extra->name, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}
		ret = (temp + count);
		*ringOut = *ringStart+((count+3) & 0xfffffffc);

		//pr_debug("RPC Read is in 2nd kind...\n");
	}
	spin_lock_bh(&extra->lock);

	if (rpc_done(extra)) {
		pr_debug("%s: Previous RPC is done, unregister myself\n", extra->name);
		update_currProc(extra, NULL);
	}

	spin_unlock_bh(&extra->lock);

	if (need_dispatch(extra))
		tasklet_schedule(&(extra->tasklet));
	up_write(&ptrSync->readSem);
	return ret;
out:
	pr_err("[%s] read error occur\n", __func__);
	up_write(&ptrSync->readSem);
	return ret;

}


ssize_t r_program_write(int opt, RPC_DEV_EXTRA *extra, RPC_DEV *dev, char *buf, size_t count)
{

	int temp, size;
	size_t r;
	ssize_t ret = 0;
	uint32_t ptmp;
	RPC_STRUCT *rpc;
	volatile uint32_t *ringStart, *ringEnd, *ringIn, *ringOut;
	RPC_SYNC_Struct *ptrSync;

#ifdef CONFIG_REALTEK_RPC_HIFI
	RPC_HIFI_DEV *hifi_dev = extra->hifi_dev;

	if (extra->serialno > 5) {
		ringStart = &hifi_dev->ringStart;
		ringEnd = &hifi_dev->ringEnd;
		ringIn = &hifi_dev->ringIn;
		ringOut = &hifi_dev->ringOut;
		ptrSync = hifi_dev->ptrSync;
	} else {
		ringStart = &dev->ringStart;
		ringEnd = &dev->ringEnd;
		ringIn = &dev->ringIn;
		ringOut = &dev->ringOut;
		ptrSync = dev->ptrSync;
	}
#else
	ringStart = &dev->ringStart;
	ringEnd = &dev->ringEnd;
	ringIn = &dev->ringIn;
	ringOut = &dev->ringOut;
	ptrSync = dev->ptrSync;
#endif


	rpc = (RPC_STRUCT *)(buf);
	pr_debug("%s: program:%u version:%u procedure:%u taskID:%u sysTID:%u sysPID:%u size:%u context:%x 90k:%u %s\n",
			__func__, ntohl(rpc->programID), ntohl(rpc->versionID), ntohl(rpc->procedureID), ntohl(rpc->taskID), ntohl(rpc->sysTID), ntohl(rpc->sysPID),
			ntohl(rpc->parameterSize),ntohl(rpc->mycontext), (u32)refclk_get_counter(refclk), in_atomic() ? "atomic" : "");
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

	wmb();


	rpc_send_interrupt(opt);


	up_write(&ptrSync->writeSem);
	return ret;
out:
	pr_err("[%s]RingBuf full! RPC FW intr write ringIn pointer is : %p\n", __func__, AVCPU2SCPU(*ringIn));
	up_write(&ptrSync->writeSem);
	return ret;
}


phys_addr_t rtk_rpc_ion_pa(struct r_program_entry *rpc_entry)
{
	struct dma_buf *buf = rpc_entry->rpc_dmabuf;
	struct sg_table *table;
	struct dma_buf_attachment *attachment;
	dma_addr_t dma_addr;
	int err;

	attachment = dma_buf_attach(buf, rpc_dev);
	if (IS_ERR(attachment)) {
		err = PTR_ERR(attachment);
		goto error;
	}
	rpc_entry->attachment = attachment;

	table = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
	if (!table) {
		err = -ENOMEM;
		goto detach;
	}

	dma_addr = sg_dma_address(table->sgl);

	rpc_entry->attachment = attachment;
	rpc_entry->table = table;

	return dma_addr;
detach:

	dma_buf_detach(buf, attachment);
error:

	return err;
}


static void rtk_rpc_free_ion(struct r_program_entry *rpc_entry)
{
	pr_err("%s free ion buffer\n", __func__);
	dma_buf_unmap_attachment(rpc_entry->attachment, rpc_entry->table, DMA_BIDIRECTIONAL);
	dma_buf_detach(rpc_entry->rpc_dmabuf, rpc_entry->attachment);
	dma_buf_put(rpc_entry->rpc_dmabuf);
}

phys_addr_t release_rpc_ion_pa(struct RPC_RELEASE_LIST *release_entry)
{
	struct dma_buf *buf = release_entry->rpc_dmabuf;
	struct sg_table *table;
	struct dma_buf_attachment *attachment;
	dma_addr_t dma_addr;
	int err;


	attachment = dma_buf_attach(buf, rpc_dev);
	if (IS_ERR(attachment)) {
		err = PTR_ERR(attachment);
		goto error;
	}
	release_entry->attachment = attachment;

	table = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
	if (!table) {
		err = -ENOMEM;
		goto detach;
	}

	dma_addr = sg_dma_address(table->sgl);

	release_entry->attachment = attachment;
	release_entry->table = table;

	return dma_addr;
detach:
	if (!IS_ERR_OR_NULL(table))
		dma_buf_unmap_attachment(attachment, table, DMA_BIDIRECTIONAL);

	dma_buf_detach(buf, attachment);
error:

	return err;
}


static void release_rpc_free_ion(struct RPC_RELEASE_LIST *release_entry)
{
	dma_buf_vunmap(release_entry->rpc_dmabuf, release_entry->vaddr);
	dma_buf_unmap_attachment(release_entry->attachment, release_entry->table, DMA_BIDIRECTIONAL);
	dma_buf_detach(release_entry->rpc_dmabuf, release_entry->attachment);
}


#if 1

enum RPC_REMOTE_CMD {
	RPC_REMOTE_CMD_ALLOC = 1,
	RPC_REMOTE_CMD_FREE = 2,
	RPC_REMOTE_CMD_ALLOC_SECURE_LEGACY = 3, /* legacy */
};

struct fw_alloc_parameter_legacy {
	uint32_t size;
} __attribute__((aligned(1)));

struct fw_free_parameter {
	uint32_t phys_addr;
} __attribute__((aligned(1)));

enum E_FW_ALLOC_FLAGS {
	eAlloc_Flag_SCPUACC                 = 1U << 31,
	eAlloc_Flag_ACPUACC                 = 1U << 30,
	eAlloc_Flag_HWIPACC                 = 1U << 29,
	eAlloc_Flag_VE_SPEC                 = 1U << 28,
	eAlloc_Flag_PROTECTED_AUDIO_POOL    = 1U << 27,
	eAlloc_Flag_PROTECTED_TP_POOL       = 1U << 26,
	eAlloc_Flag_PROTECTED_VO_POOL       = 1U << 25,
	eAlloc_Flag_PROTECTED_VIDEO_POOL    = 1U << 24,
	eAlloc_Flag_PROTECTED_AO_POOL       = 1U << 23,
	eAlloc_Flag_PROTECTED_METADATA_POOL = 1U << 22,
	eAlloc_Flag_VCPU_FWACC              = 1U << 21,
	eAlloc_Flag_CMA                     = 1U << 20,
	eAlloc_Flag_PROTECTED_FW_STACK      = 1U << 19,
	eAlloc_Flag_PROTECTED_EXT_BIT0      = 1U << 18,
	eAlloc_Flag_PROTECTED_EXT_BIT1      = 1U << 17,
	eAlloc_Flag_PROTECTED_EXT_BIT2      = 1U << 16,
};

struct fw_alloc_parameter {
	uint32_t size;
	uint32_t flags; /* enum E_FW_ALLOC_FLAGS */
} __attribute__((aligned(1)));

struct reply_fw_parameter {
	uint32_t taskID;
	uint32_t reply_value;
} __attribute__((aligned(1)));


bool is_AudioIntrRead(const RPC_DEV_EXTRA *extra)
{
	return (!strcmp(extra->name, "AudioIntrRead")) ? true : false;
}

bool is_Video1IntrRead(const RPC_DEV_EXTRA *extra)
{
	return (!strcmp(extra->name, "Video1IntrRead")) ? true : false;
}

#ifdef CONFIG_REALTEK_RPC_VE3
bool is_VE3IntrRead(const RPC_DEV_EXTRA *extra)
{
	return (!strcmp(extra->name, "VE3IntrRead")) ? true : false;
}
#endif

#ifdef CONFIG_REALTEK_RPC_HIFI
bool is_HIFIIntrRead(const RPC_DEV_EXTRA *extra)
{
	return (!strcmp(extra->name, "HIFIIntrRead")) ? true : false;
}
#endif

RPC_DEV_EXTRA * get_write_extra(const RPC_DEV_EXTRA *extra)
{
	RPC_DEV_EXTRA * ret = NULL;
	if (is_AudioIntrRead(extra)) {
		ret = &rpc_intr_extra[0];
	} else if (is_Video1IntrRead(extra)) {
		ret = &rpc_intr_extra[2];
	}
#ifdef CONFIG_REALTEK_RPC_VE3
	else if (is_VE3IntrRead(extra)) {
		ret = &rpc_intr_extra[4];
	}
#endif

#ifdef CONFIG_REALTEK_RPC_HIFI
	else if (is_HIFIIntrRead(extra)) {
		ret = &rpc_intr_extra[6];
	}
#endif

	return ret;
}

int get_opt(const RPC_DEV_EXTRA *extra)
{
	if (is_AudioIntrRead(extra))
		return RPC_AUDIO;
	else if (is_Video1IntrRead(extra))
		return RPC_VIDEO;
#ifdef CONFIG_REALTEK_RPC_VE3
	else if (is_VE3IntrRead(extra))
		return RPC_VE3;
#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
	else if (is_HIFIIntrRead(extra))
		return RPC_HIFI;
#endif

	return 0;//RPC_FAIL;
}

void endian_swap_32_read (void * buf, size_t size)
{
	if ((size%sizeof(int)) != 0) {
		pr_err("%s : Illegal size %zu\n", __func__, size);
		return;
	} else {
		unsigned int * pData = (unsigned int *) buf;
		size_t i;
		for (i=0; i<(size/sizeof(int));i++) {
			pData[i] = ntohl(pData[i]);
		}
	}
}

void endian_swap_32_write (void * buf, size_t size)
{
	if ((size%sizeof(int)) != 0) {
		pr_err("%s : Illegal size %zu\n", __func__, size);
		return;
	} else {
		unsigned int * pData = (unsigned int *) buf;
		size_t i;
		for (i=0; i<(size/sizeof(int));i++) {
			pData[i] = htonl(pData[i]);
		}
	}
}

unsigned int rpc_ion_alloc_handler_legacy (RPC_DEV_EXTRA *extra, bool bSecure, const struct fw_alloc_parameter_legacy * param)
{
#define FW_ALLOC_SPEC_MASK  0xC0000000
#define FW_ALLOC_VCPU_FWACC 0x40000000
#define FW_ALLOC_VCPU_EXTRA 0x80000000
	unsigned int reply_value = 0;
	do {
		unsigned int fw_send_value = param->size;
		size_t alloc_val = 0;

		unsigned int ion_alloc_heap_mask;
		unsigned int ion_alloc_flags;
		struct r_program_entry *rpc_entry = kmalloc(sizeof(struct r_program_entry), GFP_KERNEL);

		if (bSecure) {
			ion_alloc_heap_mask = RTK_ION_HEAP_MEDIA_MASK | RTK_ION_HEAP_SECURE_MASK;
			ion_alloc_flags = ION_FLAG_PROTECTED_V2_VO_POOL | ION_FLAG_HWIPACC;
			if (is_AudioIntrRead(extra))
				ion_alloc_flags |= ION_FLAG_ACPUACC;
		} else {
			ion_alloc_heap_mask = RTK_ION_HEAP_MEDIA_MASK;
			ion_alloc_flags = ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_HWIPACC | ION_FLAG_ACPUACC;
		}

		alloc_val = PAGE_ALIGN(fw_send_value & ~FW_ALLOC_SPEC_MASK);

		if (fw_send_value & FW_ALLOC_VCPU_FWACC) {
			ion_alloc_flags |= ION_FLAG_VCPU_FWACC;
		}


		if (fw_send_value & FW_ALLOC_VCPU_EXTRA) {
			rpc_entry->rpc_dmabuf = ion_alloc(alloc_val, ion_alloc_heap_mask, ion_alloc_flags | ION_FLAG_CMA);
		} else {
			ion_alloc_flags |= ION_USAGE_ALGO_LAST_FIT;
			rpc_entry->rpc_dmabuf = ion_alloc(alloc_val, ion_alloc_heap_mask, ion_alloc_flags);
		}

		rpc_entry->next = NULL;
		rpc_entry->phys_addr = rtk_rpc_ion_pa(rpc_entry);
		if (rpc_entry->phys_addr < 0) {
			pr_err("[%s] get dma addr failed\n", __func__);
			reply_value = -1;
			dma_buf_put(rpc_entry->rpc_dmabuf);
			kfree(rpc_entry);
			break;
		}
		rpc_entry->size = alloc_val;
		r_program_add(rpc_entry);

		reply_value = rpc_entry->phys_addr;

		pr_debug("[%s] ion_alloc addr : 0x%x (heap:0x%x flags:0x%x)\n", __func__,
				reply_value, ion_alloc_heap_mask, ion_alloc_flags);

	} while (0);
	return reply_value;
}

unsigned int rpc_ion_alloc_handler(RPC_DEV_EXTRA *extra, const struct fw_alloc_parameter * param)
{
	unsigned int reply_value = 0;
	unsigned int retry_count = 0;

	do {
		size_t alloc_size = param->size;
		unsigned int alloc_flags = param->flags;
		int handle = -1;
		unsigned int ion_alloc_heap_mask = RTK_ION_HEAP_MEDIA_MASK | RTK_ION_HEAP_SECURE_MASK;
		unsigned int ion_alloc_flags = 0;
		struct r_program_entry * rpc_entry = kmalloc(sizeof(struct r_program_entry), GFP_KERNEL);
retry:
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_SCPUACC                 ) ? ION_FLAG_SCPUACC : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_ACPUACC                 ) ? ION_FLAG_ACPUACC : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_HWIPACC                 ) ? ION_FLAG_HWIPACC : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_VE_SPEC                 ) ? ION_FLAG_VE_SPEC : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_PROTECTED_AUDIO_POOL    ) ? ION_FLAG_PROTECTED_V2_AUDIO_POOL : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_PROTECTED_TP_POOL       ) ? ION_FLAG_PROTECTED_V2_TP_POOL : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_PROTECTED_VO_POOL       ) ? ION_FLAG_PROTECTED_V2_VO_POOL : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_PROTECTED_VIDEO_POOL    ) ? ION_FLAG_PROTECTED_V2_VIDEO_POOL : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_PROTECTED_AO_POOL       ) ? ION_FLAG_PROTECTED_V2_AO_POOL : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_PROTECTED_METADATA_POOL ) ? ION_FLAG_PROTECTED_V2_METADATA_POOL : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_VCPU_FWACC              ) ? ION_FLAG_VCPU_FWACC : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_CMA                     ) ? ION_FLAG_CMA : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_PROTECTED_FW_STACK      ) ? ION_FLAG_PROTECTED_V2_FW_STACK : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_PROTECTED_EXT_BIT0      ) ? ION_FLAG_PROTECTED_EXT_BIT0 : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_PROTECTED_EXT_BIT1      ) ? ION_FLAG_PROTECTED_EXT_BIT1 : 0;
		ion_alloc_flags |= (alloc_flags & eAlloc_Flag_PROTECTED_EXT_BIT2      ) ? ION_FLAG_PROTECTED_EXT_BIT2 : 0;

		rpc_entry->rpc_dmabuf = ion_alloc(alloc_size, ion_alloc_heap_mask, ion_alloc_flags);

		if (IS_ERR(rpc_entry->rpc_dmabuf) && (ion_alloc_flags != ION_FLAG_CMA) && (ion_alloc_flags & ION_FLAG_CMA) && (PTR_ERR(rpc_entry->rpc_dmabuf) != -EINVAL)) {
			ion_alloc_flags &= ~ION_FLAG_CMA;
			rpc_entry->rpc_dmabuf = ion_alloc(alloc_size, ion_alloc_heap_mask, ion_alloc_flags);
		}


		if(IS_ERR(rpc_entry->rpc_dmabuf) && retry_count < retry_count_value && (PTR_ERR(rpc_entry->rpc_dmabuf) != -EINVAL)){
			msleep(retry_delay_value);
			retry_count++;
			ion_alloc_flags = 0;
			goto retry;
		}

		if (retry_count > 0)
			pr_err("%s: ion alloc retry_delay = %d, count = %d, MAX_COUNT = %d\n",
					__func__, retry_delay_value, retry_count, retry_count_value);


		if (IS_ERR(rpc_entry->rpc_dmabuf)) {
			pr_err("[%s] ERROR: ion_alloc fail %d(heap:0x%x flags:0x%x)\n", __func__,
											handle,
											ion_alloc_heap_mask,
											ion_alloc_flags);
			reply_value = -1U;
			break;
		}

		rpc_entry->next = NULL;
		rpc_entry->phys_addr = rtk_rpc_ion_pa(rpc_entry);
		if (rpc_entry->phys_addr < 0) {
			pr_err("[%s] get dma addr failed\n", __func__);
			reply_value = -1;
			dma_buf_put(rpc_entry->rpc_dmabuf);
			kfree(rpc_entry);
			break;
		}
		rpc_entry->size = alloc_size;
		r_program_add(rpc_entry);
		reply_value = rpc_entry->phys_addr;

		pr_debug("[%s] ion_alloc addr : 0x%x (heap:0x%x flags:0x%x)\n", __func__,
				reply_value, ion_alloc_heap_mask, ion_alloc_flags);
	} while (0);

	return reply_value;

}

unsigned int rpc_ion_free_handler(RPC_DEV_EXTRA *extra, const struct fw_free_parameter * param)
{
	unsigned int reply_value = 0;
	unsigned int phys_addr = param->phys_addr;
	struct r_program_entry * rpc_entry = r_program_remove(phys_addr);
	if (rpc_entry) {
		rtk_rpc_free_ion(rpc_entry);
		kfree(rpc_entry);
		pr_debug("[%s] ion_free addr : 0x%x (reply_value : 0x%x)\n", __func__, phys_addr, reply_value);
		reply_value = 0;
	} else {
		pr_err("[%s]cannot find rpc_entry to free:phys_addr:%x\n", __func__, phys_addr);
		reply_value = -1U;
	}
	return reply_value;
}

void rpc_ion_handler(RPC_DEV_EXTRA *extra)
{
	do {
		unsigned int reply_value = 0;
		RPC_STRUCT rpc_header;
		RPC_STRUCT * rpc = &rpc_header;
		enum RPC_REMOTE_CMD remote_cmd = 0;

		peek_rpc_struct(__func__, extra, 0);

		if (r_program_read(extra, (char *) rpc, sizeof(RPC_STRUCT)) != sizeof(RPC_STRUCT)) {
			pr_err("%s:%d\n", __func__, __LINE__);
			break;
		}
#ifdef CONFIG_REALTEK_RPC_HIFI
		if (strcmp(extra->name, "HIFIIntrRead"))
			endian_swap_32_read(rpc, sizeof(*rpc));
#else
		endian_swap_32_read(rpc, sizeof(*rpc));
#endif
		pr_debug("%s: program:%u version:%u procedure:%u taskID:%u sysTID:%u sysPID:%u size:%u context:%x 90k:%u %s\n",
				__func__, (rpc->programID), (rpc->versionID), (rpc->procedureID), (rpc->taskID), (rpc->sysTID), (rpc->sysPID),
				(rpc->parameterSize), (rpc->mycontext), (u32)refclk_get_counter(refclk), in_atomic() ? "atomic" : "");

		if (rpc->parameterSize == 0) {
			pr_err("%s: parameterSize is zero!!\n", __func__);
			break;
		}

		remote_cmd = (enum RPC_REMOTE_CMD) rpc->procedureID;

		switch (remote_cmd) {
			case RPC_REMOTE_CMD_ALLOC_SECURE_LEGACY:
			case RPC_REMOTE_CMD_ALLOC:
				if (rpc->parameterSize == sizeof(struct fw_alloc_parameter_legacy)) {
					bool bSecure = (remote_cmd == RPC_REMOTE_CMD_ALLOC_SECURE_LEGACY) ? true : false;
					struct fw_alloc_parameter_legacy param;

					if (r_program_read(extra, (char *) &param, sizeof(param)) != sizeof(param)) {
						pr_err("%s : read struct fw_alloc_parameter_legacy failed!\n", __func__);
						break;
					}
#ifdef CONFIG_REALTEK_RPC_HIFI
					if (strcmp(extra->name, "HIFIIntrRead"))
						endian_swap_32_read(&param, sizeof(param));
#else
					endian_swap_32_read(&param, sizeof(param));
#endif
					reply_value = rpc_ion_alloc_handler_legacy(extra, bSecure, &param);

				} else if (rpc->parameterSize == sizeof(struct fw_alloc_parameter)) {
					struct fw_alloc_parameter param;

					if (r_program_read(extra, (char *) &param, sizeof(param)) != sizeof(param)) {
						pr_err("%s : read struct fw_alloc_parameter failed!\n", __func__);
						break;
					}
#ifdef CONFIG_REALTEK_RPC_HIFI
					if (strcmp(extra->name, "HIFIIntrRead"))
						endian_swap_32_read(&param, sizeof(param));
#else
					endian_swap_32_read(&param, sizeof(param));
#endif

					reply_value = rpc_ion_alloc_handler(extra, &param);
				} else {
					pr_err("%s : parameterSize(%d) mismatch!\n", __func__, rpc->parameterSize);
					break;
				}

				rpc->mycontext &= 0xfffffffc;
				break;
			case RPC_REMOTE_CMD_FREE:
				if (rpc->parameterSize == sizeof(struct fw_free_parameter)) {
					struct fw_free_parameter param;

					if (r_program_read(extra, (char *) &param, sizeof(param)) != sizeof(param)) {
						pr_err("%s : read struct fw_alloc_parameter failed!\n", __func__);
						break;
					}
#ifdef CONFIG_REALTEK_RPC_HIFI
					if (strcmp(extra->name, "HIFIIntrRead"))
						endian_swap_32_read(&param, sizeof(param));
#else
					endian_swap_32_read(&param, sizeof(param));
#endif
					reply_value = rpc_ion_free_handler(extra, &param);

				} else {
					pr_err("%s : parameterSize(%d) mismatch!\n", __func__, rpc->parameterSize);
					break;
				}
				rpc->mycontext &= 0xfffffffc;
				break;
			default:
				reply_value = 0;
				pr_err("%s : Unknow cmd 0x%x parameterSize = %d\n", __func__, remote_cmd, rpc->parameterSize);
				break;
		}

		{
			/*Reply RPC*/
			RPC_DEV_EXTRA *extra_w = get_write_extra(extra);
			struct {
				RPC_STRUCT header;
				struct reply_fw_parameter reply;
			} __attribute__((aligned(1))) reply_rpc;

			reply_rpc.header.programID = REPLYID;
			reply_rpc.header.versionID = REPLYID;
			reply_rpc.header.procedureID = 0;
			reply_rpc.header.taskID = 0;
#ifdef RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID
			reply_rpc.header.sysTID = 0;
#endif /* RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID */
			reply_rpc.header.sysPID = 0;
			reply_rpc.header.parameterSize = (unsigned int) sizeof(struct reply_fw_parameter);
			reply_rpc.header.mycontext = rpc->mycontext;
			reply_rpc.reply.taskID = rpc->taskID;
			reply_rpc.reply.reply_value = reply_value;

			trace_rtk_rpc_peek_rpc_reply((struct rpc_struct_tp *)&reply_rpc.header,
					(u32)refclk_get_counter(refclk), 0, rpc->taskID, false);
#ifdef CONFIG_REALTEK_RPC_HIFI
			if (strcmp(extra->name, "HIFIIntrRead"))
					endian_swap_32_write(&reply_rpc, sizeof(reply_rpc));
#else
					endian_swap_32_write(&reply_rpc, sizeof(reply_rpc));
#endif

			while (r_program_write(
						get_opt(extra), extra_w, extra_w->dev, (char *) &reply_rpc, sizeof(reply_rpc)) != (sizeof(reply_rpc))) {
				pr_err("[%s] r_program_write error...\n", __func__);
				msleep(1);
			}
		}
		//tasklet_schedule(&(extra->tasklet));
		break;
	} while (0);
}

#else

void rpc_ion_handler(RPC_DEV_EXTRA *extra)
{
	int handle = -1;
	phys_addr_t phys_addr;
	unsigned long reply_value = 0;
	char tmpbuf[sizeof(RPC_STRUCT) + sizeof(uint32_t)];
	uint32_t *tmp;
	char replybuf[sizeof(RPC_STRUCT) + 2*sizeof(uint32_t)];
	RPC_STRUCT *rpc;
	RPC_STRUCT *rrpc;
	unsigned long map_addr;
	RPC_DEV_EXTRA *extra_w;
	r_program_entry_t *rpc_entry;
	int opt = 0;

	if(!strcmp(extra->name, "AudioIntrRead")) {
		extra_w = &rpc_intr_extra[0];
		opt = RPC_AUDIO;
	} else if (!strcmp(extra->name, "Video1IntrRead")){
		extra_w = &rpc_intr_extra[2];
		opt = RPC_VIDEO;
	} else if (!strcmp(extra->name, "VE3IntrRead") {
		extra_w = &rpc_intr_extra[4];
		opt = RPC_VE3;
	}

	pr_debug("[%s] handle %s rpc\n", extra->name);
	if (r_program_read(extra, extra->dev, (char *) &tmpbuf, sizeof(RPC_STRUCT) + sizeof(uint32_t)) != (sizeof(RPC_STRUCT) + sizeof(uint32_t))) {
		pr_err("[%s] remote allocate read error...\n", __func__);
		return;
	}

	rpc = (RPC_STRUCT *)(tmpbuf);

	//convert_rpc_struct(extra->name, rpc);
	pr_debug("%s: program:%u version:%u procedure:%u taskID:%u sysTID:%u sysPID:%u size:%u context:%x 90k:%u %s\n",
			__func__, ntohl(rpc->programID), ntohl(rpc->versionID), ntohl(rpc->procedureID), ntohl(rpc->taskID), ntohl(rpc->sysTID), ntohl(rpc->sysPID),
			ntohl(rpc->parameterSize), ntohl(rpc->mycontext), (u32)refclk_get_counter(refclk), in_atomic() ? "atomic" : "");

{
#if defined(CONFIG_ARCH_RTD129x) || defined(CONFIG_ARCH_RTD119X)
	const bool match_old_mapping = true;
#else
	const bool match_old_mapping = false;
#endif
	enum RPC_REMOTE_CMD {
		RPC_REMOTE_CMD_ALLOC = 1,
		RPC_REMOTE_CMD_FREE = 2,
		RPC_REMOTE_CMD_ALLOC_SECURE = 3,
	} remote_cmd;

	size_t align;
	size_t alloc_val;
	unsigned int fw_send_value;
	tmp = (uint32_t *)(tmpbuf + sizeof(RPC_STRUCT));

	fw_send_value = ntohl(*tmp);
	remote_cmd = (enum RPC_REMOTE_CMD) ntohl(rpc->procedureID);

	switch (remote_cmd) {
	case RPC_REMOTE_CMD_ALLOC:
	case RPC_REMOTE_CMD_ALLOC_SECURE:
	{
#define FW_ALLOC_SPEC_MASK  0xC0000000
#define FW_ALLOC_VCPU_FWACC 0x40000000
#define FW_ALLOC_VCPU_EXTRA 0x80000000
		unsigned int ion_alloc_heap_mask;
		unsigned int ion_alloc_flags;

		if (remote_cmd == RPC_REMOTE_CMD_ALLOC_SECURE) {
			ion_alloc_heap_mask = RTK_ION_HEAP_MEDIA_MASK | RTK_ION_HEAP_SECURE_MASK;
			ion_alloc_flags = ION_FLAG_PROTECTED_V2_VO_POOL | ION_FLAG_HWIPACC;
			if (opt == RPC_AUDIO)
				ion_alloc_flags |= ION_FLAG_ACPUACC;
		} else {
			ion_alloc_heap_mask = RTK_ION_HEAP_MEDIA_MASK;
			ion_alloc_flags = ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_HWIPACC | ION_FLAG_ACPUACC;
		}

		alloc_val = PAGE_ALIGN(fw_send_value & ~FW_ALLOC_SPEC_MASK);

		if (fw_send_value & FW_ALLOC_VCPU_FWACC) {
			ion_alloc_flags |= ION_FLAG_VCPU_FWACC;
		}

        if (fw_send_value & FW_ALLOC_VCPU_EXTRA) {
			handle = ion_alloc(alloc_val, ion_alloc_heap_mask, ion_alloc_flags | ION_FLAG_CMA);
			ion_alloc_flags |= ION_USAGE_ALGO_LAST_FIT;
		}

		if (handle < 0)
			handle = ion_alloc(alloc_val, ion_alloc_heap_mask, ion_alloc_flags);

		rpc->mycontext = htonl(ntohl(rpc->mycontext) & 0xfffffffc);

		if (handle < 0) {
			pr_err("[%s] ERROR: ion_alloc fail %d(heap:0x%x flags:0x%x)\n", __func__, (int *) handle, ion_alloc_heap_mask, ion_alloc_flags);
			reply_value = -1U;
			break;
		}

		rpc_entry = kmalloc(sizeof(struct r_program_entry), GFP_KERNEL);
		rpc_entry->rpc_dmabuf = dma_buf_get(handle);
		rpc_entry->phys_addr = rtk_rpc_ion_pa(rpc_entry->rpc_dmabuf->priv);
		rpc_entry->size = alloc_val;
		r_program_add(rpc_entry);
		reply_value = rpc_entry->phys_addr;

		__close_fd(current->files, handle);

		if (match_old_mapping)
			reply_value += 0x80000000;
			pr_debug("[%s] ion_alloc addr : 0x%x (heap:0x%x flags:0x%x)\n", __func__, reply_value, ion_alloc_heap_mask, ion_alloc_flags);
		}

		break;
	case RPC_REMOTE_CMD_FREE:
	{
		phys_addr = fw_send_value;
		if (match_old_mapping)
			phys_addr -= 0x80000000;

		rpc_entry = r_program_remove(phys_addr);

		if (rpc_entry) {
			rtk_rpc_free_ion(rpc_entry->rpc_dmabuf);
			kfree(rpc_entry);
			pr_debug("[%s] ion_free addr : 0x%x (reply_value : 0x%lx)\n", __func__, phys_addr, reply_value);
			reply_value = 0;
		} else {
			pr_err("[%s]cannot find rpc_entry to free:phys_addr:%x\n", __func__, phys_addr);
			reply_value = -1U;
#if 0 /* WORKAROUND : Avoid FW deadlock when phys_addr is illegal */
			return;
#endif
		}

		rpc->mycontext = htonl(ntohl(rpc->mycontext) & 0xfffffffc);
	}
		break;
	default:
		break;
	}
}

	/*Reply RPC*/
	rrpc = (RPC_STRUCT *)replybuf;
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
	*(tmp+1) = htonl(reply_value);

	if (r_program_write(opt, extra_w, extra_w->dev, (char *) &replybuf, sizeof(RPC_STRUCT) + 2*sizeof(uint32_t)) != (sizeof(RPC_STRUCT) + 2*sizeof(uint32_t))) {
		pr_err("[%s] remote allocate reply error...\n", __func__);
		return;
	}

}
#endif

void add_release_list(int pid, int rpc_type)
{
	struct release_process_lists *proc_entry;
	struct release_process_lists *tmp_entry;
	struct list_head *listptr;

	spin_lock_bh(&release_proc_lock);
	list_for_each(listptr, &release_proc_lists.list) {
		tmp_entry = list_entry(listptr, struct release_process_lists, list);
		if (tmp_entry->pid == pid){
			spin_unlock_bh(&release_proc_lock);
			return;
		}
	}
	spin_unlock_bh(&release_proc_lock);

	proc_entry = kmalloc(sizeof(struct release_process_lists), GFP_KERNEL);
	proc_entry->pid = pid;
	if (rpc_type == 4 || rpc_type == 5)
		proc_entry->cnt = 2;
	else
		proc_entry->cnt = 4;


	spin_lock_bh(&release_proc_lock);
	list_add(&proc_entry->list, &release_proc_lists.list);
	spin_unlock_bh(&release_proc_lock);
}

void rm_release_list(int pid)
{
	struct list_head *listptr;
	struct release_process_lists *entry;

	spin_lock_bh(&release_proc_lock);
	list_for_each(listptr, &release_proc_lists.list) {
		entry = list_entry(listptr, struct release_process_lists, list);
		if (entry->pid == pid) {
			entry->cnt--;
			if (entry->cnt == 0) {
				list_del(listptr);
				kfree(entry);
			}
			spin_unlock_bh(&release_proc_lock);
			return;
		}
	}
	spin_unlock_bh(&release_proc_lock);
}



int check_dead_process(int pid){

	struct list_head *listptr;
	struct release_process_lists *entry;

	spin_lock_bh(&release_proc_lock);
	list_for_each(listptr, &release_proc_lists.list) {
		entry = list_entry(listptr, struct release_process_lists, list);
		if (entry->pid == pid) {
			spin_unlock_bh(&release_proc_lock);
			return 0;
		}
	}
	spin_unlock_bh(&release_proc_lock);
	return -EINVAL;
}

void ignore_rpc(RPC_DEV_EXTRA *extra, uint32_t data_size)
{
	RPC_DEV *dev = extra->dev;
	volatile uint32_t *ringStart, *ringEnd, *ringIn, *ringOut;

#ifdef CONFIG_REALTEK_RPC_HIFI
	RPC_HIFI_DEV *hifi_dev = extra->hifi_dev;

	if (extra->serialno > 5) {
		ringStart = &hifi_dev->ringStart;
		ringEnd = &hifi_dev->ringEnd;
		ringIn = &hifi_dev->ringIn;
		ringOut = &hifi_dev->ringOut;
	} else {
		ringStart = &dev->ringStart;
		ringEnd = &dev->ringEnd;
		ringIn = &dev->ringIn;
		ringOut = &dev->ringOut;
	}
#else
	ringStart = &dev->ringStart;
	ringEnd = &dev->ringEnd;
	ringIn = &dev->ringIn;
	ringOut = &dev->ringOut;
#endif

	*ringOut = *ringOut + sizeof(RPC_STRUCT) + data_size;
	if (*ringOut >= *ringEnd)
		*ringOut = *ringStart + (*ringOut - *ringEnd);
}


void handle_dead_process_reply(RPC_STRUCT rpc, RPC_DEV_EXTRA *extra)
{
	RPC_STRUCT *rrpc;
	uint32_t *tmp;
	char replybuf[sizeof(RPC_STRUCT) + 2*sizeof(uint32_t)];
	RPC_DEV_EXTRA *extra_w;
	int opt;

	if (rpc.taskID == 0) {
		ignore_rpc(extra, rpc.parameterSize);
		return;
	}

	if(!strcmp(extra->name, "AudioIntrRead")) {
		extra_w = &rpc_intr_extra[0];
		opt = RPC_AUDIO;
	} else if (!strcmp(extra->name, "Video1IntrRead")){
		extra_w = &rpc_intr_extra[2];
		opt = RPC_VIDEO;
	}
#ifdef CONFIG_REALTEK_RPC_VE3
	else if (!strcmp(extra->name, "VE3IntrRead")){
		extra_w = &rpc_intr_extra[4];
		opt = RPC_VE3;
	}
#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
	else if (!strcmp(extra->name, "HIFIIntrRead")){
		extra_w = &rpc_intr_extra[6];
		opt = RPC_HIFI;
	}
#endif
	else {
		return;
	}
	/*Reply RPC*/
	rrpc = (RPC_STRUCT *)replybuf;
	rrpc->programID = htonl(REPLYID);
	rrpc->versionID = htonl(REPLYID);
	rrpc->procedureID = 0;
	rrpc->taskID = 0;
#ifdef RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID
	rrpc->sysTID = 0;
#endif /* RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID */
	rrpc->sysPID = 0;
	rrpc->parameterSize = htonl(2*sizeof(uint32_t));
	rrpc->mycontext = htonl(rpc.mycontext);

	/* fill the parameters... */
	tmp = (uint32_t *)(replybuf + sizeof(RPC_STRUCT));
	*(tmp+0) = htonl(rpc.taskID); /* FIXME: should be 64bit */
	*(tmp+1) = htonl(0xdead);

	ignore_rpc(extra, rpc.parameterSize);

	if (r_program_write(opt, extra_w, extra_w->dev, (char *) &replybuf, sizeof(RPC_STRUCT) + 2*sizeof(uint32_t)) != (sizeof(RPC_STRUCT) + 2*sizeof(uint32_t))) {
		pr_err("[%s] handle_dead_process_reply error...\n", __func__);
		return;
	}

}

static int acpu_remote_alloc_thread(void * p)
{
	RPC_DEV_EXTRA *extra = &rpc_intr_extra[1];

	while (1) {
		if (wait_event_interruptible(acpu_r_program_waitQueue, acpu_r_program_flag || kthread_should_stop())) {
			pr_notice("%s got signal or should stop...\n", current->comm);
			continue;
		}

		if (kthread_should_stop()) {
			pr_notice("%s exit...\n", current->comm);
			break;
		}
		spin_lock_bh(&extra->lock);
		acpu_r_program_flag = 0;
		spin_unlock_bh(&extra->lock);
		rpc_ion_handler(extra);
	}
	return 0;
}

static int vcpu_remote_alloc_thread(void * p)
{
	RPC_DEV_EXTRA *extra = &rpc_intr_extra[3];

	while (1) {
		if (wait_event_interruptible(vcpu_r_program_waitQueue, vcpu_r_program_flag || kthread_should_stop())) {
			pr_notice("%s got signal or should stop...\n", current->comm);
			continue;
		}

		if (kthread_should_stop()) {
			pr_notice("%s exit...\n", current->comm);
			break;
		}
		spin_lock_bh(&extra->lock);
		vcpu_r_program_flag = 0;
		spin_unlock_bh(&extra->lock);
		rpc_ion_handler(extra);
	}
	return 0;
}

#ifdef CONFIG_REALTEK_RPC_VE3
static int ve3_remote_alloc_thread(void * p)
{
	RPC_DEV_EXTRA *extra = &rpc_intr_extra[5];

	while (1) {
		if (wait_event_interruptible(ve3_r_program_waitQueue, ve3_r_program_flag || kthread_should_stop())) {
			pr_notice("%s got signal or should stop...\n", current->comm);
			continue;
		}

		if (kthread_should_stop()) {
			pr_notice("%s exit...\n", current->comm);
			break;
		}
		spin_lock_bh(&extra->lock);
		ve3_r_program_flag = 0;
		spin_unlock_bh(&extra->lock);
		rpc_ion_handler(extra);
	}
	return 0;
}
#endif

#ifdef CONFIG_REALTEK_RPC_HIFI
static int hifi_remote_alloc_thread(void * p)
{
	RPC_DEV_EXTRA *extra = &rpc_intr_extra[7];

	while (1) {
		if (wait_event_interruptible(hifi_r_program_waitQueue, hifi_r_program_flag || kthread_should_stop())) {
			pr_notice("%s got signal or should stop...\n", current->comm);
			continue;
		}

		if (kthread_should_stop()) {
			pr_notice("%s exit...\n", current->comm);
			break;
		}
		spin_lock_bh(&extra->lock);
		hifi_r_program_flag = 0;
		spin_unlock_bh(&extra->lock);
		rpc_ion_handler(extra);
	}
	return 0;
}
#endif

/*
 * This function may be called by tasklet and rpc_intr_read(),
 * rpc_poll_read()
 */
void rpc_dispatch(unsigned long data)
{
	RPC_DEV_EXTRA *extra = (RPC_DEV_EXTRA *) data;

	RPC_PROCESS *proc = NULL;
	uint32_t out;
	RPC_STRUCT rpc;
	int found = 0;
	uint32_t nextRpc = extra->nextRpc;
	RPC_PROCESS *curr = (RPC_PROCESS *)extra->currProc;
	int taskID = 0;
	volatile uint32_t *ringStart, *ringEnd, *ringIn, *ringOut;

#ifdef CONFIG_REALTEK_RPC_HIFI
	RPC_HIFI_DEV *hifi_dev = extra->hifi_dev;
	RPC_DEV *dev = extra->dev;

	if (extra->serialno > 5) {
		ringStart = &hifi_dev->ringStart;
		ringEnd = &hifi_dev->ringEnd;
		ringIn = &hifi_dev->ringIn;
		ringOut = &hifi_dev->ringOut;
	} else {
		ringStart = &dev->ringStart;
		ringEnd = &dev->ringEnd;
		ringIn = &dev->ringIn;
		ringOut = &dev->ringOut;
	}
#else
	RPC_DEV *dev = extra->dev;

	ringStart = &dev->ringStart;
	ringEnd = &dev->ringEnd;
	ringIn = &dev->ringIn;
	ringOut = &dev->ringOut;
#endif

	pr_debug("==*==*==%s Out:%x next:%x In:%x size:%d %s==*==*==\n",
			extra->name, *ringOut, nextRpc, *ringIn, get_ring_data_size(extra), in_atomic() ? "atomic" : "");
	if (curr != NULL || *ringOut == *ringIn || *ringOut != nextRpc) {
		pr_debug("%s: unable to disp. currProc:%d Out:%x next:%x "
				"In:%x size:%d %s\n",
				extra->name, curr ? curr->pid : 0, *ringOut, nextRpc, *ringIn,
				get_ring_data_size(extra), in_atomic() ? "atomic" : "");
		if (curr != NULL && check_dead_process(curr->pid) == 0) {
			pr_debug("[rpc_dispatch]: process has been closed. ignore RPC.\n");
			if (!rpc_done(extra)){
				*ringOut = extra->nextRpc;
			}
			spin_lock_bh(&extra->lock);
			update_currProc(extra, NULL);
			spin_unlock_bh(&extra->lock);
			if (need_dispatch(extra)) {
				tasklet_schedule(&(extra->tasklet));
			}
		}
		return;
	}

	//peek_rpc_struct(extra->name, dev);
	out = get_ring_data(extra->name, extra, *ringOut, (char *) &rpc,
			sizeof(RPC_STRUCT));
	if (out == 0)
		return;
#ifdef CONFIG_REALTEK_RPC_HIFI
	if (strcmp(extra->name, "HIFIIntrRead"))
		convert_rpc_struct(extra->name, &rpc);
#else
	convert_rpc_struct(extra->name, &rpc);
#endif
	show_rpc_struct(__func__, &rpc);

	switch (rpc.programID) {
	case R_PROGRAM:
		pr_debug("%s: program:%u version:%u procedure:%u taskID:%u sysTID:%u sysPID:%u size:%u context:%x 90k:%u %s\n",
			__func__, rpc.programID, rpc.versionID, rpc.procedureID, rpc.taskID, rpc.sysTID, rpc.sysPID,
			rpc.parameterSize, rpc.mycontext, (u32)refclk_get_counter(refclk), in_atomic() ? "atomic" : "");

		out += rpc.parameterSize;
		if (out >= *ringEnd)
			out = *ringStart + (out - *ringEnd);
		if (!strcmp(extra->name, "AudioIntrRead")) {/*audio remote allocate*/
			spin_lock_bh(&extra->lock);
			update_nextRpc(extra, out);
			update_currProc(extra, remote_proc);
			acpu_r_program_flag = 1;
			wake_up_interruptible(&acpu_r_program_waitQueue);
			spin_unlock_bh(&extra->lock);
		} else if (!strcmp(extra->name, "Video1IntrRead")) {/*video remote allocate*/
			spin_lock_bh(&extra->lock);
			update_nextRpc(extra, out);
			update_currProc(extra, remote_proc);
			vcpu_r_program_flag = 1;
			wake_up_interruptible(&vcpu_r_program_waitQueue);
			spin_unlock_bh(&extra->lock);
		}
#ifdef CONFIG_REALTEK_RPC_VE3
		else if (!strcmp(extra->name, "VE3IntrRead")) {/*hifi remote allocate*/
			spin_lock_bh(&extra->lock);
			update_nextRpc(extra, out);
			update_currProc(extra, remote_proc);
			ve3_r_program_flag = 1;
			wake_up_interruptible(&ve3_r_program_waitQueue);
			spin_unlock_bh(&extra->lock);
		}
#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
		else if (!strcmp(extra->name, "HIFIIntrRead")) {/*video remote allocate*/
			spin_lock_bh(&extra->lock);
			update_nextRpc(extra, out);
			update_currProc(extra, remote_proc);
			hifi_r_program_flag = 1;
			wake_up_interruptible(&hifi_r_program_waitQueue);
			spin_unlock_bh(&extra->lock);
		}
#endif
		else {
			pr_err("[%s] Unknow Request Remote Allocate\n", __func__);
		}

		return;
	case VIDEO_AGENT:
#ifdef CONFIG_RPC_KERN_VE2
		proc = ve2_proc;
		pr_err("%s:%d  proc:%p\n", __func__, __LINE__, proc);
		break;
#endif
	case AUDIO_AGENT:
	case VENC_AGENT:
	case HIFI_AGENT:
		proc = NULL;
		/* use sysPID directly */
		if (rpc.sysPID > 0 && rpc.sysPID < PID_MAX_DEFAULT) {
			pr_info("lookup by sysPID:%d\n", rpc.sysPID);
			taskID = rpc.sysPID;
#ifdef RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID
		/* lookup pid by sysTID */
		} else if (rpc.sysTID > 0 && rpc.sysTID < PID_MAX_DEFAULT) {
			pr_info("lookup by sysTID:%d\n", rpc.sysTID);
			taskID = rpc.sysTID;
		}
#endif /* RPC_SUPPORT_MULTI_CALLER_SEND_TID_PID */
		else {
			pr_err("PID is out of range: sysPID:%d   sysTID:%d\n", rpc.sysPID, rpc.sysTID);
			return;
		}
		proc = lookup_by_taskID(extra, taskID);
		if (unlikely(proc == NULL)) {
#ifdef CONFIG_REALTEK_RPC_PROGRAM_REGISTER
			proc = pick_supported_proc(extra, rpc.programID);
#else
			proc = pick_one_proc(extra);
#endif
		}
		if (proc == NULL || check_dead_process(taskID) == 0) {
			pr_err("[%s] cannot find process by pid(%d)", __func__, taskID);
			handle_dead_process_reply(rpc, extra);
			proc = NULL;
		}
		break;
	case REPLYID:
		if (rpc.versionID == REPLYID &&
				rpc.parameterSize >= sizeof(uint32_t)) {
			uint32_t taskID;
			if (get_ring_data(extra->name, extra, out, (char *) &taskID,
					sizeof(uint32_t)) == 0)
				return;
#ifdef CONFIG_REALTEK_RPC_HIFI
			if (strcmp(extra->name, "HIFIIntrRead"))
				taskID = ntohl(taskID);

#else
			taskID = ntohl(taskID);
#endif
#ifdef CONFIG_RPC_KERN_VE2
			if(taskID == 0xffffffff) {
				proc = ve2_proc;
				break;
			}
#endif
			pr_debug("%s:%d taskID:%u\n", extra->name, __LINE__, taskID);
			proc = lookup_by_taskID(extra, taskID);
			if (proc == NULL || check_dead_process(proc->pid) == 0) {
				pr_err("%s: ignore RPC!! clear %s(%p) current process\n",
							__func__, extra->name, dev);
				ignore_rpc(extra, rpc.parameterSize);
				proc = NULL;
			}
		}
		break;
	default:
		if (in_atomic() && __ratelimit(&ring_dump_state)) {
			pr_err("%s:%d invalid programID:%u!!! Out:%x next:%x In:%x\n",
					__func__, __LINE__, rpc.programID, ringOut, nextRpc,
					ringIn);
			show_rpc_struct(extra->name, &rpc);
			dump_ring_buffer(extra->name, extra);
		}
		return;
	}

	if (proc) {
		found = 1;
	} else if (__ratelimit(&ring_dump_state)) {
		pr_err("%s:%d can't find process for handling %s programID:%u\n",
				extra->name, __LINE__, extra->name, rpc.programID);
		show_rpc_struct(extra->name, &rpc);
	}

	out += rpc.parameterSize;
	if (out >= *ringEnd)
		out = *ringStart + (out - *ringEnd);

	spin_lock_bh(&extra->lock);
	update_nextRpc(extra, out);

	if (found) {
		update_currProc(extra, proc);
		wake_up_interruptible(&proc->waitQueue);
		pr_debug("%s:%d ###Wakeup### proc:%p(%d) process:%p(%d) and \
				update nextRpc:%x for programID:%u\n",
				extra->name, __LINE__, proc, proc ? proc->pid : 0,
				extra->currProc,
				extra->currProc ? ((RPC_PROCESS *)extra->currProc)->pid : 0,
				extra->nextRpc, rpc.programID);
	} else {
		update_currProc(extra, NULL);
		if (need_dispatch(extra)) {
			tasklet_schedule(&(extra->tasklet));
		}
	}

	spin_unlock_bh(&extra->lock);
}

int rpc_intr_init(void)
{
	static int is_init;
	int result = 0, i;
	int j = 0;
	is_init = 0;

	/* Create corresponding structures for each device. */
	rpc_intr_devices = (RPC_DEV *) AVCPU2SCPU(RPC_INTR_RECORD_ADDR);

	for (i = 0; i < RPC_INTR_DEV_TOTAL; i++) {
		pr_debug("rpc_intr_device %d addr: %p\n", i, &rpc_intr_devices[i]);
		rpc_intr_devices[i].ringBuf = RPC_INTR_DEV_ADDR +
				i * RPC_RING_SIZE * 2;

		/* Initialize pointers... */
		rpc_intr_devices[i].ringStart = rpc_intr_devices[i].ringBuf;
		rpc_intr_devices[i].ringEnd = rpc_intr_devices[i].ringBuf
				+ RPC_RING_SIZE;
		rpc_intr_devices[i].ringIn = rpc_intr_devices[i].ringBuf;
		rpc_intr_devices[i].ringOut = rpc_intr_devices[i].ringBuf;

		pr_debug("The %dth intr dev:\n", i);
		pr_debug("RPC ringStart: %p\n",
				AVCPU2SCPU(rpc_intr_devices[i].ringStart));
		pr_debug("RPC ringEnd:   %p\n",
				AVCPU2SCPU(rpc_intr_devices[i].ringEnd));
		pr_debug("RPC ringIn:    %p\n",
				AVCPU2SCPU(rpc_intr_devices[i].ringIn));
		pr_debug("RPC ringOut:   %p\n",
				AVCPU2SCPU(rpc_intr_devices[i].ringOut));

		rpc_intr_extra[i].nextRpc = rpc_intr_devices[i].ringOut;
		rpc_intr_extra[i].currProc = NULL;
		rpc_intr_extra[i].serialno = i;

		if (!is_init) {
			rpc_intr_devices[i].ptrSync = kmalloc(sizeof(RPC_SYNC_Struct),
					GFP_KERNEL);
			kmemleak_not_leak(rpc_intr_devices[i].ptrSync);

			/* Initialize wait queue... */
			init_waitqueue_head(&(rpc_intr_devices[i].ptrSync->waitQueue));

			/* Initialize sempahores... */
			init_rwsem(&rpc_intr_devices[i].ptrSync->readSem);
			init_rwsem(&rpc_intr_devices[i].ptrSync->writeSem);

			rpc_intr_extra[i].dev = (void *) &rpc_intr_devices[i];
			INIT_LIST_HEAD(&rpc_intr_extra[i].tasks);
			tasklet_init(&rpc_intr_extra[i].tasklet, rpc_dispatch,
					(unsigned long) &rpc_intr_extra[i]);
			spin_lock_init(&rpc_intr_extra[i].lock);
			switch (i) {
			case 0:
				rpc_intr_extra[i].name = "AudioIntrWrite";
				break;
			case 1:
				rpc_intr_extra[i].name = "AudioIntrRead";
				break;
			case 2:
				rpc_intr_extra[i].name = "Video1IntrWrite";
				break;
			case 3:
				rpc_intr_extra[i].name = "Video1IntrRead";
				break;
			case 4:
				rpc_intr_extra[i].name = "Video2IntrWrite";
				break;
			case 5:
				rpc_intr_extra[i].name = "Video2IntrRead";
				break;
			}
		}
	}

#ifdef CONFIG_REALTEK_RPC_VE3

	rpc_intr_ve3_devices = (RPC_DEV *) AVCPU2SCPU(RPC_INTR_VE3_RECORD_ADDR);

	for (i = 0; i < RPC_INTR_VE3_DEV_TOTAL; i++) {
		pr_debug("rpc_intr_ve3_device %d addr: %p\n", i, &rpc_intr_ve3_devices[i]);
			rpc_intr_ve3_devices[i].ringBuf = RPC_INTR_VE3_DEV_ADDR +
		i * RPC_RING_SIZE;

		/* Initialize pointers... */
		rpc_intr_ve3_devices[i].ringStart = rpc_intr_ve3_devices[i].ringBuf;
		rpc_intr_ve3_devices[i].ringEnd = rpc_intr_ve3_devices[i].ringBuf
											+ RPC_RING_SIZE;
		rpc_intr_ve3_devices[i].ringIn = rpc_intr_ve3_devices[i].ringBuf;
		rpc_intr_ve3_devices[i].ringOut = rpc_intr_ve3_devices[i].ringBuf;

		pr_debug("The %dth intr dev:\n", i + RPC_INTR_DEV_TOTAL);
		pr_debug("RPC ringStart: %p\n",
				AVCPU2SCPU(rpc_intr_ve3_devices[i].ringStart));
		pr_debug("RPC ringEnd:   %p\n",
				AVCPU2SCPU(rpc_intr_ve3_devices[i].ringEnd));
		pr_debug("RPC ringIn:    %p\n",
				AVCPU2SCPU(rpc_intr_ve3_devices[i].ringIn));
		pr_debug("RPC ringOut:   %p\n",
				AVCPU2SCPU(rpc_intr_ve3_devices[i].ringOut));

		rpc_intr_extra[i + RPC_INTR_DEV_TOTAL].nextRpc = rpc_intr_ve3_devices[i].ringOut;
		rpc_intr_extra[i + RPC_INTR_DEV_TOTAL].currProc = NULL;
		rpc_intr_extra[i + RPC_INTR_DEV_TOTAL].serialno = i + RPC_INTR_DEV_TOTAL;
		if (!is_init) {
			rpc_intr_ve3_devices[i].ptrSync = kmalloc(sizeof(RPC_SYNC_Struct),
					GFP_KERNEL);
			kmemleak_not_leak(rpc_intr_ve3_devices[i].ptrSync);

			/* Initialize wait queue... */
			init_waitqueue_head(&(rpc_intr_ve3_devices[i].ptrSync->waitQueue));

			/* Initialize sempahores... */
			init_rwsem(&rpc_intr_ve3_devices[i].ptrSync->readSem);
			init_rwsem(&rpc_intr_ve3_devices[i].ptrSync->writeSem);

			rpc_intr_extra[i + RPC_INTR_DEV_TOTAL].dev = (void *) &rpc_intr_ve3_devices[i];
			INIT_LIST_HEAD(&rpc_intr_extra[i + RPC_INTR_DEV_TOTAL].tasks);
			tasklet_init(&rpc_intr_extra[i + RPC_INTR_DEV_TOTAL].tasklet, rpc_dispatch,
					(unsigned long) &rpc_intr_extra[i + RPC_INTR_DEV_TOTAL]);
			spin_lock_init(&rpc_intr_extra[i + RPC_INTR_DEV_TOTAL].lock);
			switch (i) {
			case 0:
				rpc_intr_extra[i + RPC_INTR_DEV_TOTAL].name = "VE3IntrWrite";
				break;
			case 1:
				rpc_intr_extra[i + RPC_INTR_DEV_TOTAL].name = "VE3IntrRead";
				break;
			}
		}
	}

#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
	rpc_intr_hifi_devices = (RPC_HIFI_DEV *) AVCPU2SCPU(RPC_INTR_HIFI_RECORD_ADDR);

	for (i = 0; i < RPC_INTR_HIFI_DEV_TOTAL; i++) {
		pr_debug("rpc_intr_hifi_devices %d addr: %p\n", i, &rpc_intr_hifi_devices[i]);
							rpc_intr_hifi_devices[i].ringBuf = RPC_INTR_HIFI_DEV_ADDR +
							i * RPC_RING_SIZE;

			/* Initialize pointers... */
		rpc_intr_hifi_devices[i].ringStart = rpc_intr_hifi_devices[i].ringBuf;
		rpc_intr_hifi_devices[i].ringEnd = rpc_intr_hifi_devices[i].ringBuf
																+ RPC_RING_SIZE;
		rpc_intr_hifi_devices[i].ringIn = rpc_intr_hifi_devices[i].ringBuf;
		rpc_intr_hifi_devices[i].ringOut = rpc_intr_hifi_devices[i].ringBuf;

		pr_debug("The %dth intr dev:\n", i + RPC_INTR_DEV_TOTAL + RPC_INTR_VE3_DEV_TOTAL);
		pr_debug("RPC ringStart: %p\n", AVCPU2SCPU(rpc_intr_hifi_devices[i].ringStart));
		pr_debug("RPC ringEnd:	 %p\n", AVCPU2SCPU(rpc_intr_hifi_devices[i].ringEnd));
		pr_debug("RPC ringIn:	 %p\n", AVCPU2SCPU(rpc_intr_hifi_devices[i].ringIn));
		pr_debug("RPC ringOut:	 %p\n", AVCPU2SCPU(rpc_intr_hifi_devices[i].ringOut));
		rpc_intr_extra[i + RPC_INTR_DEV_TOTAL + RPC_INTR_VE3_DEV_TOTAL].nextRpc = rpc_intr_hifi_devices[i].ringOut;
		rpc_intr_extra[i + RPC_INTR_DEV_TOTAL + RPC_INTR_VE3_DEV_TOTAL].currProc = NULL;
		rpc_intr_extra[i + RPC_INTR_DEV_TOTAL + RPC_INTR_VE3_DEV_TOTAL].serialno = i + RPC_INTR_DEV_TOTAL + RPC_INTR_VE3_DEV_TOTAL;

		if (!is_init) {
			rpc_intr_hifi_devices[i].ptrSync = kmalloc(sizeof(RPC_SYNC_Struct),
									GFP_KERNEL);
			kmemleak_not_leak(rpc_intr_hifi_devices[i].ptrSync);

			/* Initialize wait queue... */
			init_waitqueue_head(&(rpc_intr_hifi_devices[i].ptrSync->waitQueue));

			/* Initialize sempahores... */
			init_rwsem(&rpc_intr_hifi_devices[i].ptrSync->readSem);
			init_rwsem(&rpc_intr_hifi_devices[i].ptrSync->writeSem);

			rpc_intr_extra[i + RPC_INTR_DEV_TOTAL + RPC_INTR_VE3_DEV_TOTAL].hifi_dev = (void *) &rpc_intr_hifi_devices[i];
			INIT_LIST_HEAD(&rpc_intr_extra[i + RPC_INTR_DEV_TOTAL + RPC_INTR_VE3_DEV_TOTAL].tasks);
			tasklet_init(&rpc_intr_extra[i + RPC_INTR_DEV_TOTAL + RPC_INTR_VE3_DEV_TOTAL].tasklet, rpc_dispatch,
						(unsigned long) &rpc_intr_extra[i + RPC_INTR_DEV_TOTAL + RPC_INTR_VE3_DEV_TOTAL]);
			spin_lock_init(&rpc_intr_extra[i + RPC_INTR_DEV_TOTAL + RPC_INTR_VE3_DEV_TOTAL].lock);
			switch (i) {
			case 0:
				rpc_intr_extra[i + RPC_INTR_DEV_TOTAL + RPC_INTR_VE3_DEV_TOTAL].name = "HIFIIntrWrite";
				break;
			case 1:
				rpc_intr_extra[i + RPC_INTR_DEV_TOTAL + RPC_INTR_VE3_DEV_TOTAL].name = "HIFIIntrRead";
				break;
			}
		}
	}
#endif
	init_waitqueue_head(&acpu_r_program_waitQueue);
	acpu_r_program_kthread = kthread_run(acpu_remote_alloc_thread, (void *)&j, "acpu_r_program");


	init_waitqueue_head(&vcpu_r_program_waitQueue);
	vcpu_r_program_kthread = kthread_run(vcpu_remote_alloc_thread, (void *)&j, "vcpu_r_program");

#ifdef CONFIG_REALTEK_RPC_VE3
	init_waitqueue_head(&ve3_r_program_waitQueue);
	ve3_r_program_kthread = kthread_run(ve3_remote_alloc_thread, (void *)&j, "ve3_r_program");

#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
	init_waitqueue_head(&hifi_r_program_waitQueue);
	hifi_r_program_kthread = kthread_run(hifi_remote_alloc_thread, (void *)&j, "hifi_r_program");
#endif

	is_init = 1;
	rpc_intr_is_paused = 0;
	rpc_intr_is_suspend = 0;

	remote_proc = kmalloc(sizeof(RPC_PROCESS), GFP_KERNEL | __GFP_ZERO);
	remote_proc->pid = 0;

	INIT_LIST_HEAD(&rpc_release_lists.list);
	INIT_LIST_HEAD(&release_proc_lists.list);

	dev_info(rpc_dev, "\033[31mrpc is not paused & suspended\033[m\n");

	return result;
}

int rpc_intr_pause(void)
{
	rpc_intr_is_paused = 1;
	pr_info("\033[31mrpc is paused\033[m\n");
	return 0;
}

int rpc_intr_suspend(void)
{
	rpc_intr_is_suspend = 1;
	pr_info("\033[31mrpc is suspended\033[m\n");
	return 0;
}

int rpc_intr_resume(void)
{
	rpc_intr_is_suspend = 0;
	pr_info("\033[31mrpc is not suspended\033[m\n");
	return 0;
}

void rpc_intr_cleanup(void)
{
	int i;

	/* Clean corresponding structures for each device. */
	if (rpc_intr_devices) {
		/* Clean ring buffers. */
		for (i = 0; i < RPC_INTR_DEV_TOTAL; i++) {
			//if (rpc_intr_devices[i].ringBuf)
				//kfree(rpc_intr_devices[i].ringBuf);
			kfree(rpc_intr_devices[i].ptrSync);
		}
		//kfree(rpc_intr_devices);
	}

	return;
}

#define AUDIO_ION_FLAG              (ION_FLAG_NONCACHED |ION_FLAG_SCPUACC | ION_FLAG_ACPUACC)


int update_rpc_release_lists(int type)
{
	struct RPC_RELEASE_LIST *release_entry;
	struct list_head *listptr;
	struct RPC_RELEASE_LIST *tmp_entry;
	int ret = 0;

	spin_lock_bh(&rpc_release_lock);
	list_for_each(listptr, &rpc_release_lists.list) {
		tmp_entry = list_entry(listptr, struct RPC_RELEASE_LIST, list);
		if (tmp_entry->pid == current->tgid && tmp_entry->type == type){
			spin_unlock_bh(&rpc_release_lock);
			return 0;
		}
	}
	spin_unlock_bh(&rpc_release_lock);

	release_entry = kmalloc(sizeof(struct RPC_RELEASE_LIST), GFP_KERNEL);
	if (!release_entry) {
		pr_err("[%s] kmalloc failed\n", __func__);
		ret = -ENOMEM;
		goto free_entry;
	}
	strcpy(release_entry->comm, current->comm);

	release_entry->pid = current->tgid;
	release_entry->type = type;

	release_entry->rpc_dmabuf = ion_alloc(4096, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);

	release_entry->paddr = release_rpc_ion_pa(release_entry);
	if (release_entry->paddr < 0) {
		pr_err("[%s] get paddr failed\n", __func__);
		ret = -EINVAL;
		goto put_dma_buf;
	}

	release_entry->vaddr = dma_buf_vmap(release_entry->rpc_dmabuf);
	if (release_entry->vaddr == NULL) {
		pr_err("[%s] get vaddr failed\n", __func__);
		ret = -EINVAL;
		goto detach_dma_buf;
	}
	spin_lock_bh(&rpc_release_lock);
	list_add(&release_entry->list, &rpc_release_lists.list);
	spin_unlock_bh(&rpc_release_lock);

	return 0;

detach_dma_buf:
	dma_buf_unmap_attachment(release_entry->attachment, release_entry->table, DMA_BIDIRECTIONAL);
	dma_buf_detach(release_entry->rpc_dmabuf, release_entry->attachment);
put_dma_buf:
	dma_buf_put(release_entry->rpc_dmabuf);
free_entry:
	kfree(release_entry);

	return ret;
}

struct RPC_RELEASE_LIST *rpc_find_release_list(int type, int pid){

	struct list_head *listptr;
	struct RPC_RELEASE_LIST *entry;

	list_for_each(listptr, &rpc_release_lists.list) {
		entry = list_entry(listptr, struct RPC_RELEASE_LIST, list);
		if (entry->pid == pid && entry->type == type) {
			list_del(listptr);
			return entry;
		}
	}
	return NULL;
}

int rpc_intr_open(struct inode *inode, struct file *filp)
{
	int minor = MINOR(inode->i_rdev);
	int rpc_type = minor / RPC_NR_PAIR;
	int ret = 0;

	pr_debug("RPC intr open with minor number: %d\n", minor);

	if (!filp->private_data) {
		RPC_PROCESS *proc = kmalloc(sizeof(RPC_PROCESS), GFP_KERNEL | __GFP_ZERO);
		if (proc == NULL) {
			pr_err("%s: failed to allocate RPC_PROCESS", __func__);
			return -ENOMEM;
		}

		if (minor < 8) {
			proc->extra = &rpc_intr_extra[minor / RPC_NR_PAIR];
			proc->dev = proc->extra->dev;
		}
#ifdef CONFIG_REALTEK_RPC_VE3
		if (minor == 8) {
			proc->extra = &rpc_intr_extra[4];
			proc->dev = proc->extra->dev;
		} else if (minor == 9) {
			proc->extra = &rpc_intr_extra[5];
			proc->dev = proc->extra->dev;
		}
#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
		if (minor == 10) {
			proc->extra = &rpc_intr_extra[6];
			proc->hifi_dev = proc->extra->hifi_dev;
		} else if (minor == 11) {
			proc->extra = &rpc_intr_extra[7];
			proc->hifi_dev = proc->extra->hifi_dev;
		}

#endif
		/* current->tgid = process id, current->pid = thread id */
		proc->pid = current->tgid;
		proc->bStayActive = false;
		init_waitqueue_head(&proc->waitQueue);
		INIT_LIST_HEAD(&proc->threads);
#ifdef CONFIG_REALTEK_RPC_PROGRAM_REGISTER
		INIT_LIST_HEAD(&proc->handlers);
#endif
		spin_lock_bh(&proc->extra->lock);
		list_add(&proc->list, &proc->extra->tasks);
		spin_unlock_bh(&proc->extra->lock);
		pr_debug("%s: Current process pid:%d tgid:%d => %d(%p) for %s(%p)\n",
				__func__, current->pid, current->tgid, proc->pid,
				&proc->waitQueue, proc->extra->name, proc->dev);

		filp->private_data = proc;
		filp->f_pos = (loff_t) minor;
		if (rpc_type == 1)
			ret = update_rpc_release_lists(RPC_AUDIO);
		else if (rpc_type == 3)
			ret = update_rpc_release_lists(RPC_VIDEO);
#ifdef CONFIG_REALTEK_RPC_VE3
		else if (rpc_type == 4)
			ret = update_rpc_release_lists(RPC_VE3);
#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
		else if (rpc_type == 5) {
			ret = update_rpc_release_lists(RPC_HIFI);
		}
#endif

		if (ret) {
			pr_err("%s: update_rpc_release_lists failed err:%d\n", __func__, ret);
			return ret;
		}
	}

	/* Before we maybe sleep */
	//MOD_INC_USE_COUNT;

	return 0; /* success */
}

int RPC_DESTROY_AUDIO_FLOW(int pid, phys_addr_t paddr, void *vaddr)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *cmd = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	uint32_t RPC_ret;
	int ret = -1;
	phys_addr_t dat;
	unsigned int offset;

	cmd = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)cmd + offset);
	memset_io(cmd, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	cmd->instanceID = htonl(-1);
	cmd->type = htonl(ENUM_PRIVATEINFO_AUDIO_DO_SELF_DESTROY_FLOW);
	cmd->privateInfo[0] = htonl(pid);

	if (send_rpc_command(RPC_AUDIO,
				ENUM_KERNEL_RPC_PRIVATEINFO,
				dat, dat + offset,
				&RPC_ret)) {
		pr_err("[RPC] %s send RPC failed!\n", __func__);
	}

	if (RPC_ret != S_OK) {
		pr_err("[RPC] %s received RPC failed!\n", __func__);
		goto exit;
	}

exit:

	return ret;
}

int RPC_DESTROY_VIDEO_FLOW(int pid, phys_addr_t paddr, void *vaddr)
{
	uint32_t *pid_info;
	uint32_t *res;
	uint32_t RPC_ret;
	int ret = -1;
	phys_addr_t dat;
	unsigned int offset;

	pid_info = (uint32_t *)vaddr;
	dat = paddr;
	offset = get_rpc_alignment_offset(sizeof(uint32_t));
	res = (uint32_t *)(pid_info + offset);
	memset_io(pid_info, 0, sizeof(uint32_t));
	*pid_info =  htonl(pid);

	if (send_rpc_command(RPC_VIDEO,
				170,
				dat, dat + offset,
				&RPC_ret)) {
		pr_err("[RPC] %s send RPC failed!\n", __func__);
	}

	if (RPC_ret != S_OK) {
		pr_err("[RPC] %s received RPC failed!\n", __func__);
		goto exit;
	}
exit:

	return ret;
}

int rpc_intr_release(struct inode *inode, struct file *filp)
{
#ifdef CONFIG_REALTEK_RPC_PROGRAM_REGISTER
	RPC_HANDLER *handler, *hdltmp;
#endif /* CONFIG_REALTEK_RPC_PROGRAM_REGISTER */
	RPC_THREAD *th, *thtmp;
	int minor = MINOR(inode->i_rdev);

	RPC_PROCESS *proc = filp->private_data;
	RPC_DEV_EXTRA *extra = proc->extra;

	spin_lock_bh(&extra->lock);

#ifdef CONFIG_REALTEK_RPC_PROGRAM_REGISTER
	/* unregister myself from handler list */
	list_for_each_entry_safe(handler, hdltmp, &proc->handlers, list) {
		list_del(&handler->list);
		kfree(handler);
	}
#endif /* CONFIG_REALTEK_RPC_PROGRAM_REGISTER */

	list_for_each_entry_safe(th, thtmp, &proc->threads, list) {
		list_del(&th->list);
		kfree(th);
	}

	/* remove myself from task list*/
	list_del(&proc->list);
	kfree(proc);

	spin_unlock_bh(&extra->lock);

	pr_debug("RPC intr close with minor number: %d\n", minor);

	//MOD_DEC_USE_COUNT;

	return 0;
}

static int rpc_intr_flush(struct file *filp, fl_owner_t id)
{

	struct RPC_RELEASE_LIST *rpc_release_entry = NULL;

	int minor = MINOR(filp->f_inode->i_rdev);
	int rpc_type = minor / RPC_NR_PAIR;

	RPC_PROCESS *proc = filp->private_data;
	RPC_DEV_EXTRA *extra = proc->extra;
	struct task_struct *task;
	volatile uint32_t *ringOut;
	RPC_SYNC_Struct *ptrSync;

#ifdef CONFIG_REALTEK_RPC_HIFI
	if (extra->serialno > 5) {
		ringOut = &extra->hifi_dev->ringOut;
		ptrSync = extra->hifi_dev->ptrSync;
	} else {
		ringOut = &extra->dev->ringOut;
		ptrSync = extra->dev->ptrSync;
	}
#else
	ringOut = &extra->dev->ringOut;
	ptrSync = extra->dev->ptrSync;
#endif

	if (file_count(filp) > 1)
		return 0;

	add_release_list(proc->pid, rpc_type);

	if (extra->currProc == proc) {
		pr_debug("%s: clear %s current process\n", __func__,
				proc->extra->name);
		update_currProc(extra, NULL);
		 /* intr read device (ugly code)*/
		if (minor == 3 || minor == 7 || minor == 9) {
			if (!rpc_done(extra)) {
				pr_debug("%s: previous rpc hasn't finished, force clear!! ringOut %p => %p\n",
						__func__, AVCPU2SCPU(*ringOut),
						AVCPU2SCPU(extra->nextRpc));
				down_write(&ptrSync->readSem);
				*ringOut = extra->nextRpc;
				up_write(&ptrSync->readSem);
			}
		}
	}

	spin_lock_bh(&rpc_release_lock);
	if (rpc_type == 1)
		rpc_release_entry = rpc_find_release_list(RPC_AUDIO, proc->pid);
	else if (rpc_type == 3)
		rpc_release_entry = rpc_find_release_list(RPC_VIDEO, proc->pid);
#ifdef CONFIG_REALTEK_RPC_VE3
	else if (rpc_type == 4)
		rpc_release_entry = rpc_find_release_list(RPC_VE3, proc->pid);
#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
	else if (rpc_type == 5)
		rpc_release_entry = rpc_find_release_list(RPC_HIFI, proc->pid);
#endif

	spin_unlock_bh(&rpc_release_lock);
	if (rpc_intr_is_paused) {
		pr_err("rpc is paused, no self destroy: %d\n", proc->pid);
	} else if (proc->bStayActive) {
		pr_err("bStayActive is true, no self destroy: %d\n", proc->pid);
	} else {
		task = get_pid_task(find_pid_ns(proc->pid, &init_pid_ns), PIDTYPE_PID);
		if (task != NULL && (task->flags & PF_SIGNALED))
			task = NULL;

		if (rpc_release_entry!=NULL && task == NULL) {
			pr_err("self destroy in flush: %d extra->name:%s\n", proc->pid, extra->name);
			if(!strcmp(extra->name, "AudioIntrRead")) {
				RPC_DESTROY_AUDIO_FLOW(proc->pid, rpc_release_entry->paddr, rpc_release_entry->vaddr);

			}
#if !defined(CONFIG_RPC_KERN_VE2)
			else if (!strcmp(extra->name, "Video1IntrRead")) {
				RPC_DESTROY_VIDEO_FLOW(proc->pid, rpc_release_entry->paddr, rpc_release_entry->vaddr);
			}
#endif
		}
	}
	if (rpc_release_entry!=NULL) {
		release_rpc_free_ion(rpc_release_entry);
		dma_buf_put(rpc_release_entry->rpc_dmabuf);
		kfree(rpc_release_entry);
	}


	rm_release_list(proc->pid);

	return 0;
}

/*
 * We don't need parameter f_pos here...
 * note:rpc_intr_read support both blocking & nonblocking modes
 */
ssize_t rpc_intr_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	RPC_PROCESS *proc = filp->private_data;
	RPC_DEV_EXTRA *extra = proc->extra;
	int temp, size;
	size_t r;
	ssize_t ret = 0;
	uint32_t ptmp;
	int rpc_ring_size;
	long k;
	volatile uint32_t *ringStart, *ringEnd, *ringIn, *ringOut;
	RPC_SYNC_Struct *ptrSync;

#ifdef CONFIG_REALTEK_RPC_HIFI
	RPC_HIFI_DEV *hifi_dev = extra->hifi_dev;
	RPC_DEV *dev = extra->dev;

	if (extra->serialno > 5) {
		ringStart = &hifi_dev->ringStart;
		ringEnd = &hifi_dev->ringEnd;
		ringIn = &hifi_dev->ringIn;
		ringOut = &hifi_dev->ringOut;
		ptrSync = hifi_dev->ptrSync;
		rpc_ring_size = *ringEnd - *ringStart;
	} else {
		ringStart = &dev->ringStart;
		ringEnd = &dev->ringEnd;
		ringIn = &dev->ringIn;
		ringOut = &dev->ringOut;
		ptrSync = dev->ptrSync;
		rpc_ring_size = *ringEnd - *ringStart;
	}
#else
	RPC_DEV *dev = extra->dev;

	ringStart = &dev->ringStart;
	ringEnd = &dev->ringEnd;
	ringIn = &dev->ringIn;
	ringOut = &dev->ringOut;
	ptrSync = dev->ptrSync;
	rpc_ring_size = *ringEnd - *ringStart;
#endif

	//pr_debug("%s:%d thread:%s pid:%d tgid:%d device:%s\n",
	//__func__, __LINE__, current->comm, current->pid, current->tgid,
	//extra->name);
	if (rpc_intr_is_paused) {
		pr_err("RPCintr: someone access rpc intr during the pause...\n");
		pr_err("%s:%d buf:%p count:%lu EAGAIN\n", extra->name, __LINE__, buf,
				count);
		msleep(1000);
		return -EAGAIN;
	}

	if (need_dispatch(extra))
		tasklet_schedule(&(extra->tasklet));
	//pr_debug("%s: dev:%p currProc:%p\n",
		//extra->name, dev, extra->currProc);
	if ((extra->currProc != proc) || (ring_empty(extra))) {
		if (filp->f_flags & O_NONBLOCK)
			goto done;

wait_again:

		k = wait_event_interruptible_timeout(proc->waitQueue,
			(((extra->currProc == proc) && (!ring_empty(extra))) || proc->bExit),
			timeout);
		if (k == 0)
			goto done; /* timeout */

		//if (current->flags & PF_FREEZE) {
		//refrigerator(PF_FREEZE);
		//if (!signal_pending(current))
			//goto wait_again;
		//}

		if (try_to_freeze()) {
			if (!signal_pending(current))
				goto wait_again;
		}

		if (signal_pending(current)) {
			pr_debug("RPC deblock because of receiving a signal...\n");
			goto done;
		}

		if (proc->bExit) {
			pr_info("user request to exit\n");
			goto done;
		}
	}

	down_write(&ptrSync->readSem);

	if (*ringIn > *ringOut)
		size = *ringIn - *ringOut;
	else
		size = rpc_ring_size + *ringIn - *ringOut;

	pr_debug("%s:%d ==going read== count:%zu avail:%d "
			"ringOut:%x ringIn:%x nextRPC:%x",
			extra->name, __LINE__, count, size, *ringOut, *ringIn,
			extra->nextRpc);
	peek_rpc_struct(__func__, extra, *f_pos);

	if (count > size)
		count = size;

	temp = *ringEnd - *ringOut;
	wmb();

	if (temp >= count) {
#ifdef MY_COPY
		r = my_copy_to_user((int *)buf,
				(int *)AVCPU2SCPU(*ringOut), count);
#else
		r = copy_to_user((int *)buf, (int *)AVCPU2SCPU(*ringOut), count);
#endif /* MY_COPY */
		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n",
				extra->name, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}

		ret = count;
		ptmp = *ringOut + ((count+3) & 0xfffffffc);
		if (ptmp == *ringEnd)
			*ringOut = *ringStart;
		else
			*ringOut = ptmp;

		//pr_debug("RPC Read is in 1st kind...\n");
	} else {
#ifdef MY_COPY
		r = my_copy_to_user((int *)buf,
				(int *)AVCPU2SCPU(*ringOut), temp);
#else
		r = copy_to_user((int *)buf,
				(int *)AVCPU2SCPU(*ringOut), temp);
#endif /* MY_COPY */

		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n",
					extra->name, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}

		count -= temp;

#ifdef MY_COPY
		r = my_copy_to_user((int *)(buf+temp),
				(int *)AVCPU2SCPU(*ringStart), count);
#else
		r = copy_to_user((int *)(buf+temp),
				(int *)AVCPU2SCPU(*ringStart), count);
#endif /* MY_COPY */

		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n",
					extra->name, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}
		ret = (temp + count);
		*ringOut = *ringStart+((count+3) & 0xfffffffc);

		//pr_debug("RPC Read is in 2nd kind...\n");
	}

	/*
	 * NOTE: we do not need spin lock here because we are protected by
	 * semaphore already
	 */
	spin_lock_bh(&extra->lock);
	if (rpc_done(extra) && extra->currProc == proc) {
		pr_debug("%s: Previous RPC is done, unregister myself\n", extra->name);
		update_currProc(extra, NULL);
	}
	spin_unlock_bh(&extra->lock);

	/* process next rpc command if any */
	if (need_dispatch(extra))
		tasklet_schedule(&(extra->tasklet));

	pr_debug("%s:%d pid:%d tgid:%d count:%lu actual:%ld ringOut:%x "
			"ringIn:%x nextRpc:%x currProc:%p(%d)\n",
			extra->name, __LINE__, current->pid, current->tgid, count, ret,
			*ringOut, *ringIn, extra->nextRpc, extra->currProc,
			extra->currProc ? ((RPC_PROCESS *)extra->currProc)->pid : 0);
out:
		up_write(&ptrSync->readSem);
done:
		//pr_debug("RPC intr ringOut pointer is : 0x%8x\n",
		//		(int)AVCPU2SCPU(dev->ringOut));
		//pr_debug("%s:%d pid:%d reads %d bytes\n",
		//		extra->name, __LINE__, current->pid, ret);
		return ret;
}

/*
 * We don't need parameter f_pos here...
 * note: rpc_intr_write only support nonblocking mode
 */
ssize_t rpc_intr_write(struct file *filp, const char *buf, size_t count,
		loff_t *f_pos)
{
	RPC_PROCESS *proc = filp->private_data;
	RPC_DEV_EXTRA *extra = proc->extra;
	RPC_DEV_EXTRA *rextra = extra + 1;
	int temp, size;
	size_t r;
	ssize_t ret = 0;
	uint32_t ptmp;
	int rpc_ring_size;
	volatile uint32_t *ringStart, *ringEnd, *ringIn, *ringOut;
	RPC_SYNC_Struct *ptrSync;

#ifdef CONFIG_REALTEK_RPC_HIFI
	RPC_HIFI_DEV *hifi_dev = extra->hifi_dev;
	RPC_DEV *dev = extra->dev;

	if (extra->serialno > 5) {
		ringStart = &hifi_dev->ringStart;
		ringEnd = &hifi_dev->ringEnd;
		ringIn = &hifi_dev->ringIn;
		ringOut = &hifi_dev->ringOut;
		ptrSync = hifi_dev->ptrSync;
		rpc_ring_size = *ringEnd - *ringStart;
	} else {
		ringStart = &dev->ringStart;
		ringEnd = &dev->ringEnd;
		ringIn = &dev->ringIn;
		ringOut = &dev->ringOut;
		ptrSync = dev->ptrSync;
		rpc_ring_size = *ringEnd - *ringStart;
	}
#else
	RPC_DEV *dev = extra->dev;

	ringStart = &dev->ringStart;
	ringEnd = &dev->ringEnd;
	ringIn = &dev->ringIn;
	ringOut = &dev->ringOut;
	ptrSync = dev->ptrSync;
	rpc_ring_size = *ringEnd - *ringStart;
#endif


	//pr_debug("%s:%d buf:%p count:%u\n", __func__, __LINE__, buf, count);

	if (rpc_intr_is_paused) {
		pr_err("RPCintr: someone access rpc intr during the pause...\n");
		pr_err("%s:%d buf:%p count:%lu EAGAIN\n", __func__, __LINE__, buf,
				count);
		msleep(1000);
		return -EAGAIN;
	}

	down_write(&ptrSync->writeSem);

#if 1
	/*
	 * Threads that share the same file descriptor should have the same tgid
	 * However, with uClibc pthread library, pthread_create() creates threads
	 * with pid == tgid So the tgid is not real tgid, we have to maintain the
	 * thread list that we can lookup later
	 */
	if (current->pid != proc->pid)
		update_thread_list(rextra, proc->pid);
#endif

	if (ring_empty(extra))
		size = 0; /* the ring is empty */
	else if (*ringIn > *ringOut)
		size = *ringIn - *ringOut;
	else
		size = rpc_ring_size + *ringIn - *ringOut;

	//pr_debug("%s: count:%d space:%d\n",
	//		extra->name, count, rpc_ring_size - size - 1);
	//pr_debug("%s: pid:%d tgid:%d\n",
	//		extra->name, current->pid, current->tgid);

	if (count > (rpc_ring_size - size - 1))
		goto out;

#if 0
	if (access_ok(VERIFY_READ, buf, sizeof(RPC_STRUCT))) {
		RPC_STRUCT *rpc = (RPC_STRUCT *)buf;
		uint32_t programID = ntohl(rpc->programID);

		if (programID == AUDIO_SYSTEM || programID == REPLYID || programID == 97) {
			rpc->versionID = htonl((u32)refclk_get_counter(refclk));
		} else {
			pr_warn("%s: invalid programID:%u\n", __func__, programID);
		}
	}
#endif

	temp = *ringEnd - *ringIn;

	if (temp >= count) {
#ifdef MY_COPY
		r = my_copy_from_user((int *)AVCPU2SCPU(*ringIn), (int *)buf, count);
#else
		r = copy_from_user((int *)AVCPU2SCPU(*ringIn), (int *)buf, count);
#endif /* MY_COPY */

		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n",
					__func__, __LINE__, buf, count);
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

		//pr_debug("RPC Write is in 1st kind...\n");
	} else {
#ifdef MY_COPY
			r = my_copy_from_user((int *)AVCPU2SCPU(*ringIn), (int *)buf, temp);
#else
			r = copy_from_user((int *)AVCPU2SCPU(*ringIn), (int *)buf, temp);
#endif /* MY_COPY */
			if (r) {
				pr_err("%s:%d buf:%p count:%lu EFAULT\n",
				__func__, __LINE__, buf, count);
				ret = -EFAULT;
				goto out;
			}
			count -= temp;

#ifdef MY_COPY
			r = my_copy_from_user((int *)AVCPU2SCPU(*ringStart), (int *)(buf+temp), count);
#else
			r = copy_from_user((int *)AVCPU2SCPU(*ringStart), (int *)(buf+temp), count);
#endif /* MY_COPY */
			if (r) {
				pr_err("%s:%d buf:%p count:%lu EFAULT\n",
						__func__, __LINE__, buf, count);
				ret = -EFAULT;
				goto out;
			}
			ret += (temp + count);

			//asm("DSB");
			mb();

			*ringIn = *ringStart+((count+3) & 0xfffffffc);
			//pr_debug("RPC Write is in 2nd kind...\n");
	}

	peek_rpc_struct(__func__, extra, *f_pos);
	wmb();

	/* notify all the processes in the wait queue */
	//wake_up_interruptible(&dev->waitQueue);

	/* use the "f_pos" of file object to store the device number */
	temp = (int) *f_pos;
		//if (temp == 1)
	if (temp == 1) {
		rpc_send_interrupt(RPC_AUDIO);
	} else if (temp == 5) {
		rpc_send_interrupt(RPC_VIDEO);
#ifdef CONFIG_REALTEK_RPC_VE3
	} else if (temp == 8) {
		rpc_send_interrupt(RPC_VE3);
#endif
#ifdef CONFIG_REALTEK_RPC_HIFI
	} else if (temp == 10) {
		rpc_send_interrupt(RPC_HIFI);
#endif

	} else {
		pr_err("error device number...");
	}

	pr_debug("%s:%d thread:%s pid:%d tgid:%d device:%s\n",
			__func__, __LINE__, current->comm, current->pid,
			current->tgid, extra->name);
	pr_debug("%s:%d buf:%p count:%lu actual:%ld\n",
			__func__, __LINE__, buf, count, ret);
out:
		//pr_debug("RPC intr ringIn pointer is : 0x%8x\n",
		//		(int)AVCPU2SCPU(dev->ringIn));
	up_write(&ptrSync->writeSem);
	return ret;
}

long rpc_intr_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
		int ret = 0;
		pid_t g_pid;
		pid_t g_tgid;

		while (rpc_intr_is_suspend) {
			pr_warn("RPCintr: someone access rpc poll during the suspend!!!...\n");
			msleep(1000);
		}

		switch (cmd) {
		case RPC_IOCTTIMEOUT:
			timeout = arg;
			break;
		case RPC_IOCQTIMEOUT:
			return timeout;
		case RPC_IOCTEXITLOOP: {
			RPC_PROCESS *proc = filp->private_data;
			proc->bExit = true;
			wake_up_interruptible(&proc->waitQueue);
			return 0;
		}
#ifdef CONFIG_REALTEK_RPC_PROGRAM_REGISTER
		case RPC_IOCTHANDLER: {
			int found;
			RPC_PROCESS *proc = filp->private_data;
			RPC_DEV_EXTRA *extra = proc->extra;
			RPC_HANDLER *handler;

			pr_debug("%s:%d : Register handler for programID:%lu\n",
					__func__, __LINE__, arg);
			found = 0;
			list_for_each_entry(handler, &proc->handlers, list) {
				if (handler->programID == arg) {
					found = 1;
					break;
				}
			}

			if (found)
				break;

			/* not found, add to handler list */
			handler = kmalloc(sizeof(RPC_HANDLER), GFP_KERNEL);
			if (handler == NULL) {
				pr_err("%s: failed to allocate RPC_HANDLER", __func__);
				return -ENOMEM;
			}
			handler->programID = arg;
			spin_lock_bh(&extra->lock);
			list_add(&handler->list, &proc->handlers);
			spin_unlock_bh(&extra->lock);
			pr_debug("%s:%d %s: Add handler pid:%d for programID:%lu\n",
					__func__, __LINE__, proc->extra->name, proc->pid, arg);
			break;
		}
#endif /* CONFIG_REALTEK_RPC_PROGRAM_REGISTER */
		case RPC_IOC_PROCESS_CONFIG_0:
                              {
                                  RPC_PROCESS *proc = filp->private_data;
                                  struct S_RPC_IOC_PROCESS_CONFIG_0 config;
                                  if (copy_from_user(&config, (void __user *)arg, sizeof(struct S_RPC_IOC_PROCESS_CONFIG_0))) {
                                      pr_err("ERROR! %s cmd:RPC_IOC_PROCESS_CONFIG_0 copy_from_user failed\n", __func__);
                                      return -ENOMEM;
                                  }
                                  if (proc == NULL) {
                                      pr_err("ERROR! %s cmd:RPC_IOC_PROCESS_CONFIG_0 proc:%p\n", __func__, proc);
                                      return -ENOMEM;
                                  }
                                  proc->bStayActive = (config.bStayActive > 0) ? true : false;
                                  break;
                              }
		case RPC_IOCTGETGPID:
			g_tgid = task_tgid_nr(current);
			pr_debug("[%s][RPC_IOCTGETGPID]get current global g_tgid:%d \n", __func__, g_tgid);
			if(copy_to_user((int __user *)arg, &g_tgid, sizeof(g_tgid))) {
				pr_err("[RPC_IOCTGETGPID] copy_to_user failed\n");
				return -EFAULT;
			}
			break;

		case RPC_IOCTGETGTGID:
			g_pid = task_pid_nr(current);
			pr_debug("[%s][RPC_IOCTGETGTGID]get current global g_pid:%d \n", __func__, g_pid);
			if(copy_to_user((int __user *)arg, &g_pid, sizeof(g_tgid))) {
				pr_err("[RPC_IOCTGETGTGID] copy_to_user failed\n");
				return -EFAULT;
			}
			break;
		default: /* redundant, as cmd was checked against MAXNR */
			pr_err("%s:%d unsupported ioctl cmd:%x arg:%lx", __func__,
					__LINE__, cmd, arg);
			return -ENOTTY;
		}

		return ret;
}

struct file_operations rpc_intr_fops = {
	//.llseek = scull_llseek,
	.unlocked_ioctl = rpc_intr_ioctl,
	.compat_ioctl = rpc_intr_ioctl,
	.read = rpc_intr_read,
	.write = rpc_intr_write,
	.open = rpc_intr_open,
	.release = rpc_intr_release,
	.flush = rpc_intr_flush,
};



#ifdef CONFIG_RPC_KERN_VE2
int rpc_kern_ve2_open(void)
{
	RPC_PROCESS *proc;

	pr_debug("%s %d +\n",__func__,__LINE__);
	if (ve2_proc_count > 0 ) {
		ve2_proc_count++;
	} else {
		proc = kmalloc(sizeof(RPC_PROCESS), GFP_KERNEL | __GFP_ZERO);
		if (proc == NULL) {
			pr_err("%s: failed to allocate RPC_PROCESS", __func__);
			printk("%s %d -ENOMEM\n",__func__,__LINE__);
			return -ENOMEM;
		}
		proc->dev = (RPC_DEV *)&rpc_intr_devices[3];
		proc->extra = &rpc_intr_extra[3];
		proc->pid = 0xffffffff;
		proc->bStayActive = false;
		init_waitqueue_head(&proc->waitQueue);
		ve2_proc = proc;
		ve2_proc_count = 1;
	}

	pr_debug("%s %d -\n",__func__,__LINE__);
	return 0;
}
EXPORT_SYMBOL(rpc_kern_ve2_open);

int rpc_kern_ve2_close(void)
{
	RPC_PROCESS *proc = ve2_proc;
	RPC_DEV_EXTRA *extra = proc->extra;
	phys_addr_t paddr;
	void *vaddr;
	struct dma_buf *rpc_dmabuf;
	struct sg_table *table;
	struct dma_buf_attachment *attachment;
	int err, ret;

	rpc_dmabuf = ion_alloc(4096, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(rpc_dmabuf)) {
		err = PTR_ERR(rpc_dmabuf);
		goto error;
	}

	attachment = dma_buf_attach(rpc_dmabuf, rpc_dev);
	if (IS_ERR(attachment)) {
		err = PTR_ERR(attachment);
		goto put_dma_buf;
	}
	table = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
	if (!table) {
		err = -ENOMEM;
		goto detach;
	}

	paddr = sg_dma_address(table->sgl);

	vaddr = dma_buf_vmap(rpc_dmabuf);
	if (vaddr == NULL) {
		pr_err("[%s] get vaddr failed\n", __func__);
		ret = -EINVAL;
		//goto detach_dma_buf;
	}

	pr_debug("%s %d +\n",__func__,__LINE__);
	ve2_proc_count--;
	if (ve2_proc_count == 0 ) {
		RPC_DESTROY_VIDEO_FLOW(0xffffffff, paddr, vaddr);
		kfree(proc);
	}

	pr_debug("%s %d -\n",__func__,__LINE__);

	return 0;
detach:
	if (!IS_ERR_OR_NULL(table))
		dma_buf_unmap_attachment(attachment, table, DMA_BIDIRECTIONAL);

	dma_buf_detach(rpc_dmabuf, attachment);

put_dma_buf:
	dma_buf_put(rpc_dmabuf);

error:

	return err;

}
EXPORT_SYMBOL(rpc_kern_ve2_close);

ssize_t rpc_kern_ve2_write(const char *buf, size_t count)
{
	RPC_PROCESS *proc = ve2_proc;
	RPC_DEV_EXTRA *extra = proc->extra -1;
	RPC_DEV *dev = extra->dev; /* the first listitem */
	int temp, size;
	size_t r;
	ssize_t ret = 0;
	uint32_t ptmp;
	int rpc_ring_size = dev->ringEnd - dev->ringStart;

	if (rpc_intr_is_paused) {
		pr_err("RPCintr: someone access rpc intr during the pause...\n");
		pr_err("%s:%d buf:%p count:%lu EAGAIN\n", __func__, __LINE__, buf,
				count);
		msleep(1000);
		return -EAGAIN;
	}
	down_write(&dev->ptrSync->writeSem);
	if (ring_empty(extra))
		size = 0; /* the ring is empty */
	else if (dev->ringIn > dev->ringOut)
		size = dev->ringIn - dev->ringOut;
	else
		size = rpc_ring_size + dev->ringIn - dev->ringOut;
	if (count > (rpc_ring_size - size - 1))
		goto out;
	temp = dev->ringEnd - dev->ringIn;
	if (temp >= count) {
#ifdef MY_COPY
		r = my_copy_from_user((int *)AVCPU2SCPU(dev->ringIn), (int *)buf, count);
#else
		r = copy_from_user((int *)AVCPU2SCPU(dev->ringIn), (int *)buf, count);
#endif /* MY_COPY */
		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n",
					__func__, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}
		ret += count;
		ptmp = dev->ringIn + ((count+3) & 0xfffffffc);
		mb();
		if (ptmp == dev->ringEnd)
			dev->ringIn = dev->ringStart;
		else
			dev->ringIn = ptmp;
	} else {
#ifdef MY_COPY
			r = my_copy_from_user((int *)AVCPU2SCPU(dev->ringIn), (int *)buf, temp);
#else
			r = copy_from_user((int *)AVCPU2SCPU(dev->ringIn), (int *)buf, temp);
#endif /* MY_COPY */
			if (r) {
				pr_err("%s:%d buf:%p count:%lu EFAULT\n",
				__func__, __LINE__, buf, count);
				ret = -EFAULT;
				goto out;
			}
			count -= temp;
#ifdef MY_COPY
			r = my_copy_from_user((int *)AVCPU2SCPU(dev->ringStart), (int *)(buf+temp), count);
#else
			r = copy_from_user((int *)AVCPU2SCPU(dev->ringStart), (int *)(buf+temp), count);
#endif /* MY_COPY */
			if (r) {
				pr_err("%s:%d buf:%p count:%lu EFAULT\n",
						__func__, __LINE__, buf, count);
				ret = -EFAULT;
				goto out;
			}
			ret += (temp + count);
			mb();
			dev->ringIn = dev->ringStart+((count+3) & 0xfffffffc);
	}
	peek_rpc_struct(__func__, extra, 0);
	rtk_rpc_wmb(AVCPU2SCPU(dev->ringStart),
			PAGE_ALIGN(rpc_ring_size));
	rpc_send_interrupt(RPC_VIDEO);
	pr_debug("%s:%d buf:%p count:%lu actual:%ld\n",
			__func__, __LINE__, buf, count, ret);
out:
	up_write(&dev->ptrSync->writeSem);
	return ret;
}
EXPORT_SYMBOL(rpc_kern_ve2_write);

ssize_t rpc_kern_ve2_read(char *buf, size_t count)
{
	RPC_PROCESS *proc = ve2_proc;
	RPC_DEV *dev = NULL; /* the first listitem */
	RPC_DEV_EXTRA *extra = NULL;
	int temp, size;
	size_t r;
	ssize_t ret = 0;
	uint32_t ptmp;
	int rpc_ring_size;;
	long k;

	dev = proc->dev;
	extra = proc->extra;
	rpc_ring_size = dev->ringEnd - dev->ringStart;
	pr_debug("%s %d + dev %p rpc_ring_size %d extra %p extra->name:%s ve2_proc %p proc %p\n",__func__,__LINE__, dev, rpc_ring_size, extra, extra->name, ve2_proc, proc);
	if (rpc_intr_is_paused) {
		pr_err("RPCintr: someone access rpc intr during the pause...\n");
		pr_err("%s:%d buf:%p count:%lu EAGAIN\n", extra->name, __LINE__, buf,
				count);
		msleep(1000);
		printk("%s %d AGAIN-\n",__func__,__LINE__);
		return -EAGAIN;
	}
	if ((extra->currProc != proc) || (ring_empty(extra))) {
		pr_err("%s:%d wait for event\n", __func__, __LINE__);
		if (wait_event_interruptible(proc->waitQueue,
					     ((extra->currProc == proc) && (!ring_empty(extra)) || kthread_should_stop()))) {
							 pr_info("%s %d -\n",__func__,__LINE__);
							 return -ERESTARTSYS;
		}
		if (kthread_should_stop()) {
			pr_info("[%s]exit ve2 kernel read\n", __func__);
			pr_info("%s %d STOP-\n",__func__,__LINE__);
			return -EPIPE;
		}
	}
	down_write(&dev->ptrSync->readSem);
	if (dev->ringIn > dev->ringOut)
		size = dev->ringIn - dev->ringOut;
	else
		size = rpc_ring_size + dev->ringIn - dev->ringOut;
	pr_debug("%s:%d ==going read== count:%zu avail:%d "
			"ringOut:%x ringIn:%x nextRPC:%x",
			extra->name, __LINE__, count, size, dev->ringOut, dev->ringIn,
			extra->nextRpc);
	peek_rpc_struct(__func__, extra, 0);
	if (count > size)
		count = size;
	temp = dev->ringEnd - dev->ringOut;
	rtk_rpc_wmb(AVCPU2SCPU(dev->ringStart), PAGE_ALIGN(rpc_ring_size));
	if (temp >= count) {
#ifdef MY_COPY
		r = my_copy_to_user((int *)buf,
				(int *)AVCPU2SCPU(dev->ringOut), count);
#else
		r = copy_to_user((int *)buf, (int *)AVCPU2SCPU(dev->ringOut), count);
#endif /* MY_COPY */
		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n",
				extra->name, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}
		ret = count;
		ptmp = dev->ringOut + ((count+3) & 0xfffffffc);
		if (ptmp == dev->ringEnd)
			dev->ringOut = dev->ringStart;
		else
			dev->ringOut = ptmp;
	} else {
#ifdef MY_COPY
		r = my_copy_to_user((int *)buf,
				(int *)AVCPU2SCPU(dev->ringOut), temp);
#else
		r = copy_to_user((int *)buf,
				(int *)AVCPU2SCPU(dev->ringOut), temp);
#endif /* MY_COPY */
		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n",
					extra->name, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}
		count -= temp;
#ifdef MY_COPY
		r = my_copy_to_user((int *)(buf+temp),
				(int *)AVCPU2SCPU(dev->ringStart), count);
#else
		r = copy_to_user((int *)(buf+temp),
				(int *)AVCPU2SCPU(dev->ringStart), count);
#endif /* MY_COPY */
		if (r) {
			pr_err("%s:%d buf:%p count:%lu EFAULT\n",
					extra->name, __LINE__, buf, count);
			ret = -EFAULT;
			goto out;
		}
		ret = (temp + count);
		dev->ringOut = dev->ringStart+((count+3) & 0xfffffffc);
	}
	spin_lock_bh(&extra->lock);
	if (rpc_done(extra) && extra->currProc == proc) {
		pr_debug("%s: Previous RPC is done, unregister myself\n", extra->name);
		update_currProc(extra, NULL);
	}
	spin_unlock_bh(&extra->lock);
	if (need_dispatch(extra))
		tasklet_schedule(&(extra->tasklet));
	pr_debug("%s:%d pid:%d tgid:%d count:%lu actual:%ld ringOut:%x "
			"ringIn:%x nextRpc:%x currProc:%p(%d)\n",
			extra->name, __LINE__, current->pid, current->tgid, count, ret,
			dev->ringOut, dev->ringIn, extra->nextRpc, extra->currProc,
			extra->currProc ? ((RPC_PROCESS *)extra->currProc)->pid : 0);
out:
		up_write(&dev->ptrSync->readSem);
done:
		pr_debug("%s %d -\n",__func__,__LINE__);
		return ret;
}
EXPORT_SYMBOL(rpc_kern_ve2_read);

void rpc_kern_ve2_exit_loop(bool flag)
{
	RPC_PROCESS *proc = ve2_proc;

	proc->bExit = flag;
	pr_debug("%s %d + %d flag\n",__func__,__LINE__, flag);
	if (flag == true)
		wake_up_interruptible(&proc->waitQueue);
	pr_debug("%s %d -\n",__func__,__LINE__);
}
EXPORT_SYMBOL(rpc_kern_ve2_exit_loop);
#endif


MODULE_LICENSE("GPL v2");
