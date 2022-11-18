// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2017-2020 Realtek Semiconductor Corp.
 */

#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/fdtable.h>
#include <linux/init.h>
#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/moduleparam.h>
#include <linux/mpage.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/syscalls.h> /* needed for the _IOW etc stuff used later */
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/ion.h>
#include <sound/asound.h>
#include <sound/compress_driver.h>
#include <sound/control.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <asm/cacheflush.h>

#include "snd-realtek.h"
#include "snd-realtek-compress.h"
#include <ion_rtk_alloc.h>
#define ion_alloc ext_rtk_ion_alloc

static int refclock_handle = -1;
static int refclock_from_video;

struct dma_buf *rtk_compress_rpc_dmabuf;
struct dma_buf *compr_inRing_rpc_dmabuf;
struct dma_buf *compr_inband_rpc_dmabuf;
struct dma_buf *compr_dwnstrm_rpc_dmabuf;
struct dma_buf *compr_outRing_rpc_dmabuf[8] = {NULL};
struct dma_buf *refclock_rpc_dmabuf;

struct dma_buf *rawdelay_handle_rpc_dmabuf;
struct dma_buf *rawdelay_handle2_rpc_dmabuf;
struct dma_buf *rawdelay_handle3_rpc_dmabuf;

static int mtotal_latency;
static int *rawdelay_mem;
static struct ALSA_RAW_LATENCY *rawdelay_mem2;
static struct ALSA_RAW_LATENCY *rawdelay_mem3;
static phys_addr_t phy_rawdelay;
static phys_addr_t phy_rawdelay2;
static phys_addr_t phy_rawdelay3;
static int hw_avsync_header_offset;

/*	DHCHERC-219:
 *	There has some noise when multi-video playing specific video.
 */
static char *header_handle_buf1;
static char *header_handle_buf2;
static int header_handle_flag;
static int header_handle_len;
static int delim_check_format;
static int delim_format;
static int delim_header_size;
static bool compress_first_pts;
static int log_lv;
static int data_len;

#define DEC_OUT_BUFFER_SIZE      (36 * 1024)
#define DEC_OUT_BUFFER_SIZE_LOW  (8 * 1024)
#define DEC_INBAND_BUFFER_SIZE   (16 * 1024)
#define DEC_DWNSTRM_BUFFER_SIZE  (32 * 1024)

#define AUDIO_DEC_OUTPIN         8

#define NUM_CODEC                7
#define MIN_FRAGMENT             2
#define MAX_FRAGMENT             16
#define MIN_FRAGMENT_SIZE        (1024) //(50 * 1024)
#define MAX_FRAGMENT_SIZE        (8192*64) //(1024 * 1024)

#define RAWDELAY_MEM_SIZE        8
#define AUTOMASTER_NOT_MASTER    0
#define SHARE_MEM_SIZE_RAW_LATENCY sizeof(struct ALSA_RAW_LATENCY)

struct rtk_runtime_stream {
	int audioDecId;
	int audioPPId;
	int audioOutId;
	int audioDecPinId;
	int audioAppPinId;

	unsigned int codecId;
	unsigned int audioChannel;
	unsigned int audioSamplingRate;
	long lastInRingRP;
	bool isGetInfo;
	bool isLowWater;

	int status;
	size_t bytes_written;
	size_t copied_total;
	size_t comsume_total;
	unsigned long outFrames;
	//unsigned long preOutFrames;

	// ringbuffer header
	struct RINGBUFFER_HEADER decInRingHeader;
	struct RINGBUFFER_HEADER decInbandRingHeader;
	struct RINGBUFFER_HEADER dwnstrmRingHeader;
	struct RINGBUFFER_HEADER decOutRingHeader[8];

	//virtual address of ringbuffer
	unsigned char *virDecInRing;
	unsigned char *virInbandRing;
	unsigned char *virDwnRing;
	unsigned char *virDecOutRing[8];

	unsigned char *virDwnRingLower;
	unsigned char *virDwnRingUpper;

	unsigned char *virDecOutRingLower[8];
	unsigned char *virDecOutRingUpper[8];

	//physical address of ringbuffer
	phys_addr_t phyDecInRing;
	phys_addr_t phyDecInbandRing;
	phys_addr_t phyDecDwnstrmRing;
	phys_addr_t phyDecOutRing[8];

	struct REFCLOCK *refclock;
	phys_addr_t phyRefclock;
	phys_addr_t phy_addr;
	phys_addr_t phy_addr_rpc;

	void *vaddr_rpc;
	struct dma_buf *rpc_dmabuf_rpc;
};

enum DELIVER_WHAT {
	BS,
	COMMAND
};

static phys_addr_t rtk_rpc_ion_pa(struct ion_buffer *buffer)
{
	unsigned long ret = -1UL;
	struct sg_table *table;
	struct page *page;
	phys_addr_t paddr;

	mutex_lock(&buffer->lock);
	table = buffer->sg_table;
	page = sg_page(table->sgl);
	paddr = PFN_PHYS(page_to_pfn(page));
	mutex_unlock(&buffer->lock);

	ret = paddr;

	return ret;
}

static void *rtk_rpc_ion_va(struct ion_buffer *buffer)
{
	struct scatterlist *sg;
	int i, j;
	void *vaddr;
	pgprot_t pgprot;
	struct sg_table *table = buffer->sg_table;
	int npages = PAGE_ALIGN(buffer->size) / PAGE_SIZE;
	struct page **pages = vmalloc(sizeof(struct page *) * npages);
	struct page **tmp = pages;

	if (!pages)
		return ERR_PTR(-ENOMEM);

	if (buffer->flags & ION_FLAG_NONCACHED)
		pgprot = pgprot_noncached(PAGE_KERNEL);
	else if (buffer->flags & ION_FLAG_CACHED)
		pgprot = PAGE_KERNEL;
	else
		pgprot = pgprot_writecombine(PAGE_KERNEL);

	for_each_sg(table->sgl, sg, table->nents, i) {
		int npages_this_entry = PAGE_ALIGN(sg->length) / PAGE_SIZE;
		struct page *page = sg_page(sg);

		BUG_ON(i >= npages);
		for (j = 0; j < npages_this_entry; j++)
			*(tmp++) = page++;
	}
	vaddr = vmap(pages, npages, VM_MAP, pgprot);
	vfree(pages);

	if (!vaddr)
		return ERR_PTR(-ENOMEM);

	return vaddr;
}

static void rtk_rpc_free_ion(struct dma_buf *dmabuf)
{
	pr_info("%s free ion buffer\n", __func__);

	dma_buf_end_cpu_access(dmabuf, DMA_BIDIRECTIONAL);

	dma_buf_put(dmabuf);
}

unsigned long reverseInteger(unsigned long value)
{
	unsigned long b0 = value & 0x000000ff;
	unsigned long b1 = (value & 0x0000ff00) >> 8;
	unsigned long b2 = (value & 0x00ff0000) >> 16;
	unsigned long b3 = (value & 0xff000000) >> 24;

	return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
}

void pli_IPCCopyMemory(unsigned char *src, unsigned char *des, unsigned long len)
{
	unsigned char *pdes = des;
	unsigned char *psrc = src;
	int i;

	for (i = 0; i < len; i += 4)
		writel(__cpu_to_be32(readl(&psrc[i])), &pdes[i]);
}

void MyMemcpy(enum DELIVER_WHAT type, unsigned char *pSrc, unsigned char *pDes, long size)
{
	if (type == COMMAND)
		pli_IPCCopyMemory(pSrc, pDes, size);
	else
		memcpy(pDes, pSrc, size);
}

unsigned long GetBufferFromRing(unsigned char *upper, unsigned char *lower, unsigned char *rptr, unsigned char *dest, unsigned long size)
{
	unsigned long space = 0;

	if (rptr + size < upper) {
		MyMemcpy(COMMAND, rptr, dest, size);
	} else {
		space = upper - rptr;
		MyMemcpy(COMMAND, rptr, dest, space);
		if (size > space)
			MyMemcpy(COMMAND, lower, dest + space, size - space);
	}

	return space;
}

void UpdateRingPtr(unsigned char *upper, unsigned char *lower, unsigned char *rptr,
	struct rtk_runtime_stream *stream, unsigned long size, unsigned long space)
{
	if ((rptr + size) < upper)
		stream->dwnstrmRingHeader.readPtr[0] = ntohl(ntohl(stream->dwnstrmRingHeader.readPtr[0]) + size);
	else
		stream->dwnstrmRingHeader.readPtr[0] = ntohl(ntohl(stream->dwnstrmRingHeader.beginAddr) + size - space);
}

static long ringValidData(long ring_base, long ring_limit, long ring_rp, long ring_wp)
{
	if (ring_wp >= ring_rp)
		return (ring_wp - ring_rp);
	else
		return (ring_limit - ring_base) - (ring_rp - ring_wp);
}

static long validFreeSize(long base, long limit, long rp, long wp)
{
	return (limit - base) - ringValidData(base, limit, rp, wp) - 1;
}

long GetGeneralInstanceID(long instanceID, long pinID)
{
	return ((instanceID & 0xFFFFF000) | (pinID & 0xFFF));
}

void resetPointer(struct rtk_runtime_stream *stream)
{
	stream->copied_total = 0;
	stream->comsume_total = 0;
	stream->bytes_written = 0;
	stream->outFrames = 0;
	stream->isGetInfo = false;
	stream->dwnstrmRingHeader.readPtr[0] = stream->dwnstrmRingHeader.writePtr;
	stream->lastInRingRP = (long)ntohl(stream->decInRingHeader.writePtr);
	mtotal_latency = 0;
}

int triggerAudio(struct rtk_runtime_stream *stream, int cmd)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (stream->status != SNDRV_PCM_TRIGGER_START) {
			RPC_TOAGENT_RUN_SVC(stream->phy_addr_rpc,
						stream->vaddr_rpc, stream->audioOutId);
			RPC_TOAGENT_RUN_SVC(stream->phy_addr_rpc, stream->vaddr_rpc,
						GetGeneralInstanceID(stream->audioPPId, stream->audioAppPinId));
			RPC_TOAGENT_RUN_SVC(stream->phy_addr_rpc,
						stream->vaddr_rpc, stream->audioDecId);
		}
		stream->status = SNDRV_PCM_TRIGGER_START;
		compress_first_pts = true;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (stream->status != SNDRV_PCM_TRIGGER_STOP) {

			struct AUDIO_RPC_SENDIO sendio;

			stream->refclock->RCD = -1;
			stream->refclock->mastership.systemMode = AVSYNC_FORCED_SLAVE;
			stream->refclock->mastership.audioMode = AVSYNC_FORCED_MASTER;
			stream->refclock->mastership.videoMode = AVSYNC_FORCED_MASTER;
			stream->refclock->mastership.masterState = AUTOMASTER_NOT_MASTER;
			stream->refclock->videoFreeRunThreshold = htonl(0x7FFFFFFF);
			stream->refclock->audioFreeRunThreshold = htonl(0x7FFFFFFF);

			// decoder flush
			sendio.instanceID = stream->audioDecId;
			sendio.pinID = stream->audioDecPinId;
			if (RPC_TOAGENT_FLUSH_SVC(stream->phy_addr_rpc, stream->vaddr_rpc, &sendio)) {
				pr_err("[%s %d fail]\n", __func__, __LINE__);
				return -1;
			}

			resetPointer(stream);
			RPC_TOAGENT_STOP_SVC(stream->phy_addr_rpc,
						stream->vaddr_rpc, stream->audioOutId);
			RPC_TOAGENT_STOP_SVC(stream->phy_addr_rpc,
						stream->vaddr_rpc,
						GetGeneralInstanceID(stream->audioPPId, stream->audioAppPinId));
			RPC_TOAGENT_STOP_SVC(stream->phy_addr_rpc,
						stream->vaddr_rpc, stream->audioDecId);
		}
		stream->status = SNDRV_PCM_TRIGGER_STOP;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (stream->status != SNDRV_PCM_TRIGGER_PAUSE_PUSH) {
			RPC_TOAGENT_PAUSE_SVC(stream->phy_addr_rpc,
						stream->vaddr_rpc, stream->audioOutId);
			RPC_TOAGENT_PAUSE_SVC(stream->phy_addr_rpc,
						stream->vaddr_rpc,
						GetGeneralInstanceID(stream->audioPPId, stream->audioAppPinId));
			RPC_TOAGENT_PAUSE_SVC(stream->phy_addr_rpc,
						stream->vaddr_rpc, stream->audioDecId);
		}
		stream->refclock->RCD = -1;
		stream->refclock->mastership.audioMode = AVSYNC_FORCED_MASTER;
		stream->refclock->mastership.videoMode = AVSYNC_FORCED_SLAVE;
		stream->status = SNDRV_PCM_TRIGGER_PAUSE_PUSH;
		break;
	default:
		break;
	}

	return 0;
}

void destroyAudioComponent(struct rtk_runtime_stream *stream)
{
	triggerAudio(stream, SNDRV_PCM_TRIGGER_STOP);

	RPC_TOAGENT_PP_INIT_PIN_SVC(stream->phy_addr_rpc,
				stream->vaddr_rpc,
				GetGeneralInstanceID(stream->audioPPId, stream->audioAppPinId));
	RPC_TOAGENT_DESTROY_SVC(stream->phy_addr_rpc,
				stream->vaddr_rpc,
				GetGeneralInstanceID(stream->audioPPId, stream->audioAppPinId));
	RPC_TOAGENT_DESTROY_SVC(stream->phy_addr_rpc, stream->vaddr_rpc, stream->audioDecId);

	stream->audioDecId = 0;
	stream->audioPPId = 0;
	stream->audioOutId = 0;
	stream->audioDecPinId = 0;
	stream->audioAppPinId = 0;
}

int createAudioComponent(struct rtk_runtime_stream *stream)
{
	// create decoder agent
	if (RPC_TOAGENT_CREATE_DECODER_AGENT(stream->phy_addr_rpc, stream->vaddr_rpc,
				&stream->audioDecId, &stream->audioDecPinId)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	// create pp agent and get global pp pin
	if (RPC_TOAGENT_CREATE_PP_AGENT(stream->phy_addr_rpc, stream->vaddr_rpc,
				&stream->audioPPId, &stream->audioAppPinId)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	// create ao agent
	if (RPC_TOAGENT_CREATE_AO_AGENT(stream->phy_addr_rpc, stream->vaddr_rpc,
				&stream->audioOutId, AUDIO_OUT)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}
	stream->audioOutId = GetGeneralInstanceID(stream->audioOutId, stream->audioAppPinId);

	if (RPC_TOAGENT_SEND_AUDIO_VERSION(stream->phy_addr_rpc, stream->vaddr_rpc,
				ENUM_OMX_AUDIO_VERSION_1)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	return 0;
}

int configOutput(struct rtk_runtime_stream *stream)
{
	//config i2s
	struct AUDIO_CONFIG_DAC_I2S dac_i2s_config;
	struct AUDIO_CONFIG_DAC_SPDIF dac_spdif_config;
	struct AUDIO_RPC_FOCUS focus;

	dac_i2s_config.instanceID = stream->audioOutId;
	dac_i2s_config.dacConfig.audioGeneralConfig.interface_en = 1;
	dac_i2s_config.dacConfig.audioGeneralConfig.channel_out = LEFT_CENTER_FRONT_CHANNEL_EN|RIGHT_CENTER_FRONT_CHANNEL_EN;
	dac_i2s_config.dacConfig.audioGeneralConfig.count_down_play_en = 0;
	dac_i2s_config.dacConfig.audioGeneralConfig.count_down_play_cyc = 0;
	dac_i2s_config.dacConfig.sampleInfo.sampling_rate = 48000;
	dac_i2s_config.dacConfig.sampleInfo.PCM_bitnum = 24;

	if (RPC_TOAGENT_DAC_I2S_CONFIG(stream->phy_addr_rpc, stream->vaddr_rpc,
				&dac_i2s_config)) {
		pr_err("[%s %d]\n", __func__, __LINE__);
		return -1;
	}

	//config spdif
	dac_spdif_config.instanceID = stream->audioOutId;
	dac_spdif_config.spdifConfig.audioGeneralConfig.interface_en = 1;
	dac_spdif_config.spdifConfig.audioGeneralConfig.channel_out = SPDIF_LEFT_CHANNEL_EN|SPDIF_RIGHT_CHANNEL_EN;
	dac_spdif_config.spdifConfig.audioGeneralConfig.count_down_play_en = 0;
	dac_spdif_config.spdifConfig.audioGeneralConfig.count_down_play_cyc = 0;
	dac_spdif_config.spdifConfig.sampleInfo.sampling_rate = 48000;
	dac_spdif_config.spdifConfig.sampleInfo.PCM_bitnum = 24;
	dac_spdif_config.spdifConfig.out_cs_info.non_pcm_valid = 0;
	dac_spdif_config.spdifConfig.out_cs_info.non_pcm_format = 0;
	dac_spdif_config.spdifConfig.out_cs_info.audio_format = 0;
	dac_spdif_config.spdifConfig.out_cs_info.spdif_consumer_use = 0;
	dac_spdif_config.spdifConfig.out_cs_info.copy_right = 0;
	dac_spdif_config.spdifConfig.out_cs_info.pre_emphasis = 0;
	dac_spdif_config.spdifConfig.out_cs_info.stereo_channel = 0;

	if (RPC_TOAGENT_DAC_SPDIF_CONFIG(stream->phy_addr_rpc, stream->vaddr_rpc,
				&dac_spdif_config)) {
		pr_err("[%s %d]\n", __func__, __LINE__);
		return -1;
	}

	//set focus
	focus.instanceID = stream->audioOutId;
	focus.focusID = 0;
	if (RPC_TOAGENT_SWITCH_FOCUS(stream->phy_addr_rpc, stream->vaddr_rpc, &focus)) {
		pr_err("[%s %d]\n", __func__, __LINE__);
		return -1;
	}

	focus.instanceID = stream->audioPPId;
	focus.focusID = stream->audioAppPinId;
	if (RPC_TOAGENT_SWITCH_FOCUS(stream->phy_addr_rpc, stream->vaddr_rpc, &focus)) {
		pr_err("[%s %d]\n", __func__, __LINE__);
		return -1;
	}

	return 0;
}

void destroyRingBuf(struct rtk_runtime_stream *stream)
{
	int ch;

	if (stream->codecId == SND_AUDIOCODEC_TRUEHD)
		RPC_TOAGENT_SET_TRUEHD_ERR_SELF_RESET(stream->phy_addr_rpc, stream->vaddr_rpc, false);

	if (stream->isLowWater)
		RPC_TOAGENT_SET_LOW_WATER_LEVEL(stream->phy_addr_rpc, stream->vaddr_rpc, false);

	if (!IS_ERR(refclock_rpc_dmabuf) && refclock_from_video == 0) {
		rtk_rpc_free_ion(refclock_rpc_dmabuf);
		refclock_rpc_dmabuf = NULL;
	}

	if (!IS_ERR(compr_inRing_rpc_dmabuf)) {
		rtk_rpc_free_ion(compr_inRing_rpc_dmabuf);
		compr_inRing_rpc_dmabuf = NULL;
	}

	if (!IS_ERR(compr_inband_rpc_dmabuf)) {
		rtk_rpc_free_ion(compr_inband_rpc_dmabuf);
		compr_inband_rpc_dmabuf = NULL;
	}

	if (!IS_ERR(compr_dwnstrm_rpc_dmabuf)) {
		rtk_rpc_free_ion(compr_dwnstrm_rpc_dmabuf);
		compr_dwnstrm_rpc_dmabuf = NULL;
	}

	for (ch = 0; ch < AUDIO_DEC_OUTPIN; ch++) {
		if (!IS_ERR(compr_outRing_rpc_dmabuf[ch])) {
			rtk_rpc_free_ion(compr_outRing_rpc_dmabuf[ch]);
			compr_outRing_rpc_dmabuf[ch] = NULL;
		}
	}
}

int createRingBuf(struct rtk_runtime_stream *stream, unsigned int buffer_size)
{
	int ch, i;
	phys_addr_t dat;
	unsigned int decOutSize = 0;
	struct AUDIO_RPC_RINGBUFFER_HEADER ringBufferHeader;
	struct AUDIO_RPC_CONNECTION connection;

	/* Set low water mode for according to format & sample rate */
	if (stream->isLowWater) {
		decOutSize = DEC_OUT_BUFFER_SIZE_LOW;
		RPC_TOAGENT_SET_LOW_WATER_LEVEL(stream->phy_addr_rpc, stream->vaddr_rpc, true);
	} else
		decOutSize = DEC_OUT_BUFFER_SIZE;

	//create struct RINGBUFFER_HEADER decInRing
	compr_inRing_rpc_dmabuf = ion_alloc(buffer_size, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(compr_inRing_rpc_dmabuf)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		i = PTR_ERR(compr_inRing_rpc_dmabuf);
		goto fail;
	}

	dat = rtk_rpc_ion_pa(compr_inRing_rpc_dmabuf->priv);
	if (dat == -1) {
		pr_err("[%s %d] alloc memory faild\n", __func__, __LINE__);
		goto fail;
	}
	stream->virDecInRing = rtk_rpc_ion_va(compr_inRing_rpc_dmabuf->priv);
	stream->phyDecInRing = dat;

	stream->decInRingHeader.beginAddr = htonl((unsigned long)stream->phyDecInRing);
	stream->decInRingHeader.size = htonl(buffer_size);
	stream->decInRingHeader.writePtr = stream->decInRingHeader.beginAddr;
	stream->decInRingHeader.numOfReadPtr = htonl(1);
	stream->decInRingHeader.readPtr[0] = stream->decInRingHeader.beginAddr;
	stream->lastInRingRP = (long)ntohl(stream->decInRingHeader.readPtr[0]);

	ringBufferHeader.instanceID = stream->audioDecId;
	ringBufferHeader.pinID = BASE_BS_IN; //stream->audioDecPinId;
	ringBufferHeader.readIdx = 0;
	ringBufferHeader.listSize = 1;
	ringBufferHeader.pRingBufferHeaderList[0] = (unsigned long)&stream->decInRingHeader - (unsigned long)stream + stream->phy_addr;

	if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(stream->phy_addr_rpc, stream->vaddr_rpc,
				&ringBufferHeader, ringBufferHeader.listSize) < 0) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		goto fail;
	};

	//create struct RINGBUFFER_HEADER decInbandRing;
	compr_inband_rpc_dmabuf = ion_alloc(DEC_INBAND_BUFFER_SIZE, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(compr_inband_rpc_dmabuf)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		i = PTR_ERR(compr_inband_rpc_dmabuf);
		goto fail;
	}

	dat = rtk_rpc_ion_pa(compr_inband_rpc_dmabuf->priv);
	if (dat == -1) {
		pr_err("[%s %d] alloc memory faild\n", __func__, __LINE__);
		goto fail;
	}
	stream->virInbandRing = rtk_rpc_ion_va(compr_inband_rpc_dmabuf->priv);
	stream->phyDecInbandRing = dat;

	stream->decInbandRingHeader.beginAddr = htonl((unsigned long)stream->phyDecInbandRing);
	stream->decInbandRingHeader.size = htonl(DEC_INBAND_BUFFER_SIZE);
	stream->decInbandRingHeader.writePtr = stream->decInbandRingHeader.beginAddr;
	stream->decInbandRingHeader.numOfReadPtr = htonl(1);
	stream->decInbandRingHeader.readPtr[0] = stream->decInbandRingHeader.beginAddr;

	ringBufferHeader.instanceID = stream->audioDecId;
	ringBufferHeader.pinID = INBAND_QUEUE;
	ringBufferHeader.readIdx = 0;
	ringBufferHeader.listSize = 1;
	ringBufferHeader.pRingBufferHeaderList[0] = (unsigned long)&stream->decInbandRingHeader - (unsigned long)stream + stream->phy_addr;

	if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(stream->phy_addr_rpc, stream->vaddr_rpc,
				&ringBufferHeader, ringBufferHeader.listSize) < 0) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		goto fail;
	};

	//create struct RINGBUFFER_HEADER dwnstrmRing;
	compr_dwnstrm_rpc_dmabuf = ion_alloc(DEC_DWNSTRM_BUFFER_SIZE, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(compr_dwnstrm_rpc_dmabuf)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		i = PTR_ERR(compr_dwnstrm_rpc_dmabuf);
		goto fail;
	}

	dat = rtk_rpc_ion_pa(compr_dwnstrm_rpc_dmabuf->priv);
	if (dat == -1) {
		pr_err("[%s %d] alloc memory faild\n", __func__, __LINE__);
		goto fail;
	}
	stream->virDwnRing = rtk_rpc_ion_va(compr_dwnstrm_rpc_dmabuf->priv);
	stream->phyDecDwnstrmRing = dat;

	stream->virDwnRingLower = stream->virDwnRing;
	stream->virDwnRingUpper = stream->virDwnRing + DEC_DWNSTRM_BUFFER_SIZE;

	stream->dwnstrmRingHeader.beginAddr = htonl((unsigned long)stream->phyDecDwnstrmRing);
	stream->dwnstrmRingHeader.size = htonl(DEC_DWNSTRM_BUFFER_SIZE);
	stream->dwnstrmRingHeader.writePtr = stream->dwnstrmRingHeader.beginAddr;
	stream->dwnstrmRingHeader.numOfReadPtr = htonl(1);
	stream->dwnstrmRingHeader.readPtr[0] = stream->dwnstrmRingHeader.beginAddr;

	ringBufferHeader.instanceID = stream->audioDecId;
	ringBufferHeader.pinID = DWNSTRM_INBAND_QUEUE;
	ringBufferHeader.readIdx = -1;
	ringBufferHeader.listSize = 1;
	ringBufferHeader.pRingBufferHeaderList[0] = (unsigned long)&stream->dwnstrmRingHeader - (unsigned long)stream + stream->phy_addr;

	if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(stream->phy_addr_rpc, stream->vaddr_rpc,
				&ringBufferHeader, ringBufferHeader.listSize) < 0) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		goto fail;
	};

	for (ch = 0; ch < AUDIO_DEC_OUTPIN; ch++) {
		compr_outRing_rpc_dmabuf[ch] = ion_alloc(decOutSize, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
		if (IS_ERR(compr_outRing_rpc_dmabuf[ch])) {
			pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
			i = PTR_ERR(compr_outRing_rpc_dmabuf[ch]);
			goto fail;
		}

		dat = rtk_rpc_ion_pa(compr_outRing_rpc_dmabuf[ch]->priv);
		if (dat == -1) {
			pr_err("[%s %d] alloc memory faild\n", __func__, __LINE__);
			goto fail;
		}
		stream->virDecOutRing[ch] = rtk_rpc_ion_va(compr_outRing_rpc_dmabuf[ch]->priv);
		stream->phyDecOutRing[ch] = dat;

		stream->virDecOutRingLower[ch] = stream->virDecOutRing[ch];
		stream->virDecOutRingUpper[ch] = stream->virDecOutRing[ch] + decOutSize;

		stream->decOutRingHeader[ch].beginAddr = htonl((unsigned long)stream->phyDecOutRing[ch]);
		stream->decOutRingHeader[ch].size = htonl(decOutSize);
		stream->decOutRingHeader[ch].writePtr = stream->decOutRingHeader[ch].beginAddr;
		stream->decOutRingHeader[ch].numOfReadPtr = htonl(1);
		for (i = 0; i < 4; i++)
			stream->decOutRingHeader[ch].readPtr[i] = stream->decOutRingHeader[ch].beginAddr;
	}

	ringBufferHeader.instanceID = stream->audioDecId;
	ringBufferHeader.pinID = PCM_OUT;
	ringBufferHeader.readIdx = -1; //todo maybe be to set 1 in there for getting frame pts
	ringBufferHeader.listSize = AUDIO_DEC_OUTPIN;
	for (ch = 0; ch < AUDIO_DEC_OUTPIN; ++ch)
		ringBufferHeader.pRingBufferHeaderList[ch] = (unsigned long)&stream->decOutRingHeader[ch] - (unsigned long)stream + stream->phy_addr;

	if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(stream->phy_addr_rpc, stream->vaddr_rpc,
				&ringBufferHeader, ringBufferHeader.listSize) < 0) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		goto fail;
	};

	ringBufferHeader.instanceID = stream->audioPPId;
	ringBufferHeader.pinID = stream->audioAppPinId;
	ringBufferHeader.readIdx = 0;
	ringBufferHeader.listSize = AUDIO_DEC_OUTPIN;
	for (ch = 0; ch < AUDIO_DEC_OUTPIN; ++ch)
		ringBufferHeader.pRingBufferHeaderList[ch] = (unsigned long)&stream->decOutRingHeader[ch] - (unsigned long)stream + stream->phy_addr;

	if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(stream->phy_addr_rpc, stream->vaddr_rpc,
				&ringBufferHeader, ringBufferHeader.listSize) < 0) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		goto fail;
	};

	//connection
	connection.desInstanceID = stream->audioPPId;
	connection.srcInstanceID = stream->audioDecId;
	connection.srcPinID = PCM_OUT;
	connection.desPinID = stream->audioAppPinId;

	if (RPC_TOAGENT_CONNECT_SVC(stream->phy_addr_rpc, stream->vaddr_rpc,
				&connection)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		goto fail;
	};

	if (configOutput(stream)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		goto fail;
	}

	return 0;

fail:
	destroyRingBuf(stream);
	return -1;
}

int getAudioInfo(struct rtk_runtime_stream *stream, unsigned int *channel, unsigned int *rate)
{
	int ret = 0;
	unsigned char *ptsrp = stream->virDwnRing +
					(ntohl(stream->dwnstrmRingHeader.readPtr[0]) - ntohl(stream->dwnstrmRingHeader.beginAddr));
	unsigned char *ptsrp_tmp = ptsrp;
	bool new_chIdx_type = false;
	int audio_start_thld = 0;
	unsigned long space = 0;
	long up_space = 0;
	struct AUDIO_INFO_PCM_FORMAT pcmFormat;
	struct AUDIO_INBAND_PRIVATE_INFO inbandInfo;

	///////////////////////////////////////////////////////////////////////////////
	/// privateinfo   (16)
	/// pcm_format    (32)
	/// privateinfo   (16)
	/// channel_index (20)
	///////////////////////////////////////////////////////////////////////////////
	memset(&inbandInfo, 0, INBAND_INFO_SIZE);
	space = GetBufferFromRing(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp_tmp, (unsigned char *)&inbandInfo, INBAND_INFO_SIZE);

	if (inbandInfo.infoType != AUDIO_INBAND_CMD_PRIVATE_PCM_FMT)
		return ret;

	if (space > 0)
		ptsrp_tmp = stream->virDwnRing + INBAND_INFO_SIZE - space;
	else
		ptsrp_tmp = ptsrp_tmp + INBAND_INFO_SIZE;

	memset(&pcmFormat, 0, INBAND_PCM_SIZE);
	space = GetBufferFromRing(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp_tmp, (unsigned char *)&pcmFormat, INBAND_PCM_SIZE);

	if (space > 0)
		ptsrp_tmp = stream->virDwnRing + INBAND_PCM_SIZE - space;
	else
		ptsrp_tmp = ptsrp_tmp + INBAND_PCM_SIZE;

	memset(&inbandInfo, 0, INBAND_INFO_SIZE);
	space = GetBufferFromRing(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp_tmp, (unsigned char *)&inbandInfo, INBAND_INFO_SIZE);

	if (inbandInfo.infoType != AUDIO_INBAND_CMD_PRIVATE_CH_IDX)
		return ret;

	if (space > 0)
		ptsrp_tmp = stream->virDwnRing + INBAND_INFO_SIZE - space;
	else
		ptsrp_tmp = ptsrp_tmp + INBAND_INFO_SIZE;

	//get pcm information and channel index
	*rate = pcmFormat.pcmFormat.samplerate;

	if (inbandInfo.infoSize > OLD_CHANNEL_INDEX_INFO_SIZE) {
		new_chIdx_type = true;
		pr_info("use NEW CHANNEL_INDEX_INFO\n");
	} else {
		pr_info("use OLD CHANNEL_INDEX_INFO\n");
	}

	if (new_chIdx_type == true) {
		int count;
		struct AUDIO_INFO_CHANNEL_INDEX_NEW infoChIndex;

		memset(&infoChIndex, 0, sizeof(infoChIndex));
		space = GetBufferFromRing(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp_tmp, (unsigned char *)&infoChIndex, sizeof(infoChIndex));

		for (count = 0; count < AUDIO_DEC_OUTPIN; count++) {
			if (infoChIndex.channel_index[count])
				(*channel)++;
		}
	} else {
		int count;
		struct AUDIO_INFO_CHANNEL_INDEX_OLD infoChIndex;

		memset(&infoChIndex, 0, sizeof(infoChIndex));
		space = GetBufferFromRing(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp_tmp, (unsigned char *)&infoChIndex, sizeof(infoChIndex));

		for (count = 0; count < AUDIO_DEC_OUTPIN; count++) {
			if (infoChIndex.channel_index[count])
				(*channel)++;
		}
	}
	up_space = ringValidData((long)stream->virDwnRingLower, (long)stream->virDwnRingUpper, (long)ptsrp, (long)stream->virDwnRingUpper);
	audio_start_thld = INBAND_PCM_SIZE + (2 * INBAND_INFO_SIZE) + inbandInfo.infoSize;
	UpdateRingPtr(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp, stream, audio_start_thld, up_space);
	ret = 1;

	return ret;
}

int checkAudioInfo(struct rtk_runtime_stream *stream)
{
	int ret = 0;
	unsigned char *ptswp = stream->virDwnRing +
		(ntohl(stream->dwnstrmRingHeader.writePtr) - ntohl(stream->dwnstrmRingHeader.beginAddr));
	unsigned char *ptsrp = stream->virDwnRing +
		(ntohl(stream->dwnstrmRingHeader.readPtr[0]) - ntohl(stream->dwnstrmRingHeader.beginAddr));
	long canReadsize = ringValidData((long)stream->virDwnRingLower, (long)stream->virDwnRingUpper, (long)ptsrp, (long)ptswp);

	if (canReadsize >= AUDIO_START_THRES) {
		unsigned int numChannel = 0;
		unsigned int numRate = 0;

		ret = getAudioInfo(stream, &numChannel, &numRate);

		if (numChannel > 0) {
			stream->isGetInfo = true;
			pr_info("snd_compress got stream.info isGetInfo set to true\n");
			if (numChannel != stream->audioChannel || numRate != stream->audioSamplingRate) {
				stream->audioChannel = numChannel;
				stream->audioSamplingRate = numRate;
			}
			ret = 1;
		}
	}

	return ret;
}

int GetSampleIndex(unsigned int sampleRate)
{
	unsigned int index;
	static const unsigned int aac_samplerate_tab[12] = {
		96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000};

	for (index = 0; index < ARRAY_SIZE(aac_samplerate_tab); index++) {
		if (sampleRate == aac_samplerate_tab[index])
			return index;
	}

	return -1;
}

int snd_monitor_raw_data_queue_new(struct rtk_runtime_stream *stream)
{
	int rawOutDelay = 0;

	if (rawdelay_mem2) {
		uint64_t pcmPTS;
		uint64_t curPTS;
		uint64_t diffPTS;
		unsigned int sum = 0;
		unsigned int latency = 0;
		unsigned int ptsL = 0;
		unsigned int ptsH = 0;
		unsigned int in_wp = 0;
		unsigned int decfrm = 0;
		int retry = 0;

		//to get ALSA_LATENCY_INFO frmo fw
		if (rawdelay_mem3) {
			memcpy(rawdelay_mem3, rawdelay_mem2, SHARE_MEM_SIZE_RAW_LATENCY);
			latency = htonl(rawdelay_mem3->latency);
			ptsL = htonl(rawdelay_mem3->ptsL);
			ptsH = htonl(rawdelay_mem3->ptsH);
			sum = htonl(rawdelay_mem3->sum);
			decfrm = htonl(rawdelay_mem3->decfrm_smpl);

			/* If the sync word is set, wp is physical address */
			if (rawdelay_mem3->sync == htonl(0x23792379))
				in_wp = htonl(rawdelay_mem3->decin_wp);
			else
				in_wp = htonl(rawdelay_mem3->decin_wp) & 0x0fffffff;

			while (sum != (latency + ptsL)) {
				if (retry > 1000) {
					if (ptsL < sum)
						latency = sum - ptsL;
					break;
				}
				memcpy(rawdelay_mem3, rawdelay_mem2, SHARE_MEM_SIZE_RAW_LATENCY);
				latency = htonl(rawdelay_mem3->latency);
				ptsL = htonl(rawdelay_mem3->ptsL);
				ptsH = htonl(rawdelay_mem3->ptsH);
				sum = htonl(rawdelay_mem3->sum);
				decfrm = htonl(rawdelay_mem3->decfrm_smpl);

				/* If the sync word is set, wp is physical address */
				if (rawdelay_mem3->sync == htonl(0x23792379))
					in_wp = htonl(rawdelay_mem3->decin_wp);
				else
					in_wp = htonl(rawdelay_mem3->decin_wp) & 0x0fffffff;

				retry++;
			}
		}

		pcmPTS = (((uint64_t)ptsH << 32) | ((uint64_t)ptsL));
		curPTS = snd_card_get_90k_pts();
		diffPTS = curPTS - pcmPTS;

		// no ptsL means the audio fw is old.
		if (in_wp == 0 && ptsL == 0) {
			if (rawdelay_mem) {
				rawOutDelay = *rawdelay_mem;
				rawOutDelay = htonl(rawOutDelay);
			}
		} else {
			rawOutDelay = latency - div64_ul(diffPTS * 1000, 90);
			rawOutDelay = rawOutDelay / 1000;
			if (rawOutDelay < 0)
				rawOutDelay = 0;
		}

		if (retry > 120)
			pr_info("alsa raw retry %d latency %u , ptsL %u, ptsH %u, sum %u, in_wp %u decfrm %u fr %lu curPTS %llu pcmPTS %llu rawOutDelay %d\n",
				retry, latency, ptsL, ptsH, sum, in_wp, decfrm, stream->outFrames, curPTS, pcmPTS, rawOutDelay);
	} else if (rawdelay_mem) {
		rawOutDelay = *rawdelay_mem;
		rawOutDelay = htonl(rawOutDelay);
	} else
		pr_err("NO exist share memory %s %d!!\n", __func__, __LINE__);

	mtotal_latency = rawOutDelay;
	return rawOutDelay;
}

static int snd_card_compr_open(struct snd_compr_stream *cstream)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct rtk_runtime_stream *stream = NULL;
	phys_addr_t dat;

	rtk_compress_rpc_dmabuf = ion_alloc(sizeof(struct rtk_runtime_stream), RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(rtk_compress_rpc_dmabuf)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		dat = PTR_ERR(rtk_compress_rpc_dmabuf);
		goto fail;
	}

	dat = rtk_rpc_ion_pa(rtk_compress_rpc_dmabuf->priv);
	if (dat == -1) {
		pr_err("[%s %d] alloc memory faild\n", __func__, __LINE__);
		goto fail;
	}

	stream = rtk_rpc_ion_va(rtk_compress_rpc_dmabuf->priv);
	memset(stream, 0, sizeof(struct rtk_runtime_stream));
	stream->phy_addr = dat;

	/* Preparing the ion buffer for all rpc */
	stream->rpc_dmabuf_rpc = ion_alloc(4096, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(stream->rpc_dmabuf_rpc)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		dat = PTR_ERR(stream->rpc_dmabuf_rpc);
		goto fail;
	}

	stream->phy_addr_rpc = rtk_rpc_ion_pa(stream->rpc_dmabuf_rpc->priv);
	if (stream->phy_addr_rpc == -1) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		goto fail;
	}
	stream->vaddr_rpc = rtk_rpc_ion_va(stream->rpc_dmabuf_rpc->priv);

	if (createAudioComponent(stream)) {
		pr_err("[%s %d] create Audio Component failed\n", __func__, __LINE__);
		goto fail;
	}
	stream->status = SNDRV_PCM_TRIGGER_STOP;

	runtime->private_data = stream;

	if (rawdelay_mem == NULL) {
		rawdelay_handle_rpc_dmabuf = ion_alloc(RAWDELAY_MEM_SIZE, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
		if (IS_ERR(rawdelay_handle_rpc_dmabuf)) {
			pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
			phy_rawdelay = PTR_ERR(rawdelay_handle_rpc_dmabuf);
			goto fail;
		}

		phy_rawdelay = rtk_rpc_ion_pa(rawdelay_handle_rpc_dmabuf->priv);
		if (phy_rawdelay == -1) {
			pr_info("[%s %d] alloc memory faild\n", __func__, __LINE__);
			goto fail;
		}

		rawdelay_mem = rtk_rpc_ion_va(rawdelay_handle_rpc_dmabuf->priv);
		memset(rawdelay_mem, 0, RAWDELAY_MEM_SIZE);
	}

	if (rawdelay_mem2 == NULL) {
		rawdelay_handle2_rpc_dmabuf = ion_alloc(SHARE_MEM_SIZE_RAW_LATENCY, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
		if (IS_ERR(rawdelay_handle2_rpc_dmabuf)) {
			pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
			phy_rawdelay2 = PTR_ERR(rawdelay_handle2_rpc_dmabuf);
			goto fail;
		}

		phy_rawdelay2 = rtk_rpc_ion_pa(rawdelay_handle2_rpc_dmabuf->priv);
		if (phy_rawdelay2 == -1) {
			pr_info("[%s %d] alloc memory faild\n", __func__, __LINE__);
			goto fail;
		}

		rawdelay_mem2 = rtk_rpc_ion_va(rawdelay_handle2_rpc_dmabuf->priv);
		memset(rawdelay_mem2, 0, SHARE_MEM_SIZE_RAW_LATENCY);

		/* Set up the sync word for new latency structure */
		rawdelay_mem2->sync = htonl(0x23792379);

		RPC_TOAGENT_PUT_SHARE_MEMORY_LATENCY(stream->phy_addr_rpc, stream->vaddr_rpc,
					(void *)phy_rawdelay, (void *)phy_rawdelay2,
					stream->audioDecId, stream->audioOutId,
					ENUM_PRIVATEINFO_AUDIO_PROVIDE_RAWOUT_LATENCY);
	}

	if (rawdelay_mem3 == NULL) {
		rawdelay_handle3_rpc_dmabuf = ion_alloc(SHARE_MEM_SIZE_RAW_LATENCY, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
		if (IS_ERR(rawdelay_handle3_rpc_dmabuf)) {
			pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
			phy_rawdelay3 = PTR_ERR(rawdelay_handle3_rpc_dmabuf);
			goto fail;
		}

		phy_rawdelay3 = rtk_rpc_ion_pa(rawdelay_handle3_rpc_dmabuf->priv);
		if (phy_rawdelay3 == -1) {
			pr_info("[%s %d] alloc memory faild\n", __func__, __LINE__);
			goto fail;
		}

		rawdelay_mem3 = rtk_rpc_ion_va(rawdelay_handle3_rpc_dmabuf->priv);
		memset(rawdelay_mem3, 0, SHARE_MEM_SIZE_RAW_LATENCY);

		/* Set up the sync word for new latency structure */
		rawdelay_mem3->sync = htonl(0x23792379);
	}

	/* Initialize some global data */
	mtotal_latency = 0;
	refclock_from_video = 0;
	delim_check_format = 1;
	hw_avsync_header_offset = 0;
	header_handle_flag = 0;
	header_handle_len = 0;
	delim_check_format = 1;
	delim_format = 0;
	delim_header_size = 0;
	compress_first_pts = false;
	log_lv = 0;
	data_len = 0;
	header_handle_buf1 = kmalloc(sizeof(char *)*16, GFP_KERNEL);
	header_handle_buf2 = kmalloc(sizeof(char *)*64, GFP_KERNEL);

	return 0;
fail:
	if (!IS_ERR(rawdelay_handle_rpc_dmabuf)) {
		rtk_rpc_free_ion(rawdelay_handle_rpc_dmabuf);
		rawdelay_mem = NULL;
		rawdelay_handle_rpc_dmabuf = NULL;
	}

	if (!IS_ERR(rawdelay_handle2_rpc_dmabuf)) {
		rtk_rpc_free_ion(rawdelay_handle2_rpc_dmabuf);
		rawdelay_mem2 = NULL;
		rawdelay_handle2_rpc_dmabuf = NULL;
	}

	if (!IS_ERR(rawdelay_handle3_rpc_dmabuf)) {
		rtk_rpc_free_ion(rawdelay_handle3_rpc_dmabuf);
		rawdelay_mem3 = NULL;
		rawdelay_handle3_rpc_dmabuf = NULL;
	}

	if (!IS_ERR(stream->rpc_dmabuf_rpc)) {
		rtk_rpc_free_ion(stream->rpc_dmabuf_rpc);
		stream->vaddr_rpc = NULL;
		stream->rpc_dmabuf_rpc = NULL;
	}

	if (!IS_ERR(rtk_compress_rpc_dmabuf)) {
		rtk_rpc_free_ion(rtk_compress_rpc_dmabuf);
		stream = NULL;
		rtk_compress_rpc_dmabuf = NULL;
	}

	return -1;
}

static int snd_card_compr_free(struct snd_compr_stream *cstream)
{
	int ret_val;
	struct rtk_runtime_stream *stream = cstream->runtime->private_data;

	if (rawdelay_mem) {
		if (!IS_ERR(rawdelay_handle_rpc_dmabuf)) {
			rtk_rpc_free_ion(rawdelay_handle_rpc_dmabuf);
			rawdelay_handle_rpc_dmabuf = NULL;
		}
		rawdelay_mem = NULL;
	}

	if (rawdelay_mem2) {
		RPC_TOAGENT_PUT_SHARE_MEMORY_LATENCY(stream->phy_addr_rpc,
				stream->vaddr_rpc,
				NULL, NULL, 0, 0,
				ENUM_PRIVATEINFO_AUDIO_PROVIDE_RAWOUT_LATENCY);

		if (!IS_ERR(rawdelay_handle2_rpc_dmabuf)) {
			rtk_rpc_free_ion(rawdelay_handle2_rpc_dmabuf);
			rawdelay_handle2_rpc_dmabuf = NULL;
		}
		rawdelay_mem2 = NULL;
	}

	if (rawdelay_mem3) {
		if (!IS_ERR(rawdelay_handle3_rpc_dmabuf)) {
			rtk_rpc_free_ion(rawdelay_handle3_rpc_dmabuf);
			rawdelay_handle3_rpc_dmabuf = NULL;
		}
		rawdelay_mem3 = NULL;
	}

	if (stream->audioDecId != 0)
		destroyAudioComponent(stream);

	destroyRingBuf(stream);

	/* Free pre-allocate dma buffer at the final step */
	if (!IS_ERR(stream->rpc_dmabuf_rpc))
		rtk_rpc_free_ion(stream->rpc_dmabuf_rpc);

	if (!IS_ERR(rtk_compress_rpc_dmabuf)) {
		rtk_rpc_free_ion(rtk_compress_rpc_dmabuf);
		rtk_compress_rpc_dmabuf = NULL;
		stream = NULL;
	}

	delim_check_format = 1;
	kfree(header_handle_buf1);
	kfree(header_handle_buf2);

	ret_val = 0;
	return ret_val;
}

static unsigned long buf_memcpy2_ring(unsigned long base, unsigned long limit, unsigned long ptr, char *buf, unsigned long size)
{
	if (ptr + size <= limit) {
		if (copy_from_user((char *)ptr, buf, size))
			pr_info("%s %d there are remaining some data need to write\n", __func__, __LINE__);
	} else {
		int i = limit-ptr;

		if (copy_from_user((char *)ptr, (char *)buf, i))
			pr_info("%s %d there are remaining some data need to write\n", __func__, __LINE__);

		if (copy_from_user((char *)base, (char *)(buf+i), size-i))
			pr_info("%s %d there are remaining some data need to write\n", __func__, __LINE__);
	}

	ptr += size;
	if (ptr >= limit)
		ptr = base + (ptr-limit);

	return ptr;
}

static unsigned long buf_memcpy3_ring(unsigned long base, unsigned long limit, unsigned long ptr, char *buf, unsigned long size)
{
	if (ptr + size <= limit) {
		memcpy((char *)ptr, buf, size);
	} else {
		int i = limit-ptr;

		memcpy((char *)ptr, (char *)buf, i);
		memcpy((char *)base, (char *)(buf+i), size-i);
	}

	ptr += size;
	if (ptr >= limit)
		ptr = base + (ptr-limit);

	return ptr;
}

static int writeData(struct rtk_runtime_stream *stream, void *data, int len)
{
	unsigned long base, limit, wp;

	base = (unsigned long)stream->virDecInRing;
	limit = base + ntohl(stream->decInRingHeader.size);// sizeof(stream->phyDecInbandRing);
	wp = base + (unsigned long)(ntohl(stream->decInRingHeader.writePtr) - ntohl(stream->decInRingHeader.beginAddr));

	wp = buf_memcpy2_ring(base, limit, wp, (char *)data, (unsigned long)len);
	stream->decInRingHeader.writePtr = ntohl((int)(wp - base) + ntohl(stream->decInRingHeader.beginAddr));

	return len;
}

static int writeInbandCmd2(struct rtk_runtime_stream *stream, void *data, int len)
{
	unsigned long base, limit, wp;

	base = (unsigned long)stream->virInbandRing;
	limit = base + ntohl(stream->decInbandRingHeader.size);// sizeof(stream->phyDecInbandRing);
	wp = base + (unsigned long)(ntohl(stream->decInbandRingHeader.writePtr) - ntohl(stream->decInbandRingHeader.beginAddr));

	wp = buf_memcpy3_ring(base, limit, wp, (char *)data, (unsigned long)len);
	stream->decInbandRingHeader.writePtr = ntohl((int)(wp - base) + ntohl(stream->decInbandRingHeader.beginAddr));

	return len;
}

/*
 * For Netflix diretOutput Used
 * Need write PTS Inband Command and Data
 */
static int directWriteData(struct rtk_runtime_stream *stream, void *data, int len)
{
	char *cBuf = (char *)data;
	char *header_buf;
	char delim[4] = {0x55, 0x55, 0x00, 0x01};
	char delim_2[4] = {0x55, 0x55, 0x00, 0x02};
	char delim_3[4] = {0x55, 0x55, 0x00, 0x03};
	char *bufHeader = NULL;
	uint64_t timestamp = 0;
	long long timestampref = 0;
	struct AUDIO_DEC_PTS_INFO cmd;
	int remainingLen = len;
	int sizeInByte = 0;
	int writeLen = 0;
	int retLen = 0;
	int header_offset = 0;

	header_buf = kmalloc_array(len, sizeof(int), GFP_KERNEL);
	if (copy_from_user(header_buf, cBuf, len))
		pr_info("%s %d copy data fail\n", __func__, __LINE__);

	if (delim_format == 1)
		bufHeader = (char *)memmem(header_buf, remainingLen, delim_2, 4);
	else if (delim_format == 0)
		bufHeader = (char *)memmem(header_buf, remainingLen, delim, 4);
	else
		bufHeader = (char *)memmem(header_buf, remainingLen, delim_3, 4);

	/*
	 * Reset the hw_avsync_header_offset, if the data contains header.
	 */
	if (bufHeader != NULL && hw_avsync_header_offset > 0)
		hw_avsync_header_offset = 0;

	if (hw_avsync_header_offset != -1) { /* With hw av sync header */
		if (hw_avsync_header_offset > 0) {
			if (hw_avsync_header_offset <= remainingLen)
				writeLen = hw_avsync_header_offset;
			else
				writeLen = remainingLen;

			writeData(stream, cBuf, writeLen);
			retLen = retLen + writeLen;
			remainingLen = remainingLen - writeLen;
			hw_avsync_header_offset = hw_avsync_header_offset - writeLen;
			cBuf = cBuf + writeLen;
		}

		/*	DHCHERC-486:
		 *	1. Google has the new format for more than 2 channels.
		 *	2. If this data is the new format 0x55550002, it needs to record header length.
		 *	3. The header length will not the same in each video.
		 */
		if (delim_check_format) {
			bufHeader = (char *)memmem(header_buf, remainingLen, delim_3, 4);
			if (bufHeader != NULL) {
				delim_format = 2;
				delim_header_size = ((bufHeader[19] | ((0xff & bufHeader[18]) << 8) | ((0xff & bufHeader[17]) << 16) | ((0xff & bufHeader[16]) << 24)) & 0xffffffff);
				delim_check_format = 0;
				pr_info("%s %d delim_format %d = 0x55550003 delim_header_size %d\n", __func__, __LINE__, delim_format, delim_header_size);
			}

			bufHeader = (char *)memmem(header_buf, remainingLen, delim_2, 4);
			if (bufHeader != NULL) {
				delim_format = 1;
				delim_header_size = (int)bufHeader[19];
				delim_check_format = 0;
				pr_info("%s %d delim_format %d = 0x55550002 delim_header_size %d\n", __func__, __LINE__, delim_format, delim_header_size);
			}

			bufHeader = (char *)memmem(header_buf, remainingLen, delim, 4);
			if (bufHeader != NULL) {
				delim_format = 0;
				delim_check_format = 0;
				pr_info("%s %d delim_format %d = 0x55550001\n", __func__, __LINE__, delim_format);
			}
		}

		while (remainingLen > 0) {

			/* Copy the data for more than 1 header */
			kfree(header_buf);
			header_buf = kmalloc_array(remainingLen, sizeof(int), GFP_KERNEL);
			if (copy_from_user(header_buf, cBuf, remainingLen))
				pr_info("%s %d copy data fail\n", __func__, __LINE__);

			if (delim_format == 1) {
				if (header_handle_flag) {
					memcpy(header_handle_buf2 + header_handle_len, header_buf, sizeof(char *)*(delim_header_size - header_handle_len));
					bufHeader = (char *)memmem(header_handle_buf2, delim_header_size, delim_2, 4);
				} else {
					bufHeader = (char *)memmem(header_buf, remainingLen, delim_2, 4);
				}
			} else if (delim_format == 0) {
				if (header_handle_flag) {
					memcpy(header_handle_buf1 + header_handle_len, header_buf, sizeof(char *)*(16 - header_handle_len));
					bufHeader = (char *)memmem(header_handle_buf1, 16, delim, 4);
				} else {
					bufHeader = (char *)memmem(header_buf, remainingLen, delim, 4);
				}
			} else {
				bufHeader = (char *)memmem(header_buf, remainingLen, delim_3, 4);
			}

			if (bufHeader != NULL) {

				/*	DHCHERC-219:
				 *	1. There has some noise when multi-video playing specific video.
				 *	2. The header is 16 bytes, so add the handler when remainingLen is less than 16.
				 *	3. Saving the parts of header and merge to next loop.
				 */
				if (delim_format == 1) {
					if (remainingLen > 0 && remainingLen < delim_header_size) {
						memcpy(header_handle_buf2, header_buf, remainingLen);
						header_handle_len = remainingLen;
						header_handle_flag = 1;
						return retLen;
					}
				} else if (delim_format == 0) {
					if (remainingLen > 0 && remainingLen < 16) {
						memcpy(header_handle_buf1, header_buf, remainingLen);
						header_handle_len = remainingLen;
						header_handle_flag = 1;
						return retLen;
					}
				}

				/* Write header */
				memset(&cmd, 0, sizeof(cmd));
				sizeInByte = ((int)bufHeader[7] | ((int)bufHeader[6] << 8) | ((int)bufHeader[5] << 16) | ((int)bufHeader[4] << 24));
				cmd.header.type = htonl(AUDIO_DEC_INBAND_CMD_TYPE_PTS);
				cmd.header.size = htonl(sizeof(struct AUDIO_DEC_PTS_INFO));
				cmd.PTSH = ((bufHeader[11] | ((0xff & bufHeader[10]) << 8) | ((0xff & bufHeader[9]) << 16) | ((0xff & bufHeader[8]) << 24)) & 0xffffffff);
				cmd.PTSL = ((bufHeader[15] | ((0xff & bufHeader[14]) << 8) | ((0xff & bufHeader[13]) << 16) | ((0xff & bufHeader[12]) << 24)) & 0xffffffff);
				cmd.wPtr = stream->decInRingHeader.writePtr;
				timestamp = (((int64_t)cmd.PTSH << 32) | ((int64_t)cmd.PTSL & 0xffffffff)) * 9;
				timestamp = div64_ul(timestamp, 100000);
				cmd.PTSH = htonl((int32_t)(timestamp >> 32) & 0xffffffff);
				cmd.PTSL = htonl((int32_t)(timestamp & 0xffffffffLL));

				if (compress_first_pts) {
					timestampref = __cpu_to_be64(stream->refclock->audioSystemPTS);
					if ((long long)timestamp < timestampref) {
						stream->refclock->mastership.audioMode = AVSYNC_AUTO_MASTER;
						stream->refclock->mastership.videoMode = AVSYNC_AUTO_SLAVE;
						pr_info("[seek forward] Change to auto master mode\n");
					}
					compress_first_pts = false;
				}

				/* Print more message according user flag */
				if (log_lv >= 1) {
					if (data_len != sizeInByte) {
						printk(" old data len %d new data len %d \n", data_len, sizeInByte);
						data_len = sizeInByte;
					}
					if (log_lv == 3)
						printk("timestamp_90k 0x%llx data_len %d \n", timestamp ,data_len);
				}

				writeInbandCmd2(stream, &cmd, sizeof(cmd));

				/*	DHCHERC-486:
				 *	1. The position of header file may not at the first of data in some video.
				 *	2. The data before header file need to be send to buffer.
				 *	3. The offset of header file need to consider more about this case.
				 */
				header_offset = (int)(uintptr_t)(bufHeader - header_buf);
				if (header_offset > 0 && !header_handle_flag)
					writeData(stream, cBuf, header_offset);

				if (delim_format == 1) {
					if (header_handle_flag) {
						remainingLen = remainingLen - (delim_header_size - header_handle_len);
						cBuf = cBuf + (delim_header_size - header_handle_len);
						header_handle_flag = 0;
						header_handle_len = 0;
					} else {
						remainingLen = remainingLen - delim_header_size - header_offset;
						cBuf = cBuf + delim_header_size + header_offset;
					}
				} else if (delim_format == 0) {
					if (header_handle_flag) {
						remainingLen = remainingLen - (16 - header_handle_len);
						cBuf = cBuf + (16 - header_handle_len);
						header_handle_flag = 0;
						header_handle_len = 0;
					} else {
						remainingLen = remainingLen - 16 - header_offset;
						cBuf = cBuf + 16 + header_offset;
					}
				} else {
					cBuf = cBuf + delim_header_size;
					return 0;
				}

				/* Write data */
				if (remainingLen > sizeInByte)
					writeLen = sizeInByte;
				else
					writeLen = remainingLen;

				writeData(stream, cBuf, writeLen);
				retLen = retLen + writeLen;
				remainingLen = remainingLen - writeLen;
				hw_avsync_header_offset = hw_avsync_header_offset + (sizeInByte - writeLen);
				cBuf = cBuf + writeLen;
			} else {
				if (remainingLen > 0)
					writeLen = remainingLen;
				else
					writeLen = 0;

				writeData(stream, cBuf, writeLen);
				retLen = retLen + writeLen;
				remainingLen = remainingLen - writeLen;
				cBuf = cBuf + writeLen;
			}
		}
	} else { /* without hw av sync header */
		if (remainingLen > 0)
			writeLen = remainingLen;
		else
			writeLen = 0;

		writeData(stream, cBuf, writeLen);
		retLen = retLen + writeLen;
		remainingLen = remainingLen - writeLen;
		cBuf = cBuf + writeLen;
	}

	kfree(header_buf);

	return retLen;
}

/* The setting for different channel mapping.
 * This will send rpc for AFW.
 */
static unsigned int snd_choose_channel_mapping(int num_channels)
{
	unsigned int channel_mapping_info;

	switch (num_channels) {
	case 1:
		channel_mapping_info = (0x1) << 6;
		break;
	case 2:
		channel_mapping_info = (0x3) << 6;
		break;
	case 4:
		channel_mapping_info = (0x33) << 6;
		break;
	case 6:
		channel_mapping_info = (0x3F) << 6;
		break;
	case 8:
		channel_mapping_info = (0x63F) << 6;
		break;
	default:
		channel_mapping_info = (0x3) << 6;
		break;
	}

	return channel_mapping_info;
}

static int snd_card_compr_set_params(struct snd_compr_stream *cstream, struct snd_compr_params *params)
{
	struct rtk_runtime_stream *stream = cstream->runtime->private_data;
	struct AUDIO_RPC_SENDIO sendio;
	struct AUDIO_DEC_NEW_FORMAT cmd;
	unsigned int buffer_size;

	buffer_size = params->buffer.fragment_size * params->buffer.fragments + 16;

	/* bit[2] and bit[1] contain message log level */
	data_len = 0;
	if (params->codec.reserved[1] & 0x6) {
		log_lv = (params->codec.reserved[1] & 0x6) >> 1;
		printk("snd-realtek-compress log_lv %d", log_lv);
	} else
		log_lv = 0;

	stream->isLowWater = false;
	if (params->codec.reserved[1] & 0x1) {
		/*
		 * bit 0 is for low waterlevel mode
		 * must be set before creating ring buffer
		 */
		stream->isLowWater = true;
	}

	if (createRingBuf(stream, buffer_size)) {
		pr_err("[%s %d] create Audio Component failed\n", __func__, __LINE__);
		return -1;
	}

	/*
	 * For Netflix
	 * Previous refclock_handle should free by audio hal
	 */
	if (params->codec.reserved[0] > 0) { /* with hw av sync */
		struct AUDIO_RPC_REFCLOCK audioRefClock;

		refclock_from_video = 1;
		refclock_handle = params->codec.reserved[0];
		if (refclock_handle < 0) {
			pr_err("[%s %d ion_alloc fail]\n", __func__, __LINE__);
			return -1;
		}
		pr_info("refclock_handle:%d\n", refclock_handle);

		refclock_rpc_dmabuf = dma_buf_get(refclock_handle);
		if (IS_ERR(refclock_rpc_dmabuf)) {
			pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
			return PTR_ERR(refclock_rpc_dmabuf);
		}

		stream->phyRefclock = rtk_rpc_ion_pa(refclock_rpc_dmabuf->priv);
		if (stream->phyRefclock == -1) {
			pr_err("[%s %d] alloc memory faild\n", __func__, __LINE__);
			return -1;
		}

		pr_info("stream->phyRefclock:%lx\n", (unsigned long)stream->phyRefclock);

		stream->refclock = (struct REFCLOCK *)rtk_rpc_ion_va(refclock_rpc_dmabuf->priv);

		stream->refclock->mastership.systemMode = AVSYNC_FORCED_SLAVE;
		stream->refclock->mastership.audioMode = AVSYNC_FORCED_MASTER;
		stream->refclock->mastership.videoMode = AVSYNC_FORCED_SLAVE;
		stream->refclock->mastership.masterState = AUTOMASTER_NOT_MASTER;
		stream->refclock->videoFreeRunThreshold = htonl(0x7FFFFFFF);
		stream->refclock->audioFreeRunThreshold = htonl(0x7FFFFFFF);

		audioRefClock.instanceID = stream->audioOutId;
		audioRefClock.pRefClockID = stream->audioAppPinId;
		audioRefClock.pRefClock = (long)stream->phyRefclock;

		if (RPC_TOAGENT_SETREFCLOCK(stream->phy_addr_rpc,
					stream->vaddr_rpc, &audioRefClock)) {
			pr_err("[%s %d]\n", __func__, __LINE__);
			return -1;
		}

		hw_avsync_header_offset = 0;
		pr_info("%s %d\n", __func__, __LINE__);
	} else {
		/* without hw av sync */
		//set refclock
		struct AUDIO_RPC_REFCLOCK audioRefClock;

		refclock_from_video = 0;
		refclock_rpc_dmabuf = ion_alloc(sizeof(stream->refclock),
					RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
		if (IS_ERR(refclock_rpc_dmabuf)) {
			pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
			return PTR_ERR(refclock_rpc_dmabuf);
		}

		stream->phyRefclock = rtk_rpc_ion_pa(refclock_rpc_dmabuf->priv);
		if (stream->phyRefclock == -1) {
			pr_err("[%s %d] alloc memory faild\n", __func__, __LINE__);
			return -1;
		}
		stream->refclock = (struct REFCLOCK *)rtk_rpc_ion_va(refclock_rpc_dmabuf->priv);
		memset(stream->refclock, 0, sizeof(struct REFCLOCK));

		stream->refclock->mastership.systemMode = AVSYNC_FORCED_SLAVE;
		stream->refclock->mastership.audioMode = AVSYNC_FORCED_MASTER;
		stream->refclock->mastership.videoMode = AVSYNC_FORCED_SLAVE;
		stream->refclock->mastership.masterState = AVSYNC_FORCED_SLAVE;
		stream->refclock->videoFreeRunThreshold = htonl(0x7FFFFFFF);
		stream->refclock->audioFreeRunThreshold = htonl(0x7FFFFFFF);

		audioRefClock.instanceID = stream->audioOutId;
		audioRefClock.pRefClockID = stream->audioAppPinId;
		audioRefClock.pRefClock = (long)stream->phyRefclock;

		if (RPC_TOAGENT_SETREFCLOCK(stream->phy_addr_rpc,
					stream->vaddr_rpc, &audioRefClock)) {
			pr_err("[%s %d]\n", __func__, __LINE__);
			return -1;
		}

		hw_avsync_header_offset = -1;
	}

	stream->codecId = params->codec.id;
	stream->audioChannel = params->codec.ch_in;
	stream->audioSamplingRate = params->codec.sample_rate;

	// decoder flush
	sendio.instanceID = stream->audioDecId;
	sendio.pinID = stream->audioDecPinId;
	if (RPC_TOAGENT_FLUSH_SVC(stream->phy_addr_rpc, stream->vaddr_rpc,
				&sendio)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	}

	//run audio component
	triggerAudio(stream, SNDRV_PCM_TRIGGER_START);

	//send audio format to inband cmd
	memset(&cmd, 0, sizeof(cmd));

	switch (params->codec.id) {
	case SND_AUDIOCODEC_PCM:
		pr_info("SND_AUDIOCODEC_PCM\n");
		cmd.audioType = htonl(AUDIO_LPCM_DECODER_TYPE);
		cmd.privateInfo[0] = htonl(params->codec.ch_in);
		cmd.privateInfo[1] = htonl(16);
		cmd.privateInfo[2] = htonl(params->codec.sample_rate);
		cmd.privateInfo[3] = htonl(0);
		// privateInfo[4]&0x3FFFC0) >> 6; // bit[6:21] for wave channel mask
		cmd.privateInfo[4] = htonl(snd_choose_channel_mapping(params->codec.ch_in));
		cmd.privateInfo[5] = htonl(0);
		cmd.privateInfo[6] = htonl(0);
		cmd.privateInfo[7] = htonl(AUDIO_LITTLE_ENDIAN);
		break;
	case SND_AUDIOCODEC_MP3:
		pr_info("SND_AUDIO_MP3\n");
		cmd.audioType = htonl(AUDIO_MPEG_DECODER_TYPE);
		break;
	case SND_AUDIOCODEC_AAC:
		pr_info("SND_AUDIO_AAC\n");
		cmd.audioType = htonl(AUDIO_RAW_AAC_DECODER_TYPE);
		cmd.privateInfo[0] = htonl(params->codec.ch_in);
		cmd.privateInfo[1] = htonl(GetSampleIndex(params->codec.sample_rate));
		cmd.privateInfo[2] = htonl(OBJT_TYPE_LC);
		cmd.privateInfo[3] = htonl(0);
		break;
	case SND_AUDIOCODEC_AC3:
		pr_info("SND_AUDIO_AC3\n");
		cmd.audioType = htonl(AUDIO_AC3_DECODER_TYPE);
		break;
	case SND_AUDIOCODEC_EAC3:
		pr_info("SND_AUDIO_EAC3\n");
		cmd.audioType = htonl(AUDIO_DDP_DECODER_TYPE);
		break;
	case SND_AUDIOCODEC_DTS:
		pr_info("SND_AUDIO_DTS\n");
		cmd.audioType = htonl(AUDIO_DTS_DECODER_TYPE);
		break;
	case SND_AUDIOCODEC_DTS_HD:
		pr_info("SND_AUDIO_DTS_HD\n");
		cmd.audioType = htonl(AUDIO_DTS_HD_DECODER_TYPE);
		break;
	case SND_AUDIOCODEC_TRUEHD:
		pr_info("SND_AUDIO_TRUEHD\n");
		cmd.audioType = htonl(AUDIO_MLP_DECODER_TYPE);
		RPC_TOAGENT_SET_TRUEHD_ERR_SELF_RESET(stream->phy_addr_rpc,
					stream->vaddr_rpc,true);
		break;
	case SND_AUDIOCODEC_IEC61937:
		pr_info("SND_AUDIOCODEC_IEC61937\n");
		cmd.audioType = htonl(AUDIO_LPCM_DECODER_TYPE);
		cmd.privateInfo[0] = htonl(params->codec.ch_in);
		cmd.privateInfo[1] = htonl(16);
		cmd.privateInfo[2] = htonl(params->codec.sample_rate);
		cmd.privateInfo[3] = htonl(0);
		cmd.privateInfo[4] = htonl(0x1<<26);  // IEC61937 set bit26 to 1.
		cmd.privateInfo[5] = htonl(0);
		cmd.privateInfo[6] = htonl(0);  // (a_format->privateData[0]&0x1)<<4 : float or not
		cmd.privateInfo[7] = htonl(AUDIO_LITTLE_ENDIAN);
		break;
	default:
		pr_err("[%s %d] audio format not support\n", __func__, __LINE__);
		break;
	}

	cmd.header.type = htonl(AUDIO_DEC_INBAMD_CMD_TYPE_NEW_FORMAT);
	cmd.header.size = htonl(sizeof(struct AUDIO_DEC_NEW_FORMAT));
	cmd.wPtr = htonl(stream->phyDecInRing);
#ifdef CONFIG_64BIT
	writeInbandCmd2(stream, &cmd, sizeof(struct AUDIO_DEC_NEW_FORMAT));
#else
	snd_realtek_hw_ring_write(&stream->decInbandRingHeader, &cmd, sizeof(struct AUDIO_DEC_NEW_FORMAT),
		(unsigned long)stream->virInbandRing - (unsigned long)stream->phyDecInbandRing);
#endif

	return 0;
}

static int snd_card_compr_set_metadata(
			struct snd_compr_stream *cstream,
			struct snd_compr_metadata *metadata)
{
	pr_info("%s %d\n", __func__, __LINE__);

	return 0;
}

static int snd_card_compr_trigger(struct snd_compr_stream *cstream, int cmd)
{
	int rawDelay;
	struct rtk_runtime_stream *stream = cstream->runtime->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (triggerAudio(stream, cmd))
			pr_err("[%s %d fail]\n", __func__, __LINE__);
		break;
	case SND_COMPR_TRIGGER_DRAIN:
	case SND_COMPR_TRIGGER_PARTIAL_DRAIN:
		pr_info("TRIGGER_DRAIN/TRIGGER_PARTIAL_DRAIN\n");
		break;
	case SND_COMPR_TRIGGER_NEXT_TRACK:
		pr_info("TRIGGER_NEXT_TRACK\n");
		break;
	case SND_COMPR_TRIGGER_GET_LATENCY:
		//rawDelay = snd_monitor_raw_data_queue();
		rawDelay = snd_monitor_raw_data_queue_new(stream);
		return rawDelay;
	default:
		return -EINVAL;
	}

	return 0;
}

static int snd_card_compr_pointer(struct snd_compr_stream *cstream, struct snd_compr_tstamp *tstamp)
{
	struct rtk_runtime_stream *stream = cstream->runtime->private_data;
	unsigned char *ptswp = stream->virDwnRing +
		(ntohl(stream->dwnstrmRingHeader.writePtr) - ntohl(stream->dwnstrmRingHeader.beginAddr));
	unsigned char *ptsrp = stream->virDwnRing +
		(ntohl(stream->dwnstrmRingHeader.readPtr[0]) - ntohl(stream->dwnstrmRingHeader.beginAddr));
	unsigned long space = 0;
	unsigned long frameSize = 0;
	unsigned int decfrm = 0;
	long cosume_Size = 0;
	long now_rp;

	//unsigned long queueFrames;
	if (!stream->isGetInfo) {
		if (!checkAudioInfo(stream))
			stream->isGetInfo = false;
		else {
			ptswp = stream->virDwnRing +
				(ntohl(stream->dwnstrmRingHeader.writePtr) - ntohl(stream->dwnstrmRingHeader.beginAddr));
			ptsrp = stream->virDwnRing +
				(ntohl(stream->dwnstrmRingHeader.readPtr[0]) - ntohl(stream->dwnstrmRingHeader.beginAddr));
		}
	}

	if (ptswp != ptsrp) {
		do {
			enum AUDIO_INBAND_CMD_TYPE infoType;

			memset(&infoType, 0, sizeof(enum AUDIO_INBAND_CMD_TYPE));
			GetBufferFromRing(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp, (unsigned char *)&infoType, sizeof(enum AUDIO_INBAND_CMD_TYPE));

			if (infoType == AUDIO_DEC_INBAND_CMD_TYPE_PRIVATE_INFO) {
				//skip private info
				struct AUDIO_INBAND_PRIVATE_INFO inbandInfo;

				memset(&inbandInfo, 0, INBAND_INFO_SIZE);
				space = GetBufferFromRing(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp, (unsigned char *)&inbandInfo, INBAND_INFO_SIZE);
				UpdateRingPtr(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp, stream, INBAND_INFO_SIZE, space);
				//skip pcminfo or channel index info
				if (inbandInfo.infoType == AUDIO_INBAND_CMD_PRIVATE_PCM_FMT) {
					ptsrp = stream->virDwnRing + (ntohl(stream->dwnstrmRingHeader.readPtr[0]) - ntohl(stream->dwnstrmRingHeader.beginAddr));
					space = stream->virDwnRingUpper - ptsrp;
					UpdateRingPtr(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp, stream, INBAND_PCM_SIZE, space);
				} else if (inbandInfo.infoType == AUDIO_INBAND_CMD_PRIVATE_CH_IDX) {
					ptsrp = stream->virDwnRing + (ntohl(stream->dwnstrmRingHeader.readPtr[0]) - ntohl(stream->dwnstrmRingHeader.beginAddr));
					space = stream->virDwnRingUpper - ptsrp;
					UpdateRingPtr(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp, stream, INBAND_INDEX_SIZE, space);
				} else
					break;
			} else if (infoType == AUDIO_DEC_INBAND_CMD_TYPE_PTS) {
				//get ptsinfo
				struct AUDIO_DEC_PTS_INFO pPTSInfo;

				memset(&pPTSInfo, 0, INBAND_PTS_INFO_SIZE);
				space = GetBufferFromRing(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp, (unsigned char *)&pPTSInfo, INBAND_PTS_INFO_SIZE);

				//wPtr is used to indicate the audio frame length
				if (pPTSInfo.wPtr != 0) {
					frameSize += pPTSInfo.wPtr;
					UpdateRingPtr(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp, stream, INBAND_PTS_INFO_SIZE, space);
				} else
					break;
			} else if (infoType == AUDIO_DEC_INBAND_CMD_TYPE_EOS) {
				struct AUDIO_DEC_EOS pEOSInfo;

				memset(&pEOSInfo, 0, INBAND_EOS_SIZE);
				space = GetBufferFromRing(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp, (unsigned char *)&pEOSInfo, INBAND_EOS_SIZE);
				UpdateRingPtr(stream->virDwnRingUpper, stream->virDwnRingLower, ptsrp, stream, INBAND_EOS_SIZE, space);
				if (pEOSInfo.header.size == INBAND_EOS_SIZE && pEOSInfo.EOSID == -1)
					break;
			} else
				break;

			ptswp = stream->virDwnRing +
				(ntohl(stream->dwnstrmRingHeader.writePtr) - ntohl(stream->dwnstrmRingHeader.beginAddr));
			ptsrp = stream->virDwnRing +
				(ntohl(stream->dwnstrmRingHeader.readPtr[0]) - ntohl(stream->dwnstrmRingHeader.beginAddr));

		} while (ptswp != ptsrp);
	}

	//to calculate to total read into decoder.
	now_rp = (long)ntohl(stream->decInRingHeader.readPtr[0]);
	if (now_rp != stream->lastInRingRP) {
		long rp = stream->lastInRingRP;
		long lower = (long)ntohl(stream->decInRingHeader.beginAddr);
		long upper = lower + (long)ntohl(stream->decInRingHeader.size);

		cosume_Size = ringValidData(lower, upper, rp, now_rp);
		stream->lastInRingRP = now_rp;
	}

	stream->comsume_total += cosume_Size;
	tstamp->copied_total = stream->comsume_total;
	tstamp->byte_offset = tstamp->copied_total % (u32)cstream->runtime->buffer_size;

	if (frameSize)
		stream->outFrames += (frameSize >> 2);

	//for IOCTL TSTAMP to get render position in user space
	decfrm = htonl(rawdelay_mem2->decfrm_smpl);
	if (decfrm == 0)
		tstamp->pcm_io_frames = stream->outFrames;
	else
		tstamp->pcm_io_frames = htonl(rawdelay_mem2->decfrm_smpl);

	tstamp->sampling_rate = stream->audioSamplingRate;

	return 0;
}

static int snd_card_compr_copy(struct snd_compr_stream *cstream, char __user *buf, size_t count)
{
	struct rtk_runtime_stream *stream = cstream->runtime->private_data;
	//copy data to input ringbuffer
	long wp = (long)ntohl(stream->decInRingHeader.writePtr);
	long rp = (long)ntohl(stream->decInRingHeader.readPtr[0]);
	long lower = (long)ntohl(stream->decInRingHeader.beginAddr);
	long upper = lower + (long)ntohl(stream->decInRingHeader.size);
	long writableSize = 0;
	int bufFullCount = 0;
	int write_frame = 0;

	writableSize = validFreeSize(lower, upper, rp, wp);
	while (count > writableSize) {
		if (bufFullCount++ > 5)
			return 0;

		usleep_range(500, 1000);
		rp = (long) ntohl(stream->decInRingHeader.readPtr[0]);
		writableSize = validFreeSize(lower, upper, rp, wp);
	}

	/*
	 * With hw av sync header       : Write header & data into different buffer
	 * Without hw av sync header    : Write data only
	 */
	write_frame = directWriteData(stream, (char *)buf, count);
	stream->copied_total += count;

	/*
	 * With hw av sync header       : count = write_frame + header
	 * Without hw av sync header    : count = write_frame
	 */
	stream->comsume_total += (count - write_frame);

	return count;
}

static int snd_card_compr_get_caps(struct snd_compr_stream *cstream, struct snd_compr_caps *caps)
{
	caps->num_codecs = NUM_CODEC;
	caps->direction = SND_COMPRESS_PLAYBACK;
	caps->min_fragment_size = MIN_FRAGMENT_SIZE;
	caps->max_fragment_size = MAX_FRAGMENT_SIZE;
	caps->min_fragments = MIN_FRAGMENT;
	caps->max_fragments = MAX_FRAGMENT;
	caps->codecs[0] = SND_AUDIOCODEC_MP3;
	caps->codecs[1] = SND_AUDIOCODEC_AAC;
	caps->codecs[2] = SND_AUDIOCODEC_AC3;
	caps->codecs[3] = SND_AUDIOCODEC_DTS;
	caps->codecs[4] = SND_AUDIOCODEC_DTS_HD;
	caps->codecs[5] = SND_AUDIOCODEC_EAC3;
	caps->codecs[6] = SND_AUDIOCODEC_TRUEHD;

	return 0;
}

static struct snd_compr_codec_caps caps_mp3 = {
	.num_descriptors = 1,
	.descriptor[0].max_ch = 2,
	.descriptor[0].sample_rates[0] = 48000,
	.descriptor[0].sample_rates[1] = 44100,
	.descriptor[0].sample_rates[2] = 32000,
	.descriptor[0].sample_rates[3] = 16000,
	.descriptor[0].sample_rates[4] = 8000,
	.descriptor[0].num_sample_rates = 5,
	.descriptor[0].bit_rate[0] = 320,
	.descriptor[0].bit_rate[1] = 192,
	.descriptor[0].num_bitrates = 2,
	.descriptor[0].profiles = 0,
	.descriptor[0].modes = SND_AUDIOCHANMODE_MP3_STEREO,
	.descriptor[0].formats = 0,
};

static struct snd_compr_codec_caps caps_aac = {
	.num_descriptors = 1,
	.descriptor[0].max_ch = 8,
	.descriptor[0].sample_rates[0] = 48000,
	.descriptor[0].sample_rates[1] = 44100,
	.descriptor[0].sample_rates[2] = 32000,
	.descriptor[0].sample_rates[3] = 16000,
	.descriptor[0].sample_rates[4] = 8000,
	.descriptor[0].num_sample_rates = 5,
	.descriptor[0].bit_rate[0] = 320,
	.descriptor[0].bit_rate[1] = 192,
	.descriptor[0].num_bitrates = 2,
	.descriptor[0].profiles = 0,
	.descriptor[0].modes = 0,
	.descriptor[0].formats = 0, //(SND_AUDIOSTREAMFORMAT_MP2ADTS | SND_AUDIOSTREAMFORMAT_MP4ADTS),
};

static struct snd_compr_codec_caps caps_ac3 = {
	.num_descriptors = 1,
	.descriptor[0].max_ch = 8,
	.descriptor[0].sample_rates[0] = 48000,
	.descriptor[0].sample_rates[1] = 44100,
	.descriptor[0].sample_rates[2] = 32000,
	.descriptor[0].sample_rates[3] = 16000,
	.descriptor[0].sample_rates[4] = 8000,
	.descriptor[0].num_sample_rates = 5,
	.descriptor[0].bit_rate[0] = 320,
	.descriptor[0].bit_rate[1] = 192,
	.descriptor[0].num_bitrates = 2,
	.descriptor[0].profiles = 0,
	.descriptor[0].modes = 0,
	.descriptor[0].min_buffer = 8192,
	.descriptor[0].formats = 0,
};

static struct snd_compr_codec_caps caps_eac3 = {
	.num_descriptors = 1,
	.descriptor[0].max_ch = 8,
	.descriptor[0].sample_rates[0] = 48000,
	.descriptor[0].sample_rates[1] = 44100,
	.descriptor[0].sample_rates[2] = 32000,
	.descriptor[0].sample_rates[3] = 16000,
	.descriptor[0].sample_rates[4] = 8000,
	.descriptor[0].num_sample_rates = 5,
	.descriptor[0].bit_rate[0] = 320,
	.descriptor[0].bit_rate[1] = 192,
	.descriptor[0].num_bitrates = 2,
	.descriptor[0].profiles = 0,
	.descriptor[0].modes = 0,
	.descriptor[0].min_buffer = 8192,
	.descriptor[0].formats = 0,
};

static struct snd_compr_codec_caps caps_dts = {
	.num_descriptors = 1,
	.descriptor[0].max_ch = 8,
	.descriptor[0].sample_rates[0] = 48000,
	.descriptor[0].sample_rates[1] = 44100,
	.descriptor[0].sample_rates[2] = 32000,
	.descriptor[0].sample_rates[3] = 16000,
	.descriptor[0].sample_rates[4] = 8000,
	.descriptor[0].num_sample_rates = 5,
	.descriptor[0].bit_rate[0] = 320,
	.descriptor[0].bit_rate[1] = 192,
	.descriptor[0].num_bitrates = 2,
	.descriptor[0].profiles = 0,
	.descriptor[0].modes = 0,
	.descriptor[0].min_buffer = 8192,
	.descriptor[0].formats = 0,
};

static struct snd_compr_codec_caps caps_dtsHD = {
	.num_descriptors = 1,
	.descriptor[0].max_ch = 8,
	.descriptor[0].sample_rates[0] = 48000,
	.descriptor[0].sample_rates[1] = 44100,
	.descriptor[0].sample_rates[2] = 32000,
	.descriptor[0].sample_rates[3] = 16000,
	.descriptor[0].sample_rates[4] = 8000,
	.descriptor[0].num_sample_rates = 5,
	.descriptor[0].bit_rate[0] = 320,
	.descriptor[0].bit_rate[1] = 192,
	.descriptor[0].num_bitrates = 2,
	.descriptor[0].profiles = 0,
	.descriptor[0].modes = 0,
	.descriptor[0].min_buffer = 8192,
	.descriptor[0].formats = 0,
};

static struct snd_compr_codec_caps caps_trueHD = {
	.num_descriptors = 1,
	.descriptor[0].max_ch = 8,
	.descriptor[0].sample_rates[0] = 48000,
	.descriptor[0].sample_rates[1] = 44100,
	.descriptor[0].sample_rates[2] = 32000,
	.descriptor[0].sample_rates[3] = 16000,
	.descriptor[0].sample_rates[4] = 8000,
	.descriptor[0].num_sample_rates = 5,
	.descriptor[0].bit_rate[0] = 320,
	.descriptor[0].bit_rate[1] = 192,
	.descriptor[0].num_bitrates = 2,
	.descriptor[0].profiles = 0,
	.descriptor[0].modes = 0,
	.descriptor[0].min_buffer = 8192,
	.descriptor[0].formats = 0,
};

static int snd_card_compr_get_codec_caps(struct snd_compr_stream *cstream, struct snd_compr_codec_caps *codec)
{
	pr_info("%s %d codec %x\n", __func__, __LINE__, codec->codec);

	switch (codec->codec) {
	case SND_AUDIOCODEC_MP3:
		*codec = caps_mp3;
		break;
	case SND_AUDIOCODEC_AAC:
		*codec = caps_aac;
		break;
	case SND_AUDIOCODEC_AC3:
		*codec = caps_ac3;
		break;
	case SND_AUDIOCODEC_EAC3:
		*codec = caps_eac3;
		break;
	case SND_AUDIOCODEC_DTS:
		*codec = caps_dts;
		break;
	case SND_AUDIOCODEC_DTS_HD:
		*codec = caps_dtsHD;
		break;
	case SND_AUDIOCODEC_TRUEHD:
		*codec = caps_trueHD;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int snd_monitor_raw_data_queue(void)
{
	int rawOutDelay = 0;

	if (rawdelay_mem2) {
		rawOutDelay = htonl(rawdelay_mem2->latency) / 1000;
	} else if (rawdelay_mem) {
		rawOutDelay = *rawdelay_mem;
		rawOutDelay = htonl(rawOutDelay);
	} else {
		rawOutDelay = mtotal_latency;
	}

	return rawOutDelay;
}
EXPORT_SYMBOL(snd_monitor_raw_data_queue);

static struct snd_compr_ops snd_card_rtk_compr_ops = {
	.open           = snd_card_compr_open,
	.free           = snd_card_compr_free,
	.set_params     = snd_card_compr_set_params,
	.get_params     = NULL, //snd_card_compr_get_params,
	.set_metadata   = snd_card_compr_set_metadata,
	.get_metadata   = NULL, //snd_card_compr_get_metadata,
	.trigger        = snd_card_compr_trigger,
	.pointer        = snd_card_compr_pointer,
	.copy           = snd_card_compr_copy,
	.ack            = NULL,
	.get_caps       = snd_card_compr_get_caps,
	.get_codec_caps = snd_card_compr_get_codec_caps
};

int snd_card_create_compress_instance(struct RTK_snd_card *pSnd, int instance_idx)
{
	struct snd_compr *compr = NULL;
	int ret = 0;
	int direction = SND_COMPRESS_PLAYBACK;

	compr = kzalloc(sizeof(*compr), GFP_KERNEL);
	if (compr == NULL)
		return -ENOMEM;

	compr->ops = kzalloc(sizeof(snd_card_rtk_compr_ops), GFP_KERNEL);
	if (compr->ops == NULL) {
		ret = -ENOMEM;
		goto compr_err;
	}
	memcpy(compr->ops, &snd_card_rtk_compr_ops, sizeof(snd_card_rtk_compr_ops));

	mutex_init(&compr->lock);
	ret = snd_compress_new(pSnd->card, instance_idx, direction, "rtk_snd", compr);
	if (ret < 0) {
		pr_err("%s failed\n", __func__);
		goto compr_err;
	}

	pSnd->compr = compr;
	compr->private_data = pSnd;

	/* Setup some global variable to NULL */
	rawdelay_mem = NULL;
	rawdelay_mem2 = NULL;
	rawdelay_mem3 = NULL;

	return 0;

compr_err:
	if (compr->ops != NULL)
		kfree(compr->ops);

	if (compr != NULL)
		kfree(compr);

	return ret;
}
EXPORT_SYMBOL(snd_card_create_compress_instance);

MODULE_LICENSE("GPL v2");
