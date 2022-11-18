// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2017-2020 Realtek Semiconductor Corp.
 */

#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/fdtable.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mpage.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/syscalls.h> /* needed for the _IOW etc stuff used later */
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/of_address.h>
#include <linux/ion.h>
#include <sound/asound.h>
#include <sound/control.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <trace/events/rtk_rpc.h>
#include <rtk_rpc.h>
#include <soc/realtek/rtk_refclk.h>

#include "snd-realtek-compress.h"
#include "snd-realtek.h"
#include <asm/cacheflush.h>
#include <ion_rtk_alloc.h>

#define PCM_SUBSTREAM_CHECK(sub) snd_BUG_ON(!(sub) || !(sub)->runtime)
#define ion_alloc ext_rtk_ion_alloc
#define SHARE_MEM_SIZE 8
#define SHARE_MEM_SIZE_LATENCY sizeof(struct ALSA_LATENCY_INFO)

#define SND_REALTEK_DRIVER_HDMI_IN "snd_alsa_rtk"
#define SND_REALTEK_DRIVER_I2S_IN "snd_alsa_rtk_i2s_in"
#define SND_REALTEK_DRIVER_NONPCM_IN "snd_alsa_rtk_nonpcm_in"
#define SND_REALTEK_DRIVER_AUDIO_IN "snd_alsa_rtk_audio_in"
#define SND_REALTEK_DRIVER_AUDIO_V2_IN "snd_alsa_rtk_audio_v2_in"
#define SND_REALTEK_DRIVER_AUDIO_V3_IN "snd_alsa_rtk_audio_v3_in"
#define SND_REALTEK_DRIVER_AUDIO_V4_IN "snd_alsa_rtk_audio_v4_in"
#define SND_REALTEK_DRIVER_I2S_LOOPBACK_IN "snd_alsa_rtk_i2s_loopback_in"
#define SND_REALTEK_DRIVER_DMIC_PASSTHROUGH_IN "snd_alsa_rtk_dmic_passthrough_in"
#define SND_REALTEK_DRIVER_PURE_DMIC_IN "snd_alsa_rtk_pure_dmic_in"

static int snd_card_playback_open(struct snd_pcm_substream *substream);
static int snd_card_playback_close(struct snd_pcm_substream *substream);
static int snd_card_playback_ioctl(struct snd_pcm_substream *substream,  unsigned int cmd, void *arg);
static int snd_card_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params);
static int snd_card_hw_free(struct snd_pcm_substream *substream);
static int snd_card_playback_prepare(struct snd_pcm_substream *substream);
static int snd_card_playback_trigger(struct snd_pcm_substream *substream, int cmd);
static int snd_card_playback_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *area);
static snd_pcm_uframes_t snd_card_playback_pointer(struct snd_pcm_substream *substream);

static int snd_card_capture_open(struct snd_pcm_substream *substream);
static int snd_card_capture_close(struct snd_pcm_substream *substream);
static int snd_card_capture_ioctl(struct snd_pcm_substream *substream,  unsigned int cmd, void *arg);
static int snd_card_capture_prepare(struct snd_pcm_substream *substream);
static int snd_card_capture_trigger(struct snd_pcm_substream *substream, int cmd);
static int snd_card_capture_hw_free(struct snd_pcm_substream *substream);
static snd_pcm_uframes_t snd_card_capture_pointer(struct snd_pcm_substream *substream);
static unsigned int snd_capture_monitor_delay(struct snd_pcm_substream *substream);
static int snd_card_capture_get_time_info(struct snd_pcm_substream *substream,
			struct timespec64 *system_ts, struct timespec64 *audio_ts,
			struct snd_pcm_audio_tstamp_config *audio_tstamp_config,
			struct snd_pcm_audio_tstamp_report *audio_tstamp_report);
static int snd_realtek_hw_capture_malloc_pts_ring(struct snd_pcm_runtime *runtime);
static int snd_realtek_hw_capture_init_PTS_ringheader_of_AI(struct snd_pcm_runtime *runtime);
static void snd_card_capture_setup_pts(struct snd_pcm_runtime *runtime, struct AUDIO_DEC_PTS_INFO *pPkt);
static int snd_realtek_capture_check_hdmirx_enable(void);
char *snd_realtek_capture_get_stream_name(void);
static void snd_card_capture_handle_HDMI_plug_out(struct snd_pcm_substream *substream);


static enum hrtimer_restart snd_card_timer_function(struct hrtimer *timer);
static enum hrtimer_restart snd_card_capture_lpcm_timer_function(struct hrtimer *timer);
static void snd_card_capture_calculate_pts(struct snd_pcm_runtime *runtime, long nPeriodCount);

static unsigned long valid_free_size(unsigned long base, unsigned long limit, unsigned long rp, unsigned long wp);
static unsigned long ring_add(unsigned long ring_base, unsigned long ring_limit, unsigned long ptr, unsigned int bytes);
static unsigned long ring_valid_data(unsigned long ring_base, unsigned long ring_limit, unsigned long ring_rp, unsigned long ring_wp);
static unsigned long buf_memcpy2_ring(unsigned long base, unsigned long limit, unsigned long ptr, char *buf, unsigned long size);
static int ring_check_ptr_valid_32(unsigned int ring_rp, unsigned int ring_wp, unsigned int ptr);
static long ring_memcpy2_buf(char *buf, unsigned long base, unsigned long limit, unsigned long ptr, unsigned int size);
static unsigned long ring_minus(unsigned long ring_base, unsigned long ring_limit, unsigned long ptr, int bytes);

static int snd_realtek_hw_free_ring(struct snd_pcm_runtime *runtime);
static int snd_RTK_volume_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);
static int snd_RTK_volume_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int snd_RTK_volume_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int snd_RTK_capsrc_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);
static int snd_RTK_capsrc_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int snd_RTK_capsrc_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);

static int snd_realtek_hw_capture_free_ring(struct snd_pcm_runtime *runtime);
static long snd_card_get_ring_data(struct RINGBUFFER_HEADER *pRing_BE, struct RINGBUFFER_HEADER *pRing_LE);
static void ring1_to_ring2_general_64(struct AUDIO_RINGBUF_PTR_64 *ring1, struct AUDIO_RINGBUF_PTR_64 *ring2, long size, char *dmem_buf);

/* GLOBAL */
static struct platform_device *devices[SNDRV_CARDS] = {NULL};
static bool snd_card_enable[SNDRV_CARDS] = {1, [1 ... (SNDRV_CARDS - 1)] = 0};    // only one sound card
static int pcm_substreams[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = 3};
static int pcm_capture_substreams[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = 1};
static struct snd_card *snd_RTK_cards[SNDRV_CARDS] = SNDRV_DEFAULT_PTR;
static int snd_open_count;
static int snd_open_ai_count;
static char *snd_pcm_id[] = {SND_REALTEK_DRIVER_HDMI_IN, SND_REALTEK_DRIVER_I2S_IN, SND_REALTEK_DRIVER_NONPCM_IN,
								SND_REALTEK_DRIVER_AUDIO_IN, SND_REALTEK_DRIVER_AUDIO_V2_IN,
								SND_REALTEK_DRIVER_AUDIO_V3_IN, SND_REALTEK_DRIVER_AUDIO_V4_IN,
								SND_REALTEK_DRIVER_I2S_LOOPBACK_IN, SND_REALTEK_DRIVER_DMIC_PASSTHROUGH_IN,
								SND_REALTEK_DRIVER_PURE_DMIC_IN}; // multiple PCM instances in 1 sound card
static unsigned int rtk_dec_ao_buffer = RTK_DEC_AO_BUFFER_SIZE;
static struct refclk_device *snd_refclk;
static void __iomem *sys_clk_en2_virt;

static spinlock_t playback_lock;
static spinlock_t capture_lock;
static int dec_out_msec;
static int mtotal_latency;
static bool is_suspend;

static char gRtkDriverName[] = SND_REALTEK_DRIVER_HDMI_IN;
static struct dma_buf *enc_pts_rpc_dmabuf;

static struct snd_kcontrol_new snd_mars_controls[] = {
	MARS_VOLUME("Master Volume", 0, MIXER_ADDR_MASTER),
	MARS_CAPSRC("Master Capture Switch", 0, MIXER_ADDR_MASTER),
	MARS_VOLUME("Synth Volume", 0, MIXER_ADDR_SYNTH),
};

static struct snd_pcm_ops snd_card_rtk_playback_ops = {
	.open =         snd_card_playback_open,
	.close =        snd_card_playback_close,
	.ioctl =        snd_card_playback_ioctl,
	.hw_params =    snd_card_hw_params,
	.hw_free =      snd_card_hw_free,
	.prepare =      snd_card_playback_prepare,
	.trigger =      snd_card_playback_trigger,
	.pointer =      snd_card_playback_pointer,
	.mmap =         snd_card_playback_mmap,
};

static struct snd_pcm_ops snd_card_rtk_capture_ops = {
	.open =         snd_card_capture_open,
	.close =        snd_card_capture_close,
	.ioctl =        snd_card_capture_ioctl,
	.hw_params =    snd_card_hw_params,
	.hw_free =      snd_card_capture_hw_free,
	.prepare =      snd_card_capture_prepare,
	.trigger =      snd_card_capture_trigger,
	.pointer =      snd_card_capture_pointer,
	.get_time_info = snd_card_capture_get_time_info
};

static struct snd_pcm_hardware snd_card_playback = {
	.info               = RTK_DMP_PLAYBACK_INFO,
	.formats            = RTK_DMP_PLAYBACK_FORMATS,
	.rates              = RTK_DMP_PLYABACK_RATES,
	.rate_min           = RTK_DMP_PLAYBACK_RATE_MIN,
	.rate_max           = RTK_DMP_PLAYBACK_RATE_MAX,
	.channels_min       = RTK_DMP_PLAYBACK_CHANNELS_MIN,
	.channels_max       = RTK_DMP_PLAYBACK_CHANNELS_MAX,
	.buffer_bytes_max   = RTK_DMP_PLAYBACK_MAX_BUFFER_SIZE,
	.period_bytes_min   = RTK_DMP_PLAYBACK_MIN_PERIOD_SIZE,
	.period_bytes_max   = RTK_DMP_PLAYBACK_MAX_PERIOD_SIZE,
	.periods_min        = RTK_DMP_PLAYBACK_PERIODS_MIN,
	.periods_max        = RTK_DMP_PLAYBACK_PERIODS_MAX,
	.fifo_size          = RTK_DMP_PLAYBACK_FIFO_SIZE,
};

static struct snd_pcm_hardware snd_card_mars_capture = {
	.info               = RTK_DMP_CAPTURE_INFO,
	.formats            = RTK_DMP_CAPTURE_FORMATS,
	.rates              = RTK_DMP_CAPTURE_RATES,
	.rate_min           = RTK_DMP_CAPTURE_RATE_MIN,
	.rate_max           = RTK_DMP_CAPTURE_RATE_MAX,
	.channels_min       = RTK_DMP_CAPTURE_CHANNELS_MIN,
	.channels_max       = RTK_DMP_CAPTURE_CHANNELS_MAX,
	.buffer_bytes_max   = RTK_DMP_CAPTURE_MAX_BUFFER_SIZE,
	.period_bytes_min   = RTK_DMP_CAPTURE_MIN_PERIOD_SIZE,
	.period_bytes_max   = RTK_DMP_CAPTURE_MAX_PERIOD_SIZE,
	.periods_min        = RTK_DMP_CAPTURE_PERIODS_MIN,
	.periods_max        = RTK_DMP_CAPTURE_PERIODS_MAX,
	.fifo_size          = RTK_DMP_CAPTURE_FIFO_SIZE,
};

static struct snd_pcm_hardware snd_card_mars_capture_audio = {
	.info                   = SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats                = SNDRV_PCM_FMTBIT_S16_LE,
	.rates					= SNDRV_PCM_RATE_16000,
	.rate_min				= 16000,
	.rate_max				= 16000,
	.period_bytes_min       = 64,
	.period_bytes_max       = 4096,
	.periods_min            = 4,
	.periods_max            = 1024,
	.channels_min           = 1,
	.channels_max           = 1,
	.buffer_bytes_max       = 32*1024,
};

module_param_array(snd_card_enable, bool, NULL, 0444);
MODULE_PARM_DESC(snd_card_enable, "Enable this mars soundcard.");
module_param_array(pcm_substreams, int, NULL, 0444);
MODULE_PARM_DESC(pcm_substreams, "PCM substreams # (1-16) for mars driver.");

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

int snd_monitor_audio_data_queue(void)
{
	return mtotal_latency;
}

int snd_monitor_audio_data_queue_new(struct snd_pcm_substream *substream)
{
	int audioLatency = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;

	if (dpcm->g_ShareMemPtr2) {

		uint64_t pcmPTS;
		uint64_t curPTS;
		uint64_t diffPTS;
		unsigned int inHWRingWp;
		unsigned int sum = 0;
		unsigned int latency = 0;
		unsigned int ptsL = 0;
		unsigned int ptsH = 0;
		unsigned int in_wp = 0;
		unsigned int wp_buffer;
		unsigned int wp_frame;
		int retry = 0;

		//to get ALSA_LATENCY_INFO frmo fw
		if (dpcm->g_ShareMemPtr3) {
			memcpy(dpcm->g_ShareMemPtr3, dpcm->g_ShareMemPtr2, SHARE_MEM_SIZE_LATENCY);

			latency = htonl(dpcm->g_ShareMemPtr3->latency);
			ptsL = htonl(dpcm->g_ShareMemPtr3->ptsL);
			ptsH = htonl(dpcm->g_ShareMemPtr3->ptsH);
			sum = htonl(dpcm->g_ShareMemPtr3->sum);

			/* If the sync word is set, wp is physical address */
			if (dpcm->g_ShareMemPtr3->sync == htonl(0x23792379))
				in_wp = htonl(dpcm->g_ShareMemPtr3->decin_wp);
			else
				in_wp = htonl(dpcm->g_ShareMemPtr3->decin_wp) & 0x0fffffff;

			// make sure all of ALSA_LATENCY_INFO are updated.
			while (sum != (latency + ptsL)) {
				if (retry > 100) {
					if (ptsL < sum)
						latency = sum - ptsL;
					break;
				}
				memcpy(dpcm->g_ShareMemPtr3, dpcm->g_ShareMemPtr2, SHARE_MEM_SIZE_LATENCY);
				latency = htonl(dpcm->g_ShareMemPtr3->latency);
				ptsL = htonl(dpcm->g_ShareMemPtr3->ptsL);
				ptsH = htonl(dpcm->g_ShareMemPtr3->ptsH);
				sum = htonl(dpcm->g_ShareMemPtr3->sum);

				/* If the sync word is set, wp is physical address */
				if (dpcm->g_ShareMemPtr3->sync == htonl(0x23792379))
					in_wp = htonl(dpcm->g_ShareMemPtr3->decin_wp);
				else
					in_wp = htonl(dpcm->g_ShareMemPtr3->decin_wp) & 0x0fffffff;

				retry++;
			}
		}

		inHWRingWp = ntohl(dpcm->decInRing[0].writePtr); //physical address

		if (retry > 20)
			pr_info("latency 0x%u, ptsL 0x%u, ptsH 0x%u, sum 0x%u, in_wp 0x%x inHWRingWp 0x%x\n",
						latency, ptsL, ptsH, sum, in_wp, inHWRingWp);

		pcmPTS = (((uint64_t)ptsH<< 32) | ((uint64_t)ptsL));
		curPTS = snd_card_get_90k_pts();
		diffPTS = curPTS - pcmPTS;

		wp_frame = 0;
		if (in_wp != inHWRingWp) {
			wp_buffer = ring_valid_data(dpcm->decInRing_LE[0].beginAddr
							,dpcm->decInRing_LE[0].beginAddr + dpcm->decInRing_LE[0].size
							,in_wp
							,inHWRingWp);
			wp_frame = bytes_to_frames(runtime, wp_buffer);
		}

		//old audio fw only has latency without ptsL and wp
		if (in_wp == 0 && ptsL == 0) {
			if (dpcm->g_ShareMemPtr) {
				audioLatency = *(dpcm->g_ShareMemPtr);
				audioLatency = htonl(audioLatency);
				audioLatency += dec_out_msec;
			}
		} else {
			uint64_t queuebuffer = wp_frame + ring_valid_data(0, runtime->boundary, dpcm->nTotalWrite, runtime->control->appl_ptr);

			queuebuffer = queuebuffer * 1000000 / runtime->rate;
			audioLatency = latency + queuebuffer - div64_ul(diffPTS * 1000, 90);
			audioLatency = audioLatency / 1000;
		}

		if (audioLatency < 0)
			audioLatency = 0;

	} else if (dpcm->g_ShareMemPtr) {
		audioLatency = *(dpcm->g_ShareMemPtr);
		audioLatency = htonl(audioLatency);
		audioLatency += dec_out_msec;
	} else
		pr_err("NO exist share memory !!\n");

	mtotal_latency = audioLatency;

	return audioLatency;
}

int snd_realtek_capture_check_hdmirx_enable(void)
{
#define HDMI_RX_CLK_BIT 24
	int ret = 0;

	if (sys_clk_en2_virt) {
		ret = (int)readl(sys_clk_en2_virt);
		ret = (ret >> HDMI_RX_CLK_BIT) & 0x1;
	}

	return ret;
}

char *snd_realtek_capture_get_stream_name(void)
{
	return gRtkDriverName;
}
EXPORT_SYMBOL(snd_realtek_capture_get_stream_name);

static int snd_realtek_hw_check_audio_ready(struct snd_card_RTK_pcm *dpcm)
{
	/* for blocking read */
	wait_queue_head_t waitQueue;
	int ret = 0;

	// Initialize wait queue...
	init_waitqueue_head(&waitQueue);

	ret = wait_event_interruptible_timeout(waitQueue,
				RPC_TOAGENT_CHECK_AUDIO_READY(dpcm->phy_addr_rpc, dpcm->vaddr_rpc) == 0, HZ);

	if (ret <= 0)
		return 1;   // fail
	else
		return 0;   // success
}

static int snd_realtek_hw_capture_init_ringheader_of_AI(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	struct RINGBUFFER_HEADER *pAIRingHeader = dpcm->nAIRing;
	struct RINGBUFFER_HEADER *pAIRingHeader_LE = dpcm->nAIRing_LE;
	struct AUDIO_RPC_RINGBUFFER_HEADER nRingHeader;
	int ch;

	// init ring header
	for (ch = 0; ch < runtime->channels; ++ch) {
		pAIRingHeader_LE[ch].beginAddr = (unsigned long)dpcm->phy_pAIRingData[ch];
		pAIRingHeader_LE[ch].size = dpcm->nRingSize;
		pAIRingHeader_LE[ch].readPtr[0] = pAIRingHeader_LE[ch].beginAddr;
		pAIRingHeader_LE[ch].writePtr = pAIRingHeader_LE[ch].beginAddr;
		pAIRingHeader_LE[ch].numOfReadPtr = 1;

		pAIRingHeader[ch].beginAddr = htonl((unsigned long)pAIRingHeader_LE[ch].beginAddr);
		pAIRingHeader[ch].size = htonl(pAIRingHeader_LE[ch].size);
		pAIRingHeader[ch].readPtr[0] = htonl(pAIRingHeader_LE[ch].readPtr[0]);
		pAIRingHeader[ch].writePtr = htonl(pAIRingHeader_LE[ch].writePtr);
		pAIRingHeader[ch].numOfReadPtr = htonl(pAIRingHeader_LE[ch].numOfReadPtr);

		nRingHeader.pRingBufferHeaderList[ch] = (unsigned long) (pAIRingHeader + ch) - (unsigned long)dpcm + dpcm->phy_addr;

	}

	// init AI ring header
	nRingHeader.instanceID = dpcm->AIAgentID;
	nRingHeader.pinID = PCM_OUT;
	nRingHeader.readIdx = -1;
	nRingHeader.listSize =  runtime->channels;

	// RPC set AI ring header
	if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				&nRingHeader, nRingHeader.listSize) < 0) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	}

	return 0;
}

static int snd_realtek_hw_capture_init_LPCM_ringheader_of_AI(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	struct RINGBUFFER_HEADER *pAIRingHeader = &dpcm->nLPCMRing;
	struct RINGBUFFER_HEADER *pAIRingHeader_LE = &dpcm->nLPCMRing_LE;
	struct AUDIO_RPC_RINGBUFFER_HEADER nRingHeader;

	// init ring header
	pAIRingHeader_LE->beginAddr = (unsigned int)dpcm->phy_pLPCMData;
	pAIRingHeader_LE->size = dpcm->nLPCMRingSize;
	pAIRingHeader_LE->readPtr[0] = pAIRingHeader_LE->beginAddr;
	pAIRingHeader_LE->writePtr = pAIRingHeader_LE->beginAddr;
	pAIRingHeader_LE->numOfReadPtr = 1;

	pAIRingHeader->beginAddr = htonl((unsigned int)pAIRingHeader_LE->beginAddr);
	pAIRingHeader->size = htonl(pAIRingHeader_LE->size);
	pAIRingHeader->readPtr[0] = htonl(pAIRingHeader_LE->readPtr[0]);
	pAIRingHeader->writePtr = htonl(pAIRingHeader_LE->writePtr);
	pAIRingHeader->numOfReadPtr = htonl(pAIRingHeader_LE->numOfReadPtr);

	nRingHeader.pRingBufferHeaderList[0] = (int)((unsigned long) pAIRingHeader - (unsigned long)dpcm + dpcm->phy_addr);

	// init AI ring header
	nRingHeader.instanceID = dpcm->AIAgentID;
	nRingHeader.readIdx = -1;
	nRingHeader.listSize = 1;

	if (dpcm->source_in == ENUM_AIN_AUDIO)
		nRingHeader.pinID = APP_LPCM_OUT;
	else
		nRingHeader.pinID = BASE_BS_OUT;

	// RPC set AI ring header
	if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(dpcm->phy_addr_rpc,
				dpcm->vaddr_rpc, &nRingHeader, nRingHeader.listSize) < 0) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	};

	return 0;
}

static int snd_realtek_hw_capture_init_PTS_ringheader_of_AI(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	struct RINGBUFFER_HEADER *pAIRingHeader = &dpcm->nPTSRingHdr;
	struct RINGBUFFER_HEADER *pAIRingHeader_LE, nAIRingHeader_LE;
	struct AUDIO_RPC_RINGBUFFER_HEADER nRingHeader;

	pAIRingHeader_LE = &nAIRingHeader_LE;

	dpcm->nPTSRing.base = (unsigned long)dpcm->nPTSMem.pVirt;
	dpcm->nPTSRing.wp = dpcm->nPTSRing.rp = dpcm->nPTSRing.base;
	dpcm->nPTSRing.limit = dpcm->nPTSRing.base + dpcm->nPTSMem.size;

	// init ring header
	pAIRingHeader_LE->beginAddr = (unsigned int)dpcm->nPTSMem.pPhy;
	pAIRingHeader_LE->size = dpcm->nPTSMem.size;
	pAIRingHeader_LE->readPtr[0] = pAIRingHeader_LE->beginAddr;
	pAIRingHeader_LE->writePtr = pAIRingHeader_LE->beginAddr;
	pAIRingHeader_LE->numOfReadPtr = 1;

	pAIRingHeader->beginAddr = htonl((unsigned int)pAIRingHeader_LE->beginAddr);
	pAIRingHeader->size = htonl(pAIRingHeader_LE->size);
	pAIRingHeader->readPtr[0] = htonl(pAIRingHeader_LE->readPtr[0]);
	pAIRingHeader->writePtr = htonl(pAIRingHeader_LE->writePtr);
	pAIRingHeader->numOfReadPtr = htonl(pAIRingHeader_LE->numOfReadPtr);

	nRingHeader.pRingBufferHeaderList[0] = (int)((unsigned long) pAIRingHeader - (unsigned long)dpcm + dpcm->phy_addr);

	// init AI ring header
	nRingHeader.instanceID = dpcm->AIAgentID;
	nRingHeader.pinID = MESSAGE_QUEUE;
	nRingHeader.readIdx = -1;
	nRingHeader.listSize = 1;

	// RPC set AI ring header
	if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				&nRingHeader, nRingHeader.listSize) < 0) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	};

	return 0;
}

// init ringheader of decoder_outring and AO_inring
static int snd_realtek_hw_init_ringheader_of_DEC_AO(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;
	struct RINGBUFFER_HEADER *pAORingHeader = dpcm->decOutRing;
	struct AUDIO_RPC_RINGBUFFER_HEADER nRingHeader;
	int ch, j;

	// init ring header
	for (ch = 0 ; ch < runtime->channels ; ++ch) {
		pAORingHeader[ch].beginAddr = (unsigned int)htonl(dpcm->phy_decOutData[ch]);
		pAORingHeader[ch].size = htonl(dpcm->nRingSize);
		for (j = 0 ; j < 4 ; ++j)
			pAORingHeader[ch].readPtr[j] = pAORingHeader[ch].beginAddr;
		pAORingHeader[ch].writePtr = pAORingHeader[ch].beginAddr;
		pAORingHeader[ch].numOfReadPtr = htonl(1);
	}

	// init DEC outring header
	nRingHeader.instanceID = dpcm->DECAgentID;
	nRingHeader.pinID = PCM_OUT;
	for (ch = 0 ; ch < runtime->channels ; ++ch)
		nRingHeader.pRingBufferHeaderList[ch] = (unsigned long)&dpcm->decOutRing[ch] - (unsigned long)dpcm + dpcm->phy_addr;
	nRingHeader.readIdx = -1;
	nRingHeader.listSize = runtime->channels;

	// RPC set DEC outring header
	if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				&nRingHeader, nRingHeader.listSize) < 0) {
		pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
		return -1;
	};

	// init AO inring header
	nRingHeader.instanceID = dpcm->AOAgentID;
	nRingHeader.pinID = dpcm->AOpinID;
	for (ch = 0 ; ch < runtime->channels ; ++ch)
		nRingHeader.pRingBufferHeaderList[ch] = (unsigned long)&dpcm->decOutRing[ch] - (unsigned long)dpcm + dpcm->phy_addr;
	nRingHeader.readIdx = 0;
	nRingHeader.listSize = runtime->channels;

	// RPC set AO inring header
	if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				&nRingHeader, nRingHeader.listSize) < 0) {
		pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
		return -1;
	};

	return 0;
}

// init ringheader of AO_inring without decoder
static int snd_realtek_hw_init_ringheader_of_AO(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;
	struct AUDIO_RPC_RINGBUFFER_HEADER nRingHeader;

	// init ring header using original decoder inRing instead of using decoder out ring
	dpcm->decInRing[0].beginAddr = htonl((unsigned int)runtime->dma_addr);
	dpcm->decInRing[0].bufferID = htonl(RINGBUFFER_STREAM);
	dpcm->decInRing[0].size = htonl(frames_to_bytes(runtime, runtime->buffer_size));
	dpcm->decInRing[0].writePtr = dpcm->decInRing[0].beginAddr;
	dpcm->decInRing[0].readPtr[0] = dpcm->decInRing[0].beginAddr;
	dpcm->decInRing[0].readPtr[1] = dpcm->decInRing[0].beginAddr;
	dpcm->decInRing[0].readPtr[2] = dpcm->decInRing[0].beginAddr;
	dpcm->decInRing[0].readPtr[3] = dpcm->decInRing[0].beginAddr;
	dpcm->decInRing[0].numOfReadPtr = htonl(1);

	// init ring header using original decoder inRing (little endian) instead of using decoder out ring
	dpcm->decInRing_LE[0].beginAddr = ((unsigned int)runtime->dma_addr);
	dpcm->decInRing_LE[0].size = frames_to_bytes(runtime, runtime->buffer_size);
	dpcm->decInRing_LE[0].writePtr = dpcm->decInRing_LE[0].beginAddr;
	dpcm->decInRing_LE[0].readPtr[0] = dpcm->decInRing_LE[0].beginAddr;
	dpcm->decInRing_LE[0].readPtr[1] = dpcm->decInRing_LE[0].beginAddr;
	dpcm->decInRing_LE[0].readPtr[2] = dpcm->decInRing_LE[0].beginAddr;
	dpcm->decInRing_LE[0].readPtr[3] = dpcm->decInRing_LE[0].beginAddr;

	// init AO inring header
	nRingHeader.instanceID = dpcm->AOAgentID;
	nRingHeader.pinID = dpcm->AOpinID;
	nRingHeader.pRingBufferHeaderList[0] = (unsigned long)&dpcm->decInRing - (unsigned long)dpcm + dpcm->phy_addr;
	nRingHeader.readIdx = 0;
	nRingHeader.listSize = 1;

	// RPC set AO inring header
	if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				&nRingHeader, nRingHeader.listSize) < 0) {
		pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
		return -1;
	};

	return 0;
}

// init ringheader of AO_using ai ring
static int snd_realtek_init_AO_ringheader_by_AI(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	struct RINGBUFFER_HEADER *pAIRingHeader = dpcm->nAIRing;
	struct RINGBUFFER_HEADER *pAIRingHeader_LE = dpcm->nAIRing_LE;
	struct AUDIO_RPC_RINGBUFFER_HEADER nRingHeader;
	int ch;

	// init ring header
	for (ch = 0; ch < runtime->channels; ++ch) {
		pAIRingHeader_LE[ch].beginAddr = (unsigned long)dpcm->phy_pAIRingData[ch];
		pAIRingHeader_LE[ch].size = dpcm->nRingSize;
		pAIRingHeader_LE[ch].readPtr[0] = pAIRingHeader_LE[ch].beginAddr;
		pAIRingHeader_LE[ch].writePtr = pAIRingHeader_LE[ch].beginAddr;
		pAIRingHeader_LE[ch].numOfReadPtr = 1;

		pAIRingHeader[ch].beginAddr = htonl((unsigned long)pAIRingHeader_LE[ch].beginAddr);
		pAIRingHeader[ch].size = htonl(pAIRingHeader_LE[ch].size);
		pAIRingHeader[ch].readPtr[0] = htonl(pAIRingHeader_LE[ch].readPtr[0]);
		pAIRingHeader[ch].writePtr = htonl(pAIRingHeader_LE[ch].writePtr);
		pAIRingHeader[ch].numOfReadPtr = htonl(pAIRingHeader_LE[ch].numOfReadPtr);

		nRingHeader.pRingBufferHeaderList[ch] =
					(unsigned long) (pAIRingHeader + ch) - (unsigned long)dpcm + dpcm->phy_addr;
	}

	// init AO ring header
	nRingHeader.instanceID = dpcm->AOAgentID;
	nRingHeader.pinID = dpcm->AOpinID;
	nRingHeader.readIdx = 0;
	nRingHeader.listSize = runtime->channels;

	// RPC set AO ring header
	if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				&nRingHeader, nRingHeader.listSize) < 0) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	}

	return 0;
}

// connect source to destination
static int snd_realtek_hw_init_connect(phys_addr_t paddr, void *vaddr,
			int srcInstanceID, int desInstanceID, int desPinID)
{
	struct AUDIO_RPC_CONNECTION nConnection;

	nConnection.desInstanceID = desInstanceID;
	nConnection.srcInstanceID = srcInstanceID;
	nConnection.srcPinID = PCM_OUT;
	nConnection.desPinID = desPinID;

	if (RPC_TOAGENT_CONNECT_SVC(paddr, vaddr, &nConnection)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	};

	return 0;
}

// 1. init decoder in_ring
// 2. init decoder inband ring
static int snd_realtek_hw_init_decoder_inring(struct snd_pcm_runtime *runtime)
{
    struct snd_card_RTK_pcm *dpcm = runtime->private_data;
    struct AUDIO_RPC_RINGBUFFER_HEADER ringbuf_header;

    // init HW inring header
    dpcm->decInRing[0].beginAddr = htonl((unsigned int)runtime->dma_addr);
    dpcm->decInRing[0].bufferID = htonl(RINGBUFFER_STREAM);
    dpcm->decInRing[0].size = htonl(frames_to_bytes(runtime, runtime->buffer_size));
    dpcm->decInRing[0].writePtr = dpcm->decInRing[0].beginAddr;
    dpcm->decInRing[0].readPtr[0] = dpcm->decInRing[0].beginAddr;
    dpcm->decInRing[0].readPtr[1] = dpcm->decInRing[0].beginAddr;
    dpcm->decInRing[0].readPtr[2] = dpcm->decInRing[0].beginAddr;
    dpcm->decInRing[0].readPtr[3] = dpcm->decInRing[0].beginAddr;
    dpcm->decInRing[0].numOfReadPtr = htonl(1);

    // init HW inring (little endian)
    dpcm->decInRing_LE[0].beginAddr = ((unsigned int)runtime->dma_addr);
    dpcm->decInRing_LE[0].size = frames_to_bytes(runtime, runtime->buffer_size);
    dpcm->decInRing_LE[0].writePtr = dpcm->decInRing_LE[0].beginAddr;
    dpcm->decInRing_LE[0].readPtr[0] = dpcm->decInRing_LE[0].beginAddr;
    dpcm->decInRing_LE[0].readPtr[1] = dpcm->decInRing_LE[0].beginAddr;
    dpcm->decInRing_LE[0].readPtr[2] = dpcm->decInRing_LE[0].beginAddr;
    dpcm->decInRing_LE[0].readPtr[3] = dpcm->decInRing_LE[0].beginAddr;

    // init RPC ring header
    ringbuf_header.instanceID = dpcm->DECAgentID;
    ringbuf_header.pinID = BASE_BS_IN;
    ringbuf_header.pRingBufferHeaderList[0] = (unsigned long)dpcm->decInRing - (unsigned long)dpcm + dpcm->phy_addr;
    ringbuf_header.readIdx = 0;
    ringbuf_header.listSize = 1;

    // RPC set decoder in_ring
    if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				&ringbuf_header, ringbuf_header.listSize)) {
        pr_err("[fail %s %d]\n", __FUNCTION__, __LINE__);
        return -1;
    };

    // init inband ring header
    dpcm->decInbandRing.beginAddr = htonl((int)((unsigned long)dpcm->decInbandData - (unsigned long)dpcm + dpcm->phy_addr));
    dpcm->decInbandRing.size = htonl(sizeof(dpcm->decInbandData));
    dpcm->decInbandRing.readPtr[0] = dpcm->decInbandRing.beginAddr;
    dpcm->decInbandRing.writePtr = dpcm->decInbandRing.beginAddr;
    dpcm->decInbandRing.numOfReadPtr = htonl(1);

    // init RPC ring header
    ringbuf_header.instanceID = dpcm->DECAgentID;
    ringbuf_header.pinID = INBAND_QUEUE;
    ringbuf_header.pRingBufferHeaderList[0] = (unsigned long)&dpcm->decInbandRing - (unsigned long)dpcm + dpcm->phy_addr; //physical
    ringbuf_header.readIdx = 0;
    ringbuf_header.listSize = 1;

    if (RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				&ringbuf_header, ringbuf_header.listSize)) {
        pr_err("[fail %s %d]\n", __FUNCTION__, __LINE__);
        return -1;
    };
    return 0;
}

static int snd_realtek_hw_capture_run(struct snd_card_RTK_capture_pcm *dpcm)
{
	// AO pause
	if (dpcm->source_in == ENUM_AIN_DMIC_PASSTHROUGH) {
		if (RPC_TOAGENT_PAUSE_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
					dpcm->AOAgentID | dpcm->AOpinID)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			return -1;
		}
	}

	// AI pause
	if (RPC_TOAGENT_PAUSE_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				dpcm->AIAgentID)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	}

	// AO run
	if (dpcm->source_in == ENUM_AIN_DMIC_PASSTHROUGH) {
		if (RPC_TOAGENT_RUN_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
					dpcm->AOAgentID | dpcm->AOpinID)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			return -1;
		}
	}

	// AI run
	if (RPC_TOAGENT_RUN_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				dpcm->AIAgentID)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	}

	return 0;
}

static int snd_realtek_hw_resume(struct snd_card_RTK_pcm *dpcm)
{
    printk("[%s %s %d]\n", __FILE__, __FUNCTION__, __LINE__);

	if (dpcm->ao_decode_lpcm) {
		// decoder run
		if (RPC_TOAGENT_RUN_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
					dpcm->DECAgentID)) {
			pr_err("[%s %d]\n", __FUNCTION__, __LINE__);
			return -1;
		}
	}

	// AO run
    if (RPC_TOAGENT_RUN_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				dpcm->AOAgentID | dpcm->AOpinID)) {
        pr_err("[%s %d]\n", __FUNCTION__, __LINE__);
        return -1;
    }

    return 0;
}

int writeInbandCmd(struct snd_card_RTK_pcm *dpcm, void *data, int len)
{
	unsigned long base, limit, wp;

	base = (unsigned long)dpcm->decInbandData;
	limit = base + sizeof(dpcm->decInbandData);
	wp = base + (unsigned long)(ntohl(dpcm->decInbandRing.writePtr) - ntohl(dpcm->decInbandRing.beginAddr));

	wp = buf_memcpy2_ring(base, limit, wp, (char *)data, (unsigned long)len);
	dpcm->decInbandRing.writePtr = ntohl((int)(wp - base) + ntohl(dpcm->decInbandRing.beginAddr));

	return len;
}
EXPORT_SYMBOL(writeInbandCmd);

// ring is big-endian
int snd_realtek_hw_ring_write(struct RINGBUFFER_HEADER *ring, void *data, int len, unsigned int offset)
{
	unsigned long base, limit, wp;

	//Convert to virtual address
	base = (unsigned long)(ntohl(ring->beginAddr)) + offset;
	limit = base + ntohl(ring->size);
	wp = (unsigned long)(ntohl(ring->writePtr)) + offset;
	wp = buf_memcpy2_ring((long)base, (long)limit, (long)wp, (char *)data, (long)len);
	ring->writePtr = htonl(wp - offset); // record physical address

	return len;
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

// decoder pause
// decoder run
// decoder flush
// write decoder info into inband of decoder
static int snd_realtek_hw_init_decoder_info(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;
	struct AUDIO_DEC_NEW_FORMAT cmd;
	struct AUDIO_RPC_SENDIO sendio;

	pr_info("[%s %s %d]\n", __FILE__, __FUNCTION__, __LINE__);

	// decoder pause
	if (RPC_TOAGENT_STOP_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				dpcm->DECAgentID)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	}

	// decoder flush
	sendio.instanceID = dpcm->DECAgentID;
	sendio.pinID = dpcm->DECpinID;
	if (RPC_TOAGENT_FLUSH_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				&sendio)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	}

	// decoder run
	if (RPC_TOAGENT_RUN_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				dpcm->DECAgentID)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	}

	cmd.audioType = htonl(AUDIO_LPCM_DECODER_TYPE);
	cmd.header.type = htonl(AUDIO_DEC_INBAMD_CMD_TYPE_NEW_FORMAT);
	cmd.header.size = htonl(sizeof(struct AUDIO_DEC_NEW_FORMAT));
	cmd.privateInfo[0] = htonl(runtime->channels);
	cmd.privateInfo[1] = htonl(runtime->sample_bits);
	cmd.privateInfo[2] = htonl(runtime->rate);
	cmd.privateInfo[3] = htonl(0);
	cmd.privateInfo[4] = htonl(snd_choose_channel_mapping(runtime->channels));  // privateInfo[4]&0x3FFFC0)>>6; // bit[6:21] for wave channel mask
	cmd.privateInfo[5] = htonl(0);
	cmd.privateInfo[6] = htonl(0);  // (a_format->privateData[0]&0x1)<<4 : float or not
	cmd.privateInfo[7] = htonl(0);

	switch (runtime->format) {
	case SNDRV_PCM_FORMAT_S16_BE:
	case SNDRV_PCM_FORMAT_U16_BE:
	case SNDRV_PCM_FORMAT_S24_BE:
	case SNDRV_PCM_FORMAT_U24_BE:
	case SNDRV_PCM_FORMAT_S32_BE:
	case SNDRV_PCM_FORMAT_U32_BE:
	case SNDRV_PCM_FORMAT_S24_3BE:
	case SNDRV_PCM_FORMAT_U24_3BE:
	case SNDRV_PCM_FORMAT_S20_3BE:
	case SNDRV_PCM_FORMAT_U20_3BE:
	case SNDRV_PCM_FORMAT_S18_3BE:
	case SNDRV_PCM_FORMAT_U18_3BE:
	case SNDRV_PCM_FORMAT_FLOAT_BE:
	case SNDRV_PCM_FORMAT_FLOAT64_BE:
	case SNDRV_PCM_FORMAT_IEC958_SUBFRAME_BE:
		cmd.privateInfo[7]  = htonl(AUDIO_BIG_ENDIAN);
		break;
	case SNDRV_PCM_FORMAT_S8:
	case SNDRV_PCM_FORMAT_U8:
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_U16_LE:
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_U24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_U32_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_U24_3LE:
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_U20_3LE:
	case SNDRV_PCM_FORMAT_S18_3LE:
	case SNDRV_PCM_FORMAT_U18_3LE:
	case SNDRV_PCM_FORMAT_FLOAT_LE:
	case SNDRV_PCM_FORMAT_FLOAT64_LE:
	case SNDRV_PCM_FORMAT_IEC958_SUBFRAME_LE:
	case SNDRV_PCM_FORMAT_MU_LAW:
	case SNDRV_PCM_FORMAT_A_LAW:
	case SNDRV_PCM_FORMAT_IMA_ADPCM:
	case SNDRV_PCM_FORMAT_MPEG:
	case SNDRV_PCM_FORMAT_GSM:
	case SNDRV_PCM_FORMAT_SPECIAL:
	default:
		cmd.privateInfo[7]  = htonl(AUDIO_LITTLE_ENDIAN);
		break;
	}

	cmd.wPtr = dpcm->decInRing[0].writePtr;

#ifdef CONFIG_64BIT
	writeInbandCmd(dpcm, &cmd, sizeof(struct AUDIO_DEC_NEW_FORMAT));
#else
	snd_realtek_hw_ring_write(&dpcm->decInbandRing,
				&cmd, sizeof(struct AUDIO_DEC_NEW_FORMAT),
				(unsigned long)dpcm - (unsigned long)dpcm->phy_addr);
#endif

	return 0;
}

// decoder stop
// AO pause
// AO stop
// decoder flush
// destroy decoder
static int snd_realtek_reprepare(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;
	struct AUDIO_RPC_SENDIO sendio;

	RPC_TOAGENT_PUT_SHARE_MEMORY_LATENCY(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				NULL, NULL,	0, dpcm->AOpinID,
				ENUM_PRIVATEINFO_AUDIO_GET_SHARE_MEMORY_FROM_ALSA);

	// decoder stop
	if (dpcm->ao_decode_lpcm) {
		// decoder stop
		if (RPC_TOAGENT_STOP_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
					dpcm->DECAgentID)) {
			pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
			return -1;
		}
	}

	// AO pause
	if (RPC_TOAGENT_PAUSE_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				dpcm->AOAgentID | dpcm->AOpinID)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	}

	// AO stop
	if (RPC_TOAGENT_STOP_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				dpcm->AOAgentID | dpcm->AOpinID)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -1;
	}

	if (dpcm->ao_decode_lpcm) {
		// decoder flush
		sendio.instanceID = dpcm->DECAgentID;
		sendio.pinID = dpcm->DECpinID;
		if (RPC_TOAGENT_FLUSH_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
					&sendio)) {
			pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
			return -1;
		}

		// destroy decoder
		if (RPC_TOAGENT_DESTROY_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
					dpcm->DECAgentID)) {
			pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
			return -1;
		}
	}

	// free decoder InBand ring
	memset(dpcm->decInbandData, 0, sizeof(unsigned int)*64);

	// free ao in_ring
	// decoder out_ring = ao in_ring
	snd_realtek_hw_free_ring(runtime);

	return 0;
}

// malloc ring buffer for AI.
static int snd_realtek_hw_capture_malloc_ring(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	int ch, bMallocSuccess = 1;
	phys_addr_t dat;

	for (ch = 0; ch < runtime->channels ; ++ch) {
		if (dpcm->pAIRingData[ch]) {
			pr_err("[re-malloc error !!! %s %d]\n", __func__, __LINE__);
			if (!IS_ERR(dpcm->enc_in_rpc_dmabuf[ch])) {
				rtk_rpc_free_ion(dpcm->enc_in_rpc_dmabuf[ch]);
				dpcm->enc_in_rpc_dmabuf[ch] = NULL;
			}
			dpcm->pAIRingData[ch] = NULL;
		}

		dpcm->enc_in_rpc_dmabuf[ch] = ion_alloc(RTK_ENC_AI_BUFFER_SIZE, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);

		dat = rtk_rpc_ion_pa(dpcm->enc_in_rpc_dmabuf[ch]->priv);
		if (dat == -1) {
			snd_realtek_hw_capture_free_ring(runtime);
			pr_err("[malloc ch %d fail and terminate %s %d]\n", ch, __func__, __LINE__);
			return bMallocSuccess;
		}

		dpcm->phy_pAIRingData[ch] = dat;
		dpcm->pAIRingData[ch] = rtk_rpc_ion_va(dpcm->enc_in_rpc_dmabuf[ch]->priv);
		dpcm->nRingSize = RTK_ENC_AI_BUFFER_SIZE;
	}

	bMallocSuccess = 0; //Success
	return bMallocSuccess;
}

static int snd_realtek_hw_capture_malloc_lpcm_ring(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	int bMallocSuccess = 1;
	phys_addr_t dat;

	dpcm->enc_lpcm_rpc_dmabuf = ion_alloc(RTK_ENC_LPCM_BUFFER_SIZE, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(dpcm->enc_lpcm_rpc_dmabuf)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		bMallocSuccess = PTR_ERR(dpcm->enc_lpcm_rpc_dmabuf);
		return bMallocSuccess;
	}

	dat = rtk_rpc_ion_pa(dpcm->enc_lpcm_rpc_dmabuf->priv);
	if (dat == -1) {
		pr_info("[malloc fail and terminate %s %d]\n", __func__, __LINE__);
		return bMallocSuccess;
	}

	dpcm->phy_pLPCMData = dat;
	dpcm->pLPCMData = rtk_rpc_ion_va(dpcm->enc_lpcm_rpc_dmabuf->priv);
	dpcm->nLPCMRingSize = RTK_ENC_LPCM_BUFFER_SIZE;

	bMallocSuccess = 0; //Success

	return bMallocSuccess;
}

static int snd_realtek_hw_capture_malloc_pts_ring(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	int bMallocSuccess = -1;
	phys_addr_t dat;

	dpcm->nPTSMem.size = RTK_ENC_PTS_BUFFER_SIZE;

	enc_pts_rpc_dmabuf = ion_alloc(RTK_ENC_PTS_BUFFER_SIZE, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(enc_pts_rpc_dmabuf)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		bMallocSuccess = PTR_ERR(enc_pts_rpc_dmabuf);
		return bMallocSuccess;
	}

	dat = rtk_rpc_ion_pa(enc_pts_rpc_dmabuf->priv);
	if (dat == -1) {
		pr_info("[malloc fail and terminate %s %d]\n", __func__, __LINE__);
		return bMallocSuccess;
	}

	dpcm->nPTSMem.pPhy = dat;
	dpcm->nPTSMem.pVirt = rtk_rpc_ion_va(enc_pts_rpc_dmabuf->priv);

	bMallocSuccess = 0; //Success

	return bMallocSuccess;
}


// in decoder-AO path, malloc AO in_ring (decoder out)
static int snd_realtek_hw_malloc_ring(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;
	int ch, bMallocSuccess = 1;
	phys_addr_t dat;

	for (ch = 0 ; ch < runtime->channels ; ++ch) {
		dpcm->dec_out_rpc_dmabuf[ch] = ion_alloc(rtk_dec_ao_buffer, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
		dat = rtk_rpc_ion_pa(dpcm->dec_out_rpc_dmabuf[ch]->priv);
		if (dat == -1) {
			snd_realtek_hw_free_ring(runtime);
			printk("[malloc ch %d fail and terminate %s %d]\n", ch, __FUNCTION__, __LINE__);
			return bMallocSuccess;
		}

		dpcm->phy_decOutData[ch] = dat;
		dpcm->decOutData[ch] = rtk_rpc_ion_va(dpcm->dec_out_rpc_dmabuf[ch]->priv);

		dpcm->nRingSize = rtk_dec_ao_buffer;
	}

	bMallocSuccess = 0; //Success

	return bMallocSuccess;
}

static int snd_realtek_hw_create_AI(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;

	if (snd_open_ai_count >= MAX_AI_DEVICES) {
		pr_err("[too more AI %s %d]\n", __func__, __LINE__);
		return -1;
	}

	if (dpcm->source_in == ENUM_AIN_AUDIO) {
		if (RPC_TOAGENT_GET_AI_AGENT(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
					dpcm)) {
			pr_err("[err %s %d]\n", __func__, __LINE__);
			return -1;
		}
	} else {
		if (RPC_TOAGENT_CREATE_AI_AGENT(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
					dpcm)) {
			pr_err("[err %s %d]\n", __func__, __LINE__);
			return -1;
		}
	}

	snd_open_ai_count++;

	return 0;
}

// get AO flash pin ID
static int snd_realtek_hw_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;

	if (snd_open_count >= MAX_PCM_SUBSTREAMS) {
		pr_err("[opened audio stream count excess %d]\n", MAX_PCM_SUBSTREAMS);
		return -1;
	}

	// get ao flash pin ID
	dpcm->AOpinID = RPC_TOAGENT_GET_AO_FLASH_PIN(dpcm->phy_addr_rpc,
				dpcm->vaddr_rpc, dpcm->AOAgentID);
	if (dpcm->AOpinID < 0) {
		pr_err("[can't get flash pin %s %d]\n", __func__, __LINE__);
		return -1;
	}

	// init volume
	dpcm->volume = 31;

	snd_open_count++;

	return 0;
}

// check if AO exists
// return 0(success), 1(fail)
static int snd_realtek_hw_init(struct snd_card_RTK_pcm *dpcm)
{
	if (dpcm->bHWinit != 0)
		return 0;

	if (snd_realtek_hw_check_audio_ready(dpcm))
		return -1;

	if (RPC_TOAGENT_CREATE_AO_AGENT(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				&dpcm->AOAgentID, AUDIO_ALSA_OUT)) {
		pr_err("[No AO agent %s %d]\n", __func__, __LINE__);
		return -1;
	}

	dpcm->bHWinit = 1;

	return 0;
}

static int snd_realtek_hw_capture_free_ring(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	int ch;

	pr_info("[ALSA %s %d]\n", __func__, __LINE__);

	switch (dpcm->source_in) {
	case ENUM_AIN_AUDIO_V2:
	case ENUM_AIN_AUDIO_V3:
	case ENUM_AIN_AUDIO_V4:
		break;
	default:
		// free AI in_ring
		for (ch = 0; ch < runtime->channels; ch++) {
			if (dpcm->pAIRingData[ch] != NULL) {
				if (!IS_ERR(dpcm->enc_in_rpc_dmabuf[ch])) {
					rtk_rpc_free_ion(dpcm->enc_in_rpc_dmabuf[ch]);
					dpcm->enc_in_rpc_dmabuf[ch] = NULL;
				}
				dpcm->pAIRingData[ch] = NULL;
			}
		}
		break;
	}

	// free AI lpcm ring
	if (dpcm->nAIFormat == AUDIO_ALSA_FORMAT_16BITS_LE_LPCM ||
	    dpcm->nAIFormat == AUDIO_ALSA_FORMAT_24BITS_LE_LPCM) {
		if (!IS_ERR(dpcm->enc_lpcm_rpc_dmabuf)) {
			rtk_rpc_free_ion(dpcm->enc_lpcm_rpc_dmabuf);
			dpcm->enc_lpcm_rpc_dmabuf = NULL;
		}
	}

#ifdef CAPTURE_USE_PTS_RING
	// free AI pts ring
	if (!IS_ERR(enc_pts_rpc_dmabuf)) {
		rtk_rpc_free_ion(enc_pts_rpc_dmabuf);
		enc_pts_rpc_dmabuf = NULL;
	}
#endif

	return 0;
}

static int snd_realtek_hw_free_ring(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;
	int ch;

	pr_info("[ALSA %s %d]\n", __FUNCTION__, __LINE__);

	for (ch = 0; ch < dpcm->last_channel; ch++) {
		if(dpcm->decOutData[ch] != NULL) {
			if(!IS_ERR(dpcm->dec_out_rpc_dmabuf[ch])) {
				rtk_rpc_free_ion(dpcm->dec_out_rpc_dmabuf[ch]);
				dpcm->dec_out_rpc_dmabuf[ch] = NULL;
			}
			dpcm->decOutData[ch] = NULL;
		}
	}

	return 0;
}

static void snd_card_runtime_free(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;
	struct dma_buf *rpc_dmabuf_free;

	if (dpcm == NULL)
		goto out;

	rpc_dmabuf_free = dpcm->rpc_dmabuf;

	if (!IS_ERR(rpc_dmabuf_free)) {
		pr_info("%s free playback handle dma buf\n",
					__func__);
		rtk_rpc_free_ion(rpc_dmabuf_free);
		rpc_dmabuf_free = NULL;
	}

out:
	runtime->private_data = NULL;
}

static void snd_card_capture_runtime_free(struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	struct dma_buf *rpc_dmabuf_free;

	if (dpcm == NULL)
		goto out;

	rpc_dmabuf_free = dpcm->rpc_dmabuf;

	if (!IS_ERR(rpc_dmabuf_free)) {
		pr_info("%s free capture handle dma buf\n",
					__func__);
		rtk_rpc_free_ion(rpc_dmabuf_free);
		rpc_dmabuf_free = NULL;
	}

out:
	runtime->private_data = NULL;
}

static int snd_card_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_pcm *dpcm = NULL;
	int ret = 0;
	phys_addr_t dat;
	struct dma_buf *rpc_dmabuf = NULL;

	rpc_dmabuf = ion_alloc(sizeof(struct snd_card_RTK_pcm),
				RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);

	if (IS_ERR(rpc_dmabuf)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		ret = PTR_ERR(rpc_dmabuf);
		goto fail;
	}

	dat = rtk_rpc_ion_pa(rpc_dmabuf->priv);
	if (dat == -1) {
		pr_info("[%s %d] alloc memory faild\n", __func__, __LINE__);
		goto fail;
	}

	dpcm = rtk_rpc_ion_va(rpc_dmabuf->priv);
	memset(dpcm, 0, sizeof(struct snd_card_RTK_pcm));
	dpcm->phy_addr = dat;
	dpcm->rpc_dmabuf = rpc_dmabuf;

	// private data
	runtime->private_data = dpcm;
	runtime->private_free = snd_card_runtime_free;

	/* Preparing the ion buffer for closing playback */
	dpcm->rpc_dmabuf_rpc = ion_alloc(4096, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(dpcm->rpc_dmabuf_rpc)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		ret = PTR_ERR(dpcm->rpc_dmabuf_rpc);
		goto fail;
	}

	dpcm->phy_addr_rpc = rtk_rpc_ion_pa(dpcm->rpc_dmabuf_rpc->priv);
	if (dpcm->phy_addr_rpc == -1) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		goto fail;
	}
	dpcm->vaddr_rpc = rtk_rpc_ion_va(dpcm->rpc_dmabuf_rpc->priv);

	// check if AO exists
	if (snd_realtek_hw_init(dpcm)) {
		pr_err("[error %s %d]\n", __func__, __LINE__);
		ret = -ENOMEDIUM;
		goto fail;
	}

	memcpy(&runtime->hw, &snd_card_playback, sizeof(struct snd_pcm_hardware));
	dpcm->substream = substream;
	dpcm->nEOSState = SND_REALTEK_EOS_STATE_NONE;
	if (snd_realtek_hw_open(substream) < 0) {
		pr_err("[error %s %d]\n", __func__, __LINE__);
		ret = -ENOMEM;
		goto fail;
	}

	dpcm->card = (struct RTK_snd_card *) (substream->pcm->card->private_data);

	// init dec_out_msec
	dec_out_msec = 0;

	// init hr timer
	hrtimer_init(&dpcm->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dpcm->hr_timer.function = &snd_card_timer_function;

	pr_info("ALSA: Device number is %d\n", substream->pcm->device);
	switch (substream->pcm->device) {
	case 0:
		// AO decode lpcm
		dpcm->ao_decode_lpcm = 1;
		pr_info("ALSA: AO with decoder flow.\n");
		break;
	case 1:
		// AO not decode lpcm
		dpcm->ao_decode_lpcm = 0;
		// Init the ao parameter
		dpcm->ao_paramter = NULL;
		pr_info("ALSA: AO skip decoder flow.\n");
		break;
	default:
		pr_info("ALSA: Unknown pcm number.\n");
		break;
	}

#ifdef DEBUG_RECORD
	/* Create the file needs to after initalizing the sysem. */
	dpcm->fp = filp_open("/data/debug_record.wav", O_RDWR | O_CREAT, 0644);
	if (IS_ERR(dpcm->fp))
		pr_err("[create file error %s %d]\n", __func__, __LINE__);
	dpcm->pos = 0;
#endif

	spin_lock_init(&playback_lock);

	dpcm->card->mixer_volume[MIXER_ADDR_MASTER][0] =
	dpcm->card->mixer_volume[MIXER_ADDR_MASTER][1] =
	RPC_TOAGENT_GET_VOLUME(dpcm->phy_addr_rpc, dpcm->vaddr_rpc, dpcm);

	/* Preparing the ion buffer for share momery 1 */
	dpcm->g_ShareMemPtr_dat_rpc_dmabuf = ion_alloc(SHARE_MEM_SIZE, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(dpcm->g_ShareMemPtr_dat_rpc_dmabuf)) {
		pr_err("[ALSA %s %d fail]\n", __FUNCTION__, __LINE__);
		ret = PTR_ERR(dpcm->g_ShareMemPtr_dat_rpc_dmabuf);
		goto fail;
	}

	dpcm->g_ShareMemPtr_dat = rtk_rpc_ion_pa(dpcm->g_ShareMemPtr_dat_rpc_dmabuf->priv);
	if (dpcm->g_ShareMemPtr_dat == -1) {
		pr_err("[ALSA %s %d fail]\n", __FUNCTION__, __LINE__);
		goto fail;
	}
	dpcm->g_ShareMemPtr = rtk_rpc_ion_va(dpcm->g_ShareMemPtr_dat_rpc_dmabuf->priv);
	memset(dpcm->g_ShareMemPtr, 0, SHARE_MEM_SIZE);

	/* Preparing the ion buffer for share momery 2 */
	dpcm->g_ShareMemPtr_dat2_rpc_dmabuf = ion_alloc(SHARE_MEM_SIZE_LATENCY, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(dpcm->g_ShareMemPtr_dat2_rpc_dmabuf)) {
		pr_err("[ALSA %s %d fail]\n", __FUNCTION__, __LINE__);
		ret = PTR_ERR(dpcm->g_ShareMemPtr_dat2_rpc_dmabuf);
		goto fail;
	}

	dpcm->g_ShareMemPtr_dat2 = rtk_rpc_ion_pa(dpcm->g_ShareMemPtr_dat2_rpc_dmabuf->priv);
	if (dpcm->g_ShareMemPtr_dat2 == -1) {
		pr_err("[ALSA %s %d fail]\n", __FUNCTION__, __LINE__);
		goto fail;
	}
	dpcm->g_ShareMemPtr2 = rtk_rpc_ion_va(dpcm->g_ShareMemPtr_dat2_rpc_dmabuf->priv);
	memset(dpcm->g_ShareMemPtr2, 0, SHARE_MEM_SIZE_LATENCY);

	/* Set up the sync word for new latency structure */
	dpcm->g_ShareMemPtr2->sync = htonl(0x23792379);

	/* Preparing the ion buffer for share momery 3 */
	dpcm->g_ShareMemPtr_dat3_rpc_dmabuf = ion_alloc(SHARE_MEM_SIZE_LATENCY, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(dpcm->g_ShareMemPtr_dat3_rpc_dmabuf)) {
		pr_err("[ALSA %s %d fail]\n", __FUNCTION__, __LINE__);
		ret = PTR_ERR(dpcm->g_ShareMemPtr_dat3_rpc_dmabuf);
		goto fail;
	}

	dpcm->g_ShareMemPtr_dat3 = rtk_rpc_ion_pa(dpcm->g_ShareMemPtr_dat3_rpc_dmabuf->priv);
	if (dpcm->g_ShareMemPtr_dat3 == -1) {
		pr_err("[ALSA %s %d fail]\n", __FUNCTION__, __LINE__);
		goto fail;
	}
	dpcm->g_ShareMemPtr3 = rtk_rpc_ion_va(dpcm->g_ShareMemPtr_dat3_rpc_dmabuf->priv);
	memset(dpcm->g_ShareMemPtr3, 0, SHARE_MEM_SIZE_LATENCY);

	/* Set up the sync word for new latency structure */
	dpcm->g_ShareMemPtr3->sync = htonl(0x23792379);

	return ret;
fail:
	if (!IS_ERR(dpcm->rpc_dmabuf_rpc))
		rtk_rpc_free_ion(dpcm->rpc_dmabuf_rpc);

	if (!IS_ERR(rpc_dmabuf)) {
		pr_err("%s free alsa_playback rpc_dmabuf\n", __FUNCTION__);
		rtk_rpc_free_ion(rpc_dmabuf);
		dpcm = NULL;
		rpc_dmabuf = NULL;
		runtime->private_data = NULL;
	}

	return ret;
}

// it's for audio in
#ifdef CONFIG_REALTEK_FL3236
extern void fl3236_enable_light(void);
extern void fl3236_disable_light(void);
#endif

static int snd_card_capture_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = NULL;
	int ret = ENOMEM;
	phys_addr_t dat;
	struct dma_buf *rpc_dmabuf = NULL;

	rpc_dmabuf = ion_alloc(sizeof(struct snd_card_RTK_capture_pcm),
				RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(rpc_dmabuf)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		ret = PTR_ERR(rpc_dmabuf);
		goto fail;
	}

	dat = rtk_rpc_ion_pa(rpc_dmabuf->priv);
	if (dat == -1) {
		pr_info("[%s %d] alloc memory faild\n", __func__, __LINE__);
		goto fail;
	}

	dpcm = rtk_rpc_ion_va(rpc_dmabuf->priv);
	memset(dpcm, 0, sizeof(struct snd_card_RTK_capture_pcm));
	dpcm->phy_addr = dat;
	dpcm->rpc_dmabuf = rpc_dmabuf;

	pr_info("[ALSA %s %s %d]\n", substream->pcm->name, __func__, __LINE__);

	// private data
	runtime->private_data = dpcm;
	runtime->private_free = snd_card_capture_runtime_free;
	dpcm->substream = substream;

	pr_info("ALSA: Device number is %d\n", substream->pcm->device);
	switch (substream->pcm->device) {
	case 0:
		// it's for HDMI-RX
		dpcm->source_in = ENUM_AIN_HDMIRX;
		pr_info("ALSA: This is for HDMI-RX.\n");
		break;
	case 1:
		// it's for I2S
		dpcm->source_in = ENUM_AIN_I2S;
		pr_info("ALSA: This is for I2S.\n");
		break;
	case 3:
		// it's for audio processing CS demo
		dpcm->source_in = ENUM_AIN_AUDIO;
		pr_info("ALSA: This is for audio processing CS demo.\n");
#ifdef CONFIG_REALTEK_FL3236
		fl3236_enable_light();
#endif
		break;
	case 4:
		// it's for audio processing v2 DMIC LOOPBACK
		dpcm->source_in = ENUM_AIN_AUDIO_V2;
		pr_info("ALSA: This is for audio processing v2 DMIC LOOPBACK.\n");
		break;
	case 5:
		// it's for audio processing v3 SPEECH RECOGNITION FROM DMIC
		dpcm->source_in = ENUM_AIN_AUDIO_V3;
		pr_info("ALSA: This is for audio processing v3 SPEECH RECOGNITION FROM DMIC.\n");
		break;
	case 6:
		// it's for audio processing v4 SPEECH RECOGNITION FROM I2S
		dpcm->source_in = ENUM_AIN_AUDIO_V4;
		pr_info("ALSA: This is for audio processing v4 SPEECH RECOGNITION FROM I2S.\n");
		break;
	case 7:
		// it's for ao i2s loopback to ain
		dpcm->source_in = ENUM_AIN_I2S_LOOPBACK;
		pr_info("ALSA: This is for ao i2s loopback to ain.\n");
		break;
	case 8:
		// it's for dmic pass through to ao
		dpcm->source_in = ENUM_AIN_DMIC_PASSTHROUGH;
		pr_info("ALSA: This is for dmic pass through to ao.\n");
		/* Create global ao id */
		RPC_TOAGENT_CREATE_GLOBAL_AO(dpcm->phy_addr_rpc,
					dpcm->vaddr_rpc, &dpcm->AOAgentID);
		/* get ao flash pin ID */
		dpcm->AOpinID = RPC_TOAGENT_GET_AO_FLASH_PIN(
					dpcm->phy_addr_rpc, dpcm->vaddr_rpc, dpcm->AOAgentID);
		if (dpcm->AOpinID < 0) {
			pr_err("[can't get flash pin %s %d]\n", __func__, __LINE__);
			goto fail;
		}
		break;
	case 9:
		// it's for pure DMIC
		dpcm->source_in = ENUM_AIN_PURE_DMIC;
		pr_info("ALSA: This is for pure DMIC.\n");
		break;
	default:
		pr_info("ALSA: Not in the list maybe for non-pcm.\n");
		break;
	}

	if (dpcm->source_in == ENUM_AIN_AUDIO)
		memcpy(&runtime->hw, &snd_card_mars_capture_audio, sizeof(struct snd_pcm_hardware));
	else
		memcpy(&runtime->hw, &snd_card_mars_capture, sizeof(struct snd_pcm_hardware));

	// create or get AI
	if (snd_realtek_hw_create_AI(substream) < 0) {
		pr_err("[error %s %d]\n", __func__, __LINE__);
		ret = -ENOMEM;
		goto fail;
	}

	dpcm->card = (struct RTK_snd_card *)(substream->pcm->card->private_data);

	/* Preparing the ion buffer for closing capture */
	dpcm->rpc_dmabuf_rpc = ion_alloc(4096, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(dpcm->rpc_dmabuf_rpc)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		ret = PTR_ERR(dpcm->rpc_dmabuf_rpc);
		goto fail;
	}

	dpcm->phy_addr_rpc = rtk_rpc_ion_pa(dpcm->rpc_dmabuf_rpc->priv);
	if (dpcm->phy_addr_rpc == -1) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		goto fail;
	}
	dpcm->vaddr_rpc = rtk_rpc_ion_va(dpcm->rpc_dmabuf_rpc->priv);

	// init hr timer
	hrtimer_init(&dpcm->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	spin_lock_init(&capture_lock);

	ret = 0;

	return ret;
fail:
	if (!IS_ERR(rpc_dmabuf)) {
		pr_info("%s free alsa capture rpc_dmabuf\n", __func__);
		rtk_rpc_free_ion(rpc_dmabuf);
		dpcm = NULL;
		rpc_dmabuf = NULL;
	}

	return ret;
}

static int snd_card_capture_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	struct AUDIO_RPC_SENDIO sendio;
	ktime_t remaining;
	int ret = -1;

	pr_info("[ALSA %s %d]\n", __func__, __LINE__);

	if (dpcm->source_in == ENUM_AIN_AUDIO) {
		if (RPC_TOAGENT_AI_DISCONNECT_ALSA_AUDIO(
					dpcm->phy_addr_rpc, dpcm->vaddr_rpc, runtime)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			goto exit;
		}
		/* destory using the same rpc */
		snd_realtek_hw_capture_init_LPCM_ringheader_of_AI(runtime);
#ifdef CONFIG_REALTEK_FL3236
		fl3236_disable_light();
#endif
	} else {
		if (dpcm->source_in == ENUM_AIN_DMIC_PASSTHROUGH) {
			/* stop AO */
			RPC_TOAGENT_STOP_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
						dpcm->AOAgentID | dpcm->AOpinID);
		}

		/* stop AI */
		RPC_TOAGENT_STOP_SVC(dpcm->phy_addr_rpc,
					dpcm->vaddr_rpc, dpcm->AIAgentID);

		if (dpcm->source_in == ENUM_AIN_DMIC_PASSTHROUGH) {
			RPC_TOAGENT_RELEASE_AO_FLASH_PIN(dpcm->phy_addr_rpc,
						dpcm->vaddr_rpc, dpcm->AOAgentID, dpcm->AOpinID);
			// decoder flush
			sendio.instanceID = dpcm->AOAgentID;
			sendio.pinID = dpcm->AOpinID;

			if (RPC_TOAGENT_FLUSH_SVC(dpcm->phy_addr_rpc,
						dpcm->vaddr_rpc, &sendio)) {
				pr_err("[%s %d fail]\n", __func__, __LINE__);
				goto exit;
			}
		}

		/* destroy AI */
		if (RPC_TOAGENT_DESTROY_AI_FLOW_SVC(dpcm->phy_addr_rpc,
					dpcm->vaddr_rpc, dpcm->AIAgentID)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			goto exit;
		}
	}

	/* destroy the hr timer */
	remaining = hrtimer_get_remaining(&dpcm->hr_timer);
	if (ktime_to_ns(remaining) > 0)
		ndelay(ktime_to_ns(remaining));

	ret = hrtimer_cancel(&dpcm->hr_timer);
	if (ret) {
		pr_err("The timer still alive...\n");
		goto exit;
	}

	ret = 0;
exit:
	if (!IS_ERR(dpcm->rpc_dmabuf_rpc))
		rtk_rpc_free_ion(dpcm->rpc_dmabuf_rpc);

	snd_realtek_hw_capture_free_ring(runtime);

	if (snd_open_ai_count > 0)
		snd_open_ai_count--;

	return ret;
}

static int snd_card_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;
	struct AUDIO_RPC_SENDIO sendio;
	ktime_t remaining;
	int ret = 0;

	pr_info("[ALSA %s %d]\n", __func__, __LINE__);

	RPC_TOAGENT_PUT_SHARE_MEMORY_LATENCY(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
				NULL, NULL, 0, dpcm->AOpinID,
				ENUM_PRIVATEINFO_AUDIO_GET_SHARE_MEMORY_FROM_ALSA);

	if (dpcm->AOpinID) {

		/* stop decoder */
		if(dpcm->DECAgentID && dpcm->ao_decode_lpcm)
			RPC_TOAGENT_STOP_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc, dpcm->DECAgentID);

		// AO pause
		if (RPC_TOAGENT_PAUSE_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
					dpcm->AOAgentID | dpcm->AOpinID)) {
			ret = -1;
			goto exit;
		}

		if (dpcm->bInitRing != 0) {

			if (dpcm->ao_decode_lpcm) {
				// decoder flush
				sendio.instanceID = dpcm->DECAgentID;
				sendio.pinID = dpcm->DECpinID;

				if (RPC_TOAGENT_FLUSH_SVC(dpcm->phy_addr_rpc,
							dpcm->vaddr_rpc, &sendio)) {
					ret = -1;
					goto exit;
				}
			}

			/* stop AO */
			RPC_TOAGENT_STOP_SVC(dpcm->phy_addr_rpc,
						dpcm->vaddr_rpc, dpcm->AOAgentID | dpcm->AOpinID);

			dec_out_msec = 0;
		}

		/* destroy decoder instance if exist */
		if (dpcm->DECAgentID && dpcm->ao_decode_lpcm)
			RPC_TOAGENT_DESTROY_SVC(dpcm->phy_addr_rpc, dpcm->vaddr_rpc, dpcm->DECAgentID);
	} else
		return -1;

#ifdef DEBUG_RECORD
	/* Close the record file */
	filp_close(dpcm->fp, NULL);
#endif

	/* destroy the hr timer */
	remaining = hrtimer_get_remaining(&dpcm->hr_timer);
	if (ktime_to_ns(remaining) > 0)
		ndelay(ktime_to_ns(remaining));

	ret = hrtimer_cancel(&dpcm->hr_timer);
	if (ret) {
		pr_err("The timer still alive...\n");
		goto exit;
	}

exit:
	RPC_TOAGENT_RELEASE_AO_FLASH_PIN(dpcm->phy_addr_rpc,
				dpcm->vaddr_rpc, dpcm->AOAgentID, dpcm->AOpinID);
	snd_open_count--;

	if (!IS_ERR(dpcm->rpc_dmabuf_rpc))
		rtk_rpc_free_ion(dpcm->rpc_dmabuf_rpc);

	snd_realtek_hw_free_ring(runtime);

	if (!IS_ERR(dpcm->g_ShareMemPtr_dat_rpc_dmabuf))
		rtk_rpc_free_ion(dpcm->g_ShareMemPtr_dat_rpc_dmabuf);

	if (!IS_ERR(dpcm->g_ShareMemPtr_dat2_rpc_dmabuf))
		rtk_rpc_free_ion(dpcm->g_ShareMemPtr_dat2_rpc_dmabuf);

	if (!IS_ERR(dpcm->g_ShareMemPtr_dat3_rpc_dmabuf))
		rtk_rpc_free_ion(dpcm->g_ShareMemPtr_dat3_rpc_dmabuf);

	return ret;
}

static unsigned int snd_capture_monitor_delay(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	unsigned int sample_rate = runtime->rate;
	unsigned int ret = 0;
	unsigned long base, limit, rp, wp;
	unsigned int pcm_size;  // unit: sample
	unsigned int pcm_latency;   // unit ms
	unsigned int lpcm_size; // unit: sample
	unsigned int lpcm_latency;  // unit:ms

	// calculate the size of PCM (input of AI)
	base = (unsigned long)(dpcm->nAIRing_LE[0].beginAddr);
	limit = (unsigned long)(dpcm->nAIRing_LE[0].beginAddr + dpcm->nAIRing_LE[0].size);
	wp = (unsigned long)(ntohl(dpcm->nAIRing[0].writePtr));
	rp = (unsigned long)(ntohl(dpcm->nAIRing[0].readPtr[0]));
	pcm_size = ring_valid_data(base, limit, rp, wp) >> 2;

	// calculate the size of LPCM (output of AI)
	base = (unsigned long)(dpcm->nLPCMRing_LE.beginAddr);
	limit = (unsigned long)(dpcm->nLPCMRing_LE.beginAddr + dpcm->nLPCMRing_LE.size);
	wp = (unsigned long)(ntohl(dpcm->nLPCMRing.writePtr));
	rp = (unsigned long)(ntohl(dpcm->nLPCMRing.readPtr[0]));
	lpcm_size = ring_valid_data(base, limit, rp, wp);

	switch (dpcm->nAIFormat) {
	case AUDIO_ALSA_FORMAT_16BITS_LE_LPCM:
		lpcm_size >>= 2;    // 2ch, 2bytes per sample
		break;
	case AUDIO_ALSA_FORMAT_24BITS_LE_LPCM:
		lpcm_size /= 6; // 2ch, 3bytes per sample
		break;
	default:
		pr_err("capture err, %d @ %s %d\n", dpcm->nAIFormat, __func__, __LINE__);
	}

	// calculate leatency
	pcm_latency = (pcm_size * 1000) / sample_rate;
	lpcm_latency = (lpcm_size * 1000) / sample_rate;
	ret = pcm_latency + lpcm_latency;

	return ret;
}

static int snd_card_capture_ioctl(struct snd_pcm_substream *substream,  unsigned int cmd, void *arg)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	int delay_ms;
	unsigned int *volume = NULL;

	switch (cmd) {
	case SNDRV_PCM_IOCTL_VOLUME_SET:
		volume = (unsigned int *)arg;
		return RPC_TOAGENT_SET_SOFTWARE_AI_FLASH_VOLUME(dpcm->phy_addr_rpc, dpcm->vaddr_rpc, dpcm, *volume);
	case SNDRV_PCM_IOCTL_VOLUME_GET:
		break;
	case SNDRV_PCM_IOCTL_GET_FW_DELAY:
		delay_ms = snd_capture_monitor_delay(substream);
		put_user(delay_ms, (unsigned int *) arg);
		break;
	default:
		return snd_pcm_lib_ioctl(substream, cmd, arg);
	}

	return 0;
}

static int snd_card_playback_ioctl(struct snd_pcm_substream *substream,  unsigned int cmd, void *arg)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;
	struct AUDIO_RPC_EQUALIZER_MODE *equalizer_mode;
	struct dma_buf *buf;
	int mmap_fd, mLatency;
	snd_pcm_sframes_t n;

	switch (cmd) {
	case SNDRV_PCM_IOCTL_VOLUME_SET:
		printk("[ALSA SET VOLUME]\n");
		get_user(dpcm->volume, (int *) arg);
		if (dpcm->volume < 0)
			dpcm->volume = 0;
		if (dpcm->volume > 31)
			dpcm->volume = 31;
		RPC_TOAGENT_SET_AO_FLASH_VOLUME(dpcm->phy_addr_rpc, dpcm->vaddr_rpc, dpcm);
		break;
	case SNDRV_PCM_IOCTL_VOLUME_GET:
		put_user(dpcm->volume, (int *) arg);
		break;
	case SNDRV_PCM_IOCTL_GET_LATENCY:
		mLatency = 0;
		mLatency = snd_monitor_audio_data_queue_new(substream);
		put_user(mLatency, (int *) arg);
		break;
	case SNDRV_PCM_IOCTL_GET_FW_DELAY:
		mLatency = 0;
		n = 0;
		mLatency = snd_monitor_audio_data_queue_new(substream);
		n = (mLatency * runtime->rate) / 1000;
		put_user(n, (snd_pcm_sframes_t *) arg);
		break;
	case SNDRV_PCM_IOCTL_EQ_SET:
		equalizer_mode = kmalloc(sizeof(struct AUDIO_RPC_EQUALIZER_MODE), GFP_KERNEL);
		if(copy_from_user((void *)equalizer_mode, (void *)arg, sizeof(struct AUDIO_RPC_EQUALIZER_MODE)))
			printk("%s %d copy data fail\n",__func__, __LINE__);
		RPC_TOAGENT_SET_EQ(dpcm->phy_addr_rpc, dpcm->vaddr_rpc, dpcm, equalizer_mode);
		break;
	case SNDRV_PCM_IOCTL_AO_PARAMETER_SET:
		dpcm->ao_paramter = kmalloc(sizeof(struct AUDIO_RPC_OUT_PARAMETER), GFP_KERNEL);
		if(copy_from_user((void *)dpcm->ao_paramter, (void *)arg, sizeof(struct AUDIO_RPC_OUT_PARAMETER)))
			printk("%s %d copy ao parameter fail\n",__func__, __LINE__);
		break;
	case SNDRV_PCM_IOCTL_MMAP_DATA_FD:
		buf = substream->runtime->dma_buffer_p->rtk_ion_rpc_dmabuf;
		mmap_fd = dma_buf_fd(buf, O_CLOEXEC);
		if (mmap_fd < 0)
			pr_info("%s %d Get mmap buffer fd fail.\n", __func__, __LINE__);
		put_user(mmap_fd, (int *) arg);
		break;
	default:
		return snd_pcm_lib_ioctl(substream, cmd, arg);
		break;
	}

	return 0;
}

static snd_pcm_uframes_t snd_card_capture_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	snd_pcm_uframes_t ret = 0;

	// update hw_ptr
	ret = dpcm->nTotalWrite % runtime->buffer_size;
	return ret;
}

static snd_pcm_uframes_t snd_card_playback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;
	snd_pcm_uframes_t ret = 0;

	// update runtime->status->hw_ptr
	ret = dpcm->nTotalRead % runtime->buffer_size;
	return ret;
}

static int snd_card_capture_prepare_32bits_BE(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;

	// malloc AI ring buf
	if (snd_realtek_hw_capture_malloc_ring(runtime))	{
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	// init AI ring header
	if (snd_realtek_hw_capture_init_ringheader_of_AI(runtime)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	// config for different protocol
	if (dpcm->source_in == ENUM_AIN_HDMIRX)
		RPC_TOAGENT_AI_CONFIG_HDMI_RX_IN(dpcm->phy_addr_rpc, dpcm->vaddr_rpc, dpcm); // config HDMI-RX
	else if (dpcm->source_in == ENUM_AIN_I2S)
		RPC_TOAGENT_AI_CONFIG_I2S_IN(dpcm->phy_addr_rpc, dpcm->vaddr_rpc, dpcm); // config I2S

	// private info
	if (RPC_TOAGENT_AI_CONNECT_ALSA(dpcm->phy_addr_rpc, dpcm->vaddr_rpc, runtime)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	// run
	if (snd_realtek_hw_capture_run(dpcm)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	dpcm->hr_timer.function = &snd_card_capture_lpcm_timer_function;

	return 0;
}

static int snd_card_capture_prepare_LPCM_audio(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;

	// malloc AI ring buf
	if (snd_realtek_hw_capture_malloc_ring(runtime))	{
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	// malloc LPCM ring buf
	if (snd_realtek_hw_capture_malloc_lpcm_ring(runtime)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	// init LPCM ring header of AI
	if (snd_realtek_hw_capture_init_LPCM_ringheader_of_AI(runtime)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	// private info
	if (RPC_TOAGENT_AI_CONNECT_ALSA(dpcm->phy_addr_rpc, dpcm->vaddr_rpc, runtime)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	dpcm->hr_timer.function = &snd_card_capture_lpcm_timer_function;

	return 0;
}

extern int
RPC_TOAGENT_AI_CONFIG_I2S_LOOPBACK_IN(phys_addr_t paddr, void *vaddr,
				      struct snd_card_RTK_capture_pcm *dpcm);

static int snd_card_capture_prepare_LPCM(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;

	pr_info("[ @ %s %d]\n", __func__, __LINE__);

	/* decide ai ring buf or not */
	switch (dpcm->source_in) {
	case ENUM_AIN_AUDIO_V2:
	case ENUM_AIN_AUDIO_V3:
	case ENUM_AIN_AUDIO_V4:
		break;
	default:
		// malloc AI ring buf
		if (snd_realtek_hw_capture_malloc_ring(runtime)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			return -ENOMEM;
		}

		// init ring header of AI
		if (snd_realtek_hw_capture_init_ringheader_of_AI(runtime)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			return -ENOMEM;
		}
		break;
	}

	/* decide pts buf or not */
	switch (dpcm->source_in) {
	case ENUM_AIN_AUDIO_V2:
	case ENUM_AIN_AUDIO_V3:
	case ENUM_AIN_AUDIO_V4:
	case ENUM_AIN_I2S_LOOPBACK:
	case ENUM_AIN_DMIC_PASSTHROUGH:
		break;
	default:
#ifdef CAPTURE_USE_PTS_RING
		// malloc PTS ring buf
		if (snd_realtek_hw_capture_malloc_pts_ring(runtime)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			return -ENOMEM;
		}

		// init PTS ring header of AI
		if (snd_realtek_hw_capture_init_PTS_ringheader_of_AI(runtime)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			return -ENOMEM;
		}
#endif
		break;
	}

	// malloc LPCM ring buf
	if (snd_realtek_hw_capture_malloc_lpcm_ring(runtime)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	// init LPCM ring header of AI
	if (snd_realtek_hw_capture_init_LPCM_ringheader_of_AI(runtime)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	// config for different protocol
	switch (dpcm->source_in) {
	case ENUM_AIN_AUDIO_V2:
	case ENUM_AIN_AUDIO_V3:
	case ENUM_AIN_AUDIO_V4:
		// For audio processing flow, doing connect alsa before config audio v2 or v3
		if (RPC_TOAGENT_AI_CONNECT_ALSA(dpcm->phy_addr_rpc,
					dpcm->vaddr_rpc, runtime)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			return -ENOMEM;
		}
		RPC_TOAGENT_AI_CONFIG_AUDIO_IN(dpcm->phy_addr_rpc,
					dpcm->vaddr_rpc, dpcm); // config audio v2 or v3
		break;
	case ENUM_AIN_HDMIRX:
		RPC_TOAGENT_AI_CONFIG_HDMI_RX_IN(dpcm->phy_addr_rpc,
					dpcm->vaddr_rpc, dpcm); // config HDMI-RX
		break;
	case ENUM_AIN_I2S:
		RPC_TOAGENT_AI_CONFIG_I2S_IN(dpcm->phy_addr_rpc,
					dpcm->vaddr_rpc, dpcm); // config I2S
		break;
	case ENUM_AIN_I2S_LOOPBACK:
		RPC_TOAGENT_AI_CONFIG_I2S_LOOPBACK_IN(dpcm->phy_addr_rpc,
					dpcm->vaddr_rpc, dpcm); // config I2S loopback
		break;
	case ENUM_AIN_DMIC_PASSTHROUGH:
		// init ringheader of AO_using ai ring
		if (snd_realtek_init_AO_ringheader_by_AI(runtime)) {
			pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
			return -ENOMEM;
		}
		// connect AI and AO
		if (snd_realtek_hw_init_connect(dpcm->phy_addr_rpc,
					dpcm->vaddr_rpc, dpcm->AIAgentID,
					dpcm->AOAgentID, dpcm->AOpinID)) {
			pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
			return -ENOMEM;
		}
        // It must be called when AI pass through
		if (RPC_TOAGENT_AI_CONNECT_AO(dpcm->phy_addr_rpc,
					dpcm->vaddr_rpc, dpcm)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			return -ENOMEM;
		}
		// config dmic passthrough
		RPC_TOAGENT_AI_CONFIG_DMIC_PASSTHROUGH_IN(
					dpcm->phy_addr_rpc, dpcm->vaddr_rpc, dpcm);
		break;
	case ENUM_AIN_PURE_DMIC:
		// config pure dmic
		RPC_TOAGENT_AI_CONFIG_DMIC_PASSTHROUGH_IN(
					dpcm->phy_addr_rpc, dpcm->vaddr_rpc, dpcm);
		break;
	default:
		break;
	}

	// private info
	switch (dpcm->source_in) {
	case ENUM_AIN_AUDIO_V2:
	case ENUM_AIN_AUDIO_V3:
	case ENUM_AIN_AUDIO_V4:
		break;
	default:
		if (RPC_TOAGENT_AI_CONNECT_ALSA(dpcm->phy_addr_rpc, dpcm->vaddr_rpc, runtime)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			return -ENOMEM;
		}
		break;
	}

	// run
	switch (dpcm->source_in) {
	case ENUM_AIN_AUDIO_V2:
	case ENUM_AIN_AUDIO_V3:
	case ENUM_AIN_AUDIO_V4:
		break;
	default:
		if (snd_realtek_hw_capture_run(dpcm)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			return -ENOMEM;
		}
		break;
	}

	dpcm->hr_timer.function = &snd_card_capture_lpcm_timer_function;

	return 0;
}

static int snd_card_capture_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;

	pr_err("[ALSA %s %d]\n", __func__, __LINE__);

	if (runtime->status->state == SNDRV_PCM_STATE_XRUN)
		pr_err("[SNDRV_PCM_STATE_XRUN appl_ptr %d hw_ptr %d]\n", (int)runtime->control->appl_ptr, (int)runtime->status->hw_ptr);

	/* Setup the hr timer using ktime */
	dpcm->ktime = ktime_set(0, (runtime->period_size * 1000) / runtime->rate * 1000 * 1000); //ms to ns

	pr_info("\n\n\n");
	pr_info("Capture:");
	pr_info("rate %d channels %d format %x\n", runtime->rate, runtime->channels, runtime->format);
	pr_info("period_size %d periods %d\n", (int)runtime->period_size, (int)runtime->periods);
	pr_info("buffer_size %d\n", (int)runtime->buffer_size);
	pr_info("start_threshold %d stop_threshold %d\n", (int)runtime->start_threshold, (int)runtime->stop_threshold);
	pr_info("[runtime->frame_bits %d]\n", runtime->frame_bits);
	pr_info("[runtime->sample_bits %d]\n", runtime->sample_bits);
	pr_info("\n\n\n");

	switch (runtime->access) {
	case SNDRV_PCM_ACCESS_MMAP_INTERLEAVED:
	case SNDRV_PCM_ACCESS_RW_INTERLEAVED:
		switch (runtime->format) {
		case SNDRV_PCM_FORMAT_S16_LE:
			pr_info("[SNDRV_PCM_FORMAT_S16_LE]\n");
			dpcm->nAIFormat = AUDIO_ALSA_FORMAT_16BITS_LE_LPCM;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			pr_info("[SNDRV_PCM_FORMAT_S24_LE]\n");
			dpcm->nAIFormat = AUDIO_ALSA_FORMAT_24BITS_LE_LPCM;
			break;
		case SNDRV_PCM_FORMAT_S24_3LE:
			pr_info("[SNDRV_PCM_FORMAT_S24_3LE]\n");
			dpcm->nAIFormat = AUDIO_ALSA_FORMAT_24BITS_LE_LPCM;
			break;
		default:
			pr_err("[unsupport format %d %s %d]\n", runtime->format
				, __func__, __LINE__);
			return -1;
		}
		break;
	case SNDRV_PCM_ACCESS_MMAP_NONINTERLEAVED:
	case SNDRV_PCM_ACCESS_RW_NONINTERLEAVED:
	default:
		pr_err("[unsupport access @ %s %d]\n", __func__, __LINE__);
		return -1;
	}

	if (dpcm->bInitRing) {
		dpcm->nTotalWrite = 0;
		pr_err("[Re-Prepare %d %d %s %d]\n",
					(int)runtime->control->appl_ptr,
					(int)runtime->status->hw_ptr,
					__func__, __LINE__);
		return 0;
	}

	dpcm->nPeriodBytes = frames_to_bytes(runtime, runtime->period_size);
	dpcm->nFrameBytes = frames_to_bytes(runtime, 1);

	switch (dpcm->nAIFormat) {
	case AUDIO_ALSA_FORMAT_32BITS_BE_PCM:
		pr_err("ALSA: unsupport @ %s %d\n", __func__, __LINE__);
		if (snd_card_capture_prepare_32bits_BE(substream)) {
			pr_err("[%s %d fail]\n", __func__, __LINE__);
			return -ENOMEM;
		}
		break;
	case AUDIO_ALSA_FORMAT_16BITS_LE_LPCM:
	case AUDIO_ALSA_FORMAT_24BITS_LE_LPCM:
		if (dpcm->source_in == ENUM_AIN_AUDIO) {
			if (snd_card_capture_prepare_LPCM_audio(substream)) {
				pr_err("[%s %d fail]\n", __func__, __LINE__);
				return -ENOMEM;
			}
		} else {
			if (snd_card_capture_prepare_LPCM(substream)) {
				pr_err("[%s %d fail]\n", __func__, __LINE__);
				return -ENOMEM;
			}
		}
		break;
	default:
		pr_err("[ALSA err %s %d]\n", __func__, __LINE__);
		break;
	}

	dpcm->bInitRing = 1;

	return 0;
}

static int snd_card_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;

	pr_info("[ALSA %s %d]\n", __func__, __LINE__);

	if (is_suspend) {
		pr_err("[ALSA %s %d] suspend\n", __func__, __LINE__);
		return 0;
	}

	if (runtime->status->state == SNDRV_PCM_STATE_XRUN)
		pr_err("[SNDRV_PCM_STATE_XRUN appl_ptr %d hw_ptr %d]\n", (int)runtime->control->appl_ptr, (int)runtime->status->hw_ptr);

	/* Setup the hr timer using ktime.
	 * For more precise compute the ktime in us.
	 */
	dpcm->ktime = ktime_set(0, (runtime->period_size * 1000) * 1000 / runtime->rate * 1000); //ms to ns

	if (dpcm->bInitRing) {
		/* Reset the information about playback.
		 * nPreHWPtr will influence the next information update in timer function.
		 */
		dpcm->nHWReadSize = 0;
		dpcm->nTotalRead = 0;
		dpcm->nTotalWrite = 0;
		dpcm->nPreHWPtr = 0;
		dpcm->nPre_appl_ptr = 0;
		/*	DHCKYLIN-2193:
		 *	1.  pasue and stop AO and decoder, then destory the decoder for re-sending NewFormat.
		 *	2.  re-send NewFormat cmd to audio f/w when alsa is reconfigured.
		 */
		snd_realtek_reprepare(runtime);
		pr_info("[Re-Prepare %d %d %s %d]\n",
					(int)runtime->control->appl_ptr,
					(int)runtime->status->hw_ptr,
					__func__, __LINE__);
	}

	pr_info("[======START======]\n");
	pr_info("[playback : rate %d channels %d]\n", runtime->rate, runtime->channels);
	pr_info("[period_size %d periods %d]\n", (int)runtime->period_size, (int)runtime->periods);
	pr_info("[buffer_size %d ktime %d]\n", (int)runtime->buffer_size, (int)ktime_to_ns(dpcm->ktime));
	pr_info("[start_threshold %d stop_threshold %d]\n", (int)runtime->start_threshold, (int)runtime->stop_threshold);
	pr_info("[runtime->access %d]\n", runtime->access);
	pr_info("[runtime->format %d]\n", runtime->format);
	pr_info("[runtime->frame_bits %d]\n", runtime->frame_bits);
	pr_info("[runtime->sample_bits %d]\n", runtime->sample_bits);
	pr_info("[runtime->silence_threshold %d]\n", (int)runtime->silence_threshold);
	pr_info("[runtime->silence_size %d]\n", (int)runtime->silence_size);
	pr_info("[runtime->boundary %d]\n", (int)runtime->boundary);
	pr_info("[runtime->min_align %d]\n", (int)runtime->min_align);
	pr_info("[runtime->hw_ptr_base %x]\n", (unsigned int)(uintptr_t)runtime->hw_ptr_base);
	pr_info("[runtime->dma_area %s]\n", runtime->dma_area);
	pr_info("[======END======]\n");

	dpcm->nPeriodBytes = frames_to_bytes(runtime, runtime->period_size);
	dpcm->last_channel = runtime->channels;

	if (dpcm->ao_decode_lpcm) {
		// create decoder agent
		if (RPC_TOAGENT_CREATE_DECODER_AGENT(dpcm->phy_addr_rpc,
					dpcm->vaddr_rpc, &dpcm->DECAgentID, &dpcm->DECpinID)) {
			pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
			return -ENOMEM;
		}

		RPC_TOAGENT_PUT_SHARE_MEMORY_LATENCY(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
					(void *)dpcm->g_ShareMemPtr_dat, (void *)dpcm->g_ShareMemPtr_dat2,
					dpcm->DECAgentID, dpcm->AOpinID, ENUM_PRIVATEINFO_AUDIO_GET_SHARE_MEMORY_FROM_ALSA);

		// malloc ao_inring
		if (snd_realtek_hw_malloc_ring(runtime)) {
			pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
			return -ENOMEM;
		}

		// init ringheader of decoder_outring and AO_inring
		if (snd_realtek_hw_init_ringheader_of_DEC_AO(runtime)) {
			pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
			return -ENOMEM;
		}

		// connect decoder and AO by RPC
		if (snd_realtek_hw_init_connect(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
					dpcm->DECAgentID, dpcm->AOAgentID, dpcm->AOpinID)) {
			pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
			return -ENOMEM;
		}

		// 1. init decoder in_ring
		// 2. init decoder inband ring
		if (snd_realtek_hw_init_decoder_inring(runtime)) {
			pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
			return -ENOMEM;
		}
	} else {
		RPC_TOAGENT_PUT_SHARE_MEMORY_LATENCY(dpcm->phy_addr_rpc, dpcm->vaddr_rpc,
					(void *)dpcm->g_ShareMemPtr_dat, (void *)dpcm->g_ShareMemPtr_dat2,
					-1, dpcm->AOpinID, ENUM_PRIVATEINFO_AUDIO_GET_SHARE_MEMORY_FROM_ALSA);

		// init ringheader of AO_inring without decoder
		if (snd_realtek_hw_init_ringheader_of_AO(runtime)) {
			pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
			return -ENOMEM;
		}

		if (RPC_TOAGENT_AO_CONFIG_WITHOUT_DECODER(
					dpcm->phy_addr_rpc, dpcm->vaddr_rpc, runtime)) {
			pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
			return -1;
		}
	}

	// AO pause
	if (RPC_TOAGENT_PAUSE_SVC(dpcm->phy_addr_rpc,
				dpcm->vaddr_rpc, dpcm->AOAgentID | dpcm->AOpinID)) {
		pr_err("[%s %d fail]\n", __FUNCTION__, __LINE__);
		return -1;
	}

	if (dpcm->ao_decode_lpcm) {
		// decoder pause
		// decoder run
		// decoder flush
		// write decoder info into inband of decoder
		if (snd_realtek_hw_init_decoder_info(runtime)) {
			pr_err("[%s %d]\n", __FUNCTION__, __LINE__);
			return -ENOMEM;
		}
	}

	if (snd_realtek_hw_resume(dpcm)) {
		pr_err("[%s %d]\n", __func__, __LINE__);
		return -ENOMEM;
	}

	dpcm->bInitRing = 1;

	return 0;
}

static long snd_card_get_ring_data(struct RINGBUFFER_HEADER *pRing_BE, struct RINGBUFFER_HEADER *pRing_LE)
{
	unsigned long base, limit, rp, wp, data_size;

	base = (unsigned long)pRing_LE->beginAddr;
	limit = (unsigned long)(pRing_LE->beginAddr + pRing_LE->size);
	wp = (unsigned long)(ntohl(pRing_BE->writePtr));
	rp = (unsigned long)(ntohl(pRing_BE->readPtr[0]));

	data_size = ring_valid_data(base, limit, rp, wp);

	return data_size;
}

static long ring_memcpy2_buf(char *buf, unsigned long base, unsigned long limit, unsigned long ptr, unsigned int size)
{
	if (ptr + size <= limit) {
		memcpy(buf, (char *)ptr, size);
	} else {
		int i = limit-ptr;

		memcpy((char *)buf, (char *)ptr, i);
		memcpy((char *)(buf+i), (char *)base, size-i);
	}

	ptr += size;
	if (ptr >= limit)
		ptr = base + (ptr-limit);

	return ptr;
}

uint64_t snd_card_get_90k_pts(void)
{
	return refclk_get_counter(snd_refclk);
}
EXPORT_SYMBOL(snd_card_get_90k_pts);

static void snd_card_capture_setup_pts(struct snd_pcm_runtime *runtime, struct AUDIO_DEC_PTS_INFO *pPkt)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	uint64_t curPTS;
	unsigned int pcm_pts_hi, pcm_pts_lo;
	uint64_t pcmPTS = 0;
	uint64_t diffPTS;

	curPTS = snd_card_get_90k_pts();
	pcm_pts_hi = ntohl(pPkt->PTSH);
	pcm_pts_lo = ntohl(pPkt->PTSL);

	pcmPTS = (((uint64_t)pcm_pts_hi) << 32) | ((uint64_t)pcm_pts_lo);

	diffPTS = curPTS - pcmPTS;

	ktime_get_ts64(&dpcm->ts);
	dpcm->ts.tv_sec -= (div64_ul(diffPTS, 90000));
	dpcm->ts.tv_nsec -= (div64_ul(diffPTS * 100000, 9));
}

static void snd_card_capture_calculate_pts(struct snd_pcm_runtime *runtime, long nPeriodCount)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	struct AUDIO_DEC_PTS_INFO nPkt[2];
	struct AUDIO_RINGBUF_PTR_64 *pRing = &dpcm->nPTSRing;
	unsigned long temp_rp;
	unsigned int lpcm_rp = dpcm->nLPCMRing_LE.readPtr[0];
	unsigned int nPkt_ptr[2] = {0};
	unsigned int wp_offset, rp_offset, loop_count = 0;

	// refresh wp of pts_ring
	wp_offset = ntohl(dpcm->nPTSRingHdr.writePtr) - (unsigned int)dpcm->nPTSMem.pPhy;
	pRing->wp = pRing->base + (unsigned long)wp_offset;

	// get the first PTS
	if (ring_valid_data(pRing->base, pRing->limit, pRing->rp, pRing->wp) >= sizeof(struct AUDIO_DEC_PTS_INFO)) {
		temp_rp = ring_memcpy2_buf((char *)&nPkt[0], pRing->base, pRing->limit, pRing->rp, sizeof(struct AUDIO_DEC_PTS_INFO));
	} else {
		pr_err("Err @ %s %d\n", __func__, __LINE__);
		goto exit;
	}

	do {
		if (ring_valid_data(pRing->base, pRing->limit, temp_rp, pRing->wp) >= sizeof(struct AUDIO_DEC_PTS_INFO)) {
			ring_memcpy2_buf((char *)&nPkt[1], pRing->base, pRing->limit, temp_rp, sizeof(struct AUDIO_DEC_PTS_INFO));
		} else {
			if (loop_count == 0) {
				snd_card_capture_setup_pts(runtime, &nPkt[0]);
				goto exit;
			} else {
				// get the last packet as PTS needed
				snd_card_capture_setup_pts(runtime, &nPkt[0]);
				pRing->rp = ring_minus(pRing->base, pRing->limit, temp_rp, sizeof(struct AUDIO_DEC_PTS_INFO));
				break;
			}
		}

		nPkt_ptr[0] = ntohl(nPkt[0].wPtr);
		nPkt_ptr[1] = ntohl(nPkt[1].wPtr);

		if (ring_check_ptr_valid_32(nPkt_ptr[0], nPkt_ptr[1], lpcm_rp)) {
			// get PTS
			snd_card_capture_setup_pts(runtime, &nPkt[0]);
			pRing->rp = ring_minus(pRing->base, pRing->limit, temp_rp, sizeof(struct AUDIO_DEC_PTS_INFO));
			break;
		}

		temp_rp = ring_add(pRing->base, pRing->limit, temp_rp, sizeof(struct AUDIO_DEC_PTS_INFO));
		nPkt[0] = nPkt[1];

		loop_count++;
	} while (1);

	// update rp
	rp_offset = pRing->rp - pRing->base;
	dpcm->nPTSRingHdr.readPtr[0] = htonl((unsigned int)dpcm->nPTSMem.pPhy + rp_offset);

exit:
	return;
}

static void snd_card_capture_LPCM_copy(struct snd_pcm_runtime *runtime, long nPeriodCount)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	snd_pcm_uframes_t nFrameSize = nPeriodCount * runtime->period_size;
	snd_pcm_uframes_t dma_wp = dpcm->nTotalWrite % runtime->buffer_size;
	struct AUDIO_RINGBUF_PTR_64 src_ring, dst_ring;

	src_ring.base = (unsigned long)dpcm->pLPCMData;
	src_ring.limit = (unsigned long)(src_ring.base + dpcm->nLPCMRing_LE.size);
	src_ring.rp = src_ring.base
		+ (unsigned long)(dpcm->nLPCMRing_LE.readPtr[0] - dpcm->nLPCMRing_LE.beginAddr);

	dst_ring.base = (unsigned long)runtime->dma_area;
	dst_ring.limit = (unsigned long)(runtime->dma_area + runtime->buffer_size * dpcm->nFrameBytes);
	dst_ring.wp = (unsigned long)(runtime->dma_area + dma_wp * dpcm->nFrameBytes);

	ring1_to_ring2_general_64(&src_ring, &dst_ring, nFrameSize * dpcm->nFrameBytes, NULL);
}

int snd_card_capture_get_time_info(struct snd_pcm_substream *substream,
			struct timespec64 *system_ts, struct timespec64 *audio_ts,
			struct snd_pcm_audio_tstamp_config *audio_tstamp_config,
			struct snd_pcm_audio_tstamp_report *audio_tstamp_report)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;

	*system_ts = dpcm->ts;

	return 0;
}

static void snd_card_capture_handle_HDMI_plug_out(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	unsigned int nPeriodCount = 1;
	snd_pcm_uframes_t nFrameSize = nPeriodCount * runtime->period_size;
	snd_pcm_uframes_t dma_wp = dpcm->nTotalWrite % runtime->buffer_size;
	struct AUDIO_RINGBUF_PTR_64 dst_ring;
	char *pBuf = kmalloc(nFrameSize * dpcm->nFrameBytes, GFP_KERNEL);
	unsigned long free_size;

	if (pBuf == NULL) {
		pr_err("malloc FAILED @ %s %d\n", __func__, __LINE__);
		return;
	}

	free_size = snd_pcm_capture_hw_avail(runtime);
	if (free_size <= runtime->period_size) {
		pr_err("over flow %d %d @ %s %d\n", (int)free_size, (int)runtime->period_size, __func__, __LINE__);
		kfree(pBuf);
		return;
	}

	pr_err(" @ %s\n", __func__);
	// copy MUTE to DMA buffer
	memset(pBuf, 0, nFrameSize * dpcm->nFrameBytes);
	dst_ring.base = (unsigned long)runtime->dma_area;
	dst_ring.limit = (unsigned long)(runtime->dma_area + runtime->buffer_size * dpcm->nFrameBytes);
	dst_ring.wp = (unsigned long)(runtime->dma_area + dma_wp * dpcm->nFrameBytes);
	buf_memcpy2_ring(dst_ring.base, dst_ring.limit, dst_ring.wp, pBuf, nFrameSize * dpcm->nFrameBytes);
	kfree(pBuf);

	// paste time stamp
	ktime_get_ts64(&dpcm->ts);

	dpcm->nTotalWrite += nPeriodCount * runtime->period_size;
	snd_pcm_period_elapsed(substream);
}

static enum hrtimer_restart snd_card_capture_lpcm_timer_function(struct hrtimer *timer)
{
	struct snd_card_RTK_capture_pcm *dpcm =
		container_of(timer, struct snd_card_RTK_capture_pcm, hr_timer);
	struct snd_pcm_substream *substream = dpcm->substream;
	struct snd_pcm_runtime *runtime = substream->runtime;

	snd_pcm_uframes_t nRingDataFrame, free_size;
	long nRingDataSize; // bytes
	unsigned int nPeriodCount = 0, free_period;

	if (dpcm->enHRTimer == HRTIMER_RESTART) {

		if (ring_valid_data(0,
				   (unsigned long)runtime->boundary,
				   (unsigned long)runtime->control->appl_ptr,
				   (unsigned long)runtime->status->hw_ptr) > runtime->buffer_size) {
			pr_err("[hw_ptr %d appl_ptr %d %d @ %s %d]\n"
				, (int)runtime->status->hw_ptr
				, (int)runtime->control->appl_ptr
				, (int)runtime->buffer_size, __func__, __LINE__);
		}

		// check if HDMI-RX plug out
		if (dpcm->source_in == ENUM_AIN_HDMIRX) {
			if (snd_realtek_capture_check_hdmirx_enable() == 0) {
				snd_card_capture_handle_HDMI_plug_out(substream);
				goto SET_TIMER;
			}
		}

		nRingDataSize = snd_card_get_ring_data(&dpcm->nLPCMRing, &dpcm->nLPCMRing_LE);
		nRingDataFrame = nRingDataSize / dpcm->nFrameBytes;
		if (nRingDataFrame >= runtime->period_size) {
			nPeriodCount = nRingDataFrame / runtime->period_size;

			if (nPeriodCount == runtime->periods)
				nPeriodCount--;

			// check overflow
			{
				free_size = runtime->buffer_size - ring_valid_data(0
					, (unsigned long)runtime->boundary
					, (unsigned long)runtime->control->appl_ptr
					, (unsigned long)runtime->status->hw_ptr);
				free_period = free_size / runtime->period_size;
				nPeriodCount = min(nPeriodCount, free_period);
				if (nPeriodCount == 0)
					goto SET_TIMER;
			}

			// copy data from LPCM_ring to dma_buf
			snd_card_capture_LPCM_copy(runtime, nPeriodCount);

#ifdef CAPTURE_USE_PTS_RING
			// calculate PTS
			switch (dpcm->source_in) {
			case ENUM_AIN_AUDIO:
			case ENUM_AIN_AUDIO_V2:
			case ENUM_AIN_AUDIO_V3:
			case ENUM_AIN_AUDIO_V4:
			case ENUM_AIN_I2S_LOOPBACK:
			case ENUM_AIN_DMIC_PASSTHROUGH:
				break;
			default:
				snd_card_capture_calculate_pts(runtime, nPeriodCount);
				break;
			}
#endif

			// update LPCM_ring rp.
			dpcm->nLPCMRing_LE.readPtr[0] = (unsigned int)ring_add(
				(unsigned long)dpcm->nLPCMRing_LE.beginAddr
				, (unsigned long)(dpcm->nLPCMRing_LE.beginAddr + dpcm->nLPCMRing_LE.size)
				, (unsigned long)(dpcm->nLPCMRing_LE.readPtr[0])
				, nPeriodCount * runtime->period_size * dpcm->nFrameBytes);
			dpcm->nLPCMRing.readPtr[0] = htonl(dpcm->nLPCMRing_LE.readPtr[0]);

			dpcm->nTotalWrite += nPeriodCount * runtime->period_size;

			// update runtime->status->hw_ptr
			snd_pcm_period_elapsed(substream);
		}

SET_TIMER:
		/* Set up the next time */
		hrtimer_forward_now(timer, dpcm->ktime);

		return HRTIMER_RESTART;
	} else {
		return HRTIMER_NORESTART;
	}
}

static enum hrtimer_restart snd_card_timer_function(struct hrtimer *timer)
{
	struct snd_card_RTK_pcm *dpcm =
			container_of(timer, struct snd_card_RTK_pcm, hr_timer);
	struct snd_pcm_substream *substream = dpcm->substream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t nReadAddSize = 0;
	unsigned int nPeriodCount = 0;
	unsigned int HWRingRp;
	unsigned int HWRingFreeSize;
	unsigned int HWRingFreeFrame;
	int dec_out_valid_size = 0;

	if (dpcm->enHRTimer == HRTIMER_RESTART) {
		//calculate AOInring msec
		dec_out_valid_size = ring_valid_data(
			(unsigned int)ntohl(dpcm->decOutRing[0].beginAddr),
			(unsigned int)ntohl(dpcm->decOutRing[0].beginAddr) + rtk_dec_ao_buffer,
			(unsigned int)ntohl(dpcm->decOutRing[0].readPtr[0]),
			(unsigned int)ntohl(dpcm->decOutRing[0].writePtr));
		if (dec_out_valid_size > 0 && dec_out_valid_size <= dpcm->nRingSize)
			dec_out_msec = ((dec_out_valid_size >> 2) * 1000) / runtime->rate;
		else
			dec_out_msec = 0;

		if (runtime->control->appl_ptr == runtime->status->hw_ptr)
			dpcm->audio_count = dpcm->audio_count + 1;
		else
			dpcm->audio_count = 0;

		if (dpcm->audio_count >= (HZ << 1)) {
			pr_err("[appl_ptr %d = hw_ptr %s %d]\n", (int)runtime->control->appl_ptr, __func__, __LINE__);
			pr_err("Need to check why data didn't send to alsa\n");
			dpcm->audio_count = 0;
		}

		// update HW rp (the pointer of AFW read)
		HWRingRp = (unsigned int)(ntohl(dpcm->decInRing[0].readPtr[0]));//physical address
		dpcm->decInRing_LE[0].readPtr[0] = HWRingRp;
		dpcm->nHWPtr = bytes_to_frames(runtime, (unsigned long)HWRingRp - (unsigned long)dpcm->decInRing_LE[0].beginAddr);

		// update HW read size
		if (dpcm->nHWPtr != dpcm->nPreHWPtr) {
			nReadAddSize = ring_valid_data(0, runtime->buffer_size, dpcm->nPreHWPtr, dpcm->nHWPtr);

			/* Control the rp for application.
			 * 1. If data more than half buffer means the ability is enough,
			 *    we can update length of the half buffer to rp.
			 * 2. If data less than half buffer means the ability is not enough,
			 *    we just update one period size length to rp avoiding SW-3490 problem.
			 */
			if (nReadAddSize > (runtime->buffer_size >> 1)) {
				nReadAddSize = runtime->buffer_size >> 1;
				dpcm->nHWPtr = ring_add(0, runtime->buffer_size, dpcm->nPreHWPtr, nReadAddSize);
			} else if (nReadAddSize <= (runtime->buffer_size >> 1) && nReadAddSize >= runtime->period_size) {
				nReadAddSize = runtime->period_size;
				dpcm->nHWPtr = ring_add(0, runtime->buffer_size, dpcm->nPreHWPtr, nReadAddSize);
			}

			dpcm->nHWReadSize += nReadAddSize;
			dpcm->nTotalRead = ring_add(0,
				runtime->boundary,
				dpcm->nTotalRead,
				nReadAddSize);
		}

#ifdef DEBUG_RECORD
		spin_lock_irqsave(&playback_lock, flags);
		dpcm->fs = get_fs();
		set_fs(KERNEL_DS);
		if (dpcm->nHWPtr >= dpcm->nPreHWPtr) {
			vfs_write(dpcm->fp, runtime->dma_area + frames_to_bytes(runtime, dpcm->nPreHWPtr),
					frames_to_bytes(runtime, dpcm->nHWPtr - dpcm->nPreHWPtr), &dpcm->pos);
		} else {
			nReadAddSize = ring_valid_data(0, runtime->buffer_size, dpcm->nPreHWPtr, dpcm->nHWPtr);

			vfs_write(dpcm->fp, runtime->dma_area + frames_to_bytes(runtime, dpcm->nPreHWPtr),
					frames_to_bytes(runtime, runtime->buffer_size - dpcm->nPreHWPtr), &dpcm->pos);
			vfs_write(dpcm->fp, runtime->dma_area,
					frames_to_bytes(runtime, nReadAddSize - (runtime->buffer_size - dpcm->nPreHWPtr)), &dpcm->pos);
		}
		set_fs(dpcm->fs);
		spin_unlock_irqrestore(&playback_lock, flags);
#endif

		// update wp (the pointer application send data to alsa)
		nPeriodCount = ring_valid_data(0, runtime->boundary, dpcm->nTotalWrite, runtime->control->appl_ptr) / runtime->period_size;

		/* Accumulate two periods at first time */
		if (dpcm->bInitRing == 1) {
			if (nPeriodCount >= 2) {
				nPeriodCount = 2;
				dpcm->bInitRing = dpcm->bInitRing + 1;
			} else
				nPeriodCount = 0;
		} else {
			/* Transmit one period each time */
			if (nPeriodCount >= 1)
				nPeriodCount = 1;
		}

		// Check the buffer available size between alsa and AFW
		HWRingFreeSize = valid_free_size(dpcm->decInRing_LE[0].beginAddr,
			dpcm->decInRing_LE[0].beginAddr + dpcm->decInRing_LE[0].size,
			HWRingRp,
			dpcm->decInRing_LE[0].writePtr);

		HWRingFreeFrame = bytes_to_frames(runtime, HWRingFreeSize);

		if ((runtime->period_size * nPeriodCount) > HWRingFreeFrame)
			nPeriodCount = HWRingFreeFrame / runtime->period_size;

		if (HWRingFreeSize <= dpcm->nPeriodBytes)
			nPeriodCount = 0;

		if (nPeriodCount) {
			dpcm->decInRing_LE[0].writePtr = ring_add(dpcm->decInRing_LE[0].beginAddr
				, dpcm->decInRing_LE[0].beginAddr + dpcm->decInRing_LE[0].size
				, dpcm->decInRing_LE[0].writePtr
				, frames_to_bytes(runtime, runtime->period_size * nPeriodCount));

			dpcm->nTotalWrite = ring_add(0,
				runtime->boundary,
				dpcm->nTotalWrite,
				runtime->period_size * nPeriodCount);

			// update wp (tell AFW the write pointer from application is update)
			dpcm->decInRing[0].writePtr = htonl(dpcm->decInRing_LE[0].writePtr);//record physical address
		}

		if (runtime->status->state == SNDRV_PCM_STATE_DRAINING) {
			switch (dpcm->nEOSState) {
			case SND_REALTEK_EOS_STATE_NONE:
				if (RPC_TOAGENT_INBAND_EOS_SVC(dpcm) < 0)
					pr_err("[%s %d fail]\n", __func__, __LINE__);

				dpcm->nEOSState = SND_REALTEK_EOS_STATE_FINISH;
				break;
			case SND_REALTEK_EOS_STATE_FINISH:
				if (dpcm->nTotalWrite == dpcm->nTotalRead) {
					dpcm->nHWReadSize = 0;
					snd_pcm_period_elapsed(substream);
				}
				break;
			default:
				break;
			}
		} else {
			if (dpcm->nHWReadSize >= runtime->period_size) {
				dpcm->nHWReadSize %= runtime->period_size;
				snd_pcm_period_elapsed(substream);
			}
		}

		// check if wp and rp of android and AFW both stop
		if (dpcm->nHWPtr == dpcm->nPreHWPtr && runtime->control->appl_ptr == dpcm->nPre_appl_ptr)
			dpcm->dbg_count = dpcm->dbg_count + 1;
		else
			dpcm->dbg_count = 0;

		if (dpcm->dbg_count >= (HZ << 1)) {
			pr_err("[state %d]\n", (int)runtime->status->state);
			pr_err("[runtime->control->appl_ptr %d runtime->status->hw_ptr %d]\n",
						(int)runtime->control->appl_ptr, (int)runtime->status->hw_ptr);
			pr_err("[dpcm->nTotalWrite %d dpcm->nTotalRead %d dpcm->nHWPtr %d]\n",
						(int)dpcm->nTotalWrite, (int)dpcm->nTotalRead, (int)dpcm->nHWPtr);
			pr_err("[b %x l %x w %x r %x]\n",
				(unsigned int)(ntohl(dpcm->decInRing[0].beginAddr)),
				(unsigned int)((ntohl(dpcm->decInRing[0].beginAddr)) + dpcm->decInRing_LE[0].size),
				(unsigned int)(ntohl(dpcm->decInRing[0].writePtr)),
				(unsigned int)(ntohl(dpcm->decInRing[0].readPtr[0])));
			pr_err("[HWRingRp %x nPeriodCount %d runtime->periods %d]\n",
						HWRingRp, nPeriodCount, runtime->periods);
			pr_err("[snd_pcm_playback_avail %d runtime->control->avail_min %d]\n",
						(int)snd_pcm_playback_avail(runtime), (int)runtime->control->avail_min);
			dpcm->dbg_count = 0;
		}

		dpcm->nPreHWPtr = dpcm->nHWPtr;
		dpcm->nPre_appl_ptr = runtime->control->appl_ptr;

		/* Set up the next time */
		hrtimer_forward_now(timer, dpcm->ktime);

		return HRTIMER_RESTART;
	} else
		return HRTIMER_NORESTART;
}

static int snd_card_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
	int ret = 0;
	unsigned long flags;

	// error checking
	if (snd_pcm_capture_avail(runtime) > runtime->buffer_size
		|| snd_pcm_capture_hw_avail(runtime) > runtime->buffer_size) {
		pr_err("[ERROR BUG %d %s %s %d]\n", cmd, __FILE__, __func__, __LINE__);
		pr_err("[state %d appl_ptr %d hw_ptr %d %d]\n", runtime->status->state, (int)runtime->control->appl_ptr, (int)runtime->status->hw_ptr, (int)runtime->buffer_size);
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		pr_info("[ALSA trigger start appl_ptr %d hw_ptr %d]\n", (int)runtime->control->appl_ptr, (int)runtime->status->hw_ptr);
		dpcm->enHRTimer = HRTIMER_RESTART;
		hrtimer_start(&dpcm->hr_timer, dpcm->ktime, HRTIMER_MODE_REL);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		pr_info("[ALSA trigger stop appl_ptr %d hw_ptr %d]\n", (int)runtime->control->appl_ptr, (int)runtime->status->hw_ptr);
		spin_lock_irqsave(&capture_lock, flags);
		dpcm->enHRTimer = HRTIMER_NORESTART;
		spin_unlock_irqrestore(&capture_lock, flags);
		hrtimer_try_to_cancel(&dpcm->hr_timer);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int snd_card_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;
	int ret = 0;
	unsigned long flags;

	if (cmd != SNDRV_PCM_TRIGGER_SUSPEND && is_suspend)
		return 0;

	// error checking
	if (snd_pcm_playback_avail(runtime) > runtime->buffer_size
		|| snd_pcm_playback_hw_avail(runtime) > runtime->buffer_size) {
		pr_err("[ERROR BUG %s %s %d]\n", __FILE__, __func__, __LINE__);
		pr_err("[state %d appl_ptr %d hw_ptr %d]\n", runtime->status->state, (int)runtime->control->appl_ptr, (int)runtime->status->hw_ptr);
		pr_err("[dpcm->nTotalWrite %d dpcm->nTotalRead %d]\n", (int)dpcm->nTotalWrite, (int)dpcm->nTotalRead);
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_STOP:
		spin_lock_irqsave(&playback_lock, flags);
		dpcm->enHRTimer = HRTIMER_NORESTART;
		spin_unlock_irqrestore(&playback_lock, flags);
		hrtimer_try_to_cancel(&dpcm->hr_timer);
		break;

	case SNDRV_PCM_TRIGGER_START:
		dpcm->enHRTimer = HRTIMER_RESTART;
		hrtimer_start(&dpcm->hr_timer, dpcm->ktime, HRTIMER_MODE_REL);
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irqsave(&playback_lock, flags);
		dpcm->enHRTimer = HRTIMER_NORESTART;
		spin_unlock_irqrestore(&playback_lock, flags);
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dpcm->enHRTimer = HRTIMER_RESTART;
		hrtimer_start(&dpcm->hr_timer, dpcm->ktime, HRTIMER_MODE_REL);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
		spin_lock_irqsave(&playback_lock, flags);
		pr_err("[+]SNDRV_PCM_TRIGGER_SUSPEND\n");
		dpcm->enHRTimer = HRTIMER_NORESTART;
		pr_err("[-]SNDRV_PCM_TRIGGER_SUSPEND\n");
		spin_unlock_irqrestore(&playback_lock, flags);
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		pr_err("[+]SNDRV_PCM_TRIGGER_RESUME\n");
		dpcm->enHRTimer = HRTIMER_RESTART;
		hrtimer_start(&dpcm->hr_timer, dpcm->ktime, HRTIMER_MODE_REL);
		pr_err("[-]SNDRV_PCM_TRIGGER_RESUME\n");
		break;

	default:
		pr_err("[err %d %s %d]\n", cmd, __func__, __LINE__);
		ret = -EINVAL;
	}

	return ret;
}

static int snd_card_playback_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *area)
{
	struct dma_buf *buf;
	int ret = 0;

	buf = substream->runtime->dma_buffer_p->rtk_ion_rpc_dmabuf;
	ret = buf->ops->mmap(buf, area);

	return ret;
}

static int snd_card_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime;
	struct snd_dma_buffer *dmab = NULL;
	size_t buffer_size;

	pr_info("[ALSA %s demand %d Bytes]\n", __func__, params_buffer_bytes(hw_params));

	buffer_size = params_buffer_bytes(hw_params);

	if (PCM_SUBSTREAM_CHECK(substream))
		return -EINVAL;
	if (snd_BUG_ON(substream->dma_buffer.dev.type ==
		       SNDRV_DMA_TYPE_UNKNOWN))
		return -EINVAL;
	runtime = substream->runtime;

	if (runtime->dma_buffer_p) {
		if (runtime->dma_buffer_p->bytes >= buffer_size) {
			runtime->dma_bytes = buffer_size;
			return 0;
		}
		snd_pcm_lib_free_pages(substream);
	}

	if (substream->dma_buffer.area != NULL &&
	    substream->dma_buffer.bytes >= buffer_size) {
		dmab = &substream->dma_buffer;
	} else {
		dmab = kzalloc(sizeof(*dmab), GFP_KERNEL);
		if (! dmab)
			return -ENOMEM;
		dmab->dev = substream->dma_buffer.dev;

		/* Allocate buffer using ion */
		pr_info("[+]snd_dma_alloc_pages SNDRV_DMA_TYPE_ION size %d\n", buffer_size);
		dmab->rtk_ion_rpc_dmabuf = ion_alloc(buffer_size, RTK_ION_HEAP_AUDIO_MASK,
			ION_USAGE_MMAP_WRITECOMBINE |ION_FLAG_SCPUACC | ION_FLAG_ACPUACC);
		if (IS_ERR(dmab->rtk_ion_rpc_dmabuf)) {
			pr_err("[%s %d dma_buf_get fail]\n", __FUNCTION__, __LINE__);
			kfree(dmab);
			return PTR_ERR(dmab->rtk_ion_rpc_dmabuf);
		}

		dmab->addr = rtk_rpc_ion_pa(dmab->rtk_ion_rpc_dmabuf->priv);
		if (dmab->addr == -1) {
			pr_err("snd_dma_alloc_pages allocate ion audio heap buffer failed\n");
			kfree(dmab);
			return -ENXIO;
		}
		dmab->area = rtk_rpc_ion_va(dmab->rtk_ion_rpc_dmabuf->priv);
		pr_info("[-]snd_dma_alloc_pages phy %p vir %p\n", dmab->addr, dmab->area);
	}

	snd_pcm_set_runtime_buffer(substream, dmab);
	runtime->dma_bytes = buffer_size;

	return 1;
}

static int snd_card_capture_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	struct snd_dma_buffer *dmab = NULL;

	pr_info("ALSA snd_card_hw_free\n");

	if (PCM_SUBSTREAM_CHECK(substream))
		return -EINVAL;

	runtime = substream->runtime;
	if (runtime->dma_area == NULL)
		return 0;

	if (runtime->dma_buffer_p != &substream->dma_buffer) {
		/* Free buffer allocated by ion */
		pr_info("snd_dma_free_pages SNDRV_DMA_TYPE_ION\n");
		dmab = runtime->dma_buffer_p;
		if (!IS_ERR(dmab->rtk_ion_rpc_dmabuf)) {
			rtk_rpc_free_ion(dmab->rtk_ion_rpc_dmabuf);
			dmab->rtk_ion_rpc_dmabuf = NULL;
		}
		kfree(runtime->dma_buffer_p);
	}

	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}

static int snd_card_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	struct snd_dma_buffer *dmab = NULL;

	pr_info("ALSA snd_card_hw_free\n");

	if (PCM_SUBSTREAM_CHECK(substream))
		return -EINVAL;

	runtime = substream->runtime;
	if (runtime->dma_area == NULL)
		return 0;

	if (runtime->dma_buffer_p != &substream->dma_buffer) {
		/* Free buffer allocated by ion */
		pr_info("snd_dma_free_pages SNDRV_DMA_TYPE_ION\n");
		dmab = runtime->dma_buffer_p;
		if (!IS_ERR(dmab->rtk_ion_rpc_dmabuf)) {
			rtk_rpc_free_ion(dmab->rtk_ion_rpc_dmabuf);
			dmab->rtk_ion_rpc_dmabuf = NULL;
		}
		kfree(runtime->dma_buffer_p);
	}

	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}

static unsigned long ring_valid_data(unsigned long ring_base, unsigned long ring_limit, unsigned long ring_rp, unsigned long ring_wp)
{
	if (ring_wp >= ring_rp)
		return (ring_wp-ring_rp);
	else
		return (ring_limit-ring_base)-(ring_rp-ring_wp);
}

static unsigned long ring_add(unsigned long ring_base, unsigned long ring_limit, unsigned long ptr, unsigned int bytes)
{
	ptr += bytes;

	if (ptr >= ring_limit)
		ptr = ring_base + (ptr-ring_limit);

	return ptr;
}

static unsigned long ring_minus(unsigned long ring_base, unsigned long ring_limit, unsigned long ptr, int bytes)
{
	ptr -= bytes;

	if (ptr < ring_base)
		ptr = ring_limit-(ring_base-ptr);

	return ptr;
}

static unsigned long valid_free_size(unsigned long base, unsigned long limit, unsigned long rp, unsigned long wp)
{
	return (limit-base)-ring_valid_data(base, limit, rp, wp)-1;
}

static int ring_check_ptr_valid_32(unsigned int ring_rp, unsigned int ring_wp, unsigned int ptr)
{
	if (ring_wp >= ring_rp)
		return (ptr < ring_wp && ptr >= ring_rp);
	else
		return (ptr >= ring_rp || ptr < ring_wp);
}

static unsigned long buf_memcpy2_ring(unsigned long base, unsigned long limit, unsigned long ptr, char *buf, unsigned long size)
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

// create PCM instance
static int snd_card_create_PCM_instance(struct RTK_snd_card *pSnd,
					int instance_idx,
					int playback_substreams,
					int capture_substreams)
{
	struct snd_pcm *pcm = NULL;
	struct snd_pcm_substream *p = NULL;
	int err;
	int i;

	pr_info("%d %d @ %s %d\n", playback_substreams, capture_substreams, __func__, __LINE__);

	// create PCM instance
	err = snd_pcm_new(pSnd->card
		, snd_pcm_id[instance_idx] /* id string */
		, instance_idx
		, playback_substreams
		, capture_substreams
		, &pcm);
	if (err < 0) {
		pr_info("[%s %d fail]\n", __func__, __LINE__);
		return err;
	}

	// set OPs
	switch (instance_idx) {
	case 0:
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_card_rtk_playback_ops);
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_rtk_capture_ops);
		sprintf(pcm->name, SND_REALTEK_DRIVER_HDMI_IN);
		break;
	case 1:
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_card_rtk_playback_ops);
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_rtk_capture_ops);
		sprintf(pcm->name, SND_REALTEK_DRIVER_I2S_IN);
		break;
	case 2:
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_rtk_capture_ops);
		sprintf(pcm->name, SND_REALTEK_DRIVER_NONPCM_IN);
		break;
	case 3:
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_rtk_capture_ops);
		sprintf(pcm->name, SND_REALTEK_DRIVER_AUDIO_IN);
		break;
	case 4:
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_rtk_capture_ops);
		sprintf(pcm->name, SND_REALTEK_DRIVER_AUDIO_V2_IN);
		break;
	case 5:
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_rtk_capture_ops);
		sprintf(pcm->name, SND_REALTEK_DRIVER_AUDIO_V3_IN);
		break;
	case 6:
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_rtk_capture_ops);
		sprintf(pcm->name, SND_REALTEK_DRIVER_AUDIO_V4_IN);
		break;
	case 7:
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_rtk_capture_ops);
		sprintf(pcm->name, SND_REALTEK_DRIVER_I2S_LOOPBACK_IN);
		break;
	case 8:
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_rtk_capture_ops);
		sprintf(pcm->name, SND_REALTEK_DRIVER_DMIC_PASSTHROUGH_IN);
		break;
	case 9:
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_rtk_capture_ops);
		sprintf(pcm->name, SND_REALTEK_DRIVER_PURE_DMIC_IN);
		break;
	default:
		pr_info("[%s %d fail]\n", __func__, __LINE__);
		break;
	}

	pcm->private_data = pSnd;
	pcm->info_flags = 0;

	p = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	for (i = 0; i < playback_substreams; i++) {
		p->dma_buffer.dev.dev = snd_dma_continuous_data(GFP_KERNEL);
		p->dma_buffer.dev.type = SNDRV_DMA_TYPE_ION;
		p = p->next;
	}

	p = pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
	for (i = 0; i < capture_substreams; i++) {
		p->dma_buffer.dev.dev = snd_dma_continuous_data(GFP_KERNEL);
		p->dma_buffer.dev.type = SNDRV_DMA_TYPE_ION;
		p = p->next;
	}

	snd_pcm_add_chmap_ctls(pcm, SNDRV_PCM_STREAM_PLAYBACK, snd_pcm_alt_chmaps, 8, 0, NULL);
	pSnd->pcm = pcm;

	return 0;
}

void snd_realtek_hw_playback_volume_work(struct work_struct *work)
{
	struct RTK_snd_card *mars = container_of(work, struct RTK_snd_card, work_volume);

	pr_info("[%s %d]\n", __func__, __LINE__);

	RPC_TOAGENT_SET_VOLUME(mars->mixer_volume[MIXER_ADDR_MASTER][0]);
}

static int snd_card_mars_new_mixer(struct RTK_snd_card *pRTK_card)
{
	struct snd_card *card = pRTK_card->card;
	unsigned int idx;
	int err;

	spin_lock_init(&pRTK_card->mixer_lock);
	INIT_WORK(&pRTK_card->work_volume, snd_realtek_hw_playback_volume_work);

	strcpy(card->mixername, "RTK_Mixer");

	// add snc_control
	for (idx = 0; idx < ARRAY_SIZE(snd_mars_controls); idx++) {
		err = snd_ctl_add(card, snd_ctl_new1(&snd_mars_controls[idx], pRTK_card));
		if (err < 0) {
			pr_info("[snd_ctl_add faile %s %d]\n", __func__, __LINE__);
			return err;
		}
	}
	return 0;
}

static int snd_RTK_volume_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 31;
	return 0;
}

static int snd_RTK_volume_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct RTK_snd_card *mars = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int addr = kcontrol->private_value;

	spin_lock_irqsave(&mars->mixer_lock, flags);
	switch (addr) {
	case MIXER_ADDR_MASTER:
		mars->mixer_volume[addr][1] = mars->mixer_volume[addr][0];
		break;
	default:
		break;
	}
	ucontrol->value.integer.value[0] = mars->mixer_volume[addr][0];
	ucontrol->value.integer.value[1] = mars->mixer_volume[addr][1];
	spin_unlock_irqrestore(&mars->mixer_lock, flags);
	return 0;
}

#define cpy_func(dst, src, size, dmem_buf) memcpy(dst, src, size)

static void ring1_to_ring2_general_64(struct AUDIO_RINGBUF_PTR_64 *ring1, struct AUDIO_RINGBUF_PTR_64 *ring2, long size, char *dmem_buf)
{
	if (ring1->rp + size <= ring1->limit) {
		if (ring2->wp + size <= ring2->limit) {
			cpy_func((char *)ring2->wp, (char *)ring1->rp, size, dmem_buf);
		} else {
			int i = ring2->limit-ring2->wp;

			cpy_func((char *)ring2->wp, (char *)ring1->rp, i, dmem_buf);
			cpy_func((char *)ring2->base, (char *)(ring1->rp+i), size-i, dmem_buf);
		}
	} else {
		if (ring2->wp + size <= ring2->limit) {
			int i = ring1->limit-ring1->rp;

			cpy_func((char *)ring2->wp, (char *)ring1->rp, i, dmem_buf);
			cpy_func((char *)(ring2->wp+i), (char *)(ring1->base), size-i, dmem_buf);
		} else {
			int i, j;

			i = ring1->limit-ring1->rp;
			j = ring2->limit-ring2->wp;

			if (j <= i) {
				cpy_func((char *)ring2->wp, (char *)ring1->rp, j, dmem_buf);
				cpy_func((char *)ring2->base, (char *)(ring1->rp+j), i-j, dmem_buf);
				cpy_func((char *)(ring2->base+i-j), (char *)(ring1->base), size-i, dmem_buf);
			} else {
				cpy_func((char *)ring2->wp, (char *)ring1->rp, i, dmem_buf);
				cpy_func((char *)(ring2->wp+i), (char *)ring1->base, j-i, dmem_buf);
				cpy_func((char *)ring2->base, (char *)(ring1->base+j-i), size-j, dmem_buf);
			}
		}
	}

	ring1->rp += size;
	if (ring1->rp >= ring1->limit)
		ring1->rp = ring1->base + (ring1->rp-ring1->limit);

	ring2->wp += size;
	if (ring2->wp >= ring2->limit)
		ring2->wp = ring2->base + (ring2->wp-ring2->limit);
}

static int snd_RTK_volume_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct RTK_snd_card *mars = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int change, addr = kcontrol->private_value;
	int master;

	master = ucontrol->value.integer.value[0];
	if (master < 0)
		master = 0;

	if (master > 31)
		master = 31;

	spin_lock_irqsave(&mars->mixer_lock, flags);
	change = mars->mixer_volume[addr][0] != master;
	mars->mixer_volume[addr][0] = master;
	mars->mixer_volume[addr][1] = master;
	spin_unlock_irqrestore(&mars->mixer_lock, flags);

	if (addr == MIXER_ADDR_MASTER)
		schedule_work(&mars->work_volume);

	return change;
}

static int snd_RTK_capsrc_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;

	return 0;
}

static int snd_RTK_capsrc_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct RTK_snd_card *mars = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int addr = kcontrol->private_value;

	spin_lock_irqsave(&mars->mixer_lock, flags);
	ucontrol->value.integer.value[0] = mars->capture_source[addr][0];
	ucontrol->value.integer.value[1] = mars->capture_source[addr][1];
	spin_unlock_irqrestore(&mars->mixer_lock, flags);
	return 0;
}

static int snd_RTK_capsrc_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct RTK_snd_card *mars = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int change, addr = kcontrol->private_value;
	int left, right;

	left = ucontrol->value.integer.value[0] & 1;
	right = ucontrol->value.integer.value[1] & 1;
	spin_lock_irqsave(&mars->mixer_lock, flags);
	change = mars->capture_source[addr][0] != left &&
			mars->capture_source[addr][1] != right;
	mars->capture_source[addr][0] = left;
	mars->capture_source[addr][1] = right;
	spin_unlock_irqrestore(&mars->mixer_lock, flags);
	return change;
}

#ifdef CONFIG_SYSFS

static ssize_t alsa_latency_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int mLatency = 0;

	mLatency = snd_monitor_audio_data_queue();

	return sprintf(buf, "%u\n", mLatency);
}

static ssize_t alsa_raw_delay_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int mRawDelay = 0;

	mRawDelay = snd_monitor_raw_data_queue();

	return sprintf(buf, "%u\n", mRawDelay);
}

static ssize_t alsa_active_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", rtk_dec_ao_buffer);
}

static ssize_t alsa_active_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val >= 4096 && val <= (12*1024))
		rtk_dec_ao_buffer = val;
	else
		pr_err("set dec_ao_buffer failed! (size must be from 4k to 12k)\n");

	return count;
}

static struct kobj_attribute alsa_active_attr =
	__ATTR(dec_ao_buffer_size, 0644, alsa_active_show, alsa_active_store);

static struct kobj_attribute alsa_latency_attr =
	__ATTR(latency, 0444, alsa_latency_show, NULL);

static struct kobj_attribute alsa_raw_delay_attr =
	__ATTR(rawdelay, 0444, alsa_raw_delay_show, NULL);

static struct attribute *alsa_attrs[] = {
	&alsa_active_attr.attr,
	&alsa_latency_attr.attr,
	&alsa_raw_delay_attr.attr,
	NULL,
};

static struct attribute_group rtk_alsa_attr_group = {
	.attrs = alsa_attrs,
};

static struct kobject *alsa_kobj;

static int __init alsa_sysfs_init(void)
{
	int ret;

	alsa_kobj = kobject_create_and_add("rtk_alsa", kernel_kobj);
	if (!alsa_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(alsa_kobj, &rtk_alsa_attr_group);
	if (ret)
		kobject_put(alsa_kobj);

	return ret;
}
#endif

static int snd_card_probe(struct platform_device *devptr)
{
	struct snd_card *card;
	struct RTK_snd_card *pRTKSnd;
	int idx, err;
	struct device_node *np = devptr->dev.of_node;

	pr_info("[+] @ %s\n", __func__);

	if (WARN_ON(!np))
		dev_err(&devptr->dev, "can not found device node\n");

	// Get refclk by device node
	snd_refclk = of_refclk_get(np, 0);
	if (IS_ERR(snd_refclk)) {
		err = PTR_ERR(snd_refclk);
		if (err == -EPROBE_DEFER) {
			pr_info("snd_refclk not ready, retry\n");
			snd_refclk = NULL;
		} else {
			pr_info("failed to get snd_refclk: %d\n", err);
			snd_refclk = NULL;
		}
	}

	// Get virtual mapping for clk enable 2
	sys_clk_en2_virt = of_iomap(np, 0);

	// create sound card
	err = snd_card_new(
		&devptr->dev,
		-1, /* -1 ; idx == -1 == 0xffff means: take any free slot*/
		SND_REALTEK_DRIVER_HDMI_IN, /* describe ID string of sound card */
		THIS_MODULE,
		sizeof(struct RTK_snd_card), // size of private_data
		&card);
	if (err < 0)
		return err;

	// save the platform device
	devices[card->number] = devptr;

	// private data
	pRTKSnd = (struct RTK_snd_card *)card->private_data;
	pRTKSnd->card = card;

	for (idx = 0; idx < MAX_PCM_DEVICES; idx++)	{
		// create PCM instance
		err = snd_card_create_PCM_instance(pRTKSnd, idx
			, pcm_substreams[card->number], pcm_capture_substreams[card->number]);
		if (err < 0) {
			pr_info("[%s %d fail]\n", __func__, __LINE__);
			goto __nodev;
		}

		// create Compress instance
		err = snd_card_create_compress_instance(pRTKSnd, idx);
		if (err < 0) {
			pr_info("[%s %d fail]\n", __func__, __LINE__);
			goto __nodev;
		}
	}

	// create instance ONLY for capture AI I2S in
	snd_card_create_PCM_instance(pRTKSnd, 1, 3, 1);

	// create instance ONLY for capture AI nonpcm in
	snd_card_create_PCM_instance(pRTKSnd, 2, 0, 1);

	// create instance ONLY for capture AI audio in
	snd_card_create_PCM_instance(pRTKSnd, 3, 0, 1);

	// create instance ONLY for capture AI audio v2 in
	snd_card_create_PCM_instance(pRTKSnd, 4, 0, 1);

	// create instance ONLY for capture AI audio v3 in
	snd_card_create_PCM_instance(pRTKSnd, 5, 0, 1);

	// create instance ONLY for capture AI audio v4 in
	snd_card_create_PCM_instance(pRTKSnd, 6, 0, 1);

	// create instance ONLY for capture AI i2s loopback
	snd_card_create_PCM_instance(pRTKSnd, 7, 0, 1);

	// create instance ONLY for capture Dmic pass through
	snd_card_create_PCM_instance(pRTKSnd, 8, 0, 1);

	// create instance ONLY for capture AI Pure DMIC in
	snd_card_create_PCM_instance(pRTKSnd, 9, 0, 1);

	err = snd_card_mars_new_mixer(pRTKSnd);
	if (err < 0)
		goto __nodev;

	strcpy(card->driver, SND_REALTEK_DRIVER_HDMI_IN);
	strcpy(card->shortname, SND_REALTEK_DRIVER_HDMI_IN);
	sprintf(card->longname, SND_REALTEK_DRIVER_HDMI_IN);

	/* Init suspend variable */
	is_suspend = false;

	/* Init open ai and ao count */
	snd_open_ai_count = 0;
	snd_open_count = 0;

	err = snd_card_register(card);
	if (err == 0) {
		snd_RTK_cards[card->number] = card;
		platform_set_drvdata(devptr, card);
		pr_info("[-] @ %s %d\n", __func__, __LINE__);
		return 0;
	}

	// error handling
__nodev:
	pr_info("[%s %d fail]\n", __func__, __LINE__);
	snd_card_free(card);
	return err;
}

static int snd_card_remove(struct platform_device *devptr)
{
	struct snd_card *card = platform_get_drvdata(devptr);

	pr_info("[+] @ %s\n", __func__);
	if (card) {
		pr_info(" @ %s %d\n", __func__, __LINE__);
		snd_card_free(card);
		platform_set_drvdata(devptr, NULL);
	}

	pr_info("[-] @ %s\n", __func__);
	return 0;
}

#ifdef CONFIG_PM
static int rtk_alsa_suspend(struct device *pdev)
{
	struct snd_card *card = dev_get_drvdata(pdev);
	struct RTK_snd_card *pRTKSnd = card->private_data;

	pr_info("[+]%s %d\n", __func__, __LINE__);
	snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);

	is_suspend = true;
	snd_pcm_suspend_all(pRTKSnd->pcm);

	pr_info("[-]%s %d\n", __func__, __LINE__);
	return 0;
}

static int rtk_alsa_resume(struct device *pdev)
{
	struct snd_card *card = dev_get_drvdata(pdev);

	pr_info("[+]%s %d\n", __func__, __LINE__);
	snd_power_change_state(card, SNDRV_CTL_POWER_D0);
	is_suspend = false;

	pr_info("[-]%s %d\n", __func__, __LINE__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(rtk_alsa_pm, rtk_alsa_suspend, rtk_alsa_resume);
#define RTK_ALSA_PM_OPS (&rtk_alsa_pm)
#else
#define RTK_ALSA_PM_OPS NULL
#endif

static const struct of_device_id rtk_pcm_dt_match[] = {
	{ .compatible = "realtek,rtk-alsa-pcm" },
	{}
};

static struct platform_driver rtk_alsa_driver = {
	.probe =	snd_card_probe,
	.remove =	snd_card_remove,
	.driver = {
		.name =		SND_REALTEK_DRIVER_HDMI_IN,
		.owner =	THIS_MODULE,
		.pm =		RTK_ALSA_PM_OPS,
		.of_match_table = rtk_pcm_dt_match,
	},
};

static void rtk_alsa_unregister_all(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(devices); ++i)
		platform_device_unregister(devices[i]);

	platform_driver_unregister(&rtk_alsa_driver);
}

static int __init RTK_alsa_card_init(void)
{
	int ret, err;

	pr_info("[+] @ %s\n", __func__);
	err = platform_driver_register(&rtk_alsa_driver);
	if (err < 0)
		goto RETURN_ERR;

#ifdef CONFIG_SYSFS
	ret = alsa_sysfs_init();
	if (ret)
		pr_info("%s: unable to create sysfs entry\n", __func__);
#endif

	pr_info("[-] @ %s %d\n", __func__, __LINE__);
	return 0;

RETURN_ERR:
	pr_info("[-] @ %s %d error\n", __func__, __LINE__);
	return err;
}

static void __exit RTK_alsa_card_exit(void)
{
	pr_info("[+] @ %s\n", __func__);

	if (snd_refclk)
		refclk_put(snd_refclk);

	if (sys_clk_en2_virt) {
		iounmap(sys_clk_en2_virt);
		sys_clk_en2_virt  = NULL;
	}

	rtk_alsa_unregister_all();
	pr_info("[-] @ %s\n", __func__);
}
module_init(RTK_alsa_card_init);
module_exit(RTK_alsa_card_exit);

MODULE_LICENSE("GPL v2");
