// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2017-2020 Realtek Semiconductor Corp.
 */

#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */
#include <linux/syscalls.h> /* needed for the _IOW etc stuff used later */
#include <linux/mpage.h>
#include <linux/dcache.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/dma-buf.h>
#include <linux/fdtable.h>
#include <linux/ion.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>
#include <sound/asound.h>
#include <asm/cacheflush.h>
#include "snd-realtek.h"
#include <trace/events/rtk_rpc.h>
#include <rtk_rpc.h>
#include <soc/realtek/kernel-rpc.h>
#include <ion_rtk_alloc.h>
#define ion_alloc ext_rtk_ion_alloc

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

int RPC_TOAGENT_CHECK_AUDIO_READY(phys_addr_t paddr, void *vaddr)
{
	struct RPC_DEFAULT_INPUT_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_DEFAULT_INPUT_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_DEFAULT_INPUT_T));

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_CHECK_READY,
		CONVERT_FOR_AVCPU(dat), //rpc->info address
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->info))), //rpc->retval address
		&rpc->ret)) {
		pr_err("[ALSA %s RPC fail]\n", __func__);
		goto exit;
	}

	if (rpc->ret != S_OK) {
		pr_err("[ALSA %s RPC fail]\n", __func__);
		goto exit;
	}

	if (rpc->info == 0) {
		pr_err("[ALSA Audio is not ready]\n");
		goto exit;
	}

	// successful
	ret = 0;
	pr_info("[%s %s %d ] success\n", __FILE__, __func__, __LINE__);
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_CHECK_AUDIO_READY);

int RPC_TOAGENT_SEND_AUDIO_VERSION(phys_addr_t paddr, void *vaddr, enum OMX_AUDIO_VERSION version)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *cmd = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	uint32_t RPC_ret;
	int ret = -1;
	phys_addr_t dat;
	unsigned long offset;

	cmd = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)cmd + offset);

	memset(cmd, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	cmd->type = htonl(ENUM_PRIVATEINFO_OMX_AUDIO_VERSION);
	cmd->privateInfo[0] = htonl(version);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat), //cmd address
		CONVERT_FOR_AVCPU(dat + offset), //res address
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		goto exit;
	}

	ret = 0;
	pr_info("[%s %s %d ] success\n", __FILE__, __func__, __LINE__);
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_SEND_AUDIO_VERSION);

int RPC_TOAGENT_CREATE_PP_AGENT(phys_addr_t paddr, void *vaddr, int *ppId, int *pinId)
{
	struct RPC_CREATE_AO_AGENT_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_CREATE_AO_AGENT_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_CREATE_AO_AGENT_T));
	rpc->info.instanceID = htonl(0);
	rpc->info.type = htonl(AUDIO_PP_OUT);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_CREATE_AGENT,
		CONVERT_FOR_AVCPU(dat), //rpc->info address
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->info))),//rpc->retval address
		&rpc->ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (ntohl(rpc->retval.result) != S_OK || rpc->ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	*ppId = ntohl(rpc->retval.data);
	ret = RPC_TOAGENT_GET_GLOBAL_PP_PIN(paddr, vaddr, pinId);
	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_CREATE_PP_AGENT);

int RPC_TOAGENT_CREATE_AO_AGENT(phys_addr_t paddr, void *vaddr, int *aoId, int pinId)
{
	struct RPC_CREATE_AO_AGENT_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_CREATE_AO_AGENT_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_CREATE_AO_AGENT_T));
	rpc->info.instanceID = htonl(0);
	rpc->info.type = htonl(pinId);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_CREATE_AGENT,
		CONVERT_FOR_AVCPU(dat), //rpc->info address
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->info))),//rpc->retval address
		&rpc->ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (ntohl(rpc->retval.result) != S_OK || rpc->ret != S_OK) {
		pr_err("[ALSA %x %x %s %d RPC fail]\n", rpc->retval.result, rpc->ret, __func__, __LINE__);
		goto exit;
	}

	*aoId = ntohl(rpc->retval.data);
	ret = 0;
	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_CREATE_AO_AGENT);

int RPC_TOAGENT_PUT_SHARE_MEMORY_LATENCY(phys_addr_t paddr, void *vaddr,
			void *p, void *p2, int decID, int aoID, int type)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *cmd = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	int ret = 0;
	int magic_num = 2379;
	uint32_t RPC_ret;
	phys_addr_t dat;
	unsigned long offset;

	pr_info("[%s %d ion_alloc p1 %x p2 %x decID %d aoID %d type %d\n", __FUNCTION__, __LINE__,
			(uint32_t)((long)p), (uint32_t)((long)p2), decID, aoID, type);

	cmd = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)cmd + offset);
	memset(cmd, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));

	cmd->type = htonl(type);
	cmd->privateInfo[0] = (uint32_t)htonl((uint32_t)((long)p));
	cmd->privateInfo[1] = htonl(magic_num);
	cmd->privateInfo[2] = (uint32_t)htonl((uint32_t)((long)p2));
	cmd->privateInfo[3] = (uint32_t)htonl((uint32_t)(decID));
	cmd->privateInfo[4] = (uint32_t)htonl((uint32_t)(aoID));

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_PUT_SHARE_MEMORY_LATENCY);

int RPC_TOAGENT_PUT_SHARE_MEMORY(phys_addr_t paddr, void *vaddr, void *p, int type)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *cmd = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	int ret = 0;
	uint32_t RPC_ret;
	phys_addr_t dat;
	unsigned long offset;

	cmd = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)cmd + offset);

	memset(cmd, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	cmd->type = htonl(type);
	cmd->privateInfo[0] = (uint32_t)htonl((uint32_t)((long)p));

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_PUT_SHARE_MEMORY);

int RPC_TOAGENT_GET_AO_FLASH_PIN(phys_addr_t paddr, void *vaddr, int AOAgentID)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *cmd = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	uint32_t RPC_ret;
	int ret = -1;
	phys_addr_t dat;
	unsigned long offset;

	cmd = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)cmd + offset);
	memset(cmd, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	memset(res, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_RETURNVAL));
	cmd->instanceID = htonl(AOAgentID);
	cmd->type = htonl(ENUM_PRIVATEINFO_AUDIO_GET_FLASH_PIN);
	cmd->privateInfo[0] = 0xFF;
	cmd->privateInfo[1] = 0xFF;
	cmd->privateInfo[2] = 0xFF;
	cmd->privateInfo[3] = 0xFF;
	cmd->privateInfo[4] = 0xFF;
	cmd->privateInfo[5] = 0xFF;

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat), //cmd address
		CONVERT_FOR_AVCPU(dat + offset),//res address
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	ret = ntohl(res->privateInfo[0]);

	if (ret < FLASH_AUDIO_PIN_1 || ret > FLASH_AUDIO_PIN_3) {
		pr_err("[ALSA %s %d RPC %d fail]\n", __func__, __LINE__, ret);
		ret = -1;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_GET_AO_FLASH_PIN);

int RPC_TOAGENT_GET_GLOBAL_PP_PIN(phys_addr_t paddr, void *vaddr, int *pinId)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *cmd = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	uint32_t RPC_ret;
	int ret = -1;
	phys_addr_t dat;
	unsigned long offset;

	cmd = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)cmd + offset);

	memset(cmd, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	cmd->type = htonl(ENUM_PRIVATEINFO_AUDIO_GET_PP_FREE_PINID);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat), //cmd address
		CONVERT_FOR_AVCPU(dat + offset), //res address
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (res != NULL && res->privateInfo[0] == 0x44495050) {
		*pinId = ntohl(res->privateInfo[2]);
		ret = 0;
	} else {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		ret = -1;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_GET_GLOBAL_PP_PIN);

int RPC_TOAGENT_SET_AO_FLASH_VOLUME(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_pcm *dpcm)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *cmd = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	uint32_t rpc_ret;
	int ret = -1;
	phys_addr_t dat;
	unsigned long offset;

	cmd = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)cmd + offset);

	memset(cmd, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	cmd->instanceID = htonl(dpcm->AOAgentID);
	cmd->type = htonl(ENUM_PRIVATEINFO_AUDIO_CONTROL_FLASH_VOLUME);
	cmd->privateInfo[0] = htonl(dpcm->AOpinID);
	cmd->privateInfo[1] = htonl((31-dpcm->volume));

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&rpc_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[ALSA set AO_pin %d volume %d]\n", dpcm->AOpinID, dpcm->volume);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_SET_AO_FLASH_VOLUME);

int RPC_TOAGENT_CREATE_DECODER_AGENT(phys_addr_t paddr, void *vaddr, int *decID,
				     int *pinID)
{
	struct RPC_CREATE_PCM_DECODER_CTRL_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_CREATE_PCM_DECODER_CTRL_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_CREATE_PCM_DECODER_CTRL_T));
	rpc->instance.type = htonl(AUDIO_DECODER);
	rpc->instance.instanceID = htonl(-1);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_CREATE_AGENT,
		CONVERT_FOR_AVCPU(dat), //rpc->instance address
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->instance))), //rpc->res address
		&rpc->ret)) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc->ret != S_OK) {
		pr_err("[%s %d fail]\n", __func__, __LINE__);
		goto exit;
	}

	*decID = ntohl(rpc->res.data);
	*pinID = BASE_BS_IN;

	pr_info("[ALSA Create Decoder instance %d]\n", *decID/*dpcm->DECAgentID*/);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_CREATE_DECODER_AGENT);

/* data of AUDIO_RPC_RINGBUFFER_HEADER is "hose side" */
int RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC(phys_addr_t paddr, void *vaddr,
			struct AUDIO_RPC_RINGBUFFER_HEADER *header, int buffer_count)
{
	struct RPC_INITRINGBUFFER_HEADER_T *rpc = NULL;
	int ch;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_INITRINGBUFFER_HEADER_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_INITRINGBUFFER_HEADER_T));
	rpc->header.instanceID = htonl(header->instanceID);
	rpc->header.pinID = htonl(header->pinID);
	rpc->header.readIdx = htonl(header->readIdx);
	rpc->header.listSize = htonl(header->listSize);

	pr_info(" header instance ID %d\n", header->instanceID);
	pr_info(" header pinID       %d\n", header->pinID);
	pr_info(" header readIdx     %d\n", header->readIdx);
	pr_info(" header listSize    %d\n", header->listSize);

	for (ch = 0; ch < buffer_count; ch++)
		rpc->header.pRingBufferHeaderList[ch] = htonl((unsigned int) header->pRingBufferHeaderList[ch]);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_INIT_RINGBUF,
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->ret) + sizeof(rpc->res))), //rpc->header address
		CONVERT_FOR_AVCPU(dat), //rpc->ret address
		&rpc->res)) {
		pr_err("[%s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc->res != S_OK || ntohl(rpc->ret.result) != S_OK) {
		pr_err("[%s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_INITRINGBUFFER_HEADER_SVC);

int RPC_TOAGENT_CONNECT_SVC(phys_addr_t paddr, void *vaddr,
			struct AUDIO_RPC_CONNECTION *pconnection)
{
	struct RPC_CONNECTION_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_CONNECTION_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_CONNECTION_T));
	rpc->out.srcInstanceID = htonl(pconnection->srcInstanceID);
	rpc->out.srcPinID = htonl(pconnection->srcPinID);
	rpc->out.desInstanceID = htonl(pconnection->desInstanceID);
	rpc->out.desPinID = htonl(pconnection->desPinID);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_CONNECT,
		CONVERT_FOR_AVCPU(dat), //rpc->out address
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->out))), //rpc->ret
		&rpc->res)) {
		pr_err("[%s RPC fail %d]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc->res != S_OK || ntohl(rpc->ret.result) != S_OK) {
		pr_err("[%s RPC fail %d]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_CONNECT_SVC);

int RPC_TOAGENT_SWITCH_FOCUS(phys_addr_t paddr, void *vaddr, struct AUDIO_RPC_FOCUS *focus)
{
	struct AUDIO_RPC_FOCUS_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct AUDIO_RPC_FOCUS_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_RPC_FOCUS_T));
	rpc->instanceID = htonl(focus->instanceID);
	rpc->focusID = htonl(focus->focusID);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_SWITCH_FOCUS,
		CONVERT_FOR_AVCPU(dat), //rpc->out address
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->instanceID) + sizeof(rpc->focusID))), //rpc->ret
		&rpc->res)) {
		pr_err("[%s RPC fail %d]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc->res != S_OK || ntohl(rpc->ret.result) != S_OK) {
		pr_err("[%s RPC fail %d]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_SWITCH_FOCUS);

int RPC_TOAGENT_DAC_I2S_CONFIG(phys_addr_t paddr, void *vaddr,
			struct AUDIO_CONFIG_DAC_I2S *config)
{
	struct AUDIO_CONFIG_DAC_I2S_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;
	uint32_t *result;
	unsigned long offset;

	rpc = (struct AUDIO_CONFIG_DAC_I2S_T *)vaddr;
	dat = paddr;

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_CONFIG_DAC_I2S_T));
	result = (uint32_t *)((unsigned long)rpc + offset);

	memset(rpc, 0, sizeof(struct AUDIO_CONFIG_DAC_I2S_T));
	rpc->instanceID = htonl(config->instanceID);
	rpc->dacConfig.audioGeneralConfig.interface_en = htonl(config->dacConfig.audioGeneralConfig.interface_en);
	rpc->dacConfig.audioGeneralConfig.channel_out = htonl(config->dacConfig.audioGeneralConfig.channel_out);
	rpc->dacConfig.audioGeneralConfig.count_down_play_en = htonl(config->dacConfig.audioGeneralConfig.count_down_play_en);
	rpc->dacConfig.audioGeneralConfig.count_down_play_cyc = htonl(config->dacConfig.audioGeneralConfig.count_down_play_cyc);
	rpc->dacConfig.sampleInfo.sampling_rate = htonl(config->dacConfig.sampleInfo.sampling_rate);
	rpc->dacConfig.sampleInfo.PCM_bitnum = htonl(config->dacConfig.sampleInfo.PCM_bitnum);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_DAC_I2S_CONFIG,
		CONVERT_FOR_AVCPU(dat), //rpc->out address
		CONVERT_FOR_AVCPU(dat + offset), //rpc->ret
		&rpc->res)) {
		pr_err("[%s RPC fail %d]\n", __func__, __LINE__);
		goto exit;
	}

	if (ntohl(*result) != S_OK) {
		pr_err("[%s RPC fail %d]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_DAC_I2S_CONFIG);

int RPC_TOAGENT_DAC_SPDIF_CONFIG(phys_addr_t paddr, void *vaddr,
		struct AUDIO_CONFIG_DAC_SPDIF *config)
{
	struct AUDIO_CONFIG_DAC_SPDIF_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;
	uint32_t *result;
	unsigned long offset;

	rpc = (struct AUDIO_CONFIG_DAC_SPDIF_T *)vaddr;
	dat = paddr;

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_CONFIG_DAC_SPDIF_T));
	result = (uint32_t *)((unsigned long)rpc + offset);

	memset(rpc, 0, sizeof(struct AUDIO_CONFIG_DAC_SPDIF_T));
	rpc->instanceID = htonl(config->instanceID);
	rpc->spdifConfig.audioGeneralConfig.interface_en = htonl(config->spdifConfig.audioGeneralConfig.interface_en);
	rpc->spdifConfig.audioGeneralConfig.channel_out = htonl(config->spdifConfig.audioGeneralConfig.channel_out);
	rpc->spdifConfig.audioGeneralConfig.count_down_play_en = htonl(config->spdifConfig.audioGeneralConfig.count_down_play_en);
	rpc->spdifConfig.audioGeneralConfig.count_down_play_cyc = htonl(config->spdifConfig.audioGeneralConfig.count_down_play_cyc);
	rpc->spdifConfig.sampleInfo.sampling_rate = htonl(config->spdifConfig.sampleInfo.sampling_rate);
	rpc->spdifConfig.sampleInfo.PCM_bitnum = htonl(config->spdifConfig.sampleInfo.PCM_bitnum);
	rpc->spdifConfig.out_cs_info.non_pcm_valid = htonl(config->spdifConfig.out_cs_info.non_pcm_valid);
	rpc->spdifConfig.out_cs_info.non_pcm_format = htonl(config->spdifConfig.out_cs_info.non_pcm_format);
	rpc->spdifConfig.out_cs_info.audio_format = htonl(config->spdifConfig.out_cs_info.audio_format);
	rpc->spdifConfig.out_cs_info.spdif_consumer_use = htonl(config->spdifConfig.out_cs_info.spdif_consumer_use);
	rpc->spdifConfig.out_cs_info.copy_right = htonl(config->spdifConfig.out_cs_info.copy_right);
	rpc->spdifConfig.out_cs_info.pre_emphasis = htonl(config->spdifConfig.out_cs_info.pre_emphasis);
	rpc->spdifConfig.out_cs_info.stereo_channel = htonl(config->spdifConfig.out_cs_info.stereo_channel);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_DAC_SPDIF_CONFIG,
		CONVERT_FOR_AVCPU(dat), //rpc->out address
		CONVERT_FOR_AVCPU(dat + offset), //rpc->ret
		&rpc->res)) {
		pr_err("[%s RPC fail %d]\n", __func__, __LINE__);
		goto exit;
	}

	if (ntohl(*result) != S_OK) {
		pr_err("[%s RPC fail %d]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_DAC_SPDIF_CONFIG);

int RPC_TOAGENT_SETREFCLOCK(phys_addr_t paddr, void *vaddr,
			struct AUDIO_RPC_REFCLOCK *pClock)
{
	struct AUDIO_RPC_REFCLOCK_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct AUDIO_RPC_REFCLOCK_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_RPC_REFCLOCK_T));
	rpc->instanceID = htonl(pClock->instanceID);
	rpc->pRefClockID = htonl(pClock->pRefClockID);
	rpc->pRefClock = htonl(pClock->pRefClock);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_SETREFCLOCK,
		CONVERT_FOR_AVCPU(dat), //rpc->out address
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->instanceID) + sizeof(rpc->pRefClockID) + sizeof(rpc->pRefClock))), //rpc->ret
		&rpc->res)) {
		pr_err("[%s RPC fail %d]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc->res != S_OK || ntohl(rpc->ret.result) != S_OK) {
		pr_err("[%s RPC fail %d]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_SETREFCLOCK);

int RPC_TOAGENT_PAUSE_SVC(phys_addr_t paddr, void *vaddr, int instance_id)
{
	struct RPC_TOAGENT_PAUSE_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_TOAGENT_PAUSE_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_TOAGENT_PAUSE_T));
	rpc->inst_id = htonl(instance_id);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PAUSE,
		CONVERT_FOR_AVCPU(dat), //rpc->inst_id address
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->inst_id))),//rpc->retval address
		&rpc->res)) {
		pr_err("[%s %d RPC fail\n]", __func__, __LINE__);
		goto exit;
	}

	if (rpc->res != S_OK) {
		pr_err("[%s %d RPC fail\n]", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_PAUSE_SVC);

int RPC_TOAGENT_DESTROY_AI_FLOW_SVC(phys_addr_t paddr, void *vaddr, int instance_id)
{
	struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *rpc = NULL;
	int ret = -1;
	uint32_t res;
	phys_addr_t dat;

	rpc = (struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	rpc->type = htonl(ENUM_PRIVATEINFO_AIO_ALSA_DESTROY_AI_FLOW);
	rpc->instanceID = htonl(instance_id);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat), //rpc->inst_id address
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(*rpc))),//rpc->retval address
		&res)) {
		pr_err("[%s %d RPC fail\n]", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_DESTROY_AI_FLOW_SVC);

int RPC_TOAGENT_RUN_SVC(phys_addr_t paddr, void *vaddr, int instance_id)
{
	struct RPC_TOAGENT_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_TOAGENT_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_TOAGENT_T));
	rpc->inst_id = htonl(instance_id);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_RUN,
		CONVERT_FOR_AVCPU(dat), //rpc->inst_id address
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->inst_id))), //rpc->retval address
		&rpc->res)) {
		pr_err("[%s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc->res != S_OK || ntohl(rpc->retval.result) != S_OK) {
		pr_err("[%s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_RUN_SVC);

int RPC_TOAGENT_FLUSH_SVC(phys_addr_t paddr, void *vaddr,
			struct AUDIO_RPC_SENDIO *sendio)
{
	struct RPC_TOAGENT_FLASH_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_TOAGENT_FLASH_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_TOAGENT_FLASH_T));
	rpc->sendio.instanceID = htonl(sendio->instanceID);
	rpc->sendio.pinID = htonl(sendio->pinID);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_FLUSH,
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->retval) + sizeof(rpc->res))), //rpc->sendio address
		CONVERT_FOR_AVCPU(dat), //rpc->retval address
		&rpc->res)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc->res != S_OK || ntohl(rpc->retval.result) != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_FLUSH_SVC);

int RPC_TOAGENT_RELEASE_AO_FLASH_PIN(phys_addr_t paddr, void *vaddr,
			int AOAgentID, int AOpinID)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *cmd = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	uint32_t rpc_ret = 0;
	int ret = -1;
	phys_addr_t dat;
	unsigned long offset;

	cmd = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)cmd + offset);

	memset(cmd, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	cmd->instanceID = htonl(AOAgentID);
	cmd->type = htonl(ENUM_PRIVATEINFO_AUDIO_RELEASE_FLASH_PIN);
	cmd->privateInfo[0] = htonl(AOpinID);
	cmd->privateInfo[1] = 0xFF;
	cmd->privateInfo[2] = 0xFF;
	cmd->privateInfo[3] = 0xFF;
	cmd->privateInfo[4] = 0xFF;
	cmd->privateInfo[5] = 0xFF;

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat), //cmd address
		CONVERT_FOR_AVCPU(dat + offset), //res address
		&rpc_ret)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc_ret != S_OK) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_RELEASE_AO_FLASH_PIN);

int RPC_TOAGENT_AO_CONFIG_WITHOUT_DECODER(phys_addr_t paddr, void *vaddr,
			struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_pcm *dpcm = runtime->private_data;
	struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *cmd = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	int ret = -1;
	int offset, tmp, ch;
	char *p;
	uint32_t RPC_ret;
	phys_addr_t dat;

	cmd = (struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));

	p = (char *)&cmd->argateInfo[3];
	memset(cmd, 0, sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)cmd + offset);

	cmd->type = htonl(ENUM_PRIVATEINFO_AIO_AO_FLASH_LPCM);
	cmd->instanceID = htonl(dpcm->AOAgentID);

	tmp = dpcm->AOpinID & 0xff;
	cmd->argateInfo[1] |= tmp;
	tmp = ((runtime->sample_bits >> 3) << 8);
	cmd->argateInfo[1] |= tmp;
	tmp = AUDIO_LITTLE_ENDIAN << 16;
	cmd->argateInfo[1] |= tmp;

	// disable ADFS
	if (dpcm->ao_paramter != NULL) {
		tmp = dpcm->ao_paramter->disable_adfs << 24;
		cmd->argateInfo[1] |= tmp;
	}

	cmd->argateInfo[1] = htonl(cmd->argateInfo[1]);

	cmd->argateInfo[2] = htonl(runtime->rate);
	for (ch = 0; ch < runtime->channels; ++ch)
		p[ch] = ch + 1;

	// config ao lpcm out delay and ao hw buffer delay
	if (dpcm->ao_paramter != NULL) {
		cmd->argateInfo[5] = (dpcm->ao_paramter->lpcm_out_delay << 16);
		cmd->argateInfo[5] |= dpcm->ao_paramter->hw_buffer_delay;
		cmd->argateInfo[5] = htonl(cmd->argateInfo[5]);
	} else {
		// Default value for buffer size
		cmd->argateInfo[5] = (15 << 16);
		cmd->argateInfo[5] |= 25;
		cmd->argateInfo[5] = htonl(cmd->argateInfo[5]);
	}

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[%s %d RPC fail\n]", __FUNCTION__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __FUNCTION__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __FUNCTION__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_AO_CONFIG_WITHOUT_DECODER);

int RPC_TOAGENT_STOP_SVC(phys_addr_t paddr, void *vaddr, int instanceID)
{
	struct RPC_TOAGENT_STOP_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_TOAGENT_STOP_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_TOAGENT_STOP_T));
	rpc->instanceID = htonl(instanceID);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_STOP,
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->retval) + sizeof(rpc->res))), //rpc->instanceID address
		CONVERT_FOR_AVCPU(dat),//rpc->retval address
		&rpc->res)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc->res != S_OK || ntohl(rpc->retval.result) != S_OK) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_STOP_SVC);

int RPC_TOAGENT_PP_INIT_PIN_SVC(phys_addr_t paddr, void *vaddr, int instanceID)
{
	struct RPC_TOAGENT_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_TOAGENT_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_TOAGENT_T));
	rpc->inst_id = htonl(instanceID);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_VIDEO_KERNEL_RPC_PP_INIT_PIN,
		CONVERT_FOR_AVCPU(dat), //rpc->inst_id address
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->inst_id))), //rpc->retval address
		&rpc->res)) {
		pr_err("[%s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc->res != S_OK || ntohl(rpc->retval.result) != S_OK) {
		pr_err("[%s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_PP_INIT_PIN_SVC);

int RPC_TOAGENT_DESTROY_SVC(phys_addr_t paddr, void *vaddr, int instanceID)
{
	struct RPC_TOAGENT_DESTROY_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_TOAGENT_DESTROY_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_TOAGENT_DESTROY_T));
	rpc->instanceID = htonl(instanceID);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_DESTROY,
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->retval) + sizeof(rpc->res))), //rpc->instanceID address
		CONVERT_FOR_AVCPU(dat), //rpc->retval address
		&rpc->res)) {
		pr_err("%s %d RPC fail\n", __FILE__, __LINE__);
		goto exit;
	}

	if (rpc->res != S_OK || ntohl(rpc->retval.result) != S_OK) {
		pr_err("%s %d RPC fail\n", __FILE__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_DESTROY_SVC);

int RPC_TOAGENT_INBAND_EOS_SVC(struct snd_card_RTK_pcm *dpcm)
{
	struct AUDIO_DEC_EOS cmd;

	cmd.header.type = htonl(AUDIO_DEC_INBAND_CMD_TYPE_EOS);
	cmd.header.size = htonl(sizeof(struct AUDIO_DEC_EOS));
	cmd.EOSID = 0;
	cmd.wPtr = htonl(dpcm->decInRing_LE[0].writePtr);
#ifdef CONFIG_64BIT
	writeInbandCmd(dpcm, &cmd, sizeof(struct AUDIO_DEC_EOS));
#else
	snd_realtek_hw_ring_write(&dpcm->decInbandRing, &cmd, sizeof(struct AUDIO_DEC_EOS), (unsigned int)dpcm - (unsigned int)dpcm->phy_addr);
#endif
	return 0;
}
EXPORT_SYMBOL(RPC_TOAGENT_INBAND_EOS_SVC);

/* set TRUEHD_ERR_SELF_RESET_ENABLED */
int RPC_TOAGENT_SET_TRUEHD_ERR_SELF_RESET(phys_addr_t paddr, void *vaddr, bool isON)
{
	struct AUDIO_CONFIG_COMMAND *config = NULL;
	unsigned int *res;
	uint32_t ret = 0;
	phys_addr_t dat;
	unsigned long offset;

	config = (struct AUDIO_CONFIG_COMMAND *)vaddr;
	dat = paddr;

	memset(config, 0, sizeof(struct AUDIO_CONFIG_COMMAND));
	config->msgID = htonl(AUDIO_CONFIG_TRUEHD_ERR_SELF_RESET_ENABLED);
	config->value[0] = htonl(isON);

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_CONFIG_COMMAND));
	res = (unsigned int *)((unsigned long)config + offset);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_AUDIO_CONFIG,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&ret)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		ret = -1;
		goto exit;
	}

	if (ret != S_OK) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		ret = -1;
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_SET_TRUEHD_ERR_SELF_RESET);

/* set AO volume */
int RPC_TOAGENT_SET_VOLUME(int volume)
{
	struct AUDIO_CONFIG_COMMAND *config = NULL;
	unsigned int *res;
	uint32_t ret = 0;
	phys_addr_t dat;
	unsigned long offset;
	struct dma_buf *rpc_dmabuf = NULL;

	rpc_dmabuf = ion_alloc(4096, RTK_ION_HEAP_AUDIO_MASK, AUDIO_ION_FLAG);
	if (IS_ERR(rpc_dmabuf)) {
			pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
			ret = PTR_ERR(rpc_dmabuf);
			goto exit;
	}

	dat = rtk_rpc_ion_pa(rpc_dmabuf->priv);
	if (dat == -1) {
			pr_err("[ALSA malloc fail %s]\n", __func__);
			goto exit;
	}

	config = rtk_rpc_ion_va(rpc_dmabuf->priv);

	memset(config, 0, sizeof(struct AUDIO_CONFIG_COMMAND));
	config->msgID = htonl(AUDIO_CONFIG_CMD_VOLUME);
	config->value[0] = htonl(31 - volume);

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_CONFIG_COMMAND));
	res = (unsigned int *)((unsigned long)config + offset);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_VOLUME_CONTROL,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&ret)) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		ret = -1;
		goto exit;
	}

	if (ret != S_OK) {
		pr_err("[ALSA %s %d fail]\n", __func__, __LINE__);
		ret = -1;
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:
	if (!IS_ERR(rpc_dmabuf))
		rtk_rpc_free_ion(rpc_dmabuf);

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_SET_VOLUME);

/* get AI ID */
int RPC_TOAGENT_GET_AI_AGENT(phys_addr_t paddr, void *vaddr,
		struct snd_card_RTK_capture_pcm *dpcm)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *in = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *out;
	int offset;
	uint32_t ret = 0;
	phys_addr_t dat;

	in = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	memset(in, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	out = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)in + offset);
	in->type = htonl(ENUM_PRIVATEINFO_AIO_GET_AUDIO_PROCESSING_AI);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&ret)) {
		pr_err("[fail %s %d]\n", __func__, __LINE__);
		goto exit;
	}

	dpcm->AIAgentID = ntohl(out->privateInfo[0]);
	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_GET_AI_AGENT);

/* get AO volume */
int RPC_TOAGENT_GET_VOLUME(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_pcm *mars)
{
	int volume = 0;
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *pArg = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *pRet;
	int offset;
	uint32_t rc;
	phys_addr_t dat;

	pArg = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	memset(pArg, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	pRet = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)pArg + offset);
	pArg->type = htonl(ENUM_PRIVATEINFO_AUDIO_GET_MUTE_N_VOLUME);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_GET_MUTE_N_VOLUME,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&rc)) {
		pr_err("[fail %s %d]\n", __func__, __LINE__);
		volume = -1;
		goto exit;
	}

	volume = ntohl(pRet->privateInfo[1]);
	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
exit:

	return volume;
}
EXPORT_SYMBOL(RPC_TOAGENT_GET_VOLUME);

int RPC_TOAGENT_AI_CONFIG_HDMI_RX_IN(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_capture_pcm *dpcm)
{
	struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *rpc = NULL;
	uint32_t RPC_ret;
	unsigned int offset;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	rpc->instanceID = htonl(dpcm->AIAgentID);
	rpc->type = htonl(ENUM_PRIVATEINFO_AIO_AI_PRIVATEINFO);
	rpc->argateInfo[0] = htonl(ENUM_AI_PRIVATE_HDMI_RX);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_AI_CONFIG_HDMI_RX_IN);

int RPC_TOAGENT_AI_CONFIG_I2S_IN(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_capture_pcm *dpcm)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *rpc = NULL;
	uint32_t RPC_ret;
	unsigned int offset;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	rpc->instanceID = htonl(dpcm->AIAgentID);
	rpc->type = htonl(ENUM_PRIVATEINFO_AUDIO_AI_PAD_IN);
	rpc->privateInfo[0] = htonl(48000);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_AI_CONFIG_I2S_IN);

int RPC_TOAGENT_AI_CONFIG_AUDIO_IN(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_capture_pcm *dpcm)
{
	struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *rpc = NULL;
	uint32_t RPC_ret;
	unsigned int offset;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	rpc->instanceID = htonl(dpcm->AIAgentID);
	rpc->type = htonl(ENUM_PRIVATEINFO_AIO_AI_PRIVATEINFO);

	if (dpcm->source_in == ENUM_AIN_AUDIO_V2)
		rpc->argateInfo[0] = htonl(ENUM_AI_PRIVATE_DUAL_DMIC_AND_LOOPBACK);
	else if (dpcm->source_in == ENUM_AIN_AUDIO_V3) {
		rpc->argateInfo[0] = htonl(ENUM_AI_PRIVATE_SPEECH_RECOGNITION_FROM_DMIC);
		rpc->argateInfo[1] = htonl(ENUM_AIN_AUDIO_PROCESSING_DMIC);
	} else if (dpcm->source_in == ENUM_AIN_AUDIO_V4) {
		rpc->argateInfo[0] = htonl(ENUM_AI_PRIVATE_SPEECH_RECOGNITION_FROM_DMIC);
		rpc->argateInfo[1] = htonl(ENUM_AIN_AUDIO_PROCESSING_I2S);
	}

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_AI_CONFIG_AUDIO_IN);

int RPC_TOAGENT_AI_CONFIG_I2S_LOOPBACK_IN(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_capture_pcm *dpcm)
{
	struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *rpc = NULL;
	uint32_t RPC_ret;
	unsigned int offset;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	rpc->instanceID = htonl(dpcm->AIAgentID);
	rpc->type = htonl(ENUM_PRIVATEINFO_AIO_AI_LOOPBACK_AO);
	rpc->argateInfo[0] |= (1 << ENUM_RPC_AI_LOOPBACK_FROM_AO_I2S);
	rpc->argateInfo[0] = htonl(rpc->argateInfo[0]);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_AI_CONFIG_I2S_LOOPBACK_IN);

int RPC_TOAGENT_AI_CONFIG_DMIC_PASSTHROUGH_IN(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_capture_pcm *dpcm)
{
	struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *rpc = NULL;
	uint32_t RPC_ret;
	unsigned int offset;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	rpc->instanceID = htonl(dpcm->AIAgentID);
	rpc->type = htonl(ENUM_PRIVATEINFO_AIO_AI_PRIVATEINFO);
	rpc->argateInfo[0] = htonl(ENUM_AI_PRIVATE_ADC_DMIC);
	rpc->argateInfo[1] = htonl(16000);
	rpc->argateInfo[4] = htonl(0x303); /* Enlarge the volume */

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_AI_CONFIG_DMIC_PASSTHROUGH_IN);

int RPC_TOAGENT_CREATE_GLOBAL_AO(phys_addr_t paddr, void *vaddr, int *aoId)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *rpc = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	uint32_t RPC_ret;
	unsigned int offset;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)rpc + offset);
	rpc->type = htonl(ENUM_PRIVATEINFO_AUDIO_GET_GLOBAL_AO_INSTANCEID);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	*aoId = ntohl(res->privateInfo[0]);
	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_CREATE_GLOBAL_AO);

int RPC_TOAGENT_SET_AI_FLASH_VOLUME(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_capture_pcm *dpcm, unsigned int volume)
{
	struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *rpc = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	uint32_t rpc_ret;
	unsigned int offset;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)rpc + offset);
	rpc->instanceID = htonl(dpcm->AIAgentID);
	rpc->type = htonl(ENUM_PRIVATEINFO_AIO_AI_PRIVATEINFO);
	rpc->argateInfo[0] = htonl(ENUM_AI_PRIVATE_ADC_SET_VOLUME);
	rpc->argateInfo[1] = htonl(volume); // ADC left channel digital volume in 0.5 dB step, -33.5dB~30dB
	rpc->argateInfo[2] = htonl(volume); // ADC right channel digital volume in 0.5 dB step, -33.5dB~30dB

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&rpc_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_SET_AI_FLASH_VOLUME);

int RPC_TOAGENT_SET_SOFTWARE_AI_FLASH_VOLUME(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_capture_pcm *dpcm, unsigned int volume)
{
	struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *rpc = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	uint32_t rpc_ret;
	unsigned int offset;
	int ret = -1, rpc_volume;
	phys_addr_t dat;

	/* The range of software control volume index is from 0 to 32. */
	if (volume >= 0 && volume < 17)
		rpc_volume = ENUM_AUDIO_VOLUME_CTRL_0_DB + volume;
	else if (volume >= 17 && volume < 33)
		rpc_volume = ENUM_AUDIO_VOLUME_CTRL_P1_DB + volume % 17;
	else
		goto exit;

	rpc = (struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_AIO_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)rpc + offset);
	rpc->instanceID = htonl(dpcm->AIAgentID);
	rpc->type = htonl(ENUM_PRIVATEINFO_AIO_AI_PRIVATEINFO);
	rpc->argateInfo[0] = htonl(ENUM_AI_PRIVATE_VOLUME_CTRL);
	rpc->argateInfo[1] = htonl(rpc_volume); // one of AUDIO_VOLUME_CTRL

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_AIO_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&rpc_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_SET_SOFTWARE_AI_FLASH_VOLUME);

int RPC_TOAGENT_SET_EQ(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_pcm *dpcm,
			struct AUDIO_RPC_EQUALIZER_MODE *equalizer_mode)
{
	struct AUDIO_EQUALIZER_CONFIG *rpc = NULL;
	uint32_t rpc_ret;
	unsigned int offset;
	int i, ret = -1;
	phys_addr_t dat;

	rpc = (struct AUDIO_EQUALIZER_CONFIG *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_EQUALIZER_CONFIG));
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_EQUALIZER_CONFIG));
	rpc->instanceID = htonl(dpcm->AOAgentID);
	rpc->gbl_var_eq_ID = htonl(ENUM_EQUALIZER_AO);
	rpc->ena = 0x1;

	/* Set up EQ data */
	rpc->app_eq_config.mode = htonl(equalizer_mode->mode);
	for (i = 0; i < 10; i++)
		rpc->app_eq_config.gain[i] = htonl(equalizer_mode->gain[i]);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_EQ_CONFIG,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&rpc_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (rpc_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_SET_EQ);

int RPC_TOAGENT_AI_CONFIG_NONPCM_IN(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_capture_pcm *dpcm)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *rpc = NULL;
	uint32_t RPC_ret;
	unsigned int offset;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	rpc->instanceID = htonl(dpcm->AIAgentID);
	rpc->type = htonl(ENUM_PRIVATEINFO_AUDIO_AI_NON_PCM_IN);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_AI_CONFIG_NONPCM_IN);

int RPC_TOAGENT_CREATE_AI_AGENT(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_capture_pcm *dpcm)
{
	struct RPC_CREATE_AO_AGENT_T *rpc = NULL;
	int ret = -1;
	phys_addr_t dat;

	rpc = (struct RPC_CREATE_AO_AGENT_T *)vaddr;
	dat = paddr;

	memset(rpc, 0, sizeof(struct RPC_CREATE_AO_AGENT_T));
	rpc->info.instanceID = htonl(-1);
	rpc->info.type = htonl(AUDIO_IN);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_CREATE_AGENT,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(rpc->info))),
		&rpc->ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (ntohl(rpc->retval.result) != S_OK || rpc->ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	dpcm->AIAgentID = ntohl(rpc->retval.data);
	pr_info("[%s %s %d] [ALSA Create AI instance %d]\n", __FILE__, __func__, __LINE__, dpcm->AIAgentID);
	// success
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_CREATE_AI_AGENT);

int RPC_TOAGENT_AI_DISCONNECT_ALSA_AUDIO(phys_addr_t paddr, void *vaddr,
			struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
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

	memset(cmd, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	cmd->instanceID = htonl(dpcm->AIAgentID);
	cmd->type = htonl(ENUM_PRIVATEINFO_AUDIO_AI_CONNECT_ALSA);
	cmd->privateInfo[0] = htonl(AUDIO_ALSA_FORMAT_NONE);
	cmd->privateInfo[1] = htonl(runtime->rate);

	if (dpcm->source_in == ENUM_AIN_AUDIO)
		cmd->privateInfo[2] = htonl(1);
	else
		cmd->privateInfo[2] = htonl(0);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_AI_DISCONNECT_ALSA_AUDIO);

int RPC_TOAGENT_AI_CONNECT_ALSA(phys_addr_t paddr, void *vaddr,
			struct snd_pcm_runtime *runtime)
{
	struct snd_card_RTK_capture_pcm *dpcm = runtime->private_data;
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

	memset(cmd, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	cmd->instanceID = htonl(dpcm->AIAgentID);
	cmd->type = htonl(ENUM_PRIVATEINFO_AUDIO_AI_CONNECT_ALSA);
	cmd->privateInfo[0] = htonl(dpcm->nAIFormat);
	cmd->privateInfo[1] = htonl(runtime->rate);

	switch (dpcm->source_in) {
	case ENUM_AIN_AUDIO:
		cmd->privateInfo[2] = htonl(1);
		break;
	default:
		cmd->privateInfo[2] = htonl(0);
		break;
	}

	switch (dpcm->source_in) {
	case ENUM_AIN_AUDIO_V2:
	case ENUM_AIN_AUDIO_V3:
		cmd->privateInfo[3] = htonl(1); //1 channel
		break;
	default:
		cmd->privateInfo[3] = htonl(0); //channels
		break;
	}

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_AI_CONNECT_ALSA);

int RPC_TOAGENT_AI_CONNECT_AO(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_capture_pcm *dpcm)
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

	memset(cmd, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	cmd->instanceID = htonl(dpcm->AIAgentID);
	cmd->type = htonl(ENUM_PRIVATEINFO_AUDIO_AI_SET_AO_FLASH_PIN);
	cmd->privateInfo[0] = htonl(dpcm->AOpinID);

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_AI_CONNECT_AO);

int RPC_TOAGENT_SET_LOW_WATER_LEVEL(phys_addr_t paddr, void *vaddr, bool isLowWater)
{
	struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *cmd = NULL;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	uint32_t RPC_ret;
	int ret = -1;
	phys_addr_t dat;
	unsigned long offset;

	cmd = (struct AUDIO_RPC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)cmd + offset);
	memset(cmd, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_PARAMETERS));
	memset(res, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_RETURNVAL));
	cmd->type = htonl(ENUM_PRIVATEINFO_AUDIO_SET_LOW_WATERLEVEL);
	if (isLowWater) {
		cmd->privateInfo[0] = htonl(0x1);
		pr_info("Enable pp ao low waterlevel mode\n");
	} else {
		cmd->privateInfo[0] = 0;
		pr_info("Disable pp ao low waterlevel mode\n");
	}
	cmd->privateInfo[1] = 0;
	cmd->privateInfo[2] = 0;
	cmd->privateInfo[3] = 0;

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat), //cmd address
		CONVERT_FOR_AVCPU(dat + offset),//res address
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __func__, __LINE__);
		goto exit;
	}

	// successful
	pr_info("[%s %s %d] success\n", __FILE__, __func__, __LINE__);
	ret = 0;
exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_SET_LOW_WATER_LEVEL);

int RPC_TOAGENT_SET_MAX_LATENCY(phys_addr_t paddr, void *vaddr,
			struct snd_card_RTK_pcm *dpcm)
{
	struct AUDIO_RPC_DEC_PRIVATEINFO_PARAMETERS *cmd;
	struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *res;
	uint32_t RPC_ret;
	int ret = 0;
	phys_addr_t dat;
	unsigned long offset;

	cmd = (struct AUDIO_RPC_DEC_PRIVATEINFO_PARAMETERS *)vaddr;
	dat = paddr;

	offset = get_rpc_alignment_offset(sizeof(struct AUDIO_RPC_DEC_PRIVATEINFO_PARAMETERS));
	res = (struct AUDIO_RPC_PRIVATEINFO_RETURNVAL *)((unsigned long)cmd + offset);
	memset(cmd, 0, sizeof(struct AUDIO_RPC_DEC_PRIVATEINFO_PARAMETERS));
	memset(res, 0, sizeof(struct AUDIO_RPC_PRIVATEINFO_RETURNVAL));
	cmd->instanceID = htonl(dpcm->DECAgentID);
	cmd->type = htonl(ENUM_PRIVATEINFO_DEC_ALSA_CONFIG);
	cmd->privateInfo[0] = htonl(11); /* max latency of dec out(ms) */
	cmd->privateInfo[1] = htonl(30); /* max latency of ao out(ms) */

	if (send_rpc_command(RPC_AUDIO,
		ENUM_KERNEL_RPC_DEC_PRIVATEINFO,
		CONVERT_FOR_AVCPU(dat), //cmd address
		CONVERT_FOR_AVCPU(dat + offset),//res address
		&RPC_ret)) {
		pr_err("[ALSA %s %d RPC fail]\n", __FUNCTION__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("[ALSA %s %d RPC fail]\n", __FUNCTION__, __LINE__);
		goto exit;
	}

exit:

	return ret;
}
EXPORT_SYMBOL(RPC_TOAGENT_SET_MAX_LATENCY);

MODULE_LICENSE("GPL v2");
