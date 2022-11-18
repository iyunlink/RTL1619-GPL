/*
 * RealTek HDCP 2.2 CA driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/uuid.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <asm/io.h>
#include <linux/slab.h>

#include "rtk_hdcp2_tee.h"

#define TEEC_SUCCESS				0x0
#define TEE_IOCTL_UUID_LEN			16
#define HDCP_2_2_CERT_RX_LEN			(HDCP_2_2_RECEIVER_ID_LEN +	\
						 HDCP_2_2_K_PUB_RX_LEN + 2 +	\
						 HDCP_2_2_DCP_LLC_SIG_LEN)

#define HDCP_2_2_RRX_RTX_RXCAPS_TXCAPS_LEN	(HDCP_2_2_RRX_LEN +	\
						 HDCP_2_2_RTX_LEN +	\
						 HDCP_2_2_RXCAPS_LEN +	\
						 1 + HDCP_2_2_TXCAP_MASK_LEN)

static const uuid_t ta_hdcptx2_uuid = UUID_INIT(0x8baaf200, 0x2450, 0x11e4,
						0xab, 0xe2, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b);


enum HDCP22_CMD_FOR_TA
{
	TA_TEE_OpenKeybox               = 0x1,
	TA_TEE_IsKeyboxValid            = 0x2,
	TA_TEE_GetKeyData               = 0x3,
	TA_TEE_GetDeviceID              = 0x4,
	TA_TEE_RandomData               = 0x5,
	TA_TEE_SetIVBuffer              = 0x6,
	TA_TEE_SetEntitlementKey        = 0x7,
	TA_TEE_DeriveControlWord        = 0x8,
	TA_TEE_DecryptAVContent         = 0x9,
	TA_TEE_HDCPSendAkeInit          = 0xa,
	TA_TEE_HDCPCheckLLCSignature    = 0xb,
	TA_TEE_HDCPSendAkeNoStoredKm    = 0xc,
	TA_TEE_HDCPSendLCInit           = 0xd,
	TA_TEE_HDCPComputeL             = 0xe,
	TA_TEE_HDCPSendSke              = 0xf,
	TA_TEE_HDCPLC128Cipher          = 0x10,
	TA_TEE_HDCPComputeH             = 0x11,
	TA_TEE_HDCPSaveRxInfo           = 0x12,
	TA_TEE_HDCPGetRxInfo            = 0x13,
	TA_TEE_HDCPReceiverIDList       = 0x14,
	TA_TEE_HDCPComputeM             = 0x15,
	TA_TEE_HDCPLC128CipherRESET     = 0x16,
	TA_TEE_HDCP22_SetParamKey       = 0x17,
	TA_TEE_HDCP22_GetParamKey       = 0x18,
	TA_TEE_HDCP22_SetEncState       = 0x19,
	TA_TEE_HDCPLC128Cipher_2        = 0x20,
	TA_TEE_HDCPLC128CipheClear	= 0x21,
};

static void
rtk_hdmi_hdcp2_tee_generate_random_rtx(struct rtk_hdcp2_tee *hdcp2_tee,
				       struct hdcp2_ake_init *ake_data)
{
	int ret = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	arg_in.func = TA_TEE_HDCPSendAkeInit;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_RTX_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.shm = shm;
	invoke_param[0].u.memref.size = HDCP_2_2_RTX_LEN;
	memcpy(invoke_param[0].u.memref.shm->kaddr, ake_data->r_tx,
		HDCP_2_2_RTX_LEN);

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}

	memcpy(ake_data->r_tx, invoke_param[0].u.memref.shm->kaddr,
		HDCP_2_2_RTX_LEN);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return;
}

static int
rtk_hdmi_hdcp2_tee_verify_rx_cert(struct rtk_hdcp2_tee *hdcp2_tee,
				  struct hdcp2_ake_send_cert *rx_cert)
{
	int err_ret = HDCP2_LLC_SIGNATURE_ERROR, ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	if(rx_cert->msg_id != HDCP_2_2_AKE_SEND_CERT) {
		err_ret = HDCP2_READCERT_ID_ERROR;
		goto exit;
	}

	arg_in.func = TA_TEE_HDCPCheckLLCSignature;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_CERT_RX_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = HDCP_2_2_CERT_RX_LEN;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, &rx_cert->cert_rx,
		HDCP_2_2_CERT_RX_LEN);

	invoke_param[1].u.value.a = 0;

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}

	if(invoke_param[1].u.value.a == 1)
		err_ret = HDCP2_SUCCESS;

free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static void
rtk_hdmi_hdcp2_tee_check_stored_km(struct rtk_hdcp2_tee *hdcp2_tee,
				   u8 *receiver_id,
				   struct hdcp2_ake_no_stored_km *e_kh_km_m,
				   bool *km_stored)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm, *shm2, *shm3;

	arg_in.func = TA_TEE_HDCPGetRxInfo;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	*km_stored = false;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_RECEIVER_ID_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	shm2 = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_E_KH_KM_LEN,
			     TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm2)) {
		pr_err("HDCP2_ERROR: %s tee_shm2_alloc fail\n", __func__);
		goto free_shm;
	}

	shm3 = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_RRX_LEN+HDCP_2_2_RTX_LEN,
			     TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);

	if (IS_ERR(shm3)) {
		pr_err("HDCP2_ERROR: %s tee_shm3_alloc fail\n", __func__);
		goto free_shm2;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;

	invoke_param[0].u.memref.size = HDCP_2_2_RECEIVER_ID_LEN;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, receiver_id,
		HDCP_2_2_RECEIVER_ID_LEN);

	invoke_param[1].u.memref.size = HDCP_2_2_E_KH_KM_LEN;
	invoke_param[1].u.memref.shm = shm2;
	memcpy(invoke_param[1].u.memref.shm->kaddr, e_kh_km_m->e_kpub_km,
		HDCP_2_2_E_KH_KM_LEN);

	invoke_param[2].u.memref.size = HDCP_2_2_RRX_LEN+HDCP_2_2_RTX_LEN;
	invoke_param[2].u.memref.shm = shm3;
	memcpy(invoke_param[2].u.memref.shm->kaddr, e_kh_km_m->e_kpub_km+16,
	       HDCP_2_2_RRX_LEN+HDCP_2_2_RTX_LEN);

	/*Default set the result is failed*/
	invoke_param[3].u.value.a = 1;

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm3;
	}

	if(invoke_param[3].u.value.a == 0) {
		*km_stored = true;
		memcpy(e_kh_km_m->e_kpub_km, invoke_param[1].u.memref.shm->kaddr,
			HDCP_2_2_E_KH_KM_LEN);
		memcpy(e_kh_km_m->e_kpub_km+16, invoke_param[2].u.memref.shm->kaddr,
			HDCP_2_2_RRX_LEN+HDCP_2_2_RTX_LEN);
	}

free_shm3:
	tee_shm_free(shm3);
free_shm2:
	tee_shm_free(shm2);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return;
}

static void
rtk_hdmi_hdcp2_tee_prepare_no_stored_km(struct rtk_hdcp2_tee *hdcp2_tee,
					struct hdcp2_ake_no_stored_km *ek_pub_km)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	arg_in.func = TA_TEE_HDCPSendAkeNoStoredKm;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_E_KPUB_KM_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = HDCP_2_2_E_KPUB_KM_LEN;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, ek_pub_km->e_kpub_km,
		HDCP_2_2_E_KPUB_KM_LEN);

	invoke_param[1].u.value.a = 0;

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}else if(invoke_param[1].u.value.a != 0){
		pr_err("HDCP2_ERROR:%s param value %d\n",
			__func__, invoke_param[1].u.value.a);
		goto free_shm;
	}

	memcpy(ek_pub_km->e_kpub_km, invoke_param[0].u.memref.shm->kaddr,
		HDCP_2_2_E_KPUB_KM_LEN);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return;
}

static void
rtk_hdmi_hdcp2_tee_prepare_stored_km(struct rtk_hdcp2_tee *hdcp2_tee,
				     struct hdcp2_ake_no_stored_km *ek_pub_km)
{
}

static int
rtk_hdmi_hdcp2_tee_verify_hprime(struct rtk_hdcp2_tee *hdcp2_tee,
				 struct hdcp2_ake_send_hprime *rx_hprime,
				 u8 *verified_src, u8 *h)
{
	int err_ret = HDCP2_COMPARE_H_ERROR, ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm, *shm2, *shm3;

	if(rx_hprime->msg_id != HDCP_2_2_AKE_SEND_HPRIME) {
		err_ret = HDCP2_READHPRIME_ID_ERROR;
		goto exit;
	}

	arg_in.func = TA_TEE_HDCPComputeH;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_H_PRIME_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	shm2 = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_H_PRIME_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm2)) {
		pr_err("HDCP2_ERROR: %s tee_shm2_alloc fail\n", __func__);
		goto free_shm;
	}

	shm3 = tee_shm_alloc(hdcp2_tee->hdcp2_ctx,
			     HDCP_2_2_RRX_RTX_RXCAPS_TXCAPS_LEN,
			     TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);

	if (IS_ERR(shm3)) {
		pr_err("HDCP2_ERROR: %s tee_shm3_alloc fail\n", __func__);
		goto free_shm2;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;

	invoke_param[0].u.memref.size = HDCP_2_2_H_PRIME_LEN;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, h, HDCP_2_2_H_PRIME_LEN);

	invoke_param[1].u.memref.size = HDCP_2_2_H_PRIME_LEN;
	invoke_param[1].u.memref.shm = shm2;
	memcpy(invoke_param[1].u.memref.shm->kaddr, rx_hprime->h_prime,
		HDCP_2_2_H_PRIME_LEN);

	invoke_param[2].u.memref.size = HDCP_2_2_RRX_RTX_RXCAPS_TXCAPS_LEN;
	invoke_param[2].u.memref.shm = shm3;
	memcpy(invoke_param[2].u.memref.shm->kaddr, verified_src,
		HDCP_2_2_RRX_RTX_RXCAPS_TXCAPS_LEN);

	/*Default set the result is failed*/
	invoke_param[3].u.value.a = 1;

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm3;
	}

	if(invoke_param[3].u.value.a == 0)
		err_ret = HDCP2_SUCCESS;

free_shm3:
	tee_shm_free(shm3);
free_shm2:
	tee_shm_free(shm2);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static int
rtk_hdmi_hdcp2_tee_store_pairing_info(struct rtk_hdcp2_tee *hdcp2_tee,
				      struct hdcp2_ake_send_pairing_info
							*pairing_info)
{
	int err_ret = HDCP2_READPAIRING_ERROR, ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	if(pairing_info->msg_id != HDCP_2_2_AKE_SEND_PAIRING_INFO) {
		err_ret = HDCP2_READPAIRING_ID_ERROR;
		goto exit;
	}

	arg_in.func = TA_TEE_HDCPSaveRxInfo;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_E_KH_KM_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.shm = shm;
	invoke_param[0].u.memref.size = HDCP_2_2_E_KH_KM_LEN;
	memcpy(invoke_param[0].u.memref.shm->kaddr, pairing_info->e_kh_km,
		HDCP_2_2_E_KH_KM_LEN);

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}
	err_ret = HDCP2_SUCCESS;
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static void
rtk_hdmi_hdcp2_tee_initiate_locality_check(struct rtk_hdcp2_tee *hdcp2_tee,
					   struct hdcp2_lc_init *lc_init)
{
	int ret = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	arg_in.func = TA_TEE_HDCPSendLCInit;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_RN_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.shm = shm;
	invoke_param[0].u.memref.size = HDCP_2_2_RN_LEN;
	memcpy(invoke_param[0].u.memref.shm->kaddr, lc_init->r_n,
		HDCP_2_2_RN_LEN);

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}

	memcpy(lc_init->r_n, invoke_param[0].u.memref.shm->kaddr,
		HDCP_2_2_RN_LEN);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return;
}

static int
rtk_hdmi_hdcp2_tee_verify_lprime(struct rtk_hdcp2_tee *hdcp2_tee,
				 struct hdcp2_lc_send_lprime *rx_lprime,
				 u8 *lprime)
{
	int err_ret = HDCP2_COMPARE_L_ERROR, ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm, *shm2;

	if(rx_lprime->msg_id != HDCP_2_2_LC_SEND_LPRIME) {
		err_ret = HDCP2_READLPRIME_ID_ERROR;
		goto exit;
	}

	arg_in.func = TA_TEE_HDCPComputeL;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_L_PRIME_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	shm2 = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_L_PRIME_LEN,
			     TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm2)) {
		pr_err("HDCP2_ERROR: %s tee_shm2_alloc fail\n", __func__);
		goto free_shm;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = HDCP_2_2_L_PRIME_LEN;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, lprime,
		HDCP_2_2_L_PRIME_LEN);

	invoke_param[1].u.memref.size = HDCP_2_2_L_PRIME_LEN;
	invoke_param[1].u.memref.shm = shm2;
	memcpy(invoke_param[1].u.memref.shm->kaddr, rx_lprime->l_prime,
		HDCP_2_2_L_PRIME_LEN);

	invoke_param[2].u.value.a = 1;

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm2;
	}

	memcpy(lprime, invoke_param[0].u.memref.shm->kaddr,
		HDCP_2_2_L_PRIME_LEN);
	if(invoke_param[2].u.value.a == 0)
		err_ret = HDCP2_SUCCESS;
free_shm2:
	tee_shm_free(shm2);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static void
rtk_hdmi_hdcp2_tee_get_session_key(struct rtk_hdcp2_tee *hdcp2_tee,
				   struct hdcp2_ske_send_eks *ske_data)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm, *shm2;

	arg_in.func = TA_TEE_HDCPSendSke;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_E_DKEY_KS_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	shm2 = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_RIV_LEN,
			     TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm2)) {
		pr_err("HDCP2_ERROR: %s tee_shm2_alloc fail\n", __func__);
		goto free_shm;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = HDCP_2_2_E_DKEY_KS_LEN;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, ske_data->e_dkey_ks,
		HDCP_2_2_E_DKEY_KS_LEN);

	invoke_param[1].u.memref.size = HDCP_2_2_RIV_LEN;
	invoke_param[1].u.memref.shm = shm2;
	memcpy(invoke_param[1].u.memref.shm->kaddr, ske_data->riv,
		HDCP_2_2_RIV_LEN);

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm2;
	}

	memcpy(ske_data->e_dkey_ks, invoke_param[0].u.memref.shm->kaddr,
		HDCP_2_2_E_DKEY_KS_LEN);
	memcpy(ske_data->riv, invoke_param[1].u.memref.shm->kaddr,
		HDCP_2_2_RIV_LEN);

free_shm2:
	tee_shm_free(shm2);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return;
}

static int
rtk_hdmi_hdcp2_tee_repeater_check_flow_prepare_ack(struct rtk_hdcp2_tee *hdcp2_tee,
						   u8 *buf,
						   int msg_size,
						   u8 *mV)
{
	int err_ret = HDCP2_COMPARE_V_ERROR, ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm, *shm2;

	if(buf[0] != HDCP_2_2_REP_SEND_RECVID_LIST) {
		err_ret = HDCP2_READRECEIVEIDLIST_ID_ERROR;
		goto exit;
	}

	arg_in.func = TA_TEE_HDCPReceiverIDList;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, msg_size,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	shm2 = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_V_PRIME_HALF_LEN,
			     TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm2)) {
		pr_err("HDCP2_ERROR: %s tee_shm2_alloc fail\n", __func__);
		goto free_shm;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = msg_size;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, buf, msg_size);

	invoke_param[1].u.memref.size = HDCP_2_2_V_PRIME_HALF_LEN;
	invoke_param[1].u.memref.shm = shm2;
	memcpy(invoke_param[1].u.memref.shm->kaddr, mV,
		HDCP_2_2_V_PRIME_HALF_LEN);

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm2;
	}

	memcpy(mV, invoke_param[1].u.memref.shm->kaddr,
		HDCP_2_2_V_PRIME_HALF_LEN);

	if(invoke_param[2].u.value.a == 0)
		err_ret = HDCP2_SUCCESS;
free_shm2:
	tee_shm_free(shm2);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static int
rtk_hdmi_hdcp2_tee_verify_mprime(struct rtk_hdcp2_tee *hdcp2_tee,
				 struct hdcp2_rep_stream_ready *stream_ready,
				 u8 *input, int input_size)
{
	int err_ret = HDCP2_COMPARE_M_ERROR, ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm, *shm2;

	if(stream_ready->msg_id != HDCP_2_2_REP_STREAM_READY) {
		err_ret = HDCP2_READ_M_ID_ERROR;
		goto exit;
	}

	arg_in.func = TA_TEE_HDCPComputeM;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, input_size,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	shm2 = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, HDCP_2_2_MPRIME_LEN,
			     TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm2)) {
		pr_err("HDCP2_ERROR: %s tee_shm2_alloc fail\n", __func__);
		goto free_shm;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = input_size;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, input, input_size);

	invoke_param[1].u.memref.size = HDCP_2_2_MPRIME_LEN;
	invoke_param[1].u.memref.shm = shm2;
	memcpy(invoke_param[1].u.memref.shm->kaddr, stream_ready->m_prime,
		HDCP_2_2_MPRIME_LEN);

	invoke_param[2].u.value.a = 1;

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm2;
	}

	if(invoke_param[2].u.value.a == 0)
		err_ret = HDCP2_SUCCESS;
free_shm2:
	tee_shm_free(shm2);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static void rtk_hdmi_hdcp2_tee_enable_hdcp2_cipher(struct rtk_hdcp2_tee *hdcp2_tee,
						   u8 *mCap)
{
	int ret = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	arg_in.func = TA_TEE_HDCPLC128Cipher_2;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, 1,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.shm = shm;
	invoke_param[0].u.memref.size = 1;
	memcpy(invoke_param[0].u.memref.shm->kaddr, mCap, 1);

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}
	else
		pr_err("%s: Enable hdcp 2.2 cipher.\n", __func__);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return;
}

static void rtk_hdmi_hdcp2_tee_disable_hdcp2_cipher(struct rtk_hdcp2_tee *hdcp2_tee)
{
	int ret = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;

	arg_in.func = TA_TEE_HDCPLC128CipherRESET;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_param;
	}
	else
		pr_err("%s: Disable hdcp 2.2 cipher.\n", __func__);
free_param:
	kfree(invoke_param);
exit:
	return;
}

static void
rtk_hdmi_hdcp2_tee_clear_cipher_setting(struct rtk_hdcp2_tee *hdcp2_tee)
{
	int ret = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;

	arg_in.func = TA_TEE_HDCPLC128CipheClear;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_param;
	}
	else
		pr_err("%s: Clear hdcp 2.2 cipher setting.\n", __func__);
free_param:
	kfree(invoke_param);
exit:
	return;
}

static void rtk_hdmi_hdcp2_tee_update_mCap(struct rtk_hdcp2_tee *hdcp2_tee,
					   u8 *mCap)
{
	int ret = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	if (hdcp2_tee->init_hdcp2_ta_flag == 0)
		return;

	arg_in.func = TA_TEE_HDCP22_SetEncState;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, 1,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.shm = shm;
	invoke_param[0].u.memref.size = 1;
	memcpy(invoke_param[0].u.memref.shm->kaddr, mCap, 1);

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return;
}

static int hdcp2_optee_match(struct tee_ioctl_version_data *data,
			    const void *vers)
{
	return 1;
}

static void rtk_hdmi_hdcp2_tee_api_init(struct rtk_hdcp2_tee *hdcp2_tee)
{
	int ret;
	struct tee_param  invoke_param[4];
	struct tee_ioctl_version_data vers = {
		.impl_id = TEE_OPTEE_CAP_TZ,
		.impl_caps = TEE_IMPL_ID_OPTEE,
		.gen_caps = TEE_GEN_CAP_GP,
	};

	if (hdcp2_tee->init_hdcp2_ta_flag == 1)
		return;

	hdcp2_tee->init_hdcp2_ta_flag = 1;

	pr_err("HDCP2_INFO: %s\n", __func__);

	hdcp2_tee->hdcp2_ctx = tee_client_open_context(NULL, hdcp2_optee_match, NULL, &vers);

	if (IS_ERR(hdcp2_tee->hdcp2_ctx))
		pr_err("HDCP2_ERROR: %s open context fail\n", __func__);

	memcpy(hdcp2_tee->hdcp2_arg.uuid, ta_hdcptx2_uuid.b, TEE_IOCTL_UUID_LEN);

	hdcp2_tee->hdcp2_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	hdcp2_tee->hdcp2_arg.num_params = 4;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_open_session(hdcp2_tee->hdcp2_ctx, &hdcp2_tee->hdcp2_arg, invoke_param);
	if ((ret < 0) || (hdcp2_tee->hdcp2_arg.ret != TEEC_SUCCESS))
		pr_err("HDCP2_ERROR: %s open session fail, ret=0x%x, hdcp2_arg.ret=0x%x\n",
			__func__, ret, hdcp2_tee->hdcp2_arg.ret);
}

static void rtk_hdmi_hdcp2_tee_api_deinit(struct rtk_hdcp2_tee *hdcp2_tee)
{
	if (hdcp2_tee->init_hdcp2_ta_flag == 0)
		return;

	hdcp2_tee->init_hdcp2_ta_flag = 0;

	pr_err("HDCP2_INFO: %s\n", __func__);

	if (hdcp2_tee->hdcp2_ctx == NULL)
		return;

	tee_client_close_session(hdcp2_tee->hdcp2_ctx, hdcp2_tee->hdcp2_arg.session);
	tee_client_close_context(hdcp2_tee->hdcp2_ctx);
	hdcp2_tee->hdcp2_ctx = NULL;
}

static int rtk_hdmi_hdcp2_tee_read_key(struct rtk_hdcp2_tee *hdcp2_tee)
{
	int ret = 0;
	int err_ret = HDCP2_READKEY_ERROR;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;

	if(hdcp2_tee->init_hdcp2_ta_flag == 0)
		return err_ret;

	arg_in.func = TA_TEE_HDCP22_GetParamKey;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_param;
	}
	err_ret = HDCP2_SUCCESS;
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static int
rtk_hdmi_hdcp2_tee_write_key(struct rtk_hdcp2_tee *hdcp2_tee,
			     char *key, unsigned int keyLength)
{
	int ret;
	int err_ret = HDCP2_PASSKEY_ERROR;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	if (hdcp2_tee->init_hdcp2_ta_flag == 0)
		return err_ret;

	arg_in.func = TA_TEE_HDCP22_SetParamKey;
	arg_in.session = hdcp2_tee->hdcp2_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP2_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp2_tee->hdcp2_ctx, keyLength,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP2_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = keyLength;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, key, keyLength);

	ret = tee_client_invoke_func(hdcp2_tee->hdcp2_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP2_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}else if(invoke_param[1].u.value.a != 0){
		pr_err("HDCP2_ERROR:%s param value %d\n",
			__func__, invoke_param[1].u.value.a);
		goto free_shm;
	}
	err_ret = HDCP2_SUCCESS;
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static const struct rtk_hdcp2_tee_ops rtk_hdmi_hdcp2_tee_ops = {
	.hdcp2_tee_api_init = rtk_hdmi_hdcp2_tee_api_init,
	.hdcp2_tee_api_deinit = rtk_hdmi_hdcp2_tee_api_deinit,
	.generate_random_rtx = rtk_hdmi_hdcp2_tee_generate_random_rtx,
	.verify_rx_cert = rtk_hdmi_hdcp2_tee_verify_rx_cert,
	.prepare_stored_km = rtk_hdmi_hdcp2_tee_prepare_stored_km,
	.prepare_no_stored_km = rtk_hdmi_hdcp2_tee_prepare_no_stored_km,
	.verify_hprime = rtk_hdmi_hdcp2_tee_verify_hprime,
	//.check_stored_km = rtk_hdmi_hdcp2_tee_check_stored_km,
	.store_pairing_info = rtk_hdmi_hdcp2_tee_store_pairing_info,
	.initiate_locality_check = rtk_hdmi_hdcp2_tee_initiate_locality_check,
	.verify_lprime = rtk_hdmi_hdcp2_tee_verify_lprime,
	.get_session_key = rtk_hdmi_hdcp2_tee_get_session_key,
	.repeater_check_flow_prepare_ack = rtk_hdmi_hdcp2_tee_repeater_check_flow_prepare_ack,
	.verify_mprime = rtk_hdmi_hdcp2_tee_verify_mprime,
	.enable_hdcp2_cipher = rtk_hdmi_hdcp2_tee_enable_hdcp2_cipher,
	.disable_hdcp2_cipher = rtk_hdmi_hdcp2_tee_disable_hdcp2_cipher,
	.update_mCap = rtk_hdmi_hdcp2_tee_update_mCap,
	.read_hdcp2_key = rtk_hdmi_hdcp2_tee_read_key,
	.write_hdcp2_key = rtk_hdmi_hdcp2_tee_write_key,
	.clear_hdcp2_cipher_setting = rtk_hdmi_hdcp2_tee_clear_cipher_setting,
};

void rtk_hdcp2_tee_init(struct rtk_hdcp2_tee *hdcp2_tee)
{
	hdcp2_tee->init_hdcp2_ta_flag = 0;
	hdcp2_tee->hdcp2_tee_ops = &rtk_hdmi_hdcp2_tee_ops;
}
