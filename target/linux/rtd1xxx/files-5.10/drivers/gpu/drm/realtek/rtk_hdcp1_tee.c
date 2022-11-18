/*
 * RealTek HDCP 1.4 CA driver
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

#include "rtk_hdcp1_tee.h"

#define TEEC_SUCCESS            0x0
#define TEE_IOCTL_UUID_LEN      16

#define MAX_TEE_SHA_DATA_SIZE       645
#define MAX_TEE_SHA_VPRIME_SIZE     20

static const uuid_t ta_hdcptx14_uuid = UUID_INIT(0x87ef28e8, 0xf581, 0x4e3d,
						 0xb2, 0xb2, 0xd7, 0xe3, 0xd4, 0x8b, 0x23, 0x21);

enum HDCP14_CMD_FOR_TA {
	TA_TEE_HDCP14_GenAn                     = 0x1,
	TA_TEE_HDCP14_WriteBKSV                 = 0x2,
	TA_TEE_HDCP14_SetRepeaterBitInTx        = 0x3,
	TA_TEE_HDCP14_CheckRepeaterBitInTx      = 0x4,
	TA_TEE_HDCP14_SetEnc                    = 0x5,
	TA_TEE_HDCP14_SetWinderWin              = 0x6,
	TA_TEE_HDCP14_EnRi                      = 0x7,
	TA_TEE_HDCP14_SetAVMute                 = 0x8,
	TA_TEE_HDCP14_SHAAppend                 = 0x9,
	TA_TEE_HDCP14_ComputeV                  = 0xa,
	TA_TEE_HDCP14_VerifyV                   = 0xb,
	TA_TEE_HDCP14_CheckR0                   = 0xc,
	TA_TEE_HDCP14_GetAKSV                   = 0xd,
	TA_TEE_HDCP14_GetCtrlState              = 0xe,
	TA_TEE_HDCP14_SetParamKey               = 0xf,
	TA_TEE_HDCP14_Fix480P                   = 0x10,
	TA_TEE_HDCP14_SetKeepoutwin             = 0x11,
};

static int hdcp1_optee_match(struct tee_ioctl_version_data *data,
			    const void *vers)
{
	return 1;
}

static void rtk_hdmi_hdcp1_tee_api_init(struct rtk_hdcp1_tee *hdcp1_tee)
{
	int ret;
	struct tee_param  invoke_param[4];
	struct tee_ioctl_version_data vers = {
		.impl_id = TEE_OPTEE_CAP_TZ,
		.impl_caps = TEE_IMPL_ID_OPTEE,
		.gen_caps = TEE_GEN_CAP_GP,
	};

	if (hdcp1_tee->init_hdcp1_ta_flag == 1)
		return;

	hdcp1_tee->init_hdcp1_ta_flag = 1;

	pr_err("HDCP_INFO: %s\n", __func__);

	hdcp1_tee->hdcp1_ctx = tee_client_open_context(NULL, hdcp1_optee_match, NULL, &vers);

	if (IS_ERR(hdcp1_tee->hdcp1_ctx))
		pr_err("HDCP_ERROR: %s open context fail\n", __func__);

	memcpy(hdcp1_tee->hdcp1_arg.uuid, ta_hdcptx14_uuid.b, TEE_IOCTL_UUID_LEN);

	hdcp1_tee->hdcp1_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	hdcp1_tee->hdcp1_arg.num_params = 4;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_open_session(hdcp1_tee->hdcp1_ctx, &hdcp1_tee->hdcp1_arg, invoke_param);
	if ((ret < 0) || (hdcp1_tee->hdcp1_arg.ret != TEEC_SUCCESS))
		pr_err("HDCP1_ERROR: %s open session fail, ret=0x%x, hdcp1_arg.ret=0x%x\n",
			__func__, ret, hdcp1_tee->hdcp1_arg.ret);
}

static void rtk_hdmi_hdcp1_tee_api_deinit(struct rtk_hdcp1_tee *hdcp1_tee)
{
	if (hdcp1_tee->init_hdcp1_ta_flag == 0)
		return;

	hdcp1_tee->init_hdcp1_ta_flag = 0;

	pr_err("HDCP1_INFO: %s\n", __func__);

	if (hdcp1_tee->hdcp1_ctx == NULL)
		return;

	tee_client_close_session(hdcp1_tee->hdcp1_ctx, hdcp1_tee->hdcp1_arg.session);
	tee_client_close_context(hdcp1_tee->hdcp1_ctx);
	hdcp1_tee->hdcp1_ctx = NULL;
}

static int
rtk_hdmi_hdcp1_tee_generate_an(struct rtk_hdcp1_tee *hdcp1_tee, u8 *an)
{
	int ret;
	int err_ret = HDCP1_GEN_AN_ERROR;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		return err_ret;

	arg_in.func = TA_TEE_HDCP14_GenAn;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp1_tee->hdcp1_ctx, DRM_HDCP_AN_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);

	if (IS_ERR(shm)) {
		pr_err("HDCP1_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = DRM_HDCP_AN_LEN;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, an, DRM_HDCP_AN_LEN);

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}
	memcpy(an, invoke_param[0].u.memref.shm->kaddr, DRM_HDCP_AN_LEN);
	err_ret = ret;

free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

/*
 * ta_hdcp_lib_get_aksv - Get AKSV from RPMB
 */
static int
rtk_hdmi_hdcp1_tee_read_aksv(struct rtk_hdcp1_tee *hdcp1_tee, u8 *aksv)
{
	int ret;
	int err_ret = HDCP1_AKSV_ERROR;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		return err_ret;

	arg_in.func = TA_TEE_HDCP14_GetAKSV;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp1_tee->hdcp1_ctx, DRM_HDCP_KSV_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);

	if (IS_ERR(shm)) {
		pr_err("HDCP1_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = DRM_HDCP_KSV_LEN;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, aksv, DRM_HDCP_KSV_LEN);

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}
	memcpy(aksv, invoke_param[0].u.memref.shm->kaddr, DRM_HDCP_KSV_LEN);

	err_ret = invoke_param[1].u.value.a;
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static void
rtk_hdmi_hdcp1_tee_set_hdcp1_repeater_bit(struct rtk_hdcp1_tee *hdcp1_tee,
					  u8 is_repeater)
{
	int ret = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		return;

	arg_in.func = TA_TEE_HDCP14_SetRepeaterBitInTx;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		return;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.value.a = is_repeater;

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
	}
	kfree(invoke_param);
}

static int
rtk_hdmi_hdcp1_tee_write_bksv(struct rtk_hdcp1_tee *hdcp1_tee, u8 *bksv)
{
	int ret;
	int err_ret = HDCP1_BKSV_ERROR;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		return err_ret;

	arg_in.func = TA_TEE_HDCP14_WriteBKSV;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp1_tee->hdcp1_ctx, DRM_HDCP_KSV_LEN,
				TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);

	if (IS_ERR(shm)) {
		pr_err("HDCP1_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = DRM_HDCP_KSV_LEN;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, bksv, DRM_HDCP_KSV_LEN);

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}
	err_ret = ret;
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static int
rtk_hdmi_hdcp1_tee_check_ri_prime(struct rtk_hdcp1_tee *hdcp1_tee,
				  u8 *ri_prime)
{
	int ret;
	int err_ret = HDCP1_R0_ERROR;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;
	u8 ri[DRM_HDCP_RI_LEN];

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		return err_ret;

	arg_in.func = TA_TEE_HDCP14_CheckR0;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp1_tee->hdcp1_ctx, DRM_HDCP_RI_LEN,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);

	if (IS_ERR(shm)) {
		pr_err("HDCP1_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = DRM_HDCP_RI_LEN;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, ri_prime, DRM_HDCP_RI_LEN);

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}
	memcpy(ri, invoke_param[0].u.memref.shm->kaddr, DRM_HDCP_RI_LEN);

	err_ret = invoke_param[1].u.value.a;

	if (err_ret != HDCP1_SUCCESS)
		pr_err("HDCP1_ERROR: Ri:%02x%02x != Ri':%02x%02x\n",
			ri[0], ri[1], ri_prime[0], ri_prime[1]);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static void
rtk_hdmi_hdcp1_tee_set_encryption(struct rtk_hdcp1_tee *hdcp1_tee,
				  u8 enc_state)
{
	int ret = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		return;

	arg_in.func = TA_TEE_HDCP14_SetEnc;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		return;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.value.a = enc_state;

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
	}
	kfree(invoke_param);
}

static void
rtk_hdmi_hdcp1_tee_sha_append_bstatus_m0(struct rtk_hdcp1_tee *hdcp1_tee,
					 u8 *ksv_fifo, int *byte_cnt,
					 u8 *bstatus)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm, *shm2;

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		return;

	arg_in.func = TA_TEE_HDCP14_SHAAppend;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		return;
	}

	shm = tee_shm_alloc(hdcp1_tee->hdcp1_ctx, MAX_TEE_SHA_DATA_SIZE,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP1_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	shm2 = tee_shm_alloc(hdcp1_tee->hdcp1_ctx, DRM_HDCP_BSTATUS_LEN,
			     TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm2)) {
		pr_err("HDCP1_ERROR: %s tee_shm2_alloc fail\n", __func__);
		goto free_shm;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = MAX_TEE_SHA_DATA_SIZE;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, ksv_fifo,
		MAX_TEE_SHA_DATA_SIZE);

	invoke_param[1].u.memref.size = DRM_HDCP_BSTATUS_LEN;
	invoke_param[1].u.memref.shm = shm2;
	memcpy(invoke_param[1].u.memref.shm->kaddr, bstatus,
		DRM_HDCP_BSTATUS_LEN);

	invoke_param[2].u.value.a = (*byte_cnt);

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm2;
	}

	(*byte_cnt) = invoke_param[2].u.value.a;
	memcpy(ksv_fifo, invoke_param[0].u.memref.shm->kaddr,
		MAX_TEE_SHA_DATA_SIZE);

free_shm2:
	tee_shm_free(shm2);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
}

static int
rtk_hdmi_hdcp1_tee_compute_V(struct rtk_hdcp1_tee *hdcp1_tee,
			     u8 *ksv_fifo, int *byte_cnt)
{
	int ret;
	int err_ret = HDCP1_SHA1_ERROR;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		return err_ret;

	arg_in.func = TA_TEE_HDCP14_ComputeV;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp1_tee->hdcp1_ctx, MAX_TEE_SHA_DATA_SIZE,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);

	if (IS_ERR(shm)) {
		pr_err("HDCP1_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = MAX_TEE_SHA_DATA_SIZE;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, ksv_fifo,
		MAX_TEE_SHA_DATA_SIZE);

	invoke_param[1].u.value.a = (*byte_cnt);
	invoke_param[1].u.value.b = err_ret;

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}

	(*byte_cnt) = invoke_param[1].u.value.a;
	memcpy(ksv_fifo, invoke_param[0].u.memref.shm->kaddr,
		MAX_TEE_SHA_DATA_SIZE);

	err_ret = invoke_param[1].u.value.b;
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static int
rtk_hdmi_hdcp1_tee_verify_V(struct rtk_hdcp1_tee *hdcp1_tee,
			    u8 *vprime)
{
	int ret;
	int err_ret = HDCP1_SHA1_ERROR;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		return err_ret;

	arg_in.func = TA_TEE_HDCP14_VerifyV;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp1_tee->hdcp1_ctx, MAX_TEE_SHA_VPRIME_SIZE,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);

	if (IS_ERR(shm)) {
		pr_err("HDCP1_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	/*MAX_TEE_SHA_DATA_SIZE is a Bug, it should be MAX_TEE_SHA_VPRIME_SIZE,
	  However, the TA also check this value MAX_TEE_SHA_DATA_SIZE.
	  it will return bad parameter error from TA
	  if we only just modify this CA file.
	 */
	invoke_param[0].u.memref.size = MAX_TEE_SHA_DATA_SIZE;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, vprime,
		MAX_TEE_SHA_VPRIME_SIZE);

	invoke_param[1].u.value.a = err_ret;

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}
	memcpy(vprime, invoke_param[0].u.memref.shm->kaddr,
		MAX_TEE_SHA_VPRIME_SIZE);

	err_ret = invoke_param[1].u.value.a;
	if (err_ret != HDCP1_SUCCESS)
		pr_err("HDCP1_ERROR: V != V' in second part of authentication\n");
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static void
rtk_hdmi_hdcp1_tee_set_wider_window(struct rtk_hdcp1_tee *hdcp1_tee)
{
	int ret = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		return;

	arg_in.func = TA_TEE_HDCP14_SetWinderWin;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		return;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.value.a = 1;

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
	}
	kfree(invoke_param);
}

static int rtk_hdmi_hdcp1_tee_write_key(struct rtk_hdcp1_tee *hdcp1_tee,
					unsigned char *key)
{
	int ret;
	int err_ret = HDCP1_PASSKEY_ERROR;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;
	struct tee_shm *shm;

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		return err_ret;

	arg_in.func = TA_TEE_HDCP14_SetParamKey;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(hdcp1_tee->hdcp1_ctx, HDCP14_KEY_SIZE,
			    TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);

	if (IS_ERR(shm)) {
		pr_err("HDCP1_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.memref.size = HDCP14_KEY_SIZE;
	invoke_param[0].u.memref.shm = shm;
	memcpy(invoke_param[0].u.memref.shm->kaddr, key, HDCP14_KEY_SIZE);

	invoke_param[1].u.value.a = 1;

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
		goto free_shm;
	}

	ret = invoke_param[1].u.value.a;
	if (ret != HDCP1_SUCCESS) {
		pr_err("HDCP_ERROR: %s set key fail\n", __func__);
		goto free_shm;
	}
	err_ret = HDCP1_SUCCESS;
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_ret;
}

static int rtk_hdmi_hdcp1_tee_fix480p(struct rtk_hdcp1_tee *hdcp1_tee)
{
	int ret = 0;
	int val = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		goto exit;

	arg_in.func = TA_TEE_HDCP14_Fix480P;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.value.a = 0;

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
	}
	else
		val = invoke_param[0].u.value.a;

	kfree(invoke_param);
exit:
	return val;
}

static void rtk_hdmi_hdcp1_tee_set_keepout_win(struct rtk_hdcp1_tee *hdcp1_tee)
{
	int ret = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_in;

	if(hdcp1_tee->init_hdcp1_ta_flag == 0)
		return;

	arg_in.func = TA_TEE_HDCP14_SetKeepoutwin;
	arg_in.session = hdcp1_tee->hdcp1_arg.session;
	arg_in.num_params = 4;

	invoke_param = kcalloc(4, sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP1_ERROR: %s kcalloc fail\n", __func__);
		return;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[0].u.value.a = 0x1fa; /* keepoutwinstart */
	invoke_param[0].u.value.b = 0x288; /* keepoutwinend */

	ret = tee_client_invoke_func(hdcp1_tee->hdcp1_ctx, &arg_in, invoke_param);
	if ((ret < 0) || (arg_in.ret != TEEC_SUCCESS)) {
		pr_err("HDCP1_ERROR: %s invoke fail, ret=0x%x, arg_in.ret=0x%x\n",
			__func__, ret, arg_in.ret);
	}

	kfree(invoke_param);
}

static const struct rtk_hdcp1_tee_ops rtk_hdmi_hdcp1_tee_ops = {
	.hdcp1_tee_api_init = rtk_hdmi_hdcp1_tee_api_init,
	.hdcp1_tee_api_deinit = rtk_hdmi_hdcp1_tee_api_deinit,
	.generate_an = rtk_hdmi_hdcp1_tee_generate_an,
	.read_aksv = rtk_hdmi_hdcp1_tee_read_aksv,
	.set_hdcp1_repeater_bit = rtk_hdmi_hdcp1_tee_set_hdcp1_repeater_bit,
	.write_bksv = rtk_hdmi_hdcp1_tee_write_bksv,
	.check_ri_prime = rtk_hdmi_hdcp1_tee_check_ri_prime,
	.hdcp1_set_encryption = rtk_hdmi_hdcp1_tee_set_encryption,
	.sha_append_bstatus_m0 = rtk_hdmi_hdcp1_tee_sha_append_bstatus_m0,
	.compute_V = rtk_hdmi_hdcp1_tee_compute_V,
	.verify_V = rtk_hdmi_hdcp1_tee_verify_V,
	.set_wider_window = rtk_hdmi_hdcp1_tee_set_wider_window,
	.write_hdcp1_key = rtk_hdmi_hdcp1_tee_write_key,
	.fix480p = rtk_hdmi_hdcp1_tee_fix480p,
	.set_keepout_win = rtk_hdmi_hdcp1_tee_set_keepout_win,
};

void rtk_hdcp1_tee_init(struct rtk_hdcp1_tee *hdcp1_tee)
{
	hdcp1_tee->init_hdcp1_ta_flag = 0;
	hdcp1_tee->hdcp1_tee_ops = &rtk_hdmi_hdcp1_tee_ops;
}
