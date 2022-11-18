/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include <linux/slab.h>
#include <linux/uuid.h>
#include <linux/tee_drv.h>
#include <uapi/linux/tee.h>
#include "hdcp.h"

static const uuid_t ta_hdcptx14_uuid = UUID_INIT(0x87ef28e8, 0xf581, 0x4e3d,
	0xb2, 0xb2, 0xd7, 0xe3, 0xd4, 0x8b, 0x23, 0x21);

#define TEEC_SUCCESS    0x00000000
#define ARG_NUM_ELEMENT 4

#define HDCP14_AN_CMD_LEN		8
#define HDCP14_BKSV_CMD_LEN		5
#define HDCP_AKSV_SIZE			5
#define HDCP_BSTATUS_SIZE       2
#define HDCP_R0_SIZE            2

#define HDCP14_PK_SIZE	280

#define MAX_SHA_DATA_SIZE       645
#define MAX_SHA_VPRIME_SIZE     20

#define HDCP14_PARAM_KEY_SIZE   288

#define TEE_IOCTL_UUID_LEN		16

/* VO output control flags */
#define HDCP14_FLAGS_SEND_STATE      BIT(1)
#define HDCP14_FLAGS_OUTPUT_CTRL_EN  BIT(2)
#define HDCP14_FLAGS_SEND_VORPC      BIT(3)
#define HDCP14_FLAGS_DISABLE_VO      BIT(4)

enum HDCP14_CMD_FOR_TA {
	TA_TEE_HDCP14_GenAn			= 0x1,
	TA_TEE_HDCP14_WriteBKSV		= 0x2,
	TA_TEE_HDCP14_SetRepeaterBitInTx	= 0x3,
	TA_TEE_HDCP14_CheckRepeaterBitInTx	= 0x4,
	TA_TEE_HDCP14_SetEnc		= 0x5,
	TA_TEE_HDCP14_SetWinderWin	= 0x6,
	TA_TEE_HDCP14_EnRi			= 0x7,
	TA_TEE_HDCP14_SetAVMute		= 0x8,
	TA_TEE_HDCP14_SHAAppend		= 0x9,
	TA_TEE_HDCP14_ComputeV		= 0xa,
	TA_TEE_HDCP14_VerifyV		= 0xb,
	TA_TEE_HDCP14_CheckR0		= 0xc,
	TA_TEE_HDCP14_GetAKSV		= 0xd,
	TA_TEE_HDCP14_GetCtrlState	= 0xe,
	TA_TEE_HDCP14_SetParamKey	= 0xf,
	TA_TEE_HDCP14_Fix480P       = 0x10,
	TA_TEE_HDCP14_SetKeepoutwin = 0x11,
};

static int init_ta_flag = 0;
static struct tee_ioctl_open_session_arg arg;
static struct tee_context *ctx;

static int hdcp_optee_match(struct tee_ioctl_version_data *data,
	const void *vers)
{
	return 1;
}

void ta_hdcp14_init(void)
{
	int ret;
	struct tee_param  param[4];
	struct tee_ioctl_version_data vers = {
		.impl_id = TEE_OPTEE_CAP_TZ,
		.impl_caps = TEE_IMPL_ID_OPTEE,
		.gen_caps = TEE_GEN_CAP_GP,
	};

	if (init_ta_flag == 1)
		return;

	init_ta_flag = 1;

	pr_err("HDCP_INFO: %s\n", __func__);

	ctx = tee_client_open_context(NULL, hdcp_optee_match, NULL, &vers);

	if (IS_ERR(ctx)) {
		pr_err("HDCP_ERROR: %s open context fail\n", __func__);
		init_ta_flag = 0;
		return;
	}

	memcpy(arg.uuid, ta_hdcptx14_uuid.b, TEE_IOCTL_UUID_LEN);

	arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	arg.num_params = ARG_NUM_ELEMENT;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_open_session(ctx, &arg, param);
	if ((ret < 0) || (arg.ret != TEEC_SUCCESS))
		pr_err("HDCP_ERROR: %s open session fail\n", __func__);
}
EXPORT_SYMBOL(ta_hdcp14_init);

void ta_hdcp14_deinit(void)
{
	if (init_ta_flag == 0)
		return;

	init_ta_flag = 0;

	pr_err("HDCP_INFO: %s\n", __func__);

	if (ctx == NULL)
		return;

	tee_client_close_session(ctx, arg.session);
	tee_client_close_context(ctx);
	ctx = NULL;
}

/**
 * ta_hdcp_lib_generate_an - Generate An
 */
int ta_hdcp_lib_generate_an(uint8_t *an)
{
	int ret;
	int err_code;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;
	struct tee_shm *shm;

	err_code = -HDCP_GEN_AN_ERROR;

	if (!init_ta_flag)
		goto exit;

	arg_I.func = TA_TEE_HDCP14_GenAn;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(ctx, HDCP14_AN_CMD_LEN, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].u.memref.size = HDCP14_AN_CMD_LEN;
	invoke_param[0].u.memref.shm = shm;

	memcpy(invoke_param[0].u.memref.shm->kaddr, an, HDCP14_AN_CMD_LEN);

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS)) {
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);
		goto free_shm;
	}

	memcpy(an, invoke_param[0].u.memref.shm->kaddr, HDCP14_AN_CMD_LEN);

	err_code = HDCP_OK;

free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_code;
}

/**
 * ta_hdcp_lib_get_aksv - Get AKSV from RPMB
 */
int ta_hdcp_lib_get_aksv(uint8_t *aksv)
{
	int ret;
	int check_val = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;
	struct tee_shm *shm;

	if (!init_ta_flag)
		return -EFAULT;

	arg_I.func = TA_TEE_HDCP14_GetAKSV;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param)
		return -ENOMEM;

	shm = tee_shm_alloc(ctx, HDCP_AKSV_SIZE, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm))
		return PTR_ERR(shm);

	invoke_param[0].u.memref.size = HDCP_AKSV_SIZE;
	invoke_param[0].u.memref.shm = shm;

	memcpy(invoke_param[0].u.memref.shm->kaddr, aksv, HDCP_AKSV_SIZE);

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS))
		pr_err("HDCP_ERROR: %s invoke fail\n", __func__);

	check_val = invoke_param[1].u.value.a;
	memcpy(aksv, invoke_param[0].u.memref.shm->kaddr, HDCP_AKSV_SIZE);

	tee_shm_free(shm);
	kfree(invoke_param);

	return check_val;
}

/**
 * ta_hdcp_lib_write_bksv - Write BKSV
 * @ksv_data: input data, 5 bytes of BKSV
 *
 * Return: 0 on success, other on failure
 */
int ta_hdcp_lib_write_bksv(uint8_t *ksv_data)
{
	int ret;
	int err_code;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;
	struct tee_shm *shm;
	struct tee_shm *shm1;

	err_code = -HDCP_BKSV_ERROR;

	if (!init_ta_flag)
		goto exit;

	arg_I.func = TA_TEE_HDCP14_WriteBKSV;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(ctx, HDCP14_BKSV_CMD_LEN,
			TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].u.memref.size = HDCP14_BKSV_CMD_LEN;
	invoke_param[0].u.memref.shm = shm;

	memcpy(invoke_param[0].u.memref.shm->kaddr, ksv_data, HDCP14_BKSV_CMD_LEN);

	shm1 = tee_shm_alloc(ctx, HDCP14_PK_SIZE, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm1)) {
		pr_err("HDCP_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_shm;
	}

	invoke_param[1].u.memref.size = HDCP14_PK_SIZE;
	invoke_param[1].u.memref.shm = shm1;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS)) {
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);
		goto free_shm1;
	}

	memcpy(ksv_data, invoke_param[0].u.memref.shm->kaddr, HDCP14_BKSV_CMD_LEN);

	err_code = HDCP_OK;

free_shm1:
	tee_shm_free(shm1);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return err_code;
}

void ta_hdcp_lib_set_repeater_bit_in_tx(enum hdcp_repeater rx_mode)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;

	if (!init_ta_flag)
		return;

	arg_I.func = TA_TEE_HDCP14_SetRepeaterBitInTx;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		return;
	}

	invoke_param[0].u.value.a = rx_mode;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS))
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);

	kfree(invoke_param);
}

int ta_hdcp_lib_check_repeater_bit_in_tx(void)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;
	unsigned int check_val = 0;

	if (!init_ta_flag)
		goto exit;

	arg_I.func = TA_TEE_HDCP14_CheckRepeaterBitInTx;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	invoke_param[0].u.value.a = 0;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS)) {
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);
		goto free_param;
	}

	check_val = invoke_param[0].u.value.a;

free_param:
	kfree(invoke_param);
exit:
	return check_val;
}

void ta_hdcp_lib_set_encryption(enum encryption_state enc_state)
{
	int ret;
	unsigned int flags;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;

	if (!init_ta_flag)
		return;

	arg_I.func = TA_TEE_HDCP14_SetEnc;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		return;
	}
	invoke_param[0].u.value.a = enc_state;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS)) {
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);
		return;
	}

	flags = invoke_param[0].u.value.a;
	if (flags & HDCP14_FLAGS_OUTPUT_CTRL_EN)
		pr_info("HDCP_INFO: disable_vo=%u\n",
			(flags & HDCP14_FLAGS_DISABLE_VO) ? 1:0);

	kfree(invoke_param);
}
EXPORT_SYMBOL(ta_hdcp_lib_set_encryption);

void ta_hdcp_lib_set_wider_window(void)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;

	if (!init_ta_flag)
		return;

	arg_I.func = TA_TEE_HDCP14_SetWinderWin;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		return;
	}

	invoke_param[0].u.value.a = 1;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS))
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);

	kfree(invoke_param);
}

void ta_hdcp_lib_set_ri(enum ri_state ri_state)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;

	if (!init_ta_flag)
		return;

	arg_I.func = TA_TEE_HDCP14_EnRi;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		return;
	}

	invoke_param[0].u.value.a = ri_state;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS))
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);

	kfree(invoke_param);
}

void ta_hdcp_lib_set_av_mute(enum av_mute av_mute_state)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;

	if (!init_ta_flag)
		return;

	arg_I.func = TA_TEE_HDCP14_SetAVMute;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		return;
	}

	invoke_param[0].u.value.a = av_mute_state;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS))
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);

	kfree(invoke_param);
}

void ta_hdcp_lib_SHA_append_bstatus_M0(struct hdcp_sha_in *sha, uint8_t *bstatus)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;
	struct tee_shm *shm;
	struct tee_shm *shm1;

	if (!init_ta_flag)
		goto exit;

	arg_I.func = TA_TEE_HDCP14_SHAAppend;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(ctx, MAX_SHA_DATA_SIZE, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].u.memref.size = MAX_SHA_DATA_SIZE;
	invoke_param[0].u.memref.shm = shm;

	memcpy(invoke_param[0].u.memref.shm->kaddr, sha->data, MAX_SHA_DATA_SIZE);

	shm1 = tee_shm_alloc(ctx, 2, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm1)) {
		pr_err("HDCP_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_shm;
	}

	invoke_param[1].u.memref.size = 2;
	invoke_param[1].u.memref.shm = shm1;

	memcpy(invoke_param[1].u.memref.shm->kaddr, bstatus, 2);

	invoke_param[2].u.value.a = sha->byte_counter;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS)) {
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);
		goto free_shm1;
	}

	sha->byte_counter = invoke_param[2].u.value.a;
	memcpy(sha->data, invoke_param[0].u.memref.shm->kaddr, MAX_SHA_DATA_SIZE);

free_shm1:
	tee_shm_free(shm1);
free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return;
}

/**
 * ta_hdcp_lib_compute_V
 * @sha: struct hdcp_sha_in
 *
 * Return: 0 on compute success, other on failure
 */
int ta_hdcp_lib_compute_V(struct hdcp_sha_in *sha)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;
	struct tee_shm *shm;

	ret = -HDCP_SHA1_ERROR;

	if (!init_ta_flag)
		goto exit;

	arg_I.func = TA_TEE_HDCP14_ComputeV;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(ctx, MAX_SHA_DATA_SIZE, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].u.memref.size = MAX_SHA_DATA_SIZE;
	invoke_param[0].u.memref.shm = shm;

	memcpy(invoke_param[0].u.memref.shm->kaddr, sha->data, MAX_SHA_DATA_SIZE);

	invoke_param[1].u.value.a = sha->byte_counter;
	invoke_param[1].u.value.b = ret;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS)) {
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);
		ret = -HDCP_SHA1_ERROR;
		goto free_shm;
	}

	sha->byte_counter = invoke_param[1].u.value.a;
	memcpy(sha->data, invoke_param[0].u.memref.shm->kaddr, MAX_SHA_DATA_SIZE);

	ret = invoke_param[1].u.value.b;

free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return ret;
}

/**
 * ta_hdcp_lib_verify_V
 * @sha: struct hdcp_sha_in
 *
 * Return: 0 on verify match, other on failure
 */
int ta_hdcp_lib_verify_V(struct hdcp_sha_in *sha)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;
	struct tee_shm *shm;

	ret = -HDCP_SHA1_ERROR;

	if (!init_ta_flag)
		goto exit;

	arg_I.func = TA_TEE_HDCP14_VerifyV;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(ctx, MAX_SHA_VPRIME_SIZE, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].u.memref.size = MAX_SHA_DATA_SIZE;
	invoke_param[0].u.memref.shm = shm;

	memcpy(invoke_param[0].u.memref.shm->kaddr, sha->vprime, MAX_SHA_VPRIME_SIZE);

	invoke_param[1].u.value.a = ret;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);

	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS)) {
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);
		ret = -HDCP_SHA1_ERROR;
		goto free_shm;
	}

	memcpy(sha->vprime, invoke_param[0].u.memref.shm->kaddr, MAX_SHA_VPRIME_SIZE);

	ret = invoke_param[1].u.value.a;
	if (ret == HDCP_OK)
		pr_info("HDCP_INFO: Verify V passed\n");
	else
		pr_err("HDCP_ERROR: V != V' in second part of authentication\n");

free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return ret;
}

/**
 * ta_hdcp_lib_check_r0
 * @r0_rx: R0' or Ri' (2 Bytes)
 *
 * Return: 0 on verify match, other on failure
 */
int ta_hdcp_lib_check_r0(uint8_t *r0_rx)
{
	int ret;
	struct tee_param *invoke_param;
	struct tee_ioctl_invoke_arg arg_I;
	struct tee_shm *shm;
	uint8_t r0_tx[2];

	ret = -HDCP_R0_ERROR;

	if (!init_ta_flag)
		goto exit;

	arg_I.func = TA_TEE_HDCP14_CheckR0;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(ctx, HDCP_R0_SIZE, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].u.memref.size = HDCP_R0_SIZE;
	invoke_param[0].u.memref.shm = shm;

	memcpy(invoke_param[0].u.memref.shm->kaddr, r0_rx, HDCP_R0_SIZE);

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);

	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS)) {
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);
		ret = -HDCP_R0_ERROR;
		goto free_shm;
	}

	memcpy(r0_tx, invoke_param[0].u.memref.shm->kaddr, HDCP_R0_SIZE);

	ret = invoke_param[1].u.value.a;

	if (ret != HDCP_OK)
		pr_err("HDCP_ERROR: Ri:%02x%02x != Ri':%02x%02x\n",
			r0_tx[0], r0_tx[1], r0_rx[0], r0_rx[1]);

free_shm:
	tee_shm_free(shm);
free_param:
	kfree(invoke_param);
exit:
	return ret;
}

int ta_hdcp_get_ctrl_state(void)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;
	int hdcp_state;

	hdcp_state = 0;

	if (!init_ta_flag)
		goto exit;

	arg_I.func = TA_TEE_HDCP14_GetCtrlState;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS)) {
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);
		goto free_param;
	}

	if (invoke_param[1].u.value.a == 1)
		hdcp_state = 2; /* hdcp2.x */
	else if (invoke_param[0].u.value.a == 1)
		hdcp_state = 1; /* hdcp1.4 */

free_param:
	kfree(invoke_param);
exit:
	return hdcp_state;
}

int ta_hdcp_set_param_key(unsigned char *param_key)
{
	int ret;
	int val;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;
	struct tee_shm *shm;

	ret = -EFAULT;

	if (!init_ta_flag)
		goto exit;

	arg_I.func = TA_TEE_HDCP14_SetParamKey;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	shm = tee_shm_alloc(ctx, HDCP14_PARAM_KEY_SIZE, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		pr_err("HDCP_ERROR: %s tee_shm_alloc fail\n", __func__);
		goto free_param;
	}

	invoke_param[0].u.memref.size = HDCP14_PARAM_KEY_SIZE;
	invoke_param[0].u.memref.shm = shm;

	memcpy(invoke_param[0].u.memref.shm->kaddr, param_key, HDCP14_PARAM_KEY_SIZE);

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	invoke_param[1].u.value.a = 1;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS)) {
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);
		ret = -EFAULT;
		goto free_param;
	}

	val = invoke_param[1].u.value.a;
	if (val != 0) {
		pr_err("HDCP_ERROR: %s set key fail\n", __func__);
		goto free_param;
	}

	ret = 0;
free_param:
	kfree(invoke_param);
exit:
	return ret;
}

int ta_hdcp_fix480p(void)
{
	int ret;
	int val = 0;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;

	if (!init_ta_flag)
		goto exit;

	arg_I.func = TA_TEE_HDCP14_Fix480P;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		goto exit;
	}

	invoke_param[0].u.value.a = 0;/* Reserved */

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS))
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);
	else
		val = invoke_param[0].u.value.a;

	kfree(invoke_param);
exit:
	return val;
}

void ta_hdcp_set_keepout_win(void)
{
	int ret;
	struct tee_param *invoke_param = NULL;
	struct tee_ioctl_invoke_arg arg_I;

	arg_I.func = TA_TEE_HDCP14_SetKeepoutwin;
	arg_I.session = arg.session;
	arg_I.num_params = ARG_NUM_ELEMENT;

	invoke_param = kcalloc(ARG_NUM_ELEMENT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param) {
		pr_err("HDCP_ERROR: %s kcalloc fail\n", __func__);
		return;
	}

	invoke_param[0].u.value.a = 0x1fa; /* keepoutwinstart */
	invoke_param[0].u.value.b = 0x288; /* keepoutwinend */

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	ret = tee_client_invoke_func(ctx, &arg_I, invoke_param);
	if ((ret < 0) || (arg_I.ret != TEEC_SUCCESS))
		pr_err("HDCP_ERROR: %s invoke fail, ret(0x%x)\n", __func__, arg_I.ret);

	kfree(invoke_param);
}
EXPORT_SYMBOL(ta_hdcp_set_keepout_win);
