/*
* Copyright (C) 2002-2022 Realtek Semiconductor Corp.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License version 2.1 as
* published by the Free Software Foundation.
*
*/
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/uaccess.h>
#include <soc/realtek/rtk_tee.h>

#include "../tee_private.h"
#include "../tee_client_api.h"
#include "ion_rtk_protected_notifier.h"

static const uuid_t pta_mem_uuid = UUID_INIT(0xb8b220fc, 0x3851, 0x4d5a,
	0xa6, 0x7e, 0x9d, 0x45, 0xed, 0xdf, 0xc5, 0x32);
static const uuid_t ta_mem_uuid = UUID_INIT(0x7aaaf200, 0x2450, 0x11e4,
	0xab, 0xe2, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b);

#define TEE_MEM_TAG "TEE_MEM : "

//#define TEE_MEM_EMPTY_FUNCTION // FOR TEST

enum tee_mem_cmd_for_ta {
    TA_TEE_MEM_PROTECTED_DESTROY_SSID   = 24,
    TA_TEE_MEM_PROTECTED_CREATE_SSID    = 26,
    TA_TEE_MEM_PROTECTED_CHANGE         = 27,
	TA_TEE_MEM_PROTECTED_EXT_SET_SSID = 28,
	TA_TEE_MEM_PROTECTED_EXT_UNSET_SSID = 29,
};

struct tee_mem_region {
	unsigned int type;
	unsigned long long base;
	unsigned long long size;
};

struct tee_mem_ext_region {
	unsigned int ext;
	unsigned long long base;
	unsigned long long size;
	long long parent_ssid;
};

struct tee_mem_protected_create_ssid {
    struct tee_mem_region   mem;
    long long               ssid;
};

struct tee_mem_protected_destroy_ssid {
    long long               ssid;
};

struct tee_mem_protected_change_ssid {
    struct tee_mem_region   mem;
    long long               ssid;
};

struct tee_mem_protected_ext_set_ssid {
	struct tee_mem_ext_region mem;
	long long ssid;
};

struct tee_mem_protected_ext_unset_ssid {
	long long ssid;
};

struct tee_mem_protected_slot {
    struct list_head        list;
    struct tee_mem_region   mem;
    long long               ssid;
};

struct tee_mem_protected_ext_slot {
	struct list_head list;
	struct tee_mem_ext_region mem;
	long long ssid;
};

struct tee_mem_api_module *gModule;

static int protected_handler(struct notifier_block *self, unsigned long val, void *data);

struct tee_mem_api_module *tee_mem_protected_get_gmodule(void)
{
	return gModule;
}
EXPORT_SYMBOL_GPL(tee_mem_protected_get_gmodule);

static void tee_mem_protected_slot_init (struct tee_mem_protected_slot * slot,
        unsigned long long base, unsigned long long size, long long ssid, unsigned int type)
{
    memset((void *) slot, 0, sizeof(struct tee_mem_protected_slot));
    INIT_LIST_HEAD(&slot->list);
    slot->mem.base  = base;
    slot->mem.size  = size;
	slot->mem.type  = type;
    slot->ssid      = ssid;
}

static void tee_mem_protected_ext_slot_init(struct tee_mem_protected_ext_slot *slot,
		unsigned long long base,
		unsigned long long size, long long ssid,
		unsigned int ext, struct tee_mem_protected_slot * parent)
{
	memset((void *)slot, 0, sizeof(struct tee_mem_protected_ext_slot));
	INIT_LIST_HEAD(&slot->list);
	slot->mem.base          = base;
	slot->mem.size          = size;
	slot->mem.ext           = ext;
	slot->mem.parent_ssid   = parent->ssid;
	slot->ssid              = ssid;
}

static __maybe_unused int mem_optee_match(struct tee_ioctl_version_data *data,
	const void *vers)
{
	return 1;
}

static struct tee_mem_protected_slot * tee_mem_protected_create (struct tee_mem_api_module * module, unsigned long base, size_t size, unsigned int type)
{
    struct tee_mem_protected_slot * ret = NULL;
    do {
#ifndef TEE_MEM_EMPTY_FUNCTION
        struct tee_param                        *invoke_param = NULL;
        struct tee_ioctl_invoke_arg             tee_arg;
        struct tee_shm *                        tee_shm;
        struct tee_mem_protected_create_ssid *  create_info;
#endif /* end of TEE_MEM_EMPTY_FUNCTION */
        struct tee_mem_protected_slot *         slot;

        slot = (struct tee_mem_protected_slot *) kzalloc(sizeof(struct tee_mem_protected_slot), GFP_KERNEL);
        if (!slot)
            break;

#ifndef TEE_MEM_EMPTY_FUNCTION
        tee_shm = tee_shm_alloc(module->tee_context, sizeof (struct tee_mem_protected_create_ssid), TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
        if (IS_ERR(tee_shm)) {
            kfree(slot);
            break;
        }

        create_info = (struct tee_mem_protected_create_ssid *) tee_shm->kaddr;
        create_info->mem.base = base;
        create_info->mem.size = size;
		create_info->mem.type = type;

	invoke_param = kcalloc(TEEC_CONFIG_PAYLOAD_REF_COUNT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param)
		return ret;

        invoke_param[0].u.memref.size = sizeof(struct tee_mem_protected_create_ssid);
        invoke_param[0].u.memref.shm = tee_shm;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

        memset(&tee_arg, 0, sizeof(struct tee_ioctl_invoke_arg));
        tee_arg.func = TA_TEE_MEM_PROTECTED_CREATE_SSID;
        tee_arg.session = module->tee_session;
        tee_arg.num_params = TEEC_CONFIG_PAYLOAD_REF_COUNT;

	tee_client_invoke_func(module->tee_context, &tee_arg, invoke_param);
        if (tee_arg.ret != 0) {
            pr_info("%s%s : %d ==== tee_arg.ret:%08x ====\n", TEE_MEM_TAG, __func__, __LINE__, tee_arg.ret);
            tee_shm_free(tee_shm);
	    kfree(invoke_param);
            kfree(slot);
            break;
        }

	pr_info("%s%s : %d ssid:%x\n", TEE_MEM_TAG, __func__, __LINE__, create_info->ssid);
        tee_mem_protected_slot_init(slot, base, size, create_info->ssid, type);
        tee_shm_free(tee_shm);
	kfree(invoke_param);
#else /* else of TEE_MEM_EMPTY_FUNCTION */
        tee_mem_protected_slot_init(slot, base, size, 0, 0);
#endif /* end of TEE_MEM_EMPTY_FUNCTION */

        mutex_lock(&module->protected_lock);
        list_add(&slot->list, &module->protected_list);
        mutex_unlock(&module->protected_lock);

        ret = slot;
        break;
    } while(0);

    return ret;
}

static int tee_mem_protected_destroy (struct tee_mem_api_module * module, struct tee_mem_protected_slot * slot)
{
    int ret = -1;
    do {
#ifndef TEE_MEM_EMPTY_FUNCTION
        struct tee_param                        *invoke_param = NULL;
        struct tee_ioctl_invoke_arg             tee_arg;
        struct tee_shm *                        tee_shm;
        struct tee_mem_protected_destroy_ssid * destroy_info;

        tee_shm = tee_shm_alloc(module->tee_context, sizeof (struct tee_mem_protected_destroy_ssid), TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
        if (IS_ERR(tee_shm)) {
            break;
        }

        destroy_info = (struct tee_mem_protected_destroy_ssid *) tee_shm->kaddr;
        destroy_info->ssid = slot->ssid;

	invoke_param = kcalloc(TEEC_CONFIG_PAYLOAD_REF_COUNT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param)
		return ret;

        invoke_param[0].u.memref.size = sizeof(struct tee_mem_protected_destroy_ssid);
        invoke_param[0].u.memref.shm = tee_shm;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

        memset(&tee_arg, 0, sizeof(struct tee_ioctl_invoke_arg));
        tee_arg.func = TA_TEE_MEM_PROTECTED_DESTROY_SSID;
        tee_arg.session = module->tee_session;
        tee_arg.num_params = TEEC_CONFIG_PAYLOAD_REF_COUNT;

	tee_client_invoke_func(module->tee_context, &tee_arg, invoke_param);
        if (tee_arg.ret != 0) {
            pr_info("%s%s : %d ==== tee_arg.ret:%08x ====\n", TEE_MEM_TAG, __func__, __LINE__, tee_arg.ret);
            tee_shm_free(tee_shm);
	    kfree(invoke_param);
            break;
        }

	pr_info("%s%s : %d ssid:%x\n", TEE_MEM_TAG, __func__, __LINE__, destroy_info->ssid);
        tee_shm_free(tee_shm);
	kfree(invoke_param);
#endif /* end of TEE_MEM_EMPTY_FUNCTION */

        mutex_lock(&module->protected_lock);
        list_del(&slot->list);
        mutex_unlock(&module->protected_lock);


        kfree(slot);

        ret = 0;
        break;
    } while(0);
    return ret;
}

static int tee_mem_protected_change (struct tee_mem_api_module * module, struct tee_mem_protected_slot * slot,
        unsigned long long base, unsigned long long size, unsigned int type)
{
    int ret = -1;
    do {
#ifndef TEE_MEM_EMPTY_FUNCTION
        struct tee_param                        *invoke_param = NULL;
        struct tee_ioctl_invoke_arg             tee_arg;
        struct tee_shm *                        tee_shm;
        struct tee_mem_protected_change_ssid *  change_info;

        tee_shm = tee_shm_alloc(module->tee_context, sizeof (struct tee_mem_protected_change_ssid), TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
        if (IS_ERR(tee_shm)) {
            break;
        }

        change_info = (struct tee_mem_protected_change_ssid *) tee_shm->kaddr;
        change_info->ssid = slot->ssid;
        change_info->mem.base = base;
        change_info->mem.size = size;
		change_info->mem.type = type;

	invoke_param = kcalloc(TEEC_CONFIG_PAYLOAD_REF_COUNT,
		sizeof(struct tee_param), GFP_KERNEL);

	if (!invoke_param)
		return ret;

        invoke_param[0].u.memref.size = sizeof(struct tee_mem_protected_change_ssid);
        invoke_param[0].u.memref.shm = tee_shm;

	invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

        memset(&tee_arg, 0, sizeof(struct tee_ioctl_invoke_arg));
        tee_arg.func = TA_TEE_MEM_PROTECTED_CHANGE;
        tee_arg.session = module->tee_session;
        tee_arg.num_params = TEEC_CONFIG_PAYLOAD_REF_COUNT;

	tee_client_invoke_func(module->tee_context, &tee_arg, invoke_param);

        if (tee_arg.ret != 0) {
            pr_info("%s%s : %d ==== tee_arg.ret:%08x ====\n", TEE_MEM_TAG, __func__, __LINE__, tee_arg.ret);
            tee_shm_free(tee_shm);
	    kfree(invoke_param);
            break;
        }

	pr_info("%s%s : %d ssid:%x\n", TEE_MEM_TAG, __func__, __LINE__, change_info->ssid);
        tee_shm_free(tee_shm);
	kfree(invoke_param);
#endif /* end of TEE_MEM_EMPTY_FUNCTION */

        slot->mem.base = base;
        slot->mem.size = size;


        ret = 0;
        break;
    } while(0);
    return ret;
}

static struct tee_mem_protected_ext_slot *tee_mem_protected_ext_create(struct tee_mem_api_module *module,
		unsigned long base,
		size_t size,
		unsigned int ext, struct tee_mem_protected_slot * parent)
{
	struct tee_mem_protected_ext_slot *ret = NULL;

	do {
#ifndef TEE_MEM_EMPTY_FUNCTION
		struct tee_param *invoke_param = NULL;
		struct tee_ioctl_invoke_arg tee_arg;
		struct tee_shm *tee_shm;
		struct tee_mem_protected_ext_set_ssid *create_info;
#endif /* end of TEE_MEM_EMPTY_FUNCTION */
		struct tee_mem_protected_ext_slot *slot;

		slot =
			(struct tee_mem_protected_ext_slot *)
			kzalloc(sizeof(struct tee_mem_protected_ext_slot), GFP_KERNEL);
		if (!slot)
			break;

#ifndef TEE_MEM_EMPTY_FUNCTION
		tee_shm =
			tee_shm_alloc(module->tee_context,
					sizeof(struct tee_mem_protected_ext_set_ssid),
					TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
		if (IS_ERR(tee_shm)) {
			kfree(slot);
			break;
		}

		create_info =
			(struct tee_mem_protected_ext_set_ssid *)tee_shm->kaddr;
		create_info->mem.base           = base;
		create_info->mem.size           = size;
		create_info->mem.ext            = ext;
		create_info->mem.parent_ssid    = parent->ssid;

		invoke_param = kcalloc(TEEC_CONFIG_PAYLOAD_REF_COUNT,
				sizeof(struct tee_param), GFP_KERNEL);

		if (!invoke_param)
			return NULL;

		invoke_param[0].u.memref.size =
			sizeof(struct tee_mem_protected_ext_set_ssid);
		invoke_param[0].u.memref.shm = tee_shm;

		invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
		invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
		invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
		invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

		memset(&tee_arg, 0, sizeof(struct tee_ioctl_invoke_arg));
		tee_arg.func = TA_TEE_MEM_PROTECTED_EXT_SET_SSID;
		tee_arg.session = module->tee_session;
		tee_arg.num_params = TEEC_CONFIG_PAYLOAD_REF_COUNT;

		tee_client_invoke_func(module->tee_context, &tee_arg,
				invoke_param);
		if (tee_arg.ret != 0) {
			pr_info("%s%s : %d ==== tee_arg.ret:%08x ====\n",
					TEE_MEM_TAG, __func__, __LINE__, tee_arg.ret);
			tee_shm_free(tee_shm);
			kfree(invoke_param);
			kfree(slot);
			break;
		}

		pr_info("%s%s : %d ssid:%x\n", TEE_MEM_TAG, __func__, __LINE__,
				create_info->ssid);
		tee_mem_protected_ext_slot_init(slot, base, size, create_info->ssid,
				ext, parent);
		tee_shm_free(tee_shm);
		kfree(invoke_param);
#else				/* else of TEE_MEM_EMPTY_FUNCTION */
		tee_mem_protected_ext_slot_init(slot, base, size, 0, 0, parent);
#endif				/* end of TEE_MEM_EMPTY_FUNCTION */

		mutex_lock(&module->protected_lock);
		list_add(&slot->list, &module->protected_ext_list);
		mutex_unlock(&module->protected_lock);

		ret = slot;
		break;
	} while (0);

	return ret;
}

static int tee_mem_protected_ext_destroy(struct tee_mem_api_module *module,
		struct tee_mem_protected_ext_slot *slot)
{
	int ret = -1;

	do {
#ifndef TEE_MEM_EMPTY_FUNCTION
		struct tee_param *invoke_param = NULL;
		struct tee_ioctl_invoke_arg tee_arg;
		struct tee_shm *tee_shm;
		struct tee_mem_protected_ext_unset_ssid *destroy_info;

		tee_shm =
			tee_shm_alloc(module->tee_context,
					sizeof(struct tee_mem_protected_ext_unset_ssid),
					TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
		if (IS_ERR(tee_shm))
			break;

		destroy_info =
			(struct tee_mem_protected_ext_unset_ssid *)tee_shm->kaddr;
		destroy_info->ssid = slot->ssid;

		invoke_param = kcalloc(TEEC_CONFIG_PAYLOAD_REF_COUNT,
				sizeof(struct tee_param), GFP_KERNEL);

		if (!invoke_param)
			return -1;

		invoke_param[0].u.memref.size =
			sizeof(struct tee_mem_protected_ext_unset_ssid);
		invoke_param[0].u.memref.shm = tee_shm;

		invoke_param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
		invoke_param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
		invoke_param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
		invoke_param[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

		memset(&tee_arg, 0, sizeof(struct tee_ioctl_invoke_arg));
		tee_arg.func = TA_TEE_MEM_PROTECTED_EXT_UNSET_SSID;
		tee_arg.session = module->tee_session;
		tee_arg.num_params = TEEC_CONFIG_PAYLOAD_REF_COUNT;

		tee_client_invoke_func(module->tee_context, &tee_arg,
				invoke_param);
		if (tee_arg.ret != 0) {
			pr_info("%s%s : %d ==== tee_arg.ret:%08x ====\n",
					TEE_MEM_TAG, __func__, __LINE__, tee_arg.ret);
			tee_shm_free(tee_shm);
			kfree(invoke_param);
			break;
		}

		pr_info("%s%s : %d ssid:%x\n", TEE_MEM_TAG, __func__, __LINE__,
				destroy_info->ssid);
		tee_shm_free(tee_shm);
		kfree(invoke_param);
#endif				/* end of TEE_MEM_EMPTY_FUNCTION */

		mutex_lock(&module->protected_lock);
		list_del(&slot->list);
		mutex_unlock(&module->protected_lock);

		kfree(slot);

		ret = 0;
		break;
	} while (0);
	return ret;
}

int tee_mem_api_init(void)
{
    int ret = -1;
    pr_info("%s%s init\n", TEE_MEM_TAG, __func__);
    do {
        struct tee_mem_api_module * module;
        if (gModule) {
            ret = 0;
            break;
        }

        gModule = kzalloc(sizeof(struct tee_mem_api_module), GFP_KERNEL);
        if (!gModule)
            break;

        module = gModule;

        pr_info("%s%s : %d\n", TEE_MEM_TAG, __func__, __LINE__);
        module->protected_notifier.notifier_call = protected_handler;
        INIT_LIST_HEAD(&module->protected_list);
		INIT_LIST_HEAD(&module->protected_ext_list);
        mutex_init(&module->protected_lock);

#ifndef TEE_MEM_EMPTY_FUNCTION
        {
            struct tee_ioctl_open_session_arg arg;
            struct tee_param  param[TEEC_CONFIG_PAYLOAD_REF_COUNT];

            struct tee_ioctl_version_data vers = {
                .impl_id = TEE_OPTEE_CAP_TZ,
                .impl_caps = TEE_IMPL_ID_OPTEE,
                .gen_caps = TEE_GEN_CAP_GP,
            };

	    pr_info("%s%s : %d Use PTA_MEM_UUID", TEE_MEM_TAG, __func__, __LINE__);

        pr_info("%s%s : %d\n", TEE_MEM_TAG, __func__, __LINE__);
            module->tee_context = tee_client_open_context(NULL, mem_optee_match, NULL, &vers);
        pr_info("%s%s : %d\n", TEE_MEM_TAG, __func__, __LINE__);

            if (IS_ERR(module->tee_context)) {
                pr_err("%s%s tee_client_open_context fail\n", TEE_MEM_TAG, __func__);
#if 0
                kfree(module);
                gModule = NULL;
#else
                gModule->bReady = false;
                ret = 0;
#endif
                break;
            }

            memcpy(arg.uuid, pta_mem_uuid.b, TEE_IOCTL_UUID_LEN);
            arg.clnt_login = TEEC_LOGIN_PUBLIC;
            arg.num_params = TEEC_CONFIG_PAYLOAD_REF_COUNT;
            {
                size_t n;
                for (n=0; n<TEEC_CONFIG_PAYLOAD_REF_COUNT; n++)
                    param[n].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
            }

        pr_info("%s%s : %d\n", TEE_MEM_TAG, __func__, __LINE__);
            ret = tee_client_open_session(module->tee_context, &arg, param);
        pr_info("%s%s : %d\n", TEE_MEM_TAG, __func__, __LINE__);

            if (IS_ERR(module->tee_context)) {
                pr_err("%s%s tee_client_open_session fail\n", TEE_MEM_TAG, __func__);
#if 0
                kfree(module);
                gModule = NULL;
#else
                gModule->bReady = false;
                ret = 0;
#endif
                break;
            }

            module->tee_session = arg.session;
        }
#endif /* end of TEE_MEM_EMPTY_FUNCTION */

        gModule->bReady = true;
        ret = 0;
    } while (0);

    pr_info("%s%s (%d) module:%p\n", TEE_MEM_TAG, __func__, ret, gModule);
    return ret;
}
EXPORT_SYMBOL(tee_mem_api_init);

void tee_mem_api_exit(void)
{
    if (gModule) {
        struct tee_mem_api_module * module = gModule;
        {
            struct tee_mem_protected_slot * slot, * tmp_slot;
            list_for_each_entry_safe(slot, tmp_slot, &module->protected_list, list) {
                tee_mem_protected_destroy(module, slot);
            }
        }
#ifndef TEE_MEM_EMPTY_FUNCTION
        tee_client_close_session(module->tee_context, module->tee_session);
        tee_client_close_context(module->tee_context);
#endif /* end of TEE_MEM_EMPTY_FUNCTION */
        kfree(module);
        gModule = NULL;
    }
    pr_info("%s%s\n", TEE_MEM_TAG, __func__);
}
EXPORT_SYMBOL(tee_mem_api_exit);

static int protected_handler(struct notifier_block *self,
			     unsigned long val,
			     void *data)
{
    struct tee_mem_api_module * module = container_of(self, struct tee_mem_api_module, protected_notifier);
    if (!module->bReady)
        return NOTIFY_BAD;
	switch (val) {
        case PROTECTED_REGION_CREATE:
            {
                struct ion_rtk_protected_create_info * create_info = (struct ion_rtk_protected_create_info *) data;
                struct tee_mem_protected_slot * slot = tee_mem_protected_create (module,
                        create_info->mem.base,
                        create_info->mem.size, create_info->mem.type);
                if (!slot) {
                    pr_err("%stee_mem_protected_create failed!\n", TEE_MEM_TAG);
                    return NOTIFY_BAD;
                }

                pr_info("%sPROTECTED_REGION_CREATE: %p (0x%08llx ~ 0x%08llx)\n", TEE_MEM_TAG,
                        slot, slot->mem.base, slot->mem.base + slot->mem.size);

                create_info->priv_virt = (void *) slot;
            }
            break;
        case PROTECTED_REGION_CHANGE:
            {
                struct ion_rtk_protected_change_info * change_info = (struct ion_rtk_protected_change_info *) data;
                struct tee_mem_protected_slot * slot = (struct tee_mem_protected_slot *) change_info->priv_virt;
                if (tee_mem_protected_change(module, slot, change_info->mem.base, change_info->mem.size, change_info->mem.type)) {
                    pr_err("%stee_mem_protected_change failed! %p\n", TEE_MEM_TAG, slot);
                    return NOTIFY_BAD;
                }

                pr_info("%sPROTECTED_REGION_CHANGE: %p (0x%08llx ~ 0x%08llx)\n", TEE_MEM_TAG,
                        slot, slot->mem.base, slot->mem.base + slot->mem.size);
            }
            break;
        case PROTECTED_REGION_DESTROY:
            {
                struct ion_rtk_protected_destroy_info * destroy_info = (struct ion_rtk_protected_destroy_info *) data;
                struct tee_mem_protected_slot * slot = (struct tee_mem_protected_slot *) destroy_info->priv_virt;
                if (tee_mem_protected_destroy(module, slot)) {
                    pr_err("%stee_mem_protected_destroy failed! %p\n", TEE_MEM_TAG, slot);
                    return NOTIFY_BAD;
                }

                pr_info("%sPROTECTED_REGION_DESTROY: %p\n", TEE_MEM_TAG, slot);
		}
		break;
	case PROTECTED_REGION_EXTENSION_SET:
		{
			struct ion_rtk_protected_ext_set *create_info =
				(struct ion_rtk_protected_ext_set *)data;
			struct tee_mem_protected_ext_slot *slot =
				tee_mem_protected_ext_create(module,
						create_info->mem.base,
						create_info->mem.size,
						create_info->mem.ext,
						(struct tee_mem_protected_slot *) create_info->mem.parent_priv);
			if (!slot) {
				pr_err("%stee_mem_protected_ext_create failed!\n",
						TEE_MEM_TAG);
				return NOTIFY_BAD;
			}

			pr_info
				("%sPROTECTED_REGION_EXTENSION_SET: %p (0x%08llx ~ 0x%08llx)\n",
				 TEE_MEM_TAG, slot, slot->mem.base,
				 slot->mem.base + slot->mem.size);

			create_info->priv_virt = (void *)slot;
		}
		break;
	case PROTECTED_REGION_EXTENSION_UNSET:
		{
			struct ion_rtk_protected_ext_unset *destroy_info =
				(struct ion_rtk_protected_ext_unset *)data;
			struct tee_mem_protected_ext_slot *slot =
				(struct tee_mem_protected_ext_slot *)destroy_info->priv_virt;

			if (tee_mem_protected_ext_destroy(module, slot)) {
				pr_err
					("%stee_mem_protected_ext_destroy failed! %p\n",
					 TEE_MEM_TAG, slot);
				return NOTIFY_BAD;
			}

			pr_info("%sPROTECTED_REGION_EXTENSION_UNSET: %p\n", TEE_MEM_TAG,
				slot);
		}
		break;
	default:
		return NOTIFY_BAD;
	}
	return NOTIFY_OK;
}

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
