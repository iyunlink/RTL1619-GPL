// SPDX-License-Identifier: GPL-2.0-only
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "hse.h"
#include "uapi/hse.h"

struct hse_dev_file_data {
	struct hse_device *hse_dev;
	struct hse_command_queue *cq;
};

static inline struct device *fdata2dev(struct hse_dev_file_data *data)
{
	return data->hse_dev->dev;
}

static int check_file_perm(struct file *file, unsigned int f_flags)
{
	unsigned int flags;

	if (!file)
		return -EBADF;

	flags = file->f_flags & O_ACCMODE;

	if (flags == O_RDWR)
		return 0;

	return f_flags == flags ? 0 : -EPERM;
}

struct hse_dev_dma_buf {
	struct dma_buf_attachment   *attachment;
	struct sg_table             *sgt;
	dma_addr_t                  dma_addr;
	unsigned int                dma_len;
};

static int hse_dev_import_buf(struct hse_device *hse_dev, struct hse_dev_dma_buf *hse_buf, int fd, unsigned int f_flags)
{
	struct device *dev = hse_dev->dev;
	struct dma_buf *buf = dma_buf_get(fd);
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	int ret = 0;

	if (IS_ERR(buf))
		return PTR_ERR(buf);

	ret = check_file_perm(buf->file, f_flags);
	if (ret) {
		dev_err(dev, "%s: invalid permission\n", __func__);
		goto put_dma_buf;
	}

	attach = dma_buf_attach(buf, dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		dev_err(dev, "%s: cannot attach\n", __func__);
		goto put_dma_buf;
	}

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		dev_err(dev, "%s: cannot map attachment\n", __func__);
		goto detach_dma_buf;
	}

	if (sgt->nents != 1) {
		ret = -EINVAL;
		dev_err(dev, "%s: scatter list not supportted\n", __func__);
		goto detach_dma_buf;
	}

	dma_addr = sg_dma_address(sgt->sgl);
	if (!dma_addr) {
		ret = -EINVAL;
		dev_err(dev, "%s: invalid dma address\n", __func__);
		goto detach_dma_buf;
	}

	hse_buf->attachment = attach;
	hse_buf->sgt        = sgt;
	hse_buf->dma_addr   = dma_addr;
	hse_buf->dma_len    = sg_dma_len(sgt->sgl);
	return 0;

detach_dma_buf:
	dma_buf_detach(buf, attach);
put_dma_buf:
	dma_buf_put(buf);
	return ret;
}

static void hse_dev_release_dmabuf(struct hse_dev_dma_buf *hse_buf)
{
	struct dma_buf_attachment *attach = hse_buf->attachment;
	struct dma_buf *buf;

	if (hse_buf->sgt)
		dma_buf_unmap_attachment(attach, hse_buf->sgt, DMA_BIDIRECTIONAL);

	buf = attach->dmabuf;
	dma_buf_detach(buf, attach);
	dma_buf_put(buf);
}

static inline int validate_copy_args(uint32_t dma_len, uint32_t offset, uint32_t size)
{
	if (check_add_overflow(offset, size, &size))
		return -EINVAL;
	return dma_len < size ? -EINVAL : 0;
}

struct hse_dev_cq_cbdata {
	struct completion c;
};

static void hse_dev_complete_cb(void *p)
{
	struct hse_dev_cq_cbdata *cb_data = p;

	complete(&cb_data->c);
}

static int hse_dev_wait_timeout(struct hse_dev_cq_cbdata *cb_data, int timeout_ms)
{
	int ret;

	ret = wait_for_completion_timeout(&cb_data->c, msecs_to_jiffies(timeout_ms));
	if (ret == 0)
		ret = -ETIMEDOUT;
	return ret < 0 ? ret : 0;
}

static int hse_dev_submit_cq(struct hse_device *hse_dev, struct hse_command_queue *cq)
{
	struct hse_engine *eng = &hse_dev->eng;
	struct hse_dev_cq_cbdata cb_data = { 0 };
	int ret;

	init_completion(&cb_data.c);

	hse_cq_set_complete_callback(cq, hse_dev_complete_cb, &cb_data);

	hse_engine_add_cq(eng, cq);

	hse_engine_issue_cq(eng);

	ret = hse_dev_wait_timeout(&cb_data, 500);
	if (ret)
		hse_engine_remove_cq(eng, cq);
	else {
		if (cq->status & ~HSE_STATUS_IRQ_OK)
			ret = -EFAULT;

		else if (cq->status & HSE_STATUS_IRQ_OK)
			ret = 0;
	}

	return ret;
}

static int hse_dev_ioctl_cmd_copy(struct hse_dev_file_data *fdata, struct hse_cmd_copy *cmd)
{
	struct hse_dev_dma_buf src = { 0 }, dst = { 0 };
	int ret;
	dma_addr_t dst_addr, src_addr;
	uint32_t dst_size, src_size;

	dst_size = src_size = cmd->size;

	ret = hse_dev_import_buf(fdata->hse_dev, &dst, cmd->dst_fd, O_WRONLY);
	if (ret) {
		dev_dbg(fdata2dev(fdata), "failed to import dst: %d\n", ret);
		return ret;
	}

	ret = validate_copy_args(dst.dma_len, cmd->dst_offset, dst_size);
	if (ret) {
		dev_dbg(fdata2dev(fdata), "failed to validate dst\n");
		goto release_dst;
	}

	dst_addr = dst.dma_addr + cmd->dst_offset;

	ret = hse_dev_import_buf(fdata->hse_dev, &src, cmd->src_fd, O_RDONLY);
	if (ret) {
		dev_dbg(fdata2dev(fdata), "failed to import src: %d\n", ret);
		goto release_dst;
	}

	ret = validate_copy_args(src.dma_len, cmd->src_offset, src_size);
	if (ret) {
		dev_dbg(fdata2dev(fdata), "failed to validate src\n");
		goto release_src;
	}

	src_addr = src.dma_addr + cmd->src_offset;

	ret = hse_cq_prep_copy(fdata->hse_dev, fdata->cq, dst_addr, src_addr, cmd->size);
	if (!ret)
		ret = hse_dev_submit_cq(fdata->hse_dev, fdata->cq);
	hse_cq_reset(fdata->cq);

release_src:
	hse_dev_release_dmabuf(&src);
release_dst:
	hse_dev_release_dmabuf(&dst);
	return ret;
}

static int hse_dev_ioctl_cmd_xor(struct hse_dev_file_data *fdata, struct hse_cmd_xor *cmd)
{
	struct hse_dev_dma_buf dst = { 0 }, src[HSE_XOR_NUM] = { 0 };
	int ret;
	dma_addr_t dst_addr, src_addr[HSE_XOR_NUM];
	uint32_t dst_size, src_size;
	int i;

	dst_size = src_size = cmd->size;

	ret = hse_dev_import_buf(fdata->hse_dev, &dst, cmd->dst_fd, O_WRONLY);
	if (ret) {
		dev_dbg(fdata2dev(fdata), "failed to import dst: %d\n", ret);
		return ret;
	}

	ret = validate_copy_args(dst.dma_len, cmd->dst_offset, dst_size);
	if (ret) {
		dev_dbg(fdata2dev(fdata), "failed to validate dst\n");
		goto release_dst;
	}

	dst_addr = dst.dma_addr + cmd->dst_offset;

	for (i = 0; i < cmd->src_num; i++) {
		ret = hse_dev_import_buf(fdata->hse_dev, &src[i], cmd->src_fd[i], O_RDONLY);
		if (ret) {
			dev_dbg(fdata2dev(fdata), "failed to import src%d: %d\n", i, ret);
			goto release_src;
		}

		ret = validate_copy_args(src[i].dma_len, cmd->src_offset[i], src_size);
		if (ret) {
			dev_dbg(fdata2dev(fdata), "failed to validate src%d\n", i);
			goto release_src;
		}

		src_addr[i] = src[i].dma_addr + cmd->src_offset[i];
	}

	ret = hse_cq_prep_xor(fdata->hse_dev, fdata->cq, dst_addr, src_addr, cmd->src_num, cmd->size);
	if (!ret)
		ret = hse_dev_submit_cq(fdata->hse_dev, fdata->cq);
	hse_cq_reset(fdata->cq);

release_src:
	for (i--; i >= 0; i--)
		hse_dev_release_dmabuf(&src[i]);
release_dst:
	hse_dev_release_dmabuf(&dst);
	return ret;
}

static int hse_dev_ioctl_cmd_constant_fill(struct hse_dev_file_data *fdata, struct hse_cmd_constant_fill *cmd)
{
	struct hse_dev_dma_buf dst = { 0 };
	int ret;
	dma_addr_t dst_addr;
	uint32_t dst_size;

	dst_size = cmd->size;

	ret = hse_dev_import_buf(fdata->hse_dev, &dst, cmd->dst_fd, O_WRONLY);
	if (ret) {
		dev_dbg(fdata2dev(fdata), "failed to import dst: %d\n", ret);
		return ret;
	}

	ret = validate_copy_args(dst.dma_len, cmd->dst_offset, dst_size);
	if (ret) {
		dev_dbg(fdata2dev(fdata), "failed to validate dst\n");
		goto release_dst;
	}

	dst_addr = dst.dma_addr + cmd->dst_offset;

	ret = hse_cq_prep_constant_fill(fdata->hse_dev, fdata->cq, dst_addr, cmd->val, cmd->size);
	if (!ret)
		ret = hse_dev_submit_cq(fdata->hse_dev, fdata->cq);
	hse_cq_reset(fdata->cq);

release_dst:
	hse_dev_release_dmabuf(&dst);
	return ret;
}

static int hse_dev_open(struct inode *inode, struct file *filp)
{
	struct hse_device *hse_dev = container_of(filp->private_data,
		struct hse_device, mdev);
	struct hse_dev_file_data *data;
	int ret;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->hse_dev = hse_dev;
	data->cq = hse_cq_alloc(hse_dev);
	if (!data->cq) {
		ret = -ENOMEM;
		goto free_data;
	}

	filp->private_data = data;
	return 0;
free_data:
	kfree(data);
	return ret;
}

static int hse_dev_release(struct inode *inode, struct file *filp)
{
	struct hse_dev_file_data *data = filp->private_data;
	struct hse_command_queue *cq = data->cq;

	hse_cq_free(cq);
	kfree(data);
	return 0;
}

static long hse_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct hse_dev_file_data *data = filp->private_data;
	struct hse_command_queue *cq = data->cq;
	int ret;
	union {
		struct hse_cmd cmd;
		struct hse_cmd_copy copy;
		struct hse_cmd_constant_fill constant_fill;
		struct hse_cmd_xor xor;
	} args;

	switch (cmd) {
	case HSE_IOCTL_ADD_CMD:
		if (copy_from_user(&args.cmd, (unsigned int __user *)arg, sizeof(args.cmd)))
			return -EFAULT;

		hse_cq_add_data(cq, args.cmd.cmds, args.cmd.size);
		break;

	case HSE_IOCTL_START:
		ret = hse_dev_submit_cq(data->hse_dev, cq);

		return ret;

	case HSE_IOCTL_CMD_COPY:
		if (copy_from_user(&args.copy, (unsigned int __user *)arg, sizeof(args.copy)))
			return -EFAULT;

		return hse_dev_ioctl_cmd_copy(data, &args.copy);

	case HSE_IOCTL_CMD_CONSTANT_FILL:
		if (copy_from_user(&args.constant_fill, (unsigned int __user *)arg, sizeof(args.constant_fill)))
			return -EFAULT;

		return hse_dev_ioctl_cmd_constant_fill(data, &args.constant_fill);

	case HSE_IOCTL_CMD_XOR:
		if (copy_from_user(&args.xor, (unsigned int __user *)arg, sizeof(args.xor)))
			return -EFAULT;

		return hse_dev_ioctl_cmd_xor(data, &args.xor);

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

static const struct file_operations hse_dev_fops = {
	.owner          = THIS_MODULE,
	.open           = hse_dev_open,
	.unlocked_ioctl = hse_dev_ioctl,
	.compat_ioctl   = compat_ptr_ioctl,
	.release        = hse_dev_release,
};

int hse_setup_miscdevice(struct hse_device *hse_dev)
{
	hse_dev->mdev.minor  = MISC_DYNAMIC_MINOR;
	hse_dev->mdev.name   = "hse";
	hse_dev->mdev.fops   = &hse_dev_fops;
	hse_dev->mdev.parent = NULL;
	return misc_register(&hse_dev->mdev);
}

void hse_teardown_miscdevice(struct hse_device *hse_dev)
{
	misc_deregister(&hse_dev->mdev);
}
