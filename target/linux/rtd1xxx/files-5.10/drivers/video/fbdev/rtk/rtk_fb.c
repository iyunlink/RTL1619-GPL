// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek framebuffer driver
 *
 * Copyright (c) 2019-2020 Realtek Semiconductor Corp.
 */

#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/fdtable.h>
#include <linux/gfp.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/ion.h>
#include <asm/div64.h>
#include <soc/realtek/uapi/ion_rtk.h>

#include "debug.h"
#include "rtk_fb.h"
#include "rtk_fb_rpc.h"

#include <ion_rtk_alloc.h>
#define ion_alloc ext_rtk_ion_alloc

int osd_init_status;

#ifdef CONFIG_REALTEK_AVCPU
static int fb_avcpu_event_notify(struct notifier_block *self,
	unsigned long action, void *data)
{
	struct fb_info *info = (struct fb_info *) registered_fb[0];
	struct rtk_fb *fb = container_of(info, struct rtk_fb, fb);

	return dc_avcpu_event_notify(action, &fb->video_info);
}

static struct notifier_block fb_avcpu_event_notifier = {
	.notifier_call = fb_avcpu_event_notify,
};
#endif

static inline u32 convert_bitfield(int val, struct fb_bitfield *bf)
{
	unsigned int mask = (1 << bf->length) - 1;

	return (val >> (16 - bf->length) & mask) << bf->offset;
}

/* set the software color map.  Probably doesn't need modifying. */
static int rtk_fb_set_color_reg(unsigned int regno,
				unsigned int red,
				unsigned int green,
				unsigned int blue,
				unsigned int transp,
				struct fb_info *info)
{
	struct rtk_fb *fb = container_of(info, struct rtk_fb, fb);

	if (regno < 16) {
		fb->cmap[regno] =
			convert_bitfield(transp, &fb->fb.var.transp) |
			convert_bitfield(blue, &fb->fb.var.blue) |
			convert_bitfield(green, &fb->fb.var.green) |
			convert_bitfield(red, &fb->fb.var.red);
		return 0;
	} else
		return 1;
}

/* check var to see if supported by this device.  Probably doesn't
 * need modifying.
 */
static int rtk_fb_check_var(struct fb_var_screeninfo *var,
			    struct fb_info *info)
{
	struct rtk_fb *fb = container_of(info, struct rtk_fb, fb);

	if (info->var.xres_virtual > fb->mem->width)
		info->var.xres_virtual = fb->mem->width;
	if (info->var.yres_virtual > (fb->mem->height * fb->mem->count))
		info->var.yres_virtual = fb->mem->height * fb->mem->count;

	return 0;
}

static int rtk_fb_set_par(struct fb_info *info)
{
	struct rtk_fb *fb = container_of(info, struct rtk_fb, fb);
	struct rtk_fb_mem *mem = fb->mem;

	if (fb->rotation != fb->fb.var.rotate) {
		dbg_info("[%s %d] TODO: FB_ROTATE\n", __func__, __LINE__);
		info->fix.line_length = info->var.xres * mem->pixel;
		fb->rotation = fb->fb.var.rotate;
	}

	if (info->var.xres_virtual > mem->width)
		info->var.xres_virtual = mem->width;

	if (info->var.yres_virtual > (mem->height * mem->count))
		info->var.yres_virtual = mem->height * mem->count;

	return 0;
}

static int rtk_fb_pan_display(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	struct rtk_fb *fb __attribute__ ((unused)) =
		container_of(info, struct rtk_fb, fb);

	/*
	 * Set the frame buffer base to something like:
	 * fb->fb.fix.smem_start + fb->fb.var.xres *
	 * BYTES_PER_PIXEL * var->yoffset
	 */
	fb->fb.var.xoffset = var->xoffset;
	fb->fb.var.yoffset = var->yoffset;
	fb->fb.var.xres_virtual = var->xres_virtual;
	fb->fb.var.yres_virtual = var->yres_virtual;

	return dc_swap_buffer(info, &fb->video_info);
}

static int rtk_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct rtk_fb *fb __attribute__ ((unused)) =
		container_of(info, struct rtk_fb, fb);
	unsigned long start;
	unsigned long mmio_pgoff;
	u32 len;

	start = info->fix.smem_start;
	len = info->fix.smem_len;
	mmio_pgoff = PAGE_ALIGN((start & ~PAGE_MASK) + len) >> PAGE_SHIFT;

	if (vma->vm_pgoff >= mmio_pgoff) {

		if (info->var.accel_flags)
			return -EINVAL;

		vma->vm_pgoff -= mmio_pgoff;
		start = info->fix.mmio_start;
		len = info->fix.mmio_len;
	}

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	return vm_iomap_memory(vma, start, len);
}

void rtk_fb_ion_free(struct rtk_fb *rtk_fb)
{
	struct device *dev = rtk_fb->dev;
	struct dma_buf *dma_buf = rtk_fb->fb_dmabuf;
	struct dma_buf_attachment *attachment = rtk_fb->attachment;
	struct sg_table *sgt = rtk_fb->sgt;

	dev_err(dev, "free ion buffer\n");

	dma_buf_vunmap(dma_buf, rtk_fb->mem->va_addr);
	dma_buf_end_cpu_access(dma_buf, DMA_BIDIRECTIONAL);
	dma_buf_unmap_attachment(attachment, sgt, DMA_TO_DEVICE);
	dma_buf_detach(dma_buf, attachment);
	dma_buf_put(dma_buf);
}

int rtk_fb_mem_alloc_ion(struct rtk_fb *rtk_fb, int pixel, int flags)
{
	int ret = 0;
	size_t size;
	u32 ion_heap_mask = RTK_ION_HEAP_MEDIA_MASK;
	u32 ion_heap_flag = ION_FLAG_SCPUACC | ION_FLAG_HWIPACC;
	struct rtk_fb_mem *mem = rtk_fb->mem;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	struct device *dev = rtk_fb->dev;

	size = MEMORY_ALIGN((mem->width * mem->height * mem->count * pixel),
		 PAGE_SIZE);

	if (flags & RTK_FB_MEM_ALLOC_ALGO_LAST_FIT)
		ion_heap_flag |= ION_USAGE_ALGO_LAST_FIT;

	dma_buf = ion_alloc(size, ion_heap_mask, ion_heap_flag);
	if (IS_ERR(dma_buf)) {
		ret = -ENOMEM;
		goto err;
	}

	attachment = dma_buf_attach(dma_buf, dev);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		dev_err(dev, "failed to attach dma-buf: %d\n", ret);
		goto put_dma_buf;
	}

	sgt = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		dev_err(dev, "failed to map attachment: %d\n", ret);
		goto detach_dma_buf;
	}

	if (sgt->nents != 1) {
		ret = -EINVAL;
		dev_err(dev, "scatter list not supportted\n");
		goto unmap_attachment;
	}

	dma_addr = sg_dma_address(sgt->sgl);
	if (!dma_addr) {
		ret = -ENOMEM;
		dev_err(dev, "invalid dma address\n");
		goto unmap_attachment;
	}

	dma_buf_begin_cpu_access(dma_buf, DMA_BIDIRECTIONAL);

	mem->va_addr = dma_buf_vmap(dma_buf);
	if (!mem->va_addr)
		return -ENOMEM;

	mem->size = size;
	mem->align = PAGE_SIZE;
	mem->pixel = pixel;
	mem->flags = flags;
	mem->pa_addr = dma_addr;

	rtk_fb->fb_dmabuf = dma_buf;
	rtk_fb->sgt = sgt;
	rtk_fb->attachment = attachment;

	dev_info(rtk_fb->dev, "width = %d\n", mem->width);
	dev_info(rtk_fb->dev, "height = %d\n", mem->height);
	dev_info(rtk_fb->dev, "count = %d\n", mem->count);
	dev_info(rtk_fb->dev, "pixel = %d\n", pixel);
	dev_info(rtk_fb->dev, "size = %ld\n", size);
	dev_info(rtk_fb->dev, "va_addr = 0x%lx\n",
		(unsigned long) mem->va_addr);
	dev_info(rtk_fb->dev, "pa_addr = 0x%lx\n",
		(unsigned long) mem->pa_addr);

	ret = rtk_fb_config(rtk_fb);

	if (ret < 0)
		dev_err(rtk_fb->dev, "frame buffer config error\n");

	return ret;

unmap_attachment:
	dma_buf_unmap_attachment(attachment, sgt, DMA_TO_DEVICE);
detach_dma_buf:
	dma_buf_detach(dma_buf, attachment);
put_dma_buf:
	dma_buf_put(dma_buf);
err:
	return ret;
}

void rtk_fb_mem_destroy(struct rtk_fb *rtk_fb)
{
	if (rtk_fb->mem)
		return;

	kfree(rtk_fb->mem);
}

static int rtk_fb_set_size(struct rtk_fb *fb)
{
	int ret = 0;

	rtk_fb_ion_free(fb);

	ret = rtk_fb_mem_alloc_ion(fb, fb->mem->pixel,
		RTK_FB_MEM_ALLOC_ALGO_LAST_FIT);
	if (ret < 0) {
		dev_info(fb->dev, "frame buffer config error\n");
		goto err;
	}
err:
	return ret;
}

static int rtk_fb_chk_size(struct rtk_fb *fb, struct fb_resize size)
{
	struct rtk_fb_mem *mem = fb->mem;
	int ret = -1;

	ret = (mem->width ^ size.width) + (mem->height ^ size.height) +
		(mem->count ^ size.count) + (mem->pixel ^ size.pixel);

	return ret;
}

static int rtk_fb_ioctl(struct fb_info *info, unsigned int cmd,
	unsigned long arg)
{
	struct rtk_fb *fb = container_of(info, struct rtk_fb, fb);
	int gat_cmd = 0;
	int ret = 0;
	struct fb_resize size;
	bool skip_lock = false;
	struct rtk_fb_mem *mem = fb->mem;
	struct fb_dmabuf_export dmabuf_export;
#if !defined(CONFIG_ARM)
	struct fb_dmabuf_export __user *up_dmabuf_export =
		(struct fb_dmabuf_export __user *) arg;
#endif

	switch (cmd) {
	case FBIOSET_RESIZE:

		dev_info(fb->dev, "FBIOSET_RESIZE\n");

		if (copy_from_user(&size, (void *)arg, sizeof(size)) == 0) {
			if (size.flags & RTK_FB_MEM_FREE_FB) {
				rtk_fb_ion_free(fb);
			} else {
				dev_info(fb->dev, "new size:\n");
				dev_info(fb->dev, "size.width = %d\n", size.width);
				dev_info(fb->dev, "size.height = %d\n", size.height);
				dev_info(fb->dev, "size.count = %d\n", size.count);
				dev_info(fb->dev, "size.pixel = %d\n", size.pixel);

				dev_info(fb->dev, "old size:\n");
				dev_info(fb->dev, "mem->width = %d\n", mem->width);
				dev_info(fb->dev, "mem->height = %d\n", mem->height);
				dev_info(fb->dev, "mem->count = %d\n", mem->count);
				dev_info(fb->dev, "mem->pixel = %d\n", mem->pixel);

				ret = rtk_fb_chk_size(fb, size);
				if (ret) {
				    mem->width = size.width;
				    mem->height = size.height;
				    mem->count = size.count;
				    mem->pixel = size.pixel;
				    fb->mem = mem;

				    ret = rtk_fb_set_size(fb);
				    if (ret < 0)
					break;
				}
			}
		}

		ret = 0;
		gat_cmd = 1;
		break;
	case FBIOGET_DMABUF:

		dev_info(fb->dev, "FBIOGET_DMABUF\n");

		dmabuf_export.flags = 0;
		dmabuf_export.fd = (__u32) rtk_fb_ion_fd(fb);
#if defined(CONFIG_ARM)
		ret = copy_to_user((void __user *) arg,
			&dmabuf_export, sizeof(dmabuf_export));
#else
		ret = put_user(dmabuf_export, up_dmabuf_export);
#endif /* CONFIG_ARM */

		gat_cmd = 1;
		break;
	default:
		break;
	}

	if (gat_cmd)
		goto exit;

	if (mutex_is_locked(&info->lock))
		skip_lock = true;

	if (skip_lock)
		unlock_fb_info(info);

	ret = dc_ioctl(info, (void *)&fb->video_info, cmd, arg);

#if 0 /* DAH-232 : Don't lock to avoid deadlock problems */
	if (skip_lock)
		lock_fb_info(info);
#endif
exit:
	return ret;
}

int rtk_fb_ion_fd(struct rtk_fb *fb)
{
	int ret = -1;
	struct dma_buf *dmabuf;

	if (fb->mem == NULL)
		goto err;

	fb->mem->handle = dma_buf_fd(fb->fb_dmabuf, O_CLOEXEC);
	if (fb->mem->handle < 0) {
		dev_info(fb->dev, "dma buffer failed to export (%d)\n",
			 fb->mem->handle);
		dma_buf_put(fb->fb_dmabuf);
		return fb->mem->handle;
	}

	dmabuf = dma_buf_get(fb->mem->handle);
	if (IS_ERR(dmabuf)) {
		dev_info(fb->dev, "Invalid dmabuf FD\n");
		return PTR_ERR(dmabuf);
	}

	dev_info(fb->dev, "ion fd = %d\n", fb->mem->handle);

	ret = fb->mem->handle;

err:
	return ret;
}

static struct fb_ops rtk_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = rtk_fb_check_var,
	.fb_set_par = rtk_fb_set_par,
	.fb_setcolreg = rtk_fb_set_color_reg,
	.fb_pan_display = rtk_fb_pan_display,
	.fb_ioctl = rtk_fb_ioctl,
	.fb_mmap = rtk_fb_mmap,
#ifdef CONFIG_COMPAT
	.fb_compat_ioctl = rtk_fb_ioctl,
#endif
	/* These are generic software based fb functions */
#ifdef CONFIG_FB_CFB_FILLRECT
	.fb_fillrect = cfb_fillrect,
#endif

#ifdef CONFIG_FB_CFB_COPYAREA
	.fb_copyarea = cfb_copyarea,
#endif

#ifdef CONFIG_FB_CFB_IMAGEBLIT
	.fb_imageblit = cfb_imageblit,
#endif
};

int rtk_fb_config(struct rtk_fb *fb)
{
	int ret = 0;
	int fb_cnt;
	unsigned long long temp;
	unsigned long long temp2;
	struct rtk_fb_mem *mem;

	mem = fb->mem;

	fb->fb.flags = FBINFO_FLAG_DEFAULT;
	fb->fb.pseudo_palette = fb->cmap;
	fb->fb.fix.type = FB_TYPE_PACKED_PIXELS;
	fb->fb.fix.visual = FB_VISUAL_TRUECOLOR;
	fb->fb.fix.line_length = mem->width * mem->pixel;
	fb->fb.fix.accel = FB_ACCEL_NONE;
	fb->fb.fix.ypanstep = 1;

	fb_cnt = mem->size / (fb->fb.fix.line_length * mem->height);

	if (fb_cnt > mem->count)
		fb_cnt = mem->count;

	dev_info(fb->dev, "number of buffers %d\n", fb_cnt);

	fb->fb.var.xres = mem->width;
	fb->fb.var.yres = mem->height;
	fb->fb.var.xres_virtual = mem->width;
	fb->fb.var.yres_virtual = mem->height * fb_cnt;
	fb->fb.var.activate = FB_ACTIVATE_NOW;
	fb->fb.var.height = mem->height;
	fb->fb.var.width = mem->width;

	if (mem->pixel == 4) {
		/* ARGB 8888 */
		fb->fb.var.bits_per_pixel = 32;
		fb->fb.var.transp.offset = 24;
		fb->fb.var.transp.length = 8;
		fb->fb.var.red.offset = 16;
		fb->fb.var.red.length = 8;
		fb->fb.var.green.offset = 8;
		fb->fb.var.green.length = 8;
		fb->fb.var.blue.offset = 0;
		fb->fb.var.blue.length = 8;
	} else {
		/* RGB 565 */
		fb->fb.var.bits_per_pixel = 16;
		fb->fb.var.red.offset = 11;
		fb->fb.var.red.length = 5;
		fb->fb.var.green.offset = 5;
		fb->fb.var.green.length = 6;
		fb->fb.var.blue.offset = 0;
		fb->fb.var.blue.length = 5;
	}

	fb->fb.var.width = 0;
	fb->fb.var.height = 0;

	switch (mem->width) {
	case 1920:
	case 720:
		fb->fb.var.vsync_len = 1;
		fb->fb.var.hsync_len = 2;
		break;
	case 1280:
		fb->fb.var.vsync_len = 11;
		fb->fb.var.hsync_len = 3;
	default:
		break;
	}

	temp = (fb->fb.var.xres + fb->fb.var.vsync_len) *
		(fb->fb.var.yres + fb->fb.var.hsync_len);
	temp2 = 1000000000000;
	do_div(temp2, temp);
	fb->fb.var.pixclock = temp2;
	fb->fb.var.pixclock /= fb->fps;
	fb->fb.fix.smem_len = mem->size;
	fb->fb.fix.smem_start = mem->pa_addr;
	fb->fb.screen_base = mem->va_addr;

	if (!fb->fb.screen_base) {
		dev_err(fb->dev, "could not allocate frame buffer memory");
		ret = -ENOMEM;
		goto err;
	}

	if (fb_set_var(&fb->fb, &fb->fb.var) != 0) {
		ret = -ENOMEM;
		goto err;
	}

err:
	return ret;
}

struct rtk_fb *rtk_fb_ex;

static int rtk_fb_probe(struct platform_device *pdev)
{
	int ret = 0;
	const u32 *prop;
	struct rtk_fb *rtk_fb;
	struct device_node *syscon_np;

	dev_info(&pdev->dev, "%s\n", __func__);

	rtk_fb = kzalloc(sizeof(struct rtk_fb), GFP_KERNEL);
	if (!rtk_fb) {
		ret = -ENOMEM;
		goto err;
	}

	rtk_fb_ex = rtk_fb;

	rtk_fb->dev = &pdev->dev;

	rtk_fb->mem = kzalloc(sizeof(struct rtk_fb_mem), GFP_KERNEL);
	if (!rtk_fb->mem) {
		ret = -ENOMEM;
		goto err;
	}

	prop = of_get_property(pdev->dev.of_node, "resolution", NULL);
	if (prop) {
		rtk_fb->mem->width = of_read_number(prop, 1);
		rtk_fb->mem->height = of_read_number(prop, 2);
	} else {
		rtk_fb->mem->width = DEFAULT_WIDTH;
		rtk_fb->mem->height = DEFAULT_HEIGHT;
	}

	prop = of_get_property(pdev->dev.of_node, "buffer-cnt", NULL);
	if (prop)
		rtk_fb->mem->count = of_read_number(prop, 1);
	else
		rtk_fb->mem->count = DEFAULT_COUNT;

	prop = of_get_property(pdev->dev.of_node, "fps", NULL);
	if (prop)
		rtk_fb->fps = of_read_number(prop, 1);
	else
		rtk_fb->fps = DEFAULT_FPS;

	prop = of_get_property(pdev->dev.of_node, "osd-init", NULL);
	if (prop)
		osd_init_status = of_read_number(prop, 1);
	else
		osd_init_status = DEFAULT_OSD_INIT;

	rtk_fb->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);

	dev_info(&pdev->dev, "IRQ number = %d\n", rtk_fb->irq);

	rtk_fb_mem_alloc_ion(rtk_fb, BYTES_PER_PIXEL,
		RTK_FB_MEM_ALLOC_ALGO_LAST_FIT);

	syscon_np = of_parse_phandle(pdev->dev.of_node, "syscon", 0);
	if (IS_ERR_OR_NULL(syscon_np)) {
		dev_err(&pdev->dev, "Not found syscon property\n");
		ret = -ENODEV;
		goto err;
	}

	rtk_fb->video_info.acpu_int_base = syscon_node_to_regmap(syscon_np);
	if (IS_ERR_OR_NULL(rtk_fb->video_info.acpu_int_base)) {
		dev_err(&pdev->dev, "Cannot get parent's regmap\n");
		ret = PTR_ERR(rtk_fb->video_info.acpu_int_base);
		goto err;
	}

	dc_init(&rtk_fb->video_info, &rtk_fb->fb, rtk_fb->irq);

	prop = of_get_property(pdev->dev.of_node, "skip-front", NULL);
	if (prop) {
		int count = of_read_number(prop, 1);
		dc_set_skip(&rtk_fb->video_info, count);
	}

	rtk_fb->fb.fbops = &rtk_fb_ops;

	dev_set_drvdata(&pdev->dev, rtk_fb);

	if (register_framebuffer(&rtk_fb->fb))
		goto err;

#ifdef CONFIG_REALTEK_AVCPU
	register_avcpu_notifier(&fb_avcpu_event_notifier);
#endif

	if (osd_init_status == 1) {
		if(RTK_FB_RPC_OSD_init(rtk_fb)!= 0)
			goto err;
	}

	dc2vo_sysfs_init();

	return 0;

err:
	dev_err(&pdev->dev, "probe fail\n");

	return ret;
}

static int rtk_fb_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_fb *fb = platform_get_drvdata(pdev);

	return dc_suspend(&fb->video_info);
}

void rtk_fb_pause(void)
{
	dc_suspend(&rtk_fb_ex->video_info);
}
EXPORT_SYMBOL(rtk_fb_pause);

void rtk_fb_run(void)
{
	dc_resume(&rtk_fb_ex->video_info);
}
EXPORT_SYMBOL(rtk_fb_run);

static int rtk_fb_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtk_fb *fb = platform_get_drvdata(pdev);

	return dc_resume(&fb->video_info);
}

static int rtk_fb_remove(struct platform_device *pdev)
{
	struct rtk_fb *fb = platform_get_drvdata(pdev);

	if (osd_init_status == 1) {
		RTK_FB_RPC_OSD_uninit();
	}

#ifdef CONFIG_REALTEK_AVCPU
	unregister_avcpu_notifier(&fb_avcpu_event_notifier);
#endif
	dc_deinit(&fb->video_info);
	unregister_framebuffer(&fb->fb);

	if (fb->mem != NULL)
		rtk_fb_mem_destroy(fb);

	if (fb != NULL)
		kfree(fb);

	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

const static struct of_device_id rtk_fb_ids[] = {
	{ .compatible = "realtek,framebuffer"},
	{ /* Sentinel */ },
};

MODULE_DEVICE_TABLE(of, rtk_fb_ids);

static const struct dev_pm_ops rtk_fb_pm_ops = {
	.suspend = rtk_fb_suspend,
	.resume  = rtk_fb_resume,
};

static struct platform_driver rtkfb_of_driver = {
	.probe  = rtk_fb_probe,
	.remove	= rtk_fb_remove,
	.driver = {
		.name = "realtek-framebuffer",
		.owner = THIS_MODULE,
		.of_match_table = rtk_fb_ids,
#ifdef CONFIG_PM
		.pm = &rtk_fb_pm_ops,
#endif
	},
};

static int rtk_fb_init(void)
{
	return platform_driver_register(&rtkfb_of_driver);
}
late_initcall(rtk_fb_init);

MODULE_LICENSE("GPL v2");
