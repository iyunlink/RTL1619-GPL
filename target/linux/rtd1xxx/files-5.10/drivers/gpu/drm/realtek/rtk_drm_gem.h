/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019 RealTek Inc.
 */

#ifndef _RTK_DRM_GEM_H
#define _RTK_DRM_GEM_H

#include <drm/drm_gem.h>

#define USE_ION
struct rtk_gem_object {
	struct drm_gem_object base;

#ifdef USE_ION
	struct dma_buf *dmabuf;
#endif
	struct sg_table *sgt;

	void *vaddr;
	phys_addr_t paddr;
};

#define to_rtk_gem_obj(x) container_of(x, struct rtk_gem_object, base)

extern const struct vm_operations_struct rtk_gem_vm_ops;

struct drm_gem_object *
rtk_gem_prime_import_sg_table(struct drm_device *dev,
			      struct dma_buf_attachment *attach,
			      struct sg_table *sg);

struct sg_table *rtk_gem_prime_get_sg_table(struct drm_gem_object *obj);

int rtk_gem_dumb_create(struct drm_file *file_priv,
			struct drm_device *drm,
			struct drm_mode_create_dumb *args);
int rtk_gem_prime_mmap(struct drm_gem_object *gem_obj,
		      struct vm_area_struct *vma);
void *rtk_gem_prime_vmap(struct drm_gem_object *gem_obj);
void rtk_gem_prime_vunmap(struct drm_gem_object *gem_obj, void *vaddr);
int rtk_gem_dumb_map_offset(struct drm_file *file_priv,
			    struct drm_device *dev, uint32_t handle,
			    uint64_t *offset);
int rtk_gem_mmap(struct file *filp, struct vm_area_struct *vma);
void rtk_gem_free_object(struct drm_gem_object *gem_obj);
int rtk_gem_info_debugfs(struct seq_file *m, void *unused);

#endif  /* _RTK_DRM_GEM_H_ */
