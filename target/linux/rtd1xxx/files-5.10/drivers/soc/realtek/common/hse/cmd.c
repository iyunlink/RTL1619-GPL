// SPDX-License-Identifier: GPL-2.0-only
#include "hse.h"

#define COPY_SIZE_MAX    (518144)

static int __add_copy(struct hse_command_queue *cq,
		      uint32_t dst, uint32_t src, uint32_t size)
{
	uint32_t cmds[4];

	cmds[0] = 0x1;
	cmds[1] = size;
	cmds[2] = dst;
	cmds[3] = src;
	return hse_cq_add_data(cq, cmds, 4);
}

static int __add_constant_fill_swap(struct hse_command_queue *cq,
				    uint32_t dst, uint32_t val, uint32_t size,
				    uint32_t swap_opt)
{
	uint32_t cmds[4];

	cmds[0] = 0x1 | (1 << 19) | ((swap_opt & 0x1f) << 9);
	cmds[1] = size;
	cmds[2] = dst;
	cmds[3] = val;
	return hse_cq_add_data(cq, cmds, 4);
}

static int __add_constant_fill(struct hse_command_queue *cq,
			       uint32_t dst, uint32_t val, uint32_t size)
{
	return __add_constant_fill_swap(cq, dst, val, size, 0);
}

static int hse_cq_prep_copy_multiple(struct hse_device *hse_dev, struct hse_command_queue *cq,
				     dma_addr_t dst, dma_addr_t src, uint32_t size)
{
	uint32_t c_size = size;
	uint32_t c_dst_addr = dst;
	uint32_t c_src_addr = src;
	int ret;

	int should_split = (c_src_addr % 16) || (c_dst_addr % 16) || (c_size % 16);

	while (should_split && c_size > COPY_SIZE_MAX) {
		ret = __add_copy(cq, c_dst_addr, c_src_addr, COPY_SIZE_MAX);
		if (ret)
			return ret;
		c_size -= COPY_SIZE_MAX;
		c_dst_addr += COPY_SIZE_MAX;
		c_src_addr += COPY_SIZE_MAX;
	}

	should_split = should_split && (c_size > 16);

	if (should_split) {
		uint32_t new_size;

		if (c_size <= 0x20)
			new_size = 0x10;
		else if (c_size <= 0x800)
			new_size = 0x20;
		else
			new_size = 0x800;

		ret = __add_copy(cq, c_dst_addr, c_src_addr, c_size & ~(new_size - 1));
		ret |= __add_copy(cq, c_dst_addr + c_size - new_size,
				  c_src_addr + c_size - new_size,
				  new_size);
		if (ret)
			return ret;
		c_size = 0;
	}

	if (c_size != 0)
		ret = __add_copy(cq, c_dst_addr, c_src_addr, c_size);
	return ret;
}

int hse_cq_prep_copy(struct hse_device *hse_dev, struct hse_command_queue *cq,
		    dma_addr_t dst, dma_addr_t src, uint32_t size)
{
	if (hse_should_workaround_copy(hse_dev))
		return hse_cq_prep_copy_multiple(hse_dev, cq, dst, src, size);

	return __add_copy(cq, dst, src, size);
}

static int hse_cq_prep_constant_fill_multiple(struct hse_device *hse_dev,
					     struct hse_command_queue *cq,
					     dma_addr_t dst, uint32_t val, uint32_t size)
{
	uint32_t split_size = 16777216;
	uint32_t c_dst_addr = dst;
	uint32_t c_size = size;
	int ret;
	const int swap_opt[] = {0, 18, 16, 9};

	if (size < 32)
		return -EINVAL;

	while (c_size >= 64) {
		if (c_size < split_size) {
			split_size = split_size == 8192 ? 2048 : split_size / 2;
			continue;
		}

		ret = __add_constant_fill(cq, c_dst_addr, val, split_size);
		if (ret)
			return ret;

		c_dst_addr += split_size;
		c_size     -= split_size;
	}

	if (c_size >= 32)
		return __add_constant_fill(cq, c_dst_addr, val, c_size);

	if (c_size == 0)
		return 0;

	return __add_constant_fill_swap(cq, c_dst_addr + c_size - 32,
					val, 32, swap_opt[c_size & 0x3]);
}

int hse_cq_prep_constant_fill(struct hse_device *hse_dev, struct hse_command_queue *cq,
			     dma_addr_t dst, uint32_t val, uint32_t size)
{
	if (hse_should_workaround_copy(hse_dev))
		return hse_cq_prep_constant_fill_multiple(hse_dev, cq, dst, val, size);

	return __add_constant_fill(cq, dst, val, size);
}

static int __add_xor(struct hse_command_queue *cq, uint32_t dst, uint32_t *src, uint32_t src_cnt, uint32_t size)
{
	uint32_t cmds[8];
	int i;

	cmds[0] = 0x1 | ((src_cnt - 1) << 16);
	cmds[1] = size;
	cmds[2] = dst;
	for (i = 0; i < src_cnt; i++)
		cmds[i + 3] = src[i];
	return hse_cq_add_data(cq, cmds, i + 3);
}

int hse_cq_prep_xor(struct hse_device *hse_dev, struct hse_command_queue *cq,
		    dma_addr_t dst, dma_addr_t *src, uint32_t src_cnt, uint32_t size)
{
	uint32_t s[5];
	int i;

	if (src_cnt > 5)
		return -EINVAL;

	for (i = 0; i < src_cnt; i++)
		s[i] = src[i];

	return __add_xor(cq, dst, s, src_cnt, size);
}
