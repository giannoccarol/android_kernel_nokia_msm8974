/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */
#ifndef __MSM_PIL_SSD_H
#define __MSM_PIL_SSD_H

#include <linux/device.h>
#include <linux/types.h>

struct pil_ssd_ctx {
	bool active;
	u32 id;
	int seg_base;	/* program header of seg_num 0 */
	int last_seg;	/* last program header with data */
};

/* Each returns 0 without changes when the image is not SSD encrypted. */
int pil_ssd_decrypt_image(struct device *dev, phys_addr_t pa, size_t len);
int pil_ssd_parse_md(struct device *dev, const u8 *mdt, size_t size,
		     struct pil_ssd_ctx *ctx);
int pil_ssd_decrypt_seg(struct device *dev, struct pil_ssd_ctx *ctx, int num,
			phys_addr_t pa, size_t len);

#endif
