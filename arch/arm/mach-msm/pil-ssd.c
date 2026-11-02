/*
 * Secure Software Download (SSD) decryption of PIL images through the
 * TrustZone, for the Nokia Lumia 1520.
 *
 * The stock MBA and modem images are encrypted with SSD (AES-128-CBC, key
 * wrapped in an XML metadata block). The Windows Phone TrustZone keeps the
 * keystore the UEFI loads and exposes SCM service 7:
 *  - 0x1c01 decrypts a whole image in place and moves {ptr, len} past the
 *    metadata (MBA);
 *  - 0x1c06 parses the metadata of an ELF image and returns a context;
 *  - 0x1c08 decrypts one fragment of one ELF segment. seg_num counts from
 *    the first segment after the hash segment: the TZ derives the CBC IV of
 *    each segment from it. last_seg on the final fragment of the final
 *    segment closes the context.
 * The plain text was checked against the signed hash table of the image
 * (every segment matches). Data passes through a cached bounce buffer in
 * Linux memory, as in the LK SSD helpers.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/elf.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/cacheflush.h>
#include <asm/outercache.h>
#include <mach/scm.h>

#include "pil-ssd.h"

#define SSD_DECRYPT_IMG			0x01
#define SSD_PARSE_MD			0x06
#define SSD_DECRYPT_ELF_SEG_FRAG	0x08

#define SSD_BOUNCE_SIZE			SZ_256K
#define SSD_MD_TAG			"<?xml"

static DEFINE_MUTEX(ssd_lock);
static void *bounce;
static phys_addr_t bounce_pa;

/* Kept for the life of the system: modem restarts need it again. */
static int ssd_get_bounce(void)
{
	if (bounce)
		return 0;
	bounce = alloc_pages_exact(SSD_BOUNCE_SIZE, GFP_KERNEL | __GFP_NOWARN);
	if (!bounce)
		return -ENOMEM;
	bounce_pa = virt_to_phys(bounce);
	return 0;
}

static void ssd_sync(void *virt, phys_addr_t phys, size_t len)
{
	dmac_flush_range(virt, virt + len);
	outer_flush_range(phys, phys + len);
}

int pil_ssd_decrypt_image(struct device *dev, phys_addr_t pa, size_t len)
{
	struct {
		u32 img_ptr_pa;
		u32 img_len_ptr_pa;
	} req;
	u32 *ptrs;
	void __iomem *map;
	u32 off, new_len;
	char tag[sizeof(SSD_MD_TAG) - 1];
	int ret;

	map = ioremap_wc(pa, len);
	if (!map)
		return -ENOMEM;

	memcpy_fromio(tag, map, sizeof(tag));
	if (memcmp(tag, SSD_MD_TAG, sizeof(tag))) {
		iounmap(map);
		return 0;	/* not encrypted */
	}

	ptrs = kzalloc(2 * sizeof(u32), GFP_KERNEL);
	if (!ptrs) {
		iounmap(map);
		return -ENOMEM;
	}

	mutex_lock(&ssd_lock);
	ret = ssd_get_bounce();
	if (ret)
		goto out;
	if (len > SSD_BOUNCE_SIZE) {
		ret = -EFBIG;
		goto out;
	}

	memcpy_fromio(bounce, map, len);
	ssd_sync(bounce, bounce_pa, len);
	ptrs[0] = bounce_pa;
	ptrs[1] = len;
	ssd_sync(ptrs, virt_to_phys(ptrs), 2 * sizeof(u32));

	req.img_ptr_pa = virt_to_phys(&ptrs[0]);
	req.img_len_ptr_pa = virt_to_phys(&ptrs[1]);
	ret = scm_call(SCM_SVC_SSD, SSD_DECRYPT_IMG, &req, sizeof(req),
		       NULL, 0);

	ssd_sync(ptrs, virt_to_phys(ptrs), 2 * sizeof(u32));
	ssd_sync(bounce, bounce_pa, len);
	off = ptrs[0] - (u32)bounce_pa;
	new_len = ptrs[1];
	if (ret || off > len || new_len > len - off) {
		dev_err(dev, "SSD image decryption failed: %d (+0x%x, 0x%x)\n",
			ret, off, new_len);
		ret = ret ? ret : -EINVAL;
		goto out;
	}

	/* The plain text goes where the image starts; the rest is cleared. */
	memcpy_toio(map, (u8 *)bounce + off, new_len);
	memset_io(map + new_len, 0, len - new_len);
	wmb();
	dev_info(dev, "SSD image decrypted: 0x%zx -> 0x%x bytes\n", len,
		 new_len);
out:
	mutex_unlock(&ssd_lock);
	kfree(ptrs);
	iounmap(map);
	return ret;
}

int pil_ssd_parse_md(struct device *dev, const u8 *mdt, size_t size,
		     struct pil_ssd_ctx *ctx)
{
	const struct elf32_hdr *ehdr = (const struct elf32_hdr *)mdt;
	const struct elf32_phdr *phdr;
	struct {
		u32 md_len;
		u32 md_pa;
	} req;
	struct {
		u32 status;
		u32 md_ctx_id;
		u32 md_end_pa;
	} rsp;
	const char *xml;
	size_t xml_len;
	int i, hash = -1, ret;

	ctx->active = false;

	xml = strnstr((const char *)mdt, SSD_MD_TAG, size);
	if (!xml)
		return 0;	/* not encrypted */
	xml_len = size - ((const u8 *)xml - mdt);

	if (size < sizeof(*ehdr) || memcmp(ehdr->e_ident, ELFMAG, SELFMAG) ||
	    ehdr->e_phoff + ehdr->e_phnum * sizeof(*phdr) > size)
		return -EINVAL;
	phdr = (const struct elf32_phdr *)(mdt + ehdr->e_phoff);

	ctx->last_seg = -1;
	for (i = 0; i < ehdr->e_phnum; i++) {
		if (((phdr[i].p_flags >> 24) & 7) == 2) {
			hash = i;
			continue;
		}
		if (hash >= 0 && phdr[i].p_type == PT_LOAD && phdr[i].p_filesz)
			ctx->last_seg = i;
	}
	if (hash < 0 || ctx->last_seg < 0)
		return -EINVAL;
	ctx->seg_base = hash + 1;

	mutex_lock(&ssd_lock);
	ret = ssd_get_bounce();
	if (ret)
		goto out;
	if (xml_len > SSD_BOUNCE_SIZE) {
		ret = -EFBIG;
		goto out;
	}

	memcpy(bounce, xml, xml_len);
	ssd_sync(bounce, bounce_pa, xml_len);
	req.md_len = xml_len;
	req.md_pa = bounce_pa;
	memset(&rsp, 0, sizeof(rsp));
	ret = scm_call(SCM_SVC_SSD, SSD_PARSE_MD, &req, sizeof(req),
		       &rsp, sizeof(rsp));
	if (ret || rsp.status) {
		dev_err(dev, "SSD metadata rejected: %d, status %u\n", ret,
			rsp.status);
		ret = ret ? ret : -EINVAL;
		goto out;
	}

	ctx->id = rsp.md_ctx_id;
	ctx->active = true;
	dev_info(dev, "SSD metadata parsed: context %u, segments %d..%d\n",
		 ctx->id, ctx->seg_base, ctx->last_seg);
out:
	mutex_unlock(&ssd_lock);
	return ret;
}

int pil_ssd_decrypt_seg(struct device *dev, struct pil_ssd_ctx *ctx, int num,
			phys_addr_t pa, size_t len)
{
	struct {
		u32 md_ctx_id;
		u32 seg_num;
		u32 last_seg;
		u32 last_frag;
		u32 frag_len;
		u32 frag_pa;
	} req;
	struct {
		u32 status;
	} rsp;
	size_t off, flen;
	void __iomem *map;
	int ret = 0;

	if (!ctx->active || num < ctx->seg_base || num > ctx->last_seg || !len)
		return 0;

	mutex_lock(&ssd_lock);
	ret = ssd_get_bounce();
	if (ret)
		goto out;

	for (off = 0; off < len; off += flen) {
		flen = min_t(size_t, SSD_BOUNCE_SIZE, len - off);
		map = ioremap_wc(pa + off, flen);
		if (!map) {
			ret = -ENOMEM;
			goto out;
		}
		memcpy_fromio(bounce, map, flen);
		ssd_sync(bounce, bounce_pa, flen);

		req.md_ctx_id = ctx->id;
		req.seg_num = num - ctx->seg_base;
		req.last_seg = (num == ctx->last_seg);
		req.last_frag = (off + flen == len);
		req.frag_len = flen;
		req.frag_pa = bounce_pa;
		rsp.status = 0;
		ret = scm_call(SCM_SVC_SSD, SSD_DECRYPT_ELF_SEG_FRAG,
			       &req, sizeof(req), &rsp, sizeof(rsp));
		ssd_sync(bounce, bounce_pa, flen);
		if (ret || rsp.status) {
			iounmap(map);
			dev_err(dev, "SSD segment %d at 0x%zx: %d, status %u\n",
				num, off, ret, rsp.status);
			ret = ret ? ret : -EINVAL;
			ctx->active = false;
			goto out;
		}
		memcpy_toio(map, bounce, flen);
		wmb();
		iounmap(map);
	}

	if (num == ctx->last_seg) {
		ctx->active = false;
		dev_info(dev, "SSD segments decrypted\n");
	}
out:
	mutex_unlock(&ssd_lock);
	return ret;
}
