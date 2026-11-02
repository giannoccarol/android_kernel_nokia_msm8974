/*
 * Lumia 1520 test helper: decrypt the SSD-encrypted modem firmware through
 * the Windows Phone TrustZone, without starting the modem.
 *
 * The stock MBA (mba.b00) and modem (modem.bNN) images are encrypted with
 * Qualcomm Secure Software Download (AES-128-CBC, key wrapped in the SSD
 * metadata). The TZ keeps the keystore loaded by the UEFI and exposes the
 * SSD service (SCM service 7). This file only calls it and keeps the result
 * in memory, readable through debugfs, so the plain text can be checked
 * against the hash table of the image on a PC.
 *
 *   /sys/kernel/debug/bandit_ssd/avail   read: SCM availability of 0x1c01/06/07/08
 *   /sys/kernel/debug/bandit_ssd/mba     write: decrypt mba.b00 with 0x1c01
 *   /sys/kernel/debug/bandit_ssd/modem   write: parse modem.mdt metadata (0x1c06),
 *                                         decrypt each segment (0x1c08)
 *   /sys/kernel/debug/bandit_ssd/index   read: what is in "out" and the status
 *   /sys/kernel/debug/bandit_ssd/out     read: the plain text
 *
 * Call layouts follow the LK SSD helpers (platform/msm_shared/scm.c) and the
 * argument counts in the TZ syscall table (kit/modem/MODEM-STUDIO.md).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/gfp.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <asm/cacheflush.h>
#include <asm/outercache.h>
#include <mach/scm.h>
#include <mach/tz-restricted.h>

#define SSD_DECRYPT_IMG			0x01
#define SSD_PARSE_MD			0x06
#define SSD_DECRYPT_IMG_FRAG		0x07
#define SSD_DECRYPT_ELF_SEG_FRAG	0x08

#define MODEM_MAX_SEGS			64
#define INDEX_SIZE			8192

static unsigned int frag_size = SZ_256K;
module_param(frag_size, uint, 0644);
MODULE_PARM_DESC(frag_size, "fragment size for 0x1c08 (multiple of 16)");

/*
 * seg_num counts from the first segment after the ELF header and hash
 * segments: program header 19 is seg_num 17 (found with the IV probe, the
 * plain text of segment 19 starts with a zero block).
 */
static unsigned int seg_num_base = 2;
module_param(seg_num_base, uint, 0644);

static struct platform_device *ssd_pdev;
static DEFINE_MUTEX(ssd_lock);
static u8 *out_buf;
static size_t out_len;
static char *index_buf;
static size_t index_len;

static void index_add(const char *fmt, ...)
{
	va_list args;

	if (index_len >= INDEX_SIZE - 1)
		return;
	va_start(args, fmt);
	index_len += vscnprintf(index_buf + index_len, INDEX_SIZE - index_len,
				fmt, args);
	va_end(args);
}

/* Physically contiguous, cacheable buffer handed to the TZ. */
struct tz_buf {
	void *virt;
	phys_addr_t phys;
	size_t size;
};

static int tz_buf_alloc(struct tz_buf *b, size_t size)
{
	b->size = PAGE_ALIGN(size);
	b->virt = alloc_pages_exact(b->size, GFP_KERNEL | __GFP_NOWARN);
	if (!b->virt)
		return -ENOMEM;
	b->phys = virt_to_phys(b->virt);
	return 0;
}

static void tz_buf_free(struct tz_buf *b)
{
	if (b->virt)
		free_pages_exact(b->virt, b->size);
	b->virt = NULL;
}

/* Clean and invalidate, before and after the TZ touches the memory. */
static void tz_buf_sync(struct tz_buf *b, size_t len)
{
	dmac_flush_range(b->virt, b->virt + len);
	outer_flush_range(b->phys, b->phys + len);
}

static int out_reserve(size_t len)
{
	vfree(out_buf);
	out_len = 0;
	out_buf = vzalloc(len);
	return out_buf ? 0 : -ENOMEM;
}

/* ---------------------------------------------------------------- avail */

static int avail_show(struct seq_file *s, void *unused)
{
	static const u32 cmds[] = { SSD_DECRYPT_IMG, 0x02, 0x05, SSD_PARSE_MD,
				    SSD_DECRYPT_IMG_FRAG,
				    SSD_DECRYPT_ELF_SEG_FRAG };
	int i;

	for (i = 0; i < ARRAY_SIZE(cmds); i++)
		seq_printf(s, "ssd 0x%04x: %d\n", (SCM_SVC_SSD << 10) | cmds[i],
			   scm_is_call_available(SCM_SVC_SSD, cmds[i]));
	seq_printf(s, "pil 0x0807 (is_subsystem_supported): %d\n",
		   scm_is_call_available(SCM_SVC_PIL, 7));
	return 0;
}

static int avail_open(struct inode *inode, struct file *file)
{
	return single_open(file, avail_show, NULL);
}

static const struct file_operations avail_fops = {
	.open		= avail_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* ------------------------------------------------------------------ MBA */

struct ssd_img_req {
	u32 img_ptr_pa;		/* PA of a u32 holding the image PA */
	u32 img_len_ptr_pa;	/* PA of a u32 holding the image length */
};

static int ssd_decrypt_mba(void)
{
	const struct firmware *fw;
	struct tz_buf img = { 0 }, ptrs = { 0 };
	struct ssd_img_req req;
	u32 *p;
	u32 new_pa, new_len;
	int ret, scm_ret;

	ret = request_firmware(&fw, "mba.b00", &ssd_pdev->dev);
	if (ret) {
		index_add("mba: request_firmware failed %d\n", ret);
		return ret;
	}

	ret = tz_buf_alloc(&img, fw->size);
	if (!ret)
		ret = tz_buf_alloc(&ptrs, PAGE_SIZE);
	if (ret) {
		index_add("mba: no memory\n");
		goto out;
	}

	memcpy(img.virt, fw->data, fw->size);
	p = ptrs.virt;
	p[0] = img.phys;
	p[1] = fw->size;
	req.img_ptr_pa = ptrs.phys;
	req.img_len_ptr_pa = ptrs.phys + sizeof(u32);

	tz_buf_sync(&img, fw->size);
	tz_buf_sync(&ptrs, 2 * sizeof(u32));

	pr_info("bandit_ssd: mba: 0x1c01, image %zu bytes at %pa\n",
		fw->size, &img.phys);
	scm_ret = scm_call(SCM_SVC_SSD, SSD_DECRYPT_IMG, &req, sizeof(req),
			   NULL, 0);

	tz_buf_sync(&ptrs, 2 * sizeof(u32));
	tz_buf_sync(&img, fw->size);
	new_pa = p[0];
	new_len = p[1];

	pr_info("bandit_ssd: mba: scm %d, ptr 0x%08x (+0x%x), len 0x%x\n",
		scm_ret, new_pa, new_pa - (u32)img.phys, new_len);
	index_add("mba: scm %d img_pa 0x%08x ptr 0x%08x (+0x%x) len 0x%x (was 0x%zx)\n",
		  scm_ret, (u32)img.phys, new_pa, new_pa - (u32)img.phys,
		  new_len, fw->size);

	/* Keep the whole buffer: the plain text starts at the new pointer. */
	ret = out_reserve(fw->size);
	if (!ret) {
		memcpy(out_buf, img.virt, fw->size);
		out_len = fw->size;
		index_add("out: whole mba buffer, %zu bytes\n", out_len);
		print_hex_dump(KERN_INFO, "bandit_ssd: mba+ptr: ",
			       DUMP_PREFIX_OFFSET, 16, 4,
			       (u8 *)img.virt +
			       min_t(u32, new_pa - (u32)img.phys, fw->size - 80),
			       80, false);
	}
	ret = scm_ret;
out:
	tz_buf_free(&ptrs);
	tz_buf_free(&img);
	release_firmware(fw);
	return ret;
}

/* ---------------------------------------------------------------- modem */

struct ssd_parse_md_req {
	u32 md_len;
	u32 md_pa;
};

struct ssd_parse_md_rsp {
	u32 status;
	u32 md_ctx_id;
	u32 md_end_pa;
};

struct ssd_seg_frag_req {
	u32 md_ctx_id;
	u32 seg_num;
	u32 last_seg;
	u32 last_frag;
	u32 frag_len;
	u32 frag_pa;
};

struct ssd_frag_rsp {
	u32 status;
};

/*
 * IV probe: the TZ derives the CBC IV of each segment from seg_num. Decrypt
 * the first 4 KB of one segment with seg_num = probe_from..probe_to, using
 * one metadata context, and log the first block of each result. Segment 19
 * starts with zeros in plain text, so the right seg_num gives a zero block.
 */
static unsigned int probe_seg = 19;
module_param(probe_seg, uint, 0644);
static unsigned int probe_from;
module_param(probe_from, uint, 0644);
static unsigned int probe_to = 40;
module_param(probe_to, uint, 0644);
static unsigned int probe_last_seg = 1;
module_param(probe_last_seg, uint, 0644);

static void bin2hex_str(char *dst, const u8 *src)
{
	int i;

	for (i = 0; i < 16; i++)
		sprintf(dst + 2 * i, "%02x", src[i]);
}

static int ssd_probe_iv(void)
{
	const struct firmware *mdt = NULL, *seg = NULL;
	struct tz_buf md = { 0 }, frag = { 0 };
	struct ssd_parse_md_req preq;
	struct ssd_parse_md_rsp prsp;
	const u8 *xml;
	size_t xml_off, len;
	char name[16];
	unsigned int n;
	int ret, scm_ret;

	ret = request_firmware(&mdt, "modem.mdt", &ssd_pdev->dev);
	if (ret)
		return ret;
	snprintf(name, sizeof(name), "modem.b%02u", probe_seg);
	ret = request_firmware(&seg, name, &ssd_pdev->dev);
	if (ret)
		goto out;

	xml = strnstr(mdt->data, "<?xml", mdt->size);
	if (!xml) {
		ret = -ENOENT;
		goto out;
	}
	xml_off = xml - mdt->data;
	len = min_t(size_t, seg->size & ~15, SZ_4K);

	ret = tz_buf_alloc(&md, mdt->size - xml_off);
	if (!ret)
		ret = tz_buf_alloc(&frag, SZ_4K);
	if (ret)
		goto out;

	memcpy(md.virt, xml, mdt->size - xml_off);
	tz_buf_sync(&md, mdt->size - xml_off);
	preq.md_len = mdt->size - xml_off;
	preq.md_pa = md.phys;
	memset(&prsp, 0, sizeof(prsp));
	scm_ret = scm_call(SCM_SVC_SSD, SSD_PARSE_MD, &preq, sizeof(preq),
			   &prsp, sizeof(prsp));
	index_add("probe: parse_md scm %d status %u ctx %u\n", scm_ret,
		  prsp.status, prsp.md_ctx_id);
	if (scm_ret || prsp.status) {
		ret = -EIO;
		goto out;
	}

	for (n = probe_from; n <= probe_to; n++) {
		struct ssd_seg_frag_req freq;
		struct ssd_frag_rsp frsp = { 0 };
		u8 *b = frag.virt;

		memcpy(frag.virt, seg->data, len);
		tz_buf_sync(&frag, len);
		freq.md_ctx_id = prsp.md_ctx_id;
		freq.seg_num = n;
		freq.last_seg = probe_last_seg;
		freq.last_frag = 1;
		freq.frag_len = len;
		freq.frag_pa = frag.phys;
		scm_ret = scm_call(SCM_SVC_SSD, SSD_DECRYPT_ELF_SEG_FRAG, &freq,
				   sizeof(freq), &frsp, sizeof(frsp));
		tz_buf_sync(&frag, len);
		{
			char h1[33], h2[33];

			bin2hex_str(h1, b);
			bin2hex_str(h2, b + 16);
			index_add("probe seg %u seg_num %2u: scm %d status %u first %s second %s\n",
				  probe_seg, n, scm_ret, frsp.status, h1, h2);
		}
	}
	ret = 0;
out:
	tz_buf_free(&frag);
	tz_buf_free(&md);
	release_firmware(seg);
	release_firmware(mdt);
	return ret;
}

static int ssd_decrypt_modem(void)
{
	const struct firmware *mdt = NULL, *seg;
	struct tz_buf md = { 0 }, frag = { 0 };
	struct ssd_parse_md_req preq;
	struct ssd_parse_md_rsp prsp;
	const struct elf32_hdr *ehdr;
	const struct elf32_phdr *phdr;
	size_t total = 0, pos = 0, xml_off;
	const u8 *xml;
	int i, last_seg = -1, ret, scm_ret;
	u32 ctx;

	ret = request_firmware(&mdt, "modem.mdt", &ssd_pdev->dev);
	if (ret) {
		index_add("modem: request_firmware(mdt) failed %d\n", ret);
		return ret;
	}

	ehdr = (const struct elf32_hdr *)mdt->data;
	if (mdt->size < sizeof(*ehdr) ||
	    memcmp(ehdr->e_ident, ELFMAG, SELFMAG) ||
	    ehdr->e_phnum > MODEM_MAX_SEGS ||
	    ehdr->e_phoff + ehdr->e_phnum * sizeof(*phdr) > mdt->size) {
		index_add("modem: bad mdt\n");
		ret = -EINVAL;
		goto out;
	}
	phdr = (const struct elf32_phdr *)(mdt->data + ehdr->e_phoff);

	/* Encrypted segments: data segments other than the ELF/hash ones. */
	for (i = 0; i < ehdr->e_phnum; i++) {
		bool hash = (phdr[i].p_flags & (7 << 24)) == (2 << 24);

		if (phdr[i].p_type != PT_LOAD || !phdr[i].p_filesz || hash ||
		    i < 2)
			continue;
		total += phdr[i].p_filesz;
		last_seg = i;
	}

	xml = strnstr(mdt->data, "<?xml", mdt->size);
	if (!xml) {
		index_add("modem: no SSD metadata in mdt\n");
		ret = -ENOENT;
		goto out;
	}
	xml_off = xml - mdt->data;

	ret = tz_buf_alloc(&md, mdt->size - xml_off);
	if (!ret)
		ret = tz_buf_alloc(&frag, frag_size);
	if (!ret)
		ret = out_reserve(total);
	if (ret) {
		index_add("modem: no memory\n");
		goto out;
	}

	memcpy(md.virt, xml, mdt->size - xml_off);
	tz_buf_sync(&md, mdt->size - xml_off);
	preq.md_len = mdt->size - xml_off;
	preq.md_pa = md.phys;
	memset(&prsp, 0, sizeof(prsp));
	scm_ret = scm_call(SCM_SVC_SSD, SSD_PARSE_MD, &preq, sizeof(preq),
			   &prsp, sizeof(prsp));
	pr_info("bandit_ssd: modem: 0x1c06 md at mdt+0x%zx len 0x%x: scm %d status %u ctx %u end +0x%x\n",
		xml_off, preq.md_len, scm_ret, prsp.status, prsp.md_ctx_id,
		prsp.md_end_pa - (u32)md.phys);
	index_add("modem: parse_md xml_off 0x%zx len 0x%x scm %d status %u ctx %u end +0x%x\n",
		  xml_off, preq.md_len, scm_ret, prsp.status, prsp.md_ctx_id,
		  prsp.md_end_pa - (u32)md.phys);
	if (scm_ret || prsp.status) {
		ret = scm_ret ? scm_ret : -EIO;
		goto out;
	}
	ctx = prsp.md_ctx_id;

	for (i = 2; i <= last_seg; i++) {
		char name[16];
		size_t off;
		bool hash = (phdr[i].p_flags & (7 << 24)) == (2 << 24);
		u32 status = 0;

		if (phdr[i].p_type != PT_LOAD || !phdr[i].p_filesz || hash)
			continue;

		snprintf(name, sizeof(name), "modem.b%02d", i);
		ret = request_firmware(&seg, name, &ssd_pdev->dev);
		if (ret || seg->size != phdr[i].p_filesz) {
			index_add("seg %d: request_firmware %d size %zu/%u\n",
				  i, ret, ret ? 0 : seg->size,
				  phdr[i].p_filesz);
			if (!ret)
				release_firmware(seg);
			ret = ret ? ret : -EINVAL;
			goto out;
		}

		for (off = 0; off < seg->size; off += frag_size) {
			struct ssd_seg_frag_req freq;
			struct ssd_frag_rsp frsp = { 0 };
			size_t len = min_t(size_t, frag_size, seg->size - off);

			memcpy(frag.virt, seg->data + off, len);
			tz_buf_sync(&frag, len);
			freq.md_ctx_id = ctx;
			freq.seg_num = i - seg_num_base;
			freq.last_seg = (i == last_seg);
			freq.last_frag = (off + len == seg->size);
			freq.frag_len = len;
			freq.frag_pa = frag.phys;
			scm_ret = scm_call(SCM_SVC_SSD,
					   SSD_DECRYPT_ELF_SEG_FRAG,
					   &freq, sizeof(freq),
					   &frsp, sizeof(frsp));
			tz_buf_sync(&frag, len);
			if (scm_ret || frsp.status) {
				pr_err("bandit_ssd: seg %d off 0x%zx: scm %d status %u\n",
				       i, off, scm_ret, frsp.status);
				status = scm_ret ? (u32)scm_ret : frsp.status;
				memcpy(out_buf + pos + off, frag.virt, len);
				break;
			}
			memcpy(out_buf + pos + off, frag.virt, len);
		}
		index_add("seg %d out 0x%zx len 0x%x paddr 0x%08x status %u\n",
			  i, pos, phdr[i].p_filesz, phdr[i].p_paddr, status);
		pr_info("bandit_ssd: seg %d: %u bytes, status %u\n", i,
			phdr[i].p_filesz, status);
		pos += seg->size;
		out_len = pos;
		release_firmware(seg);
		if (status) {
			ret = -EIO;
			goto out;
		}
	}
	index_add("out: %zu bytes of modem segments 2..%d\n", out_len,
		  last_seg);
	ret = 0;
out:
	tz_buf_free(&frag);
	tz_buf_free(&md);
	release_firmware(mdt);
	return ret;
}

/* -------------------------------------------------------- debugfs glue */

static ssize_t run_write(struct file *file, const char __user *ubuf,
			 size_t count, loff_t *ppos)
{
	int (*fn)(void) = file->private_data;
	int ret;

	mutex_lock(&ssd_lock);
	index_len = 0;
	index_buf[0] = 0;
	ret = fn();
	mutex_unlock(&ssd_lock);
	pr_info("bandit_ssd: done: %d\n", ret);

	return count;
}

static int run_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations run_fops = {
	.open	= run_open,
	.write	= run_write,
};

static ssize_t index_read(struct file *file, char __user *ubuf, size_t count,
			  loff_t *ppos)
{
	ssize_t ret;

	mutex_lock(&ssd_lock);
	ret = simple_read_from_buffer(ubuf, count, ppos, index_buf, index_len);
	mutex_unlock(&ssd_lock);
	return ret;
}

static const struct file_operations index_fops = {
	.read	= index_read,
};

static ssize_t out_read(struct file *file, char __user *ubuf, size_t count,
			loff_t *ppos)
{
	ssize_t ret;

	mutex_lock(&ssd_lock);
	ret = out_buf ? simple_read_from_buffer(ubuf, count, ppos, out_buf,
						out_len) : 0;
	mutex_unlock(&ssd_lock);
	return ret;
}

static const struct file_operations out_fops = {
	.read	= out_read,
};

static int __init bandit_ssd_init(void)
{
	struct dentry *dir;

	if (!msm_tz_restricted())
		return 0;

	index_buf = kzalloc(INDEX_SIZE, GFP_KERNEL);
	if (!index_buf)
		return -ENOMEM;

	ssd_pdev = platform_device_register_simple("bandit_ssd", -1, NULL, 0);
	if (IS_ERR(ssd_pdev))
		return PTR_ERR(ssd_pdev);

	dir = debugfs_create_dir("bandit_ssd", NULL);
	if (IS_ERR_OR_NULL(dir))
		return -ENODEV;
	debugfs_create_file("avail", 0400, dir, NULL, &avail_fops);
	debugfs_create_file("mba", 0200, dir, ssd_decrypt_mba, &run_fops);
	debugfs_create_file("modem", 0200, dir, ssd_decrypt_modem, &run_fops);
	debugfs_create_file("probe", 0200, dir, ssd_probe_iv, &run_fops);
	debugfs_create_file("index", 0400, dir, NULL, &index_fops);
	debugfs_create_file("out", 0400, dir, NULL, &out_fops);
	pr_info("bandit_ssd: ready\n");
	return 0;
}
late_initcall(bandit_ssd_init);
