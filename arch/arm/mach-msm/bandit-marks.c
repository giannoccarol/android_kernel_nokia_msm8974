/*
 * Nokia Lumia 1520 (bandit) bring-up: coloured boot progress bands on the
 * framebuffer the Windows Phone UEFI leaves running. See asm/bandit-mark.h.
 *
 * Bands 1-3 are painted from assembly with the MMU off; from paging_init()
 * on, this file maps the framebuffer statically (iotable, so vmalloc never
 * reuses the range) and paints the later bands and the panic band.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/io.h>
#include <asm/mach/map.h>
#include <asm/bandit-mark.h>

static bool bandit_fb_mapped;

static void bandit_fill(unsigned int row, unsigned int rows, unsigned int color)
{
	u32 __iomem *p;
	unsigned int i, n;

	if (!bandit_fb_mapped)
		return;
	p = (u32 __iomem *)(BANDIT_FB_VIRT + row * BANDIT_FB_STRIDE);
	n = rows * BANDIT_FB_STRIDE / 4;
	for (i = 0; i < n; i++)
		__raw_writel(color, p + i);
	wmb();
}

void bandit_mark(int n, unsigned int color)
{
	bandit_fill(n * BANDIT_MARK_STEP, BANDIT_MARK_ROWS, color);
}

static int bandit_panic_mark(struct notifier_block *nb, unsigned long event, void *buf)
{
	/* red band at the bottom, then a white stripe to tell it from band 1 */
	bandit_fill(BANDIT_PANIC_ROW, 200, BANDIT_RED);
	bandit_fill(BANDIT_PANIC_ROW + 90, 20, BANDIT_WHITE);
	return NOTIFY_DONE;
}

static struct notifier_block bandit_panic_nb = {
	.notifier_call = bandit_panic_mark,
	.priority = INT_MAX,
};

/* Called from the machine's map_io, inside paging_init(). */
void __init bandit_marks_map_io(void)
{
	struct map_desc fb = {
		.virtual = BANDIT_FB_VIRT,
		.pfn = __phys_to_pfn(BANDIT_FB_PHYS),
		.length = BANDIT_FB_SIZE,
		.type = MT_DEVICE,
	};

	iotable_init(&fb, 1);
	bandit_fb_mapped = true;
	atomic_notifier_chain_register(&panic_notifier_list, &bandit_panic_nb);
}
