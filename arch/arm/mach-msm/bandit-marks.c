/*
 * Nokia Lumia 1520 (bandit) bring-up: coloured boot progress bands on the
 * framebuffer the Windows Phone UEFI leaves running. See asm/bandit-mark.h.
 *
 * Bands 1-3 are painted from assembly with the MMU off; from paging_init()
 * on, this file maps the framebuffer statically (iotable, so vmalloc never
 * reuses the range) and paints the later bands and the panic band.
 *
 * It also registers a minimal text console on the lower part of the screen
 * (12x22 font, circular: a red line marks the newest line), so the kernel
 * log is readable from paging_init() on, long before simplefb and fbcon.
 * With initcall_debug the last "calling ..." line names the driver that
 * was running when the phone hung or reset.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/io.h>
#include <linux/console.h>
#include <linux/font.h>
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
	bandit_fill(BANDIT_PANIC_ROW, 60, BANDIT_RED);
	bandit_fill(BANDIT_PANIC_ROW + 25, 10, BANDIT_WHITE);
	return NOTIFY_DONE;
}

/* --- text console ------------------------------------------------------ */

#define CON_FONT_W	12
#define CON_FONT_H	22
#define CON_COLS	(1080 / CON_FONT_W)
#define CON_LINES	((BANDIT_CON_BOTTOM - BANDIT_CON_TOP) / CON_FONT_H)

static unsigned int con_line, con_col;

static void bandit_con_clear_line(unsigned int line)
{
	bandit_fill(BANDIT_CON_TOP + line * CON_FONT_H, CON_FONT_H, 0xFF000000);
}

static void bandit_con_cursor(void)
{
	unsigned int next = (con_line + 1) % CON_LINES;

	bandit_con_clear_line(next);
	bandit_fill(BANDIT_CON_TOP + next * CON_FONT_H, 3, BANDIT_RED);
}

static void bandit_con_char(unsigned int line, unsigned int col, unsigned char c)
{
	const unsigned char *glyph = (const unsigned char *)font_sun_12x22.data + c * CON_FONT_H * 2;
	u32 __iomem *row = (u32 __iomem *)(BANDIT_FB_VIRT +
			(BANDIT_CON_TOP + line * CON_FONT_H) * BANDIT_FB_STRIDE) + col * CON_FONT_W;
	unsigned int y, x;

	for (y = 0; y < CON_FONT_H; y++) {
		unsigned int bits = (glyph[2 * y] << 8) | glyph[2 * y + 1];

		for (x = 0; x < CON_FONT_W; x++)
			__raw_writel(bits & (0x8000 >> x) ? 0xFFFFFFFF : 0xFF000000, row + x);
		row += BANDIT_FB_STRIDE / 4;
	}
}

static void bandit_con_newline(void)
{
	con_line = (con_line + 1) % CON_LINES;
	con_col = 0;
	bandit_con_clear_line(con_line);
	bandit_con_cursor();
}

static void bandit_con_write(struct console *con, const char *s, unsigned int n)
{
	if (!bandit_fb_mapped)
		return;
	while (n--) {
		unsigned char c = *s++;

		if (c == '\n') {
			bandit_con_newline();
			continue;
		}
		if (c == '\r')
			continue;
		if (con_col >= CON_COLS)
			bandit_con_newline();
		bandit_con_char(con_line, con_col++, c);
	}
	wmb();
}

static struct console bandit_con = {
	.name = "bandit",
	.write = bandit_con_write,
	.flags = CON_PRINTBUFFER | CON_ENABLED,
	.index = -1,
};

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
	bandit_fill(BANDIT_CON_TOP, BANDIT_CON_BOTTOM - BANDIT_CON_TOP, 0xFF000000);
	bandit_con_cursor();
	register_console(&bandit_con);
}
