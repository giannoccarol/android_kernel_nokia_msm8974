/*
 * Nokia Lumia 1520 (bandit) bring-up: coloured boot progress bands.
 *
 * The Windows Phone UEFI leaves a live 1080x1920 a8r8g8b8 framebuffer at
 * physical 0x00400000 that the panel keeps scanning out. Each boot stage
 * paints one horizontal band, so the screen shows how far the kernel got
 * even when there is no USB, no console and no ram console yet.
 *
 * Band n covers rows n * BANDIT_MARK_STEP .. + BANDIT_MARK_ROWS.
 */
#ifndef __ASM_ARM_BANDIT_MARK_H
#define __ASM_ARM_BANDIT_MARK_H

#define BANDIT_FB_PHYS		0x00400000
#define BANDIT_FB_VIRT		0xF9000000	/* static mapping, below the MSM I/O maps */
#define BANDIT_FB_SIZE		0x00800000
#define BANDIT_FB_STRIDE	4320		/* 1080 pixels * 4 bytes */
#define BANDIT_MARK_ROWS	100
#define BANDIT_MARK_STEP	120
#define BANDIT_PANIC_ROW	1860	/* 60 rows at the bottom */
#define BANDIT_CON_TOP		1070	/* text console: rows 1070-1840 */
#define BANDIT_CON_BOTTOM	1840

#define BANDIT_RED		0xFFFF0000
#define BANDIT_ORANGE		0xFFFF8000
#define BANDIT_YELLOW		0xFFFFFF00
#define BANDIT_GREEN		0xFF00FF00
#define BANDIT_CYAN		0xFF00FFFF
#define BANDIT_BLUE		0xFF0050FF
#define BANDIT_MAGENTA		0xFFFF00FF
#define BANDIT_WHITE		0xFFFFFFFF

#ifdef __ASSEMBLY__

/*
 * Paints band \n with \color using physical addresses: only for code that
 * runs with the MMU off (zImage decompressor, kernel entry). Clobbers the
 * three scratch registers and the condition flags.
 */
	.macro	bandit_mark, n, color, ra, rb, rc
#ifdef CONFIG_BANDIT_BOOT_MARKS
	movw	\ra, #:lower16:(BANDIT_FB_PHYS + (\n) * BANDIT_MARK_STEP * BANDIT_FB_STRIDE)
	movt	\ra, #:upper16:(BANDIT_FB_PHYS + (\n) * BANDIT_MARK_STEP * BANDIT_FB_STRIDE)
	movw	\rb, #:lower16:(\color)
	movt	\rb, #:upper16:(\color)
	movw	\rc, #:lower16:(BANDIT_MARK_ROWS * BANDIT_FB_STRIDE / 4)
	movt	\rc, #:upper16:(BANDIT_MARK_ROWS * BANDIT_FB_STRIDE / 4)
1001:	str	\rb, [\ra], #4
	subs	\rc, \rc, #1
	bne	1001b
#endif
	.endm

#else /* !__ASSEMBLY__ */

#ifdef CONFIG_BANDIT_BOOT_MARKS
void bandit_marks_map_io(void);
void bandit_mark(int n, unsigned int color);
#else
static inline void bandit_marks_map_io(void) { }
static inline void bandit_mark(int n, unsigned int color) { }
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ASM_ARM_BANDIT_MARK_H */
