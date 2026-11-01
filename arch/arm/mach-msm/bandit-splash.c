/*
 * Copyright (c) 2026, Lorenzo Giannoccaro.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * Nokia Lumia 1520 (bandit): colour order of the UEFI splash pipe.
 *
 * The Windows Phone UEFI leaves the DSI command-mode panel refreshed by
 * the MDP from the framebuffer at 0x00400000 (pipe RGB0, ping-pong 0
 * autorefresh). RGB0 unpacks the memory as B,G,R,A, while Android
 * renders RGBA, so red and blue were swapped. Until a real MDSS driver
 * owns the display, keep the MDSS register clock on and make RGB0 read
 * R,G,B,A; the simple-framebuffer node is declared as a8b8g8r8 to match.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <mach/tz-restricted.h>

#define MDSS_BASE		0xfd900000
#define MDSS_SIZE		0x2000
#define MDSS_HW_VERSION		0x0000
#define CTL0_FLUSH		0x0618
#define RGB0_SRC_FORMAT		0x1e30
#define RGB0_UNPACK		0x1e34

#define MDSS_V1_2		0x10020000
#define FMT_ARGB8888		0x000237ff
#define UNPACK_BGRA		0x03020001	/* firmware: B,G,R,A in memory */
#define UNPACK_RGBA		0x03010002	/* Android: R,G,B,A in memory */
#define FLUSH_RGB0		BIT(3)

static int __init bandit_splash_colors(void)
{
	struct clk *ahb;
	void __iomem *mdss;
	u32 ver, fmt, unpack;
	int ret;

	if (!msm_tz_restricted())
		return 0;

	/* Register interface clock; kept on for the lifetime of the splash */
	ahb = clk_get_sys("mdp.0", "iface_clk");
	if (IS_ERR(ahb)) {
		pr_err("bandit-splash: no MDSS AHB clock: %ld\n", PTR_ERR(ahb));
		return PTR_ERR(ahb);
	}
	ret = clk_prepare_enable(ahb);
	if (ret) {
		pr_err("bandit-splash: MDSS AHB clock: %d\n", ret);
		clk_put(ahb);
		return ret;
	}

	mdss = ioremap(MDSS_BASE, MDSS_SIZE);
	if (!mdss) {
		clk_disable_unprepare(ahb);
		clk_put(ahb);
		return -ENOMEM;
	}

	ver = readl_relaxed(mdss + MDSS_HW_VERSION);
	fmt = readl_relaxed(mdss + RGB0_SRC_FORMAT);
	unpack = readl_relaxed(mdss + RGB0_UNPACK);

	/* Only touch the exact state the UEFI is known to leave */
	if (ver != MDSS_V1_2 || fmt != FMT_ARGB8888 || unpack != UNPACK_BGRA) {
		pr_warn("bandit-splash: unexpected MDSS state (hw %08x fmt %08x unpack %08x), colours unchanged\n",
			ver, fmt, unpack);
		iounmap(mdss);
		clk_disable_unprepare(ahb);
		clk_put(ahb);
		return 0;
	}

	writel_relaxed(UNPACK_RGBA, mdss + RGB0_UNPACK);
	wmb();
	writel_relaxed(FLUSH_RGB0, mdss + CTL0_FLUSH);
	wmb();

	pr_info("bandit-splash: RGB0 unpack %08x -> %08x (RGBA)\n",
		unpack, readl_relaxed(mdss + RGB0_UNPACK));
	iounmap(mdss);
	return 0;
}
late_initcall(bandit_splash_colors);
