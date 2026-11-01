/*
 * Phones booted by the Windows Phone firmware (Nokia Lumia 1520 "bandit").
 *
 * Copyright (c) 2026, Lorenzo Giannoccaro.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * The Lumia TrustZone does not implement the SCM services that Android
 * MSM8974 kernels use for:
 *  - the CPU cold/warm boot address (scm_set_boot_addr): the call resets
 *    the SoC before any output, so secondary CPUs and power collapse are
 *    unavailable;
 *  - deep idle states that enter power collapse through TrustZone.
 * PAS/PIL authentication works (the ADSP boots on this firmware).
 *
 * Findings on real hardware by the mainline bring-up of the Lumia 1520
 * (KorelisLabs, lumia-1520-mainline, docs/trustzone-quirks.md).
 *
 * Detection: DT root compatible "nokia,bandit", or "msm_tz_restricted=<0|1>"
 * on the command line to force it either way.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <mach/tz-restricted.h>

static int tz_restricted_param = -1;

static int __init msm_tz_restricted_setup(char *str)
{
	get_option(&str, &tz_restricted_param);
	return 0;
}
early_param("msm_tz_restricted", msm_tz_restricted_setup);

bool msm_tz_restricted(void)
{
	if (tz_restricted_param >= 0)
		return tz_restricted_param;
	return of_machine_is_compatible("nokia,bandit");
}

bool __init msm_tz_restricted_early(void)
{
	if (tz_restricted_param >= 0)
		return tz_restricted_param;
	return of_flat_dt_is_compatible(of_get_flat_dt_root(), "nokia,bandit");
}
