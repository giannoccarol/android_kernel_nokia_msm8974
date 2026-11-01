/*
 *  Copyright (c) 2003 ARM Limited
 *  All Rights Reserved
 *  Copyright (c) 2010, 2012 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/linkage.h>
#include <linux/init.h>

__CPUINIT

/*
 * MSM specific entry point for secondary CPUs.  This provides
 * a "holding pen" into which all secondary cores are held until we're
 * ready for them to initialise.
 *
 * This is executing in physical space with cache's off.
 */
ENTRY(msm_secondary_startup)
	mrc	p15, 0, r0, c0, c0, 5 	@ MPIDR
	and	r0, r0, #15		@ What CPU am I
	adr	r4, 1f			@ address of
	ldmia	r4, {r5, r6}		@ load curr addr and pen_rel addr
	sub	r4, r4, r5		@ determine virtual/phys offsets
	add	r6, r6, r4		@ apply
pen:
	ldr	r7, [r6]		@ pen_rel has cpu to remove from reset
	cmp	r7, r0			@ are we lucky?
	bne	pen

	/*
	 * we've been released from the holding pen: secondary_stack
	 * should now contain the SVC stack for this core
	 */
	b	secondary_startup
ENDPROC(msm_secondary_startup)

/*
 * Nokia Lumia 1520 (Windows Phone firmware): the UEFI parks the secondary
 * cores in a mailbox loop (MpParkDxe) and wakes the target with an SGI.
 * The loop branches here in ARM state, MMU and caches off, r0 = mailbox.
 * Acknowledge the wake-up SGI still pending in the GIC CPU interface, then
 * enter the usual holding pen. Same contract as the Lumia 930 port
 * (Markynose, lumia930-linux).
 */
	.arm
ENTRY(bandit_park_secondary_startup)
	cpsid	aif
	mvn	r1, #0
	str	r1, [r0]		@ mailbox id back to idle
	ldr	r2, =0xf9002000		@ GIC CPU interface
	ldr	r1, [r2, #0x0c]		@ GICC_IAR
	ubfx	r3, r1, #0, #10
	cmp	r3, #16			@ an SGI?
	strlo	r1, [r2, #0x10]		@ GICC_EOIR
	dsb
	b	msm_secondary_startup
	.ltorg
ENDPROC(bandit_park_secondary_startup)

1:	.long	.
	.long	pen_release
