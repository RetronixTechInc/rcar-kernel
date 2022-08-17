// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas R-Car V3H System Controller
 *
 * Copyright (C) 2021 Renesas Electronics Corp.
 * Copyright (C) 2018 Cogent Embedded, Inc.
 */

#include <linux/bug.h>
#include <linux/io.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sys_soc.h>
#include <dt-bindings/power/r8a77980-sysc.h>

#include "rcar-sysc.h"

#define SYCSPDMD_ADDR			0xe6180064
#define PDMODE					(BIT(0) | BIT(1))

static unsigned int pd_mode;

static struct rcar_sysc_area r8a77980_areas[] __initdata = {
	{ "always-on",	    0, 0, R8A77980_PD_ALWAYS_ON, -1, PD_ALWAYS_ON },
	{ "ca53-scu",	0x140, 0, R8A77980_PD_CA53_SCU,	R8A77980_PD_ALWAYS_ON,
	  PD_SCU },
	{ "ca53-cpu0",	0x200, 0, R8A77980_PD_CA53_CPU0, R8A77980_PD_CA53_SCU,
	  PD_CPU_NOCR },
	{ "ca53-cpu1",	0x200, 1, R8A77980_PD_CA53_CPU1, R8A77980_PD_CA53_SCU,
	  PD_CPU_NOCR },
	{ "ca53-cpu2",	0x200, 2, R8A77980_PD_CA53_CPU2, R8A77980_PD_CA53_SCU,
	  PD_CPU_NOCR },
	{ "ca53-cpu3",	0x200, 3, R8A77980_PD_CA53_CPU3, R8A77980_PD_CA53_SCU,
	  PD_CPU_NOCR },
	{ "cr7",	0x240, 0, R8A77980_PD_CR7,	R8A77980_PD_ALWAYS_ON },
	{ "a3ir",	0x180, 0, R8A77980_PD_A3IR,	R8A77980_PD_ALWAYS_ON },
	{ "a2ir0",	0x400, 0, R8A77980_PD_A2IR0,	R8A77980_PD_A3IR },
	{ "a2ir1",	0x400, 1, R8A77980_PD_A2IR1,	R8A77980_PD_A3IR },
	{ "a2ir2",	0x400, 2, R8A77980_PD_A2IR2,	R8A77980_PD_A3IR },
	{ "a2ir3",	0x400, 3, R8A77980_PD_A2IR3,	R8A77980_PD_A3IR },
	{ "a2ir4",	0x400, 4, R8A77980_PD_A2IR4,	R8A77980_PD_A3IR },
	{ "a2ir5",	0x400, 5, R8A77980_PD_A2IR5,	R8A77980_PD_A3IR },
	{ "a2sc0",	0x400, 6, R8A77980_PD_A2SC0,	R8A77980_PD_A3IR },
	{ "a2sc1",	0x400, 7, R8A77980_PD_A2SC1,	R8A77980_PD_A3IR },
	{ "a2sc2",	0x400, 8, R8A77980_PD_A2SC2,	R8A77980_PD_A3IR },
	{ "a2sc3",	0x400, 9, R8A77980_PD_A2SC3,	R8A77980_PD_A3IR },
	{ "a2sc4",	0x400, 10, R8A77980_PD_A2SC4,	R8A77980_PD_A3IR },
	{ "a2dp0",	0x400, 11, R8A77980_PD_A2DP0,	R8A77980_PD_A3IR },
	{ "a2dp1",	0x400, 12, R8A77980_PD_A2DP1,	R8A77980_PD_A3IR },
	{ "a2cn",	0x400, 13, R8A77980_PD_A2CN,	R8A77980_PD_A3IR },
	{ "a3vip0",	0x2c0, 0, R8A77980_PD_A3VIP0,	R8A77980_PD_ALWAYS_ON },
	{ "a3vip1",	0x300, 0, R8A77980_PD_A3VIP1,	R8A77980_PD_ALWAYS_ON },
	{ "a3vip2",	0x280, 0, R8A77980_PD_A3VIP2,	R8A77980_PD_ALWAYS_ON },
};

/* Fixups for R-Car V3H ES2.* revision */
static const struct soc_device_attribute r8a77980[] __initconst = {
	{ .soc_id = "r8a77980", .revision = "ES2.*" },
	{ /* sentinel */ }
};

static int __init r8a77980_sysc_init(void)
{
	void __iomem *syscpdmd;

	if (!soc_device_match(r8a77980)) {
		pd_mode = 0; /* No handle PDMODE */
		return 0;
	}

	/* Get PDMODE bitfield */
	syscpdmd = ioremap(SYCSPDMD_ADDR, 0x04);
	pd_mode = readl(syscpdmd) & PDMODE;
	iounmap(syscpdmd);
	pr_info("%s: PDMODE %d is selected\n", __func__, pd_mode);
	return 0;
}

static struct rcar_sysc_extra_regs r8a77980_extra_regs = {
	.sysc_extmask_offs = 0x138, .sysc_extmask_msks = BIT(0)
};

const struct rcar_sysc_info r8a77980_sysc_info __initconst = {
	.init = r8a77980_sysc_init,
	.areas = r8a77980_areas,
	.num_areas = ARRAY_SIZE(r8a77980_areas),
	.extra_regs = &r8a77980_extra_regs,
	.mode = &pd_mode,
};
