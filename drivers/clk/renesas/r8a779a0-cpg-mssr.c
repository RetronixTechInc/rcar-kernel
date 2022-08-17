// SPDX-License-Identifier: GPL-2.0
/*
 * r8a779a0 Clock Pulse Generator / Module Standby and Software Reset
 *
 * Copyright (C) 2020 Renesas Electronics Corp.
 *
 * Based on r8a7795-cpg-mssr.c
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/renesas/rcar-rst.h>
#include <linux/sys_soc.h>

#include <dt-bindings/clock/r8a779a0-cpg-mssr.h>

#include "renesas-cpg-mssr.h"
#include "rcar-v3u-cpg.h"

enum clk_ids {
	/* Core Clock Outputs exported to DT */
	LAST_DT_CORE_CLK = R8A779A0_CLK_OSC,

	/* External Input Clocks */
	CLK_EXTAL,
	CLK_EXTALR,

	/* Internal Core Clocks */
	CLK_MAIN,
	CLK_PLL1,
	CLK_PLL20,
	CLK_PLL21,
	CLK_PLL30,
	CLK_PLL31,
	CLK_PLL4,
	CLK_PLL5,
	CLK_PLL20_DIV2,
	CLK_PLL21_DIV2,
	CLK_PLL4_DIV2,
	CLK_PLL30_DIV2,
	CLK_PLL31_DIV2,
	CLK_PLL1_DIV2,
	CLK_PLL5_DIV2,
	CLK_PLL5_DIV4,
	CLK_S1,
	CLK_S2,
	CLK_S3,
	CLK_SG,
	CLK_SDSRC,
	CLK_RPCSRC,
	CLK_OCO,

	/* Module Clocks */
	MOD_CLK_BASE
};

static const struct cpg_core_clk r8a779a0_core_clks[] __initconst = {
	/* External Clock Inputs */
	DEF_INPUT("extal",  CLK_EXTAL),
	DEF_INPUT("extalr", CLK_EXTALR),

	/* Internal Core Clocks */
	DEF_BASE(".main",       CLK_MAIN,  CLK_TYPE_R8A779A0_MAIN,  CLK_EXTAL),
	DEF_BASE(".pll20",      CLK_PLL20, CLK_TYPE_R8A779A0_PLL20, CLK_MAIN),
	DEF_BASE(".pll21",      CLK_PLL21, CLK_TYPE_R8A779A0_PLL21, CLK_MAIN),
	DEF_BASE(".pll4",       CLK_PLL4,  CLK_TYPE_R8A779A0_PLL4,  CLK_MAIN),
	DEF_BASE(".pll30",      CLK_PLL30, CLK_TYPE_R8A779A0_PLL30, CLK_MAIN),
	DEF_BASE(".pll31",      CLK_PLL31, CLK_TYPE_R8A779A0_PLL31, CLK_MAIN),
	DEF_BASE(".pll1",       CLK_PLL1,  CLK_TYPE_R8A779A0_PLL1,  CLK_MAIN),
	DEF_BASE(".pll5",       CLK_PLL5,  CLK_TYPE_R8A779A0_PLL5,  CLK_MAIN),

	DEF_FIXED(".pll20_div2", CLK_PLL20_DIV2,   CLK_PLL20,  2, 1),
	DEF_FIXED(".pll21_div2", CLK_PLL21_DIV2,   CLK_PLL21,  2, 1),
	DEF_FIXED(".pll4_div2",  CLK_PLL4_DIV2,	   CLK_PLL4,   2, 1),
	DEF_FIXED(".pll30_div2", CLK_PLL30_DIV2,   CLK_PLL30,  2, 1),
	DEF_FIXED(".pll31_div2", CLK_PLL31_DIV2,   CLK_PLL31,  2, 1),
	DEF_FIXED(".pll1_div2",	 CLK_PLL1_DIV2,	   CLK_PLL1,   2, 1),
	DEF_FIXED(".pll5_div2",	 CLK_PLL5_DIV2,	   CLK_PLL5,   2, 1),
	DEF_FIXED(".pll5_div4",  CLK_PLL5_DIV4,    CLK_PLL5_DIV2,   2, 1),
	DEF_FIXED(".s1",	 CLK_S1,	   CLK_PLL1_DIV2,   2, 1),
	DEF_FIXED(".s2",	 CLK_S2,	   CLK_PLL1_DIV2,   133, 50),
	DEF_FIXED(".s3",	 CLK_S3,	   CLK_PLL1_DIV2,   4, 1),
	DEF_FIXED(".sg",	 CLK_SG,	   CLK_PLL1_DIV2,   1, 1),
	DEF_BASE(".rpcsrc",	 CLK_RPCSRC,	   CLK_TYPE_R8A779A0_RPCSRC, CLK_PLL5),
	DEF_FIXED(".sdsrc",	 CLK_SDSRC,  	   CLK_PLL5_DIV4,   1, 1),
	DEF_RATE(".oco",	 CLK_OCO,           32768),

	DEF_BASE("rpc",		 R8A779A0_CLK_RPC, CLK_TYPE_R8A779A0_RPC,
		 CLK_RPCSRC),
	DEF_BASE("rpcd2",	 R8A779A0_CLK_RPCD2, CLK_TYPE_R8A779A0_RPCD2,
		 R8A779A0_CLK_RPC),

	/* Core Clock Outputs */
	DEF_R8A779A0_Z("z0", R8A779A0_CLK_Z0,    CLK_TYPE_R8A779A0_Z, CLK_PLL20_DIV2,  1, 1),
	DEF_R8A779A0_Z("z1", R8A779A0_CLK_Z1,    CLK_TYPE_R8A779A0_Z, CLK_PLL21_DIV2,  1, 1),
	DEF_FIXED("zx",		R8A779A0_CLK_ZX,     CLK_PLL20_DIV2,  2, 1),
	DEF_FIXED("zk0",	R8A779A0_CLK_ZK0,    CLK_PLL20_DIV2,  1, 1),
	DEF_FIXED("zk1",	R8A779A0_CLK_ZK1,    CLK_PLL21_DIV2,  1, 1),
	DEF_FIXED("s1d1",	R8A779A0_CLK_S1D1,   CLK_S1, 1, 1),
	DEF_FIXED("s1d2",	R8A779A0_CLK_S1D2,   CLK_S1, 2, 1),
	DEF_FIXED("s1d4",	R8A779A0_CLK_S1D4,   CLK_S1, 4, 1),
	DEF_FIXED("s1d8",	R8A779A0_CLK_S1D8,   CLK_S1, 8, 1),
	DEF_FIXED("s1d12",	R8A779A0_CLK_S1D12,  CLK_S1, 12, 1),
	DEF_FIXED("s2d1",	R8A779A0_CLK_S2D1,   CLK_S2, 1, 1),
	DEF_FIXED("s2d2",	R8A779A0_CLK_S2D2,   CLK_S2, 2, 1),
	DEF_FIXED("s2d4",	R8A779A0_CLK_S2D4,   CLK_S2, 4, 1),
	DEF_FIXED("s3d1",	R8A779A0_CLK_S3D1,   CLK_S3, 1, 1),
	DEF_FIXED("s3d2",	R8A779A0_CLK_S3D2,   CLK_S3, 2, 1),
	DEF_FIXED("s3d4",	R8A779A0_CLK_S3D4,   CLK_S3, 4, 1),
	DEF_FIXED("sgd1",	R8A779A0_CLK_SGD1,   CLK_SG, 1, 1),
	DEF_FIXED("sgd2",	R8A779A0_CLK_SGD2,   CLK_SG, 2, 1),
	DEF_FIXED("sgd4",	R8A779A0_CLK_SGD4,   CLK_SG, 4, 1),
	DEF_FIXED("zs",		R8A779A0_CLK_ZS,     CLK_PLL1_DIV2,   4, 1),
	DEF_FIXED("zt",		R8A779A0_CLK_ZT,     CLK_PLL1_DIV2,   2, 1),
	DEF_FIXED("ztr",	R8A779A0_CLK_ZTR,    CLK_PLL1_DIV2,   2, 1),
	DEF_FIXED("zr",		R8A779A0_CLK_ZR,     CLK_PLL1_DIV2,   1, 1),
	DEF_FIXED("cnndsp",	R8A779A0_CLK_CNNDSP, CLK_PLL5_DIV4,   1, 1),
	DEF_FIXED("vip",	R8A779A0_CLK_VIP,    CLK_PLL5,        5, 1),
	DEF_FIXED("adgh",	R8A779A0_CLK_ADGH,   CLK_PLL5_DIV4,   1, 1),
	DEF_FIXED("icu",	R8A779A0_CLK_ICU,    CLK_PLL5_DIV4,   2, 1),
	DEF_FIXED("icud2",	R8A779A0_CLK_ICUD2,  CLK_PLL5_DIV4,   4, 1),
	DEF_FIXED("vcbus",	R8A779A0_CLK_VCBUS,  CLK_PLL5_DIV4,   1, 1),

	DEF_FIXED("cl",		R8A779A0_CLK_CL,     CLK_PLL1_DIV2,  32, 1),
	DEF_FIXED("cbfusa",	R8A779A0_CLK_CPFUSA, CLK_MAIN,	     2, 1),
	DEF_FIXED("cp",		R8A779A0_CLK_CP,     CLK_MAIN,	     2, 1),
	DEF_FIXED("clk13m",	R8A779A0_CLK_13M,    CLK_MAIN,	     2, 1),
	DEF_FIXED("cl16m",	R8A779A0_CLK_CL16M,  CLK_PLL1_DIV2,  64, 1),

	DEF_R8A779A0_SD("sd0",	R8A779A0_CLK_SD0,    CLK_SDSRC,	     0x0870),

	DEF_DIV6P1("mso",	R8A779A0_CLK_MSO,    CLK_PLL5_DIV4, 0x87c),
	DEF_DIV6P1("canfd",	R8A779A0_CLK_CANFD,  CLK_PLL5_DIV4, 0x878),
	DEF_DIV6P1("csi0",	R8A779A0_CLK_CSI0,   CLK_PLL5_DIV4, 0x880),
	DEF_DIV6P1("dsi",	R8A779A0_CLK_DSI,    CLK_PLL5_DIV4, 0x884),
	DEF_DIV6P1("fray",	R8A779A0_CLK_FRAY,   CLK_PLL5_DIV4, 0x88c),
	DEF_DIV6P1("ipc",	R8A779A0_CLK_IPC,    CLK_PLL5_DIV4, 0x888),
	DEF_DIV6P1("post",	R8A779A0_CLK_POST,   CLK_PLL5_DIV4, 0x890),
	DEF_DIV6P1("post2",	R8A779A0_CLK_POST2,  CLK_PLL5_DIV4, 0x894),
	DEF_DIV6P1("post3",	R8A779A0_CLK_POST3,  CLK_PLL5_DIV4, 0x898),
	DEF_DIV6P1("post4",	R8A779A0_CLK_POST4,  CLK_PLL5_DIV4, 0x89c),

	DEF_R8A779A0_OSC("osc",	R8A779A0_CLK_OSC,   CLK_EXTAL,     8),
	DEF_R8A779A0_MDSEL("r",	R8A779A0_CLK_R, 29, CLK_EXTALR, 1, CLK_OCO, 1),
};

static const struct mssr_mod_clk r8a779a0_mod_clks[] __initconst = {
	DEF_MOD("stv0",			  1,	R8A779A0_CLK_VIP),
	DEF_MOD("stv1",			  2,	R8A779A0_CLK_VIP),
	DEF_MOD("smd_post0",		  5,	R8A779A0_CLK_VIP),
	DEF_MOD("smd_post1",		  6,	R8A779A0_CLK_VIP),
	DEF_MOD("smd_ps0",		  7,	R8A779A0_CLK_VIP),
	DEF_MOD("smd_ps1",		  8,	R8A779A0_CLK_VIP),
	DEF_MOD("dof0",			  9,	R8A779A0_CLK_VIP),
	DEF_MOD("dof1",			 10,	R8A779A0_CLK_VIP),
	DEF_MOD("acf0",			 11,	R8A779A0_CLK_VIP),
	DEF_MOD("acf1",			 12,	R8A779A0_CLK_VIP),
	DEF_MOD("acf2",			 13,	R8A779A0_CLK_VIP),
	DEF_MOD("acf3",			 14,	R8A779A0_CLK_VIP),
	DEF_MOD("isp0",			 16,	R8A779A0_CLK_S1D1),
	DEF_MOD("isp1",			 17,	R8A779A0_CLK_S1D1),
	DEF_MOD("isp2",			 18,	R8A779A0_CLK_S1D1),
	DEF_MOD("isp3",			 19,	R8A779A0_CLK_S1D1),
	DEF_MOD("radsp0",		 20,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("radsp1",		 21,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("impcnn0",		 22,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("spmc0",		 23,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("imp0",			 24,	R8A779A0_CLK_S1D1),
	DEF_MOD("imp1",			 25,	R8A779A0_CLK_S1D1),
	DEF_MOD("impdma0",		 26,	R8A779A0_CLK_S1D1),
	DEF_MOD("imppsc0",		 27,	R8A779A0_CLK_S1D1),
	DEF_MOD("ocv0",			 28,	R8A779A0_CLK_S1D1),
	DEF_MOD("ocv1",			 29,	R8A779A0_CLK_S1D1),
	DEF_MOD("ocv2",			 30,	R8A779A0_CLK_S1D1),
	DEF_MOD("ocv3",			 31,	R8A779A0_CLK_S1D1),
	DEF_MOD("ocv4",			 100,	R8A779A0_CLK_S1D1),
	DEF_MOD("impcnn2",		 101,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("spmc2",		 102,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("imp2",			 103,	R8A779A0_CLK_S1D1),
	DEF_MOD("imp3",			 104,	R8A779A0_CLK_S1D1),
	DEF_MOD("impdma1",		 105,	R8A779A0_CLK_S1D1),
	DEF_MOD("imppsc1",		 106,	R8A779A0_CLK_S1D1),
	DEF_MOD("ocv5",			 107,	R8A779A0_CLK_S1D1),
	DEF_MOD("impcnn1",		 108,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("spmc1",		 109,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("impdta",		 116,	R8A779A0_CLK_S1D1),
	DEF_MOD("impldma",		 117,	R8A779A0_CLK_S1D1),
	DEF_MOD("impslv",		 118,	R8A779A0_CLK_S1D1),
	DEF_MOD("ipmmuir",		 119,	R8A779A0_CLK_S1D1),
	DEF_MOD("spmi0",		 120,	R8A779A0_CLK_S1D1),
	DEF_MOD("spmi1",		 121,	R8A779A0_CLK_S1D1),
	DEF_MOD("avb0",			 211,	R8A779A0_CLK_S3D1),
	DEF_MOD("avb1",			 212,	R8A779A0_CLK_S3D1),
	DEF_MOD("avb2",			 213,	R8A779A0_CLK_S3D1),
	DEF_MOD("avb3",			 214,	R8A779A0_CLK_S3D1),
	DEF_MOD("avb4",			 215,	R8A779A0_CLK_S3D1),
	DEF_MOD("avb5",			 216,	R8A779A0_CLK_S3D1),
	DEF_MOD("ocv6",			 313,	R8A779A0_CLK_S1D1),
	DEF_MOD("ocv7",			 314,	R8A779A0_CLK_S1D1),
	DEF_MOD("can-fd",		 328,	R8A779A0_CLK_CANFD),
	DEF_MOD("csi40",		 331,	R8A779A0_CLK_CSI0),
	DEF_MOD("csi41",		 400,	R8A779A0_CLK_CSI0),
	DEF_MOD("csi42",		 401,	R8A779A0_CLK_CSI0),
	DEF_MOD("csi43",		 402,	R8A779A0_CLK_CSI0),
	DEF_MOD("du0",			 411,	R8A779A0_CLK_S2D1),
	DEF_MOD("ipmmuvi0",		 412,	R8A779A0_CLK_S1D1),
	DEF_MOD("ipmmuvi1",		 413,	R8A779A0_CLK_S1D1),
	DEF_MOD("dsi0",			 415,	R8A779A0_CLK_DSI),
	DEF_MOD("dsi1",			 416,	R8A779A0_CLK_DSI),
	DEF_MOD("fcpcs",		 507,	R8A779A0_CLK_S1D1),
	DEF_MOD("fcpvd0",		 508,	R8A779A0_CLK_S3D1),
	DEF_MOD("fcpvd1",		 509,	R8A779A0_CLK_S3D1),
	DEF_MOD("hscif0",		 514,	R8A779A0_CLK_S1D2),
	DEF_MOD("hscif1",		 515,	R8A779A0_CLK_S1D2),
	DEF_MOD("hscif2",		 516,	R8A779A0_CLK_S1D2),
	DEF_MOD("hscif3",		 517,	R8A779A0_CLK_S1D2),
	DEF_MOD("i2c0",			 518,	R8A779A0_CLK_S1D4),
	DEF_MOD("i2c1",			 519,	R8A779A0_CLK_S1D4),
	DEF_MOD("i2c2",			 520,	R8A779A0_CLK_S1D4),
	DEF_MOD("i2c3",			 521,	R8A779A0_CLK_S1D4),
	DEF_MOD("i2c4",			 522,	R8A779A0_CLK_S1D4),
	DEF_MOD("i2c5",			 523,	R8A779A0_CLK_S1D4),
	DEF_MOD("i2c6",			 524,	R8A779A0_CLK_S1D4),
	DEF_MOD("imr2",			 525,	R8A779A0_CLK_S2D2),
	DEF_MOD("imr3",			 526,	R8A779A0_CLK_S2D2),
	DEF_MOD("imr4",			 527,	R8A779A0_CLK_S2D2),
	DEF_MOD("imr5",			 528,	R8A779A0_CLK_S2D2),
	DEF_MOD("imr0",			 529,	R8A779A0_CLK_S2D2),
	DEF_MOD("imr1",			 530,	R8A779A0_CLK_S2D2),
	DEF_MOD("ipmmuds0",		 601,	R8A779A0_CLK_S1D1),
	DEF_MOD("ipmmuds1",		 602,	R8A779A0_CLK_S1D1),
	DEF_MOD("ipmmumm",		 603,	R8A779A0_CLK_S1D1),
	DEF_MOD("ipmmupv0",		 604,	R8A779A0_CLK_S1D1),
	DEF_MOD("ipmmurt0",		 605,	R8A779A0_CLK_S1D1),
	DEF_MOD("ipmmurt1",		 606,	R8A779A0_CLK_S1D1),
	DEF_MOD("ipmmuvc",		 607,	R8A779A0_CLK_S1D1),
	DEF_MOD("ipmmuvip0",		 609,	R8A779A0_CLK_S1D1),
	DEF_MOD("ipmmuvip1",		 610,	R8A779A0_CLK_S1D1),
	DEF_MOD("ispcs0",		 612,	R8A779A0_CLK_S1D1),
	DEF_MOD("ispcs1",		 613,	R8A779A0_CLK_S1D1),
	DEF_MOD("ispcs2",		 614,	R8A779A0_CLK_S1D1),
	DEF_MOD("ispcs3",		 615,	R8A779A0_CLK_S1D1),
	DEF_MOD("ivcp1e",		 616,	R8A779A0_CLK_S1D1),
	DEF_MOD("mfis",		 	 617,	R8A779A0_CLK_S1D4),
	DEF_MOD("msi0",			 618,	R8A779A0_CLK_MSO),
	DEF_MOD("msi1",			 619,	R8A779A0_CLK_MSO),
	DEF_MOD("msi2",			 620,	R8A779A0_CLK_MSO),
	DEF_MOD("msi3",			 621,	R8A779A0_CLK_MSO),
	DEF_MOD("msi4",			 622,	R8A779A0_CLK_MSO),
	DEF_MOD("msi5",			 623,	R8A779A0_CLK_MSO),
	DEF_MOD("pci0",			 624,	R8A779A0_CLK_S1D1),
	DEF_MOD("pci1",			 625,	R8A779A0_CLK_S1D1),
	DEF_MOD("pci2",			 626,	R8A779A0_CLK_S1D1),
	DEF_MOD("pci3",			 627,	R8A779A0_CLK_S1D1),
	DEF_MOD("pwm0",			 628,	R8A779A0_CLK_S1D8),
	DEF_MOD("rpc-if",		 629,	R8A779A0_CLK_RPCD2),
	DEF_MOD("rtdm0",		 630,	R8A779A0_CLK_SGD4),
	DEF_MOD("rtdm1",		 631,	R8A779A0_CLK_SGD4),
	DEF_MOD("rtdm2",		 700,	R8A779A0_CLK_SGD4),
	DEF_MOD("rtdm3",		 701,	R8A779A0_CLK_SGD4),
	DEF_MOD("scif0",		 702,	R8A779A0_CLK_S1D8),
	DEF_MOD("scif1",		 703,	R8A779A0_CLK_S1D8),
	DEF_MOD("scif3",		 704,	R8A779A0_CLK_S1D8),
	DEF_MOD("scif4",		 705,	R8A779A0_CLK_S1D8),
	DEF_MOD("sdhi0",		 706,	R8A779A0_CLK_SD0),
	DEF_MOD("sydm1",		 709,	R8A779A0_CLK_S1D2),
	DEF_MOD("sydm2",		 710,	R8A779A0_CLK_S1D2),
	DEF_MOD("tmu0",			 713,	R8A779A0_CLK_CL16M),
	DEF_MOD("tmu1",			 714,	R8A779A0_CLK_S1D4),
	DEF_MOD("tmu2",			 715,	R8A779A0_CLK_S1D4),
	DEF_MOD("tmu3",			 716,	R8A779A0_CLK_S1D4),
	DEF_MOD("tmu4",			 717,	R8A779A0_CLK_S1D4),
	DEF_MOD("tpu0",			 718,	R8A779A0_CLK_S1D8),
	DEF_MOD("caiplite0",		 721,	R8A779A0_CLK_S1D1),
	DEF_MOD("caiplite1",		 722,	R8A779A0_CLK_S1D1),
	DEF_MOD("caiplite2",		 723,	R8A779A0_CLK_S1D1),
	DEF_MOD("caiplite3",		 724,	R8A779A0_CLK_S1D1),
	DEF_MOD("caiplite4",		 725,	R8A779A0_CLK_S1D1),
	DEF_MOD("caiplite5",		 726,	R8A779A0_CLK_S1D1),
	DEF_MOD("caiplite6",		 727,	R8A779A0_CLK_S1D1),
	DEF_MOD("caiplite7",		 728,	R8A779A0_CLK_S1D1),
	DEF_MOD("vcpl4",		 729,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin00",		 730,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin01",		 731,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin02",		 800,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin03",		 801,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin04",		 802,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin05",		 803,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin06",		 804,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin07",		 805,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin10",		 806,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin11",		 807,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin12",		 808,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin13",		 809,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin14",		 810,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin15",		 811,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin16",		 812,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin17",		 813,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin20",		 814,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin21",		 815,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin22",		 816,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin23",		 817,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin24",		 818,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin25",		 819,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin26",		 820,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin27",		 821,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin30",		 822,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin31",		 823,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin32",		 824,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin33",		 825,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin34",		 826,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin35",		 827,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin36",		 828,	R8A779A0_CLK_S1D1),
	DEF_MOD("vin37",		 829,	R8A779A0_CLK_S1D1),
	DEF_MOD("vspd0",		 830,	R8A779A0_CLK_S3D1),
	DEF_MOD("vspd1",		 831,	R8A779A0_CLK_S3D1),
	DEF_MOD("wcrc_caiplite0",	 903,	R8A779A0_CLK_S1D1),
	DEF_MOD("wcrc_caiplite1",	 904,	R8A779A0_CLK_S1D1),
	DEF_MOD("wcrc_caiplite2",	 905,	R8A779A0_CLK_S1D1),
	DEF_MOD("wcrc_caiplite3",	 906,	R8A779A0_CLK_S1D1),
	DEF_MOD("wdt",			 907,	R8A779A0_CLK_R),
	DEF_MOD("cmt0",			 910,	R8A779A0_CLK_R),
	DEF_MOD("cmt1",			 911,	R8A779A0_CLK_R),
	DEF_MOD("cmt2",			 912,	R8A779A0_CLK_R),
	DEF_MOD("cmt3",			 913,	R8A779A0_CLK_R),
	DEF_MOD("pfc0",			 915,	R8A779A0_CLK_CP),
	DEF_MOD("pfc1",			 916,	R8A779A0_CLK_CP),
	DEF_MOD("pfc2",			 917,	R8A779A0_CLK_CP),
	DEF_MOD("pfc3",			 918,	R8A779A0_CLK_CP),
	DEF_MOD("vspx0",		 1028,	R8A779A0_CLK_S1D1),
	DEF_MOD("vspx1",		 1029,	R8A779A0_CLK_S1D1),
	DEF_MOD("vspx2",		 1030,	R8A779A0_CLK_S1D1),
	DEF_MOD("vspx3",		 1031,	R8A779A0_CLK_S1D1),
	DEF_MOD("fbc",			 1117,	R8A779A0_CLK_S1D4),
	DEF_MOD("wwdt0",		 1200,	R8A779A0_CLK_CL16M),
	DEF_MOD("wwdt1",		 1201,	R8A779A0_CLK_CL16M),
	DEF_MOD("wwdt2",		 1202,	R8A779A0_CLK_CL16M),
	DEF_MOD("wwdt3",		 1203,	R8A779A0_CLK_CL16M),
	DEF_MOD("wwdt4",		 1204,	R8A779A0_CLK_CL16M),
	DEF_MOD("wwdt5",		 1205,	R8A779A0_CLK_CL16M),
	DEF_MOD("wwdt6",		 1206,	R8A779A0_CLK_CL16M),
	DEF_MOD("wwdt7",		 1207,	R8A779A0_CLK_CL16M),
	DEF_MOD("wwdt8",		 1208,	R8A779A0_CLK_CL16M),
	DEF_MOD("wwdt9",		 1209,	R8A779A0_CLK_CL16M),
	DEF_MOD("fba_acf0",		 1829,	R8A779A0_CLK_VIP),
	DEF_MOD("fba_acf1",		 1830,	R8A779A0_CLK_VIP),
	DEF_MOD("fba_cnn0_main", 1831,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn0_sub0", 1900,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn0_sub1", 1901,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn0_sub2", 1902,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn0_sub3", 1903,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn1_main", 1904,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn1_sub0", 1905,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn1_sub1", 1906,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn1_sub2", 1907,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn1_sub3", 1908,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn2_main", 1909,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn2_sub0", 1910,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn2_sub1", 1911,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn2_sub2", 1912,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnn2_sub3", 1913,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnram0",	 1914,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnram1",	 1915,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_cnram2",	 1916,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_stv0",		 1922,	R8A779A0_CLK_VIP),
	DEF_MOD("fba_radsp0",	 1923,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_radsp1",	 1924,	R8A779A0_CLK_CNNDSP),
	DEF_MOD("fba_imp0",		 1925,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_imp1",		 1926,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_imp2",		 1927,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_imp3",		 1928,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_imr0",		 1931,	R8A779A0_CLK_S2D2),
	DEF_MOD("fba_imr1",		 2000,	R8A779A0_CLK_S2D2),
	DEF_MOD("fba_imr2",		 2001,	R8A779A0_CLK_S2D2),
	DEF_MOD("fba_imr3",		 2002,	R8A779A0_CLK_S2D2),
	DEF_MOD("fba_ims0",		 2003,	R8A779A0_CLK_S2D2),
	DEF_MOD("fba_ims1",		 2004,	R8A779A0_CLK_S2D2),
	DEF_MOD("fba_isp0",		 2007,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_isp1",		 2008,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_isp2",		 2009,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_isp3",		 2010,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_cve0",		 2012,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_cve1",		 2013,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_cve2",		 2014,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_cve3",		 2015,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_cve4",		 2016,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_cve5",		 2017,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_dp0",		 2020,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_dp1",		 2021,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_dof0",		 2102,	R8A779A0_CLK_VIP),
	DEF_MOD("fba_dof1",		 2103,	R8A779A0_CLK_VIP),
	DEF_MOD("fba_cve6",		 2220,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_cve7",		 2221,	R8A779A0_CLK_S1D1),
	DEF_MOD("fba_stv1",		 2223,	R8A779A0_CLK_VIP),
};

static const unsigned int r8a779a0_crit_mod_clks[] __initconst = {
	MOD_CLK_ID(408),	/* INTC-AP (GIC) */
};


/*
 * CPG Clock Data
 */

/*
 *   MD		EXTAL		PLL1	PLL20,21	PLL30,31	PLL4	PLL5	PLL6	OSC
 * 14 13	(MHz)
 * --------------------------------------------------------------------------------
 * 0  0		16.66 x 1	x128	x216		x128		x144	x192	x128	/16
 * 0  1		20    x 1	x106	x180		x106		x120	x160	x106	/19
 * 1  0		xx    x x	xxxx	xxxx		xxxx		xxxx	xxxx	xxxx	/x
 * 1  1		33.33 / 2	x128	x216		x128		x144	x192	x128	/32
 */
#define CPG_PLL_CONFIG_INDEX(md)	((((md) & BIT(14)) >> 13) | \
					 (((md) & BIT(13)) >> 13))

static const struct rcar_r8a779a0_cpg_pll_config cpg_pll_configs[4] = {
	/* EXTAL div	PLL1 mult/div	PLL5 mult/div	OSC prediv */
	{ 1,			128,	1,		192,	1,			16,	},
	{ 1,			106,	1,		160,	1,			19,	},
	{ 0,			0,		0,		0,		0,			0,	},
	{ 2,			128,	1,		192,	1,			32,	},
};

static int __init r8a779a0_cpg_mssr_init(struct device *dev)
{
	const struct rcar_r8a779a0_cpg_pll_config *cpg_pll_config;
	u32 cpg_mode;
	int error;

	error = rcar_rst_read_mode_pins(&cpg_mode);
	if (error)
		return error;

	cpg_pll_config = &cpg_pll_configs[CPG_PLL_CONFIG_INDEX(cpg_mode)];

	return rcar_r8a779a0_cpg_init(cpg_pll_config, CLK_EXTALR, cpg_mode);
}

const struct cpg_mssr_info r8a779a0_cpg_mssr_info __initconst = {
	/* Core Clocks */
	.core_clks = r8a779a0_core_clks,
	.num_core_clks = ARRAY_SIZE(r8a779a0_core_clks),
	.last_dt_core_clk = LAST_DT_CORE_CLK,
	.num_total_core_clks = MOD_CLK_BASE,

	/* Module Clocks */
	.mod_clks = r8a779a0_mod_clks,
	.num_mod_clks = ARRAY_SIZE(r8a779a0_mod_clks),
	.num_hw_mod_clks = 24 * 32,

	/* Critical Module Clocks */
	.crit_mod_clks = r8a779a0_crit_mod_clks,
	.num_crit_mod_clks = ARRAY_SIZE(r8a779a0_crit_mod_clks),

	/* Callbacks */
	.init = r8a779a0_cpg_mssr_init,
	.cpg_clk_register = rcar_r8a779a0_cpg_clk_register,

	/* The device has only MSTP Control Register */
	.mstpctrl = true,

	/* Device generation */
	.gen = 3,
};
