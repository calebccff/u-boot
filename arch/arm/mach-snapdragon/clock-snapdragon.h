/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Qualcomm APQ8016, APQ8096, SDM845
 *
 * (C) Copyright 2017 Jorge Ramirez-Ortiz <jorge.ramirez-ortiz@linaro.org>
 */
#ifndef _CLOCK_SNAPDRAGON_H
#define _CLOCK_SNAPDRAGON_H

#include <linux/types.h>

#define CFG_CLK_SRC_CXO   (0 << 8)
#define CFG_CLK_SRC_GPLL0 (1 << 8)
#define CFG_CLK_SRC_GPLL0_EVEN (6 << 8)
#define CFG_CLK_SRC_GPLL0_OUT_EARLY	(1 << 8)
#define CFG_CLK_SRC_GPLL0_OUT_AUX2 (2 << 8)
#define CFG_CLK_SRC_GPLL6_OUT_MAIN (4 << 8)
#define CFG_CLK_SRC_MASK  (7 << 8)

struct pll_vote_clk {
	uintptr_t status;
	int status_bit;
	uintptr_t ena_vote;
	int vote_bit;
};

struct vote_clk {
	uintptr_t cbcr_reg;
	uintptr_t ena_vote;
	int vote_bit;
};
struct bcr_regs {
	uintptr_t cfg_rcgr;
	uintptr_t cmd_rcgr;
	uintptr_t M;
	uintptr_t N;
	uintptr_t D;
};

struct regmap;

struct msm_clk_priv {
	struct regmap *regmap;
};

void clk_enable_gpll0(struct regmap *map, const struct pll_vote_clk *gpll0);
void clk_bcr_update(struct regmap *map, uint off);
void clk_enable_cbc(struct regmap *map, uint off);
void clk_enable_vote_clk(struct regmap *map, const struct vote_clk *vclk);
void clk_rcg_set_rate_mnd(struct regmap *map, const struct bcr_regs *regs,
			  int div, int m, int n, int source);
void clk_rcg_set_rate(struct regmap *map, const struct bcr_regs *regs, int div,
		      int source);

#endif
