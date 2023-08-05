// SPDX-License-Identifier: BSD-3-Clause
/*
 * Clock drivers for Qualcomm APQ8016, APQ8096
 *
 * (C) Copyright 2015 Mateusz Kulikowski <mateusz.kulikowski@gmail.com>
 *
 * Based on Little Kernel driver, simplified
 */

#include <common.h>
#include <clk-uclass.h>
#include <dm.h>
#include <errno.h>
#include <asm/io.h>
#include <regmap.h>
#include <linux/bitops.h>

#include "clock-snapdragon.h"

/* CBCR register fields */
#define CBCR_BRANCH_ENABLE_BIT  BIT(0)
#define CBCR_BRANCH_OFF_BIT     BIT(31)

extern ulong msm_set_rate(struct clk *clk, ulong rate);
extern int msm_enable(struct clk *clk);

/* Enable clock controlled by CBC soft macro */
void clk_enable_cbc(struct regmap *map, uint off)
{
	int val;

	regmap_update_bits(map, off, CBCR_BRANCH_ENABLE_BIT, CBCR_BRANCH_ENABLE_BIT);

	regmap_read_poll_timeout(map, off, val, !(val & CBCR_BRANCH_OFF_BIT),
				 0, 1000000);
}

void clk_enable_gpll0(struct regmap *map, const struct pll_vote_clk *gpll0)
{
	int val;
	regmap_read(map, gpll0->status, &val);
	if (val & gpll0->status_bit)
		return; /* clock already enabled */

	regmap_update_bits(map, gpll0->ena_vote, gpll0->vote_bit, gpll0->vote_bit);

	regmap_read_poll_timeout(map, gpll0->status, val, val & gpll0->status_bit,
				 0, 1000000);
}

#define BRANCH_ON_VAL (0)
#define BRANCH_NOC_FSM_ON_VAL BIT(29)
#define BRANCH_CHECK_MASK GENMASK(31, 28)

void clk_enable_vote_clk(struct regmap *map, const struct vote_clk *vclk)
{
	u32 val;

	regmap_update_bits(map, vclk->ena_vote, vclk->vote_bit, vclk->vote_bit);

	regmap_read_poll_timeout(map, vclk->cbcr_reg, val,
					((val & BRANCH_CHECK_MASK) != BRANCH_ON_VAL
					&& (val & BRANCH_CHECK_MASK) != BRANCH_NOC_FSM_ON_VAL),
				 0, 1000000);
}

#define APPS_CMD_RCGR_UPDATE BIT(0)

/* Update clock command via CMD_RCGR */
void clk_bcr_update(struct regmap *map, uint off)
{
	int val;
	regmap_update_bits(map, off, APPS_CMD_RCGR_UPDATE, APPS_CMD_RCGR_UPDATE);

	/* Wait for frequency to be updated. */
	regmap_read_poll_timeout(map, off, val, !(val & APPS_CMD_RCGR_UPDATE),
				 0, 1000000);
}

#define CFG_MODE_DUAL_EDGE (0x2 << 12) /* Counter mode */

#define CFG_MASK 0x3FFF

#define CFG_DIVIDER_MASK 0x1F

/* root set rate for clocks with half integer and MND divider */
void clk_rcg_set_rate_mnd(struct regmap *map, const struct bcr_regs *regs,
			  int div, int m, int n, int source)
{
	u32 cfg;
	/* M value for MND divider. */
	u32 m_val = m;
	/* NOT(N-M) value for MND divider. */
	u32 n_val = ~((n) - (m)) * !!(n);
	/* NOT 2D value for MND divider. */
	u32 d_val = ~(n);

	/* Program MND values */
	regmap_write(map, regs->M, m_val);
	regmap_write(map, regs->N, n_val);
	regmap_write(map, regs->D, d_val);

	/* setup src select and divider */
	cfg = source & CFG_CLK_SRC_MASK; /* Select clock source */

	/* Set the divider; HW permits fraction dividers (+0.5), but
	   for simplicity, we will support integers only */
	if (div)
		cfg |= (2 * div - 1) & CFG_DIVIDER_MASK;

	if (n_val)
		cfg |= CFG_MODE_DUAL_EDGE;

	regmap_update_bits(map, regs->cfg_rcgr, CFG_MASK, cfg); /* Write new clock configuration */

	/* Inform h/w to start using the new config. */
	clk_bcr_update(map, regs->cmd_rcgr);
}

/* root set rate for clocks with half integer and mnd_width=0 */
void clk_rcg_set_rate(struct regmap *map, const struct bcr_regs *regs, int div,
		      int source)
{
	u32 cfg;

	/* setup src select and divider */
	cfg = source & CFG_CLK_SRC_MASK; /* Select clock source */

	/*
	 * Set the divider; HW permits fraction dividers (+0.5), but
	 * for simplicity, we will support integers only
	 */
	if (div)
		cfg |= (2 * div - 1) & CFG_DIVIDER_MASK;

	regmap_update_bits(map, regs->cfg_rcgr, CFG_MASK, cfg); /* Write new clock configuration */

	/* Inform h/w to start using the new config. */
	clk_bcr_update(map, regs->cmd_rcgr);
}

static int msm_clk_probe(struct udevice *dev)
{
	struct msm_clk_priv *priv = dev_get_priv(dev);
	int ret;

	ret = regmap_init_mem(dev_ofnode(dev), &priv->regmap);
	if (ret < 0) {
		printf("Failed to init regmap: %d\n", ret);
		return -EINVAL;
	}

	return 0;
}

static ulong msm_clk_set_rate(struct clk *clk, ulong rate)
{
	return msm_set_rate(clk, rate);
}

static int msm_clk_enable(struct clk *clk)
{
	return msm_enable(clk);
}

static struct clk_ops msm_clk_ops = {
	.set_rate = msm_clk_set_rate,
	.enable = msm_clk_enable,
};

static const struct udevice_id msm_clk_ids[] = {
	{ .compatible = "qcom,gcc-msm8916" },
	{ .compatible = "qcom,gcc-apq8016" },
	{ .compatible = "qcom,gcc-msm8996" },
	{ .compatible = "qcom,gcc-apq8096" },
	{ .compatible = "qcom,gcc-sdm845" },
	{ .compatible = "qcom,gcc-sm6115" },
	{ .compatible = "qcom,gcc-sm8250" },
	{ .compatible = "qcom,gcc-qcs404" },
	{ }
};

U_BOOT_DRIVER(clk_msm) = {
	.name		= "clk_msm",
	.id		= UCLASS_CLK,
	.of_match	= msm_clk_ids,
	.ops		= &msm_clk_ops,
	.priv_auto	= sizeof(struct msm_clk_priv),
	.probe		= msm_clk_probe,
};
