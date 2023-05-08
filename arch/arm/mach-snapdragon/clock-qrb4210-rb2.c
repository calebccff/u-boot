// SPDX-License-Identifier: BSD-3-Clause
/*
 * Clock drivers for Qualcomm SM6115
 *
 * (C) Copyright 2023 Bhupesh Sharma <bhupesh.sharma@linaro.org>
 *
 * Based on Kernel driver, simplified
 */

#include <common.h>
#include <clk-uclass.h>
#include <dm.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include "clock-snapdragon.h"

#include <dt-bindings/clock/qcom,gcc-sm6115.h>

/* GPLL clock control registers */
#define GPLL0_STATUS_ACTIVE	BIT(31)

#define GCC_GLOBAL_EN_BASE		(0x84000)
#define GCC_CLOBAL_PERIPHERALS_EN	BIT(5)

#define GCC_APCS_CLOCK_BRANCH_ENA_VOTE_1	(0x7900C)

#define F(f, s, h, m, n) { (f), (s), (2 * (h) - 1), (m), (n) }

struct freq_tbl {
	uint freq;
	uint src;
	u8 pre_div;
	u16 m;
	u16 n;
};

static const struct freq_tbl ftbl_gcc_qupv3_wrap0_s0_clk_src[] = {
	F(7372800, CFG_CLK_SRC_GPLL0_OUT_AUX2, 1, 384, 15625),
	F(14745600, CFG_CLK_SRC_GPLL0_OUT_AUX2, 1, 768, 15625),
	F(19200000, CFG_CLK_SRC_CXO, 1, 0, 0),
	F(29491200, CFG_CLK_SRC_GPLL0_OUT_AUX2, 1, 1536, 15625),
	F(32000000, CFG_CLK_SRC_GPLL0_OUT_AUX2, 1, 8, 75),
	F(48000000, CFG_CLK_SRC_GPLL0_OUT_AUX2, 1, 4, 25),
	F(64000000, CFG_CLK_SRC_GPLL0_OUT_AUX2, 1, 16, 75),
	F(75000000, CFG_CLK_SRC_GPLL0_OUT_AUX2, 4, 0, 0),
	F(80000000, CFG_CLK_SRC_GPLL0_OUT_AUX2, 1, 4, 15),
	F(96000000, CFG_CLK_SRC_GPLL0_OUT_AUX2, 1, 8, 25),
	F(100000000, CFG_CLK_SRC_GPLL0_OUT_AUX2, 3, 0, 0),
	F(102400000, CFG_CLK_SRC_GPLL0_OUT_AUX2, 1, 128, 375),
	F(112000000, CFG_CLK_SRC_GPLL0_OUT_AUX2, 1, 28, 75),
	F(117964800, CFG_CLK_SRC_GPLL0_OUT_AUX2, 1, 6144, 15625),
	F(120000000, CFG_CLK_SRC_GPLL0_OUT_AUX2, 2.5, 0, 0),
	F(128000000, CFG_CLK_SRC_GPLL6_OUT_MAIN, 3, 0, 0),
	{ }
};

static const struct bcr_regs uart4_regs = {
	.cfg_rcgr = UART4_APPS_CFG_RCGR,
	.cmd_rcgr = UART4_APPS_CMD_RCGR,
	.M = UART4_APPS_M,
	.N = UART4_APPS_N,
	.D = UART4_APPS_D,
};

static const struct bcr_regs usb30_master_regs = {
	.cfg_rcgr = USB30_PRIM_MASTER_CFG_RCGR,
	.cmd_rcgr = USB30_PRIM_MASTER_CMD_RCGR,
	.M = USB30_PRIM_MASTER_M,
	.N = USB30_PRIM_MASTER_N,
	.D = USB30_PRIM_MASTER_D,
};

static const struct bcr_regs sdhc1_regs = {
	.cfg_rcgr = SDCC1_CFG_RCGR,
	.cmd_rcgr = SDCC1_CMD_RCGR,
	.M = SDCC1_M,
	.N = SDCC1_N,
	.D = SDCC1_D,
};

static const struct bcr_regs sdhc2_regs = {
	.cfg_rcgr = SDCC2_CFG_RCGR,
	.cmd_rcgr = SDCC2_CMD_RCGR,
	.M = SDCC2_M,
	.N = SDCC2_N,
	.D = SDCC2_D,
};

static struct pll_vote_clk gpll0_vote_clk = {
	.status = GPLL0_STATUS,
	.status_bit = GPLL0_STATUS_ACTIVE,
	.ena_vote = APCS_GPLL_ENA_VOTE,
	.vote_bit = BIT(0),
};

const struct freq_tbl *qcom_find_freq(const struct freq_tbl *f, uint rate)
{
	if (!f)
		return NULL;

	if (!f->freq)
		return f;

	for (; f->freq; f++)
		if (rate <= f->freq)
			return f;

	/* Default to our fastest rate */
	return f - 1;
}

static int clk_init_uart(struct msm_clk_priv *priv, uint rate)
{
	const struct freq_tbl *freq = qcom_find_freq(ftbl_gcc_qupv3_wrap0_s0_clk_src, rate);

	clk_rcg_set_rate_mnd(priv->base, &uart4_regs,
						freq->pre_div, freq->m, freq->n, freq->src);

	return 0;
}

ulong msm_set_rate(struct clk *clk, ulong rate)
{
	struct msm_clk_priv *priv = dev_get_priv(clk->dev);

	switch (clk->id) {
	case GCC_USB30_PRIM_MASTER_CLK:
		writel(0xF8282000, 0x0141a004); //GDSC
		clk_enable_cbc(priv->base + USB30_PRIM_MASTER_CBCR);
		clk_rcg_set_rate_mnd(priv->base, &usb30_master_regs, 4, 0, 0,
				     CFG_CLK_SRC_GPLL0);
		break;
	case GCC_CFG_NOC_USB3_PRIM_AXI_CLK:
		clk_enable_cbc(priv->base + CFG_NOC_USB3_PRIM_AXI_CBCR);
		break;
	case GCC_SYS_NOC_USB3_PRIM_AXI_CLK:
		clk_enable_cbc(priv->base + SYS_NOC_USB3_PRIM_AXI_CBCR);
		break;
	case GCC_USB30_PRIM_SLEEP_CLK:
		clk_enable_cbc(priv->base + USB30_PRIM_SLEEP_CBCR);
		break;
	case GCC_USB30_PRIM_MOCK_UTMI_CLK:
		clk_enable_cbc(priv->base + USB30_PRIM_MOCK_UTMI_CBCR);
		break;
	case GCC_USB3_PRIM_CLKREF_CLK:
		clk_enable_cbc(priv->base + USB3_PRIM_CLKREF_EN);
		break;
	case GCC_USB3_PRIM_PHY_COM_AUX_CLK:
		clk_enable_cbc(priv->base + USB3_PRIM_PHY_COM_AUX_CLK);
		break;
	case GCC_USB3_PRIM_PHY_PIPE_CLK:
		clk_enable_cbc(priv->base + USB3_PRIM_PHY_PIPE_CLK);
		break;
	case GCC_AHB2PHY_USB_CLK:
		clk_enable_cbc(priv->base + AHB2PHY_USB_CBCR);
		break;
	case GCC_SDCC1_APPS_CLK:
		/* SDCC1: 200MHz */
		clk_rcg_set_rate_mnd(priv->base, &sdhc1_regs, 4, 0, 0,
				     CFG_CLK_SRC_GPLL0);
		clk_enable_gpll0(priv->base, &gpll0_vote_clk);
		clk_enable_cbc(priv->base + SDCC1_APPS_CBCR);
		break;
	case GCC_SDCC1_AHB_CLK:
		clk_enable_cbc(priv->base + SDCC1_AHB_CBCR);
		break;
	case GCC_SDCC2_APPS_CLK:
		/* SDCC2: 200MHz */
		clk_rcg_set_rate_mnd(priv->base, &sdhc2_regs, 4, 0, 0,
				     CFG_CLK_SRC_GPLL0);
		clk_enable_gpll0(priv->base, &gpll0_vote_clk);
		clk_enable_cbc(priv->base + SDCC2_APPS_CBCR);
		break;
	case GCC_SDCC2_AHB_CLK:
		clk_enable_cbc(priv->base + SDCC2_AHB_CBCR);
		break;
	case GCC_QUPV3_WRAP0_S4_CLK: /* UART4 */
		return clk_init_uart(priv, rate);
	default:
		return 0;
	}

	return 0;
}

int msm_enable(struct clk *clk)
{
	return 0;
}
