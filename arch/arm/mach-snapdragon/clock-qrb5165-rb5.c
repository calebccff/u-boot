// SPDX-License-Identifier: BSD-3-Clause
/*
 * Clock drivers for Qualcomm SM8250
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

#include <dt-bindings/clock/qcom,gcc-sm8250.h>

/* GPLL clock control registers */
#define GPLL0_STATUS_ACTIVE	BIT(31)

#define F(f, s, h, m, n) { (f), (s), (2 * (h) - 1), (m), (n) }

struct freq_tbl {
	uint freq;
	uint src;
	u8 pre_div;
	u16 m;
	u16 n;
};

static const struct freq_tbl ftbl_gcc_qupv3_wrap1_s4_clk_src[] = {
	F(7372800, CFG_CLK_SRC_GPLL0_EVEN, 1, 384, 15625),
	F(14745600, CFG_CLK_SRC_GPLL0_EVEN, 1, 768, 15625),
	F(19200000, CFG_CLK_SRC_CXO, 1, 0, 0),
	F(29491200, CFG_CLK_SRC_GPLL0_EVEN, 1, 1536, 15625),
	F(32000000, CFG_CLK_SRC_GPLL0_EVEN, 1, 8, 75),
	F(48000000, CFG_CLK_SRC_GPLL0_EVEN, 1, 4, 25),
	F(50000000, CFG_CLK_SRC_GPLL0_EVEN, 6, 0, 0),
	F(64000000, CFG_CLK_SRC_GPLL0_EVEN, 1, 16, 75),
	F(75000000, CFG_CLK_SRC_GPLL0_EVEN, 4, 0, 0),
	F(80000000, CFG_CLK_SRC_GPLL0_EVEN, 1, 4, 15),
	F(96000000, CFG_CLK_SRC_GPLL0_EVEN, 1, 8, 25),
	F(100000000, CFG_CLK_SRC_GPLL0, 6, 0, 0),
	{ }
};

static const struct bcr_regs debug_uart_regs = {
	.cfg_rcgr = DEBUG_UART_APPS_CFG_RCGR,
	.cmd_rcgr = DEBUG_UART_APPS_CMD_RCGR,
	.M = DEBUG_UART_APPS_M,
	.N = DEBUG_UART_APPS_N,
	.D = DEBUG_UART_APPS_D,
};

static const struct bcr_regs usb30_master_regs = {
	.cfg_rcgr = USB30_SEC_MASTER_CFG_RCGR,
	.cmd_rcgr = USB30_SEC_MASTER_CMD_RCGR,
	.M = USB30_SEC_MASTER_M,
	.N = USB30_SEC_MASTER_N,
	.D = USB30_SEC_MASTER_D,
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
	const struct freq_tbl *freq = qcom_find_freq(ftbl_gcc_qupv3_wrap1_s4_clk_src, rate);

	clk_rcg_set_rate_mnd(priv->base, &debug_uart_regs,
						freq->pre_div, freq->m, freq->n, freq->src);

	return 0;
}

ulong msm_set_rate(struct clk *clk, ulong rate)
{
	struct msm_clk_priv *priv = dev_get_priv(clk->dev);

	switch (clk->id) {
	case GCC_USB30_SEC_MASTER_CLK:
		writel(0xFFFDDA00, 0x00110004); //GDSC
		clk_enable_cbc(priv->base + USB30_SEC_MASTER_CBCR);
		clk_rcg_set_rate_mnd(priv->base, &usb30_master_regs, 4, 0, 0,
				     CFG_CLK_SRC_GPLL0);
		break;
	case GCC_CFG_NOC_USB3_SEC_AXI_CLK:
		clk_enable_cbc(priv->base + CFG_NOC_USB3_SEC_AXI_CBCR);
		break;
	case GCC_AGGRE_USB3_SEC_AXI_CLK:
		clk_enable_cbc(priv->base + AGGRE_USB3_SEC_AXI_CBCR);
		break;
	case GCC_USB30_SEC_SLEEP_CLK:
		clk_enable_cbc(priv->base + USB30_SEC_SLEEP_CBCR);
		break;
	case GCC_USB30_SEC_MOCK_UTMI_CLK:
		clk_enable_cbc(priv->base + USB30_SEC_MOCK_UTMI_CBCR);
		break;
	case GCC_USB3_SEC_CLKREF_EN:
		clk_enable_cbc(priv->base + USB3_SEC_CLKREF_EN);
		clk_enable_cbc(priv->base + USB3_SEC_PHY_AUX_CBCR);
		clk_enable_cbc(priv->base + USB3_SEC_PHY_COM_AUX_CBCR);
		clk_enable_cbc(priv->base + USB3_SEC_PHY_PIPE_CBCR);
		//clk_enable_cbc(priv->base + USB3_SEC_PHY_PIPE_MUXR);
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
	case GCC_QUPV3_WRAP1_S4_CLK: /* Debug UART */
		return clk_init_uart(priv, rate);
	default:
		return 0;
	}

	return 0;
}

void msm_rpmh_rsc_writes(void)
{
	writel(0x10108,   0x18220d30);
	writel(0x30080,   0x18220d34);
	writel(0x5,       0x18220d38);
	writel(0x1,       0x18220d1c);
	writel(0x0,       0x18220d14);
	writel(0x0,       0x18220d14);
	writel(0x10000,   0x18220d14);
	writel(0x1010000, 0x18220d14);
	writel(0x0,       0x18220d1c);
	//tcs_tx_done
	writel(0x1,       0x18220d08);

	writel(0x10008,   0x18220d30);
	//tcs_id=1
	writel(0x10108,   0x18220fd0);
	writel(0x30080,   0x18220d34);
	//tcs_id=1
	writel(0x300a0,   0x18220fd4);
	writel(0x5,       0x18220d38);
	writel(0x1,       0x18220d1c);
	writel(0x10000,   0x18220d14);
	//tcs_id=1
	writel(0x3,       0x18220fd8);
	writel(0x10000,   0x18220d14);
	writel(0x1010000, 0x18220d14);
	//tcs_id=1
	writel(0x1,       0x18220fbc);
	writel(0x0,       0x18220d1c);
	//tcs_id=1
	writel(0x0,       0x18220fb4);
	//tcs_tx_done
	writel(0x1,       0x18220d08);

	//tcs_id=1
	writel(0x0,       0x18220fb4);
	writel(0x10000,   0x18220fb4);
	writel(0x1010000, 0x18220fb4);
	//tcs_tx_done
	writel(0x2,       0x18220d08);

	writel(0x10008,   0x18220d30);
	writel(0x30080,   0x18220d34);
	writel(0x5,       0x18220d38);
	writel(0x1,       0x18220d1c);
	writel(0x10000,   0x18220d14);
	writel(0x0,       0x18220d14);
	writel(0x10000,   0x18220d14);
	writel(0x1010000, 0x18220d14);
	writel(0x0,       0x18220d1c);
	//tcs_tx_done
	writel(0x1,       0x18220d08);

	writel(0x10008,   0x18220d30);
	writel(0x30080,   0x18220d34);
	writel(0x5,       0x18220d38);
	writel(0x1,       0x18220d1c);

	writel(0x10000,   0x18220d14);
	writel(0x0,       0x18220d14);
	writel(0x10000,   0x18220d14);
	writel(0x1010000, 0x18220d14);
	writel(0x0,       0x18220d1c);
	//tcs_id=1
	writel(0x10108,   0x18220fd0);
	//tcs_tx_done
	writel(0x1,       0x18220d08);

	//tcs_id=1
	writel(0x30010,   0x18220fd4);
	writel(0x8,       0x18220fd8);
	writel(0x1,       0x18220fbc);
	writel(0x10000,   0x18220fb4);
	writel(0x0,       0x18220fb4);
	writel(0x10000,   0x18220fb4);
	writel(0x1010000, 0x18220fb4);
	writel(0x0,       0x18220fbc);
	//tcs_tx_done
	writel(0x2,       0x18220d08);

	writel(0x10108,   0x18220d30);
	writel(0x30000,   0x18220d34);
	writel(0x8,       0x18220d38);
	writel(0x1,       0x18220d1c);
	writel(0x10000,   0x18220d14);
	writel(0x0,       0x18220d14);
	writel(0x10000,   0x18220d14);
	writel(0x1010000, 0x18220d14);
	writel(0x0,       0x18220d1c);
	//tcs_tx_done
	writel(0x1,       0x18220d08);


	writel(0x10008,   0x18220d30);
	writel(0x30010,   0x18220d34);
	writel(0x8,       0x18220d38);
	writel(0x1,       0x18220d1c);
	writel(0x10000,   0x18220d14);
	writel(0x0,       0x18220d14);
	writel(0x10000,   0x18220d14);
	writel(0x1010000, 0x18220d14);
	//tcs_id=1
	writel(0x10008,   0x18220fd0);
	writel(0x0,       0x18220d1c);
	//tcs_id=1
	writel(0x30000,   0x18220fd4);
	//tcs_tx_done
	writel(0x1,       0x18220d08);

	//tcs_id=1
	writel(0x8,       0x18220fd8);
	writel(0x1,       0x18220fbc);
	writel(0x10000,   0x18220fb4);
	writel(0x0,       0x18220fb4);
	writel(0x10000,   0x18220fb4);
	writel(0x1010000, 0x18220fb4);
	writel(0x0,       0x18220fbc);
	//tcs_tx_done
	writel(0x2,       0x18220d08);
}

int msm_enable(struct clk *clk)
{
	msm_rpmh_rsc_writes();

	return 0;
}
