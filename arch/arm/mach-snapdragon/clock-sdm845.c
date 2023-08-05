// SPDX-License-Identifier: BSD-3-Clause
/*
 * Clock drivers for Qualcomm SDM845
 *
 * (C) Copyright 2017 Jorge Ramirez Ortiz <jorge.ramirez-ortiz@linaro.org>
 * (C) Copyright 2021 Dzmitry Sankouski <dsankouski@gmail.com>
 *
 * Based on Little Kernel driver, simplified
 */

#include <common.h>
#include <clk-uclass.h>
#include <dm.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <dm/device_compat.h>
#include <dt-bindings/clock/qcom,gcc-sdm845.h>

#include "clock-snapdragon.h"

#define F(f, s, h, m, n) { (f), (s), (2 * (h) - 1), (m), (n) }

struct freq_tbl {
	uint freq;
	uint src;
	u8 pre_div;
	u16 m;
	u16 n;
};

static const char * const gcc_clk_names[] = {
	"GCC_AGGRE_NOC_PCIE_TBU_CLK",
	"GCC_AGGRE_UFS_CARD_AXI_CLK",
	"GCC_AGGRE_UFS_PHY_AXI_CLK",
	"GCC_AGGRE_USB3_PRIM_AXI_CLK",
	"GCC_AGGRE_USB3_SEC_AXI_CLK",
	"GCC_BOOT_ROM_AHB_CLK",
	"GCC_CAMERA_AHB_CLK",
	"GCC_CAMERA_AXI_CLK",
	"GCC_CAMERA_XO_CLK",
	"GCC_CE1_AHB_CLK",
	"GCC_CE1_AXI_CLK",
	"GCC_CE1_CLK",
	"GCC_CFG_NOC_USB3_PRIM_AXI_CLK",
	"GCC_CFG_NOC_USB3_SEC_AXI_CLK",
	"GCC_CPUSS_AHB_CLK",
	"GCC_CPUSS_AHB_CLK_SRC",
	"GCC_CPUSS_RBCPR_CLK",
	"GCC_CPUSS_RBCPR_CLK_SRC",
	"GCC_DDRSS_GPU_AXI_CLK",
	"GCC_DISP_AHB_CLK",
	"GCC_DISP_AXI_CLK",
	"GCC_DISP_GPLL0_CLK_SRC",
	"GCC_DISP_GPLL0_DIV_CLK_SRC",
	"GCC_DISP_XO_CLK",
	"GCC_GP1_CLK",
	"GCC_GP1_CLK_SRC",
	"GCC_GP2_CLK",
	"GCC_GP2_CLK_SRC",
	"GCC_GP3_CLK",
	"GCC_GP3_CLK_SRC",
	"GCC_GPU_CFG_AHB_CLK",
	"GCC_GPU_GPLL0_CLK_SRC",
	"GCC_GPU_GPLL0_DIV_CLK_SRC",
	"GCC_GPU_MEMNOC_GFX_CLK",
	"GCC_GPU_SNOC_DVM_GFX_CLK",
	"GCC_MSS_AXIS2_CLK",
	"GCC_MSS_CFG_AHB_CLK",
	"GCC_MSS_GPLL0_DIV_CLK_SRC",
	"GCC_MSS_MFAB_AXIS_CLK",
	"GCC_MSS_Q6_MEMNOC_AXI_CLK",
	"GCC_MSS_SNOC_AXI_CLK",
	"GCC_PCIE_0_AUX_CLK",
	"GCC_PCIE_0_AUX_CLK_SRC",
	"GCC_PCIE_0_CFG_AHB_CLK",
	"GCC_PCIE_0_CLKREF_CLK",
	"GCC_PCIE_0_MSTR_AXI_CLK",
	"GCC_PCIE_0_PIPE_CLK",
	"GCC_PCIE_0_SLV_AXI_CLK",
	"GCC_PCIE_0_SLV_Q2A_AXI_CLK",
	"GCC_PCIE_1_AUX_CLK",
	"GCC_PCIE_1_AUX_CLK_SRC",
	"GCC_PCIE_1_CFG_AHB_CLK",
	"GCC_PCIE_1_CLKREF_CLK",
	"GCC_PCIE_1_MSTR_AXI_CLK",
	"GCC_PCIE_1_PIPE_CLK",
	"GCC_PCIE_1_SLV_AXI_CLK",
	"GCC_PCIE_1_SLV_Q2A_AXI_CLK",
	"GCC_PCIE_PHY_AUX_CLK",
	"GCC_PCIE_PHY_REFGEN_CLK",
	"GCC_PCIE_PHY_REFGEN_CLK_SRC",
	"GCC_PDM2_CLK",
	"GCC_PDM2_CLK_SRC",
	"GCC_PDM_AHB_CLK",
	"GCC_PDM_XO4_CLK",
	"GCC_PRNG_AHB_CLK",
	"GCC_QMIP_CAMERA_AHB_CLK",
	"GCC_QMIP_DISP_AHB_CLK",
	"GCC_QMIP_VIDEO_AHB_CLK",
	"GCC_QUPV3_WRAP0_S0_CLK",
	"GCC_QUPV3_WRAP0_S0_CLK_SRC",
	"GCC_QUPV3_WRAP0_S1_CLK",
	"GCC_QUPV3_WRAP0_S1_CLK_SRC",
	"GCC_QUPV3_WRAP0_S2_CLK",
	"GCC_QUPV3_WRAP0_S2_CLK_SRC",
	"GCC_QUPV3_WRAP0_S3_CLK",
	"GCC_QUPV3_WRAP0_S3_CLK_SRC",
	"GCC_QUPV3_WRAP0_S4_CLK",
	"GCC_QUPV3_WRAP0_S4_CLK_SRC",
	"GCC_QUPV3_WRAP0_S5_CLK",
	"GCC_QUPV3_WRAP0_S5_CLK_SRC",
	"GCC_QUPV3_WRAP0_S6_CLK",
	"GCC_QUPV3_WRAP0_S6_CLK_SRC",
	"GCC_QUPV3_WRAP0_S7_CLK",
	"GCC_QUPV3_WRAP0_S7_CLK_SRC",
	"GCC_QUPV3_WRAP1_S0_CLK",
	"GCC_QUPV3_WRAP1_S0_CLK_SRC",
	"GCC_QUPV3_WRAP1_S1_CLK",
	"GCC_QUPV3_WRAP1_S1_CLK_SRC",
	"GCC_QUPV3_WRAP1_S2_CLK",
	"GCC_QUPV3_WRAP1_S2_CLK_SRC",
	"GCC_QUPV3_WRAP1_S3_CLK",
	"GCC_QUPV3_WRAP1_S3_CLK_SRC",
	"GCC_QUPV3_WRAP1_S4_CLK",
	"GCC_QUPV3_WRAP1_S4_CLK_SRC",
	"GCC_QUPV3_WRAP1_S5_CLK",
	"GCC_QUPV3_WRAP1_S5_CLK_SRC",
	"GCC_QUPV3_WRAP1_S6_CLK",
	"GCC_QUPV3_WRAP1_S6_CLK_SRC",
	"GCC_QUPV3_WRAP1_S7_CLK",
	"GCC_QUPV3_WRAP1_S7_CLK_SRC",
	"GCC_QUPV3_WRAP_0_M_AHB_CLK",
	"GCC_QUPV3_WRAP_0_S_AHB_CLK",
	"GCC_QUPV3_WRAP_1_M_AHB_CLK",
	"GCC_QUPV3_WRAP_1_S_AHB_CLK",
	"GCC_SDCC2_AHB_CLK",
	"GCC_SDCC2_APPS_CLK",
	"GCC_SDCC2_APPS_CLK_SRC",
	"GCC_SDCC4_AHB_CLK",
	"GCC_SDCC4_APPS_CLK",
	"GCC_SDCC4_APPS_CLK_SRC",
	"GCC_SYS_NOC_CPUSS_AHB_CLK",
	"GCC_TSIF_AHB_CLK",
	"GCC_TSIF_INACTIVITY_TIMERS_CLK",
	"GCC_TSIF_REF_CLK",
	"GCC_TSIF_REF_CLK_SRC",
	"GCC_UFS_CARD_AHB_CLK",
	"GCC_UFS_CARD_AXI_CLK",
	"GCC_UFS_CARD_AXI_CLK_SRC",
	"GCC_UFS_CARD_CLKREF_CLK",
	"GCC_UFS_CARD_ICE_CORE_CLK",
	"GCC_UFS_CARD_ICE_CORE_CLK_SRC",
	"GCC_UFS_CARD_PHY_AUX_CLK",
	"GCC_UFS_CARD_PHY_AUX_CLK_SRC",
	"GCC_UFS_CARD_RX_SYMBOL_0_CLK",
	"GCC_UFS_CARD_RX_SYMBOL_1_CLK",
	"GCC_UFS_CARD_TX_SYMBOL_0_CLK",
	"GCC_UFS_CARD_UNIPRO_CORE_CLK",
	"GCC_UFS_CARD_UNIPRO_CORE_CLK_SRC",
	"GCC_UFS_MEM_CLKREF_CLK",
	"GCC_UFS_PHY_AHB_CLK",
	"GCC_UFS_PHY_AXI_CLK",
	"GCC_UFS_PHY_AXI_CLK_SRC",
	"GCC_UFS_PHY_ICE_CORE_CLK",
	"GCC_UFS_PHY_ICE_CORE_CLK_SRC",
	"GCC_UFS_PHY_PHY_AUX_CLK",
	"GCC_UFS_PHY_PHY_AUX_CLK_SRC",
	"GCC_UFS_PHY_RX_SYMBOL_0_CLK",
	"GCC_UFS_PHY_RX_SYMBOL_1_CLK",
	"GCC_UFS_PHY_TX_SYMBOL_0_CLK",
	"GCC_UFS_PHY_UNIPRO_CORE_CLK",
	"GCC_UFS_PHY_UNIPRO_CORE_CLK_SRC",
	"GCC_USB30_PRIM_MASTER_CLK",
	"GCC_USB30_PRIM_MASTER_CLK_SRC",
	"GCC_USB30_PRIM_MOCK_UTMI_CLK",
	"GCC_USB30_PRIM_MOCK_UTMI_CLK_SRC",
	"GCC_USB30_PRIM_SLEEP_CLK",
	"GCC_USB30_SEC_MASTER_CLK",
	"GCC_USB30_SEC_MASTER_CLK_SRC",
	"GCC_USB30_SEC_MOCK_UTMI_CLK",
	"GCC_USB30_SEC_MOCK_UTMI_CLK_SRC",
	"GCC_USB30_SEC_SLEEP_CLK",
	"GCC_USB3_PRIM_CLKREF_CLK",
	"GCC_USB3_PRIM_PHY_AUX_CLK",
	"GCC_USB3_PRIM_PHY_AUX_CLK_SRC",
	"GCC_USB3_PRIM_PHY_COM_AUX_CLK",
	"GCC_USB3_PRIM_PHY_PIPE_CLK",
	"GCC_USB3_SEC_CLKREF_CLK",
	"GCC_USB3_SEC_PHY_AUX_CLK",
	"GCC_USB3_SEC_PHY_AUX_CLK_SRC",
	"GCC_USB3_SEC_PHY_PIPE_CLK",
	"GCC_USB3_SEC_PHY_COM_AUX_CLK",
	"GCC_USB_PHY_CFG_AHB2PHY_CLK",
	"GCC_VIDEO_AHB_CLK",
	"GCC_VIDEO_AXI_CLK",
	"GCC_VIDEO_XO_CLK",
	"GPLL0",
	"GPLL0_OUT_EVEN",
	"GPLL0_OUT_MAIN",
	"GCC_GPU_IREF_CLK",
	"GCC_SDCC1_AHB_CLK",
	"GCC_SDCC1_APPS_CLK",
	"GCC_SDCC1_ICE_CORE_CLK",
	"GCC_SDCC1_APPS_CLK_SRC",
	"GCC_SDCC1_ICE_CORE_CLK_SRC",
	"GCC_APC_VS_CLK",
	"GCC_GPU_VS_CLK",
	"GCC_MSS_VS_CLK",
	"GCC_VDDA_VS_CLK",
	"GCC_VDDCX_VS_CLK",
	"GCC_VDDMX_VS_CLK",
	"GCC_VS_CTRL_AHB_CLK",
	"GCC_VS_CTRL_CLK",
	"GCC_VS_CTRL_CLK_SRC",
	"GCC_VSENSOR_CLK_SRC",
	"GPLL4",
	"GCC_CPUSS_DVM_BUS_CLK",
	"GCC_CPUSS_GNOC_CLK",
	"GCC_QSPI_CORE_CLK_SRC",
	"GCC_QSPI_CORE_CLK",
	"GCC_QSPI_CNOC_PERIPH_AHB_CLK",
	"GCC_LPASS_Q6_AXI_CLK",
	"GCC_LPASS_SWAY_CLK",
};

static const struct freq_tbl ftbl_gcc_qupv3_wrap0_s0_clk_src[] = {
	F(7372800, CFG_CLK_SRC_GPLL0_EVEN, 1, 384, 15625),
	F(14745600, CFG_CLK_SRC_GPLL0_EVEN, 1, 768, 15625),
	F(19200000, CFG_CLK_SRC_CXO, 1, 0, 0),
	F(29491200, CFG_CLK_SRC_GPLL0_EVEN, 1, 1536, 15625),
	F(32000000, CFG_CLK_SRC_GPLL0_EVEN, 1, 8, 75),
	F(48000000, CFG_CLK_SRC_GPLL0_EVEN, 1, 4, 25),
	F(64000000, CFG_CLK_SRC_GPLL0_EVEN, 1, 16, 75),
	F(80000000, CFG_CLK_SRC_GPLL0_EVEN, 1, 4, 15),
	F(96000000, CFG_CLK_SRC_GPLL0_EVEN, 1, 8, 25),
	F(100000000, CFG_CLK_SRC_GPLL0_EVEN, 3, 0, 0),
	F(102400000, CFG_CLK_SRC_GPLL0_EVEN, 1, 128, 375),
	F(112000000, CFG_CLK_SRC_GPLL0_EVEN, 1, 28, 75),
	F(117964800, CFG_CLK_SRC_GPLL0_EVEN, 1, 6144, 15625),
	F(120000000, CFG_CLK_SRC_GPLL0_EVEN, 2.5, 0, 0),
	F(128000000, CFG_CLK_SRC_GPLL0, 1, 16, 75),
	{ }
};

static const struct bcr_regs uart2_regs = {
	.cfg_rcgr = SE9_UART_APPS_CFG_RCGR,
	.cmd_rcgr = SE9_UART_APPS_CMD_RCGR,
	.M = SE9_UART_APPS_M,
	.N = SE9_UART_APPS_N,
	.D = SE9_UART_APPS_D,
};

static struct pll_vote_clk gcc_ufs_mem_clkref_clk = {
	.status = GCC_UFS_MEM_CLKREF_EN,
	.ena_vote = GCC_UFS_MEM_CLKREF_EN,
	.vote_bit = BIT(0),
	.status_bit = BIT(0),
};

static struct pll_vote_clk gcc_ufs_card_clkref_clk = {
	.status = GCC_UFS_CARD_CLKREF_EN,
	.ena_vote = GCC_UFS_CARD_CLKREF_EN,
	.vote_bit = BIT(0),
	.status_bit = BIT(0),
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

	clk_rcg_set_rate_mnd(priv->regmap, &uart2_regs,
						freq->pre_div, freq->m, freq->n, freq->src);

	return 0;
}

ulong msm_set_rate(struct clk *clk, ulong rate)
{
	struct msm_clk_priv *priv = dev_get_priv(clk->dev);

	printf("%s: clk %s, rate %lu\n", __func__, gcc_clk_names[clk->id], rate);

	switch (clk->id) {
	case GCC_QUPV3_WRAP1_S1_CLK: /*UART2*/
		return clk_init_uart(priv, rate);
	default:
		return 0;
	}

	return 0;
}

int msm_enable(struct clk *clk)
{
	struct msm_clk_priv *priv = dev_get_priv(clk->dev);

	//dev_info(clk->dev, "clk %s\n", gcc_clk_names[clk->id]);

	// if (clk->id != GCC_UFS_PHY_AXI_CLK) {
	// 	printf("Ignoring clk %s\n", gcc_clk_names[clk->id]);
	// 	return 0;
	// }

	switch (clk->id) {
	/* UFS controller clocks */
	case GCC_UFS_PHY_AXI_CLK:
		clk_enable_cbc(priv->regmap, GCC_UFS_PHY_AXI_CBCR);
		break;
	case GCC_AGGRE_UFS_PHY_AXI_CLK:
		clk_enable_cbc(priv->regmap, GCC_AGGRE_UFS_PHY_AXI_CBCR);
		break;
	case GCC_UFS_PHY_AHB_CLK:
		clk_enable_cbc(priv->regmap, GCC_UFS_PHY_AHB_CBCR);
		break;
	case GCC_UFS_PHY_UNIPRO_CORE_CLK:
		clk_enable_cbc(priv->regmap, GCC_UFS_PHY_UNIPRO_CORE_CBCR);
		break;
	// These are broken, either don't enable or cause a hang not sure which
	// we don't need to enable them explicitly anyway...
	case GCC_UFS_PHY_TX_SYMBOL_0_CLK:
		//clk_enable_cbc(priv->regmap, GCC_UFS_PHY_TX_SYMBOL_0_CBCR);
		break;
	case GCC_UFS_PHY_RX_SYMBOL_0_CLK:
		//clk_enable_cbc(priv->regmap, GCC_UFS_PHY_RX_SYMBOL_0_CBCR);
		break;
	case GCC_UFS_PHY_RX_SYMBOL_1_CLK:
		//clk_enable_cbc(priv->regmap, GCC_UFS_PHY_RX_SYMBOL_1_CBCR);
		break;
	case GCC_UFS_PHY_ICE_CORE_CLK:
		//clk_enable_cbc(priv->regmap, GCC_UFS_PHY_ICE_CORE_CBCR);
		break;
	/* UFS PHY clocks */
	case GCC_UFS_MEM_CLKREF_CLK:
		clk_enable_gpll0(priv->regmap, &gcc_ufs_mem_clkref_clk);
		break;
	case GCC_UFS_CARD_CLKREF_CLK:
		clk_enable_gpll0(priv->regmap, &gcc_ufs_card_clkref_clk);
		break;
	case GCC_UFS_PHY_PHY_AUX_CLK:
		// Crashdumps :<
		//clk_enable_cbc(priv->regmap, GCC_UFS_PHY_PHY_AUX_CBCR);
		break;
	default:
		break;
	}

	return 0;
}
