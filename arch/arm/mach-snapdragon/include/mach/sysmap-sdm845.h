/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Qualcomm SDM845 sysmap
 *
 * (C) Copyright 2021 Dzmitry Sankouski <dsankouski@gmail.com>
 */
#ifndef _MACH_SYSMAP_SDM845_H
#define _MACH_SYSMAP_SDM845_H

#define TLMM_BASE_ADDR			(0x1010000)

/* Strength (sdc1) */
#define SDC1_HDRV_PULL_CTL_REG		(TLMM_BASE_ADDR + 0x0012D000)

/* Clocks: (from CLK_CTL_BASE)  */
#define GPLL0_STATUS			(0x0000)
#define APCS_GPLL_ENA_VOTE		(0x52000)
#define APCS_CLOCK_BRANCH_ENA_VOTE	(0x52004)

#define SDCC2_BCR			(0x14000) /* block reset */
#define SDCC2_APPS_CBCR			(0x14004) /* branch control */
#define SDCC2_AHB_CBCR			(0x14008)
#define SDCC2_CMD_RCGR			(0x1400c)
#define SDCC2_CFG_RCGR			(0x14010)
#define SDCC2_M				(0x14014)
#define SDCC2_N				(0x14018)
#define SDCC2_D				(0x1401C)

#define RCG2_CFG_REG			0x4
#define M_REG			0x8
#define N_REG			0xc
#define D_REG			0x10

#define SE9_AHB_CBCR			(0x25004)
#define SE9_UART_APPS_CBCR		(0x29004)
#define SE9_UART_APPS_CMD_RCGR	(0x18148)
#define SE9_UART_APPS_CFG_RCGR	(0x1814C)
#define SE9_UART_APPS_M		(0x18150)
#define SE9_UART_APPS_N		(0x18154)
#define SE9_UART_APPS_D		(0x18158)

// Relative addresses (to CLK_CTL ie GCC)
#define GCC_UFS_PHY_AXI_CBCR		(0x07700C)
#define GCC_AGGRE_UFS_PHY_AXI_CBCR	(0x082024)
#define GCC_UFS_PHY_AHB_CBCR		(0x077010)
#define GCC_UFS_PHY_UNIPRO_CORE_CBCR	(0x077054)
#define GCC_UFS_PHY_TX_SYMBOL_0_CBCR 	(0x077014)
#define GCC_UFS_PHY_RX_SYMBOL_0_CBCR	(0x077018)
#define GCC_UFS_PHY_RX_SYMBOL_1_CBCR	(0x0770a8)
#define GCC_UFS_PHY_ICE_CORE_CBCR	(0x077058)
#define GCC_UFS_PHY_PHY_AUX_CBCR	(0x07708c)
#define GCC_UFS_MEM_CLKREF_EN		(0x08c000)
#define GCC_UFS_CARD_CLKREF_EN		(0x08c004)

#endif
