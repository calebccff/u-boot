/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Qualcomm SM8250 sysmap
 *
 * (C) Copyright 2023 Bhupesh Sharma <bhupesh.sharma@linaro.org>
 */
#ifndef _MACH_SYSMAP_SM8250_H
#define _MACH_SYSMAP_SM8250_H

/* Clocks: (from CLK_CTL_BASE)  */
#define GPLL0_STATUS			(0x0)
#define APCS_GPLL_ENA_VOTE		(0x79000)
#define APCS_CLOCK_BRANCH_ENA_VOTE	(0x79004)

/* Uart clock control registers */
#define DEBUG_UART_APPS_CBCR		(0x184CC)
#define DEBUG_UART_APPS_CMD_RCGR	(0x184D0)
#define DEBUG_UART_APPS_CFG_RCGR	(0x184D4)
#define DEBUG_UART_APPS_M		(0x184D8)
#define DEBUG_UART_APPS_N		(0x184DC)
#define DEBUG_UART_APPS_D		(0x184E0)

/* USB controller clock control registers */
#define USB30_SEC_MASTER_CBCR		(0x10010)
#define USB30_SEC_SLEEP_CBCR		(0x10018)
#define USB30_SEC_MOCK_UTMI_CBCR	(0x1001C)
#define CFG_NOC_USB3_SEC_AXI_CBCR	(0x1007C)
#define AGGRE_USB3_SEC_AXI_CBCR		(0x10080)
#define USB3_SEC_CLKREF_EN		(0x8C010)

#define USB3_SEC_PHY_AUX_CBCR		(0x10054)
#define USB3_SEC_PHY_COM_AUX_CBCR	(0x10058)
#define USB3_SEC_PHY_PIPE_CBCR		(0x1005C)
#define USB3_SEC_PHY_PIPE_MUXR		(0x10060)

#define USB30_SEC_MASTER_CMD_RCGR	(0x10020)
#define USB30_SEC_MASTER_CFG_RCGR	(0x10024)
#define USB30_SEC_MASTER_M		(0x10028)
#define USB30_SEC_MASTER_N		(0x1002C)
#define USB30_SEC_MASTER_D		(0x10030)

/* SDCC2 controller clock control registers */
#define SDCC2_BCR		(0x14000)
#define SDCC2_APPS_CBCR		(0x14004)
#define SDCC2_AHB_CBCR		(0x14008)
#define SDCC2_CMD_RCGR		(0x1400C)
#define SDCC2_CFG_RCGR		(0x14010)
#define SDCC2_M			(0x14014)
#define SDCC2_N			(0x14018)
#define SDCC2_D			(0x1401c)

#endif
