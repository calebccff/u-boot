// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Bhupesh Sharma <bhupesh.sharma@linaro.org>
 *
 * Based on Linux driver
 */

#include <common.h>
#include <dm.h>
#include <generic-phy.h>
#include <linux/bitops.h>
#include <asm/io.h>
#include <reset.h>
#include <clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>

#include "phy-qcom-qmp.h"
#include "phy-qcom-qmp-pcs-misc-v3.h"

/* QPHY_SW_RESET bit */
#define SW_RESET				BIT(0)
/* QPHY_POWER_DOWN_CONTROL */
#define SW_PWRDN				BIT(0)
/* QPHY_START_CONTROL bits */
#define SERDES_START				BIT(0)
#define PCS_START				BIT(1)
/* QPHY_PCS_STATUS bit */
#define PHYSTATUS				BIT(6)

/* QPHY_V3_DP_COM_RESET_OVRD_CTRL register bits */
/* DP PHY soft reset */
#define SW_DPPHY_RESET				BIT(0)
/* mux to select DP PHY reset control, 0:HW control, 1: software reset */
#define SW_DPPHY_RESET_MUX			BIT(1)
/* USB3 PHY soft reset */
#define SW_USB3PHY_RESET			BIT(2)
/* mux to select USB3 PHY reset control, 0:HW control, 1: software reset */
#define SW_USB3PHY_RESET_MUX			BIT(3)

/* QPHY_V3_DP_COM_PHY_MODE_CTRL register bits */
#define USB3_MODE				BIT(0) /* enables USB3 mode */
#define DP_MODE					BIT(1) /* enables DP mode */

/* QPHY_PCS_AUTONOMOUS_MODE_CTRL register bits */
#define ARCVR_DTCT_EN				BIT(0)
#define ALFPS_DTCT_EN				BIT(1)
#define ARCVR_DTCT_EVENT_SEL			BIT(4)

/* QPHY_PCS_LFPS_RXTERM_IRQ_CLEAR register bits */
#define IRQ_CLEAR				BIT(0)

/* QPHY_V3_PCS_MISC_CLAMP_ENABLE register bits */
#define CLAMP_EN				BIT(0) /* enables i/o clamp_n */

#define PHY_INIT_COMPLETE_TIMEOUT		10000

struct qmp_phy_init_tbl {
	unsigned int offset;
	unsigned int val;
	/*
	 * mask of lanes for which this register is written
	 * for cases when second lane needs different values
	 */
	u8 lane_mask;
};

#define QMP_PHY_INIT_CFG(o, v)		\
	{				\
		.offset = o,		\
		.val = v,		\
		.lane_mask = 0xff,	\
	}

#define QMP_PHY_INIT_CFG_LANE(o, v, l)	\
	{				\
		.offset = o,		\
		.val = v,		\
		.lane_mask = l,		\
	}

/* set of registers with offsets different per-PHY */
enum qphy_reg_layout {
	/* PCS registers */
	QPHY_SW_RESET,
	QPHY_START_CTRL,
	QPHY_PCS_STATUS,
	QPHY_PCS_AUTONOMOUS_MODE_CTRL,
	QPHY_PCS_LFPS_RXTERM_IRQ_CLEAR,
	QPHY_PCS_POWER_DOWN_CONTROL,
	/* Keep last to ensure regs_layout arrays are properly initialized */
	QPHY_LAYOUT_SIZE
};

static const unsigned int qmp_v3_usb3phy_regs_layout[QPHY_LAYOUT_SIZE] = {
	[QPHY_SW_RESET]			= QPHY_V3_PCS_SW_RESET,
	[QPHY_START_CTRL]		= QPHY_V3_PCS_START_CONTROL,
	[QPHY_PCS_STATUS]		= QPHY_V3_PCS_PCS_STATUS,
	[QPHY_PCS_AUTONOMOUS_MODE_CTRL]	= QPHY_V3_PCS_AUTONOMOUS_MODE_CTRL,
	[QPHY_PCS_LFPS_RXTERM_IRQ_CLEAR] = QPHY_V3_PCS_LFPS_RXTERM_IRQ_CLEAR,
	[QPHY_PCS_POWER_DOWN_CONTROL]	= QPHY_V3_PCS_POWER_DOWN_CONTROL,
};

static const struct qmp_phy_init_tbl qcm2290_usb3_serdes_tbl[] = {
	QMP_PHY_INIT_CFG(QSERDES_COM_SYSCLK_EN_SEL, 0x14),
	QMP_PHY_INIT_CFG(QSERDES_COM_BIAS_EN_CLKBUFLR_EN, 0x08),
	QMP_PHY_INIT_CFG(QSERDES_COM_CLK_SELECT, 0x30),
	QMP_PHY_INIT_CFG(QSERDES_COM_SYS_CLK_CTRL, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_COM_RESETSM_CNTRL, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_RESETSM_CNTRL2, 0x08),
	QMP_PHY_INIT_CFG(QSERDES_COM_BG_TRIM, 0x0f),
	QMP_PHY_INIT_CFG(QSERDES_COM_SVS_MODE_CLK_SEL, 0x01),
	QMP_PHY_INIT_CFG(QSERDES_COM_HSCLK_SEL, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_DEC_START_MODE0, 0x82),
	QMP_PHY_INIT_CFG(QSERDES_COM_DIV_FRAC_START1_MODE0, 0x55),
	QMP_PHY_INIT_CFG(QSERDES_COM_DIV_FRAC_START2_MODE0, 0x55),
	QMP_PHY_INIT_CFG(QSERDES_COM_DIV_FRAC_START3_MODE0, 0x03),
	QMP_PHY_INIT_CFG(QSERDES_COM_CP_CTRL_MODE0, 0x0b),
	QMP_PHY_INIT_CFG(QSERDES_COM_PLL_RCTRL_MODE0, 0x16),
	QMP_PHY_INIT_CFG(QSERDES_COM_PLL_CCTRL_MODE0, 0x28),
	QMP_PHY_INIT_CFG(QSERDES_COM_INTEGLOOP_GAIN0_MODE0, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_COM_INTEGLOOP_GAIN1_MODE0, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_CORECLK_DIV, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP1_MODE0, 0x15),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP2_MODE0, 0x34),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP3_MODE0, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP_EN, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_CORE_CLK_EN, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP_CFG, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_VCO_TUNE_MAP, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_BG_TIMER, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_COM_SSC_EN_CENTER, 0x01),
	QMP_PHY_INIT_CFG(QSERDES_COM_SSC_PER1, 0x31),
	QMP_PHY_INIT_CFG(QSERDES_COM_SSC_PER2, 0x01),
	QMP_PHY_INIT_CFG(QSERDES_COM_SSC_ADJ_PER1, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_SSC_ADJ_PER2, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_SSC_STEP_SIZE1, 0xde),
	QMP_PHY_INIT_CFG(QSERDES_COM_SSC_STEP_SIZE2, 0x07),
	QMP_PHY_INIT_CFG(QSERDES_COM_PLL_IVCO, 0x0f),
	QMP_PHY_INIT_CFG(QSERDES_COM_CMN_CONFIG, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_COM_INTEGLOOP_INITVAL, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_COM_BIAS_EN_CTRL_BY_PSM, 0x01),
};

static const struct qmp_phy_init_tbl qcm2290_usb3_tx_tbl[] = {
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_HIGHZ_DRVR_EN, 0x10),
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_RCV_DETECT_LVL_2, 0x12),
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_LANE_MODE_1, 0xc6),
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_RES_CODE_LANE_OFFSET_TX, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_RES_CODE_LANE_OFFSET_RX, 0x00),
};

static const struct qmp_phy_init_tbl qcm2290_usb3_rx_tbl[] = {
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_FASTLOCK_FO_GAIN, 0x0b),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_PI_CONTROLS, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_FASTLOCK_COUNT_LOW, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_FASTLOCK_COUNT_HIGH, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_FO_GAIN, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_SO_GAIN, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_SO_SATURATION_AND_ENABLE, 0x75),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_EQU_ADAPTOR_CNTRL2, 0x02),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_EQU_ADAPTOR_CNTRL3, 0x4e),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_EQU_ADAPTOR_CNTRL4, 0x18),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_EQ_OFFSET_ADAPTOR_CNTRL1, 0x77),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_OFFSET_ADAPTOR_CNTRL2, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_VGA_CAL_CNTRL2, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_SIGDET_CNTRL, 0x03),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_SIGDET_DEGLITCH_CNTRL, 0x16),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_SIGDET_ENABLES, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_MODE_00, 0x00),
};

static const struct qmp_phy_init_tbl qcm2290_usb3_pcs_tbl[] = {
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_TXMGN_V0, 0x9f),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_TXDEEMPH_M6DB_V0, 0x17),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_TXDEEMPH_M3P5DB_V0, 0x0f),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_FLL_CNTRL2, 0x83),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_FLL_CNTRL1, 0x02),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_FLL_CNT_VAL_L, 0x09),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_FLL_CNT_VAL_H_TOL, 0xa2),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_FLL_MAN_CODE, 0x85),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_LOCK_DETECT_CONFIG1, 0xd1),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_LOCK_DETECT_CONFIG2, 0x1f),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_LOCK_DETECT_CONFIG3, 0x47),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RXEQTRAINING_WAIT_TIME, 0x75),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RXEQTRAINING_RUN_TIME, 0x13),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_LFPS_TX_ECSTART_EQTLOCK, 0x86),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_PWRUP_RESET_DLY_TIME_AUXCLK, 0x04),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_TSYNC_RSYNC_TIME, 0x44),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RCVR_DTCT_DLY_P1U2_L, 0xe7),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RCVR_DTCT_DLY_P1U2_H, 0x03),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RCVR_DTCT_DLY_U3_L, 0x40),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RCVR_DTCT_DLY_U3_H, 0x00),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RX_SIGDET_LVL, 0x88),
};

struct qmp_usb_offsets {
	u16 serdes;
	u16 pcs;
	u16 pcs_misc;
	u16 pcs_usb;
	u16 tx;
	u16 rx;
	/* for PHYs with >= 2 lanes */
	u16 tx2;
	u16 rx2;
};

/* struct qmp_phy_cfg - per-PHY initialization config */
struct qmp_phy_cfg {
	int lanes;

	const struct qmp_usb_offsets *offsets;

	/* Init sequence for PHY blocks - serdes, tx, rx, pcs */
	const struct qmp_phy_init_tbl *serdes_tbl;
	int serdes_tbl_num;
	const struct qmp_phy_init_tbl *tx_tbl;
	int tx_tbl_num;
	const struct qmp_phy_init_tbl *rx_tbl;
	int rx_tbl_num;
	const struct qmp_phy_init_tbl *pcs_tbl;
	int pcs_tbl_num;
	const struct qmp_phy_init_tbl *pcs_usb_tbl;
	int pcs_usb_tbl_num;

	/* clock ids to be requested */
	const char * const *clk_list;
	int num_clks;
	/* resets to be requested */
	const char * const *reset_list;
	int num_resets;
	/* regulators to be requested */
	const char * const *vreg_list;
	int num_vregs;

	/* array of registers with different offsets */
	const unsigned int *regs;

	/* true, if PHY needs delay after POWER_DOWN */
	bool has_pwrdn_delay;

	/* true, if PHY has a separate DP_COM control block */
	bool has_phy_dp_com_ctrl;

	/* Offset from PCS to PCS_USB region */
	unsigned int pcs_usb_offset;
};

struct qmp_phy_priv {
	struct phy *phy;

	void __iomem *base;
	void __iomem *serdes;
	void __iomem *pcs;
	void __iomem *pcs_misc;
	void __iomem *pcs_usb;
	void __iomem *tx;
	void __iomem *rx;
	void __iomem *tx2;
	void __iomem *rx2;

	void __iomem *dp_com;

	struct clk *pipe_clk;
	unsigned long pipe_clk_rate;
	struct clk_bulk clks;

	struct reset_ctl_bulk resets;

	const struct qmp_phy_cfg *cfg;
};

static inline void qphy_setbits(void __iomem *base, u32 offset, u32 val)
{
	u32 reg;

	reg = readl(base + offset);
	reg |= val;
	writel(reg, base + offset);

	/* ensure that above write is through */
	readl(base + offset);
}

static inline void qphy_clrbits(void __iomem *base, u32 offset, u32 val)
{
	u32 reg;

	reg = readl(base + offset);
	reg &= ~val;
	writel(reg, base + offset);

	/* ensure that above write is through */
	readl(base + offset);
}

/* list of clocks required by phy */
static const char * const qcm2290_usb3phy_clk_l[] = {
	"cfg_ahb", "ref", "com_aux",
};

/* list of resets */
static const char * const qcm2290_usb3phy_reset_l[] = {
	"phy_phy", "phy",
};

/* list of regulators */
static const char * const qmp_phy_vreg_l[] = {
	"vdda-phy", "vdda-pll",
};

static const struct qmp_usb_offsets qmp_usb_offsets_v3 = {
	.serdes		= 0,
	.pcs		= 0xc00,
	.pcs_misc	= 0xa00,
	.tx		= 0x200,
	.rx		= 0x400,
	.tx2		= 0x600,
	.rx2		= 0x800,
};

static const struct qmp_phy_cfg qcm2290_usb3phy_cfg = {
	.lanes			= 2,

	.offsets		= &qmp_usb_offsets_v3,

	.serdes_tbl		= qcm2290_usb3_serdes_tbl,
	.serdes_tbl_num		= ARRAY_SIZE(qcm2290_usb3_serdes_tbl),
	.tx_tbl			= qcm2290_usb3_tx_tbl,
	.tx_tbl_num		= ARRAY_SIZE(qcm2290_usb3_tx_tbl),
	.rx_tbl			= qcm2290_usb3_rx_tbl,
	.rx_tbl_num		= ARRAY_SIZE(qcm2290_usb3_rx_tbl),
	.pcs_tbl		= qcm2290_usb3_pcs_tbl,
	.pcs_tbl_num		= ARRAY_SIZE(qcm2290_usb3_pcs_tbl),
	.clk_list		= qcm2290_usb3phy_clk_l,
	.num_clks		= ARRAY_SIZE(qcm2290_usb3phy_clk_l),
	.reset_list		= qcm2290_usb3phy_reset_l,
	.num_resets		= ARRAY_SIZE(qcm2290_usb3phy_reset_l),
	.vreg_list		= qmp_phy_vreg_l,
	.num_vregs		= ARRAY_SIZE(qmp_phy_vreg_l),
	.regs			= qmp_v3_usb3phy_regs_layout,
};

static void qmp_usb_configure_lane(void __iomem *base,
					const struct qmp_phy_init_tbl tbl[],
					int num,
					u8 lane_mask)
{
	int i;
	const struct qmp_phy_init_tbl *t = tbl;

	if (!t)
		return;

	for (i = 0; i < num; i++, t++) {
		if (!(t->lane_mask & lane_mask))
			continue;

		writel(t->val, base + t->offset);
	}
}

static void qmp_usb_configure(void __iomem *base,
				   const struct qmp_phy_init_tbl tbl[],
				   int num)
{
	qmp_usb_configure_lane(base, tbl, num, 0xff);
}

static int qmp_usb_serdes_init(struct qmp_phy_priv *qmp)
{
	const struct qmp_phy_cfg *cfg = qmp->cfg;
	void __iomem *serdes = qmp->serdes;
	const struct qmp_phy_init_tbl *serdes_tbl = cfg->serdes_tbl;
	int serdes_tbl_num = cfg->serdes_tbl_num;

	qmp_usb_configure(serdes, serdes_tbl, serdes_tbl_num);

	return 0;
}

static int qmp_usb_init(struct phy *phy)
{
	struct qmp_phy_priv *qmp = dev_get_priv(phy->dev);
	const struct qmp_phy_cfg *cfg = qmp->cfg;
	void __iomem *pcs = qmp->pcs;
	void __iomem *dp_com = qmp->dp_com;

	if (cfg->has_phy_dp_com_ctrl) {
		qphy_setbits(dp_com, QPHY_V3_DP_COM_POWER_DOWN_CTRL,
			     SW_PWRDN);
		/* override hardware control for reset of qmp phy */
		qphy_setbits(dp_com, QPHY_V3_DP_COM_RESET_OVRD_CTRL,
			     SW_DPPHY_RESET_MUX | SW_DPPHY_RESET |
			     SW_USB3PHY_RESET_MUX | SW_USB3PHY_RESET);

		/* Default type-c orientation, i.e CC1 */
		qphy_setbits(dp_com, QPHY_V3_DP_COM_TYPEC_CTRL, 0x02);

		qphy_setbits(dp_com, QPHY_V3_DP_COM_PHY_MODE_CTRL,
			     USB3_MODE | DP_MODE);

		/* bring both QMP USB and QMP DP PHYs PCS block out of reset */
		qphy_clrbits(dp_com, QPHY_V3_DP_COM_RESET_OVRD_CTRL,
			     SW_DPPHY_RESET_MUX | SW_DPPHY_RESET |
			     SW_USB3PHY_RESET_MUX | SW_USB3PHY_RESET);

		qphy_clrbits(dp_com, QPHY_V3_DP_COM_SWI_CTRL, 0x03);
		qphy_clrbits(dp_com, QPHY_V3_DP_COM_SW_RESET, SW_RESET);
	}

	qphy_setbits(pcs, cfg->regs[QPHY_PCS_POWER_DOWN_CONTROL], SW_PWRDN);

	return 0;
}

static int qmp_phy_do_reset(struct qmp_phy_priv *qmp)
{
	int ret;

	ret = reset_assert_bulk(&qmp->resets);
	if (ret)
		return ret;

	udelay(10);

	ret = reset_deassert_bulk(&qmp->resets);
	if (ret)
		return ret;

	return 0;
}

static int qmp_phy_power_on(struct phy *phy)
{
	struct qmp_phy_priv *qmp = dev_get_priv(phy->dev);
	const struct qmp_phy_cfg *cfg = qmp->cfg;
	void __iomem *tx = qmp->tx;
	void __iomem *rx = qmp->rx;
	void __iomem *pcs = qmp->pcs;
	void __iomem *status;
	unsigned int val;
	int ret;

	ret = qmp_phy_do_reset(qmp);
	if (ret)
		return ret;
	
	qmp_usb_serdes_init(qmp);

	ret = clk_prepare_enable(qmp->pipe_clk);
	if (ret)
		return ret;

	/* Tx, Rx, and PCS configurations */
	qmp_usb_configure_lane(tx, cfg->tx_tbl, cfg->tx_tbl_num, 1);
	qmp_usb_configure_lane(rx, cfg->rx_tbl, cfg->rx_tbl_num, 1);

	if (cfg->lanes >= 2) {
		qmp_usb_configure_lane(qmp->tx2, cfg->tx_tbl, cfg->tx_tbl_num, 2);
		qmp_usb_configure_lane(qmp->rx2, cfg->rx_tbl, cfg->rx_tbl_num, 2);
	}

	qmp_usb_configure(pcs, cfg->pcs_tbl, cfg->pcs_tbl_num);

	if (cfg->has_pwrdn_delay)
		udelay(20);

	/* Pull PHY out of reset state */
	qphy_clrbits(pcs, cfg->regs[QPHY_SW_RESET], SW_RESET);

	/* start SerDes and Phy-Coding-Sublayer */
	qphy_setbits(pcs, cfg->regs[QPHY_START_CTRL], SERDES_START | PCS_START);

	status = pcs + cfg->regs[QPHY_PCS_STATUS];
	ret = readl_poll_timeout(status, val, !(val & PHYSTATUS), PHY_INIT_COMPLETE_TIMEOUT);
	if (ret)
		return ret;

	ret = qmp_usb_init(phy);
	if (ret)
		return ret;

	return 0;
}

static int qmp_phy_power_off(struct phy *phy)
{
	struct qmp_phy_priv *qmp = dev_get_priv(phy->dev);
	const struct qmp_phy_cfg *cfg = qmp->cfg;

	/* PHY reset */
	qphy_setbits(qmp->pcs, cfg->regs[QPHY_SW_RESET], SW_RESET);

	/* stop SerDes and Phy-Coding-Sublayer */
	qphy_clrbits(qmp->pcs, cfg->regs[QPHY_START_CTRL],
			SERDES_START | PCS_START);

	/* Put PHY into POWER DOWN state: active low */
	qphy_clrbits(qmp->pcs, cfg->regs[QPHY_PCS_POWER_DOWN_CONTROL],
			SW_PWRDN);

	clk_disable_unprepare(qmp->pipe_clk);

	clk_disable_bulk(&qmp->clks);
	clk_release_bulk(&qmp->clks);

	return 0;
}

static int qmp_phy_clk_init(struct udevice *dev, struct qmp_phy_priv *phy)
{
	int ret;

	ret = clk_get_bulk(dev, &phy->clks);
	if (ret == -ENOSYS || ret == -ENOENT)
		return 0;
	if (ret)
		return ret;

	ret = clk_enable_bulk(&phy->clks);
	if (ret) {
		clk_release_bulk(&phy->clks);
		return ret;
	}

	return 0;
}

/*
 * Register a fixed rate pipe clock.
 *
 * The <s>_pipe_clksrc generated by PHY goes to the GCC that gate
 * controls it. The <s>_pipe_clk coming out of the GCC is requested
 * by the PHY driver for its operations.
 * We register the <s>_pipe_clksrc here. The gcc driver takes care
 * of assigning this <s>_pipe_clksrc as parent to <s>_pipe_clk.
 * Below picture shows this relationship.
 *
 *         +---------------+
 *         |   PHY block   |<<---------------------------------------+
 *         |               |                                         |
 *         |   +-------+   |                   +-----+               |
 *   I/P---^-->|  PLL  |---^--->pipe_clksrc--->| GCC |--->pipe_clk---+
 *    clk  |   +-------+   |                   +-----+
 *         +---------------+
 */
static int qmp_phy_probe(struct udevice *dev)
{
	struct qmp_phy_priv *qmp = dev_get_priv(dev);
	int ret;

	qmp->base = (void __iomem *)dev_read_addr(dev);
	if (IS_ERR(qmp->base))
		return PTR_ERR(qmp->base);

	ret = qmp_phy_clk_init(dev, qmp);
	if (ret)
		return ret;

	ret = reset_get_bulk(dev, &qmp->resets);
	if (ret)
		return ret;

	qmp->cfg = (const struct qmp_phy_cfg *)dev_get_driver_data(dev);
	if (!qmp->cfg)
		return -EINVAL;

	qmp->serdes = qmp->base + qmp->cfg->offsets->serdes;
	qmp->pcs = qmp->base + qmp->cfg->offsets->pcs;
	qmp->pcs_misc = qmp->base + qmp->cfg->offsets->pcs_misc;
	qmp->pcs_usb = qmp->base + qmp->cfg->offsets->pcs_usb;
	qmp->tx = qmp->base + qmp->cfg->offsets->tx;
	qmp->rx = qmp->base + qmp->cfg->offsets->rx;

	if (qmp->cfg->lanes >= 2) {
		qmp->tx2 = qmp->base + qmp->cfg->offsets->tx2;
		qmp->rx2 = qmp->base + qmp->cfg->offsets->rx2;
	}

	clk_get_by_name(dev, "pipe", qmp->pipe_clk);
	
	/* controllers using QMP phys use 125MHz pipe clock interface */
	clk_set_rate(qmp->pipe_clk, 125000000);

	return 0;
}

static struct phy_ops qmp_phy_ops = {
	.power_on = qmp_phy_power_on,
	.power_off = qmp_phy_power_off,
};

static const struct udevice_id qmp_phy_ids[] = {
	{ .compatible = "qcom,qmp-usb3-phy" },
	{ .compatible = "qcom,sm6115-qmp-usb3-phy", .data = (ulong)&qcm2290_usb3phy_cfg },
	{ }
};

U_BOOT_DRIVER(qcom_qmp_usb) = {
	.name		= "qcom-qmp-usb",
	.id		= UCLASS_PHY,
	.of_match	= qmp_phy_ids,
	.ops		= &qmp_phy_ops,
	.probe		= qmp_phy_probe,
	.priv_auto	= sizeof(struct qmp_phy_priv),
};
