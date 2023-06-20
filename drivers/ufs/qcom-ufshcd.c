// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Bhupesh Sharma <bhupesh.sharma@linaro.org>
 *
 * Based on Linux driver
 */

#include <asm/io.h>
#include <clk.h>
#include <common.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <generic-phy.h>
#include <ufs.h>

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>

#include "ufs.h"
#include "ufs-qcom.h"

#define MCQ_QCFGPTR_MASK	GENMASK(7, 0)
#define MCQ_QCFGPTR_UNIT	0x200
#define MCQ_SQATTR_OFFSET(c) \
	((((c) >> 16) & MCQ_QCFGPTR_MASK) * MCQ_QCFGPTR_UNIT)
#define MCQ_QCFG_SIZE	0x40

#define MSEC_PER_SEC	(1000L)
#define USEC_PER_SEC	(1000000L)
#define NSEC_PER_SEC	(1000000000L)

enum {
	TSTBUS_UAWM,
	TSTBUS_UARM,
	TSTBUS_TXUC,
	TSTBUS_RXUC,
	TSTBUS_DFC,
	TSTBUS_TRLUT,
	TSTBUS_TMRLUT,
	TSTBUS_OCSC,
	TSTBUS_UTP_HCI,
	TSTBUS_COMBINED,
	TSTBUS_WRAPPER,
	TSTBUS_UNIPRO,
	TSTBUS_MAX,
};

static int ufs_qcom_set_dme_vs_core_clk_ctrl_clear_div(struct ufs_hba *hba,
						       u32 clk_cycles);

static int ufs_qcom_clk_get(struct udevice *dev,
		const char *name, struct clk **clk_out, bool optional)
{
	struct clk *clk;
	int err = 0;

	clk = devm_clk_get(dev, name);
	if (!IS_ERR(clk)) {
		*clk_out = clk;
		return 0;
	}

	err = PTR_ERR(clk);

	if (optional && err == -ENOENT) {
		*clk_out = NULL;
		return 0;
	}

	if (err != -EPROBE_DEFER)
		dev_err(dev, "failed to get %s err %d\n", name, err);

	return err;
}

static int ufs_qcom_clk_enable(struct udevice *dev,
		const char *name, struct clk *clk)
{
	int err = 0;

	err = clk_prepare_enable(clk);
	if (err)
		dev_err(dev, "%s: %s enable failed %d\n", __func__, name, err);

	return err;
}

static int ufs_qcom_enable_lane_clks(struct ufs_qcom_priv *priv)
{
	int err;
	struct udevice *dev = priv->hba->dev;

	printf("%s: Entered function\n", __func__);
	if (priv->is_lane_clks_enabled)
		return 0;

	err = ufs_qcom_clk_enable(dev, "rx_lane0_sync_clk",
		priv->rx_l0_sync_clk);
	if (err)
		return err;

	err = ufs_qcom_clk_enable(dev, "tx_lane0_sync_clk",
		priv->tx_l0_sync_clk);
	if (err)
		goto disable_rx_l0;

	/* In case of single lane per direction, don't read lane1 clocks */
	if (priv->hba->lanes_per_direction > 1) {
		err = ufs_qcom_clk_enable(dev, "rx_lane1_sync_clk",
				priv->rx_l1_sync_clk);
		if (err)
			goto disable_tx_l0;

		err = ufs_qcom_clk_enable(dev, "tx_lane1_sync_clk",
				priv->tx_l1_sync_clk);
		if (err)
			goto disable_rx_l1;
	}

	priv->is_lane_clks_enabled = true;

	printf("%s: Exiting function with success\n", __func__);
	return 0;

disable_rx_l1:
	clk_disable_unprepare(priv->rx_l1_sync_clk);
disable_tx_l0:
	clk_disable_unprepare(priv->tx_l0_sync_clk);
disable_rx_l0:
	clk_disable_unprepare(priv->rx_l0_sync_clk);

	printf("%s: Exiting function with error:%d\n", __func__, err);
	return err;
}

static int ufs_qcom_init_lane_clks(struct ufs_qcom_priv *priv)
{
	int err = 0;
	struct udevice *dev = priv->hba->dev;

	printf("%s: Entered function\n", __func__);
	err = ufs_qcom_clk_get(dev, "rx_lane0_sync_clk",
					&priv->rx_l0_sync_clk, false);
	if (err)
		return err;

	err = ufs_qcom_clk_get(dev, "tx_lane0_sync_clk",
					&priv->tx_l0_sync_clk, false);
	if (err)
		return err;

	/* In case of single lane per direction, don't read lane1 clocks */
	if (priv->hba->lanes_per_direction > 1) {
		err = ufs_qcom_clk_get(dev, "rx_lane1_sync_clk",
			&priv->rx_l1_sync_clk, false);
		if (err)
			return err;

		err = ufs_qcom_clk_get(dev, "tx_lane1_sync_clk",
			&priv->tx_l1_sync_clk, true);
	}

	printf("%s: Exiting function with success\n", __func__);
	return 0;
}

static int ufs_qcom_enable_core_clks(struct ufs_qcom_priv *priv)
{
	int err;
	struct udevice *dev = priv->hba->dev;

	printf("%s: Entered function\n", __func__);
	if (priv->is_core_clks_enabled)
		return 0;

	err = ufs_qcom_clk_enable(dev, "core_clk", priv->core_clk);
	if (err)
		return err;

	err = ufs_qcom_clk_enable(dev, "bus_aggr_clk", priv->bus_aggr_clk);
	if (err)
		goto disable_core_clk;

	err = ufs_qcom_clk_enable(dev, "iface_clk", priv->iface_clk);
	if (err)
		goto disable_bus_aggr_clk;

	priv->is_core_clks_enabled = true;

	printf("%s: Exiting function with success\n", __func__);
	return 0;

disable_bus_aggr_clk:
	clk_disable_unprepare(priv->bus_aggr_clk);
disable_core_clk:
	clk_disable_unprepare(priv->core_clk);

	printf("%s: Exiting function with error, ret = %d\n", __func__, err);
	return err;
}

static int ufs_qcom_init_core_clks(struct ufs_qcom_priv *priv)
{
	int err = 0;
	struct udevice *dev = priv->hba->dev;

	printf("%s: Entered function\n", __func__);
	err = ufs_qcom_clk_get(dev, "core_clk",
					&priv->core_clk, false);
	if (err)
		return err;

	err = ufs_qcom_clk_get(dev, "bus_aggr_clk",
					&priv->bus_aggr_clk, false);
	if (err)
		return err;

	err = ufs_qcom_clk_get(dev, "iface_clk", &priv->iface_clk, false);
	if (err)
		return err;

#if 0
	err = ufs_qcom_clk_get(dev, "ref_clk", &priv->ref_clk, true); /* optional */
	if (err)
		return err;
#endif

	printf("%s: Exiting function with success\n", __func__);
	return 0;
}

static void ufs_qcom_select_unipro_mode(struct ufs_qcom_priv *priv)
{
	printf("%s: Entered function\n", __func__);
	ufshcd_rmwl(priv->hba, QUNIPRO_SEL,
		   ufs_qcom_cap_qunipro(priv) ? QUNIPRO_SEL : 0,
		   REG_UFS_CFG1);

	if (priv->hw_ver.major == 0x05)
		ufshcd_rmwl(priv->hba, QUNIPRO_G4_SEL, 0, REG_UFS_CFG0);

	/* make sure above configuration is applied before we return */
	mb();
	printf("%s: Exiting function with success\n", __func__);
}

/*
 * ufs_qcom_reset - reset host controller and PHY
 */
static int ufs_qcom_reset(struct ufs_hba *hba)
{
	struct ufs_qcom_priv *priv = dev_get_priv(hba->dev);
	int ret = 0;

	printf("%s: Entered function\n", __func__);
	ret = reset_assert(&priv->core_reset);
	if (ret) {
		dev_err(hba->dev, "%s: core_reset assert failed, err = %d\n",
				 __func__, ret);
		return ret;
	}

	/*
	 * The hardware requirement for delay between assert/deassert
	 * is at least 3-4 sleep clock (32.7KHz) cycles, which comes to
	 * ~125us (4/32768). To be on the safe side add 200us delay.
	 */
	udelay(210);

	ret = reset_deassert(&priv->core_reset);
	if (ret)
		dev_err(hba->dev, "%s: core_reset deassert failed, err = %d\n",
				 __func__, ret);

	udelay(1100);

	printf("%s: Exiting function with success\n", __func__);
	return 0;
}

/**
 * ufs_qcom_advertise_quirks - advertise the known QCOM UFS controller quirks
 * @hba: host controller instance
 *
 * QCOM UFS host controller might have some non standard behaviours (quirks)
 * than what is specified by UFSHCI specification. Advertise all such
 * quirks to standard UFS host controller driver so standard takes them into
 * account.
 */
static void ufs_qcom_advertise_quirks(struct ufs_hba *hba)
{
	struct ufs_qcom_priv *priv = dev_get_priv(hba->dev);

	printf("%s: Entered function\n", __func__);
	if (priv->hw_ver.major == 0x01) {
		hba->quirks |= UFSHCD_QUIRK_DELAY_BEFORE_DME_CMDS
			    | UFSHCD_QUIRK_BROKEN_PA_RXHSUNTERMCAP
			    | UFSHCD_QUIRK_DME_PEER_ACCESS_AUTO_MODE;

		if (priv->hw_ver.minor == 0x0001 && priv->hw_ver.step == 0x0001)
			hba->quirks |= UFSHCD_QUIRK_BROKEN_INTR_AGGR;

		hba->quirks |= UFSHCD_QUIRK_BROKEN_LCC;
	}

	if (priv->hw_ver.major == 0x2) {
		hba->quirks |= UFSHCD_QUIRK_BROKEN_UFS_HCI_VERSION;

		if (!ufs_qcom_cap_qunipro(priv))
			/* Legacy UniPro mode still need following quirks */
			hba->quirks |= (UFSHCD_QUIRK_DELAY_BEFORE_DME_CMDS
				| UFSHCD_QUIRK_DME_PEER_ACCESS_AUTO_MODE
				| UFSHCD_QUIRK_BROKEN_PA_RXHSUNTERMCAP);
	}

	if (priv->hw_ver.major > 0x3)
		hba->quirks |= UFSHCD_QUIRK_REINIT_AFTER_MAX_GEAR_SWITCH;
	printf("%s: Exiting function with success\n", __func__);
}

/**
 * ufs_qcom_setup_clocks - enables/disable clocks
 * @hba: host controller instance
 * @on: If true, enable clocks else disable them.
 * @status: PRE_CHANGE or POST_CHANGE notify
 *
 * Returns 0 on success, non-zero on failure.
 */
static int ufs_qcom_setup_clocks(struct ufs_hba *hba, bool on,
				 enum ufs_notify_change_status status)
{
	printf("%s: Entered function\n", __func__);
	switch (status) {
	case PRE_CHANGE:
		break;
	case POST_CHANGE:
		break;
	}

	printf("%s: Exiting function with success\n", __func__);
	return 0;
}

static int ufs_qcom_power_up_sequence(struct ufs_hba *hba)
{
	struct ufs_qcom_priv *priv = dev_get_priv(hba->dev);
	int ret;

	printf("%s: Entered function\n", __func__);
	/* Reset UFS Host Controller and PHY */
	ret = ufs_qcom_reset(hba);
	if (ret)
		dev_warn(hba->dev, "%s: host reset returned %d\n",
				  __func__, ret);

	ufs_qcom_select_unipro_mode(priv);

	printf("%s: Exiting function with success\n", __func__);
	return 0;
}

/*
 * The UTP controller has a number of internal clock gating cells (CGCs).
 * Internal hardware sub-modules within the UTP controller control the CGCs.
 * Hardware CGCs disable the clock to inactivate UTP sub-modules not involved
 * in a specific operation, UTP controller CGCs are by default disabled and
 * this function enables them (after every UFS link startup) to save some power
 * leakage.
 */
static void ufs_qcom_enable_hw_clk_gating(struct ufs_hba *hba)
{
	ufshcd_writel(hba,
		ufshcd_readl(hba, REG_UFS_CFG2) | REG_UFS_CFG2_CGC_EN_ALL,
		REG_UFS_CFG2);

	/* Ensure that HW clock gating is enabled before next operations */
	mb();
}

static int ufs_qcom_hce_enable_notify(struct ufs_hba *hba,
				      enum ufs_notify_change_status status)
{
	struct ufs_qcom_priv *priv = dev_get_priv(hba->dev);
	int err = 0;

	printf("%s: Entered function\n", __func__);
	switch (status) {
	case PRE_CHANGE:
		ufs_qcom_power_up_sequence(hba);
		/*
		 * The PHY PLL output is the source of tx/rx lane symbol
		 * clocks, hence, enable the lane clocks only after PHY
		 * is initialized.
		 */
		err = ufs_qcom_enable_lane_clks(priv);
		break;
	case POST_CHANGE:
		/* check if UFS PHY moved from DISABLED to HIBERN8 */
		ufs_qcom_enable_hw_clk_gating(hba);
		break;
	default:
		dev_err(hba->dev, "%s: invalid status %d\n", __func__, status);
		err = -EINVAL;
		break;
	}
	
	printf("%s: Exiting function with success, err:%d\n", __func__, err);
	return err;
}

/*
 * Returns zero for success and non-zero in case of a failure
 */
static int ufs_qcom_cfg_timers(struct ufs_hba *hba, u32 gear,
			       u32 hs, u32 rate, bool update_link_startup_timer)
{
	struct ufs_qcom_priv *priv = dev_get_priv(hba->dev);
	u32 core_clk_period_in_ns;
	u32 tx_clk_cycles_per_us = 0;
	unsigned long core_clk_rate = 0;
	u32 core_clk_cycles_per_us = 0;

	static u32 pwm_fr_table[][2] = {
		{UFS_PWM_G1, 0x1},
		{UFS_PWM_G2, 0x1},
		{UFS_PWM_G3, 0x1},
		{UFS_PWM_G4, 0x1},
	};

	static u32 hs_fr_table_rA[][2] = {
		{UFS_HS_G1, 0x1F},
		{UFS_HS_G2, 0x3e},
		{UFS_HS_G3, 0x7D},
	};

	static u32 hs_fr_table_rB[][2] = {
		{UFS_HS_G1, 0x24},
		{UFS_HS_G2, 0x49},
		{UFS_HS_G3, 0x92},
	};

	printf("%s: Entered function\n", __func__);
	
	/*
	 * The Qunipro controller does not use following registers:
	 * SYS1CLK_1US_REG, TX_SYMBOL_CLK_1US_REG, CLK_NS_REG &
	 * UFS_REG_PA_LINK_STARTUP_TIMER
	 * But UTP controller uses SYS1CLK_1US_REG register for Interrupt
	 * Aggregation logic.
	*/
	if (ufs_qcom_cap_qunipro(priv))
		return 0;

	if (gear == 0) {
		dev_err(hba->dev, "%s: invalid gear = %d\n", __func__, gear);
		return -EINVAL;
	}

	core_clk_rate = clk_get_rate(priv->core_clk);
	
	/* If frequency is smaller than 1MHz, set to 1MHz */
	if (core_clk_rate < DEFAULT_CLK_RATE_HZ)
		core_clk_rate = DEFAULT_CLK_RATE_HZ;

	core_clk_cycles_per_us = core_clk_rate / USEC_PER_SEC;
	if (ufshcd_readl(hba, REG_UFS_SYS1CLK_1US) != core_clk_cycles_per_us) {
		ufshcd_writel(hba, core_clk_cycles_per_us, REG_UFS_SYS1CLK_1US);
		/*
		 * make sure above write gets applied before we return from
		 * this function.
		 */
		mb();
	}

	if (ufs_qcom_cap_qunipro(priv))
		return 0;

	core_clk_period_in_ns = NSEC_PER_SEC / core_clk_rate;
	core_clk_period_in_ns <<= OFFSET_CLK_NS_REG;
	core_clk_period_in_ns &= MASK_CLK_NS_REG;

	switch (hs) {
	case FASTAUTO_MODE:
	case FAST_MODE:
		if (rate == PA_HS_MODE_A) {
			if (gear > ARRAY_SIZE(hs_fr_table_rA)) {
				dev_err(hba->dev,
					"%s: index %d exceeds table size %zu\n",
					__func__, gear,
					ARRAY_SIZE(hs_fr_table_rA));
				return -EINVAL;
			}
			tx_clk_cycles_per_us = hs_fr_table_rA[gear-1][1];
		} else if (rate == PA_HS_MODE_B) {
			if (gear > ARRAY_SIZE(hs_fr_table_rB)) {
				dev_err(hba->dev,
					"%s: index %d exceeds table size %zu\n",
					__func__, gear,
					ARRAY_SIZE(hs_fr_table_rB));
				return -EINVAL;
			}
			tx_clk_cycles_per_us = hs_fr_table_rB[gear-1][1];
		} else {
			dev_err(hba->dev, "%s: invalid rate = %d\n",
				__func__, rate);
			return -EINVAL;
		}
		break;
	case SLOWAUTO_MODE:
	case SLOW_MODE:
		if (gear > ARRAY_SIZE(pwm_fr_table)) {
			dev_err(hba->dev,
					"%s: index %d exceeds table size %zu\n",
					__func__, gear,
					ARRAY_SIZE(pwm_fr_table));
			return -EINVAL;
		}
		tx_clk_cycles_per_us = pwm_fr_table[gear-1][1];
		break;
	case UNCHANGED:
	default:
		dev_err(hba->dev, "%s: invalid mode = %d\n", __func__, hs);
		return -EINVAL;
	}

	if (ufshcd_readl(hba, REG_UFS_TX_SYMBOL_CLK_NS_US) !=
	    (core_clk_period_in_ns | tx_clk_cycles_per_us)) {
		/* this register 2 fields shall be written at once */
		ufshcd_writel(hba, core_clk_period_in_ns | tx_clk_cycles_per_us,
			      REG_UFS_TX_SYMBOL_CLK_NS_US);
		/*
		 * make sure above write gets applied before we return from
		 * this function.
		 */
		mb();
	}

	if (update_link_startup_timer && priv->hw_ver.major != 0x5) {
		ufshcd_writel(hba, ((core_clk_rate / MSEC_PER_SEC) * 100),
			      REG_UFS_CFG0);
		/*
		 * make sure that this configuration is applied before
		 * we return
		 */
		mb();
	}

	printf("%s: Exiting function with success\n", __func__);

	return 0;
}

static int ufs_qcom_set_dme_vs_core_clk_ctrl_clear_div(struct ufs_hba *hba,
						       u32 clk_cycles)
{
	int err;
	u32 core_clk_ctrl_reg;

	printf("%s: Entered function\n", __func__);
	if (clk_cycles > DME_VS_CORE_CLK_CTRL_MAX_CORE_CLK_1US_CYCLES_MASK)
		return -EINVAL;

	err = ufshcd_dme_get(hba,
			    UIC_ARG_MIB(DME_VS_CORE_CLK_CTRL),
			    &core_clk_ctrl_reg);
	if (err)
		return err;

	core_clk_ctrl_reg &= ~DME_VS_CORE_CLK_CTRL_MAX_CORE_CLK_1US_CYCLES_MASK;
	core_clk_ctrl_reg |= clk_cycles;

	/* Clear CORE_CLK_DIV_EN */
	core_clk_ctrl_reg &= ~DME_VS_CORE_CLK_CTRL_CORE_CLK_DIV_EN_BIT;

	printf("%s: Exiting function with success\n", __func__);
	return ufshcd_dme_set(hba,
			    UIC_ARG_MIB(DME_VS_CORE_CLK_CTRL),
			    core_clk_ctrl_reg);
}

static int ufs_qcom_link_startup_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status status)
{
	struct ufs_qcom_priv *priv = dev_get_priv(hba->dev);
	int err = 0;

	printf("%s: Entered function\n", __func__);
	switch (status) {
	case PRE_CHANGE:
		if (ufs_qcom_cfg_timers(hba, UFS_PWM_G1, SLOWAUTO_MODE,
					0, true)) {
			dev_err(hba->dev, "%s: ufs_qcom_cfg_timers() failed\n",
				__func__);
			return -EINVAL;
		}

		if (ufs_qcom_cap_qunipro(priv))
			/*
			 * set unipro core clock cycles to 150 & clear clock
			 * divider
			 */
			err = ufs_qcom_set_dme_vs_core_clk_ctrl_clear_div(hba,
									  150);
		break;
	default:
		break;
	}

	printf("%s: Exiting function with success\n", __func__);
	return err;
}

/**
 * ufs_qcom_init - bind phy with controller
 * @hba: host controller instance
 *
 * Powers up PHY enabling clocks and regulators.
 *
 * Returns -EPROBE_DEFER if binding fails, returns negative error
 * on phy power up failure and returns zero on success.
 */
static int ufs_qcom_init(struct ufs_hba *hba)
{
	struct ufs_qcom_priv *priv = dev_get_priv(hba->dev);
	struct phy phy;
	int err;

	printf("%s: Entered function\n", __func__);
	priv->hba = hba;

	/* get phy */
	err = generic_phy_get_by_name(hba->dev, "ufsphy", &phy);
	if (err) {
		dev_warn(hba->dev, "%s: Unable to get QMP ufs phy, ret = %d\n",
			 __func__, err);
		return err;
	}

	/* phy initialization */
	err = generic_phy_init(&phy);
	if (err) {
		dev_err(hba->dev, "%s: phy init failed, ret = %d\n",
			__func__, err);
		return err;
	}

	/* power on phy */
	err = generic_phy_power_on(&phy);
	if (err) {
		dev_err(hba->dev, "%s: phy power on failed, ret = %d\n",
			__func__, err);
		goto out_disable_phy;
	}

	ufs_qcom_get_controller_revision(hba, &priv->hw_ver.major,
		&priv->hw_ver.minor, &priv->hw_ver.step);
	dev_info(hba->dev, "Qcom UFS HC version: %d.%d.%d\n", priv->hw_ver.major,
		priv->hw_ver.minor, priv->hw_ver.step);

	err = ufs_qcom_init_core_clks(priv);
	if (err) {
		dev_err(hba->dev, "failed to initialize core clocks, err:%d\n", err);
		return err;
	}

	err = ufs_qcom_enable_core_clks(priv);
	if (err) {
		dev_err(hba->dev, "failed to enable core clocks, err:%d\n", err);
		return err;
	}

	err = ufs_qcom_init_lane_clks(priv);
	if (err) {
		dev_err(hba->dev, "failed to initialize lane clocks, err:%d\n", err);
		return err;
	}

	ufs_qcom_advertise_quirks(hba);
	ufs_qcom_setup_clocks(hba, true, POST_CHANGE);

	/*
	 * Power up the PHY using the minimum supported gear (UFS_HS_G2).
	 * Switching to max gear will be performed during reinit if supported.
	 */
	priv->hs_gear = UFS_HS_G2;

	printf("%s: Exiting function with success\n", __func__);
	return 0;

out_disable_phy:
	generic_phy_exit(&phy);

	printf("%s: Exiting function with error, ret = %d\n", __func__, err);
	return err;
}

static struct ufs_hba_ops ufs_qcom_hba_ops = {
	.init			= ufs_qcom_init,
	.hce_enable_notify	= ufs_qcom_hce_enable_notify,
	.link_startup_notify	= ufs_qcom_link_startup_notify,
};

static int ufs_qcom_probe(struct udevice *dev)
{
	struct ufs_qcom_priv *priv = dev_get_priv(dev);
	int ret;

	printf("%s: Entered function\n", __func__);

	/* get resets */
	ret = reset_get_by_name(dev, "rst", &priv->core_reset);
	if (ret) {
		dev_err(dev, "failed to get reset, ret:%d\n", ret);
		return ret;
	}

	ret = ufshcd_probe(dev, &ufs_qcom_hba_ops);
	if (ret) {
		dev_err(dev, "ufshcd_probe() failed, ret:%d\n", ret);
		return ret;
	}

	printf("%s: Exiting function successfully\n", __func__);
	return 0;
}

static int ufs_qcom_bind(struct udevice *dev)
{
	struct udevice *scsi_dev;

	return ufs_scsi_bind(dev, &scsi_dev);
}

static const struct udevice_id ufs_qcom_ids[] = {
	{ .compatible = "qcom,ufshc"},
	{},
};

U_BOOT_DRIVER(qcom_ufshcd) = {
	.name		= "qcom-ufshcd",
	.id		= UCLASS_UFS,
	.of_match	= ufs_qcom_ids,
	.probe		= ufs_qcom_probe,
	.bind		= ufs_qcom_bind,
	.priv_auto	= sizeof(struct ufs_qcom_priv),
};
