// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Bhupesh Sharma <bhupesh.sharma@linaro.org>
 *
 * Based on Linux driver
 */

#include <common.h>
#include <dm.h>
#include <linux/bitops.h>
#include <power/pmic.h>
#include <power/regulator.h>

struct qcom_rpm_reg {
	struct device *dev;

	struct qcom_smd_rpm *rpm;

	u32 type;
	u32 id;

	struct regulator_desc desc;

	int is_enabled;
	int uV;
	u32 load;

	unsigned int enabled_updated:1;
	unsigned int uv_updated:1;
	unsigned int load_updated:1;
};

struct rpm_regulator_req {
	__le32 key;
	__le32 nbytes;
	__le32 value;
};

#define RPM_KEY_SWEN	0x6e657773 /* "swen" */
#define RPM_KEY_UV	0x00007675 /* "uv" */
#define RPM_KEY_MA	0x0000616d /* "ma" */

static int rpm_reg_write_active(struct qcom_rpm_reg *vreg)
{
	struct rpm_regulator_req req[3];
	int reqlen = 0;
	int ret;

	if (vreg->enabled_updated) {
		req[reqlen].key = cpu_to_le32(RPM_KEY_SWEN);
		req[reqlen].nbytes = cpu_to_le32(sizeof(u32));
		req[reqlen].value = cpu_to_le32(vreg->is_enabled);
		reqlen++;
	}

	if (vreg->uv_updated && vreg->is_enabled) {
		req[reqlen].key = cpu_to_le32(RPM_KEY_UV);
		req[reqlen].nbytes = cpu_to_le32(sizeof(u32));
		req[reqlen].value = cpu_to_le32(vreg->uV);
		reqlen++;
	}

	if (vreg->load_updated && vreg->is_enabled) {
		req[reqlen].key = cpu_to_le32(RPM_KEY_MA);
		req[reqlen].nbytes = cpu_to_le32(sizeof(u32));
		req[reqlen].value = cpu_to_le32(vreg->load / 1000);
		reqlen++;
	}

	if (!reqlen)
		return 0;

	ret = qcom_rpm_smd_write(vreg->rpm, QCOM_SMD_RPM_ACTIVE_STATE,
				 vreg->type, vreg->id,
				 req, sizeof(req[0]) * reqlen);
	if (!ret) {
		vreg->enabled_updated = 0;
		vreg->uv_updated = 0;
		vreg->load_updated = 0;
	}

	return ret;
}

static int rpm_reg_enable(struct regulator_dev *rdev)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);
	int ret;

	vreg->is_enabled = 1;
	vreg->enabled_updated = 1;

	ret = rpm_reg_write_active(vreg);
	if (ret)
		vreg->is_enabled = 0;

	return ret;
}

static int rpm_reg_is_enabled(struct regulator_dev *rdev)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);

	return vreg->is_enabled;
}

static int rpm_reg_disable(struct regulator_dev *rdev)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);
	int ret;

	vreg->is_enabled = 0;
	vreg->enabled_updated = 1;

	ret = rpm_reg_write_active(vreg);
	if (ret)
		vreg->is_enabled = 1;

	return ret;
}

static int rpm_reg_get_voltage(struct regulator_dev *rdev)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);

	return vreg->uV;
}

static int rpm_reg_set_voltage(struct regulator_dev *rdev,
			       int min_uV,
			       int max_uV,
			       unsigned *selector)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);
	int ret;
	int old_uV = vreg->uV;

	vreg->uV = min_uV;
	vreg->uv_updated = 1;

	ret = rpm_reg_write_active(vreg);
	if (ret)
		vreg->uV = old_uV;

	return ret;
}

static int rpm_reg_set_load(struct regulator_dev *rdev, int load_uA)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);
	u32 old_load = vreg->load;
	int ret;

	vreg->load = load_uA;
	vreg->load_updated = 1;
	ret = rpm_reg_write_active(vreg);
	if (ret)
		vreg->load = old_load;

	return ret;
}

static const struct regulator_desc pm6125_ftsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(300000, 0, 268, 4000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 269,
	.ops = &rpm_smps_ldo_ops,
};

struct rpm_regulator_data {
	const char *name;
	u32 type;
	u32 id;
	const struct regulator_desc *desc;
	const char *supply;
};

static const struct rpm_regulator_data rpm_pm6125_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pm6125_ftsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pm6125_ftsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pm6125_ftsmps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPA, 4, &pm6125_ftsmps, "vdd_s4" },
	{ "s5", QCOM_SMD_RPM_SMPA, 5, &pm8998_hfsmps, "vdd_s5" },
	{ "s6", QCOM_SMD_RPM_SMPA, 6, &pm8998_hfsmps, "vdd_s6" },
	{ "s7", QCOM_SMD_RPM_SMPA, 7, &pm8998_hfsmps, "vdd_s7" },
	{ "s8", QCOM_SMD_RPM_SMPA, 8, &pm6125_ftsmps, "vdd_s8" },
	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pm660_nldo660, "vdd_l1_l7_l17_l18" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pm660_nldo660, "vdd_l2_l3_l4" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pm660_nldo660, "vdd_l2_l3_l4" },
	{ "l4", QCOM_SMD_RPM_LDOA, 4, &pm660_nldo660, "vdd_l2_l3_l4" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pm660_pldo660, "vdd_l5_l15_l19_l20_l21_l22" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pm660_nldo660, "vdd_l6_l8" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pm660_nldo660, "vdd_l1_l7_l17_l18" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pm660_nldo660, "vdd_l6_l8" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pm660_ht_lvpldo, "vdd_l9_l11" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm660_ht_lvpldo, "vdd_l10_l13_l14" },
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm660_ht_lvpldo, "vdd_l9_l11" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm660_ht_lvpldo, "vdd_l12_l16" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm660_ht_lvpldo, "vdd_l10_l13_l14" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm660_ht_lvpldo, "vdd_l10_l13_l14" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm660_pldo660, "vdd_l5_l15_l19_l20_l21_l22" },
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pm660_ht_lvpldo, "vdd_l12_l16" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm660_nldo660, "vdd_l1_l7_l17_l18" },
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pm660_nldo660, "vdd_l1_l7_l17_l18" },
	{ "l19", QCOM_SMD_RPM_LDOA, 19, &pm660_pldo660, "vdd_l5_l15_l19_l20_l21_l22" },
	{ "l20", QCOM_SMD_RPM_LDOA, 20, &pm660_pldo660, "vdd_l5_l15_l19_l20_l21_l22" },
	{ "l21", QCOM_SMD_RPM_LDOA, 21, &pm660_pldo660, "vdd_l5_l15_l19_l20_l21_l22" },
	{ "l22", QCOM_SMD_RPM_LDOA, 22, &pm660_pldo660, "vdd_l5_l15_l19_l20_l21_l22" },
	{ "l23", QCOM_SMD_RPM_LDOA, 23, &pm660_pldo660, "vdd_l23_l24" },
	{ "l24", QCOM_SMD_RPM_LDOA, 24, &pm660_pldo660, "vdd_l23_l24" },
	{ }
};

/**
 * rpm_regulator_init_vreg() - initialize all attributes of a qcom_smd-regulator
 * @vreg:		Pointer to the individual qcom_smd-regulator resource
 * @dev:		Pointer to the top level qcom_smd-regulator PMIC device
 * @node:		Pointer to the individual qcom_smd-regulator resource
 *			device node
 * @rpm:		Pointer to the rpm bus node
 * @pmic_rpm_data:	Pointer to a null-terminated array of qcom_smd-regulator
 *			resources defined for the top level PMIC device
 *
 * Return: 0 on success, errno on failure
 */
static int rpm_regulator_init_vreg(struct qcom_rpm_reg *vreg, struct device *dev,
				   struct device_node *node, struct qcom_smd_rpm *rpm,
				   const struct rpm_regulator_data *pmic_rpm_data)
{
	struct regulator_config config = {};
	const struct rpm_regulator_data *rpm_data;
	struct regulator_dev *rdev;
	int ret;

	for (rpm_data = pmic_rpm_data; rpm_data->name; rpm_data++)
		if (of_node_name_eq(node, rpm_data->name))
			break;

	if (!rpm_data->name) {
		dev_err(dev, "Unknown regulator %pOFn\n", node);
		return -EINVAL;
	}

	vreg->dev	= dev;
	vreg->rpm	= rpm;
	vreg->type	= rpm_data->type;
	vreg->id	= rpm_data->id;

	memcpy(&vreg->desc, rpm_data->desc, sizeof(vreg->desc));
	vreg->desc.name = rpm_data->name;
	vreg->desc.supply_name = rpm_data->supply;
	vreg->desc.owner = THIS_MODULE;
	vreg->desc.type = REGULATOR_VOLTAGE;
	vreg->desc.of_match = rpm_data->name;

	config.dev		= dev;
	config.of_node		= node;
	config.driver_data	= vreg;

	rdev = devm_regulator_register(dev, &vreg->desc, &config);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(dev, "%pOFn: devm_regulator_register() failed, ret=%d\n", node, ret);
		return ret;
	}

	return 0;
}

static int rpm_reg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct rpm_regulator_data *vreg_data;
	struct device_node *node;
	struct qcom_rpm_reg *vreg;
	struct qcom_smd_rpm *rpm;
	int ret;

	rpm = dev_get_drvdata(pdev->dev.parent);
	if (!rpm) {
		dev_err(&pdev->dev, "Unable to retrieve handle to rpm\n");
		return -ENODEV;
	}

	vreg_data = of_device_get_match_data(dev);
	if (!vreg_data)
		return -ENODEV;

	for_each_available_child_of_node(dev->of_node, node) {
		vreg = devm_kzalloc(&pdev->dev, sizeof(*vreg), GFP_KERNEL);
		if (!vreg) {
			of_node_put(node);
			return -ENOMEM;
		}

		ret = rpm_regulator_init_vreg(vreg, dev, node, rpm, vreg_data);

		if (ret < 0) {
			of_node_put(node);
			return ret;
		}
	}

	return 0;
}

static int qcom_smd_regulator_probe(struct udevice *dev)
{
	struct dm_regulator_uclass_plat *uc_pdata;
	struct qcom_smd_priv *priv = dev_get_priv(dev);

	/* LDOs are named numerically in DT so can directly index */
	if (dev->driver_data < 1 ||
	    dev->driver_data > ARRAY_SIZE(qcom_smd_ldo_info))
		return -EINVAL;
	priv->reg_info = &qcom_smd_ldo_info[dev->driver_data - 1];

	uc_pdata = dev_get_uclass_plat(dev);
	uc_pdata->type = REGULATOR_TYPE_LDO;
	if (uc_pdata->type == REGULATOR_TYPE_BUCK) {
		uc_pdata->mode = pfuze_sw_modes;
		uc_pdata->mode_count = ARRAY_SIZE(pfuze_sw_modes);
	} else if (uc_pdata->type == REGULATOR_TYPE_LDO) {
		uc_pdata->mode = pfuze_ldo_modes;
		uc_pdata->mode_count = ARRAY_SIZE(pfuze_ldo_modes);
	} else {
		uc_pdata->mode = NULL;
		uc_pdata->mode_count = 0;
	}

	return 0;
}

static const struct dm_regulator_ops qcom_smd_regulator_ops = {
	.get_value  = qcom_smd_get_voltage,
	.set_value  = qcom_smd_set_voltage,
	.get_enable = qcom_smd_get_enable,
	.set_enable = qcom_smd_set_enable,
	.get_mode   = qcom_smd_get_mode,
	.set_mode   = qcom_smd_set_mode,
};

static const struct udevice_id qcom_smd_regulator_ids[] = {
	{ .compatible = "qcom,rpm-pm6125-regulators", .data = (ulong)&rpm_pm6125_regulators },
	{}
};

U_BOOT_DRIVER(qcom_smd_regulator) = {
	.name = "qcom_smd_regulator",
	.id = UCLASS_REGULATOR,
	.of_match = qcom_smd_regulator_ids,
	.ops = &qcom_smd_regulator_ops,
	.probe = qcom_smd_regulator_probe,
	.priv_auto = sizeof(struct qcom_smd_priv),
};
