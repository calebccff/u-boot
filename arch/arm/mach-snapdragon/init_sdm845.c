// SPDX-License-Identifier: GPL-2.0+
/*
 * Common init part for boards based on SDM845
 *
 * (C) Copyright 2021 Dzmitry Sankouski <dsankouski@gmail.com>
 */

#include <init.h>
#include <env.h>
#include <fdt_support.h>
#include <efi_loader.h>
#include <common.h>
#include <asm/system.h>
#include <asm/gpio.h>
#include <dm.h>
#include <asm/io.h>
#include <asm/psci.h>
#include <linux/arm-smccc.h>
#include <linux/psci.h>

DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	return fdtdec_setup_mem_size_base();
}

int dram_init_banksize(void)
{
	return fdtdec_setup_memory_banksize();
}

static void show_psci_version(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(ARM_PSCI_0_2_FN_PSCI_VERSION, 0, 0, 0, 0, 0, 0, 0, &res);

	printf("PSCI:  v%ld.%ld\n",
	       PSCI_VERSION_MAJOR(res.a0),
		PSCI_VERSION_MINOR(res.a0));
}

#define ARM_SMMU_GR0_sCR0	(0x15000000UL)
#define ARM_SMMU_sCR0_USFCFG	BIT(10)
/* Stream mapping registers */
#define ARM_SMMU_GR0_SMR(n)		(0x800 + ((n) << 2))
#define ARM_SMMU_SMR_VALID		BIT(31)
#define ARM_SMMU_SMR_MASK		GENMASK(31, 16)
#define ARM_SMMU_SMR_ID			GENMASK(15, 0)

void smmu_init(void)
{
	// u32 reg = readl(ARM_SMMU_GR0_sCR0);
	// printf("%s: Checking SMMU config. Initial sCR0 reg value is 0x%x\n",
	// 	       __func__, reg);

	// icache_disable();
	// dcache_disable();
}

void reset_cpu(void)
{
	psci_system_reset();
}

__weak int board_init(void)
{
	show_psci_version();

	smmu_init();
	return 0;
}

struct board_info {
	char *display_match;
	char *pretty_name;
	char *codename;
	char *devicetree;
	int width;
	int height;
	unsigned int bootdev : 8;
	unsigned int bootpart : 8;
};

static const struct board_info sdm845_boards[] = {
	{
		.display_match = "dsi_lt9611_1080_video_display",
		.pretty_name = "Qualcomm RB3",
		.codename = "db845c",
		.devicetree = "/dtbs/qcom/sdm845-db845c.dtb",
		.width = 1920,
		.height = 1080,
		.bootdev = 4, // modem_a
		.bootpart = 4,
	}, {
		.display_match = "dsi_shift6mq_rm69299_1080p_video_display",
		.pretty_name = "SHIFT6mq",
		.codename = "axolotl",
		.devicetree = "/dtbs/qcom/sdm845-shift-axolotl.dtb",
		.width = 1080,
		.height = 2160,
		.bootdev = 0,
		.bootpart = 7, // super
	}, {
		.display_match = "dsi_samsung_sofef00_m_cmd_display", // XXX: CHECK
		.pretty_name = "OnePlus 6",
		.codename = "enchilada",
		.devicetree = "/dtbs/qcom/sdm845-oneplus-enchilada.dtb",
		.width = 1080,
		.height = 2280,
		.bootdev = 4,
		.bootpart = 4, // XXX: CHECK
	}
};

static const struct board_info *get_board(const char *display)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sdm845_boards); i++) {
		if (!strncmp(display, sdm845_boards[i].display_match, strlen(sdm845_boards[i].display_match)))
			return &sdm845_boards[i];
	}

	return NULL;
}

static int get_cmdline_option(const char *cmdline, const char *key, char *out, int out_len)
{
	const char *p, *p_end;
	int len;

	p = strstr(cmdline, key);
	if (!p)
		return -1;

	p += strlen(key);
	p_end = strstr(p, " ");
	if (!p_end)
		return -1;

	len = p_end - p;
	if (len > out_len)
		len = out_len;

	strncpy(out, p, len);
	out[len] = '\0';

	return 0;
}

#define DISPLAY_KEY "msm_drm.dsi_display0="
#define SLOT_SUFFIX_KEY "androidboot.slot_suffix="

static void init_sdm845_board(const char *cmdline)
{
	char display[64], slot[3];
	const struct board_info *board;

	if (get_cmdline_option(cmdline, DISPLAY_KEY, display, sizeof(display)) < 0) {
		printf("Failed to get display info from cmdline\n");
		return;
	}

	if (get_cmdline_option(cmdline, SLOT_SUFFIX_KEY, slot, sizeof(slot)) < 0) {
		printf("Failed to get slot suffix from cmdline\n");
		return;
	}

	board = get_board(display);
	if (!board) {
		printf("Failed to detect board for display: %s\n", display);
		return;
	}

	printf("Detected board: %s (%s)\n", board->pretty_name, board->codename);

	env_set("board", board->codename);
	env_set("devicetree", board->devicetree);
	env_set("boot_slot", &slot[1]);
	env_set("bootdev", simple_itoa(board->bootdev));
	env_set("bootpart", simple_itoa(board->bootpart));
}

/* Check for vol- and power buttons */
__weak int misc_init_r(void)
{
	struct udevice *pon;
	struct gpio_desc resin;
	int node, ret;

	ret = uclass_get_device_by_name(UCLASS_GPIO, "pm8998_pon@800", &pon);
	if (ret < 0) {
		printf("Failed to find PMIC pon node. Check device tree\n");
		return 0;
	}

	node = fdt_subnode_offset(gd->fdt_blob, dev_of_offset(pon),
				  "key_vol_down");
	if (node < 0) {
		printf("Failed to find key_vol_down node. Check device tree\n");
		return 0;
	}
	if (gpio_request_by_name_nodev(offset_to_ofnode(node), "gpios", 0,
				       &resin, 0)) {
		printf("Failed to request key_vol_down button.\n");
		return 0;
	}
	if (dm_gpio_get_value(&resin)) {
		env_set("key_vol_down", "1");
		printf("Volume down button pressed\n");
	} else {
		env_set("key_vol_down", "0");
	}

	node = fdt_subnode_offset(gd->fdt_blob, dev_of_offset(pon),
				  "key_power");
	if (node < 0) {
		printf("Failed to find key_power node. Check device tree\n");
		return 0;
	}
	if (gpio_request_by_name_nodev(offset_to_ofnode(node), "gpios", 0,
				       &resin, 0)) {
		printf("Failed to request key_power button.\n");
		return 0;
	}
	if (dm_gpio_get_value(&resin)) {
		env_set("key_power", "1");
		printf("Power button pressed\n");
	} else {
		env_set("key_power", "0");
	}

	/* Get KASLR seed from the FDT provided by ABL */
	const void *prevbl_fdt = (void *)env_get_ulong("prevbl_fdt_addr", 16, 0);
	if (!prevbl_fdt) {
		printf("ABL did not pass FDT to U-Boot\n");
		return 0;
	}
	node = fdt_path_offset(prevbl_fdt, "/chosen");
	if (!node) {
		printf("ABL did not pass chosen node to U-Boot\n");
		return 0;
	}

	const uint64_t *kaslr_seed = fdt_getprop(prevbl_fdt, node, "kaslr-seed", NULL);
	env_set_hex("KASLR", *kaslr_seed);

	const char *cmdline = fdt_getprop(prevbl_fdt, node, "bootargs", NULL);

	init_sdm845_board(cmdline);

	return 0;
}

int ft_board_setup(void *fdt, struct bd_info *bd)
{
	const char *slot;
	uint64_t kaslr_seed;
	int ret;

	slot = env_get("boot_slot");
	kaslr_seed = env_get_hex("KASLR", 0);

	/*
	 * We can't customise the cmdline that grub passes to the kernel,
	 * but we need to somehow pass on the boot slot property from ABL
	 */
	ret = fdt_find_and_setprop(fdt, "/chosen", "boot-slot", slot, strlen(slot), 1);
	if (ret < 0) {
		printf("Failed to set boot-slot property\n");
		return ret;
	}

	ret = fdt_find_and_setprop(fdt, "/chosen", "kaslr-seed", &kaslr_seed, sizeof(kaslr_seed), 1);
	if (ret < 0) {
		printf("Failed to set kaslr-seed property\n");
		return ret;
	}

	return 0;
}
