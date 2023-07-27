// SPDX-License-Identifier: GPL-2.0+
/*
 * Board init file for QRB5165-RB5
 *
 * (C) Copyright 2023 Bhupesh Sharma <bhupesh.sharma@linaro.org>
 */

#include <common.h>
#include <cpu_func.h>
#include <dm.h>
#include <env.h>
#include <init.h>
#include <asm/cache.h>
#include <asm/gpio.h>
#include <asm/global_data.h>
#include <fdt_support.h>
#include <asm/io.h>
#include <asm/psci.h>
#include <asm/arch/dram.h>

#include <linux/arm-smccc.h>
#include <linux/psci.h>

DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	return fdtdec_setup_mem_size_base();
}

#define MEM_REG1_BASE	(0x80000000UL)
#define MEM_REG1_SIZE	(0x3C900000UL)

#define MEM_REG2_BASE	(0xC0000000UL)
#define MEM_REG2_SIZE	(0x80000000UL)

#define MEM_REG3_BASE	(0x140000000UL)
#define MEM_REG3_SIZE	(0x0C0000000UL)

int dram_init_banksize(void)
{
#if 0
	gd->bd->bi_dram[0].start = MEM_REG1_BASE;
	gd->bd->bi_dram[0].size =  MEM_REG1_SIZE;

	gd->bd->bi_dram[1].start = MEM_REG2_BASE;
	gd->bd->bi_dram[1].size =  MEM_REG2_SIZE;

	gd->bd->bi_dram[2].start = MEM_REG3_BASE;
	gd->bd->bi_dram[2].size =  MEM_REG3_SIZE;

	return 0;
#else
	return fdtdec_setup_memory_banksize();
#endif
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

void set_smmu_bypass_mode(void)
{
	u32 reg = readl(ARM_SMMU_GR0_sCR0);
	printf("%s: Checking SMMU config. Initial sCR0 reg value is 0x%lx\n",
		       __func__, reg);

	/* bypass SMMU */
	reg &= ~ARM_SMMU_sCR0_USFCFG;
	writel(reg, ARM_SMMU_GR0_sCR0);

	printf("%s: Setting SMMU in bypass mode. Writing 0x%x at 0x%lx\n", __func__,
		       reg, ARM_SMMU_GR0_sCR0);
}

int board_init(void)
{
	show_psci_version();

	set_smmu_bypass_mode();
#if 0
	icache_disable();
	dcache_disable();
#endif

	return 0;
}

void reset_cpu(void)
{
	psci_system_reset();
}

/* Check for vol- and power buttons */
int misc_init_r(void)
{
	struct udevice *pon;
	struct gpio_desc resin;
	int node, ret;

	ret = uclass_get_device_by_name(UCLASS_GPIO, "pm8150_pon@800", &pon);
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

	/*
	 * search for kaslr address, set by primary bootloader by searching first
	 * 0x100 relocated bytes at u-boot's initial load address range
	 */
	uintptr_t start = gd->ram_base;
	uintptr_t end = start + 0x800000;
	u8 *addr = (u8 *)start;
	phys_addr_t *relocaddr = (phys_addr_t *)gd->relocaddr;
	u32 block_size = 0x1000;

	while (memcmp(addr, relocaddr, 0x100) && (uintptr_t)addr < end)
		addr += block_size;

	if ((uintptr_t)addr >= end)
		printf("KASLR not found in range 0x%lx - 0x%lx", start, end);
	else
		env_set_addr("KASLR", addr);

	return 0;
}
