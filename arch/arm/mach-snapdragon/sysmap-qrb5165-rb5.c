// SPDX-License-Identifier: GPL-2.0+
/*
 * Qualcomm SM8250 memory map
 *
 * (C) Copyright 2023 Bhupesh Sharma <bhupesh.sharma@linaro.org>
 */

#include <common.h>
#include <asm/armv8/mmu.h>

// static struct mm_region sm8250_mem_map[] = {
// 	{
// 		.virt = 0x0UL, /* Peripheral block */
// 		.phys = 0x0UL, /* Peripheral block */
// 		.size = 0x80000000UL,
// 		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
// 			 PTE_BLOCK_NON_SHARE |
// 			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
// 	}, {
// 		.virt = 0x80000000UL, /* DDR */
// 		.phys = 0x80000000UL, /* DDR */
// 		.size = 0x3B800000UL, /* GiB */
// 		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
// 			 PTE_BLOCK_INNER_SHARE
// 	}, {
// 		.virt = 0xC0000000UL, /* DDR */
// 		.phys = 0xC0000000UL, /* DDR */
// 		.size = 0xC0000000UL, /* GB */
// 		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL_NC) |
// 			 PTE_BLOCK_NON_SHARE
// 	}, {
// 		.virt = 0x180000000UL, /* DDR */
// 		.phys = 0x180000000UL, /* DDR */
// 		.size = 0x100000000UL, /* GB */
// 		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
// 			 PTE_BLOCK_INNER_SHARE
// 	},
// 	/* Terminator */
// 	{ 0 }
// };

static struct mm_region sdm845_mem_map[] = {
	{
		.virt = 0x0UL, /* Peripheral block */
		.phys = 0x0UL, /* Peripheral block */
		.size = 0x80000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		.virt = 0x80000000UL, /* DDR */
		.phys = 0x80000000UL, /* DDR */
		.size = 0x200000000UL, /* 8GiB - maximum allowed memory */
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = sdm845_mem_map;
