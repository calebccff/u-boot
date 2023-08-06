/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration file for boards, based on Qualcomm SDM845 chip
 *
 * (C) Copyright 2021 Dzmitry Sankouski <dsankouski@gmail.com>
 */

#ifndef __CONFIGS_SDM845_H
#define __CONFIGS_SDM845_H

#include <linux/sizes.h>
#include <asm/arch/sysmap-sdm845.h>

#define CFG_SYS_BAUDRATE_TABLE	{ 115200, 230400, 460800, 921600, 3000000 }

// // There's a bunch of reserved memory between 0x80000000 and 0x90000000
// // So to be safe, load everything except the extlinux conf to somewhere
// // in the 0x90000000 range
// #define SDRAM_OFFSET(x)		0x9##x
// #define SDRAM_BASE		0x80000000
// #define PXEFILE_ADDR_R		__stringify(SDRAM_BASE)
// #define BOOTM_LOW		__stringify(SDRAM_OFFSET(0000000))
// // FDT first
// #define FDT_ADDR_R		__stringify(SDRAM_OFFSET(0000000))
// // Decompress kernel at 512k offset
// #define KERNEL_COMP_ADDR_R	__stringify(SDRAM_OFFSET(0080000))
// // Kernel at 128M offset
// #define KERNEL_ADDR_R		__stringify(SDRAM_OFFSET(8080000))
// // Safe 128M to decompress kernel
// #define KERNEL_COMP_SIZE	__stringify(SZ_128M)
// // Ramdisk at 256M offset (framebuffer is at 0x9D400000 which should be after)
// // the kernel
// #define RAMDISK_ADDR_R		__stringify(0xA0000000)


// There's a bunch of reserved memory between 0x80000000 and 0x90000000
// The addresses here are taken from ABL behaviour.
#define SDRAM_OFFSET(x)		0x8##x
#define SDRAM_BASE		0x80000000
#define PXEFILE_ADDR_R		__stringify(SDRAM_BASE)
#define BOOTM_LOW		__stringify(SDRAM_OFFSET(0000000))
// FDT goes somewhere in the middle of the reserved memory
#define FDT_ADDR_R		__stringify(SDRAM_OFFSET(52F1000))
// Decompress kernel at 4M offset (differ from ABL to follow Linux rules)
// and be 4M aligned at last!
#define KERNEL_COMP_ADDR_R	__stringify(SDRAM_OFFSET(0200000))
// Safe 124M to decompress kernel
#define KERNEL_COMP_SIZE	__stringify(SZ_128M)
// Load kernel at 386M offset (well out the way)
#define KERNEL_ADDR_R		__stringify(0x98200000)
// Load ramdisk in between some reserved memory regions
#define RAMDISK_ADDR_R		__stringify(0x91000000)

#define CFG_EXTRA_ENV_SETTINGS \
	"bootm_size=0x4000000\0"	\
	"bootm_low=0x80000000\0"	\
	"pxefile_addr_r=" PXEFILE_ADDR_R "\0" \
	"kernel_addr_r=" KERNEL_ADDR_R "\0" \
	"kernel_comp_addr_r=" KERNEL_COMP_ADDR_R "\0" \
	"kernel_comp_size=" KERNEL_COMP_SIZE "\0" \
	"fdt_addr_r=" FDT_ADDR_R "\0" \
	"ramdisk_addr_r=" RAMDISK_ADDR_R "\0" \
	"bootdelay=1\0" \
	"stdin=serial\0"	\
	"stdout=serial,vidconsole\0"	\
	"stderr=serial,vidconsole\0"	\
	"preboot=scsi scan;\0" \
	"bootcmd=load scsi ${bootdev}:${bootpart} ${kernel_comp_addr_r} /EFI/Boot/bootaa64.efi; " \
		"load scsi ${bootdev}:${bootpart} ${fdt_addr_r} /${devicetree}; " \
		"bootefi $kernel_comp_addr_r $fdt_addr_r\0" \

#endif
