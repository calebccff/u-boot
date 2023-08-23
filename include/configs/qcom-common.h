/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Common configuration for Qualcomm boards.
 *
 * (C) Copyright 2023 Linaro
 * Author: Caleb Connolly <caleb.connolly@linaro.org>
 */

#ifndef __CONFIGS_QCOM_H
#define __CONFIGS_QCOM_H

#include <linux/sizes.h>

#define CFG_SYS_BAUDRATE_TABLE	{ 115200, 230400, 460800, 921600 }

#ifndef SDRAM_OFFSET
#define SDRAM_OFFSET(x)		0x8##x
#endif
#ifndef SDRAM_BASE
#define SDRAM_BASE		0x80000000
#endif
#ifndef PXEFILE_ADDR_R
#define PXEFILE_ADDR_R		__stringify(SDRAM_BASE)
#endif
#ifndef BOOTM_LOW
#define BOOTM_LOW		__stringify(SDRAM_OFFSET(0000000))
#endif
#ifndef FDT_ADDR_R
// FDT goes somewhere in the middle of the reserved memory
#define FDT_ADDR_R		__stringify(SDRAM_OFFSET(52F1000))
#endif
#ifndef KERNEL_COMP_ADDR_R
// Decompress kernel at 4M offset (differ from ABL to follow Linux rules)
// and be 4M aligned at last!
#define KERNEL_COMP_ADDR_R	__stringify(SDRAM_OFFSET(0200000))
#endif
#ifndef KERNEL_COMP_SIZE
// Safe 124M to decompress kernel
#define KERNEL_COMP_SIZE	__stringify(SZ_128M)
#endif
#ifndef KERNEL_ADDR_R
// Load kernel at 386M offset (well out the way)
#define KERNEL_ADDR_R		__stringify(0x98200000)
#endif
#ifndef RAMDISK_ADDR_R
// Load ramdisk in between some reserved memory regions
#define RAMDISK_ADDR_R		__stringify(0x91000000)
#endif

#define BOOT_TARGET_DEVICES(func) \
	func(SCSI, scsi, 0) \

#include <config_distro_bootcmd.h>

#define ENV_MEM_LAYOUT_SETTINGS \
	"bootm_size=0x4000000\0"	\
	"bootm_low=" BOOTM_LOW "\0"	\
	"pxefile_addr_r=" PXEFILE_ADDR_R "\0" \
	"kernel_addr_r=" KERNEL_ADDR_R "\0" \
	"kernel_comp_addr_r=" KERNEL_COMP_ADDR_R "\0" \
	"kernel_comp_size=" KERNEL_COMP_SIZE "\0" \
	"fdt_addr_r=" FDT_ADDR_R "\0" \
	"ramdisk_addr_r=" RAMDISK_ADDR_R "\0"

#define CFG_EXTRA_ENV_SETTINGS \
	ENV_MEM_LAYOUT_SETTINGS \
	"bootdelay=3\0"		\
	"stdin=serial,button-kbd\0"	\
	"stdout=serial,vidconsole\0"	\
	"stderr=serial,vidconsole\0"	\
	BOOTENV

#endif
