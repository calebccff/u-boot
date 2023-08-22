// SPDX-License-Identifier: GPL-2.0+
/*
 * Common init part for boards based on SDM845
 *
 * (C) Copyright 2021 Dzmitry Sankouski <dsankouski@gmail.com>
 */

#include <init.h>
#include <env.h>
#include <common.h>
#include <asm/system.h>
#include <asm/gpio.h>
#include <dm.h>

DECLARE_GLOBAL_DATA_PTR;

__weak int misc_init_r(void)
{
	return 0;
}
