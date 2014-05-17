/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/


/* FUJITSU:2012-01-16 __ATMEL_MXT_TS_APO_H start */
#ifndef __ATMEL_MXT_TS_APO_H
#define __ATMEL_MXT_TS_APO_H

#include <linux/types.h>

/* Orient */
#define MXT_NORMAL		0x0
#define MXT_DIAGONAL		0x1
#define MXT_HORIZONTAL_FLIP	0x2
#define MXT_ROTATED_90_COUNTER	0x3
#define MXT_VERTICAL_FLIP	0x4
#define MXT_ROTATED_90		0x5
#define MXT_ROTATED_180		0x6
#define MXT_DIAGONAL_COUNTER	0x7

/* FUJITSU:2012-03-07 add start */
#define MXT_ENABLE_ORIGINAL_FUNCTION  0
#define MXT_ENABLE_DEBUG_FUNCTION     1
#define MXT_DEVICE_ID                 "maxtouch"
/* FUJITSU:2012-03-07 add end */

/* FUJITSU:2012-03-07 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	const u8 *config;
	size_t config_length;

	unsigned int x_size;
	unsigned int y_size;
	unsigned long irqflags;
	bool	i2c_pull_up;

	u8(*read_chg) (void);
	int (*init_hw) (bool);
	int (*power_on) (bool);
};
#else
struct mxt_config_data {
	u8 type;
	u8 *config;
	size_t length;
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	unsigned int *config_crc;
	struct mxt_config_data* config_t6;
	struct mxt_config_data* config_t7;
	struct mxt_config_data* config_t8;
	struct mxt_config_data* config_t9;
	struct mxt_config_data* config_t15;
	struct mxt_config_data* config_t18;
	struct mxt_config_data* config_t19;
	struct mxt_config_data* config_t23;
	struct mxt_config_data* config_t25;
	struct mxt_config_data* config_t40;
	struct mxt_config_data* config_t42;
	struct mxt_config_data* config_t46;
	struct mxt_config_data* config_t47;
	struct mxt_config_data* config_t48;
	struct mxt_config_data* config_t38;

	unsigned long irqflags;
	bool	i2c_pull_up;

	u8(*read_chg) (void);
	int (*init_hw) (bool);
	int (*power_on) (bool);
	int (*get_batt_status) (void);
	int (*get_config_rev) (void);
	void (*gpio_check) (void);
};
#endif
/* FUJITSU:2012-03-07 MXT_ENABLE_ORIGINAL_FUNCTION end */

#endif /* __ATMEL_MXT_TS_APO_H */
/* FUJITSU:2012-01-16 __ATMEL_MXT_TS_APO_H end */
