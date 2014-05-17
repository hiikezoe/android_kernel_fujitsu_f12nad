/*
 * Copyright (C) 2010 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

/* Defines generic platform structures for touch drivers */
#ifndef _LINUX_TOUCH_PLATFORM_H
#define _LINUX_TOUCH_PLATFORM_H

#include <linux/types.h>

/* FUJITSU:2011-09-27 add start */
#define CY_I2C_NAME                 "cyttsp3-i2c"
#define CY_SPI_NAME                 "cyttsp3-spi"
#define CY_DRIVER_VERSION           "Rev3-2M-17"
#define CY_DRIVER_DATE              "2011-08-12"

#define CY_I2C_XRST_GPIO    143     /* KYTREE:RESET GPIO */
#define CY_I2C_IRQ_GPIO     18      /* KYTREE:INT GPIO */
#define CY_I2C_ADR          0x3B    /* KYTREE:I2C SLAVE ADDR*/

#define CY_MAXX 479
#define CY_MAXY 799

#define CY_ABS_MIN_X 0
#define CY_ABS_MIN_Y 0
#define CY_ABS_MIN_P 0
#define CY_ABS_MIN_W 0
#define CY_ABS_MIN_T 0
#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MAX_P 255
#define CY_ABS_MAX_W 255
//#define CY_ABS_MAX_T 255
#define CY_ABS_MAX_T 16
/* FUJITSU:2011-09-27 add end */

/* FUJITSU:2012-03-15 CYPRESS TOUCH PRESS add start */
#define CY_PR_I2C_NAME              "cyttsp_press_i2c"
#define CY_PR_SPI_NAME              "cyttsp_press_spi"
#define CY_PR_DRIVER_VERSION        "Rev3-2M-17"
#define CY_PR_DRIVER_DATE           "2012-05-31"

#define CY_PR_I2C_XRST_GPIO         170
#define CY_PR_I2C_IRQ_GPIO          169
#define CY_PR_I2C_ADR               0x24
/* FUJITSU:2012-03-15 CYPRESS TPUCH PRESS add end */

struct touch_settings {
	const uint8_t   *data;
	uint8_t         size;
	uint8_t         tag;
} __attribute__ ((packed));

struct touch_firmware {
	const uint8_t   *img;
	uint32_t        size;
	const uint8_t   *ver;
	uint8_t         vsize;
} __attribute__ ((packed));

struct touch_framework {
	const uint16_t  *abs;
	uint8_t         size;
	uint8_t         enable_vkeys;
} __attribute__ ((packed));

struct touch_platform_data {
	struct touch_settings   *sett[256];
	struct touch_firmware   *fw;
	struct touch_framework  *frmwrk;

	uint8_t         addr[2];
	uint16_t        flags;

	int         (*hw_reset)(void);
	int         (*hw_recov)(int);
	/* FUJITSU:2011-08-31 add Power start */
	int         (*hw_power)(int);
	/* FUJITSU:2011-08-31 add Power end */
	int         (*irq_stat)(void);
	/* FUJITSU:2011-08-11 add start */
	int         (*init)(int);
	/* FUJITSU:2011-08-11 add end */
} __attribute__ ((packed));

#endif /* _LINUX_TOUCH_PLATFORM_H */
