/*
 * Header file for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) touchscreen drivers.
 * For use with Cypress Txx3xx parts.
 * Supported parts include:
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009-2011 Cypress Semiconductor, Inc.
 * Copyright (C) 2010-2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#ifndef __CYTTSP_CORE_H__
#define __CYTTSP_CORE_H__

#include <linux/kernel.h>
#include <linux/err.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#if 0  /* FUJITSU:2011-09-27 del start */
#define CY_I2C_NAME                 "cyttsp3-i2c"
#define CY_SPI_NAME                 "cyttsp3-spi"
#define CY_DRIVER_VERSION           "Rev3-2M-17"
#define CY_DRIVER_DATE              "2011-08-12"
#endif /* FUJITSU:2011-09-27 del end */

/* FUJITSU:2011-10-02 mod start */
//#define CY_NUM_RETRY                26 /* max retries for rd/wr ops */
#define CY_NUM_RETRY                5 /* max retries for rd/wr ops */
/* FUJITSU:2011-10-02 mod end */

/* use the following define to enable register peak/poke capability
#define CY_USE_REG_ACCESS
 */

/* use the following define to enable special debug tools for test only
#define CY_USE_DEBUG_TOOLS
 */

/* use the following define for using Multi-Touch custom handling
#define CY_USE_XTD_MT
 */

/* FUJITSU:2011-09-27 add start */
#define CY_USE_REG_ACCESS
#define CY_USE_DEBUG_TOOLS
#define CONFIG_TOUCHSCREEN_DEBUG
#define CONFIG_TOUCHSCREEN_WATCHDOG
/* FUJITSU:2011-09-27 add end */
#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* use the following defines for dynamic debug printing */
/*
 * Level 0: Default Level
 * All debug (cyttsp_dbg) prints turned off
 */
#define CY_DBG_LVL_0			0
/*
 * Level 1:  Used to verify driver and IC are working
 *    Input from IC, output to event queue
 */
#define CY_DBG_LVL_1			1
/*
 * Level 2:  Used to further verify/debug the IC
 *    Output to IC
 */
#define CY_DBG_LVL_2			2
/*
 * Level 3:  Used to further verify/debug the driver
 *    Driver internals
 */
#define CY_DBG_LVL_3			3
#define CY_DBG_LVL_MAX			CY_DBG_LVL_3

#ifdef CY_USE_DEBUG_TOOLS
#define CY_DBG_SUSPEND                  4
#define CY_DBG_RESUME                   5
#define CY_DBG_FLIP                     16
#define CY_DBG_INV_X                    17
#define CY_DBG_INV_Y                    18
#define CY_DBG_NOINV                    19
#define CY_DBG_RESET                    99
#endif

#define cyttsp_dbg(ts, l, f, a...) {\
	if (ts->bus_ops->tsdebug >= (l)) \
		pr_info(f, ## a);\
}
#else
#define cyttsp_dbg(ts, l, f, a...)
#endif

struct cyttsp_bus_ops {
	s32 (*write)(void *handle, u8 addr, u8 length, const void *values);
	s32 (*read)(void *handle, u8 addr, u8 length, void *values);
	struct device *dev;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	u8 tsdebug;
#endif
};

void *cyttsp_core_init(struct cyttsp_bus_ops *bus_ops,
	struct device *dev, int irq, char *name);

void cyttsp_core_release(void *handle);
#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
int cyttsp_resume(void *handle);
int cyttsp_suspend(void *handle);
#endif

#endif /* __CYTTSP_CORE_H__ */
