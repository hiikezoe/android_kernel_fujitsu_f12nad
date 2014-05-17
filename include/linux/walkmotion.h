/*
 * Copyright(C) 2011 FUJITSU LIMITED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/


#ifndef _FJ_WALKMOTION_H
#define _FJ_WALKMOTION_H


#include <linux/ioctl.h>

#define FJ_WM_IOC_MAGIC '~'

/* Initialize */
#define FJ_WM_IOCT_INITIALIZE 		_IO(FJ_WM_IOC_MAGIC, 0)
/* Cancel initialize */
#define FJ_WM_IOCT_CANCELINITIALIZE	_IO(FJ_WM_IOC_MAGIC, 1)
/* Request IRQ */
#define FJ_WM_IOCS_REQUESTMOTIONIRQ	_IOW(FJ_WM_IOC_MAGIC, 2, unsigned int)
/* Cancel request IRQ */
#define FJ_WM_IOCT_CANCELMOTIONIRQ	_IO(FJ_WM_IOC_MAGIC, 3)
/* Set interrupt terminal */
#define FJ_WM_IOCS_SETSCIFACONTROL	_IOW(FJ_WM_IOC_MAGIC, 4, unsigned int)

/* FUJITSU:2011-07-26 Add new ioctl command start */
/* WakeUp */
#define FJ_WM_IOCS_WAKEUPCONTROL	_IOW(FJ_WM_IOC_MAGIC, 5, unsigned int)
/* FUJITSU:2011-07-26 Add new ioctl command end */

/* Detection of high edge */
#define FJ_WM_EDGE_HIGH			1
/* Detection of low edge */
#define FJ_WM_EDGE_LOW			0
/* UART port */
#define FJ_WM_MODE_GPIO			1
/* GPIO port */
#define FJ_WM_MODE_UART			0

/* FUJITSU:2011-07-26 Add new ioctl command start */
/* Wakeup High */
#define FJ_WM_WAKEUP_HIGH		1
/* Wakeup Low */
#define FJ_WM_WAKEUP_LOW		0
/* FUJITSU:2011-07-26 Add new ioctl command end */

/* FUJITSU:2012-01-16 bsp-sensor add start */
#if defined(CONFIG_MACH_F11APO) || defined(CONFIG_MACH_F12APON) || defined(CONFIG_MACH_FJI12)
#define FJ_WM_GPIO_HOSU_IRQ		(42)   /* IRQ */
#define FJ_WM_GPIO_HOSU_WUP		(64)   /* Wakeup */
#define FJ_WM_GPIO_HOSU_REL		(65)   /* RELEASE(not use) */
#define FJ_WM_GPIO_RESET		(120)   /* Reset */
#define FJ_WM_GPIO_HOSU_I2C2	(180)   /* I2CIRQ(not use) */
/* FUJITSU:2012-03-27 NAD change start */
//#elif defined(CONFIG_MACH_F11SKY) || defined(CONFIG_MACH_F09D) || defined(CONFIG_MACH_F12NAD)
#elif defined(CONFIG_MACH_F11SKY) || defined(CONFIG_MACH_F09D)
#define FJ_WM_GPIO_HOSU_IRQ		(42)   /* IRQ */
#define FJ_WM_GPIO_HOSU_WUP		(64)   /* Wakeup */
#define FJ_WM_GPIO_HOSU_REL		(65)   /* RELEASE(not use) */
#define FJ_WM_GPIO_RESET		(93)   /* Reset */
#define FJ_WM_GPIO_HOSU_I2C2	(180)   /* I2CIRQ(not use) */
#elif defined(CONFIG_MACH_F12NAD)
#define FJ_WM_GPIO_HOSU_IRQ		(42)   /* IRQ */
#define FJ_WM_GPIO_HOSU_WUP		(161)   /* Wakeup */
#define FJ_WM_GPIO_RESET		(93)   /* Reset */
/* FUJITSU:2012-03-27 NAD change end */
#endif
/* FUJITSU:2012-01-16 bsp-sensor add end */

/* Walk Motion MC Platform Data */
struct fj_wm_platform_data {
	/* Motion IRQ */
	int motion_irq;
	/* Delay */
	int mc_init_delay;	
};

#endif /** _FJ_WALKMOTION_H */
