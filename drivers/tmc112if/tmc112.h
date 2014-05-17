/*----------------------------------------------------------------------------*/
/* Copyright (c) 2010,2011 NTT DOCOMO, INC. All Rights Reserved.			  */
/* All Rights Reserved, Copyright(c) FUJITSU LIMITED 2011.					  */
/*----------------------------------------------------------------------------
 * File Name:
 *				tmc112.h
 *
 * Description:
 *				tmc112
 *
 * Notes:
 *----------------------------------------------------------------------------*/
/* $NoKeywords: $ */
/*!
 * @file  tmc112.h
 * @brief tmc112<BR>
 */

#ifndef _TMC112_H_
#define _TMC112_H_

#include <linux/clk.h>
#include <mach/vreg.h>
#include <linux/spinlock.h>

int tmc112_boot(void);
int tmc112_halt(void);

/// 
struct tmc112_pdata {
	int pin_reset;				///< RESETGPIO pin
	int pin_request;			///< REQUESTGPIO pin
	int gpio_request_irq;		///< REQUEST IRQ
	int pin_ready;				///< READYGPIO pin
	int pin_xirq;				///< XIRQGPIO pin
	struct vreg *vreg_core;		///< CORE
	struct vreg *vreg_io;		///< IO
	struct clk* clk_b;			///< GP_CLK_B

	int request_raised;			///< REQUEST
	int ready_raised;			///< READY
	wait_queue_head_t	wq_request;	///< REQUEST

	
	int suspend;				///
	int suspend_withrun;		///
	int resume;					///

	int cancel_read;			///
	int running;				///
};

#endif
