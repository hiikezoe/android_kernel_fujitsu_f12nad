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
#ifndef compassdriver_h
#define compassdriver_h
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/compass.h>
/*--------------------------------------------------------------------
Interrupt
--------------------------------------------------------------------*/
#if defined(CONFIG_MACH_F11SKY) || defined(CONFIG_MACH_F09D) || defined(CONFIG_MACH_F12NAD)
#define JIKI_DRDY_GPIO 97
#elif defined(CONFIG_MACH_F11APO) || defined(CONFIG_MACH_F12APON) || defined(CONFIG_MACH_FJI12)
#define JIKI_DRDY_GPIO 144
#endif


#define AKM_IRQ()  gpio_to_irq(JIKI_DRDY_GPIO)
/*--------------------------------------------------------------------
Register address
--------------------------------------------------------------------*/
#define REG_WIA      0x00 // R:  Device ID
#define REG_INFO     0x01 // R:  Infomation
#define REG_ST1      0x02 // R:  Status1
#define REG_XLSB     0x03 // R:  X Axis Output data register(LSB)
#define REG_XMSB     0x04 // R:  X Axis Output data register(MSB)
#define REG_YLSB     0x05 // R:  Y Axis Output data register(LSB)
#define REG_YMSB     0x06 // R:  Y Axis Output data register(MSB)
#define REG_ZLSB     0x07 // R:  Z Axis Output data register(LSB)
#define REG_ZMSB     0x08 // R:  Z Axis Output data register(MSB)
#define REG_ST2      0x09 // R:  Status2
#define REG_CNTL     0x0A // RW: Mode Control
#define REG_RSV      0x0B // RW: Reservation
#define REG_ASTC     0x0C // RW: Self Test
#define REG_TS1      0x0D // RW: Test1
#define REG_TS2      0x0E // RW: Test2
#define REG_I2CDIST  0x0F // RW: I2C Invalidity
#define REG_ASAX     0x10 // R:  X Axis Adjustment
#define REG_ASAY     0x11 // R:  Y Axis Adjustment
#define REG_ASAZ     0x12 // R:  Z Axis Adjustment

// The maximum number of consecutive register numbers (XLSB-YMSB=6)
// (I2C It uses it for the size decision in the sending and receiving buffer.)
#define MAX_CONTIGUOUS_REGS (REG_ZMSB - REG_XLSB + 1)
#define OUTPUTDATA_AND_CHECKCODE_REGS     8
/*--------------------------------------------------------------------
Register Content
--------------------------------------------------------------------*/
#define MODE_POWER_DOWN                 0x00 //Porew Down Mode
#define MODE_SINGLE_MEASUREMENT         0x01 //Single Measurement Mode
#define MODE_SELF_TEST                  0x08 //Self Test Mode
#define MODE_FUSE_ROM_ACCESS            0x0F //Fuse Rom Accese Mode

#define RESET_CODE                      0x00 //Reset Code
#define SELFTEST_SET                    0x40 //Selftest Code
/*--------------------------------------------------------------------
Error code
--------------------------------------------------------------------*/
#define DERR       1 //Measurement Data Error
#define HOFL       2 //Measurement Data Over Flow

/*--------------------------------------------------------------------
AKM Peculiar instance data
--------------------------------------------------------------------*/
typedef struct akm {
	wait_queue_head_t measure_end_queue; // Present measurement completion queue
	compass_data_t measurement; // The last result of a measurement

	int use_count;              // Frequency that has been opened now
	unsigned max_retry_count;   // Maximum retrying frequency

	unsigned error_count;       // This time ioctl() error frequency
	unsigned retry_count;       // This time ioctl() retrying frequency
	u_long total_error_count;   // Total when system starts
	u_long total_retry_count;   // Total when system starts

	// State of operation
	volatile uint8_t state;		// Status
#define STAT_IDLE		0	// State of idol
#define STAT_MEASURING	1	// AKM measuring
#define STAT_MEASURED	2	// Measurement completion
#define STAT_AVAILABLE	3	// The measuring data can be read

	// AKM register
	cmps_registers_t reg;
} akm_t;

// Mode (MS1.MODE[1:0]) get.
#define MODE()           (akm->reg.cntl1 & AKM_MODE_MASK)

// Mode (MS1.MODE[1:0]) set.
#define SETMODE(mode)    (akm->reg.cntl1 = (mode))

// State of operation
#define STATE()              (akm->state)
#define SETSTATE(newState)   (STATE() = (newState))
#define MAX_RETRY_COUNT      2
/*--------------------------------------------------------------------
AKM Timing data
--------------------------------------------------------------------*/
//mode change time(us)
#define MODE_Wait  100

// Measurement time maximum value (ms)
#define MAX_MEASUREMENT_TIME 20

#define MAX_MEASUREMENT_COUNT 2
/*--------------------------------------------------------------------
--------------------------------------------------------------------*/
#ifdef __KERNEL__
extern akm_t akm[1];

#if 1   /* FUJITSU:2011-12-01 ICS mod start */
long akm_ioctl( struct file*, unsigned, u_long );
#else
int akm_ioctl(struct inode*, struct file*, unsigned, u_long);
#endif  /* FUJITSU:2011-12-01 ICS mod end */
int akm_open(struct inode*, struct file*);
int akm_release(struct inode*, struct file*);
void akm_module_init(void);
void akm_module_term(void);

#endif /* __KERNEL__ */
#endif /* compassdriver_h */
