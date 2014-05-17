/*
 * COPYRIGHT(C) 2011-2012 FUJITSU LIMITED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
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
#ifndef _FJTOUCH_IF_H_
#define _FJTOUCH_IF_H_

#include <linux/ioctl.h>

/* DeviceStatus */
enum devicepowerstatus
{
	DEEPSLEEP        = 0x00,   /* DeepSleep          */
	WAKEUP,                    /* WakeUp             */
	HARDRESET,                 /* HardReset          */
	SOFTRESET,                 /* SoftReset          */
	TOUCH_EVENT_ENABLE,        /* TouchEvent Enable  */
	TOUCH_EVENT_DISABLE        /* TouchEvent Disable */
};

struct fj_touch_i2c_data
{
	int  slaveaddress;
	int  offset;
	int  use_offset;
	int  length;
	char *i2c_data_buf;
};

struct fj_touch_grip_suppression_data
{
	int  use_grip;
	int  grip_mode;
	int  x_lo_grip;
	int  x_hi_grip;
	int  y_lo_grip;
	int  y_hi_grip;
};

struct fj_touch_do_selftest_data
{
	int  test_cmd;
	unsigned int low_signal_limit;
	unsigned int high_signal_limit;
	char *result_data_buf;
};

struct fj_touch_debug_diagnostic_data
{
	int  test_cmd;
	char *raw_data_buf;
};

#define FJTOUCH_IOCTL_MAGIC 'c'

#define IOCTL_SET_POWERMODE         _IOR(FJTOUCH_IOCTL_MAGIC, 1, int)
#define IOCTL_GET_FIRMVERSION       _IOW(FJTOUCH_IOCTL_MAGIC, 2,  char * )
#define IOCTL_DO_CALIBRATION        _IO(FJTOUCH_IOCTL_MAGIC, 3)
#define IOCTL_I2C_READ              _IOR(FJTOUCH_IOCTL_MAGIC, 4, struct fj_touch_i2c_data)
#define IOCTL_I2C_WRITE             _IOW(FJTOUCH_IOCTL_MAGIC, 5, struct fj_touch_i2c_data)
#define IOCTL_SET_GRIPSUPPRESSION   _IOW(FJTOUCH_IOCTL_MAGIC, 6, struct fj_touch_grip_suppression_data)
#define IOCTL_DO_SELFTEST           _IOR(FJTOUCH_IOCTL_MAGIC, 7, struct fj_touch_do_selftest_data)
#define IOCTL_DEBUG_DIAGNOSTIC      _IOR(FJTOUCH_IOCTL_MAGIC, 8, struct fj_touch_debug_diagnostic_data)

#endif // _FJTOUCH_IF_H_
