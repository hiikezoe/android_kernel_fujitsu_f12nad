/*
 * COPYRIGHT(C) 2012 FUJITSU LIMITED
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
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2012
/*----------------------------------------------------------------------------*/
#ifndef _CYTTSP_PRESS_IF_H_
#define _CYTTSP_PRESS_IF_H_

#include <linux/ioctl.h>

/* TouchStatus */
enum touchstatus {
    TOUCH_DOWN          = 0x00, /* TOUCH_DOWN   */
    LIFT_OFF            = 0x01  /* LIFT_OFF     */
};

/* DeviceStatus */
enum devicepowerstatus {
    DEEP_SLEEP          = 0x00, /* DeepSleep            */
    WAKE_UP,                    /* WakeUp               */
    RESET_ON,                   /* ResetSetting         */
    RESET_OFF                   /* ResetCancellation    */
};

/* PressValue */
struct press_value {
    unsigned short filteredraw; /* filteredraw          */
    unsigned short raw;         /* raw                  */
    unsigned short base;        /* base                 */
};


#define PRESS_IOCTL_MAGIC 'p'

#define PRESS_SET_TOUCHSTATUS           _IOR(PRESS_IOCTL_MAGIC, 0, int)
#define PRESS_SET_POWERMODE             _IOR(PRESS_IOCTL_MAGIC, 1, int)
#define PRESS_FIRMWARE_DOWNLOAD         _IO(PRESS_IOCTL_MAGIC, 2)
#define PRESS_GET_DEVICE_INFORMATION    _IOW(PRESS_IOCTL_MAGIC, 3, char *)
#define PRESS_DO_CALIBRATION            _IOW(PRESS_IOCTL_MAGIC, 4, unsigned char *)    /* 10bytes */
#define PRESS_GET_PRESS_VALUE           _IOW(PRESS_IOCTL_MAGIC, 5, struct press_value *)
#define PRESS_GET_FW_VERSION            _IOW(PRESS_IOCTL_MAGIC, 6, unsigned short *)

/* PRESS_DO_CALIBRATION */
/*  Contents of arg. 10bytes
     sensor info.
        unsigned short pre_filterdraw;
        unsigned short calibrated_filterdraw;
        unsigned char  calibrated_idac;
     reference capacitor info.
        unsigned short pre_filterdraw;
        unsigned short calibrated_filterdraw;
        unsigned char  calibrated_idac;
*/

#endif // _CYTTSP_PRESS_IF_H_
