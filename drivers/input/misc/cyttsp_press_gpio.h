/*
 * Copyright(C) 2012 FUJITSU LIMITED
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
// COPYRIGHT(C) FUJITSU LIMITED 2012
/*----------------------------------------------------------------------------*/

#ifndef _CYTTSP_PRESS_GPIO_H_
#define _CYTTSP_PRESS_GPIO_H_

/**
 * This struct defines a i2c command for I2C slave I/O operation.
 */
#define RET_OK      0
#define RET_ERR     -1

#define boolean int

#define SCL                     178           // I2C SCL GPIO No
#define SDA                     177           // I2C SDA GPIO No

/* When use Cypress TouchPress IC chipset, Use the following define to dummy receive
#define CY_USE_TOUCH_PRESS_IC_DUMMY_RECV
 */
#define CY_USE_TOUCH_PRESS_IC_DUMMY_RECV

struct PressI2CCmdType {
    unsigned char  slave_addr;          // Slave address
    unsigned char  *pwdata;             // Pointer to Write data buffer
    unsigned short wlen;                // Count of bytes to transfer
    unsigned char  *prdata;             // Pointer to Write data buffer
    unsigned short rlen;                // Count of bytes to transfer
    u8 tsdebug;                         // Debug Level
} __attribute__((packed, aligned(4)));


struct i2c_client;

extern int press_gpioi2c_read(struct PressI2CCmdType *);
extern int press_gpioi2c_write(struct PressI2CCmdType *);

extern int press_i2c_read(struct i2c_client*, struct PressI2CCmdType *);
extern int press_i2c_write(struct i2c_client*, struct PressI2CCmdType *);

#endif // _CYTTSP_PRESS_GPIO_H_
