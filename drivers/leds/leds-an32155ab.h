/*
 * Copyright(C) 2011-2012 FUJITSU LIMITED
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

#ifndef _LINUX_AN32155AB_H
#define _LINUX_AN32155AB_H

/*--------------------------------------------------------------------
I2C register (AN32155AB)
--------------------------------------------------------------------*/
#define I2C_LED_SRESET			0x00
#define I2C_LED_POWERCNT		0x01
#define I2C_LED_LEDMODE			0x02
#define I2C_LED_VDETLEDCN		0x05
#define I2C_LED_MTXON			0x06

#define I2C_LED_LEDON1			0x10
#define I2C_LED_LEDON2			0x11
#define I2C_LED_FADE1			0x12
#define I2C_LED_FADE2			0x13
#define I2C_LED_FF1				0x14
#define I2C_LED_FF2				0x15
#define I2C_LED_COUNT1			0x18
#define I2C_LED_COUNT2			0x19
#define I2C_LED_COUNT3			0x1A

#define I2C_LED_SEQA2			0x21
#define I2C_LED_SEQB1			0x23
#define I2C_LED_SEQB2			0x24
#define I2C_LED_SEQB3			0x25
#define I2C_LED_SEQC2			0x27
#define I2C_LED_SEQD1			0x29
#define I2C_LED_SEQD2			0x2A
#define I2C_LED_SEQD3			0x2B

#define I2C_LED_A2SET1			0x33
#define I2C_LED_A2SET2			0x34
#define I2C_LED_A2SET3			0x35
#define I2C_LED_B1SET1			0x39
#define I2C_LED_B1SET2			0x3A
#define I2C_LED_B1SET3			0x3B
#define I2C_LED_B2SET1			0x3C
#define I2C_LED_B2SET2			0x3D
#define I2C_LED_B2SET3			0x3E
#define I2C_LED_B3SET1			0x3F

#define I2C_LED_B3SET2			0x40
#define I2C_LED_B3SET3			0x41
#define I2C_LED_C2SET1			0x45
#define I2C_LED_C2SET2			0x46
#define I2C_LED_C2SET3			0x47
#define I2C_LED_D1SET1			0x4B
#define I2C_LED_D1SET2			0x4C
#define I2C_LED_D1SET3			0x4D
#define I2C_LED_D2SET1			0x4E
#define I2C_LED_D2SET2			0x4F

#define I2C_LED_D2SET3			0x50
#define I2C_LED_D3SET1			0x51
#define I2C_LED_D3SET2			0x52
#define I2C_LED_D3SET3			0x53

/* FUJITSU:2011-10-11 LED start */
#define I2C_LED_COUNT1			0x18
#define I2C_LED_COUNT2			0x19
#define I2C_LED_COUNT3			0x1A
/* FUJITSU:2011-10-11 LED end */

/*--------------------------------------------------------------------
I2C status (AN32155AB)
--------------------------------------------------------------------*/
#define SW_RESET				0x01

#define SETLED_A1				0x01
#define SETLED_A2				0x02
#define SETLED_A3				0x04
#define SETLED_B1				0x10
#define SETLED_B2				0x20
#define SETLED_B3				0x40
#define SETLED_C1				0x01
#define SETLED_C2				0x02
#define SETLED_C3				0x04
#define SETLED_D1				0x10
#define SETLED_D2				0x20
#define SETLED_D3				0x40

/* FUJITSU:2011-10-11 LED start */
#define LED_FLAG_A2 0x80
#define LED_FLAG_B1 0x40
#define LED_FLAG_B2 0x20
#define LED_FLAG_B3 0x10
#define LED_FLAG_C2 0x08
#define LED_FLAG_D1 0x04
#define LED_FLAG_D2 0x02
#define LED_FLAG_D3 0x01
/* FUJITSU:2011-10-11 LED end */

#endif /* _LINUX_AN32155AB_H */
