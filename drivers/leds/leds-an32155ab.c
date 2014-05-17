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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include "leds.h"
#include "leds-an32155ab.h"
/* FUJITSU:2011-08-18 AN32155AB start */
#include <linux/ctype.h>
#include "../arch/arm/mach-msm/smd_private.h"
/* FUJITSU:2011-08-18 AN32155AB end */


#define LED_DEBUG 0
#if LED_DEBUG
#define LED_DBG(x...)  printk(KERN_DEBUG x)
#else
#define LED_DBG(x...)  
#endif


#define LED_I2C_SLAVE   0x34
#define BUFFER_SIZE 64
#define READ_SIZE 2
#define I2C_TRANS_WRITE 1
#define I2C_TARNS_READ  2
#define MODE_FLOUORE 12

#define LED_RST_GPIO 130  /* MSM GPIO Number 130 */
#define PMIC_GPIO_LIGHT	25  /* PMIC GPIO Number 25 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)

/* FUJITSU:2011-10-25 AN32155AB start */
#define LED_CHARGE_BRIGHTNESS 0xFF
/* FUJITSU:2011-10-25 AN32155AB end */

/* FUJITSU:2011-09-05 AN32155AB start */
#define AN32155_GREEN_BRIGHTNESS_MAX       0x3F
#define AN32155_GREEN_BRIGHTNESS(green)    ((green * AN32155_GREEN_BRIGHTNESS_MAX)/0xFF)
/* FUJITSU:2011-09-05 AN32155AB end */

struct i2c_adapter *i2c_led;
/* FUJITSU:2012-2-23 add start */
static struct workqueue_struct *an32155ab_workqueue;
/* FUJITSU:2012-2-23 add end */

/*===========================================================================
  GLOBAL VARIABLES
  ===========================================================================*/
static int g_position = 0;
static int g_red = 0;
static int g_green = 0;
static int g_blue = 0;
static int g_bt_l = 0;
static int g_bt_s = 0;
static int g_bt_r = 0;
/* FUJITSU:2011-10-11 AN32155AB start */
static int led_active_flag = 0;
static int led_use_flag = 0;
/* FUJITSU:2011-10-11 AN32155AB end */

/* FUJITSU:2011-08-18 AN32155AB start */
/* FUJITSU:2012-04-06 PREVIEW LED start */
//static unsigned char led_notify;
static unsigned int led_notify;
/* FUJITSU:2012-04-06 PREVIEW LED start */
static unsigned char led_charge;

struct smem_led_color_type
	{
		uint8_t led_alpha;
		uint8_t led_red;
		uint8_t led_green;
		uint8_t led_blue;
		unsigned int  on_time;  /* FUJITSU:2011-08-25 AN32155AB add */
		unsigned int  off_time; /* FUJITSU:2011-08-25 AN32155AB add */
	}; 

/* FUJITSU:2011-08-25 AN32155AB start */
typedef enum {
	ILLUMI_PATTERN_NONE,	/* pattern0 */
	ILLUMI_PATTERN_1,		/* pattern1 */
	ILLUMI_PATTERN_2,		/* pattern2 */
	ILLUMI_PATTERN_3,		/* pattern3 */
	ILLUMI_PATTERN_4,		/* pattern4 */
	ILLUMI_PATTERN_5,		/* pattern5 */
	ILLUMI_PATTERN_6,		/* pattern6 */
	ILLUMI_PATTERN_7,		/* pattern7 */
	ILLUMI_PATTERN_8,		/* pattern8 */
	ILLUMI_PATTERN_9,		/* pattern9 */
	ILLUMI_PATTERN_10,		/* pattern10 */
	ILLUMI_PATTERN_CONT,	/* pattern CONT */
	ILLUMI_PATTERN_BLINK,	/* pattern BLINK */
	ILLUMI_PATTERN_SBLINK,	/* pattern SBLINK */
	ILLUMI_PATTERN_MAX
}e_illumi_pattern;

typedef enum {
	ILLUMI_COLOR_NONE,		/* color0 */
	ILLUMI_COLOR_1,			/* color1 */
	ILLUMI_COLOR_2,			/* color2 */
	ILLUMI_COLOR_3,			/* color3 */
	ILLUMI_COLOR_4,			/* color4 */
	ILLUMI_COLOR_5,			/* color5 */
	ILLUMI_COLOR_6,			/* color6 */
	ILLUMI_COLOR_7,			/* color7 */
	ILLUMI_COLOR_8,			/* color8 */
	ILLUMI_MULTI_COLOR_1,	/* multicolor1 */
	ILLUMI_MULTI_COLOR_2,	/* multicolor2 */
	ILLUMI_MULTI_COLOR_3,	/* multicolor3 */
	ILLUMI_MULTI_COLOR_4,	/* multicolor4 */
	ILLUMI_MULTI_COLOR_5,	/* multicolor5 */
	ILLUMI_MULTI_COLOR_6,	/* multicolor6 */
	ILLUMI_MULTI_COLOR_7,	/* multicolor7 */
	ILLUMI_MULTI_COLOR_8,	/* multicolor8 */
	ILLUMI_MULTI_COLOR_9,	/* multicolor9 */
	ILLUMI_MULTI_COLOR_10,	/* multicolor10 */
	ILLUMI_MULTI_COLOR_11,	/* multicolor11 */
	ILLUMI_MULTI_COLOR_12,	/* multicolor12 */
	ILLUMI_MULTI_COLOR_13,	/* multicolor13 */
	ILLUMI_MULTI_COLOR_14,	/* multicolor14 */
	ILLUMI_MULTI_COLOR_15,	/* multicolor15 */
	ILLUMI_COLOR_MAX
}e_illumi_color;

struct smem_led_illumi_info
	{
		e_illumi_pattern pattern_id;
		e_illumi_color color_id;
	};

struct smem_led_illumi_info smem_led_illumi_info;

/* FUJITSU:2011-08-25 AN32155AB end */

struct smem_led_color_type smem_led_color;

/* FUJITSU:2011-08-18 AN32155AB end */

/* FUJITSU:2012-1-18 mod start*/
#if 0
extern void timer_trig_activate(struct led_classdev *led_cdev);
#else
extern void timer_trig_probe(struct led_classdev *led_cdev);
#endif
/* FUJITSU:2012-1-18 mod end */
/*
Function : led_i2c_write
*/
static int
led_i2c_write(unsigned char reg, unsigned char data)
{
	struct i2c_msg msg;
	u_int8_t buf[BUFFER_SIZE];
	int ret = 0;
	msg.addr = LED_I2C_SLAVE;
	msg.flags = 0;
	buf[0] = reg;
	buf[1] = data;

	msg.buf  = buf;
	msg.len  = 2;

	ret = i2c_transfer(i2c_led, &msg, I2C_TRANS_WRITE);
/* FUJITSU:2011-10-11 AN32155AB start */
	if(0 > ret){
		pr_err ("[illumi log]%s:I2C write err addr=%x\n",__func__,reg);
	}
/* FUJITSU:2011-10-11 AN32155AB end */
	return ret;
}

/*
Function : led_i2c_read
*/
static int
led_i2c_read(unsigned short reg, unsigned char *data, uint32_t len)
{
	struct i2c_msg msg[READ_SIZE];
	u_int8_t msgbuf[READ_SIZE];
	int ret = 0;
	memcpy(msgbuf, &reg, sizeof(reg));

	msg[0].addr  = LED_I2C_SLAVE;
	msg[0].flags = 0;
	msg[0].buf   = msgbuf;
	msg[0].len   = 1;

	msg[1].addr  = LED_I2C_SLAVE;
	msg[1].flags = I2C_M_RD;
	msg[1].buf   = data;
	msg[1].len   = len;

	ret = i2c_transfer(i2c_led, msg, I2C_TARNS_READ);

	return ret;
}
/* FUJITSU:2011-10-11 AN32155AB start */
static void
led_active (int state)
{

	LED_DBG("[illumi log]%s:state=%d\n",__func__,state);
	if (state){
		gpio_set_value(LED_RST_GPIO, 1);
		led_active_flag = 1;
		mdelay(3);
		led_i2c_write (I2C_LED_SRESET, SW_RESET);
		led_i2c_write (I2C_LED_VDETLEDCN, 0x0f);
/* FUJITSU:2011-09-05 AN32155AB start */
		led_i2c_write (I2C_LED_MTXON, 0xa6);
/* FUJITSU:2011-09-05 AN32155AB end */
		led_i2c_write (I2C_LED_POWERCNT, 0x13);
	}
	else {
		led_active_flag = 0;
/* FUJITSU:2011-10-11 AN32155AB start */
		led_i2c_write (I2C_LED_MTXON, 0xa6);
/* FUJITSU:2011-10-11 AN32155AB start */
		gpio_set_value(LED_RST_GPIO, 0);
		led_use_flag = 0;
		g_position = 0;
		g_red = 0;
		g_green = 0;
		g_blue = 0;
		g_bt_l = 0;
		g_bt_s = 0;
		g_bt_r = 0;
	}
}

/* FUJITSU:2011-10-11 AN32155AB start */
static void
led_set_firefly (struct led_classdev *led_cdev , int type)
{
	int on, off;
	uint8_t rbuf_ff1 = 0;
	uint8_t rbuf_ff2 = 0;

	on = led_cdev->firefly_on_time;
	off = led_cdev->firefly_off_time;

	led_i2c_write (I2C_LED_COUNT1, 0x55);
	led_i2c_write (I2C_LED_COUNT2, 0x55);
	led_i2c_write (I2C_LED_COUNT3, 0x55);

/* FUJITSU:2011-08-10 AN32155AB start */
	if (type == 2) {
		if (g_bt_l <= 1) {
			led_i2c_read (I2C_LED_FF1, &rbuf_ff1, 1);
			if (g_bt_l == 1) {
				rbuf_ff1 = rbuf_ff1 | SETLED_A2;
				led_i2c_write (I2C_LED_A2SET2, on);
				led_i2c_write (I2C_LED_A2SET3, (0x80 | off));
			}
			else {
				rbuf_ff1 = rbuf_ff1 & 0xFD;
				led_i2c_write (I2C_LED_A2SET2, 0);
				led_i2c_write (I2C_LED_A2SET3, 0);
			}
			led_i2c_write (I2C_LED_FF1, rbuf_ff1);
		}

		if (g_bt_r <= 1) {
			led_i2c_read (I2C_LED_FF2, &rbuf_ff2, 1);
			if (g_bt_r == 1) {
				rbuf_ff2 = rbuf_ff2 | SETLED_C2;
				led_i2c_write (I2C_LED_C2SET2, on);
				led_i2c_write (I2C_LED_C2SET3, (0x80 | off));
			}
			else {
				rbuf_ff2 = rbuf_ff2 & 0xFD;
				led_i2c_write (I2C_LED_C2SET2, 0);
				led_i2c_write (I2C_LED_C2SET3, 0);
			}
			led_i2c_write (I2C_LED_FF2, rbuf_ff2);
		}
/* FUJITSU:2011-09-19 AN32155AB start */
		if (g_bt_s <= 1) {
			led_i2c_read (I2C_LED_FF1, &rbuf_ff1, 1);
			if (g_bt_s == 1) {
				rbuf_ff1 = rbuf_ff1 | SETLED_B1;
				led_i2c_write (I2C_LED_B1SET2, on);
				led_i2c_write (I2C_LED_B1SET3, (0x80 | off));

				rbuf_ff1 = rbuf_ff1 | SETLED_B2;
				led_i2c_write (I2C_LED_B2SET2, on);
				led_i2c_write (I2C_LED_B2SET3, (0x80 | off));

				rbuf_ff1 = rbuf_ff1 | SETLED_B3;
				led_i2c_write (I2C_LED_B3SET2, on);
				led_i2c_write (I2C_LED_B3SET3, (0x80 | off));
			}else{
				rbuf_ff1 = rbuf_ff1 | SETLED_B1;
				led_i2c_write (I2C_LED_B1SET2, 0);
				led_i2c_write (I2C_LED_B1SET3, 0);

				rbuf_ff1 = rbuf_ff1 | SETLED_B2;
				led_i2c_write (I2C_LED_B2SET2, 0);
				led_i2c_write (I2C_LED_B2SET3, 0);

				rbuf_ff1 = rbuf_ff1 | SETLED_B3;
				led_i2c_write (I2C_LED_B3SET2, 0);
				led_i2c_write (I2C_LED_B3SET3, 0);
			}
			led_i2c_write (I2C_LED_FF1, rbuf_ff1);
		}
/* FUJITSU:2011-09-19 AN32155AB end */
	}
/* FUJITSU:2011-08-10 AN32155AB end */

	if (g_position == 0 || g_position == 2 || g_bt_l == 1) {
		led_i2c_read (I2C_LED_FF1, &rbuf_ff1, 1);
		if (g_position == 0 || g_position == 2) {
			if (g_red) {
				rbuf_ff1 = rbuf_ff1 | SETLED_B1;
				led_i2c_write (I2C_LED_B1SET2, on);
				led_i2c_write (I2C_LED_B1SET3, (0x80 | off));
			}
			else {
				rbuf_ff1 = rbuf_ff1 & 0xEF;
				led_i2c_write (I2C_LED_B1SET2, 0);
				led_i2c_write (I2C_LED_B1SET3, 0);
			}

			if (g_green) {
				rbuf_ff1 = rbuf_ff1 | SETLED_B2;
				led_i2c_write (I2C_LED_B2SET2, on);
				led_i2c_write (I2C_LED_B2SET3, (0x80 | off));
			}
			else {
				rbuf_ff1 = rbuf_ff1 & 0xDF;
				led_i2c_write (I2C_LED_B2SET2, 0);
				led_i2c_write (I2C_LED_B2SET3, 0);
			}

			if (g_blue) {
				rbuf_ff1 = rbuf_ff1 | SETLED_B3;
				led_i2c_write (I2C_LED_B3SET2, on);
				led_i2c_write (I2C_LED_B3SET3, (0x80 | off));
			}
			else {
				rbuf_ff1 = rbuf_ff1 & 0xBF;
				led_i2c_write (I2C_LED_B3SET2, 0);
				led_i2c_write (I2C_LED_B3SET3, 0);
			}
		}

		if (g_bt_l == 1) {
			rbuf_ff1 = rbuf_ff1 | SETLED_A2;
			led_i2c_write (I2C_LED_A2SET2, on);
			led_i2c_write (I2C_LED_A2SET3, (0x80 | off));
		}
		else {
			rbuf_ff1 = rbuf_ff1 & 0xFD;
			led_i2c_write (I2C_LED_A2SET2, 0);
			led_i2c_write (I2C_LED_A2SET3, 0);
		}

		led_i2c_write (I2C_LED_FF1, rbuf_ff1);
	}
	if (g_position == 1 || g_position == 2 || g_bt_r == 1) {
		led_i2c_read (I2C_LED_FF2, &rbuf_ff2, 1);
		if (g_position == 1 || g_position == 2) {
			if (g_red) {
				rbuf_ff2 = rbuf_ff2 | SETLED_D1;
				led_i2c_write (I2C_LED_D1SET2, on);
				led_i2c_write (I2C_LED_D1SET3, (0x80 | off));
			}
			else {
				rbuf_ff2 = rbuf_ff2 & 0xEF;
				led_i2c_write (I2C_LED_D1SET2, 0);
				led_i2c_write (I2C_LED_D1SET3, 0);
			}

			if (g_green) {
				rbuf_ff2 = rbuf_ff2 | SETLED_D2;
				led_i2c_write (I2C_LED_D2SET2, on);
				led_i2c_write (I2C_LED_D2SET3, (0x80 | off));
			}
			else {
				rbuf_ff2 = rbuf_ff2 & 0xDF;
				led_i2c_write (I2C_LED_D2SET2, 0);
				led_i2c_write (I2C_LED_D2SET3, 0);
			}

			if (g_blue) {
				rbuf_ff2 = rbuf_ff2 | SETLED_D3;
				led_i2c_write (I2C_LED_D3SET2, on);
				led_i2c_write (I2C_LED_D3SET3, (0x80 | off));
			}
			else {
				rbuf_ff2 = rbuf_ff2 & 0xBF;
				led_i2c_write (I2C_LED_D3SET2, 0);
				led_i2c_write (I2C_LED_D3SET3, 0);
			}
		}

		if (g_bt_r == 1) {
			rbuf_ff2 = rbuf_ff2 | SETLED_C2;
			led_i2c_write (I2C_LED_C2SET2, on);
			led_i2c_write (I2C_LED_C2SET3, (0x80 | off));
		}
		else {
			rbuf_ff2 = rbuf_ff2 & 0xFD;
			led_i2c_write (I2C_LED_C2SET2, 0);
			led_i2c_write (I2C_LED_C2SET3, 0);
		}

		led_i2c_write (I2C_LED_FF2, rbuf_ff2);
	}
}
/* FUJITSU:2011-10-11 AN32155AB end */

static void
led_set_color (unsigned long state)
{

	uint8_t rbuf_ledon1 = 0;
	uint8_t rbuf_ledon2 = 0;

/* FUJITSU:2011-10-11 AN32155AB start */
	if (!led_active_flag) {
		if (state || (g_chrg_flag == 1))
			led_active(1);
		else
			return;
	}
/* FUJITSU:2011-10-11 AN32155AB end */

	g_position = (state >> 24) & 0xFF;
	g_red = (state >> 16) & 0xFF;
	g_green = (state >> 8) & 0xFF;
	g_blue = state & 0xFF;

	led_i2c_read (I2C_LED_LEDON1, &rbuf_ledon1, 1);
	led_i2c_read (I2C_LED_LEDON2, &rbuf_ledon2, 1);

	if (g_position == 0 || g_position == 2) {
		if (g_red) {
		led_use_flag = led_use_flag | LED_FLAG_B1;
			rbuf_ledon1 = rbuf_ledon1 | SETLED_B1;
			led_i2c_write (I2C_LED_B1SET1, g_red);
		}
		else {
/* FUJITSU:2011-08-25 AN32155AB start */
			if(((g_green == 0) && (g_blue == 0)) && (g_chrg_flag == 1) && (g_chrg_count != 2)){
/* FUJITSU:2012-03-27 PREVIEW LED start */
				printk("led_set_color state:0x%lx,notify:0x%x\n",state,led_notify);
				if((led_notify & 0x00000100) == 0x00000100){
					led_use_flag = led_use_flag & (0xFF ^ LED_FLAG_B1);
					rbuf_ledon1 = rbuf_ledon1 & 0xEF;
					led_i2c_write (I2C_LED_B1SET1, 0);
				}else{
					led_use_flag = led_use_flag | LED_FLAG_B1;
					rbuf_ledon1 = rbuf_ledon1 | SETLED_B1;
					led_i2c_write (I2C_LED_B1SET1, LED_CHARGE_BRIGHTNESS);
				}
/* FUJITSU:2012-03-27 PREVIEW LED end */
			}else{
				led_use_flag = led_use_flag & (0xFF ^ LED_FLAG_B1);
				rbuf_ledon1 = rbuf_ledon1 & 0xEF;
				led_i2c_write (I2C_LED_B1SET1, 0);
			}
/* FUJITSU:2011-08-25 AN32155AB end */
		}

		if (g_green) {
			led_use_flag = led_use_flag | LED_FLAG_B2;
			rbuf_ledon1 = rbuf_ledon1 | SETLED_B2;
/* FUJITSU:2011-09-05 AN32155AB start */
			led_i2c_write (I2C_LED_B2SET1, AN32155_GREEN_BRIGHTNESS(g_green));
/* FUJITSU:2011-09-05 AN32155AB start */
		}
		else {
			led_use_flag = led_use_flag & (0xFF ^ LED_FLAG_B2);
			rbuf_ledon1 = rbuf_ledon1 & 0xDF;
			led_i2c_write (I2C_LED_B2SET1, 0);
		}

		if (g_blue) {
			led_use_flag = led_use_flag | LED_FLAG_B3;
			rbuf_ledon1 = rbuf_ledon1 | SETLED_B3;
			led_i2c_write (I2C_LED_B3SET1, g_blue);
		}
		else {
			led_use_flag = led_use_flag & (0xFF ^ LED_FLAG_B3);
			rbuf_ledon1 = rbuf_ledon1 & 0xBF;
			led_i2c_write (I2C_LED_B3SET1, 0);
		}

		led_i2c_write (I2C_LED_LEDON1, rbuf_ledon1);
	}
	else {
		led_use_flag = led_use_flag & (0xFF ^ LED_FLAG_B1 ^ LED_FLAG_B2 ^ LED_FLAG_B3);
		rbuf_ledon1 = rbuf_ledon1 & 0x8F;
		led_i2c_write (I2C_LED_LEDON1, rbuf_ledon1);
		led_i2c_write (I2C_LED_B1SET1, 0);
		led_i2c_write (I2C_LED_B2SET1, 0);
		led_i2c_write (I2C_LED_B3SET1, 0);
	}
	if (g_position == 1 || g_position == 2) {
		if (g_red) {
			led_use_flag = led_use_flag | LED_FLAG_D1;
			rbuf_ledon2 = rbuf_ledon2 | SETLED_D1;
			led_i2c_write (I2C_LED_D1SET1, g_red);
		}
		else {
			led_use_flag = led_use_flag & (0xFF ^ LED_FLAG_D1);
			rbuf_ledon2 = rbuf_ledon2 & 0xEF;
			led_i2c_write (I2C_LED_D1SET1, 0);
		}

		if (g_green) {
			led_use_flag = led_use_flag | LED_FLAG_D2;
			rbuf_ledon2 = rbuf_ledon2 | SETLED_D2;
/* FUJITSU:2011-09-27 AN32155AB start */
			led_i2c_write (I2C_LED_D2SET1, g_green);
/* FUJITSU:2011-09-27 AN32155AB start */
		}
		else {
			led_use_flag = led_use_flag & (0xFF ^ LED_FLAG_D2);
			rbuf_ledon2 = rbuf_ledon2 & 0xDF;
			led_i2c_write (I2C_LED_D2SET1, 0);
		}

		if (g_blue) {
			led_use_flag = led_use_flag | LED_FLAG_D3;
			rbuf_ledon2 = rbuf_ledon2 | SETLED_D3;
			led_i2c_write (I2C_LED_D3SET1, g_blue);
		}
		else {
			led_use_flag = led_use_flag & (0xFF ^ LED_FLAG_D3);
			rbuf_ledon2 = rbuf_ledon2 & 0xBF;
			led_i2c_write (I2C_LED_D3SET1, 0);
		}

		led_i2c_write (I2C_LED_LEDON2, rbuf_ledon2);
	}
	else {
		led_use_flag = led_use_flag & (0xFF ^ LED_FLAG_D1 ^ LED_FLAG_D2 ^ LED_FLAG_D3);
		rbuf_ledon1 = rbuf_ledon1 & 0x8F;
		led_i2c_write (I2C_LED_LEDON2, rbuf_ledon2);
		led_i2c_write (I2C_LED_D1SET1, 0);
		led_i2c_write (I2C_LED_D2SET1, 0);
		led_i2c_write (I2C_LED_D3SET1, 0);
	}
	if ((led_active_flag) && (!led_use_flag)) {
		led_active(0);
	}
}

/* FUJITSU:2011-10-11 AN32155AB start */
static void
led_light_on (int state)
{
	uint8_t rbuf_led = 0;

	if (state){
		led_i2c_read (I2C_LED_MTXON, &rbuf_led, 1);
		rbuf_led = rbuf_led | 0x01;
		led_i2c_write (I2C_LED_MTXON, rbuf_led);
	}else{
		led_active(0);
	}
}
/* FUJITSU:2011-10-11 AN32155AB end */

/* FUJITSU:2012-2-23 add start */
static void
an32155ab_led_set(struct led_classdev * led_cdev, enum led_brightness value)
{
	LED_DBG("[illumi log]%s\n", led_cdev->name);
/* FUJITSU:2012-3-09 mod start */
	led_cdev->w_brightness = value;
/* FUJITSU:2012-3-09 mod start */
	queue_work(an32155ab_workqueue, &led_cdev->work);
}
/* FUJITSU:2012-2-23 add end */

/* FUJITSU:2011-10-11 AN32155AB start */
static void
color_led_set (struct led_classdev *led_cdev, enum led_brightness value)
{
	unsigned long state = value;

	LED_DBG("[illumi log]%s:state=%ld\n",__func__,state);
	led_set_color (state);
	if(led_active_flag){
		if(led_cdev->firefly_flag == 1)
			led_set_firefly (led_cdev,1);
		led_light_on(1);
	}
}
/* FUJITSU:2011-10-11 AN32155AB end */

static void
button_led_set (struct led_classdev *led_cdev, enum led_brightness value)
{

	uint8_t rbuf_ledon1 = 0;
	uint8_t rbuf_ledon2 = 0;
	unsigned long state = value;

	if (!led_active_flag) {
		if (state & 0x00FFFFFF)
			led_active(1);
		else
			return;
	}

/* FUJITSU:2011-09-08 AN32155AB start */
	g_bt_l = 2;
	g_bt_s = 2;
	g_bt_r = 2;

	if((state & 0x01000000)>0){
		if((state & 0x000000FF)>0){
			g_bt_r = 1;
		}else{
			g_bt_r = 0;
		}
	}
	if((state & 0x02000000)>0){
		if((state & 0x0000FF00)>0){
			g_bt_s = 1;
		 }else{
			g_bt_s = 0;
		}
	}
	if((state & 0x04000000)>0){
		if((state & 0x00FF0000)>0){
			g_bt_l = 1;
		}else{
			g_bt_l = 0;
		}
	}
/* FUJITSU:2011-09-08 AN32155AB end */

	LED_DBG("[illumi log]%s:g_bt_l=%d,g_bt_s=%d,g_bt_r=%d\n",__func__,g_bt_l,g_bt_s,g_bt_r);

	switch (g_bt_l) {
	case 0:
		led_use_flag = led_use_flag & (0xFF ^ LED_FLAG_A2);
		led_i2c_read (I2C_LED_LEDON1, &rbuf_ledon1, 1);
		rbuf_ledon1 = rbuf_ledon1 & 0xFD;
		led_i2c_write (I2C_LED_LEDON1, rbuf_ledon1);
		led_i2c_write (I2C_LED_A2SET1, 0x00);
		break;
	case 1:
		led_use_flag = led_use_flag | LED_FLAG_A2;
		led_i2c_read (I2C_LED_LEDON1, &rbuf_ledon1, 1);
		rbuf_ledon1 = rbuf_ledon1 | SETLED_A2;
		led_i2c_write (I2C_LED_LEDON1, rbuf_ledon1);
/* FUJITSU:2011-09-20 AN32155AB start */
		led_i2c_write (I2C_LED_A2SET1, (state & 0x00FF0000)>>16);
/* FUJITSU:2011-09-20 AN32155AB end */
		break;
	case 2:
	default:
		break;
	}

	switch (g_bt_s) {
	case 0:
		led_use_flag = led_use_flag & (0xFF ^ LED_FLAG_B1 ^ LED_FLAG_B2 ^ LED_FLAG_B3);
		led_i2c_read (I2C_LED_LEDON1, &rbuf_ledon1, 1);
		rbuf_ledon1 = rbuf_ledon1 & SETLED_A2;
		led_i2c_write (I2C_LED_LEDON1, rbuf_ledon1);
		led_i2c_write (I2C_LED_B1SET1, 0x00);
		led_i2c_write (I2C_LED_B2SET1, 0x00);
		led_i2c_write (I2C_LED_B3SET1, 0x00);
		break;
	case 1:
		led_use_flag = led_use_flag | LED_FLAG_B1 | LED_FLAG_B2 | LED_FLAG_B3;
		led_i2c_read (I2C_LED_LEDON1, &rbuf_ledon1, 1);
		rbuf_ledon1 = rbuf_ledon1 | SETLED_B1 | SETLED_B2 | SETLED_B3;
		led_i2c_write (I2C_LED_LEDON1, rbuf_ledon1);
/* FUJITSU:2011-09-08 AN32155AB start */
		led_i2c_write (I2C_LED_B1SET1, (state & 0x0000FF00)>>8);
		led_i2c_write (I2C_LED_B2SET1, AN32155_GREEN_BRIGHTNESS(((state & 0x0000FF00)>>8)));
		led_i2c_write (I2C_LED_B3SET1, (state & 0x0000FF00)>>8);
/* FUJITSU:2011-09-08 AN32155AB end */
		break;
	case 2:
	default:
		break;
	}

	switch (g_bt_r) {
	case 0:
		led_use_flag = led_use_flag & (0xFF ^ LED_FLAG_C2);
		led_i2c_read (I2C_LED_LEDON2, &rbuf_ledon2, 1);
		rbuf_ledon2 = rbuf_ledon2 & 0xFD;
		led_i2c_write (I2C_LED_LEDON2, rbuf_ledon2);
		led_i2c_write (I2C_LED_C2SET1, 0x00);
		break;
	case 1:
		led_use_flag = led_use_flag | LED_FLAG_C2;
		led_i2c_read (I2C_LED_LEDON2, &rbuf_ledon2, 1);
		rbuf_ledon2 = rbuf_ledon2 | SETLED_C2;
		led_i2c_write (I2C_LED_LEDON2, rbuf_ledon2);
/* FUJITSU:2011-09-20 AN32155AB start */
		led_i2c_write (I2C_LED_C2SET1, (state & 0x000000FF));
/* FUJITSU:2011-09-20 AN32155AB end */
		break;
	case 2:
	default:
		break;
	}
/* FUJITSU:2011-10-11 AN32155AB start */
	if ((led_active_flag) && (!led_use_flag)) {
		led_active(0);
	}
	else {
		if(led_cdev->firefly_flag == 1)
			led_set_firefly (led_cdev , 2);
		led_light_on(1);
	}
/* FUJITSU:2011-10-11 AN32155AB end */
}

static void
mobile_led_set (struct led_classdev *led_cdev, enum led_brightness value)
{
	int i;

	if (value){
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_LIGHT -1), 1);
		mdelay(2);
		for (i = 0; i < value; i++) {
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_LIGHT -1), 0);
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_LIGHT -1), 1);
		}
	}
	else {
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_LIGHT -1), 0);
	}
}

ssize_t
led_modeset_store (struct device * dev, struct device_attribute * attr,
                  const char *buf, size_t size)
{
	char *after;
	int on, off;
	unsigned long state = simple_strtoul (buf, &after, 10);
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if(state){
		on = (state >> 4) & 0xFF;
		off = state & 0xFF;

		led_cdev->firefly_flag = 1;
		led_cdev->firefly_on_time = on;
		led_cdev->firefly_off_time = off;
	}
	else {
		led_cdev->firefly_flag = 0;
		led_cdev->firefly_on_time = 0;
		led_cdev->firefly_off_time = 0;
	}
	return size;
}

ssize_t
led_modeset_show (struct device * dev, struct device_attribute * attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	return sprintf (buf, "firefly_flag = %x , firefly_on_time = %x , firefly_off_time = %x\n",
		led_cdev->firefly_flag,led_cdev->firefly_on_time,led_cdev->firefly_off_time);
}

ssize_t
led_notify_store (struct device * dev, struct device_attribute * attr,
		  const char *buf, size_t size)
{
/* FUJITSU:2011-08-25 AN32155AB start */
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul (buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
/* FUJITSU:2012-03-27 PREVIEW LED start */
//		led_notify = (unsigned char)(state & 0x000000ff);
//		led_charge = (unsigned char)((state & 0x0000ff00) >> 8);
		led_notify = (unsigned int)(state & 0x0000ffff);
		led_charge = (unsigned char)((state & 0x00ff0000) >> 16);
/* FUJITSU:2012-03-27 PREVIEW LED end */
		led_charge = led_charge & 0x07;
		printk("led_notify_store state:0x%lx,notify:0x%x,charge:0x%x\n",state,led_notify,led_charge);
/* FUJITSU:2011-10-11 AN32155AB start */
		g_chrg_flag = led_charge;
/* FUJITSU:2011-10-11 AN32155AB end */
		if(ILLUMI_COLOR_MAX > (unsigned char)((state & 0x00ff0000) >> 16)){
			smem_led_illumi_info.color_id = (unsigned char)((state & 0x00ff0000) >> 16);
		}else{
			smem_led_illumi_info.color_id = 0;
		}
		if(ILLUMI_PATTERN_MAX > (unsigned char)((state & 0xff000000) >> 24)){
			smem_led_illumi_info.pattern_id = (unsigned char)((state & 0xff000000) >> 24);
		}else{
			smem_led_illumi_info.pattern_id = 0;
		}
/* FUJITSU:2011-08-25 AN32155AB end */

	}

	return size;
}

ssize_t
led_notify_show (struct device * dev, struct device_attribute * attr,
		 char *buf)
{
/* FUJITSU:2011-08-18 AN32155AB start */
	return sprintf (buf, "pattern= %x color= %x charge=%x notify=%x\n",
						smem_led_illumi_info.pattern_id,
						smem_led_illumi_info.color_id,
						led_charge,
						led_notify);
/* FUJITSU:2011-08-18 AN32155AB end */
}


ssize_t
led_color_store (struct device * dev, struct device_attribute * attr,
                  const char *buf, size_t size)
{
/* FUJITSU:2011-08-25 AN32155AB start */
	char *after;

	char red[3];
	char green[3];
	char blue[3];
	char onTime[9];
	char offTime[9];

	unsigned long red_state;
	unsigned long green_state;
	unsigned long blue_state;
	unsigned long onTime_count;
	unsigned long offTime_count;
	memset(red, 0x00, 3);
	memset(green, 0x00, 3);
	memset(blue, 0x00, 3);
	memset(onTime, 0x00, 9);
	memset(offTime, 0x00, 9);

	strncpy(offTime, buf, 8);
	strncpy(onTime, buf+8, 8);
	strncpy(red, buf+16, 2);
	strncpy(green, buf+18, 2);
	strncpy(blue, buf+20, 2);

	red_state = simple_strtoul (red, &after, 16);
	green_state = simple_strtoul (green, &after, 16);
	blue_state = simple_strtoul (blue, &after, 16);
	onTime_count = simple_strtoul (onTime, &after, 16);
	offTime_count = simple_strtoul (offTime, &after, 16);

	smem_led_color.led_red = (unsigned char)red_state;
	smem_led_color.led_green = (unsigned char)green_state;
	smem_led_color.led_blue = (unsigned char)blue_state;
	smem_led_color.on_time = (unsigned int)onTime_count;
	smem_led_color.off_time = (unsigned int)offTime_count;

	return size;

/* FUJITSU:2011-08-25 AN32155AB end */
}

ssize_t
led_color_show (struct device * dev, struct device_attribute * attr, char *buf)
{
/* FUJITSU:2011-08-18 AN32155AB start */
	return sprintf (buf, "r= %x g= %x b=%x on=%x off=%x\n", smem_led_color.led_red,
															smem_led_color.led_green,
															smem_led_color.led_blue,
															smem_led_color.on_time,
															smem_led_color.off_time);
/* FUJITSU:2011-08-18 AN32155AB end */
}

static struct led_classdev an32155ab_led_data[] = {
  {
   .name = "illumi-light",
/* FUJITSU:2012-2-23 mod start */
#if 0
   .brightness_set = color_led_set,
#else
   .brightness_set = an32155ab_led_set,
#endif
/* FUJITSU:2012-2-23 mod end */
   .brightness = LED_OFF,
   .max_brightness = 2147483647,
   .firefly_flag = 0,
   .firefly_on_time = 0,
   .firefly_off_time = 0,
   },
  {
   .name = "button-backlight",
/* FUJITSU:2012-2-23 mod start */
#if 0
   .brightness_set = button_led_set,
#else
   .brightness_set = an32155ab_led_set,
#endif
/* FUJITSU:2012-2-23 mod end */
   .brightness = LED_OFF,
   .max_brightness = 2147483647,
   .firefly_flag = 0,
   .firefly_on_time = 0,
   .firefly_off_time = 0,
   },
  {
   .name = "mobile-light",
/* FUJITSU:2012-2-23 mod start */
#if 0
   .brightness_set = mobile_led_set,
#else
   .brightness_set = an32155ab_led_set,
#endif
/* FUJITSU:2012-2-23 mod end */
   .brightness = LED_OFF,
   .max_brightness = 16,
   },
};
/* FUJITSU:2012-2-23 add start */
static void an32155ab_led_work(struct work_struct * w)
{
	struct led_classdev * led = container_of(w, struct led_classdev, work);
/* FUJITSU:2012-3-09 mod start */
	if (!strcmp(led->name, "illumi-light"))
		color_led_set(led, led->w_brightness);
	else if (!strcmp(led->name, "button-backlight"))
		button_led_set(led, led->w_brightness);
	else if (!strcmp(led->name, "mobile-light"))
		mobile_led_set(led, led->w_brightness);
	else
		pr_err("unknown LED device %s\n", led->name);
/* FUJITSU:2012-3-09 mod end */
}
/* FUJITSU:2012-2-23 add end */

static int
an32155ab_led_probe (struct platform_device *pdev)
{
	int rc, i;

	i2c_led = i2c_get_adapter (0);

	if (IS_ERR (i2c_led))
		return PTR_ERR (i2c_led);

	for (i = 0; i <= 2; i++){
		INIT_WORK(&an32155ab_led_data[i].work, an32155ab_led_work);
		rc = led_classdev_register (&pdev->dev, &an32155ab_led_data[i]);
		if (rc){
			printk (KERN_ERR "unable to register led class driver :%s\n",
			an32155ab_led_data[i].name);
			return rc;
		}
/* FUJITSU:2012-1-18 mod start */
#if 0
		timer_trig_activate(&an32155ab_led_data[i]);
#else
		timer_trig_probe(&an32155ab_led_data[i]);
#endif
/* FUJITSU:2012-1-18 mod end */
	}

/* FUJITSU:2012-2-23 add start */
	an32155ab_workqueue = create_singlethread_workqueue("an32155ab_led");
/* FUJITSU:2012-2-23 add end */
	return rc;
}

static int __devexit
an32155ab_led_remove (struct platform_device *pdev)
{
	int i;

	for (i = 0; i <= 2; i++){
		led_classdev_unregister (&an32155ab_led_data[i]);
	}
/* FUJITSU:2012-2-23 add start */
	destroy_workqueue(an32155ab_workqueue);
/* FUJITSU:2012-2-23 add end */
	return 0;
}

static int
an32155ab_led_suspend (struct platform_device *dev, pm_message_t state)
{
/* FUJITSU:2011-08-18 AN32155AB start */
/* FUJITSU:2011-08-25 AN32155AB start */
	static unsigned int *led_ctrl_notify = NULL;
/* FUJITSU:2011-08-25 AN32155AB start */
	static unsigned char *led_ctrl_charge = NULL;
	static struct smem_led_color_type *led_ctrl_color = NULL;
/* FUJITSU:2011-08-25 AN32155AB start */
	static struct smem_led_illumi_info *led_illumi_info = NULL;
/* FUJITSU:2011-08-25 AN32155AB end */

	if (led_ctrl_notify == NULL) {
/* FUJITSU:2011-08-25 AN32155AB start */
		led_ctrl_notify = (unsigned int *)smem_alloc_vendor1(SMEM_OEM_002);
/* FUJITSU:2011-08-25 AN32155AB end */
	}

	if (led_ctrl_charge == NULL) {
		led_ctrl_charge = (unsigned char *)smem_alloc_vendor1(SMEM_OEM_003);
	}

	if (led_ctrl_color == NULL) {
		led_ctrl_color = (struct smem_led_color_type *)smem_alloc_vendor1(SMEM_OEM_010);
	}
/* FUJITSU:2011-08-25 AN32155AB start */
	if (led_illumi_info == NULL) {
		led_illumi_info = (struct smem_led_illumi_info *)smem_alloc_vendor1(SMEM_OEM_014);
	}
/* FUJITSU:2011-08-25 AN32155AB end */

	if (led_ctrl_notify != NULL) {
		*led_ctrl_notify = led_notify;
	}

	if (led_ctrl_charge != NULL) {
		*led_ctrl_charge = led_charge;
	}

	if (led_ctrl_color != NULL) {
		*led_ctrl_color = smem_led_color;
	}

/* FUJITSU:2011-08-25 AN32155AB start */
	if (led_illumi_info != NULL) {
		*led_illumi_info = smem_led_illumi_info;
	}
/* FUJITSU:2011-08-25 AN32155AB end */

	return 0;
/* FUJITSU:2011-08-18 AN32155AB end */
}

static int
an32155ab_led_resume (struct platform_device *dev)
{
	return 0;
}

static struct platform_driver an32155ab_led_driver = {
	.probe = an32155ab_led_probe,
	.remove = an32155ab_led_remove,
	.suspend = an32155ab_led_suspend,
	.resume = an32155ab_led_resume,
	.driver = {
	.name = "an32155ab-leds",
	.owner = THIS_MODULE,
	},
};

static int __init
i2c_led_init (void)
{

	int rc;

	rc = gpio_tlmm_config(GPIO_CFG(LED_RST_GPIO, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		printk (KERN_ERR "[illumi log]%s: Could not configure gpio %d\n",
			__func__, LED_RST_GPIO);
	}

	return platform_driver_register (&an32155ab_led_driver);
}
module_init (i2c_led_init);

static void __exit
i2c_led_exit (void)
{
	platform_driver_unregister (&an32155ab_led_driver);
}
module_exit (i2c_led_exit);

MODULE_AUTHOR("FUJITSU");
MODULE_DESCRIPTION ("AN32155AB LED driver");
MODULE_LICENSE ("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS ("platform:an32155ab-leds");
