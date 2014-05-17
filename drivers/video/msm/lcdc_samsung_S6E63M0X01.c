/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/

/* FUJITSU:2011-11-02 start */
#define CONFIG_SPI_QUP

//#include <linux/mmc/wlan_power.h>
#include <linux/clk.h>
#include <mach/vreg.h>
#include <mach/gpio.h>
/* FUJITSU:2011-11-02 end */

#include <linux/delay.h>
#include <linux/pwm.h>
#ifdef CONFIG_SPI_QUP
#include <linux/spi/spi.h>
#else
#include <mach/gpio.h>
#endif
#include "msm_fb.h"


/* FUJITSU:2011-08-31 DEBUG disable start */	
//#define DEBUG	
/* FUJITSU:2011-08-31 DEBUG disable end */
/* #define SYSFS_DEBUG_CMD */

#ifdef CONFIG_SPI_QUP
#define LCDC_SAMSUNG_SPI_DEVICE_NAME	"lcdc_samsung_ams367pe02"
static struct spi_device *lcdc_spi_client;
#else
static int spi_cs;
static int spi_sclk;
static int spi_mosi;
#endif

struct samsung_state_type {
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
	int brightness;
};

/* FUJITSU:2011-11-02 start */
#if 1
struct samsung_spi_data {
	u8 addr;
	u8 len;
	u8 data[32];
};

static struct samsung_spi_data panel_sequence[] = {
	{ .addr = 0xf8, .len = 14, .data = { 0x01, 0x27, 0x27, 0x07, 0x07,
	 0x54, 0x9f, 0x63, 0x86, 0x1a, 0x33, 0x0d, 0x00, 0x00 } },
};
static struct samsung_spi_data display_sequence[] = {
	{ .addr = 0xf2, .len = 5, .data = { 0x02, 0x03, 0x1c, 0x10, 0x10 } },
	{ .addr = 0xf7, .len = 3, .data = { 0x03, 0x00, 0x00 } },
};

/* FUJITSU:2011-09-01 brightness control start */
static struct samsung_spi_data gamma_sequence_level14[] = {		//	brightness:300 - 280
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x42, 0x51,
	 0x25, 0xb0, 0xba, 0xa5, 0xab, 0xb3, 0x9f, 0xbb, 0xc2, 0xb3, 0x00, 0xb8,
	 0x00, 0xa9, 0x00, 0xf0 } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level13[] = {		//	brightness:280 - 260
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x40, 0x4c,
	 0x25, 0xb2, 0xbb, 0xa8, 0xab, 0xb4, 0x9f, 0xbe, 0xc4, 0xb5, 0x00, 0xb1,
	 0x00, 0xa3, 0x00, 0xe8 } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level12[] = {		//	brightness:260 - 240
/* FUJITSU:2011-10-07 gamma value change start */
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x40, 0x4d,
	 0x23, 0xb3, 0xbc, 0xa8, 0xac, 0xb5, 0xa1, 0xbf, 0xc5, 0xb6, 0x00, 0xab,
	 0x00, 0x9e, 0x00, 0xe0 } },
/* FUJITSU:2011-10-07 gamma value change end */
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level11[] = {		//	brightness:240 - 220
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x42, 0x4b,
	 0x28, 0xb3, 0xbc, 0xa8, 0xae, 0xb7, 0xa3, 0xc0, 0xc6, 0xb7, 0x00, 0xa5,
	 0x00, 0x98, 0x00, 0xd8 } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level10[] = {		//	brightness:220 - 200
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x43, 0x4b,
	 0x2a, 0xb3, 0xbc, 0xa8, 0xaf, 0xb8, 0xa4, 0xc2, 0xc8, 0xb9, 0x00, 0x9f,
	 0x00, 0x92, 0x00, 0xd0 } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level9[] = {		//	brightness:200 - 180
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x44, 0x4d,
	 0x2a, 0xb4, 0xbd, 0xa9, 0xaf, 0xb9, 0xa5, 0xc3, 0xc9, 0xba, 0x00, 0x99,
	 0x00, 0x8d, 0x00, 0xc8 } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level8[] = {		//	brightness:180 - 160
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x44, 0x4a,
	 0x2a, 0xb5, 0xbe, 0xab, 0xb1, 0xba, 0xa6, 0xc5, 0xca, 0xbc, 0x00, 0x91,
	 0x00, 0x86, 0x00, 0xbe } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level7[] = {		//	brightness:160 - 140
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x45, 0x49,
	 0x2a, 0xb5, 0xbe, 0xab, 0xb2, 0xbb, 0xa7, 0xc6, 0xcc, 0xbd, 0x00, 0x8b,
	 0x00, 0x80, 0x00, 0xb6 } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level6[] = {		//	brightness:140 - 120
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x47, 0x46,
	 0x2c, 0xb5, 0xbe, 0xab, 0xb4, 0xbd, 0xa9, 0xc8, 0xce, 0xbf, 0x00, 0x83,
	 0x00, 0x79, 0x00, 0xac } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level5[] = {		//	brightness:120 - 100
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x49, 0x43,
	 0x2e, 0xb7, 0xc0, 0xad, 0xb5, 0xbe, 0xaa, 0xc9, 0xd0, 0xc1, 0x00, 0x7b,
	 0x00, 0x71, 0x00, 0xa1 } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level4[] = {		//	brightness:100 - 80
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x49, 0x3d,
	 0x2e, 0xb7, 0xc0, 0xad, 0xb7, 0xbf, 0xac, 0xcc, 0xd2, 0xc5, 0x00, 0x72,
	 0x00, 0x69, 0x00, 0x95 } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level3[] = {		//	brightness:80 - 60
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x4b, 0x35,
	 0x2e, 0xb8, 0xc0, 0xae, 0xb8, 0xc1, 0xad, 0xcc, 0xd3, 0xc5, 0x00, 0x69,
	 0x00, 0x61, 0x00, 0x8a } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level2[] = {		//	brightness:60 - 40
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x4d, 0x26,
	 0x31, 0xb9, 0xbf, 0xaf, 0xbb, 0xc3, 0xb1, 0xcf, 0xd5, 0xc7, 0x00, 0x5d,
	 0x00, 0x56, 0x00, 0x7b } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level1[] = {		//	brightness:40
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x47, 0x38,
	 0x29, 0xb9, 0xbf, 0xaf, 0xbc, 0xc4, 0xb2, 0xd3, 0xd8, 0xcc, 0x00, 0x4e,
	 0x00, 0x49, 0x00, 0x68 } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data gamma_sequence_level0[] = {		//	brightness: < 40
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x47, 0x38,
	 0x29, 0xb9, 0xbf, 0xaf, 0xbc, 0xc4, 0xb2, 0xd3, 0xd8, 0xcc, 0x00, 0x4e,
	 0x00, 0x49, 0x00, 0x68 } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};
/* FUJITSU:2011-09-01 brightness control end */

static struct samsung_spi_data etc_sequence[] = {
	{ .addr = 0xF6, .len = 3, .data = { 0x00, 0x8e, 0x07 } },
	{ .addr = 0xB3, .len = 1, .data = { 0x6C } },
	{ .addr = 0xB5, .len = 32, .data = { 0x2C, 0x12, 0x0c, 0x0a, 0x10, 0x0e,
	 0x17, 0x13, 0x1f, 0x1a, 0x2a, 0x24, 0x1f, 0x1b, 0x1a, 0x17, 0x2b, 0x26,
	 0x22, 0x20, 0x3a, 0x34, 0x30, 0x2c, 0x29, 0x26, 0x25, 0x23, 0x21, 0x20,
	 0x1e, 0x1e } },
	{ .addr = 0xB6, .len = 16, .data = { 0x00, 0x00, 0x11, 0x22, 0x33,
	 0x44, 0x44, 0x44, 0x55, 0x55, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66 } },
	{ .addr = 0xB7, .len = 32, .data = { 0x2c, 0x12, 0x0c, 0x0a, 0x10, 0x0e,
	 0x17, 0x13, 0x1f, 0x1a, 0x2a, 0x24, 0x1f, 0x1b, 0x1a, 0x17, 0x2b, 0x26,
	 0x22, 0x20, 0x3a, 0x34, 0x30, 0x2c, 0x29, 0x26, 0x25, 0x23, 0x21, 0x20,
	 0x1e, 0x1e } },
	{ .addr = 0xB8, .len = 16, .data = { 0x00, 0x00, 0x11, 0x22, 0x33,
	 0x44, 0x44, 0x44, 0x55, 0x55, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66 } },
	{ .addr = 0xB9, .len = 32, .data = { 0x2c, 0x12, 0x0c, 0x0a, 0x10, 0x0e,
	 0x17, 0x13, 0x1f, 0x1a, 0x2a, 0x24, 0x1f, 0x1b, 0x1a, 0x17, 0x2b, 0x26,
	 0x22, 0x20, 0x3a, 0x34, 0x30, 0x2c, 0x29, 0x26, 0x25, 0x23, 0x21, 0x20,
	 0x1e, 0x1e } },
	{ .addr = 0xBA, .len = 16, .data = { 0x00, 0x00, 0x11, 0x22, 0x33,
	 0x44, 0x44, 0x44, 0x55, 0x55, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66 } },
//        { .addr = 0xB1, .len = 3, .data = { 0x0B, 0x00, 0x16 } }, // add	/* FUJITSU:2011-11-15 del */
};
#else
/* FUJITSU:2011-11-02 end */
struct samsung_spi_data {
	u8 addr;
	u8 len;
	u8 data[22];
};

static struct samsung_spi_data panel_sequence[] = {
	{ .addr = 0xf8, .len = 14, .data = { 0x01, 0x27, 0x27, 0x07, 0x07,
	 0x54, 0x9f, 0x63, 0x86, 0x1a, 0x33, 0x0d, 0x00, 0x00 } },
};
static struct samsung_spi_data display_sequence[] = {
	{ .addr = 0xf2, .len = 5, .data = { 0x02, 0x03, 0x1c, 0x10, 0x10 } },
	{ .addr = 0xf7, .len = 3, .data = { 0x00, 0x00, 0x30 } },
};

/* lum=300 cd/m2 */
static struct samsung_spi_data gamma_sequence_300[] = {
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x7d, 0x77,
	 0x5b, 0xbe, 0xc1, 0xb1, 0xb3, 0xb7, 0xa6, 0xc3, 0xc5, 0xb9, 0x00, 0xb3,
	 0x00, 0xaf, 0x00, 0xe8 } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};
/* lum = 180 cd/m2*/
static struct samsung_spi_data gamma_sequence_180[] = {
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x83, 0x78,
	 0x60, 0xc5, 0xc6, 0xb8, 0xba, 0xbe, 0xad, 0xcb, 0xcd, 0xc2, 0x00, 0x92,
	 0x00, 0x8e, 0x00, 0xbc } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};
/* lum = 80 cd/m2*/
static struct samsung_spi_data gamma_sequence_80[] = {
	{ .addr = 0xfa, .len = 22, .data = { 0x02, 0x18, 0x08, 0x24, 0x94, 0x73,
	 0x6c, 0xcb, 0xca, 0xbe, 0xc4, 0xc7, 0xb8, 0xd3, 0xd5, 0xcb, 0x00, 0x6d,
	 0x00, 0x69, 0x00, 0x8b } },
	{ .addr = 0xFA, .len = 1, .data = { 0x03 } },
};

static struct samsung_spi_data etc_sequence[] = {
	{ .addr = 0xF6, .len = 3, .data = { 0x00, 0x8e, 0x07 } },
	{ .addr = 0xB3, .len = 1, .data = { 0x0C } },
};
/* FUJITSU:2011-11-02 start */
#endif
/* FUJITSU:2011-11-02 end */

/* FUJITSU:2011-11-04 ACL control start */
static struct samsung_spi_data acl_sequence[] = {
	{ .addr = 0xc0, .len = 1, .data = { 0x01 } },
	{ .addr = 0xc1, .len = 27, .data = { 0x4d, 0x96, 0x1d, 0x00, 0x00, 0x01,
	 0xdf, 0x00, 0x00, 0x03, 0x1f, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x02,
	 0x02, 0x03, 0x06, 0x09, 0x0d, 0x0f, 0x12, 0x15, 0x18 } },
};
/* FUJITSU:2011-11-04 ACL control end */

/* FUJITSU:2011-09-01 brightness control start */
//static struct samsung_state_type samsung_state = { .brightness = 180 };
static struct samsung_state_type samsung_state = { .brightness = 15 };
/* FUJITSU:2011-09-01 brightness control end */
static struct msm_panel_common_pdata *lcdc_samsung_pdata;

/* FUJITSU:2011-09-01 brightness control start */
static bool brightness_set_flag = FALSE;

static void brightness_control(int brightness);
extern int factory_mode(void);
/* FUJITSU:2011-09-01 brightness control end */

/* FUJITSU:2011-09-12 report SPI Status start */
extern int set_spi_gpio_exclusive_ctrl(int runtype, int csno);

static bool spi_status = FALSE;
static void report_spi_status(bool spi_request)
{
	int ret = 0;
	if((spi_status == FALSE) && (spi_request == TRUE))	//	off -> on
	{
		ret = set_spi_gpio_exclusive_ctrl(1,1);
		printk("spi status off->on \n");
		spi_status = TRUE;
	}
	else if((spi_status == TRUE) && (spi_request == FALSE))
	{
		ret = set_spi_gpio_exclusive_ctrl(0,1);
		printk("spi status on->off\n");
		spi_status = FALSE;
	}
	else
	{
		printk("spi status not change\n");
	}

	if(ret != 0)
	{
		printk("set_spi_gpio_exclusive_ctrl() = %d\n",ret);
	}
}
/* FUJITSU:2011-09-12 report SPI Status end */

/* FUJITSU:2011-10-12 Lcd status check start */
static bool lcdon = FALSE;
bool get_lcd_status(void)
{
	return lcdon;
}
/* FUJITSU:2011-10-12 Lcd status check end */

#ifndef CONFIG_SPI_QUP
static void samsung_spi_write_byte(boolean dc, u8 data)
{
	uint32 bit;
	int bnum;

	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_mosi, dc ? 1 : 0);
	udelay(1);			/* at least 20 ns */
	gpio_set_value(spi_sclk, 1);	/* clk high */
	udelay(1);			/* at least 20 ns */

	bnum = 8;			/* 8 data bits */
	bit = 0x80;
	while (bnum--) {
		gpio_set_value(spi_sclk, 0); /* clk low */
		gpio_set_value(spi_mosi, (data & bit) ? 1 : 0);
		udelay(1);
		gpio_set_value(spi_sclk, 1); /* clk high */
		udelay(1);
		bit >>= 1;
	}
	gpio_set_value(spi_mosi, 0);

}

static void samsung_spi_read_bytes(u8 cmd, u8 *data, int num)
{
	int bnum;

	/* Chip Select - low */
	gpio_set_value(spi_cs, 0);
	udelay(2);

	/* command byte first */
	samsung_spi_write_byte(0, cmd);
	udelay(2);

	gpio_direction_input(spi_mosi);

	if (num > 1) {
		/* extra dummy clock */
		gpio_set_value(spi_sclk, 0);
		udelay(1);
		gpio_set_value(spi_sclk, 1);
		udelay(1);
	}

	/* followed by data bytes */
	bnum = num * 8;	/* number of bits */
	*data = 0;
	while (bnum) {
		gpio_set_value(spi_sclk, 0); /* clk low */
		udelay(1);
		*data <<= 1;
		*data |= gpio_get_value(spi_mosi) ? 1 : 0;
		gpio_set_value(spi_sclk, 1); /* clk high */
		udelay(1);
		--bnum;
		if ((bnum % 8) == 0)
			++data;
	}

	gpio_direction_output(spi_mosi, 0);

	/* Chip Select - high */
	udelay(2);
	gpio_set_value(spi_cs, 1);
}
#endif

#ifdef DEBUG
static const char *byte_to_binary(const u8 *buf, int len)
{
/* FUJITSU:2012-02-24 DISP fix buffer overflow start */
//	static char b[32*8+1];
	static char b[100*8+1];
/* FUJITSU:2012-02-24 DISP fix buffer overflow end */
	char *p = b;
	int i, z;

	for (i = 0; i < len; ++i) {
		u8 val = *buf++;
		for (z = 1 << 7; z > 0; z >>= 1)
			*p++ = (val & z) ? '1' : '0';
	}
	*p = 0;

	return b;
}
#endif

#define BIT_OFFSET	(bit_size % 8)
#define ADD_BIT(val) do { \
		tx_buf[bit_size / 8] |= \
			(u8)((val ? 1 : 0) << (7 - BIT_OFFSET)); \
		++bit_size; \
	} while (0)

#define ADD_BYTE(data) do { \
		tx_buf[bit_size / 8] |= (u8)(data >> BIT_OFFSET); \
		bit_size += 8; \
		if (BIT_OFFSET != 0) \
			tx_buf[bit_size / 8] |= (u8)(data << (8 - BIT_OFFSET));\
	} while (0)

static int samsung_serigo(struct samsung_spi_data data)
{
#ifdef CONFIG_SPI_QUP
/* FUJITSU:2011-11-02 start */
	//char                tx_buf[32];
	char                tx_buf[100];//tx_buf[64];
/* FUJITSU:2011-11-02 end */
	int                 bit_size = 0, i, rc;
	struct spi_message  m;
	struct spi_transfer t;

	if (!lcdc_spi_client) {
		pr_err("%s lcdc_spi_client is NULL\n", __func__);
		return -EINVAL;
	}

	memset(&t, 0, sizeof t);
	memset(tx_buf, 0, sizeof tx_buf);
	t.tx_buf = tx_buf;
	spi_setup(lcdc_spi_client);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ADD_BIT(FALSE);
	ADD_BYTE(data.addr);
/* FUJITSU:2011-11-02 start */
	ADD_BIT(FALSE);//1
	ADD_BIT(FALSE);//2
	ADD_BIT(FALSE);//3
	ADD_BIT(FALSE);//4
	ADD_BIT(FALSE);//5
	ADD_BIT(FALSE);//6
	ADD_BIT(FALSE);//7
/* FUJITSU:2011-11-02 end */
	for (i = 0; i < data.len; ++i) {
		ADD_BIT(TRUE);
		ADD_BYTE(data.data[i]);
/* FUJITSU:2011-11-02 start */
		ADD_BIT(FALSE);//1
		ADD_BIT(FALSE);//2
		ADD_BIT(FALSE);//3
		ADD_BIT(FALSE);//4
		ADD_BIT(FALSE);//5
		ADD_BIT(FALSE);//6
		ADD_BIT(FALSE);//7
/* FUJITSU:2011-11-02 end */
	}

	/* add padding bits so we round to next byte */
	t.len = (bit_size+7) / 8;
/* FUJITSU:2011-11-02 start */
#if 0
	if (t.len <= 4)
		t.bits_per_word = bit_size;
#endif
/* FUJITSU:2011-11-02 end */
	rc = spi_sync(lcdc_spi_client, &m);

#ifdef DEBUG
	pr_info("%s: addr=0x%02x, #args=%d[%d] [%s], rc=%d\n",
		__func__, data.addr, t.len, t.bits_per_word,
		byte_to_binary(tx_buf, t.len), rc);
#endif
	return rc;
#else
	int i;

	/* Chip Select - low */
	gpio_set_value(spi_cs, 0);
	udelay(2);

	samsung_spi_write_byte(FALSE, data.addr);
	udelay(2);

	for (i = 0; i < data.len; ++i) {
		samsung_spi_write_byte(TRUE, data.data[i]);
		udelay(2);
	}

	/* Chip Select - high */
	gpio_set_value(spi_cs, 1);
#ifdef DEBUG
	pr_info("%s: cmd=0x%02x, #args=%d\n", __func__, data.addr, data.len);
#endif
	return 0;
#endif
}

static int samsung_write_cmd(u8 cmd)
{
#ifdef CONFIG_SPI_QUP
	char                tx_buf[2];
	int                 bit_size = 0, rc;
	struct spi_message  m;
	struct spi_transfer t;

	if (!lcdc_spi_client) {
		pr_err("%s lcdc_spi_client is NULL\n", __func__);
		return -EINVAL;
	}

	memset(&t, 0, sizeof t);
	memset(tx_buf, 0, sizeof tx_buf);
	t.tx_buf = tx_buf;
	spi_setup(lcdc_spi_client);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ADD_BIT(FALSE);
	ADD_BYTE(cmd);

	t.len = 2;
	t.bits_per_word = 9;

	rc = spi_sync(lcdc_spi_client, &m);
#ifdef DEBUG
	pr_info("%s: addr=0x%02x, #args=%d[%d] [%s], rc=%d\n",
		__func__, cmd, t.len, t.bits_per_word,
		byte_to_binary(tx_buf, t.len), rc);
#endif
	return rc;
#else
	/* Chip Select - low */
	gpio_set_value(spi_cs, 0);
	udelay(2);

	samsung_spi_write_byte(FALSE, cmd);

	/* Chip Select - high */
	udelay(2);
	gpio_set_value(spi_cs, 1);
#ifdef DEBUG
	pr_info("%s: cmd=0x%02x\n", __func__, cmd);
#endif
	return 0;
#endif
}

static int samsung_serigo_list(struct samsung_spi_data *data, int count)
{
	int i, rc;
	for (i = 0; i < count; ++i, ++data) {
		rc = samsung_serigo(*data);
		if (rc)
			return rc;
/* FUJITSU:2011-11-11 modify start */
		//msleep(10);
		mdelay(1);
/* FUJITSU:2011-11-11 modify end */
	}
	return 0;
}

#ifndef CONFIG_SPI_QUP
static void samsung_spi_init(void)
{
	spi_sclk = *(lcdc_samsung_pdata->gpio_num);
	spi_cs   = *(lcdc_samsung_pdata->gpio_num + 1);
	spi_mosi = *(lcdc_samsung_pdata->gpio_num + 2);

	/* Set the output so that we don't disturb the slave device */
	gpio_set_value(spi_sclk, 1);
	gpio_set_value(spi_mosi, 0);

	/* Set the Chip Select deasserted (active low) */
	gpio_set_value(spi_cs, 1);
}
#endif

/* FUJITSU:2011-11-02 start */
#if 0 //2011-07-22
static void samsung_disp_powerup(void)
{
	if (!samsung_state.disp_powered_up && !samsung_state.display_on)
		samsung_state.disp_powered_up = TRUE;
}
#else
/* FUJITSU:2011-11-02 end */
/* FUJITSU:2011-11-02 start */
void samsung_disp_power_on(void)
{
	printk("### samsung_disp_power_on()-begin-\n");

	if (!samsung_state.disp_powered_up && !samsung_state.display_on)
		samsung_state.disp_powered_up = TRUE;

    //gpio init
    gpio_tlmm_config(GPIO_CFG(33, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
 
    //Reset
    gpio_set_value(33, 0);
    mdelay(1);  //min 1ms

    gpio_set_value(33, 1);

	brightness_set_flag = FALSE;	/* FUJITSU:2011-09-01 brightness control */

	report_spi_status(TRUE);	//spi use	/* FUJITSU:2011-09-12 report SPI Status */

	lcdon = TRUE;	/* FUJITSU:2011-10-12 Lcd status check */

	printk("### samsung_disp_power_on()-end-\n");
}
#endif
/* FUJITSU:2011-11-02 end */



static struct work_struct disp_on_delayed_work;
static void samsung_disp_on_delayed_work(struct work_struct *work_ptr)
{
/* FUJITSU:2011-11-02 start */
#if 0
	/* 0x01: Software Reset */
	samsung_write_cmd(0x01);
	msleep(120);

	msleep(300);
#endif
/* FUJITSU:2011-11-02 end */

/* FUJITSU:2011-10-17 wait modify start */
    //mdelay(10); //2011-07-22 add
	msleep(10);
/* FUJITSU:2011-10-17 wait modify end */

	samsung_serigo_list(panel_sequence,
		sizeof(panel_sequence)/sizeof(*panel_sequence));
	samsung_serigo_list(display_sequence,
		sizeof(display_sequence)/sizeof(*display_sequence));

/* FUJITSU:2011-09-01 brightness control start */
#if 1
    brightness_control(samsung_state.brightness);
#else
	switch (samsung_state.brightness) {
	case 300:
		samsung_serigo_list(gamma_sequence_300,
			sizeof(gamma_sequence_300)/sizeof(*gamma_sequence_300));
		break;
	case 180:
	default:
		samsung_serigo_list(gamma_sequence_180,
			sizeof(gamma_sequence_180)/sizeof(*gamma_sequence_180));
		break;
	case 80:
		samsung_serigo_list(gamma_sequence_80,
			sizeof(gamma_sequence_80)/sizeof(*gamma_sequence_80));
		break;
	}
#endif
/* FUJITSU:2011-09-01 brightness control end */

	samsung_serigo_list(etc_sequence,
		sizeof(etc_sequence)/sizeof(*etc_sequence));

/* FUJITSU:2011-11-04 ACL control start */
	samsung_serigo_list(acl_sequence,
		sizeof(acl_sequence)/sizeof(*acl_sequence));
/* FUJITSU:2011-11-04 ACL control end */

	/* 0x11: Sleep Out */
	samsung_write_cmd(0x11);

/* FUJITSU:2011-10-17 wait modify start */
	//mdelay(120);
	msleep(120);
/* FUJITSU:2011-10-17 wait modify end */

/* FUJITSU:2011-11-02 start */
#if 0
	msleep(120);
	/* 0x13: Normal Mode On */
	samsung_write_cmd(0x13);

#ifndef CONFIG_SPI_QUP
	{
		u8 data;

		msleep(120);
		/* 0x0A: Read Display Power Mode */
		samsung_spi_read_bytes(0x0A, &data, 1);
		pr_info("%s: power=[%s]\n", __func__,
			byte_to_binary(&data, 1));

		msleep(120);
		/* 0x0C: Read Display Pixel Format */
		samsung_spi_read_bytes(0x0C, &data, 1);
		pr_info("%s: pixel-format=[%s]\n", __func__,
			byte_to_binary(&data, 1));
	}
#endif
	msleep(120);
#endif
/* FUJITSU:2011-11-02 end */
	/* 0x29: Display On */
	samsung_write_cmd(0x29);
}

static void samsung_disp_on(void)
{
	if (samsung_state.disp_powered_up && !samsung_state.display_on) {

/* FUJITSU:2011-11-02 start */
#if 0 // 2011-07-22 start
		INIT_WORK(&disp_on_delayed_work, samsung_disp_on_delayed_work);
		schedule_work(&disp_on_delayed_work);
#else
        samsung_disp_on_delayed_work(&disp_on_delayed_work);
#endif // 2011-07-22 end
/* FUJITSU:2011-11-02 end */

		samsung_state.display_on = TRUE;
	}
}

static int lcdc_samsung_panel_on(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	if (!samsung_state.disp_initialized) {
#ifndef CONFIG_SPI_QUP
		lcdc_samsung_pdata->panel_config_gpio(1);
		samsung_spi_init();
#endif

/* FUJITSU:2011-11-02 start */
#if 0 //2011-07-22 move to mdp4_overlay_lcdc.c mdp_lcdc_on()
		samsung_disp_powerup();
#endif
/* FUJITSU:2011-11-02 end */

		samsung_disp_on();
		samsung_state.disp_initialized = TRUE;
	}
	pr_info("%s end\n", __func__);	/* FUJITSU:2011-10-17 add message */
	return 0;
}

static int lcdc_samsung_panel_off(struct platform_device *pdev)
{
    
	pr_info("%s\n", __func__);
/* FUJITSU:2011-10-12 Lcd status check start */
	lcdon = FALSE;
/* FUJITSU:2011-10-12 Lcd status check end */
	if (samsung_state.disp_powered_up && samsung_state.display_on) {
		/* 0x10: Sleep In */
		samsung_write_cmd(0x10);
		msleep(120);
        
/* FUJITSU:2011-09-12 report SPI Status start */
		report_spi_status(FALSE);	//spi not use
/* FUJITSU:2011-09-12 report SPI Status end */

		samsung_state.display_on = FALSE;
		samsung_state.disp_initialized = FALSE;
	}
	pr_info("%s end\n", __func__);	/* FUJITSU:2011-10-17 add message */
	return 0;
}

#ifdef SYSFS_DEBUG_CMD
static ssize_t samsung_rda_cmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret = snprintf(buf, PAGE_SIZE, "n/a\n");
	pr_info("%s: 'n/a'\n", __func__);
	return ret;
}

static ssize_t samsung_wta_cmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	uint32 cmd;

	sscanf(buf, "%x", &cmd);
	samsung_write_cmd((u8)cmd);

	return ret;
}

static DEVICE_ATTR(cmd, S_IRUGO | S_IWUGO, samsung_rda_cmd, samsung_wta_cmd);
static struct attribute *fs_attrs[] = {
	&dev_attr_cmd.attr,
	NULL,
};
static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};
#endif

static int samsung_probe(struct platform_device *pdev)
{
#ifdef SYSFS_DEBUG_CMD
	struct platform_device *fb_dev;
	struct msm_fb_data_type *mfd;
	int rc;
#endif

	pr_info("%s: id=%d\n", __func__, pdev->id);
	if (pdev->id == 0) {
		lcdc_samsung_pdata = pdev->dev.platform_data;
		return 0;
	}

#ifndef SYSFS_DEBUG_CMD
	msm_fb_add_device(pdev);
#else
	fb_dev = msm_fb_add_device(pdev);
	mfd = platform_get_drvdata(fb_dev);
	rc = sysfs_create_group(&mfd->fbi->dev->kobj, &fs_attr_group);
	if (rc) {
		pr_err("%s: sysfs group creation failed, rc=%d\n", __func__,
			rc);
		return rc;
	}
#endif
	return 0;
}

#ifdef CONFIG_SPI_QUP
static int __devinit lcdc_samsung_spi_probe(struct spi_device *spi)
{
	pr_info("%s\n", __func__);
	lcdc_spi_client = spi;
/* FUJITSU:2011-11-02 start */
//	lcdc_spi_client->bits_per_word = 32;
	lcdc_spi_client->bits_per_word = 9;
/* FUJITSU:2011-11-02 end */
	return 0;
}
static int __devexit lcdc_samsung_spi_remove(struct spi_device *spi)
{
	lcdc_spi_client = NULL;
	return 0;
}
static struct spi_driver lcdc_samsung_spi_driver = {
	.driver.name   = LCDC_SAMSUNG_SPI_DEVICE_NAME,
	.driver.owner  = THIS_MODULE,
	.probe         = lcdc_samsung_spi_probe,
	.remove        = __devexit_p(lcdc_samsung_spi_remove),
};
#endif

/* FUJITSU:2011-09-12 ext panel setting start */
void ext_panel_control(struct samsung_spi_data spi_data)
{
	int i;

	printk(KERN_DEBUG "[LCD]%s: enter\n", __func__);
	printk(KERN_DEBUG "address = %x\n", spi_data.addr);
	printk(KERN_DEBUG "length = %d\n", spi_data.len);

	for(i=0;i<spi_data.len;i++)
		printk(KERN_DEBUG "data[%d] = 0x%02x\n",i,*(spi_data.data+i));

	if(factory_mode())
	{
		printk(KERN_DEBUG "[LCD]: ext_panel_control() enable \n");
		samsung_serigo(spi_data);
	}
	else
	{
		printk(KERN_DEBUG "[LCD]: ext_panel_control() disable \n");
	}
}
/* FUJITSU:2011-09-12 ext panel setting end */

/* FUJITSU:2011-09-01 brightness control start */
void ext_brightness_control(int brightness)
{
    printk(KERN_DEBUG "[LCD]%s: enter Brightness = %d\n", __func__,brightness);
	if(factory_mode())
	{
		printk(KERN_DEBUG "[LCD]: ext_brightness_control() enable \n");
		brightness_control(brightness);
	}
	else
	{
		printk(KERN_DEBUG "[LCD]: ext_brightness_control() disable \n");
	}
}

static void brightness_control(int brightness)
{
    printk(KERN_DEBUG "[LCD]%s: enter Brightness = %d\n", __func__,brightness);

	if(brightness > 15)
		brightness = 15;

	if(brightness < 1)
		brightness = 1;

    if(brightness == 15) {
		samsung_serigo_list(gamma_sequence_level14,
			sizeof(gamma_sequence_level14)/sizeof(*gamma_sequence_level14));
    }
    else if(brightness == 14){
		samsung_serigo_list(gamma_sequence_level13,
			sizeof(gamma_sequence_level13)/sizeof(*gamma_sequence_level13));
    }
    else if(brightness == 13){
		samsung_serigo_list(gamma_sequence_level12,
			sizeof(gamma_sequence_level12)/sizeof(*gamma_sequence_level12));
    }
    else if(brightness == 12){
		samsung_serigo_list(gamma_sequence_level11,
			sizeof(gamma_sequence_level11)/sizeof(*gamma_sequence_level11));
    }
    else if(brightness == 11){
		samsung_serigo_list(gamma_sequence_level10,
			sizeof(gamma_sequence_level10)/sizeof(*gamma_sequence_level10));
    }
    else if(brightness == 10){
		samsung_serigo_list(gamma_sequence_level9,
			sizeof(gamma_sequence_level9)/sizeof(*gamma_sequence_level9));
    }
    else if(brightness == 9){
		samsung_serigo_list(gamma_sequence_level8,
			sizeof(gamma_sequence_level8)/sizeof(*gamma_sequence_level8));
    }
    else if(brightness == 8){
		samsung_serigo_list(gamma_sequence_level7,
			sizeof(gamma_sequence_level7)/sizeof(*gamma_sequence_level7));
    }
    else if(brightness == 7){
		samsung_serigo_list(gamma_sequence_level6,
			sizeof(gamma_sequence_level6)/sizeof(*gamma_sequence_level6));
    }
    else if(brightness == 6){
		samsung_serigo_list(gamma_sequence_level5,
			sizeof(gamma_sequence_level5)/sizeof(*gamma_sequence_level5));
    }
    else if(brightness == 5){
		samsung_serigo_list(gamma_sequence_level4,
			sizeof(gamma_sequence_level4)/sizeof(*gamma_sequence_level4));
    }
    else if(brightness == 4){
		samsung_serigo_list(gamma_sequence_level3,
			sizeof(gamma_sequence_level3)/sizeof(*gamma_sequence_level3));
    }
    else if(brightness == 3){
		samsung_serigo_list(gamma_sequence_level2,
			sizeof(gamma_sequence_level2)/sizeof(*gamma_sequence_level2));
    }
    else if(brightness == 2){
		samsung_serigo_list(gamma_sequence_level1,
			sizeof(gamma_sequence_level1)/sizeof(*gamma_sequence_level1));
    }
    else {
		samsung_serigo_list(gamma_sequence_level0,
			sizeof(gamma_sequence_level0)/sizeof(*gamma_sequence_level0));
    }

    
    //printk(KERN_INFO "[LCD]%s: leave\n", __func__);
    return;
}

static void set_brightness(struct msm_fb_data_type *mfd)
{
    samsung_state.brightness = mfd->bl_level;
    printk(KERN_DEBUG "[LCD]%s: enter Brightness = %d\n", __func__,samsung_state.brightness);
    
	if((brightness_set_flag == FALSE) || (!factory_mode()))
	{
		if(get_lcd_status() != FALSE)
		{
			brightness_control(samsung_state.brightness);
		}
		brightness_set_flag = TRUE;
	}
    
    //printk(KERN_INFO "[LCD]%s: leave\n", __func__);
    return;
}
/* FUJITSU:2011-09-01 brightness control end */

static struct platform_driver this_driver = {
	.probe		= samsung_probe,
	.driver.name	= "lcdc_samsung_oled",
};

static struct msm_fb_panel_data samsung_panel_data = {
	.on = lcdc_samsung_panel_on,
	.off = lcdc_samsung_panel_off,
/* FUJITSU:2011-09-01 brightness control start */
    .set_backlight  = set_brightness
/* FUJITSU:2011-09-01 brightness control end */
};

static struct platform_device this_device = {
	.name	= "lcdc_samsung_oled",
	.id	= 1,
	.dev.platform_data = &samsung_panel_data,
};

/* FUJITSU:2012-02-27 DISP data enable start */
int lcdc_panel_get_data_en_polarity(void)
{
    //Hi enable
    return 1;
}
/* FUJITSU:2012-02-27 DISP data enable end */

static int __init lcdc_samsung_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_LCDC_AUTO_DETECT
	if (msm_fb_detect_client("lcdc_samsung_oled")) {
		pr_err("%s: detect failed\n", __func__);
		return 0;
	}
#endif

	ret = platform_driver_register(&this_driver);
	if (ret) {
		pr_err("%s: driver register failed, rc=%d\n", __func__, ret);
		return ret;
	}

	pinfo = &samsung_panel_data.panel_info;
	pinfo->xres = 480;
	pinfo->yres = 800;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 2;
/* FUJITSU:2011-11-02 start */
#if 1
/* FUJITSU:2012-02-24 DISP mod clk_rate start */
//	pinfo->clk_rate = 23800000;
	pinfo->clk_rate = 24576000;
/* FUJITSU:2012-02-24 DISP mod clk_rate end */
#else
#ifdef CONFIG_ARCH_MSM7X30
	pinfo->clk_rate = 30720000; /* Max 27.77MHz */
#else
	pinfo->clk_rate = 25600000; /* Max 27.77MHz */
#endif
#endif
/* FUJITSU:2011-11-02 end */
/* FUJITSU:2011-09-01 brightness control start */
	pinfo->bl_max = 15;
	pinfo->bl_min = 1;
/* FUJITSU:2011-09-01 brightness control end */

	/* AMS367PE02 Operation Manual, Page 7 */
	pinfo->lcdc.h_back_porch = 16-2;	/* HBP-HLW */
	pinfo->lcdc.h_front_porch = 16;
	pinfo->lcdc.h_pulse_width = 2;
	/* AMS367PE02 Operation Manual, Page 6 */
	pinfo->lcdc.v_back_porch = 3-2;		/* VBP-VLW */
	pinfo->lcdc.v_front_porch = 28;
	pinfo->lcdc.v_pulse_width = 2;

	pinfo->lcdc.border_clr = 0;
	pinfo->lcdc.underflow_clr = 0xff;
	pinfo->lcdc.hsync_skew = 0;

/* FUJITSU:2012-02-27 DISP add LCD actual size start */
        pinfo->actual_height          = 86;
        pinfo->actual_width           = 52;
/* FUJITSU:2012-02-27 DISP add LCD actual size end */

	ret = platform_device_register(&this_device);
	if (ret) {
		pr_err("%s: device register failed, rc=%d\n", __func__, ret);
		goto fail_driver;
	}
#ifdef CONFIG_SPI_QUP
	ret = spi_register_driver(&lcdc_samsung_spi_driver);

	if (ret) {
		pr_err("%s: spi register failed: rc=%d\n", __func__, ret);
		goto fail_device;
	}
	pr_info("%s: SUCCESS (SPI)\n", __func__);
#else
	pr_info("%s: SUCCESS (BitBang)\n", __func__);
#endif
	return ret;

#ifdef CONFIG_SPI_QUP
fail_device:
	platform_device_unregister(&this_device);
#endif
fail_driver:
	platform_driver_unregister(&this_driver);

	return ret;
}

module_init(lcdc_samsung_panel_init);
#ifdef CONFIG_SPI_QUP
static void __exit lcdc_samsung_panel_exit(void)
{
	pr_info("%s\n", __func__);
	spi_unregister_driver(&lcdc_samsung_spi_driver);
}
module_exit(lcdc_samsung_panel_exit);
#endif
