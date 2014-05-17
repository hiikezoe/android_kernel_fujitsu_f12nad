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

#ifndef _BOARD_TOUCH_FJ_C_
#define _BOARD_TOUCH_FJ_C_



#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <mach/vreg.h>
/* FUJITSU:2012-03-30 FUJITSU mod start */
//#include <linux/i2c/atmel_mxt_ts_apo.h>
#include <linux/i2c/atmel_mxt_ts_nad.h>
/* FUJITSU:2012-03-30 FUJITSU mod end */

#if 0
#include <linux/power_supply.h>

extern int msm_battery_get_property(enum power_supply_property psp,
										union power_supply_propval *val);
#endif

#define MXT_CONFIG_CRC         0x5F1C9F
#define MXT_I2C_SLAVE_ADDRESS  0x4A
/* FUJITSU:2012-03-09 FUJITSU mod start */
#define MXT_GPIO_TCH_INT       18
#define MXT_GPIO_TCH_XRST      143
/* FUJITSU:2012-03-09 FUJITSU mod start */

/* FUJITSU:2012-03-30 FUJITSU add start */
#define LED_I2C_SLAVE_ADDR_IN_TOUCH_FJ	0x76
#define MXT_GPIO_LCD_TYPE				105
#define MXT_CONFIG_TCHW_LCDH             0
#define MXT_CONFIG_TCHW_LCDS             1
#define MXT_CONFIG_TCHK_LCDH             2
#define MXT_CONFIG_TCHK_LCDS             3
/* FUJITSU:2012-03-30 FUJITSU add end */


/* GEN_COMMANDCONFIG_T6 */
static u8 mxt_config_t6_data[]  = {
	0,   /* RESET      */
	0,   /* BACKUPNV   */
	0,   /* CALIBRAT   */
	0,   /* REPORTALL  */
	0,   /* DIAGNOSTIC */
};
/* SPT_USERDATA_T38 */
static u8 mxt_config_t38_data[] = {
	84,  /* DATA_0 */
	87,  /* DATA_1 */
	2,   /* DATA_2 */
	4,   /* DATA_3 */
	0,   /* DATA_4 */
	0,   /* DATA_5 */
	0,   /* DATA_6 */
	0,   /* DATA_7 */
};
/* GEN_POWERCONFIG_T7 */
static u8 mxt_config_t7_data[]  = {
	16,  /* IDLEACQINT  */
	16,  /* ACTVACQINT  */
	50,  /* ACTV2IDLETO */
};
/* GEN_ACQUISITIONCONFIG_T8 */
static u8 mxt_config_t8_data[]  = {
	20,  /* CHRGTIME        */
	0,   /* ATCHDRIFT       */
	5,   /* TCHDRIFT        */
	0,   /* DRIFTST         */
	0,   /* TCHAUTOCAL      */
	0,   /* SYNC            */
	4,   /* ATCHCALST       */
	35,  /* ATCHCALSTHR     */
	32,  /* ATCHFRCCALTHR   */
	135, /* ATCHFRCCALRATIO */
};

/*FUJITSU:2012-06-26 Noise WA Add START */
/* GEN_ACQUISITIONCONFIG_T8 */
static u8 mxt_config_t8_auto_recover_off_data[]  = {
	20,  /* CHRGTIME        */
	0,   /* ATCHDRIFT       */
	5,   /* TCHDRIFT        */
	0,   /* DRIFTST         */
	0,   /* TCHAUTOCAL      */
	0,   /* SYNC            */
	0,   /* ATCHCALST       */
	1,   /* ATCHCALSTHR     */
	0,   /* ATCHFRCCALTHR   */
	0,   /* ATCHFRCCALRATIO */
};
/*FUJITSU:2012-06-26 Noise WA Add END */

/* TOUCH_MULTITOUCHSCREEN_T9 */
static u8 mxt_config_t9_data[]  = {
	131, /* CTRL       */
	0,   /* XORIGIN    */
	0,   /* YORIGIN    */
	18,  /* XSIZE      */
	11,  /* YSIZE      */
	0,   /* AKSCFG     */
	16,  /* BLEN       */
	40,  /* TCHTHR     */
	1,   /* TCHDI      */
	1,   /* ORIENT     */
	10,  /* MRGTIMEOUT */
	3,   /* MOVHYSTI   */
	1,   /* MOVHYSTN   */
	46,  /* MOVFILTER  */
	5,   /* NUMTOUCH   */
	1,   /* MRGHYST    */
	253, /* MRGTHR     */
	20,  /* AMPHYST    */
	31,  /* XRANGE_LO 799 */
	3,   /* XRANGE_HI 479 */
	223, /* YRANGE_LO  */
	1,   /* YRANGE_HI  */
	2,   /* XLOCLIP    */
	2,   /* XHICLIP    */
	13,  /* YLOCLIP    */
	13,  /* YHICLIP    */
	216, /* XEDGECTRL  */
	50,  /* XEDGEDIST  */
	224, /* YEDGECTRL  */
	70,  /* YEDGEDIST  */
	35,  /* JUMPLIMIT  */
	10,  /* TCHHYST    */
	49,  /* XPITCH     */
	49,  /* YPITCH     */
	1,   /* NEXTTCHDI  */
};
/* TOUCH_KEYARRAY_T15 */
static u8 mxt_config_t15_data[] = {
	0,   /* CTRL       */
	0,   /* XORIGIN    */
	0,   /* YORIGIN    */
	0,   /* XSIZE      */
	0,   /* YSIZE      */
	0,   /* AKSCFG     */
	0,   /* BLEN       */
	0,   /* TCHTHR     */
	0,   /* TCHDI      */
	0,   /* RESERVED_0 */
	0,   /* RESERVED_1 */
};
static u8 mxt_config_t18_data[] = {
	1,   /* CTRL    */
	0,   /* COMMAND */
};
/* SPT_GPIOPWM_T19 */
static u8 mxt_config_t19_data[] = {
	0,   /* CTRL       */
	0,   /* REPORTMASK */
	0,   /* DIR        */
	0,   /* INTPULLUP  */
	0,   /* OUT        */
	0,   /* WAKE       */
	0,   /* PWM        */
	0,   /* PERIOD     */
	0,   /* DUTY_0     */
	0,   /* DUTY_1     */
	0,   /* DUTY_2     */
	0,   /* DUTY_3     */
	0,   /* TRIGGER_0  */
	0,   /* TRIGGER_1  */
	0,   /* TRIGGER_2  */
	0,   /* TRIGGER_3  */
};
/* TOUCH_PROXIMITY_T23 */
static u8 mxt_config_t23_data[] = {
	0,   /* CTRL          */
	0,   /* XORIGIN       */
	0,   /* YORIGIN       */
	0,   /* XSIZE         */
	0,   /* YSIZE         */
	0,   /* RESERVED_0    */
	0,   /* BLEN          */
	0,   /* FXDDTHR_LS    */
	0,   /* FXDDTHR_MS    */
	0,   /* FXDDI         */
	0,   /* AVERAGE       */
	0,   /* MVNULLRATE_LS */
	0,   /* MVNULLRATE_MS */
	0,   /* MVDTHR_LS     */
	0,   /* MVDTHR_MS     */
};
/* SPT_SELFTEST_T25 */
static u8 mxt_config_t25_data[] = {
	3,   /* CTRL          */
	0,   /* CMD           */
	132, /* HISIGLIM_0_LS 26500 */
	103, /* HISIGLIM_0_MS */
	204, /* LOSIGLIM_0_LS 23500 */
	91,  /* LOSIGLIM_0_MS */
	0,   /* HISIGLIM_1_LS */
	0,   /* HISIGLIM_1_MS */
	0,   /* LOSIGLIM_1_LS */
	0,   /* LOSIGLIM_1_MS */
	0,   /* HISIGLIM_2_LS */
	0,   /* HISIGLIM_2_MS */
	0,   /* LOSIGLIM_2_LS */
	0,   /* LOSIGLIM_2_MS */
};
/* PROCI_GRIPSUPPRESSION_T40 */
static u8 mxt_config_t40_data[] = {
	0,   /* CTRL    */
	0,   /* XLOGRIP */
	0,   /* XHIGRIP */
	0,   /* YLOGRIP */
	0,   /* YHIGRIP */
};
/* PROCI_TOUCHSUPPRESSION_T42 */
static u8 mxt_config_t42_data[] = {
	0,   /* CTRL          */
	26,  /* APPRTHR       */
	58,  /* MAXAPPRAREA   */
	53,  /* MAXTCHAREA    */
	208, /* SUPSTRENGTH   */
	0,   /* SUPEXTTO      */
	0,   /* MAXNUMTCHS    */
	0,   /* SHAPESTRENGTH */
};
/* SPT_CTECONFIG_T46 */
static u8 mxt_config_t46_data[] = {
	0,   /* CTRL          */
	3,   /* MODE          */
	35,  /* IDLESYNCSPERX */
	35,  /* ACTVSYNCSPERX */
	0,   /* ADCSPERSYNC   */
	0,   /* PULSESPERADC  */
	1,   /* XSLEW         */
	0,   /* SYNCDELAY_LS  */
	0,   /* SYNCDELAY_MS  */
};
/* PROCI_STYLUS_T47 */
static u8 mxt_config_t47_data[] = {
	0,   /* CTRL       */
	0,   /* CONTMIN    */
	0,   /* CONTMAX    */
	0,   /* STABILITY  */
	0,   /* MAXTCHAREA */
	0,   /* AMPLTHR    */
	0,   /* STYSHAPE   */
	0,   /* HOVERSUP   */
	0,   /* CONFTHR    */
	0,   /* SYNCSPERX  */
};
/* PROCG_NOISESUPPRESSION_T48 */
static u8 mxt_config_t48_data[] = {
	3,   /* CTRL               */
	4,   /* CFG                */
	98,  /* CALCFG             */
	10,  /* BASEFREQ           */
	0,   /* RESERVED_0         */
	0,   /* RESERVED_1         */
	0,   /* RESERVED_2         */
	0,   /* RESERVED_3         */
	2,   /* MFFREQ_0           */
	3,   /* MFFREQ_1           */
	0,   /* RESERVED_4         */
	0,   /* RESERVED_5         */
	0,   /* RESERVED_6         */
	6,   /* GCACTVINVLDADCS    */
	6,   /* GCIDLEINVLDADCS    */
	0,   /* RESERVED_6         */
	0,   /* RESERVED_7         */
	100, /* GCMAXADCSPERX      */
	4,   /* GCLIMITMIN         */
	64,  /* GCLIMITMAX         */
	10,  /* GCCOUNTMINTGT_MINI */
	0,   /* GCCOUNTMINTGT_MAX  */
	20,  /* MFINVLDDIFFTHR     */
	0,   /* MFINCADCSPXTHR_LS  */
	0,   /* MFINCADCSPXTHR_MS  */
	36,  /* MFERRORTHR_LS      */
	0,   /* MFERRORTHR_MS      */
	5,   /* SELFREQMAX         */
	0,   /* RESERVED_8         */
	0,   /* RESERVED_9         */
	0,   /* RESERVED_10        */
	0,   /* RESERVED_11        */
	0,   /* RESERVED_12        */
	0,   /* RESERVED_13        */
	0,   /* BLEN               */
	75,  /* TCHTHR             */
	3,   /* TCHDI              */
	15,  /* MOVHYSTI           */
	1,   /* MOVHYSTN           */
	47,  /* MOVFILTER          */
	10,  /* NUMTOUCH           */
	20,  /* MRGHYST            */
	50,  /* MRGTHR             */
	0,   /* XLOCLIP            */
	0,   /* XHICLIP            */
	0,   /* YLOCLIP            */
	0,   /* YHICLIP            */
	64,  /* XEDGECTRL          */
	0,   /* XEDGEDIST          */
	64,  /* YEDGECTRL          */
	0,   /* YEDGEDIST          */
	0,   /* JUMPLIMIT          */
	15,  /* TCHHYST            */
	3,   /* NEXTTCHDI          */
};

static struct mxt_config_data mxt_config_t38 =
	{
		.type = 38,
		.config = mxt_config_t38_data,
		.length = sizeof(mxt_config_t38_data),
	};
static struct mxt_config_data mxt_config_t6 =
	{
		.type = 6,
		.config = mxt_config_t6_data,
		.length = sizeof(mxt_config_t6_data),
	};
static struct mxt_config_data mxt_config_t7 =
	{
		.type = 7,
		.config = mxt_config_t7_data,
		.length = sizeof(mxt_config_t7_data),
	};
static struct mxt_config_data mxt_config_t8 =
	{
		.type = 8,
		.config = mxt_config_t8_data,
		.length = sizeof(mxt_config_t8_data),
	};

/*FUJITSU:2012-06-26 Noise WA Add START */
static struct mxt_config_data mxt_config_t8_auto_recover_off =
	{
		.type = 8,
		.config = mxt_config_t8_auto_recover_off_data,
		.length = sizeof(mxt_config_t8_auto_recover_off_data),
	};
/*FUJITSU:2012-06-26 Noise WA Add END */

static struct mxt_config_data mxt_config_t9 =
	{
		.type = 9,
		.config = mxt_config_t9_data,
		.length = sizeof(mxt_config_t9_data),
	};
static struct mxt_config_data mxt_config_t15 =
	{
		.type = 15,
		.config = mxt_config_t15_data,
		.length = sizeof(mxt_config_t15_data),
	};
static struct mxt_config_data mxt_config_t18 =
	{
		.type = 18,
		.config = mxt_config_t18_data,
		.length = sizeof(mxt_config_t18_data),
	};
static struct mxt_config_data mxt_config_t19 =
	{
		.type = 19,
		.config = mxt_config_t19_data,
		.length = sizeof(mxt_config_t19_data),
	};
static struct mxt_config_data mxt_config_t40 =
	{
		.type = 40,
		.config = mxt_config_t40_data,
		.length = sizeof(mxt_config_t40_data),
	};
static struct mxt_config_data mxt_config_t42 =
	{
		.type = 42,
		.config = mxt_config_t42_data,
		.length = sizeof(mxt_config_t42_data),
	};
static struct mxt_config_data mxt_config_t48 =
	{
		.type = 48,
		.config = mxt_config_t48_data,
		.length = sizeof(mxt_config_t48_data),
	};
static struct mxt_config_data mxt_config_t47 =
	{
		.type = 47,
		.config = mxt_config_t47_data,
		.length = sizeof(mxt_config_t47_data),
	};
static struct mxt_config_data mxt_config_t23 =
	{
		.type = 23,
		.config = mxt_config_t23_data,
		.length = sizeof(mxt_config_t23_data),
	};
static struct mxt_config_data mxt_config_t25 =
	{
		.type = 25,
		.config = mxt_config_t25_data,
		.length = sizeof(mxt_config_t25_data),
	};
static struct mxt_config_data mxt_config_t46 =
	{
		.type = 46,
		.config = mxt_config_t46_data,
		.length = sizeof(mxt_config_t46_data),
	};


static u8 mxt_read_chg(void);
static int mxt_get_batt_status(void);
static int mxt_power_on(bool on);
static int mxt_init_hw(bool on);
static int mxt_get_config_rev(void);
static void mxt_gpio_check(void);
static unsigned int mxt_config_rev_crc = MXT_CONFIG_CRC;

#if 1
static struct mxt_platform_data mxt_platform_data = {
	.config_crc      = &mxt_config_rev_crc,
	.config_t6       = &mxt_config_t6,
	.config_t7       = &mxt_config_t7,
	.config_t8       = &mxt_config_t8,
	.config_t9       = &mxt_config_t9,
	.config_t15      = &mxt_config_t15,
	.config_t18      = &mxt_config_t18,
	.config_t19      = &mxt_config_t19,
	.config_t23      = &mxt_config_t23,
	.config_t25      = &mxt_config_t25,
	.config_t40      = &mxt_config_t40,
	.config_t42      = &mxt_config_t42,
	.config_t46      = &mxt_config_t46,
	.config_t47      = &mxt_config_t47,
	.config_t48      = &mxt_config_t48,
	.config_t38      = &mxt_config_t38,
	.config_t8_auto_recover_off= &mxt_config_t8_auto_recover_off,	/*FUJITSU:2012-06-26 Noise WA Add */

	.irqflags        = IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	.read_chg        = &mxt_read_chg,
	.init_hw         = &mxt_init_hw,
	.power_on        = &mxt_power_on,
	.get_batt_status = &mxt_get_batt_status,
	.get_config_rev  = &mxt_get_config_rev,
	.gpio_check      = &mxt_gpio_check,
};

#else
static struct mxt_platform_data mxt_platform_data[] = {
	{
	.config_crc      = &mxt_config_rev_crc,
	.config_t6       = &mxt_config_t6,
	.config_t7       = &mxt_config_t7,
	.config_t8       = &mxt_config_t8,
	.config_t9       = &mxt_config_t9,
	.config_t15      = &mxt_config_t15,
	.config_t18      = &mxt_config_t18,
	.config_t19      = &mxt_config_t19,
	.config_t23      = &mxt_config_t23,
	.config_t25      = &mxt_config_t25,
	.config_t40      = &mxt_config_t40,
	.config_t42      = &mxt_config_t42,
	.config_t46      = &mxt_config_t46,
	.config_t47      = &mxt_config_t47,
	.config_t48      = &mxt_config_t48,
	.config_t38      = &mxt_config_t38,

	.irqflags        = IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	.read_chg        = &mxt_read_chg,
	.init_hw         = &mxt_init_hw,
	.power_on        = &mxt_power_on,
	.get_batt_status = &mxt_get_batt_status,
	.get_config_rev  = &mxt_get_config_rev,
	.gpio_check      = &mxt_gpio_check,
	},
	{
	.config_crc      = &mxt_config_rev_crc,
	.config_t6       = &mxt_config_t6,
	.config_t7       = &mxt_config_t7,
	.config_t8       = &mxt_config_t8,
	.config_t9       = &mxt_config_t9,
	.config_t15      = &mxt_config_t15,
	.config_t18      = &mxt_config_t18,
	.config_t19      = &mxt_config_t19,
	.config_t23      = &mxt_config_t23,
	.config_t25      = &mxt_config_t25,
	.config_t40      = &mxt_config_t40,
	.config_t42      = &mxt_config_t42,
	.config_t46      = &mxt_config_t46,
	.config_t47      = &mxt_config_t47,
	.config_t48      = &mxt_config_t48,
	.config_t38      = &mxt_config_t38,

	.irqflags        = IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	.read_chg        = &mxt_read_chg,
	.init_hw         = &mxt_init_hw,
	.power_on        = &mxt_power_on,
	.get_batt_status = &mxt_get_batt_status,
	.get_config_rev  = &mxt_get_config_rev,
	.gpio_check      = &mxt_gpio_check,
	},
	{
	.config_crc      = &mxt_config_rev_crc,
	.config_t6       = &mxt_config_t6,
	.config_t7       = &mxt_config_t7,
	.config_t8       = &mxt_config_t8,
	.config_t9       = &mxt_config_t9,
	.config_t15      = &mxt_config_t15,
	.config_t18      = &mxt_config_t18,
	.config_t19      = &mxt_config_t19,
	.config_t23      = &mxt_config_t23,
	.config_t25      = &mxt_config_t25,
	.config_t40      = &mxt_config_t40,
	.config_t42      = &mxt_config_t42,
	.config_t46      = &mxt_config_t46,
	.config_t47      = &mxt_config_t47,
	.config_t48      = &mxt_config_t48,
	.config_t38      = &mxt_config_t38,

	.irqflags        = IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	.read_chg        = &mxt_read_chg,
	.init_hw         = &mxt_init_hw,
	.power_on        = &mxt_power_on,
	.get_batt_status = &mxt_get_batt_status,
	.get_config_rev  = &mxt_get_config_rev,
	.gpio_check      = &mxt_gpio_check,
	},
	{
	.config_crc      = &mxt_config_rev_crc,
	.config_t6       = &mxt_config_t6,
	.config_t7       = &mxt_config_t7,
	.config_t8       = &mxt_config_t8,
	.config_t9       = &mxt_config_t9,
	.config_t15      = &mxt_config_t15,
	.config_t18      = &mxt_config_t18,
	.config_t19      = &mxt_config_t19,
	.config_t23      = &mxt_config_t23,
	.config_t25      = &mxt_config_t25,
	.config_t40      = &mxt_config_t40,
	.config_t42      = &mxt_config_t42,
	.config_t46      = &mxt_config_t46,
	.config_t47      = &mxt_config_t47,
	.config_t48      = &mxt_config_t48,
	.config_t38      = &mxt_config_t38,

	.irqflags        = IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	.read_chg        = &mxt_read_chg,
	.init_hw         = &mxt_init_hw,
	.power_on        = &mxt_power_on,
	.get_batt_status = &mxt_get_batt_status,
	.get_config_rev  = &mxt_get_config_rev,
	.gpio_check      = &mxt_gpio_check,
	},
};

#endif
static struct i2c_board_info mxt_info[] __initdata = {
	{
		I2C_BOARD_INFO(MXT_DEVICE_ID, MXT_I2C_SLAVE_ADDRESS),
#if 1
		.platform_data = &mxt_platform_data,
#else
		.platform_data = mxt_platform_data,
#endif
		.irq = MSM_GPIO_TO_INT(MXT_GPIO_TCH_INT),
	},
};


static int mxt_power_on(bool on)
{
	int ret;
	struct  i2c_msg msg;
	struct  i2c_adapter *i2c_bkl;
	u_int8_t buf[8];

	/* Get adpter */
	i2c_bkl = i2c_get_adapter(0);

	/* Set Param    */
	msg.addr    = LED_I2C_SLAVE_ADDR_IN_TOUCH_FJ;
	msg.buf     = buf;
	msg.len     = 2;
	msg.flags   = 0;

	if (on) {
		printk("%s: Power ON\n", __func__);
/* FUJITSU:2012-03-09 touchscreen start */
		/* Set Voltage 15H(0x94:2.7V+1.8V) */
		buf[0] = 0x15;  /* client addr     */
//		buf[1] = 0x94;  /* LDO4 2.7V LDO3 1.8V */
		buf[1] = 0xF4;  /* LDO4 3.3V LDO3 1.8V */
/* FUJITSU:2012-03-09 taouchscreen end */
		ret = i2c_transfer(i2c_bkl, &msg, 1);
		if (ret < 0) {
		    printk("%s: I2C ERROR ret = %d\n", __func__, ret);
		    return ret;
		}

		mdelay(10);
		   
		/* Set Param    */
		/* Voltage ON  13H(0x0F)    */
		buf[0] = 0x13;  /* client addr     */
		buf[1] = 0x0B;  /* 13H: LDO1-4 ON  */  

		printk("%s: Power ON 2.8V\n", __func__);
		ret = i2c_transfer(i2c_bkl, &msg, 1);
		if (ret < 0) {
		    printk("%s: I2C ERROR ret = %d\n", __func__, ret);
		    return ret;
		}

		mdelay(10);

		buf[0] = 0x13;  /* client addr     */
		buf[1] = 0x0F;  /* 13H: LDO1-4 ON  */  

		printk("%s: Power ON 1.8V\n", __func__);
		ret = i2c_transfer(i2c_bkl, &msg, 1);
		if (ret < 0) {
		    printk("%s: I2C ERROR ret = %d\n", __func__, ret);
		    return ret;
		}

		mdelay(40);
	} else {
		printk("%s: No Power OFF\n", __func__);
	}

	return 0;
}

static int mxt_init_hw(bool on)
{
    int rc = 0;

	printk( KERN_INFO "%s :[IN]\n", __func__ );

	if (on) {
		printk( KERN_INFO "%s : INIT HW ON\n", __func__ );
		/* request XRST */
		rc = gpio_request(MXT_GPIO_TCH_XRST, "gpio_tcp_xrst" );
		if (rc) {
			pr_err("gpio_request failed on RESET(%d) (rc=%d)\n", MXT_GPIO_TCH_XRST, rc);
		}

		/* config XRST */
		rc = gpio_tlmm_config(GPIO_CFG(MXT_GPIO_TCH_XRST, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(MXT_GPIO_TCH_XRST, 1);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(XRST)=%d\n", __func__, rc );
		}


		/* request INT */
		rc = gpio_request(MXT_GPIO_TCH_INT, "gpio_tcp_int" );
		if (rc) {
			pr_err("gpio_request failed on RESET(%d) (rc=%d)\n", MXT_GPIO_TCH_INT, rc);
		}

		/* config INT */
		rc = gpio_tlmm_config(GPIO_CFG(MXT_GPIO_TCH_INT, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(INT)=%d\n", __func__, rc );
		}
	} else {
		printk( KERN_INFO "%s : INIT HW OFF\n", __func__ );
	}

	return rc;
}

static u8 mxt_read_chg(void)
{
//	printk( KERN_INFO "%s :[IN]\n", __func__ );
	return gpio_get_value(MXT_GPIO_TCH_INT);
}

static int mxt_get_batt_status(void)
{
//	union power_supply_propval value;
	printk( KERN_INFO "%s :[IN] return 0\n", __func__ );
//	msm_battery_get_property(POWER_SUPPLY_PROP_STATUS, &value);
//	return value.intval;
	return 0;
}

static int mxt_get_config_rev(void)
{
	int retval = 0;
/*FUJITSU:2012/03/30 Multi Vender Mod Start */
	retval = gpio_get_value(MXT_GPIO_LCD_TYPE);
	if(retval == 0) {
		printk( KERN_INFO "%s :LCD Type = S\n", __func__ );
	} else if(retval == 1) {
		printk( KERN_INFO "%s :LCD Type = H\n", __func__ );
	} else {
		printk( KERN_ERR "%s :Unknown LCD Type\n", __func__ );
		retval = -1;
	}
/*FUJITSU:2012/03/30 Multi Vender Mod End */
	return retval;
}

static void mxt_gpio_check(void)
{
	printk( KERN_INFO "%s :[IN]\n", __func__ );
}

#endif //_BOARD_TOUCH_FJ_C_
