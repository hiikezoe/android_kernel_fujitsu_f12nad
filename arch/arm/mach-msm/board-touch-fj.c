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
#include <linux/i2c/atmel_mxt_ts_apo.h>

#include <linux/power_supply.h>

extern int msm_battery_get_property(enum power_supply_property psp,
										union power_supply_propval *val);


#define MXT_CONFIG_CRC         0x2B555F
#define MXT_I2C_SLAVE_ADDRESS  0x4A
#define MXT_GPIO_TCH_INT       142

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
	0,   /* DATA_0 */
	0,   /* DATA_1 */
	0,   /* DATA_2 */
	0,   /* DATA_3 */
	0,   /* DATA_4 */
	0,   /* DATA_5 */
	0,   /* DATA_6 */
	0,   /* DATA_7 */
};
/* GEN_POWERCONFIG_T7 */
static u8 mxt_config_t7_data[]  = {
	16,  /* IDLEACQINT  */
	17,  /* ACTVACQINT  */
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
	3,   /* ATCHFRCCALTHR   */
	25,  /* ATCHFRCCALRATIO */
};
/* TOUCH_MULTITOUCHSCREEN_T9 */
static u8 mxt_config_t9_data[]  = {
	131, /* CTRL       */
	0,   /* XORIGIN    */
	0,   /* YORIGIN    */
	19,  /* XSIZE      */
	11,  /* YSIZE      */
	0,   /* AKSCFG     */
	32,  /* BLEN       */
	71,  /* TCHTHR     */
	1,   /* TCHDI      */
	1,   /* ORIENT     */
	10,  /* MRGTIMEOUT */
	3,   /* MOVHYSTI   */
	1,   /* MOVHYSTN   */
	46,  /* MOVFILTER  */
	5,   /* NUMTOUCH   */
	20,  /* MRGHYST    */
	50,  /* MRGTHR     */
	5,   /* AMPHYST    */
	31, /* XRANGE_LO  */
	3,   /* XRANGE_HI  */
	223,  /* YRANGE_LO  */
	1,   /* YRANGE_HI  */
	1,   /* XLOCLIP    */
	1,   /* XHICLIP    */
	3,   /* YLOCLIP    */
	3,   /* YHICLIP    */
	226, /* XEDGECTRL  */
	45,  /* XEDGEDIST  */
	223, /* YEDGECTRL  */
	80,  /* YEDGEDIST  */
	32,  /* JUMPLIMIT  */
	15,  /* TCHHYST    */
	0,   /* XPITCH     */
	0,   /* YPITCH     */
	0,   /* NEXTTCHDI  */
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
	0,   /* CTRL          */
	0,   /* CMD           */
	0,   /* HISIGLIM_0_LS */
	0,   /* HISIGLIM_0_MS */
	0,   /* LOSIGLIM_0_LS */
	0,   /* LOSIGLIM_0_MS */
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
	3,   /* CTRL          */
	26,  /* APPRTHR       */
	58,  /* MAXAPPRAREA   */
	96,  /* MAXTCHAREA    */
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
	64,  /* CALCFG             */
	8,   /* BASEFREQ           */
	0,   /* RESERVED_0         */
	0,   /* RESERVED_1         */
	0,   /* RESERVED_2         */
	0,   /* RESERVED_3         */
	10,  /* MFFREQ_0           */
	15,  /* MFFREQ_1           */
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
	5,   /* MFINCADCSPXTHR_LS  */
	0,   /* MFINCADCSPXTHR_MS  */
	25,  /* MFERRORTHR_LS      */
	0,   /* MFERRORTHR_MS      */
	5,   /* SELFREQMAX         */
	0,   /* RESERVED_8         */
	0,   /* RESERVED_9         */
	0,   /* RESERVED_10        */
	0,   /* RESERVED_11        */
	0,   /* RESERVED_12        */
	0,   /* RESERVED_13        */
	32,  /* BLEN               */
	55,  /* TCHTHR             */
	2,   /* TCHDI              */
	5,   /* MOVHYSTI           */
	2,   /* MOVHYSTN           */
	1,   /* MOVFILTER          */
	5,   /* NUMTOUCH           */
	20,  /* MRGHYST            */
	50,  /* MRGTHR             */
	0,   /* XLOCLIP            */
	0,   /* XHICLIP            */
	10,  /* YLOCLIP            */
	10,  /* YHICLIP            */
	136, /* XEDGECTRL          */
	0,   /* XEDGEDIST          */
	136, /* YEDGECTRL          */
	0,   /* YEDGEDIST          */
	25,  /* JUMPLIMIT          */
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

	.irqflags        = IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	.read_chg        = &mxt_read_chg,
	.init_hw         = &mxt_init_hw,
	.power_on        = &mxt_power_on,
	.get_batt_status = &mxt_get_batt_status,
	.get_config_rev  = &mxt_get_config_rev,
	.gpio_check      = &mxt_gpio_check,
};

static struct i2c_board_info mxt_info[] __initdata = {
	{
		I2C_BOARD_INFO(MXT_DEVICE_ID, MXT_I2C_SLAVE_ADDRESS),
		.platform_data = &mxt_platform_data,
		.irq = MSM_GPIO_TO_INT(MXT_GPIO_TCH_INT),
	}
};

static struct vreg *cap_vreg_l8;
static struct vreg *cap_vreg_l10;

static int mxt_power_on(bool on)
{
    int     retval;

    /* for New model (use PM8058) */
    if ( on ){
        /* get structure */
        cap_vreg_l8 = vreg_get( NULL, "gp7" );
        if ( IS_ERR(cap_vreg_l8) ){
            printk( KERN_ERR "%s: get vreg structure error retval = %d\n", __func__, (int)cap_vreg_l8 );
            return -1;
        }

        /* set voltage */
        retval = vreg_set_level( cap_vreg_l8, 1800 );
        if ( retval ) {
            printk( KERN_ERR "%s: set voltage error retval = %d\n", __func__, retval );
            return -1;
        }

        /* output enable */
        retval = vreg_enable( cap_vreg_l8 );
        if ( retval ) {
            printk( KERN_ERR "%s: output enable error retval = %d\n", __func__, retval );
            return -1;
        }

        cap_vreg_l10 = vreg_get( NULL, "gp4" );
        if ( IS_ERR(cap_vreg_l10) ){
            printk( KERN_ERR "%s: get vreg structure error retval = %d\n", __func__, (int)cap_vreg_l10 );
            return -1;
        }

        /* set voltage */
        retval = vreg_set_level( cap_vreg_l10, 2900 );
        if ( retval ) {
            printk( KERN_ERR "%s: set voltage error retval = %d\n", __func__, retval );
            return -1;
        }

        /* output enable */
        retval = vreg_enable( cap_vreg_l10 );
        if ( retval ) {
            printk( KERN_ERR "%s: output enable error retval = %d\n", __func__, retval );
            return -1;
        }

        msleep( 40 );
    }else{
        /* check structure */
        if ( IS_ERR(cap_vreg_l8) ){
            printk( KERN_ERR "%s: invalid vreg structure \n", __func__ );
            return 0;
        }

        /* output disable */
        retval = vreg_disable( cap_vreg_l8 );
        if ( retval ) {
            printk( KERN_ERR "%s: output disable error retval = %d\n", __func__, retval );
            return retval;
        }

        vreg_put( cap_vreg_l8 );
        cap_vreg_l8 = NULL;

        /* check structure */
        if ( IS_ERR(cap_vreg_l10) ){
            printk( KERN_ERR "%s: invalid vreg structure \n", __func__ );
            return 0;
        }

        /* output disable */
        retval = vreg_disable( cap_vreg_l10 );
        if ( retval ) {
            printk( KERN_ERR "%s: output disable error retval = %d\n", __func__, retval );
            return retval;
        }

        vreg_put( cap_vreg_l10 );
        cap_vreg_l10 = NULL;
        return 0;

	}
	return 0;
}

static int mxt_init_hw(bool on)
{
    int rc = 0;

	printk( KERN_INFO "%s :[IN]\n", __func__ );

	if (on) {
		printk( KERN_INFO "%s : INIT HW ON\n", __func__ );
		/* request */
		rc = gpio_request(MXT_GPIO_TCH_INT, "gpio_ts_int" );
		if (rc) {
			pr_err("gpio_request failed on RESET(%d) (rc=%d)\n", MXT_GPIO_TCH_INT, rc);
		}

		/* config */
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
	union power_supply_propval value;
//	printk( KERN_INFO "%s :[IN]\n", __func__ );
	msm_battery_get_property(POWER_SUPPLY_PROP_STATUS, &value);
	return value.intval;
}

static int mxt_get_config_rev(void)
{
	return 0;
}

static void mxt_gpio_check(void)
{
	printk( KERN_INFO "%s :[IN]\n", __func__ );
}

#endif //_BOARD_TOUCH_FJ_C_
