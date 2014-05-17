/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
/* FUJITSU:2012-02-22 mod start */
//#include <linux/i2c/atmel_mxt_ts.h>
//#include <linux/input.h>
#include <linux/i2c/atmel_mxt_ts_apo.h>
#include <linux/input/mt.h>
/* FUJITSU:2012-02-22 mod end */
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
#else
#include "fj_touch_if.h"
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/time.h>
#include <asm/uaccess.h>
#include <linux/power_supply.h>
#if defined(CONFIG_TOUCHSCREEN_F12)
#include <linux/max17040_battery.h>
#endif
#if defined(CONFIG_TOUCHSCREEN_F12_APO) || defined(CONFIG_TOUCHSCREEN_F12_NAD)
#include <linux/msm_battery_cb.h>
#endif

static char *mxt_batt_power_str[] = {
	"UNKNOWN",
	"CHARGING",
	"DISCHARGING",
	"NOT_CHARGING",
	"FULL"
};
#endif
/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION end */

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define MXT_SUSPEND_LEVEL 1
#endif

/* Family ID */
#define MXT224_ID	0x80
#define MXT1386_ID	0xA0

/* Version */
#define MXT_VER_20		20
#define MXT_VER_21		21
#define MXT_VER_22		22

/* Slave addresses */
#define MXT_APP_LOW		0x4a
#define MXT_APP_HIGH		0x4b
#define MXT_BOOT_LOW		0x24
#define MXT_BOOT_HIGH		0x25

/* Firmware */
#define MXT_FW_NAME		"maxtouch.fw"

/* Registers */
#define MXT_FAMILY_ID		0x00
#define MXT_VARIANT_ID		0x01
#define MXT_VERSION		0x02
#define MXT_BUILD		0x03
#define MXT_MATRIX_X_SIZE	0x04
#define MXT_MATRIX_Y_SIZE	0x05
#define MXT_OBJECT_NUM		0x06
#define MXT_OBJECT_START	0x07

#define MXT_OBJECT_SIZE		6

/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
/* Object types */
#define MXT_DEBUG_DIAGNOSTIC	37
#define MXT_GEN_MESSAGE		5
#define MXT_GEN_COMMAND		6
#define MXT_GEN_POWER		7
#define MXT_GEN_ACQUIRE		8
#define MXT_TOUCH_MULTI		9
#define MXT_TOUCH_KEYARRAY	15
#define MXT_TOUCH_PROXIMITY	23
#define MXT_PROCI_GRIPFACE	20
#define MXT_PROCG_NOISE		22
#define MXT_PROCI_ONETOUCH	24
#define MXT_PROCI_TWOTOUCH	27
#define MXT_PROCI_GRIP		40
#define MXT_PROCI_PALM		41
#define MXT_SPT_COMMSCONFIG	18
#define MXT_SPT_GPIOPWM		19
#define MXT_SPT_SELFTEST	25
#define MXT_SPT_CTECONFIG	28
#define MXT_SPT_USERDATA	38
#define MXT_SPT_DIGITIZER	43
#define MXT_SPT_MESSAGECOUNT	44
#else
/* Object types */
#define MXT_DEBUG_DIAGNOSTIC	37
#define MXT_GEN_MESSAGE		5
#define MXT_GEN_COMMAND		6
#define MXT_GEN_POWER		7
#define MXT_GEN_ACQUIRE		8
#define MXT_TOUCH_MULTI		9
#define MXT_TOUCH_KEYARRAY	15
#define MXT_TOUCH_PROXIMITY	23
#define MXT_PROCI_GRIPFACE	20
#define MXT_PROCG_NOISE		22
#define MXT_PROCI_ONETOUCH	24
#define MXT_PROCI_TWOTOUCH	27
#define MXT_PROCI_GRIP		40
#define MXT_PROCI_PALM		41
#define MXT_PROCI_TOUCHSUPPRESSION	42
#define MXT_PROCI_STYLUS		47
#define MXT_PROCG_NOISESUPPRESSION	48
#define MXT_SPT_COMMSCONFIG	18
#define MXT_SPT_GPIOPWM		19
#define MXT_SPT_SELFTEST	25
#define MXT_SPT_CTECONFIG	46
#define MXT_SPT_USERDATA	38
#define MXT_SPT_DIGITIZER	43
#define MXT_SPT_MESSAGECOUNT	44
#endif
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* MXT_GEN_COMMAND field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* MXT_GEN_POWER field */
#define MXT_POWER_IDLEACQINT	0
#define MXT_POWER_ACTVACQINT	1
#define MXT_POWER_ACTV2IDLETO	2

/* MXT_GEN_ACQUIRE field */
#define MXT_ACQUIRE_CHRGTIME	0
#define MXT_ACQUIRE_TCHDRIFT	2
#define MXT_ACQUIRE_DRIFTST	3
#define MXT_ACQUIRE_TCHAUTOCAL	4
#define MXT_ACQUIRE_SYNC	5
#define MXT_ACQUIRE_ATCHCALST	6
#define MXT_ACQUIRE_ATCHCALSTHR	7

/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
/* MXT_TOUCH_MULTI field */
#define MXT_TOUCH_CTRL		0
#define MXT_TOUCH_XORIGIN	1
#define MXT_TOUCH_YORIGIN	2
#define MXT_TOUCH_XSIZE		3
#define MXT_TOUCH_YSIZE		4
#define MXT_TOUCH_BLEN		6
#define MXT_TOUCH_TCHTHR	7
#define MXT_TOUCH_TCHDI		8
#define MXT_TOUCH_ORIENT	9
#define MXT_TOUCH_MOVHYSTI	11
#define MXT_TOUCH_MOVHYSTN	12
#define MXT_TOUCH_NUMTOUCH	14
#define MXT_TOUCH_MRGHYST	15
#define MXT_TOUCH_MRGTHR	16
#define MXT_TOUCH_AMPHYST	17
#define MXT_TOUCH_XRANGE_LSB	18
#define MXT_TOUCH_XRANGE_MSB	19
#define MXT_TOUCH_YRANGE_LSB	20
#define MXT_TOUCH_YRANGE_MSB	21
#define MXT_TOUCH_XLOCLIP	22
#define MXT_TOUCH_XHICLIP	23
#define MXT_TOUCH_YLOCLIP	24
#define MXT_TOUCH_YHICLIP	25
#define MXT_TOUCH_XEDGECTRL	26
#define MXT_TOUCH_XEDGEDIST	27
#define MXT_TOUCH_YEDGECTRL	28
#define MXT_TOUCH_YEDGEDIST	29
#define MXT_TOUCH_JUMPLIMIT	30
#else //MXT_ENABLE_ORIGINAL_FUNCTION
/* MXT_TOUCH_MULTI field */
#define MXT_TOUCH_CTRL		0
#define MXT_TOUCH_XORIGIN	1
#define MXT_TOUCH_YORIGIN	2
#define MXT_TOUCH_XSIZE		3
#define MXT_TOUCH_YSIZE		4
#define MXT_TOUCH_BLEN		6
#define MXT_TOUCH_TCHTHR	7
#define MXT_TOUCH_TCHDI		8
#define MXT_TOUCH_ORIENT	9
#define MXT_TOUCH_MOVHYSTI	11
#define MXT_TOUCH_MOVHYSTN	12
#define MXT_TOUCH_NUMTOUCH	14
#define MXT_TOUCH_MRGHYST	15
#define MXT_TOUCH_MRGTHR	16
#define MXT_TOUCH_AMPHYST	17
#define MXT_TOUCH_XRANGE_LSB	18
#define MXT_TOUCH_XRANGE_MSB	19
#define MXT_TOUCH_YRANGE_LSB	20
#define MXT_TOUCH_YRANGE_MSB	21
#define MXT_TOUCH_XLOCLIP	22
#define MXT_TOUCH_XHICLIP	23
#define MXT_TOUCH_YLOCLIP	24
#define MXT_TOUCH_YHICLIP	25
#define MXT_TOUCH_XEDGECTRL	26
#define MXT_TOUCH_XEDGEDIST	27
#define MXT_TOUCH_YEDGECTRL	28
#define MXT_TOUCH_YEDGEDIST	29
#define MXT_TOUCH_JUMPLIMIT	30
#define MXT_TOUCH_TCHHYST	31
#define MXT_TOUCH_XPITCH	32
#define MXT_TOUCH_YPITCH	33
#define MXT_TOUCH_NEXTTCHDI	34
#endif
/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* MXT_PROCI_GRIPFACE field */
#define MXT_GRIPFACE_CTRL	0
#define MXT_GRIPFACE_XLOGRIP	1
#define MXT_GRIPFACE_XHIGRIP	2
#define MXT_GRIPFACE_YLOGRIP	3
#define MXT_GRIPFACE_YHIGRIP	4
#define MXT_GRIPFACE_MAXTCHS	5
#define MXT_GRIPFACE_SZTHR1	7
#define MXT_GRIPFACE_SZTHR2	8
#define MXT_GRIPFACE_SHPTHR1	9
#define MXT_GRIPFACE_SHPTHR2	10
#define MXT_GRIPFACE_SUPEXTTO	11

/* MXT_PROCI_NOISE field */
#define MXT_NOISE_CTRL		0
#define MXT_NOISE_OUTFLEN	1
#define MXT_NOISE_GCAFUL_LSB	3
#define MXT_NOISE_GCAFUL_MSB	4
#define MXT_NOISE_GCAFLL_LSB	5
#define MXT_NOISE_GCAFLL_MSB	6
#define MXT_NOISE_ACTVGCAFVALID	7
#define MXT_NOISE_NOISETHR	8
#define MXT_NOISE_FREQHOPSCALE	10
#define MXT_NOISE_FREQ0		11
#define MXT_NOISE_FREQ1		12
#define MXT_NOISE_FREQ2		13
#define MXT_NOISE_FREQ3		14
#define MXT_NOISE_FREQ4		15
#define MXT_NOISE_IDLEGCAFVALID	16

/* MXT_SPT_COMMSCONFIG */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1

/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
/* MXT_SPT_CTECONFIG field */
#define MXT_CTE_CTRL		0
#define MXT_CTE_CMD		1
#define MXT_CTE_MODE		2
#define MXT_CTE_IDLEGCAFDEPTH	3
#define MXT_CTE_ACTVGCAFDEPTH	4
#define MXT_CTE_VOLTAGE		5
#else
/* MXT_SPT_CTECONFIG field */
#define MXT_CTE_CTRL		0
#define MXT_CTE_MODE		1
#define MXT_CTE_IDLESYNCSPERX	2
#define MXT_CTE_ACTVSYNCSPERX	3
#endif
/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION end */

#define MXT_VOLTAGE_DEFAULT	2700000
#define MXT_VOLTAGE_STEP	10000

#define MXT_VTG_MIN_UV		2700000
#define MXT_VTG_MAX_UV		3300000
#define MXT_ACTIVE_LOAD_UA	15000
#define MXT_LPM_LOAD_UA		10

#define MXT_I2C_VTG_MIN_UV	1800000
#define MXT_I2C_VTG_MAX_UV	1800000
#define MXT_I2C_LOAD_UA		10000
#define MXT_I2C_LPM_LOAD_UA	10

/* Define for MXT_GEN_COMMAND */
#define MXT_BOOT_VALUE		0xa5
#define MXT_BACKUP_VALUE	0x55
#define MXT_BACKUP_TIME		25	/* msec */
#define MXT224_RESET_TIME		65	/* msec */
#define MXT1386_RESET_TIME		250	/* msec */
#define MXT_RESET_TIME		250	/* msec */
#define MXT_RESET_NOCHGREAD		400	/* msec */

#define MXT_FWRESET_TIME	175	/* msec */

#define MXT_WAKE_TIME		25

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f

/* Touch status */
#define MXT_SUPPRESS		(1 << 1)
#define MXT_AMP			(1 << 2)
#define MXT_VECTOR		(1 << 3)
#define MXT_MOVE		(1 << 4)
#define MXT_RELEASE		(1 << 5)
#define MXT_PRESS		(1 << 6)
#define MXT_DETECT		(1 << 7)

/* Touch orient bits */
#define MXT_XY_SWITCH		(1 << 0)
#define MXT_X_INVERT		(1 << 1)
#define MXT_Y_INVERT		(1 << 2)

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_MAX_FINGER		10

#define T7_DATA_SIZE		3
#define MXT_MAX_RW_TRIES	3
#define MXT_BLOCK_SIZE		256

/* FUJITSU:2012-04-09 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
#else
/* MXT_PROCG_NOISESUPPRESSION field */
#define MXT_NOISESUPPRESSION_CALCFG		0x02
#define MXT_NOISESUPPRESSION_MFFREQ_0	0x08
#define MXT_NOISESUPPRESSION_MFFREQ_1	0x09
#define MXT_NOISESUPPRESSION_SELFREQMAX	0x1B

/* MXT_COMMAND_DIAGNOSTIC field */
#define MXT_PAGE_UP                  0x01
#define MXT_PAGE_DOWN                0x02
#define MXT_DELTAS_MODE              0x10
#define MXT_REFERENCES_MODE          0x11

#define MXT_AVDD_TEST                0x01
#define MXT_PIN_FAULT_TEST           0x11
#define MXT_SIGNAL_LIMIT_TEST        0x17
#define MXT_ALL_THE_TEST             0xFE

#define MXT_ACNOISE_CHECK_THRESHOLD  176
#define MXT_WATCHDOG_TIME            msecs_to_jiffies(3000)
#define MXT_FIRMVERSION_SIZE         16
#define MXT_DEBUG_LOG_LEVEL1         1
#define MXT_DEBUG_LOG_LEVEL2         2
#define MXT_DEBUG_LOG_LEVEL3         3
#define MXT_COEFFICIENT              24
#define MXT_MAX_Z_VALUE              255
#define MXT_RESET_NOCHGREAD          400 /* msec */
#define MXT_MAX_INIT_WAIT            500 /* msec */
#define MXT_CONFIG_REV_0             0
#define MXT_CONFIG_REV_1             1

#define T25_DATA_SIZE                7
#define T37_DATA_SIZE                130
#define T40_DATA_SIZE                5

static int set_log_level = 0;
#define mxt_dbg_log(level, fmt, arg...) {\
	if(set_log_level >= level) \
	pr_info(fmt, ## arg);\
}
#endif
/* FUJITSU:2012-04-09 MXT_ENABLE_ORIGINAL_FUNCTION end */

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 max_reportid;
};

struct mxt_message {
	u8 reportid;
	u8 message[7];
	u8 checksum;
};

/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
struct mxt_finger {
	int status;
	int x;
	int y;
	int area;
};
#else
struct mxt_finger {
	int status;
	int x;
	int y;
	int area;
	int z;
	u8 component;
};
#endif
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info info;
	struct mxt_finger finger[MXT_MAX_FINGER];
	unsigned int irq;
	struct regulator *vcc;
	struct regulator *vcc_i2c;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

	u8 t7_data[T7_DATA_SIZE];
	u16 t7_start_addr;
	u8 t9_ctrl;
};
#else
/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info info;
	struct mxt_finger finger[MXT_MAX_FINGER];
	unsigned int irq;
	struct regulator *vcc;
	struct regulator *vcc_i2c;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct cdev mxt_touch_cdev;
	struct mutex data_lock;
	struct msm_battery_callback_info battery_callback;
	bool mxt_chrgon_flag;
	bool mxt_suspend_flag;
	bool mxt_touchevent_flag;
	int mxt_irq_cnt;
	u8 diag_mode;
	unsigned int max_x;
	unsigned int max_y;
	unsigned int mxt_crc;
	unsigned int mxt_batt_status;

	u8 t7_data[T7_DATA_SIZE];
	u16 t7_start_addr;
	u8 t9_ctrl;
};

static int cdev_major = 0;
static struct class* udev_class;
#endif
/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_MESSAGE:
	case MXT_GEN_COMMAND:
	case MXT_GEN_POWER:
	case MXT_GEN_ACQUIRE:
	case MXT_TOUCH_MULTI:
	case MXT_TOUCH_KEYARRAY:
	case MXT_TOUCH_PROXIMITY:
	case MXT_PROCI_GRIPFACE:
	case MXT_PROCG_NOISE:
	case MXT_PROCI_ONETOUCH:
	case MXT_PROCI_TWOTOUCH:
	case MXT_PROCI_GRIP:
	case MXT_PROCI_PALM:
	case MXT_SPT_COMMSCONFIG:
	case MXT_SPT_GPIOPWM:
	case MXT_SPT_SELFTEST:
	case MXT_SPT_CTECONFIG:
	case MXT_SPT_USERDATA:
	case MXT_SPT_DIGITIZER:
		return true;
	default:
		return false;
	}
}
#else
static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_MESSAGE:
	case MXT_GEN_COMMAND:
	case MXT_GEN_POWER:
	case MXT_GEN_ACQUIRE:
	case MXT_TOUCH_MULTI:
	case MXT_TOUCH_KEYARRAY:
	case MXT_TOUCH_PROXIMITY:
	case MXT_PROCI_GRIPFACE:
	case MXT_PROCG_NOISE:
	case MXT_PROCI_ONETOUCH:
	case MXT_PROCI_TWOTOUCH:
	case MXT_PROCI_GRIP:
	case MXT_PROCI_PALM:
	case MXT_PROCI_TOUCHSUPPRESSION:
	case MXT_PROCI_STYLUS:
	case MXT_PROCG_NOISESUPPRESSION:
	case MXT_SPT_COMMSCONFIG:
	case MXT_SPT_GPIOPWM:
	case MXT_SPT_SELFTEST:
	case MXT_SPT_CTECONFIG:
	case MXT_SPT_USERDATA:
	case MXT_SPT_DIGITIZER:
	case MXT_SPT_MESSAGECOUNT:
		return true;
	default:
		return false;
	}
}
#endif
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static bool mxt_object_writable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND:
	case MXT_GEN_POWER:
	case MXT_GEN_ACQUIRE:
	case MXT_TOUCH_MULTI:
	case MXT_TOUCH_KEYARRAY:
	case MXT_TOUCH_PROXIMITY:
	case MXT_PROCI_GRIPFACE:
	case MXT_PROCG_NOISE:
	case MXT_PROCI_ONETOUCH:
	case MXT_PROCI_TWOTOUCH:
	case MXT_PROCI_GRIP:
	case MXT_PROCI_PALM:
	case MXT_SPT_GPIOPWM:
	case MXT_SPT_SELFTEST:
	case MXT_SPT_CTECONFIG:
	case MXT_SPT_USERDATA:
	case MXT_SPT_DIGITIZER:
		return true;
	default:
		return false;
	}
}
#endif
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static void mxt_dump_message(struct device *dev,
				  struct mxt_message *message)
{
	dev_dbg(dev, "reportid:\t0x%x\n", message->reportid);
	dev_dbg(dev, "message1:\t0x%x\n", message->message[0]);
	dev_dbg(dev, "message2:\t0x%x\n", message->message[1]);
	dev_dbg(dev, "message3:\t0x%x\n", message->message[2]);
	dev_dbg(dev, "message4:\t0x%x\n", message->message[3]);
	dev_dbg(dev, "message5:\t0x%x\n", message->message[4]);
	dev_dbg(dev, "message6:\t0x%x\n", message->message[5]);
	dev_dbg(dev, "message7:\t0x%x\n", message->message[6]);
	dev_dbg(dev, "checksum:\t0x%x\n", message->checksum);
}
#else
static void mxt_dump_message(struct device *dev,
				  struct mxt_message *message)
{
	mxt_dbg_log(MXT_DEBUG_LOG_LEVEL1,
		"[TPD_Rw] RID:0x%02X,M1:0x%02X,M2:0x%02X,M3:0x%02X,"
		"M4:0x%02X,M5:0x%02X,M6:0x%02X,M7:0x%02X,CHK:0x%02X\n",
		message->reportid, message->message[0], message->message[1],
		message->message[2], message->message[3], message->message[4],
		message->message[5], message->message[6], message->checksum);
}
#endif
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION end */

static int mxt_check_bootloader(struct i2c_client *client,
				     unsigned int state)
{
	u8 val;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		return -EINVAL;
	}

	return 0;
}

static int mxt_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = MXT_UNLOCK_CMD_LSB;
	buf[1] = MXT_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_fw_write(struct i2c_client *client,
			     const u8 *data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int i = 0;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	do {
		if (i2c_transfer(client->adapter, xfer, 2) == 2)
			return 0;
		msleep(MXT_WAKE_TIME);
	} while (++i < MXT_MAX_RW_TRIES);

	dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
	return -EIO;
}

static int mxt_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return __mxt_read_reg(client, reg, 1, val);
}

static int __mxt_write_reg(struct i2c_client *client,
		    u16 addr, u16 length, u8 *value)
{
	u8 buf[MXT_BLOCK_SIZE + 2];
	int i, tries = 0;

	if (length > MXT_BLOCK_SIZE)
		return -EINVAL;

	buf[0] = addr & 0xff;
	buf[1] = (addr >> 8) & 0xff;
	for (i = 0; i < length; i++)
		buf[i + 2] = *value++;

	do {
		if (i2c_master_send(client, buf, length + 2) == (length + 2))
			return 0;
		msleep(MXT_WAKE_TIME);
	} while (++tries < MXT_MAX_RW_TRIES);

	dev_err(&client->dev, "%s: i2c send failed\n", __func__);
	return -EIO;
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

static int mxt_read_object_table(struct i2c_client *client,
				      u16 reg, u8 *object_buf)
{
	return __mxt_read_reg(client, reg, MXT_OBJECT_SIZE,
				   object_buf);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type\n");
	return NULL;
}

static int mxt_read_message(struct mxt_data *data,
				 struct mxt_message *message)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, MXT_GEN_MESSAGE);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg,
			sizeof(struct mxt_message), message);
}

static int mxt_read_object(struct mxt_data *data,
				u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg + offset, 1, val);
}

static int mxt_write_object(struct mxt_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return mxt_write_reg(data->client, reg + offset, val);
}

/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
#else
static int mxt_multi_write_object(struct mxt_data *data,
				 u8 type, void *buf, int len)
{
	struct mxt_object *object = NULL;
	u16 reg = 0x00;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;

	if (len > (object->size + 1) * (object->instances + 1)) {
		printk("%s: SizeErr type:%d MaxSize:%d WriteSize:%d\n",
			__func__, type, (object->size + 1) * (object->instances + 1), len);
		return -EINVAL;
	}

	return __mxt_write_reg(data->client, reg, len, buf);
}

static u8 mxt_reportid_to_type(struct mxt_data *data, u8 reportid )
{
	struct mxt_object *object = NULL;
	int i = 0;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->max_reportid >= reportid) {
			return object->type;
		}
	}

	return 0;
}

#if defined(CONFIG_TOUCHSCREEN_F12)
static void mxt_set_noisesuppression(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	int retval = 0;

	if (data->mxt_chrgon_flag) {
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD]%s: Charger ON\n", __func__);
		/* MXT_TOUCH_MULTI field */
		retval = mxt_write_object(data, MXT_TOUCH_MULTI,
			MXT_TOUCH_TCHDI, 0x03);
		retval = mxt_write_object(data, MXT_TOUCH_MULTI,
			MXT_TOUCH_MRGTHR, 0xF0);
		retval = mxt_write_object(data, MXT_TOUCH_MULTI,
			MXT_TOUCH_NEXTTCHDI, 0x03);

		/* MXT_SPT_CTECONFIG field */
		retval = mxt_write_object(data, MXT_SPT_CTECONFIG,
			MXT_CTE_IDLESYNCSPERX, 0x10);
		retval = mxt_write_object(data, MXT_SPT_CTECONFIG,
			MXT_CTE_ACTVSYNCSPERX, 0x20);

		/* MXT_PROCG_NOISESUPPRESSION field */
		retval = mxt_write_object(data, MXT_PROCG_NOISESUPPRESSION,
			MXT_NOISESUPPRESSION_MFFREQ_0, 0x13);
		retval = mxt_write_object(data, MXT_PROCG_NOISESUPPRESSION,
			MXT_NOISESUPPRESSION_MFFREQ_1, 0x14);
		retval = mxt_write_object(data, MXT_PROCG_NOISESUPPRESSION,
			MXT_NOISESUPPRESSION_SELFREQMAX, 0x00);
		retval = mxt_write_object(data, MXT_PROCG_NOISESUPPRESSION,
			MXT_NOISESUPPRESSION_CALCFG, 0x62);
	} else {
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD]%s: Charger OFF\n", __func__);
		/* MXT_TOUCH_MULTI field */
		retval = mxt_write_object(data, MXT_TOUCH_MULTI,
			MXT_TOUCH_TCHDI, pdata->config_t9->config[MXT_TOUCH_TCHDI]);
		retval = mxt_write_object(data, MXT_TOUCH_MULTI,
			MXT_TOUCH_MRGTHR, pdata->config_t9->config[MXT_TOUCH_MRGTHR]);
		retval = mxt_write_object(data, MXT_TOUCH_MULTI,
			MXT_TOUCH_NEXTTCHDI, pdata->config_t9->config[MXT_TOUCH_NEXTTCHDI]);

		/* MXT_SPT_CTECONFIG field */
		retval = mxt_write_object(data, MXT_SPT_CTECONFIG,
			MXT_CTE_IDLESYNCSPERX,pdata->config_t46->config[MXT_CTE_IDLESYNCSPERX]);
		retval = mxt_write_object(data, MXT_SPT_CTECONFIG,
			MXT_CTE_ACTVSYNCSPERX, pdata->config_t46->config[MXT_CTE_ACTVSYNCSPERX]);

		/* MXT_PROCG_NOISESUPPRESSION field */
		retval = mxt_write_object(data, MXT_PROCG_NOISESUPPRESSION,
			MXT_NOISESUPPRESSION_MFFREQ_0,
			pdata->config_t48->config[MXT_NOISESUPPRESSION_MFFREQ_0]);
		retval = mxt_write_object(data, MXT_PROCG_NOISESUPPRESSION,
			MXT_NOISESUPPRESSION_MFFREQ_1,
			pdata->config_t48->config[MXT_NOISESUPPRESSION_MFFREQ_1]);
		retval = mxt_write_object(data, MXT_PROCG_NOISESUPPRESSION,
			MXT_NOISESUPPRESSION_SELFREQMAX,
			pdata->config_t48->config[MXT_NOISESUPPRESSION_SELFREQMAX]);
		retval = mxt_write_object(data, MXT_PROCG_NOISESUPPRESSION,
			MXT_NOISESUPPRESSION_CALCFG,
			pdata->config_t48->config[MXT_NOISESUPPRESSION_CALCFG]);
	}
}
#endif

#if defined(CONFIG_TOUCHSCREEN_F12_APO) || defined(CONFIG_TOUCHSCREEN_F12_NAD)
static void mxt_set_noisesuppression(struct mxt_data *data)
{
	int retval = 0;
	u8 val = 0x00;

	if (data->mxt_chrgon_flag) {
		val = 0x62;
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD]%s: Charger ON Set CALCFG=0x%02X\n", __func__, val);
	} else {
		val = 0x40;
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD]%s: Charger OFF Set CALCFG=0x%02X\n", __func__, val);
	}
	
	retval = mxt_write_object(data, MXT_PROCG_NOISESUPPRESSION,
		MXT_NOISESUPPRESSION_CALCFG, val);
}
#endif

static int mxt_cntr_charger_armor(struct mxt_data *data)
{
	int retval = 0;

	mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3,
		"[TPD]%s: Power Supply Status %s\n", __func__, mxt_batt_power_str[data->mxt_batt_status]);

	if ((data->mxt_batt_status == POWER_SUPPLY_STATUS_UNKNOWN) ||
		(data->mxt_batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING) ||
		(data->mxt_batt_status == POWER_SUPPLY_STATUS_DISCHARGING)) {
		/* Not Charging */
		if (data->mxt_chrgon_flag) {
			data->mxt_chrgon_flag = false;
		} else {
			goto mxt_cntr_charger_armor_exit;
		}
	} else {
		/* Charging */
		if (data->mxt_chrgon_flag) {
			goto mxt_cntr_charger_armor_exit;
		} else {
			data->mxt_chrgon_flag = true;
		}
	}

	mxt_set_noisesuppression(data);

mxt_cntr_charger_armor_exit:
	return retval;
}

static int mxt_battery_info(unsigned int batt_status, void *data)
{
	struct mxt_data *pdata = data;
	int retval = 0;

	mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD]%s: IN\n",  __func__);

	if (NULL == pdata){
		printk(KERN_ERR "%s :parameter error \n", __func__ );
		return 0;
	}

	pdata->mxt_batt_status = batt_status;

	if (pdata->mxt_suspend_flag) {
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3,
			"[TPD]%s: Cannot Charger Control. Power Supply Status %s\n",
			__func__, mxt_batt_power_str[pdata->mxt_batt_status]);
		return 0;
	}

	retval = mxt_cntr_charger_armor(pdata);

	mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD]%s: OUT\n",  __func__);
	return 0;
}

static int mxt_calibration(struct mxt_data *data)
{
	int retval = 0;

	/* W:GEN_COMMANDPROCESSOR_T6:T6_CFG_CALIBRATE, 0x01 */
	retval = mxt_write_object(data, MXT_GEN_COMMAND, MXT_COMMAND_CALIBRATE, 0x01);

	mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] Calibration Start!!\n");

	return retval;
}

#endif
/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-02-22 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static void mxt_input_report(struct mxt_data *data, int single_id)
{
	struct mxt_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
	int status = finger[single_id].status;
	int finger_num = 0;
	int id;

	for (id = 0; id < MXT_MAX_FINGER; id++) {
		if (!finger[id].status)
			continue;

		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
				finger[id].status != MXT_RELEASE ?
				finger[id].area : 0);
		input_report_abs(input_dev, ABS_MT_POSITION_X,
				finger[id].x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y,
				finger[id].y);
		input_mt_sync(input_dev);

		if (finger[id].status == MXT_RELEASE)
			finger[id].status = 0;
		else
			finger_num++;
	}

	input_report_key(input_dev, BTN_TOUCH, finger_num > 0);

	if (status != MXT_RELEASE) {
		input_report_abs(input_dev, ABS_X, finger[single_id].x);
		input_report_abs(input_dev, ABS_Y, finger[single_id].y);
	}

	input_sync(input_dev);
}
#else
static void mxt_input_report(struct mxt_data *data, int single_id)
{
	struct mxt_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
	int finger_num = 0;
	int id;

	if(!data->mxt_touchevent_flag) {
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] Touch Event Disable\n");
		return;
	}

	/* multi-touch-protocol B start */
	for (id = 0; id < MXT_MAX_FINGER; id++) {
		if (!finger[id].status)
			continue;

		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER,
				finger[id].status != MXT_RELEASE);

		if (MXT_DEBUG_LOG_LEVEL1 < set_log_level) {
			mxt_dbg_log(MXT_DEBUG_LOG_LEVEL2,
				"[TPD_Ps] ID=%d X=%03d Y=%03d Z=%03d A=%03d S=0x%02X CM=%02X %s%s%s%s%s\n",
				id, finger[id].x, finger[id].y,  finger[id].z,
				finger[id].area, finger[id].status, finger[id].component,
				finger[id].status & MXT_PRESS ? "[PRESS]" : "",
				finger[id].status & MXT_MOVE ? "[MOVE]" : "",
				finger[id].status & MXT_VECTOR ? "[VECTOR]" : "",
				finger[id].status & MXT_AMP ? "[AMP]" : "",
				finger[id].status & MXT_RELEASE ? "[RELEASE]" : "");
		}

		if (finger[id].status != MXT_RELEASE) {
			finger_num++;
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
				finger[id].area);
			input_report_abs(input_dev, ABS_MT_ORIENTATION,
				finger[id].component);
			input_report_abs(input_dev, ABS_MT_POSITION_X,
				finger[id].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y,
				finger[id].y);
			input_report_abs(input_dev, ABS_MT_PRESSURE,
				finger[id].z);
		} else {
			finger[id].status = 0;
		}
	}

	input_report_key(input_dev, BTN_TOUCH, finger_num > 0);

	input_sync(input_dev);
	/* multi-touch-protocol B end */
}
#endif
/* FUJITSU:2012-02-22 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-02-22 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static void mxt_input_touchevent(struct mxt_data *data,
				      struct mxt_message *message, int id)
{
	struct mxt_finger *finger = data->finger;
	struct device *dev = &data->client->dev;
	u8 status = message->message[0];
	int x;
	int y;
	int area;

	/* Check the touch is present on the screen */
	if (!(status & MXT_DETECT)) {
		if (status & MXT_RELEASE) {
			dev_dbg(dev, "[%d] released\n", id);

			finger[id].status = MXT_RELEASE;
			mxt_input_report(data, id);
		}
		return;
	}

	/* Check only AMP detection */
	if (!(status & (MXT_PRESS | MXT_MOVE)))
		return;

	x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0xf);
	y = (message->message[2] << 4) | ((message->message[3] & 0xf));
	if (data->pdata->x_size < 1024)
		x = x >> 2;
	if (data->pdata->y_size < 1024)
		y = y >> 2;

	area = message->message[4];

	dev_dbg(dev, "[%d] %s x: %d, y: %d, area: %d\n", id,
		status & MXT_MOVE ? "moved" : "pressed",
		x, y, area);

	finger[id].status = status & MXT_MOVE ?
				MXT_MOVE : MXT_PRESS;
	finger[id].x = x;
	finger[id].y = y;
	finger[id].area = area;

	mxt_input_report(data, id);
}
#else //MXT_ENABLE_ORIGINAL_FUNCTION
static void mxt_input_touchevent(struct mxt_data *data,
				      struct mxt_message *message, int id)
{
	struct mxt_finger *finger = data->finger;
	struct device *dev = &data->client->dev;
	u8 status = message->message[0];
	int x;
	int y;
	int area;
	int z;
	u8 component;

	mxt_dbg_log(MXT_DEBUG_LOG_LEVEL1,
		"[TPD_Rw] RID:0x%02X,M1:0x%02X,M2:0x%02X,M3:0x%02X,"
		"M4:0x%02X,M5:0x%02X,M6:0x%02X,M7:0x%02X,CHK:0x%02X\n",
		message->reportid, message->message[0], message->message[1],
		message->message[2], message->message[3], message->message[4],
		message->message[5], message->message[6], message->checksum);

	/* Check the touch is present on the screen */
	if (!(status & MXT_DETECT)) {
		if (status & MXT_RELEASE) {
			dev_dbg(dev, "[%d] released\n", id);

			finger[id].status = MXT_RELEASE;
		} else if(status & MXT_PRESS) {
			finger[id].status = MXT_RELEASE;
		} else if(status & MXT_SUPPRESS) {
			mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3,
			"[TPD]%s: Touch Suppress UP EVENT ID=%02d\n",
			__func__, id);
			finger[id].status = MXT_RELEASE;
		}
		return;
	}

	/* Check only AMP detection */
	if (!(status & (MXT_PRESS | MXT_MOVE | MXT_VECTOR)))
		return;

	x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0xf);
	y = (message->message[2] << 4) | ((message->message[3] & 0xf));
	if (data->max_x < 1024)
		x = x >> 2;
	if (data->max_y < 1024)
		y = y >> 2;

	area = message->message[4];
	z = message->message[5];
	z = (z * MXT_COEFFICIENT) / 10;
	if(MXT_MAX_Z_VALUE < z) {
		z = MXT_MAX_Z_VALUE;
	}
	component = message->message[6];

	dev_dbg(dev, "[%d] %s x: %d, y: %d, area: %d\n", id,
		status & MXT_MOVE ? "moved" : "pressed",
		x, y, area);

	finger[id].status = status;
	finger[id].x = x;
	finger[id].y = y;
	finger[id].area = area;
	finger[id].z = z;
	finger[id].component = component;
}
#endif
/* FUJITSU:2012-02-22 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	struct mxt_message message;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int id;
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;

	do {
		if (mxt_read_message(data, &message)) {
			dev_err(dev, "Failed to read message\n");
			goto end;
		}

		reportid = message.reportid;

		/* whether reportid is thing of MXT_TOUCH_MULTI */
		object = mxt_get_object(data, MXT_TOUCH_MULTI);
		if (!object)
			goto end;

		max_reportid = object->max_reportid;
		min_reportid = max_reportid - object->num_report_ids + 1;
		id = reportid - min_reportid;

		if (reportid >= min_reportid && reportid <= max_reportid)
			mxt_input_touchevent(data, &message, id);
		else
			mxt_dump_message(dev, &message);
	} while (reportid != 0xff);

end:
	return IRQ_HANDLED;
}
#else //MXT_ENABLE_ORIGINAL_FUNCTION
static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	struct mxt_message message;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int id;
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;
	struct timeval tv;
	bool touch_event = false;

	if (MXT_DEBUG_LOG_LEVEL1 < set_log_level) {
		do_gettimeofday(&tv);
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL2,
			"[TPD_Tm]%s: IN  time:%lu%06lu usec\n",
			__func__, tv.tv_sec, tv.tv_usec);
	}

	do {
		if (mxt_read_message(data, &message)) {
			dev_err(dev, "Failed to read message\n");
			goto end;
		}

		reportid = message.reportid;

		/* whether reportid is thing of MXT_TOUCH_MULTI */
		object = mxt_get_object(data, MXT_TOUCH_MULTI);
		if (!object)
			goto end;

		max_reportid = object->max_reportid;
		min_reportid = max_reportid - object->num_report_ids + 1;
		id = reportid - min_reportid;

		if (reportid >= min_reportid && reportid <= max_reportid) {
			touch_event = true;
			mxt_input_touchevent(data, &message, id);
		} else {
			mxt_dump_message(dev, &message);
		}
	} while (reportid != 0xff);

	if (touch_event) {
		mxt_input_report(data, 0);
	}

end:

	if(MXT_DEBUG_LOG_LEVEL1 < set_log_level) {
		do_gettimeofday(&tv);
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL2,
			"[TPD_Tm]%s: OUT time:%lu%06lu usec\n",
			__func__, tv.tv_sec, tv.tv_usec);
	}

	return IRQ_HANDLED;
}
#endif
/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static int mxt_check_reg_init(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int index = 0;
	int i, j, config_offset;

	if (!pdata->config) {
		dev_dbg(dev, "No cfg data defined, skipping reg init\n");
		return 0;
	}

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_writable(object->type))
			continue;

		for (j = 0; j < object->size + 1; j++) {
			config_offset = index + j;
			if (config_offset > pdata->config_length) {
				dev_err(dev, "Not enough config data!\n");
				return -EINVAL;
			}
			mxt_write_object(data, object->type, j,
					 pdata->config[config_offset]);
		}
		index += object->size + 1;
	}

	return 0;
}
#endif
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION end */

static int mxt_make_highchg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_message message;
	int count = 10;
	int error;

	/* Read dummy message to make high CHG pin */
	do {
		error = mxt_read_message(data, &message);
		if (error)
			return error;
	} while (message.reportid != 0xff && --count);

	if (!count) {
		dev_err(dev, "CHG pin isn't cleared\n");
		return -EBUSY;
	}

	return 0;
}

static int mxt_get_info(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;

	error = mxt_read_reg(client, MXT_FAMILY_ID, &val);
	if (error)
		return error;
	info->family_id = val;

	error = mxt_read_reg(client, MXT_VARIANT_ID, &val);
	if (error)
		return error;
	info->variant_id = val;

	error = mxt_read_reg(client, MXT_VERSION, &val);
	if (error)
		return error;
	info->version = val;

	error = mxt_read_reg(client, MXT_BUILD, &val);
	if (error)
		return error;
	info->build = val;

	error = mxt_read_reg(client, MXT_OBJECT_NUM, &val);
	if (error)
		return error;
	info->object_num = val;

	return 0;
}

static int mxt_get_object_table(struct mxt_data *data)
{
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[MXT_OBJECT_SIZE];

	for (i = 0; i < data->info.object_num; i++) {
		struct mxt_object *object = data->object_table + i;

		reg = MXT_OBJECT_START + MXT_OBJECT_SIZE * i;
		error = mxt_read_object_table(data->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3];
		object->instances = buf[4];
		object->num_report_ids = buf[5];

		if (object->num_report_ids) {
			reportid += object->num_report_ids *
					(object->instances + 1);
			object->max_reportid = reportid;
		}
	}

	return 0;
}

static void mxt_reset_delay(struct mxt_data *data)
{
	struct mxt_info *info = &data->info;

	switch (info->family_id) {
	case MXT224_ID:
		msleep(MXT224_RESET_TIME);
		break;
	case MXT1386_ID:
		msleep(MXT1386_RESET_TIME);
		break;
	default:
		msleep(MXT_RESET_TIME);
	}
}

/* FUJITSU:2012-04-09 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
#else
static int mxt_soft_reset(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_message message;
	int reset_timeout = 0;
	int retval = 0;
	u8 val;

	/* Soft reset */
	retval = mxt_write_object(data, MXT_GEN_COMMAND, MXT_COMMAND_RESET, 1);
	msleep(MXT_RESET_TIME);
	if (data->pdata->read_chg == NULL) {
		msleep(MXT_RESET_NOCHGREAD);
		while ((reset_timeout++ <= 10)) {
			mxt_read_message(data, &message);
			val = mxt_reportid_to_type(data, message.reportid);
			if ((MXT_GEN_COMMAND == val) && (0x80 & message.message[0])) {
				break;
			}
			msleep(10);
		}
	} else {
		while ((reset_timeout++ <= 10)) {
			if (data->pdata->read_chg() == 0) {
				mxt_read_message(data, &message);
				val = mxt_reportid_to_type(data, message.reportid);
				if ((MXT_GEN_COMMAND == val) && (0x80 & message.message[0])) {
					break;
				}
			}
			msleep(10);
		}
	}
	if (reset_timeout >= 100) {
		dev_err(&client->dev, "No response after reset!\n");
		retval = -EIO;
	} else {
		data->mxt_crc = ((message.message[3] << 16)&0x00FF0000) | ((message.message[2] << 8)&0x0000FF00) | message.message[1];
		printk("[TPD]%s: FrimConfig CRC=0x%08X\n", __func__, data->mxt_crc);
	}

	return retval;
}

static ssize_t mxt_soft_reset_attr(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	mxt_soft_reset(data);
	return count;
}

static int mxt_nvbackup(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_message message;
	int reset_timeout = 0;
	int retval = 0;
	u8 val;

	/* nv backup */
	retval = mxt_write_object(data, MXT_GEN_COMMAND,
			MXT_COMMAND_BACKUPNV,
			MXT_BACKUP_VALUE);
	msleep(MXT_BACKUP_TIME);

	if (data->pdata->read_chg == NULL) {
		msleep(MXT_RESET_NOCHGREAD);
		while (reset_timeout++ <= 10) {
			mxt_read_message(data, &message);
			val = mxt_reportid_to_type(data, message.reportid);
			if (MXT_GEN_COMMAND == val) {
				break;
			}
			msleep(10);
		}
	} else {
		while (reset_timeout++ <= 10) {
			if (data->pdata->read_chg() == 0) {
				mxt_read_message(data, &message);
				val = mxt_reportid_to_type(data, message.reportid);
				if (MXT_GEN_COMMAND == val) {
					break;
				}
			}
			msleep(10);
		}
	}

	if (reset_timeout >= 100) {
		dev_err(&client->dev, "No response after nvbackup!\n");
		retval = -EIO;
	}

	return retval;
}
static ssize_t mxt_nvbackup_attr(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	mxt_nvbackup(data);
	return count;
}

static int mxt_atob(char a)
{
	int retval = -1;

	if ((a >= '0') && (a <= '9')) {
		retval = a - '0';
	} else if ((a >= 'a') && (a <= 'f')) {
		retval = a - 'a' + 10;
	} else if ((a >= 'A') && (a <= 'F')) {
		retval = a - 'A' + 10;
	}

	return retval;
}
static int mxt_asctobin(char msb, char lsb)
{
	return (mxt_atob(lsb) | (mxt_atob(msb) << 4));
}
static ssize_t mxt_param(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int retval = -EINVAL;
	int object_type, offset, value;
	
	/* TT:OO:VV */
	if ((NULL == buf) || (count < 8) || (buf[2] != ':') || (buf[5] != ':'))
	{
		pr_err("%s: Parameter error\n", __func__);
		return 0;
	}
	object_type = mxt_asctobin(buf[0], buf[1]);
	offset = mxt_asctobin(buf[3], buf[4]);
	value = mxt_asctobin(buf[6], buf[7]);
	if (object_type < 0 || offset < 0 || value < 0)
	{
		pr_err("%s: Parameter error (%s)\n", __func__, buf);
		return 0;
	}

	printk("[TPD] write type:%d, offset:%d, value:%d\n", object_type, offset, value);
	retval = mxt_write_object(data, (u8)object_type, (u8)offset, (u8)value);
	if (retval < 0)
	{
	pr_err("%s: write object error %d\n", __func__, retval);
	}
	
	return count;
}

static ssize_t mxt_loglevel(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int val = 0;

	if ((NULL == buf) || (count < 2) || (3 < count)){
		pr_err("%s: Parameter error\n", __func__);
		return 0;
	}
	val = mxt_asctobin(buf[0], buf[1]);
	printk("[TPD]%s: Change LogLevel %d -> %d\n",
		__func__, set_log_level, val);
	set_log_level = val;

	return count;
}

static ssize_t mxt_power_on_attr(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD]%s: power on\n", __func__);
	data->pdata->power_on(true);
	return count;
}

static ssize_t mxt_power_off_attr(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD]%s: power off\n", __func__);
	data->pdata->power_on(false);
	return count;
}

static ssize_t mxt_gpio_check_attr(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD]%s: gpio check\n", __func__);
	data->pdata->gpio_check();
	return count;
}

static int mxt_grip_suppression_param_check(struct fj_touch_grip_suppression_data grip_data)
{
	if (grip_data.use_grip < 0 ||
		grip_data.use_grip > 1) {
		printk("[TPD]%s: use_grip=%d\n", __func__, grip_data.use_grip);
		return -EINVAL;
	}

	if (grip_data.grip_mode < 0 ||
		grip_data.grip_mode > 1) {
		printk("[TPD]%s: grip_mode=%d\n", __func__, grip_data.grip_mode);
		return -EINVAL;
	}

	if (grip_data.x_lo_grip < 0 ||
		grip_data.x_lo_grip > 255) {
		printk("[TPD]%s: x_lo_grip=%d\n", __func__, grip_data.x_lo_grip);
		return -EINVAL;
	}

	if (grip_data.x_hi_grip < 0 ||
		grip_data.x_hi_grip > 255) {
		printk("[TPD]%s: x_hi_grip=%d\n", __func__, grip_data.x_hi_grip);
		return -EINVAL;
	}

	if (grip_data.y_lo_grip < 0 ||
		grip_data.y_lo_grip > 255) {
		printk("[TPD]%s: y_lo_grip=%d\n", __func__, grip_data.y_lo_grip);
		return -EINVAL;
	}

	if (grip_data.y_hi_grip < 0 ||
		grip_data.y_hi_grip > 255) {
		printk("[TPD]%s: y_hi_grip=%d\n", __func__, grip_data.y_hi_grip);
		return -EINVAL;
	}

	return 0;
}

static int mxt_grip_suppression(struct mxt_data *data,
					struct fj_touch_grip_suppression_data grip_data)
{
	struct mxt_object *object;
	u8 t40_data[T40_DATA_SIZE] = {0};
	u16 reg = 0x00;
	int retval = 0;

	retval = mxt_grip_suppression_param_check(grip_data);
	if (retval < 0) {
		printk("[TPD]%s: Grip Data Invalid Err retval=%d\n", __func__, retval);
		return retval;
	}

	t40_data[0] = t40_data[0] | (grip_data.use_grip & 0x01);
	t40_data[0] = t40_data[0] | ((grip_data.grip_mode <<  4) & 0x10);
	t40_data[1] = grip_data.x_lo_grip;
	t40_data[2] = grip_data.x_hi_grip;
	t40_data[3] = grip_data.y_lo_grip;
	t40_data[4] = grip_data.y_hi_grip;

	object = mxt_get_object(data, MXT_PROCI_GRIP);
	if (!object) {
		retval = -EINVAL;
		return retval;
	}

	reg = object->start_address;
	retval = __mxt_write_reg(data->client, reg, T40_DATA_SIZE, t40_data);
	if (retval < 0) {
		printk("[TPD]%s: Grip Data Err retval=%d\n", __func__, retval);
		return retval;
	}

	return retval;
}

static int mxt_selftest_param_check(struct fj_touch_do_selftest_data self_data)
{
	switch (self_data.test_cmd) {
	case MXT_AVDD_TEST:
		break;
	case MXT_PIN_FAULT_TEST:
		break;
	case MXT_SIGNAL_LIMIT_TEST:
		break;
	case MXT_ALL_THE_TEST:
		break;
	default:
		printk("[TPD]%s: test_cmd=%d\n", __func__, self_data.test_cmd);
		return -EINVAL;
	}

	if (self_data.high_signal_limit < 0 ||
		self_data.high_signal_limit > 65535) {
		printk("[TPD]%s: high_signal_limit=%d\n", __func__, self_data.high_signal_limit);
		return -EINVAL;
	}

	if (self_data.low_signal_limit < 0 ||
		self_data.low_signal_limit > 65535) {
		printk("[TPD]%s: low_signal_limit=%d\n", __func__, self_data.low_signal_limit);
		return -EINVAL;
	}

	if (self_data.result_data_buf == NULL) {
		printk("[TPD]%s: result_data_buf NULL\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int mxt_selftest(struct mxt_data *data,
					struct fj_touch_do_selftest_data self_data)
{
	struct mxt_message message;
	struct device *dev = &data->client->dev;
	int cnt = 0;
	int retval = 0;
	int retval_user = 0;
	char self_result_data[T25_DATA_SIZE];

	memset(&self_result_data, 0xFF, sizeof(self_result_data));

	retval = mxt_selftest_param_check(self_data);
	if (retval < 0) {
		printk("[TPD]%s: SelftTest Data Invalid Err retval=%d\n", __func__, retval);
		goto err_mxt_selftest;
	}

	retval = mxt_write_object(data, MXT_SPT_SELFTEST, 0x00, 0x03);

	retval = mxt_write_object(data, MXT_SPT_SELFTEST, 0x02,
		self_data.high_signal_limit & 0xFF);
	retval = mxt_write_object(data, MXT_SPT_SELFTEST, 0x03,
		(self_data.high_signal_limit >> 8) & 0xFF);
	retval = mxt_write_object(data, MXT_SPT_SELFTEST, 0x04,
		self_data.low_signal_limit & 0xFF);
	retval = mxt_write_object(data, MXT_SPT_SELFTEST, 0x05,
		(self_data.low_signal_limit >> 8) & 0xFF);

	retval = mxt_write_object(data, MXT_SPT_SELFTEST, 0x01, self_data.test_cmd);

    for (cnt = 0; cnt < MXT_MAX_INIT_WAIT; cnt += 10) {
		if (data->pdata->read_chg() == 0) {
			if (mxt_read_message(data, &message) == 0) {
				u8 val = mxt_reportid_to_type(data, message.reportid);
				if (MXT_SPT_SELFTEST == val) {
					printk("[TPD] Self Test Complete\n");
					mxt_dump_message(dev, &message);
					break;
				}
			}
		}
		msleep( 10 );
    }

	retval = mxt_write_object(data, MXT_SPT_SELFTEST, 0x00, 0x00);

	if (cnt >= MXT_MAX_INIT_WAIT) {
		printk("[TPD]%s: SELFTEST Err\n", __func__);
		mxt_dump_message(dev, &message);
		retval = -EFAULT;
		goto err_mxt_selftest;
	}

	self_result_data[0] = message.message[0];
	self_result_data[1] = message.message[1];
	self_result_data[2] = message.message[2];
	self_result_data[3] = message.message[3];
	self_result_data[4] = message.message[4];
	self_result_data[5] = message.message[5];
	self_result_data[6] = message.message[6];

	mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3,
		"[TPD] ST:0x%02X INFO:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X --:0x%02X\n",
		self_result_data[0],
		self_result_data[1],
		self_result_data[2],
		self_result_data[3],
		self_result_data[4],
		self_result_data[5],
		self_result_data[6]);

	retval = copy_to_user((int __user *)self_data.result_data_buf, &self_result_data, sizeof(self_result_data));
	if (retval != 0) {
		printk(KERN_ERR "copy_to_user err %d \n", retval);
	}

	return retval;

err_mxt_selftest:
	if(self_data.result_data_buf != NULL) {
		retval_user = copy_to_user((int __user *)self_data.result_data_buf, &self_result_data, sizeof(self_result_data));
		if (retval_user != 0) {
			printk(KERN_ERR "copy_to_user err %d \n", retval_user);
			retval = retval_user;
		}
	}

	return retval;
}

static int mxt_diagnostic_param_check(struct fj_touch_debug_diagnostic_data diag_data)
{
	switch (diag_data.test_cmd) {
	case MXT_PAGE_UP:
		break;
	case MXT_PAGE_DOWN:
		break;
	case MXT_DELTAS_MODE:
		break;
	case MXT_REFERENCES_MODE:
		break;
	default:
		printk("[TPD]%s: test_cmd=%d\n", __func__, diag_data.test_cmd);
		return -EINVAL;
	}

	if (diag_data.raw_data_buf == NULL) {
		printk("[TPD]%s: raw_data_buf NULL\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int mxt_diagnostic(struct mxt_data *data,
					struct fj_touch_debug_diagnostic_data diag_data)
{
	struct mxt_object *object;
	u16 reg = 0x00;
	u8 val = 0x00;
	int cnt = 0;
	int retval = 0;
	int retval_user = 0;
	char diag_raw_data[T37_DATA_SIZE];

	memset(&diag_raw_data, 0xFF, sizeof(diag_raw_data));

	retval = mxt_diagnostic_param_check(diag_data);
	if (retval < 0) {
		printk("[TPD]%s: Diagnostic Data Invalid Err retval=%d\n", __func__, retval);
		goto err_mxt_diagnostic;
	}

	switch(diag_data.test_cmd) {
	case MXT_PAGE_UP:
		retval = mxt_write_object(data, MXT_GEN_COMMAND, MXT_COMMAND_DIAGNOSTIC, MXT_PAGE_UP);
		break;
	case MXT_PAGE_DOWN:
		retval = mxt_write_object(data, MXT_GEN_COMMAND, MXT_COMMAND_DIAGNOSTIC, MXT_PAGE_DOWN);
		break;
	case MXT_DELTAS_MODE:
		if(data->diag_mode == MXT_DELTAS_MODE) {
			break;
		}
		data->diag_mode = MXT_DELTAS_MODE;
		retval = mxt_write_object(data, MXT_GEN_COMMAND, MXT_COMMAND_DIAGNOSTIC, MXT_DELTAS_MODE);
		break;
	case MXT_REFERENCES_MODE:
		if(data->diag_mode == MXT_REFERENCES_MODE) {
			break;
		}
		data->diag_mode = MXT_REFERENCES_MODE;
		retval = mxt_write_object(data, MXT_GEN_COMMAND, MXT_COMMAND_DIAGNOSTIC, MXT_REFERENCES_MODE);
		break;
	default:
		printk("DIAGNOSTIC Command Err\n");
		retval = -EINVAL;
		goto err_mxt_diagnostic;
	}

	for (cnt = 0; cnt < MXT_MAX_INIT_WAIT; cnt += 10) {
		retval = mxt_read_object(data, MXT_GEN_COMMAND, MXT_COMMAND_DIAGNOSTIC, &val);
		if(val == 0x00) {
			mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3,
				"[TPD] Diagnostic Command Set\n");
			break;
		}
		msleep(10);
	}

	if(cnt >= MXT_MAX_INIT_WAIT) {
		printk("[TPD]%s: Diagnostic Command Set Err\n", __func__);
		retval = -EFAULT;
		goto err_mxt_diagnostic;
	}

	object = mxt_get_object(data, MXT_DEBUG_DIAGNOSTIC);
	if (!object) {
		retval = -EINVAL;
		goto err_mxt_diagnostic;
	}

	reg = object->start_address;
	retval = __mxt_read_reg(data->client, reg,
			T37_DATA_SIZE, diag_raw_data);
	if (retval < 0) {
		printk("[TPD]%s: Debug Diagnostic Err retval=%d\n", __func__, retval);
		goto err_mxt_diagnostic;
	}

	if(MXT_DEBUG_LOG_LEVEL3 <= set_log_level) {
		printk("DIAGNOSTIC Raw Data\n");
		printk("MODE:0x%02X PAGE:%d\n",
			diag_raw_data[0],
			diag_raw_data[1]);
		for(cnt = 2; cnt < 128 ; cnt++) {
			printk("DataIndex[%03d]:0x%02X\n", cnt, diag_raw_data[cnt]);
		}
	}

	msleep(50);

	retval = copy_to_user((int __user *)diag_data.raw_data_buf, &diag_raw_data, sizeof(diag_raw_data));
	if (retval != 0) {
		printk(KERN_ERR "copy_to_user err %d \n", retval);
	}

	return retval;

err_mxt_diagnostic:
	if(diag_data.raw_data_buf != NULL) {
		retval_user = copy_to_user((int __user *)diag_data.raw_data_buf, &diag_raw_data, sizeof(diag_raw_data));
		if (retval_user != 0) {
			printk(KERN_ERR "copy_to_user err %d \n", retval_user);
			retval = retval_user;
		}
	}

	return retval;
}

static int mxt_reset_and_config(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	int retval = 0;
	unsigned int check_crc = *pdata->config_crc;

	retval = mxt_soft_reset(data);
	if (retval < 0) {
		return retval;
	}

	if (check_crc && (check_crc == data->mxt_crc)) {
		return retval;
	}

	printk("[TPD]%s: Different CRC 0x%08X -> 0x%08X\n", __func__, check_crc, data->mxt_crc);

	/* W:SPT_CTECONFIG */
	mxt_multi_write_object(data, MXT_SPT_CTECONFIG,
		(void *)pdata->config_t46->config, pdata->config_t46->length);

	/* W:GEN_COMMANDPROCESSOR:CFG_BACKUPNV, 0x55 */
	 mxt_write_object(data, MXT_GEN_COMMAND,
		MXT_COMMAND_BACKUPNV,
		MXT_BACKUP_VALUE);

	/* 10ms wait */
	msleep(10);

	/* W:GEN_COMMANDPROCESSOR:CFG_RESET, 0x11 */
	mxt_write_object(data, MXT_GEN_COMMAND, MXT_COMMAND_RESET, 0x11);

	/* 100ms wait */
	msleep(100);

	/* W:GEN_COMMANDPROCESSOR */
	mxt_multi_write_object(data, MXT_GEN_COMMAND,
		(void *)pdata->config_t6->config, pdata->config_t6->length);

	/* W:GEN_POWERCONFIG */
	mxt_multi_write_object(data, MXT_GEN_POWER,
		(void *)pdata->config_t7->config, pdata->config_t7->length);

	/* W:GEN_ACQUISITIONCONFIG */
	mxt_multi_write_object(data, MXT_GEN_ACQUIRE,
		(void *)pdata->config_t8->config, pdata->config_t8->length);

	/* W:TOUCH_MULTITOUCHSCREEN */
	mxt_multi_write_object(data, MXT_TOUCH_MULTI,
		(void *)pdata->config_t9->config, pdata->config_t9->length);

	/* W:TOUCH_KEYARRAY */
	mxt_multi_write_object(data, MXT_TOUCH_KEYARRAY,
		(void *)pdata->config_t15->config, pdata->config_t15->length);

	/* W:SPT_COMCONFIG */
	mxt_multi_write_object(data, MXT_SPT_COMMSCONFIG,
		(void *)pdata->config_t18->config, pdata->config_t18->length);

	/* W:SPT_GPIOPWM */
	mxt_multi_write_object(data, MXT_SPT_GPIOPWM,
		(void *)pdata->config_t19->config, pdata->config_t19->length);

	/* W:PROCI_GRIPSUPPRESSION */
	mxt_multi_write_object(data, MXT_PROCI_GRIP,
		(void *)pdata->config_t40->config, pdata->config_t40->length);

	/* W:PROCI_TOUCHSUPPRESSION */
	mxt_multi_write_object(data, MXT_PROCI_TOUCHSUPPRESSION,
		(void *)pdata->config_t42->config, pdata->config_t42->length);

	/* W:PROCG_NOISESUPPRESSION */
	mxt_multi_write_object(data, MXT_PROCG_NOISESUPPRESSION,
		(void *)pdata->config_t48->config, pdata->config_t48->length);

	/* W:PROCI_STYLUS */
	mxt_multi_write_object(data, MXT_PROCI_STYLUS,
		(void *)pdata->config_t47->config, pdata->config_t47->length);

	/* W:TOUCH_PROXIMITY */
	mxt_multi_write_object(data, MXT_TOUCH_PROXIMITY,
		(void *)pdata->config_t23->config, pdata->config_t23->length);

	/* W:SPT_SELFTEST */
	mxt_multi_write_object(data, MXT_SPT_SELFTEST,
		(void *)pdata->config_t25->config, pdata->config_t25->length);

	/* W:SPT_CTECONFIG */
	mxt_multi_write_object(data, MXT_SPT_CTECONFIG,
		(void *)pdata->config_t46->config, pdata->config_t46->length);

	/* msleep(10)*n */
	/* R:GEN_MESSAGEPROCESSOR; */
	retval = mxt_nvbackup(data);

	return retval;
}

static void mxt_calc_resolution(struct mxt_data *data)
{
	unsigned int max_x = 0;
	unsigned int max_y = 0;

	max_x = ((data->pdata->config_t9->config[MXT_TOUCH_XRANGE_LSB] & 0x00ff) |
		 ((data->pdata->config_t9->config[MXT_TOUCH_XRANGE_MSB] << 8) & 0xff00));
	max_y = ((data->pdata->config_t9->config[MXT_TOUCH_YRANGE_LSB] & 0x00ff) |
		 ((data->pdata->config_t9->config[MXT_TOUCH_YRANGE_MSB] << 8) & 0xff00));

	if (data->pdata->config_t9->config[MXT_TOUCH_ORIENT] & MXT_XY_SWITCH) {
		data->max_x = max_y;
		data->max_y = max_x;
	} else {
		data->max_x = max_x;
		data->max_y = max_y;
	}
}
#endif
/* FUJITSU:2012-04-09 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	int timeout_counter = 0;
	u8 val;
	u8 command_register;
	struct mxt_object *t7_object;

	error = mxt_get_info(data);
	if (error)
		return error;

	data->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt_object),
				     GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_get_object_table(data);
	if (error)
		goto free_object_table;

	/* Check register init values */
	error = mxt_check_reg_init(data);
	if (error)
		goto free_object_table;

	/* Store T7 and T9 locally, used in suspend/resume operations */
	t7_object = mxt_get_object(data, MXT_GEN_POWER);
	if (!t7_object) {
		dev_err(&client->dev, "Failed to get T7 object\n");
		error = -EINVAL;
		goto free_object_table;
	}

	data->t7_start_addr = t7_object->start_address;
	error = __mxt_read_reg(client, data->t7_start_addr,
				T7_DATA_SIZE, data->t7_data);
	if (error < 0) {
		dev_err(&client->dev,
			"Failed to save current power state\n");
		goto free_object_table;
	}
	error = mxt_read_object(data, MXT_TOUCH_MULTI, MXT_TOUCH_CTRL,
			&data->t9_ctrl);
	if (error < 0) {
		dev_err(&client->dev, "Failed to save current touch object\n");
		goto free_object_table;
	}

	/* Backup to memory */
	mxt_write_object(data, MXT_GEN_COMMAND,
			MXT_COMMAND_BACKUPNV,
			MXT_BACKUP_VALUE);
	msleep(MXT_BACKUP_TIME);
	do {
		error =  mxt_read_object(data, MXT_GEN_COMMAND,
					MXT_COMMAND_BACKUPNV,
					&command_register);
		if (error)
			goto free_object_table;
		usleep_range(1000, 2000);
	} while ((command_register != 0) && (++timeout_counter <= 100));
	if (timeout_counter > 100) {
		dev_err(&client->dev, "No response after backup!\n");
		error = -EIO;
		goto free_object_table;
	}


	/* Soft reset */
	mxt_write_object(data, MXT_GEN_COMMAND,
			MXT_COMMAND_RESET, 1);

	mxt_reset_delay(data);

	/* Update matrix size at info struct */
	error = mxt_read_reg(client, MXT_MATRIX_X_SIZE, &val);
	if (error)
		goto free_object_table;
	info->matrix_xsize = val;

	error = mxt_read_reg(client, MXT_MATRIX_Y_SIZE, &val);
	if (error)
		goto free_object_table;
	info->matrix_ysize = val;

	dev_info(&client->dev,
			"Family ID: %d Variant ID: %d Version: %d Build: %d\n",
			info->family_id, info->variant_id, info->version,
			info->build);

	dev_info(&client->dev,
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);

	return 0;

free_object_table:
	kfree(data->object_table);
	return error;
}
#else
static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;
	struct mxt_object *t7_object;

	error = mxt_get_info(data);
	if (error)
		return error;

	data->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt_object),
				     GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_get_object_table(data);
	if (error)
		goto free_object_table;

	/* reset and configuration */
	mxt_reset_and_config(data);

	/* Store T7 and T9 locally, used in suspend/resume operations */
	t7_object = mxt_get_object(data, MXT_GEN_POWER);
	if (!t7_object) {
		dev_err(&client->dev, "Failed to get T7 object\n");
		error = -EINVAL;
		goto free_object_table;
	}

	data->t7_start_addr = t7_object->start_address;
	error = __mxt_read_reg(client, data->t7_start_addr,
				T7_DATA_SIZE, data->t7_data);
	if (error < 0) {
		dev_err(&client->dev,
			"Failed to save current power state\n");
		goto free_object_table;
	}
	error = mxt_read_object(data, MXT_TOUCH_MULTI, MXT_TOUCH_CTRL,
			&data->t9_ctrl);
	if (error < 0) {
		dev_err(&client->dev, "Failed to save current touch object\n");
		goto free_object_table;
	}

	/* Update matrix size at info struct */
	error = mxt_read_reg(client, MXT_MATRIX_X_SIZE, &val);
	if (error)
		goto free_object_table;
	info->matrix_xsize = val;

	error = mxt_read_reg(client, MXT_MATRIX_Y_SIZE, &val);
	if (error)
		goto free_object_table;
	info->matrix_ysize = val;

	printk("[TPD] FamilyID:%d VariantID:%d Version:%d Build:%d "
			"MatrixX Size:%d MatrixY Size:%d ObjectNum:%d\n",
			info->family_id, info->variant_id, info->version,
			info->build, info->matrix_xsize, info->matrix_ysize,
			info->object_num);
/*
	dev_info(&client->dev,
			"Family ID: %d Variant ID: %d Version: %d Build: %d\n",
			info->family_id, info->variant_id, info->version,
			info->build);

	dev_info(&client->dev,
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);
*/
	return 0;

free_object_table:
	kfree(data->object_table);
	return error;
}
#endif
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION end */

static ssize_t mxt_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 val;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		count += snprintf(buf + count, PAGE_SIZE - count,
				"Object[%d] (Type %d)\n",
				i + 1, object->type);
		if (count >= PAGE_SIZE)
			return PAGE_SIZE - 1;

		if (!mxt_object_readable(object->type)) {
			count += snprintf(buf + count, PAGE_SIZE - count,
					"\n");
			if (count >= PAGE_SIZE)
				return PAGE_SIZE - 1;
			continue;
		}

		for (j = 0; j < object->size + 1; j++) {
			error = mxt_read_object(data,
						object->type, j, &val);
			if (error)
				return error;

			count += snprintf(buf + count, PAGE_SIZE - count,
					"\t[%2d]: %02x (%d)\n", j, val, val);
			if (count >= PAGE_SIZE)
				return PAGE_SIZE - 1;
		}

		count += snprintf(buf + count, PAGE_SIZE - count, "\n");
		if (count >= PAGE_SIZE)
			return PAGE_SIZE - 1;
	}

	return count;
}

static int mxt_load_fw(struct device *dev, const char *fn)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	int ret;

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	}

	/* Change to the bootloader mode */
	mxt_write_object(data, MXT_GEN_COMMAND,
			MXT_COMMAND_RESET, MXT_BOOT_VALUE);

	mxt_reset_delay(data);

	/* Change to slave address of bootloader */
	if (client->addr == MXT_APP_LOW)
		client->addr = MXT_BOOT_LOW;
	else
		client->addr = MXT_BOOT_HIGH;

	ret = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);
	if (ret)
		goto out;

	/* Unlock bootloader */
	mxt_unlock_bootloader(client);

	while (pos < fw->size) {
		ret = mxt_check_bootloader(client,
						MXT_WAITING_FRAME_DATA);
		if (ret)
			goto out;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;

		/* Write one frame to device */
		mxt_fw_write(client, fw->data + pos, frame_size);

		ret = mxt_check_bootloader(client,
						MXT_FRAME_CRC_PASS);
		if (ret)
			goto out;

		pos += frame_size;

		dev_dbg(dev, "Updated %d bytes / %zd bytes\n", pos, fw->size);
	}

out:
	release_firmware(fw);

	/* Change to slave address of application */
	if (client->addr == MXT_BOOT_LOW)
		client->addr = MXT_APP_LOW;
	else
		client->addr = MXT_APP_HIGH;

	return ret;
}

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	disable_irq(data->irq);

	error = mxt_load_fw(dev, MXT_FW_NAME);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_dbg(dev, "The firmware update succeeded\n");

		/* Wait for reset */
		msleep(MXT_FWRESET_TIME);

		kfree(data->object_table);
		data->object_table = NULL;

		mxt_initialize(data);
	}

	enable_irq(data->irq);

	error = mxt_make_highchg(data);
	if (error)
		return error;

	return count;
}

/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static DEVICE_ATTR(object, 0444, mxt_object_show, NULL);
static DEVICE_ATTR(update_fw, 0664, NULL, mxt_update_fw_store);

static struct attribute *mxt_attrs[] = {
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
	NULL
};
#else //MXT_ENABLE_ORIGINAL_FUNCTION
static DEVICE_ATTR(object, 0444, mxt_object_show, NULL);
static DEVICE_ATTR(update_fw, 0664, NULL, mxt_update_fw_store);
static DEVICE_ATTR(reset, 0664, NULL, mxt_soft_reset_attr);
static DEVICE_ATTR(nvbackup, 0664, NULL, mxt_nvbackup_attr);
static DEVICE_ATTR(param, 0664, NULL, mxt_param);
static DEVICE_ATTR(loglevel, 0664, NULL, mxt_loglevel);
static DEVICE_ATTR(power_on, 0664, NULL, mxt_power_on_attr);
static DEVICE_ATTR(power_off, 0664, NULL, mxt_power_off_attr);
static DEVICE_ATTR(gpio_check, 0664, NULL, mxt_gpio_check_attr);

static struct attribute *mxt_attrs[] = {
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_reset.attr,
	&dev_attr_nvbackup.attr,
	&dev_attr_param.attr,
	&dev_attr_loglevel.attr,
	&dev_attr_power_on.attr,
	&dev_attr_power_off.attr,
	&dev_attr_gpio_check.attr,
	NULL
};
#endif
/* FUJITSU:2012-03-07 MXT_ENABLE_ORIGINAL_FUNCTION end */

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

static int mxt_start(struct mxt_data *data)
{
	int error;

	/* restore the old power state values and reenable touch */
	error = __mxt_write_reg(data->client, data->t7_start_addr,
				T7_DATA_SIZE, data->t7_data);
	if (error < 0) {
		dev_err(&data->client->dev,
			"failed to restore old power state\n");
		return error;
	}

	error = mxt_write_object(data,
			MXT_TOUCH_MULTI, MXT_TOUCH_CTRL, data->t9_ctrl);
	if (error < 0) {
		dev_err(&data->client->dev, "failed to restore touch\n");
		return error;
	}

	return 0;
}

static int mxt_stop(struct mxt_data *data)
{
	int error;
	u8 t7_data[T7_DATA_SIZE] = {0};

	/* disable touch and configure deep sleep mode */
	error = mxt_write_object(data, MXT_TOUCH_MULTI, MXT_TOUCH_CTRL, 0);
	if (error < 0) {
		dev_err(&data->client->dev, "failed to disable touch\n");
		return error;
	}

	error = __mxt_write_reg(data->client, data->t7_start_addr,
				T7_DATA_SIZE, t7_data);
	if (error < 0) {
		dev_err(&data->client->dev,
			"failed to configure deep sleep mode\n");
		return error;
	}

	return 0;
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int error;

	error = mxt_start(data);
	if (error < 0) {
		dev_err(&data->client->dev, "mxt_start failed in input_open\n");
		return error;
	}

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int error;

	error = mxt_stop(data);
	if (error < 0)
		dev_err(&data->client->dev, "mxt_stop failed in input_close\n");

}

static int mxt_power_on(struct mxt_data *data, bool on)
{
	int rc;

	if (on == false)
		goto power_off;

	rc = regulator_set_optimum_mode(data->vcc, MXT_ACTIVE_LOAD_UA);
	if (rc < 0) {
		dev_err(&data->client->dev, "Regulator set_opt failed rc=%d\n",
									rc);
		return rc;
	}

	rc = regulator_enable(data->vcc);
	if (rc) {
		dev_err(&data->client->dev, "Regulator enable failed rc=%d\n",
									rc);
		goto error_reg_en_vcc;
	}

	if (data->pdata->i2c_pull_up) {
		rc = regulator_set_optimum_mode(data->vcc_i2c, MXT_I2C_LOAD_UA);
		if (rc < 0) {
			dev_err(&data->client->dev,
				"Regulator set_opt failed rc=%d\n", rc);
			goto error_reg_opt_i2c;
		}

		rc = regulator_enable(data->vcc_i2c);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_i2c;
		}
	}

	msleep(130);

	return 0;

error_reg_en_vcc_i2c:
	if (data->pdata->i2c_pull_up)
		regulator_set_optimum_mode(data->vcc_i2c, 0);
error_reg_opt_i2c:
	regulator_disable(data->vcc);
error_reg_en_vcc:
	regulator_set_optimum_mode(data->vcc, 0);
	return rc;

power_off:
	regulator_set_optimum_mode(data->vcc, 0);
	regulator_disable(data->vcc);
	if (data->pdata->i2c_pull_up) {
		regulator_set_optimum_mode(data->vcc_i2c, 0);
		regulator_disable(data->vcc_i2c);
	}
	msleep(50);
	return 0;
}

/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static int mxt_regulator_configure(struct mxt_data *data, bool on)
{
	int rc;

	if (on == false)
		goto hw_shutdown;

	data->vcc = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vcc)) {
		rc = PTR_ERR(data->vcc);
		dev_err(&data->client->dev, "Regulator get failed rc=%d\n",
									rc);
		return rc;
	}

	if (regulator_count_voltages(data->vcc) > 0) {
		rc = regulator_set_voltage(data->vcc, MXT_VTG_MIN_UV,
							MXT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc;
		}
	}

	if (data->pdata->i2c_pull_up) {
		data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
		if (IS_ERR(data->vcc_i2c)) {
			rc = PTR_ERR(data->vcc_i2c);
			dev_err(&data->client->dev,
				"Regulator get failed rc=%d\n",	rc);
			goto error_get_vtg_i2c;
		}
		if (regulator_count_voltages(data->vcc_i2c) > 0) {
			rc = regulator_set_voltage(data->vcc_i2c,
				MXT_I2C_VTG_MIN_UV, MXT_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_i2c;
			}
		}
	}

	return 0;

error_set_vtg_i2c:
	regulator_put(data->vcc_i2c);
error_get_vtg_i2c:
	if (regulator_count_voltages(data->vcc) > 0)
		regulator_set_voltage(data->vcc, 0, MXT_VTG_MAX_UV);
error_set_vtg_vcc:
	regulator_put(data->vcc);
	return rc;

hw_shutdown:
	if (regulator_count_voltages(data->vcc) > 0)
		regulator_set_voltage(data->vcc, 0, MXT_VTG_MAX_UV);
	regulator_put(data->vcc);
	if (data->pdata->i2c_pull_up) {
		if (regulator_count_voltages(data->vcc_i2c) > 0)
			regulator_set_voltage(data->vcc_i2c, 0,
						MXT_I2C_VTG_MAX_UV);
		regulator_put(data->vcc_i2c);
	}
	return 0;
}
#endif
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION end */

#ifdef CONFIG_PM
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static int mxt_regulator_lpm(struct mxt_data *data, bool on)
{

	int rc;

	if (on == false)
		goto regulator_hpm;

	rc = regulator_set_optimum_mode(data->vcc, MXT_LPM_LOAD_UA);
	if (rc < 0) {
		dev_err(&data->client->dev,
			"Regulator set_opt failed rc=%d\n", rc);
		goto fail_regulator_lpm;
	}

	if (data->pdata->i2c_pull_up) {
		rc = regulator_set_optimum_mode(data->vcc_i2c,
						MXT_I2C_LPM_LOAD_UA);
		if (rc < 0) {
			dev_err(&data->client->dev,
				"Regulator set_opt failed rc=%d\n", rc);
			goto fail_regulator_lpm;
		}
	}

	return 0;

regulator_hpm:

	rc = regulator_set_optimum_mode(data->vcc, MXT_ACTIVE_LOAD_UA);
	if (rc < 0) {
		dev_err(&data->client->dev,
			"Regulator set_opt failed rc=%d\n", rc);
		goto fail_regulator_hpm;
	}

	if (data->pdata->i2c_pull_up) {
		rc = regulator_set_optimum_mode(data->vcc_i2c, MXT_I2C_LOAD_UA);
		if (rc < 0) {
			dev_err(&data->client->dev,
				"Regulator set_opt failed rc=%d\n", rc);
			goto fail_regulator_hpm;
		}
	}

	return 0;

fail_regulator_lpm:
	regulator_set_optimum_mode(data->vcc, MXT_ACTIVE_LOAD_UA);
	if (data->pdata->i2c_pull_up)
		regulator_set_optimum_mode(data->vcc_i2c, MXT_I2C_LOAD_UA);

	return rc;

fail_regulator_hpm:
	regulator_set_optimum_mode(data->vcc, MXT_LPM_LOAD_UA);
	if (data->pdata->i2c_pull_up)
		regulator_set_optimum_mode(data->vcc_i2c, MXT_I2C_LPM_LOAD_UA);

	return rc;
}
#endif
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	int error;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users) {
		error = mxt_stop(data);
		if (error < 0) {
			dev_err(dev, "mxt_stop failed in suspend\n");
			mutex_unlock(&input_dev->mutex);
			return error;
		}

	}

	mutex_unlock(&input_dev->mutex);

	/* put regulators in low power mode */
	error = mxt_regulator_lpm(data, true);
	if (error < 0) {
		dev_err(dev, "failed to enter low power mode\n");
		return error;
	}

	return 0;
}
#else
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	struct mxt_finger *finger = data->finger;
	int error;
	int id = 0;

	if(data->mxt_suspend_flag) {
		printk("[TPD]%s: Already Suspend Mode\n", __func__);
		return 0;
	}

    disable_irq(data->irq);

	mutex_lock(&input_dev->mutex);

	data->mxt_suspend_flag = true;

	for (id = 0; id < MXT_MAX_FINGER; id++) {
		if (!finger[id].status) {
			continue;
		}
		finger[id].status = MXT_RELEASE;
	}
	mxt_input_report(data, 0);

	mxt_write_object(data, MXT_SPT_COMMSCONFIG, MXT_COMMS_CMD, 0x02);
	mdelay(1);

	if (input_dev->users) {
		error = mxt_stop(data);
		if (error < 0) {
			dev_err(dev, "mxt_stop failed in suspend\n");
			mutex_unlock(&input_dev->mutex);
			return error;
		}

	}

	mxt_make_highchg(data);

	mutex_unlock(&input_dev->mutex);

	return 0;
}
#endif
/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	int error;

	/* put regulators in high power mode */
	error = mxt_regulator_lpm(data, false);
	if (error < 0) {
		dev_err(dev, "failed to enter high power mode\n");
		return error;
	}

	mutex_lock(&input_dev->mutex);

	if (input_dev->users) {
		error = mxt_start(data);
		if (error < 0) {
			dev_err(dev, "mxt_start failed in resume\n");
			mutex_unlock(&input_dev->mutex);
			return error;
		}
	}

	mutex_unlock(&input_dev->mutex);

	return 0;
}
#else //MXT_ENABLE_ORIGINAL_FUNCTION
static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	int error;

	if(!data->mxt_suspend_flag) {
		printk("[TPD]%s: Already Active Mode\n", __func__);
		return 0;
	}

	mutex_lock(&input_dev->mutex);

	data->mxt_suspend_flag = false;

	if (input_dev->users) {
		error = mxt_start(data);
		if (error < 0) {
			dev_err(dev, "mxt_start failed in resume\n");
			mutex_unlock(&input_dev->mutex);
			return error;
		}
	}

	mdelay(1);
	mxt_write_object(data, MXT_SPT_COMMSCONFIG, MXT_COMMS_CMD, 0x01);

	mxt_cntr_charger_armor(data);

	mxt_make_highchg(data);

	enable_irq(data->irq);

	mxt_calibration(data);

	mutex_unlock(&input_dev->mutex);

	return 0;
}
#endif
/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION end */

#if defined(CONFIG_HAS_EARLYSUSPEND)
/* FUJITSU:2012-01-30 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static void mxt_early_suspend(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);

	mxt_suspend(&data->client->dev);
}

static void mxt_late_resume(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);

	mxt_resume(&data->client->dev);
}
#else //MXT_ENABLE_ORIGINAL_FUNCTION
static void mxt_early_suspend(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);

	if (NULL == h || NULL == data) {
		printk(KERN_ERR "%s :parameter error \n", __func__ );
		return;
	}

	printk(KERN_INFO "[TPD]%s: EARLY SUSPEND data=%p\n", __func__, data);

	mxt_suspend(&data->client->dev);

	printk(KERN_INFO "[TPD]%s: EARLY SUSPEND EXIT data=%p\n", __func__, data);
}

static void mxt_late_resume(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);

	if (NULL == h || NULL == data) {
		printk(KERN_ERR "%s :parameter error \n", __func__ );
		return;
	}

	printk(KERN_INFO "[TPD]%s: LATE RESUME data=%p\n", __func__, data);

	mxt_resume(&data->client->dev);

	printk(KERN_INFO "[TPD]%s: LATE RESUME EXIT data=%p\n", __func__, data);
}
#endif
/* FUJITSU:2012-01-30 MXT_ENABLE_ORIGINAL_FUNCTION end */
#endif

static const struct dev_pm_ops mxt_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= mxt_suspend,
	.resume		= mxt_resume,
#endif
};
#endif

/* FUJITSU:2012-04-09 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
#else
static int mxt_touch_reg_param_check(struct fj_touch_i2c_data data)
{
	if (data.slaveaddress < 0 ||
		data.slaveaddress > 255) {
		printk("[TPD]%s: slaveaddress=%d\n", __func__, data.slaveaddress);
		return -EINVAL;
	}

	if (data.offset < 0 ||
		data.offset > 65535) {
		printk("[TPD]%s: offset=%d\n", __func__, data.offset);
		return -EINVAL;
	}

	if (data.use_offset < 0 ||
		data.use_offset > 1) {
		printk("[TPD]%s: use_offset=%d\n", __func__, data.use_offset);
		return -EINVAL;
	}

	if (data.length < 0 ||
		data.length > 65535) {
		printk("[TPD]%s: length=%d\n", __func__, data.length);
		return -EINVAL;
	}

	if (data.i2c_data_buf == NULL) {
		printk("[TPD]%s: i2c_data_buf NULL\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int mxt_touch_read_reg_sequence( struct mxt_data *ts, int offset, int length, int slaveaddress, struct fj_touch_i2c_data data)
{
	int ret = 0;
	uint8_t  rdata[length];
	int count;

	__mxt_read_reg(ts->client, offset, sizeof(rdata), &rdata);

	if (MXT_DEBUG_LOG_LEVEL3 <= set_log_level) {
		for(count = 0; count < length; count++ ) {
			printk("[TPD] rdata[%d] = 0x%x \n",count, rdata[count]);
		}
	}

	ret = copy_to_user((int __user *)data.i2c_data_buf, &rdata, sizeof(rdata));
	if (ret != 0) {
		printk(KERN_ERR "copy_to_user err %d \n", ret);
	}

	return ret;
}

static int mxt_touch_write_reg_sequence( struct mxt_data *ts, int offset, int length, int slaveaddress, struct fj_touch_i2c_data data)
{
	int ret = 0;
	uint8_t  wdata[length];
	int count;

	for(count = 0; count < data.length; count++ ){
		wdata[count] = data.i2c_data_buf[count];
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3,
			"[TPD] wdata[%d] = 0x%x \n",count, wdata[count]);
	}

	__mxt_write_reg(ts->client, offset, sizeof(wdata), wdata);

	return ret;
}

static long mxt_touch_ioctl( struct file     *file,
							unsigned int    cmd,
							unsigned long   arg    )
{
	int retval = 0;
	enum devicepowerstatus mode;
	char verdata[MXT_FIRMVERSION_SIZE];
	struct mxt_data *data = file->private_data; 
	const struct mxt_platform_data *pdata = data->pdata;
	struct fj_touch_i2c_data i2c_data;
	struct fj_touch_grip_suppression_data grip_data;
	struct fj_touch_do_selftest_data self_data;
	struct fj_touch_debug_diagnostic_data diag_data;
	struct mxt_message message;
	int cnt = 0;
	bool cal_flag = false;
	int retval_user = 0;
	unsigned int check_crc = *pdata->config_crc;

	memset(&i2c_data, 0, sizeof(i2c_data));
	memset(&grip_data, 0, sizeof(grip_data));
	memset(&self_data, 0, sizeof(self_data));
	memset(&diag_data, 0, sizeof(diag_data));

	switch(cmd) {
	case IOCTL_SET_POWERMODE:
		retval = copy_from_user( &mode, (int __user *) arg, sizeof(int));
		if(retval != 0) {
			printk(KERN_ERR "copy_from_user err %d \n",retval);
			break;
		}
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] mode %d \n", mode);

		switch(mode) {
		case DEEPSLEEP:
			mxt_dbg_log(MXT_DEBUG_LOG_LEVEL2, "[TPD] Enter DEEPSLEEP\n");
			/* supend */
			mxt_suspend(&data->client->dev);
			break;

		/* i2c wakeup */
		case WAKEUP:
			mxt_dbg_log(MXT_DEBUG_LOG_LEVEL2, "[TPD] Enter WAKEUP\n");
			/* resume */
			mxt_resume(&data->client->dev);
			break;

		/* XReset wakeup */
		case HARDRESET:
			mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] Enter HARDRESET\n");
			/* hardreset */
			break;

		/* SoftReset */
		case SOFTRESET:
			mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] Enter SOFTRESET\n");
			if (data->mxt_suspend_flag) {
				printk("[TPD]%s: Cannot SOFTRESET\n", __func__);
				retval = -EFAULT;
				break;
			}
			/* softreset */
			disable_irq(data->irq);
			retval = mxt_soft_reset(data);
			if (retval < 0) {
				printk("[TPD]%s: SoftReset Err retval=%d\n", __func__, retval);
			}
			data->diag_mode = 0x00;
			enable_irq(data->irq);
			break;

		/* TouchEvent Enable */
		case TOUCH_EVENT_ENABLE:
			mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] Enter TOUCH_EVENT_ENABLE\n");
			/* touch event enable */
			data->mxt_touchevent_flag = true;
			break;

		/* TouchEvent Disable */
		case TOUCH_EVENT_DISABLE:
			mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] Enter TOUCH_EVENT_DISABLE\n");
			/* touch event disable */
			data->mxt_touchevent_flag = false;
			break;
		default:
			printk("[TPD]%s: Invalid mode=%d\n", __func__, mode);
			retval = -EINVAL;
		}
		break;

	case IOCTL_GET_FIRMVERSION:
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] Enter FIRMVERSION\n");
		if (data->mxt_suspend_flag) {
			printk("[TPD]%s: Cannot Get FIRMVERSION\n", __func__);
			retval = -EFAULT;
			break;
		}
		memset(verdata, 0xff, sizeof(verdata));

		retval = __mxt_read_reg(data->client, 0x00, sizeof(struct mxt_info), verdata);
		verdata[13] = (check_crc >> 16) & 0xFF;
		verdata[14] = (check_crc >> 8) & 0xFF;
		verdata[15] = check_crc & 0xFF;

		if (retval != 0) {
			printk("[TPD] Get FIRMVERSION Read Err retval=%d\n", retval);
			memset(verdata, 0xff, sizeof(verdata));
		}

		retval_user = copy_to_user( (int __user *) arg, &verdata, sizeof(verdata) );
		if (retval_user != 0) {
			printk(KERN_ERR "copy_to_user err %d \n",retval_user);
			retval = retval_user;
		}
		break;

	case IOCTL_DO_CALIBRATION:
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD]Enter CALIBRATION\n");
		if(data->mxt_suspend_flag) {
			printk("[TPD]%s: Cannot CALIBRATION\n", __func__);
			retval = -EFAULT;
			break;
		}
		/* calibration */
		disable_irq(data->irq);
		mxt_write_object(data, MXT_GEN_COMMAND, MXT_COMMAND_CALIBRATE, 1);
	    for (cnt = 0; cnt < MXT_MAX_INIT_WAIT; cnt += 10) {
			if (pdata->read_chg() == 0) {
				if (mxt_read_message(data, &message) == 0) {
					u8 val = mxt_reportid_to_type(data, message.reportid);
					if ((MXT_GEN_COMMAND == val) && (0x10 & message.message[0])){
						printk("[TPD] Calibrating...\n");
						cal_flag = true;
					} else if ((MXT_GEN_COMMAND == val) && !(0x10 & message.message[0])){
						if(cal_flag) {
							printk("[TPD] Calibrating...Complete\n");
							break;
						}
					}
				}
			}
			msleep( 10 );
	    }
		enable_irq(data->irq);
		if(cnt >= MXT_MAX_INIT_WAIT) {
			printk("[TPD]%s: CALIBRATION Err\n", __func__);
			retval = -EFAULT;
			break;
		}
		data->mxt_crc = ((message.message[3] << 16)&0x00FF0000) | ((message.message[2] << 8)&0x0000FF00) | message.message[1];
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, 
			"[TPD] Get CRC=0x%X\n", data->mxt_crc);
		break;

	case IOCTL_I2C_READ:
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] Enter IOCTL_I2C_READ\n");
		if(data->mxt_suspend_flag) {
			printk("[TPD]%s: Cannot I2C READ\n", __func__);
			retval = -EFAULT;
			break;
		}

		retval = copy_from_user( &i2c_data, (int __user *) arg, sizeof(i2c_data));
		if(retval != 0) {
			printk(KERN_ERR "copy_from_user err %d \n",retval);
			break;
		}

		retval = mxt_touch_reg_param_check(i2c_data);
		if (retval < 0) {
			printk("[TPD]%s: I2C Read Data Invalid Err retval=%d\n", __func__, retval);
			break;
		}

		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3,
			"[TPD] i2cdata offset 0x%x length 0x%x slave 0x%x\n",
			i2c_data.offset, i2c_data.length, i2c_data.slaveaddress);

		retval = mxt_touch_read_reg_sequence( data, i2c_data.offset, i2c_data.length, i2c_data.slaveaddress, i2c_data );

		if(retval != 0) {
			printk(KERN_ERR "mxt_touch_read_reg_sequence err %d \n",retval);
			retval = -EIO;
			break;
		}

		break;

	case IOCTL_I2C_WRITE:
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] Enter IOCTL_I2C_WRITE\n");
		if(data->mxt_suspend_flag) {
			printk("[TPD]%s: Cannot I2C WRITE\n", __func__);
			retval = -EFAULT;
			break;
		}

		retval = copy_from_user( &i2c_data, (int __user *) arg, sizeof(i2c_data));
		if(retval != 0) {
			printk(KERN_ERR "copy_from_user err %d \n",retval);
			break;
		}

		retval = mxt_touch_reg_param_check(i2c_data);
		if (retval < 0) {
			printk("[TPD]%s: I2C Write Data Invalid Err retval=%d\n", __func__, retval);
			break;
		}

		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3,
			"[TPD] i2cdata offset 0x%x length 0x%x slave 0x%x\n",
			i2c_data.offset, i2c_data.length, i2c_data.slaveaddress);

		retval = mxt_touch_write_reg_sequence( data, i2c_data.offset, i2c_data.length, i2c_data.slaveaddress, i2c_data );
		if(retval != 0) {
			printk(KERN_ERR "mxt_touch_write_reg_sequence err %d \n",retval);
			retval = -EIO;
			break;
		}
		break;

	case IOCTL_SET_GRIPSUPPRESSION:
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] Enter IOCTL_SET_GRIPSUPPRESSION\n");
		if (data->mxt_suspend_flag) {
			printk("[TPD]%s: Cannot Set Gripsuppression\n", __func__);
			retval = -EFAULT;
			break;
		}

		retval = copy_from_user( &grip_data, (int __user *) arg, sizeof(grip_data));
		if (retval != 0) {
			printk(KERN_ERR "copy_from_user err %d \n",retval);
			break;
		}

		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3,
			"[TPD] grip_data use:%s mode:%s x_lo:%03d x_hi:%03d y_lo:%03d y_hi:%03d\n",
			grip_data.use_grip ? "ON" : "OFF",
			grip_data.grip_mode ? "NoLook" : "Look",
			grip_data.x_lo_grip,
			grip_data.x_hi_grip,
			grip_data.y_lo_grip,
			grip_data.y_hi_grip);

		retval = mxt_grip_suppression(data, grip_data);
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3,
			"[TPD] mxt_grip_suppression retval=%d\n", retval);
		break;

	case IOCTL_DO_SELFTEST:
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] Enter IOCTL_DO_SELFTEST\n");
		if (data->mxt_suspend_flag) {
			printk("[TPD]%s: Cannot SELFTEST\n", __func__);
			retval = -EFAULT;
			break;
		}

		retval = copy_from_user( &self_data, (int __user *) arg, sizeof(self_data));
		if (retval != 0) {
			printk(KERN_ERR "copy_from_user err %d \n",retval);
			break;
		}

		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3,
			"[TPD] self_data cmd:0x%X low_limit:%d high_limit:%d\n",
			self_data.test_cmd,
			self_data.low_signal_limit,
			self_data.high_signal_limit);

		disable_irq(data->irq);
		retval = mxt_selftest(data, self_data);
		enable_irq(data->irq);

		if (retval < 0) {
			printk("[TPD] SELFTEST Err retval=%d\n", retval);
		}

		break;

	case IOCTL_DEBUG_DIAGNOSTIC:
		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3, "[TPD] Enter IOCTL_DEBUG_DIAGNOSTIC\n");
		if (data->mxt_suspend_flag) {
			printk("[TPD]%s: Cannot DEBUG DIAGNOSTIC\n", __func__);
			retval = -EFAULT;
			break;
		}

		retval = copy_from_user( &diag_data, (int __user *) arg, sizeof(diag_data));
		if (retval != 0) {
			printk(KERN_ERR "copy_from_user err %d \n",retval);
			break;
		}

		mxt_dbg_log(MXT_DEBUG_LOG_LEVEL3,
			"[TPD] diag_data cmd:0x%02X\n",
			diag_data.test_cmd)

		disable_irq(data->irq);
		retval = mxt_diagnostic(data, diag_data);
		enable_irq(data->irq);

		if (retval < 0) {
			printk("[TPD] DEBUG DIAGNOSTIC Err retval=%d\n", retval);
		}

		break;

	default:
		break;
	}

    return retval;
}

static int mxt_touch_ioctl_open( struct inode *inode, struct file *file )
{
	struct mxt_data *data = container_of( inode->i_cdev,struct mxt_data, mxt_touch_cdev );

	printk( KERN_INFO "%s :[IN]\n", __func__ );
	file->private_data = data;
	return 0;
}

static int mxt_touch_ioctl_release(struct inode *inode, struct file *file)
{
	printk( KERN_INFO "%s :[IN]\n", __func__ );
	return 0;
}


static struct file_operations mxt_touch_fops = {
	.owner          = THIS_MODULE,
	.release        = mxt_touch_ioctl_release,
	.open           = mxt_touch_ioctl_open,
	.unlocked_ioctl = mxt_touch_ioctl,
};
#endif
/* FUJITSU:2012-04-09 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static int __devinit mxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct mxt_platform_data *pdata = client->dev.platform_data;
	struct mxt_data *data;
	struct input_dev *input_dev;
	int error;

	if (!pdata)
		return -EINVAL;

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	input_dev->name = "Atmel maXTouch Touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	data->client = client;
	data->input_dev = input_dev;
	data->pdata = pdata;
	data->irq = client->irq;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->pdata->x_size, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->pdata->y_size, 0, 0);

	/* For multi touch */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->pdata->x_size, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->pdata->y_size, 0, 0);

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	if (pdata->init_hw)
		error = pdata->init_hw(true);
	else
		error = mxt_regulator_configure(data, true);
	if (error) {
		dev_err(&client->dev, "Failed to intialize hardware\n");
		goto err_free_mem;
	}

	if (pdata->power_on)
		error = pdata->power_on(true);
	else
		error = mxt_power_on(data, true);
	if (error) {
		dev_err(&client->dev, "Failed to power on hardware\n");
		goto err_regulator_on;
	}

	error = mxt_initialize(data);
	if (error)
		goto err_power_on;

	error = request_threaded_irq(client->irq, NULL, mxt_interrupt,
			pdata->irqflags, client->dev.driver->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}

	error = mxt_make_highchg(data);
	if (error)
		goto err_free_irq;

	error = input_register_device(input_dev);
	if (error)
		goto err_free_irq;

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error)
		goto err_unregister_device;

#if defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						MXT_SUSPEND_LEVEL;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_object:
	kfree(data->object_table);
err_power_on:
	if (pdata->power_on)
		pdata->power_on(false);
	else
		mxt_power_on(data, false);
err_regulator_on:
	if (pdata->init_hw)
		pdata->init_hw(false);
	else
		mxt_regulator_configure(data, false);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return error;
}
#else //MXT_ENABLE_ORIGINAL_FUNCTION
static int __devinit mxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct mxt_platform_data *pdata = client->dev.platform_data;
	struct mxt_data *data;
	struct input_dev *input_dev;
	int error = 0;
	dev_t dev_ioctl;
	int retval = 0;
	int devno;
	int config_rev = 0;

	if (!pdata)
		return -EINVAL;

	printk("[TPD]%s: probe start...\n", __func__);
	set_log_level = 0;

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	dev_ioctl = MKDEV(cdev_major, 0);
	udev_class = class_create(THIS_MODULE, "fj_touch");

	retval = alloc_chrdev_region(&dev_ioctl, 0, 1, "fj_touch");
	cdev_major = MAJOR(dev_ioctl);
	if (cdev_major == 0) {
		cdev_major = retval;
	}

	devno = MKDEV( cdev_major, 0 ); 
	cdev_init( &(data->mxt_touch_cdev), &mxt_touch_fops );
	data->mxt_touch_cdev.owner = THIS_MODULE;
	data->mxt_touch_cdev.ops = &mxt_touch_fops;
	retval = cdev_add ( &(data->mxt_touch_cdev), devno, 1 );

	device_create(udev_class, NULL, devno, NULL, "fj_touch");

	input_dev->name = MXT_DEVICE_ID;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	data->client = client;
	data->input_dev = input_dev;
	data->pdata = pdata;

	config_rev = pdata->get_config_rev();

	if (pdata->init_hw)
		error = pdata->init_hw(true);

	data->irq = client->irq;

	mutex_init(&data->data_lock);
	data->mxt_chrgon_flag = false;
	data->mxt_suspend_flag = false;
	data->mxt_touchevent_flag = true;
	data->mxt_irq_cnt = 0;
	data->mxt_crc = 0x00;
	data->mxt_batt_status = 0x00;

	mxt_calc_resolution(data);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	/* DeviceType is TouchScreen */
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	/* For multi touch */
	/* multi-touch-protocol B */
	input_mt_init_slots(input_dev, MXT_MAX_FINGER);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, 30, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
			     0, 255, 0, 0);

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	if (pdata->power_on)
		error = pdata->power_on(true);

	if (error) {
		dev_err(&client->dev, "Failed to power on hardware\n");
		goto err_power_on;
	}

	error = mxt_initialize(data);
	if (error)
		goto err_power_on;

	error = request_threaded_irq(client->irq, NULL, mxt_interrupt,
			pdata->irqflags, client->dev.driver->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}

	error = mxt_make_highchg(data);
	if (error)
		goto err_free_irq;

	error = input_register_device(input_dev);
	if (error)
		goto err_free_irq;

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error)
		goto err_unregister_device;

#if defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						MXT_SUSPEND_LEVEL;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	data->battery_callback.callback = mxt_battery_info;
	data->battery_callback.data = data;
	set_msm_battery_callback_info(&data->battery_callback);

	printk("[TPD]%s: probe complete (Built %s @ %s) config_rev %d\n",
		__func__, __DATE__, __TIME__, config_rev);
	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_object:
	kfree(data->object_table);
err_power_on:
	if (pdata->power_on)
		pdata->power_on(false);
	else
		mxt_power_on(data, false);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return error;
}
#endif
/* FUJITSU:2012-04-03 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif

	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		mxt_power_on(data, false);

	if (data->pdata->init_hw)
		data->pdata->init_hw(false);
	else
		mxt_regulator_configure(data, false);

	kfree(data->object_table);
	kfree(data);

	return 0;
}
#else //MXT_ENABLE_ORIGINAL_FUNCTION
static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif

	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		mxt_power_on(data, false);

	kfree(data->object_table);
	kfree(data);

	return 0;
}
#endif
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION end */

/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "mXT224", 0 },
	{ }
};
#else //MXT_ENABLE_ORIGINAL_FUNCTION
static const struct i2c_device_id mxt_id[] = {
	{ MXT_DEVICE_ID, 4 },
	{ }
};
#endif
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION end */
MODULE_DEVICE_TABLE(i2c, mxt_id);

/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION start */
#if MXT_ENABLE_ORIGINAL_FUNCTION
static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &mxt_pm_ops,
#endif
	},
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.id_table	= mxt_id,
};
#else //MXT_ENABLE_ORIGINAL_FUNCTION
static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= MXT_DEVICE_ID,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &mxt_pm_ops,
#endif
	},
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.id_table	= mxt_id,
};
#endif
/* FUJITSU:2012-01-16 MXT_ENABLE_ORIGINAL_FUNCTION end */

static int __init mxt_init(void)
{
	return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
	i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
