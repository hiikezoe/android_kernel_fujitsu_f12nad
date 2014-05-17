/*
 * Core Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) touchscreen drivers.
 * For use with Cypress Txx3xx parts.
 * Supported parts include:
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009-2011 Cypress Semiconductor, Inc.
 * Copyright (C) 2010-2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

/* FUJITSU:2011-09-27 del start */
//#include "cyttsp3_core.h"
/* FUJITSU:2011-09-27 del end */
/* FUJITSU:2011-08-11 add start */
#include "cyttsp_core.h"
#include "cyttsp_if.h"
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/time.h>
#include <asm/uaccess.h>
/* FUJITSU:2011-08-11 add end */

#include <linux/delay.h>
/* FUJITSU:2011-12-27 mod start */
//#include <linux/input.h>
#include <linux/input/mt.h>
/* FUJITSU:2011-12-27 mod end */
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input/touch_platform.h>
#include <linux/version.h>	/* Required for kernel version checking */
#include <linux/firmware.h>	/* This enables firmware class loader code */

#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
#include <linux/workqueue.h>
#endif

#define CY_USE_HW_RESET

/* Soft reset here is for platforms with no hw reset line */
#ifdef CY_USE_HW_RESET
#define _cyttsp_soft_reset(ts)
#else
#define _cyttsp_soft_reset(ts) {\
	int rc;\
	u8 cmd = CY_SOFT_RESET_MODE;\
	rc = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);\
	if (rc < 0)\
		return rc;\
}
#endif

/* Bootloader File 0 offset */
#define CY_BL_FILE0       0x00
/* Bootloader command directive */
#define CY_BL_CMD         0xFF
/* Bootloader Exit and Verify Checksum command */
#define CY_BL_EXIT        0xA5
/* Bootloader number of command keys */
#define CY_NUM_BL_KEYS    8
/* Bootloader default command keys */
/* FUJITSU:2011-08-11 mod start */
#if 0  /* FUJITSU:2011-08-31 del start */
#define CY_BL_KEY0 0
#define CY_BL_KEY1 1
#define CY_BL_KEY2 2
#define CY_BL_KEY3 3
#define CY_BL_KEY4 4
#define CY_BL_KEY5 5
#define CY_BL_KEY6 6
#define CY_BL_KEY7 7
#endif /* FUJITSU:2011-08-31 del end */
#define CY_BL_KEY0 0xAA
#define CY_BL_KEY1 0x55
#define CY_BL_KEY2 0x33
#define CY_BL_KEY3 0x68
#define CY_BL_KEY4 0x98
#define CY_BL_KEY5 0x0B
#define CY_BL_KEY6 0x1D
#define CY_BL_KEY7 0xAC
/* FUJITSU:2011-08-11 mod end */

/* helpers */
#define GET_NUM_TOUCHES(x)          ((x) & 0x0F)
#define IS_LARGE_AREA(x)            (((x) & 0x10) >> 4)
#define IS_BAD_PKT(x)               ((x) & 0x20)
#define IS_VALID_APP(x)             ((x) & 0x01)
#define IS_OPERATIONAL_ERR(x)       ((x) & 0x3F)
#define GET_HSTMODE(reg)            ((reg & 0x70) >> 4)
#define GET_BOOTLOADERMODE(reg)     ((reg & 0x10) >> 4)
#define BL_WATCHDOG_DETECT(reg)     (reg & 0x02)

/* maximum number of concurrent tracks */
#define CY_NUM_TCH_ID               4
/* maximum number of track IDs */
#define CY_NUM_TRK_ID               16

#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
/*
 * maximum number of touch reports with
 * current touches=0 before performing Driver reset
 */
#define CY_MAX_NTCH                 10
#endif

#define CY_NTCH                     0 /* lift off */
#define CY_TCH                      1 /* touch down */
#define CY_SMALL_TOOL_WIDTH         10
#define CY_LARGE_TOOL_WIDTH         255
#define CY_REG_BASE                 0x00
#define CY_REG_OP_START             0x1B
#define CY_REG_OP_END               0x1F
#define CY_REG_SI_START             0x17
#define CY_REG_SI_END               0x1F
#define CY_REG_ACT_DIST             0x1E
#define CY_REG_ACT_INTRVL           0x1D
#define CY_REG_TCH_TMOUT            (CY_REG_ACT_INTRVL+1)
#define CY_REG_LP_INTRVL            (CY_REG_TCH_TMOUT+1)
#define CY_RW_REGID_MAX             0x1F
#define CY_RW_REG_DATA_MAX          0xFF
#define CY_MAXZ                     255
#define CY_DELAY_DFLT               20 /* ms */
#define CY_DELAY_MAX                (500/CY_DELAY_DFLT) /* half second */
#define CY_ACT_DIST_DFLT            0xF8
#define CY_ACT_DIST_BITS            0x0F
#define CY_HNDSHK_BIT               0x80
#define CY_HST_MODE_CHANGE_BIT      0x08
/* device mode bits */
#define CY_OPERATE_MODE             0x00
#define CY_SYSINFO_MODE             0x10
/* power mode select bits */
#define CY_SOFT_RESET_MODE          0x01 /* return to Bootloader mode */
#define CY_DEEP_SLEEP_MODE          0x02
#define CY_LOW_POWER_MODE           0x04
/* abs settings */
#define CY_NUM_ABS_SET              5 /* number of abs values per setting */
/* abs value offset */
#define CY_SIGNAL_OST               0
#define CY_MIN_OST                  1
#define CY_MAX_OST                  2
#define CY_FUZZ_OST                 3
#define CY_FLAT_OST                 4
/* axis signal offset */
#define CY_ABS_X_OST                0
#define CY_ABS_Y_OST                1
#define CY_ABS_P_OST                2
#define CY_ABS_W_OST                3
#define CY_ABS_ID_OST               4

#define HI_TRACKID(reg)        ((reg & 0xF0) >> 4)
#define LO_TRACKID(reg)        ((reg & 0x0F) >> 0)

#define CY_BL_PAGE_SIZE		16
#define CY_BL_NUM_PAGES		5
#define CY_BL_BUSY		0x80
#define CY_BL_READY_NO_APP	0x10
#define CY_BL_READY_APP		0x11
#define CY_BL_RUNNING		0x20
#define CY_BL_MAX_DATA_LEN	(CY_BL_PAGE_SIZE * 2)
#define CY_BL_ENTER_CMD_SIZE	11
#define CY_BL_EXIT_CMD_SIZE	11
#define CY_BL_WR_BLK_CMD_SIZE	79
#define CY_BL_VERS_SIZE		9
#define CY_BL_FW_IMG_SIZE	37152
#define CY_BL_FW_NAME_SIZE	NAME_MAX
#define CY_MAX_PRBUF_SIZE	PIPE_BUF
#define CY_IRQ_DEASSERT		1
#define CY_IRQ_ASSERT		0

/* FUJITSU:2012-05-14 add start */
#define GET_STYLUS_OBJECT(reg)     ((reg & 0x20) >> 5)

#define CY_REG_MFG_STAT             0x01
#define CY_REG_MFG_CMD              0x02
#define CY_REG_CID_0                0x03
#define CY_REG_CID_1                0x04

#define CY_I2C_NACK_WAIT_VALUE      1
#define CY_FIRMVERSION_SIZE         16
#define CY_UP_JUDGE_TIME            msecs_to_jiffies(55)
#define CY_MOVE_JUDGE_TIME          msecs_to_jiffies(70)
#define CY_MOVE_JUDGE_AREA          120
/* FUJITSU:2012-05-14 add end */

struct cyttsp_vers {
	u8 tts_verh;
	u8 tts_verl;
	u8 app_idh;
	u8 app_idl;
	u8 app_verh;
	u8 app_verl;
	u8 cid[3];
};

struct cyttsp_bin_image {
	u8 vers_size;
	struct cyttsp_vers vers;
	u8 image[CY_BL_FW_IMG_SIZE];
};

enum cyttsp_sett_flags {
	CY_USE_HNDSHK = 0x01,
	CY_USE_SLEEP = 0x02,
	CY_FORCE_LOAD = 0x04,
#ifdef CY_USE_DEBUG_TOOLS
	CY_FLIP = 0x08,
	CY_INV_X = 0x10,
	CY_INV_Y = 0x20,
#endif
};

enum cyttsp_powerstate {
	CY_IDLE_STATE,		/* IC cannot be reached */
	CY_READY_STATE,		/* pre-operational; ready to go to ACTIVE */
	CY_ACTIVE_STATE,	/* app is running, IC is scanning */
	CY_LOW_PWR_STATE,	/* not currently used  */
	CY_SLEEP_STATE,		/* app is running, IC is idle */
	CY_BL_STATE,		/* bootloader is running */
	CY_LDR_STATE,		/* loader is running */
	CY_SYSINFO_STATE,	/* switching to sysinfo mode */
	CY_INVALID_STATE	/* always last in the list */
};

static char *cyttsp_powerstate_string[] = {
	/* Order must match enum cyttsp_powerstate above */
	"IDLE",
	"READY",
	"ACTIVE",
	"LOW_PWR",
	"SLEEP",
	"BOOTLOADER",
	"LOADER",
	"SYSINFO",
	"INVALID"
};

enum cyttsp_ic_grpnum {
	CY_IC_GRPNUM_RESERVED = 0,
	CY_IC_GRPNUM_OP,
	CY_IC_GRPNUM_SI,
	CY_IC_GRPNUM_BL,
	CY_IC_GRPNUM_NUM	/* always last */
};

enum cyttsp_op_registers {
	CY_OP_REG_RESERVED = 0,
	CY_OP_REG_ACT_DIST,
};

enum cyttsp_si_registers {
	CY_SI_REG_RESERVED = 0,
	CY_SI_REG_ACT_INTRVL,
	CY_SI_REG_TCH_TMOUT,
	CY_SI_REG_LP_INTRVL,
};

/* Touch structure */
struct cyttsp_touch {
	__be16 x;
	__be16 y;
	u8 z;
} __attribute__((packed));

/* TrueTouch Standard Product Gen3 interface definition */
struct cyttsp_xydata {
	u8 hst_mode;
	u8 tt_mode;
	u8 tt_stat;
	struct cyttsp_touch tch1;
	u8 touch12_id;
	struct cyttsp_touch tch2;
	u8 gest_cnt;
	u8 gest_id;
	struct cyttsp_touch tch3;
	u8 touch34_id;
	struct cyttsp_touch tch4;
	u8 tt_undef[3];
	u8 act_dist;
	u8 tt_reserved;
} __attribute__((packed));

/* TTSP System Information interface definition */
struct cyttsp_sysinfo_data {
	u8 hst_mode;
	u8 mfg_cmd;
	u8 mfg_stat;
	u8 cid[3];
	u8 tt_undef1;
	u8 uid[8];
	u8 bl_verh;
	u8 bl_verl;
	u8 tts_verh;
	u8 tts_verl;
	u8 app_idh;
	u8 app_idl;
	u8 app_verh;
	u8 app_verl;
	u8 tt_undef[5];
	u8 scn_typ;
	u8 act_intrvl;
	u8 tch_tmout;
	u8 lp_intrvl;
	/* FUJITSU:2011-08-11 add start */
	u8 tt_undef2;
	u8 pump_freg;
	u8 imo_trm;
	u8 setting;
	u8 large_th;
	u8 large_num;
	u8 iir_reset;
	u8 iir_weight1;
	u8 weight_change34;
	u8 tt_undef3;
	u8 iir_weight2;
	u8 weight_change23;
	u8 weight_change12;
	u8 tt_undef4[3];
	u8 fth;
	u8 nth;
	u8 pre;
	u8 nsub;
	u8 nsub_shift;
	u8 ncycle;
	u8 self_fth;
	u8 self_pre;
	u8 self_nsub;
	u8 self_shift;
	u8 self_ncycle;
	u8 self_th_mut_reset;
	u8 mut_reset_delay;
	u8 mut_th_self_reset;
	u8 self_reset_delay;
	u8 tt_undef5;
	u8 stylus_to_finger_th;
	u8 stylus_noise_th;
	u8 stylus_touch_th;
	u8 stylus_hyst_th;
	u8 stylus_coef;
	u8 stylus_prescaler;
	u8 stylus_nsub;
	u8 stylus_nshift;
	u8 stylus_ncycles;
	u8 stylus_adc_resolution;
	u8 stylus_reset_delay;
	/* FUJITSU:2011-08-11 add end */
};

/* TTSP Bootloader Register Map interface definition */
#define CY_BL_CHKSUM_OK 0x01
struct cyttsp_bootloader_data {
	u8 bl_file;
	u8 bl_status;
	u8 bl_error;
	u8 blver_hi;
	u8 blver_lo;
	u8 bld_blver_hi;
	u8 bld_blver_lo;
	u8 ttspver_hi;
	u8 ttspver_lo;
	u8 appid_hi;
	u8 appid_lo;
	u8 appver_hi;
	u8 appver_lo;
	u8 cid_0;
	u8 cid_1;
	u8 cid_2;
};

struct cyttsp_tch {
	struct cyttsp_touch *tch;
	u8 *id;
};

struct cyttsp_trk {
	bool tch;
	int abs[CY_NUM_ABS_SET];
};

struct cyttsp {
	struct device *dev;
	int irq;
	struct input_dev *input;
	struct mutex data_lock;		/* Used to prevent concurrent access */
	struct mutex startup_mutex;	/* protect power on sequence */
	char phys[32];
	const struct bus_type *bus_type;
	const struct touch_platform_data *platform_data;
	struct cyttsp_bus_ops *bus_ops;
	struct cyttsp_xydata xy_data;
	struct cyttsp_bootloader_data bl_data;
	struct cyttsp_sysinfo_data sysinfo_data;
	struct cyttsp_trk prv_trk[CY_NUM_TRK_ID];
	struct cyttsp_tch tch_map[CY_NUM_TCH_ID];
	struct completion bl_int_running;
	struct completion si_int_running;
	bool bl_ready_flag;
	enum cyttsp_powerstate power_state;
	bool irq_enabled;
	bool waiting_for_fw;
	bool powered;
	char *fwname;
	u16 flags;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
	struct work_struct work;
	struct timer_list timer;
	int ntch_count;
	int prv_tch;
#endif
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	int ic_grpnum;
	int ic_grpoffset;
	int ic_grpstart[CY_IC_GRPNUM_NUM];
#endif
#ifdef CY_USE_REG_ACCESS
	u8 rw_regid;
#endif
/* FUJITSU:2012-05-14 add start */
	struct cdev cyttsp_cdev;
	int dev_id;
	int mt_trc_id[CY_NUM_TCH_ID];
	struct timer_list timer_up;
	unsigned long first_down_time;
	bool timer_up_flag;
	int jdg_area_size;
};

static int cdev_major = 0;
static struct class* udev_class;
static void cyttsp_calibration(struct cyttsp *ts);
/* FUJITSU:2012-05-14 add end */

struct cyttsp_track_data {
	struct cyttsp_trk cur_trk[CY_NUM_TRK_ID];
};

static const u8 bl_command[] = {
	CY_BL_FILE0, CY_BL_CMD, CY_BL_EXIT,
	CY_BL_KEY0, CY_BL_KEY1, CY_BL_KEY2,
	CY_BL_KEY3, CY_BL_KEY4, CY_BL_KEY5,
	CY_BL_KEY6, CY_BL_KEY7
};

static int _cyttsp_startup(struct cyttsp *ts);

static void cyttsp_pr_state(struct cyttsp *ts)
{
	pr_info("%s: %s\n", __func__,
		ts->power_state < CY_INVALID_STATE ?
		cyttsp_powerstate_string[ts->power_state] :
		"INVALID");
}

static int ttsp_read_block_data(struct cyttsp *ts, u8 command,
	u8 length, void *buf)
{
	int retval;
	int tries;
    /* FUJITSU:2011-08-11 add start */
	u32 i2c_param = 0x00;
    /* FUJITSU:2011-08-11 add end */

	if (!buf || !length)
		return -EIO;

	for (tries = 0, retval = -1;
		(tries < CY_NUM_RETRY) && (retval < 0);
		tries++) {
		retval = ts->bus_ops->read(ts->bus_ops, command, length, buf);
        /* FUJITSU:2011-08-11 mod start */
		if (retval) {
			i2c_param = ~retval + 0x01;
			if(tries == 0 && (i2c_param & (1U << 3))) {
				cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: I2C Read NACK Err Retry \n", __func__);
				mdelay(CY_I2C_NACK_WAIT_VALUE);
			} else {
				cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: I2C Read Err Retry \n", __func__);
				msleep(CY_DELAY_DFLT);
			}
		}
        /* FUJITSU:2011-08-11 mod end */
	}

	if (retval < 0) {
		pr_err("%s: bus read block data fail (ret=%d)\n",
			__func__, retval);
	}
	return retval;
}

static int ttsp_write_block_data(struct cyttsp *ts, u8 command,
	u8 length, const void *buf)
{
	int retval;
	int tries;

	if (!buf || !length)
		return -EIO;

	for (tries = 0, retval = -1;
		(tries < CY_NUM_RETRY) && (retval < 0);
		tries++) {
		retval = ts->bus_ops->write(ts->bus_ops, command, length, buf);
		if (retval)
			msleep(CY_DELAY_DFLT);
	}

	if (retval < 0) {
		pr_err("%s: bus write block data fail (ret=%d)\n",
			__func__, retval);
	}
	return retval;
}

static int _cyttsp_hndshk(struct cyttsp *ts, u8 hst_mode)
{
	int retval;
	u8 cmd;

	cmd = hst_mode & CY_HNDSHK_BIT ?
		hst_mode & ~CY_HNDSHK_BIT :
		hst_mode | CY_HNDSHK_BIT;

	retval = ttsp_write_block_data(ts, CY_REG_BASE,
		sizeof(cmd), (u8 *)&cmd);

	if (retval < 0) {
		pr_err("%s: bus write fail on handshake (ret=%d)\n",
			__func__, retval);
	}

	return retval;
}

static int _cyttsp_load_bl_regs(struct cyttsp *ts)
{
	int retval;

	memset(&(ts->bl_data), 0, sizeof(struct cyttsp_bootloader_data));

	retval = ttsp_read_block_data(ts, CY_REG_BASE,
		sizeof(ts->bl_data), &(ts->bl_data));

	if (retval < 0) {
		pr_err("%s: bus fail reading Bootloader regs (ret=%d)\n",
			__func__, retval);
		/*
		 * Calling process determines state change requirement
		 */
		goto cyttsp_load_bl_regs_exit;
	}

	if (GET_BOOTLOADERMODE(ts->bl_data.bl_status)) {
		cyttsp_dbg(ts, CY_DBG_LVL_2,
			"%s: Bootloader Regs:\n"
			"  file=%02X status=%02X error=%02X\n"
			"  BL Version:          0x%02X%02X\n"
			"  Build BL Version:    0x%02X%02X\n"
			"  TTSP Version:        0x%02X%02X\n"
			"  Application ID:      0x%02X%02X\n"
			"  Application Version: 0x%02X%02X\n"
			"  Custom ID:           0x%02X%02X%02X\n",
			__func__,
			ts->bl_data.bl_file, ts->bl_data.bl_status,
			ts->bl_data.bl_error,
			ts->bl_data.blver_hi, ts->bl_data.blver_lo,
			ts->bl_data.bld_blver_hi, ts->bl_data.bld_blver_lo,
			ts->bl_data.ttspver_hi, ts->bl_data.ttspver_lo,
			ts->bl_data.appid_hi, ts->bl_data.appid_lo,
			ts->bl_data.appver_hi, ts->bl_data.appver_lo,
			ts->bl_data.cid_0, ts->bl_data.cid_1,
			ts->bl_data.cid_2);
	} else {
		cyttsp_dbg(ts, CY_DBG_LVL_2,
			"%s: Not Bootloader mode:\n"
			"  mode=%02X status=%02X error=%02X\n",
			__func__,
			ts->bl_data.bl_file, ts->bl_data.bl_status,
			ts->bl_data.bl_error);
	}

cyttsp_load_bl_regs_exit:
	return retval;
}

static int _cyttsp_bl_app_valid(struct cyttsp *ts)
{
	int retval;

	retval = _cyttsp_load_bl_regs(ts);
	if (retval < 0) {
		pr_err("%s: bus fail on bl regs read\n", __func__);
		ts->power_state = CY_IDLE_STATE;
		cyttsp_pr_state(ts);
		retval = -ENODEV;
		goto cyttsp_bl_app_valid_exit;
	}

	if (GET_BOOTLOADERMODE(ts->bl_data.bl_status)) {
		ts->power_state = CY_BL_STATE;
		if (IS_VALID_APP(ts->bl_data.bl_status)) {
			pr_info("%s: App found; normal boot\n", __func__);
			retval = 0;
			goto cyttsp_bl_app_valid_exit;
		} else {
			/*
			 * The bootloader is running, but the app firmware
			 * is invalid.  Keep the state as Bootloader and
			 * let the loader try to update the firmware
			 */
			pr_err("%s: NO APP; load firmware!!\n", __func__);
			retval = 0;
			goto cyttsp_bl_app_valid_exit;
		}
	} else if (GET_HSTMODE(ts->bl_data.bl_file) == CY_OPERATE_MODE) {
		ts->power_state = CY_ACTIVE_STATE;
		/* No bootloader found in the firmware */
		if (!(IS_OPERATIONAL_ERR(ts->bl_data.bl_status))) {
			/* go directly to Operational Active status */
			pr_info("%s: Operational\n", __func__);
			retval = 0;
			goto cyttsp_bl_app_valid_exit;
		} else {
			/*
			 * Operational error
			 * Continue operation in Active status
			 */
			pr_err("%s:  Operational failure\n", __func__);
			retval = -ENODEV;
			goto cyttsp_bl_app_valid_exit;
		}
	} else {
		/*
		 * General failure of the device
		 * Cannot operate in any status
		 */
		pr_err("%s: Unknown system failure\n", __func__);
		ts->power_state = CY_INVALID_STATE;
		cyttsp_pr_state(ts);
		retval = -ENODEV;
		goto cyttsp_bl_app_valid_exit;
	}

cyttsp_bl_app_valid_exit:
	return retval;
}

static int _cyttsp_exit_bl_mode(struct cyttsp *ts)
{
	int retval;
	int tries = 0;
	u8 bl_cmd[sizeof(bl_command)];

	memcpy(bl_cmd, bl_command, sizeof(bl_command));
#if 0  /* FUJITSU:2011-08-12 del start */
	if (ts->platform_data->sett[CY_IC_GRPNUM_BL]->data)
		memcpy(&bl_cmd[sizeof(bl_command) -
			ts->platform_data->sett[CY_IC_GRPNUM_BL]->size],
			ts->platform_data->sett[CY_IC_GRPNUM_BL]->data,
			ts->platform_data->sett[CY_IC_GRPNUM_BL]->size);
#endif /* FUJITSU:2011-08-12 del end */
	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: bl_cmd= "
		"%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
		__func__, bl_cmd[0], bl_cmd[1], bl_cmd[2],
		bl_cmd[3], bl_cmd[4], bl_cmd[5], bl_cmd[6],
		bl_cmd[7], bl_cmd[8], bl_cmd[9], bl_cmd[10]);

	retval = ttsp_write_block_data(ts, CY_REG_BASE,
		sizeof(bl_cmd), (void *)bl_cmd);
	if (retval < 0) {
		pr_err("%s: bus write fail exit Bootloader mode (ret=%d)\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp_pr_state(ts);
		return retval;
	}

	/* wait for TTSP Device to complete switch to Operational mode */
	tries = 0;
	do {
		/* FUJITSU:2011-08-12 mod start */
//		msleep(CY_DELAY_DFLT);
		msleep(60);
		/* FUJITSU:2011-08-12 mod end */
		retval = _cyttsp_load_bl_regs(ts);
	} while (!((retval == 0) &&
		!GET_BOOTLOADERMODE(ts->bl_data.bl_status)) &&
		(tries++ < CY_DELAY_MAX));

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: check bl ready tries=%d ret=%d stat=%02X\n",
		__func__, tries, retval, ts->bl_data.bl_status);

	if (tries >= CY_DELAY_MAX) {
		pr_err("%s: operational ready fail on tries>=%d ret=%d\n",
			__func__, CY_DELAY_MAX, retval);
		if (retval < 0) {
			pr_err("%s: bus fail exiting bootload\n", __func__);
			ts->power_state = CY_IDLE_STATE;
			cyttsp_pr_state(ts);
		} else {
			pr_err("%s: Missing App: cannot exit bootloader\n",
				__func__);
			ts->power_state = CY_BL_STATE;
			cyttsp_pr_state(ts);

			/* this is a soft failure */
			retval = 0;
		}
	} else {
		ts->power_state = CY_READY_STATE;
		cyttsp_pr_state(ts);
		retval = 0;
	}

	return retval;
}

static int _cyttsp_set_operational_mode(struct cyttsp *ts)
{
	int retval;
	/* FUJITSU:2011-10-02 mod start */
//	int tries;
//	u8 cmd = CY_OPERATE_MODE + CY_HST_MODE_CHANGE_BIT;
	u8 cmd = CY_OPERATE_MODE + CY_LOW_POWER_MODE;
	/* FUJITSU:2011-10-02 mod end */

	/* FUJITSU:2011-10-02 add start */
	cmd = cmd | (ts->sysinfo_data.hst_mode & CY_HNDSHK_BIT);
	/* FUJITSU:2011-10-02 add end */

	/* wait for interrupt to set ready completion */
	ts->power_state = CY_SYSINFO_STATE;
	/* FUJITSU:2011-10-02 del start */
//	INIT_COMPLETION(ts->si_int_running);
	/* FUJITSU:2011-10-02 del end */
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	if (retval < 0) {
		pr_err("%s: I2C write fail set Operational mode (ret=%d)\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp_pr_state(ts);
		return retval;
	}

	/* FUJITSU:2011-10-02 add start */
	ts->power_state = CY_ACTIVE_STATE;
	cyttsp_pr_state(ts);
	/* FUJITSU:2011-10-02 add end */

#if 0  /* FUJITSU:2011-10-02 del start */
	if (mutex_is_locked(&ts->data_lock)) {
		mutex_unlock(&ts->data_lock);
		retval = wait_for_completion_interruptible_timeout(
			&ts->si_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX));
		if (retval < 0) {
			pr_err("%s: SysInfo mode timeout (ret=%d)\n",
				__func__, retval);
			/*
			 * if the SysInfo interrupts are disabled,
			 * then just wait a max time
			 */
			msleep(CY_DELAY_DFLT * CY_DELAY_MAX);
		}
		mutex_lock(&ts->data_lock);
	} else {
		retval = wait_for_completion_interruptible_timeout(
			&ts->si_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX));
		if (retval < 0) {
			pr_err("%s: SysInfo mode timeout (ret=%d)\n",
				__func__, retval);
			/*
			 * if the SysInfo interrupts are disabled,
			 * then just wait a max time
			 */
			msleep(CY_DELAY_DFLT * CY_DELAY_MAX);
		}
	}
	ts->power_state = CY_READY_STATE;

	/* wait for TTSP Device to complete switch to Operational mode */
	tries = 0;
	do {
		memset(&ts->xy_data, 0, sizeof(struct cyttsp_xydata));
		retval = ttsp_read_block_data(ts, CY_REG_BASE,
			sizeof(struct cyttsp_xydata), &ts->xy_data);
		if (!(retval < 0) &&
			!(ts->xy_data.hst_mode & CY_HST_MODE_CHANGE_BIT)) {
			_cyttsp_hndshk(ts, ts->xy_data.hst_mode);
			break;
		}
		msleep(CY_DELAY_DFLT);
	} while (!((retval == 0) &&
		((ts->xy_data.act_dist & CY_ACT_DIST_BITS) ==
		(CY_ACT_DIST_DFLT & CY_ACT_DIST_BITS))) &&
		(tries++ < CY_DELAY_MAX));

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: check op ready hst_mode=%02X tries=%d ret=%d dist=%02X\n",
		__func__,
		ts->xy_data.hst_mode, tries, retval, ts->xy_data.act_dist);

	if (retval || tries >= CY_DELAY_MAX) {
		pr_err("%s: fail enter Operational mode "
			"tries=%d ret=%d dist=%02X "
			"host mode bytes=%02X %02X %02X\n",
			__func__, tries, retval,
			ts->xy_data.act_dist, ts->xy_data.hst_mode,
			ts->xy_data.tt_mode, ts->xy_data.tt_stat);
		if (GET_BOOTLOADERMODE(ts->xy_data.tt_mode)) {
			ts->power_state = CY_BL_STATE;
			cyttsp_pr_state(ts);
		}
	} else {
		ts->power_state = CY_ACTIVE_STATE;
		cyttsp_pr_state(ts);
	}
#endif /* FUJITSU:2011-10-02 del end */

	return retval;
}

static int _cyttsp_set_sysinfo_mode(struct cyttsp *ts)
{
	int tries;
	int retval;
	/* FUJITSU:2011-08-12 mod start */
//	u8 cmd = CY_SYSINFO_MODE + CY_HST_MODE_CHANGE_BIT;
	u8 cmd = CY_SYSINFO_MODE;
	/* FUJITSU:2011-08-12 mod end */

	memset(&(ts->sysinfo_data), 0, sizeof(struct cyttsp_sysinfo_data));

	/* switch to sysinfo mode */
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	if (retval < 0) {
		pr_err("%s: write fail set SysInfo mode (ret=%d)\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp_pr_state(ts);
		return retval;
	}

	/* wait for interrupt to set ready completion */
	ts->power_state = CY_SYSINFO_STATE;
	tries = 0;
	do {
		INIT_COMPLETION(ts->si_int_running);
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			retval = wait_for_completion_interruptible_timeout(
				&ts->si_int_running,
				msecs_to_jiffies(CY_DELAY_DFLT));
			mutex_lock(&ts->data_lock);
		} else {
			retval = wait_for_completion_interruptible_timeout(
				&ts->si_int_running,
				msecs_to_jiffies(CY_DELAY_DFLT));
		}
		if (retval < 0) {
			pr_err("%s: Error setting up SysInfo mode timeout"
				" (ret=%d)\n", __func__, retval);
			/* just wait a max time */
			msleep(CY_DELAY_DFLT * CY_DELAY_MAX);
			retval = 0;
			break;
		} else {
			/* FUJITSU:2011-08-12 add start */
			msleep(100);
			/* FUJITSU:2011-08-12 add end */
			/* read sysinfo registers */
			retval = ttsp_read_block_data(ts, CY_REG_BASE,
				sizeof(ts->sysinfo_data), &(ts->sysinfo_data));
			mb();
			if (retval < 0) {
				pr_err("%s: SysInfo access err (ret=%d)\n",
					__func__, retval);
			}
			if (ts->sysinfo_data.app_verh ||
				ts->sysinfo_data.app_verl)
				break;
		}
	/* FUJITSU:2011-08-12 mod start */
#if 0
	} while ((ts->sysinfo_data.hst_mode & CY_HST_MODE_CHANGE_BIT) &&
		(tries++ < CY_DELAY_MAX));
#endif
	} while (tries++ < CY_DELAY_MAX);
	/* FUJITSU:2011-08-12 mod end */

	/* FUJITSU:2011-08-11 add start */
	printk("[TPD_Sy] %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
		"%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
		"%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
		"%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n"
		"[TPD_Sy] %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
		"%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
		"%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
		"%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
		ts->sysinfo_data.hst_mode, ts->sysinfo_data.mfg_cmd, ts->sysinfo_data.mfg_stat,
		ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1], ts->sysinfo_data.cid[2],
		ts->sysinfo_data.tt_undef1, ts->sysinfo_data.uid[0], ts->sysinfo_data.uid[1],
		ts->sysinfo_data.uid[2], ts->sysinfo_data.uid[3], ts->sysinfo_data.uid[4],
		ts->sysinfo_data.uid[5], ts->sysinfo_data.uid[6], ts->sysinfo_data.uid[7],
		ts->sysinfo_data.bl_verh, ts->sysinfo_data.bl_verl, ts->sysinfo_data.tts_verh,
		ts->sysinfo_data.tts_verl, ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
		ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl, ts->sysinfo_data.tt_undef[0],
		ts->sysinfo_data.tt_undef[1], ts->sysinfo_data.tt_undef[2], ts->sysinfo_data.tt_undef[3],
		ts->sysinfo_data.tt_undef[4], ts->sysinfo_data.scn_typ, ts->sysinfo_data.act_intrvl,
		ts->sysinfo_data.tch_tmout, ts->sysinfo_data.lp_intrvl, ts->sysinfo_data.tt_undef2,
		ts->sysinfo_data.pump_freg, ts->sysinfo_data.imo_trm, ts->sysinfo_data.setting,
		ts->sysinfo_data.large_th, ts->sysinfo_data.large_num, ts->sysinfo_data.iir_reset,
		ts->sysinfo_data.iir_weight1, ts->sysinfo_data.weight_change34, ts->sysinfo_data.tt_undef3,
		ts->sysinfo_data.iir_weight2, ts->sysinfo_data.weight_change23, ts->sysinfo_data.weight_change12,
		ts->sysinfo_data.tt_undef4[0], ts->sysinfo_data.tt_undef4[1], ts->sysinfo_data.tt_undef4[2],
		ts->sysinfo_data.fth, ts->sysinfo_data.nth, ts->sysinfo_data.pre, ts->sysinfo_data.nsub,
		ts->sysinfo_data.nsub_shift, ts->sysinfo_data.ncycle, ts->sysinfo_data.self_fth,
		ts->sysinfo_data.self_pre, ts->sysinfo_data.self_nsub, ts->sysinfo_data.self_shift,
		ts->sysinfo_data.self_ncycle, ts->sysinfo_data.self_th_mut_reset, ts->sysinfo_data.mut_reset_delay,
		ts->sysinfo_data.mut_th_self_reset, ts->sysinfo_data.self_reset_delay, ts->sysinfo_data.tt_undef5,
		ts->sysinfo_data.stylus_to_finger_th, ts->sysinfo_data.stylus_noise_th, ts->sysinfo_data.stylus_touch_th,
		ts->sysinfo_data.stylus_hyst_th, ts->sysinfo_data.stylus_coef, ts->sysinfo_data.stylus_prescaler,
		ts->sysinfo_data.stylus_nsub, ts->sysinfo_data.stylus_nshift, ts->sysinfo_data.stylus_ncycles,
		ts->sysinfo_data.stylus_adc_resolution, ts->sysinfo_data.stylus_reset_delay);
	/* FUJITSU:2011-08-11 add end */

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: check sysinfo ready hst_mode=%02X tries=%d ret=%d\n",
		__func__, ts->sysinfo_data.hst_mode, tries, retval);
	pr_info("%s: hm=%02X tv=%02X%02X ai=0x%02X%02X "
		"av=0x%02X%02X ci=0x%02X%02X%02X\n", __func__,
		ts->sysinfo_data.hst_mode,
		ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
		ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
		ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
		ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1],
		ts->sysinfo_data.cid[2]);

	return retval;
}

static int _cyttsp_set_sysinfo_regs(struct cyttsp *ts)
{
	int retval = 0;
	/* FUJITSU:2011-08-11 add start */
	u8 toggle_cmd = 0x00;
	u8 act_cmd = 0x11;
//	u8 setting_cmd = 0x00;
	u8 scn_typ_cmd = 0x00;

	retval = ttsp_write_block_data(ts, CY_REG_ACT_INTRVL, sizeof(act_cmd), &act_cmd);
//	retval = ttsp_write_block_data(ts, 0x23, sizeof(setting_cmd), &setting_cmd);
	scn_typ_cmd = (ts->sysinfo_data.scn_typ & 0x0F);
	scn_typ_cmd = scn_typ_cmd | 0xA0;
	retval = ttsp_write_block_data(ts, 0x1C, sizeof(scn_typ_cmd), &scn_typ_cmd);

	toggle_cmd = ts->sysinfo_data.hst_mode & CY_HNDSHK_BIT ?
		ts->sysinfo_data.hst_mode & ~CY_HNDSHK_BIT :
		ts->sysinfo_data.hst_mode | CY_HNDSHK_BIT;

	cyttsp_dbg(ts, CY_DBG_LVL_3, 
		"[TPD]%s: Sys.mode Toggle Command = 0x%02X\n",
		__func__, toggle_cmd);
	retval = ttsp_write_block_data(ts, CY_REG_BASE,
				       sizeof(toggle_cmd), (u8 *)&toggle_cmd);

	msleep(20);
	/* FUJITSU:2011-08-11 add end */

#if 0  /* FUJITSU:2011-08-12 del start */
	u8 size = ts->platform_data->sett[CY_IC_GRPNUM_SI]->size;
	u8 tag = ts->platform_data->sett[CY_IC_GRPNUM_SI]->tag;
	u8 reg_offset;
	u8 data_len;

	reg_offset = CY_REG_SI_START + tag;
	data_len = size - tag;  /* Overflows if tag > size; checked below */

	/* Do bounds checking on the data */
	if (reg_offset > CY_REG_SI_END) {
		pr_err("%s: Sysinfo reg data starts out of bounds--%s",
			__func__, "IC defaults will be used\n");
		goto cyttsp_set_sysinfo_regs_exit;
	} else if (tag > size) { /* overflow if (size - tag) is negative */
		pr_err("%s: Invalid sysinfo data length--%s",
			__func__, "IC defaults will be used\n");
		goto cyttsp_set_sysinfo_regs_exit;
	} else if ((reg_offset + data_len - 1) > CY_REG_SI_END) {
		pr_err("%s: Sysinfo reg data overflow--%s",
			__func__, "IC defaults will be used\n");
		goto cyttsp_set_sysinfo_regs_exit;
	}

	retval = ttsp_write_block_data(ts, reg_offset, data_len,
		&(ts->platform_data->sett[CY_IC_GRPNUM_SI]->data[tag]));
	mb();

	if (retval < 0) {
		pr_err("%s: write fail on SysInfo Regs (ret=%d)\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp_pr_state(ts);
	} else
		msleep(CY_DELAY_DFLT);	/* wait for register setup */

cyttsp_set_sysinfo_regs_exit:
#endif /* FUJITSU:2011-08-12 del end */
	return retval;
}

static int _cyttsp_hard_reset(struct cyttsp *ts)
{
	int retval = 0;

	ts->power_state = CY_BL_STATE;
	cyttsp_pr_state(ts);

	_cyttsp_soft_reset(ts);  /* Does nothing if CY_USE_HW_RESET defined */

	if (ts->platform_data->hw_reset) {
		retval = ts->platform_data->hw_reset();
		if (retval < 0) {
			pr_err("%s: fail on hard reset (ret=%d)\n",
				__func__, retval);
			goto _cyttsp_hard_reset_exit;
		}
	} else {
		pr_err("%s: no hardware reset function (ret=%d)\n",
			__func__, retval);
		retval = -ENOSYS;
		goto _cyttsp_hard_reset_exit;
	}

	/* wait for interrupt to set ready completion */
	INIT_COMPLETION(ts->bl_int_running);
	if (mutex_is_locked(&ts->data_lock)) {
		mutex_unlock(&ts->data_lock);
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX));
		mutex_lock(&ts->data_lock);
	} else {
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX));
	}
	if (retval < 0) {
		pr_err("%s: Hard reset timer setup fail (ret=%d)\n",
			__func__, retval);
		/* just sleep a default time */

		msleep(CY_DELAY_DFLT * CY_DELAY_MAX);
		retval = 0;
	}

	cyttsp_dbg(ts, CY_DBG_LVL_2,
		"%s:  Hard Reset: ret=%d", __func__, retval);

	if (retval > 0)
		retval = 0;

_cyttsp_hard_reset_exit:
	return retval;
}

#if 0  /* FUJITSU:2011-08-11 del start */
static int _cyttsp_set_operational_regs(struct cyttsp *ts)
{
	int retval = 0;
	u8 size = ts->platform_data->sett[CY_IC_GRPNUM_OP]->size;
	u8 tag = ts->platform_data->sett[CY_IC_GRPNUM_OP]->tag;
	u8 reg_offset;
	u8 data_len;

	reg_offset = CY_REG_OP_START + tag;
	data_len = size - tag;  /* Overflows if tag > size; checked below */

	/* Do bounds checking on the data */
	if (reg_offset > CY_REG_OP_END) {
		pr_err("%s: Op reg data starts out of bounds--%s\n",
			__func__, "IC defaults will be used");
		goto cyttsp_set_op_regs_exit;
	} else if (tag > size) { /* overflow if (size - tag) is negative */
		pr_err("%s: Invalid op data length--%s\n",
			__func__, "IC defaults will be used");
		goto cyttsp_set_op_regs_exit;
	} else if ((reg_offset + data_len - 1) > CY_REG_OP_END) {
		pr_err("%s: Op reg data overflow--%s\n",
			__func__, "IC defaults will be used");
		goto cyttsp_set_op_regs_exit;
	}

	retval = ttsp_write_block_data(ts, reg_offset, data_len,
		&(ts->platform_data->sett[CY_IC_GRPNUM_OP]->data[tag]));

	cyttsp_dbg(ts, CY_DBG_LVL_2,
		"%s: Write OP regs ret=%d\n", __func__, retval);

	if (retval < 0) {
		pr_err("%s: bus write fail on Op Regs (ret=%d)\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp_pr_state(ts);
	}

cyttsp_set_op_regs_exit:
	return retval;
}
#endif /* FUJITSU:2011-08-11 del end */

/* map pointers to touch information to allow loop on get xy_data */
static void _cyttsp_init_tch_map(struct cyttsp *ts)
{
	ts->tch_map[0].tch = &ts->xy_data.tch1;
	ts->tch_map[0].id = &ts->xy_data.touch12_id;
	ts->tch_map[1].tch = &ts->xy_data.tch2;
	ts->tch_map[1].id = &ts->xy_data.touch12_id;
	ts->tch_map[2].tch = &ts->xy_data.tch3;
	ts->tch_map[2].id = &ts->xy_data.touch34_id;
	ts->tch_map[3].tch = &ts->xy_data.tch4;
	ts->tch_map[3].id = &ts->xy_data.touch34_id;
}

#ifdef CY_USE_XTD_MT
#include "cyttsp3_xtd.h"
#else
static bool cyttsp_xtd_multi_touch(struct cyttsp_trk *cur_trk,
	struct cyttsp *ts)
{
	/* FUJITSU:2011-12-27 mod start */
	u8 id;
	int i = 0;
//	int cnt = 0;

	/* terminate any previous touch where the track
	 * is missing from the current event */
	for (id = 0; id < CY_NUM_TRK_ID; id++) {
		if ((ts->prv_trk[id].tch == CY_NTCH) || (cur_trk[id].tch != CY_NTCH))
			continue;

		input_mt_slot(ts->input, ts->prv_trk[id].abs[CY_ABS_ID_OST]);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);

#if 0
		input_report_abs(ts->input, ABS_MT_TRACKING_ID,
					ts->prv_trk[id].abs[CY_ABS_ID_OST]);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, CY_NTCH);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
					ts->prv_trk[id].abs[CY_ABS_W_OST]);
		input_report_abs(ts->input, ABS_MT_POSITION_X,
					ts->prv_trk[id].abs[CY_ABS_X_OST]);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,
					ts->prv_trk[id].abs[CY_ABS_Y_OST]);

		input_mt_sync(ts->input);
#endif

		cyttsp_dbg(ts, CY_DBG_LVL_2,
			"[TPD_Ps] ID=%02d X=%04d Y=%04d Z=%03d W=%03d UP\n", 
			ts->prv_trk[id].abs[CY_ABS_ID_OST],
			ts->prv_trk[id].abs[CY_ABS_X_OST],
			ts->prv_trk[id].abs[CY_ABS_Y_OST],
			ts->prv_trk[id].abs[CY_ABS_P_OST],
			ts->prv_trk[id].abs[CY_ABS_W_OST]);

		ts->prv_trk[id].tch = CY_NTCH;
		ts->prv_trk[id].abs[CY_ABS_X_OST] = 0;
		ts->prv_trk[id].abs[CY_ABS_Y_OST] = 0;
		ts->prv_trk[id].abs[CY_ABS_P_OST] = 0;
		ts->mt_trc_id[ts->prv_trk[id].abs[CY_ABS_ID_OST]] = -1;
		ts->prv_trk[id].abs[CY_ABS_ID_OST] = -1;
	}

	/* set Multi-Touch current event signals */
	for (id = 0; id < CY_NUM_TRK_ID; id++) {
		if (cur_trk[id].tch != CY_TCH)
			continue;

		for(i = 0; i < CY_NUM_TCH_ID ; i++) {
			if(ts->mt_trc_id[i] == id) {
				cur_trk[id].abs[CY_ABS_ID_OST] = i;
				break;
			}
		}
		if(i == CY_NUM_TCH_ID) {
			for(i = 0; i < CY_NUM_TCH_ID ; i++) {
				if(ts->mt_trc_id[i] < 0) {
					ts->mt_trc_id[i] = id;
					cur_trk[id].abs[CY_ABS_ID_OST] = i;
					break;
				}
			}
		}

		input_mt_slot(ts->input, cur_trk[id].abs[CY_ABS_ID_OST]);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
//		input_report_abs(ts->input, ABS_MT_TRACKING_ID,
//					cur_trk[id].abs[CY_ABS_ID_OST]);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
					cur_trk[id].abs[CY_ABS_W_OST]);
//		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
//					cur_trk[id].abs[CY_ABS_W_OST]);
		input_report_abs(ts->input, ABS_MT_PRESSURE,
					cur_trk[id].abs[CY_ABS_P_OST]);
		input_report_abs(ts->input, ABS_MT_POSITION_X,
					cur_trk[id].abs[CY_ABS_X_OST]);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,
					cur_trk[id].abs[CY_ABS_Y_OST]);

//		input_mt_sync(ts->input);
//		cnt++;

		cyttsp_dbg(ts, CY_DBG_LVL_2,
			"[TPD_Ps] ID=%02d X=%04d Y=%04d Z=%03d W=%03d Down/Move\n", 
			cur_trk[id].abs[CY_ABS_ID_OST],
			cur_trk[id].abs[CY_ABS_X_OST],
			cur_trk[id].abs[CY_ABS_Y_OST],
			cur_trk[id].abs[CY_ABS_P_OST],
			cur_trk[id].abs[CY_ABS_W_OST]);

		ts->prv_trk[id].tch = CY_TCH;
		ts->prv_trk[id].abs[CY_ABS_X_OST] = cur_trk[id].abs[CY_ABS_X_OST];
		ts->prv_trk[id].abs[CY_ABS_Y_OST] = cur_trk[id].abs[CY_ABS_Y_OST];
		ts->prv_trk[id].abs[CY_ABS_W_OST] = cur_trk[id].abs[CY_ABS_W_OST];
		ts->prv_trk[id].abs[CY_ABS_ID_OST] = cur_trk[id].abs[CY_ABS_ID_OST];
	}

//	if(!cnt){
//		input_mt_sync(ts->input);
//	}
	
	input_sync(ts->input);

//	return false;
	return true;
	/* FUJITSU:2011-12-27 mod end */
}

/* FUJITSU:2012-05-14 add start */
static bool cyttsp_quickdownup_guardcancel(struct cyttsp_trk *cur_trk,
	struct cyttsp *ts)
{

	u8 id;
	
	for (id = 0; id < CY_NUM_TRK_ID; id++) {
		if ((cur_trk[id].tch == CY_NTCH) || (ts->prv_trk[id].tch == CY_NTCH))
			continue;
		if((    cur_trk[id].abs[CY_ABS_X_OST] > (ts->prv_trk[id].abs[CY_ABS_X_OST] + ts->jdg_area_size))
			|| (cur_trk[id].abs[CY_ABS_X_OST] < (ts->prv_trk[id].abs[CY_ABS_X_OST] - ts->jdg_area_size))
			|| (cur_trk[id].abs[CY_ABS_Y_OST] > (ts->prv_trk[id].abs[CY_ABS_Y_OST] + ts->jdg_area_size))
			|| (cur_trk[id].abs[CY_ABS_Y_OST] < (ts->prv_trk[id].abs[CY_ABS_Y_OST] - ts->jdg_area_size))){

			cyttsp_dbg(ts, CY_DBG_LVL_3, "[TPD]%s: Quick DOWN/UP Event Guard Cancel\n", __func__);
			input_mt_slot(ts->input, ts->prv_trk[id].abs[CY_ABS_ID_OST]);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);

			cyttsp_dbg(ts, CY_DBG_LVL_2,
				"[TPD_Ps] ID=%02d X=%04d Y=%04d Z=%03d W=%03d UP\n", 
				ts->prv_trk[id].abs[CY_ABS_ID_OST],
				ts->prv_trk[id].abs[CY_ABS_X_OST],
				ts->prv_trk[id].abs[CY_ABS_Y_OST],
				ts->prv_trk[id].abs[CY_ABS_P_OST],
				ts->prv_trk[id].abs[CY_ABS_W_OST]);

			ts->prv_trk[id].tch = CY_NTCH;
			ts->prv_trk[id].abs[CY_ABS_X_OST] = 0;
			ts->prv_trk[id].abs[CY_ABS_Y_OST] = 0;
			ts->prv_trk[id].abs[CY_ABS_P_OST] = 0;
			ts->mt_trc_id[ts->prv_trk[id].abs[CY_ABS_ID_OST]] = -1;
			ts->prv_trk[id].abs[CY_ABS_ID_OST] = -1;
			cur_trk[id].tch = CY_TCH;

		}
	}
	
	input_sync(ts->input);
	ts->timer_up_flag = false;
	return true;
}
#endif
/* FUJITSU:2012-05-14 add end */
/* read xy_data for all current touches */
static int _cyttsp_xy_worker(struct cyttsp *ts)
{
	u8 cur_tch;
	u8 tch;
	u8 id;
#ifdef CY_USE_DEBUG_TOOLS
	int tmp;
#endif
	int frmwrk_abs;
	int frmwrk_size = ts->platform_data->frmwrk->size/CY_NUM_ABS_SET;
	struct cyttsp_trk cur_trk[CY_NUM_TRK_ID];

	/*
	 * Get event data from CYTTSP device.
	 * The event data includes all data
	 * for all active touches.
	 */
	memset(&ts->xy_data, 0, sizeof(struct cyttsp_xydata));
	if (ttsp_read_block_data(ts,
		CY_REG_BASE, sizeof(struct cyttsp_xydata), &ts->xy_data)) {
		pr_err("%s: read fail on operational reg\n",
			__func__);
		ts->power_state = CY_IDLE_STATE;
		cyttsp_pr_state(ts);
		return -1;
	}

	/* FUJITSU:2011-10-17 add start */
	cyttsp_dbg(ts, CY_DBG_LVL_1,
		"[TPD_Rw] %02X,%02X,%02X,%04X,%04X,%02X,%02X,"
		"%04X,%04X,%02X,%02X,%02X,%04X,%04X,%02X,%02X,%04X,%04X,%02X,%02X,%02X\n",
		ts->xy_data.hst_mode, ts->xy_data.tt_mode, ts->xy_data.tt_stat,
		be16_to_cpu(ts->xy_data.tch1.x),
		be16_to_cpu(ts->xy_data.tch1.y),
		ts->xy_data.tch1.z,
		ts->xy_data.touch12_id,
		be16_to_cpu(ts->xy_data.tch2.x),
		be16_to_cpu(ts->xy_data.tch2.y),
		ts->xy_data.tch2.z,
		ts->xy_data.gest_cnt,
		ts->xy_data.gest_id,
		be16_to_cpu(ts->xy_data.tch3.x),
		be16_to_cpu(ts->xy_data.tch3.y),
		ts->xy_data.tch3.z, ts->xy_data.touch34_id,
		be16_to_cpu(ts->xy_data.tch4.x),
		be16_to_cpu(ts->xy_data.tch4.y),
		ts->xy_data.tch4.z, ts->xy_data.act_dist, ts->xy_data.tt_reserved);
	/* FUJITSU:2011-10-17 add end */
#if 0  /* FUJITSU:2011-08-31 del start */
	cyttsp_dbg(ts, CY_DBG_LVL_1,
		"%s:\n"
		"  hm=%02X tm=%02X ts=%02X\n"
		"  X1=%04X Y1=%04X Z1=%02X ID12=%02X\n"
		"  X2=%04X Y2=%04X Z2=%02X\n"
		"  X3=%04X Y3=%04X Z3=%02X ID34=%02X\n"
		"  X4=%04X Y4=%04X Z4=%02X AD=%02X\n",
		__func__,
		ts->xy_data.hst_mode, ts->xy_data.tt_mode, ts->xy_data.tt_stat,
		be16_to_cpu(ts->xy_data.tch1.x),
		be16_to_cpu(ts->xy_data.tch1.y),
		ts->xy_data.tch1.z, ts->xy_data.touch12_id,
		be16_to_cpu(ts->xy_data.tch2.x),
		be16_to_cpu(ts->xy_data.tch2.y),
		ts->xy_data.tch2.z,
		be16_to_cpu(ts->xy_data.tch3.x),
		be16_to_cpu(ts->xy_data.tch3.y),
		ts->xy_data.tch3.z, ts->xy_data.touch34_id,
		be16_to_cpu(ts->xy_data.tch4.x),
		be16_to_cpu(ts->xy_data.tch4.y),
		ts->xy_data.tch4.z, ts->xy_data.act_dist);
#endif /* FUJITSU:2011-08-31 del end */

	/* provide flow control handshake */
	if (ts->flags & CY_USE_HNDSHK) {
		if (_cyttsp_hndshk(ts, ts->xy_data.hst_mode)) {
			pr_err("%s: handshake fail on operational reg\n",
				__func__);
			ts->power_state = CY_IDLE_STATE;
			cyttsp_pr_state(ts);
			return -1;
		}
	}

	/* determine number of currently active touches */
	cur_tch = GET_NUM_TOUCHES(ts->xy_data.tt_stat);

	/* check for any error conditions */
	if (GET_BOOTLOADERMODE(ts->xy_data.tt_mode)) {
		pr_err("%s: BL mode detected in active state\n", __func__);
#if 0  /* FUJITSU:2011-08-11 del start */
		if (BL_WATCHDOG_DETECT(ts->xy_data.tt_mode))
			pr_err("%s: BL watchdog timeout detected\n", __func__);
#endif /* FUJITSU:2011-08-11 del end */
		ts->power_state = CY_BL_STATE;
		cyttsp_pr_state(ts);
		return -1;
	} else if (GET_HSTMODE(ts->xy_data.hst_mode) ==
		GET_HSTMODE(CY_SYSINFO_MODE)) {
		pr_err("%s: got SysInfo interrupt; expected touch "
			"(hst_mode=%02X)\n", __func__, ts->xy_data.hst_mode);
		return 0;
	} else if (IS_LARGE_AREA(ts->xy_data.tt_stat) == 1) {
		/* terminate all active tracks */
		cur_tch = CY_NTCH;
		cyttsp_dbg(ts, CY_DBG_LVL_3,
			"%s: Large area detected\n", __func__);
	} else if (cur_tch == (CY_NUM_TRK_ID-1)) {
		/* terminate all active tracks */
		cur_tch = CY_NTCH;
		pr_err("%s: Num touch error detected\n", __func__);
	} else if (cur_tch >= CY_NUM_TCH_ID) {
		/* set cur_tch to maximum */
		cyttsp_dbg(ts, CY_DBG_LVL_2,
			"%s: Number of Touches %d set to max %d\n",
			__func__, cur_tch, CY_NUM_TCH_ID);
		cur_tch = CY_NUM_TCH_ID;
	} else if (IS_BAD_PKT(ts->xy_data.tt_mode)) {
		/* terminate all active tracks */
		cur_tch = CY_NTCH;
		pr_err("%s: Invalid buffer detected\n", __func__);
	}

#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
	/*
	 * send no events if there were no
	 * previous touches and no new touches
	 */
	if ((ts->prv_tch == CY_NTCH) && (cur_tch == CY_NTCH)) {
		if (++ts->ntch_count > CY_MAX_NTCH) {
			/* TTSP device has a stuck operational mode */
			ts->ntch_count = CY_NTCH;
#if 0  /* FUJITSU:2011-08-11 del start */
			pr_err("%s: Error, stuck no-touch ct=%d\n",
				__func__, cur_tch);
			if (_cyttsp_startup(ts))
				pr_err("%s: Error, startup fail\n", __func__);
#endif /* FUJITSU:2011-08-11 del end */
			return 0;
		}
	} else
		ts->ntch_count = CY_NTCH;

	/* FUJITSU:2012-05-14 add start */
	if((ts->prv_tch == CY_NTCH) && (cur_tch != CY_NTCH)) {
		ts->first_down_time = jiffies;
	}

	if((ts->prv_tch != CY_NTCH) && (cur_tch == CY_NTCH)) {
		if(((jiffies - ts->first_down_time) <= CY_MOVE_JUDGE_TIME)
			&& ((jiffies - ts->first_down_time) > 0)) {
			cyttsp_dbg(ts, CY_DBG_LVL_3, "[TPD]%s: Quick Down/UP Event!! Timer ON\n", __func__);
			mod_timer(&ts->timer_up, jiffies + CY_UP_JUDGE_TIME);
			ts->timer_up_flag = true;
			return 0;
		}
		cyttsp_dbg(ts, CY_DBG_LVL_3, "[TPD]%s: Normal UP Event\n", __func__);
	}
	/* FUJITSU:2012-05-14 add end */

	ts->prv_tch = cur_tch;
#endif

	/* clear current touch tracking structures */
	memset(cur_trk, CY_NTCH, sizeof(cur_trk));

	/* extract xy_data for all currently reported touches */
	for (tch = 0; tch < cur_tch; tch++) {
		id = tch & 0x01 ?
			LO_TRACKID(*(ts->tch_map[tch].id)) :
			HI_TRACKID(*(ts->tch_map[tch].id));
		cur_trk[id].tch = CY_TCH;
		cur_trk[id].abs[CY_ABS_X_OST] =
			be16_to_cpu((ts->tch_map[tch].tch)->x);
		cur_trk[id].abs[CY_ABS_Y_OST] =
			be16_to_cpu((ts->tch_map[tch].tch)->y);
		cur_trk[id].abs[CY_ABS_P_OST] =
			(ts->tch_map[tch].tch)->z;
		cur_trk[id].abs[CY_ABS_W_OST] = CY_SMALL_TOOL_WIDTH;
		cur_trk[id].abs[CY_ABS_ID_OST] = id;

#ifdef CY_USE_DEBUG_TOOLS
		if (ts->flags & CY_FLIP) {
			tmp = cur_trk[id].abs[CY_ABS_X_OST];
			cur_trk[id].abs[CY_ABS_X_OST] =
				cur_trk[id].abs[CY_ABS_Y_OST];
			cur_trk[id].abs[CY_ABS_Y_OST] = tmp;
		} else {
			if (ts->flags & CY_INV_X) {
				cur_trk[id].abs[CY_ABS_X_OST] =
					ts->platform_data->frmwrk->abs
					[(CY_ABS_X_OST*CY_NUM_ABS_SET)
					+2] - cur_trk[id].abs[CY_ABS_X_OST];
			}
			if (ts->flags & CY_INV_Y) {
				cur_trk[id].abs[CY_ABS_Y_OST] =
					ts->platform_data->frmwrk->abs
					[(CY_ABS_Y_OST*CY_NUM_ABS_SET)
					+2] - cur_trk[id].abs[CY_ABS_Y_OST];
			}
		}
#endif
	}
	/* FUJITSU:2012-05-14 add start */
	if(ts->timer_up_flag){
		cyttsp_quickdownup_guardcancel(cur_trk, ts);
	}
	/* FUJITSU:2012-05-14 add end */
	/* provide input event signaling for each active touch */
	if (!cyttsp_xtd_multi_touch(cur_trk, ts)) {
		for (id = 0; id < CY_NUM_TRK_ID; id++) {
			if (cur_trk[id].tch) {
				for (tch = 0; tch < frmwrk_size; tch++) {
					frmwrk_abs = ts->platform_data->frmwrk->
						abs[(tch*CY_NUM_ABS_SET)+0];
					if (frmwrk_abs) {
						input_report_abs(ts->input,
							frmwrk_abs,
							cur_trk[id].abs[tch]);
					}
				}
			}
//			input_mt_sync(ts->input);
			cyttsp_dbg(ts, CY_DBG_LVL_1,
				"%s: ID:%3d  X:%3d  Y:%3d  "
				"Z:%3d  W=%3d  T=%3d\n",
				__func__, id,
				cur_trk[id].abs[CY_ABS_X_OST],
				cur_trk[id].abs[CY_ABS_Y_OST],
				cur_trk[id].abs[CY_ABS_P_OST],
				cur_trk[id].abs[CY_ABS_W_OST],
				cur_trk[id].abs[CY_ABS_ID_OST]);
		}
		input_sync(ts->input);
	}

	return 0;
}

#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
/* FUJITSU:2011-08-11 mod start */
//#define CY_TIMEOUT msecs_to_jiffies(1000)
#define CY_TIMEOUT msecs_to_jiffies(3000)
/* FUJITSU:2011-08-11 mod end */
/* FUJITSU:2012-01-10 add start */
static void cyttsp_recovery(struct cyttsp *ts)
{
	printk("%s: IN PS=%d\n", __func__, ts->power_state);

//	del_timer(&ts->timer);
//	cancel_work_sync(&ts->work);
	/* change state */
	ts->power_state = CY_READY_STATE;
	/* wake up device with a strobe(power i/o) */
	ts->platform_data->hw_power(0);
	ts->platform_data->hw_power(1);
	/* startup */
	_cyttsp_startup(ts);
	mod_timer(&ts->timer, jiffies + CY_TIMEOUT);

	printk("%s: Recovery Complete. PS=%d\n", __func__, ts->power_state);

	return;
}
/* FUJITSU:2011-01-10 add end */
static void cyttsp_timer_watchdog(struct work_struct *work)
{
	struct cyttsp *ts = container_of(work, struct cyttsp, work);
	int retval;
	/* FUJITSU:2011-10-02 mod start */
//	u8 cmd;
	u8 cmd[2];
	/* FUJITSU:2011-10-02 mod end */
	/* FUJITSU:2011-10-02 add start */
	int i = 0;

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: IN PS=%d\n", __func__, ts->power_state);
	/* FUJITSU:2011-10-02 add end */


	if (ts == NULL) {
		pr_err("%s: NULL context pointer\n", __func__);
		return;
	}

	/* FUJITSU:2011-12-27 add start */
	if(mutex_is_locked(&ts->data_lock)){
		printk("[TPD]%s: Mutex is Locked\n", __func__);
		return;
	}
	if (ts->power_state == CY_SLEEP_STATE) {
		pr_err("%s: Already suspended\n", __func__);
		return;
	}
	/* FUJITSU:2011-12-27 add end */
	mutex_lock(&ts->data_lock);
	if (ts->power_state == CY_ACTIVE_STATE) {
#if 0  /* FUJITSU:2011-10-02 del start */
		retval = ttsp_read_block_data(ts, CY_REG_BASE,
			sizeof(cmd), &cmd);
		if (retval < 0) {
			pr_err("%s: Read Block fail (r=%d)\n",
				__func__, retval);
			goto cyttsp_timer_watchdog_exit;
		}
#endif /* FUJITSU:2011-10-02 del end */
		/* FUJITSU:2011-10-02 add start */
		retval = ts->bus_ops->read(ts->bus_ops, CY_REG_BASE, sizeof(cmd), &cmd);
		if(retval < 0) {
			mdelay(CY_I2C_NACK_WAIT_VALUE);
			for(i = 0; i < 3 ; i++) {
				retval = ts->bus_ops->read(ts->bus_ops, CY_REG_BASE, sizeof(cmd), &cmd);
				if(retval >= 0) {
					break;
				} else {
					cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: I2C Read Err try=%d\n", __func__, i);
					mdelay(10);
				}
			}
		}

		if(retval < 0) {
			pr_err("%s: Read Block fail (r=%d)\n", __func__, retval);
			cyttsp_recovery(ts);
			goto cyttsp_timer_watchdog_exit;
		}
		/* FUJITSU:2011-10-02 add end */

		/* FUJITSU:2011-10-02 mod start */
		if (GET_BOOTLOADERMODE(cmd[1])) {
			pr_err("%s: BL mode detected in active state\n", __func__);
			ts->power_state = CY_BL_STATE;
			cyttsp_recovery(ts);
//			if (_cyttsp_startup(ts)) {
//				pr_err("%s: Error, startup fail\n", __func__);
//			}
			goto cyttsp_timer_watchdog_exit;
		}

		if (GET_HSTMODE(cmd[0]) ==
			GET_HSTMODE(CY_SYSINFO_MODE)) {
			retval = _cyttsp_set_operational_mode(ts);
			if (retval < 0) {
				pr_err("%s: fail set op mode r=%d\n",
					__func__, retval);
				goto cyttsp_timer_watchdog_exit;
			}
#if 0
			retval = _cyttsp_set_operational_regs(ts);
			if (retval < 0) {
				pr_err("%s: fail set op regs r=%d\n",
					__func__, retval);
				goto cyttsp_timer_watchdog_exit;
			}
#endif
		}
		/* FUJITSU:2011-10-02 mod end */
	}

cyttsp_timer_watchdog_exit:
	mod_timer(&ts->timer, jiffies + CY_TIMEOUT);
	mutex_unlock(&ts->data_lock);
	/* FUJITSU:2011-10-02 add start */
	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: OUT PS=%d\n", __func__, ts->power_state);
	/* FUJITSU:2011-10-02 add end */
	return;
}

static void cyttsp_timer(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	if (!work_pending(&ts->work))
		schedule_work(&ts->work);

	return;
}
#endif

/* FUJITSU:2012-05-14 add start */
static void cyttsp_timer_up(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;
	u8 id;

	cyttsp_dbg(ts, CY_DBG_LVL_3, "[TPD]%s: IN\n", __func__);

	for (id = 0; id < CY_NUM_TRK_ID; id++) {
		if (ts->prv_trk[id].tch == CY_NTCH)
			continue;

		input_mt_slot(ts->input, ts->prv_trk[id].abs[CY_ABS_ID_OST]);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
#if 0
		input_report_abs(ts->input, ABS_MT_TRACKING_ID,
					ts->prv_trk[id].abs[CY_ABS_ID_OST]);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, CY_NTCH);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
					ts->prv_trk[id].abs[CY_ABS_W_OST]);
		input_report_abs(ts->input, ABS_MT_POSITION_X,
					ts->prv_trk[id].abs[CY_ABS_X_OST]);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,
					ts->prv_trk[id].abs[CY_ABS_Y_OST]);

		input_mt_sync(ts->input);
#endif
		cyttsp_dbg(ts, CY_DBG_LVL_2,
			"[TPD_Ps] ID=%02d X=%04d Y=%04d Z=%03d W=%03d UP\n", 
			ts->prv_trk[id].abs[CY_ABS_ID_OST],
			ts->prv_trk[id].abs[CY_ABS_X_OST],
			ts->prv_trk[id].abs[CY_ABS_Y_OST],
			ts->prv_trk[id].abs[CY_ABS_P_OST],
			ts->prv_trk[id].abs[CY_ABS_W_OST]);

		ts->prv_trk[id].tch = CY_NTCH;
		ts->prv_trk[id].abs[CY_ABS_X_OST] = 0;
		ts->prv_trk[id].abs[CY_ABS_Y_OST] = 0;
		ts->prv_trk[id].abs[CY_ABS_P_OST] = 0;
		ts->mt_trc_id[ts->prv_trk[id].abs[CY_ABS_ID_OST]] = -1;
		ts->prv_trk[id].abs[CY_ABS_ID_OST] = -1;
	}
//	input_mt_sync(ts->input);
	input_sync(ts->input);
	ts->timer_up_flag = false;
	ts->prv_tch = CY_NTCH;

	return;
}
/* FUJITSU:2012-05-14 add end */

static int _cyttsp_wait_bl_ready(struct cyttsp *ts, int loop_delay, int max_try)
{
	int tries;
	int retval = 0;
	bool bl_ok = false;

	tries = 0;
	while (1) {
		if (ts->bl_ready_flag) {
			ts->bl_ready_flag = false;
			break;
		} else if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			if (loop_delay < 0)
				udelay(abs(loop_delay));
			else
				msleep(abs(loop_delay));
			mutex_lock(&ts->data_lock);
		} else {
			if (loop_delay < 0)
				udelay(abs(loop_delay));
			else
				msleep(abs(loop_delay));
		}
		if (tries++ > max_try)
			break;
	};

	retval = _cyttsp_load_bl_regs(ts);
	if (retval < 0) {
		pr_err("%s: Load BL regs err r=%d\n",
			__func__, retval);
		bl_ok = false;
	} else if (!(ts->bl_data.bl_status & CY_BL_BUSY) &&
		(ts->bl_data.bl_error == CY_BL_RUNNING)) {
		bl_ok = true;
	} else
		bl_ok = false;

	if (bl_ok)
		return 0;	/* no time out */
	else {
		pr_err("%s: BL timeout err"
			" tries=%d delay=%d max=%d"
			" r=%d bl_s=%02X bl_e=%02X\n",
			__func__,
			tries, loop_delay, max_try, retval,
			ts->bl_data.bl_status,
			ts->bl_data.bl_error);
		return -ETIMEDOUT;	/* timed out */
	}
}

static int _cyttsp_wait_bl_ready_no_stat(struct cyttsp *ts,
	int loop_delay, int max_try)
{
	int tries;
	int retval = 0;
	bool bl_ok = false;

	tries = 0;
	while (1) {
		if (loop_delay < 0)
			udelay(abs(loop_delay));
		else
			msleep(abs(loop_delay));
		if (ts->bl_ready_flag) {
			bl_ok = true;
			break;
		}
		if (tries++ > max_try) {
			bl_ok = true;
			break;
		}
	}

	if (bl_ok) {
		cyttsp_dbg(ts, CY_DBG_LVL_1,
			"%s: rdy=%d tries=%d\n", __func__,
			ts->bl_ready_flag, tries);
		retval = 0;	/* no time out */
	} else {
		pr_err("%s: BL timeout err"
			" tries=%d delay=%d max=%d"
			" r=%d bl_s=%02X bl_e=%02X\n",
			__func__,
			tries, loop_delay, max_try, retval,
			ts->bl_data.bl_status,
			ts->bl_data.bl_error);
		retval = -ETIMEDOUT;
	}

	return retval;
}

static int _cyttsp_wr_blk_chunks(struct cyttsp *ts, u8 cmd,
	u8 length, const u8 *values)
{
	int retval = 0;
	int block = 1;
	int tries = 0;

	u8 dataray[CY_BL_MAX_DATA_LEN];

	mutex_unlock(&ts->data_lock);

	/* first page already includes the bl page offset */
	memcpy(dataray, values, CY_BL_PAGE_SIZE + 1);
	ts->bl_ready_flag = false;
	retval = ttsp_write_block_data(ts, cmd, CY_BL_PAGE_SIZE + 1, dataray);
	if (retval) {
		pr_err("%s: Write chunk err block=%d r=%d\n",
			__func__, 0, retval);
		goto _cyttsp_wr_blk_chunks_exit;
	}
	values += CY_BL_PAGE_SIZE + 1;
	length -= CY_BL_PAGE_SIZE + 1;

	/* remaining pages require bl page offset stuffing */
	while (length && (block < CY_BL_NUM_PAGES) && !(retval < 0)) {
		dataray[0] = CY_BL_PAGE_SIZE * block;

		retval = _cyttsp_wait_bl_ready_no_stat(ts, -100, 100);
		if (retval < 0) {
			pr_err("%s: wait ready timeout block=%d length=%d\n",
				__func__, block, length);
			goto _cyttsp_wr_blk_chunks_exit;
		}

		memcpy(&dataray[1], values, length >= CY_BL_PAGE_SIZE ?
			CY_BL_PAGE_SIZE : length);
		ts->bl_ready_flag = false;
		retval = ttsp_write_block_data(ts, cmd,
			length >= CY_BL_PAGE_SIZE ?
			CY_BL_PAGE_SIZE + 1 : length + 1, dataray);
		if (retval < 0) {
			pr_err("%s: Write chunk err block=%d r=%d\n",
				__func__, block, retval);
			goto _cyttsp_wr_blk_chunks_exit;
		}

		values += CY_BL_PAGE_SIZE;
		length = length >= CY_BL_PAGE_SIZE ?
			length - CY_BL_PAGE_SIZE : 0;
		block++;
	}

	retval = _cyttsp_wait_bl_ready_no_stat(ts, -100, 200);
	if (retval < 0) {
		pr_err("%s: last wait ready timeout block=%d length=%d\n",
			__func__, block, length);
		goto _cyttsp_wr_blk_chunks_exit;
	}

	tries = 0;
	do {
		retval = _cyttsp_load_bl_regs(ts);
		if (retval < 0) {
			pr_err("%s: Load BL regs err r=%d\n",
				__func__, retval);
		} else if ((ts->bl_data.bl_status == CY_BL_READY_NO_APP) &&
			(ts->bl_data.bl_error == CY_BL_RUNNING)) {
			retval = 0;
			break;
		} else if (ts->bl_data.bl_status == CY_BL_READY_APP) {
			retval = 0;
			break;
		} else
			msleep(CY_DELAY_DFLT);
	} while (!(retval < 0) && (tries++ < CY_DELAY_MAX));

_cyttsp_wr_blk_chunks_exit:
	mutex_lock(&ts->data_lock);
	return retval;
}

static int _cyttsp_load_app(struct cyttsp *ts, const u8 *fw, int fw_size)
{
	int retval = 0;
	int loc = 0;

	ts->power_state = CY_LDR_STATE;

	/* send bootload initiation command */
	pr_info("%s: Send BL Enter\n", __func__);
	ts->bl_ready_flag = false;
	retval = ttsp_write_block_data(ts, CY_REG_BASE,
		CY_BL_ENTER_CMD_SIZE, (void *)(&fw[loc]));
	if (retval) {
		pr_err("%s: BL fail startup r=%d\n", __func__, retval);
		goto loader_exit;
	}
	retval = _cyttsp_wait_bl_ready(ts, 100, 100);
	if (retval < 0) {
		pr_err("%s: BL timeout startup\n", __func__);
		goto loader_exit;
	}
	retval = _cyttsp_load_bl_regs(ts);
	if (retval < 0) {
		pr_err("%s: BL Read error\n", __func__);
		goto loader_exit;
	}
	if ((ts->bl_data.bl_status & CY_BL_BUSY) ||
		(ts->bl_data.bl_error != CY_BL_RUNNING)) {
		/* signal a status err */
		pr_err("%s: BL READY ERR on enter "
			"bl_file=%02X bl_status=%02X bl_error=%02X\n",
			__func__, ts->bl_data.bl_file,
			ts->bl_data.bl_status, ts->bl_data.bl_error);
		retval = -1;
		goto loader_exit;
	} else
		loc += CY_BL_ENTER_CMD_SIZE;

	/* send bootload firmware load blocks */
	pr_info("%s: Send BL Blocks\n", __func__);
	while ((fw_size - loc) > CY_BL_WR_BLK_CMD_SIZE) {
		cyttsp_dbg(ts, CY_DBG_LVL_2, "%s: BL_LOAD block=%d "
			"f=%02X s=%02X e=%02X loc=%d\n", __func__,
			loc / CY_BL_WR_BLK_CMD_SIZE,
			ts->bl_data.bl_file, ts->bl_data.bl_status,
			ts->bl_data.bl_error, loc);
		retval = _cyttsp_wr_blk_chunks(ts, CY_REG_BASE,
			CY_BL_WR_BLK_CMD_SIZE, &fw[loc]);
		if (retval) {
			pr_err("%s: BL_LOAD fail "
				"r=%d block=%d loc=%d\n", __func__, retval,
				loc / CY_BL_WR_BLK_CMD_SIZE, loc);
			goto loader_exit;
		}

		retval = _cyttsp_load_bl_regs(ts);
		if (retval < 0) {
			pr_err("%s: BL Read error\n", __func__);
			goto loader_exit;
		}
		if ((ts->bl_data.bl_status & CY_BL_BUSY) ||
			(ts->bl_data.bl_error != CY_BL_RUNNING)) {
			/* signal a status err */
			pr_err("%s: BL READY ERR on write block"
				" bl_file=%02X bl_status=%02X bl_error=%02X\n",
				__func__, ts->bl_data.bl_file,
				ts->bl_data.bl_status, ts->bl_data.bl_error);
			retval = -1;
			goto loader_exit;
		} else
			loc += CY_BL_WR_BLK_CMD_SIZE;
	}

	/* send bootload terminate command */
	pr_info("%s: Send BL Terminate\n", __func__);
	if (loc == (fw_size - CY_BL_EXIT_CMD_SIZE)) {
		ts->bl_ready_flag = false;
		retval = ttsp_write_block_data(ts, CY_REG_BASE,
			CY_BL_EXIT_CMD_SIZE, (void *)(&fw[loc]));
		if (retval) {
			pr_err("%s: BL fail Terminate r=%d\n",
				__func__, retval);
			goto loader_exit;
		}
		retval = _cyttsp_wait_bl_ready(ts, 100, 10);
		if (retval < 0) {
			pr_err("%s: BL Terminate timeout\n", __func__);
			if (IS_VALID_APP(ts->bl_data.bl_status))
				retval = 0;
			goto loader_exit;
		}
		retval = _cyttsp_load_bl_regs(ts);
		if (retval < 0) {
			pr_err("%s: BL Read error\n", __func__);
			goto loader_exit;
		}
		if ((ts->bl_data.bl_status & CY_BL_BUSY) ||
			(ts->bl_data.bl_error == CY_BL_RUNNING)) {
			/* signal a status err */
			pr_err("%s: BL READY ERR on exit"
				"bl_status=%02X bl_error=%02X\n",
				__func__,
				ts->bl_data.bl_status,
				ts->bl_data.bl_error);
			retval = -1;
			goto loader_exit;
		} else
			retval = 0;
	} else {
		pr_err("%s: FW size mismatch\n", __func__);
		retval = -1;
		goto loader_exit;
	}

loader_exit:
	ts->power_state = CY_BL_STATE;

	return retval;
}

#if 0  /* FUJITSU:2011-08-31 del start */
static int _cyttsp_boot_loader(struct cyttsp *ts)
{
	int retval = 0;

	if (ts->power_state == CY_SLEEP_STATE) {
		pr_err("%s: cannot load firmware in sleep state\n",
			__func__);
		retval = 0;
	} else if (!ts->platform_data->fw->ver || !ts->platform_data->fw->img) {
		cyttsp_dbg(ts, CY_DBG_LVL_3,
			"%s: empty version list or no image\n",
			__func__);
		retval = 0;
	} else if (ts->platform_data->fw->vsize != CY_BL_VERS_SIZE) {
		pr_err("%s: bad fw version list size=%d\n",
			__func__, ts->platform_data->fw->vsize);
		retval = 0;
	} else if (ts->platform_data->fw->size != CY_BL_FW_IMG_SIZE) {
		pr_err("%s: bad file image size=%d expect=%d\n",
			__func__,
			ts->platform_data->fw->size, CY_BL_FW_IMG_SIZE);
		retval = 0;
	} else {
		bool new_vers = false;
		struct cyttsp_vers *fw;

		/* automatically update firmware if new version detected */
		fw = (struct cyttsp_vers *)ts->platform_data->fw->ver;
		new_vers =
			(((u16)fw->app_verh << 8) +
			(u16)fw->app_verl) >
			(((u16)ts->bl_data.appver_hi << 8) +
			(u16)ts->bl_data.appver_lo);

		if (ts->flags & CY_FORCE_LOAD ||
			!IS_VALID_APP(ts->bl_data.bl_status) || new_vers) {
			retval = _cyttsp_load_app(ts,
				ts->platform_data->fw->img,
				ts->platform_data->fw->size);
			if (retval < 0) {
				pr_err("%s: bus fail on load fw\n", __func__);
				ts->power_state = CY_IDLE_STATE;
				cyttsp_pr_state(ts);
			} else {
				/* reset TTSP Device back to bootloader mode */
				retval = _cyttsp_hard_reset(ts);
			}
		}
	}

	return retval;
}
#endif /* FUJITSU:2011-08-31 del end */


/* Driver version */
static ssize_t cyttsp_drv_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver: %s\nVersion: %s\nDate: %s\n",
		ts->input->name, CY_DRIVER_VERSION, CY_DRIVER_DATE);
}
static DEVICE_ATTR(drv_ver, S_IRUGO, cyttsp_drv_ver_show, NULL);

/* Driver status */
static ssize_t cyttsp_drv_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver state is %s\n",
		cyttsp_powerstate_string[ts->power_state]);
}
static DEVICE_ATTR(drv_stat, S_IRUGO, cyttsp_drv_stat_show, NULL);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Group Number */
static ssize_t cyttsp_ic_grpnum_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Group: %d\n", ts->ic_grpnum);
}
static ssize_t cyttsp_ic_grpnum_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	int retval = 0;

	mutex_lock(&ts->data_lock);
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp_ic_grpnum_store_error_exit;
	}

	if (value > 0xFF)
		value = 0xFF;
	ts->ic_grpnum = value;

cyttsp_ic_grpnum_store_error_exit:
	retval = size;
	mutex_unlock(&ts->data_lock);
	return retval;
}
static DEVICE_ATTR(ic_grpnum, S_IRUSR | S_IWUSR,
	cyttsp_ic_grpnum_show, cyttsp_ic_grpnum_store);

/* Group Offset */
static ssize_t cyttsp_ic_grpoffset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Offset: %u\n", ts->ic_grpoffset);
}
static ssize_t cyttsp_ic_grpoffset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;

	mutex_lock(&ts->data_lock);
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp_ic_grpoffset_store_error_exit;
	}

	if (value > 0xFF)
		value = 0xFF;
	ts->ic_grpoffset = value;

cyttsp_ic_grpoffset_store_error_exit:
	retval = size;
	mutex_unlock(&ts->data_lock);
	return retval;
}
static DEVICE_ATTR(ic_grpoffset, S_IRUSR | S_IWUSR,
	cyttsp_ic_grpoffset_show, cyttsp_ic_grpoffset_store);

/* Group Data */
static ssize_t cyttsp_ic_grpdata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int i;
	int retval = 0;
	int num_read = 0;
	int start_read = 0;
	bool restore_op_mode = false;
	u8 ic_buf[sizeof(ts->xy_data)];

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: grpnum=%d grpoffset=%u\n",
		__func__, ts->ic_grpnum, ts->ic_grpoffset);

	if (!(ts->ic_grpnum < CY_IC_GRPNUM_NUM)) {
		pr_err("%s: Group %d does not exist.\n",
			__func__, ts->ic_grpnum);
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Group %d does not exist.\n",
			ts->ic_grpnum);
	}

	if (ts->ic_grpoffset > ts->platform_data->sett[ts->ic_grpnum]->size) {
		pr_err("%s: Offset %u exceeds group size of %d\n",
			__func__, ts->ic_grpoffset, ts->ic_grpnum);
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Offset %u exceeds group size of %d\n",
			ts->ic_grpoffset, ts->ic_grpnum);
	}

	mutex_lock(&ts->data_lock);
	if ((ts->power_state == CY_ACTIVE_STATE) &&
		(ts->ic_grpnum == CY_IC_GRPNUM_SI)) {
		retval = _cyttsp_set_sysinfo_mode(ts);
		if (retval < 0) {
			pr_err("%s: Cannot access SI Group %d Data.\n",
				__func__, ts->ic_grpnum);
			mutex_unlock(&ts->data_lock);
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Cannot access SI Group %d Data.\n",
					ts->ic_grpnum);
		} else
			restore_op_mode = true;
	}

	start_read = ts->ic_grpoffset;
	num_read = ts->platform_data->sett[ts->ic_grpnum]->size - start_read;
	if (num_read) {
		retval = ttsp_read_block_data(ts,
			ts->ic_grpoffset + ts->ic_grpstart[ts->ic_grpnum],
			num_read, &ic_buf);
		if (retval < 0) {
			pr_err("%s: Cannot read Group %d Data.\n",
				__func__, ts->ic_grpnum);
			mutex_unlock(&ts->data_lock);
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Cannot read Group %d Data.\n",
				ts->ic_grpnum);
		}
	}

	if (restore_op_mode) {
		retval = _cyttsp_set_operational_mode(ts);
		if (retval < 0) {
			pr_err("%s: Cannot restore op mode\n",
				__func__);
		}
	}
	mutex_unlock(&ts->data_lock);

	snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Group %d, Offset %u:\n", ts->ic_grpnum, ts->ic_grpoffset);
	for (i = 0; i < num_read; i++) {
		snprintf(buf, CY_MAX_PRBUF_SIZE,
			"%s0x%02X\n", buf, ic_buf[i]);
	}
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"%s(%d bytes)\n", buf, num_read);
}
static ssize_t cyttsp_ic_grpdata_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;
	const char *pbuf = buf;
	u8 ic_buf[sizeof(ts->xy_data)];
	int i;
	int j;
	char scan_buf[5];
	bool restore_op_mode = false;
	int grpsize = 0;

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: grpnum=%d grpoffset=%u\n",
		__func__, ts->ic_grpnum, ts->ic_grpoffset);

	if (!(ts->ic_grpnum < CY_IC_GRPNUM_NUM)) {
		pr_err("%s: Group %d does not exist.\n",
			__func__, ts->ic_grpnum);
		return size;
	}

	if (ts->ic_grpoffset > ts->platform_data->sett[ts->ic_grpnum]->size) {
		pr_err("%s: Offset %u exceeds group size of %d\n",
			__func__, ts->ic_grpoffset, ts->ic_grpnum);
		return size;
	}

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: pbuf=%p buf=%p size=%d sizeof(scan_buf)=%d\n",
		__func__, pbuf, buf, size, sizeof(scan_buf));
	i = 0;
	while (pbuf <= ((buf + size) - (sizeof(scan_buf)-1))) {
		while (((*pbuf == ' ') || (*pbuf == ',')) &&
			(pbuf < ((buf + size) - 4)))
			pbuf++;
		if (pbuf <= ((buf + size) - (sizeof(scan_buf)-1))) {
			memset(scan_buf, 0, sizeof(scan_buf));
			for (j = 0; j <  sizeof(scan_buf)-1; j++)
				scan_buf[j] = *pbuf++;
			retval = strict_strtoul(scan_buf, 16, &value);
			if (retval < 0) {
				pr_err("%s: Invalid data format. "
					"Use \"0xHH,...,0xHH\" instead.\n",
					__func__);
				goto cyttsp_ic_grpdata_store_error_exit;
			} else {
				ic_buf[i] = value;
				cyttsp_dbg(ts, CY_DBG_LVL_3,
					"%s: ic_buf[%d] = 0x%02X\n",
					__func__, i, ic_buf[i]);
				i++;
			}
		} else
			break;
	}

	mutex_lock(&ts->data_lock);
	if ((ts->power_state == CY_ACTIVE_STATE) &&
		(ts->ic_grpnum == CY_IC_GRPNUM_SI)) {
		retval = _cyttsp_set_sysinfo_mode(ts);
		if (retval < 0) {
			pr_err("%s: Cannot write SI Group Data.\n", __func__);
			goto cyttsp_ic_grpdata_store_error_exit;
		} else
			restore_op_mode = true;
	}

	grpsize = ts->platform_data->sett[ts->ic_grpnum]->size;
	cyttsp_dbg(ts, CY_DBG_LVL_2,
		"%s: write %d bytes at grpnum=%d grpoffset=%u "
		"grpsize=%d\n",
		__func__, i, ts->ic_grpnum, ts->ic_grpoffset, grpsize);
	i = i > grpsize - ts->ic_grpoffset ? grpsize - ts->ic_grpoffset : i;
	retval = ttsp_write_block_data(ts,
		ts->ic_grpoffset + ts->ic_grpstart[ts->ic_grpnum], i, &ic_buf);
	if (retval < 0)
		pr_err("%s: Err write numbytes=%d\n", __func__, i);
	else {
		grpsize = i;
		for (i = 0; i < grpsize; i++) {
			cyttsp_dbg(ts, CY_DBG_LVL_2,
				"%s: %02X\n", __func__, ic_buf[i]);
		}
		cyttsp_dbg(ts, CY_DBG_LVL_2,
			"%s: (%d bytes)\n", __func__, grpsize);
	}

	if (restore_op_mode) {
		retval = _cyttsp_set_operational_mode(ts);
		if (retval < 0) {
			pr_err("%s: Cannot restore operating mode.\n",
				__func__);
		}
	}

cyttsp_ic_grpdata_store_error_exit:
	mutex_unlock(&ts->data_lock);
	retval = size;
	return retval;
}
static DEVICE_ATTR(ic_grpdata, S_IRUSR | S_IWUSR,
	cyttsp_ic_grpdata_show, cyttsp_ic_grpdata_store);

static ssize_t cyttsp_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Application Version: 0x%02X%02X\n"
		"TTSP Version:        0x%02X%02X\n"
		"Applicaton ID:       0x%02X%02X\n"
		"Custom ID:           0x%02X%02X%02X\n",
		ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
		ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
		ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
		ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1],
		ts->sysinfo_data.cid[2]);
}
static DEVICE_ATTR(ic_ver, S_IRUSR | S_IWUSR,
	cyttsp_ic_ver_show, NULL);


static ssize_t cyttsp_ic_irqstat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	struct cyttsp *ts = dev_get_drvdata(dev);

	if (ts->platform_data->irq_stat) {
		retval = ts->platform_data->irq_stat();
		switch (retval) {
		case 0:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is LOW.\n");
		case 1:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is HIGH.\n");
		default:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Function irq_stat() returned %d.\n", retval);
		}
	} else {
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Function irq_stat() undefined.\n");
	}
}
static DEVICE_ATTR(ic_irqstat, S_IRUSR | S_IWUSR,
	cyttsp_ic_irqstat_show, NULL);
#endif


#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Disable Driver IRQ */
static ssize_t cyttsp_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver IRQ is %s\n", ts->irq_enabled ? "enabled" : "disabled");
}
static ssize_t cyttsp_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval = 0;
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned long value;

	mutex_lock(&ts->data_lock);

	if (size > 2) {
		pr_err("%s: Err, data too large\n", __func__);
		retval = -EOVERFLOW;
		goto cyttsp_drv_irq_store_error_exit;
	}

	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp_drv_irq_store_error_exit;
	}

	if (ts->irq_enabled == false) {
		if (value == 1) {
			/* Enable IRQ */
			enable_irq(ts->irq);
#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
			mod_timer(&ts->timer, jiffies + CY_TIMEOUT);
#endif
			pr_info("%s: Driver IRQ now enabled\n", \
				__func__);
			ts->irq_enabled = true;
		} else {
			pr_info("%s: Driver IRQ already disabled\n", __func__);
		}
	} else {
		if (value == 0) {
			/* Disable IRQ */
			disable_irq_nosync(ts->irq);
#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
			del_timer(&ts->timer);
			cancel_work_sync(&ts->work);
#endif
			pr_info("%s: Driver IRQ now disabled\n", __func__);
			ts->irq_enabled = false;
		} else {
			pr_info("%s: Driver IRQ already enabled\n", __func__);
		}
	}

	retval = size;

cyttsp_drv_irq_store_error_exit:
	mutex_unlock(&ts->data_lock);
	return retval;
}
static DEVICE_ATTR(drv_irq, S_IRUSR | S_IWUSR,
	cyttsp_drv_irq_show, cyttsp_drv_irq_store);
#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Force firmware upgrade */
static int _cyttsp_firmware_class_loader(struct cyttsp *ts,
	const u8 *data, int size)
{
	int retval = 0;
	const u8 *img = data + data[0] + 1;
	int img_size = size - (data[0] + 1);

	if (img_size != CY_BL_FW_IMG_SIZE) {
		pr_err("%s: bad file image size=%d expect=%d\n",
			__func__, img_size, CY_BL_FW_IMG_SIZE);
		retval = 0;
	} else {
		retval = _cyttsp_load_bl_regs(ts);
		if (retval < 0) {
			pr_err("%s: Fail read bootloader\n", __func__);
			goto _cyttsp_firmware_class_loader_error;
		}
		ts->power_state = CY_BL_STATE;
		retval = _cyttsp_hard_reset(ts);
		if (retval < 0) {
			pr_err("%s: Fail reset device on loader\n",
				__func__);
			goto _cyttsp_firmware_class_loader_error;
		}
		retval = _cyttsp_load_bl_regs(ts);
		if (retval < 0) {
			pr_err("%s: Fail read bootloader\n",
				__func__);
			goto _cyttsp_firmware_class_loader_error;
		}

		pr_info("%s: call load_app\n", __func__);
		retval = _cyttsp_load_app(ts, img, size - (data[0] + 1));
		if (retval < 0) {
			pr_err("%s: fail on load fw r=%d\n",
				__func__, retval);
			goto _cyttsp_firmware_class_loader_error;
		}
	}
	goto _cyttsp_firmware_class_loader_exit;


_cyttsp_firmware_class_loader_error:
	ts->power_state = CY_IDLE_STATE;
_cyttsp_firmware_class_loader_exit:
	cyttsp_pr_state(ts);
	return retval;
}
static void cyttsp_firmware_cont(const struct firmware *fw, void *context)
{
	int retval = 0;

	struct device *dev = context;

	struct cyttsp *ts = dev_get_drvdata(dev);

	if (!fw) {
		pr_err("%s: firmware not found\n", __func__);
		goto cyttsp_firmware_cont_exit;
	}

	mutex_lock(&ts->data_lock);

	retval = _cyttsp_firmware_class_loader(ts, fw->data, fw->size);

	if (!retval) {
		retval = _cyttsp_startup(ts);
		if (retval < 0) {
			pr_info("%s: return from _cyttsp_startup after"
			" fw load r=%d\n",
			__func__, retval);
		}
	}


	mutex_unlock(&ts->data_lock);

	release_firmware(fw);

cyttsp_firmware_cont_exit:
	ts->waiting_for_fw = false;
	return;
}
static ssize_t cyttsp_ic_reflash_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"%s\n", ts->waiting_for_fw ?
		"Driver is waiting for firmware load" :
		"No firmware loading in progress");
}
static ssize_t cyttsp_ic_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int i;
	int retval = 0;
	struct cyttsp *ts = dev_get_drvdata(dev);

	if (ts->waiting_for_fw) {
		pr_err("%s: Driver is already waiting for firmware\n",
			__func__);
		retval = -EALREADY;
		goto cyttsp_ic_reflash_store_exit;
	}

	/*
	 * must configure FW_LOADER in .config file
	 * CONFIG_HOTPLUG=y
	 * CONFIG_FW_LOADER=y
	 * CONFIG_FIRMWARE_IN_KERNEL=y
	 * CONFIG_EXTRA_FIRMWARE=""
	 * CONFIG_EXTRA_FIRMWARE_DIR=""
	 */

	if (size > CY_BL_FW_NAME_SIZE) {
		pr_err("%s: Filename too long\n", __func__);
		retval = -ENAMETOOLONG;
		goto cyttsp_ic_reflash_store_exit;
	} else {
		/*
		 * name string must be in alloc() memory
		 * or is lost on context switch
		 * strip off any line feed character(s)
		 * at the end of the buf string
		 */
		for (i = 0; buf[i]; i++) {
			if (buf[i] < ' ')
				ts->fwname[i] = 0;
			else
				ts->fwname[i] = buf[i];
		}
	}

	pr_info("%s: Enabling firmware class loader\n", __func__);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
	retval = request_firmware_nowait(THIS_MODULE,
		FW_ACTION_NOHOTPLUG, (const char *)ts->fwname, ts->dev,
		GFP_KERNEL, ts->dev, cyttsp_firmware_cont);
#else /* Kernel 2.6.32 */
	retval = request_firmware_nowait(THIS_MODULE,
		FW_ACTION_NOHOTPLUG, (const char *)ts->fwname, ts->dev,
		ts->dev, cyttsp_firmware_cont);
#endif
	mutex_lock(&ts->data_lock);
	if (retval) {
		pr_err("%s: Fail request firmware class file load\n",
			__func__);
		ts->waiting_for_fw = false;
		goto cyttsp_ic_reflash_store_exit;
	} else {
		ts->waiting_for_fw = true;
		retval = size;
	}
	mutex_unlock(&ts->data_lock);

cyttsp_ic_reflash_store_exit:
	return retval;
}
static DEVICE_ATTR(ic_reflash, S_IRUSR | S_IWUSR,
	cyttsp_ic_reflash_show, cyttsp_ic_reflash_store);
#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Driver debugging */
static ssize_t cyttsp_drv_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

#ifdef CY_USE_DEBUG_TOOLS
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Debug Setting: %u flip=%d inv-x=%d inv-y=%d"
		"Driver name=%s ver=%s date=%s\n",
		ts->bus_ops->tsdebug,
		(int)(!!(ts->flags & CY_FLIP)),
		(int)(!!(ts->flags & CY_INV_X)),
		(int)(!!(ts->flags & CY_INV_Y)),
		CY_I2C_NAME, CY_DRIVER_VERSION, CY_DRIVER_DATE);
#else
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Debug Setting: %u\n", ts->bus_ops->tsdebug);
#endif
}
static ssize_t cyttsp_drv_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int retval = 0;
	unsigned long value;

	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp_drv_debug_store_exit;
	}

	switch (value) {
	case CY_DBG_LVL_0:
	case CY_DBG_LVL_1:
	case CY_DBG_LVL_2:
	case CY_DBG_LVL_3:
		pr_info("%s: Debug setting=%d\n", __func__, (int)value);
		ts->bus_ops->tsdebug = value;
		break;
#ifdef CY_USE_DEBUG_TOOLS
	case CY_DBG_SUSPEND:
		pr_info("%s: SUSPEND (ts=%p)\n", __func__, ts);
		cyttsp_suspend(ts);
		break;
	case CY_DBG_RESUME:
		pr_info("%s: RESUME (ts=%p)\n", __func__, ts);
		cyttsp_resume(ts);
		break;
	case CY_DBG_RESET:
		pr_info("%s: RESET (ts=%p)\n",
			__func__, ts);
		mutex_lock(&ts->data_lock);
		retval = _cyttsp_startup(ts);
		mutex_unlock(&ts->data_lock);
		pr_info("%s: return from _cyttsp_startup test r=%d\n",
			__func__, retval);
		break;
	case CY_DBG_FLIP:
		pr_info("%s: FLIP ON (ts=%p)\n", __func__, ts);
		ts->flags |= CY_FLIP;
		break;
	case CY_DBG_INV_X:
		pr_info("%s: INV X ON (ts=%p)\n", __func__, ts);
		ts->flags |= CY_INV_X;
		break;
	case CY_DBG_INV_Y:
		pr_info("%s: INV Y ON (ts=%p)\n", __func__, ts);
		ts->flags |= CY_INV_Y;
		break;
	case CY_DBG_NOINV:
		pr_info("%s: FLIP OFF (ts=%p)\n", __func__, ts);
		pr_info("%s: INV X OFF (ts=%p)\n", __func__, ts);
		pr_info("%s: INV Y OFF (ts=%p)\n", __func__, ts);
		ts->flags &= ~(CY_FLIP | CY_INV_X | CY_INV_Y);
		break;
#endif
	default:
		ts->bus_ops->tsdebug = CY_DBG_LVL_MAX;
		pr_err("%s: Invalid debug setting; set to max=%d\n",
			__func__, ts->bus_ops->tsdebug);
		break;
	}

	retval = size;

cyttsp_drv_debug_store_exit:
	return retval;
}
static DEVICE_ATTR(drv_debug, S_IRUSR | S_IWUSR,
	cyttsp_drv_debug_show, cyttsp_drv_debug_store);
#endif

#ifdef CY_USE_REG_ACCESS
static ssize_t cyttsp_drv_rw_regid_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Read/Write Regid=%02X(%d)\n",
		ts->rw_regid, ts->rw_regid);
}
static ssize_t cyttsp_drv_rw_regid_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int retval = 0;
	unsigned long value;

	mutex_lock(&ts->data_lock);
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		retval = strict_strtoul(buf, 16, &value);
		if (retval < 0) {
			pr_err("%s: Failed to convert value\n",
				__func__);
			goto cyttsp_drv_rw_regid_store_exit;
		}
	}

	if (value > CY_RW_REGID_MAX) {
		ts->rw_regid = CY_RW_REGID_MAX;
		pr_err("%s: Invalid Read/Write Regid; set to max=%d\n",
			__func__, ts->rw_regid);
	} else
		ts->rw_regid = value;

	retval = size;

cyttsp_drv_rw_regid_store_exit:
	mutex_unlock(&ts->data_lock);
	return retval;
}
static DEVICE_ATTR(drv_rw_regid, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
	cyttsp_drv_rw_regid_show, cyttsp_drv_rw_regid_store);


static ssize_t cyttsp_drv_rw_reg_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int retval;
	u8 reg_data;

	retval = ttsp_read_block_data(ts,
		CY_REG_BASE + ts->rw_regid, sizeof(reg_data), &reg_data);

	if (retval < 0)
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Read/Write Regid(%02X(%d) Failed\n",
			ts->rw_regid, ts->rw_regid);
	else
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Read/Write Regid=%02X(%d) Data=%02X(%d)\n",
			ts->rw_regid, ts->rw_regid, reg_data, reg_data);
}
static ssize_t cyttsp_drv_rw_reg_data_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int retval = 0;
	unsigned long value;
	u8 cmd;

	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		retval = strict_strtoul(buf, 16, &value);
		if (retval < 0) {
			pr_err("%s: Failed to convert value\n",
				__func__);
			goto cyttsp_drv_rw_reg_data_store_exit;
		}
	}

	if (value > CY_RW_REG_DATA_MAX) {
		pr_err("%s: Invalid Register Data Range; no write\n",
			__func__);
	} else {
		cmd = (u8)value;
		retval = ttsp_write_block_data(ts, CY_REG_BASE + ts->rw_regid,
			sizeof(cmd), &cmd);
		if (retval < 0) {
			pr_err("%s: Failed write to Regid=%02X(%d)\n",
				__func__, ts->rw_regid, ts->rw_regid);
		}
	}

	retval = size;

cyttsp_drv_rw_reg_data_store_exit:
	return retval;
}
static DEVICE_ATTR(drv_rw_reg_data, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
	cyttsp_drv_rw_reg_data_show, cyttsp_drv_rw_reg_data_store);
#endif

static void cyttsp_ldr_init(struct cyttsp *ts)
{
	if (device_create_file(ts->dev, &dev_attr_drv_ver))
		pr_err("%s: Cannot create drv_ver\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_stat))
		pr_err("%s: Cannot create drv_stat\n", __func__);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (device_create_file(ts->dev, &dev_attr_drv_debug))
		pr_err("%s: Cannot create drv_debug\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_irq))
		pr_err("%s: Cannot create drv_irq\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpdata))
		pr_err("%s: Cannot create ic_grpdata\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpnum))
		pr_err("%s: Cannot create ic_grpnum\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpoffset))
		pr_err("%s: Cannot create ic_grpoffset\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_reflash))
		pr_err("%s: Cannot create ic_reflash\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_ver))
		pr_err("%s: Cannot create ic_ver\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_irqstat))
		pr_err("%s: Cannot create ic_irqstat\n", __func__);
#endif
#ifdef CY_USE_REG_ACCESS
	if (device_create_file(ts->dev, &dev_attr_drv_rw_regid))
		pr_err("%s: Cannot create drv_rw_regid\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_rw_reg_data))
		pr_err("%s: Cannot create drv_rw_reg_data\n", __func__);
#endif

	return;
}

static void cyttsp_ldr_free(struct cyttsp *ts)
{
	device_remove_file(ts->dev, &dev_attr_drv_ver);
	device_remove_file(ts->dev, &dev_attr_drv_stat);
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	device_remove_file(ts->dev, &dev_attr_drv_debug);
	device_remove_file(ts->dev, &dev_attr_drv_irq);
	device_remove_file(ts->dev, &dev_attr_ic_irqstat);
	device_remove_file(ts->dev, &dev_attr_ic_grpdata);
	device_remove_file(ts->dev, &dev_attr_ic_grpnum);
	device_remove_file(ts->dev, &dev_attr_ic_grpoffset);
	device_remove_file(ts->dev, &dev_attr_ic_reflash);
	device_remove_file(ts->dev, &dev_attr_ic_ver);
#endif
#ifdef CY_USE_REG_ACCESS
	device_remove_file(ts->dev, &dev_attr_drv_rw_regid);
	device_remove_file(ts->dev, &dev_attr_drv_rw_reg_data);
#endif
}

static int _cyttsp_startup(struct cyttsp *ts)
{
	int retval;

	/* FUJITSU:2011-08-31 add start */
	printk("[TPD]%s: Cypress TouchDriver Startup\n", __func__);
	/* FUJITSU:2011-08-31 add end */

	retval = _cyttsp_hard_reset(ts);
	if (retval < 0) {
		pr_err("%s: HW reset failure\n", __func__);
		ts->power_state = CY_INVALID_STATE;
		cyttsp_pr_state(ts);
		goto bypass;
	}
	retval = _cyttsp_bl_app_valid(ts);
	switch (ts->power_state) {
	case CY_ACTIVE_STATE:
		goto no_bl_bypass;
		break;
	case CY_BL_STATE:
		goto normal_exit_bl;
		break;
	case CY_IDLE_STATE:
		goto bypass;
		break;
	default:
		goto bypass;
		break;
	}

normal_exit_bl:
#if 0  /* FUJITSU:2011-08-31 del start */
	retval = _cyttsp_boot_loader(ts);
	if (retval < 0)
		goto bypass;  /* We are now in IDLE state */
#endif /* FUJITSU:2011-08-31 del end */

	retval = _cyttsp_exit_bl_mode(ts);
	if (retval < 0)
		goto bypass;

	if (ts->power_state != CY_READY_STATE)
		goto bypass;

no_bl_bypass:
	/* At this point, we are in READY to go active state */
	retval = _cyttsp_set_sysinfo_mode(ts);
	if (retval < 0)
		goto op_bypass;

	retval = _cyttsp_set_sysinfo_regs(ts);
	if (retval < 0)
		goto bypass;

op_bypass:
	retval = _cyttsp_set_operational_mode(ts);
	if (retval < 0)
		goto bypass;

	/* init active distance */
#if 0 /* FUJITSU:2011-08-31 del start */
	retval = _cyttsp_set_operational_regs(ts);
	if (retval < 0)
		goto bypass;
#endif /* FUJITSU:2011-08-31 del end */
	ts->power_state = CY_ACTIVE_STATE;
	/* FUJITSU:2011-08-31 add start */
	printk("[TPD]%s: Cypress TouchDriver is Active\n", __func__);
	/* FUJITSU:2011-08-31 add end */
bypass:
	return retval;
}

static irqreturn_t cyttsp_irq(int irq, void *handle)
{
	struct cyttsp *ts = handle;
	int retval;
	/* FUJITSU:2011-10-17 add start */
	struct timeval tv;

	del_timer(&ts->timer_up);
	if(CY_DBG_LVL_1 < ts->bus_ops->tsdebug) {
		do_gettimeofday(&tv);
		cyttsp_dbg(ts, CY_DBG_LVL_2, "[TPD_Tm] %s: IN  time:%lu%06lu usec\n",
			__func__, tv.tv_sec, tv.tv_usec);
	}
	/* FUJITSU:2011-10-17 add end */

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: GOT IRQ ps=%d\n", __func__, ts->power_state);

	/* FUJITSU:2011-09-27 del start */
//	mutex_lock(&ts->data_lock);
	/* FUJITSU:2011-09-27 del end */

	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: DO IRQ ps=%d\n", __func__, ts->power_state);

	switch (ts->power_state) {
	case CY_BL_STATE:
		complete(&ts->bl_int_running);
		break;
	case CY_SYSINFO_STATE:
		complete(&ts->si_int_running);
		break;
	case CY_LDR_STATE:
		ts->bl_ready_flag = true;
		break;
	case CY_IDLE_STATE:
		/* device now available; signal initialization */
		pr_info("%s: Received IRQ in IDLE state\n",
			__func__);
		/* Try to determine the IC's current state */
		retval = _cyttsp_load_bl_regs(ts);
		if (retval < 0) {
			pr_err("%s: Still unable to find IC after IRQ\n",
				__func__);
		} else if (GET_BOOTLOADERMODE(ts->bl_data.bl_status)) {
			pr_info("%s: BL mode found in IDLE state\n",
				__func__);
			ts->power_state = CY_BL_STATE;
			cyttsp_pr_state(ts);
		} else {
			pr_info("%s: ACTIVE mode found in IDLE state\n",
				__func__);
			/* FUJITSU:2011-09-27 add start */
			mutex_lock(&ts->data_lock);
			/* FUJITSU:2011-09-27 add end */
			ts->power_state = CY_ACTIVE_STATE;
			cyttsp_pr_state(ts);
#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
			mod_timer(&ts->timer, jiffies + CY_TIMEOUT);
#endif
			retval = _cyttsp_xy_worker(ts);
			/* FUJITSU:2011-09-27 add start */
			mutex_unlock(&ts->data_lock);
			/* FUJITSU:2011-09-27 add end */
		}
		break;
	case CY_ACTIVE_STATE:
		/* FUJITSU:2011-09-27 add start */
		mutex_lock(&ts->data_lock);
		/* FUJITSU:2011-09-27 add end */
#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
		mod_timer(&ts->timer, jiffies + CY_TIMEOUT);
#endif
		retval = _cyttsp_xy_worker(ts);
		/* FUJITSU:2011-09-27 add start */
		mutex_unlock(&ts->data_lock);
		/* FUJITSU:2011-09-27 add end */
		/* FUJITSU:2011-10-02 mod start */
//		if (ts->power_state == CY_BL_STATE) {
		if (ts->power_state == CY_BL_STATE || ts->power_state == CY_IDLE_STATE) {
			pr_info("%s: Reached BL/IDLE state in IRQ PS=%d\n",
				__func__, ts->power_state);
		/* FUJITSU:2011-10-02 mod end */
			/*
			 * TTSP device has reset back to bootloader mode
			 * Reset driver touch history and restore
			 * operational mode
			 */
			/* FUJITSU:2011-08-11 add start */
			memset(ts->mt_trc_id, -1, sizeof(ts->mt_trc_id));
			/* FUJITSU:2011-08-11 add end */
			memset(ts->prv_trk, CY_NTCH, sizeof(ts->prv_trk));
			/* FUJITSU:2011-01-10 mod start */
//			retval = _cyttsp_startup(ts);
			del_timer(&ts->timer);
			cancel_work_sync(&ts->work);
			cyttsp_recovery(ts);
			/* FUJITSU:2011-01-10 mod end */
		}
		break;
	default:
		break;
	}

	/* FUJITSU:2011-09-27 del start */
//	mutex_unlock(&ts->data_lock);
	/* FUJITSU:2011-09-27 del end */

	/* FUJITSU:2011-08-11 add start */
	if(CY_DBG_LVL_1 < ts->bus_ops->tsdebug) {
		do_gettimeofday(&tv);
		cyttsp_dbg(ts, CY_DBG_LVL_2, "[TPD_Tm] %s: OUT time:%lu%06lu usec\n",
			__func__, tv.tv_sec, tv.tv_usec);
	}
	/* FUJITSU:2011-08-11 add end */
	return IRQ_HANDLED;
}

static int cyttsp_power_on(struct cyttsp *ts)
{
	int retval = 0;

	/* Communicate with the IC */
	mutex_lock(&ts->data_lock);
	retval = _cyttsp_startup(ts);
	mutex_unlock(&ts->data_lock);

#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
	mod_timer(&ts->timer, jiffies + CY_TIMEOUT);
#endif
	return retval;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
int cyttsp_resume(void *handle)
{
	struct cyttsp *ts = handle;
	int retval = 0;
	/* FUJITSU:2011-10-02 mod start */
//	int tries;
	int tries = 0;
	/* FUJITSU:2011-10-02 mod end */
	struct cyttsp_xydata xydata;

	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: Resuming...", __func__);

	mutex_lock(&ts->data_lock);
	if (ts->power_state != CY_SLEEP_STATE) {
		pr_err("%s: Already resumed\n", __func__);
		goto cyttsp_resume_exit;
	}

	if ((ts->flags & CY_USE_SLEEP) &&
		(ts->power_state != CY_ACTIVE_STATE)) {
		ts->irq_enabled = true;
		enable_irq(ts->irq);
		/* FUJITSU:2011-09-27 add start */
		if (ts->platform_data->hw_recov) {
			/* wake up device with a strobe */
			ts->platform_data->hw_recov(1);
		}
		/* FUJITSU:2011-09-27 add end */
		cyttsp_dbg(ts, CY_DBG_LVL_3,
			   "%s: default wake up device with a read\n",
			   __func__);
#if 0  /* FUJITSU:2011-10-02 del start */
		retval = ttsp_read_block_data(ts, CY_REG_BASE,
			sizeof(xydata), &xydata);
		tries = 0;
		do {
			msleep(CY_DELAY_DFLT);
			retval = ttsp_read_block_data(ts, CY_REG_BASE,
				sizeof(xydata), &xydata);
		} while ((retval < 0) || ((!(retval < 0) &&
			((xydata.hst_mode &
			~(CY_HNDSHK_BIT | CY_HST_MODE_CHANGE_BIT))
			+ (xydata.tt_mode & ~0xC0) +
			xydata.tt_stat)) && (tries++ < CY_DELAY_MAX)));
#endif /* FUJITSU:2011-10-02 del end */

		/* FUJITSU:2011-10-02 add start */
		retval = ts->bus_ops->read(ts->bus_ops, CY_REG_BASE, sizeof(xydata), &xydata);
		if(retval < 0) {
			mdelay(CY_I2C_NACK_WAIT_VALUE);
			for(tries = 0; tries < 3 ; tries++) {
				retval = ts->bus_ops->read(ts->bus_ops, CY_REG_BASE, sizeof(xydata), &xydata);
				if(retval >= 0) {
					break;
				} else {
					cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: I2C Read Err try=%d\n", __func__, tries);
					mdelay(10);
				}
			}
		}

		if(retval < 0) {
			pr_err("%s: Read Block fail (r=%d)\n", __func__, retval);
			cyttsp_recovery(ts);
			goto cyttsp_resume_exit;
		}
		/* FUJITSU:2011-10-02 add end */

		/* FUJITSU:2011-10-02 mod start */
		if (GET_BOOTLOADERMODE(xydata.tt_mode)) {
//			ts->power_state = CY_IDLE_STATE;
			ts->power_state = CY_BL_STATE;
			pr_err("%s: BL mode detected in sleep state\n", __func__);
//			retval = _cyttsp_startup(ts);
			cyttsp_recovery(ts);
			goto cyttsp_resume_exit;
		} else if (GET_HSTMODE(xydata.hst_mode) == CY_OPERATE_MODE) {
			ts->power_state = CY_ACTIVE_STATE;
		}
		/* FUJITSU:2011-10-02 mod end */

		cyttsp_dbg(ts, CY_DBG_LVL_2,
			"%s: Wake up: %s hst_mode=%02X tt_mode=%02X "
			"tt_status=%02X tries=%d ret=%d\n",
			__func__, (retval < 0) ? "FAIL" : "PASS",
			xydata.hst_mode, xydata.tt_mode,
			xydata.tt_stat, tries, retval);
#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
		mod_timer(&ts->timer, jiffies + CY_TIMEOUT);
#endif
	}
	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: Wake Up %s\n", __func__,
		(retval < 0) ? "FAIL" : "PASS");

	/* FUJITSU:2011-09-27 add start */
	cyttsp_pr_state(ts);
	/* FUJITSU:2011-09-27 add end */
cyttsp_resume_exit:
	mutex_unlock(&ts->data_lock);
	return retval;
}
EXPORT_SYMBOL_GPL(cyttsp_resume);

int cyttsp_suspend(void *handle)
{
	struct cyttsp *ts = handle;
	u8 sleep_mode = 0;
	int retval = 0;
	ts->timer_up_flag = false;

	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: Suspending...", __func__);

	/* FUJITSU:2011-12-27 mod start */
//	mutex_lock(&ts->data_lock);
	if (ts->power_state == CY_SLEEP_STATE) {
		pr_err("%s: Already suspended\n", __func__);
		goto cyttsp_suspend_exit;
	}

	if (ts->power_state == CY_LDR_STATE) {
		retval = -EBUSY;
		pr_err("%s: Suspend blocked - Loader busy.\n", __func__);
		goto cyttsp_suspend_exit;
	}
	disable_irq(ts->irq);
#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
	del_timer(&ts->timer);
	cancel_work_sync(&ts->work);
#endif
	del_timer(&ts->timer_up);
	mutex_lock(&ts->data_lock);
	/* FUJITSU:2011-12-27 mod end */

	if ((ts->flags & CY_USE_SLEEP) &&
		(ts->power_state == CY_ACTIVE_STATE)) {
		/* FUJITSU:2011-09-27 mod start */
//		sleep_mode = CY_DEEP_SLEEP_MODE;
		sleep_mode = CY_DEEP_SLEEP_MODE | CY_LOW_POWER_MODE;
		/* FUJITSU:2011-09-27 mod end */
		/* FUJITSU:2011-12-27 mod start */
//		disable_irq(ts->irq);
		ts->waiting_for_fw = false;
//#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
//		del_timer(&ts->timer);
//		cancel_work_sync(&ts->work);
//#endif
		/* FUJITSU:2011-12-27 mod end */
		retval = ttsp_write_block_data(ts,
			CY_REG_BASE, sizeof(sleep_mode), &sleep_mode);
		/* FUJITSU:2011-08-11 add start */
		msleep(60);
		/* FUJITSU:2011-08-11 add end */
		if (retval < 0) {
			enable_irq(ts->irq);
			ts->irq_enabled = true;
		} else
			ts->power_state = CY_SLEEP_STATE;
	}
	cyttsp_pr_state(ts);

cyttsp_suspend_exit:
	mutex_unlock(&ts->data_lock);
	return retval;
}
EXPORT_SYMBOL_GPL(cyttsp_suspend);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
void cyttsp_early_suspend(struct early_suspend *h)
{
	struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);
	int retval = 0;

	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: EARLY SUSPEND ts=%p\n", __func__, ts);
	retval = cyttsp_suspend(ts);
	if (retval < 0) {
		pr_err("%s: Early suspend failed with error code %d\n",
			__func__, retval);
	}
	/* FUJITSU:2011-10-17 add start */
	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: EARLY SUSPEND EXIT ts=%p\n", __func__, ts);
	/* FUJITSU:2011-10-17 add end */
}

void cyttsp_late_resume(struct early_suspend *h)
{
	struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);
	int retval = 0;

	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: LATE RESUME ts=%p\n", __func__, ts);
	retval = cyttsp_resume(ts);
	if (retval < 0) {
		pr_err("%s: Late resume failed with error code %d\n",
			__func__, retval);
	}
	/* FUJITSU:2011-10-17 add start */
	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: LATE RESUME EXIT ts=%p\n", __func__, ts);
	/* FUJITSU:2011-10-17 add end */
}
#endif

/* FUJITSU:2011-12-27 add start */
static void cyttsp_calibration(struct cyttsp *ts)
{
	int retval = 0;
	int count = 0;
	u8 scn_typ_cmd = 0x00;
	u8 toggle_cmd = 0x00;
	u8 set_cmd = 0x00;
	u8 rdata = 0x00;

	cyttsp_dbg(ts, CY_DBG_LVL_3, "[TPD] %s: IN\n", __func__);

	ts->power_state = CY_READY_STATE;
	cyttsp_pr_state(ts);

	/* kill watchdog */
	del_timer(&ts->timer);
	cancel_work_sync(&ts->work);

	/* switch to sysinfo mode */
	retval = _cyttsp_set_sysinfo_mode(ts);

	scn_typ_cmd = (ts->sysinfo_data.scn_typ & 0x0F);
	cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: Set Scantyp cmd = 0x%02X\n", __func__, scn_typ_cmd);
	retval = ttsp_write_block_data(ts, 0x1C, sizeof(scn_typ_cmd), &scn_typ_cmd);

	toggle_cmd = ts->sysinfo_data.hst_mode & CY_HNDSHK_BIT ?
		ts->sysinfo_data.hst_mode & ~CY_HNDSHK_BIT :
		ts->sysinfo_data.hst_mode | CY_HNDSHK_BIT;
	retval = ttsp_write_block_data(ts, CY_REG_BASE,
				       sizeof(toggle_cmd), (u8 *)&toggle_cmd);

	msleep(20);


	ts->power_state = CY_READY_STATE;
	cyttsp_pr_state(ts);

	/* Calibration Start */
	/* write data 0x23 reg 2 */
	set_cmd = 0x23;
	retval = ttsp_write_block_data(ts, CY_REG_MFG_CMD, sizeof(set_cmd), &set_cmd);
	if (retval < 0) {
		printk(KERN_ERR "calibration write data 0x23 reg 2 err:%d\n",retval);
	}

	/* write data 0x00 reg 3 */
	set_cmd = 0x00;
	retval = ttsp_write_block_data(ts, CY_REG_CID_0, sizeof(set_cmd), &set_cmd);
	if (retval < 0) {
		printk(KERN_ERR "calibration write data 0x00 reg 3 err:%d\n",retval);
	}

	/* write data 0x00 reg 4 */
	set_cmd = 0x00;
	retval = ttsp_write_block_data(ts, CY_REG_CID_1, sizeof(set_cmd), &set_cmd);
	if (retval < 0) {
		printk(KERN_ERR "calibration write data 0x00 reg 4 err:%d\n",retval);
	}

	/* write data 0x20 reg 2 */
	set_cmd = 0x20;
	retval = ttsp_write_block_data(ts, CY_REG_MFG_CMD, sizeof(set_cmd), &set_cmd);
	if (retval < 0) {
		printk(KERN_ERR "calibration write data 0x20 reg 2 err:%d\n",retval);
	}

	/* read reg 1 */
	do {
		/* 1.9s wait */
		mdelay(1900);
		count ++;
		retval = ttsp_read_block_data(ts, CY_REG_MFG_STAT,sizeof(rdata), &rdata);
		printk(KERN_INFO"CALIBRATION Data Read %x\n",rdata);
		if(count >= 3) {
			printk(KERN_INFO"count Up");
			break;
		}
	} while(0x82 != (rdata & 0x82));

	/* write data 0x22 reg 2 */
	set_cmd = 0x22;
	retval = ttsp_write_block_data(ts, CY_REG_MFG_CMD, sizeof(set_cmd), &set_cmd);
	if (retval < 0) {
		printk(KERN_ERR "calibration write data 0x22 reg 2 err:%d\n",retval);
	}

	/* switch back to Operational mode */
	set_cmd = CY_OPERATE_MODE + CY_LOW_POWER_MODE;
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(set_cmd), &set_cmd);

	msleep(300);

	retval = _cyttsp_set_sysinfo_mode(ts);
	if(retval < 0) {
		printk(KERN_ERR "CALIBRATION set sys %d",retval);
	}

	retval = _cyttsp_set_sysinfo_regs(ts);
	if(retval < 0) {
		printk(KERN_ERR "CALIBRATION set sys reg %d",retval);
	}

	set_cmd = CY_OPERATE_MODE + CY_LOW_POWER_MODE;
	set_cmd = set_cmd | (ts->sysinfo_data.hst_mode & CY_HNDSHK_BIT);
	cyttsp_dbg(ts, CY_DBG_LVL_3, 
		"[TPD]%s: Ope.mode Command = 0x%02X\n",
		__func__, set_cmd);
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(set_cmd), &set_cmd);

	ts->power_state = CY_ACTIVE_STATE;
	cyttsp_pr_state(ts);

	mod_timer(&ts->timer, jiffies + CY_TIMEOUT);

	cyttsp_dbg(ts, CY_DBG_LVL_3, "[TPD] %s: OUT\n", __func__);
	return;
}

static int cyttsp_read_reg_sequence( struct cyttsp *ts, int offset, int length, int slaveaddress, struct cyttsp_i2c_data data )
{
	int ret = 0;
	u8  rdata[length];
	int count;

//	printk( KERN_INFO "%s :[IN]\n", __func__ );

	ret = ttsp_read_block_data(ts, offset, sizeof(rdata), &rdata);

	for(count = 0; count < length; count++ ){
		data.i2c_data_buf[count] = rdata[count];
//		printk(KERN_INFO "rdata[%d] = 0x%x \n",count, rdata[count]);
		cyttsp_dbg(ts, CY_DBG_LVL_3, "rdata[%d] = 0x%x \n",count, rdata[count]);
	}

	return ret;
}

static int cyttsp_write_reg_sequence( struct cyttsp *ts, int offset, int length, int slaveaddress, struct cyttsp_i2c_data data )
{
	int ret = 0;
	u8  wdata[length];
	int count;

//	printk( KERN_INFO "%s :[IN]\n", __func__ );

	for(count = 0; count < data.length; count++ ){
		wdata[count] = data.i2c_data_buf[count];
//		printk(KERN_INFO "wdata[%d] = 0x%x \n",count, wdata[count]);
		cyttsp_dbg(ts, CY_DBG_LVL_3, "wdata[%d] = 0x%x \n",count, wdata[count]);
	}

	ret = ttsp_write_block_data(ts, offset, sizeof(wdata), &wdata);

	return ret;
}

static long cyttsp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
//	int i = 0; 
	enum devicepowerstatus mode;
	char verdata[CY_FIRMVERSION_SIZE];
	struct cyttsp *ts = file->private_data; 
	struct cyttsp_i2c_data i2c_data;
	memset(&i2c_data, 0, sizeof(i2c_data));

	switch(cmd) {
	case IOCTL_SET_POWERMODE:
		ret = copy_from_user( &mode, (int __user *) arg, sizeof(int));
		if(ret != 0) {
			printk(KERN_ERR "copy_from_user err %d \n",ret);
		}
		printk(KERN_INFO "mode %d \n", mode);

		switch(mode) {
		case DEEPSLEEP:
			printk(KERN_INFO "Enter DEEPSLEEP\n");
			cyttsp_suspend(ts);
			break;

		/* i2c wakeup */
		case WAKEUP:
			printk(KERN_INFO "Enter WAKEUP\n");
			ret = cyttsp_resume(ts);
			break;

		/* XReset wakeup */
		case HARDRESET:
			printk(KERN_INFO "Enter HARDRESET\n");
			if(ts->power_state == CY_ACTIVE_STATE) {
#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
				del_timer(&ts->timer);
				cancel_work_sync(&ts->work);
#endif
			} else if(ts->power_state == CY_SLEEP_STATE) {
				enable_irq(ts->irq);
			}
			ts->power_state = CY_READY_STATE;
			cyttsp_pr_state(ts);
			_cyttsp_startup(ts);
			mod_timer(&ts->timer, jiffies + CY_TIMEOUT);
			break;
		}
		break;

	case IOCTL_GET_FIRMVERSION:
		printk(KERN_INFO "Enter FIRMVERSION\n");

		if (ts->power_state == CY_SLEEP_STATE) {
			cyttsp_pr_state(ts);
			printk("[TPD]%s: Cannot Get Firmversion\n", __func__);
			ret = -EFAULT;
			break;
		}

#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
		del_timer(&ts->timer);
		cancel_work_sync(&ts->work);
#endif
		ts->power_state = CY_READY_STATE;
		cyttsp_pr_state(ts);

		/* switch to sysinfo mode */
		ret = _cyttsp_set_sysinfo_mode(ts);

		verdata[0] = ts->sysinfo_data.app_verh;
		verdata[1] = ts->sysinfo_data.app_verl;
		verdata[2] = ts->sysinfo_data.app_idh;
		verdata[3] = ts->sysinfo_data.app_idl;
		verdata[4] = ts->sysinfo_data.tts_verh;
		verdata[5] = ts->sysinfo_data.tts_verl;
		verdata[6] = ts->sysinfo_data.bl_verh;
		verdata[7] = ts->sysinfo_data.bl_verl;
		verdata[8] = ts->sysinfo_data.cid[0];
		verdata[9] = ts->sysinfo_data.cid[1];
		verdata[10] = ts->sysinfo_data.cid[2];
		verdata[11] = 0xFF;
		verdata[12] = 0xFF;
		verdata[13] = 0xFF;
		verdata[14] = 0xFF;
		verdata[15] = 0xFF;

		ret = _cyttsp_set_operational_mode(ts);
		mod_timer(&ts->timer, jiffies + CY_TIMEOUT);

		ret = copy_to_user( (int __user *) arg, &verdata, sizeof(verdata) );
		if(ret != 0) {
			printk(KERN_ERR "copy_to_user err %d \n",ret);
		}
		break;

	case IOCTL_DO_CALIBRATION:
		printk(KERN_INFO "Enter CALIBRATION\n");

		if (ts->power_state == CY_SLEEP_STATE) {
			cyttsp_pr_state(ts);
			printk("[TPD]%s: Cannot Calibration\n", __func__);
			ret = -EFAULT;
			break;
		}

		cyttsp_calibration(ts);
		break;

	case IOCTL_I2C_READ:
//		printk(KERN_INFO "Enter IOCTL_I2C_READ\n");

		if (ts->power_state == CY_SLEEP_STATE) {
			cyttsp_pr_state(ts);
			printk("[TPD]%s: Cannot I2C Read\n", __func__);
			ret = -EFAULT;
			break;
		}

		ret = copy_from_user( &i2c_data, (int __user *) arg, sizeof(i2c_data));
		if(ret != 0) {
			printk(KERN_ERR "copy_from_user err %d \n",ret);
		}

//		printk(KERN_INFO "i2cdata offset 0x%x length 0x%x slave 0x%x\n", i2c_data.offset, i2c_data.length, i2c_data.cyttsp_slaveaddress);
		cyttsp_dbg(ts, CY_DBG_LVL_3, "i2cdata offset 0x%x length 0x%x slave 0x%x\n", i2c_data.offset, i2c_data.length, i2c_data.cyttsp_slaveaddress);

		ret = cyttsp_read_reg_sequence( ts, i2c_data.offset, i2c_data.length, i2c_data.cyttsp_slaveaddress, i2c_data );

		if(ret != 0) {
			printk(KERN_ERR "cyttsp_read_reg_sequence err %d \n",ret);
			ret = -EIO;
			return ret;
		}

		ret = copy_to_user( (int __user *) arg, &i2c_data, sizeof(i2c_data) );
		if(ret != 0){
			printk(KERN_ERR "copy_to_user err %d \n",ret);
		}
		break;

	case IOCTL_I2C_WRITE:
//		printk(KERN_INFO "Enter IOCTL_I2C_WRITE\n");

		if (ts->power_state == CY_SLEEP_STATE) {
			cyttsp_pr_state(ts);
			printk("[TPD]%s: Cannot I2C Write\n", __func__);
			ret = -EFAULT;
			break;
		}

		ret = copy_from_user( &i2c_data, (int __user *) arg, sizeof(i2c_data));
		if(ret != 0) {
			printk(KERN_ERR "copy_from_user err %d \n",ret);
		}

//		printk(KERN_INFO "i2cdata offset 0x%x length 0x%x slave 0x%x\n", i2c_data.offset, i2c_data.length, i2c_data.cyttsp_slaveaddress);
		cyttsp_dbg(ts, CY_DBG_LVL_3, "i2cdata offset 0x%x length 0x%x slave 0x%x\n", i2c_data.offset, i2c_data.length, i2c_data.cyttsp_slaveaddress);

		if(i2c_data.length == 1) {
			if(i2c_data.offset == 0x00) 
				switch (GET_HSTMODE(i2c_data.i2c_data_buf[0])) {
				case GET_HSTMODE(CY_OPERATE_MODE):
					if(ts->power_state != CY_ACTIVE_STATE) {
						ts->power_state = CY_ACTIVE_STATE;
						enable_irq(ts->irq);
						mod_timer(&ts->timer, CY_TIMEOUT);
						cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: "
							"Switch to Operational Mode "
							"ps=%d\n", __func__,
							ts->power_state);
					}
					break;
				case GET_HSTMODE(CY_SYSINFO_MODE):
					if(ts->power_state != CY_SYSINFO_STATE) {
						ts->power_state = CY_SYSINFO_STATE;
						disable_irq_nosync(ts->irq);
						/* kill watchdog */
						del_timer(&ts->timer);
						cancel_work_sync(&ts->work);
						cyttsp_dbg(ts, CY_DBG_LVL_3, "%s: "
							"Switch to SysInfo Mode "
							"ps=%d\n", __func__,
							ts->power_state);
					}
					break;
				default:
					break;
			}
		}

		ret = cyttsp_write_reg_sequence( ts, i2c_data.offset, i2c_data.length, i2c_data.cyttsp_slaveaddress, i2c_data );
		if(ret != 0) {
			printk(KERN_ERR "cyttsp_write_reg_sequence err %d \n",ret);
			ret = -EIO;
			return ret;
		}

	default:
		break;
	}

	return ret;
}

static int cyttsp_ioctl_open( struct inode *inode, struct file *file )
	{
	struct cyttsp *ts = container_of( inode->i_cdev,struct cyttsp, cyttsp_cdev );

    printk( KERN_INFO "%s :[IN]\n", __func__ );
    file->private_data = ts;
    return 0;
	}

static int cyttsp_ioctl_close(struct inode *inode, struct file *file)
	{
	printk( KERN_INFO "%s :[IN]\n", __func__ );
    return 0;
	}

static struct file_operations cyttsp_fops = {
    .owner            = THIS_MODULE,
	.release          = cyttsp_ioctl_close,
    .open             = cyttsp_ioctl_open,
    .unlocked_ioctl   = cyttsp_ioctl,
};
/* FUJITSU:2011-12-27 add end */

void cyttsp_core_release(void *handle)
{
	struct cyttsp *ts = handle;

	if (ts) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&ts->early_suspend);
#endif
		cyttsp_ldr_free(ts);
		mutex_destroy(&ts->data_lock);
		mutex_destroy(&ts->startup_mutex);
		free_irq(ts->irq, ts);
		input_unregister_device(ts->input);
		kfree(ts->fwname);
		kfree(ts);
	}
}
EXPORT_SYMBOL_GPL(cyttsp_core_release);

static int cyttsp_open(struct input_dev *dev)
{
	int retval = 0;
#if 0  /* FUJITSU:2011-09-27 del start */
	struct cyttsp *ts = input_get_drvdata(dev);

	if (ts == NULL) {
		pr_err("%s: NULL context pointer\n", __func__);
		return -ENOMEM;
	}

	mutex_lock(&ts->startup_mutex);
	if (!ts->powered) {
		retval = cyttsp_power_on(ts);

		/* Powered if no hard failure */
		if (retval == 0)
			ts->powered = true;
		else
			ts->powered = false;
	}
	mutex_unlock(&ts->startup_mutex);
#endif /* FUJITSU:2011-09-27 del end */
	return retval;
}

static void cyttsp_close(struct input_dev *dev)
{
	/*
	 * close() normally powers down the device
	 * this call simply returns unless power
	 * to the device can be controlled by the driver
	 */
	return;
}

void *cyttsp_core_init(struct cyttsp_bus_ops *bus_ops,
	struct device *dev, int irq, char *name)
{
	int i;
	int signal;
	int retval = 0;
	/* FUJITSU:2011-08-11 add start */
	int devno;
	dev_t dev_ioctl;
	/* FUJITSU:2011-08-11 add end */
	struct input_dev *input_device;
	struct cyttsp *ts = kzalloc(sizeof(*ts), GFP_KERNEL);

	if (!ts) {
		pr_err("%s: Error, kzalloc\n", __func__);
		goto error_alloc_data;
	}

	ts->fwname = kzalloc(CY_BL_FW_NAME_SIZE, GFP_KERNEL);
	if (!ts->fwname || (dev == NULL) || (bus_ops == NULL)) {
		pr_err("%s: Error, dev, bus_ops, or fwname null\n",
			__func__);
		kfree(ts);
		goto error_alloc_data;
	}

	ts->waiting_for_fw = false;
	ts->powered = false;
#ifdef CY_USE_REG_ACCESS
	ts->rw_regid = 0;
#endif
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->ic_grpnum = 1;
	ts->ic_grpoffset = 0;
	memset(ts->ic_grpstart, 0, sizeof(ts->ic_grpstart));
	ts->ic_grpstart[CY_IC_GRPNUM_SI] = CY_REG_SI_START;
	ts->ic_grpstart[CY_IC_GRPNUM_OP] = CY_REG_OP_START;
#endif

	mutex_init(&ts->data_lock);
	mutex_init(&ts->startup_mutex);
	ts->dev = dev;
	ts->platform_data = dev->platform_data;
	ts->bus_ops = bus_ops;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->bus_ops->tsdebug = CY_DBG_LVL_0;
#endif
	init_completion(&ts->bl_int_running);
	init_completion(&ts->si_int_running);
	ts->flags = ts->platform_data->flags;

	/* FUJITSU:2012-05-14 add start */
	ts->timer_up_flag = false;
	ts->jdg_area_size = CY_MOVE_JUDGE_AREA;
	dev_ioctl = MKDEV(cdev_major, 0);
	udev_class = class_create(THIS_MODULE, "cyttsp");

	retval = alloc_chrdev_region(&dev_ioctl, 0, 1, "cyttsp");
	cdev_major = MAJOR(dev_ioctl);
	if (cdev_major == 0) {
		cdev_major = retval;
	}

	devno = MKDEV( cdev_major, 0 ); 
	cdev_init( &(ts->cyttsp_cdev), &cyttsp_fops );
	ts->cyttsp_cdev.owner = THIS_MODULE;
	ts->cyttsp_cdev.ops = &cyttsp_fops;
	retval = cdev_add ( &(ts->cyttsp_cdev), devno, 1 );

	device_create(udev_class, NULL, devno, NULL, "cyttsp");
	/* FUJITSU:2012-05-14 add end */

	ts->irq = irq;
	if (ts->irq <= 0) {
		pr_err("%s: Error, failed to allocate irq\n", __func__);
		goto error_init;
	}

	/* Create the input device and register it. */
	input_device = input_allocate_device();
	if (!input_device) {
		pr_err("%s: Error, failed to allocate input device\n",
			__func__);
		goto error_input_allocate_device;
	}

	ts->input = input_device;
	input_device->name = name;
	snprintf(ts->phys, sizeof(ts->phys), "%s", dev_name(dev));
	input_device->phys = ts->phys;
	input_device->dev.parent = ts->dev;
	ts->bus_type = bus_ops->dev->bus;
#ifdef CONFIG_TOUCHSCREEN_WATCHDOG
	INIT_WORK(&ts->work, cyttsp_timer_watchdog);
	setup_timer(&ts->timer, cyttsp_timer, (unsigned long)ts);
	ts->ntch_count = CY_NTCH;
	ts->prv_tch = CY_NTCH;
#endif
	input_device->open = cyttsp_open;
	input_device->close = cyttsp_close;
	input_set_drvdata(input_device, ts);
	dev_set_drvdata(dev, ts);

	_cyttsp_init_tch_map(ts);
	memset(ts->prv_trk, CY_NTCH, sizeof(ts->prv_trk));
	/* FUJITSU:2011-10-17 add start */
	setup_timer(&ts->timer_up, cyttsp_timer_up, (unsigned long)ts);
	memset(ts->mt_trc_id, -1, sizeof(ts->mt_trc_id));
	/* FUJITSU:2011-10-17 add end */

	__set_bit(EV_ABS, input_device->evbit);
	/* FUJITSU:2012-03-28 mod start */
	__set_bit(INPUT_PROP_DIRECT, input_device->propbit);
	input_mt_init_slots(input_device, CY_MAX_NTCH);
//	for (i = 0; i < (ts->platform_data->frmwrk->size/CY_NUM_ABS_SET); i++) {
	for (i = 0; i < CY_NUM_ABS_SET; i++) {
		signal = ts->platform_data->frmwrk->abs[
			(i*CY_NUM_ABS_SET)+CY_SIGNAL_OST];
		if (signal) {
			input_set_abs_params(input_device,
				signal,
				ts->platform_data->frmwrk->abs[
					(i*CY_NUM_ABS_SET)+CY_MIN_OST],
				ts->platform_data->frmwrk->abs[
					(i*CY_NUM_ABS_SET)+CY_MAX_OST],
				ts->platform_data->frmwrk->abs[
					(i*CY_NUM_ABS_SET)+CY_FUZZ_OST],
				ts->platform_data->frmwrk->abs[
					(i*CY_NUM_ABS_SET)+CY_FLAT_OST]);
		}
	}
	/* FUJITSU:2012-03-28 mod end */

/* FUJITSU:2011-09-27 del start */
#if 0 //(LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
	input_set_events_per_packet(input_device, 6 * CY_NUM_TCH_ID);
#endif
/* FUJITSU:2011-09-27 del end */

	if (ts->platform_data->frmwrk->enable_vkeys)
		input_set_capability(input_device, EV_KEY, KEY_PROG1);

	if (input_register_device(input_device)) {
		pr_err("%s: Error, failed to register input device\n",
			__func__);
		goto error_input_register_device;
	}

	/* FUJITSU:2011-08-31 add start */
	if (ts->platform_data->init)
		retval = ts->platform_data->init(1);
	/* FUJITSU:2011-08-31 add end */

	/* enable interrupts */
	cyttsp_dbg(ts, CY_DBG_LVL_3,
		"%s: Initialize IRQ\n", __func__);
	retval = request_threaded_irq(ts->irq, NULL, cyttsp_irq,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		ts->input->name, ts);
	if (retval < 0) {
		pr_err("%s: IRQ request failed r=%d\n",
			__func__, retval);
		ts->irq_enabled = false;
		goto error_input_register_device;
	} else
		ts->irq_enabled = true;

	/* FUJITSU:2011-08-31 add start */
	retval = cyttsp_power_on(ts);
	/* FUJITSU:2011-08-31 add end */

	/* Add /sys files */
	cyttsp_ldr_init(ts);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = cyttsp_early_suspend;
	ts->early_suspend.resume = cyttsp_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	goto no_error;

error_input_register_device:
	input_free_device(input_device);
error_input_allocate_device:

error_init:
	mutex_destroy(&ts->data_lock);
	mutex_destroy(&ts->startup_mutex);
	kfree(ts->fwname);
	kfree(ts);
error_alloc_data:
	pr_err("%s: Failed Initialization\n", __func__);
no_error:
	return ts;
}
EXPORT_SYMBOL_GPL(cyttsp_core_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver core");
MODULE_AUTHOR("Cypress");

