/*
 * Core Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) TouchPress drivers.
 * For use with Cypress Txx3xx parts.
 * Supported parts include:
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009-2012 Cypress Semiconductor, Inc.
 * Copyright (C) 2010-2012 Motorola Mobility, Inc.
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
// COPYRIGHT(C) FUJITSU LIMITED 2012
/*----------------------------------------------------------------------------*/
#include "cyttsp_press_core.h"
#include "cyttsp_press_gpio.h"
#include "cyttsp_press_if.h"
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/time.h>
#include <asm/uaccess.h>

#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input/touch_platform.h>
#include <linux/version.h>  /* Required for kernel version checking */
#include <linux/firmware.h> /* This enables firmware class loader code */
#include <linux/workqueue.h>
#include <linux/string.h> 
#include <linux/ctype.h> 

#define CY_USE_HW_RESET

/* Soft reset here is for platforms with no hw reset line */
#ifdef CY_USE_HW_RESET
#define cyttsp_soft_reset(ts)
#else
#define cyttsp_soft_reset(ts) {\
    int rc;\
    u8 cmd = CY_SOFT_RESET_MODE;\
    rc = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);\
    if (rc < 0) {\
        return rc;\
    }\
}
#endif

/* irq enable */
#define cyttsp_press_enable_irq(ts) {\
    if (!ts->irq_enabled) {\
        enable_irq(ts->irq);\
        ts->irq_enabled = true;\
    }\
}

/* irq disable */
#define cyttsp_press_disable_irq(ts) {\
    if (ts->irq_enabled) {\
        disable_irq(ts->irq);\
        ts->irq_enabled = false;\
    }\
}

/* Bootloader number of command keys */
#define CY_NUM_BL_KEYS              8

/* Bootloader File 0 offset */
#define CY_BL_FILE0                 0x00
/* Bootloader command directive */
#define CY_BL_CMD                   0xFF
/* Bootloader Exit and Verify Checksum command */
#define CY_BL_EXIT                  0xA5
/* Bootloader default command keys */
#define CY_BL_KEY0                  0xAA
#define CY_BL_KEY1                  0x55
#define CY_BL_KEY2                  0x33
#define CY_BL_KEY3                  0x68
#define CY_BL_KEY4                  0x98
#define CY_BL_KEY5                  0x0B
#define CY_BL_KEY6                  0x1D
#define CY_BL_KEY7                  0xAC

/* helpers */
#define IS_BAD_PKT(x)               ((x) & 0x20)
#define IS_VALID_APP(x)             ((x) & 0x01)
#define IS_OPERATIONAL_ERR(x)       ((x) & 0x3F)
#define IS_LOW_POWER(x)             ((x) & 0x04)
#define GET_HSTMODE(reg)            ((reg & 0x70) >> 4)
#define GET_BOOTLOADERMODE(reg)     ((reg & 0x10) >> 4)

/* REG ADDRESS */
#define CY_REG_BASE                 0x00
#define CY_REG_ADR_MIN              CY_REG_BASE
#define CY_REG_ADR_MAX              0x1F
#define CY_REG_ADR_BASE             0x05
#define CY_REG_ADR_RIDAC            0x0F
#define CY_REG_ADR_CALIB            0x11
#define CY_REG_ADR_RGAIN            0x13
#define CY_REG_ADR_ACT_INTRVL       0x1D

/* REG VALUE */
#define CY_REG_VAL_MIN              0x00
#define CY_REG_VAL_MAX              0xFF
#define CY_REG_VAL_TARGET_IDAC      0xC0
#define CY_REG_VAL_GAIN_X4          0x02
#define CY_REG_VAL_MANUAL_CALIB     0x01
#define CY_REG_VAL_MANUAL_CALIB_APPLY 0x03
#define CY_REG_VAL_MANUAL_CALIB_OK  0x80
#define CY_REG_VAL_AUTO_CALIB       0x09
#define CY_REG_VAL_AUTO_CALIB_APPLY 0x02
#define CY_REG_VAL_AUTO_CALIB_OK    0x88

/*  RAW MASK*/
#define CY_RAW_MSB_MASK             0xFF00
#define CY_RAW_LSB_MASK             0x00FF
#define CY_RAW_SHIT_BIT_NUM         8

#define CY_DELAY_DFLT               20 /* ms */
#define CY_DELAY_MAX                (500 / CY_DELAY_DFLT) /* half second */

/* hand shake bit */
#define CY_HNDSHK_BIT               0x80

/* device mode bits */
#define CY_OPERATE_MODE             0x00
#define CY_SYSINFO_MODE             0x10

/* power mode select bits */
#define CY_SOFT_RESET_MODE          0x01 /* return to Bootloader mode */
#define CY_DEEP_SLEEP_MODE          0x02
#define CY_LOW_POWER_MODE           0x04

/* abs settings */
#define CY_CNT_ABS_SET              2 /* number of abs settings */
#define CY_NUM_ABS_SET              5 /* number of abs values per setting */
/* abs value offset */
#define CY_SIGNAL_OST               0
#define CY_MIN_OST                  1
#define CY_MAX_OST                  2
#define CY_FUZZ_OST                 3
#define CY_FLAT_OST                 4

#define CY_BL_PAGE_SIZE             16
#define CY_BL_NUM_PAGES             5
#define CY_BL_READY_NO_APP          0x10
#define CY_BL_READY_APP             0x11
#define CY_BL_RUNNING               0x20
#define CY_BL_MAX_DATA_LEN          (CY_BL_PAGE_SIZE * 2)
#define CY_BL_ENTER_CMD_SIZE        11
#define CY_BL_EXIT_CMD_SIZE         11
#define CY_BL_WR_BLK_CMD_SIZE       79
#define CY_BL_FW_IMG_SIZE           17718
#define CY_BL_FW_NAME_SIZE          NAME_MAX
#define CY_MAX_PRBUF_SIZE           PIPE_BUF

/* Delay Interval Unit:ms */
#define CY_I2C_NACK_WAIT_VALUE              1

/* Read Retry Count */
#define CY_INVALID_BUFFER_READ_CNT          10
#define CY_PRESS_VALUE_READ_CNT             20
#define CY_CALI_WAIT_MAX_CNT                3

/* Buffer Size */
#define CY_DEVICEINFO_SIZE                  20
#define CY_CALIBRATION_SIZE                 10

/* Delimiter */
#define CY_DELIMITER_COMMA                  ","

/* Character */
#define CY_MARK_SPACE                       ' '
#define CY_MARK_EQUAL                       '='
#define CY_MARK_CR                          0x0D
#define CY_MARK_LF                          0x0A

/* Interrupt Interval Unit:ms */
#define CY_INTRVL_INT_08MS                  8
#define CY_INTRVL_INT_10MS                  10
#define CY_INTRVL_INT_20MS                  20

/* Sleep Interval Unit:ms */
#define CY_INTRVL_READ_PRESS                10
#define CY_INTRVL_TO_MANU_CALIB             50
#define CY_INTRVL_TO_AUTO_CALIB             30

/* Firmware Version */
#define CY_FW_VERSION_V00R09                0x0009
#define CY_FW_VERSION_V00R18                0x0018

/* On / Off Flag */
enum cyttsp_onoff_flag {
    CY_OFF = 0,         /* off */
    CY_ON = 1           /* on */
};

/* Base Range */
enum cyttsp_base_range {
    CY_BASE_VAL_MIN = 13000,    /* Min Base Value */
    CY_BASE_VAL_MAX = 15000     /* Max Base Value */
};

/* Setting Flag */
enum cyttsp_sett_flags {
    CY_USE_HNDSHK = 0x01,
    CY_USE_SLEEP = 0x02,
    CY_FORCE_LOAD = 0x04,
};

/* Power State */
enum cyttsp_powerstate {
    CY_IDLE_STATE,      /* IC cannot be reached */
    CY_READY_STATE,     /* pre-operational; ready to go to ACTIVE */
    CY_ACTIVE_STATE,    /* app is running, IC is scanning */
    CY_LOW_PWR_STATE,   /* not currently used  */
    CY_SLEEP_STATE,     /* app is running, IC is idle */
    CY_BL_STATE,        /* bootloader is running */
    CY_LDR_STATE,       /* loader is running */
    CY_SYSINFO_STATE,   /* switching to sysinfo mode */
    CY_INVALID_STATE    /* always last in the list */
};

/* Power State String */
static char *cyttsp_powerstate_string[] = {
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

enum cyttsp_sleep_intrvl {
    READ_PRESS_INTRVL = 0,  /* Read Press Value INTRVL */
    TO_MANUAL_CALIB = 1,    /* Switch to Manual Calibration */
    TO_AUTO_CALIB = 2       /* Switch to Automatic Calibration */
};

/* Regs Name */
enum cyttsp_regs_name {
    CY_BOOT_REGS = 0x01,    /* Bootloader Regs */
    CY_SYS_REGS = 0x02,     /* System Information Regs */
    CY_OP_REGS = 0x03       /* Operational Regs */
};

/* Calibration Wait Count */
enum cyttsp_calib_wait_count {
    CY_CALIB_PRE_WAIT_CNT = 100,        /* Wait 1000ms */
    CY_CALIB_START_WAIT_CNT = 12,       /* Wait 120ms */
    CY_CALIB_END_WAIT_CNT = 10,         /* Wait 100ms */
    CY_CALIB_APPLY_START_WAIT_CNT = 3,  /* Wait 30ms */
    CY_CALIB_BASE_CHECK_WAIT_CNT = 1000 /* Wait 10000ms */
};

#ifdef CONFIG_TOUCHPRESS_DEBUG
static char *cyttsp_echo_keys[] = {
    "VIEW",
    "IRQ",
    "PRESS_CNT",
    "INVALID_CNT",
    "CALIB_CNT",
    "PRESS_INTRVL",
    "MANUAL_CALIB",
    "AUTO_CALIB",
    "REG_ADDR",
    "FW_MODE"
};

enum cyttsp_drv_attr {
    CY_DRV_BASE_INFO = 1,       /* Driver Base Info */
    CY_OPTION_INFO = 2,         /* Driver Option Info */
    CY_BOOT_REG_INFO = 3,       /* Bootloader Regs */
    CY_SYS_REG_INFO = 4,        /* System Information Regs */
    CY_OP_REG_INFO = 5          /* Operational Mode Regs */
};

#ifdef CY_TOUCH_PRESS_DEBUG_TOOLS
#define CY_SUPPORT_ECHO_KEY     10
#else
#define CY_SUPPORT_ECHO_KEY     2
#endif
#endif

/* press structure */
struct cyttsp_press {
    u8 msb;
    u8 lsb;
} __attribute__((packed));

/* TrueTouch Standard Product Gen3 interface definition */
struct cyttsp_press_data {
    u8 hst_mode;
    u8 tt_mode;
    u8 tt_stat;
    struct cyttsp_press fraw;
    struct cyttsp_press base;
    u8 tt_reserved1[2];
    struct cyttsp_press diff;
    struct cyttsp_press raw;
    u8 tt_undef1[2];
    u8 idac;
    u8 baseline_reset;
    u8 calibration;
    u8 prescaler;
    u8 idac_gain;
    u8 tt_undef2[2];
    u8 act_int_intrvl;
    u8 lp_int_intrvl;
    u8 tt_undef3;
    u8 imo_tr;
    u8 fw_ver;
    u8 fw_rev;
    u8 alive;
    u8 lcd_vendor_id;
    u8 debug_uart;
    u8 tt_reserved2;
} __attribute__((packed));

/* TTSP System Information interface definition */
struct cyttsp_sysinfo_data {
    u8 hst_mode;
    u8 mfg_stat;
    u8 mfg_cmd;
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
    u8 tt_undef[6];
    u8 act_intrvl;
    u8 tch_tmout;
    u8 lp_intrvl;
};

/* TTSP Bootloader Register Map interface definition */
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
    u8 cid[3];
    u8 ap_data_sta;
    u8 ap_data_end;
};

/* Calibration State */
enum cyttsp_calib_state {
    CY_CALIB_PRE = 0,       /* Calibration Preparation */
    CY_CALIB_START,         /* Calibration Start */
    CY_CALIB_END,           /* Calibration End */
    CY_CALIB_APPLY_START,   /* Calibration Apply Start */
    CY_CALIB_APPLY_END,     /* Calibration Apply End */
    CY_CALIB_BASE_CHECK     /* Calibration Base Check */
};

/* Calibration Info */
struct cyttsp_calib {
    enum cyttsp_powerstate calib_state;
    int wait_count;
    int retry_count;
    int total_count;

    int calib_pre_wait_cnt;         /* Calibration Pre Wait Count */
    int calib_start_wait_cnt;       /* Calibration Start Wait Count */
    int calib_end_wait_cnt;         /* Calibration End Wait Count */
    int calib_apply_start_wait_cnt; /* Calibration Apply Start Wait Count */
    int calib_base_check_wait_cnt;  /* Calibration Base Check Wait Count */
};

struct cyttsp {
    struct device *dev;
    int irq;
    struct input_dev *input;
    struct mutex data_lock;     /* Used to prevent concurrent access */
    char phys[32];
    const struct bus_type *bus_type;
    const struct touch_platform_data *platform_data;
    struct cyttsp_bus_ops *bus_ops;
    struct cyttsp_press_data press_data;
    struct cyttsp_bootloader_data bl_data;
    struct cyttsp_sysinfo_data sysinfo_data;
    struct completion bl_int_running;
    struct completion si_int_running;
    enum cyttsp_powerstate power_state;
    bool irq_enabled;
    bool waiting_for_fw;
    char *fwname;
    u16 flags;
    int press_value_count;
    int invalid_buf_read_count;
    int sleep_intrvl[TO_AUTO_CALIB + 1];
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
#ifdef CONFIG_TOUCHPRESS_DEBUG
    enum cyttsp_drv_attr drv_attr_mode;
    u8  rw_regid;
#endif
    struct cdev cyttsp_cdev;
    int dev_id;
    unsigned long first_down_time;
    enum touchstatus tch_sts;
    bool tch_enabled;
    bool poweron;
    struct cyttsp_calib calib;
};
static short prev_filteredraw;
static int pretch_sts;

static int cdev_major = 0;
static int devno = 0;
static struct class* udev_class;
char reg_buf[CY_MAX_PRBUF_SIZE + 1];
/* GPIO/I2C flag */
extern bool is_use_i2c;

// command sets
static const u8 bl_command[] = {
    CY_BL_FILE0, CY_BL_CMD, CY_BL_EXIT,
    CY_BL_KEY0, CY_BL_KEY1, CY_BL_KEY2,
    CY_BL_KEY3, CY_BL_KEY4, CY_BL_KEY5,
    CY_BL_KEY6, CY_BL_KEY7
};

/*============================================================================
                        INTERNAL API DECLARATIONS
============================================================================*/
static int cyttsp_press_calibration(struct cyttsp *ts, char *caldata);
static int cyttsp_press_startup(struct cyttsp *ts);
void cyttsp_press_touch_status(void *handle, enum touchstatus tch_sts);
static void cyttsp_press_cmd_reset(int on);
static int cyttsp_press_load_operational_regs(struct cyttsp *ts, int count);
#ifdef CY_TOUCH_PRESS_DEBUG_TOOLS
static int cyttsp_press_mc_putbl(struct cyttsp *ts);
static int cyttsp_press_mc_wr_blk_chunks(struct cyttsp *ts, u8 command,
    u8 length, const u8 *values);
#endif

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_PS_STRING
===========================================================================*/
static char* cyttsp_press_ps_string(struct cyttsp *ts)
{
    return ts->power_state < CY_INVALID_STATE ?
            cyttsp_powerstate_string[ts->power_state] :
            "INVALID"; 
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_PR_STATE
===========================================================================*/
static void cyttsp_press_pr_state(struct cyttsp *ts)
{
    pr_info("[PPD-core]%s: %s\n", __func__, cyttsp_press_ps_string(ts));
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_TOUCH_LIFTOFF
===========================================================================*/
static void cyttsp_press_touch_liftoff(struct cyttsp *ts)
{
    ts->tch_sts = LIFT_OFF;
    ts->calib.calib_state = CY_CALIB_PRE;
    ts->calib.wait_count = 1;
    ts->calib.total_count = 1;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3,
        "[PPD-core]%s: calib_pre_wait_cnt=%d, calib_start_wait_cnt=%d, "
        "calib_end_wait_cnt=%d, calib_apply_start_wait_cnt=%d, calib_base_check_wait_cnt=%d\n",
        __func__,
        ts->calib.calib_pre_wait_cnt,
        ts->calib.calib_start_wait_cnt,
        ts->calib.calib_end_wait_cnt,
        ts->calib.calib_apply_start_wait_cnt,
        ts->calib.calib_base_check_wait_cnt);
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_GET_CALIB_WAIT_CNT
===========================================================================*/
static int cyttsp_press_get_calib_wait_cnt(int wait_cnt, int rate)
{
    int ret_cnt = wait_cnt / rate;

    if (wait_cnt % rate)
        return ret_cnt + 1;
    else
        return ret_cnt;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_READ_BLOCK_DATA
===========================================================================*/
static int cyttsp_press_read_block_data(struct cyttsp *ts, u8 command,
    u8 length, void *buf)
{
    int retval;
    int tries;
    u32 i2c_param = 0x00;

    if (!buf || !length) {
        return -EIO;
    }

    for (tries = 0, retval = -1;
        (tries < CY_NUM_RETRY) && (retval < 0);
        tries++) {
        retval = ts->bus_ops->read(ts->bus_ops, command, length, buf);
        if (retval) {
            i2c_param = ~retval + 0x01;
            if (tries == 0 && (i2c_param & (1U << 3))) {
                cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: I2C Read NACK Err Retry \n", __func__);
                mdelay(CY_I2C_NACK_WAIT_VALUE);
            } else {
                cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: I2C Read Err Retry \n", __func__);
                msleep(CY_DELAY_DFLT);
            }
        }
    }

    if (retval < 0) {
        pr_err("[PPD-core]%s: bus read block data fail (ret=%d)\n",
            __func__, retval);
    }

    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_WRITE_BLOCK_DATA
===========================================================================*/
static int cyttsp_press_write_block_data(struct cyttsp *ts, u8 command,
    u8 length, const void *buf)
{
    int retval;
    int tries;

    if (!buf || !length) {
        return -EIO;
    }

    for (tries = 0, retval = -1;
        (tries < CY_NUM_RETRY) && (retval < 0);
        tries++) {
        retval = ts->bus_ops->write(ts->bus_ops, command, length, buf);
        if (retval) {
            cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: I2C write Err Retry \n", __func__);
            msleep(CY_DELAY_DFLT);
        }
    }

    if (retval < 0) {
        pr_err("[PPD-core]%s: bus write block data fail (ret=%d)\n",
            __func__, retval);
    }

    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_READ_OPERATIONAL_REG_SEQUENCE
===========================================================================*/
static int cyttsp_read_operational_reg_sequence(struct cyttsp *ts)
{
    int retval = 0;
    int tries = 1;

    for (tries = 1; tries <= ts->invalid_buf_read_count; tries++) {
        // Read Operational Reg
        memset(&(ts->press_data), 0, sizeof(struct cyttsp_press_data));
        retval = cyttsp_press_read_block_data(ts,
                    CY_REG_BASE, sizeof(struct cyttsp_press_data), &(ts->press_data));
        if (retval < 0) {
            pr_err("[PPD-core]%s: read fail on Operational Reg\n", __func__);
            return -EIO;
        }

        // Invalid buffer detected
        if (IS_BAD_PKT(ts->press_data.tt_mode)) {
            cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: Invalid buffer detected (tries = %d)\n", __func__, tries);
            mdelay(CY_I2C_NACK_WAIT_VALUE);
            continue;
        } else {
            // Read OK
            break;
        }
    }

    if (tries > ts->invalid_buf_read_count) {
        pr_err("[PPD-core]%s: Invalid Buffer Count Over\n", __func__);
        return -EBADE;
    }

    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_REGINFO_STRING
===========================================================================*/
static char* cyttsp_press_reginfo_string(struct cyttsp *ts, enum cyttsp_regs_name regs)
{
    switch (regs) {
    /* Bootloader Regs */
    case CY_BOOT_REGS:
        snprintf(reg_buf, CY_MAX_PRBUF_SIZE,
            "[PPD-core] Bootloader Register Map:\n"
            "  Bootloader Command File:     0x%02X\n"
            "  Bootloader Status:           0x%02X\n"
            "  Bootloader Error:            0x%02X\n"
            "  Bootloader Version:          0x%02X%02X\n"
            "  Build Bootloader Version:    0x%02X%02X\n"
            "  TTSP Version:                0x%02X%02X\n"
            "  Application ID:              0x%02X%02X\n"
            "  Application Version:         0x%02X%02X\n"
            "  Custom ID:                   0x%02X%02X%02X\n"
            "  Application Data Area:       0x%02X%02X\n",
            ts->bl_data.bl_file,
            ts->bl_data.bl_status,
            ts->bl_data.bl_error,
            ts->bl_data.blver_hi, ts->bl_data.blver_lo,
            ts->bl_data.bld_blver_hi, ts->bl_data.bld_blver_lo,
            ts->bl_data.ttspver_hi, ts->bl_data.ttspver_lo,
            ts->bl_data.appid_hi, ts->bl_data.appid_lo,
            ts->bl_data.appver_hi, ts->bl_data.appver_lo,
            ts->bl_data.cid[0], ts->bl_data.cid[1], ts->bl_data.cid[2],
            ts->bl_data.ap_data_sta, ts->bl_data.ap_data_end);
        break;

    /* System Information Regs */
    case CY_SYS_REGS:
        snprintf(reg_buf, CY_MAX_PRBUF_SIZE,
            "[PPD-core] System Information Mode Register Map:\n"
            "  Host Mode:                   0x%02X\n"
            "  Manufacturing Test Status:   0x%02X\n"
            "  Manufacturing Test Command:  0x%02X\n"
            "  Custom ID:                   0x%02X%02X%02X\n"
            "  Undefined:                   0x%02X\n"
            "  Unique ID:                   0x%02X%02X%02X%02X%02X%02X%02X%02X\n"
            "  Bootloader Version:          0x%02X%02X\n"
            "  TTSP Version:                0x%02X%02X\n"
            "  Application ID:              0x%02X%02X\n"
            "  Application Version:         0x%02X%02X\n"
            "  Undefined:                   0x%02X%02X%02X%02X%02X%02X\n"
            "  Active Refresh Interval:     0x%02X\n"
            "  Active Touch Timeout:        0x%02X\n"
            "  Low Power Refresh Interval:  0x%02X\n",
            ts->sysinfo_data.hst_mode,
            ts->sysinfo_data.mfg_stat,
            ts->sysinfo_data.mfg_cmd,
            ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1], ts->sysinfo_data.cid[2],
            ts->sysinfo_data.tt_undef1,
            ts->sysinfo_data.uid[0], ts->sysinfo_data.uid[1], ts->sysinfo_data.uid[2],
            ts->sysinfo_data.uid[3], ts->sysinfo_data.uid[4], ts->sysinfo_data.uid[5],
            ts->sysinfo_data.uid[6], ts->sysinfo_data.uid[7],
            ts->sysinfo_data.bl_verh, ts->sysinfo_data.bl_verl,
            ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
            ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
            ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
            ts->sysinfo_data.tt_undef[0], ts->sysinfo_data.tt_undef[1],
            ts->sysinfo_data.tt_undef[2], ts->sysinfo_data.tt_undef[3],
            ts->sysinfo_data.tt_undef[4], ts->sysinfo_data.tt_undef[5],
            ts->sysinfo_data.act_intrvl,
            ts->sysinfo_data.tch_tmout,
            ts->sysinfo_data.lp_intrvl);
        break;

    /* Operational Regs */
    case CY_OP_REGS:
        snprintf(reg_buf, CY_MAX_PRBUF_SIZE,
            "[PPD-core] Operating Mode Register Map:\n"
            "  Host Mode:                   0x%02X\n"
            "  TrueTouch Mode:              0x%02X\n"
            "  TrueTouch Status:            0x%02X\n"
            "  Filtered Raw:                0x%02X%02X\n"
            "  Base:                        0x%02X%02X\n"
            "  Reserved:                    0x%02X%02X\n"
            "  Diff:                        0x%02X%02X\n"
            "  Raw:                         0x%02X%02X\n"
            "  Undefined:                   0x%02X%02X\n"
            "  IDAC:                        0x%02X\n"
            "  Baseline Reset:              0x%02X\n"
            "  Calibration:                 0x%02X\n"
            "  Prescaler:                   0x%02X\n"
            "  IDAC_GAIN:                   0x%02X\n"
            "  Undefined:                   0x%02X%02X\n"
            "  Active INT Interval:         0x%02X\n"
            "  Low Power INT Interval:      0x%02X\n"
            "  Undefined:                   0x%02X\n"
            "  IMO_TR:                      0x%02X\n"
            "  FW Application Version:      0x%02X%02X\n"
            "  Alive Counter:               0x%02X\n"
            "  LCD VENDOR ID:               0x%02X\n"
            "  DebugUART:                   0x%02X\n"
            "  Reserved:                    0x%02X\n",
            ts->press_data.hst_mode,
            ts->press_data.tt_mode,
            ts->press_data.tt_stat,
            ts->press_data.fraw.msb, ts->press_data.fraw.lsb,
            ts->press_data.base.msb, ts->press_data.base.lsb,
            ts->press_data.tt_reserved1[0], ts->press_data.tt_reserved1[1],
            ts->press_data.diff.msb, ts->press_data.diff.lsb,
            ts->press_data.raw.msb, ts->press_data.raw.lsb,
            ts->press_data.tt_undef1[0], ts->press_data.tt_undef1[1],
            ts->press_data.idac,
            ts->press_data.baseline_reset,
            ts->press_data.calibration,
            ts->press_data.prescaler,
            ts->press_data.idac_gain,
            ts->press_data.tt_undef2[0], ts->press_data.tt_undef2[1],
            ts->press_data.act_int_intrvl,
            ts->press_data.lp_int_intrvl,
            ts->press_data.tt_undef3,
            ts->press_data.imo_tr,
            ts->press_data.fw_ver, ts->press_data.fw_rev,
            ts->press_data.alive,
            ts->press_data.lcd_vendor_id,
            ts->press_data.debug_uart,
            ts->press_data.tt_reserved2);
        break;

    default:
        snprintf(reg_buf, CY_MAX_PRBUF_SIZE, "Appointed Regs not exist");
        break;
    }

    return reg_buf;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_HNDSHK
===========================================================================*/
static int cyttsp_press_hndshk(struct cyttsp *ts, u8 hst_mode)
{
    int retval;
    u8 cmd;

    cmd = hst_mode & CY_HNDSHK_BIT ?
        hst_mode & ~CY_HNDSHK_BIT :
        hst_mode | CY_HNDSHK_BIT;

    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE,
        sizeof(cmd), (u8 *)&cmd);

    if (retval < 0) {
        pr_err("[PPD-core]%s: bus write fail on handshake (ret=%d)\n",
            __func__, retval);
    }

    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_LOAD_BL_REGS
===========================================================================*/
static int cyttsp_press_load_bl_regs(struct cyttsp *ts)
{
    int retval = 0;

    memset(&(ts->bl_data), 0, sizeof(struct cyttsp_bootloader_data));

    retval = cyttsp_press_read_block_data(ts, CY_REG_BASE,
        sizeof(ts->bl_data), &(ts->bl_data));

    if (retval < 0) {
        pr_err("[PPD-core]%s: bus fail reading Bootloader regs (ret=%d)\n",
            __func__, retval);
        /*
         * Calling process determines state change requirement
         */
        goto cyttsp_load_bl_regs_exit;
    }

    if (GET_BOOTLOADERMODE(ts->bl_data.bl_status)) {
        cyttsp_press_dbg(ts, CY_DBG_LVL_2,
            "%s: %s", __func__, cyttsp_press_reginfo_string(ts, CY_BOOT_REGS));
    } else {
        cyttsp_press_dbg(ts, CY_DBG_LVL_2,
            "[PPD-core]%s: Not Bootloader mode:\n"
            "  file=%02X status=%02X error=%02X\n",
            __func__,
            ts->bl_data.bl_file, ts->bl_data.bl_status,
            ts->bl_data.bl_error);
    }

cyttsp_load_bl_regs_exit:
    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_BL_APP_VALID
===========================================================================*/
static int cyttsp_press_bl_app_valid(struct cyttsp *ts)
{
    int retval = 0;

    retval = cyttsp_press_load_bl_regs(ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: bus fail on bl regs read\n", __func__);
        ts->power_state = CY_IDLE_STATE;
        cyttsp_press_pr_state(ts);
        retval = -ENODEV;
        goto cyttsp_bl_app_valid_exit;
    }

    if (GET_BOOTLOADERMODE(ts->bl_data.bl_status)) {
        ts->power_state = CY_BL_STATE;
        if (IS_VALID_APP(ts->bl_data.bl_status)) {
            pr_info("[PPD-core]%s: App found; normal boot\n", __func__);
            retval = 0;
            goto cyttsp_bl_app_valid_exit;
        } else {
            /*
             * The bootloader is running, but the app firmware
             * is invalid.  Keep the state as Bootloader and
             * let the loader try to update the firmware
             */
            pr_info("[PPD-core]%s: NO APP; load firmware!!\n", __func__);
            retval = 0;
            goto cyttsp_bl_app_valid_exit;
        }
    } else if (GET_HSTMODE(ts->bl_data.bl_file) == CY_OPERATE_MODE) {
        ts->power_state = CY_ACTIVE_STATE;
        /* No bootloader found in the firmware */
        if (!(IS_OPERATIONAL_ERR(ts->bl_data.bl_status))) {
            /* go directly to Operational Active status */
            pr_info("[PPD-core]%s: Operational\n", __func__);
            retval = 0;
            goto cyttsp_bl_app_valid_exit;
        } else {
            /*
             * Operational error
             * Continue operation in Active status
             */
            pr_err("[PPD-core]%s:  Operational failure\n", __func__);
            retval = -ENODEV;
            goto cyttsp_bl_app_valid_exit;
        }
    } else {
        /*
         * General failure of the device
         * Cannot operate in any status
         */
        pr_err("[PPD-core]%s: Unknown system failure\n", __func__);
        ts->power_state = CY_INVALID_STATE;
        cyttsp_press_pr_state(ts);
        retval = -ENODEV;
        goto cyttsp_bl_app_valid_exit;
    }

cyttsp_bl_app_valid_exit:
    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_EXIT_BL_MODE
===========================================================================*/
static int cyttsp_press_exit_bl_mode(struct cyttsp *ts)
{
    int retval = 0;
    int tries = 0;
    u8 bl_cmd[sizeof(bl_command)];

    memcpy(bl_cmd, bl_command, sizeof(bl_command));

    cyttsp_press_dbg(ts, CY_DBG_LVL_3,
        "[PPD-core]%s: bl_cmd= "
        "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
        __func__, bl_cmd[0], bl_cmd[1], bl_cmd[2],
        bl_cmd[3], bl_cmd[4], bl_cmd[5], bl_cmd[6],
        bl_cmd[7], bl_cmd[8], bl_cmd[9], bl_cmd[10]);

    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE,
        sizeof(bl_cmd), (void *)bl_cmd);
    if (retval < 0) {
        pr_err("[PPD-core]%s: bus write fail exit Bootloader mode (ret=%d)\n",
            __func__, retval);
        ts->power_state = CY_IDLE_STATE;
        cyttsp_press_pr_state(ts);
        return retval;
    }

    /* wait for TTSP Device to complete switch to Operational mode */
    tries = 0;
    do {
        msleep(60);
        retval = cyttsp_press_load_bl_regs(ts);
    } while (!((retval == 0) &&
        !GET_BOOTLOADERMODE(ts->bl_data.bl_status)) &&
        (tries++ < CY_DELAY_MAX));

    cyttsp_press_dbg(ts, CY_DBG_LVL_3,
        "[PPD-core]%s: check bl ready tries=%d ret=%d stat=%02X\n",
        __func__, tries, retval, ts->bl_data.bl_status);

    if (tries >= CY_DELAY_MAX) {
        pr_err("[PPD-core]%s: operational ready fail on tries>=%d ret=%d\n",
            __func__, CY_DELAY_MAX, retval);
        if (retval < 0) {
            pr_err("%s: bus fail exiting bootload\n", __func__);
            ts->power_state = CY_IDLE_STATE;
            cyttsp_press_pr_state(ts);
        } else {
            pr_err("[PPD-core]%s: Missing App: cannot exit bootloader\n",
                __func__);
            ts->power_state = CY_BL_STATE;
            cyttsp_press_pr_state(ts);

            /* this is a soft failure */
            retval = 0;
        }
    } else {
        ts->power_state = CY_READY_STATE;
        cyttsp_press_pr_state(ts);
        retval = 0;
    }

    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_SET_OPERATIONAL_MODE
===========================================================================*/
static int cyttsp_press_set_operational_mode(struct cyttsp *ts)
{
    int retval;
    u8 cmd = CY_OPERATE_MODE + CY_LOW_POWER_MODE;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: IN\n", __func__);

    cmd = cmd | (ts->sysinfo_data.hst_mode & CY_HNDSHK_BIT);

    /* wait for interrupt to set ready completion */
    ts->power_state = CY_SYSINFO_STATE;
    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
    if (retval < 0) {
        pr_err("[PPD-core]%s: I2C write fail set Operational mode (ret=%d)\n",
            __func__, retval);
        ts->power_state = CY_IDLE_STATE;
        cyttsp_press_pr_state(ts);
        goto cyttsp_press_set_operational_mode_exit;
    }

    ts->power_state = CY_ACTIVE_STATE;
    cyttsp_press_pr_state(ts);

cyttsp_press_set_operational_mode_exit :
    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: OUT\n", __func__);
    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_SET_SYSINFO_MODE
===========================================================================*/
static int cyttsp_press_set_sysinfo_mode(struct cyttsp *ts)
{
    int tries;
    int retval;
    u8 cmd = CY_SYSINFO_MODE;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: IN\n", __func__);

    memset(&(ts->sysinfo_data), 0, sizeof(struct cyttsp_sysinfo_data));

    /* switch to sysinfo mode */
    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
    if (retval < 0) {
        pr_err("[PPD-core]%s: write fail set SysInfo mode (ret=%d)\n",
            __func__, retval);
        ts->power_state = CY_IDLE_STATE;
        cyttsp_press_pr_state(ts);
        return retval;
    }

    /* wait for interrupt to set ready completion */
    ts->power_state = CY_SYSINFO_STATE;
    cyttsp_press_pr_state(ts);

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
            pr_err("[PPD-core]%s: Error setting up SysInfo mode timeout"
                " (ret=%d)\n", __func__, retval);
            /* just wait a max time */
            msleep(CY_DELAY_DFLT * CY_DELAY_MAX);
            retval = 0;
            break;
        } else {
            msleep(100);
            /* read sysinfo registers */
            retval = cyttsp_press_read_block_data(ts, CY_REG_BASE,
                sizeof(ts->sysinfo_data), &(ts->sysinfo_data));
            mb();
            if (retval < 0) {
                pr_err("[PPD-core]%s: SysInfo access err (ret=%d)\n",
                    __func__, retval);
            }
            if (ts->sysinfo_data.app_verh ||
                ts->sysinfo_data.app_verl) {
                break;
            }
        }
    } while (tries++ < CY_DELAY_MAX);

    cyttsp_press_dbg(ts, CY_DBG_LVL_3,
        "[PPD-core]%s: check sysinfo ready HST_MODE=%02X tries=%d ret=%d\n",
        __func__, ts->sysinfo_data.hst_mode, tries, retval);

    cyttsp_press_dbg(ts, CY_DBG_LVL_0,
        "%s: %s", __func__, cyttsp_press_reginfo_string(ts, CY_SYS_REGS));

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: OUT\n", __func__);

    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_SET_SYSINFO_REGS
===========================================================================*/
static int cyttsp_press_set_sysinfo_regs(struct cyttsp *ts)
{
    int retval = 0;
    u8 act_cmd = CY_INTRVL_INT_20MS;
    u8 toggle_cmd = CY_REG_VAL_MIN;
    unsigned short fw_ver = ts->sysinfo_data.app_verh << CY_RAW_SHIT_BIT_NUM | ts->sysinfo_data.app_verl;

    // Version:V00R18
    if (fw_ver >= CY_FW_VERSION_V00R18) {
        ts->calib.calib_pre_wait_cnt = cyttsp_press_get_calib_wait_cnt(CY_CALIB_PRE_WAIT_CNT * 4, 2);
        ts->calib.calib_end_wait_cnt = cyttsp_press_get_calib_wait_cnt(CY_CALIB_END_WAIT_CNT, 4);
        ts->calib.calib_base_check_wait_cnt = cyttsp_press_get_calib_wait_cnt(CY_CALIB_BASE_CHECK_WAIT_CNT, 4);

        act_cmd = CY_INTRVL_INT_08MS;
        retval = cyttsp_press_write_block_data(ts, CY_REG_ADR_ACT_INTRVL, sizeof(act_cmd), &act_cmd);
        if (retval < 0) {
            pr_err("[PPD-core]%s: write SysInfo ACT_INTRVL error (ret=%d)\n", __func__, retval);
            return retval;
        }
        msleep(1);
        ts->sysinfo_data.act_intrvl = CY_INTRVL_INT_08MS;

        ts->calib.calib_pre_wait_cnt = cyttsp_press_get_calib_wait_cnt(CY_CALIB_PRE_WAIT_CNT * CY_INTRVL_INT_10MS * 4, 2 * CY_INTRVL_INT_08MS);
    } else {
        // Version:V00R09
        act_cmd = CY_INTRVL_INT_20MS;
        if (fw_ver >= CY_FW_VERSION_V00R09) {
            retval = cyttsp_press_write_block_data(ts, CY_REG_ADR_ACT_INTRVL, sizeof(act_cmd), &act_cmd);
            if (retval < 0) {
                pr_err("[PPD-core]%s: write SysInfo ACT_INTRVL error (ret=%d)\n", __func__, retval);
                return retval;
            }

            msleep(1);
            ts->sysinfo_data.act_intrvl = CY_INTRVL_INT_20MS;
            ts->calib.calib_pre_wait_cnt = cyttsp_press_get_calib_wait_cnt(CY_CALIB_PRE_WAIT_CNT, 2);
            ts->calib.calib_end_wait_cnt = cyttsp_press_get_calib_wait_cnt(CY_CALIB_END_WAIT_CNT, 2);
            ts->calib.calib_base_check_wait_cnt = cyttsp_press_get_calib_wait_cnt(CY_CALIB_BASE_CHECK_WAIT_CNT, 2);
        }
    }

    // Run Toggle Command
    toggle_cmd = ts->sysinfo_data.hst_mode & CY_HNDSHK_BIT ?
        ts->sysinfo_data.hst_mode & ~CY_HNDSHK_BIT :
        ts->sysinfo_data.hst_mode | CY_HNDSHK_BIT;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3,
        "[PPD-core]%s: SysInfo Toggle Command = 0x%02X hst_mode=0x%02X\n",
        __func__, toggle_cmd, ts->sysinfo_data.hst_mode);
    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(toggle_cmd), (u8 *)&toggle_cmd);
    if (retval < 0) {
        pr_err("[PPD-core]%s: SysInfo Toggle Command error (ret=%d)\n", __func__, retval);
        return retval;
    }
    msleep(CY_DELAY_DFLT);

    return 0;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_HW_RESET
===========================================================================*/
static int cyttsp_press_hw_reset(struct cyttsp *ts)
{
    int retval = 0;

    ts->power_state = CY_BL_STATE;
    cyttsp_press_pr_state(ts);

    cyttsp_soft_reset(ts);  /* Does nothing if CY_USE_HW_RESET defined */

    if (ts->platform_data->hw_reset) {
        retval = ts->platform_data->hw_reset();
        if (retval < 0) {
            pr_err("[PPD-core]%s: fail on hard reset (ret=%d)\n",
                __func__, retval);
            goto cyttsp_press_hw_reset_exit;
        }
    } else {
        pr_err("[PPD-core]%s: no hardware reset function (ret=%d)\n",
            __func__, retval);
        retval = -ENOSYS;
        goto cyttsp_press_hw_reset_exit;
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
        pr_err("[PPD-core]%s: Hard reset timer setup fail (ret=%d)\n",
            __func__, retval);
        /* just sleep a default time */

        msleep(CY_DELAY_DFLT * CY_DELAY_MAX);
        retval = 0;
    }

    cyttsp_press_dbg(ts, CY_DBG_LVL_2,
        "[PPD-core]%s:  Hard Reset: ret=%d", __func__, retval);

    if (retval > 0) {
        retval = 0;
    }

cyttsp_press_hw_reset_exit:
    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_DO_CALIBRATION
===========================================================================*/
static int cyttsp_press_do_calibration(struct cyttsp *ts, struct cyttsp_calib *calib, bool readflag)
{
    int retval = 0;
    u8 set_cmd = CY_REG_VAL_MIN;
    u8 rdata = CY_REG_VAL_MIN;
    unsigned short base = 0;

    cyttsp_press_dbg(ts, CY_DBG_LVL_2,
        "[PPD-core]%s: calib_state=%d, wait_count=%d, total_count=%d\n",
        __func__,
        calib->calib_state,
        calib->wait_count,
        calib->total_count);

    // Calibration State
    switch (calib->calib_state) {
    // Calibration Preparation
    case CY_CALIB_PRE:
        //         FW>=V00R18 : 4s wait
        // V00R09<=FW<=V00R17 : 1s wait
        if (calib->wait_count == calib->calib_pre_wait_cnt) {
            calib->calib_state = CY_CALIB_START;
            calib->wait_count = 1;

            cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                "[PPD-core]%s: Calibration Preparation Out\n", __func__);

            // Active Mode -> Low Power Mode
            if (ts->power_state == CY_ACTIVE_STATE) {
                cyttsp_press_dbg(ts, CY_DBG_LVL_2, "[PPD-core]%s: Switch to Lower Power\n", __func__);
                // switch to LowPower mode
                set_cmd = CY_OPERATE_MODE + CY_LOW_POWER_MODE;
                retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(set_cmd), &set_cmd);
                if (retval < 0) {
                    pr_err("[PPD-core]%s: set Low Power mode error. (ret=%d)\n", __func__, retval);
                    return retval;
                }
            }
        } else {
            if (calib->wait_count == 1) {
                cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                    "[PPD-core]%s: Calibration Preparation In\n", __func__);
            }

            calib->wait_count++;
            break;
        }

    // Calibration Start
    case CY_CALIB_START:
        // 30s wait x 4
        if (calib->wait_count > calib->calib_start_wait_cnt) {
            // Base Check
            calib->calib_state = CY_CALIB_BASE_CHECK;
            calib->wait_count = 1;

            cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                "[PPD-core]%s: Calibration Start Out\n", __func__);
            break;
        } else {
            if (calib->wait_count == 1) {
                cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                    "[PPD-core]%s: Calibration Start In\n", __func__);
            }

            // Do Calibration
            if (calib->wait_count % (calib->retry_count + 1) == 1) {
                // write 0x09 to address 0x11
                set_cmd = CY_REG_VAL_AUTO_CALIB;
                retval = cyttsp_press_write_block_data(ts, CY_REG_ADR_CALIB, sizeof(set_cmd), &set_cmd);
                if (retval < 0) {
                    pr_err("[PPD-core]%s: Write Calibration fail (ret=%d)\n", __func__, retval);
                    return retval;
                }
                cyttsp_press_dbg(ts, CY_DBG_LVL_1, "[PPD-core]%s: Do Calibration\n", __func__);
                calib->wait_count++;
                break;
            } else {
                // read data from address 0x11
                retval = cyttsp_press_read_block_data(ts, CY_REG_ADR_CALIB, sizeof(rdata), &rdata);
                if (retval < 0) {
                    pr_err("[PPD-core]%s: Read Calibration Complete fail (ret=%d)\n", __func__, retval);
                    return retval;
                }

                cyttsp_press_dbg(ts, CY_DBG_LVL_1, "[PPD-core]%s: Calibration rdata=0x%02X\n", __func__, rdata);
                if (CY_REG_VAL_MANUAL_CALIB_OK == (rdata & CY_REG_VAL_MANUAL_CALIB_OK)) {
                    cyttsp_press_dbg(ts, CY_DBG_LVL_1, "[PPD-core]%s: Calibration OK\n", __func__);
                    calib->calib_state = CY_CALIB_END;
                    calib->wait_count = 1;

                    cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                        "[PPD-core]%s: Calibration Start Out\n", __func__);
                } else {
                    calib->wait_count++;
                    break;
                }
            }
        }

    // Calibration End
    case CY_CALIB_END:
        // 100ms wait
        if (calib->wait_count == calib->calib_end_wait_cnt) {
            calib->calib_state = CY_CALIB_APPLY_START;
            calib->wait_count = 1;

            cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                "[PPD-core]%s: Calibration End Out\n", __func__);
        } else {
            if (calib->wait_count == 1) {
                cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                    "[PPD-core]%s: Calibration End In\n", __func__);
            }

            calib->wait_count++;
            break;
        }

    // Calibration Apply Start
    case CY_CALIB_APPLY_START:
        // 30s wait
        if (calib->wait_count > calib->calib_apply_start_wait_cnt + 1) {
            // Base Check
            calib->calib_state = CY_CALIB_BASE_CHECK;
            calib->wait_count = 1;

            cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                "[PPD-core]%s: Calibration Apply Start Out\n", __func__);
            break;
        } else {
            if (calib->wait_count == 1) {
                cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                    "[PPD-core]%s: Calibration Apply Start In\n", __func__);

                // write 0x02 to address 0x11
                set_cmd = CY_REG_VAL_AUTO_CALIB_APPLY;
                retval = cyttsp_press_write_block_data(ts, CY_REG_ADR_CALIB, sizeof(set_cmd), &set_cmd);
                if (retval < 0) {
                    pr_err("[PPD-core]%s: Write Calibration Apply fail (ret=%d)\n", __func__, retval);
                    return retval;
                }
                cyttsp_press_dbg(ts, CY_DBG_LVL_1, "[PPD-core]%s: Do Calibration Apply\n", __func__);
                calib->wait_count++;
                break;
            } else {
                // read data from address 0x11
                retval = cyttsp_press_read_block_data(ts, CY_REG_ADR_CALIB, sizeof(rdata), &rdata);
                if (retval < 0) {
                    pr_err("[PPD-core]%s: Read Calibration Apply Complete fail (ret=%d)\n", __func__, retval);
                    return retval;
                }

                cyttsp_press_dbg(ts, CY_DBG_LVL_1, "[PPD-core]%s: Calibration Apply rdata=0x%02X\n", __func__, rdata);
                if (CY_REG_VAL_MANUAL_CALIB_OK == (rdata & CY_REG_VAL_MANUAL_CALIB_OK)) {
                    cyttsp_press_dbg(ts, CY_DBG_LVL_1, "[PPD-core]%s: Calibration Apply OK\n", __func__);
                    calib->calib_state = CY_CALIB_APPLY_END;
                    calib->wait_count = 1;

                cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                    "[PPD-core]%s: Calibration Apply Start Out\n", __func__);
                } else {
                    calib->wait_count++;
                    break;
                }
            }
        }

    // Calibration Apply End
    case CY_CALIB_APPLY_END:
        if (readflag == false) {
            break;
        }

        cyttsp_press_dbg(ts, CY_DBG_LVL_1,
            "[PPD-core]%s: Calibration Apply End In/Out\n", __func__);

#if 0   // Because Framework do not use IDAC, deleted
        // read data from address 0x0F
        retval = cyttsp_press_read_block_data(ts, CY_REG_ADR_RIDAC, sizeof(rdata), &rdata);
        if (retval < 0) {
            pr_err("[PPD-core]%s: Read IDAC fail (ret=%d)\n", __func__, retval);
            return retval;
        }
        cyttsp_press_dbg(ts, CY_DBG_LVL_1, "[PPD-core]%s: IDAC=0x%02X\n", __func__, rdata);

        // report IDAC
        input_report_abs(ts->input, ABS_MT_ORIENTATION, rdata);
        input_sync(ts->input);
#endif

        // Change Calibration State
        calib->calib_state = CY_CALIB_BASE_CHECK;
        calib->wait_count = 1;
        break;

    // Calibration Base Check
    case CY_CALIB_BASE_CHECK:
        // 10s wait
        if (calib->wait_count == calib->calib_base_check_wait_cnt) {
            cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                "[PPD-core]%s: Calibration Base Check Out\n", __func__);

            // read Base from address 0x05,0x06
            retval = cyttsp_press_read_block_data(ts, CY_REG_ADR_BASE, sizeof(ts->press_data.base), &(ts->press_data.base));
            if (retval < 0) {
                pr_err("[PPD-core]%s: Read IDAC fail (ret=%d)\n", __func__, retval);
                return retval;
            }

            // Base
            base = ts->press_data.base.msb << CY_RAW_SHIT_BIT_NUM | ts->press_data.base.lsb;
            cyttsp_press_dbg(ts, CY_DBG_LVL_1, "[PPD-core]%s: Base=0x%04X(%d)\n", __func__, base, base);

            // Base Range Check
            if (base > CY_BASE_VAL_MIN && base < CY_BASE_VAL_MAX) {
                calib->calib_state = CY_CALIB_BASE_CHECK;
            } else {
                calib->calib_state = CY_CALIB_START;
            }
            calib->wait_count = 1;
        } else {
            if (calib->wait_count == 1) {
                cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                    "[PPD-core]%s: Calibration Base Check In\n", __func__);
            }

            calib->wait_count++;
        }
        break;

    default:
        break;
    }
    calib->total_count++;

    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_PRESS_WORKER
===========================================================================*/
static void cyttsp_press_press_worker(struct cyttsp *ts)
{
    int retval = 0;
    short filteredraw = 0;
    short base = 0;
    unsigned short fw_ver = ts->sysinfo_data.app_verh << CY_RAW_SHIT_BIT_NUM | ts->sysinfo_data.app_verl;

    // touch press invalid
    if (ts->tch_enabled == false) {
        cyttsp_press_dbg(ts, CY_DBG_LVL_2, "[PPD-core]%s: Touch Press Disable\n", __func__);
        return;
    }

    cyttsp_press_dbg(ts, CY_DBG_LVL_2,
        "[PPD-core]%s: touch status=%s, fw_ver=0x%04X\n",
        __func__,
        ts->tch_sts == TOUCH_DOWN ? "TOUCH_DOWN" : "LIFT_OFF",
        fw_ver);

    // Version:V00R09
    if (fw_ver >= CY_FW_VERSION_V00R09) {
        // Do Calibration
        if (ts->tch_sts == LIFT_OFF) {
            retval = cyttsp_press_do_calibration(ts, &(ts->calib), true);
            if (retval) {
                pr_err("[PPD-core]%s: Do Calibration error = %d\n", __func__, retval);
                ts->power_state = CY_IDLE_STATE;
                cyttsp_press_pr_state(ts);
                return;
            }
        }

        // Lift Off
        if ((ts->calib.calib_state != CY_CALIB_PRE) && (ts->tch_sts != TOUCH_DOWN)) {
            return;
        }
    }

    if (CY_ON == pretch_sts) {
        pretch_sts = CY_OFF;
        return;
    }

    /* Read Operational Reg */
    retval = cyttsp_read_operational_reg_sequence(ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: read fail on Operational Reg\n", __func__);
        ts->power_state = CY_IDLE_STATE;
        cyttsp_press_pr_state(ts);
        return;
    }

    if (ts->bus_ops->tsdebug > CY_DBG_LVL_1) {
        cyttsp_press_dbg(ts, CY_DBG_LVL_2, "%s: %s", __func__, cyttsp_press_reginfo_string(ts, CY_OP_REGS));
    } else {
        cyttsp_press_dbg(ts, CY_DBG_LVL_1,
            "[PPD-core]%s: "
            "%02X,"
            "%02X,"
            "%02X,"
            "%02X,%02X,"
            "%02X,%02X,"
            "%02X,%02X,"
            "%02X,%02X,"
            "%02X,%02X,"
            "%02X,%02X,"
            "%02X,"
            "%02X,"
            "%02X,"
            "%02X,"
            "%02X,"
            "%02X,%02X,"
            "%02X,"
            "%02X,"
            "%02X,"
            "%02X,"
            "%02X,%02X,"
            "%02X,"
            "%02X,"
            "%02X,"
            "%02X\n",
            __func__,
            ts->press_data.hst_mode,
            ts->press_data.tt_mode,
            ts->press_data.tt_stat,
            ts->press_data.fraw.msb, ts->press_data.fraw.lsb,
            ts->press_data.base.msb, ts->press_data.base.lsb,
            ts->press_data.tt_reserved1[0], ts->press_data.tt_reserved1[1],
            ts->press_data.diff.msb, ts->press_data.diff.lsb,
            ts->press_data.raw.msb, ts->press_data.raw.lsb,
            ts->press_data.tt_undef1[0], ts->press_data.tt_undef1[1],
            ts->press_data.idac,
            ts->press_data.baseline_reset,
            ts->press_data.calibration,
            ts->press_data.prescaler,
            ts->press_data.idac_gain,
            ts->press_data.tt_undef2[0], ts->press_data.tt_undef2[1],
            ts->press_data.act_int_intrvl,
            ts->press_data.lp_int_intrvl,
            ts->press_data.tt_undef3,
            ts->press_data.imo_tr,
            ts->press_data.fw_ver, ts->press_data.fw_rev,
            ts->press_data.alive,
            ts->press_data.lcd_vendor_id,
            ts->press_data.debug_uart,
            ts->press_data.tt_reserved2);
    }

    /* provide flow control handshake */
    if (ts->flags & CY_USE_HNDSHK) {
        if (cyttsp_press_hndshk(ts, ts->press_data.hst_mode)) {
            pr_err("[PPD-core]%s: handshake fail on operational reg\n",
                __func__);
            ts->power_state = CY_IDLE_STATE;
            cyttsp_press_pr_state(ts);
            return;
        }
    }

    /* check for any error conditions */
    if (GET_BOOTLOADERMODE(ts->press_data.tt_mode)) {
        pr_err("[PPD-core]%s: BL mode detected in active state\n", __func__);
        ts->power_state = CY_BL_STATE;
        cyttsp_press_pr_state(ts);
        return;
    } else if (GET_HSTMODE(ts->press_data.hst_mode) == GET_HSTMODE(CY_SYSINFO_MODE)) {
        pr_err("[PPD-core]%s: got SysInfo interrupt; expected touch "
            "(hst_mode=%02X)\n", __func__, ts->press_data.hst_mode);
        return;
    }

    /* report Filtered Raw */
    filteredraw = ts->press_data.fraw.msb << CY_RAW_SHIT_BIT_NUM | ts->press_data.fraw.lsb;
    base = ts->press_data.base.msb << CY_RAW_SHIT_BIT_NUM | ts->press_data.base.lsb;

    // Version:V00R09
    if (fw_ver >= CY_FW_VERSION_V00R09) {
        filteredraw = filteredraw - base;
    }

    cyttsp_press_dbg(ts, CY_DBG_LVL_1,
        "[PPD-core]%s: FRAW=0x%02X%02X BASE=0x%02X%02X DIFF=0x%02X%02X RAW=0x%02X%02X IDAC=0x%02X input_report_abs=%d\n",
        __func__,
        ts->press_data.fraw.msb,
        ts->press_data.fraw.lsb,
        ts->press_data.base.msb,
        ts->press_data.base.lsb,
        ts->press_data.diff.msb,
        ts->press_data.diff.lsb,
        ts->press_data.raw.msb,
        ts->press_data.raw.lsb,
        ts->press_data.idac,
        filteredraw);

    if (filteredraw != prev_filteredraw) {
        prev_filteredraw = filteredraw;
        input_report_abs(ts->input, ABS_MT_PRESSURE, filteredraw);
        input_sync(ts->input);
    }
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_RECOVERY
===========================================================================*/
static void cyttsp_press_recovery(struct cyttsp *ts)
{
    pr_info("[PPD-core]%s: IN PS=%d\n", __func__, ts->power_state);

    /* change state */
    ts->power_state = CY_READY_STATE;
    /* startup */
    cyttsp_press_startup(ts);

    pr_info("[PPD-core]%s: Recovery Complete. OUT PS=%d\n", __func__, ts->power_state);
    return;
}

#ifdef CONFIG_TOUCHPRESS_DEBUG
/*===========================================================================
    FUNCTION  CYTTSP_PRESS_DRV_ATTR_SHOW
===========================================================================*/
static ssize_t cyttsp_press_drv_attr_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct cyttsp *ts = dev_get_drvdata(dev);
    int retval = 0;
    int ret = 0;
    char irqinfo[20];

    pr_info("[PPD-core]%s: drv_attr_mode = %d\n", __func__, ts->drv_attr_mode);

    switch (ts->drv_attr_mode) {
    case CY_DRV_BASE_INFO:
        snprintf(irqinfo, sizeof(irqinfo), "%s", "UNKNOWN");
        if (ts->platform_data->irq_stat) {
            ret = ts->platform_data->irq_stat();
            switch (ret) {
            case 0:
                snprintf(irqinfo, sizeof(irqinfo), "%s", "LOW");
                break;
            case 1:
                snprintf(irqinfo, sizeof(irqinfo), "%s", "HIGH");
                break;
            default:
                snprintf(irqinfo, sizeof(irqinfo), "%s(%d)", "UNKNOWN", ret);
                break;
            }
        }

        retval = snprintf(buf, CY_MAX_PRBUF_SIZE,
            "[PPD-core] Driver Base Info:\n"
            "  Driver Name:                     %s\n"
            "  Version:                         %s\n"
            "  Date:                            %s\n"
            "  Reset GPIO Pin:                  %d\n"
            "  INT GPIO Pin:                    %d\n"
            "  SCL GPIO Pin:                    %d\n"
            "  SDA GPIO Pin:                    %d\n"
            "  Slaver Address:                  0x%02X\n"
            "  Power State:                     %s\n"
            "  IRQ Status:                      %s\n"
            "  Interrupt Line:                  %s\n"
            "  Debug Level:                     %u\n",
            ts->input->name,
            CY_PR_DRIVER_VERSION,
            CY_PR_DRIVER_DATE,
            CY_PR_I2C_XRST_GPIO,
            CY_PR_I2C_IRQ_GPIO,
            SCL,
            SDA,
            CY_PR_I2C_ADR,
            ts->poweron ? cyttsp_press_ps_string(ts) : "PowerOff",
            ts->irq_enabled ? "enabled" : "disabled",
            irqinfo,
            ts->bus_ops->tsdebug);
        break;

    case CY_OPTION_INFO:
        retval = snprintf(buf, CY_MAX_PRBUF_SIZE,
            "[PPD-core] Driver Option Info:\n"
            "  Press Value Read Count:          %d\n"
            "  Invaild Buffer Read Count:       %d\n"
            "  Set Calibration Mode Loop Count: %d\n"
            "  Read Press Value Interval:       %dms\n"
            "  Switch to Manual Calibration:    %dms\n"
            "  Switch to Automatic Calibration: %dms\n"
            "  Read/Write Regs Address:         0x%02X\n",
            ts->press_value_count,
            ts->invalid_buf_read_count,
            ts->calib.retry_count,
            ts->sleep_intrvl[READ_PRESS_INTRVL],
            ts->sleep_intrvl[TO_MANUAL_CALIB],
            ts->sleep_intrvl[TO_AUTO_CALIB],
            ts->rw_regid);
        break;

    case CY_BOOT_REG_INFO:
        if (!ts->poweron) {
            pr_err("[PPD-core]%s: Can Not Read BootLoader Regs Because Of Power Off\n", __func__);
        } else {
            ret = cyttsp_press_read_block_data(ts, CY_REG_BASE, sizeof(ts->bl_data), &(ts->bl_data));
            if (ret < 0) {
                pr_err("[PPD-core] Read BootLoader Regs error:%d\n", ret);
                retval = snprintf(buf, CY_MAX_PRBUF_SIZE,
                    "[PPD-core] BootLoader Regs Info:  Read Error %d\n", ret);
            } else {
                retval = snprintf(buf, CY_MAX_PRBUF_SIZE,
                    "%s", cyttsp_press_reginfo_string(ts, CY_BOOT_REGS));
            }
        }
        break;

    case CY_SYS_REG_INFO:
        if (!ts->poweron) {
            pr_err("[PPD-core]%s: Can Not Read SystemInformation Regs Because Of Power Off\n", __func__);
        } else {
            ret = cyttsp_press_read_block_data(ts, CY_REG_BASE, sizeof(ts->sysinfo_data), &(ts->sysinfo_data));
            if (ret < 0) {
                pr_err("[PPD-core] Read SystemInformation Regs error:%d\n", ret);
                retval = snprintf(buf, CY_MAX_PRBUF_SIZE,
                    "[PPD-core] SystemInformation Regs Info:  Read Error %d\n", ret);
            } else {
                retval = snprintf(buf, CY_MAX_PRBUF_SIZE,
                    "%s", cyttsp_press_reginfo_string(ts, CY_SYS_REGS));
            }
        }
        break;

    case CY_OP_REG_INFO:
        if (!ts->poweron) {
            pr_err("[PPD-core]%s: Can Not Read Operating Regs Because Of Power Off\n", __func__);
        } else {
            ret = cyttsp_press_read_block_data(ts, CY_REG_BASE, sizeof(struct cyttsp_press_data), &(ts->press_data));
            if (ret < 0) {
                pr_err("[PPD-core] Read Operating Regs error:%d\n", ret);
                retval = snprintf(buf, CY_MAX_PRBUF_SIZE,
                    "[PPD-core] Operating Regs Info:  Read Error %d\n", ret);
            } else {
                retval = snprintf(buf, CY_MAX_PRBUF_SIZE,
                    "%s", cyttsp_press_reginfo_string(ts, CY_OP_REGS));
            }
        }
        break;

    default:
        break;
    }

    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_DRV_ATTR_STORE
===========================================================================*/
static ssize_t cyttsp_press_drv_attr_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct cyttsp *ts = dev_get_drvdata(dev);
    unsigned long value = 0;
    int retval = 0;
    int i = 0;
    int j = 0;
    char inbuf[256];
    char inbuf_tmp[256];
    char echoKey[256];
    char *cp = NULL;
    char *p_value;
#ifdef CY_TOUCH_PRESS_DEBUG_TOOLS
    u8 host_reg = CY_REG_VAL_MAX;
#endif

    pr_info("[PPD-core]%s: keynum=%d buf=%s\n", __func__, CY_SUPPORT_ECHO_KEY, buf);

    if (size > 256) {
        pr_err("[PPD-core]%s: Err, data too large\n", __func__);
        retval = -EOVERFLOW;
        goto cyttsp_press_drv_attr_store_error_exit;
    }

    // trim space
    memset(inbuf, 0, 256);
    for (i = 0; i < size; i++) {
        if (buf[i] == CY_MARK_SPACE
            || buf[i] == CY_MARK_CR
            || buf[i] == CY_MARK_LF) {
            continue;
        }

        if (isalpha(buf[i])) {
            inbuf[j] = toupper(buf[i]);
        } else {
            inbuf[j] = buf[i];
        }
        j++;
    }

    for (i = 0; i < CY_SUPPORT_ECHO_KEY; i++) {
        strcpy(inbuf_tmp, inbuf);
        snprintf(echoKey, sizeof(echoKey), "%s%c", cyttsp_echo_keys[i], CY_MARK_EQUAL);

        // find echo key
        cp = strstr(inbuf_tmp, echoKey);
        if (cp == NULL)
            continue;

        cp += strlen(echoKey);
        p_value = strsep(&cp, CY_DELIMITER_COMMA);
        if (p_value != NULL && strlen(p_value) != 0) {
            retval = strict_strtoul(p_value, 10, &value);
            if (retval < 0) {
                retval = strict_strtoul(p_value, 16, &value);
                if (retval < 0) {
                    pr_err("%s: %s Failed to convert value\n", __func__, cyttsp_echo_keys[i]);
                    goto cyttsp_press_drv_attr_store_error_exit;
                }
            }
        } else {
            goto cyttsp_press_drv_attr_store_error_exit;
        }

        // echo key
        pr_info("[PPD-core]%s: echoKey[%d] = %s, value = %d\n", __func__, i, cyttsp_echo_keys[i], (int)value);

        switch (i) {
        // View Mode
        case 0:
            if (value > CY_OP_REG_INFO || value < CY_DRV_BASE_INFO) {
                value = CY_DRV_BASE_INFO;
            }
            pr_info("[PPD-core]%s: Driver View Mode setting = %d\n", __func__, (int)value);
            ts->drv_attr_mode = value;
            break;

#ifdef CY_TOUCH_PRESS_DEBUG_TOOLS
        // Irq enabled/disabled
        case 1:
            if (value > 1 || value < 0) {
                pr_err("[PPD-core]%s: Irq Setting Error\n", __func__);
                break;
            }

            if (ts->irq_enabled == false) {
                if (value == 1) {
                    /* Enable IRQ */
                    cyttsp_press_enable_irq(ts);
                    pr_info("[PPD-core]%s: Driver IRQ now enabled\n", __func__);
                } else {
                    pr_info("[PPD-core]%s: Driver IRQ already disabled\n", __func__);
                }
            } else {
                if (value == 0) {
                    /* Disable IRQ */
                    disable_irq_nosync(ts->irq);
                    ts->irq_enabled = false;
                    pr_info("[PPD-core]%s: Driver IRQ now disabled\n", __func__);
                } else {
                    pr_info("[PPD-core]%s: Driver IRQ already enabled\n", __func__);
                }
            }
            break;

        // Press Value Read Count
        case 2:
            if (value > 100000 || value < 1) {
                value = CY_PRESS_VALUE_READ_CNT;
            }
            pr_info("[PPD-core]%s: Press Value Count setting = %d\n", __func__, (int)value);
            ts->press_value_count = value;
            break;

        // Invaild Buffer Read Count
        case 3:
            if (value > 100 || value < 1) {
                value = CY_INVALID_BUFFER_READ_CNT;
            }
            pr_info("[PPD-core]%s: Press Value Count setting = %d\n", __func__, (int)value);
            ts->invalid_buf_read_count = value;
            break;

        // Set Calibration Mode Loop Count
        case 4:
            if (value > 100 || value < 1) {
                value = CY_CALI_WAIT_MAX_CNT;
            }
            pr_info("[PPD-core]%s: Calibration Mode Loop Count setting = %d\n", __func__, (int)value);
            ts->calib.retry_count = value;
            ts->calib.calib_start_wait_cnt = value * (CY_CALI_WAIT_MAX_CNT + 1);
            break;

        // Read Press Value Interval
        case 5:
            if (value > 10000 || value < 1) {
                value = CY_INTRVL_READ_PRESS;
            }
            pr_info("[PPD-core]%s: Read Press Value Interval setting = %dms\n", __func__, (int)value);
            ts->sleep_intrvl[READ_PRESS_INTRVL] = value;
            break;

        // Switch to Manual Calibration
        case 6:
            if (value > 10000 || value < 1) {
                value = CY_INTRVL_TO_MANU_CALIB;
            }
            pr_info("[PPD-core]%s: Switch to Manual Calibration Interval setting = %dms\n", __func__, (int)value);
            ts->sleep_intrvl[TO_MANUAL_CALIB] = value;
            break;

        // Switch to Automatic Calibration
        case 7:
            if (value > 10000 || value < 1) {
                value = CY_INTRVL_TO_AUTO_CALIB;
            }
            pr_info("[PPD-core]%s: Switch to Automatic Calibration setting = %dms\n", __func__, (int)value);
            ts->sleep_intrvl[TO_AUTO_CALIB] = value;
            break;

        // Read/Write Regs Address
        case 8:
            if (value > CY_REG_ADR_MAX) {
                value = CY_REG_ADR_MAX;
            }
            pr_info("[PPD-core]%s: Read/Write Regs Address setting = 0x%02X\n", __func__, (int)value);
            ts->rw_regid = value;
            break;

        // FW_MODE
        case 9:
            switch (value) {
            // SUSPEND
            case 1:
                pr_info("[PPD-core]%s: SUSPEND (ts = %p)\n", __func__, ts);
                cyttsp_press_suspend(ts);
                break;

            // RESUME
            case 2:
                pr_info("[PPD-core]%s: RESUME (ts = %p)\n", __func__, ts);
                cyttsp_press_resume(ts);
                break;

            // RESET
            case 3:
                pr_info("%s: RESET (ts=%p)\n", __func__, ts);
                if (!ts->poweron) {
                    pr_err("[PPD-core]%s: Can Not Reset Because Of Power Off\n", __func__);
                } else {
                    mutex_lock(&ts->data_lock);
                    ts->power_state = CY_INVALID_STATE;
                    retval = cyttsp_press_startup(ts);
                    mutex_unlock(&ts->data_lock);
                }
                break;

            // BootLoader Mode
            case 4:
                pr_info("%s: Switch to BootLoader Mode\n", __func__);
                if (!ts->poweron) {
                    pr_err("[PPD-core]%s: Can Not Switch to BootLoader Mode Because Of Power Off\n", __func__);
                } else {
                    /* swtich TTSP Device to BootLoader mode */
                    host_reg = CY_SOFT_RESET_MODE;
                    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(host_reg), &host_reg);
                    if (retval < 0) {
                        pr_err("[PPD-core]%s: Can Not Switch to BootLoader Mode\n", __func__);
                    } else {
                        msleep(1000);
                        /* read BootLoader registers */
                        retval = cyttsp_press_read_block_data(ts, CY_REG_BASE, sizeof(ts->bl_data), &(ts->bl_data));
                        if (retval < 0) {
                            pr_err("[PPD-core]%s: BootLoader registers access err (ret=%d)\n", __func__, retval);
                        } else {
                            pr_info("%s: %s", __func__, cyttsp_press_reginfo_string(ts, CY_BOOT_REGS));
                        }
                    }
                }
                break;

            // SystemInformation Mode
            case 5:
                pr_info("%s: Switch to SystemInformation Mode\n", __func__);
                if (!ts->poweron) {
                    pr_err("[PPD-core]%s: Can Not Switch to SystemInformation Mode Because Of Power Off\n", __func__);
                } else {
                    /* swtich TTSP Device to SystemInformation mode */
                    host_reg = CY_SYSINFO_MODE;
                    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(host_reg), &host_reg);
                    if (retval < 0) {
                        pr_err("[PPD-core]%s: Can Not Switch to SystemInformation Mode\n", __func__);
                    } else {
                        msleep(1000);
                        /* read SystemInformation registers */
                        retval = cyttsp_press_read_block_data(ts, CY_REG_BASE, sizeof(ts->sysinfo_data), &(ts->sysinfo_data));
                        if (retval < 0) {
                            pr_err("[PPD-core]%s: SystemInformation registers access err (ret=%d)\n", __func__, retval);
                        } else {
                            pr_info("%s: %s", __func__, cyttsp_press_reginfo_string(ts, CY_SYS_REGS));
                        }
                    }
                }
                break;

            // Active Mode
            case 6:
                if (host_reg == CY_REG_VAL_MAX) {
                    host_reg = CY_OPERATE_MODE;
                    pr_info("%s: Switch to Operating Active Mode\n", __func__);
                }
            // DeepSleep Mode
            case 7:
                if (host_reg == CY_REG_VAL_MAX) {
                    host_reg = CY_DEEP_SLEEP_MODE;
                    pr_info("%s: Switch to Operating DeepSleep Mode\n", __func__);
                }
            // Lower Power Mode
            case 8:
            default:
                if (host_reg == CY_REG_VAL_MAX) {
                    host_reg = CY_LOW_POWER_MODE;
                    pr_info("%s: Switch to Operating LowerPower Mode\n", __func__);
                }
                if (!ts->poweron) {
                    pr_err("[PPD-core]%s: Can Not Switch to Operating Mode Because Of Power Off\n", __func__);
                } else {
                    /* swtich TTSP Device to Operating mode */
                    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(host_reg), &host_reg);
                    if (retval < 0) {
                        pr_err("[PPD-core]%s: Can Not Switch to Operating Mode\n", __func__);
                    } else {
                        msleep(1000);
                        /* read Operating registers */
                        retval = cyttsp_press_read_block_data(ts, CY_REG_BASE, sizeof(struct cyttsp_press_data), &(ts->press_data));
                        if (retval < 0) {
                            pr_err("[PPD-core]%s: Operating registers access err (ret=%d)\n", __func__, retval);
                        } else {
                            pr_info("%s: %s", __func__, cyttsp_press_reginfo_string(ts, CY_OP_REGS));
                        }
                    }
                }
                break;
            }
            break;
#endif

        default:
            break;
        }
    }

    retval = size;

cyttsp_press_drv_attr_store_error_exit:
    return retval;
}

static DEVICE_ATTR(drv_attr, S_IRUSR | S_IWUSR,
    cyttsp_press_drv_attr_show, cyttsp_press_drv_attr_store);

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_DRV_DEBUG_STORE
===========================================================================*/
static ssize_t cyttsp_press_drv_debug_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct cyttsp *ts = dev_get_drvdata(dev);
    int retval = 0;
    unsigned long value;

    retval = strict_strtoul(buf, 10, &value);
    if (retval < 0) {
        pr_err("[PPD-core]%s: Failed to convert value\n", __func__);
        goto cyttsp_press_drv_debug_store_exit;
    }

    switch (value) {
    case CY_DBG_LVL_0:
    case CY_DBG_LVL_1:
    case CY_DBG_LVL_2:
    case CY_DBG_LVL_3:
        pr_info("[PPD-core]%s: Debug setting = %d\n", __func__, (int)value);
        ts->bus_ops->tsdebug = value;
        break;
    default:
        ts->bus_ops->tsdebug = CY_DBG_LVL_0;
        pr_info("[PPD-core]%s: Invalid Debug setting; set to min = %d\n",
            __func__, ts->bus_ops->tsdebug);
        break;
    }

    retval = size;

cyttsp_press_drv_debug_store_exit:
    return retval;
}
static DEVICE_ATTR(drv_debug, S_IWUSR,
    NULL, cyttsp_press_drv_debug_store);

#ifdef CY_TOUCH_PRESS_DEBUG_TOOLS
/*===========================================================================
    FUNCTION  CYTTSP_PRESS_DRV_RW_REG_DATA_SHOW
===========================================================================*/
static ssize_t cyttsp_press_drv_rw_reg_data_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct cyttsp *ts = dev_get_drvdata(dev);
    int retval = 0;
    u8 reg_data = CY_REG_VAL_MIN;

    if (!ts->poweron) {
        pr_err("[PPD-core]%s: Read/Write Regid(%02X(%d) Failed Because Of Power Off\n",
            __func__, ts->rw_regid, ts->rw_regid);
        return retval;
    }

    retval = cyttsp_press_read_block_data(ts,
        CY_REG_BASE + ts->rw_regid, sizeof(reg_data), &reg_data);

    if (retval < 0)
        return snprintf(buf, CY_MAX_PRBUF_SIZE,
            "[PPD-core]%s: Read/Write Regid(%02X(%d) Failed\n",
            __func__, ts->rw_regid, ts->rw_regid);
    else
        return snprintf(buf, CY_MAX_PRBUF_SIZE,
            "[PPD-core]%s: Read/Write Regid=%02X(%d) Data=%02X(%d)\n",
            __func__, ts->rw_regid, ts->rw_regid, reg_data, reg_data);
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_DRV_RW_REG_DATA_STORE
===========================================================================*/
static ssize_t cyttsp_press_drv_rw_reg_data_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    struct cyttsp *ts = dev_get_drvdata(dev);
    int retval = 0;
    unsigned long value;
    u8 cmd = CY_REG_VAL_MIN;

    if (!ts->poweron) {
        pr_err("[PPD-core]%s: Write Register Error Because Of Power Off\n",
            __func__);
        goto cyttsp_press_drv_rw_reg_data_store_exit;
    }

    retval = strict_strtoul(buf, 16, &value);
    if (retval < 0) {
        pr_err("[PPD-core]%s: Failed to convert value\n",
            __func__);
        goto cyttsp_press_drv_rw_reg_data_store_exit;
    }

    if (value > CY_REG_VAL_MAX) {
        pr_err("[PPD-core]%s: Invalid Register Data Range; no write\n", __func__);
    } else {
        cmd = (u8)value;
        retval = cyttsp_press_write_block_data(ts, CY_REG_BASE + ts->rw_regid,
            sizeof(cmd), &cmd);
        if (retval < 0) {
            pr_err("[PPD-core]%s: Failed write to Regid=%02X(%d)\n",
                __func__, ts->rw_regid, ts->rw_regid);
        }
    }

    retval = size;

cyttsp_press_drv_rw_reg_data_store_exit:
    return retval;
}
static DEVICE_ATTR(drv_rw_reg_data, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
    cyttsp_press_drv_rw_reg_data_show, cyttsp_press_drv_rw_reg_data_store);

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_FIRMWARE_CLASS_LOADER
===========================================================================*/
static int cyttsp_press_firmware_class_loader(struct cyttsp *ts,
    const u8 *data, int size)
{
    int retval = 0;
    const u8 *img = data + data[0] + 1;
    int img_size = size - (data[0] + 1);
    int i = 0, tries = 0;
    u8 host_reg;
    int loc = 0;
    int fw_size = 0;

    if (img_size != CY_BL_FW_IMG_SIZE) {
        pr_err("[PPD-core]%s: bad file image size=%d expect=%d\n", __func__, img_size, CY_BL_FW_IMG_SIZE);
    } else {
        pr_info("[PPD-core]%s: load new firmware --- ttsp_ver=0x%02X%02X app_id=0x%02X%02X app_ver=0x%02X%02X cid=0x%02X%02X%02X\n",
            __func__,
            data[1], data[2],
            data[3], data[4],
            data[5], data[6],
            data[7], data[8], data[9]);

        fw_size = img_size;
        /* reset TTSP Device back to bootloader mode */
        host_reg = CY_SOFT_RESET_MODE;
        retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(host_reg), &host_reg);

        /* wait for TTSP Device to complete reset back to bootloader */
        tries = 0;
        do {
            usleep_range(1000, 1000);
            cyttsp_press_mc_putbl(ts);
        } while (ts->bl_data.bl_status != CY_BL_READY_NO_APP && ts->bl_data.bl_status != CY_BL_READY_APP && tries++ < 100);
        pr_info("%s(RESET - tires:%d): %s", __func__, tries, cyttsp_press_reginfo_string(ts, CY_BOOT_REGS));

        /* download new TTSP Application to the Bootloader */
        if (!(retval < 0)) {
            i = 0;

            /* send bootload initiation command */
            ts->bl_data.bl_file = 0;
            ts->bl_data.bl_status = 0;
            ts->bl_data.bl_error = 0;
            retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, CY_BL_ENTER_CMD_SIZE, &img[loc]);

            /* delay to allow bl to get ready for block writes */
            i++;
            tries = 0;
            do {
                msleep(100);
                cyttsp_press_mc_putbl(ts);
            } while (ts->bl_data.bl_status != CY_BL_READY_NO_APP && ts->bl_data.bl_status != CY_BL_READY_APP && tries++ < 100);
            pr_info("%s(INIT - tires:%d): %s", __func__, tries, cyttsp_press_reginfo_string(ts, CY_BOOT_REGS));

            /* send bootload firmware load blocks */
            if (!(retval < 0)) {
                loc += CY_BL_ENTER_CMD_SIZE;
                while ((fw_size - loc) > CY_BL_WR_BLK_CMD_SIZE) {
                    retval = cyttsp_press_mc_wr_blk_chunks(ts, CY_REG_BASE, CY_BL_WR_BLK_CMD_SIZE, &img[loc]);

                    pr_info("[PPD-core]%s: Bootloader Record=%3d bl_file=%02X bl_status=%02X bl_error=%02X loc=%d\n", __func__,
                        (loc / CY_BL_WR_BLK_CMD_SIZE) + 1,
                        ts->bl_data.bl_file, ts->bl_data.bl_status,
                        ts->bl_data.bl_error, loc);

                    i++;
                    if (retval < 0) {
                        pr_err("[PPD-core]%s: Bootloader Download Fail Record=%3d retval=%d\n",
                            __func__, (loc / CY_BL_WR_BLK_CMD_SIZE) + 1, retval);
                        break;
                    } else {
                        tries = 0;
                        cyttsp_press_mc_putbl(ts);

                        while (
                            !((ts->bl_data.bl_status == CY_BL_READY_NO_APP) &&
                            (ts->bl_data.bl_error == CY_BL_RUNNING)) &&
                            !((ts->bl_data.bl_status == CY_BL_READY_APP) &&
                            (ts->bl_data.bl_error == CY_BL_RUNNING)) &&
                            (tries++ < 100)) {
                            usleep_range(1000, 1000);
                            cyttsp_press_mc_putbl(ts);
                        }
                        loc += CY_BL_WR_BLK_CMD_SIZE;
                    }
                }

                /* send bootload terminate command */
                pr_info("[PPD-core]%s: Send BL Terminate\n", __func__);
                if (loc == (fw_size - CY_BL_EXIT_CMD_SIZE)) {
                    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, CY_BL_EXIT_CMD_SIZE, &img[loc]);

                    i++;
                    tries = 0;
                    do {
                        msleep(100);
                        cyttsp_press_mc_putbl(ts);
                    } while (ts->bl_data.bl_status != CY_BL_READY_NO_APP && ts->bl_data.bl_status != CY_BL_READY_APP && tries++ < 100);
                    pr_info("%s(TERM- tires:%d): %s", __func__, tries, cyttsp_press_reginfo_string(ts, CY_BOOT_REGS));
                } else {
                    pr_err("[PPD-core]%s: FW size mismatch\n", __func__);
                    retval = -1;
                    return retval;
                }
            }
        }

        /* reset TTSP Device back to bootloader mode */
        host_reg = CY_SOFT_RESET_MODE;
        retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(host_reg), &host_reg);

        /* wait for TTSP Device to complete reset back to bootloader */
        tries = 0;
        do {
            usleep_range(1000, 1000);
            cyttsp_press_mc_putbl(ts);
        } while (ts->bl_data.bl_status != CY_BL_READY_NO_APP && ts->bl_data.bl_status != CY_BL_READY_APP && tries++ < 100);
        pr_info("%s(FINISHED - tires:%d): %s", __func__, tries, cyttsp_press_reginfo_string(ts, CY_BOOT_REGS));

        msleep(1000);
        retval = 0;
    }

    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_FIRMWARE_CONT
===========================================================================*/
static void cyttsp_press_firmware_cont(const struct firmware *fw, void *context)
{
    int retval = 0;
    struct device *dev = context;
    struct cyttsp *ts = dev_get_drvdata(dev);

    if (!fw) {
        pr_err("[PPD-core]%s: firmware not found\n", __func__);
        return;
    }

    // resume
    retval = cyttsp_press_resume(ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: Cannot resume. (ret=%d)\n", __func__, retval);
        return;
    }

    cyttsp_press_disable_irq(ts);
    mutex_lock(&ts->data_lock);
    ts->tch_enabled = false;
    ts->waiting_for_fw = true;

    // FW DownLoad
    retval = cyttsp_press_firmware_class_loader(ts, fw->data, fw->size);
    if (retval < 0) {
        pr_err("[PPD-core]%s: Cannot Load Firmware. (ret=%d)\n", __func__, retval);
        retval = -EFAULT;
    } else {
        pr_info("[PPD-core]%s: Load Firmware OK.\n", __func__);
    }

    // restart
    ts->power_state = CY_INVALID_STATE;
    cyttsp_press_enable_irq(ts);
    cyttsp_press_startup(ts);
    ts->tch_enabled = true;
    ts->waiting_for_fw = false;
    mutex_unlock(&ts->data_lock);

    release_firmware(fw);
    return;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_IC_REFLASH_SHOW
===========================================================================*/
static ssize_t cyttsp_press_ic_reflash_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct cyttsp *ts = dev_get_drvdata(dev);

    return snprintf(buf, CY_MAX_PRBUF_SIZE,
        "[PPD-core]%s\n", ts->waiting_for_fw ?
        "Driver is waiting for firmware load" :
        "No firmware loading in progress");
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_IC_REFLASH_STORE
===========================================================================*/
static ssize_t cyttsp_press_ic_reflash_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    int i = 0;
    int retval = 0;
    struct cyttsp *ts = dev_get_drvdata(dev);

    if (ts->waiting_for_fw) {
        pr_err("[PPD-core]%s: Driver is already waiting for load firmware\n", __func__);
        retval = -EALREADY;
        return retval;
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
        pr_err("[PPD-core]%s: Filename too long\n", __func__);
        retval = -ENAMETOOLONG;
        return retval;
    } else {
        /*
         * name string must be in alloc() memory
         * or is lost on context switch
         * strip off any line feed character(s)
         * at the end of the buf string
         */
        for (i = 0; buf[i]; i++) {
            if (buf[i] < CY_MARK_SPACE)
                ts->fwname[i] = 0;
            else
                ts->fwname[i] = buf[i];
        }
    }

    pr_info("[PPD-core]%s: Enabling firmware class loader\n", __func__);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
    retval = request_firmware_nowait(THIS_MODULE,
        FW_ACTION_NOHOTPLUG, (const char *)ts->fwname, ts->dev,
        GFP_KERNEL, ts->dev, cyttsp_press_firmware_cont);
#else /* Kernel 2.6.32 */
    retval = request_firmware_nowait(THIS_MODULE,
        FW_ACTION_NOHOTPLUG, (const char *)ts->fwname, ts->dev,
        ts->dev, cyttsp_press_firmware_cont);
#endif

    if (retval) {
        pr_err("[PPD-core]%s: Fail request firmware class file load\n", __func__);
    } else {
        retval = size;
    }

    return retval;
}
static DEVICE_ATTR(ic_reflash, S_IRUSR | S_IWUSR,
    cyttsp_press_ic_reflash_show, cyttsp_press_ic_reflash_store);
#endif
#endif

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_LDR_INIT
===========================================================================*/
static void cyttsp_press_ldr_init(struct cyttsp *ts)
{
#ifdef CONFIG_TOUCHPRESS_DEBUG
    if (device_create_file(ts->dev, &dev_attr_drv_debug))
        pr_err("[PPD-core]%s: Cannot create drv_debug\n", __func__);

    if (device_create_file(ts->dev, &dev_attr_drv_attr))
        pr_err("[PPD-core]%s: Cannot create drv_attr\n", __func__);

#ifdef CY_TOUCH_PRESS_DEBUG_TOOLS
    if (device_create_file(ts->dev, &dev_attr_drv_rw_reg_data))
        pr_err("[PPD-core]%s: Cannot create drv_rw_reg_data\n", __func__);

    if (device_create_file(ts->dev, &dev_attr_ic_reflash))
        pr_err("[PPD-core]%s: Cannot create ic_reflash\n", __func__);

#endif
#endif
    return;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_LDR_FREE
===========================================================================*/
static void cyttsp_press_ldr_free(struct cyttsp *ts)
{
#ifdef CONFIG_TOUCHPRESS_DEBUG
    device_remove_file(ts->dev, &dev_attr_drv_debug);
    device_remove_file(ts->dev, &dev_attr_drv_attr);
#ifdef CY_TOUCH_PRESS_DEBUG_TOOLS
    device_remove_file(ts->dev, &dev_attr_drv_rw_reg_data);
    device_remove_file(ts->dev, &dev_attr_ic_reflash);
#endif
#endif
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_STARTUP
===========================================================================*/
static int cyttsp_press_startup(struct cyttsp *ts)
{
    int retval = 0;

    pr_info("[PPD-core]%s: Cypress TouchDriver Startup\n", __func__);

    cyttsp_press_touch_liftoff(ts);
    ts->calib.calib_state = CY_CALIB_START;

    retval = cyttsp_press_hw_reset(ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: HW reset failure\n", __func__);
        ts->power_state = CY_INVALID_STATE;
        cyttsp_press_pr_state(ts);
        goto bypass;
    }
    retval = cyttsp_press_bl_app_valid(ts);
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
    retval = cyttsp_press_exit_bl_mode(ts);
    if (retval < 0) {
        goto bypass;
    }

    if (ts->power_state != CY_READY_STATE) {
        goto bypass;
    }

no_bl_bypass:
    /* At this point, we are in READY to go active state */
    retval = cyttsp_press_set_sysinfo_mode(ts);
    if (retval < 0) {
        goto op_bypass;
    }

    retval = cyttsp_press_set_sysinfo_regs(ts);
    if (retval < 0) {
        goto bypass;
    }

op_bypass:
    retval = cyttsp_press_set_operational_mode(ts);
    if (retval < 0) {
        goto bypass;
    }

    /* init active distance */
    ts->power_state = CY_ACTIVE_STATE;
    pr_info("[PPD-core]%s: Cypress TouchPress Driver is Active\n", __func__);
bypass:
    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_IRQ
===========================================================================*/
static irqreturn_t cyttsp_press_irq(int irq, void *handle)
{
    struct cyttsp *ts = handle;
    int retval = 0;
    struct timeval tv;

    /* 2012/05/14 PPD NULL Check Start */
    if (NULL == ts) {
        pr_info("[PPD-core]%s: ts = NULL\n",
              __func__); 
        return IRQ_HANDLED;
    }
    if (NULL == ts->bus_ops) {
        pr_info("[PPD-core]%s: ts->bus_ops = NULL\n",
              __func__); 
        return IRQ_HANDLED;
    }
    if (NULL == &(ts->bl_data)) {
        pr_info("[PPD-core]%s: ts->bl_data = NULL\n",
              __func__); 
        return IRQ_HANDLED;
    }
    /* 2012/05/14 PPD NULL Check End */

    if (CY_DBG_LVL_1 < ts->bus_ops->tsdebug) {
        do_gettimeofday(&tv);
        cyttsp_press_dbg(ts, CY_DBG_LVL_2, "[PPD-core]%s: IN  time:%lu%06lu usec\n",
            __func__, tv.tv_sec, tv.tv_usec);
    }
    cyttsp_press_dbg(ts, CY_DBG_LVL_3,
        "[PPD-core]%s: GOT IRQ ps=%s\n", __func__, cyttsp_press_ps_string(ts));

    switch (ts->power_state) {
    case CY_BL_STATE:
        complete(&ts->bl_int_running);
        break;
    case CY_SYSINFO_STATE:
        complete(&ts->si_int_running);
        break;
    case CY_IDLE_STATE:
        /* device now available; signal initialization */
        pr_info("[PPD-core]%s: Received IRQ in IDLE state\n",
            __func__);
        /* Try to determine the IC's current state */
        retval = cyttsp_press_load_bl_regs(ts);
        if (retval < 0) {
            pr_err("[PPD-core]%s: Still unable to find IC after IRQ\n",
                __func__);
        } else if (GET_BOOTLOADERMODE(ts->bl_data.bl_status)) {
            pr_info("[PPD-core]%s: BL mode found in IDLE state\n",
                __func__);
            ts->power_state = CY_BL_STATE;
            cyttsp_press_pr_state(ts);
        } else {
            pr_info("[PPD-core]%s: ACTIVE mode found in IDLE state\n",
                __func__);
            mutex_lock(&ts->data_lock);
            ts->power_state = CY_ACTIVE_STATE;
            cyttsp_press_pr_state(ts);
            cyttsp_press_press_worker(ts);
            mutex_unlock(&ts->data_lock);
        }
        break;
    case CY_ACTIVE_STATE:
        mutex_lock(&ts->data_lock);
        cyttsp_press_press_worker(ts);
        mutex_unlock(&ts->data_lock);

        if (ts->power_state == CY_BL_STATE || ts->power_state == CY_IDLE_STATE) {
            pr_info("[PPD-core]%s: Reached BL/IDLE state in IRQ PS=%d\n",
                __func__, ts->power_state);
            /*
             * TTSP device has reset back to bootloader mode
             * operational mode
             */
            cyttsp_press_recovery(ts);
        }
        break;
    default:
        break;
    }

    if (CY_DBG_LVL_1 < ts->bus_ops->tsdebug) {
        do_gettimeofday(&tv);
        cyttsp_press_dbg(ts, CY_DBG_LVL_2, "[PPD-core]%s: OUT time:%lu%06lu usec\n",
            __func__, tv.tv_sec, tv.tv_usec);
    }

    return IRQ_HANDLED;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_POWER_ON
===========================================================================*/
static int cyttsp_press_power_on(struct cyttsp *ts)
{
    int retval = 0;

    if (ts->poweron) {
        pr_info("[PPD-core]%s: Touch Press Already PowerOn\n", __func__);
        return retval;
    }

    /* Communicate with the IC */
    mutex_lock(&ts->data_lock);
    ts->poweron = true;
    ts->power_state = CY_INVALID_STATE;
    cyttsp_press_enable_irq(ts);

    if (ts->platform_data->hw_power) {
        retval = ts->platform_data->hw_power(CY_ON);
        if (retval < 0) {
            pr_err("[PPD-core]%s: HW PLATFORM hw_power error=%d\n",
                __func__, retval);
            ts->poweron = false;
            goto cyttsp_press_power_on_exit;
        }
    }

    // INT
    gpio_direction_input(CY_PR_I2C_IRQ_GPIO);
    mdelay(1);

    if (!is_use_i2c) {
        gpio_direction_input(SCL);
        mdelay(1);
        gpio_direction_input(SDA);
        mdelay(1);
    }

    retval = cyttsp_press_startup(ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: touch press startup error=%d\n",
            __func__, retval);
        ts->poweron = false;
        goto cyttsp_press_power_on_exit;
    }
    msleep(1);

cyttsp_press_power_on_exit :
    if (ts->poweron) {
        ts->irq_enabled = true;
        ts->tch_enabled = true;
    } else {
        cyttsp_press_disable_irq(ts);
        ts->tch_enabled = false;
    }

    mutex_unlock(&ts->data_lock);
    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_POWER_OFF
===========================================================================*/
static int cyttsp_press_power_off(struct cyttsp *ts)
{
    int retval = 0;

    cyttsp_press_disable_irq(ts);

    /* Communicate with the IC */
    mutex_lock(&ts->data_lock);
    ts->tch_enabled = false;

    if(!is_use_i2c) {
        gpio_direction_output(SCL, 0);
        mdelay(1);
        gpio_direction_output(SDA, 0);
        mdelay(1);
    }

    // ResetON
    cyttsp_press_cmd_reset(CY_ON);

    // INT
    gpio_direction_output(CY_PR_I2C_IRQ_GPIO, 0);
    mdelay(1);

    if (ts->platform_data->hw_power) {
        retval = ts->platform_data->hw_power(CY_OFF);
        if (retval < 0) {
            pr_err("[PPD-core]%s: HW PLATFORM hw_power error=%d\n",
                __func__, retval);
        }
    }

    ts->poweron = false;
    ts->power_state = CY_INVALID_STATE;
    mutex_unlock(&ts->data_lock);
    return retval;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
/*===========================================================================
    FUNCTION  CYTTSP_PRESS_RESUME
===========================================================================*/
int cyttsp_press_resume(void *handle)
{
    int retval = 0;
    struct cyttsp *ts = handle;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: Resuming...", __func__);

    // Power On
    retval = cyttsp_press_power_on(ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: touch press power on failed ret=%d\n",
            __func__, retval);
        return retval;
    }

    return retval;
}
EXPORT_SYMBOL_GPL(cyttsp_press_resume);

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_SUSPEND
===========================================================================*/
int cyttsp_press_suspend(void *handle)
{
    struct cyttsp *ts = handle;
    int retval = 0;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: Suspending...", __func__);

    if (!ts->poweron) {
        pr_info("[PPD-core]%s: Touch Press Already PowerOff\n", __func__);
        return retval;
    }

    if (ts->waiting_for_fw) {
        retval = -EBUSY;
        pr_err("[PPD-core]%s: Suspend blocked - Loader busy.\n", __func__);
        return retval;
    }

    // Power Off
    retval = cyttsp_press_power_off(ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: touch press power off failed ret=%d\n",
            __func__, retval);
    }
    return retval;
}
EXPORT_SYMBOL_GPL(cyttsp_press_suspend);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
/*===========================================================================
    FUNCTION  CYTTSP_PRESS_EARLY_SUSPEND
===========================================================================*/
void cyttsp_press_early_suspend(struct early_suspend *h)
{
    struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);
    int retval = 0;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: EARLY SUSPEND ts=%p\n", __func__, ts);
    retval = cyttsp_press_suspend(ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: Early suspend failed with error code %d\n",
            __func__, retval);
    }
    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: EARLY SUSPEND EXIT ts=%p\n", __func__, ts);
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_LATE_RESUME
===========================================================================*/
void cyttsp_press_late_resume(struct early_suspend *h)
{
    struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);
    int retval = 0;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: LATE RESUME ts=%p\n", __func__, ts);
    retval = cyttsp_press_resume(ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: Late resume failed with error code %d\n",
            __func__, retval);
    }
    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: LATE RESUME EXIT ts=%p\n", __func__, ts);
}
#endif

/*===========================================================================
    FUNCTION   CYTTSP_PRESS_CMD_RESET
===========================================================================*/
static void cyttsp_press_cmd_reset(int on)
{
    gpio_set_value(CY_PR_I2C_XRST_GPIO, on);
    mdelay(1);
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_LOAD_OPERATIONAL_REGS
===========================================================================*/
static int cyttsp_press_load_operational_regs(struct cyttsp *ts, int count)
{
    int retval = 0;
    int num = 1;
    unsigned int total_filtere_draw = 0;
    unsigned short filteredraw = 0;
    int failcnt = 0;

    while (num <= count) {
        /* Read Operational Reg */
        retval = cyttsp_read_operational_reg_sequence(ts);
        if (retval < 0) {
            pr_err("[PPD-core]%s: Read Block fail (retval=%d)\n", __func__, retval);
            return retval;
        }

        if (ts->bus_ops->tsdebug > CY_DBG_LVL_1) {
            cyttsp_press_dbg(ts, CY_DBG_LVL_2,
                "%s(%d): %s", __func__, num, cyttsp_press_reginfo_string(ts, CY_OP_REGS));
        } else {
            cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                "[PPD-core]%s(%d): "
                "%02X,"
                "%02X,"
                "%02X,"
                "%02X,%02X,"
                "%02X,%02X,"
                "%02X,%02X,"
                "%02X,%02X,"
                "%02X,%02X,"
                "%02X,%02X,"
                "%02X,"
                "%02X,"
                "%02X,"
                "%02X,"
                "%02X,"
                "%02X,%02X,"
                "%02X,"
                "%02X,"
                "%02X,"
                "%02X,"
                "%02X,%02X,"
                "%02X,"
                "%02X,"
                "%02X,"
                "%02X\n",
                __func__, num,
                ts->press_data.hst_mode,
                ts->press_data.tt_mode,
                ts->press_data.tt_stat,
                ts->press_data.fraw.msb, ts->press_data.fraw.lsb,
                ts->press_data.base.msb, ts->press_data.base.lsb,
                ts->press_data.tt_reserved1[0], ts->press_data.tt_reserved1[1],
                ts->press_data.diff.msb, ts->press_data.diff.lsb,
                ts->press_data.raw.msb, ts->press_data.raw.lsb,
                ts->press_data.tt_undef1[0], ts->press_data.tt_undef1[1],
                ts->press_data.idac,
                ts->press_data.baseline_reset,
                ts->press_data.calibration,
                ts->press_data.prescaler,
                ts->press_data.idac_gain,
                ts->press_data.tt_undef2[0], ts->press_data.tt_undef2[1],
                ts->press_data.act_int_intrvl,
                ts->press_data.lp_int_intrvl,
                ts->press_data.tt_undef3,
                ts->press_data.imo_tr,
                ts->press_data.fw_ver, ts->press_data.fw_rev,
                ts->press_data.alive,
                ts->press_data.lcd_vendor_id,
                ts->press_data.debug_uart,
                ts->press_data.tt_reserved2);

            cyttsp_press_dbg(ts, CY_DBG_LVL_1,
                "[PPD-core]%s(%d): FRAW=0x%02X%02X BASE=0x%02X%02X DIFF=0x%02X%02X RAW=0x%02X%02X IDAC=0x%02X\n",
                __func__, num,
                ts->press_data.fraw.msb,
                ts->press_data.fraw.lsb,
                ts->press_data.base.msb,
                ts->press_data.base.lsb,
                ts->press_data.diff.msb,
                ts->press_data.diff.lsb,
                ts->press_data.raw.msb,
                ts->press_data.raw.lsb,
                ts->press_data.idac);
        }

        // get FilteredRaw
        filteredraw = ts->press_data.fraw.msb << CY_RAW_SHIT_BIT_NUM | ts->press_data.fraw.lsb;

        // Retry 5 times
        if (0 == filteredraw) {
            failcnt ++;
            if (failcnt < CY_NUM_RETRY) {
                cyttsp_press_dbg(ts, CY_DBG_LVL_3,
                    "[PPD-core]%s(%d): Get value failed to be continue, Filtered Raw=0x%04X(%d) (failcnt=%d)\n", 
                    __func__, num, filteredraw, filteredraw, failcnt);
                continue;
            } else {
                pr_err("[PPD-core]%s(%d): Get value failed, Filtered Raw=0x%04X(%d) (failcnt=%d)\n", 
                    __func__, num, filteredraw, filteredraw, failcnt);
                return -EIO;
            }
        }

        failcnt = 0;
        total_filtere_draw += filteredraw;
        num++;

        // default: 10ms wait
        msleep(ts->sleep_intrvl[READ_PRESS_INTRVL]);
    }

    // set FiltedRaw averge
    filteredraw = total_filtere_draw / count;
    ts->press_data.fraw.msb = (filteredraw & CY_RAW_MSB_MASK) >> CY_RAW_SHIT_BIT_NUM;
    ts->press_data.fraw.lsb = filteredraw & CY_RAW_LSB_MASK;

    cyttsp_press_dbg(ts, CY_DBG_LVL_1,
        "[PPD-core]%s: FRAW(averge)=0x%02X%02X BASE=0x%02X%02X DIFF=0x%02X%02X RAW=0x%02X%02X IDAC=0x%02X\n",
        __func__,
        ts->press_data.fraw.msb,
        ts->press_data.fraw.lsb,
        ts->press_data.base.msb,
        ts->press_data.base.lsb,
        ts->press_data.diff.msb,
        ts->press_data.diff.lsb,
        ts->press_data.raw.msb,
        ts->press_data.raw.lsb,
        ts->press_data.idac);

    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_MANUAL_CALIBRATION
===========================================================================*/
static int cyttsp_press_manual_calibration(struct cyttsp *ts)
{
    int retval = 0;
    int count = 0;
    u8 set_cmd = CY_REG_VAL_MIN;
    u8 rdata = CY_REG_VAL_MIN;
    unsigned short fw_ver = ts->sysinfo_data.app_verh << CY_RAW_SHIT_BIT_NUM | ts->sysinfo_data.app_verl;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: IN  fw_ver=0x%04X\n", __func__, fw_ver);
    /* write IDAC GAIN */
    do {
        if (count >= ts->calib.retry_count) {
            pr_err("[PPD-core]%s: Manual Calibration Count Over\n", __func__);
            return -EIO;
        }

        // write 0xC0 to IDAC(0x0F)
        set_cmd = CY_REG_VAL_TARGET_IDAC;
        retval = cyttsp_press_write_block_data(ts, CY_REG_ADR_RIDAC, sizeof(set_cmd), &set_cmd);
        if (retval < 0) {
            pr_err("[PPD-core]%s: write IDAC fail (ret=%d)\n", __func__, retval);
        }
        // default: 1ms wait
        msleep(1);

        // write 0x02 to GAIN(0x13)
        set_cmd = CY_REG_VAL_GAIN_X4;
        retval = cyttsp_press_write_block_data(ts, CY_REG_ADR_RGAIN, sizeof(set_cmd), &set_cmd);
        if (retval < 0) {
            pr_err("[PPD-core]%s: write GAIN fail (ret=%d)\n", __func__, retval);
        }
        // default: 1ms wait
        msleep(1);

        if (fw_ver < CY_FW_VERSION_V00R09) {
            // write 0x01 to address 0x11
            set_cmd = CY_REG_VAL_MANUAL_CALIB;
        } else {
            // write 0x03 to address 0x11
            set_cmd = CY_REG_VAL_MANUAL_CALIB_APPLY;
        }
        retval = cyttsp_press_write_block_data(ts, CY_REG_ADR_CALIB, sizeof(set_cmd), &set_cmd);
        if (retval < 0) {
            pr_err("[PPD-core]%s: write Manual Calibration fail (ret=%d)\n", __func__, retval);
        }

        // default: 50ms wait
        msleep(ts->sleep_intrvl[TO_MANUAL_CALIB]);

        // read data from address 0x11
        retval = cyttsp_press_read_block_data(ts, CY_REG_ADR_CALIB, sizeof(rdata), &rdata);
        cyttsp_press_dbg(ts, CY_DBG_LVL_1, "[PPD-core]%s: Calibration Data Read 0x%02X\n", __func__, rdata);
        count++;

    } while(CY_REG_VAL_MANUAL_CALIB_OK != (rdata & CY_REG_VAL_MANUAL_CALIB_OK));

    // default: 50ms wait
    msleep(ts->sleep_intrvl[TO_MANUAL_CALIB]);

    /* Read FilteredRaw Value */
    retval = cyttsp_press_load_operational_regs(ts, ts->press_value_count);
    if (retval < 0) {
        pr_err("[PPD-core]Read Previous FilteredRaw Value error:%d\n",retval);
        return -EIO;
    }

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: OUT\n", __func__);
    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_AUTOMATIC_CALIBRATION
===========================================================================*/
static int cyttsp_press_automatic_calibration(struct cyttsp *ts)
{
    int retval = 0;
    int count = 0;
    u8 set_cmd = CY_REG_VAL_MIN;
    u8 rdata = CY_REG_VAL_MIN;
    struct cyttsp_calib autoCalib;
    unsigned short fw_ver = ts->sysinfo_data.app_verh << CY_RAW_SHIT_BIT_NUM | ts->sysinfo_data.app_verl;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: IN  fw_ver=0x%04X\n", __func__, fw_ver);

    if (fw_ver < CY_FW_VERSION_V00R09) {
        do {
            if (count >= ts->calib.retry_count) {
                pr_err("[PPD-core]%s: Automatic Calibration Count Over\n", __func__);
                return -EIO;
            }

            // write 0x09 to address 0x11
            set_cmd = CY_REG_VAL_AUTO_CALIB;
            retval = cyttsp_press_write_block_data(ts, CY_REG_ADR_CALIB, sizeof(set_cmd), &set_cmd);
            if (retval < 0) {
                pr_err("[PPD-core]%s: write Automatic Calibration fail (ret=%d)\n", __func__, retval);
            }

            // default: 30ms wait
            msleep(ts->sleep_intrvl[TO_AUTO_CALIB]);

            // read data from address 0x11
            retval = cyttsp_press_read_block_data(ts, CY_REG_ADR_CALIB, sizeof(rdata), &rdata);
            cyttsp_press_dbg(ts, CY_DBG_LVL_1, "[PPD-core]%s: Calibration Data Read 0x%02X\n", __func__, rdata);
            count ++;
        } while(CY_REG_VAL_AUTO_CALIB_OK != (rdata & CY_REG_VAL_AUTO_CALIB_OK));
    } else {
        for (count = 0; count < 2; count++) {
            memcpy(&autoCalib, &(ts->calib), sizeof(struct cyttsp_calib));
            // Set Calibration State
            autoCalib.calib_state = CY_CALIB_START;

            // Set Wait Count
            autoCalib.wait_count = 1;
            autoCalib.total_count = 1;
            autoCalib.calib_pre_wait_cnt = CY_CALIB_PRE_WAIT_CNT;
            autoCalib.calib_end_wait_cnt = CY_CALIB_END_WAIT_CNT / 2;
            autoCalib.calib_base_check_wait_cnt = CY_CALIB_BASE_CHECK_WAIT_CNT;

            while (1) {
                msleep(CY_INTRVL_INT_20MS);

                retval = cyttsp_press_do_calibration(ts, &autoCalib, false);
                if (retval) {
                    pr_err("[PPD-core]%s: Automatic Calibration fail (ret=%d)\n", __func__, retval);
                    return -EIO;
                }

                // Automatic Calibration Successed
                if (autoCalib.calib_state == CY_CALIB_APPLY_END) {
                    break;
                }

                // Automatic Calibration Failed
                if (autoCalib.calib_state == CY_CALIB_BASE_CHECK) {
                    pr_err("[PPD-core]%s: Automatic Calibration State Error\n", __func__);
                    return -EIO;
                }
            }
            msleep(80);
        }
        msleep(320);
    }

    /* Read FilteredRaw Value */
    retval = cyttsp_press_load_operational_regs(ts, ts->press_value_count);
    if (retval < 0) {
        pr_err("[PPD-core]%s: Read Calibrated FilteredRaw Value error:%d\n", __func__, retval);
        return -EIO;
    }

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: OUT\n", __func__);
    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_CALIBRATION
===========================================================================*/
static int cyttsp_press_calibration(struct cyttsp *ts, char *caldata)
{
    int retval = 0;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: IN\n", __func__);

    /* Read Manual Calibration FilteredRaw */
    retval = cyttsp_press_manual_calibration(ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: Read Manual Calibration FilteredRaw error:%d\n", __func__, retval);
        return -EIO;
    }

    /* set previous FilteredRaw */
    /* Sensor */
    caldata[0] = ts->press_data.fraw.msb;
    caldata[1] = ts->press_data.fraw.lsb;
    /* Reference Capacitor */
    caldata[5] = CY_REG_VAL_MAX;
    caldata[6] = CY_REG_VAL_MAX;

    /* Read Automatic Calibration FilteredRaw/IDAC */
    retval = cyttsp_press_automatic_calibration(ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: Read Automatic Calibration FilteredRaw/IDAC error:%d\n", __func__, retval);
        return -EIO;
    }

    /* set Calibrated FilteredRaw */
    /* Sensor */
    caldata[2] = ts->press_data.fraw.msb;
    caldata[3] = ts->press_data.fraw.lsb;
    /* Reference Capacitor */
    caldata[7] = CY_REG_VAL_MAX;
    caldata[8] = CY_REG_VAL_MAX;

    /* set IDAC */
    /* Sensor */
    caldata[4] = ts->press_data.idac;
    /* Reference Capacitor */
    caldata[9] = CY_REG_VAL_MAX;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: OUT\n", __func__);
    return 0;
}

#ifdef CY_TOUCH_PRESS_DEBUG_TOOLS
#include "cyttsp_press_fw.h"

/* Bootloader Initiate Bootload */
#define CY_BL_INIT_LOAD         0x38
/* Bootloader Write a Block */
#define CY_BL_WRITE_BLK         0x39
/* Bootloader Terminate Bootload */
#define CY_BL_TERMINATE         0x3B

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_MC_PUTBL
===========================================================================*/
static int cyttsp_press_mc_putbl(struct cyttsp *ts)
{
    int retval = 0;

    memset(&(ts->bl_data), 0, sizeof(struct cyttsp_bootloader_data));

    retval = cyttsp_press_read_block_data(ts, CY_REG_BASE,
        sizeof(ts->bl_data), &(ts->bl_data));

    if (retval < 0) {
        pr_err("[PPD-core]%s: bus fail reading Bootloader regs (ret=%d)\n",
            __func__, retval);
    }

    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_MC_WR_BLK_CHUNKS
===========================================================================*/
static int cyttsp_press_mc_wr_blk_chunks(struct cyttsp *ts, u8 command,
    u8 length, const u8 *values)
{
    int retval = 0;
    int block = 1;
    u8 dataray[256];

    /* first page already includes the bl page offset */
    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, CY_BL_PAGE_SIZE + 1, values);

   values += CY_BL_PAGE_SIZE + 1;
    length -= CY_BL_PAGE_SIZE + 1;

    /* rem blocks require bl page offset stuffing */
    while (length && (block < CY_BL_NUM_PAGES) && !(retval < 0)) {
        udelay(43 * 2);   /* TRM * 2 */
        dataray[0] = CY_BL_PAGE_SIZE * block;
        memcpy(&dataray[1], values, (length >= CY_BL_PAGE_SIZE) ? CY_BL_PAGE_SIZE : length);

        retval = cyttsp_press_write_block_data(ts, CY_REG_BASE,
                        (length >= CY_BL_PAGE_SIZE) ? (CY_BL_PAGE_SIZE + 1) : (length + 1), dataray);

        values += CY_BL_PAGE_SIZE;
        length = (length >= CY_BL_PAGE_SIZE) ? length - CY_BL_PAGE_SIZE : 0;
        block++;
    }

    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_MC_LOAD_FIRMWARE
===========================================================================*/
static int cyttsp_press_mc_load_firmware(struct cyttsp *ts)
{
    int retval = 0;
    int i = 0, tries = 0;
    u8 host_reg;

    pr_info("[PPD-core]%s: load new firmware --- ttsp_ver=0x%02X%02X app_id=0x%02X%02X app_ver=0x%02X%02X cid=0x%02X%02X%02X\n",
        __func__,
        cyttsp_fw_tts_verh, cyttsp_fw_tts_verl,
        cyttsp_fw_app_idh, cyttsp_fw_app_idl,
        cyttsp_fw_app_verh, cyttsp_fw_app_verl,
        cyttsp_fw_cid_0, cyttsp_fw_cid_1, cyttsp_fw_cid_2);

    /* reset TTSP Device back to bootloader mode */
    host_reg = CY_SOFT_RESET_MODE;
    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(host_reg), &host_reg);

    /* wait for TTSP Device to complete reset back to bootloader */
    tries = 0;
    do {
        usleep_range(1000, 1000);
        cyttsp_press_mc_putbl(ts);
    } while (ts->bl_data.bl_status != CY_BL_READY_NO_APP && ts->bl_data.bl_status != CY_BL_READY_APP && tries++ < 100);
    pr_info("%s(RESET - tires:%d): %s", __func__, tries, cyttsp_press_reginfo_string(ts, CY_BOOT_REGS));

    /* download new TTSP Application to the Bootloader */
    if (!(retval < 0)) {
        i = 0;
        /* send bootload initiation command */
        if (cyttsp_fw[i].Command == CY_BL_INIT_LOAD) {
            ts->bl_data.bl_file = 0;
            ts->bl_data.bl_status = 0;
            ts->bl_data.bl_error = 0;
            retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, cyttsp_fw[i].Length, cyttsp_fw[i].Block);

            /* delay to allow bl to get ready for block writes */
            i++;
            tries = 0;
            do {
                msleep(100);
                cyttsp_press_mc_putbl(ts);
            } while (ts->bl_data.bl_status != CY_BL_READY_NO_APP && ts->bl_data.bl_status != CY_BL_READY_APP && tries++ < 100);
            pr_info("%s(INIT - tires:%d): %s", __func__, tries, cyttsp_press_reginfo_string(ts, CY_BOOT_REGS));

            /* send bootload firmware load blocks */
            if (!(retval < 0)) {
                while (cyttsp_fw[i].Command == CY_BL_WRITE_BLK) {
                    retval = cyttsp_press_mc_wr_blk_chunks(ts, CY_REG_BASE, cyttsp_fw[i].Length, cyttsp_fw[i].Block);

                    pr_info("[PPD-core]%s: Bootloader Download Record=%3d Length=%3d Address=%04X\n",
                        __func__,
                        cyttsp_fw[i].Record,
                        cyttsp_fw[i].Length,
                        cyttsp_fw[i].Address);

                    i++;
                    if (retval < 0) {
                        pr_err("[PPD-core]%s: Bootloader Download Fail Record=%3d retval=%d\n",
                            __func__, cyttsp_fw[i-1].Record, retval);
                        break;
                    } else {
                        tries = 0;
                        cyttsp_press_mc_putbl(ts);

                        while (
                            !((ts->bl_data.bl_status == CY_BL_READY_NO_APP) &&
                            (ts->bl_data.bl_error == CY_BL_RUNNING)) &&
                            !((ts->bl_data.bl_status == CY_BL_READY_APP) &&
                            (ts->bl_data.bl_error == CY_BL_RUNNING)) &&
                            (tries++ < 100)) {
                            usleep_range(1000, 1000);
                            cyttsp_press_mc_putbl(ts);
                        }
                    }
                }

                if (!(retval < 0)) {
                    while (i < cyttsp_fw_records) {
                        retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, cyttsp_fw[i].Length, cyttsp_fw[i].Block);

                        i++;
                        tries = 0;
                        do {
                            msleep(100);
                            cyttsp_press_mc_putbl(ts);
                        } while (ts->bl_data.bl_status != CY_BL_READY_NO_APP && ts->bl_data.bl_status != CY_BL_READY_APP && tries++ < 100);
                        pr_info("%s(TERM- tires:%d): %s", __func__, tries, cyttsp_press_reginfo_string(ts, CY_BOOT_REGS));

                        if (retval < 0) {
                            break;
                        }
                    }
                }
            }
        }
    }

    /* reset TTSP Device back to bootloader mode */
    host_reg = CY_SOFT_RESET_MODE;
    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(host_reg), &host_reg);

    /* wait for TTSP Device to complete reset back to bootloader */
    tries = 0;
    do {
        usleep_range(1000, 1000);
        cyttsp_press_mc_putbl(ts);
    } while (ts->bl_data.bl_status != CY_BL_READY_NO_APP && ts->bl_data.bl_status != CY_BL_READY_APP && tries++ < 100);
    pr_info("%s(FINISHED - tires:%d): %s", __func__, tries, cyttsp_press_reginfo_string(ts, CY_BOOT_REGS));

    msleep(1000);
    return retval;
}
#else
/*===========================================================================
    FUNCTION  CYTTSP_PRESS_MC_LOAD_FIRMWARE
===========================================================================*/
static int cyttsp_press_mc_load_firmware(struct cyttsp *ts)
{
    pr_info("[PPD-core]%s: no-load new firmware\n", __func__);
    return 0;
}
#endif

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_IOCTL_DEEPSLEEP
===========================================================================*/
static int cyttsp_press_ioctl_deepsleep(struct cyttsp *ts)
{
    int retval = 0;
    u8 sleep_mode = CY_REG_VAL_MIN;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: ioctl deep sleep...", __func__);

    if (!ts->poweron) {
        retval = -EFAULT;
        pr_err("[PPD-core]%s: Already PowerOff\n", __func__);
        return retval;
    }

    if (ts->power_state == CY_SLEEP_STATE) {
        pr_info("[PPD-core]%s: Already deep sleep\n", __func__);
        return retval;
    }

    if (ts->waiting_for_fw) {
        retval = -EBUSY;
        pr_err("[PPD-core]%s: DeepSleep blocked - Loader busy.\n", __func__);
        return retval;
    }

    cyttsp_press_disable_irq(ts);
    mutex_lock(&ts->data_lock);

    sleep_mode = CY_DEEP_SLEEP_MODE | CY_LOW_POWER_MODE;
    retval = cyttsp_press_write_block_data(ts,
        CY_REG_BASE, sizeof(sleep_mode), &sleep_mode);

    // default: 60ms wait
    msleep(60);
    if (retval < 0) {
        cyttsp_press_enable_irq(ts);
    } else {
        cyttsp_press_touch_liftoff(ts);
        ts->power_state = CY_SLEEP_STATE;
    }

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: Deep Sleep %s\n", __func__,
        (retval < 0) ? "FAIL" : "SUCCESS");
    cyttsp_press_pr_state(ts);

    mutex_unlock(&ts->data_lock);
    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_IOCTL_WAKEUP
===========================================================================*/
static int cyttsp_press_ioctl_wakeup(struct cyttsp *ts)
{
    int retval = 0;
    u8 cmd = CY_OPERATE_MODE;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: ioctl wake up...", __func__);

    if (!ts->poweron) {
        retval = -EFAULT;
        pr_err("[PPD-core]%s: Already PowerOff\n", __func__);
        return retval;
    }

    if (ts->power_state != CY_SLEEP_STATE) {
        pr_info("[PPD-core]%s: Already wake up (state = %s)\n", __func__,
            cyttsp_press_ps_string(ts));
        return retval;
    }

    mutex_lock(&ts->data_lock);

    // Deep Sleep
    if (ts->power_state == CY_SLEEP_STATE) {
        // release from DeepSleep
        cmd = CY_REG_VAL_MIN;
        retval = cyttsp_press_write_block_data(ts, CY_REG_ADR_MAX, sizeof(cmd), &cmd);
        if (retval < 0) {
            pr_err("[PPD-core]%s: write fail release from DeepSleep (ret=%d)\n",
                __func__, retval);
            cyttsp_press_recovery(ts);
            goto cyttsp_press_ioctl_wakeup_error;
        }
        // default: 40ms wait
        msleep(40);
    }

    // switch to Operational Mode
    cmd = CY_OPERATE_MODE;
    retval = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
    if (retval < 0) {
        pr_err("[PPD-core]%s: write fail set Operational Mode (ret=%d)\n",
            __func__, retval);
        cyttsp_press_recovery(ts);
        goto cyttsp_press_ioctl_wakeup_error;
    }
    // default: 40ms wait
    msleep(40);
    ts->power_state = CY_ACTIVE_STATE;
    cyttsp_press_touch_liftoff(ts);
    ts->tch_sts = TOUCH_DOWN;
    cyttsp_press_enable_irq(ts);

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: Wake Up %s\n", __func__,
        (retval < 0) ? "FAIL" : "SUCCESS");
    goto cyttsp_press_ioctl_wakeup_no_error;

cyttsp_press_ioctl_wakeup_error:
    cyttsp_press_touch_liftoff(ts);

cyttsp_press_ioctl_wakeup_no_error :
    mutex_unlock(&ts->data_lock);
    cyttsp_press_pr_state(ts);
    return retval;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_IOCTL
===========================================================================*/
static long cyttsp_press_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    enum devicepowerstatus mode;
    enum touchstatus touchmode;
    struct cyttsp *ts;
    char devdata[CY_DEVICEINFO_SIZE];
    char caldata[CY_CALIBRATION_SIZE];
    struct press_value press_value;
    u8 hst_mode;
    unsigned short fw_ver = 0;

    if (file == NULL || file->private_data == NULL) {
        return -EFAULT;
    }
    ts = file->private_data;

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: IN PowerState=%s\n", __func__, ts->poweron ? "On" : "Off");

    switch (cmd) {
    case PRESS_SET_TOUCHSTATUS:
        // touch press invalid
        if (ts->tch_enabled == false) {
            cyttsp_press_dbg(ts, CY_DBG_LVL_2, "[PPD-core]%s: Touch Press Disable\n", __func__);
            break;
        }

        ret = copy_from_user(&touchmode, (int __user *)arg, sizeof(int));
        if (ret != 0) {
            pr_err("[PPD-core]%s: copy touchmode error. (ret=%d)\n", __func__, ret);
            ret = -EFAULT;
            break;
        }

        mutex_lock(&ts->data_lock);
        cyttsp_press_touch_liftoff(ts);

        switch (touchmode) {
        /* TOUCH_DOWN */
        case TOUCH_DOWN:
            cyttsp_press_dbg(ts, CY_DBG_LVL_1, "[PPD-core]%s: TOUCH DOWN\n", __func__);
            // read HST_MODE
            ret = cyttsp_press_read_block_data(ts, CY_REG_BASE, sizeof(hst_mode), &hst_mode);
            if (ret < 0) {
                pr_err("[PPD-core]%s: Read Block fail. (ret=%d)\n", __func__, ret);
                cyttsp_press_recovery(ts);
                ret = -EIO;
                break;
            }

            // Low Power Mode -> Active mode
            if (IS_LOW_POWER(hst_mode) == CY_LOW_POWER_MODE) {
                cyttsp_press_dbg(ts, CY_DBG_LVL_2, "[PPD-core]%s: Low Power\n", __func__);

                cmd = CY_OPERATE_MODE;
                // switch to Operational mode
                ret = cyttsp_press_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
                if (ret < 0) {
                    pr_err("[PPD-core]%s: set Operational mode error. (ret=%d)\n", __func__, ret);
                    cyttsp_press_recovery(ts);
                    ret = -EIO;
                    break;
                }
            }

            ts->power_state = CY_ACTIVE_STATE;
            ts->tch_sts = TOUCH_DOWN;
            prev_filteredraw = 0;
            pretch_sts = CY_ON;
            break;

        /* LIFT_OFF */
        case LIFT_OFF:
            cyttsp_press_dbg(ts, CY_DBG_LVL_1, "[PPD-core]%s: LIFT OFF\n", __func__);
            break;

        default:
            break;
        }
        mutex_unlock(&ts->data_lock);
        break;

    case PRESS_GET_FW_VERSION:
        pr_info("[PPD-core]%s: Enter PRESS_GET_FW_VERSION\n", __func__);
        fw_ver = ts->sysinfo_data.app_verh << CY_RAW_SHIT_BIT_NUM | ts->sysinfo_data.app_verl;

        ret = copy_to_user((int __user *)arg, &fw_ver, sizeof(fw_ver));
        if (ret != 0) {
            pr_err("[PPD-core]%s: copy_to_user error. (ret=%d)\n", __func__, ret);
            ret = -EFAULT;
        }
        break;

    case PRESS_SET_POWERMODE:
        pr_info("[PPD-core]%s: Enter PRESS_SET_POWERMODE\n", __func__);

        ret = copy_from_user(&mode, (int __user *)arg, sizeof(int));
        if (ret != 0) {
            pr_err("[PPD-core]%s: copy_from_user error (ret=%d)\n", __func__, ret);
            ret = -EFAULT;
        }
        pr_info("[PPD-core]%s: POWER MODE (mode = %d)\n", __func__, mode);

        switch (mode) {
        /* DEEP_SLEEP */
        case DEEP_SLEEP:
            pr_info("[PPD-core]%s: Enter DEEP_SLEEP\n", __func__);
            ret = cyttsp_press_ioctl_deepsleep(ts);
            if (ret < 0) {
                pr_err("[PPD-core]%s: set Deep Sleep Error. (ret=%d)\n", __func__, ret);
                ret = -EFAULT;
            }
            break;

        /* WAKE_UP */
        case WAKE_UP:
            pr_info("[PPD-core]%s: Enter WAKE_UP\n", __func__);
            ret = cyttsp_press_ioctl_wakeup(ts);
            if (ret < 0) {
                pr_err("[PPD-core]%s: set Wake Up Error. (ret=%d)\n", __func__, ret);
                ret = -EFAULT;
            }
            break;

        /* RESET_ON */
        case RESET_ON:
            pr_info("[PPD-core]%s: Enter RESET_ON\n", __func__);
            // PowerOff
            if (!ts->poweron) {
                ret = -EFAULT;
                pr_err("[PPD-core]%s: Already PowerOff (ret=%d)\n", __func__, ret);
            // PowerOn
            } else {
                ts->tch_enabled = false;
                cyttsp_press_disable_irq(ts);
                cyttsp_press_cmd_reset(CY_ON);
            }
            break;

        /* RESET_OFF */
        case RESET_OFF:
            pr_info("[PPD-core]%s: Enter RESET_OFF\n", __func__);
            // PowerOff
            if (!ts->poweron) {
                ret = -EFAULT;
                pr_err("[PPD-core]%s: Already PowerOff (ret=%d)\n", __func__, ret);
            // PowerOn
            } else {
                // HW Reset & Restart
                ts->power_state = CY_INVALID_STATE;
                cyttsp_press_enable_irq(ts);

                cyttsp_press_startup(ts);
                ts->tch_enabled = true;
            }
            break;
        }
        break;

    case PRESS_FIRMWARE_DOWNLOAD:
        pr_info("[PPD-core]%s: Enter PRESS_FIRMWARE_DOWNLOAD\n", __func__);
        if (ts->waiting_for_fw) {
            pr_err("[PPD-core]%s: Driver is already waiting for load firmware\n", __func__);
            ret = -EALREADY;
            break;
        }

        // WakeUp
        ts->tch_enabled = false;
        ret = cyttsp_press_ioctl_wakeup(ts);
        if (ret < 0) {
            pr_err("[PPD-core]%s: Cannot WakeUp. (ret=%d)\n", __func__, ret);
            ts->tch_enabled = true;
            ts->waiting_for_fw = false;
            ret = -EFAULT;
            break;
        }

        cyttsp_press_disable_irq(ts);
        mutex_lock(&ts->data_lock);
        ts->waiting_for_fw = true;

        /* FW Loading... */
        ret = cyttsp_press_mc_load_firmware(ts);
        if (ret < 0) {
            pr_err("[PPD-core]%s: Cannot Load Firmware. (ret=%d)\n", __func__, ret);
            ret = -EFAULT;
        } else {
            pr_info("[PPD-core]%s: Load Firmware OK.\n", __func__);
        }

        // restart
        ts->power_state = CY_INVALID_STATE;
        cyttsp_press_enable_irq(ts);

        cyttsp_press_startup(ts);
        ts->tch_enabled = true;
        ts->waiting_for_fw = false;
        mutex_unlock(&ts->data_lock);
        break;

    case PRESS_GET_DEVICE_INFORMATION:
        pr_info("[PPD-core]%s: Enter PRESS_GET_DEVICE_INFORMATION\n", __func__);
        // WakeUp
        ret = cyttsp_press_ioctl_wakeup(ts);
        if (ret < 0) {
            pr_err("[PPD-core]%s: Can not WakeUp. (ret=%d)\n", __func__, ret);
            ret = -EFAULT;
            break;
        }

        devdata[0] = ts->sysinfo_data.cid[0];
        devdata[1] = ts->sysinfo_data.cid[1];
        devdata[2] = ts->sysinfo_data.cid[2];
        devdata[3] = ts->sysinfo_data.tt_undef1;
        devdata[4] = ts->sysinfo_data.uid[0];
        devdata[5] = ts->sysinfo_data.uid[1];
        devdata[6] = ts->sysinfo_data.uid[2];
        devdata[7] = ts->sysinfo_data.uid[3];
        devdata[8] = ts->sysinfo_data.uid[4];
        devdata[9] = ts->sysinfo_data.uid[5];
        devdata[10] = ts->sysinfo_data.uid[6];
        devdata[11] = ts->sysinfo_data.uid[7];
        devdata[12] = ts->sysinfo_data.bl_verh;
        devdata[13] = ts->sysinfo_data.bl_verl;
        devdata[14] = ts->sysinfo_data.tts_verh;
        devdata[15] = ts->sysinfo_data.tts_verl;
        devdata[16] = ts->sysinfo_data.app_idh;
        devdata[17] = ts->sysinfo_data.app_idl;
        devdata[18] = ts->sysinfo_data.app_verh;
        devdata[19] = ts->sysinfo_data.app_verl;

        ret = copy_to_user((int __user *)arg, &devdata, sizeof(devdata));
        if (ret != 0) {
            pr_err("[PPD-core]%s: copy_to_user error. (ret=%d)\n", __func__, ret);
            ret = -EFAULT;
        }
        break;

    case PRESS_DO_CALIBRATION:
        pr_info("[PPD-core]%s: Enter CALIBRATION\n", __func__);
        ts->tch_enabled = false;
        // WakeUp
        ret = cyttsp_press_ioctl_wakeup(ts);
        if (ret < 0) {
            pr_err("[PPD-core]%s: Can not WakeUp. (ret=%d)\n", __func__, ret);
            ret = -EFAULT;
            goto PRESS_DO_CALIBRATION_ERR;
        }

        /* do calibration */
        ret = cyttsp_press_calibration(ts, caldata);
        if (ret < 0) {
            pr_err("[PPD-core]%s: do calibration error. (ret=%d)\n", __func__, ret);
            ret = -EIO;
            goto PRESS_DO_CALIBRATION_ERR;
        }

        ret = copy_to_user((int __user *)arg, &caldata, sizeof(caldata));
        if (ret != 0) {
            pr_err("[PPD-core]%s: copy_to_user error. (ret=%d)\n", __func__, ret);
            ret = -EFAULT;
        }
        break;

PRESS_DO_CALIBRATION_ERR :
        ts->tch_enabled = true;
        break;

    case PRESS_GET_PRESS_VALUE:
        pr_info("[PPD-core]%s: Enter PRESSVALUE\n", __func__);
        ts->tch_enabled = false;
        // WakeUp
        ret = cyttsp_press_ioctl_wakeup(ts);
        if (ret < 0) {
            pr_err("[PPD-core]%s: Can not WakeUp. (ret=%d)\n", __func__, ret);
            ret = -EFAULT;
            goto PRESS_GET_PRESS_VALUE_ERR;
        }

        /* Read Press Value */
        ret = cyttsp_press_load_operational_regs(ts, ts->press_value_count);
        if (ret < 0) {
            pr_err("[PPD-core]%s: Read Press Value error. (ret=%d)\n", __func__, ret);
            ret = -EIO;
            goto PRESS_GET_PRESS_VALUE_ERR;
        }

        /* Set Press Value */
        memset(&press_value, 0, sizeof(struct press_value));
        press_value.filteredraw = ts->press_data.fraw.msb << CY_RAW_SHIT_BIT_NUM | ts->press_data.fraw.lsb;
        press_value.raw = ts->press_data.raw.msb << CY_RAW_SHIT_BIT_NUM | ts->press_data.raw.lsb;
        press_value.base = ts->press_data.base.msb << CY_RAW_SHIT_BIT_NUM | ts->press_data.base.lsb;

        ret = copy_to_user((int __user *)arg, &press_value, sizeof(press_value));
        if (ret != 0) {
            pr_err("[PPD-core]%s: copy_to_user error. (ret=%d)\n", __func__, ret);
            ret = -EFAULT;
        }
        break;

PRESS_GET_PRESS_VALUE_ERR :
        ts->tch_enabled = true;
        break;

    default:
        break;
    }
    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: OUT (ret=%d)\n", __func__, ret);
    return (ret < 0) ? ret : 0;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_IOCTL_OPEN
===========================================================================*/
static int cyttsp_press_ioctl_open(struct inode *inode, struct file *file)
{
    struct cyttsp *ts = container_of(inode->i_cdev, struct cyttsp, cyttsp_cdev);

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: IN\n", __func__);
    file->private_data = ts;
    return 0;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_IOCTL_CLOSE
===========================================================================*/
static int cyttsp_press_ioctl_close(struct inode *inode, struct file *file)
{
    struct cyttsp *ts = container_of(inode->i_cdev, struct cyttsp, cyttsp_cdev);

    cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: IN\n", __func__);
    return 0;
}

static struct file_operations cyttsp_fops = {
    .owner          = THIS_MODULE,
    .open           = cyttsp_press_ioctl_open,
    .release        = cyttsp_press_ioctl_close,
    .unlocked_ioctl = cyttsp_press_ioctl,
};

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_CORE_RELEASE
===========================================================================*/
void cyttsp_press_core_release(void *handle)
{
    struct cyttsp *ts = handle;

    if (ts) {
#ifdef CONFIG_HAS_EARLYSUSPEND
        unregister_early_suspend(&ts->early_suspend);
#endif
        cyttsp_press_ldr_free(ts);
        mutex_destroy(&ts->data_lock);
        free_irq(ts->irq, ts);
        input_unregister_device(ts->input);
        if (ts->platform_data->init) {
            ts->platform_data->init(CY_OFF);
        }
        device_destroy(udev_class, devno);
        class_destroy(udev_class);
        kfree(ts->fwname);
        kfree(ts);
    }
}
EXPORT_SYMBOL_GPL(cyttsp_press_core_release);

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_OPEN
===========================================================================*/
static int cyttsp_press_open(struct input_dev *dev)
{
    return 0;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_CLOSE
===========================================================================*/
static void cyttsp_press_close(struct input_dev *dev)
{
    /*
     * close() normally powers down the device
     * this call simply returns unless power
     * to the device can be controlled by the driver
     */
    return;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_CORE_INIT
===========================================================================*/
void *cyttsp_press_core_init(struct cyttsp_bus_ops *bus_ops,
    struct device *dev, int irq, char *name)
{
    int i = 0;
    int signal = 0;
    int retval = 0;
    dev_t dev_ioctl;
    struct input_dev *input_device;
    struct cyttsp *ts = kzalloc(sizeof(*ts), GFP_KERNEL);

    if (!ts) {
        pr_err("[PPD-core]%s: Error, kzalloc\n", __func__);
        goto error_alloc_data;
    }

    ts->fwname = kzalloc(CY_BL_FW_NAME_SIZE, GFP_KERNEL);
    if (!ts->fwname || (dev == NULL) || (bus_ops == NULL)) {
        pr_err("[PPD-core]%s: Error, dev, bus_ops, or fwname null\n",
            __func__);
        kfree(ts);
        goto error_alloc_data;
    }

    ts->waiting_for_fw = false;
    ts->irq_enabled = false;

    // Calibration Info
    ts->calib.retry_count = CY_CALI_WAIT_MAX_CNT;
    ts->calib.calib_pre_wait_cnt = CY_CALIB_PRE_WAIT_CNT;
    ts->calib.calib_start_wait_cnt = CY_CALIB_START_WAIT_CNT;
    ts->calib.calib_end_wait_cnt = CY_CALIB_END_WAIT_CNT;
    ts->calib.calib_apply_start_wait_cnt = CY_CALIB_APPLY_START_WAIT_CNT;
    ts->calib.calib_base_check_wait_cnt = CY_CALIB_BASE_CHECK_WAIT_CNT;

    // Loop Count
    ts->press_value_count = CY_PRESS_VALUE_READ_CNT;
    ts->invalid_buf_read_count = CY_INVALID_BUFFER_READ_CNT;

    // Sleep Interval
    ts->sleep_intrvl[READ_PRESS_INTRVL] = CY_INTRVL_READ_PRESS;
    ts->sleep_intrvl[TO_MANUAL_CALIB] = CY_INTRVL_TO_MANU_CALIB;
    ts->sleep_intrvl[TO_AUTO_CALIB] = CY_INTRVL_TO_AUTO_CALIB;

#ifdef CONFIG_TOUCHPRESS_DEBUG
    ts->drv_attr_mode = CY_DRV_BASE_INFO;
    ts->rw_regid = 0;
#endif

    mutex_init(&ts->data_lock);
    ts->dev = dev;
    ts->platform_data = dev->platform_data;
    ts->bus_ops = bus_ops;
    ts->bus_ops->tsdebug = CY_DBG_LVL_0;

    init_completion(&ts->bl_int_running);
    init_completion(&ts->si_int_running);
    ts->flags = ts->platform_data->flags;

    dev_ioctl = MKDEV(cdev_major, 0);
    udev_class = class_create(THIS_MODULE, "cyttsp_press");

    retval = alloc_chrdev_region(&dev_ioctl, 0, 1, "cyttsp_press");
    cdev_major = MAJOR(dev_ioctl);
    if (cdev_major == 0) {
        cdev_major = retval;
    }

    devno = MKDEV(cdev_major, 0);
    cdev_init(&(ts->cyttsp_cdev), &cyttsp_fops);
    ts->cyttsp_cdev.owner = THIS_MODULE;
    ts->cyttsp_cdev.ops = &cyttsp_fops;
    retval = cdev_add (&(ts->cyttsp_cdev), devno, 1);

    device_create(udev_class, NULL, devno, NULL, "cyttsp_press");

    ts->irq = irq;
    if (ts->irq <= 0) {
        pr_err("[PPD-core]%s: Error, failed to allocate irq\n", __func__);
        goto error_init;
    }

    /* Create the input device and register it. */
    input_device = input_allocate_device();
    if (!input_device) {
        pr_err("[PPD-core]%s: Error, failed to allocate input device\n",
            __func__);
        goto error_input_allocate_device;
    }

    ts->input = input_device;
    input_device->name = name;
    snprintf(ts->phys, sizeof(ts->phys), "%s", dev_name(dev));
    input_device->phys = ts->phys;
    input_device->dev.parent = ts->dev;
    ts->bus_type = bus_ops->dev->bus;
    input_device->open = cyttsp_press_open;
    input_device->close = cyttsp_press_close;
    input_set_drvdata(input_device, ts);
    dev_set_drvdata(dev, ts);

    pretch_sts = CY_OFF;
    __set_bit(EV_ABS, input_device->evbit);
    __set_bit(INPUT_PROP_DIRECT, input_device->evbit);

    for (i = 0; i < CY_CNT_ABS_SET; i++) {
        signal = ts->platform_data->frmwrk->abs[(i * CY_NUM_ABS_SET) + CY_SIGNAL_OST];
        if (signal) {
            input_set_abs_params(input_device,
                signal,
                ts->platform_data->frmwrk->abs[(i * CY_NUM_ABS_SET) + CY_MIN_OST],
                ts->platform_data->frmwrk->abs[(i * CY_NUM_ABS_SET) + CY_MAX_OST],
                ts->platform_data->frmwrk->abs[(i * CY_NUM_ABS_SET) + CY_FUZZ_OST],
                ts->platform_data->frmwrk->abs[(i * CY_NUM_ABS_SET) + CY_FLAT_OST]);
        }
    }

    if (ts->platform_data->frmwrk->enable_vkeys) {
        input_set_capability(input_device, EV_KEY, KEY_PROG1);
    }

    if (input_register_device(input_device)) {
        pr_err("[PPD-core]%s: Error, failed to register input device\n",
            __func__);
        goto error_input_register_device;
    }

/* FUJITSU:2012-03-15 CYPRESS TOUCH PRESS add start In Future To Be Deleted */
    pr_info("[***PPD-core start***(146/144)]\n");
    // GPIO CFG
    gpio_tlmm_config(GPIO_CFG(146, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(144, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

    // Request
    gpio_request(146, "gpio_146");
    gpio_request(144, "gpio_144");

    gpio_direction_input(146);
    gpio_direction_input(144);
    msleep(10);
    pr_info("[***PPD-core end***(146/144)]\n");
/* FUJITSU:2012-03-15 CYPRESS TOUCH PRESS add end In Future To Be Deleted */

    if (ts->platform_data->init) {
        retval = ts->platform_data->init(CY_ON);
        if (retval < 0) {
            pr_err("[PPD-core]%s: HW PLATFORM INIT error=%d\n",
                __func__, retval);
            goto error_input_register_device;
        }
    }

    /* enable interrupts */
    retval = request_threaded_irq(ts->irq, NULL, cyttsp_press_irq,
        IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
        ts->input->name, ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: IRQ request failed ret=%d\n",
            __func__, retval);
        ts->irq_enabled = false;
        goto error_platform_data_init;
    } else {
        ts->irq_enabled = true;
    }

    if (!is_use_i2c) {
        cyttsp_press_dbg(ts, CY_DBG_LVL_3, "[PPD-core]%s: SCL=%d, SDA=%d\n", __func__, SCL, SDA);
        // Request
        gpio_request(SCL, "cyttsp_press_i2c_scl");
        gpio_request(SDA, "cyttsp_press_i2c_sda");

        // GPIO CFG
        gpio_tlmm_config(GPIO_CFG(SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        gpio_tlmm_config(GPIO_CFG(SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    }

    retval = cyttsp_press_power_on(ts);
    if (retval < 0) {
        pr_err("[PPD-core]%s: touch press power on failed ret=%d\n",
            __func__, retval);
        goto error_power_on;
    }

    /* Add /sys files */
    cyttsp_press_ldr_init(ts);

#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = cyttsp_press_early_suspend;
    ts->early_suspend.resume = cyttsp_press_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

    goto no_error;

error_power_on :
    free_irq(ts->irq, ts);
error_platform_data_init :
    if (ts->platform_data->init) {
        ts->platform_data->init(CY_OFF);
    }
error_input_register_device:
    input_free_device(input_device);
error_input_allocate_device:
error_init:
    mutex_destroy(&ts->data_lock);
    device_destroy(udev_class, devno);
    class_destroy(udev_class);
    kfree(ts->fwname);
    kfree(ts);
error_alloc_data:
    pr_err("[PPD-core]%s: Failed Initialization\n", __func__);
    ts = NULL;
no_error:
    return ts;
}
EXPORT_SYMBOL_GPL(cyttsp_press_core_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Touch Press driver core");
MODULE_AUTHOR("Cypress");
