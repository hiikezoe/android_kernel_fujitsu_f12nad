/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/

/*
 * this needs to be before <linux/kernel.h> is loaded,
 * and <linux/sched.h> loads <linux/kernel.h>
 */
#define DEBUG  1

#include <linux/slab.h>
#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <asm/atomic.h>

#include <mach/msm_rpcrouter.h>
#include <mach/msm_battery.h>

/* FUJITSU:2011-12-01 add Battery start */
#if (defined(CONFIG_FUJITSU_MSM_TEMP) || defined(CONFIG_FUJITSU_RVOLTAGE))
#include "../arch/arm/mach-msm/smd_private.h"
#endif
/* FUJITSU:2011-12-01 add Battery end */

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VER_1_1	0x00010001
#define BATTERY_RPC_VER_2_1	0x00020001
#define BATTERY_RPC_VER_4_1     0x00040001
#define BATTERY_RPC_VER_5_1     0x00050001

#define BATTERY_RPC_CB_PROG	(BATTERY_RPC_PROG | 0x01000000)

#define CHG_RPC_PROG		0x3000001a
#define CHG_RPC_VER_1_1		0x00010001
#define CHG_RPC_VER_1_3		0x00010003
#define CHG_RPC_VER_2_2		0x00020002
#define CHG_RPC_VER_3_1         0x00030001
#define CHG_RPC_VER_4_1         0x00040001

#define BATTERY_REGISTER_PROC				2
#define BATTERY_MODIFY_CLIENT_PROC			4
#define BATTERY_DEREGISTER_CLIENT_PROC			5
#define BATTERY_READ_MV_PROC				12
#define BATTERY_ENABLE_DISABLE_FILTER_PROC		14

#define VBATT_FILTER			2

#define BATTERY_CB_TYPE_PROC		1
#define BATTERY_CB_ID_ALL_ACTIV		1
#define BATTERY_CB_ID_LOW_VOL		2

#define BATTERY_LOW		3200
#define BATTERY_HIGH		4300

#define ONCRPC_CHG_GET_GENERAL_STATUS_PROC	12
#define ONCRPC_CHARGER_API_VERSIONS_PROC	0xffffffff

#define BATT_RPC_TIMEOUT    5000	/* 5 sec */

#define INVALID_BATT_HANDLE    -1

#define RPC_TYPE_REQ     0
#define RPC_TYPE_REPLY   1
#define RPC_REQ_REPLY_COMMON_HEADER_SIZE   (3 * sizeof(uint32_t))

/* FUJITSU:2011-12-01 add Battery start */
#define BATT_UPDATE_TIMEOUT	(2000)
/* FUJITSU:2011-12-01 add Battery end */

#if DEBUG
#define DBG_LIMIT(x...) do {if (printk_ratelimit()) pr_debug(x); } while (0)
#else
#define DBG_LIMIT(x...) do {} while (0)
#endif

/* FUJITSU:2011-12-01 add Battery start */
static DEFINE_MUTEX(msm_mutex);
static struct workqueue_struct *msm_battery_workqueue;
static void get_battery_voltage(struct work_struct *work);
static DECLARE_DELAYED_WORK(timer_work, get_battery_voltage);
static struct wake_lock batt_wakelock;
extern int msm_hsusb_chg_status(void);
typedef void (*msm_batt_cbfunc)(void);
/* FUJITSU:2011-12-01 add Battery End */

enum {
	BATTERY_REGISTRATION_SUCCESSFUL = 0,
	BATTERY_DEREGISTRATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_MODIFICATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_INTERROGATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_CLIENT_TABLE_FULL = 1,
	BATTERY_REG_PARAMS_WRONG = 2,
	BATTERY_DEREGISTRATION_FAILED = 4,
	BATTERY_MODIFICATION_FAILED = 8,
	BATTERY_INTERROGATION_FAILED = 16,
	/* Client's filter could not be set because perhaps it does not exist */
	BATTERY_SET_FILTER_FAILED         = 32,
	/* Client's could not be found for enabling or disabling the individual
	 * client */
	BATTERY_ENABLE_DISABLE_INDIVIDUAL_CLIENT_FAILED  = 64,
	BATTERY_LAST_ERROR = 128,
};

enum {
	BATTERY_VOLTAGE_UP = 0,
	BATTERY_VOLTAGE_DOWN,
	BATTERY_VOLTAGE_ABOVE_THIS_LEVEL,
	BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
	BATTERY_VOLTAGE_LEVEL,
	BATTERY_ALL_ACTIVITY,
	VBATT_CHG_EVENTS,
	BATTERY_VOLTAGE_UNKNOWN,
};

/*
 * This enum contains defintions of the charger hardware status
 */
enum chg_charger_status_type {
	/* The charger is good      */
	CHARGER_STATUS_GOOD,
	/* The charger is bad       */
	CHARGER_STATUS_BAD,
	/* The charger is weak      */
	CHARGER_STATUS_WEAK,
	/* Invalid charger status.  */
	CHARGER_STATUS_INVALID
};

/*
 *This enum contains defintions of the charger hardware type
 */
enum chg_charger_hardware_type {
	/* The charger is removed                 */
	CHARGER_TYPE_NONE,
	/* The charger is a regular wall charger   */
	CHARGER_TYPE_WALL,
	/* The charger is a PC USB                 */
	CHARGER_TYPE_USB_PC,
	/* The charger is a wall USB charger       */
	CHARGER_TYPE_USB_WALL,
	/* The charger is a USB carkit             */
	CHARGER_TYPE_USB_CARKIT,

/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */
	CHARGER_TYPE_WIRELESS,
	CHARGER_TYPE_HOLDER,
	CHARGER_TYPE_MHL,
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now end */

	/* Invalid charger hardware status.        */
	CHARGER_TYPE_INVALID
};

/*
 *  This enum contains defintions of the battery status
 */
enum chg_battery_status_type {
	/* The battery is good        */
	BATTERY_STATUS_GOOD,
	/* The battery is cold/hot    */
	BATTERY_STATUS_BAD_TEMP,
	/* The battery is bad         */
	BATTERY_STATUS_BAD,
	/* The battery is removed     */
	BATTERY_STATUS_REMOVED,		/* on v2.2 only */
	BATTERY_STATUS_INVALID_v1 = BATTERY_STATUS_REMOVED,
	/* Invalid battery status.    */
	BATTERY_STATUS_INVALID
};

/*
 *This enum contains defintions of the battery voltage level
 */
enum chg_battery_level_type {
	/* The battery voltage is dead/very low (less than 3.2V) */
	BATTERY_LEVEL_DEAD,
	/* The battery voltage is weak/low (between 3.2V and 3.4V) */
	BATTERY_LEVEL_WEAK,
	/* The battery voltage is good/normal(between 3.4V and 4.2V) */
	BATTERY_LEVEL_GOOD,
	/* The battery voltage is up to full (close to 4.2V) */
	BATTERY_LEVEL_FULL,
	/* Invalid battery voltage level. */
	BATTERY_LEVEL_INVALID
};

#ifndef CONFIG_BATTERY_MSM_FAKE
struct rpc_reply_batt_chg_v1 {
	struct rpc_reply_hdr hdr;
	u32 	more_data;

	u32	charger_status;
	u32	charger_type;
	u32	battery_status;
	u32	battery_level;
	u32     battery_voltage;
	u32	battery_temp;
};

struct rpc_reply_batt_chg_v2 {
	struct rpc_reply_batt_chg_v1	v1;

	u32	is_charger_valid;
	u32	is_charging;
	u32	is_battery_valid;
	u32	ui_event;
};

union rpc_reply_batt_chg {
	struct rpc_reply_batt_chg_v1	v1;
	struct rpc_reply_batt_chg_v2	v2;
};

static union rpc_reply_batt_chg rep_batt_chg;
#endif

struct msm_battery_info {
	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 chg_api_version;
	u32 batt_technology;
	u32 batt_api_version;

	u32 avail_chg_sources;
	u32 current_chg_source;

	u32 batt_status;
	u32 batt_health;
	u32 charger_valid;
	u32 batt_valid;
	u32 batt_capacity; /* in percentage */

	u32 charger_status;
	u32 charger_type;
	u32 battery_status;
	u32 battery_level;
	u32 battery_voltage; /* in millie volts */
/* FUJITSU:2012-03-30 add Battery start */
#ifdef CONFIG_FUJITSU_RVOLTAGE
	u32 battery_rvoltage;
#endif //CONFIG_FUJITSU_RVOLTAGE
/* FUJITSU:2012-03-30 add Battery end */
	u32 battery_temp;  /* in celsius */
/* FUJITSU:2011-12-01 add Battery start */
#ifdef CONFIG_FUJITSU_MSM_TEMP
	u32 msm_temp;
#endif //CONFIG_FUJITSU_MSM_TEMP
/* FUJITSU:2011-12-01 add Battery end */
/* FUJITSU:2012-05-23 add temperature start */
#ifdef CONFIG_FUJITSU_CHARGER_12_1
	u32 ambient_temp;
	u32 case_temp;
#endif //CONFIG_FUJITSU_CHARGER_12_1
/* FUJITSU:2012-05-23 add temperature end */
	u32(*calculate_capacity) (u32 voltage);

	s32 batt_handle;

	struct power_supply *msm_psy_ac;
	struct power_supply *msm_psy_usb;

/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */
#ifdef CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS
	struct power_supply *msm_psy_wireless;
#endif //CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_HOLDER
	struct power_supply *msm_psy_holder;
#endif

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_MHL
	struct power_supply *msm_psy_mhl;
#endif
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now end */

	struct power_supply *msm_psy_batt;
	struct power_supply *current_ps;

	struct msm_rpc_client *batt_client;
	struct msm_rpc_endpoint *chg_ep;

	wait_queue_head_t wait_q;

	u32 vbatt_modify_reply_avail;

	struct early_suspend early_suspend;
};

static struct msm_battery_info msm_batt_info = {
	.batt_handle = INVALID_BATT_HANDLE,
	.charger_status = CHARGER_STATUS_BAD,
	.charger_type = CHARGER_TYPE_INVALID,
	.battery_status = BATTERY_STATUS_GOOD,
	.battery_level = BATTERY_LEVEL_FULL,
	.battery_voltage = BATTERY_HIGH,
/* FUJITSU:2012-03-30 add Battery start */
#ifdef CONFIG_FUJITSU_RVOLTAGE
	.battery_rvoltage = BATTERY_HIGH,
#endif //CONFIG_FUJITSU_RVOLTAGE
/* FUJITSU:2012-03-30 add Battery end */
	.batt_capacity = 100,
	.batt_status = POWER_SUPPLY_STATUS_DISCHARGING,
	.batt_health = POWER_SUPPLY_HEALTH_GOOD,
	.batt_valid  = 1,
	.battery_temp = 23,
	.vbatt_modify_reply_avail = 0,
/* FUJITSU:2011-12-01 add Battery start */
#ifdef CONFIG_FUJITSU_MSM_TEMP
/* FUJITSU:2012-05-23 mod temperature start */
	.msm_temp = 186,
/* FUJITSU:2012-05-23 mod temperature end */
#endif //CONFIG_FUJITSU_MSM_TEMP
/* FUJITSU:2011-12-01 add Battery end */
/* FUJITSU:2012-05-23 add temperature start */
#ifdef CONFIG_FUJITSU_CHARGER_12_1
	.ambient_temp = 0,
	.case_temp = 0
#endif //CONFIG_FUJITSU_CHARGER_12_1
/* FUJITSU:2012-05-23 add temperature end */
};


/* FUJITSU:2011-12-01 add Battery start */
#ifdef CONFIG_FUJITSU_BATTERY_MSM_CB
#include <linux/msm_battery_cb.h>

static struct msm_battery_callback_info *cb_info = NULL;
static unsigned int old_batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
static DEFINE_MUTEX(msm_battery_mutex_cb);
/* FUJITSU:2012-05-28 add temperature log start */
static unsigned int check_erapsed_time = 9999999;
/* FUJITSU:2012-05-28 add temperature log end */

int set_msm_battery_callback_info(struct msm_battery_callback_info *info)
{
    if (info != NULL && info->callback != NULL){
        cb_info = info;
        return 0;
    }else{
        return -1;
    }
}
EXPORT_SYMBOL(set_msm_battery_callback_info);

int unset_msm_battery_callback_info(void)
{
    if (cb_info == NULL){
        return -1;
    }else{
        cb_info = NULL;
        return 0;
    }
}
EXPORT_SYMBOL(unset_msm_battery_callback_info);
#endif

static void batt_status_cb_work(void)
{

#ifdef CONFIG_FUJITSU_BATTERY_MSM_CB
    if (old_batt_status != msm_batt_info.batt_status){
        old_batt_status = msm_batt_info.batt_status;
        DBG_LIMIT("BATT: CB batt_status=%d\n", msm_batt_info.batt_status);
        if (cb_info != NULL && cb_info->callback != NULL){
            mutex_lock(&msm_battery_mutex_cb);
            cb_info->callback(msm_batt_info.batt_status, cb_info->data);
            mutex_unlock(&msm_battery_mutex_cb);
            DBG_LIMIT("BATT: CB exec & out\n");
        }
    }
#endif

}
/* FUJITSU:2011-12-01 add Battery end */

/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */
void msm_battery_health(int health)
{
#ifdef CONFIG_FUJITSU_CHARGER_12_1
	msm_batt_info.batt_health = health;
#endif
}
EXPORT_SYMBOL(msm_battery_health);

/* FUJITSU:2012-05-23 mod temperature start */
#ifdef CONFIG_FUJITSU_MSM_TEMP
#ifdef CONFIG_FUJITSU_CHARGER_12_1
static int adc_to_temp(unsigned int msm_temp)
{
	int ret_temp;

	const signed char tbl_adc_to_temp[242] = {
	/*   0 */ -40,
	/*   1 */ -40,
	/*   2 */ -40,
	/*   3 */ -40,
	/*   4 */ -40,
	/*   5 */ -40,
	/*   6 */ -40,
	/*   7 */ -40,
	/*   8 */ -40,
	/*   9 */ -40,
	/*  10 */ -40,
	/*  11 */ -40,
	/*  12 */ 125,
	/*  13 */ 123,
	/*  14 */ 120,
	/*  15 */ 117,
	/*  16 */ 114,
	/*  17 */ 111,
	/*  18 */ 109,
	/*  19 */ 107,
	/*  20 */ 105,
	/*  21 */ 102,
	/*  22 */ 101,
	/*  23 */  99,
	/*  24 */  97,
	/*  25 */  95,
	/*  26 */  94,
	/*  27 */  92,
	/*  28 */  91,
	/*  29 */  89,
	/*  30 */  88,
	/*  31 */  86,
	/*  32 */  85,
	/*  33 */  84,
	/*  34 */  83,
	/*  35 */  81,
	/*  36 */  80,
	/*  37 */  79,
	/*  38 */  78,
	/*  39 */  77,
	/*  40 */  76,
	/*  41 */  75,
	/*  42 */  74,
	/*  43 */  73,
	/*  44 */  72,
	/*  45 */  71,
	/*  46 */  70,
	/*  47 */  69,
	/*  48 */  68,
	/*  49 */  68,
	/*  50 */  67,
	/*  51 */  66,
	/*  52 */  65,
	/*  53 */  64,
	/*  54 */  63,
	/*  55 */  63,
	/*  56 */  62,
	/*  57 */  61,
	/*  58 */  60,
	/*  59 */  60,
	/*  60 */  59,
	/*  61 */  58,
	/*  62 */  58,
	/*  63 */  57,
	/*  64 */  56,
	/*  65 */  56,
	/*  66 */  55,
	/*  67 */  54,
	/*  68 */  54,
	/*  69 */  53,
	/*  70 */  53,
	/*  71 */  52,
	/*  72 */  51,
	/*  73 */  51,
	/*  74 */  50,
	/*  75 */  50,
	/*  76 */  49,
	/*  77 */  48,
	/*  78 */  48,
	/*  79 */  47,
	/*  80 */  47,
	/*  81 */  46,
	/*  82 */  46,
	/*  83 */  45,
	/*  84 */  45,
	/*  85 */  44,
	/*  86 */  44,
	/*  87 */  43,
	/*  88 */  42,
	/*  89 */  42,
	/*  90 */  41,
	/*  91 */  41,
	/*  92 */  40,
	/*  93 */  40,
	/*  94 */  39,
	/*  95 */  39,
	/*  96 */  39,
	/*  97 */  38,
	/*  98 */  38,
	/*  99 */  37,
	/* 100 */  37,
	/* 101 */  36,
	/* 102 */  36,
	/* 103 */  35,
	/* 104 */  35,
	/* 105 */  34,
	/* 106 */  34,
	/* 107 */  33,
	/* 108 */  33,
	/* 109 */  32,
	/* 110 */  32,
	/* 111 */  32,
	/* 112 */  31,
	/* 113 */  31,
	/* 114 */  30,
	/* 115 */  30,
	/* 116 */  29,
	/* 117 */  29,
	/* 118 */  29,
	/* 119 */  28,
	/* 120 */  28,
	/* 121 */  27,
	/* 122 */  27,
	/* 123 */  26,
	/* 124 */  26,
	/* 125 */  26,
	/* 126 */  25,
	/* 127 */  25,
	/* 128 */  24,
	/* 129 */  24,
	/* 130 */  23,
	/* 131 */  23,
	/* 132 */  23,
	/* 133 */  22,
	/* 134 */  22,
	/* 135 */  21,
	/* 136 */  21,
	/* 137 */  21,
	/* 138 */  20,
	/* 139 */  20,
	/* 140 */  19,
	/* 141 */  19,
	/* 142 */  18,
	/* 143 */  18,
	/* 144 */  18,
	/* 145 */  17,
	/* 146 */  17,
	/* 147 */  16,
	/* 148 */  16,
	/* 149 */  16,
	/* 150 */  15,
	/* 151 */  15,
	/* 152 */  14,
	/* 153 */  14,
	/* 154 */  14,
	/* 155 */  13,
	/* 156 */  13,
	/* 157 */  12,
	/* 158 */  12,
	/* 159 */  11,
	/* 160 */  11,
	/* 161 */  11,
	/* 162 */  10,
	/* 163 */  10,
	/* 164 */   9,
	/* 165 */   9,
	/* 166 */   9,
	/* 167 */   8,
	/* 168 */   8,
	/* 169 */   7,
	/* 170 */   7,
	/* 171 */   6,
	/* 172 */   6,
	/* 173 */   6,
	/* 174 */   5,
	/* 175 */   5,
	/* 176 */   4,
	/* 177 */   4,
	/* 178 */   3,
	/* 179 */   3,
	/* 180 */   2,
	/* 181 */   2,
	/* 182 */   1,
	/* 183 */   1,
	/* 184 */   1,
	/* 185 */   0,
	/* 186 */   0,
	/* 187 */  -1,
	/* 188 */  -1,
	/* 189 */  -2,
	/* 190 */  -2,
	/* 191 */  -3,
	/* 192 */  -3,
	/* 193 */  -4,
	/* 194 */  -4,
	/* 195 */  -5,
	/* 196 */  -5,
	/* 197 */  -6,
	/* 198 */  -6,
	/* 199 */  -7,
	/* 200 */  -7,
	/* 201 */  -8,
	/* 202 */  -8,
	/* 203 */  -9,
	/* 204 */ -10,
	/* 205 */ -10,
	/* 206 */ -11,
	/* 207 */ -11,
	/* 208 */ -12,
	/* 209 */ -13,
	/* 210 */ -13,
	/* 211 */ -14,
	/* 212 */ -14,
	/* 213 */ -15,
	/* 214 */ -16,
	/* 215 */ -16,
	/* 216 */ -17,
	/* 217 */ -18,
	/* 218 */ -18,
	/* 219 */ -19,
	/* 220 */ -20,
	/* 221 */ -21,
	/* 222 */ -21,
	/* 223 */ -22,
	/* 224 */ -23,
	/* 225 */ -24,
	/* 226 */ -25,
	/* 227 */ -25,
	/* 228 */ -26,
	/* 229 */ -27,
	/* 230 */ -28,
	/* 231 */ -29,
	/* 232 */ -30,
	/* 233 */ -31,
	/* 234 */ -32,
	/* 235 */ -33,
	/* 236 */ -34,
	/* 237 */ -35,
	/* 238 */ -37,
	/* 239 */ -38,
	/* 240 */ -39,
	/* 241 */ -40,
	};

	if (msm_temp >= 12 && msm_temp <= 241){
		ret_temp =  (int)(tbl_adc_to_temp[msm_temp]);
		return ret_temp;
	}else{
		return -40;
	}
}
#endif
#endif
/* FUJITSU:2012-05-23 mod temperature end */
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now end */

static enum power_supply_property msm_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_power_supplied_to[] = {
	"battery",
};

static int msm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = msm_batt_info.current_chg_source & AC_CHG
			    ? 1 : 0;
		}
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = msm_batt_info.current_chg_source & USB_CHG
			    ? 1 : 0;
		}
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */
#ifdef CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS
		if (psy->type == POWER_SUPPLY_TYPE_WIRELESS) {
			val->intval = msm_batt_info.current_chg_source & WIRELESS_CHG
			    ? 1 : 0;
		}
#endif //CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_HOLDER
		if (psy->type == POWER_SUPPLY_TYPE_HOLDER) {
			val->intval = msm_batt_info.current_chg_source & HOLDER_CHG
			    ? 1 : 0;
		}
#endif

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_MHL
		if (psy->type == POWER_SUPPLY_TYPE_MHL) {
			val->intval = msm_batt_info.current_chg_source & MHL_CHG
			    ? 1 : 0;
		}
#endif
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now end */
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static struct power_supply msm_psy_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */
#ifdef CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS
static struct power_supply msm_psy_wireless = {
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};
#endif //CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_HOLDER
static struct power_supply msm_psy_holder = {
	.name = "holder",
	.type = POWER_SUPPLY_TYPE_HOLDER,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};
#endif

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_MHL
static struct power_supply msm_psy_mhl = {
	.name = "mhl",
	.type = POWER_SUPPLY_TYPE_MHL,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};
#endif 
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now end */


static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
/* FUJITSU:2011-12-01 add Battery start */
	POWER_SUPPLY_PROP_TEMP,
#ifdef CONFIG_FUJITSU_MSM_TEMP
	POWER_SUPPLY_PROP_TEMP_MSM,
#endif //CONFIG_FUJITSU_MSM_TEMP
/* FUJITSU:2012-05-23 add temperature start */
#ifdef CONFIG_FUJITSU_CHARGER_12_1
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
	POWER_SUPPLY_PROP_TEMP_CASE,
#endif //CONFIG_FUJITSU_CHARGER_12_1
/* FUJITSU:2012-05-23 add temperature end */
#ifdef CONFIG_FUJITSU_RVOLTAGE
	POWER_SUPPLY_PROP_RVOLTAGE_NOW,
#endif //CONFIG_FUJITSU_RVOLTAGE

/* FUJITSU:2011-12-01 add Battery end */
};

static int msm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = msm_batt_info.batt_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = msm_batt_info.batt_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = msm_batt_info.batt_valid;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = msm_batt_info.batt_technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_batt_info.voltage_max_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_batt_info.voltage_min_design;
		break;

/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */
#if (defined(CONFIG_FUJITSU_CHARGER_12_1) && defined(CONFIG_FUJITSU_RVOLTAGE))
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = msm_batt_info.battery_rvoltage;
		break;
#else
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = msm_batt_info.battery_voltage;
		break;
#endif
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now end */
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = msm_batt_info.batt_capacity;
		break;
/* FUJITSU:2011-12-01 add Battery start */
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = msm_batt_info.battery_temp*10;
		break;

#ifdef CONFIG_FUJITSU_MSM_TEMP
	case POWER_SUPPLY_PROP_TEMP_MSM:
		val->intval = msm_batt_info.msm_temp;
		break;
#endif //CONFIG_FUJITSU_MSM_TEMP
/* FUJITSU:2012-05-23 add temperature start */
#ifdef CONFIG_FUJITSU_CHARGER_12_1
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
		val->intval = msm_batt_info.ambient_temp*10;
		break;
	case POWER_SUPPLY_PROP_TEMP_CASE:
		val->intval = msm_batt_info.case_temp*10;
		break;
#endif //CONFIG_FUJITSU_CHARGER_12_1
/* FUJITSU:2012-05-23 add temperature end */
#ifdef CONFIG_FUJITSU_RVOLTAGE
	case POWER_SUPPLY_PROP_RVOLTAGE_NOW:
		val->intval = msm_batt_info.battery_rvoltage;
		break;
#endif //CONFIG_FUJITSU_MSM_TEMP

/* FUJITSU:2011-12-01 add Battery end */
	default:
		return -EINVAL;
	}
	return 0;
}

/* FUJITSU:2011-12-01 add Battery start */
int msm_battery_get_property(enum power_supply_property psp,
			     union power_supply_propval *val)
{
	return msm_batt_power_get_property(NULL, psp, val);
}
EXPORT_SYMBOL(msm_battery_get_property);
/* FUJITSU:2011-12-01 add Battery end */

static struct power_supply msm_psy_batt = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_batt_power_props,
	.num_properties = ARRAY_SIZE(msm_batt_power_props),
	.get_property = msm_batt_power_get_property,
};

#ifndef CONFIG_BATTERY_MSM_FAKE
struct msm_batt_get_volt_ret_data {
	u32 battery_voltage;
};

static int msm_batt_get_volt_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct msm_batt_get_volt_ret_data *data_ptr, *buf_ptr;

	data_ptr = (struct msm_batt_get_volt_ret_data *)data;
	buf_ptr = (struct msm_batt_get_volt_ret_data *)buf;

	data_ptr->battery_voltage = be32_to_cpu(buf_ptr->battery_voltage);

	return 0;
}

static u32 msm_batt_get_vbatt_voltage(void)
{
	int rc;

	struct msm_batt_get_volt_ret_data rep;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_READ_MV_PROC,
			NULL, NULL,
			msm_batt_get_volt_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt get volt. rc=%d\n", __func__, rc);
		return 0;
	}

	return rep.battery_voltage;
}

/* FUJITSU:2012-05-21 add disable camera key on charge holder start */
#ifdef CONFIG_MACH_F12NAD
int msm_get_current_chg_source(void)
{
	return msm_batt_info.current_chg_source;
}
EXPORT_SYMBOL(msm_get_current_chg_source);
#endif /* CONFIG_MACH_F12NAD */
/* FUJITSU:2012-05-21 add disable camera key on charge holder end */

#define	be32_to_cpu_self(v)	(v = be32_to_cpu(v))

static int msm_batt_get_batt_chg_status(void)
{
	int rc;

	struct rpc_req_batt_chg {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_batt_chg;
	struct rpc_reply_batt_chg_v1 *v1p;

	req_batt_chg.more_data = cpu_to_be32(1);

	memset(&rep_batt_chg, 0, sizeof(rep_batt_chg));

	v1p = &rep_batt_chg.v1;
	rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
				ONCRPC_CHG_GET_GENERAL_STATUS_PROC,
				&req_batt_chg, sizeof(req_batt_chg),
				&rep_batt_chg, sizeof(rep_batt_chg),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		pr_err("%s: ERROR. msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, ONCRPC_CHG_GET_GENERAL_STATUS_PROC, rc);
		return rc;
	} else if (be32_to_cpu(v1p->more_data)) {
		be32_to_cpu_self(v1p->charger_status);
		be32_to_cpu_self(v1p->charger_type);
		be32_to_cpu_self(v1p->battery_status);
		be32_to_cpu_self(v1p->battery_level);
		be32_to_cpu_self(v1p->battery_voltage);
		be32_to_cpu_self(v1p->battery_temp);
	} else {
		pr_err("%s: No battery/charger data in RPC reply\n", __func__);
		return -EIO;
	}

	return 0;
}

static void msm_batt_update_psy_status(void)
{
	static u32 unnecessary_event_count;
	u32	charger_status;
	u32	charger_type;
	u32	battery_status;
	u32	battery_level;
	u32     battery_voltage;
	u32	battery_temp;
	struct	power_supply	*supp;
/* FUJITSU:2011-12-01 add Battery start */
	int	exec_wakelock = 0;
/* FUJITSU:2011-12-01 add Battery end */

	if (msm_batt_get_batt_chg_status())
		return;

	charger_status = rep_batt_chg.v1.charger_status;
	charger_type = rep_batt_chg.v1.charger_type;
	battery_status = rep_batt_chg.v1.battery_status;
	battery_level = rep_batt_chg.v1.battery_level;
	battery_voltage = rep_batt_chg.v1.battery_voltage;
	battery_temp = rep_batt_chg.v1.battery_temp;

	/* Make correction for battery status */
	if (battery_status == BATTERY_STATUS_INVALID_v1) {
		if (msm_batt_info.chg_api_version < CHG_RPC_VER_3_1){
			battery_status = BATTERY_STATUS_INVALID;
			DBG_LIMIT("BATTSTAT: BATTERY_STATUS_INVALID\n");
                }
	}

	if (charger_status == msm_batt_info.charger_status &&
	    charger_type == msm_batt_info.charger_type &&
	    battery_status == msm_batt_info.battery_status &&
	    battery_level == msm_batt_info.battery_level &&
	    battery_voltage == msm_batt_info.battery_voltage &&
	    battery_temp == msm_batt_info.battery_temp) {
		/* Got unnecessary event from Modem PMIC VBATT driver.
		 * Nothing changed in Battery or charger status.
		 */
		unnecessary_event_count++;
		if ((unnecessary_event_count % 20) == 1)
			DBG_LIMIT("BATT: same event count = %u\n",
				 unnecessary_event_count);
		return;
	}

	unnecessary_event_count = 0;

	DBG_LIMIT("BATT: rcvd: %d, %d, %d, %d; %d, %d\n",
		 charger_status, charger_type, battery_status,
		 battery_level, battery_voltage, battery_temp);

	if (battery_status == BATTERY_STATUS_INVALID &&
	    battery_level != BATTERY_LEVEL_INVALID) {
		DBG_LIMIT("BATT: change status(%d) to (%d) for level=%d\n",
			 battery_status, BATTERY_STATUS_GOOD, battery_level);
		battery_status = BATTERY_STATUS_GOOD;
		DBG_LIMIT("BATTSTAT: BATTERY_STATUS_GOOD\n");
	}

	if (msm_batt_info.charger_type != charger_type) {
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */
		if (charger_type == CHARGER_TYPE_USB_PC || charger_type == CHARGER_TYPE_USB_CARKIT) {
			DBG_LIMIT("BATT: USB charger plugged in\n");
			msm_batt_info.current_chg_source = USB_CHG;
			supp = &msm_psy_usb;

		} else if (charger_type == CHARGER_TYPE_USB_WALL) {
			DBG_LIMIT("BATT: USB_WALL changer plugged in\n");
			msm_batt_info.current_chg_source = AC_CHG;
			supp = &msm_psy_ac;

#ifndef CONFIG_FUJITSU_CHARGER_TYPE_HOLDER
		} else if (charger_type == CHARGER_TYPE_WALL) {
			DBG_LIMIT("BATT: AC Wall changer plugged in\n");
			msm_batt_info.current_chg_source = AC_CHG;
			supp = &msm_psy_ac;
#else
		} else if (charger_type == CHARGER_TYPE_WALL) {
			DBG_LIMIT("BATT: HOLDER changer plugged in\n");
			msm_batt_info.current_chg_source = HOLDER_CHG;
			supp = &msm_psy_holder;
#endif

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS
		} else if (charger_type == CHARGER_TYPE_WIRELESS) {
			DBG_LIMIT("BATT: WIRELESS changer plugged in\n");
			msm_batt_info.current_chg_source = WIRELESS_CHG;
			supp = &msm_psy_wireless;
#endif //CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_MHL
		} else if (charger_type == CHARGER_TYPE_MHL) {
			DBG_LIMIT("BATT: MHL changer plugged in\n");
			msm_batt_info.current_chg_source = MHL_CHG;
			supp = &msm_psy_mhl;
#endif //CONFIG_FUJITSU_CHARGER_TYPE_MHL
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now end */

		} else {
			if (msm_batt_info.current_chg_source & AC_CHG)
				DBG_LIMIT("BATT: AC Wall charger removed\n");
			else if (msm_batt_info.current_chg_source & USB_CHG)
				DBG_LIMIT("BATT: USB charger removed\n");

/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */
#ifdef CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS
			else if (msm_batt_info.current_chg_source & WIRELESS_CHG)
				DBG_LIMIT("BATT: WIRELESS charger removed\n");
#endif //CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_HOLDER
			else if (msm_batt_info.current_chg_source & HOLDER_CHG)
				DBG_LIMIT("BATT: HOLDER charger removed\n");
#endif //CONFIG_FUJITSU_CHARGER_TYPE_HOLDER

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_MHL
			else if (msm_batt_info.current_chg_source & MHL_CHG)
				DBG_LIMIT("BATT: MHL charger removed\n");
#endif //CONFIG_FUJITSU_CHARGER_TYPE_MHL
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */

			else
				DBG_LIMIT("BATT: No charger present\n");
			msm_batt_info.current_chg_source = 0;
			supp = &msm_psy_batt;

			/* Correct charger status */
			if (charger_status != CHARGER_STATUS_INVALID) {
				DBG_LIMIT("BATT: No charging!\n");
				charger_status = CHARGER_STATUS_INVALID;

/* FUJITSU:2012-05-03 mod Battery 12_1 NOT_CHARGING(plug source), DISCHARGING(unplug source) start */
#ifdef  CONFIG_FUJITSU_CHARGER_12_1
				if (msm_batt_info.current_chg_source){
					msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
					DBG_LIMIT("BATTSTAT: CHARGER_STATUS_INVALID POWER_SUPPLY_STATUS_NOT_CHARGING\n");
				}else{
					msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
					DBG_LIMIT("BATTSTAT: CHARGER_STATUS_INVALID POWER_SUPPLY_STATUS_DISCHARGING\n");
				}

#else
				msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
				DBG_LIMIT("BATTSTAT: CHARGER_STATUS_INVALID POWER_SUPPLY_STATUS_NOT_CHARGING\n");
#endif
/* FUJITSU:2012-05-03 mod Battery 12_1 NOT_CHARGING(plug source), DISCHARGING(unplug source) end */
			}
		}
/* FUJITSU:2011-12-01 add Battery start */
		exec_wakelock = 1;
/* FUJITSU:2011-12-01 add Battery end */
	} else
		supp = NULL;

	if (msm_batt_info.charger_status != charger_status) {
		if (charger_status == CHARGER_STATUS_GOOD ||
		    charger_status == CHARGER_STATUS_WEAK) {
			if (msm_batt_info.current_chg_source) {

/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */
				DBG_LIMIT("BATT: Charging %d.\n", msm_batt_info.current_chg_source);
		                DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_CHARGING\n");
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_CHARGING;
				/* Correct when supp==NULL */
				if (msm_batt_info.current_chg_source & AC_CHG)
					supp = &msm_psy_ac;
				else if (msm_batt_info.current_chg_source & USB_CHG)
					supp = &msm_psy_usb;
#ifdef CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS
				else if (msm_batt_info.current_chg_source & WIRELESS_CHG) {
					supp = &msm_psy_wireless;
				}
#endif //CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_HOLDER
				else if (msm_batt_info.current_chg_source & HOLDER_CHG) {
					supp = &msm_psy_holder;
				}
#endif //CONFIG_FUJITSU_CHARGER_TYPE_HOLDER

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_MHL
				else if (msm_batt_info.current_chg_source & MHL_CHG) {
					supp = &msm_psy_mhl;
				}
#endif //CONFIG_FUJITSU_CHARGER_TYPE_MHL
				else{
					supp = &msm_psy_usb;
				}
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now end */

			}
		} else {
			DBG_LIMIT("BATT: No charging.\n");
/* FUJITSU:2012-05-03 mod Battery 12_1 NOT_CHARGING(plug source), DISCHARGING(unplug source) start */
#ifdef  CONFIG_FUJITSU_CHARGER_12_1
                        if (msm_batt_info.current_chg_source){
				msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		                DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_NOT_CHARGING\n");
			}else{
				msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		                DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_DISCHARGING\n");
			}

#else
			msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_NOT_CHARGING\n");
#endif
/* FUJITSU:2012-05-03 mod Battery 12_1 NOT_CHARGING(plug source), DISCHARGING(unplug source) end */


			supp = &msm_psy_batt;
/* FUJITSU:2012-05-03 mod Battery 12_1 NOT_CHARGING(plug source), DISCHARGING(unplug source) end */
		}
/* FUJITSU:2011-12-01 add Battery start */
		exec_wakelock = 1;
/* FUJITSU:2011-12-01 add Battery end */
	} else {
		/* Correct charger status */
		if (charger_type != CHARGER_TYPE_INVALID &&
		    charger_status == CHARGER_STATUS_GOOD) {
			DBG_LIMIT("BATT: In charging\n");
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_CHARGING;
		        DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_CHARGING\n");
		}
	}

	/* Correct battery voltage and status */
	if (!battery_voltage) {
		if (charger_status == CHARGER_STATUS_INVALID) {
			DBG_LIMIT("BATT: Read VBATT\n");
			battery_voltage = msm_batt_get_vbatt_voltage();
		} else
			/* Use previous */
			battery_voltage = msm_batt_info.battery_voltage;
	}
	if (battery_status == BATTERY_STATUS_INVALID) {
		if (battery_voltage >= msm_batt_info.voltage_min_design &&
		    battery_voltage <= msm_batt_info.voltage_max_design) {
			DBG_LIMIT("BATT: Battery valid\n");
			msm_batt_info.batt_valid = 1;
			battery_status = BATTERY_STATUS_GOOD;
		        DBG_LIMIT("BATTSTAT: BATTERY_STATUS_GOOD\n");
		}
	}

	if (msm_batt_info.battery_status != battery_status) {
		if (battery_status != BATTERY_STATUS_INVALID) {
			msm_batt_info.batt_valid = 1;

			if (battery_status == BATTERY_STATUS_BAD) {
				DBG_LIMIT("BATT: Battery bad.\n");
				msm_batt_info.batt_health =
					POWER_SUPPLY_HEALTH_DEAD;
		                DBG_LIMIT("BATTSTAT: POWER_SUPPLY_HEALTH_DEAD\n");
			} else if (battery_status == BATTERY_STATUS_BAD_TEMP) {
/* FUJITSU:2012-5-18 mod Battery start */
#if 0
				DBG_LIMIT("BATT: Battery overheat.\n");
				msm_batt_info.batt_health =
					POWER_SUPPLY_HEALTH_OVERHEAT;
				DBG_LIMIT("BATTSTAT: POWER_SUPPLY_HEALTH_OVERHEAT\n");
#endif
				if (battery_temp <= 0){
					DBG_LIMIT("BATT: Battery cold.\n");
					msm_batt_info.batt_health =
						POWER_SUPPLY_HEALTH_COLD;
					DBG_LIMIT("BATTSTAT: POWER_SUPPLY_HEALTH_COLD\n");
				}else{
					DBG_LIMIT("BATT: Battery overheat.\n");
					msm_batt_info.batt_health =
						POWER_SUPPLY_HEALTH_OVERHEAT;
					DBG_LIMIT("BATTSTAT: POWER_SUPPLY_HEALTH_OVERHEAT\n");
				}
/* FUJITSU:2012-5-18 mod Battery end */
			} else {
				DBG_LIMIT("BATT: Battery good.\n");
				msm_batt_info.batt_health =
					POWER_SUPPLY_HEALTH_GOOD;
		                DBG_LIMIT("BATTSTAT: POWER_SUPPLY_HEALTH_GOOD\n");
			}
		} else {
			msm_batt_info.batt_valid = 0;
			DBG_LIMIT("BATT: Battery invalid.\n");
			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
			DBG_LIMIT("BATTSTAT: POWER_SUPPLY_HEALTH_UNKNOWN\n");
		}

		if (msm_batt_info.batt_status != POWER_SUPPLY_STATUS_CHARGING) {
			if (battery_status == BATTERY_STATUS_INVALID) {
				DBG_LIMIT("BATT: Battery -> unknown\n");
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_UNKNOWN;
		                DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_UNKNOWN\n");
			} else {
				DBG_LIMIT("BATT: Battery -> discharging\n");
/* FUJITSU:2012-05-03 mod Battery 12_1 NOT_CHARGING(plug source), DISCHARGING(unplug source) start */
#ifdef  CONFIG_FUJITSU_CHARGER_12_1
				if (msm_batt_info.current_chg_source){
					msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		                	DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_NOT_CHARGING\n");
				}else{
					msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		                	DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_DISCHARGING\n");
				}
#else
				msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
				DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_DISCHARGING\n");
#endif
/* FUJITSU:2012-05-03 mod Battery 12_1 NOT_CHARGING(plug source), DISCHARGING(unplug source) end */


			}
		}

		if (!supp) {
			if (msm_batt_info.current_chg_source) {
				if (msm_batt_info.current_chg_source & AC_CHG)
					supp = &msm_psy_ac;

/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */
				else if(msm_batt_info.current_chg_source & USB_CHG)
					supp = &msm_psy_usb;

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS
				else if (msm_batt_info.current_chg_source & WIRELESS_CHG)
					supp = &msm_psy_wireless;
#endif

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_HOLDER
				else if (msm_batt_info.current_chg_source & HOLDER_CHG)
					supp = &msm_psy_holder;
#endif

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_MHL
				else if (msm_batt_info.current_chg_source & MHL_CHG)
					supp = &msm_psy_mhl;
#endif
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now end */
				else
					supp = &msm_psy_usb;


			} else
				supp = &msm_psy_batt;
		}
	}

	msm_batt_info.charger_status 	= charger_status;
	msm_batt_info.charger_type 	= charger_type;
	msm_batt_info.battery_status 	= battery_status;
	msm_batt_info.battery_level 	= battery_level;
	msm_batt_info.battery_temp 	= battery_temp;
/* FUJITSU:2012-05-23 mod temperature start */
#ifdef CONFIG_FUJITSU_CHARGER_12_1
#ifndef CONFIG_FUJITSU_MSM_TEMP
	msm_batt_info.ambient_temp = msm_batt_info.battery_temp;
	msm_batt_info.case_temp = msm_batt_info.ambient_temp;
#endif
#endif
/* FUJITSU:2012-05-23 mod temperature end */

/* FUJITSU:2011-12-01 add Battery start */
	batt_status_cb_work();
/* FUJITSU:2011-12-01 add Battery end */


	if (msm_batt_info.battery_voltage != battery_voltage) {
/* FUJITSU:2011-12-01 add Battery start */
		mutex_lock(&msm_mutex);
/* FUJITSU:2011-12-01 add Battery end */
		msm_batt_info.battery_voltage  	= battery_voltage;
		msm_batt_info.batt_capacity =
			msm_batt_info.calculate_capacity(battery_voltage);
		DBG_LIMIT("BATT: voltage = %u mV [capacity = %d%%]\n",
			 battery_voltage, msm_batt_info.batt_capacity);

/* FUJITSU:2011-12-01 add Battery start */
		if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
			if (msm_batt_info.batt_capacity == 100) {
				msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
		                DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_FULL\n");
			}
		}
		mutex_unlock(&msm_mutex);
/* FUJITSU:2011-12-01 add Battery end */

		if (!supp)
			supp = msm_batt_info.current_ps;
	}

	if (supp) {
/* FUJITSU:2011-12-01 add Battery start */
		if(exec_wakelock){
			wake_lock_timeout(&batt_wakelock, 300);
		}
/* FUJITSU:2011-12-01 add Battery end */
		msm_batt_info.current_ps = supp;
		DBG_LIMIT("BATT: Supply = %s\n", supp->name);
/* FUJITSU:2011-12-01 add Battery start */
	        mutex_lock(&msm_mutex);
/* FUJITSU:2011-12-01 add Battery end */
		power_supply_changed(supp);
/* FUJITSU:2011-12-01 add Battery start */
	        mutex_unlock(&msm_mutex);
/* FUJITSU:2011-12-01 add Battery end */
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
struct batt_modify_client_req {

	u32 client_handle;

	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
};

struct batt_modify_client_rep {
	u32 result;
};

static int msm_batt_modify_client_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_modify_client_req *batt_modify_client_req =
		(struct batt_modify_client_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(batt_modify_client_req->client_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->cb_data);
	size += sizeof(u32);

	return size;
}

static int msm_batt_modify_client_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct  batt_modify_client_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_modify_client_rep *)data;
	buf_ptr = (struct batt_modify_client_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);

	return 0;
}

static int msm_batt_modify_client(u32 client_handle, u32 desired_batt_voltage,
	     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	int rc;

	struct batt_modify_client_req  req;
	struct batt_modify_client_rep rep;

	req.client_handle = client_handle;
	req.desired_batt_voltage = desired_batt_voltage;
	req.voltage_direction = voltage_direction;
	req.batt_cb_id = batt_cb_id;
	req.cb_data = cb_data;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_MODIFY_CLIENT_PROC,
			msm_batt_modify_client_arg_func, &req,
			msm_batt_modify_client_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: ERROR. failed to modify  Vbatt client\n",
		       __func__);
		return rc;
	}

	if (rep.result != BATTERY_MODIFICATION_SUCCESSFUL) {
		pr_err("%s: ERROR. modify client failed. result = %u\n",
		       __func__, rep.result);
		return -EIO;
	}

	return 0;
}

void msm_batt_early_suspend(struct early_suspend *h)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
/* FUJITSU:2011-12-01 mod Battery start */
#if 0
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				BATTERY_LOW, BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
				BATTERY_CB_ID_LOW_VOL, BATTERY_LOW);
#else
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				BATTERY_LOW, BATTERY_ALL_ACTIVITY,
				BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
#endif
/* FUJITSU:2011-12-01 mod Battery end */

		if (rc < 0) {
			pr_err("%s: msm_batt_modify_client. rc=%d\n",
			       __func__, rc);
			return;
		}
	} else {
		pr_err("%s: ERROR. invalid batt_handle\n", __func__);
		return;
	}

	pr_debug("%s: exit\n", __func__);
}

void msm_batt_late_resume(struct early_suspend *h)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
		if (rc < 0) {
			pr_err("%s: msm_batt_modify_client FAIL rc=%d\n",
			       __func__, rc);
			return;
		}
	} else {
		pr_err("%s: ERROR. invalid batt_handle\n", __func__);
		return;
	}

	msm_batt_update_psy_status();
	pr_debug("%s: exit\n", __func__);
}
#endif

struct msm_batt_vbatt_filter_req {
	u32 batt_handle;
	u32 enable_filter;
	u32 vbatt_filter;
};

struct msm_batt_vbatt_filter_rep {
	u32 result;
};

static int msm_batt_filter_arg_func(struct msm_rpc_client *batt_client,

		void *buf, void *data)
{
	struct msm_batt_vbatt_filter_req *vbatt_filter_req =
		(struct msm_batt_vbatt_filter_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(vbatt_filter_req->batt_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->enable_filter);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->vbatt_filter);
	size += sizeof(u32);
	return size;
}

static int msm_batt_filter_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{

	struct msm_batt_vbatt_filter_rep *data_ptr, *buf_ptr;

	data_ptr = (struct msm_batt_vbatt_filter_rep *)data;
	buf_ptr = (struct msm_batt_vbatt_filter_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);
	return 0;
}

static int msm_batt_enable_filter(u32 vbatt_filter)
{
	int rc;
	struct  msm_batt_vbatt_filter_req  vbatt_filter_req;
	struct  msm_batt_vbatt_filter_rep  vbatt_filter_rep;

	vbatt_filter_req.batt_handle = msm_batt_info.batt_handle;
	vbatt_filter_req.enable_filter = 1;
	vbatt_filter_req.vbatt_filter = vbatt_filter;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_ENABLE_DISABLE_FILTER_PROC,
			msm_batt_filter_arg_func, &vbatt_filter_req,
			msm_batt_filter_ret_func, &vbatt_filter_rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: enable vbatt filter. rc=%d\n",
		       __func__, rc);
		return rc;
	}

	if (vbatt_filter_rep.result != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: FAIL: enable vbatt filter: result=%d\n",
		       __func__, vbatt_filter_rep.result);
		return -EIO;
	}

	pr_debug("%s: enable vbatt filter: OK\n", __func__);
	return rc;
}

struct batt_client_registration_req {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 more_data;
	u32 batt_error;
};

struct batt_client_registration_req_4_1 {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 batt_error;
};

struct batt_client_registration_rep {
	u32 batt_handle;
};

struct batt_client_registration_rep_4_1 {
	u32 batt_handle;
	u32 more_data;
	u32 err;
};

static int msm_batt_register_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_registration_req *batt_reg_req =
		(struct batt_client_registration_req *)data;

	u32 *req = (u32 *)buf;
	int size = 0;


	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->voltage_direction);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_cb_id);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->cb_data);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_error);
		size += sizeof(u32);

		return size;
	} else {
		*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->voltage_direction);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_cb_id);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->cb_data);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->more_data);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_error);
		size += sizeof(u32);

		return size;
	}

}

static int msm_batt_register_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_registration_rep *data_ptr, *buf_ptr;
	struct batt_client_registration_rep_4_1 *data_ptr_4_1, *buf_ptr_4_1;

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		data_ptr_4_1 = (struct batt_client_registration_rep_4_1 *)data;
		buf_ptr_4_1 = (struct batt_client_registration_rep_4_1 *)buf;

		data_ptr_4_1->batt_handle
			= be32_to_cpu(buf_ptr_4_1->batt_handle);
		data_ptr_4_1->more_data
			= be32_to_cpu(buf_ptr_4_1->more_data);
		data_ptr_4_1->err = be32_to_cpu(buf_ptr_4_1->err);
		return 0;
	} else {
		data_ptr = (struct batt_client_registration_rep *)data;
		buf_ptr = (struct batt_client_registration_rep *)buf;

		data_ptr->batt_handle = be32_to_cpu(buf_ptr->batt_handle);
		return 0;
	}
}

static int msm_batt_register(u32 desired_batt_voltage,
			     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	struct batt_client_registration_req batt_reg_req;
	struct batt_client_registration_req_4_1 batt_reg_req_4_1;
	struct batt_client_registration_rep batt_reg_rep;
	struct batt_client_registration_rep_4_1 batt_reg_rep_4_1;
	void *request;
	void *reply;
	int rc;

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		batt_reg_req_4_1.desired_batt_voltage = desired_batt_voltage;
		batt_reg_req_4_1.voltage_direction = voltage_direction;
		batt_reg_req_4_1.batt_cb_id = batt_cb_id;
		batt_reg_req_4_1.cb_data = cb_data;
		batt_reg_req_4_1.batt_error = 1;
		request = &batt_reg_req_4_1;
	} else {
		batt_reg_req.desired_batt_voltage = desired_batt_voltage;
		batt_reg_req.voltage_direction = voltage_direction;
		batt_reg_req.batt_cb_id = batt_cb_id;
		batt_reg_req.cb_data = cb_data;
		batt_reg_req.more_data = 1;
		batt_reg_req.batt_error = 0;
		request = &batt_reg_req;
	}

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1)
		reply = &batt_reg_rep_4_1;
	else
		reply = &batt_reg_rep;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_REGISTER_PROC,
			msm_batt_register_arg_func, request,
			msm_batt_register_ret_func, reply,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt register. rc=%d\n", __func__, rc);
		return rc;
	}

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		if (batt_reg_rep_4_1.more_data != 0
			&& batt_reg_rep_4_1.err
				!= BATTERY_REGISTRATION_SUCCESSFUL) {
			pr_err("%s: vBatt Registration Failed proc_num=%d\n"
					, __func__, BATTERY_REGISTER_PROC);
			return -EIO;
		}
		msm_batt_info.batt_handle = batt_reg_rep_4_1.batt_handle;
	} else
		msm_batt_info.batt_handle = batt_reg_rep.batt_handle;

	return 0;
}

struct batt_client_deregister_req {
	u32 batt_handle;
};

struct batt_client_deregister_rep {
	u32 batt_error;
};

static int msm_batt_deregister_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_deregister_req *deregister_req =
		(struct  batt_client_deregister_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(deregister_req->batt_handle);
	size += sizeof(u32);

	return size;
}

static int msm_batt_deregister_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_deregister_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_client_deregister_rep *)data;
	buf_ptr = (struct batt_client_deregister_rep *)buf;

	data_ptr->batt_error = be32_to_cpu(buf_ptr->batt_error);

	return 0;
}

static int msm_batt_deregister(u32 batt_handle)
{
	int rc;
	struct batt_client_deregister_req req;
	struct batt_client_deregister_rep rep;

	req.batt_handle = batt_handle;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_DEREGISTER_CLIENT_PROC,
			msm_batt_deregister_arg_func, &req,
			msm_batt_deregister_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt deregister. rc=%d\n", __func__, rc);
		return rc;
	}

	if (rep.batt_error != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: vbatt deregistration FAIL. error=%d, handle=%d\n",
		       __func__, rep.batt_error, batt_handle);
		return -EIO;
	}

	return 0;
}
#endif  /* CONFIG_BATTERY_MSM_FAKE */

static int msm_batt_cleanup(void)
{
	int rc = 0;

/* FUJITSU:2011-12-01 add Battery start */
        cancel_delayed_work(&timer_work);
/* FUJITSU:2011-12-01 add Battery end */

#ifndef CONFIG_BATTERY_MSM_FAKE
	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {

		rc = msm_batt_deregister(msm_batt_info.batt_handle);
		if (rc < 0)
			pr_err("%s: FAIL: msm_batt_deregister. rc=%d\n",
			       __func__, rc);
	}

	msm_batt_info.batt_handle = INVALID_BATT_HANDLE;

	if (msm_batt_info.batt_client)
		msm_rpc_unregister_client(msm_batt_info.batt_client);
#endif  /* CONFIG_BATTERY_MSM_FAKE */

	if (msm_batt_info.msm_psy_ac)
		power_supply_unregister(msm_batt_info.msm_psy_ac);

	if (msm_batt_info.msm_psy_usb)
		power_supply_unregister(msm_batt_info.msm_psy_usb);

/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */
#ifdef CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS
	if (msm_batt_info.msm_psy_wireless)
		power_supply_unregister(msm_batt_info.msm_psy_wireless);
#endif  //CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_HOLDER
	if (msm_batt_info.msm_psy_holder)
		power_supply_unregister(msm_batt_info.msm_psy_holder);
#endif  //CONFIG_FUJITSU_CHARGER_TYPE_HOLDER

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_MHL
	if (msm_batt_info.msm_psy_mhl)
		power_supply_unregister(msm_batt_info.msm_psy_mhl);
#endif  //CONFIG_FUJITSU_CHARGER_TYPE_MHL
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now end */

	if (msm_batt_info.msm_psy_batt)
		power_supply_unregister(msm_batt_info.msm_psy_batt);

#ifndef CONFIG_BATTERY_MSM_FAKE
	if (msm_batt_info.chg_ep) {
		rc = msm_rpc_close(msm_batt_info.chg_ep);
		if (rc < 0) {
			pr_err("%s: FAIL. msm_rpc_close(chg_ep). rc=%d\n",
			       __func__, rc);
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (msm_batt_info.early_suspend.suspend == msm_batt_early_suspend)
		unregister_early_suspend(&msm_batt_info.early_suspend);
#endif
#endif

/* FUJITSU:2011-12-01 add Battery start */
        destroy_workqueue(msm_battery_workqueue);
/* FUJITSU:2011-12-01 add Battery end */

	return rc;
}

static u32 msm_batt_capacity(u32 current_voltage)
{
	u32 low_voltage = msm_batt_info.voltage_min_design;
	u32 high_voltage = msm_batt_info.voltage_max_design;

	if (current_voltage <= low_voltage)
		return 0;
	else if (current_voltage >= high_voltage)
		return 100;
	else
		return (current_voltage - low_voltage) * 100
			/ (high_voltage - low_voltage);
}

#ifndef CONFIG_BATTERY_MSM_FAKE
int msm_batt_get_charger_api_version(void)
{
	int rc ;
	struct rpc_reply_hdr *reply;

	struct rpc_req_chg_api_ver {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_chg_api_ver;

	struct rpc_rep_chg_api_ver {
		struct rpc_reply_hdr hdr;
		u32 num_of_chg_api_versions;
		u32 *chg_api_versions;
	};

	u32 num_of_versions;

	struct rpc_rep_chg_api_ver *rep_chg_api_ver;


	req_chg_api_ver.more_data = cpu_to_be32(1);

	msm_rpc_setup_req(&req_chg_api_ver.hdr, CHG_RPC_PROG, CHG_RPC_VER_1_1,
			  ONCRPC_CHARGER_API_VERSIONS_PROC);

	rc = msm_rpc_write(msm_batt_info.chg_ep, &req_chg_api_ver,
			sizeof(req_chg_api_ver));
	if (rc < 0) {
		pr_err("%s: FAIL: msm_rpc_write. proc=0x%08x, rc=%d\n",
		       __func__, ONCRPC_CHARGER_API_VERSIONS_PROC, rc);
		return rc;
	}

	for (;;) {
		rc = msm_rpc_read(msm_batt_info.chg_ep, (void *) &reply, -1,
				BATT_RPC_TIMEOUT);
		if (rc < 0)
			return rc;
		if (rc < RPC_REQ_REPLY_COMMON_HEADER_SIZE) {
			pr_err("%s: LENGTH ERR: msm_rpc_read. rc=%d (<%d)\n",
			       __func__, rc, RPC_REQ_REPLY_COMMON_HEADER_SIZE);

			rc = -EIO;
			break;
		}
		/* we should not get RPC REQ or call packets -- ignore them */
		if (reply->type == RPC_TYPE_REQ) {
			pr_err("%s: TYPE ERR: type=%d (!=%d)\n",
			       __func__, reply->type, RPC_TYPE_REQ);
			kfree(reply);
			continue;
		}

		/* If an earlier call timed out, we could get the (no
		 * longer wanted) reply for it.	 Ignore replies that
		 * we don't expect
		 */
		if (reply->xid != req_chg_api_ver.hdr.xid) {
			pr_err("%s: XID ERR: xid=%d (!=%d)\n", __func__,
			       reply->xid, req_chg_api_ver.hdr.xid);
			kfree(reply);
			continue;
		}
		if (reply->reply_stat != RPCMSG_REPLYSTAT_ACCEPTED) {
			rc = -EPERM;
			break;
		}
		if (reply->data.acc_hdr.accept_stat !=
				RPC_ACCEPTSTAT_SUCCESS) {
			rc = -EINVAL;
			break;
		}

		rep_chg_api_ver = (struct rpc_rep_chg_api_ver *)reply;

		num_of_versions =
			be32_to_cpu(rep_chg_api_ver->num_of_chg_api_versions);

		rep_chg_api_ver->chg_api_versions =  (u32 *)
			((u8 *) reply + sizeof(struct rpc_reply_hdr) +
			sizeof(rep_chg_api_ver->num_of_chg_api_versions));

		rc = be32_to_cpu(
			rep_chg_api_ver->chg_api_versions[num_of_versions - 1]);

		pr_debug("%s: num_of_chg_api_versions = %u. "
			"The chg api version = 0x%08x\n", __func__,
			num_of_versions, rc);
		break;
	}
	kfree(reply);
	return rc;
}

static int msm_batt_cb_func(struct msm_rpc_client *client,
			    void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *req;
	u32 procedure;
	u32 accept_status;

	req = (struct rpc_request_hdr *)buffer;
	procedure = be32_to_cpu(req->procedure);

	switch (procedure) {
	case BATTERY_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;

	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		pr_err("%s: ERROR. procedure (%d) not supported\n",
		       __func__, procedure);
		break;
	}

	msm_rpc_start_accepted_reply(msm_batt_info.batt_client,
			be32_to_cpu(req->xid), accept_status);

	rc = msm_rpc_send_accepted_reply(msm_batt_info.batt_client, 0);
	if (rc)
		pr_err("%s: FAIL: sending reply. rc=%d\n", __func__, rc);

	if (accept_status == RPC_ACCEPTSTAT_SUCCESS)
		msm_batt_update_psy_status();

	return rc;
}
#endif  /* CONFIG_BATTERY_MSM_FAKE */

static int __devinit msm_batt_probe(struct platform_device *pdev)
{
	int rc;
	struct msm_psy_batt_pdata *pdata = pdev->dev.platform_data;

	if (pdev->id != -1) {
		dev_err(&pdev->dev,
			"%s: MSM chipsets Can only support one"
			" battery ", __func__);
		return -EINVAL;
	}

#ifndef CONFIG_BATTERY_MSM_FAKE
	if (pdata->avail_chg_sources & AC_CHG) {
#else
	{
#endif
		rc = power_supply_register(&pdev->dev, &msm_psy_ac);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_ac = &msm_psy_ac;
		msm_batt_info.avail_chg_sources |= AC_CHG;
	}

	if (pdata->avail_chg_sources & USB_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_usb);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_usb = &msm_psy_usb;
		msm_batt_info.avail_chg_sources |= USB_CHG;
	}

/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now start */
#ifdef CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS
	if (pdata->avail_chg_sources & WIRELESS_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_wireless);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register(wireless) failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_wireless = &msm_psy_wireless;
		msm_batt_info.avail_chg_sources |= WIRELESS_CHG;
	}
#endif //CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_HOLDER
	if (pdata->avail_chg_sources & HOLDER_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_holder);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register(holder) failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_holder = &msm_psy_holder;
		msm_batt_info.avail_chg_sources |= HOLDER_CHG;
	}
#endif //CONFIG_FUJITSU_CHARGER_TYPE_HOLDER


#ifdef CONFIG_FUJITSU_CHARGER_TYPE_MHL
	if (pdata->avail_chg_sources & MHL_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_mhl);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register(mhl) failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_mhl = &msm_psy_mhl;
		msm_batt_info.avail_chg_sources |= MHL_CHG;
	}
/* FUJITSU:2011-12-01 add Battery end */
#endif //CONFIG_FUJITSU_CHARGER_TYPE_MHL

	if (!msm_batt_info.msm_psy_ac && !msm_batt_info.msm_psy_usb
#ifdef CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS
	&& !msm_batt_info.msm_psy_wireless
#endif //CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS
#ifdef CONFIG_FUJITSU_CHARGER_TYPE_HOLDER
	&& !msm_batt_info.msm_psy_holder
#endif //CONFIG_FUJITSU_CHARGER_TYPE_HOLDER
#ifdef CONFIG_FUJITSU_CHARGER_TYPE_MHL
	&& !msm_batt_info.msm_psy_mhl
#endif //CONFIG_FUJITSU_CHARGER_TYPE_MHL
	) {
		dev_err(&pdev->dev,
			"%s: No external Power supply(AC, USB,WIRELESS, HOLDER,MHL)"
			"is avilable\n", __func__);
		msm_batt_cleanup();
	}
/* FUJITSU:2012-04-28 mod Battery 12_1 holder,mhl type, voltage_now end */

	msm_batt_info.voltage_max_design = pdata->voltage_max_design;
	msm_batt_info.voltage_min_design = pdata->voltage_min_design;
	msm_batt_info.batt_technology = pdata->batt_technology;
	msm_batt_info.calculate_capacity = pdata->calculate_capacity;

	if (!msm_batt_info.voltage_min_design)
		msm_batt_info.voltage_min_design = BATTERY_LOW;
	if (!msm_batt_info.voltage_max_design)
		msm_batt_info.voltage_max_design = BATTERY_HIGH;

	if (msm_batt_info.batt_technology == POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
		msm_batt_info.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION;

	if (!msm_batt_info.calculate_capacity)
		msm_batt_info.calculate_capacity = msm_batt_capacity;

	rc = power_supply_register(&pdev->dev, &msm_psy_batt);
	if (rc < 0) {
		dev_err(&pdev->dev, "%s: power_supply_register failed"
			" rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}
	msm_batt_info.msm_psy_batt = &msm_psy_batt;

#ifndef CONFIG_BATTERY_MSM_FAKE
	rc = msm_batt_register(BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_register failed rc = %d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	rc =  msm_batt_enable_filter(VBATT_FILTER);

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_enable_filter failed rc = %d\n",
			__func__, rc);
		msm_batt_cleanup();
		return rc;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	msm_batt_info.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	msm_batt_info.early_suspend.suspend = msm_batt_early_suspend;
	msm_batt_info.early_suspend.resume = msm_batt_late_resume;
	register_early_suspend(&msm_batt_info.early_suspend);
#endif
	msm_batt_update_psy_status();

#else
	power_supply_changed(&msm_psy_ac);
#endif  /* CONFIG_BATTERY_MSM_FAKE */

	return 0;
}

static int __devexit msm_batt_remove(struct platform_device *pdev)
{
	int rc;
	rc = msm_batt_cleanup();

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_cleanup  failed rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static struct platform_driver msm_batt_driver = {
	.probe = msm_batt_probe,
	.remove = __devexit_p(msm_batt_remove),
	.driver = {
		   .name = "msm-battery",
		   .owner = THIS_MODULE,
		   },
};

/* FUJITSU:2011-12-01 add Battery start */
static void get_battery_voltage(struct work_struct *work)
{
        u32 battery_voltage;
        struct  power_supply    *supp;
#ifdef CONFIG_FUJITSU_MSM_TEMP
        unsigned char *smemprm = NULL;
#endif //CONFIG_FUJITSU_MSM_TEMP

#ifdef CONFIG_FUJITSU_RVOLTAGE
        uint32_t *smemprmrv = NULL;
#endif // CONFIG_FUJITSU_RVOLTAGE

        battery_voltage = msm_batt_get_vbatt_voltage();
        if (battery_voltage == 0){
            /* RPC error  */;
        }else{
            mutex_lock(&msm_mutex);

            if (msm_batt_info.calculate_capacity){
                msm_batt_info.batt_capacity = msm_batt_info.calculate_capacity(battery_voltage);
            }

            switch(msm_batt_info.batt_status){
            case POWER_SUPPLY_STATUS_CHARGING:
                break;

            case POWER_SUPPLY_STATUS_NOT_CHARGING:
                if(msm_batt_info.batt_capacity == 100){
			msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
			DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_FULL\n");
                }
                break;

            case POWER_SUPPLY_STATUS_FULL:
                if(msm_batt_info.batt_capacity < 100){
                    /* If Charger is connected Make status as Charging other wise make it Not Charging */
                    if (msm_batt_info.charger_status == CHARGER_STATUS_GOOD || 
                        msm_batt_info.charger_status == CHARGER_STATUS_WEAK){
                        /*if charger is connected and battery goes down*/
                        if (msm_batt_info.current_chg_source){
                            msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
		                DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_CHARGING\n");
                        }
                    }else{
                        /* If charger is not connected */
/* FUJITSU:2012-05-03 mod Battery 12_1 NOT_CHARGING(plug source), DISCHARGING(unplug source) start */
#ifdef  CONFIG_FUJITSU_CHARGER_12_1
                        if (msm_batt_info.current_chg_source){
				            msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		                    DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_NOT_CHARGING\n");
			            }else{
				            msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		                   DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_DISCHARGING\n");
			            }

#else
			            msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			            DBG_LIMIT("BATTSTAT: POWER_SUPPLY_STATUS_NOT_CHARGING\n");
#endif
/* FUJITSU:2012-05-03 mod Battery 12_1 NOT_CHARGING(plug source), DISCHARGING(unplug source) end */

                    }
                }
                break;
            }

/* FUJITSU:2012-05-23 mod temperature start */
#ifdef CONFIG_FUJITSU_MSM_TEMP
#ifdef CONFIG_FUJITSU_CHARGER_12_1
			smemprm = (unsigned char *)smem_alloc_vendor1(SMEM_OEM_004);
			if (smemprm != NULL){
				msm_batt_info.msm_temp = *smemprm;
				msm_batt_info.ambient_temp = adc_to_temp(msm_batt_info.msm_temp);
				msm_batt_info.case_temp = msm_batt_info.ambient_temp;
			}
#else //CONFIG_FUJITSU_CHARGER_12_1
			smemprm = (unsigned char *)smem_alloc_vendor1(SMEM_OEM_004);
			if (smemprm != NULL){
				msm_batt_info.msm_temp = *smemprm;
			}
#endif //CONFIG_FUJITSU_CHARGER_12_1
#endif //CONFIG_FUJITSU_MSM_TEMP
/* FUJITSU:2012-05-23 mod temperature end */

#ifdef CONFIG_FUJITSU_RVOLTAGE
            smemprmrv = (uint32_t *)smem_alloc_vendor1(SMEM_OEM_018);
            if (smemprmrv != NULL){
                msm_batt_info.battery_rvoltage = *smemprmrv;
            }
#endif // CONFIG_FUJITSU_RVOLTAGE

            supp = &msm_psy_batt;
            power_supply_changed(supp);
            mutex_unlock(&msm_mutex);
        }

/* FUJITSU:2012-05-28 add temperature log start */
		if( check_erapsed_time >= 600000) {
			DBG_LIMIT("BATTINFO: battery_voltage = %d\n", battery_voltage);
#ifdef CONFIG_FUJITSU_RVOLTAGE
			DBG_LIMIT("BATTINFO: battery_rvoltage = %d\n", msm_batt_info.battery_rvoltage);
#endif // CONFIG_FUJITSU_RVOLTAGE
#ifdef CONFIG_FUJITSU_CHARGER_12_1
			DBG_LIMIT("BATTINFO: ambient_temp = %d\n", msm_batt_info.ambient_temp*10);
#endif //CONFIG_FUJITSU_CHARGER_12_1
			check_erapsed_time = 0;
		}
		else {
			check_erapsed_time += BATT_UPDATE_TIMEOUT;
		}
/* FUJITSU:2012-05-28 add temperature log end */

        queue_delayed_work(msm_battery_workqueue, &timer_work, msecs_to_jiffies(BATT_UPDATE_TIMEOUT));
}
/* FUJITSU:2011-12-01 add Battery end */

static int __devinit msm_batt_init_rpc(void)
{
	int rc;

#ifdef CONFIG_BATTERY_MSM_FAKE
	pr_info("Faking MSM battery\n");
#else

	msm_batt_info.chg_ep =
		msm_rpc_connect_compatible(CHG_RPC_PROG, CHG_RPC_VER_4_1, 0);
	msm_batt_info.chg_api_version =  CHG_RPC_VER_4_1;
	if (msm_batt_info.chg_ep == NULL) {
		pr_err("%s: rpc connect CHG_RPC_PROG = NULL\n", __func__);
		return -ENODEV;
	}

	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_3_1, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_3_1;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_1_1, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_1_1;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_1_3, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_1_3;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_2_2, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_2_2;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		rc = PTR_ERR(msm_batt_info.chg_ep);
		pr_err("%s: FAIL: rpc connect for CHG_RPC_PROG. rc=%d\n",
		       __func__, rc);
		msm_batt_info.chg_ep = NULL;
		return rc;
	}

	/* Get the real 1.x version */
	if (msm_batt_info.chg_api_version == CHG_RPC_VER_1_1)
		msm_batt_info.chg_api_version =
			msm_batt_get_charger_api_version();

	/* Fall back to 1.1 for default */
	if (msm_batt_info.chg_api_version < 0)
		msm_batt_info.chg_api_version = CHG_RPC_VER_1_1;
	msm_batt_info.batt_api_version =  BATTERY_RPC_VER_4_1;

	msm_batt_info.batt_client =
		msm_rpc_register_client("battery", BATTERY_RPC_PROG,
					BATTERY_RPC_VER_4_1,
					1, msm_batt_cb_func);

	if (msm_batt_info.batt_client == NULL) {
		pr_err("%s: FAIL: rpc_register_client. batt_client=NULL\n",
		       __func__);
		return -ENODEV;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_1_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_1_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_2_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_2_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_5_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_5_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		rc = PTR_ERR(msm_batt_info.batt_client);
		pr_err("%s: ERROR: rpc_register_client: rc = %d\n ",
		       __func__, rc);
		msm_batt_info.batt_client = NULL;
		return rc;
	}
#endif  /* CONFIG_BATTERY_MSM_FAKE */

	rc = platform_driver_register(&msm_batt_driver);

	if (rc < 0)
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		       __func__, rc);

/* FUJITSU:2011-12-01 add Battery start */
	msm_battery_workqueue = create_singlethread_workqueue("msm_battery");
        queue_delayed_work(msm_battery_workqueue, &timer_work, msecs_to_jiffies(BATT_UPDATE_TIMEOUT));
/* FUJITSU:2011-12-01 add Battery end */

	return rc;
}

static int __init msm_batt_init(void)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

/* FUJITSU:2011-12-01 add Battery start */
	wake_lock_init(&batt_wakelock, WAKE_LOCK_SUSPEND, "batt");
/* FUJITSU:2011-12-01 add Battery end */

	rc = msm_batt_init_rpc();

	if (rc < 0) {
		pr_err("%s: FAIL: msm_batt_init_rpc.  rc=%d\n", __func__, rc);
/* FUJITSU:2011-12-01 add Battery start */
		wake_lock_destroy(&batt_wakelock);
/* FUJITSU:2011-12-01 add Battery end */
		msm_batt_cleanup();
		return rc;
	}

	pr_info("%s: Charger/Battery = 0x%08x/0x%08x (RPC version)\n",
		__func__, msm_batt_info.chg_api_version,
		msm_batt_info.batt_api_version);

	return 0;
}

static void __exit msm_batt_exit(void)
{
/* FUJITSU:2011-12-01 add Battery start */
	wake_lock_destroy(&batt_wakelock);
/* FUJITSU:2011-12-01 add Battery end */
	platform_driver_unregister(&msm_batt_driver);
}

module_init(msm_batt_init);
module_exit(msm_batt_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kiran Kandi, Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:msm_battery");
