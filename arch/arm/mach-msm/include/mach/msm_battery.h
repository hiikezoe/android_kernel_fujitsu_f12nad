/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
#ifndef __MSM_BATTERY_H__
#define __MSM_BATTERY_H__

#define AC_CHG        0x00000001
#define USB_CHG       0x00000002

/* FUJITSU:2012-05-09 mod Battery 12_1 holder,mhl type, voltage_now start */
#ifdef CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS
#define WIRELESS_CHG  0x00000004
#endif //CONFIG_FUJITSU_CHARGER_TYPE_WIRELESS

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_HOLDER
#define HOLDER_CHG    0x00000008
#endif //CONFIG_FUJITSU_CHARGER_TYPE_HOLDER

#ifdef CONFIG_FUJITSU_CHARGER_TYPE_MHL
#define MHL_CHG       0x00000010
#endif //CONFIG_FUJITSU_CHARGER_TYPE_MHL
/* FUJITSU:2012-05-09 mod Battery 12_1 holder,mhl type, voltage_now end */

struct msm_psy_batt_pdata {
	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 avail_chg_sources;
	u32 batt_technology;
	u32 (*calculate_capacity)(u32 voltage);
};

#endif
