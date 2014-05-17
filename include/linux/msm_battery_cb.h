#ifndef _MSM_BATTERY_CB_H
#define _MSM_BATTERY_CB_H

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

struct msm_battery_callback_info{
    int (*callback)(unsigned int batt_status, void *data);
    void *data;
};

int set_msm_battery_callback_info(struct msm_battery_callback_info *info);
int unset_msm_battery_callback_info(void);

#endif
