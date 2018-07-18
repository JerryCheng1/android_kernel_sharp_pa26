/* drivers/sharp/shtps/sy3000/shtps_filter_water_cling_reject_check.c
 *
 * Copyright (c) 2014, Sharp. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/* -------------------------------------------------------------------------- */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include <sharp/shtps_dev.h>

#include "shtps_rmi.h"
#include "shtps_param_extern.h"
#include "shtps_log.h"

#include "shtps_filter.h"
/* -------------------------------------------------------------------------- */
#if defined(SHTPS_WATER_CLING_REJECTION_ENABLE)
struct shtps_filter_water_cling_reject_info{
	int				count[SHTPS_FINGER_MAX];
	unsigned long	tu_time[SHTPS_FINGER_MAX];
};

/* -------------------------------------------------------------------------- */
void shtps_filter_water_cling_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int w, ratio;
	int i;
	int fingerMax = shtps_get_fingermax(ts);

	if(!SHTPS_WATER_CLING_REJECT_ENABLE){
		return;
	}
	
	for(i = 0;i < fingerMax;i++){
		if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				if(ts->water_cling_reject_p->count[i] != 0){
					if(time_after(jiffies, ts->water_cling_reject_p->tu_time[i] + msecs_to_jiffies(SHTPS_WATER_CLING_REJECT_COUNT_CLEAR_TIME_THRESH)) != 0){
						SHTPS_LOG_DBG_WATER_CLING_PRINT("<%d> clear count by tu\n", i);
						ts->water_cling_reject_p->count[i]     = 0;
					}
				}
			}
			
			w = (ts->fw_report_info.fingers[i].wx >= ts->fw_report_info.fingers[i].wy)?
						ts->fw_report_info.fingers[i].wx : ts->fw_report_info.fingers[i].wy;
			ratio = (ts->fw_report_info.fingers[i].z * 100) / ((w > 0)?w:1);

			if(ratio <= SHTPS_WATER_CLING_REJECT_RATIO_THRESH || 
				ts->fw_report_info.fingers[i].z <= SHTPS_WATER_CLING_REJECT_Z_THRESH)
			{
				ts->water_cling_reject_p->count[i]++;
				SHTPS_LOG_DBG_WATER_CLING_PRINT("<%d> ratio = %d. z = %d. count inc %d.\n", 
										i, ratio, ts->fw_report_info.fingers[i].z, ts->water_cling_reject_p->count[i]);

				if(ts->water_cling_reject_p->count[i] >= SHTPS_WATER_CLING_REJECT_COUNT_MAX){
					u8 j;
					SHTPS_LOG_DBG_WATER_CLING_PRINT("<%d> exec force cal.\n", i);
					shtps_rezero(ts);
					for(j = 0;j < fingerMax;j++){
						info->fingers[j].state = SHTPS_TOUCH_STATE_NO_TOUCH;
					}
					memset(ts->water_cling_reject_p, 0, sizeof(struct shtps_filter_water_cling_reject_info));
					return;
				}
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_water_cling_reject_forcetouchup(struct shtps_rmi_spi *ts)
{
	memset(ts->water_cling_reject_p, 0, sizeof(struct shtps_filter_water_cling_reject_info));
}

/* -------------------------------------------------------------------------- */
void shtps_filter_water_cling_reject_init(struct shtps_rmi_spi *ts)
{
	ts->water_cling_reject_p = kzalloc(sizeof(struct shtps_filter_water_cling_reject_info), GFP_KERNEL);
	if(ts->water_cling_reject_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(ts->water_cling_reject_p, 0, sizeof(struct shtps_filter_water_cling_reject_info));	// no need
}

/* -------------------------------------------------------------------------- */
void shtps_filter_water_cling_reject_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->water_cling_reject_p)	kfree(ts->water_cling_reject_p);
	ts->water_cling_reject_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_WATER_CLING_REJECTION_ENABLE */
