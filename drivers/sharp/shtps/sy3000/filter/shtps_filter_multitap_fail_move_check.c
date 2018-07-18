/* drivers/sharp/shtps/sy3000/shtps_filter_multitap_fail_move_check.c
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
#include <linux/hrtimer.h>

#include <sharp/shtps_dev.h>

#include "shtps_rmi.h"
#include "shtps_param_extern.h"
#include "shtps_log.h"

#include "shtps_filter.h"
/* -------------------------------------------------------------------------- */
#if defined(SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE)
struct shtps_filter_multitap_fail_move_reject_info{
	u8				state;
	u8				id;
	unsigned long	time;
};

void shtps_filter_multitap_fail_move_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	u8 i;
	u8 id;
	int numOfFingers = 0;
	int fingerMax = shtps_get_fingermax(ts);
	
	if(!SHTPS_MULTI_TAP_FAIL_MOVE_REJECT_ENABLE){
		return;
	}
	
	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			if(numOfFingers == 0){
				id = i;
			}
			numOfFingers++;
		}
	}
	
	if(numOfFingers == 1){
		if(ts->fw_report_info_store.fingers[id].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			ts->multitap_fail_move_reject_p->id   = id;
			ts->multitap_fail_move_reject_p->time = jiffies;
			ts->multitap_fail_move_reject_p->state= 1;
		}
		if(ts->multitap_fail_move_reject_p->state == 1){
			if(time_after(jiffies, ts->multitap_fail_move_reject_p->time + msecs_to_jiffies(SHTPS_MULTI_TAP_FAIL_MOVE_REJECT_TIME_THRESH)) != 0){
				ts->multitap_fail_move_reject_p->state = 2;
			}
		}
	}else if(numOfFingers == 2 && ts->multitap_fail_move_reject_p->state == 1){
		id = ts->multitap_fail_move_reject_p->id;
		
		if(ts->fw_report_info.fingers[id].state == SHTPS_TOUCH_STATE_FINGER){
			if(time_after(jiffies, ts->multitap_fail_move_reject_p->time + msecs_to_jiffies(SHTPS_MULTI_TAP_FAIL_MOVE_REJECT_TIME_THRESH)) == 0){
				if(abs(ts->fw_report_info.fingers[id].x - ts->fw_report_info_store.fingers[id].x) > SHTPS_MULTI_TAP_FAIL_MOVE_REJECT_POS_THRESH ||
				   abs(ts->fw_report_info.fingers[id].y - ts->fw_report_info_store.fingers[id].y) > SHTPS_MULTI_TAP_FAIL_MOVE_REJECT_POS_THRESH)
				{
					SHTPS_LOG_DBG_MULTI_TAP_FAIL_MOVE_PRINT("[%d] cancel event\n", i);
					shtps_report_touch_on(ts, id,
										  SHTPS_TOUCH_CANCEL_COORDINATES_X,
										  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
										  shtps_get_fingerwidth(ts, id, &ts->report_info),
										  ts->report_info.fingers[id].wx,
										  ts->report_info.fingers[id].wy,
										  ts->report_info.fingers[id].z);
					input_sync(ts->input);
					ts->multitap_fail_move_reject_p->state = 0;
				}
			}
		}
		ts->multitap_fail_move_reject_p->state = 0;
	}else{
		ts->multitap_fail_move_reject_p->state = 0;
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_multitap_fail_move_forcetouchup(struct shtps_rmi_spi *ts)
{
	memset(ts->multitap_fail_move_reject_p, 0, sizeof(struct shtps_filter_multitap_fail_move_reject_info));
	//memset(&ts->multitap_fail_move_reject, 0, sizeof(ts->multitap_fail_move_reject));
}

/* -------------------------------------------------------------------------- */
void shtps_filter_multitap_fail_move_init(struct shtps_rmi_spi *ts)
{
	ts->multitap_fail_move_reject_p = kzalloc(sizeof(struct shtps_filter_multitap_fail_move_reject_info), GFP_KERNEL);
	if(ts->multitap_fail_move_reject_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(&ts->multitap_fail_move_reject_p, 0, sizeof(struct shtps_filter_multitap_fail_move_reject_info));	//no need
}

/* -------------------------------------------------------------------------- */
void shtps_filter_multitap_fail_move_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->multitap_fail_move_reject_p)	kfree(ts->multitap_fail_move_reject_p);
	ts->multitap_fail_move_reject_p = NULL;
}
#endif /* SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE */
