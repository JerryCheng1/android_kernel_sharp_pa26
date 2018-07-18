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
#include <linux/hrtimer.h>

#include <sharp/shtps_dev.h>

#include "shtps_rmi.h"
#include "shtps_param_extern.h"
#include "shtps_log.h"

#include "shtps_filter.h"
/* -------------------------------------------------------------------------- */
#if defined(SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
struct shtps_filter_topofscreen_ghost_reject_info{
	u8						is_ghost[SHTPS_FINGER_MAX];
	unsigned long			timeout[SHTPS_FINGER_MAX];
	struct delayed_work		ghost_clear_check_delayed_work;
	//
	struct shtps_rmi_spi *ts_p;
};

enum{
	TOP_OF_SCREEN_GHOST_NOT_DETECTED,
	TOP_OF_SCREEN_GHOST_DETECTED,
};

enum{
	TOP_OF_SCREEN_GHOST_NOT_CLEARED,
	TOP_OF_SCREEN_GHOST_CLEARED,
};


/* -------------------------------------------------------------------------- */
static int ghost_check(struct shtps_rmi_spi *ts, u8 finger)
{
	int ret = TOP_OF_SCREEN_GHOST_NOT_DETECTED;
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int numOfSamelineFingers = 0;
	int numOfBottomAreaFingers = 0;
	int bottomAreaZMax = 0;
	unsigned long diff;
	
	if(ts->topofscreen_ghost_reject_p->is_ghost[finger] != 0){
		return TOP_OF_SCREEN_GHOST_DETECTED;
	}
	
	if(ts->fw_report_info_store.fingers[finger].state == SHTPS_TOUCH_STATE_NO_TOUCH &&
		ts->fw_report_info.fingers[finger].state != SHTPS_TOUCH_STATE_NO_TOUCH)
	{
		if(ts->fw_report_info.fingers[finger].y <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_Y_THRESH &&
		   ts->fw_report_info.fingers[finger].z <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_Z_THRESH)
		{
			for(i = 0;i < fingerMax;i++){
				if(i == finger || ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
					continue;
				}
				
				diff = abs(ts->fw_report_info.fingers[i].x - ts->fw_report_info.fingers[finger].x);
				
				if(diff <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_XDIFF_THRESH &&
				   ts->fw_report_info.fingers[i].y  >  ts->fw_report_info.fingers[finger].y  &&
				   ts->fw_report_info.fingers[i].z  >  ts->fw_report_info.fingers[finger].z  &&
				   ts->fw_report_info.fingers[i].wx >= ts->fw_report_info.fingers[finger].wx &&
				   ((ts->fw_report_info.fingers[finger].z * 100) / ts->fw_report_info.fingers[i].z) < SHTPS_TOP_OF_SCREEN_GHOST_REJECT_Z_RATIO_THRESH)
				{
					numOfSamelineFingers++;
					if(numOfSamelineFingers >= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_FINGER_NUM_THRESH ||
					   ts->fw_report_info.fingers[i].wy >= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_FINGER_WY_THRESH)
					{
						ret = TOP_OF_SCREEN_GHOST_DETECTED;
						SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] detect ghost\n", finger);
						break;
					}
				}
				
				if(ts->fw_report_info.fingers[i].y > SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_CHECK_AREA){
					numOfBottomAreaFingers++;
					bottomAreaZMax = (bottomAreaZMax < ts->fw_report_info.fingers[i].z)? ts->fw_report_info.fingers[i].z : bottomAreaZMax;
				}
			}
		}
		
		if(ret != TOP_OF_SCREEN_GHOST_DETECTED){
			if(bottomAreaZMax == 255 ||
			   (numOfBottomAreaFingers >= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_FINGER_NUM_THRESH &&
			    bottomAreaZMax >= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_FINGER_Z_THRESH))
			{
				if(ts->fw_report_info.fingers[finger].y <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_Y_THRESH &&
				   ts->fw_report_info.fingers[finger].z <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_Z_THRESH)
				{
					ret = TOP_OF_SCREEN_GHOST_DETECTED;
					SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] detect ghost by bottom palm touch\n", finger);
				}
			}
		}
	}
	
	return ret;
}

static int ghost_clear_check(struct shtps_rmi_spi *ts, u8 finger)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int numOfBottomAreaFingers = 0;
	int bottomAreaZMax = 0;
	unsigned long diff;

	if(ts->topofscreen_ghost_reject_p->is_ghost[finger] == 0){
		return TOP_OF_SCREEN_GHOST_CLEARED;
	}
	
	if(ts->fw_report_info.fingers[finger].state == SHTPS_TOUCH_STATE_NO_TOUCH){
		SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] clear ghost by touch-state\n", finger);
		return TOP_OF_SCREEN_GHOST_CLEARED;
	}
	
	if(time_after(jiffies, ts->topofscreen_ghost_reject_p->timeout[finger])){
		SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] clear ghost by time\n", finger);
		return TOP_OF_SCREEN_GHOST_CLEARED;
	}
	
	if(ts->fw_report_info.fingers[finger].y > SHTPS_TOP_OF_SCREEN_GHOST_REJECT_Y_THRESH ||
	   ts->fw_report_info.fingers[finger].z > SHTPS_TOP_OF_SCREEN_GHOST_REJECT_Z_THRESH)
	{
		SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] clear ghost by ypos(%d) or z(%d)\n", finger,
			ts->fw_report_info.fingers[finger].y, ts->fw_report_info.fingers[finger].z);
		return TOP_OF_SCREEN_GHOST_CLEARED;
	}
	
	for(i = 0;i < fingerMax;i++){
		if(i == finger || ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			continue;
		}
		
		diff = abs(ts->fw_report_info.fingers[i].x - ts->fw_report_info.fingers[finger].x);
		
		if(diff <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_XDIFF_THRESH){
			return TOP_OF_SCREEN_GHOST_NOT_CLEARED;
		}
		
		if(ts->fw_report_info.fingers[i].y > SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_CHECK_AREA){
			numOfBottomAreaFingers++;
			bottomAreaZMax = (bottomAreaZMax < ts->fw_report_info.fingers[i].z)? ts->fw_report_info.fingers[i].z : bottomAreaZMax;
		}
	}
	
	if(bottomAreaZMax == 255 ||
	   (numOfBottomAreaFingers >= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_FINGER_NUM_THRESH &&
	    bottomAreaZMax >= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_FINGER_Z_THRESH))
	{
		if(ts->fw_report_info.fingers[finger].z <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_Z_THRESH){
			return TOP_OF_SCREEN_GHOST_NOT_CLEARED;
		}
	}

	SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] clear ghost by other finger\n", finger);
	return TOP_OF_SCREEN_GHOST_CLEARED;
}

/* -------------------------------------------------------------------------- */
static void ghost_clear_check_timer_start(struct shtps_rmi_spi *ts)
{
	int i;
	int fingerMax   = shtps_get_fingermax(ts);
	u8 isNeedWork   = 0;
	unsigned long timeoutTime = 0;
	unsigned long tempTime;

	for(i = 0;i < fingerMax;i++){
		if(ts->topofscreen_ghost_reject_p->is_ghost[i] != 0){
			isNeedWork = 1;
			tempTime   = ts->topofscreen_ghost_reject_p->timeout[i] - jiffies;
			
			SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] timeout = %lu\n", i, tempTime);
			if(timeoutTime == 0 || timeoutTime < tempTime){
				timeoutTime = tempTime;
			}
		}
	}
	
	if(isNeedWork){
		if(timeoutTime == 0){
			timeoutTime = 1;
		}
		SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("start timer (%d ms)\n", jiffies_to_msecs(timeoutTime));
		cancel_delayed_work(&ts->topofscreen_ghost_reject_p->ghost_clear_check_delayed_work);
		schedule_delayed_work(&ts->topofscreen_ghost_reject_p->ghost_clear_check_delayed_work, timeoutTime);
	}
}

/* -------------------------------------------------------------------------- */
static void ghost_clear_check_timer_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("cancel ghost check work\n");
	cancel_delayed_work(&ts->topofscreen_ghost_reject_p->ghost_clear_check_delayed_work);
}

/* -------------------------------------------------------------------------- */
static void ghost_clear_check_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_filter_topofscreen_ghost_reject_info *wgr = container_of(dw, struct shtps_filter_topofscreen_ghost_reject_info, ghost_clear_check_delayed_work);
	struct shtps_rmi_spi *ts = wgr->ts_p;

	SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("start ghost check work\n");

	shtps_mutex_lock_ctrl();
	if(ts->state_mgr.state == SHTPS_STATE_ACTIVE || ts->state_mgr.state == SHTPS_STATE_FACETOUCH){
		int i;
		int fingerMax = shtps_get_fingermax(ts);
		u8 isStateChange = 0;
		
		for(i = 0;i < fingerMax;i++){
			if(ts->topofscreen_ghost_reject_p->is_ghost[i] != 0){
				if(ghost_clear_check(ts, i) == TOP_OF_SCREEN_GHOST_CLEARED){
					isStateChange = 1;
					ts->topofscreen_ghost_reject_p->is_ghost[i] = 0;
				}
			}
		}
		
		if(isStateChange){
			shtps_read_touchevent(ts, ts->state_mgr.state);
		}
		
		ghost_clear_check_timer_start(ts);
	}
	shtps_mutex_unlock_ctrl();

	SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("end ghost check work\n");
	
	return;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_topofscreen_ghost_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);

	if(!SHTPS_TOP_OF_SCREEN_GHOST_REJECT_ENABLE){
		return;
	}
	
	for(i = 0;i < fingerMax;i++){
		if(ts->topofscreen_ghost_reject_p->is_ghost[i] == 0){
			if(ghost_check(ts, i) == TOP_OF_SCREEN_GHOST_DETECTED){
				ts->topofscreen_ghost_reject_p->is_ghost[i] = 1;
				ts->topofscreen_ghost_reject_p->timeout[i]  = jiffies + msecs_to_jiffies(SHTPS_TOP_OF_SCREEN_GHOST_REJECT_CLEAR_TIME);
			}
		}else{
			if(ghost_clear_check(ts, i) == TOP_OF_SCREEN_GHOST_CLEARED){
				ts->topofscreen_ghost_reject_p->is_ghost[i] = 0;
			}
		}
	}

	for(i = 0;i < fingerMax;i++){
		if(ts->topofscreen_ghost_reject_p->is_ghost[i] != 0){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] reject ghost\n", i);
		}
	}
	
	ghost_clear_check_timer_start(ts);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_topofscreen_ghost_reject_forcetouchup(struct shtps_rmi_spi *ts)
{
	ghost_clear_check_timer_cancel(ts);
	memset(ts->topofscreen_ghost_reject_p->is_ghost, 0, sizeof(ts->topofscreen_ghost_reject_p->is_ghost));
}

/* -------------------------------------------------------------------------- */
void shtps_filter_topofscreen_ghost_reject_init(struct shtps_rmi_spi *ts)
{
	ts->topofscreen_ghost_reject_p = kzalloc(sizeof(struct shtps_filter_topofscreen_ghost_reject_info), GFP_KERNEL);
	if(ts->topofscreen_ghost_reject_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	INIT_DELAYED_WORK(&ts->topofscreen_ghost_reject_p->ghost_clear_check_delayed_work, ghost_clear_check_delayed_work_function);
	ts->topofscreen_ghost_reject_p->ts_p = ts;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_topofscreen_ghost_reject_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->topofscreen_ghost_reject_p)	kfree(ts->topofscreen_ghost_reject_p);
	ts->topofscreen_ghost_reject_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */
