/* drivers/sharp/shtps/sy3000/shtps_filter_absorption_check.c
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
#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
struct shtps_filter_absorption_check_info{
	u8							absorption_hold_enable;
	u8							absorption_hold_tu_finger;
	struct shtps_touch_info		absorption_hold_finger_info;
	struct delayed_work			absorption_hold_off_delayed_work;

	#if defined(SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE)
		unsigned short			pre_diff_x;
		unsigned short			pre_diff_y;
	#endif /* SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE */

	struct shtps_rmi_spi		*ts_p;
};

/* -------------------------------------------------------------------------- */
static void shtps_filter_absorption_hold_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_filter_absorption_check_info *ac = container_of(dw, struct shtps_filter_absorption_check_info, absorption_hold_off_delayed_work);
	//struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, absorption_hold_off_delayed_work);
	struct shtps_rmi_spi *ts = ac->ts_p;

	SHTPS_LOG_FUNC_CALL();
	shtps_mutex_lock_ctrl();
	if(ts->absorption_check_p->absorption_hold_enable){
		SHTPS_LOG_DBG_PRINT("%s() notify hold tu event", __func__);
		if(ts->absorption_check_p->absorption_hold_tu_finger < SHTPS_FINGER_MAX){
			shtps_report_touch_off(ts, ts->absorption_check_p->absorption_hold_tu_finger,
						  ts->absorption_check_p->absorption_hold_finger_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger].x,
						  ts->absorption_check_p->absorption_hold_finger_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger].y,
						  0,
						  ts->absorption_check_p->absorption_hold_finger_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger].wx,
						  ts->absorption_check_p->absorption_hold_finger_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger].wy,
						  ts->absorption_check_p->absorption_hold_finger_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger].z);
			
			input_sync(ts->input);
			ts->touch_state.numOfFingers = 1;
			memcpy(&ts->report_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger], 
					&ts->absorption_check_p->absorption_hold_finger_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger], 
					sizeof(ts->report_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger]));
		}

		ts->absorption_check_p->absorption_hold_enable = 0;
		ts->absorption_check_p->absorption_hold_tu_finger = 0;
	}
	shtps_mutex_unlock_ctrl();
}

/* -------------------------------------------------------------------------- */
void shtps_filter_absorption_hold_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	cancel_delayed_work(&ts->absorption_check_p->absorption_hold_off_delayed_work);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_absorption_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int fingerMax = shtps_get_fingermax(ts);
	int i;
	int td_finger = -1;
	int tu_finger = -1;
	int numOfFingers = 0;
	#if defined(SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE)
		unsigned short x[2], y[2];
	#endif /* SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE */
	
	SHTPS_LOG_FUNC_CALL();
	if(!SHTPS_FINGER_ABSORPTION_DIST_THRESHOLD || !SHTPS_FINGER_ABSORPTION_HOLD_TIME_MS){
		ts->absorption_check_p->absorption_hold_enable = 0;
		return;
	}
	
	if(!ts->absorption_check_p->absorption_hold_enable){
		if(ts->touch_state.numOfFingers == 2){
			for (i = 0; i < fingerMax; i++) {
				if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
					numOfFingers++;
					td_finger = i;

					#if defined(SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE)
						if(numOfFingers <= 2){
							x[numOfFingers-1] = info->fingers[i].x;
							y[numOfFingers-1] = info->fingers[i].y;
						}
					#endif /* SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE */
					
				}else if(info->fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH && 
							ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER)
				{
					tu_finger = i;
				}
			}
			SHTPS_LOG_DBG_PRINT("%s() report fingers = %d, cur fingers = %d", __func__, 
									ts->touch_state.numOfFingers, numOfFingers);

			#if defined(SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE)
				if(numOfFingers == 2){
					ts->absorption_check_p->pre_diff_x = (x[0] >= x[1])? x[0] - x[1] : x[1] - x[0];
					ts->absorption_check_p->pre_diff_y = (y[0] >= y[1])? y[0] - y[1] : y[1] - y[0];
				}
			#endif /* SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE */

			if( (numOfFingers == 1) && (td_finger >= 0) && (tu_finger >= 0) ){
				int diff_x = info->fingers[td_finger].x - ts->report_info.fingers[tu_finger].x;
				int diff_y = info->fingers[td_finger].y - ts->report_info.fingers[tu_finger].y;
				
				diff_x = (diff_x < 0)? -diff_x : diff_x;
				diff_y = (diff_y < 0)? -diff_y : diff_y;

				SHTPS_LOG_DBG_PRINT("%s() [%d]pos1 = (%d, %d), [%d]pos2 = (%d, %d)", __func__, 
										td_finger, info->fingers[td_finger].x, info->fingers[td_finger].y,
										tu_finger, ts->report_info.fingers[tu_finger].x, ts->report_info.fingers[tu_finger].y);
				SHTPS_LOG_DBG_PRINT("%s() diff_x = %d, diff_y = %d", __func__, diff_x, diff_y);
				
				if(diff_x < SHTPS_FINGER_ABSORPTION_DIST_THRESHOLD &&
					diff_y < SHTPS_FINGER_ABSORPTION_DIST_THRESHOLD)
				{
					SHTPS_LOG_DBG_PRINT("%s() start hold", __func__);
					ts->absorption_check_p->absorption_hold_enable = 1;
					ts->absorption_check_p->absorption_hold_tu_finger = tu_finger;
					memcpy(&ts->absorption_check_p->absorption_hold_finger_info, info, sizeof(ts->absorption_check_p->absorption_hold_finger_info));
					
					shtps_filter_absorption_hold_cancel(ts);
					schedule_delayed_work(&ts->absorption_check_p->absorption_hold_off_delayed_work, 
											msecs_to_jiffies(SHTPS_FINGER_ABSORPTION_HOLD_TIME_MS));
				}
				#if defined(SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE)
				else if(!SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_DISABLE){
					if((diff_x > ts->absorption_check_p->pre_diff_x && (diff_x - ts->absorption_check_p->pre_diff_x) > SHTPS_PINCHOUT_DIST_THRESHOLD) ||
					   (diff_y > ts->absorption_check_p->pre_diff_y && (diff_y - ts->absorption_check_p->pre_diff_y) > SHTPS_PINCHOUT_DIST_THRESHOLD))
					{
						SHTPS_LOG_DBG_PRINT("%s() start hold (caused by pinch out)", __func__);
						ts->absorption_check_p->absorption_hold_enable = 1;
						ts->absorption_check_p->absorption_hold_tu_finger = tu_finger;
						memcpy(&ts->absorption_check_p->absorption_hold_finger_info, info, sizeof(ts->absorption_check_p->absorption_hold_finger_info));
						
						shtps_filter_absorption_hold_cancel(ts);
						schedule_delayed_work(&ts->absorption_check_p->absorption_hold_off_delayed_work, 
												msecs_to_jiffies(SHTPS_PINCHOUT_HOLD_TIME_MS));
					}
				}
				#endif /* SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE */
			}
		}
	}else{
		for (i = 0; i < fingerMax; i++) {
			if((info->fingers[i].state != ts->report_info.fingers[i].state && i != ts->absorption_check_p->absorption_hold_tu_finger) ||
			   (info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH && i == ts->absorption_check_p->absorption_hold_tu_finger))
			{
				shtps_filter_absorption_hold_cancel(ts);
				
				SHTPS_LOG_DBG_PRINT("%s() notify hold tu event", __func__);
				
				if(ts->absorption_check_p->absorption_hold_tu_finger < SHTPS_FINGER_MAX){
					shtps_report_touch_off(ts, ts->absorption_check_p->absorption_hold_tu_finger,
								  ts->absorption_check_p->absorption_hold_finger_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger].x,
								  ts->absorption_check_p->absorption_hold_finger_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger].y,
								  0,
								  ts->absorption_check_p->absorption_hold_finger_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger].wx,
								  ts->absorption_check_p->absorption_hold_finger_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger].wy,
								  ts->absorption_check_p->absorption_hold_finger_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger].z);
					
					input_sync(ts->input);
					ts->touch_state.numOfFingers = 1;
					memcpy(&ts->report_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger], 
							&ts->absorption_check_p->absorption_hold_finger_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger], 
							sizeof(ts->report_info.fingers[ts->absorption_check_p->absorption_hold_tu_finger]));
				}

				ts->absorption_check_p->absorption_hold_enable = 0;
				ts->absorption_check_p->absorption_hold_tu_finger = 0;

				break;
			}
		}
	}
	
}


/* -------------------------------------------------------------------------- */
int shtps_filter_absorption_event_report(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int rc;

	rc = ts->absorption_check_p->absorption_hold_enable;
	if(rc){
		int i, fingerMax = shtps_get_fingermax(ts);
		for(i = 0;i < fingerMax;i++){
			SHTPS_LOG_DBG_PRINT("[%s]Drop event[%d] touch=%d(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
							(info->fingers[i].state == SHTPS_TOUCH_STATE_PEN)?    "pen" :
							(info->fingers[i].state == SHTPS_TOUCH_STATE_HOVER)?  "hover" : "finger",
							(info->fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH)? 0 : 100,
							i, info->fingers[i].z, info->fingers[i].x, info->fingers[i].x, 
							info->fingers[i].y, info->fingers[i].y, shtps_get_fingerwidth(ts, i, info), 
							info->fingers[i].wx, info->fingers[i].wy);
		}
	}
	return rc;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_absorption_forcetouchup(struct shtps_rmi_spi *ts)
{
	shtps_filter_absorption_hold_cancel(ts);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_absorption_init(struct shtps_rmi_spi *ts)
{
	ts->absorption_check_p = kzalloc(sizeof(struct shtps_filter_absorption_check_info), GFP_KERNEL);
	if(ts->absorption_check_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	
	// ts->absorption_check_p->absorption_hold_enable = 0;		// no need
	// ts->absorption_check_p->absorption_hold_tu_finger = 0;	// no need
	// memset(&ts->absorption_check_p->absorption_hold_finger_info, 0, sizeof(ts->absorption_check_p->absorption_hold_finger_info)); // no need
	INIT_DELAYED_WORK(&ts->absorption_check_p->absorption_hold_off_delayed_work, shtps_filter_absorption_hold_delayed_work_function);
	ts->absorption_check_p->ts_p = ts;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_absorption_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->absorption_check_p)	kfree(ts->absorption_check_p);
	ts->absorption_check_p = NULL;
}
/* -------------------------------------------------------------------------- */
#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */
