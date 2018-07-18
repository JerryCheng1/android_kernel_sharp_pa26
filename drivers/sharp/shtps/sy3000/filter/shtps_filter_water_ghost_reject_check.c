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
#if defined(SHTPS_WATER_GHOST_REJECTION_ENABLE)
struct shtps_filter_water_ghost_reject_info{
	u8					is_ghost[SHTPS_FINGER_MAX];
	int					count[SHTPS_FINGER_MAX];
	int					threshold[SHTPS_FINGER_MAX];
	unsigned long		tu_time[SHTPS_FINGER_MAX];
	unsigned short		td_pos_x[SHTPS_FINGER_MAX];
	unsigned short		td_pos_y[SHTPS_FINGER_MAX];
	struct delayed_work		ghost_check_delayed_work;
	unsigned long		timer_start_time;
	//
	struct shtps_rmi_spi *ts_p;
};

/* -------------------------------------------------------------------------- */
static void shtps_filter_water_ghost_reject_start_ghost_check_work(struct shtps_rmi_spi *ts)
{
	int i;
	int fingerMax  = shtps_get_fingermax(ts);
	u8 isNeedWork = 0;
	int minTime   = 9999;

	for(i = 0;i < fingerMax;i++){
		if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH &&
			ts->water_ghost_reject_p->is_ghost[i] == 0 && 
			ts->water_ghost_reject_p->count[i] > 0 && 
			ts->water_ghost_reject_p->count[i] < SHTPS_WATER_GHOST_REJECT_COUNT_MAX)
		{
			if(minTime > (SHTPS_WATER_GHOST_REJECT_COUNT_MAX - ts->water_ghost_reject_p->count[i])){
				isNeedWork = 1;
				minTime   = SHTPS_WATER_GHOST_REJECT_COUNT_MAX - ts->water_ghost_reject_p->count[i];
			}
		}
	}
	
	if(isNeedWork){
		ts->water_ghost_reject_p->timer_start_time = jiffies;
		schedule_delayed_work(&ts->water_ghost_reject_p->ghost_check_delayed_work, msecs_to_jiffies(minTime * 17));
	}
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_water_ghost_reject_cancel_ghost_check_work(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_DBG_WATER_GHOST_PRINT("cancel ghost check work\n");
	cancel_delayed_work(&ts->water_ghost_reject_p->ghost_check_delayed_work);
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_water_ghost_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_filter_water_ghost_reject_info *wgr = container_of(dw, struct shtps_filter_water_ghost_reject_info, ghost_check_delayed_work);
//	struct shtps_rmi_spi *ts = container_of(wgr, struct shtps_rmi_spi, water_ghost_reject_p);
	struct shtps_rmi_spi *ts = wgr->ts_p;

	SHTPS_LOG_DBG_WATER_GHOST_PRINT("start ghost check work\n");

	shtps_mutex_lock_ctrl();
	{
		u8 i;
		int fingerMax = shtps_get_fingermax(ts);
		long add_time = jiffies_to_msecs(jiffies - ts->water_ghost_reject_p->timer_start_time) / 17;
		
		if(add_time > 0){
			for(i = 0;i < fingerMax;i++){
				if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH &&
					ts->water_ghost_reject_p->is_ghost[i] == 0 && 
					ts->water_ghost_reject_p->count[i] > 0 && 
					ts->water_ghost_reject_p->count[i] < SHTPS_WATER_GHOST_REJECT_COUNT_MAX)
				{
					if((ts->water_ghost_reject_p->count[i] + add_time) >= SHTPS_WATER_GHOST_REJECT_COUNT_MAX){
						ts->water_ghost_reject_p->count[i]    = SHTPS_WATER_GHOST_REJECT_COUNT_MAX;
						ts->water_ghost_reject_p->is_ghost[i] = 1;
						if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
							shtps_report_touch_on(ts, i,
										  SHTPS_TOUCH_CANCEL_COORDINATES_X,
										  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
										  shtps_get_fingerwidth(ts, i, &ts->report_info),
										  ts->report_info.fingers[i].wx,
										  ts->report_info.fingers[i].wy,
										  ts->report_info.fingers[i].z);
							input_sync(ts->input);
						}
						SHTPS_LOG_DBG_WATER_GHOST_PRINT("<%d> start reject ghost.\n", i);
					}else{
						ts->water_ghost_reject_p->count[i] += add_time;
						SHTPS_LOG_DBG_WATER_GHOST_PRINT("<%d> ghost check work. count inc %d.\n", 
												i, ts->water_ghost_reject_p->count[i]);
					}
				}
			}
		}
		shtps_filter_water_ghost_reject_start_ghost_check_work(ts);
	}
	shtps_mutex_unlock_ctrl();

	SHTPS_LOG_DBG_WATER_GHOST_PRINT("end ghost check work\n");
	
	return;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_water_ghost_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int w, ratio;
	u8 i;
	int fingerMax = shtps_get_fingermax(ts);

	if(!SHTPS_WATER_GHOST_REJECT_ENABLE){
		return;
	}
	
	for(i = 0;i < fingerMax;i++){
		if(ts->water_ghost_reject_p->is_ghost[i] == 0){
			if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
					u8 is_init = 0;
					
					if(ts->water_ghost_reject_p->count[i] == 0){
						is_init = 1;
					}else{
						if(time_after(jiffies, ts->water_ghost_reject_p->tu_time[i] + msecs_to_jiffies(SHTPS_WATER_GHOST_REJECT_COUNT_CLEAR_TIME_THRESH)) != 0){
							SHTPS_LOG_DBG_WATER_GHOST_PRINT("<%d> clear count by tu\n", i);
							is_init = 1;
						}
					}
					
					if(is_init){
						ts->water_ghost_reject_p->count[i]     = 0;
						ts->water_ghost_reject_p->is_ghost[i]  = 0;
						ts->water_ghost_reject_p->td_pos_x[i]  = ts->fw_report_info.fingers[i].x;
						ts->water_ghost_reject_p->td_pos_y[i]  = ts->fw_report_info.fingers[i].y;
						ts->water_ghost_reject_p->threshold[i] = SHTPS_WATER_GHOST_REJECT_GHOST_THRESH;
					}
				}else{
					if(abs(ts->water_ghost_reject_p->td_pos_x[i] - ts->fw_report_info.fingers[i].x) >= SHTPS_WATER_GHOST_REJECT_COUNT_CLEAR_POS_THRESH ||
					   abs(ts->water_ghost_reject_p->td_pos_y[i] - ts->fw_report_info.fingers[i].y) >= SHTPS_WATER_GHOST_REJECT_COUNT_CLEAR_POS_THRESH)
					{
						SHTPS_LOG_DBG_WATER_GHOST_PRINT("<%d> clear count by move. diff(%lu, %lu)\n", 
												i, 
												abs(ts->water_ghost_reject_p->td_pos_x[i] - ts->fw_report_info.fingers[i].x),
												abs(ts->water_ghost_reject_p->td_pos_y[i] - ts->fw_report_info.fingers[i].y));

						ts->water_ghost_reject_p->count[i]     = 0;
						ts->water_ghost_reject_p->td_pos_x[i]  = ts->fw_report_info.fingers[i].x;
						ts->water_ghost_reject_p->td_pos_y[i]  = ts->fw_report_info.fingers[i].y;
						ts->water_ghost_reject_p->threshold[i] = SHTPS_WATER_GHOST_REJECT_GHOST_THRESH;
					}
				}
				
				w = (ts->fw_report_info.fingers[i].wx >= ts->fw_report_info.fingers[i].wy)?
							ts->fw_report_info.fingers[i].wx : ts->fw_report_info.fingers[i].wy;
				ratio = (ts->fw_report_info.fingers[i].z * 100) / ((w > 0)?w:1);

				if(ratio <= SHTPS_WATER_GHOST_REJECT_GHOST_THRESH_CHG_VAL){
					ts->water_ghost_reject_p->threshold[i] = SHTPS_WATER_GHOST_REJECT_GHOST_THRESH_2;
				}
				
				if(ratio <= ts->water_ghost_reject_p->threshold[i]){
					ts->water_ghost_reject_p->count[i]++;
					SHTPS_LOG_DBG_WATER_GHOST_PRINT("<%d> ratio = %d. count inc %d.\n", 
											i, ratio, ts->water_ghost_reject_p->count[i]);
					if(ts->water_ghost_reject_p->count[i] >= SHTPS_WATER_GHOST_REJECT_COUNT_MAX){
						ts->water_ghost_reject_p->is_ghost[i] = 1;
						info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
	
						if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
							shtps_report_touch_on(ts, i,
										  SHTPS_TOUCH_CANCEL_COORDINATES_X,
										  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
										  shtps_get_fingerwidth(ts, i, &ts->report_info),
										  ts->report_info.fingers[i].wx,
										  ts->report_info.fingers[i].wy,
										  ts->report_info.fingers[i].z);
							input_sync(ts->input);
						}
						SHTPS_LOG_DBG_WATER_GHOST_PRINT("<%d> start reject ghost.\n", i);
					}
				}
			}else{
				ts->water_ghost_reject_p->tu_time[i] = jiffies;
			}
		}else {
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				ts->water_ghost_reject_p->is_ghost[i] = 0;
				ts->water_ghost_reject_p->count[i]    = 0;
			}else if(ts->water_ghost_reject_p->is_ghost[i] == 1){
				w = (ts->fw_report_info.fingers[i].wx >= ts->fw_report_info.fingers[i].wy)?
							ts->fw_report_info.fingers[i].wx : ts->fw_report_info.fingers[i].wy;
				ratio = (ts->fw_report_info.fingers[i].z * 100) / ((w > 0)?w:1);
				
				if(ratio < SHTPS_WATER_GHOST_REJECT_RETURN_THRESH){
					if(abs(ts->water_ghost_reject_p->td_pos_x[i] - ts->fw_report_info.fingers[i].x) >= SHTPS_WATER_GHOST_REJECT_COUNT_CLEAR_POS_THRESH ||
					   abs(ts->water_ghost_reject_p->td_pos_y[i] - ts->fw_report_info.fingers[i].y) >= SHTPS_WATER_GHOST_REJECT_COUNT_CLEAR_POS_THRESH)
					{
						ts->water_ghost_reject_p->is_ghost[i] = 2;
						ts->water_ghost_reject_p->count[i]    = 0;
						SHTPS_LOG_DBG_WATER_GHOST_PRINT("<%d> exit reject ghost by move.\n", i);
					}else{
						info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
						SHTPS_LOG_DBG_WATER_GHOST_PRINT("<%d> reject ghost.\n", i);
					}
				}else{
					ts->water_ghost_reject_p->is_ghost[i] = 2;
					ts->water_ghost_reject_p->count[i]    = 0;
					SHTPS_LOG_DBG_WATER_GHOST_PRINT("<%d> exit reject ghost by z/w value(%d).\n", i, ratio);
				}
			}
		}
	}
	
	shtps_filter_water_ghost_reject_start_ghost_check_work(ts);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_water_ghost_reject_forcetouchup(struct shtps_rmi_spi *ts)
{
	shtps_filter_water_ghost_reject_cancel_ghost_check_work(ts);
	memset(ts->water_ghost_reject_p->is_ghost, 0, sizeof(ts->water_ghost_reject_p->is_ghost));
	memset(ts->water_ghost_reject_p->count, 0, sizeof(ts->water_ghost_reject_p->count));
}

/* -------------------------------------------------------------------------- */
void shtps_filter_water_ghost_reject_init(struct shtps_rmi_spi *ts)
{
	ts->water_ghost_reject_p = kzalloc(sizeof(struct shtps_filter_water_ghost_reject_info), GFP_KERNEL);
	if(ts->water_ghost_reject_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(ts->water_ghost_reject_p, 0, sizeof(struct shtps_filter_water_ghost_reject_info));	// no need
	INIT_DELAYED_WORK(&ts->water_ghost_reject_p->ghost_check_delayed_work, shtps_filter_water_ghost_delayed_work_function);
	ts->water_ghost_reject_p->ts_p = ts;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_water_ghost_reject_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->water_ghost_reject_p)	kfree(ts->water_ghost_reject_p);
	ts->water_ghost_reject_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_WATER_GHOST_REJECTION_ENABLE */
