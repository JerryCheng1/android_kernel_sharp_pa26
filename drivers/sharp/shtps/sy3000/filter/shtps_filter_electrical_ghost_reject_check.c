/* drivers/sharp/shtps/sy3000/shtps_filter_electrical_ghost_reject_check.c
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
#include <linux/time.h>


#include <sharp/shtps_dev.h>

#include "shtps_rmi.h"
#include "shtps_param_extern.h"
#include "shtps_log.h"

#include "shtps_filter.h"
/* -------------------------------------------------------------------------- */
#if defined(SHTPS_ELECTRICAL_GHOST_REJECTION_ENABLE)
struct shtps_filter_electrical_ghost_reject_info{
	unsigned long				td_time;
	struct shtps_touch_info		hold_finger_info;
	struct timeval				td_timeval;
	int							hold_flg;

};

/* -------------------------------------------------------------------------- */
static int shtps_filter_electrical_ghost_reject_area_check(unsigned short x, unsigned short y)
{
	int area = 0;

	if(y <= SHTPS_ELECTRICAL_GHOST_REJECT_AREA_THRESH_Y ||
	   y >= (CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1 - SHTPS_ELECTRICAL_GHOST_REJECT_AREA_THRESH_Y))
	{
		area = 1;
		SHTPS_LOG_DBG_ELECTRICAL_GHOST_PRINT("check area x[%d], y[%d]", x, y);

	}else if(x <= SHTPS_ELECTRICAL_GHOST_REJECT_AREA_THRESH_X ||
			 x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_ELECTRICAL_GHOST_REJECT_AREA_THRESH_X))
	{
		area = 1;
		SHTPS_LOG_DBG_ELECTRICAL_GHOST_PRINT("check area x[%d], y[%d]", x, y);
	}
	return area;
}

/* -------------------------------------------------------------------------- */
static int shtps_filter_electrical_ghost_reject_check_skip_event(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	struct timeval nowtime;
	int i = 0;
	int fingerMax = shtps_get_fingermax(ts);
	int fingerNum = 0;
	int ret = 0;
	unsigned int t_now = 0;
	unsigned int t_tdtime = 0;
	
	
	do_gettimeofday(&nowtime);
	t_now 		= nowtime.tv_sec * 1000 + nowtime.tv_usec / 1000;
	t_tdtime	= ts->electrical_ghost_reject_p->td_timeval.tv_sec * 1000 + ts->electrical_ghost_reject_p->td_timeval.tv_usec / 1000;
	SHTPS_LOG_DBG_ELECTRICAL_GHOST_PRINT(" t_now [%d], t_tdtime [%d] ", t_now, t_tdtime);
	
	if((t_tdtime + SHTPS_ELECTRICAL_GHOST_REJECT_TIMER_THRESH) >= t_now){
		for(i = 0; i < fingerMax; i++){
			if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				fingerNum++;
			}
		}
		if(fingerNum == 0){
			ret = 1;
		}else{
			SHTPS_LOG_DBG_ELECTRICAL_GHOST_PRINT("not all_touch_up. fingerNum[%d]", fingerNum);
		}
	}else{
		SHTPS_LOG_DBG_ELECTRICAL_GHOST_PRINT("pending time over");
	}
	
	return ret;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_electrical_ghost_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int check_ghost = 0;
	int fingerMax = shtps_get_fingermax(ts);
	
	if(!SHTPS_ELECTRICAL_GHOST_REJECT_ENABLE){
		return;
	}
	
	if(ts->electrical_ghost_reject_p->hold_flg == 0){
		for(i = 0; i < fingerMax; i++){
			if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
					if((shtps_filter_electrical_ghost_reject_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y) == 1) || 
					   (ts->fw_report_info.fingers[i].wx == 0 || ts->fw_report_info.fingers[i].wy == 0) ||
					   ((ts->fw_report_info.fingers[i].wx >= SHTPS_ELECTRICAL_GHOST_REJECT_W_THRESH_X) || (ts->fw_report_info.fingers[i].wy >= SHTPS_ELECTRICAL_GHOST_REJECT_W_THRESH_Y))
					){
						check_ghost = 1;
					}
				}
			}else{
				SHTPS_LOG_DBG_ELECTRICAL_GHOST_PRINT("not ghost. (report_info state[%d])", ts->report_info.fingers[i].state);
				check_ghost = 0;
				break;
			}
		}
	}
	
	if(ts->electrical_ghost_reject_p->hold_flg == 1){
		
		if(shtps_filter_electrical_ghost_reject_check_skip_event(ts, info) == 1){
			
			SHTPS_LOG_DBG_ELECTRICAL_GHOST_PRINT("ignore event.");
			
		}else{
			for(i = 0; i < fingerMax; i++){
				if(ts->electrical_ghost_reject_p->hold_finger_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
					shtps_report_touch_on(ts, i,
										  ts->electrical_ghost_reject_p->hold_finger_info.fingers[i].x,
										  ts->electrical_ghost_reject_p->hold_finger_info.fingers[i].y,
										  shtps_get_fingerwidth(ts, i, &ts->electrical_ghost_reject_p->hold_finger_info),
										  ts->electrical_ghost_reject_p->hold_finger_info.fingers[i].wx,
										  ts->electrical_ghost_reject_p->hold_finger_info.fingers[i].wy,
										  ts->electrical_ghost_reject_p->hold_finger_info.fingers[i].z);
					input_sync(ts->input);
					memcpy(&ts->report_info.fingers[i], 
								&ts->electrical_ghost_reject_p->hold_finger_info.fingers[i], sizeof(ts->electrical_ghost_reject_p->hold_finger_info.fingers[i]));

					SHTPS_LOG_DBG_ELECTRICAL_GHOST_PRINT("Notify pending event [%d] x=%d, y=%d, wx=%d, wy=%d, z=%d\n", 
										i,
										ts->electrical_ghost_reject_p->hold_finger_info.fingers[i].x,
										ts->electrical_ghost_reject_p->hold_finger_info.fingers[i].y,
										ts->electrical_ghost_reject_p->hold_finger_info.fingers[i].wx,
										ts->electrical_ghost_reject_p->hold_finger_info.fingers[i].wy,
										ts->electrical_ghost_reject_p->hold_finger_info.fingers[i].z);
				}
			}
		}
		memset(&ts->electrical_ghost_reject_p->hold_finger_info, 0, sizeof(ts->electrical_ghost_reject_p->hold_finger_info));
		ts->electrical_ghost_reject_p->hold_flg = 0;
	}
	
	if(check_ghost == 1){
		for(i = 0; i < fingerMax; i++){
			if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				memcpy(&ts->electrical_ghost_reject_p->hold_finger_info.fingers[i], &info->fingers[i], sizeof(ts->electrical_ghost_reject_p->hold_finger_info.fingers[i]));
				info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			}
		}
		do_gettimeofday(&ts->electrical_ghost_reject_p->td_timeval);
		ts->electrical_ghost_reject_p->hold_flg = 1;
		SHTPS_LOG_DBG_ELECTRICAL_GHOST_PRINT("pending event.");
	}
	return;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_electrical_ghost_forcetouchup(struct shtps_rmi_spi *ts)
{
	memset(ts->electrical_ghost_reject_p, 0, sizeof(struct shtps_filter_electrical_ghost_reject_info));
}

/* -------------------------------------------------------------------------- */
void shtps_filter_electrical_ghost_reject_init(struct shtps_rmi_spi *ts)
{
	ts->electrical_ghost_reject_p = kzalloc(sizeof(struct shtps_filter_electrical_ghost_reject_info), GFP_KERNEL);
	if(ts->electrical_ghost_reject_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	ts->electrical_ghost_reject_p->hold_flg = 0;
	// memset(ts->electrical_ghost_reject_p, 0, sizeof(struct shtps_filter_electrical_ghost_reject_info));	// no need
}

/* -------------------------------------------------------------------------- */
void shtps_filter_electrical_ghost_reject_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->electrical_ghost_reject_p)	kfree(ts->electrical_ghost_reject_p);
	ts->electrical_ghost_reject_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_ELECTRICAL_GHOST_REJECTION_ENABLE */
