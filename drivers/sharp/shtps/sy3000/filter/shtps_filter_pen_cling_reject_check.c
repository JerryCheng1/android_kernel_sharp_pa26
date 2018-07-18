/* drivers/sharp/shtps/sy3000/shtps_filter_pen_cling_reject_check.c
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
#if defined( SHTPS_PEN_CLING_REJECTION_ENABLE )
enum{
	SHTPS_PEN_CLING_REJECT_STATE_IDLE     = 0,
	SHTPS_PEN_CLING_REJECT_STATE_CHECKING = 1,
	SHTPS_PEN_CLING_REJECT_STATE_DETECTED = 2,
};

struct shtps_filter_pen_cling_reject_info{
	u8								main_state;
	u8								state[SHTPS_FINGER_MAX];
	struct shtps_touch_pos_info		base_pos[SHTPS_FINGER_MAX];
	unsigned long					time[SHTPS_FINGER_MAX];
	unsigned long					tu_time[SHTPS_FINGER_MAX];
	struct delayed_work				delayed_work;
	//
	struct shtps_rmi_spi			*ts_p;
};

/* -------------------------------------------------------------------------- */
static void shtps_filter_pen_cling_reject_timer_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_PEN_CLING_REJECT_PRINT("check timer stop\n");
	cancel_delayed_work(&ts->pen_cling_reject_p->delayed_work);
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_pen_cling_reject_timer_start(struct shtps_rmi_spi *ts, unsigned long time)
{
	SHTPS_PEN_CLING_REJECT_PRINT("check timer start (%lu ms)\n", time);
	schedule_delayed_work(&ts->pen_cling_reject_p->delayed_work, time);
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_pen_cling_reject_clear(struct shtps_rmi_spi *ts)
{
	shtps_filter_pen_cling_reject_timer_stop(ts);
	ts->pen_cling_reject_p->main_state = SHTPS_PEN_CLING_REJECT_STATE_IDLE;
	memset(ts->pen_cling_reject_p->state, 0, sizeof(ts->pen_cling_reject_p->state));
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_pen_cling_reject_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
//	struct shtps_filter_pen_cling_reject *spcr = container_of(dw, struct shtps_filter_pen_cling_reject, delayed_work);
//	struct shtps_rmi_spi *ts = container_of(spcr, struct shtps_rmi_spi, pen_cling_reject);
	struct shtps_filter_pen_cling_reject_info	*spcr = container_of(dw, struct shtps_filter_pen_cling_reject_info, delayed_work);
	struct shtps_rmi_spi *ts = spcr->ts_p;

	SHTPS_PEN_CLING_REJECT_PRINT("start check work\n");

	shtps_mutex_lock_ctrl();
	shtps_read_touchevent(ts, SHTPS_STATE_ACTIVE);
	shtps_mutex_unlock_ctrl();
}

/* -------------------------------------------------------------------------- */
int shtps_filter_pen_cling_reject_check2(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int cling_detect = 0;
	unsigned long minTime = 0xFFFFFFFF;
	int fingerMax = shtps_get_fingermax(ts);
	int numOfFingers = 0;
	int timerStart = 0;
	
	if(!SHTPS_PEN_CLING_REJECT_ENABLE){
		return 0;
	}
	
	shtps_filter_pen_cling_reject_timer_stop(ts);
	
	if(ts->pen_cling_reject_p->main_state != SHTPS_PEN_CLING_REJECT_STATE_DETECTED){
		for(i = 0;i < fingerMax;i++){
			if(ts->pen_cling_reject_p->state[i] == SHTPS_PEN_CLING_REJECT_STATE_IDLE){
				if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN &&
					ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH &&
					(ts->fw_report_info.fingers[i].x <= SHTPS_PEN_CLING_REJECT_AREA ||
					 ts->fw_report_info.fingers[i].x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_PEN_CLING_REJECT_AREA) ||
					 ts->fw_report_info.fingers[i].y <= SHTPS_PEN_CLING_REJECT_AREA ||
					 ts->fw_report_info.fingers[i].y >= (CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1 - SHTPS_PEN_CLING_REJECT_AREA)))
				{
					ts->pen_cling_reject_p->state[i]      = SHTPS_PEN_CLING_REJECT_STATE_CHECKING;
					ts->pen_cling_reject_p->time[i]       = jiffies;
					ts->pen_cling_reject_p->base_pos[i].x = ts->fw_report_info.fingers[i].x;
					ts->pen_cling_reject_p->base_pos[i].y = ts->fw_report_info.fingers[i].y;
				}
			}else if(ts->pen_cling_reject_p->state[i] == SHTPS_PEN_CLING_REJECT_STATE_CHECKING){
				if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN){
					unsigned long diff_x = abs(ts->fw_report_info.fingers[i].x - ts->pen_cling_reject_p->base_pos[i].x);
					unsigned long diff_y = abs(ts->fw_report_info.fingers[i].y - ts->pen_cling_reject_p->base_pos[i].y);
					
					if(diff_x > SHTPS_PEN_CLING_REJECT_POS_THRESH || diff_y > SHTPS_PEN_CLING_REJECT_POS_THRESH){
						ts->pen_cling_reject_p->state[i] = SHTPS_PEN_CLING_REJECT_STATE_IDLE;
						SHTPS_PEN_CLING_REJECT_PRINT("[%d] check stop by move (%lu, %lu)\n", i, diff_x, diff_y);
					}
				}else if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
					if(ts->fw_report_info_store.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH)
					{
						ts->pen_cling_reject_p->tu_time[i] = jiffies;
					}else{
						if(time_after(jiffies, ts->pen_cling_reject_p->tu_time[i] + msecs_to_jiffies(SHTPS_PEN_CLING_REJECT_TU_TIME_THRESH)) != 0){
							ts->pen_cling_reject_p->state[i] = SHTPS_PEN_CLING_REJECT_STATE_IDLE;
							SHTPS_PEN_CLING_REJECT_PRINT("[%d] check stop by touch-up\n", i);
						}
					}
				}else{
					ts->pen_cling_reject_p->state[i] = SHTPS_PEN_CLING_REJECT_STATE_IDLE;
					SHTPS_PEN_CLING_REJECT_PRINT("[%d] check stop by tool-type\n", i);
				}
			}
		}

		for(i = 0;i < fingerMax;i++){
			if(ts->pen_cling_reject_p->state[i] == SHTPS_PEN_CLING_REJECT_STATE_CHECKING){
				if(time_after(jiffies, ts->pen_cling_reject_p->time[i] + msecs_to_jiffies(SHTPS_PEN_CLING_REJECT_TIME_THRESH)) != 0){
					SHTPS_PEN_CLING_REJECT_PRINT("[%d] detect cling\n", i);
					ts->pen_cling_reject_p->state[i]   = SHTPS_PEN_CLING_REJECT_STATE_DETECTED;
					ts->pen_cling_reject_p->main_state = SHTPS_PEN_CLING_REJECT_STATE_DETECTED;
				}else{
					unsigned long time = (ts->pen_cling_reject_p->time[i] + msecs_to_jiffies(SHTPS_PEN_CLING_REJECT_TIME_THRESH)) - jiffies;
					if(timerStart == 0 || minTime >= time){
						minTime = time;
					}
					timerStart = 1;
					
					if(time == 0){
						SHTPS_PEN_CLING_REJECT_PRINT("[%d] detect cling\n", i);
						ts->pen_cling_reject_p->state[i]   = SHTPS_PEN_CLING_REJECT_STATE_DETECTED;
						ts->pen_cling_reject_p->main_state = SHTPS_PEN_CLING_REJECT_STATE_DETECTED;
					}
				}
			}
		}
	}
	
	if(ts->pen_cling_reject_p->main_state == SHTPS_PEN_CLING_REJECT_STATE_DETECTED){
		for(i = 0;i < fingerMax;i++){
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER ||
				(ts->pen_cling_reject_p->state[i] != SHTPS_PEN_CLING_REJECT_STATE_DETECTED &&
				 ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH))
			{
				numOfFingers++;
			}
		}
		
		if(numOfFingers == 0){
			SHTPS_PEN_CLING_REJECT_PRINT("force cal execute\n");
			shtps_filter_pen_cling_reject_clear(ts);
			shtps_rezero(ts);
			
			for(i = 0;i < fingerMax;i++){
				ts->fw_report_info.fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
				info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			}
		}
	}else{
		if(timerStart){
			shtps_filter_pen_cling_reject_timer_start(ts, minTime);
		}
	}
	
	return cling_detect;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pen_cling_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	shtps_filter_pen_cling_reject_check2(ts, info);
}
/* -------------------------------------------------------------------------- */
void shtps_filter_pen_cling_reject_forcetouchup(struct shtps_rmi_spi *ts)
{
	shtps_filter_pen_cling_reject_clear(ts);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pen_cling_reject_init(struct shtps_rmi_spi *ts)
{
	ts->pen_cling_reject_p = kzalloc(sizeof(struct shtps_filter_pen_cling_reject_info), GFP_KERNEL);
	if(ts->pen_cling_reject_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}

	INIT_DELAYED_WORK(&ts->pen_cling_reject_p->delayed_work, shtps_filter_pen_cling_reject_delayed_work_function);
	shtps_filter_pen_cling_reject_clear(ts);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pen_cling_reject_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->pen_cling_reject_p)	kfree(ts->pen_cling_reject_p);
	ts->pen_cling_reject_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_PEN_CLING_REJECTION_ENABLE */

