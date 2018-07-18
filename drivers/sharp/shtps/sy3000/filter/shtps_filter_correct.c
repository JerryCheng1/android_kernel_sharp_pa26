/* drivers/sharp/shtps/sy3000/shtps_filter_correct.c
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
struct shtps_hover_hist{
	int							x;
	int							y;
	unsigned long				time;
};

struct shtps_hold_info{	
	unsigned short	x;
	unsigned short	y;
	unsigned char	wx;
	unsigned char	wy;
	unsigned char	z;
};

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE) || defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE) || defined(SHTPS_HOVER_REJECT_ENABLE)
struct shtps_filter_correct_info{
	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE)
		struct delayed_work			hover_touch_up_delayed_work;
		u8							hover_touch_up_delayed_finger;
		struct shtps_hover_hist		hover_hist[SHTPS_HOVER_HIST_MAX];
		struct shtps_hover_hist		hover_center_hist[SHTPS_HOVER_CENTER_HIST_MAX];
		u8							hover_hist_count;
		u8							hover_center_hist_count;
		u8							hover_ignore_touch_info;
		u8							hover_invalid_touch_info;
	#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE */

	#if defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE)
		u8							pen_z_hist[SHTPS_FINGER_MAX][SHTPS_PEN_Z_HIST_MAX];
		u8							pen_z_hist_count[SHTPS_FINGER_MAX];
		u8							pen_event_fail_cont_reject_chattering_enable[SHTPS_FINGER_MAX];
		unsigned long				pen_event_fail_cont_reject_chattering_time_max[SHTPS_FINGER_MAX];
		u8							pen_z_hist_2nd[SHTPS_FINGER_MAX][SHTPS_PEN_Z_HIST_MAX];
		u8							pen_z_hist_count_2nd[SHTPS_FINGER_MAX];
		u8							pen_z_dummy_tu_min[SHTPS_FINGER_MAX];
		u8							pen_event_fail_cont_reject_chattering_2nd_enable[SHTPS_FINGER_MAX];
		u16							pen_event_fail_cont_reject_chattering_2nd_count[SHTPS_FINGER_MAX];
		struct shtps_hold_info		pen_event_fail_cont_reject_hold_info[SHTPS_FINGER_MAX][SHTPS_PEN_FAIL_CONT_HOLD_INFO_NUM];
		u8							pen_event_fail_cont_reject_dummy_tu_state[SHTPS_FINGER_MAX];
	#endif /* SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE */

	#if defined(SHTPS_HOVER_REJECT_ENABLE)
		u16							is_hover_finger;
		u16							is_force_hover_finger;
		u8							tu_check_enable[SHTPS_FINGER_MAX];
		u32							tu_time[SHTPS_FINGER_MAX];
		u16							tu_pos[SHTPS_FINGER_MAX][2];
		u8							finger_tu_pen_ignore[SHTPS_FINGER_MAX];
		u8							finger_tu_finger_ignore_count[SHTPS_FINGER_MAX];
		struct shtps_touch_info		finger_tu_finger_ignore_td_info;
		u8							finger_tu_finger_ignore_td_check_enable[SHTPS_FINGER_MAX];
		u32							finger_tu_finger_ignore_tu_time[SHTPS_FINGER_MAX];
		u32							finger_tu_finger_ignore_td_time[SHTPS_FINGER_MAX];
		u16							finger_tu_finger_ignore_tu_pos[SHTPS_FINGER_MAX][2];
		u8							finger_tu_finger_ignore_td_cont_check_enable[SHTPS_FINGER_MAX];
		struct shtps_touch_info		hover_reject_zero_info;
		unsigned long				hover_reject_event_drop_time_max[SHTPS_FINGER_MAX];
		struct delayed_work			read_touchevent_delayed_work;
		u8							read_touchevent_delayed_enable;
		u8							hover_reject_pen_area[SHTPS_FINGER_MAX];
		u8							hover_reject_pen_chatt_cnt[SHTPS_FINGER_MAX];
		u8							hover_reject_pen_chatt_z_total[SHTPS_FINGER_MAX];
		u8							hover_reject_pen_td_chatt_state[SHTPS_FINGER_MAX];
		u8							hover_reject_pen_td_move_check_state[SHTPS_FINGER_MAX];
		u8							hover_reject_pen_pending_cnt[SHTPS_FINGER_MAX];
		struct shtps_touch_info		hover_reject_pen_pending_event[5];
	/*	struct shtps_touch_info		hover_reject_pen_chatt_pending;	*/	//AAAAA
	#endif /* SHTPS_HOVER_REJECT_ENABLE */

	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE) || defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE) || defined(SHTPS_HOVER_REJECT_ENABLE)
		struct shtps_rmi_spi *ts_p;
	#endif
};
#endif /* defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE) || defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE) || defined(SHTPS_HOVER_REJECT_ENABLE) */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE)
static int shtps_hover_tu_timer_start(struct shtps_rmi_spi *ts, unsigned long delay_ms)
{
	SHTPS_LOG_FUNC_CALL_INPARAM((int)delay_ms);

	cancel_delayed_work(&ts->filter_correct_p->hover_touch_up_delayed_work);
	schedule_delayed_work(&ts->filter_correct_p->hover_touch_up_delayed_work, msecs_to_jiffies(delay_ms));

	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_hover_tu_timer_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->filter_correct_p->hover_touch_up_delayed_work);

	return 0;
}

/* -------------------------------------------------------------------------- */
static void shtps_hover_touch_up_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_filter_correct_info *fc = container_of(dw, struct shtps_filter_correct_info, hover_touch_up_delayed_work);
	//struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, read_touchevent_delayed_work);
	struct shtps_rmi_spi *ts = fc->ts_p;
	int fingerMax = shtps_get_fingermax(ts);
	int i;
	int isEvent = 0;

	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_ctrl();

	for(i = 0; i < fingerMax; i++){
		if( (ts->filter_correct_p->hover_touch_up_delayed_finger & (1 << i)) != 0 ){
			if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
				isEvent = 1;
				shtps_report_touch_hover_off(ts, i,
									  ts->report_info.fingers[i].x,
									  ts->report_info.fingers[i].y,
									  shtps_get_fingerwidth(ts, i, &ts->report_info),
									  ts->report_info.fingers[i].wx,
									  ts->report_info.fingers[i].wy,
									  ts->report_info.fingers[i].z);

				ts->report_info.fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			}
		}
	}

	if(isEvent){
		input_sync(ts->input);

		ts->filter_correct_p->hover_touch_up_delayed_finger = 0x00;
		ts->filter_correct_p->hover_hist_count = 0;
		ts->filter_correct_p->hover_center_hist_count = 0;
	}

	shtps_mutex_unlock_ctrl();

	return;
}
#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE */
/* -------------------------------------------------------------------------- */
#if defined( SHTPS_HOVER_REJECT_ENABLE )
static u8 shtps_hover_reject_area_check(int x, int y)
{
	u8 is_hover_reject_area = 0;

	if( (x <= SHTPS_HOVER_REJECT_EDGE_AREA) ||
		(x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_HOVER_REJECT_EDGE_AREA)) ||
		(y <= SHTPS_HOVER_REJECT_EDGE_AREA) ||
		(y >= (CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1 - SHTPS_HOVER_REJECT_EDGE_AREA)) )
	{
		is_hover_reject_area = 1;
	}

	return is_hover_reject_area;
}

/* -------------------------------------------------------------------------- */
static void shtps_hover_reject_pen_add_pend_event(struct shtps_rmi_spi *ts, u8 id, struct shtps_touch_info *info)
{
	u8 idx = ts->filter_correct_p->hover_reject_pen_pending_cnt[id];
	
	if(idx >= (sizeof(ts->filter_correct_p->hover_reject_pen_pending_event) / sizeof(struct shtps_touch_info))){
		SHTPS_LOG_HOVER_REJECT("<%d> pending buffer is overflow. (idx = %d, size=%d)\n", id,
				idx,(sizeof(ts->filter_correct_p->hover_reject_pen_pending_event) / sizeof(struct shtps_touch_info)));
		return;
	}
	
	memcpy(&ts->filter_correct_p->hover_reject_pen_pending_event[idx].fingers[id], &info->fingers[id], 
								sizeof(ts->filter_correct_p->hover_reject_pen_pending_event[idx].fingers[id]));
	ts->filter_correct_p->hover_reject_pen_pending_cnt[id]++;
}

/* -------------------------------------------------------------------------- */
static void shtps_hover_reject_pen_clear_pend_event(struct shtps_rmi_spi *ts, u8 id)
{
	ts->filter_correct_p->hover_reject_pen_pending_cnt[id] = 0;
}

/* -------------------------------------------------------------------------- */
static void shtps_hover_reject_pen_notify_pend_event(struct shtps_rmi_spi *ts, u8 id)
{
	u8 i;
	
	if(ts->filter_correct_p->hover_reject_pen_pending_cnt[id] == 0){
		return;
	}
	
	SHTPS_LOG_HOVER_REJECT("<%d> notify pending events\n", id);
	for(i = 0;i < ts->filter_correct_p->hover_reject_pen_pending_cnt[id];i++){
		shtps_report_touch_pen_on(ts, id,
							  ts->filter_correct_p->hover_reject_pen_pending_event[i].fingers[id].x,
							  ts->filter_correct_p->hover_reject_pen_pending_event[i].fingers[id].y,
							  shtps_get_fingerwidth(ts, id, &ts->filter_correct_p->hover_reject_pen_pending_event[i]),
							  ts->filter_correct_p->hover_reject_pen_pending_event[i].fingers[id].wx,
							  ts->filter_correct_p->hover_reject_pen_pending_event[i].fingers[id].wy,
							  ts->filter_correct_p->hover_reject_pen_pending_event[i].fingers[id].z);
		input_sync(ts->input);
	}
	
	if(ts->report_info.fingers[id].state == SHTPS_TOUCH_STATE_NO_TOUCH){
		ts->touch_state.numOfFingers++;
	}

	ts->report_info.fingers[id].state = SHTPS_TOUCH_STATE_PEN;
	ts->report_info.fingers[id].x     = ts->filter_correct_p->hover_reject_pen_pending_event[i-1].fingers[id].x;
	ts->report_info.fingers[id].y     = ts->filter_correct_p->hover_reject_pen_pending_event[i-1].fingers[id].y;
	ts->report_info.fingers[id].wx    = ts->filter_correct_p->hover_reject_pen_pending_event[i-1].fingers[id].wx;
	ts->report_info.fingers[id].wy    = ts->filter_correct_p->hover_reject_pen_pending_event[i-1].fingers[id].wy;
	ts->report_info.fingers[id].z     = ts->filter_correct_p->hover_reject_pen_pending_event[i-1].fingers[id].z;
}

/* -------------------------------------------------------------------------- */
static int shtps_read_touchevent_timer_start(struct shtps_rmi_spi *ts, unsigned long delay_ms)
{
	SHTPS_LOG_FUNC_CALL_INPARAM((int)delay_ms);

	cancel_delayed_work(&ts->filter_correct_p->read_touchevent_delayed_work);
	schedule_delayed_work(&ts->filter_correct_p->read_touchevent_delayed_work, msecs_to_jiffies(delay_ms));

	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_read_touchevent_timer_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->filter_correct_p->read_touchevent_delayed_work);

	return 0;
}

/* -------------------------------------------------------------------------- */
static void shtps_read_touchevent_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_filter_correct_info *fc = container_of(dw, struct shtps_filter_correct_info, read_touchevent_delayed_work);
	//struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, read_touchevent_delayed_work);
	struct shtps_rmi_spi *ts = fc->ts_p;

	shtps_mutex_lock_ctrl();

	if(ts->filter_correct_p->read_touchevent_delayed_enable != 0){
		if(ts->state_mgr.state == SHTPS_STATE_ACTIVE){
			shtps_read_touchevent(ts, SHTPS_STATE_ACTIVE);
		}else{
			ts->filter_correct_p->read_touchevent_delayed_enable = 0;
		}
	}

	shtps_mutex_unlock_ctrl();
}
#endif /* SHTPS_HOVER_REJECT_ENABLE */

/* -------------------------------------------------------------------------- */
void shtps_filter_ignore_touch_info_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE)
	{
		int i;
		int fingerMax = shtps_get_fingermax(ts);

		for(i = 0; i < fingerMax; i++){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_HOVER)
			{
				if( ((SHTPS_HOVER_IGNORE_WX_MIN <= info->fingers[i].wx) && (info->fingers[i].wx <= SHTPS_HOVER_IGNORE_WX_MAX)) ||
					((SHTPS_HOVER_IGNORE_WY_MIN <= info->fingers[i].wy) && (info->fingers[i].wy <= SHTPS_HOVER_IGNORE_WY_MAX)) )
				{
					if(hover_debug_log_enable & 0x01){
						SHTPS_LOG_DBG_PRINT("[hover][%d] ignored by wx/wy value\n", i);
					}

					ts->filter_correct_p->hover_ignore_touch_info |= (1 << i);

					info->fingers[i].state	= ts->report_info.fingers[i].state;
					info->fingers[i].x		= ts->report_info.fingers[i].x;
					info->fingers[i].y		= ts->report_info.fingers[i].y;
					info->fingers[i].wx		= ts->report_info.fingers[i].wx;
					info->fingers[i].wy		= ts->report_info.fingers[i].wy;
					info->fingers[i].z		= ts->report_info.fingers[i].z;
				}
			}
		}
	}
	#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE */

	#if defined( SHTPS_HOVER_REJECT_ENABLE )
	if(SHTPS_HOST_HOVER_DETECT_REJECTION_ENABLE != 0)
	{
		int i;
		int fingerMax = shtps_get_fingermax(ts);
		u8 pen_chatt_cnt;
		u8 pen_chatt_zth_cond;
		u8 pen_chatt_zth_cond2;
		u8 pen_chatt_zth_min;
		u8 pen_chatt_zth_max;
		u8 pen_chatt_tap_cnt;

		for(i = 0; i < fingerMax; i++){
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN &&
				ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH)
			{
				ts->filter_correct_p->hover_reject_pen_area[i] = shtps_hover_reject_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);
			}
			
			if( ts->filter_correct_p->hover_reject_pen_area[i] != 0){
				pen_chatt_cnt		= SHTPS_HOVER_REJECT_1_PEN_CHATT_CNT;
				pen_chatt_zth_cond	= 0;
				pen_chatt_zth_cond2	= 0;
				pen_chatt_zth_min	= SHTPS_HOVER_REJECT_1_PEN_CHATT_Z_THRESHOLD_MIN;
				pen_chatt_zth_max	= SHTPS_HOVER_REJECT_1_PEN_CHATT_Z_THRESHOLD_MAX;
				pen_chatt_tap_cnt	= SHTPS_HOVER_REJECT_1_PEN_CHATT_TAP_CNT;
			}else{
				pen_chatt_cnt		= SHTPS_HOVER_REJECT_6_PEN_CHATT_CNT;
				pen_chatt_zth_cond	= SHTPS_HOVER_REJECT_6_PEN_CHATT_Z_THRESHOLD_COND;
				pen_chatt_zth_cond2	= SHTPS_HOVER_REJECT_6_PEN_CHATT_Z_THRESHOLD_COND_2;
				pen_chatt_zth_min	= SHTPS_HOVER_REJECT_6_PEN_CHATT_Z_THRESHOLD_MIN;
				pen_chatt_zth_max	= SHTPS_HOVER_REJECT_6_PEN_CHATT_Z_THRESHOLD_MAX;
				pen_chatt_tap_cnt	= SHTPS_HOVER_REJECT_6_PEN_CHATT_TAP_CNT;
			}

			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				if(ts->filter_correct_p->hover_reject_pen_chatt_cnt[i] > 0 &&
					ts->filter_correct_p->hover_reject_pen_chatt_cnt[i] < pen_chatt_tap_cnt)
				{
					shtps_hover_reject_pen_notify_pend_event(ts, i);
				}

				ts->filter_correct_p->is_hover_finger &= ~(1 << i);
				ts->filter_correct_p->is_force_hover_finger &= ~(1 << i);
				ts->filter_correct_p->hover_reject_pen_chatt_cnt[i] = 0;
				ts->filter_correct_p->hover_reject_pen_chatt_z_total[i] = 0;
				ts->filter_correct_p->hover_reject_pen_td_chatt_state[i] = 0;
				ts->filter_correct_p->hover_reject_pen_td_move_check_state[i] = 0;
				shtps_hover_reject_pen_clear_pend_event(ts, i);
			}

			if(info->fingers[i].state == SHTPS_TOUCH_STATE_PEN){
				if( (ts->filter_correct_p->hover_reject_pen_chatt_cnt[i] > 0) &&
					(ts->filter_correct_p->hover_reject_pen_chatt_cnt[i] < pen_chatt_cnt) )
				{
					ts->filter_correct_p->hover_reject_pen_chatt_cnt[i]++;
					ts->filter_correct_p->hover_reject_pen_chatt_z_total[i] += ts->fw_report_info.fingers[i].z;
					shtps_hover_reject_pen_add_pend_event(ts, i, info);

					SHTPS_LOG_HOVER_REJECT("<%d> pen z sum <cnt=%d><total=%d>\n",
												i, ts->filter_correct_p->hover_reject_pen_chatt_cnt[i], ts->filter_correct_p->hover_reject_pen_chatt_z_total[i]);
				}
			}else{
				if(ts->filter_correct_p->hover_reject_pen_chatt_cnt[i] > 0){
					ts->filter_correct_p->hover_reject_pen_chatt_z_total[i] = 0;
					ts->filter_correct_p->hover_reject_pen_td_chatt_state[i] = 0;
					ts->filter_correct_p->hover_reject_pen_chatt_cnt[i] = 0;
					shtps_hover_reject_pen_clear_pend_event(ts, i);
					SHTPS_LOG_HOVER_REJECT("<%d> pen z sum clear by state change\n", i);
				}
			}

			/* ---------- */
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH &&
				ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_FINGER)
			{
				ts->filter_correct_p->tu_time[i]                 = jiffies;
				ts->filter_correct_p->tu_check_enable[i]         = 1;
				ts->filter_correct_p->tu_pos[i][SHTPS_POSTYPE_X] = ts->fw_report_info_store.fingers[i].x;
				ts->filter_correct_p->tu_pos[i][SHTPS_POSTYPE_Y] = ts->fw_report_info_store.fingers[i].y;
				ts->filter_correct_p->finger_tu_pen_ignore[i]    = 0;
			}

			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN &&
				ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_FINGER)
			{
				ts->filter_correct_p->finger_tu_pen_ignore[i] = 1;
				info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
				SHTPS_LOG_HOVER_REJECT("<%d> pen ignore by finger -> pen start\n", i);
			}

			if(ts->filter_correct_p->finger_tu_pen_ignore[i] == 0){
				if( (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN) &&
					(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) )
				{
					if(ts->filter_correct_p->tu_check_enable[i] != 0)
					{
						unsigned short diff_x;
						unsigned short diff_y;

						diff_x = shtps_get_diff(ts->filter_correct_p->tu_pos[i][SHTPS_POSTYPE_X], ts->fw_report_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
						diff_y = shtps_get_diff(ts->filter_correct_p->tu_pos[i][SHTPS_POSTYPE_Y], ts->fw_report_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));

						if( (diff_x <= SHTPS_HOVER_REJECT_FINGER_TU_PEN_IGNORE_MOVE_THRESHOLD) &&
							(diff_y <= SHTPS_HOVER_REJECT_FINGER_TU_PEN_IGNORE_MOVE_THRESHOLD) )
						{
							if( time_after(jiffies, ts->filter_correct_p->tu_time[i] + msecs_to_jiffies(SHTPS_HOVER_REJECT_FINGER_TU_PEN_IGNORE_TIME)) == 0 )
							{
								ts->filter_correct_p->finger_tu_pen_ignore[i] = 1;
								info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
								SHTPS_LOG_HOVER_REJECT("<%d> pen ignore by after finger up start\n", i);
							}else{
								ts->filter_correct_p->tu_check_enable[i] = 0;
							}
						}
					}
				}
			}
			else{
				if( (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) ||
					(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) )
				{
					ts->filter_correct_p->finger_tu_pen_ignore[i] = 0;
					SHTPS_LOG_HOVER_REJECT("<%d> pen ignore by after finger end\n", i);
				}
				else{
					info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
					SHTPS_LOG_HOVER_REJECT("<%d> pen ignore by after finger up continue\n", i);
				}
			}

			/* ---------- */
			if(ts->filter_correct_p->hover_reject_pen_td_move_check_state[i] == 1){
				if(info->fingers[i].state == SHTPS_TOUCH_STATE_PEN)
				{
					int diff_x = 0;
					int diff_y = 0;

					if(ts->filter_correct_p->hover_reject_zero_info.fingers[i].x <= SHTPS_HOVER_REJECT_EDGE_AREA){
						diff_x = (ts->fw_report_info.fingers[i].x - ts->filter_correct_p->hover_reject_zero_info.fingers[i].x);
					}
					else if(ts->filter_correct_p->hover_reject_zero_info.fingers[i].x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_HOVER_REJECT_EDGE_AREA)){
						diff_x = (ts->filter_correct_p->hover_reject_zero_info.fingers[i].x - ts->fw_report_info.fingers[i].x);
					}

					if(ts->filter_correct_p->hover_reject_zero_info.fingers[i].y <= SHTPS_HOVER_REJECT_EDGE_AREA){
						diff_y = (ts->fw_report_info.fingers[i].y - ts->filter_correct_p->hover_reject_zero_info.fingers[i].y);
					}
					else if(ts->filter_correct_p->hover_reject_zero_info.fingers[i].y >= (CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1 - SHTPS_HOVER_REJECT_EDGE_AREA)){
						diff_y = (ts->filter_correct_p->hover_reject_zero_info.fingers[i].y - ts->fw_report_info.fingers[i].y);
					}

					if( (diff_x >= SHTPS_HOVER_REJECT_PEN_TD_MOVE_THRESH) ||
						(diff_y >= SHTPS_HOVER_REJECT_PEN_TD_MOVE_THRESH) )
					{
						struct shtps_touch_info rep_info;
						u8 event;

						memcpy(&rep_info, &ts->report_info, sizeof(struct shtps_touch_info));
						rep_info.fingers[i].state = ts->filter_correct_p->hover_reject_zero_info.fingers[i].state;
						rep_info.fingers[i].x  = ts->filter_correct_p->hover_reject_zero_info.fingers[i].x;
						rep_info.fingers[i].y  = ts->filter_correct_p->hover_reject_zero_info.fingers[i].y;
						rep_info.fingers[i].wx = ts->filter_correct_p->hover_reject_zero_info.fingers[i].wx;
						rep_info.fingers[i].wy = ts->filter_correct_p->hover_reject_zero_info.fingers[i].wy;
						rep_info.fingers[i].z  = ts->filter_correct_p->hover_reject_zero_info.fingers[i].z;

						#if defined(SHTPS_EDGE_POS_ADJUST_ENABLE)
							shtps_filter_touch_position_adjust_edge(ts, &rep_info);	//	shtps_touch_position_adjust(ts, &rep_info);
						#endif	/* SHTPS_EDGE_POS_ADJUST_ENABLE */
						#if defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
							shtps_filter_touch_position_adjust_shift_edge(ts, &rep_info);	//	shtps_touch_position_adjust(ts, &rep_info);
						#endif	/* SHTPS_SHIFT_EDGE_INWARD_ENABLE */

						if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
							rep_info.finger_num++;
						}

						shtps_set_eventtype(&event, 0xff);
						if(ts->touch_state.numOfFingers == 0){
							shtps_set_eventtype(&event, SHTPS_EVENT_TD);
						}else{
							shtps_set_eventtype(&event, SHTPS_EVENT_MTDU);
						}

						SHTPS_LOG_HOVER_REJECT("[0] pen td report event [%d]<x=%d, y=%d, wx=%d, wy=%d, z=%d>\n",
													i, rep_info.fingers[i].x, rep_info.fingers[i].y,
													rep_info.fingers[i].wx, rep_info.fingers[i].wy, rep_info.fingers[i].z);

						shtps_event_report(ts, &rep_info, event);
						ts->filter_correct_p->hover_reject_pen_td_move_check_state[i] = 2;
					}else{
						ts->filter_correct_p->hover_reject_pen_td_move_check_state[i] = 0;
					}
				}else{
					ts->filter_correct_p->hover_reject_pen_td_move_check_state[i] = 0;
				}
			}

			if((ts->filter_correct_p->is_force_hover_finger & (1 << i)) == 0){
				if((ts->filter_correct_p->is_hover_finger & (1 << i)) != 0){
					if(ts->filter_correct_p->hover_reject_pen_td_chatt_state[i] != 0){
						if( time_after(jiffies, ts->filter_correct_p->hover_reject_event_drop_time_max[i]) == 0 )
						{
							unsigned short diff_x;
							unsigned short diff_y;

							diff_x = shtps_get_diff(ts->filter_correct_p->hover_reject_zero_info.fingers[i].x, ts->fw_report_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
							diff_y = shtps_get_diff(ts->filter_correct_p->hover_reject_zero_info.fingers[i].y, ts->fw_report_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));

							if( (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN) &&
								(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_PEN) )
							{
								int diff_z = abs(ts->fw_report_info.fingers[i].z - ts->filter_correct_p->hover_reject_zero_info.fingers[i].z);

								if(diff_z >= SHTPS_HOVER_REJECT_4_PEN_Z_FLUCTUATE){
									ts->filter_correct_p->is_force_hover_finger |= (1 << i);
									ts->filter_correct_p->is_hover_finger |= (1 << i);
									ts->filter_correct_p->hover_reject_pen_td_chatt_state[i] = 0;
									SHTPS_LOG_HOVER_REJECT("[4-4] <%d> force event drop start by pen z fluctuate\n", i);
								}
							}

							if( (diff_x >= SHTPS_HOVER_REJECT_4_MOVE_THRESHOLD) ||
								(diff_y >= SHTPS_HOVER_REJECT_4_MOVE_THRESHOLD) )
							{
								ts->filter_correct_p->is_hover_finger &= ~(1 << i);
								ts->filter_correct_p->is_force_hover_finger &= ~(1 << i);
								ts->filter_correct_p->hover_reject_pen_td_chatt_state[i] = 0;
								SHTPS_LOG_HOVER_REJECT("[4-1] <%d> event drop end by move\n", i);
							}

							if(ts->filter_correct_p->hover_reject_zero_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN){
								if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
									ts->filter_correct_p->is_hover_finger &= ~(1 << i);
									ts->filter_correct_p->is_force_hover_finger &= ~(1 << i);
									ts->filter_correct_p->hover_reject_pen_td_chatt_state[i] = 0;
									SHTPS_LOG_HOVER_REJECT("[4-3] <%d> event drop end by finger event\n", i);
								}
							}

							if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN){
								if(ts->fw_report_info.fingers[i].z >= SHTPS_HOVER_REJECT_4_PEN_Z_THRESHOLD){
									ts->filter_correct_p->is_force_hover_finger |= (1 << i);
									ts->filter_correct_p->is_hover_finger |= (1 << i);
									ts->filter_correct_p->hover_reject_pen_td_chatt_state[i] = 0;
									SHTPS_LOG_HOVER_REJECT("[4-2] <%d> force event drop start by pen z threshold\n", i);
								}
							}
						}else{
							ts->filter_correct_p->is_hover_finger &= ~(1 << i);
							ts->filter_correct_p->hover_reject_pen_td_chatt_state[i] = 0;
							SHTPS_LOG_HOVER_REJECT("[4] <%d> event drop end by time over\n", i);
						}
					}

					if(ts->filter_correct_p->hover_reject_pen_td_chatt_state[i] == 0){
						if(ts->filter_correct_p->hover_reject_pen_chatt_cnt[i] > 0){
							ts->filter_correct_p->is_hover_finger |= (1 << i);
						}

						if(ts->filter_correct_p->hover_reject_pen_chatt_cnt[i] >= pen_chatt_cnt){
							if((ts->filter_correct_p->is_force_hover_finger & (1 << i)) == 0){
								if( (pen_chatt_zth_min <= ts->filter_correct_p->hover_reject_pen_chatt_z_total[i]) &&
									(ts->filter_correct_p->hover_reject_pen_chatt_z_total[i] <= pen_chatt_zth_max) )
								{
									ts->filter_correct_p->is_hover_finger &= ~(1 << i);
									shtps_hover_reject_pen_notify_pend_event(ts, i);
									SHTPS_LOG_HOVER_REJECT("[1] <%d> event drop end by z total\n", i);
								}else{
									ts->filter_correct_p->is_force_hover_finger |= (1 << i);
									ts->filter_correct_p->is_hover_finger |= (1 << i);
									SHTPS_LOG_HOVER_REJECT("[1] <%d> force event drop start by z total\n", i);
								}
							}

							ts->filter_correct_p->hover_reject_pen_chatt_cnt[i] = 0;
						}
					}

					if((ts->filter_correct_p->is_hover_finger & (1 << i)) == 0){
						if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
							if(ts->fw_report_info.fingers[i].z <= SHTPS_HOVER_REJECT_4_FINGER_Z_THRESHOLD){
								ts->filter_correct_p->is_force_hover_finger |= (1 << i);
								ts->filter_correct_p->is_hover_finger |= (1 << i);
								SHTPS_LOG_HOVER_REJECT("[4-3-1] <%d> force event drop start by finger z threshold\n", i);
							}
						}
					}
					else{
						if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
							if(ts->fw_report_info.fingers[i].z > SHTPS_HOVER_REJECT_5_FINGER_Z_THRESHOLD){
								ts->filter_correct_p->is_hover_finger &= ~(1 << i);
								SHTPS_LOG_HOVER_REJECT("[5] <%d> event drop end by finger z threshold\n", i);
							}
						}
					}
				}
			}
			else{
				if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
					if(ts->fw_report_info.fingers[i].z > SHTPS_HOVER_REJECT_5_FINGER_Z_THRESHOLD){
						ts->filter_correct_p->is_hover_finger &= ~(1 << i);
						ts->filter_correct_p->is_force_hover_finger &= ~(1 << i);
						SHTPS_LOG_HOVER_REJECT("[5] <%d> event drop end by finger z threshold\n", i);
					}
				}
				else{
					SHTPS_LOG_HOVER_REJECT("[5] <%d> force event drop\n", i);
				}

				{
					int diff_x;
					int diff_y;

					diff_x = shtps_get_diff(ts->filter_correct_p->hover_reject_zero_info.fingers[i].x, ts->fw_report_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
					diff_y = shtps_get_diff(ts->filter_correct_p->hover_reject_zero_info.fingers[i].y, ts->fw_report_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));

					if( (diff_x >= SHTPS_HOVER_REJECT_5_MOVE_THRESHOLD) ||
						(diff_y >= SHTPS_HOVER_REJECT_5_MOVE_THRESHOLD) )
					{
						ts->filter_correct_p->is_hover_finger &= ~(1 << i);
						ts->filter_correct_p->is_force_hover_finger &= ~(1 << i);
						SHTPS_LOG_HOVER_REJECT("[5] <%d> event drop end by move threshold\n", i);
					}
				}
			}

			if((ts->filter_correct_p->is_hover_finger & (1 << i)) == 0){
				if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
					if(info->fingers[i].state == SHTPS_TOUCH_STATE_PEN)
					{
						u8 is_event_drop = 0;
						u8 is_force_event_drop = 0;
						int event_drop_time = 0;

						if( (ts->fw_report_info.fingers[i].z >= pen_chatt_zth_cond) ||
							(ts->fw_report_info.fingers[i].z <= pen_chatt_zth_cond2) )
						{
							ts->filter_correct_p->hover_reject_zero_info.fingers[i].state = ts->fw_report_info.fingers[i].state;
							ts->filter_correct_p->hover_reject_zero_info.fingers[i].x     = ts->fw_report_info.fingers[i].x;
							ts->filter_correct_p->hover_reject_zero_info.fingers[i].y     = ts->fw_report_info.fingers[i].y;
							ts->filter_correct_p->hover_reject_zero_info.fingers[i].wx    = ts->fw_report_info.fingers[i].wx;
							ts->filter_correct_p->hover_reject_zero_info.fingers[i].wy    = ts->fw_report_info.fingers[i].wy;
							ts->filter_correct_p->hover_reject_zero_info.fingers[i].z     = ts->fw_report_info.fingers[i].z;

							ts->filter_correct_p->hover_reject_pen_td_move_check_state[i] = 1;

							if(ts->filter_correct_p->hover_reject_pen_chatt_cnt[i] < pen_chatt_cnt){
								ts->filter_correct_p->is_hover_finger |= (1 << i);
								ts->filter_correct_p->hover_reject_pen_chatt_cnt[i]++;
								ts->filter_correct_p->hover_reject_pen_chatt_z_total[i] += ts->fw_report_info.fingers[i].z;

								shtps_hover_reject_pen_add_pend_event(ts, i, info);

								SHTPS_LOG_HOVER_REJECT("<%d> pen z sum <cnt=%d><total=%d>\n",
															i, ts->filter_correct_p->hover_reject_pen_chatt_cnt[i], ts->filter_correct_p->hover_reject_pen_chatt_z_total[i]);
							}

							if(ts->fw_report_info.fingers[i].z <= SHTPS_HOVER_REJECT_3_1_Z_THRESHOLD){
								is_force_event_drop = 1;
								SHTPS_LOG_HOVER_REJECT("[3-1] <%d> detect\n", i);
							}
							else if(ts->fw_report_info.fingers[i].z > SHTPS_HOVER_REJECT_3_2_Z_THRESHOLD){
								is_force_event_drop = 1;
								SHTPS_LOG_HOVER_REJECT("[3-3] <%d> detect\n", i);
							}
						}

						if(is_force_event_drop == 0){
							if(is_event_drop != 0){
								ts->filter_correct_p->is_hover_finger |= (1 << i);
								if(event_drop_time > 0){
									ts->filter_correct_p->hover_reject_event_drop_time_max[i] = jiffies + msecs_to_jiffies(event_drop_time);
									ts->filter_correct_p->hover_reject_pen_td_chatt_state[i] = 1;
									SHTPS_LOG_HOVER_REJECT("[3] <%d> event drop time (%d ms) start\n", i, event_drop_time);
								}else{
									SHTPS_LOG_HOVER_REJECT("[1] <%d> event drop start\n", i);
								}
							}
						}else{
							ts->filter_correct_p->is_force_hover_finger |= (1 << i);
							ts->filter_correct_p->is_hover_finger |= (1 << i);
							SHTPS_LOG_HOVER_REJECT("[3] <%d> force event drop start\n", i);
						}
					}
					else if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
						if( shtps_hover_reject_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y) != 0 ){
							ts->filter_correct_p->hover_reject_zero_info.fingers[i].state = SHTPS_TOUCH_STATE_FINGER;
							ts->filter_correct_p->hover_reject_zero_info.fingers[i].x     = ts->fw_report_info.fingers[i].x;
							ts->filter_correct_p->hover_reject_zero_info.fingers[i].y     = ts->fw_report_info.fingers[i].y;
							ts->filter_correct_p->hover_reject_zero_info.fingers[i].wx    = ts->fw_report_info.fingers[i].wx;
							ts->filter_correct_p->hover_reject_zero_info.fingers[i].wy    = ts->fw_report_info.fingers[i].wy;
							ts->filter_correct_p->hover_reject_zero_info.fingers[i].z     = ts->fw_report_info.fingers[i].z;

							if(ts->fw_report_info.fingers[i].z <= SHTPS_HOVER_REJECT_2_FINGER_Z_THRESHOLD){
								ts->filter_correct_p->is_force_hover_finger |= (1 << i);
								ts->filter_correct_p->is_hover_finger |= (1 << i);
								SHTPS_LOG_HOVER_REJECT("[2] <%d> force event drop start\n", i);
							}
						}
					}
				}
			}

			if((ts->filter_correct_p->is_force_hover_finger & (1 << i)) != 0){
				ts->filter_correct_p->hover_reject_pen_td_move_check_state[i] = 0;
			}

			if((ts->filter_correct_p->is_hover_finger & (1 << i)) != 0){
				if(ts->filter_correct_p->hover_reject_pen_td_move_check_state[i] == 2){
					info->fingers[i].state = ts->report_info.fingers[i].state;
					info->fingers[i].x  = ts->report_info.fingers[i].x;
					info->fingers[i].y  = ts->report_info.fingers[i].y;
					info->fingers[i].wx = ts->report_info.fingers[i].wx;
					info->fingers[i].wy = ts->report_info.fingers[i].wy;
					info->fingers[i].z  = ts->report_info.fingers[i].z;
					SHTPS_LOG_HOVER_REJECT("[0] <%d> event preserve\n", i);
				}else{
					info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
					SHTPS_LOG_HOVER_REJECT("<%d> event droped\n", i);
				}
			}else{
				ts->filter_correct_p->hover_reject_pen_td_move_check_state[i] = 0;
			}
		}

		if(ts->filter_correct_p->is_hover_finger == 0){
			if(ts->filter_correct_p->read_touchevent_delayed_enable != 0){
				ts->filter_correct_p->read_touchevent_delayed_enable = 0;
				shtps_read_touchevent_timer_stop(ts);
			}
		}else{
			if(SHTPS_HOVER_REJECT_READ_TOUCH_EVENT_POLLING_TIME_MS > 0){
				ts->filter_correct_p->read_touchevent_delayed_enable = 1;
				shtps_read_touchevent_timer_start(ts, SHTPS_HOVER_REJECT_READ_TOUCH_EVENT_POLLING_TIME_MS);
			}
		}
	}
	#endif /* SHTPS_HOVER_REJECT_ENABLE */

	#if defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE)
	if(SHTPS_HOST_PEN_CONNECT_REJECTION_ENABLE != 0)
	{
		int i, j;
		int fingerMax = shtps_get_fingermax(ts);
		u8 is_force_touch_up = 0;

		for(i = 0; i < fingerMax; i++){
			is_force_touch_up = 0;

			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				ts->filter_correct_p->pen_z_hist_count[i] = 0;
				ts->filter_correct_p->pen_event_fail_cont_reject_chattering_enable[i] = 0;
				ts->filter_correct_p->pen_z_hist_count_2nd[i] = 0;
				ts->filter_correct_p->pen_z_dummy_tu_min[i] = 0;
				ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_enable[i] = 0;
				ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i] = 0;
				ts->filter_correct_p->pen_event_fail_cont_reject_dummy_tu_state[i] = 0;
			}

			if(info->fingers[i].state == SHTPS_TOUCH_STATE_PEN)
			{
				if( ((SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_TYPE & 0x01) != 0) &&
					(ts->filter_correct_p->pen_z_hist_count[i] >= SHTPS_PEN_Z_HIST_COUNT_MAX) )
				{
					u16 z_effect_avg = 0;
					u8 is_large_z_detect = 0;

					for(j = 0; j < SHTPS_PEN_Z_HIST_COUNT_MAX; j++){
						z_effect_avg += ts->filter_correct_p->pen_z_hist[i][j];

						if(ts->filter_correct_p->pen_z_hist[i][j] >= SHTPS_PEN_FORCE_EFFECT_Z_THRESHOLD){
							is_large_z_detect = 1;
						}
					}
					z_effect_avg /= SHTPS_PEN_Z_HIST_COUNT_MAX;
					z_effect_avg = (z_effect_avg * SHTPS_PEN_EFFECT_Z_AVERAGE_PERCENT) / 100;
					SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[1] <%d> check enable [z effect avg = %d]\n", i, z_effect_avg);

					if(ts->filter_correct_p->pen_event_fail_cont_reject_chattering_enable[i] == 0){
						if(is_large_z_detect == 0){
							if( (ts->fw_report_info.fingers[i].z < z_effect_avg) &&
								((ts->fw_report_info_store.fingers[i].z - ts->fw_report_info.fingers[i].z) >= SHTPS_PEN_EFFECT_Z_DIFF_THRESHOLD) )
							{
								ts->filter_correct_p->pen_event_fail_cont_reject_chattering_time_max[i] = jiffies + msecs_to_jiffies(SHTPS_PEN_CHATTERING_TIME_MS);
								ts->filter_correct_p->pen_event_fail_cont_reject_chattering_enable[i] = 1;

								info->fingers[i].state = ts->report_info.fingers[i].state;
								info->fingers[i].x     = ts->report_info.fingers[i].x;
								info->fingers[i].y     = ts->report_info.fingers[i].y;
								info->fingers[i].wx    = ts->report_info.fingers[i].wx;
								info->fingers[i].wy    = ts->report_info.fingers[i].wy;
								info->fingers[i].z     = ts->report_info.fingers[i].z;

								SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[1] <%d> chattering start [%d ms]\n", i, SHTPS_PEN_CHATTERING_TIME_MS);
							}
						}
					}
					else{
						if( time_after(jiffies, ts->filter_correct_p->pen_event_fail_cont_reject_chattering_time_max[i]) == 0 ){
							if(ts->fw_report_info.fingers[i].z >= z_effect_avg){
								ts->filter_correct_p->pen_event_fail_cont_reject_chattering_enable[i] = 0;
								SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[1] <%d> chattering end by z threshold over\n", i);
							}
							else{
								info->fingers[i].state = ts->report_info.fingers[i].state;
								info->fingers[i].x     = ts->report_info.fingers[i].x;
								info->fingers[i].y     = ts->report_info.fingers[i].y;
								info->fingers[i].wx    = ts->report_info.fingers[i].wx;
								info->fingers[i].wy    = ts->report_info.fingers[i].wy;
								info->fingers[i].z     = ts->report_info.fingers[i].z;
								SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[1] <%d> chattering continue\n", i);
							}
						}
						else{
							info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
							ts->filter_correct_p->pen_z_hist_count[i] = 0;
							ts->filter_correct_p->pen_event_fail_cont_reject_chattering_enable[i] = 0;
							is_force_touch_up = 1;
							SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[1] <%d> chattering end by time over\n", i);
						}
					}
				}

				if( ((SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_TYPE & 0x02) != 0) &&
					(ts->filter_correct_p->pen_z_hist_count_2nd[i] >= SHTPS_PEN_Z_HIST_COUNT_2ND_MAX) )
				{
					u16 z_effect_max = 0;

					for(j = 0; j < SHTPS_PEN_Z_HIST_COUNT_2ND_MAX; j++){
						if(z_effect_max < ts->filter_correct_p->pen_z_hist_2nd[i][j]){
							z_effect_max = ts->filter_correct_p->pen_z_hist_2nd[i][j];
						}
					}
					z_effect_max /= SHTPS_PEN_EFFECT_Z_MAX_DIVIDE;
					SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[2] <%d> check enable [z effect max = %d]\n", i, z_effect_max);

					if(ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_enable[i] == 0){
						if( (ts->fw_report_info.fingers[i].z <= z_effect_max) &&
							(ts->fw_report_info.fingers[i].z <= SHTPS_PEN_EFFECT_Z_MAX_DIVIDE_THRESHOLD) )
						{
							ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_enable[i] = 1;
							ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i] = 0;

							info->fingers[i].state = ts->report_info.fingers[i].state;
							info->fingers[i].x     = ts->report_info.fingers[i].x;
							info->fingers[i].y     = ts->report_info.fingers[i].y;
							info->fingers[i].wx    = ts->report_info.fingers[i].wx;
							info->fingers[i].wy    = ts->report_info.fingers[i].wy;
							info->fingers[i].z     = ts->report_info.fingers[i].z;

							ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]].x  = ts->fw_report_info.fingers[i].x;
							ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]].y  = ts->fw_report_info.fingers[i].y;
							ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]].wx = ts->fw_report_info.fingers[i].wx;
							ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]].wy = ts->fw_report_info.fingers[i].wy;
							ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]].z  = ts->fw_report_info.fingers[i].z;

							ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]++;

							SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[2] <%d> chattering count start [%d]\n",
																i, ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]);
						}
					}
					else{
						if(ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i] >= SHTPS_PEN_CHATTERING_COUNT_MAX){
							if(ts->filter_correct_p->pen_event_fail_cont_reject_dummy_tu_state[i] == 0){
								if( (ts->fw_report_info.fingers[i].z <= z_effect_max) &&
									(ts->fw_report_info.fingers[i].z <= SHTPS_PEN_EFFECT_Z_MAX_DIVIDE_THRESHOLD) )
								{
									info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
									ts->filter_correct_p->pen_event_fail_cont_reject_dummy_tu_state[i] = 1;
									ts->filter_correct_p->pen_z_dummy_tu_min[i] = ts->fw_report_info.fingers[i].z;

									SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[2] <%d> dummy tu start [z=%d]\n",
																		i, ts->fw_report_info.fingers[i].z);
								}
								else{
									SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[2] <%d> chattering end and hold event report\n", i);

									for(j = 0; j < SHTPS_PEN_CHATTERING_COUNT_MAX; j++)
									{
										int w = (ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].wx >= ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].wy) ?
													ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].wx : ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].wy;

										if(w < SHTPS_FINGER_WIDTH_MIN){
											w = SHTPS_FINGER_WIDTH_MIN;
										}

										shtps_report_touch_pen_on(ts, i,
																  ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].x,
																  ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].y,
																  w,
																  ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].wx,
																  ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].wy,
																  ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].z);
										input_sync(ts->input);

										if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
											ts->touch_state.numOfFingers++;
										}

										ts->report_info.fingers[i].state = SHTPS_TOUCH_STATE_PEN;
										ts->report_info.fingers[i].x     = ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].x;
										ts->report_info.fingers[i].y     = ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].y;
										ts->report_info.fingers[i].wx    = ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].wx;
										ts->report_info.fingers[i].wy    = ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].wy;
										ts->report_info.fingers[i].z     = ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][j].z;
									}

									ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_enable[i] = 0;
									ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i] = 0;
								}
							}
							else{
								u16 dummy_tu_z_effect_max = 0;

								SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[2] <%d> dummy tu z min = %d\n", i, ts->filter_correct_p->pen_z_dummy_tu_min[i]);

								for(j = 0; j < SHTPS_PEN_Z_HIST_COUNT_2ND_MAX; j++){
									if(dummy_tu_z_effect_max < ts->filter_correct_p->pen_z_hist_2nd[i][j]){
										dummy_tu_z_effect_max = ts->filter_correct_p->pen_z_hist_2nd[i][j];
									}
								}
								dummy_tu_z_effect_max = (dummy_tu_z_effect_max * SHTPS_PEN_CHATTERING_CANCEL_Z_MAX_PERCENT) / 100;
								SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[2] <%d> dummy tu z effect max = %d\n", i, dummy_tu_z_effect_max);

								if( (ts->fw_report_info.fingers[i].z >= (ts->filter_correct_p->pen_z_dummy_tu_min[i] + SHTPS_PEN_EFFECT_Z_DIFF_MIN_THRESHOLD)) &&
									(ts->fw_report_info.fingers[i].z >= dummy_tu_z_effect_max) &&
									(ts->fw_report_info.fingers[i].z >= SHTPS_PEN_CHATTERING_CANCEL_Z_THRESHOLD) )
								{
									ts->filter_correct_p->pen_event_fail_cont_reject_dummy_tu_state[i] = 0;
									ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_enable[i] = 0;
									ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i] = 0;
									SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[2] <%d> dummy tu end\n", i);
								}
								else{
									info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;

									if(ts->filter_correct_p->pen_z_dummy_tu_min[i] > ts->fw_report_info.fingers[i].z){
										ts->filter_correct_p->pen_z_dummy_tu_min[i] = ts->fw_report_info.fingers[i].z;
										SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[2] <%d> dummy tu z min update [z=%d]\n", i, ts->fw_report_info.fingers[i].z);
									}
								}
							}
						}
						else{
							info->fingers[i].state = ts->report_info.fingers[i].state;
							info->fingers[i].x     = ts->report_info.fingers[i].x;
							info->fingers[i].y     = ts->report_info.fingers[i].y;
							info->fingers[i].wx    = ts->report_info.fingers[i].wx;
							info->fingers[i].wy    = ts->report_info.fingers[i].wy;
							info->fingers[i].z     = ts->report_info.fingers[i].z;

							ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]].x  = ts->fw_report_info.fingers[i].x;
							ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]].y  = ts->fw_report_info.fingers[i].y;
							ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]].wx = ts->fw_report_info.fingers[i].wx;
							ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]].wy = ts->fw_report_info.fingers[i].wy;
							ts->filter_correct_p->pen_event_fail_cont_reject_hold_info[i][ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]].z  = ts->fw_report_info.fingers[i].z;

							ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]++;

							SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[2] <%d> chattering count continue [%d]\n",
																i, ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count[i]);
						}
					}
				}

				if( (is_force_touch_up == 0) &&
					(ts->filter_correct_p->pen_event_fail_cont_reject_chattering_enable[i] == 0) )
				{
					if(ts->filter_correct_p->pen_z_hist_count[i] < SHTPS_PEN_Z_HIST_COUNT_MAX){
						ts->filter_correct_p->pen_z_hist[i][ts->filter_correct_p->pen_z_hist_count[i]] = ts->fw_report_info.fingers[i].z;
						ts->filter_correct_p->pen_z_hist_count[i]++;
						SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[1] <%d> z hist add [count=%d][z=%d]\n",
															i, ts->filter_correct_p->pen_z_hist_count[i], ts->fw_report_info.fingers[i].z);
					}
					else{
						for(j = 1; j < SHTPS_PEN_Z_HIST_COUNT_MAX; j++){
							ts->filter_correct_p->pen_z_hist[i][j - 1] = ts->filter_correct_p->pen_z_hist[i][j];
						}
						ts->filter_correct_p->pen_z_hist[i][SHTPS_PEN_Z_HIST_COUNT_MAX - 1] = ts->fw_report_info.fingers[i].z;
						SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[1] <%d> z hist update [count=%d][z=%d]\n",
															i, ts->filter_correct_p->pen_z_hist_count[i], ts->filter_correct_p->pen_z_hist[i][SHTPS_PEN_Z_HIST_COUNT_MAX - 1]);
					}
				}

				if( (ts->filter_correct_p->pen_event_fail_cont_reject_dummy_tu_state[i] == 0) &&
					(ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_enable[i] == 0) )
				{
					if(ts->filter_correct_p->pen_z_hist_count_2nd[i] < SHTPS_PEN_Z_HIST_COUNT_2ND_MAX){
						ts->filter_correct_p->pen_z_hist_2nd[i][ts->filter_correct_p->pen_z_hist_count_2nd[i]] = ts->fw_report_info.fingers[i].z;
						ts->filter_correct_p->pen_z_hist_count_2nd[i]++;
						SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[2] <%d> z hist add [count=%d][z=%d]\n",
															i, ts->filter_correct_p->pen_z_hist_count_2nd[i], ts->fw_report_info.fingers[i].z);
					}
					else{
						for(j = 1; j < SHTPS_PEN_Z_HIST_COUNT_2ND_MAX; j++){
							ts->filter_correct_p->pen_z_hist_2nd[i][j - 1] = ts->filter_correct_p->pen_z_hist_2nd[i][j];
						}
						ts->filter_correct_p->pen_z_hist_2nd[i][SHTPS_PEN_Z_HIST_COUNT_2ND_MAX - 1] = ts->fw_report_info.fingers[i].z;
						SHTPS_LOG_PEN_EVENT_FAIL_CONTINUE("[2] <%d> z hist update [count=%d][z=%d]\n",
															i, ts->filter_correct_p->pen_z_hist_count_2nd[i], ts->filter_correct_p->pen_z_hist_2nd[i][SHTPS_PEN_Z_HIST_COUNT_2ND_MAX - 1]);
					}
				}
			}
		}
	}
	#endif /* SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE */
}

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE)
void shtps_filter_invalid_area_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	SHTPS_LOG_FUNC_CALL();
	{
		int x, y, i;
		int fingerMax = shtps_get_fingermax(ts);

		for(i = 0; i < fingerMax; i++){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
				x = info->fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
				y = info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;

				if( y <= SHTPS_EDGE_HOVER_FAIL_RANGE_Y ||
					y >= ((CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1) - SHTPS_EDGE_HOVER_FAIL_RANGE_Y) ||
					x <= SHTPS_EDGE_HOVER_FAIL_RANGE_X ||
					x >= ((CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1) - SHTPS_EDGE_HOVER_FAIL_RANGE_X) )
				{
					#if defined(SHTPS_HOVER_EDGE_LOST_RESOLV_ENABLE)
					if(!SHTPS_HOVER_EDGE_LOST_RESOLV_DISABLE){
						if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
							if(y <= SHTPS_EDGE_HOVER_FAIL_RANGE_Y){
								info->fingers[i].y = SHTPS_EDGE_HOVER_FAIL_RANGE_Y + 1;
							}else if(y >= ((CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1) - SHTPS_EDGE_HOVER_FAIL_RANGE_Y)){
								info->fingers[i].y = ((CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1) - SHTPS_EDGE_HOVER_FAIL_RANGE_Y) - 1;
							}
							
							if(x <= SHTPS_EDGE_HOVER_FAIL_RANGE_X){
								info->fingers[i].x = SHTPS_EDGE_HOVER_FAIL_RANGE_X + 1;
							}else if(x >= ((CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1) - SHTPS_EDGE_HOVER_FAIL_RANGE_X)){
								info->fingers[i].x = ((CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1) - SHTPS_EDGE_HOVER_FAIL_RANGE_X) - 1;
							}
							continue;
						}
					}
					#endif /* SHTPS_HOVER_EDGE_LOST_RESOLV_ENABLE */
					
					ts->filter_correct_p->hover_invalid_touch_info |= (1 << i);
					info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
					if(hover_debug_log_enable & 0x01){
						SHTPS_LOG_DBG_PRINT("[hover][%d] detect in invalid area (%d, %d)\n", i, x, y);
					}
				}
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_chattering_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{

	int x, y, i, j;
	int fingerMax = shtps_get_fingermax(ts);
	unsigned long now_time;

	if(hover_report_info_calc_type == 0){
		return;
	}

	SHTPS_LOG_FUNC_CALL();
	now_time = jiffies;
	for(i = 0; i < fingerMax; i++){
		if( (info->fingers[i].state == SHTPS_TOUCH_STATE_HOVER) &&
			(((ts->filter_correct_p->hover_ignore_touch_info >> i) & 0x01) == 0) )
		{
			if( (1 < SHTPS_HOVER_HIST_COUNT_MAX) && (SHTPS_HOVER_HIST_COUNT_MAX < SHTPS_HOVER_HIST_MAX) )
			{
				x = info->fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
				y = info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;

				if( ts->filter_correct_p->hover_hist_count < (SHTPS_HOVER_HIST_COUNT_MAX - 1) ){
					ts->filter_correct_p->hover_hist[ts->filter_correct_p->hover_hist_count].x = x;
					ts->filter_correct_p->hover_hist[ts->filter_correct_p->hover_hist_count].y = y;
					ts->filter_correct_p->hover_hist[ts->filter_correct_p->hover_hist_count].time = now_time + msecs_to_jiffies(SHTPS_HOVER_INFO_EFFECT_TIME_MS);

					if(hover_debug_log_enable & 0x02){
						SHTPS_LOG_DBG_PRINT("hover hist add continue <%d>(%d, %d)(time : %lu)\n",
												ts->filter_correct_p->hover_hist_count, x, y, ts->filter_correct_p->hover_hist[ts->filter_correct_p->hover_hist_count].time);
					}

					ts->filter_correct_p->hover_hist_count++;

					ts->filter_correct_p->hover_ignore_touch_info |= (1 << i);
					info->fingers[i].state	= ts->report_info.fingers[i].state;
					info->fingers[i].x		= ts->report_info.fingers[i].x;
					info->fingers[i].y		= ts->report_info.fingers[i].y;
					info->fingers[i].wx		= ts->report_info.fingers[i].wx;
					info->fingers[i].wy		= ts->report_info.fingers[i].wy;
					info->fingers[i].z		= ts->report_info.fingers[i].z;

				}else if(ts->filter_correct_p->hover_hist_count < SHTPS_HOVER_HIST_COUNT_MAX){
					ts->filter_correct_p->hover_hist[ts->filter_correct_p->hover_hist_count].x = x;
					ts->filter_correct_p->hover_hist[ts->filter_correct_p->hover_hist_count].y = y;
					ts->filter_correct_p->hover_hist[ts->filter_correct_p->hover_hist_count].time = now_time + msecs_to_jiffies(SHTPS_HOVER_INFO_EFFECT_TIME_MS);
					if(hover_debug_log_enable & 0x02){
						SHTPS_LOG_DBG_PRINT("hover hist add finished <%d>(%d, %d)(time : %lu)\n",
												ts->filter_correct_p->hover_hist_count, x, y, ts->filter_correct_p->hover_hist[ts->filter_correct_p->hover_hist_count].time);
					}
					ts->filter_correct_p->hover_hist_count++;
				}else{
					for(j = 1; j < SHTPS_HOVER_HIST_COUNT_MAX; j++){
						ts->filter_correct_p->hover_hist[j - 1] = ts->filter_correct_p->hover_hist[j];
					}
					ts->filter_correct_p->hover_hist[SHTPS_HOVER_HIST_COUNT_MAX - 1].x = x;
					ts->filter_correct_p->hover_hist[SHTPS_HOVER_HIST_COUNT_MAX - 1].y = y;
					ts->filter_correct_p->hover_hist[SHTPS_HOVER_HIST_COUNT_MAX - 1].time = now_time + msecs_to_jiffies(SHTPS_HOVER_INFO_EFFECT_TIME_MS);
					if(hover_debug_log_enable & 0x02){
						SHTPS_LOG_DBG_PRINT("hover hist update <%d>(%d, %d)(time : %lu)\n", j, x, y, ts->filter_correct_p->hover_hist[SHTPS_HOVER_HIST_COUNT_MAX - 1].time);
					}
				}
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_set_report_touch_info(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int fingerMax = shtps_get_fingermax(ts);
	int i, j, k, tmp;
	int *work_x, *work_y;
	int center_x, center_y;
	int max_x, max_y, min_x, min_y;
	unsigned long now_time;
	u8 hover_effect_num = 0;

	if(hover_report_info_calc_type == 0){
		return;
	}

	SHTPS_LOG_FUNC_CALL();
	work_x = (int*)kzalloc(sizeof(int) * SHTPS_HOVER_HIST_COUNT_MAX, GFP_KERNEL);
	work_y = (int*)kzalloc(sizeof(int) * SHTPS_HOVER_HIST_COUNT_MAX, GFP_KERNEL);
	now_time = jiffies;

	for(i = 0; i < fingerMax; i++){
		if( (info->fingers[i].state == SHTPS_TOUCH_STATE_HOVER) &&
			(((ts->filter_correct_p->hover_ignore_touch_info >> i) & 0x01) == 0) )
		{
			if( (1 < SHTPS_HOVER_HIST_COUNT_MAX) && (SHTPS_HOVER_HIST_COUNT_MAX < SHTPS_HOVER_HIST_MAX) ){
				if(ts->filter_correct_p->hover_hist_count >= SHTPS_HOVER_HIST_COUNT_MAX){
					if(hover_debug_log_enable & 0x04){
						SHTPS_LOG_DBG_PRINT("hover hist now time : %lu\n", now_time);
					}

					for(j = 0; j < SHTPS_HOVER_HIST_COUNT_MAX; j++){
						if( time_after(now_time, ts->filter_correct_p->hover_hist[j].time) == 0 ){
							work_x[hover_effect_num] = ts->filter_correct_p->hover_hist[j].x;
							work_y[hover_effect_num] = ts->filter_correct_p->hover_hist[j].y;
							hover_effect_num++;
							if(hover_debug_log_enable & 0x04){
								SHTPS_LOG_DBG_PRINT("hover hist data <%d>(%d, %d)(time : %lu) enable\n",
														j, ts->filter_correct_p->hover_hist[j].x, ts->filter_correct_p->hover_hist[j].y, ts->filter_correct_p->hover_hist[j].time);
							}
						}else{
							if(hover_debug_log_enable & 0x04){
								SHTPS_LOG_DBG_PRINT("hover hist data <%d>(%d, %d)(time : %lu) disable\n",
														j, ts->filter_correct_p->hover_hist[j].x, ts->filter_correct_p->hover_hist[j].y, ts->filter_correct_p->hover_hist[j].time);
							}
						}
					}

					if(hover_report_info_calc_type == 1)
					{
						if(hover_effect_num > 0){
							for (j = 0; j < hover_effect_num - 1; j++) {
								for (k = j + 1; k < hover_effect_num; k++) {
									if (work_x[j] > work_x[k]) {
										tmp = work_x[j];
										work_x[j] = work_x[k];
										work_x[k] = tmp;
									}

									if (work_y[j] > work_y[k]) {
										tmp = work_y[j];
										work_y[j] = work_y[k];
										work_y[k] = tmp;
									}
								}
							}

							if( (hover_effect_num & 0x01) != 0 ){
								center_x = work_x[hover_effect_num / 2];
								center_y = work_y[hover_effect_num / 2];
							} else {
								center_x = (work_x[hover_effect_num / 2 - 1] + work_x[hover_effect_num / 2]) / 2;
								center_y = (work_y[hover_effect_num / 2 - 1] + work_y[hover_effect_num / 2]) / 2;
							}

							if(ts->filter_correct_p->hover_center_hist_count < SHTPS_HOVER_CENTER_HIST_MAX){
								ts->filter_correct_p->hover_center_hist[ts->filter_correct_p->hover_center_hist_count].x = center_x;
								ts->filter_correct_p->hover_center_hist[ts->filter_correct_p->hover_center_hist_count].y = center_y;
								ts->filter_correct_p->hover_center_hist[ts->filter_correct_p->hover_center_hist_count].time = now_time + msecs_to_jiffies(SHTPS_HOVER_CENTER_INFO_EFFECT_TIME_MS);

								if(hover_debug_log_enable & 0x04){
									SHTPS_LOG_DBG_PRINT("hover hist center data add <%d>(%d, %d)(time : %lu)\n",
															ts->filter_correct_p->hover_center_hist_count, center_x, center_y,
															ts->filter_correct_p->hover_center_hist[ts->filter_correct_p->hover_center_hist_count].time);
								}

								ts->filter_correct_p->hover_center_hist_count++;
							}else{
								for(j = 1; j < SHTPS_HOVER_CENTER_HIST_MAX; j++){
									ts->filter_correct_p->hover_center_hist[j - 1] = ts->filter_correct_p->hover_center_hist[j];
								}
								ts->filter_correct_p->hover_center_hist[SHTPS_HOVER_CENTER_HIST_MAX - 1].x = center_x;
								ts->filter_correct_p->hover_center_hist[SHTPS_HOVER_CENTER_HIST_MAX - 1].y = center_y;
								ts->filter_correct_p->hover_center_hist[SHTPS_HOVER_CENTER_HIST_MAX - 1].time = now_time + msecs_to_jiffies(SHTPS_HOVER_CENTER_INFO_EFFECT_TIME_MS);

								if(hover_debug_log_enable & 0x04){
									SHTPS_LOG_DBG_PRINT("hover hist center data add <%d>(%d, %d)(time : %lu)\n",
															(SHTPS_HOVER_CENTER_HIST_MAX - 1), center_x, center_y,
															ts->filter_correct_p->hover_center_hist[SHTPS_HOVER_CENTER_HIST_MAX - 1].time);
								}
							}
						}

						max_x = -1;
						max_y = -1;
						min_x = CONFIG_SHTPS_SY3000_LCD_SIZE_X;
						min_y = CONFIG_SHTPS_SY3000_LCD_SIZE_Y;

						for(j = 0; j < ts->filter_correct_p->hover_center_hist_count; j++){
							if( time_after(now_time, ts->filter_correct_p->hover_center_hist[j].time) == 0 ){
								if(hover_debug_log_enable & 0x08){
									SHTPS_LOG_DBG_PRINT("hover hist center data <%d>(%d, %d)(time : %lu) enable\n",
															j, ts->filter_correct_p->hover_center_hist[j].x, ts->filter_correct_p->hover_center_hist[j].y,
															ts->filter_correct_p->hover_center_hist[j].time);
								}

								if(max_x < ts->filter_correct_p->hover_center_hist[j].x){
									max_x = ts->filter_correct_p->hover_center_hist[j].x;
								}
								if(max_y < ts->filter_correct_p->hover_center_hist[j].y){
									max_y = ts->filter_correct_p->hover_center_hist[j].y;
								}
								if(min_x > ts->filter_correct_p->hover_center_hist[j].x){
									min_x = ts->filter_correct_p->hover_center_hist[j].x;
								}
								if(min_y > ts->filter_correct_p->hover_center_hist[j].y){
									min_y = ts->filter_correct_p->hover_center_hist[j].y;
								}
							}else{
								if(hover_debug_log_enable & 0x08){
									SHTPS_LOG_DBG_PRINT("hover hist center data <%d>(%d, %d)(time : %lu) disable\n",
															j, ts->filter_correct_p->hover_center_hist[j].x, ts->filter_correct_p->hover_center_hist[j].y,
															ts->filter_correct_p->hover_center_hist[j].time);
								}
							}
						}

						if( (max_x >= 0) && (max_y >= 0) &&
							(min_x < CONFIG_SHTPS_SY3000_LCD_SIZE_X) && (min_y < CONFIG_SHTPS_SY3000_LCD_SIZE_Y) ){
							info->fingers[i].x = ((max_x + min_x) / 2);
							info->fingers[i].y = ((max_y + min_y) / 2);
						}
					}
					else{
						if(hover_effect_num == SHTPS_HOVER_HIST_COUNT_MAX){
							for (j = 0; j < SHTPS_HOVER_HIST_COUNT_MAX - 1; j++) {
								for (k = j + 1; k < SHTPS_HOVER_HIST_COUNT_MAX; k++) {
									if (work_x[j] > work_x[k]) {
										tmp = work_x[j];
										work_x[j] = work_x[k];
										work_x[k] = tmp;
									}

									if (work_y[j] > work_y[k]) {
										tmp = work_y[j];
										work_y[j] = work_y[k];
										work_y[k] = tmp;
									}
								}
							}

							if( hover_report_info_calc_type == 2 ){
								center_x = (work_x[1] + work_x[SHTPS_HOVER_HIST_COUNT_MAX - 2]) / 2;
								center_y = (work_y[1] + work_y[SHTPS_HOVER_HIST_COUNT_MAX - 2]) / 2;
							} else {
								center_x = (work_x[SHTPS_HOVER_HIST_COUNT_MAX / 2 - 1] + work_x[SHTPS_HOVER_HIST_COUNT_MAX / 2]) / 2;
								center_y = (work_y[SHTPS_HOVER_HIST_COUNT_MAX / 2 - 1] + work_y[SHTPS_HOVER_HIST_COUNT_MAX / 2]) / 2;
							}

							info->fingers[i].x = center_x;
							info->fingers[i].y = center_y;
						}else{
							ts->filter_correct_p->hover_ignore_touch_info |= (1 << i);
							info->fingers[i].state	= ts->report_info.fingers[i].state;
							info->fingers[i].x		= ts->report_info.fingers[i].x;
							info->fingers[i].y		= ts->report_info.fingers[i].y;
							info->fingers[i].wx		= ts->report_info.fingers[i].wx;
							info->fingers[i].wy		= ts->report_info.fingers[i].wy;
							info->fingers[i].z		= ts->report_info.fingers[i].z;
						}
					}

					if(hover_debug_log_enable & 0x01){
						SHTPS_LOG_DBG_PRINT("hover report data <%d>(%d, %d)\n", i, info->fingers[i].x, info->fingers[i].y);
					}
				}
			}
		}
	}

	kfree(work_x);
	kfree(work_y);
}
/* -------------------------------------------------------------------------- */
void shtps_filter_set_touch_info_hover_detect_init(struct shtps_rmi_spi *ts)
{
	ts->filter_correct_p->hover_ignore_touch_info = 0x00;
	ts->filter_correct_p->hover_invalid_touch_info = 0x00;
	#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
		if(hover_report_info_calc_type > 1){
			SHTPS_HOVER_HIST_COUNT_MAX = 6;
		}else{
			SHTPS_HOVER_HIST_COUNT_MAX = 5;
		}
	#endif /* SHTPS_DEBUG_VARIABLE_DEFINES */
}

/* -------------------------------------------------------------------------- */
void shtps_filter_set_touch_info_hover_detect(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, int i)
{
	if(stylus_detect_is_hover_event_enable != 0){
		if(info->fingers[i].state == SHTPS_TOUCH_STATE_PEN){
			info->fingers[i].state = SHTPS_TOUCH_STATE_HOVER;
			ts->fw_report_info.fingers[i].state = SHTPS_TOUCH_STATE_HOVER;

			SHTPS_LOG_EVENT(
				DBG_PRINTK("Touch No.%d state changed pen to hover\n", i);
			);
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_set_hover_detect_tu_timer(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int fingerMax = shtps_get_fingermax(ts);
	int i;

	for(i = 0;i < fingerMax;i++){
		if( (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) ||
			(((ts->filter_correct_p->hover_invalid_touch_info >> i) & 0x01) != 0) )
		{
			if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
				info->fingers[i].state = SHTPS_TOUCH_STATE_HOVER;
				info->fingers[i].x  = ts->report_info.fingers[i].x;
				info->fingers[i].y  = ts->report_info.fingers[i].y;
				info->fingers[i].wx = ts->report_info.fingers[i].wx;
				info->fingers[i].wy = ts->report_info.fingers[i].wy;
				info->fingers[i].z  = ts->report_info.fingers[i].z;

				if((ts->filter_correct_p->hover_touch_up_delayed_finger & (1 << i)) == 0){
					ts->filter_correct_p->hover_touch_up_delayed_finger |= (1 << i);
					shtps_hover_tu_timer_start(ts, SHTPS_HOVER_TU_DELAY_TIME_MS);
				}
			}else if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
				ts->filter_correct_p->hover_hist_count = 0;
				ts->filter_correct_p->hover_center_hist_count = 0;
			}
		}else{
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
				if(((ts->filter_correct_p->hover_ignore_touch_info >> i) & 0x01) == 0){
					ts->filter_correct_p->hover_touch_up_delayed_finger &= ~(1 << i);
				}
			}else if( (ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_HOVER) ||
					  ((ts->filter_correct_p->hover_touch_up_delayed_finger & (1 << i)) == 0) )
			{
				ts->filter_correct_p->hover_touch_up_delayed_finger &= ~(1 << i);
				ts->filter_correct_p->hover_hist_count = 0;
				ts->filter_correct_p->hover_center_hist_count = 0;
			}else{
				ts->filter_correct_p->hover_touch_up_delayed_finger &= ~(1 << i);
			}
		}
	}
}
#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE) || defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE) || defined(SHTPS_HOVER_REJECT_ENABLE)
void shtps_filter_correct_sleep(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE)
		shtps_hover_tu_timer_stop(ts);
		ts->filter_correct_p->hover_touch_up_delayed_finger = 0x00;
		ts->filter_correct_p->hover_hist_count = 0;
		ts->filter_correct_p->hover_center_hist_count = 0;
	#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE */

	#if defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE)
		memset(ts->filter_correct_p->pen_z_hist_count, 0, sizeof(ts->filter_correct_p->pen_z_hist_count));
		memset(ts->filter_correct_p->pen_event_fail_cont_reject_chattering_enable, 0, sizeof(ts->filter_correct_p->pen_event_fail_cont_reject_chattering_enable));
		memset(ts->filter_correct_p->pen_z_hist_count_2nd, 0, sizeof(ts->filter_correct_p->pen_z_hist_count_2nd));
		memset(ts->filter_correct_p->pen_z_dummy_tu_min, 0, sizeof(ts->filter_correct_p->pen_z_dummy_tu_min));
		memset(ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_enable, 0, sizeof(ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_enable));
		memset(ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count, 0, sizeof(ts->filter_correct_p->pen_event_fail_cont_reject_chattering_2nd_count));
		memset(ts->filter_correct_p->pen_event_fail_cont_reject_dummy_tu_state, 0, sizeof(ts->filter_correct_p->pen_event_fail_cont_reject_dummy_tu_state));
	#endif /* SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE */

	#if defined( SHTPS_HOVER_REJECT_ENABLE )
		ts->filter_correct_p->read_touchevent_delayed_enable = 0;
		shtps_read_touchevent_timer_stop(ts);
		ts->filter_correct_p->is_hover_finger = 0;
		ts->filter_correct_p->is_force_hover_finger = 0;
		memset(ts->filter_correct_p->tu_check_enable, 0, sizeof(ts->filter_correct_p->tu_check_enable));
		memset(ts->filter_correct_p->tu_time, 0, sizeof(ts->filter_correct_p->tu_time));
		memset(ts->filter_correct_p->tu_pos, 0, sizeof(ts->filter_correct_p->tu_pos));
		memset(ts->filter_correct_p->finger_tu_pen_ignore, 0, sizeof(ts->filter_correct_p->finger_tu_pen_ignore));
		memset(&ts->filter_correct_p->hover_reject_zero_info, 0, sizeof(ts->filter_correct_p->hover_reject_zero_info));
		memset(ts->filter_correct_p->hover_reject_event_drop_time_max, 0, sizeof(ts->filter_correct_p->hover_reject_event_drop_time_max));
		memset(ts->filter_correct_p->finger_tu_finger_ignore_count, 0, sizeof(ts->filter_correct_p->finger_tu_finger_ignore_count));
		memset(&ts->filter_correct_p->finger_tu_finger_ignore_td_info, 0, sizeof(ts->filter_correct_p->finger_tu_finger_ignore_td_info));
		memset(ts->filter_correct_p->finger_tu_finger_ignore_td_check_enable, 0, sizeof(ts->filter_correct_p->finger_tu_finger_ignore_td_check_enable));
		memset(ts->filter_correct_p->finger_tu_finger_ignore_tu_time, 0, sizeof(ts->filter_correct_p->finger_tu_finger_ignore_tu_time));
		memset(ts->filter_correct_p->finger_tu_finger_ignore_td_time, 0, sizeof(ts->filter_correct_p->finger_tu_finger_ignore_td_time));
		memset(ts->filter_correct_p->finger_tu_finger_ignore_tu_pos, 0, sizeof(ts->filter_correct_p->finger_tu_finger_ignore_tu_pos));
		memset(ts->filter_correct_p->finger_tu_finger_ignore_td_cont_check_enable, 0, sizeof(ts->filter_correct_p->finger_tu_finger_ignore_td_cont_check_enable));
		memset(ts->filter_correct_p->hover_reject_pen_chatt_cnt, 0, sizeof(ts->filter_correct_p->hover_reject_pen_chatt_cnt));
		memset(ts->filter_correct_p->hover_reject_pen_chatt_z_total, 0, sizeof(ts->filter_correct_p->hover_reject_pen_chatt_z_total));
		memset(ts->filter_correct_p->hover_reject_pen_td_chatt_state, 0, sizeof(ts->filter_correct_p->hover_reject_pen_td_chatt_state));
		memset(ts->filter_correct_p->hover_reject_pen_td_move_check_state, 0, sizeof(ts->filter_correct_p->hover_reject_pen_td_move_check_state));
		memset(ts->filter_correct_p->hover_reject_pen_pending_cnt, 0, sizeof(ts->filter_correct_p->hover_reject_pen_pending_cnt));
	#endif /* SHTPS_HOVER_REJECT_ENABLE */
}

/* -------------------------------------------------------------------------- */
void shtps_filter_correct_init(struct shtps_rmi_spi *ts)
{
	ts->filter_correct_p = kzalloc(sizeof(struct shtps_filter_correct_info), GFP_KERNEL);
	if(ts->filter_correct_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	ts->filter_correct_p->ts_p = ts;

	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE)
		INIT_DELAYED_WORK(&ts->filter_correct_p->hover_touch_up_delayed_work, shtps_hover_touch_up_delayed_work_function);
	#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE */

	#if defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE)
	#endif /* SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE */

	#if defined( SHTPS_HOVER_REJECT_ENABLE )
		INIT_DELAYED_WORK(&ts->filter_correct_p->read_touchevent_delayed_work, shtps_read_touchevent_delayed_work_function);
	#endif /* SHTPS_HOVER_REJECT_ENABLE */
}

/* -------------------------------------------------------------------------- */
void shtps_filter_correct_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->filter_correct_p)	kfree(ts->filter_correct_p);
	ts->filter_correct_p = NULL;
}
#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE || SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE || SHTPS_HOVER_REJECT_ENABLE */
