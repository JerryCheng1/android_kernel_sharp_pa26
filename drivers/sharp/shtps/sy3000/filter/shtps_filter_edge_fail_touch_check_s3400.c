/* drivers/sharp/shtps/sy3000/shtps_filter_edge_fail_touch_check.c
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
#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
enum{
	SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_NONE			= 0x00,
	SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_SIDE			= 0x01,
	SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_TOP			= 0x02,
	SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_TOP_SIDE		= 0x04,
	SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_BOTTOM			= 0x08,
	SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_BOTTOM_SIDE	= 0x10,
};

struct shtps_edge_fail_touch_info{
	unsigned long	time;
	unsigned short	x;
	unsigned short	y;
	unsigned char	z;
	u8				id;
};

struct shtps_filter_edge_fail_touch_rej_info{
	u8									edge_fail_touch_enable;
	u8									edge_fail_touch_inhibit_id;
	u8									edge_fail_touch_decide_mt;
	struct shtps_edge_fail_touch_info	edge_fail_touch_td_info[SHTPS_FINGER_MAX];
	u8									edge_fail_touch_td_cnt;
	u8									edge_fail_touch_pre_single;
	//follow; s3400
	u8									edge_fail_touch_top_inhibit_id;
	struct shtps_touch_info				edge_fail_touch_top_td_info;
	u8									edge_fail_touch_side_inhibit_id;
	struct shtps_touch_info				edge_fail_touch_side_td_info;
	u8									edge_fail_touch_bottom_inhibit_id;
	struct shtps_touch_info				edge_fail_touch_bottom_td_info;
	#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
		u8								right_edge_fail_touch_inhibit_id;
	#endif /* SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */
};

/* -------------------------------------------------------------------------- */
static void shtps_filter_event_touch_cancel(struct shtps_rmi_spi *ts, u8 id)
{
	SHTPS_LOG_FUNC_CALL();
	if(ts->report_info.fingers[id].state == SHTPS_TOUCH_STATE_FINGER){
		shtps_report_touch_on(ts, id,
							  SHTPS_TOUCH_CANCEL_COORDINATES_X,
							  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
							  shtps_get_fingerwidth(ts, id, &ts->report_info),
							  ts->report_info.fingers[id].wx,
							  ts->report_info.fingers[id].wy,
							  ts->report_info.fingers[id].z);
		input_sync(ts->input);

		ts->report_info.fingers[id].x = SHTPS_TOUCH_CANCEL_COORDINATES_X;
		ts->report_info.fingers[id].y = SHTPS_TOUCH_CANCEL_COORDINATES_Y;
	}

	#if defined(SHTPS_PEN_DETECT_ENABLE)
		if(ts->report_info.fingers[id].state == SHTPS_TOUCH_STATE_PEN){
			shtps_report_touch_pen_on(ts, id,
								  SHTPS_TOUCH_CANCEL_COORDINATES_X,
								  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
								  shtps_get_fingerwidth(ts, id, &ts->report_info),
								  ts->report_info.fingers[id].wx,
								  ts->report_info.fingers[id].wy,
								  ts->report_info.fingers[id].z);
			input_sync(ts->input);

			ts->report_info.fingers[id].x = SHTPS_TOUCH_CANCEL_COORDINATES_X;
			ts->report_info.fingers[id].y = SHTPS_TOUCH_CANCEL_COORDINATES_Y;
		}
	#endif /* SHTPS_PEN_DETECT_ENABLE */
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_edge_fail_touch_set_inhibit_finger(struct shtps_rmi_spi *ts, u8 finger, u8 area)
{
	SHTPS_LOG_FUNC_CALL();
	ts->edge_fail_touch_rej_p->edge_fail_touch_inhibit_id |= (1 << ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[finger].id);
	
	#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
		if(area == 2){
			ts->edge_fail_touch_rej_p->right_edge_fail_touch_inhibit_id |= (1 << ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[finger].id);
			SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("right edge inhibit finger : 0x%02x\n", ts->edge_fail_touch_rej_p->right_edge_fail_touch_inhibit_id);
		}
	#endif /* SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */
	
	if(finger == 0){
		memcpy(&ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0], &ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1], sizeof(struct shtps_edge_fail_touch_info));
	}
	if(ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt > 0){
		ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt--;
	}
}

/* -------------------------------------------------------------------------- */
static u8 shtps_filter_edge_fail_touch_area_check(unsigned short x, unsigned short y)
{
	u8 area = 0;

	SHTPS_LOG_FUNC_CALL();
	if(y >= SHTPS_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_Y){
		if(x <= SHTPS_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_X){
			area = 1;
		}
		else if(x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_X)){
			area = 2;
		}
	}

	return area;
}

/* -------------------------------------------------------------------------- */
static int shtps_filter_edge_fail_touch_middle_area_check(unsigned short x, unsigned short y)
{
	int area = SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_NONE;

	SHTPS_LOG_FUNC_CALL();
	if( SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_AREA_TOP_ENABLE == 1){
		if(y <= SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_AREA_THRESH_Y){
			if( (x <= SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_AREA_THRESH_X) ||
				(x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_AREA_THRESH_X)) )
			{
				area |= SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_TOP;
			}
		}
	}

	if( SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_AREA_SIDE_ENABLE == 1){
		if(y <= SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_SIDE_AREA_THRESH_Y){
			if( (x <= SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_SIDE_AREA_THRESH_X) ||
				(x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_SIDE_AREA_THRESH_X)) )
			{
				area |= SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_TOP_SIDE;
			}
		}
	}

	if( SHTPS_EDGE_FAIL_TOUCH_REJECT_BOTTOM_AREA_SIDE_ENABLE == 1){
		if(y >= SHTPS_EDGE_FAIL_TOUCH_REJECT_BOTTOM_SIDE_AREA_THRESH_Y){
			if( (x <= SHTPS_EDGE_FAIL_TOUCH_REJECT_BOTTOM_SIDE_AREA_THRESH_X) ||
				(x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_EDGE_FAIL_TOUCH_REJECT_BOTTOM_SIDE_AREA_THRESH_X)) )
			{
				area |= SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_BOTTOM_SIDE;
			}
		}
	}

	return area;
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_edge_fail_touch_top_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);

	SHTPS_LOG_FUNC_CALL();

	if(SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_ENABLE == 0){
		return;
	}

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			if( (ts->edge_fail_touch_rej_p->edge_fail_touch_top_inhibit_id & (1 << i)) != 0 ){
				ts->edge_fail_touch_rej_p->edge_fail_touch_top_inhibit_id &= ~(1 << i);
				SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[top]inhibit clear id : %d\n", i);
			}
		}

		if( (ts->edge_fail_touch_rej_p->edge_fail_touch_top_inhibit_id & (1 << i)) != 0 ){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER)
			{
				int inhibit_cancel = 0;
				int now_area = SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_NONE;
				int td_area = SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_NONE;

				now_area = shtps_filter_edge_fail_touch_middle_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);
				td_area = shtps_filter_edge_fail_touch_middle_area_check(ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].x,
																		ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].y);

				if( ((now_area & SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_TOP) == 0) &&
					((now_area & SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_TOP_SIDE) == 0) )
				{
					inhibit_cancel = 1;
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[top][%d] inhibit cancel by out of area <x=%d><y=%d>\n",
															i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);
				}

				if( (td_area & SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_TOP_SIDE) != 0 ){
					if(SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_SIDE_AREA_INHIBIT_CANCEL_MOVE_THRESH_Y > 0)
					{
						int diff_y;

						diff_y = (ts->fw_report_info.fingers[i].y - ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].y);

						if(diff_y >= SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_SIDE_AREA_INHIBIT_CANCEL_MOVE_THRESH_Y){
							inhibit_cancel = 1;
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[top][%d] inhibit cancel by y move thresh <y=%d><diff_y=%d>\n", i, ts->fw_report_info.fingers[i].y, diff_y);
						}
					}
				}

				if(inhibit_cancel != 0){
					ts->edge_fail_touch_rej_p->edge_fail_touch_top_inhibit_id &= ~(1 << i);

					{
						struct shtps_touch_info rep_info;
						u8 event;

						memcpy(&rep_info, &ts->report_info, sizeof(struct shtps_touch_info));
						rep_info.fingers[i].state = ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].state;
						rep_info.fingers[i].x  = ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].x;
						rep_info.fingers[i].y  = ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].y;
						rep_info.fingers[i].wx = ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].wx;
						rep_info.fingers[i].wy = ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].wy;
						rep_info.fingers[i].z  = ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].z;

						#if defined(SHTPS_EDGE_POS_ADJUST_ENABLE)
							shtps_filter_touch_position_adjust_edge(ts, &rep_info);
						#endif	/* SHTPS_EDGE_POS_ADJUST_ENABLE */
						#if defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
							shtps_filter_touch_position_adjust_shift_edge(ts, &rep_info);
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

						SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("td report event [%d]<x=%d, y=%d, wx=%d, wy=%d, z=%d>\n",
																i, rep_info.fingers[i].x, rep_info.fingers[i].y,
																rep_info.fingers[i].wx, rep_info.fingers[i].wy, rep_info.fingers[i].z);

						shtps_event_report(ts, &rep_info, event);
					}
				}
			}
		}

		if( (ts->edge_fail_touch_rej_p->edge_fail_touch_top_inhibit_id & (1 << i)) == 0 ){
			if( (info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER) &&
				(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) )
			{
				int area = SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_NONE;

				ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].state = info->fingers[i].state;
				ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].x  = ts->fw_report_info.fingers[i].x;
				ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].y  = ts->fw_report_info.fingers[i].y;
				ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].wx = ts->fw_report_info.fingers[i].wx;
				ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].wy = ts->fw_report_info.fingers[i].wy;
				ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info.fingers[i].z  = ts->fw_report_info.fingers[i].z;

				area = shtps_filter_edge_fail_touch_middle_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);

				if( ((area & SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_TOP) != 0) ||
					((area & SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_TOP_SIDE) != 0) )
				{
//					if(ts->fw_report_info.fingers[i].z <= SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_AREA_THRESH_Z)
					if(SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_AREA_THRESH_ZW_RATIO > 0){
						u8 w = (ts->fw_report_info.fingers[i].wx > ts->fw_report_info.fingers[i].wy) ?
								ts->fw_report_info.fingers[i].wx : ts->fw_report_info.fingers[i].wy;

						if(w == 0){
							w = 1;
						}

						if( ((ts->fw_report_info.fingers[i].z * 100) / w) <= SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_AREA_THRESH_ZW_RATIO ){
							ts->edge_fail_touch_rej_p->edge_fail_touch_top_inhibit_id |= (1 << i);
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[top]inhibit [%d]<x=%d, y=%d, wx=%d, wy=%d, z=%d>\n",
																	i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y,
																	ts->fw_report_info.fingers[i].wx, ts->fw_report_info.fingers[i].wy, ts->fw_report_info.fingers[i].z);
						}
					}

					if(SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_AREA_RELATION_ZW_RATIO > 0){
						u8 w = (ts->fw_report_info.fingers[i].wx > ts->fw_report_info.fingers[i].wy) ?
								ts->fw_report_info.fingers[i].wx : ts->fw_report_info.fingers[i].wy;

						if(w > (((ts->fw_report_info.fingers[i].z * SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_AREA_RELATION_ZW_RATIO) / 100) - SHTPS_EDGE_FAIL_TOUCH_REJECT_TOP_AREA_RELATION_ZW_DIFF) ){
							ts->edge_fail_touch_rej_p->edge_fail_touch_top_inhibit_id |= (1 << i);
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[top]inhibit [%d]<x=%d, y=%d, wx=%d, wy=%d, z=%d>\n",
																	i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y,
																	ts->fw_report_info.fingers[i].wx, ts->fw_report_info.fingers[i].wy, ts->fw_report_info.fingers[i].z);
						}
					}
				}
			}
		}
	}

	for(i = 0; i < fingerMax; i++){
		if( (ts->edge_fail_touch_rej_p->edge_fail_touch_top_inhibit_id & (1 << i)) != 0 ){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[top][%d] is inhibited\n", i);
		}
	}
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_edge_fail_touch_bottom_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);

	SHTPS_LOG_FUNC_CALL();

	if(SHTPS_EDGE_FAIL_TOUCH_REJECT_BOTTOM_ENABLE == 0){
		return;
	}

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			if( (ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id & (1 << i)) != 0 ){
				ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id &= ~(1 << i);
				SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[bottom]inhibit clear id : %d\n", i);
			}
		}

		if( (ts->edge_fail_touch_rej_p->edge_fail_touch_inhibit_id & (1 << i)) != 0 ){
			if( (ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id & (1 << i)) != 0 ){
				ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id &= ~(1 << i);
				SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[bottom]inhibit clear id : %d\n", i);
			}
		}
		else{
			if( (ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id & (1 << i)) != 0 ){
				if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER)
				{
					int inhibit_cancel = 0;
					int now_area = SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_NONE;
					int td_area = SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_NONE;

					now_area = shtps_filter_edge_fail_touch_middle_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);
					td_area = shtps_filter_edge_fail_touch_middle_area_check(ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].x,
																			ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].y);

					if( ((now_area & SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_BOTTOM) == 0) &&
						((now_area & SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_BOTTOM_SIDE) == 0) )
					{
						inhibit_cancel = 1;
						SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[bottom][%d] inhibit cancel by out of area <x=%d><y=%d>\n",
																i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);
					}

					if( (td_area & SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_BOTTOM_SIDE) != 0 ){
						if(SHTPS_EDGE_FAIL_TOUCH_REJECT_BOTTOM_SIDE_AREA_INHIBIT_CANCEL_MOVE_THRESH_Y > 0)
						{
							int diff_y;

							diff_y = (ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].y - ts->fw_report_info.fingers[i].y);

							if(diff_y >= SHTPS_EDGE_FAIL_TOUCH_REJECT_BOTTOM_SIDE_AREA_INHIBIT_CANCEL_MOVE_THRESH_Y){
								inhibit_cancel = 1;
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[bottom][%d] inhibit cancel by y move thresh <y=%d><diff_y=%d>\n", i, ts->fw_report_info.fingers[i].y, diff_y);
							}
						}
					}

					if(inhibit_cancel != 0){
						ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id &= ~(1 << i);

						{
							struct shtps_touch_info rep_info;
							u8 event;

							memcpy(&rep_info, &ts->report_info, sizeof(struct shtps_touch_info));
							rep_info.fingers[i].state = ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].state;
							rep_info.fingers[i].x  = ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].x;
							rep_info.fingers[i].y  = ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].y;
							rep_info.fingers[i].wx = ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].wx;
							rep_info.fingers[i].wy = ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].wy;
							rep_info.fingers[i].z  = ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].z;

							#if defined(SHTPS_EDGE_POS_ADJUST_ENABLE)
								shtps_filter_touch_position_adjust_edge(ts, &rep_info);
							#endif	/* SHTPS_EDGE_POS_ADJUST_ENABLE */
							#if defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
								shtps_filter_touch_position_adjust_shift_edge(ts, &rep_info);
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

							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("td report event [%d]<x=%d, y=%d, wx=%d, wy=%d, z=%d>\n",
																	i, rep_info.fingers[i].x, rep_info.fingers[i].y,
																	rep_info.fingers[i].wx, rep_info.fingers[i].wy, rep_info.fingers[i].z);

							shtps_event_report(ts, &rep_info, event);
						}
					}
				}
			}

			if( (ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id & (1 << i)) == 0 ){
				if( (info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER) &&
					(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) )
				{
					int area = SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_NONE;

					ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].state = info->fingers[i].state;
					ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].x  = ts->fw_report_info.fingers[i].x;
					ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].y  = ts->fw_report_info.fingers[i].y;
					ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].wx = ts->fw_report_info.fingers[i].wx;
					ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].wy = ts->fw_report_info.fingers[i].wy;
					ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_td_info.fingers[i].z  = ts->fw_report_info.fingers[i].z;

					area = shtps_filter_edge_fail_touch_middle_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);

					if( ((area & SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_BOTTOM) != 0) ||
						((area & SHTPS_EDGE_FAIL_TOUCH_REJECT_MIDDLE_AREA_BOTTOM_SIDE) != 0) )
					{
						if(SHTPS_EDGE_FAIL_TOUCH_REJECT_BOTTOM_AREA_RELATION_ZW_RATIO > 0){
							u8 w = (ts->fw_report_info.fingers[i].wx > ts->fw_report_info.fingers[i].wy) ?
									ts->fw_report_info.fingers[i].wx : ts->fw_report_info.fingers[i].wy;

							if(w > (((ts->fw_report_info.fingers[i].z * SHTPS_EDGE_FAIL_TOUCH_REJECT_BOTTOM_AREA_RELATION_ZW_RATIO) / 100) - SHTPS_EDGE_FAIL_TOUCH_REJECT_BOTTOM_AREA_RELATION_ZW_DIFF) ){
								ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id |= (1 << i);
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[bottom]inhibit [%d]<x=%d, y=%d, wx=%d, wy=%d, z=%d>\n",
																		i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y,
																		ts->fw_report_info.fingers[i].wx, ts->fw_report_info.fingers[i].wy, ts->fw_report_info.fingers[i].z);
							}
						}
					}
				}
			}
		}
	}

	for(i = 0; i < fingerMax; i++){
		if( (ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id & (1 << i)) != 0 ){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[bottom][%d] is inhibited\n", i);
		}
	}
}

/* -------------------------------------------------------------------------- */
static u8 shtps_filter_edge_fail_touch_side_area_check(unsigned short x, unsigned short y)
{
	u8 is_side_area = 0;

	if( (x <= SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_AREA_THRESH_X) ||
		(x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_AREA_THRESH_X)) )
	{
		is_side_area = 1;
	}

	return is_side_area;
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_edge_fail_touch_side_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 fingerMax = shtps_get_fingermax(ts);

	SHTPS_LOG_FUNC_CALL();

	if(SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_ENABLE == 0){
		return;
	}

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			if( (ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id & (1 << i)) != 0 ){
				ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id &= ~(1 << i);
				SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[side]inhibit clear id : %d\n", i);
			}
		}

		if( (ts->edge_fail_touch_rej_p->edge_fail_touch_inhibit_id & (1 << i)) != 0 ){
			if( (ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id & (1 << i)) != 0 ){
				ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id &= ~(1 << i);
				SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[side]inhibit clear id : %d\n", i);
			}
		}
		else if( (ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id & (1 << i)) != 0 ){
			if( (ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id & (1 << i)) != 0 ){
				ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id &= ~(1 << i);
				SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[side]inhibit clear id : %d\n", i);
			}
		}
		else{
			if( (ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id & (1 << i)) != 0 ){
				if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
					if( shtps_filter_edge_fail_touch_side_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y) == 0 )
					{
						struct shtps_touch_info rep_info;
						u8 event;

						ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id &= ~(1 << i);

						memcpy(&rep_info, &ts->report_info, sizeof(struct shtps_touch_info));
						rep_info.fingers[i].state = ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info.fingers[i].state;
						rep_info.fingers[i].x  = ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info.fingers[i].x;
						rep_info.fingers[i].y  = ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info.fingers[i].y;
						rep_info.fingers[i].wx = ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info.fingers[i].wx;
						rep_info.fingers[i].wy = ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info.fingers[i].wy;
						rep_info.fingers[i].z  = ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info.fingers[i].z;

						#if defined(SHTPS_EDGE_POS_ADJUST_ENABLE)
							shtps_filter_touch_position_adjust_edge(ts, &rep_info);
						#endif	/* SHTPS_EDGE_POS_ADJUST_ENABLE */
						#if defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
							shtps_filter_touch_position_adjust_shift_edge(ts, &rep_info);
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

						SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("td report event [%d]<x=%d, y=%d, wx=%d, wy=%d, z=%d>\n",
																i, rep_info.fingers[i].x, rep_info.fingers[i].y,
																rep_info.fingers[i].wx, rep_info.fingers[i].wy, rep_info.fingers[i].z);

						shtps_event_report(ts, &rep_info, event);
					}
				}
			}

			if( (ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id & (1 << i)) == 0 ){
				if( (info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER) &&
					(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) )
				{
					if( (shtps_filter_edge_fail_touch_side_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y) != 0) )
					{
						u8 w = (ts->fw_report_info.fingers[i].wx > ts->fw_report_info.fingers[i].wy) ?
								ts->fw_report_info.fingers[i].wx : ts->fw_report_info.fingers[i].wy;

						if(w == 0){
							w = 1;
						}

						if(SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_AREA_THRESH_ZW_RATIO > 0){
							if( ((ts->fw_report_info.fingers[i].z * 100) / w) <= SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_AREA_THRESH_ZW_RATIO ){
								ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id |= (1 << i);
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[side]inhibit by z/w thresh (id : %d)\n", i);
							}
						}

						if(SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_AREA_THRESH_Z > 0){
							if(ts->fw_report_info.fingers[i].z <= SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_AREA_THRESH_Z){
								ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id |= (1 << i);
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[side]inhibit by z thresh (id : %d)\n", i);
							}
						}

						if(SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_AREA_THRESH_W > 0){
							if(w >= SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_AREA_THRESH_W){
								ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id |= (1 << i);
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[side]inhibit by w thresh (id : %d)\n", i);
							}
						}

						ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info.fingers[i].state = info->fingers[i].state;
						ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info.fingers[i].x  = ts->fw_report_info.fingers[i].x;
						ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info.fingers[i].y  = ts->fw_report_info.fingers[i].y;
						ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info.fingers[i].wx = ts->fw_report_info.fingers[i].wx;
						ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info.fingers[i].wy = ts->fw_report_info.fingers[i].wy;
						ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info.fingers[i].z  = ts->fw_report_info.fingers[i].z;
					}
				}
			}
		}
	}

	for(i = 0; i < fingerMax; i++){
		if( (ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id & (1 << i)) != 0 ){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[side][%d] is inhibited\n", i);
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_edge_fail_touch_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i, j;
	int fingerMax = shtps_get_fingermax(ts);

	if(SHTPS_EDGE_FAIL_TOUCH_REJECT_ENABLE == 0){
		return;
	}

	if(ts->edge_fail_touch_rej_p->edge_fail_touch_enable == 0){
		return;
	}

	SHTPS_LOG_FUNC_CALL();
	shtps_filter_edge_fail_touch_top_check(ts, info);

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			if( (ts->edge_fail_touch_rej_p->edge_fail_touch_inhibit_id & (1 << i)) != 0 ){
				ts->edge_fail_touch_rej_p->edge_fail_touch_inhibit_id &= ~(1 << i);
				SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("inhibit clear id : %d\n", i);
			}
			#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
				if( (ts->edge_fail_touch_rej_p->right_edge_fail_touch_inhibit_id & (1 << i)) != 0 ){
					ts->edge_fail_touch_rej_p->right_edge_fail_touch_inhibit_id &= ~(1 << i);
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("right edge inhibit finger : 0x%02x\n", ts->edge_fail_touch_rej_p->right_edge_fail_touch_inhibit_id);
				}
			#endif /* SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */
		}

		if( (ts->edge_fail_touch_rej_p->edge_fail_touch_inhibit_id & (1 << i)) != 0 ){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				if( shtps_filter_edge_fail_touch_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y) == 0 ){
					ts->edge_fail_touch_rej_p->edge_fail_touch_inhibit_id &= ~(1 << i);
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[%d] inhibit cancel by out of area <x=%d><y=%d>\n",
															i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);
				}
			}
		}

		if( (ts->edge_fail_touch_rej_p->edge_fail_touch_inhibit_id & (1 << i)) == 0 ){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER)
			{
				u8 already_add = 0;

				for(j = 0; j < ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt; j++){
					if(i == ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[j].id){
						already_add = 1;
					}
				}

				if(already_add == 0){
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[%d]td info add : x=%d, y=%d, z=%d\n", ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt,
															ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y, ts->fw_report_info.fingers[i].z);

					ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt].x  = ts->fw_report_info.fingers[i].x;
					ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt].y  = ts->fw_report_info.fingers[i].y;
					ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt].z  = ts->fw_report_info.fingers[i].z;
					ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt].id = i;
					ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt].time = jiffies;
					ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt++;
				}
			}
			else
			{
				if(ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt > 0)
				{
					int find_id = -1;

					for(j = 0; j < ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt; j++){
						if(i == ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[j].id){
							find_id = j;
						}
					}

					if(find_id >= 0){
						for(j = find_id; j < ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt; j++){
							if(j < (SHTPS_FINGER_MAX - 1)){
								memcpy(&ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[j], &ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[j + 1], sizeof(struct shtps_edge_fail_touch_info));
							}
						}

						SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[%d]td info del\n", find_id);
						if(ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt > 0){
							ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt--;
						}
					}
				}
			}
		}
	}

	if(ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt != 0){
		if(ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt < 2){
			ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 0;
			SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("mt decide cancel\n");
		}
	}

	if(ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt == 0){
		if(ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt > 2){
			ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
			ts->edge_fail_touch_rej_p->edge_fail_touch_pre_single = 0;
			SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("mt decide by td num = %d\n", ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt);
		}
		else if(ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt == 1){
			ts->edge_fail_touch_rej_p->edge_fail_touch_pre_single = 1;
		}
		else if(ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt == 2)
		{
			u8 td_area_1st;
			u8 td_area_2nd;
			int diff_x;
			int diff_y;
			int diff_x_2;
			int diff_y_2;
			u8 now_z_1;
			u8 now_z_2;
			int distance;
			u8 is_check_time_over = 0;

			SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("2nd td detect\n");

			td_area_1st = shtps_filter_edge_fail_touch_area_check(ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].x, ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].y);
			td_area_2nd = shtps_filter_edge_fail_touch_area_check(ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].x, ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].y);

			if(time_after(ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].time,
							ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].time + msecs_to_jiffies(SHTPS_EDGE_FAIL_TOUCH_REJECT_EFFECT_TIME)) == 0)
			{
				is_check_time_over = 0;
			}else{
				is_check_time_over = 1;
			}

			if(td_area_1st != 0){
				if(is_check_time_over != 0){
					ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide by 2nd td check time over\n");
				}
				else{
					diff_x = shtps_get_diff(ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].x, ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].x, SHTPS_POS_SCALE_X(ts));
					diff_y = shtps_get_diff(ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].y, ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].y, SHTPS_POS_SCALE_Y(ts));
					distance = (diff_x * diff_x) + (diff_y * diff_y);

					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]check start\n");

					if(td_area_2nd != 0){
						if(td_area_1st == td_area_2nd){
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]check area same side\n");

							if(distance >= (SHTPS_EDGE_FAIL_TOUCH_REJECT_DISTANCE * SHTPS_EDGE_FAIL_TOUCH_REJECT_DISTANCE)){
								if(ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].z < ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].z){
									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("use z ratio 2\n");
									if( ((ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].z * 100) / ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].z) >= SHTPS_EDGE_FAIL_TOUCH_REJECT_Z_RATIO_2 ){
										if(ts->edge_fail_touch_rej_p->edge_fail_touch_pre_single != 0){
											if( ((ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id & (1 << ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id)) == 0) ||
												((ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id & (1 << ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id)) == 0) )
											{
												SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("touch cancel event report\n");
												shtps_filter_event_touch_cancel(ts, ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id);
											}
										}

										SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]inhibit id : %d\n", ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id);
										shtps_filter_edge_fail_touch_set_inhibit_finger(ts, 0, td_area_1st);
									}
									else{
										ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
										SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
									}
								}
								else if(ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].z > ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].z){
									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("use z ratio 1\n");
									if( ((ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].z * 100) / ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].z) >= SHTPS_EDGE_FAIL_TOUCH_REJECT_Z_RATIO ){
										SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]inhibit id : %d\n", ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].id);
										shtps_filter_edge_fail_touch_set_inhibit_finger(ts, 1, td_area_2nd);
									}
									else{
										ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
										SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
									}
								}
								else{
									ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
								}
							}
							else{
								ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
							}
						}
						else{
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]check area opposite side\n");

							if(ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].z < ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].z){
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("use z ratio 2\n");
								if( ((ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].z * 100) / ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].z) >= SHTPS_EDGE_FAIL_TOUCH_REJECT_Z_RATIO_2 ){
									if(ts->edge_fail_touch_rej_p->edge_fail_touch_pre_single != 0){
										if( ((ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id & (1 << ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id)) == 0) ||
											((ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id & (1 << ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id)) == 0) )
										{
											SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("touch cancel event report\n");
											shtps_filter_event_touch_cancel(ts, ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id);
										}
									}

									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]inhibit id : %d\n", ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id);
									shtps_filter_edge_fail_touch_set_inhibit_finger(ts, 0, td_area_1st);
								}
								else{
									ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
								}
							}
							else if(ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].z > ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].z){
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("use z ratio 1\n");
								if( ((ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].z * 100) / ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].z) >= SHTPS_EDGE_FAIL_TOUCH_REJECT_Z_RATIO ){
									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]inhibit id : %d\n", ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].id);
									shtps_filter_edge_fail_touch_set_inhibit_finger(ts, 1, td_area_2nd);
								}
								else{
									ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
								}
							}
							else{
								ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
							}
						}
					}
					else{
						SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]check area out side\n");

						if(distance >= (SHTPS_EDGE_FAIL_TOUCH_REJECT_DISTANCE * SHTPS_EDGE_FAIL_TOUCH_REJECT_DISTANCE)){
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("use z ratio 2\n");
							if( ((ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].z * 100) / ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].z) >= SHTPS_EDGE_FAIL_TOUCH_REJECT_Z_RATIO_2 ){
								if(ts->edge_fail_touch_rej_p->edge_fail_touch_pre_single != 0){
									if( ((ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id & (1 << ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id)) == 0) ||
										((ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id & (1 << ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id)) == 0) )
									{
										SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("touch cancel event report\n");
										shtps_filter_event_touch_cancel(ts, ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id);
									}
								}

								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]inhibit id : %d\n", ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id);
								shtps_filter_edge_fail_touch_set_inhibit_finger(ts, 0, td_area_1st);
							}
							else{
								ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
							}
						}
						else{
							ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
						}
					}
				}
			}
			else{
				if(td_area_2nd != 0){
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-2]check start\n");

					diff_x_2 = shtps_get_diff(ts->fw_report_info.fingers[ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id].x,
												ts->fw_report_info.fingers[ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].id].x, SHTPS_POS_SCALE_X(ts));
					diff_y_2 = shtps_get_diff(ts->fw_report_info.fingers[ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id].y,
												ts->fw_report_info.fingers[ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].id].y, SHTPS_POS_SCALE_Y(ts));
					distance = (diff_x_2 * diff_x_2) + (diff_y_2 * diff_y_2);

					now_z_1 = ts->fw_report_info.fingers[ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id].z;
					now_z_2 = ts->fw_report_info.fingers[ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].id].z;

					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("info check [1]<x=%d, y=%d, z=%d> [2]<x=%d, y=%d, z=%d>\n",
														ts->fw_report_info.fingers[ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id].x,
														ts->fw_report_info.fingers[ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id].y,
														ts->fw_report_info.fingers[ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[0].id].z,
														ts->fw_report_info.fingers[ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].id].x,
														ts->fw_report_info.fingers[ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].id].y,
														ts->fw_report_info.fingers[ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].id].z);

					if(distance >= (SHTPS_EDGE_FAIL_TOUCH_REJECT_DISTANCE * SHTPS_EDGE_FAIL_TOUCH_REJECT_DISTANCE)){
						SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("use z ratio 1\n");
						if( ((now_z_1 * 100) / now_z_2) >= SHTPS_EDGE_FAIL_TOUCH_REJECT_Z_RATIO ){
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-2]inhibit add id : %d\n", ts->edge_fail_touch_rej_p->edge_fail_touch_td_info[1].id);
							shtps_filter_edge_fail_touch_set_inhibit_finger(ts, 1, td_area_2nd);
						}
						else{
							ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-2]mt decide\n");
						}
					}
					else{
						ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
						SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-2]mt decide\n");
					}
				}
				else{
					ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 1;
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("mt decide by check area no td\n");
				}
			}

			if(ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt == 1){
				ts->edge_fail_touch_rej_p->edge_fail_touch_pre_single = 1;
			}else{
				ts->edge_fail_touch_rej_p->edge_fail_touch_pre_single = 0;
			}
		}
		else{
			ts->edge_fail_touch_rej_p->edge_fail_touch_pre_single = 0;
		}
	}

	shtps_filter_edge_fail_touch_bottom_check(ts, info);
	shtps_filter_edge_fail_touch_side_check(ts, info);

	for(i = 0; i < fingerMax; i++){
		if( (ts->edge_fail_touch_rej_p->edge_fail_touch_inhibit_id & (1 << i)) != 0 ){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			SHTPS_LOG_DBG_PRINT("[edge_fail_reject][%d] is inhibited\n", i);
		}
	}
}

/* -------------------------------------------------------------------------- */
int shtps_filter_edge_fail_touch_check_inhibit(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
		if( (SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_DISABLE == 0) &&
			(ts->edge_fail_touch_rej_p->right_edge_fail_touch_inhibit_id != 0) ){
			//
			return 1;
		}else{
			return 0;
		}
	#else
		return 0;
	#endif
}

/* -------------------------------------------------------------------------- */
void shtps_filter_edge_fail_touch_switch(struct shtps_rmi_spi *ts, int on)
{
	if(on == 0){
		ts->edge_fail_touch_rej_p->edge_fail_touch_enable = 0;
	}else{
		ts->edge_fail_touch_rej_p->edge_fail_touch_enable = 1;
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_edge_fail_touch_sleep(struct shtps_rmi_spi *ts)
{
	ts->edge_fail_touch_rej_p->edge_fail_touch_inhibit_id = 0;
	ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 0;
	ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt = 0;
	ts->edge_fail_touch_rej_p->edge_fail_touch_top_inhibit_id = 0;
	ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id = 0;
	ts->edge_fail_touch_rej_p->edge_fail_touch_bottom_inhibit_id = 0;

	#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
		ts->edge_fail_touch_rej_p->right_edge_fail_touch_inhibit_id = 0;
	#endif /* SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */
}

/* -------------------------------------------------------------------------- */
void shtps_filter_edge_fail_touch_init(struct shtps_rmi_spi *ts)
{
	ts->edge_fail_touch_rej_p = kzalloc(sizeof(struct shtps_filter_edge_fail_touch_rej_info), GFP_KERNEL);
	if(ts->edge_fail_touch_rej_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}

	ts->edge_fail_touch_rej_p->edge_fail_touch_enable = 1;
#if 0 // no need
	ts->edge_fail_touch_rej_p->edge_fail_touch_inhibit_id = 0;
	ts->edge_fail_touch_rej_p->edge_fail_touch_decide_mt = 0;
	ts->edge_fail_touch_rej_p->edge_fail_touch_td_cnt = 0;
	ts->edge_fail_touch_rej_p->edge_fail_touch_top_inhibit_id = 0;
	ts->edge_fail_touch_rej_p->edge_fail_touch_side_inhibit_id = 0;
	memset(ts->edge_fail_touch_rej_p->edge_fail_touch_td_info, 0, sizeof(ts->edge_fail_touch_rej_p->edge_fail_touch_td_info));
	memset(&ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info, 0, sizeof(ts->edge_fail_touch_rej_p->edge_fail_touch_top_td_info));
	memset(&ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info, 0, sizeof(ts->edge_fail_touch_rej_p->edge_fail_touch_side_td_info));

	#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
		ts->edge_fail_touch_rej_p->right_edge_fail_touch_inhibit_id = 0;
	#endif /* SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */
#endif
}

/* -------------------------------------------------------------------------- */
void shtps_filter_edge_fail_touch_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->edge_fail_touch_rej_p)	kfree(ts->edge_fail_touch_rej_p);
	ts->edge_fail_touch_rej_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */
