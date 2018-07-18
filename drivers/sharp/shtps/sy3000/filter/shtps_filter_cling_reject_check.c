/* drivers/sharp/shtps/sy3000/shtps_filter_cling_reject_check.c
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

#if defined(SHTPS_CLING_REJECTION_ENABLE)
struct shtps_filter_cling_reject_info{
	struct shtps_touch_pos_info		hover_base_pos[SHTPS_FINGER_MAX];
	u8								hover_base_pos_decide[SHTPS_FINGER_MAX];
	u8								hover_base_pos_change_check[SHTPS_FINGER_MAX];
	unsigned long					hover_tu_time[SHTPS_FINGER_MAX];
	unsigned long					hover_detect_time[SHTPS_FINGER_MAX];
	u8								hover_rest_check[SHTPS_FINGER_MAX];
	u16								hover_anchor_cnt[SHTPS_FINGER_MAX];
	u8								hover_anchor_cnt_state[SHTPS_FINGER_MAX];
	u16								hover_level_jump_cnt[SHTPS_FINGER_MAX];
	u8								hover_level_jump_cnt_state[SHTPS_FINGER_MAX];
	u16								hover_riot_jump_cnt[SHTPS_FINGER_MAX];
	struct shtps_touch_pos_info		finger_tu_pos[SHTPS_FINGER_MAX];
	unsigned long					last_finger_tu_time;
	u8								finger_tu_hover_check;

	u16								finger_jump_cnt[SHTPS_FINGER_MAX];
	u8								finger_jump_cnt_state[SHTPS_FINGER_MAX];
	unsigned long					finger_tu_time[SHTPS_FINGER_MAX];
	u8								finger_tu_finger_td_check;
	struct shtps_touch_hist_info	finger_tu_info[SHTPS_FINGER_MAX];

	u8								static_pos_check_state;
	unsigned long					static_pos_time[SHTPS_FINGER_MAX];
};

/* -------------------------------------------------------------------------- */
static u8 shtps_filter_cling_reject_check_hover(struct shtps_rmi_spi *ts)
{
	int i, j;
	int fingerMax = shtps_get_fingermax(ts);
	int diff_x;
	int diff_y;
	int diff_x_2;
	int diff_y_2;
	u8 is_cling_detect = 0;
	struct shtps_filter_cling_reject_info *cling_reject_p = ts->cling_reject_p;

	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_CLING_REJECT("check hover\n");

	if(SHTPS_CLING_REJECT_HOVER_ENABLE == 0){
		return 0;
	}

	if(SHTPS_CLING_REJECT_HOVER_MODE_4_ENABLE != 0)
	{
		u8 is_old_notouch = 1;
		u8 is_finger_detect = 0;

		for(i = 0; i < fingerMax; i++){
			if(ts->fw_report_info_store.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				is_old_notouch = 0;
			}

			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				is_finger_detect = 1;
			}
		}

		if( (is_old_notouch != 0) && (is_finger_detect != 0) ){
			if(cling_reject_p->finger_tu_hover_check != 0){
				cling_reject_p->finger_tu_hover_check = 0;
				SHTPS_LOG_CLING_REJECT("finger tu check clear by finger td\n");
			}
		}
	}

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
				cling_reject_p->hover_tu_time[i] = jiffies;
				cling_reject_p->hover_base_pos_change_check[i] = 1;
				SHTPS_LOG_CLING_REJECT("[%d]hover tu detect\n", i);
			}

			if(SHTPS_CLING_REJECT_HOVER_MODE_4_ENABLE != 0){
				if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
					cling_reject_p->last_finger_tu_time = jiffies;
					cling_reject_p->finger_tu_hover_check |= (1 << i);
					cling_reject_p->finger_tu_pos[i].x = ts->fw_report_info_store.fingers[i].x;
					cling_reject_p->finger_tu_pos[i].y = ts->fw_report_info_store.fingers[i].y;
					SHTPS_LOG_CLING_REJECT("[%d]finger tu detect <x=%d, y=%d>\n",
							i, cling_reject_p->finger_tu_pos[i].x, cling_reject_p->finger_tu_pos[i].y);
				}
			}

			cling_reject_p->hover_riot_jump_cnt[i] = 0;
			cling_reject_p->hover_rest_check[i] = 0;
		}
		else if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
			SHTPS_LOG_CLING_REJECT("[%d]hover detect <x=%d, y=%d>\n", i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);

			if(SHTPS_CLING_REJECT_HOVER_MODE_1_ENABLE != 0){
				if(cling_reject_p->hover_rest_check[i] == 0){
					cling_reject_p->hover_detect_time[i] = jiffies;
					cling_reject_p->hover_rest_check[i] = 1;
				}
				else{
					if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
						if( time_after(jiffies, cling_reject_p->hover_detect_time[i] + msecs_to_jiffies(SHTPS_CLING_REJECT_HOVER_REST_TIME)) != 0 ){
							is_cling_detect = 1;
							SHTPS_LOG_CLING_REJECT("[%d]cling detect by hover rest detect\n", i);
						}
					}
					cling_reject_p->hover_detect_time[i] = jiffies;
				}
			}

			if(SHTPS_CLING_REJECT_HOVER_MODE_4_ENABLE != 0)
			{
				u8 is_near_detect = 0;
				u8 is_far_detect = 0;

				if(cling_reject_p->finger_tu_hover_check != 0){
					if( time_after(jiffies, cling_reject_p->last_finger_tu_time + msecs_to_jiffies(SHTPS_CLING_REJECT_FINGER_TU_HOVER_TD_TIME)) == 0 ){
						for(j = 0; j < fingerMax; j++){
							if(((cling_reject_p->finger_tu_hover_check >> j) & 0x01) != 0 ){
								diff_x = shtps_get_diff(ts->fw_report_info.fingers[i].x, cling_reject_p->finger_tu_pos[j].x, SHTPS_POS_SCALE_X(ts));
								diff_y = shtps_get_diff(ts->fw_report_info.fingers[i].y, cling_reject_p->finger_tu_pos[j].y, SHTPS_POS_SCALE_Y(ts));

								SHTPS_LOG_CLING_REJECT("[%d]finger tu - hover pos <diff_x=%d, diff_y=%d>\n", i, diff_x, diff_y);

								if( (diff_x >= SHTPS_CLING_REJECT_FINGER_TU_HOVER_TD_DISTANCE_X) || (diff_y >= SHTPS_CLING_REJECT_FINGER_TU_HOVER_TD_DISTANCE_Y) ){
									is_far_detect = 1;
								}else{
									is_near_detect = 1;
								}
							}
						}
					}

					if( (is_near_detect == 0) && (is_far_detect != 0) ){
						is_cling_detect = 1;
						SHTPS_LOG_CLING_REJECT("[%d]cling detect by finger tu - hover td far + fast\n", i);
					}

					cling_reject_p->finger_tu_hover_check = 0;
				}
			}

			if(cling_reject_p->hover_base_pos_change_check[i] != 0){
				if( time_after(jiffies, cling_reject_p->hover_tu_time[i] + msecs_to_jiffies(SHTPS_CLING_REJECT_HOVER_BASE_POS_CHANGE_TIME)) != 0 ){
					cling_reject_p->hover_base_pos_decide[i] = 0;
					SHTPS_LOG_CLING_REJECT("[%d]base pos decide clear\n", i);
				}
				cling_reject_p->hover_base_pos_change_check[i] = 0;
			}

			if(cling_reject_p->hover_base_pos_decide[i] == 0){
				cling_reject_p->hover_base_pos[i].x = ts->fw_report_info.fingers[i].x;
				cling_reject_p->hover_base_pos[i].y = ts->fw_report_info.fingers[i].y;
				cling_reject_p->hover_base_pos_decide[i] = 1;
				cling_reject_p->hover_anchor_cnt_state[i] = 1;
				cling_reject_p->hover_anchor_cnt[i] = 0;
				cling_reject_p->hover_level_jump_cnt[i] = 0;
				cling_reject_p->hover_level_jump_cnt_state[i] = 1;
				SHTPS_LOG_CLING_REJECT("[%d]base pos decide <x=%d, y=%d>\n",
						i, cling_reject_p->hover_base_pos[i].x, cling_reject_p->hover_base_pos[i].y);
			}
			else{
				diff_x = shtps_get_diff(ts->fw_report_info.fingers[i].x, cling_reject_p->hover_base_pos[i].x, SHTPS_POS_SCALE_X(ts));
				diff_y = shtps_get_diff(ts->fw_report_info.fingers[i].y, cling_reject_p->hover_base_pos[i].y, SHTPS_POS_SCALE_Y(ts));
				diff_x_2 = shtps_get_diff(ts->fw_report_info.fingers[i].x, ts->fw_report_info_store.fingers[i].x, SHTPS_POS_SCALE_X(ts));
				diff_y_2 = shtps_get_diff(ts->fw_report_info.fingers[i].y, ts->fw_report_info_store.fingers[i].y, SHTPS_POS_SCALE_Y(ts));
				SHTPS_LOG_CLING_REJECT("[%d]base diff (diff_x=%d, diff_y=%d)\n", i, diff_x, diff_y);
				SHTPS_LOG_CLING_REJECT("[%d]detect diff (diff_x=%d, diff_y=%d)\n", i, diff_x_2, diff_y_2);

				if(SHTPS_CLING_REJECT_HOVER_MODE_1_ENABLE != 0){
					if(cling_reject_p->hover_anchor_cnt_state[i] != 0){
						if( (diff_x <= SHTPS_CLING_REJECT_HOVER_ANCHOR_AREA_X) && (diff_y <= SHTPS_CLING_REJECT_HOVER_ANCHOR_AREA_Y) ){
							cling_reject_p->hover_anchor_cnt[i]++;
							SHTPS_LOG_CLING_REJECT("[%d]anchor count up <%d>\n", i, cling_reject_p->hover_anchor_cnt[i]);
						}
						else{
							cling_reject_p->hover_anchor_cnt_state[i] = 0;
							cling_reject_p->hover_anchor_cnt[i] = 0;
							SHTPS_LOG_CLING_REJECT("[%d]anchor count end by area over\n", i);
						}
					}
					if(cling_reject_p->hover_anchor_cnt[i] >= SHTPS_CLING_REJECT_HOVER_ANCHOR_CNT_MAX){
						is_cling_detect = 1;
						SHTPS_LOG_CLING_REJECT("[%d]cling detect by anchor count over\n", i);
					}
				}

				if(SHTPS_CLING_REJECT_HOVER_MODE_2_ENABLE != 0){
					if(cling_reject_p->hover_level_jump_cnt_state[i] != 0){
						if(diff_y < SHTPS_CLING_REJECT_HOVER_LEVEL_JUMP_AREA_Y){
							if(diff_x_2 >= SHTPS_CLING_REJECT_HOVER_LEVEL_JUMP_THRESH_X){
								cling_reject_p->hover_level_jump_cnt[i]++;
								SHTPS_LOG_CLING_REJECT("[%d]level jump count up <%d>\n", i, cling_reject_p->hover_level_jump_cnt[i]);
							}
						}
						else{
							cling_reject_p->hover_level_jump_cnt_state[i] = 0;
							cling_reject_p->hover_level_jump_cnt[i] = 0;
							SHTPS_LOG_CLING_REJECT("[%d]level jump count end by area over\n", i);
						}
					}
					if(cling_reject_p->hover_level_jump_cnt[i] >= SHTPS_CLING_REJECT_HOVER_LEVEL_JUMP_CNT_MAX){
						is_cling_detect = 1;
						SHTPS_LOG_CLING_REJECT("[%d]cling detect by level jump count over\n", i);
					}
				}

				if(SHTPS_CLING_REJECT_HOVER_MODE_3_ENABLE != 0){
					if( (diff_x_2 >= SHTPS_CLING_REJECT_HOVER_RIOT_JUMP_THRESH_X) && (diff_y_2 >= SHTPS_CLING_REJECT_HOVER_RIOT_JUMP_THRESH_Y) ){
						cling_reject_p->hover_riot_jump_cnt[i]++;
						SHTPS_LOG_CLING_REJECT("[%d]riot jump count up <%d>\n", i, cling_reject_p->hover_riot_jump_cnt[i]);
					}
					if(cling_reject_p->hover_riot_jump_cnt[i] >= SHTPS_CLING_REJECT_HOVER_RIOT_JUMP_CNT_MAX){
						is_cling_detect = 1;
						SHTPS_LOG_CLING_REJECT("[%d]cling detect by level riot jump count over\n", i);
					}
				}
			}
		}
		else{
			cling_reject_p->hover_anchor_cnt_state[i] = 0;
			cling_reject_p->hover_anchor_cnt[i] = 0;
			cling_reject_p->hover_level_jump_cnt_state[i] = 0;
			cling_reject_p->hover_level_jump_cnt[i] = 0;
			cling_reject_p->hover_riot_jump_cnt[i] = 0;
			cling_reject_p->hover_base_pos_change_check[i] = 0;
			cling_reject_p->hover_base_pos_decide[i] = 0;
			cling_reject_p->hover_rest_check[i] = 0;
			cling_reject_p->finger_tu_hover_check &= ~(1 << i);
		}
	}

	return is_cling_detect;
}

/* -------------------------------------------------------------------------- */
static u8 shtps_filter_cling_reject_check_finger(struct shtps_rmi_spi *ts)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	u8 is_cling_detect = 0;
	struct shtps_filter_cling_reject_info *cling_reject_p = ts->cling_reject_p;

	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_CLING_REJECT("check finger\n");

	if(SHTPS_CLING_REJECT_FINGER_ENABLE == 0){
		return 0;
	}

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			SHTPS_LOG_CLING_REJECT("[%d] check info <x=%d, y=%d, wx=%d, wy=%d, z=%d>\n",
											i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y,
											ts->fw_report_info.fingers[i].wx, ts->fw_report_info.fingers[i].wy,
											ts->fw_report_info.fingers[i].z);
		}
	}

	if(SHTPS_CLING_REJECT_FINGER_MODE_1_ENABLE != 0){
		for(i = 0; i < fingerMax; i++){
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				if( (ts->fw_report_info.fingers[i].wx >= SHTPS_CLING_REJECT_FINGER_DETECT_THRESH_W) ||
					(ts->fw_report_info.fingers[i].wy >= SHTPS_CLING_REJECT_FINGER_DETECT_THRESH_W) )
				{
					is_cling_detect = 1;
					SHTPS_LOG_CLING_REJECT("[%d] cling detect by finger w threshold over\n", i);
				}
			}
		}

	}

	if(SHTPS_CLING_REJECT_FINGER_MODE_2_ENABLE != 0){
		for(i = 0; i < fingerMax; i++){
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				if( (ts->fw_report_info.fingers[i].wx >= SHTPS_CLING_REJECT_FINGER_DETECT_THRESH_W) ||
					(ts->fw_report_info.fingers[i].wy >= SHTPS_CLING_REJECT_FINGER_DETECT_THRESH_W) )
				{
					if( (ts->fw_report_info.fingers[i].wx == 0) ||
						(ts->fw_report_info.fingers[i].wy == 0) )
					{
						is_cling_detect = 1;
						SHTPS_LOG_CLING_REJECT("[%d] cling detect by finger w threshold over + one side zero\n", i);
					}
				}
			}
		}
	}

	if(SHTPS_CLING_REJECT_FINGER_MODE_3_ENABLE != 0)
	{
		int diff_x;
		int diff_y;
		u8 diff_wx;
		u8 diff_wy;
		u8 diff_z;

		for(i = 0; i < fingerMax; i++){
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
					cling_reject_p->finger_tu_time[i] = jiffies;
					cling_reject_p->finger_tu_finger_td_check |= (1 << i);
					cling_reject_p->finger_tu_info[i].x  = ts->fw_report_info_store.fingers[i].x;
					cling_reject_p->finger_tu_info[i].y  = ts->fw_report_info_store.fingers[i].y;
					cling_reject_p->finger_tu_info[i].wx = ts->fw_report_info_store.fingers[i].wx;
					cling_reject_p->finger_tu_info[i].wy = ts->fw_report_info_store.fingers[i].wy;
					cling_reject_p->finger_tu_info[i].z  = ts->fw_report_info_store.fingers[i].z;

					SHTPS_LOG_CLING_REJECT("[%d] finger tu info <x=%d, y=%d, wx=%d, wy=%d, z=%d>\n",
							i, cling_reject_p->finger_tu_info[i].x, cling_reject_p->finger_tu_info[i].y,
							cling_reject_p->finger_tu_info[i].wx, cling_reject_p->finger_tu_info[i].wy,
							cling_reject_p->finger_tu_info[i].z);
				}
				else{
					cling_reject_p->finger_jump_cnt_state[i] = 0;
					cling_reject_p->finger_jump_cnt[i] = 0;
					cling_reject_p->finger_tu_finger_td_check &= ~(1 << i);
				}
			}
			else if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				if( ((cling_reject_p->finger_tu_finger_td_check >> i) & 0x01) != 0 ){
					if( time_after(jiffies, cling_reject_p->finger_tu_time[i] + msecs_to_jiffies(SHTPS_CLING_REJECT_FINGER_TU_NEGLECT_TIME_MS)) != 0 ){
						cling_reject_p->finger_jump_cnt_state[i] = 0;
						cling_reject_p->finger_jump_cnt[i] = 0;
						SHTPS_LOG_CLING_REJECT("[%d] finger jump count clear by time over\n", i);
					}
					cling_reject_p->finger_tu_finger_td_check &= ~(1 << i);

					diff_x = shtps_get_diff(ts->fw_report_info.fingers[i].x, cling_reject_p->finger_tu_info[i].x, SHTPS_POS_SCALE_X(ts));
					diff_y = shtps_get_diff(ts->fw_report_info.fingers[i].y, cling_reject_p->finger_tu_info[i].y, SHTPS_POS_SCALE_Y(ts));
					diff_wx = (ts->fw_report_info.fingers[i].wx > cling_reject_p->finger_tu_info[i].wx)?
								(ts->fw_report_info.fingers[i].wx - cling_reject_p->finger_tu_info[i].wx) :
								(cling_reject_p->finger_tu_info[i].wx - ts->fw_report_info.fingers[i].wx);
					diff_wy = (ts->fw_report_info.fingers[i].wy > cling_reject_p->finger_tu_info[i].wy)?
								(ts->fw_report_info.fingers[i].wy - cling_reject_p->finger_tu_info[i].wy) :
								(cling_reject_p->finger_tu_info[i].wy - ts->fw_report_info.fingers[i].wy);
					diff_z = (ts->fw_report_info.fingers[i].z > cling_reject_p->finger_tu_info[i].z)?
								(ts->fw_report_info.fingers[i].z - cling_reject_p->finger_tu_info[i].z) :
								(cling_reject_p->finger_tu_info[i].z - ts->fw_report_info.fingers[i].z);
				}
				else if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
					diff_x = shtps_get_diff(ts->fw_report_info.fingers[i].x, ts->fw_report_info_store.fingers[i].x, SHTPS_POS_SCALE_X(ts));
					diff_y = shtps_get_diff(ts->fw_report_info.fingers[i].y, ts->fw_report_info_store.fingers[i].y, SHTPS_POS_SCALE_Y(ts));
					diff_wx = (ts->fw_report_info.fingers[i].wx > ts->fw_report_info_store.fingers[i].wx)?
								(ts->fw_report_info.fingers[i].wx - ts->fw_report_info_store.fingers[i].wx) :
								(ts->fw_report_info_store.fingers[i].wx - ts->fw_report_info.fingers[i].wx);
					diff_wy = (ts->fw_report_info.fingers[i].wy > ts->fw_report_info_store.fingers[i].wy)?
								(ts->fw_report_info.fingers[i].wy - ts->fw_report_info_store.fingers[i].wy) :
								(ts->fw_report_info_store.fingers[i].wy - ts->fw_report_info.fingers[i].wy);
					diff_z = (ts->fw_report_info.fingers[i].z > ts->fw_report_info_store.fingers[i].z)?
								(ts->fw_report_info.fingers[i].z - ts->fw_report_info_store.fingers[i].z) :
								(ts->fw_report_info_store.fingers[i].z - ts->fw_report_info.fingers[i].z);
				}
				else{
					diff_x  = 0;
					diff_y  = 0;
					diff_wx = 0;
					diff_wy = 0;
					diff_z  = 0;
				}

				SHTPS_LOG_CLING_REJECT("[%d] finger diff info <x=%d, y=%d, wx=%d, wy=%d, z=%d>\n",
											i, diff_x, diff_y, diff_wx, diff_wy, diff_z);

				if( ((diff_x >= SHTPS_CLING_REJECT_FINGER_JUMP_THRESH_X) ||
					 (diff_y >= SHTPS_CLING_REJECT_FINGER_JUMP_THRESH_Y))
					&&
					((diff_wx >= SHTPS_CLING_REJECT_FINGER_DETECT_THRESH_DIFF_W) ||
					 (diff_wy >= SHTPS_CLING_REJECT_FINGER_DETECT_THRESH_DIFF_W) ||
					 (diff_z >= SHTPS_CLING_REJECT_FINGER_DETECT_THRESH_DIFF_Z)) )
				{
					cling_reject_p->finger_jump_cnt[i]++;
					SHTPS_LOG_CLING_REJECT("[%d] finger jump count up <%d>\n", i, cling_reject_p->finger_jump_cnt[i]);
				}

				if(cling_reject_p->finger_jump_cnt[i] >= SHTPS_CLING_REJECT_FINGER_JUMP_CNT_MAX){
					is_cling_detect = 1;
					SHTPS_LOG_CLING_REJECT("[%d] cling detect by finger jump count max\n", i);
				}
			}
			else{
				cling_reject_p->finger_jump_cnt_state[i] = 0;
				cling_reject_p->finger_jump_cnt[i] = 0;
				cling_reject_p->finger_tu_finger_td_check &= ~(1 << i);
			}
		}
	}

	if(SHTPS_CLING_REJECT_FINGER_MODE_4_ENABLE != 0)
	{
		int numOfFingers = 0;

		for(i = 0; i < fingerMax; i++){
			if( (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) ||
				(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PALM) )
			{
				numOfFingers++;
			}
		}

		if(cling_reject_p->static_pos_check_state == 0){
			if(numOfFingers >= SHTPS_CLING_REJECT_FINGER_STATIC_NUM_THRESH){
				for(i = 0; i < fingerMax; i++){
					if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
						cling_reject_p->static_pos_time[i] = jiffies;
					}
				}

				cling_reject_p->static_pos_check_state = 1;
				SHTPS_LOG_CLING_REJECT("static pos check start\n");
			}
		}
		else{
			if(numOfFingers < SHTPS_CLING_REJECT_FINGER_STATIC_NUM_THRESH){
				cling_reject_p->static_pos_check_state = 0;
				SHTPS_LOG_CLING_REJECT("static pos check end by few detected finger numbers\n");
			}
			else{
				for(i = 0; i < fingerMax; i++){
					if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
						if(ts->fw_report_info_store.fingers[i].state != SHTPS_TOUCH_STATE_FINGER){
							cling_reject_p->static_pos_time[i] = jiffies;
							SHTPS_LOG_CLING_REJECT("[%d]static time set by state change\n", i);
						}
						else{
							if( (ts->fw_report_info.fingers[i].x == ts->fw_report_info_store.fingers[i].x) &&
								(ts->fw_report_info.fingers[i].y == ts->fw_report_info_store.fingers[i].y) &&
								(SHTPS_ABS_CALC(ts->fw_report_info.fingers[i].wx, ts->fw_report_info_store.fingers[i].wx) <= SHTPS_CLING_REJECT_FINGER_STATIC_DIFF_W) &&
								(SHTPS_ABS_CALC(ts->fw_report_info.fingers[i].wy, ts->fw_report_info_store.fingers[i].wy) <= SHTPS_CLING_REJECT_FINGER_STATIC_DIFF_W) &&
								(SHTPS_ABS_CALC(ts->fw_report_info.fingers[i].z, ts->fw_report_info_store.fingers[i].z) <= SHTPS_CLING_REJECT_FINGER_STATIC_DIFF_Z) )
							{
								if( time_after(jiffies, cling_reject_p->static_pos_time[i] + msecs_to_jiffies(SHTPS_CLING_REJECT_FINGER_STATIC_TIME_MS)) != 0 ){
									is_cling_detect = 1;
									SHTPS_LOG_CLING_REJECT("[%d] cling detect by static pos\n", i);
								}
							}
							else{
								cling_reject_p->static_pos_time[i] = jiffies;
								SHTPS_LOG_CLING_REJECT("[%d]static time set by touch info\n", i);
							}
						}
					}
				}
			}
		}
	}

	return is_cling_detect;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_cling_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	u8 is_cling_detect = 0;

	SHTPS_LOG_FUNC_CALL();
	if( shtps_filter_cling_reject_check_hover(ts) != 0 ){
		is_cling_detect = 1;
	}

	if( shtps_filter_cling_reject_check_finger(ts) != 0 ){
		is_cling_detect = 1;
	}

	if(is_cling_detect != 0){
		SHTPS_LOG_CLING_REJECT("rezero execute\n");
		shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_REZERO, 0);
		memset(ts->cling_reject_p, 0, sizeof(struct shtps_filter_cling_reject_info));
		//memset(&ts->cling_reject, 0, sizeof(struct shtps_filter_cling_reject_info));

		shtps_event_force_touchup(ts);
		memset(info, 0, sizeof(struct shtps_touch_info));
		#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
			shtps_key_event_force_touchup(ts);
		#endif /* SHTPS_PHYSICAL_KEY_ENABLE */
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_cling_reject_init(struct shtps_rmi_spi *ts)
{
	ts->cling_reject_p = kzalloc(sizeof(struct shtps_filter_cling_reject_info), GFP_KERNEL);
	if(ts->cling_reject_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(ts->cling_reject_p, 0, sizeof(struct shtps_filter_cling_reject_info));	// no need
}

/* -------------------------------------------------------------------------- */
void shtps_filter_cling_reject_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->cling_reject_p)	kfree(ts->cling_reject_p);
	ts->cling_reject_p = NULL;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_cling_reject_sleep(struct shtps_rmi_spi *ts)
{
	memset(ts->cling_reject_p, 0, sizeof(struct shtps_filter_cling_reject_info));
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_CLING_REJECTION_ENABLE */
