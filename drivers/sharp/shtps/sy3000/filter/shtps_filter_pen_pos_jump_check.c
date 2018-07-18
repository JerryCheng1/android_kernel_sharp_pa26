/* drivers/sharp/shtps/sy3000/shtps_filter_pen_pos_jump_check.c
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
#if defined(SHTPS_PEN_POS_JUMP_REJECT_ENABLE)
struct shtps_pen_pos_jump_hist{
	unsigned short	x;
	unsigned short	y;
	u8				is_td;
};

struct shtps_hold_info{
	unsigned short	x;
	unsigned short	y;
	unsigned char	wx;
	unsigned char	wy;
	unsigned char	z;
};

struct shtps_filter_pen_pos_jump_info{
	struct shtps_pen_pos_jump_hist	pen_pos_jump_hist[SHTPS_FINGER_MAX][SHTPS_PEN_POS_JUMP_HIST_MAX];
	u8								pen_pos_jump_hist_count[SHTPS_FINGER_MAX];
	struct shtps_hold_info			pen_pos_jump_hold_info[SHTPS_FINGER_MAX][SHTPS_PEN_POS_JUMP_HOLD_MAX];
	u8								pen_pos_jump_hold_info_count[SHTPS_FINGER_MAX];
	u8								pen_pos_jump_hold_state[SHTPS_FINGER_MAX];
	unsigned long					pen_pos_jump_hold_time[SHTPS_FINGER_MAX];
	u8								pen_pos_jump_td_check_count[SHTPS_FINGER_MAX];
	u8								pen_pos_jump_td_check_state[SHTPS_FINGER_MAX];
};

/* -------------------------------------------------------------------------- */
void shtps_filter_pen_pos_jump_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	if(SHTPS_HOST_PEN_JUMP_REJECTION_ENABLE != 0)
	{
		int i, j;
		int fingerMax = shtps_get_fingermax(ts);
		u8 is_pen_detect = 0;
		u8 is_palm_detect = 0;

		for(i = 0; i < fingerMax; i++){
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN){
				is_pen_detect = 1;
			}

			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PALM){
				is_palm_detect = 1;
			}
		}

		for(i = 0; i < fingerMax; i++){
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				ts->pen_pos_jump_p->pen_pos_jump_hist_count[i] = 0;
				ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i] = 0;
				ts->pen_pos_jump_p->pen_pos_jump_hold_state[i] = 0;
				ts->pen_pos_jump_p->pen_pos_jump_td_check_count[i] = 0;
				ts->pen_pos_jump_p->pen_pos_jump_td_check_state[i] = 0;
			}

			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN)
			{
				if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
					ts->pen_pos_jump_p->pen_pos_jump_td_check_state[i] = 1;
				}

				if(ts->pen_pos_jump_p->pen_pos_jump_td_check_count[i] < SHTPS_PEN_POS_JUMP_TD_CHECK_COUNT_MAX){
					ts->pen_pos_jump_p->pen_pos_jump_td_check_count[i]++;
				}else{
					ts->pen_pos_jump_p->pen_pos_jump_td_check_state[i] = 0;
				}

				if(ts->pen_pos_jump_p->pen_pos_jump_hold_state[i] == 0){
					if(ts->pen_pos_jump_p->pen_pos_jump_hist_count[i] < SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX){
						ts->pen_pos_jump_p->pen_pos_jump_hist[i][ts->pen_pos_jump_p->pen_pos_jump_hist_count[i]].x = ts->fw_report_info.fingers[i].x;
						ts->pen_pos_jump_p->pen_pos_jump_hist[i][ts->pen_pos_jump_p->pen_pos_jump_hist_count[i]].y = ts->fw_report_info.fingers[i].y;
						if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
							ts->pen_pos_jump_p->pen_pos_jump_hist[i][ts->pen_pos_jump_p->pen_pos_jump_hist_count[i]].is_td = 1;
						}else{
							ts->pen_pos_jump_p->pen_pos_jump_hist[i][ts->pen_pos_jump_p->pen_pos_jump_hist_count[i]].is_td = 0;
						}
						ts->pen_pos_jump_p->pen_pos_jump_hist_count[i]++;
						SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> hist add [count=%d][x=%d][y=%d]\n",
												i, ts->pen_pos_jump_p->pen_pos_jump_hist_count[i], ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);
					}
					else{
						for(j = 1; j < SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX; j++){
							ts->pen_pos_jump_p->pen_pos_jump_hist[i][j - 1].x = ts->pen_pos_jump_p->pen_pos_jump_hist[i][j].x;
							ts->pen_pos_jump_p->pen_pos_jump_hist[i][j - 1].y = ts->pen_pos_jump_p->pen_pos_jump_hist[i][j].y;
							ts->pen_pos_jump_p->pen_pos_jump_hist[i][j - 1].is_td = ts->pen_pos_jump_p->pen_pos_jump_hist[i][j].is_td;
						}
						ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 1].x = ts->fw_report_info.fingers[i].x;
						ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 1].y = ts->fw_report_info.fingers[i].y;
						if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
							ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 1].is_td = 1;
						}else{
							ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 1].is_td = 0;
						}

						SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> hist update [count=%d][x=%d][y=%d]\n",
												i, ts->pen_pos_jump_p->pen_pos_jump_hist_count[i], ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);
					}
				}
				else{
					ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 1].x = ts->fw_report_info.fingers[i].x;
					ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 1].y = ts->fw_report_info.fingers[i].y;
					if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
						ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 1].is_td = 1;
					}else{
						ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 1].is_td = 0;
					}

					SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> hist update during hold [count=%d][x=%d][y=%d]\n",
											i, ts->pen_pos_jump_p->pen_pos_jump_hist_count[i], ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);
				}
			}

			if( (is_pen_detect != 0) && (is_palm_detect == 0) &&
				(ts->pen_pos_jump_p->pen_pos_jump_hist_count[i] >= SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX) &&
				(SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX > 2) )
			{
				int dx, dy;
				int ddx, ddy;
				u8 is_pen_jump = 0;

				dx = ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 3].x - ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 2].x;
				dy = ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 3].y - ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 2].y;

				ddx = dx - (ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 2].x - ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 1].x);
				ddy = dy - (ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 2].y - ts->pen_pos_jump_p->pen_pos_jump_hist[i][SHTPS_PEN_POS_JUMP_HIST_COUNT_MAX - 1].y);

				SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> pen pos jump acceleration = %d\n", i, (ddx * ddx) + (ddy * ddy));

				if( (ddx * ddx) + (ddy * ddy) > SHTPS_PEN_POS_JUMP_ACCELERATION_THRESHOLD ){
					is_pen_jump = 1;
				}

				if(ts->pen_pos_jump_p->pen_pos_jump_hold_state[i] == 0){
					if(is_pen_jump != 0){
						if(ts->pen_pos_jump_p->pen_pos_jump_td_check_state[i] != 0)
						{
							ts->pen_pos_jump_p->pen_pos_jump_td_check_count[i] = 0;

							shtps_report_touch_pen_on(ts, i,
													  SHTPS_TOUCH_CANCEL_COORDINATES_X,
													  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
													  shtps_get_fingerwidth(ts, i, &ts->fw_report_info),
													  ts->fw_report_info.fingers[i].wx,
													  ts->fw_report_info.fingers[i].wy,
													  ts->fw_report_info.fingers[i].z);
							input_sync(ts->input);

							SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> touch cancel notify event x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
													i, SHTPS_TOUCH_CANCEL_COORDINATES_X, SHTPS_TOUCH_CANCEL_COORDINATES_Y,
													ts->fw_report_info.fingers[i].wx, ts->fw_report_info.fingers[i].wy,
													ts->fw_report_info.fingers[i].z);

						#if defined(SHTPS_PEN_DETECT_ENABLE)
							shtps_report_touch_pen_off(ts, i,
													  ts->fw_report_info.fingers[i].x,
													  ts->fw_report_info.fingers[i].y,
													  shtps_get_fingerwidth(ts, i, &ts->fw_report_info),
													  ts->fw_report_info.fingers[i].wx,
													  ts->fw_report_info.fingers[i].wy,
													  ts->fw_report_info.fingers[i].z);
							input_sync(ts->input);

							SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> touch cancel tu notify event x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
													i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y,
													ts->fw_report_info.fingers[i].wx, ts->fw_report_info.fingers[i].wy,
													ts->fw_report_info.fingers[i].z);

							shtps_report_touch_pen_on(ts, i,
													  ts->fw_report_info_store.fingers[i].x,
													  ts->fw_report_info_store.fingers[i].y,
													  shtps_get_fingerwidth(ts, i, &ts->fw_report_info_store),
													  ts->fw_report_info_store.fingers[i].wx,
													  ts->fw_report_info_store.fingers[i].wy,
													  ts->fw_report_info_store.fingers[i].z);
							input_sync(ts->input);
						#endif
							SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> after touch cancel notify event 1 x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
													i, ts->fw_report_info_store.fingers[i].x, ts->fw_report_info_store.fingers[i].y,
													ts->fw_report_info_store.fingers[i].wx, ts->fw_report_info_store.fingers[i].wy,
													ts->fw_report_info_store.fingers[i].z);

							shtps_report_touch_pen_on(ts, i,
													  ts->fw_report_info.fingers[i].x,
													  ts->fw_report_info.fingers[i].y,
													  shtps_get_fingerwidth(ts, i, &ts->fw_report_info),
													  ts->fw_report_info.fingers[i].wx,
													  ts->fw_report_info.fingers[i].wy,
													  ts->fw_report_info.fingers[i].z);
							input_sync(ts->input);

							SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> after touch cancel notify event 2 x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
													i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y,
													ts->fw_report_info.fingers[i].wx, ts->fw_report_info.fingers[i].wy,
													ts->fw_report_info.fingers[i].z);

							if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
								ts->touch_state.numOfFingers++;
							}

							ts->report_info.fingers[i].state = SHTPS_TOUCH_STATE_PEN;
							ts->report_info.fingers[i].x     = ts->fw_report_info.fingers[i].x;
							ts->report_info.fingers[i].y     = ts->fw_report_info.fingers[i].y;
							ts->report_info.fingers[i].wx    = ts->fw_report_info.fingers[i].wx;
							ts->report_info.fingers[i].wy    = ts->fw_report_info.fingers[i].wy;
							ts->report_info.fingers[i].z     = ts->fw_report_info.fingers[i].z;

							info->fingers[i].state = SHTPS_TOUCH_STATE_PEN;
							info->fingers[i].x     = ts->fw_report_info.fingers[i].x;
							info->fingers[i].y     = ts->fw_report_info.fingers[i].y;
							info->fingers[i].wx    = ts->fw_report_info.fingers[i].wx;
							info->fingers[i].wy    = ts->fw_report_info.fingers[i].wy;
							info->fingers[i].z     = ts->fw_report_info.fingers[i].z;
						}
						else{
							ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i] = 0;

							ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i]].x  = ts->fw_report_info.fingers[i].x;
							ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i]].y  = ts->fw_report_info.fingers[i].y;
							ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i]].wx = ts->fw_report_info.fingers[i].wx;
							ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i]].wy = ts->fw_report_info.fingers[i].wy;
							ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i]].z  = ts->fw_report_info.fingers[i].z;

							info->fingers[i].state = ts->report_info.fingers[i].state;
							info->fingers[i].x     = ts->report_info.fingers[i].x;
							info->fingers[i].y     = ts->report_info.fingers[i].y;
							info->fingers[i].wx    = ts->report_info.fingers[i].wx;
							info->fingers[i].wy    = ts->report_info.fingers[i].wy;
							info->fingers[i].z     = ts->report_info.fingers[i].z;

							ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i] = 1;
							ts->pen_pos_jump_p->pen_pos_jump_hold_state[i] = 1;
							ts->pen_pos_jump_p->pen_pos_jump_hold_time[i] = jiffies + msecs_to_jiffies(SHTPS_PEN_POS_JUMP_HOLD_EVENT_TIME_MAX);

							SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> touch info hold [1] %dms start x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
															i,
															SHTPS_PEN_POS_JUMP_HOLD_EVENT_TIME_MAX,
															ts->fw_report_info.fingers[i].x,
															ts->fw_report_info.fingers[i].y,
															ts->fw_report_info.fingers[i].wx,
															ts->fw_report_info.fingers[i].wy,
															ts->fw_report_info.fingers[i].z);
						}
					}
				}
				else{
					if( time_after(jiffies, ts->pen_pos_jump_p->pen_pos_jump_hold_time[i]) == 0 ){
						if( (ddx * ddx) + (ddy * ddy) <= SHTPS_PEN_POS_JUMP_HOLD_ACCELERATION_THRESHOLD ){
							ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i] = 0;
							ts->pen_pos_jump_p->pen_pos_jump_hold_state[i] = 0;
							SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> touch hold end by acceleration threshold\n", i);
						}
						else{
							if(ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i] < SHTPS_PEN_POS_JUMP_HOLD_MAX){
								ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i]].x  = ts->fw_report_info.fingers[i].x;
								ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i]].y  = ts->fw_report_info.fingers[i].y;
								ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i]].wx = ts->fw_report_info.fingers[i].wx;
								ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i]].wy = ts->fw_report_info.fingers[i].wy;
								ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i]].z  = ts->fw_report_info.fingers[i].z;

								info->fingers[i].state = ts->report_info.fingers[i].state;
								info->fingers[i].x     = ts->report_info.fingers[i].x;
								info->fingers[i].y     = ts->report_info.fingers[i].y;
								info->fingers[i].wx    = ts->report_info.fingers[i].wx;
								info->fingers[i].wy    = ts->report_info.fingers[i].wy;
								info->fingers[i].z     = ts->report_info.fingers[i].z;

								ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i]++;

								SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> touch info hold add [%d] x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
																i,
																ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i],
																ts->fw_report_info.fingers[i].x,
																ts->fw_report_info.fingers[i].y,
																ts->fw_report_info.fingers[i].wx,
																ts->fw_report_info.fingers[i].wy,
																ts->fw_report_info.fingers[i].z);
							}
						}
					}
					else{
						u8 is_hold_info_drag = 0;

						SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> touch hold end by time over\n", i);

						if(ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i] >= 2)
						{
							int diff_x;
							int diff_y;

							diff_x = shtps_get_diff(ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i] - 1].x,
													ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i] - 2].x,
													SHTPS_POS_SCALE_X(ts));
							diff_y = shtps_get_diff(ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i] - 1].y,
													ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i] - 2].y,
													SHTPS_POS_SCALE_Y(ts));

							if( ((diff_x < SHTPS_PEN_POS_JUMP_HOLD_DRAG_THRESHOLD_MIN) && (diff_y < SHTPS_PEN_POS_JUMP_HOLD_DRAG_THRESHOLD_MIN)) || 
								((diff_x > SHTPS_PEN_POS_JUMP_HOLD_DRAG_THRESHOLD_MAX) || (diff_y > SHTPS_PEN_POS_JUMP_HOLD_DRAG_THRESHOLD_MAX)) ){
								is_hold_info_drag = 0;
							}else{
								is_hold_info_drag = 1;
							}
						}

						if(is_hold_info_drag != 0){
							for(j = 0; j < ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i]; j++){
								int w;

								w = (ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].wx >= ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].wy) ?
										ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].wx : ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].wx;

								if(w < SHTPS_FINGER_WIDTH_MIN){
									w = SHTPS_FINGER_WIDTH_MIN;
								}

								shtps_report_touch_pen_on(ts, i,
														  ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].x,
														  ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].y,
														  w,
														  ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].wx,
														  ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].wy,
														  ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].z);

								input_sync(ts->input);

								SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> touch hold notify event x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
																i,
																ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].x,
																ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].y,
																ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].wx,
																ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].wy, 
																ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].z);

								if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
									ts->touch_state.numOfFingers++;
								}

								ts->report_info.fingers[i].state = SHTPS_TOUCH_STATE_PEN;
								ts->report_info.fingers[i].x     = ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].x;
								ts->report_info.fingers[i].y     = ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].y;
								ts->report_info.fingers[i].wx    = ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].wx;
								ts->report_info.fingers[i].wy    = ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].wy;
								ts->report_info.fingers[i].z     = ts->pen_pos_jump_p->pen_pos_jump_hold_info[i][j].z;
							}
						}
						else{
							shtps_report_touch_pen_off(ts, i,
													  ts->fw_report_info.fingers[i].x,
													  ts->fw_report_info.fingers[i].y,
													  shtps_get_fingerwidth(ts, i, &ts->fw_report_info),
													  ts->fw_report_info.fingers[i].wx,
													  ts->fw_report_info.fingers[i].wy,
													  ts->fw_report_info.fingers[i].z);
							input_sync(ts->input);

							SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> touch cancel tu notify event x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
													i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y,
													ts->fw_report_info.fingers[i].wx, ts->fw_report_info.fingers[i].wy,
													ts->fw_report_info.fingers[i].z);

							shtps_report_touch_pen_on(ts, i,
													  ts->fw_report_info.fingers[i].x,
													  ts->fw_report_info.fingers[i].y,
													  shtps_get_fingerwidth(ts, i, &ts->fw_report_info),
													  ts->fw_report_info.fingers[i].wx,
													  ts->fw_report_info.fingers[i].wy,
													  ts->fw_report_info.fingers[i].z);
							input_sync(ts->input);

							SHTPS_LOG_PEN_POS_JUMP_REJECT("<%d> after touch cancel notify event x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
													i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y,
													ts->fw_report_info.fingers[i].wx, ts->fw_report_info.fingers[i].wy,
													ts->fw_report_info.fingers[i].z);

							if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
								ts->touch_state.numOfFingers++;
							}

							ts->report_info.fingers[i].state = SHTPS_TOUCH_STATE_PEN;
							ts->report_info.fingers[i].x     = ts->fw_report_info.fingers[i].x;
							ts->report_info.fingers[i].y     = ts->fw_report_info.fingers[i].y;
							ts->report_info.fingers[i].wx    = ts->fw_report_info.fingers[i].wx;
							ts->report_info.fingers[i].wy    = ts->fw_report_info.fingers[i].wy;
							ts->report_info.fingers[i].z     = ts->fw_report_info.fingers[i].z;

							info->fingers[i].state = SHTPS_TOUCH_STATE_PEN;
							info->fingers[i].x     = ts->fw_report_info.fingers[i].x;
							info->fingers[i].y     = ts->fw_report_info.fingers[i].y;
							info->fingers[i].wx    = ts->fw_report_info.fingers[i].wx;
							info->fingers[i].wy    = ts->fw_report_info.fingers[i].wy;
							info->fingers[i].z     = ts->fw_report_info.fingers[i].z;

							ts->pen_pos_jump_p->pen_pos_jump_td_check_count[i] = 0;
							ts->pen_pos_jump_p->pen_pos_jump_td_check_state[i] = 1;
						}

						ts->pen_pos_jump_p->pen_pos_jump_hist_count[i] = 0;
						ts->pen_pos_jump_p->pen_pos_jump_hold_info_count[i] = 0;
						ts->pen_pos_jump_p->pen_pos_jump_hold_state[i] = 0;
					}
				}
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pen_pos_jump_sleep(struct shtps_rmi_spi *ts)
{
	memset(ts->pen_pos_jump_p->pen_pos_jump_hist_count, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_hist_count));
	memset(ts->pen_pos_jump_p->pen_pos_jump_hold_info_count, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_hold_info_count));
	memset(ts->pen_pos_jump_p->pen_pos_jump_hold_state, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_hold_state));
	memset(ts->pen_pos_jump_p->pen_pos_jump_td_check_count, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_td_check_count));
	memset(ts->pen_pos_jump_p->pen_pos_jump_td_check_state, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_td_check_state));
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pen_pos_jump_init(struct shtps_rmi_spi *ts)
{
	ts->pen_pos_jump_p = kzalloc(sizeof(struct shtps_filter_pen_pos_jump_info), GFP_KERNEL);
	if(ts->pen_pos_jump_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	#if 0	// no need
	memset(ts->pen_pos_jump_p->pen_pos_jump_hist, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_hist));
	memset(ts->pen_pos_jump_p->pen_pos_jump_hist_count, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_hist_count));
	memset(&ts->pen_pos_jump_p->pen_pos_jump_hold_info, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_hold_info));
	memset(&ts->pen_pos_jump_p->pen_pos_jump_hold_info_count, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_hold_info_count));
	memset(ts->pen_pos_jump_p->pen_pos_jump_hold_state, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_hold_state));
	memset(ts->pen_pos_jump_p->pen_pos_jump_hold_time, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_hold_time));
	memset(ts->pen_pos_jump_p->pen_pos_jump_td_check_count, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_td_check_count));
	memset(ts->pen_pos_jump_p->pen_pos_jump_td_check_state, 0, sizeof(ts->pen_pos_jump_p->pen_pos_jump_td_check_state));
	#endif
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pen_pos_jump_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->pen_pos_jump_p)	kfree(ts->pen_pos_jump_p);
	ts->pen_pos_jump_p = NULL;
}
/* -------------------------------------------------------------------------- */
#endif /* SHTPS_PEN_POS_JUMP_REJECT_ENABLE */
