/* drivers/sharp/shtps/sy3000/shtps_filter_drag_s3400.c
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
struct shtps_drag_hist{
	int							pre;
	u8							dir;
	u8							count;
	#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
		int						history[SHTPS_DRAG_HISTORY_SIZE_MAX];
		int						pre_comp_history_FIXED;
		int						history_old;
		int						count_up_base;
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
};

struct shtps_filter_drag_hist_info{
	struct shtps_touch_info		center_info;
	struct shtps_drag_hist	drag_hist[SHTPS_FINGER_MAX][2];
};
/* -------------------------------------------------------------------------- */
#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
#if defined( SHTPS_LOG_DEBUG_ENABLE )
int SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(int val)
{
	int i;
	int dec_val = (val) - SHTPS_DRAG_SMOOTH_INT_TO_FIXED(SHTPS_DRAG_SMOOTH_FIXED_TO_INT((val)));
	int ret = 0;
	int add_val = 10;
	for(i = 0; i < SHTPS_DRAG_SMOOTH_FIXED_SHIFT; i++){
		add_val *= 10;
	}
	for(i = 1; i < SHTPS_DRAG_SMOOTH_FIXED_SHIFT; i++){
		add_val /= 2;
		if((dec_val >> (SHTPS_DRAG_SMOOTH_FIXED_SHIFT - i)) & 0x01){
			ret += add_val;
		}
	}
	return ret/1000000;
}
#endif /* SHTPS_LOG_DEBUG_ENABLE */
static void shtps_filter_pos_compensation(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, int xy)
{
	int inc_ave_FIXED,temp_FIXED;
	int i;
	int drag_smooth_leave_max_dot_FIXED;
	int last_history;
	int last_history_FIXED;

	if(!SHTPS_DRAG_SMOOTH){
		return;
	}

	for(i = 0;i < shtps_get_fingermax(ts);i++){
		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER /* info->fingers[i].state == SHTPS_TOUCH_STATE_PEN */){
			if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				if(ts->drag_hist_p->drag_hist[i][xy].count >= SHTPS_DRAG_SMOOTH_COUNT_MIN){
					drag_smooth_leave_max_dot_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(SHTPS_DRAG_SMOOTH_LEAVE_MAX_DOT);
					last_history		= ts->drag_hist_p->drag_hist[i][xy].history[ts->drag_hist_p->drag_hist[i][xy].count - 1];
					last_history_FIXED	= SHTPS_DRAG_SMOOTH_INT_TO_FIXED(last_history);

					inc_ave_FIXED = 
						SHTPS_DRAG_SMOOTH_INT_TO_FIXED(
							last_history- ts->drag_hist_p->drag_hist[i][xy].history[0]) / 
							(ts->drag_hist_p->drag_hist[i][xy].count-1);
					SHTPS_LOG_DRAG_SMOOTH("   [X]inc_ave=%d.%03d history[0]=%d\n", 
						SHTPS_DRAG_SMOOTH_FIXED_TO_INT(inc_ave_FIXED), 
						SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(inc_ave_FIXED), 
						ts->drag_hist_p->drag_hist[i][xy].history[0]);
						
					if(xy == SHTPS_POSTYPE_X){
						if(ts->drag_hist_p->drag_hist[i][xy].history_old == last_history){
							info->fingers[i].x = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED);
						}else{
							if(((ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) > drag_smooth_leave_max_dot_FIXED){
								temp_FIXED = last_history_FIXED + drag_smooth_leave_max_dot_FIXED;
								SHTPS_LOG_DRAG_SMOOTH("   [X]temp=%d.%03d(upper limit)\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}else if(((ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) < (0-drag_smooth_leave_max_dot_FIXED)){
								temp_FIXED = last_history_FIXED - drag_smooth_leave_max_dot_FIXED;
								SHTPS_LOG_DRAG_SMOOTH("   [X]temp=%d.%03d(lower limit)\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}else{
								temp_FIXED = ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED;
								SHTPS_LOG_DRAG_SMOOTH("   [X]temp=%d.%03d\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}
							
							if(temp_FIXED < 0){
								info->fingers[i].x = 0;
								ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED = 0;
							}else if(temp_FIXED >= SHTPS_DRAG_SMOOTH_INT_TO_FIXED(CONFIG_SHTPS_SY3000_PANEL_SIZE_X)){
								info->fingers[i].x = CONFIG_SHTPS_SY3000_PANEL_SIZE_X -1;
								ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].x);
							} else{
								info->fingers[i].x = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED);
								ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED = temp_FIXED;
							}
							ts->drag_hist_p->drag_hist[i][xy].history_old = last_history;
						}
					}else{
						if(ts->drag_hist_p->drag_hist[i][xy].history_old == last_history){
							info->fingers[i].y = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED);
						}else{
							if(((ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) > drag_smooth_leave_max_dot_FIXED){
								temp_FIXED = last_history_FIXED + drag_smooth_leave_max_dot_FIXED;
								SHTPS_LOG_DRAG_SMOOTH("   [Y]temp=%d.%03d(upper limit)\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}else if(((ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) < (0-drag_smooth_leave_max_dot_FIXED)){
								temp_FIXED = last_history_FIXED - drag_smooth_leave_max_dot_FIXED;
								SHTPS_LOG_DRAG_SMOOTH("   [Y]temp=%d.%03d(lower limit)\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}else{
								temp_FIXED = ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED;
								SHTPS_LOG_DRAG_SMOOTH("   [Y]temp=%d.%03d\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}

							if(temp_FIXED < 0){
								info->fingers[i].y = 0;
								ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED = 0;
							}else if(temp_FIXED >= SHTPS_DRAG_SMOOTH_INT_TO_FIXED(CONFIG_SHTPS_SY3000_PANEL_SIZE_Y)){
								info->fingers[i].y = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y -1;
								ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].y);
							} else{
								info->fingers[i].y = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED);
								ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED = temp_FIXED;
							}
							ts->drag_hist_p->drag_hist[i][xy].history_old = last_history;
						}
					}
				}
			}
		}
	}
}
#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
/* -------------------------------------------------------------------------- */
#if defined( SHTPS_DRAG_STEP_ENABLE )
/* -------------------------------------------------------------------------- */
static void shtps_rec_notify_time(struct shtps_rmi_spi *ts, int xy, int index)
{
//	SHTPS_LOG_FUNC_CALL();
	ts->touch_state.drag_timeout[index][xy] = jiffies + msecs_to_jiffies(ts->touch_state.dragStepReturnTime[index][xy]);
}

/* -------------------------------------------------------------------------- */
static int shtps_chk_notify_time(struct shtps_rmi_spi *ts, int xy, int index)
{
//	SHTPS_LOG_FUNC_CALL();
	if(time_after(jiffies, ts->touch_state.drag_timeout[index][xy])){
		return -1;
	}
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_get_dragstep(struct shtps_rmi_spi *ts, int xy, int type, int fingers, int state)
{
	int dragStep;

	if((SHTPS_DRAG_STEP_FINGER_ENABLE == 0) && (state == SHTPS_TOUCH_STATE_FINGER)){
		return 1;
	}
	if((SHTPS_DRAG_STEP_PEN_ENABLE == 0) && (state == SHTPS_TOUCH_STATE_PEN)){
		return 1;
	}

	if(type == SHTPS_DRAG_THRESHOLD_ZERO){
		if(xy == SHTPS_POSTYPE_X){
			if(state == SHTPS_TOUCH_STATE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_ZERO : SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_X_ZERO : SHTPS_PEN_DRAG_THRESH_VAL_X_1ST_MULTI;
			}
		}else{
			if(state == SHTPS_TOUCH_STATE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_ZERO : SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_Y_ZERO : SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST_MULTI;
			}
		}
	}else if(type == SHTPS_DRAG_THRESHOLD_1ST){
		if(xy == SHTPS_POSTYPE_X){
			if(state == SHTPS_TOUCH_STATE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_1ST : SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_X_1ST : SHTPS_PEN_DRAG_THRESH_VAL_X_1ST_MULTI;
			}
		}else{
			if(state == SHTPS_TOUCH_STATE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_1ST : SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST : SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST_MULTI;
			}
		}
	}else{
		if(xy == SHTPS_POSTYPE_X){
			if(state == SHTPS_TOUCH_STATE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_2ND : SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_X_2ND : SHTPS_PEN_DRAG_THRESH_VAL_X_2ND_MULTI;
			}
		}else{
			if(state == SHTPS_TOUCH_STATE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_2ND : SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_Y_2ND : SHTPS_PEN_DRAG_THRESH_VAL_Y_2ND_MULTI;
			}
		}
	}

	return dragStep;
}

/* -------------------------------------------------------------------------- */
static void shtps_set_dragstep(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, int type, int xy, int finger)
{
//	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__SET_DRAG_STEP, "%d|%d|%d", type, xy, finger);

	if(type == SHTPS_DRAG_THRESHOLD_ZERO){
		ts->touch_state.dragStep[finger][xy] = type;
		ts->touch_state.dragStepReturnTime[finger][xy] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
	}else if(type == SHTPS_DRAG_THRESHOLD_1ST){
		ts->touch_state.dragStep[finger][xy] = type;
		ts->touch_state.dragStepReturnTime[finger][xy] = SHTPS_DRAG_THRESH_RETURN_TIME;
	}else{
		if(xy == SHTPS_POSTYPE_X){
			_log_msg_sync( LOGMSG_ID__SET_FINGER_CENTER, "%d|%d|%d", xy,
							info->fingers[finger].x * SHTPS_POS_SCALE_X(ts) / 10000,
							info->fingers[finger].x);
			ts->drag_hist_p->center_info.fingers[finger].x = info->fingers[finger].x;
		}else{
			_log_msg_sync( LOGMSG_ID__SET_FINGER_CENTER, "%d|%d|%d", xy,
							info->fingers[finger].y * SHTPS_POS_SCALE_Y(ts) / 10000,
							info->fingers[finger].y);
			ts->drag_hist_p->center_info.fingers[finger].y = info->fingers[finger].y;
		}
		ts->touch_state.dragStep[finger][xy] = type;
		shtps_rec_notify_time(ts, xy, finger);
	}
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_DRAG_STEP_ENABLE */

/* -------------------------------------------------------------------------- */
static void shtps_init_drag_hist(struct shtps_rmi_spi *ts, int xy, int finger, int pos)
{
//	SHTPS_LOG_FUNC_CALL();
	ts->drag_hist_p->drag_hist[finger][xy].pre   = pos;
	ts->drag_hist_p->drag_hist[finger][xy].count = 0;
	#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
		ts->drag_hist_p->drag_hist[finger][xy].count_up_base = 0;
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
}

/* -------------------------------------------------------------------------- */
static void shtps_add_drag_hist(struct shtps_rmi_spi *ts, int xy, int finger, int pos)
{
	int pre = ts->drag_hist_p->drag_hist[finger][xy].pre;
	u8 dir  = (pos > pre)? SHTPS_DRAG_DIR_PLUS :
			  (pos < pre)? SHTPS_DRAG_DIR_MINUS :
						   SHTPS_DRAG_DIR_NONE;

	#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
		int i;
		int drag_smooth_count_limit;
		int drag_smooth_count_limit_new;
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

//	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_DBG_PRINT("add drag hist[%d][%s] pre = %d, cur = %d, dir = %s, cnt = %d, remain time = %d\n",
		finger, (xy == SHTPS_POSTYPE_X)? "X" : "Y",
		pre, pos, 
		(dir == SHTPS_DRAG_DIR_PLUS)? "PLUS" : (dir == SHTPS_DRAG_DIR_MINUS)? "MINUS" : "NONE",
		ts->drag_hist_p->drag_hist[finger][xy].count,
		time_after(jiffies, ts->touch_state.drag_timeout[finger][xy]));

	if(dir != SHTPS_DRAG_DIR_NONE){
		if(ts->drag_hist_p->drag_hist[finger][xy].count == 0){
			#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
				if(SHTPS_DRAG_SMOOTH){
					#if defined( SHTPS_DEVELOP_MODE_ENABLE )
						if(SHTPS_DRAG_SMOOTH_COUNT_MIN > SHTPS_DRAG_HISTORY_SIZE_MAX){
							SHTPS_DRAG_SMOOTH_COUNT_MIN = SHTPS_DRAG_HISTORY_SIZE_MAX;
						}
						if(SHTPS_DRAG_SMOOTH_COUNT_MAX > SHTPS_DRAG_HISTORY_SIZE_MAX){
							SHTPS_DRAG_SMOOTH_COUNT_MAX = SHTPS_DRAG_HISTORY_SIZE_MAX;
						}
					#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */
					ts->drag_hist_p->drag_hist[finger][xy].history[ts->drag_hist_p->drag_hist[finger][xy].count] = pos;
				}
			#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
			ts->drag_hist_p->drag_hist[finger][xy].dir   = dir;
			ts->drag_hist_p->drag_hist[finger][xy].count = 1;
		}else{

			#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
				if(SHTPS_DRAG_SMOOTH){
					drag_smooth_count_limit = ts->drag_hist_p->drag_hist[finger][xy].count;
					drag_smooth_count_limit_new = 
						SHTPS_DRAG_SMOOTH_COUNT_MIN + 
						(ts->drag_hist_p->drag_hist[finger][xy].count_up_base / SHTPS_DRAG_SMOOTH_COUNT_UP_STEP);
					if(drag_smooth_count_limit < drag_smooth_count_limit_new){
						drag_smooth_count_limit = drag_smooth_count_limit_new;
					}
					if(drag_smooth_count_limit > SHTPS_DRAG_SMOOTH_COUNT_MAX){
						drag_smooth_count_limit = SHTPS_DRAG_SMOOTH_COUNT_MAX;
					}

					if(ts->drag_hist_p->drag_hist[finger][xy].dir != dir){
						drag_smooth_count_limit = SHTPS_DRAG_SMOOTH_COUNT_MIN;
						if(drag_smooth_count_limit < ts->drag_hist_p->drag_hist[finger][xy].count){
							for(i= 0; i < drag_smooth_count_limit; i++){
								ts->drag_hist_p->drag_hist[finger][xy].history[i] = 
									ts->drag_hist_p->drag_hist[finger][xy].history[ts->drag_hist_p->drag_hist[finger][xy].count - 
									drag_smooth_count_limit + i];
							}
							ts->drag_hist_p->drag_hist[finger][xy].count = drag_smooth_count_limit;
						}

						ts->drag_hist_p->drag_hist[finger][xy].count_up_base = 0;
					}
					
					if(ts->drag_hist_p->drag_hist[finger][xy].count < SHTPS_DRAG_SMOOTH_COUNT_MIN-1){
						ts->drag_hist_p->drag_hist[finger][xy].history[ts->drag_hist_p->drag_hist[finger][xy].count] = pos;
						ts->drag_hist_p->drag_hist[finger][xy].pre_comp_history_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(pos);
						ts->drag_hist_p->drag_hist[finger][xy].count++;
					}else{
						if(ts->drag_hist_p->drag_hist[finger][xy].count == drag_smooth_count_limit-1){
							ts->drag_hist_p->drag_hist[finger][xy].count++;
						}else{
							for(i= 0; i < drag_smooth_count_limit-1; i++){
								ts->drag_hist_p->drag_hist[finger][xy].history[i] = 
									ts->drag_hist_p->drag_hist[finger][xy].history[i+1];
							}
						}
						ts->drag_hist_p->drag_hist[finger][xy].history[drag_smooth_count_limit-1] = pos;
						ts->drag_hist_p->drag_hist[finger][xy].count_up_base++;
					}
				}
			#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

			#if defined( SHTPS_DRAG_STEP_ENABLE )
				if(ts->drag_hist_p->drag_hist[finger][xy].dir == dir){
					#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
						if(!SHTPS_DRAG_SMOOTH){
							if(ts->drag_hist_p->drag_hist[finger][xy].count < SHTPS_DRAG_DIR_FIX_CNT){
								ts->drag_hist_p->drag_hist[finger][xy].count++;
							}
						}
					#else
					if(ts->drag_hist_p->drag_hist[finger][xy].count < SHTPS_DRAG_DIR_FIX_CNT){
						ts->drag_hist_p->drag_hist[finger][xy].count++;
					}
					#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

					if(ts->drag_hist_p->drag_hist[finger][xy].count >= SHTPS_DRAG_DIR_FIX_CNT &&
							ts->touch_state.dragStep[finger][xy] == SHTPS_DRAG_THRESHOLD_2ND)
					{
						shtps_rec_notify_time(ts, xy, finger);
						if(xy == SHTPS_POSTYPE_X){
							ts->drag_hist_p->center_info.fingers[finger].x = pos;
						}else{
							ts->drag_hist_p->center_info.fingers[finger].y = pos;
						}
						SHTPS_LOG_DBG_PRINT("update center pos(%d, %d) time=%lu\n",
									ts->drag_hist_p->center_info.fingers[finger].x, ts->drag_hist_p->center_info.fingers[finger].y,
									ts->touch_state.drag_timeout[finger][xy]);
					}
				}else{
					#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
						if(!SHTPS_DRAG_SMOOTH){
							ts->drag_hist_p->drag_hist[finger][xy].count = 1;
						}
					#else
						ts->drag_hist_p->drag_hist[finger][xy].count = 1;
					#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
				}
			#else
				#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
					if(!SHTPS_DRAG_SMOOTH){
						ts->drag_hist_p->drag_hist[finger][xy].count = 1;
					}
				#else
					if(ts->drag_hist_p->drag_hist[finger][xy].dir != dir){
						ts->drag_hist_p->drag_hist[finger][xy].count = 1;
					}
				#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
			#endif /* SHTPS_DRAG_STEP_ENABLE */

			ts->drag_hist_p->drag_hist[finger][xy].dir = dir;
		}

		ts->drag_hist_p->drag_hist[finger][xy].pre = pos;
	}
}

/* -------------------------------------------------------------------------- */
static int shtps_apply_dragstep(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 *event)
{
#if defined( SHTPS_DRAG_STEP_ENABLE )
	int		i;
	int		fingerMax = shtps_get_fingermax(ts);
	int 	diff_x;
	int 	diff_y;
	int 	diff_cx;
	int 	diff_cy;
	int		dragStep1stX;
	int		dragStep1stY;
	int		FingerDragStep1stX;
	int		FingerDragStep1stY;
	int		PenDragStep1stX;
	int		PenDragStep1stY;
	int		dragStepCurX;
	int		dragStepCurY;
	int		numOfFingers = 0;
	int		numOfPen = 0;

	if( (SHTPS_DRAG_STEP_FINGER_ENABLE == 0) && (SHTPS_DRAG_STEP_PEN_ENABLE == 0) ){
		return -1;
	}

	for (i = 0; i < fingerMax; i++) {
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				numOfFingers++;
			}else if(info->fingers[i].state == SHTPS_TOUCH_STATE_PEN){
				numOfPen++;
			}
		}
	}

	FingerDragStep1stX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, SHTPS_DRAG_THRESHOLD_1ST, numOfFingers, SHTPS_TOUCH_STATE_FINGER);
	FingerDragStep1stY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, SHTPS_DRAG_THRESHOLD_1ST, numOfFingers, SHTPS_TOUCH_STATE_FINGER);

	PenDragStep1stX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, SHTPS_DRAG_THRESHOLD_1ST, numOfPen, SHTPS_TOUCH_STATE_PEN);
	PenDragStep1stY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, SHTPS_DRAG_THRESHOLD_1ST, numOfPen, SHTPS_TOUCH_STATE_PEN);

	for(i = 0;i < fingerMax;i++){
		_log_msg_sync( LOGMSG_ID__FW_EVENT, "%d|%d|%d|%d|%d|%d|%d|%d|%d", i, info->fingers[i].state,
							info->fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000,
							info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000,
							info->fingers[i].x,
							info->fingers[i].y,
							info->fingers[i].wx,
							info->fingers[i].wy,
							info->fingers[i].z);

		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			dragStep1stX = FingerDragStep1stX;
			dragStep1stY = FingerDragStep1stY;
			dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfFingers, SHTPS_TOUCH_STATE_FINGER);
			dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfFingers, SHTPS_TOUCH_STATE_FINGER);
		}else{
			dragStep1stX = PenDragStep1stX;
			dragStep1stY = PenDragStep1stY;
			dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfPen, SHTPS_TOUCH_STATE_PEN);
			dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfPen, SHTPS_TOUCH_STATE_PEN);
		}

		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){

			diff_x = shtps_get_diff(info->fingers[i].x, ts->report_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
			diff_y = shtps_get_diff(info->fingers[i].y, ts->report_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));
			diff_cx= shtps_get_diff(info->fingers[i].x, ts->drag_hist_p->center_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
			diff_cy= shtps_get_diff(info->fingers[i].y, ts->drag_hist_p->center_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));

			if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				if(diff_cy >= dragStep1stY){
					if(ts->touch_state.dragStep[i][1] != SHTPS_DRAG_THRESHOLD_2ND){
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_X, i);
						if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
							dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X,
												ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfFingers, SHTPS_TOUCH_STATE_FINGER);
						}else{
							dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X,
												ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfPen, SHTPS_TOUCH_STATE_PEN);
						}
					}
				}

				if(diff_x >= dragStepCurX){
					if(diff_cx >= dragStep1stX){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
						if(ts->touch_state.dragStep[i][0] != SHTPS_DRAG_THRESHOLD_2ND){
							shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_Y, i);
							if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
								dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y,
													ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfFingers, SHTPS_TOUCH_STATE_FINGER);
							}else{
								dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y,
													ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfPen, SHTPS_TOUCH_STATE_PEN);
							}
						}
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_X, i);

					}else if(shtps_chk_notify_time(ts, SHTPS_POSTYPE_X, i) == 0 ||
								ts->touch_state.dragStep[i][SHTPS_POSTYPE_X] != SHTPS_DRAG_THRESHOLD_2ND){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);

					}else{
						info->fingers[i].x = ts->report_info.fingers[i].x;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_X, i);
					}
				}else{
					info->fingers[i].x = ts->report_info.fingers[i].x;
				}

				if(diff_y >= dragStepCurY){
					if(diff_cy >= dragStep1stY){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_Y, i);

					}else if(shtps_chk_notify_time(ts, SHTPS_POSTYPE_Y, i) == 0 ||
								ts->touch_state.dragStep[i][1] != SHTPS_DRAG_THRESHOLD_2ND)
					{
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
						ts->touch_state.dragStep[i][1] = SHTPS_DRAG_THRESHOLD_2ND;

					}else{
						info->fingers[i].y = ts->report_info.fingers[i].y;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_Y, i);
					}
				}else{
					info->fingers[i].y = ts->report_info.fingers[i].y;
				}
			}else{
				ts->drag_hist_p->center_info.fingers[i].x = info->fingers[i].x;
				ts->drag_hist_p->center_info.fingers[i].y = info->fingers[i].y;
			}
		}else{
			shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_ZERO, SHTPS_POSTYPE_X, i);
			shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_ZERO, SHTPS_POSTYPE_Y, i);
		}

		if(info->fingers[i].state != ts->report_info.fingers[i].state){
			shtps_set_eventtype(event, SHTPS_EVENT_MTDU);
		}
	}

	return 0;
#else
	return (-1);
#endif /* SHTPS_DRAG_STEP_ENABLE */

}

/* -------------------------------------------------------------------------- */
void shtps_filter_drag(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 *event)
{
	int		i;
	int		fingerMax = shtps_get_fingermax(ts);
	int 	diff_x;
	int 	diff_y;
	int		ret;

	SHTPS_LOG_FUNC_CALL();

	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				shtps_add_drag_hist(ts, SHTPS_POSTYPE_X, i, info->fingers[i].x);
				shtps_add_drag_hist(ts, SHTPS_POSTYPE_Y, i, info->fingers[i].y);
			}
		}else{
			shtps_init_drag_hist(ts, SHTPS_POSTYPE_X, i, info->fingers[i].x);
			shtps_init_drag_hist(ts, SHTPS_POSTYPE_Y, i, info->fingers[i].y);
		}
	}

	#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
		shtps_filter_pos_compensation(ts, info, SHTPS_POSTYPE_X);
		shtps_filter_pos_compensation(ts, info, SHTPS_POSTYPE_Y);
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

	/* apply dragstep and set event type */
	ret = shtps_apply_dragstep(ts, info, event);
	if(ret < 0){
		/* set event type */
		for(i = 0;i < fingerMax;i++){
			_log_msg_sync( LOGMSG_ID__FW_EVENT, "%d|%d|%d|%d|%d|%d|%d|%d|%d", i, info->fingers[i].state,
								info->fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000,
								info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000,
								info->fingers[i].x,
								info->fingers[i].y,
								info->fingers[i].wx,
								info->fingers[i].wy,
								info->fingers[i].z);

			if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
					diff_x = shtps_get_diff(info->fingers[i].x, ts->report_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
					diff_y = shtps_get_diff(info->fingers[i].y, ts->report_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));

					if((diff_x > 0) || (diff_y > 0)){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
					}
				}
			}

			if(info->fingers[i].state != ts->report_info.fingers[i].state){
				shtps_set_eventtype(event, SHTPS_EVENT_MTDU);
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_drag_init(struct shtps_rmi_spi *ts)
{
	ts->drag_hist_p = kzalloc(sizeof(struct shtps_filter_drag_hist_info), GFP_KERNEL);
	if(ts->drag_hist_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(ts->drag_hist_p, 0, sizeof(struct shtps_filter_drag_hist_info));	// no need
	// memset(&ts->center_info, 0, sizeof(ts->center_info));	// no need
	#if defined( SHTPS_DRAG_STEP_ENABLE )
	{
		int i;
		for(i = 0;i < SHTPS_FINGER_MAX;i++){
			ts->touch_state.dragStep[i][0] = SHTPS_DRAG_THRESHOLD_ZERO;
			ts->touch_state.dragStep[i][1] = SHTPS_DRAG_THRESHOLD_ZERO;
			ts->touch_state.dragStepReturnTime[i][0] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
			ts->touch_state.dragStepReturnTime[i][1] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
		}
	}
	#endif /* SHTPS_DRAG_STEP_ENABLE */
}

/* -------------------------------------------------------------------------- */
void shtps_filter_drag_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->drag_hist_p)	kfree(ts->drag_hist_p);
	ts->drag_hist_p = NULL;
}
/* -------------------------------------------------------------------------- */

