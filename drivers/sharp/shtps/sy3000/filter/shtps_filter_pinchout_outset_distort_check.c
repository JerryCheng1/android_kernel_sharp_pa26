/* drivers/sharp/shtps/sy3000/shtps_filter_pinchout_outset_distort_check.c
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
#if defined( SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE )
void shtps_filter_pinchout_outset_distort_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int numOfFingers = 0;
	int numOfFingers_old = 0;
	int diff_x, diff_y, diff_z;
	u8 id_1st = 0;
	u8 id_2nd = 0;

	if(SHTPS_PINCHOUT_OUTSET_DISTORT_REJECT_ENABLE == 0){
		return;
	}

	for(i = 0;i < fingerMax;i++){
		if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			numOfFingers_old++;
			id_1st = i;
		}
		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			numOfFingers++;
			if(id_1st != i){
				id_2nd = i;
			}
		}
	}

	if( (numOfFingers_old == 1) && (numOfFingers == 2) ){
		diff_x = shtps_get_diff(info->fingers[id_1st].x, info->fingers[id_2nd].x, SHTPS_POS_SCALE_X(ts));
		diff_y = shtps_get_diff(info->fingers[id_1st].y, info->fingers[id_2nd].y, SHTPS_POS_SCALE_Y(ts));
		diff_z = ts->report_info.fingers[id_1st].z - info->fingers[id_1st].z;

		SHTPS_LOG_DBG_PINCHOUT_DISTORT_PRINT("[id1:%d][id2:%d][diff_x=%d][diff_y=%d][diff_z=%d][distance = %d]\n",
												id_1st, id_2nd, diff_x, diff_y, diff_z, ((diff_x * diff_x) + (diff_y * diff_y)));

		if( ((diff_x * diff_x) + (diff_y * diff_y)) < (SHTPS_PINCHOUT_OUTSET_DISTORT_REJECT_DISTANCE_THRESH * SHTPS_PINCHOUT_OUTSET_DISTORT_REJECT_DISTANCE_THRESH) ){
			if(diff_z >= SHTPS_PINCHOUT_OUTSET_DISTORT_REJECT_Z_REDUCE_THRESH){
				SHTPS_LOG_DBG_PINCHOUT_DISTORT_PRINT("report touch cancel\n");

				shtps_report_touch_on(ts, id_1st,
									  SHTPS_TOUCH_CANCEL_COORDINATES_X,
									  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
									  shtps_get_fingerwidth(ts, id_1st, &ts->report_info),
									  ts->report_info.fingers[id_1st].wx,
									  ts->report_info.fingers[id_1st].wy,
									  ts->report_info.fingers[id_1st].z);
				input_sync(ts->input);

				shtps_report_touch_off(ts, id_1st,
									  SHTPS_TOUCH_CANCEL_COORDINATES_X,
									  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
									  shtps_get_fingerwidth(ts, id_1st, &ts->report_info),
									  ts->report_info.fingers[id_1st].wx,
									  ts->report_info.fingers[id_1st].wy,
									  ts->report_info.fingers[id_1st].z);
				input_sync(ts->input);

				ts->report_info.fingers[id_1st].state = SHTPS_TOUCH_STATE_NO_TOUCH;
				ts->report_info.fingers[id_1st].x  = SHTPS_TOUCH_CANCEL_COORDINATES_X;
				ts->report_info.fingers[id_1st].y  = SHTPS_TOUCH_CANCEL_COORDINATES_Y;
			}
		}
	}
}
/* -------------------------------------------------------------------------- */
void shtps_filter_pinchout_outset_distort_init(struct shtps_rmi_spi *ts)
{
	
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pinchout_outset_distort_deinit(struct shtps_rmi_spi *ts)
{
	
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE */
