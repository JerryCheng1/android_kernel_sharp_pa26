/* drivers/sharp/shtps/sy3000/shtps_filter_pinch_fail_response_check.c
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
#include "shtps_fwctl.h"

#include "shtps_filter.h"
/* -------------------------------------------------------------------------- */
#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
struct shtps_filter_pinch_fail_reject_info{
	u8				segmentation_aggressiveness_def[3];
	u8				segmentation_aggressiveness_set_state;
	u8				segmentation_aggressiveness_set_check;
	u32				finger_distance;
};

/* -------------------------------------------------------------------------- */
void shtps_filter_pinch_fail_response_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int numOfFingers = 0;
	u16 finger_x[SHTPS_FINGER_MAX], finger_y[SHTPS_FINGER_MAX];
	int diff_x, diff_y;

	if(SHTPS_PINCH_FAIL_RESPONSE_REJECT_ENABLE == 0){
		return;
	}

	for(i = 0;i < fingerMax;i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			finger_x[numOfFingers] = ts->fw_report_info.fingers[i].x;
			finger_y[numOfFingers] = ts->fw_report_info.fingers[i].y;
			numOfFingers++;
		}
	}

	if(numOfFingers == 2){
		diff_x = shtps_get_diff(finger_x[0], finger_x[1], SHTPS_POS_SCALE_X(ts));
		diff_y = shtps_get_diff(finger_y[0], finger_y[1], SHTPS_POS_SCALE_Y(ts));
		ts->pinch_fail_reject_p->finger_distance = ((diff_x * diff_x) + (diff_y * diff_y));
	}else{
		ts->pinch_fail_reject_p->finger_distance = 0;
	}

	SHTPS_LOG_DBG_SHTPS_PINCH_FAIL_RESPONSE_PRINT("finger distance : %d\n", ts->pinch_fail_reject_p->finger_distance);

	if(ts->pinch_fail_reject_p->segmentation_aggressiveness_set_state != 0){
		if( (numOfFingers != 2) ||
			(ts->pinch_fail_reject_p->finger_distance > (SHTPS_PINCH_FAIL_FINGER_2ND_DISTANCE_THRESH * SHTPS_PINCH_FAIL_FINGER_2ND_DISTANCE_THRESH)) )
		{
			shtps_fwctl_reg_write_segmentation_aggressiveness(ts, ts->pinch_fail_reject_p->segmentation_aggressiveness_def);
			ts->pinch_fail_reject_p->segmentation_aggressiveness_set_state = 0;
			SHTPS_LOG_DBG_SHTPS_PINCH_FAIL_RESPONSE_PRINT("segmentation aggressiveness set def [%d]\n", ts->pinch_fail_reject_p->segmentation_aggressiveness_def[2]);
		}
	}

	if(ts->pinch_fail_reject_p->segmentation_aggressiveness_set_state == 0){
		if(ts->pinch_fail_reject_p->segmentation_aggressiveness_set_check != 0){
			if(numOfFingers != 2){
				ts->pinch_fail_reject_p->segmentation_aggressiveness_set_check = 0;
				SHTPS_LOG_DBG_SHTPS_PINCH_FAIL_RESPONSE_PRINT("check end by finger num [%d]", numOfFingers);
			}else{
				if(ts->pinch_fail_reject_p->finger_distance <= (SHTPS_PINCH_FAIL_FINGER_2ND_DISTANCE_THRESH * SHTPS_PINCH_FAIL_FINGER_2ND_DISTANCE_THRESH)){
					u8 buf[3];
					memcpy(buf, ts->pinch_fail_reject_p->segmentation_aggressiveness_def, sizeof(buf));
					buf[2] = SHTPS_SEGMENTATION_AGGRESSIVENESS_SET_VAL;
					shtps_fwctl_reg_write_segmentation_aggressiveness(ts, buf);
					ts->pinch_fail_reject_p->segmentation_aggressiveness_set_state = 1;
					SHTPS_LOG_DBG_SHTPS_PINCH_FAIL_RESPONSE_PRINT("segmentation aggressiveness set [%d]\n", SHTPS_SEGMENTATION_AGGRESSIVENESS_SET_VAL);
				}
			}
		}

		if(ts->pinch_fail_reject_p->segmentation_aggressiveness_set_check == 0){
			if(ts->pinch_fail_reject_p->finger_distance > (SHTPS_PINCH_FAIL_FINGER_1ST_DISTANCE_THRESH * SHTPS_PINCH_FAIL_FINGER_1ST_DISTANCE_THRESH)){
				ts->pinch_fail_reject_p->segmentation_aggressiveness_set_check = 1;
				SHTPS_LOG_DBG_SHTPS_PINCH_FAIL_RESPONSE_PRINT("check start\n");
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_get_register_segmentation_aggressiveness(struct shtps_rmi_spi *ts)
{
	shtps_fwctl_get_segmentation_aggressiveness(ts, ts->pinch_fail_reject_p->segmentation_aggressiveness_def);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pinch_fail_response_sleep(struct shtps_rmi_spi *ts)
{
	if(ts->pinch_fail_reject_p->segmentation_aggressiveness_set_state != 0){
		shtps_fwctl_reg_write_segmentation_aggressiveness(ts, ts->pinch_fail_reject_p->segmentation_aggressiveness_def);
		SHTPS_LOG_DBG_SHTPS_PINCH_FAIL_RESPONSE_PRINT("segmentation aggressiveness set def [%d]\n", ts->pinch_fail_reject_p->segmentation_aggressiveness_def[2]);
	}
	ts->pinch_fail_reject_p->segmentation_aggressiveness_set_state = 0;
	ts->pinch_fail_reject_p->segmentation_aggressiveness_set_check = 0;
	ts->pinch_fail_reject_p->finger_distance = 0;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pinch_fail_response_init(struct shtps_rmi_spi *ts)
{
	ts->pinch_fail_reject_p = kzalloc(sizeof(struct shtps_filter_pinch_fail_reject_info), GFP_KERNEL);
	if(ts->pinch_fail_reject_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(&(ts->pinch_fail_reject), 0, sizeof(struct shtps_pinch_fail_reject));	// no need
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pinch_fail_response_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->pinch_fail_reject_p)	kfree(ts->pinch_fail_reject_p);
	ts->pinch_fail_reject_p = NULL;
}
/* -------------------------------------------------------------------------- */
#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */
