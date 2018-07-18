/* drivers/sharp/shtps/sy3000/shtps_filter_detect_ghost.c
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
#if defined(SHTPS_GHOST_REJECTION_ENABLE)
struct shtps_filter_detect_ghost_info{
	int		is_ghost[SHTPS_FINGER_MAX];
};

int shtps_filter_detect_ghost(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	unsigned short finger_distance = 0;
	int diff_x;
	int diff_y;
	int numOfFingers = 0;
	u8 ghost = 0;
	u8 pen_td = 0;

	if(SHTPS_HOST_GHOST_REJECTION_ENABLE == 0){
		return 0;
	}

	for(i = 0; i < fingerMax; i++){
		if( (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) &&
			(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) )
		{
			diff_x = shtps_get_diff(ts->fw_report_info.fingers[i].x, ts->fw_report_info_store.fingers[i].x, SHTPS_POS_SCALE_X(ts));
			if(finger_distance < diff_x){
				finger_distance = diff_x;
			}

			diff_y = shtps_get_diff(ts->fw_report_info.fingers[i].y, ts->fw_report_info_store.fingers[i].y, SHTPS_POS_SCALE_Y(ts));
			if(finger_distance < diff_y){
				finger_distance = diff_y;
			}
		}

		if( (info->fingers[i].state == SHTPS_TOUCH_STATE_PEN) &&
			(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) )
		{
			pen_td |= (1 << i);
		}

		if( (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) ||
			(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN) )
		{
			numOfFingers++;
		}
	}

	for(i = 0; i < fingerMax; i++){
		if(ts->detect_ghost_p->is_ghost[i] == 0){
			if(((pen_td >> i) & 0x01) != 0){
				if(finger_distance > SHTPS_GHOST_THRESHOLD){
					if(numOfFingers >= SHTPS_GHOST_CHECK_TOUCH_NUM_THRESH){
						ts->detect_ghost_p->is_ghost[i] = 1;
						ghost |= (1 << i);
					}
				}
			}
		}
		else{
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				ts->detect_ghost_p->is_ghost[i] = 0;
			}
			else{
				ghost |= (1 << i);
			}
		}
	}

	return ghost;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_detect_ghost_forcetouchup(struct shtps_rmi_spi *ts)
{
	int i, fingerMax = shtps_get_fingermax(ts);

	for(i = 0;i < fingerMax;i++){
		ts->detect_ghost_p->is_ghost[i] = 0;
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_detect_ghost_init(struct shtps_rmi_spi *ts)
{
	ts->detect_ghost_p = kzalloc(sizeof(struct shtps_filter_detect_ghost_info), GFP_KERNEL);
	if(ts->detect_ghost_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(ts->detect_ghost_p->is_ghost, 0, sizeof(ts->detect_ghost_p->is_ghost));	// no need
}
/* -------------------------------------------------------------------------- */
void shtps_filter_detect_ghost_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->detect_ghost_p)	kfree(ts->detect_ghost_p);
	ts->detect_ghost_p = NULL;
}
/* -------------------------------------------------------------------------- */
#endif	/* SHTPS_GHOST_REJECTION_ENABLE */
