/* drivers/sharp/shtps/sy3000/shtps_filter_inverting_ghost_check.c
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
#include <sharp/proximity.h>

#include "shtps_rmi.h"
#include "shtps_param_extern.h"

#include "shtps_log.h"

#include "shtps_filter.h"
/* -------------------------------------------------------------------------- */
#if defined( SHTPS_INVERTING_GHOST_REJECTION_ENABLE )
struct shtps_filter_inverting_check_info{
	u8 debug;
	int change_palmthresh_state;
	int ignore_event_state;
};

int shtps_filter_inverting_ghost_check_face_touchmode(struct shtps_rmi_spi *ts)
{
	int ret = 0;
	if(ts->state_mgr.state == SHTPS_STATE_FACETOUCH){
		ret = 1;
		SHTPS_INVERTING_GHOST_REJECT_PRINT("state : facetouch mode.\n");
	}
	return ret;
}


int shtps_filter_inverting_ghost_proximity_check(struct shtps_rmi_spi *ts)
{
	int ret = 0;
	int data = -1;
	
	if(SHTPS_INVERTING_GHOST_REJECT_PROXIMITY_CHECK_ENABLE == 0) return ret;

	SHTPS_INVERTING_GHOST_REJECT_PRINT("[proximity] check start\n");
	PROX_dataread_func(&data);
	SHTPS_INVERTING_GHOST_REJECT_PRINT("[proximity] check end(data:%d)\n",data);
	
	if(SH_PROXIMITY_NEAR == data){
		ret = 1;
	}
	
	return ret;
}

/* -------------------------------------------------------------------------- */
int shtps_filter_inverting_ghost_face_down_check(struct shtps_rmi_spi *ts)
{
	struct shub_face_acc_info data;
	int ret = 0;
	int ret_sensor = 0;

//	SHTPS_INVERTING_GHOST_REJECT_PRINT("shub_api_get_face_down_info st \n");
	ret = shub_api_get_face_down_info(&data);
//	SHTPS_INVERTING_GHOST_REJECT_PRINT("shub_api_get_face_down_info ed ret[%d]\n", ret);
	SHTPS_INVERTING_GHOST_REJECT_PRINT("sensor info :  nJudge[%d] nStat[%d] nX[%d] nY[%d] nZ[%d]\n", data.nJudge, data.nStat, data.nX, data.nY, data.nZ);
	if(ret != 0){
		SHTPS_INVERTING_GHOST_REJECT_PRINT("shub_api_get_face_down_info ret error[%d]\n", ret);
	}
	
	if(data.nJudge == 1){
		ret_sensor = 1;
	}
	
	return ret_sensor;
}

void shtps_filter_inverting_ghost_change_palm_thresh(struct shtps_rmi_spi *ts)
{
	shtps_fwctl_set_palm_amplitude_threshold(ts, SHTPS_INVERTING_GHOST_REJECT_PALM_AMPLITUDE_THRESH);
	shtps_fwctl_set_palm_area(ts, SHTPS_INVERTING_GHOST_REJECT_PALM_AREA_THRESH);
	
	shtps_fwctl_set_palm_filter_mode_enable(ts);
	
	return;
}

void shtps_filter_inverting_ghost_set_palm_thresh_default(struct shtps_rmi_spi *ts)
{
	u8 def_palm_amp = 0;
	u8 def_palm_area = 0;
	
	shtps_fwctl_get_palm_amplitude_threshold(ts, &def_palm_amp);
	shtps_fwctl_get_palm_area(ts, &def_palm_area);
	shtps_fwctl_set_palm_amplitude_threshold(ts, def_palm_amp);
	shtps_fwctl_set_palm_area(ts, def_palm_area);
	
	shtps_fwctl_set_palm_filter_mode_disable(ts);
	SHTPS_INVERTING_GHOST_REJECT_PRINT("set palm thresh default\n");
	ts->inverting_check_p->change_palmthresh_state = 0;

	return;
}

void shtps_filter_inverting_ghost_event_all_touchup(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	
	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				shtps_report_touch_on(ts, i,
									  SHTPS_TOUCH_CANCEL_COORDINATES_X,
									  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
									  1,
									  ts->report_info.fingers[i].wx,
									  ts->report_info.fingers[i].wy,
									  ts->report_info.fingers[i].z);
				input_sync(ts->input);
			}
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
		}
	}
	return;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_inverting_ghost_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int td_finger_num = 0;
	int fingerMax = shtps_get_fingermax(ts);
	int sensor_state = 0;
	int all_finger_num = 0;
	
	if(SHTPS_INVERTING_GHOST_REJECT_ENABLE == 0) return;
	
	if(shtps_filter_inverting_ghost_check_face_touchmode(ts) == 1) return;
	
	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				td_finger_num++;
			}
			all_finger_num++;
		}
	}
	
	if(all_finger_num > 0){
		
		sensor_state = shtps_filter_inverting_ghost_face_down_check(ts);
		
		if(sensor_state == 1){
			if(ts->inverting_check_p->change_palmthresh_state == 0){
				shtps_filter_inverting_ghost_change_palm_thresh(ts);
				SHTPS_INVERTING_GHOST_REJECT_PRINT("change palm thresh\n");
				ts->inverting_check_p->change_palmthresh_state = 1;
			}
			
			if(ts->inverting_check_p->ignore_event_state == 1){
				for(i = 0; i < fingerMax; i++){
					if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
						info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
					}
				}
			}
			
			if(ts->inverting_check_p->ignore_event_state == 0){
				if(shtps_filter_inverting_ghost_proximity_check(ts) == 1){
					ts->inverting_check_p->ignore_event_state = 1;
					shtps_filter_inverting_ghost_event_all_touchup(ts, info);
					SHTPS_INVERTING_GHOST_REJECT_PRINT("Proximity near. ignore event enable.\n");
				}
			}
			
			
		}else{
			//sensor_state = 0
			if(ts->inverting_check_p->change_palmthresh_state == 1){
				shtps_filter_inverting_ghost_set_palm_thresh_default(ts);
			}
			if(ts->inverting_check_p->ignore_event_state == 1){
				ts->inverting_check_p->ignore_event_state = 0;
				SHTPS_INVERTING_GHOST_REJECT_PRINT("not face down. ignore event diseable.\n");
			}
		}
	}else{
		if(ts->inverting_check_p->ignore_event_state == 1){
			ts->inverting_check_p->ignore_event_state = 0;
			SHTPS_INVERTING_GHOST_REJECT_PRINT("all touch up. ignore event diseable.\n");
		}
	}
	
	return;
}
/* -------------------------------------------------------------------------- */
void shtps_filter_inverting_ghost_init(struct shtps_rmi_spi *ts)
{
	ts->inverting_check_p = kzalloc(sizeof(struct shtps_filter_inverting_check_info), GFP_KERNEL);
	if(ts->inverting_check_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_inverting_ghost_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->inverting_check_p)	kfree(ts->inverting_check_p);
	ts->inverting_check_p = NULL;
}
/* -------------------------------------------------------------------------- */
#endif /* SHTPS_INVERTING_GHOST_REJECTION_ENABLE */
