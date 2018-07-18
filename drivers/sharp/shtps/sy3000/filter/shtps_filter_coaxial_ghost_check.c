/* drivers/sharp/shtps/sy3000/shtps_filter_water_cling_reject_check.c
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
#if defined( SHTPS_COAXIAL_GHOST_REJECTION_ENABLE )
struct shtps_filter_coaxial_ghost_reject_info{
	int							reject_num[SHTPS_FINGER_MAX];
	u8							pending_cnt;
	u8							pending_ghost[2][SHTPS_FINGER_MAX];
	struct shtps_touch_info		pending_info;
};

/* -------------------------------------------------------------------------- */
static void shtps_filter_coaxial_push_pending_ghost(struct shtps_rmi_spi *ts, u8 finger, u8 ghost)
{
	int i;
	int idx = 0;

	if(ts->coaxial_ghost_reject_p->pending_cnt > 0){
		for(i = 0;i < ts->coaxial_ghost_reject_p->pending_cnt;i++){
			if(ts->coaxial_ghost_reject_p->pending_ghost[i][0] == finger){
				idx = i;
				break;
			}
		}
		if(i == ts->coaxial_ghost_reject_p->pending_cnt){
			if(ts->coaxial_ghost_reject_p->pending_cnt >= 2){
				return;
			}else{
				idx = ts->coaxial_ghost_reject_p->pending_cnt;
				memset(ts->coaxial_ghost_reject_p->pending_ghost[idx], 0xFF, sizeof(ts->coaxial_ghost_reject_p->pending_ghost[idx]));
				ts->coaxial_ghost_reject_p->pending_cnt++;
			}
		}
		SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] pending count = %d\n", ts->coaxial_ghost_reject_p->pending_cnt);
	}else{
		idx = ts->coaxial_ghost_reject_p->pending_cnt;
		memset(ts->coaxial_ghost_reject_p->pending_ghost[idx], 0xFF, sizeof(ts->coaxial_ghost_reject_p->pending_ghost[idx]));
		ts->coaxial_ghost_reject_p->pending_cnt++;
		SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] pending count = %d\n", ts->coaxial_ghost_reject_p->pending_cnt);
	}

	ts->coaxial_ghost_reject_p->pending_ghost[idx][0] = finger;
	for(i = 1;i < shtps_get_fingermax(ts);i++){
		if(ts->coaxial_ghost_reject_p->pending_ghost[idx][i] == 0xFF){
			ts->coaxial_ghost_reject_p->pending_ghost[idx][i] = ghost;
			SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] Add pending id [%d] %d, %d, %d, %d, %d\n",
									idx,
									ts->coaxial_ghost_reject_p->pending_ghost[idx][0],
									ts->coaxial_ghost_reject_p->pending_ghost[idx][1],
									ts->coaxial_ghost_reject_p->pending_ghost[idx][2],
									ts->coaxial_ghost_reject_p->pending_ghost[idx][3],
									ts->coaxial_ghost_reject_p->pending_ghost[idx][4]);
			break;
		}
	}
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_coaxial_proc_pending_ghost(struct shtps_rmi_spi *ts)
{
	int i, j;
	u8 finger;
	u8 ghost;

	if(ts->coaxial_ghost_reject_p->pending_cnt == 0){
		return;
	}

	SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] pending count = %d\n", ts->coaxial_ghost_reject_p->pending_cnt);

	for(i = 0;i < ts->coaxial_ghost_reject_p->pending_cnt;i++){
		finger = ts->coaxial_ghost_reject_p->pending_ghost[i][0];
		for(j = 1;j < shtps_get_fingermax(ts);j++){
			ghost = ts->coaxial_ghost_reject_p->pending_ghost[i][j];

			SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] finger[%d] state = %d, z = %d / ghost[%d] state = %d, pre z = %d\n",
				finger, ts->fw_report_info.fingers[finger].state, ts->fw_report_info.fingers[finger].z,
				ghost, ts->fw_report_info.fingers[ghost].state, ts->fw_report_info_store.fingers[ghost].z);

			if(ghost != 0xFF){
				if(ts->fw_report_info.fingers[finger].state != SHTPS_TOUCH_STATE_NO_TOUCH &&
					ts->fw_report_info.fingers[finger].z != 0 &&
					ts->fw_report_info_store.fingers[ghost].z != 0)
				{
					int z_ratio = (ts->fw_report_info_store.fingers[ghost].z * 100) / ts->fw_report_info.fingers[finger].z;

					SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] z ratio = %d\n", z_ratio);
					
					if(ts->coaxial_ghost_reject_p->pending_info.fingers[ghost].state != SHTPS_TOUCH_STATE_NO_TOUCH &&
							(z_ratio > SHTPS_COAXIAL_GHOST_REJECT_Z_THRESH_TD || ts->fw_report_info.fingers[ghost].state != SHTPS_TOUCH_STATE_NO_TOUCH))
					{
						shtps_report_touch_on(ts, ghost,
											  ts->coaxial_ghost_reject_p->pending_info.fingers[ghost].x,
											  ts->coaxial_ghost_reject_p->pending_info.fingers[ghost].y,
											  shtps_get_fingerwidth(ts, ghost, &ts->coaxial_ghost_reject_p->pending_info),
											  ts->coaxial_ghost_reject_p->pending_info.fingers[ghost].wx,
											  ts->coaxial_ghost_reject_p->pending_info.fingers[ghost].wy,
											  ts->coaxial_ghost_reject_p->pending_info.fingers[ghost].z);
						input_sync(ts->input);
						memcpy(&ts->report_info.fingers[ghost], 
									&ts->coaxial_ghost_reject_p->pending_info.fingers[ghost], sizeof(ts->report_info.fingers[ghost]));

						SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] Notify pending event [%d] x=%d, y=%d, wx=%d, wy=%d, z=%d\n", 
											ghost,
											ts->coaxial_ghost_reject_p->pending_info.fingers[ghost].x,
											ts->coaxial_ghost_reject_p->pending_info.fingers[ghost].y,
											ts->coaxial_ghost_reject_p->pending_info.fingers[ghost].wx,
											ts->coaxial_ghost_reject_p->pending_info.fingers[ghost].wy,
						                    ts->coaxial_ghost_reject_p->pending_info.fingers[ghost].z);
					}
				}
			}else{
				break;
			}
		}
	}
	ts->coaxial_ghost_reject_p->pending_cnt = 0;
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_coaxial_hold_pending_ghost(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i, j;
	
	if(ts->coaxial_ghost_reject_p->pending_cnt == 0){
		return;
	}

	memcpy(&ts->coaxial_ghost_reject_p->pending_info, info, sizeof(ts->coaxial_ghost_reject_p->pending_info));

	for(i = 0;i < ts->coaxial_ghost_reject_p->pending_cnt;i++){
		for(j = 1;j < shtps_get_fingermax(ts);j++){
			if(ts->coaxial_ghost_reject_p->pending_ghost[i][j] != 0xFF){
				u8 ghost  = ts->coaxial_ghost_reject_p->pending_ghost[i][j];
				info->fingers[ghost].state = SHTPS_TOUCH_STATE_NO_TOUCH;
				SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] reject ghost. [%d] x=%d, y=%d, wx=%d, wy=%d, z=%d "
											"(finger[%d] x=%d, y=%d, wx=%d, wy=%d, z=%d)\n", 
											ghost, 
											ts->fw_report_info.fingers[ghost].x, 
											ts->fw_report_info.fingers[ghost].y,
											ts->fw_report_info.fingers[ghost].wx, 
											ts->fw_report_info.fingers[ghost].wy,
											ts->fw_report_info.fingers[ghost].z,
											ts->coaxial_ghost_reject_p->pending_ghost[i][0], 
											ts->fw_report_info.fingers[ts->coaxial_ghost_reject_p->pending_ghost[i][0]].x, 
											ts->fw_report_info.fingers[ts->coaxial_ghost_reject_p->pending_ghost[i][0]].y,
											ts->fw_report_info.fingers[ts->coaxial_ghost_reject_p->pending_ghost[i][0]].wx, 
											ts->fw_report_info.fingers[ts->coaxial_ghost_reject_p->pending_ghost[i][0]].wy,
											ts->fw_report_info.fingers[ts->coaxial_ghost_reject_p->pending_ghost[i][0]].z);
			}else{
				break;
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
static int shtps_filter_coaxial_is_ghost(struct shtps_rmi_spi *ts, int id1, int id2, int y_th, int y_th2, int z_th, int z_th2)
{
	unsigned long y_diff;
	int finger;
	int ghost;
	
	if(ts->fw_report_info.fingers[id1].z < ts->fw_report_info.fingers[id2].z){
		ghost  = id1;
		finger = id2;
	}else{
		ghost  = id2;
		finger = id1;
	}

	#if defined(SHTPS_PEN_DETECT_ENABLE)
		if(SHTPS_COAXIAL_GHOST_REJECT_PEN_ONLY){
			if(ts->fw_report_info.fingers[ghost].state != SHTPS_TOUCH_STATE_PEN){
				return -1;
			}
		}
	#endif /* SHTPS_PEN_DETECT_ENABLE */

	SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] y[%d]=%d, y[%d]=%d, diff_y=%lu\n",
			id1, ts->fw_report_info.fingers[id1].y, id2, ts->fw_report_info.fingers[id2].y, 
			abs(ts->fw_report_info.fingers[id1].y - ts->fw_report_info.fingers[id2].y));
	
	y_diff = abs(ts->fw_report_info.fingers[finger].y - ts->fw_report_info.fingers[ghost].y);
	if(y_diff <= y_th &&
		ts->fw_report_info.fingers[finger].z != 0 && ts->fw_report_info.fingers[ghost].z != 0)
	{
		int z_ratio = (ts->fw_report_info.fingers[ghost].z * 100) / ts->fw_report_info.fingers[finger].z;

		SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] z[%d]=%d, z[%d]=%d, z_ratio=%d\n",
				finger, ts->fw_report_info.fingers[finger].z, ghost, ts->fw_report_info.fingers[ghost].z, z_ratio);
		
		if((z_ratio <= z_th) || (y_diff <= y_th2 && z_ratio <= z_th2)){
			return ghost;
		}else{
			if(ts->fw_report_info_store.fingers[finger].state == SHTPS_TOUCH_STATE_NO_TOUCH &&
				ts->fw_report_info_store.fingers[ghost].state == SHTPS_TOUCH_STATE_NO_TOUCH)
			{
				shtps_filter_coaxial_push_pending_ghost(ts, finger, ghost);
			}
		}
	}
	return -1;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_coaxial_ghost_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i, j;
	int fingerMax = shtps_get_fingermax(ts);
	u8 is_reject[SHTPS_FINGER_MAX];

	if(!SHTPS_COAXIAL_GHOST_REJECT_ENABLE){
		return;
	}

	shtps_filter_coaxial_proc_pending_ghost(ts);
	memset(is_reject, 0, sizeof(is_reject));

	for(i = 0; i < fingerMax - 1; i++) {
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			for(j = i + 1; j < fingerMax; j++){
				if(info->fingers[j].state != SHTPS_TOUCH_STATE_NO_TOUCH){
					int ghost = -1;
					
					if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH &&
						ts->fw_report_info_store.fingers[j].state == SHTPS_TOUCH_STATE_NO_TOUCH)
					{
						if(ts->coaxial_ghost_reject_p->reject_num[i] == 0 &&
							ts->coaxial_ghost_reject_p->reject_num[j] == 0)
						{
							SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] Check TD ghost\n");
							ghost = shtps_filter_coaxial_is_ghost(ts, i, j, 
														   SHTPS_COAXIAL_GHOST_REJECT_YDIFF_THRESH_TD,
														   SHTPS_COAXIAL_GHOST_REJECT_YDIFF_THRESH_TD_ST, 
														   SHTPS_COAXIAL_GHOST_REJECT_Z_THRESH_TD, 
														   SHTPS_COAXIAL_GHOST_REJECT_Z_THRESH_TD_ST);
						}
					}else{
						if(ts->coaxial_ghost_reject_p->reject_num[i] < SHTPS_COAXIAL_GHOST_REJECT_DRAG_MAX_NUM ||
							ts->coaxial_ghost_reject_p->reject_num[j] < SHTPS_COAXIAL_GHOST_REJECT_DRAG_MAX_NUM)
						{
							SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] Check MV ghost\n");
							ghost = shtps_filter_coaxial_is_ghost(ts, i, j, 
														   SHTPS_COAXIAL_GHOST_REJECT_YDIFF_THRESH_MV,
														   SHTPS_COAXIAL_GHOST_REJECT_YDIFF_THRESH_MV_ST, 
														   SHTPS_COAXIAL_GHOST_REJECT_Z_THRESH_MV, 
														   SHTPS_COAXIAL_GHOST_REJECT_Z_THRESH_MV_ST);
							
							if(ghost != -1){
								int finger = (ghost == i)? j : i;
								unsigned long diff_y = abs(ts->fw_report_info.fingers[finger].y - ts->fw_report_info_store.fingers[finger].y);
								if(diff_y < SHTPS_COAXIAL_GHOST_REJECT_DRAG_YDIFF_COND){
									ghost = 0xFF;
								}
							}
						}
					}
					if(ghost >= 0 && ghost < fingerMax && ts->coaxial_ghost_reject_p->reject_num[ghost] < SHTPS_COAXIAL_GHOST_REJECT_DRAG_MAX_NUM){
						is_reject[ghost] = 1;
						info->fingers[ghost].state = SHTPS_TOUCH_STATE_NO_TOUCH;
						SHTPS_LOG_DBG_PRINT("[COAXIAL_GHOST] reject ghost. [%d] x=%d, y=%d, wx=%d, wy=%d, z=%d "
											"(finger[%d] x=%d, y=%d, wx=%d, wy=%d, z=%d)\n", 
											ghost, 
											ts->fw_report_info.fingers[ghost].x, 
											ts->fw_report_info.fingers[ghost].y,
											ts->fw_report_info.fingers[ghost].wx, 
											ts->fw_report_info.fingers[ghost].wy,
											ts->fw_report_info.fingers[ghost].z,
											(ghost == i)? j : i, 
											ts->fw_report_info.fingers[(ghost == i)? j : i].x, 
											ts->fw_report_info.fingers[(ghost == i)? j : i].y,
											ts->fw_report_info.fingers[(ghost == i)? j : i].wx, 
											ts->fw_report_info.fingers[(ghost == i)? j : i].wy,
											ts->fw_report_info.fingers[(ghost == i)? j : i].z);
					
						if(ghost == i){
							break;
						}
					}
				}
			}
		}
	}
	
	for(i = 0;i < fingerMax;i++){
		if(is_reject[i]){
			ts->coaxial_ghost_reject_p->reject_num[i]++;
		}else{
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				ts->coaxial_ghost_reject_p->reject_num[i] = 0;
			}else{
				ts->coaxial_ghost_reject_p->reject_num[i] = SHTPS_COAXIAL_GHOST_REJECT_DRAG_MAX_NUM;
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_coaxial_ghost_event_report(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 event)
{
	shtps_filter_coaxial_hold_pending_ghost(ts, info);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_coaxial_ghost_forcetouchup(struct shtps_rmi_spi *ts)
{
	memset(ts->coaxial_ghost_reject_p->reject_num, 0, sizeof(ts->coaxial_ghost_reject_p->reject_num));
	ts->coaxial_ghost_reject_p->pending_cnt= 0;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_coaxial_ghost_init(struct shtps_rmi_spi *ts)
{
	ts->coaxial_ghost_reject_p = kzalloc(sizeof(struct shtps_filter_coaxial_ghost_reject_info), GFP_KERNEL);
	if(ts->coaxial_ghost_reject_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(ts->coaxial_ghost_reject_p->reject_num, 0, sizeof(ts->coaxial_ghost_reject_p->reject_num));	// no need
	// ts->coaxial_ghost_reject_p->pending_cnt= 0;	// no need
}

/* -------------------------------------------------------------------------- */
void shtps_filter_coaxial_ghost_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->coaxial_ghost_reject_p)	kfree(ts->coaxial_ghost_reject_p);
	ts->coaxial_ghost_reject_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_COAXIAL_GHOST_REJECTION_ENABLE */
