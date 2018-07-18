/* drivers/sharp/shtps/sy3000/shtps_filter_lgm_split_touch_combining_check_s3400.c
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
#if defined( SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE )
struct shtps_filter_lgm_split_touch_combining_info{
	u8					finger_swap;
	u8					finger_adjust;
	//
	u8					fingerBase;
	u8					finger1st;
	u8					pos_hist_cnt;
	unsigned long		pos_hist_t[3];
	unsigned short		pos_hist_x[3];
	unsigned short		pos_hist_y[3];
};

/* -------------------------------------------------------------------------- */
static void shtps_filter_swap_finger(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, int a, int b)
{
	struct shtps_touch_info info_bak;

	SHTPS_LOG_FUNC_CALL();
	memcpy(info_bak.fingers, info->fingers, sizeof(info_bak.fingers));

	info->fingers[a].state = info->fingers[b].state;
	info->fingers[a].wx    = info->fingers[b].wx;
	info->fingers[a].wy    = info->fingers[b].wy;
	info->fingers[a].z     = info->fingers[b].z;

	info->fingers[b].state = info_bak.fingers[a].state;
	info->fingers[b].wx    = info_bak.fingers[a].wx;
	info->fingers[b].wy    = info_bak.fingers[a].wy;
	info->fingers[b].z     = info_bak.fingers[a].z;

	if(ts->lgm_split_touch_combining_p->finger_adjust != 0)
	{
		int diff_x_1, diff_y_1;
		int diff_x_2, diff_y_2;

		diff_x_1 = ts->report_info.fingers[a].x - info->fingers[b].x;
		diff_y_1 = ts->report_info.fingers[a].y - info->fingers[b].y;
		diff_x_2 = ts->report_info.fingers[b].x - info->fingers[a].x;
		diff_y_2 = ts->report_info.fingers[b].y - info->fingers[a].y;

		info->fingers[a].x = ts->report_info.fingers[a].x - (diff_x_1 / 2);
		info->fingers[a].y = ts->report_info.fingers[a].y - (diff_y_1 / 2);
		info->fingers[b].x = ts->report_info.fingers[b].x - (diff_x_2 / 2);
		info->fingers[b].y = ts->report_info.fingers[b].y - (diff_y_2 / 2);

		ts->lgm_split_touch_combining_p->finger_adjust = 0;
	}
	else{
		info->fingers[a].x = info->fingers[b].x;
		info->fingers[a].y = info->fingers[b].y;

		info->fingers[b].x = info_bak.fingers[a].x;
		info->fingers[b].y = info_bak.fingers[a].y;
	}

	SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("[swap] [%d](%d, %d), state=%d\n", a,
		info->fingers[a].x, info->fingers[a].y, info->fingers[a].state);
	SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("[swap] [%d](%d, %d), state=%d\n", b,
		info->fingers[b].x, info->fingers[b].y, info->fingers[b].state);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_lgm_split_touch_combining_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	u8 fingerNum = 0;
	u8 id0, id1;
	
	u8 swap = 0;
	int dist_x = 0;
	int dist_y = 0;

	if (SHTPS_LGM_SPLIT_TOUCH_COMBINE_ENABLE == 0) {
		return;
	}

	SHTPS_LOG_FUNC_CALL();
	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			id1 = i;
			fingerNum++;
		}
	}

	if(fingerNum == 1){
		if(ts->lgm_split_touch_combining_p->fingerBase == 0xFF){
			ts->lgm_split_touch_combining_p->fingerBase  = id1;
			ts->lgm_split_touch_combining_p->finger1st   = id1;
			ts->lgm_split_touch_combining_p->finger_swap = 0;
			ts->lgm_split_touch_combining_p->pos_hist_cnt= 0;
		}
	}else{
		ts->lgm_split_touch_combining_p->fingerBase  = 0xFF;
		ts->lgm_split_touch_combining_p->finger_swap = 0;
		ts->lgm_split_touch_combining_p->pos_hist_cnt= 0;
		return;
	}
	
	id0 = ts->lgm_split_touch_combining_p->finger1st;
	if(id0 >= fingerMax || id1 >= fingerMax || ts->lgm_split_touch_combining_p->fingerBase >= fingerMax){
		SHTPS_LOG_ERR_PRINT("%s() Invalid finger id. (%d, %d, %d)\n",
								__func__, id0, id1, ts->lgm_split_touch_combining_p->fingerBase);
		return;
	}
	
	if(ts->lgm_split_touch_combining_p->pos_hist_cnt > 0){
		u8 idx = ts->lgm_split_touch_combining_p->pos_hist_cnt - 1;
		if(time_after(jiffies, ts->lgm_split_touch_combining_p->pos_hist_t[idx] + msecs_to_jiffies(SHTPS_LGM_SPLIT_TOUCH_COMBINE_TIME_THRESH)) == 0)
		{
			if(ts->lgm_split_touch_combining_p->pos_hist_cnt >= 2){
				unsigned long diff_x = abs(ts->lgm_split_touch_combining_p->pos_hist_x[idx] - ts->lgm_split_touch_combining_p->pos_hist_x[idx - 1]);
				unsigned long diff_y = abs(ts->lgm_split_touch_combining_p->pos_hist_y[idx] - ts->lgm_split_touch_combining_p->pos_hist_y[idx - 1]);
				
				dist_x = (diff_x * SHTPS_LGM_SPLIT_TOUCH_COMBINE_DIST_THRESH_RATIO) / 100;
				dist_y = (diff_y * SHTPS_LGM_SPLIT_TOUCH_COMBINE_DIST_THRESH_RATIO) / 100;
			}
		}
	}

	if( ((ts->report_info.fingers[id0].state == SHTPS_TOUCH_STATE_FINGER) || (ts->report_info.fingers[id0].state == SHTPS_TOUCH_STATE_PEN)) &&
		(info->fingers[id0].state == SHTPS_TOUCH_STATE_NO_TOUCH) &&
		(ts->report_info.fingers[id1].state == SHTPS_TOUCH_STATE_NO_TOUCH) &&
		((info->fingers[id1].state == SHTPS_TOUCH_STATE_FINGER) || (info->fingers[id1].state == SHTPS_TOUCH_STATE_PEN)) )
	{
		if(dist_x < SHTPS_LGM_SPLIT_TOUCH_COMBINE_DIST_THRESH){
			dist_x = SHTPS_LGM_SPLIT_TOUCH_COMBINE_DIST_THRESH;
		}

		if(dist_y < SHTPS_LGM_SPLIT_TOUCH_COMBINE_DIST_THRESH){
			dist_y = SHTPS_LGM_SPLIT_TOUCH_COMBINE_DIST_THRESH;
		}

		SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("threshold = (%d, %d)\n", dist_x, dist_y);

		if( (abs(ts->report_info.fingers[id0].x - info->fingers[id1].x) < dist_x) &&
			(abs(ts->report_info.fingers[id0].y - info->fingers[id1].y) < dist_y) )
		{
			swap = 1;
			SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("need swap status detect\n");
		}
	}

	if(swap == 1){
		if(ts->lgm_split_touch_combining_p->fingerBase != id1){
			ts->lgm_split_touch_combining_p->finger_adjust = 1;
			shtps_filter_swap_finger(ts, info, ts->lgm_split_touch_combining_p->fingerBase, id1);
			ts->lgm_split_touch_combining_p->finger_swap = 1;
		}else{
			int diff_x_1, diff_y_1;
			int diff_x_2, diff_y_2;

			diff_x_1 = ts->report_info.fingers[id1].x - info->fingers[id1].x;
			diff_y_1 = ts->report_info.fingers[id1].y - info->fingers[id1].y;
			diff_x_2 = ts->report_info.fingers[id0].x - info->fingers[id0].x;
			diff_y_2 = ts->report_info.fingers[id0].y - info->fingers[id0].y;

			info->fingers[id1].x = ts->report_info.fingers[id1].x - (diff_x_1 / 2);
			info->fingers[id1].y = ts->report_info.fingers[id1].y - (diff_y_1 / 2);
			info->fingers[id0].x = ts->report_info.fingers[id0].x - (diff_x_2 / 2);
			info->fingers[id0].y = ts->report_info.fingers[id0].y - (diff_y_2 / 2);

			ts->lgm_split_touch_combining_p->finger_swap = 0;

			SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("[re-swap] [%d](%d, %d), state=%d\n",
				id1, info->fingers[id1].x, info->fingers[id1].y, info->fingers[id1].state);
			SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("[re-swap] [%d](%d, %d), state=%d\n",
				id0, info->fingers[id0].x, info->fingers[id0].y, info->fingers[id0].state);
			
		}
		ts->lgm_split_touch_combining_p->finger1st = id1;
	}
	else if(ts->lgm_split_touch_combining_p->finger_swap != 0){
		shtps_filter_swap_finger(ts, info, ts->lgm_split_touch_combining_p->fingerBase, id1);
	}

	if (ts->lgm_split_touch_combining_p->finger_swap) {
		SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("finger swapped %d<->%d",
											ts->lgm_split_touch_combining_p->fingerBase, id1);
	}
	
	if(info->fingers[ts->lgm_split_touch_combining_p->fingerBase].state != SHTPS_TOUCH_STATE_NO_TOUCH){
		u8 i;
		u8 idx;
		if(ts->lgm_split_touch_combining_p->pos_hist_cnt < SHTPS_LGM_SPLIT_TOUCH_COMBINE_POS_HIST_CNT){
			idx = ts->lgm_split_touch_combining_p->pos_hist_cnt;
			ts->lgm_split_touch_combining_p->pos_hist_cnt++;
		}else{
			idx = SHTPS_LGM_SPLIT_TOUCH_COMBINE_POS_HIST_CNT - 1;
			for(i = 0;i < SHTPS_LGM_SPLIT_TOUCH_COMBINE_POS_HIST_CNT - 1;i++){
				ts->lgm_split_touch_combining_p->pos_hist_x[i] = ts->lgm_split_touch_combining_p->pos_hist_x[i+1];
				ts->lgm_split_touch_combining_p->pos_hist_y[i] = ts->lgm_split_touch_combining_p->pos_hist_y[i+1];
				ts->lgm_split_touch_combining_p->pos_hist_t[i] = ts->lgm_split_touch_combining_p->pos_hist_t[i+1];
			}
		}
		
		ts->lgm_split_touch_combining_p->pos_hist_x[idx] = info->fingers[ts->lgm_split_touch_combining_p->fingerBase].x;
		ts->lgm_split_touch_combining_p->pos_hist_y[idx] = info->fingers[ts->lgm_split_touch_combining_p->fingerBase].y;
		ts->lgm_split_touch_combining_p->pos_hist_t[idx] = jiffies;
	}else{
		ts->lgm_split_touch_combining_p->pos_hist_cnt = 0;
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_lgm_split_touch_combining_check_forcetouchup(struct shtps_rmi_spi *ts)
{
	ts->lgm_split_touch_combining_p->finger_swap = 0;
	ts->lgm_split_touch_combining_p->finger_adjust = 0;
	ts->lgm_split_touch_combining_p->fingerBase = 0xFF;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_lgm_split_touch_combining_sleep(struct shtps_rmi_spi *ts)
{
	ts->lgm_split_touch_combining_p->finger_swap = 0;
	ts->lgm_split_touch_combining_p->finger_adjust = 0;
	ts->lgm_split_touch_combining_p->fingerBase = 0xFF;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_lgm_split_touch_combining_init(struct shtps_rmi_spi *ts)
{
	ts->lgm_split_touch_combining_p = kzalloc(sizeof(struct shtps_filter_lgm_split_touch_combining_info), GFP_KERNEL);
	if(ts->lgm_split_touch_combining_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(&ts->lgm_split_touch_combining, 0, sizeof(ts->lgm_split_touch_combining));	// no need
	ts->lgm_split_touch_combining_p->fingerBase = 0xFF;
}
/* -------------------------------------------------------------------------- */
void shtps_filter_lgm_split_touch_combining_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->lgm_split_touch_combining_p)	kfree(ts->lgm_split_touch_combining_p);
	ts->lgm_split_touch_combining_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */
