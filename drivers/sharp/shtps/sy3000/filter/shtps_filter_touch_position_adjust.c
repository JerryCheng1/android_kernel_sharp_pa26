/* drivers/sharp/shtps/sy3000/shtps_touch_position_adjust.c
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
#if defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
enum{
	SHTPS_SHIFT_EDGE_INWARD_STATE_IDLE     = 0x00,
	SHTPS_SHIFT_EDGE_INWARD_STATE_ON_LEFT  = 0x01,
	SHTPS_SHIFT_EDGE_INWARD_STATE_ON_RIGHT = 0x02,
	SHTPS_SHIFT_EDGE_INWARD_STATE_ON_TOP   = 0x04,
	SHTPS_SHIFT_EDGE_INWARD_STATE_ON_BOTTOM= 0x08,
};

struct shtps_filter_shift_edge_inward_info{
	u8 state[SHTPS_FINGER_MAX];
};
#endif /* SHTPS_SHIFT_EDGE_INWARD_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_EDGE_POS_ADJUST_ENABLE)
void shtps_filter_touch_position_adjust_edge(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);

	int adj_x;
	int adj_y;

	int detect_min_x = SHTPS_EDGE_DISABLE_AREA_OFFSET_X;
	int detect_min_y = SHTPS_EDGE_DISABLE_AREA_OFFSET_Y;
	int detect_max_x = (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_EDGE_DISABLE_AREA_OFFSET_X);
	int detect_max_y = (CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1 - SHTPS_EDGE_DISABLE_AREA_OFFSET_Y);

	int adj_area_for_min_x = SHTPS_EDGE_ADJUST_AREA_OFFSET_X;
	int adj_area_for_min_y = SHTPS_EDGE_ADJUST_AREA_OFFSET_Y;
	int adj_area_for_max_x = (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_EDGE_ADJUST_AREA_OFFSET_X);
	int adj_area_for_max_y = (CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1 - SHTPS_EDGE_ADJUST_AREA_OFFSET_Y);

	SHTPS_LOG_FUNC_CALL();

	for(i = 0; i < fingerMax; i++){
		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER)
		{
			adj_x = info->fingers[i].x;
			adj_y = info->fingers[i].y;

			if(info->fingers[i].x < detect_min_x){
				adj_x = 0;
			}else if(detect_max_x < info->fingers[i].x){
				adj_x = CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1;
			}else if(info->fingers[i].x < adj_area_for_min_x){
				adj_x = adj_area_for_min_x -
						((adj_area_for_min_x - (info->fingers[i].x)) *
						 (adj_area_for_min_x - 0) /
						 (adj_area_for_min_x - detect_min_x));
			}else if(adj_area_for_max_x < info->fingers[i].x){
				adj_x = adj_area_for_max_x + 
						(((info->fingers[i].x) - adj_area_for_max_x) *
						 ((CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1) - adj_area_for_max_x) /
						 (detect_max_x - adj_area_for_max_x));
			}

			if(info->fingers[i].y < detect_min_y){
				adj_y = 0;
			}else if(detect_max_y < info->fingers[i].y){
				adj_y = CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1;
			}
			else if(info->fingers[i].y < adj_area_for_min_y)
			{
				adj_y = adj_area_for_min_y -
						((adj_area_for_min_y - (info->fingers[i].y)) *
						 (adj_area_for_min_y - 0) /
						 (adj_area_for_min_y - detect_min_y));
			}
			else if(adj_area_for_max_y < info->fingers[i].y)
			{
				adj_y = adj_area_for_max_y +
						(((info->fingers[i].y) - adj_area_for_max_y) *
						 ((CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1) - adj_area_for_max_y) /
						 (detect_max_y - adj_area_for_max_y));
			}

			if( (info->fingers[i].x != adj_x) || (info->fingers[i].y != adj_y) ){
				SHTPS_LOG_DBG_PRINT("pos adjust [X: %d -> %d][Y: %d -> %d]\n",
										info->fingers[i].x, adj_x, info->fingers[i].y, adj_y);
			}

			info->fingers[i].x = adj_x;
			info->fingers[i].y = adj_y;
			ts->fw_report_info.fingers[i].x = info->fingers[i].x;
			ts->fw_report_info.fingers[i].y = info->fingers[i].y;
		}
	}
}
#endif /* SHTPS_EDGE_POS_ADJUST_ENABLE */
/* -------------------------------------------------------------------------- */
#if defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
void shtps_filter_touch_position_adjust_shift_edge(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);

	if(SHTPS_SHIFT_EDGE_INWARD == 0){
		return;
	}

	SHTPS_LOG_FUNC_CALL();
	for(i = 0; i < fingerMax; i++){
		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER && ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			ts->shift_edge_inward_p->state[i] = SHTPS_SHIFT_EDGE_INWARD_STATE_IDLE;

			if(info->fingers[i].x == 0){
				SHTPS_LOG_DBG_PRINT("[SHIFT_EDGE_INWARD][%d](x : %d -> %d)\n", i, info->fingers[i].x, SHTPS_SHIFT_EDGE_INWARD_OFFSET);
				info->fingers[i].x = SHTPS_SHIFT_EDGE_INWARD_OFFSET;
				ts->shift_edge_inward_p->state[i] |= SHTPS_SHIFT_EDGE_INWARD_STATE_ON_LEFT;
			}else if(info->fingers[i].x == (CONFIG_SHTPS_SY3000_PANEL_SIZE_X-1)){
				SHTPS_LOG_DBG_PRINT("[SHIFT_EDGE_INWARD][%d](x : %d -> %d)\n",
										i, info->fingers[i].x, (CONFIG_SHTPS_SY3000_PANEL_SIZE_X-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET);
				info->fingers[i].x = (CONFIG_SHTPS_SY3000_PANEL_SIZE_X-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET;
				ts->shift_edge_inward_p->state[i] |= SHTPS_SHIFT_EDGE_INWARD_STATE_ON_RIGHT;
			}

			if(info->fingers[i].y == 0){
				SHTPS_LOG_DBG_PRINT("[SHIFT_EDGE_INWARD][%d](y : %d -> %d)\n", i, info->fingers[i].y, SHTPS_SHIFT_EDGE_INWARD_OFFSET);
				info->fingers[i].y = SHTPS_SHIFT_EDGE_INWARD_OFFSET;
				ts->shift_edge_inward_p->state[i] |= SHTPS_SHIFT_EDGE_INWARD_STATE_ON_TOP;
			}else if(info->fingers[i].y == (CONFIG_SHTPS_SY3000_PANEL_SIZE_Y-1)){
				SHTPS_LOG_DBG_PRINT("[SHIFT_EDGE_INWARD][%d](y : %d -> %d)\n",
										i, info->fingers[i].y, (CONFIG_SHTPS_SY3000_PANEL_SIZE_Y-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET);
				info->fingers[i].y = (CONFIG_SHTPS_SY3000_PANEL_SIZE_Y-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET;
				ts->shift_edge_inward_p->state[i] |= SHTPS_SHIFT_EDGE_INWARD_STATE_ON_BOTTOM;
			}
		}else if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			if((ts->shift_edge_inward_p->state[i] & SHTPS_SHIFT_EDGE_INWARD_STATE_ON_LEFT) != 0){
				if(info->fingers[i].x >= SHTPS_SHIFT_EDGE_INWARD_OFFSET){
					ts->shift_edge_inward_p->state[i] &= ~SHTPS_SHIFT_EDGE_INWARD_STATE_ON_LEFT;
				}else{
					SHTPS_LOG_DBG_PRINT("[SHIFT_EDGE_INWARD][%d](x : %d -> %d)\n", i, info->fingers[i].x, SHTPS_SHIFT_EDGE_INWARD_OFFSET);
					info->fingers[i].x = SHTPS_SHIFT_EDGE_INWARD_OFFSET;
				}
			}
			if((ts->shift_edge_inward_p->state[i] & SHTPS_SHIFT_EDGE_INWARD_STATE_ON_RIGHT) != 0){
				if(info->fingers[i].x <= (CONFIG_SHTPS_SY3000_PANEL_SIZE_X-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET){
					ts->shift_edge_inward_p->state[i] &= ~SHTPS_SHIFT_EDGE_INWARD_STATE_ON_RIGHT;
				}else{
					SHTPS_LOG_DBG_PRINT("[SHIFT_EDGE_INWARD][%d](x : %d -> %d)\n",
											i, info->fingers[i].x, (CONFIG_SHTPS_SY3000_PANEL_SIZE_X-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET);
					info->fingers[i].x = (CONFIG_SHTPS_SY3000_PANEL_SIZE_X-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET;
				}
			}
			if((ts->shift_edge_inward_p->state[i] & SHTPS_SHIFT_EDGE_INWARD_STATE_ON_TOP) != 0){
				if(info->fingers[i].y >= SHTPS_SHIFT_EDGE_INWARD_OFFSET){
					ts->shift_edge_inward_p->state[i] &= ~SHTPS_SHIFT_EDGE_INWARD_STATE_ON_TOP;
				}else{
					SHTPS_LOG_DBG_PRINT("[SHIFT_EDGE_INWARD][%d](y : %d -> %d)\n", i, info->fingers[i].y, SHTPS_SHIFT_EDGE_INWARD_OFFSET);
					info->fingers[i].y = SHTPS_SHIFT_EDGE_INWARD_OFFSET;
				}
			}
			if((ts->shift_edge_inward_p->state[i] & SHTPS_SHIFT_EDGE_INWARD_STATE_ON_BOTTOM) != 0){
				if(info->fingers[i].y <= (CONFIG_SHTPS_SY3000_PANEL_SIZE_Y-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET){
					ts->shift_edge_inward_p->state[i] &= ~SHTPS_SHIFT_EDGE_INWARD_STATE_ON_BOTTOM;
				}else{
					SHTPS_LOG_DBG_PRINT("[SHIFT_EDGE_INWARD][%d](y : %d -> %d)\n",
										i, info->fingers[i].y, (CONFIG_SHTPS_SY3000_PANEL_SIZE_Y-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET);
					info->fingers[i].y = (CONFIG_SHTPS_SY3000_PANEL_SIZE_Y-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET;
				}
			}
		}else{
			ts->shift_edge_inward_p->state[i] = SHTPS_SHIFT_EDGE_INWARD_STATE_IDLE;
		}
	}
}
#endif /* SHTPS_SHIFT_EDGE_INWARD_ENABLE */
/* -------------------------------------------------------------------------- */
#if defined(SHTPS_EDGE_POS_ADJUST_ENABLE) || defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
void shtps_filter_touch_position_adjust_init(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
		ts->shift_edge_inward_p = kzalloc(sizeof(struct shtps_filter_shift_edge_inward_info), GFP_KERNEL);
		if(ts->shift_edge_inward_p == NULL){
			PR_ERROR("memory allocation error:%s()\n", __func__);
			return;
		}
	#endif /* SHTPS_SHIFT_EDGE_INWARD_ENABLE */
}

/* -------------------------------------------------------------------------- */
void shtps_filter_touch_position_adjust_deinit(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
		if(ts->shift_edge_inward_p)	kfree(ts->shift_edge_inward_p);
		ts->shift_edge_inward_p = NULL;
	#endif /* SHTPS_SHIFT_EDGE_INWARD_ENABLE */
}

#endif	/* SHTPS_EDGE_POS_ADJUST_ENABLE || SHTPS_SHIFT_EDGE_INWARD_ENABLE */
/* -------------------------------------------------------------------------- */
