/* drivers/sharp/shtps/sy3000/shtps_filter_hover_jitter_filter_p->c
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
#if defined(SHTPS_HOVER_HOST_JITTER_FILTER_ENABLE)
struct shtps_filter_hover_jitter_filter_info{
	int					dir;
	int					jump_dir;
	unsigned short		jump_base;
	u8					jump_count;
};

/* -------------------------------------------------------------------------- */
void shtps_filter_hover_jitter_filter(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	SHTPS_LOG_FUNC_CALL();
	if(!SHTPS_HOVER_JITTER_FILTER_DISABLE){
		if(ts->report_info.fingers[0].state == SHTPS_TOUCH_STATE_HOVER &&
			ts->fw_report_info.fingers[0].state == SHTPS_TOUCH_STATE_HOVER)
		{
			int dir  = (ts->fw_report_info.fingers[0].x >= ts->report_info.fingers[0].x)? 1: -1;
			int diff = (ts->fw_report_info.fingers[0].x >= ts->report_info.fingers[0].x)?
							ts->fw_report_info.fingers[0].x - ts->report_info.fingers[0].x : 
							ts->report_info.fingers[0].x - ts->fw_report_info.fingers[0].x;

			if(ts->hover_jitter_filter_p->dir == 0 || ts->hover_jitter_filter_p->dir != dir){
				if(diff <= SHTPS_HOVER_JITTER_FILTER_THRESHOLD){
					info->fingers[0].x = ts->report_info.fingers[0].x;
					SHTPS_LOG_DBG_PRINT("[hover_jitter_filter][0] change xpos %d => %d\n",
						ts->fw_report_info.fingers[0].x, info->fingers[0].x);
				}else{
					ts->hover_jitter_filter_p->dir = dir;
				}
			}
			
			if(diff >= SHTPS_HOVER_JITTER_FILTER_JUMP_THRESHOLD){
				if(!ts->hover_jitter_filter_p->jump_count){
					ts->hover_jitter_filter_p->jump_base = ts->fw_report_info.fingers[0].x;
					ts->hover_jitter_filter_p->jump_dir  = dir;
					ts->hover_jitter_filter_p->jump_count++;
				}else{
					ts->hover_jitter_filter_p->jump_count++;
					if(ts->hover_jitter_filter_p->jump_dir == dir){
						if((dir == 1 && ts->hover_jitter_filter_p->jump_base < ts->fw_report_info.fingers[0].x) ||
						   (dir ==-1 && ts->hover_jitter_filter_p->jump_base > ts->fw_report_info.fingers[0].x))
						{
							ts->hover_jitter_filter_p->jump_count = 0;
							SHTPS_LOG_DBG_PRINT("[hover_jitter_filter] clear jump condition (before xos = %d, current xpos = %d)\n", 
													ts->hover_jitter_filter_p->jump_base, ts->fw_report_info.fingers[0].x);
						}
					}
				}
				
				if(ts->hover_jitter_filter_p->jump_count > 0 &&
					ts->hover_jitter_filter_p->jump_count < SHTPS_HOVER_JITTER_FILTER_JUMP_COUNT)
				{
					info->fingers[0].x = ts->report_info.fingers[0].x;
					SHTPS_LOG_DBG_PRINT("[hover_jitter_filter][1] change xpos %d => %d\n",
						ts->fw_report_info.fingers[0].x, info->fingers[0].x);
				}else{
					ts->hover_jitter_filter_p->jump_count = 0;
				}
			}else{
				ts->hover_jitter_filter_p->jump_count = 0;
			}
		}else{
			memset(ts->hover_jitter_filter_p, 0, sizeof(ts->hover_jitter_filter_p));
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_hover_jitter_filter_init(struct shtps_rmi_spi *ts)
{
	ts->hover_jitter_filter_p = kzalloc(sizeof(struct shtps_filter_hover_jitter_filter_info), GFP_KERNEL);
	if(ts->hover_jitter_filter_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(&ts->hover_jitter_filter_p, 0, sizeof(ts->hover_jitter_filter_p));	// no need
}

/* -------------------------------------------------------------------------- */
void shtps_filter_hover_jitter_filter_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->hover_jitter_filter_p)	kfree(ts->hover_jitter_filter_p);
	ts->hover_jitter_filter_p = NULL;
}

#endif /* SHTPS_HOVER_HOST_JITTER_FILTER_ENABLE */
