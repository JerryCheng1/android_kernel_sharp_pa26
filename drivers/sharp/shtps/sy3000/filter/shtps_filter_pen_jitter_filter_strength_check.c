/* drivers/sharp/shtps/sy3000/shtps_filter_pen_jitter_filter_strength_check.c
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
#include "shtps_fwctl.h"
#include "shtps_log.h"

#include "shtps_filter.h"
/* -------------------------------------------------------------------------- */
#if defined( SHTPS_VARIABLE_PEN_JITTER_ENABLE )
struct shtps_filter_pen_jitter_filter_info{
	unsigned long	variable_pen_jitter_hist_time[SHTPS_FINGER_MAX][16];
	u16				variable_pen_jitter_hist_x[SHTPS_FINGER_MAX][16];
	u16				variable_pen_jitter_hist_y[SHTPS_FINGER_MAX][16];
	u8				variable_pen_jitter_hist_count[SHTPS_FINGER_MAX];
	u8				variable_pen_jitter_def_val;
	u8				variable_pen_jitter_changed;
};

static void shtps_filter_set_pen_jitter_filter_strength(struct shtps_rmi_spi *ts, u8 val)
{
	if(SHTPS_HOST_SET_PEN_JITTER_FILTER_ENABLE == 0){
		return;
	}
	shtps_fwctl_reg_write_pen_jitter(ts, &val);
	SHTPS_LOG_DBG_PRINT("[VARIABLE_PEN_JITTER] Pen jitter filter strength is changed to 0x%02X\n", val);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_get_register_pen_jitter(struct shtps_rmi_spi *ts)
{
	shtps_fwctl_get_pen_jitter(ts, &ts->pen_jitter_filter_p->variable_pen_jitter_def_val);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pen_jitter_filter_strength_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	u8 is_pendown = 0;

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_PEN)	continue;
		is_pendown = 1;

		if(ts->pen_jitter_filter_p->variable_pen_jitter_changed != 0)	continue;

		if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			ts->pen_jitter_filter_p->variable_pen_jitter_hist_count[i] = 0;
		}

		if(ts->pen_jitter_filter_p->variable_pen_jitter_hist_count[i] < SHTPS_VARIABLE_PEN_JITTER_COMPARE_HIST){
			ts->pen_jitter_filter_p->variable_pen_jitter_hist_time[i][ts->pen_jitter_filter_p->variable_pen_jitter_hist_count[i]] = jiffies + msecs_to_jiffies(SHTPS_VARIABLE_PEN_JITTER_TIME_THRESHOLD);
			ts->pen_jitter_filter_p->variable_pen_jitter_hist_x[i][ts->pen_jitter_filter_p->variable_pen_jitter_hist_count[i]] = ts->fw_report_info.fingers[i].x;
			ts->pen_jitter_filter_p->variable_pen_jitter_hist_y[i][ts->pen_jitter_filter_p->variable_pen_jitter_hist_count[i]] = ts->fw_report_info.fingers[i].y;
			ts->pen_jitter_filter_p->variable_pen_jitter_hist_count[i]++;

		}else{
			u16 delta_x = (ts->pen_jitter_filter_p->variable_pen_jitter_hist_x[i][0] > ts->fw_report_info.fingers[i].x)?
								ts->pen_jitter_filter_p->variable_pen_jitter_hist_x[i][0] - ts->fw_report_info.fingers[i].x :
								ts->fw_report_info.fingers[i].x - ts->pen_jitter_filter_p->variable_pen_jitter_hist_x[i][0];

			u16 delta_y = (ts->pen_jitter_filter_p->variable_pen_jitter_hist_y[i][0] > ts->fw_report_info.fingers[i].y)?
								ts->pen_jitter_filter_p->variable_pen_jitter_hist_y[i][0] - ts->fw_report_info.fingers[i].y :
								ts->fw_report_info.fingers[i].y - ts->pen_jitter_filter_p->variable_pen_jitter_hist_y[i][0];

			if(time_after(jiffies, ts->pen_jitter_filter_p->variable_pen_jitter_hist_time[i][0]) == 0 &&
				((delta_x * delta_x) + (delta_y * delta_y)) >= (SHTPS_VARIABLE_PEN_JITTER_POSITION_THRESHOLD * SHTPS_VARIABLE_PEN_JITTER_POSITION_THRESHOLD))
			{
				ts->pen_jitter_filter_p->variable_pen_jitter_changed = 1;
				shtps_filter_set_pen_jitter_filter_strength(ts, SHTPS_VARIABLE_PEN_JITTER_FILTER_VAL);

			}else{
				int j;
				for(j = 0;j < (SHTPS_VARIABLE_PEN_JITTER_COMPARE_HIST - 1);j++){
					ts->pen_jitter_filter_p->variable_pen_jitter_hist_time[i][j] = ts->pen_jitter_filter_p->variable_pen_jitter_hist_time[i][j+1];
					ts->pen_jitter_filter_p->variable_pen_jitter_hist_x[i][j] = ts->pen_jitter_filter_p->variable_pen_jitter_hist_x[i][j+1];
					ts->pen_jitter_filter_p->variable_pen_jitter_hist_y[i][j] = ts->pen_jitter_filter_p->variable_pen_jitter_hist_y[i][j+1];
				}
				ts->pen_jitter_filter_p->variable_pen_jitter_hist_time[i][ts->pen_jitter_filter_p->variable_pen_jitter_hist_count[i]-1] = jiffies + msecs_to_jiffies(SHTPS_VARIABLE_PEN_JITTER_TIME_THRESHOLD);
				ts->pen_jitter_filter_p->variable_pen_jitter_hist_x[i][ts->pen_jitter_filter_p->variable_pen_jitter_hist_count[i]-1] = ts->fw_report_info.fingers[i].x;
				ts->pen_jitter_filter_p->variable_pen_jitter_hist_y[i][ts->pen_jitter_filter_p->variable_pen_jitter_hist_count[i]-1] = ts->fw_report_info.fingers[i].y;
			}
		}
	}

	if(ts->pen_jitter_filter_p->variable_pen_jitter_changed != 0 && is_pendown == 0){
		ts->pen_jitter_filter_p->variable_pen_jitter_changed = 0;
		shtps_filter_set_pen_jitter_filter_strength(ts, ts->pen_jitter_filter_p->variable_pen_jitter_def_val);
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pen_jitter_filter_strength_sleep(struct shtps_rmi_spi *ts)
{
	if(ts->pen_jitter_filter_p->variable_pen_jitter_changed != 0){
		shtps_filter_set_pen_jitter_filter_strength(ts, ts->pen_jitter_filter_p->variable_pen_jitter_def_val);
	}
	ts->pen_jitter_filter_p->variable_pen_jitter_changed = 0;
	memset(ts->pen_jitter_filter_p->variable_pen_jitter_hist_count, 0, sizeof(ts->pen_jitter_filter_p->variable_pen_jitter_hist_count));
	memset(ts->pen_jitter_filter_p->variable_pen_jitter_hist_time, 0, sizeof(ts->pen_jitter_filter_p->variable_pen_jitter_hist_time));
	memset(ts->pen_jitter_filter_p->variable_pen_jitter_hist_x, 0, sizeof(ts->pen_jitter_filter_p->variable_pen_jitter_hist_x));
	memset(ts->pen_jitter_filter_p->variable_pen_jitter_hist_y, 0, sizeof(ts->pen_jitter_filter_p->variable_pen_jitter_hist_y));
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pen_jitter_filter_strength_init(struct shtps_rmi_spi *ts)
{
	ts->pen_jitter_filter_p = kzalloc(sizeof(struct shtps_filter_pen_jitter_filter_info), GFP_KERNEL);
	if(ts->pen_jitter_filter_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	#if 0 // no need
	ts->pen_jitter_filter_p->variable_pen_jitter_def_val = 0;
	ts->pen_jitter_filter_p->variable_pen_jitter_changed = 0;
	memset(ts->pen_jitter_filter_p->variable_pen_jitter_hist_count, 0, sizeof(ts->pen_jitter_filter_p->variable_pen_jitter_hist_count));
	memset(ts->pen_jitter_filter_p->variable_pen_jitter_hist_time, 0, sizeof(ts->pen_jitter_filter_p->variable_pen_jitter_hist_time));
	memset(ts->pen_jitter_filter_p->variable_pen_jitter_hist_x, 0, sizeof(ts->pen_jitter_filter_p->variable_pen_jitter_hist_x));
	memset(ts->pen_jitter_filter_p->variable_pen_jitter_hist_y, 0, sizeof(ts->pen_jitter_filter_p->variable_pen_jitter_hist_y));
	#endif
}

/* -------------------------------------------------------------------------- */
void shtps_filter_pen_jitter_filter_strength_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->pen_jitter_filter_p)	kfree(ts->pen_jitter_filter_p);
	ts->pen_jitter_filter_p = NULL;
}
/* -------------------------------------------------------------------------- */
#endif /* SHTPS_VARIABLE_PEN_JITTER_ENABLE */

