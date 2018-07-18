/* drivers/sharp/shtps/sy3000/shtps_filter_multi_hover_select.c
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
#if defined(SHTPS_MULTI_HOVER_SELECT_ENABLE)
struct shtps_filter_multi_hover_select_info{
	u8						report_hover_id;
	struct shtps_touch_info report_hover_info;
};

/* -------------------------------------------------------------------------- */
void shtps_filter_multi_hover_select(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int hover_id = -1;
	int max_z = -1;

	const int detect_min_x = SHTPS_MULTI_HOVER_EDGE_FAIL_RANGE_X;
	const int detect_max_x = CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_MULTI_HOVER_EDGE_FAIL_RANGE_X;
	int x;
	if (ts->multi_hover_select_p->report_hover_id >= 0) {
		for (i = 0; i < fingerMax; i++) {
			if (info->fingers[i].state == SHTPS_TOUCH_STATE_HOVER) {
				if (max_z < info->fingers[i].z) {
					max_z = info->fingers[i].z;
					x = info->fingers[i].x;
					if (x > detect_min_x && x < detect_max_x) {
						hover_id = i;
					}
				}
			}
		}
		if (hover_id < 0) {
			// use last reported id
			hover_id = ts->multi_hover_select_p->report_hover_id;
		}
	} else {
		for (i = 0; i < fingerMax; i++) {
			if (info->fingers[i].state == SHTPS_TOUCH_STATE_HOVER) {
				if (max_z < info->fingers[i].z) {
					max_z = info->fingers[i].z;
					hover_id = i;
				}
			}
		}
	}

	if (max_z > 0) {
		if (ts->multi_hover_select_p->report_hover_id >= 0 && hover_id >= 0) {
			if (ts->multi_hover_select_p->report_hover_info.fingers[0].x > detect_min_x &&
				ts->multi_hover_select_p->report_hover_info.fingers[0].x < detect_max_x) {
				if (info->fingers[hover_id].x <= detect_min_x ||
					info->fingers[hover_id].x >= detect_max_x) {
					int diff = ts->multi_hover_select_p->report_hover_info.fingers[0].x - info->fingers[hover_id].x;
					if (diff < 0) diff = -diff;
					if (diff > SHTPS_MULTI_HOVER_DIFF_THRESH_X) {
						hover_id = -1;
					}
				}
			}
		}

		ts->multi_hover_select_p->report_hover_id = hover_id;
		if (hover_id >= 0) {
			ts->multi_hover_select_p->report_hover_info.fingers[0].state = info->fingers[hover_id].state;
			ts->multi_hover_select_p->report_hover_info.fingers[0].x     = info->fingers[hover_id].x;
			ts->multi_hover_select_p->report_hover_info.fingers[0].y     = info->fingers[hover_id].y;
			ts->multi_hover_select_p->report_hover_info.fingers[0].wx    = info->fingers[hover_id].wx;
			ts->multi_hover_select_p->report_hover_info.fingers[0].wy    = info->fingers[hover_id].wy;
			ts->multi_hover_select_p->report_hover_info.fingers[0].z     = info->fingers[hover_id].z;
		}

		for (i = 0; i < fingerMax; i++) {
			if (info->fingers[i].state == SHTPS_TOUCH_STATE_HOVER) {
				info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
				ts->fw_report_info.fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			}
		}

		if (hover_id >= 0) {
			ts->fw_report_info.fingers[0].state = ts->multi_hover_select_p->report_hover_info.fingers[0].state;
			ts->fw_report_info.fingers[0].x     = ts->multi_hover_select_p->report_hover_info.fingers[0].x;
			ts->fw_report_info.fingers[0].y     = ts->multi_hover_select_p->report_hover_info.fingers[0].y;
			ts->fw_report_info.fingers[0].wx    = ts->multi_hover_select_p->report_hover_info.fingers[0].wx;
			ts->fw_report_info.fingers[0].wy    = ts->multi_hover_select_p->report_hover_info.fingers[0].wy;
			ts->fw_report_info.fingers[0].z     = ts->multi_hover_select_p->report_hover_info.fingers[0].z;

			info->fingers[0].state = ts->fw_report_info.fingers[0].state;
			info->fingers[0].x     = ts->fw_report_info.fingers[0].x;
			info->fingers[0].y     = ts->fw_report_info.fingers[0].y;
			info->fingers[0].wx    = ts->fw_report_info.fingers[0].wx;
			info->fingers[0].wy    = ts->fw_report_info.fingers[0].wy;
			info->fingers[0].z     = ts->fw_report_info.fingers[0].z;
		}
	} else {
		// no hover events exist
		ts->multi_hover_select_p->report_hover_id = -1;
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_multi_hover_select_init(struct shtps_rmi_spi *ts)
{
	ts->multi_hover_select_p = kzalloc(sizeof(struct shtps_filter_multi_hover_select_info), GFP_KERNEL);
	if(ts->multi_hover_select_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(&ts->multi_hover_select_p->report_hover_info, 0, sizeof(ts->multi_hover_select_p->report_hover_info));	// no need
	ts->multi_hover_select_p->report_hover_id = (-1);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_multi_hover_select_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->multi_hover_select_p)	kfree(ts->multi_hover_select_p);
	ts->multi_hover_select_p = NULL;

}
#endif /* SHTPS_MULTI_HOVER_SELECT_ENABLE */
