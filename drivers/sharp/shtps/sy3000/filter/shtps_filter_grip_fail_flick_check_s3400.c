/* drivers/sharp/shtps/sy3000/shtps_filter_grip_fail_flick_check_s3400.c
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
#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
struct shtps_filter_grip_fail_info{
	u32	padding;
	u8	grip_fail_touch_inhibit_id;
	u8	grip_fail_flick_inhibit_id;
	u8	pad[2];
};

/* -------------------------------------------------------------------------- */
void shtps_filter_grip_fail_flick_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int diff_x;
	int diff_y;
	int w;

	if(SHTPS_GRIP_FAIL_FLICK_REJECT_ENABLE == 0){
		return;
	}

	SHTPS_LOG_FUNC_CALL();

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			if( (ts->filter_grip_fail_p->grip_fail_flick_inhibit_id & (1 << i)) != 0 ){
				ts->filter_grip_fail_p->grip_fail_flick_inhibit_id &= ~(1 << i);
			}
		}

		if( (ts->filter_grip_fail_p->grip_fail_flick_inhibit_id & (1 << i)) == 0 ){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
					if( (ts->fw_report_info.fingers[i].x < SHTPS_GRIP_FAIL_FLICK_REJECT_AREA) ||
						(ts->fw_report_info.fingers[i].x > (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_GRIP_FAIL_FLICK_REJECT_AREA)) )
					{
						diff_x = shtps_get_diff(ts->fw_report_info.fingers[i].x, ts->report_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
						diff_y = shtps_get_diff(ts->fw_report_info.fingers[i].y, ts->report_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));
						w = (ts->fw_report_info.fingers[i].wx > ts->fw_report_info.fingers[i].wy) ?
							 ts->fw_report_info.fingers[i].wx : ts->fw_report_info.fingers[i].wy;

						if(w == 0){
							w = 1;
						}

						if(((diff_x * diff_x) + (diff_y * diff_y)) > (SHTPS_GRIP_FAIL_FLICK_REJECT_DIST * SHTPS_GRIP_FAIL_FLICK_REJECT_DIST)){
							if(SHTPS_GRIP_FAIL_FLICK_REJECT_ZW_RATIO_THRESH > 0){
								if( ((ts->fw_report_info.fingers[i].z * 100) / w) <= SHTPS_GRIP_FAIL_FLICK_REJECT_ZW_RATIO_THRESH ){
									ts->filter_grip_fail_p->grip_fail_flick_inhibit_id |= (1 << i);
									SHTPS_LOG_DBG_GRIP_FAIL_TOUCH_PRINT("[flick]inhibit by z/w thresh (id : %d)\n", i);
								}
							}

							if(SHTPS_GRIP_FAIL_FLICK_REJECT_W_THRESH > 0){
								if(w >= SHTPS_GRIP_FAIL_FLICK_REJECT_W_THRESH){
									ts->filter_grip_fail_p->grip_fail_flick_inhibit_id |= (1 << i);
									SHTPS_LOG_DBG_GRIP_FAIL_TOUCH_PRINT("[flick]inhibit by w thresh (id : %d)\n", i);
								}
							}
						}
					}
				}
			}
		}
	}

	for(i = 0; i < fingerMax; i++){
		if( (ts->filter_grip_fail_p->grip_fail_flick_inhibit_id & (1 << i)) != 0 ){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			SHTPS_LOG_DBG_GRIP_FAIL_TOUCH_PRINT("[flick][%d] is inhibited\n", i);
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_grip_fail_touch_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);

	if(SHTPS_GRIP_FAIL_TOUCH_REJECT_ENABLE == 0){
		return;
	}

	SHTPS_LOG_FUNC_CALL();
	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			if( (ts->filter_grip_fail_p->grip_fail_touch_inhibit_id & (1 << i)) != 0 ){
				ts->filter_grip_fail_p->grip_fail_touch_inhibit_id &= ~(1 << i);
			}
		}

		if( (ts->filter_grip_fail_p->grip_fail_touch_inhibit_id & (1 << i)) != 0 ){
			if( (ts->fw_report_info.fingers[i].x >= SHTPS_GRIP_FAIL_TOUCH_REJECT_AREA) &&
				(ts->fw_report_info.fingers[i].x <= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_GRIP_FAIL_TOUCH_REJECT_AREA)) )
			{
				ts->filter_grip_fail_p->grip_fail_touch_inhibit_id &= ~(1 << i);
				SHTPS_LOG_DBG_GRIP_FAIL_TOUCH_PRINT("[%d] inhibit cancel by area over\n", i);
			}
		}
		if( (ts->filter_grip_fail_p->grip_fail_touch_inhibit_id & (1 << i)) == 0 ){
			if( (info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER) &&
				(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) )
			{
				if( (ts->fw_report_info.fingers[i].x < SHTPS_GRIP_FAIL_TOUCH_REJECT_AREA) ||
					(ts->fw_report_info.fingers[i].x > (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_GRIP_FAIL_TOUCH_REJECT_AREA)) )
				{
					if( (ts->fw_report_info.fingers[i].wx >= SHTPS_GRIP_FAIL_TOUCH_REJECT_W_THRESH) ||
						(ts->fw_report_info.fingers[i].wy >= SHTPS_GRIP_FAIL_TOUCH_REJECT_W_THRESH) )
					{
						ts->filter_grip_fail_p->grip_fail_touch_inhibit_id |= (1 << i);
					}
				}
			}
		}
	}

	for(i = 0; i < fingerMax; i++){
		if( (ts->filter_grip_fail_p->grip_fail_touch_inhibit_id & (1 << i)) != 0 ){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			SHTPS_LOG_DBG_GRIP_FAIL_TOUCH_PRINT("[td][%d] is inhibited\n", i);
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_grip_fail_sleep(struct shtps_rmi_spi *ts)
{
	ts->filter_grip_fail_p->grip_fail_touch_inhibit_id = 0;
	ts->filter_grip_fail_p->grip_fail_flick_inhibit_id = 0;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_grip_fail_init(struct shtps_rmi_spi *ts)
{
	ts->filter_grip_fail_p = kzalloc(sizeof(struct shtps_filter_grip_fail_info), GFP_KERNEL);
	if(ts->filter_grip_fail_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// ts->filter_grip_fail_p->grip_fail_touch_inhibit_id = 0;	// no need
	// ts->filter_grip_fail_p->grip_fail_flick_inhibit_id = 0;	// no need
}

/* -------------------------------------------------------------------------- */
void shtps_filter_grip_fail_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->filter_grip_fail_p)	kfree(ts->filter_grip_fail_p);
	ts->filter_grip_fail_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */
