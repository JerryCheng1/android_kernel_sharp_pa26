/* drivers/sharp/shtps/sy3000/shtps_filter_diagonal_ghost_check.c
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
#if defined(SHTPS_DIAGONAL_GHOST_CHECK_ENABLE)
struct shtps_filter_diagonal_ghost_reject_info{	
	u8				state;
	unsigned short	finger_x[2];
	unsigned short	finger_y[2];
	unsigned long	tu_time;
};

/* -------------------------------------------------------------------------- */
static int shtps_filter_is_diagonal(int x1, int y1, int x2, int y2, int x3, int y3)
{
	int diagonal_x1 = x1;
	int diagonal_y1 = y2;
	int diagonal_x2 = x2;
	int diagonal_y2 = y1;
	int diff_x, diff_y;

	SHTPS_LOG_FUNC_CALL();
	diff_x = diagonal_x1 > x3 ? diagonal_x1 - x3 : x3 - diagonal_x1;
	diff_y = diagonal_y1 > y3 ? diagonal_y1 - y3 : y3 - diagonal_y1;
	if (diff_x < SHTPS_DIAGONAL_GHOST_CHECK_AREA &&
		diff_y < SHTPS_DIAGONAL_GHOST_CHECK_AREA) {
		return 1;
	}

	diff_x = diagonal_x2 > x3 ? diagonal_x2 - x3 : x3 - diagonal_x2;
	diff_y = diagonal_y2 > y3 ? diagonal_y2 - y3 : y3 - diagonal_y2;
	if (diff_x < SHTPS_DIAGONAL_GHOST_CHECK_AREA &&
		diff_y < SHTPS_DIAGONAL_GHOST_CHECK_AREA) {
		return 1;
	}

	return 0;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_diagonal_ghost_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int finger_x[2];
	int finger_y[2];
	int cnt;
	int diagonal;

	cnt = 0;
	for (i = 0; i < fingerMax; i++) {
		if (((info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER) || 
		     (info->fingers[i].state == SHTPS_TOUCH_STATE_PEN)) &&
			((ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) ||
			 (ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN)) ) {
			if (cnt < 2) {
				finger_x[cnt] = info->fingers[i].x;
				finger_y[cnt] = info->fingers[i].y;
				cnt++;
			}
		}
	}
	if (cnt != 2) {
		if (SHTPS_DIAGONAL_GHOST_CHECK_AFTER_TU_ENABLE){
			if (cnt < 2) {
				if (ts->diagonal_ghost_reject_p->state == 1){
					ts->diagonal_ghost_reject_p->state   = 2;
					ts->diagonal_ghost_reject_p->tu_time = jiffies;
				}
			} else {
				ts->diagonal_ghost_reject_p->state = 0;
			}
		}else{
			ts->diagonal_ghost_reject_p->state = 0;
		}
	} else {
		ts->diagonal_ghost_reject_p->state = 1;
	}

	if (ts->diagonal_ghost_reject_p->state == 2 && 
			time_after(jiffies, ts->diagonal_ghost_reject_p->tu_time + msecs_to_jiffies(SHTPS_DIAGONAL_GHOST_TIMEOUT)) )
	{
		ts->diagonal_ghost_reject_p->state = 0;
	}
	
	if (ts->diagonal_ghost_reject_p->state != 0) {
		if (ts->diagonal_ghost_reject_p->state == 1)
		{
			ts->diagonal_ghost_reject_p->finger_x[0] = finger_x[0];
			ts->diagonal_ghost_reject_p->finger_x[1] = finger_x[1];
			ts->diagonal_ghost_reject_p->finger_y[0] = finger_y[0];
			ts->diagonal_ghost_reject_p->finger_y[1] = finger_y[1];
		}
		
		for (i = 0; i < fingerMax; i++) {
			if (((info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER) ||
			     (info->fingers[i].state == SHTPS_TOUCH_STATE_PEN)) &&
				ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) {
				diagonal = shtps_filter_is_diagonal(
							ts->diagonal_ghost_reject_p->finger_x[0],
							ts->diagonal_ghost_reject_p->finger_y[0],
							ts->diagonal_ghost_reject_p->finger_x[1],
							ts->diagonal_ghost_reject_p->finger_y[1],
							info->fingers[i].x, 
							info->fingers[i].y);
				if (diagonal) {
					if (info->fingers[i].z <= SHTPS_DIAGONAL_GHOST_Z_THRESHOLD) {
						info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;

						SHTPS_LOG_DBG_PRINT("[diagonal] ignore ghost (id=%d, x=%d, y=%d, z=%d)\n",
								i,
								info->fingers[i].x,
								info->fingers[i].y,
								info->fingers[i].z);
					}
				}
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_diagonal_ghost_sleep(struct shtps_rmi_spi *ts)
{
	ts->diagonal_ghost_reject_p->state = 0;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_diagonal_ghost_init(struct shtps_rmi_spi *ts)
{
	ts->diagonal_ghost_reject_p = kzalloc(sizeof(struct shtps_filter_diagonal_ghost_reject_info), GFP_KERNEL);
	if(ts->diagonal_ghost_reject_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	ts->diagonal_ghost_reject_p->state = 0;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_diagonal_ghost_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->diagonal_ghost_reject_p)	kfree(ts->diagonal_ghost_reject_p);
	ts->diagonal_ghost_reject_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_DIAGONAL_GHOST_CHECK_ENABLE */
