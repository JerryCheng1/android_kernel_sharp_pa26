/* drivers/sharp/shtps/sy3000/shtps_filter_multitouch_pen_ghost_rejection_check.c
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
#if defined(SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE)
struct multitouch_pen_ghost_rejection_info{
	u32	padding;
	u8	multitouch_pen_ghost_rejection_finger_flg;
	u8	pad[3];
};

/* -------------------------------------------------------------------------- */
void shtps_filter_multitouch_pen_ghost_rejection_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	u8 numOfPenFingers = 0;

	if (SHTPS_MULTITOUCH_PEN_GHOST_REJECTION == 0) {
		return;
	}

	for (i = 0; i < fingerMax; i++) {
		if ( (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) ||
		     (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN) ) {
			numOfPenFingers++;
		}
	}

	for (i = 0; i < fingerMax; i++) {
		if ((ts->multitouch_pen_ghost_rejection_p->multitouch_pen_ghost_rejection_finger_flg & (1 << i)) != 0) {
			if ( (info->fingers[i].z <= SHTPS_MULTITOUCH_PEN_GHOST_Z_THRESHOLD_MIN) ||
			     (info->fingers[i].z >= SHTPS_MULTITOUCH_PEN_GHOST_Z_THRESHOLD_MAX) )
			{
				ts->multitouch_pen_ghost_rejection_p->multitouch_pen_ghost_rejection_finger_flg &= ~(1 << i);
			}
		}
		else if (numOfPenFingers >= 3) {
			if (info->fingers[i].state == SHTPS_TOUCH_STATE_PEN) {
				if ( (info->fingers[i].z > SHTPS_MULTITOUCH_PEN_GHOST_Z_THRESHOLD_MIN) &&
				     (info->fingers[i].z < SHTPS_MULTITOUCH_PEN_GHOST_Z_THRESHOLD_MAX) )
				{
					ts->multitouch_pen_ghost_rejection_p->multitouch_pen_ghost_rejection_finger_flg |= (1 << i);
				}
			}
		}
	}

	for (i = 0; i < fingerMax; i++) {
		if ((ts->multitouch_pen_ghost_rejection_p->multitouch_pen_ghost_rejection_finger_flg & (1 << i)) != 0) {
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;

			SHTPS_LOG_DBG_PRINT("[PenGhostReject] ignore pen ghost (id=%d, x=%d, y=%d, z=%d)\n",
					i,
					info->fingers[i].x,
					info->fingers[i].y,
					info->fingers[i].z);
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_multitouch_pen_ghost_rejection_sleep(struct shtps_rmi_spi *ts)
{
	ts->multitouch_pen_ghost_rejection_p->multitouch_pen_ghost_rejection_finger_flg = 0;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_multitouch_pen_ghost_rejection_init(struct shtps_rmi_spi *ts)
{
	ts->multitouch_pen_ghost_rejection_p = kzalloc(sizeof(struct multitouch_pen_ghost_rejection_info), GFP_KERNEL);
	if(ts->multitouch_pen_ghost_rejection_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// ts->multitouch_pen_ghost_rejection_p->multitouch_pen_ghost_rejection_finger_flg = 0;	// no need
}

/* -------------------------------------------------------------------------- */
void shtps_filter_multitouch_pen_ghost_rejection_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->multitouch_pen_ghost_rejection_p)	kfree(ts->multitouch_pen_ghost_rejection_p);
	ts->multitouch_pen_ghost_rejection_p = NULL;
}

#endif /* SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE */
