/* drivers/sharp/shtps/sy3000/shtps_filter_topedge_fail_touch_check.c
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
#if defined(SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
struct shtps_filter_topedge_fail_info{
	u32	padding;	//padding
	u8	topedge_fail_touch_inhibit_id;
	u8	pad[3];		//padding
};

/* -------------------------------------------------------------------------- */
void shtps_filter_topedge_fail_touch_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	if(SHTPS_TOP_EDGE_FAIL_TOUCH_REJECT_ENABLE){
		int i;
		int fingerMax = shtps_get_fingermax(ts);

		for(i = 0; i < fingerMax; i++){
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				if( (ts->topedge_fail_p->topedge_fail_touch_inhibit_id & (1 << i)) != 0 ){
					ts->topedge_fail_p->topedge_fail_touch_inhibit_id &= ~(1 << i);
				}
			}

			if( (ts->topedge_fail_p->topedge_fail_touch_inhibit_id & (1 << i)) == 0 ){
				if( (info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER) &&
					(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) )
				{
					if(ts->fw_report_info.fingers[i].y <= SHTPS_TOP_EDGE_FAIL_TOUCH_REJECT_AREA){
						if( ((ts->fw_report_info.fingers[i].wx >= SHTPS_TOP_EDGE_FAIL_TOUCH_REJECT_WX_THRESH_MAX)) || 
							(((ts->fw_report_info.fingers[i].wx < SHTPS_TOP_EDGE_FAIL_TOUCH_REJECT_WX_THRESH_MAX)&&(ts->fw_report_info.fingers[i].wx >= SHTPS_TOP_EDGE_FAIL_TOUCH_REJECT_WX_THRESH_MIN))&&
							(ts->fw_report_info.fingers[i].z >= SHTPS_TOP_EDGE_FAIL_TOUCH_REJECT_Z_THRESH)) )
						{
							ts->topedge_fail_p->topedge_fail_touch_inhibit_id |= (1 << i);
						}
					}
				}
			}else if(ts->fw_report_info.fingers[i].y > SHTPS_TOP_EDGE_FAIL_TOUCH_REJECT_AREA){
				ts->topedge_fail_p->topedge_fail_touch_inhibit_id &= ~(1 << i);
			}
		}

		for(i = 0; i < fingerMax; i++){
			if( (ts->topedge_fail_p->topedge_fail_touch_inhibit_id & (1 << i)) != 0 ){
				info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
				SHTPS_LOG_DBG_TOP_EDGE_FAIL_TOUCH_PRINT("[td][%d] is inhibited\n", i);
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_topedge_fail_touch_sleep(struct shtps_rmi_spi *ts)
{
	ts->topedge_fail_p->topedge_fail_touch_inhibit_id = 0;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_topedge_fail_touch_init(struct shtps_rmi_spi *ts)
{
	ts->topedge_fail_p = kzalloc(sizeof(struct shtps_filter_topedge_fail_info), GFP_KERNEL);
	if(ts->topedge_fail_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// ts->topedge_fail_p->topedge_fail_touch_inhibit_id = 0;	// no need
}

/* -------------------------------------------------------------------------- */
void shtps_filter_topedge_fail_touch_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->topedge_fail_p)	kfree(ts->topedge_fail_p);
	ts->topedge_fail_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE */
