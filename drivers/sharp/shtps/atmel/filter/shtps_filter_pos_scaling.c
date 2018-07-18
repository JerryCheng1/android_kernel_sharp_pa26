/* drivers/sharp/shtps/atmel/filter/shtps_filter_pos_scaling.c
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
#include <linux/slab.h>

#include <sharp/shtps_dev.h>

#include "shtps_mxt.h"
#include "shtps_param_extern.h"
#include "shtps_log.h"

#include "shtps_filter.h"

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_POS_SCALING_ENABLE )
void shtps_filter_pos_scaling(struct shtps_mxt *ts)
{
	int		i;
	int		fingerMax = shtps_get_fingermax(ts);
	struct	fingers *report_fingers = ts->input_event_info.report.cur;

	for(i = 0;i < fingerMax;i++){
		if(report_fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			report_fingers[i].x = report_fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
			report_fingers[i].y = report_fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
		}
	}
}

#endif /* SHTPS_POS_SCALING_ENABLE */
