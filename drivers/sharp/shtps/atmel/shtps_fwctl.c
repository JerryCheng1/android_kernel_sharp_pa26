/* drivers/sharp/shtps/atmel/shtps_fwctl.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <sharp/shtps_dev.h>

#include "shtps_mxt.h"
#include "shtps_fwctl.h"
#include "shtps_log.h"
#include "shtps_param_extern.h"

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_LOG_ERROR_ENABLE )
	#define SHTPS_LOG_FWCTL_FUNC_CALL()		// SHTPS_LOG_FUNC_CALL()
#else
	#define SHTPS_LOG_FWCTL_FUNC_CALL()
#endif
/* -------------------------------------------------------------------------- */
int shtps_fwctl_init(struct shtps_mxt *ts_p)
{
	ts_p->fwctl_p = kzalloc(sizeof(struct shtps_fwctl_info), GFP_KERNEL);
	if(ts_p->fwctl_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return -ENOMEM;
	}

	return 0;
}

/* -------------------------------------------------------------------------- */
void shtps_fwctl_deinit(struct shtps_mxt *ts_p)
{
	if(ts_p->fwctl_p)	kfree(ts_p->fwctl_p);
	ts_p->fwctl_p = NULL;
}

/* -------------------------------------------------------------------------- */
void shtps_fwctl_set_dev_state(struct shtps_mxt *ts_p, u8 state)
{
	struct shtps_fwctl_info *fc_p = ts_p->fwctl_p;

	SHTPS_LOG_FWCTL_FUNC_CALL();
	if(fc_p->dev_state != state){
		SHTPS_LOG_ANALYSIS("[dev_state] set (%s -> %s)\n",
								(fc_p->dev_state == SHTPS_DEV_STATE_SLEEP) ? "sleep" :
								(fc_p->dev_state == SHTPS_DEV_STATE_DOZE) ? "doze" :
								(fc_p->dev_state == SHTPS_DEV_STATE_ACTIVE) ? "active" :
								(fc_p->dev_state == SHTPS_DEV_STATE_LPWG) ? "lpwg" :
								(fc_p->dev_state == SHTPS_DEV_STATE_LOADER) ? "loader" : "unknown",
								(state == SHTPS_DEV_STATE_SLEEP) ? "sleep" :
								(state == SHTPS_DEV_STATE_DOZE) ? "doze" :
								(state == SHTPS_DEV_STATE_ACTIVE) ? "active" :
								(state == SHTPS_DEV_STATE_LPWG) ? "lpwg" :
								(state == SHTPS_DEV_STATE_LOADER) ? "loader" : "unknown" );
	}

	fc_p->dev_state = state;
}

u8 shtps_fwctl_get_dev_state(struct shtps_mxt *ts_p)
{
	struct shtps_fwctl_info *fc_p = ts_p->fwctl_p;
	SHTPS_LOG_FWCTL_FUNC_CALL();
	return fc_p->dev_state;
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_maxXPosition(struct shtps_mxt *ts_p)
{
	return CONFIG_SHTPS_PANEL_SIZE_X;
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_maxYPosition(struct shtps_mxt *ts_p)
{
	return CONFIG_SHTPS_PANEL_SIZE_Y;
}
