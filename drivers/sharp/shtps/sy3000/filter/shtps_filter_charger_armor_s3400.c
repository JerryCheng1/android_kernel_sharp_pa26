/* drivers/sharp/shtps/sy3000/shtps_filter_charger_armor_s3400.c
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
#include "shtps_fwctl.h"
#include "shtps_filter.h"
/* -------------------------------------------------------------------------- */
#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
struct shtps_filter_charger_armor_info{
	int	padding;
	u8	charger_armor_state;
	u8	pad[3];
};

/* -------------------------------------------------------------------------- */
static int shtps_is_uimode(int state)
{
	switch( state /* ts->state_mgr.state */ ){
		case SHTPS_STATE_ACTIVE:
		case SHTPS_STATE_FACETOUCH:
		case SHTPS_STATE_SLEEP:
		case SHTPS_STATE_SLEEP_FACETOUCH:
			return 1;

		default:
			return 0;
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_set_charger_armor_initparam(struct shtps_rmi_spi *ts)
{
	if(ts->charger_armor_p->charger_armor_state == 0){
		shtps_fwctl_set_jitter_filter_strength(ts, SHTPS_CHARGER_ARMOR_STRENGTH_DEFAULT);
	}else{
		shtps_fwctl_set_jitter_filter_strength(ts, SHTPS_CHARGER_ARMOR_STRENGTH);
	}
}

/* -------------------------------------------------------------------------- */
int shtps_filter_set_charger_armor(struct shtps_rmi_spi *ts, int state, int on)
{
	if(on != 0){
		if(ts->charger_armor_p->charger_armor_state == 0){
			if(shtps_is_uimode(state)){
				shtps_fwctl_set_jitter_filter_strength(ts, SHTPS_CHARGER_ARMOR_STRENGTH);
			}
			ts->charger_armor_p->charger_armor_state = 1;
		}
	}else{
		if(ts->charger_armor_p->charger_armor_state != 0){
			if(shtps_is_uimode(state)){
				shtps_fwctl_set_jitter_filter_strength(ts, SHTPS_CHARGER_ARMOR_STRENGTH_DEFAULT);
			}
			ts->charger_armor_p->charger_armor_state = 0;
		}
	}

	return 0;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_charger_armor_init(struct shtps_rmi_spi *ts)
{
	ts->charger_armor_p = kzalloc(sizeof(struct shtps_filter_charger_armor_info), GFP_KERNEL);
	if(ts->charger_armor_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// ts->charger_armor_p->charger_armor_state = 0;	// no need
}

/* -------------------------------------------------------------------------- */
void shtps_filter_charger_armor_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->charger_armor_p)	kfree(ts->charger_armor_p);
	ts->charger_armor_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_CHARGER_ARMOR_ENABLE */
