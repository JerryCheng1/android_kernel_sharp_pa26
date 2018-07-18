/* drivers/sharp/shtps/sy3000/shtps_filter_dynamic_reset_check.c
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
#include "shtps_fwctl.h"
#include "shtps_rmi_devctl.h"
/* -------------------------------------------------------------------------- */
#if defined(SHTPS_DYNAMIC_RESET_CONTROL_ENABLE)
struct shtps_filter_dynamic_reset_info{	
	u8	coaxial_pen_touch_cnt;
	u8	coaxial_pen_touch_cntup_enable;
	u8	coaxial_pen_reset_flg;
};

/* -------------------------------------------------------------------------- */
static int shtps_filter_dynamic_reset_ready_check(struct shtps_rmi_spi *ts)
{
	int need_reset = 0;

	SHTPS_LOG_FUNC_CALL();
	if(SHTPS_DYNAMIC_RESET_F54_COMMAND_ENABLE != 0)
	{
		u8 buf = 0xFF;

		/* M_READ_FUNC(ts, ts->map.fn54.commandBase, &buf, 1); */
		shtps_fwctl_get_AnalogCMD(ts, &buf);
		if( (buf & 0x06) != 0 ){
			need_reset = 1;
			SHTPS_LOG_ANALYSIS("[dynamic_reset]need reset by f54 status [0x%02X]\n", buf);
		}
	}

	if(SHTPS_DYNAMIC_RESET_COAXIAL_PEN_ENABLE != 0){
		if(ts->dynamic_reset_p->coaxial_pen_reset_flg != 0){
			need_reset = 1;
			SHTPS_LOG_DYNAMIC_RESET("need reset by coaxial pen\n");
		}
	}
	#if defined(SHTPS_DYNAMIC_RESET_ESD_ENABLE)
	{
		u8 buf[2] = {0xFF};
		u16 ver;

		/* M_READ_FUNC( ts,ts->map.fn12.data.num[15].addr,buf, 2); */
		shtps_fwctl_get_ObjectAttention(ts, buf);
		if( (buf[1] & 0x80) != 0 ){
			need_reset = 2;
		}else{
			ver = shtps_fwver(ts);
			if(ver==0x00 || ver==0xFF){
				need_reset = 2;
			}
		}
	}
	#endif /* SHTPS_DYNAMIC_RESET_ESD_ENABLE */

	return need_reset;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_dynamic_reset_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	if(SHTPS_DYNAMIC_RESET_COAXIAL_PEN_ENABLE != 0)
	{
		int i;
		int fingerMax = shtps_get_fingermax(ts);
		u8 numOfPen = 0;

		for(i = 0; i < fingerMax; i++){
			if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN){
				numOfPen++;
			}
		}

		if( (numOfPen > 0) && (ts->dynamic_reset_p->coaxial_pen_touch_cntup_enable != 0) ){
			if(ts->dynamic_reset_p->coaxial_pen_touch_cnt < SHTPS_DYNAMIC_RESET_COAXIAL_PEN_TOUCH_COUNT){
				ts->dynamic_reset_p->coaxial_pen_touch_cnt++;
				SHTPS_LOG_DYNAMIC_RESET("pen detect count up <%d>\n", ts->dynamic_reset_p->coaxial_pen_touch_cnt);
			}
			ts->dynamic_reset_p->coaxial_pen_touch_cntup_enable = 0;
		}

		if(ts->dynamic_reset_p->coaxial_pen_touch_cnt >= SHTPS_DYNAMIC_RESET_COAXIAL_PEN_TOUCH_COUNT){
			if(ts->dynamic_reset_p->coaxial_pen_reset_flg == 0){
				SHTPS_LOG_DYNAMIC_RESET("ready by coaxial pen\n");
				ts->dynamic_reset_p->coaxial_pen_reset_flg = 1;
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_dynamic_reset_sleep(struct shtps_rmi_spi *ts)
{
	if(SHTPS_DYNAMIC_RESET_COAXIAL_PEN_ENABLE == 0){
		memset(&ts->dynamic_reset_p, 0, sizeof(struct shtps_filter_dynamic_reset_info));
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_dynamic_reset_sleep_wakeup(struct shtps_rmi_spi *ts)
{
	if(SHTPS_DYNAMIC_RESET_COAXIAL_PEN_ENABLE != 0){
		ts->dynamic_reset_p->coaxial_pen_touch_cntup_enable = 1;
	}
}
/* -------------------------------------------------------------------------- */
void shtps_filter_dynamic_reset_init(struct shtps_rmi_spi *ts)
{
	ts->dynamic_reset_p = kzalloc(sizeof(struct shtps_filter_dynamic_reset_info), GFP_KERNEL);
	if(ts->dynamic_reset_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(&ts->dynamic_reset_p, 0, sizeof(struct shtps_filter_dynamic_reset_info));	// no need
}

/* -------------------------------------------------------------------------- */
void shtps_filter_dynamic_reset_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->dynamic_reset_p)	kfree(ts->dynamic_reset_p);
	ts->dynamic_reset_p = NULL;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_dynamic_reset_sleep_process(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_DYNAMIC_RESET_ESD_ENABLE)
		extern struct device *	shtpsif_device;
	#endif
	int rc = 0;

	shtps_mutex_lock_ctrl();
	rc = shtps_filter_dynamic_reset_ready_check(ts);
	shtps_mutex_unlock_ctrl();

	#if defined(SHTPS_DYNAMIC_RESET_ESD_ENABLE)
	if(rc == 2){
		if((shtps_system_get_hw_revision() == SHTPS_GET_HW_VERSION_RET_PP_2)||(shtps_system_get_hw_revision() == SHTPS_GET_HW_VERSION_RET_MP)){
			DBG_PRINTK("[ESD_regulator_reset] start\n");
			shtps_lvs1_regulator_get(shtpsif_device);
			shtps_lvs1_regulator_reset(ts->rst_pin);
			shtps_lvs1_regulator_put();
			shtps_shutdown(ts);
			shtps_start(ts);
			shtps_wait_startup(ts);
			DBG_PRINTK("[ESD_regulator_reset] end\n");
		}
	}else if(rc == 1){
	#else
	if(rc != 0){
	#endif /* SHTPS_DYNAMIC_RESET_ESD_ENABLE */
		SHTPS_LOG_ANALYSIS("[dynamic_reset] start\n");
		shtps_shutdown(ts);
		shtps_start(ts);
		shtps_wait_startup(ts);
		SHTPS_LOG_ANALYSIS("[dynamic_reset] end\n");

		shtps_mutex_lock_ctrl();
		memset(&ts->dynamic_reset_p, 0, sizeof(struct shtps_filter_dynamic_reset_info));
		shtps_mutex_unlock_ctrl();
	}
}
#endif /* SHTPS_DYNAMIC_RESET_CONTROL_ENABLE */
