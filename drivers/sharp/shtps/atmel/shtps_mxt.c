/* drivers/sharp/shtps/atmel/shtps_mxt.c
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

#include <linux/module.h>
#include <linux/slab.h>

#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/wakelock.h>
#include <linux/input/mt.h>

#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/syscalls.h>

#include <sharp/sh_smem.h>
#include <sharp/sh_boot_manager.h>
#include <sharp/shtps_dev_mxt.h>

#include "shtps_mxt.h"
#include "shtps_cfg.h"
#include "shtps_param.h"

#include "shtps_mxt_sub.h"
#include "shtps_mxt_debug.h"
#include "shtps_log.h"

#if defined(SHTPS_MOTION_PEDO_STOP_REQ_ENABLE)
	#include <sharp/shub_driver.h>
#endif /* SHTPS_MOTION_PEDO_STOP_REQ_ENABLE */


/* -----------------------------------------------------------------------------------
 */
static struct shtps_mxt shtps_mxt_org = {0};
struct shtps_mxt *gShtps_mxt = &shtps_mxt_org;

#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
	u8					gLogOutputEnable = 0;
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */

static DEFINE_MUTEX(shtps_ctrl_lock);
/* -----------------------------------------------------------------------------------
 */
#define	TPS_MUTEX_LOG_LOCK(VALUE)		SHTPS_LOG_DBG_PRINT("mutex_lock:   -> (%s)\n",VALUE)
#define	TPS_MUTEX_LOG_UNLOCK(VALUE)		SHTPS_LOG_DBG_PRINT("mutex_unlock: <- (%s)\n",VALUE)
/* -----------------------------------------------------------------------------------
 */
static void shtps_calc_notify(struct shtps_mxt *ts, u8 *event);
static void shtps_report_touch_on(struct input_dev *input_dev, int id, int x, int y, int w, int z, int orientation);
static void shtps_report_touch_off(struct input_dev *input_dev, int id, int x, int y, int w, int z, int orientation);
static void shtps_diag_init(void);
static void shtps_diag_input_event_notify(void);

#if defined(SHTPS_LPWG_MODE_ENABLE)
	static void shtps_lpwg_notify_interval_delayed_work_function(struct work_struct *work);
	static void shtps_lpwg_notify_interval_stop(struct shtps_mxt *ts);
	static void shtps_lpwg_notify_interval_start(struct shtps_mxt *ts);
	static void shtps_lpwg_wakelock_init(struct shtps_mxt *ts);
	static void shtps_lpwg_wakelock_destroy(struct shtps_mxt *ts);
//	static void shtps_lpwg_wakelock(struct shtps_mxt *ts, int on);
#endif /* SHTPS_LPWG_MODE_ENABLE */

#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
	static int shtps_init_inputdev_key(struct shtps_mxt *ts, struct device *ctrl_dev_p);
	static void shtps_deinit_inputdev_key(struct shtps_mxt *ts);
#endif /* SHTPS_PHYSICAL_KEY_ENABLE || SHTPS_LPWG_MODE_ENABLE */

/* -----------------------------------------------------------------------------------
 */
void shtps_mutex_lock_ctrl(void)
{
	TPS_MUTEX_LOG_LOCK("shtps_ctrl_lock");
	mutex_lock(&shtps_ctrl_lock);
}
void shtps_mutex_unlock_ctrl(void)
{
	TPS_MUTEX_LOG_UNLOCK("shtps_ctrl_lock");
	mutex_unlock(&shtps_ctrl_lock);
}

#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
static int shtps_init_inputdev_key(struct shtps_mxt *ts, struct device *ctrl_dev_p)
{
	SHTPS_LOG_FUNC_CALL();
	ts->input_key = input_allocate_device();
	if(!ts->input_key){
		return -ENOMEM;
	}

	ts->input_key->name 		= "atmel_mxt_key";
	ts->input_key->phys         = ts->phys;
	ts->input_key->id.vendor	= 0x0000;
	ts->input_key->id.product	= 0x0000;
	ts->input_key->id.version	= 0x0000;
	ts->input_key->dev.parent	= ctrl_dev_p;

	__set_bit(EV_KEY, ts->input_key->evbit);

	input_set_drvdata(ts->input_key, ts);

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		ts->input_key->keycode = ts->keycodes;
		ts->input_key->keycodemax = SHTPS_PHYSICAL_KEY_NUM;
		ts->input_key->keycodesize = sizeof(ts->keycodes);
		ts->keycodes[SHTPS_PHYSICAL_KEY_DOWN] = KEY_VOLUMEDOWN;
		ts->keycodes[SHTPS_PHYSICAL_KEY_UP] = KEY_VOLUMEUP;

		__set_bit(KEY_VOLUMEDOWN, ts->input_key->keybit);
		__set_bit(KEY_VOLUMEUP, ts->input_key->keybit);

		input_set_capability(ts->input_key, EV_MSC, MSC_SCAN);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		__set_bit(KEY_SWEEPON, ts->input_key->keybit);
	#endif /*  SHTPS_LPWG_MODE_ENABLE */
	
	__clear_bit(KEY_RESERVED, ts->input_key->keybit);

	if(input_register_device(ts->input_key)){
		input_free_device(ts->input_key);
		return -EFAULT;
	}
	
	return 0;
}

static void shtps_deinit_inputdev_key(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();
	if(ts && ts->input_key){
		input_free_device(ts->input_key);
	}
}
#endif /* SHTPS_PHYSICAL_KEY_ENABLE || SHTPS_LPWG_MODE_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
static u8 shtps_get_lpmode_state(struct shtps_mxt *ts)
{
	#if defined( SHTPS_HOVER_DETECT_ENABLE )
		u8 req = (ts->lpmode_req_state | ts->lpmode_continuous_req_state);

		if((req & SHTPS_LPMODE_REQ_HOVER_OFF) != 0){
			if((req & SHTPS_LPMODE_REQ_LCD_BRIGHT) == 0){
				req &= ~SHTPS_LPMODE_REQ_HOVER_OFF;
			}
		}else{
			if((req & SHTPS_LPMODE_REQ_LCD_BRIGHT) != 0){
				req &= ~SHTPS_LPMODE_REQ_LCD_BRIGHT;
			}
		}

		return (req != SHTPS_LPMODE_REQ_NONE);
	#else
		return ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) != SHTPS_LPMODE_REQ_NONE);
	#endif /* SHTPS_HOVER_DETECT_ENABLE */
}

void shtps_set_lpmode(struct shtps_mxt *ts, int type, int req, int on)
{
	u8 cur_lpmode_state;
	u8 pre_lpmode_state = shtps_get_lpmode_state(ts);
	int changed = 0;

	SHTPS_LOG_FUNC_CALL();
	if(on){
		if(type == SHTPS_LPMODE_TYPE_NON_CONTINUOUS){
			if((ts->lpmode_req_state & req) == 0){
				ts->lpmode_req_state |= req;
				changed = 1;
				SHTPS_LOG_DBG_PRINT("[LPMODE]<ON> type = NON CONTINUOUS, req = %s\n",
										(req == SHTPS_LPMODE_REQ_COMMON)? "common" :
										(req == SHTPS_LPMODE_REQ_ECO)? "eco" :
										#if defined( SHTPS_HOVER_DETECT_ENABLE )
											(req == SHTPS_LPMODE_REQ_HOVER_OFF)? "hover_off" :
										#endif /* SHTPS_HOVER_DETECT_ENABLE */
										(req == SHTPS_LPMODE_REQ_LCD_BRIGHT)? "lcd_bright" : "unknown");
			}
		}else{
			if((ts->lpmode_continuous_req_state & req) == 0){
				ts->lpmode_continuous_req_state |= req;
				changed = 1;
				SHTPS_LOG_DBG_PRINT("[LPMODE]<ON> type = CONTINUOUS, req = %s\n",
										(req == SHTPS_LPMODE_REQ_COMMON)? "common" :
										(req == SHTPS_LPMODE_REQ_ECO)? "eco" :
										#if defined( SHTPS_HOVER_DETECT_ENABLE )
											(req == SHTPS_LPMODE_REQ_HOVER_OFF)? "hover_off" :
										#endif /* SHTPS_HOVER_DETECT_ENABLE */
										(req == SHTPS_LPMODE_REQ_LCD_BRIGHT)? "lcd_bright" : "unknown");
			}
		}
	}
	else{
		if(type == SHTPS_LPMODE_TYPE_NON_CONTINUOUS){
			if((ts->lpmode_req_state & req) != 0){
				ts->lpmode_req_state &= ~req;
				changed = 1;
				SHTPS_LOG_DBG_PRINT("[LPMODE]<OFF> type = NON CONTINUOUS, req = %s\n",
										(req == SHTPS_LPMODE_REQ_COMMON)? "common" :
										(req == SHTPS_LPMODE_REQ_ECO)? "eco" :
										#if defined( SHTPS_HOVER_DETECT_ENABLE )
											(req == SHTPS_LPMODE_REQ_HOVER_OFF)? "hover_off" :
										#endif /* SHTPS_HOVER_DETECT_ENABLE */
										(req == SHTPS_LPMODE_REQ_LCD_BRIGHT)? "lcd_bright" : "unknown");
			}
		}else{
			if((ts->lpmode_continuous_req_state & req) != 0){
				ts->lpmode_continuous_req_state &= ~req;
				changed = 1;
				SHTPS_LOG_DBG_PRINT("[LPMODE]<OFF> type = CONTINUOUS, req = %s\n",
										(req == SHTPS_LPMODE_REQ_COMMON)? "common" :
										(req == SHTPS_LPMODE_REQ_ECO)? "eco" :
										#if defined( SHTPS_HOVER_DETECT_ENABLE )
											(req == SHTPS_LPMODE_REQ_HOVER_OFF)? "hover_off" :
										#endif /* SHTPS_HOVER_DETECT_ENABLE */
										(req == SHTPS_LPMODE_REQ_LCD_BRIGHT)? "lcd_bright" : "unknown");
			}
		}
	}

	if(changed){
		if((cur_lpmode_state = shtps_get_lpmode_state(ts)) !=  pre_lpmode_state){
			if(cur_lpmode_state){
				mxt_set_lpmode(ts->dev, 1);
			}else{
				mxt_set_lpmode(ts->dev, 0);
			}
		}
	}

	SHTPS_LOG_DBG_PRINT("[LPMODE]lpmode request recieved. req_state = 0x%02x, continuous_req_state = 0x%02x\n",
							ts->lpmode_req_state, ts->lpmode_continuous_req_state);
}
#endif /* SHTPS_LOW_POWER_MODE_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_LPWG_MODE_ENABLE )
static void shtps_lpwg_notify_interval_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_lpwg_ctrl *lpwg_p = container_of(dw, struct shtps_lpwg_ctrl, notify_interval_delayed_work);

	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_ctrl();
	lpwg_p->notify_enable = 1;
	shtps_mutex_unlock_ctrl();

	SHTPS_LOG_DBG_PRINT("[LPWG] notify interval end\n");
}

static void shtps_lpwg_notify_interval_stop(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->lpwg.notify_interval_delayed_work);
}

static void shtps_lpwg_notify_interval_start(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_lpwg_notify_interval_stop(ts);
	schedule_delayed_work(&ts->lpwg.notify_interval_delayed_work, msecs_to_jiffies(SHTPS_LPWG_MIN_NOTIFY_INTERVAL));

	SHTPS_LOG_DBG_PRINT("[LPWG] notify interval start\n");
}

static void shtps_lpwg_wakelock_init(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();
	memset(&ts->lpwg, 0, sizeof(ts->lpwg));

	ts->lpwg.notify_enable = 1;
	ts->lpwg.lpwg_switch = 0;

	#if defined(SHTPS_LPWG_GRIP_SUPPORT_ENABLE)
		ts->lpwg.lpwg_state  = SHTPS_LPWG_STATE_GRIP_ONLY;
	#else
		ts->lpwg.lpwg_state  = SHTPS_LPWG_STATE_OFF;
	#endif /* SHTPS_LPWG_GRIP_SUPPORT_ENABLE */

    wake_lock_init(&ts->lpwg.wake_lock, WAKE_LOCK_SUSPEND, "shtps_lpwg_wake_lock");
	pm_qos_add_request(&ts->lpwg.pm_qos_lock_idle, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	ts->lpwg.pm_qos_idle_value = SHTPS_LPWG_QOS_LATENCY_DEF_VALUE;
	INIT_DELAYED_WORK(&ts->lpwg.notify_interval_delayed_work, shtps_lpwg_notify_interval_delayed_work_function);
}

static void shtps_lpwg_wakelock_destroy(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();
   wake_lock_destroy(&ts->lpwg.wake_lock);
   pm_qos_remove_request(&ts->lpwg.pm_qos_lock_idle);
}

static void shtps_lpwg_wakelock(struct shtps_mxt *ts, int on)
{
	SHTPS_LOG_FUNC_CALL();
	if(on){
	    wake_lock(&ts->lpwg.wake_lock);
	    pm_qos_update_request(&ts->lpwg.pm_qos_lock_idle, ts->lpwg.pm_qos_idle_value);
		SHTPS_LOG_DBG_PRINT("lpwg wake lock ON\n");
	}else{
		wake_unlock(&ts->lpwg.wake_lock);
	    pm_qos_update_request(&ts->lpwg.pm_qos_lock_idle, PM_QOS_DEFAULT_VALUE);
		SHTPS_LOG_DBG_PRINT("lpwg wake lock OFF\n");
	}
}

void shtps_set_lpwg_mode_on(struct shtps_mxt *ts)
{
	struct shtps_lpwg_info info = {0};
	char SYMDATA[] = {SHTPS_LPWG_SYMDATA};

	if(ts->lpwg.lpwg_switch == 1){
		return;
	}

	SHTPS_LOG_FUNC_CALL();

	shtps_lpwg_notify_interval_stop(ts);
	ts->lpwg.notify_enable = 1;
	ts->lpwg.lpwg_switch = 1;

//	shtps_fwctl_set_lpwg_mode_on(ts);
	info.idleacqint = SHTPS_LPWG_IDLEACQINT_VALUE;
	info.actvacqint = SHTPS_LPWG_ACTVACQINT_VALUE;
	info.symdata = SYMDATA;
	info.symdata_size = sizeof(SYMDATA);
	info.grip_sup_x_mm = SHTPS_LPWG_GRIP_SUPPRESSION_X_MM;
	info.grip_sup_y_mm = SHTPS_LPWG_GRIP_SUPPRESSION_Y_MM;
	info.mov_mm = SHTPS_LPWG_MOV_MM;
	info.sym_time_max_ms = SHTPS_LPWG_SYM_TIMEMAX_MS;

	mxt_set_lpwg(ts->dev, 1, &info);

	SHTPS_LOG_DBG_PRINT("LPWG mode ON\n");
}

void shtps_set_lpwg_mode_off(struct shtps_mxt *ts)
{
	if(ts->lpwg.lpwg_switch == 0){
		return;
	}
	SHTPS_LOG_FUNC_CALL();

	ts->lpwg.lpwg_switch = 0;

//	shtps_fwctl_set_lpwg_mode_off(ts);
	mxt_set_lpwg(ts->dev, 0, NULL);

	SHTPS_LOG_DBG_PRINT("LPWG mode OFF\n");
}

int shtps_is_lpwg_active(struct shtps_mxt *ts)
{
	int ret = 0;
	
	SHTPS_LOG_FUNC_CALL();
	if(ts->lpwg.lpwg_state == SHTPS_LPWG_STATE_OFF){
		ret = 0;
	}else if(ts->lpwg.lpwg_state == SHTPS_LPWG_STATE_ON){
		ret = 1;
	}
	#if defined(SHTPS_LPWG_GRIP_SUPPORT_ENABLE)
		else if(ts->lpwg.lpwg_state == SHTPS_LPWG_STATE_GRIP_ONLY){
			if(ts->lpwg.grip_state){
				ret = 1;
			}else{
				ret = 0;
			}
		}
	#endif /* SHTPS_LPWG_GRIP_SUPPORT_ENABLE */

	return ret;
}

static void shtps_notify_wakeup_event(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_DBG_PRINT("wakeup touch blocked until rezero\n");

//	ts->lpwg.block_touchevent = 1;
	ts->lpwg.notify_enable = 0;

	input_report_key(ts->input_key, KEY_SWEEPON, 1);
	input_sync(ts->input_key);

	input_report_key(ts->input_key, KEY_SWEEPON, 0);
	input_sync(ts->input_key);
	
	shtps_lpwg_notify_interval_start(ts);
}
#endif /* SHTPS_LPWG_MODE_ENABLE */

/* -----------------------------------------------------------------------------------
 */
void shtps_sleep(struct shtps_mxt *ts, int sleep)
{
	struct device *dev = ts->dev;

	SHTPS_LOG_DBG_PRINT("%s: sleep=%d\n", __func__, sleep);

	if (dev == NULL){
		return;
	}

	if(ts->state_mgr.state == SHTPS_STATE_BOOTLOADER){
		SHTPS_LOG_DBG_PRINT("%s: do nothing because current state is bootloader mode.\n", __func__);
	}
	else{
		mxt_set_sleep(dev, sleep);
	}
}
/* -----------------------------------------------------------------------------------
 */
int shtps_get_fingermax(struct shtps_mxt *ts)
{
	return SHTPS_FINGER_MAX;
}

int shtps_get_diff(unsigned short pos1, unsigned short pos2)
{
	int diff;
	diff = pos1 - pos2;
	return (diff >= 0) ? diff : -diff;
}

void shtps_set_eventtype(u8 *event, u8 type)
{
	*event = type;
}

static void shtps_set_touch_info(struct shtps_mxt *ts)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	struct fingers *report_pre_fingers = ts->input_event_info.report.pre;
	struct fingers *report_fingers = ts->input_event_info.report.cur;

	memcpy(ts->input_event_info.report.cur, 
		ts->input_event_info.fw.cur, sizeof(ts->input_event_info.report.cur));

	for(i = 0;i < fingerMax;i++){
		if(report_fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			SHTPS_LOG_EVENT(
				DBG_PRINTK("[%s]Touch Info[%d] x=%d, y=%d, w=%d, z=%d\n",
						report_fingers[i].state == SHTPS_TOUCH_STATE_FINGER ? "Finger" :
						report_fingers[i].state == SHTPS_TOUCH_STATE_PEN ? "Pen" : "Other" ,
						i,
						report_fingers[i].x,
						report_fingers[i].y,
						report_fingers[i].w,
						report_fingers[i].z
				);
			);
		}else if(report_pre_fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			SHTPS_LOG_EVENT(
				DBG_PRINTK("[%s]Touch Info[%d] x=%d, y=%d, w=%d, z=%d\n",
						"NoTouch",
						i,
						report_fingers[i].x,
						report_fingers[i].y,
						report_fingers[i].w,
						report_fingers[i].z
				);
			);
		}
	}

	return;
}

static void shtps_calc_notify(struct shtps_mxt *ts, u8 *event)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int numOfFingers = 0;
	int numOfPreFingers = 0;
	int diff_x;
	int diff_y;
	struct fingers *report_pre_fingers = ts->input_event_info.report.pre;
	struct fingers *report_fingers = ts->input_event_info.report.cur;

	SHTPS_LOG_FUNC_CALL();

	shtps_set_eventtype(event, 0xff);
	shtps_set_touch_info(ts);

	shtps_filter_main(ts);

	for (i = 0; i < fingerMax; i++) {
		if(report_fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			numOfFingers++;
		}
	}
	for (i = 0; i < fingerMax; i++) {
		if(report_pre_fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			numOfPreFingers++;
		}
	}

	/* set event type */
	for(i = 0;i < fingerMax;i++){
		if(report_fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			if(report_pre_fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				diff_x = shtps_get_diff(report_fingers[i].x, report_pre_fingers[i].x);
				diff_y = shtps_get_diff(report_fingers[i].y, report_pre_fingers[i].y);

				if((diff_x > 0) || (diff_y > 0)){
					shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
				}
			}
		}

		if(report_fingers[i].state != report_pre_fingers[i].state){
			shtps_set_eventtype(event, SHTPS_EVENT_MTDU);
		}
	}
	if(numOfFingers > 0){
		if(numOfPreFingers == 0){
			shtps_set_eventtype(event, SHTPS_EVENT_TD);
			#if defined(SHTPS_MOTION_PEDO_STOP_REQ_ENABLE)
				shub_api_stop_pedometer_func(SHUB_STOP_PED_TYPE_TPS);
			#endif /* SHTPS_MOTION_PEDO_STOP_REQ_ENABLE */
		}
	}else{
		if(numOfPreFingers != 0){
			shtps_set_eventtype(event, SHTPS_EVENT_TU);
			#if defined(SHTPS_MOTION_PEDO_STOP_REQ_ENABLE)
				shub_api_restart_pedometer_func(SHUB_STOP_PED_TYPE_TPS);
			#endif /* SHTPS_MOTION_PEDO_STOP_REQ_ENABLE */
		}
	}
}

static void shtps_report_touch_on(struct input_dev *input_dev, int id, int x, int y, int w, int z, int orientation)
{
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);

	if(z != TPSDEV_INPUT_ITEM_INVALID){
		input_report_abs(input_dev, ABS_MT_PRESSURE, z);
	}
	
	if(w != TPSDEV_INPUT_ITEM_INVALID){
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, w);
	}

	if(orientation != TPSDEV_INPUT_ITEM_INVALID){
		input_report_abs(input_dev, ABS_MT_ORIENTATION, orientation);
	}
	SHTPS_LOG_EVENT(
		DBG_PRINTK("[finger]Notify event[%d] touch=100(%d), x=%d(%d), y=%d(%d) w=%d\n",
						id, z, x, x, y, y, w);
	);
}

static void shtps_report_touch_off(struct input_dev *input_dev, int id, int x, int y, int w, int z, int orientation)
{
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	SHTPS_LOG_EVENT(
		DBG_PRINTK("[finger]Notify event[%d] touch=0(%d), x=%d(%d), y=%d(%d) w=%d\n",
						id, z, x, x, y, y, w);
	);
}
/* -----------------------------------------------------------------------------------
 */
static void shtps_diag_init(void)
{
	struct shtps_mxt *ts = gShtps_mxt;
	ts->diag.input_event_detect = 0;
	ts->diag.poll_stop_request  = 0;
	init_waitqueue_head(&ts->diag.input_event_wait);
}

static void shtps_diag_input_event_notify(void)
{
	struct shtps_mxt *ts = gShtps_mxt;
	ts->diag.input_event_detect = 1;
	wake_up_interruptible(&ts->diag.input_event_wait);
}

void shtps_diag_input_event_poll_stop_request(void)
{
	struct shtps_mxt *ts = gShtps_mxt;
	ts->diag.poll_stop_request = 1;
	wake_up_interruptible(&ts->diag.input_event_wait);
}

int shtps_diag_input_event_poll(struct shtps_input_event_info **input_event_info_pp)
{
	struct shtps_mxt *ts = gShtps_mxt;
	int ret;

	ts->diag.poll_stop_request = 0;

	ret = wait_event_interruptible_timeout(ts->diag.input_event_wait, 
										   (ts->diag.input_event_detect == 1) ||
										   (ts->diag.poll_stop_request  == 1),
										   msecs_to_jiffies(SHTPS_DIAG_POLL_TIMEOUT));	

	if((ret != 0) && (ts->diag.poll_stop_request == 0)){
		ts->diag.input_event_detect = 0;
		*input_event_info_pp = &ts->input_event_info;
		ret = 0;
	}
	else{
		ret = -EFAULT;
	}

	ts->diag.poll_stop_request = 0;

	return ret;
}

/* -----------------------------------------------------------------------------------
 */
int shtps_get_logflag(void)
{
	#if defined( SHTPS_DEVELOP_MODE_ENABLE )
		return gLogOutputEnable;
	#else
		return 0;
	#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */
}

/* -----------------------------------------------------------------------------------
 */
int shtps_fwup_flag_check(void)
{
	#if defined( SHTPS_FACTORY_MODE_ENABLE )
		return 0;
	#else
		sharp_smem_common_type *smemdata = NULL;

		SHTPS_LOG_FUNC_CALL();
		smemdata = sh_smem_get_common_address();
		if(smemdata != NULL){
			SHTPS_LOG_DBG_PRINT("shtps_fwup_flag : %s\n", smemdata->shtps_fwup_flag == 0 ? "off" : "on");
			if(smemdata->shtps_fwup_flag == 0){
				return 0;
			}else{
				return 1;
			}
		}

		return -1;
	#endif /* #if defined( SHTPS_FACTORY_MODE_ENABLE ) */
}

int shtps_system_get_hw_revision(void)
{
	SHTPS_LOG_FUNC_CALL();
	return sh_boot_get_hw_revision();
}

int shtps_system_get_hw_type(void)
{
	unsigned char handset;
	int ret = 0;

	SHTPS_LOG_FUNC_CALL();
	handset = sh_boot_get_handset();

	if(handset == 0){
		ret = SHTPS_HW_TYPE_BOARD;
	}else{
		ret = SHTPS_HW_TYPE_HANDSET;
	}

	return ret;
}

static void shtps_mxt_state_active_enter(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();

	ts->state_mgr.state = SHTPS_STATE_ACTIVE;

	shtps_fwctl_set_dev_state(ts, SHTPS_DEV_STATE_ACTIVE);

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if (ts->lpwg.lpwg_switch){
			ts->lpwg.wakeup_time = jiffies;
//			shtps_set_lpwg_mode_off(ts);
//			shtps_read_touchevent(ts, SHTPS_STATE_IDLE);
		}else{
//			shtps_system_set_wakeup(ts);
		}
	#else
		shtps_system_set_wakeup(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */
}

static void shtps_mxt_state_sleep_enter(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();

	ts->state_mgr.state = SHTPS_STATE_SLEEP;

	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	{
		u8 pre_lpmode_state = shtps_get_lpmode_state(ts);
		
		ts->lpmode_req_state = SHTPS_LPMODE_REQ_NONE;
		if(pre_lpmode_state != shtps_get_lpmode_state(ts)){
			if(!shtps_get_lpmode_state(ts)){
				mxt_clr_lpmode_flag(ts->dev);
			}
		}
	}
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	shtps_fwctl_set_dev_state(ts, SHTPS_DEV_STATE_SLEEP);

	shtps_perflock_sleep(ts);	//SHTPS_CPU_CLOCK_CONTROL_ENABLE

	shtps_filter_sleep_enter(ts);
}

static void shtps_mxt_state_lpwg_enter(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();

	ts->state_mgr.state = SHTPS_STATE_SLEEP;

	shtps_fwctl_set_dev_state(ts, SHTPS_DEV_STATE_LPWG);

	shtps_perflock_sleep(ts);	//SHTPS_CPU_CLOCK_CONTROL_ENABLE
}

static void shtps_mxt_state_loader_enter(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();

	ts->state_mgr.state = SHTPS_STATE_BOOTLOADER;

	shtps_fwctl_set_dev_state(ts, SHTPS_DEV_STATE_LOADER);

	shtps_perflock_sleep(ts);	//SHTPS_CPU_CLOCK_CONTROL_ENABLE
}

void shtps_mxt_notify_devstate(int new_state)
{
	struct shtps_mxt *ts = gShtps_mxt;

	switch(new_state){
		case SHTPS_DEV_STATE_SLEEP:
			shtps_mxt_state_sleep_enter(ts);
			break;
		case SHTPS_DEV_STATE_ACTIVE:
			shtps_mxt_state_active_enter(ts);
			break;
		case SHTPS_DEV_STATE_LPWG:
			shtps_mxt_state_lpwg_enter(ts);
			break;
		case SHTPS_DEV_STATE_LOADER:
			shtps_mxt_state_loader_enter(ts);
			break;
		default:
			break;
	}
}
EXPORT_SYMBOL(shtps_mxt_notify_devstate);

void shtps_mxt_notify_make_input_dev(struct input_dev *input_dev)
{
	#if defined ( SHTPS_DEVELOP_MODE_ENABLE )
		shtps_debug_notify_make_input_dev(input_dev);
	#endif /* #if defined ( SHTPS_DEVELOP_MODE_ENABLE ) */
}
void shtps_mxt_notify_free_input_dev()
{
	#if defined ( SHTPS_DEVELOP_MODE_ENABLE )
		shtps_debug_notify_free_input_dev();
	#endif /* #if defined ( SHTPS_DEVELOP_MODE_ENABLE ) */
}

void shtps_mxt_cust_initialize(struct device *dev)
{
	int result = 0;
	struct shtps_mxt *ts = gShtps_mxt;
	ts->dev = dev;
	
	memset(&ts->input_event_info, 0, sizeof(ts->input_event_info));

	shtps_fwctl_init(ts);

	shtps_perflock_init(ts);	//SHTPS_CPU_CLOCK_CONTROL_ENABLE

	shtps_cpu_idle_sleep_wake_lock_init(ts);	//SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		shtps_lpwg_wakelock_init(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
		result = shtps_init_inputdev_key(ts, dev);
		if(result != 0){
			SHTPS_LOG_DBG_PRINT("Failed init input key-device\n");
		}
	#endif /* defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE) */

	shtps_diag_init();
	
	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		shtps_perflock_register_notifier();
	#endif /*  defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE) */

	/* -------------------------------------------------------------------------- */
	shtps_filter_init(ts);

	return;
}
EXPORT_SYMBOL(shtps_mxt_cust_initialize);

void shtps_mxt_cust_deinitialize(void)
{
	struct shtps_mxt *ts = gShtps_mxt;

	shtps_filter_deinit(ts);

	shtps_perflock_deinit(ts);					//SHTPS_CPU_CLOCK_CONTROL_ENABLE
	shtps_cpu_idle_sleep_wake_lock_deinit(ts);	//SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		shtps_lpwg_wakelock_destroy(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	#if defined(SHTPS_PHYSICAL_KEY_ENABLE) || defined(SHTPS_LPWG_MODE_ENABLE)
		shtps_deinit_inputdev_key(ts);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE || SHTPS_LPWG_MODE_ENABLE */
	
	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		shtps_perflock_unregister_notifier();
	#endif /*  defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE) */

	return;
}
EXPORT_SYMBOL(shtps_mxt_cust_deinitialize);

void shtps_mxt_add_input_event(int id, int state, int tool, int x, int y, 
	int w_enable, int w, int z_enable, int z, int ori_enable, int orientation)
{
	struct shtps_mxt *ts = gShtps_mxt;
	struct fingers *fw_cur_fingers = 
				ts->input_event_info.fw.cur;
	
	if(id >= SHTPS_FINGER_MAX){
		SHTPS_LOG_DBG_PRINT("%s: received invalid finger id (%d).\n", __func__, id);
		return;
	}
	
	if(state){
		if(tool == MT_TOOL_FINGER){
			fw_cur_fingers[id].state = SHTPS_TOUCH_STATE_FINGER;
		}
		else if(tool == MT_TOOL_PEN){
			fw_cur_fingers[id].state = SHTPS_TOUCH_STATE_PEN;
		}
		else{
			fw_cur_fingers[id].state = SHTPS_TOUCH_STATE_UNKNOWN;
		}
	}
	else{
		fw_cur_fingers[id].state = SHTPS_TOUCH_STATE_NO_TOUCH;
	}
	fw_cur_fingers[id].x = x;
	fw_cur_fingers[id].y = y;
	fw_cur_fingers[id].w = (w_enable)? w : TPSDEV_INPUT_ITEM_INVALID;
	fw_cur_fingers[id].z = (z_enable)? z : TPSDEV_INPUT_ITEM_INVALID;
	fw_cur_fingers[id].orientation = (ori_enable)? orientation : TPSDEV_INPUT_ITEM_INVALID;
	#if defined ( SHTPS_TOUCH_EMURATOR_ENABLE )
		if(shtps_touch_emu_is_recording() != 0){
			shtps_touch_emu_rec_finger_info(id, state, tool, x, y, w, z, orientation);
		}
	#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */
}
EXPORT_SYMBOL(shtps_mxt_add_input_event);

void shtps_mxt_input_sync(struct input_dev *input_dev)
{
	struct shtps_mxt *ts = gShtps_mxt;
	int i;
	u8 event = 0xff;
	struct fingers *report_pre_fingers = ts->input_event_info.report.pre;
	struct fingers *report_fingers = ts->input_event_info.report.cur;
	
	shtps_calc_notify(ts, &event);
	
	if(event != 0xff){
		shtps_perflock_enable_start(ts, event);	//SHTPS_CPU_CLOCK_CONTROL_ENABLE

		for(i = 0;i < SHTPS_FINGER_MAX;i++){
			if(report_fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				shtps_report_touch_on(input_dev, i,
									  report_fingers[i].x,
									  report_fingers[i].y,
									  report_fingers[i].w,
									  report_fingers[i].z,
									  report_fingers[i].orientation);

			}else if(report_pre_fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				shtps_report_touch_off(input_dev, i,
									  report_fingers[i].x,
									  report_fingers[i].y,
									  report_fingers[i].w,
									  report_fingers[i].z,
									  report_fingers[i].orientation);
			}
		}

		memcpy(ts->input_event_info.report.pre, 
			ts->input_event_info.report.cur, sizeof(ts->input_event_info.report.pre));

		shtps_diag_input_event_notify();

		shtps_perflock_set_event(ts, event);	//SHTPS_CPU_CLOCK_CONTROL_ENABLE
	}

	memcpy(ts->input_event_info.fw.pre, 
		ts->input_event_info.fw.cur, sizeof(ts->input_event_info.fw.pre));

	#if defined ( SHTPS_TOUCH_EMURATOR_ENABLE )
		if(shtps_touch_emu_is_recording() != 0){
			shtps_touch_emu_sync_rec_finger();
		}
	#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */
}
EXPORT_SYMBOL(shtps_mxt_input_sync);

void shtps_mxt_input_force_touchup(struct input_dev *input_dev)
{
	struct shtps_mxt *ts = gShtps_mxt;
	int i;
	struct fingers *report_pre_fingers = ts->input_event_info.report.pre;
	struct fingers *report_fingers = ts->input_event_info.report.cur;
	struct fingers *fw_fingers = ts->input_event_info.fw.cur;
	
	SHTPS_LOG_FUNC_CALL();

	#if defined(SHTPS_MOTION_PEDO_STOP_REQ_ENABLE)
		shub_api_restart_pedometer_func(SHUB_STOP_PED_TYPE_TPS);
	#endif /* SHTPS_MOTION_PEDO_STOP_REQ_ENABLE */

	shtps_filter_force_touchup(ts);

	for(i = 0;i < SHTPS_FINGER_MAX;i++){
		if(report_pre_fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			shtps_report_touch_off(input_dev, i,
								  report_fingers[i].x,
								  report_fingers[i].y,
								  report_fingers[i].w,
								  report_fingers[i].z,
								  report_fingers[i].orientation);
		}
		report_fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
		fw_fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
	}

	memcpy(ts->input_event_info.report.pre, 
		ts->input_event_info.report.cur, sizeof(ts->input_event_info.report.pre));
	memcpy(ts->input_event_info.fw.pre, 
		ts->input_event_info.fw.cur, sizeof(ts->input_event_info.fw.pre));

	shtps_diag_input_event_notify();
}
EXPORT_SYMBOL(shtps_mxt_input_force_touchup);

void shtps_mxt_lpwg_wakelock(int on)
{
	struct shtps_mxt *ts = gShtps_mxt;
	shtps_lpwg_wakelock(ts, on);
}
EXPORT_SYMBOL(shtps_mxt_lpwg_wakelock);

void shtps_mxt_lpwg_wakeup(void)
{
	struct shtps_mxt *ts = gShtps_mxt;

	if(ts->lpwg.notify_enable != 0)
	{
		#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
		{
			if(SHTPS_LPWG_PROXIMITY_SUPPORT_ENABLE != 0){
				if (shtps_proximity_state_check(ts) == 0) {
					if( shtps_proximity_check(ts) != SHTPS_PROXIMITY_NEAR ){
						shtps_notify_wakeup_event(ts);
					}else{
						SHTPS_LOG_DBG_PRINT("[LPWG] proximity near\n");
					}
				}else{
					SHTPS_LOG_DBG_PRINT("[LPWG] proximity power_on + near\n");
				}
			}
			else{
				shtps_notify_wakeup_event(ts);
			}
		}
		#else
			shtps_notify_wakeup_event(ts);
		#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */
	}
	else{
		SHTPS_LOG_DBG_PRINT("[LPWG] notify event blocked\n");
	}
}
EXPORT_SYMBOL(shtps_mxt_lpwg_wakeup);

void shtps_mxt_wake_lock_idle(void)
{
	#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
		struct shtps_mxt *ts = gShtps_mxt;
		if(ts->cpu_idle_sleep_ctrl_p != NULL){
			shtps_wake_lock_idle(ts);
		}
	#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */
}
EXPORT_SYMBOL(shtps_mxt_wake_lock_idle);

void shtps_mxt_wake_unlock_idle(void)
{
	#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
		struct shtps_mxt *ts = gShtps_mxt;
		if(ts->cpu_idle_sleep_ctrl_p != NULL){
			shtps_wake_unlock_idle(ts);
		}
	#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */
}
EXPORT_SYMBOL(shtps_mxt_wake_unlock_idle);

int shtps_mxt_tpin_enable_check(void)
{
	#if defined( SHTPS_TPIN_CHECK_ENABLE )
		struct shtps_mxt *ts = gShtps_mxt;
		return shtps_tpin_enable_check(ts);
	#endif /* SHTPS_TPIN_CHECK_ENABLE */

	return 0;
}
EXPORT_SYMBOL(shtps_mxt_tpin_enable_check);

int shtps_mxt_get_lpmode_setting(u8 *active_value, u8 *idle_value)
{
	#if defined(SHTPS_LOW_POWER_MODE_ENABLE)
		*active_value = SHTPS_LPMODE_ACTVACQINT_VALUE;
		*idle_value   = SHTPS_LPMODE_IDLEACQINT_VALUE;
		return 0;
	#else
		return -1;
	#endif /* SHTPS_LOW_POWER_MODE_ENABLE */
}
EXPORT_SYMBOL(shtps_mxt_get_lpmode_setting);

/* -----------------------------------------------------------------------------------
 */
void msm_tps_setsleep(int sleep)
{
	struct shtps_mxt *ts = gShtps_mxt;
	struct device *dev = ts->dev;

	SHTPS_LOG_DBG_PRINT("%s: sleep=%d\n", __func__, sleep);

	if (dev == NULL){
		return;
	}

	shtps_mutex_lock_ctrl();

	if(sleep == 0){
		#if defined(SHTPS_LPWG_MODE_ENABLE)
			if (ts->lpwg.lpwg_switch){
				ts->lpwg.wakeup_time = jiffies;
				shtps_set_lpwg_mode_off(ts);
//				shtps_read_touchevent(ts, SHTPS_STATE_IDLE);
			}else{
//				shtps_system_set_wakeup(ts);
			}
		#else
//			shtps_system_set_wakeup(ts);
		#endif /* SHTPS_LPWG_MODE_ENABLE */

		shtps_sleep(ts, 0);
	}
	else{
		#if defined(SHTPS_LPWG_MODE_ENABLE)
			if(shtps_is_lpwg_active(ts)){
//				#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
//					shtps_irq_enable(ts);
//				#else
//					shtps_irq_wake_enable(ts);
//				#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

				shtps_set_lpwg_mode_on(ts);
//				shtps_set_lpwg_mode_cal(ts);
			}else{
				shtps_sleep(ts, 1);
//				shtps_system_set_sleep(ts);
			}
		#else
			shtps_sleep(ts, 1);
//			shtps_system_set_sleep(ts);
		#endif /* SHTPS_LPWG_MODE_ENABLE */
	}

	shtps_mutex_unlock_ctrl();

	return;
}
EXPORT_SYMBOL(msm_tps_setsleep);

void msm_tps_set_grip_state(int on)
{
#if defined(SHTPS_LPWG_MODE_ENABLE) && defined(SHTPS_LPWG_GRIP_SUPPORT_ENABLE)
	struct shtps_mxt *ts = gShtps_mxt;
	u8 request = (on == 0)? 0 : 1;

	SHTPS_LOG_FUNC_CALL_INPARAM(on);

	if (ts->dev == NULL){
		return;
	}

	shtps_mutex_lock_ctrl();

	if(ts->lpwg.grip_state != request){
		ts->lpwg.grip_state = request;
		SHTPS_LOG_DBG_PRINT("[LPWG] grip_state = %d\n", ts->lpwg.grip_state);
		
		if(SHTPS_STATE_SLEEP == ts->state_mgr.state){
			u8 new_setting = shtps_is_lpwg_active(ts);
			
			if(new_setting != ts->lpwg.lpwg_switch){
				if(new_setting){
//					#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
//						shtps_irq_enable(ts);
//					#else
//						shtps_irq_wake_enable(ts);
//					#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

//					shtps_system_set_wakeup(ts);
					shtps_set_lpwg_mode_on(ts);
				}else{
//					#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
//						shtps_irq_disable(ts);
//					#else
//						shtps_irq_wake_disable(ts);
//					#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

					shtps_set_lpwg_mode_off(ts);
					shtps_sleep(ts, 1);
//					shtps_system_set_sleep(ts);
				}
			}
		}
	}
	shtps_mutex_unlock_ctrl();
#endif /* SHTPS_LPWG_MODE_ENABLE && SHTPS_LPWG_GRIP_SUPPORT_ENABLE */
	return;
}
EXPORT_SYMBOL(msm_tps_set_grip_state);

void shtps_setFlipInformation(int state)
{
	SHTPS_LOG_DBG_PRINT("%s: state=%d\n", __func__, state);
	return;
}
EXPORT_SYMBOL(shtps_setFlipInformation);

int msm_tps_set_veilview_state_on(void)
{
	SHTPS_LOG_FUNC_CALL();
	return 0;
}
EXPORT_SYMBOL(msm_tps_set_veilview_state_on);

int msm_tps_set_veilview_state_off(void)
{
	SHTPS_LOG_FUNC_CALL();
	return 0;
}
EXPORT_SYMBOL(msm_tps_set_veilview_state_off);

int msm_tps_get_veilview_pattern(void)
{
	SHTPS_LOG_FUNC_CALL();
	return 0;
}
EXPORT_SYMBOL(msm_tps_get_veilview_pattern);

