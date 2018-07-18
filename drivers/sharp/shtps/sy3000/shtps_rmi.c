/* drivers/sharp/shtps/sy3000/shtps_rmi.c
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
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/input/mt.h>

#include <sharp/shtps_dev.h>
#include <sharp/sh_smem.h>
#include <sharp/sh_boot_manager.h>

#include "shtps_rmi.h"
#include "shtps_cfg.h"
#include "shtps_param.h"

#include "shtps_fwctl.h"
#include "shtps_rmi_sub.h"
#include "shtps_rmi_devctl.h"
#include "shtps_rmi_debug.h"
#include "shtps_log.h"

#if defined(SHTPS_MOTION_PEDO_STOP_REQ_ENABLE)
	#include <sharp/shub_driver.h>
#endif /* SHTPS_MOTION_PEDO_STOP_REQ_ENABLE */

/* -----------------------------------------------------------------------------------
 */
static DEFINE_MUTEX(shtps_ctrl_lock);
static DEFINE_MUTEX(shtps_loader_lock);

#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
	static DEFINE_MUTEX(shtps_proc_lock);
#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	static DEFINE_MUTEX(shtps_facetouch_qosctrl_lock);
#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */

/* -----------------------------------------------------------------------------------
 */
#define	TPS_MUTEX_LOG_LOCK(VALUE)		SHTPS_LOG_DBG_PRINT("mutex_lock:   -> (%s)\n",VALUE)
#define	TPS_MUTEX_LOG_UNLOCK(VALUE)		SHTPS_LOG_DBG_PRINT("mutex_unlock: <- (%s)\n",VALUE)

/* -----------------------------------------------------------------------------------
 */
struct shtps_rmi_spi*	gShtps_rmi_spi = NULL;

#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
	u8					gLogOutputEnable = 0;
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */

#if defined( SHTPS_MODULE_PARAM_ENABLE )
	static int shtps_irq_wake_state = 0;
	static int shtps_spi_clk_ctrl_state = 0;
	static int shtps_rezero_state = 0;

	module_param(shtps_irq_wake_state, int, S_IRUGO);
	module_param(shtps_spi_clk_ctrl_state, int, S_IRUGO);
	module_param(shtps_rezero_state, int, S_IRUGO | S_IWUSR);
#endif /* SHTPS_MODULE_PARAM_ENABLE */

/* -----------------------------------------------------------------------------------
 */
typedef int (shtps_state_func)(struct shtps_rmi_spi *ts, int param);
struct shtps_state_func {
	shtps_state_func	*enter;
	shtps_state_func	*start;
	shtps_state_func	*stop;
	shtps_state_func	*sleep;
	shtps_state_func	*wakeup;
	shtps_state_func	*start_ldr;
	shtps_state_func	*start_tm;
	shtps_state_func	*stop_tm;
	shtps_state_func	*facetouch_on;
	shtps_state_func	*facetouch_off;
	shtps_state_func	*interrupt;
	shtps_state_func	*timeout;
};

/* -----------------------------------------------------------------------------------
 */
#if (defined(SHTPS_BOOT_FWUPDATE_ENABLE) && defined(SHTPS_BOOT_FWUPDATE_ONLY_ON_HANDSET)) || \
		(defined(SHTPS_MULTI_FW_ENABLE) && defined(SHTPS_CHECK_HWID_ENABLE))
	static int shtps_system_get_hw_type(void);
#endif /* (defined(SHTPS_BOOT_FWUPDATE_ENABLE) && defined(SHTPS_BOOT_FWUPDATE_ONLY_ON_HANDSET)) || 
			(defined(SHTPS_MULTI_FW_ENABLE) && defined(SHTPS_CHECK_HWID_ENABLE)) */
#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
	static void shtps_irq_proc(struct shtps_rmi_spi *ts, u8 dummy);
	static void shtps_grip_proc(struct shtps_rmi_spi *ts, u8 request);
	static void shtps_setsleep_proc(struct shtps_rmi_spi *ts, u8 sleep);
	static void shtps_async_open_proc(struct shtps_rmi_spi *ts, u8 dummy);
	static void shtps_async_close_proc(struct shtps_rmi_spi *ts, u8 dummy);
	static void shtps_async_enable_proc(struct shtps_rmi_spi *ts, u8 dummy);
	static void shtps_suspend_spi_wake_lock(struct shtps_rmi_spi *ts, u8 lock);
	#ifdef SHTPS_DEVELOP_MODE_ENABLE
		static void shtps_exec_suspend_pending_proc_delayed(struct shtps_rmi_spi *ts);
		static void shtps_deter_suspend_spi_pending_proc_delayed_work_function(struct work_struct *work);
	#endif /* SHTPS_DEVELOP_MODE_ENABLE */
	static void shtps_exec_suspend_pending_proc(struct shtps_rmi_spi *ts);
	static void shtps_deter_suspend_spi_pending_proc_work_function(struct work_struct *work);
#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
static irqreturn_t shtps_irq_handler(int irq, void *dev_id);
static irqreturn_t shtps_irq(int irq, void *dev_id);
static void shtps_work_tmof(struct work_struct *data);
static enum hrtimer_restart shtps_delayed_rezero_timer_function(struct hrtimer *timer);
static void shtps_rezero_delayed_work_function(struct work_struct *work);
static void shtps_delayed_rezero(struct shtps_rmi_spi *ts, unsigned long delay_us);
static void shtps_delayed_rezero_cancel(struct shtps_rmi_spi *ts);
#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	static int shtps_is_singlefinger(struct shtps_rmi_spi *ts, u8 gs_info);
	static void shtps_autorezero_disable(struct shtps_rmi_spi *ts);
	static void shtps_autorezero_enable(struct shtps_rmi_spi *ts);
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */

static void shtps_rezero_handle(struct shtps_rmi_spi *ts, u8 event, u8 gs);
static void shtps_reset_startuptime(struct shtps_rmi_spi *ts);
static unsigned long shtps_check_startuptime(struct shtps_rmi_spi *ts);
#if defined(SHTPS_PEN_DETECT_ENABLE)
	static int shtps_set_pen_enable(struct shtps_rmi_spi *ts, int onoff);
	static void shtps_set_pen_detect_init(struct shtps_rmi_spi *ts);
#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */

#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	static int shtps_map_construct(struct shtps_rmi_spi *ts, u8 func_check);
#else
	static int shtps_map_construct(struct shtps_rmi_spi *ts);
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	static u8 shtps_get_lpmode_state(struct shtps_rmi_spi *ts);
#endif /* SHTPS_LOW_POWER_MODE_ENABLE */

#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	static void shtps_set_doze_mode(struct shtps_rmi_spi *ts, int on);
	static void shtps_set_lpmode_init(struct shtps_rmi_spi *ts);
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

#if defined(SHTPS_HOVER_DETECT_ENABLE)
	static int shtps_set_hover_detect_init(struct shtps_rmi_spi *ts);
#endif /* SHTPS_HOVER_DETECT_ENABLE */

#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )
	static void shtps_touchkey_inproxymity_delayed_work_function(struct work_struct *work);
	static void shtps_touchkey_inproxymity_delayed_work_cancel(struct shtps_rmi_spi *ts);
	static void shtps_touchkey_inproxymity_delayed_work_start(struct shtps_rmi_spi *ts);
	static void shtps_touchkey_delayed_work_function(struct work_struct *work);
	static void shtps_touchkey_delayed_work_cancel(struct shtps_rmi_spi *ts);
	static void shtps_touchkey_delayed_work_start(struct shtps_rmi_spi *ts);
#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */

#if defined(SHTPS_LPWG_MODE_ENABLE)
	static void shtps_lpwg_notify_interval_delayed_work_function(struct work_struct *work);
	static void shtps_lpwg_notify_interval_stop(struct shtps_rmi_spi *ts);
	static void shtps_lpwg_notify_interval_start(struct shtps_rmi_spi *ts);
	static void shtps_lpwg_wakelock_init(struct shtps_rmi_spi *ts);
	static void shtps_lpwg_wakelock_destroy(struct shtps_rmi_spi *ts);
	static void shtps_lpwg_wakelock(struct shtps_rmi_spi *ts, int on);
	static void shtps_lpwg_prepare(struct shtps_rmi_spi *ts);
	static void shtps_lpwg_disposal(struct shtps_rmi_spi *ts);
	static void shtps_set_lpwg_mode_cal(struct shtps_rmi_spi *ts);
#endif /* SHTPS_LPWG_MODE_ENABLE */

#if defined(SHTPS_MULTI_FW_ENABLE)
	static void shtps_get_multi_fw_type(struct shtps_rmi_spi *ts);
#endif /* SHTPS_MULTI_FW_ENABLE */

static int shtps_init_param(struct shtps_rmi_spi *ts);
static void shtps_standby_param(struct shtps_rmi_spi *ts);
static void shtps_clr_startup_err(struct shtps_rmi_spi *ts);
static void shtps_notify_startup(struct shtps_rmi_spi *ts, u8 err);
static void shtps_set_startmode(struct shtps_rmi_spi *ts, u8 mode);
static int shtps_get_startmode(struct shtps_rmi_spi *ts);
static void shtps_set_touch_info(struct shtps_rmi_spi *ts, u8 *buf, struct shtps_touch_info *info);
static void shtps_calc_notify(struct shtps_rmi_spi *ts, u8 *buf, struct shtps_touch_info *info, u8 *event);

#if defined(SHTPS_LPWG_MODE_ENABLE)
	static void shtps_event_update(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
#endif /* #if defined(SHTPS_LPWG_MODE_ENABLE) */

#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
	#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )

		#if defined(SHTPS_LOG_DEBUG_ENABLE) || defined(SHTPS_LOG_EVENT_ENABLE)
			static char* shtps_get_key_name(int key);
		#endif /* SHTPS_LOG_DEBUG_ENABLE || SHTPS_LOG_EVENT_ENABLE */

		static int shtps_key_event_report_each_key(struct shtps_rmi_spi *ts, u8 state, int key);
	#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */

	static void shtps_key_event_report(struct shtps_rmi_spi *ts, u8 state);
	static void shtps_key_event_force_touchup(struct shtps_rmi_spi *ts);
#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

#if defined(SHTPS_LPWG_MODE_ENABLE)
	static void shtps_notify_wakeup_event(struct shtps_rmi_spi *ts);
	#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
		void shtps_notify_cancel_wakeup_event(struct shtps_rmi_spi *ts);
	#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */
#endif /* SHTPS_LPWG_MODE_ENABLE */
static int shtps_tm_irqcheck(struct shtps_rmi_spi *ts);
static int shtps_tm_wait_attn(struct shtps_rmi_spi *ts);
static void shtps_tm_wakeup(struct shtps_rmi_spi *ts);
static void shtps_tm_cancel(struct shtps_rmi_spi *ts);
static int shtps_start_tm(struct shtps_rmi_spi *ts);
static void shtps_stop_tm(struct shtps_rmi_spi *ts);
#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
	//nothing
#else
	static void shtps_irq_wake_disable(struct shtps_rmi_spi *ts);
	static void shtps_irq_wake_enable(struct shtps_rmi_spi *ts);
#endif /* !SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

static int shtps_irq_resuest(struct shtps_rmi_spi *ts);
static void shtps_irqtimer_start(struct shtps_rmi_spi *ts, long time_ms);
static void shtps_irqtimer_stop(struct shtps_rmi_spi *ts);
#if defined(SHTPS_LPWG_MODE_ENABLE)
	static void shtps_read_touchevent_insleep(struct shtps_rmi_spi *ts, int state);
#endif /*  SHTPS_LPWG_MODE_ENABLE */

#if defined(SHTPS_IRQ_LOADER_CHECK_INT_STATUS_ENABLE)
	static int shtps_loader_irqclr(struct shtps_rmi_spi *ts);
#else
	static void shtps_loader_irqclr(struct shtps_rmi_spi *ts);
#endif /* SHTPS_IRQ_LOADER_CHECK_INT_STATUS_ENABLE */
#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
	int shtps_boot_fwupdate_enable_check(struct shtps_rmi_spi *ts);
	#if defined(SHTPS_BOOT_FWUPDATE_FORCE_UPDATE)
		//nothing
	#else
		int shtps_fwup_flag_check(void);
	#endif /* #if !defined(SHTPS_BOOT_FWUPDATE_FORCE_UPDATE) */
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

static int shtps_fwupdate_enable(struct shtps_rmi_spi *ts);
static int shtps_loader_wait_attn(struct shtps_rmi_spi *ts);
static void shtps_loader_wakeup(struct shtps_rmi_spi *ts);
static int shtps_exit_bootloader(struct shtps_rmi_spi *ts);
static int state_change(struct shtps_rmi_spi *ts, int state);
static int shtps_statef_nop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_error(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_facetouch_on(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_facetouch_off(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_idle_start(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_idle_start_ldr(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_idle_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_waiwakeup_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_waiwakeup_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_waiwakeup_tmo(struct shtps_rmi_spi *ts, int param);
static int shtps_state_waiready_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_state_waiready_int(struct shtps_rmi_spi *ts, int param);
static int shtps_state_waiready_tmo(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_sleep(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_facetouch_on(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_loader_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_loader_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_loader_int(struct shtps_rmi_spi *ts, int param);
#if defined( SHTPS_INVERTING_GHOST_REJECTION_ENABLE )
static int shtps_statef_facetouch_enter(struct shtps_rmi_spi *ts, int param);
#endif /* SHTPS_INVERTING_GHOST_REJECTION_ENABLE */
static int shtps_statef_facetouch_sleep(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_stoptm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_wakeup(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_on(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_wakeup(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_int(struct shtps_rmi_spi *ts, int param);
static int shtps_rmi_open(struct input_dev *dev);
static void shtps_rmi_close(struct input_dev *dev);
static int shtps_init_internal_variables(struct shtps_rmi_spi *ts);
static void shtps_deinit_internal_variables(struct shtps_rmi_spi *ts);
static int shtps_init_inputdev(struct shtps_rmi_spi *ts, struct device *ctrl_dev_p, char *modalias);
static void shtps_deinit_inputdev(struct shtps_rmi_spi *ts);
#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
	static int shtps_init_inputdev_key(struct shtps_rmi_spi *ts, struct device *ctrl_dev_p);
	static void shtps_deinit_inputdev_key(struct shtps_rmi_spi *ts);
#endif /* SHTPS_PHYSICAL_KEY_ENABLE || SHTPS_LPWG_MODE_ENABLE */

/* -----------------------------------------------------------------------------------
 */
const static struct shtps_state_func state_idle = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_idle_start,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_idle_start_ldr,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .interrupt      = shtps_statef_idle_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_waiwakeup = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_waiwakeup_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .interrupt      = shtps_statef_waiwakeup_int,
    .timeout        = shtps_statef_waiwakeup_tmo
};

const static struct shtps_state_func state_waiready = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_state_waiready_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .interrupt      = shtps_state_waiready_int,
    .timeout        = shtps_state_waiready_tmo
};

const static struct shtps_state_func state_active = {
    .enter          = shtps_statef_active_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_active_stop,
    .sleep          = shtps_statef_active_sleep,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_active_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_active_facetouch_on,
    .facetouch_off  = shtps_statef_nop,
    .interrupt      = shtps_statef_active_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_loader = {
    .enter          = shtps_statef_loader_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_loader_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_nop,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .interrupt      = shtps_statef_loader_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_facetouch = {
#if defined( SHTPS_INVERTING_GHOST_REJECTION_ENABLE )
    .enter          = shtps_statef_facetouch_enter,
#else
    .enter          = shtps_statef_nop,
#endif/* SHTPS_INVERTING_GHOST_REJECTION_ENABLE */
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_active_stop,
    .sleep          = shtps_statef_facetouch_sleep,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_facetouch_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_nop,
    .facetouch_off  = shtps_statef_facetouch_facetouch_off,
    .interrupt      = shtps_statef_facetouch_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_fwtm = {
    .enter          = shtps_statef_fwtm_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_fwtm_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_nop,
    .stop_tm        = shtps_statef_fwtm_stoptm,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .interrupt      = shtps_statef_fwtm_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_sleep = {
    .enter          = shtps_statef_sleep_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_sleep_wakeup,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_sleep_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_sleep_facetouch_on,
    .facetouch_off  = shtps_statef_nop,
    .interrupt      = shtps_statef_sleep_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_sleep_facetouch = {
    .enter          = shtps_statef_sleep_facetouch_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_sleep_facetouch_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_sleep_facetouch_wakeup,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_sleep_facetouch_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_nop,
    .facetouch_off  = shtps_statef_sleep_facetouch_facetouch_off,
    .interrupt      = shtps_statef_sleep_facetouch_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func *state_func_tbl[] = {
	&state_idle,
	&state_waiwakeup,
	&state_waiready,
	&state_active,
	&state_loader,
	&state_facetouch,
	&state_fwtm,
	&state_sleep,
	&state_sleep_facetouch,
};

#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
typedef void (*shtps_deter_suspend_spi_pending_func_t)(struct shtps_rmi_spi*, u8);
static const shtps_deter_suspend_spi_pending_func_t SHTPS_SUSPEND_PENDING_FUNC_TBL[] = {
	shtps_irq_proc,							/**< SHTPS_DETER_SUSPEND_SPI_PROC_IRQ */
	shtps_charger_armor_proc,				/**< SHTPS_DETER_SUSPEND_SPI_PROC_CHARGER_ARMOR */
	shtps_setsleep_proc,					/**< SHTPS_DETER_SUSPEND_SPI_PROC_SETSLEEP */
	shtps_ioctl_setlpwg_proc,				/**< SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPWG */
	shtps_ioctl_setlpmode_proc,				/**< SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPMODE */
	shtps_ioctl_setconlpmode_proc,			/**< SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETCONLPMODE */
	shtps_ioctl_setlcdbrightlpmode_proc,	/**< SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLCDBRIGHTLPMODE */
	shtps_async_open_proc,					/**< SHTPS_DETER_SUSPEND_SPI_PROC_OPEN */
	shtps_async_close_proc,					/**< SHTPS_DETER_SUSPEND_SPI_PROC_CLOSE */
	shtps_async_enable_proc,				/**< SHTPS_DETER_SUSPEND_SPI_PROC_ENABLE */
	shtps_grip_proc,						/**< SHTPS_DETER_SUSPEND_SPI_PROC_GRIP */
	shtps_ioctl_sethover_proc,				/**< SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETHOVER */
	shtps_ioctl_setpen_proc,				/**< SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETPEN */
};
#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

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

void shtps_mutex_lock_loader(void)
{
	TPS_MUTEX_LOG_LOCK("shtps_loader_lock");
	mutex_lock(&shtps_loader_lock);
}

void shtps_mutex_unlock_loader(void)
{
	TPS_MUTEX_LOG_UNLOCK("shtps_loader_lock");
	mutex_unlock(&shtps_loader_lock);
}

void shtps_mutex_lock_proc(void)
{
#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
	TPS_MUTEX_LOG_LOCK("shtps_proc_lock");
	mutex_lock(&shtps_proc_lock);
#endif
}

void shtps_mutex_unlock_proc(void)
{
#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
	TPS_MUTEX_LOG_UNLOCK("shtps_proc_lock");
	mutex_unlock(&shtps_proc_lock);
#endif
}

void shtps_mutex_lock_facetouch_qos_ctrl(void)
{
#if defined(CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT)
	TPS_MUTEX_LOG_LOCK("shtps_facetouch_qosctrl_lock");
	mutex_lock(&shtps_facetouch_qosctrl_lock);
#endif
}

void shtps_mutex_unlock_facetouch_qos_ctrl(void)
{
#if defined(CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT)
	TPS_MUTEX_LOG_UNLOCK("shtps_facetouch_qosctrl_lock");
	mutex_unlock(&shtps_facetouch_qosctrl_lock);
#endif
}

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CHECK_HWID_ENABLE)
int shtps_system_get_hw_revision(void)
{
	SHTPS_LOG_FUNC_CALL();
	return sh_boot_get_hw_revision();
}
#endif /* SHTPS_CHECK_HWID_ENABLE */

#if (defined(SHTPS_BOOT_FWUPDATE_ENABLE) && defined(SHTPS_BOOT_FWUPDATE_ONLY_ON_HANDSET)) || \
		(defined(SHTPS_MULTI_FW_ENABLE) && defined(SHTPS_CHECK_HWID_ENABLE))
static int shtps_system_get_hw_type(void)
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
#endif /* (defined(SHTPS_BOOT_FWUPDATE_ENABLE) && defined(SHTPS_BOOT_FWUPDATE_ONLY_ON_HANDSET)) || 
			(defined(SHTPS_MULTI_FW_ENABLE) && defined(SHTPS_CHECK_HWID_ENABLE)) */

/* -----------------------------------------------------------------------------------
 */
void shtps_system_set_sleep(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_INPUT_POWER_MODE_CHANGE_ENABLE)
		extern struct device*	shtpsif_device;
		SHTPS_LOG_FUNC_CALL();
		shtps_device_sleep(shtpsif_device);
	#endif /* SHTPS_INPUT_POWER_MODE_CHANGE_ENABLE */
}

void shtps_system_set_wakeup(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_INPUT_POWER_MODE_CHANGE_ENABLE)
		extern struct device*	shtpsif_device;
		SHTPS_LOG_FUNC_CALL();
		shtps_device_wakeup(shtpsif_device);
	#endif /* SHTPS_INPUT_POWER_MODE_CHANGE_ENABLE */
}

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
static void shtps_irq_proc(struct shtps_rmi_spi *ts, u8 dummy)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);
	shtps_wake_lock_idle(ts);	//SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if(0 != ts->lpwg.lpwg_switch && SHTPS_STATE_SLEEP == ts->state_mgr.state){
			shtps_lpwg_wakelock(ts, 1);
		}
	#endif /* defined(SHTPS_LPWG_MODE_ENABLE) */

	request_event(ts, SHTPS_EVENT_INTERRUPT, 1);

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		shtps_lpwg_wakelock(ts, 0);
	#endif /* defined(SHTPS_LPWG_MODE_ENABLE) */

	shtps_wake_unlock_idle(ts);	//SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE
	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_END);
}

static void shtps_grip_proc(struct shtps_rmi_spi *ts, u8 request)
{
	SHTPS_LOG_FUNC_CALL();

	#if defined(SHTPS_LPWG_MODE_ENABLE) && defined(SHTPS_LPWG_GRIP_SUPPORT_ENABLE)
		shtps_mutex_lock_ctrl();

		if(ts->lpwg.grip_state != request){
			ts->lpwg.grip_state = request;
			SHTPS_LOG_DBG_PRINT("[LPWG] grip_state = %d\n", ts->lpwg.grip_state);

			if(SHTPS_STATE_SLEEP == ts->state_mgr.state){
				u8 new_setting = shtps_is_lpwg_active(ts);

				if(new_setting != ts->lpwg.lpwg_switch){
					if(new_setting){
						#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
							shtps_irq_enable(ts);
						#else
							shtps_irq_wake_enable(ts);
						#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

						shtps_system_set_wakeup(ts);
						shtps_set_lpwg_mode_on(ts);
					}else{
						#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
							shtps_irq_disable(ts);
						#else
							shtps_irq_wake_disable(ts);
						#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

						shtps_set_lpwg_mode_off(ts);
						shtps_sleep(ts, 1);
						shtps_system_set_sleep(ts);
					}
				}
			}
		}
		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_LPWG_MODE_ENABLE && SHTPS_LPWG_GRIP_SUPPORT_ENABLE */
}

void shtps_charger_armor_proc(struct shtps_rmi_spi *ts, u8 charger)
{
	SHTPS_LOG_FUNC_CALL();

	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		shtps_mutex_lock_ctrl();
		shtps_filter_set_charger_armor(ts, (int)ts->state_mgr.state, (int)charger);
		shtps_mutex_unlock_ctrl();
	#endif /* #if defined( SHTPS_CHARGER_ARMOR_ENABLE ) */
}

static void shtps_setsleep_proc(struct shtps_rmi_spi *ts, u8 sleep)
{
	SHTPS_LOG_FUNC_CALL();

	if(sleep){
		#if defined(SHTPS_DYNAMIC_RESET_CONTROL_ENABLE)
			shtps_filter_dynamic_reset_sleep_process(ts);
		#endif /* SHTPS_DYNAMIC_RESET_CONTROL_ENABLE */

		request_event(ts, SHTPS_EVENT_SLEEP, 0);
	}else{
		#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
			ts->system_boot_mode = SH_BOOT_NORMAL;
		#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

		request_event(ts, SHTPS_EVENT_WAKEUP, 0);
	}
}

void shtps_ioctl_setlpwg_proc(struct shtps_rmi_spi *ts, u8 on)	
{
	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_ctrl();
	ts->lpwg.lpwg_state = on;
	SHTPS_LOG_DBG_PRINT(" [LPWG] lpwg_state = %d\n", ts->lpwg.lpwg_state);
	if (SHTPS_STATE_SLEEP == ts->state_mgr.state) {
		u8 new_setting = shtps_is_lpwg_active(ts);

		if(new_setting != ts->lpwg.lpwg_switch){
			if(new_setting){
				#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
					shtps_irq_enable(ts);
				#else
					shtps_irq_wake_enable(ts);
				#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

				shtps_system_set_wakeup(ts);
				shtps_set_lpwg_mode_on(ts);
			}else{
				#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
					shtps_irq_disable(ts);
				#else
					shtps_irq_wake_disable(ts);
				#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

				shtps_set_lpwg_mode_off(ts);
				shtps_sleep(ts, 1);
				shtps_system_set_sleep(ts);
			}
		}
	}
	shtps_mutex_unlock_ctrl();
}

void shtps_ioctl_sethover_proc(struct shtps_rmi_spi *ts, u8 on)
{
	SHTPS_LOG_FUNC_CALL();

	#if defined(SHTPS_HOVER_DETECT_ENABLE)
		shtps_mutex_lock_ctrl();

		if(on == 0){
			shtps_set_hover_detect_disable(ts);

			#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
				shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_HOVER_OFF, 1);
			#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
		}else{
			shtps_set_hover_detect_enable(ts);

			#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
				shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_HOVER_OFF, 0);
			#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
		}
		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_HOVER_DETECT_ENABLE */
}

void shtps_ioctl_setlpmode_proc(struct shtps_rmi_spi *ts, u8 on)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	SHTPS_LOG_FUNC_CALL();
	shtps_mutex_lock_ctrl();
	shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_COMMON, (int)on);
	shtps_mutex_unlock_ctrl();
#endif /* SHTPS_LOW_POWER_MODE_ENABLE */
}

void shtps_ioctl_setconlpmode_proc(struct shtps_rmi_spi *ts, u8 on)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	SHTPS_LOG_FUNC_CALL();
	shtps_mutex_lock_ctrl();
	shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_ECO, (int)on);
	shtps_mutex_unlock_ctrl();
#endif /* SHTPS_LOW_POWER_MODE_ENABLE */
}

void shtps_ioctl_setlcdbrightlpmode_proc(struct shtps_rmi_spi *ts, u8 on)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	SHTPS_LOG_FUNC_CALL();
	shtps_mutex_lock_ctrl();
	shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_LCD_BRIGHT, (int)on);
	shtps_mutex_unlock_ctrl();
#endif /* SHTPS_LOW_POWER_MODE_ENABLE */
}

static void shtps_async_open_proc(struct shtps_rmi_spi *ts, u8 dummy)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_OPEN);
}

static void shtps_async_close_proc(struct shtps_rmi_spi *ts, u8 dummy)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_CLOSE);
}

static void shtps_async_enable_proc(struct shtps_rmi_spi *ts, u8 dummy)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_ENABLE);
}

void shtps_ioctl_setpen_proc(struct shtps_rmi_spi *ts, u8 on)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_PEN_DETECT_ENABLE)
		shtps_mutex_lock_ctrl();
		shtps_set_pen_enable(ts, (int)on);
		shtps_mutex_unlock_ctrl();
	#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */
}

static void shtps_suspend_spi_wake_lock(struct shtps_rmi_spi *ts, u8 lock)
{
	SHTPS_LOG_FUNC_CALL();
	if(lock){
		if(ts->deter_suspend_spi.wake_lock_state == 0){
			ts->deter_suspend_spi.wake_lock_state = 1;
			wake_lock(&ts->deter_suspend_spi.wake_lock);
			pm_qos_update_request(&ts->deter_suspend_spi.pm_qos_lock_idle, SHTPS_QOS_LATENCY_DEF_VALUE);
		    SHTPS_LOG_DBG_PRINT("[suspend spi] wake_lock\n");
		}
	}else{
		if(ts->deter_suspend_spi.wake_lock_state == 1){
			ts->deter_suspend_spi.wake_lock_state = 0;
			wake_unlock(&ts->deter_suspend_spi.wake_lock);
			pm_qos_update_request(&ts->deter_suspend_spi.pm_qos_lock_idle, PM_QOS_DEFAULT_VALUE);
		    SHTPS_LOG_DBG_PRINT("[suspend spi] wake_unlock\n");
		}
	}
}

int shtps_check_suspend_state(struct shtps_rmi_spi *ts, int proc, u8 param)
{
	int ret = 0;
	
	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_proc();
	if(ts->deter_suspend_spi.suspend){
		shtps_suspend_spi_wake_lock(ts, 1);
		ts->deter_suspend_spi.pending_info[proc].pending= 1;
		ts->deter_suspend_spi.pending_info[proc].param  = param;
		ret = 1;
	}else{
		ts->deter_suspend_spi.pending_info[proc].pending= 0;
		ret = 0;
	}

	if(proc == SHTPS_DETER_SUSPEND_SPI_PROC_OPEN ||
		proc == SHTPS_DETER_SUSPEND_SPI_PROC_ENABLE)
	{
		if(ts->deter_suspend_spi.pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_CLOSE].pending){
		    SHTPS_LOG_DBG_PRINT("[suspend spi] Pending flag of TPS Close Reqeust clear\n");
			ts->deter_suspend_spi.pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_CLOSE].pending = 0;
		}
	}else if(proc == SHTPS_DETER_SUSPEND_SPI_PROC_CLOSE){
		if(ts->deter_suspend_spi.pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_OPEN].pending){
		    SHTPS_LOG_DBG_PRINT("[suspend spi] Pending flag of TPS Open Reqeust clear\n");
			ts->deter_suspend_spi.pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_OPEN].pending  = 0;
		}
		if(ts->deter_suspend_spi.pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_ENABLE].pending){
		    SHTPS_LOG_DBG_PRINT("[suspend spi] Pending flag of TPS Enable Reqeust clear\n");
			ts->deter_suspend_spi.pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_ENABLE].pending= 0;
		}
	}
	
	shtps_mutex_unlock_proc();
	return ret;
}

#ifdef SHTPS_DEVELOP_MODE_ENABLE
static void shtps_exec_suspend_pending_proc_delayed(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
    SHTPS_LOG_DBG_PRINT("%s() cancel_delayed_work()\n", __func__);
	cancel_delayed_work(&ts->deter_suspend_spi.pending_proc_work_delay);

    SHTPS_LOG_DBG_PRINT("%s() schedule_delayed_work(%d ms)\n", __func__, SHTPS_SUSPEND_SPI_RESUME_FUNC_DELAY);
	schedule_delayed_work(&ts->deter_suspend_spi.pending_proc_work_delay, 
							msecs_to_jiffies(SHTPS_SUSPEND_SPI_RESUME_FUNC_DELAY));

    SHTPS_LOG_DBG_PRINT("%s() done\n", __func__);
}

static void shtps_deter_suspend_spi_pending_proc_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_deter_suspend_spi *dss = container_of(dw, struct shtps_deter_suspend_spi, pending_proc_work_delay);
	struct shtps_rmi_spi *ts = container_of(dss, struct shtps_rmi_spi, deter_suspend_spi);

	SHTPS_LOG_FUNC_CALL();

	schedule_work(&ts->deter_suspend_spi.pending_proc_work);
	return;
}
#endif /* SHTPS_DEVELOP_MODE_ENABLE */

static void shtps_exec_suspend_pending_proc(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_work_sync(&ts->deter_suspend_spi.pending_proc_work);

	#ifdef SHTPS_DEVELOP_MODE_ENABLE
		if(SHTPS_SUSPEND_SPI_RESUME_FUNC_DELAY > 0){
			shtps_exec_suspend_pending_proc_delayed(ts);
		}else{
			schedule_work(&ts->deter_suspend_spi.pending_proc_work);
		}
	#else /* SHTPS_DEVELOP_MODE_ENABLE */
		schedule_work(&ts->deter_suspend_spi.pending_proc_work);
	#endif /* SHTPS_DEVELOP_MODE_ENABLE */

    SHTPS_LOG_DBG_PRINT("%s() done\n", __func__);
}

static void shtps_deter_suspend_spi_pending_proc_work_function(struct work_struct *work)
{
	struct shtps_deter_suspend_spi *dss = container_of(work, struct shtps_deter_suspend_spi, pending_proc_work);
	struct shtps_rmi_spi *ts = container_of(dss, struct shtps_rmi_spi, deter_suspend_spi);
	int i;

	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_proc();

	for(i = 0;i < SHTPS_DETER_SUSPEND_SPI_PROC_NUM;i++){
		if(ts->deter_suspend_spi.pending_info[i].pending){
			SHTPS_SUSPEND_PENDING_FUNC_TBL[i](ts, ts->deter_suspend_spi.pending_info[i].param);
			ts->deter_suspend_spi.pending_info[i].pending = 0;
		}
	}
	shtps_suspend_spi_wake_lock(ts, 0);

	shtps_mutex_unlock_proc();
}

void shtps_set_suspend_state(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_proc();
	ts->deter_suspend_spi.suspend = 1;
	shtps_mutex_unlock_proc();
}

void shtps_clr_suspend_state(struct shtps_rmi_spi *ts)
{
	int i;
	int hold_process = 0;
	
	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_proc();
	ts->deter_suspend_spi.suspend = 0;
	for(i = 0;i < SHTPS_DETER_SUSPEND_SPI_PROC_NUM;i++){
		if(ts->deter_suspend_spi.pending_info[i].pending){
			hold_process = 1;
			break;
		}
	}
	shtps_mutex_unlock_proc();

	if(hold_process){
		shtps_exec_suspend_pending_proc(ts);
	}else{
		shtps_suspend_spi_wake_lock(ts, 0);
	}

	shtps_mutex_lock_ctrl();
	if(ts->deter_suspend_spi.suspend_irq_detect != 0){
		#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
			if(ts->deter_suspend_spi.suspend_irq_state == SHTPS_IRQ_STATE_ENABLE){
				SHTPS_LOG_DBG_PRINT("[suspend_spi] irq wake enable\n");
				SHTPS_LOG_DBG_PRINT("[suspend_spi] irq enable\n");
				shtps_irq_enable(ts);
			}
		#else
			if(ts->deter_suspend_spi.suspend_irq_wake_state == SHTPS_IRQ_WAKE_ENABLE){
				SHTPS_LOG_DBG_PRINT("[suspend_spi] irq wake enable\n");
				shtps_irq_wake_enable(ts);
			}
			if(ts->deter_suspend_spi.suspend_irq_state == SHTPS_IRQ_STATE_ENABLE){
				SHTPS_LOG_DBG_PRINT("[suspend_spi] irq enable\n");
				shtps_irq_enable(ts);
			}
		#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

		ts->deter_suspend_spi.suspend_irq_detect = 0;
	}
	shtps_mutex_unlock_ctrl();
}
#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_PALM_HOST_DETECT_TOUCH_UP_ENABLE )
static void shtps_palm_host_detect_touch_up(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, int notify_cancel)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int fingerNum = 0;
	int palm_detect = 0;

	if(!SHTPS_PALM_DETECT_TOUCH_UP_ENABLE){
		return;
	}
	
	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			fingerNum++;
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_PALM){
				palm_detect = 1;
			}
		}
	}
	
	if(fingerNum == 0){
		ts->host_palm_detect_wait_all_touchup = 0;
	}
	
	if(palm_detect == 1 || ts->host_palm_detect_wait_all_touchup == 1){
		for(i = 0; i < fingerMax; i++){
			if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				if(notify_cancel == 1){
					if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
						shtps_report_touch_on(ts, i,
											  SHTPS_TOUCH_CANCEL_COORDINATES_X,
											  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
											  1,
											  ts->report_info.fingers[i].wx,
											  ts->report_info.fingers[i].wy,
											  ts->report_info.fingers[i].z);
						input_sync(ts->input);
					}
				}
				info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			}
		}
	}
	
	if(palm_detect == 1){
		ts->host_palm_detect = 1;
		ts->host_palm_detect_wait_all_touchup = 1;
	}else{
		ts->host_palm_detect = 0;
	}
	
	return;
}

static int shtps_palm_host_detect_get_palm_state(struct shtps_rmi_spi *ts)
{
	return ts->host_palm_detect;
}
#endif /* SHTPS_PALM_HOST_DETECT_TOUCH_UP_ENABLE */

/* -----------------------------------------------------------------------------------
 */

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
int shtps_get_fingermax(struct shtps_rmi_spi *ts);

#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
static void shtps_delayed_facetouch_off_notify_wakelock(struct shtps_rmi_spi *ts, int on)
{
	if(on){
		wake_lock(&ts->facetouch.facetouch_off_delayed_wake_lock);
		pm_qos_update_request(&ts->facetouch.facetouch_off_delayed_qos, SHTPS_QOS_LATENCY_DEF_VALUE);
		SHTPS_LOG_DBG_PRINT("%s(): wake_lock\n", __func__);
	}else{
		wake_unlock(&ts->facetouch.facetouch_off_delayed_wake_lock);
		pm_qos_update_request(&ts->facetouch.facetouch_off_delayed_qos, PM_QOS_DEFAULT_VALUE);
		SHTPS_LOG_DBG_PRINT("%s(): wake_unlock\n", __func__);
	}
}

static enum hrtimer_restart shtps_delayed_facetouch_off_notify_function(struct hrtimer *timer)
{
	struct shtps_rmi_spi *ts = container_of(timer, struct shtps_rmi_spi, facetouch_off_notify_delayed_timer);

	SHTPS_LOG_FUNC_CALL();
	
	if(ts->facetouch.state != 0 || ts->facetouch.facetouch_off_force_flg != 0){
		shtps_facetouch_wakelock(ts, 1);
		ts->facetouch.state = 0;
		ts->facetouch.detect = 1;
		ts->facetouch.palm_det = 0;
		wake_up_interruptible(&ts->facetouch.wait_off);
		SHTPS_LOG_DBG_PRINT("face touch off detect. wake_up() (by facetouch off delay timer...)\n");
	}else{
		SHTPS_LOG_DBG_PRINT("touch off detect but pre-state isn't face touch.\n");
	}
	
	ts->facetouch.facetouch_off_notify_delayed_state = 0;
	ts->facetouch.facetouch_off_force_flg = 0;
	
	shtps_delayed_facetouch_off_notify_wakelock(ts, 0);
	
	return HRTIMER_NORESTART;
}

static void shtps_delayed_facetouch_off_notify(struct shtps_rmi_spi *ts, unsigned long delay_us)
{
	SHTPS_LOG_FUNC_CALL();
	if(ts->facetouch.facetouch_off_notify_delayed_state == 0){
		shtps_delayed_facetouch_off_notify_wakelock(ts, 1);
		
		ts->facetouch.palm_det = 1;
		hrtimer_cancel(&ts->facetouch_off_notify_delayed_timer);
		ts->facetouch.facetouch_off_notify_delayed_state = 1;
		hrtimer_start(&ts->facetouch_off_notify_delayed_timer, ktime_set(0, delay_us * 1000), HRTIMER_MODE_REL);
		SHTPS_FACETOUCH_OFF_DETECT_CHATT_PRINT("delayed facetouch_off notify.... [%lu]us", delay_us);
	}
}

static void shtps_delayed_facetouch_off_notify_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	if(ts->facetouch.facetouch_off_notify_delayed_state == 1){
		shtps_delayed_facetouch_off_notify_wakelock(ts, 0);
		
		ts->facetouch.facetouch_off_notify_delayed_state = 0;
		ts->facetouch.facetouch_off_force_flg = 0;
		SHTPS_FACETOUCH_OFF_DETECT_CHATT_PRINT("cancel facetouch_off notify timer. ");
		hrtimer_try_to_cancel(&ts->facetouch_off_notify_delayed_timer);
	}
	
	#if defined( SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE )
		ts->facetouch.rezero_disable = 0;
	#endif /* SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE */
	
}
#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */

#if defined( SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHANGE_ENABLE )
int shtps_set_motion_suppression(struct shtps_rmi_spi *ts, u8 x_suppression, u8 y_suppression){
	
	if(SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHG_ENABLE != 1){
		return 0;
	}
	
	SHTPS_LOG_DBG_PRINT("[shtps]change motion_suppression. X[%d] Y[%d]\n", x_suppression, y_suppression);
	return shtps_fwctl_set_motion_suppression(ts, x_suppression, y_suppression);
}

int shtps_set_motion_suppression_default(struct shtps_rmi_spi *ts){
	u8 def_val[2];
	shtps_fwctl_get_motion_suppression(ts, def_val);
	return shtps_set_motion_suppression(ts, def_val[0], def_val[1]);
}
#endif /* SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHANGE_ENABLE */

static int shtps_get_facetouchmode(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	return ts->facetouch.mode;
}

static void shtps_set_facetouchmode(struct shtps_rmi_spi *ts, int mode)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(mode);
	_log_msg_sync( LOGMSG_ID__SET_FACETOUCH_MODE, "%d", mode);
	ts->facetouch.mode = mode;
	if(mode == 0){
		
		#if defined( SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHANGE_ENABLE )
			shtps_set_motion_suppression_default(ts);
		#endif /* SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHANGE_ENABLE */
		
		ts->facetouch.detect = 0;
	}
}

static void shtps_facetouch_init(struct shtps_rmi_spi *ts)
{
	ts->facetouch.mode = 0;
	ts->facetouch.state = 0;
	ts->facetouch.detect = 0;
	ts->facetouch.palm_thresh = -1;
	ts->facetouch.wake_sig = 0;
	ts->facetouch.palm_det = 0;
	ts->facetouch.wakelock_state = 0;
	ts->facetouch.touch_num = 0;

	#if defined( SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE )
		ts->facetouch.rezero_disable = 1;
	#endif /* SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE */

	#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
		ts->facetouch.facetouch_off_notify_delayed_state = 0;
		ts->facetouch.facetouch_off_force_flg = 0;
	#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */

	init_waitqueue_head(&ts->facetouch.wait_off);
    wake_lock_init(&ts->facetouch.wake_lock, WAKE_LOCK_SUSPEND, "shtps_facetouch_wake_lock");
	pm_qos_add_request(&ts->facetouch.qos_cpu_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
}

void shtps_facetouch_wakelock(struct shtps_rmi_spi *ts, u8 on)
{
	if(on){
		if(ts->state_mgr.state == SHTPS_STATE_SLEEP_FACETOUCH){
			mutex_lock(&shtps_facetouch_qosctrl_lock);
			if(ts->facetouch.wakelock_state == 0){
				SHTPS_LOG_DBG_PRINT("face touch wake_lock on\n");
				wake_lock(&ts->facetouch.wake_lock);
				pm_qos_update_request(&ts->facetouch.qos_cpu_latency, SHTPS_QOS_LATENCY_DEF_VALUE);
				ts->facetouch.wakelock_state = 1;
			}
			mutex_unlock(&shtps_facetouch_qosctrl_lock);
		}
	}else{
		mutex_lock(&shtps_facetouch_qosctrl_lock);
		if(ts->facetouch.wakelock_state != 0){
			SHTPS_LOG_DBG_PRINT("face touch wake_lock off\n");
			wake_unlock(&ts->facetouch.wake_lock);
			pm_qos_update_request(&ts->facetouch.qos_cpu_latency, PM_QOS_DEFAULT_VALUE);
			ts->facetouch.wakelock_state = 0;
		}
		mutex_unlock(&shtps_facetouch_qosctrl_lock);
	}
}

static void shtps_event_all_cancel(struct shtps_rmi_spi *ts)
{
	int	i;
	int isEvent = 0;
	u8 	fingerMax = shtps_get_fingermax(ts);

	SHTPS_LOG_FUNC_CALL();

	for(i = 0;i < fingerMax;i++){
		if(ts->report_info.fingers[i].state != 0x00){
			isEvent = 1;
			shtps_report_touch_on(ts, i,
								  SHTPS_TOUCH_CANCEL_COORDINATES_X,
								  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
								  1,
								  ts->report_info.fingers[i].wx,
								  ts->report_info.fingers[i].wy,
								  ts->report_info.fingers[i].z);
		}
	}
	if(isEvent){
		input_sync(ts->input);
		
		for(i = 0;i < fingerMax;i++){
			if(ts->report_info.fingers[i].state != 0x00){
				isEvent = 1;
				shtps_report_touch_off(ts, i,
									  SHTPS_TOUCH_CANCEL_COORDINATES_X,
									  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
									  0,
									  ts->report_info.fingers[i].wx,
									  ts->report_info.fingers[i].wy,
									  ts->report_info.fingers[i].z);
			}
		}
		ts->touch_state.numOfFingers = 0;
		memset(&ts->report_info, 0, sizeof(ts->report_info));
	}
}

static int shtps_chck_palm(struct shtps_rmi_spi *ts)
{
	int i;
	u8 	fingerMax = shtps_get_fingermax(ts);
	for (i = 0; i < fingerMax; i++) {
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PALM){
			SHTPS_LOG_DBG_PRINT("Detect palm :id[%d]\n",i);
			return 1;
		}
	}
	return 0;
}

static void shtps_notify_facetouch(struct shtps_rmi_spi *ts)
{
	if(ts->facetouch.state == 0){
		shtps_facetouch_wakelock(ts, 1);
		ts->facetouch.state = 1;
		ts->facetouch.detect = 1;
		wake_up_interruptible(&ts->facetouch.wait_off);
		SHTPS_LOG_DBG_PRINT("face touch detect. wake_up()\n");
	}else{
		SHTPS_LOG_DBG_PRINT("face touch detect but pre-state isn't face off touch.\n");
	}
}

static void shtps_notify_facetouchoff(struct shtps_rmi_spi *ts, int force)
{
#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
	if(ts->facetouch.state != 0 || force != 0){
		if(ts->facetouch.facetouch_off_notify_delayed_state == 0){
			ts->facetouch.facetouch_off_force_flg = force;
			
			#if defined( SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE )
				ts->facetouch.rezero_disable = 1;
			#endif /* SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE */
			
			shtps_delayed_facetouch_off_notify(ts, SHTPS_FACETOUCH_OFF_DETECT_CHATT_THRESH_TIME);
		}else{
			SHTPS_FACETOUCH_OFF_DETECT_CHATT_PRINT("facetouch_off_notify watiting....");
		}
	}else{
		shtps_delayed_facetouch_off_notify_cancel(ts);
		SHTPS_FACETOUCH_OFF_DETECT_CHATT_PRINT("touch off detect but pre-state isn't face touch.\n");
	}
#else
	if(ts->facetouch.state != 0 || force != 0){
		shtps_facetouch_wakelock(ts, 1);
		ts->facetouch.state = 0;
		ts->facetouch.detect = 1;
		
		#if defined( SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE )
			ts->facetouch.rezero_disable = 1;
		#endif /* SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE */
		
		wake_up_interruptible(&ts->facetouch.wait_off);
		SHTPS_LOG_DBG_PRINT("face touch off detect(force flag = %d). wake_up()\n", force);
	}else{
		SHTPS_LOG_DBG_PRINT("touch off detect but pre-state isn't face touch.\n");
	}
#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */
}

static void shtps_check_facetouch(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 	palm_det  = 0;
	u8 	fingerMax = shtps_get_fingermax(ts);
	u8	numOfFingers = 0;

	SHTPS_LOG_DBG_PRINT("%s() gesture flag = 0x%02x\n", __func__, info->gs1);
	
//	if(SHTPS_FACETOUCH_DETECT_CHECK_PALMFLAG == 1 && (info->gs1 & 0x02) == 0x02){
	if(SHTPS_FACETOUCH_DETECT_CHECK_PALMFLAG == 1 && shtps_chck_palm(ts) == 1){
		SHTPS_LOG_DBG_PRINT("Detect palm touch by palm flag.\n");
		palm_det = 1;
	}else{
		for(i = 0;i < fingerMax;i++){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				if(SHTPS_FACETOUCH_DETECT_CHECK_FINGERWIDTH == 1){
					if( (ts->fw_report_info.fingers[i].wx >= SHTPS_FACETOUCH_DETECT_PALMDET_W_THRESHOLD) ||
						(ts->fw_report_info.fingers[i].wy >= SHTPS_FACETOUCH_DETECT_PALMDET_W_THRESHOLD) )
					{
						SHTPS_LOG_DBG_PRINT("Detect palm touch by finger width.\n");
						palm_det = 1;
						break;
					}
				}
				numOfFingers++;
			}
		}
	}
	
#if defined( SHTPS_PALM_HOST_DETECT_TOUCH_UP_ENABLE )
	if(SHTPS_PALM_DETECT_TOUCH_UP_ENABLE == 1 && (shtps_palm_host_detect_get_palm_state(ts) == 1)){
		palm_det = 1;
	}
#endif /* SHTPS_PALM_HOST_DETECT_TOUCH_UP_ENABLE */
	
	
	if(SHTPS_FACETOUCH_DETECT_CHECK_MULTITOUCH == 1 && numOfFingers >= SHTPS_FACETOUCH_DETECT_CHECK_MULTITOUCH_FINGER_NUM){
		SHTPS_LOG_DBG_PRINT("Detect palm touch by multi-touch.\n");
		palm_det = 1;
	}
	
	if(palm_det != 0 && ts->facetouch.state == 0){
		shtps_notify_facetouch(ts);
	}
	
	if(ts->facetouch.palm_det != 0){
		if(palm_det == 0 && ts->fw_report_info.finger_num == 0){
			ts->facetouch.palm_det = 0;
		}
	}else{
		ts->facetouch.palm_det = palm_det;
	}
}

static void shtps_check_facetouchoff(struct shtps_rmi_spi *ts)
{
	if(ts->facetouch.palm_det == 0 && ts->facetouch.state != 0){
		shtps_notify_facetouchoff(ts, 0);
	}
}

static void shtps_facetouch_forcecal_handle(struct shtps_rmi_spi *ts)
{
	ts->facetouch.palm_det = 0;
	if(ts->facetouch.state != 0){
		shtps_notify_facetouchoff(ts, 0);
	}
}

static void shtps_read_touchevent_infacetouchmode(struct shtps_rmi_spi *ts)
{
	u8 buf[2 + SHTPS_FINGER_MAX * 8];
	u8 i;
	struct shtps_touch_info info;
	u8 *pFingerInfoBuf;
	u8 irq_sts, ext_sts;
	u8 fingerMax = shtps_get_fingermax(ts);
	int numOfFingers = 0;

	SHTPS_LOG_FUNC_CALL();



	memset(buf, 0, sizeof(buf));
	shtps_fwctl_get_fingerinfo(ts, buf, 0, &irq_sts, &ext_sts, &pFingerInfoBuf);
	shtps_set_touch_info(ts, pFingerInfoBuf, &info);

/*
	ts->finger_state[0] = buf[2];
	if (fingerMax > 4){
		ts->finger_state[1] = buf[3];
	}
	if(fingerMax > 8){
		ts->finger_state[2] = buf[4];
	}
*/
	{
		u8 buf[2] = {0xFF};
		/* M_READ_FUNC( ts,ts->map.fn12.data.num[15].addr,buf, 2); */
		shtps_fwctl_get_ObjectAttention(ts, buf);
		ts->finger_state[0] = buf[0];
		if (fingerMax > 8){
			ts->finger_state[1] = buf[1];
		}
		SHTPS_LOG_DBG_PRINT("[debug]buf[0]:0x%02x  buf[1]:0x%02x\n", ts->finger_state[0], ts->finger_state[1]);
	}


#if defined( SHTPS_PALM_HOST_DETECT_TOUCH_UP_ENABLE )
	if(ts->state_mgr.state == SHTPS_STATE_SLEEP_FACETOUCH){
		shtps_palm_host_detect_touch_up(ts, &info, 0);
	}else{
		shtps_palm_host_detect_touch_up(ts, &info, 1);
	}
#endif /* SHTPS_PALM_HOST_DETECT_TOUCH_UP_ENABLE */

	for (i = 0; i < fingerMax; i++) {
		if(info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			numOfFingers++;
		}
	}
	info.finger_num = numOfFingers;
	
	shtps_fwctl_get_gesture(ts, fingerMax, buf, &info.gs1, &info.gs2);
	
//	SHTPS_LOG_DBG_PRINT("%s() numOfFingers = %d, palm det = 0x%02x\n", __func__, ts->fw_report_info.finger_num, buf[offset + fingerMax * 5]);
	
	shtps_check_facetouch(ts, &info);
	shtps_check_facetouchoff(ts);

	if(SHTPS_FACETOUCH_DETECT_OFF_NOTIFY_BY_ALLTU){
		if( (ts->fw_report_info_store.finger_num != 0) && (ts->fw_report_info.finger_num == 0) ){
			if(SHTPS_FACETOUCH_DETECT_CHECK_PALMFLAG == 1){
				if(ts->facetouch.palm_det == 0){
					shtps_notify_facetouchoff(ts, 1);
				}
			}else{
				shtps_notify_facetouchoff(ts, 1);
			}
		}
	}
	
	#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
		if(ts->facetouch.facetouch_off_notify_delayed_state == 1){
			if(ts->fw_report_info.finger_num > 0){
				shtps_delayed_facetouch_off_notify_cancel(ts);
				SHTPS_FACETOUCH_OFF_DETECT_CHATT_PRINT("Detect touch. cancel touch off timer");
			}
		}
	#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */
	
	ts->facetouch.touch_num = info.finger_num;
	
	memcpy(&ts->fw_report_info_store, &ts->fw_report_info, sizeof(ts->fw_report_info));
}

#if defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE )
static int shtps_is_lpmode(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		if(((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_NONE) ||
		   ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_LCD_BRIGHT) ||
		   ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_HOVER_OFF)){
			return 0;
		}else{
			return 1;
		}
	#else
		return 0;
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
}
#endif /* #if defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE ) */

#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */

/* -----------------------------------------------------------------------------------
 */
static irqreturn_t shtps_irq_handler(int irq, void *dev_id)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_START);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t shtps_irq(int irq, void *dev_id)
{
	struct shtps_rmi_spi	*ts = dev_id;

	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IRQ, 0) == 0){
			shtps_irq_proc(ts, 0);
		}else{
			shtps_mutex_lock_ctrl();
			ts->deter_suspend_spi.suspend_irq_state = ts->irq_mgr.state;
			ts->deter_suspend_spi.suspend_irq_wake_state = ts->irq_mgr.wake;
			ts->deter_suspend_spi.suspend_irq_detect = 1;
			SHTPS_LOG_DBG_PRINT("[suspend_spi] irq detect <irq_state:%d><irq_wake_state:%d>\n",
									ts->deter_suspend_spi.suspend_irq_state, ts->deter_suspend_spi.suspend_irq_wake_state);
			SHTPS_LOG_DBG_PRINT("[suspend_spi] irq disable\n");
			SHTPS_LOG_DBG_PRINT("[suspend_spi] irq wake disable\n");
			#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
				shtps_irq_disable(ts);
			#else
				shtps_irq_wake_disable(ts);
				shtps_irq_disable(ts);
			#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
			shtps_mutex_unlock_ctrl();
		}
	#else
		shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);

		shtps_wake_lock_idle(ts);	//SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE

		_log_msg_send( LOGMSG_ID__IRQ_NOTIFY, "");
		_log_msg_recv( LOGMSG_ID__IRQ_NOTIFY, "");

		#if defined(SHTPS_LPWG_MODE_ENABLE)
			if(0 != ts->lpwg.lpwg_switch && SHTPS_STATE_SLEEP == ts->state_mgr.state){
				shtps_lpwg_wakelock(ts, 1);
			}
		#endif /* defined(SHTPS_LPWG_MODE_ENABLE) */

		request_event(ts, SHTPS_EVENT_INTERRUPT, 1);

		#if defined(SHTPS_LPWG_MODE_ENABLE)
			shtps_lpwg_wakelock(ts, 0);
		#endif /* defined(SHTPS_LPWG_MODE_ENABLE) */

		shtps_wake_unlock_idle(ts);	//SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE

		shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_END);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return IRQ_HANDLED;
}

/* -----------------------------------------------------------------------------------
 */
static void shtps_work_tmof(struct work_struct *data)
{
	struct delayed_work  *dw = container_of(data, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, tmo_check);

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__TIMER_TIMEOUT, "");
	request_event(ts, SHTPS_EVENT_TIMEOUT, 0);
}

static enum hrtimer_restart shtps_delayed_rezero_timer_function(struct hrtimer *timer)
{
	struct shtps_rmi_spi *ts = container_of(timer, struct shtps_rmi_spi, rezero_delayed_timer);

	SHTPS_LOG_FUNC_CALL();
	_log_msg_send( LOGMSG_ID__DELAYED_REZERO_TRIGGER, "");
	schedule_work(&ts->rezero_delayed_work);
	return HRTIMER_NORESTART;
}

static void shtps_rezero_delayed_work_function(struct work_struct *work)
{
	struct shtps_rmi_spi *ts = container_of(work, struct shtps_rmi_spi, rezero_delayed_work);
	u8 rezero_exec = 0;

	SHTPS_LOG_FUNC_CALL();
	_log_msg_recv( LOGMSG_ID__DELAYED_REZERO_TRIGGER, "");
	shtps_mutex_lock_ctrl();

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if(ts->lpwg.block_touchevent != 0){
			if((ts->finger_state[0] | ts->finger_state[1] | ts->finger_state[2]) == 0){
				rezero_exec = 1;
				ts->lpwg.block_touchevent = 0;
				ts->lpwg.tu_rezero_req = 0;
				#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
					ts->wakeup_touch_event_inhibit_state = 0;
					SHTPS_LOG_WAKEUP_FAIL_TOUCH_EVENT_REJECT("TouchEvent Inhibit end by rezero exec\n");
				#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */
			}else{
				SHTPS_LOG_DBG_PRINT("LPWG delayed rezero fail by touch state\n");
				ts->lpwg.tu_rezero_req = 1;
			}
		}else{
			rezero_exec = 1;
			ts->lpwg.block_touchevent = 0;
			ts->lpwg.tu_rezero_req = 0;
			#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
				ts->wakeup_touch_event_inhibit_state = 0;
				SHTPS_LOG_WAKEUP_FAIL_TOUCH_EVENT_REJECT("TouchEvent Inhibit end by rezero exec\n");
			#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */
		}
	#else
		rezero_exec = 1;
		#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
			ts->wakeup_touch_event_inhibit_state = 0;
			SHTPS_LOG_WAKEUP_FAIL_TOUCH_EVENT_REJECT("TouchEvent Inhibit end by rezero exec\n");
		#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	if(rezero_exec){
		shtps_rezero(ts);
	}

	shtps_mutex_unlock_ctrl();
}

static void shtps_delayed_rezero(struct shtps_rmi_spi *ts, unsigned long delay_us)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__DELAYED_REZERO_SET, "%lu", delay_us);
	hrtimer_cancel(&ts->rezero_delayed_timer);
	hrtimer_start(&ts->rezero_delayed_timer, ktime_set(0, delay_us * 1000), HRTIMER_MODE_REL);
}

static void shtps_delayed_rezero_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__DELAYED_REZERO_CANCEL, "");
	hrtimer_try_to_cancel(&ts->rezero_delayed_timer);
}

void shtps_rezero(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_DBG_PRINT("fw rezero execute\n");
	shtps_fwctl_rezero(ts);
	shtps_event_force_touchup(ts);

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_facetouch_forcecal_handle(ts);
	#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */

	#if defined( SHTPS_MODULE_PARAM_ENABLE )
		shtps_rezero_state = 1;
	#endif /* SHTPS_MODULE_PARAM_ENABLE */
}

#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
static int shtps_is_singlefinger(struct shtps_rmi_spi *ts, u8 gs_info)
{
	SHTPS_LOG_FUNC_CALL();
	return shtps_fwctl_is_singlefinger(ts);
}

static void shtps_autorezero_disable(struct shtps_rmi_spi *ts)
{
	shtps_fwctl_autorezero_disable(ts);
}

static void shtps_autorezero_enable(struct shtps_rmi_spi *ts)
{
	shtps_fwctl_autorezero_enable(ts);
}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */

void shtps_rezero_request(struct shtps_rmi_spi *ts, u8 request, u8 trigger)
{
#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	int single_finger_count;

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__REZERO_REQUEST, "%d|%d", request, trigger);
	if(request & SHTPS_REZERO_REQUEST_WAKEUP_REZERO){
		shtps_delayed_rezero(ts, SHTPS_SLEEP_OUT_WAIT_US);
	}
	if(request & SHTPS_REZERO_REQUEST_REZERO){
		shtps_rezero(ts);
	}

	if(request & SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE){
		shtps_autorezero_disable(ts);
	}

	if(request & SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE){
		shtps_autorezero_enable(ts);
	}
#else
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__REZERO_REQUEST, "%d|%d", request, trigger);
	if(request & SHTPS_REZERO_REQUEST_WAKEUP_REZERO){
		#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
			if(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECT_REZERO_ENABLE != 0){
				ts->wakeup_touch_event_inhibit_state = 1;
				SHTPS_LOG_WAKEUP_FAIL_TOUCH_EVENT_REJECT("TouchEvent Inhibit start until rezero exec\n");
			}
		#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */
		shtps_delayed_rezero(ts, SHTPS_SLEEP_OUT_WAIT_US);
	}
	if(request & SHTPS_REZERO_REQUEST_REZERO){
		shtps_rezero(ts);
	}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */
}

static void shtps_rezero_handle(struct shtps_rmi_spi *ts, u8 event, u8 gs)
{
#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__REZERO_HANDLE, "%d|%d|%d", ts->poll_info.stop_margin, event, gs);
	if(!ts->poll_info.stop_margin){
		return;
	}

	if(event == SHTPS_REZERO_HANDLE_EVENT_MTD){
		ts->poll_info.single_fingers_enable = 0;

	}else if(event == SHTPS_REZERO_HANDLE_EVENT_TOUCH){
		if(ts->poll_info.single_fingers_enable && (shtps_is_singlefinger(ts, gs) == 1)){
			ts->poll_info.single_fingers_count++;
			SHTPS_LOG_DBG_PRINT("single finger count = %d\n",
									ts->poll_info.single_fingers_count);
		}

	}else if(event == SHTPS_REZERO_HANDLE_EVENT_TOUCHUP){
		ts->poll_info.single_fingers_enable = 1;
		if((++ts->poll_info.stop_count >= ts->poll_info.stop_margin) &&
			(ts->poll_info.single_fingers_count >= ts->poll_info.single_fingers_max))
		{
			shtps_autorezero_disable(ts);

			ts->poll_info.stop_margin          = 0;
			ts->poll_info.single_fingers_count = 0;
			ts->poll_info.single_fingers_enable= 0;
			ts->poll_info.single_fingers_max   = 0;
		}
	}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */
}

static void shtps_reset_startuptime(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->state_mgr.starttime = jiffies + msecs_to_jiffies(SHTPS_STARTUP_MIN_TIME);
}

static unsigned long shtps_check_startuptime(struct shtps_rmi_spi *ts)
{
	unsigned long remainingtime;
	SHTPS_LOG_FUNC_CALL();
	if(time_after(jiffies, ts->state_mgr.starttime)){
		return 0;
	}
	remainingtime = jiffies_to_msecs(ts->state_mgr.starttime - jiffies);
	if(remainingtime > SHTPS_STARTUP_MIN_TIME){
		remainingtime = SHTPS_STARTUP_MIN_TIME;
	}
	return remainingtime;
}

int shtps_start(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	return request_event(ts, SHTPS_EVENT_START, 0);
}

void shtps_shutdown(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	request_event(ts, SHTPS_EVENT_STOP, 0);
}

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_PEN_DETECT_ENABLE)
static int shtps_set_pen_enable(struct shtps_rmi_spi *ts, int onoff)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(onoff);

	if(onoff == 0){
		/* pen disable setting */
		shtps_fwctl_pen_disable(ts);
		ts->pen_enable = 0;
	}
	else if(onoff == 1){
		/* pen enable setting */
		shtps_fwctl_pen_enable(ts);
		ts->pen_enable = 1;
	}
	else{
		return -EINVAL;
	}
	return 0;
}

static void shtps_set_pen_detect_init(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_FACTORY_MODE_ENABLE)
		shtps_set_pen_enable(ts, 0);
	#else
		if(ts->pen_enable == 0){
			shtps_set_pen_enable(ts, 0);
		}
		else{
			shtps_set_pen_enable(ts, 1);
		}
	#endif /* SHTPS_FACTORY_MODE_ENABLE */
}
#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */

#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
static int shtps_map_construct(struct shtps_rmi_spi *ts, u8 func_check)
#else
static int shtps_map_construct(struct shtps_rmi_spi *ts)
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */
{
	int rc;
	SHTPS_LOG_FUNC_CALL();
#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	rc = shtps_fwctl_map_construct(ts, func_check);
#else
	rc = shtps_fwctl_map_construct(ts, 0);
#endif

	if(rc == 0){
		#if defined( SHTPS_VARIABLE_PEN_JITTER_ENABLE )
			shtps_filter_get_register_pen_jitter(ts);
		#endif /* SHTPS_VARIABLE_PEN_JITTER_ENABLE */

		#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
			shtps_filter_get_register_segmentation_aggressiveness(ts);
		#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */
	}
	return rc;
}

void shtps_reset(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_ANALYSIS("HW Reset execute\n");
	shtps_device_reset(ts->rst_pin);
}

#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
static u8 shtps_get_lpmode_state(struct shtps_rmi_spi *ts)
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

		return (req == SHTPS_LPMODE_REQ_NONE);
	#else
		return ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_NONE);
	#endif /* SHTPS_HOVER_DETECT_ENABLE */
}
#endif /* SHTPS_LOW_POWER_MODE_ENABLE */

void shtps_sleep(struct shtps_rmi_spi *ts, int on)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(on);

	if(on){
		shtps_delayed_rezero_cancel(ts);
		#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
			shtps_fwctl_set_doze(ts);
		#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */

		shtps_fwctl_set_sleepmode_on(ts);

		#if defined( SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE )
			if(SHTPS_SLEEP_IN_WAIT_MS > 0){
				msleep(SHTPS_SLEEP_IN_WAIT_MS);
			}
		#endif /* SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE */
	}else{
		#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
			#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
				if(shtps_get_lpmode_state(ts) != 0){
					shtps_fwctl_set_active(ts);
				}else{
					shtps_fwctl_set_doze(ts);
				}
			#else
				shtps_fwctl_set_active(ts);
			#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
		#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */
		shtps_fwctl_set_sleepmode_off(ts);

		#if defined( SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE )
			if(SHTPS_SLEEP_OUT_WAIT_MS > 0){
				msleep(SHTPS_SLEEP_OUT_WAIT_MS);
			}
		#endif /* SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE */
	}

	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		SHTPS_LOG_DBG_PRINT("[LPMODE]sleep request recieved. req_state = 0x%02x, continuous_req_state = 0x%02x\n",
								ts->lpmode_req_state, ts->lpmode_continuous_req_state);
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
}

int shtps_check_set_doze_enable(void)
{
	#if defined(SHTPS_LOW_POWER_MODE_CHEK_HW_REV_ENABLE)
		if(shtps_system_get_hw_type() != SHTPS_HW_TYPE_HANDSET){
			return 0;
		}else{
			{
				u8 hwrev;
				
				hwrev = shtps_system_get_hw_revision();
				
				if(SHTPS_GET_HW_VERSION_RET_ES_0 == hwrev ||
				   SHTPS_GET_HW_VERSION_RET_ES_1 == hwrev ||
				   SHTPS_GET_HW_VERSION_RET_ES_2 == hwrev 
				){
					return 0;
				}
			}
		}
	#endif /* SHTPS_LOW_POWER_MODE_CHEK_HW_REV_ENABLE */
	
	return 1;
}

#if defined( SHTPS_LOW_POWER_MODE_ENABLE ) || defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE ) || defined( SHTPS_LPWG_MODE_ENABLE )
static void shtps_set_doze_mode(struct shtps_rmi_spi *ts, int on)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(on);

	if(shtps_fwctl_is_sleeping(ts) != 0){
		return;
	}

	if(on){
		shtps_fwctl_set_doze(ts);
		SHTPS_LOG_DBG_PRINT("doze mode on\n");
	}else{
		shtps_fwctl_set_active(ts);
		SHTPS_LOG_DBG_PRINT("doze mode off\n");
	}
}
#endif /* defined( SHTPS_LOW_POWER_MODE_ENABLE ) || defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE ) || defined( SHTPS_LPWG_MODE_ENABLE ) */

#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
static void shtps_set_lpmode_init(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	if(shtps_get_lpmode_state(ts) != 0){
		shtps_set_doze_mode(ts, 0);
	}else{
		shtps_set_doze_mode(ts, 1);
	}

	SHTPS_LOG_DBG_PRINT("[LPMODE]lpmode init. req_state = 0x%02x, continuous_req_state = 0x%02x\n",
							ts->lpmode_req_state, ts->lpmode_continuous_req_state);
}

void shtps_set_lpmode(struct shtps_rmi_spi *ts, int type, int req, int on)
{
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
		if(shtps_get_lpmode_state(ts) != 0){
			shtps_set_doze_mode(ts, 0);
		}else{
			shtps_set_doze_mode(ts, 1);
		}
	}

	SHTPS_LOG_DBG_PRINT("[LPMODE]lpmode request recieved. req_state = 0x%02x, continuous_req_state = 0x%02x\n",
							ts->lpmode_req_state, ts->lpmode_continuous_req_state);
}
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

#if defined(SHTPS_HOVER_DETECT_ENABLE)
int shtps_set_hover_detect_enable(struct shtps_rmi_spi *ts)
{
	int rc;
	SHTPS_LOG_FUNC_CALL();
	rc = shtps_fwctl_hover_enable(ts);
	if(!rc){
		ts->hover_enable_state = 1;
	}
	return rc;
}

int shtps_set_hover_detect_disable(struct shtps_rmi_spi *ts)
{
	int rc;
	SHTPS_LOG_FUNC_CALL();
	rc = shtps_fwctl_hover_disable(ts);
	if(!rc){
		ts->hover_enable_state = 0;
	}
	return rc;
}

static int shtps_set_hover_detect_init(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->hover_ctrl_base_adr = SHTPS_HOVER_CTRL_BASE_ADR;

	if(ts->hover_enable_state == 0){
		shtps_set_hover_detect_disable(ts);
	}else{
		shtps_set_hover_detect_enable(ts);
	}

	return 0;
}
#endif /* SHTPS_HOVER_DETECT_ENABLE */

#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )
static void shtps_touchkey_inproxymity_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, touchkey_inproxymity_delayed_work);

	SHTPS_LOG_FUNC_CALL();
	
	shtps_mutex_lock_ctrl();
	if(ts->key_down_reserved){
		ts->key_down_ignored |= ts->key_down_reserved;
		ts->key_down_reserved = 0;
		SHTPS_LOG_DBG_PRINT("[key] proximity near. ignore key=0x%02x\n", ts->key_down_ignored);
	}
	shtps_mutex_unlock_ctrl();
}

static void shtps_touchkey_inproxymity_delayed_work_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->touchkey_inproxymity_delayed_work);
}

static void shtps_touchkey_inproxymity_delayed_work_start(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_touchkey_inproxymity_delayed_work_cancel(ts);
	schedule_delayed_work(&ts->touchkey_inproxymity_delayed_work, msecs_to_jiffies(SHTPS_KEY_PROXIMITY_DOWN_HOLD_TIME_MS));
}

static void shtps_touchkey_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, touchkey_delayed_work);
	int prox_data;
	int isEvent = 0;

	SHTPS_LOG_FUNC_CALL();
	
	prox_data = shtps_proximity_check(ts);


	shtps_mutex_lock_ctrl();
	
	if (prox_data == SHTPS_PROXIMITY_NEAR) {
		shtps_touchkey_inproxymity_delayed_work_start(ts);
	}
	else{
		if( (ts->key_down_reserved & (1 << SHTPS_PHYSICAL_KEY_DOWN)) != 0 ){
			input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_DOWN);
			input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_DOWN], 1);
			input_sync(ts->input_key);
			SHTPS_LOG_EVENT(
				DBG_PRINTK("[key]Notify event KEY_VOLUMEDOWN:DOWN\n");
			);
			isEvent = 1;

			ts->key_state |= (1 << SHTPS_PHYSICAL_KEY_DOWN);
			ts->key_down_reserved &= ~(1 << SHTPS_PHYSICAL_KEY_DOWN);
		}
		if( (ts->key_down_reserved & (1 << SHTPS_PHYSICAL_KEY_UP)) != 0 ){
			input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_UP);
			input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_UP], 1);
			input_sync(ts->input_key);
			SHTPS_LOG_EVENT(
				DBG_PRINTK("[key]Notify event KEY_VOLUMEUP:DOWN\n");
			);
			isEvent = 1;

			ts->key_state |= (1 << SHTPS_PHYSICAL_KEY_UP);
			ts->key_down_reserved &= ~(1 << SHTPS_PHYSICAL_KEY_UP);
		}

		if(isEvent){
			ts->diag.event_touchkey = 1;
			wake_up_interruptible(&ts->diag.wait);
		}
	}

	ts->key_proximity_check_state = 0;

	shtps_mutex_unlock_ctrl();

	SHTPS_LOG_DBG_PRINT("[TouchKey] proximity check end\n");
}

static void shtps_touchkey_delayed_work_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->touchkey_delayed_work);
	ts->key_proximity_check_state = 0;
}

static void shtps_touchkey_delayed_work_start(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	SHTPS_LOG_DBG_PRINT("[TouchKey] proximity check start\n");

	shtps_touchkey_delayed_work_cancel(ts);
	schedule_delayed_work(&ts->touchkey_delayed_work, msecs_to_jiffies(0));
	ts->key_proximity_check_state = 1;
}
#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */

#if defined(SHTPS_LPWG_MODE_ENABLE)
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

static void shtps_lpwg_notify_interval_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->lpwg.notify_interval_delayed_work);
}

static void shtps_lpwg_notify_interval_start(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_lpwg_notify_interval_stop(ts);
	schedule_delayed_work(&ts->lpwg.notify_interval_delayed_work, msecs_to_jiffies(SHTPS_LPWG_MIN_NOTIFY_INTERVAL));

	SHTPS_LOG_DBG_PRINT("[LPWG] notify interval start\n");
}

static void shtps_lpwg_wakelock_init(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	memset(&ts->lpwg, 0, sizeof(ts->lpwg));

	ts->lpwg.notify_enable = 1;
	ts->lpwg.lpwg_switch = 0;
	ts->lpwg.tu_rezero_req = 0;
	ts->lpwg.block_touchevent = 0;

	#if defined(SHTPS_LPWG_GRIP_SUPPORT_ENABLE)
		ts->lpwg.lpwg_state  = SHTPS_LPWG_STATE_GRIP_ONLY;
	#else
		ts->lpwg.lpwg_state  = SHTPS_LPWG_STATE_OFF;
	#endif /* SHTPS_LPWG_GRIP_SUPPORT_ENABLE */

	ts->lpwg_hover_enable_state_sotre = 0;
    wake_lock_init(&ts->lpwg.wake_lock, WAKE_LOCK_SUSPEND, "shtps_lpwg_wake_lock");
	pm_qos_add_request(&ts->lpwg.pm_qos_lock_idle, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	ts->lpwg.pm_qos_idle_value = SHTPS_LPWG_QOS_LATENCY_DEF_VALUE;
	INIT_DELAYED_WORK(&ts->lpwg.notify_interval_delayed_work, shtps_lpwg_notify_interval_delayed_work_function);
}

static void shtps_lpwg_wakelock_destroy(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
   wake_lock_destroy(&ts->lpwg.wake_lock);
   pm_qos_remove_request(&ts->lpwg.pm_qos_lock_idle);
}

static void shtps_lpwg_wakelock(struct shtps_rmi_spi *ts, int on)
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

static void shtps_lpwg_prepare(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_HOVER_DETECT_ENABLE)
		ts->lpwg_hover_enable_state_sotre = ts->hover_enable_state;
		shtps_set_hover_detect_disable(ts);
	#endif /* SHTPS_HOVER_DETECT_ENABLE */
}

static void shtps_lpwg_disposal(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_HOVER_DETECT_ENABLE)
		if(ts->lpwg_hover_enable_state_sotre == 0){
			shtps_set_hover_detect_disable(ts);
		}else{
			shtps_set_hover_detect_enable(ts);
		}

		ts->lpwg_hover_enable_state_sotre = 0;
	#endif /* SHTPS_HOVER_DETECT_ENABLE */
}

void shtps_set_lpwg_mode_on(struct shtps_rmi_spi *ts)
{
	if(ts->lpwg.lpwg_switch == 1){
		return;
	}

	SHTPS_LOG_FUNC_CALL();

	shtps_lpwg_notify_interval_stop(ts);
	ts->lpwg.notify_enable = 1;
	ts->lpwg.lpwg_switch = 1;
	ts->lpwg.tu_rezero_req = 0;
	ts->lpwg.block_touchevent = 0;

	shtps_lpwg_prepare(ts);

	#if defined( SHTPS_HOST_LPWG_MODE_ENABLE )
		if(shtps_check_host_lpwg_enable() == 1){
			int i;
			for(i = 0; i < SHTPS_FINGER_MAX; i++){
				ts->lpwg.pre_info.fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
				ts->lpwg.pre_info.fingers[i].x     = 0xFFFF;
				ts->lpwg.pre_info.fingers[i].y     = 0xFFFF;
			}
			ts->lpwg.swipe_check_time = 0;
		}
	#endif /* SHTPS_HOST_LPWG_MODE_ENABLE */

	shtps_fwctl_set_lpwg_mode_on(ts);

	SHTPS_LOG_DBG_PRINT("LPWG mode ON\n");
}

void shtps_set_lpwg_mode_off(struct shtps_rmi_spi *ts)
{
	if(ts->lpwg.lpwg_switch == 0){
		return;
	}
	SHTPS_LOG_FUNC_CALL();

	ts->lpwg.lpwg_switch = 0;

	shtps_lpwg_disposal(ts);

	shtps_fwctl_set_lpwg_mode_off(ts);

	SHTPS_LOG_DBG_PRINT("LPWG mode OFF\n");
}

static void shtps_set_lpwg_mode_cal(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_fwctl_set_lpwg_mode_cal(ts);
	if(SHTPS_LPWG_MODE_ON_AFTER_REZERO_ENABLE){
		shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_REZERO, 0);
	}
}

int shtps_is_lpwg_active(struct shtps_rmi_spi *ts)
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
#endif /* SHTPS_LPWG_MODE_ENABLE */

int shtps_fwdate(struct shtps_rmi_spi *ts, u8 *year, u8 *month)
{
	int rc;
	u8 retry = 3;

	SHTPS_LOG_FUNC_CALL();
	do{
		rc = shtps_fwctl_get_fwdate(ts, year, month);
		if(rc == 0)	break;
	}while(retry-- > 0);

	return rc;
}

int shtps_get_serial_number(struct shtps_rmi_spi *ts, u8 *buf)
{
	SHTPS_LOG_FUNC_CALL();
	return shtps_fwctl_get_serial_number(ts, buf);
}

u16 shtps_fwver(struct shtps_rmi_spi *ts)
{
	int rc;
	u16 ver = 0;
	u8 retry = 3;

	SHTPS_LOG_FUNC_CALL();
	do{
		rc = shtps_fwctl_get_fwver(ts, &ver);
		if(rc == 0)	break;
	}while(retry-- > 0);

	return ver;
}

u16 shtps_fwver_builtin(struct shtps_rmi_spi *ts)
{
	u16 ver = 0;

	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_MULTI_FW_ENABLE)
		ver = SHTPS_MULTI_FW_INFO_TBL[ts->multi_fw_type].ver;
		if(ver == 0){
			ver = SHTPS_FWVER_NEWER;
		}
	#else
		ver = SHTPS_FWVER_NEWER;
	#endif /* SHTPS_MULTI_FW_ENABLE */

	return ver;
}

int shtps_fwsize_builtin(struct shtps_rmi_spi *ts)
{
	int size = 0;

	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_MULTI_FW_ENABLE)
		size = SHTPS_MULTI_FW_INFO_TBL[ts->multi_fw_type].size;
		if(size == 0){
			size = SHTPS_FWSIZE_NEWER;
		}
	#else
		size = SHTPS_FWSIZE_NEWER;
	#endif /* SHTPS_MULTI_FW_ENABLE */

	return size;
}

unsigned char* shtps_fwdata_builtin(struct shtps_rmi_spi *ts)
{
	unsigned char *data = NULL;

	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_MULTI_FW_ENABLE)
		data = (unsigned char *)SHTPS_MULTI_FW_INFO_TBL[ts->multi_fw_type].data;
		if(data == NULL){
			data = (unsigned char *)tps_fw_data;
		}
	#else
		data = (unsigned char *)tps_fw_data;
	#endif /* SHTPS_MULTI_FW_ENABLE */

	return data;
}

#if defined(SHTPS_MULTI_FW_ENABLE)
static void shtps_get_multi_fw_type(struct shtps_rmi_spi *ts)
{
	ts->multi_fw_type = SHTPS_MULTI_FW_INFO_SIZE - 1;

	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_CHECK_HWID_ENABLE)
	{
		u8 i;
		u8 hwrev = SHTPS_GET_HW_VERSION_RET_MP;

		if(shtps_system_get_hw_type() != SHTPS_HW_TYPE_HANDSET){
			hwrev = SHTPS_GET_HW_VERSION_RET_ES_0;
		}else{
			hwrev = shtps_system_get_hw_revision();
		}
		
		for(i = 0;i < SHTPS_MULTI_FW_INFO_SIZE;i++){
			if(SHTPS_MULTI_FW_INFO_TBL[i].hwrev == hwrev){
				ts->multi_fw_type = i;
				break;
			}
		}
	}
	#endif /* SHTPS_CHECK_HWID_ENABLE */
}
#endif /* SHTPS_MULTI_FW_ENABLE */

static int shtps_init_param(struct shtps_rmi_spi *ts)
{
	int rc;

	_log_msg_sync( LOGMSG_ID__FW_INIT, "");
	SHTPS_LOG_FUNC_CALL();

	#if defined(SHTPS_MULTI_FW_ENABLE)
		shtps_get_multi_fw_type(ts);
		SHTPS_LOG_DBG_PRINT("multi fw type is %s\n", SHTPS_MULTI_FW_INFO_TBL[ts->multi_fw_type].name);
	#endif /* SHTPS_MULTI_FW_ENABLE */

	rc = shtps_fwctl_initparam(ts);
	SPI_ERR_CHECK(rc, err_exit);

	#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
		rc = shtps_fwctl_initparam_activemode(ts);
		SPI_ERR_CHECK(rc, err_exit);
	#else
		rc = shtps_fwctl_initparam_dozemode(ts);
		SPI_ERR_CHECK(rc, err_exit);
	#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */

	#if defined(SHTPS_PEN_DETECT_ENABLE)
		shtps_set_pen_detect_init(ts);
	#endif /* SHTPS_PEN_DETECT_ENABLE */

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		rc = shtps_fwctl_initparam_key(ts);
		SPI_ERR_CHECK(rc, err_exit);
	#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) */

	#if defined(SHTPS_HOVER_DETECT_ENABLE)
		shtps_set_hover_detect_init(ts);
	#endif /* SHTPS_HOVER_DETECT_ENABLE */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		shtps_fwctl_initparam_lpwgmode(ts);
		SPI_ERR_CHECK(rc, err_exit);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
		shtps_fwctl_initparam_autorezero(ts);
		SPI_ERR_CHECK(rc, err_exit);
	#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */

	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		shtps_filter_set_charger_armor_initparam(ts);
	#endif /* #if defined( SHTPS_CHARGER_ARMOR_ENABLE ) */

	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		shtps_set_lpmode_init(ts);
	#endif /* SHTPS_LOW_POWER_MODE_ENABLE */

	#if defined(SHTPS_LOW_REPORTRATE_MODE)
		rc = shtps_fwctl_initparam_reportrate(ts, ts->low_report_rate_mode_state);
		SPI_ERR_CHECK(rc, err_exit);
	#endif /* SHTPS_LOW_REPORTRATE_MODE */

	#if defined(SHTPS_CHANGE_LAND_LOCK_DISTANCE_ENABLE)
		rc = shtps_fwctl_initparam_land_lock_distance(ts);
		SPI_ERR_CHECK(rc, err_exit);
	#endif /* SHTPS_CHANGE_LAND_LOCK_DISTANCE_ENABLE */

	return 0;

err_exit:
	return -1;
}

static void shtps_standby_param(struct shtps_rmi_spi *ts)
{
	_log_msg_sync( LOGMSG_ID__FW_STANDBY, "");
	SHTPS_LOG_FUNC_CALL();

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		shtps_filter_absorption_hold_cancel(ts);
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE ) */

	#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
		ts->wakeup_touch_event_inhibit_state = 0;
	#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */

	shtps_sleep(ts, 1);
}

static void shtps_clr_startup_err(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->state_mgr.starterr = SHTPS_STARTUP_SUCCESS;
}

int shtps_wait_startup(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__FW_STARTUP_COMP_WAIT, "");
	wait_event_interruptible(ts->wait_start,
		ts->state_mgr.state == SHTPS_STATE_ACTIVE          ||
		ts->state_mgr.state == SHTPS_STATE_BOOTLOADER      ||
		ts->state_mgr.state == SHTPS_STATE_FWTESTMODE      ||
		ts->state_mgr.state == SHTPS_STATE_SLEEP           ||
		ts->state_mgr.state == SHTPS_STATE_FACETOUCH       ||
		ts->state_mgr.state == SHTPS_STATE_SLEEP_FACETOUCH ||
		ts->state_mgr.starterr == SHTPS_STARTUP_FAILED);

	_log_msg_recv( LOGMSG_ID__FW_STARTUP_COMP, "%d|%d", ts->state_mgr.state, ts->state_mgr.starterr);

	return ts->state_mgr.starterr;
}

static void shtps_notify_startup(struct shtps_rmi_spi *ts, u8 err)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(err);
	ts->state_mgr.starterr = err;
	_log_msg_send( LOGMSG_ID__FW_STARTUP_COMP, "%d|%d", ts->state_mgr.state, ts->state_mgr.starterr);
	wake_up_interruptible(&ts->wait_start);
}

static void shtps_set_startmode(struct shtps_rmi_spi *ts, u8 mode)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(mode);
	_log_msg_sync( LOGMSG_ID__FW_STARTUP_MODE, "%d", mode);
	ts->state_mgr.mode = mode;
}

static int shtps_get_startmode(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	return ts->state_mgr.mode;
}

int shtps_get_fingermax(struct shtps_rmi_spi *ts)
{
	return shtps_fwctl_get_fingermax(ts);
}

/* -----------------------------------------------------------------------------------
 */
int shtps_get_diff(unsigned short pos1, unsigned short pos2, unsigned long factor)
{
	int diff;
//	SHTPS_LOG_FUNC_CALL();
	diff = (pos1 * factor / 10000) - (pos2 * factor / 10000);
	return (diff >= 0)? diff : -diff;
}

int shtps_get_fingerwidth(struct shtps_rmi_spi *ts, int num, struct shtps_touch_info *info)
{
	int w = (info->fingers[num].wx >= info->fingers[num].wy)? info->fingers[num].wx : info->fingers[num].wy;
//	SHTPS_LOG_FUNC_CALL();
	#if defined( SHTPS_FINGER_WIDTH_MODERATION_ENABLE )
	if (!SHTPS_FINGER_WIDTH_MODERATION_DISABLE){
		if (ts->report_info.fingers[num].state == SHTPS_TOUCH_STATE_FINGER &&
			info->fingers[num].state == SHTPS_TOUCH_STATE_FINGER) {
			int dwx = info->fingers[num].wx - ts->report_info.fingers[num].wx;
			int dwy = info->fingers[num].wy - ts->report_info.fingers[num].wy;
			if (dwx > SHTPS_FINGER_WIDTH_GAIN_THRESHOLD ||
				dwy > SHTPS_FINGER_WIDTH_GAIN_THRESHOLD) {
				ts->w_before_gain[num] = (ts->report_info.fingers[num].wx >= ts->report_info.fingers[num].wy) ?
					ts->report_info.fingers[num].wx : ts->report_info.fingers[num].wy;
				SHTPS_LOG_DBG_PRINT("finger width moderation start ([%d] w0=%d)\n", num, ts->w_before_gain[num]);
			}
			if (ts->w_before_gain[num]) {
				if (w > ts->w_before_gain[num]) {
					w = (w - ts->w_before_gain[num]) / SHTPS_FINGER_WIDTH_MODERATION_RATIO + ts->w_before_gain[num];
				} else {
					ts->w_before_gain[num] = 0;
					SHTPS_LOG_DBG_PRINT("finger width moderation end\n");
				}
			}
		} else if (ts->report_info.fingers[num].state == SHTPS_TOUCH_STATE_NO_TOUCH) {
			ts->w_before_gain[num] = 0;
		}
	}
	#endif	/* SHTPS_FINGER_WIDTH_MODERATION_ENABLE */

	return (w < SHTPS_FINGER_WIDTH_MIN)? SHTPS_FINGER_WIDTH_MIN : w;
}

void shtps_set_eventtype(u8 *event, u8 type)
{
//	SHTPS_LOG_FUNC_CALL();
	*event = type;
}

static void shtps_set_touch_info(struct shtps_rmi_spi *ts, u8 *buf, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	u8* fingerInfo;
	int FingerNum = 0;

	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE)
		shtps_filter_set_touch_info_hover_detect_init(ts);
	#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE */

	for(i=0;i<fingerMax;i++){
		//fingerInfo = &buf[i * 8];
		fingerInfo = shtps_fwctl_get_finger_info_buf(ts, i, fingerMax, buf);

		ts->fw_report_info.fingers[i].state	= shtps_fwctl_get_finger_state(ts, i, fingerMax, buf);
		ts->fw_report_info.fingers[i].x		= shtps_fwctl_get_finger_pos_x(ts, fingerInfo);
		ts->fw_report_info.fingers[i].y		= shtps_fwctl_get_finger_pos_y(ts, fingerInfo);
		ts->fw_report_info.fingers[i].wx	= shtps_fwctl_get_finger_wx(ts, fingerInfo);
		ts->fw_report_info.fingers[i].wy	= shtps_fwctl_get_finger_wy(ts, fingerInfo);
		ts->fw_report_info.fingers[i].z		= shtps_fwctl_get_finger_z(ts, fingerInfo);

		info->fingers[i].state	= ts->fw_report_info.fingers[i].state;
		info->fingers[i].x		= ts->fw_report_info.fingers[i].x;
		info->fingers[i].y		= ts->fw_report_info.fingers[i].y;
		info->fingers[i].wx		= ts->fw_report_info.fingers[i].wx;
		info->fingers[i].wy		= ts->fw_report_info.fingers[i].wy;
		info->fingers[i].z		= ts->fw_report_info.fingers[i].z;

		if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			FingerNum++;
			SHTPS_LOG_EVENT(
				DBG_PRINTK("[%s]Touch Info[%d] x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
					ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER ? "Finger" :
					ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN ? "Pen" :
					ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_HOVER ? "Hover" :
					ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PALM ? "Palm" :
					ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_UNKNOWN ? "Unknown" : "Other" ,
					i,
					ts->fw_report_info.fingers[i].x,
					ts->fw_report_info.fingers[i].y,
					ts->fw_report_info.fingers[i].wx,
					ts->fw_report_info.fingers[i].wy,
					ts->fw_report_info.fingers[i].z
				);
			);

			#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE)
				shtps_filter_set_touch_info_hover_detect(ts, info, i);
			#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE */
		}
		else if(ts->fw_report_info_store.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			SHTPS_LOG_EVENT(
				DBG_PRINTK("[NoTouch]Touch Info[%d] x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
					i,
					ts->fw_report_info.fingers[i].x,
					ts->fw_report_info.fingers[i].y,
					ts->fw_report_info.fingers[i].wx,
					ts->fw_report_info.fingers[i].wy,
					ts->fw_report_info.fingers[i].z
				);
			);
		}
	}
	
	ts->fw_report_info.finger_num = FingerNum;
	
	#if defined(SHTPS_FINGER_KEY_EXCLUDE_ENABLE)
	{
		unsigned long now_time = jiffies;
		u8 is_finger_disable_check = 0;

		if(ts->exclude_touch_disable_check_state != 0){
			if( time_after(now_time, ts->exclude_touch_disable_check_time) != 0 ){
				ts->exclude_touch_disable_check_state = 0;
			}else{
				is_finger_disable_check = 1;
			}
		}

		if(is_finger_disable_check != 0){
			for(i = 0; i < fingerMax; i++){
				if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
					if( (info->fingers[i].y > (CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1 - SHTPS_KEY_DISABLE_EFFECT_AREA)) ){
						ts->exclude_touch_disable_finger |= (1 << i);
						SHTPS_LOG_DBG_PRINT("[exclude]finger disable check correspond [%d]\n", i);
					}
				}
			}
		}

		for(i = 0; i < fingerMax; i++){
			if( ((ts->exclude_touch_disable_finger >> i) & 0x01) != 0 ){
				if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
					info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
					SHTPS_LOG_DBG_PRINT("[exclude]finger [%d] disable\n", i);
				}else{
					ts->exclude_touch_disable_finger &= ~(1 << i);
				}
			}
		}

		for(i = 0; i < fingerMax; i++){
			if( (info->fingers[i].state != SHTPS_TOUCH_STATE_FINGER) &&
				(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) )
			{
				if( (ts->report_info.fingers[i].y > (CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1 - SHTPS_KEY_DISABLE_EFFECT_AREA)) ){
					ts->exclude_key_disable_check_state = 1;
					ts->exclude_key_disable_check_time = now_time + msecs_to_jiffies(SHTPS_KEY_DISABLE_TIME_MS);
				}
			}
		}
	}
	#endif /* SHTPS_FINGER_KEY_EXCLUDE_ENABLE */
}


static void shtps_calc_notify(struct shtps_rmi_spi *ts, u8 *buf, struct shtps_touch_info *info, u8 *event)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int numOfFingers = 0;

	SHTPS_LOG_FUNC_CALL();

	shtps_set_eventtype(event, 0xff);
	shtps_set_touch_info(ts, buf, info);


#if defined( SHTPS_PALM_HOST_DETECT_TOUCH_UP_ENABLE )
	shtps_palm_host_detect_touch_up(ts, info, 1);
#endif /* SHTPS_PALM_HOST_DETECT_TOUCH_UP_ENABLE */


	shtps_filter_main(ts, info);

	for (i = 0; i < fingerMax; i++) {
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			numOfFingers++;
		}
	}
	info->finger_num = numOfFingers;

	shtps_filter_drag(ts, info, event);				//SHTPS_DRAG_STEP_ENABLE
	shtps_fwctl_get_gesture(ts, fingerMax, buf, &info->gs1, &info->gs2);

	if(numOfFingers > 0){
		ts->poll_info.stop_count = 0;
		if(ts->touch_state.numOfFingers == 0){
			shtps_set_eventtype(event, SHTPS_EVENT_TD);
		}
		if(numOfFingers >= 2 && ts->touch_state.numOfFingers < 2){
			shtps_rezero_handle(ts, SHTPS_REZERO_HANDLE_EVENT_MTD, info->gs2);
		}
		shtps_rezero_handle(ts, SHTPS_REZERO_HANDLE_EVENT_TOUCH, info->gs2);
	}else{
		if(ts->touch_state.numOfFingers != 0){
			shtps_set_eventtype(event, SHTPS_EVENT_TU);
		}
		shtps_rezero_handle(ts, SHTPS_REZERO_HANDLE_EVENT_TOUCHUP, info->gs2);
	}

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		if(ts->state_mgr.state == SHTPS_STATE_FACETOUCH){
			shtps_check_facetouch(ts, info);
		}
	#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */

	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE)
		shtps_filter_set_hover_detect_tu_timer(ts, info);
	#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE */

}


void shtps_report_touch_on(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z)
{
	int lcd_x;
	int lcd_y;

//	SHTPS_LOG_FUNC_CALL();
	if( (x == SHTPS_TOUCH_CANCEL_COORDINATES_X) && (y == SHTPS_TOUCH_CANCEL_COORDINATES_Y) ){
		lcd_x = x;
		lcd_y = y;
	}else{
		lcd_x = x * SHTPS_POS_SCALE_X(ts) / 10000;
		lcd_y = y * SHTPS_POS_SCALE_Y(ts) / 10000;
	}

	shtps_filter_offset_pos(ts, &lcd_x, &lcd_y);

	input_mt_slot(ts->input, finger);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input, ABS_MT_POSITION_X,  lcd_x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y,  lcd_y);
	input_report_abs(ts->input, ABS_MT_PRESSURE,    z);

	SHTPS_LOG_EVENT(
		DBG_PRINTK("[finger]Notify event[%d] touch=100(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
						finger, z, lcd_x, x, lcd_y, y, w, wx, wy);
	);
	_log_msg_sync( LOGMSG_ID__EVENT_NOTIFY, "%d|100|%d|%d|%d|%d|%d|%d|%d|%d",
						finger, lcd_x, x, lcd_y, y, w, wx, wy, z);

	#if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE )
		if(shtps_touch_eventlog_is_recording() != 0){
			shtps_touch_eventlog_rec_event_info(SHTPS_TOUCH_STATE_FINGER, finger, x, y, w, wx, wy, z);
		}
	#endif /* #if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE ) */
}

void shtps_report_touch_off(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z)
{
//	SHTPS_LOG_FUNC_CALL();
	input_mt_slot(ts->input, finger);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);

	SHTPS_LOG_EVENT(
		DBG_PRINTK("[finger]Notify event[%d] touch=0(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
						finger, z, (x * SHTPS_POS_SCALE_X(ts) / 10000), x, (y * SHTPS_POS_SCALE_Y(ts) / 10000), y,
						w, wx, wy);
	);
	_log_msg_sync( LOGMSG_ID__EVENT_NOTIFY, "%d|0|%d|%d|%d|%d|%d|%d|%d|%d",
						finger, (x * SHTPS_POS_SCALE_X(ts) / 10000), x, (y * SHTPS_POS_SCALE_Y(ts) / 10000), y,
						w, wx, wy, z);

	#if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE )
		if(shtps_touch_eventlog_is_recording() != 0){
			shtps_touch_eventlog_rec_event_info(SHTPS_TOUCH_STATE_NO_TOUCH, finger, x, y, w, wx, wy, z);
		}
	#endif /* #if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE ) */
}

#if defined(SHTPS_PEN_DETECT_ENABLE)
void shtps_report_touch_pen_on(struct shtps_rmi_spi *ts, int pen, int x, int y, int w, int wx, int wy, int z)
{
	int lcd_x;
	int lcd_y;

//	SHTPS_LOG_FUNC_CALL();
	if( (x == SHTPS_TOUCH_CANCEL_COORDINATES_X) && (y == SHTPS_TOUCH_CANCEL_COORDINATES_Y) ){
		lcd_x = x;
		lcd_y = y;
	}else{
		lcd_x = x * SHTPS_POS_SCALE_X(ts) / 10000;
		lcd_y = y * SHTPS_POS_SCALE_Y(ts) / 10000;
	}

	shtps_filter_offset_pos_pen(ts, &lcd_x, &lcd_y);

	#if defined(SHTPS_PEN_SCREEN_EDGE_DISABLE)
	{
		u8 pen_screen_edge_check = 1;

		if((x == SHTPS_TOUCH_CANCEL_COORDINATES_X) && (y == SHTPS_TOUCH_CANCEL_COORDINATES_Y) ){
			pen_screen_edge_check = 0;
		}

		if(SHTPS_HOST_EVENT_BLOCK_EDGE_PEN_ENABLE == 0){
			pen_screen_edge_check = 0;
		}

		if(pen_screen_edge_check != 0){
			if(lcd_x < SHTPS_PEN_EDGE_DISABLE_RANGE_X){
				SHTPS_LOG_DBG_PRINT("[PEN_SCREEN_EDGE_DISABLE][%d] x = %d -> %d\n", pen, lcd_x, SHTPS_PEN_EDGE_DISABLE_RANGE_X);
				lcd_x = SHTPS_PEN_EDGE_DISABLE_RANGE_X;
			}else if(lcd_x > (CONFIG_SHTPS_SY3000_PANEL_SIZE_X - SHTPS_PEN_EDGE_DISABLE_RANGE_X)){
				SHTPS_LOG_DBG_PRINT("[PEN_SCREEN_EDGE_DISABLE][%d] x = %d -> %d\n", pen, lcd_x, CONFIG_SHTPS_SY3000_PANEL_SIZE_X - SHTPS_PEN_EDGE_DISABLE_RANGE_X);
				lcd_x = CONFIG_SHTPS_SY3000_PANEL_SIZE_X - SHTPS_PEN_EDGE_DISABLE_RANGE_X;
			}
			if(lcd_y < SHTPS_PEN_EDGE_DISABLE_RANGE_Y){
				SHTPS_LOG_DBG_PRINT("[PEN_SCREEN_EDGE_DISABLE][%d] y = %d -> %d\n", pen, lcd_y, SHTPS_PEN_EDGE_DISABLE_RANGE_Y);
				lcd_y = SHTPS_PEN_EDGE_DISABLE_RANGE_Y;
			}else if(lcd_y > (CONFIG_SHTPS_SY3000_PANEL_SIZE_Y - SHTPS_PEN_EDGE_DISABLE_RANGE_Y)){
				SHTPS_LOG_DBG_PRINT("[PEN_SCREEN_EDGE_DISABLE][%d] y = %d -> %d\n", pen, lcd_y, CONFIG_SHTPS_SY3000_PANEL_SIZE_Y - SHTPS_PEN_EDGE_DISABLE_RANGE_Y);
				lcd_y = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y - SHTPS_PEN_EDGE_DISABLE_RANGE_Y;
			}
		}
	}
	#endif /* SHTPS_PEN_SCREEN_EDGE_DISABLE */

	input_mt_slot(ts->input, pen);
	input_mt_report_slot_state(ts->input, MT_TOOL_PEN, true);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input, ABS_MT_POSITION_X,  lcd_x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y,  lcd_y);
	input_report_abs(ts->input, ABS_MT_PRESSURE,    z);

	SHTPS_LOG_EVENT(
		DBG_PRINTK("[pen]Notify event[%d] touch=100(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
						pen, z, lcd_x, x, lcd_y, y, w, wx, wy);
	);

	#if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE )
		if(shtps_touch_eventlog_is_recording() != 0){
			shtps_touch_eventlog_rec_event_info(SHTPS_TOUCH_STATE_PEN, pen, x, y, w, wx, wy, z);
		}
	#endif /* #if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE ) */
}

void shtps_report_touch_pen_off(struct shtps_rmi_spi *ts, int pen, int x, int y, int w, int wx, int wy, int z)
{
//	SHTPS_LOG_FUNC_CALL();
	input_mt_slot(ts->input, pen);
	input_mt_report_slot_state(ts->input, MT_TOOL_PEN, false);

	SHTPS_LOG_EVENT(
		DBG_PRINTK("[pen]Notify event[%d] touch=0(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
						pen, z, (x * SHTPS_POS_SCALE_X(ts) / 10000), x, (y * SHTPS_POS_SCALE_Y(ts) / 10000), y,
						w, wx, wy);
	);

	#if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE )
		if(shtps_touch_eventlog_is_recording() != 0){
			shtps_touch_eventlog_rec_event_info(SHTPS_TOUCH_STATE_NO_TOUCH, pen, x, y, w, wx, wy, z);
		}
	#endif /* #if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE ) */
}
#endif /* SHTPS_PEN_DETECT_ENABLE */

#if defined(SHTPS_HOVER_DETECT_ENABLE)
void shtps_report_touch_hover_on(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z)
{
	int lcd_x;
	int lcd_y;

//	SHTPS_LOG_FUNC_CALL();
	if( (x == SHTPS_TOUCH_CANCEL_COORDINATES_X) && (y == SHTPS_TOUCH_CANCEL_COORDINATES_Y) ){
		lcd_x = x;
		lcd_y = y;
	}else{
		lcd_x = x * SHTPS_POS_SCALE_X(ts) / 10000;
		lcd_y = y * SHTPS_POS_SCALE_Y(ts) / 10000;
	}

	shtps_filter_offset_pos(ts, &lcd_x, &lcd_y);

	input_mt_slot(ts->input, finger);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input, ABS_MT_POSITION_X,  lcd_x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y,  lcd_y);
	input_report_abs(ts->input, ABS_MT_PRESSURE,    0);

	SHTPS_LOG_EVENT(
		DBG_PRINTK("[hover]Notify event[%d] touch=100(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
						finger, z, lcd_x, x, lcd_y, y, w, wx, wy);
	);
	_log_msg_sync( LOGMSG_ID__EVENT_NOTIFY, "%d|100|%d|%d|%d|%d|%d|%d|%d|%d",
						finger, lcd_x, x, lcd_y, y, w, wx, wy, z);

	#if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE )
		if(shtps_touch_eventlog_is_recording() != 0){
			shtps_touch_eventlog_rec_event_info(SHTPS_TOUCH_STATE_HOVER, finger, x, y, w, wx, wy, z);
		}
	#endif /* #if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE ) */
}

void shtps_report_touch_hover_off(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z)
{
	input_mt_slot(ts->input, finger);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);

//	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_EVENT(
		DBG_PRINTK("[hover]Notify event[%d] touch=0(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
						finger, z, (x * SHTPS_POS_SCALE_X(ts) / 10000), x, (y * SHTPS_POS_SCALE_Y(ts) / 10000), y,
						w, wx, wy);
	);
	_log_msg_sync( LOGMSG_ID__EVENT_NOTIFY, "%d|0|%d|%d|%d|%d|%d|%d|%d|%d",
						finger, (x * SHTPS_POS_SCALE_X(ts) / 10000), x, (y * SHTPS_POS_SCALE_Y(ts) / 10000), y,
						w, wx, wy, z);

	#if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE )
		if(shtps_touch_eventlog_is_recording() != 0){
			shtps_touch_eventlog_rec_event_info(SHTPS_TOUCH_STATE_NO_TOUCH, finger, x, y, w, wx, wy, z);
		}
	#endif /* #if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE ) */
}
#endif /* SHTPS_HOVER_DETECT_ENABLE */

void shtps_event_report(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 event)
{
	int	i;
	int fingerMax = shtps_get_fingermax(ts);

	#if defined(SHTPS_GHOST_REJECTION_ENABLE)
		int	ghost = shtps_filter_detect_ghost(ts, info);
	#endif	/* SHTPS_GHOST_REJECTION_ENABLE */

	SHTPS_LOG_FUNC_CALL();

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		if(shtps_filter_absorption_event_report(ts, info))	return;
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */

	#if defined( SHTPS_COAXIAL_GHOST_REJECTION_ENABLE )
		shtps_filter_coaxial_ghost_event_report(ts, info, event);
	#endif /* SHTPS_COAXIAL_GHOST_REJECTION_ENABLE */

	for(i = 0;i < fingerMax;i++){
		#if defined(SHTPS_GHOST_REJECTION_ENABLE)
			if(((ghost >> i) & 0x01) != 0){
				SHTPS_LOG_DBG_PRINT("[GHOST_REJECT][%d] is ignored by ghost judge\n", i);
				continue;
			}
		#endif	/* SHTPS_GHOST_REJECTION_ENABLE */

		#if defined(SHTPS_REPORT_TOOL_TYPE_LOCK_ENABLE)
			if(SHTPS_HOST_EVENT_TOOL_TYPE_LOCK_ENABLE != 0){
				if( (info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER) &&
					(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN) )
				{
					info->fingers[i].state = SHTPS_TOUCH_STATE_PEN;
					SHTPS_LOG_DBG_PRINT("[TOOL_TYPE_LOCK] Finger -> Pen changed\n");
				}
				else if( (info->fingers[i].state == SHTPS_TOUCH_STATE_PEN) &&
							(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) )
				{
					info->fingers[i].state = SHTPS_TOUCH_STATE_FINGER;
					SHTPS_LOG_DBG_PRINT("[TOOL_TYPE_LOCK] Pen -> Finger changed\n");
				}
			}
		#endif /* SHTPS_REPORT_TOOL_TYPE_LOCK_ENABLE */

		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			shtps_report_touch_on(ts, i,
								  info->fingers[i].x,
								  info->fingers[i].y,
								  shtps_get_fingerwidth(ts, i, info),
								  info->fingers[i].wx,
								  info->fingers[i].wy,
								  info->fingers[i].z);

		#if defined(SHTPS_HOVER_DETECT_ENABLE)
			}else if(info->fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
				shtps_report_touch_hover_on(ts, i,
									  info->fingers[i].x,
									  info->fingers[i].y,
									  shtps_get_fingerwidth(ts, i, info),
									  info->fingers[i].wx,
									  info->fingers[i].wy,
									  info->fingers[i].z);
		#endif /* SHTPS_HOVER_DETECT_ENABLE */

		#if defined(SHTPS_PEN_DETECT_ENABLE)
			}else if(info->fingers[i].state == SHTPS_TOUCH_STATE_PEN){
				shtps_report_touch_pen_on(ts, i,
									  info->fingers[i].x,
									  info->fingers[i].y,
									  shtps_get_fingerwidth(ts, i, info),
									  info->fingers[i].wx,
									  info->fingers[i].wy,
									  info->fingers[i].z);
			}else if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN){
				shtps_report_touch_pen_off(ts, i,
									  info->fingers[i].x,
									  info->fingers[i].y,
									  shtps_get_fingerwidth(ts, i, info),
									  info->fingers[i].wx,
									  info->fingers[i].wy,
									  info->fingers[i].z);
		#endif /* SHTPS_PEN_DETECT_ENABLE */

		#if defined(SHTPS_HOVER_DETECT_ENABLE)
			}else if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
				shtps_report_touch_hover_off(ts, i,
									  info->fingers[i].x,
									  info->fingers[i].y,
									  shtps_get_fingerwidth(ts, i, info),
									  info->fingers[i].wx,
									  info->fingers[i].wy,
									  info->fingers[i].z);
		#endif /* SHTPS_HOVER_DETECT_ENABLE */

		}else if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			shtps_report_touch_off(ts, i,
								  info->fingers[i].x,
								  info->fingers[i].y,
								  shtps_get_fingerwidth(ts, i, info),
								  info->fingers[i].wx,
								  info->fingers[i].wy,
								  info->fingers[i].z);
		}
	}
	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);
	input_sync(ts->input);
	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);

	ts->touch_state.numOfFingers = info->finger_num;

	ts->diag.event = 1;
	memcpy(&ts->report_info, info, sizeof(ts->report_info));

	shtps_perflock_set_event(ts, event);	//SHTPS_CPU_CLOCK_CONTROL_ENABLE

	wake_up_interruptible(&ts->diag.wait);
}

#if defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_event_update(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int numOfFingers = 0;

	SHTPS_LOG_FUNC_CALL();

	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			numOfFingers++;
		}
	}
	ts->touch_state.numOfFingers = numOfFingers;
}
#endif /* #if defined(SHTPS_LPWG_MODE_ENABLE) */

void shtps_event_force_touchup(struct shtps_rmi_spi *ts)
{
	int	i;
	int isEvent = 0;
	int fingerMax = shtps_get_fingermax(ts);

	SHTPS_LOG_FUNC_CALL();

	#if defined(SHTPS_MOTION_PEDO_STOP_REQ_ENABLE)
		shub_api_restart_pedometer_func(SHUB_STOP_PED_TYPE_TPS);
	#endif /* SHTPS_MOTION_PEDO_STOP_REQ_ENABLE */

	shtps_filter_force_touchup(ts);

	#if defined( SHTPS_TOUCHCANCEL_BEFORE_FORCE_TOUCHUP_ENABLE )
		for(i = 0;i < fingerMax;i++){
			if(ts->report_info.fingers[i].state != 0x00){
				isEvent = 1;
				shtps_report_touch_on(ts, i,
									  SHTPS_TOUCH_CANCEL_COORDINATES_X,
									  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
									  shtps_get_fingerwidth(ts, i, &ts->report_info),
									  ts->report_info.fingers[i].wx,
									  ts->report_info.fingers[i].wy,
									  ts->report_info.fingers[i].z);
			}
		}
		if(isEvent){
			input_sync(ts->input);
			isEvent = 0;
		}
	#endif /* SHTPS_TOUCHCANCEL_BEFORE_FORCE_TOUCHUP_ENABLE */

	for(i = 0;i < fingerMax;i++){
		#if defined(SHTPS_HOVER_DETECT_ENABLE)
			if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
				isEvent = 1;
				shtps_report_touch_hover_off(ts, i,
									  ts->report_info.fingers[i].x,
									  ts->report_info.fingers[i].y,
									  0,
									  0,
									  0,
									  0);
			}else
		#endif /* SHTPS_HOVER_DETECT_ENABLE */

		#if defined(SHTPS_PEN_DETECT_ENABLE)
			if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN){
				isEvent = 1;
				shtps_report_touch_pen_off(ts, i,
									  ts->report_info.fingers[i].x,
									  ts->report_info.fingers[i].y,
									  0,
									  0,
									  0,
									  0);
			}else
		#endif /* SHTPS_PEN_DETECT_ENABLE */

		if(ts->report_info.fingers[i].state != 0x00){
			isEvent = 1;
			shtps_report_touch_off(ts, i,
								  ts->report_info.fingers[i].x,
								  ts->report_info.fingers[i].y,
								  0,
								  0,
								  0,
								  0);
		}
	}
	if(isEvent){
		input_sync(ts->input);
		ts->touch_state.numOfFingers = 0;
		memset(&ts->report_info, 0, sizeof(ts->report_info));
	}
}

#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )

#if defined(SHTPS_LOG_DEBUG_ENABLE) || defined(SHTPS_LOG_EVENT_ENABLE)
static char* shtps_get_key_name(int key)
{
	SHTPS_LOG_FUNC_CALL();
	switch(key){
		case SHTPS_PHYSICAL_KEY_UP:
			return "KEY_VOLUMEUP";
		case SHTPS_PHYSICAL_KEY_DOWN:
			return "KEY_VOLUMEDOWN";
	}
	return "KEY_UNKNOWN";
}
#endif /* SHTPS_LOG_DEBUG_ENABLE || SHTPS_LOG_EVENT_ENABLE */

static int shtps_key_event_report_each_key(struct shtps_rmi_spi *ts, u8 state, int key)
{
	int isEvent = 0;
	int isReportedKeyDownState = (ts->key_state & (1 << key) ? 1 : 0);
	int isReserveKeyDownState = (ts->key_down_reserved & (1 << key) ? 1 : 0);
	int isKeyDownState = (state & (1 << key) ? 1 : 0);

	SHTPS_LOG_FUNC_CALL();
	if((ts->key_down_ignored & (1 << key)) != 0){
		if(isKeyDownState == 0){
			ts->key_down_ignored &= ~(1 << key);
		}

		SHTPS_LOG_DBG_PRINT("[TouchKey] %s ignored", shtps_get_key_name(key));

		return isEvent;
	}

	if((isReportedKeyDownState == 0) && (isKeyDownState == 1)){
		if(ts->key_proximity_check_state == 0){
			shtps_touchkey_delayed_work_start(ts);
		}
		ts->key_down_reserved |= (1 << key);
	}
	else if((isReportedKeyDownState == 1) && (isKeyDownState == 0)){
		input_event(ts->input_key, EV_MSC, MSC_SCAN, key);
		input_report_key(ts->input_key, ts->keycodes[key], 0);
		input_sync(ts->input_key);
		SHTPS_LOG_EVENT(
			DBG_PRINTK("[key]Notify event %s:UP\n", shtps_get_key_name(key));
		);
		isEvent = 1;

		ts->key_down_reserved &= ~(1 << key);
		ts->key_state &= ~(1 << key);
	}
	else if((isReserveKeyDownState == 1) && (isKeyDownState == 0)){
		input_event(ts->input_key, EV_MSC, MSC_SCAN, key);
		input_report_key(ts->input_key, ts->keycodes[key], 1);
		input_sync(ts->input_key);
		SHTPS_LOG_EVENT(
			DBG_PRINTK("[key]Notify event %s:DOWN\n", shtps_get_key_name(key));
		);
		input_event(ts->input_key, EV_MSC, MSC_SCAN, key);
		input_report_key(ts->input_key, ts->keycodes[key], 0);
		input_sync(ts->input_key);
		SHTPS_LOG_EVENT(
			DBG_PRINTK("[key]Notify event %s:UP\n", shtps_get_key_name(key));
		);
		isEvent = 1;

		ts->key_down_reserved &= ~(1 << key);
		ts->key_state &= ~(1 << key);
	}

	return isEvent;
}
#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */

static void shtps_key_event_report(struct shtps_rmi_spi *ts, u8 state)
{
	#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )
		int isEvent = 0;

		SHTPS_LOG_FUNC_CALL();

		isEvent |= shtps_key_event_report_each_key(ts, state, SHTPS_PHYSICAL_KEY_DOWN);
		isEvent |= shtps_key_event_report_each_key(ts, state, SHTPS_PHYSICAL_KEY_UP);

		if(isEvent){
			ts->diag.event_touchkey = 1;
			wake_up_interruptible(&ts->diag.wait);
		}
	#else /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */
		int isEvent = 0;

		SHTPS_LOG_FUNC_CALL();

		if(ts->key_state != state){
			#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
			if (SHTPS_KEY_PROXIMITY_SUPPORT_ENABLE &&
				(	/* whether key down bits on */
					((~(ts->key_state & (1 << SHTPS_PHYSICAL_KEY_DOWN)) & (state & (1 << SHTPS_PHYSICAL_KEY_DOWN))) != 0) ||
					((~(ts->key_state & (1 << SHTPS_PHYSICAL_KEY_UP)) & (state & (1 << SHTPS_PHYSICAL_KEY_UP))) != 0)
					)
				) {
				int prox_data;
				SHTPS_LOG_DBG_PRINT("[key] proximity check start\n");
				prox_data = shtps_proximity_check(ts);
				SHTPS_LOG_DBG_PRINT("[key] proximity check end\n");
				if (prox_data == SHTPS_PROXIMITY_NEAR) {
					SHTPS_LOG_DBG_PRINT("[key] proximity near\n");
					/* clear key down bits forcedly */
					state &= ~(1 << SHTPS_PHYSICAL_KEY_DOWN);
					state &= ~(1 << SHTPS_PHYSICAL_KEY_UP);
				}
			}
			#endif	/* SHTPS_PROXIMITY_SUPPORT_ENABLE */
			if( ((ts->key_state & (1 << SHTPS_PHYSICAL_KEY_DOWN)) ^ (state & (1 << SHTPS_PHYSICAL_KEY_DOWN))) != 0 ){
				input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_DOWN);
				input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_DOWN], ((state >> SHTPS_PHYSICAL_KEY_DOWN) & 0x01));
				isEvent = 1;
				SHTPS_LOG_EVENT(
					DBG_PRINTK("[key]Notify event KEY_VOLUMEDOWN:%s\n", (((state >> SHTPS_PHYSICAL_KEY_DOWN) & 0x01) ? "DOWN" : "UP"));
				);
			}
			if( ((ts->key_state & (1 << SHTPS_PHYSICAL_KEY_UP)) ^ (state & (1 << SHTPS_PHYSICAL_KEY_UP))) != 0 ){
				input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_UP);
				input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_UP], ((state >> SHTPS_PHYSICAL_KEY_UP) & 0x01));
				isEvent = 1;
				SHTPS_LOG_EVENT(
					DBG_PRINTK("[key]Notify event KEY_VOLUMEUP:%s\n", (((state >> SHTPS_PHYSICAL_KEY_UP) & 0x01) ? "DOWN" : "UP"));
				);
			}
		}

		if(isEvent){
			input_sync(ts->input_key);
			ts->key_state = state;
			ts->diag.event_touchkey = 1;
			wake_up_interruptible(&ts->diag.wait);
		}
	#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */
}

static void shtps_key_event_force_touchup(struct shtps_rmi_spi *ts)
{
	int isEvent = 0;

	SHTPS_LOG_FUNC_CALL();

	if(ts->key_state != 0){
		if( (ts->key_state & (1 << SHTPS_PHYSICAL_KEY_DOWN)) != 0 ){
			input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_DOWN);
			input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_DOWN], 0);
			isEvent = 1;
			SHTPS_LOG_EVENT(
				DBG_PRINTK("[key]Notify event KEY_VOLUMEDOWN:UP\n");
			);
		}
		if( (ts->key_state & (1 << SHTPS_PHYSICAL_KEY_UP)) != 0 ){
			input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_UP);
			input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_UP], 0);
			isEvent = 1;
			SHTPS_LOG_EVENT(
				DBG_PRINTK("[key]Notify event KEY_VOLUMEUP:UP\n");
			);
		}
	}

	if(isEvent){
		input_sync(ts->input_key);
	}

	ts->key_state = 0;

	#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )
		ts->key_down_reserved = 0;
		ts->key_down_ignored = 0;
		shtps_touchkey_delayed_work_cancel(ts);
		shtps_touchkey_inproxymity_delayed_work_cancel(ts);
	#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */
}
#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

#if defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_notify_wakeup_event(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_DBG_PRINT("wakeup touch blocked until rezero\n");

	ts->lpwg.block_touchevent = 1;
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
static int shtps_tm_irqcheck(struct shtps_rmi_spi *ts)
{
	int rc;
	u8 sts;

	SHTPS_LOG_FUNC_CALL();
	rc = shtps_fwctl_irqclear_get_irqfactor(ts, &sts);
	if((rc == 0) && ((sts & SHTPS_IRQ_ANALOG) != 0x00)){
		return 1;

	}else{
		return 0;

	}
}

static int shtps_tm_wait_attn(struct shtps_rmi_spi *ts)
{
	int rc;

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_ATTN_WAIT, "");
	rc = wait_event_interruptible_timeout(ts->diag.tm_wait_ack,
			ts->diag.tm_ack == 1 || ts->diag.tm_stop == 1,
			msecs_to_jiffies(SHTPS_FWTESTMODE_ACK_TMO));
	_log_msg_recv( LOGMSG_ID__FW_TESTMODE_ATTN, "");

#if defined( SHTPS_LOG_SEQ_ENABLE )
	if(rc == 0){
		_log_msg_sync( LOGMSG_ID__FW_TESTMODE_ATTN_TIMEOUT, "");
		SHTPS_LOG_ERR_PRINT("shtps_tm_wait_attn() :ATTN_TIMEOUT\n");
	}
#endif /* #if defined( SHTPS_LOG_SEQ_ENABLE ) */

	if(ts->diag.tm_ack == 0 || ts->diag.tm_stop == 1){
		SHTPS_LOG_ERR_PRINT("shtps_tm_wait_attn() tm_ack:%d, tm_stop:%d\n", ts->diag.tm_ack, ts->diag.tm_stop);
		return -1;
	}

	ts->diag.tm_ack = 0;
	return 0;
}

static void shtps_tm_wakeup(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->diag.tm_ack = 1;
	_log_msg_send( LOGMSG_ID__FW_TESTMODE_ATTN, "");
	wake_up_interruptible(&ts->diag.tm_wait_ack);
}

static void shtps_tm_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->diag.tm_stop = 1;

	wake_up_interruptible(&ts->diag.tm_wait_ack);
}

int shtps_get_tm_rxsize(struct shtps_rmi_spi *ts)
{
	return shtps_fwctl_get_tm_rxsize(ts);
}

int shtps_get_tm_txsize(struct shtps_rmi_spi *ts)
{
	return shtps_fwctl_get_tm_txsize(ts);
}

static int shtps_start_tm(struct shtps_rmi_spi *ts)
{
	int rc;

	SHTPS_LOG_FUNC_CALL();

	ts->diag.tm_stop = 0;
	ts->diag.tm_ack = 0;

	#if defined(SHTPS_HOVER_DETECT_ENABLE)
	{
		u8 cur_hover_state = ts->hover_enable_state;

		shtps_set_hover_detect_enable(ts);
		msleep(SHTPS_FORCECAL_AFTER_HOVERSETTING_WAIT);

		ts->hover_enable_state = cur_hover_state;
	}
	#endif /* SHTPS_HOVER_DETECT_ENABLE */

	rc = shtps_fwctl_start_testmode(ts, ts->diag.tm_mode);

	return rc;
}

int shtps_baseline_offset_disable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	return shtps_fwctl_baseline_offset_disable(ts);
}

static void shtps_stop_tm(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_tm_cancel(ts);
	shtps_fwctl_stop_testmode(ts);
	shtps_init_param(ts);
}

void shtps_read_tmdata(struct shtps_rmi_spi *ts, u8 mode)
{
	int rc;
	SHTPS_LOG_FUNC_CALL();

	if(mode == SHTPS_TMMODE_FRAMELINE){
		shtps_mutex_lock_ctrl();
		rc = shtps_fwctl_cmd_tm_frameline(ts, ts->diag.tm_mode);
		shtps_mutex_unlock_ctrl();
		if(rc == 0){
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;
		}
		shtps_mutex_lock_ctrl();
		rc = shtps_fwctl_get_tm_frameline(ts, ts->diag.tm_mode, ts->diag.tm_data);
		shtps_mutex_unlock_ctrl();

	}else if(mode == SHTPS_TMMODE_BASELINE){
		shtps_mutex_lock_ctrl();
		rc = shtps_fwctl_cmd_tm_baseline(ts, ts->diag.tm_mode);
		shtps_mutex_unlock_ctrl();
		if(rc == 0){
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;
		}
		shtps_mutex_lock_ctrl();
		rc = shtps_fwctl_get_tm_baseline(ts, ts->diag.tm_mode, ts->diag.tm_data);
		shtps_mutex_unlock_ctrl();

	}else if(mode == SHTPS_TMMODE_BASELINE_RAW){
		shtps_mutex_lock_ctrl();
		rc = shtps_fwctl_cmd_tm_baseline_raw(ts, ts->diag.tm_mode);
		shtps_mutex_unlock_ctrl();
		if(rc == 0){
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;
		}
		shtps_mutex_lock_ctrl();
		rc = shtps_fwctl_get_tm_baseline_raw(ts, ts->diag.tm_mode, ts->diag.tm_data);
		shtps_mutex_unlock_ctrl();

	}else{
		memset(ts->diag.tm_data, 0, sizeof(ts->diag.tm_data));
	}
	return;

tm_read_cancel:
	memset(ts->diag.tm_data, 0, sizeof(ts->diag.tm_data));
	return;
}

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
	//nothing
#else
static void shtps_irq_wake_disable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_WAKE_DISABLE, "%d", ts->irq_mgr.wake);
	if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_DISABLE){
		disable_irq_wake(ts->irq_mgr.irq);
		ts->irq_mgr.wake = SHTPS_IRQ_WAKE_DISABLE;

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			shtps_irq_wake_state = SHTPS_IRQ_WAKE_DISABLE;
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	}
}

static void shtps_irq_wake_enable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_WAKE_ENABLE, "%d", ts->irq_mgr.wake);
	if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_ENABLE){
		enable_irq_wake(ts->irq_mgr.irq);
		ts->irq_mgr.wake = SHTPS_IRQ_WAKE_ENABLE;

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			shtps_irq_wake_state = SHTPS_IRQ_WAKE_ENABLE;
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	}
}
#endif /* !SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

void shtps_irq_disable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_DISABLE, "%d", ts->irq_mgr.state);
	if(ts->irq_mgr.state != SHTPS_IRQ_STATE_DISABLE){
		disable_irq_nosync(ts->irq_mgr.irq);
		ts->irq_mgr.state = SHTPS_IRQ_STATE_DISABLE;
	}
	
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_DISABLE){
			disable_irq_wake(ts->irq_mgr.irq);
			ts->irq_mgr.wake = SHTPS_IRQ_WAKE_DISABLE;

			#if defined( SHTPS_MODULE_PARAM_ENABLE )
				shtps_irq_wake_state = SHTPS_IRQ_WAKE_DISABLE;
			#endif /* SHTPS_MODULE_PARAM_ENABLE */
		}
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
}

void shtps_irq_enable(struct shtps_rmi_spi *ts)	
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_ENABLE, "%d", ts->irq_mgr.state);
	
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_ENABLE){
			enable_irq_wake(ts->irq_mgr.irq);
			ts->irq_mgr.wake = SHTPS_IRQ_WAKE_ENABLE;

			#if defined( SHTPS_MODULE_PARAM_ENABLE )
				shtps_irq_wake_state = SHTPS_IRQ_WAKE_ENABLE;
			#endif /* SHTPS_MODULE_PARAM_ENABLE */
		}
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	if(ts->irq_mgr.state != SHTPS_IRQ_STATE_ENABLE){
		enable_irq(ts->irq_mgr.irq);
		ts->irq_mgr.state = SHTPS_IRQ_STATE_ENABLE;
	}
}

static int shtps_irq_resuest(struct shtps_rmi_spi *ts)
{
	int rc;

	SHTPS_LOG_FUNC_CALL();

	_log_msg_sync( LOGMSG_ID__IRQ_REQUEST, "%d", ts->irq_mgr.irq);

	#if defined(SHTPS_IRQ_LEVEL_ENABLE)
		rc = request_threaded_irq(ts->irq_mgr.irq,
							  shtps_irq_handler,
							  shtps_irq,
							  IRQF_TRIGGER_LOW | IRQF_ONESHOT,
							  SH_TOUCH_DEVNAME,
							  ts);
	#else
		rc = request_threaded_irq(ts->irq_mgr.irq,
							  shtps_irq_handler,
							  shtps_irq,
							  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							  SH_TOUCH_DEVNAME,
							  ts);
	#endif /* SHTPS_IRQ_LEVEL_ENABLE */

	if(rc){
		_log_msg_sync( LOGMSG_ID__IRQ_REQUEST_NACK, "");
		SHTPS_LOG_ERR_PRINT("request_threaded_irq error:%d\n",rc);
		return -1;
	}

	ts->irq_mgr.state = SHTPS_IRQ_STATE_ENABLE;
	ts->irq_mgr.wake  = SHTPS_IRQ_WAKE_DISABLE;
	shtps_irq_disable(ts);
	return 0;
}

static void shtps_irqtimer_start(struct shtps_rmi_spi *ts, long time_ms)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_TIMER_START, "%lu", time_ms);
	schedule_delayed_work(&ts->tmo_check, msecs_to_jiffies(time_ms));
}

static void shtps_irqtimer_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_TIMER_CANCEL, "");
	cancel_delayed_work(&ts->tmo_check);
}

#if defined(SHTPS_HOST_LPWG_MODE_ENABLE)
int shtps_check_host_lpwg_enable(void)
{
	
	if(SHTPS_HOST_LPWG_HW_REV_CHK_ENABLE == 0){
		return SHTPS_HOST_LPWG_ENABLE;
	}else{
		if(shtps_system_get_hw_type() != SHTPS_HW_TYPE_HANDSET){
			return 1;
		}else{
			{
				u8 hwrev;
				
				hwrev = shtps_system_get_hw_revision();
				
				if(SHTPS_GET_HW_VERSION_RET_ES_0 == hwrev ||
				   SHTPS_GET_HW_VERSION_RET_ES_1 == hwrev ||
				   SHTPS_GET_HW_VERSION_RET_ES_2 == hwrev 
				){
					return 1;
				}
			}
		}
	}
	return 0;
}
#endif /* SHTPS_HOST_LPWG_MODE_ENABLE */

#if defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_read_touchevent_insleep(struct shtps_rmi_spi *ts, int state)
{
	#if defined( SHTPS_HOST_LPWG_MODE_ENABLE )
		struct shtps_touch_info info;
		u8* fingerInfo;
		unsigned short diff_x;
		unsigned short diff_y;
	#endif /* SHTPS_HOST_LPWG_MODE_ENABLE */
	u8 irq_sts;
	u8 val = 0;

	SHTPS_LOG_FUNC_CALL();

	shtps_fwctl_irqclear_get_irqfactor(ts, &irq_sts);

	#if defined( SHTPS_HOST_LPWG_MODE_ENABLE )
	{
		u8 buf[2 + 8];
		if(shtps_check_host_lpwg_enable() == 1){
			int fingerMax = shtps_get_fingermax(ts);
			memset(buf, 0, sizeof(buf));
			shtps_fwctl_get_one_fingerinfo(ts, 0, buf, &fingerInfo);

			info.fingers[0].state	= shtps_fwctl_get_finger_state(ts, 0, fingerMax, fingerInfo);
			info.fingers[0].x		= (shtps_fwctl_get_finger_pos_x(ts, fingerInfo) * SHTPS_POS_SCALE_X(ts) / 10000);
			info.fingers[0].y		= (shtps_fwctl_get_finger_pos_y(ts, fingerInfo) * SHTPS_POS_SCALE_Y(ts) / 10000);

			SHTPS_LOG_DBG_PRINT("[LPWG] TouchInfo <state: %d><x: %d><y: %d>\n", info.fingers[0].state, info.fingers[0].x, info.fingers[0].y);

			if( (info.fingers[0].state == SHTPS_TOUCH_STATE_FINGER) && (ts->lpwg.pre_info.fingers[0].state == SHTPS_TOUCH_STATE_NO_TOUCH) ){
				ts->lpwg.pre_info.fingers[0].state = info.fingers[0].state;
				ts->lpwg.pre_info.fingers[0].x     = info.fingers[0].x;
				ts->lpwg.pre_info.fingers[0].y     = info.fingers[0].y;
				ts->lpwg.swipe_check_time = jiffies + msecs_to_jiffies(SHTPS_LPWG_SWIPE_CHECK_TIME_MS);

				SHTPS_LOG_DBG_PRINT("[LPWG] swipe check zero pos <x: %d><y: %d>\n", ts->lpwg.pre_info.fingers[0].x, ts->lpwg.pre_info.fingers[0].y);
			}
			else if( (info.fingers[0].state == SHTPS_TOUCH_STATE_FINGER) && (ts->lpwg.pre_info.fingers[0].state == SHTPS_TOUCH_STATE_FINGER) ){
				if( time_after(jiffies, ts->lpwg.swipe_check_time) == 0 ){
					diff_x = (info.fingers[0].x >= ts->lpwg.pre_info.fingers[0].x) ?
								(info.fingers[0].x - ts->lpwg.pre_info.fingers[0].x) :
								(ts->lpwg.pre_info.fingers[0].x - info.fingers[0].x);

					diff_y = (info.fingers[0].y >= ts->lpwg.pre_info.fingers[0].y) ?
								(info.fingers[0].y - ts->lpwg.pre_info.fingers[0].y) :
								(ts->lpwg.pre_info.fingers[0].y - info.fingers[0].y);

					SHTPS_LOG_DBG_PRINT("[LPWG] swipe distance (%d, %d)\n", diff_x, diff_y);

					if( (diff_x > SHTPS_LPWG_SWIPE_DIST_THRESHOLD) || (diff_y > SHTPS_LPWG_SWIPE_DIST_THRESHOLD) ){
						val = SHTPS_LPWG_DETECT_GESTURE_TYPE_SWIPE;
					}
				}
				else{
					ts->lpwg.pre_info.fingers[0].x = info.fingers[0].x;
					ts->lpwg.pre_info.fingers[0].y = info.fingers[0].y;
					ts->lpwg.swipe_check_time = jiffies + msecs_to_jiffies(SHTPS_LPWG_SWIPE_CHECK_TIME_MS);

					SHTPS_LOG_DBG_PRINT("[LPWG] swipe check zero pos update<x: %d><y: %d>\n", ts->lpwg.pre_info.fingers[0].x, ts->lpwg.pre_info.fingers[0].y);
				}
			}
		}
		else{
			shtps_fwctl_get_gesturetype(ts, &val);
			SHTPS_LOG_DBG_PRINT("LPWG detect <0x%02X><%s>\n", val,
									(val == SHTPS_LPWG_DETECT_GESTURE_TYPE_NONE) ? "None" :
									(val == SHTPS_LPWG_DETECT_GESTURE_TYPE_SWIPE) ? "Swipe" :
									(val == SHTPS_LPWG_DETECT_GESTURE_TYPE_DOUBLE_TAP) ? "Double Tap" : "Unkown");
		}
	}
	#else
		shtps_fwctl_get_gesturetype(ts, &val);
		SHTPS_LOG_DBG_PRINT("LPWG detect <0x%02x><%s>\n", val,
								(val == SHTPS_LPWG_DETECT_GESTURE_TYPE_NONE) ? "None" :
								(val == SHTPS_LPWG_DETECT_GESTURE_TYPE_SWIPE) ? "Swipe" :
								(val == SHTPS_LPWG_DETECT_GESTURE_TYPE_DOUBLE_TAP) ? "Double Tap" : "Unkown");
	#endif /* SHTPS_HOST_LPWG_MODE_ENABLE */

	if((val & SHTPS_LPWG_DETECT_GESTURE_TYPE_SWIPE) != 0){
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
	}else{
		SHTPS_LOG_DBG_PRINT("LPWG: Not Support Gesture Type Detect\n");\
	}

	#if defined( SHTPS_HOST_LPWG_MODE_ENABLE )
		if(shtps_check_host_lpwg_enable() != 0){
			if(info.fingers[0].state != SHTPS_TOUCH_STATE_FINGER){
				ts->lpwg.pre_info.fingers[0].state = SHTPS_TOUCH_STATE_NO_TOUCH;
				ts->lpwg.pre_info.fingers[0].x     = 0xFFFF;
				ts->lpwg.pre_info.fingers[0].y     = 0xFFFF;
				ts->lpwg.swipe_check_time = 0;
			}
		}
	#endif /* SHTPS_HOST_LPWG_MODE_ENABLE */
}
#endif /*  SHTPS_LPWG_MODE_ENABLE */

void shtps_read_touchevent(struct shtps_rmi_spi *ts, int state)
{
	u8 buf[2 + SHTPS_FINGER_MAX * 8];
	u8 event;
	u8 irq_sts, ext_sts;
	u8 *pFingerInfoBuf;
	struct shtps_touch_info info;

	#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
		u8 is_hold_keyevent = 0;
		u8 present_key_state = 0;
	#endif /* SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__READ_EVENT, "%d", state);

	memset(buf, 0, sizeof(buf));
	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);
	shtps_fwctl_get_fingerinfo(ts, buf, 0, &irq_sts, &ext_sts, &pFingerInfoBuf);
	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);

	#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
		if(ts->wakeup_touch_event_inhibit_state != 0){
			memset(pFingerInfoBuf, 0, sizeof(buf) - 2);
			SHTPS_LOG_WAKEUP_FAIL_TOUCH_EVENT_REJECT("TouchEvent Cleared by not rezero exec yet\n");
		}
	#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */

	switch(state){
	case SHTPS_STATE_SLEEP:
	case SHTPS_STATE_IDLE:
		break;

	case SHTPS_STATE_SLEEP_FACETOUCH:
		break;

	case SHTPS_STATE_FACETOUCH:
	case SHTPS_STATE_ACTIVE:
	default:
		#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
			if((irq_sts & SHTPS_IRQ_BUTTON) != 0){
				int fingerMax = shtps_get_fingermax(ts);

				#if !defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
					u8 present_key_state = 0;
				#endif /* !SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */

				shtps_fwctl_get_keystate(ts, &present_key_state);

				#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
					if((present_key_state & 0xFC) != 0){
						SHTPS_LOG_DBG_PRINT("[shtpskey] Detect invalid value(0x%02X). Set to zero.\n", present_key_state);
						present_key_state = 0;
					}
				#endif /* SHTPS_PDT_READ_RETRY_ENABLE */

				#if defined(SHTPS_FINGER_KEY_EXCLUDE_ENABLE)
					if( (ts->key_state == 0) && (present_key_state != 0) )
					{
						int i;
						u8 is_key_disable = 0;

						for(i = 0;i < fingerMax;i++){
							if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
								if( (ts->report_info.fingers[i].y > (CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1 - SHTPS_KEY_DISABLE_EFFECT_AREA)) ){
									is_key_disable = 1;
									SHTPS_LOG_DBG_PRINT("[exclude]key down disable (finger[%d] event pos <%d, %d>)\n",
															i, ts->report_info.fingers[i].x, ts->report_info.fingers[i].y);
								}
							}
						}

						if(ts->exclude_key_disable_check_state != 0){
							if( time_after(jiffies, ts->exclude_key_disable_check_time) != 0 ){
								ts->exclude_key_disable_check_state = 0;
							}else{
								is_key_disable = 1;
								SHTPS_LOG_DBG_PRINT("[exclude]key down disable (disable effect time = %lu / now time = %lu)\n",
														ts->exclude_key_disable_check_time, jiffies);
							}
						}

						if(is_key_disable == 0){
							#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
								is_hold_keyevent = 1;
							#else
								shtps_key_event_report(ts, present_key_state);
							#endif /* SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */

							ts->exclude_touch_disable_check_state = 1;
							ts->exclude_touch_disable_check_time = jiffies + msecs_to_jiffies(SHTPS_TOUCH_DISABLE_TIME_MS);
							SHTPS_LOG_DBG_PRINT("[exclude]finger specific area disable time set : %lu\n", ts->exclude_touch_disable_check_time);
						}
					}
					else{
						#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
							is_hold_keyevent = 1;
						#else
							shtps_key_event_report(ts, present_key_state);
						#endif /* SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */
					}
				#else
					#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
						is_hold_keyevent = 1;
					#else
						shtps_key_event_report(ts, present_key_state);
					#endif /* SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */
				#endif /* SHTPS_FINGER_KEY_EXCLUDE_ENABLE */
			}
		#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

		#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
			{
				int fingerMax = shtps_get_fingermax(ts);
				if(SHTPS_STATE_FACETOUCH == state && (pFingerInfoBuf[fingerMax * 5] & 0x02) != 0){
					shtps_event_all_cancel(ts);
				}
			}
		#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

		shtps_calc_notify(ts, pFingerInfoBuf, &info, &event);

		#if defined(SHTPS_MOTION_PEDO_STOP_REQ_ENABLE)
			if(event == SHTPS_EVENT_TD){
				shub_api_stop_pedometer_func(SHUB_STOP_PED_TYPE_TPS);
			}else if(event == SHTPS_EVENT_TU){
				shub_api_restart_pedometer_func(SHUB_STOP_PED_TYPE_TPS);
			}
		#endif /* SHTPS_MOTION_PEDO_STOP_REQ_ENABLE */

		#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE) && defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
			if(is_hold_keyevent){
				if( shtps_filter_edge_fail_touch_check_inhibit(ts) ){
					present_key_state &= ~(1 << SHTPS_PHYSICAL_KEY_UP);
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[edge and touchkey exclude] ignore vol-up key down\n");
				}
				shtps_key_event_report(ts, present_key_state);
			}
		#endif /* SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */

		if(event != 0xff){
			shtps_perflock_enable_start(ts, event);	//SHTPS_CPU_CLOCK_CONTROL_ENABLE

			#if defined(SHTPS_LPWG_MODE_ENABLE)
				if(ts->lpwg.block_touchevent == 0){
					shtps_event_report(ts, &info, event);
				}else{
					SHTPS_LOG_DBG_PRINT("LPWG touch event blocked\n");
					shtps_event_update(ts, &info);
				}
			#else
				shtps_event_report(ts, &info, event);
			#endif /* SHTPS_LPWG_MODE_ENABLE */
		}

		#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
			if(SHTPS_STATE_FACETOUCH == state){
				shtps_check_facetouchoff(ts);
			}
		#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

		#if defined(SHTPS_LPWG_MODE_ENABLE)
			if(ts->lpwg.block_touchevent != 0){
				u8 rezero_req = 0;

				if(time_after(jiffies,
					ts->lpwg.wakeup_time + msecs_to_jiffies(SHTPS_LPWG_BLOCK_TIME_MAX_MS)))
				{
					rezero_req = 1;
					SHTPS_LOG_DBG_PRINT("LPWG force rezero\n");
				}else if((ts->lpwg.tu_rezero_req != 0) && 
							(ts->finger_state[0] | ts->finger_state[1] | ts->finger_state[2]) == 0)
				{
					rezero_req = 1;
					SHTPS_LOG_DBG_PRINT("LPWG rezero by touch up\n");
				}

				if(rezero_req != 0){
					ts->touch_state.numOfFingers = 0;
					ts->lpwg.block_touchevent = 0;
					ts->lpwg.tu_rezero_req = 0;
					shtps_rezero_request(ts,
										 SHTPS_REZERO_REQUEST_REZERO,
										 SHTPS_REZERO_TRIGGER_WAKEUP);

					#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
						ts->wakeup_touch_event_inhibit_state = 0;
						SHTPS_LOG_WAKEUP_FAIL_TOUCH_EVENT_REJECT("TouchEvent Inhibit end by rezero exec\n");
					#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */
				}
			}
		#endif /* SHTPS_LPWG_MODE_ENABLE */

		memcpy(&ts->fw_report_info_store, &ts->fw_report_info, sizeof(ts->fw_report_info));

		break;
	}
}

#if defined(SHTPS_IRQ_LOADER_CHECK_INT_STATUS_ENABLE)
static int shtps_loader_irqclr(struct shtps_rmi_spi *ts)
{
	int rc;
	u8 sts;
	SHTPS_LOG_FUNC_CALL();
	rc =  shtps_fwctl_irqclear_get_irqfactor(ts, &sts);
	if(rc == 0){
		return sts;
	}else{
		return 0;
	}
}
#else
static void shtps_loader_irqclr(struct shtps_rmi_spi *ts)
{
	u8 sts;
	SHTPS_LOG_FUNC_CALL();
	shtps_fwctl_irqclear_get_irqfactor(ts, &sts);
}
#endif /* SHTPS_IRQ_LOADER_CHECK_INT_STATUS_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
int shtps_boot_fwupdate_enable_check(struct shtps_rmi_spi *ts)	
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_BOOT_FWUPDATE_ONLY_ON_HANDSET) || defined(SHTPS_CHECK_HWID_ENABLE)
	{
	#if 0
		sharp_smem_common_type *sharp_smem;
		unsigned char handset;

		sharp_smem = sh_smem_get_common_address();
		handset = sharp_smem->sh_hw_handset;

		if(handset == 0){
	#else
		if(shtps_system_get_hw_type() == SHTPS_HW_TYPE_BOARD){
	#endif
			return 0;
		}
	}
	#endif /* SHTPS_BOOT_FWUPDATE_ONLY_ON_HANDSET || SHTPS_CHECK_HWID_ENABLE */

	#if defined( SHTPS_TPIN_CHECK_ENABLE )
		if(shtps_tpin_enable_check(ts) != 0){
			return 0;
		}
	#endif /* SHTPS_TPIN_CHECK_ENABLE */

	return 1;
}

#if defined(SHTPS_BOOT_FWUPDATE_FORCE_UPDATE)
	//nothing
#else
int shtps_fwup_flag_check(void)
{
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
}
#endif /* #if !defined(SHTPS_BOOT_FWUPDATE_FORCE_UPDATE) */

void shtps_fwup_flag_clear(void)
{
	sharp_smem_common_type *smemdata = NULL;

	SHTPS_LOG_FUNC_CALL();
	smemdata = sh_smem_get_common_address();
	if(smemdata != NULL){
		smemdata->shtps_fwup_flag = 0;
	}
}
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

static int shtps_fwupdate_enable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( SHTPS_FWUPDATE_DISABLE )
	return 0;
#else
	return 1;
#endif /* #if defined( SHTPS_FWUPDATE_DISABLE ) */
}

static int shtps_loader_wait_attn(struct shtps_rmi_spi *ts)
{
	int rc;

	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_DBG_PRINT("shtps_loader_wait_attn() start:%d\n", ts->loader.ack);

	rc = wait_event_interruptible_timeout(ts->loader.wait_ack,
			ts->loader.ack == 1,
			msecs_to_jiffies(SHTPS_BOOTLOADER_ACK_TMO));

	if(0 == rc && 0 == ts->loader.ack){
		_log_msg_sync( LOGMSG_ID__BL_ATTN_ERROR, "");
		SHTPS_LOG_ERR_PRINT("shtps_loader_wait_attn() ACK:%d\n", ts->loader.ack);
		return -1;
	}

	if(0 == rc){
		_log_msg_sync( LOGMSG_ID__BL_ATTN_TIMEOUT, "");
		SHTPS_LOG_ERR_PRINT("shtps_loader_wait_attn() warning rc = %d\n", rc);
	}

	ts->loader.ack = 0;

	SHTPS_LOG_DBG_PRINT("shtps_loader_wait_attn() end:%d\n", ts->loader.ack);
	return 0;
}

static void shtps_loader_wakeup(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->loader.ack = 1;
	wake_up_interruptible(&ts->loader.wait_ack);
}

int shtps_enter_bootloader(struct shtps_rmi_spi *ts)
{
	int rc;

	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_loader();

	request_event(ts, SHTPS_EVENT_STOP, 0);
	msleep(SHTPS_SLEEP_IN_WAIT_MS);
	if(request_event(ts, SHTPS_EVENT_STARTLOADER, 0) != 0){
		SHTPS_LOG_ERR_PRINT("shtps_enter_bootloader() start loader error\n");
		goto err_exit;
	}
	shtps_wait_startup(ts);

	shtps_sleep(ts, 1);
	#if !defined(SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE)
		msleep(SHTPS_SLEEP_IN_WAIT_MS);
	#endif /* !SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE */

	ts->loader.ack = 0;
	shtps_fwctl_loader_cmd_enterbl(ts);
	rc = shtps_loader_wait_attn(ts);
	if(rc){
		SHTPS_LOG_ERR_PRINT("shtps_enter_bootloader() mode change error\n");
		goto err_exit;
	}

	shtps_fwctl_set_dev_state(ts, SHTPS_DEV_STATE_LOADER);

#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	if(shtps_map_construct(ts, 0) != 0){
#else
	if(shtps_map_construct(ts) != 0){
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */

		SHTPS_LOG_ERR_PRINT("shtps_map_construct() error!!\n");
	}

	SHTPS_LOG_DBG_PRINT("shtps_enter_bootloader() done\n");
	shtps_mutex_unlock_loader();
	return 0;

err_exit:
	shtps_mutex_unlock_loader();
	return -1;
}

static int shtps_exit_bootloader(struct shtps_rmi_spi *ts)
{
	int rc;

	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_ANALYSIS("SW Reset execute\n");

	rc = shtps_fwctl_loader_exit(ts);
	request_event(ts, SHTPS_EVENT_STOP, 0);

	SHTPS_LOG_DBG_PRINT("shtps_exit_bootloader() done:%d\n",rc);
	return rc;
}

int shtps_lockdown_bootloader(struct shtps_rmi_spi *ts, u8* fwdata)
{
	return 0;
}

int shtps_flash_erase(struct shtps_rmi_spi *ts)
{
	int rc;

	SHTPS_LOG_FUNC_CALL();
	if(!shtps_fwupdate_enable(ts)){
		return 0;
	}
	shtps_mutex_lock_loader();

	rc = shtps_fwctl_loader_cmd_erase(ts);
	if(rc == 0)	rc = shtps_loader_wait_attn(ts);
	if(rc == 0)	rc = shtps_fwctl_loader_get_result_erase(ts);
	if(rc == 0)	rc = shtps_fwctl_loader_init_writeconfig(ts);

	shtps_mutex_unlock_loader();
	SHTPS_LOG_DBG_PRINT("shtps_flash_erase() done:%d\n",rc);
	return rc;
}

int shtps_flash_writeImage(struct shtps_rmi_spi *ts, u8 *fwdata)
{
	int rc;

	SHTPS_LOG_FUNC_CALL();
	if(!shtps_fwupdate_enable(ts)){
		return 0;
	}

	shtps_mutex_lock_loader();

	rc = shtps_fwctl_loader_write_image(ts, fwdata, shtps_fwctl_loader_get_blocksize(ts));
	if(rc == 0)	rc = shtps_fwctl_loader_cmd_writeimage(ts);
	if(rc == 0)	rc = shtps_loader_wait_attn(ts);
	if(rc == 0)	rc = shtps_fwctl_loader_get_result_writeimage(ts);

	shtps_mutex_unlock_loader();

	SHTPS_LOG_DBG_PRINT("shtps_flash_writeImage() done\n");
	return rc;
}

int shtps_flash_writeConfig(struct shtps_rmi_spi *ts, u8 *fwdata)
{
	int rc;
	int block;
	int blockNum;
	int blockSize;

	SHTPS_LOG_FUNC_CALL();
	if(!shtps_fwupdate_enable(ts)){
		return 0;
	}

	shtps_mutex_lock_loader();

	blockNum  = (int)shtps_fwctl_loader_get_config_blocknum(ts);
	blockSize = (int)shtps_fwctl_loader_get_blocksize(ts);
	rc = shtps_fwctl_loader_init_writeconfig(ts);
	for(block = 0;(rc == 0)&&(block < blockNum);block++){
		rc = shtps_fwctl_loader_write_config(ts, &fwdata[block * blockSize], blockSize);
		if(rc == 0)	rc = shtps_fwctl_loader_cmd_writeconfig(ts);
		if(rc == 0)	rc = shtps_loader_wait_attn(ts);
		if(rc == 0)	rc = shtps_fwctl_loader_get_result_writeconfig(ts);
	}
	if(rc == 0)	rc = shtps_exit_bootloader(ts);

	shtps_mutex_unlock_loader();
	SHTPS_LOG_DBG_PRINT("shtps_flash_writeConfig() done:%d\n",rc);
	return rc;
}

/* -----------------------------------------------------------------------------------
*/
int shtps_set_veilview_state(struct shtps_rmi_spi *ts, unsigned long arg)
{
	return 0;
}

int shtps_get_veilview_pattern(struct shtps_rmi_spi *ts)
{
	return SHTPS_VEILVIEW_PATTERN;
}

/* -----------------------------------------------------------------------------------
 */
int request_event(struct shtps_rmi_spi *ts, int event, int param)	
{
	int ret;

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__REQUEST_EVENT, "%d|%d", event, ts->state_mgr.state);

	SHTPS_LOG_DBG_PRINT("event %d in state %d\n", event, ts->state_mgr.state);

	shtps_mutex_lock_ctrl();

	switch(event){
	case SHTPS_EVENT_START:
		ret = state_func_tbl[ts->state_mgr.state]->start(ts, param);
		break;
	case SHTPS_EVENT_STOP:
		ret = state_func_tbl[ts->state_mgr.state]->stop(ts, param);
		break;
	case SHTPS_EVENT_SLEEP:
		ret = state_func_tbl[ts->state_mgr.state]->sleep(ts, param);
		break;
	case SHTPS_EVENT_WAKEUP:
		ret = state_func_tbl[ts->state_mgr.state]->wakeup(ts, param);
		break;
	case SHTPS_EVENT_STARTLOADER:
		ret = state_func_tbl[ts->state_mgr.state]->start_ldr(ts, param);
		break;
	case SHTPS_EVENT_STARTTM:
		ret = state_func_tbl[ts->state_mgr.state]->start_tm(ts, param);
		break;
	case SHTPS_EVENT_STOPTM:
		ret = state_func_tbl[ts->state_mgr.state]->stop_tm(ts, param);
		break;
	case SHTPS_EVENT_FACETOUCHMODE_ON:
		ret = state_func_tbl[ts->state_mgr.state]->facetouch_on(ts, param);
		break;
	case SHTPS_EVENT_FACETOUCHMODE_OFF:
		ret = state_func_tbl[ts->state_mgr.state]->facetouch_off(ts, param);
		break;
	case SHTPS_EVENT_INTERRUPT:
		ret = state_func_tbl[ts->state_mgr.state]->interrupt(ts, param);
		break;
	case SHTPS_EVENT_TIMEOUT:
		ret = state_func_tbl[ts->state_mgr.state]->timeout(ts, param);
		break;
	default:
		ret = -1;
		break;
	}

	shtps_mutex_unlock_ctrl();

	return ret;
}

static int state_change(struct shtps_rmi_spi *ts, int state)
{
	int ret = 0;
	int old_state = ts->state_mgr.state;

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__STATE_CHANGE, "%d|%d", ts->state_mgr.state, state);
	SHTPS_LOG_DBG_PRINT("state %d -> %d\n", ts->state_mgr.state, state);

	if(ts->state_mgr.state != state){
		ts->state_mgr.state = state;
		ret = state_func_tbl[ts->state_mgr.state]->enter(ts, old_state);
	}
	return ret;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_nop(struct shtps_rmi_spi *ts, int param)
{
	_log_msg_sync( LOGMSG_ID__STATEF_NOP, "%d", ts->state_mgr.state);
	SHTPS_LOG_FUNC_CALL();
	return 0;
}

static int shtps_statef_cmn_error(struct shtps_rmi_spi *ts, int param)
{
	_log_msg_sync( LOGMSG_ID__STATEF_ERROR, "%d", ts->state_mgr.state);
	SHTPS_LOG_FUNC_CALL();
	return -1;
}

static int shtps_statef_cmn_stop(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__STATEF_STOP, "%d", ts->state_mgr.state);
	
	#if defined(SHTPS_MOTION_PEDO_STOP_REQ_ENABLE)
		shub_api_restart_pedometer_func(SHUB_STOP_PED_TYPE_TPS);
	#endif /* SHTPS_MOTION_PEDO_STOP_REQ_ENABLE */
	
	shtps_standby_param(ts);
	shtps_irq_disable(ts);
	shtps_system_set_sleep(ts);
	state_change(ts, SHTPS_STATE_IDLE);
	return 0;
}

static int shtps_statef_cmn_facetouch_on(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_set_facetouchmode(ts, 1);
#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */
	return 0;
}

static int shtps_statef_cmn_facetouch_off(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_set_facetouchmode(ts, 0);
#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_idle_start(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined( SHTPS_TPIN_CHECK_ENABLE )
		if(shtps_tpin_enable_check(ts) != 0){
			return -1;
		}
	#endif /* SHTPS_TPIN_CHECK_ENABLE */

	shtps_system_set_wakeup(ts);
	shtps_clr_startup_err(ts);
	shtps_reset(ts);
	shtps_irq_enable(ts);
	shtps_irqtimer_start(ts, 100);
	shtps_set_startmode(ts, SHTPS_MODE_NORMAL);
	state_change(ts, SHTPS_STATE_WAIT_WAKEUP);
	return 0;
}

static int shtps_statef_idle_start_ldr(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined( SHTPS_TPIN_CHECK_ENABLE )
		if(shtps_tpin_enable_check(ts) != 0){
			return -1;
		}
	#endif /* SHTPS_TPIN_CHECK_ENABLE */

	shtps_system_set_wakeup(ts);
	shtps_clr_startup_err(ts);
	shtps_reset(ts);
	shtps_irq_enable(ts);
	shtps_irqtimer_start(ts, 100);
	shtps_set_startmode(ts, SHTPS_MODE_LOADER);
	state_change(ts, SHTPS_STATE_WAIT_WAKEUP);
	return 0;
}

static int shtps_statef_idle_int(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_read_touchevent(ts, SHTPS_STATE_IDLE);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_waiwakeup_stop(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_irqtimer_stop(ts);
	shtps_statef_cmn_stop(ts, param);
	return 0;
}

static int shtps_statef_waiwakeup_int(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_reset_startuptime(ts);
	shtps_irqtimer_stop(ts);
	shtps_irqtimer_start(ts, 1000);
	state_change(ts, SHTPS_STATE_WAIT_READY);
	return 0;
}

static int shtps_statef_waiwakeup_tmo(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_irqtimer_start(ts, 1000);
	state_change(ts, SHTPS_STATE_WAIT_READY);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_state_waiready_stop(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_irqtimer_stop(ts);
	shtps_statef_cmn_stop(ts, param);
	return 0;
}

static int shtps_state_waiready_int(struct shtps_rmi_spi *ts, int param)
{
#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	int rc;
	int retry = SHTPS_PDT_READ_RETRY_COUNT;
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */
	unsigned long time;
	SHTPS_LOG_FUNC_CALL();
	if((time = shtps_check_startuptime(ts)) != 0){
		SHTPS_LOG_DBG_PRINT("startup wait time : %lu\n", time);
		msleep(time);
	}

	shtps_irqtimer_stop(ts);

#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	do{
		rc = shtps_map_construct(ts, 1);
		if(rc){
			msleep(SHTPS_PDT_READ_RETRY_INTERVAL);
		}
	}while(rc != 0 && retry-- > 0);
#else
	if(shtps_map_construct(ts) != 0){
		SHTPS_LOG_ERR_PRINT("shtps_map_construct() error!!\n");
		shtps_statef_cmn_stop(ts, 0);
		shtps_notify_startup(ts, SHTPS_STARTUP_FAILED);
		return -1;
	}
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */

	if(SHTPS_MODE_NORMAL == shtps_get_startmode(ts)){
		state_change(ts, SHTPS_STATE_ACTIVE);
	}else{
		state_change(ts, SHTPS_STATE_BOOTLOADER);
	}

#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	if(rc != 0){
		shtps_notify_startup(ts, SHTPS_STARTUP_FAILED);
	}else
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */
	shtps_notify_startup(ts, SHTPS_STARTUP_SUCCESS);
	return 0;
}

static int shtps_state_waiready_tmo(struct shtps_rmi_spi *ts, int param)
{
#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	int rc;
	int retry = SHTPS_PDT_READ_RETRY_COUNT;

	do{
		rc = shtps_map_construct(ts, 1);
		if(rc){
			msleep(SHTPS_PDT_READ_RETRY_INTERVAL);
		}
	}while(rc != 0 && retry-- > 0);
#else
	if(shtps_map_construct(ts) != 0){
		SHTPS_LOG_ERR_PRINT("shtps_map_construct() error!!\n");
		shtps_statef_cmn_stop(ts, 0);
		shtps_notify_startup(ts, SHTPS_STARTUP_FAILED);
		return -1;
	}
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */

	if(SHTPS_MODE_NORMAL == shtps_get_startmode(ts)){
		state_change(ts, SHTPS_STATE_ACTIVE);
	}else{
		state_change(ts, SHTPS_STATE_BOOTLOADER);
	}

#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	if(rc != 0){
		shtps_notify_startup(ts, SHTPS_STARTUP_FAILED);
	}else
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */
	shtps_notify_startup(ts, SHTPS_STARTUP_SUCCESS);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_active_enter(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	if(param == SHTPS_STATE_WAIT_READY){
		shtps_init_param(ts);
		if(ts->poll_info.boot_rezero_flag == 0){
			ts->poll_info.boot_rezero_flag = 1;
			shtps_rezero_request(ts,
								 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
								 SHTPS_REZERO_TRIGGER_BOOT);
		}
		shtps_read_touchevent(ts, ts->state_mgr.state);
		#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
			shtps_irq_enable(ts);
		#else
			shtps_irq_wake_enable(ts);
		#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
	}

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		if(1 == shtps_get_facetouchmode(ts)){
			shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE, 0);
			state_change(ts, SHTPS_STATE_FACETOUCH);
		}
	#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */

	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		if( (ts->system_boot_mode == SH_BOOT_O_C) || (ts->system_boot_mode == SH_BOOT_U_O_C) ){
			state_change(ts, SHTPS_STATE_SLEEP);
		}
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	return 0;
}

static int shtps_statef_active_stop(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_active_sleep(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	state_change(ts, SHTPS_STATE_SLEEP);
	return 0;
}

static int shtps_statef_active_starttm(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_active_facetouch_on(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE, 0);
		shtps_set_facetouchmode(ts, 1);
		state_change(ts, SHTPS_STATE_FACETOUCH);
#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */
	return 0;
}

static int shtps_statef_active_int(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_read_touchevent(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_loader_enter(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_wake_lock_for_fwupdate(ts);	//SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE

	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_enable(ts);
	#else
		shtps_irq_wake_enable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	shtps_loader_irqclr(ts);
	return 0;
}

static int shtps_statef_loader_stop(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	shtps_wake_unlock_for_fwupdate(ts);	//SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE

	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_loader_int(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();

	#if defined(SHTPS_IRQ_LOADER_CHECK_INT_STATUS_ENABLE)
		if(shtps_loader_irqclr(ts) != 0){
			shtps_loader_wakeup(ts);
		}
	#else
		shtps_loader_irqclr(ts);
		shtps_loader_wakeup(ts);
	#endif /* SHTPS_IRQ_LOADER_CHECK_INT_STATUS_ENABLE */

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_INVERTING_GHOST_REJECTION_ENABLE )
static int shtps_statef_facetouch_enter(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_filter_inverting_ghost_set_palm_thresh_default(ts);
	
	return 0;
}
#endif /* SHTPS_INVERTING_GHOST_REJECTION_ENABLE */

static int shtps_statef_facetouch_sleep(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		#if defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE )
			shtps_set_doze_mode(ts, 1);
		#endif /* SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE */
	#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	state_change(ts, SHTPS_STATE_SLEEP_FACETOUCH);
	return 0;
}

static int shtps_statef_facetouch_starttm(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_rezero_request(ts,
							 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
							 SHTPS_REZERO_TRIGGER_ENDCALL);
		shtps_set_facetouchmode(ts, 0);
		state_change(ts, SHTPS_STATE_ACTIVE);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return 0;
}

static int shtps_statef_facetouch_int(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_read_touchevent(ts, SHTPS_STATE_FACETOUCH);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_fwtm_enter(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_tm_irqcheck(ts);

	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_enable(ts);
	#else
		shtps_irq_wake_enable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	shtps_start_tm(ts);
	return 0;
}

static int shtps_statef_fwtm_stop(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_fwtm_stoptm(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_stop_tm(ts);
	state_change(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

static int shtps_statef_fwtm_int(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	if(shtps_tm_irqcheck(ts)){
		shtps_tm_wakeup(ts);
	}
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_sleep_enter(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined( SHTPS_DEVELOP_MODE_ENABLE )
		shtps_debug_sleep_enter();
	#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */

	shtps_perflock_sleep(ts);	//SHTPS_CPU_CLOCK_CONTROL_ENABLE

	#if defined(SHTPS_FINGER_KEY_EXCLUDE_ENABLE)
		ts->exclude_touch_disable_check_state = 0;
		ts->exclude_touch_disable_finger = 0;
		ts->exclude_key_disable_check_state = 0;
	#endif /* SHTPS_FINGER_KEY_EXCLUDE_ENABLE */

	shtps_filter_sleep_enter(ts);

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		shtps_key_event_force_touchup(ts);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */
	shtps_event_force_touchup(ts);

	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	{
		u8 buf;
		shtps_fwctl_irqclear_get_irqfactor(ts, &buf);
	}

	#if defined( SHTPS_PALM_HOST_DETECT_TOUCH_UP_ENABLE )
		ts->host_palm_detect_wait_all_touchup = 0;
	#endif /* SHTPS_PALM_HOST_DETECT_TOUCH_UP_ENABLE */

	#if defined( SHTPS_INVERTING_GHOST_REJECTION_ENABLE )
		shtps_filter_inverting_ghost_set_palm_thresh_default(ts);
	#endif /* SHTPS_INVERTING_GHOST_REJECTION_ENABLE */


	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		ts->lpmode_req_state = SHTPS_LPMODE_REQ_NONE;
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if(shtps_is_lpwg_active(ts)){
			#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
				shtps_irq_enable(ts);
			#else
				shtps_irq_wake_enable(ts);
			#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

			shtps_set_lpwg_mode_on(ts);
			shtps_set_lpwg_mode_cal(ts);
		}else{
			shtps_sleep(ts, 1);
			shtps_system_set_sleep(ts);
		}
	#else
		shtps_sleep(ts, 1);
		shtps_system_set_sleep(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	return 0;
}

static int shtps_statef_sleep_wakeup(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_enable(ts);
	#else
		shtps_irq_wake_enable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if (ts->lpwg.lpwg_switch){
			ts->lpwg.wakeup_time = jiffies;
			shtps_set_lpwg_mode_off(ts);
			shtps_read_touchevent(ts, SHTPS_STATE_IDLE);
		}else{
			shtps_system_set_wakeup(ts);
		}
	#else
		shtps_system_set_wakeup(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	shtps_sleep(ts, 0);

	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		{
			u8 buf;
			shtps_fwctl_irqclear_get_irqfactor(ts, &buf);
		}
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	#if defined( SHTPS_DYNAMIC_RESET_CONTROL_ENABLE )
		shtps_filter_dynamic_reset_sleep_wakeup(ts);
	#endif /* SHTPS_DYNAMIC_RESET_CONTROL_ENABLE */

	shtps_rezero_request(ts,
						 SHTPS_REZERO_REQUEST_WAKEUP_REZERO |
						 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
						 SHTPS_REZERO_TRIGGER_WAKEUP);
	state_change(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

static int shtps_statef_sleep_starttm(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_system_set_wakeup(ts);
	shtps_sleep(ts, 0);
	#if !defined(SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE)
		if(SHTPS_SLEEP_OUT_WAIT_MS > 0){
			msleep(SHTPS_SLEEP_OUT_WAIT_MS);
		}
	#endif /* !SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE */
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_sleep_facetouch_on(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
			shtps_irq_enable(ts);
		#else
			shtps_irq_wake_enable(ts);
		#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
		
		#if defined(SHTPS_LPWG_MODE_ENABLE)
			if (ts->lpwg.lpwg_switch){
				ts->lpwg.wakeup_time = jiffies;
				shtps_set_lpwg_mode_off(ts);
				shtps_read_touchevent(ts, SHTPS_STATE_IDLE);
			}else{
				shtps_system_set_wakeup(ts);
			}
		#else
			shtps_system_set_wakeup(ts);
		#endif /* SHTPS_LPWG_MODE_ENABLE */

		shtps_sleep(ts, 0);

		#if defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE )
			shtps_set_doze_mode(ts, 1);
		#endif /* SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE */

		shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE, 0);
		shtps_set_facetouchmode(ts, 1);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	state_change(ts, SHTPS_STATE_SLEEP_FACETOUCH);
	return 0;
}

static int shtps_statef_sleep_int(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if (ts->lpwg.lpwg_switch){
			shtps_read_touchevent_insleep(ts, SHTPS_STATE_SLEEP);
		}else{
			shtps_read_touchevent(ts, SHTPS_STATE_SLEEP);
		}
	#else
		shtps_read_touchevent(ts, SHTPS_STATE_SLEEP);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_sleep_facetouch_enter(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	ts->facetouch.touch_num = ts->touch_state.numOfFingers;

	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		shtps_perflock_sleep(ts);
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
//		shtps_absorption_hold_cancel(ts);
		shtps_filter_absorption_hold_cancel(ts);
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE ) */

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		shtps_key_event_force_touchup(ts);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

	#if defined( SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHANGE_ENABLE )
		shtps_set_motion_suppression(ts, SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_X_THRESH, SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_Y_THRESH);
	#endif /* SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHANGE_ENABLE */

	shtps_event_all_cancel(ts);

	return 0;
#else
	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		ts->lpmode_req_state = SHTPS_LPMODE_REQ_NONE;
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	shtps_sleep(ts, 1);
	shtps_system_set_sleep(ts);
	return 0;
#endif /* #if !defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
}

static int shtps_statef_sleep_facetouch_stop(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )

	#if defined( SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHANGE_ENABLE )
		shtps_set_motion_suppression_default(ts);
	#endif /* SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHANGE_ENABLE */

	shtps_facetouch_wakelock(ts, 0);
	
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_sleep_facetouch_wakeup(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_facetouch_wakelock(ts, 0);

	#if defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE )
		if(shtps_is_lpmode(ts) == 0){
			shtps_set_doze_mode(ts, 0);
		}
	#endif /* SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE */

	#if defined( SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHANGE_ENABLE )
		shtps_set_motion_suppression_default(ts);
	#endif /* SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHANGE_ENABLE */

#else
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_enable(ts);
	#else
		shtps_irq_wake_enable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
	
	shtps_system_set_wakeup(ts);
	shtps_sleep(ts, 0);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	
	
#if defined( SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE )
	if(ts->facetouch.rezero_disable == 1 && SHTPS_FACETOUCHOFF_DETECT_SKIP_REZERO_ENABLE == 1){
		ts->facetouch.rezero_disable = 0;
		SHTPS_LOG_DBG_PRINT("Detect face touch off. skip rezero...\n");
	}else{
		shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_WAKEUP_REZERO, 0);
	}
#else  /*SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE*/
	shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_WAKEUP_REZERO, 0);
#endif /*SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE*/
	
	state_change(ts, SHTPS_STATE_FACETOUCH);
	return 0;
}

static int shtps_statef_sleep_facetouch_starttm(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )

	#if defined( SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHANGE_ENABLE )
		shtps_set_motion_suppression_default(ts);
	#endif /* SHTPS_FACETOUCH_SLEEP_MOTION_SUPPRESSION_CHANGE_ENABLE */

	shtps_facetouch_wakelock(ts, 0);
	shtps_sleep(ts, 0);
#else
	shtps_system_set_wakeup(ts);
	shtps_sleep(ts, 0);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_sleep_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_facetouch_wakelock(ts, 0);
	shtps_rezero_request(ts,
						 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
						 SHTPS_REZERO_TRIGGER_ENDCALL);

	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if(shtps_is_lpwg_active(ts)){
			#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
				shtps_irq_enable(ts);
			#else
				shtps_irq_wake_enable(ts);
			#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
			
			shtps_set_lpwg_mode_on(ts);
			shtps_set_lpwg_mode_cal(ts);
		}else{
			shtps_sleep(ts, 1);
			shtps_system_set_sleep(ts);
		}
	#else
		shtps_sleep(ts, 1);
		shtps_system_set_sleep(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	shtps_set_facetouchmode(ts, 0);		
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
//	shtps_set_facetouchmode(ts, 0);
	state_change(ts, SHTPS_STATE_SLEEP);
	return 0;
}

static int shtps_statef_sleep_facetouch_int(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_read_touchevent_infacetouchmode(ts);
#else
	shtps_read_touchevent(ts, SHTPS_STATE_SLEEP);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_rmi_open(struct input_dev *dev)
{
	struct shtps_rmi_spi *ts = (struct shtps_rmi_spi*)input_get_drvdata(dev);

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__OPEN, "%ld", sys_getpid());
	SHTPS_LOG_DBG_PRINT("[shtps]Open(PID:%ld)\n", sys_getpid());

	shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_OPEN);

	return 0;
}

static void shtps_rmi_close(struct input_dev *dev)
{
	struct shtps_rmi_spi *ts = (struct shtps_rmi_spi*)input_get_drvdata(dev);

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__CLOSE, "%ld", sys_getpid());
	SHTPS_LOG_DBG_PRINT("[shtps]Close(PID:%ld)\n", sys_getpid());

	shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_CLOSE);
}

static int shtps_init_internal_variables(struct shtps_rmi_spi *ts)
{
	int result = 0;
	
	SHTPS_LOG_FUNC_CALL();

	INIT_DELAYED_WORK(&ts->tmo_check, shtps_work_tmof);

	init_waitqueue_head(&ts->wait_start);
	init_waitqueue_head(&ts->loader.wait_ack);
	init_waitqueue_head(&ts->diag.wait);
	init_waitqueue_head(&ts->diag.tm_wait_ack);

	hrtimer_init(&ts->rezero_delayed_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->rezero_delayed_timer.function = shtps_delayed_rezero_timer_function;
	INIT_WORK(&ts->rezero_delayed_work, shtps_rezero_delayed_work_function);

#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
	hrtimer_init(&ts->facetouch_off_notify_delayed_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->facetouch_off_notify_delayed_timer.function = shtps_delayed_facetouch_off_notify_function;
    wake_lock_init(&ts->facetouch.facetouch_off_delayed_wake_lock, WAKE_LOCK_SUSPEND, "shtps_faceoff_delayed_wake_lock");
	pm_qos_add_request(&ts->facetouch.facetouch_off_delayed_qos, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */

	result = shtps_func_async_init(ts);
	if(result < 0){
		goto fail_init;
	}

	ts->state_mgr.state = SHTPS_STATE_IDLE;
	ts->loader.ack = 0;
	ts->diag.event = 0;
	ts->offset.enabled = 0;
	ts->bt_ver = 0;

	memset(&ts->poll_info,   0, sizeof(ts->poll_info));
	memset(&ts->fw_report_info, 0, sizeof(ts->fw_report_info));
	memset(&ts->fw_report_info_store, 0, sizeof(ts->fw_report_info_store));
	memset(&ts->report_info, 0, sizeof(ts->report_info));
	/* memset(&ts->center_info, 0, sizeof(ts->center_info)); */
	memset(&ts->touch_state, 0, sizeof(ts->touch_state));

	shtps_filter_drag_init(ts);	//SHTPS_DRAG_STEP_ENABLE
	shtps_perflock_init(ts);	//SHTPS_CPU_CLOCK_CONTROL_ENABLE

	shtps_cpu_idle_sleep_wake_lock_init(ts);	//SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE
	shtps_fwupdate_wake_lock_init(ts);			//SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE

	shtps_fwctl_set_dev_state(ts, SHTPS_DEV_STATE_SLEEP);
	shtps_performance_check_init();

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_facetouch_init(ts);
	#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		ts->lpmode_req_state = SHTPS_LPMODE_REQ_NONE;
		ts->lpmode_continuous_req_state = SHTPS_LPMODE_REQ_NONE;
	#endif /*` #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	#if defined(SHTPS_HOVER_DETECT_ENABLE)
		ts->hover_enable_state = 0;
	#endif /* SHTPS_HOVER_DETECT_ENABLE */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		shtps_lpwg_wakelock_init(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		ts->key_state  = 0;
		ts->diag.event_touchkey = 0;
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

	#if defined(SHTPS_FINGER_KEY_EXCLUDE_ENABLE)
		ts->exclude_touch_disable_check_state = 0;
		ts->exclude_touch_disable_finger = 0;
		ts->exclude_key_disable_check_state = 0;
		ts->exclude_touch_disable_check_time = 0;
		ts->exclude_key_disable_check_time = 0;
	#endif /* SHTPS_FINGER_KEY_EXCLUDE_ENABLE */

	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		ts->system_boot_mode = sh_boot_get_bootmode();
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
	    ts->deter_suspend_spi.wake_lock_state = 0;
		memset(&ts->deter_suspend_spi, 0, sizeof(ts->deter_suspend_spi));
	    wake_lock_init(&ts->deter_suspend_spi.wake_lock, WAKE_LOCK_SUSPEND, "shtps_resume_wake_lock");
		pm_qos_add_request(&ts->deter_suspend_spi.pm_qos_lock_idle, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
		INIT_WORK(&ts->deter_suspend_spi.pending_proc_work, shtps_deter_suspend_spi_pending_proc_work_function);
		#ifdef SHTPS_DEVELOP_MODE_ENABLE
			INIT_DELAYED_WORK(&ts->deter_suspend_spi.pending_proc_work_delay, shtps_deter_suspend_spi_pending_proc_delayed_work_function);
		#endif /* SHTPS_DEVELOP_MODE_ENABLE */
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
		ts->wakeup_touch_event_inhibit_state = 0;
	#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */

	#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )
		ts->key_down_reserved = 0;
		ts->key_down_ignored = 0;
		ts->key_proximity_check_state = 0;
		INIT_DELAYED_WORK(&ts->touchkey_delayed_work, shtps_touchkey_delayed_work_function);
		INIT_DELAYED_WORK(&ts->touchkey_inproxymity_delayed_work, shtps_touchkey_inproxymity_delayed_work_function);
	#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */

	#if defined(SHTPS_LOW_REPORTRATE_MODE)
		ts->low_report_rate_mode_state = 0;
	#endif /* SHTPS_LOW_REPORTRATE_MODE */

	#if defined(SHTPS_PEN_DETECT_ENABLE)
		ts->pen_enable = 1;
	#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */

	/* -------------------------------------------------------------------------- */
	shtps_filter_init(ts);

	/* -------------------------------------------------------------------------- */
	return 0;

fail_init:
	shtps_func_async_deinit(ts);

	return result;
}


static void shtps_deinit_internal_variables(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	if(ts){
		hrtimer_cancel(&ts->rezero_delayed_timer);

		shtps_filter_deinit(ts);
		shtps_filter_drag_deinit(ts);				//SHTPS_DRAG_STEP_ENABLE

		shtps_perflock_deinit(ts);					//SHTPS_CPU_CLOCK_CONTROL_ENABLE
		shtps_func_async_deinit(ts);
		shtps_cpu_idle_sleep_wake_lock_deinit(ts);	//SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE
		shtps_fwupdate_wake_lock_deinit(ts);		//SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE

		#if defined(SHTPS_LPWG_MODE_ENABLE)
			shtps_lpwg_wakelock_destroy(ts);
		#endif /* SHTPS_LPWG_MODE_ENABLE */

	}
}

static int shtps_init_inputdev(struct shtps_rmi_spi *ts, struct device *ctrl_dev_p, char *modalias)
{
	SHTPS_LOG_FUNC_CALL();
	ts->input = input_allocate_device();
	if (!ts->input){
		SHTPS_LOG_ERR_PRINT("Failed input_allocate_device\n");
		return -ENOMEM;
	}

	ts->input->name 		= modalias;
	ts->input->phys			= ts->phys;
	ts->input->id.vendor	= 0x0001;
	ts->input->id.product	= 0x0002;
	ts->input->id.version	= 0x0100;
	ts->input->dev.parent	= ctrl_dev_p;
	ts->input->open			= shtps_rmi_open;
	ts->input->close		= shtps_rmi_close;

	/** set properties */
	__set_bit(EV_KEY, ts->input->evbit);
	__set_bit(EV_ABS, ts->input->evbit);
	__set_bit(INPUT_PROP_DIRECT, ts->input->propbit);
	
	#if defined( CONFIG_SHTPS_SY3000_VIRTUAL_KEY )
		__set_bit(KEY_PROG1, ts->input->keybit);
	#endif /* #if defined( CONFIG_SHTPS_SY3000_VIRTUAL_KEY ) */

	input_set_drvdata(ts->input, ts);
	input_mt_init_slots(ts->input, SHTPS_FINGER_MAX);

	if(ts->input->mt == NULL){
		input_free_device(ts->input);
		return -ENOMEM;
	}

	/** set parameters */
	#if defined(SHTPS_PEN_DETECT_ENABLE)
		input_set_abs_params(ts->input, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);
	#endif /* SHTPS_PEN_DETECT_ENABLE */
	
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, SHTPS_VAL_FINGER_WIDTH_MAXSIZE, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_X,  0, CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y,  0, CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1, 0, 0);

	#if defined( SHTPS_HOVER_DETECT_ENABLE )
	{
		int i;

		input_set_abs_params(ts->input, ABS_MT_PRESSURE,    0, 256, 0, 0);

		for(i = 0; i < ts->input->mtsize; i++){
			input_mt_set_value(&ts->input->mt[i], ABS_MT_PRESSURE, 256);
		}
	}
	#else
		input_set_abs_params(ts->input, ABS_MT_PRESSURE,    0, 255, 0, 0);
	#endif /* SHTPS_HOVER_DETECT_ENABLE */

	/** register input device */
	if(input_register_device(ts->input) != 0){
		input_free_device(ts->input);
		return -EFAULT;
	}

	return 0;
}

static void shtps_deinit_inputdev(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	if(ts && ts->input){
		if(ts->input->mt){
			input_mt_destroy_slots(ts->input);
		}
		input_free_device(ts->input);
	}
}

#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
static int shtps_init_inputdev_key(struct shtps_rmi_spi *ts, struct device *ctrl_dev_p)
{
	SHTPS_LOG_FUNC_CALL();
	ts->input_key = input_allocate_device();
	if(!ts->input_key){
		return -ENOMEM;
	}

	ts->input_key->name 		= "shtps_key";
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

static void shtps_deinit_inputdev_key(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	if(ts && ts->input_key){
		input_free_device(ts->input_key);
	}
}
#endif /* SHTPS_PHYSICAL_KEY_ENABLE || SHTPS_LPWG_MODE_ENABLE */


int shtps_rmi_core_probe(
						struct device *ctrl_dev_p,
						struct shtps_ctrl_functbl *func_p,
						void *ctrl_p,
						char *modalias,
						int gpio_irq)
{
	extern void shtps_init_io_debugfs(struct shtps_rmi_spi *ts);
	extern void shtps_init_debugfs(struct shtps_rmi_spi *ts);
	extern struct device*	shtpsif_device;

	int result = 0;
	struct shtps_rmi_spi *ts;
	#ifdef CONFIG_OF
		; // nothing
	#else
		struct shtps_platform_data *pdata = ctrl_dev_p->platform_data;
	#endif /* CONFIG_OF */

	SHTPS_LOG_FUNC_CALL();
	shtps_mutex_lock_ctrl();

	ts = kzalloc(sizeof(struct shtps_rmi_spi), GFP_KERNEL);
	if(!ts){
		SHTPS_LOG_ERR_PRINT("memory allocation error\n");
		result = -ENOMEM;
		shtps_mutex_unlock_ctrl();
		goto fail_alloc_mem;
	}

	result = shtps_fwctl_init(ts, ctrl_p, func_p);
	if(result){
		SHTPS_LOG_ERR_PRINT("no ctrl tpsdevice:error\n");
		result = -EFAULT;
		shtps_mutex_unlock_ctrl();
		goto fail_alloc_mem;
	}

	if(shtps_init_internal_variables(ts)){
		shtps_mutex_unlock_ctrl();
		goto fail_init_internal_variables;
	}

	/** set device info */
	dev_set_drvdata(ctrl_dev_p, ts);
	gShtps_rmi_spi = ts;


	#ifdef CONFIG_OF
		ts->irq_mgr.irq	= irq_of_parse_and_map(ctrl_dev_p->of_node, 0);
		ts->rst_pin		= of_get_named_gpio(ctrl_dev_p->of_node, "shtps_rmi,rst_pin", 0);
	#else
		ts->rst_pin		= pdata->gpio_rst;
		ts->irq_mgr.irq	= gpio_irq;
	#endif /* CONFIG_OF */

    if(!gpio_is_valid(ts->rst_pin)){
		SHTPS_LOG_ERR_PRINT("gpio resource error\n");
		result = -EFAULT;
		shtps_mutex_unlock_ctrl();
		goto fail_get_dtsinfo;
	}

	snprintf(ts->phys, sizeof(ts->phys), "%s", dev_name(ctrl_dev_p));

	/** setup device */
	#ifdef CONFIG_OF
		result = shtps_device_setup(ts->irq_mgr.irq, ts->rst_pin);
		if(result){
			SHTPS_LOG_ERR_PRINT("Filed shtps_device_setup\n");
			shtps_mutex_unlock_ctrl();
			goto fail_device_setup;
		}
	#else
		if (pdata && pdata->setup) {
			result = pdata->setup(ctrl_dev_p);
			if (result){
				shtps_mutex_unlock_ctrl();
				goto fail_alloc_mem;
			}
		}
	#endif /* CONFIG_OF */
	
	if(shtps_irq_resuest(ts)){
		result = -EFAULT;
		SHTPS_LOG_ERR_PRINT("shtps:request_irq error\n");
		shtps_mutex_unlock_ctrl();
		goto fail_irq_request;
	}

	shtps_mutex_unlock_ctrl();


	/** init device info */
	result = shtps_init_inputdev(ts, ctrl_dev_p, modalias);
	if(result != 0){
		SHTPS_LOG_DBG_PRINT("Failed init input device\n");
		goto fail_init_inputdev;
	}
	
	#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
		result = shtps_init_inputdev_key(ts, ctrl_dev_p);
		if(result != 0){
			SHTPS_LOG_DBG_PRINT("Failed init input key-device\n");
			goto fail_init_inputdev_key;
		}
	#endif /* defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE) */

	/** init sysfs I/F */
	shtps_init_io_debugfs(ts);

	/** init debug fs */
	shtps_init_debugfs(ts);

	#if defined(SHTPS_DYNAMIC_RESET_ESD_ENABLE)
		shtps_lvs1_regulator_init(shtpsif_device);
	#endif /* SHTPS_DYNAMIC_RESET_ESD_ENABLE */

	#if defined( SHTPS_DEVELOP_MODE_ENABLE )
	{
		struct shtps_debug_init_param param;
		param.shtps_root_kobj = ts->kobj;

		/** init debug function data */
		shtps_debug_init(&param);
	}
	#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
	    shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_ENABLE);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
	shtps_perflock_register_notifier();
#endif /*  defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE) */
	
	SHTPS_LOG_DBG_PRINT("shtps_rmi_probe() done\n");
	return 0;

#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
fail_init_inputdev_key:
	input_free_device(ts->input);
#endif /* defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE) */

fail_init_inputdev:
fail_irq_request:
fail_get_dtsinfo:
	shtps_deinit_internal_variables(ts);

fail_init_internal_variables:
fail_device_setup:
	kfree(ts);
	
fail_alloc_mem:
	return result;
}

int shtps_rmi_core_remove(struct shtps_rmi_spi *ts, struct device *ctrl_dev_p)
{
	extern void shtps_deinit_io_debugfs(struct shtps_rmi_spi *ts);
	extern void shtps_deinit_debugfs(struct shtps_rmi_spi *ts);
	extern struct device*	shtpsif_device;

	#ifndef CONFIG_OF
		struct shtps_platform_data *pdata = (struct shtps_platform_data *)&ctrl_dev_p->platform_data;
	#endif /* !CONFIG_OF */

	SHTPS_LOG_FUNC_CALL();
	gShtps_rmi_spi = NULL;

#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
	shtps_perflock_unregister_notifier();
#endif /*  defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE) */

	if(ts){
		free_irq(ts->irq_mgr.irq, ts);

		#ifdef CONFIG_OF
			shtps_device_teardown(ts->irq_mgr.irq, ts->rst_pin);
		#else
			if (pdata && pdata->teardown){
				pdata->teardown(&ctrl_dev_p);
			}
		#endif /* CONFIG_OF */

		shtps_deinit_internal_variables(ts);
		#if defined( SHTPS_DEVELOP_MODE_ENABLE )
			shtps_debug_deinit();
		#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */
		shtps_deinit_debugfs(ts);
		shtps_deinit_io_debugfs(ts);

		shtps_deinit_inputdev(ts);
		#if defined(SHTPS_PHYSICAL_KEY_ENABLE) || defined(SHTPS_LPWG_MODE_ENABLE)
			shtps_deinit_inputdev_key(ts);
		#endif /* SHTPS_PHYSICAL_KEY_ENABLE || SHTPS_LPWG_MODE_ENABLE */

		shtps_fwctl_deinit(ts);

		kfree(ts);
	}

	#if defined(SHTPS_DYNAMIC_RESET_ESD_ENABLE)
		shtps_lvs1_regulator_get(shtpsif_device);
		shtps_lvs1_regulator_remove();
	#endif /* SHTPS_DYNAMIC_RESET_ESD_ENABLE */

	_log_msg_sync( LOGMSG_ID__REMOVE_DONE, "");
	SHTPS_LOG_DBG_PRINT("shtps_rmi_remove() done\n");
	return 0;
}

int shtps_rmi_core_suspend(struct shtps_rmi_spi *ts, pm_message_t mesg)
{
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		shtps_set_suspend_state(ts);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_SYSTEM_HOT_STANDBY_ENABLE)
		_log_msg_sync( LOGMSG_ID__SUSPEND, "");

		request_event(ts, SHTPS_EVENT_SLEEP, 0);
	#else
		_log_msg_sync( LOGMSG_ID__SUSPEND, "");
	#endif /* SHTPS_SYSTEM_HOT_STANDBY_ENABLE */

	return 0;
}

int shtps_rmi_core_resume(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_SYSTEM_HOT_STANDBY_ENABLE)
		_log_msg_sync( LOGMSG_ID__RESUME, "");
	#else
		_log_msg_sync( LOGMSG_ID__RESUME, "");
	#endif /* SHTPS_SYSTEM_HOT_STANDBY_ENABLE */

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		shtps_clr_suspend_state(ts);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return 0;
}


/* -----------------------------------------------------------------------------------
 */
void msm_tps_setsleep(int on)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	_log_msg_sync( LOGMSG_ID__API_SLEEP, "%d", on);
	SHTPS_LOG_FUNC_CALL_INPARAM(on);

	if(ts){
		#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
			if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_SETSLEEP, (u8)on) == 0){
				shtps_setsleep_proc(ts, (u8)on);
			}
		#else
			if(on){
				#if defined(SHTPS_DYNAMIC_RESET_CONTROL_ENABLE)
					shtps_filter_dynamic_reset_sleep_process(ts);
				#endif /* SHTPS_DYNAMIC_RESET_CONTROL_ENABLE */

				request_event(ts, SHTPS_EVENT_SLEEP, 0);
			}else{
				#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
					ts->system_boot_mode = SH_BOOT_NORMAL;
				#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

				request_event(ts, SHTPS_EVENT_WAKEUP, 0);
			}
		#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
	}
	_log_msg_sync( LOGMSG_ID__API_SLEEP_DONE, "");
}
EXPORT_SYMBOL(msm_tps_setsleep);

void shtps_setFlipInformation(int state)
{
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	SHTPS_LOG_FUNC_CALL();
	request_event(ts, SHTPS_EVENT_FORMCHANGE, state);
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
}
EXPORT_SYMBOL(shtps_setFlipInformation);

int msm_tps_set_veilview_state_on(void)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;

	SHTPS_LOG_FUNC_CALL();

	rc = shtps_set_veilview_state(ts, 1);
	return rc;
}
EXPORT_SYMBOL(msm_tps_set_veilview_state_on);

int msm_tps_set_veilview_state_off(void)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;

	SHTPS_LOG_FUNC_CALL();

	rc = shtps_set_veilview_state(ts, 0);
	return rc;
}
EXPORT_SYMBOL(msm_tps_set_veilview_state_off);

int msm_tps_get_veilview_pattern(void)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;

	SHTPS_LOG_FUNC_CALL();

	rc = shtps_get_veilview_pattern(ts);
	return rc;
}
EXPORT_SYMBOL(msm_tps_get_veilview_pattern);

void msm_tps_set_grip_state(int on)
{
#if defined(SHTPS_LPWG_MODE_ENABLE) && defined(SHTPS_LPWG_GRIP_SUPPORT_ENABLE)
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	u8 request = (on == 0)? 0 : 1;

	SHTPS_LOG_FUNC_CALL_INPARAM(on);

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_GRIP, request) == 0){
			shtps_grip_proc(ts, request);
		}
	#else
		shtps_mutex_lock_ctrl();

		if(ts->lpwg.grip_state != request){
			ts->lpwg.grip_state = request;
			SHTPS_LOG_DBG_PRINT("[LPWG] grip_state = %d\n", ts->lpwg.grip_state);
			
			if(SHTPS_STATE_SLEEP == ts->state_mgr.state){
				u8 new_setting = shtps_is_lpwg_active(ts);
				
				if(new_setting != ts->lpwg.lpwg_switch){
					if(new_setting){
						#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
							shtps_irq_enable(ts);
						#else
							shtps_irq_wake_enable(ts);
						#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

						shtps_system_set_wakeup(ts);
						shtps_set_lpwg_mode_on(ts);
					}else{
						#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
							shtps_irq_disable(ts);
						#else
							shtps_irq_wake_disable(ts);
						#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

						shtps_set_lpwg_mode_off(ts);
						shtps_sleep(ts, 1);
						shtps_system_set_sleep(ts);
					}
				}
			}
		}
		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* SHTPS_LPWG_MODE_ENABLE && SHTPS_LPWG_GRIP_SUPPORT_ENABLE */
}
EXPORT_SYMBOL(msm_tps_set_grip_state);

#if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) || defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
int shtps_fw_update(struct shtps_rmi_spi *ts, const unsigned char *fw_data)
{
	int i;
	unsigned long blockSize;
	unsigned long blockNum;

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE, "");

	if(0 != shtps_enter_bootloader(ts)){
		SHTPS_LOG_ERR_PRINT("error - shtps_enter_bootloader()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "0");
		return -1;
	}

	if(0 != shtps_lockdown_bootloader(ts, (u8*)&fw_data[0x00d0])){
		SHTPS_LOG_ERR_PRINT("error - shtps_lockdown_bootloader()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "1");
		return -1;
	}

	if(0 != shtps_flash_erase(ts)){
		SHTPS_LOG_ERR_PRINT("error - shtps_flash_erase()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "2");
		return -1;
	}

	blockSize = shtps_fwctl_loader_get_blocksize(ts);
	blockNum  = shtps_fwctl_loader_get_firm_blocknum(ts);

	for(i = 0;i < blockNum;i++){
		if(0 != shtps_flash_writeImage(ts, (u8*)&fw_data[0x0100 + i * blockSize])){
			SHTPS_LOG_ERR_PRINT("error - shtps_flash_writeImage(%d)\n", i);
			_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "3");
			return -1;
		}
	}

	if(0 != shtps_flash_writeConfig(ts, (u8*)&fw_data[0x0100 + (blockNum * blockSize)])){
		SHTPS_LOG_ERR_PRINT("error - shtps_flash_writeConfig()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "4");
		return -1;
	}
	if(0 != shtps_exit_bootloader(ts)){
		SHTPS_LOG_ERR_PRINT("error - shtps_exit_bootloader()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "5");
		return -1;
	}
	DBG_PRINTK("fw update done\n");
	_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_DONE, "");

	return 0;
}
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) || defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */

int shtps_get_logflag(void)
{
	#if defined( SHTPS_DEVELOP_MODE_ENABLE )
		return gLogOutputEnable;
	#else
		return 0;
	#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */
}

#if defined( SHTPS_DEVELOP_MODE_ENABLE )
int shtps_read_touchevent_from_outside(void)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int ret;

	shtps_wake_lock_idle(ts);	//SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE
	shtps_mutex_lock_ctrl();

	if(ts->state_mgr.state == SHTPS_STATE_ACTIVE){
		shtps_read_touchevent(ts, SHTPS_STATE_ACTIVE);
		ret = 0;
	}
	else{
		ret = -1;
	}

	shtps_mutex_unlock_ctrl();
	shtps_wake_unlock_idle(ts);	//SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE

	return ret;
}
#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */


MODULE_DESCRIPTION("SHARP TOUCHPANEL DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
