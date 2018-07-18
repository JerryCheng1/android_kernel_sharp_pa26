/* drivers/sharp/shtps/sy3000/shtps_rmi.h
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
#ifndef __SHTPS_RMI_H__
#define __SHTPS_RMI_H__
/* --------------------------------------------------------------------------- */
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/pm_qos.h>

#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/cpuidle.h>

#include <sharp/proximity.h>
#include <sharp/shtps_dev.h>
#include <sharp/shub_driver.h>

#include "shtps_cfg.h"
#include "shtps_spi.h"
#include "shtps_fwctl.h"
#include "shtps_filter.h"
#include "shtps_rmi_debug.h"

/* ===================================================================================
 * Debug
 */

/* ===================================================================================
 * Common
 */
#define SPI_ERR_CHECK(check, label) \
	if((check)) goto label

#define SHTPS_POSTYPE_X (0)
#define SHTPS_POSTYPE_Y (1)

#define SHTPS_TOUCH_CANCEL_COORDINATES_X (0)
#define SHTPS_TOUCH_CANCEL_COORDINATES_Y (9999)

#define SHTPS_ABS_CALC(A,B)	(((A)>(B)) ? ((A)-(B)) : ((B)-(A)))

/* ===================================================================================
 * Structure / enum
 */
struct shtps_irq_info{
	int							irq;
	u8							state;
	u8							wake;
};

struct shtps_state_info{
	int							state;
	int							mode;
	int							starterr;
	unsigned long				starttime;
};

struct shtps_loader_info{
	int							ack;
	wait_queue_head_t			wait_ack;
};

struct shtps_diag_info{
	u8							pos_mode;
	u8							tm_mode;
	u8							tm_data[SHTPS_TM_TXNUM_MAX * SHTPS_TM_RXNUM_MAX * 2];
	int							event;
#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
	int							event_touchkey;
#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) */
	int							tm_ack;
	int							tm_stop;
	wait_queue_head_t			wait;
	wait_queue_head_t			tm_wait_ack;
};

struct shtps_facetouch_info{
	int							mode;
	int							state;
	int							detect;
	int							palm_thresh;
	int							wake_sig;
	u8							palm_det;
	u8							touch_num;
	wait_queue_head_t			wait_off;
	
	u8							wakelock_state;
	struct wake_lock            wake_lock;
	struct pm_qos_request		qos_cpu_latency;
	
	
#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
	int							facetouch_off_notify_delayed_state;
	int							facetouch_off_force_flg;
	struct wake_lock            facetouch_off_delayed_wake_lock;
	struct pm_qos_request		facetouch_off_delayed_qos;
#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */
	
#if defined( SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE )
	int							rezero_disable;
#endif /* SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE */
	
};

struct shtps_offset_info{
	int							enabled;
	u16							base[5];
	signed short				diff[12];
};

struct shtps_polling_info{
	int							boot_rezero_flag;
	int							stop_margin;
	int							stop_count;
	int							single_fingers_count;
	int							single_fingers_max;
	u8							single_fingers_enable;
};

struct shtps_lpwg_ctrl{		//**********	SHTPS_LPWG_MODE_ENABLE
	u8							lpwg_switch;
	u8							lpwg_state;
	u8							notify_enable;

	struct wake_lock			wake_lock;
	struct pm_qos_request		pm_qos_lock_idle;
	signed long					pm_qos_idle_value;
	struct delayed_work			notify_interval_delayed_work;
	u8							block_touchevent;
	unsigned long				wakeup_time;
	u8							tu_rezero_req;
	
	#if defined( SHTPS_LPWG_GRIP_SUPPORT_ENABLE )
		u8							grip_state;
	#endif /* SHTPS_LPWG_GRIP_SUPPORT_ENABLE */

	#if defined( SHTPS_HOST_LPWG_MODE_ENABLE )
		struct shtps_touch_info		pre_info;
		unsigned long				swipe_check_time;
	#endif /* SHTPS_HOST_LPWG_MODE_ENABLE */

};


struct shtps_touch_pos_info{
	unsigned short	x;
	unsigned short	y;
};

struct shtps_touch_hist_info{
	unsigned short	x;
	unsigned short	y;
	unsigned char	state;
	unsigned char	wx;
	unsigned char	wy;
	unsigned char	z;
};

enum{	//**********	SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE
	SHTPS_DETER_SUSPEND_SPI_PROC_IRQ = 0x00,
	SHTPS_DETER_SUSPEND_SPI_PROC_CHARGER_ARMOR,
	SHTPS_DETER_SUSPEND_SPI_PROC_SETSLEEP,
	SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPWG,
	SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPMODE,
	SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETCONLPMODE,
	SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLCDBRIGHTLPMODE,
	SHTPS_DETER_SUSPEND_SPI_PROC_OPEN,
	SHTPS_DETER_SUSPEND_SPI_PROC_CLOSE,
	SHTPS_DETER_SUSPEND_SPI_PROC_ENABLE,
	SHTPS_DETER_SUSPEND_SPI_PROC_GRIP,
	SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETHOVER,
	SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETPEN,

	SHTPS_DETER_SUSPEND_SPI_PROC_NUM,
};

struct shtps_deter_suspend_spi{	//**********	SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE
	u8							suspend;
	struct work_struct			pending_proc_work;
	u8							wake_lock_state;
	struct wake_lock			wake_lock;
	struct pm_qos_request		pm_qos_lock_idle;
	
	#ifdef SHTPS_DEVELOP_MODE_ENABLE
		struct delayed_work			pending_proc_work_delay;
	#endif /* SHTPS_DEVELOP_MODE_ENABLE */
	
	struct shtps_deter_suspend_spi_pending_info{
		u8						pending;
		u8						param;
	} pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_NUM];

	u8							suspend_irq_state;
	u8							suspend_irq_wake_state;
	u8							suspend_irq_detect;
};

/* -------------------------------------------------------------------------- */
struct shtps_rmi_spi {
	struct input_dev*			input;
	int							rst_pin;
	struct shtps_irq_info		irq_mgr;

	struct shtps_touch_info		fw_report_info;
	struct shtps_touch_info		fw_report_info_store;
	struct shtps_touch_info		report_info;

	struct shtps_state_info		state_mgr;
	struct shtps_loader_info	loader;
	struct shtps_diag_info		diag;
	struct shtps_facetouch_info	facetouch;
	struct shtps_polling_info	poll_info;
	struct shtps_touch_state	touch_state;
	wait_queue_head_t			wait_start;
	struct delayed_work 		tmo_check;
	unsigned char				finger_state[3];	/* SHTPS_FINGER_MAX/4+1 */
	struct hrtimer				rezero_delayed_timer;
	struct work_struct			rezero_delayed_work;
	u16							bt_ver;
	char						phys[32];
	struct kobject				*kobj;


	struct shtps_offset_info	offset;
	#if defined(SHTPS_PEN_DETECT_ENABLE)
		struct shtps_offset_info	offset_pen;
		u8							pen_enable;
	#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */

	#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
		struct hrtimer				facetouch_off_notify_delayed_timer;
	#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */

	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		u8							lpmode_req_state;
		u8							lpmode_continuous_req_state;
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
		struct input_dev*			input_key;
		u16							keycodes[2];
		u8							key_state;
	#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE) */

	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		u16							system_boot_mode;
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	#if defined( SHTPS_FINGER_WIDTH_MODERATION_ENABLE )
	#endif	/* SHTPS_FINGER_WIDTH_MODERATION_ENABLE */

	#if defined(SHTPS_HOVER_DETECT_ENABLE)
		u8							hover_ctrl_base_adr;
		u8							hover_enable_state;
	#endif /* SHTPS_HOVER_DETECT_ENABLE */

	#if defined( SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE )
	#endif /* SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		struct shtps_lpwg_ctrl		lpwg;
		u8							lpwg_hover_enable_state_sotre;

	#endif /* SHTPS_LPWG_MODE_ENABLE */

	#if defined(SHTPS_MULTI_FW_ENABLE)
		u8							multi_fw_type;
	#endif /* SHTPS_MULTI_FW_ENABLE */

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		struct shtps_deter_suspend_spi		deter_suspend_spi;
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
		u8							wakeup_touch_event_inhibit_state;
	#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */

	#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )
		u8							key_down_reserved;
		u8							key_down_ignored;
		u8							key_proximity_check_state;
		struct delayed_work			touchkey_delayed_work;
		struct delayed_work			touchkey_inproxymity_delayed_work;
	#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */

	#if defined(SHTPS_FINGER_KEY_EXCLUDE_ENABLE)
		u8							exclude_touch_disable_check_state;
		u8							exclude_key_disable_check_state;
		u16							exclude_touch_disable_finger;
		unsigned long				exclude_touch_disable_check_time;
		unsigned long				exclude_key_disable_check_time;
	#endif /* SHTPS_FINGER_KEY_EXCLUDE_ENABLE */

	#if defined( SHTPS_FINGER_WIDTH_MODERATION_ENABLE )
		int							w_before_gain[SHTPS_FINGER_MAX];
	#endif	/* SHTPS_FINGER_WIDTH_MODERATION_ENABLE */

	#if defined(SHTPS_LOW_REPORTRATE_MODE)
		u8							low_report_rate_mode_state;
	#endif /* SHTPS_LOW_REPORTRATE_MODE */


	#if defined(SHTPS_PALM_HOST_DETECT_TOUCH_UP_ENABLE)
		int 						host_palm_detect;
		int 						host_palm_detect_wait_all_touchup;
	#endif /* SHTPS_PALM_HOST_DETECT_TOUCH_UP_ENABLE */


	/* ------------------------------------------------------------------------ */
	struct shtps_filter_drag_hist_info						*drag_hist_p;

	/* ------------------------------------------------------------------------ */
	/* acync */
	struct workqueue_struct		*workqueue_p;
	struct work_struct			work_data;
	struct list_head			queue;
	spinlock_t					queue_lock;
	struct shtps_req_msg		*cur_msg_p;

	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		struct shtps_cpu_clock_ctrl_info					*cpu_clock_ctrl_p;
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

	#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
		struct shtps_cpu_idle_sleep_ctrl_info				*cpu_idle_sleep_ctrl_p;
	#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

	#if defined(SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE)
		struct shtps_cpu_sleep_ctrl_fwupdate_info			*cpu_sleep_ctrl_fwupdate_p;
	#endif /* SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE */

	/* ------------------------------------------------------------------------ */
	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		struct shtps_filter_charger_armor_info				*charger_armor_p;
	#endif /* #if defined( SHTPS_CHARGER_ARMOR_ENABLE ) */

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		struct shtps_filter_absorption_check_info			*absorption_check_p;
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */

	#if defined( SHTPS_INVERTING_GHOST_REJECTION_ENABLE )
		struct shtps_filter_inverting_check_info			*inverting_check_p;
	#endif /* SHTPS_INVERTING_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		struct shtps_filter_edge_fail_touch_rej_info		*edge_fail_touch_rej_p;
	#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
		struct shtps_filter_grip_fail_info					*filter_grip_fail_p;
	#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
		struct shtps_filter_pinch_fail_reject_info			*pinch_fail_reject_p;
	#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

	#if defined(SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE)
		struct shtps_filter_lgm_split_touch_combining_info	*lgm_split_touch_combining_p;
	#endif /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */
	
	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE) || defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE) || defined(SHTPS_HOVER_REJECT_ENABLE)
		struct shtps_filter_correct_info					*filter_correct_p;
	#endif

	#if defined(SHTPS_CLING_REJECTION_ENABLE)
		struct shtps_filter_cling_reject_info				*cling_reject_p;
	#endif /* SHTPS_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		struct shtps_filter_topedge_fail_info				*topedge_fail_p;
	#endif /* SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_MULTI_HOVER_SELECT_ENABLE)
		struct shtps_filter_multi_hover_select_info			*multi_hover_select_p;
	#endif /* SHTPS_MULTI_HOVER_SELECT_ENABLE */

	#if defined(SHTPS_HOVER_HOST_JITTER_FILTER_ENABLE)
		struct shtps_filter_hover_jitter_filter_info		*hover_jitter_filter_p;
	#endif /* SHTPS_HOVER_HOST_JITTER_FILTER_ENABLE */

	#if defined(SHTPS_DIAGONAL_GHOST_CHECK_ENABLE)
		struct shtps_filter_diagonal_ghost_reject_info		*diagonal_ghost_reject_p;
	#endif /* SHTPS_DIAGONAL_GHOST_CHECK_ENABLE */

	#if defined( SHTPS_DYNAMIC_RESET_CONTROL_ENABLE )
		struct shtps_filter_dynamic_reset_info				*dynamic_reset_p;
	#endif /* SHTPS_DYNAMIC_RESET_CONTROL_ENABLE */

	#if defined(SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE)
		struct multitouch_pen_ghost_rejection_info			*multitouch_pen_ghost_rejection_p;
	#endif /* SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_PEN_POS_JUMP_REJECT_ENABLE)
		struct shtps_filter_pen_pos_jump_info				*pen_pos_jump_p;
	#endif /* SHTPS_PEN_POS_JUMP_REJECT_ENABLE */

	#if defined( SHTPS_COAXIAL_GHOST_REJECTION_ENABLE )
		struct shtps_filter_coaxial_ghost_reject_info		*coaxial_ghost_reject_p;
	#endif /* SHTPS_COAXIAL_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_WATER_GHOST_REJECTION_ENABLE)
		struct shtps_filter_water_ghost_reject_info			*water_ghost_reject_p;
	#endif /* SHTPS_WATER_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_WATER_CLING_REJECTION_ENABLE)
		struct shtps_filter_water_cling_reject_info			*water_cling_reject_p;
	#endif /* SHTPS_WATER_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE)
		struct shtps_filter_multitap_fail_move_reject_info	*multitap_fail_move_reject_p;
	#endif /* SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE */

	#if defined(SHTPS_GHOST_REJECTION_ENABLE)
		struct shtps_filter_detect_ghost_info				*detect_ghost_p;
	#endif	/* SHTPS_GHOST_REJECTION_ENABLE */

	#if defined( SHTPS_VARIABLE_PEN_JITTER_ENABLE )
		struct shtps_filter_pen_jitter_filter_info			*pen_jitter_filter_p;
	#endif /* SHTPS_VARIABLE_PEN_JITTER_ENABLE */

	#if defined( SHTPS_PEN_CLING_REJECTION_ENABLE )
		struct shtps_filter_pen_cling_reject_info			*pen_cling_reject_p;
	#endif /* SHTPS_PEN_CLING_REJECTION_ENABLE */

	#if defined( SHTPS_SHIFT_EDGE_INWARD_ENABLE )
		struct shtps_filter_shift_edge_inward_info			*shift_edge_inward_p;
	#endif /* SHTPS_SHIFT_EDGE_INWARD_ENABLE */

	#if defined(SHTPS_ELECTRICAL_GHOST_REJECTION_ENABLE)
		struct shtps_filter_electrical_ghost_reject_info	*electrical_ghost_reject_p;
	#endif /* SHTPS_ELECTRICAL_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
		struct shtps_filter_topofscreen_ghost_reject_info	*topofscreen_ghost_reject_p;
	#endif /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

	/* ------------------------------------------------------------------------ */
	struct device				*ctrl_dev_p;
	struct shtps_fwctl_info	*fwctl_p;
};

/* ----------------------------------------------------------------------------
*/
enum{
	SHTPS_FWTESTMODE_V01 = 0x00,
	SHTPS_FWTESTMODE_V02,
	SHTPS_FWTESTMODE_V03,
};

enum{
	SHTPS_REZERO_REQUEST_REZERO				= 0x01,
	SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE	= 0x02,
	SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE	= 0x04,
	SHTPS_REZERO_REQUEST_WAKEUP_REZERO		= 0x08,
};

enum{
	SHTPS_REZERO_HANDLE_EVENT_MTD = 0,
	SHTPS_REZERO_HANDLE_EVENT_TOUCH,
	SHTPS_REZERO_HANDLE_EVENT_TOUCHUP,
};

enum{
	SHTPS_REZERO_TRIGGER_BOOT = 0,
	SHTPS_REZERO_TRIGGER_WAKEUP,
	SHTPS_REZERO_TRIGGER_ENDCALL,
};

enum{
	SHTPS_EVENT_TU,
	SHTPS_EVENT_TD,
	SHTPS_EVENT_DRAG,
	SHTPS_EVENT_MTDU,
};

enum{
	SHTPS_TOUCH_STATE_NO_TOUCH = 0,
	SHTPS_TOUCH_STATE_FINGER,
	SHTPS_TOUCH_STATE_PEN,
	SHTPS_TOUCH_STATE_PALM,
	SHTPS_TOUCH_STATE_UNKNOWN,
	SHTPS_TOUCH_STATE_HOVER,
};

enum{
	SHTPS_STARTUP_SUCCESS,
	SHTPS_STARTUP_FAILED
};

enum{
	SHTPS_IRQ_WAKE_DISABLE,
	SHTPS_IRQ_WAKE_ENABLE,
};

enum{
	SHTPS_IRQ_STATE_DISABLE,
	SHTPS_IRQ_STATE_ENABLE,
};

enum{
	SHTPS_MODE_NORMAL,
	SHTPS_MODE_LOADER,
};

enum{
	SHTPS_EVENT_START,
	SHTPS_EVENT_STOP,
	SHTPS_EVENT_SLEEP,
	SHTPS_EVENT_WAKEUP,
	SHTPS_EVENT_STARTLOADER,
	SHTPS_EVENT_STARTTM,
	SHTPS_EVENT_STOPTM,
	SHTPS_EVENT_FACETOUCHMODE_ON,
	SHTPS_EVENT_FACETOUCHMODE_OFF,
	SHTPS_EVENT_INTERRUPT,
	SHTPS_EVENT_TIMEOUT,
};

enum{
	SHTPS_STATE_IDLE,
	SHTPS_STATE_WAIT_WAKEUP,
	SHTPS_STATE_WAIT_READY,
	SHTPS_STATE_ACTIVE,
	SHTPS_STATE_BOOTLOADER,
	SHTPS_STATE_FACETOUCH,
	SHTPS_STATE_FWTESTMODE,
	SHTPS_STATE_SLEEP,
	SHTPS_STATE_SLEEP_FACETOUCH,
};

enum{
	SHTPS_PHYSICAL_KEY_DOWN = 0,
	SHTPS_PHYSICAL_KEY_UP,
	SHTPS_PHYSICAL_KEY_NUM,
};

enum{
	SHTPS_IRQ_FLASH		= 0x01,
	SHTPS_IRQ_STATE		= 0x02,
	SHTPS_IRQ_ABS 		= 0x04,
	SHTPS_IRQ_ANALOG 	= 0x08,
	SHTPS_IRQ_BUTTON 	= 0x10,
	SHTPS_IRQ_SENSOR 	= 0x20,
	SHTPS_IRQ_ALL		= (  SHTPS_IRQ_FLASH
							| SHTPS_IRQ_STATE
							| SHTPS_IRQ_ABS
							| SHTPS_IRQ_ANALOG
							| SHTPS_IRQ_BUTTON
							| SHTPS_IRQ_SENSOR),
};

enum{
	SHTPS_LPMODE_TYPE_NON_CONTINUOUS = 0,
	SHTPS_LPMODE_TYPE_CONTINUOUS,
};

enum{
	SHTPS_LPMODE_REQ_NONE		= 0x00,
	SHTPS_LPMODE_REQ_COMMON		= 0x01,
	SHTPS_LPMODE_REQ_ECO		= 0x02,
	SHTPS_LPMODE_REQ_HOVER_OFF	= 0x04,	
	SHTPS_LPMODE_REQ_LCD_BRIGHT	= 0x08,
};

enum{
	SHTPS_FUNC_REQ_EVEMT_OPEN = 0,
	SHTPS_FUNC_REQ_EVEMT_CLOSE,
	SHTPS_FUNC_REQ_EVEMT_ENABLE,
	SHTPS_FUNC_REQ_EVEMT_DISABLE,
	SHTPS_FUNC_REQ_EVEMT_CHECK_CRC_ERROR,
	SHTPS_FUNC_REQ_EVEMT_PROXIMITY_CHECK,
};

enum{
	SHTPS_DRAG_DIR_NONE = 0,
	SHTPS_DRAG_DIR_PLUS,
	SHTPS_DRAG_DIR_MINUS,
};

enum{
	SHTPS_LPWG_DETECT_GESTURE_TYPE_NONE			= 0x00,
	SHTPS_LPWG_DETECT_GESTURE_TYPE_SWIPE		= 0x01,
	SHTPS_LPWG_DETECT_GESTURE_TYPE_DOUBLE_TAP	= 0x02,
};

enum{
	SHTPS_LPWG_STATE_OFF		= 0x00,
	SHTPS_LPWG_STATE_ON			= 0x01,
	SHTPS_LPWG_STATE_GRIP_ONLY	= 0x02,
};

enum{
	SHTPS_HW_TYPE_BOARD = 0,
	SHTPS_HW_TYPE_HANDSET,
};

enum{
	SHTPS_HW_REV_ES_0 = 0,
	SHTPS_HW_REV_ES_1,
	SHTPS_HW_REV_PP_1,
	SHTPS_HW_REV_PP_2,
	SHTPS_HW_REV_PMP,
	SHTPS_HW_REV_MP,
};

#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
enum{
	SHTPS_PROXIMITY_ENABLE = SH_PROXIMITY_ENABLE,
	SHTPS_PROXIMITY_DISABLE = SH_PROXIMITY_DISABLE,
};

enum{
	SHTPS_PROXIMITY_NEAR = SH_PROXIMITY_NEAR,
	SHTPS_PROXIMITY_FAR = SH_PROXIMITY_FAR,
};
#endif

enum{
	SHTPS_DEV_STATE_SLEEP = 0,
	SHTPS_DEV_STATE_DOZE,
	SHTPS_DEV_STATE_ACTIVE,
	SHTPS_DEV_STATE_LPWG,
	SHTPS_DEV_STATE_LOADER,
	SHTPS_DEV_STATE_TESTMODE,
};

/* ----------------------------------------------------------------------------
*/
extern struct shtps_rmi_spi*	gShtps_rmi_spi;

void shtps_mutex_lock_ctrl(void);
void shtps_mutex_unlock_ctrl(void);
void shtps_mutex_lock_loader(void);
void shtps_mutex_unlock_loader(void);
void shtps_mutex_lock_proc(void);
void shtps_mutex_unlock_proc(void);
void shtps_mutex_lock_facetouch_qos_ctrl(void);
void shtps_mutex_unlock_facetouch_qos_ctrl(void);
#if defined(SHTPS_CHECK_HWID_ENABLE)
	int shtps_system_get_hw_revision(void);
#endif /* SHTPS_CHECK_HWID_ENABLE */

void shtps_system_set_sleep(struct shtps_rmi_spi *ts);
void shtps_system_set_wakeup(struct shtps_rmi_spi *ts);
#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
	void shtps_charger_armor_proc(struct shtps_rmi_spi *ts, u8 charger);
	void shtps_ioctl_setlpwg_proc(struct shtps_rmi_spi *ts, u8 on);
	void shtps_ioctl_setlpmode_proc(struct shtps_rmi_spi *ts, u8 on);
	void shtps_ioctl_setconlpmode_proc(struct shtps_rmi_spi *ts, u8 on);
	void shtps_ioctl_setlcdbrightlpmode_proc(struct shtps_rmi_spi *ts, u8 on);
	void shtps_ioctl_setpen_proc(struct shtps_rmi_spi *ts, u8 on);
	int shtps_check_suspend_state(struct shtps_rmi_spi *ts, int proc, u8 param);
	void shtps_set_suspend_state(struct shtps_rmi_spi *ts);
	void shtps_clr_suspend_state(struct shtps_rmi_spi *ts);
	void shtps_ioctl_sethover_proc(struct shtps_rmi_spi *ts, u8 on);
#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
void shtps_rezero(struct shtps_rmi_spi *ts);

void shtps_rezero_request(struct shtps_rmi_spi *ts, u8 request, u8 trigger);
int shtps_start(struct shtps_rmi_spi *ts);
void shtps_shutdown(struct shtps_rmi_spi *ts);

void shtps_reset(struct shtps_rmi_spi *ts);



int shtps_fwdate(struct shtps_rmi_spi *ts, u8 *year, u8 *month);
int shtps_get_serial_number(struct shtps_rmi_spi *ts, u8 *buf);
u16 shtps_fwver(struct shtps_rmi_spi *ts);
u16 shtps_fwver_builtin(struct shtps_rmi_spi *ts);
int shtps_fwsize_builtin(struct shtps_rmi_spi *ts);
unsigned char* shtps_fwdata_builtin(struct shtps_rmi_spi *ts);

int shtps_wait_startup(struct shtps_rmi_spi *ts);
int shtps_get_fingermax(struct shtps_rmi_spi *ts);
int shtps_get_diff(unsigned short pos1, unsigned short pos2, unsigned long factor);
int shtps_get_fingerwidth(struct shtps_rmi_spi *ts, int num, struct shtps_touch_info *info);
void shtps_set_eventtype(u8 *event, u8 type);
void shtps_report_touch_on(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z);
void shtps_report_touch_off(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z);
#if defined(SHTPS_PEN_DETECT_ENABLE)
	void shtps_report_touch_pen_on(struct shtps_rmi_spi *ts, int pen, int x, int y, int w, int wx, int wy, int z);
	void shtps_report_touch_pen_off(struct shtps_rmi_spi *ts, int pen, int x, int y, int w, int wx, int wy, int z);
#endif /* SHTPS_PEN_DETECT_ENABLE */

void shtps_event_report(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 event);

#if defined(SHTPS_LPWG_MODE_ENABLE)
	void shtps_set_lpwg_mode_on(struct shtps_rmi_spi *ts);
	void shtps_set_lpwg_mode_off(struct shtps_rmi_spi *ts);
	int shtps_is_lpwg_active(struct shtps_rmi_spi *ts);
	#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
		void shtps_notify_cancel_wakeup_event(struct shtps_rmi_spi *ts);
	#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */
#endif /* SHTPS_LPWG_MODE_ENABLE */
int shtps_get_tm_rxsize(struct shtps_rmi_spi *ts);
int shtps_get_tm_txsize(struct shtps_rmi_spi *ts);
int shtps_baseline_offset_disable(struct shtps_rmi_spi *ts);
void shtps_read_tmdata(struct shtps_rmi_spi *ts, u8 mode);

void shtps_irq_disable(struct shtps_rmi_spi *ts);
void shtps_irq_enable(struct shtps_rmi_spi *ts);

void shtps_read_touchevent(struct shtps_rmi_spi *ts, int state);
#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
	int shtps_boot_fwupdate_enable_check(struct shtps_rmi_spi *ts);
	#if defined(SHTPS_BOOT_FWUPDATE_FORCE_UPDATE)
		//nothing
	#else
		int shtps_fwup_flag_check(void);
	#endif /* #if !defined(SHTPS_BOOT_FWUPDATE_FORCE_UPDATE) */

	void shtps_fwup_flag_clear(void);
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

int shtps_enter_bootloader(struct shtps_rmi_spi *ts);
int shtps_lockdown_bootloader(struct shtps_rmi_spi *ts, u8* fwdata);
int shtps_flash_erase(struct shtps_rmi_spi *ts);
int shtps_flash_writeImage(struct shtps_rmi_spi *ts, u8 *fwdata);
int shtps_flash_writeConfig(struct shtps_rmi_spi *ts, u8 *fwdata);
int shtps_set_veilview_state(struct shtps_rmi_spi *ts, unsigned long arg);
int shtps_get_veilview_pattern(struct shtps_rmi_spi *ts);
int request_event(struct shtps_rmi_spi *ts, int event, int param);

void shtps_event_force_touchup(struct shtps_rmi_spi *ts);

#if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) || defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
int shtps_fw_update(struct shtps_rmi_spi *ts, const unsigned char *fw_data);
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) || defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */
int shtps_get_logflag(void);
#if defined( SHTPS_DEVELOP_MODE_ENABLE )
int shtps_read_touchevent_from_outside(void);
#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */

#if defined(SHTPS_HOVER_DETECT_ENABLE)
int shtps_set_hover_detect_enable(struct shtps_rmi_spi *ts);
int shtps_set_hover_detect_disable(struct shtps_rmi_spi *ts);
#endif /* SHTPS_HOVER_DETECT_ENABLE */

#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	void shtps_set_lpmode(struct shtps_rmi_spi *ts, int type, int req, int on);
#endif	/* SHTPS_LOW_POWER_MODE_ENABLE */

#if defined(SHTPS_HOVER_DETECT_ENABLE)
	void shtps_report_touch_hover_on(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z);
	void shtps_report_touch_hover_off(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z);
#endif /* SHTPS_HOVER_DETECT_ENABLE */

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	void shtps_facetouch_wakelock(struct shtps_rmi_spi *ts, u8 on);
#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */


#if defined(SHTPS_HOST_LPWG_MODE_ENABLE)
	int shtps_check_host_lpwg_enable(void);
#endif /* SHTPS_HOST_LPWG_MODE_ENABLE */



void shtps_sleep(struct shtps_rmi_spi *ts, int on);
int shtps_check_set_doze_enable(void);

#endif /* __SHTPS_RMI_H__ */
