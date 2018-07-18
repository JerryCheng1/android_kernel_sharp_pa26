/* drivers/sharp/atmel/shtps_mxt_debug.h
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

#ifndef __SHTPS_MXT_DEBUG_H__
#define __SHTPS_MXT_DEBUG_H__

/* -----------------------------------------------------------------------------------
 */
#ifdef	SHTPS_DEVELOP_MODE_ENABLE
	#define SHTPS_TOUCH_EMURATOR_ENABLE
//	#define SHTPS_TOUCH_EVENTLOG_ENABLE
#endif

/* -----------------------------------------------------------------------------------
 */
struct shtps_debug_init_param{
	struct kobject		*shtps_root_kobj;
};

#define SHTPS_DEBUG_PARAM_NAME_ONOFF	"enable"
#define SHTPS_DEBUG_PARAM_NAME_LOG		"log"

struct shtps_debug_param_regist_info{
	const char *parent_name;
	const char *param_name;
	int *param_p;
	ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
};

int shtps_debug_param_add(struct shtps_debug_param_regist_info *info_p);


/* -----------------------------------------------------------------------------------
 */
int shtps_debug_init(struct shtps_debug_init_param *param);
void shtps_debug_deinit(void);
void shtps_debug_sleep_enter(void);
void shtps_debug_notify_make_input_dev(struct input_dev *input_dev);
void shtps_debug_notify_free_input_dev(void);
#if defined ( SHTPS_TOUCH_EMURATOR_ENABLE )
	int shtps_touch_emu_is_running(void);
	int shtps_touch_emu_is_recording(void);
	int shtps_touch_emu_rec_finger_info(int id, int state, int tool, int x, int y, int w, int z, int orientation);
	int shtps_touch_emu_sync_rec_finger(void);
	void shtps_touch_emu_stop(void);
#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */
#if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE )
	int shtps_touch_eventlog_is_recording(void);
	int shtps_touch_eventlog_rec_event_info(int state, int finger, int x, int y, int w, int wx, int wy, int z);
#endif /* #if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE ) */

#endif /* __SHTPS_MXT_DEBUG_H__ */
