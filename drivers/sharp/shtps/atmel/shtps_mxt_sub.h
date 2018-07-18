/* drivers/sharp/shtps/atmel/shtps_mxt_sub.h
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
#ifndef __SHTPS_MXT_SUB_H__
#define __SHTPS_MXT_SUB_H__
/* -------------------------------------------------------------------------- */
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/pm_qos.h>

#include <linux/mfd/pm8xxx/pm8921.h>

/* -------------------------------------------------------------------------- */

#include "shtps_mxt.h"

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
	int shtps_proximity_state_check(struct shtps_mxt *ts);
	int shtps_proximity_check(struct shtps_mxt *ts);
#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
	#include <linux/cpufreq.h>
	#include <linux/notifier.h>

	struct shtps_cpu_clock_ctrl_info{
		struct delayed_work			perf_lock_disable_delayed_work;
		int							report_event;
		struct shtps_mxt			*ts_p;
	};

	void shtps_perflock_register_notifier(void);
	void shtps_perflock_unregister_notifier(void);
	void shtps_perflock_enable_start(struct shtps_mxt *ts, u8 event);
	void shtps_perflock_sleep(struct shtps_mxt *ts);
	void shtps_perflock_init(struct shtps_mxt *ts);
	void shtps_perflock_deinit(struct shtps_mxt *ts);
	void shtps_perflock_set_event(struct shtps_mxt *ts, u8 event);
#else
	#define shtps_perflock_register_notifier()
	#define shtps_perflock_unregister_notifier()
	#define shtps_perflock_enable_start(A, B)
	#define shtps_perflock_sleep(A)
	#define shtps_perflock_init(A)
	#define shtps_perflock_deinit(A)
	#define shtps_perflock_set_event(A, B)
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
	struct shtps_cpu_idle_sleep_ctrl_info{
		struct pm_qos_request		qos_cpu_latency;
		int							wake_lock_idle_state;
	};

	void shtps_wake_lock_idle(struct shtps_mxt *ts);
	void shtps_wake_unlock_idle(struct shtps_mxt *ts);
	void shtps_cpu_idle_sleep_wake_lock_init( struct shtps_mxt *ts );
	void shtps_cpu_idle_sleep_wake_lock_deinit( struct shtps_mxt *ts );
#else
	#define shtps_wake_lock_idle(A)
	#define shtps_wake_unlock_idle(A)
	#define shtps_cpu_idle_sleep_wake_lock_init(A)
	#define shtps_cpu_idle_sleep_wake_lock_deinit(A)
#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_TPIN_CHECK_ENABLE ) || defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
	int shtps_tpin_enable_check(struct shtps_mxt *ts);
#endif /* SHTPS_TPIN_CHECK_ENABLE || SHTPS_CHECK_CRC_ERROR_ENABLE*/

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#endif	/* __SHTPS_MXT_SUB_H__ */
