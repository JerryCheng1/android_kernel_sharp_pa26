/* drivers/sharp/shtps/atmel/mXT336T/shtps_param_mXT336T.h
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
#ifndef __SHTPS_PARAM_MXT336T_H__
#define __SHTPS_PARAM_MXT336T_H__

/* ===================================================================================
 * [ Parameters ]
 */
/* -----------------------------------------------------------------------------------
 */
SHTPS_PARAM_DEF( SHTPS_VEILVIEW_PATTERN, 	SHTPS_VEILVIEW_PATTERN_MONOCHROME_1H);

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
	#define SHTPS_PERF_LOCK_CLOCK_FREQUENCY				1497600
	SHTPS_PARAM_DEF( SHTPS_PERF_LOCK_ENABLE_TIME_MS, 	100);
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */


/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_LPWG_MODE_ENABLE)
	#define SHTPS_LPWG_SYMDATA				\
			0x02,0x00,0x00,0x4C,0x23,0x02,0x00,0x00,0x52,0x24,0x02,0x00,0x00,0x55,0x25,0x02,0x00,0x00,0x44,0x26

	SHTPS_PARAM_DEF( SHTPS_LPWG_IDLEACQINT_VALUE, 			0xFE);
	SHTPS_PARAM_DEF( SHTPS_LPWG_ACTVACQINT_VALUE, 			0x64);

	SHTPS_PARAM_DEF( SHTPS_LPWG_GRIP_SUPPRESSION_X_MM, 		3);
	SHTPS_PARAM_DEF( SHTPS_LPWG_GRIP_SUPPRESSION_Y_MM, 		0);
	SHTPS_PARAM_DEF( SHTPS_LPWG_MOV_MM, 					25);
	SHTPS_PARAM_DEF( SHTPS_LPWG_SYM_TIMEMAX_MS, 			800);

	SHTPS_PARAM_DEF( SHTPS_LPWG_QOS_LATENCY_DEF_VALUE, 		SHTPS_QOS_LATENCY_DEF_VALUE);
	SHTPS_PARAM_DEF( SHTPS_LPWG_MIN_NOTIFY_INTERVAL, 		1000);

	#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
		#include <sharp/proximity.h>
		SHTPS_PARAM_DEF( SHTPS_LPWG_PROXIMITY_SUPPORT_ENABLE, 	1);
		SHTPS_PARAM_DEF( SHTPS_LPWG_MIN_NOTIFY_CANCEL_INTERVAL, 800);
		SHTPS_PARAM_DEF( SHTPS_LPWG_PROXIMITY_CHECK_PREWAIT, 	100);
	#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */
#endif /* SHTPS_LPWG_MODE_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_LOW_POWER_MODE_ENABLE)
	SHTPS_PARAM_DEF( SHTPS_LPMODE_IDLEACQINT_VALUE,			0x32);
	SHTPS_PARAM_DEF( SHTPS_LPMODE_ACTVACQINT_VALUE, 		0x0B);
#endif /* SHTPS_LOW_POWER_MODE_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#endif /* __SHTPS_PARAM_MXT336T_H__ */
