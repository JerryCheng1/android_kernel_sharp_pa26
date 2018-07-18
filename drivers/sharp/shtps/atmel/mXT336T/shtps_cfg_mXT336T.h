/* drivers/sharp/shtps/atmel/mXT336T/shtps_cfg_mXT336T.h
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
#ifndef __SHTPS_CFG_MXT336T_H__
#define __SHTPS_CFG_MXT336T_H__

/* ===================================================================================
 * [ Debug ]
 */
#define SHTPS_DEVELOP_MODE_ENABLE

#ifdef	SHTPS_DEVELOP_MODE_ENABLE
	#define SHTPS_LOG_DEBUG_ENABLE
	#define	SHTPS_LOG_EVENT_ENABLE
	#define	SHTPS_MODULE_PARAM_ENABLE
	#define	SHTPS_DEBUG_VARIABLE_DEFINES
	#define SHTPS_CREATE_KOBJ_ENABLE
#endif

#define	SHTPS_LOG_ERROR_ENABLE

#ifdef SHTPS_LOG_EVENT_ENABLE
	#define SHTPS_LOG_OUTPUT_SWITCH_ENABLE
#endif /* #if defined( SHTPS_LOG_EVENT_ENABLE ) */

/* ===================================================================================
 * [ Diag ]
 */
#ifdef	SHTPS_FACTORY_MODE_ENABLE
	#define SHTPS_TPIN_CHECK_ENABLE
#else
	#undef	SHTPS_TPIN_CHECK_ENABLE
#endif

/* ===================================================================================
 * [ Model specifications ]
 */
#define CONFIG_SHTPS_PANEL_SIZE_X				1080
#define CONFIG_SHTPS_PANEL_SIZE_Y				1920
#define CONFIG_SHTPS_LCD_SIZE_X					CONFIG_SHTPS_PANEL_SIZE_X
#define CONFIG_SHTPS_LCD_SIZE_Y					CONFIG_SHTPS_PANEL_SIZE_Y

//#define SHTPS_POS_SCALING_ENABLE
#define SHTPS_POSITION_OFFSET

#define SHTPS_LPWG_MODE_ENABLE
#define SHTPS_LOW_POWER_MODE_ENABLE

#define SHTPS_QOS_LATENCY_DEF_VALUE	 			34

/* ===================================================================================
 * [ Firmware control ]
 */
#define SHTPS_POS_SCALE_X(ts)	(((CONFIG_SHTPS_PANEL_SIZE_X) * 10000) / shtps_fwctl_get_maxXPosition(ts))
#define SHTPS_POS_SCALE_Y(ts)	(((CONFIG_SHTPS_PANEL_SIZE_Y) * 10000) / shtps_fwctl_get_maxYPosition(ts))

/* ===================================================================================
 * [ Hardware specifications ]
 */
#define SHTPS_GPIO_TPIN_NO					96

#define SHTPS_PROXIMITY_SUPPORT_ENABLE

#ifdef CONFIG_SHUB_ML630Q790
	#define SHTPS_MOTION_PEDO_STOP_REQ_ENABLE
#endif

/* ===================================================================================
 * [ Performance ]
 */
#define SHTPS_CPU_CLOCK_CONTROL_ENABLE
#define SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE

#define SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE

/* ===================================================================================
 * [ Standard ]
 */
#define SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE

/* ===================================================================================
 * [ Host functions ]
 */
#if defined(SHTPS_LPWG_MODE_ENABLE)
	#define SHTPS_LPWG_GRIP_SUPPORT_ENABLE
#endif /* SHTPS_LPWG_MODE_ENABLE */

#endif /* __SHTPS_CFG_MXT336T_H__ */
