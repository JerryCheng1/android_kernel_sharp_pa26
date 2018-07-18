/* include/sharp/sh_bootlog.h
 *
 * Copyright (C) 2013 Sharp Corporation
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

#ifndef __SH_BOOTLOG_H__
#define __SH_BOOTLOG_H__
/*===========================================================================
INCLUDE
===========================================================================*/
#include <mach/msm_iomap.h>
#include <asm/io.h>

/*===========================================================================
ENUM
===========================================================================*/
typedef enum pass_point{
/* Unavailable */
	SH_BTLG_POINT_RESERVE_00		= 0x00000000,

/* APPSBL */
	SH_BTLG_POINT_APPSBL_IN_01		= 0x00000001,
	SH_BTLG_POINT_APPSBL_IN_02		= 0x00000002,
	SH_BTLG_POINT_APPSBL_IN_03		= 0x00000004,
	SH_BTLG_POINT_APPSBL_IN_04		= 0x00000008,
	SH_BTLG_POINT_APPSBL_IN_05		= 0x00000010,
	SH_BTLG_POINT_APPSBL_IN_06		= 0x00000020,
	SH_BTLG_POINT_APPSBL_IN_07		= 0x00000040,
	SH_BTLG_POINT_APPSBL_IN_08		= 0x00000080,
	SH_BTLG_POINT_APPSBL_IN_09		= 0x00000100,
	SH_BTLG_POINT_APPSBL_IN_10		= 0x00000200,
	SH_BTLG_POINT_APPSBL_OUT_11		= 0x00000400,
	SH_BTLG_POINT_APPSBL_OUT_12		= 0x00000800,
	SH_BTLG_POINT_APPSBL_OUT_13		= 0x00001000,
	SH_BTLG_POINT_APPSBL_OUT_14		= 0x00002000,
	SH_BTLG_POINT_APPSBL_OUT_15		= 0x00004000,
	SH_BTLG_POINT_APPSBL_OUT_16		= 0x00008000,

/* LINUX */
	SH_BTLG_POINT_RESERVE_17		= 0x00010000,
	SH_BTLG_POINT_RESERVE_18		= 0x00020000,
	SH_BTLG_POINT_RESERVE_19		= 0x00040000,
	SH_BTLG_POINT_RESERVE_20		= 0x00080000,
	SH_BTLG_POINT_RESERVE_21		= 0x00100000,
	SH_BTLG_POINT_RESERVE_22		= 0x00200000,
	SH_BTLG_POINT_RESERVE_23		= 0x00400000,
	SH_BTLG_POINT_RESERVE_24		= 0x00800000,
	SH_BTLG_POINT_RESERVE_25		= 0x01000000,
	SH_BTLG_POINT_RESERVE_26		= 0x02000000,
	SH_BTLG_POINT_RESERVE_27		= 0x04000000,
	SH_BTLG_POINT_RESERVE_28		= 0x08000000,
	SH_BTLG_POINT_RESERVE_29		= 0x10000000,
	SH_BTLG_POINT_RESERVE_30		= 0x20000000,
	SH_BTLG_POINT_RESERVE_31		= 0x40000000,

/* Unavailable */
//	SH_BTLG_POINT_RESERVE_32		= 0x80000000,
//	SH_BTLG_POINT_MAX				= 0xFFFFFFFF,
}SH_BTLG_PASS_POINT;

/*===========================================================================
DEFINE
===========================================================================*/
#define USE_SH_BOOT_LOG_IMEM_DEBUG	/* use imem debug */

#ifdef USE_SH_BOOT_LOG_IMEM_DEBUG
	#define SH_SET_PASS_POINT(point) __raw_writel((__raw_readl(MSM_IMEM_BASE+0x00000028))|(point),MSM_IMEM_BASE+0x00000028);
#else /* USE_SH_BOOT_LOG_IMEM_DEBUG */
	#define SH_SET_PASS_POINT(point)
#endif /* USE_SH_BOOT_LOG_IMEM_DEBUG */


/*===========================================================================
FUNCTION
===========================================================================*/

#endif /* __SH_BOOTLOG_H__ */

