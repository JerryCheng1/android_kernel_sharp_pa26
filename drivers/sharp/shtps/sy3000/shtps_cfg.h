/* drivers/sharp/shtps/sy3000/shtps_cfg.h
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
#ifndef __SHTPS_CFG_H__
#define __SHTPS_CFG_H__
/* --------------------------------------------------------------------------- */

#if 0	/** For build test */
	#undef CONFIG_SHTPS_SY3000_TM3058_001
	#undef CONFIG_SHTPS_SY3000_TM3059_001

	#define CONFIG_SHTPS_SY3000_TM3058_001
#endif

#include <sharp/shtps_dev.h>
/* --------------------------------------------------------------------------- */
#if defined( CONFIG_SHTPS_SY3000_TM3058_001 )
	#include "tm3058-001/shtps_cfg_tm3058-001.h"

#elif defined( CONFIG_SHTPS_SY3000_TM3059_001 )
	#include "tm3059-001/shtps_cfg_tm3059-001.h"

#else
	#include "tm3058-001/shtps_cfg_tm3058-001.h"

#endif

/* --------------------------------------------------------------------------- */
#endif	/* __SHTPS_CFG_H__ */

