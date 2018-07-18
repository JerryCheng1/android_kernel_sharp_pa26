/* drivers/sharp/shtps/atmel/shtps_cfg.h
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
	#undef CONFIG_SHTPS_ATMEL_MXT336T
#endif

#include <sharp/shtps_dev.h>
/* --------------------------------------------------------------------------- */
#if defined( CONFIG_SHTPS_ATMEL_MXT336T )
	#include "mXT336T/shtps_cfg_mXT336T.h"
#else
	#include "mXT336T/shtps_cfg_mXT336T.h"
#endif

/* --------------------------------------------------------------------------- */
#endif	/* __SHTPS_CFG_H__ */

