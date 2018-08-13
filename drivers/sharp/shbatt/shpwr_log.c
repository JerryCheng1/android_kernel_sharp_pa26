/* drivers/sharp/shbatt/shbif.c
 *
 * Copyright (C) 2014 Sharp Corporation
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

/*+-----------------------------------------------------------------------------+*/
/*| @ DEFINE COMPILE SWITCH :													|*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/file.h>
#include "sharp/shpwr_log.h"

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL MACRO DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */


/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ STATIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

static int						shpwr_batt_level = SHPWR_LOG_LEVEL_ERR;
module_param( shpwr_batt_level, int, 0664 );

#ifdef CONFIG_PM_APQ_THERM
static int						shpwr_tmp_level = SHPWR_LOG_LEVEL_ERR;
module_param( shpwr_tmp_level, int, 0664 );
#endif /* CONFIG_PM_APQ_THERM */

#if defined(CONFIG_BIF) && defined(CONFIG_PM_SUPPORT_BIF)
static int						shpwr_bif_level = SHPWR_LOG_LEVEL_ERR;
module_param( shpwr_bif_level, int, 0664 );
#endif /* defined(CONFIG_BIF) && defined(CONFIG_PM_SUPPORT_BIF) */

static int						shpwr_chg_level = SHPWR_LOG_LEVEL_ERR;
module_param( shpwr_chg_level, int, 0664 );

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN FUNCTION PROTO TYPE DECLARE :                                      |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION PROTO TYPE DECLARE :                                       |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION'S CODE AREA :                                              |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION'S CODE AREA :                                             |*/
/*+-----------------------------------------------------------------------------+*/

shpwr_log_level shpwr_log_current_level(
	shpwr_log_type				type
) {
	shpwr_log_level				ret;
	
	switch( type )
	{
	case SHPWR_LOG_TYPE_BATT:
		ret = shpwr_batt_level;
		break;
#ifdef CONFIG_PM_APQ_THERM
	case SHPWR_LOG_TYPE_TMP:
		ret = shpwr_tmp_level;
		break;
#endif /* CONFIG_PM_APQ_THERM */
#if defined(CONFIG_BIF) && defined(CONFIG_PM_SUPPORT_BIF)
	case SHPWR_LOG_TYPE_BIF:
		ret = shpwr_bif_level;
		break;
#endif /* defined(CONFIG_BIF) && defined(CONFIG_PM_SUPPORT_BIF) */
	case SHPWR_LOG_TYPE_CHG:
		ret = shpwr_chg_level;
		break;
	default:
		ret = SHPWR_LOG_LEVEL_ERR;
		break;
	}
	if( ( ret < SHPWR_LOG_LEVEL_EMERG ) || ( ret > SHPWR_LOG_LEVEL_DEBUG ) )
	{
		ret = SHPWR_LOG_LEVEL_ERR;
	}
	return ret;
}

