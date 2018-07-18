/* drivers/sharp/shtps/atmel/filter/shtps_filter.c
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
/* -------------------------------------------------------------------------- */
#include "shtps_mxt.h"
#include "shtps_param_extern.h"
#include "shtps_log.h"
#include "shtps_filter.h"

/* -------------------------------------------------------------------------- */
void shtps_filter_main(struct shtps_mxt *ts)
{
	/* It processes first */
	#if defined(SHTPS_POS_SCALING_ENABLE)
		shtps_filter_pos_scaling(ts);
	#endif /* SHTPS_POS_SCALING_ENABLE */
	/* ---------------------------------------------------------------------- */





	/* ---------------------------------------------------------------------- */
	/* It processes at the end */
	#if defined(SHTPS_POSITION_OFFSET)
		shtps_filter_offset_pos(ts);
	#endif /* SHTPS_POSITION_OFFSET */
}

/* -------------------------------------------------------------------------- */
void shtps_filter_init(struct shtps_mxt *ts)
{
	#if defined(SHTPS_POSITION_OFFSET)
		shtps_filter_offset_pos_init(ts);
	#endif /* SHTPS_POSITION_OFFSET */
}

/* -------------------------------------------------------------------------- */
void shtps_filter_deinit(struct shtps_mxt *ts)
{
	#if defined(SHTPS_POSITION_OFFSET)
		shtps_filter_offset_pos_deinit(ts);
	#endif /* SHTPS_POSITION_OFFSET */
}
/* -------------------------------------------------------------------------- */
void shtps_filter_sleep_enter(struct shtps_mxt *ts)
{
}

/* -------------------------------------------------------------------------- */
void shtps_filter_force_touchup(struct shtps_mxt *ts)
{
}
/* -------------------------------------------------------------------------- */

