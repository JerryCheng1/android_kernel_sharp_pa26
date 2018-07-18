/* drivers/sharp/shtps/atmel/shtps_filter.h
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
#ifndef __SHTPS_FILTER_H__
#define __SHTPS_FILTER_H__

/* -------------------------------------------------------------------------- */
struct shtps_mxt;

/* -------------------------------------------------------------------------- */
typedef void (*shtps_filter_check_f)(struct shtps_mxt *);
typedef void (*shtps_filter_init_f)(struct shtps_mxt *);
typedef void (*shtps_filter_deinit_f)(struct shtps_mxt *);

typedef void (*shtps_filter_sleep_f)(struct shtps_mxt *);
typedef void (*shtps_filter_touchup_f)(struct shtps_mxt *);

/* -------------------------------------------------------------------------- */
void shtps_filter_main(struct shtps_mxt *ts);
void shtps_filter_init(struct shtps_mxt *ts);
void shtps_filter_deinit(struct shtps_mxt *ts);
void shtps_filter_sleep_enter(struct shtps_mxt *ts);
void shtps_filter_force_touchup(struct shtps_mxt *ts);

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_POS_SCALING_ENABLE )
	void shtps_filter_pos_scaling(struct shtps_mxt *ts);
#endif /* SHTPS_POS_SCALING_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_POSITION_OFFSET )
	struct shtps_pos_offset_info;

	void shtps_filter_offset_pos(struct shtps_mxt *ts);
	void shtps_filter_set_offset_pos_param(struct shtps_mxt *ts, u8 *data);
	void shtps_filter_offset_pos_init(struct shtps_mxt *ts);
	void shtps_filter_offset_pos_deinit(struct shtps_mxt *ts);
#endif /* SHTPS_POSITION_OFFSET */

/* -------------------------------------------------------------------------- */
#endif	/* __SHTPS_FILTER_H__ */

