/* drivers/sharp/shtps/atmel/shtps_fwctl.h
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
#ifndef __SHTPS_FWCTL_H__
#define __SHTPS_FWCTL_H__
/* --------------------------------------------------------------------------- */
struct shtps_mxt;

/* --------------------------------------------------------------------------- */
struct shtps_fwctl_info{
	u8								dev_state;
};

/* --------------------------------------------------------------------------- */
int shtps_fwctl_init(struct shtps_mxt *);
void shtps_fwctl_deinit(struct shtps_mxt *);

void shtps_fwctl_set_dev_state(struct shtps_mxt *, u8);
u8 shtps_fwctl_get_dev_state(struct shtps_mxt *);
int shtps_fwctl_get_maxXPosition(struct shtps_mxt *);
int shtps_fwctl_get_maxYPosition(struct shtps_mxt *);

/* --------------------------------------------------------------------------- */
#endif	/* __SHTPS_FWCTL_H__ */
