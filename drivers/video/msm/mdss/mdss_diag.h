/* drivers/video/msm/mdss/mdss_diag.h  (Display Driver)
 *
 * Copyright (C) 2014 SHARP CORPORATION
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

#ifndef MDSS_DIAG_H
#define MDSS_DIAG_H

#include "mdss_panel.h"
#include <linux/msm_mdp.h>

extern int mdss_diag_mipi_check(struct mdp_mipi_check_param * mipi_check_param,
			struct mdss_panel_data *pdata);
extern int mdss_diag_mipi_check_get_exec_state(void);
extern int mdss_diag_mipi_clkchg(struct mdp_mipi_clkchg_param * mipi_clkchg_param);

#endif /* MDSS_DIAG_H */
