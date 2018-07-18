/* include/sharp/shdisp_to_user_context.h  (Display Driver)
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

struct shdisp_to_user_context {
    unsigned short hw_handset;
    unsigned short hw_revision;
    int handset_color;
    int upper_unit_is_connected;
    int bdic_is_exist;
    int dtv_status;
    int main_disp_status;
    struct shdisp_igc_lut igc_lut;
    int is_vcom_tracking;
};
