/* include/sharp/shdisp_kerl_context.h  (Display Driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION
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

struct shdisp_kernel_context {
    struct shdisp_boot_context boot_ctx;
    int driver_is_open;
    int driver_open_cnt;
    int driver_is_initialized;
    int shutdown_in_progress;
    int dtv_status;
    int thermal_status;
    int usb_chg_status;
    int main_disp_status;
    struct shdisp_main_bkl_ctl main_bkl;
    struct shdisp_tri_led tri_led;
};

