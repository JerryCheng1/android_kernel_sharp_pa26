/* include/sharp/shdisp_boot_context.h  (Display Driver)
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

struct shdisp_boot_context {
    int driver_is_initialized;
    unsigned short hw_handset;
    unsigned short hw_revision;
    unsigned char device_code;
    int handset_color;
    int upper_unit_is_connected;
    int main_disp_status;
    int is_trickled;
    struct shdisp_main_bkl_ctl main_bkl;
    struct shdisp_tri_led tri_led;
    unsigned short vcom;
    unsigned short vcom_low;
    unsigned short vcom_nvram;
    struct shdisp_photo_sensor_adj photo_sensor_adj;
    struct shdisp_lcddr_phy_gamma_reg lcddr_phy_gamma;
    struct shdisp_ledc_status ledc_status;
    int bdic_is_exist;
    int bdic_chipver;
    struct shdisp_bdic_status bdic_status;
    struct shdisp_psals_status psals_status;
    void *image_addr;
    char err_on[SHDISP_NOOS_RESET_NUM];
    struct shdisp_dbg_error_code err_code[SHDISP_NOOS_RESET_NUM];
    unsigned short lut_status;
    struct shdisp_argc_lut argc_lut;
    struct shdisp_igc_lut igc_lut;
    struct shdisp_boot_context *shdisp_boot_ctx_smem_phy_addr;
};
