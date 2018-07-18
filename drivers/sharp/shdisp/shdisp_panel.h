/* drivers/sharp/shdisp/shdisp_panel.h  (Display Driver)
 *
 * Copyright (C) 2012 SHARP CORPORATION
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */
#ifndef SHDISP_PANEL_API_H
#define SHDISP_PANEL_API_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <sharp/shdisp_dsi.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_DSI_LOW_POWER_MODE = 0,
    SHDISP_DSI_LOW_POWER_MODE_MS,
    SHDISP_DSI_LOW_POWER_MODE_SL,
    SHDISP_DSI_LOW_POWER_MODE_BOTH,

    SHDISP_DSI_HIGH_SPEED_MODE,
    SHDISP_DSI_HIGH_SPEED_MODE_MS,
    SHDISP_DSI_HIGH_SPEED_MODE_SL,
    SHDISP_DSI_HIGH_SPEED_MODE_BOTH,

    SHDISP_DSI_TRANSFER_MODE_MAX
};

enum {
     SHDISP_PANEL_POWER_FIRST_ON,
     SHDISP_PANEL_POWER_NORMAL_ON,
     SHDISP_PANEL_POWER_RECOVERY_ON
};

struct shdisp_panel_context {
    unsigned char device_code;
    unsigned short vcom;
    unsigned short vcom_low;
    unsigned short vcom_nvram;
    struct shdisp_lcddr_phy_gamma_reg lcddr_phy_gamma;
};

#if defined(CONFIG_SHDISP_PANEL_ANDY)
struct mdp_mipi_clkchg_panel_andy {
    unsigned char rtn;
    unsigned char gip;
};
#endif /* CONFIG_SHDISP_PANEL_ANDY */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
void shdisp_panel_API_create(void);
int shdisp_panel_API_get_recovery_type(int *type);
int shdisp_panel_API_init_io(struct shdisp_panel_context *panel_ctx);
int shdisp_panel_API_exit_io(void);
int shdisp_panel_API_power_on(int mode);
int shdisp_panel_API_power_off(int mode);
int shdisp_panel_API_disp_on(void);
int shdisp_panel_API_disp_off(void);
int shdisp_panel_API_start_display(void);
int shdisp_panel_API_post_video_start(void);
int shdisp_panel_API_disp_update(struct shdisp_main_update *update);
int shdisp_panel_API_disp_clear_screen(struct shdisp_main_clear *clear);
int shdisp_panel_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size);
int shdisp_panel_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size);
#ifndef SHDISP_NOT_SUPPORT_FLICKER
int shdisp_panel_API_diag_set_flicker_param(struct shdisp_diag_flicker_param vcom);
int shdisp_panel_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *vcom);
int shdisp_panel_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *vcom);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
int shdisp_panel_API_check_recovery(void);
int shdisp_panel_API_diag_set_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info);
int shdisp_panel_API_diag_get_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info);
int shdisp_panel_API_diag_set_gamma(struct shdisp_diag_gamma *gamma);
int shdisp_panel_API_shutdown(void);
#if defined(CONFIG_ANDROID_ENGINEERING)
void shdisp_panel_API_dump(int type);
#endif /* CONFIG_ANDROID_ENGINEERING */
int shdisp_panel_API_set_irq(int enable);

int shdisp_panel_API_mipi_dsi_cmds_tx(int commit, struct shdisp_dsi_cmd_desc *cmds, int cnt);
int shdisp_panel_API_mipi_dsi_cmds_rx(unsigned char *rbuf, struct shdisp_dsi_cmd_desc *cmds, unsigned char size);
int shdisp_panel_API_mipi_diag_write_reg(char dtype, unsigned char addr, char *write_data, unsigned char size);
int shdisp_panel_API_mipi_diag_read_reg(char dtype, unsigned char addr, unsigned char *read_data, unsigned char size);

#if defined(CONFIG_SHDISP_PANEL_ANDY)
int shdisp_andy_API_set_freq_param(struct mdp_mipi_clkchg_panel_andy freq);
int shdisp_andy_API_vcom_tracking(int tracking);
int shdisp_andy_API_vcom_is_adjusted(void);
#endif /* CONFIG_SHDISP_PANEL_ANDY */


struct shdisp_panel_operations {
    int (*init_io)(struct shdisp_panel_context *panel_ctx);
    int (*exit_io)(void);
    int (*init_isr)(void);
    int (*power_on)(int mode);
    int (*power_off)(int mode);
    int (*disp_on) (void);
    int (*disp_off)(void);
    int (*start_display)(void);
    int (*post_video_start)(void);
    int (*check_flicker)(unsigned short vcom_in, unsigned short *vcom_out);
    int (*write_reg)(unsigned char addr, unsigned char *write_data, unsigned char size);
    int (*read_reg)(unsigned char addr, unsigned char *read_data, unsigned char size);
    int (*set_flicker)(struct shdisp_diag_flicker_param vcom);
    int (*get_flicker)(struct shdisp_diag_flicker_param *vcom);
    int (*get_flicker_low)(struct shdisp_diag_flicker_param *vcom);
    int (*check_recovery)(void);
    int (*set_gammatable_and_voltage)(struct shdisp_diag_gamma_info *gamma_info);
    int (*get_gammatable_and_voltage)(struct shdisp_diag_gamma_info *gamma_info);
    int (*set_gamma)(struct shdisp_diag_gamma *gamma);
    int (*shutdown)(void);
    void (*dump)(int param);
    int (*set_irq)(int enable);
};
#endif /* SHDISP_PANEL_API_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
