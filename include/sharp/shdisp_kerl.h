/* include/sharp/shdisp_kerl.h  (Display Driver)
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

#ifndef SHDISP_KERN_H
#define SHDISP_KERN_H


/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include "shdisp_context_def.h"


/* ------------------------------------------------------------------------- */
/* NOT SUPPORT FUNCTION                                                      */
/* ------------------------------------------------------------------------- */

#if defined(SHDISP_FACTORY_MODE_ENABLE)
#define SHDISP_NOT_SUPPORT_DET
#if defined(CONFIG_MACH_REN)
#define SHDISP_DET_DSI_MIPI_ERROR
#endif    /* defined(CONFIG_MACH_REN) */
#else   /* defined(SHDISP_FACTORY_MODE_ENABLE) */

#if defined(CONFIG_MACH_LYNX_DL60)
#elif defined(CONFIG_MACH_REN)
#define SHDISP_DET_DSI_MIPI_ERROR
#elif defined(CONFIG_MACH_DECKARD_AL15)
#else
#warning "unknown config!!!"
#define SHDISP_NOT_SUPPORT_DET
#endif  /* defined(CONFIG_MACH_XXX) */

#endif  /* defined(SHDISP_FACTORY_MODE_ENABLE) */

#if defined(CONFIG_MACH_LYNX_DL60) || defined(CONFIG_MACH_LYNX_DL63)
#define SHDISP_TRI_LED2
#endif  /* defined(CONFIG_MACH_LYNX_DL60) || defined(CONFIG_MACH_LYNX_DL63) */

#if defined(CONFIG_MACH_LYNX_DL63)
#define SHDISP_EXTEND_COLOR_LED
#endif  /* defined(CONFIG_MACH_LYNX_DL63) */

/* ------------------------------------------------------------------------- */
/* IOCTL                                                                     */
/* ------------------------------------------------------------------------- */

#define SHDISP_IOC_MAGIC 's'
#define SHDISP_IOCTL_GET_CONTEXT            _IOR  (SHDISP_IOC_MAGIC,  0, struct shdisp_to_user_context)
#define SHDISP_IOCTL_SET_HOST_GPIO          _IOW  (SHDISP_IOC_MAGIC,  1, struct shdisp_host_gpio)
#define SHDISP_IOCTL_TRI_LED_SET_COLOR      _IOW  (SHDISP_IOC_MAGIC,  2, struct shdisp_tri_led)
#define SHDISP_IOCTL_BDIC_WRITE_REG         _IOW  (SHDISP_IOC_MAGIC,  3, struct shdisp_diag_bdic_reg)
#define SHDISP_IOCTL_BDIC_READ_REG          _IOWR (SHDISP_IOC_MAGIC,  4, struct shdisp_diag_bdic_reg)
#define SHDISP_IOCTL_GET_LUX                _IOWR (SHDISP_IOC_MAGIC,  5, struct shdisp_photo_sensor_val)
#define SHDISP_IOCTL_PWM_CONFIG             _IOW  (SHDISP_IOC_MAGIC,  6, int)
#define SHDISP_IOCTL_PWM_STOP               _IO   (SHDISP_IOC_MAGIC,  7)
#define SHDISP_IOCTL_PHOTO_SENSOR_POW_CTL   _IOW  (SHDISP_IOC_MAGIC,  8, struct shdisp_photo_sensor_power_ctl)
#define SHDISP_IOCTL_LCDDR_WRITE_REG        _IOW  (SHDISP_IOC_MAGIC,  9, struct shdisp_lcddr_reg)
#define SHDISP_IOCTL_LCDDR_READ_REG         _IOWR (SHDISP_IOC_MAGIC, 10, struct shdisp_lcddr_reg)
#define SHDISP_IOCTL_SET_FLICKER_PARAM      _IOW  (SHDISP_IOC_MAGIC, 11, struct shdisp_diag_flicker_param)
#define SHDISP_IOCTL_GET_FLICKER_PARAM      _IOWR (SHDISP_IOC_MAGIC, 12, struct shdisp_diag_flicker_param)
#define SHDISP_IOCTL_BKL_SET_AUTO_MODE      _IOW  (SHDISP_IOC_MAGIC, 13, struct shdisp_main_bkl_auto)
#define SHDISP_IOCTL_BDIC_MULTI_READ_REG    _IOWR (SHDISP_IOC_MAGIC, 14, struct shdisp_diag_bdic_reg_multi)
#define SHDISP_IOCTL_BKL_SET_DTV_MODE       _IOW  (SHDISP_IOC_MAGIC, 15, int)
#define SHDISP_IOCTL_BKL_SET_EMG_MODE       _IOW  (SHDISP_IOC_MAGIC, 16, int)
#define SHDISP_IOCTL_LEDC_POWER_ON          _IO   (SHDISP_IOC_MAGIC, 17)
#define SHDISP_IOCTL_LEDC_POWER_OFF         _IO   (SHDISP_IOC_MAGIC, 18)
#define SHDISP_IOCTL_LEDC_SET_RGB           _IOW  (SHDISP_IOC_MAGIC, 19, struct shdisp_ledc_rgb)
#define SHDISP_IOCTL_LEDC_SET_COLOR         _IOW  (SHDISP_IOC_MAGIC, 20, struct shdisp_ledc_req)
#define SHDISP_IOCTL_LEDC_WRITE_REG         _IOW  (SHDISP_IOC_MAGIC, 21, struct shdisp_diag_ledc_reg)
#define SHDISP_IOCTL_LEDC_READ_REG          _IOWR (SHDISP_IOC_MAGIC, 22, struct shdisp_diag_ledc_reg)
#define SHDISP_IOCTL_LUX_CHANGE_IND         _IOWR (SHDISP_IOC_MAGIC, 23, struct shdisp_photo_sensor_val)
#define SHDISP_IOCTL_BKL_SET_ECO_MODE       _IOW  (SHDISP_IOC_MAGIC, 24, int)
#define SHDISP_IOCTL_SET_CABC               _IOWR (SHDISP_IOC_MAGIC, 25, struct shdisp_main_dbc)
#define SHDISP_IOCTL_BKL_SET_CHG_MODE       _IOW  (SHDISP_IOC_MAGIC, 26, int)
#define SHDISP_IOCTL_GET_FLICKER_LOW_PARAM  _IOWR (SHDISP_IOC_MAGIC, 27, struct shdisp_diag_flicker_param)
#define SHDISP_IOCTL_LCDC_SET_DRIVE_FREQ    _IOW  (SHDISP_IOC_MAGIC, 28, struct shdisp_main_drive_freq)
#define SHDISP_IOCTL_SET_GAMMATABLE_AND_VOLTAGE _IOW  (SHDISP_IOC_MAGIC, 29, struct shdisp_diag_gamma_info)
#define SHDISP_IOCTL_GET_GAMMATABLE_AND_VOLTAGE _IOWR (SHDISP_IOC_MAGIC, 30, struct shdisp_diag_gamma_info)
#define SHDISP_IOCTL_SET_GAMMA              _IOW  (SHDISP_IOC_MAGIC, 31, struct shdisp_diag_gamma)
#define SHDISP_IOCTL_GET_AVE_ADO            _IOWR (SHDISP_IOC_MAGIC, 32, struct shdisp_ave_ado)
#define SHDISP_IOCTL_PSALS_READ_REG         _IOWR (SHDISP_IOC_MAGIC, 33, struct shdisp_diag_psals_reg)
#define SHDISP_IOCTL_PSALS_WRITE_REG        _IOWR (SHDISP_IOC_MAGIC, 34, struct shdisp_diag_psals_reg)
#define SHDISP_IOCTL_GET_ALS                _IOWR (SHDISP_IOC_MAGIC, 35, struct shdisp_photo_sensor_raw_val)
#define SHDISP_IOCTL_TRI_LED_SET_COLOR2     _IOW  (SHDISP_IOC_MAGIC, 36, struct shdisp_tri_led)
#define SHDISP_IOCTL_SET_IRQ_MASK           _IOW  (SHDISP_IOC_MAGIC, 37, int)
#define SHDISP_IOCTL_INSERT_SP_PIERCE       _IOW  (SHDISP_IOC_MAGIC, 38, int)
#define SHDISP_IOCTL_REMOVE_SP_PIERCE       _IOW  (SHDISP_IOC_MAGIC, 39, int)
#define SHDISP_IOCTL_GET_SP_PIERCE_STATE    _IOR  (SHDISP_IOC_MAGIC, 40, int)
#define SHDISP_IOCTL_VCOM_TRACKING          _IOW  (SHDISP_IOC_MAGIC, 41, int)

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define DEV_NAME    ("shdisp")
#define SHDISP_DEV    DEV_NAME

#define SHDISP_NOT_ADJUST_VCOM_VAL      (0xFFFF)

#ifndef TRUE
#define TRUE    (1)
#endif  /* TRUE */

#ifndef FALSE
#define FALSE   (0)
#endif  /* FALSE */



/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

enum {
    SHDISP_BDIC_I2C_M_W,
    SHDISP_BDIC_I2C_M_R,
    SHDISP_BDIC_I2C_M_R_MODE1,
    SHDISP_BDIC_I2C_M_R_MODE2,
    SHDISP_BDIC_I2C_M_R_MODE3,
    NUM_SHDISP_BDIC_I2C_M
};

enum {
    SHDISP_PROX_SENSOR_POWER_OFF,
    SHDISP_PROX_SENSOR_POWER_ON,
    SHDISP_PROX_SENSOR_BGMODE_ON,
    SHDISP_PROX_SENSOR_BGMODE_OFF,
    NUM_SHDISP_PROX_SENSOR_POWER
};

enum {
    SHDISP_IRQ_TYPE_ALS,
    SHDISP_IRQ_TYPE_PS,
    SHDISP_IRQ_TYPE_DET,
    SHDISP_IRQ_TYPE_I2CERR,
    NUM_SHDISP_IRQ_TYPE
};

enum {
    SHDISP_LEDC_PWR_STATUS_OFF,
    SHDISP_LEDC_PWR_STATUS_ON,
    NUM_SHDISP_LEDC_PWR_STATUS
};

enum {
    SHDISP_RESULT_SUCCESS,
    SHDISP_RESULT_FAILURE,
    SHDISP_RESULT_FAILURE_I2C_TMO,
    NUM_SHDISP_RESULT
};

enum {
    SHDISP_MAIN_BKL_CHG_OFF,
    SHDISP_MAIN_BKL_CHG_ON,
    NUM_SHDISP_MAIN_BKL_CHG
};

enum {
    SHDISP_BDIC_DISABLE,
    SHDISP_BDIC_ENABLE,
    NUM_SHDISP_BDIC
};

enum {
    SHDISP_LCDC_DISABLE,
    SHDISP_LCDC_ENABLE,
    NUM_SHDISP_LCDC
};

enum {
    SHDISP_TRI_LED_INTERVAL_NONE,
    SHDISP_TRI_LED_INTERVAL_1S,
    SHDISP_TRI_LED_INTERVAL_2S,
    SHDISP_TRI_LED_INTERVAL_3S,
    SHDISP_TRI_LED_INTERVAL_4S,
    SHDISP_TRI_LED_INTERVAL_5S,
    SHDISP_TRI_LED_INTERVAL_6S,
    SHDISP_TRI_LED_INTERVAL_7S,
    SHDISP_TRI_LED_INTERVAL_8S,
    SHDISP_TRI_LED_INTERVAL_9S,
    SHDISP_TRI_LED_INTERVAL_10S,
    SHDISP_TRI_LED_INTERVAL_11S,
    SHDISP_TRI_LED_INTERVAL_12S,
    SHDISP_TRI_LED_INTERVAL_13S,
    SHDISP_TRI_LED_INTERVAL_14S,
    SHDISP_TRI_LED_INTERVAL_15S,
    NUM_SHDISP_TRI_LED_INTERVAL
};



enum {
    SHDISP_LEDC_RGB_MODE_NORMAL,
    SHDISP_LEDC_RGB_MODE_PATTERN1,
    SHDISP_LEDC_RGB_MODE_PATTERN2,
    SHDISP_LEDC_RGB_MODE_PATTERN3,
    SHDISP_LEDC_RGB_MODE_PATTERN4,
    SHDISP_LEDC_RGB_MODE_PATTERN5,
    SHDISP_LEDC_RGB_MODE_PATTERN6,
    SHDISP_LEDC_RGB_MODE_PATTERN7,
    SHDISP_LEDC_RGB_MODE_PATTERN8,
    SHDISP_LEDC_RGB_MODE_PATTERN9,
    SHDISP_LEDC_RGB_MODE_PATTERN10,
    SHDISP_LEDC_RGB_MODE_PATTERN11,
    SHDISP_LEDC_RGB_MODE_PATTERN12,
    SHDISP_LEDC_RGB_MODE_ANIMATION1,
    SHDISP_LEDC_RGB_MODE_ANIMATION2,
    SHDISP_LEDC_RGB_MODE_ANIMATION3,
    SHDISP_LEDC_RGB_MODE_ANIMATION4,
    SHDISP_LEDC_RGB_MODE_ANIMATION5,
    SHDISP_LEDC_RGB_MODE_ANIMATION6,
    SHDISP_LEDC_RGB_MODE_ANIMATION7,
    SHDISP_LEDC_RGB_MODE_ANIMATION8,
    SHDISP_LEDC_RGB_MODE_ANIMATION9,
    SHDISP_LEDC_RGB_MODE_ANIMATION10,

    NUM_SHDISP_LEDC_RGB_MODE
};

enum {
    SHDISP_MAIN_DISP_CABC_MODE_OFF,
    SHDISP_MAIN_DISP_CABC_MODE_DBC,
    SHDISP_MAIN_DISP_CABC_MODE_ACC,
    SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC,
    NUM_SHDISP_CABC_MODE
};

enum {
    SHDISP_MAIN_DISP_CABC_LUT0,
    SHDISP_MAIN_DISP_CABC_LUT1,
    SHDISP_MAIN_DISP_CABC_LUT2,
    SHDISP_MAIN_DISP_CABC_LUT3,
    SHDISP_MAIN_DISP_CABC_LUT4,
    SHDISP_MAIN_DISP_CABC_LUT5,
    NUM_SHDISP_CABC_LUT
};

enum {
    SHDISP_BLACKSCREEN_TYPE_T10,
    SHDISP_BLACKSCREEN_TYPE_T30,
    NUM_SHDISP_BLACKSCREEN_TYPE
};

enum {
    SHDISP_IRQ_NO_MASK,
    SHDISP_IRQ_MASK,
    NUM_SHDISP_IRQ_SWITCH
};

#define SHDISP_REG_WRITE        (0x01)
#define SHDISP_SAVE_VALUE       (0x02)
#define SHDISP_SAVE_VALUE_LOW   (0x04)
#define SHDISP_RESET_VALUE      (0x08)

enum {
    SHDISP_PIERCE_STATE_REMOVE = 0,
    SHDISP_PIERCE_STATE_INSERT
};

struct shdisp_procfs {
    int id;
    int par[4];
};


struct shdisp_bdic_i2c_msg {
    unsigned short addr;
    unsigned char mode;
    unsigned char wlen;
    unsigned char rlen;
    const unsigned char *wbuf;
    unsigned char *rbuf;
};

struct shdisp_panel_param_str{
    unsigned short vcom_alpha;
    unsigned short vcom_alpha_low;
    unsigned int   shdisp_lcd;
};

struct shdisp_main_update {
    void *buf;
    unsigned short src_width;
    unsigned short src_xps;
    unsigned short src_yps;
    unsigned short upd_xsz;
    unsigned short upd_ysz;
    unsigned short dst_xps;
    unsigned short dst_yps;
};

struct shdisp_main_clear {
    unsigned long color;
    unsigned short start_xps;
    unsigned short start_yps;
    unsigned short end_xps;
    unsigned short end_yps;
};

struct shdisp_subscribe {
    int   irq_type;
    void (*callback)(void);
};

struct shdisp_prox_params {
    unsigned int threshold_low;
    unsigned int threshold_high;
};



struct shdisp_main_dbc {
    int mode;
    int auto_mode;
};

struct shdisp_check_cabc_val {
    int old_mode;
    int old_lut;
    int mode;
    int lut;
    int change;
};


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

int shdisp_api_get_fbcon_status(void);
int shdisp_api_main_lcd_power_on(void);
int shdisp_api_main_lcd_power_off(void);
int shdisp_api_main_disp_init_1st(void);
int shdisp_api_main_disp_init_2nd(void);
int shdisp_api_main_lcd_disp_on(void);
int shdisp_api_main_lcd_disp_off(void);
int shdisp_api_main_lcd_start_display(void);
int shdisp_api_main_lcd_post_video_start(void);
int shdisp_api_main_bkl_on(struct shdisp_main_bkl_ctl *bkl);
int shdisp_api_main_bkl_off(void);
int shdisp_api_main_pll_ctl(int ctl);
int shdisp_api_shutdown(void);
int shdisp_api_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
int shdisp_api_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
int shdisp_api_prox_sensor_pow_ctl(int power_mode, struct shdisp_prox_params *prox_params);
int shdisp_api_get_main_disp_status(void);
void shdisp_api_get_boot_context(void);
int shdisp_api_get_panel_info(void);
int shdisp_api_get_clock_info(void);
int shdisp_api_get_clock_sw(void);
int shdisp_api_get_boot_disp_status(void);
int shdisp_api_main_display_done(void);
struct shdisp_argc_lut *shdisp_api_get_argc_lut(void);
struct shdisp_igc_lut *shdisp_api_get_igc_lut(void);
void shdisp_api_clr_mipi_dsi_det_recovery_flag(void);
int  shdisp_api_get_mipi_dsi_det_recovery_flag(void);

#ifdef SHDISP_DET_DSI_MIPI_ERROR
int shdisp_api_do_mipi_dsi_det_recovery(void);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */

int shdisp_api_get_bdic_reset_port(void);

int shdisp_api_check_cabc(struct shdisp_check_cabc_val *value);
int shdisp_api_pwm_enable(void);
int shdisp_api_pwm_disable(void);

int shdisp_api_get_shutdown_mode(void);

int shdisp_api_insert_sp_pierce(void);
int shdisp_api_remove_sp_pierce(void);

#define shdisp_api_check_blackscreen_timeout(a)



#include "shdisp_boot_context.h"
#include "shdisp_kerl_context.h"
#include "shdisp_to_user_context.h"


#endif /* SHDISP_KERN_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
