/* drivers/sharp/shdisp/shdisp_aria.c  (Display Driver)
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <sharp/shdisp_kerl.h>
#include "shdisp_panel.h"
#include "shdisp_aria.h"
#include "shdisp_system.h"
#include "shdisp_bdic.h"
#include "shdisp_type.h"
#include "shdisp_pm.h"
#include "shdisp_dbg.h"
#include "shdisp_kerl_priv.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_ARIA_VCOM_REG_NUM                (3)

#define SHDISP_ARIA_GAMMA_SETTING_SIZE          (60)
#define SHDISP_ARIA_GAMMA_LEVEL_MIN             (1)
#define SHDISP_ARIA_GAMMA_LEVEL_MAX             (30)
#define SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET       (30)
#define SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL    (2)
#define SHDISP_ARIA_GAMMA_GROUP_BELONG_ADDR     (4)

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_init_flicker_param(unsigned short vcom, unsigned short vcom_low);
static int shdisp_aria_API_init_io(struct shdisp_panel_context *panel_ctx);
static int shdisp_aria_API_power_on(int mode);
static int shdisp_aria_API_power_off(int mode);
static int shdisp_aria_API_disp_on(void);
static int shdisp_aria_API_disp_off(void);
static int shdisp_aria_API_start_display(void);
static int shdisp_aria_API_post_video_start(void);
static int shdisp_aria_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_aria_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size);
static int shdisp_aria_API_diag_set_flicker_param(struct shdisp_diag_flicker_param vcom);
static int shdisp_aria_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *vcom);
static int shdisp_aria_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *vcom);
static int shdisp_aria_API_check_recovery(void);
static int shdisp_aria_API_diag_set_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info);
static int shdisp_aria_API_diag_get_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info);
static int shdisp_aria_API_diag_set_gamma(struct shdisp_diag_gamma *gamma);
static int shdisp_aria_API_shutdown(void);
static void shdisp_aria_API_dump(int type);
static void shdisp_aria_hw_reset(bool);
static int shdisp_aria_mipi_cmd_lcd_on(void);
static int shdisp_aria_mipi_cmd_lcd_off(void);
static int shdisp_aria_mipi_cmd_display_on(void);
static int shdisp_aria_set_switchcommand(char val);

#ifndef SHDISP_NOT_SUPPORT_FLICKER
static int shdisp_aria_diag_set_flicker_param_internal(struct shdisp_diag_flicker_param flicker_param);
static int shdisp_aria_diag_set_flicker_param_ctx(struct shdisp_diag_flicker_param flicker_param);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

static int shdisp_aria_init_phy_gamma(struct shdisp_lcddr_phy_gamma_reg *phy_gamma);
static int shdisp_aria_sleepout_wait_proc(void);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
#ifndef SHDISP_NOT_SUPPORT_FLICKER
static unsigned char aria_wdata[8];
static unsigned char aria_rdata[8];
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

static struct shdisp_panel_context shdisp_panel_ctx;
static struct shdisp_diag_gamma_info diag_tmp_gamma_info;
static int diag_tmp_gamma_info_set = 0;

/* ------------------------------------------------------------------------- */
/*      packet header                                                        */
/* ------------------------------------------------------------------------- */
/*      LCD ON                                                              */
/*      Initial Setting                                                     */

#include "./data/shdisp_aria_data_pa26.h"

static struct shdisp_panel_operations shdisp_aria_fops = {
    shdisp_aria_API_init_io,
    NULL,
    NULL,
    shdisp_aria_API_power_on,
    shdisp_aria_API_power_off,
    shdisp_aria_API_disp_on,
    shdisp_aria_API_disp_off,
    shdisp_aria_API_start_display,
    shdisp_aria_API_post_video_start,
    NULL,
    shdisp_aria_API_diag_write_reg,
    shdisp_aria_API_diag_read_reg,
    shdisp_aria_API_diag_set_flicker_param,
    shdisp_aria_API_diag_get_flicker_param,
    shdisp_aria_API_diag_get_flicker_low_param,
    shdisp_aria_API_check_recovery,
    shdisp_aria_API_diag_set_gammatable_and_voltage,
    shdisp_aria_API_diag_get_gammatable_and_voltage,
    shdisp_aria_API_diag_set_gamma,
    shdisp_aria_API_shutdown,
    shdisp_aria_API_dump,
    NULL,
};

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define MIPI_DSI_COMMAND_TX(x)              (shdisp_panel_API_mipi_dsi_cmds_tx(0, x, ARRAY_SIZE(x)))
#define MIPI_DSI_COMMAND_TX_COMMIT(x)       (shdisp_panel_API_mipi_dsi_cmds_tx(1, x, ARRAY_SIZE(x)))
#define IS_FLICKER_ADJUSTED(param)          (((param & 0xF000) == 0x9000) ? 1 : 0)

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_ANDROID_ENGINEERING)
static int shdisp_aria_dump_reg(void);
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_create                                                    */
/* ------------------------------------------------------------------------- */
struct shdisp_panel_operations *shdisp_aria_API_create(void)
{
    return &shdisp_aria_fops;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_init_flicker_param                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_init_flicker_param(unsigned short vcom, unsigned short vcom_low)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    SHDISP_TRACE("in");
    SHDISP_DEBUG("vcom=0x%04x vcom_low=0x%04x", vcom, vcom_low);

    mipi_sh_aria_cmd_PowerSetting[NO_VCOM1][1] = vcom & 0xFF;
    mipi_sh_aria_cmd_PowerSetting[NO_VCOM1_H][1] =
                        (mipi_sh_aria_cmd_PowerSetting[NO_VCOM1_H][1] & 0xFC) | 0x02;
    if ((vcom >> 8) & 0x01) {
        mipi_sh_aria_cmd_PowerSetting[NO_VCOM1_H][1] |= 0x01;
    }

    SHDISP_DEBUG("VCOM1=0x%02x VCOM1_H=0x%02x",
                        mipi_sh_aria_cmd_PowerSetting[NO_VCOM1][1],
                        mipi_sh_aria_cmd_PowerSetting[NO_VCOM1_H][1]);

    SHDISP_TRACE("out");

#endif
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_init_io                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_init_io(struct shdisp_panel_context *panel_ctx)
{
    int ret = 0;

    SHDISP_TRACE("in");

    memcpy(&(shdisp_panel_ctx), panel_ctx, sizeof(struct shdisp_panel_context));

#ifndef SHDISP_NOT_SUPPORT_FLICKER
    if (shdisp_aria_init_flicker_param(shdisp_panel_ctx.vcom, shdisp_panel_ctx.vcom_low)) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_aria_init_flicker_param.");
    }
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

    ret = shdisp_aria_init_phy_gamma(&shdisp_panel_ctx.lcddr_phy_gamma);
    if (ret) {
        SHDISP_DEBUG("<RESULT_FAILURE> shdisp_aria_init_phy_gamma.");
    }

    if( shdisp_api_get_boot_disp_status() == SHDISP_MAIN_DISP_ON ){
        shdisp_aria_hw_reset(false);
      /* BATTERY_SH */ //shdisp_SYS_API_panel_external_clk_ctl(true);
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_power_on                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_power_on(int mode)
{
    SHDISP_TRACE("in mode=%d", mode);

    shdisp_bdic_API_LCD_power_on();
    if ((mode == SHDISP_PANEL_POWER_FIRST_ON) || (mode == SHDISP_PANEL_POWER_RECOVERY_ON)) {
        shdisp_SYS_API_delay_us(1 * 1000);
    }
    shdisp_bdic_API_LCD_m_power_on();
    if ((mode == SHDISP_PANEL_POWER_FIRST_ON) || (mode == SHDISP_PANEL_POWER_RECOVERY_ON)) {
        shdisp_SYS_API_delay_us(10 * 1000);
    }

    switch (mode) {
    case SHDISP_PANEL_POWER_FIRST_ON:
    case SHDISP_PANEL_POWER_RECOVERY_ON:
        shdisp_aria_hw_reset(false);
        shdisp_SYS_API_delay_us(10);
        shdisp_aria_hw_reset(true);
        shdisp_SYS_API_delay_us(10);
        break;
    case SHDISP_PANEL_POWER_NORMAL_ON:
    default:
        shdisp_aria_hw_reset(true);
        shdisp_SYS_API_delay_us(3 * 1000);
        break;
    }
    shdisp_aria_hw_reset(false);
    shdisp_SYS_API_delay_us(10 * 1000);
   /* BATTERY_SH */ //shdisp_SYS_API_panel_external_clk_ctl(true);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_power_off                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_power_off(int mode)
{
    switch (mode) {
    case SHDISP_PANEL_POWER_RECOVERY_OFF:
        SHDISP_TRACE("in RECOVERY_OFF: mode=%d", mode);
        break;
    case SHDISP_PANEL_POWER_SHUTDOWN_OFF:
        SHDISP_TRACE("in SHUTDOWN_OFF: mode=%d", mode);
        break;
    case SHDISP_PANEL_POWER_NORMAL_OFF:
    default:
        SHDISP_TRACE("in NORMAL_OFF: mode=%d", mode);
        break;
    }

    shdisp_bdic_API_LCD_m_power_off();
    shdisp_bdic_API_LCD_power_off();

    switch (mode) {
    case SHDISP_PANEL_POWER_RECOVERY_OFF:
    case SHDISP_PANEL_POWER_SHUTDOWN_OFF:
        SHDISP_DEBUG("excute aria HW reset");
        shdisp_aria_hw_reset(true);
        shdisp_SYS_API_delay_us(1 * 1000);
        break;
    case SHDISP_PANEL_POWER_NORMAL_OFF:
    default:
        break;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_disp_on                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_disp_on(void)
{
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_aria_mipi_cmd_lcd_on();

    SHDISP_TRACE("out ret=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_disp_off                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_disp_off(void)
{
    SHDISP_TRACE("in");

    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_OFF);
    shdisp_aria_mipi_cmd_lcd_off();

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_start_display                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_start_display(void)
{
    SHDISP_TRACE("in");

    shdisp_aria_mipi_cmd_display_on();

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_post_video_start                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_post_video_start(void)
{
    SHDISP_TRACE("in");
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_start_video                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_start_video(void)
{
    struct {
        unsigned char addr;
        unsigned char data;
        unsigned char size;
        unsigned long wait;
    } reg_data[] = {
        { 0xFF, 0x00, 1, 0 },
        { 0x29, 0x00, 0, 0 },
    };
    int i;
    int ret = 0;

    SHDISP_TRACE("in");

    for (i = 0; i < 2; i++) {
        ret = shdisp_aria_API_diag_write_reg(reg_data[i].addr, &reg_data[i].data, reg_data[i].size);
        if (reg_data[i].wait) {
            shdisp_SYS_API_delay_us(reg_data[i].wait);
        }
    }
    shdisp_bdic_API_IRQ_det_irq_ctrl(true);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_stop_video                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_stop_video(void)
{
    struct {
        unsigned char addr;
        unsigned char data;
        unsigned char size;
        unsigned long wait;
    } reg_data[] = {
        { 0xFF, 0x00, 1, 0 },
        { 0x28, 0x00, 0, WAIT_1FRAME_US * 1 },
    };
    int i;
    int ret = 0;

    SHDISP_TRACE("in");

    shdisp_bdic_API_IRQ_det_irq_ctrl(false);
    for (i = 0; i < 2; i++) {
        ret = shdisp_aria_API_diag_write_reg(reg_data[i].addr, &reg_data[i].data, reg_data[i].size);
        if (reg_data[i].wait) {
            shdisp_SYS_API_delay_us(reg_data[i].wait);
        }
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_diag_write_reg                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    int ret = 0;
    char dtype;

    SHDISP_TRACE("in");

    if (size == 0) {
        dtype = SHDISP_DTYPE_DCS_WRITE;
    } else if (size == 1) {
        dtype = SHDISP_DTYPE_DCS_WRITE1;
    } else {
        dtype = SHDISP_DTYPE_DCS_LWRITE;
    }
    ret = shdisp_panel_API_mipi_diag_write_reg(dtype, addr, write_data, size);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out dokick err ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_diag_read_reg                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ, addr, read_data, size);
    if (ret) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_diag_set_flicker_param                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_diag_set_flicker_param(struct shdisp_diag_flicker_param flicker_param)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_aria_diag_set_flicker_param_internal(flicker_param);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_aria_diag_set_flicker_param_internal.");
        return SHDISP_RESULT_FAILURE;
    } else {
        shdisp_aria_diag_set_flicker_param_ctx(flicker_param);
    }

    SHDISP_TRACE("out");
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}

#ifndef SHDISP_NOT_SUPPORT_FLICKER
/* ------------------------------------------------------------------------- */
/* shdisp_aria_diag_set_flicker_param_internal                               */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_diag_set_flicker_param_internal(struct shdisp_diag_flicker_param flicker_param)
{
    int vcom = flicker_param.master_alpha;
    int vcom_low = flicker_param.master_alpha;
    int i;
    int ret = 0;
    unsigned char aria_rdata_tmp[8];

    SHDISP_TRACE("in");

    if (flicker_param.request & SHDISP_REG_WRITE) {

        for (i = 1; i <= 7; i++) {
            aria_rdata[i] = 0;
            aria_rdata_tmp[i] = 0;
            aria_wdata[i] = 0;
        }

        shdisp_aria_set_switchcommand(0x20);

        aria_rdata_tmp[0] = (unsigned char)(vcom & 0xFF);
        aria_rdata_tmp[2] =  0x02;
        if ((vcom >> 8) & 0x01) {
            aria_rdata_tmp[2] |= 0x01;
        }

        for (i = 0; i < SHDISP_ARIA_VCOM_REG_NUM; i++) {
            if (i != 1) {
                aria_wdata[0] = aria_rdata_tmp[i];
                ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                                  AD_VCOM1 + i, &aria_wdata[0], 1);
                if (ret) {
                    SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg!!" );
                    break;
                }
            }
        }

        SHDISP_DEBUG("VCOM1=0x%02x VCOM1_H=0x%02x",
                              aria_rdata_tmp[0], aria_rdata_tmp[2]);
        SHDISP_DEBUG("vcom=0x%04x", vcom);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("out dokick err ret=%d", ret);
            return ret;
        }
    }

    if (flicker_param.request & (SHDISP_SAVE_VALUE | SHDISP_SAVE_VALUE_LOW)) {
        if (!(flicker_param.request & SHDISP_SAVE_VALUE)) {
            vcom = shdisp_panel_ctx.vcom;
        }
        if (!(flicker_param.request & SHDISP_SAVE_VALUE_LOW)) {
            vcom_low = shdisp_panel_ctx.vcom_low;
        }
        if (shdisp_aria_init_flicker_param(vcom, vcom_low)) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_aria_init_flicker_param.");
        }
    }
    if (flicker_param.request & SHDISP_RESET_VALUE) {
        if (shdisp_aria_init_flicker_param(flicker_param.master_alpha, flicker_param.master_alpha)) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_aria_init_flicker_param.");
        }
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_diag_set_flicker_param_ctx                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_diag_set_flicker_param_ctx(struct shdisp_diag_flicker_param flicker_param)
{
    SHDISP_TRACE("in");

    if (flicker_param.request & SHDISP_SAVE_VALUE) {
        shdisp_panel_ctx.vcom = flicker_param.master_alpha;
        shdisp_panel_ctx.vcom_nvram = 0x9000 | flicker_param.master_alpha;
    }
    if (flicker_param.request & SHDISP_SAVE_VALUE_LOW) {
        shdisp_panel_ctx.vcom_low = flicker_param.master_alpha;
    }
    if (flicker_param.request & SHDISP_RESET_VALUE) {
        shdisp_panel_ctx.vcom = 0;
        shdisp_panel_ctx.vcom_low = 0;
        shdisp_panel_ctx.vcom_nvram = 0;
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_diag_get_flicker_param                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *flicker_param)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int i;
    unsigned char aria_rdata_tmp[8];

    SHDISP_TRACE("in");

    if (flicker_param == NULL) {
        SHDISP_ERR("<NULL_POINTER> flicker_param.");
        return SHDISP_RESULT_FAILURE;
    }

    for (i = 1; i <= 7; i++) {
        aria_rdata[i] = 0;
        aria_rdata_tmp[i] = 0;
    }

    shdisp_aria_set_switchcommand(0x20);

    for (i = 0; i < SHDISP_ARIA_VCOM_REG_NUM; i++) {
        if (i != 1) {
            ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                      AD_VCOM1 + i, aria_rdata, 1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg addr=0x%02x data=0x%02x", AD_VCOM1 + i,
                                                                                                  aria_rdata[0]);
            }
            aria_rdata_tmp[i] = aria_rdata[0];
        }
    }

    flicker_param->master_alpha = ((aria_rdata_tmp[2] & 0x01) << 8) | aria_rdata_tmp[0];

    SHDISP_TRACE("out master_alpha=0x%04X", flicker_param->master_alpha);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_diag_get_flicker_low_param                                */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *flicker_param)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int i;
    unsigned char aria_rdata_tmp[8];

    SHDISP_TRACE("in");

    if (flicker_param == NULL) {
        SHDISP_ERR("<NULL_POINTER> flicker_param.");
        return SHDISP_RESULT_FAILURE;
    }

    for (i = 1; i <= 7; i++) {
        aria_rdata[i] = 0;
        aria_rdata_tmp[i] = 0;
    }

    SHDISP_TRACE("out master_alpha=0x%04X", flicker_param->master_alpha);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_check_recovery                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_check_recovery(void)
{
    int ret;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif  /* SHDISP_RESET_LOG */


    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_RECOVERY_check_restoration();

#if defined(CONFIG_ANDROID_ENGINEERING)
    if (shdisp_dbg_API_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_DETLOW) {
        shdisp_dbg_API_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        SHDISP_DEBUG("force lcd det low.");
        ret = SHDISP_RESULT_FAILURE;
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_RECOVERY_check_restoration.");
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_DET_LOW;
        shdisp_dbg_API_err_output(&err_code, 0);
        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_DET_LOW);
#endif /* SHDISP_RESET_LOG */
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_diag_set_gammatable_and_voltage                               */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_diag_set_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info,
                                                       int set_applied_voltage)
{
#define DEF_POOL_NUM   (50)
    int i, j = 0;
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char aria_gamma_wdata[377];
    unsigned char aria_gamma_addr[377] = {
        mipi_sh_aria_cmd_SwitchCommand[1][0],
        mipi_sh_aria_cmd_OTP_Reload_Control[0],
        mipi_sh_aria_cmd_GAMMAREDposi[0][0],
        mipi_sh_aria_cmd_GAMMAREDposi[1][0],
        mipi_sh_aria_cmd_GAMMAREDposi[2][0],
        mipi_sh_aria_cmd_GAMMAREDposi[3][0],
        mipi_sh_aria_cmd_GAMMAREDposi[4][0],
        mipi_sh_aria_cmd_GAMMAREDposi[5][0],
        mipi_sh_aria_cmd_GAMMAREDposi[6][0],
        mipi_sh_aria_cmd_GAMMAREDposi[7][0],
        mipi_sh_aria_cmd_GAMMAREDposi[8][0],
        mipi_sh_aria_cmd_GAMMAREDposi[9][0],
        mipi_sh_aria_cmd_GAMMAREDposi[10][0],
        mipi_sh_aria_cmd_GAMMAREDposi[11][0],
        mipi_sh_aria_cmd_GAMMAREDposi[12][0],
        mipi_sh_aria_cmd_GAMMAREDposi[13][0],
        mipi_sh_aria_cmd_GAMMAREDposi[14][0],
        mipi_sh_aria_cmd_GAMMAREDposi[15][0],
        mipi_sh_aria_cmd_GAMMAREDposi[16][0],
        mipi_sh_aria_cmd_GAMMAREDposi[17][0],
        mipi_sh_aria_cmd_GAMMAREDposi[18][0],
        mipi_sh_aria_cmd_GAMMAREDposi[19][0],
        mipi_sh_aria_cmd_GAMMAREDposi[20][0],
        mipi_sh_aria_cmd_GAMMAREDposi[21][0],
        mipi_sh_aria_cmd_GAMMAREDposi[22][0],
        mipi_sh_aria_cmd_GAMMAREDposi[23][0],
        mipi_sh_aria_cmd_GAMMAREDposi[24][0],
        mipi_sh_aria_cmd_GAMMAREDposi[25][0],
        mipi_sh_aria_cmd_GAMMAREDposi[26][0],
        mipi_sh_aria_cmd_GAMMAREDposi[27][0],
        mipi_sh_aria_cmd_GAMMAREDposi[28][0],
        mipi_sh_aria_cmd_GAMMAREDposi[29][0],
        mipi_sh_aria_cmd_GAMMAREDposi[30][0],
        mipi_sh_aria_cmd_GAMMAREDposi[31][0],
        mipi_sh_aria_cmd_GAMMAREDposi[32][0],
        mipi_sh_aria_cmd_GAMMAREDposi[33][0],
        mipi_sh_aria_cmd_GAMMAREDposi[34][0],
        mipi_sh_aria_cmd_GAMMAREDposi[35][0],
        mipi_sh_aria_cmd_GAMMAREDposi[36][0],
        mipi_sh_aria_cmd_GAMMAREDposi[37][0],
        mipi_sh_aria_cmd_GAMMAREDposi[38][0],
        mipi_sh_aria_cmd_GAMMAREDposi[39][0],
        mipi_sh_aria_cmd_GAMMAREDposi[40][0],
        mipi_sh_aria_cmd_GAMMAREDposi[41][0],
        mipi_sh_aria_cmd_GAMMAREDposi[42][0],
        mipi_sh_aria_cmd_GAMMAREDposi[43][0],
        mipi_sh_aria_cmd_GAMMAREDposi[44][0],
        mipi_sh_aria_cmd_GAMMAREDposi[45][0],
        mipi_sh_aria_cmd_GAMMAREDposi[46][0],
        mipi_sh_aria_cmd_GAMMAREDposi[47][0],
        mipi_sh_aria_cmd_GAMMAREDposi[48][0],
        mipi_sh_aria_cmd_GAMMAREDposi[49][0],
        mipi_sh_aria_cmd_GAMMAREDposi[50][0],
        mipi_sh_aria_cmd_GAMMAREDposi[51][0],
        mipi_sh_aria_cmd_GAMMAREDposi[52][0],
        mipi_sh_aria_cmd_GAMMAREDposi[53][0],
        mipi_sh_aria_cmd_GAMMAREDposi[54][0],
        mipi_sh_aria_cmd_GAMMAREDposi[55][0],
        mipi_sh_aria_cmd_GAMMAREDposi[56][0],
        mipi_sh_aria_cmd_GAMMAREDposi[57][0],
        mipi_sh_aria_cmd_GAMMAREDposi[58][0],
        mipi_sh_aria_cmd_GAMMAREDposi[59][0],
        mipi_sh_aria_cmd_GAMMAREDnega[0][0],
        mipi_sh_aria_cmd_GAMMAREDnega[1][0],
        mipi_sh_aria_cmd_GAMMAREDnega[2][0],
        mipi_sh_aria_cmd_GAMMAREDnega[3][0],
        mipi_sh_aria_cmd_GAMMAREDnega[4][0],
        mipi_sh_aria_cmd_GAMMAREDnega[5][0],
        mipi_sh_aria_cmd_GAMMAREDnega[6][0],
        mipi_sh_aria_cmd_GAMMAREDnega[7][0],
        mipi_sh_aria_cmd_GAMMAREDnega[8][0],
        mipi_sh_aria_cmd_GAMMAREDnega[9][0],
        mipi_sh_aria_cmd_GAMMAREDnega[10][0],
        mipi_sh_aria_cmd_GAMMAREDnega[11][0],
        mipi_sh_aria_cmd_GAMMAREDnega[12][0],
        mipi_sh_aria_cmd_GAMMAREDnega[13][0],
        mipi_sh_aria_cmd_GAMMAREDnega[14][0],
        mipi_sh_aria_cmd_GAMMAREDnega[15][0],
        mipi_sh_aria_cmd_GAMMAREDnega[16][0],
        mipi_sh_aria_cmd_GAMMAREDnega[17][0],
        mipi_sh_aria_cmd_GAMMAREDnega[18][0],
        mipi_sh_aria_cmd_GAMMAREDnega[19][0],
        mipi_sh_aria_cmd_GAMMAREDnega[20][0],
        mipi_sh_aria_cmd_GAMMAREDnega[21][0],
        mipi_sh_aria_cmd_GAMMAREDnega[22][0],
        mipi_sh_aria_cmd_GAMMAREDnega[23][0],
        mipi_sh_aria_cmd_GAMMAREDnega[24][0],
        mipi_sh_aria_cmd_GAMMAREDnega[25][0],
        mipi_sh_aria_cmd_GAMMAREDnega[26][0],
        mipi_sh_aria_cmd_GAMMAREDnega[27][0],
        mipi_sh_aria_cmd_GAMMAREDnega[28][0],
        mipi_sh_aria_cmd_GAMMAREDnega[29][0],
        mipi_sh_aria_cmd_GAMMAREDnega[30][0],
        mipi_sh_aria_cmd_GAMMAREDnega[31][0],
        mipi_sh_aria_cmd_GAMMAREDnega[32][0],
        mipi_sh_aria_cmd_GAMMAREDnega[33][0],
        mipi_sh_aria_cmd_GAMMAREDnega[34][0],
        mipi_sh_aria_cmd_GAMMAREDnega[35][0],
        mipi_sh_aria_cmd_GAMMAREDnega[36][0],
        mipi_sh_aria_cmd_GAMMAREDnega[37][0],
        mipi_sh_aria_cmd_GAMMAREDnega[38][0],
        mipi_sh_aria_cmd_GAMMAREDnega[39][0],
        mipi_sh_aria_cmd_GAMMAREDnega[40][0],
        mipi_sh_aria_cmd_GAMMAREDnega[41][0],
        mipi_sh_aria_cmd_GAMMAREDnega[42][0],
        mipi_sh_aria_cmd_GAMMAREDnega[43][0],
        mipi_sh_aria_cmd_GAMMAREDnega[44][0],
        mipi_sh_aria_cmd_GAMMAREDnega[45][0],
        mipi_sh_aria_cmd_GAMMAREDnega[46][0],
        mipi_sh_aria_cmd_GAMMAREDnega[47][0],
        mipi_sh_aria_cmd_GAMMAREDnega[48][0],
        mipi_sh_aria_cmd_GAMMAREDnega[49][0],
        mipi_sh_aria_cmd_GAMMAREDnega[50][0],
        mipi_sh_aria_cmd_GAMMAREDnega[51][0],
        mipi_sh_aria_cmd_GAMMAREDnega[52][0],
        mipi_sh_aria_cmd_GAMMAREDnega[53][0],
        mipi_sh_aria_cmd_GAMMAREDnega[54][0],
        mipi_sh_aria_cmd_GAMMAREDnega[55][0],
        mipi_sh_aria_cmd_GAMMAREDnega[56][0],
        mipi_sh_aria_cmd_GAMMAREDnega[57][0],
        mipi_sh_aria_cmd_GAMMAREDnega[58][0],
        mipi_sh_aria_cmd_GAMMAREDnega[59][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[0][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[1][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[2][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[3][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[4][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[5][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[6][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[7][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[8][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[9][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[10][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[11][0],
        mipi_sh_aria_cmd_SwitchCommand[2][0],
        mipi_sh_aria_cmd_OTP_Reload_Control[0],
        mipi_sh_aria_cmd_GAMMAGREENposi[12][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[13][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[14][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[15][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[16][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[17][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[18][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[19][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[20][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[21][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[22][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[23][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[24][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[25][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[26][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[27][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[28][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[29][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[30][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[31][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[32][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[33][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[34][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[35][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[36][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[37][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[38][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[39][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[40][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[41][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[42][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[43][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[44][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[45][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[46][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[47][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[48][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[49][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[50][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[51][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[52][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[53][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[54][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[55][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[56][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[57][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[58][0],
        mipi_sh_aria_cmd_GAMMAGREENposi[59][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[0][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[1][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[2][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[3][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[4][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[5][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[6][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[7][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[8][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[9][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[10][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[11][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[12][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[13][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[14][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[15][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[16][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[17][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[18][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[19][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[20][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[21][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[22][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[23][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[24][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[25][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[26][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[27][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[28][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[29][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[30][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[31][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[32][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[33][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[34][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[35][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[36][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[37][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[38][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[39][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[40][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[41][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[42][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[43][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[44][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[45][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[46][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[47][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[48][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[49][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[50][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[51][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[52][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[53][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[54][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[55][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[56][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[57][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[58][0],
        mipi_sh_aria_cmd_GAMMAGREENnega[59][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[0][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[1][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[2][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[3][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[4][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[5][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[6][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[7][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[8][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[9][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[10][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[11][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[12][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[13][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[14][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[15][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[16][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[17][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[18][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[19][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[20][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[21][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[22][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[23][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[24][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[25][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[26][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[27][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[28][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[29][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[30][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[31][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[32][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[33][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[34][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[35][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[36][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[37][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[38][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[39][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[40][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[41][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[42][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[43][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[44][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[45][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[46][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[47][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[48][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[49][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[50][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[51][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[52][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[53][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[54][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[55][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[56][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[57][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[58][0],
        mipi_sh_aria_cmd_GAMMABLUEposi[59][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[0][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[1][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[2][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[3][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[4][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[5][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[6][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[7][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[8][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[9][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[10][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[11][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[12][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[13][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[14][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[15][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[16][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[17][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[18][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[19][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[20][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[21][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[22][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[23][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[24][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[25][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[26][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[27][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[28][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[29][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[30][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[31][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[32][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[33][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[34][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[35][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[36][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[37][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[38][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[39][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[40][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[41][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[42][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[43][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[44][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[45][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[46][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[47][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[48][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[49][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[50][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[51][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[52][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[53][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[54][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[55][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[56][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[57][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[58][0],
        mipi_sh_aria_cmd_GAMMABLUEnega[59][0],
        mipi_sh_aria_cmd_SwitchCommand[1][0],
        mipi_sh_aria_cmd_OTP_Reload_Control[0],
        mipi_sh_aria_cmd_PowerSetting[NO_DCA][0],
        mipi_sh_aria_cmd_PowerSetting[NO_DCAB][0],
        mipi_sh_aria_cmd_PowerSetting[NO_DCB][0],
        mipi_sh_aria_cmd_PowerSetting[NO_BTA][0],
        mipi_sh_aria_cmd_PowerSetting[NO_VGH][0],
        mipi_sh_aria_cmd_PowerSetting[NO_VGL][0],
        mipi_sh_aria_cmd_PowerSetting[NO_VCL][0],
        mipi_sh_aria_cmd_PowerSetting[NO_GVDDP][0],
        mipi_sh_aria_cmd_PowerSetting[NO_GVDDN][0],
        mipi_sh_aria_cmd_PowerSetting[NO_VGHO][0],
        mipi_sh_aria_cmd_PowerSetting[NO_VGLO][0],
    };
    static char mipi_sh_aria_cmd_work[1][2] = {
        {0x00, 0x00},
    };
    static struct shdisp_dsi_cmd_desc mipi_sh_aria_cmds_work[] = {
        {SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_aria_cmd_work[0]}
    };

    SHDISP_TRACE("in");

    for (i = 0; i < SHDISP_ARIA_GAMMA_SETTING_SIZE; i++) {
        if (i == 0) {
            aria_gamma_wdata[j++] = mipi_sh_aria_cmd_SwitchCommand[1][1];
            aria_gamma_wdata[j++] = mipi_sh_aria_cmd_OTP_Reload_Control[1];
        }
        aria_gamma_wdata[j++] = ((gamma_info->gammaR[i] >> 8) & 0x0003);
        aria_gamma_wdata[j++] = (gamma_info->gammaR[i] & 0x00FF);
    }

    for (i = 0; i < SHDISP_ARIA_GAMMA_SETTING_SIZE; i++) {
        if (i == 6) {
            aria_gamma_wdata[j++] = mipi_sh_aria_cmd_SwitchCommand[2][1];
            aria_gamma_wdata[j++] = mipi_sh_aria_cmd_OTP_Reload_Control[1];
        }
        aria_gamma_wdata[j++] = ((gamma_info->gammaG[i] >> 8) & 0x0003);
        aria_gamma_wdata[j++] = (gamma_info->gammaG[i] & 0x00FF);
    }

    for (i = 0; i < SHDISP_ARIA_GAMMA_SETTING_SIZE; i++) {
        aria_gamma_wdata[j++] = ((gamma_info->gammaB[i] >> 8) & 0x0003);
        aria_gamma_wdata[j++] = (gamma_info->gammaB[i] & 0x00FF);
    }

    if (!set_applied_voltage) {
        for (i = 0; i < j; i++) {
            mipi_sh_aria_cmd_work[0][0] = aria_gamma_addr[i];
            mipi_sh_aria_cmd_work[0][1] = aria_gamma_wdata[i];
            if ((i % DEF_POOL_NUM) == (DEF_POOL_NUM - 1)) {
                ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_aria_cmds_work);
            } else {
                ret = MIPI_DSI_COMMAND_TX(mipi_sh_aria_cmds_work);
            }
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_write_reg!!" );
                goto shdisp_end;
            }
        }
        goto shdisp_end;
    }

    aria_gamma_wdata[j++] = mipi_sh_aria_cmd_SwitchCommand[1][1];
    aria_gamma_wdata[j++] = mipi_sh_aria_cmd_OTP_Reload_Control[1];
    aria_gamma_wdata[j++] = gamma_info->dca;
    aria_gamma_wdata[j++] = gamma_info->dcab;
    aria_gamma_wdata[j++] = gamma_info->dcb;
    aria_gamma_wdata[j++] = gamma_info->bta;
    aria_gamma_wdata[j++] = gamma_info->vgh;
    aria_gamma_wdata[j++] = gamma_info->vgl;
    aria_gamma_wdata[j++] = gamma_info->vcl;
    aria_gamma_wdata[j++] = gamma_info->gvddp;
    aria_gamma_wdata[j++] = gamma_info->gvddn;
    aria_gamma_wdata[j++] = gamma_info->vgho;
    aria_gamma_wdata[j++] = gamma_info->vglo;

    for (i = 0; i < j; i++) {
        mipi_sh_aria_cmd_work[0][0] = aria_gamma_addr[i];
        mipi_sh_aria_cmd_work[0][1] = aria_gamma_wdata[i];
        if ((i % DEF_POOL_NUM) == (DEF_POOL_NUM - 1)) {
            ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_aria_cmds_work);
        } else {
            ret = MIPI_DSI_COMMAND_TX(mipi_sh_aria_cmds_work);
        }
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_write_reg!!" );
            goto shdisp_end;
        }
    }
    memcpy(&diag_tmp_gamma_info, gamma_info, sizeof(diag_tmp_gamma_info));
    diag_tmp_gamma_info_set = 1;

shdisp_end:
    mipi_sh_aria_cmd_work[0][0] = mipi_sh_aria_cmd_SwitchCommand[0][0];
    mipi_sh_aria_cmd_work[0][1] = mipi_sh_aria_cmd_SwitchCommand[0][1];
    ret = MIPI_DSI_COMMAND_TX(mipi_sh_aria_cmds_work);
    mipi_sh_aria_cmd_work[0][0] = mipi_sh_aria_cmd_OTP_Reload_Control[0];
    mipi_sh_aria_cmd_work[0][1] = mipi_sh_aria_cmd_OTP_Reload_Control[1];
    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_aria_cmds_work);
    if (ret) {
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_diag_get_gammatable_and_voltage                               */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_diag_get_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info,
                                                       int set_applied_voltage)
{
    int i, j;
    int ret = 0;
    unsigned char aria_rdata[1];
    unsigned short aria_temp_data[SHDISP_ARIA_GAMMA_SETTING_SIZE];

    SHDISP_TRACE("in");

    if (gamma_info == NULL) {
        SHDISP_ERR("<NULL_POINTER> gamma_info.");
        return SHDISP_RESULT_FAILURE;
    }

    memset(aria_temp_data, 0, sizeof(aria_temp_data));
    for (i = 0, j = 0; i < (SHDISP_ARIA_GAMMA_SETTING_SIZE / 2); i++) {
        if (i == 0) {
            ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                                        mipi_sh_aria_cmd_SwitchCommand[1][0],
                                                       &mipi_sh_aria_cmd_SwitchCommand[1][1],
                                                       1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
                goto shdisp_end;
            }
            memset(aria_rdata, 0, sizeof(aria_rdata));
            ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                      mipi_sh_aria_cmd_GAMMAREDposi[0][0],
                                                      aria_rdata,
                                                      1);
        }
        memset(aria_rdata, 0, sizeof(aria_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_aria_cmd_GAMMAREDposi[j++][0],
                                                  aria_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        aria_temp_data[i] = ((aria_rdata[0] << 8) & 0x0300);
        memset(aria_rdata, 0, sizeof(aria_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_aria_cmd_GAMMAREDposi[j++][0],
                                                  aria_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        aria_temp_data[i] |= (aria_rdata[0] & 0x00FF);
    }

    for (i = SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET, j = 0; i < SHDISP_ARIA_GAMMA_SETTING_SIZE; i++) {
        memset(aria_rdata, 0, sizeof(aria_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_aria_cmd_GAMMAREDnega[j++][0],
                                                  aria_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        aria_temp_data[i] = ((aria_rdata[0] << 8) & 0x0300);
        memset(aria_rdata, 0, sizeof(aria_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_aria_cmd_GAMMAREDnega[j++][0],
                                                  aria_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        aria_temp_data[i] |= (aria_rdata[0] & 0x00FF);
    }
    memcpy(gamma_info->gammaR, aria_temp_data, sizeof(aria_temp_data));

    memset(aria_temp_data, 0, sizeof(aria_temp_data));
    for (i = 0, j = 0; i < (SHDISP_ARIA_GAMMA_SETTING_SIZE / 2); i++) {
        if (i == 6) {
            ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                                        mipi_sh_aria_cmd_SwitchCommand[2][0],
                                                       &mipi_sh_aria_cmd_SwitchCommand[2][1],
                                                       1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.");
                goto shdisp_end;
            }
        }
        memset(aria_rdata, 0, sizeof(aria_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_aria_cmd_GAMMAGREENposi[j++][0],
                                                  aria_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        aria_temp_data[i] = ((aria_rdata[0] << 8) & 0x0300);
        memset(aria_rdata, 0, sizeof(aria_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_aria_cmd_GAMMAGREENposi[j++][0],
                                                  aria_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        aria_temp_data[i] |= (aria_rdata[0] & 0x00FF);
    }

    for (i = SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET, j = 0; i < SHDISP_ARIA_GAMMA_SETTING_SIZE; i++) {
        memset(aria_rdata, 0, sizeof(aria_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_aria_cmd_GAMMAGREENnega[j++][0],
                                                  aria_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        aria_temp_data[i] = ((aria_rdata[0] << 8) & 0x0300);
        memset(aria_rdata, 0, sizeof(aria_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_aria_cmd_GAMMAGREENnega[j++][0],
                                                  aria_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        aria_temp_data[i] |= (aria_rdata[0] & 0x00FF);
    }
    memcpy(gamma_info->gammaG, aria_temp_data, sizeof(aria_temp_data));

    memset(aria_temp_data, 0, sizeof(aria_temp_data));
    for (i = 0, j = 0; i < (SHDISP_ARIA_GAMMA_SETTING_SIZE / 2); i++) {
        memset(aria_rdata, 0, sizeof(aria_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_aria_cmd_GAMMABLUEposi[j++][0],
                                                  aria_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        aria_temp_data[i] = ((aria_rdata[0] << 8) & 0x0300);
        memset(aria_rdata, 0, sizeof(aria_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_aria_cmd_GAMMABLUEposi[j++][0],
                                                  aria_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        aria_temp_data[i] |= (aria_rdata[0] & 0x00FF);
    }

    for (i = SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET, j = 0; i < SHDISP_ARIA_GAMMA_SETTING_SIZE; i++) {
        memset(aria_rdata, 0, sizeof(aria_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_aria_cmd_GAMMABLUEnega[j++][0],
                                                  aria_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        aria_temp_data[i] = ((aria_rdata[0] << 8) & 0x0300);
        memset(aria_rdata, 0, sizeof(aria_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_aria_cmd_GAMMABLUEnega[j++][0],
                                                  aria_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        aria_temp_data[i] |= (aria_rdata[0] & 0x00FF);
    }
    memcpy(gamma_info->gammaB, aria_temp_data, sizeof(aria_temp_data));

    if (!set_applied_voltage) {
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                                mipi_sh_aria_cmd_SwitchCommand[1][0],
                                               &mipi_sh_aria_cmd_SwitchCommand[1][1],
                                               1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.");
        goto shdisp_end;
    }

    memset(aria_rdata, 0, sizeof(aria_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_aria_cmd_PowerSetting[NO_DCA][0],
                                              aria_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gamma_info->dca = aria_rdata[0];

    memset(aria_rdata, 0, sizeof(aria_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_aria_cmd_PowerSetting[NO_DCAB][0],
                                              aria_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gamma_info->dcab = aria_rdata[0];

    memset(aria_rdata, 0, sizeof(aria_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_aria_cmd_PowerSetting[NO_DCB][0],
                                              aria_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gamma_info->dcb = aria_rdata[0];

    memset(aria_rdata, 0, sizeof(aria_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_aria_cmd_PowerSetting[NO_BTA][0],
                                              aria_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gamma_info->bta = aria_rdata[0];

    memset(aria_rdata, 0, sizeof(aria_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_aria_cmd_PowerSetting[NO_VGH][0],
                                              aria_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gamma_info->vgh = aria_rdata[0];

    memset(aria_rdata, 0, sizeof(aria_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_aria_cmd_PowerSetting[NO_VGL][0],
                                              aria_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gamma_info->vgl = aria_rdata[0];

    memset(aria_rdata, 0, sizeof(aria_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_aria_cmd_PowerSetting[NO_VCL][0],
                                              aria_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gamma_info->vcl = aria_rdata[0];

    memset(aria_rdata, 0, sizeof(aria_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_aria_cmd_PowerSetting[NO_GVDDP][0],
                                              aria_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gamma_info->gvddp = aria_rdata[0];

    memset(aria_rdata, 0, sizeof(aria_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_aria_cmd_PowerSetting[NO_GVDDN][0],
                                              aria_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gamma_info->gvddn = aria_rdata[0];

    memset(aria_rdata, 0, sizeof(aria_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_aria_cmd_PowerSetting[NO_VGHO][0],
                                              aria_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gamma_info->vgho = aria_rdata[0];

    memset(aria_rdata, 0, sizeof(aria_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_aria_cmd_PowerSetting[NO_VGLO][0],
                                              aria_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gamma_info->vglo = aria_rdata[0];

shdisp_end:
    shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                          mipi_sh_aria_cmd_SwitchCommand[0][0],
                                         &mipi_sh_aria_cmd_SwitchCommand[0][1],
                                         1);

    if (ret) {
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_diag_set_gammatable_and_voltage                           */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_diag_set_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info)
{
    int ret = 0;
    int pcnt, ncnt, i;

    SHDISP_TRACE("in");

    ret = shdisp_aria_diag_set_gammatable_and_voltage(gamma_info, 1);
    if (ret) {
        return ret;
    }

    for (pcnt = 0; pcnt < SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET; pcnt++) {
        ncnt = pcnt + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET;
        i = pcnt * 2;
        mipi_sh_aria_cmd_GAMMAREDposi[i][1]       = ((gamma_info->gammaR[pcnt] >> 8) & 0x0003);
        mipi_sh_aria_cmd_GAMMAREDposi[i + 1][1]   = ( gamma_info->gammaR[pcnt] & 0x00FF);
        mipi_sh_aria_cmd_GAMMAREDnega[i][1]       = ((gamma_info->gammaR[ncnt] >> 8) & 0x0003);
        mipi_sh_aria_cmd_GAMMAREDnega[i + 1][1]   = ( gamma_info->gammaR[ncnt] & 0x00FF);
        mipi_sh_aria_cmd_GAMMAGREENposi[i][1]     = ((gamma_info->gammaG[pcnt] >> 8) & 0x0003);
        mipi_sh_aria_cmd_GAMMAGREENposi[i + 1][1] = ( gamma_info->gammaG[pcnt] & 0x00FF);
        mipi_sh_aria_cmd_GAMMAGREENnega[i][1]     = ((gamma_info->gammaG[ncnt] >> 8) & 0x0003);
        mipi_sh_aria_cmd_GAMMAGREENnega[i + 1][1] = ( gamma_info->gammaG[ncnt] & 0x00FF);
        mipi_sh_aria_cmd_GAMMABLUEposi[i][1]      = ((gamma_info->gammaB[pcnt] >> 8) & 0x0003);
        mipi_sh_aria_cmd_GAMMABLUEposi[i + 1][1]  = ( gamma_info->gammaB[pcnt] & 0x00FF);
        mipi_sh_aria_cmd_GAMMABLUEnega[i][1]      = ((gamma_info->gammaB[ncnt] >> 8) & 0x0003);
        mipi_sh_aria_cmd_GAMMABLUEnega[i + 1][1]  = ( gamma_info->gammaB[ncnt] & 0x00FF);
    }

    mipi_sh_aria_cmd_PowerSetting[NO_DCA][1]    = gamma_info->dca;
    mipi_sh_aria_cmd_PowerSetting[NO_DCAB][1]    = gamma_info->dcab;
    mipi_sh_aria_cmd_PowerSetting[NO_DCB][1]    = gamma_info->dcb;
    mipi_sh_aria_cmd_PowerSetting[NO_BTA][1]    = gamma_info->bta;
    mipi_sh_aria_cmd_PowerSetting[NO_VGH][1]    = gamma_info->vgh;
    mipi_sh_aria_cmd_PowerSetting[NO_VGL][1]    = gamma_info->vgl;
    mipi_sh_aria_cmd_PowerSetting[NO_VCL][1]    = gamma_info->vcl;
    mipi_sh_aria_cmd_PowerSetting[NO_GVDDP][1]  = gamma_info->gvddp;
    mipi_sh_aria_cmd_PowerSetting[NO_GVDDN][1]  = gamma_info->gvddn;
    mipi_sh_aria_cmd_PowerSetting[NO_VGHO][1]   = gamma_info->vgho;
    mipi_sh_aria_cmd_PowerSetting[NO_VGLO][1]   = gamma_info->vglo;

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_diag_get_gammatable_and_voltage                           */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_diag_get_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info)
{
    int ret = 0;

    SHDISP_TRACE("in");

    shdisp_aria_stop_video();
    ret = shdisp_aria_diag_get_gammatable_and_voltage(gamma_info, 1);
    shdisp_aria_start_video();
    if (ret) {
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_diag_set_gamma                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_diag_set_gamma(struct shdisp_diag_gamma *gamma)
{
    int ret = 0;
    int i = 0, j = 0, k = 0;
    int group_idx, level_idx, addr_idx;
    unsigned char aria_gamma_wdata[26];
    unsigned char aria_gamma_addr[26];

    SHDISP_TRACE("in");

    if ((gamma->level < SHDISP_ARIA_GAMMA_LEVEL_MIN) || (gamma->level > SHDISP_ARIA_GAMMA_LEVEL_MAX)) {
        SHDISP_ERR("<INVALID_VALUE> gamma->level(%d).", gamma->level);
        return SHDISP_RESULT_FAILURE;
    }

    if (!diag_tmp_gamma_info_set) {
        shdisp_aria_stop_video();
        ret = shdisp_aria_diag_get_gammatable_and_voltage(&diag_tmp_gamma_info, 0);
        shdisp_aria_start_video();
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_aria_diag_get_gammatable_and_voltage.");
            goto shdisp_end;
        }
        diag_tmp_gamma_info_set = 1;
    }

    diag_tmp_gamma_info.gammaR[gamma->level - 1] = gamma->gammaR_p;
    diag_tmp_gamma_info.gammaR[SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET + (gamma->level - 1)] = gamma->gammaR_n;
    diag_tmp_gamma_info.gammaG[gamma->level - 1] = gamma->gammaG_p;
    diag_tmp_gamma_info.gammaG[SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET + (gamma->level - 1)] = gamma->gammaG_n;
    diag_tmp_gamma_info.gammaB[gamma->level - 1] = gamma->gammaB_p;
    diag_tmp_gamma_info.gammaB[SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET + (gamma->level - 1)] = gamma->gammaB_n;

    group_idx = (gamma->level - 1) / 2;
    level_idx = group_idx * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL;
    addr_idx = group_idx * SHDISP_ARIA_GAMMA_GROUP_BELONG_ADDR;

    aria_gamma_wdata[j++] = mipi_sh_aria_cmd_SwitchCommand[1][1];
    aria_gamma_addr[k++] = mipi_sh_aria_cmd_SwitchCommand[1][0];

    for (i = 0; i < SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL; i++) {
        aria_gamma_wdata[j++] = ((diag_tmp_gamma_info.gammaR[level_idx + i] >> 8) & 0x0003);
        aria_gamma_wdata[j++] = (diag_tmp_gamma_info.gammaR[level_idx + i] & 0x00FF);
        aria_gamma_addr[k++] = mipi_sh_aria_cmd_GAMMAREDposi[addr_idx + i * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL][0];
        aria_gamma_addr[k++] =
            mipi_sh_aria_cmd_GAMMAREDposi[addr_idx + i * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL + 1][0];
    }

    for (i = 0; i < SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL; i++) {
        aria_gamma_wdata[j++] =
            ((diag_tmp_gamma_info.gammaR[level_idx + i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET] >> 8) & 0x0003);
        aria_gamma_wdata[j++] =
             (diag_tmp_gamma_info.gammaR[level_idx + i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET] & 0x00FF);
        aria_gamma_addr[k++] = mipi_sh_aria_cmd_GAMMAREDnega[addr_idx + i * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL][0];
        aria_gamma_addr[k++] =
            mipi_sh_aria_cmd_GAMMAREDnega[addr_idx + i * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL + 1][0];
    }

    if (gamma->level > 6) {
        aria_gamma_wdata[j++] = mipi_sh_aria_cmd_SwitchCommand[2][1];
        aria_gamma_addr[k++] = mipi_sh_aria_cmd_SwitchCommand[2][0];
    }
    for (i = 0; i < SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL; i++) {
        aria_gamma_wdata[j++] = ((diag_tmp_gamma_info.gammaG[level_idx + i] >> 8) & 0x0003);
        aria_gamma_wdata[j++] = (diag_tmp_gamma_info.gammaG[level_idx + i] & 0x00FF);
        aria_gamma_addr[k++] =
            mipi_sh_aria_cmd_GAMMAGREENposi[addr_idx + i * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL][0];
        aria_gamma_addr[k++] =
            mipi_sh_aria_cmd_GAMMAGREENposi[addr_idx + i * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL + 1][0];
    }

    if (gamma->level <= 6) {
        aria_gamma_wdata[j++] = mipi_sh_aria_cmd_SwitchCommand[2][1];
        aria_gamma_addr[k++] = mipi_sh_aria_cmd_SwitchCommand[2][0];
    }
    for (i = 0; i < SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL; i++) {
        aria_gamma_wdata[j++] =
            ((diag_tmp_gamma_info.gammaG[level_idx + i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET] >> 8) & 0x0003);
        aria_gamma_wdata[j++] =
             (diag_tmp_gamma_info.gammaG[level_idx + i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET] & 0x00FF);
        aria_gamma_addr[k++]  =
            mipi_sh_aria_cmd_GAMMAGREENnega[addr_idx + i * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL][0];
        aria_gamma_addr[k++]  =
            mipi_sh_aria_cmd_GAMMAGREENnega[addr_idx + i * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL + 1][0];
    }

    for (i = 0; i < SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL; i++) {
        aria_gamma_wdata[j++] = ((diag_tmp_gamma_info.gammaB[level_idx + i] >> 8) & 0x0003);
        aria_gamma_wdata[j++] = (diag_tmp_gamma_info.gammaB[level_idx + i] & 0x00FF);
        aria_gamma_addr[k++] = mipi_sh_aria_cmd_GAMMABLUEposi[addr_idx + i * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL][0];
        aria_gamma_addr[k++] =
            mipi_sh_aria_cmd_GAMMABLUEposi[addr_idx + i * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL + 1][0];
    }

    for (i = 0; i < SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL; i++) {
        aria_gamma_wdata[j++] =
            ((diag_tmp_gamma_info.gammaB[level_idx + i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET] >> 8) & 0x0003);
        aria_gamma_wdata[j++] =
             (diag_tmp_gamma_info.gammaB[level_idx + i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET] & 0x00FF);
        aria_gamma_addr[k++] =
            mipi_sh_aria_cmd_GAMMABLUEnega[addr_idx + i * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL][0];
        aria_gamma_addr[k++] =
            mipi_sh_aria_cmd_GAMMABLUEnega[addr_idx + i * SHDISP_ARIA_GAMMA_GROUP_BELONG_LEVEL + 1][0];
    }

    for (i = 0; i < (sizeof(aria_gamma_addr) / sizeof(*aria_gamma_addr)); i++) {
        ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                                             aria_gamma_addr[i],
                                                            &aria_gamma_wdata[i], 1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_write_reg!!" );
            goto shdisp_end;
        }
    }

shdisp_end:
    if (ret) {
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_shutdown                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_API_shutdown(void)
{
    shdisp_aria_hw_reset(true);
    shdisp_SYS_API_delay_us(5 * 1000);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_API_dump                                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_aria_API_dump(int type)
{
#if defined(CONFIG_ANDROID_ENGINEERING)
    shdisp_aria_dump_reg();
#endif /* CONFIG_ANDROID_ENGINEERING */
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_set_switchcommand                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_set_switchcommand(char page)
{
    struct shdisp_dsi_cmd_desc cmd;
    char payload[2] = {0xff, 00};

    payload[1] = page;
    memset(&cmd, 0, sizeof(cmd));
    cmd.dtype = SHDISP_DTYPE_DCS_WRITE1;
    cmd.dlen = 2;
    cmd.wait = 0;
    cmd.payload = payload;

    shdisp_panel_API_mipi_dsi_cmds_tx(1, &cmd, 1);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_hw_reset                                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_aria_hw_reset(bool reset)
{
    SHDISP_TRACE("call reset=%d", reset);
    if (reset) {
        shdisp_SYS_API_Host_gpio_free(SHDISP_GPIO_NUM_PANEL_RST_N);
    } else {
        shdisp_SYS_API_Host_gpio_request(SHDISP_GPIO_NUM_PANEL_RST_N);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_mipi_cmd_display_on                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_mipi_cmd_display_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    (void)shdisp_aria_sleepout_wait_proc();

    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_aria_cmds_display_on);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2 ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_mipi_cmd_lcd_off                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_mipi_cmd_lcd_off(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_aria_cmds_display_off);
    shdisp_SYS_API_delay_us((WAIT_1FRAME_US * 4));
    /* BATTERY_SH */ //shdisp_SYS_API_panel_external_clk_ctl(false);
    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_aria_cmds_deep_standby);

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_mipi_cmd_lcd_on                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_mipi_cmd_lcd_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_aria_cmds_power);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_aria_cmds_gate_eq);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_aria_cmds_gamma);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_aria_cmds_gip);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out4 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_aria_cmds_panel);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out5 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_aria_cmds_timing);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out6 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_aria_cmds_output_signal);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out7 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_aria_cmds_display_mode);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out8 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_aria_cmds_clock_setting);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out9 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_aria_cmds_exit_sleep);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out10 ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_aria_init_phy_gamma                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_init_phy_gamma(struct shdisp_lcddr_phy_gamma_reg *phy_gamma)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int i;
    unsigned int checksum;

    SHDISP_TRACE("in");

    if (phy_gamma == NULL) {
        SHDISP_ERR("phy_gamma is NULL.");
        return SHDISP_RESULT_FAILURE;
    }

    if (phy_gamma->status != SHDISP_LCDDR_GAMMA_STATUS_OK) {
        SHDISP_DEBUG("gammg status invalid. status=%02x", phy_gamma->status);
        ret = SHDISP_RESULT_FAILURE;
    } else {
        checksum = phy_gamma->status;
        for (i = 0; i < SHDISP_LCDDR_PHY_GAMMA_BUF_MAX; i++) {
            checksum = checksum + phy_gamma->buf[i];
        }
        for (i = 0; i < SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE; i++) {
            checksum = checksum + phy_gamma->applied_voltage[i];
        }
        if ((checksum & 0x00FFFFFF) != phy_gamma->chksum) {
            SHDISP_DEBUG("%s: gammg chksum NG. chksum=%06x calc_chksum=%06x",
                         __func__, phy_gamma->chksum, (checksum & 0x00FFFFFF));
            ret = SHDISP_RESULT_FAILURE;
        }
    }

    if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("phy_gamma error");
        return SHDISP_RESULT_FAILURE;
    }

    for (i = 0; i < SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET; i++) {
        mipi_sh_aria_cmd_GAMMAREDposi[i * 2][1] = ((phy_gamma->buf[i] >> 8) & 0x0003);
        mipi_sh_aria_cmd_GAMMAREDposi[i * 2 + 1][1] = (phy_gamma->buf[i] & 0x00FF);
        mipi_sh_aria_cmd_GAMMAREDnega[i * 2][1] =
            ((phy_gamma->buf[i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET] >> 8) & 0x0003);
        mipi_sh_aria_cmd_GAMMAREDnega[i * 2 + 1][1] =
            (phy_gamma->buf[i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET] & 0x00FF);
        mipi_sh_aria_cmd_GAMMAGREENposi[i * 2][1] =
            ((phy_gamma->buf[i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET * 2] >> 8) & 0x0003);
        mipi_sh_aria_cmd_GAMMAGREENposi[i * 2 + 1][1] =
            (phy_gamma->buf[i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET * 2] & 0x00FF);
        mipi_sh_aria_cmd_GAMMAGREENnega[i * 2][1] =
            ((phy_gamma->buf[i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET * 3] >> 8) & 0x0003);
        mipi_sh_aria_cmd_GAMMAGREENnega[i * 2 + 1][1] =
            (phy_gamma->buf[i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET * 3] & 0x00FF);
        mipi_sh_aria_cmd_GAMMABLUEposi[i * 2][1] =
            ((phy_gamma->buf[i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET * 4] >> 8) & 0x0003);
        mipi_sh_aria_cmd_GAMMABLUEposi[i * 2 + 1][1] =
            (phy_gamma->buf[i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET * 4] & 0x00FF);
        mipi_sh_aria_cmd_GAMMABLUEnega[i * 2][1] =
            ((phy_gamma->buf[i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET * 5] >> 8) & 0x0003);
        mipi_sh_aria_cmd_GAMMABLUEnega[i * 2 + 1][1] =
            (phy_gamma->buf[i + SHDISP_ARIA_GAMMA_NEGATIVE_OFFSET * 5] & 0x00FF);
    }

    mipi_sh_aria_cmd_PowerSetting[NO_DCA][1] = phy_gamma->applied_voltage[0];
    mipi_sh_aria_cmd_PowerSetting[NO_DCAB][1] = phy_gamma->applied_voltage[1];
    mipi_sh_aria_cmd_PowerSetting[NO_DCB][1] = phy_gamma->applied_voltage[2];
    mipi_sh_aria_cmd_PowerSetting[NO_BTA][1] = phy_gamma->applied_voltage[3];
    mipi_sh_aria_cmd_PowerSetting[NO_VGH][1] = phy_gamma->applied_voltage[4];
    mipi_sh_aria_cmd_PowerSetting[NO_VGL][1] = phy_gamma->applied_voltage[5];
    mipi_sh_aria_cmd_PowerSetting[NO_VCL][1] = phy_gamma->applied_voltage[6];
    mipi_sh_aria_cmd_PowerSetting[NO_GVDDP][1] = phy_gamma->applied_voltage[7];
    mipi_sh_aria_cmd_PowerSetting[NO_GVDDN][1] = phy_gamma->applied_voltage[8];
    mipi_sh_aria_cmd_PowerSetting[NO_VGHO][1] = phy_gamma->applied_voltage[9];
    mipi_sh_aria_cmd_PowerSetting[NO_VGLO][1] = phy_gamma->applied_voltage[10];

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_aria_dump_reg                                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_dump_reg(void)
{
    int i, arraysize;
    struct shdisp_dsi_cmd_desc *dumpptr;
    unsigned char addr, page, read_data;

    printk("[SHDISP] PANEL PARAMETER INFO ->>\n");

    printk("[SHDISP] shdisp_panel_ctx.device_code = %d\n", shdisp_panel_ctx.device_code);
    printk("[SHDISP] shdisp_panel_ctx.vcom       = 0x%04X\n", shdisp_panel_ctx.vcom);
    printk("[SHDISP] shdisp_panel_ctx.vcom_low   = 0x%04X\n", shdisp_panel_ctx.vcom_low);
    printk("[SHDISP] shdisp_panel_ctx.vcom_nvram = 0x%04X\n", shdisp_panel_ctx.vcom_nvram);
    printk("[SHDISP] shdisp_panel_ctx.lcddr_phy_gamma.status = %d\n", shdisp_panel_ctx.lcddr_phy_gamma.status);
    printk("[SHDISP] shdisp_panel_ctx.lcddr_phy_gamma.buf = ");
    for (i = 0; i < SHDISP_LCDDR_PHY_GAMMA_BUF_MAX; i++) {
        printk("%02X,", shdisp_panel_ctx.lcddr_phy_gamma.buf[i]);
    }
    printk("\n");
    printk("[SHDISP] shdisp_panel_ctx.lcddr_phy_gamma.applied_voltage = ");
    for (i = 0; i < SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE; i++) {
        printk("%02X,", shdisp_panel_ctx.lcddr_phy_gamma.applied_voltage[i]);
    }
    printk("\n");
    printk("[SHDISP] shdisp_panel_ctx.lcddr_phy_gamma.chksum = %d\n", shdisp_panel_ctx.lcddr_phy_gamma.chksum);

    arraysize = ARRAY_SIZE(mipi_sh_aria_cmds_power);
    dumpptr   = mipi_sh_aria_cmds_power;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_aria_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_aria_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_aria_cmds_gate_eq);
    dumpptr   = mipi_sh_aria_cmds_gate_eq;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_aria_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_aria_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_aria_cmds_gamma);
    dumpptr   = mipi_sh_aria_cmds_gamma;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_aria_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_aria_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_aria_cmds_gip);
    dumpptr = mipi_sh_aria_cmds_gip;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_aria_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_aria_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_aria_cmds_panel);
    dumpptr = mipi_sh_aria_cmds_panel;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_aria_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_aria_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_aria_cmds_timing);
    dumpptr = mipi_sh_aria_cmds_timing;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_aria_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_aria_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_aria_cmds_output_signal);
    dumpptr   = mipi_sh_aria_cmds_output_signal;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_aria_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_aria_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_aria_cmds_display_mode);
    dumpptr   = mipi_sh_aria_cmds_display_mode;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_aria_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_aria_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }


    arraysize = ARRAY_SIZE(mipi_sh_aria_cmds_clock_setting);
    dumpptr   = mipi_sh_aria_cmds_clock_setting;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_aria_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_aria_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    printk("[SHDISP] PANEL PARAMETER INFO <<-\n");
    return SHDISP_RESULT_SUCCESS;
}
#endif /* CONFIG_ANDROID_ENGINEERING */
/* ------------------------------------------------------------------------- */
/* shdisp_aria_sleepout_wait_proc                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_aria_sleepout_wait_proc(void)
{
    struct timespec ts_start, ts_end;
    unsigned long long wtime = 0;
    unsigned long long sleepout_time = 100*1000;

    SHDISP_TRACE("in");

    getnstimeofday(&ts_start);

    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_INIT);
    shdisp_bdic_API_update_led_value();

    shdisp_SYS_API_delay_us(10*1000);

    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_ON);

    getnstimeofday(&ts_end);
    wtime = (ts_end.tv_sec - ts_start.tv_sec) * 1000000;
    wtime += (ts_end.tv_nsec - ts_start.tv_nsec) / 1000;
    SHDISP_PERFORMANCE("rest of als_mode_on wait=%lld, wtime=%llu", (sleepout_time - wtime), wtime);

    if (wtime < sleepout_time) {
        shdisp_SYS_API_delay_us(sleepout_time - wtime);
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
