/* drivers/sharp/shdisp/shdisp_kerl.c  (Display Driver)
 *
 * Copyright (C) 2013 SHARP CORPORATION
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
#include <linux/proc_fs.h>
#include <linux/fb.h>
#ifdef CONFIG_TOUCHSCREEN_SHTPS
#include <sharp/shtps_dev.h>
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/wakelock.h>

#ifdef CONFIG_SHTERM
#include <sharp/shterm_k.h>
#endif /* CONFIG_SHTERM */
#include <sharp/sh_smem.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/proximity.h>
#include "shdisp_kerl_priv.h"
#include "shdisp_system.h"
#include "shdisp_type.h"
#include "shdisp_dbg.h"
#include "shdisp_panel.h"
#include "shdisp_pm.h"
#include "shdisp_bdic.h"
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/delay.h>
#include <asm/param.h>

#ifndef SHDISP_NOT_SUPPORT_DET
void mdss_shdisp_video_transfer_ctrl(int onoff);
void mdss_shdisp_lock_recovery(void);
void mdss_shdisp_unlock_recovery(void);
#endif /* SHDISP_NOT_SUPPORT_DET */

#if defined(CONFIG_SHDISP_PANEL_ARIA)
#ifdef SHDISP_DET_DSI_MIPI_ERROR
extern void mdss_shdisp_dsi_mipi_err_clear(void);
extern int mdss_shdisp_dsi_mipi_err_ctrl(bool enable);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */
extern void mdss_shdisp_mdp_cmd_kickoff(void);
#endif /* CONFIG_SHDISP_PANEL_ARIA */

extern int mdss_shdisp_mdp_hr_video_suspend(void);
extern int mdss_shdisp_mdp_hr_video_resume(void);
extern int mdss_shdisp_set_lp00_mode(int enable);
extern int mdss_shdisp_fps_led_start(void);
extern void mdss_shdisp_fps_led_stop(void);

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_NAME "shdisp"

#define SHDISP_BOOT_MODE_NORMAL             (0xFFFF)
#define SHDISP_BOOT_MODE_OFF_CHARGE         (0x0020)
#define SHDISP_BOOT_MODE_USB_OFFCHARGE      (0x0021)

#define SHDISP_ALS_IRQ_REQ_BK_CTL           (0x01)
#define SHDISP_ALS_IRQ_REQ_DBC              (0x02)

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static dev_t shdisp_dev;
static dev_t shdisp_major = 0;
static dev_t shdisp_minor = 0;
static struct cdev shdisp_cdev;
static struct class *shdisp_class;
static struct shdisp_kernel_context shdisp_kerl_ctx;
static struct semaphore shdisp_sem;
static struct semaphore shdisp_sem_callback;
static struct semaphore shdisp_sem_irq_fac;
static struct semaphore shdisp_sem_timer;
static struct timer_list shdisp_timer;
static int shdisp_timer_stop = 1;
static struct semaphore shdisp_lux_change_sem;
static int    lux_change_wait_flg = SHDISP_LUX_CHANGE_STATE_INIT;
static struct completion lux_change_notify;
static sharp_smem_common_type *sh_smem_common = NULL;
static int first_lcd_on = false;

static int shdisp_subscribe_type_table[NUM_SHDISP_IRQ_TYPE] = {
    SHDISP_SUBSCRIBE_TYPE_INT,
    SHDISP_SUBSCRIBE_TYPE_INT,
    SHDISP_SUBSCRIBE_TYPE_INT,
    SHDISP_SUBSCRIBE_TYPE_INT
};


static void (*shdisp_callback_table[NUM_SHDISP_IRQ_TYPE])(void) = {
    NULL,
    NULL,
    NULL,
    NULL
};

static spinlock_t                 shdisp_q_lock;
static struct shdisp_queue_data_t shdisp_queue_data;

static struct workqueue_struct    *shdisp_wq_gpio;
static struct work_struct         shdisp_wq_gpio_wk;

static struct workqueue_struct    *shdisp_wq_gpio_task;
static struct work_struct         shdisp_wq_gpio_task_wk;


static struct workqueue_struct    *shdisp_wq_timer_task;
static struct work_struct         shdisp_wq_timer_task_wk;

static int shdisp_smem_read_flag = 0;



static struct       wake_lock shdisp_wake_lock_wq;
static int          shdisp_wake_lock_wq_refcnt;

static spinlock_t   shdisp_wake_spinlock;

static struct workqueue_struct    *shdisp_wq_recovery;
static struct semaphore shdisp_sem_req_recovery_lcd;
static unsigned int shdisp_recovery_lcd_queued_flag;
static struct work_struct         shdisp_wq_recovery_lcd_wk;
static struct semaphore shdisp_sem_req_recovery_psals;
static unsigned int shdisp_recovery_psals_queued_flag;
static struct work_struct         shdisp_wq_recovery_psals_wk;

#ifdef SHDISP_TRI_LED2
static spinlock_t               shdisp_swic_spinlock;
static struct workqueue_struct  *shdisp_wq_pierce;
static struct work_struct       shdisp_wq_pierce_wk;

static struct {
    struct shdisp_tri_led led2;
    int     insert;
    int     onoff;
    int     request;
} shdisp_pierce_status;
#endif  /* SHDISP_TRI_LED2 */
/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static void shdisp_init_context(void);
static void shdisp_get_boot_context(void);
static void shdisp_boot_context_initialize(void);

static int shdisp_check_initialized(void);
static int shdisp_check_upper_unit(void);
static int shdisp_check_bdic_exist(void);
static int shdisp_get_boot_disp_status(void);
static unsigned short shdisp_get_hw_revision(void);
static int shdisp_get_bdic_is_exist(void);
static struct shdisp_argc_lut *shdisp_get_argc_lut(void);
static struct shdisp_igc_lut *shdisp_get_igc_lut(void);
static int shdisp_bdic_subscribe_check(struct shdisp_subscribe *subscribe);
static int shdisp_bdic_unsubscribe_check(int irq_type);

static int shdisp_open(struct inode *inode, struct file *filp);
static long shdisp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int shdisp_release(struct inode *inode, struct file *filp);

static int shdisp_ioctl_get_context(void __user *argp);
static int shdisp_ioctl_set_host_gpio(void __user *argp);
static int shdisp_ioctl_tri_led_set_color(void __user *argp);
static int shdisp_ioctl_bdic_write_reg(void __user *argp);
static int shdisp_ioctl_bdic_read_reg(void __user *argp);
static int shdisp_ioctl_bdic_multi_read_reg(void __user *argp);
static int shdisp_ioctl_get_lux(void __user *argp);
static int shdisp_ioctl_photo_sensor_pow_ctl(void __user *argp);
static int shdisp_ioctl_lcddr_write_reg(void __user *argp);
static int shdisp_ioctl_lcddr_read_reg(void __user *argp);
static int shdisp_ioctl_set_flicker_param(void __user *argp);
static int shdisp_ioctl_get_flicker_param(void __user *argp);
static int shdisp_ioctl_get_flicker_low_param(void __user *argp);
static int shdisp_ioctl_bkl_set_auto_mode(void __user *argp);
static int shdisp_ioctl_bkl_set_dtv_mode(void __user *argp);
static int shdisp_ioctl_bkl_set_emg_mode(void __user *argp);
static int shdisp_ioctl_lux_change_ind(void __user *argp);
static int shdisp_ioctl_set_cabc(void __user *argp);
static int shdisp_ioctl_bkl_set_chg_mode(void __user *argp);
static int shdisp_ioctl_panel_set_gammatable_and_voltage(void __user *argp);
static int shdisp_ioctl_panel_get_gammatable_and_voltage(void __user *argp);
static int shdisp_ioctl_panel_set_gamma(void __user *argp);
static int shdisp_ioctl_get_ave_ado(void __user *argp);
static int shdisp_ioctl_psals_read_reg(void __user *argp);
static int shdisp_ioctl_psals_write_reg(void __user *argp);
static int shdisp_ioctl_get_als(void __user *argp);
#ifdef SHDISP_TRI_LED2
static int shdisp_ioctl_tri_led_set_color2(void __user *argp);
static int shdisp_ioctl_insert_sp_pierce(void __user *argp);
static int shdisp_ioctl_remove_sp_pierce(void __user *argp);
static int shdisp_ioctl_get_sp_pierce_state(void __user *argp);
#endif  /* SHDISP_TRI_LED2 */
static int shdisp_ioctl_set_irq_mask(void __user *argp);
static int shdisp_ioctl_vcom_tracking(void __user * argp);

static int shdisp_SQE_main_lcd_power_on(void);
static int shdisp_SQE_main_lcd_power_off(void);
static int shdisp_SQE_main_lcd_disp_on(void);
static int shdisp_SQE_main_lcd_disp_off(void);
static int shdisp_SQE_main_lcd_start_display(void);
static int shdisp_SQE_main_lcd_post_video_start(void);
static int shdisp_SQE_main_lcd_display_done(void);
static int shdisp_SQE_shutdown(void);
static int shdisp_SQE_main_bkl_ctl(int type, struct shdisp_main_bkl_ctl *bkl);
static int shdisp_SQE_main_bkl_set_dtv_mode(int dtv_mode);
static int shdisp_SQE_main_bkl_set_emg_mode(int emg_mode);
static int shdisp_SQE_main_bkl_set_chg_mode(int chg_mode);
static int shdisp_SQE_set_host_gpio(struct shdisp_host_gpio *host_gpio);
static int shdisp_SQE_tri_led_set_color(struct shdisp_tri_led *tri_led);
static int shdisp_SQE_bdic_write_reg(unsigned char reg, unsigned char val);
static int shdisp_SQE_bdic_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_SQE_bdic_multi_read_reg(unsigned char reg, unsigned char *val, int size);
static int shdisp_SQE_get_lux(struct shdisp_photo_sensor_val *value);
static int shdisp_SQE_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
static int shdisp_SQE_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
static int shdisp_SQE_photo_sensor_pow_ctl(struct shdisp_photo_sensor_power_ctl *ctl);
static int shdisp_SQE_panel_write_reg(struct shdisp_lcddr_reg *panel_reg);
static int shdisp_SQE_panel_read_reg(struct shdisp_lcddr_reg *panel_reg);
static int shdisp_SQE_prox_sensor_pow_ctl(int power_mode);
#ifndef SHDISP_NOT_SUPPORT_FLICKER
static int shdisp_SQE_set_flicker_param(struct shdisp_diag_flicker_param vcom);
static int shdisp_SQE_get_flicker_param(struct shdisp_diag_flicker_param *vcom);
static int shdisp_SQE_get_flicker_low_param(struct shdisp_diag_flicker_param *vcom);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
static int shdisp_SQE_check_recovery(void);
#ifndef SHDISP_NOT_SUPPORT_DET
static int shdisp_SQE_do_recovery(void);
#endif /* SHDISP_NOT_SUPPORT_DET */
static int shdisp_SQE_event_subscribe(struct shdisp_subscribe *subscribe);
static int shdisp_SQE_event_unsubscribe(int irq_type);
static int shdisp_SQE_lux_change_ind(struct shdisp_photo_sensor_val *value);
static int shdisp_SQE_set_cabc(struct shdisp_main_dbc *value);
static int shdisp_SQE_set_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info);
static int shdisp_SQE_get_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info);
static int shdisp_SQE_set_gamma(struct shdisp_diag_gamma *gamma);
static int shdisp_SQE_get_ave_ado(struct shdisp_ave_ado *ave_ado);
static int  shdisp_SQE_psals_read_reg(struct shdisp_diag_psals_reg *psals_reg);
static int  shdisp_SQE_psals_write_reg(struct shdisp_diag_psals_reg *psals_reg);
static int shdisp_SQE_get_als(struct shdisp_photo_sensor_raw_val *val);
#ifdef SHDISP_TRI_LED2
static int shdisp_SQE_tri_led_set_color2(struct shdisp_tri_led *tri_led);
static int shdisp_SQE_insert_sp_pierce(void);
static int shdisp_SQE_remove_sp_pierce(void);
#endif  /* SHDISP_TRI_LED2 */
static int shdisp_SQE_set_irq_mask(int irq_msk_ctl);
static int shdisp_SQE_vcom_tracking(int tracking);
static void shdisp_SQE_lcd_det_recovery(void);
static int shdisp_SQE_psals_recovery(void);

static irqreturn_t shdisp_gpio_int_isr( int irq_num, void *data );
static void shdisp_workqueue_handler_gpio(struct work_struct *work);
static void shdisp_workqueue_gpio_task(struct work_struct *work);
static void shdisp_wake_lock_init(void);
static void shdisp_wake_lock(void);
static void shdisp_wake_unlock(void);
static void shdisp_timer_int_isr(unsigned long data);
static void shdisp_timer_int_register(void);
static void shdisp_timer_int_delete(void);
static void shdisp_timer_int_mod(void);
static void shdisp_workqueue_timer_task(struct work_struct *work);
#ifdef SHDISP_TRI_LED2
static void shdisp_workqueue_pierce(struct work_struct *work);
#endif  /* SHDISP_TRI_LED2 */
static int shdisp_do_lcd_det_recovery(void);
static void shdisp_workqueue_handler_recovery_lcd(struct work_struct *work);
static void shdisp_lcd_det_recovery(void);
static int shdisp_lcd_det_recovery_subscribe(void);
static int shdisp_lcd_det_recovery_unsubscribe(void);
static int shdisp_do_psals_recovery(void);
static void shdisp_workqueue_handler_recovery_psals(struct work_struct *work);
static void shdisp_psals_recovery(void);
static int shdisp_event_subscribe(struct shdisp_subscribe *subscribe);
static int shdisp_event_unsubscribe(int irq_type);
static void shdisp_det_mipi_err_ctrl(bool enable);

static void shdisp_semaphore_start(void);
static void shdisp_semaphore_end(const char *func);
#if defined(CONFIG_ANDROID_ENGINEERING)
static int shdisp_proc_write(struct file *file, const char *buffer, unsigned long count, void *data);
static int shdisp_proc_read(char *page, char **start, off_t offset, int count, int *eof, void *data);
static void shdisp_dbg_info_output(int mode);
static void shdisp_dbg_que(int kind);
static void shdisp_debug_subscribe(void);
static void callback_ps(void);
#endif /* CONFIG_ANDROID_ENGINEERING */
static void shdisp_fb_open(void);
static void shdisp_fb_close(void);
static void shdisp_boot_err_output(void);

static struct file_operations shdisp_fops = {
    .owner          = THIS_MODULE,
    .open           = shdisp_open,
    .write          = NULL,
    .read           = NULL,
    .mmap           = NULL,
    .unlocked_ioctl = shdisp_ioctl,
    .release        = shdisp_release,
};

/* ------------------------------------------------------------------------- */
/* KERNEL LOG DEBUG MACROS(module_param)                                     */
/* ------------------------------------------------------------------------- */
static int cabc_mode_set = 0;

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_power_on                                              */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_power_on(void)
{
    int ret;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("RESUME PANEL POWER-ON 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_lcd_power_on();

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_lcd_power_on.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME PANEL POWER-ON 0010 END");
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_power_off                                             */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_power_off(void)
{
    int ret;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("SUSPEND PANEL POWER-OFF 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_lcd_power_off();

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_lcd_power_off.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("SUSPEND PANEL POWER-OFF 0010 END");
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_disp_on                                               */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_disp_on(void)
{
    int ret;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("RESUME PANEL LCD-ON 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();
    ret = shdisp_SQE_main_lcd_disp_on();
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_lcd_disp_on.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME PANEL LCD-ON 0010 END");
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_start_display                                         */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_start_display(void)
{
    int ret = 0;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("RESUME PANEL START-DISP 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_lcd_start_display();

    shdisp_semaphore_end(__func__);

    SHDISP_PERFORMANCE("RESUME PANEL START-DISP 0010 END");
    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_post_video_start                                      */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_post_video_start(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("RESUME PANEL POST-VIDEO-START 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    (void)shdisp_SQE_main_lcd_post_video_start();

    shdisp_semaphore_end(__func__);

    SHDISP_PERFORMANCE("RESUME PANEL POST-VIDEO-START 0010 END");
    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_display_done                                              */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_display_done(void)
{
    int ret;
    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("RESUME PANEL DISPLAY-DONE 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_lcd_display_done();

    shdisp_semaphore_end(__func__);

#ifdef CONFIG_TOUCHSCREEN_SHTPS
    SHDISP_DEBUG("msm_tps_setsleep on");
    msm_tps_setsleep(0);
#endif /* CONFIG_TOUCHSCREEN_SHTPS */

    SHDISP_PERFORMANCE("RESUME PANEL DISPLAY-DONE 0010 END");
    SHDISP_TRACE("out");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_disp_off                                              */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_disp_off(void)
{
    int ret;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("SUSPEND PANEL LCD-OFF 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    SHDISP_DEBUG("");

#ifdef CONFIG_TOUCHSCREEN_SHTPS
    SHDISP_DEBUG("msm_tps_setsleep off");
    msm_tps_setsleep(1);
#endif /* CONFIG_TOUCHSCREEN_SHTPS */

    shdisp_semaphore_start();

    shdisp_lcd_det_recovery_unsubscribe();
    ret = shdisp_SQE_main_lcd_disp_off();

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_lcd_disp_off.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("SUSPEND PANEL LCD-OFF 0010 END");
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_bkl_on                                                    */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_bkl_on(struct shdisp_main_bkl_ctl *bkl)
{
    int ret;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("RESUME PANEL BACKLIGHT-ON 0010 START");

    if (bkl == NULL) {
        SHDISP_ERR("<NULL_POINTER> bkl.");
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    if ((bkl->mode <= SHDISP_MAIN_BKL_MODE_OFF) ||
        (bkl->mode >= SHDISP_MAIN_BKL_MODE_DTV_OFF)) {
        SHDISP_ERR("<INVALID_VALUE> bkl->mode.");
        return SHDISP_RESULT_FAILURE;
    }

    if (bkl->mode != SHDISP_MAIN_BKL_MODE_FIX) {
        SHDISP_DEBUG("out4");
        return SHDISP_RESULT_SUCCESS;
    }

    if (bkl->param <= SHDISP_MAIN_BKL_PARAM_MIN) {
        SHDISP_ERR("<INVALID_VALUE> bkl->param.");
        return SHDISP_RESULT_FAILURE;
    }

    if (bkl->param > SHDISP_MAIN_BKL_PARAM_MAX) {
        bkl->param = SHDISP_MAIN_BKL_PARAM_MAX;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP, bkl);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_ctl.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME PANEL BACKLIGHT-ON 0010 END");
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_bkl_off                                                   */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_bkl_off(void)
{
    int ret;
    struct shdisp_main_bkl_ctl bkl_ctl;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("SUSPEND PANEL BACKLIGHT-OFF 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    bkl_ctl.mode  = SHDISP_MAIN_BKL_MODE_OFF;
    bkl_ctl.param = SHDISP_MAIN_BKL_PARAM_OFF;

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP, &(bkl_ctl));

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_ctl.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("SUSPEND PANEL BACKLIGHT-OFF 0010 END");
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_shutdown                                                       */
/* ------------------------------------------------------------------------- */
int shdisp_api_shutdown(void)
{
    SHDISP_TRACE("in");

    shdisp_semaphore_start();
    shdisp_SQE_shutdown();
    shdisp_semaphore_end(__func__);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_write_bdic_i2c                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_api_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg)
{
    int ret;

    SHDISP_PERFORMANCE("RESUME BDIC WRITE-BDICI2C 0010 START");

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    if (i2c_msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg.");
        return SHDISP_RESULT_FAILURE;
    }

    if (i2c_msg->wbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg->wbuf.");
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_pm_API_is_ps_active() != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("<RESULT_FAILURE> ps is not active.");
        return SHDISP_RESULT_FAILURE;
    }

    if (i2c_msg->mode != SHDISP_BDIC_I2C_M_W) {
        i2c_msg->mode = SHDISP_BDIC_I2C_M_W;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_write_bdic_i2c(i2c_msg);

    shdisp_semaphore_end(__func__);

    if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SQE_write_bdic_i2c.");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    } else if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_write_bdic_i2c.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME BDIC WRITE-BDICI2C 0010 END");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_read_bdic_i2c                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_api_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg)
{
    int ret;

    SHDISP_PERFORMANCE("RESUME BDIC READ-BDICI2C 0010 START");

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    if (i2c_msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg.");
        return SHDISP_RESULT_FAILURE;
    }

    if (i2c_msg->wbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg->wbuf.");
        return SHDISP_RESULT_FAILURE;
    }

    if (i2c_msg->rbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg->rbuf.");
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_pm_API_is_ps_active() != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("<RESULT_FAILURE> ps is not active.");
        return SHDISP_RESULT_FAILURE;
    }

    if (i2c_msg->mode != SHDISP_BDIC_I2C_M_R) {
        i2c_msg->mode = SHDISP_BDIC_I2C_M_R;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_read_bdic_i2c(i2c_msg);

    shdisp_semaphore_end(__func__);

    if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SQE_read_bdic_i2c.");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    } else if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_read_bdic_i2c.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME BDIC READ-BDICI2C 0010 END");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_prox_sensor_pow_ctl                                            */
/* ------------------------------------------------------------------------- */
int shdisp_api_prox_sensor_pow_ctl(int power_mode, struct shdisp_prox_params *prox_params)
{
    int ret;

    SHDISP_PERFORMANCE("RESUME PANEL PROXSENSOR-CTL 0010 START");

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    if (power_mode >= NUM_SHDISP_PROX_SENSOR_POWER) {
        SHDISP_ERR("<INVALID_VALUE> power_mode(%d).", power_mode);
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_DEBUG(":power_mode=%d", power_mode );

    if (power_mode == SHDISP_PROX_SENSOR_POWER_ON) {
        if (prox_params == NULL) {
            SHDISP_ERR("<NULL_POINTER> prox_params.");
            return SHDISP_RESULT_FAILURE;
        }
        shdisp_bdic_API_set_prox_sensor_param(prox_params);
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_prox_sensor_pow_ctl(power_mode);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_prox_sensor_pow_ctl.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME PANEL PROXSENSOR-CTL 0010 START");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_get_boot_context                                               */
/* ------------------------------------------------------------------------- */
void shdisp_api_get_boot_context(void)
{
    shdisp_init_context();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_get_boot_disp_status                                           */
/* ------------------------------------------------------------------------- */
int shdisp_api_get_boot_disp_status(void)
{
    return shdisp_get_boot_disp_status();
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_get_argc_lut                                                   */
/* ------------------------------------------------------------------------- */
struct shdisp_argc_lut *shdisp_api_get_argc_lut(void)
{
    return shdisp_get_argc_lut();
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_get_igc_lut                                                    */
/* ------------------------------------------------------------------------- */
struct shdisp_igc_lut *shdisp_api_get_igc_lut(void)
{
    return shdisp_get_igc_lut();
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_get_main_disp_status                                           */
/* ------------------------------------------------------------------------- */
int shdisp_api_get_main_disp_status(void)
{
    int ret = SHDISP_MAIN_DISP_OFF;

    SHDISP_TRACE("in.");

    ret = shdisp_kerl_ctx.main_disp_status;

    SHDISP_TRACE("out status=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_insert_sp_pierce                                               */
/* ------------------------------------------------------------------------- */
int shdisp_api_insert_sp_pierce(void)
{
#ifdef SHDISP_TRI_LED2
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_SQE_insert_sp_pierce();
    SHDISP_TRACE("out");
    return ret;
#else   /* SHDISP_TRI_LED2 */
    return SHDISP_RESULT_FAILURE;
#endif  /* SHDISP_TRI_LED2 */
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_remove_sp_pierce                                               */
/* ------------------------------------------------------------------------- */
int shdisp_api_remove_sp_pierce(void)
{
#ifdef SHDISP_TRI_LED2
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_SQE_remove_sp_pierce();
    SHDISP_TRACE("out");
    return ret;
#else   /* SHDISP_TRI_LED2 */
    return SHDISP_RESULT_FAILURE;
#endif  /* SHDISP_TRI_LED2 */
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_tri_led_set_color                                              */
/* ------------------------------------------------------------------------- */
int shdisp_api_tri_led_set_color(struct shdisp_tri_led *tri_led)
{
    int ret;

    shdisp_semaphore_start();
    ret = shdisp_SQE_tri_led_set_color(tri_led);
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_tri_led_set_color.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

#ifdef SHDISP_DET_DSI_MIPI_ERROR
/* ------------------------------------------------------------------------- */
/* shdisp_api_do_mipi_dsi_det_recovery                                       */
/* ------------------------------------------------------------------------- */
int shdisp_api_do_mipi_dsi_det_recovery(void)
{
    int ret= SHDISP_RESULT_SUCCESS;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in");
    SHDISP_ERR("MIPI Error");

#ifdef SHDISP_RESET_LOG
    err_code.mode = SHDISP_DBG_MODE_LINUX;
    err_code.type = SHDISP_DBG_TYPE_PANEL;
    err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
    err_code.subcode = SHDISP_DBG_SUBCODE_ESD_MIPI;
    shdisp_dbg_API_err_output(&err_code, 0);
    shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_ESD_MIPI);
#endif /* SHDISP_RESET_LOG */

#ifndef SHDISP_NOT_SUPPORT_DET
    ret = shdisp_do_lcd_det_recovery();
#endif /* SHDISP_NOT_SUPPORT_DET */

    SHDISP_TRACE("out (ret=%d)", ret);
    return ret;
}
#endif /* SHDISP_DET_DSI_MIPI_ERROR */

#if defined(CONFIG_SHDISP_PANEL_ANDY)
/* ------------------------------------------------------------------------- */
/* shdisp_api_set_freq_param                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_api_set_freq_param(struct mdp_mipi_clkchg_panel_andy freq)
{
    int ret= SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();
    ret = shdisp_andy_API_set_freq_param(freq);
    shdisp_semaphore_end(__func__);

    SHDISP_TRACE("out (ret=%d)", ret);
    return ret;
}
#endif /* CONFIG_SHDISP_PANEL_ANDY */

/* ------------------------------------------------------------------------- */
/* shdisp_API_is_lcd_det_recovering                                          */
/* ------------------------------------------------------------------------- */
int shdisp_API_is_lcd_det_recovering(void)
{
    int ret = false;

    SHDISP_TRACE("in.");

    down(&shdisp_sem_req_recovery_lcd);
    if (shdisp_recovery_lcd_queued_flag) {
        ret = true;
    }
    up(&shdisp_sem_req_recovery_lcd);

    SHDISP_TRACE("out ret=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_get_upper_unit                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_API_get_upper_unit(void)
{
    int ret;

    ret = shdisp_check_upper_unit();

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_get_hw_revision                                                */
/* ------------------------------------------------------------------------- */
unsigned short shdisp_API_get_hw_revision(void)
{
    return shdisp_get_hw_revision();
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_get_hw_handset                                                 */
/* ------------------------------------------------------------------------- */
unsigned short shdisp_API_get_hw_handset(void)
{
    return shdisp_kerl_ctx.boot_ctx.hw_handset;
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_check_panel                                                    */
/* ------------------------------------------------------------------------- */
int shdisp_API_check_panel(void)
{
    if (shdisp_API_get_hw_handset()) {
#if defined(CONFIG_MACH_DECKARD_AL15)
        if (shdisp_API_get_hw_revision() >= SHDISP_HW_REV_PP1) {
#else /* defined(CONFIG_MACH_DECKARD_AL15) */
        if (shdisp_API_get_hw_revision() >= SHDISP_HW_REV_PP2) {
#endif /* defined(CONFIG_MACH_DECKARD_AL15) */
            return SHDISP_RESULT_SUCCESS;
        }
    }

    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_get_bdic_is_exist                                              */
/* ------------------------------------------------------------------------- */
int shdisp_API_get_bdic_is_exist(void)
{
    return shdisp_get_bdic_is_exist();
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_is_open                                                        */
/* ------------------------------------------------------------------------- */
int shdisp_API_is_open(void)
{
    return shdisp_kerl_ctx.driver_is_open;
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_do_lcd_det_recovery                                            */
/* ------------------------------------------------------------------------- */
int shdisp_API_do_lcd_det_recovery(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_do_lcd_det_recovery();

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_det_mipi_err_ctrl                                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_det_mipi_err_ctrl(bool enable)
{
    SHDISP_TRACE("in enable=%d", enable);

    if (enable) {
#if defined(CONFIG_SHDISP_PANEL_ANDY)
        (void)shdisp_panel_API_set_irq(SHDISP_IRQ_ENABLE);
#elif defined(CONFIG_SHDISP_PANEL_ARIA)
#ifdef SHDISP_DET_DSI_MIPI_ERROR
        (void)mdss_shdisp_dsi_mipi_err_ctrl(true);
#endif  /* SHDISP_DET_DSI_MIPI_ERROR */
#endif  /* CONFIG_SHDISP_PANEL_ANDY || CONFIG_SHDISP_PANEL_ARIA */
    }else{
#if defined(CONFIG_SHDISP_PANEL_ANDY)
        (void)shdisp_panel_API_set_irq(SHDISP_IRQ_DISABLE);
#elif defined(CONFIG_SHDISP_PANEL_ARIA)
#ifdef SHDISP_DET_DSI_MIPI_ERROR
        (void)mdss_shdisp_dsi_mipi_err_ctrl(false);
#endif  /* SHDISP_DET_DSI_MIPI_ERROR */
#endif  /* CONFIG_SHDISP_PANEL_ANDY || CONFIG_SHDISP_PANEL_ARIA */
    }

    SHDISP_TRACE("out");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_psals_recovery_subscribe                                       */
/* ------------------------------------------------------------------------- */
void shdisp_API_psals_recovery_subscribe(void)
{
    struct shdisp_subscribe subscribe;

    SHDISP_TRACE("in");

    subscribe.irq_type = SHDISP_IRQ_TYPE_I2CERR;
    subscribe.callback = shdisp_psals_recovery;

    shdisp_event_subscribe(&subscribe);

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_psals_recovery_unsubscribe                                     */
/* ------------------------------------------------------------------------- */
void shdisp_API_psals_recovery_unsubscribe(void)
{
    SHDISP_TRACE("in");

    shdisp_event_unsubscribe(SHDISP_IRQ_TYPE_I2CERR);

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_subscribe_check                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_subscribe_check(struct shdisp_subscribe *subscribe)
{


    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    if (subscribe == NULL) {
        SHDISP_ERR("<NULL POINTER> INT_SUBSCRIBE subscribe");
        return SHDISP_RESULT_FAILURE;
    }

    if ((subscribe->irq_type < SHDISP_IRQ_TYPE_ALS) || (subscribe->irq_type >= NUM_SHDISP_IRQ_TYPE)) {
        SHDISP_ERR("<INVALID_VALUE> subscribe->irq_type(%d)", subscribe->irq_type);
        return SHDISP_RESULT_FAILURE;
    }

    if (subscribe->callback == NULL) {
        SHDISP_ERR("<NULL_POINTER> subscribe->callback");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_unsubscribe_check                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_unsubscribe_check(int irq_type)
{
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    if ((irq_type < SHDISP_IRQ_TYPE_ALS) || (irq_type >= NUM_SHDISP_IRQ_TYPE)) {
        SHDISP_ERR("<INVALID_VALUE> irq_type(%d)", irq_type);
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* INITIALIZE                                                                */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_init_context                                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_init_context(void)
{
    if (shdisp_smem_read_flag != 0) {
        return;
    }

    shdisp_get_boot_context();

#ifdef SHDISP_NOT_SUPPORT_NO_OS
    shdisp_kerl_ctx.boot_ctx.bdic_is_exist      = SHDISP_BDIC_IS_EXIST;
#endif /* SHDISP_NOT_SUPPORT_NO_OS */

    shdisp_kerl_ctx.driver_is_open              = false;
    shdisp_kerl_ctx.driver_open_cnt             = 0;
    shdisp_kerl_ctx.driver_is_initialized       = SHDISP_DRIVER_IS_NOT_INITIALIZED;
    shdisp_kerl_ctx.shutdown_in_progress        = false;
    shdisp_kerl_ctx.dtv_status                  = SHDISP_DTV_OFF;
    shdisp_kerl_ctx.thermal_status              = SHDISP_MAIN_BKL_EMG_OFF;
    shdisp_kerl_ctx.usb_chg_status              = SHDISP_MAIN_BKL_CHG_OFF;
    shdisp_kerl_ctx.main_disp_status            = shdisp_kerl_ctx.boot_ctx.main_disp_status;
    shdisp_kerl_ctx.main_bkl.mode               = shdisp_kerl_ctx.boot_ctx.main_bkl.mode;
    shdisp_kerl_ctx.main_bkl.param              = shdisp_kerl_ctx.boot_ctx.main_bkl.param;
    shdisp_kerl_ctx.tri_led.red                 = shdisp_kerl_ctx.boot_ctx.tri_led.red;
    shdisp_kerl_ctx.tri_led.green               = shdisp_kerl_ctx.boot_ctx.tri_led.green;
    shdisp_kerl_ctx.tri_led.blue                = shdisp_kerl_ctx.boot_ctx.tri_led.blue;
    shdisp_kerl_ctx.tri_led.ext_mode            = shdisp_kerl_ctx.boot_ctx.tri_led.ext_mode;
    shdisp_kerl_ctx.tri_led.led_mode            = shdisp_kerl_ctx.boot_ctx.tri_led.led_mode;
    shdisp_kerl_ctx.tri_led.ontime              = shdisp_kerl_ctx.boot_ctx.tri_led.ontime;
    shdisp_kerl_ctx.tri_led.interval            = shdisp_kerl_ctx.boot_ctx.tri_led.interval;
    shdisp_kerl_ctx.tri_led.count               = shdisp_kerl_ctx.boot_ctx.tri_led.count;

    shdisp_smem_read_flag = 1;
#ifdef SHDISP_TRI_LED2
    shdisp_pierce_status.led2.red               = 0;
    shdisp_pierce_status.led2.green             = 0;
    shdisp_pierce_status.led2.blue              = 0;
    shdisp_pierce_status.led2.ext_mode          = 0;
    shdisp_pierce_status.led2.led_mode          = 0;
    shdisp_pierce_status.led2.ontime            = 0;
    shdisp_pierce_status.led2.interval          = 0;
    shdisp_pierce_status.led2.count             = 0;
    shdisp_pierce_status.insert                 = SHDISP_PIERCE_INS_MODE_OFF;
    shdisp_pierce_status.onoff                  = SHDISP_PIERCE_LED_MODE_OFF;
    shdisp_pierce_status.request                = SHDISP_PIERCE_REQ_MODE_OFF;
#endif  /* SHDISP_TRI_LED2 */

#if defined(CONFIG_ANDROID_ENGINEERING)
#ifdef SHDISP_LOG_ENABLE   /* for debug */
    shdisp_dbg_info_output(SHDISP_DEBUG_INFO_TYPE_POWERON);
#endif  /* SHDISP_LOG_ENABLE */
#endif  /* CONFIG_ANDROID_ENGINEERING */
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_common_address                                                 */
/* ------------------------------------------------------------------------- */
static sharp_smem_common_type *shdisp_get_common_address(void)
{
    sharp_smem_common_type *sh_smem_common_adr = NULL;
#ifndef SHDISP_NOT_SUPPORT_NO_OS
    sh_smem_common_adr = (sharp_smem_common_type *)sh_smem_get_common_address();
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */
    return sh_smem_common_adr;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_boot_context                                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_get_boot_context(void)
{
    sh_smem_common = shdisp_get_common_address();
    if (sh_smem_common == NULL) {
        shdisp_boot_context_initialize();

        shdisp_bdic_API_check_sensor_param(&(shdisp_kerl_ctx.boot_ctx.photo_sensor_adj), &(shdisp_kerl_ctx.boot_ctx.photo_sensor_adj));

        /* Check upper unit connect status */
        if (shdisp_SYS_API_check_upper_unit() == SHDISP_RESULT_SUCCESS) {
            shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected = SHDISP_UPPER_UNIT_IS_CONNECTED;
        } else {
            shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected = SHDISP_UPPER_UNIT_IS_NOT_CONNECTED;
        }

    } else {
        memcpy(&(shdisp_kerl_ctx.boot_ctx), &sh_smem_common->shdisp_data_buf, sizeof(struct shdisp_boot_context));
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_context_initialize                                                 */
/* ------------------------------------------------------------------------- */
static void shdisp_boot_context_initialize(void)
{
    int i;

    shdisp_kerl_ctx.boot_ctx.driver_is_initialized       = SHDISP_DRIVER_IS_INITIALIZED;
    shdisp_kerl_ctx.boot_ctx.hw_handset                  = 1;
    shdisp_kerl_ctx.boot_ctx.hw_revision                 = SHDISP_HW_REV_PP2;
    shdisp_kerl_ctx.boot_ctx.device_code                 = 0xff;
    shdisp_kerl_ctx.boot_ctx.handset_color               = 0;
    shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected     = SHDISP_UPPER_UNIT_IS_CONNECTED;
    shdisp_kerl_ctx.boot_ctx.main_disp_status            = SHDISP_MAIN_DISP_OFF;
    shdisp_kerl_ctx.boot_ctx.is_trickled                 = false;
    shdisp_kerl_ctx.boot_ctx.main_bkl.mode               = SHDISP_MAIN_BKL_MODE_OFF;
    shdisp_kerl_ctx.boot_ctx.main_bkl.param              = SHDISP_MAIN_BKL_PARAM_OFF;
    shdisp_kerl_ctx.boot_ctx.tri_led.red                 = 0;
    shdisp_kerl_ctx.boot_ctx.tri_led.green               = 0;
    shdisp_kerl_ctx.boot_ctx.tri_led.blue                = 0;
    shdisp_kerl_ctx.boot_ctx.tri_led.ext_mode            = SHDISP_TRI_LED_EXT_MODE_DISABLE;
    shdisp_kerl_ctx.boot_ctx.tri_led.led_mode            = SHDISP_TRI_LED_MODE_NORMAL;
    shdisp_kerl_ctx.boot_ctx.tri_led.ontime              = 0;
    shdisp_kerl_ctx.boot_ctx.tri_led.interval            = 0;
    shdisp_kerl_ctx.boot_ctx.tri_led.count               = 0;
    shdisp_kerl_ctx.boot_ctx.vcom                        = 0;
    shdisp_kerl_ctx.boot_ctx.vcom_nvram                  = 0;

    memset(&(shdisp_kerl_ctx.boot_ctx.photo_sensor_adj), 0, sizeof(struct shdisp_photo_sensor_adj));
    memset(&(shdisp_kerl_ctx.boot_ctx.lcddr_phy_gamma), 0, sizeof(struct shdisp_lcddr_phy_gamma_reg));

    for (i = 0; i < SHDISP_IGC_LUT_ENTRIES; i++) {
        if (i == 0) {
            shdisp_kerl_ctx.boot_ctx.igc_lut.r_data[i] = 0x0000;
            shdisp_kerl_ctx.boot_ctx.igc_lut.g_data[i] = 0x0000;
            shdisp_kerl_ctx.boot_ctx.igc_lut.b_data[i] = 0x0000;
        } else {
            shdisp_kerl_ctx.boot_ctx.igc_lut.r_data[i] = 0x0010;
            shdisp_kerl_ctx.boot_ctx.igc_lut.g_data[i] = 0x0010;
            shdisp_kerl_ctx.boot_ctx.igc_lut.b_data[i] = 0x0010;
        }
    }

    memset(shdisp_kerl_ctx.boot_ctx.argc_lut.red, 0, sizeof(shdisp_kerl_ctx.boot_ctx.argc_lut.red));
    memset(shdisp_kerl_ctx.boot_ctx.argc_lut.green, 0, sizeof(shdisp_kerl_ctx.boot_ctx.argc_lut.green));
    memset(shdisp_kerl_ctx.boot_ctx.argc_lut.blue, 0, sizeof(shdisp_kerl_ctx.boot_ctx.argc_lut.blue));
    shdisp_kerl_ctx.boot_ctx.argc_lut.red[SHDISP_ARGC_STAGE_NUM - 1][1] = 0x0100;
    shdisp_kerl_ctx.boot_ctx.argc_lut.green[SHDISP_ARGC_STAGE_NUM - 1][1] = 0x0100;
    shdisp_kerl_ctx.boot_ctx.argc_lut.blue[SHDISP_ARGC_STAGE_NUM - 1][1] = 0x0100;

    return;
}

/* ------------------------------------------------------------------------- */
/* CHECKER                                                                   */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_check_initialized                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_check_initialized(void)
{
    if (shdisp_kerl_ctx.driver_is_initialized == SHDISP_DRIVER_IS_INITIALIZED) {
        return SHDISP_RESULT_SUCCESS;
    } else {
        SHDISP_ERR("[SHDISP] shdisp_check_initialized error : driver is not initialized.");
        return SHDISP_RESULT_FAILURE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_check_upper_unit                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_check_upper_unit(void)
{
    if (shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected == SHDISP_UPPER_UNIT_IS_CONNECTED) {
        return SHDISP_RESULT_SUCCESS;
    } else {
        SHDISP_ERR("[SHDISP] shdisp_check_upper_unit error : upper unit is not connected.");
        return SHDISP_RESULT_FAILURE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_check_bdic_exist                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_check_bdic_exist(void)
{
    if (shdisp_kerl_ctx.boot_ctx.bdic_is_exist == SHDISP_BDIC_IS_EXIST) {
        return SHDISP_RESULT_SUCCESS;
    } else {
        SHDISP_ERR("[SHDISP] shdisp_check_bdic_exist error : bdic is not exist.");
        return SHDISP_RESULT_FAILURE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_boot_disp_status                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_get_boot_disp_status(void)
{
    return shdisp_kerl_ctx.boot_ctx.main_disp_status;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_hw_revision                                                    */
/* ------------------------------------------------------------------------- */
static unsigned short shdisp_get_hw_revision(void)
{
    return shdisp_kerl_ctx.boot_ctx.hw_revision;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_bdic_is_exist                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_get_bdic_is_exist(void)
{
    return shdisp_kerl_ctx.boot_ctx.bdic_is_exist;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_argc_lut                                                       */
/* ------------------------------------------------------------------------- */
static struct shdisp_argc_lut *shdisp_get_argc_lut(void)
{
    return &shdisp_kerl_ctx.boot_ctx.argc_lut;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_igc_lut                                                        */
/* ------------------------------------------------------------------------- */
static struct shdisp_igc_lut *shdisp_get_igc_lut(void)
{
    return &shdisp_kerl_ctx.boot_ctx.igc_lut;
}

/* ------------------------------------------------------------------------- */
/* FOPS                                                                      */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_open                                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_open(struct inode *inode, struct file *filp)
{

    if (shdisp_kerl_ctx.driver_open_cnt == 0) {
        SHDISP_DEBUG("[SHDISP] new open shdisp device driver.");
    }

    shdisp_kerl_ctx.driver_open_cnt++;

    if (shdisp_kerl_ctx.driver_open_cnt == 1 && !shdisp_kerl_ctx.driver_is_open) {
        shdisp_kerl_ctx.driver_is_open = true;

        shdisp_boot_err_output();
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl                                                              */
/* ------------------------------------------------------------------------- */
static long shdisp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret;
    void __user *argp = (void __user*)arg;

    switch (cmd) {
    case SHDISP_IOCTL_GET_CONTEXT:
        ret = shdisp_ioctl_get_context(argp);
        break;
    case SHDISP_IOCTL_SET_HOST_GPIO:
        ret = shdisp_ioctl_set_host_gpio(argp);
        break;
    case SHDISP_IOCTL_TRI_LED_SET_COLOR:
        ret = shdisp_ioctl_tri_led_set_color(argp);
        break;
    case SHDISP_IOCTL_BDIC_WRITE_REG:
        ret = shdisp_ioctl_bdic_write_reg(argp);
        break;
    case SHDISP_IOCTL_BDIC_READ_REG:
        ret = shdisp_ioctl_bdic_read_reg(argp);
        break;
    case SHDISP_IOCTL_BDIC_MULTI_READ_REG:
        ret = shdisp_ioctl_bdic_multi_read_reg(argp);
        break;
    case SHDISP_IOCTL_GET_LUX:
        SHDISP_PERFORMANCE("RESUME PANEL GET-LUX 0010 START");
        ret = shdisp_ioctl_get_lux(argp);
        SHDISP_PERFORMANCE("RESUME PANEL GET-LUX 0010 END");
        break;
    case SHDISP_IOCTL_PHOTO_SENSOR_POW_CTL:
        SHDISP_PERFORMANCE("RESUME PANEL PHOTO-SENCOR 0010 START");
        ret = shdisp_ioctl_photo_sensor_pow_ctl(argp);
        SHDISP_PERFORMANCE("RESUME PANEL PHOTO-SENCOR 0010 END");
        break;
    case SHDISP_IOCTL_LCDDR_WRITE_REG:
        ret = shdisp_ioctl_lcddr_write_reg(argp);
        break;
    case SHDISP_IOCTL_LCDDR_READ_REG:
        ret = shdisp_ioctl_lcddr_read_reg(argp);
        break;
    case SHDISP_IOCTL_SET_FLICKER_PARAM:
        ret = shdisp_ioctl_set_flicker_param(argp);
        break;
    case SHDISP_IOCTL_GET_FLICKER_PARAM:
        ret = shdisp_ioctl_get_flicker_param(argp);
        break;
    case SHDISP_IOCTL_BKL_SET_AUTO_MODE:
        ret = shdisp_ioctl_bkl_set_auto_mode(argp);
        break;
    case SHDISP_IOCTL_BKL_SET_DTV_MODE:
        ret = shdisp_ioctl_bkl_set_dtv_mode(argp);
        break;
    case SHDISP_IOCTL_BKL_SET_EMG_MODE:
        ret = shdisp_ioctl_bkl_set_emg_mode(argp);
        break;
    case SHDISP_IOCTL_LUX_CHANGE_IND:
        ret = shdisp_ioctl_lux_change_ind(argp);
        break;
    case SHDISP_IOCTL_SET_CABC:
        ret = shdisp_ioctl_set_cabc(argp);
        break;
    case SHDISP_IOCTL_BKL_SET_CHG_MODE:
        ret = shdisp_ioctl_bkl_set_chg_mode(argp);
        break;
    case SHDISP_IOCTL_GET_FLICKER_LOW_PARAM:
        ret = shdisp_ioctl_get_flicker_low_param(argp);
        break;
    case SHDISP_IOCTL_SET_GAMMATABLE_AND_VOLTAGE:
        SHDISP_TRACE("SHDISP_IOCTL_SET_GAMMATABLE_AND_VOLTAGE Requested. cmd=%08X", cmd);
        ret = shdisp_ioctl_panel_set_gammatable_and_voltage(argp);
        SHDISP_TRACE("SHDISP_IOCTL_SET_GAMMATABLE_AND_VOLTAGE Completed ret:%d", ret);
        break;
    case SHDISP_IOCTL_GET_GAMMATABLE_AND_VOLTAGE:
        SHDISP_TRACE("SHDISP_IOCTL_GET_GAMMATABLE_AND_VOLTAGE Requested. cmd=%08X", cmd);
        ret = shdisp_ioctl_panel_get_gammatable_and_voltage(argp);
        SHDISP_TRACE("SHDISP_IOCTL_GET_GAMMATABLE_AND_VOLTAGE Completed ret:%d", ret);
        break;
    case SHDISP_IOCTL_SET_GAMMA:
        SHDISP_TRACE("SHDISP_IOCTL_SET_GAMMA Requested. cmd=%08X", cmd);
        ret = shdisp_ioctl_panel_set_gamma(argp);
        SHDISP_TRACE("SHDISP_IOCTL_SET_GAMMA Completed ret:%d", ret);
        break;
    case SHDISP_IOCTL_GET_AVE_ADO:
        SHDISP_TRACE("SHDISP_IOCTL_GET_AVE_ADO Requested. cmd=%08X", cmd);
        ret = shdisp_ioctl_get_ave_ado(argp);
        SHDISP_TRACE("SHDISP_IOCTL_GET_AVE_ADO Completed ret:%d", ret);
        break;
    case SHDISP_IOCTL_PSALS_READ_REG:
        SHDISP_TRACE("SHDISP_IOCTL_PSALS_READ_REG Requested. cmd=%08X", cmd);
        ret = shdisp_ioctl_psals_read_reg(argp);
        SHDISP_TRACE("SHDISP_IOCTL_PSALS_READ_REG Completed ret:%d", ret);
        break;
    case SHDISP_IOCTL_PSALS_WRITE_REG:
        SHDISP_TRACE("SHDISP_IOCTL_PSALS_WRITE_REG Requested. cmd=%08X", cmd);
        ret = shdisp_ioctl_psals_write_reg(argp);
        SHDISP_TRACE("SHDISP_IOCTL_PSALS_WRITE_REG Completed ret:%d", ret);
        break;
    case SHDISP_IOCTL_GET_ALS:
        SHDISP_TRACE("SHDISP_IOCTL_GET_ALS Requested. cmd=%08X", cmd);
        ret = shdisp_ioctl_get_als(argp);
        SHDISP_TRACE("SHDISP_IOCTL_GET_ALS Completed ret:%d", ret);
        break;
#ifdef SHDISP_TRI_LED2
    case SHDISP_IOCTL_TRI_LED_SET_COLOR2:
        ret = shdisp_ioctl_tri_led_set_color2(argp);
        break;
    case SHDISP_IOCTL_INSERT_SP_PIERCE:
        ret = shdisp_ioctl_insert_sp_pierce(argp);
        break;
    case SHDISP_IOCTL_REMOVE_SP_PIERCE:
        ret = shdisp_ioctl_remove_sp_pierce(argp);
        break;
    case SHDISP_IOCTL_GET_SP_PIERCE_STATE:
        ret = shdisp_ioctl_get_sp_pierce_state(argp);
        break;
#endif  /* SHDISP_TRI_LED2 */
    case SHDISP_IOCTL_SET_IRQ_MASK:
        ret = shdisp_ioctl_set_irq_mask(argp);
        break;
    case SHDISP_IOCTL_VCOM_TRACKING:
        ret = shdisp_ioctl_vcom_tracking(argp);
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> cmd(0x%08x).", cmd);
        ret = -EFAULT;
        break;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_release                                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_release(struct inode *inode, struct file *filp)
{
    if (shdisp_kerl_ctx.driver_open_cnt > 0) {
        shdisp_kerl_ctx.driver_open_cnt--;

        if (shdisp_kerl_ctx.driver_open_cnt == 0) {
            SHDISP_DEBUG("[SHDISP] all close shdisp device driver.");
        }
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* IOCTL                                                                     */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_context                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_context(void __user *argp)
{
    int ret;
    struct shdisp_to_user_context shdisp_user_ctx;

    shdisp_user_ctx.hw_handset                  = shdisp_kerl_ctx.boot_ctx.hw_handset;
    shdisp_user_ctx.hw_revision                 = shdisp_kerl_ctx.boot_ctx.hw_revision;
    shdisp_user_ctx.handset_color               = shdisp_kerl_ctx.boot_ctx.handset_color;
    shdisp_user_ctx.upper_unit_is_connected     = shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected;
    shdisp_user_ctx.bdic_is_exist               = shdisp_kerl_ctx.boot_ctx.bdic_is_exist;
    shdisp_user_ctx.dtv_status                  = shdisp_kerl_ctx.dtv_status;
    shdisp_user_ctx.main_disp_status            = shdisp_kerl_ctx.boot_ctx.main_disp_status;
    memcpy(&(shdisp_user_ctx.igc_lut), &(shdisp_kerl_ctx.boot_ctx.igc_lut), sizeof(struct shdisp_igc_lut));
#if defined(CONFIG_SHDISP_PANEL_ANDY)
    if (shdisp_API_check_panel() != SHDISP_RESULT_SUCCESS) {
        shdisp_user_ctx.is_vcom_tracking            = shdisp_andy_API_vcom_is_adjusted();
    } else {
        shdisp_user_ctx.is_vcom_tracking            = 0;
    }
#else /* CONFIG_SHDISP_PANEL_ANDY */
    shdisp_user_ctx.is_vcom_tracking            = 0;
#endif /* CONFIG_SHDISP_PANEL_ANDY */

    shdisp_semaphore_start();

    ret = copy_to_user(argp, &shdisp_user_ctx, sizeof(struct shdisp_to_user_context));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_host_gpio                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_host_gpio(void __user *argp)
{
    int ret;
    struct shdisp_host_gpio host_gpio;

    host_gpio.num   = 0;
    host_gpio.value = 0;

    shdisp_semaphore_start();

    ret = shdisp_SQE_set_host_gpio(&host_gpio);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_host_gpio.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_tri_led_set_color                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_tri_led_set_color(void __user *argp)
{

    int ret;
    struct shdisp_tri_led tri_led;

    shdisp_semaphore_start();

    ret = copy_from_user(&tri_led, argp, sizeof(struct shdisp_tri_led));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_tri_led_set_color(&tri_led);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_tri_led_set_color.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

#ifdef SHDISP_TRI_LED2
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_tri_led_set_color2                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_tri_led_set_color2(void __user *argp)
{
    int ret;
    struct shdisp_tri_led tri_led;
    struct shdisp_tri_led ledwk;
    unsigned long flags = 0;
    unsigned char color;

    SHDISP_TRACE("in");

    ret = copy_from_user(&tri_led, argp, sizeof(struct shdisp_tri_led));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        return ret;
    }

    if (tri_led.green > 0 || tri_led.blue > 0) {
        return SHDISP_RESULT_FAILURE;
    }

    ledwk.red     = tri_led.red;
    ledwk.green   = 0;
    ledwk.blue    = 0;

    color = shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(&ledwk);

    spin_lock_irqsave(&shdisp_swic_spinlock, flags);

    if (color > 0) {
        shdisp_pierce_status.request = SHDISP_PIERCE_REQ_MODE_ON;
    } else {
        shdisp_pierce_status.request = SHDISP_PIERCE_REQ_MODE_OFF;
    }

    if (shdisp_pierce_status.insert == SHDISP_PIERCE_INS_MODE_ON) {
        if (shdisp_pierce_status.request == SHDISP_PIERCE_REQ_MODE_ON) {
            ret = shdisp_SYS_API_Host_gpio_request(SHDISP_GPIO_NUM_SP_SWIC);
            spin_unlock_irqrestore(&shdisp_swic_spinlock, flags);
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_API_Host_gpio_request.");
                return -EIO;
            }
        } else {
            ret = shdisp_SYS_API_Host_gpio_free(SHDISP_GPIO_NUM_SP_SWIC);
            spin_unlock_irqrestore(&shdisp_swic_spinlock, flags);
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_API_Host_gpio_free.");
                return -EIO;
            }
        }
    } else {
        spin_unlock_irqrestore(&shdisp_swic_spinlock, flags);
    }

    shdisp_semaphore_start();
    ret = shdisp_SQE_tri_led_set_color2(&tri_led);
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_tri_led_set_color2.");
        return -EIO;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_insert_sp_pierce                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_insert_sp_pierce(void __user *argp)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_SQE_insert_sp_pierce();

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_insert_sp_pierce.");
        return -EIO;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_remove_sp_pierce                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_remove_sp_pierce(void __user *argp)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_SQE_remove_sp_pierce();

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_remove_sp_pierce.");
        return -EIO;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_sp_pierce_state                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_sp_pierce_state(void __user *argp)
{
    int ret;
    int val;

    SHDISP_TRACE("in");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    shdisp_semaphore_start();

    if (shdisp_pierce_status.insert == SHDISP_PIERCE_INS_MODE_ON) {
        val = SHDISP_PIERCE_STATE_INSERT;
    } else {
        val = SHDISP_PIERCE_STATE_REMOVE;
    }
    ret = copy_to_user(argp, &val, sizeof(int));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }
    SHDISP_TRACE("out val=%d, *argp=%d", val, *((int*)argp));
    return SHDISP_RESULT_SUCCESS;
}
#endif  /* SHDISP_TRI_LED2 */

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bdic_write_reg                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bdic_write_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_bdic_reg bdic_reg;

    shdisp_semaphore_start();

    ret = copy_from_user(&bdic_reg, argp, sizeof(struct shdisp_diag_bdic_reg));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_bdic_write_reg(bdic_reg.reg, bdic_reg.val);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_bdic_write_reg.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bdic_read_reg                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bdic_read_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_bdic_reg bdic_reg;

    shdisp_semaphore_start();

    ret = copy_from_user(&bdic_reg, argp, sizeof(struct shdisp_diag_bdic_reg));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_bdic_read_reg(bdic_reg.reg, &(bdic_reg.val));

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_bdic_read_reg.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &bdic_reg, sizeof(struct shdisp_diag_bdic_reg));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bdic_multi_read_reg                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bdic_multi_read_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_bdic_reg_multi bdic_reg;

    shdisp_semaphore_start();

    ret = copy_from_user(&bdic_reg, argp, sizeof(struct shdisp_diag_bdic_reg_multi));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_bdic_multi_read_reg(bdic_reg.reg, bdic_reg.val, (int)bdic_reg.size);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_bdic_multi_read_reg.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &bdic_reg, sizeof(struct shdisp_diag_bdic_reg_multi));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_lux                                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_lux(void __user *argp)
{
    int ret;
    struct shdisp_photo_sensor_val val;

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&val, argp, sizeof(struct shdisp_photo_sensor_val));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind(&(val.mode));
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind.");
        val.result = SHDISP_RESULT_FAILURE;
    } else {
        val.result = SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_SQE_get_lux(&(val));

    SHDISP_DEBUG(" value=0x%04X, lux=%lu, mode=%d", val.value, val.lux, val.mode );

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_lux.");
    }

    ret = copy_to_user(argp, &val, sizeof(struct shdisp_photo_sensor_val));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_photo_sensor_pow_ctl                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_photo_sensor_pow_ctl(void __user *argp)
{
    int ret;
    struct shdisp_photo_sensor_power_ctl power_ctl;

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&power_ctl, argp, sizeof(struct shdisp_photo_sensor_power_ctl));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_photo_sensor_pow_ctl(&power_ctl);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_photo_sensor_pow_ctl.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_lcddr_write_reg                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_lcddr_write_reg(void __user *argp)
{
    int ret;
    struct shdisp_lcddr_reg panel_reg;

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&panel_reg, argp, sizeof(struct shdisp_lcddr_reg));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_panel_write_reg(&panel_reg);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_panel_write_reg.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_lcddr_read_reg                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_lcddr_read_reg(void __user *argp)
{
    int ret;
    struct shdisp_lcddr_reg panel_reg;

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&panel_reg, argp, sizeof(struct shdisp_lcddr_reg));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_panel_read_reg(&panel_reg);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_panel_read_reg.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &panel_reg, sizeof(struct shdisp_lcddr_reg));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_flicker_param                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_flicker_param(void __user *argp)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int ret;
    struct shdisp_diag_flicker_param vcom;

    shdisp_semaphore_start();

    ret = copy_from_user(&vcom, argp, sizeof(struct shdisp_diag_flicker_param));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_set_flicker_param(vcom);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_flicker_param.");
        return -EIO;
    }
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_flicker_param                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_flicker_param(void __user *argp)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int ret;
    struct shdisp_diag_flicker_param vcom;

    vcom.master_alpha = 0;
    vcom.slave_alpha = 0;

    shdisp_semaphore_start();

    ret = shdisp_SQE_get_flicker_param(&vcom);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_flicker_param.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &vcom, sizeof(struct shdisp_diag_flicker_param));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_flicker_low_param                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_flicker_low_param(void __user *argp)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int ret;
    struct shdisp_diag_flicker_param vcom;

    vcom.master_alpha = 0;
    vcom.slave_alpha = 0;

    shdisp_semaphore_start();

    ret = shdisp_SQE_get_flicker_low_param(&vcom);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_flicker_low_param.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &vcom, sizeof(struct shdisp_diag_flicker_param));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_auto_mode                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bkl_set_auto_mode(void __user *argp)
{
    int ret;
    struct shdisp_main_bkl_auto auto_bkl;
    struct shdisp_main_bkl_ctl bkl_ctl;

    SHDISP_PERFORMANCE("RESUME PANEL BACKLIGHT-SET-AUTO-MODE 0010 START");

    shdisp_semaphore_start();

    ret = copy_from_user(&auto_bkl, argp, sizeof(struct shdisp_main_bkl_auto));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    if (auto_bkl.mode == SHDISP_MAIN_BKL_AUTO_OFF) {
        SHDISP_DEBUG("BKL_AUTO_MODE : AUTO_OFF");
        bkl_ctl.mode  = SHDISP_MAIN_BKL_MODE_AUTO;
        bkl_ctl.param = SHDISP_MAIN_BKL_PARAM_OFF;
    } else if (auto_bkl.mode == SHDISP_MAIN_BKL_AUTO_ON) {
        if ((auto_bkl.param >= SHDISP_MAIN_BKL_PARAM_MIN_AUTO) &&
            (auto_bkl.param <= SHDISP_MAIN_BKL_PARAM_MAX_AUTO)) {
            SHDISP_DEBUG("BKL_AUTO_MODE : AUTO_ON");
            bkl_ctl.mode  = SHDISP_MAIN_BKL_MODE_AUTO;
            bkl_ctl.param = auto_bkl.param;
        } else {
            SHDISP_ERR("<INVALID_VALUE> mode(%d) param(%d)", auto_bkl.mode, auto_bkl.param);
            shdisp_semaphore_end(__func__);
            return -EIO;
        }
    } else {
        SHDISP_ERR("<INVALID_VALUE> mode(%d).", auto_bkl.mode);
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO, &(bkl_ctl));

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_ctl.");
        return -EIO;
    }

    SHDISP_PERFORMANCE("RESUME PANEL BACKLIGHT-SET-AUTO-MODE 0010 END");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_dtv_mode                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bkl_set_dtv_mode(void __user *argp)
{
    int ret;
    int dtv_mode;

    ret = copy_from_user(&dtv_mode, argp, sizeof(int));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        return ret;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_set_dtv_mode(dtv_mode);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_set_dtv_mode.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_emg_mode                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bkl_set_emg_mode(void __user *argp)
{
    int ret;
    int emg_mode;

    ret = copy_from_user(&emg_mode, argp, sizeof(int));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        return ret;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_set_emg_mode(emg_mode);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_set_emg_mode.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_lux_change_ind                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_lux_change_ind(void __user *argp)
{
    int ret;
    struct shdisp_photo_sensor_val val;


    down(&shdisp_lux_change_sem);

    ret = copy_from_user(&val, argp, sizeof(struct shdisp_photo_sensor_val));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        up(&shdisp_lux_change_sem);
        return ret;
    }

    ret = shdisp_SQE_lux_change_ind(&(val));

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_lux_change_ind.");
    }

    ret = copy_to_user(argp, &val, sizeof(struct shdisp_photo_sensor_val));

    up(&shdisp_lux_change_sem);


    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_cabc                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_cabc(void __user *argp)
{
    int ret;
    struct shdisp_main_dbc val;

    ret = copy_from_user(&val, argp, sizeof(struct shdisp_main_dbc));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        return ret;
    }

    SHDISP_DEBUG(" : shdisp_SQE_set_cabc S");
    SHDISP_DEBUG(" : val->mode = %d ", val.mode);
    SHDISP_DEBUG(" : val->auto_mode = %d ", val.auto_mode);
    ret = shdisp_SQE_set_cabc(&val);
    SHDISP_DEBUG(" : val->mode = %d ", val.mode);
    SHDISP_DEBUG(" : val->auto_mode = %d ", val.auto_mode);
    SHDISP_DEBUG(" : shdisp_SQE_set_cabc E");

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_cabc.");
    }

    ret = copy_to_user(argp, &val, sizeof(struct shdisp_main_dbc));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_chg_mode                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bkl_set_chg_mode(void __user *argp)
{
    int ret;
    int chg_mode;

    ret = copy_from_user(&chg_mode, argp, sizeof(int));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        return ret;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_set_chg_mode(chg_mode);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_set_chg_mode.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_panel_set_gammatable_and_voltage                             */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_panel_set_gammatable_and_voltage(void __user *argp)
{
    int ret;
    struct shdisp_diag_gamma_info gamma_info;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&gamma_info,
                         argp,
                         sizeof(struct shdisp_diag_gamma_info));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_set_gammatable_and_voltage(&gamma_info);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_gammatable_and_voltage.");
        return -EIO;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_panel_get_gammatable_and_voltage                             */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_panel_get_gammatable_and_voltage(void __user *argp)
{
    int ret;
    struct shdisp_diag_gamma_info gamma_info;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    shdisp_semaphore_start();

    memset(&gamma_info, 0, sizeof(gamma_info));
    ret = shdisp_SQE_get_gammatable_and_voltage(&gamma_info);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_gammatable_and_voltage.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp,
                       &gamma_info,
                       sizeof(struct shdisp_diag_gamma_info));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_panel_set_gamma                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_panel_set_gamma(void __user *argp)
{
    int ret;
    struct shdisp_diag_gamma gamma;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&gamma,
                         argp,
                         sizeof(struct shdisp_diag_gamma));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }


    ret = shdisp_SQE_set_gamma(&gamma);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_gamma.");
        return -EIO;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_ave_ado                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_ave_ado(void __user *argp)
{
    int ret;
    struct shdisp_ave_ado ave_ado;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&ave_ado, argp, sizeof(struct shdisp_ave_ado));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_get_ave_ado(&ave_ado);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_ioctl_get_ave_ado.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &ave_ado, sizeof(struct shdisp_ave_ado));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_psals_read_reg                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_psals_read_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_psals_reg psals_reg;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&psals_reg, argp, sizeof(struct shdisp_diag_psals_reg));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_psals_read_reg(&psals_reg);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_psals_read_reg.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &psals_reg, sizeof(struct shdisp_diag_psals_reg));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_psals_write_reg                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_psals_write_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_psals_reg psals_reg;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&psals_reg, argp, sizeof(struct shdisp_diag_psals_reg));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_psals_write_reg(&psals_reg);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_ioctl_get_ave_ado.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_als                                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_als(void __user *argp)
{
    int ret;
    struct shdisp_photo_sensor_raw_val val;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&val, argp, sizeof(struct shdisp_photo_sensor_raw_val));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_get_als(&val);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_als.");
    }

    ret = copy_to_user(argp, &val, sizeof(struct shdisp_photo_sensor_raw_val));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_irq_mask                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_irq_mask(void __user *argp)
{
    int ret;
    int irq_msk_ctl;

    ret = copy_from_user(&irq_msk_ctl, argp, sizeof(int));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_set_irq_mask(irq_msk_ctl);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_ioctl_set_irq_mask.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_vcom_tracking                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_vcom_tracking(void __user *argp)
{
    int ret;
    int tracking;

    ret = copy_from_user(&tracking, argp, sizeof(int));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        return ret;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_vcom_tracking(tracking);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_ioctl_vcom_tracking.\n");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* SEQUENCE                                                                  */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_power_on                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_power_on(void)
{

    SHDISP_TRACE("in");
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_ON);

    if (first_lcd_on) {
        shdisp_panel_API_power_on(SHDISP_PANEL_POWER_NORMAL_ON);
    } else {
        shdisp_panel_API_power_on(SHDISP_PANEL_POWER_FIRST_ON);
        first_lcd_on = true;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_power_off                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_power_off(void)
{
    SHDISP_TRACE("in");
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        shdisp_panel_API_power_off(SHDISP_PANEL_POWER_SHUTDOWN_OFF);
    } else {
        shdisp_panel_API_power_off(SHDISP_PANEL_POWER_NORMAL_OFF);
    }

    (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_OFF);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_disp_on                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_disp_on(void)
{
    SHDISP_TRACE("in");
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_panel_API_disp_on();

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_start_display                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_start_display(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_start_display();

    shdisp_kerl_ctx.main_disp_status = SHDISP_MAIN_DISP_ON;

#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_LCDPOW, 1);
#endif  /* CONFIG_SHTERM */

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_post_video_start                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_post_video_start(void)
{
    int ret;
    SHDISP_TRACE("in");

    ret = shdisp_panel_API_post_video_start();

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_display_done                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_display_done(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_lcd_det_recovery_subscribe();

    ret = shdisp_SQE_check_recovery();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("lcd det low");
        shdisp_do_lcd_det_recovery();
    }

    shdisp_det_mipi_err_ctrl(true);

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_disp_off                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_disp_off(void)
{
    SHDISP_TRACE("in");
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_OFF) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_det_mipi_err_ctrl(false);

    shdisp_panel_API_disp_off();

    shdisp_kerl_ctx.main_disp_status = SHDISP_MAIN_DISP_OFF;

#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_LCDPOW, 0);
#endif  /* CONFIG_SHTERM */

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_shutdown                                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_shutdown(void)
{
    SHDISP_TRACE("in main_disp_status=%d", shdisp_kerl_ctx.main_disp_status);

    shdisp_kerl_ctx.shutdown_in_progress = true;

#if defined(CONFIG_SHDISP_PANEL_ANDY)
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_OFF) {
        SHDISP_DEBUG("shutdown progress main_disp_status=%d", shdisp_kerl_ctx.main_disp_status);
        (void)shdisp_panel_API_shutdown();
    }
#endif /* CONFIG_SHDISP_PANEL_ANDY */

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_ctl                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_bkl_ctl(int type, struct shdisp_main_bkl_ctl *bkl)
{
    struct shdisp_main_bkl_ctl temp, request;
    unsigned long int notify_value = 0, notify_brightness = 0;

    SHDISP_TRACE("in");
    if (type >= NUM_SHDISP_MAIN_BKL_DEV_TYPE) {
        SHDISP_ERR("<INVALID_VALUE> type(%d).", type);
        return SHDISP_RESULT_FAILURE;
    }

    temp.mode  = bkl->mode;
    temp.param = bkl->param;
    shdisp_bdic_API_LCD_BKL_get_request(type, &temp, &request);

    if ((request.mode == shdisp_kerl_ctx.main_bkl.mode) && (request.param == shdisp_kerl_ctx.main_bkl.param)) {
        return SHDISP_RESULT_SUCCESS;
    }

    switch (request.mode) {
    case SHDISP_MAIN_BKL_MODE_OFF:
        shdisp_bdic_API_LCD_BKL_off();
        notify_value = 0;
        notify_brightness = 0;
        break;
    case SHDISP_MAIN_BKL_MODE_FIX:
        shdisp_bdic_API_LCD_BKL_fix_on(request.param);
        notify_value = 1;
        shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
        break;
    case SHDISP_MAIN_BKL_MODE_AUTO:
        shdisp_bdic_API_LCD_BKL_auto_on(request.param);
        notify_value = 1;
        shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
        break;
    default:
        break;
    }

    shdisp_kerl_ctx.main_bkl.mode  = request.mode;
    shdisp_kerl_ctx.main_bkl.param = request.param;

#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT, notify_value);
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT_LEV, notify_brightness);
#endif  /* CONFIG_SHTERM */

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_set_dtv_mode                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_bkl_set_dtv_mode(int dtv_mode)
{
    unsigned long int notify_brightness = 0;

    if (dtv_mode == SHDISP_MAIN_BKL_DTV_OFF) {
        SHDISP_DEBUG("BKL_DTV_MODE : DTV_OFF");
        shdisp_bdic_API_LCD_BKL_dtv_off();
    } else if (dtv_mode == SHDISP_MAIN_BKL_DTV_ON) {
        SHDISP_DEBUG("BKL_DTV_MODE : DTV_ON");
        shdisp_bdic_API_LCD_BKL_dtv_on();
    } else {
        SHDISP_ERR("<INVALID_VALUE> dtv_mode(%d).", dtv_mode);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_kerl_ctx.dtv_status = dtv_mode;

    shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT_LEV, notify_brightness);
#endif  /* CONFIG_SHTERM */

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_set_emg_mode                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_bkl_set_emg_mode(int emg_mode)
{
    unsigned long int notify_brightness = 0;

    if (emg_mode == SHDISP_MAIN_BKL_EMG_OFF) {
        SHDISP_DEBUG("BKL_EMG_MODE : NORMAL");
        shdisp_bdic_API_LCD_BKL_emg_off();
    } else if (emg_mode == SHDISP_MAIN_BKL_EMG_ON) {
        SHDISP_DEBUG("BKL_EMG_MODE : EMERGENCY");
        shdisp_bdic_API_LCD_BKL_emg_on();
    } else {
        SHDISP_ERR("<INVALID_VALUE> emg_mode(%d).", emg_mode);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_kerl_ctx.thermal_status = emg_mode;

    shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT_LEV, notify_brightness);
#endif  /* CONFIG_SHTERM */

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_set_chg_mode                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_bkl_set_chg_mode(int chg_mode)
{
    unsigned long int notify_brightness = 0;

    if (chg_mode == SHDISP_MAIN_BKL_CHG_OFF) {
        SHDISP_DEBUG("BKL_CHG_MODE : OFF");
        shdisp_bdic_API_LCD_BKL_chg_off();
    } else if (chg_mode == SHDISP_MAIN_BKL_CHG_ON) {
        SHDISP_DEBUG("BKL_CHG_MODE : ON");
        shdisp_bdic_API_LCD_BKL_chg_on();
    } else {
        SHDISP_ERR("<INVALID_VALUE> chg_mode(%d).", chg_mode);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_kerl_ctx.usb_chg_status = chg_mode;

    shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT_LEV, notify_brightness);
#endif  /* CONFIG_SHTERM */

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_host_gpio                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_host_gpio(struct shdisp_host_gpio *host_gpio)
{
    int ret;

    ret = shdisp_SYS_API_set_Host_gpio((host_gpio->num), (host_gpio->value));

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_API_set_Host_gpio.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_tri_led_set_color                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_tri_led_set_color(struct shdisp_tri_led *tri_led)
{
    unsigned char color, xstb_ch012;
    struct shdisp_tri_led  led;

    led.red = tri_led->red;
    led.green = tri_led->green;
    led.blue = tri_led->blue;
    color = shdisp_bdic_API_TRI_LED_get_color_index_and_reedit( &led );

    if (tri_led->led_mode == SHDISP_TRI_LED_MODE_NORMAL) {
        if ((shdisp_kerl_ctx.tri_led.red      == led.red) &&
            (shdisp_kerl_ctx.tri_led.green    == led.green) &&
            (shdisp_kerl_ctx.tri_led.blue     == led.blue) &&
            (shdisp_kerl_ctx.tri_led.led_mode == tri_led->led_mode)) {
            return SHDISP_RESULT_SUCCESS;
        }
    }

    xstb_ch012 = (color == 0) ? 0 : 1;

    if (xstb_ch012 != 0) {
        SHDISP_DEBUG("led_mode=%d color:%d, ontime:%d, interval:%d, count:%d",
                      tri_led->led_mode, color, tri_led->ontime, tri_led->interval, tri_led->count);

        switch (tri_led->led_mode) {
        case SHDISP_TRI_LED_MODE_NORMAL:
            shdisp_bdic_API_TRI_LED_normal_on(color);
            break;
        case SHDISP_TRI_LED_MODE_BLINK:
            shdisp_bdic_API_TRI_LED_blink_on(color, tri_led->ontime, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_FIREFLY:
            shdisp_bdic_API_TRI_LED_firefly_on(color, tri_led->ontime, tri_led->interval, tri_led->count);
            break;
#ifdef SHDISP_EXTEND_COLOR_LED
        case SHDISP_TRI_LED_MODE_HISPEED:
            shdisp_bdic_API_TRI_LED_high_speed_on(color, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_STANDARD:
            shdisp_bdic_API_TRI_LED_standard_on(color, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_BREATH:
            shdisp_bdic_API_TRI_LED_breath_on(color, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_LONG_BREATH:
            shdisp_bdic_API_TRI_LED_long_breath_on(color, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_WAVE:
            shdisp_bdic_API_TRI_LED_wave_on(color, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_FLASH:
            shdisp_bdic_API_TRI_LED_flash_on(color, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_AURORA:
            shdisp_bdic_API_TRI_LED_aurora_on(tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_RAINBOW:
            shdisp_bdic_API_TRI_LED_rainbow_on(tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_PATTERN1:
            shdisp_bdic_API_TRI_LED_pattern1_on(tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_PATTERN2:
            shdisp_bdic_API_TRI_LED_pattern2_on(tri_led->interval, tri_led->count);
            break;
#endif  /* SHDISP_EXTEND_COLOR_LED */
        default:
            SHDISP_ERR("led_mode=%d not supported.", tri_led->led_mode);
            break;
        }
    } else {
        shdisp_bdic_API_TRI_LED_off();
    }

    shdisp_kerl_ctx.tri_led.red      = led.red;
    shdisp_kerl_ctx.tri_led.green    = led.green;
    shdisp_kerl_ctx.tri_led.blue     = led.blue;
    shdisp_kerl_ctx.tri_led.led_mode = tri_led->led_mode;
    if ((tri_led->led_mode == SHDISP_TRI_LED_MODE_BLINK) ||
        (tri_led->led_mode == SHDISP_TRI_LED_MODE_FIREFLY)) {
        shdisp_kerl_ctx.tri_led.ontime   = tri_led->ontime;
        shdisp_kerl_ctx.tri_led.interval = tri_led->interval;
        shdisp_kerl_ctx.tri_led.count    = tri_led->count;
    } else if (tri_led->led_mode != SHDISP_TRI_LED_MODE_NORMAL) {
        shdisp_kerl_ctx.tri_led.interval = tri_led->interval;
        shdisp_kerl_ctx.tri_led.count    = tri_led->count;
    }
    return SHDISP_RESULT_SUCCESS;
}

#ifdef SHDISP_TRI_LED2
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_tri_led_set_color2                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_tri_led_set_color2(struct shdisp_tri_led *tri_led)
{
    unsigned char color;
    struct shdisp_tri_led led;
    static struct shdisp_tri_led before_led2;

    SHDISP_TRACE("in");

    if (tri_led->green > 0 || tri_led->blue > 0) {
        return SHDISP_RESULT_FAILURE;
    }

    if (tri_led->red >= 2) {
        tri_led->red = 1;
    }

    led.red     = tri_led->red;
    led.green   = tri_led->green;
    led.blue    = tri_led->blue;

    color = shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(&led);

    if (tri_led->led_mode == SHDISP_TRI_LED_MODE_NORMAL) {
        if ((before_led2.red      == led.red) &&
            (before_led2.led_mode == tri_led->led_mode)) {
            return SHDISP_RESULT_SUCCESS;
        }
    }

    if (color != 0) {
        shdisp_pierce_status.led2.red      = led.red;
        shdisp_pierce_status.led2.green    = led.green;
        shdisp_pierce_status.led2.blue     = led.blue;
        shdisp_pierce_status.led2.ext_mode = tri_led->ext_mode;
        shdisp_pierce_status.led2.led_mode = tri_led->led_mode;
        shdisp_pierce_status.led2.ontime   = tri_led->ontime;
        shdisp_pierce_status.led2.interval = tri_led->interval;
        shdisp_pierce_status.led2.count    = tri_led->count;

        if (shdisp_pierce_status.insert == SHDISP_PIERCE_INS_MODE_ON) {
            SHDISP_DEBUG("led_mode=%d color:%d, ontime:%d, interval:%d, count:%d",
                          tri_led->led_mode, color, tri_led->ontime, tri_led->interval, tri_led->count);

            switch (tri_led->led_mode) {
            case SHDISP_TRI_LED_MODE_NORMAL:
                shdisp_bdic_API_TRI_LED_normal_on2(color);
                break;
            case SHDISP_TRI_LED_MODE_BLINK:
                shdisp_bdic_API_TRI_LED_blink_on2(color, tri_led->ontime, tri_led->interval, tri_led->count);
                break;
            case SHDISP_TRI_LED_MODE_FIREFLY:
                shdisp_bdic_API_TRI_LED_firefly_on2(color, tri_led->ontime, tri_led->interval, tri_led->count);
                break;
#ifdef SHDISP_EXTEND_COLOR_LED
            case SHDISP_TRI_LED_MODE_HISPEED:
                shdisp_bdic_API_TRI_LED_high_speed_on2(color, tri_led->interval, tri_led->count);
                break;
            case SHDISP_TRI_LED_MODE_STANDARD:
                shdisp_bdic_API_TRI_LED_standard_on2(color, tri_led->interval, tri_led->count);
                break;
            case SHDISP_TRI_LED_MODE_BREATH:
                shdisp_bdic_API_TRI_LED_breath_on2(color, tri_led->interval, tri_led->count);
                break;
            case SHDISP_TRI_LED_MODE_LONG_BREATH:
                shdisp_bdic_API_TRI_LED_long_breath_on2(color, tri_led->interval, tri_led->count);
                break;
            case SHDISP_TRI_LED_MODE_WAVE:
                shdisp_bdic_API_TRI_LED_wave_on2(color, tri_led->interval, tri_led->count);
                break;
            case SHDISP_TRI_LED_MODE_FLASH:
                shdisp_bdic_API_TRI_LED_flash_on2(color, tri_led->interval, tri_led->count);
                break;
            case SHDISP_TRI_LED_MODE_AURORA:
                shdisp_bdic_API_TRI_LED_aurora_on2(tri_led->interval, tri_led->count);
                break;
            case SHDISP_TRI_LED_MODE_RAINBOW:
                shdisp_bdic_API_TRI_LED_rainbow_on2(tri_led->interval, tri_led->count);
                break;
            case SHDISP_TRI_LED_MODE_PATTERN1:
                shdisp_bdic_API_TRI_LED_pattern1_on2(tri_led->interval, tri_led->count);
                break;
            case SHDISP_TRI_LED_MODE_PATTERN2:
                shdisp_bdic_API_TRI_LED_pattern2_on2(tri_led->interval, tri_led->count);
                break;
#endif  /* SHDISP_EXTEND_COLOR_LED */
            default:
                SHDISP_ERR("led_mode=%d not supported.", tri_led->led_mode);
                break;
            }

            switch (tri_led->led_mode) {
            case SHDISP_TRI_LED_MODE_NORMAL:
            case SHDISP_TRI_LED_MODE_BLINK:
            case SHDISP_TRI_LED_MODE_FIREFLY:
#ifdef SHDISP_EXTEND_COLOR_LED
            case SHDISP_TRI_LED_MODE_HISPEED:
            case SHDISP_TRI_LED_MODE_STANDARD:
            case SHDISP_TRI_LED_MODE_BREATH:
            case SHDISP_TRI_LED_MODE_LONG_BREATH:
            case SHDISP_TRI_LED_MODE_WAVE:
            case SHDISP_TRI_LED_MODE_FLASH:
            case SHDISP_TRI_LED_MODE_AURORA:
            case SHDISP_TRI_LED_MODE_RAINBOW:
#endif  /* SHDISP_EXTEND_COLOR_LED */
                shdisp_pierce_status.onoff = SHDISP_PIERCE_LED_MODE_ON;
                break;
            default:
                break;
            }
        } else {
            return SHDISP_RESULT_SUCCESS;
        }
    } else {
        shdisp_bdic_API_TRI_LED_off2();
        shdisp_pierce_status.onoff = SHDISP_PIERCE_LED_MODE_OFF;
    }

    before_led2.red      = led.red;
    before_led2.green    = led.green;
    before_led2.blue     = led.blue;
    before_led2.led_mode = tri_led->led_mode;
    if ((tri_led->led_mode == SHDISP_TRI_LED_MODE_BLINK) ||
        (tri_led->led_mode == SHDISP_TRI_LED_MODE_FIREFLY)) {
        before_led2.ontime   = tri_led->ontime;
        before_led2.interval = tri_led->interval;
        before_led2.count    = tri_led->count;
    } else if (tri_led->led_mode != SHDISP_TRI_LED_MODE_NORMAL) {
        before_led2.interval = tri_led->interval;
        before_led2.count    = tri_led->count;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_insert_sp_pierce                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_insert_sp_pierce(void)
{
    int ret = SHDISP_RESULT_FAILURE;
    unsigned long flags = 0;

    SHDISP_TRACE("in");

    spin_lock_irqsave(&shdisp_swic_spinlock, flags);
    shdisp_pierce_status.insert = SHDISP_PIERCE_INS_MODE_ON;

    if (shdisp_pierce_status.request == SHDISP_PIERCE_REQ_MODE_ON) {
        ret = shdisp_SYS_API_Host_gpio_request(SHDISP_GPIO_NUM_SP_SWIC);
        spin_unlock_irqrestore(&shdisp_swic_spinlock, flags);
        if (ret != SHDISP_RESULT_SUCCESS) {
            return SHDISP_RESULT_FAILURE;
        }

        if (shdisp_wq_pierce) {
            ret = queue_work(shdisp_wq_pierce, &shdisp_wq_pierce_wk);
            if (ret == 0) {
                SHDISP_ERR("<QUEUE_WORK_FAILURE> ");
                return SHDISP_RESULT_FAILURE;
            }
        }
    } else {
        spin_unlock_irqrestore(&shdisp_swic_spinlock, flags);
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_remove_sp_pierce                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_remove_sp_pierce(void)
{
    int ret = SHDISP_RESULT_FAILURE;
    unsigned long flags = 0;

    SHDISP_TRACE("in");

    spin_lock_irqsave(&shdisp_swic_spinlock, flags);
    shdisp_pierce_status.insert = SHDISP_PIERCE_INS_MODE_OFF;

    if (shdisp_pierce_status.request == SHDISP_PIERCE_REQ_MODE_ON) {
        ret = shdisp_SYS_API_Host_gpio_free(SHDISP_GPIO_NUM_SP_SWIC);
        spin_unlock_irqrestore(&shdisp_swic_spinlock, flags);
        if (ret != SHDISP_RESULT_SUCCESS) {
            return SHDISP_RESULT_FAILURE;
        }

        if (shdisp_wq_pierce) {
            ret = queue_work(shdisp_wq_pierce, &shdisp_wq_pierce_wk);
            if (ret == 0) {
                SHDISP_ERR("<QUEUE_WORK_FAILURE> ");
                return SHDISP_RESULT_FAILURE;
            }
        }
    } else {
        spin_unlock_irqrestore(&shdisp_swic_spinlock, flags);
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}
#endif  /* SHDISP_TRI_LED2 */

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_bdic_write_reg                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_bdic_write_reg(unsigned char reg, unsigned char val)
{
    shdisp_bdic_API_DIAG_write_reg(reg, val);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_bdic_read_reg                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_bdic_read_reg(unsigned char reg, unsigned char *val)
{
    shdisp_bdic_API_DIAG_read_reg(reg, val);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_bdic_multi_read_reg                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_bdic_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret = 0;

    ret = shdisp_bdic_API_DIAG_multi_read_reg(reg, val, size);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_lux                                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_lux(struct shdisp_photo_sensor_val *value)
{
    int ret;

    ret = shdisp_bdic_API_PHOTO_SENSOR_get_lux(&(value->value), &(value->lux));

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_PHOTO_SENSOR_get_lux.");
        value->result = SHDISP_RESULT_FAILURE;
    } else {
        value->result = SHDISP_RESULT_SUCCESS;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_als                                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_als(struct shdisp_photo_sensor_raw_val *raw_val)
{
    int ret;

    ret = shdisp_bdic_API_PHOTO_SENSOR_get_raw_als(&(raw_val->clear), &(raw_val->ir));

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_PHOTO_SENSOR_get_lux.");
        raw_val->result = SHDISP_RESULT_FAILURE;
    } else {
        raw_val->result = SHDISP_RESULT_SUCCESS;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_write_bdic_i2c                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg)
{
    int ret;

    ret = shdisp_bdic_API_i2c_transfer(i2c_msg);

    if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_bdic_API_i2c_transfer.");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    } else if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_i2c_transfer.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_read_bdic_i2c                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg)
{
    int ret;

    ret = shdisp_bdic_API_i2c_transfer(i2c_msg);

    if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_bdic_API_i2c_transfer.");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    } else if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_i2c_transfer.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_photo_sensor_pow_ctl                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_photo_sensor_pow_ctl(struct shdisp_photo_sensor_power_ctl *ctl)
{
    int ret;

    ret = shdisp_bdic_API_als_sensor_pow_ctl(ctl->type, ctl->power);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_als_sensor_pow_ctl.");
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_panel_write_reg                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_panel_write_reg(struct shdisp_lcddr_reg *panel_reg)
{
    int ret;

    ret = shdisp_panel_API_diag_write_reg(panel_reg->address, panel_reg->buf, panel_reg->size);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_write_reg.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_panel_read_reg                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_panel_read_reg(struct shdisp_lcddr_reg *panel_reg)
{
    int ret;

    ret = shdisp_panel_API_diag_read_reg(panel_reg->address, panel_reg->buf, panel_reg->size);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_read_reg.");
        return SHDISP_RESULT_FAILURE;
    }


    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_prox_sensor_pow_ctl                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_prox_sensor_pow_ctl(int power_mode)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in power_mode=%d", power_mode);

    switch (power_mode) {
    case SHDISP_PROX_SENSOR_POWER_OFF:
        shdisp_pm_API_ps_user_manager(SHDISP_DEV_TYPE_PS, SHDISP_DEV_REQ_OFF);
        break;

    case SHDISP_PROX_SENSOR_POWER_ON:
        shdisp_pm_API_ps_user_manager(SHDISP_DEV_TYPE_PS, SHDISP_DEV_REQ_ON);
        break;

    default:
        ret = SHDISP_RESULT_FAILURE;
        SHDISP_ERR("POWER_MODE ERROR(mode=%d)", power_mode);
        break;
    }

    SHDISP_TRACE("out ret=%d", ret);
    return ret;

}

#ifndef SHDISP_NOT_SUPPORT_FLICKER
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_flicker_param                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_flicker_param(struct shdisp_diag_flicker_param vcom)
{
    int ret;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_diag_set_flicker_param(vcom);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_set_flicker_param.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_flicker_param                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_flicker_param(struct shdisp_diag_flicker_param *vcom)
{
    int ret;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_diag_get_flicker_param(vcom);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_get_flicker_param.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_flicker_low_param                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_flicker_low_param(struct shdisp_diag_flicker_param *vcom)
{
    int ret;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_diag_get_flicker_low_param(vcom);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_get_flicker_low_param.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_check_recovery                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_check_recovery(void)
{
    int ret;

    ret = shdisp_panel_API_check_recovery();

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_check_recovery.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

#ifndef SHDISP_NOT_SUPPORT_DET
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_do_recovery                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_do_recovery(void)
{
    SHDISP_DEBUG("recovery : start");

    shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_RECOVERY, SHDISP_DEV_REQ_ON);

    shdisp_SQE_main_lcd_disp_off();

#if defined(CONFIG_SHDISP_PANEL_ANDY)
    mdss_shdisp_video_transfer_ctrl(false);
    mdss_shdisp_set_lp00_mode(false);
#endif /* CONFIG_SHDISP_PANEL_ANDY */

    shdisp_panel_API_power_off(SHDISP_PANEL_POWER_RECOVERY_OFF);

    shdisp_kerl_ctx.main_disp_status = SHDISP_MAIN_DISP_OFF;

#if defined(CONFIG_SHDISP_PANEL_ANDY)   /* CONFIG_SHDISP_PANEL_ANDY */
    msleep(500);
#elif defined(CONFIG_SHDISP_PANEL_ARIA) /* CONFIG_SHDISP_PANEL_ARIA */
    msleep(40);
#else  /* CONFIG_SHDISP_PANEL_XXXX */
#warning "unknown panel!!!"
    msleep(500);
#endif  /* CONFIG_SHDISP_PANEL_ANDY || CONFIG_SHDISP_PANEL_ARIA */

    shdisp_kerl_ctx.main_disp_status = SHDISP_MAIN_DISP_ON;
    shdisp_panel_API_power_on(SHDISP_PANEL_POWER_RECOVERY_ON);

#if defined(CONFIG_SHDISP_PANEL_ANDY)
    mdss_shdisp_set_lp00_mode(true);
#endif /* CONFIG_SHDISP_PANEL_ANDY */

    shdisp_panel_API_disp_on();

#if defined(CONFIG_SHDISP_PANEL_ARIA)
    mdss_shdisp_mdp_cmd_kickoff();
#endif  /* CONFIG_SHDISP_PANEL_ARIA */

    shdisp_panel_API_start_display();

#if defined(CONFIG_SHDISP_PANEL_ANDY)
    mdss_shdisp_video_transfer_ctrl(true);
#endif /* CONFIG_SHDISP_PANEL_ANDY */

    shdisp_panel_API_post_video_start();

    shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_RECOVERY, SHDISP_DEV_REQ_OFF);

    SHDISP_DEBUG("recovery : end");

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_NOT_SUPPORT_DET */

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_event_subscribe                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_event_subscribe(struct shdisp_subscribe *subscribe)
{
    int ret;
    int i;
    int bAllNull = 0;

    if (shdisp_subscribe_type_table[subscribe->irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
        ret = shdisp_bdic_API_IRQ_check_type(subscribe->irq_type);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_IRQ_check_type.");
            return SHDISP_RESULT_FAILURE;
        }
    }

    down(&shdisp_sem_callback);

    bAllNull = 1;
    for (i = 0; i < NUM_SHDISP_IRQ_TYPE; i++) {
        if ((shdisp_subscribe_type_table[i] == SHDISP_SUBSCRIBE_TYPE_INT) &&
            (shdisp_callback_table[i] != NULL)) {
            bAllNull = 0;
        }
    }

    if (shdisp_callback_table[subscribe->irq_type] != NULL) {
        SHDISP_DEBUG("INT_SUBSCRIBE CHANGE(irq_type=%d)", subscribe->irq_type);
    } else {
        SHDISP_DEBUG("INT_SUBSCRIBE NEW ENTRY(irq_type=%d)", subscribe->irq_type);
    }

    shdisp_callback_table[subscribe->irq_type] = subscribe->callback;

    if (shdisp_subscribe_type_table[subscribe->irq_type] != SHDISP_SUBSCRIBE_TYPE_INT) {
        shdisp_timer_int_register();
    }

    if (subscribe->irq_type == SHDISP_IRQ_TYPE_DET) {
        shdisp_bdic_API_IRQ_det_irq_ctrl(true);
    }

    up(&shdisp_sem_callback);

    if (shdisp_subscribe_type_table[subscribe->irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
        if (bAllNull) {
            SHDISP_DEBUG("INT_SUBSCRIBE enable_irq");
            shdisp_SYS_API_set_irq(SHDISP_IRQ_ENABLE);
        }
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_event_unsubscribe                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_event_unsubscribe(int irq_type)
{
    int ret;
    int i;
    int bAllNull = 0;

    if (shdisp_subscribe_type_table[irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
        ret = shdisp_bdic_API_IRQ_check_type(irq_type);
        if (ret !=  SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_IRQ_check_type.");
            return SHDISP_RESULT_FAILURE;
        }
    }

    down(&shdisp_sem_callback);

    if (shdisp_callback_table[irq_type] == NULL) {
        SHDISP_DEBUG("INT_UNSUBSCRIBE DONE(irq_type=%d)", irq_type);
    } else {
        if (shdisp_subscribe_type_table[irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
        } else {
            shdisp_timer_int_delete();
        }
        if (irq_type == SHDISP_IRQ_TYPE_DET) {
            shdisp_bdic_API_IRQ_det_irq_ctrl(false);
        }

        shdisp_callback_table[irq_type] = NULL;

        if (shdisp_subscribe_type_table[irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
            bAllNull = 1;
            for (i = 0; i < NUM_SHDISP_IRQ_TYPE; i++) {
                if ((shdisp_subscribe_type_table[i] == SHDISP_SUBSCRIBE_TYPE_INT) &&
                    (shdisp_callback_table[i] != NULL)) {
                    bAllNull = 0;
                }
            }
            if (bAllNull) {
                shdisp_SYS_API_set_irq(SHDISP_IRQ_DISABLE);
                SHDISP_DEBUG("INT_UNSUBSCRIBE disable_irq");
            }
        }

        SHDISP_DEBUG("INT_UNSUBSCRIBE SUCCESS(irq_type=%d)", irq_type);
    }

    up(&shdisp_sem_callback);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_lux_change_ind                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_lux_change_ind(struct shdisp_photo_sensor_val *value)
{
    int ret;

    SHDISP_DEBUG(": wait complete");

    lux_change_wait_flg = SHDISP_LUX_CHANGE_STATE_WAIT;
    INIT_COMPLETION(lux_change_notify);
    ret = wait_for_completion_interruptible(&lux_change_notify);
    if (ret != 0) {
        value->result = SHDISP_RESULT_FAILURE;
        return SHDISP_RESULT_FAILURE;
    }

    if (lux_change_wait_flg == SHDISP_LUX_CHANGE_STATE_EXIT) {
        value->result = SHDISP_RESULT_FAILURE;
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_DEBUG(": wake up by complete");

    shdisp_semaphore_start();

    ret = shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind(&(value->mode));

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind.");
        value->result = SHDISP_RESULT_FAILURE;
    } else {
        value->result = SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_SQE_get_lux(value);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_lux.");
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_cabc                                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_cabc(struct shdisp_main_dbc *value)
{
    int ret;

    SHDISP_DEBUG(": value->mode = %d ", value->mode);
    SHDISP_DEBUG(": value->auto_mode = %d ", value->auto_mode);

    if (!(value->mode == 0 || value->mode == 1)) {
        SHDISP_ERR("<RESULT_FAILURE> : value->mode .");
        return SHDISP_RESULT_FAILURE;
    }

    if (!(value->auto_mode == 0 || value->auto_mode == 1)) {
        SHDISP_ERR("<RESULT_FAILURE> : value->auto_mode .");
        return SHDISP_RESULT_FAILURE;
    }

    if (value->mode == 0 && value->auto_mode == 0) {
        SHDISP_DEBUG(": SHDISP_MAIN_DISP_CABC_MODE_OFF ");
        cabc_mode_set = SHDISP_MAIN_DISP_CABC_MODE_OFF;
    } else if (value->mode == 1 && value->auto_mode == 0) {
        SHDISP_DEBUG(": SHDISP_MAIN_DISP_CABC_MODE_DBC ");
        cabc_mode_set = SHDISP_MAIN_DISP_CABC_MODE_DBC;
    } else if (value->mode == 0 && value->auto_mode == 1) {
        SHDISP_DEBUG(": SHDISP_MAIN_DISP_CABC_MODE_ACC ");
        cabc_mode_set = SHDISP_MAIN_DISP_CABC_MODE_ACC;
    } else if (value->mode == 1 && value->auto_mode == 1) {
        SHDISP_DEBUG(": SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC ");
        cabc_mode_set = SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC;
    }


    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_gammatable_and_voltage                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info)
{
    int ret;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_diag_set_gammatable_and_voltage(gamma_info);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_set_gammatable_and_voltage.");
        return SHDISP_RESULT_FAILURE;
    }


    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_gammatable_and_voltage                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info)
{
    int ret;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_diag_get_gammatable_and_voltage(gamma_info);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_get_gammatable_and_voltage.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_gamma                                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_gamma(struct shdisp_diag_gamma *gamma)
{
    int ret;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_diag_set_gamma(gamma);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_set_gamma.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_psals_read_reg                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_psals_read_reg(struct shdisp_diag_psals_reg *psals_reg)
{
    int ret;
    struct shdisp_bdic_i2c_msg i2c_msg;

    if ((shdisp_pm_API_is_ps_active() != SHDISP_DEV_STATE_ON) &&
        (shdisp_pm_API_is_als_active() != SHDISP_DEV_STATE_ON)) {
        SHDISP_ERR("<RESULT_FAILURE> ps&als is not active.");
        return SHDISP_RESULT_FAILURE;
    }

    if (psals_reg->reg < SENSOR_REG_COMMAND1 || psals_reg->reg > SENSOR_REG_D2_MSB) {
        SHDISP_ERR("<RESULT_FAILURE> Register address out of range.");
        return SHDISP_RESULT_FAILURE;
   }
    SHDISP_DEBUG("(Register (addr:0x%02x)", psals_reg->reg);

    i2c_msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
    i2c_msg.mode = SHDISP_BDIC_I2C_M_R;
    i2c_msg.wlen = 1;
    i2c_msg.rlen = 1;
    i2c_msg.wbuf = &psals_reg->reg;
    i2c_msg.rbuf = &psals_reg->val;
    ret = shdisp_bdic_API_i2c_transfer(&i2c_msg);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_psals_read_reg.");
        return SHDISP_RESULT_FAILURE;
    }
    SHDISP_DEBUG(" Read data(0x%02x)", psals_reg->val);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_psals_write_reg                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_psals_write_reg(struct shdisp_diag_psals_reg *psals_reg)
{
    int ret;
    struct shdisp_bdic_i2c_msg i2c_msg;
    unsigned char i2c_wbuf[2];

    if ((shdisp_pm_API_is_ps_active() != SHDISP_DEV_STATE_ON) &&
        (shdisp_pm_API_is_als_active() != SHDISP_DEV_STATE_ON)) {
        SHDISP_ERR("<RESULT_FAILURE> ps&als is not active.");
        return SHDISP_RESULT_FAILURE;
    }

    if (psals_reg->reg < SENSOR_REG_COMMAND1 || psals_reg->reg > SENSOR_REG_D2_MSB) {
        SHDISP_ERR("<RESULT_FAILURE> Register address out of range.");
        return SHDISP_RESULT_FAILURE;
   }
    SHDISP_DEBUG("(Register (addr : 0x%02x, data : 0x%02x)", psals_reg->reg, psals_reg->val);
    i2c_wbuf[0] = psals_reg->reg;
    i2c_wbuf[1] = psals_reg->val;

    i2c_msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
    i2c_msg.mode = SHDISP_BDIC_I2C_M_W;
    i2c_msg.wlen = 2;
    i2c_msg.rlen = 0;
    i2c_msg.wbuf = &i2c_wbuf[0];
    i2c_msg.rbuf = NULL;
    ret = shdisp_bdic_API_i2c_transfer(&i2c_msg);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_get_ave_ado.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_ave_ado                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_ave_ado(struct shdisp_ave_ado *ave_ado)
{
    int ret;

    if (ave_ado->als_range >= NUM_SHDISP_MAIN_DISP_ALS_RANGE) {
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_bdic_API_get_ave_ado(ave_ado);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_get_ave_ado.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_lcd_det_recovery                                               */
/* ------------------------------------------------------------------------- */
static void shdisp_SQE_lcd_det_recovery(void)
{
#ifndef SHDISP_NOT_SUPPORT_DET
    int i;
    int retry_max = 3;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
    int err_code_reset;
    unsigned char subcode;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in");

    shdisp_bdic_API_IRQ_det_irq_ctrl(false);

#ifdef SHDISP_RESET_LOG
    subcode = shdisp_dbg_API_get_subcode();
#endif /* SHDISP_RESET_LOG */
    SHDISP_DEBUG("retry_max=%d", retry_max);
    for (i = 0; i < retry_max; i++) {

        shdisp_SQE_do_recovery();

        if (shdisp_SQE_check_recovery() == SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("recovery completed");
            shdisp_bdic_API_IRQ_det_irq_ctrl(true);

            /* enable REQOUT error interrupt for andy panel */
#if defined(CONFIG_SHDISP_PANEL_ANDY)
            shdisp_SYS_API_delay_us(20 * 1000);
#endif  /* CONFIG_SHDISP_PANEL_ANDY */

#if defined(CONFIG_SHDISP_PANEL_ARIA)
#ifdef SHDISP_DET_DSI_MIPI_ERROR
            /* reject the mipi error detecting contention */
            mdss_shdisp_dsi_mipi_err_clear();
#endif  /* SHDISP_DET_DSI_MIPI_ERROR */
#endif  /* CONFIG_SHDISP_PANEL_ARIA */

            /* enable the mipi error detection */
            shdisp_det_mipi_err_ctrl(true);

#ifdef SHDISP_RESET_LOG
            shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_NONE);
#endif /* SHDISP_RESET_LOG */
            return;
        }
        SHDISP_WARN("recovery retry(%d)", i);
    }

    SHDISP_ERR("recovery retry over");
#ifdef SHDISP_RESET_LOG
    err_code.mode = SHDISP_DBG_MODE_LINUX;
    err_code.type = SHDISP_DBG_TYPE_PANEL;
    err_code.code = SHDISP_DBG_CODE_RETRY_OVER;
    err_code.subcode = subcode;
    err_code_reset = 0;
 #if defined(CONFIG_ANDROID_ENGINEERING)
    if (shdisp_dbg_API_get_reset_flg() & SHDISP_DBG_RESET_PANEL_RETRY_OVER) {
        err_code_reset = 1;
    }
 #endif /* defined (CONFIG_ANDROID_ENGINEERING) */
    shdisp_dbg_API_err_output(&err_code, err_code_reset);
    shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_NONE);
#endif /* SHDISP_RESET_LOG */
#else   /* SHDISP_NOT_SUPPORT_DET */
    SHDISP_DEBUG("skip lcd det recovery");
#ifdef SHDISP_RESET_LOG
    shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_NONE);
#endif /* SHDISP_RESET_LOG */
#endif /* SHDISP_NOT_SUPPORT_DET */
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_psals_recovery                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_psals_recovery(void)
{
    int result = SHDISP_RESULT_SUCCESS;
    int ps_flg = 0;

    SHDISP_TRACE("in");

    ps_flg = shdisp_pm_API_is_ps_active();

    if (ps_flg == SHDISP_DEV_STATE_ON) {
        /* notify to proximity module that recovery is staring */
        PROX_recovery_start_func();
    }

    shdisp_semaphore_start();

    shdisp_pm_API_psals_error_power_off();

    shdisp_SYS_API_delay_us(10 * 1000);
    shdisp_bdic_API_IRQ_i2c_error_Clear();

    shdisp_pm_API_psals_error_power_recovery();

    down(&shdisp_sem_req_recovery_psals);
    shdisp_recovery_psals_queued_flag = 0;
    up(&shdisp_sem_req_recovery_psals);

    result = shdisp_bdic_API_psals_is_recovery_successful();
    if (result != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("PALS Recovery Error!!");
    }

    shdisp_semaphore_end(__func__);

    if (ps_flg == SHDISP_DEV_STATE_ON) {

        /* notify to proximity module that recovery is ending */
        msleep(20);
        PROX_recovery_end_func();
    }

    SHDISP_DEBUG("main_disp_status=%d", shdisp_kerl_ctx.main_disp_status)
    if ((result == SHDISP_RESULT_SUCCESS) ||
            (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON)) {
        SHDISP_DEBUG("enable_irq for psals_recovery");
        shdisp_SYS_API_set_irq(SHDISP_IRQ_ENABLE);
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_irq_mask                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_irq_mask(int irq_msk_ctl)
{
    SHDISP_TRACE("in");

    if (irq_msk_ctl == SHDISP_IRQ_NO_MASK) {
        shdisp_det_mipi_err_ctrl(true);
        shdisp_lcd_det_recovery_subscribe();
    } else {
        shdisp_det_mipi_err_ctrl(false);
        shdisp_lcd_det_recovery_unsubscribe();
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_vcom_tracking                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_vcom_tracking(int tracking)
{
    int ret;

    SHDISP_TRACE("in");

#if defined(CONFIG_SHDISP_PANEL_ANDY)
    ret = shdisp_andy_API_vcom_tracking(tracking);
#else /* CONFIG_SHDISP_PANEL_ANDY */
    ret = SHDISP_RESULT_SUCCESS;
#endif /* CONFIG_SHDISP_PANEL_ANDY */

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_semaphore_start                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_semaphore_start(void)
{
    SHDISP_INFO("in");
    down(&shdisp_sem);

#ifdef SHDISP_SYS_SW_TIME_API
    shdisp_sys_API_dbg_hw_check_start();
#endif  /* SHDISP_SYS_SW_TIME_API */
    SHDISP_INFO("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_semaphore_end                                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_semaphore_end(const char *func)
{
    SHDISP_INFO("in");
#ifdef SHDISP_SYS_SW_TIME_API
    shdisp_sys_API_dbg_hw_check_end(func);
#endif  /* SHDISP_SYS_SW_TIME_API */

    up(&shdisp_sem);
    SHDISP_INFO("out");
}

/* ------------------------------------------------------------------------- */
/* INTERRUPT                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_gpio_int_isr                                                       */
/* ------------------------------------------------------------------------- */
static irqreturn_t shdisp_gpio_int_isr( int irq_num, void *data )
{
    irqreturn_t rc = IRQ_HANDLED;
    int ret;
    unsigned long flags = 0;

    shdisp_SYS_API_set_irq(SHDISP_IRQ_DISABLE);

    spin_lock_irqsave(&shdisp_q_lock, flags);

    SHDISP_TRACE(":Start");

    if (shdisp_wq_gpio) {
        shdisp_wake_lock();
        ret = queue_work(shdisp_wq_gpio, &shdisp_wq_gpio_wk);
        if (ret == 0) {
            shdisp_wake_unlock();
            SHDISP_ERR("<QUEUE_WORK_FAILURE>");
        }
    }
    spin_unlock_irqrestore(&shdisp_q_lock, flags);

    return rc;
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_gpio                                             */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_handler_gpio(struct work_struct *work)
{
    struct shdisp_queue_data_t *qdata = NULL;
    int    i;
    int    bFirstQue = 0;
    int    ret;
    int    nBDIC_QueFac = 0;

    SHDISP_TRACE("Start");

    shdisp_semaphore_start();
    shdisp_bdic_API_IRQ_save_fac();

    do {
        ret = shdisp_bdic_API_IRQ_check_fac();
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("no factory");
            break;
        }

        down(&shdisp_sem_irq_fac);
        for (i = 0; i < SHDISP_IRQ_MAX_KIND; i++) {
            nBDIC_QueFac = shdisp_bdic_API_IRQ_get_fac(i);
            if (nBDIC_QueFac == SHDISP_BDIC_IRQ_TYPE_NONE) {
                break;
            }

            if (shdisp_wq_gpio_task) {
                qdata = kmalloc( sizeof(shdisp_queue_data), GFP_KERNEL );
                if (qdata != NULL) {
                    qdata->irq_GFAC = nBDIC_QueFac;
                    list_add_tail(&qdata->list, &shdisp_queue_data.list);
                    if (bFirstQue == 0) {
                        bFirstQue = 1;
                        shdisp_wake_lock();
                        ret = queue_work(shdisp_wq_gpio_task, &shdisp_wq_gpio_task_wk);
                        if (ret == 0) {
                            shdisp_wake_unlock();
                            SHDISP_DEBUG("<QUEUE_WORK_FAILURE> queue_work failed");
                        }
                    }
                } else {
                   SHDISP_ERR("<QUEUE_WORK_FAILURE> kmalloc failed (BDIC_QueFac=%d)", nBDIC_QueFac);
                }
            }
        }
        up(&shdisp_sem_irq_fac);

    } while (0);

    shdisp_bdic_API_IRQ_Clear();
    shdisp_semaphore_end(__func__);

    if (shdisp_bdic_API_IRQ_check_DET() != SHDISP_BDIC_IRQ_TYPE_DET &&
        shdisp_bdic_API_IRQ_check_I2C_ERR() != SHDISP_BDIC_IRQ_TYPE_I2C_ERR) {
        SHDISP_DEBUG("enable_irq for \"No DET\" or \"No I2C_ERR\"");
        shdisp_SYS_API_set_irq(SHDISP_IRQ_ENABLE);
    }
    SHDISP_TRACE("Finish");
    shdisp_wake_unlock();
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_gpio_task                                                */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_gpio_task(struct work_struct *work)
{
    struct list_head *listptr;
    struct shdisp_queue_data_t  *entry;
    struct shdisp_queue_data_t  *entryFirst = NULL;
    int     nFirstBDIC_GFAC = 0;
    int     nFirst_GFAC = -1;
    int     bFirst = 0;
    int     bThrough = 0;
    int     ret;
    void (*temp_callback)(void);
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("Start");

    do {
        down(&shdisp_sem_irq_fac);
        bThrough = 0;
        entryFirst = NULL;
        bFirst = 0;
        nFirstBDIC_GFAC = 0;
        list_for_each(listptr, &shdisp_queue_data.list) {
            entry = list_entry( listptr, struct shdisp_queue_data_t, list);
            if (bFirst == 0) {
                entryFirst = entry;
                nFirstBDIC_GFAC = entry->irq_GFAC;
                bFirst = 1;
            } else {
                if (entry->irq_GFAC == nFirstBDIC_GFAC) {
                    bThrough = 1;
                }
            }
        }

        if (entryFirst != NULL) {
            list_del( &entryFirst->list );
            kfree( entryFirst );
        } else {
            SHDISP_DEBUG("no entry");
            up(&shdisp_sem_irq_fac);
            break;
        }
        up(&shdisp_sem_irq_fac);


        if (bThrough == 0) {
            if (nFirstBDIC_GFAC == SHDISP_BDIC_IRQ_TYPE_NONE) {
                SHDISP_DEBUG("failed (no BDIC_GFAC=%d)", nFirstBDIC_GFAC);
            } else {
                nFirst_GFAC = -1;
                switch (nFirstBDIC_GFAC) {
                case SHDISP_BDIC_IRQ_TYPE_ALS:
                        nFirst_GFAC = SHDISP_IRQ_TYPE_ALS;
                        break;
                case SHDISP_BDIC_IRQ_TYPE_PS:
                        nFirst_GFAC = SHDISP_IRQ_TYPE_PS;
                        break;
                default:
                        break;
                }

                SHDISP_DEBUG("Callback[%d] Start", nFirstBDIC_GFAC);
                if (nFirst_GFAC >= 0) {
                    down(&shdisp_sem_callback);
                    temp_callback = shdisp_callback_table[nFirst_GFAC];
                    up(&shdisp_sem_callback);

                    if (temp_callback != NULL) {
                        (*temp_callback)();
                    } else {
                        SHDISP_DEBUG("Callback is Nulle pointer(irq_type=%d)", nFirst_GFAC);
                    }
                } else if (nFirstBDIC_GFAC == SHDISP_BDIC_IRQ_TYPE_DET) {
                    SHDISP_DEBUG("enable_irq for DET before");
                    shdisp_semaphore_start();
                    ret = shdisp_bdic_API_RECOVERY_check_restoration();
                    shdisp_semaphore_end(__func__);
                    if (ret == SHDISP_RESULT_FAILURE) {
                        SHDISP_ERR("lcd det bdic");
#ifdef SHDISP_RESET_LOG
                        err_code.mode = SHDISP_DBG_MODE_LINUX;
                        err_code.type = SHDISP_DBG_TYPE_PANEL;
                        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
                        err_code.subcode = SHDISP_DBG_SUBCODE_ESD_DETIN;
                        shdisp_dbg_API_err_output(&err_code, 0);
                        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_ESD_DETIN);
#endif /* SHDISP_RESET_LOG */
                        shdisp_do_lcd_det_recovery();
                    } else {
                        SHDISP_DEBUG("lcd det bdic detects the false");
                        shdisp_bdic_API_IRQ_det_irq_ctrl(true);
                    }
                    SHDISP_DEBUG("enable_irq for DET after");
                    shdisp_SYS_API_set_irq(SHDISP_IRQ_ENABLE);
                } else if (nFirstBDIC_GFAC == SHDISP_BDIC_IRQ_TYPE_I2C_ERR) {
                    shdisp_do_psals_recovery();
                }
            }
        } else {
            SHDISP_DEBUG("Skip (BDIC_GFAC=%d)", nFirstBDIC_GFAC);
        }
    } while (1);


    SHDISP_TRACE("Finish");
    shdisp_wake_unlock();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_wake_lock_init                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_wake_lock_init(void)
{
    unsigned long flags = 0;

    spin_lock_irqsave(&shdisp_wake_spinlock, flags);
    shdisp_wake_lock_wq_refcnt = 0;
    wake_lock_init(&shdisp_wake_lock_wq, WAKE_LOCK_SUSPEND, "shdisp_wake_lock_wq");
    spin_unlock_irqrestore(&shdisp_wake_spinlock, flags);
}

/* ------------------------------------------------------------------------- */
/* shdisp_wake_lock                                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_wake_lock(void)
{
    unsigned long flags = 0;

    spin_lock_irqsave(&shdisp_wake_spinlock, flags);
    if (shdisp_wake_lock_wq_refcnt++ == 0) {
        wake_lock(&shdisp_wake_lock_wq);
    }
    spin_unlock_irqrestore(&shdisp_wake_spinlock, flags);
}

/* ------------------------------------------------------------------------- */
/* shdisp_wake_unlock                                                        */
/* ------------------------------------------------------------------------- */
static void shdisp_wake_unlock(void)
{
    unsigned long flags = 0;

    spin_lock_irqsave(&shdisp_wake_spinlock, flags);
    if (--shdisp_wake_lock_wq_refcnt <= 0) {
        wake_unlock(&shdisp_wake_lock_wq);
        shdisp_wake_lock_wq_refcnt = 0;
    }
    spin_unlock_irqrestore(&shdisp_wake_spinlock, flags);
}

/* ------------------------------------------------------------------------- */
/* shdisp_timer_int_isr                                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_timer_int_isr(unsigned long data)
{
    int ret;

    SHDISP_DEBUG("Timeout ( registered %ld, now %ld ).", data, jiffies);
    SHDISP_TRACE(":Start");

    if (shdisp_timer_stop) {
        SHDISP_DEBUG("Timer is not to be restarted.");
        return;
    }
    if (shdisp_wq_timer_task) {
        ret = queue_work(shdisp_wq_timer_task, &shdisp_wq_timer_task_wk);
        if (ret == 0) {
            SHDISP_ERR("<QUEUE_WORK_FAILURE> ");
        }
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_timer_int_register                                                 */
/* ------------------------------------------------------------------------- */
static void shdisp_timer_int_register(void)
{
    down(&shdisp_sem_timer);

    shdisp_timer.expires  = jiffies + (10 * HZ);
    shdisp_timer.data     = (unsigned long)jiffies;
    shdisp_timer.function = shdisp_timer_int_isr;

    if (!shdisp_timer_stop) {
        up(&shdisp_sem_timer);
        return;
    }

    add_timer(&shdisp_timer);
    shdisp_timer_stop = 0;

    up(&shdisp_sem_timer);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_timer_int_delete                                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_timer_int_delete(void)
{
    down(&shdisp_sem_timer);

    del_timer_sync(&shdisp_timer);
    shdisp_timer_stop = 1;

    up(&shdisp_sem_timer);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_timer_int_mod                                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_timer_int_mod(void)
{
    down(&shdisp_sem_timer);

    if (shdisp_timer_stop) {
        up(&shdisp_sem_timer);
        return;
    }

    mod_timer(&shdisp_timer, (unsigned long)(jiffies + (10 * HZ)));
    shdisp_timer_stop = 0;

    up(&shdisp_sem_timer);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_timer_task                                               */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_timer_task(struct work_struct *work)
{
    int     ret = 0;
    int     nFirst_GFAC = -1;
    void    (*temp_callback)(void);

    nFirst_GFAC = SHDISP_IRQ_TYPE_DET;

    shdisp_semaphore_start();
    ret = shdisp_panel_API_check_recovery();
    shdisp_semaphore_end(__func__);
    if (ret == SHDISP_RESULT_SUCCESS) {
        shdisp_timer_int_mod();
        return;
    }
    SHDISP_DEBUG("A recovery processing is required.");

    down(&shdisp_sem_callback);
    temp_callback = shdisp_callback_table[nFirst_GFAC];
    up(&shdisp_sem_callback);

    if (temp_callback != NULL) {
        (*temp_callback)();
    } else {
        SHDISP_DEBUG(" Callback is Nulle pointer(irq_type=%d)", nFirst_GFAC);
    }

    shdisp_timer_int_mod();

    return;
}

#ifdef SHDISP_TRI_LED2
/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_pierce                                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_pierce(struct work_struct *work)
{
    int ret;
    struct shdisp_tri_led led;

    SHDISP_TRACE("in");

    shdisp_semaphore_start();
    if (shdisp_pierce_status.insert == SHDISP_PIERCE_INS_MODE_ON) {
        ret = shdisp_SQE_tri_led_set_color2(&shdisp_pierce_status.led2);
    } else {
        led.red = 0;
        led.green = 0;
        led.blue = 0;
        led.ext_mode = shdisp_pierce_status.led2.ext_mode;
        led.led_mode = shdisp_pierce_status.led2.led_mode;
        led.ontime   = shdisp_pierce_status.led2.ontime;
        led.interval = shdisp_pierce_status.led2.interval;
        led.count    = shdisp_pierce_status.led2.count;
        ret = shdisp_SQE_tri_led_set_color2(&led);
    }
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_tri_led_set_color2.");
    }

    SHDISP_TRACE("out");
    return;
}
#endif  /* SHDISP_TRI_LED2 */

/* ------------------------------------------------------------------------- */
/* shdisp_do_lcd_det_recovery                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_do_lcd_det_recovery(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int queued = 0;

    SHDISP_TRACE("in");

    if (!shdisp_wq_recovery) {
        SHDISP_ERR(" workqueue nothing.");
        return SHDISP_RESULT_FAILURE;
    }
    shdisp_wake_lock();

    down(&shdisp_sem_req_recovery_lcd);

    if (!shdisp_recovery_lcd_queued_flag) {
        shdisp_recovery_lcd_queued_flag = 1;

        ret = queue_work(shdisp_wq_recovery, &shdisp_wq_recovery_lcd_wk);

        if (ret == 0) {
            shdisp_recovery_lcd_queued_flag = 0;
            SHDISP_ERR("<QUEUE_WORK_FAILURE> .");
            ret = SHDISP_RESULT_FAILURE;
        } else {
            queued = 1;
            ret = SHDISP_RESULT_SUCCESS;
        }
    } else {
        SHDISP_DEBUG("queued. now recovering... ");
        ret = SHDISP_RESULT_SUCCESS;
    }

    up(&shdisp_sem_req_recovery_lcd);

    if (!queued) {
        shdisp_wake_unlock();
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_recovery_lcd                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_handler_recovery_lcd(struct work_struct *work)
{
    void (*temp_callback)(void);

    SHDISP_TRACE("in");

    down(&shdisp_sem_callback);
    temp_callback = shdisp_callback_table[SHDISP_IRQ_TYPE_DET];
    up(&shdisp_sem_callback);

    if (temp_callback != NULL) {
        (*temp_callback)();
    }

    down(&shdisp_sem_req_recovery_lcd);
    shdisp_recovery_lcd_queued_flag = 0;
    up(&shdisp_sem_req_recovery_lcd);

    shdisp_wake_unlock();

    SHDISP_TRACE("out");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_lcd_det_recovery                                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_lcd_det_recovery(void)
{
    SHDISP_TRACE("in");

#ifdef CONFIG_TOUCHSCREEN_SHTPS
    SHDISP_DEBUG("msm_tps_setsleep off");
    msm_tps_setsleep(1);
#endif /* CONFIG_TOUCHSCREEN_SHTPS */

#ifndef SHDISP_NOT_SUPPORT_DET
    mdss_shdisp_lock_recovery();
#endif /* SHDISP_NOT_SUPPORT_DET */

    shdisp_semaphore_start();

    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_OFF) {
        shdisp_semaphore_end(__func__);
#ifndef SHDISP_NOT_SUPPORT_DET
        mdss_shdisp_unlock_recovery();
#endif /* SHDISP_NOT_SUPPORT_DET */
        SHDISP_WARN("out1");
        return;
    }

#if defined(CONFIG_SHDISP_PANEL_ANDY)
    mdss_shdisp_mdp_hr_video_suspend();
#endif /* CONFIG_SHDISP_PANEL_ANDY */
    shdisp_SQE_lcd_det_recovery();
#if defined(CONFIG_SHDISP_PANEL_ANDY)
    mdss_shdisp_mdp_hr_video_resume();
#endif /* CONFIG_SHDISP_PANEL_ANDY */

    shdisp_semaphore_end(__func__);

#ifndef SHDISP_NOT_SUPPORT_DET
    mdss_shdisp_unlock_recovery();
#endif /* SHDISP_NOT_SUPPORT_DET */

#ifdef CONFIG_TOUCHSCREEN_SHTPS
    SHDISP_DEBUG("msm_tps_setsleep on");
    msm_tps_setsleep(0);
#endif /* CONFIG_TOUCHSCREEN_SHTPS */

    SHDISP_TRACE("out");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_lcd_det_recovery_subscribe                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_lcd_det_recovery_subscribe(void)
{
    int ret = 0;
    struct shdisp_subscribe lcd_subs;

    SHDISP_TRACE("in");

    lcd_subs.irq_type = SHDISP_IRQ_TYPE_DET;
    lcd_subs.callback = shdisp_lcd_det_recovery;
    shdisp_event_subscribe(&lcd_subs);

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_lcd_det_recovery_unsubscribe                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_lcd_det_recovery_unsubscribe(void)
{
    int ret = 0;

    SHDISP_TRACE("in");

    shdisp_event_unsubscribe(SHDISP_IRQ_TYPE_DET);

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_do_psals_recovery                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_do_psals_recovery(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    int queued = 0;
    SHDISP_TRACE("in");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    if (!shdisp_wq_recovery) {
        SHDISP_ERR(" workqueue nothing.");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_wake_lock();

    down(&shdisp_sem_req_recovery_psals);

    if (!shdisp_recovery_psals_queued_flag) {
        shdisp_recovery_psals_queued_flag = 1;
        ret = queue_work(shdisp_wq_recovery, &shdisp_wq_recovery_psals_wk);

        if (ret == 0) {
            shdisp_recovery_psals_queued_flag = 0;
            SHDISP_ERR("<QUEUE_WORK_FAILURE> .");
            ret = SHDISP_RESULT_FAILURE;
        } else {
            queued = 1;
            ret = SHDISP_RESULT_SUCCESS;
        }
    } else {
        SHDISP_DEBUG("queued. now recovering... ");
        ret = SHDISP_RESULT_SUCCESS;
    }
    up(&shdisp_sem_req_recovery_psals);

    if (!queued) {
        shdisp_wake_unlock();
    }

    SHDISP_TRACE("out");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_recovery_psals                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_handler_recovery_psals(struct work_struct *work)
{
    void (*temp_callback)(void);

    SHDISP_TRACE("in");

    down(&shdisp_sem_callback);
    temp_callback = shdisp_callback_table[SHDISP_IRQ_TYPE_I2CERR];
    up(&shdisp_sem_callback);

    if (temp_callback != NULL) {
        (*temp_callback)();
    }

    shdisp_wake_unlock();

    SHDISP_TRACE("End");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_psals_recovery                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_psals_recovery(void)
{
    SHDISP_TRACE("in");

    shdisp_SQE_psals_recovery();

    SHDISP_TRACE("out");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_event_subscribe                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_event_subscribe(struct shdisp_subscribe *subscribe)
{
    int ret;

    SHDISP_PERFORMANCE("RESUME PANEL EVENT-SUBSCRIBE 0010 START");


    SHDISP_TRACE(":Start(irq_type=%d)", subscribe->irq_type);

    if (shdisp_bdic_subscribe_check(subscribe) != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    if (subscribe->irq_type == SHDISP_IRQ_TYPE_DET) {
        if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
            return SHDISP_RESULT_SUCCESS;
        }
    }

    if ((subscribe->irq_type == SHDISP_IRQ_TYPE_ALS) &&
        (shdisp_subscribe_type_table[subscribe->irq_type] == SHDISP_SUBSCRIBE_TYPE_INT)) {
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_SQE_event_subscribe(subscribe);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_event_subscribe.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME PANEL EVENT-SUBSCRIBE 0010 END");

    return SHDISP_RESULT_SUCCESS;

}

/* ------------------------------------------------------------------------- */
/* shdisp_event_unsubscribe                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_event_unsubscribe(int irq_type)
{
    int ret;

    SHDISP_PERFORMANCE("RESUME PANEL EVENT-UNSUBSCRIBE 0010 START");

    if (shdisp_bdic_unsubscribe_check(irq_type) != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    if ((irq_type == SHDISP_IRQ_TYPE_ALS) && (shdisp_subscribe_type_table[irq_type] == SHDISP_SUBSCRIBE_TYPE_INT)) {
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_SQE_event_unsubscribe(irq_type);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_event_unsubscribe.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME PANEL EVENT-UNSUBSCRIBE 0010 END");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* OTHER                                                                     */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_proc_write                                                         */
/* ------------------------------------------------------------------------- */
#define PROC_BUF_LENGTH                 (4096)
#define PROC_BUF_REWIND_LENGTH          (4000)
#define SHDISP_DEBUG_CONSOLE(fmt, args...) \
        do { \
            int buflen = 0; \
            int remain = PROC_BUF_LENGTH - proc_buf_pos - 1; \
            if (remain > 0) { \
                buflen = snprintf(&proc_buf[proc_buf_pos], remain, fmt, ## args); \
                proc_buf_pos += (buflen > 0) ? buflen : 0; \
            } \
        } while (0)
static unsigned char proc_buf[PROC_BUF_LENGTH] = {0};
static unsigned int  proc_buf_pos = 0;

static int shdisp_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
#define SHDISP_LEN_ID                   (2)
#define SHDISP_LEN_PARAM                (4)
#define SHDISP_PARAM_MAX                (4)

    unsigned long len = count;
    struct shdisp_procfs shdisp_pfs;
    char buf[SHDISP_LEN_PARAM + 1];
    char kbuf[SHDISP_LEN_ID + SHDISP_PARAM_MAX * SHDISP_LEN_PARAM];
    int i;
    int ret = 0;
    struct shdisp_bdic_i2c_msg i2c_msg;
    unsigned char i2c_wbuf[6];
    unsigned char i2c_rbuf[6];
    struct shdisp_prox_params prox_params;
    struct shdisp_main_bkl_ctl bkl;
    unsigned char   val;

    char *kbufex;
    unsigned char *param = NULL;
    int paramlen = 0;
    int needalloc = 0;
    struct shdisp_tri_led tri_led;
    struct shdisp_lcddr_reg panel_reg;
    int recovery_error_flag;
    int recovery_error_count;

    len--;
    /* Check length */
    if (len < SHDISP_LEN_ID) {
        return count;
    }
    if (len > (SHDISP_LEN_ID + SHDISP_PARAM_MAX * SHDISP_LEN_PARAM)) {
        len = SHDISP_LEN_ID + SHDISP_PARAM_MAX * SHDISP_LEN_PARAM;
        needalloc = 1;
    }

    if (copy_from_user(kbuf, buffer, len)) {
        return -EFAULT;
    }
    /* Get FunctionID */
    memcpy(buf, kbuf, SHDISP_LEN_ID);
    buf[SHDISP_LEN_ID] = '\0';
    shdisp_pfs.id = simple_strtol(buf, NULL, 10);
    shdisp_pfs.par[0] = 0;
    shdisp_pfs.par[1] = 0;
    shdisp_pfs.par[2] = 0;
    shdisp_pfs.par[3] = 0;

    /* Get Parameters */
    for (i = 0; (i + 1) * SHDISP_LEN_PARAM <= (len - SHDISP_LEN_ID); i++) {
        memcpy(buf, &(kbuf[SHDISP_LEN_ID + i * SHDISP_LEN_PARAM]), SHDISP_LEN_PARAM);
        buf[SHDISP_LEN_PARAM] = '\0';
        shdisp_pfs.par[i] = simple_strtol(buf, NULL, 16);
    }

    printk("[SHDISP] shdisp_proc_write(%d, 0x%04x, 0x%04x, 0x%04x, 0x%04x)\n", shdisp_pfs.id,
                                                                               shdisp_pfs.par[0], shdisp_pfs.par[1],
                                                                               shdisp_pfs.par[2], shdisp_pfs.par[3]);

    switch (shdisp_pfs.id) {
    case SHDISP_DEBUG_DSI_WRITE:
        if (len < 6) {
            SHDISP_ERR("(%d): DSI_WRITE param error", shdisp_pfs.id);
            goto out;
        }
        needalloc = 1;
        break;
    }

    if (needalloc) {
        len = count - (SHDISP_LEN_ID + 1);
        if (len > (1024 * SHDISP_PARAM_MAX) - (SHDISP_LEN_ID + 1)) {
           len = (1024 * SHDISP_PARAM_MAX) - (SHDISP_LEN_ID + 1);
        }
        kbufex = kmalloc(len, GFP_KERNEL);
        if (!kbufex) {
            return -EFAULT;
        }
        buffer += SHDISP_LEN_ID;
        if (copy_from_user(kbufex, buffer, len)) {
            kfree(kbufex);
            return -EFAULT;
        }
        paramlen = len / (SHDISP_LEN_PARAM / 2);
        param = kmalloc(paramlen, GFP_KERNEL);
        if (!param) {
            kfree(kbufex);
            return -EFAULT;
        }
        /* Get Parameters */
        for (i = 0; i < paramlen; i++) {
            memcpy(buf, &(kbufex[i * (SHDISP_LEN_PARAM / 2)]), (SHDISP_LEN_PARAM / 2));
            buf[(SHDISP_LEN_PARAM / 2)] = '\0';
            param[i] = simple_strtol(buf, NULL, 16);
        }
        kfree(kbufex);
    }

    switch (shdisp_pfs.id) {
    case SHDISP_DEBUG_PROCESS_STATE_OUTPUT:
        shdisp_semaphore_start();
        shdisp_dbg_info_output((int)shdisp_pfs.par[0]);
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_TRACE_LOG_SWITCH:
        if (shdisp_pfs.par[0] == 0) {
            printk("[SHDISP] Trace log OFF\n");
        } else {
            printk("[SHDISP] Trace log ON(%d)\n", shdisp_pfs.par[0]);
        }
        SHDISP_SET_LOG_LV((unsigned char)shdisp_pfs.par[0]);
        SHDISP_DEBUG("TraceLog enable check!!");
        break;

    case SHDISP_DEBUG_BDIC_I2C_WRITE:
        printk("[SHDISP] BDIC-I2C WRITE (addr : 0x%02x, data : 0x%02x)\n", shdisp_pfs.par[0], shdisp_pfs.par[1]);
        i2c_wbuf[0] = shdisp_pfs.par[0];
        i2c_wbuf[1] = shdisp_pfs.par[1];

        i2c_msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
        i2c_msg.mode = SHDISP_BDIC_I2C_M_W;
        i2c_msg.wlen = 2;
        i2c_msg.rlen = 0;
        i2c_msg.wbuf = &i2c_wbuf[0];
        i2c_msg.rbuf = NULL;
        shdisp_semaphore_start();
        ret = shdisp_SQE_write_bdic_i2c(&i2c_msg);
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_BDIC_I2C_READ:
        printk("[SHDISP] BDIC-I2C READ (addr : 0x%02x)\n", shdisp_pfs.par[0]);
        i2c_wbuf[0] = shdisp_pfs.par[0];
        i2c_rbuf[0] = 0x00;

        i2c_msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
        i2c_msg.mode = SHDISP_BDIC_I2C_M_R;
        i2c_msg.wlen = 1;
        i2c_msg.rlen = 1;
        i2c_msg.wbuf = &i2c_wbuf[0];
        i2c_msg.rbuf = &i2c_rbuf[0];
        shdisp_semaphore_start();
        ret = shdisp_SQE_read_bdic_i2c(&i2c_msg);
        shdisp_semaphore_end(__func__);
        printk("[SHDISP] Read data(0x%02x)\n", i2c_rbuf[0]);
        SHDISP_DEBUG_CONSOLE("<COMMAND = I2C_READ>");
        SHDISP_DEBUG_CONSOLE("  IN     = 0x%02x", i2c_wbuf[0]);
        SHDISP_DEBUG_CONSOLE("  OUT    = 0x%02x", i2c_rbuf[0]);
        if (ret == SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG_CONSOLE("  RESULT = OK");
        } else {
            SHDISP_DEBUG_CONSOLE("  RESULT = NG");
        }
        break;

    case SHDISP_DEBUG_PROX_SENSOR_CTL:
        switch (shdisp_pfs.par[0]) {
        case 0:
            ret = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_OFF, NULL);
            break;
        case 1:
            printk("[SHDISP] POWER_ON_PARAM (LOW : %d, HIGH : %d)\n", shdisp_pfs.par[1], shdisp_pfs.par[2]);
            prox_params.threshold_low  = (unsigned int)shdisp_pfs.par[1];
            prox_params.threshold_high = (unsigned int)shdisp_pfs.par[2];
            ret = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_ON, &prox_params);
            break;
        default:
            break;
        }
        break;

    case SHDISP_DEBUG_BKL_CTL:
        if (shdisp_pfs.par[0] == 0) {
            printk("[SHDISP] BKL_OFF\n");
            ret = shdisp_api_main_bkl_off();
        } else {
            printk("[SHDISP] BKL_ON (mode : %d, param : %d)\n", shdisp_pfs.par[1], shdisp_pfs.par[2]);
            bkl.mode  = shdisp_pfs.par[1];
            bkl.param = shdisp_pfs.par[2];
            if (bkl.mode == SHDISP_MAIN_BKL_MODE_AUTO) {
                shdisp_semaphore_start();
                ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO, &bkl);
                shdisp_semaphore_end(__func__);
            } else {
                ret = shdisp_api_main_bkl_on(&bkl);
            }
        }
        break;

    case SHDISP_DEBUG_LIGHT_SENSOR_CTL:
        shdisp_semaphore_start();
        switch (shdisp_pfs.par[0]) {
        case 0:
            shdisp_pm_API_als_user_manager(
                    SHDISP_DEV_TYPE_ALS_MASK & (unsigned long)shdisp_pfs.par[1],
                    SHDISP_DEV_REQ_OFF);
            break;
        case 1:
            shdisp_pm_API_als_user_manager(
                    SHDISP_DEV_TYPE_ALS_MASK & (unsigned long)shdisp_pfs.par[1],
                    SHDISP_DEV_REQ_ON);
            break;
        default:
            break;
        }
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_IRQ_LOGIC_CHK:
        i = shdisp_pfs.par[0];
        printk("[SHDISP] shdisp_proc_write(%d):Test Pattern=%d\n", shdisp_pfs.id, i);
        shdisp_dbg_que(i);
        break;

    case SHDISP_DEBUG_BDIC_IRQ_ALL_CLEAR:
        printk("[SHDISP] shdisp_proc_write(%d):Interrupt Clear All\n", shdisp_pfs.id);
        shdisp_bdic_API_IRQ_dbg_Clear_All();
        break;

    case SHDISP_DEBUG_BDIC_IRQ_CLEAR:
        printk("[SHDISP] shdisp_proc_write(%d):Interrupt Clear\n", shdisp_pfs.id);
        shdisp_bdic_API_IRQ_Clear();
        break;

    case SHDISP_DEBUG_DUMMY_SUBSCRIBE:
        printk("[SHDISP] shdisp_proc_write(%d):dummy subscribe\n", shdisp_pfs.id);
        shdisp_semaphore_start();
        shdisp_debug_subscribe();
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_DUMMY_UNSUBSCRIBE_PS:
        printk("[SHDISP] shdisp_proc_write(%d):dummy unsubscribe\n", shdisp_pfs.id);
        shdisp_event_unsubscribe(SHDISP_IRQ_TYPE_PS);
        break;

    case SHDISP_DEBUG_BDIC_WRITE:
        printk("[SHDISP] shdisp_proc_write(%d): BDIC register write\n", shdisp_pfs.id);
        val = shdisp_pfs.par[0] & 0x00FF;
        printk("[SHDISP] shdisp_SQE_bdic_write_reg() reg=0x%02x val=0x%02x\n", ((shdisp_pfs.par[0] >> 8) & 0x00FF),
                                                                               val);
        shdisp_SQE_bdic_write_reg(((shdisp_pfs.par[0] >> 8) & 0x00FF), val);
        break;

    case SHDISP_DEBUG_BDIC_READ:
        printk("[SHDISP] shdisp_proc_write(%d): BDIC register read\n", shdisp_pfs.id);
        val = 0;
        printk("[SHDISP] shdisp_SQE_bdic_read_reg() reg=0x%02x\n", ((shdisp_pfs.par[0] >> 8) & 0x00FF));
        ret = shdisp_SQE_bdic_read_reg(((shdisp_pfs.par[0] >> 8) & 0x00FF), &val);
        printk("[SHDISP] value=0x%02x\n", val);
        SHDISP_DEBUG_CONSOLE("<COMMAND = BDIC_register_READ>");
        SHDISP_DEBUG_CONSOLE("  IN     = 0x%02x", ((shdisp_pfs.par[0] >> 8) & 0x00FF));
        SHDISP_DEBUG_CONSOLE("  OUT    = 0x%02x", val);
        if (ret == SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG_CONSOLE("  RESULT = OK");
        } else {
            SHDISP_DEBUG_CONSOLE("  RESULT = NG");
        }
        break;

    case SHDISP_DEBUG_RGB_LED:
        tri_led.red      = ((shdisp_pfs.par[0] >> 8) & 0x00FF);
        tri_led.green    = ( shdisp_pfs.par[0]       & 0x00FF);
        tri_led.blue     = ((shdisp_pfs.par[1] >> 8) & 0x00FF);
        tri_led.ext_mode = ( shdisp_pfs.par[1]       & 0x00FF);
        tri_led.led_mode = ((shdisp_pfs.par[2] >> 8) & 0x00FF);
        tri_led.ontime   = ( shdisp_pfs.par[2]       & 0x00FF);
        tri_led.interval = ((shdisp_pfs.par[3] >> 8) & 0x00FF);
        tri_led.count    = ( shdisp_pfs.par[3]       & 0x00FF);
        ret = shdisp_SQE_tri_led_set_color(&tri_led);
        break;

    case SHDISP_DEBUG_LED_REG_DUMP:
        if (shdisp_pfs.par[0] == 1) {
            shdisp_bdic_API_TRI_LED2_INFO_output();
        }else {
            shdisp_bdic_API_TRI_LED_INFO_output();
        }
        break;

    case SHDISP_DEBUG_BDIC_RESTART:
        shdisp_event_unsubscribe(SHDISP_IRQ_TYPE_I2CERR);
        shdisp_event_unsubscribe(SHDISP_IRQ_TYPE_DET);
        shdisp_pm_API_bdic_shutdown();
        shdisp_pm_API_bdic_resume();

        if (shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.status == SHDISP_ALS_SENSOR_ADJUST_STATUS_COMPLETED) {
            shdisp_bdic_API_als_sensor_adjust(&(shdisp_kerl_ctx.boot_ctx.photo_sensor_adj));
        }

        break;

#ifdef SHDISP_TRI_LED2
    case SHDISP_DEBUG_PIERCE_INOUT:
        if (shdisp_pfs.par[0] == 0) {
            printk("[SHDISP] Smart phone pierce Removed.\n");
            ret = shdisp_api_remove_sp_pierce();
        } else if(shdisp_pfs.par[0] == 1) {
            printk("[SHDISP] Smart phone pierce Inserted.\n");
            ret = shdisp_api_insert_sp_pierce();
        }
        break;
#endif  /* SHDISP_TRI_LED2 */

    case SHDISP_DEBUG_MIPI_TX_FREQ_CHG:

        break;

    case SHDISP_DEBUG_FPS_LED:
        if (shdisp_pfs.par[0]) {
            mdss_shdisp_fps_led_start();
        } else {
            mdss_shdisp_fps_led_stop();
        }
        break;

    case SHDISP_DEBUG_DISPLAYLOG_ERROR_LOG_TEST:
    {
        struct shdisp_dbg_error_code    code;

        code.mode      = (unsigned char)shdisp_pfs.par[0];
        code.type      = (unsigned char)shdisp_pfs.par[1];
        code.code      = (unsigned char)shdisp_pfs.par[2];
        code.subcode   = (unsigned char)shdisp_pfs.par[3];
        shdisp_dbg_API_add_err_log(&code);
        break;
    }
    case SHDISP_DEBUG_DISPLAYLOG_SUMMARY_TEST:
    {
        struct shdisp_dbg_error_code    code;

        code.mode      = (unsigned char)shdisp_pfs.par[0];
        code.type      = (unsigned char)shdisp_pfs.par[1];
        code.code      = (unsigned char)shdisp_pfs.par[2];
        code.subcode   = (unsigned char)shdisp_pfs.par[3];
        shdisp_dbg_API_err_countup(&code);
        break;
    }

#ifndef SHDISP_NOT_SUPPORT_BKL_CHG_MODE
    case SHDISP_DEBUG_CHARGE_BLK_MODE:
        shdisp_SQE_main_bkl_set_chg_mode(shdisp_pfs.par[0]);

        break;
#endif  /* SHDISP_NOT_SUPPORT_BKL_CHG_MODE */

#ifndef SHDISP_NOT_SUPPORT_BKL_EMG_MODE
    case SHDISP_DEBUG_EMG_BLK_MODE:
        shdisp_SQE_main_bkl_set_emg_mode(shdisp_pfs.par[0]);
        break;
#endif /* SHDISP_NOT_SUPPORT_BKL_EMG_MODE */

    case SHDISP_DEBUG_RECOVERY_NG:
        if (shdisp_pfs.par[0] == 1) {
            recovery_error_flag = SHDISP_DBG_RECOVERY_ERROR_DISPON;
            printk("[SHDISP] set recovery check error disp on (reg)\n");
        } else if (shdisp_pfs.par[0] == 2) {
            recovery_error_flag = SHDISP_DBG_RECOVERY_ERROR_DETLOW;
            printk("[SHDISP] set recovery check error det low\n");
        } else if (shdisp_pfs.par[0] == 3) {
            recovery_error_flag = SHDISP_DBG_RECOVERY_ERROR_PSALS;
            printk("[SHDISP] set recovery check error psals\n");
        } else if (shdisp_pfs.par[0] == 4) {
            recovery_error_flag = SHDISP_DBG_RECOVERY_ERROR_DISPON_READ;
            printk("[SHDISP] set recovery check error disp on (read)\n");
        } else {
            recovery_error_flag = SHDISP_DBG_RECOVERY_ERROR_NONE;
            printk("[SHDISP] set recovery check error none\n");
        }

        recovery_error_count = shdisp_pfs.par[1];
        printk("[SHDISP] set recovery check error retry count=%d\n", recovery_error_count);

        shdisp_dbg_API_set_recovery_check_error(recovery_error_flag, recovery_error_count);
        break;

    case SHDISP_DEBUG_DSI_WRITE:
        panel_reg.size = param[0];
        panel_reg.address = param[1];
        memset(panel_reg.buf, 0, sizeof(panel_reg.buf));

        SHDISP_DEBUG("PANEL INFO ->>");
        SHDISP_DEBUG(" Address : %02Xh", panel_reg.address);
        SHDISP_DEBUG(" Size    : %2d", panel_reg.size);
        for (i = 0; i < panel_reg.size; i++) {
            panel_reg.buf[i] = param[i + 2];
            if ((i % 8) == 0) {
                printk("[SHDISP_DEBUG][%s]  WData    : ", __func__);
            }
            printk("%02X ", panel_reg.buf[i]);
            if ((i % 8) == 7) {
                printk("\n");
            }
        }
        printk("\n");
        shdisp_SQE_panel_write_reg(&panel_reg);

        break;

    case SHDISP_DEBUG_DSI_READ:
        panel_reg.size    = ((shdisp_pfs.par[0] >> 8) & 0x00FF);
        panel_reg.address = ( shdisp_pfs.par[0]       & 0x00FF);
        memset(panel_reg.buf, 0, sizeof(panel_reg.buf));
        shdisp_SQE_panel_read_reg(&panel_reg);

        SHDISP_DEBUG("PANEL INFO ->>");
        SHDISP_DEBUG(" Address : %02Xh", panel_reg.address);
        SHDISP_DEBUG(" Size    : %2d", panel_reg.size);
        for (i = 0; i < panel_reg.size; i++) {
            if ((i % 8) == 0) {
                printk("[SHDISP_DEBUG][%s]  RData    : ", __func__);
            }
            printk("%02X ", panel_reg.buf[i]);
            if ((i % 8) == 7) {
                printk("\n");
            }
        }
        printk("\n");

        break;

    default:
        break;
    }

    printk("[SHDISP] result : %d.\n", ret);

    if (needalloc) {
        kfree(param);
    }

out:

    return count;
}

/* ------------------------------------------------------------------------- */
/* shdisp_proc_read                                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_proc_read( char *page, char **start, off_t offset, int count, int *eof, void *data )
{
    int len = 0;

    len += snprintf(page, count, "%s", proc_buf);
    proc_buf[0] = 0;
    proc_buf_pos = 0;

    return len;
}
#endif /* CONFIG_ANDROID_ENGINEERING */


#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_info_output                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_info_output(int mode)
{
    int i;

    switch (mode) {
    case SHDISP_DEBUG_INFO_TYPE_BOOT:
        printk("[SHDISP] BOOT INFO ->>\n");
        printk("[SHDISP] sizeof(shdisp_kerl_ctx.boot_ctx)                = %d.\n", sizeof(shdisp_kerl_ctx.boot_ctx));
        printk("[SHDISP] boot_ctx.driver_is_initialized         = %d.\n", shdisp_kerl_ctx.boot_ctx.driver_is_initialized);
        printk("[SHDISP] boot_ctx.hw_handset                    = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.hw_handset);
        printk("[SHDISP] boot_ctx.hw_revision                   = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.hw_revision);
        printk("[SHDISP] boot_ctx.device_code                   = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.device_code);
        printk("[SHDISP] boot_ctx.handset_color                 = %d.\n", shdisp_kerl_ctx.boot_ctx.handset_color);
        printk("[SHDISP] boot_ctx.upper_unit_is_connected       = %d.\n", shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected);
        printk("[SHDISP] boot_ctx.main_disp_status              = %d.\n", shdisp_kerl_ctx.boot_ctx.main_disp_status);
        printk("[SHDISP] boot_ctx.is_trickled                   = %d.\n", shdisp_kerl_ctx.boot_ctx.is_trickled);
        printk("[SHDISP] boot_ctx.main_bkl.mode                 = %d.\n", shdisp_kerl_ctx.boot_ctx.main_bkl.mode);
        printk("[SHDISP] boot_ctx.main_bkl.param                = %d.\n", shdisp_kerl_ctx.boot_ctx.main_bkl.param);
        printk("[SHDISP] boot_ctx.tri_led.red                   = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.tri_led.red);
        printk("[SHDISP] boot_ctx.tri_led.green                 = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.tri_led.green);
        printk("[SHDISP] boot_ctx.tri_led.blue                  = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.tri_led.blue);
        printk("[SHDISP] boot_ctx.tri_led.ext_mode              = %d.\n", shdisp_kerl_ctx.boot_ctx.tri_led.ext_mode);
        printk("[SHDISP] boot_ctx.tri_led.led_mode              = %d.\n", shdisp_kerl_ctx.boot_ctx.tri_led.led_mode);
        printk("[SHDISP] boot_ctx.tri_led.ontime                = %d.\n", shdisp_kerl_ctx.boot_ctx.tri_led.ontime);
        printk("[SHDISP] boot_ctx.tri_led.interval              = %d.\n", shdisp_kerl_ctx.boot_ctx.tri_led.interval);
        printk("[SHDISP] boot_ctx.tri_led.count                 = %d.\n", shdisp_kerl_ctx.boot_ctx.tri_led.count);
        printk("[SHDISP] boot_ctx.vcom                         = 0x%04X.\n", (unsigned int)shdisp_kerl_ctx.boot_ctx.vcom);
        printk("[SHDISP] boot_ctx.vcom_low                     = 0x%04X.\n", (unsigned int)shdisp_kerl_ctx.boot_ctx.vcom_low);
        printk("[SHDISP] boot_ctx.vcom_nvram                   = 0x%04X.\n",
                 (unsigned int)shdisp_kerl_ctx.boot_ctx.vcom_nvram);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.status       = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.status);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj0     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].als_adj0);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj1     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].als_adj1);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_shift    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].als_shift);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.clear_offset = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].clear_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.ir_offset    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].ir_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj0     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].als_adj0);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj1     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].als_adj1);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_shift    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].als_shift);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.clear_offset = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].clear_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.ir_offset    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].ir_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.key_backlight    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.key_backlight);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.chksum       = 0x%06X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.chksum);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_is_exist     = %d.\n", shdisp_kerl_ctx.boot_ctx.ledc_status.ledc_is_exist);
        printk("[SHDISP] boot_ctx.ledc_status.power_status      = %d.\n", shdisp_kerl_ctx.boot_ctx.ledc_status.power_status);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.red      = %d.\n",
                (int)shdisp_kerl_ctx.boot_ctx.ledc_status.ledc_req.red);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.green    = %d.\n",
                (int)shdisp_kerl_ctx.boot_ctx.ledc_status.ledc_req.green);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.blue     = %d.\n",
                (int)shdisp_kerl_ctx.boot_ctx.ledc_status.ledc_req.blue);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.led_mode = %d.\n",
                shdisp_kerl_ctx.boot_ctx.ledc_status.ledc_req.led_mode);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.on_count = %d.\n",
                shdisp_kerl_ctx.boot_ctx.ledc_status.ledc_req.on_count);
        printk("[SHDISP] boot_ctx.lcddr_phy_gamma.status        = 0x%02X.\n", shdisp_kerl_ctx.boot_ctx.lcddr_phy_gamma.status);
        printk("[SHDISP] boot_ctx.lcddr_phy_gamma.buf           = ");
        for (i = 0; i < SHDISP_LCDDR_PHY_GAMMA_BUF_MAX; i++) {
            printk("%02X,", shdisp_kerl_ctx.boot_ctx.lcddr_phy_gamma.buf[i]);
        }
        printk("\n");
        printk("[SHDISP] boot_ctx.lcddr_phy_gamma.applied_voltage = ");
        for (i = 0; i < SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE; i++) {
            printk("%02X,", shdisp_kerl_ctx.boot_ctx.lcddr_phy_gamma.applied_voltage[i]);
        }
        printk("\n");
        printk("[SHDISP] boot_ctx.lcddr_phy_gamma.chksum        = 0x%04X.\n", shdisp_kerl_ctx.boot_ctx.lcddr_phy_gamma.chksum);

        printk("[SHDISP] boot_ctx.lut_status                    = 0x%04X.\n", shdisp_kerl_ctx.boot_ctx.lut_status);
        printk("[SHDISP] boot_ctx.argc_lut.red                  = ");
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.red[i][0]);
        }
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.red[i][1]);
        }
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.red[i][2]);
        }
        printk("\n");
        printk("[SHDISP] boot_ctx.argc_lut.green                = ");
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.green[i][0]);
        }
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.green[i][1]);
        }
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.green[i][2]);
        }
        printk("\n");
        printk("[SHDISP] boot_ctx.argc_lut.blue                 = ");
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.blue[i][0]);
        }
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.blue[i][1]);
        }
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.blue[i][2]);
        }
        printk("\n");

        printk("[SHDISP] boot_ctx.igc_lut.r_data                = ");
        for (i = 0; i < SHDISP_IGC_LUT_ENTRIES; i++) {
            if (i % 16 == 0) {
                printk("\n    ");
            }
            printk("%02X,", shdisp_kerl_ctx.boot_ctx.igc_lut.r_data[i]);
        }
        printk("\n");
        printk("[SHDISP] boot_ctx.igc_lut.g_data                = ");
        for (i = 0; i < SHDISP_IGC_LUT_ENTRIES; i++) {
            if (i % 16 == 0) {
                printk("\n    ");
            }
            printk("%02X,", shdisp_kerl_ctx.boot_ctx.igc_lut.g_data[i]);
        }
        printk("\n");
        printk("[SHDISP] boot_ctx.igc_lut.b_data                = ");
        for (i = 0; i < SHDISP_IGC_LUT_ENTRIES; i++) {
            if (i % 16 == 0) {
                printk("\n    ");
            }
            printk("%02X,", shdisp_kerl_ctx.boot_ctx.igc_lut.b_data[i]);
        }
        printk("\n");

        printk("[SHDISP] boot_ctx.bdic_is_exist                 = %d.\n", shdisp_kerl_ctx.boot_ctx.bdic_is_exist);
        printk("[SHDISP] boot_ctx.bdic_chipver                  = %d.\n", shdisp_kerl_ctx.boot_ctx.bdic_chipver);
        printk("[SHDISP] boot_ctx.bdic_status.power_status      = %d.\n", shdisp_kerl_ctx.boot_ctx.bdic_status.power_status);
        printk("[SHDISP] boot_ctx.bdic_status.users             = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.bdic_status.users);
        printk("[SHDISP] boot_ctx.psals_status.power_status     = %d.\n", shdisp_kerl_ctx.boot_ctx.psals_status.power_status);
        printk("[SHDISP] boot_ctx.psals_status.als_users        = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.psals_status.als_users);
        printk("[SHDISP] boot_ctx.psals_status.ps_um_status     = %d.\n", shdisp_kerl_ctx.boot_ctx.psals_status.ps_um_status);
        printk("[SHDISP] boot_ctx.psals_status.als_um_status    = %d.\n", shdisp_kerl_ctx.boot_ctx.psals_status.als_um_status);
        printk("[SHDISP] BOOT INFO <<-\n");
        break;
    case SHDISP_DEBUG_INFO_TYPE_KERNEL:
        printk("[SHDISP] KERNEL INFO ->>\n");
        printk("[SHDISP] sizeof(shdisp_kerl_ctx)                = %d.\n", sizeof(shdisp_kerl_ctx));
        printk("[SHDISP] kerl_ctx.driver_is_open                = %d.\n", shdisp_kerl_ctx.driver_is_open);
        printk("[SHDISP] kerl_ctx.driver_open_cnt               = %d.\n", shdisp_kerl_ctx.driver_open_cnt);
        printk("[SHDISP] kerl_ctx.driver_is_initialized         = %d.\n", shdisp_kerl_ctx.driver_is_initialized);
        printk("[SHDISP] kerl_ctx.dtv_status                    = %d.\n", shdisp_kerl_ctx.dtv_status);
        printk("[SHDISP] kerl_ctx.thermal_status                = %d.\n", shdisp_kerl_ctx.thermal_status);
        printk("[SHDISP] kerl_ctx.usb_chg_status                = %d.\n", shdisp_kerl_ctx.usb_chg_status);
        printk("[SHDISP] kerl_ctx.main_disp_status              = %d.\n", shdisp_kerl_ctx.main_disp_status);
        printk("[SHDISP] kerl_ctx.main_bkl.mode                 = %d.\n", shdisp_kerl_ctx.main_bkl.mode);
        printk("[SHDISP] kerl_ctx.main_bkl.param                = %d.\n", shdisp_kerl_ctx.main_bkl.param);
        printk("[SHDISP] kerl_ctx.tri_led.red                   = %d.\n", (int)shdisp_kerl_ctx.tri_led.red);
        printk("[SHDISP] kerl_ctx.tri_led.green                 = %d.\n", (int)shdisp_kerl_ctx.tri_led.green);
        printk("[SHDISP] kerl_ctx.tri_led.blue                  = %d.\n", (int)shdisp_kerl_ctx.tri_led.blue);
        printk("[SHDISP] kerl_ctx.tri_led.ext_mode              = %d.\n", shdisp_kerl_ctx.tri_led.ext_mode);
        printk("[SHDISP] kerl_ctx.tri_led.led_mode              = %d.\n", shdisp_kerl_ctx.tri_led.led_mode);
        printk("[SHDISP] kerl_ctx.tri_led.ontime                = %d.\n", shdisp_kerl_ctx.tri_led.ontime);
        printk("[SHDISP] kerl_ctx.tri_led.interval              = %d.\n", shdisp_kerl_ctx.tri_led.interval);
        printk("[SHDISP] kerl_ctx.tri_led.count                 = %d.\n", shdisp_kerl_ctx.tri_led.count);
        printk("\n");

        for (i = 0; i < NUM_SHDISP_IRQ_TYPE ; i++) {
            if (shdisp_callback_table[i] != NULL) {
                printk("[SHDISP] shdisp_callback_table[%d]              = subscribed.\n", i);
            } else {
                printk("[SHDISP] shdisp_callback_table[%d]              = no subscribed.\n", i);
            }
        }

        printk("[SHDISP] kerl_ctx.shutdown_in_progress          = %d.\n", shdisp_kerl_ctx.shutdown_in_progress);

        printk("[SHDISP] KERNEL INFO <<-\n");
        break;

    case SHDISP_DEBUG_INFO_TYPE_POWERON:
        printk("[SHDISP] BOOT INFO ->>\n");
        printk("[SHDISP] sizeof(shdisp_kerl_ctx.boot_ctx)                = %d.\n", sizeof(shdisp_kerl_ctx.boot_ctx));
        printk("[SHDISP] boot_ctx.driver_is_initialized         = %d.\n", shdisp_kerl_ctx.boot_ctx.driver_is_initialized);
        printk("[SHDISP] boot_ctx.hw_handset                    = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.hw_handset);
        printk("[SHDISP] boot_ctx.hw_revision                   = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.hw_revision);
        printk("[SHDISP] boot_ctx.device_code                   = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.device_code);
        printk("[SHDISP] boot_ctx.handset_color                 = %d.\n", shdisp_kerl_ctx.boot_ctx.handset_color);
        printk("[SHDISP] boot_ctx.upper_unit_is_connected       = %d.\n", shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected);
        printk("[SHDISP] boot_ctx.main_disp_status              = %d.\n", shdisp_kerl_ctx.boot_ctx.main_disp_status);
        printk("[SHDISP] boot_ctx.is_trickled                   = %d.\n", shdisp_kerl_ctx.boot_ctx.is_trickled);
        printk("[SHDISP] boot_ctx.main_bkl.mode                 = %d.\n", shdisp_kerl_ctx.boot_ctx.main_bkl.mode);
        printk("[SHDISP] boot_ctx.main_bkl.param                = %d.\n", shdisp_kerl_ctx.boot_ctx.main_bkl.param);
        printk("[SHDISP] boot_ctx.tri_led.red                   = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.tri_led.red);
        printk("[SHDISP] boot_ctx.tri_led.green                 = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.tri_led.green);
        printk("[SHDISP] boot_ctx.tri_led.blue                  = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.tri_led.blue);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.status       = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.status);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj0     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].als_adj0);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj1     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].als_adj1);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_shift    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].als_shift);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.clear_offset = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].clear_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.ir_offset    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].ir_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj0     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].als_adj0);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj1     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].als_adj1);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_shift    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].als_shift);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.clear_offset = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].clear_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.ir_offset    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].ir_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.chksum       = 0x%06X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.chksum);
        printk("[SHDISP] boot_ctx.bdic_is_exist                 = %d.\n", shdisp_kerl_ctx.boot_ctx.bdic_is_exist);
        printk("[SHDISP] boot_ctx.bdic_chipver                  = %d.\n", shdisp_kerl_ctx.boot_ctx.bdic_chipver);
        printk("[SHDISP] boot_ctx.bdic_status.power_status      = %d.\n", shdisp_kerl_ctx.boot_ctx.bdic_status.power_status);
        printk("[SHDISP] boot_ctx.bdic_status.users             = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.bdic_status.users);
        printk("[SHDISP] boot_ctx.psals_status.power_status     = %d.\n", shdisp_kerl_ctx.boot_ctx.psals_status.power_status);
        printk("[SHDISP] boot_ctx.psals_status.als_users        = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.psals_status.als_users);
        printk("[SHDISP] boot_ctx.psals_status.ps_um_status     = %d.\n", shdisp_kerl_ctx.boot_ctx.psals_status.ps_um_status);
        printk("[SHDISP] boot_ctx.psals_status.als_um_status    = %d.\n", shdisp_kerl_ctx.boot_ctx.psals_status.als_um_status);
        printk("[SHDISP] BOOT INFO <<-\n");
        printk("[SHDISP] KERNEL INFO ->>\n");
        printk("[SHDISP] sizeof(shdisp_kerl_ctx)                = %d.\n", sizeof(shdisp_kerl_ctx));
        printk("[SHDISP] kerl_ctx.driver_is_open                = %d.\n", shdisp_kerl_ctx.driver_is_open);
        printk("[SHDISP] kerl_ctx.driver_open_cnt               = %d.\n", shdisp_kerl_ctx.driver_open_cnt);
        printk("[SHDISP] kerl_ctx.driver_is_initialized         = %d.\n", shdisp_kerl_ctx.driver_is_initialized);
        printk("[SHDISP] kerl_ctx.dtv_status                    = %d.\n", shdisp_kerl_ctx.dtv_status);
        printk("[SHDISP] kerl_ctx.thermal_status                = %d.\n", shdisp_kerl_ctx.thermal_status);
        printk("[SHDISP] kerl_ctx.usb_chg_status                = %d.\n", shdisp_kerl_ctx.usb_chg_status);
        printk("[SHDISP] kerl_ctx.main_disp_status              = %d.\n", shdisp_kerl_ctx.main_disp_status);
        printk("[SHDISP] kerl_ctx.main_bkl.mode                 = %d.\n", shdisp_kerl_ctx.main_bkl.mode);
        printk("[SHDISP] kerl_ctx.main_bkl.param                = %d.\n", shdisp_kerl_ctx.main_bkl.param);
        printk("[SHDISP] kerl_ctx.tri_led.red                   = %d.\n", (int)shdisp_kerl_ctx.tri_led.red);
        printk("[SHDISP] kerl_ctx.tri_led.green                 = %d.\n", (int)shdisp_kerl_ctx.tri_led.green);
        printk("[SHDISP] kerl_ctx.tri_led.blue                  = %d.\n", (int)shdisp_kerl_ctx.tri_led.blue);
        printk("[SHDISP] kerl_ctx.tri_led.ext_mode              = %d.\n", shdisp_kerl_ctx.tri_led.ext_mode);
        printk("[SHDISP] kerl_ctx.tri_led.led_mode              = %d.\n", shdisp_kerl_ctx.tri_led.led_mode);
        printk("[SHDISP] kerl_ctx.tri_led.ontime                = %d.\n", shdisp_kerl_ctx.tri_led.ontime);
        printk("[SHDISP] kerl_ctx.tri_led.interval              = %d.\n", shdisp_kerl_ctx.tri_led.interval);
        printk("[SHDISP] kerl_ctx.tri_led.count                 = %d.\n", shdisp_kerl_ctx.tri_led.count);
        printk("[SHDISP] KERNEL INFO <<-\n");
        break;

    case SHDISP_DEBUG_INFO_TYPE_BDIC:
        shdisp_bdic_API_DBG_INFO_output();
        break;
    case SHDISP_DEBUG_INFO_TYPE_SENSOR:
        shdisp_bdic_API_PSALS_INFO_output();
        break;
    case SHDISP_DEBUG_INFO_TYPE_PANEL:
        shdisp_panel_API_dump(0);
        break;
    case SHDISP_DEBUG_INFO_TYPE_PM:
        shdisp_pm_API_power_manager_users_dump();
        break;
    case SHDISP_DEBUG_INFO_TYPE_BDIC_OPT:
        shdisp_bdic_API_OPT_INFO_output();
        break;
    default:
        break;
    }

    return;
}
#endif /* CONFIG_ANDROID_ENGINEERING */

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_que                                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_que(int kind)
{
    unsigned int nRcvGFAC = 0;
    struct shdisp_queue_data_t *qdata = NULL;
    int    i;
    int    bFirstQue = 0;
    int    ret;
    int    nBDIC_QueFac = 0;


    SHDISP_TRACE(": Start");

    switch (kind) {
    case 1:
        nRcvGFAC = 0x00000000;
        break;
    case 2:
        nRcvGFAC = 0x00200000;
        break;
    case 3:
        nRcvGFAC = 0x00000100;
        break;
    case 4:
        nRcvGFAC = 0x00200100;
        break;
    case 5:
        nRcvGFAC = 0x00000008;
        break;
    case 6:
        nRcvGFAC = 0x00200008;
        break;
    case 7:
        nRcvGFAC = 0x00000108;
        break;
    case 8:
        nRcvGFAC = 0x00200108;
        break;
    case 9:
        nRcvGFAC = 0x00080000;
        break;
    case 10:
        nRcvGFAC = 0x00280000;
        break;
    case 11:
        nRcvGFAC = 0x00080100;
        break;
    case 12:
        nRcvGFAC = 0x00280100;
        break;
    case 13:
        nRcvGFAC = 0x00080008;
        break;
    case 14:
        nRcvGFAC = 0x00280008;
        break;
    case 15:
        nRcvGFAC = 0x00080108;
        break;
    case 16:
        nRcvGFAC = 0x00280108;
        break;
    case 17:
        nRcvGFAC = 0x00040000;
        break;
    case 18:
        nRcvGFAC = 0x00240000;
        break;
    case 19:
        nRcvGFAC = 0x00040100;
        break;
    case 20:
        nRcvGFAC = 0x00240100;
        break;
    case 21:
        nRcvGFAC = 0x00040008;
        break;
    case 22:
        nRcvGFAC = 0x00240008;
        break;
    case 23:
        nRcvGFAC = 0x00040108;
        break;
    case 24:
        nRcvGFAC = 0x00240108;
        break;
    case 25:
        nRcvGFAC = 0x000C0000;
        break;
    case 26:
        nRcvGFAC = 0x002C0000;
        break;
    case 27:
        nRcvGFAC = 0x000C0100;
        break;
    case 28:
        nRcvGFAC = 0x002C0100;
        break;
    case 29:
        nRcvGFAC = 0x000C0008;
        break;
    case 30:
        nRcvGFAC = 0x002C0008;
        break;
    case 31:
        nRcvGFAC = 0x000C0108;
        break;
    case 32:
        nRcvGFAC = 0x002C0108;
        break;
    case 33:
        nRcvGFAC = 0x00000200;
        break;
    case 34:
        nRcvGFAC = 0x00080200;
        break;
    case 35:
        nRcvGFAC = 0x00200200;
        break;
    case 36:
        nRcvGFAC = 0x00280200;
        break;
    case 37:
        nRcvGFAC = 0x00000300;
        break;
    case 38:
        nRcvGFAC = 0x00080300;
        break;
    case 39:
        nRcvGFAC = 0x00200300;
        break;
    case 40:
        nRcvGFAC = 0x00280300;
        break;
    case 41:
        nRcvGFAC = 0x00000208;
        break;
    case 42:
        nRcvGFAC = 0x00080208;
        break;
    case 43:
        nRcvGFAC = 0x00200208;
        break;
    case 44:
        nRcvGFAC = 0x00280208;
        break;
    case 45:
        nRcvGFAC = 0x00000308;
        break;
    case 46:
        nRcvGFAC = 0x00080308;
        break;
    case 47:
        nRcvGFAC = 0x00200308;
        break;
    case 48:
        nRcvGFAC = 0x00280308;
        break;
    case 49:
        nRcvGFAC = 0x00040200;
        break;
    case 50:
        nRcvGFAC = 0x000C0200;
        break;
    case 51:
        nRcvGFAC = 0x00240200;
        break;
    case 52:
        nRcvGFAC = 0x002C0200;
        break;
    case 53:
        nRcvGFAC = 0x00040300;
        break;
    case 54:
        nRcvGFAC = 0x000C0300;
        break;
    case 55:
        nRcvGFAC = 0x00240300;
        break;
    case 56:
        nRcvGFAC = 0x002C0300;
        break;
    case 57:
        nRcvGFAC = 0x00040208;
        break;
    case 58:
        nRcvGFAC = 0x000C0208;
        break;
    case 59:
        nRcvGFAC = 0x00240208;
        break;
    case 60:
        nRcvGFAC = 0x002C0208;
        break;
    case 61:
        nRcvGFAC = 0x00040308;
        break;
    case 62:
        nRcvGFAC = 0x000C0308;
        break;
    case 63:
        nRcvGFAC = 0x00240308;
        break;
    case 64:
        nRcvGFAC = 0x002C0308;
        break;

    default:
        nRcvGFAC = 0;
        break;
    }

    shdisp_SYS_API_set_irq(SHDISP_IRQ_DISABLE);
    shdisp_wake_lock();

    shdisp_bdic_API_IRQ_dbg_set_fac(nRcvGFAC);

    do {
        shdisp_semaphore_start();
        ret = shdisp_bdic_API_IRQ_check_fac();
        shdisp_semaphore_end(__func__);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG(": no factory");
            break;
        }

        down(&shdisp_sem_irq_fac);
        for (i = 0; i < SHDISP_IRQ_MAX_KIND; i++) {
            shdisp_semaphore_start();
            nBDIC_QueFac = shdisp_bdic_API_IRQ_get_fac(i);
            shdisp_semaphore_end(__func__);
            if (nBDIC_QueFac == SHDISP_BDIC_IRQ_TYPE_NONE) {
                break;
            }

            if (shdisp_wq_gpio_task) {
                qdata = kmalloc( sizeof(shdisp_queue_data), GFP_KERNEL );
                if (qdata != NULL) {
                    qdata->irq_GFAC = nBDIC_QueFac;
                    list_add_tail(&qdata->list, &shdisp_queue_data.list);
                    if (bFirstQue == 0) {
                        bFirstQue = 1;
                        shdisp_wake_lock();
                        ret = queue_work(shdisp_wq_gpio_task, &shdisp_wq_gpio_task_wk);
                        if (ret == 0) {
                            shdisp_wake_unlock();
                            SHDISP_ERR("<QUEUE_WORK_FAILURE> ");
                        }
                    }
                } else {
                   SHDISP_ERR("<QUEUE_WORK_FAILURE> :kmalloc failed (BDIC_QueFac=%d)", nBDIC_QueFac);
                }
            }
        }
        up(&shdisp_sem_irq_fac);

    } while (0);

    shdisp_semaphore_start();
    shdisp_bdic_API_IRQ_Clear();
    shdisp_semaphore_end(__func__);

    if (shdisp_bdic_API_IRQ_check_DET() != SHDISP_BDIC_IRQ_TYPE_DET) {
        shdisp_SYS_API_set_irq(SHDISP_IRQ_ENABLE);
    }

    SHDISP_TRACE(": Finish");
    shdisp_wake_unlock();
}

/* ------------------------------------------------------------------------- */
/* shdisp_debug_subscribe                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_debug_subscribe(void)
{
    struct shdisp_subscribe dbg_subs;

    dbg_subs.irq_type = SHDISP_IRQ_TYPE_PS;
    dbg_subs.callback = callback_ps;
    shdisp_event_subscribe(&dbg_subs);
}

/* ------------------------------------------------------------------------- */
/* callback_ps                                                               */
/* ------------------------------------------------------------------------- */
static void callback_ps(void)
{
    printk("[SHDISP] callback_ps Start\n");
    msleep(1000);
    printk("[SHDISP] callback_ps Finish\n");
}
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* shdisp_fb_open                                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_fb_open(void)
{
    struct fb_info *info = NULL;

    if (!num_registered_fb) {
        return;
    }
    info = registered_fb[0];
    if (!info) {
        return;
    }
    if (!try_module_get(info->fbops->owner)) {
        return;
    }
    if (info->fbops->fb_open && info->fbops->fb_open(info, 0)) {
        module_put(info->fbops->owner);
        return;
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_fb_close                                                           */
/* ------------------------------------------------------------------------- */
static void shdisp_fb_close(void)
{
    struct fb_info *info = NULL;

    info = registered_fb[0];
    if (!info) {
        return;
    }
    if (info->fbops->fb_release) {
        info->fbops->fb_release(info, 0);
    }
    module_put(info->fbops->owner);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_boot_err_output                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_boot_err_output(void)
{
#ifdef SHDISP_RESET_LOG
    int i;
    struct shdisp_dbg_error_code* err_codes;
    int *err_codes_reset;
    int err_codes_count;

    for (i = 0; i < SHDISP_NOOS_RESET_NUM; i++) {
        if (shdisp_kerl_ctx.boot_ctx.err_on[i] == 1) {
            shdisp_kerl_ctx.boot_ctx.err_on[i] = 0;
            shdisp_dbg_API_err_output(&shdisp_kerl_ctx.boot_ctx.err_code[i], 0);
        }
    }

    shdisp_dbg_API_get_boot_errcodes(&err_codes, &err_codes_reset, &err_codes_count);
    for (i = 0; i < err_codes_count; i++) {
        shdisp_dbg_API_err_output(&err_codes[i], 0);
    }
    shdisp_dbg_API_clear_boot_errcodes();
#endif /* SHDISP_RESET_LOG */
}

/* ------------------------------------------------------------------------- */
/* shdisp_init                                                               */
/* ------------------------------------------------------------------------- */
static int __init shdisp_init(void)
{
    int ret;
    struct shdisp_bdic_state_str    state_str;
    int shdisp_subscribe_type;
    int i;
    unsigned long int notify_value = 0, notify_brightness = 0;
    struct shdisp_main_bkl_ctl bkl_ctl;
    struct shdisp_tri_led tri_led;
    struct shdisp_panel_context shdisp_panel_ctx;

#if defined(CONFIG_ANDROID_ENGINEERING)
    struct proc_dir_entry *entry;
#endif /* CONFIG_ANDROID_ENGINEERING */

    shdisp_init_context();

    shdisp_panel_API_create();

    shdisp_SYS_API_Host_gpio_init();

    ret = alloc_chrdev_region(&shdisp_dev, 0, 1, SHDISP_NAME);

    if (!ret) {
        shdisp_major = MAJOR(shdisp_dev);
        shdisp_minor = MINOR(shdisp_dev);
    } else {
        goto shdisp_err_1;
    }

    cdev_init(&shdisp_cdev, &shdisp_fops);

    shdisp_cdev.owner = THIS_MODULE;
    shdisp_cdev.ops   = &shdisp_fops;

    ret = cdev_add(&shdisp_cdev, MKDEV(shdisp_major, 0), 1);

    if (ret) {
        goto shdisp_err_2;
    }

    shdisp_class = class_create(THIS_MODULE, SHDISP_NAME);

    if (IS_ERR(shdisp_class)) {
        goto shdisp_err_3;
    }

    device_create(shdisp_class, NULL,
                  shdisp_dev, &shdisp_cdev, SHDISP_NAME);

    ret = shdisp_SYS_API_bdic_i2c_init();

    if (ret) {
        goto shdisp_err_4;
    }

    ret = shdisp_SYS_API_sensor_i2c_init();

    if (ret) {
        goto shdisp_err_6;
    }


    shdisp_panel_ctx.device_code    = shdisp_kerl_ctx.boot_ctx.device_code;
    shdisp_panel_ctx.vcom           = shdisp_kerl_ctx.boot_ctx.vcom;
    shdisp_panel_ctx.vcom_low       = shdisp_kerl_ctx.boot_ctx.vcom_low;
    shdisp_panel_ctx.vcom_nvram     = shdisp_kerl_ctx.boot_ctx.vcom_nvram;

    memcpy(&(shdisp_panel_ctx.lcddr_phy_gamma), &(shdisp_kerl_ctx.boot_ctx.lcddr_phy_gamma),
            sizeof(struct shdisp_lcddr_phy_gamma_reg));

    ret = shdisp_panel_API_init_io(&shdisp_panel_ctx);

    if (ret) {
        goto shdisp_err_61;
    }

    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        first_lcd_on = true;
    }

#if defined(CONFIG_ANDROID_ENGINEERING)
    entry = create_proc_entry("driver/SHDISP", 0666, NULL);

    if (entry == NULL) {
        goto shdisp_err_7;
    }

    entry->write_proc = shdisp_proc_write;
    entry->read_proc  = shdisp_proc_read;
#endif /* CONFIG_ANDROID_ENGINEERING */

    sema_init(&shdisp_sem, 1);

    sema_init(&shdisp_sem_callback, 1);
    sema_init(&shdisp_sem_irq_fac, 1 );
    sema_init(&shdisp_lux_change_sem, 1);
    sema_init(&shdisp_sem_timer, 1);
    sema_init(&shdisp_sem_req_recovery_lcd, 1);
    sema_init(&shdisp_sem_req_recovery_psals, 1);

    spin_lock_init(&shdisp_q_lock);
    spin_lock_init(&shdisp_wake_spinlock);
#ifdef  SHDISP_TRI_LED2
    spin_lock_init(&shdisp_swic_spinlock);
#endif  /* SHDISP_TRI_LED2 */

    shdisp_dbg_API_init();
    shdisp_SYS_API_set_irq_init();

    shdisp_wake_lock_init();

    memset(&shdisp_queue_data, 0, sizeof(shdisp_queue_data));
    INIT_LIST_HEAD( &shdisp_queue_data.list);

    shdisp_wq_gpio = create_singlethread_workqueue("shdisp_gpio_queue");

    if (shdisp_wq_gpio) {
        INIT_WORK(&shdisp_wq_gpio_wk,
                  shdisp_workqueue_handler_gpio);
    } else {
        goto shdisp_err_8;
    }

    shdisp_wq_gpio_task = create_singlethread_workqueue("shdisp_gpio_queue_task");

    if (shdisp_wq_gpio_task) {
        INIT_WORK(&shdisp_wq_gpio_task_wk,
                  shdisp_workqueue_gpio_task);
    } else {
        goto shdisp_err_9;
    }

    shdisp_wq_recovery = create_singlethread_workqueue("shdisp_recovery_task");

    if (shdisp_wq_recovery) {
        INIT_WORK(&shdisp_wq_recovery_lcd_wk,
                  shdisp_workqueue_handler_recovery_lcd);
        INIT_WORK(&shdisp_wq_recovery_psals_wk,
                  shdisp_workqueue_handler_recovery_psals);
    } else {
        goto shdisp_err_91;
    }

    down(&shdisp_sem_callback);
    for (i = 0; i < NUM_SHDISP_IRQ_TYPE ; i++) {
        shdisp_callback_table[i] = NULL;
    }
    up(&shdisp_sem_callback);

    init_timer(&shdisp_timer);

    shdisp_wq_timer_task = create_singlethread_workqueue("shdisp_timer_queue_task");
    if (shdisp_wq_timer_task) {
        INIT_WORK(&shdisp_wq_timer_task_wk, shdisp_workqueue_timer_task);
    } else {
        goto shdisp_err_10;
    }

#ifdef  SHDISP_TRI_LED2
    shdisp_wq_pierce = create_singlethread_workqueue("shdisp_pierce_queue");
    if (shdisp_wq_pierce) {
        INIT_WORK(&shdisp_wq_pierce_wk, shdisp_workqueue_pierce);
    } else {
        goto shdisp_err_11;
    }
#endif  /* SHDISP_TRI_LED2 */

    for (i = 0; i < NUM_SHDISP_IRQ_TYPE ; i++) {
        shdisp_subscribe_type = SHDISP_SUBSCRIBE_TYPE_INT;
        shdisp_subscribe_type_table[i] = shdisp_subscribe_type;
    }

    shdisp_kerl_ctx.driver_is_initialized = SHDISP_DRIVER_IS_INITIALIZED;

    state_str.bdic_chipver  = shdisp_kerl_ctx.boot_ctx.bdic_chipver;
    state_str.handset_color = shdisp_kerl_ctx.boot_ctx.handset_color;

    memcpy(&(state_str.photo_sensor_adj),
                                &(shdisp_kerl_ctx.boot_ctx.photo_sensor_adj), sizeof(struct shdisp_photo_sensor_adj));

#ifdef SHDISP_NOT_SUPPORT_NO_OS
    shdisp_kerl_ctx.boot_ctx.bdic_is_exist = shdisp_bdic_API_boot_init();
    shdisp_kerl_ctx.boot_ctx.bdic_chipver = shdisp_bdic_API_initialize(&state_str);
#else  /* SHDISP_NOT_SUPPORT_NO_OS*/
    shdisp_bdic_API_boot_init();
    shdisp_bdic_API_initialize(&state_str);
#endif /* SHDISP_NOT_SUPPORT_NO_OS*/

    bkl_ctl.mode  = shdisp_kerl_ctx.main_bkl.mode;
    bkl_ctl.param = shdisp_kerl_ctx.main_bkl.param;
    shdisp_bdic_API_LCD_BKL_set_request(SHDISP_MAIN_BKL_DEV_TYPE_APP, &bkl_ctl);

    tri_led.red      = shdisp_kerl_ctx.tri_led.red;
    tri_led.green    = shdisp_kerl_ctx.tri_led.green;
    tri_led.blue     = shdisp_kerl_ctx.tri_led.blue;
    tri_led.ext_mode = shdisp_kerl_ctx.tri_led.ext_mode;
    tri_led.led_mode = shdisp_kerl_ctx.tri_led.led_mode;
    tri_led.ontime   = shdisp_kerl_ctx.tri_led.ontime;
    tri_led.interval = shdisp_kerl_ctx.tri_led.interval;
    tri_led.count    = shdisp_kerl_ctx.tri_led.count;
    shdisp_bdic_API_TRI_LED_set_request(&tri_led);

    init_completion(&lux_change_notify);

    ret = shdisp_SYS_API_request_irq( shdisp_gpio_int_isr );
    if (ret) {
        goto shdisp_err_top;
    }
    shdisp_pm_API_init(&(shdisp_kerl_ctx.boot_ctx));
    shdisp_fb_open();

#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_LCDPOW, shdisp_kerl_ctx.main_disp_status);
#endif  /* CONFIG_SHTERM */

    if (shdisp_kerl_ctx.main_bkl.mode != SHDISP_MAIN_BKL_MODE_OFF) {
        notify_value = 1;
        shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
    }

#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT, notify_value);
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT_LEV, notify_brightness);
#endif  /* CONFIG_SHTERM */

    return SHDISP_RESULT_SUCCESS;

shdisp_err_top:
#ifdef  SHDISP_TRI_LED2
    flush_workqueue(shdisp_wq_pierce);
    destroy_workqueue(shdisp_wq_pierce);
    shdisp_wq_pierce = NULL;

shdisp_err_11:
#endif  /* SHDISP_TRI_LED2 */
    flush_workqueue(shdisp_wq_timer_task);
    destroy_workqueue(shdisp_wq_timer_task);
    shdisp_wq_timer_task = NULL;

shdisp_err_10:
    flush_workqueue(shdisp_wq_recovery);
    destroy_workqueue(shdisp_wq_recovery);
    shdisp_wq_recovery = NULL;

shdisp_err_91:
    flush_workqueue(shdisp_wq_gpio_task);
    destroy_workqueue(shdisp_wq_gpio_task);
    shdisp_wq_gpio_task = NULL;

shdisp_err_9:
    flush_workqueue(shdisp_wq_gpio);
    destroy_workqueue(shdisp_wq_gpio);
    shdisp_wq_gpio = NULL;

shdisp_err_8:
#if defined(CONFIG_ANDROID_ENGINEERING)
shdisp_err_7:
#endif /* CONFIG_ANDROID_ENGINEERING */
    shdisp_panel_API_exit_io();

shdisp_err_61:

shdisp_err_6:
    shdisp_SYS_API_sensor_i2c_exit();
    shdisp_SYS_API_bdic_i2c_exit();

shdisp_err_4:
    device_destroy(shdisp_class, MKDEV(shdisp_major, 0));

shdisp_err_3:
    class_destroy(shdisp_class);

shdisp_err_2:
    cdev_del(&shdisp_cdev);

shdisp_err_1:
    shdisp_SYS_API_Host_gpio_exit();
    unregister_chrdev_region(MKDEV(shdisp_major, 0), 1);
    return -EIO;
}
module_init(shdisp_init);

/* ------------------------------------------------------------------------- */
/* shdisp_exit                                                               */
/* ------------------------------------------------------------------------- */
static void shdisp_exit(void)
{
    shdisp_fb_close();

    lux_change_wait_flg = SHDISP_LUX_CHANGE_STATE_EXIT;
    complete(&lux_change_notify);

    wake_lock_destroy(&shdisp_wake_lock_wq);

    shdisp_SYS_API_Host_control(SHDISP_HOST_CTL_CMD_LCD_CLK_EXIT, 0);
    shdisp_SYS_API_free_irq();
    if (shdisp_wq_gpio) {
        flush_workqueue(shdisp_wq_gpio);
        destroy_workqueue(shdisp_wq_gpio);
        shdisp_wq_gpio = NULL;
    }

    if (shdisp_wq_gpio_task) {
        flush_workqueue(shdisp_wq_gpio_task);
        destroy_workqueue(shdisp_wq_gpio_task);
        shdisp_wq_gpio_task = NULL;
    }

    shdisp_panel_API_exit_io();
    shdisp_SYS_API_sensor_i2c_exit();
    shdisp_SYS_API_bdic_i2c_exit();
    shdisp_SYS_API_Host_gpio_exit();
    device_destroy(shdisp_class, MKDEV(shdisp_major, 0));
    class_destroy(shdisp_class);
    cdev_del(&shdisp_cdev);
    unregister_chrdev_region(MKDEV(shdisp_major, 0), 1);
    return;
}
module_exit(shdisp_exit);

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
