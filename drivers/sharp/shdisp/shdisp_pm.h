/* drivers/sharp/shdisp/shdisp_pm.h  (Display Driver)
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
#ifndef SHDISP_PM_H
#define SHDISP_PM_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_DEV_TYPE_NONE        (0x00000000)
#define SHDISP_DEV_TYPE_LCD         (0x00000001)
#define SHDISP_DEV_TYPE_BKL         (0x00000002)
#define SHDISP_DEV_TYPE_LED         (0x00000004)
#define SHDISP_DEV_TYPE_ALS         (0x00000008)
#define SHDISP_DEV_TYPE_PS          (0x00000010)
#define SHDISP_DEV_TYPE_LED2        (0x00000020)
#define SHDISP_DEV_TYPE_ALS_APP     (0x00000100)
#define SHDISP_DEV_TYPE_CAMERA      (0x00000200)
#define SHDISP_DEV_TYPE_KEYLED      (0x00000400)
#define SHDISP_DEV_TYPE_DIAG        (0x00000800)
#define SHDISP_DEV_TYPE_RECOVERY    (0x01000000)
#define SHDISP_DEV_TYPE_INIT        (0x02000000)
#define SHDISP_DEV_TYPE_DEBUG       (0x10000000)

#define SHDISP_DEV_TYPE_BDIC_MASK   (SHDISP_DEV_TYPE_LCD      | \
                                     SHDISP_DEV_TYPE_BKL      | \
                                     SHDISP_DEV_TYPE_LED      | \
                                     SHDISP_DEV_TYPE_ALS      | \
                                     SHDISP_DEV_TYPE_PS       | \
                                     SHDISP_DEV_TYPE_LED2     | \
                                     SHDISP_DEV_TYPE_RECOVERY | \
                                     SHDISP_DEV_TYPE_INIT     | \
                                     SHDISP_DEV_TYPE_DEBUG)

#define SHDISP_DEV_TYPE_PSALS_MASK  (SHDISP_DEV_TYPE_ALS      | \
                                     SHDISP_DEV_TYPE_PS)

#define SHDISP_DEV_TYPE_PS_MASK     (SHDISP_DEV_TYPE_PS)

#define SHDISP_DEV_TYPE_ALS_MASK    (SHDISP_DEV_TYPE_LCD      | \
                                     SHDISP_DEV_TYPE_BKL      | \
                                     SHDISP_DEV_TYPE_ALS_APP  | \
                                     SHDISP_DEV_TYPE_CAMERA   | \
                                     SHDISP_DEV_TYPE_KEYLED   | \
                                     SHDISP_DEV_TYPE_DIAG     | \
                                     SHDISP_DEV_TYPE_DEBUG)

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_DEV_STATE_NOINIT = 0,
    SHDISP_DEV_STATE_OFF,
    SHDISP_DEV_STATE_ON,
    NUM_SHDISP_DEV_STATE
};

enum {
    SHDISP_DEV_REQ_INIT = 0,
    SHDISP_DEV_REQ_OFF,
    SHDISP_DEV_REQ_ON,
    NUM_SHDISP_DEV_REQ
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                    */
/* ------------------------------------------------------------------------- */
void shdisp_pm_API_init(struct shdisp_boot_context *shdisp_boot_ctx);
int shdisp_pm_API_bdic_power_manager(int user, int onoff);
int shdisp_pm_API_ps_user_manager(int user, int onoff);
int shdisp_pm_API_als_user_manager(int user, int onoff);
int shdisp_pm_API_is_ps_active(void);
int shdisp_pm_API_is_als_active(void);
void shdisp_pm_API_bdic_shutdown(void);
void shdisp_pm_API_bdic_resume(void);
void shdisp_pm_API_psals_error_power_off(void);
int shdisp_pm_API_psals_error_power_recovery(void);
#if defined(CONFIG_ANDROID_ENGINEERING)
void shdisp_pm_API_power_manager_users_dump(void);
#endif /* CONFIG_ANDROID_ENGINEERING */

#endif /* SHDISP_PM_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
