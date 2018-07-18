/* drivers/sharp/shdisp/shdisp_pm.c  (Display Driver)
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/irq.h>

#include <sharp/shdisp_kerl.h>
#include "shdisp_kerl_priv.h"

#include "shdisp_system.h"
#include "shdisp_bdic.h"
#include "shdisp_pm.h"
#include "shdisp_dbg.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_NAME "shdisp"

#define GET_DEV_STATE_STR(n)    (((n) == SHDISP_DEV_STATE_NOINIT) ? "NOINIT" : \
                                (((n) == SHDISP_DEV_STATE_OFF)    ? "OFF"    : \
                                (((n) == SHDISP_DEV_STATE_ON)     ? "ON"     : \
                                                                    "UNKNOWN")))

#define GET_DEV_REQ_STR(n)      (((n) == SHDISP_DEV_REQ_INIT)     ? "INIT"   : \
                                (((n) == SHDISP_DEV_REQ_OFF)      ? "OFF"    : \
                                (((n) == SHDISP_DEV_REQ_ON)       ? "ON"     : \
                                                                    "UNKNOWN")))

#define GET_PSALS_STATE_STR(n)  (((n) == SHDISP_PSALS_STATE_OFF)  ? "OFF"    : \
                                (((n) == SHDISP_PSALS_STATE_INIT) ? "INIT"   : \
                                (((n) == SHDISP_PSALS_STATE_ON)   ? "ON"     : \
                                                                    "UNKNOWN")))

#define GET_SENSOR_STATE_STR(n) (((n) == SHDISP_SENSOR_STATE_POWER_OFF)       ? "ALL_OFF"    : \
                                (((n) == SHDISP_SENSOR_STATE_POWER_ON)        ? "ON/OFF/OFF" : \
                                (((n) == SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF) ? "ON/ON/OFF"  : \
                                (((n) == SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON) ? "ON/OFF/ON"  : \
                                (((n) == SHDISP_SENSOR_STATE_PROX_ON_ALS_ON)  ? "ON/ON/ON"   : \
                                                                                "UNKNOWN")))))

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_PSALS_STATE_OFF = 0,
    SHDISP_PSALS_STATE_INIT,
    SHDISP_PSALS_STATE_ON
};

enum {
    SHDISP_SENSOR_STATE_POWER_OFF = 0,
    SHDISP_SENSOR_STATE_POWER_ON,
    SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF,
    SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON,
    SHDISP_SENSOR_STATE_PROX_ON_ALS_ON,
    NUM_SHDISP_SENSOR_STATE
};

struct shdisp_pm_context {
    struct shdisp_bdic_status bdic_status;
    struct shdisp_psals_status psals_status;
};

struct shdisp_pm_psals_state_row {
    int user;
    int onoff;
    int (* funcs[NUM_SHDISP_SENSOR_STATE])(void);
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_power_manager(int user, int onoff);
static const struct shdisp_pm_psals_state_row *shdisp_pm_psals_find_state_table(int user, int onoff);
static int shdisp_pm_bdic_set_active(void);
static int shdisp_pm_bdic_set_standby(void);
static int shdisp_pm_psals_power_on(void);
static int shdisp_pm_psals_power_off(void);
static int shdisp_pm_psals_init(void);
static int shdisp_pm_psals_deinit(void);
static int shdisp_pm_psals_ps_init_als_off(void);
static int shdisp_pm_psals_ps_init_als_on(void);
static int shdisp_pm_psals_ps_deinit_als_off(void);
static int shdisp_pm_psals_ps_deinit_als_on(void);
static int shdisp_pm_psals_als_init_ps_off(void);
static int shdisp_pm_psals_als_init_ps_on(void);
static int shdisp_pm_psals_als_deinit_ps_off(void);
static int shdisp_pm_psals_als_deinit_ps_on(void);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static struct shdisp_pm_context shdisp_pm_ctx;
static struct shdisp_pm_context shdisp_pm_ctx_recovery;

static const struct shdisp_pm_psals_state_row
        shdisp_pm_psals_state_table[] = {
    {
        SHDISP_DEV_TYPE_ALS, SHDISP_DEV_REQ_INIT,
        {
            shdisp_pm_psals_init,
            NULL,
            shdisp_pm_psals_als_init_ps_on,
            NULL,
            NULL
        }
    },
    {
        SHDISP_DEV_TYPE_PS, SHDISP_DEV_REQ_ON,
        {
            shdisp_pm_psals_ps_init_als_off,
            shdisp_pm_psals_ps_init_als_off,
            NULL,
            shdisp_pm_psals_ps_init_als_on,
            NULL
        }
    },
    {
        SHDISP_DEV_TYPE_PS, SHDISP_DEV_REQ_OFF,
        {
            NULL,
            shdisp_pm_psals_deinit,
            shdisp_pm_psals_ps_deinit_als_off,
            NULL,
            shdisp_pm_psals_ps_deinit_als_on
        }
    },
    {
        SHDISP_DEV_TYPE_ALS, SHDISP_DEV_REQ_ON,
        {
            shdisp_pm_psals_als_init_ps_off,
            shdisp_pm_psals_als_init_ps_off,
            shdisp_pm_psals_als_init_ps_on,
            NULL,
            NULL
        }
    },
    {
        SHDISP_DEV_TYPE_ALS, SHDISP_DEV_REQ_OFF,
        {
            NULL,
            shdisp_pm_psals_deinit,
            NULL,
            shdisp_pm_psals_als_deinit_ps_off,
            shdisp_pm_psals_als_deinit_ps_on
        }
    }
};

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_pm_API_init                                                        */
/* ------------------------------------------------------------------------- */
void shdisp_pm_API_init(struct shdisp_boot_context *shdisp_boot_ctx)
{
    memcpy(&(shdisp_pm_ctx.bdic_status), &(shdisp_boot_ctx->bdic_status), sizeof(struct shdisp_bdic_status));
    memcpy(&(shdisp_pm_ctx.psals_status), &(shdisp_boot_ctx->psals_status), sizeof(struct shdisp_psals_status));

    SHDISP_DEBUG("shdisp_pm_ctx.psals_status.power_status=%d", shdisp_pm_ctx.psals_status.power_status);
    if (shdisp_pm_ctx.psals_status.power_status != SHDISP_SENSOR_STATE_POWER_OFF) {
        shdisp_API_psals_recovery_subscribe();
    }
    shdisp_pm_ctx_recovery.bdic_status.power_status  = SHDISP_DEV_STATE_NOINIT;
    shdisp_pm_ctx_recovery.bdic_status.users         = SHDISP_DEV_TYPE_NONE;

    shdisp_pm_ctx_recovery.psals_status.power_status = SHDISP_SENSOR_STATE_POWER_OFF;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_API_bdic_power_manager                                          */
/* ------------------------------------------------------------------------- */
int shdisp_pm_API_bdic_power_manager(int user, int onoff)
{
    int ret;
    unsigned long users_wk;

    SHDISP_TRACE("in  [BDIC_PM]  users:0x%08X, power_status:%s, user:0x%08X, onoff:%s",
            (int)shdisp_pm_ctx.bdic_status.users,
            GET_DEV_STATE_STR(shdisp_pm_ctx.bdic_status.power_status),
            user,
            GET_DEV_REQ_STR(onoff));

    if ((user != (user & SHDISP_DEV_TYPE_BDIC_MASK)) || (user == SHDISP_DEV_TYPE_NONE)) {
        SHDISP_ERR("invalid user argument. user:0x%08X", user);
        return SHDISP_RESULT_FAILURE;
    }

    if ((onoff != SHDISP_DEV_REQ_OFF) &&
        (onoff != SHDISP_DEV_REQ_ON)) {
        SHDISP_ERR("invalid onoff argument. onoff:%d", onoff);
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_API_get_bdic_is_exist() != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("bdic does not exist.");
        return SHDISP_RESULT_SUCCESS;
    }

    if (onoff == SHDISP_DEV_REQ_ON) {
        switch (shdisp_pm_ctx.bdic_status.power_status) {
        case SHDISP_DEV_STATE_NOINIT:
        case SHDISP_DEV_STATE_OFF:
            ret = shdisp_pm_bdic_set_active();
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_bdic_set_active.");
                return SHDISP_RESULT_FAILURE;
            }
            shdisp_pm_ctx.bdic_status.power_status = SHDISP_DEV_STATE_ON;
            break;
        }
    } else if (onoff == SHDISP_DEV_REQ_OFF) {
        switch (shdisp_pm_ctx.bdic_status.power_status) {
        case SHDISP_DEV_STATE_ON:
            users_wk = shdisp_pm_ctx.bdic_status.users;
            users_wk &= (unsigned long)(~(user & SHDISP_DEV_TYPE_BDIC_MASK));
            if (users_wk == SHDISP_DEV_TYPE_NONE) {
                ret = shdisp_pm_bdic_set_standby();
                if (ret != SHDISP_RESULT_SUCCESS) {
                    SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_bdic_set_standby.");
                    return SHDISP_RESULT_FAILURE;
                }
                shdisp_pm_ctx.bdic_status.power_status = SHDISP_DEV_STATE_OFF;
            }
            break;
        }
    }

    if (onoff == SHDISP_DEV_REQ_ON) {
        shdisp_pm_ctx.bdic_status.users |= (unsigned long)(user & SHDISP_DEV_TYPE_BDIC_MASK);
    } else if (onoff == SHDISP_DEV_REQ_OFF) {
        shdisp_pm_ctx.bdic_status.users &= (unsigned long)(~(user & SHDISP_DEV_TYPE_BDIC_MASK));
    }


    SHDISP_TRACE("out [BDIC_PM]  users:0x%08X, power_status:%s",
            (int)shdisp_pm_ctx.bdic_status.users,
            GET_DEV_STATE_STR(shdisp_pm_ctx.bdic_status.power_status));

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_API_ps_user_manager                                             */
/* ------------------------------------------------------------------------- */
int shdisp_pm_API_ps_user_manager(int user, int onoff)
{
    int ret;

    SHDISP_TRACE("in     [PS_UM]    ps_um_status:%s, user:0x%08X, onoff:%s",
            GET_PSALS_STATE_STR(shdisp_pm_ctx.psals_status.ps_um_status),
            user,
            GET_DEV_REQ_STR(onoff));

    if ((user != (user & SHDISP_DEV_TYPE_PS_MASK)) || (user == SHDISP_DEV_TYPE_NONE)) {
        SHDISP_ERR("invalid user argument. user:0x%08X", user);
        return SHDISP_RESULT_FAILURE;
    }

    if ((onoff != SHDISP_DEV_REQ_OFF) &&
        (onoff != SHDISP_DEV_REQ_ON)) {
        SHDISP_ERR("invalid onoff argument. onoff:%d", onoff);
        return SHDISP_RESULT_FAILURE;
    }

    if (onoff == SHDISP_DEV_REQ_ON) {
        if (shdisp_pm_ctx.psals_status.ps_um_status == SHDISP_PSALS_STATE_OFF) {
            ret = shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_PS, SHDISP_DEV_REQ_ON);
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_API_bdic_power_manager.");
                return SHDISP_RESULT_FAILURE;
            }

            ret = shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_PS, SHDISP_DEV_REQ_ON);
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_psals_power_manager.");
                return SHDISP_RESULT_FAILURE;
            }

            shdisp_pm_ctx.psals_status.ps_um_status = SHDISP_PSALS_STATE_ON;
        }
    } else if (onoff == SHDISP_DEV_REQ_OFF) {
        if (shdisp_pm_ctx.psals_status.ps_um_status == SHDISP_PSALS_STATE_ON) {
            ret = shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_PS, SHDISP_DEV_REQ_OFF);
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_psals_power_manager.");
                return SHDISP_RESULT_FAILURE;
            }

            ret = shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_PS, SHDISP_DEV_REQ_OFF);
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_API_bdic_power_manager.");
                return SHDISP_RESULT_FAILURE;
            }

            shdisp_pm_ctx.psals_status.ps_um_status = SHDISP_PSALS_STATE_OFF;
        }
    }


    SHDISP_TRACE("out    [PS_UM]    ps_um_status:%s",
            GET_PSALS_STATE_STR(shdisp_pm_ctx.psals_status.ps_um_status));

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_API_als_user_manager                                            */
/* ------------------------------------------------------------------------- */
int shdisp_pm_API_als_user_manager(int user, int onoff)
{
    int ret;
    unsigned long users_wk;

    SHDISP_TRACE("in    [ALS_UM]   als_users:0x%08X, als_um_status:%s, user:0x%08X, onoff:%s",
            (int)shdisp_pm_ctx.psals_status.als_users,
            GET_PSALS_STATE_STR(shdisp_pm_ctx.psals_status.als_um_status),
            user,
            GET_DEV_REQ_STR(onoff));

    if ((user != (user & SHDISP_DEV_TYPE_ALS_MASK)) || (user == SHDISP_DEV_TYPE_NONE)) {
        SHDISP_ERR("invalid user argument. user:0x%08X", user);
        return SHDISP_RESULT_FAILURE;
    }

    if ((onoff != SHDISP_DEV_REQ_INIT) &&
        (onoff != SHDISP_DEV_REQ_OFF) &&
        (onoff != SHDISP_DEV_REQ_ON)) {
        SHDISP_ERR("invalid onoff argument. onoff:%d", onoff);
        return SHDISP_RESULT_FAILURE;
    }

    switch (onoff) {
    case SHDISP_DEV_REQ_INIT:
        switch (shdisp_pm_ctx.psals_status.als_um_status) {
        case SHDISP_PSALS_STATE_OFF:
            ret = shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_ALS, SHDISP_DEV_REQ_ON);
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_API_bdic_power_manager.");
                return SHDISP_RESULT_FAILURE;
            }

            ret = shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS, SHDISP_DEV_REQ_INIT);
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_psals_power_manager.");
                return SHDISP_RESULT_FAILURE;
            }

            shdisp_pm_ctx.psals_status.als_um_status = SHDISP_PSALS_STATE_INIT;
            break;
        }
        break;

    case SHDISP_DEV_REQ_ON:
        switch (shdisp_pm_ctx.psals_status.als_um_status) {
        case SHDISP_PSALS_STATE_OFF:
            ret = shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_ALS, SHDISP_DEV_REQ_ON);
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_API_bdic_power_manager.");
                return SHDISP_RESULT_FAILURE;
            }

        case SHDISP_PSALS_STATE_INIT:
            ret = shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS, SHDISP_DEV_REQ_ON);
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_psals_power_manager.");
                return SHDISP_RESULT_FAILURE;
            }

            shdisp_pm_ctx.psals_status.als_um_status = SHDISP_PSALS_STATE_ON;
            break;
        }
        break;

    case SHDISP_DEV_REQ_OFF:
        switch (shdisp_pm_ctx.psals_status.als_um_status) {
        case SHDISP_PSALS_STATE_INIT:
        case SHDISP_PSALS_STATE_ON:
            users_wk = shdisp_pm_ctx.psals_status.als_users;
            users_wk &= (unsigned long)(~(user & SHDISP_DEV_TYPE_ALS_MASK));
            if (users_wk == SHDISP_DEV_TYPE_NONE) {
                ret = shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS, SHDISP_DEV_REQ_OFF);
                if (ret != SHDISP_RESULT_SUCCESS) {
                    SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_psals_power_manager.");
                    return SHDISP_RESULT_FAILURE;
                }

                ret = shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_ALS, SHDISP_DEV_REQ_OFF);
                if (ret != SHDISP_RESULT_SUCCESS) {
                    SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_API_bdic_power_manager.");
                    return SHDISP_RESULT_FAILURE;
                }

                shdisp_pm_ctx.psals_status.als_um_status = SHDISP_PSALS_STATE_OFF;
            }
            break;
        }
        break;
    }

    switch (onoff) {
    case SHDISP_DEV_REQ_INIT:
    case SHDISP_DEV_REQ_ON:
        shdisp_pm_ctx.psals_status.als_users |= (unsigned long)(user & SHDISP_DEV_TYPE_ALS_MASK);
        break;

    case SHDISP_DEV_REQ_OFF:
        shdisp_pm_ctx.psals_status.als_users &= (unsigned long)(~(user & SHDISP_DEV_TYPE_ALS_MASK));
        break;
    }


    SHDISP_TRACE("out   [ALS_UM]   als_users:0x%08X, als_um_status:%s",
            (int)shdisp_pm_ctx.psals_status.als_users,
            GET_PSALS_STATE_STR(shdisp_pm_ctx.psals_status.als_um_status));

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_API_bdic_shutdown                                               */
/* ------------------------------------------------------------------------- */
void shdisp_pm_API_bdic_shutdown(void)
{
    SHDISP_TRACE("in");

    shdisp_pm_ctx_recovery.bdic_status.power_status = shdisp_pm_ctx.bdic_status.power_status;
    shdisp_pm_ctx_recovery.bdic_status.users        = shdisp_pm_ctx.bdic_status.users;

    shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_BDIC_MASK, SHDISP_DEV_REQ_OFF);

    shdisp_SYS_API_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_API_delay_us(15000);

    shdisp_pm_ctx.bdic_status.power_status = SHDISP_DEV_STATE_NOINIT;

    SHDISP_TRACE("out");

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_API_bdic_resume                                                 */
/* ------------------------------------------------------------------------- */
void shdisp_pm_API_bdic_resume(void)
{
    SHDISP_TRACE("in");

    shdisp_SYS_API_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_API_delay_us(1000);

    if (shdisp_pm_ctx_recovery.bdic_status.users != SHDISP_DEV_TYPE_NONE) {
        shdisp_pm_API_bdic_power_manager(shdisp_pm_ctx_recovery.bdic_status.users, SHDISP_DEV_REQ_ON);
    }

    shdisp_pm_ctx_recovery.bdic_status.power_status = SHDISP_DEV_STATE_NOINIT;
    shdisp_pm_ctx_recovery.bdic_status.users        = SHDISP_DEV_TYPE_NONE;

    SHDISP_TRACE("out");

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_API_psals_error_power_off                                       */
/* ------------------------------------------------------------------------- */
void shdisp_pm_API_psals_error_power_off(void)
{
    SHDISP_TRACE("in");

    shdisp_pm_ctx_recovery.psals_status.power_status = shdisp_pm_ctx.psals_status.power_status;

    shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS, SHDISP_DEV_REQ_OFF);
    shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_PS,  SHDISP_DEV_REQ_OFF);

    SHDISP_TRACE("out");

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_API_psals_error_power_recovery                                  */
/* ------------------------------------------------------------------------- */
int shdisp_pm_API_psals_error_power_recovery(void)
{
    int ret;

    SHDISP_TRACE("in");

    if ((shdisp_pm_ctx_recovery.psals_status.power_status == SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF) ||
        (shdisp_pm_ctx_recovery.psals_status.power_status == SHDISP_SENSOR_STATE_PROX_ON_ALS_ON)) {
        ret = shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_PS, SHDISP_DEV_REQ_ON);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("out ret = SHDISP_RESULT_FAILURE");
            return SHDISP_RESULT_FAILURE;
        }
    }

    if ((shdisp_pm_ctx_recovery.psals_status.power_status == SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON) ||
        (shdisp_pm_ctx_recovery.psals_status.power_status == SHDISP_SENSOR_STATE_PROX_ON_ALS_ON)) {
        ret = shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS, SHDISP_DEV_REQ_ON);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("out ret = SHDISP_RESULT_FAILURE");
            return SHDISP_RESULT_FAILURE;
        }
    }

    shdisp_pm_ctx_recovery.psals_status.power_status = SHDISP_SENSOR_STATE_POWER_OFF;

    SHDISP_TRACE("out ret = SHDISP_RESULT_SUCCESS");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_API_is_ps_active                                                */
/* ------------------------------------------------------------------------- */
int shdisp_pm_API_is_ps_active(void)
{
    if (shdisp_pm_ctx.psals_status.ps_um_status == SHDISP_PSALS_STATE_ON) {
        return SHDISP_DEV_STATE_ON;
    } else {
        return SHDISP_DEV_STATE_OFF;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_API_is_als_active                                               */
/* ------------------------------------------------------------------------- */
int shdisp_pm_API_is_als_active(void)
{
    if (shdisp_pm_ctx.psals_status.als_um_status == SHDISP_PSALS_STATE_ON) {
        return SHDISP_DEV_STATE_ON;
    } else {
        return SHDISP_DEV_STATE_OFF;
    }
}

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_pm_API_power_manager_users_dump                                    */
/* ------------------------------------------------------------------------- */
void shdisp_pm_API_power_manager_users_dump(void)
{
    printk("[SHDISP] [BDIC_PM]  users:0x%08X, power_status:%s\n",
            (int)shdisp_pm_ctx.bdic_status.users,
            GET_DEV_STATE_STR(shdisp_pm_ctx.bdic_status.power_status));

    printk("[SHDISP] [PSALS_PM] power_status(power/ps/als):%s\n",
            GET_SENSOR_STATE_STR(shdisp_pm_ctx.psals_status.power_status));

    printk("[SHDISP] [PS_UM]    ps_um_status:%s\n",
            GET_PSALS_STATE_STR(shdisp_pm_ctx.psals_status.ps_um_status));

    printk("[SHDISP] [ALS_UM]   als_users:0x%08X, als_um_status:%s\n",
            (int)shdisp_pm_ctx.psals_status.als_users,
            GET_PSALS_STATE_STR(shdisp_pm_ctx.psals_status.als_um_status));

    return;
}
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_power_manager                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_power_manager(int user, int onoff)
{
    int ret;
    int (* func)(void);
    int next_status;
    int curr_status = shdisp_pm_ctx.psals_status.power_status;
    const struct shdisp_pm_psals_state_row *state_row = NULL;

    SHDISP_TRACE("in     [PSALS_PM] power_status(power/ps/als):%s, user:0x%08X, onoff:%s",
            GET_SENSOR_STATE_STR(shdisp_pm_ctx.psals_status.power_status),
            user,
            GET_DEV_REQ_STR(onoff));

    if ((user != SHDISP_DEV_TYPE_PS) &&
        (user != SHDISP_DEV_TYPE_ALS)) {
        SHDISP_ERR("invalid user argument. user:0x%08X", user);
        return SHDISP_RESULT_FAILURE;
    }

    if ((onoff != SHDISP_DEV_REQ_INIT) &&
        (onoff != SHDISP_DEV_REQ_OFF) &&
        (onoff != SHDISP_DEV_REQ_ON)) {
        SHDISP_ERR("invalid onoff argument. onoff:%d", onoff);
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_API_get_bdic_is_exist() != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("bdic does not exist.");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_pm_ctx.bdic_status.power_status != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("bdic is not active.");
        return SHDISP_RESULT_FAILURE;
    }

    state_row = shdisp_pm_psals_find_state_table(user, onoff);
    if (state_row == NULL) {
        SHDISP_ERR("invalid arguments. user:0x%08X onoff:%d", user, onoff);
        return SHDISP_RESULT_FAILURE;
    }

    if (curr_status == SHDISP_SENSOR_STATE_POWER_OFF) {
        switch (onoff) {
        case SHDISP_DEV_REQ_INIT:
        case SHDISP_DEV_REQ_ON:
            ret = shdisp_pm_psals_power_on();
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_psals_power_on.");
                return SHDISP_RESULT_FAILURE;
            }
            break;
        }
    }

    func = state_row->funcs[curr_status];
    if (func != NULL) {
        next_status = func();
    } else {
        next_status = curr_status;
    }

    if (next_status != curr_status) {
        if (next_status == SHDISP_SENSOR_STATE_POWER_OFF) {
            ret = shdisp_pm_psals_power_off();
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_psals_power_off.");
                return SHDISP_RESULT_FAILURE;
            }
        }
    }

    shdisp_pm_ctx.psals_status.power_status = next_status;


    SHDISP_TRACE("out    [PSALS_PM] power_status(power/ps/als):%s",
            GET_SENSOR_STATE_STR(shdisp_pm_ctx.psals_status.power_status));

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_find_state_table                                          */
/* ------------------------------------------------------------------------- */
static const struct shdisp_pm_psals_state_row *shdisp_pm_psals_find_state_table(int user, int onoff)
{
    int i;
    const struct shdisp_pm_psals_state_row *state_row = NULL;

    for (i = 0; i < ARRAY_SIZE(shdisp_pm_psals_state_table); i++) {
        if (((shdisp_pm_psals_state_table[i].user & user) == user) &&
             (shdisp_pm_psals_state_table[i].onoff        == onoff)) {
            state_row = &shdisp_pm_psals_state_table[i];
            break;
        }
    }

    return state_row;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_set_active                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_bdic_set_active(void)
{
    int ret;

    SHDISP_TRACE("in bdic_status:%d", shdisp_pm_ctx.bdic_status.power_status);

    ret = shdisp_bdic_API_set_active(shdisp_pm_ctx.bdic_status.power_status);

    SHDISP_TRACE("out ret=%d", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_set_standby                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_bdic_set_standby(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_set_standby();

    SHDISP_TRACE("out ret=%d", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_power_on                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_power_on(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_psals_power_on();
    if (ret == SHDISP_RESULT_SUCCESS) {
        if (shdisp_pm_ctx_recovery.psals_status.power_status == SHDISP_SENSOR_STATE_POWER_OFF) {
            shdisp_API_psals_recovery_subscribe();
        }
    } else {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_psals_power_on.");
    }

    SHDISP_TRACE("out ret=%d", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_power_off                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_power_off(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_psals_power_off();
    if (ret == SHDISP_RESULT_SUCCESS) {
        if (shdisp_pm_ctx_recovery.psals_status.power_status == SHDISP_SENSOR_STATE_POWER_OFF) {
            shdisp_API_psals_recovery_unsubscribe();
        }
    } else {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_psals_power_off.");
    }


    SHDISP_TRACE("out ret=%d", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_init                                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_init(void)
{
    SHDISP_TRACE("in");

    SHDISP_TRACE("out");

    return SHDISP_SENSOR_STATE_POWER_ON;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_deinit                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_deinit(void)
{
    SHDISP_TRACE("in");

    SHDISP_TRACE("out");

    return SHDISP_SENSOR_STATE_POWER_OFF;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_ps_init_als_off                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_ps_init_als_off(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_psals_ps_init_als_off();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_psals_ps_init_als_off.");
    }

    SHDISP_TRACE("out");

    return SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_ps_init_als_on                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_ps_init_als_on(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_psals_ps_init_als_on();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_psals_ps_init_als_on.");
    }

    SHDISP_TRACE("out");

    return SHDISP_SENSOR_STATE_PROX_ON_ALS_ON;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_ps_deinit_als_off                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_ps_deinit_als_off(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_psals_ps_deinit_als_off();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_psals_ps_deinit_als_off.");
    }

    SHDISP_TRACE("out");

    return SHDISP_SENSOR_STATE_POWER_OFF;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_ps_deinit_als_on                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_ps_deinit_als_on(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_psals_ps_deinit_als_on();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_psals_ps_deinit_als_on.");
    }

    SHDISP_TRACE("out");

    return SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_als_init_ps_off                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_als_init_ps_off(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_psals_als_init_ps_off();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_psals_als_init_ps_off.");
    }

    SHDISP_TRACE("out");

    return SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_als_init_ps_on                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_als_init_ps_on(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_psals_als_init_ps_on();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_psals_als_init_ps_on.");
    }

    SHDISP_TRACE("out");

    return SHDISP_SENSOR_STATE_PROX_ON_ALS_ON;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_als_deinit_ps_off                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_als_deinit_ps_off(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_psals_als_deinit_ps_off();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_psals_als_deinit_ps_off.");
    }

    SHDISP_TRACE("out");

    return SHDISP_SENSOR_STATE_POWER_OFF;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_als_deinit_ps_on                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_als_deinit_ps_on(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_psals_als_deinit_ps_on();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_psals_als_deinit_ps_on.");
    }

    SHDISP_TRACE("out");

    return SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF;
}

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
