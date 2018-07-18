/* drivers/sharp/shdisp/shdisp_system.c  (Display Driver)
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
#include <linux/spi/spi.h>
#include <mach/gpio.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/qpnp/qpnp-api.h>
#include <sharp/sh_smem.h>
#include "shdisp_system.h"
#include "shdisp_dbg.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_INT_FLAGS (IRQF_TRIGGER_LOW | IRQF_DISABLED)

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_host_gpio_request(int num);
static int shdisp_host_gpio_free(int num);

static int  shdisp_bdic_i2c_probe(struct i2c_client *client, const struct i2c_device_id * devid);
static int  shdisp_bdic_i2c_remove(struct i2c_client *client);

static int  shdisp_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id * devid);
static int  shdisp_sensor_i2c_remove(struct i2c_client *client);

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
typedef struct bdic_data_tag
{
    struct i2c_client *this_client;
} bdic_i2c_data_t;

typedef struct sensor_data_tag {
    struct i2c_client *this_client;
} sensor_data_t;

#ifdef CONFIG_OF
static const struct of_device_id shdisp_system_bdic_dt_match[] = {
    { .compatible = SHDISP_BDIC_I2C_DEVNAME, },
    {}
};
#endif /* CONFIG_OF */

static const struct i2c_device_id shdisp_bdic_id[] = {
    { SHDISP_BDIC_I2C_DEVNAME, 0 },
    { }
};

static struct i2c_driver bdic_driver =
{
    .driver = {
        .owner   = THIS_MODULE,
        .name    = SHDISP_BDIC_I2C_DEVNAME,
#ifdef CONFIG_OF
        .of_match_table = shdisp_system_bdic_dt_match,
#endif /* CONFIG_OF */
    },
    .class    = I2C_CLASS_HWMON,
    .probe    = shdisp_bdic_i2c_probe,
    .id_table = shdisp_bdic_id,
    .remove   = shdisp_bdic_i2c_remove,
};

#ifdef CONFIG_OF
static const struct of_device_id shdisp_system_sensor_dt_match[] = {
    { .compatible = SHDISP_SENSOR_DEVNAME, },
    {}
};
#endif /* CONFIG_OF */

static const struct i2c_device_id shdisp_sensor_id[] = {
    { SHDISP_SENSOR_DEVNAME, 0 },
    { }
};

static struct i2c_driver sensor_driver =
{
    .driver = {
        .owner   = THIS_MODULE,
        .name    = SHDISP_SENSOR_DEVNAME,
#ifdef CONFIG_OF
        .of_match_table = shdisp_system_sensor_dt_match,
#endif /* CONFIG_OF */
    },
    .class    = I2C_CLASS_HWMON,
    .probe    = shdisp_sensor_i2c_probe,
    .id_table = shdisp_sensor_id,
    .remove   = shdisp_sensor_i2c_remove,
};

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static bdic_i2c_data_t *bdic_i2c_p = NULL;

static sensor_data_t   *sensor_data_p = NULL;

static unsigned int shdisp_int_irq_port = 0;
static struct platform_device *shdisp_int_irq_port_pdev = NULL;
static int shdisp_int_irq_port_staus = 0;
static spinlock_t shdisp_set_irq_spinlock;


/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

#ifdef SHDISP_SYS_SW_TIME_API
static void shdisp_dbg_wait_start(void);
static void shdisp_dbg_wait_end(unsigned long usec);
#define SHDISP_SYS_DBG_API_WAIT_START           shdisp_dbg_wait_start();
#define SHDISP_SYS_DBG_API_WAIT_END(usec)       shdisp_dbg_wait_end(usec);
struct shdisp_sys_dbg_api_info {
    int flag;
    struct timespec t_api_start;
    struct timespec t_wait_start;
    struct timespec t_wait_req;
    struct timespec t_wait_sum;
};
static struct shdisp_sys_dbg_api_info shdisp_sys_dbg_api;
#ifdef SHDISP_SYS_SW_TIME_BDIC
static void shdisp_dbg_bdic_init(void);
static void shdisp_dbg_bdic_logout(void);
static void shdisp_dbg_bdic_singl_write_start(void);
static void shdisp_dbg_bdic_singl_write_retry(void);
static void shdisp_dbg_bdic_singl_write_end(int ret);
static void shdisp_dbg_bdic_singl_read_start(void);
static void shdisp_dbg_bdic_singl_read_retry(void);
static void shdisp_dbg_bdic_singl_read_end(int ret);
static void shdisp_dbg_bdic_multi_read_start(void);
static void shdisp_dbg_bdic_multi_read_retry(void);
static void shdisp_dbg_bdic_multi_read_end(int ret);
static void shdisp_dbg_bdic_multi_write_start(void);
static void shdisp_dbg_bdic_multi_write_retry(void);
static void shdisp_dbg_bdic_multi_write_end(int ret);
#define SHDISP_SYS_DBG_DBIC_INIT                shdisp_dbg_bdic_init();
#define SHDISP_SYS_DBG_DBIC_LOGOUT              shdisp_dbg_bdic_logout();
#define SHDISP_SYS_DBG_DBIC_SINGL_W_START       shdisp_dbg_bdic_singl_write_start();
#define SHDISP_SYS_DBG_DBIC_SINGL_W_RETRY       shdisp_dbg_bdic_singl_write_retry();
#define SHDISP_SYS_DBG_DBIC_SINGL_W_END(ret)    shdisp_dbg_bdic_singl_write_end(ret);
#define SHDISP_SYS_DBG_DBIC_SINGL_R_START       shdisp_dbg_bdic_singl_read_start();
#define SHDISP_SYS_DBG_DBIC_SINGL_R_RETRY       shdisp_dbg_bdic_singl_read_retry();
#define SHDISP_SYS_DBG_DBIC_SINGL_R_END(ret)    shdisp_dbg_bdic_singl_read_end(ret);
#define SHDISP_SYS_DBG_DBIC_MULTI_R_START       shdisp_dbg_bdic_multi_read_start();
#define SHDISP_SYS_DBG_DBIC_MULTI_R_RETRY       shdisp_dbg_bdic_multi_read_retry();
#define SHDISP_SYS_DBG_DBIC_MULTI_R_END(ret)    shdisp_dbg_bdic_multi_read_end(ret);
#define SHDISP_SYS_DBG_DBIC_MULTI_W_START       shdisp_dbg_bdic_multi_write_start();
#define SHDISP_SYS_DBG_DBIC_MULTI_W_RETRY       shdisp_dbg_bdic_multi_write_retry();
#define SHDISP_SYS_DBG_DBIC_MULTI_W_END(ret)    shdisp_dbg_bdic_multi_write_end(ret);
struct shdisp_sys_dbg_i2c_rw_info {
    unsigned long   w_singl_ok_count;
    unsigned long   w_singl_ng_count;
    unsigned long   w_singl_retry;
    struct timespec w_singl_t_start;
    struct timespec w_singl_t_sum;
    unsigned long   r_singl_ok_count;
    unsigned long   r_singl_ng_count;
    unsigned long   r_singl_retry;
    struct timespec r_singl_t_start;
    struct timespec r_singl_t_sum;
    unsigned long   r_multi_ok_count;
    unsigned long   r_multi_ng_count;
    unsigned long   r_multi_retry;
    struct timespec r_multi_t_start;
    struct timespec r_multi_t_sum;
    unsigned long   w_multi_ok_count;
    unsigned long   w_multi_ng_count;
    unsigned long   w_multi_retry;
    struct timespec w_multi_t_start;
    struct timespec w_multi_t_sum;
};
static struct shdisp_sys_dbg_i2c_rw_info shdisp_sys_dbg_bdic;
#else  /* SHDISP_SYS_SW_TIME_BDIC */
#define SHDISP_SYS_DBG_DBIC_INIT
#define SHDISP_SYS_DBG_DBIC_LOGOUT
#define SHDISP_SYS_DBG_DBIC_SINGL_W_START
#define SHDISP_SYS_DBG_DBIC_SINGL_W_RETRY
#define SHDISP_SYS_DBG_DBIC_SINGL_W_END(ret)
#define SHDISP_SYS_DBG_DBIC_SINGL_R_START
#define SHDISP_SYS_DBG_DBIC_SINGL_R_RETRY
#define SHDISP_SYS_DBG_DBIC_SINGL_R_END(ret)
#define SHDISP_SYS_DBG_DBIC_MULTI_R_START
#define SHDISP_SYS_DBG_DBIC_MULTI_R_RETRY
#define SHDISP_SYS_DBG_DBIC_MULTI_R_END(ret)
#define SHDISP_SYS_DBG_DBIC_MULTI_W_START
#define SHDISP_SYS_DBG_DBIC_MULTI_W_RETRY
#define SHDISP_SYS_DBG_DBIC_MULTI_W_END(ret)
#endif /* SHDISP_SYS_SW_TIME_BDIC */
#else  /* SHDISP_SYS_SW_TIME_API */
#define SHDISP_SYS_DBG_API_WAIT_START
#define SHDISP_SYS_DBG_API_WAIT_END(usec)
#define SHDISP_SYS_DBG_DBIC_INIT
#define SHDISP_SYS_DBG_DBIC_SINGL_W_START
#define SHDISP_SYS_DBG_DBIC_SINGL_W_RETRY
#define SHDISP_SYS_DBG_DBIC_SINGL_W_END(ret)
#define SHDISP_SYS_DBG_DBIC_SINGL_R_START
#define SHDISP_SYS_DBG_DBIC_SINGL_R_RETRY
#define SHDISP_SYS_DBG_DBIC_SINGL_R_END(ret)
#define SHDISP_SYS_DBG_DBIC_MULTI_R_START
#define SHDISP_SYS_DBG_DBIC_MULTI_R_RETRY
#define SHDISP_SYS_DBG_DBIC_MULTI_R_END(ret)
#define SHDISP_SYS_DBG_DBIC_MULTI_W_START
#define SHDISP_SYS_DBG_DBIC_MULTI_W_RETRY
#define SHDISP_SYS_DBG_DBIC_MULTI_W_END(ret)
#endif /* SHDISP_SYS_SW_TIME_API */

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_control                                               */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_Host_control(int cmd, unsigned long rate)
{
    int ret = SHDISP_RESULT_SUCCESS;

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_delay_us                                                   */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_delay_us(unsigned long usec)
{
    struct timespec tu;

    if (usec >= 1000 * 1000) {
        tu.tv_sec  = usec / 1000000;
        tu.tv_nsec = (usec % 1000000) * 1000;
    } else {
        tu.tv_sec  = 0;
        tu.tv_nsec = usec * 1000;
    }

    SHDISP_SYS_DBG_API_WAIT_START;

    hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);

    SHDISP_SYS_DBG_API_WAIT_END(usec);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_gpio_init                                             */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_Host_gpio_init(void)
{
    shdisp_host_gpio_request(SHDISP_GPIO_NUM_BL_RST_N);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_gpio_exit                                             */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_Host_gpio_exit(void)
{
    shdisp_host_gpio_free(SHDISP_GPIO_NUM_BL_RST_N);
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_gpio_request                                          */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_Host_gpio_request(int num)
{
    return shdisp_host_gpio_request(num);
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_gpio_free                                             */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_Host_gpio_free(int num)
{
    return shdisp_host_gpio_free(num);
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_set_Host_gpio                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_set_Host_gpio(int num, int value)
{
    if (value != SHDISP_GPIO_CTL_LOW &&
        value != SHDISP_GPIO_CTL_HIGH) {
        SHDISP_ERR("<INVALID_VALUE> value(%d).", value);
        return SHDISP_RESULT_FAILURE;
    }

    switch (num) {
    case SHDISP_GPIO_NUM_BL_RST_N:
    case SHDISP_GPIO_NUM_ANDY_VDD:
        SHDISP_DEBUG("gpio set num=%d value=%d", num, value);
        gpio_set_value(num, value);
        return SHDISP_RESULT_SUCCESS;
    default:
        SHDISP_ERR("<INVALID_VALUE> num(%d).", num);
        return SHDISP_RESULT_FAILURE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_get_Host_gpio                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_get_Host_gpio(int num)
{
    return gpio_get_value(num);
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_check_upper_unit                                           */
/* ------------------------------------------------------------------------- */
int shdisp_SYS_API_check_upper_unit(void)
{
    int val;
    SHDISP_TRACE("in");
    gpio_request(SHDISP_GPIO_NUM_UPPER_UNIT, "upper_unit");

    gpio_tlmm_config(GPIO_CFG(SHDISP_GPIO_NUM_UPPER_UNIT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
                                                                                               GPIO_CFG_ENABLE);
    shdisp_SYS_API_delay_us(50);
    val = gpio_get_value(SHDISP_GPIO_NUM_UPPER_UNIT);
    gpio_tlmm_config(GPIO_CFG(SHDISP_GPIO_NUM_UPPER_UNIT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
                                                                                                 GPIO_CFG_ENABLE);

    gpio_free(SHDISP_GPIO_NUM_UPPER_UNIT);
    SHDISP_DEBUG("check_upper_unit val=%d", val);

    if (!val) {
        SHDISP_ERR("<OTHER> Upper unit does not exist.");
        return SHDISP_RESULT_FAILURE;
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_set_irq_port                                               */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_set_irq_port(int irq_port, struct platform_device *pdev)
{
    shdisp_int_irq_port = irq_port;
    shdisp_int_irq_port_pdev = pdev;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_request_irq                                                */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_request_irq(irqreturn_t (*irq_handler)( int , void * ) )
{
    int ret = SHDISP_RESULT_SUCCESS;
    if ((irq_handler == NULL)
    ||  (shdisp_int_irq_port_pdev == NULL)) {
        return SHDISP_RESULT_FAILURE;
    }

    ret = devm_request_irq(&shdisp_int_irq_port_pdev->dev, shdisp_int_irq_port, *irq_handler,
                        SHDISP_INT_FLAGS,   "shdisp", NULL);

    if (ret == 0) {
        disable_irq(shdisp_int_irq_port);
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_free_irq                                                   */
/* ------------------------------------------------------------------------- */
void  shdisp_SYS_API_free_irq(void)
{
    free_irq(shdisp_int_irq_port, NULL);
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_set_irq_init                                               */
/* ------------------------------------------------------------------------- */
void  shdisp_SYS_API_set_irq_init(void)
{
    spin_lock_init( &shdisp_set_irq_spinlock );
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_set_irq                                                    */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_set_irq( int enable )
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned long flags = 0;

    spin_lock_irqsave( &shdisp_set_irq_spinlock, flags);

    if (enable == shdisp_int_irq_port_staus) {
        spin_unlock_irqrestore( &shdisp_set_irq_spinlock, flags);
        return SHDISP_RESULT_SUCCESS;
    }

    if (enable == SHDISP_IRQ_ENABLE) {
        enable_irq_wake(shdisp_int_irq_port);
        enable_irq(shdisp_int_irq_port);
        shdisp_int_irq_port_staus = enable;
    } else if (enable == SHDISP_IRQ_DISABLE) {
        disable_irq_nosync(shdisp_int_irq_port);
        disable_irq_wake(shdisp_int_irq_port);
        shdisp_int_irq_port_staus = enable;
    } else {
        SHDISP_ERR("<INVALID_VALUE> enable=%d", enable);
        ret = SHDISP_RESULT_FAILURE;
    }
    spin_unlock_irqrestore( &shdisp_set_irq_spinlock, flags);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_panel_external_clk_ctl     depends on BATTERY_SH           */
/* ------------------------------------------------------------------------- 
int shdisp_SYS_API_panel_external_clk_ctl(int enable)
{
    int ret = 0;

    SHDISP_TRACE("in enable=%d", enable);
    if (enable) {
        ret = qpnp_bbclk2_control_enable(true);
    } else {
        ret = qpnp_bbclk2_control_enable(false);
    }

    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> ret=%d", ret);
        ret = SHDISP_RESULT_FAILURE;
    } else {
        ret = SHDISP_RESULT_SUCCESS;
    }
    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}
*/
/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_i2c_send                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_Host_i2c_send(unsigned char slaveaddr, unsigned char *sendval, unsigned char size)
{
    struct i2c_adapter *adap;
    struct i2c_msg msg;
    int i2c_ret;
    int result = 1;
    int retry;
#ifdef SHDISP_LOG_ENABLE
    int i;
    char logbuf[512], work[16];
#endif /* SHDISP_LOG_ENABLE */

    if (slaveaddr == sensor_data_p->this_client->addr) {
        adap = sensor_data_p->this_client->adapter;
    } else {
        SHDISP_ERR("<OTHER> slaveaddr(0x%02x) device nothing.", slaveaddr);
        return SHDISP_RESULT_FAILURE;
    }

    if (sendval == NULL) {
        SHDISP_ERR("<NULL_POINTER> data.");
        return SHDISP_RESULT_FAILURE;
    }


    memset(&msg, 0, sizeof(msg));
    msg.addr     = slaveaddr;
    msg.flags    = 0;
    msg.len      = size;
    msg.buf      = sendval;


    for (retry = 0; retry <= 10; retry++) {
        i2c_ret = i2c_transfer(adap, &msg, 1);
        if (i2c_ret > 0) {
            result = 0;
            break;
        } else {
        }
    }


#ifdef SHDISP_LOG_ENABLE
    memset(logbuf, 0x00, sizeof(logbuf));
    for (i = 0; i < size; i++) {
        sprintf(work, "%02X", msg.buf[i]);
        strcat(logbuf, work);
    }
    SHDISP_I2CLOG("slaveaddr=0x%02X, sendval=0x%s, size=%d", slaveaddr, logbuf, size);
#endif /* SHDISP_LOG_ENABLE */
    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(slaveaddr = 0x%02x, i2c_ret = %d).", slaveaddr, i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_i2c_recv                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_Host_i2c_recv(unsigned char slaveaddr, unsigned char *sendval, unsigned char sendsize,
                                   unsigned char *recvval, unsigned char recvsize)
{
    struct i2c_adapter *adap;
    struct i2c_msg msg[2];
    int i2c_ret;
    int result = 1;
    int retry;
#ifdef SHDISP_LOG_ENABLE
    int i;
    char logbuf[512], work[16];
#endif /* SHDISP_LOG_ENABLE */

    if (slaveaddr == sensor_data_p->this_client->addr) {
        adap = sensor_data_p->this_client->adapter;
    } else {
        SHDISP_ERR("<OTHER> slaveaddr(0x%02x) device nothing.", slaveaddr);
        return SHDISP_RESULT_FAILURE;
    }

    if ((sendval == NULL) || (recvval == NULL)) {
        SHDISP_ERR("<NULL_POINTER> data.");
        return SHDISP_RESULT_FAILURE;
    }

    memset(msg, 0, sizeof(*msg) * ARRAY_SIZE(msg));
    msg[0].addr     = slaveaddr;
    msg[0].flags    = 0;
    msg[0].len      = sendsize;
    msg[0].buf      = sendval;

    msg[1].addr  = slaveaddr;
    msg[1].flags = I2C_M_RD;
    msg[1].len   = recvsize;
    msg[1].buf   = recvval;

    for (retry = 0; retry <= 10; retry++) {

        i2c_ret = i2c_transfer(adap, msg, 2);

        if (i2c_ret > 0) {
            result = 0;
            break;
        } else {
        }
    }

#ifdef SHDISP_LOG_ENABLE
    memset(logbuf, 0x00, sizeof(logbuf));
    for (i = 0; i < sendsize; i++) {
        sprintf(work, "%02X", msg[0].buf[i]);
        strcat(logbuf, work);
    }
    SHDISP_I2CLOG("msg[0]: slaveaddr=0x%02X, sendval=0x%s, size=%d", slaveaddr, logbuf, sendsize);
    memset(logbuf, 0x00, sizeof(logbuf));
    for (i = 0; i < recvsize; i++) {
        sprintf(work, "%02X", msg[1].buf[i]);
        strcat(logbuf, work);
    }
    SHDISP_I2CLOG("msg[1]: slaveaddr=0x%02X, recvval=0x%s, size=%d", slaveaddr, logbuf, recvsize);
#endif /* SHDISP_LOG_ENABLE */
    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_init                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_init(void)
{
    int ret;

    ret = i2c_add_driver(&bdic_driver);
    if (ret < 0) {
        SHDISP_ERR("<RESULT_FAILURE> i2c_add_driver.");
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_exit                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_exit(void)
{
    i2c_del_driver(&bdic_driver);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_write                                             */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_write(unsigned char addr, unsigned char data)
{
    struct i2c_msg msg;
    unsigned char write_buf[2];
    int i2c_ret;
    int result = 1;
    int retry;

    SHDISP_SYS_DBG_DBIC_SINGL_W_START;
    SHDISP_I2CLOG("(addr=0x%02X, data=0x%02X)", addr, data);

    msg.addr     = bdic_i2c_p->this_client->addr;
    msg.flags    = 0;
    msg.len      = 2;
    msg.buf      = write_buf;
    write_buf[0] = addr;
    write_buf[1] = data;

    for (retry = 0; retry <= 10; retry++) {
        i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
        if (i2c_ret > 0) {
            result = 0;
            break;
        } else {
            SHDISP_SYS_DBG_DBIC_SINGL_W_RETRY;
        }
    }

    SHDISP_SYS_DBG_DBIC_SINGL_W_END(result);

    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_mask_write                                        */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_mask_write(unsigned char addr, unsigned char data, unsigned char mask)
{
    unsigned char read_data;
    unsigned char write_data;
    int ret;

    SHDISP_I2CLOG("(addr=0x%02X, data=0x%02X, mask=0x%02X)", addr, data, mask);
    ret = shdisp_SYS_API_bdic_i2c_read(addr, &read_data);
    if (ret == SHDISP_RESULT_SUCCESS) {
        write_data = ((read_data & ~mask) | (data & mask));
        if (write_data != read_data) {
            ret =  shdisp_SYS_API_bdic_i2c_write(addr, write_data);
        }
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_multi_write                                       */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_multi_write(unsigned char addr, unsigned char *wval, unsigned char size)
{
    struct i2c_msg msg;
    unsigned char write_buf[21];
    int i2c_ret;
    int result = 1;
    int retry;

    if ((size < 1) || (size > 20)) {
        SHDISP_ERR("<INVALID_VALUE> size(%d).", size);
        return SHDISP_RESULT_FAILURE;
    }

    if (wval == NULL) {
        SHDISP_ERR("<NULL_POINTER> val.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_SYS_DBG_DBIC_MULTI_W_START;

    msg.addr     = bdic_i2c_p->this_client->addr;
    msg.flags    = 0;
    msg.len      = size + 1;
    msg.buf      = write_buf;
    memset(write_buf, 0x00, sizeof(write_buf));
    write_buf[0] = addr;
    memcpy( &write_buf[1], wval, (int)size );
    SHDISP_I2CLOG("(addr=0x%02X, size=0x%02X", addr, size);
    SHDISP_I2CLOG("*wval=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X)",
                                write_buf[1], write_buf[2], write_buf[3], write_buf[4], write_buf[5],
                                write_buf[6], write_buf[7], write_buf[8], write_buf[9], write_buf[10],
                                write_buf[11], write_buf[12], write_buf[13], write_buf[14], write_buf[15],
                                write_buf[16], write_buf[17], write_buf[18], write_buf[19], write_buf[20]);

    for (retry = 0; retry <= 10; retry++) {
        i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
        if (i2c_ret > 0) {
            result = 0;
            break;
        } else {
            SHDISP_SYS_DBG_DBIC_MULTI_W_RETRY;
        }
    }

    SHDISP_SYS_DBG_DBIC_MULTI_W_END(result);

    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_read                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_read(unsigned char addr, unsigned char *data)
{
    struct i2c_msg msg;
    unsigned char write_buf[2];
    unsigned char read_buf[2];
    int i2c_ret;
    int result = 1;
    int retry;

    if (data == NULL) {
        SHDISP_ERR("<NULL_POINTER> data.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_SYS_DBG_DBIC_SINGL_R_START;

    for (retry = 0; retry <= 10; retry++) {
        msg.addr     = bdic_i2c_p->this_client->addr;
        msg.flags    = 0;
        msg.len      = 1;
        msg.buf      = write_buf;
        write_buf[0] = addr;

        i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);

        if (i2c_ret > 0) {

            msg.addr  = bdic_i2c_p->this_client->addr;
            msg.flags = I2C_M_RD;
            msg.len   = 1;
            msg.buf   = read_buf;

            i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
            if (i2c_ret > 0) {
                *data = read_buf[0];
                result = 0;
                break;
            } else {
                SHDISP_SYS_DBG_DBIC_SINGL_R_RETRY;
            }
        } else {
            SHDISP_SYS_DBG_DBIC_SINGL_R_RETRY;
        }
    }

    SHDISP_I2CLOG("(addr=0x%02X, data=0x%02X)", addr, *data);
    SHDISP_SYS_DBG_DBIC_SINGL_R_END(result);

    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_multi_read                                        */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_multi_read(unsigned char addr, unsigned char *data, int size)
{
    struct i2c_msg msg;
    unsigned char write_buf[2];
    unsigned char read_buf[1 + 8];
    int i2c_ret;
    int result = 1;
    int retry;

    if ((size < 1) || (size > 8)) {
        SHDISP_ERR("<INVALID_VALUE> size(%d).", size);
        return SHDISP_RESULT_FAILURE;
    }

    if (data == NULL) {
        SHDISP_ERR("<NULL_POINTER> data.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_SYS_DBG_DBIC_MULTI_R_START;

    msg.addr     = bdic_i2c_p->this_client->addr;
    msg.flags    = 0;
    msg.len      = 1;
    msg.buf      = write_buf;
    write_buf[0] = addr;

    for (retry = 0; retry <= 10; retry++) {
        i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
        if (i2c_ret > 0) {
            result = 0;
            break;
        } else {
            SHDISP_SYS_DBG_DBIC_MULTI_R_RETRY;
        }
    }

    if (result == 0) {
        msg.addr  = bdic_i2c_p->this_client->addr;
        msg.flags = I2C_M_RD;
        msg.len   = size;
        msg.buf   = read_buf;
        memset(read_buf, 0x00, sizeof(read_buf));
        for (retry = 0; retry <= 10; retry++) {
            i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
            if (i2c_ret > 0) {
                memcpy(data, &read_buf[0], size);
                result = 0;
                break;
            } else {
                SHDISP_SYS_DBG_DBIC_MULTI_R_RETRY;
            }
        }
    }

    SHDISP_I2CLOG("(addr=0x%02X, size=0x%02X, *data=%02X%02X%02X%02X%02X%02X%02X%02X)",
                        addr, size,
                        read_buf[0], read_buf[1], read_buf[2], read_buf[3],
                        read_buf[4], read_buf[5], read_buf[6], read_buf[7]);
    SHDISP_SYS_DBG_DBIC_MULTI_R_END(result);

    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_sensor_i2c_init                                            */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_sensor_i2c_init(void)
{
    int ret;

    ret = i2c_add_driver(&sensor_driver);
    if (ret < 0) {
        SHDISP_ERR("<RESULT_FAILURE> i2c_add_driver.");
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_sensor_i2c_exit                                            */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_sensor_i2c_exit(void)
{
    i2c_del_driver(&sensor_driver);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* SUB ROUTINE                                                               */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_host_gpio_request                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_host_gpio_request(int num)
{
    int ret = SHDISP_RESULT_SUCCESS;

    if (num == SHDISP_GPIO_NUM_BL_RST_N) {
        gpio_request(num, "BL_RST_N");
    } else if (num == SHDISP_GPIO_NUM_PANEL_RST_N) {
        gpio_request(num, "PANEL_RST_N");
    } else if (num == SHDISP_GPIO_NUM_CLK_SEL) {
        gpio_request(SHDISP_GPIO_NUM_CLK_SEL, "LCD_CLK_SEL");
#ifdef  SHDISP_TRI_LED2
    } else if (num == SHDISP_GPIO_NUM_SP_SWIC) {
        SHDISP_DEBUG("num(%d).", num);
        gpio_request(SHDISP_GPIO_NUM_SP_SWIC, "SWIC_SEL");
#endif  /* SHDISP_TRI_LED2 */
    } else {
        SHDISP_ERR("<INVALID_VALUE> num(%d).", num);
        ret = SHDISP_RESULT_FAILURE;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_host_gpio_free                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_host_gpio_free(int num)
{
    int ret = SHDISP_RESULT_SUCCESS;

    switch (num) {
    case SHDISP_GPIO_NUM_BL_RST_N:
    case SHDISP_GPIO_NUM_PANEL_RST_N:
    case SHDISP_GPIO_NUM_CLK_SEL:
#ifdef  SHDISP_TRI_LED2
    case SHDISP_GPIO_NUM_SP_SWIC:
        SHDISP_DEBUG("num(%d).", num);
#endif  /* SHDISP_TRI_LED2 */
        gpio_free(num);
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> num(%d).", num);
        ret = SHDISP_RESULT_FAILURE;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_i2c_probe                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_i2c_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
    bdic_i2c_data_t *i2c_p;

    if (bdic_i2c_p != NULL) {
        return -EPERM;
    }

    i2c_p = (bdic_i2c_data_t *)kzalloc(sizeof(bdic_i2c_data_t), GFP_KERNEL);
    if (i2c_p == NULL) {
        return -ENOMEM;
    }

    bdic_i2c_p = i2c_p;

    i2c_set_clientdata(client, i2c_p);
    i2c_p->this_client = client;

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_i2c_remove                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_i2c_remove(struct i2c_client *client)
{
    bdic_i2c_data_t *i2c_p;

    i2c_p = i2c_get_clientdata(client);

    kfree(i2c_p);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sensor_i2c_probe                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
    sensor_data_t *i2c_p;

    if (sensor_data_p != NULL) {
        return -EPERM;
    }

    i2c_p = (sensor_data_t *)kzalloc(sizeof(sensor_data_t), GFP_KERNEL);
    if (i2c_p == NULL) {
        return -ENOMEM;
    }

    sensor_data_p = i2c_p;

    i2c_set_clientdata(client, i2c_p);
    i2c_p->this_client = client;

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sensor_i2c_remove                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_sensor_i2c_remove(struct i2c_client *client)
{
    sensor_data_t *i2c_p;

    i2c_p = i2c_get_clientdata(client);

    kfree(i2c_p);
    sensor_data_p = NULL;

    return SHDISP_RESULT_SUCCESS;
}

#ifdef SHDISP_SYS_SW_TIME_API
/* ------------------------------------------------------------------------- */
/* shdisp_sys_API_dbg_hw_check_start                                         */
/* ------------------------------------------------------------------------- */
void shdisp_sys_API_dbg_hw_check_start(void)
{
    memset(&shdisp_sys_dbg_api, 0, sizeof(shdisp_sys_dbg_api));

    SHDISP_SYS_DBG_DBIC_INIT;

    shdisp_sys_dbg_api.flag = 1;

    getnstimeofday(&shdisp_sys_dbg_api.t_api_start);
}

/* ------------------------------------------------------------------------- */
/* shdisp_sys_API_dbg_hw_check_end                                           */
/* ------------------------------------------------------------------------- */
void shdisp_sys_API_dbg_hw_check_end(const char *func)
{
    struct timespec stop, df;
    u64 msec_api, msec_req, msec_sum;
    u64 usec_api, usec_req, usec_sum;

    getnstimeofday(&stop);

    df = timespec_sub(stop, shdisp_sys_dbg_api.t_api_start);

    msec_api = timespec_to_ns(&df);
    do_div(msec_api, NSEC_PER_USEC);
    usec_api = do_div(msec_api, USEC_PER_MSEC);

    msec_req = timespec_to_ns(&shdisp_sys_dbg_api.t_wait_req);
    do_div(msec_req, NSEC_PER_USEC);
    usec_req = do_div(msec_req, USEC_PER_MSEC);

    msec_sum = timespec_to_ns(&shdisp_sys_dbg_api.t_wait_sum);
    do_div(msec_sum, NSEC_PER_USEC);
    usec_sum = do_div(msec_sum, USEC_PER_MSEC);

    printk(KERN_ERR "[API]%s() total=%lu.%03lums, wait=%lu.%03lums( %lu.%03lums )\n", func,
    (unsigned long)msec_api, (unsigned long)usec_api,
    (unsigned long)msec_sum, (unsigned long)usec_sum,
    (unsigned long)msec_req, (unsigned long)usec_req );

    SHDISP_SYS_DBG_DBIC_LOGOUT;

    shdisp_sys_dbg_api.flag = 0;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_wait_start                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_wait_start(void)
{
    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    getnstimeofday(&shdisp_sys_dbg_api.t_wait_start);
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_wait_end                                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_wait_end(unsigned long usec)
{
    struct timespec stop, df;

    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    getnstimeofday(&stop);

    df = timespec_sub(stop, shdisp_sys_dbg_api.t_wait_start);

    shdisp_sys_dbg_api.t_wait_sum = timespec_add(shdisp_sys_dbg_api.t_wait_sum, df);

    timespec_add_ns(&shdisp_sys_dbg_api.t_wait_req, (usec * 1000));
}

#ifdef SHDISP_SYS_SW_TIME_BDIC
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_init                                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_init(void)
{
    memset(&shdisp_sys_dbg_bdic, 0, sizeof(shdisp_sys_dbg_bdic));
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_logout                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_logout(void)
{
    u64 nsec_wk;
    unsigned long usec_wk1, usec_avl;

    nsec_wk = timespec_to_ns(&shdisp_sys_dbg_bdic.w_singl_t_sum);
    if (nsec_wk != 0) {
        do_div(nsec_wk, NSEC_PER_USEC);
        usec_wk1 = nsec_wk;
        usec_avl = usec_wk1 / shdisp_sys_dbg_bdic.w_singl_ok_count;
        printk(KERN_ERR "[---] -- bdic w_s %lu,%lu,%lu, total=%lu.%03lums, avl=%lu.%03lums\n",
        shdisp_sys_dbg_bdic.w_singl_ok_count, shdisp_sys_dbg_bdic.w_singl_ng_count, shdisp_sys_dbg_bdic.w_singl_retry,
        usec_wk1 / USEC_PER_MSEC, usec_wk1 % USEC_PER_MSEC, usec_avl / USEC_PER_MSEC, usec_avl % USEC_PER_MSEC);
    }

    nsec_wk = timespec_to_ns(&shdisp_sys_dbg_bdic.r_singl_t_sum);
    if (nsec_wk != 0) {
        do_div(nsec_wk, NSEC_PER_USEC);
        usec_wk1 = nsec_wk;
        usec_avl = usec_wk1 / shdisp_sys_dbg_bdic.r_singl_ok_count;
        printk(KERN_ERR "[---] -- bdic r_s %lu,%lu,%lu, total=%lu.%03lums, avl=%lu.%03lums\n",
        shdisp_sys_dbg_bdic.r_singl_ok_count, shdisp_sys_dbg_bdic.r_singl_ng_count, shdisp_sys_dbg_bdic.r_singl_retry,
        usec_wk1 / USEC_PER_MSEC, usec_wk1 % USEC_PER_MSEC, usec_avl / USEC_PER_MSEC, usec_avl % USEC_PER_MSEC);
    }

    nsec_wk = timespec_to_ns(&shdisp_sys_dbg_bdic.r_multi_t_sum);
    if (nsec_wk != 0) {
        do_div(nsec_wk, NSEC_PER_USEC);
        usec_wk1 = nsec_wk;
        usec_avl = usec_wk1 / shdisp_sys_dbg_bdic.r_multi_ok_count;
        printk(KERN_ERR "[---] -- bdic r_m %lu,%lu,%lu, total=%lu.%03lums, avl=%lu.%03lums\n",
        shdisp_sys_dbg_bdic.r_multi_ok_count, shdisp_sys_dbg_bdic.r_multi_ng_count, shdisp_sys_dbg_bdic.r_multi_retry,
        usec_wk1 / USEC_PER_MSEC, usec_wk1 % USEC_PER_MSEC, usec_avl / USEC_PER_MSEC, usec_avl % USEC_PER_MSEC);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_singl_write_start                                         */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_singl_write_start(void)
{
    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    getnstimeofday(&shdisp_sys_dbg_bdic.w_singl_t_start);
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_singl_write_retry                                         */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_singl_write_retry(void)
{
    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    shdisp_sys_dbg_bdic.w_singl_retry++;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_singl_write_end                                           */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_singl_write_end(int ret)
{
    struct timespec stop, df;

    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    getnstimeofday(&stop);

    df = timespec_sub(stop, shdisp_sys_dbg_bdic.w_singl_t_start);

    shdisp_sys_dbg_bdic.w_singl_t_sum = timespec_add(shdisp_sys_dbg_bdic.w_singl_t_sum, df);

    if (ret == 0) {
        shdisp_sys_dbg_bdic.w_singl_ok_count++;
    }
    } else {
        shdisp_sys_dbg_bdic.w_singl_ng_count++;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_singl_read_start                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_singl_read_start(void)
{
    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    getnstimeofday(&shdisp_sys_dbg_bdic.r_singl_t_start);
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_singl_read_retry                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_singl_read_retry(void)
{
    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    shdisp_sys_dbg_bdic.r_singl_retry++;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_singl_read_end                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_singl_read_end(int ret)
{
    struct timespec stop, df;

    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    getnstimeofday(&stop);

    df = timespec_sub(stop, shdisp_sys_dbg_bdic.r_singl_t_start);

    shdisp_sys_dbg_bdic.r_singl_t_sum = timespec_add(shdisp_sys_dbg_bdic.r_singl_t_sum, df);

    if (ret == 0) {
        shdisp_sys_dbg_bdic.r_singl_ok_count++;
    } else {
        shdisp_sys_dbg_bdic.r_singl_ng_count++;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_multi_read_start                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_multi_read_start(void)
{
    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    getnstimeofday(&shdisp_sys_dbg_bdic.r_multi_t_start);
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_multi_read_retry                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_multi_read_retry(void)
{
    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    shdisp_sys_dbg_bdic.r_multi_retry++;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_multi_read_end                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_multi_read_end(int ret)
{
    struct timespec stop, df;

    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    getnstimeofday(&stop);

    df = timespec_sub(stop, shdisp_sys_dbg_bdic.r_multi_t_start);

    shdisp_sys_dbg_bdic.r_multi_t_sum = timespec_add(shdisp_sys_dbg_bdic.r_multi_t_sum, df);

    if (ret == 0) {
        shdisp_sys_dbg_bdic.r_multi_ok_count++;
    } else {
        shdisp_sys_dbg_bdic.r_multi_ng_count++;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_multi_write_start                                         */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_multi_write_start(void)
{
    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    getnstimeofday(&shdisp_sys_dbg_bdic.w_multi_t_start);
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_multi_write_retry                                         */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_multi_write_retry(void)
{
    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    shdisp_sys_dbg_bdic.w_multi_retry++;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_multi_write_end                                           */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_bdic_multi_write_end(int ret)
{
    struct timespec stop, df;

    if (shdisp_sys_dbg_api.flag == 0) {
        return;
    }

    getnstimeofday(&stop);

    df = timespec_sub(stop, shdisp_sys_dbg_bdic.w_multi_t_start);

    shdisp_sys_dbg_bdic.w_multi_t_sum = timespec_add(shdisp_sys_dbg_bdic.w_multi_t_sum, df);

    if (ret == 0) {
        shdisp_sys_dbg_bdic.w_multi_ok_count++;
    } else {
        shdisp_sys_dbg_bdic.w_multi_ng_count++;
    }
}
#endif /* SHDISP_SYS_SW_TIME_BDIC */
#endif /* SHDISP_SYS_SW_TIME_API */

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
