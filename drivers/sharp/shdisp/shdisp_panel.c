/* drivers/sharp/shdisp/shdisp_panel.c  (Display Driver)
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
#include "shdisp_system.h"
#include "shdisp_bdic.h"
#include "shdisp_dbg.h"

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/delay.h>
#include <asm/param.h>


#include "shdisp_panel.h"
#if defined(CONFIG_SHDISP_PANEL_CARIN)
#include "shdisp_carin.h"
#elif defined(CONFIG_SHDISP_PANEL_ANDY)
#include "shdisp_andy.h"
#elif defined(CONFIG_SHDISP_PANEL_ARIA)
#include "shdisp_aria.h"
#else   /* defined(CONFIG_SHDISP_PANEL_CARIN) */
#include "shdisp_null_panel.h"
#endif  /* defined(CONFIG_SHDISP_PANEL_CARIN) */

#define MIPI_DSI_SHORT_PACKET_LEN            (8)
#define MIPI_DSI_READ_RESPONSE_LEN           (8)

static struct shdisp_panel_operations *shdisp_panel_fops = NULL;

static struct shdisp_panel_operations shdisp_def_fops = {
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
extern int mdss_shdisp_host_dsi_tx(int commit, struct shdisp_dsi_cmd_desc *shdisp_cmds, int size);
extern int mdss_shdisp_host_dsi_rx(struct shdisp_dsi_cmd_desc *cmds, unsigned char *rx_data, int rx_size);

static void shdisp_panel_dsi_wlog(struct shdisp_dsi_cmd_desc *cmds, int cmdslen);
static void shdisp_panel_dsi_rlog(char addr, char *rbuf, int len);

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define MIPI_SHARP_RW_MAX_SIZE          SHDISP_LCDDR_BUF_MAX
#define MIPI_SHARP_R_SIZE               (10)

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_panel_dsi_wlog                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_panel_dsi_wlog(struct shdisp_dsi_cmd_desc *cmds, int cmdslen)
{
#ifdef SHDISP_LOG_ENABLE
    char buf[128];
    char *pbuf;
    int cmdcnt;
    int arylen;
    int writelen;

    for (cmdcnt = 0; cmdcnt != cmdslen; cmdcnt++) {
        int i;
        char dtype = cmds[cmdcnt].dtype;
        short payloadlen = cmds[cmdcnt].dlen;
        unsigned char *payload = (unsigned char *)cmds[cmdcnt].payload;

        pbuf = buf;
        arylen = sizeof(buf) / sizeof(*buf);

        writelen = snprintf(pbuf, arylen, "dtype= %02X, ", dtype);
        arylen -= writelen;
        pbuf += writelen;

        writelen = snprintf(pbuf, arylen, "payload= %02X ", payload[0]);
        arylen -= writelen;
        pbuf += writelen;

        for (i = 1; i != payloadlen; ++i) {
            if ((!((i - 1) % 16)) && (i != 1)) {
                int spacecnt = 23;
                *pbuf = '\0';
                SHDISP_DEBUG("%s", buf);

                arylen = sizeof(buf) / sizeof(*buf);
                pbuf = buf;
                memset(pbuf, ' ', spacecnt);
                arylen -= spacecnt;
                pbuf += spacecnt;
            }
            writelen = snprintf(pbuf, arylen, "%02X ", payload[i]);
            arylen -= writelen;
            pbuf += writelen;
        }

        *pbuf = '\0';
        SHDISP_DEBUG("%s", buf);
    }
#endif /* SHDISP_LOG_ENABLE */
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_dsi_rlog                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_panel_dsi_rlog(char addr, char *rbuf, int len)
{
#ifdef SHDISP_LOG_ENABLE
    char buf[128];
    char *pbuf;
    unsigned char *prbuf = (unsigned char *)rbuf;
    int arylen;
    int writelen;
    int i = 0;

    arylen = sizeof(buf) / sizeof(*buf);
    pbuf = buf;

    writelen = snprintf(pbuf, arylen, "addr = %02X, val = ", (unsigned char)addr);
    arylen -= writelen;
    pbuf += writelen;

    for (i = 0; i != len; ++i) {
        if ((!(i % 16)) && (i)) {
            int spacecnt = 17;
            *pbuf = '\0';
            SHDISP_DEBUG("%s", buf);

            arylen = sizeof(buf) / sizeof(*buf);
            pbuf = buf;
            memset(pbuf, ' ', spacecnt);
            arylen -= spacecnt;
            pbuf += spacecnt;
        }
        writelen = snprintf(pbuf, arylen, "%02X ", *prbuf);
        arylen -= writelen;
        pbuf += writelen;
        prbuf++;
    }

    *pbuf = '\0';
    SHDISP_DEBUG("%s", buf);
#endif /* SHDISP_LOG_ENABLE */
}

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_create                                                   */
/* ------------------------------------------------------------------------- */
void shdisp_panel_API_create(void)
{
    shdisp_panel_fops = &shdisp_def_fops;

#if defined(CONFIG_SHDISP_PANEL_CARIN)
    shdisp_panel_fops = shdisp_carin_API_create();
#elif defined(CONFIG_SHDISP_PANEL_ANDY)
    shdisp_panel_fops = shdisp_andy_API_create();
#elif defined(CONFIG_SHDISP_PANEL_ARIA)
    shdisp_panel_fops = shdisp_aria_API_create();
#else   /* defined(CONFIG_SHDISP_PANEL_CARIN) */
    shdisp_panel_fops = shdisp_null_panel_API_create();
#endif  /* defined(CONFIG_SHDISP_PANEL_CARIN) */
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_init_io                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_init_io(struct shdisp_panel_context *panel_ctx)
{
    if (shdisp_panel_fops->init_io) {
        return shdisp_panel_fops->init_io(panel_ctx);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_exit_io                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_exit_io(void)
{
    if (shdisp_panel_fops->exit_io) {
        return shdisp_panel_fops->exit_io();
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_power_on                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_power_on(int mode)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->power_on) {
        SHDISP_DEBUG("out1");
        return shdisp_panel_fops->power_on(mode);
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_power_off                                                */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_power_off(int mode)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->power_off) {
        SHDISP_DEBUG("out1");
        return shdisp_panel_fops->power_off(mode);
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_disp_on                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_disp_on(void)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->disp_on) {
        SHDISP_DEBUG("out1");
        return shdisp_panel_fops->disp_on();
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_disp_off                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_disp_off(void)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->disp_off) {
        SHDISP_DEBUG("out1");
        return shdisp_panel_fops->disp_off();
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_write_reg                                           */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    if (shdisp_panel_fops->write_reg) {
        return shdisp_panel_fops->write_reg(addr, write_data, size);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_read_reg                                            */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    if (shdisp_panel_fops->read_reg) {
        return shdisp_panel_fops->read_reg(addr, read_data, size);
    }
    return SHDISP_RESULT_SUCCESS;
}

#ifndef SHDISP_NOT_SUPPORT_FLICKER

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_set_flicker_param                                   */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_set_flicker_param(struct shdisp_diag_flicker_param vcom)
{
    if (shdisp_panel_fops->set_flicker) {
        return shdisp_panel_fops->set_flicker(vcom);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_get_flicker_param                                   */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *vcom)
{
    if (shdisp_panel_fops->get_flicker) {
        return shdisp_panel_fops->get_flicker(vcom);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_get_flicker_low_param                               */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *vcom)
{
    if (shdisp_panel_fops->get_flicker_low) {
        return shdisp_panel_fops->get_flicker_low(vcom);
    }
    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_mipi_diag_write_reg                                      */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_mipi_diag_write_reg(char dtype, unsigned char addr, char *write_data, unsigned char size)
{
    int retry = 5;
    int ret;
    struct shdisp_dsi_cmd_desc cmd[1];
    char cmd_buf[MIPI_SHARP_RW_MAX_SIZE + 1];

    if (size > MIPI_SHARP_RW_MAX_SIZE) {
        SHDISP_ERR("size over, -EINVAL");
        return -EINVAL;
    }

    cmd_buf[0] = addr;
    cmd_buf[1] = 0x00;
    memcpy(&cmd_buf[1], write_data, size);

    cmd[0].dtype = dtype;
    cmd[0].wait = 0x00;
    cmd[0].dlen = size + 1;
    cmd[0].payload = cmd_buf;

    for (; retry >= 0; retry--) {
        ret = shdisp_panel_API_mipi_dsi_cmds_tx(1, cmd, ARRAY_SIZE(cmd));
        if (ret == SHDISP_RESULT_SUCCESS) {
            break;
        } else {
            SHDISP_WARN("shdisp_panel_API_mipi_dsi_cmds_tx() failure. ret=%d retry=%d", ret, retry);
        }
    }
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_tx error");
        return -ENODEV;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_mipi_diag_read_reg                                       */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_mipi_diag_read_reg(char dtype, unsigned char addr, unsigned char *read_data, unsigned char size)
{
    int retry = 5;
    int ret;
#ifdef SHDISP_LOG_ENABLE
    int i;
    unsigned char rlen;
#endif  /* SHDISP_LOG_ENABLE */
    struct shdisp_dsi_cmd_desc cmd[1];
    char cmd_buf[2 + 1];

    SHDISP_TRACE("in address:%02X, buf:0x%08X, size:%d", addr, (int)read_data, size);
    if ((size > MIPI_SHARP_RW_MAX_SIZE) || (size == 0)) {
        SHDISP_ERR("size over, -EINVAL");
        return -EINVAL;
    }

    cmd_buf[0] = addr;
    cmd_buf[1] = 0x00;

    cmd[0].dtype = dtype;
    cmd[0].wait     = 0x00;
    cmd[0].dlen     = 2;
    cmd[0].payload  = cmd_buf;

    memset(read_data, 0, size);

    for (; retry >= 0; retry--) {
        ret = shdisp_panel_API_mipi_dsi_cmds_rx(read_data, cmd, size);
        if (ret == SHDISP_RESULT_SUCCESS) {
            break;
        } else {
            SHDISP_WARN("shdisp_panel_API_mipi_dsi_cmds_rx() failure. ret=%d retry=%d", ret, retry);
        }
    }
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_rx error");
        return -ENODEV;
    }

#ifdef SHDISP_LOG_ENABLE
    rlen = size;
    for (i = 0; i < rlen; i++) {
        if ((i % MIPI_DSI_SHORT_PACKET_LEN) == 0) {
            SHDISP_DEBUG(" ");
            SHDISP_PRINTK(SHDISP_LOG_LV_DEBUG, "[SHDISP_DEBUG][%s] Data    : ", __func__);
        }
        SHDISP_PRINTK(SHDISP_LOG_LV_DEBUG, "%02X ", read_data[i]);
    }
    SHDISP_DEBUG(" ");
#endif  /* SHDISP_LOG_ENABLE */

    SHDISP_TRACE("out SHDISP_RESULT_SUCCESS");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_start_display                                            */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_start_display(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");
    if (shdisp_panel_fops->start_display) {
        return shdisp_panel_fops->start_display();
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_post_video_start                                         */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_post_video_start(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");
    if (shdisp_panel_fops->post_video_start) {
        return shdisp_panel_fops->post_video_start();
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_check_recovery                                           */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_check_recovery(void)
{
    if (shdisp_panel_fops->check_recovery) {
        return shdisp_panel_fops->check_recovery();
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_set_gammatable_and_voltage                          */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_set_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info)
{
    if (shdisp_panel_fops->set_gammatable_and_voltage) {
        return shdisp_panel_fops->set_gammatable_and_voltage(gamma_info);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_get_gammatable_and_voltage                          */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_get_gammatable_and_voltage(struct shdisp_diag_gamma_info *gamma_info)
{
    if (shdisp_panel_fops->get_gammatable_and_voltage) {
        return shdisp_panel_fops->get_gammatable_and_voltage(gamma_info);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_set_gamma                                           */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_set_gamma(struct shdisp_diag_gamma *gamma)
{
    if (shdisp_panel_fops->set_gamma) {
        return shdisp_panel_fops->set_gamma(gamma);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_shutdown                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_shutdown(void)
{
    int ret = 0;
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->shutdown) {
        ret = shdisp_panel_fops->shutdown();
        SHDISP_DEBUG("out ret=%d", ret);
        return ret;
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_dump_reg                                                 */
/* ------------------------------------------------------------------------- */
void shdisp_panel_API_dump(int type)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->dump) {
        shdisp_panel_fops->dump(type);
    }
    SHDISP_TRACE("out");
}
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_set_irq                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_set_irq(int enable)
{
    int ret = 0;

    SHDISP_TRACE("in");

    if (shdisp_panel_fops->set_irq) {
        ret = shdisp_panel_fops->set_irq(enable);
        SHDISP_DEBUG("out ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_mipi_dsi_cmds_tx                                         */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_mipi_dsi_cmds_tx(int commit, struct shdisp_dsi_cmd_desc *cmds, int cnt)
{
    int ret;
    SHDISP_TRACE("in cnt=%d", cnt);
    shdisp_panel_dsi_wlog(cmds, cnt);
    ret = mdss_shdisp_host_dsi_tx(commit, cmds, cnt );
    SHDISP_TRACE("out ret=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_mipi_dsi_cmds_rx                                         */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_mipi_dsi_cmds_rx(unsigned char *rbuf, struct shdisp_dsi_cmd_desc *cmds, unsigned char size)
{
    int ret;
    SHDISP_TRACE("in size:%d", size);
    shdisp_panel_dsi_wlog(cmds, 1);
    ret = mdss_shdisp_host_dsi_rx(cmds, rbuf, size);
    shdisp_panel_dsi_rlog(cmds->payload[0], (char *)rbuf, size);
    SHDISP_TRACE("out ret:%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
