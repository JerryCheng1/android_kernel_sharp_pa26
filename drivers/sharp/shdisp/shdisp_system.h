/* drivers/sharp/shdisp/shdisp_system.h  (Display Driver)
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
#ifndef SHDISP_SYSTEM_H
#define SHDISP_SYSTEM_H
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_GPIO_CTL_LOW                 (0)
#define SHDISP_GPIO_CTL_HIGH                (1)

#define SHDISP_BOOT_SW_ENABLE_DISPLAY       (1)


#define SHDISP_GPIO_NUM_BL_RST_N            (76)
#define SHDISP_GPIO_NUM_PANEL_RST_N         (85)
#define SHDISP_GPIO_NUM_LCD_CLK             (58)
#define SHDISP_GPIO_NUM_LCD_SCS_N           (32)
#define SHDISP_GPIO_NUM_CLK_SEL             (107)
#define SHDISP_GPIO_NUM_SP_SWIC             (51)
#define SHDISP_GPIO_NUM_ANDY_VDD            (69)
#define SHDISP_GPIO_NUM_UPPER_UNIT          (96)
#define SHDISP_GPIO_NUM_MIPI_ERROR          (75)

#define SHDISP_HW_REV_ES0                   (0x00)            /* ES0   */
#define SHDISP_HW_REV_ES1                   (0x01)            /* ES1   */
#define SHDISP_HW_REV_ES2                   (0x02)            /* ES2   */
#define SHDISP_HW_REV_PP1                   (0x03)            /* PP1   */
#define SHDISP_HW_REV_PP15                  (0x04)            /* PP1.5 */
#define SHDISP_HW_REV_PP2                   (0x05)            /* PP2   */
#define SHDISP_HW_REV_PP25                  (0x06)            /* PP2.5 */
#define SHDISP_HW_REV_MP                    (0x07)            /* MP    */

#define SHDISP_HW_REV_BIT_ES0               (0)
#define SHDISP_HW_REV_BIT_ES1               (2)
#define SHDISP_HW_REV_BIT_ES2               (4)
#define SHDISP_HW_REV_BIT_PP1               (6)
#define SHDISP_HW_REV_BIT_PP2               (1)
#define SHDISP_HW_REV_BIT_RESERVE1          (5)
#define SHDISP_HW_REV_BIT_RESERVE2          (3)
#define SHDISP_HW_REV_BIT_MP                (7)

#define SHDISP_BDIC_I2C_DEVNAME             ("sharp,bdic_i2c")
#define SHDISP_SENSOR_DEVNAME               ("sharp,sensor_i2c")

#define WAIT_1FRAME_US                      (16666)

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
#define SHDISP_IRQ_MAX_KIND                 (4)

enum {
    SHDISP_HOST_CTL_CMD_LCD_CLK_START,
    SHDISP_HOST_CTL_CMD_LCD_CLK_STOP,
    SHDISP_HOST_CTL_CMD_LCD_CLK_INIT,
    SHDISP_HOST_CTL_CMD_LCD_CLK_EXIT,
    NUM_SHDISP_HOST_CTL_CMD
};

enum {
    SHDISP_IRQ_DISABLE,
    SHDISP_IRQ_ENABLE,
    NUM_SHDISP_IRQ_CMD
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_Host_control(int cmd, unsigned long rate);
void shdisp_SYS_API_delay_us(unsigned long usec);
void shdisp_SYS_API_Host_gpio_init(void);
void shdisp_SYS_API_Host_gpio_exit(void);
int  shdisp_SYS_API_Host_gpio_request(int num);
int  shdisp_SYS_API_Host_gpio_free(int num);
int  shdisp_SYS_API_set_Host_gpio(int num, int value);
int  shdisp_SYS_API_get_Host_gpio(int num);
int  shdisp_SYS_API_check_upper_unit(void);

void shdisp_SYS_API_set_irq_port(int irq_port, struct platform_device *pdev);
int  shdisp_SYS_API_request_irq(irqreturn_t (*irq_handler)( int , void * ) );
void shdisp_SYS_API_free_irq(void);
void shdisp_SYS_API_set_irq_init(void);
int  shdisp_SYS_API_set_irq( int enable );
void shdisp_sys_API_dbg_hw_check_start(void);
void shdisp_sys_API_dbg_hw_check_end(const char *func);

int  shdisp_SYS_API_bdic_i2c_init(void);
int  shdisp_SYS_API_bdic_i2c_exit(void);
int  shdisp_SYS_API_bdic_i2c_write(unsigned char addr, unsigned char data);
int  shdisp_SYS_API_bdic_i2c_mask_write(unsigned char addr, unsigned char data, unsigned char mask);
int  shdisp_SYS_API_bdic_i2c_multi_write(unsigned char addr, unsigned char *wval, unsigned char size);
int  shdisp_SYS_API_bdic_i2c_read(unsigned char addr, unsigned char *data);
int  shdisp_SYS_API_bdic_i2c_multi_read(unsigned char addr, unsigned char *data, int size);

int  shdisp_SYS_API_sensor_i2c_init(void);
int  shdisp_SYS_API_sensor_i2c_exit(void);

int  shdisp_SYS_API_panel_external_clk_ctl(int enable);

int  shdisp_SYS_API_Host_i2c_send(unsigned char slaveaddr, unsigned char *sendval, unsigned char size);
int  shdisp_SYS_API_Host_i2c_recv(unsigned char slaveaddr, unsigned char *sendval, unsigned char sendsize,
                                   unsigned char *recvval, unsigned char recvsize);
#endif /* SHDISP_SYSTEM_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
