/* drivers/sharp/shsensor_hub/ml630q790/shub-input_diag.c
 *
 * Copyright (C) 2014 Sharp Corporation
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input-polldev.h>

#include <asm/uaccess.h> 
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ioctl.h>

#include "shub_io.h"
#include "ml630q790.h"

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/poll.h>

#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/irq.h>

// SHMDS_HUB_0701_01 add S
#ifdef CONFIG_ANDROID_ENGINEERING
static int shub_diag_log = 0;
module_param(shub_diag_log, int, 0600);
#define DBG_DIAG_IO(msg, ...) {                      \
    if(shub_diag_log)                                \
        printk("[shub][diag] " msg, ##__VA_ARGS__);  \
}
#else
#define DBG_DIAG_IO(msg, ...)
#endif
// SHMDS_HUB_0701_01 add E

#define SHUB_DIAG_SHUB   0x01
#define SHUB_DIAG_6AXIS  0x02
#define SHUB_DIAG_MAG    0x03

#define HC_MCU_GET_VERSION              (0x0001u)
#define HC_MCU_ACC_SENSOR_REG           (0x0009u)
#define HC_MCU_SELF_TEST                (0x000Au)
#define HC_MCU_I2C_IO                   (0x000Cu)


#define YAS_PCB_ADDR_ID                 (0x80)
#define YAS_PCB_ADDR_COIL               (0x81)
#define YAS_PCB_ADDR_MEASURE_COMMAND    (0x82)
#define YAS_PCB_ADDR_CONFIG             (0x83)
#define YAS_PCB_ADDR_MEASURE_INTERVAL   (0x84)
#define YAS_PCB_ADDR_OFFSETX            (0x85)
#define YAS_PCB_ADDR_OFFSETY1           (0x86)
#define YAS_PCB_ADDR_OFFSETY2           (0x87)
#define YAS_PCB_ADDR_TEST               (0x88)
#define YAS_PCB_ADDR_TEST2              (0x89)
#define YAS_PCB_ADDR_CAL                (0x90)
#define YAS_PCB_ADDR_MEASURE_DATA       (0xB0)

#define YAS_PCB_MEASURE_COMMAND_START   (0x01)
#define YAS_PCB_MEASURE_COMMAND_LDTC    (0x02)
#define YAS_PCB_MEASURE_COMMAND_FORS    (0x04)

#define YAS_CAL_REG_NUM                 (14)
#define YAS_PCB_MEASURE_DATA_REG_NUM    (8)
#define YAS_PCB_HARD_OFFSET_CORRECT     (16)
#define YAS_PCB_COIL_INIT_CALC_NUM      (5)

#define YAS532_COEFX_VERSION_AC         (850)
#define YAS532_COEFY1_VERSION_AC        (750)
#define YAS532_COEFY2_VERSION_AC        (750)
#define YAS532_RAWDATA_CENTER           (4096)
#define YAS532_RAWDATA_OVERFLOW         (8190)
#define YAS532_SLAVE_ADDRESS            (0x2Eu)
#define YAS532_TEMP20DEGREE_TYPICAL     (390)			// SHMDS_HUB_0103_10 add

#define LSM6DS0_ACC_GYRO_INT_CTRL       (0x0C)
#define LSM6DS0_ACC_GYRO_CTRL_REG1_G    (0x10)
#define LSM6DS0_ACC_GYRO_CTRL_REG2_G    (0x11)
#define LSM6DS0_ACC_GYRO_CTRL_REG3_G    (0x12)
#define LSM6DS0_ACC_GYRO_OUT_X_L_G      (0x18)
#define LSM6DS0_ACC_GYRO_CTRL_REG4      (0x1E)
#define LSM6DS0_ACC_GYRO_CTRL_REG5_XL   (0x1F)
#define LSM6DS0_ACC_GYRO_CTRL_REG6_XL   (0x20)
#define LSM6DS0_ACC_GYRO_CTRL_REG7_XL   (0x21)
#define LSM6DS0_ACC_GYRO_CTRL_REG8      (0x22)
#define LSM6DS0_ACC_GYRO_CTRL_REG9      (0x23)
#define LSM6DS0_ACC_GYRO_OUT_X_L_XL     (0x28)
#define LSM6DS0_SLAVE_ADDRESS           (0x6Bu)

struct yas_pcb_correction {
    int32_t s32Cx, s32Cy1, s32Cy2;
    int32_t s32A2, s32A3, s32A4, s32A5, s32A6, s32A7, s32A8, s32A9, s32K;
    int32_t s32ZFlag;
    int32_t s32Rx, s32Ry1, s32Ry2;
    int32_t s32Fx, s32Fy1, s32Fy2;
    int32_t s32Ver;
};

/* How often we poll keys - msecs */

static struct platform_device *pdev;
static struct input_dev *shub_idev;

static int isFlowFlg = 0;
static int read_cal_init_flg = 0;
static uint8_t calRegData[YAS_CAL_REG_NUM];
static struct yas_pcb_correction gstCorrect;
static int gyro_first_read_wait_flg = 1;
static int initial_gyro_flg = 0;
static int initial_acc_flg = 0;
static int initial_mag_flg = 0;

/* mag sensor read */
static int shub_diag_magSensorReadSendCmd(uint8_t addr, uint8_t* resData)
{
    int32_t ret;
    uint16_t errRes;
    uint8_t prm[16];

    memset(prm, 0, sizeof(prm));
    prm[0] = 0x06;
    prm[1] = 0x00;
    prm[2] = addr;
    ret = shub_direct_sendcmd(HC_MCU_ACC_SENSOR_REG, prm);
    shub_direct_get_error(&errRes);
    if((ret == 0) && (errRes == 0)) {
        shub_direct_get_result(resData);
    }else{
        printk("[shub]%s err.addr=%02x ret=%d errRes=%d\n", __func__, addr, ret, errRes);
        return -1;
    }

    return 0;
}

/* mag sensor write */
static int shub_diag_magSensorWriteSendCmd(uint8_t addr, uint8_t data)
{
    int32_t ret;
    uint16_t errRes;
    uint8_t prm[16];

    memset(prm, 0, sizeof(prm));
    prm[0] = 0x06;
    prm[1] = 0x01;
    prm[2] = addr;
    prm[3] = data;
    ret = shub_direct_sendcmd(HC_MCU_ACC_SENSOR_REG, prm);
    shub_direct_get_error(&errRes);
    if((ret != 0) || (errRes != 0)) {
        printk("[shub]%s err.addr=%02x data=%02x ret=%d errRes=%d\n", __func__, addr, data, ret, errRes);
        return -1;
    }

    return 0;
}

/* acc gyro sensor read */
static int shub_diag_accGyroSensorReadSendCmd(uint8_t addr, uint8_t* resData)
{
    int32_t ret;
    uint16_t errRes;
    uint8_t prm[16];

    memset(prm, 0, sizeof(prm));
    prm[0] = 0x00;
    prm[1] = 0x00;
    prm[2] = addr;
    ret = shub_direct_sendcmd(HC_MCU_ACC_SENSOR_REG, prm);
    shub_direct_get_error(&errRes);
    if((ret == 0) && (errRes == 0)) {
        shub_direct_get_result(resData);
    }else{
        printk("[shub]%s err.addr=%02x ret=%d errRes=%d\n", __func__, addr, ret, errRes);
        return -1;
    }

    return 0;
}

/* acc gyro sensor write */
static int shub_diag_accGyroSensorWriteSendCmd(uint8_t addr, uint8_t data)
{
    int32_t ret;
    uint16_t errRes;
    uint8_t prm[16];

    memset(prm, 0, sizeof(prm));
    prm[0] = 0x00;
    prm[1] = 0x01;
    prm[2] = addr;
    prm[3] = data;
    ret = shub_direct_sendcmd(HC_MCU_ACC_SENSOR_REG, prm);
    shub_direct_get_error(&errRes);
    if((ret != 0) || (errRes != 0)) {
        printk("[shub]%s err.addr=%02x data=%02x ret=%d errRes=%d\n", __func__, addr, data, ret, errRes);
        return -1;
    }

    return 0;
}

/* sensor multi read */
static int shub_diag_sensorMultiReadSendCmd(uint8_t addr, uint8_t* resData, uint8_t slaveAddr, uint8_t size)
{
    int32_t ret;
    uint16_t errRes;
    uint8_t prm[16];
    uint8_t readSize = size;

    memset(prm, 0, sizeof(prm));
    prm[0] = slaveAddr;
    prm[1] = 0x00;
    prm[2] = addr;
    prm[3] = readSize;
    ret = shub_direct_sendcmd(HC_MCU_I2C_IO, prm);
    shub_direct_get_error(&errRes);
    if((ret == 0) && (errRes == 0)) {
        shub_direct_multi_get_result(resData);
    }
    else{
        printk("[shub]%s set err.addr=%02x ret=%d errRes=%d\n", __func__, addr, ret, errRes);
        return -1;
    }

    return 0;
}

/* mag initial setting */
static int shub_diag_mag_initialize(void)
{
    int ret;
    uint8_t setData = 0;
    ret = shub_diag_magSensorWriteSendCmd(YAS_PCB_ADDR_TEST, setData);
    if(ret != 0) {
        return -1;
    }
    ret = shub_diag_magSensorWriteSendCmd(YAS_PCB_ADDR_TEST2, setData);
    if(ret != 0) {
        return -1;
    }
    ret = shub_diag_magSensorWriteSendCmd(YAS_PCB_ADDR_MEASURE_INTERVAL, setData);
    if(ret != 0) {
        return -1;
    }
    return 0;
}

/* acc initial setting */
static int shub_diag_acc_initialize(void)
{
    int ret;
    uint8_t setData;

    /* INT_DRDY_XL setting */
    setData = 0x00;
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_INT_CTRL, setData);
    if(ret != 0) {
        return -1;
    }
    /* Accelerometer axis output enable */
    setData = 0x38;
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_CTRL_REG5_XL, setData);
    if(ret != 0) {
        return -1;
    }
    /* HR mode HPF disable */
    setData = 0x00;
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_CTRL_REG7_XL, setData);
    if(ret != 0) {
        return -1;
    }
    /* Block data update enable */
    setData = 0x44;
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_CTRL_REG8, setData);
    if(ret != 0) {
        return -1;
    }
    /* DA timer enabled */
    setData = 0x00;
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_CTRL_REG9, setData);
    if(ret != 0) {
        return -1;
    }
    return 0;
}

/* gyro initial setting */
static int shub_diag_gyro_initialize(void)
{
    int ret;
    uint8_t setData;

    gyro_first_read_wait_flg = 1;

    /* INT_DRDY_G setting */
    setData = 0x00;
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_INT_CTRL, setData);
    if(ret != 0) {
        return -1;
    }
    /* LPF HPF setting */
    setData = 0x0A;
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_CTRL_REG2_G, setData);
    if(ret != 0) {
        return -1;
    }
    /* HPF enable */
    setData = 0x00;
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_CTRL_REG3_G, setData);
    if(ret != 0) {
        return -1;
    }
    /* Gyroscope axis output enable */
    setData = 0x38;
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_CTRL_REG4, setData);
    if(ret != 0) {
        return -1;
    }
    /* Block data update enable */
    setData = 0x44;
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_CTRL_REG8, setData);
    if(ret != 0) {
        return -1;
    }
    /* DA timer enabled */
    setData = 0x00;
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_CTRL_REG9, setData);
    if(ret != 0) {
        return -1;
    }
    return 0;
}

/* cal data read */
static void shub_diag_mag_calc_correction(const uint8_t *pu08Data)
{
    uint8_t u08Dx  = pu08Data[0];
    uint8_t u08Dy1 = pu08Data[1];
    uint8_t u08Dy2 = pu08Data[2];
    uint8_t u08D2  = (uint8_t)((pu08Data[3] >> 2) & 0x3F);
    uint8_t u08D3  = (uint8_t)(((pu08Data[3] << 2) & 0x0C) | ((pu08Data[4] >> 6) & 0x03));
    uint8_t u08D4  = (uint8_t)(pu08Data[4] & 0x3F);
    uint8_t u08D5  = (uint8_t)((pu08Data[5] >> 2) & 0x3f);
    uint8_t u08D6  = (uint8_t)(((pu08Data[5] << 4) & 0x30) | ((pu08Data[6] >> 4) & 0x0F));
    uint8_t u08D7  = (uint8_t)(((pu08Data[6] << 3) & 0x78) | ((pu08Data[7] >> 5) & 0x07));
    uint8_t u08D8  = (uint8_t)(((pu08Data[7] << 1) & 0x3E) | ((pu08Data[8] >> 7) & 0x01));
    uint8_t u08D9  = (uint8_t)(((pu08Data[8] << 1) & 0xFE) | ((pu08Data[9] >> 7) & 0x01));
    uint8_t u08D0  = (uint8_t)((pu08Data[9] >> 2) & 0x1F);
    uint8_t u08Rx  = (uint8_t)((pu08Data[10] >> 1) & 0x3F);
    uint8_t u08Fx  = (uint8_t)(((pu08Data[10] & 0x01) << 1) | ((pu08Data[11] >> 7) & 0x01));
    uint8_t u08Ry1 = (uint8_t)((pu08Data[11] >> 1) & 0x3F);
    uint8_t u08Fy1 = (uint8_t)(((pu08Data[11] & 0x01) << 1) | ((pu08Data[12] >> 7) & 0x01));
    uint8_t u08Ry2 = (uint8_t)((pu08Data[12] >> 1) & 0x3F);
    uint8_t u08Fy2 = (uint8_t)(((pu08Data[12] & 0x01) << 1) | ((pu08Data[13] >> 7) & 0x01));
    uint8_t u08Ver = pu08Data[13] & 0x01;

    gstCorrect.s32Cx  = (int32_t)((u08Dx * 10) - 1280);
    gstCorrect.s32Cy1 = (int32_t)((u08Dy1 * 10) - 1280);
    gstCorrect.s32Cy2 = (int32_t)((u08Dy2 * 10) - 1280);
    gstCorrect.s32A2  = (int32_t)(u08D2 - 32);
    gstCorrect.s32A3  = (int32_t)(u08D3 - 8);
    gstCorrect.s32A4  = (int32_t)(u08D4 - 32);
    gstCorrect.s32A5  = (int32_t)(u08D5 + 38);
    gstCorrect.s32A6  = (int32_t)(u08D6 - 32);
    gstCorrect.s32A7  = (int32_t)(u08D7 - 64);
    gstCorrect.s32A8  = (int32_t)(u08D8 - 32);
    gstCorrect.s32A9  = (int32_t)u08D9;
    gstCorrect.s32K   = (int32_t)u08D0;
    gstCorrect.s32ZFlag = (int32_t)1;
    gstCorrect.s32Rx  = (int32_t)((int8_t)(u08Rx << 2) >> 2);
    gstCorrect.s32Fx  = (int32_t)u08Fx;
    gstCorrect.s32Ry1 = (int32_t)((int8_t)(u08Ry1 << 2) >> 2);
    gstCorrect.s32Fy1 = (int32_t)u08Fy1;
    gstCorrect.s32Ry2 = (int32_t)((int8_t)(u08Ry2 << 2) >> 2);
    gstCorrect.s32Fy2 = (int32_t)u08Fy2;
    gstCorrect.s32Ver = (int32_t)u08Ver;
    read_cal_init_flg = 1;
}

/* cal data read */
static int shub_diag_mag_read_cal(void)
{
    int i;
    int ret;
    int size = YAS_CAL_REG_NUM;
    int len = size - 1;
    uint8_t resData[2];

    if(read_cal_init_flg){
        return 0;
    }
    memset(calRegData, 0, sizeof(calRegData));
    memset(resData, 0, sizeof(resData));
    /* Dummy read */
/* SHMDS_HUB_0103_09 mod S */
//    for(i = 0; i < len; i++)
    for(i = 0; i < size; i++)
    {
/* SHMDS_HUB_0103_09 mod E */
        ret = shub_diag_magSensorReadSendCmd(YAS_PCB_ADDR_CAL + i, resData);
        if(ret != 0) {
            printk("[shub]%s dummy read err.\n", __func__);
            return -1;
        }
    }

    memset(resData, 0, sizeof(resData));
/* SHMDS_HUB_0103_09 mod S */
//    for(i = 0; i < len; i++)
    for(i = 0; i < size; i++)
    {
/* SHMDS_HUB_0103_09 mod E */
        ret = shub_diag_magSensorReadSendCmd(YAS_PCB_ADDR_CAL + i, resData);
        if(ret != 0) {
            printk("[shub]%s read err.\n", __func__);
            return -1;
        }
        calRegData[i] = resData[0];
    }

    /* cal register is all 0 */
    for (i = 0; i < size; i++) {
        if (calRegData[i] != 0x00) {
            /* OK */
            shub_diag_mag_calc_correction(calRegData);
            return 0;
        }
    }

    /* MSB is not 0 */
    if (calRegData[len] & 0x80) {
        /* OK */
        shub_diag_mag_calc_correction(calRegData);
        return 0;
    }

    printk("[shub]%s check err.\n", __func__);
    return -1;
}

/* config data setting */
static int shub_diag_mag_set_config(void)
{
    int ret;
    uint8_t setData;
    
    setData = 0x01;
    setData = (uint8_t)(setData | (uint8_t)((calRegData[9]  & 0x03) << 3) | (uint8_t)((calRegData[10] & 0x80) >> 5));
    ret = shub_diag_magSensorWriteSendCmd(YAS_PCB_ADDR_CONFIG, setData);
    if(ret != 0) {
        return -1;
    }
    return 0;
}

/* coil data setting */
static int shub_diag_mag_set_coil(void)
{
    int ret;
    uint8_t setData = 0;
    
    ret = shub_diag_magSensorWriteSendCmd(YAS_PCB_ADDR_COIL, setData);
    if(ret != 0) {
        return -1;
    }
    isFlowFlg = 0;
    return 0;
}

/* offset data setting */
static int shub_diag_mag_set_offset_data(uint8_t* offsetData)
{
    int ret;
    uint8_t setData;
    
    setData = offsetData[0];
    ret = shub_diag_magSensorWriteSendCmd(YAS_PCB_ADDR_OFFSETX, setData);
    if(ret != 0) {
        return -1;
    }
    setData = offsetData[1];
    ret = shub_diag_magSensorWriteSendCmd(YAS_PCB_ADDR_OFFSETY1, setData);
    if(ret != 0) {
        return -1;
    }
    setData = offsetData[2];
    ret = shub_diag_magSensorWriteSendCmd(YAS_PCB_ADDR_OFFSETY2, setData);
    if(ret != 0) {
        return -1;
    }

    return 0;
}

/* Accelerometer mode setting */
static int shub_diag_acc_mode_change(int mode)
{
    int ret;
    uint8_t setData;

    if(mode){
        /* power mode(Accelerometer data out) */
        setData = 0xD4;
    }
    else{
        /* power down mode */
        setData = 0x00;
    }
    /* Accelerometer mode setting */
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_CTRL_REG6_XL, setData);
    if(ret != 0) {
        return -1;
    }
    return 0;
}

/* Gyroscope mode setting */
static int shub_diag_gyro_mode_change(int mode)
{
    int ret;
    uint8_t setData;

    if(mode){
        /* power mode(Gyroscope data out) */
        setData = 0xD8;
    }
    else{
        /* power down mode */
        setData = 0x00;
        gyro_first_read_wait_flg = 1;
    }
    /* Gyroscope mode setting */
    ret = shub_diag_accGyroSensorWriteSendCmd(LSM6DS0_ACC_GYRO_CTRL_REG1_G, setData);
    if(ret != 0) {
        return -1;
    }
    return 0;
}

/* mag measure one time */
static int shub_diag_mag_measure(int32_t* magData, int* temperature, uint8_t cmd)
{
    int cnt;
    int ret;
    uint8_t mesData[YAS_PCB_MEASURE_DATA_REG_NUM + 1];
    uint8_t setData;
    uint8_t resData[2];
    
    /* measure start */
    setData = cmd;
    ret = shub_diag_magSensorWriteSendCmd(YAS_PCB_ADDR_MEASURE_COMMAND, setData);
    if(ret != 0) {
        return -1;
    }
    
    memset(resData, 0, sizeof(resData));
    /* measure complete wait */
    for(cnt = 0; cnt < 100; cnt++){
        usleep(100);
        ret = shub_diag_magSensorReadSendCmd(YAS_PCB_ADDR_MEASURE_DATA, resData);
        if(ret != 0) {
            printk("[shub]%s err.\n", __func__);
            return -1;
        }
        if(resData[0] & 0x80){
            printk("[shub]%s measurement now. cnt=%d\n", __func__, cnt);
            ret = -1;
        }
        else{
            ret = 0;
            break;
        }
    }
    if(ret != 0) {
        printk("[shub]%s measure timeout.\n", __func__);
        return -1;
    }
    
    usleep(1500);
    
    memset(mesData, 0, sizeof(mesData));
    ret = shub_diag_sensorMultiReadSendCmd(YAS_PCB_ADDR_MEASURE_DATA, mesData, YAS532_SLAVE_ADDRESS, YAS_PCB_MEASURE_DATA_REG_NUM);
    if((ret != 0) || (mesData[0] != 0)) {
        printk("[shub]%s measurement read err.ret=%d %d\n", __func__, ret, mesData[0]);
        return -1;
    }
    *temperature = (((int32_t)(mesData[1] & 0x7F) << 3) | ((mesData[2] >> 5) & 0x07));
    magData[0] = (int32_t)(((int32_t)(mesData[3] & 0x7F) << 6) | ((mesData[4] >> 2) & 0x3F));
    magData[1] = (int32_t)(((int32_t)(mesData[5] & 0x7F) << 6) | ((mesData[6] >> 2) & 0x3F));
    magData[2] = (int32_t)(((int32_t)(mesData[7] & 0x7F) << 6) | ((mesData[8] >> 2) & 0x3F));

    return 0;
}

/* acc measure one time */
static int shub_diag_acc_measure(int16_t* accData)
{
    int ret;
    uint8_t mesData[9];

    memset(mesData, 0, sizeof(mesData));
    ret = shub_diag_sensorMultiReadSendCmd(LSM6DS0_ACC_GYRO_OUT_X_L_XL, mesData, LSM6DS0_SLAVE_ADDRESS, 6);
    if((ret != 0) || (mesData[0] != 0)) {
        printk("[shub]%s measurement read err.ret=%d %d\n", __func__, ret, mesData[0]);
        return -1;
    }
    accData[0] = (int16_t)( ((uint16_t)mesData[2] << 8) | (uint16_t)mesData[1]);
    accData[1] = (int16_t)( ((uint16_t)mesData[4] << 8) | (uint16_t)mesData[3]);
    accData[2] = (int16_t)( ((uint16_t)mesData[6] << 8) | (uint16_t)mesData[5]);

    return 0;
}

/* gyro measure one time */
static int shub_diag_gyro_measure(int16_t* gyroData)
{
    int ret;
    uint8_t mesData[9];

    memset(mesData, 0, sizeof(mesData));
    ret = shub_diag_sensorMultiReadSendCmd(LSM6DS0_ACC_GYRO_OUT_X_L_G, mesData, LSM6DS0_SLAVE_ADDRESS, 6);
    if((ret != 0) || (mesData[0] != 0)) {
        printk("[shub]%s measurement read err.ret=%d %d\n", __func__, ret, mesData[0]);
        return -1;
    }
    gyroData[0] = (int16_t)( ((uint16_t)mesData[2] << 8) | (uint16_t)mesData[1]);
    gyroData[1] = (int16_t)( ((uint16_t)mesData[4] << 8) | (uint16_t)mesData[3]);
    gyroData[2] = (int16_t)( ((uint16_t)mesData[6] << 8) | (uint16_t)mesData[5]);

    return 0;
}

/* offset adjust value calc */
static int shub_diag_mag_offset_adjust_calc(void)
{
    int ret = 0;
    int i;
    int k;
    int temperature;
    int8_t offsetCorrect = YAS_PCB_HARD_OFFSET_CORRECT;
    int32_t magData[3];
    uint8_t offsetData[3];
    
    memset(offsetData, 0, sizeof(offsetData));
    ret = shub_diag_mag_set_offset_data(offsetData);
    if(ret != 0) {
        return -1;
    }

    for(i = 0; i < YAS_PCB_COIL_INIT_CALC_NUM; i++){
        temperature = 0;
        memset(magData, 0, sizeof(magData));
        ret = shub_diag_mag_measure(magData, &temperature, YAS_PCB_MEASURE_COMMAND_START);
        if(ret != 0) {
            break;
        }
        for(k = 0; k < 3; k++){
            if(YAS532_RAWDATA_CENTER < magData[k]){
                offsetData[k] += offsetCorrect;
                ret = 0;
            }
            else if(magData[k] < YAS532_RAWDATA_CENTER){
                offsetData[k] -= offsetCorrect;
                ret = 0;
            }
            else{
                ret += 1;
            }
        }
        if(ret == 3){
            break;
        }
        ret = shub_diag_mag_set_offset_data(offsetData);
        if(ret != 0) {
            break;
        }
        offsetCorrect = (int8_t)((uint8_t)offsetCorrect >> 1);
    }

    return ret;
}

/* temperature revision and coordinate change  */
static void shub_diag_mag_data_revision(int32_t* mesData, int temperature, int32_t* revData)
{
    int32_t s32Sx  = mesData[0] - ((gstCorrect.s32Cx  * temperature) / 1000);
    int32_t s32Sy1 = mesData[1] - ((gstCorrect.s32Cy1 * temperature) / 1000);
    int32_t s32Sy2 = mesData[2] - ((gstCorrect.s32Cy2 * temperature) / 1000);
    int32_t s32Sy  =  s32Sy1 - s32Sy2;
    int32_t s32Sz  = -s32Sy1 - s32Sy2;

/* SHMDS_HUB_0103_06 add S */
    revData[0] = (gstCorrect.s32K
             * ((100 * s32Sx)
             + (gstCorrect.s32A2 * s32Sy)
             + (gstCorrect.s32A3 * s32Sz))) / 10;
    revData[1] = (gstCorrect.s32K
             * ((gstCorrect.s32A4 * s32Sx)
             + (gstCorrect.s32A5 * s32Sy)
             + (gstCorrect.s32A6 * s32Sz))) / 10;
    revData[2] = (gstCorrect.s32K
             * ((gstCorrect.s32A7 * s32Sx)
             + (gstCorrect.s32A8 * s32Sy)
             + (gstCorrect.s32A9 * s32Sz))) / 10;

    revData[0] = revData[0] / 1000;
    revData[1] = revData[1] / 1000;
    revData[2] = revData[2] / 1000;
/* SHMDS_HUB_0103_06 add E */
}

/* check overflow and underflow */
static int shub_diag_mag_is_flow_occued(int32_t* mesData, int32_t underflow, int32_t overflow)
{
    int ret = 0;
    int32_t s32Tmp;
    uint8_t i;

    for (i = 0; i < 3; i++) {
        s32Tmp = mesData[i];
        if (s32Tmp <= underflow){
            printk("[shub]%s underflow err.\n", __func__);
            ret = -1;
        }
        else{
            if (overflow <= s32Tmp){
                printk("[shub]%s overflow err.\n", __func__);
                ret = -1;
            }
        }
    }

    return ret;
}


/* collect mag initial setting */
static int shub_diag_mag_initialize_collect(void)
{
    int ret;
    /* mag initial setting */
    ret = shub_diag_mag_initialize();
    if(ret != 0) {
        return -1;
    }
    /* cal data read */
    ret = shub_diag_mag_read_cal();
    if(ret != 0) {
        return -1;
    }
    /* config data setting */
    ret = shub_diag_mag_set_config();
    if(ret != 0) {
        return -1;
    }
    /* coil data setting */
    ret = shub_diag_mag_set_coil();
    if(ret != 0) {
        return -1;
    }
    /* offset adjust value calc */
    ret = shub_diag_mag_offset_adjust_calc();
    if(ret != 0) {
        return -1;
    }
    return 0;
}

/* magnetic field data measure(continuation call) */
static int shub_diag_mag_data_measure_cont(int32_t* revData)
{
    int ret;
    int temperature;
    int32_t mesData[3];

    if(isFlowFlg){
        /* coil data setting */
        ret = shub_diag_mag_set_coil();
        if(ret != 0) {
            return -1;
        }
        /* offset adjust value calc */
        ret = shub_diag_mag_offset_adjust_calc();
        if(ret != 0) {
            return -1;
        }
    }
    /* mag measure one time */
    ret = shub_diag_mag_measure(mesData, &temperature, YAS_PCB_MEASURE_COMMAND_START);
    if(ret != 0) {
        return -1;
    }
    /* temperature revision and coordinate change  */
    temperature = (temperature - YAS532_TEMP20DEGREE_TYPICAL) * 10;			// SHMDS_HUB_0103_10 add
    shub_diag_mag_data_revision(mesData, temperature, revData);
    /* check overflow and underflow */
    ret = shub_diag_mag_is_flow_occued(mesData, 0, YAS532_RAWDATA_OVERFLOW);
    if(ret != 0) {
        isFlowFlg = 1;
    }
    return 0;
}

/* magnetic field data measure(one time call) */
static int shub_diag_mag_data_measure(int32_t* revData)
{
    int ret;
    int temperature;
    int32_t mesData[3];

    /* mag measure one time */
    ret = shub_diag_mag_measure(mesData, &temperature, YAS_PCB_MEASURE_COMMAND_START);
    if(ret != 0) {
        return -1;
    }
    /* temperature revision and coordinate change  */
    shub_diag_mag_data_revision(mesData, temperature, revData);
    return 0;
}

/* acc data measure */
static int shub_diag_acc_data_measure(int16_t* revData)
{
    int ret;

    /* Accelerometer mode setting */
    ret = shub_diag_acc_mode_change(1);
    if(ret != 0) {
        return -1;
    }
    
    usleep(3500);
    
    /* acc measure one time */
    ret = shub_diag_acc_measure(revData);
    if(ret != 0) {
        return -1;
    }
    
    /* Accelerometer mode setting(power down) */
    ret = shub_diag_acc_mode_change(0);
    if(ret != 0) {
        return -1;
    }
    return 0;
}

/* gyro data measure */
static int shub_diag_gyro_data_measure(int16_t* revData)
{
    int ret;

    /* Gyroscope mode setting */
    ret = shub_diag_gyro_mode_change(1);
    if(ret != 0) {
        return -1;
    }
    
    if(gyro_first_read_wait_flg){
        gyro_first_read_wait_flg = 0;
        usleep(100000);
    }
    
    /* gyro measure one time */
    ret = shub_diag_gyro_measure(revData);
    if(ret != 0) {
        return -1;
    }
    return 0;
}

/* self test check */
static int shub_diag_mag_self_test_check(int32_t *data)
{
    int ret;
    int sx;
    int sy;
    int temperature;
    uint8_t cmd;
    int32_t mesDataP[3];
    int32_t mesDataM[3];

    /* initial setting */
    ret = shub_diag_mag_initialize();
    if(ret != 0) {
        return -1;
    }
    /* cal data read */
    ret = shub_diag_mag_read_cal();
    if(ret != 0) {
        return -1;
    }
    /* config data setting */
    ret = shub_diag_mag_set_config();
    if(ret != 0) {
        return -1;
    }

    /* coil data setting */
    ret = shub_diag_mag_set_coil();
    if(ret != 0) {
        return -1;
    }
    /* offset adjust value calc */
    ret = shub_diag_mag_offset_adjust_calc();
    if(ret != 0) {
        return -1;
    }
    /* mag measure */
    cmd = YAS_PCB_MEASURE_COMMAND_START | YAS_PCB_MEASURE_COMMAND_LDTC;
    ret = shub_diag_mag_measure(mesDataP, &temperature, cmd);
    if(ret != 0) {
        return -1;
    }
    /* mag measure */
    cmd = YAS_PCB_MEASURE_COMMAND_START | YAS_PCB_MEASURE_COMMAND_LDTC | YAS_PCB_MEASURE_COMMAND_FORS;
    ret = shub_diag_mag_measure(mesDataM, &temperature, cmd);
    if(ret != 0) {
        return -1;
    }
    /* NHX = k x (Xp-Xm)/1.8 */
    sx = (int)(gstCorrect.s32K * 100 * (mesDataP[0] - mesDataM[0]));
    sx /= 1000;
    sx = (sx * 10) / 18;
    /* NHY = k x a5 x (Yp-Ym)/1.8 */
    sy = (int)(gstCorrect.s32K * gstCorrect.s32A5 * ((mesDataP[1] - mesDataM[1]) - (mesDataP[2] - mesDataM[2])));
    sy /= 1000;
    sy = (sy * 10) / 18;
    /* 17<=NHX 22<=NHY */
/* SHMDS_HUB_0103_05 mod S */
    if((sx >= 17 * 10) && (sy >= 22 * 10)){
/* SHMDS_HUB_0103_05 mod E */
        /* OK */
        printk("[shub]%s self test OK. sx=%d sy=%d\n", __func__, sx, sy);
        ret = 0;
    }
    else{
        /* NG */
        printk("[shub]%s self test NG. sx=%d sy=%d\n", __func__, sx, sy);
        ret = -2;
    }
    data[0] = sx;
    data[1] = sy;
    return ret;
}

/* fw_get_version */
static int shub_diag_fw_get_version(uint8_t *arg_iData)
{
    int32_t ret;
    uint8_t prm[16];
    uint8_t resData[8];

    memset(prm, 0, sizeof(prm));
    ret = shub_direct_sendcmd(HC_MCU_GET_VERSION, prm);
    shub_direct_multi_get_result(resData);
    if(ret != 0) {
        printk("[shub]%s error ret=%d\n", __func__, ret);
        return -1;
    }

    arg_iData[0] = resData[0];
    arg_iData[1] = resData[1];
    arg_iData[2] = resData[2];
    arg_iData[3] = resData[3];
    arg_iData[4] = resData[4];
    arg_iData[5] = resData[5];
    arg_iData[6] = resData[6];
    arg_iData[7] = resData[7];

    printk("FW Version=%02x %02x %02x %02x %02x %02x %02x %02x\n",
        arg_iData[0], arg_iData[1], arg_iData[2], arg_iData[3], arg_iData[4], arg_iData[5], arg_iData[6], arg_iData[7]);
    return 0;
}

/* fw_checksum */
static int shub_diag_fw_checksum(void)
{
    int32_t ret;
    uint8_t prm[16];
    uint8_t resData[2];

    memset(prm, 0, sizeof(prm));
    ret = shub_direct_sendcmd(HC_MCU_SELF_TEST, prm);
    shub_direct_get_result(resData);
    if(ret != 0) {
        printk("[shub]%s error ret=%d\n", __func__, ret);
        ret = -1;
    }
    else if(resData[0] == 0x01) {
        printk("[shub]%s Checksum error.\n", __func__);
        ret = -2;
    }

    return ret;
}

static long shub_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    
//    printk("[shub]shub_ioctl cmd=%d\n", cmd);
    switch (cmd) {
        case SHUB_DIAG_GET_REG:
            {
                uint8_t readData[2];
                struct IoctlDiagRes res;
                
                ret = copy_from_user(&res,argp,sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_GET_REG)\n");
                    return -EFAULT;
                }
                if(res.addr == SHUB_DIAG_6AXIS){
                    res.rtn = shub_diag_accGyroSensorReadSendCmd(res.reg, readData);
                }else if(res.addr == SHUB_DIAG_MAG){
                    res.rtn = shub_diag_magSensorReadSendCmd(res.reg, readData);
                }else if(res.addr == SHUB_DIAG_SHUB){
                    res.rtn = shub_hostif_read(res.reg, readData, 1);
                }else{
                    res.rtn = -10;
                }
                res.data = readData[0];
                
                DBG_DIAG_IO("ioctl(cmd = Get_Reg) : addr=%d, data=...\n", res.addr); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_GET_REG)\n");
                    return -EFAULT;
                }
            }
            break;

        case SHUB_DIAG_SET_REG:
            {
                struct IoctlDiagRes res;
                
                ret = copy_from_user(&res,argp,sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_SET_REG)\n");
                    return -EFAULT;
                }

                DBG_DIAG_IO("ioctl(cmd = Set_Reg) : addr=%d, data=...\n", res.addr); // SHMDS_HUB_0701_01 add
                if(res.addr == SHUB_DIAG_6AXIS){
                    res.rtn = shub_diag_accGyroSensorWriteSendCmd(res.reg, res.data);
                }else if(res.addr == SHUB_DIAG_MAG){
                    res.rtn = shub_diag_magSensorWriteSendCmd(res.reg, res.data);
                }else if(res.addr == SHUB_DIAG_SHUB){
                    res.rtn = shub_hostif_write(res.reg, &res.data, 1);
                }else{
                    res.rtn = -10;
                }
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_SET_REG)\n");
                    return -EFAULT;
                }
            }
            break;
            
        case SHUB_DIAG_GET_PORT_STATE:
            {
                int gpio_state = 0xFF;
                
                ret = shub_get_int_state(&gpio_state);
                if(ret != 0) {
                    return -EFAULT;
                }
                DBG_DIAG_IO("ioctl(cmd = Get_Port_State) : state=%d\n", gpio_state); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &gpio_state, sizeof(gpio_state));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_GET_PORT_STATE)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_MES_GYRO:
            {
                struct IoctlDiagRes res;
                memset(&res, 0, sizeof(struct IoctlDiagRes));
                res.rtn = shub_diag_gyro_initialize();
                if(res.rtn == 0){
                    res.rtn = shub_diag_gyro_data_measure(&res.accGyroData[0]);
                }
                DBG_DIAG_IO("ioctl(cmd = Gyro) : ret=%d, x=%d, y=%d, z=%d\n", 
                             res.rtn, res.accGyroData[0],res.accGyroData[1],res.accGyroData[2]); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_MES_GYRO)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_MES_ACC:
            {
                struct IoctlDiagRes res;
                memset(&res, 0, sizeof(struct IoctlDiagRes));
                
                res.rtn = shub_diag_acc_initialize();
                if(res.rtn == 0){
                    res.rtn = shub_diag_acc_data_measure(&res.accGyroData[0]);
                }
                DBG_DIAG_IO("ioctl(cmd = Acc) : ret=%d, x=%d, y=%d, z=%d\n", 
                             res.rtn, res.accGyroData[0],res.accGyroData[1],res.accGyroData[2]); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_MES_ACC)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_MES_MAG:
            {
                struct IoctlDiagRes res;
                memset(&res, 0, sizeof(struct IoctlDiagRes));
                
                res.rtn = shub_diag_mag_initialize_collect();
                if(res.rtn == 0){
                    res.rtn = shub_diag_mag_data_measure(&res.magData[0]);
                }
                DBG_DIAG_IO("ioctl(cmd = Mag) : ret=%d, x=%d, y=%d, z=%d\n", 
                             res.rtn, res.magData[0],res.magData[1],res.magData[2]); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_MES_MAG)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_MES_MAG_SELF_TEST:
            {
                struct IoctlDiagRes res;
                memset(&res, 0, sizeof(struct IoctlDiagRes));
                
                res.rtn = shub_diag_mag_self_test_check(&res.magData[0]);
                DBG_DIAG_IO("ioctl(cmd = Mag_Self_Test) : ret=%d, val=0x%04x, 0x%04x\n", res.rtn, res.magData[0], res.magData[1]); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_MES_MAG)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_MES_GYRO_CONT:
            {
                struct IoctlDiagRes res;
                memset(&res, 0, sizeof(struct IoctlDiagRes));
                if(!initial_gyro_flg){
                    res.rtn = shub_diag_gyro_initialize();
                    if(res.rtn == 0){
                        initial_gyro_flg = 1;
                    }
                }
                if(res.rtn == 0){
                    res.rtn = shub_diag_gyro_data_measure(&res.accGyroData[0]);
                }
                DBG_DIAG_IO("ioctl(cmd = Gyro_Cont) : ret=%d, x=%d, y=%d, z=%d\n", 
                             res.rtn, res.accGyroData[0],res.accGyroData[1],res.accGyroData[2]); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_MES_GYRO_CONT)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_MES_ACC_CONT:
            {
                struct IoctlDiagRes res;
                memset(&res, 0, sizeof(struct IoctlDiagRes));
                if(!initial_acc_flg){
                    res.rtn = shub_diag_acc_initialize();
                    if(res.rtn == 0){
                        initial_acc_flg = 1;
                    }
                }
                if(res.rtn == 0){
                    res.rtn = shub_diag_acc_data_measure(&res.accGyroData[0]);
                }
                DBG_DIAG_IO("ioctl(cmd = Acc_Cont) : ret=%d, x=%d, y=%d, z=%d\n", 
                             res.rtn, res.accGyroData[0],res.accGyroData[1],res.accGyroData[2]); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_MES_ACC_CONT)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_MES_MAG_CONT:
            {
                struct IoctlDiagRes res;
                memset(&res, 0, sizeof(struct IoctlDiagRes));
                if(!initial_mag_flg){
                    res.rtn = shub_diag_mag_initialize_collect();
                    if(res.rtn == 0){
                        initial_mag_flg = 1;
                    }
                }
                if(res.rtn == 0){
                    res.rtn = shub_diag_mag_data_measure_cont(&res.magData[0]);
                }
                DBG_DIAG_IO("ioctl(cmd = Mag_Cont) : ret=%d, x=%d, y=%d, z=%d\n", 
                             res.rtn, res.magData[0],res.magData[1],res.magData[2]); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_MES_MAG_CONT)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_SENSOR_INIT:
            {
                struct IoctlDiagRes res;
                DBG_DIAG_IO("ioctl(cmd = Sensor_Init)\n"); // SHMDS_HUB_0701_01 add
                memset(&res, 0, sizeof(struct IoctlDiagRes));
                isFlowFlg = 0;
                gyro_first_read_wait_flg = 1;
                read_cal_init_flg = 0;
                initial_gyro_flg = 0;
                initial_acc_flg = 0;
                initial_mag_flg = 0;
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_SENSOR_INIT)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_MAG_READ_CAL:
            {
                struct IoctlDiagMagCalData res;
                memset(&res, 0, sizeof(struct IoctlDiagMagCalData));
                if(read_cal_init_flg == 0){
                    res.rtn = shub_diag_mag_initialize();
                    if(res.rtn == 0){
                        res.rtn = shub_diag_mag_read_cal();
                    }
                }
                if(res.rtn == 0){
                    res.s32Cx = gstCorrect.s32Cx;
                    res.s32Cy1 = gstCorrect.s32Cy1;
                    res.s32Cy2 = gstCorrect.s32Cy2;
                    res.s32A2 = gstCorrect.s32A2;
                    res.s32A3 = gstCorrect.s32A3;
                    res.s32A4 = gstCorrect.s32A4;
                    res.s32A5 = gstCorrect.s32A5;
                    res.s32A6 = gstCorrect.s32A6;
                    res.s32A7 = gstCorrect.s32A7;
                    res.s32A8 = gstCorrect.s32A8;
                    res.s32A9 = gstCorrect.s32A9;
                    res.s32K = gstCorrect.s32K;
                }
                DBG_DIAG_IO("ioctl(cmd = Mag_Read_Cal) : ret=%d, data=...\n", res.rtn); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagMagCalData));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_SENSOR_INIT)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_FW_GET_VERSION:
            {
                struct IoctlDiagFirmwareVersion res;
                memset(&res, 0, sizeof(struct IoctlDiagFirmwareVersion));
                
                res.rtn = shub_diag_fw_get_version(&res.data[0]);
                DBG_DIAG_IO("ioctl(cmd = FW_Get_Version) : ret=%d, data=...\n", res.rtn); // SHMDS_HUB_0701_01 mod
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagFirmwareVersion));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_FW_GET_VERSION)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_FW_CHECKSUM:
            {
                struct IoctlDiagRes res;
                memset(&res, 0, sizeof(struct IoctlDiagRes));
                
                res.rtn = shub_diag_fw_checksum();
                DBG_DIAG_IO("ioctl(cmd = FW_CheckSum) : ret=%d\n", res.rtn); // SHMDS_HUB_0701_01 mod
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_FW_CHECKSUM)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_MES_ONLY_GYRO:
            {
                struct IoctlDiagRes res;
                memset(&res, 0, sizeof(struct IoctlDiagRes));
                if(!initial_gyro_flg){
                    res.rtn = shub_diag_gyro_initialize();
                    if(res.rtn == 0){
                        initial_gyro_flg = 1;
                    }
                }
                if(res.rtn == 0){
                    res.rtn = shub_diag_gyro_measure(&res.accGyroData[0]);
                }
                DBG_DIAG_IO("ioctl(cmd = Only_Gyro) : ret=%d, x=%d, y=%d, z=%d\n", 
                             res.rtn, res.accGyroData[0],res.accGyroData[1],res.accGyroData[2]); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_MES_ONLY_GYRO)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_MES_ONLY_ACC:
            {
                struct IoctlDiagRes res;
                memset(&res, 0, sizeof(struct IoctlDiagRes));
                if(!initial_acc_flg){
                    res.rtn = shub_diag_acc_initialize();
                    if(res.rtn == 0){
                        initial_acc_flg = 1;
                    }
                }
                if(res.rtn == 0){
                    res.rtn = shub_diag_acc_measure(&res.accGyroData[0]);
                }
                DBG_DIAG_IO("ioctl(cmd = Only_Acc) : ret=%d, x=%d, y=%d, z=%d\n", 
                             res.rtn, res.accGyroData[0],res.accGyroData[1],res.accGyroData[2]); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagRes));
                if (ret) {
                    printk("error : shub_ioctl(cmd = SHUB_DIAG_MES_ONLY_ACC)\n");
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_CMD_WRITE:
            {
                struct IoctlDiagCmdReq cmdreq;
                
                ret = copy_from_user(&cmdreq, argp, sizeof(struct IoctlDiagCmdReq));
                if (ret) {
                    printk("error(copy_from_user) : shub_ioctl(cmd = SHUB_DIAG_CMD_WRITE)\n" );
                    return -EFAULT;
                }
                ret = shub_cmd_wite( &cmdreq );
                cmdreq.rtn = ret;
                DBG_DIAG_IO("ioctl(cmd = Cmd_Write) : ret=%d, cmd=0x%04x, size=%d, data=...\n", 
                             cmdreq.rtn, cmdreq.m_Cmd, cmdreq.m_req_size); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &cmdreq, sizeof(struct IoctlDiagCmdReq));
                if (ret) {
                    printk("error(copy_to_user) : shub_ioctl(cmd = SHUB_DIAG_CMD_WRITE)\n" );
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_CMD_READ:
            {
                struct IoctlDiagCmdReq cmdreq;
                
                ret = copy_from_user(&cmdreq, argp, sizeof(struct IoctlDiagCmdReq));
                if (ret) {
                    printk("error(copy_from_user) : shub_ioctl(cmd = SHUB_DIAG_CMD_READ)\n" );
                    return -EFAULT;
                }
                ret = shub_cmd_read( &cmdreq );
                cmdreq.rtn = ret;
                DBG_DIAG_IO("ioctl(cmd = Cmd_Read) : ret=%d, cmd=0x%04x, req_size=%d, res_size=%d, data=...\n", 
                             cmdreq.rtn, cmdreq.m_Cmd, cmdreq.m_req_size, cmdreq.m_res_size); // SHMDS_HUB_0701_01 add
                ret = copy_to_user(argp, &cmdreq, sizeof(struct IoctlDiagCmdReq));
                if (ret) {
                    printk("error(copy_to_user) : shub_ioctl(cmd = SHUB_DIAG_CMD_READ)\n" );
                    return -EFAULT;
                }
            }
            break;
/* SHMDS_HUB_0103_06 add S */
        case SHUB_DIAG_ACC_SET_CAL:
            {
                struct IoctlDiagAccCalibration res;
                
                ret = copy_from_user(&res,argp,sizeof(struct IoctlDiagAccCalibration));
                if (ret) {
                    printk("error : copy_from_user(cmd = SHUB_DIAG_ACC_SET_CAL)ret=%d\n", ret);
                    return -EFAULT;
                }
                
                DBG_DIAG_IO("ioctl(cmd = Acc_set_call) : x=%d, y=%d, z=%d\n", res.AccCal[0], res.AccCal[1], res.AccCal[2]); // SHMDS_HUB_0701_01 add
                ret = shub_set_acc_offset(&res.AccCal[0]);
                if (ret) {
                    printk("error : shub_set_acc_offset(cmd = SHUB_DIAG_ACC_SET_CAL)ret=%d\n", ret);
                    return -EFAULT;
                }
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagAccCalibration));
                if (ret) {
                    printk("error : copy_to_user(cmd = SHUB_DIAG_ACC_SET_CAL)ret=%d\n", ret);
                    return -EFAULT;
                }
            }
            break;
        case SHUB_DIAG_MAG_SET_CAL:
            {
                struct IoctlDiagMagCalibration res;
                
                ret = copy_from_user(&res,argp,sizeof(struct IoctlDiagMagCalibration));
                if (ret) {
                    printk("error : copy_from_user(cmd = SHUB_DIAG_MAG_SET_CAL)ret=%d\n", ret);
                    return -EFAULT;
                }
                
                DBG_DIAG_IO("ioctl(cmd = Mag_set_call) : data=...\n"); // SHMDS_HUB_0701_01 add
                ret = shub_cal_mag_axis_interfrence(&res.MagCal[0]);
                if (ret) {
                    printk("error : shub_cal_mag_axis_interfrence(cmd = SHUB_DIAG_MAG_SET_CAL)ret=%d\n", ret);
                    return -EFAULT;
                }
                
                ret = copy_to_user(argp, &res, sizeof(struct IoctlDiagMagCalibration));
                if (ret) {
                    printk("error : copy_to_user(cmd = SHUB_DIAG_MAG_SET_CAL)ret=%d\n", ret);
                    return -EFAULT;
                }
            }
            break;
/* SHMDS_HUB_0103_06 add E */
        default:
            return -ENOTTY;
    }
    return 0;
}

static int32_t shub_io_open( struct inode* inode, struct file* filp )
{
    return 0;
}

static int32_t shub_io_release( struct inode* inode, struct file* filp )
{
    return 0;
}

static unsigned int shub_io_poll(struct file *fp, poll_table *wait)
{
    return 0;
}

static ssize_t shub_io_read(struct file *fp, char *buff, size_t count, loff_t *ops)
{
    return 0;
}

static struct file_operations shub_fops = {
    .owner   = THIS_MODULE,
    .open    = shub_io_open,
    .release = shub_io_release,
    .unlocked_ioctl = shub_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = shub_ioctl,
#endif
    .poll    = shub_io_poll,
    .read    = shub_io_read,
};

static struct miscdevice shub_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "shub_io_diag",
    .fops  = &shub_fops,
};

static int32_t __init shub_init(void)
{
    int32_t ret = 0;

    pdev = platform_device_register_simple("shub_io_diag", -1, NULL, 0);
    if (IS_ERR(pdev)) {
        ret = PTR_ERR(pdev);
        goto out_driver;
    }
//  printk(KERN_INFO "shub_io_diag: platform_device_register_simple\n"); // SHMDS_HUB_0106_06 del

    ret = misc_register(&shub_device);
    if (ret) {
        printk("shub_io_diag: shub_io_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    return 0;

exit_misc_device_register_failed:
    input_free_device(shub_idev);
//  printk(KERN_INFO "shub_io_diag: input_free_polled_device\n"); // SHMDS_HUB_0106_06 del
    platform_device_unregister(pdev);
//  printk(KERN_INFO "shub_io_diag: platform_device_unregister\n"); // SHMDS_HUB_0106_06 del
out_driver:
    // out_region:
    return ret;
}

static void __exit shub_exit(void)
{
    misc_deregister(&shub_device);
//  printk(KERN_INFO "shub_io_diag: misc_deregister\n"); // SHMDS_HUB_0106_06 del
    input_unregister_device(shub_idev);
//  printk(KERN_INFO "shub_io_diag: input_unregister_polled_device\n"); // SHMDS_HUB_0106_06 del
    input_free_device(shub_idev);
//  printk(KERN_INFO "shub_io_diag: input_free_polled_device\n"); // SHMDS_HUB_0106_06 del
    platform_device_unregister(pdev);
//  printk(KERN_INFO "shub_io_diag: platform_device_unregister\n"); // SHMDS_HUB_0106_06 del
}

module_init(shub_init);
module_exit(shub_exit);

MODULE_DESCRIPTION("SensorHub Input Device (misc)");
MODULE_AUTHOR("LAPIS SEMICOMDUCTOR");
MODULE_LICENSE("GPL v2");
