/*
 *  ml630q790.h - Linux kernel modules for Sensor Hub 
 *
 *  Copyright (C) 2012-2014 LAPIS SEMICONDUCTOR CO., LTD.
 *
 *  This file is based on :
 *    ml610q792.h - Linux kernel modules for acceleration sensor
 *    http://android-dev.kyocera.co.jp/source/versionSelect_URBANO.html
 *    (C) 2012 KYOCERA Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _ML630Q790_H_
#define _ML630Q790_H_
#ifndef NO_LINUX
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/poll.h>
#endif
#include "shub_io.h"

#define SENOSR_HUB_I2C_SLAVE_ADDRESS (0x2f)
#define SENOSR_HUB_DRIVER_NAME    "sensorhub"

#define SHUB_ACTIVE_ACC                   (0x00000010)
#define SHUB_ACTIVE_PEDOM                 (0x00000020)
#define SHUB_ACTIVE_PEDOM_NO_NOTIFY       (0x00000040)
#define SHUB_ACTIVE_EXT_PEDOM             (0x00000080)
#define SHUB_ACTIVE_EXT_TOTAL_STATUS      (0x00000100)

/* SHMDS_HUB_0201_01 add S */
#define SHUB_ACTIVE_SHEX_ACC              (0x00000800)
/* SHMDS_HUB_0201_01 add E */

#define SHUB_ACTIVE_GYRO                  (0x00001000)
#define SHUB_ACTIVE_MAG                   (0x00002000)

#define SHUB_ACTIVE_ORI                   (0x00008000)
#define SHUB_ACTIVE_GRAVITY               (0x00010000)
#define SHUB_ACTIVE_LACC                  (0x00020000)
#define SHUB_ACTIVE_RV                    (0x00040000)
#define SHUB_ACTIVE_RV_NONMAG             (0x00080000)
#define SHUB_ACTIVE_RV_NONGYRO            (0x00100000)

#define SHUB_ACTIVE_GYROUNC               (0x00400000)
#define SHUB_ACTIVE_MAGUNC                (0x00800000)
#define SHUB_ACTIVE_PEDODEC               (0x01000000)
#define SHUB_ACTIVE_SIGNIFICANT           (0x02000000)

#define SHUB_ACTIVE_GDEC                  (0x08000000)
#define SHUB_ACTIVE_MOTIONDEC             (0x10000000)
#define SHUB_ACTIVE_PEDODEC_NO_NOTIFY     (0x20000000)

#define SHUB_ACTIVE_ERROR                 (0x80000000)

/* SHMDS_HUB_0201_01 SHMDS_HUB_0206_02 SHUB_ACTIVE_SHEX_ACC mod S */
#define ACTIVE_FUNC_MASK (\
        SHUB_ACTIVE_ACC                   | \
        SHUB_ACTIVE_PEDOM                 | \
        SHUB_ACTIVE_PEDOM_NO_NOTIFY       | \
        SHUB_ACTIVE_SHEX_ACC              | \
        SHUB_ACTIVE_SIGNIFICANT           | \
        SHUB_ACTIVE_GYRO                  | \
        SHUB_ACTIVE_MAG                   | \
        SHUB_ACTIVE_ORI                   | \
        SHUB_ACTIVE_GRAVITY               | \
        SHUB_ACTIVE_LACC                  | \
        SHUB_ACTIVE_RV                    | \
        SHUB_ACTIVE_RV_NONMAG             | \
        SHUB_ACTIVE_RV_NONGYRO            | \
        SHUB_ACTIVE_GYROUNC               | \
        SHUB_ACTIVE_MAGUNC                | \
        SHUB_ACTIVE_MOTIONDEC             | \
        SHUB_ACTIVE_GDEC                  | \
        SHUB_ACTIVE_PEDODEC_NO_NOTIFY     | \
        SHUB_ACTIVE_EXT_PEDOM             | \
        SHUB_ACTIVE_EXT_TOTAL_STATUS      | \
        SHUB_ACTIVE_PEDODEC)

#define APPTASK_GROUP_MASK (\
        SHUB_ACTIVE_PEDOM                 | \
        SHUB_ACTIVE_PEDOM_NO_NOTIFY       | \
        SHUB_ACTIVE_SIGNIFICANT           | \
        SHUB_ACTIVE_MOTIONDEC             | \
        SHUB_ACTIVE_EXT_PEDOM             | \
        SHUB_ACTIVE_PEDODEC_NO_NOTIFY     | \
        SHUB_ACTIVE_PEDODEC)
/* SHMDS_HUB_0201_01 mod E */

#define GYRO_GROUP_MASK (SHUB_ACTIVE_GYRO | SHUB_ACTIVE_GYROUNC )
#define MAG_GROUP_MASK (SHUB_ACTIVE_MAG | SHUB_ACTIVE_MAGUNC )

#define PEDOM_GROUP_MASK (\
        SHUB_ACTIVE_SIGNIFICANT           | \
        SHUB_ACTIVE_PEDOM                 | \
        SHUB_ACTIVE_PEDOM_NO_NOTIFY       | \
        SHUB_ACTIVE_PEDODEC_NO_NOTIFY     | \
        SHUB_ACTIVE_PEDODEC)

#define STEPCOUNT_GROUP_MASK (\
        SHUB_ACTIVE_PEDOM                 | \
        SHUB_ACTIVE_PEDOM_NO_NOTIFY)

#define STEPDETECT_GROUP_MASK (\
        SHUB_ACTIVE_PEDODEC_NO_NOTIFY     | \
        SHUB_ACTIVE_PEDODEC)

#define FUSION_MAG_GROUP_MASK (\
        SHUB_ACTIVE_RV_NONGYRO )

#define FUSION_GYRO_GROUP_MASK (\
        SHUB_ACTIVE_RV_NONMAG)

#define FUSION9AXIS_GROUP_MASK (\
        SHUB_ACTIVE_ORI          |\
        SHUB_ACTIVE_GRAVITY      |\
        SHUB_ACTIVE_LACC         |\
        SHUB_ACTIVE_RV)

#define FUSION_GROUP_MASK (FUSION_MAG_GROUP_MASK | FUSION9AXIS_GROUP_MASK | FUSION_GYRO_GROUP_MASK)

/* 30min */
#define SHUB_TIMER_MAX            (30*60*1000)

/*
   Logging (batch mode)
   */
#define LOGGING_RAM_SIZE          (3*1024)
#define FIFO_SIZE                 (512)
#define PRM_SIZE                  (16)

// data size
#define DATA_SIZE_ACC                7
#define DATA_SIZE_MAG                8
#define DATA_SIZE_GYRO               7
#define DATA_SIZE_MAG_CAL_OFFSET     6
#define DATA_SIZE_GYRO_CAL_OFFSET    6
#define DATA_SIZE_ORI                8
#define DATA_SIZE_GRAVITY            7
#define DATA_SIZE_LINEARACC          7
#define DATA_SIZE_RVECT              9
#define DATA_SIZE_GEORV              10
#define DATA_SIZE_GAMERV             9
#define DATA_SIZE_PEDOCNT            10
#define DATA_SIZE_TOTALSTATUS        9 

#define TIMER_RESO 5

// SHMDS_HUB_0601_01 add S
enum{
    SHUB_INPUT_ACC,                 // [----] Accelerometer
    SHUB_INPUT_MAG,                 // [COM1] Magnetic Filed sensor
    SHUB_INPUT_GYRO,                // [COM2] Gyroscope
    SHUB_INPUT_GRAVITY,             // [COM3] Gravity
    SHUB_INPUT_LINEARACC,           // [COM3] Linear Acceleration
    SHUB_INPUT_ORI,                 // [----] Orientation
    SHUB_INPUT_RVECT,               // [----] Rotation Vector
    SHUB_INPUT_SIGNIFICANT,         // [COM5] Significant Motion
    SHUB_INPUT_GYROUNC,             // [COM2] Gyroscope Uncalibrated
    SHUB_INPUT_MAGUNC,              // [COM1] Magnetic Field sensor Uncalibrated
    SHUB_INPUT_GAMEROT,             // [COM4] Game Rotation Vector
    SHUB_INPUT_PEDO,                // [COM5] Step Counter
    SHUB_INPUT_PEDODET,             // [COM5] Step Detector
    SHUB_INPUT_GEORV,               // [COM4] Geomegnetic Rotation Vector
    SHUB_INPUT_MAXNUM,              // max number 
    SHUB_INPUT_META_DATA = 0x0100   // METADATA SHMDS_HUB_0601_05
};
// SHMDS_HUB_0601_01 add E

// SHMDS_HUB_1101_01 add S
void shub_qos_start(void);
void shub_qos_end(void);
// SHMDS_HUB_1101_01 add E

#ifdef CONFIG_HOSTIF_I2C
int32_t shub_suspend( struct i2c_client *client, pm_message_t mesg );
int32_t shub_resume( struct i2c_client *client );
#endif
#ifdef CONFIG_HOSTIF_SPI
int32_t shub_suspend( struct spi_device *client, pm_message_t mesg );
int32_t shub_resume( struct spi_device *client );
#endif
int32_t shub_initialize( void );
int32_t shub_update_fw(bool boot, uint8_t *arg_iDataPage1, uint8_t *arg_iDataPage2, uint32_t arg_iLen);
int32_t shub_get_fw_version(uint8_t *arg_iData);
int32_t shub_get_sensors_data(int32_t type, int32_t* data);
int32_t shub_activate(int32_t arg_iSensType, int32_t arg_iEnable);
int32_t shub_activate_logging(int32_t arg_iSensType, int32_t arg_iEnable);
int32_t shub_set_delay(int32_t arg_iSensType, int32_t delay);
int32_t shub_set_delay_logging(int32_t arg_iSensType, int32_t delay);
int32_t shub_get_current_active(void);
int32_t shub_get_current_active_logging(void);
int32_t shub_direct_hostcmd(uint16_t cmd, const uint8_t *prm, uint8_t *rslt);
int32_t shub_logging_flush(void);
void shub_logging_clear(void);
int32_t shub_get_pedo_event( void );
int32_t shub_probe(void);
void shub_debug_level_chg(int32_t lv);

int32_t shub_set_param(int32_t type,int32_t* data);
int32_t shub_get_param(int32_t type,int32_t* data);

int32_t shub_init_app(int32_t type);
int32_t shub_clear_data(int32_t type);
int32_t shub_get_data_pedometer(int32_t* data);
int32_t shub_get_data_normal_pedometer(int32_t* data);
int32_t shub_get_data_act_detection(int32_t* data);
int32_t shub_get_data_low_power(int32_t* data);
int32_t shub_get_data_motion(int32_t* data);

int32_t shub_read_sensor(uint8_t type, uint8_t addr ,uint8_t *data);
int32_t shub_write_sensor(uint8_t type, uint8_t addr ,uint8_t data);
int32_t shub_set_acc_offset(int32_t* offsets);
int32_t shub_get_acc_offset(int32_t* offsets);
int32_t shub_set_acc_position(int32_t type);
int32_t shub_set_mag_offset(int32_t* offsets);
int32_t shub_get_mag_offset(int32_t* offsets);
int32_t shub_set_mag_position(int32_t type);
int32_t shub_cal_mag_axis_interfrence(int32_t* mat);
int32_t shub_set_gyro_offset(int32_t* offsets);
int32_t shub_get_gyro_offset(int32_t* offsets);
int32_t shub_set_gyro_position(int32_t type);
int32_t shub_adjust_value(int32_t min,int32_t max,int32_t value);
/* SHMDS_HUB_0103_01 add S */
int32_t shub_direct_sendcmd(uint16_t cmd, const uint8_t *prm);
int32_t shub_noreturn_sendcmd(uint16_t cmd, const uint8_t *prm);
int32_t shub_direct_get_error(uint16_t *error);
int32_t shub_direct_get_result(uint8_t *result);
int32_t shub_direct_multi_get_result(uint8_t *result);
int32_t shub_hostif_write(uint8_t adr, const uint8_t *data, uint16_t size);
int32_t shub_hostif_read(uint8_t adr, uint8_t *data, uint16_t size);
int32_t shub_cmd_wite(struct IoctlDiagCmdReq *arg_cmd);
int32_t shub_cmd_read(struct IoctlDiagCmdReq *arg_cmd);
int32_t shub_get_int_state(int *state);
/* SHMDS_HUB_0103_01 add E */

/* Input Subsystem */
void shub_input_report_acc(int32_t *data);
void shub_input_report_mag(int32_t *data);
void shub_input_report_gyro(int32_t *data);
void shub_input_report_gyro_uncal(int32_t *data);
void shub_input_report_mag_uncal(int32_t *data);
void shub_input_report_orien(int32_t *data);
void shub_input_report_grav(int32_t *data);
void shub_input_report_linear(int32_t *data);
void shub_input_report_rot(int32_t *data);
void shub_input_report_rot_gyro(int32_t *data);
void shub_input_report_rot_mag(int32_t *data);
void shub_input_report_stepdetect(int32_t *data);
void shub_input_report_stepcnt(int32_t *data);
void shub_input_report_significant(int32_t *data);
bool shub_fw_update_check(void);
bool shub_connect_check(void);
/* SHMDS_HUB_0201_01 add S */
void shub_input_report_exif_grav_det(bool send);
void shub_input_report_exif_mot_det(unsigned char det_info);
void shub_input_report_exif_shex_acc(int32_t *data);
void shub_input_report_exif_judge(void);
int shub_set_default_parameter(void);
/* SHMDS_HUB_0201_01 add E */
/* SHMDS_HUB_0202_01 add S */
void shub_set_already_md_flg(int data);
void shub_clr_already_md_flg(int data);
int shub_get_already_md_flg(void);
/* SHMDS_HUB_0202_01 add E */
/* SHMDS_HUB_0109_02 add S */
int shub_get_acc_axis_val(void);
int shub_get_gyro_axis_val(void);
int shub_get_mag_axis_val(void);
/* SHMDS_HUB_0109_02 add E */
/* SHMDS_HUB_0601_01 add S */
struct input_dev *shub_com_allocate_device(int inp_type, struct device *dev);
void shub_com_unregister_device(int inp_type);
/* SHMDS_HUB_0601_01 add E */
int shub_get_exif_delay_ms(void);   // SHMDS_HUB_0206_03 add
void shub_set_param_check_exif(int type, int *data);    // SHMDS_HUB_0207_01 add
void shub_get_param_check_exif(int type, int *data);    // SHMDS_HUB_0207_01 add
void shub_set_enable_ped_exif_flg(int en);              // SHMDS_HUB_0207_01 add
int shub_get_mcu_ped_enable(void);                      // SHMDS_HUB_0206_06 add

void shub_suspend_acc(void);
void shub_suspend_mag(void);
void shub_suspend_mag_uncal(void);
void shub_suspend_gyro(void);
void shub_suspend_gyro_uncal(void);
void shub_suspend_orien(void);
void shub_suspend_grav(void);
void shub_suspend_linear(void);
void shub_suspend_rot(void);
void shub_suspend_rot_gyro(void);
void shub_suspend_rot_mag(void);
void shub_suspend_exif(void);   // SHMDS_HUB_0203_01 add

void shub_resume_acc(void);
void shub_resume_mag(void);
void shub_resume_mag_uncal(void);
void shub_resume_gyro(void);
void shub_resume_gyro_uncal(void);
void shub_resume_orien(void);
void shub_resume_grav(void);
void shub_resume_linear(void);
void shub_resume_rot(void);
void shub_resume_rot_gyro(void);
void shub_resume_rot_mag(void);
void shub_resume_exif(void);    // SHMDS_HUB_0203_01 add

#endif /* _ML630Q790_H_ */
