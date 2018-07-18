/* include/sharp/proximity_cdc.h  (proximity cdc Sensor Driver)
 *
 * Copyright (C) 2013 SHARP CORPORATION All rights reserved.
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
/* SHARP PROXIMITY SENSOR DRIVER FOR KERNEL MODE                             */
/* ------------------------------------------------------------------------- */

#ifndef PROXIMITY_CDC_H
#define PROXIMITY_CDC_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/ioctl.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

/* IOCTLs for PROXIMITYCDC */
#define PROXIMITYCDC                        0xA2

#define CDC_IOCTL_ENABLE                    _IO(PROXIMITYCDC,   0x01)
#define CDC_IOCTL_DISABLE                   _IO(PROXIMITYCDC,   0x02)
#define CDC_IOCTL_READ_DATA                 _IOR(PROXIMITYCDC,  0x03, proximity_cdc_ioctl)
#define CDC_IOCTL_AMBIENT_READ              _IOWR(PROXIMITYCDC, 0x04, proximity_cdc_ioctl)
#define CDC_IOCTL_AMBIENT_RESET             _IO(PROXIMITYCDC,   0x05)
#define CDC_IOCTL_POS_THRESHOLD_READ        _IOWR(PROXIMITYCDC, 0x06, proximity_cdc_ioctl)
#define CDC_IOCTL_NEG_THRESHOLD_READ        _IOWR(PROXIMITYCDC, 0x07, proximity_cdc_ioctl)
#define CDC_IOCTL_POS_THRESHOLD_SENS_WRITE  _IOW(PROXIMITYCDC,  0x08, proximity_cdc_ioctl)
#define CDC_IOCTL_NEG_THRESHOLD_SENS_WRITE  _IOW(PROXIMITYCDC,  0x09, proximity_cdc_ioctl)
#define CDC_IOCTL_RESULT_READ               _IOWR(PROXIMITYCDC, 0x0A, proximity_cdc_ioctl)
#define CDC_IOCTL_STAGE_READ_DATA           _IOWR(PROXIMITYCDC, 0x0B, proximity_cdc_ioctl)
#define CDC_IOCTL_OFFSET_ADJ_WRITE          _IOW(PROXIMITYCDC,  0x0C, proximity_cdc_ioctl)
#define CDC_IOCTL_GET_STATUS                _IOWR(PROXIMITYCDC, 0x0D, struct proximity_cdc_status)
#define CDC_IOCTL_POWERMODE                 _IOW(PROXIMITYCDC,  0x0E, proximity_cdc_ioctl)

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SH_PROXIMITY_CDC_RESULT_SUCCESS = 0,
    SH_PROXIMITY_CDC_RESULT_FAILURE,
    SH_PROXIMITY_CDC_RESULT_FAILURE_USER,
    NUM_SH_PROXIMITY_CDC_RESULT
};

enum {
    SH_PROXIMITY_CDC_STAGE0 = 0,
    SH_PROXIMITY_CDC_STAGE1,
    SH_PROXIMITY_CDC_STAGE2,
    SH_PROXIMITY_CDC_STAGE3,
    SH_PROXIMITY_CDC_STAGE4,
    SH_PROXIMITY_CDC_STAGE5,
    SH_PROXIMITY_CDC_STAGE6,
    SH_PROXIMITY_CDC_STAGE7,
    SH_PROXIMITY_CDC_STAGE8,
    SH_PROXIMITY_CDC_STAGE9,
    SH_PROXIMITY_CDC_STAGE10,
    SH_PROXIMITY_CDC_STAGE11,
    NUM_SH_PROXIMITY_CDC_STAGE
};

enum {
    SH_PROXIMITY_CDC_SENS_25_00 = 0,
    SH_PROXIMITY_CDC_SENS_29_73,
    SH_PROXIMITY_CDC_SENS_34_40,
    SH_PROXIMITY_CDC_SENS_39_08,
    SH_PROXIMITY_CDC_SENS_43_79,
    SH_PROXIMITY_CDC_SENS_48_47,
    SH_PROXIMITY_CDC_SENS_53_15,
    SH_PROXIMITY_CDC_SENS_57_83,
    SH_PROXIMITY_CDC_SENS_62_51,
    SH_PROXIMITY_CDC_SENS_67_22,
    SH_PROXIMITY_CDC_SENS_71_90,
    SH_PROXIMITY_CDC_SENS_76_58,
    SH_PROXIMITY_CDC_SENS_81_28,
    SH_PROXIMITY_CDC_SENS_85_96,
    SH_PROXIMITY_CDC_SENS_90_64,
    SH_PROXIMITY_CDC_SENS_95_32,
    NUM_SH_PROXIMITY_CDC_SENS
};

enum {
    SH_PROXIMITY_CDC_STATUS_CLOSE = 0,
    SH_PROXIMITY_CDC_STATUS_OPEN,
    NUM_SH_PROXIMITY_CDC_OPEN_STATUS,
};

enum {
    SH_PROXIMITY_CDC_STATUS_DISABLE = 0,
    SH_PROXIMITY_CDC_STATUS_ENABLE,
    NUM_SH_PROXIMITY_CDC_ENABLE_STATUS,
};

enum {
    SH_PROXIMITY_CDC_FULLSHUTDOWN_MODE = 0,
    SH_PROXIMITY_CDC_FULLPOWER_MODE,
    SH_PROXIMITY_CDC_LOWPOWER_MODE,
    NUM_SH_PROXIMITY_CDC_POWERMODE,
};

/* ------------------------------------------------------------------------- */
/* STRUCT                                                                    */
/* ------------------------------------------------------------------------- */

struct proximity_cdc_status{
    unsigned char open_status;
    unsigned char enable_status;
};

typedef struct{
    short stage;
    short data;
    char  mode;
}proximity_cdc_ioctl;


#endif /* PROXIMITY_CDC_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
