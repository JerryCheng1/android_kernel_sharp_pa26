/* include/sharp/shub_driver.h  (Shub Driver)
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

#ifndef SHUB_DRIVER_H
#define SHUB_DRIVER_H

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHUB_STOP_PED_TYPE_VIB,
    SHUB_STOP_PED_TYPE_TPS,
    NUM_SHUB_STOP_PED_TYPE
};

/* ------------------------------------------------------------------------- */
/* STRUCT                                                                    */
/* ------------------------------------------------------------------------- */
// SHMDS_HUB_1301_01 add S
struct shub_face_acc_info {
    int nJudge;
    int nStat;
    int32_t nX;
    int32_t nY;
    int32_t nZ;
};
// SHMDS_HUB_1301_01 add E

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int shub_api_stop_pedometer_func (int type);
int shub_api_restart_pedometer_func (int type);
int shub_api_get_face_down_info(struct shub_face_acc_info *info); // SHMDS_HUB_1301_01 add

#endif /* SHUB_DRIVER_H */
