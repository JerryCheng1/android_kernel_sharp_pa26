/* drivers/sharp/shtps/sy3000/shtps_fwctl.c
 *
 * Copyright (c) 2014, Sharp. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <sharp/shtps_dev.h>

#include "shtps_rmi.h"
#include "shtps_fwctl.h"
#include "shtps_log.h"
#include "shtps_param_extern.h"

/* -------------------------------------------------------------------------- */
extern struct shtps_fwctl_functbl		shtps_fwctl_s3400_function_table;

struct shtps_fwctl_functbl	*shtps_fwctl_function[]={
	&shtps_fwctl_s3400_function_table,
	NULL,
};

/* -------------------------------------------------------------------------- */
int shtps_fwctl_init(struct shtps_rmi_spi *ts_p, void *tps_ctrl_p, struct shtps_ctrl_functbl *func_p)
{
	ts_p->fwctl_p = kzalloc(sizeof(struct shtps_fwctl_info), GFP_KERNEL);
	if(ts_p->fwctl_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return -ENOMEM;
	}
	ts_p->fwctl_p->fwctl_ic_type = 0;
	ts_p->fwctl_p->fwctl_func_p = shtps_fwctl_function[ ts_p->fwctl_p->fwctl_ic_type ];
	ts_p->fwctl_p->devctrl_func_p = func_p;	/* func.table spi */
	ts_p->fwctl_p->tps_ctrl_p = tps_ctrl_p;	/* struct device */

	ts_p->fwctl_p->map_p = shtps_fwctl_ic_init(ts_p);
	if(ts_p->fwctl_p->map_p == NULL){
		PR_ERROR("map memory allocation error:%s()\n", __func__);
		kfree(ts_p->fwctl_p);
		return -ENOMEM;
	}

	return 0;
}

/* -------------------------------------------------------------------------- */
void shtps_fwctl_deinit(struct shtps_rmi_spi *ts_p)
{
	if(ts_p->fwctl_p->map_p)	kfree(ts_p->fwctl_p->map_p);
	ts_p->fwctl_p->map_p = NULL;

	if(ts_p->fwctl_p)	kfree(ts_p->fwctl_p);
	ts_p->fwctl_p = NULL;
}

/* -------------------------------------------------------------------------- */
#ifdef	SHTPS_BLD_FWCTL_FUNC_MACRO_ENABLE
	/* nothing */
#else
/* -------------------------------------------------------------------------- */
struct rmi_map* shtps_fwctl_ic_init(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->ic_init_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_init_writeconfig(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_init_writeconfig_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_get_config_blocknum(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_get_config_blocknum_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_get_firm_blocknum(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_get_firm_blocknum_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_get_blocksize(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_get_blocksize_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_get_result_writeconfig(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_get_result_writeconfig_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_get_result_writeimage(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_get_result_writeimage_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_get_result_erase(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_get_result_erase_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_write_config(struct shtps_rmi_spi *ts_p, u8 *fwdata, int blockSize)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_write_config_f(ts_p->fwctl_p, fwdata, blockSize);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_write_image(struct shtps_rmi_spi *ts_p, u8 *fwdata, int blockSize)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_write_image_f(ts_p->fwctl_p, fwdata, blockSize);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_cmd(struct shtps_rmi_spi *ts_p, u8 cmd, u8 isLockdown)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_cmd_f(ts_p->fwctl_p, cmd, isLockdown);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_cmd_erase(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_cmd_erase_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_cmd_writeimage(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_cmd_writeimage_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_cmd_writeconfig(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_cmd_writeconfig_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_cmd_enterbl(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_cmd_enterbl_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_loader_exit(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->loader_exit_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_device_status(struct shtps_rmi_spi *ts_p, u8 *status)
{
	return ts_p->fwctl_p->fwctl_func_p->get_device_status_f(ts_p->fwctl_p, status);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_soft_reset(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->soft_reset_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_irqclear_get_irqfactor(struct shtps_rmi_spi *ts_p, u8 *status)
{
	return ts_p->fwctl_p->fwctl_func_p->irqclear_get_irqfactor_f(ts_p->fwctl_p, status);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_rezero(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->rezero_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_map_construct(struct shtps_rmi_spi *ts_p, int func_check)
{
	return ts_p->fwctl_p->fwctl_func_p->map_construct_f(ts_p->fwctl_p, func_check);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_is_sleeping(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->is_sleeping_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_is_singlefinger(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->is_singlefinger_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_reg_read_pen_jitter(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->reg_read_pen_jitter_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_reg_write_pen_jitter(struct shtps_rmi_spi *ts_p, u8 *val)
{
	return ts_p->fwctl_p->fwctl_func_p->reg_write_pen_jitter_f(ts_p->fwctl_p, val);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_reg_read_segmentation_aggressiveness(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->reg_read_segmentation_aggressiveness_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_reg_write_segmentation_aggressiveness(struct shtps_rmi_spi *ts_p, u8 *val)
{
	return ts_p->fwctl_p->fwctl_func_p->reg_write_segmentation_aggressiveness_f(ts_p->fwctl_p, val);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_reg_read_pixel_touch_threshold(struct shtps_rmi_spi *ts_p, u8 threshold)
{
	return ts_p->fwctl_p->fwctl_func_p->reg_read_pixel_touch_threshold_f(ts_p->fwctl_p, threshold);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_reg_write_pixel_touch_threshold(struct shtps_rmi_spi *ts_p, u8 threshold)
{
	return ts_p->fwctl_p->fwctl_func_p->reg_write_pixel_touch_threshold_f(ts_p->fwctl_p, threshold);
}

/* -------------------------------------------------------------------------- */
void shtps_fwctl_get_pen_jitter(struct shtps_rmi_spi *ts_p, u8 *val)
{
	ts_p->fwctl_p->fwctl_func_p->get_pen_jitter_f(ts_p->fwctl_p, val);
}

/* -------------------------------------------------------------------------- */
shtps_fwctl_get_palm_amplitude_threshold(struct shtps_rmi_spi *ts_p, u8 *val)
{
	ts_p->fwctl_p->fwctl_func_p->get_palm_amplitude_threshold_f(ts_p->fwctl_p, val);
}
/* -------------------------------------------------------------------------- */
shtps_fwctl_get_palm_area(struct shtps_rmi_spi *ts_p, u8 *val)
{
	ts_p->fwctl_p->fwctl_func_p->get_palm_area_f(ts_p->fwctl_p, val);
}
/* -------------------------------------------------------------------------- */
void shtps_fwctl_get_segmentation_aggressiveness(struct shtps_rmi_spi *ts_p, u8 *val)
{
	ts_p->fwctl_p->fwctl_func_p->get_segmentation_aggressiveness_f(ts_p->fwctl_p, val);
}

/* -------------------------------------------------------------------------- */
void shtps_fwctl_get_pixel_touch_threshold_def_val(struct shtps_rmi_spi *ts_p, u8 *val)
{
	ts_p->fwctl_p->fwctl_func_p->get_pixel_touch_threshold_def_val_f(ts_p->fwctl_p, val);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_palm_amplitude_threshold(struct shtps_rmi_spi *ts_p, u8 threshold)
{
	return ts_p->fwctl_p->fwctl_func_p->set_palm_amplitude_threshold_f(ts_p->fwctl_p, threshold);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_palm_area(struct shtps_rmi_spi *ts_p, u8 area)
{
	return ts_p->fwctl_p->fwctl_func_p->set_palm_area_f(ts_p->fwctl_p, area);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_palm_filter_mode_enable(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->set_palm_filter_mode_enable(ts_p->fwctl_p);
}
/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_palm_filter_mode_disable(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->set_palm_filter_mode_disable(ts_p->fwctl_p);
}
/* -------------------------------------------------------------------------- */
void shtps_fwctl_get_motion_suppression(struct shtps_rmi_spi *ts_p, u8 *val)
{
	ts_p->fwctl_p->fwctl_func_p->get_motion_suppression_f(ts_p->fwctl_p, val);
}
/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_doze(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->set_doze_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_active(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->set_active_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_sleepmode_on(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->set_sleepmode_on_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_sleepmode_off(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->set_sleepmode_off_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_lpwg_mode_on(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->set_lpwg_mode_on_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_lpwg_mode_off(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->set_lpwg_mode_off_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_lpwg_mode_cal(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->set_lpwg_mode_cal_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_motion_suppression(struct shtps_rmi_spi *ts_p, u8 x_supp, u8 y_supp)
{
	return ts_p->fwctl_p->fwctl_func_p->set_motion_suppression_f(ts_p->fwctl_p, x_supp, y_supp);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_jitter_filter_strength(struct shtps_rmi_spi *ts_p, u8 strength)
{
	return ts_p->fwctl_p->fwctl_func_p->set_jitter_filter_strength_f(ts_p->fwctl_p, strength);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_saturation_capacitance(struct shtps_rmi_spi *ts_p, u8 msb, u8 lsb)
{
	return ts_p->fwctl_p->fwctl_func_p->set_saturation_capacitance_f(ts_p->fwctl_p, msb, lsb);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_palmthresh(struct shtps_rmi_spi *ts_p, u8 threshold)
{
	return ts_p->fwctl_p->fwctl_func_p->set_palmthresh_f(ts_p->fwctl_p, threshold);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_set_low_reportrate_mode(struct shtps_rmi_spi *ts_p, int mode)
{
	return ts_p->fwctl_p->fwctl_func_p->set_low_reportrate_mode_f(ts_p->fwctl_p, mode);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_fingermax(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->get_fingermax_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_fingerinfo(struct shtps_rmi_spi *ts_p, u8 *buf, int read_cnt, u8 *irqsts, u8 *extsts, u8 **finger_p)
{
	return ts_p->fwctl_p->fwctl_func_p->get_fingerinfo_f(ts_p->fwctl_p, buf, read_cnt, irqsts, extsts, finger_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_one_fingerinfo(struct shtps_rmi_spi *ts_p, int id, u8 *buf, u8 **finger_p)
{
	return ts_p->fwctl_p->fwctl_func_p->get_one_fingerinfo_f(ts_p->fwctl_p, id, buf, finger_p);
}

/* -------------------------------------------------------------------------- */
u8* shtps_fwctl_get_finger_info_buf(struct shtps_rmi_spi *ts_p, int fingerid, int fingerMax, u8 *buf)
{
	return ts_p->fwctl_p->fwctl_func_p->get_finger_info_buf_f(ts_p->fwctl_p, fingerid, fingerMax, buf);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_finger_state(struct shtps_rmi_spi *ts_p, int fingerid, int fingerMax, u8 *buf)
{
	return ts_p->fwctl_p->fwctl_func_p->get_finger_state_f(ts_p->fwctl_p, fingerid, fingerMax, buf);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_finger_pos_x(struct shtps_rmi_spi *ts_p, u8 *buf)
{
	return ts_p->fwctl_p->fwctl_func_p->get_finger_pos_x_f(ts_p->fwctl_p, buf);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_finger_pos_y(struct shtps_rmi_spi *ts_p, u8 *buf)
{
	return ts_p->fwctl_p->fwctl_func_p->get_finger_pos_y_f(ts_p->fwctl_p, buf);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_finger_wx(struct shtps_rmi_spi *ts_p, u8 *buf)
{
	return ts_p->fwctl_p->fwctl_func_p->get_finger_wx_f(ts_p->fwctl_p, buf);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_finger_wy(struct shtps_rmi_spi *ts_p, u8 *buf)
{
	return ts_p->fwctl_p->fwctl_func_p->get_finger_wy_f(ts_p->fwctl_p, buf);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_finger_z(struct shtps_rmi_spi *ts_p, u8 *buf)
{
	return ts_p->fwctl_p->fwctl_func_p->get_finger_z_f(ts_p->fwctl_p, buf);
}

/* -------------------------------------------------------------------------- */
void shtps_fwctl_get_gesture(struct shtps_rmi_spi *ts_p, int fingerMax, u8 *buf, u8 *gs1, u8 *gs2)
{
	ts_p->fwctl_p->fwctl_func_p->get_gesture_f(ts_p->fwctl_p, fingerMax, buf, gs1, gs2);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_keystate(struct shtps_rmi_spi *ts_p, u8 *status)
{
	return ts_p->fwctl_p->fwctl_func_p->get_keystate_f(ts_p->fwctl_p, status);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_gesturetype(struct shtps_rmi_spi *ts_p, u8 *status)
{
	return ts_p->fwctl_p->fwctl_func_p->get_gesturetype_f(ts_p->fwctl_p, status);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_fwdate(struct shtps_rmi_spi *ts_p, u8 *year, u8 *month)
{
	return ts_p->fwctl_p->fwctl_func_p->get_fwdate_f(ts_p->fwctl_p, year, month);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_serial_number(struct shtps_rmi_spi *ts_p, u8 *buf)
{
	return ts_p->fwctl_p->fwctl_func_p->get_serial_number_f(ts_p->fwctl_p, buf);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_fwver(struct shtps_rmi_spi *ts_p, u16 *ver)
{
	return ts_p->fwctl_p->fwctl_func_p->get_fwver_f(ts_p->fwctl_p, ver);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_tm_mode(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->get_tm_mode_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_tm_rxsize(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->get_tm_rxsize_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_tm_txsize(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->get_tm_txsize_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_tm_frameline(struct shtps_rmi_spi *ts_p, u8 tm_mode, u8 *tm_data)
{
	return ts_p->fwctl_p->fwctl_func_p->get_tm_frameline_f(ts_p->fwctl_p, tm_mode, tm_data);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_tm_baseline(struct shtps_rmi_spi *ts_p, u8 tm_mode, u8 *tm_data)
{
	return ts_p->fwctl_p->fwctl_func_p->get_tm_baseline_f(ts_p->fwctl_p, tm_mode, tm_data);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_tm_baseline_raw(struct shtps_rmi_spi *ts_p, u8 tm_mode, u8 *tm_data)
{
	return ts_p->fwctl_p->fwctl_func_p->get_tm_baseline_raw_f(ts_p->fwctl_p, tm_mode, tm_data);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_cmd_tm_frameline(struct shtps_rmi_spi *ts_p, u8 tm_mode)
{
	return ts_p->fwctl_p->fwctl_func_p->cmd_tm_frameline_f(ts_p->fwctl_p, tm_mode);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_cmd_tm_baseline(struct shtps_rmi_spi *ts_p, u8 tm_mode)
{
	return ts_p->fwctl_p->fwctl_func_p->cmd_tm_baseline_f(ts_p->fwctl_p, tm_mode);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_cmd_tm_baseline_raw(struct shtps_rmi_spi *ts_p, u8 tm_mode)
{
	return ts_p->fwctl_p->fwctl_func_p->cmd_tm_baseline_raw_f(ts_p->fwctl_p, tm_mode);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_initparam(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->initparam_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_initparam_activemode(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->initparam_activemode_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_initparam_dozemode(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->initparam_dozemode_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_initparam_key(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->initparam_key_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_initparam_lpwgmode(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->initparam_lpwgmode_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_initparam_autorezero(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->initparam_autorezero_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_initparam_reportrate(struct shtps_rmi_spi *ts_p, int mode)
{
	return ts_p->fwctl_p->fwctl_func_p->initparam_reportrate_f(ts_p->fwctl_p, mode);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_hover_status(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->get_hover_status_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_hover_enable(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->hover_enable_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_hover_disable(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->hover_disable_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_autorezero_enable(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->autorezero_enable_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_autorezero_disable(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->autorezero_disable_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_pen_enable(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->pen_enable_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_pen_disable(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->pen_disable_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_start_testmode(struct shtps_rmi_spi *ts_p, u8 tm_mode)
{
	return ts_p->fwctl_p->fwctl_func_p->start_testmode_f(ts_p->fwctl_p, tm_mode);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_stop_testmode(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->stop_testmode_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_baseline_offset_disable(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->baseline_offset_disable_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
void shtps_fwctl_set_dev_state(struct shtps_rmi_spi *ts_p, u8 state)
{
	ts_p->fwctl_p->fwctl_func_p->set_dev_state_f(ts_p->fwctl_p, state);
}

/* -------------------------------------------------------------------------- */
u8 shtps_fwctl_get_dev_state(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->get_dev_state_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_maxXPosition(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->get_maxXPosition_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_maxYPosition(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->get_maxYPosition_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_AnalogCMD(struct shtps_rmi_spi *ts_p, u8 *buf)
{
	return ts_p->fwctl_p->fwctl_func_p->get_AnalogCMD_f(ts_p->fwctl_p, buf);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_get_ObjectAttention(struct shtps_rmi_spi *ts_p, u8 *buf)
{
	return ts_p->fwctl_p->fwctl_func_p->get_ObjectAttention_f(ts_p->fwctl_p, buf);
}

/* -------------------------------------------------------------------------- */
int shtps_fwctl_initparam_land_lock_distance(struct shtps_rmi_spi *ts_p)
{
	return ts_p->fwctl_p->fwctl_func_p->initparam_land_lock_distance_f(ts_p->fwctl_p);
}

/* -------------------------------------------------------------------------- */
#endif	/* SHTPS_BLD_FWCTL_FUNC_MACRO_ENABLE */

