/* drivers/sharp/shtps/sy3000/shtps_fwctl.h
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
#ifndef __SHTPS_FWCTL_H__
#define __SHTPS_FWCTL_H__
/* --------------------------------------------------------------------------- */
struct shtps_rmi_spi;
struct shtps_fwctl_functbl;
struct shtps_ctrl_functbl;
struct shtps_fwctl_functbl;
struct rmi_map;

/* --------------------------------------------------------------------------- */
#define SHTPS_BLD_FWCTL_FUNC_MACRO_ENABLE
/* --------------------------------------------------------------------------- */
struct shtps_fwctl_info{
	int								fwctl_ic_type;
	struct rmi_map					*map_p;
	void							*tps_ctrl_p;	/* same as 'struct spi_device* spi;' */
	struct shtps_ctrl_functbl		*devctrl_func_p;
	struct shtps_fwctl_functbl		*fwctl_func_p;
	//
	u8								dev_state;
};
/* --------------------------------------------------------------------------- */
#define DEVICE_ERR_CHECK(check, label)		if((check)) goto label
/* --------------------------------------------------------------------------- */

int shtps_fwctl_init(struct shtps_rmi_spi *, void *, struct shtps_ctrl_functbl *);
void shtps_fwctl_deinit(struct shtps_rmi_spi *);
/* --------------------------------------------------------------------------- */
typedef struct rmi_map* (*fwctl_if_ic_init_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_loader_init_writeconfig_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_loader_get_config_blocknum_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_loader_get_firm_blocknum_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_loader_get_blocksize_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_loader_get_result_writeconfig_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_loader_get_result_writeimage_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_loader_get_result_erase_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_loader_write_config_t)(struct shtps_fwctl_info *, u8 *, int);
typedef int (*fwctl_if_loader_write_image_t)(struct shtps_fwctl_info *, u8 *, int);
typedef int (*fwctl_if_loader_cmd_t)(struct shtps_fwctl_info *, u8 , u8);
typedef int (*fwctl_if_loader_cmd_erase_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_loader_cmd_writeimage_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_loader_cmd_writeconfig_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_loader_cmd_enterbl_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_loader_exit_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_get_device_status_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_soft_reset_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_irqclear_get_irqfactor_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_rezero_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_map_construct_t)(struct shtps_fwctl_info *, int);
typedef int (*fwctl_if_is_sleeping_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_is_singlefinger_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_reg_read_pen_jitter_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_reg_write_pen_jitter_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_reg_read_segmentation_aggressiveness_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_reg_write_segmentation_aggressiveness_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_reg_read_pixel_touch_threshold_t)(struct shtps_fwctl_info *, u8);
typedef int (*fwctl_if_reg_write_pixel_touch_threshold_t)(struct shtps_fwctl_info *, u8);
typedef void (*fwctl_if_get_pen_jitter_t)(struct shtps_fwctl_info *, u8 *);
typedef void (*fwctl_if_get_segmentation_aggressiveness_t)(struct shtps_fwctl_info *, u8 *);
typedef void (*fwctl_if_get_pixel_touch_threshold_def_val_t)(struct shtps_fwctl_info *, u8 *);
typedef void (*fwctl_if_get_motion_suppression_t)(struct shtps_fwctl_info *, u8 *);
typedef void (*fwctl_if_get_palm_amplitude_threshold_t)(struct shtps_fwctl_info *, u8 *);
typedef void (*fwctl_if_get_palm_area_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_set_palm_amplitude_threshold_t)(struct shtps_fwctl_info *, u8);
typedef int (*fwctl_if_set_palm_area_t)(struct shtps_fwctl_info *, u8);
typedef int (*fwctl_if_set_doze_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_set_palm_filter_mode_enable_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_set_palm_filter_mode_disable_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_set_active_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_set_sleepmode_on_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_set_sleepmode_off_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_set_lpwg_mode_on_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_set_lpwg_mode_off_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_set_lpwg_mode_cal_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_set_motion_suppression_t)(struct shtps_fwctl_info *, u8, u8);
typedef int (*fwctl_if_set_jitter_filter_strength_t)(struct shtps_fwctl_info *, u8);
typedef int (*fwctl_if_set_saturation_capacitance_t)(struct shtps_fwctl_info *, u8, u8);
typedef int (*fwctl_if_set_palmthresh_t)(struct shtps_fwctl_info *, u8);
typedef int (*fwctl_if_set_low_reportrate_mode_t)(struct shtps_fwctl_info *, int);
typedef int (*fwctl_if_get_fingermax_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_get_fingerinfo_t)(struct shtps_fwctl_info *, u8 *, int, u8 *, u8 *, u8 **);
typedef int (*fwctl_if_get_one_fingerinfo_t)(struct shtps_fwctl_info *, int, u8 *, u8 **);
typedef u8* (*fwctl_if_get_finger_info_buf_t)(struct shtps_fwctl_info *, int, int, u8 *);
typedef int (*fwctl_if_get_finger_state_t)(struct shtps_fwctl_info *, int, int, u8 *);
typedef int (*fwctl_if_get_finger_pos_x_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_get_finger_pos_y_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_get_finger_wx_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_get_finger_wy_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_get_finger_z_t)(struct shtps_fwctl_info *, u8 *);
typedef void (*fwctl_if_get_gesture_t)(struct shtps_fwctl_info *, int, u8 *, u8 *, u8 *);
typedef int (*fwctl_if_get_keystate_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_get_gesturetype_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_get_fwdate_t)(struct shtps_fwctl_info *, u8 *, u8 *);
typedef int (*fwctl_if_get_serial_number_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_get_fwver_t)(struct shtps_fwctl_info *, u16 *);
typedef int (*fwctl_if_get_tm_mode_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_get_tm_rxsize_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_get_tm_txsize_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_get_tm_frameline_t)(struct shtps_fwctl_info *, u8, u8 *);
typedef int (*fwctl_if_get_tm_baseline_t)(struct shtps_fwctl_info *, u8, u8 *);
typedef int (*fwctl_if_get_tm_baseline_raw_t)(struct shtps_fwctl_info *, u8, u8 *);
typedef int (*fwctl_if_cmd_tm_frameline_t)(struct shtps_fwctl_info *, u8);
typedef int (*fwctl_if_cmd_tm_baseline_t)(struct shtps_fwctl_info *, u8);
typedef int (*fwctl_if_cmd_tm_baseline_raw_t)(struct shtps_fwctl_info *, u8);
typedef int (*fwctl_if_initparam_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_initparam_activemode_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_initparam_dozemode_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_initparam_key_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_initparam_lpwgmode_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_initparam_autorezero_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_initparam_reportrate_t)(struct shtps_fwctl_info *, int);
typedef int (*fwctl_if_get_hover_status_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_hover_enable_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_hover_disable_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_autorezero_enable_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_autorezero_disable_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_pen_enable_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_pen_disable_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_start_testmode_t)(struct shtps_fwctl_info *, u8);
typedef int (*fwctl_if_stop_testmode_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_baseline_offset_disable_t)(struct shtps_fwctl_info *);
typedef void (*fwctl_if_set_dev_state_t)(struct shtps_fwctl_info *, u8);
typedef u8 (*fwctl_if_get_dev_state_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_get_maxXPosition_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_get_maxYPosition_t)(struct shtps_fwctl_info *);
typedef int (*fwctl_if_get_AnalogCMD_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_get_ObjectAttention_t)(struct shtps_fwctl_info *, u8 *);
typedef int (*fwctl_if_initparam_land_lock_distance_t)(struct shtps_fwctl_info *);
/* --------------------------------------------------------------------------- */
struct shtps_fwctl_functbl{
	fwctl_if_ic_init_t									ic_init_f;
	fwctl_if_loader_init_writeconfig_t					loader_init_writeconfig_f;
	fwctl_if_loader_get_config_blocknum_t				loader_get_config_blocknum_f;
	fwctl_if_loader_get_firm_blocknum_t					loader_get_firm_blocknum_f;
	fwctl_if_loader_get_blocksize_t						loader_get_blocksize_f;
	fwctl_if_loader_get_result_writeconfig_t			loader_get_result_writeconfig_f;
	fwctl_if_loader_get_result_writeimage_t				loader_get_result_writeimage_f;
	fwctl_if_loader_get_result_erase_t					loader_get_result_erase_f;
	fwctl_if_loader_write_config_t						loader_write_config_f;
	fwctl_if_loader_write_image_t						loader_write_image_f;
	fwctl_if_loader_cmd_t								loader_cmd_f;
	fwctl_if_loader_cmd_erase_t							loader_cmd_erase_f;
	fwctl_if_loader_cmd_writeimage_t					loader_cmd_writeimage_f;
	fwctl_if_loader_cmd_writeconfig_t					loader_cmd_writeconfig_f;
	fwctl_if_loader_cmd_enterbl_t						loader_cmd_enterbl_f;
	fwctl_if_loader_exit_t								loader_exit_f;
	fwctl_if_get_device_status_t						get_device_status_f;
	fwctl_if_soft_reset_t								soft_reset_f;
	fwctl_if_irqclear_get_irqfactor_t					irqclear_get_irqfactor_f;
	fwctl_if_rezero_t									rezero_f;
	fwctl_if_map_construct_t							map_construct_f;
	fwctl_if_is_sleeping_t								is_sleeping_f;
	fwctl_if_is_singlefinger_t							is_singlefinger_f;
	fwctl_if_reg_read_pen_jitter_t						reg_read_pen_jitter_f;
	fwctl_if_reg_write_pen_jitter_t						reg_write_pen_jitter_f;
	fwctl_if_reg_read_segmentation_aggressiveness_t		reg_read_segmentation_aggressiveness_f;
	fwctl_if_reg_write_segmentation_aggressiveness_t	reg_write_segmentation_aggressiveness_f;
	fwctl_if_reg_read_pixel_touch_threshold_t			reg_read_pixel_touch_threshold_f;
	fwctl_if_reg_write_pixel_touch_threshold_t			reg_write_pixel_touch_threshold_f;
	fwctl_if_get_pen_jitter_t							get_pen_jitter_f;
	fwctl_if_get_segmentation_aggressiveness_t			get_segmentation_aggressiveness_f;
	fwctl_if_get_pixel_touch_threshold_def_val_t		get_pixel_touch_threshold_def_val_f;
	fwctl_if_get_motion_suppression_t					get_motion_suppression_f;
	fwctl_if_get_palm_amplitude_threshold_t				get_palm_amplitude_threshold_f;
	fwctl_if_get_palm_area_t							get_palm_area_f;
	fwctl_if_set_palm_amplitude_threshold_t				set_palm_amplitude_threshold_f;
	fwctl_if_set_palm_area_t							set_palm_area_f;
	fwctl_if_set_palm_filter_mode_enable_t				set_palm_filter_mode_enable_f;
	fwctl_if_set_palm_filter_mode_disable_t				set_palm_filter_mode_disable_f;
	fwctl_if_set_doze_t									set_doze_f;
	fwctl_if_set_active_t								set_active_f;
	fwctl_if_set_sleepmode_on_t							set_sleepmode_on_f;
	fwctl_if_set_sleepmode_off_t						set_sleepmode_off_f;
	fwctl_if_set_lpwg_mode_on_t							set_lpwg_mode_on_f;
	fwctl_if_set_lpwg_mode_off_t						set_lpwg_mode_off_f;
	fwctl_if_set_lpwg_mode_cal_t						set_lpwg_mode_cal_f;
	fwctl_if_set_motion_suppression_t					set_motion_suppression_f;
	fwctl_if_set_jitter_filter_strength_t				set_jitter_filter_strength_f;
	fwctl_if_set_saturation_capacitance_t				set_saturation_capacitance_f;
	fwctl_if_set_palmthresh_t							set_palmthresh_f;
	fwctl_if_set_low_reportrate_mode_t					set_low_reportrate_mode_f;
	fwctl_if_get_fingermax_t							get_fingermax_f;
	fwctl_if_get_fingerinfo_t							get_fingerinfo_f;
	fwctl_if_get_one_fingerinfo_t						get_one_fingerinfo_f;
	fwctl_if_get_finger_info_buf_t						get_finger_info_buf_f;
	fwctl_if_get_finger_state_t							get_finger_state_f;
	fwctl_if_get_finger_pos_x_t							get_finger_pos_x_f;
	fwctl_if_get_finger_pos_y_t							get_finger_pos_y_f;
	fwctl_if_get_finger_wx_t							get_finger_wx_f;
	fwctl_if_get_finger_wy_t							get_finger_wy_f;
	fwctl_if_get_finger_z_t								get_finger_z_f;
	fwctl_if_get_gesture_t								get_gesture_f;
	fwctl_if_get_keystate_t								get_keystate_f;
	fwctl_if_get_gesturetype_t							get_gesturetype_f;
	fwctl_if_get_fwdate_t								get_fwdate_f;
	fwctl_if_get_serial_number_t						get_serial_number_f;
	fwctl_if_get_fwver_t								get_fwver_f;
	fwctl_if_get_tm_mode_t								get_tm_mode_f;
	fwctl_if_get_tm_rxsize_t							get_tm_rxsize_f;
	fwctl_if_get_tm_txsize_t							get_tm_txsize_f;
	fwctl_if_get_tm_frameline_t							get_tm_frameline_f;
	fwctl_if_get_tm_baseline_t							get_tm_baseline_f;
	fwctl_if_get_tm_baseline_raw_t						get_tm_baseline_raw_f;
	fwctl_if_cmd_tm_frameline_t							cmd_tm_frameline_f;
	fwctl_if_cmd_tm_baseline_t							cmd_tm_baseline_f;
	fwctl_if_cmd_tm_baseline_raw_t						cmd_tm_baseline_raw_f;
	fwctl_if_initparam_t								initparam_f;
	fwctl_if_initparam_activemode_t						initparam_activemode_f;
	fwctl_if_initparam_dozemode_t						initparam_dozemode_f;
	fwctl_if_initparam_key_t							initparam_key_f;
	fwctl_if_initparam_lpwgmode_t						initparam_lpwgmode_f;
	fwctl_if_initparam_autorezero_t						initparam_autorezero_f;
	fwctl_if_initparam_reportrate_t						initparam_reportrate_f;
	fwctl_if_get_hover_status_t							get_hover_status_f;
	fwctl_if_hover_enable_t								hover_enable_f;
	fwctl_if_hover_disable_t							hover_disable_f;
	fwctl_if_autorezero_enable_t						autorezero_enable_f;
	fwctl_if_autorezero_disable_t						autorezero_disable_f;
	fwctl_if_pen_enable_t								pen_enable_f;
	fwctl_if_pen_disable_t								pen_disable_f;
	fwctl_if_start_testmode_t							start_testmode_f;
	fwctl_if_stop_testmode_t							stop_testmode_f;
	fwctl_if_baseline_offset_disable_t					baseline_offset_disable_f;
	fwctl_if_set_dev_state_t							set_dev_state_f;
	fwctl_if_get_dev_state_t							get_dev_state_f;
	fwctl_if_get_maxXPosition_t							get_maxXPosition_f;
	fwctl_if_get_maxYPosition_t							get_maxYPosition_f;
	fwctl_if_get_AnalogCMD_t							get_AnalogCMD_f;
	fwctl_if_get_ObjectAttention_t						get_ObjectAttention_f;
	fwctl_if_initparam_land_lock_distance_t				initparam_land_lock_distance_f;
};
/* --------------------------------------------------------------------------- */
#ifdef	SHTPS_BLD_FWCTL_FUNC_MACRO_ENABLE
	#define shtps_fwctl_ic_init(ts)														ts->fwctl_p->fwctl_func_p->ic_init_f(ts->fwctl_p)
	#define shtps_fwctl_loader_init_writeconfig(ts)										ts->fwctl_p->fwctl_func_p->loader_init_writeconfig_f(ts->fwctl_p)
	#define shtps_fwctl_loader_get_config_blocknum(ts)									ts->fwctl_p->fwctl_func_p->loader_get_config_blocknum_f(ts->fwctl_p)
	#define shtps_fwctl_loader_get_firm_blocknum(ts)									ts->fwctl_p->fwctl_func_p->loader_get_firm_blocknum_f(ts->fwctl_p)
	#define shtps_fwctl_loader_get_blocksize(ts)										ts->fwctl_p->fwctl_func_p->loader_get_blocksize_f(ts->fwctl_p)
	#define shtps_fwctl_loader_get_result_writeconfig(ts)								ts->fwctl_p->fwctl_func_p->loader_get_result_writeconfig_f(ts->fwctl_p)
	#define shtps_fwctl_loader_get_result_writeimage(ts)								ts->fwctl_p->fwctl_func_p->loader_get_result_writeimage_f(ts->fwctl_p)
	#define shtps_fwctl_loader_get_result_erase(ts)										ts->fwctl_p->fwctl_func_p->loader_get_result_erase_f(ts->fwctl_p)
	#define shtps_fwctl_loader_write_config(ts, fwdata, blockSize)						ts->fwctl_p->fwctl_func_p->loader_write_config_f(ts->fwctl_p, fwdata, blockSize)
	#define shtps_fwctl_loader_write_image(ts, fwdata, blockSize)						ts->fwctl_p->fwctl_func_p->loader_write_image_f(ts->fwctl_p, fwdata, blockSize)
	#define shtps_fwctl_loader_cmd(ts, cmd, isLockdown)									ts->fwctl_p->fwctl_func_p->loader_cmd_f(ts->fwctl_p, cmd, isLockdown)
	#define shtps_fwctl_loader_cmd_erase(ts)											ts->fwctl_p->fwctl_func_p->loader_cmd_erase_f(ts->fwctl_p)
	#define shtps_fwctl_loader_cmd_writeimage(ts)										ts->fwctl_p->fwctl_func_p->loader_cmd_writeimage_f(ts->fwctl_p)
	#define shtps_fwctl_loader_cmd_writeconfig(ts)										ts->fwctl_p->fwctl_func_p->loader_cmd_writeconfig_f(ts->fwctl_p)
	#define shtps_fwctl_loader_cmd_enterbl(ts)											ts->fwctl_p->fwctl_func_p->loader_cmd_enterbl_f(ts->fwctl_p)
	#define shtps_fwctl_loader_exit(ts)													ts->fwctl_p->fwctl_func_p->loader_exit_f(ts->fwctl_p)
	#define shtps_fwctl_get_device_status(ts, status)									ts->fwctl_p->fwctl_func_p->get_device_status_f(ts->fwctl_p, status)
	#define shtps_fwctl_soft_reset(ts)													ts->fwctl_p->fwctl_func_p->soft_reset_f(ts->fwctl_p)
	#define shtps_fwctl_irqclear_get_irqfactor(ts, status)								ts->fwctl_p->fwctl_func_p->irqclear_get_irqfactor_f(ts->fwctl_p, status)
	#define shtps_fwctl_rezero(ts)														ts->fwctl_p->fwctl_func_p->rezero_f(ts->fwctl_p)
	#define shtps_fwctl_map_construct(ts, func_check)									ts->fwctl_p->fwctl_func_p->map_construct_f(ts->fwctl_p, func_check)
	#define shtps_fwctl_is_sleeping(ts)													ts->fwctl_p->fwctl_func_p->is_sleeping_f(ts->fwctl_p)
	#define shtps_fwctl_is_singlefinger(ts)												ts->fwctl_p->fwctl_func_p->is_singlefinger_f(ts->fwctl_p)
	#define shtps_fwctl_reg_read_pen_jitter(ts)											ts->fwctl_p->fwctl_func_p->reg_read_pen_jitter_f(ts->fwctl_p)
	#define shtps_fwctl_reg_write_pen_jitter(ts, val)									ts->fwctl_p->fwctl_func_p->reg_write_pen_jitter_f(ts->fwctl_p, val)
	#define shtps_fwctl_reg_read_segmentation_aggressiveness(ts)						ts->fwctl_p->fwctl_func_p->reg_read_segmentation_aggressiveness_f(ts->fwctl_p)
	#define shtps_fwctl_reg_write_segmentation_aggressiveness(ts, val)					ts->fwctl_p->fwctl_func_p->reg_write_segmentation_aggressiveness_f(ts->fwctl_p, val)
	#define shtps_fwctl_reg_read_pixel_touch_threshold(ts, threshold)					ts->fwctl_p->fwctl_func_p->reg_read_pixel_touch_threshold_f(ts->fwctl_p, threshold)
	#define shtps_fwctl_reg_write_pixel_touch_threshold(ts, threshold)					ts->fwctl_p->fwctl_func_p->reg_write_pixel_touch_threshold_f(ts->fwctl_p, threshold)
	#define shtps_fwctl_get_pen_jitter(ts, val)											ts->fwctl_p->fwctl_func_p->get_pen_jitter_f(ts->fwctl_p, val)
	#define shtps_fwctl_get_segmentation_aggressiveness(ts, val)						ts->fwctl_p->fwctl_func_p->get_segmentation_aggressiveness_f(ts->fwctl_p, val)
	#define shtps_fwctl_get_pixel_touch_threshold_def_val(ts, val)						ts->fwctl_p->fwctl_func_p->get_pixel_touch_threshold_def_val_f(ts->fwctl_p, val)
	#define shtps_fwctl_get_motion_suppression(ts, val)									ts->fwctl_p->fwctl_func_p->get_motion_suppression_f(ts->fwctl_p, val)
	#define shtps_fwctl_get_palm_amplitude_threshold(ts, val)							ts->fwctl_p->fwctl_func_p->get_palm_amplitude_threshold_f(ts->fwctl_p, val)
	#define shtps_fwctl_get_palm_area(ts, val)											ts->fwctl_p->fwctl_func_p->get_palm_area_f(ts->fwctl_p, val)
	#define shtps_fwctl_set_palm_amplitude_threshold(ts, threshold)				ts->fwctl_p->fwctl_func_p->set_palm_amplitude_threshold_f(ts->fwctl_p, threshold)
	#define shtps_fwctl_set_palm_area(ts, area)											ts->fwctl_p->fwctl_func_p->set_palm_area_f(ts->fwctl_p, area)
	#define shtps_fwctl_set_palm_filter_mode_enable(ts)							ts->fwctl_p->fwctl_func_p->set_palm_filter_mode_enable_f(ts->fwctl_p)
	#define shtps_fwctl_set_palm_filter_mode_disable(ts)							ts->fwctl_p->fwctl_func_p->set_palm_filter_mode_disable_f(ts->fwctl_p)
	#define shtps_fwctl_set_doze(ts)													ts->fwctl_p->fwctl_func_p->set_doze_f(ts->fwctl_p)
	#define shtps_fwctl_set_active(ts)													ts->fwctl_p->fwctl_func_p->set_active_f(ts->fwctl_p)
	#define shtps_fwctl_set_sleepmode_on(ts)											ts->fwctl_p->fwctl_func_p->set_sleepmode_on_f(ts->fwctl_p)
	#define shtps_fwctl_set_sleepmode_off(ts)											ts->fwctl_p->fwctl_func_p->set_sleepmode_off_f(ts->fwctl_p)
	#define shtps_fwctl_set_lpwg_mode_on(ts)											ts->fwctl_p->fwctl_func_p->set_lpwg_mode_on_f(ts->fwctl_p)
	#define shtps_fwctl_set_lpwg_mode_off(ts)											ts->fwctl_p->fwctl_func_p->set_lpwg_mode_off_f(ts->fwctl_p)
	#define shtps_fwctl_set_lpwg_mode_cal(ts)											ts->fwctl_p->fwctl_func_p->set_lpwg_mode_cal_f(ts->fwctl_p)
	#define shtps_fwctl_set_motion_suppression(ts, x_supp, y_supp)						ts->fwctl_p->fwctl_func_p->set_motion_suppression_f(ts->fwctl_p, x_supp, y_supp)
	#define shtps_fwctl_set_jitter_filter_strength(ts, strength)						ts->fwctl_p->fwctl_func_p->set_jitter_filter_strength_f(ts->fwctl_p, strength)
	#define shtps_fwctl_set_saturation_capacitance(ts, msb, lsb)						ts->fwctl_p->fwctl_func_p->set_saturation_capacitance_f(ts->fwctl_p, msb, lsb)
	#define shtps_fwctl_set_palmthresh(ts, threshold)									ts->fwctl_p->fwctl_func_p->set_palmthresh_f(ts->fwctl_p, threshold)
	#define shtps_fwctl_set_low_reportrate_mode(ts, mode)								ts->fwctl_p->fwctl_func_p->set_low_reportrate_mode_f(ts->fwctl_p, mode)
	#define shtps_fwctl_get_fingermax(ts)												ts->fwctl_p->fwctl_func_p->get_fingermax_f(ts->fwctl_p)
	#define shtps_fwctl_get_fingerinfo(ts, buf, read_cnt, irqsts, extsts, finger)		ts->fwctl_p->fwctl_func_p->get_fingerinfo_f(ts->fwctl_p, buf, read_cnt, irqsts, extsts, finger)
	#define shtps_fwctl_get_one_fingerinfo(ts, id, buf, finger)							ts->fwctl_p->fwctl_func_p->get_one_fingerinfo_f(ts->fwctl_p, id, buf, finger)
	#define shtps_fwctl_get_finger_info_buf(ts, fingerid, fingerMax, buf)				ts->fwctl_p->fwctl_func_p->get_finger_info_buf_f(ts->fwctl_p, fingerid, fingerMax, buf)
	#define shtps_fwctl_get_finger_state(ts, fingerid, fingerMax, buf)					ts->fwctl_p->fwctl_func_p->get_finger_state_f(ts->fwctl_p, fingerid, fingerMax, buf)
	#define shtps_fwctl_get_finger_pos_x(ts, buf)										ts->fwctl_p->fwctl_func_p->get_finger_pos_x_f(ts->fwctl_p, buf)
	#define shtps_fwctl_get_finger_pos_y(ts, buf)										ts->fwctl_p->fwctl_func_p->get_finger_pos_y_f(ts->fwctl_p, buf)
	#define shtps_fwctl_get_finger_wx(ts, buf)											ts->fwctl_p->fwctl_func_p->get_finger_wx_f(ts->fwctl_p, buf)
	#define shtps_fwctl_get_finger_wy(ts, buf)											ts->fwctl_p->fwctl_func_p->get_finger_wy_f(ts->fwctl_p, buf)
	#define shtps_fwctl_get_finger_z(ts, buf)											ts->fwctl_p->fwctl_func_p->get_finger_z_f(ts->fwctl_p, buf)
	#define shtps_fwctl_get_gesture(ts, fingerMax, buf, gs1, gs2)						ts->fwctl_p->fwctl_func_p->get_gesture_f(ts->fwctl_p, fingerMax, buf, gs1, gs2)
	#define shtps_fwctl_get_keystate(ts, status)										ts->fwctl_p->fwctl_func_p->get_keystate_f(ts->fwctl_p, status)
	#define shtps_fwctl_get_gesturetype(ts, status)										ts->fwctl_p->fwctl_func_p->get_gesturetype_f(ts->fwctl_p, status)
	#define shtps_fwctl_get_fwdate(ts, year, month)										ts->fwctl_p->fwctl_func_p->get_fwdate_f(ts->fwctl_p, year, month)
	#define shtps_fwctl_get_serial_number(ts, buf)										ts->fwctl_p->fwctl_func_p->get_serial_number_f(ts->fwctl_p, buf)
	#define shtps_fwctl_get_fwver(ts, ver)												ts->fwctl_p->fwctl_func_p->get_fwver_f(ts->fwctl_p, ver)
	#define shtps_fwctl_get_tm_mode(ts)													ts->fwctl_p->fwctl_func_p->get_tm_mode_f(ts->fwctl_p)
	#define shtps_fwctl_get_tm_rxsize(ts)												ts->fwctl_p->fwctl_func_p->get_tm_rxsize_f(ts->fwctl_p)
	#define shtps_fwctl_get_tm_txsize(ts)												ts->fwctl_p->fwctl_func_p->get_tm_txsize_f(ts->fwctl_p)
	#define shtps_fwctl_get_tm_frameline(ts, tm_mode, tm_data)							ts->fwctl_p->fwctl_func_p->get_tm_frameline_f(ts->fwctl_p, tm_mode, tm_data)
	#define shtps_fwctl_get_tm_baseline(ts, tm_mode, tm_data)							ts->fwctl_p->fwctl_func_p->get_tm_baseline_f(ts->fwctl_p, tm_mode, tm_data)
	#define shtps_fwctl_get_tm_baseline_raw(ts, tm_mode, tm_data)						ts->fwctl_p->fwctl_func_p->get_tm_baseline_raw_f(ts->fwctl_p, tm_mode, tm_data)
	#define shtps_fwctl_cmd_tm_frameline(ts, tm_mode)									ts->fwctl_p->fwctl_func_p->cmd_tm_frameline_f(ts->fwctl_p, tm_mode)
	#define shtps_fwctl_cmd_tm_baseline(ts, tm_mode)									ts->fwctl_p->fwctl_func_p->cmd_tm_baseline_f(ts->fwctl_p, tm_mode)
	#define shtps_fwctl_cmd_tm_baseline_raw(ts, tm_mode)								ts->fwctl_p->fwctl_func_p->cmd_tm_baseline_raw_f(ts->fwctl_p, tm_mode)
	#define shtps_fwctl_initparam(ts)													ts->fwctl_p->fwctl_func_p->initparam_f(ts->fwctl_p)
	#define shtps_fwctl_initparam_activemode(ts)										ts->fwctl_p->fwctl_func_p->initparam_activemode_f(ts->fwctl_p)
	#define shtps_fwctl_initparam_dozemode(ts)											ts->fwctl_p->fwctl_func_p->initparam_dozemode_f(ts->fwctl_p)
	#define shtps_fwctl_initparam_key(ts)												ts->fwctl_p->fwctl_func_p->initparam_key_f(ts->fwctl_p)
	#define shtps_fwctl_initparam_lpwgmode(ts)											ts->fwctl_p->fwctl_func_p->initparam_lpwgmode_f(ts->fwctl_p)
	#define shtps_fwctl_initparam_autorezero(ts)										ts->fwctl_p->fwctl_func_p->initparam_autorezero_f(ts->fwctl_p)
	#define shtps_fwctl_initparam_reportrate(ts, mode)									ts->fwctl_p->fwctl_func_p->initparam_reportrate_f(ts->fwctl_p, mode)
	#define shtps_fwctl_get_hover_status(ts)											ts->fwctl_p->fwctl_func_p->get_hover_status_f(ts->fwctl_p)
	#define shtps_fwctl_hover_enable(ts)												ts->fwctl_p->fwctl_func_p->hover_enable_f(ts->fwctl_p)
	#define shtps_fwctl_hover_disable(ts)												ts->fwctl_p->fwctl_func_p->hover_disable_f(ts->fwctl_p)
	#define shtps_fwctl_autorezero_enable(ts)											ts->fwctl_p->fwctl_func_p->autorezero_enable_f(ts->fwctl_p)
	#define shtps_fwctl_autorezero_disable(ts)											ts->fwctl_p->fwctl_func_p->autorezero_disable_f(ts->fwctl_p)
	#define shtps_fwctl_pen_enable(ts)													ts->fwctl_p->fwctl_func_p->pen_enable_f(ts->fwctl_p)
	#define shtps_fwctl_pen_disable(ts)													ts->fwctl_p->fwctl_func_p->pen_disable_f(ts->fwctl_p)
	#define shtps_fwctl_start_testmode(ts, tm_mode)										ts->fwctl_p->fwctl_func_p->start_testmode_f(ts->fwctl_p, tm_mode)
	#define shtps_fwctl_stop_testmode(ts)												ts->fwctl_p->fwctl_func_p->stop_testmode_f(ts->fwctl_p)
	#define shtps_fwctl_baseline_offset_disable(ts)										ts->fwctl_p->fwctl_func_p->baseline_offset_disable_f(ts->fwctl_p)
	#define shtps_fwctl_set_dev_state(ts, state)										ts->fwctl_p->fwctl_func_p->set_dev_state_f(ts->fwctl_p, state)
	#define shtps_fwctl_get_dev_state(ts)												ts->fwctl_p->fwctl_func_p->get_dev_state_f(ts->fwctl_p)
	#define shtps_fwctl_get_maxXPosition(ts)											ts->fwctl_p->fwctl_func_p->get_maxXPosition_f(ts->fwctl_p)
	#define shtps_fwctl_get_maxYPosition(ts)											ts->fwctl_p->fwctl_func_p->get_maxYPosition_f(ts->fwctl_p)
	#define shtps_fwctl_get_AnalogCMD(ts, buf)											ts->fwctl_p->fwctl_func_p->get_AnalogCMD_f(ts->fwctl_p, buf)
	#define shtps_fwctl_get_ObjectAttention(ts, buf)									ts->fwctl_p->fwctl_func_p->get_ObjectAttention_f(ts->fwctl_p, buf)
	#define shtps_fwctl_initparam_land_lock_distance(ts)								ts->fwctl_p->fwctl_func_p->initparam_land_lock_distance_f(ts->fwctl_p)
#else
	struct rmi_map* shtps_fwctl_ic_init(struct shtps_rmi_spi *);
	int shtps_fwctl_loader_init_writeconfig(struct shtps_rmi_spi *);
	int shtps_fwctl_loader_get_config_blocknum(struct shtps_rmi_spi *);
	int shtps_fwctl_loader_get_firm_blocknum(struct shtps_rmi_spi *);
	int shtps_fwctl_loader_get_blocksize(struct shtps_rmi_spi *);
	int shtps_fwctl_loader_get_result_writeconfig(struct shtps_rmi_spi *);
	int shtps_fwctl_loader_get_result_writeimage(struct shtps_rmi_spi *);
	int shtps_fwctl_loader_get_result_erase(struct shtps_rmi_spi *);
	int shtps_fwctl_loader_write_config(struct shtps_rmi_spi *, u8 *, int);
	int shtps_fwctl_loader_write_image(struct shtps_rmi_spi *, u8 *, int);
	int shtps_fwctl_loader_cmd(struct shtps_rmi_spi *, u8, u8);
	int shtps_fwctl_loader_cmd_erase(struct shtps_rmi_spi *);
	int shtps_fwctl_loader_cmd_writeimage(struct shtps_rmi_spi *);
	int shtps_fwctl_loader_cmd_writeconfig(struct shtps_rmi_spi *);
	int shtps_fwctl_loader_cmd_enterbl(struct shtps_rmi_spi *);
	int shtps_fwctl_loader_exit(struct shtps_rmi_spi *);
	int shtps_fwctl_get_device_status(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_soft_reset(struct shtps_rmi_spi *);
	int shtps_fwctl_irqclear_get_irqfactor(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_rezero(struct shtps_rmi_spi *);
	int shtps_fwctl_map_construct(struct shtps_rmi_spi *, int);
	int shtps_fwctl_is_sleeping(struct shtps_rmi_spi *);
	int shtps_fwctl_is_singlefinger(struct shtps_rmi_spi *);
	int shtps_fwctl_reg_read_pen_jitter(struct shtps_rmi_spi *);
	int shtps_fwctl_reg_write_pen_jitter(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_reg_read_segmentation_aggressiveness(struct shtps_rmi_spi *);
	int shtps_fwctl_reg_write_segmentation_aggressiveness(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_reg_read_pixel_touch_threshold(struct shtps_rmi_spi *, u8);
	int shtps_fwctl_reg_write_pixel_touch_threshold(struct shtps_rmi_spi *, u8);
	void shtps_fwctl_get_pen_jitter(struct shtps_rmi_spi *, u8 *);
	void shtps_fwctl_get_segmentation_aggressiveness(struct shtps_rmi_spi *, u8 *);
	void shtps_fwctl_get_pixel_touch_threshold_def_val(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_set_doze(struct shtps_rmi_spi *);
	int shtps_fwctl_set_active(struct shtps_rmi_spi *);
	int shtps_fwctl_set_sleepmode_on(struct shtps_rmi_spi *);
	int shtps_fwctl_set_sleepmode_off(struct shtps_rmi_spi *);
	int shtps_fwctl_set_lpwg_mode_on(struct shtps_rmi_spi *);
	int shtps_fwctl_set_lpwg_mode_off(struct shtps_rmi_spi *);
	int shtps_fwctl_set_lpwg_mode_cal(struct shtps_rmi_spi *);
	int shtps_fwctl_set_motion_suppression(struct shtps_rmi_spi *, u8, u8);
	int shtps_fwctl_set_jitter_filter_strength(struct shtps_rmi_spi *, u8);
	int shtps_fwctl_set_saturation_capacitance(struct shtps_rmi_spi *, u8, u8);
	int shtps_fwctl_set_palmthresh(struct shtps_rmi_spi *, u8);
	int shtps_fwctl_set_low_reportrate_mode(struct shtps_rmi_spi *, int);
	int shtps_fwctl_get_fingermax(struct shtps_rmi_spi *);
	int shtps_fwctl_get_fingerinfo(struct shtps_rmi_spi *, u8 *, int, u8 *, u8 *, u8 **);
	int shtps_fwctl_get_one_fingerinfo(struct shtps_rmi_spi *, int, u8 *, u8 **);
	u8* shtps_fwctl_get_finger_info_buf(struct shtps_rmi_spi *, int, int, u8 *);
	int shtps_fwctl_get_finger_state(struct shtps_rmi_spi *, int, int, u8 *);
	int shtps_fwctl_get_finger_pos_x(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_get_finger_pos_y(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_get_finger_wx(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_get_finger_wy(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_get_finger_z(struct shtps_rmi_spi *, u8 *);
	void shtps_fwctl_get_gesture(struct shtps_rmi_spi *, int, u8 *, u8 *, u8 *);
	int shtps_fwctl_get_keystate(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_get_gesturetype(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_get_fwdate(struct shtps_rmi_spi *, u8 *, u8 *);
	int shtps_fwctl_get_serial_number(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_get_fwver(struct shtps_rmi_spi *, u16 *);
	int shtps_fwctl_get_tm_mode(struct shtps_rmi_spi *);
	int shtps_fwctl_get_tm_rxsize(struct shtps_rmi_spi *);
	int shtps_fwctl_get_tm_txsize(struct shtps_rmi_spi *);
	int shtps_fwctl_get_tm_frameline(struct shtps_rmi_spi *, u8, u8 *);
	int shtps_fwctl_get_tm_baseline(struct shtps_rmi_spi *, u8, u8 *);
	int shtps_fwctl_get_tm_baseline_raw(struct shtps_rmi_spi *, u8, u8 *);
	int shtps_fwctl_cmd_tm_frameline(struct shtps_rmi_spi *, u8 );
	int shtps_fwctl_cmd_tm_baseline(struct shtps_rmi_spi *, u8 );
	int shtps_fwctl_cmd_tm_baseline_raw(struct shtps_rmi_spi *, u8);
	int shtps_fwctl_initparam(struct shtps_rmi_spi *);
	int shtps_fwctl_initparam_activemode(struct shtps_rmi_spi *);
	int shtps_fwctl_initparam_dozemode(struct shtps_rmi_spi *);
	int shtps_fwctl_initparam_key(struct shtps_rmi_spi *);
	int shtps_fwctl_initparam_lpwgmode(struct shtps_rmi_spi *);
	int shtps_fwctl_initparam_autorezero(struct shtps_rmi_spi *);
	int shtps_fwctl_initparam_reportrate(struct shtps_rmi_spi *, int);
	int shtps_fwctl_get_hover_status(struct shtps_rmi_spi *);
	int shtps_fwctl_hover_enable(struct shtps_rmi_spi *);
	int shtps_fwctl_hover_disable(struct shtps_rmi_spi *);
	int shtps_fwctl_autorezero_enable(struct shtps_rmi_spi *);
	int shtps_fwctl_autorezero_disable(struct shtps_rmi_spi *);
	int shtps_fwctl_pen_enable(struct shtps_rmi_spi *);
	int shtps_fwctl_pen_disable(struct shtps_rmi_spi *);
	int shtps_fwctl_start_testmode(struct shtps_rmi_spi *, u8);
	int shtps_fwctl_stop_testmode(struct shtps_rmi_spi *);
	int shtps_fwctl_baseline_offset_disable(struct shtps_rmi_spi *);

	void shtps_fwctl_set_dev_state(struct shtps_rmi_spi *, u8);
	u8 shtps_fwctl_get_dev_state(struct shtps_rmi_spi *);

	int shtps_fwctl_get_maxXPosition(struct shtps_rmi_spi *);
	int shtps_fwctl_get_maxYPosition(struct shtps_rmi_spi *);
	int shtps_fwctl_get_AnalogCMD(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_get_ObjectAttention(struct shtps_rmi_spi *, u8 *);
	int shtps_fwctl_initparam_land_lock_distance(struct shtps_rmi_spi *);
#endif	/* SHTPS_BLD_FWCTL_FUNC_MACRO_ENABLE */
/* --------------------------------------------------------------------------- */
#endif	/* __SHTPS_FWCTL_H__ */
