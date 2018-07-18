/* drivers/sharp/shtps/sy3000/shtps_filter.h
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
/* -------------------------------------------------------------------------- */
#ifndef __SHTPS_FILTER_H__
#define __SHTPS_FILTER_H__

/* -------------------------------------------------------------------------- */
struct shtps_rmi_spi;
struct shtps_touch_info;

/* -------------------------------------------------------------------------- */
typedef void (*shtps_filter_check_f)(struct shtps_rmi_spi *, struct shtps_touch_info *);
typedef void (*shtps_filter_init_f)(struct shtps_rmi_spi *);
typedef void (*shtps_filter_deinit_f)(struct shtps_rmi_spi *);

typedef void (*shtps_filter_sleep_f)(struct shtps_rmi_spi *);
typedef void (*shtps_filter_touchup_f)(struct shtps_rmi_spi *);

/* -------------------------------------------------------------------------- */
void shtps_filter_main(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
void shtps_filter_init(struct shtps_rmi_spi *ts);
void shtps_filter_deinit(struct shtps_rmi_spi *ts);
void shtps_filter_sleep_enter(struct shtps_rmi_spi *ts);
void shtps_filter_force_touchup(struct shtps_rmi_spi *ts);

/* -------------------------------------------------------------------------- */
#if defined ( CONFIG_SHTPS_SY3000_POSITION_OFFSET )
	int shtps_filter_offset_pos(struct shtps_rmi_spi *ts, int *x, int *y);
	int shtps_filter_offset_pos_pen(struct shtps_rmi_spi *ts, int *x, int *y);
#else
	#define shtps_filter_offset_pos(ts, x, y);
	#define shtps_filter_offset_pos_pen(ts, x, y);
#endif
/* -------------------------------------------------------------------------- */
#if defined(SHTPS_CLING_REJECTION_ENABLE)
	struct shtps_filter_cling_reject_info;

	void shtps_filter_cling_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_cling_reject_init(struct shtps_rmi_spi *ts);
	void shtps_filter_cling_reject_deinit(struct shtps_rmi_spi *ts);
	void shtps_filter_cling_reject_sleep(struct shtps_rmi_spi *ts);
#endif /* SHTPS_CLING_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_COAXIAL_GHOST_REJECTION_ENABLE )
	struct shtps_filter_coaxial_ghost_reject_info;

	void shtps_filter_coaxial_ghost_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_coaxial_ghost_event_report(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 event);
	void shtps_filter_coaxial_ghost_forcetouchup(struct shtps_rmi_spi *ts);
	void shtps_filter_coaxial_ghost_init(struct shtps_rmi_spi *ts);
	void shtps_filter_coaxial_ghost_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_COAXIAL_GHOST_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_WATER_CLING_REJECTION_ENABLE)
	struct shtps_filter_water_cling_reject_info;

	void shtps_filter_water_cling_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_water_cling_reject_forcetouchup(struct shtps_rmi_spi *ts);
	void shtps_filter_water_cling_reject_init(struct shtps_rmi_spi *ts);
	void shtps_filter_water_cling_reject_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_WATER_CLING_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_WATER_GHOST_REJECTION_ENABLE)
	struct shtps_filter_water_ghost_reject_info;

	void shtps_filter_water_ghost_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_water_ghost_reject_forcetouchup(struct shtps_rmi_spi *ts);
	void shtps_filter_water_ghost_reject_init(struct shtps_rmi_spi *ts);
	void shtps_filter_water_ghost_reject_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_WATER_GHOST_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_ELECTRICAL_GHOST_REJECTION_ENABLE)
	struct shtps_filter_electrical_ghost_reject_info;

	void shtps_filter_electrical_ghost_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_electrical_ghost_forcetouchup(struct shtps_rmi_spi *ts);
	void shtps_filter_electrical_ghost_reject_init(struct shtps_rmi_spi *ts);
	void shtps_filter_electrical_ghost_reject_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_ELECTRICAL_GHOST_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
	struct shtps_filter_topedge_fail_info;

	void shtps_filter_topedge_fail_touch_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_topedge_fail_touch_sleep(struct shtps_rmi_spi *ts);
	void shtps_filter_topedge_fail_touch_init(struct shtps_rmi_spi *ts);
	void shtps_filter_topedge_fail_touch_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_MULTI_HOVER_SELECT_ENABLE)
	struct shtps_filter_multi_hover_select_info;

	void shtps_filter_multi_hover_select(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_multi_hover_select_init(struct shtps_rmi_spi *ts);
	void shtps_filter_multi_hover_select_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_MULTI_HOVER_SELECT_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_DYNAMIC_RESET_CONTROL_ENABLE)
	struct shtps_filter_dynamic_reset_info;

	void shtps_filter_dynamic_reset_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_dynamic_reset_sleep(struct shtps_rmi_spi *ts);
	void shtps_filter_dynamic_reset_sleep_wakeup(struct shtps_rmi_spi *ts);
	void shtps_filter_dynamic_reset_init(struct shtps_rmi_spi *ts);
	void shtps_filter_dynamic_reset_deinit(struct shtps_rmi_spi *ts);
	void shtps_filter_dynamic_reset_sleep_process(struct shtps_rmi_spi *ts);
#endif /* SHTPS_DYNAMIC_RESET_CONTROL_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_DIAGONAL_GHOST_CHECK_ENABLE)
	struct shtps_filter_diagonal_ghost_reject_info;

	void shtps_filter_diagonal_ghost_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_diagonal_ghost_sleep(struct shtps_rmi_spi *ts);
	void shtps_filter_diagonal_ghost_init(struct shtps_rmi_spi *ts);
	void shtps_filter_diagonal_ghost_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_DIAGONAL_GHOST_CHECK_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE)
	struct multitouch_pen_ghost_rejection_info;

	void shtps_filter_multitouch_pen_ghost_rejection_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_multitouch_pen_ghost_rejection_sleep(struct shtps_rmi_spi *ts);
	void shtps_filter_multitouch_pen_ghost_rejection_init(struct shtps_rmi_spi *ts);
	void shtps_filter_multitouch_pen_ghost_rejection_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
	struct shtps_filter_edge_fail_touch_rej_info;

	int shtps_filter_edge_fail_touch_check_inhibit(struct shtps_rmi_spi *ts);
	void shtps_filter_edge_fail_touch_switch(struct shtps_rmi_spi *ts, int on);

	void shtps_filter_edge_fail_touch_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_edge_fail_touch_sleep(struct shtps_rmi_spi *ts);
	void shtps_filter_edge_fail_touch_init(struct shtps_rmi_spi *ts);
	void shtps_filter_edge_fail_touch_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
	struct shtps_filter_grip_fail_info;

	void shtps_filter_grip_fail_flick_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_grip_fail_touch_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_grip_fail_sleep(struct shtps_rmi_spi *ts);
	void shtps_filter_grip_fail_init(struct shtps_rmi_spi *ts);
	void shtps_filter_grip_fail_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_EDGE_POS_ADJUST_ENABLE)
	void shtps_filter_touch_position_adjust_edge(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
#endif /* SHTPS_EDGE_POS_ADJUST_ENABLE */
#if defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
	struct shtps_filter_shift_edge_inward_info;
	void shtps_filter_touch_position_adjust_shift_edge(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
#endif /* SHTPS_SHIFT_EDGE_INWARD_ENABLE */
#if defined(SHTPS_EDGE_POS_ADJUST_ENABLE) || defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
	void shtps_filter_touch_position_adjust_init(struct shtps_rmi_spi *ts);
	void shtps_filter_touch_position_adjust_deinit(struct shtps_rmi_spi *ts);
#endif	/* SHTPS_EDGE_POS_ADJUST_ENABLE || SHTPS_SHIFT_EDGE_INWARD_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE) || defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE) || defined(SHTPS_HOVER_REJECT_ENABLE)
	struct shtps_filter_correct_info;

	void shtps_filter_ignore_touch_info_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_correct_sleep(struct shtps_rmi_spi *ts);
	void shtps_filter_correct_init(struct shtps_rmi_spi *ts);
	void shtps_filter_correct_deinit(struct shtps_rmi_spi *ts);
	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE)
		void shtps_filter_invalid_area_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
		void shtps_filter_chattering_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
		void shtps_filter_set_report_touch_info(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
		void shtps_filter_set_touch_info_hover_detect_init(struct shtps_rmi_spi *ts);
		void shtps_filter_set_touch_info_hover_detect(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, int i);
		void shtps_filter_set_hover_detect_tu_timer(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE */
#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE || SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE || SHTPS_HOVER_REJECT_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE )
	void shtps_filter_pinchout_outset_distort_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_pinchout_outset_distort_init(struct shtps_rmi_spi *ts);
	void shtps_filter_pinchout_outset_distort_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_PEN_POS_JUMP_REJECT_ENABLE)
	struct shtps_filter_pen_pos_jump_info;

	void shtps_filter_pen_pos_jump_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_pen_pos_jump_sleep(struct shtps_rmi_spi *ts);
	void shtps_filter_pen_pos_jump_init(struct shtps_rmi_spi *ts);
	void shtps_filter_pen_pos_jump_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_PEN_POS_JUMP_REJECT_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_HOVER_HOST_JITTER_FILTER_ENABLE)
	struct shtps_filter_hover_jitter_filter_info;
	void shtps_filter_hover_jitter_filter(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_hover_jitter_filter_init(struct shtps_rmi_spi *ts);
	void shtps_filter_hover_jitter_filter_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_HOVER_HOST_JITTER_FILTER_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE)
	struct shtps_filter_multitap_fail_move_reject_info;

	void shtps_filter_multitap_fail_move_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_multitap_fail_move_forcetouchup(struct shtps_rmi_spi *ts);
	void shtps_filter_multitap_fail_move_init(struct shtps_rmi_spi *ts);
	void shtps_filter_multitap_fail_move_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE )
	struct shtps_lgm_split_touch_combining;

	void shtps_filter_lgm_split_touch_combining_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_lgm_split_touch_combining_check_forcetouchup(struct shtps_rmi_spi *ts);
	void shtps_filter_lgm_split_touch_combining_sleep(struct shtps_rmi_spi *ts);
	void shtps_filter_lgm_split_touch_combining_init(struct shtps_rmi_spi *ts);
	void shtps_filter_lgm_split_touch_combining_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
	struct shtps_filter_absorption_check_info;

	void shtps_filter_absorption_hold_cancel(struct shtps_rmi_spi *ts);
	void shtps_filter_absorption_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	int shtps_filter_absorption_event_report(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_absorption_forcetouchup(struct shtps_rmi_spi *ts);
	void shtps_filter_absorption_init(struct shtps_rmi_spi *ts);
	void shtps_filter_absorption_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_VARIABLE_PEN_JITTER_ENABLE )
	struct shtps_filter_pen_jitter_filter_info;

	void shtps_filter_get_register_pen_jitter(struct shtps_rmi_spi *ts);
	void shtps_filter_pen_jitter_filter_strength_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_pen_jitter_filter_strength_sleep(struct shtps_rmi_spi *ts);
	void shtps_filter_pen_jitter_filter_strength_init(struct shtps_rmi_spi *ts);
	void shtps_filter_pen_jitter_filter_strength_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_VARIABLE_PEN_JITTER_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
	struct shtps_filter_pinch_fail_reject_info;

	void shtps_filter_pinch_fail_response_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_get_register_segmentation_aggressiveness(struct shtps_rmi_spi *ts);
	void shtps_filter_pinch_fail_response_sleep(struct shtps_rmi_spi *ts);
	void shtps_filter_pinch_fail_response_init(struct shtps_rmi_spi *ts);
	void shtps_filter_pinch_fail_response_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_GHOST_REJECTION_ENABLE)
	struct shtps_filter_detect_ghost_info;

	int shtps_filter_detect_ghost(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_detect_ghost_forcetouchup(struct shtps_rmi_spi *ts);
	void shtps_filter_detect_ghost_init(struct shtps_rmi_spi *ts);
	void shtps_filter_detect_ghost_deinit(struct shtps_rmi_spi *ts);
#endif	/* SHTPS_GHOST_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
	struct shtps_filter_charger_armor_info;

	void shtps_filter_set_charger_armor_initparam(struct shtps_rmi_spi *ts);
	int shtps_filter_set_charger_armor(struct shtps_rmi_spi *ts, int state, int on);
	void shtps_filter_charger_armor_init(struct shtps_rmi_spi *ts);
	void shtps_filter_charger_armor_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_CHARGER_ARMOR_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_PEN_CLING_REJECTION_ENABLE )
	struct shtps_filter_pen_cling_reject_info;

	int  shtps_filter_pen_cling_reject_check2(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_pen_cling_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_pen_cling_reject_forcetouchup(struct shtps_rmi_spi *ts);
	void shtps_filter_pen_cling_reject_init(struct shtps_rmi_spi *ts);
	void shtps_filter_pen_cling_reject_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_PEN_CLING_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE )
	struct shtps_filter_topofscreen_ghost_reject_info;

	void shtps_filter_topofscreen_ghost_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
	void shtps_filter_topofscreen_ghost_reject_forcetouchup(struct shtps_rmi_spi *ts);
	void shtps_filter_topofscreen_ghost_reject_init(struct shtps_rmi_spi *ts);
	void shtps_filter_topofscreen_ghost_reject_deinit(struct shtps_rmi_spi *ts);
#endif /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_INVERTING_GHOST_REJECTION_ENABLE )
void shtps_filter_inverting_ghost_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info);
void shtps_filter_inverting_ghost_init(struct shtps_rmi_spi *ts);
void shtps_filter_inverting_ghost_deinit(struct shtps_rmi_spi *ts);
void shtps_filter_inverting_ghost_set_palm_thresh_default(struct shtps_rmi_spi *ts);
#endif /* SHTPS_INVERTING_GHOST_REJECTION_ENABLE */

/* -------------------------------------------------------------------------- */
struct shtps_filter_drag_hist_info;

void shtps_filter_drag(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 *event);
void shtps_filter_drag_init(struct shtps_rmi_spi *ts);
void shtps_filter_drag_deinit(struct shtps_rmi_spi *ts);

/* -------------------------------------------------------------------------- */
#endif	/* __SHTPS_FILTER_H__ */

