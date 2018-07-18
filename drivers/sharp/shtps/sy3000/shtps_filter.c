/* drivers/sharp/shtps/sy3000/shtps_filter.c
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
#include "shtps_rmi.h"
#include "shtps_param_extern.h"
#include "shtps_log.h"
#include "shtps_filter.h"

/* -------------------------------------------------------------------------- */
#if defined ( CONFIG_SHTPS_SY3000_POSITION_OFFSET )
static int shtps_filter_offset_area(struct shtps_offset_info *offset_p, int x, int y)
{
//	SHTPS_LOG_FUNC_CALL();
	if(y < offset_p->base[2]){
		if(x < offset_p->base[0]){
			return 0x00;
		}else if(x < offset_p->base[1]){
			return 0x01;
		}else{
			return 0x02;
		}
	}else if(y < offset_p->base[3]){
		if(x < offset_p->base[0]){
			return 0x03;
		}else if(x < offset_p->base[1]){
			return 0x04;
		}else{
			return 0x05;
		}
	}else if(y < offset_p->base[4]){
		if(x < offset_p->base[0]){
			return 0x06;
		}else if(x < offset_p->base[1]){
			return 0x07;
		}else{
			return 0x08;
		}
	}else{
		if(x < offset_p->base[0]){
			return 0x09;
		}else if(x < offset_p->base[1]){
			return 0x0A;
		}else{
			return 0x0B;
		}
	}
	return 0x00;
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_change_position(struct shtps_offset_info *offset_p, int area, int *x, int *y)
{
	int pq, rs;
	int xp, xq, xr, xs;
	int yp, yq, yr, ys;
	int base_xp, base_xq;
	int base_yp, base_yq;

	xp = xq = xr = xs = yp = yq = yr = ys = 0;
	if(area == 0x00){
		xq = xs = offset_p->diff[0];
		yr = ys = offset_p->diff[1];
		base_xp = 0;
		base_xq = offset_p->base[0];
		base_yp = 0;
		base_yq = offset_p->base[2];
	}else if(area == 0x01){
		xp = xr = offset_p->diff[0];
		xq = xs = offset_p->diff[2];
		yr = offset_p->diff[1];
		ys = offset_p->diff[3];
		base_xp = offset_p->base[0];
		base_xq = offset_p->base[1];
		base_yp = 0;
		base_yq = offset_p->base[2];
	}else if(area == 0x02){
		xq = xr = offset_p->diff[2];
		yr = ys = offset_p->diff[3];
		base_xp = offset_p->base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = 0;
		base_yq = offset_p->base[2];
	}else if(area == 0x03){
		xq = offset_p->diff[0];
		xs = offset_p->diff[4];
		yp = yq = offset_p->diff[1];
		yr = ys = offset_p->diff[5];
		base_xp = 0;
		base_xq = offset_p->base[0];
		base_yp = offset_p->base[2];
		base_yq = offset_p->base[3];
	}else if(area == 0x04){
		xp = offset_p->diff[0];
		xq = offset_p->diff[2];
		xr = offset_p->diff[4];
		xs = offset_p->diff[6];
		yp = offset_p->diff[1];
		yq = offset_p->diff[3];
		yr = offset_p->diff[5];
		ys = offset_p->diff[7];
		base_xp = offset_p->base[0];
		base_xq = offset_p->base[1];
		base_yp = offset_p->base[2];
		base_yq = offset_p->base[3];
	}else if(area == 0x05){
		xp = offset_p->diff[2];
		xr = offset_p->diff[6];
		yp = yq = offset_p->diff[3];
		yr = ys = offset_p->diff[7];
		base_xp = offset_p->base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = offset_p->base[2];
		base_yq = offset_p->base[3];
	}else if(area == 0x06){
		xq = offset_p->diff[4];
		xs = offset_p->diff[8];
		yp = yq = offset_p->diff[5];
		yr = ys = offset_p->diff[9];
		base_xp = 0;
		base_xq = offset_p->base[0];
		base_yp = offset_p->base[3];
		base_yq = offset_p->base[4];
	}else if(area == 0x07){
		xp = offset_p->diff[4];
		xq = offset_p->diff[6];
		xr = offset_p->diff[8];
		xs = offset_p->diff[10];
		yp = offset_p->diff[5];
		yq = offset_p->diff[7];
		yr = offset_p->diff[9];
		ys = offset_p->diff[11];
		base_xp = offset_p->base[0];
		base_xq = offset_p->base[1];
		base_yp = offset_p->base[3];
		base_yq = offset_p->base[4];
	}else if(area == 0x08){
		xp = offset_p->diff[6];
		xr = offset_p->diff[10];
		yp = yq = offset_p->diff[7];
		yr = ys = offset_p->diff[11];
		base_xp = offset_p->base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = offset_p->base[3];
		base_yq = offset_p->base[4];
	}else if(area == 0x09){
		xq = xs = offset_p->diff[8];
		yp = yq = offset_p->diff[9];
		base_xp = 0;
		base_xq = offset_p->base[0];
		base_yp = offset_p->base[4];
		base_yq = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y;
	}else if(area == 0x0A){
		xp = xr = offset_p->diff[8];
		xq = xs = offset_p->diff[10];
		yp = offset_p->diff[9];
		yq = offset_p->diff[11];
		base_xp = offset_p->base[0];
		base_xq = offset_p->base[1];
		base_yp = offset_p->base[4];
		base_yq = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y;
	}else{
		xq = xr = offset_p->diff[10];
		yp = yq = offset_p->diff[11];
		base_xp = offset_p->base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = offset_p->base[4];
		base_yq = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y;
	}

	pq = (xq - xp) * (*x - base_xp) / (base_xq - base_xp) + xp;
	rs = (xs - xr) * (*x - base_xp) / (base_xq - base_xp) + xr;
	*x -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);

	pq = (yq - yp) * (*x - base_xp) / (base_xq - base_xp) + yp;
	rs = (ys - yr) * (*x - base_xp) / (base_xq - base_xp) + yr;
	*y -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);

	if(*x >= CONFIG_SHTPS_SY3000_LCD_SIZE_X){
		*x = CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1;
	}
	if(*x < 0){
		*x = 0;
	}
	if(*y >= CONFIG_SHTPS_SY3000_LCD_SIZE_Y){
		*y = CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1;
	}
	if(*y < 0){
		*y = 0;
	}
}
/* -------------------------------------------------------------------------- */
int shtps_filter_offset_pos(struct shtps_rmi_spi *ts, int *x, int *y)
{
	int area;

//	SHTPS_LOG_FUNC_CALL();
	if(!ts->offset.enabled){
		return 0;
	}

	if((*x == SHTPS_TOUCH_CANCEL_COORDINATES_X) && (*y == SHTPS_TOUCH_CANCEL_COORDINATES_Y) ){
		return 0;
	}

	area = shtps_filter_offset_area(&ts->offset, *x, *y);
	shtps_filter_change_position(&ts->offset, area, x, y);

	return 0;
}

/* -------------------------------------------------------------------------- */
int shtps_filter_offset_pos_pen(struct shtps_rmi_spi *ts, int *x, int *y)
{
#if defined(SHTPS_PEN_DETECT_ENABLE)
	int area;
//	SHTPS_LOG_FUNC_CALL();
	if(!ts->offset_pen.enabled){
		return 0;
	}

	if((*x == SHTPS_TOUCH_CANCEL_COORDINATES_X) && (*y == SHTPS_TOUCH_CANCEL_COORDINATES_Y) ){
		return 0;
	}

	area = shtps_filter_offset_area(&ts->offset_pen, *x, *y);
	shtps_filter_change_position(&ts->offset_pen, area, x, y);

#endif /* SHTPS_PEN_DETECT_ENABLE */
	return 0;
}
#endif /* CONFIG_SHTPS_SY3000_POSITION_OFFSET */

/* -------------------------------------------------------------------------- */
void shtps_filter_main(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{

	#if defined( SHTPS_INVERTING_GHOST_REJECTION_ENABLE )
		shtps_filter_inverting_ghost_check(ts, info);
	#endif /* SHTPS_INVERTING_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_MULTI_HOVER_SELECT_ENABLE)
		shtps_filter_multi_hover_select(ts, info);
	#endif /* SHTPS_MULTI_HOVER_SELECT_ENABLE */

	#if defined(SHTPS_VARIABLE_PEN_JITTER_ENABLE)
		shtps_filter_pen_jitter_filter_strength_check(ts, info);
	#endif /* SHTPS_VARIABLE_PEN_JITTER_ENABLE */

	#if defined(SHTPS_CLING_REJECTION_ENABLE)
		shtps_filter_cling_reject_check(ts, info);
	#endif /* SHTPS_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_WATER_CLING_REJECTION_ENABLE)
		shtps_filter_water_cling_reject_check(ts, info);
	#endif /* SHTPS_WATER_CLING_REJECTION_ENABLE */

	#if defined( SHTPS_PEN_CLING_REJECTION_ENABLE )
		shtps_filter_pen_cling_reject_check(ts, info);
	#endif /* SHTPS_PEN_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_WATER_GHOST_REJECTION_ENABLE)
		shtps_filter_water_ghost_reject_check(ts, info);
	#endif /* SHTPS_WATER_GHOST_REJECTION_ENABLE */


	#if defined(SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
		shtps_filter_topofscreen_ghost_reject_check(ts, info);
	#endif /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */
	#if defined(SHTPS_DYNAMIC_RESET_CONTROL_ENABLE)
		shtps_filter_dynamic_reset_check(ts, info);
	#endif /* SHTPS_DYNAMIC_RESET_CONTROL_ENABLE */

	#if defined(SHTPS_DIAGONAL_GHOST_CHECK_ENABLE)
		shtps_filter_diagonal_ghost_check(ts, info);
	#endif /* SHTPS_DIAGONAL_GHOST_CHECK_ENABLE */

	#if defined(SHTPS_COAXIAL_GHOST_REJECTION_ENABLE)
		shtps_filter_coaxial_ghost_check(ts, info);
	#endif /* SHTPS_COAXIAL_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE)
		shtps_filter_multitouch_pen_ghost_rejection_check(ts, info);
	#endif /* SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_grip_fail_touch_check(ts, info);
	#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_topedge_fail_touch_check(ts, info);
	#endif /* SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_edge_fail_touch_check(ts, info);
	#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_grip_fail_flick_check(ts, info);
	#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_ELECTRICAL_GHOST_REJECTION_ENABLE)
		shtps_filter_electrical_ghost_reject_check(ts, info);
	#endif /* SHTPS_ELECTRICAL_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_EDGE_POS_ADJUST_ENABLE)
		shtps_filter_touch_position_adjust_edge(ts, info);	//	shtps_touch_position_adjust(ts, info);
	#endif	/* SHTPS_EDGE_POS_ADJUST_ENABLE */
	#if defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
		shtps_filter_touch_position_adjust_shift_edge(ts, info);	//	shtps_touch_position_adjust(ts, info);
	#endif	/* SHTPS_SHIFT_EDGE_INWARD_ENABLE */

	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE) || defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE) || defined(SHTPS_HOVER_REJECT_ENABLE)
		shtps_filter_ignore_touch_info_check(ts, info);
	#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE || SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE || SHTPS_HOVER_REJECT_ENABLE */

	#if defined(SHTPS_PEN_POS_JUMP_REJECT_ENABLE)
		shtps_filter_pen_pos_jump_check(ts, info);
	#endif /* SHTPS_PEN_POS_JUMP_REJECT_ENABLE */

	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE)
		shtps_filter_invalid_area_check(ts, info);
		shtps_filter_chattering_check(ts, info);
		shtps_filter_set_report_touch_info(ts, info);
	#endif	/*SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE  */

	#if defined(SHTPS_HOVER_HOST_JITTER_FILTER_ENABLE)
		shtps_filter_hover_jitter_filter(ts, info);
	#endif /* SHTPS_HOVER_HOST_JITTER_FILTER_ENABLE */

	#if defined(SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE)
		shtps_filter_absorption_check(ts, info);
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */

	#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
		shtps_filter_pinch_fail_response_check(ts, info);
	#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

	#if defined(SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE)
		shtps_filter_lgm_split_touch_combining_check(ts, info);
	#endif /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

	#if defined(SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE)
		shtps_filter_pinchout_outset_distort_check(ts, info);
	#endif /* SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE */

	#if defined(SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE)
		shtps_filter_multitap_fail_move_check(ts, info);
	#endif /* SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE */

}

/* -------------------------------------------------------------------------- */
void shtps_filter_init(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		shtps_filter_charger_armor_init(ts);
	#endif /* #if defined( SHTPS_CHARGER_ARMOR_ENABLE ) */

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		shtps_filter_absorption_init(ts);
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */

	#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_edge_fail_touch_init(ts);
	#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_grip_fail_init(ts);
	#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
		shtps_filter_pinch_fail_response_init(ts);
	#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

	#if defined(SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE)
		shtps_filter_lgm_split_touch_combining_init(ts);
	#endif /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE) || defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE) || defined(SHTPS_HOVER_REJECT_ENABLE)
		shtps_filter_correct_init(ts);
	#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE || SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE || SHTPS_HOVER_REJECT_ENABLE */

	#if defined(SHTPS_MULTI_HOVER_SELECT_ENABLE)
		shtps_filter_multi_hover_select_init(ts);
	#endif /* SHTPS_MULTI_HOVER_SELECT_ENABLE */

	#if defined(SHTPS_CLING_REJECTION_ENABLE)
		shtps_filter_cling_reject_init(ts);
	#endif /* SHTPS_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_HOVER_HOST_JITTER_FILTER_ENABLE)
		shtps_filter_hover_jitter_filter_init(ts);
	#endif /* SHTPS_HOVER_HOST_JITTER_FILTER_ENABLE */

	#if defined(SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_topedge_fail_touch_init(ts);
	#endif /* SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_DIAGONAL_GHOST_CHECK_ENABLE)
		shtps_filter_diagonal_ghost_init(ts);
	#endif /* SHTPS_DIAGONAL_GHOST_CHECK_ENABLE */

	#if defined(SHTPS_DYNAMIC_RESET_CONTROL_ENABLE)
		shtps_filter_dynamic_reset_init(ts);
	#endif /* SHTPS_DYNAMIC_RESET_CONTROL_ENABLE */

	#if defined(SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE)
		shtps_filter_multitouch_pen_ghost_rejection_init(ts);
	#endif /* SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_PEN_POS_JUMP_REJECT_ENABLE)
		shtps_filter_pen_pos_jump_init(ts);
	#endif /* SHTPS_PEN_POS_JUMP_REJECT_ENABLE */

	#if defined(SHTPS_GHOST_REJECTION_ENABLE)
		shtps_filter_detect_ghost_init(ts);
	#endif	/* SHTPS_GHOST_REJECTION_ENABLE */

	#if defined( SHTPS_COAXIAL_GHOST_REJECTION_ENABLE )
		shtps_filter_coaxial_ghost_init(ts);
	#endif /* SHTPS_COAXIAL_GHOST_REJECTION_ENABLE */

	#if defined( SHTPS_VARIABLE_PEN_JITTER_ENABLE )
		shtps_filter_pen_jitter_filter_strength_init(ts);
	#endif /* SHTPS_VARIABLE_PEN_JITTER_ENABLE */

	#if defined(SHTPS_WATER_GHOST_REJECTION_ENABLE)
		shtps_filter_water_ghost_reject_init(ts);
	#endif /* SHTPS_WATER_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
		shtps_filter_topofscreen_ghost_reject_init(ts);
	#endif /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_WATER_CLING_REJECTION_ENABLE)
		shtps_filter_water_cling_reject_init(ts);
	#endif /* SHTPS_WATER_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_ELECTRICAL_GHOST_REJECTION_ENABLE)
		shtps_filter_electrical_ghost_reject_init(ts);
	#endif /* SHTPS_ELECTRICAL_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE)
		shtps_filter_multitap_fail_move_init(ts);
	#endif /* SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE */

	#if defined( SHTPS_PEN_CLING_REJECTION_ENABLE )
		shtps_filter_pen_cling_reject_init(ts);
	#endif /* SHTPS_PEN_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_EDGE_POS_ADJUST_ENABLE) || defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
		shtps_filter_touch_position_adjust_init(ts);
	#endif	/* SHTPS_EDGE_POS_ADJUST_ENABLE || SHTPS_SHIFT_EDGE_INWARD_ENABLE */

	#if defined( SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE )
		shtps_filter_pinchout_outset_distort_init(ts);
	#endif /* SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE */

	#if defined( SHTPS_INVERTING_GHOST_REJECTION_ENABLE )
		shtps_filter_inverting_ghost_init(ts);
	#endif /* SHTPS_INVERTING_GHOST_REJECTION_ENABLE */


	//
}

/* -------------------------------------------------------------------------- */
void shtps_filter_deinit(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		shtps_filter_charger_armor_deinit(ts);
	#endif /* #if defined( SHTPS_CHARGER_ARMOR_ENABLE ) */

	#if defined(SHTPS_CLING_REJECTION_ENABLE)
		shtps_filter_cling_reject_deinit(ts);
	#endif /* SHTPS_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_WATER_CLING_REJECTION_ENABLE)
		shtps_filter_water_cling_reject_deinit(ts);
	#endif /* SHTPS_WATER_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_WATER_GHOST_REJECTION_ENABLE)
		shtps_filter_water_ghost_reject_deinit(ts);
	#endif /* SHTPS_WATER_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_ELECTRICAL_GHOST_REJECTION_ENABLE)
		shtps_filter_electrical_ghost_reject_deinit(ts);
	#endif /* SHTPS_ELECTRICAL_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
		shtps_filter_topofscreen_ghost_reject_deinit(ts);
	#endif /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

	#if defined( SHTPS_COAXIAL_GHOST_REJECTION_ENABLE )
		shtps_filter_coaxial_ghost_deinit(ts);
	#endif /* SHTPS_COAXIAL_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_topedge_fail_touch_deinit(ts);
	#endif /* SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_MULTI_HOVER_SELECT_ENABLE)
		shtps_filter_multi_hover_select_deinit(ts);
	#endif /* SHTPS_MULTI_HOVER_SELECT_ENABLE */

	#if defined(SHTPS_DYNAMIC_RESET_CONTROL_ENABLE)
		shtps_filter_dynamic_reset_deinit(ts);
	#endif /* SHTPS_DYNAMIC_RESET_CONTROL_ENABLE */

	#if defined(SHTPS_DIAGONAL_GHOST_CHECK_ENABLE)
		shtps_filter_diagonal_ghost_deinit(ts);
	#endif /* SHTPS_DIAGONAL_GHOST_CHECK_ENABLE */

	#if defined(SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE)
		shtps_filter_multitouch_pen_ghost_rejection_deinit(ts);
	#endif /* SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_grip_fail_deinit(ts);
	#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_edge_fail_touch_deinit(ts);
	#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE) || defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE) || defined(SHTPS_HOVER_REJECT_ENABLE)
		shtps_filter_correct_deinit(ts);
	#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE || SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE || SHTPS_HOVER_REJECT_ENABLE */

	#if defined(SHTPS_PEN_POS_JUMP_REJECT_ENABLE)
		shtps_filter_pen_pos_jump_deinit(ts);
	#endif /* SHTPS_PEN_POS_JUMP_REJECT_ENABLE */

	#if defined(SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE)
		shtps_filter_multitap_fail_move_deinit(ts);
	#endif /* SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE */

	#if defined(SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE)
		shtps_filter_lgm_split_touch_combining_deinit(ts);
	#endif /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		shtps_filter_absorption_deinit(ts);
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */

	#if defined(SHTPS_GHOST_REJECTION_ENABLE)
		shtps_filter_detect_ghost_deinit(ts);
	#endif	/* SHTPS_GHOST_REJECTION_ENABLE */

	#if defined( SHTPS_VARIABLE_PEN_JITTER_ENABLE )
		shtps_filter_pen_jitter_filter_strength_deinit(ts);
	#endif /* SHTPS_VARIABLE_PEN_JITTER_ENABLE */

	#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
		shtps_filter_pinch_fail_response_deinit(ts);
	#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

	#if defined(SHTPS_HOVER_HOST_JITTER_FILTER_ENABLE)
		shtps_filter_hover_jitter_filter_deinit(ts);
	#endif /* SHTPS_HOVER_HOST_JITTER_FILTER_ENABLE */

	#if defined( SHTPS_PEN_CLING_REJECTION_ENABLE )
		shtps_filter_pen_cling_reject_deinit(ts);
	#endif /* SHTPS_PEN_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_EDGE_POS_ADJUST_ENABLE) || defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
		shtps_filter_touch_position_adjust_deinit(ts);
	#endif	/* SHTPS_EDGE_POS_ADJUST_ENABLE || SHTPS_SHIFT_EDGE_INWARD_ENABLE */

	#if defined( SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE )
		shtps_filter_pinchout_outset_distort_deinit(ts);
	#endif /* SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE */
	
	#if defined( SHTPS_INVERTING_GHOST_REJECTION_ENABLE )
		shtps_filter_inverting_ghost_deinit(ts);
	#endif /* SHTPS_INVERTING_GHOST_REJECTION_ENABLE */

}
/* -------------------------------------------------------------------------- */
void shtps_filter_sleep_enter(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE) || defined(SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE) || defined(SHTPS_HOVER_REJECT_ENABLE)
		shtps_filter_correct_sleep(ts);
	#endif /* SHTPS_HOVER_DETECT_FAIL_RESOLV_ENABLE || SHTPS_PEN_EVENT_FAIL_CONTINUE_REJECT_ENABLE || SHTPS_HOVER_REJECT_ENABLE */

	#if defined(SHTPS_CLING_REJECTION_ENABLE)
		shtps_filter_cling_reject_sleep(ts);
	#endif /* SHTPS_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_edge_fail_touch_sleep(ts);
	#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_grip_fail_sleep(ts);
	#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		shtps_filter_topedge_fail_touch_sleep(ts);
	#endif /* SHTPS_TOP_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
		shtps_filter_pinch_fail_response_sleep(ts);
	#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

	#if defined( SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE )
		shtps_filter_lgm_split_touch_combining_sleep(ts);
	#endif  /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

	#if defined( SHTPS_DYNAMIC_RESET_CONTROL_ENABLE )
		shtps_filter_dynamic_reset_sleep(ts);
	#endif /* SHTPS_DYNAMIC_RESET_CONTROL_ENABLE */

	#if defined(SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE)
		shtps_filter_multitouch_pen_ghost_rejection_sleep(ts);
	#endif /* SHTPS_MULTITOUCH_PEN_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_PEN_POS_JUMP_REJECT_ENABLE)
		shtps_filter_pen_pos_jump_sleep(ts);
	#endif /* SHTPS_PEN_POS_JUMP_REJECT_ENABLE */

	#if defined( SHTPS_VARIABLE_PEN_JITTER_ENABLE )
		shtps_filter_pen_jitter_filter_strength_sleep(ts);
	#endif /* SHTPS_VARIABLE_PEN_JITTER_ENABLE */

	#if defined(SHTPS_DIAGONAL_GHOST_CHECK_ENABLE)
		shtps_filter_diagonal_ghost_sleep(ts);
	#endif /* SHTPS_DIAGONAL_GHOST_CHECK_ENABLE */
}

/* -------------------------------------------------------------------------- */
void shtps_filter_force_touchup(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		shtps_filter_absorption_forcetouchup(ts);
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE ) */

	#if defined( SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE )
		shtps_filter_lgm_split_touch_combining_check_forcetouchup(ts);
	#endif  /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

	#if defined( SHTPS_COAXIAL_GHOST_REJECTION_ENABLE )
		shtps_filter_coaxial_ghost_forcetouchup(ts);
	#endif /* SHTPS_COAXIAL_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
		shtps_filter_topofscreen_ghost_reject_forcetouchup(ts);
	#endif /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_WATER_GHOST_REJECTION_ENABLE)
		shtps_filter_water_ghost_reject_forcetouchup(ts);
	#endif /* SHTPS_WATER_GHOST_REJECTION_ENABLE */

	#if defined(SHTPS_WATER_CLING_REJECTION_ENABLE)
		shtps_filter_water_cling_reject_forcetouchup(ts);
	#endif /* SHTPS_WATER_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE)
		shtps_filter_multitap_fail_move_forcetouchup(ts);
	#endif /* SHTPS_MULTI_TAP_FAIL_MOVE_REJECTION_ENABLE */

	#if defined(SHTPS_GHOST_REJECTION_ENABLE)
		shtps_filter_detect_ghost_forcetouchup(ts);
	#endif	/* SHTPS_GHOST_REJECTION_ENABLE */

	#if defined( SHTPS_PEN_CLING_REJECTION_ENABLE )
		shtps_filter_pen_cling_reject_forcetouchup(ts);
	#endif /* SHTPS_PEN_CLING_REJECTION_ENABLE */
}
/* -------------------------------------------------------------------------- */

