/* drivers/sharp/shtps/atmel/filter/shtps_filter_pos_offset.c
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <sharp/shtps_dev.h>

#include "shtps_mxt.h"
#include "shtps_param_extern.h"
#include "shtps_log.h"

#include "shtps_filter.h"
/* -------------------------------------------------------------------------- */
#if defined( SHTPS_POSITION_OFFSET )
struct shtps_pos_offset{
	int							enabled;
	u16							base[5];
	signed short				diff[12];
};

struct shtps_pos_offset_info{
	struct shtps_pos_offset		offset;
};

static int shtps_filter_offset_area(struct shtps_pos_offset *offset_p, int x, int y)
{
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
static void shtps_filter_change_position(struct shtps_pos_offset *offset_p, int area, int *x, int *y)
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
		base_xq = CONFIG_SHTPS_PANEL_SIZE_X;
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
		base_xq = CONFIG_SHTPS_PANEL_SIZE_X;
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
		base_xq = CONFIG_SHTPS_PANEL_SIZE_X;
		base_yp = offset_p->base[3];
		base_yq = offset_p->base[4];
	}else if(area == 0x09){
		xq = xs = offset_p->diff[8];
		yp = yq = offset_p->diff[9];
		base_xp = 0;
		base_xq = offset_p->base[0];
		base_yp = offset_p->base[4];
		base_yq = CONFIG_SHTPS_PANEL_SIZE_Y;
	}else if(area == 0x0A){
		xp = xr = offset_p->diff[8];
		xq = xs = offset_p->diff[10];
		yp = offset_p->diff[9];
		yq = offset_p->diff[11];
		base_xp = offset_p->base[0];
		base_xq = offset_p->base[1];
		base_yp = offset_p->base[4];
		base_yq = CONFIG_SHTPS_PANEL_SIZE_Y;
	}else{
		xq = xr = offset_p->diff[10];
		yp = yq = offset_p->diff[11];
		base_xp = offset_p->base[1];
		base_xq = CONFIG_SHTPS_PANEL_SIZE_X;
		base_yp = offset_p->base[4];
		base_yq = CONFIG_SHTPS_PANEL_SIZE_Y;
	}

	pq = (xq - xp) * (*x - base_xp) / (base_xq - base_xp) + xp;
	rs = (xs - xr) * (*x - base_xp) / (base_xq - base_xp) + xr;
	*x -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);

	pq = (yq - yp) * (*x - base_xp) / (base_xq - base_xp) + yp;
	rs = (ys - yr) * (*x - base_xp) / (base_xq - base_xp) + yr;
	*y -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);

	if(*x >= CONFIG_SHTPS_LCD_SIZE_X){
		*x = CONFIG_SHTPS_LCD_SIZE_X - 1;
	}
	if(*x < 0){
		*x = 0;
	}
	if(*y >= CONFIG_SHTPS_LCD_SIZE_Y){
		*y = CONFIG_SHTPS_LCD_SIZE_Y - 1;
	}
	if(*y < 0){
		*y = 0;
	}
}
/* -------------------------------------------------------------------------- */
static int shtps_filter_offset_pos_single(struct shtps_mxt *ts, int *x, int *y)
{
	int area;

	if((*x == SHTPS_TOUCH_CANCEL_COORDINATES_X) && (*y == SHTPS_TOUCH_CANCEL_COORDINATES_Y) ){
		return 0;
	}

	area = shtps_filter_offset_area(&ts->pos_offset_p->offset, *x, *y);
	shtps_filter_change_position(&ts->pos_offset_p->offset, area, x, y);

	return 0;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_offset_pos(struct shtps_mxt *ts)
{
	int		i;
	int		fingerMax = shtps_get_fingermax(ts);
	struct	fingers *report_fingers = ts->input_event_info.report.cur;

	SHTPS_LOG_FUNC_CALL();

	if(ts->pos_offset_p == NULL){
		return;
	}

	if(!ts->pos_offset_p->offset.enabled){
		return;
	}

	for(i = 0; i < fingerMax; i++){
		if(report_fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			shtps_filter_offset_pos_single(ts, &report_fingers[i].x, &report_fingers[i].y);
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_set_offset_pos_param(struct shtps_mxt *ts, u8 *data)
{
	if(ts->pos_offset_p == NULL){
		return;
	}

	memcpy(ts->pos_offset_p->offset.base, data, sizeof(u16) * 5);
	ts->pos_offset_p->offset.diff[0] = (signed short)(data[11] << 0x08 | data[10]);
	ts->pos_offset_p->offset.diff[1] = (signed short)(data[13] << 0x08 | data[12]);
	ts->pos_offset_p->offset.diff[2] = (signed short)(data[15] << 0x08 | data[14]);
	ts->pos_offset_p->offset.diff[3] = (signed short)(data[17] << 0x08 | data[16]);
	ts->pos_offset_p->offset.diff[4] = (signed short)(data[19] << 0x08 | data[18]);
	ts->pos_offset_p->offset.diff[5] = (signed short)(data[21] << 0x08 | data[20]);
	ts->pos_offset_p->offset.diff[6] = (signed short)(data[23] << 0x08 | data[22]);
	ts->pos_offset_p->offset.diff[7] = (signed short)(data[25] << 0x08 | data[24]);
	ts->pos_offset_p->offset.diff[8] = (signed short)(data[27] << 0x08 | data[26]);
	ts->pos_offset_p->offset.diff[9] = (signed short)(data[29] << 0x08 | data[28]);
	ts->pos_offset_p->offset.diff[10]= (signed short)(data[31] << 0x08 | data[30]);
	ts->pos_offset_p->offset.diff[11]= (signed short)(data[33] << 0x08 | data[32]);

	if(ts->pos_offset_p->offset.base[0] == 0){
		ts->pos_offset_p->offset.enabled = 0;
	}else{
		ts->pos_offset_p->offset.enabled = 1;
	}

	SHTPS_LOG_DBG_PRINT("[%s] offset.enabled  = %d\n", __func__, ts->pos_offset_p->offset.enabled);
	SHTPS_LOG_DBG_PRINT("[%s] offset.diff[ 0] = %d\n", __func__, ts->pos_offset_p->offset.diff[ 0]);
	SHTPS_LOG_DBG_PRINT("[%s] offset.diff[ 1] = %d\n", __func__, ts->pos_offset_p->offset.diff[ 1]);
	SHTPS_LOG_DBG_PRINT("[%s] offset.diff[ 2] = %d\n", __func__, ts->pos_offset_p->offset.diff[ 2]);
	SHTPS_LOG_DBG_PRINT("[%s] offset.diff[ 3] = %d\n", __func__, ts->pos_offset_p->offset.diff[ 3]);
	SHTPS_LOG_DBG_PRINT("[%s] offset.diff[ 4] = %d\n", __func__, ts->pos_offset_p->offset.diff[ 4]);
	SHTPS_LOG_DBG_PRINT("[%s] offset.diff[ 5] = %d\n", __func__, ts->pos_offset_p->offset.diff[ 5]);
	SHTPS_LOG_DBG_PRINT("[%s] offset.diff[ 6] = %d\n", __func__, ts->pos_offset_p->offset.diff[ 6]);
	SHTPS_LOG_DBG_PRINT("[%s] offset.diff[ 7] = %d\n", __func__, ts->pos_offset_p->offset.diff[ 7]);
	SHTPS_LOG_DBG_PRINT("[%s] offset.diff[ 8] = %d\n", __func__, ts->pos_offset_p->offset.diff[ 8]);
	SHTPS_LOG_DBG_PRINT("[%s] offset.diff[ 9] = %d\n", __func__, ts->pos_offset_p->offset.diff[ 9]);
	SHTPS_LOG_DBG_PRINT("[%s] offset.diff[10] = %d\n", __func__, ts->pos_offset_p->offset.diff[10]);
	SHTPS_LOG_DBG_PRINT("[%s] offset.diff[11] = %d\n", __func__, ts->pos_offset_p->offset.diff[11]);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_offset_pos_init(struct shtps_mxt *ts)
{
	ts->pos_offset_p = kzalloc(sizeof(struct shtps_pos_offset_info), GFP_KERNEL);
	if(ts->pos_offset_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_offset_pos_deinit(struct shtps_mxt *ts)
{
	if(ts->pos_offset_p)	kfree(ts->pos_offset_p);
	ts->pos_offset_p = NULL;
}
/* -------------------------------------------------------------------------- */

#endif /* SHTPS_POSITION_OFFSET */

