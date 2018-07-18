/* drivers/video/msm/mdss/mdss_diag.c  (Display Driver)
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
#include <mdss_shdisp.h>
#include <mdss_diag.h>
#include <linux/types.h>
#include <mach/board.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/shdisp_dsi.h>
#include "mdss_fb.h"
#include <mdss_dsi.h>
#include <mdss_mdp.h>
#include <linux/iopoll.h>

#define MDSS_DIAG_MIPI_CHECK_AMP_OFF   0x0580
#define MDSS_DIAG_WAIT_1FRAME_US       (16666)

struct mdss_panel_data     * mdss_diag_panel_data;
struct mdss_dsi_ctrl_pdata * mdss_diag_dsi_ctrl;

static uint8_t mdss_diag_mipi_check_amp_data;
static uint8_t mdss_diag_mipi_check_rec_sens_data;
static int mdss_diag_mipi_check_exec_state = false;

static int mdss_diag_mipi_check_exec(uint8_t flame_cnt, uint8_t amp, uint8_t sensitiv);
static int mdss_diag_mipi_check_manual(struct mdp_mipi_check_param * mipi_check_param);
static int mdss_diag_mipi_check_auto(struct mdp_mipi_check_param * mipi_check_param);
static void mdss_diag_mipi_check_set_param(uint8_t amp, uint8_t sensitiv);
static void mdss_diag_mipi_check_get_param(uint8_t *amp, uint8_t *sensitiv);
static int mdss_diag_mipi_check_test(uint8_t flame_cnt);
static int mdss_diag_mipi_check_test_video(uint8_t flame_cnt);
static int mdss_diag_mipi_check_test_cmd(uint8_t flame_cnt);
static int mdss_diag_read_sensitiv(uint8_t *read_data);
static int mdss_diag_dsi_cmd_bta_sw_trigger(struct mdss_panel_data *pdata);
static int mdss_diag_dsi_ack_err_status(struct mdss_dsi_ctrl_pdata *ctrl);
static void mdss_diag_mipi_check_result_convert(
			uint8_t befoe[MDSS_MIPICHK_SENSITIV_NUM][MDSS_MIPICHK_AMP_NUM],
			struct mdp_mipi_check_param * mipi_check_param);
static inline struct mdss_mdp_ctl* mdss_diag_get_mdpctrl(int fbinx);
static int mdss_diag_mipi_clkchg_setparam(struct mdp_mipi_clkchg_param * mipi_clkchg_param, struct mdss_mdp_ctl *pctl);
static void mdss_diag_mipi_clkchg_host_data(struct mdp_mipi_clkchg_param * mipi_clkchg_param, struct mdss_panel_data *pdata);
static int mdss_diag_mipi_clkchg_host(struct mdp_mipi_clkchg_param * mipi_clkchg_param, struct mdss_mdp_ctl *pctl);
static int mdss_diag_mipi_clkchg_panel_clk_update(struct mdss_panel_data *pdata);
static int mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata);
static int mdss_diag_mipi_clkchg_panel_porch_update(struct mdss_panel_data *pdata);
static int mdss_diag_mipi_clkchg_panel(struct mdp_mipi_clkchg_param * mipi_clkchg_param, struct mdss_mdp_ctl *pctl);

#if defined(CONFIG_SHDISP_PANEL_ANDY)
extern int shdisp_api_set_freq_param(struct mdp_mipi_clkchg_panel_andy freq);
#endif  /* CONFIG_SHDISP_PANEL_ANDY */
extern void mdss_dsi_err_intr_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, u32 mask,int enable);
extern void mdss_shdisp_video_transfer_ctrl(int onoff);
extern int mdss_shdisp_host_dsi_tx(int commit,
		struct shdisp_dsi_cmd_desc * shdisp_cmds, int size);
extern int mdss_shdisp_host_dsi_rx(struct shdisp_dsi_cmd_desc * cmds,
		unsigned char * rx_data, int rx_size);
extern void mdss_dsi_pll_relock(struct mdss_dsi_ctrl_pdata *ctrl);
#ifndef SHDISP_DISABLE_HR_VIDEO
extern int mdss_mdp_hr_video_suspend(struct mdss_mdp_ctl *ctl, int tg_en_flg);
extern int mdss_mdp_hr_video_resume(struct mdss_mdp_ctl *ctl, int tg_en_flg);
extern int mdss_mdp_hr_video_clkchg_mdp_update(struct mdss_mdp_ctl *ctl);
#endif /* SHDISP_DISABLE_HR_VIDEO */

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_check_get_exec_state(void)
{
	return mdss_diag_mipi_check_exec_state;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
extern struct fb_info* mdss_fb_get_fbinfo(int);
static inline struct mdss_mdp_ctl* mdss_diag_get_mdpctrl(int fbinx)
{
	return ((struct mdss_overlay_private*)((struct msm_fb_data_type*)mdss_fb_get_fbinfo(fbinx)->par)->mdp.private1)->ctl;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_check(struct mdp_mipi_check_param * mipi_check_param,
			 struct mdss_panel_data *pdata)
{
	int ret;
	u32 isr;

	mdss_diag_panel_data = pdata;
	mdss_diag_dsi_ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: called\n", __func__);

	if (!mdss_diag_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_diag_dsi_ctrl=0x%p", __func__, mdss_diag_dsi_ctrl);
		return -ENXIO;
	}

#ifndef SHDISP_DET_DSI_MIPI_ERROR
	/* disable dsi error interrupt */
	mdss_dsi_err_intr_ctrl(mdss_diag_dsi_ctrl, DSI_INTR_ERROR_MASK, 0);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */
	mdss_diag_mipi_check_exec_state = true;

	mdss_diag_dsi_cmd_bta_sw_trigger(mdss_diag_panel_data);

	mdss_diag_mipi_check_get_param(&mdss_diag_mipi_check_amp_data, &mdss_diag_mipi_check_rec_sens_data);

	if (mdss_diag_dsi_ctrl->panel_mode == DSI_VIDEO_MODE) {
		mdss_shdisp_video_transfer_ctrl(false);
	}

	if (mipi_check_param->mode == MDSS_MIPICHK_MANUAL) {
		ret = mdss_diag_mipi_check_manual(mipi_check_param);
	} else if (mipi_check_param->mode == MDSS_MIPICHK_AUTO) {
		ret = mdss_diag_mipi_check_auto(mipi_check_param);
	} else {
		pr_err("%s:mode=%d\n", __func__, mipi_check_param->mode);
		return -EINVAL;
	}

	mdss_diag_mipi_check_set_param(mdss_diag_mipi_check_amp_data, mdss_diag_mipi_check_rec_sens_data);

	if (mdss_diag_dsi_ctrl->panel_mode == DSI_VIDEO_MODE) {
		mdss_shdisp_video_transfer_ctrl(true);
	}

	mdss_diag_mipi_check_exec_state = false;

	isr = MIPI_INP(mdss_diag_dsi_ctrl->ctrl_base + 0x0110);/* DSI_INTR_CTRL */
	MIPI_OUTP(mdss_diag_dsi_ctrl->ctrl_base + 0x0110, isr);

#ifndef SHDISP_DET_DSI_MIPI_ERROR
	/* enable dsi error interrupt */
	mdss_dsi_err_intr_ctrl(mdss_diag_dsi_ctrl, DSI_INTR_ERROR_MASK, 1);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */

	pr_debug("%s: end", __func__);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_manual(struct mdp_mipi_check_param * mipi_check_param)
{
	int ret = 0;

	pr_debug("%s: called\n", __func__);

	if ((mipi_check_param->amp & ~0x07) != 0) {
		pr_err("LCDERR: %s AMP=0x%X Out of range", __func__, mipi_check_param->amp);
		return -ENXIO;
	}

	if ((mipi_check_param->sensitiv & ~0x0F) != 0) {
		pr_err("LCDERR: %s SENSITIV=0x%X Out of range", __func__, mipi_check_param->amp);
		return -ENXIO;
	}

	ret = mdss_diag_mipi_check_exec(mipi_check_param->flame_cnt, mipi_check_param->amp, mipi_check_param->sensitiv);

	mipi_check_param->result[0] = ret;
	
	pr_debug("%s: end", __func__);

	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_auto(struct mdp_mipi_check_param * mipi_check_param)
{
	int ret = 0;
	int ret2 = 0;
	uint8_t i,j;
    uint8_t result_temp[MDSS_MIPICHK_SENSITIV_NUM][MDSS_MIPICHK_AMP_NUM]={{0}};
	uint8_t set_flame       = 0x01;
	uint8_t max_amp         = 0x07;
	uint8_t max_sensitiv    = 0x0F;

	pr_debug("%s: called\n", __func__);
	

	for (i = 0; i < MDSS_MIPICHK_SENSITIV_NUM; i++) {
		for (j = 0; j < MDSS_MIPICHK_AMP_NUM; j++) {
			ret = mdss_diag_mipi_check_exec(mipi_check_param->flame_cnt, j, i);
			if (ret == MDSS_MIPICHK_RESULT_NG) {
				ret2 = mdss_diag_mipi_check_exec(set_flame, max_amp, max_sensitiv);
				if (ret2 == MDSS_MIPICHK_RESULT_NG) {
					pr_err("LCDERR: %s mdss_diag_mipi_check_exec ret=%d", __func__, ret);
				}
			}
			result_temp[i][j] = ret;
		}
	}

	mdss_diag_mipi_check_result_convert(result_temp, mipi_check_param);

	pr_debug("%s: end", __func__);

	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_diag_mipi_check_result_convert(
			uint8_t befoe_result[MDSS_MIPICHK_SENSITIV_NUM][MDSS_MIPICHK_AMP_NUM],
			struct mdp_mipi_check_param * mipi_check_param)
{
	uint8_t i,j,x,y;
	uint8_t after_result[MDSS_MIPICHK_SENSITIV_NUM] = {0};

	for (j = 0; j < MDSS_MIPICHK_AMP_NUM; j++) {
		for (i = 0; i < MDSS_MIPICHK_SENSITIV_NUM; i++) {
			if(befoe_result[i][j] == MDSS_MIPICHK_RESULT_OK) {
				x = j * 2;
				if (i >= 8) {
					y = i - 8;
					x++;
				} else {
					y = i;
				}
				after_result[x] |= (1 << (7-y));
			}
		}
	}
	memcpy(mipi_check_param->result, after_result, sizeof(after_result)); 
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_exec(uint8_t flame_cnt, uint8_t amp, uint8_t sensitiv)
{
	int ret = 0;
	u32 amp_reg;
	u32 amp_tmp;
    uint8_t set_sensitiv = 0;

	uint8_t amp_tbl[MDSS_MIPICHK_AMP_NUM] = {
	    0x03,
	    0x02,
	    0x00,
	    0x01,
	    0x04,
	    0x05,
	    0x06,
	    0x07
	};
	
	pr_debug("%s: called flame_cnt=0x%02X amp=0x%02X sensitiv=0x%02X\n", __func__, flame_cnt, amp, sensitiv);

	amp_tmp = amp_tbl[amp];
	amp_reg = (amp_tmp << 1) | 1;

#if defined(CONFIG_SHDISP_PANEL_ANDY)
		set_sensitiv = (sensitiv << 4);
#elif defined(CONFIG_SHDISP_PANEL_ARIA)
		set_sensitiv = (sensitiv << 3);
#endif  /* CONFIG_SHDISP_PANEL_ANDY || CONFIG_SHDISP_PANEL_ARIA */

	mdss_diag_mipi_check_set_param(amp_reg, set_sensitiv);

	ret = mdss_diag_mipi_check_test(flame_cnt);

	pr_debug("%s: end ret=%d \n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_check_set_param(uint8_t amp, uint8_t sensitiv)
{
	char payload_sensitiv[2][2] = {
	    {0xFF, 0x00},
	    {0x45, 0x00},
	};

	struct shdisp_dsi_cmd_desc cmds_sensitiv[] = {
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_sensitiv[0]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_sensitiv[1]},
	};

	MIPI_OUTP((mdss_diag_dsi_ctrl->ctrl_base) + MDSS_DIAG_MIPI_CHECK_AMP_OFF, amp);
	wmb();

#if defined(CONFIG_SHDISP_PANEL_ANDY)
	payload_sensitiv[0][1] = 0xEE;
	sensitiv |= (mdss_diag_mipi_check_rec_sens_data & 0x0F);
#elif defined(CONFIG_SHDISP_PANEL_ARIA)
	payload_sensitiv[0][1] = 0xE0;
	sensitiv |= (mdss_diag_mipi_check_rec_sens_data & 0x87);
#endif  /* CONFIG_SHDISP_PANEL_ANDY || CONFIG_SHDISP_PANEL_ARIA */
	payload_sensitiv[1][1] = sensitiv;

	mdss_shdisp_host_dsi_tx(1, cmds_sensitiv, ARRAY_SIZE(cmds_sensitiv));
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_check_get_param(uint8_t *amp, uint8_t *sensitiv)
{
	int ret = 0;
	*amp = MIPI_INP((mdss_diag_dsi_ctrl->ctrl_base) + MDSS_DIAG_MIPI_CHECK_AMP_OFF);

	ret = mdss_diag_read_sensitiv(sensitiv);

	pr_debug("%s: amp=0x%02X sensitiv=0x%02X\n", __func__, *amp, *sensitiv);

}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test(uint8_t flame_cnt)
{
	int ret = 0;
	char mode;
	
	mode = mdss_diag_dsi_ctrl->panel_mode;
	
	if (mode == DSI_VIDEO_MODE) {
		ret = mdss_diag_mipi_check_test_video(flame_cnt); 
	} else if (mode == DSI_CMD_MODE) {
		ret = mdss_diag_mipi_check_test_cmd(flame_cnt);
	} else {
		pr_err("LCDERR: %s paneltype=%d\n", __func__, mode);
		ret = -EINVAL;
	}
	
	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test_video(uint8_t flame_cnt)
{
	int ret = 0;
	uint32_t sleep;

	sleep = flame_cnt * MDSS_DIAG_WAIT_1FRAME_US;
	pr_debug("%s: flame=%d sleep time=%d\n", __func__, flame_cnt, sleep);

	mdss_shdisp_video_transfer_ctrl(true);

	usleep(sleep);

	ret = mdss_diag_dsi_cmd_bta_sw_trigger(mdss_diag_panel_data);
	if (ret) {
		ret = MDSS_MIPICHK_RESULT_NG;
	} else {
		ret = MDSS_MIPICHK_RESULT_OK;
	}

	mdss_shdisp_video_transfer_ctrl(false);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test_cmd(uint8_t flame_cnt)
{
	int ret = 0;
	int ret2 = 0;
	int i;
	struct mdss_mdp_ctl *pctl;

	pctl = mdss_diag_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] mdpctrl is NULL.\n", __func__);
		return MDSS_MIPICHK_RESULT_NG;
	}
	
	if (!pctl->display_fnc) {
		pr_err("LCDERR:[%s] display_fnc is NULL.\n", __func__);
		return MDSS_MIPICHK_RESULT_NG;
	}

	for (i = 0; i < flame_cnt; i++) {
		mutex_lock(&pctl->lock);

		mdss_mdp_ctl_perf_update_ctl(pctl, 1);

		if (pctl->wait_pingpong) {
			ret2 = pctl->wait_pingpong(pctl, NULL);
			if(ret2){
				pr_err("LCDERR:[%s] failed to wait_pingpong(). (ret=%d)\n", __func__, ret2);
			}
		}

		ret = pctl->display_fnc(pctl, NULL);
		if (ret) {
			pr_err("LCDERR:[%s] failed to display_fnc(). (ret=%d)\n", __func__, ret);
			mutex_unlock(&pctl->lock);
			return MDSS_MIPICHK_RESULT_NG;
		}
		mutex_unlock(&pctl->lock);
	}

	if (pctl->wait_pingpong) {
		ret2 = pctl->wait_pingpong(pctl, NULL);
		if(ret2){
			pr_err("LCDERR:[%s] failed to wait_pingpong(). (ret=%d)\n", __func__, ret2);
		}
	}

	ret = mdss_diag_dsi_cmd_bta_sw_trigger(mdss_diag_panel_data);
	if (ret) {
		ret = MDSS_MIPICHK_RESULT_NG;
	} else {
		ret = MDSS_MIPICHK_RESULT_OK;
	}

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_read_sensitiv(uint8_t *read_data)
{
    int ret = 0;
	struct shdisp_dsi_cmd_desc cmd[1];
	char cmd_buf[1 + 2];
	char payload_page_ee[1][2] = {
	    {0xFF, 0xEE}
	};
	struct shdisp_dsi_cmd_desc cmds_sensitiv[] = {
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_page_ee[0]},
	};
	
#if defined(CONFIG_SHDISP_PANEL_ANDY)
		payload_page_ee[0][1] = 0xEE;
#elif defined(CONFIG_SHDISP_PANEL_ARIA)
		payload_page_ee[0][1] = 0xE0;
#endif  /* CONFIG_SHDISP_PANEL_ANDY || CONFIG_SHDISP_PANEL_ARIA */

	mdss_shdisp_host_dsi_tx(1, cmds_sensitiv, ARRAY_SIZE(cmds_sensitiv));

	cmd_buf[0] = 0x45;
	cmd_buf[1] = 0x00;

	cmd[0].dtype = SHDISP_DTYPE_DCS_READ;
	cmd[0].wait = 0x00;
	cmd[0].dlen = 1;
	cmd[0].payload = cmd_buf;

	ret = mdss_shdisp_host_dsi_rx(cmd, read_data, 1);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_dsi_cmd_bta_sw_trigger(struct mdss_panel_data *pdata)
{
	u32 status;
	int timeout_us = 35000, ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x098, 0x01);	/* trigger */
	wmb();

	/* Check for CMD_MODE_DMA_BUSY */
	if (readl_poll_timeout(((ctrl_pdata->ctrl_base) + 0x0008),
				status, ((status & 0x0010) == 0),
				0, timeout_us))
	{
		pr_info("%s: DSI status=%x failed\n", __func__, status);
		return -EIO;
	}

	ret = mdss_diag_dsi_ack_err_status(ctrl_pdata);
	
	pr_debug("%s: BTA done, status = %d\n", __func__, status);
	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_dsi_ack_err_status(struct mdss_dsi_ctrl_pdata *ctrl)
{
	u32 status;
	unsigned char *base;
	u32 ack = 0x10000000;

	base = ctrl->ctrl_base;

	status = MIPI_INP(base + 0x0068);/* DSI_ACK_ERR_STATUS */

	if (status) {
		MIPI_OUTP(base + 0x0068, status);
		/* Writing of an extra 0 needed to clear error bits */
		MIPI_OUTP(base + 0x0068, 0);

		status &= ~(ack);
		if(status){
			pr_err("%s: status=%x\n", __func__, status);
			return -EIO;
		}
	}
	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_clkchg(struct mdp_mipi_clkchg_param * mipi_clkchg_param)
{
	int ret = 0;
	struct mdss_mdp_ctl *pctl;
	struct mdss_panel_data *pdata;

	pctl = mdss_diag_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] mdpctrl is NULL.\n", __func__);
		return -EIO;
	}
	pdata = pctl->panel_data;

	mdss_shdisp_lock_recovery();

	mdss_diag_mipi_clkchg_host_data(mipi_clkchg_param, pdata);
#if defined(CONFIG_SHDISP_PANEL_ANDY)
	shdisp_api_set_freq_param(mipi_clkchg_param->panel.andy);
#endif  /* CONFIG_SHDISP_PANEL_ANDY */

	if (mdss_shdisp_is_disp_on()) {
		ret = mdss_diag_mipi_clkchg_setparam(mipi_clkchg_param, pctl);
	} else {
		ret = mdss_diag_mipi_clkchg_panel_clk_data(pdata);
	}

	mdss_shdisp_unlock_recovery();

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_clkchg_setparam(struct mdp_mipi_clkchg_param * mipi_clkchg_param, struct mdss_mdp_ctl *pctl)
{
	int ret = 0;
	struct fb_info * fbi;
	struct msm_fb_data_type *mfd;
	fbi = mdss_fb_get_fbinfo(0);
	mfd = (struct msm_fb_data_type *)fbi->par;

	if (pctl->is_video_mode) {
#ifndef SHDISP_DISABLE_HR_VIDEO
		ret |= mdss_mdp_hr_video_suspend(pctl, false);
#endif /* SHDISP_DISABLE_HR_VIDEO */
	}

	ret |= mdss_diag_mipi_clkchg_panel(mipi_clkchg_param, pctl);

	ret |= mdss_diag_mipi_clkchg_host(mipi_clkchg_param, pctl);

	if (pctl->is_video_mode) {
#ifndef SHDISP_DISABLE_HR_VIDEO
		ret |= mdss_mdp_hr_video_resume(pctl, false);
#endif /* SHDISP_DISABLE_HR_VIDEO */
	}

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_diag_mipi_clkchg_host_data(struct mdp_mipi_clkchg_param * mipi_clkchg_param, struct mdss_panel_data *pdata)
{
	int i;
	struct mdss_panel_info *pinfo = &(pdata->panel_info);

	pinfo->clk_rate = mipi_clkchg_param->host.clock_rate;
	pinfo->xres = mipi_clkchg_param->host.display_width;
	pinfo->yres = mipi_clkchg_param->host.display_height;
	pinfo->lcdc.h_pulse_width = mipi_clkchg_param->host.hsync_pulse_width;
	pinfo->lcdc.h_back_porch = mipi_clkchg_param->host.h_back_porch;
	pinfo->lcdc.h_front_porch = mipi_clkchg_param->host.h_front_porch;
	pinfo->lcdc.v_pulse_width = mipi_clkchg_param->host.vsync_pulse_width;
	pinfo->lcdc.v_back_porch = mipi_clkchg_param->host.v_back_porch;
	pinfo->lcdc.v_front_porch = mipi_clkchg_param->host.v_front_porch;
	pinfo->mipi.t_clk_post = mipi_clkchg_param->host.t_clk_post;
	pinfo->mipi.t_clk_pre = mipi_clkchg_param->host.t_clk_pre;
	for ( i=0; i<12; i++ ) {
		pinfo->mipi.dsi_phy_db.timing[i] = mipi_clkchg_param->host.timing_ctrl[i];
	}
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_clkchg_host(struct mdp_mipi_clkchg_param * mipi_clkchg_param, struct mdss_mdp_ctl *pctl)
{
	int ret = 0;
	struct mdss_panel_data *pdata = pctl->panel_data;

	ret |= mdss_diag_mipi_clkchg_panel_clk_update(pdata);
#ifndef SHDISP_DISABLE_HR_VIDEO
	ret |= mdss_mdp_hr_video_clkchg_mdp_update(pctl);
#endif /* SHDISP_DISABLE_HR_VIDEO */
	ret |= mdss_diag_mipi_clkchg_panel_porch_update(pdata);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_clkchg_panel_clk_update(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	ret = mdss_diag_mipi_clkchg_panel_clk_data(pdata);

	mdss_dsi_pll_relock(ctrl_pdata);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_panel_info *pinfo = &(pdata->panel_info);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	ret = mdss_dsi_clk_div_config(pinfo, pinfo->mipi.frame_rate);
	if (ret) {
		pr_err("LCDERR:[%s] mdss_dsi_clk_div_config err.\n", __func__);
		return ret;
	}
	pr_debug("%s: clk_rate = %d\n", __func__, pinfo->mipi.dsi_pclk_rate);
	ctrl_pdata->pclk_rate =
		pinfo->mipi.dsi_pclk_rate;
	ctrl_pdata->byte_clk_rate =
		pinfo->clk_rate / 8;

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_clkchg_panel_porch_update(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = &(pdata->panel_info);
	struct mipi_panel_info *mipi;
	u32 hbp, hfp, vbp, vfp, hspw, vspw, width, height;
	u32 ystride, bpp, data, dst_bpp;
	u32 dummy_xres, dummy_yres;
	u32 hsync_period, vsync_period;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	mdss_dsi_phy_init(pdata);

	dst_bpp = pdata->panel_info.fbc.enabled ?
		(pdata->panel_info.fbc.target_bpp) : (pinfo->bpp);

	hbp = mult_frac(pdata->panel_info.lcdc.h_back_porch, dst_bpp,
			pdata->panel_info.bpp);
	hfp = mult_frac(pdata->panel_info.lcdc.h_front_porch, dst_bpp,
			pdata->panel_info.bpp);
	vbp = mult_frac(pdata->panel_info.lcdc.v_back_porch, dst_bpp,
			pdata->panel_info.bpp);
	vfp = mult_frac(pdata->panel_info.lcdc.v_front_porch, dst_bpp,
			pdata->panel_info.bpp);
	hspw = mult_frac(pdata->panel_info.lcdc.h_pulse_width, dst_bpp,
			pdata->panel_info.bpp);
	vspw = pdata->panel_info.lcdc.v_pulse_width;
	width = mult_frac(pdata->panel_info.xres, dst_bpp,
			pdata->panel_info.bpp);
	height = pdata->panel_info.yres;

	if (pdata->panel_info.type == MIPI_VIDEO_PANEL) {
		dummy_xres = pdata->panel_info.lcdc.xres_pad;
		dummy_yres = pdata->panel_info.lcdc.yres_pad;
	}

	vsync_period = vspw + vbp + height + dummy_yres + vfp;
	hsync_period = hspw + hbp + width + dummy_xres + hfp;

	mipi  = &pdata->panel_info.mipi;
	if (pdata->panel_info.type == MIPI_VIDEO_PANEL) {
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x24,
			((hspw + hbp + width + dummy_xres) << 16 |
			(hspw + hbp)));
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x28,
			((vspw + vbp + height + dummy_yres) << 16 |
			(vspw + vbp)));
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x2C,
				((vsync_period - 1) << 16)
				| (hsync_period - 1));

		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x30, (hspw << 16));
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x34, 0);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x38, (vspw << 16));

	} else {		/* command mode */
		if (mipi->dst_format == DSI_CMD_DST_FORMAT_RGB888)
			bpp = 3;
		else if (mipi->dst_format == DSI_CMD_DST_FORMAT_RGB666)
			bpp = 3;
		else if (mipi->dst_format == DSI_CMD_DST_FORMAT_RGB565)
			bpp = 2;
		else
			bpp = 3;	/* Default format set to RGB888 */

		ystride = width * bpp + 1;

		/* DSI_COMMAND_MODE_MDP_STREAM_CTRL */
		data = (ystride << 16) | (mipi->vc << 8) | DTYPE_DCS_LWRITE;
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x60, data);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x58, data);

		/* DSI_COMMAND_MODE_MDP_STREAM_TOTAL */
		data = height << 16 | width;
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x64, data);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x5C, data);
	}

	mdss_dsi_sw_reset(pdata);
	mdss_dsi_host_init(pdata);
	mdss_dsi_op_mode_config(mipi->mode, pdata);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_clkchg_panel(struct mdp_mipi_clkchg_param * mipi_clkchg_param, struct mdss_mdp_ctl *pctl)
{
	int ret = 0;
#if defined(CONFIG_SHDISP_PANEL_ANDY)
	static char mipi_sh_andy_cmds_clkchgSetting[4][2] = {
		{0xFF, 0x05 },
		{0x90, 0x00 },
		{0x9B, 0x00 },
		{0xFF, 0x00 }
	};
	static struct shdisp_dsi_cmd_desc mipi_sh_andy_cmds_clkchg[] = {
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_andy_cmds_clkchgSetting[0]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_andy_cmds_clkchgSetting[1]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_andy_cmds_clkchgSetting[2]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_andy_cmds_clkchgSetting[3]},
	};

	mipi_sh_andy_cmds_clkchgSetting[1][1] = mipi_clkchg_param->panel.andy.rtn;

	mipi_sh_andy_cmds_clkchgSetting[2][1] = mipi_clkchg_param->panel.andy.gip;

	ret = mdss_shdisp_host_dsi_tx(1, mipi_sh_andy_cmds_clkchg, ARRAY_SIZE(mipi_sh_andy_cmds_clkchg));

#elif defined(CONFIG_SHDISP_PANEL_ARIA)

#endif	/* CONFIG_SHDISP_PANEL_ANDY || CONFIG_SHDISP_PANEL_ARIA */
	return ret;
}

