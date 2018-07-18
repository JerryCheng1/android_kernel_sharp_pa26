/* drivers/video/msm/mdss/mdss_shdisp.c  (Display Driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION
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
#include <linux/types.h>
#include <mach/board.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/shdisp_dsi.h>
#include "mdss_fb.h"
#include <mdss_dsi.h>
#include <mdss_mdp.h>

#define MDSS_SHDISP_DSI_COLLECT_MAX      (384)
#define MDSS_SHDISP_DSI_PAYLOAD_BUF_LEN  (4 * MDSS_SHDISP_DSI_COLLECT_MAX)

#define MDSS_DSI_CTRL_DSI_EN             BIT(0)
#define MDSS_DSI_CTRL_VIDEO_MODE_EN      BIT(1)
#define MDSS_DSI_CTRL_CMD_MODE_EN        BIT(2)

static int lcd_disp_on = false;
struct mdss_panel_data     * mdss_panel_data;
struct mdss_dsi_ctrl_pdata * mdss_dsi_ctrl;
static int mdss_shdisp_callback_data = 0;
static int mdss_shdisp_collect_cmd_cnt;
static struct dsi_cmd_desc mdss_shdisp_collect_cmds[MDSS_SHDISP_DSI_COLLECT_MAX];
static char mdss_shdisp_collect_payloads[MDSS_SHDISP_DSI_PAYLOAD_BUF_LEN];
static int mdss_shdisp_used_payloads = 0;
static int mdss_shdisp_video_transfer_ctrl_kickoff_flg = false;
static bool mdss_shdisp_is_required_dsi_clk_ctrl = true;

static int mdss_shdisp_is_cmdmode_eng_on(void);
static void mdss_shdisp_cmdmode_eng_ctrl(int enable);
static int mdss_shdisp_first_display_done = 0;
static int mdss_shdisp_video_transfer_ctrl_no_commit(struct msm_fb_data_type *mfd, int onoff);

static struct semaphore mdss_shdisp_recovery_sem;
static struct semaphore mdss_shdisp_host_dsi_cmd_sem;

static inline struct mdss_mdp_ctl* mdss_shdisp_get_mdpctrl(int fbinx);
static void mdss_shdisp_set_required_clk_ctrl(bool onoff);
static void mdss_shdisp_clk_ctrl(bool onoff);
static int mdss_shdisp_mdp_cmd_clk_ctrl(bool onoff);

int mdss_shdisp_mdp_hr_video_suspend(void);
int mdss_shdisp_mdp_hr_video_resume(void);
int mdss_shdisp_mdp_hr_video_clk_on(void);
int mdss_shdisp_mdp_hr_video_clk_off(void);

extern int shdisp_api_tri_led_set_color(struct shdisp_tri_led *tri_led);

extern int mdss_mdp_cmd_cancel_clk_work(struct mdss_mdp_ctl *pctl);
extern int mdss_mdp_cmd_clk_ctrl(struct mdss_mdp_ctl *pctl, bool onoff);

#ifdef SHDISP_DET_DSI_MIPI_ERROR
extern void mdss_dsi_phy_dln0_err_clear(struct mdss_dsi_ctrl_pdata *ctrl);
extern void mdss_dsi_phy_dln0_err_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, bool onoff);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */
#ifndef SHDISP_DISABLE_HR_VIDEO
extern void mdss_mdp_wait_sw_tg_off(void);
extern int mdss_mdp_fps_led_start(void);
extern void mdss_mdp_fps_led_stop(void);
extern int mdss_mdp_hr_video_suspend(struct mdss_mdp_ctl *ctl, int tg_en_flg);
extern int mdss_mdp_hr_video_resume(struct mdss_mdp_ctl *ctl, int tg_en_flg);
extern void mdss_mdp_hr_video_transfer_ctrl(struct msm_fb_data_type *mfd, int onoff);
extern int mdss_mdp_hr_video_clk_ctrl(struct mdss_mdp_ctl *ctl, int onoff);
#else  /* SHDISP_DISABLE_HR_VIDEO */
extern void mdss_mdp_video_transfer_ctrl(struct msm_fb_data_type *mfd, int onoff);
#endif /* SHDISP_DISABLE_HR_VIDEO */
extern struct fb_info *mdss_fb_get_fbinfo(int id);

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_lock_recovery(void)
{
	down(&mdss_shdisp_recovery_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_unlock_recovery(void)
{
	up(&mdss_shdisp_recovery_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_lock_host_dsi_cmd(void)
{
	down(&mdss_shdisp_host_dsi_cmd_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_unlock_host_dsi_cmd(void)
{
	up(&mdss_shdisp_host_dsi_cmd_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
bool mdss_shdisp_get_disp_status(void)
{
	shdisp_api_get_boot_context();

	if( shdisp_api_get_boot_disp_status() ) {
		lcd_disp_on = true;
		return true;
	}

	return false;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_power_on(void)
{
	if (lcd_disp_on == true) {
		pr_debug("%s: already power on.\n", __func__);
		return;
	}
	shdisp_api_main_lcd_power_on();
}


/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_power_off(void)
{
	shdisp_api_main_lcd_power_off();
}


/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_on(struct mdss_panel_data *pdata)
{
	int cmdengon;
	mdss_panel_data = pdata;
	mdss_dsi_ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	mdss_shdisp_set_required_clk_ctrl(false);

	cmdengon = mdss_shdisp_is_cmdmode_eng_on();
	if (!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(1);
	}

	mdss_shdisp_collect_cmd_cnt = 0;
	mdss_shdisp_used_payloads = 0;
	shdisp_api_main_lcd_disp_on();
	if (mdss_panel_data->panel_info.type == MIPI_VIDEO_PANEL) {
		mdss_shdisp_dsi_panel_start_display();
	}

	if (!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(0);
	}
	
	mdss_shdisp_set_required_clk_ctrl(true);
}


/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_set_dsi_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	mdss_dsi_ctrl = ctrl_pdata;
	mdss_panel_data = &ctrl_pdata->panel_data;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_off(void)
{
	int cmdengon;

	mdss_shdisp_set_required_clk_ctrl(false);

	cmdengon = mdss_shdisp_is_cmdmode_eng_on();
	if(!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(1);
	}

	mdss_shdisp_collect_cmd_cnt = 0;
	mdss_shdisp_used_payloads = 0;
	shdisp_api_main_lcd_disp_off();
	lcd_disp_on = false;

	if(!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(0);
	}
	mdss_shdisp_first_display_done = 0;
	
	mdss_shdisp_set_required_clk_ctrl(true);

	mdss_dsi_ctrl = NULL;
	mdss_panel_data = NULL;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
int mdss_shdisp_is_disp_on(void)
{
	return lcd_disp_on;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_start_display(void)
{
	shdisp_api_main_lcd_start_display();
	lcd_disp_on = true;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_post_video_start()
{
	shdisp_api_main_lcd_post_video_start();
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_bkl_ctl(u32 bl_level)
{
	struct shdisp_main_bkl_ctl bkl;

	pr_debug("%s: called bl_level=%u\n", __func__, bl_level);

	if( bl_level == 0 ) {
		shdisp_api_main_bkl_off();
	} else {
		bkl.mode = SHDISP_MAIN_BKL_MODE_FIX;
		bkl.param = bl_level;
		shdisp_api_main_bkl_on(&bkl);
	}
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_tri_led_set_color(char red, char green, char blue)
{
	struct shdisp_tri_led param;

	param.red = red;
	param.green = green;
	param.blue = blue;
	param.ext_mode = SHDISP_TRI_LED_EXT_MODE_DISABLE;
	param.led_mode = SHDISP_TRI_LED_MODE_NORMAL;

	shdisp_api_tri_led_set_color(&param);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */ 
/* ----------------------------------------------------------------------- */
void mdss_shdisp_shutdown( void )
{
	shdisp_api_shutdown();
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_dsi_to_mdss_dsi(const struct shdisp_dsi_cmd_desc * shdisp_cmds,  struct dsi_cmd_desc * mdss_cmds,
								int size, int isAck)
{
	int cnt;
	for (cnt = 0; cnt != size; cnt++) {
		if (mdss_shdisp_used_payloads + shdisp_cmds->dlen < MDSS_SHDISP_DSI_PAYLOAD_BUF_LEN) {
			mdss_cmds->dchdr.dtype  = shdisp_cmds->dtype;
			mdss_cmds->dchdr.last = shdisp_cmds->wait ? 1 : 0;
			mdss_cmds->dchdr.vc	    = 0;
			mdss_cmds->dchdr.ack    = isAck ? 1 : 0;
			mdss_cmds->dchdr.wait   = shdisp_cmds->wait ? ((shdisp_cmds->wait+1000)/1000) : 0; /* mdss_dsi(ms) <- shdisp_dsi(usec) */
			mdss_cmds->dchdr.dlen   = shdisp_cmds->dlen;
			mdss_cmds->payload      = mdss_shdisp_collect_payloads + mdss_shdisp_used_payloads;
			memcpy(mdss_cmds->payload, shdisp_cmds->payload, shdisp_cmds->dlen);
			mdss_shdisp_used_payloads += shdisp_cmds->dlen;
		} else {
			pr_err("LCDERR: buffer size over %s: shdisp_cmds->dlen=%d, mdss_shdisp_used_payloads = %d\n",
			                                __func__, shdisp_cmds->dlen, mdss_shdisp_used_payloads );
		}
		mdss_cmds++;
		shdisp_cmds++;
	}
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_collect_cmd(struct shdisp_dsi_cmd_desc * shdisp_cmds, int size)
{
	if (mdss_shdisp_collect_cmd_cnt + size >= MDSS_SHDISP_DSI_COLLECT_MAX) {
		pr_err("LCDERR: buffer size over %s: size=%d, mdss_shdisp_collect_cmd_cnt = %d\n",
		                                __func__, size, mdss_shdisp_collect_cmd_cnt );
		return;
	}
	mdss_shdisp_dsi_to_mdss_dsi(shdisp_cmds, &mdss_shdisp_collect_cmds[mdss_shdisp_collect_cmd_cnt], size, 0);
	mdss_shdisp_collect_cmd_cnt += size;
	pr_debug("%s: size=%d, mdss_shdisp_collect_cmd_cnt = %d\n", __func__, size, mdss_shdisp_collect_cmd_cnt );
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_shdisp_kick_collect_cmd(void)
{
	int ret = 0;
	int cmdengon;
	struct dcs_cmd_req cmdreq;

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		return SHDISP_RESULT_FAILURE;
	}

	pr_debug("%s: begin cnt=%d", __func__, mdss_shdisp_collect_cmd_cnt);
	if (!mdss_shdisp_collect_cmd_cnt) {
		return SHDISP_RESULT_SUCCESS;
	}

	mdss_shdisp_collect_cmds[mdss_shdisp_collect_cmd_cnt-1].dchdr.last   = 1;    
	pr_debug("%s: kick_count = %d\n", __func__, mdss_shdisp_collect_cmd_cnt );

	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.cmds = mdss_shdisp_collect_cmds;
	cmdreq.cmds_cnt = mdss_shdisp_collect_cmd_cnt;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	cmdengon = mdss_shdisp_is_cmdmode_eng_on();
	if(!cmdengon){
		mdss_shdisp_cmdmode_eng_ctrl(1);
	}
	ret = mdss_dsi_cmdlist_put(mdss_dsi_ctrl, &cmdreq);
	if (!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(0);
	}

	mdss_shdisp_collect_cmd_cnt = 0;
	mdss_shdisp_used_payloads = 0;

	pr_debug("%s: end", __func__);
	if (!ret) {
		return SHDISP_RESULT_SUCCESS;
	} else {
		return SHDISP_RESULT_FAILURE;
	}
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_shdisp_is_cmdmode_eng_on(void)
{
	int dsi_ctrl;
	int cmdmode_on = MDSS_DSI_CTRL_DSI_EN | MDSS_DSI_CTRL_CMD_MODE_EN;
	int ret = 0;

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		return 0;
	}

	dsi_ctrl = MIPI_INP((mdss_dsi_ctrl->ctrl_base) + 0x0004);
	
	if ((dsi_ctrl&cmdmode_on) == cmdmode_on) {
		ret = 1;
	} else {
		ret = 0;
	}
	return ret;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_cmdmode_eng_ctrl(int enable)
{
	int mode;

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		return;
	}

	mode = (enable ? DSI_CMD_MODE : \
                             (mdss_dsi_ctrl->panel_mode == DSI_CMD_MODE ? \
                                  DSI_CMD_MODE : DSI_VIDEO_MODE \
                             ) \
                   );

	pr_debug("%s: request=%d\n", __func__, enable);

	mdss_dsi_op_mode_config(mode, mdss_panel_data);

	return;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_host_dsi_init_cbdata(void)
{
	mdss_shdisp_callback_data = 0xffff;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_host_dsi_get_cbdata(void)
{
	return mdss_shdisp_callback_data;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_host_dsi_cb(int data)
{
	mdss_shdisp_callback_data = data;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_host_dsi_tx(int commit, struct shdisp_dsi_cmd_desc * shdisp_cmds, int size)
{
	int ret = 0;
	mdss_shdisp_lock_host_dsi_cmd();
	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		mdss_shdisp_unlock_host_dsi_cmd();
		return SHDISP_RESULT_FAILURE;
	}

	mdss_shdisp_collect_cmd(shdisp_cmds, size);

	if (commit) {
		mdss_shdisp_clk_ctrl(true);

#ifndef SHDISP_DISABLE_HR_VIDEO
		if (mdss_dsi_ctrl->panel_mode == DSI_VIDEO_MODE) {
			mdss_mdp_wait_sw_tg_off();
		}
#endif /* SHDISP_DISABLE_HR_VIDEO */

		ret = mdss_shdisp_kick_collect_cmd();

		mdss_shdisp_clk_ctrl(false);
	}

	mdss_shdisp_unlock_host_dsi_cmd();

	if (!ret) {
		return SHDISP_RESULT_SUCCESS;
	} else {
		return SHDISP_RESULT_FAILURE;
	}
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_host_dsi_rx(struct shdisp_dsi_cmd_desc * cmds, unsigned char * rx_data, int rx_size)
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc mdss_cmds;
	char payload[2];
	int cmdengon;

	mdss_shdisp_lock_host_dsi_cmd();

	if ( !mdss_dsi_ctrl ) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		mdss_shdisp_unlock_host_dsi_cmd();
		return SHDISP_RESULT_FAILURE;
	}

#ifndef SHDISP_DISABLE_HR_VIDEO
	if (mdss_dsi_ctrl->panel_mode == DSI_VIDEO_MODE) {
		mdss_mdp_wait_sw_tg_off();
	}
#endif /* SHDISP_DISABLE_HR_VIDEO */

	mdss_shdisp_clk_ctrl(true);

	ret = mdss_shdisp_kick_collect_cmd();
	if (ret) {
		mdss_shdisp_clk_ctrl(false);
		mdss_shdisp_unlock_host_dsi_cmd();
		return SHDISP_RESULT_FAILURE;
	}

	memset(&cmdreq, 0, sizeof(cmdreq) );
	memset(&mdss_cmds, 0, sizeof(mdss_cmds) );
	mdss_shdisp_dsi_to_mdss_dsi(cmds, &mdss_cmds, 1, 1);
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.cmds = &mdss_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.cb = mdss_shdisp_host_dsi_cb;
	cmdreq.rbuf = rx_data;
	cmdreq.rlen = rx_size;

	if (mdss_cmds.dchdr.dlen>1) {
		payload[0] = mdss_cmds.payload[0];
		payload[1] = mdss_cmds.payload[1];
	} else {
		payload[0] = mdss_cmds.payload[0];
		payload[1] = 0;
	}

	mdss_cmds.payload = payload;

	cmdengon = mdss_shdisp_is_cmdmode_eng_on();
	if (!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(1);
	}

	mdss_shdisp_host_dsi_init_cbdata();
	mdss_dsi_cmdlist_put(mdss_dsi_ctrl, &cmdreq);

	pr_debug( "rx_data: payload[0][1] = 0x%02x, 0x%02x\n", payload[0], payload[1] );

	if (cmdreq.rlen != mdss_shdisp_host_dsi_get_cbdata()) {
		pr_err("LCDERR: %s callback_data=%d\n", __func__, mdss_shdisp_host_dsi_get_cbdata());
		if (!cmdengon) {
			mdss_shdisp_cmdmode_eng_ctrl(0);
		}
		mdss_shdisp_clk_ctrl(false);
		mdss_shdisp_unlock_host_dsi_cmd();
		return SHDISP_RESULT_FAILURE;
	}

	if (!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(0);
	}

	mdss_shdisp_clk_ctrl(false);
	mdss_shdisp_unlock_host_dsi_cmd();

	return SHDISP_RESULT_SUCCESS;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_video_transfer_ctrl(int onoff)
{
	struct fb_info * fbi;
	struct msm_fb_data_type *mfd;
	fbi = mdss_fb_get_fbinfo(0);
	mfd = (struct msm_fb_data_type *)fbi->par;
#ifndef SHDISP_DISABLE_HR_VIDEO
	mdss_mdp_hr_video_transfer_ctrl(mfd, onoff);
#else  /* SHDISP_DISABLE_HR_VIDEO */
	mdss_mdp_video_transfer_ctrl(mfd, onoff);
#endif /* SHDISP_DISABLE_HR_VIDEO */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_shdisp_video_transfer_ctrl_no_commit(struct msm_fb_data_type *mfd, int onoff)
{
	int ret = 0;
	struct mdss_mdp_ctl *ctl;

	pr_debug("%s : onoff=%d\n", __func__, onoff);

	ctl = mfd_to_ctl(mfd);
	if (ctl == NULL) {
		pr_debug("%s : ctl is NULL\n", __func__);
		return -EINVAL;
	}

#ifndef SHDISP_DISABLE_HR_VIDEO
	if (onoff) {
		ret = mdss_mdp_hr_video_resume(ctl, false);
	} else {
		ret = mdss_mdp_hr_video_suspend(ctl, false);
	}
#endif /* SHDISP_DISABLE_HR_VIDEO */
	pr_debug("%s : ret=%d\n", __func__, ret);
	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_video_transfer_ctrl_set_flg(struct msm_fb_data_type *mfd, int change)
{
	if ((mfd->panel.type == MIPI_VIDEO_PANEL) &&
	    (mdss_shdisp_video_transfer_ctrl_kickoff_flg != change)) {
		mdss_shdisp_video_transfer_ctrl_kickoff_flg = change;
		pr_debug("%s : video_transfer_ctrl_kickoff_flg=%d\n", __func__, change);
	}
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_video_transfer_ctrl_kickoff(struct msm_fb_data_type *mfd, int onoff)
{
	int ret = -EPERM;

	if ((mfd->panel.type == MIPI_VIDEO_PANEL) &&
	    (mdss_shdisp_video_transfer_ctrl_kickoff_flg == true)) {
		ret = mdss_shdisp_video_transfer_ctrl_no_commit(mfd, onoff);
	}
	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_display_done(void)
{
	int ret = SHDISP_RESULT_SUCCESS;

	if (mdss_shdisp_first_display_done == 0) {
		mdss_shdisp_first_display_done = 1;
		ret = shdisp_api_main_display_done();
	}
	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static inline struct mdss_mdp_ctl* mdss_shdisp_get_mdpctrl(int fbinx)
{
	return ((struct mdss_overlay_private*)((struct msm_fb_data_type*)mdss_fb_get_fbinfo(fbinx)->par)->mdp.private1)->ctl;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_set_required_clk_ctrl(bool onoff)
{
	if ( !mdss_dsi_ctrl ){
		mdss_shdisp_is_required_dsi_clk_ctrl = false;
		return;
	}
	
	mdss_shdisp_is_required_dsi_clk_ctrl = onoff;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_clk_ctrl(bool onoff)
{
	if ( !mdss_dsi_ctrl ){
		return;
	}
	
	if ( !mdss_shdisp_is_required_dsi_clk_ctrl ) {
		return;
	}
	
	if ( mdss_dsi_ctrl->panel_mode == DSI_CMD_MODE ) {
		mdss_shdisp_mdp_cmd_clk_ctrl(onoff);
	} else {
#if defined(CONFIG_SHDISP_PANEL_ANDY)
		if (onoff) {
			mdss_shdisp_mdp_hr_video_clk_on();
		} else {
			mdss_shdisp_mdp_hr_video_clk_off();
		}
#endif /* CONFIG_SHDISP_PANEL_ANDY */
	}
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_shdisp_mdp_cmd_clk_ctrl(bool onoff)
{
	int ret;
	struct mdss_mdp_ctl *pctl;

	pr_debug("LCDDBG:[%s] enter - (onoff=%d)\n", __func__, onoff);

	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] mdpctrl is NULL.\n", __func__);
		return -EINVAL;
	}

	ret = mdss_mdp_cmd_cancel_clk_work(pctl);
	if (ret) {
		pr_err("LCDERR:[%s] failed to cancel DSI clock work. (ret=%d mdp=%p)\n", __func__, ret, pctl);
		return ret;
	}

	ret = mdss_mdp_cmd_clk_ctrl(pctl, onoff);
	if (ret) {
		pr_err("LCDERR:[%s] failed to control DSI clock. (ret=%d mdp=%p)\n", __func__, ret, pctl);
		return ret;
	}

	pr_debug("LCDDBG:[%s] leave - (ret=0)\n", __func__);
	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_mdp_cmd_kickoff()
{
	int ret;
	struct mdss_mdp_ctl *pctl;

	pr_debug("LCDDBG:[%s] enter - ()\n", __func__);

	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] mdpctrl is NULL.\n", __func__);
		return;
	}

	if (!pctl->display_fnc) {
		pr_err("LCDERR:[%s] display_fnc is NULL.\n", __func__);
		return;
	}

	mutex_lock(&pctl->lock);

	ret = pctl->display_fnc(pctl, NULL);
	if (ret) {
		pr_err("LCDERR:[%s] failed to display_fnc(). (ret=%d)\n", __func__, ret);
		mutex_unlock(&pctl->lock);
		return;
	}

	mutex_unlock(&pctl->lock);

	pr_debug("LCDDBG:[%s] leave - ()\n", __func__);
	return;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_mdp_hr_video_suspend(void)
{
	struct mdss_mdp_ctl *pctl;
	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] pctl is NULL.\n", __func__);
		return -EINVAL;
	}
#ifndef SHDISP_DISABLE_HR_VIDEO
	return mdss_mdp_hr_video_suspend(pctl, true);
#else  /* SHDISP_DISABLE_HR_VIDEO */
	return 0;
#endif /* SHDISP_DISABLE_HR_VIDEO */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_mdp_hr_video_resume(void)
{
	struct mdss_mdp_ctl *pctl;
	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] pctl is NULL.\n", __func__);
		return -EINVAL;
	}
#ifndef SHDISP_DISABLE_HR_VIDEO
	return mdss_mdp_hr_video_resume(pctl, false);
#else  /* SHDISP_DISABLE_HR_VIDEO */
	return 0;
#endif /* SHDISP_DISABLE_HR_VIDEO */
}

int mdss_shdisp_mdp_hr_video_clk_on(void)
{
	struct mdss_mdp_ctl *pctl;
	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] pctl is NULL.\n", __func__);
		return -EINVAL;
	}
#ifndef SHDISP_DISABLE_HR_VIDEO
	return mdss_mdp_hr_video_clk_ctrl(pctl, 1);
#else  /* SHDISP_DISABLE_HR_VIDEO */
	return 0;
#endif /* SHDISP_DISABLE_HR_VIDEO */
}

int mdss_shdisp_mdp_hr_video_clk_off(void)
{
	struct mdss_mdp_ctl *pctl;
	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] pctl is NULL.\n", __func__);
		return -EINVAL;
	}
#ifndef SHDISP_DISABLE_HR_VIDEO
	return mdss_mdp_hr_video_clk_ctrl(pctl, 0);
#else  /* SHDISP_DISABLE_HR_VIDEO */
	return 0;
#endif /* SHDISP_DISABLE_HR_VIDEO */
}

#ifdef SHDISP_DET_DSI_MIPI_ERROR
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_mipi_err_clear()
{
	pr_debug("LCDDBG:[%s] enter - ()\n", __func__);

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR:[%s] invalid DSI control.\n", __func__);
		return;
	}

	mdss_shdisp_clk_ctrl(true);
	mdss_dsi_phy_dln0_err_clear(mdss_dsi_ctrl);
	mdss_shdisp_clk_ctrl(false);

	pr_debug("LCDDBG:[%s] leave - ()\n", __func__);
	return;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_dsi_mipi_err_ctrl(bool enable)
{
	pr_debug("LCDDBG:[%s] enter - (enable=%d)\n", __func__, enable);

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR:[%s] invalid DSI control.\n", __func__);
		return -EINVAL;
	}

	mdss_shdisp_clk_ctrl(true);
	mdss_dsi_phy_dln0_err_ctrl(mdss_dsi_ctrl, enable);
	mdss_shdisp_clk_ctrl(false);

	pr_debug("LCDDBG:[%s] leave - (ret=0)\n", __func__);
	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_det_recovery(int status)
{
	int nret;

	pr_debug("LCDDBG:[%s] enter - (status=%d)\n", __func__, status);

	pr_err("LCDERR:[%s] MIPI error has been detected!!! (status=%#x)\n", __func__, status);

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR:[%s] invalid DSI control, will be exited without recovery. ()\n", __func__);
		return;
	}

	nret = shdisp_api_do_mipi_dsi_det_recovery();
	if (nret) {
		pr_err("LCDERR:[%s] faild to recovery. (ret=%d)\n", __func__, nret);
		mdss_shdisp_clk_ctrl(true);
		mdss_dsi_phy_dln0_err_ctrl(mdss_dsi_ctrl, true);
		mdss_shdisp_clk_ctrl(false);
		return;
	}

	pr_debug("LCDDBG:[%s] leave - ()\n", __func__);
	return;
}
#endif /* SHDISP_DET_DSI_MIPI_ERROR */

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_set_lp00_mode(int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	u32 dsi_ctrl;
	static u32 backup = 0;

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		return SHDISP_RESULT_FAILURE;
	}

	pr_debug("%s: called\n", __func__);
	ctrl_pdata = container_of(mdss_panel_data, struct mdss_dsi_ctrl_pdata,
			panel_data);
	dsi_ctrl = MIPI_INP((ctrl_pdata->ctrl_base) + 0x0004);
	pr_debug("%s: addr=0x%p INP dsi_ctrl=0x%02X\n", __func__, ((ctrl_pdata->ctrl_base) + 0x0004), dsi_ctrl);
	if (!enable) {
		backup = dsi_ctrl;
		dsi_ctrl = 0x000;
	} else {
		dsi_ctrl |= backup;
		backup = 0;
	}
	pr_debug("%s: addr=0x%p OUTP dsi_ctrl=0x%02X\n", __func__, ((ctrl_pdata->ctrl_base) + 0x0004), dsi_ctrl);
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0004, dsi_ctrl);
	wmb();

	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_fps_led_start(void)
{
#ifndef SHDISP_DISABLE_HR_VIDEO
	int ret;

	ret = mdss_mdp_fps_led_start();
	if (!ret) {
		return SHDISP_RESULT_SUCCESS;
	} else {
		return SHDISP_RESULT_FAILURE;
	}
#else  /* SHDISP_DISABLE_HR_VIDEO */
	return SHDISP_RESULT_SUCCESS;
#endif /* SHDISP_DISABLE_HR_VIDEO */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_fps_led_stop(void)
{
#ifndef SHDISP_DISABLE_HR_VIDEO
	mdss_mdp_fps_led_stop();
#endif /* SHDISP_DISABLE_HR_VIDEO */
}

static int __init mdss_shdisp_init(void)
{
	sema_init(&mdss_shdisp_recovery_sem,1);
	sema_init(&mdss_shdisp_host_dsi_cmd_sem, 1);
	return 0;
}
module_init(mdss_shdisp_init);