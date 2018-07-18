/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/iopoll.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/memblock.h>
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (MFR) */
#include <linux/uaccess.h>
#endif /* CONFIG_SHDISP */

#include "mdss_fb.h"
#include "mdss_mdp.h"
#include "mdss_panel.h"
#include "mdss_debug.h"

#ifdef CONFIG_SHDISP /* CUST_ID_00050 */
#include "mdss_dsi.h"
#endif /* CONFIG_SHDISP */

#include "mdss_mdp_trace.h"
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
#include "mdss_dsi.h"
#endif /* CONFIG_SHDISP */
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (FPSLED) */
#include "mdss_shdisp.h"
#endif /* CONFIG_SHDISP */

/* Note:
 * VSYNC_TIMER must be larger than actual vsync period
 */
#define VSYNC_TIMER		16666666
#define INACTIVITY_TIMER	10
#define VSYNC_TIMEOUT_US	100000

/* The number of ticks to wait for before starting the process of entering hr_video
 * mode. i.e blank blank blank repeat repeat
 */
#define INACTIVITY_CNT 0

/* The number of blank frames to be inserted 
 */
#define BLANK_NUM 3

/* The number of repeat frames to be sent
 */
#define REPEAT_CNT 2

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
/* The number of wait frames to be start hr_video mode
 */
#define WAIT_CNT 1 /* must not be 0 */
#endif /* CONFIG_SHDISP */

/* The fps for the minimum refresh rate. This is the number of ticks before we
 * send the minimum update frame. 
 * e.g if hr_video refresh rate is 1fps, we need to send frames every 60 ticks
 * if 2fps, this needs to be 60/2 = 30
 */
#define HR_VIDEO_REFRESH_CNT 60

/* This is the delay in msecs that the DSI pll takes to lock
 */
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
#define DSI_CLK_DELAY 1 //vsync
#else  /* CONFIG_SHDISP */
#define DSI_CLK_DELAY 10 //ms
#define SCHEDULE_DSI_CLK_ON_TIME	nsecs_to_jiffies((VSYNC_TIMER*HR_VIDEO_REFRESH_CNT) - (DSI_CLK_DELAY))
#endif /* CONFIG_SHDISP */

#define MDP_INTR_MASK_INTF_VSYNC(intf_num) \
	(1 << (2 * (intf_num - MDSS_MDP_INTF0) + MDSS_MDP_IRQ_INTF_VSYNC))

int mdss_dsi_state_reset(struct mdss_panel_data *pdata);
/* Possible states for hr_video state machine i.e.ctx->hr_video_state */
enum mdss_mdp_hr_video_mode {
	POWER_ON,
	NEW_UPDATE,
	FIRST_UPDATE,
	BLANK_FRAME,
	REPEAT_FRAME,
	HR_VIDEO_MODE,
	SOFT_HR_VIDEO_MODE,
	POWER_OFF,
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (MFR) */
	HR_VIDEO_MFR,
#endif /* CONFIG_SHDISP */
};

/* Possible states for timing generator i.e. ctx->tg_state */
enum mdss_mdp_hr_video_tg_state {
	HW_TG_ON,
	SW_TG_OFF,
	HW_TG_OFF,
};

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (POL) */
enum mdss_mdp_hr_pol_state {
	POL_STATE_INIT,
	POL_STATE_POS,
	POL_STATE_NEG,
};
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00037 */
enum sus_state {
	SUS_IDLE = 0,
	SUS_START,
	SUS_DONE,
	RES_WAIT,
	RES_START,
};
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (FPSLED) */
enum fps_led_state {
	FPS_LED_STATE_NONE = 0,
	FPS_LED_STATE_60HZ,
	FPS_LED_STATE_30HZ,
	FPS_LED_STATE_1HZ,
};

struct fps_led_ctx {
	int inited;
	enum fps_led_state state;
	spinlock_t lock;
	struct hrtimer timer;
	ktime_t frame_time;
	ktime_t frame_time_bef;
	unsigned int frame_hist;
	char led_red;
	char led_green;
	char led_blue;
	struct work_struct light_work;
};
#endif /* CONFIG_SHDISP */

/* intf timing settings */
struct intf_timing_params {
	u32 width;
	u32 height;
	u32 xres;
	u32 yres;

	u32 h_back_porch;
	u32 h_front_porch;
	u32 v_back_porch;
	u32 v_front_porch;
	u32 hsync_pulse_width;
	u32 vsync_pulse_width;

	u32 border_clr;
	u32 underflow_clr;
	u32 hsync_skew;
};

struct mdss_mdp_hr_video_ctx {
	u32 intf_num;
	char __iomem *base;
	u32 intf_type;
	u8 ref_cnt;
	bool power_on;
	int vsync_pending;
	int tg_toggle_pending;
#ifndef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	ktime_t last_hrtick_time;
#endif /* CONFIG_SHDISP */

	u8 timegen_en;
	bool polling_en;
	u32 poll_cnt;
	struct completion vsync_comp;
	atomic_t vsync_handler_cnt;
	s32 hrtimer_next_wait_ns;
	int wait_pending;
	enum mdss_mdp_hr_video_tg_state tg_state;
	enum mdss_mdp_hr_video_mode hr_video_state;
	u32 tick_cnt;
	struct mutex clk_mtx;
	int clk_enabled;
	struct work_struct clk_work;
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
	struct work_struct hr_video_clk_work;
#else  /* CONFIG_SHDISP */
	struct delayed_work hr_video_clk_work;
#endif /* CONFIG_SHDISP */

	spinlock_t hrtimer_lock;
	struct hrtimer vsync_hrtimer;
	bool hrtimer_init;
	bool repeat_frame_running;

	atomic_t vsync_ref;
	spinlock_t vsync_lock;
	struct mutex vsync_mtx;
	struct list_head vsync_handlers;
	struct mdss_mdp_ctl *ctl;

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	ktime_t curr;
	ktime_t next;
	int vsync_skip;
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (POL) */
	enum mdss_mdp_hr_pol_state pol_state;
	int pol_first;
	int pol_sum;
	ktime_t pol_time;
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (MFR) */
	int mfr;
	int mfr_req;
	int mfr_state;
	struct completion mfr_comp;
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (FPSLED) */
	struct fps_led_ctx fps_led;
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
	int work_cnt;
	int work_ctrl;
	int work_need_notify;
	struct completion work_comp;
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-HRT) */
	ktime_t hrt_hint;
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00037 */
	int suspend;
	int suspend_tg;
	struct completion suspend_comp;
	struct mutex suspend_mtx;
	struct mutex clk_ctrl_mtx;
	int clk_cnt;
#endif /* CONFIG_SHDISP */
};

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (FPSLED) */
static void mdss_mdp_fps_led_vsync(
		struct mdss_mdp_hr_video_ctx *ctx,
		ktime_t vsync_time);
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSIERR) */
static spinlock_t *hr_video_tg_state_lock = NULL;
static enum mdss_mdp_hr_video_tg_state *hr_video_tg_state = NULL;

int mdss_mdp_get_tg_state(void)
{
	enum mdss_mdp_hr_video_tg_state ret;
	unsigned long flags;
	if (!hr_video_tg_state || !hr_video_tg_state_lock) {
		pr_err("not initialized");
		return HW_TG_OFF;
	}
	spin_lock_irqsave(hr_video_tg_state_lock, flags);
	ret = *hr_video_tg_state;
	spin_unlock_irqrestore(hr_video_tg_state_lock, flags);
	return ret;
}

void mdss_mdp_wait_sw_tg_off(void)
{
	const int sleep_time = 100; // us
	int retry = 20000 / sleep_time; // 20ms
	int flg = 0;

	while (mdss_mdp_get_tg_state() == SW_TG_OFF && retry > 0) {
		if (!flg) {
			flg = 1;
			pr_debug("wait start\n");
		}
		usleep(sleep_time);
		retry--;
	}

	if (retry == 0) {
		pr_warn("timed out.\n");
	}

	if (flg) {
		pr_debug("wait end\n");
	}
}
#endif /* CONFIG_SHDISP */

#if 0 /* CONFIG_SHDISP
redef: drivers/video/msm/mdss/mdss_mdp.h
JERRY_UPDATE
CUST_ID_00012 (1HZ) */
static inline struct mdss_mdp_ctl *mdss_mdp_get_split_ctl(
		struct mdss_mdp_ctl *ctl)
{
	if (ctl && ctl->mixer_right && (ctl->mixer_right->ctl != ctl))
		return ctl->mixer_right->ctl;

	return NULL;
}
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (POL) */
static int mdss_mdp_timegen_enable(struct mdss_mdp_ctl *ctl, bool enable);

static void mdss_mdp_hr_polarity_init(struct mdss_mdp_hr_video_ctx *ctx)
{
	pr_debug("POLARITY: init\n");

	ctx->pol_state = POL_STATE_INIT;
	ctx->pol_first = true;
	ctx->pol_sum = 0;
}

static void mdss_mdp_hr_polarity_commit(struct mdss_mdp_hr_video_ctx *ctx)
{
	pr_debug("POLARITY: commit\n");

	ctx->pol_first = true;
}

static void mdss_mdp_hr_polarity_exec(
		struct mdss_mdp_ctl *ctl,
		ktime_t vsync_time,
		int vsync_pending)
{
	struct mdss_mdp_hr_video_ctx *ctx = ctl->priv_data;

	if (ctx->tg_state != HW_TG_OFF) {
		enum mdss_mdp_hr_pol_state pol_state = ctx->pol_state;

		if (pol_state == POL_STATE_INIT) {
			pol_state = POL_STATE_POS;

			pr_debug("POLARITY: pol_time:%14lld pol_sum:%6d pol_state:INIT "
					 "hr_video_state=%d, vsync_pending=%d\n",
				ktime_to_ns(vsync_time),
				ctx->pol_sum,
				ctx->hr_video_state,
				vsync_pending);
		} else {
			s64 diff;
			int diff_frame;
			int biased = false;
			int pol_sum = ctx->pol_sum;

			diff = ktime_to_ns(ktime_sub(vsync_time, ctx->pol_time));
			if (diff < 0) {
				pr_warn("POLARITY: negative diff:%lld\n", diff);
				diff = 0;
			} else if (diff > (VSYNC_TIMER * HR_VIDEO_REFRESH_CNT * 2)) {
				pr_warn("POLARITY: too large diff:%lld\n", diff);
				diff = VSYNC_TIMER * HR_VIDEO_REFRESH_CNT * 2;
			}

			diff_frame = ((int)diff + (VSYNC_TIMER / 2)) / VSYNC_TIMER;

			switch (pol_state) {
			case POL_STATE_POS:
				pol_sum += diff_frame;
				pol_state = POL_STATE_NEG;
				if (pol_sum < 0) {
					biased = true;
				}
				break;

			case POL_STATE_NEG:
				pol_sum -= diff_frame;
				pol_state = POL_STATE_POS;
				if (pol_sum > 0) {
					biased = true;
				}
				break;

			default:
				;
			}

			pr_debug("POLARITY: pol_time:%14lld pol_sum:%6d pol_state:%s "
					 "hr_video_state=%d, vsync_pending=%d\n",
				ktime_to_ns(vsync_time),
				pol_sum,
				pol_state == POL_STATE_POS ? "POS" : "NEG",
				ctx->hr_video_state,
				vsync_pending);

			if (ctx->pol_first &&
				!ctx->mfr &&
				(ctx->hr_video_state == REPEAT_FRAME) &&
				(vsync_pending == 1)) {
				if (biased) {
					pr_debug("POLARITY: INSERT additional repeat frame\n");

					ctx->vsync_pending++;

					if (ctx->tg_state != HW_TG_ON) {
						ctx->tick_cnt--;
						mdss_mdp_timegen_enable(ctl, true);
					}
				}
				ctx->pol_first = false;
			}

			ctx->pol_sum = pol_sum;
		}

		ctx->pol_state = pol_state;
		ctx->pol_time = vsync_time;
	}
}
#endif /* CONFIG_SHDISP */

static void print_state(struct mdss_mdp_hr_video_ctx *ctx, char *func)
{
#ifdef PRINT_LOGS 
	static int prev_state = -1;
	static int prev_tog_pending = -1;
	if (prev_state != ctx->tg_state || prev_tog_pending !=
			ctx->tg_toggle_pending) {
		pr_debug("%s state=%d toggle_pend=%d \n", func,
				ctx->tg_state, ctx->tg_toggle_pending);
		prev_state = ctx->tg_state;
		prev_tog_pending = ctx->tg_toggle_pending;
	}
#endif

}

static inline void video_vsync_irq_enable(struct mdss_mdp_ctl *ctl, bool clear)
{
	struct mdss_mdp_hr_video_ctx *ctx = ctl->priv_data;

	mutex_lock(&ctx->vsync_mtx);
	if (atomic_inc_return(&ctx->vsync_ref) == 1)
		mdss_mdp_irq_enable(MDSS_MDP_IRQ_INTF_VSYNC, ctl->intf_num);
	else if (clear)
		mdss_mdp_irq_clear(ctl->mdata, MDSS_MDP_IRQ_INTF_VSYNC,
				ctl->intf_num);
	mutex_unlock(&ctx->vsync_mtx);
}

static inline void video_vsync_irq_disable(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_hr_video_ctx *ctx = ctl->priv_data;

	mutex_lock(&ctx->vsync_mtx);
	if (atomic_dec_return(&ctx->vsync_ref) == 0)
		mdss_mdp_irq_disable(MDSS_MDP_IRQ_INTF_VSYNC, ctl->intf_num);
	mutex_unlock(&ctx->vsync_mtx);
}

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
static int mdss_dsi_clk_lane(struct mdss_panel_data *pdata, int hs)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	u32 tmp;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	tmp = MIPI_INP((ctrl_pdata->ctrl_base) + 0xac);
	if (hs) {
		tmp |= (1 << 28);
	} else {
		tmp &= ~(1 << 28);
	}

	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0xac, tmp);
	wmb();

	return 0;
}

static void mdss_dsi_hs(struct mdss_panel_data *pdata)
{
	mdss_dsi_clk_lane(pdata, 1);
	return;
}

static void mdss_dsi_lp(struct mdss_panel_data *pdata)
{
	mdss_dsi_clk_lane(pdata, 0);
	return;
}

static void mdss_mdp_dsi_clk_ctl(struct mdss_mdp_hr_video_ctx *ctx, int onoff)
{
	pr_debug("Calling CLK_CTRL(%d)\n", onoff);
	mdss_mdp_ctl_intf_event(
			ctx->ctl, MDSS_EVENT_PANEL_CLK_CTRL, (void *)onoff);
	pr_debug("Done    CLK_CTRL(%d)\n", onoff);
	return;
}

static inline void mdss_mdp_hr_video_clk_on_sub(
		struct mdss_mdp_hr_video_ctx *ctx)
{
	int rc;
	pr_debug("Sending CLK ON %s %d\n",
			__func__, __LINE__);

	if (ctx->timegen_en) {
		rc = mdss_iommu_ctrl(1);
		if (IS_ERR_VALUE(rc))
			pr_err("IOMMU attach failed\n");
	}
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);
	mdss_mdp_dsi_clk_ctl(ctx, 1);
	ctx->clk_enabled = 1;
}
#endif /* CONFIG_SHDISP */

static inline void mdss_mdp_hr_video_clk_on(struct mdss_mdp_hr_video_ctx *ctx)
{
	/* Hook to enable POWER related call like DSI CLK/MDP Clocks */
	mutex_lock(&ctx->clk_mtx);
	if (!ctx->clk_enabled) {
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
		/* Send the DSI_ON EVENT */
		mdss_mdp_hr_video_clk_on_sub(ctx);
#else  /* CONFIG_SHDISP */
		ctx->clk_enabled = 1;
		pr_debug("Sending CLK ON %s %d\n",
				__func__, __LINE__);
		/* Send the DSI_ON EVENT */
#endif /* CONFIG_SHDISP */
	}
	mutex_unlock(&ctx->clk_mtx);
//	video_vsync_irq_enable(ctx->ctl, true);

}

static inline void mdss_mdp_hr_video_clk_off(struct mdss_mdp_hr_video_ctx *ctx)
{

	/* Hook to disable POWER related call like DSI CLK/MDP Clocks */
	mutex_lock(&ctx->clk_mtx);
	if (ctx->clk_enabled) {
		ctx->clk_enabled = 0;
		pr_debug("Sending CLK OFF %s %d \n",
				__func__, __LINE__);
		//Send DSI ON EVENT
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
		mdss_mdp_dsi_clk_ctl(ctx, 0);
		if (ctx->timegen_en)
			mdss_iommu_ctrl(0);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
#endif /* CONFIG_SHDISP */
	}
	mutex_unlock(&ctx->clk_mtx);
//	video_vsync_irq_disable(ctx->ctl);

}

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
static int schedule_work_ctrl(struct mdss_mdp_hr_video_ctx *ctx, int flag, struct work_struct *work)
{
	int ret = 0;
	if (flag == 0) {
		/* assume that ctx->hrtimer_lock is spin_locked */
		ctx->work_ctrl = 0;
		ctx->work_cnt = 0;
		if (ctx->work_need_notify) {
			ctx->work_need_notify = 0;
			complete_all(&ctx->work_comp);
		}
	} else if (work) {
		/* assume that ctx->hrtimer_lock is spin_locked */
		if (ctx->work_ctrl < 0) {
			pr_debug("queueing work:%d denied\n", flag);
		} else {
			schedule_work(work);
			flag |= (flag << 2);
			ctx->work_ctrl &= 3;
			ctx->work_ctrl |= flag;
		}
	} else {
		unsigned long flags;
		int need_wait = 0;
	retry:
		spin_lock_irqsave(&ctx->hrtimer_lock, flags);
		ret = ctx->work_ctrl;
		if (flag < 0 && ret < 0) {
			INIT_COMPLETION(ctx->work_comp);
			ctx->work_need_notify = 1;
			need_wait = 1;
		} else {
			if (flag > 3) {
				int tmp = (ret ^ flag) & (3 << 2);
				flag = ret;
				ret = tmp;
			} else if (flag > 0) {
				flag = ret & !flag;
			}
			ctx->work_ctrl = flag;
		}
		spin_unlock_irqrestore(&ctx->hrtimer_lock, flags);
		if (need_wait) {
			pr_debug("Wait until commit\n");
			wait_for_completion_timeout(&ctx->work_comp, usecs_to_jiffies(VSYNC_TIMEOUT_US));
			pr_debug("Done\n");
			need_wait = 0;
			goto retry;
		}
		if (flag < 0 && ret > 0) {
			if (ret & 1) {
				pr_debug("Cancelling CLK OFF work\n");
				cancel_work_sync(&ctx->clk_work);
				pr_debug("Done\n");
			}
			if (ret & 2) {
				pr_debug("Cancelling CLK ON work\n");
				cancel_work_sync(&ctx->hr_video_clk_work);
				pr_debug("Done\n");
			}
		}
	}
	return ret;
}
#endif /* CONFIG_SHDISP */

static void clk_ctrl_hr_video_work(struct work_struct *work)
{
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	struct mdss_mdp_hr_video_ctx *ctx = container_of(work, typeof(*ctx),
			hr_video_clk_work);
#else  /* CONFIG_SHDISP */
	struct delayed_work *dw = to_delayed_work(work);
	struct mdss_mdp_hr_video_ctx *ctx = container_of(dw, typeof(*ctx),
			hr_video_clk_work);
#endif /* CONFIG_SHDISP */
	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return;
	}
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
	if (schedule_work_ctrl(ctx, 2 << 2, NULL) != 0) {
		pr_debug("cancel clk on\n");
	} else
#endif /* CONFIG_SHDISP */
	mdss_mdp_hr_video_clk_on(ctx);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
	schedule_work_ctrl(ctx, 2, NULL);
#endif /* CONFIG_SHDISP */
}

static void clk_ctrl_work(struct work_struct *work)
{

	struct mdss_mdp_hr_video_ctx *ctx = container_of(work, typeof(*ctx),
			clk_work);
	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return;
	}

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
	if (schedule_work_ctrl(ctx, 1 << 2, NULL) != 0) {
		pr_debug("cancel clk off\n");
	} else
#endif /* CONFIG_SHDISP */
	mdss_mdp_hr_video_clk_off(ctx);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
	schedule_work_ctrl(ctx, 1, NULL);
#endif /* CONFIG_SHDISP */

}

static inline void mdp_video_write(struct mdss_mdp_hr_video_ctx *ctx,
				   u32 reg, u32 val)
{
	writel_relaxed(val, ctx->base + reg);
}

static inline u32 mdp_video_read(struct mdss_mdp_hr_video_ctx *ctx,
				   u32 reg)
{
	return readl_relaxed(ctx->base + reg);
}

static inline u32 mdss_mdp_hr_video_line_count(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_hr_video_ctx *ctx;
	u32 line_cnt = 0;
	if (!ctl || !ctl->priv_data)
		goto line_count_exit;
	ctx = ctl->priv_data;
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);
	line_cnt = mdp_video_read(ctx, MDSS_MDP_REG_INTF_LINE_COUNT);
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
line_count_exit:
	return line_cnt;
}

static int mdss_mdp_cancel_hrtimer(struct mdss_mdp_hr_video_ctx *ctx)
{
	int ret = 0;
	if (ctx->hrtimer_init) {
		pr_debug("Cancel timer  %s %d \n", __func__, __LINE__);
		ret = hrtimer_cancel(&ctx->vsync_hrtimer);
	}
	ctx->hrtimer_init = false;
	return 0;
}

static bool mdss_mdp_hr_video_start_timer(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_hr_video_ctx *ctx;

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	if (!ctx->hrtimer_init) {
		pr_debug("Hrtimer start %s %d \n", __func__, __LINE__);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-HRT) */
		ctx->hrt_hint = ktime_add_ns(ktime_get(), VSYNC_TIMER);
#endif /* CONFIG_SHDISP */
		hrtimer_start(&ctx->vsync_hrtimer, ns_to_ktime(VSYNC_TIMER), HRTIMER_MODE_REL);
	}
	ctx->hrtimer_init = true;

	return ctx->hrtimer_init;
}

static int mdss_mdp_timegen_enable(struct mdss_mdp_ctl *ctl, bool enable)
{
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
	struct mdss_mdp_hr_video_ctx *ctx;
	int state;
	char *clk = "";

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	state = ctx->tg_state;
	if (enable) {
		if (state != HW_TG_ON) {
			state = HW_TG_ON;
		}
	} else {
		if (state == HW_TG_ON) {
			state = SW_TG_OFF;
		} else if (state == SW_TG_OFF) {
			state = HW_TG_OFF;
		}
	}

	if (ctx->tg_state != state) {
		if (ctx->tg_state == HW_TG_OFF) {
			mdss_dsi_hs(ctx->ctl->panel_data);
			udelay(6);
			clk = "hs";
		}

		ctx->tg_state = state;

		if (state == HW_TG_OFF) {
			mdss_dsi_lp(ctx->ctl->panel_data);
			clk = "lp";
		} else {
			if (state == HW_TG_ON) {
				state = 1;
			} else {
				state = 0;
			}
			mdp_video_write(ctx, MDSS_MDP_REG_INTF_TIMING_ENGINE_EN, state);
			wmb();
		}
		pr_debug("TG=%d, tg_state=%d %s\n", enable, ctx->tg_state, clk);
	}

	return 0;
#else  /* CONFIG_SHDISP */
	struct mdss_mdp_hr_video_ctx *ctx;
	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	pr_debug(" Setting tg=%d %s %d \n", enable, __func__, __LINE__);
	if (enable == true) {
		mdp_video_write(ctx, MDSS_MDP_REG_INTF_TIMING_ENGINE_EN, 1);
	} else {
		mdp_video_write(ctx, MDSS_MDP_REG_INTF_TIMING_ENGINE_EN, 0);
	}
	wmb();
	return 0;

#endif /* CONFIG_SHDISP */
}

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) (MFR) */
static int mdss_mdp_hr_video_switch_timer(
		struct mdss_mdp_hr_video_ctx *ctx, bool enable)
{
	s64 tmpns;
	unsigned int tmp;
	unsigned int rem;
	unsigned int period = 0;
	unsigned int ticks;
	ktime_t now;

	if (ctx->work_cnt > 0 || ctx->tg_state != HW_TG_OFF) {
		/* Timer already works with 16.6ms period and clock will be on */
		if (ctx->hr_video_state == HR_VIDEO_MODE && enable) {
			ctx->hr_video_state = SOFT_HR_VIDEO_MODE;
			pr_debug("Going to SOFT_HR_VIDEO mode\n");
			if (ctx->work_cnt > 0) {
				ctx->tick_cnt = HR_VIDEO_REFRESH_CNT - 1;
			}
		} else if (ctx->hr_video_state == SOFT_HR_VIDEO_MODE && !enable) {
			ctx->hr_video_state = HR_VIDEO_MODE;
			pr_debug("Going to HR_VIDEO mode\n");
		}
		return 0;
	}

	now = ktime_get();
	tmpns = ktime_to_ns(ktime_sub(ctx->next, now));

	if (tmpns < 0) {
		tmpns += VSYNC_TIMER * HR_VIDEO_REFRESH_CNT;
	}

	if ((tmpns < 0) || (tmpns > (VSYNC_TIMER * HR_VIDEO_REFRESH_CNT))) {
		tmpns = VSYNC_TIMER * HR_VIDEO_REFRESH_CNT;
	}

	tmp = tmpns;

	if ((ctx->hr_video_state == HR_VIDEO_MODE) && enable) {
		tmp = (VSYNC_TIMER * HR_VIDEO_REFRESH_CNT) - tmp;
		ticks = tmp / VSYNC_TIMER;
		period = VSYNC_TIMER;
		rem = period - (tmp - (ticks * VSYNC_TIMER));
		tmp = SOFT_HR_VIDEO_MODE;
	} else if ((ctx->hr_video_state == SOFT_HR_VIDEO_MODE) && !enable) {
		period = tmp;
		rem = period;
		tmp = HR_VIDEO_MODE;
	}

	if (period) {
		if (hrtimer_try_to_cancel(&ctx->vsync_hrtimer) < 0) {
			pr_debug("Cannot cancel timer, retry\n");
			return -1;
		}

		ctx->hrt_hint = ktime_add_ns(now, rem);
		hrtimer_start(
				&ctx->vsync_hrtimer, ns_to_ktime(rem),
				HRTIMER_MODE_REL);

		ctx->hr_video_state = tmp;
		ctx->hrtimer_next_wait_ns = period;

		if (tmp == HR_VIDEO_MODE) {
			pr_debug("Going to HR_VIDEO mode, tick=%d, remain=%d us\n",
					ctx->tick_cnt, rem/1000);

			if (ctx->tick_cnt < HR_VIDEO_REFRESH_CNT - DSI_CLK_DELAY*2) {
				if (ctx->clk_enabled) {
					pr_debug("Turning DSI off\n");
					schedule_work_ctrl(ctx, 1, &ctx->clk_work);
				}
			}
			ctx->tick_cnt = 10000; /* dummy */
		} else {
			ctx->tick_cnt = ticks;
			pr_debug("Going to SOFT_HR_VIDEO, tick=%d, remain=%d us\n",
					ctx->tick_cnt, rem/1000);
			if (!ctx->clk_enabled) {
				pr_debug("Turning DSI on\n");
				schedule_work_ctrl(ctx, 2, &ctx->hr_video_clk_work);
			}
		}
	}

	return 0;
}

static int mdss_mdp_end_full_hr_video_mode(struct mdss_mdp_hr_video_ctx *ctx)
{
	if (ctx->hr_video_state == HR_VIDEO_MODE && ctx->tg_state == HW_TG_OFF) {
		return mdss_mdp_hr_video_switch_timer(ctx, true);
	}
	return 0;
}

static int hr_video_clk_ctrl(struct mdss_mdp_ctl *ctl, int onoff)
{
	struct mdss_mdp_hr_video_ctx *ctx = ctl->priv_data;
	unsigned long flags;
	int ret;
	
	mutex_lock(&ctx->clk_ctrl_mtx);
 retry:
	ret = 0;
	spin_lock_irqsave(&ctx->hrtimer_lock, flags);
	// pr_debug("begin: ret=%d, clk_cnt=%d\n", ret, ctx->clk_cnt);
	if (onoff & 1) {
		if (ctx->clk_cnt == 0) {
			ret = mdss_mdp_hr_video_switch_timer(ctx, true);
		}
		if (ret == 0) {
			ctx->clk_cnt++;
			if (onoff & 2) {
				atomic_inc(&ctx->vsync_handler_cnt);
			}
			ret = 1;
		}
	} else if (ctx->clk_cnt) {
		if (ctx->clk_cnt == 1) {
			ret = mdss_mdp_hr_video_switch_timer(ctx, false);
		}
		if (ret == 0) {
			ctx->clk_cnt--;
			if (onoff & 2) {
				atomic_dec(&ctx->vsync_handler_cnt);
			}
			ret = 2;
		}
	}
	// pr_debug("end ret=%d, clk_cnt=%d\n", ret, ctx->clk_cnt);
	spin_unlock_irqrestore(&ctx->hrtimer_lock, flags);
	if (ret < 0) {
		usleep(30);
		goto retry;
	}
	if (ret == 1) {
		cancel_work_sync(&ctx->clk_work);
		mdss_mdp_hr_video_clk_on(ctx);
	}
	mutex_unlock(&ctx->clk_ctrl_mtx);
	return ret;
}
#else  /* CONFIG_SHDISP */
static int mdss_mdp_hr_video_timer_reprogram(struct mdss_mdp_hr_video_ctx *ctx)
{
	int ret = 0;

	ret = hrtimer_try_to_cancel(&ctx->vsync_hrtimer);
	pr_debug("Reprogramming the timer to the Vsync Timer. %s %d \n", __func__, __LINE__);
	if (ret == -1) {
		/* Timer is still running the callback. Reprogram the timer to
		 * next vsync
		 */
	} else {
		/* Timer was cancelled, start the timer again.
		 */
		hrtimer_start(&ctx->vsync_hrtimer, ns_to_ktime(ctx->hrtimer_next_wait_ns), HRTIMER_MODE_REL);
	}
	return 0;
}

static void mdss_mdp_end_full_hr_video_mode(struct mdss_mdp_hr_video_ctx *ctx)
{
	u32 rem = ktime_to_ns(hrtimer_expires_remaining(&ctx->vsync_hrtimer));

	ctx->hrtimer_next_wait_ns = (u32)rem;

	if (rem > VSYNC_TIMER) {
		div_u64_rem(ktime_to_ns(hrtimer_expires_remaining(&ctx->vsync_hrtimer)), VSYNC_TIMER, &ctx->hrtimer_next_wait_ns);
		pr_debug(" new timer tick scheduled after %d %s %d \n", ctx->hrtimer_next_wait_ns,  __func__, __LINE__);
		mdss_mdp_hr_video_timer_reprogram(ctx);

		ctx->hrtimer_next_wait_ns = VSYNC_TIMER;

	} else  {
		ctx->hrtimer_next_wait_ns = VSYNC_TIMER;
	}
	ctx->tg_toggle_pending++;
	ctx->hr_video_state = NEW_UPDATE;

}
#endif /* CONFIG_SHDISP */

static int mdss_mdp_queue_commit(struct mdss_mdp_hr_video_ctx *ctx)
{
	unsigned long flags;
	enum mdss_mdp_hr_video_mode mode;
	enum mdss_mdp_hr_video_tg_state tstate;

	spin_lock_irqsave(&ctx->hrtimer_lock, flags);

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-HRT) */
	while (mdss_mdp_end_full_hr_video_mode(ctx)) {
		spin_unlock_irqrestore(&ctx->hrtimer_lock, flags);
		usleep(30);
		spin_lock_irqsave(&ctx->hrtimer_lock, flags);
	}
#endif /* CONFIG_SHDISP */

	mode = ctx->hr_video_state;
	tstate = ctx->tg_state;
	ctx->tick_cnt = 0;

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (POL) */
	mdss_mdp_hr_polarity_commit(ctx);
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-HRT) */
	if (1) {
#else  /* CONFIG_SHDISP */
	if (mode == HR_VIDEO_MODE) {
		mdss_mdp_end_full_hr_video_mode(ctx);
		spin_unlock_irqrestore(&ctx->hrtimer_lock, flags);
		return 0;
	} else if (mode == FIRST_UPDATE) {
		ctx->vsync_pending++;
		ctx->vsync_pending++;
		mdss_mdp_hr_video_start_timer(ctx->ctl);
		ctx->tg_state = HW_TG_ON;
		mdss_mdp_timegen_enable(ctx->ctl, true);
		ctx->last_hrtick_time = ktime_get();
	} else {
#endif /* CONFIG_SHDISP */
		switch (tstate) {
		case HW_TG_ON:
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
			ctx->tick_cnt = (u32)-1;
			ctx->vsync_pending = 2;
#else  /* CONFIG_SHDISP */
			ctx->vsync_pending++;
			ctx->hr_video_state = NEW_UPDATE;
#endif /* CONFIG_SHDISP */
			break;
		case SW_TG_OFF:
			if (mdp_video_read(ctx, MDSS_MDP_REG_INTF_LINE_COUNT) <
					(mdss_panel_get_vtotal(&ctx->ctl->panel_data->panel_info) - 1)) { 
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
				ctx->tick_cnt = (u32)-1;
				ctx->vsync_pending = 2;
#else  /* CONFIG_SHDISP */
				ctx->vsync_pending++;
#endif /* CONFIG_SHDISP */
				mdss_mdp_timegen_enable(ctx->ctl, true);
#ifndef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
				ctx->tg_state = HW_TG_ON;
#endif /* CONFIG_SHDISP */
			} else {
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
				ctx->tg_toggle_pending = 1;
#else  /* CONFIG_SHDISP */
				ctx->tg_toggle_pending++;
#endif /* CONFIG_SHDISP */
			}
#ifndef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
			ctx->hr_video_state = NEW_UPDATE;
#endif /* CONFIG_SHDISP */
			break;
		case HW_TG_OFF:
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
			ctx->tg_toggle_pending = 1;
#else  /* CONFIG_SHDISP */
			ctx->tg_toggle_pending++;
			ctx->hr_video_state = NEW_UPDATE;
#endif /* CONFIG_SHDISP */
			break;
		default:
			pr_err("%s %d: Timing Generator state invalid\n",
					__func__, __LINE__);
			BUG_ON(1);
			break;

		}
	}

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) (MFR) */
	if (ctx->mfr_req) {
		ctx->hr_video_state = HR_VIDEO_MFR;
		if (ctx->tg_toggle_pending == 0) {
			ctx->mfr_state = 0;
		}
	} else {
		ctx->hr_video_state = NEW_UPDATE;
	}
#else  /* CONFIG_SHDISP */
	ctx->hr_video_state = NEW_UPDATE;
#endif /* CONFIG_SHDISP */
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
	schedule_work_ctrl(ctx, 0, NULL);
#endif /* CONFIG_SHDISP */

	spin_unlock_irqrestore(&ctx->hrtimer_lock, flags);

	return 0;
}

static int mdss_mdp_push_one_frame(struct mdss_mdp_hr_video_ctx *ctx)
{
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	if (ctx->tg_state != HW_TG_ON) {
		mdss_mdp_timegen_enable(ctx->ctl, true);
	}
	ctx->vsync_pending = 2;
	return 0;
#else  /* CONFIG_SHDISP */
	enum mdss_mdp_hr_video_tg_state tstate;
	tstate = ctx->tg_state;
	switch (tstate) {
		case HW_TG_ON:
			ctx->last_hrtick_time = ktime_get();
			ctx->vsync_pending++;
			break;
		case  SW_TG_OFF:
			if (mdp_video_read(ctx, MDSS_MDP_REG_INTF_LINE_COUNT) <
					(mdss_panel_get_vtotal(&ctx->ctl->panel_data->panel_info) - 1)) { 
				ctx->vsync_pending++;
				ctx->tg_state = HW_TG_ON;
				mdss_mdp_timegen_enable(ctx->ctl, true);
			} else {

				mdss_dsi_state_reset(ctx->ctl->panel_data);
				ctx->vsync_pending++;
				ctx->vsync_pending++;
				ctx->tg_state = HW_TG_ON;
				mdss_mdp_timegen_enable(ctx->ctl, true);
			}
			break;
		case HW_TG_OFF:
			mdss_dsi_state_reset(ctx->ctl->panel_data);
			ctx->vsync_pending++;
			ctx->vsync_pending++;
			ctx->tg_state = HW_TG_ON;
			mdss_mdp_timegen_enable(ctx->ctl, true);
			break;
		default:
			break;
	}
	return 0;

#endif /* CONFIG_SHDISP */
}

static void mdss_mdp_start_full_hr_video_mode(struct mdss_mdp_hr_video_ctx *ctx)
{
	ctx->hr_video_state = HR_VIDEO_MODE;
	pr_debug("Going to hr_video mode %s %d \n", __func__, __LINE__);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	ctx->tick_cnt = 10000; /* dummy */
	ctx->hrtimer_next_wait_ns = ktime_to_ns(ktime_sub(ctx->next, ctx->curr));
#else  /* CONFIG_SHDISP */
	schedule_delayed_work(&ctx->hr_video_clk_work,
			SCHEDULE_DSI_CLK_ON_TIME);
	ctx->hrtimer_next_wait_ns = VSYNC_TIMER * HR_VIDEO_REFRESH_CNT;

#endif /* CONFIG_SHDISP */
}

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
static int mdss_mdp_toggle_timegen(struct mdss_mdp_ctl *ctl, int isr)
#else  /* CONFIG_SHDISP */
static int mdss_mdp_toggle_timegen(struct mdss_mdp_ctl *ctl)
#endif /* CONFIG_SHDISP */
{

	struct mdss_mdp_hr_video_ctx *ctx;
	enum mdss_mdp_hr_video_tg_state tstate;
	enum mdss_mdp_hr_video_mode mode;
	int tmp_cnt;

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (MFR) */
	if (ctx->mfr != ctx->mfr_req) {
		ctx->mfr = ctx->mfr_req;
		ctx->hrtimer_next_wait_ns = VSYNC_TIMER;
		ctx->mfr_state = 0;
		complete_all(&ctx->mfr_comp);
	}

	if (ctx->mfr) {
		if (ctx->mfr == 1) {
			mdss_mdp_push_one_frame(ctx);
			ctx->tg_toggle_pending = 0;
		} else {
			if (isr == 0 && ctx->tg_toggle_pending) {
				ctx->tg_toggle_pending = 0;
				ctx->mfr_state = 0;
			}

			if (ctx->mfr_state == 0) {
				ctx->mfr_state = 1;
				if (!isr) {
					mdss_mdp_push_one_frame(ctx);
				}
			} else {
				ctx->mfr_state++;
				if (ctx->mfr_state == ctx->mfr) {
					ctx->mfr_state = 0;
				}
			}
		}
		return 0;
	}
#endif /* CONFIG_SHDISP */

	pr_debug("enabling timing gen for intf=%d\n", ctl->intf_num);
	tstate = ctx->tg_state;
	mode = ctx->hr_video_state;

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
	if (mode == HR_VIDEO_MODE || mode == SOFT_HR_VIDEO_MODE) {
		if (mode == HR_VIDEO_MODE) {
			if (ctx->tick_cnt == 10000) {
				ctx->work_cnt = -1;
			}
		} else if (ctx->tick_cnt + 1 == HR_VIDEO_REFRESH_CNT - DSI_CLK_DELAY) {
			ctx->work_cnt = -1;
		}
		if (ctx->work_cnt) {
			if (ctx->work_cnt < 0) {
				schedule_work_ctrl(ctx, 2, &ctx->hr_video_clk_work);
				ctx->hrtimer_next_wait_ns = VSYNC_TIMER;
				ctx->work_cnt = DSI_CLK_DELAY;
				ctx->tick_cnt++;
			} else if (ctx->work_cnt > 1) {
				ctx->work_cnt--;
				ctx->tick_cnt++;
			} else  if (ctx->clk_enabled) {
				ctx->work_cnt = 0;
			}
			if (ctx->work_cnt) {
				pr_debug("wait until clk_on(%d)\n", ctx->work_cnt);
				return 0;
			}
		}
	}

	if (isr && ctx->tg_toggle_pending && tstate == HW_TG_OFF) {
		pr_debug("wait next hrt\n");
		return 0;
	}
#endif /* CONFIG_SHDISP */

	ctx->tick_cnt++;
	tmp_cnt = ctx->tick_cnt;

	if (ctx->tg_toggle_pending > 0) {
			mdss_mdp_push_one_frame(ctx);
#ifndef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
			ctx->last_hrtick_time = ktime_get();
#endif /* CONFIG_SHDISP */
			ctx->tick_cnt = 0;
			ctx->tg_toggle_pending--;
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
			if (ctx->hr_video_state == NEW_UPDATE) {
				pr_debug("Sending UPDATE frame\n");
			}
#endif /* CONFIG_SHDISP */

	} else if (mode == SOFT_HR_VIDEO_MODE) {
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
		if (isr) {
			pr_debug(">>> hr_video SOFT REFRESH end <<<\n");
			ctx->next = ktime_add_ns(
					ctx->curr, VSYNC_TIMER*(HR_VIDEO_REFRESH_CNT - 1));
		}
#else
		pr_debug(">>>>SOFT REFRESH<<<<\n");
#endif /* CONFIG_SHDISP */
		if (tmp_cnt == HR_VIDEO_REFRESH_CNT) {
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
			if (!ctx->clk_enabled) {
				pr_debug("wait next vsync\n");
				ctx->tick_cnt--;
				return 0;
			}
			pr_debug(">>> hr_video SOFT REFRESH start <<<\n");
#endif /* CONFIG_SHDISP */
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
			mdss_mdp_push_one_frame(ctx);
			ctx->tick_cnt = 0;
#else  /* CONFIG_SHDISP */
			if (atomic_read(&ctx->vsync_handler_cnt) == 0) {
				/* Vsync disable was called, lets go to full hr_video */
				mdss_mdp_start_full_hr_video_mode(ctx);
				mdss_mdp_push_one_frame(ctx);
			} else {
				/* Update every 60ticks. */
				ctx->hrtimer_next_wait_ns = VSYNC_TIMER;
				mdss_mdp_push_one_frame(ctx);
				ctx->tick_cnt = 0;
			}
#endif /* CONFIG_SHDISP */
		}
	} else if (mode == HR_VIDEO_MODE) {
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
		if (isr) {
			pr_debug(">>> hr_video REFRESH end <<<\n");
			ctx->next = ktime_add_ns(
					ctx->next, VSYNC_TIMER * HR_VIDEO_REFRESH_CNT);
			mdss_mdp_start_full_hr_video_mode(ctx);
			schedule_work_ctrl(ctx, 1, &ctx->clk_work);
		} else {
			pr_debug(">>> hr_video REFRESH start <<<\n");
			mdss_mdp_push_one_frame(ctx);
		}
#else  /* CONFIG_SHDISP */
		mdss_mdp_push_one_frame(ctx);
		mdss_mdp_start_full_hr_video_mode(ctx);
		pr_debug(">>>>hr_video REFRESH<<<<\n");
#endif /* CONFIG_SHDISP */

	} else if (mode == NEW_UPDATE && ctx->vsync_pending > 1) {
		ctx->tick_cnt = 0;
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	} else if (mode == NEW_UPDATE && ctx->vsync_pending == 1) {
		pr_debug("Sending UPDATE frame (continuous)\n");
#endif /* CONFIG_SHDISP */
	} else if (tmp_cnt >= INACTIVITY_CNT + 1  &&
			tmp_cnt <INACTIVITY_CNT + BLANK_NUM + 1) {
		/* Blank frames. Do nothing */
		ATRACE_BEGIN("BLANK");
		pr_debug("Sending %d BLANK frames %s %d \n", BLANK_NUM, __func__,
				__LINE__);
		ctx->hr_video_state = BLANK_FRAME;
		ATRACE_END("BLANK");

	} else if (tmp_cnt == (INACTIVITY_CNT + BLANK_NUM + 1 )) {

		pr_debug("Sending %d REPEAT Frames %s %d \n", REPEAT_CNT, __func__,
				__LINE__);
		ATRACE_BEGIN("REPEAT");
		if (REPEAT_CNT > 0) {
			ctx->hr_video_state = REPEAT_FRAME;
			mdss_mdp_push_one_frame(ctx);
			ctx->vsync_pending += (REPEAT_CNT - 1);
#ifndef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
			ctx->last_hrtick_time = ktime_get();
#endif /* CONFIG_SHDISP */
		}
		ATRACE_END("REPEAT");
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	} else if ((tmp_cnt > INACTIVITY_CNT + BLANK_NUM + 1) &&
	           (tmp_cnt < INACTIVITY_CNT + BLANK_NUM + REPEAT_CNT + 1)) {
		pr_debug("Sending %d REPEAT Frames %s %d \n",
				REPEAT_CNT, __func__, __LINE__);
	} else if (tmp_cnt == INACTIVITY_CNT + BLANK_NUM + REPEAT_CNT + 1) {
		pr_debug("Sending %d WAIT Frame\n", WAIT_CNT);
	} else if ((tmp_cnt > INACTIVITY_CNT + BLANK_NUM + REPEAT_CNT + 1) &&
		   (tmp_cnt < INACTIVITY_CNT + BLANK_NUM + REPEAT_CNT + WAIT_CNT + 1)) {
		pr_debug("Sending %d WAIT Frame\n", WAIT_CNT);
	} else if (tmp_cnt == INACTIVITY_CNT + BLANK_NUM + REPEAT_CNT + WAIT_CNT + 1) {
		ctx->next = ktime_add_ns(
				ctx->curr, (VSYNC_TIMER * HR_VIDEO_REFRESH_CNT) - VSYNC_TIMER*(DSI_CLK_DELAY + WAIT_CNT + 1));
		ctx->tick_cnt = WAIT_CNT + 1;
#else  /* CONFIG_SHDISP */
	} else if (tmp_cnt == INACTIVITY_CNT + BLANK_NUM + REPEAT_CNT) {
#endif /* CONFIG_SHDISP */
		if (atomic_read(&ctx->vsync_handler_cnt) == 0) {
			mdss_mdp_start_full_hr_video_mode(ctx);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
			schedule_work_ctrl(ctx, 1, &ctx->clk_work);
#endif /* CONFIG_SHDISP */
		} else {
			pr_debug("Going to SOFT hr_video %s %d\n", __func__,
					__LINE__);
			ctx->hr_video_state = SOFT_HR_VIDEO_MODE;
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
			ctx->work_cnt = 0;
#else  /* CONFIG_SHDISP */
			ctx->hrtimer_next_wait_ns = VSYNC_TIMER;
#endif /* CONFIG_SHDISP */
		}
	}

	return 0;

}

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
static void vsync_common(struct mdss_mdp_hr_video_ctx *ctx, int isr)
{
	ktime_t curr = ktime_get();
	struct mdss_mdp_vsync_handler *tmp;
	
	ctx->curr = curr;
	pr_debug("isr=%d handler=%d\n", isr, atomic_read(&ctx->vsync_handler_cnt));
	if (atomic_read(&ctx->vsync_handler_cnt) > 0) {
		spin_lock(&ctx->vsync_lock);
		list_for_each_entry(tmp, &ctx->vsync_handlers, list) {
			tmp->vsync_handler(ctx->ctl, curr);
		}
		spin_unlock(&ctx->vsync_lock);
	}
}

static enum hrtimer_restart hrt_vsync_cb(struct hrtimer *vsync_timer)
{
	unsigned long flags;
	int res, ret = HRTIMER_NORESTART;
	struct mdss_mdp_hr_video_ctx *ctx =
			container_of(vsync_timer, typeof(*ctx), vsync_hrtimer);

	ATRACE_BEGIN("hrtick");
	if (!ctx) {
		pr_err("invalid ctx \n");
		goto exit_cb;
	}

	spin_lock_irqsave(&ctx->hrtimer_lock, flags);
	pr_debug("ENTER\n");

#ifdef CONFIG_SHDISP /* CUST_ID_00037 */
	if (ctx->suspend) {
		if (ctx->suspend == RES_START) {
			ctx->suspend = SUS_IDLE;
			if (ctx->mfr_req) {
				ctx->hr_video_state = HR_VIDEO_MFR;
			} else {
				ctx->hr_video_state = NEW_UPDATE;
			}
			ctx->tg_toggle_pending = 1;
			complete_all(&ctx->suspend_comp);
		} else {
			if (ctx->suspend == SUS_START) {
				ctx->suspend = SUS_DONE;
				complete_all(&ctx->suspend_comp);
				if (ctx->suspend_tg) {
					mdss_mdp_timegen_enable(ctx->ctl, true);
				}
			}
			if (ctx->suspend_tg == 0) {
				vsync_common(ctx, 0);
				complete_all(&ctx->vsync_comp);
			}
			ret = HRTIMER_RESTART;
			res = VSYNC_TIMER;
			goto exit_cb_1;
		}
	}
#endif /* CONFIG_SHDISP */

	if (ctx->hr_video_state == POWER_OFF) {
		goto exit_cb_2;
	}

	vsync_common(ctx, 0);
	mdss_mdp_toggle_timegen(ctx->ctl, 0);

exit_cb_2:
	if (ctx->tg_state == HW_TG_ON) {
		ctx->vsync_skip = 1;
	} else if (ctx->tg_state == HW_TG_OFF) {
		res = ctx->hrtimer_next_wait_ns;
		ret = HRTIMER_RESTART;
	}

#ifdef CONFIG_SHDISP /* CUST_ID_00037 */
exit_cb_1:
#endif /* CONFIG_SHDISP */

	if (ret == HRTIMER_RESTART) {
		ctx->hrt_hint = ktime_add_ns(ctx->hrt_hint, res);
		hrtimer_forward_now(&ctx->vsync_hrtimer, ns_to_ktime(res));
		pr_debug("LEAVE %d:RESTART %d\n", ctx->vsync_skip, res);
	} else {
		pr_debug("LEAVE %d:NORESTART\n", ctx->vsync_skip);
	}

	spin_unlock_irqrestore(&ctx->hrtimer_lock, flags);

exit_cb:
	ATRACE_END("hrtick");
	return ret;
}
#else  /* CONFIG_SHDISP */
static int mod_hrtimer(struct hrtimer *vsync_timer, u32 duration)
{
	return hrtimer_forward_now(vsync_timer, ns_to_ktime(duration));
}

static enum hrtimer_restart hrt_vsync_cb(struct hrtimer *vsync_timer)
{

	int ret = HRTIMER_NORESTART;
	struct mdss_mdp_vsync_handler *tmp;
	unsigned long flags;
	ktime_t curr = ktime_get();
	char args[128];
	struct mdss_mdp_hr_video_ctx *ctx = container_of(vsync_timer, typeof(*ctx), vsync_hrtimer);

	if (!ctx) {
		pr_err("invalid ctx \n");
		ret = HRTIMER_NORESTART;
	}
	ATRACE_BEGIN("hrtick");

	if (atomic_read(&ctx->vsync_handler_cnt) > 0) {
		spin_lock(&ctx->vsync_lock);
		list_for_each_entry(tmp, &ctx->vsync_handlers, list) {
			tmp->vsync_handler(ctx->ctl, curr);
		}
		spin_unlock(&ctx->vsync_lock);
	sprintf(args, "vsync_pending=%d; state=%d;",
			ctx->vsync_pending, ctx->tg_state);
	}


	spin_lock_irqsave(&ctx->hrtimer_lock, flags);
	mdss_mdp_toggle_timegen(ctx->ctl);

	mod_hrtimer(&ctx->vsync_hrtimer, ctx->hrtimer_next_wait_ns);
	spin_unlock_irqrestore(&ctx->hrtimer_lock, flags);

	ret = HRTIMER_RESTART;
	ATRACE_END("hrtick");

	return ret;
}
#endif /* CONFIG_SHDISP */

static int mdss_mdp_register_hrtimer(struct mdss_mdp_hr_video_ctx *ctx)
{
	hrtimer_init(&ctx->vsync_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ctx->vsync_hrtimer.function = hrt_vsync_cb;
	pr_debug("Register the callback for the hrtimer %s %d \n", __func__, __LINE__);
	return 0;
}

int mdss_mdp_hr_video_addr_setup(struct mdss_data_type *mdata,
				u32 *offsets,  u32 count)
{
	struct mdss_mdp_hr_video_ctx *head;
	u32 i;

	head = devm_kzalloc(&mdata->pdev->dev,
			sizeof(struct mdss_mdp_hr_video_ctx) * count, GFP_KERNEL);
	if (!head)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
		spin_lock_init(&head[i].fps_led.lock);
		head[i].base = mdata->mdp_base + offsets[i];
#else  /* CONFIG_SHDISP */
		head[i].base = mdata->mdss_base + offsets[i];
#endif /* CONFIG_SHDISP */
		pr_debug("adding Video Intf #%d offset=0x%x virt=%p\n", i,
				offsets[i], head[i].base);
		head[i].ref_cnt = 0;
		head[i].intf_num = i + MDSS_MDP_INTF0;
		INIT_LIST_HEAD(&head[i].vsync_handlers);
	}

	mdata->video_intf = head;
	mdata->nintf = count;
	return 0;
}

static int mdss_mdp_hr_video_timegen_setup(struct mdss_mdp_ctl *ctl,
					struct intf_timing_params *p)
{
	u32 hsync_period, vsync_period;
	u32 hsync_start_x, hsync_end_x, display_v_start, display_v_end;
	u32 active_h_start, active_h_end, active_v_start, active_v_end;
	u32 den_polarity, hsync_polarity, vsync_polarity;
	u32 display_hctl, active_hctl, hsync_ctl, polarity_ctl;
	struct mdss_mdp_hr_video_ctx *ctx;

	ctx = ctl->priv_data;
	hsync_period = p->hsync_pulse_width + p->h_back_porch +
			p->width + p->h_front_porch;
	vsync_period = p->vsync_pulse_width + p->v_back_porch +
			p->height + p->v_front_porch;

	display_v_start = ((p->vsync_pulse_width + p->v_back_porch) *
			hsync_period) + p->hsync_skew;
	display_v_end = ((vsync_period - p->v_front_porch) * hsync_period) +
			p->hsync_skew - 1;

	if (ctx->intf_type == MDSS_INTF_EDP) {
		display_v_start += p->hsync_pulse_width + p->h_back_porch;
		display_v_end -= p->h_front_porch;
	}

	hsync_start_x = p->h_back_porch + p->hsync_pulse_width;
	hsync_end_x = hsync_period - p->h_front_porch - 1;

	if (p->width != p->xres) {
		active_h_start = hsync_start_x;
		active_h_end = active_h_start + p->xres - 1;
	} else {
		active_h_start = 0;
		active_h_end = 0;
	}

	if (p->height != p->yres) {
		active_v_start = display_v_start;
		active_v_end = active_v_start + (p->yres * hsync_period) - 1;
	} else {
		active_v_start = 0;
		active_v_end = 0;
	}


	if (active_h_end) {
		active_hctl = (active_h_end << 16) | active_h_start;
		active_hctl |= BIT(31);	/* ACTIVE_H_ENABLE */
	} else {
		active_hctl = 0;
	}

	if (active_v_end)
		active_v_start |= BIT(31); /* ACTIVE_V_ENABLE */

	hsync_ctl = (hsync_period << 16) | p->hsync_pulse_width;
	display_hctl = (hsync_end_x << 16) | hsync_start_x;

	den_polarity = 0;
	if (MDSS_INTF_HDMI == ctx->intf_type) {
		hsync_polarity = p->yres >= 720 ? 0 : 1;
		vsync_polarity = p->yres >= 720 ? 0 : 1;
	} else {
		hsync_polarity = 0;
		vsync_polarity = 0;
	}
	polarity_ctl = (den_polarity << 2)   | /*  DEN Polarity  */
		       (vsync_polarity << 1) | /* VSYNC Polarity */
		       (hsync_polarity << 0);  /* HSYNC Polarity */

	mdp_video_write(ctx, MDSS_MDP_REG_INTF_HSYNC_CTL, hsync_ctl);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_VSYNC_PERIOD_F0,
			vsync_period * hsync_period);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_VSYNC_PULSE_WIDTH_F0,
			   p->vsync_pulse_width * hsync_period);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_DISPLAY_HCTL, display_hctl);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_DISPLAY_V_START_F0,
			   display_v_start);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_DISPLAY_V_END_F0, display_v_end);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_ACTIVE_HCTL, active_hctl);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_ACTIVE_V_START_F0,
			   active_v_start);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_ACTIVE_V_END_F0, active_v_end);

	mdp_video_write(ctx, MDSS_MDP_REG_INTF_BORDER_COLOR, p->border_clr);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_UNDERFLOW_COLOR,
			   p->underflow_clr);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_HSYNC_SKEW, p->hsync_skew);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_POLARITY_CTL, polarity_ctl);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_FRAME_LINE_COUNT_EN, 0x3);

	return 0;
}


static int mdss_mdp_hr_video_add_hrt_vsync_handler(struct mdss_mdp_ctl *ctl,
		struct mdss_mdp_vsync_handler *handle)
{
	struct mdss_mdp_hr_video_ctx *ctx;
	unsigned long flags;
	int ret = 0;
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	bool switch_hrt = false;
#endif /* CONFIG_SHDISP */

	if (!handle || !(handle->vsync_handler)) {
		ret = -EINVAL;
		goto exit;
	}

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx for ctl=%d\n", ctl->num);
		ret = -ENODEV;
		goto exit;
	}
#ifndef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	atomic_inc(&ctx->vsync_handler_cnt);
#endif /* CONFIG_SHDISP */

	MDSS_XLOG(ctl->num, ctl->vsync_cnt, handle->enabled);

	spin_lock_irqsave(&ctx->vsync_lock, flags);
	if (!handle->enabled) {
		handle->enabled = true;
		list_add(&handle->list, &ctx->vsync_handlers);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
		atomic_inc(&ctx->vsync_handler_cnt);
		switch_hrt = true;
#endif /* CONFIG_SHDISP */
	}
	spin_unlock_irqrestore(&ctx->vsync_lock, flags);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	if (switch_hrt) {
		hr_video_clk_ctrl(ctl, 1);
	}
#endif /* CONFIG_SHDISP */
exit:
	return ret;
}

static int mdss_mdp_hr_video_remove_hrt_vsync_handler(struct mdss_mdp_ctl *ctl,
		struct mdss_mdp_vsync_handler *handle)
{
	struct mdss_mdp_hr_video_ctx *ctx;
	unsigned long flags;
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	bool switch_hrt = false;
#endif /* CONFIG_SHDISP */

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx for ctl=%d\n", ctl->num);
		return -ENODEV;
	}

	MDSS_XLOG(ctl->num, ctl->vsync_cnt, handle->enabled);
#ifndef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	atomic_dec(&ctx->vsync_handler_cnt);
#endif /* CONFIG_SHDISP */

	spin_lock_irqsave(&ctx->vsync_lock, flags);
	if (handle->enabled) {
		handle->enabled = false;
		list_del_init(&handle->list);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
		atomic_dec(&ctx->vsync_handler_cnt);
		switch_hrt = true;
#endif /* CONFIG_SHDISP */
	}
	spin_unlock_irqrestore(&ctx->vsync_lock, flags);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	if (switch_hrt) {
		hr_video_clk_ctrl(ctl, 0);
	}
#endif /* CONFIG_SHDISP */

	return 0;
}

static int mdss_mdp_hr_video_stop(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_hr_video_ctx *ctx;
	struct mdss_mdp_vsync_handler *tmp, *handle;
	struct mdss_mdp_ctl *sctl;
	unsigned long flags;
	int rc;
	u32 frame_rate = 0;

	pr_debug("stop ctl=%d\n", ctl->num);

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx for ctl=%d\n", ctl->num);
		return -ENODEV;
	}
	MDSS_XLOG(ctl->num, ctl->vsync_cnt);
	list_for_each_entry_safe(handle, tmp, &ctx->vsync_handlers, list)
		mdss_mdp_hr_video_remove_hrt_vsync_handler(ctl, handle);

	mdss_mdp_cancel_hrtimer(ctx);

	spin_lock_irqsave(&ctx->hrtimer_lock, flags);
	ctx->hr_video_state = POWER_OFF;
	ctx->tg_state = HW_TG_OFF;
	ctx->vsync_pending = 0;
	ctx->tg_toggle_pending = 0;

	spin_unlock_irqrestore(&ctx->hrtimer_lock, flags);

	video_vsync_irq_disable(ctl);

	if (cancel_work_sync(&ctx->clk_work)) {
		pr_debug("Cancelling clk_ok %s %d\n", __func__,
				__LINE__);

	}
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	if (cancel_work_sync(&ctx->hr_video_clk_work)) {
		pr_err("Cancelling hr_video_clk on%s %d \n", __func__, __LINE__);
	}
#else  /* CONFIG_SHDISP */
	if (cancel_delayed_work_sync(&ctx->hr_video_clk_work)) {
		pr_debug("Cancelling scheduled clk on%s %d \n", __func__,
				__LINE__);
	}
#endif /* CONFIG_SHDISP */

	if (ctx->power_on) {
		ctx->power_on = false;
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
		mdss_mdp_hr_video_clk_on(ctx);
#endif /* CONFIG_SHDISP */
		mdp_video_write(ctx, MDSS_MDP_REG_INTF_TIMING_ENGINE_EN, 1);
		rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_BLANK, NULL);
		if (rc == -EBUSY) {
			pr_debug("intf #%d busy don't turn off\n",
					ctl->intf_num);
			return rc;
		}
		WARN(rc, "intf %d blank error (%d)\n", ctl->intf_num, rc);

		mdp_video_write(ctx, MDSS_MDP_REG_INTF_TIMING_ENGINE_EN, 0);
		/* wait for at least one VSYNC on HDMI intf for proper TG OFF */
		if (MDSS_INTF_HDMI == ctx->intf_type) {
			frame_rate = mdss_panel_get_framerate
				(&(ctl->panel_data->panel_info));
			if (!(frame_rate >= 24 && frame_rate <= 240))
				frame_rate = 24;
			msleep((1000/frame_rate) + 1);
		}
		mdss_iommu_ctrl(0);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
		ctx->timegen_en = false;

		rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_OFF, NULL);
		WARN(rc, "intf %d timegen off error (%d)\n", ctl->intf_num, rc);

		mdss_mdp_irq_disable(MDSS_MDP_IRQ_INTF_UNDER_RUN,
				ctl->intf_num);
		sctl = mdss_mdp_get_split_ctl(ctl);
		if (sctl)
			mdss_mdp_irq_disable(MDSS_MDP_IRQ_INTF_UNDER_RUN,
					sctl->intf_num);
	}

	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_INTF_VSYNC, ctl->intf_num,
			NULL, NULL);
	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_INTF_UNDER_RUN, ctl->intf_num,
			NULL, NULL);

	mdss_mdp_ctl_reset(ctl);
	ctx->ref_cnt--;
	ctl->priv_data = NULL;

	return 0;
}

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-HRT) */
static int mdss_mdp_hr_video_update_hrt(struct mdss_mdp_hr_video_ctx *ctx)
{
	int hrt = 0, tstate = ctx->tg_state;
	
	if (ctx->mfr == 1 && ctx->suspend == SUS_IDLE) {
		tstate = SW_TG_OFF;
	}
	if (tstate == HW_TG_ON) {
		ctx->hrt_hint = ktime_add_ns(ctx->hrt_hint, VSYNC_TIMER);
	} else {
		int diff;
		if (tstate == SW_TG_OFF) {
			diff = ktime_to_ns(ktime_sub(ctx->curr, ctx->hrt_hint));
			if (diff > 0) {
				if (diff < VSYNC_TIMER/2) {
					diff = 0;
				}
			}
			ctx->hrt_hint = ktime_add_ns(ctx->hrt_hint, diff + VSYNC_TIMER);
		} else {
			ktime_t now = ktime_get();
			hrt = ctx->hrtimer_next_wait_ns;
			diff = ktime_to_ns(ktime_sub(now, ctx->hrt_hint));
			ctx->hrt_hint = ktime_add_ns(ctx->hrt_hint, hrt);
			if (diff > 0) {
				hrt -= diff;
				if (hrt < VSYNC_TIMER/2) {
					hrt = ctx->hrtimer_next_wait_ns;
					ctx->hrt_hint = ktime_add_ns(now, hrt);
				}
			}
			hrtimer_start(&ctx->vsync_hrtimer, ns_to_ktime(hrt), HRTIMER_MODE_REL);
		}
		pr_debug("HRT delay(%d) = %5d us\n", ctx->tg_state, diff/1000);
	}
	return hrt;
}
#endif /* CONFIG_SHDISP */

void mdss_mdp_hr_video_vsync_intr_done(void *arg)
{
	struct mdss_mdp_ctl *ctl = arg;
	struct mdss_mdp_hr_video_ctx *ctx = ctl->priv_data;
	ktime_t vsync_time;
	s32 cnt;
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	int skipped, hrt = 0;
#endif /* CONFIG_SHDISP */

	if (!ctx) {
		pr_err("invalid ctx\n");
		return;
	}

	ATRACE_BEGIN("vsync");

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	pr_debug("ENTER\n");
	spin_lock(&ctx->hrtimer_lock);
	skipped = ctx->vsync_skip;
	spin_unlock(&ctx->hrtimer_lock);
	if (skipped) {
		ctx->vsync_skip = 0;
	} else {
		vsync_common(ctx, 1);
	}
	vsync_time = ctx->curr;
#else  /* CONFIG_SHDISP */
	vsync_time = ktime_get();
#endif /* CONFIG_SHDISP */
	ctl->vsync_cnt++;

	MDSS_XLOG(ctl->num, ctl->vsync_cnt, ctl->vsync_cnt);

	pr_debug("intr ctl=%d vsync cnt=%u vsync_time=%d\n",
		 ctl->num, ctl->vsync_cnt, (int)ktime_to_ms(vsync_time));

	ctx->polling_en = false;
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	if ((mdss_mdp_ctl_read(ctl, MDSS_MDP_REG_CTL_FLUSH) & BIT(17)) == 0) {
		complete_all(&ctx->vsync_comp);
	}
#else  /* CONFIG_SHDISP */
	complete_all(&ctx->vsync_comp);
#endif /* CONFIG_SHDISP */
	spin_lock(&ctx->hrtimer_lock);

#ifdef CONFIG_SHDISP /* CUST_ID_00037 */
	if (ctx->suspend) {
		if (ctx->suspend == SUS_START || ctx->suspend == RES_WAIT) {
			mdss_mdp_timegen_enable(ctl, false);
			if (ctx->tg_state == HW_TG_OFF) {
				mdss_dsi_state_reset(ctx->ctl->panel_data);
				if (ctx->suspend == RES_WAIT) {
					ctx->suspend = RES_START;
				} else if (ctx->suspend_tg == 0) {
					ctx->suspend = SUS_DONE;
					complete_all(&ctx->suspend_comp);
				}
			}
			if (ctx->suspend == SUS_START || ctx->suspend == SUS_DONE) {
				hrt = mdss_mdp_hr_video_update_hrt(ctx);
			}
		}
		goto exit_intr_done_0;
	}
#endif /* CONFIG_SHDISP */

	print_state(ctx, "mdss_mdp_hr_video_vsync_intr_done");

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	if (ctx->hr_video_state == POWER_OFF) {
		goto exit_intr_done_0;
	}
#endif

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (MFR) */
	if (ctx->mfr == 1 && ctx->mfr == ctx->mfr_req) {
		pr_debug("MFR:skip\n");
		goto exit_intr_done_1;
	}
#endif /* CONFIG_SHDISP */

	ctx->vsync_pending--;
	cnt = ctx->vsync_pending;
	if (cnt > 1) {
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
		mdss_mdp_timegen_enable(ctl, true);
#else  /* CONFIG_SHDISP */
		ctx->tg_state = HW_TG_ON;
#endif /* CONFIG_SHDISP */
	} else if (cnt == 1) {
		/* No new updates came in stop the timegen */
		mdss_mdp_timegen_enable(ctl, false);
#ifndef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
		ctx->tg_state = SW_TG_OFF;
#endif /* CONFIG_SHDISP */

	} else if (cnt == 0) {
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
		mdss_mdp_timegen_enable(ctl, false);
#else  /* CONFIG_SHDISP */
		ctx->tg_state = HW_TG_OFF;
#endif /* CONFIG_SHDISP */
#ifndef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
		if (ctx->hr_video_state == HR_VIDEO_MODE) {
			schedule_work(&ctx->clk_work);
		}
#endif /* CONFIG_SHDISP */
		mdss_dsi_state_reset(ctx->ctl->panel_data);
	} else if (cnt < 0) {
		ctx->vsync_pending++;
		ctx->tg_state = HW_TG_OFF;
	}

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
exit_intr_done_1:
	if (!skipped) {
		mdss_mdp_toggle_timegen(ctx->ctl, 1);
	}
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (POL) */
	mdss_mdp_hr_polarity_exec(ctl, vsync_time, cnt);
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (FPSLED) */
	mdss_mdp_fps_led_vsync(ctx, vsync_time);
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	hrt = mdss_mdp_hr_video_update_hrt(ctx);

exit_intr_done_0:
	if (hrt) {
		pr_debug("LEAVE hrt expire %d\n", hrt);
	} else {
		pr_debug("LEAVE\n");
	}
#endif /* CONFIG_SHDISP */

	spin_unlock(&ctx->hrtimer_lock);
	ATRACE_END("vsync");
}

static int mdss_mdp_hr_video_pollwait(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_hr_video_ctx *ctx = ctl->priv_data;
	u32 mask, status;
	int rc;
	unsigned long flags;

	mask = MDP_INTR_MASK_INTF_VSYNC(ctl->intf_num);

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);
	rc = readl_poll_timeout(ctl->mdata->mdp_base + MDSS_MDP_REG_INTR_STATUS,
		status,
		(status & mask) || try_wait_for_completion(&ctx->vsync_comp),
		1000,
		VSYNC_TIMEOUT_US);
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);

	if (rc == 0) {
		MDSS_XLOG(ctl->num, ctl->vsync_cnt);
		pr_debug("vsync poll successful! rc=%d status=0x%x\n",
				rc, status);
		ctx->poll_cnt++;
		if (status) {
			struct mdss_mdp_vsync_handler *tmp;
			ktime_t vsync_time = ktime_get();

			spin_lock_irqsave(&ctx->vsync_lock, flags);
			list_for_each_entry(tmp, &ctx->vsync_handlers, list)
				tmp->vsync_handler(ctl, vsync_time);
			spin_unlock_irqrestore(&ctx->vsync_lock, flags);
		}
	} else {
		pr_warn("vsync poll timed out! rc=%d status=0x%x mask=0x%x\n",
				rc, status, mask);
	}

	return rc;
}

static int mdss_mdp_hr_video_wait4comp(struct mdss_mdp_ctl *ctl, void *arg)
{
	struct mdss_mdp_hr_video_ctx *ctx;
	int rc;

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	WARN(!ctx->wait_pending, "waiting without commit! ctl=%d", ctl->num);

	if (ctx->polling_en) {
		rc = mdss_mdp_hr_video_pollwait(ctl);
	} else {
		mutex_unlock(&ctl->lock);
		rc = wait_for_completion_timeout(&ctx->vsync_comp,
				usecs_to_jiffies(VSYNC_TIMEOUT_US));
		mutex_lock(&ctl->lock);
		if (rc == 0) {
			pr_warn("vsync wait timeout %d, fallback to poll mode\n",
					ctl->num);
			ctx->polling_en++;
			rc = mdss_mdp_hr_video_pollwait(ctl);
		} else {
			rc = 0;
		}
	}

	mdss_mdp_ctl_notify(ctl,
			rc ? MDP_NOTIFY_FRAME_TIMEOUT : MDP_NOTIFY_FRAME_DONE);

	if (ctx->wait_pending) {
		ctx->wait_pending = 0;
//		video_vsync_irq_disable(ctl);
	}

	return rc;
}

static void recover_underrun_work(struct work_struct *work)
{
	struct mdss_mdp_ctl *ctl =
		container_of(work, typeof(*ctl), recover_work);

	if (!ctl || !ctl->add_vsync_handler) {
		pr_err("ctl or vsync handler is NULL\n");
		return;
	}

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);
	ctl->add_vsync_handler(ctl, &ctl->recover_underrun_handler);
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
}

static void mdss_mdp_hr_video_underrun_intr_done(void *arg)
{
	struct mdss_mdp_ctl *ctl = arg;
	if (unlikely(!ctl))
		return;

	ctl->underrun_cnt++;
	MDSS_XLOG(ctl->num, ctl->underrun_cnt);
	MDSS_XLOG_TOUT_HANDLER("mdp", "dsi0", "dsi1", "edp", "hdmi", "panic");
	trace_mdp_video_underrun_done(ctl->num, ctl->underrun_cnt);
	pr_debug("display underrun detected for ctl=%d count=%d\n", ctl->num,
			ctl->underrun_cnt);

	if (ctl->opmode & MDSS_MDP_CTL_OP_PACK_3D_ENABLE)
		schedule_work(&ctl->recover_work);
}

static int mdss_mdp_hr_video_vfp_fps_update(struct mdss_mdp_ctl *ctl, int new_fps)
{
	int curr_fps;
	u32 add_v_lines = 0;
	u32 current_vsync_period_f0, new_vsync_period_f0;
	struct mdss_panel_data *pdata;
	struct mdss_mdp_hr_video_ctx *ctx;
	u32 vsync_period, hsync_period;

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	pdata = ctl->panel_data;
	if (pdata == NULL) {
		pr_err("%s: Invalid panel data\n", __func__);
		return -EINVAL;
	}

	vsync_period = mdss_panel_get_vtotal(&pdata->panel_info);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	hsync_period = mdss_panel_get_htotal(&pdata->panel_info);
#else  /* CONFIG_SHDISP */
	hsync_period = mdss_panel_get_htotal(&pdata->panel_info, true);
#endif /* CONFIG_SHDISP */
	curr_fps = mdss_panel_get_framerate(&pdata->panel_info);

	if (curr_fps > new_fps) {
		add_v_lines = mult_frac(vsync_period,
				(curr_fps - new_fps), new_fps);
		pdata->panel_info.lcdc.v_front_porch += add_v_lines;
	} else {
		add_v_lines = mult_frac(vsync_period,
				(new_fps - curr_fps), new_fps);
		pdata->panel_info.lcdc.v_front_porch -= add_v_lines;
	}

	vsync_period = mdss_panel_get_vtotal(&pdata->panel_info);
	current_vsync_period_f0 = mdp_video_read(ctx,
		MDSS_MDP_REG_INTF_VSYNC_PERIOD_F0);
	new_vsync_period_f0 = (vsync_period * hsync_period);

	mdp_video_write(ctx, MDSS_MDP_REG_INTF_VSYNC_PERIOD_F0,
			current_vsync_period_f0 | 0x800000);
	if (new_vsync_period_f0 & 0x800000) {
		mdp_video_write(ctx, MDSS_MDP_REG_INTF_VSYNC_PERIOD_F0,
			new_vsync_period_f0);
	} else {
		mdp_video_write(ctx, MDSS_MDP_REG_INTF_VSYNC_PERIOD_F0,
			new_vsync_period_f0 | 0x800000);
		mdp_video_write(ctx, MDSS_MDP_REG_INTF_VSYNC_PERIOD_F0,
			new_vsync_period_f0 & 0x7fffff);
	}

	return 0;
}

static int mdss_mdp_hr_video_config_fps(struct mdss_mdp_ctl *ctl,
					struct mdss_mdp_ctl *sctl, int new_fps)
{
	struct mdss_mdp_hr_video_ctx *ctx;
	struct mdss_panel_data *pdata;
	int rc = 0;
	u32 hsync_period, vsync_period;

	pr_debug("Updating fps for ctl=%d\n", ctl->num);

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	pdata = ctl->panel_data;
	if (pdata == NULL) {
		pr_err("%s: Invalid panel data\n", __func__);
		return -EINVAL;
	}

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (MFR) */
	if (0 <= new_fps && new_fps <= 6) {
		unsigned long flags;
		pr_debug("NEW_FPS=%d, MFR=%d, MFR_REQ=%d\n",
				new_fps, ctx->mfr, ctx->mfr_req);
		if ((new_fps == ctx->mfr) || (ctx->mfr != ctx->mfr_req)) {
			rc = -EINVAL;
		}
		if (!ctx->power_on) {
			rc = -EINVAL;
		}
		if (rc == 0) {
#ifdef CONFIG_SHDISP /* CUST_ID_00053 */
			mutex_lock(&ctx->suspend_mtx);
#endif /* CONFIG_SHDISP */
			schedule_work_ctrl(ctx, -1, NULL); /* -1:block queueing work */
			mdss_mdp_hr_video_clk_on(ctx);
			spin_lock_irqsave(&ctx->hrtimer_lock, flags);
			ctx->mfr_req = new_fps;
			INIT_COMPLETION(ctx->mfr_comp);
			spin_unlock_irqrestore(&ctx->hrtimer_lock, flags);
			mdss_mdp_queue_commit(ctx);
			wait_for_completion_timeout(
					&ctx->mfr_comp, usecs_to_jiffies(VSYNC_TIMEOUT_US));
			pr_debug("done MFR=%d\n", ctx->mfr);
#ifdef CONFIG_SHDISP /* CUST_ID_00053 */
			mutex_unlock(&ctx->suspend_mtx);
#endif /* CONFIG_SHDISP */
		}
		return rc;
	}
#endif /* CONFIG_SHDISP */

	if (!pdata->panel_info.dynamic_fps) {
		pr_err("%s: Dynamic fps not enabled for this panel\n",
						__func__);
		return -EINVAL;
	}

	vsync_period = mdss_panel_get_vtotal(&pdata->panel_info);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	hsync_period = mdss_panel_get_htotal(&pdata->panel_info);
#else  /* CONFIG_SHDISP */
	hsync_period = mdss_panel_get_htotal(&pdata->panel_info, true);
#endif /* CONFIG_SHDISP */

	if (pdata->panel_info.dfps_update
			!= DFPS_SUSPEND_RESUME_MODE) {
		if (pdata->panel_info.dfps_update
				== DFPS_IMMEDIATE_CLK_UPDATE_MODE) {
			if (!ctx->timegen_en) {
				pr_err("TG is OFF. DFPS mode invalid\n");
				return -EINVAL;
			}
			ctl->force_screen_state = MDSS_SCREEN_FORCE_BLANK;
			mdss_mdp_display_commit(ctl, NULL);
			mdss_mdp_display_wait4comp(ctl);
			mdp_video_write(ctx,
					MDSS_MDP_REG_INTF_TIMING_ENGINE_EN, 0);
			/*
			 * Need to wait for atleast one vsync time for proper
			 * TG OFF before doing changes on interfaces
			 */
			msleep(20);
			rc = mdss_mdp_ctl_intf_event(ctl,
					MDSS_EVENT_PANEL_UPDATE_FPS,
					(void *) (unsigned long) new_fps);
			WARN(rc, "intf %d panel fps update error (%d)\n",
							ctl->intf_num, rc);
			mdp_video_write(ctx,
					MDSS_MDP_REG_INTF_TIMING_ENGINE_EN, 1);
			/*
			 * Add memory barrier to make sure the MDP Video
			 * mode engine is enabled before next frame is sent
			 */
			mb();
			ctl->force_screen_state = MDSS_SCREEN_DEFAULT;
			mdss_mdp_display_commit(ctl, NULL);
			mdss_mdp_display_wait4comp(ctl);
		} else if (pdata->panel_info.dfps_update
				== DFPS_IMMEDIATE_PORCH_UPDATE_MODE){
			if (!ctx->timegen_en) {
				pr_err("TG is OFF. DFPS mode invalid\n");
				return -EINVAL;
			}

			video_vsync_irq_enable(ctl, true);
			INIT_COMPLETION(ctx->vsync_comp);
			rc = wait_for_completion_timeout(&ctx->vsync_comp,
				usecs_to_jiffies(VSYNC_TIMEOUT_US));
			WARN(rc <= 0, "timeout (%d) vsync interrupt on ctl=%d\n",
				rc, ctl->num);
			rc = 0;
			video_vsync_irq_disable(ctl);

			rc = mdss_mdp_hr_video_vfp_fps_update(ctl, new_fps);
			if (rc < 0) {
				pr_err("%s: Error during DFPS\n", __func__);
				return rc;
			}
			if (sctl) {
				rc = mdss_mdp_hr_video_vfp_fps_update(sctl,
								new_fps);
				if (rc < 0) {
					pr_err("%s: DFPS error\n", __func__);
					return rc;
				}
			}
			rc = mdss_mdp_ctl_intf_event(ctl,
					MDSS_EVENT_PANEL_UPDATE_FPS,
					(void *) (unsigned long) new_fps);
			WARN(rc, "intf %d panel fps update error (%d)\n",
							ctl->intf_num, rc);
		} else {
			pr_err("intf %d panel, unknown FPS mode\n",
							ctl->intf_num);
			return -EINVAL;
		}
	} else {
		rc = mdss_mdp_ctl_intf_event(ctl,
				MDSS_EVENT_PANEL_UPDATE_FPS,
				(void *) (unsigned long) new_fps);
		WARN(rc, "intf %d panel fps update error (%d)\n",
						ctl->intf_num, rc);
	}

	return rc;
}

static int mdss_mdp_hr_video_display(struct mdss_mdp_ctl *ctl, void *arg)
{
	struct mdss_mdp_hr_video_ctx *ctx;
	struct mdss_panel_data *pdata = ctl->panel_data;
	int rc;

	pr_debug("kickoff ctl=%d\n", ctl->num);

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	if (!ctx->wait_pending) {
		ctx->wait_pending++;
		INIT_COMPLETION(ctx->vsync_comp);
	} else {
		WARN(1, "commit without wait! ctl=%d", ctl->num);
	}

	MDSS_XLOG(ctl->num, ctl->underrun_cnt);

#ifdef CONFIG_SHDISP /* CUST_ID_00037 */
	if (ctx->suspend) {
		return 0;
	}
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	schedule_work_ctrl(ctx, -1, NULL); /* -1:block queueing work */
#else  /* CONFIG_SHDISP */
	if (cancel_delayed_work_sync(&ctx->hr_video_clk_work)) {
		pr_debug("Cancelling Pending  CLK ON at next tick%s %d \n", __func__,
				__LINE__);
	}
#endif /* CONFIG_SHDISP */

	if (!ctx->power_on)  {
		/* First on, need to turn on the panel */
#ifndef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
		ctx->hr_video_state = FIRST_UPDATE;
#endif /* CONFIG_SHDISP */
		ctx->power_on = true;

		rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_UNBLANK, NULL);
		if (rc) {
			pr_warn("intf #%d unblank error (%d)\n",
					ctl->intf_num, rc);
			video_vsync_irq_disable(ctl);
			ctx->wait_pending = 0;
			return rc;
		}

		pr_debug("enabling timing gen for intf=%d\n", ctl->intf_num);

		if (pdata->panel_info.cont_splash_enabled) {
			rc = wait_for_completion_timeout(&ctx->vsync_comp,
					usecs_to_jiffies(VSYNC_TIMEOUT_US));
		}

		rc = mdss_iommu_ctrl(1);
		if (IS_ERR_VALUE(rc)) {
			pr_err("IOMMU attach failed\n");
			return rc;
		}
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);

		mdss_mdp_irq_enable(MDSS_MDP_IRQ_INTF_UNDER_RUN, ctl->intf_num);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
		mdss_mdp_hr_video_start_timer(ctx->ctl);
#endif /* CONFIG_SHDISP */
		mdss_mdp_hr_video_clk_on(ctx);
		mdss_mdp_queue_commit(ctx);

		ctx->timegen_en = true;
		rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_ON, NULL);
		WARN(rc, "intf %d panel on error (%d)\n", ctl->intf_num, rc);
	} else {
		mdss_mdp_hr_video_clk_on(ctx);
		mdss_mdp_queue_commit(ctx);

	}

	return 0;
}

int mdss_mdp_hr_video_reconfigure_splash_done(struct mdss_mdp_ctl *ctl,
	bool handoff)
{
	struct mdss_panel_data *pdata;
	int i, ret = 0, off;
	u32 data, flush;
	struct mdss_mdp_hr_video_ctx *ctx;

	off = 0;
	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx for ctl=%d\n", ctl->num);
		return -ENODEV;
	}

	pdata = ctl->panel_data;

	pdata->panel_info.cont_splash_enabled = 0;

	if (!handoff) {
		ret = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_CONT_SPLASH_BEGIN,
#ifdef CONFIG_SHDISP /* CUST_ID_00050 */
							(void *) DSI_HS_MODE);
#else  /* CONFIG_SHDISP */
					      NULL);
#endif /* CONFIG_SHDISP */
		if (ret) {
			pr_err("%s: Failed to handle 'CONT_SPLASH_BEGIN' event\n"
				, __func__);
			return ret;
		}

		/* clear up mixer0 and mixer1 */
		flush = 0;
		for (i = 0; i < 2; i++) {
			data = mdss_mdp_ctl_read(ctl,
				MDSS_MDP_REG_CTL_LAYER(i));
			if (data) {
				mdss_mdp_ctl_write(ctl,
					MDSS_MDP_REG_CTL_LAYER(i),
					MDSS_MDP_LM_BORDER_COLOR);
				flush |= (0x40 << i);
			}
		}
		mdss_mdp_ctl_write(ctl, MDSS_MDP_REG_CTL_FLUSH, flush);

		mdp_video_write(ctx, MDSS_MDP_REG_INTF_TIMING_ENGINE_EN, 0);
		/* wait for 1 VSYNC for the pipe to be unstaged */
		msleep(20);
#ifdef CONFIG_SHDISP /* CUST_ID_00050 */

		ret = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_CONT_SPLASH_BEGIN,
							(void *) DSI_LP_MODE);
		if (ret) {
			pr_err("%s: Failed to handle 'CONT_SPLASH_BEGIN' event\n"
				, __func__);
			return ret;
		}

#endif /* CONFIG_SHDISP */

		ret = mdss_mdp_ctl_intf_event(ctl,
			MDSS_EVENT_CONT_SPLASH_FINISH, NULL);
	}

	return ret;
}


int mdss_mdp_hr_video_start(struct mdss_mdp_ctl *ctl)
{
	struct mdss_data_type *mdata;
	struct mdss_panel_info *pinfo;
	struct mdss_mdp_hr_video_ctx *ctx;
	struct intf_timing_params itp = {0};
	u32 dst_bpp;
	int i;

	mdata = ctl->mdata;
	pinfo = &ctl->panel_data->panel_info;

	i = ctl->intf_num - MDSS_MDP_INTF0;
	if (i < mdata->nintf) {
		ctx = ((struct mdss_mdp_hr_video_ctx *) mdata->video_intf) + i;
		if (ctx->ref_cnt) {
			pr_err("Intf %d already in use\n", ctl->intf_num);
			return -EBUSY;
		}
		pr_debug("video Intf #%d base=%p", ctx->intf_num, ctx->base);
		ctx->ref_cnt++;
	} else {
		pr_err("Invalid intf number: %d\n", ctl->intf_num);
		return -EINVAL;
	}

	MDSS_XLOG(ctl->num, ctl->vsync_cnt);
	pr_debug("start ctl=%u\n", ctl->num);

	ctl->priv_data = ctx;
	ctx->intf_type = ctl->intf_type;
	ctx->power_on = false;
	ctx->hr_video_state = POWER_OFF;
	ctx->tg_state = HW_TG_OFF;
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSICLK) */
	ctx->clk_enabled = 1;
#endif /* CONFIG_SHDISP */
	ctx->ctl = ctl;

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (POL) */
	mdss_mdp_hr_polarity_init(ctx);
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (MFR) */
	ctx->mfr = 0;
	ctx->mfr_req = 0;
	init_completion(&ctx->mfr_comp);
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ-WORK) */
	ctx->work_cnt = 0;
	ctx->work_ctrl = 0;
	init_completion(&ctx->work_comp);
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00037 */
	ctx->clk_cnt = 0;
	ctx->suspend = SUS_IDLE;
	init_completion(&ctx->suspend_comp);
	mutex_init(&ctx->suspend_mtx);
	mutex_init(&ctx->clk_ctrl_mtx);
#endif /* CONFIG_SHDISP */

	init_completion(&ctx->vsync_comp);
	spin_lock_init(&ctx->vsync_lock);
	spin_lock_init(&ctx->hrtimer_lock);
	mutex_init(&ctx->vsync_mtx);
	atomic_set(&ctx->vsync_ref, 0);
	INIT_WORK(&ctl->recover_work, recover_underrun_work);
	mutex_init(&ctx->clk_mtx);
	INIT_WORK(&ctx->clk_work, clk_ctrl_work);
#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	INIT_WORK(&ctx->hr_video_clk_work, clk_ctrl_hr_video_work);
#else  /* CONFIG_SHDISP */
	INIT_DELAYED_WORK(&ctx->hr_video_clk_work, clk_ctrl_hr_video_work);
#endif /* CONFIG_SHDISP */
	ctx->hrtimer_next_wait_ns = VSYNC_TIMER;

	video_vsync_irq_enable(ctl, true);
	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_INTF_VSYNC, ctl->intf_num,
				   mdss_mdp_hr_video_vsync_intr_done, ctl);
	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_INTF_UNDER_RUN, ctl->intf_num,
				   mdss_mdp_hr_video_underrun_intr_done, ctl);

	dst_bpp = pinfo->fbc.enabled ? (pinfo->fbc.target_bpp) : (pinfo->bpp);

	itp.width = mult_frac((pinfo->xres + pinfo->lcdc.xres_pad),
				dst_bpp, pinfo->bpp);
	itp.height = pinfo->yres + pinfo->lcdc.yres_pad;
	itp.border_clr = pinfo->lcdc.border_clr;
	itp.underflow_clr = pinfo->lcdc.underflow_clr;
	itp.hsync_skew = pinfo->lcdc.hsync_skew;

	itp.xres =  mult_frac(pinfo->xres, dst_bpp, pinfo->bpp);
	itp.yres = pinfo->yres;
	itp.h_back_porch =  mult_frac(pinfo->lcdc.h_back_porch, dst_bpp,
			pinfo->bpp);
	itp.h_front_porch = mult_frac(pinfo->lcdc.h_front_porch, dst_bpp,
			pinfo->bpp);
	itp.v_back_porch =  mult_frac(pinfo->lcdc.v_back_porch, dst_bpp,
			pinfo->bpp);
	itp.v_front_porch = mult_frac(pinfo->lcdc.v_front_porch, dst_bpp,
			pinfo->bpp);
	itp.hsync_pulse_width = mult_frac(pinfo->lcdc.h_pulse_width, dst_bpp,
			pinfo->bpp);
	itp.vsync_pulse_width = pinfo->lcdc.v_pulse_width;

	mdss_mdp_register_hrtimer(ctx);

	if (mdss_mdp_hr_video_timegen_setup(ctl, &itp)) {
		pr_err("unable to get timing parameters\n");
		return -EINVAL;
	}
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_PANEL_FORMAT, ctl->dst_format);

	ctl->stop_fnc = mdss_mdp_hr_video_stop;
	ctl->display_fnc = mdss_mdp_hr_video_display;
	ctl->wait_fnc = mdss_mdp_hr_video_wait4comp;
	ctl->read_line_cnt_fnc = mdss_mdp_hr_video_line_count;
	ctl->add_vsync_handler = mdss_mdp_hr_video_add_hrt_vsync_handler;
	ctl->remove_vsync_handler = mdss_mdp_hr_video_remove_hrt_vsync_handler;
	ctl->config_fps_fnc = mdss_mdp_hr_video_config_fps;

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (DSIERR) */
	hr_video_tg_state = &ctx->tg_state;
	hr_video_tg_state_lock = &ctx->hrtimer_lock;
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
	{
		u32 tg_en = mdp_video_read(ctx, MDSS_MDP_REG_INTF_TIMING_ENGINE_EN);
		pr_debug("TG_EN=%d\n", tg_en);
		if (tg_en) {
			ctx->hrtimer_init = true;
			ctx->tg_state = HW_TG_ON;
			mdss_mdp_queue_commit(ctx);
		}
	}
#endif /* CONFIG_SHDISP */

	return 0;
}

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (FPSLED) */
extern struct fb_info *mdss_fb_get_fbinfo(int id);

static void mdss_mdp_fps_led_light_worker(struct work_struct *work)
{
	struct fps_led_ctx *fps_led;

	fps_led = container_of(work, struct fps_led_ctx, light_work);

	mdss_shdisp_tri_led_set_color(
			fps_led->led_red,
			fps_led->led_green,
			fps_led->led_blue);
}

static enum hrtimer_restart mdss_mdp_fps_led_timer_cb(struct hrtimer *timer)
{
	ktime_t frame_time;
	struct fps_led_ctx *fps_led;
	int new_state;

	pr_debug("in\n");

	fps_led = container_of(timer, struct fps_led_ctx, timer);

	hrtimer_forward_now(timer, ns_to_ktime(VSYNC_TIMER));

	spin_lock(&fps_led->lock);
	frame_time = fps_led->frame_time;
	spin_unlock(&fps_led->lock);

	fps_led->frame_hist <<= 1;

	if (!ktime_equal(fps_led->frame_time_bef, ns_to_ktime(0))) {
		if (!ktime_equal(frame_time, fps_led->frame_time_bef)) {
			fps_led->frame_hist |= 1;
		}
	}

	pr_debug("FPS_LED: frame_hist=%d%d%d%d%d%d%d%d\n",
			((fps_led->frame_hist >> 7) & 1),
			((fps_led->frame_hist >> 6) & 1),
			((fps_led->frame_hist >> 5) & 1),
			((fps_led->frame_hist >> 4) & 1),
			((fps_led->frame_hist >> 3) & 1),
			((fps_led->frame_hist >> 2) & 1),
			((fps_led->frame_hist >> 1) & 1),
			((fps_led->frame_hist >> 0) & 1));

	new_state = fps_led->state;

	if ((fps_led->frame_hist & 0x03) == 0x03) {
		new_state = FPS_LED_STATE_60HZ;
	} else if ((fps_led->frame_hist & 0x07) == 0x05) {
		new_state = FPS_LED_STATE_30HZ;
	} else if ((fps_led->frame_hist & 0x03) == 0x00) {
		new_state = FPS_LED_STATE_1HZ;
	}

	if (fps_led->state != new_state) {
		switch (new_state) {
		case FPS_LED_STATE_60HZ:
			pr_debug("FPS_LED: 60Hz\n");
			fps_led->led_red = 1;
			fps_led->led_green = 0;
			fps_led->led_blue = 0;
			schedule_work(&fps_led->light_work);
			break;
		case FPS_LED_STATE_30HZ:
			pr_debug("FPS_LED: 30Hz\n");
			fps_led->led_red = 0;
			fps_led->led_green = 1;
			fps_led->led_blue = 0;
			schedule_work(&fps_led->light_work);
			break;
		case FPS_LED_STATE_1HZ:
			pr_debug("FPS_LED: 1Hz\n");
			fps_led->led_red = 0;
			fps_led->led_green = 0;
			fps_led->led_blue = 1;
			schedule_work(&fps_led->light_work);
			break;
		default:
			; // Do nothing
		}
	}

	fps_led->state = new_state;
	fps_led->frame_time_bef = frame_time;

	pr_debug("out\n");

	return HRTIMER_RESTART;
}

static void mdss_mdp_fps_led_vsync(
		struct mdss_mdp_hr_video_ctx *ctx,
		ktime_t vsync_time)
{
	if (ctx->tg_state != HW_TG_OFF) {
		pr_debug("FPS_LED: output frame\n");
		spin_lock(&ctx->fps_led.lock);
		ctx->fps_led.frame_time = vsync_time;
		spin_unlock(&ctx->fps_led.lock);
	}
}

int mdss_mdp_fps_led_start(void)
{
	struct fb_info * fbi;
	struct msm_fb_data_type *mfd;
	struct mdss_overlay_private *mdp5_data = NULL;
	struct mdss_mdp_ctl *ctl;
	struct mdss_mdp_hr_video_ctx *ctx;
	ktime_t start_time;
	unsigned long flags;

	fbi = mdss_fb_get_fbinfo(0);
	if (!fbi) {
		pr_err("invalid fbi\n");
		return -ENODEV;
	}

	mfd = (struct msm_fb_data_type *)fbi->par;
	if (!mfd) {
		pr_err("invalid mfd\n");
		return -ENODEV;
	}

	mdp5_data = mfd_to_mdp5_data(mfd);
	if (!mdp5_data) {
		pr_err("invalid mdp5_data\n");
		return -ENODEV;
	}

	ctl = mdp5_data->ctl;
	if (!ctl) {
		pr_err("invalid ctl\n");
		return -ENODEV;
	}

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	if (!ctx->fps_led.inited) {
		ctx->fps_led.inited = true;
		INIT_WORK(&ctx->fps_led.light_work, mdss_mdp_fps_led_light_worker);
		hrtimer_init(&ctx->fps_led.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ctx->fps_led.timer.function = mdss_mdp_fps_led_timer_cb;
	} else {
		hrtimer_cancel(&ctx->fps_led.timer);
	}

	ctx->fps_led.state = FPS_LED_STATE_NONE;
	ctx->fps_led.frame_time_bef = ns_to_ktime(0);
	ctx->fps_led.frame_hist = 0;

	{
		ktime_t crr, vsync_time;
		s32 itmp;

		spin_lock_irqsave(&ctx->fps_led.lock, flags);
		vsync_time = ctx->fps_led.frame_time;
		spin_unlock_irqrestore(&ctx->fps_led.lock, flags);

		crr = ktime_get();
		div_s64_rem(ktime_to_ns(ktime_sub(crr, vsync_time)), VSYNC_TIMER, &itmp);
		itmp = VSYNC_TIMER - itmp;
		itmp += VSYNC_TIMER / 2;
		start_time = ktime_add(crr, ns_to_ktime(itmp));

		pr_debug("current:%lld vsync_time:%lld start_time:%lld\n",
				ktime_to_ns(crr),
				ktime_to_ns(vsync_time),
				ktime_to_ns(start_time));
	}

	hrtimer_start(
			&ctx->fps_led.timer,
			start_time,
			HRTIMER_MODE_ABS);

	return 0;
}

void mdss_mdp_fps_led_stop(void)
{
	struct fb_info * fbi;
	struct msm_fb_data_type *mfd;
	struct mdss_overlay_private *mdp5_data = NULL;
	struct mdss_mdp_ctl *ctl;
	struct mdss_mdp_hr_video_ctx *ctx;

	fbi = mdss_fb_get_fbinfo(0);
	if (!fbi) {
		pr_err("invalid fbi\n");
		return;
	}

	mfd = (struct msm_fb_data_type *)fbi->par;
	if (!mfd) {
		pr_err("invalid mfd\n");
		return;
	}

	mdp5_data = mfd_to_mdp5_data(mfd);
	if (!mdp5_data) {
		pr_err("invalid mdp5_data\n");
		return;
	}

	ctl = mdp5_data->ctl;
	if (!ctl) {
		pr_err("invalid ctl\n");
		return;
	}

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return;
	}

	if (ctx->fps_led.inited) {
		hrtimer_cancel(&ctx->fps_led.timer);
	}
}
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00037 */
int mdss_mdp_hr_video_suspend(struct mdss_mdp_ctl *ctl, int tg_en_flg)
{
	struct mdss_mdp_hr_video_ctx *ctx;
	unsigned long flags;
	
	pr_debug("ENTER(%d)\n", tg_en_flg);
	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}
	if (!ctx->power_on)  {
		pr_debug("panel power off\n");
		return -EPERM;
	}
	if (ctx->suspend) {
		pr_err("already suspended\n");
		return -EPERM;
	}
	mutex_lock(&ctx->suspend_mtx);
	hr_video_clk_ctrl(ctl, 1 + 2); /* lock clock */
	spin_lock_irqsave(&ctx->hrtimer_lock, flags);
	ctx->suspend = SUS_START;
	ctx->suspend_tg = tg_en_flg;
	INIT_COMPLETION(ctx->suspend_comp);
	spin_unlock_irqrestore(&ctx->hrtimer_lock, flags);
	wait_for_completion(&ctx->suspend_comp);
	pr_debug("LEAVE\n");
	return 0;
}

int mdss_mdp_hr_video_resume(struct mdss_mdp_ctl *ctl, int tg_en_flg)
{
	struct mdss_mdp_hr_video_ctx *ctx;
	unsigned long flags;
	
	pr_debug("ENTER(%d)\n", tg_en_flg);
	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}
	if (!ctx->power_on)  {
		pr_debug("panel power off\n");
		return -EPERM;
	}
	if (!ctx->suspend) {
		pr_err("not suspended\n");
		return -EPERM;
	}
	
	spin_lock_irqsave(&ctx->hrtimer_lock, flags);
	if (mdp_video_read(ctx, MDSS_MDP_REG_INTF_TIMING_ENGINE_EN)) {
		ctx->tg_state = HW_TG_ON;
		ctx->suspend = RES_WAIT;
	} else {
		ctx->tg_state = HW_TG_OFF;
		ctx->suspend = RES_START;
	}
	INIT_COMPLETION(ctx->suspend_comp);
	spin_unlock_irqrestore(&ctx->hrtimer_lock, flags);
	wait_for_completion(&ctx->suspend_comp);
	hr_video_clk_ctrl(ctl, 0 + 2); /* unlock clock */
	mutex_unlock(&ctx->suspend_mtx);
	pr_debug("LEAVE\n");
	return 0;
}
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00040 */
void mdss_mdp_hr_video_transfer_ctrl(struct msm_fb_data_type *mfd, int onoff)
{
	struct mdss_overlay_private *mdp5_data = NULL;
	struct mdss_mdp_ctl *ctl;
	struct mdss_mdp_hr_video_ctx *ctx;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata * mdss_dsi_ctrl;

	if (!mfd) {
		pr_err("invalid mfd\n");
		return;
	}

	mdp5_data = mfd_to_mdp5_data(mfd);
	if (!mdp5_data) {
		pr_err("invalid mdp5_data\n");
		return;
	}

	ctl = mdp5_data->ctl;
	if (!ctl) {
		pr_err("invalid ctl\n");
		return;
	}

	ctx = (struct mdss_mdp_hr_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return;
	}

	pdata = ctl->panel_data;
	if (!pdata) {
		pr_err("invalid pdata\n");
		return;
	}

	mdss_dsi_ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	
	if (onoff) {
		mdp_video_write(ctx, MDSS_MDP_REG_INTF_TIMING_ENGINE_EN, 1);
		wmb();
		mdss_dsi_ctrl->ctrl_state |= CTRL_STATE_MDP_ACTIVE;
		ctl->force_screen_state = MDSS_SCREEN_DEFAULT;
		mdss_mdp_display_commit(ctl, NULL);
		mdss_mdp_display_wait4comp(ctl);
	} else {
		ctl->force_screen_state = MDSS_SCREEN_FORCE_BLANK;
		mdss_mdp_display_commit(ctl, NULL);
		mdss_mdp_display_wait4comp(ctl);
		mdp_video_write(ctx, MDSS_MDP_REG_INTF_TIMING_ENGINE_EN, 0);
		wmb();
		msleep(20);
		mdss_dsi_ctrl->ctrl_state &= ~CTRL_STATE_MDP_ACTIVE;
		mdss_dsi_controller_cfg(true, pdata);
	}
}
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00012 (1HZ) */
int mdss_mdp_hr_video_clk_ctrl(struct mdss_mdp_ctl *ctl, int onoff)
{
	struct mdss_mdp_hr_video_ctx *ctx = ctl->priv_data;
	
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}
	return hr_video_clk_ctrl(ctl, (onoff & 1) + 2);
}
#endif /* CONFIG_SHDISP */

#ifdef CONFIG_SHDISP /* CUST_ID_00053 */
int mdss_mdp_hr_video_clkchg_mdp_update(struct mdss_mdp_ctl *ctl)
{
	int ret = 0;
	u32 hsync_period, vsync_period;
	u32 hsync_start_x, hsync_end_x, display_v_start, display_v_end;
	u32 display_hctl, hsync_ctl;
	struct mdss_panel_info *pinfo;
	struct mdss_mdp_hr_video_ctx *ctx;

	pinfo = &ctl->panel_data->panel_info;
	ctx = ctl->priv_data;
	if (pinfo == NULL) {
		pr_err("invalid pinfo\n");
		return -ENODEV;
	}
	if (ctx == NULL) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}
	hsync_period = pinfo->lcdc.h_pulse_width + pinfo->lcdc.h_back_porch +
			pinfo->xres + pinfo->lcdc.xres_pad + pinfo->lcdc.h_front_porch;
	vsync_period = pinfo->lcdc.v_pulse_width + pinfo->lcdc.v_back_porch +
			pinfo->yres + pinfo->lcdc.yres_pad + pinfo->lcdc.v_front_porch;

	display_v_start = ((pinfo->lcdc.v_pulse_width + pinfo->lcdc.v_back_porch) *
			hsync_period) + pinfo->lcdc.hsync_skew;
	display_v_end = ((vsync_period - pinfo->lcdc.v_front_porch) * hsync_period) +
			pinfo->lcdc.hsync_skew - 1;

	if (ctx->intf_type == MDSS_INTF_EDP) {
		display_v_start += pinfo->lcdc.h_pulse_width + pinfo->lcdc.h_back_porch;
		display_v_end -= pinfo->lcdc.h_front_porch;
	}

	hsync_start_x = pinfo->lcdc.h_back_porch + pinfo->lcdc.h_pulse_width;
	hsync_end_x = hsync_period - pinfo->lcdc.h_front_porch - 1;

	hsync_ctl = (hsync_period << 16) | pinfo->lcdc.h_pulse_width;
	display_hctl = (hsync_end_x << 16) | hsync_start_x;


	mdp_video_write(ctx, MDSS_MDP_REG_INTF_HSYNC_CTL, hsync_ctl);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_VSYNC_PERIOD_F0,
			vsync_period * hsync_period);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_VSYNC_PULSE_WIDTH_F0,
			   pinfo->lcdc.v_pulse_width * hsync_period);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_DISPLAY_HCTL, display_hctl);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_DISPLAY_V_START_F0,
			   display_v_start);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_DISPLAY_V_END_F0, display_v_end);

	return ret;
}
#endif /* CONFIG_SHDISP */

