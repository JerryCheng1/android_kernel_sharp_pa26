/* drivers/sharp/shtps/sy3000/shtps_rmi_sub.c
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
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/input/mt.h>

#include <linux/pm_qos.h>

#include <linux/init.h>
#include <asm/processor.h>
#include <asm/uaccess.h>


#include <linux/cpufreq.h>
#include <linux/notifier.h>

#include <sharp/shtps_dev.h>
#include <sharp/sh_smem.h>
#include <sharp/sh_boot_manager.h>

#include "shtps_mxt.h"
#include "shtps_param_extern.h"
#include "shtps_mxt_sub.h"
#include "shtps_log.h"

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
	static DEFINE_MUTEX(shtps_cpu_clock_ctrl_lock);
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
	static DEFINE_MUTEX(shtps_cpu_idle_sleep_ctrl_lock);
#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

#define	TPS_MUTEX_LOG_LOCK(VALUE)		SHTPS_LOG_DBG_PRINT("mutex_lock:   -> (%s)\n",VALUE)
#define	TPS_MUTEX_LOG_UNLOCK(VALUE)		SHTPS_LOG_DBG_PRINT("mutex_unlock: <- (%s)\n",VALUE)

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
	static void shtps_mutex_lock_cpu_clock_ctrl(void);
	static void shtps_mutex_unlock_cpu_clock_ctrl(void);
	static void shtps_perflock_enable(struct shtps_mxt *ts);
	static void shtps_perflock_disable(struct shtps_mxt *ts);
	static int shtps_perflock_disable_timer_start(struct shtps_mxt *ts, unsigned long delay_ms);
	static void shtps_perflock_disable_delayed_work_function(struct work_struct *work);
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
	static void shtps_mutex_lock_cpu_idle_sleep_ctrl(void);
	static void shtps_mutex_unlock_cpu_idle_sleep_ctrl(void);
	static void shtps_wake_lock_idle_l(struct shtps_mxt *ts);
	static void shtps_wake_unlock_idle_l(struct shtps_mxt *ts);
#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
int shtps_proximity_state_check(struct shtps_mxt *ts)
{
	int state = -1;
	int data = -1;

	SHTPS_LOG_FUNC_CALL();

	SHTPS_LOG_DBG_PRINT("[proximity] check state start\n");
	PROX_stateread_func(&state, &data);
	SHTPS_LOG_DBG_PRINT("[proximity] check state end(state:%d, data:%d)\n",state, data);

	if (state == SHTPS_PROXIMITY_ENABLE &&
		data == SHTPS_PROXIMITY_NEAR) {
		return 1;
	} else {
		return 0;
	}
}

int shtps_proximity_check(struct shtps_mxt *ts)
{
	int data = -1;

	SHTPS_LOG_FUNC_CALL();

	SHTPS_LOG_DBG_PRINT("[proximity] check start\n");
	PROX_dataread_func(&data);
	SHTPS_LOG_DBG_PRINT("[proximity] check end(data:%d)\n",data);

	return data;
}
#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
static unsigned int perflock_freq = 0;

static void shtps_mutex_lock_cpu_clock_ctrl(void)
{
	TPS_MUTEX_LOG_LOCK("shtps_cpu_clock_ctrl_lock");
	mutex_lock(&shtps_cpu_clock_ctrl_lock);
}

static void shtps_mutex_unlock_cpu_clock_ctrl(void)
{
	TPS_MUTEX_LOG_UNLOCK("shtps_cpu_clock_ctrl_lock");
	mutex_unlock(&shtps_cpu_clock_ctrl_lock);
}

static void shtps_perflock_enable(struct shtps_mxt *ts)
{
	int cpu;
	shtps_mutex_lock_cpu_clock_ctrl();

	perflock_freq = SHTPS_PERF_LOCK_CLOCK_FREQUENCY;
	for_each_online_cpu(cpu) {
		cpufreq_update_policy(cpu);
	}
	SHTPS_LOG_DBG_PRINT("perf_lock start (%d ms)\n", SHTPS_PERF_LOCK_ENABLE_TIME_MS);

	shtps_mutex_unlock_cpu_clock_ctrl();
}

static void shtps_perflock_disable(struct shtps_mxt *ts)
{
	int cpu;
	shtps_mutex_lock_cpu_clock_ctrl();

	perflock_freq = 0; 
	for_each_online_cpu(cpu) { 
		cpufreq_update_policy(cpu);
	}
	SHTPS_LOG_DBG_PRINT("perf_lock end\n");

	shtps_mutex_unlock_cpu_clock_ctrl();
}

static int shtps_perflock_ctrl_callback(struct notifier_block *nb, unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	switch (event) {
	case CPUFREQ_ADJUST:
//		SHTPS_LOG_DBG_PRINT("set perflock [%d]\n", perflock_freq);
		cpufreq_verify_within_limits(policy, perflock_freq, UINT_MAX); 
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static int shtps_perflock_disable_timer_start(struct shtps_mxt *ts, unsigned long delay_ms)
{
	cancel_delayed_work(&ts->cpu_clock_ctrl_p->perf_lock_disable_delayed_work);
	schedule_delayed_work(&ts->cpu_clock_ctrl_p->perf_lock_disable_delayed_work, msecs_to_jiffies(delay_ms));

	return 0;
}

static void shtps_perflock_disable_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_cpu_clock_ctrl_info *cpuctrl = container_of(dw, struct shtps_cpu_clock_ctrl_info, perf_lock_disable_delayed_work);
	//struct shtps_mxt *ts = container_of(cpuctrl, struct shtps_mxt, cpu_clock_ctrl_p);
	struct shtps_mxt *ts = cpuctrl->ts_p;

	SHTPS_LOG_FUNC_CALL();

	shtps_perflock_disable(ts);
	SHTPS_LOG_DBG_PRINT("perf_lock end by Timer\n");

	return;
}

static struct notifier_block shtps_perflock_ctrl_notifier = { 
	.notifier_call = shtps_perflock_ctrl_callback,
};

void shtps_perflock_register_notifier(void)
{
	if( cpufreq_register_notifier(&shtps_perflock_ctrl_notifier, CPUFREQ_POLICY_NOTIFIER) ){
		pr_err("%s: cannot register cpufreq notifier\n", __func__);
	}
}

void shtps_perflock_unregister_notifier(void)
{
	if( cpufreq_unregister_notifier(&shtps_perflock_ctrl_notifier, CPUFREQ_POLICY_NOTIFIER) ){
		pr_err("%s: cannot unregister cpufreq notifier\n", __func__);
	}
}

#if defined(SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE)
static int shtps_read_curfreq_file(char *filename, long *freq_p)
{
	int ret = 0;
	struct file *file;
	mm_segment_t fs;
	char buf[16];
	int nr_read;

	file = filp_open(filename, O_RDONLY, 0); 

	if(IS_ERR(file)){
		return -1;
	}

	fs = get_fs();
	set_fs(get_ds());

	nr_read = file->f_op->read(file, buf, sizeof(buf)-1, &file->f_pos);
	buf[nr_read] = '\0';

	if(nr_read == 0){
		ret = -1;
	}else{
		ret = kstrtol(buf, 10, freq_p);
	}

	set_fs(fs);
	filp_close(file, NULL);

	return ret;
}

static int shtps_check_curfreq_condition(void)
{
	long cpu0_freq;
	long cpu3_freq;
	if(shtps_read_curfreq_file("sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", &cpu0_freq) != 0){
		SHTPS_LOG_DBG_PRINT("File not found : sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq \n");
		return 0;
	}

	if(shtps_read_curfreq_file("sys/devices/system/cpu/cpu3/cpufreq/scaling_cur_freq", &cpu3_freq) != 0){
		SHTPS_LOG_DBG_PRINT("File not found : sys/devices/system/cpu/cpu3/cpufreq/scaling_cur_freq \n");
		return 0;
	}

	if(cpu0_freq > 960000 || cpu3_freq > 960000){
		SHTPS_LOG_DBG_PRINT("cpu_freq > 960000\n");
		return 0;
	}

	return 1;
}
#endif /* SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE */

void shtps_perflock_enable_start(struct shtps_mxt *ts, u8 event)
{
	if(event == SHTPS_EVENT_TD){
		
		#if defined(SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE)
			if(shtps_check_curfreq_condition() != 1)	return;
		#endif /*SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE*/
		
		if( shtps_fwctl_get_dev_state(ts) == SHTPS_DEV_STATE_ACTIVE
		#if !defined(SHTPS_PERFLOCK_DOZE_DISABLE)
			|| shtps_fwctl_get_dev_state(ts) == SHTPS_DEV_STATE_DOZE
		#endif /*!defined(SHTPS_PERFLOCK_DOZE_DISABLE)*/
		 ){
			shtps_perflock_enable(ts);
			shtps_perflock_disable_timer_start(ts, SHTPS_PERF_LOCK_ENABLE_TIME_MS /* ts->perf_lock_enable_time_ms */);
			SHTPS_LOG_DBG_PRINT("perf_lock start by TouchDown\n");
		}

	}else if(event == SHTPS_EVENT_DRAG){
		if(ts->cpu_clock_ctrl_p->report_event == SHTPS_EVENT_TD){
			
			#if defined(SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE)
				if(shtps_check_curfreq_condition() != 1)	return;
			#endif /*SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE*/
			
			if( shtps_fwctl_get_dev_state(ts) == SHTPS_DEV_STATE_ACTIVE
			#if !defined(SHTPS_PERFLOCK_DOZE_DISABLE)
				|| shtps_fwctl_get_dev_state(ts) == SHTPS_DEV_STATE_DOZE
			#endif /*!defined(SHTPS_PERFLOCK_DOZE_DISABLE)*/
			 ){
				shtps_perflock_enable(ts);
				shtps_perflock_disable_timer_start(ts, SHTPS_PERF_LOCK_ENABLE_TIME_MS /* ts->perf_lock_enable_time_ms */);
				SHTPS_LOG_DBG_PRINT("perf_lock start by Drag\n");
			}
		}
	}
	#if !defined( SHTPS_PERFLOCK_TU_RELEASE_DISABLE )
	else if(event == SHTPS_EVENT_TU){
		shtps_perflock_disable(ts);
		SHTPS_LOG_DBG_PRINT("perf_lock end by TouchUp\n");
	}
	#endif /* SHTPS_PERFLOCK_TU_RELEASE_DISABLE */
}

void shtps_perflock_sleep(struct shtps_mxt *ts)
{
	shtps_perflock_set_event(ts, SHTPS_EVENT_TU);	/* ts->cpu_clock_ctrl_p->report_event = SHTPS_EVENT_TU; */
	shtps_perflock_disable(ts);
	SHTPS_LOG_DBG_PRINT("perf_lock end by ForceTouchUp\n");
}

void shtps_perflock_init(struct shtps_mxt *ts)
{
	ts->cpu_clock_ctrl_p = kzalloc(sizeof(struct shtps_cpu_clock_ctrl_info), GFP_KERNEL);
	if(ts->cpu_clock_ctrl_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}

	shtps_perflock_set_event(ts, SHTPS_EVENT_TU);	/* ts->cpu_clock_ctrl_p->report_event = SHTPS_EVENT_TU; */
	//ts->cpu_clock_ctrl_p->perf_lock_enable_time_ms = SHTPS_perflock_ENABLE_TIME_MS;
//	perf_lock_init(&ts->cpu_clock_ctrl_p->perf_lock, SHTPS_perflock_CLOCK_FREQUENCY, "shtps_perf_lock");
	INIT_DELAYED_WORK(&ts->cpu_clock_ctrl_p->perf_lock_disable_delayed_work, shtps_perflock_disable_delayed_work_function);
	ts->cpu_clock_ctrl_p->ts_p = ts;
}

void shtps_perflock_deinit(struct shtps_mxt *ts)
{
	if(ts->cpu_clock_ctrl_p)	kfree(ts->cpu_clock_ctrl_p);
	ts->cpu_clock_ctrl_p = NULL;

}

void shtps_perflock_set_event(struct shtps_mxt *ts, u8 event)
{
	ts->cpu_clock_ctrl_p->report_event = event;
}
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
static void shtps_mutex_lock_cpu_idle_sleep_ctrl(void)
{
	TPS_MUTEX_LOG_LOCK("shtps_cpu_idle_sleep_ctrl_lock");
	mutex_lock(&shtps_cpu_idle_sleep_ctrl_lock);
}

static void shtps_mutex_unlock_cpu_idle_sleep_ctrl(void)
{
	TPS_MUTEX_LOG_UNLOCK("shtps_cpu_idle_sleep_ctrl_lock");
	mutex_unlock(&shtps_cpu_idle_sleep_ctrl_lock);
}

static void shtps_wake_lock_idle_l(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_cpu_idle_sleep_ctrl();
	if(ts->cpu_idle_sleep_ctrl_p->wake_lock_idle_state == 0){
		SHTPS_LOG_DBG_PRINT("wake_lock_idle on\n");
		pm_qos_update_request(&ts->cpu_idle_sleep_ctrl_p->qos_cpu_latency, SHTPS_QOS_LATENCY_DEF_VALUE);
		ts->cpu_idle_sleep_ctrl_p->wake_lock_idle_state = 1;
	}
	shtps_mutex_unlock_cpu_idle_sleep_ctrl();
}

static void shtps_wake_unlock_idle_l(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_cpu_idle_sleep_ctrl();
	if(ts->cpu_idle_sleep_ctrl_p->wake_lock_idle_state != 0){
		SHTPS_LOG_DBG_PRINT("wake_lock_idle off\n");
		pm_qos_update_request(&ts->cpu_idle_sleep_ctrl_p->qos_cpu_latency, PM_QOS_DEFAULT_VALUE);
		ts->cpu_idle_sleep_ctrl_p->wake_lock_idle_state = 0;
	}
	shtps_mutex_unlock_cpu_idle_sleep_ctrl();
}

void shtps_wake_lock_idle(struct shtps_mxt *ts)
{
	if(SHTPS_STATE_ACTIVE == ts->state_mgr.state){
		shtps_wake_lock_idle_l(ts);
	}
}

void shtps_wake_unlock_idle(struct shtps_mxt *ts)
{
	shtps_wake_unlock_idle_l(ts);
}

void shtps_cpu_idle_sleep_wake_lock_init( struct shtps_mxt *ts )
{
	ts->cpu_idle_sleep_ctrl_p = kzalloc(sizeof(struct shtps_cpu_idle_sleep_ctrl_info), GFP_KERNEL);
	if(ts->cpu_idle_sleep_ctrl_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// ts->cpu_idle_sleep_ctrl_p->wake_lock_idle_state = 0;	// no need
	pm_qos_add_request(&ts->cpu_idle_sleep_ctrl_p->qos_cpu_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
}

void shtps_cpu_idle_sleep_wake_lock_deinit( struct shtps_mxt *ts )
{
	pm_qos_remove_request(&ts->cpu_idle_sleep_ctrl_p->qos_cpu_latency);

	if(ts->cpu_idle_sleep_ctrl_p)	kfree(ts->cpu_idle_sleep_ctrl_p);
	ts->cpu_idle_sleep_ctrl_p = NULL;
}

#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_TPIN_CHECK_ENABLE ) || defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
int shtps_tpin_enable_check(struct shtps_mxt *ts)
{
	int ret;
	int val;

	SHTPS_LOG_FUNC_CALL();
	ret = gpio_request(SHTPS_GPIO_TPIN_NO, "tpin");
	if(ret){
		SHTPS_LOG_DBG_PRINT("%s() gpio_request() error[%d]\n", __func__, ret);
	}

	ret = gpio_tlmm_config(GPIO_CFG(SHTPS_GPIO_TPIN_NO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(ret){
		SHTPS_LOG_DBG_PRINT("%s() gpio_tlmm_config(set) error[%d]\n", __func__, ret);
	}
	udelay(50);
	val = gpio_get_value(SHTPS_GPIO_TPIN_NO);
	SHTPS_LOG_DBG_PRINT("%s() gpio_get_value() val = %d\n", __func__, val);
	
	ret = gpio_tlmm_config(GPIO_CFG(SHTPS_GPIO_TPIN_NO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(ret){
		SHTPS_LOG_DBG_PRINT("%s() gpio_tlmm_config(reset) error[%d]\n", __func__, ret);
	}

	gpio_free(SHTPS_GPIO_TPIN_NO);

	if(!val) {
		SHTPS_LOG_ERR_PRINT("Upper unit does not exist.\n");
		return -1;
	}

	return 0;
}
#endif /* SHTPS_TPIN_CHECK_ENABLE || SHTPS_CHECK_CRC_ERROR_ENABLE*/

/* -----------------------------------------------------------------------------------
 */
