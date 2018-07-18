/* drivers/sharp/shtps/atmel/shtps_mxt_ldm.c
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
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/slab.h>

#include <sharp/sh_smem.h>
#include <sharp/shtps_dev.h>

#include "shtps_mxt.h"
#include "shtps_mxt_sub.h"
#include "shtps_log.h"
#include "shtps_param_extern.h"
#include "shtps_filter.h"

/* -----------------------------------------------------------------------------------
 */
dev_t 					shtpsif_devid;
struct class*			shtpsif_class;
struct device*			shtpsif_device;
struct cdev 			shtpsif_cdev;

void shtps_init_io_debugfs(struct shtps_mxt *ts);
void shtps_deinit_io_debugfs(struct shtps_mxt *ts);

/* -----------------------------------------------------------------------------------
 */
/* for sysfs I/F */
#if defined(SHTPS_ENGINEER_BUILD_ENABLE)
	#define SHTPSIF_DEFINE(name, show_func, store_func) \
	static struct kobj_attribute shtpsif_##name = \
		__ATTR(name, (S_IRUGO | S_IWUGO), show_func, store_func)
#else
	#define SHTPSIF_DEFINE(name, show_func, store_func) \
	static struct kobj_attribute shtpsif_##name = \
		__ATTR(name, (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP), show_func, store_func)
#endif /* SHTPS_ENGINEER_BUILD_ENABLE */

#define SHTPSIF_SHOW_COMMON		shtpsif_show_common
#define SHTPSIF_STORE_COMMON	shtpsif_store_common

#define SHTPSIF_LOG_FUNC_CALL()								\
    if((gShtpsIfLog & 0x01) != 0){							\
        printk(KERN_DEBUG TPS_ID1" %s()\n", __func__);	\
    }

int gShtpsIfLog = 0;

/* -----------------------------------------------------------------------------------
 */
static int shtps_ioctl_pwrctrl(unsigned int cmd)
{
	struct shtps_mxt *ts = gShtps_mxt;
	int rc;

	shtps_mutex_lock_ctrl();

	if(SHTPS_STATE_BOOTLOADER == ts->state_mgr.state){
		SHTPS_LOG_ERR_PRINT("disable power control because current state is bootloader mode.\n");
		rc = -1;
	}
	else{
		if(cmd == TPSDEV_ENABLE){
			rc = mxt_power_enable(ts->dev);
		}else{
			rc = mxt_power_disable(ts->dev);
		}
	}

	shtps_mutex_unlock_ctrl();

	return rc;
}

static int shtps_ioctl_reset(void)
{
	struct shtps_mxt *ts = gShtps_mxt;
	int ret;

	shtps_mutex_lock_ctrl();

	if(SHTPS_STATE_BOOTLOADER == ts->state_mgr.state){
		SHTPS_LOG_ERR_PRINT("not reset because current state is bootloader mode.\n");
		ret = -1;
	}
	else{
		ret = mxt_hard_reset(ts->dev);
	}

	shtps_mutex_unlock_ctrl();

	return ret;
}

static int shtps_ioctl_enter_bootloader(unsigned long arg)
{
	struct shtps_mxt *ts = gShtps_mxt;
	int ret;

	shtps_mutex_lock_ctrl();

	if(SHTPS_STATE_BOOTLOADER == ts->state_mgr.state){
		SHTPS_LOG_ERR_PRINT("enter bootloader mode error. already bootloader mode.\n");
		ret = -1;
	}
	else{
		if(arg == TPSDEV_BOOTLOADER_FW){
			ret = mxt_set_fwupdate_mode(ts->dev, 1);
		}
		else if(arg == TPSDEV_BOOTLOADER_CFG){
			ret = mxt_set_cfgupdate_mode(ts->dev, 1);
		}
		else{
			ret = -1;
		}
	}

	shtps_mutex_unlock_ctrl();

	return ret;
}

static int shtps_ioctl_exit_bootloader(unsigned long arg)
{
	struct shtps_mxt *ts = gShtps_mxt;
	int ret;

	shtps_mutex_lock_ctrl();

	if(arg == TPSDEV_BOOTLOADER_FW){
		ret = mxt_set_fwupdate_mode(ts->dev, 0);
	}
	else if(arg == TPSDEV_BOOTLOADER_CFG){
		ret = mxt_set_cfgupdate_mode(ts->dev, 0);
	}
	else{
		ret = -1;
	}

	shtps_mutex_unlock_ctrl();

	return ret;
}

static int shtps_ioctl_get_touchinfo(unsigned long arg)
{
	int ret;
	struct shtps_input_event_info *input_event_info_p = NULL;
	
	ret = shtps_diag_input_event_poll(&input_event_info_p);

	if(!ret){
		if(copy_to_user((u8*)arg, (u8*)input_event_info_p, sizeof(struct shtps_input_event_info))){
			return -EFAULT;
		}
	}

	return ret;
}

static int shtps_ioctl_get_touchinfo_nowait(unsigned long arg)
{
	struct shtps_mxt *ts = gShtps_mxt;
	if(copy_to_user((u8*)arg, (u8*)&ts->input_event_info, sizeof(struct shtps_input_event_info))){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_cancel_get_touchinfo(void)
{
	shtps_diag_input_event_poll_stop_request();
	return 0;
}

static int shtps_ioctl_calibration_param(unsigned long arg)
{
#if defined(SHTPS_POSITION_OFFSET)
	struct shtps_mxt *ts = gShtps_mxt;
	u8 *data;
	struct shtps_ioctl_param param;

	SHTPS_LOG_FUNC_CALL();
	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	data = (u8*)kmalloc(param.size, GFP_KERNEL);
	if(data == NULL){
		return -EINVAL;
	}
	if(0 != copy_from_user(data, param.data, param.size)){
		kfree(data);
		return -EINVAL;
	}

	shtps_filter_set_offset_pos_param(ts, data);
#endif /* SHTPS_POSITION_OFFSET */

	return 0;
}

static int shtps_ioctl_get_smem_baseline(unsigned long arg)
{
	sharp_smem_common_type *smemdata = sh_smem_get_common_address();

	if(!smemdata){
		return -EFAULT;
	}
	
	if(copy_to_user((u8*)arg, (u8*)(smemdata->shdiag_TpsBaseLineTbl),
		SHTPS_TM_RXNUM_MAX * SHTPS_TM_TXNUM_MAX * 2)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_set_lowpower_mode(unsigned int cmd, unsigned long arg)
{
	struct shtps_mxt *ts = gShtps_mxt;
	int ret = 0;
	
	shtps_mutex_lock_ctrl();
	
	if(cmd == TPSDEV_SET_LOWPOWER_MODE){
		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_COMMON, (int)arg);
	}else if(cmd == TPSDEV_SET_CONT_LOWPOWER_MODE){
		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_ECO, (int)arg);
	}else if(cmd == TPSDEV_SET_LCD_LOWPOWER_MODE){
		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_LCD_BRIGHT, (int)arg);
	}else{
		ret = -EFAULT;
	}

	shtps_mutex_unlock_ctrl();

	return ret;
}

static int shtps_ioctl_set_charger_state(unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int shtps_ioctl_lpwg_enable(unsigned long arg)
{
	struct shtps_mxt *ts = gShtps_mxt;

	SHTPS_LOG_FUNC_CALL();

	#if defined( SHTPS_LPWG_MODE_ENABLE )
		shtps_mutex_lock_ctrl();
		ts->lpwg.lpwg_state = arg;
		ts->lpwg.notify_enable = 1;
		SHTPS_LOG_DBG_PRINT(" [LPWG] lpwg_state = %d\n", ts->lpwg.lpwg_state);
		if (SHTPS_STATE_SLEEP == ts->state_mgr.state) {
			u8 new_setting = shtps_is_lpwg_active(ts);
			
			if(new_setting != ts->lpwg.lpwg_switch){
				if(new_setting){
//					#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
//						shtps_irq_enable(ts);
//					#else
//						shtps_irq_wake_enable(ts);
//					#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

//					shtps_system_set_wakeup(ts);
					shtps_set_lpwg_mode_on(ts);
				}else{
//					#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
//						shtps_irq_disable(ts);
//					#else
//						shtps_irq_wake_disable(ts);
//					#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

					shtps_set_lpwg_mode_off(ts);
					shtps_sleep(ts, 1);
//					shtps_system_set_sleep(ts);
				}
			}
		}
		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	return 0;
}

static int shtps_ioctl_set_veilview_state(unsigned long arg)
{
	return 0;
}

static int shtps_ioctl_get_veilview_pattern(unsigned long arg)
{
	return 0;
}

static int shtps_ioctl_log_enable(unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	gLogOutputEnable = (int)arg;
	return 0;
}

static int shtps_ioctl_get_update_flg(void)
{
	return shtps_fwup_flag_check();
}

static int shtps_ioctl_get_hw_type(void)
{
	return shtps_system_get_hw_type();
}

static int shtps_ioctl_get_hw_revision(void)
{
	return shtps_system_get_hw_revision();
}

static long shtpsif_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct shtps_mxt *ts = gShtps_mxt;
	int	rc = 0;

	SHTPS_LOG_DBG_PRINT("%s(cmd:0x%08X(num:%d), arg:%lu)\n", __func__, cmd, (cmd >> _IOC_NRSHIFT) & _IOC_NRMASK, arg);

	if (ts->dev == NULL){
		return -EFAULT;
	}

	switch(cmd){
		case TPSDEV_ENABLE:
		case TPSDEV_DISABLE:
			rc = shtps_ioctl_pwrctrl(cmd);
			break;
		case TPSDEV_RESET:
			rc = shtps_ioctl_reset();
			break;

		case TPSDEV_ENTER_BOOTLOADER:
			rc = shtps_ioctl_enter_bootloader(arg);
			break;

		case TPSDEV_EXIT_BOOTLOADER:
			rc = shtps_ioctl_exit_bootloader(arg);
			break;

		case TPSDEV_GET_TOUCHINFO:
			rc = shtps_ioctl_get_touchinfo(arg);
			break;

		case TPSDEV_GET_TOUCHINFO_NOWAIT:
			rc = shtps_ioctl_get_touchinfo_nowait(arg);
			break;

		case TPSDEV_CANCEL_GET_TOUCHINFO:
			rc = shtps_ioctl_cancel_get_touchinfo();
			break;

		case TPSDEV_CALIBRATION_PARAM:
			rc = shtps_ioctl_calibration_param(arg);
			break;
			
		case TPSDEV_GET_SMEM_BASELINE:
			rc = shtps_ioctl_get_smem_baseline(arg);
			break;

		case TPSDEV_SET_LOWPOWER_MODE:
		case TPSDEV_SET_CONT_LOWPOWER_MODE:
		case TPSDEV_SET_LCD_LOWPOWER_MODE:
			rc = shtps_ioctl_set_lowpower_mode(cmd, arg);
			break;
		
		case TPSDEV_SET_CHARGER_ARMOR:
		case TPSDEV_SET_WIRELESS_CHARGER_ARMOR:
			rc = shtps_ioctl_set_charger_state(cmd, arg);
			break;

		case TPSDEV_LPWG_ENABLE:
			rc = shtps_ioctl_lpwg_enable(arg);
			break;
			
		case TPSDEV_SET_VEILVIEW_STATE:
			rc = shtps_ioctl_set_veilview_state(arg);
			break;
			
		case TPSDEV_GET_VEILVIEW_PATTERN:
			rc = shtps_ioctl_get_veilview_pattern(arg);
			break;
			
		case TPSDEV_LOGOUTPUT_ENABLE:
			rc = shtps_ioctl_log_enable(arg);
			break;
		
		case TPSDEV_GET_UPDATE_FLG:
			rc = shtps_ioctl_get_update_flg();
			break;

		case TPSDEV_GET_HW_TYPE:
			rc = shtps_ioctl_get_hw_type();
			break;

		case TPSDEV_GET_HW_REVISION:
			rc = shtps_ioctl_get_hw_revision();
			break;

		case TPSDEV_GET_TOUCHKEYINFO:
		case TPSDEV_SET_NARROW_FRAME_MODE:
		case TPSDEV_SET_PEN_ENABLE:
		case TPSDEV_GET_PEN_ENABLE:
		case TPSDEV_CALIBRATION_PEN_PARAM:
		case TPSDEV_HOVER_ENABLE:
		case TPSDEV_START_FACETOUCHMODE:
		case TPSDEV_STOP_FACETOUCHMODE:
		case TPSDEV_POLL_FACETOUCHOFF:
		case TPSDEV_ACK_FACETOUCHOFF:
			return -EFAULT;

		default:
			break;
	}
	return rc;
}

static int shtpsif_open(struct inode *inode, struct file *file)
{
	SHTPS_LOG_DBG_PRINT("%s(PID:%ld)\n", __func__, sys_getpid());
	return 0;
}

static int shtpsif_release(struct inode *inode, struct file *file)
{
	SHTPS_LOG_DBG_PRINT("%s(PID:%ld)\n", __func__, sys_getpid());
	return 0;
}

static ssize_t shtpsif_read(struct file *file, char *buf, size_t count, loff_t *pos)
{
	return 0;
}

static unsigned int shtpsif_poll(struct file *file, poll_table *wait)
{
	return 0;
}

static const struct file_operations shtpsif_fileops = {
	.owner   = THIS_MODULE,
	.open    = shtpsif_open,
	.release = shtpsif_release,
	.read    = shtpsif_read,
	.poll    = shtpsif_poll,
	.unlocked_ioctl   = shtpsif_ioctl,
};

int __init shtpsif_init(void)
{
	struct shtps_mxt *ts = gShtps_mxt;
	int rc;
	
	rc = alloc_chrdev_region(&shtpsif_devid, 0, 1, SH_TOUCH_IF_DEVNAME);
	if(rc < 0){
		SHTPS_LOG_ERR_PRINT("shtpsif:alloc_chrdev_region error\n");
		return rc;
	}
	
	shtpsif_class = class_create(THIS_MODULE, SH_TOUCH_IF_DEVNAME);
	if (IS_ERR(shtpsif_class)) {
		rc = PTR_ERR(shtpsif_class);
		SHTPS_LOG_ERR_PRINT("shtpsif:class_create error\n");
		goto error_vid_class_create;
	}
	
	shtpsif_device = device_create(shtpsif_class, NULL,
								shtpsif_devid, &shtpsif_cdev,
								SH_TOUCH_IF_DEVNAME);
	
	if (IS_ERR(shtpsif_device)) {
		rc = PTR_ERR(shtpsif_device);
		SHTPS_LOG_ERR_PRINT("shtpsif:device_create error\n");
		goto error_vid_class_device_create;
	}

	cdev_init(&shtpsif_cdev, &shtpsif_fileops);
	shtpsif_cdev.owner = THIS_MODULE;
	rc = cdev_add(&shtpsif_cdev, shtpsif_devid, 1);
	if(rc < 0){
		SHTPS_LOG_ERR_PRINT("shtpsif:cdev_add error\n");
		goto err_via_cdev_add;
	}

	/** init sysfs I/F */
	shtps_init_io_debugfs(ts);

	#if defined( SHTPS_DEVELOP_MODE_ENABLE )
	{
		struct shtps_debug_init_param param;
		param.shtps_root_kobj = ts->kobj;

		/** init debug function data */
		shtps_debug_init(&param);
	}
	#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */

	return 0;

err_via_cdev_add:
	cdev_del(&shtpsif_cdev);
error_vid_class_device_create:
	class_destroy(shtpsif_class);
error_vid_class_create:
	unregister_chrdev_region(shtpsif_devid, 1);
	
	return rc;
}
module_init(shtpsif_init);

static void __exit shtpsif_exit(void)
{
	struct shtps_mxt *ts = gShtps_mxt;

	shtps_deinit_io_debugfs(ts);

	cdev_del(&shtpsif_cdev);
	class_destroy(shtpsif_class);
	unregister_chrdev_region(shtpsif_devid, 1);

	SHTPS_LOG_DBG_PRINT("%s() done\n", __func__);
}
module_exit(shtpsif_exit);

/* -----------------------------------------------------------------------------------
 */
static int shtps_numStrToList(
	const char	*numStr,		/* [I  ] num strings */
	int			*numList,		/* [I/O] num list */
	int			numListMaxSize	/* [I/ ] num list size */
)
{
	int			i;
	int			numListNum;
	int			isParam;
	char		buf[10];
	int			buf_current;
	int			num;
	int			rc;

	if((numStr == NULL) || (numList == NULL) || (numListMaxSize < 1)){
		return 0;
	}

	numListNum = 0;
	isParam = 0;
	buf_current = 0;

	for(i = 0; i < PAGE_SIZE; i++){
		if((numStr[i] == '\0') || (numStr[i] == '\n') || (numStr[i] == ',') || (numStr[i] == ' ') || (numStr[i] == '\t')){
			if(isParam == 1){
				buf[buf_current] = '\0';
				rc = kstrtoint(buf, 0, &num);
				if(rc == 0){
					numList[numListNum] = num;
				}
				else{
					/* Conversion failure */
					return 0;
				}
				numListNum++;
			}
			isParam = 0;
			if(numListNum >= numListMaxSize){
				break;
			}
			if( (numStr[i] == '\0') ){
				break;
			}
		}
		else{
			if(isParam == 0){
				isParam = 1;
				buf_current = 0;
			}
			buf[buf_current] = numStr[i];
			buf_current++;
			if(buf_current >= sizeof(buf)){
				/* Conversion failure */
				return 0;
			}
		}
	}

	return numListNum;
}
/* -----------------------------------------------------------------------------------
 */
static ssize_t shtpsif_show_common(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return -EINVAL;
}

static ssize_t shtpsif_store_common(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return -EINVAL;
}
/* -----------------------------------------------------------------------------------
 */
static ssize_t shtpsif_show_shtpsiflog(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", gShtpsIfLog);
}
static ssize_t shtpsif_store_shtpsiflog(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int argc;
	int argv[1];

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc >= 1){
		gShtpsIfLog = argv[0];
	}
	else{
		return -EINVAL;
	}

	return count;
}
SHTPSIF_DEFINE(shtpsiflog, shtpsif_show_shtpsiflog, shtpsif_store_shtpsiflog);

static ssize_t shtpsif_store_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;

	SHTPSIF_LOG_FUNC_CALL();

	ret = shtps_ioctl_pwrctrl(TPSDEV_ENABLE);

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(enable, SHTPSIF_SHOW_COMMON, shtpsif_store_enable);

static ssize_t shtpsif_store_disable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;

	SHTPSIF_LOG_FUNC_CALL();

	ret = shtps_ioctl_pwrctrl(TPSDEV_DISABLE);

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(disable, SHTPSIF_SHOW_COMMON, shtpsif_store_disable);

static ssize_t shtpsif_store_reset(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;

	SHTPSIF_LOG_FUNC_CALL();

	ret = shtps_ioctl_reset();

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(reset, SHTPSIF_SHOW_COMMON, shtpsif_store_reset);

static ssize_t shtpsif_store_enter_bootloader(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	ret = shtps_ioctl_enter_bootloader(argv[0]);

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(enter_bootloader, SHTPSIF_SHOW_COMMON, shtpsif_store_enter_bootloader);

static ssize_t shtpsif_store_exit_bootloader(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	ret = shtps_ioctl_exit_bootloader(argv[0]);

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(exit_bootloader, SHTPSIF_SHOW_COMMON, shtpsif_store_exit_bootloader);

static ssize_t shtpsif_show_hw_type(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", shtps_ioctl_get_hw_type());
}
SHTPSIF_DEFINE(hw_type, shtpsif_show_hw_type, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_hw_revision(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", shtps_ioctl_get_hw_revision());
}
SHTPSIF_DEFINE(hw_revision, shtpsif_show_hw_revision, SHTPSIF_STORE_COMMON);

#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
static ssize_t shtpsif_show_log_enable(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", gLogOutputEnable);
}
static ssize_t shtpsif_store_log_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	gLogOutputEnable = argv[0];

	return count;
}
SHTPSIF_DEFINE(log_enable, shtpsif_show_log_enable, shtpsif_store_log_enable);
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */

static struct attribute *attrs_shtpsif[] = {
	&shtpsif_shtpsiflog.attr,
	&shtpsif_enable.attr,
	&shtpsif_disable.attr,
	&shtpsif_reset.attr,
	&shtpsif_enter_bootloader.attr,
	&shtpsif_exit_bootloader.attr,
	&shtpsif_hw_type.attr,
	&shtpsif_hw_revision.attr,
	#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
		&shtpsif_log_enable.attr,
	#endif /* SHTPS_LOG_OUTPUT_SWITCH_ENABLE */
	NULL
};

static struct attribute_group shtps_attr_grp_shtpsif = {
	.name = "shtpsif",
	.attrs = attrs_shtpsif,
};

/*  -----------------------------------------------------------------------------------
 */
void shtps_init_io_debugfs(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();

	ts->kobj = kobject_create_and_add("shtps", kernel_kobj);
	if(ts->kobj == NULL){
		SHTPS_LOG_ERR_PRINT("kobj create failed : shtps\n");
	}else{
		if(sysfs_create_group(ts->kobj, &shtps_attr_grp_shtpsif)){
			SHTPS_LOG_ERR_PRINT("kobj create failed : shtps_attr_grp_shtpsif\n");
		}
	}
}

void shtps_deinit_io_debugfs(struct shtps_mxt *ts)
{
	SHTPS_LOG_FUNC_CALL();
	if(ts->kobj != NULL){
		sysfs_remove_group(ts->kobj, &shtps_attr_grp_shtpsif);
		kobject_put(ts->kobj);
	}
}
