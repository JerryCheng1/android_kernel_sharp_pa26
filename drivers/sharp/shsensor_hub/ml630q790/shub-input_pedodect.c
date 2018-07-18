/*
 *  shub-input_pedodect.c - Linux kernel modules for interface of ML630Q790
 *
 *  Copyright (C) 2014 LAPIS SEMICONDUCTOR CO., LTD.
 *
 *  This file is based on :
 *    alps-input.c - Linux kernel modules for interface of ML610Q792
 *    http://android-dev.kyocera.co.jp/source/versionSelect_URBANO.html
 *    (C) 2012 KYOCERA Corporation
 *    Copyright (C) 2010 ALPS
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <asm/uaccess.h> 

#include "shub_io.h"
#include "ml630q790.h"

// SHMDS_HUB_0701_01 add S
#ifdef CONFIG_ANDROID_ENGINEERING
static int shub_pedodet_log = 0;
module_param(shub_pedodet_log, int, 0600);
#define DBG_PEDODET_IO(msg, ...) {                      \
    if(shub_pedodet_log & 0x01)                         \
        printk("[shub][pedodet] " msg, ##__VA_ARGS__);  \
}
#define DBG_PEDODET_DATA(msg, ...) {                    \
    if(shub_pedodet_log & 0x02)                         \
        printk("[shub][pedodet] " msg, ##__VA_ARGS__);  \
}
#else
#define DBG_PEDODET_IO(msg, ...)
#define DBG_PEDODET_DATA(msg, ...)
#endif
// SHMDS_HUB_0701_01 add E

#define PEDOM_MAX 1
#define PEDOM_MIN 0

#define INDEX_STPS         0
#define INDEX_TM           1
#define INDEX_TMNS         2
#define INDEX_SUM          3
#define INPUT_DEV_NAME "shub_pedodet"
#define INPUT_DEV_PHYS "shub_pedodet/input0"
#define MISC_DEV_NAME  "shub_io_pedodet"
#define SHUB_EVENTS_PER_PACKET ((LOGGING_RAM_SIZE/(DATA_SIZE_PEDOCNT+1))*2) /*+1 is ID size*/

static DEFINE_MUTEX(shub_lock);

static struct platform_device *pdev;
static struct input_dev *shub_idev;
static int32_t        power_state     = 0;
static IoCtlBatchInfo batch_param     = { 0, 0, 0 };
static int32_t currentActive;

extern int32_t setMaxBatchReportLatency(uint32_t sensor, int64_t latency);

static long shub_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1, tmpval = 0;
    switch (cmd) {
        case SHUBIO_PEDODET_ACTIVATE:
            ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
            if (ret) {
                printk("error : shub_ioctl(cmd = SHUBIO_PEDODET_ACTIVATE)\n");
                return -EFAULT;
            }
            DBG_PEDODET_IO("ioctl(cmd = Set_Active) : val=%d\n", tmpval); // SHMDS_HUB_0701_01 add
            mutex_lock(&shub_lock);
            currentActive = shub_get_current_active() & SHUB_ACTIVE_PEDODEC;

            if((batch_param.m_Latency != 0) && (currentActive == 0)){
                //polling off and batch enable
                if(tmpval != 0){
                    //batch start/stop
                    if(batch_param.m_Latency > 0){
                        ret = shub_activate_logging(SHUB_ACTIVE_PEDODEC_NO_NOTIFY, 1);
                    }else{
                        ret = shub_activate_logging(SHUB_ACTIVE_PEDODEC_NO_NOTIFY, 0);
                    }
                }else{
                    //batch stop
                    ret = shub_activate_logging(SHUB_ACTIVE_PEDODEC_NO_NOTIFY, 0);
                    setMaxBatchReportLatency(SHUB_ACTIVE_PEDODEC, 0);
                }
            }else{
                //set mcu sensor measure
                ret = shub_activate( SHUB_ACTIVE_PEDODEC, tmpval);

                //batch stop 
                ret = shub_activate_logging(SHUB_ACTIVE_PEDODEC_NO_NOTIFY, 0);
                setMaxBatchReportLatency(SHUB_ACTIVE_PEDODEC, 0);
            }
            if(ret != -1){
                power_state = tmpval;
            }else{
                mutex_unlock(&shub_lock);
                return -EFAULT;
            }
            mutex_unlock(&shub_lock);
            break;

        case SHUBIO_PEDODET_SET_BATCH :
            {
                IoCtlBatchInfo param;
                uint64_t delayNs;
                int32_t delay;
                ret = copy_from_user(&param, argp, sizeof(param));
                if (ret) {
                    printk( "error(copy_from_user) : shub_ioctl(cmd = SHUBIO_PEDODET_SET_BATCH)\n" );
                    return -EFAULT;
                }
                DBG_PEDODET_IO("ioctl(cmd = Set_Batch) : flg=%d, Period=%lld, Latency=%lld\n", param.m_Flasg, param.m_PeriodNs, param.m_Latency); // SHMDS_HUB_0701_01 add
                if((param.m_Flasg & 0x01) == 0x01){
                    return 0;
                }
                mutex_lock(&shub_lock);
                delayNs = param.m_PeriodNs;
                delay = (int32_t)do_div(delayNs, 1000000);
                delay = (int32_t)delayNs;
                delay = (delay > SHUB_TIMER_MAX) ? SHUB_TIMER_MAX : delay;
                if(power_state != 0){
                    //poll on -> batch on
                    if((batch_param.m_Latency == 0) && (param.m_Latency > 0))
                    {
                        batch_param.m_Flasg    = param.m_Flasg;
                        batch_param.m_PeriodNs = param.m_PeriodNs;
                        batch_param.m_Latency  = param.m_Latency;
                        //disable poll 
                        shub_set_delay(SHUB_ACTIVE_PEDODEC, delay);
                        ret = shub_activate( SHUB_ACTIVE_PEDODEC, 0);
                        currentActive = shub_get_current_active() & SHUB_ACTIVE_PEDODEC;
                        if(ret == -1){
                            mutex_unlock(&shub_lock);
                            return -EFAULT;
                        }

                        //enable batch
                        setMaxBatchReportLatency(SHUB_ACTIVE_PEDODEC, batch_param.m_Latency);
                        shub_set_delay_logging(SHUB_ACTIVE_PEDODEC_NO_NOTIFY, batch_param.m_PeriodNs);
                        ret = shub_activate_logging(SHUB_ACTIVE_PEDODEC_NO_NOTIFY, 1);
                        if(ret == -1){
                            mutex_unlock(&shub_lock);
                            return -EFAULT;
                        }

                        mutex_unlock(&shub_lock);
                        return 0;
                    }
                    //batch on -> poll on
                    if((batch_param.m_Latency > 0) && (param.m_Latency == 0))
                    {
                        batch_param.m_Flasg    = param.m_Flasg;
                        batch_param.m_PeriodNs = param.m_PeriodNs;
                        batch_param.m_Latency  = param.m_Latency;

                        //disable batch
                        setMaxBatchReportLatency(SHUB_ACTIVE_PEDODEC, batch_param.m_Latency);
                        shub_set_delay_logging(SHUB_ACTIVE_PEDODEC_NO_NOTIFY, batch_param.m_PeriodNs);
                        ret = shub_activate_logging(SHUB_ACTIVE_PEDODEC_NO_NOTIFY, 0);
                        if(ret == -1){
                            mutex_unlock(&shub_lock);
                            return -EFAULT;
                        }

                        //enable poll
                        shub_set_delay(SHUB_ACTIVE_PEDODEC, delay);
                        ret = shub_activate( SHUB_ACTIVE_PEDODEC, 1);
                        currentActive = shub_get_current_active() & SHUB_ACTIVE_PEDODEC;
                        if(ret == -1){
                            mutex_unlock(&shub_lock);
                            return -EFAULT;
                        }


                        mutex_unlock(&shub_lock);
                        return 0;
                    }
                }
                /* flag SENSORS_BATCH_DRY_RUN is OFF */
                batch_param.m_Flasg    = param.m_Flasg;
                batch_param.m_PeriodNs = param.m_PeriodNs;
                batch_param.m_Latency  = param.m_Latency;

                if(param.m_Latency == 0){
                    shub_set_delay(SHUB_ACTIVE_PEDODEC, delay);
                }else{
                    shub_set_delay_logging(SHUB_ACTIVE_PEDODEC_NO_NOTIFY, batch_param.m_PeriodNs);
                }
                setMaxBatchReportLatency(SHUB_ACTIVE_PEDODEC, batch_param.m_Latency);
                mutex_unlock(&shub_lock);
            }
            break;

        case SHUBIO_PEDODET_FLUSH :
            {
                DBG_PEDODET_IO("ioctl(cmd = Flush)\n"); // SHMDS_HUB_0701_01 add
                mutex_lock(&shub_lock);
                if(power_state != 0){
                    shub_logging_flush();
                    setMaxBatchReportLatency(SHUB_ACTIVE_PEDODEC, batch_param.m_Latency);
                    shub_idev->sync = 0;
// SHMDS_HUB_0701_05 mod S
//                  input_event(shub_idev, EV_SYN, SYN_REPORT, 2);
                    input_event(shub_idev, EV_SYN, SYN_REPORT, (SHUB_INPUT_META_DATA | SHUB_INPUT_PEDODET));
// SHMDS_HUB_0701_05 mod E

                }else{
                    mutex_unlock(&shub_lock);
                    return -EFAULT;
                }
                mutex_unlock(&shub_lock);
            }
            break;

        default:
            return -ENOTTY;
    }
    return 0;
}

// SHMDS_HUB_1101_01 add S
static long shub_ioctl_wrapper(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    shub_qos_start();
    ret = shub_ioctl(filp, cmd , arg);
    shub_qos_end();

    return ret;
}
// SHMDS_HUB_1101_01 add E

void shub_input_report_stepdetect(int32_t *data)
{
    if(data == NULL) {
        return;
    }

// SHMDS_HUB_0701_01 add S
    DBG_PEDODET_DATA("data X=%d, t(s)=%d, t(ns)=%d\n", data[INDEX_STPS],data[INDEX_TM],data[INDEX_TMNS]);
// SHMDS_HUB_0701_01 add E

#if 1  // SHMDS_HUB_0601_01 mod S
    input_report_abs(shub_idev, ABS_Z, data[INDEX_STPS]);
    input_report_abs(shub_idev, ABS_HAT2Y, data[INDEX_TM]);
    input_report_abs(shub_idev, ABS_HAT2X, data[INDEX_TMNS]);
    shub_idev->sync = 0;
    input_event(shub_idev, EV_SYN, SYN_REPORT, SHUB_INPUT_PEDODET);
#else
    input_report_abs(shub_idev, ABS_X, data[INDEX_STPS]);
    input_report_abs(shub_idev, ABS_MISC, data[INDEX_TM]);
    input_report_abs(shub_idev, ABS_VOLUME, data[INDEX_TMNS]);
    shub_idev->sync = 0;
    input_event(shub_idev, EV_SYN, SYN_REPORT, 1);
#endif // SHMDS_HUB_0601_01 mod E
}

#if 0  // SHMDS_HUB_0601_01 del S
static void shub_set_abs_params(void)
{
    input_set_abs_params(shub_idev, ABS_MISC, 0, 0xFFFFFFFF, 0, 0);
    input_set_abs_params(shub_idev, ABS_VOLUME, 0, 0xFFFFFFFF, 0, 0);
    input_set_abs_params(shub_idev, ABS_X, PEDOM_MIN, PEDOM_MAX, 0, 0);
}
#endif // SHMDS_HUB_0601_01 del E

// SHMDS_HUB_1101_01 mod S
static struct file_operations shub_fops = {
    .owner   = THIS_MODULE,
    .unlocked_ioctl = shub_ioctl_wrapper,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = shub_ioctl_wrapper,
#endif
};
// SHMDS_HUB_1101_01 mod E

static struct miscdevice shub_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = MISC_DEV_NAME,
    .fops  = &shub_fops,
};

static int32_t __init shub_init(void)
{
    int32_t ret = 0;


    if(!shub_connect_check()){
        printk(KERN_INFO "shub_stepdec Connect Error!!\n");
        ret = -ENODEV;
        goto out_driver;
    }


    pdev = platform_device_register_simple(INPUT_DEV_NAME, -1, NULL, 0);
    if (IS_ERR(pdev)) {
        ret = PTR_ERR(pdev);
        goto out_driver;
    }

#if 1  // SHMDS_HUB_0601_01 mod S
    shub_idev = shub_com_allocate_device(SHUB_INPUT_PEDODET, &pdev->dev);
    if (!shub_idev) {
        ret = -ENOMEM;
        goto out_device;
    }
#else
    shub_idev = input_allocate_device();
    if (!shub_idev) {
        ret = -ENOMEM;
        goto out_device;
    }

    shub_idev->name = INPUT_DEV_NAME;
    shub_idev->phys = INPUT_DEV_PHYS;
    shub_idev->id.bustype = BUS_HOST;
    shub_idev->dev.parent = &pdev->dev;
    shub_idev->evbit[0] = BIT_MASK(EV_ABS);

    shub_set_abs_params();
    input_set_events_per_packet(shub_idev, SHUB_EVENTS_PER_PACKET);

    ret = input_register_device(shub_idev);
    if (ret)
        goto out_idev;
#endif // SHMDS_HUB_0601_01 mod E

    ret = misc_register(&shub_device);
    if (ret) {
        printk("shub-init: shub_io_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    return 0;

exit_misc_device_register_failed:
#if 0  // SHMDS_HUB_0601_01 del S
out_idev:
    input_free_device(shub_idev);
#endif // SHMDS_HUB_0601_01 del E
out_device:
    platform_device_unregister(pdev);
out_driver:
    return ret;
}

static void __exit shub_exit(void)
{
    misc_deregister(&shub_device);
#if 1  // SHMDS_HUB_0601_01 mod S
    shub_com_unregister_device(SHUB_INPUT_PEDODET);
#else
    input_unregister_device(shub_idev);
    input_free_device(shub_idev);
#endif // SHMDS_HUB_0601_01 mod E
    platform_device_unregister(pdev);
}

module_init(shub_init);
module_exit(shub_exit);

MODULE_DESCRIPTION("SensorHub Input Device (StepDetector)");
MODULE_AUTHOR("LAPIS SEMICOMDUCTOR");
MODULE_LICENSE("GPL v2");
