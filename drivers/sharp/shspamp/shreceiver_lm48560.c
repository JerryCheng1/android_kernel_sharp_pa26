/* drivers/sharp/shreceiver/shreceiver_lm48560.c
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
/* CONFIG_SH_AUDIO_DRIVER newly created */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <sharp/shspamp.h>
#include <linux/of_gpio.h>
#ifdef SHRECEIVER_ENG
#include <linux/debugfs.h>
#endif
#ifdef CONFIG_SH_AUDIO_DRIVER /* 09-101 */
#include <sharp/shspamp.h>
#include <sharp/sh_smem.h>
#endif /* CONFIG_SH_AUDIO_DRIVER */ /* 09-101 */

static struct i2c_client *this_client;
#ifdef SHRECEIVER_ENG
static struct dentry *shreceiver_dbgfile;
#endif

static DEFINE_MUTEX(receiver_amp_lock);
static int shreceiver_opened;
static int shreceiver_en = -1;
#ifdef CONFIG_SH_AUDIO_DRIVER /* 09-101 */
static int hw_revision = 0;
#endif /* CONFIG_SH_AUDIO_DRIVER */ /* 09-101 */

#ifdef CONFIG_SH_AUDIO_DRIVER /* 09-101 */
static bool getHWRevision(void)
{
	sharp_smem_common_type *sharp_smem;
	sharp_smem = sh_smem_get_common_address();

	if( sharp_smem != 0 ) {
		hw_revision = sharp_smem->sh_hw_revision;
		pr_err("%s() The current revision is %d", __func__, hw_revision);
		return true;
	}

	pr_err("%s() Can't check HW revision\n", __func__);
	return false;
}
#endif /* CONFIG_SH_AUDIO_DRIVER */ /* 09-101 */

#ifdef SHRECEIVER_ENG
/*
 * For now, we use it just in debugfs, so add define SHRECEIVER_ENG for compile error.
 * If you'll use in other way, please remove SHRECEIVER_ENG.
 *
 */

static int shreceiver_i2c_read(u8 *buf, int len)
{
    int ret = 0;

    struct i2c_msg msgs[2] = {
        {
            .flags  = !I2C_M_RD,
            .addr   = this_client->addr,
            .len    = I2C_ADDR_LENGTH,
            .buf    = &buf[0],
        },
        {
            .flags  = I2C_M_RD,
            .addr   = this_client->addr,
            .len    = len-I2C_ADDR_LENGTH,
            .buf    = &buf[I2C_ADDR_LENGTH],
        },
    };
    ret = i2c_transfer(this_client->adapter, msgs, 2);
    if (ret < 0)
        dev_err(&this_client->dev, "%s: i2c read error.\n",
            __func__);

    return ret;
}
#endif

static int shreceiver_i2c_write(int reg, u8 data)
{
    int rc;
    u8 buf[2];
    struct i2c_msg msg[] = {
    {
        .addr = this_client->addr,
        .flags= 0,
        .len  = 2,
        .buf  = buf,
        },
    };

    buf[0] = reg;
    buf[1] = data;
    rc = i2c_transfer(this_client->adapter, msg, 1);
    if(rc != 1){
        dev_err(&this_client->dev,
                "shreceiver_i2c_write FAILED: writing to reg %d\n", reg);
        rc = -1;
    }
    return rc;
}

static int shreceiver_open(struct inode *inode, struct file *file)
{
    int rc = 0;
    pr_debug("{shreceiver} %s\n", __func__);

    mutex_lock(&receiver_amp_lock);

    if (shreceiver_opened) {
        pr_err("%s: busy\n", __func__);
        rc = -EBUSY;
        goto done;
    }

    shreceiver_opened = 1;
done:
    mutex_unlock(&receiver_amp_lock);
    return rc;
}

static int shreceiver_release(struct inode *inode, struct file *file)
{

    pr_debug("{shreceiver} %s\n", __func__);

    mutex_lock(&receiver_amp_lock);
    shreceiver_opened = 0;
    mutex_unlock(&receiver_amp_lock);
    return 0;
}

static struct file_operations shreceiver_fops = {
    .owner   = THIS_MODULE,
    .open    = shreceiver_open,
    .release = shreceiver_release,
};

static struct miscdevice shreceiver_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "shreceiver",
    .fops = &shreceiver_fops,
};

void shreceiver_set_receiver(int on){
    static int is_on;

    pr_debug("%s: \n", __func__);

    mutex_lock(&receiver_amp_lock);
    if (on && !is_on) {
        msleep(30);

        gpio_direction_output(shreceiver_en, 1);
        shreceiver_i2c_write(0x00, 0x07);   /* [TURN_ON] Normal turn on time, tWU = 15ms */
                                            /* [IN_SEL]  Input 2 selected */
                                            /* [BOOST_EN]Boost enabled */
                                            /* [SHDN] Device enabled */
        if (204==CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER) {
            shreceiver_i2c_write(0x01, 0x02);   /* [PLEV2 (B2) PLEV1 (B1)PLEV0 (B0)] VTH(VLIM) = 17VP-P */
        } else {
            shreceiver_i2c_write(0x01, 0x06);   /* [PLEV2 (B2) PLEV1 (B1)PLEV0 (B0)] VTH(VLIM) = 28VP-P */
        }
        shreceiver_i2c_write(0x02, 0x01);   /* [GAIN1(B1) GAIN0 (B0)] 24dB */

        is_on = 1;

        pr_debug("%s: ON\n", __func__);
    } else if (!on && is_on){
        if (is_on) {
            shreceiver_i2c_write(0x00, 0x00);   /* [SHDN] Device shutdown */

            gpio_direction_output(shreceiver_en, 0);
            is_on = 0;
            msleep(10);
            pr_debug("%s: OFF\n", __func__);
        }
    }
    mutex_unlock(&receiver_amp_lock);

    return ;
}

#ifdef SHRECEIVER_ENG
static int shreceiver_debug_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}

static ssize_t shreceiver_reg_read_file(struct file *file, char __user *user_buf,
				   size_t count, loff_t *pos)
{
    int i, n = 0;
    char buf[512];
    char data[2] = {0x00, 0x55};
    for (i=0; i<=3; i++) {
        data[0] = i;
        data[1] = 0x55;
        if (shreceiver_i2c_read(data, 2) < 0)
            n += sprintf(buf+n, "Read error\n");
        else
            n += sprintf(buf+n, "0x%x: 0x%x\n", data[0], data[1]);
    }
    buf[n] = 0;

    return simple_read_from_buffer(user_buf, count, pos, buf, n);;
}

static ssize_t shreceiver_reg_write_file(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
    char buf[32];
    size_t buf_size;
    char *start = buf;
    unsigned long reg, value;

    buf_size = min(count, (sizeof(buf)-1));
    if (copy_from_user(buf, user_buf, buf_size))
        return -EFAULT;

    buf[buf_size] = '\0';
    pr_debug("%s buf:%s", __func__, buf);

    while (*start == ' ')
        start++;
    reg = simple_strtoul(start, &start, 16);
    while (*start == ' ')
        start++;
    if (strict_strtoul(start, 16, &value))
        return -EINVAL;

    pr_debug("%s reg:%lx value:%lx\n", __func__, reg, value);

    shreceiver_i2c_write(reg, value);

    return buf_size;
}
static const struct file_operations shreceiver_dbg_ops = {
    .open = shreceiver_debug_open,
    .read = shreceiver_reg_read_file,
    .write = shreceiver_reg_write_file,
};

static void shreceiver_init_debugfs(void)
{
    shreceiver_dbgfile = debugfs_create_file("shreceiver", S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
                          NULL, NULL, &shreceiver_dbg_ops);
    if (shreceiver_dbgfile == NULL || IS_ERR(shreceiver_dbgfile)) {
        pr_err("%s debugfs_create_file failed: ret=%ld\n", __func__, PTR_ERR(shreceiver_dbgfile));
    }
}

static void shreceiver_cleanup_debugfs(void)
{
    debugfs_remove(shreceiver_dbgfile);
}
#else
static void shreceiver_init_debugfs(void)
{
}

static void shreceiver_cleanup_debugfs(void)
{
}
#endif

static int shreceiver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    pr_debug("{shreceiver} %s\n", __func__);

#ifdef CONFIG_SH_AUDIO_DRIVER /* 09-101 */
	getHWRevision();
	switch(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER) {
	case 204:
		break;
	case 105:
	case 106:
		if (hw_revision == 0 || hw_revision == 2)
			break;
	case 305:
	default:
		pr_err("%s: This HW does not support PR", __func__);
		return ENODEV;
	}
#endif /* CONFIG_SH_AUDIO_DRIVER */ /* 09-101 */

    this_client = client;

    ret = misc_register(&shreceiver_device);
    if (ret) {
        pr_err("%s: shreceiver_device register failed\n", __func__);
    }

    shreceiver_en = of_get_named_gpio(client->dev.of_node,
        "sharp,shreceiver-en-gpio", 0);

    if (shreceiver_en < -1) {
        pr_err("shreceiver_en = %d\n", shreceiver_en);
    } else {
        pr_debug("shreceiver_en = %d\n", shreceiver_en);
    }

    ret = gpio_request(shreceiver_en, "shreceiver");
    if (ret) {
        pr_err("%s: Failed to request gpio %d\n", __func__,
            shreceiver_en);
        goto err_free_gpio;
    }
    gpio_direction_output(shreceiver_en, 0);

    shreceiver_init_debugfs();

    return 0;

err_free_gpio:
    gpio_free(shreceiver_en);
    shreceiver_en = -1;
    return ret;
}

static int shreceiver_remove(struct i2c_client *client)
{
    this_client = i2c_get_clientdata(client);
    shreceiver_cleanup_debugfs();
    return 0;
}

static const struct i2c_device_id shreceiver_id[] = {
    { SHRECEIVER_I2C_NAME, 0 },
    { }
};

static const struct of_device_id mdss_dsi_ctrl_dt_match[] = {
    {.compatible = "sharp,shreceiver_i2c"},
    {}
};
MODULE_DEVICE_TABLE(of, mdss_dsi_ctrl_dt_match);

static struct i2c_driver shreceiver_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name  = SHRECEIVER_I2C_NAME,
		.of_match_table = mdss_dsi_ctrl_dt_match,
    },
    .class    = I2C_CLASS_HWMON,
    .probe    = shreceiver_probe,
    .id_table = shreceiver_id,
    .remove   = shreceiver_remove,
};

static int __init shreceiver_init(void)
{
    pr_debug("{shreceiver} %s\n", __func__);
    return i2c_add_driver(&shreceiver_driver);
}

module_init(shreceiver_init);

MODULE_DESCRIPTION("shreceiver driver");
MODULE_LICENSE("GPL");
