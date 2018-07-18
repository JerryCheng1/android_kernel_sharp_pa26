/* include/sharp/shspamp.h - shspamp speaker amplifier driver
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

#ifndef __ASM_ARM_ARCH_SHSPAMP_H
#define __ASM_ARM_ARCH_SHSPAMP_H

struct shspamp_platform_data {
	uint32_t gpio_shspamp_spk_en;
};

struct shspamp_config_data {
	unsigned char *cmd_data;  /* [mode][cmd_len][cmds..] */
	unsigned int mode_num;
	unsigned int data_len;
};

extern void shspamp_set_speaker_amp(int on);

#ifdef CONFIG_SHRECEIVER_LM48560
#define SHRECEIVER_I2C_NAME "shreceiver_i2c"
#define SHRECEIVER_CMD_LEN 8
#define I2C_ADDR_LENGTH 1

struct shreceiver_platform_data {
 uint32_t gpio_shreceiver_receiver_en;
};

struct shreceiver_config_data {
 unsigned char *cmd_data;  /* [mode][cmd_len][cmds..] */
 unsigned int mode_num;
 unsigned int data_len;
};
/* Bone Conduction Receiver control */
extern void shreceiver_set_receiver(int on);
#endif /* CONFIG_SHRECEIVER_LM48560 */
#endif
