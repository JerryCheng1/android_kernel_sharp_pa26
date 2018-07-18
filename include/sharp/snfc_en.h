/* include/sharp/snfc_en.h (NFC driver)
 *
 * Copyright (C) 2013-2014 SHARP CORPORATION
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
#ifndef _LINUX_SNFC_EN_H
#define _LINUX_SNFC_EN_H

#define SNFC_EN_IOC_MAGIC 's'

#define SHSNFC_EN_REQ_CHIPRESET			_IO(SNFC_EN_IOC_MAGIC, 1)
#define SHSNFC_EN_REQ_PVDD_ENABLE		_IOW(SNFC_EN_IOC_MAGIC, 2, int)
#define SHSNFC_EN_REQ_AVDD_ENABLE		_IOW(SNFC_EN_IOC_MAGIC, 3, int)
#define SHSNFC_EN_REQ_TVDD_ENABLE		_IOW(SNFC_EN_IOC_MAGIC, 4, int)
#define SHSNFC_EN_REQ_VEN_ENABLE		_IOW(SNFC_EN_IOC_MAGIC, 5, int)
#define SHSNFC_EN_REQ_FIRM_ENABLE		_IOW(SNFC_EN_IOC_MAGIC, 6, int)
#define SHSNFC_EN_GET_UART_RX_VALUE		_IOW(SNFC_EN_IOC_MAGIC, 7, int)

#endif /* _LINUX_SNFC_EN_H */

