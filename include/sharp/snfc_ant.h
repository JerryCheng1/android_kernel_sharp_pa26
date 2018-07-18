/* include/sharp/snfc_ant.h (NFC driver)
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
#ifndef _LINUX_SNFC_ANT_H
#define _LINUX_SNFC_ANT_H

#define IOC_MAGIC 's'

#define SNFC_ANT_PPC_RELOAD								_IO(IOC_MAGIC, 1)					/* reload */
#define SNFC_ANT_PPC_READ_CONTROLPORT_E2PROM			_IOR(IOC_MAGIC, 2, unsigned char)	/* read ControlPort E2PROM */
#define SNFC_ANT_PPC_READ_CONTROLPORT_REGISTER			_IOR(IOC_MAGIC, 3, unsigned char)	/* read ControlPort Register */
#define SNFC_ANT_PPC_WRITE_CONTROLPORT_E2PROM			_IOW(IOC_MAGIC, 4, unsigned char)	/* write ControlPort E2PROM */
#define SNFC_ANT_PPC_WRITE_CONTROLPORT_REGISTER			_IOW(IOC_MAGIC, 5, unsigned char)	/* write ControlPort Register */

#endif /* _LINUX_SNFC_ANT_H */

