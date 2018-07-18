/* include/sharp/shdisp_dsi.h  (Display Driver)
 *
 * Copyright (C) 2013 SHARP CORPORATION
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

#ifndef SHDISP_DSI_H
#define SHDISP_DSI_H

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_DTYPE_DCS_WRITE     (0x05)
#define SHDISP_DTYPE_DCS_WRITE1    (0x15)
#define SHDISP_DTYPE_DCS_READ      (0x06)
#define SHDISP_DTYPE_DCS_LWRITE    (0x39)

#define SHDISP_DTYPE_GEN_WRITE     (0x03)
#define SHDISP_DTYPE_GEN_WRITE1    (0x13)
#define SHDISP_DTYPE_GEN_WRITE2    (0x23)
#define SHDISP_DTYPE_GEN_LWRITE    (0x29)
#define SHDISP_DTYPE_GEN_READ      (0x04)
#define SHDISP_DTYPE_GEN_READ1     (0x14)
#define SHDISP_DTYPE_GEN_READ2     (0x24)


/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
struct shdisp_dsi_cmd_desc {
    char dtype;
    short dlen;
    char *payload;
    int  wait;
};

#endif /* SHDISP_PANEL_API_H */
