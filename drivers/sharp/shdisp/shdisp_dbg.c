/* drivers/sharp/shdisp/shdisp_dbg.c  (Display Driver)
 *
 * Copyright (C) 2012 SHARP CORPORATION
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
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/path.h>
#include <linux/namei.h>
#include <linux/time.h>
#include <asm/unwind.h>
#include <asm/stacktrace.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/sh_boot_manager.h>
#include "shdisp_kerl_priv.h"
#include "shdisp_dbg.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_LOG_ENABLE
#if defined(CONFIG_ANDROID_ENGINEERING)
    unsigned char shdisp_log_lv = SHDISP_LOG_LV_ERR | SHDISP_LOG_LV_WARN;
#else  /* CONFIG_ANDROID_ENGINEERING */
    unsigned char shdisp_log_lv = SHDISP_LOG_LV_ERR;
#endif /* CONFIG_ANDROID_ENGINEERING */
#endif  /* SHDISP_LOG_ENABLE */

#define SHDISP_DBG_DISPLAY_ERROR_FILE               ("/durable/display/displaylog.txt")

#define SHDISP_DBG_ERROR_LOG_ADD_NORMAL             (0)
#define SHDISP_DBG_ERROR_LOG_ADD_CYCLIC             (1)
#define SHDISP_DBG_ERROR_LOG_FILE_TITLE             ("Display Driver Log File\n")
#define SHDISP_DBG_ERROR_LOG_FILE_TITLE_SIZE        (sizeof(SHDISP_DBG_ERROR_LOG_FILE_TITLE) - 1)

#define SHDISP_DBG_ERROR_LOG_TITLE                  ("\n<Error Log>\n")
#define SHDISP_DBG_ERROR_LOG_HEADER                 ("No,  Date,            Time,     Timezone,   Date2,           "\
                                                     "Time2,      Mode,  Type,       Code,              Sub Code,  "\
                                                     "        \n")
#define SHDISP_DBG_ERROR_LOG_BLANK                  ("                                                             "\
                                                     "                                                             "\
                                                     "        \n")
#define SHDISP_DBG_ERROR_LOG_NUM                    (100)

#define SHDISP_DBG_ERROR_LOG_TITLE_SIZE             (sizeof(SHDISP_DBG_ERROR_LOG_TITLE) - 1)
#define SHDISP_DBG_ERROR_LOG_HEADER_SIZE            (sizeof(SHDISP_DBG_ERROR_LOG_HEADER) - 1)
#define SHDISP_DBG_ERROR_LOG_LINE_SIZE              (sizeof(SHDISP_DBG_ERROR_LOG_BLANK) - 1)
#define SHDISP_DBG_ERROR_LOG_TOP                    (SHDISP_DBG_ERROR_LOG_FILE_TITLE_SIZE)

#define SHDISP_DBG_MODE_LINUX_STR                   "Linux, "
#define SHDISP_DBG_MODE_NoOS_STR                    "NoOS,  "
#define SHDISP_DBG_MODE_LINUX_BOOTING_STR           "Boot,  "
#define SHDISP_DBG_MODE_APPSBL_STR                  "APPSBL,"

#define SHDISP_DBG_TYPE_PANEL_STR                   "Panel,      "
#define SHDISP_DBG_TYPE_BDIC_STR                    "BDIC,       "
#define SHDISP_DBG_TYPE_PSALS_STR                   "PSALS,      "

#define SHDISP_DBG_CODE_RETRYOVER_STR               "Retry Over,        "
#define SHDISP_DBG_CODE_READERR_STR                 "Read Error,        "
#define SHDISP_DBG_CODE_ERRDET_STR                  "Error Detection,   "

#define SHDISP_DBG_SUBCODE_NONE_STR                 "---,               "
#define SHDISP_DBG_SUBCODE_DEVCODE_STR              "Devcode,           "
#define SHDISP_DBG_SUBCODE_DISPONCHK_STR            "DISPON CHK,        "
#define SHDISP_DBG_SUBCODE_DETLOW_STR               "DET LOW,           "
#define SHDISP_DBG_SUBCODE_ESDDETIN_STR             "ESD DETIN,         "
#define SHDISP_DBG_SUBCODE_ESDMIPI_STR              "ESD MIPI Error,    "
#define SHDISP_DBG_SUBCODE_I2CREAD_STR              "I2C READ,          "
#define SHDISP_DBG_SUBCODE_I2CERR_STR               "I2C_ERR,           "
#define SHDISP_DBG_SUBCODE_PSREQ_STR                "PS_REQ,            "
#define SHDISP_DBG_SUBCODE_RECOVNG_STR              "Recovery NG,       "
#define SHDISP_DBG_SUBCODE_PSALSNG_STR              "PSALS ON NG,       "
#define SHDISP_DBG_SUBCODE_POWERNG_STR              "POWER ON NG,       "

#define SHDISP_DBG_ERROR_LOG_SUMMARY_TITLE        ("\n<Error Log Summary>\n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_HEADER       ("Mode,  Type,       Code,              Sub Code,          Count \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE01         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_READERR_STR       \
                                                     SHDISP_DBG_SUBCODE_DISPONCHK_STR  \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE02         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_DISPONCHK_STR  \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE03         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_DETLOW_STR     \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE04         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_ESDDETIN_STR   \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE05         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_ESDMIPI_STR    \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE06         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_RETRYOVER_STR     \
                                                     SHDISP_DBG_SUBCODE_DISPONCHK_STR  \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE07         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_RETRYOVER_STR     \
                                                     SHDISP_DBG_SUBCODE_DETLOW_STR     \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE08         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_RETRYOVER_STR     \
                                                     SHDISP_DBG_SUBCODE_ESDDETIN_STR   \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE09         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_RETRYOVER_STR     \
                                                     SHDISP_DBG_SUBCODE_ESDMIPI_STR    \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE10         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PSALS_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_I2CERR_STR     \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE11         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PSALS_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_RECOVNG_STR    \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE12         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PSALS_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_PSALSNG_STR    \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE13         (SHDISP_DBG_MODE_LINUX_STR         \
                                                     SHDISP_DBG_TYPE_PSALS_STR         \
                                                     SHDISP_DBG_CODE_RETRYOVER_STR     \
                                                     SHDISP_DBG_SUBCODE_POWERNG_STR    \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE14         (SHDISP_DBG_MODE_APPSBL_STR        \
                                                     SHDISP_DBG_TYPE_PSALS_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_PSALSNG_STR    \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE15         (SHDISP_DBG_MODE_APPSBL_STR        \
                                                     SHDISP_DBG_TYPE_PSALS_STR         \
                                                     SHDISP_DBG_CODE_RETRYOVER_STR     \
                                                     SHDISP_DBG_SUBCODE_POWERNG_STR    \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE16         (SHDISP_DBG_MODE_LINUX_BOOTING_STR \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_READERR_STR       \
                                                     SHDISP_DBG_SUBCODE_DISPONCHK_STR  \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE17         (SHDISP_DBG_MODE_LINUX_BOOTING_STR \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_DISPONCHK_STR  \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE18         (SHDISP_DBG_MODE_LINUX_BOOTING_STR \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_DETLOW_STR     \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE19         (SHDISP_DBG_MODE_LINUX_BOOTING_STR \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_ESDDETIN_STR    \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE20         (SHDISP_DBG_MODE_LINUX_BOOTING_STR \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_ESDMIPI_STR    \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE21         (SHDISP_DBG_MODE_LINUX_BOOTING_STR \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_RETRYOVER_STR     \
                                                     SHDISP_DBG_SUBCODE_DISPONCHK_STR  \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE22         (SHDISP_DBG_MODE_LINUX_BOOTING_STR \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_RETRYOVER_STR     \
                                                     SHDISP_DBG_SUBCODE_DETLOW_STR     \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE23         (SHDISP_DBG_MODE_LINUX_BOOTING_STR \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_RETRYOVER_STR     \
                                                     SHDISP_DBG_SUBCODE_ESDDETIN_STR   \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE24         (SHDISP_DBG_MODE_LINUX_BOOTING_STR \
                                                     SHDISP_DBG_TYPE_PANEL_STR         \
                                                     SHDISP_DBG_CODE_RETRYOVER_STR     \
                                                     SHDISP_DBG_SUBCODE_ESDMIPI_STR    \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE25         (SHDISP_DBG_MODE_LINUX_BOOTING_STR \
                                                     SHDISP_DBG_TYPE_PSALS_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_I2CERR_STR     \
                                                     "   0  \n")
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE26         (SHDISP_DBG_MODE_LINUX_BOOTING_STR \
                                                     SHDISP_DBG_TYPE_PSALS_STR         \
                                                     SHDISP_DBG_CODE_ERRDET_STR        \
                                                     SHDISP_DBG_SUBCODE_RECOVNG_STR    \
                                                     "   0  \n")


#define SHDISP_DBG_ERROR_LOG_SUMMARY_TITLE_SIZE   (sizeof(SHDISP_DBG_ERROR_LOG_SUMMARY_TITLE) - 1)
#define SHDISP_DBG_ERROR_LOG_SUMMARY_HEADER_SIZE  (sizeof(SHDISP_DBG_ERROR_LOG_SUMMARY_HEADER) - 1)
#define SHDISP_DBG_ERROR_LOG_SUMMARY_LINE_SIZE    (sizeof(SHDISP_DBG_ERROR_LOG_SUMMARY_LINE01) - 1)
#define SHDISP_DBG_ERROR_LOG_SUMMARY_COUNTER_POS  (SHDISP_DBG_ERROR_LOG_SUMMARY_LINE_SIZE - 7)
#define SHDISP_DBG_ERROR_LOG_SUMMARY_TOP          (SHDISP_DBG_ERROR_LOG_TOP + SHDISP_DBG_ERROR_LOG_TITLE_SIZE + \
                                                   SHDISP_DBG_ERROR_LOG_HEADER_SIZE + \
                                                  (SHDISP_DBG_ERROR_LOG_LINE_SIZE * SHDISP_DBG_ERROR_LOG_NUM))

struct shdisp_dbg_reset_work_command {
    void (*proc)(void *arg, int *reset);
    void *arg;
    struct list_head list;
};

#if defined(SHDISP_DBG_DUMP) || defined(SHDISP_RESET_LOG)
static struct shdisp_dbg_reset_work_command *shdisp_dbg_reset_work_alloc_command(void);
static void shdisp_dbg_reset_work_add_command(struct shdisp_dbg_reset_work_command *cmd);
static void shdisp_dbg_reset_work_start(void);
#endif /* SHDISP_DBG_DUMP || SHDISP_RESET_LOG */
static void shdisp_dbg_reset_work_free_command(struct shdisp_dbg_reset_work_command *cmd);

#ifdef SHDISP_RESET_LOG
#define SHDISP_DBG_BOOT_ERRCODE_NUM (3)
struct shdisp_workqueue_handler_dbg_reset_log_params {
    struct shdisp_dbg_error_code code;
    int reset;
};
static int shdisp_dbg_boot_errcodes_count = 0;
static struct shdisp_dbg_error_code shdisp_dbg_boot_errcodes[SHDISP_DBG_BOOT_ERRCODE_NUM];
static int shdisp_dbg_boot_errcodes_reset[SHDISP_DBG_BOOT_ERRCODE_NUM];
#endif /* SHDISP_RESET_LOG */
#ifdef SHDISP_DBG_DUMP
#define SHDISP_DBG_DUMPFILE_MAX_RETRY (10)
#define SHDISP_DBG_DUMPFILE_NUM    (5)
#define SHDISP_DBG_DUMPFILE_DIR    "/durable/display"
#define SHDISP_DBG_DUMPFILE_FNAME  "displaydump"
#define SHDISP_DBG_DUMPFILE_INNER_FNAME "displaydump.txt"

struct shdisp_dbg_dispdump_worker_params {
    int reset;
    int mode;
    struct shdisp_dbg_ptrinfo ring_dump_buf;
    struct shdisp_dbg_ptrinfo stack_dump_buf;
    struct shdisp_dbg_ptrinfo edram_dump_buf;
    struct shdisp_dbg_ptrinfo sram_dump_buf;
};
#endif /* SHDISP_DBG_DUMP */
static char printk_buf[LINE_BUF_SIZE];
static int log_buf_offset = 0;
static char log_buf[LOG_BUF_SIZE] = {0};
static int ring = 0;
#if defined(CONFIG_ANDROID_ENGINEERING)
static int err_sysetem_reset = SHDISP_DBG_RESET_OFF;
static int shdisp_dbg_recovery_check_error_flag = SHDISP_DBG_RECOVERY_ERROR_NONE;
static int shdisp_dbg_recovery_check_error_count = 0;
#endif /* CONFIG_ANDROID_ENGINEERING */

static void shdisp_dbg_reset_work_worker(struct work_struct *wk);
static DEFINE_SPINLOCK(shdisp_dbg_reset_work_queue_lock);
static LIST_HEAD(shdisp_dbg_reset_work_queue);
static struct workqueue_struct *shdisp_dbg_reset_work_wq = NULL;
static DECLARE_WORK(shdisp_dbg_reset_work_wk, shdisp_dbg_reset_work_worker);

const static const char *WeekOfDay[] = {
    "Sun",
    "Mon",
    "Tue",
    "Wed",
    "Thu",
    "Fri",
    "Sat"
};

const static const char *ModeIndex[] = {
    SHDISP_DBG_MODE_LINUX_STR,
    SHDISP_DBG_MODE_APPSBL_STR,
    SHDISP_DBG_MODE_NoOS_STR,
    SHDISP_DBG_MODE_LINUX_STR,
    SHDISP_DBG_MODE_LINUX_BOOTING_STR
};

const static const char *TypeIndex[] = {
    SHDISP_DBG_TYPE_PANEL_STR,
    SHDISP_DBG_TYPE_BDIC_STR,
    SHDISP_DBG_TYPE_PSALS_STR
};

const static const char *CodeIndex[] = {
    SHDISP_DBG_CODE_RETRYOVER_STR,
    SHDISP_DBG_CODE_READERR_STR,
    SHDISP_DBG_CODE_ERRDET_STR
};

const static const char *SubCodeIndex[] = {
    SHDISP_DBG_SUBCODE_NONE_STR,
    SHDISP_DBG_SUBCODE_DISPONCHK_STR,
    SHDISP_DBG_SUBCODE_DETLOW_STR,
    SHDISP_DBG_SUBCODE_ESDDETIN_STR,
    SHDISP_DBG_SUBCODE_ESDMIPI_STR,
    SHDISP_DBG_SUBCODE_I2CREAD_STR,
    SHDISP_DBG_SUBCODE_I2CERR_STR,
    SHDISP_DBG_SUBCODE_PSREQ_STR,
    SHDISP_DBG_SUBCODE_RECOVNG_STR,
    SHDISP_DBG_SUBCODE_PSALSNG_STR,
    SHDISP_DBG_SUBCODE_POWERNG_STR
};

const static struct shdisp_dbg_error_code ErrorCodeIndex[] = {
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_READ_ERROR,
                                                                                SHDISP_DBG_SUBCODE_DISPON_CHK       },
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_DISPON_CHK       },
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_DET_LOW          },
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_ESD_DETIN        },
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_ESD_MIPI         },
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,
                                                                                SHDISP_DBG_SUBCODE_DISPON_CHK       },
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,
                                                                                SHDISP_DBG_SUBCODE_DET_LOW          },
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,
                                                                                SHDISP_DBG_SUBCODE_ESD_DETIN        },
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,
                                                                                SHDISP_DBG_SUBCODE_ESD_MIPI         },
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_I2C_ERROR        },
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_RECOVERY_NG      },
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_PSALS_ON_NG      },
    {SHDISP_DBG_MODE_LINUX_BOOTED,  SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_RETRY_OVER,
                                                                                SHDISP_DBG_SUBCODE_POWER_ON_NG      },
    {SHDISP_DBG_MODE_APPSBL,        SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_PSALS_ON_NG      },
    {SHDISP_DBG_MODE_APPSBL,        SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_RETRY_OVER,
                                                                                SHDISP_DBG_SUBCODE_POWER_ON_NG      },
    {SHDISP_DBG_MODE_LINUX_BOOTING, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_READ_ERROR,
                                                                                SHDISP_DBG_SUBCODE_DISPON_CHK       },
    {SHDISP_DBG_MODE_LINUX_BOOTING, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_DISPON_CHK       },
    {SHDISP_DBG_MODE_LINUX_BOOTING, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_DET_LOW          },
    {SHDISP_DBG_MODE_LINUX_BOOTING, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_ESD_DETIN        },
    {SHDISP_DBG_MODE_LINUX_BOOTING, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_ESD_MIPI         },
    {SHDISP_DBG_MODE_LINUX_BOOTING, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,
                                                                                SHDISP_DBG_SUBCODE_DISPON_CHK       },
    {SHDISP_DBG_MODE_LINUX_BOOTING, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,
                                                                                SHDISP_DBG_SUBCODE_DET_LOW          },
    {SHDISP_DBG_MODE_LINUX_BOOTING, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,
                                                                                SHDISP_DBG_SUBCODE_ESD_DETIN        },
    {SHDISP_DBG_MODE_LINUX_BOOTING, SHDISP_DBG_TYPE_PANEL,      SHDISP_DBG_CODE_RETRY_OVER,
                                                                                SHDISP_DBG_SUBCODE_ESD_MIPI         },
    {SHDISP_DBG_MODE_LINUX_BOOTING, SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_I2C_ERROR        },
    {SHDISP_DBG_MODE_LINUX_BOOTING, SHDISP_DBG_TYPE_PSALS,      SHDISP_DBG_CODE_ERROR_DETECT,
                                                                                SHDISP_DBG_SUBCODE_RECOVERY_NG      }
};

const static const char *SummaryFormat[] = {
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE01,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE02,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE03,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE04,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE05,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE06,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE07,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE08,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE09,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE10,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE11,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE12,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE13,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE14,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE15,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE16,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE17,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE18,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE19,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE20,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE21,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE22,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE23,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE24,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE25,
    SHDISP_DBG_ERROR_LOG_SUMMARY_LINE26
};

static int subcode_val = SHDISP_DBG_SUBCODE_NONE;


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_vprintk(const char *fmt, va_list args, int offset);
static spinlock_t shdisp_dbg_spin_lock;
static int shdisp_dbg_lock_init_done = 0;

static size_t shdisp_dbg_get_stacktrace(char *stackdumpbuf, size_t length);
static ssize_t shdisp_dbg_kernel_write(struct file *fp, const char *buf, size_t size);
static ssize_t shdisp_dbg_kernel_read(struct file *fp, char *buf, size_t size , unsigned int offset);
static ssize_t shdisp_dbg_kernel_seek(struct file *fp, unsigned int offset);
static int shdisp_dbg_kernel_sync(struct file *fp);

static int shdisp_dbg_add_err_log_one_line(char *buf, struct shdisp_dbg_error_code *code);
static int shdisp_dbg_add_err_log_file(struct shdisp_dbg_error_code *code);
static int shdisp_dbg_summary_init_file(void);
static int shdisp_dbg_summary_countup_file(int idx);
static void shdisp_dbg_summary_countup_line(char *buf);
static int shdisp_dbg_countup_one(char *pCount);

#ifdef SHDISP_RESET_LOG
static void shdisp_workqueue_handler_dbg_reset_log(void *arg, int *reset);
static int  shdisp_dbg_reset_log_output(struct shdisp_dbg_error_code *code, int reset);
#endif /* SHDISP_RESET_LOG */


/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_init                                                       */
/* ------------------------------------------------------------------------- */
void shdisp_dbg_API_init(void)
{
    shdisp_dbg_lock_init_done = 0;

#if defined(CONFIG_ANDROID_ENGINEERING)
    shdisp_dbg_API_set_reset_flg(SHDISP_DBG_RESET_OFF);
#endif /* CONFIG_ANDROID_ENGINEERING */

    shdisp_dbg_reset_work_wq = create_singlethread_workqueue("shdisp_dbg_reset_work_wq");
    if (!shdisp_dbg_reset_work_wq) {
        SHDISP_DEBUG("shdisp_dbg_reset_work_wq create failed." );
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_printk                                                     */
/* ------------------------------------------------------------------------- */
int shdisp_dbg_API_printk(const char *fmt, ...)
{
    unsigned long irq_flag, spin_flag;

    va_list args;
    int offset;

    if (!shdisp_dbg_lock_init_done) {
        return SHDISP_RESULT_SUCCESS;
    }

    local_irq_save(irq_flag);
    spin_lock_irqsave(&shdisp_dbg_spin_lock, spin_flag);

    va_start(args, fmt);
    offset = log_buf_offset;
    offset = shdisp_vprintk(fmt, args, offset);
    log_buf_offset = offset;
    va_end(args);

    spin_unlock_irqrestore(&shdisp_dbg_spin_lock, spin_flag);
    local_irq_restore(irq_flag);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_vprintk                                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_vprintk(const char *fmt, va_list args, int offset)
{
    int len;
    int this_cpu;
    unsigned long long t;
    unsigned long nanosec_rem;

    this_cpu = smp_processor_id();

    /* Add the current time stamp */
    t = cpu_clock(this_cpu);
    nanosec_rem = do_div(t, 1000000000);
    len = sprintf(printk_buf, "[%5lu.%06lu] ",
            (unsigned long) t,
            nanosec_rem / 1000);

    len += vscnprintf(&printk_buf[len],
                     sizeof(printk_buf) - len, fmt, args);

    if ((offset + len) < LOG_BUF_SIZE) {
        memcpy(&log_buf[offset], printk_buf, len);
        offset = offset + len;
    } else {
        ring = 1;
        memcpy(&log_buf[offset], printk_buf, LOG_BUF_SIZE - offset);
        memcpy(&log_buf[0], &printk_buf[LOG_BUF_SIZE - offset], len - (LOG_BUF_SIZE - offset));
        offset = len - (LOG_BUF_SIZE - offset);
    }
    log_buf[offset] = '\0';
    offset++;

    return offset;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_stacktrace_dump                                            */
/* ------------------------------------------------------------------------- */
size_t shdisp_dbg_API_stacktrace_dump(char *buf, size_t length)
{
    printk(KERN_ERR "[SHDISP] STACK DUMP -------------------->\n");
    dump_stack();
    printk(KERN_ERR "<-------------------- [SHDISP] STACK DUMP\n");

    if (buf != NULL) {
        return shdisp_dbg_get_stacktrace(buf, length);
    } else {
        return 0;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_ringbuffer_dump                                            */
/* ------------------------------------------------------------------------- */
void shdisp_dbg_API_ringbuffer_dump(char *buf)
{
    int l;
    int s;
    int len = 0;
    char *p;
    char *file_p = buf;
    unsigned long irq_flag, spin_flag;
    if (!shdisp_dbg_lock_init_done) {
        return;
    }

    local_irq_save(irq_flag);
    spin_lock_irqsave(&shdisp_dbg_spin_lock, spin_flag);

    if (ring == 0) {
        s = 0;
    } else {
        s = log_buf_offset;
    }

    printk(KERN_ERR "[SHDISP] LOG DUMP -------------------->\n");

    while (len < LOG_BUF_SIZE) {

        p = printk_buf;
        l = 0;

        while (s < LOG_BUF_SIZE && l < LINE_BUF_SIZE - 1) {

            if (log_buf[s] == '\0') {
                break;
            }

            p[l] = log_buf[s];
            s++;
            l++;
        }

        if (s >= LOG_BUF_SIZE) {

            s = 0;

            while (s < LOG_BUF_SIZE && l < LINE_BUF_SIZE - 1) {

                if (log_buf[s] == '\0') {
                    break;
                }

                p[l] = log_buf[s];
                s++;
                l++;
            }
        }

        if (l > 0) {
            if (len > LOG_BUF_SIZE_TO_KERNEL) {
                p[l] = '\0';
                printk(KERN_ERR "%s", p);
            }
            if (file_p != NULL) {
                p[l] = '\n';
                memcpy(file_p + len, p, l + 1);
            }
        }
        s++;
        len += (l + 1);

    }

    printk(KERN_ERR "<-------------------- [SHDISP] LOG DUMP\n");

    spin_unlock_irqrestore(&shdisp_dbg_spin_lock, spin_flag);
    local_irq_restore(irq_flag);

    return;

}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_get_stacktrace                                                 */
/* ------------------------------------------------------------------------- */
static size_t shdisp_dbg_get_stacktrace(char *stackdumpbuf, size_t length)
{
    size_t writelen = 0;
    struct stackframe frame = { 0 };
    register unsigned long current_sp asm ("sp");

    frame.fp = (unsigned long)__builtin_frame_address(0);
    frame.sp = current_sp;
    frame.lr = (unsigned long)__builtin_return_address(0);
    frame.pc = (unsigned long)shdisp_dbg_get_stacktrace;

   SHDISP_TRACE("ptr=0x%p len=%d", stackdumpbuf, length);
    while (1) {
        int urc, rtn;

        rtn = snprintf( stackdumpbuf, length, "%pS\n", (void *)frame.pc );


        if (rtn < 0) {
            SHDISP_ERR("dump heap shortage error.");
            break;
        }

        urc = unwind_frame(&frame);
        length -= rtn;
        writelen += rtn;
        stackdumpbuf += rtn;

        if (urc != URC_OK) {
             break;
        }
    }
    SHDISP_TRACE("dump stuck trace size=%d", writelen);
    return writelen;
}

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_set_reset_flg                                              */
/* ------------------------------------------------------------------------- */
void shdisp_dbg_API_set_reset_flg(int sw)
{
    err_sysetem_reset = sw;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_get_reset_flg                                              */
/* ------------------------------------------------------------------------- */
int shdisp_dbg_API_get_reset_flg(void)
{
    return err_sysetem_reset;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_set_recovery_check_error                                   */
/* ------------------------------------------------------------------------- */
void shdisp_dbg_API_set_recovery_check_error(int flag, int count)
{
    shdisp_dbg_recovery_check_error_flag = flag;
    shdisp_dbg_recovery_check_error_count = count;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_get_recovery_check_error                                   */
/* ------------------------------------------------------------------------- */
int shdisp_dbg_API_get_recovery_check_error(void)
{
    return shdisp_dbg_recovery_check_error_flag;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_update_recovery_check_error                                */
/* ------------------------------------------------------------------------- */
void shdisp_dbg_API_update_recovery_check_error(int flag)
{
    if (shdisp_dbg_recovery_check_error_count != 0) {
        shdisp_dbg_recovery_check_error_count--;
        if (shdisp_dbg_recovery_check_error_count == 0) {
            shdisp_dbg_recovery_check_error_flag = flag;
        }
    }
}
#endif /* defined (CONFIG_ANDROID_ENGINEERING) */

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_kernel_write                                                   */
/* ------------------------------------------------------------------------- */
static ssize_t shdisp_dbg_kernel_write(struct file *fp, const char *buf, size_t size)
{
    mm_segment_t old_fs;
    ssize_t res = 0;

    if (buf == NULL) {
        SHDISP_ERR("<NULL_POINTER>");
        return res;
    }

    old_fs = get_fs();
    set_fs(get_ds());
    res = fp->f_op->write(fp, buf, size, &fp->f_pos);
    set_fs(old_fs);

    return res;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_kernel_read                                                    */
/* ------------------------------------------------------------------------- */
static ssize_t shdisp_dbg_kernel_read(struct file *fp, char *buf, size_t size , unsigned int offset)
{
    mm_segment_t old_fs;
    ssize_t res = 0;
    loff_t fpos = offset;

    if (buf == NULL) {
        SHDISP_ERR("<NULL_POINTER>");
        return res;
    }

    old_fs = get_fs();
    set_fs(get_ds());
    res = fp->f_op->read(fp, buf, size, &fpos);
    set_fs(old_fs);

    return res;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_kernel_seek                                                    */
/* ------------------------------------------------------------------------- */
static ssize_t shdisp_dbg_kernel_seek(struct file *fp, unsigned int offset)
{
    ssize_t res;
    loff_t fpos;

    fpos = offset;
    res = fp->f_op->llseek(fp, fpos, SEEK_SET);

    return res;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_kernel_sync                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_kernel_sync(struct file *fp)
{
    int res;

    res = fp->f_op->fsync(fp, 0, LLONG_MAX, 0);

    return res;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_add_err_log                                                */
/* ------------------------------------------------------------------------- */
int shdisp_dbg_API_add_err_log(struct shdisp_dbg_error_code *code)
{
    int ret;

    SHDISP_TRACE("in");

    if (code->mode    >= SHDISP_DBG_MODE_MAX ||
        code->type    >= SHDISP_DBG_TYPE_MAX ||
        code->code    >= SHDISP_DBG_CODE_MAX ||
        code->subcode >= SHDISP_DBG_SUBCODE_MAX) {
        SHDISP_ERR("parameter error.");
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_dbg_add_err_log_file(code);
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_add_err_log_file                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_add_err_log_file(struct shdisp_dbg_error_code *code)
{
    struct path  path;
    struct file *fp;
    unsigned int offset;
    char *buf;
    int ret = -EINVAL;
    size_t size = (SHDISP_DBG_ERROR_LOG_LINE_SIZE * SHDISP_DBG_ERROR_LOG_NUM);

    SHDISP_TRACE("in");
    ret = kern_path(SHDISP_DBG_DISPLAY_ERROR_FILE, LOOKUP_OPEN, &path);
    if (ret != 0) {
        ret = shdisp_dbg_summary_init_file();
        if (ret != SHDISP_RESULT_SUCCESS) {
            return SHDISP_RESULT_FAILURE;
        }
    }
    offset =  SHDISP_DBG_ERROR_LOG_TOP;
    offset += SHDISP_DBG_ERROR_LOG_TITLE_SIZE;
    offset += SHDISP_DBG_ERROR_LOG_HEADER_SIZE;

    buf = kzalloc((size + SHDISP_DBG_ERROR_LOG_LINE_SIZE), GFP_KERNEL);
    if (!buf) {
        SHDISP_ERR("allocate read buffer error. [no memory]");
        return -ENOMEM;
    }
    fp = filp_open(SHDISP_DBG_DISPLAY_ERROR_FILE, O_RDWR, 0660);
    if (IS_ERR_OR_NULL(fp)) {
        kfree(buf);
        SHDISP_ERR("Cannot open file: %s err=%d", SHDISP_DBG_DISPLAY_ERROR_FILE, (int)fp);
        return SHDISP_RESULT_FAILURE;
    }
    shdisp_dbg_kernel_read(fp, buf, size, offset);

    ret = shdisp_dbg_add_err_log_one_line(buf, code);
    shdisp_dbg_kernel_seek(fp, offset);

    if (ret == SHDISP_DBG_ERROR_LOG_ADD_NORMAL) {
        shdisp_dbg_kernel_write(fp, buf, size);
    } else {
        shdisp_dbg_kernel_write(fp, buf + SHDISP_DBG_ERROR_LOG_LINE_SIZE, size);
    }
    kfree(buf);
    {
        int res;
        if ((res = shdisp_dbg_kernel_sync(fp)) != 0) {
            SHDISP_ERR("fsync result: %d", res);
        }
    }
    filp_close(fp, NULL);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_add_err_log_one_line                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_add_err_log_one_line(char *buf, struct shdisp_dbg_error_code *code)
{
    struct timeval tv;
    struct tm tm1, tm2;
    int i, blank_area = 0;
    char *bufwk;

    bufwk = buf;

    do_gettimeofday(&tv);
    time_to_tm((time_t)tv.tv_sec, 0, &tm1);
    time_to_tm((time_t)tv.tv_sec, (sys_tz.tz_minuteswest * 60 * (-1)), &tm2);

    for (i = 1; i <= SHDISP_DBG_ERROR_LOG_NUM; i++) {
        if (*bufwk == ' ') {
            blank_area = 1;
            break;
        } else {
            bufwk += SHDISP_DBG_ERROR_LOG_LINE_SIZE;
        }
    }

    sprintf(bufwk, "%03d, %04d/%02d/%02d(%s), %02d:%02d:%02d, UTC +00h,"
                   "   %04d/%02d/%02d(%s), %02d:%02d:%02d,   %s%s%s%s",
        ((blank_area == 1) ? i : SHDISP_DBG_ERROR_LOG_NUM),
        (int)(tm1.tm_year + 1900), tm1.tm_mon + 1, tm1.tm_mday, WeekOfDay[tm1.tm_wday],
        tm1.tm_hour, tm1.tm_min, tm1.tm_sec,
        (int)(tm2.tm_year + 1900), tm2.tm_mon + 1, tm2.tm_mday, WeekOfDay[tm2.tm_wday],
        tm2.tm_hour, tm2.tm_min, tm2.tm_sec,
        ModeIndex[code->mode],
        TypeIndex[code->type],
        CodeIndex[code->code],
        SubCodeIndex[code->subcode]);
    bufwk += (SHDISP_DBG_ERROR_LOG_LINE_SIZE - 1);
    if (*bufwk == 0x00) {
        *bufwk = 0x0a;
    }

    if (!blank_area) {
        bufwk = buf + SHDISP_DBG_ERROR_LOG_LINE_SIZE;
        for (i = 1; i < SHDISP_DBG_ERROR_LOG_NUM; i++) {
            *(bufwk + 0) = '0' + (i / 100);
            *(bufwk + 1) = '0' + ((i % 100) / 10);
            *(bufwk + 2) = '0' + (i % 10);
            bufwk += SHDISP_DBG_ERROR_LOG_LINE_SIZE;
        }
        return SHDISP_DBG_ERROR_LOG_ADD_CYCLIC;
    }
    return SHDISP_DBG_ERROR_LOG_ADD_NORMAL;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_err_countup                                                */
/* ------------------------------------------------------------------------- */
int shdisp_dbg_API_err_countup(struct shdisp_dbg_error_code *code)
{
    int idx = -1;
    int i, ret;

    SHDISP_TRACE("in");
    for (i = 0; i < ARRAY_SIZE(ErrorCodeIndex); i++) {
        if (code->mode      == ErrorCodeIndex[i].mode &&
            code->type      == ErrorCodeIndex[i].type &&
            code->code      == ErrorCodeIndex[i].code &&
            code->subcode   == ErrorCodeIndex[i].subcode) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        SHDISP_ERR("parameter not matched.mode=%d.type=%d.code=%d.subcode=%d",
            code->mode,
            code->type,
            code->code,
            code->subcode);
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_dbg_summary_countup_file(idx);
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_summary_init_file                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_summary_init_file(void)
{
    struct file *fp;
    int i;

    SHDISP_TRACE("in");

    SHDISP_DEBUG("open file: %s pid=%d tgid=%d comm=%s", SHDISP_DBG_DISPLAY_ERROR_FILE, current->pid, current->tgid,
                                                                                                       current->comm);
    fp = filp_open(SHDISP_DBG_DISPLAY_ERROR_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0660);
    if (IS_ERR_OR_NULL(fp)) {
        SHDISP_ERR("Cannot create file: %s err=%d pid=%d tgid=%d comm=%s", SHDISP_DBG_DISPLAY_ERROR_FILE, (int)fp,
                                                                             current->pid, current->tgid,
                                                                             current->comm);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_dbg_kernel_write(fp, SHDISP_DBG_ERROR_LOG_FILE_TITLE, SHDISP_DBG_ERROR_LOG_FILE_TITLE_SIZE);
    shdisp_dbg_kernel_write(fp, SHDISP_DBG_ERROR_LOG_TITLE, SHDISP_DBG_ERROR_LOG_TITLE_SIZE);
    shdisp_dbg_kernel_write(fp, SHDISP_DBG_ERROR_LOG_HEADER, SHDISP_DBG_ERROR_LOG_HEADER_SIZE);
    for (i = 0; i < SHDISP_DBG_ERROR_LOG_NUM; i++) {
        shdisp_dbg_kernel_write(fp, SHDISP_DBG_ERROR_LOG_BLANK, SHDISP_DBG_ERROR_LOG_LINE_SIZE);
    }

    shdisp_dbg_kernel_write(fp, SHDISP_DBG_ERROR_LOG_SUMMARY_TITLE, SHDISP_DBG_ERROR_LOG_SUMMARY_TITLE_SIZE);
    shdisp_dbg_kernel_write(fp, SHDISP_DBG_ERROR_LOG_SUMMARY_HEADER, SHDISP_DBG_ERROR_LOG_SUMMARY_HEADER_SIZE);
    for (i = 0; i < ARRAY_SIZE(SummaryFormat); i++) {
        shdisp_dbg_kernel_write(fp, SummaryFormat[i], SHDISP_DBG_ERROR_LOG_SUMMARY_LINE_SIZE);
    }

    {
        int res;
        if ((res = shdisp_dbg_kernel_sync(fp)) != 0) {
            SHDISP_ERR("fsync result: %d", res);
        }
    }
    filp_close(fp, NULL);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_summary_countup_file                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_summary_countup_file(int idx)
{
    struct path  path;
    struct file *fp;
    unsigned int offset;
    char *buf;
    int ret = -EINVAL;
    size_t size = SHDISP_DBG_ERROR_LOG_SUMMARY_LINE_SIZE;

    SHDISP_TRACE("in");
    ret = kern_path(SHDISP_DBG_DISPLAY_ERROR_FILE, LOOKUP_OPEN, &path);
    if (ret != 0) {
        ret = shdisp_dbg_summary_init_file();
        if (ret != SHDISP_RESULT_SUCCESS) {
            return SHDISP_RESULT_FAILURE;
        }
    }
    offset =  SHDISP_DBG_ERROR_LOG_SUMMARY_TOP;
    offset += SHDISP_DBG_ERROR_LOG_SUMMARY_TITLE_SIZE;
    offset += SHDISP_DBG_ERROR_LOG_SUMMARY_HEADER_SIZE;
    offset += (idx * size);

    buf = kzalloc(size, GFP_KERNEL);
    if (!buf) {
        SHDISP_ERR("allocate read buffer error. [no memory]");
        return -ENOMEM;
    }
    fp = filp_open(SHDISP_DBG_DISPLAY_ERROR_FILE, O_RDWR, 0660);
    if (IS_ERR_OR_NULL(fp)) {
        kfree(buf);
        SHDISP_ERR("Cannot open file: %s err=%d pid=%d tgid=%d comm=%s", SHDISP_DBG_DISPLAY_ERROR_FILE, (int)fp,
                                                                           current->pid, current->tgid, current->comm);
        return SHDISP_RESULT_FAILURE;
    }
    shdisp_dbg_kernel_read(fp, buf, size, offset);
    shdisp_dbg_summary_countup_line(buf);
    shdisp_dbg_kernel_seek(fp, offset);
    shdisp_dbg_kernel_write(fp, buf, size);
    kfree(buf);
    {
        int res;
        if ((res = shdisp_dbg_kernel_sync(fp)) != 0) {
            SHDISP_ERR("fsync result: %d", res);
        }
    }
    filp_close(fp, NULL);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_summary_countup_line                                           */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_summary_countup_line(char *buf)
{
    char *pCount1000, *pCount100, *pCount10, *pCount1;
    int carry = 0;

    pCount1000  = buf + SHDISP_DBG_ERROR_LOG_SUMMARY_COUNTER_POS;
    pCount100   = pCount1000 + 1;
    pCount10    = pCount100  + 1;
    pCount1     = pCount10   + 1;

    if (*pCount1000 == '9' && *pCount100 == '9' && *pCount10 == '9' && *pCount1 == '9') {
        SHDISP_DEBUG("Counter is Max Value %c%c%c%c", *pCount1000, *pCount100, *pCount10, *pCount1);
        return;
    }

    carry = shdisp_dbg_countup_one(pCount1);
    if (carry) {
        carry = shdisp_dbg_countup_one(pCount10);
    }
    if (carry) {
        carry = shdisp_dbg_countup_one(pCount100);
    }
    if (carry) {
        carry = shdisp_dbg_countup_one(pCount1000);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_countup_one                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_countup_one(char *pCount)
{
    int carry = FALSE;

    if (*pCount >= '0' && *pCount <= '8') {
        *pCount += 0x01;
    } else if (*pCount == '9') {
        *pCount = '0';
        carry = TRUE;
    } else {
        *pCount = '1';
    }
    return carry;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_set_subcode                                                */
/* ------------------------------------------------------------------------- */
int shdisp_dbg_API_set_subcode(int sub_code)
{
    subcode_val = sub_code;

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_get_subcode                                                */
/* ------------------------------------------------------------------------- */
int shdisp_dbg_API_get_subcode(void)
{
    return subcode_val;
}

#ifdef SHDISP_RESET_LOG
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_reset_log_output                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_reset_log_output(struct shdisp_dbg_error_code *code, int reset)
{
    struct shdisp_dbg_reset_work_command *cmd = NULL;
    struct shdisp_workqueue_handler_dbg_reset_log_params *params = NULL;
    int error = -EINVAL;

    SHDISP_TRACE("in");

    params = (struct shdisp_workqueue_handler_dbg_reset_log_params *)kzalloc(
                                        sizeof(struct shdisp_workqueue_handler_dbg_reset_log_params), GFP_KERNEL);
    if (params == NULL) {
        SHDISP_ERR("allocate parameter error. [no memory]");
        error = -ENOMEM;
        goto errout;
    }

    params->code = *code;
    params->reset = reset;

    cmd = shdisp_dbg_reset_work_alloc_command();
    if (cmd == NULL) {
        SHDISP_ERR("allocate command error. [no memory]");
        error = -ENOMEM;
        goto errout;
    }

    cmd->proc = shdisp_workqueue_handler_dbg_reset_log;
    cmd->arg = params;

    shdisp_dbg_reset_work_add_command(cmd);
    shdisp_dbg_reset_work_start();

    SHDISP_TRACE("out normaly finished.");
    return SHDISP_RESULT_SUCCESS;

errout:
    if (cmd != NULL) {
        shdisp_dbg_reset_work_free_command(cmd);
    }
    if (params != NULL) {
        kfree(params);
    }

    SHDISP_ERR("abnormaly finished. error=%d", error);
    return error;
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_dbg_reset_log                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_handler_dbg_reset_log(void *arg, int *reset)
{
    struct shdisp_workqueue_handler_dbg_reset_log_params *params;

    SHDISP_TRACE("in");

    params = (struct shdisp_workqueue_handler_dbg_reset_log_params*)arg;

    shdisp_dbg_API_add_err_log(&params->code);
    shdisp_dbg_API_err_countup(&params->code);

    if (params->reset == 1) {
        *reset = 1;
    }

    kfree(params);

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_get_boot_errcodes                                          */
/* ------------------------------------------------------------------------- */
void shdisp_dbg_API_get_boot_errcodes(struct shdisp_dbg_error_code **codes, int **reset, int *count)
{
    *codes = shdisp_dbg_boot_errcodes;
    *reset = shdisp_dbg_boot_errcodes_reset;
    *count = shdisp_dbg_boot_errcodes_count;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_clear_boot_errcodes                                        */
/* ------------------------------------------------------------------------- */
void shdisp_dbg_API_clear_boot_errcodes(void)
{
    shdisp_dbg_boot_errcodes_count = 0;
}
#endif /* SHDISP_RESET_LOG */

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_err_output                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_dbg_API_err_output(struct shdisp_dbg_error_code *code, int reset)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");
#ifdef SHDISP_RESET_LOG
    {
        struct shdisp_dbg_error_code tmp_code = *code;

        if (shdisp_API_is_open()) {
            if (tmp_code.mode == SHDISP_DBG_MODE_LINUX) {
                tmp_code.mode = SHDISP_DBG_MODE_LINUX_BOOTED;
            }
            ret = shdisp_dbg_reset_log_output(&tmp_code, reset);
        } else {
            if (tmp_code.mode == SHDISP_DBG_MODE_LINUX) {
                tmp_code.mode = SHDISP_DBG_MODE_LINUX_BOOTING;
            }
            if (shdisp_dbg_boot_errcodes_count < SHDISP_DBG_BOOT_ERRCODE_NUM) {
                shdisp_dbg_boot_errcodes[shdisp_dbg_boot_errcodes_count] = tmp_code;
                shdisp_dbg_boot_errcodes_reset[shdisp_dbg_boot_errcodes_count] = reset;
                shdisp_dbg_boot_errcodes_count++;
            } else {
                SHDISP_ERR("Boot time error code buffer is full.");
            }
        }
    }
#endif /* SHDISP_RESET_LOG */
    SHDISP_TRACE("out");

    return ret;
}

#if defined(SHDISP_DBG_DUMP) || defined(SHDISP_RESET_LOG)
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_reset_work_alloc_command                                       */
/* ------------------------------------------------------------------------- */
static struct shdisp_dbg_reset_work_command *shdisp_dbg_reset_work_alloc_command(void)
{
    struct shdisp_dbg_reset_work_command *cmd;
    cmd = (struct shdisp_dbg_reset_work_command *)kzalloc(sizeof(struct shdisp_dbg_reset_work_command), GFP_KERNEL);
    if (cmd == NULL) {
        SHDISP_ERR("kzalloc() failure.");
    }
    return cmd;
}
#endif /* SHDISP_DBG_DUMP || SHDISP_RESET_LOG */

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_reset_work_free_command                                        */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_reset_work_free_command(struct shdisp_dbg_reset_work_command *cmd)
{
    if (cmd != NULL) {
        kfree(cmd);
    } else {
        SHDISP_ERR("null pointer.");
    }
}

#if defined(SHDISP_DBG_DUMP) || defined(SHDISP_RESET_LOG)
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_reset_work_add_command                                         */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_reset_work_add_command(struct shdisp_dbg_reset_work_command *cmd)
{
    SHDISP_DEBUG("add queue: proc=%pS arg=%p", cmd->proc, cmd->arg);

    spin_lock(&shdisp_dbg_reset_work_queue_lock);
    list_add_tail(&cmd->list, &shdisp_dbg_reset_work_queue);
    spin_unlock(&shdisp_dbg_reset_work_queue_lock);
}
#endif /* SHDISP_DBG_DUMP || SHDISP_RESET_LOG */

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_reset_work_worker                                              */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_reset_work_worker(struct work_struct *wk)
{
    struct list_head *item;
    struct shdisp_dbg_reset_work_command *cmd;
    int reset = 0;

    SHDISP_TRACE("in");

    for (; ;) {
        item = NULL;

        spin_lock(&shdisp_dbg_reset_work_queue_lock);
        if (!list_empty(&shdisp_dbg_reset_work_queue)) {
            item = shdisp_dbg_reset_work_queue.next;
            list_del(item);
        }
        spin_unlock(&shdisp_dbg_reset_work_queue_lock);

        if (item == NULL) {
            break;
        }

        cmd = list_entry(item, struct shdisp_dbg_reset_work_command, list);
        SHDISP_DEBUG("execute: proc=%pS arg=%p", cmd->proc, cmd->arg);
        cmd->proc(cmd->arg, &reset);

        shdisp_dbg_reset_work_free_command(cmd);
    }

#if defined(CONFIG_ANDROID_ENGINEERING)
    if (reset) {
        BUG();
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    SHDISP_TRACE("out");
}

#if defined(SHDISP_DBG_DUMP) || defined(SHDISP_RESET_LOG)
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_reset_work_start                                               */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_reset_work_start(void)
{
    SHDISP_TRACE("in");

    if (shdisp_dbg_reset_work_wq != NULL) {
        if (queue_work(shdisp_dbg_reset_work_wq, &shdisp_dbg_reset_work_wk) == 0) {
            SHDISP_DEBUG("work already pending.");
        }
    } else {
        SHDISP_ERR("workqueue not created.");
    }
}
#endif /* SHDISP_DBG_DUMP || SHDISP_RESET_LOG */

#ifdef SHDISP_DBG_DUMP
#include <linux/zlib.h>
#include <linux/crc32.h>
#include <linux/syscalls.h>
#include <linux/rtc.h>
#include <linux/vmalloc.h>

#define SHDISP_DBG_ZIP_WBITS MAX_WBITS
#define SHDISP_DBG_ZIP_MEM_LEVEL DEF_MEM_LEVEL
#define SHDISP_DBG_ZIP_IN_BUFLEN (512 * 1024)
#define SHDISP_DBG_ZIP_OUT_BUFLEN (150 * 1024)

struct shdisp_dbg_zip_context {
    char filename[64];
    int filename_len;
    u32 filetime;
    u32 filetimed;
    int in_len;
    int in_p_size;
    unsigned char *in_p;
    int out_len;
    int out_p_size;
    unsigned char *out_p;
    u32 crc32;
    z_stream zstrm;
    int zready;
    int ready;
};

int shdisp_dbg_zip_out_buflen_override = -1;
int shdisp_dbg_dispdump_worker_wait_ms = -1;

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_zip_wr1                                                        */
/* ------------------------------------------------------------------------- */
static inline size_t shdisp_dbg_zip_wr1(struct file *fp, u8 d)
{
    return shdisp_dbg_kernel_write(fp, (char *)&d, sizeof(d));
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_zip_wr2                                                        */
/* ------------------------------------------------------------------------- */
static inline size_t shdisp_dbg_zip_wr2(struct file *fp, u16 d)
{
    return shdisp_dbg_kernel_write(fp, (char *)&d, sizeof(d));
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_zip_wr4                                                        */
/* ------------------------------------------------------------------------- */
static inline size_t shdisp_dbg_zip_wr4(struct file *fp, u32 d)
{
    return shdisp_dbg_kernel_write(fp, (char *)&d, sizeof(d));
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_zip_set_fileinfo                                               */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_zip_set_fileinfo(struct shdisp_dbg_zip_context *ctx, const char *filename, struct timespec *ts)
{
    struct rtc_time tm;
    extern struct timezone sys_tz;

    ctx->filetime = ts->tv_sec;

    rtc_time_to_tm(ts->tv_sec - 60 * sys_tz.tz_minuteswest, &tm);

    strncpy(ctx->filename, filename, sizeof(ctx->filename) - 1);
    ctx->filename_len = strlen(ctx->filename);

    if (tm.tm_year >= 80) {
        u32 t = 0;
        t += (tm.tm_year - 80) << 25;
        t += (tm.tm_mon + 1)   << 21;
        t += (tm.tm_mday)      << 16;
        t += (tm.tm_hour)      << 11;
        t += (tm.tm_min)       << 5;
        t += (tm.tm_sec + 1)   >> 1;
        ctx->filetimed = t;
    } else {
        ctx->filetimed = 0;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_zip_write_head                                                 */
/* ------------------------------------------------------------------------- */
static u32 shdisp_dbg_zip_write_head(struct shdisp_dbg_zip_context *ctx, struct file *fp, u32 crc, u32 len, u32 clen)
{
    shdisp_dbg_zip_wr4(fp, 0x04034b50);
    shdisp_dbg_zip_wr2(fp, 20);
    shdisp_dbg_zip_wr2(fp, 0);
    shdisp_dbg_zip_wr2(fp, 8);
    shdisp_dbg_zip_wr4(fp, ctx->filetimed);
    shdisp_dbg_zip_wr4(fp, crc);
    shdisp_dbg_zip_wr4(fp, clen);
    shdisp_dbg_zip_wr4(fp, len);
    shdisp_dbg_zip_wr2(fp, ctx->filename_len);
    shdisp_dbg_zip_wr2(fp, 9);
    shdisp_dbg_kernel_write(fp, ctx->filename, ctx->filename_len);
    shdisp_dbg_zip_wr2(fp, 0x5455);
    shdisp_dbg_zip_wr2(fp, 5);
    shdisp_dbg_zip_wr1(fp, 1);
    shdisp_dbg_zip_wr4(fp, ctx->filetime);

    return 39 + ctx->filename_len;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_zip_write_tail1                                                */
/* ------------------------------------------------------------------------- */
static u32 shdisp_dbg_zip_write_tail1(struct shdisp_dbg_zip_context *ctx, struct file *fp, u32 crc, u32 len, u32 clen)
{
    shdisp_dbg_zip_wr4(fp, 0x02014b50);
    shdisp_dbg_zip_wr1(fp, 63);
    shdisp_dbg_zip_wr1(fp, 255);
    shdisp_dbg_zip_wr2(fp, 20);
    shdisp_dbg_zip_wr2(fp, 0);
    shdisp_dbg_zip_wr2(fp, 8);
    shdisp_dbg_zip_wr4(fp, ctx->filetimed);
    shdisp_dbg_zip_wr4(fp, crc);
    shdisp_dbg_zip_wr4(fp, clen);
    shdisp_dbg_zip_wr4(fp, len);
    shdisp_dbg_zip_wr2(fp, ctx->filename_len);
    shdisp_dbg_zip_wr2(fp, 9);
    shdisp_dbg_zip_wr2(fp, 0);
    shdisp_dbg_zip_wr2(fp, 0);
    shdisp_dbg_zip_wr2(fp, 0);
    shdisp_dbg_zip_wr4(fp, 0);
    shdisp_dbg_zip_wr4(fp, 0);
    shdisp_dbg_kernel_write(fp, ctx->filename, ctx->filename_len);
    shdisp_dbg_zip_wr2(fp, 0x5455);
    shdisp_dbg_zip_wr2(fp, 5);
    shdisp_dbg_zip_wr1(fp, 1);
    shdisp_dbg_zip_wr4(fp, ctx->filetime);

    return 55 + ctx->filename_len;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_zip_write_tail2                                                */
/* ------------------------------------------------------------------------- */
static u32 shdisp_dbg_zip_write_tail2(struct shdisp_dbg_zip_context *ctx, struct file *fp, u32 clen, u32 hlen,
                                                                                                     u32 tlen)
{
    shdisp_dbg_zip_wr4(fp, 0x06054b50);
    shdisp_dbg_zip_wr2(fp, 0);
    shdisp_dbg_zip_wr2(fp, 0);
    shdisp_dbg_zip_wr2(fp, 1);
    shdisp_dbg_zip_wr2(fp, 1);
    shdisp_dbg_zip_wr4(fp, tlen);
    shdisp_dbg_zip_wr4(fp, hlen + clen);
    shdisp_dbg_zip_wr2(fp, 0);

    return 22;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_zip_deinit                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_zip_deinit(struct shdisp_dbg_zip_context *ctx)
{
    int zres;

    if (ctx->zready) {
        zres = zlib_deflateEnd(&ctx->zstrm);
        if (zres != Z_OK) {
            SHDISP_ERR("zlib_deflateEnd() failure. ret=%d", zres);
        }
    }

    if (ctx->in_p != NULL) {
        vfree(ctx->in_p);
    }

    if (ctx->out_p != NULL) {
        vfree(ctx->out_p);
    }

    if (ctx->zstrm.workspace != NULL) {
        vfree(ctx->zstrm.workspace);
    }

    memset(ctx, 0, sizeof(*ctx));
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_zip_init                                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_zip_init(struct shdisp_dbg_zip_context *ctx, int in_buf_size, int out_buf_size)
{
    int zres;

    memset(ctx, 0, sizeof(*ctx));

    ctx->in_p_size = in_buf_size;
    ctx->in_p = vmalloc(in_buf_size);
    if (ctx->in_p == NULL) {
        SHDISP_ERR("vmalloc() failure. (input buffer)");
        goto error;
    }

    ctx->out_p_size = out_buf_size;
    ctx->out_p = vmalloc(out_buf_size);
    if (ctx->out_p == NULL) {
        SHDISP_ERR("vmalloc() failure. (output buffer)");
        goto error;
    }

    ctx->zstrm.workspace = vmalloc(zlib_deflate_workspacesize(SHDISP_DBG_ZIP_WBITS, SHDISP_DBG_ZIP_MEM_LEVEL));
    if (ctx->zstrm.workspace == NULL) {
        SHDISP_ERR("vmalloc() failure. (deflate workspace)");
        goto error;
    }

    zres = zlib_deflateInit2(
            &ctx->zstrm,
            Z_BEST_COMPRESSION,
            Z_DEFLATED,
            -(SHDISP_DBG_ZIP_WBITS),
            SHDISP_DBG_ZIP_MEM_LEVEL,
            Z_DEFAULT_STRATEGY);
    if (zres == Z_OK) {
        ctx->zready = 1;
    } else {
        SHDISP_ERR("zlib_deflateInit2() failure. ret=%d", zres);
        goto error;
    }

    ctx->ready = 1;

    return SHDISP_RESULT_SUCCESS;

error:
    shdisp_dbg_zip_deinit(ctx);

    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_zip_write                                                      */
/* ------------------------------------------------------------------------- */
static ssize_t shdisp_dbg_zip_write(struct shdisp_dbg_zip_context *ctx, const char *buf, size_t len)
{
    if (ctx->ready) {
        int remain = ctx->in_p_size - ctx->in_len;
        int l = len < remain ? len : remain;
        if (l < len) {
            SHDISP_ERR("not enough input buffer. remain=%d data=%d", remain, len);
        }
        memcpy(ctx->in_p + ctx->in_len, buf, l);
        ctx->in_len += l;
        return l;
    } else {
        return -1;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_zip_compress                                                   */
/* ------------------------------------------------------------------------- */
static size_t shdisp_dbg_zip_compress(struct shdisp_dbg_zip_context *ctx)
{
    if (ctx->ready) {
        int zres;

        ctx->crc32 = ~crc32(~0, ctx->in_p, ctx->in_len);
        ctx->zstrm.next_in = ctx->in_p;
        ctx->zstrm.avail_in = ctx->in_len;
        ctx->zstrm.total_in = 0;
        ctx->zstrm.next_out = ctx->out_p;
        ctx->zstrm.avail_out = ctx->out_p_size;
        ctx->zstrm.total_out = 0;

        zres = zlib_deflate(&ctx->zstrm, Z_FINISH);
        if (zres == Z_STREAM_END) {
            ctx->out_len = ctx->zstrm.total_out;
            return SHDISP_RESULT_SUCCESS;
        } else if (zres == Z_OK) {
            SHDISP_ERR("not enough output buffer.");
            return SHDISP_RESULT_FAILURE;
        } else {
            SHDISP_ERR("zlib_deflate() failure. ret=%d", zres);
            return SHDISP_RESULT_FAILURE;
        }
    } else {
        SHDISP_ERR("shdisp_dbg_zip_context is not initialized.");
        return SHDISP_RESULT_FAILURE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_zip_output                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_zip_output(struct shdisp_dbg_zip_context *ctx, struct file *fp)
{
    if (ctx->ready) {
        u32 total, hlen, tlen1, tlen2;

        total = 0;
        total += (hlen = shdisp_dbg_zip_write_head(ctx, fp, ctx->crc32, ctx->in_len, ctx->out_len));
        total += shdisp_dbg_kernel_write(fp, ctx->out_p, ctx->out_len);
        total += (tlen1 = shdisp_dbg_zip_write_tail1(ctx, fp, ctx->crc32, ctx->in_len, ctx->out_len));
        total += (tlen2 = shdisp_dbg_zip_write_tail2(ctx, fp, ctx->out_len, hlen, tlen1));

        return total;
    } else {
        return -1;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_dispdump_check_dir                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_dispdump_check_dir(void)
{
    struct path path;
    int error;

    error = kern_path(SHDISP_DBG_DUMPFILE_DIR, 0, &path);
    if (error) {
        return error;
    }
    if (!path.dentry->d_inode) {
        return -ENOENT;
    }
    if (IS_RDONLY(path.dentry->d_inode)) {
        return -EROFS;
    }
    return SHDISP_RESULT_SUCCESS;
}

#ifndef  SHDISP_DBG_DUMPFILE_DO_RING
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_dispdump_check_max_file_number                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_dispdump_check_max_file_number(void)
{
    char *buf;
    struct nameidata ni;
    struct dentry *trap;
    int error;

    buf = __getname();
    if (!buf) {
        return -ENOMEM;
    }

    memset(&ni, 0, sizeof(struct nameidata));
    sprintf(buf, "%s/%s%03d", SHDISP_DBG_DUMPFILE_DIR, SHDISP_DBG_DUMPFILE_FNAME, SHDISP_DBG_DUMPFILE_NUM - 1);
    error = kern_path_parent(buf, &ni);
    __putname(buf);

    if (error == 0) {
        mutex_lock_nested(&ni.path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
        trap = lookup_one_len(ni.last.name, ni.path.dentry, ni.last.len);
        if (!IS_ERR_OR_NULL(trap) && trap->d_inode) {
            error =  -EEXIST;
        }
        dput(trap);
        mutex_unlock(&ni.path.dentry->d_inode->i_mutex);
        path_put(&ni.path);
    }
    return error;
}
#endif  /* SHDISP_DBG_DUMPFILE_DO_RING */

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_dispdump_file_rotation                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_dbg_dispdump_file_rotation(void)
{
    char *buf;
    struct nameidata ni;
    struct dentry *trap;
    struct dentry *d_from;
    struct dentry *d_to;
    int seqno;
    int error = -EINVAL;

    buf = __getname();
    if (!buf) {
        return -ENOMEM;
    }

#ifdef  SHDISP_DBG_DUMPFILE_DO_RING
    memset(&ni, 0, sizeof(struct nameidata));
    sprintf(buf, "%s/%s%03d", SHDISP_DBG_DUMPFILE_DIR, SHDISP_DBG_DUMPFILE_FNAME, SHDISP_DBG_DUMPFILE_NUM - 1);
    error = kern_path_parent(buf, &ni);
    if (error == 0) {
        mutex_lock_nested(&ni.path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
        trap = lookup_one_len(ni.last.name, ni.path.dentry, ni.last.len);
        if (!IS_ERR_OR_NULL(trap)) {
            if (trap->d_inode) {
                error = vfs_unlink(ni.path.dentry->d_inode, trap);
            }
            dput(trap);
        }
        mutex_unlock(&ni.path.dentry->d_inode->i_mutex);
        path_put(&ni.path);
    }

    error = 0;
#endif  /* SHDISP_DBG_DUMPFILE_DO_RING */

    for (seqno = SHDISP_DBG_DUMPFILE_NUM - 1; seqno > 0; seqno--) {
        memset(&ni, 0, sizeof(struct nameidata));

        sprintf(buf, "%s/%s%03d", SHDISP_DBG_DUMPFILE_DIR, SHDISP_DBG_DUMPFILE_FNAME, seqno - 1);
        error = kern_path_parent(buf, &ni);
        if (error) {
            continue;
        }

        trap = lock_rename(ni.path.dentry, ni.path.dentry);

        d_from = lookup_one_len(ni.last.name, ni.path.dentry, ni.last.len);
        if (IS_ERR_OR_NULL(d_from)) {
            unlock_rename(ni.path.dentry, ni.path.dentry);
            path_put(&ni.path);
            continue;
        }
        if (!d_from->d_inode) {
            dput(d_from);
            unlock_rename(ni.path.dentry, ni.path.dentry);
            path_put(&ni.path);
            continue;
        }

        sprintf(buf, "%s%03d", SHDISP_DBG_DUMPFILE_FNAME, seqno);
        d_to = lookup_one_len(buf, ni.path.dentry, strlen(buf));
        if (IS_ERR_OR_NULL(d_to)) {
            dput(d_from);
            unlock_rename(ni.path.dentry, ni.path.dentry);
            path_put(&ni.path);
            continue;
        }
        if (d_to->d_inode) {
            dput(d_from);
            dput(d_to);
            unlock_rename(ni.path.dentry, ni.path.dentry);
            path_put(&ni.path);
            continue;
        }

        error = vfs_rename(ni.path.dentry->d_inode, d_from, ni.path.dentry->d_inode, d_to);
        unlock_rename(ni.path.dentry, ni.path.dentry);

        dput(d_from);
        dput(d_to);
        path_put(&ni.path);

        if (error) {
            SHDISP_ERR("Dumpfile rename error errno=%d", error);
            break;
        }
    }

    __putname(buf);

    return error;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_dispdump_get_errmsg                                            */
/* ------------------------------------------------------------------------- */
static const char *shdisp_dbg_dispdump_get_errmsg(int err_code)
{
    static const char *dump_fixed_err[] = {
        "\n",
        " Heap Error\n",
        " NoOS\n",
    };

    switch (err_code) {
    case SHDISP_DBG_ERR_HEAP_NULL:
        return dump_fixed_err[1];
    case SHDISP_DBG_INFO_NO_OS:
        return dump_fixed_err[2];
    default:
        return dump_fixed_err[0];
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_dispdump_generate                                              */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_dispdump_generate(struct shdisp_dbg_zip_context *ctx,
                                         struct shdisp_dbg_dispdump_worker_params *params)
{
    static const char *dump_fixed_data[] = {
        "\n",
        "## eDRAM dump\n",
        "## SRAM dump\n",
        "## Ring buffer dump\n",
        "## Stack trace dump\n",
    };

    char buf[128];
    const int buflen = sizeof(buf);
    const char *errmsg;
    struct timeval tv;
    struct tm tm1, tm2;

    SHDISP_TRACE("in");

    do_gettimeofday(&tv);
    time_to_tm((time_t)tv.tv_sec, 0, &tm1);
    time_to_tm((time_t)tv.tv_sec, (sys_tz.tz_minuteswest * 60 * (-1)), &tm2);

    snprintf(buf, buflen, "%04d/%02d/%02d(%s), %02d:%02d:%02d, UTC +00h,  %04d/%02d/%02d(%s), %02d:%02d:%02d\n",
                          (int)(tm1.tm_year + 1900),
                           tm1.tm_mon + 1, tm1.tm_mday, WeekOfDay[tm1.tm_wday], tm1.tm_hour, tm1.tm_min, tm1.tm_sec,
                          (int)(tm2.tm_year + 1900),
                           tm2.tm_mon + 1, tm2.tm_mday, WeekOfDay[tm2.tm_wday], tm2.tm_hour, tm2.tm_min, tm2.tm_sec);
    shdisp_dbg_zip_write(ctx, buf, strlen(buf));

    shdisp_dbg_zip_write(ctx, dump_fixed_data[0], strlen(dump_fixed_data[0]));
    shdisp_dbg_zip_write(ctx, dump_fixed_data[3], strlen((char *)dump_fixed_data[3]));
    if (params->ring_dump_buf.ptr != NULL) {
        shdisp_dbg_zip_write(ctx,
           params->ring_dump_buf.ptr,
           params->ring_dump_buf.length);
    } else {
        if (params->mode == SHDISP_DBG_MODE_APPSBL) {
            errmsg = shdisp_dbg_dispdump_get_errmsg(SHDISP_DBG_INFO_NO_OS);
            shdisp_dbg_zip_write(ctx, errmsg, strlen(errmsg));
        } else {
            errmsg = shdisp_dbg_dispdump_get_errmsg(SHDISP_DBG_ERR_HEAP_NULL);
            shdisp_dbg_zip_write(ctx, errmsg, strlen(errmsg));
        }
    }

    shdisp_dbg_zip_write(ctx, dump_fixed_data[0], strlen(dump_fixed_data[0]));
    shdisp_dbg_zip_write(ctx, dump_fixed_data[4], strlen((char *)dump_fixed_data[4]));
    if (params->stack_dump_buf.ptr != NULL) {
        shdisp_dbg_zip_write(ctx,
           params->stack_dump_buf.ptr,
           params->stack_dump_buf.length);
    } else {
        if (params->mode == SHDISP_DBG_MODE_APPSBL) {
            errmsg = shdisp_dbg_dispdump_get_errmsg(SHDISP_DBG_INFO_NO_OS);
            shdisp_dbg_zip_write(ctx, errmsg, strlen(errmsg));
        } else {
            errmsg = shdisp_dbg_dispdump_get_errmsg(SHDISP_DBG_ERR_HEAP_NULL);
            shdisp_dbg_zip_write(ctx, errmsg, strlen(errmsg));
        }
    }

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_dispdump_worker                                                */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_dispdump_worker(void *arg, int *reset)
{
    struct file *fp;
    char *name = NULL;
    int error;
    int rotationflg = 0;
    struct shdisp_dbg_dispdump_worker_params *params;
    struct shdisp_dbg_zip_context ctx;
    int out_buf_len = shdisp_dbg_zip_out_buflen_override >= 0 ? shdisp_dbg_zip_out_buflen_override :
                                                                SHDISP_DBG_ZIP_OUT_BUFLEN;
    int retry_cnt;

    SHDISP_TRACE("in");

    params = (struct shdisp_dbg_dispdump_worker_params *)arg;

    if (shdisp_dbg_zip_init(&ctx, SHDISP_DBG_ZIP_IN_BUFLEN, out_buf_len) != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("shdisp_dbg_zip_init() failure.");
        goto out;
    }

    shdisp_dbg_dispdump_generate(&ctx, params);

    if (shdisp_dbg_zip_compress(&ctx) != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("shdisp_dbg_zip_compress() failure.");
        goto out;
    }

    name = __getname();
    if (!name) {
        SHDISP_ERR("Buffer allocation error.");
        goto out;
    }

    sprintf(name, "%s/%s000", SHDISP_DBG_DUMPFILE_DIR, SHDISP_DBG_DUMPFILE_FNAME);

    retry_cnt = 0;
    while (1) {
        error = shdisp_dbg_dispdump_check_dir();
        if (error) {
            SHDISP_ERR("FW timeout dumpfile output dir error[%d] continue", error);
            goto retry;
        }

#ifndef SHDISP_DBG_DUMPFILE_DO_RING
        error = shdisp_dbg_dispdump_check_max_file_number();
        if (error == -EEXIST) {
            SHDISP_ERR("FW timeout dumpfile is full. giveup");
            break;
        } else if (error) {
            SHDISP_ERR("shdisp_dbg_dispdump_check_max_file_number() error[%d] continue", error);
            goto retry;
        }
#endif /* SHDISP_DBG_DUMPFILE_DO_RING */

        if (!rotationflg) {
            error = shdisp_dbg_dispdump_file_rotation();
            if (error) {
                SHDISP_ERR("FW timeout dumpfile rotation error[%d]. continue.", error);
                goto retry;
            }
            rotationflg = 1;
        }

        fp = filp_open(name, O_WRONLY | O_CREAT | O_TRUNC, 0440);
        if (IS_ERR_OR_NULL(fp)) {
            if (PTR_ERR(fp) == -ENOENT) {
                SHDISP_ERR("FW timeout dumpfile open error [%ld] continue", PTR_ERR(fp));
                goto retry;
            }
            SHDISP_ERR("FW timeout dumpfile open error [%ld] giveup", PTR_ERR(fp));
            break;
        } else {
            struct timespec ts;

            if (shdisp_dbg_dispdump_worker_wait_ms > 0) {
                msleep(shdisp_dbg_dispdump_worker_wait_ms);
            }

            getnstimeofday(&ts);
            shdisp_dbg_zip_set_fileinfo(&ctx, SHDISP_DBG_DUMPFILE_INNER_FNAME, &ts);
            shdisp_dbg_zip_output(&ctx, fp);
            {
                int res;
                if ((res = shdisp_dbg_kernel_sync(fp)) != 0) {
                    SHDISP_ERR("fsync result: %d", res);
                }
            }
            filp_close(fp, NULL);
            SHDISP_DEBUG("Output FW dumpfile Success.");
            break;
        }

retry:
        retry_cnt++;
        if (retry_cnt >= SHDISP_DBG_DUMPFILE_MAX_RETRY) {
            SHDISP_ERR("Output FW dumpfile retry over.");
            break;
        }
        SHDISP_WARN("Output FW dumpfile retry(%d)", retry_cnt);
        msleep(1000);
    }

out:
    shdisp_dbg_zip_deinit(&ctx);

    if (name) {
        __putname(name);
    }
    if (params->ring_dump_buf.ptr != NULL && params->ring_dump_buf.need_free) {
        kfree(params->ring_dump_buf.ptr);
    }
    if (params->stack_dump_buf.ptr != NULL && params->stack_dump_buf.need_free) {
        kfree(params->stack_dump_buf.ptr);
    }
    if (params->edram_dump_buf.ptr != NULL && params->edram_dump_buf.need_free) {
        kfree(params->edram_dump_buf.ptr);
    }
    if (params->sram_dump_buf.ptr != NULL && params->sram_dump_buf.need_free) {
        kfree(params->sram_dump_buf.ptr);
    }

    if ((shdisp_dbg_API_get_reset_flg() & SHDISP_DBG_RESET_CLMR) && params->reset) {
#if defined(CONFIG_ANDROID_ENGINEERING)
    if ((shdisp_dbg_API_get_reset_flg() & SHDISP_DBG_DUMP_RESET) && params->reset) {
        *reset = 1;
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    kfree(params);

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_API_display_dump                                               */
/* ------------------------------------------------------------------------- */
int shdisp_dbg_API_display_dump(
        int mode,
        struct shdisp_dbg_ptrinfo *ring_dump_buf,
        struct shdisp_dbg_ptrinfo *stack_dump_buf,
        struct shdisp_dbg_ptrinfo *edram_dump_buf,
        struct shdisp_dbg_ptrinfo *sram_dump_buf)
{
    struct shdisp_dbg_reset_work_command *cmd = NULL;
    struct shdisp_dbg_dispdump_worker_params *params = NULL;
    int error = -EINVAL;

    SHDISP_TRACE("in");

    params = (struct shdisp_dbg_dispdump_worker_params *)kzalloc(sizeof(struct shdisp_dbg_dispdump_worker_params),
                                                                GFP_KERNEL);
    if (params == NULL) {
        SHDISP_ERR("allocate parameter error. [no memory]");
        error = -ENOMEM;
        goto errout;
    }

    params->reset = shdisp_API_is_open() ? 1 : 0;
    params->mode = mode;
    if (ring_dump_buf != NULL) {
        params->ring_dump_buf = *ring_dump_buf;
    }
    if (stack_dump_buf != NULL) {
        params->stack_dump_buf = *stack_dump_buf;
    }
    if (edram_dump_buf != NULL) {
        params->edram_dump_buf = *edram_dump_buf;
    }
    if (sram_dump_buf != NULL) {
        params->sram_dump_buf = *sram_dump_buf;
    }

    cmd = shdisp_dbg_reset_work_alloc_command();
    if (cmd == NULL) {
        SHDISP_ERR("allocate command error. [no memory]");
        error = -ENOMEM;
        goto errout;
    }

    cmd->proc = shdisp_dbg_dispdump_worker;
    cmd->arg = params;

    shdisp_dbg_reset_work_add_command(cmd);
    shdisp_dbg_reset_work_start();

    SHDISP_TRACE("out normaly finished.");
    return SHDISP_RESULT_SUCCESS;

errout:
    if (cmd != NULL) {
        shdisp_dbg_reset_work_free_command(cmd);
    }

    if (params != NULL) {
        kfree(params);
    }

    SHDISP_ERR("abnormaly finished. error=%d", error);
    return error;
}
#endif /* SHDISP_DBG_DUMP */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
