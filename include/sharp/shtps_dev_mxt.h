#ifndef __SHTPS_DEV_MXT_H__
#define __SHTPS_DEV_MXT_H__

/* Following definitions are declared only to avoid build error.
 * TODO: fix them correctly
 */
/* -----------------------------------------------------------------------------------
 */
#include <linux/input/mt.h>

#define SH_TOUCH_DEVNAME	"shtps_rmi"
#define SH_TOUCH_IF_DEVNAME	"shtpsif"
#define SH_TOUCH_IF_DEVPATH	"/dev/shtpsif"

#if defined( CONFIG_SHTPS_ATMEL_MXT336T )
	#define SHTPS_TM_TXNUM_MAX		24
	#define SHTPS_TM_RXNUM_MAX		14
#else
	#define SHTPS_TM_TXNUM_MAX		24
	#define SHTPS_TM_RXNUM_MAX		14
#endif

#define SHTPS_FINGER_MAX	10

#define TPS_IOC_MAGIC					0xE0

#define TPSDEV_ENABLE                           _IO  ( TPS_IOC_MAGIC,  1)
#define TPSDEV_DISABLE                          _IO  ( TPS_IOC_MAGIC,  2)
#define TPSDEV_RESET                            _IO  ( TPS_IOC_MAGIC,  3)
//#define TPSDEV_SOFT_RESET                       _IO  ( TPS_IOC_MAGIC,  4)
//#define TPSDEV_GET_FW_VERSION                   _IOR ( TPS_IOC_MAGIC,  5, unsigned short)
#define TPSDEV_ENTER_BOOTLOADER                _IOW ( TPS_IOC_MAGIC,  6, int)
#define TPSDEV_EXIT_BOOTLOADER                 _IOW ( TPS_IOC_MAGIC,  7, int)
//#define TPSDEV_ERASE_FLASE                      _IO  ( TPS_IOC_MAGIC,  8)
//#define TPSDEV_WRITE_IMAGE                      _IOW ( TPS_IOC_MAGIC,  9, struct shtps_ioctl_param)
//#define TPSDEV_WRITE_CONFIG                     _IOW ( TPS_IOC_MAGIC, 10, struct shtps_ioctl_param)
#define TPSDEV_GET_TOUCHINFO                    _IOR ( TPS_IOC_MAGIC, 11, struct shtps_input_event_info)
#define TPSDEV_GET_TOUCHINFO_NOWAIT             _IOR ( TPS_IOC_MAGIC, 12, struct shtps_input_event_info)
#define TPSDEV_CANCEL_GET_TOUCHINFO             _IO ( TPS_IOC_MAGIC, 13)
//#define TPSDEV_GET_TOUCHINFO_UNTRANS		_IOR ( TPS_IOC_MAGIC, 12, struct shtps_touch_info)
//#define TPSDEV_SET_TOUCHMONITOR_MODE		_IOW ( TPS_IOC_MAGIC, 13, unsigned char)
//#define TPSDEV_READ_REG                         _IOWR( TPS_IOC_MAGIC, 14, struct shtps_ioctl_param)
//#define TPSDEV_READ_ALL_REG                     _IOR ( TPS_IOC_MAGIC, 15, struct shtps_ioctl_param)
//#define TPSDEV_WRITE_REG                        _IOW ( TPS_IOC_MAGIC, 16, struct shtps_ioctl_param)
//#define TPSDEV_START_TM                         _IOW ( TPS_IOC_MAGIC, 17, struct shtps_ioctl_param)
//#define TPSDEV_STOP_TM                          _IO  ( TPS_IOC_MAGIC, 18)
//#define TPSDEV_GET_BASELINE                     _IOR ( TPS_IOC_MAGIC, 19, unsigned short*)
//#define TPSDEV_GET_FRAMELINE                    _IOR ( TPS_IOC_MAGIC, 20, unsigned char*)
#define TPSDEV_START_FACETOUCHMODE              _IO  ( TPS_IOC_MAGIC, 21)
#define TPSDEV_STOP_FACETOUCHMODE               _IO  ( TPS_IOC_MAGIC, 22)
#define TPSDEV_POLL_FACETOUCHOFF                _IO  ( TPS_IOC_MAGIC, 23)
//#define TPSDEV_GET_FWSTATUS                     _IOR ( TPS_IOC_MAGIC, 24, unsigned char)
//#define TPSDEV_GET_FWDATE                       _IOR ( TPS_IOC_MAGIC, 25, unsigned short)
#define TPSDEV_CALIBRATION_PARAM                _IOW ( TPS_IOC_MAGIC, 26, struct shtps_ioctl_param)
//#define TPSDEV_DEBUG_REQEVENT                   _IOW ( TPS_IOC_MAGIC, 27, int)
//#define TPSDEV_SET_DRAGSTEP                     _IOW ( TPS_IOC_MAGIC, 28, int)
//#define TPSDEV_SET_POLLINGINTERVAL              _IOW ( TPS_IOC_MAGIC, 29, int)
//#define TPSDEV_SET_FINGERFIXTIME                _IOW ( TPS_IOC_MAGIC, 30, int)
//#define TPSDEV_REZERO                           _IO  ( TPS_IOC_MAGIC, 31)
#define TPSDEV_ACK_FACETOUCHOFF                 _IO  ( TPS_IOC_MAGIC, 32)
//#define TPSDEV_START_TM_F05                     _IOW ( TPS_IOC_MAGIC, 33, int)
//#define TPSDEV_SET_DRAGSTEP_X                   _IOW ( TPS_IOC_MAGIC, 34, int)
//#define TPSDEV_SET_DRAGSTEP_Y                   _IOW ( TPS_IOC_MAGIC, 35, int)
#define TPSDEV_LOGOUTPUT_ENABLE                 _IOW ( TPS_IOC_MAGIC, 36, int)
#define TPSDEV_GET_TOUCHKEYINFO                 _IOR ( TPS_IOC_MAGIC, 37, struct shtps_touch_key_info)
//#define TPSDEV_GET_FW_VERSION_BUILTIN		_IOR ( TPS_IOC_MAGIC, 38, unsigned short)
#define TPSDEV_GET_SMEM_BASELINE                _IOR ( TPS_IOC_MAGIC, 39, unsigned short*)
#define TPSDEV_SET_LOWPOWER_MODE                _IOW ( TPS_IOC_MAGIC, 40, int)
#define TPSDEV_SET_CONT_LOWPOWER_MODE		_IOW ( TPS_IOC_MAGIC, 41, int)
//#define TPSDEV_SET_INVALID_AREA                 _IOW ( TPS_IOC_MAGIC, 42, int)
#define TPSDEV_SET_CHARGER_ARMOR                _IOW(TPS_IOC_MAGIC, 43, int)
#define TPSDEV_SET_WIRELESS_CHARGER_ARMOR       _IOW(TPS_IOC_MAGIC, 44, int)
#define TPSDEV_LPWG_ENABLE			_IOW(TPS_IOC_MAGIC, 45, int)
#define TPSDEV_SET_VEILVIEW_STATE               _IOW ( TPS_IOC_MAGIC, 46, int)
//#define TPSDEV_READ_REG_BLOCK                   _IOWR( TPS_IOC_MAGIC, 47, struct shtps_ioctl_param)
//#define TPSDEV_WRITE_REG_BLOCK                  _IOW ( TPS_IOC_MAGIC, 48, struct shtps_ioctl_param)
#define TPSDEV_GET_VEILVIEW_PATTERN             _IOR ( TPS_IOC_MAGIC, 49, int)
//#define TPSDEV_READ_REG_PACKET                  _IOWR( TPS_IOC_MAGIC, 50, struct shtps_ioctl_param)
//#define TPSDEV_WRITE_REG_PACKET                 _IOW ( TPS_IOC_MAGIC, 51, struct shtps_ioctl_param)
#define TPSDEV_HOVER_ENABLE                     _IOW ( TPS_IOC_MAGIC, 52, int)
//#define TPSDEV_GET_BASELINE_RAW                 _IOR ( TPS_IOC_MAGIC, 53, unsigned short*)
#define TPSDEV_CALIBRATION_PEN_PARAM		_IOW ( TPS_IOC_MAGIC, 54, struct shtps_ioctl_param)
#define TPSDEV_SET_NARROW_FRAME_MODE		_IOW ( TPS_IOC_MAGIC, 55, int)
#define TPSDEV_SET_LCD_LOWPOWER_MODE		_IOW ( TPS_IOC_MAGIC, 56, int)
//#define TPSDEV_BASELINE_OFFSET_DISABLE		_IOW ( TPS_IOC_MAGIC, 57, int)
#define TPSDEV_CHECK_CRC_ERROR			_IOW ( TPS_IOC_MAGIC, 58, int)
#define TPSDEV_SET_PEN_ENABLE                   _IOW ( TPS_IOC_MAGIC, 59, int)
#define TPSDEV_GET_PEN_ENABLE                   _IOR ( TPS_IOC_MAGIC, 60, int)
//#define TPSDEV_SET_LOW_REPORTRATE_MODE	_IOR ( TPS_IOC_MAGIC, 61, int)
//#define TPSDEV_GET_SERIAL_NUMBER		_IOR ( TPS_IOC_MAGIC, 62, unsigned char*)
//#define TPSDEV_GET_SERIAL_NUMBER_SIZE	_IOR ( TPS_IOC_MAGIC, 63, int)
#define TPSDEV_GET_UPDATE_FLG                   _IOR ( TPS_IOC_MAGIC, 64, int)
#define TPSDEV_GET_HW_TYPE                      _IOR ( TPS_IOC_MAGIC, 65, int)
#define TPSDEV_GET_HW_REVISION                  _IOR ( TPS_IOC_MAGIC, 66, int)


#define TPSDEV_FACETOUCHOFF_NOCHG       0x00
#define TPSDEV_FACETOUCHOFF_DETECT      0x01

#define TPSDEV_TOUCHINFO_MODE_LCDSIZE   0
#define TPSDEV_TOUCHINFO_MODE_DEVSIZE   1

#define TPSDEV_BOOTLOADER_FW			0
#define TPSDEV_BOOTLOADER_CFG			1

enum{
	SHTPS_VEILVIEW_PATTERN_RGB_CHIDORI_1H = 0,
	SHTPS_VEILVIEW_PATTERN_RGB_CHIDORI_2H,
	SHTPS_VEILVIEW_PATTERN_MONOCHROME_1H,
	SHTPS_VEILVIEW_PATTERN_MONOCHROME_2H,
};

enum{
	SHTPS_DEV_STATE_SLEEP = 0,
	SHTPS_DEV_STATE_DOZE,
	SHTPS_DEV_STATE_ACTIVE,
	SHTPS_DEV_STATE_LPWG,
	SHTPS_DEV_STATE_LOADER,
};

struct shtps_bootloader_info {
	unsigned long	block_size;
	unsigned long	program_block_num;
	unsigned long	config_block_num;
};

#define TPSDEV_INPUT_ITEM_INVALID		(256)

struct fingers{
	int				state;
	int				x;
	int				y;
	int				w;
	int				z;
	int				orientation;
};

struct shtps_input_event_info{
	struct{
		 struct fingers	pre[SHTPS_FINGER_MAX];
		 struct fingers	cur[SHTPS_FINGER_MAX];
	} fw;
	
	struct{
		 struct fingers	pre[SHTPS_FINGER_MAX];
		 struct fingers	cur[SHTPS_FINGER_MAX];
	} report;
};

struct shtps_ioctl_param {
	int		size;
	unsigned char*	data;
};

struct shtps_touch_key_info {
	unsigned char	menu_key_state;
	unsigned char	home_key_state;
	unsigned char	back_key_state;
	unsigned char	down_key_state;
	unsigned char	up_key_state;
};

struct shtps_lpwg_info {
	unsigned char	idleacqint;
	unsigned char	actvacqint;
	unsigned char	*symdata;
	unsigned int	symdata_size;
	int				grip_sup_x_mm;
	int				grip_sup_y_mm;
	int				mov_mm;
	int				sym_time_max_ms;
};


/* -----------------------------------------------------------------------------------
 */
#define SHTPS_DIAG_POLL_TIMEOUT		(1000)

/* -----------------------------------------------------------------------------------
 */
/* for atmel_mxt_ts */
extern void shtps_mxt_cust_initialize(struct device *dev);
extern void shtps_mxt_cust_deinitialize(void);
extern void shtps_mxt_add_input_event(int id, int state, int tool, int x, int y, int w_enable, int w, int z_enable, int z, int ori_enable, int orientation);
extern void shtps_mxt_input_sync(struct input_dev *input_dev);
extern void shtps_mxt_input_force_touchup(struct input_dev *input_dev);
extern void shtps_mxt_notify_devstate(int new_state);
extern void shtps_mxt_lpwg_wakelock(int on);
extern void shtps_mxt_lpwg_wakeup(void);
extern void shtps_mxt_wake_lock_idle(void);
extern void shtps_mxt_wake_unlock_idle(void);
extern int shtps_mxt_tpin_enable_check(void);
extern int shtps_mxt_get_lpmode_setting(u8 *active_value, u8 *idle_value);

extern void shtps_mxt_notify_make_input_dev(struct input_dev *input_dev);
extern void shtps_mxt_notify_free_input_dev(void);

/* for other module */
extern void msm_tps_setsleep(int on);
extern int msm_tps_set_veilview_state_on(void);
extern int msm_tps_set_veilview_state_off(void);
extern int msm_tps_get_veilview_pattern(void);
extern void msm_tps_set_grip_state(int on);

#endif	/* __SHTPS_DEV_MXT_H__ */
