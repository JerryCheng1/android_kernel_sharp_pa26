ifneq (, $(filter F%, $(SH_BUILD_ID)))
	EXTRA_CFLAGS += -DSHTPS_FACTORY_MODE_ENABLE
endif

ifeq ($(TARGET_BUILD_VARIANT),eng)
	EXTRA_CFLAGS += -DSHTPS_ENGINEER_BUILD_ENABLE
endif

EXTRA_CFLAGS += -I$(src)/..

obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_absorption_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_cling_reject_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_coaxial_ghost_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_correct.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_detect_ghost.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_diagonal_ghost_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_dynamic_reset_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_hover_jitter_filter.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_multitap_fail_move_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_multitouch_pen_ghost_rejection_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_multi_hover_select.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_pen_cling_reject_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_pen_jitter_filter_strength_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_pen_pos_jump_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_pinchout_outset_distort_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_topedge_fail_touch_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_touch_position_adjust.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_water_cling_reject_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_water_ghost_reject_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_electrical_ghost_reject_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_topofscreen_ghost_reject_check.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_inverting_ghost_check.o


obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_drag_s3400.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_charger_armor_s3400.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_edge_fail_touch_check_s3400.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_grip_fail_flick_check_s3400.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_lgm_split_touch_combining_check_s3400.o
obj-$(CONFIG_SHTPS_SY3X00_DEV)	+= 	shtps_filter_pinch_fail_response_check_s3400.o
