ifneq (, $(filter F%, $(SH_BUILD_ID)))
	EXTRA_CFLAGS += -DSHTPS_FACTORY_MODE_ENABLE
endif

ifeq ($(TARGET_BUILD_VARIANT),eng)
	EXTRA_CFLAGS += -DSHTPS_ENGINEER_BUILD_ENABLE
endif

EXTRA_CFLAGS += -I$(src)/..

obj-$(CONFIG_SHTPS_MXT_DEV)	+= 	shtps_filter_pos_scaling.o
obj-$(CONFIG_SHTPS_MXT_DEV)	+= 	shtps_filter_pos_offset.o
