ifneq (, $(filter F%, $(SH_BUILD_ID)))
	EXTRA_CFLAGS += -DSHTPS_FACTORY_MODE_ENABLE
endif

ifeq ($(TARGET_BUILD_VARIANT),eng)
	EXTRA_CFLAGS += -DSHTPS_ENGINEER_BUILD_ENABLE
endif

obj-$(CONFIG_SHTPS_MXT_DEV)	+= 	shtps_mxt.o				\
								shtps_mxt_sub.o			\
								shtps_mxt_ldm.o			\
								shtps_fwctl.o			\
								shtps_filter.o			\
								shtps_mxt_debug.o

obj-$(CONFIG_SHTPS_MXT_DEV)	+= 	filter/
