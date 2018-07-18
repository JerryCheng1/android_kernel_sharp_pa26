ifneq (, $(filter F%, $(SH_BUILD_ID)))
EXTRA_CFLAGS += -DSHDISP_FACTORY_MODE_ENABLE
endif
obj-$(CONFIG_SHDISP) += shdisp_kerl.o
obj-$(CONFIG_SHDISP) += shdisp_system.o
obj-$(CONFIG_SHDISP) += shdisp_bl71y6.o
obj-$(CONFIG_SHDISP) += shdisp_pm.o
obj-$(CONFIG_SHDISP) += shdisp_panel.o
ifeq ($(CONFIG_SHDISP_PANEL_CARIN),y)
obj-$(CONFIG_SHDISP) += shdisp_carin.o
endif
ifeq ($(CONFIG_SHDISP_PANEL_ANDY),y)
obj-$(CONFIG_SHDISP) += shdisp_andy.o
endif
ifeq ($(CONFIG_SHDISP_PANEL_ARIA),y)
obj-$(CONFIG_SHDISP) += shdisp_aria.o
endif
obj-$(CONFIG_SHDISP) += shdisp_dbg.o
#obj-$(CONFIG_SHDISP) += shlcdc_eventlog.o
EXTRA_CFLAGS    += -DSH_BUILD_ID=\"$(SH_BUILD_ID)\"
EXTRA_CFLAGS    += -DUSE_LINUX
