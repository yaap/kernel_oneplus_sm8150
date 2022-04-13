#
# Makefile for the touchscreen drivers.
#
ccflags-y += -mllvm -inline-threshold=15000
ccflags-y += -mllvm -inlinehint-threshold=10000
subdir-ccflags-$(CONFIG_TOUCHPANEL_MTK_PLATFORM) += -I$(srctree)/drivers/misc/mediatek/include/
subdir-ccflags-y += -Wunused-variable 
ifneq ($(cc-name),clang)
subdir-ccflags-y += -Wunused-but-set-variable -Wunused-const-variable=2
endif

obj-$(CONFIG_TOUCHPANEL_ONEPLUS) += touchpanel_common_driver.o
obj-$(CONFIG_TOUCHPANEL_SYNAPTICS) += Synaptics/
obj-$(CONFIG_TOUCHPANEL_FOCAL) += Focal/
obj-$(CONFIG_TOUCHPANEL_GOODIX) += Goodix/
obj-$(CONFIG_TOUCHPANEL_NOVA) += Novatek/
obj-$(CONFIG_TOUCHPANEL_HIMAX) += Himax/
obj-$(CONFIG_TOUCHPANEL_SAMSUNG) += Samsung/
obj-$(CONFIG_TOUCHPANEL_ST) += St/

