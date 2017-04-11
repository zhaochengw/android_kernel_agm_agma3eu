#
# Makefile for the Focaltech touchscreen drivers.
#

obj-$(CONFIG_TOUCHSCREEN_FT5X06)	+= focaltech_ts.o
mydir := $(dir $(lastword $(MAKEFILE_LIST)))

ifeq ($(strip $(CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER)),y)
    ifeq ($(strip $(CONFIG_ARM64)),y)
        ftlib := $(mydir)tslib/ft_gesture_lib_64.a
    else
        ftlib := $(mydir)tslib/ft_gesture_lib.a
    endif
endif
ifeq ($(strip $(CONFIG_ARM64)),y)
    ftlib += $(mydir)tslib/libmcaptest_64.a
else
    ftlib += $(mydir)tslib/libmcaptest.a
endif
ldflags-y += -L$(mydir) --whole-archive $(ftlib) --no-whole-archive
