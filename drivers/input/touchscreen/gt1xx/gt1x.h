/* drivers/input/touchscreen/gt1x.h
 *
 * 2010 - 2013 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 1.4
 * Release Date:  2015/07/23
 */

#ifndef _GOODIX_GT1X_H_
#define _GOODIX_GT1X_H_
#include "gt1x_generic.h"
#include <linux/gpio.h>
#ifdef GTP_CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#endif
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "../ts_func_test.h"

#define IIC_MAX_TRANSFER_SIZE	   250

/* Customize your I/O ports & I/O operations */
#ifdef GTP_CONFIG_OF
extern int gt1x_rst_gpio;
extern int gt1x_int_gpio;
#define GTP_RST_PORT gt1x_rst_gpio
#define GTP_INT_PORT gt1x_int_gpio
#else
#define GTP_RST_PORT	102
#define GTP_INT_PORT	52
#endif

#define GTP_INT_IRQ	 gpio_to_irq(GTP_INT_PORT)

#define GTP_GPIO_AS_INPUT(pin)		  do {\
											gpio_direction_input(pin);\
										} while (0)
#define GTP_GPIO_AS_INT(pin)			do {\
											GTP_GPIO_AS_INPUT(pin);\
										} while (0)
#define GTP_GPIO_GET_VALUE(pin)		 gpio_get_value(pin)
#define GTP_GPIO_OUTPUT(pin, level)	  gpio_direction_output(pin, level)
#define GTP_IRQ_TAB					 {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH}

#define GT1X_VTG_MIN_UV			2600000
#define GT1X_VTG_MAX_UV			3300000
#define GT1X_I2C_VTG_MIN_UV		1800000
#define GT1X_I2C_VTG_MAX_UV		1800000
#define GT1X_VDD_LOAD_MIN_UA	0
#define GT1X_VDD_LOAD_MAX_UA	10000
#define GT1X_VIO_LOAD_MIN_UA	0
#define GT1X_VIO_LOAD_MAX_UA	10000
#define PROP_NAME_SIZE			24
#define GOODIX_MAX_CFG_GROUP	6
#define MAX_GESTURE				15
#define GESTURE_MAX_POINT_COUNT	64


struct goodix_ts_map{
	unsigned char nbuttons;
	unsigned int *map;
};

#ifdef CONFIG_TOUCHSCREEN_GT1X_GESTURE
#pragma pack(1)
typedef struct {
	u8 ic_msg[6];		/*from the first byte */
	u8 gestures[4];
	u8 data[3 + GESTURE_MAX_POINT_COUNT * 4 + 80];
} st_gesture_data;
#pragma pack()
#endif

struct goodix_ts_platform_data {
	int irq_gpio;
	u32 irq_gpio_flags;
	int reset_gpio;
	u32 reset_gpio_flags;
	u32 config_data_len[GOODIX_MAX_CFG_GROUP];
	u8 *config_data[GOODIX_MAX_CFG_GROUP];
	u32 test_data_len[GOODIX_MAX_CFG_GROUP];
	u8 *test_data[GOODIX_MAX_CFG_GROUP];
	u32 chr_data_len[GOODIX_MAX_CFG_GROUP];
	u8 *chr_data[GOODIX_MAX_CFG_GROUP];
	struct goodix_ts_map *gesture_map;
	struct goodix_ts_map *gesture_key;
	struct goodix_ts_map *button_map;
	u32 touch_area_param;
};

struct goodix_ts_data {
	struct goodix_ts_platform_data *pdata;
	spinlock_t irq_lock;
	struct i2c_client *client;
	struct input_dev  *input_dev;
	struct work_struct  work;

	struct regulator *avdd;
	struct regulator *vdd;
	struct regulator *vcc_i2c;

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	struct work_struct resume_work;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	struct ts_func_test_device ts_test_dev;

	u16 config_version;
	u16 ic_config_version;
	u16 ic_fw_version;
	u16 fw_version;
	u8  sensor_id;
	bool fw_updating;
	bool is_suspended;

#ifdef CONFIG_TOUCHSCREEN_GT1X_GESTURE
	bool gesture_enabled;
	bool gesture_wakeup_enable_pre;
	unsigned int gesture_state;
	u16 gesture_track_x[GESTURE_MAX_POINT_COUNT];
	u16 gesture_track_y[GESTURE_MAX_POINT_COUNT];
	short gesture_track_pointnum;
#endif
#ifdef CONFIG_TOUCHSCREEN_GT1X_FH
	bool usb_is_plugin;
#endif
};

int gt1x_power_init(struct goodix_ts_data *ts, bool on);
int gt1x_power_on(struct goodix_ts_data *ts, bool on);

extern s32 gtp_test_sysfs_init(void);
extern void gtp_test_sysfs_deinit(void);

#endif /* _GOODIX_GT1X_H_ */
