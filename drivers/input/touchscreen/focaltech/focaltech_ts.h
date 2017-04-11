/*
 *
 * FocalTech ft5x06 TouchScreen driver header file.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/vmalloc.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif
#include <linux/unistd.h>
#include <asm/uaccess.h>
#include "../ts_func_test.h"

#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
#include <linux/sensors.h>
#include <linux/wakelock.h>
#endif

#define FT5X06_ID		0x55
#define FT5X16_ID		0x0A
#define FT5X36_ID		0x14
#define FT6X06_ID		0x06
#define FT6X36_ID       0x36
#define FT_DRIVER_VERSION	0x02

#define FT_META_REGS		3
#define FT_ONE_TCH_LEN		6
#define FT_TCH_LEN(x)		(FT_META_REGS + FT_ONE_TCH_LEN * x)

#define FT_PRESS		0x7F
#define FT_MAX_ID		0x0F
#define FT_TOUCH_X_H_POS	3
#define FT_TOUCH_X_L_POS	4
#define FT_TOUCH_Y_H_POS	5
#define FT_TOUCH_Y_L_POS	6
#define FT_TOUCH_WEIGHT_POS	7
#define FT_TOUCH_AREA_POS	8

#define FT_TD_STATUS		2
#define FT_TOUCH_EVENT_POS	3
#define FT_TOUCH_ID_POS		5
#define FT_TOUCH_DOWN		0
#define FT_TOUCH_CONTACT	2

/*register address*/
#define FT_REG_DEV_MODE		0x00
#define FT_DEV_MODE_REG_CAL	0x02
#define FT_REG_ID		0xA3
#define FT_REG_PMODE		0xA5
#define FT_REG_FW_VER		0xA6
#define FT_RGE_PANNEL_ID 		0xA8
#define FT_REG_POINT_RATE	0x88
#define FT_REG_THGROUP		0x80
#define FT_REG_ECC		0xCC
#define FT_REG_RESET_FW		0x07
/*#define FT_REG_FW_MAJ_VER	0xB1*/
#define FT_REG_FW_MAJ_VER		0xA6
#define FT_REG_FW_MIN_VER	0xB2
#define FT_REG_FW_SUB_MIN_VER	0xB3

/* power register bits*/
#define FT_PMODE_ACTIVE		0x00
#define FT_PMODE_MONITOR	0x01
#define FT_PMODE_STANDBY	0x02
#define FT_PMODE_HIBERNATE	0x03
#define FT_FACTORYMODE_VALUE	0x40
#define FT_WORKMODE_VALUE	0x00
#define FT_RST_CMD_REG1		0xFC
#define FT_RST_CMD_REG2		0xBC
#define FT_READ_ID_REG		0x90
#define FT_ERASE_APP_REG	0x61
#define FT_ERASE_PANEL_REG	0x63
#define FT_FW_START_REG		0xBF

#define FT_STATUS_NUM_TP_MASK	0x0F

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

#define FT_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define FT_8BIT_SHIFT		8
#define FT_4BIT_SHIFT		4
#define FT_FW_NAME_MAX_LEN	128

#define FT_UPGRADE_AA		0xAA
#define FT_UPGRADE_55		0x55

#define FT_FW_MIN_SIZE		8
#define FT_FW_MAX_SIZE		65536

/* Firmware file is not supporting minor and sub minor so use 0 */
#define FT_FW_FILE_MAJ_VER(x)	((x)->data[(x)->size - 2])
#define FT_6436FW_FILE_MAJ_VER(x)	((x)->data[266])
#define FT_FW_FILE_MIN_VER(x)	0
#define FT_FW_FILE_SUB_MIN_VER(x) 0

#define FT_FW_CHECK(x)		\
	(((x)->data[(x)->size - 8] ^ (x)->data[(x)->size - 6]) == 0xFF \
	&& (((x)->data[(x)->size - 7] ^ (x)->data[(x)->size - 5]) == 0xFF \
	&& (((x)->data[(x)->size - 3] ^ (x)->data[(x)->size - 4]) == 0xFF)))

#define FT_MAX_TRIES				5
#define FT_RETRY_DLY				20

#define FT_MAX_WR_BUF				10
#define FT_MAX_RD_BUF				2
#define FT_FW_PKT_LEN				128
#define FT_FW_PKT_META_LEN			6
#define FT_FW_PKT_DLY_MS			20
#define FT_FW_LAST_PKT				0x6ffa
#define FT_EARSE_DLY_MS				100
#define FT_55_AA_DLY_NS				5000

#define FT_UPGRADE_LOOP				10
#define FT_CAL_START				0x04
#define FT_CAL_FIN					0x00
#define FT_CAL_STORE				0x05
#define FT_CAL_RETRY				100
#define FT_REG_CAL					0x00
#define FT_CAL_MASK					0x70

#define FT_INFO_MAX_LEN				512

#define FT_FAMILY_MAX_TOUCHES 		(8)

#define FT_BLOADER_SIZE_OFF			12
#define FT_BLOADER_NEW_SIZE			30
#define FT_DATA_LEN_OFF_OLD_FW		8
#define FT_DATA_LEN_OFF_NEW_FW		14
#define FT_FINISHING_PKT_LEN_OLD_FW	6
#define FT_FINISHING_PKT_LEN_NEW_FW	12
#define FT_MAGIC_BLOADER_Z7			0x7bfa
#define FT_MAGIC_BLOADER_LZ4		0x6ffa
#define FT_MAGIC_BLOADER_GZF_30		0x7ff4
#define FT_MAGIC_BLOADER_GZF		0x7bf4
#define FT_RAWDATA_MAX_ROW 			(30)
#define FT_RAWDATA_MAX_COL  		(30)
#define FT_DEBUG_DIR_NAME			"ts_debug"
#define RESET_DELAY_ARRAY_LENGTH 	11

#define MAX_GESTURE					10
#define MAX_GESTURE_REG				6
enum {
	FW_UPGRADE_FAILED = 0,
	FW_UPGRADE_SUCCESS,
	FW_IS_UPDATETING,
};

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
enum {
	GESTURE_OFF = 0,
	GESTURE_ON,
};

enum {
	GESTURE_KEY_NEXTSONG		= 510,
	GESTURE_KEY_PRESONG			= 511,
	GESTURE_KEY_QUICKVIEW		= 512,
	GESTURE_KEY_VOICEASSISTANT 	= 513,
	GESTURE_KEY_WRITE			= 514,
	GESTURE_KEY_CAMERA			= 515,
	GESTURE_KEY_VIBRATE			= 516,
	GESTURE_KEY_PLAY			= 517,
	GESTURE_KEY_DIAL			= 518,
};
enum {
	GESTURE_LEFT_SLIDE			= 0x20,
	GESTURE_RIGHT_SLIDE			= 0x21,
	GESTURE_UP_SLIDE			= 0x22,
	GESTURE_DOWN_SLIDE			= 0x23,
	GESTURE_DOUBLECLICK			= 0x24,
	GESTURE_AT					= 0x50,
	GESTURE_LEFTARROW			= 0x51,
	GESTURE_RIGHTARROW			= 0x52,
	GESTURE_UPARROW				= 0x53,
	GESTURE_DOWNARROW			= 0x54,
	GESTURE_O					= 0x30,
	GESTURE_W					= 0x31,
	GESTURE_M					= 0x32,
	GESTURE_e					= 0x33,
	GESTURE_C					= 0x34,
	GESTURE_g					= 0x35,
	GESTURE_a					= 0x36,
	GESTURE_d					= 0x37,
	GESTURE_n					= 0x40,
	GESTURE_Z					= 0x41,
	GESTURE_b					= 0x42,
	GESTURE_q					= 0x43,
	GESTURE_L					= 0x44,
	GESTURE_p					= 0x45,
	GESTURE_S					= 0x46,
	GESTURE_u					= 0x47,
	GESTURE_h					= 0x70,
	GESTURE_k					= 0x71,
	GESTURE_y					= 0x72,
	GESTURE_r					= 0x73,
	GESTURE_THREE				= 0x60,
	GESTURE_SIX					= 0x61,
	GESTURE_NINE				= 0x62,
	GESTURE_SEVEN				= 0x63,
	GESTURE_EIGHT				= 0x64,
	GESTURE_TWO					= 0x65,
};
#define FTS_GESTRUE_POINTS_HEADER		8
#define FTS_GESTURE_POINTS		255
#define FTS_GESTURE_POINTS_NUM		128
#define	GESTURE_REG					0xd0	/*gesture function register*/
#define	GESTURE_WRITE_LOOP			10
#define	GESTURE_LOOP_DELAY			30

#endif

#define FT_TEST_CONFIG_NAME_MAX_LEN	128

enum {
	FT_BLOADER_VERSION_LZ4 	= 0,
	FT_BLOADER_VERSION_Z7 	= 1,
	FT_BLOADER_VERSION_GZF 	= 2,
};

enum {
	FT_FT5336_FAMILY_ID_0x11 = 0x11,
	FT_FT5336_FAMILY_ID_0x12 = 0x12,
	FT_FT5336_FAMILY_ID_0x13 = 0x13,
	FT_FT5336_FAMILY_ID_0x14 = 0x14,
	FT_FT5446_FAMILY_ID_0x54 = 0x54,
	FT_FT6436_FAMILY_ID_0x36 = 0x36,
	FT_FT5822_FAMILY_ID_0x58 = 0x58,
	FT_FT8606_FAMILY_ID_0x86 = 0x86,
};

#define FT_STORE_TS_INFO(buf, id, name, max_tch, group_id, fw_vkey_support, \
			fw_name, fw_maj, fw_min, fw_sub_min) \
			snprintf(buf, FT_INFO_MAX_LEN, \
				"controller\t= focaltech\n" \
				"model\t\t= 0x%x\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"drv_ver\t\t= 0x%x\n" \
				"group_id\t= 0x%x\n" \
				"fw_vkey_support\t= %s\n" \
				"fw_name\t\t= %s\n" \
				"fw_ver\t\t= %d.%d.%d\n", id, name, \
				max_tch, FT_DRIVER_VERSION, group_id, \
				fw_vkey_support, fw_name, fw_maj, fw_min, \
				fw_sub_min)


struct fw_upgrade_info {
	bool auto_cal;
	u16 delay_aa;
	u16 delay_55;
	u8 upgrade_id_1;
	u8 upgrade_id_2;
	u16 delay_readid;
	u16 delay_erase_flash;
};

struct ft5x06_ts_platform_data {
	struct fw_upgrade_info info;
	const char *name;
	const char *fw_name;
	const char *fw_name_pref;
	u32 irqflags;
	int irq_gpio;
	u32 irq_gpio_flags;
	int reset_gpio;
	u32 reset_gpio_flags;
	u32 family_id;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 group_id;
	u32 hard_rst_dly;
	u32 soft_rst_dly;
	u32 num_max_touches;
	u32 rawdata_range[2];
	u8 panelid_command[4];
	bool fw_vkey_support;
	bool no_force_update;
	bool i2c_pull_up;
	bool ignore_id_check;
	bool support_usb_check;
	u32 ignore_bottom_pixels;
	int (*power_init) (bool);
	int (*power_on) (bool);
	u32 gesture_func_map[10];
	u32 gesture_figure_map[10];
	u32 gesture_num;
	u32 gesture_reg_map[6];
	u32 gesture_reg_value_map[6];
	u32 gesture_reg_num;
	u32 touch_area_param;
};

struct ft5x06_touch_event {
	u8  count;
	u16 x[FT_FAMILY_MAX_TOUCHES];	/*x coordinate */
	u16 y[FT_FAMILY_MAX_TOUCHES];	/*y coordinate */
	u8 status[FT_FAMILY_MAX_TOUCHES]; /* touch event: 0 -- down; 1-- contact; 2 -- contact */
	u8 id[FT_FAMILY_MAX_TOUCHES];	/*touch ID */
	u8 weight[FT_FAMILY_MAX_TOUCHES];
	u8 area[FT_FAMILY_MAX_TOUCHES];
};

struct ft5x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ft5x06_ts_platform_data *pdata;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FT_FW_NAME_MAX_LEN];
	char fw_path[FT_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
#if defined(CONFIG_DEBUG_FS)
	struct dentry *dir;
#endif
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 last_touch_event_index;
	struct ft5x06_touch_event *event[2];
	u8 fw_ver[3];
	u8 pannel_id;
	struct ts_func_test_device ts_test_dev;
	struct work_struct resume_work;
	int 	ac_usb_plugin;

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	bool gesture_en;
	bool gesture_suspend_en;
	u16 gesture_track_x[FTS_GESTURE_POINTS_NUM];
	u16 gesture_track_y[FTS_GESTURE_POINTS_NUM];
	short gesture_track_pointnum;
	unsigned int gesture_state;

#endif
	char test_config_path[FT_TEST_CONFIG_NAME_MAX_LEN];

#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
	struct input_dev *ps_input_dev;
	struct wake_lock ps_wake_lock;
	struct sensors_classdev ps_cdev;
	bool ps_en;
	int  ps_lcd_state;
	bool tp_ps_en;
#endif
};

struct upgrade_config {
	const struct firmware *firmware;
	bool need_upgrade;
	bool enter_upgrade_mode;
};

static int debug_mask;
#if defined(CONFIG_TOUCHSCREEN_FT5X06_FH)
extern int usb_flag;
#endif
struct i2c_client *G_Client;
