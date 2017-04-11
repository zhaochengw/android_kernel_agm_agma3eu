/*
 *
 * FocalTech ft5x06 TouchScreen driver.
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
#define pr_fmt(fmt)	"%s: " fmt, __func__
#include"focaltech_ts.h"
#include "../ts_func_test.h"
#include <linux/productinfo.h>

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER)
#include "ft_gesture_lib.h"

#endif
#include "mcap_test_lib.h"

#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
#define NEAR_CODE	1
#define FAR_CODE	0

static struct sensors_classdev tp_proximity_cdev = {
	.name = "proximity",
	.vendor = "FocalTech",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static void psensor_report_dist(struct ft5x06_ts_data *data, int dist_code);
static int last_ps_val;
#endif

module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR);

#undef dev_info

#define dev_info(dev, format, arg...) do { if (debug_mask) {dev_printk(KERN_INFO , dev , format , ## arg); } } while (0)

static int ft5x06_get_fw_upgrade_config(struct ft5x06_ts_data *data,
										struct upgrade_config *config);
static int ft5x06_enter_update(struct ft5x06_ts_data *ts_data);
static int ft5x46_enter_update(struct ft5x06_ts_data *ts_data);
static int ft5x06_leave_update(struct ft5x06_ts_data *data);
static int ft5x46_leave_update(struct ft5x06_ts_data *data);
static void ft5x06_triger_update(struct device *dev, bool force);
static int ft5x06_ts_suspend(struct device *dev);
static int ft5x06_ts_resume(struct device *dev);
static int ft5x06_ts_resume_force(struct device *dev, bool force);
static int ft5x06_fetch_fw(struct ft5x06_ts_data *data, const struct firmware **ppfw);
static int ft5x06_fw_upgrade(struct device *dev, bool force);
static void ft5x06_ts_register_productinfo(struct ft5x06_ts_data *ts_data);
extern int qpnp_lbc_is_usb_chg_plugged(void);
static int hidi2c_to_stdi2c(struct i2c_client *client);

static unsigned char FT_Reset_delay_5336[RESET_DELAY_ARRAY_LENGTH] = {21, 18, 15, 30, 33, 36, 39, 42, 45, 27, 24};

static int ft5x06_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

static int ft5x06_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return ft5x06_i2c_write(client, buf, sizeof(buf));
}

static int ft5x0x_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return ft5x06_i2c_read(client, &addr, 1, val, 1);
}

int FTS_i2c_read(unsigned char *w_buf, int w_len, unsigned char *r_buf, int r_len)
{
	if (NULL == G_Client) {
		return -EIO;
	}
	return ft5x06_i2c_read(G_Client, w_buf, w_len, r_buf, r_len);
}

int FTS_i2c_write(unsigned char *w_buf, int w_len)
{
	if (NULL == G_Client) {
		return -EIO;
	}
	return ft5x06_i2c_write(G_Client, w_buf, w_len);
}

static void ft5x06_update_fw_ver(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0)
		dev_err(&client->dev, "fw major version read failed");

	reg_addr = FT_REG_FW_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[1], 1);
	if (err < 0)
		dev_err(&client->dev, "fw minor version read failed");

	reg_addr = FT_REG_FW_SUB_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[2], 1);
	if (err < 0)
		dev_err(&client->dev, "fw sub minor version read failed");

	dev_info(&client->dev, "Firmware version = %d.%d.%d\n",
		data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);
}

static bool ft5x06_check_rawdata_need(struct ft5x06_ts_data *data)
{
	if (FT_FT5336_FAMILY_ID_0x11 == data->family_id
		|| FT_FT5336_FAMILY_ID_0x12 == data->family_id
		|| FT_FT5336_FAMILY_ID_0x13 == data->family_id
		|| FT_FT5336_FAMILY_ID_0x14 == data->family_id) {
		return true;
	}
	return false;
}

static int  ft5x06_enter_factory(struct ft5x06_ts_data *data)
{
	int i = 0;
	u8 regval = 0xFF;

	while (i++ < 3) {

		ft5x0x_write_reg(data->client, 0x00, 0x40);	/*goto factory mode  */
		msleep(200);					/*make sure already enter factory mode*/
		if (ft5x0x_read_reg(data->client, 0x00, &regval) >= 0) {
			if ((regval & 0x70) == 0x40)
				break;
		}
	}

	return i < 3 ? 0 : -EIO;
}

static int  ft5x06_leave_factory(struct ft5x06_ts_data *data)
{
	int i = 0;
	u8 regval = 0xFF;

	while (i++ < 3) {

		ft5x0x_write_reg(data->client, 0x00, 0x00);	/*leve factory mode  */
		msleep(200);									/*make sure already enter factory mode*/
		if (ft5x0x_read_reg(data->client, 0x00, &regval) >= 0) {
			if ((regval & 0x70) == 0x00)
				break;
		}
	}

	return i < 3 ? 0 : -EIO;
}

static int ft5x06_get_rawdata_rowcol(struct ft5x06_ts_data *data, u8 *tx, u8 *rx)
{
	int retval  = 0;
	u8 tx_num = 0, rx_num = 0;

	if (FT_FT5446_FAMILY_ID_0x54 == data->family_id) {
		retval = ft5x0x_read_reg(data->client, 0x02, &tx_num);
		if (retval < 0) {
			return retval;
		}

		retval = ft5x0x_read_reg(data->client, 0x03, &rx_num);
		if (retval < 0) {
			return retval;
		}
	} else {
		retval = ft5x0x_read_reg(data->client, 0x03, &tx_num);
		if (retval < 0) {
			return retval;
		}

		retval = ft5x0x_read_reg(data->client, 0x04, &rx_num);
		if (retval < 0) {
			return retval;
		}
	}

	rx_num > tx_num ? rx_num-- : tx_num--;

	if (rx_num < FT_RAWDATA_MAX_ROW && tx_num < FT_RAWDATA_MAX_COL) {
		*tx = tx_num;
		*rx = rx_num;
		return 0;
	} else {
		return -ERANGE;
	}
}

static int ft5x06_get_rawdata_info(struct ft5x06_ts_data *data, char *buf)
{
	int ret = 0;
	u8 tx = 0, rx = 0;

	ret = ft5x06_enter_factory(data);
	if (ret < 0) {
		return 0;
	}

	ret = ft5x06_get_rawdata_rowcol(data, &tx, &rx);
	if (ret == 0) {
		 ret = snprintf(buf, PAGE_SIZE, "RX:%u TX:%u HIGH:%u LOW:%u\n",
					   rx, tx, data->pdata->rawdata_range[0], data->pdata->rawdata_range[1]);
	}
	ft5x06_leave_factory(data);
	return ret;
}

static int ft5x06_get_rawdata(struct ft5x06_ts_data *data,  char *buf)
{
	int ret = 0, size = 0;
	u8 rx_num = 0, tx_num = 0, devmode = 0, i, j;
	u8 *read_buf = NULL, read_size = 0;
	u8 write_buf[2];
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;

	ret = ft5x06_enter_factory(data);
	if (ret < 0) {
		return 0;
	}
	ret = ft5x06_get_rawdata_rowcol(data, &tx_num, &rx_num);
	if (ret < 0) {
		goto LEAVE_RETURN;
	}

	ft5x0x_read_reg(client, 0x00, &devmode);
	devmode |= 0x80;
	ft5x0x_write_reg(client , 0x00, devmode);
	msleep(150);
	ft5x0x_read_reg(client , 0x00, &devmode);

	if ((devmode & 0x80) != 0) {
			dev_err(dev, "%s: ERROR: could not scan", __func__);
			goto LEAVE_RETURN;
	}

	read_size = rx_num * 2;
	read_buf = devm_kzalloc(dev, read_size, GFP_KERNEL);
	if (read_buf  == NULL) {
		goto LEAVE_RETURN;
	}

	for (i = 0; i < tx_num; i++) {
		if (ft5x0x_write_reg(client, 0x01, i) < 0)
			goto LEAVE_RETURN;

		mdelay(1);
		write_buf[0] = 0x10;
		write_buf[1] = read_size;
		if (ft5x06_i2c_read(client, write_buf, 2, read_buf, read_size) < 0)
			goto LEAVE_RETURN;

		for (j = 0; j < rx_num; j++) {
			size += snprintf(buf + size, PAGE_SIZE, j < rx_num-1 ? "%u " : "%u\n", (read_buf[2 * j] << 8) + read_buf[2 * j + 1]);
		}
	}

LEAVE_RETURN:
	if (read_buf != NULL)
		devm_kfree(dev, read_buf);
	ft5x06_leave_factory(data);
	return size;
}

static int factory_check_fw_update_need(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	struct upgrade_config config = {
		.enter_upgrade_mode = false,
		.need_upgrade = false,
	};

	ft5x06_get_fw_upgrade_config(data, &config);

	release_firmware(config.firmware);

	if (config.enter_upgrade_mode == true) {
		if (FT_FT5446_FAMILY_ID_0x54 == data->family_id)
			ft5x46_leave_update(data);
		else
			ft5x06_leave_update(data);
	}
	return config.need_upgrade;
}

static	int factory_get_fw_update_progress(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return data->loading_fw ? FW_IS_UPDATETING : FW_UPGRADE_SUCCESS;
}

static	int factory_proc_fw_update(struct device *dev, bool force)
{
	ft5x06_triger_update(dev, force);

	return 0;
}

static int factory_get_rawdata(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (ft5x06_check_rawdata_need(data)) {
		return ft5x06_get_rawdata(data, buf);
	} else {
		return snprintf(buf, PAGE_SIZE, "%s\n", "NOT SUPPORTED");
	}
}

static int factory_get_rawdata_info(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (ft5x06_check_rawdata_need(data)) {
		return ft5x06_get_rawdata_info(data, buf);
	} else {
		return snprintf(buf, PAGE_SIZE, "%s\n", "NOT SUPPORTED");
	}
}

static int factory_proc_hibernate_test(struct device *dev)
{
	int err = 0;
	u8 reg_value = 0;
	u8 reg_addr = FT_REG_ID;
	int i;
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	gpio_direction_output(data->pdata->reset_gpio, 1);
	mdelay(1);
	gpio_direction_output(data->pdata->reset_gpio, 0);
	mdelay(5);

	err = ft5x06_i2c_read(data->client, &reg_addr, 1, &reg_value, 1);
	if (err >= 0) {
		dev_err(dev, "%s: read i2c ok,with result=%d\n", __func__, err);
		goto out;
	}

	gpio_direction_output(data->pdata->reset_gpio, 1);
	msleep(200);

	err = ft5x06_i2c_read(data->client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(dev, "reset test Fail!\n");
		goto out;
	}
	dev_info(&data->client->dev, "reset test success!\n");

	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	return 1;
out:
	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	return 0;
}

static int factory_get_ic_fw_version(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", data->fw_ver[0]);
}

static int factory_get_fs_fw_version(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int rc = 0;

	u8 fs_fw_version = 0;

	rc = ft5x06_fetch_fw(data, &fw);
	if (rc == 0 && fw != NULL) {
		if (FT_FT6436_FAMILY_ID_0x36 == data->family_id)
			fs_fw_version = FT_6436FW_FILE_MAJ_VER(fw);
		else
			fs_fw_version = FT_FW_FILE_MAJ_VER(fw);
		release_firmware(fw);
	}
	return snprintf(buf, PAGE_SIZE, "0x%02X\n", fs_fw_version);
}

static int factory_get_module_id(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", data->pannel_id);
}

static int factory_get_calibration_ret(struct device *dev)
{
	return 1;
}

static int factory_set_fw_path(struct device *dev, const char *buf)
{
	size_t len;
	char *pfile;
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	strlcpy(data->fw_path, buf, sizeof(data->fw_path));
	len = strlen(data->fw_path);
	if (len > 0) {
		if (data->fw_path[len-1] == '\n')
			data->fw_path[len-1] = '\0';

		pfile = strrchr(data->fw_path, '/');
		if (pfile) {
			strlcpy (data->fw_name, pfile+1, sizeof(data->fw_name));
			pfile[1] = '\0';
		}
	}
	return 1;
}

static int factory_get_fw_path(struct device *dev, char *buf, size_t buf_size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	strlcpy(buf, data->fw_path, buf_size);
	strlcat(buf, data->fw_name, buf_size);
	return 1;
}

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
static unsigned int asic_to_hex(unsigned char val)
{
	if ((val >= '0') && (val <= '9')) {
		val -= '0';
		}
	else if ((val >= 'a') && (val <= 'z')) {
		val = val - 'a' + 10;
		}
	else if ((val >= 'A') && (val <= 'Z')) {
		val = val - 'A' + 10;
		}
	return (unsigned int)val;
}

static int set_gesture_switch(struct device *dev, const char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned char gesture[10], len;

	strlcpy(gesture, buf, sizeof(gesture));
	len = strlen(gesture);
	if (len > 0) {
		if ((gesture[len-1] == '\n') || (gesture[len-1] == '\0')) {
			len--;
		}
	}

	dev_info(&data->client->dev, "%s len: %d gtp_state: %d,%d,%d.\n",
					__func__, len, gesture[0], gesture[1], gesture[2]);
	if (len == 1) {
		if (gesture[0] == '1')
			data->gesture_state = 0xffff;	/*cmd =1, it will open all gesture.*/
		else if (gesture[0] == '0')
			data->gesture_state = 0x0;		/*cmd =0, it will close all gesture.*/
	} else if (len == 4) {
		data->gesture_state = asic_to_hex(gesture[0]) * 0x1000
						+ asic_to_hex(gesture[1]) * 0x100
						+ asic_to_hex(gesture[2]) * 0x10
						+ asic_to_hex(gesture[3]);
	} else {
		dev_info(&data->client->dev, "[set_gesture_switch]write wrong cmd.");
		return 0;
	}
	if (!data->gesture_state)
		data->gesture_en = false;
	else
		data->gesture_en = true;

	printk("%s is %x.\n", __func__, data->gesture_state);

	return 0;
}

static bool get_gesture_switch(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return data->gesture_en;
}

static int get_gesture_pos(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	char temp[20] = {0};
	char gesture_pos[512] = {0};
	int i = 0;

	for (i = 0; i < data->gesture_track_pointnum; i++) {
		snprintf(temp, ARRAY_SIZE(temp), "%u,%u;", (unsigned int)data->gesture_track_x[i], (unsigned int)data->gesture_track_y[i]);
		strlcat(gesture_pos, temp, 512);
	}
	return snprintf(buf, PAGE_SIZE, "%s\n", gesture_pos);
}

#endif

static bool get_tp_enable_switch(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return !data->suspended;
}

static int set_tp_enable_switch(struct device *dev, bool enable)
{
	static bool is_the_first_set = 1;
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	static bool gesture_switch;
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
#endif

	if (is_the_first_set) {
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
		gesture_switch = data->gesture_en;
#endif
		is_the_first_set = 0;
	}

	if (enable) {
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
		data->gesture_en = gesture_switch;
#endif
		return ft5x06_ts_resume(dev);
	} else {
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
		gesture_switch = data->gesture_en;
		data->gesture_en = 0;
#endif
		return ft5x06_ts_suspend(dev);
	}
}

static int ft5x06_get_ini_size(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128] = {0};

	snprintf(filepath, ARRAY_SIZE(filepath), "%s", config_name);
	pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("Error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);

	return fsize;
}

static int ft5x06_read_ini_data(char *config_name, char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128] = {0};
	loff_t pos;
	mm_segment_t old_fs;

	snprintf(filepath, ARRAY_SIZE(filepath), "%s", config_name);
	pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("Error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}
static int ft5x06_get_testparam_from_ini(char *config_name)
{
	char *filedata = NULL;
	int ini_size = ft5x06_get_ini_size(config_name);

	pr_info("Ini_size = %d\n", ini_size);
	if (ini_size <= 0) {
		pr_err("Get firmware size failed.\n");
		return -EIO;
	}
	filedata = vmalloc(ini_size + 1);
	if (filedata == NULL)
		return -ENOMEM;
	if (ft5x06_read_ini_data(config_name, filedata)) {
		pr_err("Request ini file failed.");
		kfree(filedata);
		return -EIO;
	}
	SetParamData(filedata);

	return 0;
}
static int factory_get_short_test(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int cap_test_flag;

	mutex_lock(&dev->mutex);
	Init_I2C_Read_Func(FTS_i2c_read);
	Init_I2C_Write_Func(FTS_i2c_write);

	if (ft5x06_get_testparam_from_ini(data->test_config_path) < 0) {
		dev_err(&data->client->dev, "Get testparam from ini failture.\n");
		mutex_unlock(&dev->mutex);
		return 0;
	} else {
		if (true == StartTestTP()) {
			dev_info(&data->client->dev, "Cap test pass.\n");
			cap_test_flag = 1;
		} else {
			dev_info(&data->client->dev, "Cap test failed.\n");
			cap_test_flag = 0;
		}
		/* FreeTestParamData(); */
		mutex_unlock(&dev->mutex);
		return cap_test_flag;
	}
}

static bool factory_get_test_config_need(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	if (FT_FT5446_FAMILY_ID_0x54 == data->family_id) {
		return true;
	}
	return false;
}

static int factory_set_test_config_path(struct device *dev, const char *buf)
{
	size_t len;
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	strlcpy(data->test_config_path, buf, sizeof(data->test_config_path));
	len = strlen(data->test_config_path);
	if (len > 0) {
		if (data->test_config_path[len-1] == '\n')
			data->test_config_path[len-1] = '\0';
	}
	return len;
}

static int factory_ts_func_test_register(struct ft5x06_ts_data *data)
{
	data->ts_test_dev.dev = &data->client->dev;
	data->ts_test_dev.check_fw_update_need = factory_check_fw_update_need;
	data->ts_test_dev.get_calibration_ret = factory_get_calibration_ret;
	data->ts_test_dev.get_fs_fw_version = factory_get_fs_fw_version;
	data->ts_test_dev.get_fw_update_progress = factory_get_fw_update_progress;
	data->ts_test_dev.get_ic_fw_version = factory_get_ic_fw_version;
	data->ts_test_dev.get_module_id = factory_get_module_id;
	data->ts_test_dev.get_rawdata = factory_get_rawdata;
	data->ts_test_dev.get_rawdata_info = factory_get_rawdata_info;
	data->ts_test_dev.proc_fw_update = factory_proc_fw_update;
	data->ts_test_dev.proc_hibernate_test = factory_proc_hibernate_test;
	data->ts_test_dev.set_fw_path = factory_set_fw_path;
	data->ts_test_dev.get_fw_path = factory_get_fw_path;

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	data->ts_test_dev.get_gesture_switch = get_gesture_switch;
	data->ts_test_dev.set_gesture_switch = set_gesture_switch;
	data->ts_test_dev.get_gesture_pos = get_gesture_pos;
#endif
	data->ts_test_dev.get_tp_enable_switch = get_tp_enable_switch;
	data->ts_test_dev.set_tp_enable_switch = set_tp_enable_switch;

	if (FT_FT5446_FAMILY_ID_0x54 == data->family_id) {
		data->ts_test_dev.get_short_test = factory_get_short_test;
		data->ts_test_dev.need_test_config = factory_get_test_config_need;
		data->ts_test_dev.set_test_config_path = factory_set_test_config_path;
	}

	register_ts_func_test_device(&data->ts_test_dev);
	return 0;
}

static struct device *s_ctp_dev;

static int ctp_register_device(struct device *dev)
{
	s_ctp_dev = dev;
	return 0;
}

static int ctp_unregister_device(struct device *dev)
{
	s_ctp_dev = NULL;
	return 0;
}

#if defined(CONFIG_TOUCHSCREEN_FT5X06_FH)
static int ft5x06_work_with_ac_usb_plugin(struct ft5x06_ts_data *data, int plugin)
{
	u8 val = 0;
	if (plugin != 0) {
		val = 1;
	}

	dev_info(&data->client->dev, "%s: %d\n", __func__, val);

	return ft5x0x_write_reg(data->client, 0x8B, val);
}

int ctp_work_with_ac_usb_plugin(int plugin)
{
	struct ft5x06_ts_data *data;

	if (s_ctp_dev == NULL)
		return -EIO;

	data = dev_get_drvdata(s_ctp_dev);

	if (!data->suspended) {
	    if (data->pdata->support_usb_check) {
			if (0 == plugin)
				dev_info(&data->client->dev, "USB is plugged Out(%d,%s)\n", __LINE__, __FUNCTION__);
			else
				dev_info(&data->client->dev, "USB is plugged In(%d,%s)\n", __LINE__, __FUNCTION__);

			ft5x06_work_with_ac_usb_plugin(data, plugin);
		} else {
			dev_info(&data->client->dev, "ctp no need check usb(%d,%s)\n", __LINE__, __FUNCTION__);
		}
	} else {
		data->ac_usb_plugin = plugin;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(ctp_work_with_ac_usb_plugin);
#endif

static int ft5x06_enter_update(struct ft5x06_ts_data *ts_data)
{
	struct i2c_client *client = ts_data->client;
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 reset_reg;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};

	int i, j;

	for (i = 0; i < RESET_DELAY_ARRAY_LENGTH; i++) {
		dev_info(&ts_data->client->dev, "Step 1:Reset the CTP.\n");
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(FT_Reset_delay_5336[i]);
		dev_info(&ts_data->client->dev, " FT_Reset_delay_5336[%d]=%d\n", i, FT_Reset_delay_5336[i]);

		/* Enter upgrade mode */
		dev_info(&ts_data->client->dev, "Step 2:Enter upgrade mode.\n");
		w_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, w_buf, 1);
		usleep(FT_55_AA_DLY_NS);
		w_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, w_buf, 1);

		/* check READ_ID */
		dev_info(&ts_data->client->dev, " Step 3: Check IC ID.\n");
		mdelay(info.delay_readid);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;
		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
		if ((r_buf[0] != info.upgrade_id_1) || (r_buf[1] != info.upgrade_id_2)) {
			dev_err(&client->dev, " Upgrade ID(%u,%u) mismatch (%u,%u)(%d)\n",
					r_buf[0], r_buf[1], info.upgrade_id_1, info.upgrade_id_2, i);
		} else {
			dev_info(&ts_data->client->dev, " IC_ID_1=0x%x,IC_ID_2=0x%x.\n", r_buf[0], r_buf[1]);
			return 0;
		}
	}

	dev_info(&ts_data->client->dev, "Hard Reset Style Check IC ID ERROR!\n");

	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
			for (j = 0; j < FT_UPGRADE_LOOP; j++) {
			if (FT6X06_ID == ts_data->family_id || FT_FT6436_FAMILY_ID_0x36 == ts_data->family_id) {
				reset_reg = FT_RST_CMD_REG2;
			} else {
				reset_reg = FT_RST_CMD_REG1;
			}

			/* Reset the Ctp */
			dev_info(&ts_data->client->dev, " Step 1:Reset the ctp\n");
			dev_info(&ts_data->client->dev, " i=%d,info.delay_aa=%d\n", i, (info.delay_aa + 5 * i));
			ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
			mdelay(info.delay_aa + 5 * i);
			dev_info(&ts_data->client->dev, " j=%d,info.delay_55=%d\n", j, (info.delay_55 + 5 * j));
			ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
			mdelay(info.delay_55 + 5 * j);

			/* Enter upgrade mode */
			dev_info(&ts_data->client->dev, " Step 2:Enter upgrade mode.\n");
			w_buf[0] = FT_UPGRADE_55;
			ft5x06_i2c_write(client, w_buf, 1);
			usleep(FT_55_AA_DLY_NS);
			w_buf[0] = FT_UPGRADE_AA;
			ft5x06_i2c_write(client, w_buf, 1);

			/* check READ_ID */
			dev_info(&ts_data->client->dev, " Step 3: Check IC ID.\n");
			mdelay(info.delay_readid);
			w_buf[0] = FT_READ_ID_REG;
			w_buf[1] = 0x00;
			w_buf[2] = 0x00;
			w_buf[3] = 0x00;
			ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
			if ((r_buf[0] != info.upgrade_id_1) || (r_buf[1] != info.upgrade_id_2)) {
				dev_err(&client->dev, " Upgrade ID(%u,%u) mismatch (%u,%u)(%d)\n",
						r_buf[0], r_buf[1], info.upgrade_id_1, info.upgrade_id_2, i);
			} else {
				dev_info(&ts_data->client->dev, "IC_ID_1=0x%x,IC_ID_2=0x%x.\n", r_buf[0], r_buf[1]);
				return 0;
			}
		}
	}

	dev_info(&ts_data->client->dev, "Soft Reset Style Check IC ID ERROR!\n");
	dev_err(&client->dev, "Abort upgrade\n");
	return -EIO;
}

static int ft5x06_leave_update(struct ft5x06_ts_data *data)
{
	/* reset */
	u8 w_buf[FT_MAX_WR_BUF] = {0};

	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(data->client, w_buf, 1);
	msleep(data->pdata->soft_rst_dly);
	dev_info(&data->client->dev, "Ft5x06 leave update!\n");
	return 0;
}

static int ft5x06_get_pannel_id_enter_upgrademode_if_needed(
									struct ft5x06_ts_data *ts_data, bool *enter_upgrade)
{
	u8 pannel_id = 0;
	int rc = 0;
	struct device *dev = &(ts_data->client->dev);

	*enter_upgrade =  false;
	dev_info(dev, "In normal mode, try get pannel id....\n");
	rc = ft5x0x_read_reg(ts_data->client, FT_RGE_PANNEL_ID, &pannel_id);

	if (rc < 0 || pannel_id == 0) {
		dev_err(dev, "Can't get pannel id in normal mode; Try to enter upgrade mode....\n");
		if (FT_FT5446_FAMILY_ID_0x54 == ts_data->family_id) {
			if (ft5x46_enter_update(ts_data) == 0) {
				*enter_upgrade = true;
				ft5x06_i2c_write(ts_data->client, ts_data->pdata->panelid_command,
								sizeof(ts_data->pdata->panelid_command));
				msleep(5);
				rc = ft5x06_i2c_read(ts_data->client, ts_data->pdata->panelid_command,
									sizeof(ts_data->pdata->panelid_command), &pannel_id, 1);
				if (rc < 0) {
					dev_err(&ts_data->client->dev, "Ft5x46 in upgrade mode, Get pannel id failed\n");
					return rc;
				}
			} else
				return -EIO;
		} else {
			if (ft5x06_enter_update(ts_data) == 0) {
				*enter_upgrade = true;
				dev_info(dev, "Ft5x06 in upgrade mode, try get pannel id....\n");
				rc = ft5x06_i2c_read(ts_data->client, ts_data->pdata->panelid_command,
									 sizeof(ts_data->pdata->panelid_command), &pannel_id, 1);
				if (rc < 0) {
					dev_err(dev, "Ft5x06 in upgrade mode, Get pannel id failed\n");
					return rc;
				}
			} else
				return -EIO;
		}
	}

	ts_data->pannel_id = pannel_id;
	dev_info(dev, "Get pannel id = 0x%X\n", pannel_id);
	return 0;
}

static int ft5x06_get_fw_filename(struct ft5x06_ts_data *data, u8 pannel_id, char *buf, int size)
{
	char tmp[16] = {0};

	if (pannel_id != 0) {
		strlcpy(buf, "fw_", size);
		strlcat(buf, CONFIG_HIS_PRODUCT_NAME, size);
		strlcat(buf, "_", size);
		strlcat(buf, data->pdata->name, size);
		snprintf(tmp, ARRAY_SIZE(tmp), "_0x%02x", pannel_id);
		strlcat(buf, tmp, size);
		strlcat(buf, ".bin", size);
	} else {
		strlcpy(buf, data->pdata->fw_name_pref, size);
	}
	return 0;
}

static int ft5x06_fetch_fw(struct ft5x06_ts_data *data, const struct firmware **ppfw)
{
	int rc;
	size_t len;
	const struct firmware *fw = NULL;
	struct device *dev = &data->client->dev;
	char fw_file_name[FT_FW_NAME_MAX_LEN] = {0};
	char fw_path[FT_FW_NAME_MAX_LEN] = {0};

	if (ppfw == NULL) {
		return -EINVAL;
	}

	if (data->fw_name[0]) {
		strlcpy(fw_file_name, data->fw_name, sizeof(fw_file_name));
		dev_info(dev, "%s: with ori fw_name = %s\n",
				__FUNCTION__, data->fw_name);
	} else {
		ft5x06_get_fw_filename(data, data->pannel_id, fw_file_name, sizeof(fw_file_name));
		dev_info(dev, "%s: with ori fw_name=null, fw_file_name=%s\n",
				__FUNCTION__, fw_file_name);
	}

	if (data->fw_path[0]) {
		strlcpy(fw_path, data->fw_path, sizeof(fw_path));
		len = strlen(fw_path);
		if (fw_path[len-1] != '/') {
			strlcat(fw_path, "/", sizeof(fw_path));
		}
	}

	strlcat(fw_path, fw_file_name, sizeof(fw_path));
	dev_info(dev, "%s: fw_path=%s\n", __FUNCTION__, fw_path);
	rc = request_firmware(&fw, fw_path, dev);

	if (rc < 0) {
		dev_err(dev, "Request firmware failed (%d)\n", rc);
		return rc;
	}
	*ppfw = fw;

	return 0;
}

static int ft5x06_get_fw_upgrade_config(struct ft5x06_ts_data *data,
										struct upgrade_config *config)
{
	int rc;
	const struct firmware *fw;
	struct device *dev = &data->client->dev;
	u8 fw_file_maj, fw_file_min, fw_file_sub_min;
	bool need_update  = false;

	rc = ft5x06_get_pannel_id_enter_upgrademode_if_needed (data, &config->enter_upgrade_mode);

	if (rc < 0) {
		return -EIO;
	}

	rc = ft5x06_fetch_fw(data, &config->firmware);
	fw = config->firmware;
	if (rc != 0 || fw == NULL) {
		return -EIO;
	}

	if (fw->size < FT_FW_MIN_SIZE || fw->size > FT_FW_MAX_SIZE) {
		dev_err(dev, "Invalid firmware size (%zd)\n", fw->size);
		return -EIO;
	}
	if (FT_FT6436_FAMILY_ID_0x36 == data->family_id)
		fw_file_maj = FT_6436FW_FILE_MAJ_VER(fw);
	else
		fw_file_maj = FT_FW_FILE_MAJ_VER(fw);
	fw_file_min = FT_FW_FILE_MIN_VER(fw);
	fw_file_sub_min = FT_FW_FILE_SUB_MIN_VER(fw);

	dev_info(dev, "Current firmware: %d.%d.%d", data->fw_ver[0],
				data->fw_ver[1], data->fw_ver[2]);
	dev_info(dev, "New firmware: %d.%d.%d", fw_file_maj,
				fw_file_min, fw_file_sub_min);

	need_update = 	(fw_file_maj > data->fw_ver[0]) ? true :
									(fw_file_maj < data->fw_ver[0]) ? false :
										(fw_file_min > data->fw_ver[1]) ? true :
											(fw_file_min < data->fw_ver[1]) ? false :
												(fw_file_sub_min > data->fw_ver[2]) ? true : false;

	config->need_upgrade = need_update;
	return 0;
}

static void ft5x06_triger_update(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, force);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);
}

static struct ft5x06_touch_event *ft5x06_allocate_touch_event_seq(
														struct device *dev, u32 max_touches)
{
	struct ft5x06_touch_event *event = NULL;
	event = devm_kzalloc(dev, sizeof(struct ft5x06_touch_event), GFP_KERNEL);
	return event;
}

static struct ft5x06_touch_event *ft5x06_new_touchdata_index(struct ft5x06_ts_data *data)
{
	struct ft5x06_touch_event *event;

	u8 index = (++data->last_touch_event_index) % 2;
	event = data->event[index];
	memset(event, 0, sizeof(struct ft5x06_touch_event));

	return event;
}

static struct ft5x06_touch_event *ft5x06_pri_touchdata_event(struct ft5x06_ts_data *data)
{
	u8 index = (data->last_touch_event_index + 1) % 2;
	return  data->event[index];
}

static struct ft5x06_touch_event *ft5x06_cur_touchdata_event(struct ft5x06_ts_data *data)
{
	u8 index = (data->last_touch_event_index) % 2;
	return data->event[index];
}

static int ft5x06_fetch_touchdata(struct ft5x06_ts_data *data)
{
	struct ft5x06_touch_event *event;
	int rc, i;
	u8 id;
	u8 reg = 0x00;
	u8 *buf = data->tch_data;

	rc = ft5x06_i2c_read(data->client, &reg, 1, buf, data->tch_data_len);
	if (rc < 0) {
		dev_err(&data->client->dev, "%s: read data fail\n", __func__);
		return rc;
	}

	event = ft5x06_new_touchdata_index(data);

	for (i = 0; i < data->pdata->num_max_touches; i++) {

	   id = (buf[FT_TOUCH_ID_POS + FT_ONE_TCH_LEN * i]) >> 4;
	   if (id >= FT_MAX_ID)
		   break;

	   event->x[i] = (buf[FT_TOUCH_X_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
		   (buf[FT_TOUCH_X_L_POS + FT_ONE_TCH_LEN * i]);
	   event->y[i] = (buf[FT_TOUCH_Y_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
		   (buf[FT_TOUCH_Y_L_POS + FT_ONE_TCH_LEN * i]);
	   event->id[i] = id;
	   event->status[i] = buf[FT_TOUCH_EVENT_POS + FT_ONE_TCH_LEN * i] >> 6;
	   event->weight[i] = buf[FT_TOUCH_WEIGHT_POS + FT_ONE_TCH_LEN * i];
	   event->area[i] = (data->pdata->touch_area_param) * (buf[FT_TOUCH_AREA_POS + FT_ONE_TCH_LEN * i] >> 4);
	}
	event->count = i;

#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
	if (data->ps_en) {
		buf[1] = buf[1] >> 5;
		if (last_ps_val != buf[1]) {
			if (buf[1] == 0x07) {	/*far*/
				psensor_report_dist(data, FAR_CODE);
				printk("PS_NEAR_TO_FAR INT occurred.\n");
			}
			if (buf[1] == 0x06) {	/*near*/
				psensor_report_dist(data, NEAR_CODE);
				printk("PS_FAR_TO_NEAR INT occurred.\n");
			}
			last_ps_val = buf[1];
		}
	}
#endif

	return 0;
}

static void ft5x06_adjust_touchdata(struct ft5x06_ts_data *data)
{
	int i, j;
	u8 pre_id;
	struct ft5x06_touch_event *cur_event = ft5x06_cur_touchdata_event(data);
	struct ft5x06_touch_event *pre_event = ft5x06_pri_touchdata_event(data);

	for (i = 0; i < pre_event->count; i++) {
		pre_id = pre_event->id[i];
		if (pre_event->status[i] == 0 || pre_event->status[i] == 2) {
			for (j = 0; j < cur_event->count; j++) {
				if (cur_event->id[j] == pre_id)
					break;
			}
			if ((j >= cur_event->count)
				&& (cur_event->count < data->pdata->num_max_touches)) {
				cur_event->id[cur_event->count] = pre_id;
				cur_event->status[cur_event->count] = 1;
				cur_event->count++;
			}
		}
	}
}

static void ft5x06_report_touchdata(struct ft5x06_ts_data *data)
{
	int i;
	struct ft5x06_touch_event *event = ft5x06_cur_touchdata_event(data);
	struct input_dev *input_dev =  data->input_dev;

	for (i = 0; i < event->count; i++) {
		input_mt_slot(input_dev, event->id[i]);
		if (event->status[i] == FT_TOUCH_DOWN || event->status[i] == FT_TOUCH_CONTACT) {
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, event->x[i]);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, event->y[i]);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);
			/*input_report_abs(input_dev, ABS_MT_PRESSURE, event->weight[i]);*/
			if (event->y[i] > data->pdata->y_max) {
				dev_info(&data->client->dev,
					"%s: [x, y]=[%d, %d],id = %d\n", __func__, event->x[i], event->y[i], event->id[i]);
			}
			dev_info(&data->client->dev, "%s: [x, y]=[%d, %d]\n", __func__, event->x[i], event->y[i]);
		} else {
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		}
	}

	if (event->count) {
		input_mt_report_pointer_emulation(input_dev, false);
		input_sync(input_dev);
	}
}

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER)
static int ft5x06_init_gesture(struct ft5x06_ts_data *data)
{
	return 0;
}

#else
static int ft5x06_init_gesture(struct ft5x06_ts_data *data)
{
	int i, j;
	u8 reg_value;

	for (i = 0; i < data->pdata->gesture_reg_num; i++) {
		for (j = 0; j < GESTURE_WRITE_LOOP; j++) {
			ft5x0x_write_reg(data->client, data->pdata->gesture_reg_map[i], data->pdata->gesture_reg_value_map[i]);
			ft5x0x_read_reg(data->client, data->pdata->gesture_reg_map[i], &reg_value);
			if (data->pdata->gesture_reg_value_map[i] == reg_value) {
				dev_info(&data->client->dev, "Write reg 0x%2x success.loop = %d.(%s,%d)\n", data->pdata->gesture_reg_map[i], j, __FUNCTION__, __LINE__);
				break;
			}
		}
		if (GESTURE_WRITE_LOOP == j) {
			dev_err(&data->client->dev, "Write reg 0x%2x failed.loop = %d.(%s,%d)\n", data->pdata->gesture_reg_map[i], j, __FUNCTION__, __LINE__);
		}
	}
	return 0;
}
#endif
static bool ft5x06_report_gesture(struct ft5x06_ts_data *data)
{
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FTS_GESTURE_POINTS * 3] = {0};
	int ret = 0;
	int gesture_id = 0;
	int index = 0;

	w_buf[0] = 0xd3;

	ret = ft5x06_i2c_read(data->client, w_buf, 1, r_buf, FTS_GESTRUE_POINTS_HEADER);
	if (ret < 0) {
		 dev_err(&data->client->dev, "ft5x06_i2c_read() read data error!\n");
		 return false;
	}
	gesture_id = r_buf[0];
	data->gesture_track_pointnum = r_buf[1] & 0xff;

	if (data->gesture_track_pointnum > FTS_GESTURE_POINTS_NUM) {
		pr_err_buf("pointnum(%d) is out of range!\n", data->gesture_track_pointnum);
		return false;
	}

	if (FT_FT5446_FAMILY_ID_0x54 == data->family_id
	|| FT_FT5822_FAMILY_ID_0x58 == data->family_id
	|| FT_FT8606_FAMILY_ID_0x86 == data->family_id) {
		if ((data->gesture_track_pointnum * 4 + 2) < 255) {
			ret = ft5x06_i2c_read(data->client, w_buf, 1, r_buf, (data->gesture_track_pointnum * 4 + 2));
		} else {
			ret = ft5x06_i2c_read(data->client, w_buf, 1, r_buf, 255);
			ret = ft5x06_i2c_read(data->client, w_buf, 0, r_buf + 255, (data->gesture_track_pointnum * 4 + 2) - 255);
		}
		if (ret < 0) {
			dev_err(&data->client->dev, "read touchdata failed.\n");
			return ret;
		}

		for (index = 0; index < data->gesture_track_pointnum; index++) {
			data->gesture_track_x[index] = (r_buf[2 + 4 * index] & 0x0F) << 8 | (r_buf[3 + 4 * index] & 0xFF);
			data->gesture_track_y[index] = (r_buf[4 + 4 * index] & 0x0F) << 8 | (r_buf[5 + 4 * index] & 0xFF);
		}
		goto gesture_report;
	} else {
		if (GESTURE_DOUBLECLICK == r_buf[0]) {
			gesture_id = GESTURE_DOUBLECLICK;
			if (data->gesture_state & 0x0002) {
				input_report_key(data->input_dev, data->pdata->gesture_func_map[0], 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, data->pdata->gesture_func_map[0], 0);
				input_sync(data->input_dev);
			}
			return true;
		}
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER)
		if ((data->gesture_track_pointnum * 4 + 8) < 255) {
			ret = ft5x06_i2c_read(data->client, w_buf, 1, r_buf, (data->gesture_track_pointnum * 4 + 8));
		} else {
			ret = ft5x06_i2c_read(data->client, w_buf, 1, r_buf, 255);
			ret = ft5x06_i2c_read(data->client, w_buf, 0, r_buf + 255, (data->gesture_track_pointnum * 4 + 8) - 255);
		}
		if (ret < 0) {
			dev_err(&data->client->dev, "read touchdata failed.\n");
			return ret;
		}

		gesture_id = fetch_object_sample(r_buf, data->gesture_track_pointnum);

		for (index = 0; index < data->gesture_track_pointnum; index++) {
			data->gesture_track_x[index] = (r_buf[0 + 4 * index] & 0x0F) << 8 | (r_buf[1 + 4 * index] & 0xFF);
			data->gesture_track_y[index] = (r_buf[2 + 4 * index] & 0x0F) << 8 | (r_buf[3 + 4 * index] & 0xFF);
		}
#endif
	}

gesture_report:
	printk("%s:gestrue_id = %x\n", __func__, gesture_id);
	for (index = 0; index < data->pdata->gesture_num; index++) {
		if (gesture_id == data->pdata->gesture_figure_map[index])
			break;
	}
	if (index >= data->pdata->gesture_num) {
		printk("%s couldn't find the matched gesture.\n", __func__);
		return true;
	} else {
		if ((data->gesture_state) >> (index + 1) & 1) {
			input_report_key(data->input_dev, data->pdata->gesture_func_map[index], 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, data->pdata->gesture_func_map[index], 0);
			input_sync(data->input_dev);
		} else {
			printk("%s not open the gesture switch.\n", __func__);
		}
	}

	return true;
}
#endif

static irqreturn_t ft5x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x06_ts_data *data = dev_id;
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	int	count = 10;
#endif

	if (!data) {
		pr_err("%s: Invalid data\n", __func__);
		return IRQ_HANDLED;
	}
	if (data->suspended) {
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
		if (data->gesture_en) {
			do {
				if (ft5x06_report_gesture(data))
					break;
				msleep(GESTURE_LOOP_DELAY);
			} while (count--);
		}
#endif
	}	else {
		ft5x06_fetch_touchdata(data);
		ft5x06_adjust_touchdata(data);
		ft5x06_report_touchdata(data);
	}
	return IRQ_HANDLED;
}

static int ft5x06_power_on(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}

static int ft5x06_power_init(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int ft5x06_ts_pinctrl_init(struct ft5x06_ts_data *ft5x06_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ft5x06_data->ts_pinctrl = devm_pinctrl_get(&(ft5x06_data->client->dev));
	if (IS_ERR_OR_NULL(ft5x06_data->ts_pinctrl)) {
		dev_dbg(&ft5x06_data->client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(ft5x06_data->ts_pinctrl);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	ft5x06_data->gpio_state_active
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			"pmx_ts_active");
	if (IS_ERR_OR_NULL(ft5x06_data->gpio_state_active)) {
		dev_dbg(&ft5x06_data->client->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(ft5x06_data->gpio_state_active);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	ft5x06_data->gpio_state_suspend
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			"pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ft5x06_data->gpio_state_suspend)) {
		dev_err(&ft5x06_data->client->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(ft5x06_data->gpio_state_suspend);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int ft5x06_ts_pinctrl_select(struct ft5x06_ts_data *ft5x06_data,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? ft5x06_data->gpio_state_active
		: ft5x06_data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(ft5x06_data->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&ft5x06_data->client->dev,
				"can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else {
		dev_err(&ft5x06_data->client->dev,
			"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return 0;
}

#ifdef CONFIG_PM
static int ft5x06_ts_suspend(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	char txbuf[2];
	int err;
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	u8 gesture_switch, i;
#endif

	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}

#ifdef CONFIG_TOUCHSCREEN_FACE_DETECTION
	if (data->ps_en)
		return 0;
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	data->gesture_suspend_en = data->gesture_en;
	if (data->gesture_suspend_en) {
		for (i = 0; i < GESTURE_WRITE_LOOP; i++) {
			ft5x0x_write_reg(data->client, GESTURE_REG, GESTURE_ON);/*open ctp's gesture function*/
			ft5x0x_read_reg(data->client, GESTURE_REG, &gesture_switch);

			if (GESTURE_ON == gesture_switch) {
				dev_info(dev, "open ctp's gesture_fun.i = %d.(%s,%d)\n", i, __FUNCTION__, __LINE__);
				break;
			}

			dev_err(dev, "ctp's gesture_switch = %d\n", gesture_switch);
		}
		ft5x06_init_gesture(data);
	} else
#endif
	{
		disable_irq(data->client->irq);

		if (gpio_is_valid(data->pdata->reset_gpio)) {
				txbuf[0] = FT_REG_PMODE;
				txbuf[1] = FT_PMODE_HIBERNATE;
				err = ft5x06_i2c_write(data->client, txbuf, sizeof(txbuf));
				if (err < 0) {
				    dev_err(dev, "failed to be made sleep.\n");
				}
		}

		if (data->pdata->power_on) {
			err = data->pdata->power_on(false);
			if (err) {
				dev_err(dev, "power off failed");
				goto pwr_off_fail;
			}
		} else {
			err = ft5x06_power_on(data, false);
			if (err) {
				dev_err(dev, "power off failed");
				goto pwr_off_fail;
			}
		}
	}
	data->suspended = true;
	return 0;

pwr_off_fail:
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	enable_irq(data->client->irq);
	return err;
}

static void ft5x06_ts_resume_work(struct work_struct *work)
{
	struct ft5x06_ts_data *ts_data = container_of(work,
											   struct ft5x06_ts_data, resume_work);

	ft5x06_ts_resume(&ts_data->client->dev);
}

static int ft5x06_ts_resume(struct device *dev)
{
	return ft5x06_ts_resume_force(dev, false);
}
static int ft5x06_ts_resume_force(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	int err;
	int i;
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	u8 gesture_switch;
#endif

	if (!force && !data->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}
#ifdef CONFIG_TOUCHSCREEN_FACE_DETECTION
	if (data->ps_en) {
		if (data->tp_ps_en) {
			if (data->pdata->power_on) {
				err = data->pdata->power_on(true);
				if (err) {
					dev_err(dev, "power on failed");
					return err;
				}
			}	else {
				err = ft5x06_power_on(data, true);
				if (err) {
					dev_err(dev, "power on failed");
					return err;
				}
			}
			if (gpio_is_valid(data->pdata->reset_gpio)) {
				gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
				msleep(data->pdata->hard_rst_dly);
				gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
			}
			msleep(data->pdata->soft_rst_dly);
			enable_irq(data->client->irq);
			printk("%s: reset ic.\n", __func__);
			data->tp_ps_en = 0;
			err = ft5x0x_write_reg(data->client, 0XB0, 1);
			if (err < 0) {
				printk(KERN_ERR " enable tp ps function failed\n");
				data->ps_en = 0;
			}
			printk("%s: resend face state %d.\n", __func__, data->ps_en);
			err = enable_irq_wake(data->client->irq);
			if (0 != err) {
				printk(KERN_ERR " enable_irq_wake failed for tp_ps_irq_handler\n");
				ft5x0x_write_reg(data->client, 0XB0, 0);
				data->ps_en = 0;
			}
			data->suspended = false;
		}
		return 0;
	}
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	if (data->gesture_suspend_en) {
		for (i = 0; i < GESTURE_WRITE_LOOP; i++) {
			ft5x0x_write_reg(data->client, GESTURE_REG, GESTURE_OFF);/*close ctp's gesture function*/
			ft5x0x_read_reg(data->client, GESTURE_REG, &gesture_switch);
			if (GESTURE_OFF == gesture_switch) {
				dev_info(dev, "close ctp's gesture_fun.i = %d.\n", i);
				break;
			}
		}
		if (gpio_is_valid(data->pdata->reset_gpio)) {
			gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		}
		msleep(data->pdata->soft_rst_dly);
	} else
#endif
	{
		if (data->pdata->power_on) {
			err = data->pdata->power_on(true);
			if (err) {
				dev_err(dev, "power on failed");
				return err;
			}
		}	else {
			err = ft5x06_power_on(data, true);
			if (err) {
				dev_err(dev, "power on failed");
				return err;
			}
		}

		if (gpio_is_valid(data->pdata->reset_gpio)) {
			gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		}
		msleep(data->pdata->soft_rst_dly);
		enable_irq(data->client->irq);
	}
	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 1);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);
	data->suspended = false;

#if defined(CONFIG_TOUCHSCREEN_FT5X06_FH)
	if (usb_flag) {
		ctp_work_with_ac_usb_plugin(true);
	}	else {
		ctp_work_with_ac_usb_plugin(false);
	}
#endif

	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ft5x06_ts_data *ft5x06_data =
		container_of(self, struct ft5x06_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ft5x06_data && ft5x06_data->client) {
		blank = evdata->data;
		printk("fb_notifier_callback:ft5x06_tp blank=%d\n", *blank);
		if (*blank == FB_BLANK_UNBLANK) {
			if (!work_pending(&ft5x06_data->resume_work)) {
				schedule_work(&ft5x06_data->resume_work);
			}
		} else if (*blank == FB_BLANK_POWERDOWN) {
			/* not use (work_pending(&ft5x06_data->resume_work)) */
			cancel_work_sync(&ft5x06_data->resume_work);
			ft5x06_ts_suspend(&ft5x06_data->client->dev);
		}
#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
		ft5x06_data->ps_lcd_state = *blank;
#endif
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x06_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_suspend(&data->client->dev);
}

static void ft5x06_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_resume(&data->client->dev);
}
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
static int ft5x06_ts_suspend_gesture(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (data->gesture_suspend_en) {
		dev_info(dev, "ft5x06_ts_suspend_gesture\n");
		disable_irq(data->client->irq);
		enable_irq_wake(data->client->irq);/*make tp can wake the system*/
	}

	return 0;
}
static int ft5x06_ts_resume_gesture(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (data->gesture_suspend_en) {
		dev_info(dev, "ft5x06_ts_resume_gesture\n");
		disable_irq_wake(data->client->irq);/*make tp cannot wake the system*/
		enable_irq(data->client->irq);
	}

	return 0;
}
#endif

static const struct dev_pm_ops ft5x06_ts_pm_ops = {
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	.suspend = ft5x06_ts_suspend_gesture,
	.resume = ft5x06_ts_resume_gesture,
#endif
};
#endif

static int ft5x06_auto_cal(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	u8 temp = 0, i;

	/* set to factory mode */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* start calibration */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_START);
	msleep(2 * data->pdata->soft_rst_dly);
	for (i = 0; i < FT_CAL_RETRY; i++) {
		ft5x0x_read_reg(client, FT_REG_CAL, &temp);
		/*return to normal mode, calibration finish */
		if (((temp & FT_CAL_MASK) >> FT_4BIT_SHIFT) == FT_CAL_FIN)
			break;
	}

	/*calibration OK */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* store calibration data */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_STORE);
	msleep(2 * data->pdata->soft_rst_dly);

	/* set to normal mode */
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_WORKMODE_VALUE);
	msleep(2 * data->pdata->soft_rst_dly);

	return 0;
}

static int ft5x06_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;
	u8 is_5336_new_bootloader = false;
	u8 is_5336_fwsize_30 = false;
	u8 fw_ecc;

	/* determine firmware size */
	if (*(data + data_len - FT_BLOADER_SIZE_OFF) == FT_BLOADER_NEW_SIZE)
		is_5336_fwsize_30 = true;
	else
		is_5336_fwsize_30 = false;

	if (ft5x06_enter_update(ts_data) != 0) {
		dev_info(&client->dev, "enter update mode failed and Reset the CTP.(%s,%d)\n", __FUNCTION__, __LINE__);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->soft_rst_dly);

		return -EIO;
	}

	w_buf[0] = 0xcd;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] <= 4)
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;
	else if (r_buf[0] == 7)
		is_5336_new_bootloader = FT_BLOADER_VERSION_Z7;
	else if (r_buf[0] >= 0x0f &&
		((ts_data->family_id == FT_FT5336_FAMILY_ID_0x11) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x12) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x13) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x14)))
		is_5336_new_bootloader = FT_BLOADER_VERSION_GZF;
	else
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(info.delay_erase_flash);

	if (is_5336_fwsize_30) {
		w_buf[0] = FT_ERASE_PANEL_REG;
		ft5x06_i2c_write(client, w_buf, 1);
	}
	msleep(FT_EARSE_DLY_MS);

	/* program firmware */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4
		|| is_5336_new_bootloader == FT_BLOADER_VERSION_Z7)
		data_len = data_len - FT_DATA_LEN_OFF_OLD_FW;
	else
		data_len = data_len - FT_DATA_LEN_OFF_NEW_FW;

	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send remaining bytes */
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send the finishing packet */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4 ||
		is_5336_new_bootloader == FT_BLOADER_VERSION_Z7) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_OLD_FW; i++) {
			if (is_5336_new_bootloader  == FT_BLOADER_VERSION_Z7)
				temp = FT_MAGIC_BLOADER_Z7 + i;
			else if (is_5336_new_bootloader ==
						FT_BLOADER_VERSION_LZ4)
				temp = FT_MAGIC_BLOADER_LZ4 + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);
		}
	} else if (is_5336_new_bootloader == FT_BLOADER_VERSION_GZF) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_NEW_FW; i++) {
			if (is_5336_fwsize_30)
				temp = FT_MAGIC_BLOADER_GZF_30 + i;
			else
				temp = FT_MAGIC_BLOADER_GZF + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);

		}
	}

	/* verify checksum */
	w_buf[0] = FT_REG_ECC;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		dev_err(&client->dev, "ECC error! dev_ecc=%02x fw_ecc=%02x\n",
					r_buf[0], fw_ecc);
		return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	dev_info(&client->dev, "Firmware upgrade finish.\n");

	return 0;
}

static int hidi2c_to_stdi2c(struct i2c_client *client)
{
	u8 w_buf[5] = {0};
	int ret = 0;

	w_buf[0] = 0xeb;
	w_buf[1] = 0xaa;
	w_buf[2] = 0x09;
	ret = ft5x06_i2c_write(client, w_buf, 3);
	msleep(10);
	w_buf[0] = w_buf[1] = w_buf[2] = 0;
	ft5x06_i2c_read(client, w_buf, 0, w_buf, 3);

	if (0xeb == w_buf[0] && 0xaa == w_buf[1] && 0x08 == w_buf[2]) {
		dev_info(&client->dev, "hidi2c_to_stdi2c successful.\n");
		ret = 1;
	} else {
		dev_info(&client->dev, "hidi2c_to_stdi2c error.\n");
		ret = 0;
	}

	return ret;
}

static int ft5x46_enter_update(struct ft5x06_ts_data *ts_data)
{
	struct fw_upgrade_info info = ts_data->pdata->info;
	struct i2c_client *client = ts_data->client;
	u8 reset_reg;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	int i, j, ret;

	ret = hidi2c_to_stdi2c(client);
	if (ret == 0) {
		dev_err(&client->dev, "[FTS] hid change to i2c fail ! \n");
	}


	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
		for (j = 0; j < FT_UPGRADE_LOOP; j++) {
			if (FT6X06_ID == ts_data->family_id || FT_FT6436_FAMILY_ID_0x36 == ts_data->family_id) {
				reset_reg = FT_RST_CMD_REG2;
			} else {
				reset_reg = FT_RST_CMD_REG1;
			}

			/* Reset the Ctp */
			dev_info(&ts_data->client->dev, " Step 1:Reset the ctp\n");
			dev_info(&ts_data->client->dev, " i=%d,info.delay_aa=%d\n", i, (info.delay_aa + 5*i));
			ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
			mdelay(info.delay_aa + 5*i);
			dev_info(&ts_data->client->dev, " j=%d,info.delay_55=%d\n", j, (info.delay_55 + 10*j));
			ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
			mdelay(info.delay_55 + 5*j);

			ret = hidi2c_to_stdi2c(client);
			if (ret == 0) {
				dev_err(&client->dev, "[FTS] hid change to i2c fail ! \n");
			}
			msleep(10);
			/* Enter upgrade mode */
			dev_info(&ts_data->client->dev, " Step 2:Enter upgrade mode.\n");
			w_buf[0] = FT_UPGRADE_55;
			w_buf[1] = FT_UPGRADE_AA;
			ft5x06_i2c_write(client, w_buf, 2);

			/* check READ_ID */
			dev_info(&ts_data->client->dev, " Step 3: Check IC ID.\n");
			mdelay(info.delay_readid);
			w_buf[0] = FT_READ_ID_REG;
			w_buf[1] = 0x00;
			w_buf[2] = 0x00;
			w_buf[3] = 0x00;
			ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
			if ((r_buf[0] != info.upgrade_id_1) || (r_buf[1] != info.upgrade_id_2)) {
				dev_err(&client->dev, " Upgrade ID(%u,%u) mismatch (%u,%u)(%d)\n",
						r_buf[0], r_buf[1], info.upgrade_id_1, info.upgrade_id_2, i);
			} else {
				dev_info(&ts_data->client->dev, "IC_ID_1=0x%x,IC_ID_2=0x%x.\n", r_buf[0], r_buf[1]);
				break;
			}
		}
		if (j != FT_UPGRADE_LOOP)
			break;
	}

	if (i == FT_UPGRADE_LOOP) {
		dev_info(&ts_data->client->dev, "Soft Reset Style Check IC ID ERROR!\n");
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}
	dev_info(&ts_data->client->dev, " F5x46 enter update success.\n");
	msleep(50);
	return 0;
}

static int ft5x46_leave_update(struct ft5x06_ts_data *data)
{
	/* reset */
	u8 w_buf[FT_MAX_WR_BUF] = {0};
	int ret;

	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(data->client, w_buf, 1);
	msleep(data->pdata->soft_rst_dly);
	ret = hidi2c_to_stdi2c(data->client);
	if (ret == 0) {
		dev_err(&data->client->dev, "[FTS] hid change to i2c fail ! \n");
	}
	dev_info(&data->client->dev, "Ft5x46 leave update!\n");
	return 0;
}

static int ft5x46_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;
	u8 fw_ecc;
	if (ft5x46_enter_update(ts_data) != 0) {
		dev_info(&client->dev, "enter update mode failed and Reset the CTP.(%s,%d)\n", __FUNCTION__, __LINE__);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->soft_rst_dly);

		return -EIO;
	}

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(1350);
	for (i = 0; i < 15; i++) {
		w_buf[0] = 0x6a;
		r_buf[0] = 0x00;
		r_buf[1] = 0x00;
		ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);

		if (0xF0 == r_buf[0] && 0xAA == r_buf[1]) {
			break;
		} else {
			dev_err(&client->dev, "%d != 0xF0,%d != 0xAA\n", r_buf[0], r_buf[1]);
		}
		mdelay(50);
	}

	w_buf[0] = 0xB0;
	w_buf[1] = (u8)((data_len >> 16) & 0xFF);
	w_buf[2] = (u8)((data_len >> 8) & 0xFF);
	w_buf[3] = (u8)(data_len & 0xFF);
	ft5x06_i2c_write(client, w_buf, 4);
	/*write firmware*/
	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);

		for (j = 0; j < 30; j++) {
			w_buf[0] = 0x6a;
			r_buf[0] = 0x00;
			r_buf[0] = 0x00;
			ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);
			if ((i + 0x1000) == (((r_buf[0]) << 8) | r_buf[1])) {
				break;
			}
			mdelay(5);
		}
	}

	/*send the remaining bytes*/
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);

		for (i = 0; i < 30; i++) {
			w_buf[0] = 0x6a;
			r_buf[0] = 0x00;
			r_buf[0] = 0x00;
			ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);
			if ((j + 0x1000) == (((r_buf[0]) << 8) | r_buf[1])) {
				break;
			}
			mdelay(5);
		}
	}

	/*read out checksum*/
	w_buf[0] = 0x64;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(100);

	temp = 0;
	w_buf[0] = 0x65;
	w_buf[1] = (u8)(temp >> 16);
	w_buf[2] = (u8)(temp >> 8);
	w_buf[3] = (u8)(temp);
	temp = data_len;
	w_buf[4] = (u8)(temp >> 8);
	w_buf[5] = (u8)(temp);
	ft5x06_i2c_write(client, w_buf, 6);
	msleep(200);
	for (i = 0; i < 100; i++) {
		w_buf[0] = 0x6a;
		r_buf[0] = 0x00;
		r_buf[1] = 0x00;
		ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);
		if (0xF0 == r_buf[0] && 0x55 == r_buf[1]) {
			break;
		}
		mdelay(5);
	}

	w_buf[0] = 0x66;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] != fw_ecc) {
			dev_err(&client->dev, "ECC error! dev_ecc=%02x fw_ecc=%02x\n",
						r_buf[0], fw_ecc);
			return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	dev_info(&client->dev, "Firmware upgrade finish.\n");

	return 0;
}

static int ft6x36_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;
	u8 fw_ecc;

	/*check fw first byte*/
	if (data[0] != 0x02) {
		dev_err(&client->dev, "FW first byte is not 0x02,not valid.");
		return -EIO;
	}
	/*check fw lenght*/
	if (data_len > 0x11f) {
		pkt_len = ((u32)data[0x100] << 8) + data[101];
		if (data_len < pkt_len) {
			dev_err(&client->dev, "FW lenght is invalid.");
			return -EIO;
		}
	} else {
		dev_err(&client->dev, "FW lenght is invalid.");
		return -EIO;
	}

	/*Enter update mode*/
	if (ft5x06_enter_update(ts_data) != 0) {
		dev_info(&client->dev, "enter update mode failed and Reset the CTP.(%s,%d)\n", __FUNCTION__, __LINE__);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->soft_rst_dly);

		return -EIO;
	}

	/*erase app and panel paramenter area*/
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(info.delay_erase_flash);

	for (i = 0; i < 200; i++) {
		w_buf[0] = 0x6a;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;
		r_buf[0] = 0x00;
		r_buf[1] = 0x00;
		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
		if (0xb0 == r_buf[0] && 0x02 == r_buf[1]) {
			dev_info(&client->dev, "erase app finished.");
			break;
		}
		msleep(50);
	}

	/*write firmware to ctpm flash*/
	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);

		for (j = 0; j < 30; j++) {
			w_buf[0] = 0x6a;
			w_buf[1] = 0x00;
			w_buf[2] = 0x00;
			w_buf[3] = 0x00;
			r_buf[0] = 0x00;
			r_buf[1] = 0x00;
			ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
			if (0xb0 == (r_buf[0] & 0xf0) && (0x03 + (i % 0x0ffd)) == (((r_buf[0] & 0x0f) << 8) | r_buf[1])) {
				break;
			}
			msleep(1);
		}
	}

	/*send the remaining bytes*/
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		for (j = 0; j < 30; j++) {
			w_buf[0] = 0x6a;
			w_buf[1] = 0x00;
			w_buf[2] = 0x00;
			w_buf[3] = 0x00;
			r_buf[0] = 0x00;
			r_buf[1] = 0x00;
			ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
			if (0xb0 == (r_buf[0] & 0xf0) && (0x03 + (i % 0x0ffd)) == (((r_buf[0] & 0x0f) << 8) | r_buf[1])) {
				break;
			}
			msleep(1);
		}
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* verify checksum */
	w_buf[0] = FT_REG_ECC;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		dev_err(&client->dev, "ECC error! dev_ecc=%02x fw_ecc=%02x\n",
					r_buf[0], fw_ecc);
		return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	dev_info(&client->dev, "Firmware upgrade finish.\n");
	return 0;
}

static int ft_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	int ret;

	if (FT_FT5446_FAMILY_ID_0x54 == ts_data->family_id) {
		ret = ft5x46_fw_upgrade_start(client, data, data_len);
	}	else if (FT_FT6436_FAMILY_ID_0x36 == ts_data->family_id) {
		ret = ft6x36_fw_upgrade_start(client, data, data_len);
	} else {
		ret = ft5x06_fw_upgrade_start(client, data, data_len);
	}

	return ret;
}

static int ft5x06_fw_upgrade(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int rc;
	struct upgrade_config config = {
		.enter_upgrade_mode = false,
		.need_upgrade = false,
		.firmware = NULL,
	};

	rc = ft5x06_get_fw_upgrade_config(data, &config);
	fw = config.firmware;

	if (rc < 0) {
		goto rel_fw;
	}

	if (!force && !config.need_upgrade) {
		dev_info(dev, "Exiting fw upgrade...\n");
		rc = -EFAULT;
		goto rel_fw;
	}

	/* start firmware upgrade */
	/*if (FT_FW_CHECK(fw))  {*/
	if (1) {
		rc = ft_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0) {
			dev_err(dev, "update failed (%d). try later...\n", rc);
			goto rel_fw;
		} else if (data->pdata->info.auto_cal) {
			ft5x06_auto_cal(data->client);
		}
	} else {
		dev_err(dev, "FW format error\n");
		rc = -EIO;
	}

	ft5x06_update_fw_ver(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);

	ft5x06_ts_register_productinfo(data);

rel_fw:
	if (config.enter_upgrade_mode == true) {
		if (FT_FT5446_FAMILY_ID_0x54 == data->family_id)
			ft5x46_leave_update(data);
		else
			ft5x06_leave_update(data);
	}
	release_firmware(fw);
	return rc;
}

static ssize_t ft5x06_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t ft5x06_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

#if 1
	if (val) {
		ft5x06_triger_update(dev, false);
	}
#else
	if (data->suspended) {
		dev_info(dev, "In suspend state, try again later...\n");
		return size;
	}

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, false);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);
#endif
	return size;
}

static DEVICE_ATTR(update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_update_fw_store);

static ssize_t ft5x06_force_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;
#if 1
	if (val) {
		ft5x06_triger_update(dev, true);
	}
#else
	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, true);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);
#endif
	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_force_update_fw_store);

static ssize_t ft5x06_fw_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, FT_FW_NAME_MAX_LEN - 1, "%s\n", data->fw_name);
}

static ssize_t ft5x06_fw_name_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (size > FT_FW_NAME_MAX_LEN - 1)
		return -EINVAL;

	strlcpy(data->fw_name, buf, size);
	if (data->fw_name[size-1] == '\n')
		data->fw_name[size-1] = 0;

	return size;
}

static DEVICE_ATTR(fw_name, 0664, ft5x06_fw_name_show, ft5x06_fw_name_store);

static bool ft5x06_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int ft5x06_debug_data_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_data_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;
	int rc;
	u8 reg;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr)) {
		rc = ft5x0x_read_reg(data->client, data->addr, &reg);
		if (rc < 0)
			dev_err(&data->client->dev,
				"FT read register 0x%x failed (%d)\n",
				data->addr, rc);
		else
			*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, ft5x06_debug_data_get,
			ft5x06_debug_data_set, "0x%02llX\n");

static int ft5x06_debug_addr_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	if (ft5x06_debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int ft5x06_debug_addr_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, ft5x06_debug_addr_get,
			ft5x06_debug_addr_set, "0x%02llX\n");

static int ft5x06_debug_suspend_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		ft5x06_ts_suspend(&data->client->dev);
	else
		ft5x06_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_suspend_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, ft5x06_debug_suspend_get,
			ft5x06_debug_suspend_set, "%lld\n");

static int ft5x06_debug_dump_info(struct seq_file *m, void *v)
{
	struct ft5x06_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, ft5x06_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};

#ifdef CONFIG_OF
static int ft5x06_get_dt_coords(struct device *dev, char *name,
				struct ft5x06_ts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];
	u8 read_flash_cmd[] = {0x3, 0x00, 0x07, 0xb4};
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	int i = 0;
	u32 gesture_map[MAX_GESTURE];
#ifndef CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER
	u32 gesture_reg_map[MAX_GESTURE_REG];
#endif
#endif

	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = ft5x06_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft5x06_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"focaltech,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->reset_gpio_flags);

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->irq_gpio_flags);

	pdata->fw_name_pref = "ft_fw.bin";
	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name_pref);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw name\n");
		return rc;
	}

	rc = of_property_read_u32(np, "focaltech,group-id", &temp_val);
	if (!rc)
		pdata->group_id = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.auto_cal = of_property_read_bool(np,
					"focaltech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np,
						"focaltech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np,
						"focaltech,ignore-id-check");

	pdata->support_usb_check = of_property_read_bool(np,
							"focaltech,support-usb-check");

	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,button-map", button_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	prop = of_find_property(np, "focaltech,gesture-func-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_GESTURE)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,gesture-func-map", gesture_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read gesture func map\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			pdata->gesture_func_map[i] = gesture_map[i];
		pdata->gesture_num = num_buttons;
	}
	prop = of_find_property(np, "focaltech,gesture-figure-map", NULL);
	if (prop) {
		rc = of_property_read_u32_array(np,
			"focaltech,gesture-figure-map", gesture_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read gesture figure map\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			pdata->gesture_figure_map[i] = gesture_map[i];
	}
#ifndef CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER
	prop = of_find_property(np, "focaltech,gesture-reg-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_GESTURE_REG)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,gesture-reg-map", gesture_reg_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read gesture reg map\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			pdata->gesture_reg_map[i] = gesture_reg_map[i];
		pdata->gesture_reg_num = num_buttons;
	}
	prop = of_find_property(np, "focaltech,gesture-reg-value-map", NULL);
	if (prop) {
		rc = of_property_read_u32_array(np,
			"focaltech,gesture-reg-value-map", gesture_reg_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read gesture reg value map\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			pdata->gesture_reg_value_map[i] = gesture_reg_map[i];
	}
#endif
#endif

	prop = of_find_property(np, "focaltech,fw-rawdata-range", NULL);
	if (prop) {
		rc = of_property_read_u32_array(np,
			"focaltech,fw-rawdata-range", pdata->rawdata_range, ARRAY_SIZE(pdata->rawdata_range));
	}

	prop = of_find_property(np, "focaltech,fw-panelid-command", NULL);
	if (prop && prop->value) {
		memcpy(pdata->panelid_command, prop->value, sizeof(pdata->panelid_command));
	} else {
		memcpy(pdata->panelid_command, read_flash_cmd, sizeof(pdata->panelid_command));
	}

	rc = of_property_read_u32(np, "focaltech,touch-area-param",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read touch area param\n");
		return rc;
	} else if (rc) {
		pdata->touch_area_param = 1;
	} else {
		pdata->touch_area_param = temp_val;
	}

	return 0;
}
#else
static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static void ft5x06_ts_register_productinfo(struct ft5x06_ts_data *ts_data)
{
	/* format as flow: version:0x01 Module id:0x57*/
	char deviceinfo[64];
	snprintf(deviceinfo, ARRAY_SIZE(deviceinfo), "FW version:0x%2x Module id:0x%2x",
								ts_data->fw_ver[0], ts_data->pannel_id);

	productinfo_register(PRODUCTINFO_CTP_ID, NULL, deviceinfo);
}
#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
static void psensor_report_dist(struct ft5x06_ts_data *data, int dist_code)
{
	input_report_abs(data->ps_input_dev, ABS_DISTANCE, dist_code ? 1000 : 1);
	input_report_abs(data->ps_input_dev, ABS_DISTANCE, dist_code ? 1023 : 0);
	input_sync(data->ps_input_dev);
	if (dist_code == FAR_CODE) {
		if (!wake_lock_active(&data->ps_wake_lock)) {
			printk("wake_lock PROX_NEAR_TO_FAR_WLOCK not be locked, and lock it!\n");
			wake_lock_timeout(&data->ps_wake_lock, HZ);
		} else {
			printk("wake_lock PROX_NEAR_TO_FAR_WLOCK be locked, do nothing!\n");
		}
	}
}

static int tp_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ft5x06_ts_data *data =
		container_of(sensors_cdev, struct ft5x06_ts_data, ps_cdev);
	int ret = 0;
	if ((enable < 0) || (enable > 1)) {
		printk("%s(): It is an illegal para %d!\n", __func__, enable);
		return -EINVAL;
	}

	if (data->ps_en == enable)
		return 0;

	if (data->suspended) {
		data->tp_ps_en = 1;/*send in resume*/
		data->ps_en = enable;
		last_ps_val = 0;	/* permit the last val diff with the current ps state.*/
		dev_err(&data->client->dev, "%s: tp is in suspend, ps_en = %d, resend in resume.\n", __func__, enable);
		return 0;
	}
	dev_err(&data->client->dev, "%s: ps_en = %d\n", __func__, enable);

	ret = ft5x0x_write_reg(data->client, 0XB0, enable);
	if (ret < 0) {
		printk(KERN_ERR " enable tp ps function failed\n");
		enable = 0;
		/* Add set ps func off fail process */
		goto quit;
	}
	ret = enable ?
		enable_irq_wake(data->client->irq) :
		disable_irq_wake(data->client->irq);
	if (0 != ret) {
		printk(KERN_ERR " enable_irq_wake failed for tp_ps_irq_handler\n");
		ft5x0x_write_reg(data->client, 0XB0, 0);
		enable = 0;
	}
quit:
	data->ps_en = enable;
	last_ps_val = enable ? 0x07 : 0;
	/* Force report a FAR value when excute this function.*/
	psensor_report_dist(data, FAR_CODE);
	return 0;
}

static int tp_ps_input_init(struct ft5x06_ts_data *data)
{
	int ret = 0;

	data->ps_input_dev = input_allocate_device();
	if (!data->ps_input_dev) {
		pr_err("[PS][FocalTeck TP error]%s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	data->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, data->ps_input_dev->evbit);
	input_set_abs_params(data->ps_input_dev, ABS_DISTANCE, 0, 1023, 0, 0);
	input_set_drvdata(data->ps_input_dev, data);

	ret = input_register_device(data->ps_input_dev);
	if (ret < 0) {
		pr_err(
			"[PS][CM36686 error]%s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}
	return ret;

err_free_ps_input_device:
	input_free_device(data->ps_input_dev);
	return ret;
}

int powerkey_close_face_detect(void)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(s_ctp_dev);
	if (data->ps_en)	{
		printk("%s lcd state:%d.\n", __func__, data->ps_lcd_state);
		if (data->ps_lcd_state == FB_BLANK_POWERDOWN && data->tp_ps_en == 0) {
			input_report_key(data->input_dev, KEY_POWER, 1);
			input_report_key(data->input_dev, KEY_POWER, 0);
			input_sync(data->input_dev);
			printk("%s: powerkey send FAR state.\n", __func__);
			psensor_report_dist(data, FAR_CODE);
		}
	}
	return 0;
}
EXPORT_SYMBOL(powerkey_close_face_detect);

#endif
static int ft5x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x06_ts_platform_data *pdata;
	struct ft5x06_ts_data *data;
	struct input_dev *input_dev;
#if defined(CONFIG_DEBUG_FS)
	struct dentry *temp;
#endif
	u8 reg_value;
	u8 reg_addr;
	int err, len;
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	int index;
#endif

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = ft5x06_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FT_FW_NAME_MAX_LEN - 1) {
			dev_err(&client->dev, "Invalid firmware name\n");
			return -EINVAL;
		}
		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	data->tch_data_len = FT_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev,
				data->tch_data_len, GFP_KERNEL);

	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	data->event[0] = ft5x06_allocate_touch_event_seq(&client->dev, pdata->num_max_touches);
	data->event[1] = ft5x06_allocate_touch_event_seq(&client->dev, pdata->num_max_touches);
	if (!data->event[0] || !data->event[1]) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	input_dev->name = "ft5x06_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	for (index = 0; index < data->pdata->gesture_num; index++) {
		__set_bit(data->pdata->gesture_func_map[index], input_dev->keybit);
	}
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER)
	init_para(data->pdata->x_max, data->pdata->y_max, 100, 0, 0);
#endif
	data->gesture_state = 0x00;
#endif

	input_mt_init_slots(input_dev, pdata->num_max_touches, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,
			     pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	/*input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);*/

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
		goto free_inputdev;
	}
#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
	__set_bit(KEY_POWER, input_dev->keybit);
	err = tp_ps_input_init(data);
	if (err) {
		dev_err(&client->dev,
			"PS function Input device registration failed\n");
		goto unreg_inputdev;
	}

	data->ps_cdev = tp_proximity_cdev;
	data->ps_cdev.sensors_enable = tp_ps_set_enable;
	data->ps_cdev.sensors_poll_delay = NULL,

	err = sensors_classdev_register(&client->dev, &data->ps_cdev);
	if (err) {
		pr_err("%s: Unable to register to sensors class: %d\n",
				__func__, err);
		goto unreg_ps_inputdev;
	}

	wake_lock_init(&(data->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity_tp");
#endif

	err = (pdata->power_init != NULL) ?
		pdata->power_init(true) :
		ft5x06_power_init(data, true);
	if (err) {
		dev_err(&client->dev, "power init failed");
#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
		goto unreg_sns_dev;
#else
		goto unreg_inputdev;
#endif
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}

	err = ft5x06_ts_pinctrl_init(data);
	if (!err && data->ts_pinctrl) {
		err = ft5x06_ts_pinctrl_select(data, true);
		if (err < 0)
			goto pwr_off;
	}

	if (gpio_is_valid(pdata->irq_gpio)) {
		err = gpio_request(pdata->irq_gpio, "ft5x06_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
			goto pwr_off;
		}
		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio, "ft5x06_reset_gpio");
		if (err) {
			dev_err(&client->dev, "reset gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->reset_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	/* make sure CTP already finish startup process */
	msleep(data->pdata->soft_rst_dly);

	/* check the controller id */
	reg_value = 0;
	reg_addr = FT_REG_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "version read failed");
		goto free_reset_gpio;
	}

	dev_info(&client->dev, "Device ID = 0x%x\n", reg_value);

	if (pdata->family_id != reg_value) {
		dev_err(&client->dev, "%s:Unsupported controller\n", __func__);
		/*goto free_reset_gpio;*/
	}
	data->family_id = reg_value ?  reg_value : pdata->family_id;

	err = request_threaded_irq(client->irq, NULL,
				ft5x06_ts_interrupt,
				pdata->irqflags | IRQF_ONESHOT,
				client->dev.driver->name, data);
	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto free_reset_gpio;
	}

	err = device_create_file(&client->dev, &dev_attr_fw_name);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto irq_free;
	}

	err = device_create_file(&client->dev, &dev_attr_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_fw_name_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_force_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_update_fw_sys;
	}
#if defined(CONFIG_DEBUG_FS)
	data->dir = debugfs_create_dir(FT_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
		goto free_force_update_fw_sys;
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}
#endif
	data->ts_info = devm_kzalloc(&client->dev,
				FT_INFO_MAX_LEN, GFP_KERNEL);
	if (!data->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
#if defined(CONFIG_DEBUG_FS)
		goto free_debug_dir;
#else
		goto free_force_update_fw_sys;
#endif
	}

	/*get some register information */
	reg_addr = FT_REG_POINT_RATE;
	ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "report rate read failed");

	dev_info(&client->dev, "report rate = %dHz\n", reg_value * 10);

	reg_addr = FT_REG_THGROUP;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "threshold read failed");

	dev_dbg(&client->dev, "touch threshold = %d\n", reg_value * 4);

	ft5x06_update_fw_ver(data);

	reg_addr = FT_RGE_PANNEL_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "Pannel Id version read failed");
	dev_info(&client->dev, "Pannel Id version = 0x%x\n", reg_value);
	data->pannel_id =  reg_value;

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);

	factory_ts_func_test_register(data);

	ft5x06_ts_register_productinfo(data);

#if defined(CONFIG_FB)

	INIT_WORK(&data->resume_work, ft5x06_ts_resume_work);

	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	data->early_suspend.suspend = ft5x06_ts_early_suspend;
	data->early_suspend.resume = ft5x06_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	ctp_register_device(&data->client->dev);
	G_Client = data->client;

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	data->gesture_en = false;
#endif

   return 0;

#if defined(CONFIG_DEBUG_FS)
free_debug_dir:
	debugfs_remove_recursive(data->dir);
#endif
free_force_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
free_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_update_fw);
free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
irq_free:
	free_irq(client->irq, data);
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
	if (data->ts_pinctrl) {
		if (ft5x06_ts_pinctrl_select(data, false) < 0) {
			pr_err("Cannot get idle pinctrl state\n");
		}
	}
pwr_off:
	if (pdata->power_on)
		pdata->power_on(false);
	else
		ft5x06_power_on(data, false);
pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		ft5x06_power_init(data, false);
#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
unreg_sns_dev:
	wake_lock_destroy(&(data->ps_wake_lock));
	sensors_classdev_unregister(&data->ps_cdev);
unreg_ps_inputdev:
	input_unregister_device(data->ps_input_dev);
#endif
unreg_inputdev:
	input_unregister_device(input_dev);
	input_dev = NULL;
free_inputdev:
	input_free_device(input_dev);
	return err;
}

static int ft5x06_ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	int rc;

	ctp_unregister_device(&data->client->dev);

#if defined(CONFIG_DEBUG_FS)
	debugfs_remove_recursive(data->dir);
#endif
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
	device_remove_file(&client->dev, &dev_attr_update_fw);
	device_remove_file(&client->dev, &dev_attr_fw_name);

#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	if (data->ts_pinctrl) {
		rc = ft5x06_ts_pinctrl_select(data, false);
		if (rc < 0)
			pr_err("Cannot get idle pinctrl state\n");
	}

	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		ft5x06_power_on(data, false);

	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		ft5x06_power_init(data, false);

	input_unregister_device(data->input_dev);

	return 0;
}

static const struct i2c_device_id ft5x06_ts_id[] = {
	{"ft5x06_ts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft5x06_match_table[] = {
	{ .compatible = "focaltech,5x06",},
	{ },
};
#else
#define ft5x06_match_table NULL
#endif

static struct i2c_driver ft5x06_ts_driver = {
	.probe = ft5x06_ts_probe,
	.remove = ft5x06_ts_remove,
	.driver = {
		   .name = "ft5x06_ts",
		   .owner = THIS_MODULE,
		.of_match_table = ft5x06_match_table,
#ifdef CONFIG_PM
		   .pm = &ft5x06_ts_pm_ops,
#endif
	},
	.id_table = ft5x06_ts_id,
};

static int __init ft5x06_ts_init(void)
{
	return i2c_add_driver(&ft5x06_ts_driver);
}
module_init(ft5x06_ts_init);

static void __exit ft5x06_ts_exit(void)
{
	i2c_del_driver(&ft5x06_ts_driver);
}
module_exit(ft5x06_ts_exit);

MODULE_DESCRIPTION("FocalTech ft5x06 TouchScreen driver");
MODULE_LICENSE("GPL v2");
