/*
 * Copyright (C) 2015 Hisense Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>

#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/version.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>

#include "tas2552.h"

/* do not change below */
#define MAX_BUFFER_SIZE		32

static const struct tas2552_reg_val_set tas2552_reg_val_init[] = {
	TAS2552_REG_VAL(0x01, 0x12),
	TAS2552_REG_VAL(0x08, 0x20),
	TAS2552_REG_VAL(0x03, 0x4d),
	TAS2552_REG_VAL(0x04, 0x00),
	TAS2552_REG_VAL(0x05, 0x00),
	TAS2552_REG_VAL(0x06, 0x00),
	TAS2552_REG_VAL(0x07, 0xc8),
	TAS2552_REG_VAL(0x09, 0x00),
	TAS2552_REG_VAL(0x0a, 0x00),
	TAS2552_REG_VAL(0x12, 0x15),
	TAS2552_REG_VAL(0x14, 0x0f),
	TAS2552_REG_VAL(0x0d, 0xc0),
	TAS2552_REG_VAL(0x0e, 0x20),
	TAS2552_REG_VAL(0x02, 0xea),
	TAS2552_REG_VAL(0x01, 0x10),
};

static struct tas2552 *tas2552_dev;
static int32_t temp_data1 = 0;
static int32_t temp_data2 = 0;
static int32_t audio_effect = AUDIO_EFFECT_0;
static int32_t voice_effect = AUDIO_EFFECT_1;
static bool audio_playback_status = false;
static bool voice_call_status = false;

/*
static int tas2552_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if(ret < 0)
		pr_err("%s: read err: %d\n", __func__, ret);

	return ret;
}
*/

int tas2552_do_calibration(void)
{
	int ret = 0;

	if (temp_data1 && temp_data2){
		ret = opalum_afe_set_param_calib(temp_data1, temp_data2);
		if (ret)
			pr_err("%s: err: %d\n", __func__, ret);
		pr_debug("%s: data1=%d,data2=%d\n",__func__, temp_data1, temp_data2);
	}

	return ret;
}

int tas2552_set_audio_effect(void)
{
	int ret = 0;
	int effect;

	if (voice_call_status == true ){
		if (voice_effect > AUDIO_EFFECT_0)
			effect = voice_effect;
		else
			return 0;
	} else {
		if (audio_effect > AUDIO_EFFECT_0)
			effect = audio_effect;
		else
			return 0;
	}
	pr_debug("%s: effect=%d, voice_call_status=%d\n",__func__,effect,voice_call_status);
	opalum_afe_set_param_effect(effect);
	/*Delay time added for the contex switch*/
	msleep(10);

	return ret;
}

void tas2552_set_voice_call_status(bool st)
{
	voice_call_status = st;
	pr_debug("%s: status=%d\n",__func__,voice_call_status);
}

void tas2552_set_playback_status( bool st)
{
	audio_playback_status = st;
}

static int tas2552_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);
	if (ret < 0)
		pr_err("%s: write err: %d\n", __func__, ret);

	return ret;
}

void  tas2552_hard_enable(int enable)
{
	pr_info("tas2552_hard_enable: enable =%d\n", enable);
	if (enable) {
		gpio_direction_output(tas2552_dev->en_gpio, 1);
		usleep_range(TAS2552_EN_DELAY,
			     TAS2552_EN_DELAY + TAS2552_EN_DELAY_DELTA);
	} else
		gpio_direction_output(tas2552_dev->en_gpio, 0);
}

void  tas2552_soft_enable(int enable)
{
	pr_info("tas2552_soft_enable: enable =%d\n", enable);
	if (enable) {
		tas2552_write_reg(tas2552_dev->client, 0x02, 0xea);
		tas2552_write_reg(tas2552_dev->client, 0x01, 0x10);
	} else {
		tas2552_write_reg(tas2552_dev->client, 0x01, 0x12);
		tas2552_write_reg(tas2552_dev->client, 0x02, 0x2a);
	}
}

void tas2552_init(void)
{
	int i;
	pr_info("tas2552_init\n");
	for (i = 0; i < ARRAY_SIZE(tas2552_reg_val_init); i++)
		tas2552_write_reg(tas2552_dev->client,
		tas2552_reg_val_init[i].reg, tas2552_reg_val_init[i].val);
}

static unsigned int tas2552_dev_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	return mask;
}

static ssize_t tas2552_dev_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *offset)
{
	struct tas2552 *tas2552_dev = filp->private_data;
	unsigned char tmp[MAX_BUFFER_SIZE];
	int total = 0;
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	ret = opalum_afe_get_calib_values( tas2552_dev->opalum_f0_values,
		                   tas2552_dev->opalum_temperature_values);
	if (ret < 0){
		pr_info("tas2552_dev_read:data is not valid\n");
		return ret;
	}

	memset(tmp, 0, sizeof(tmp));
	tmp[MAX_BUFFER_SIZE-1] = '\n';
	sprintf(tmp, "f0 %d %d temp %d %d",tas2552_dev->opalum_f0_values[0],
		tas2552_dev->opalum_f0_values[1],tas2552_dev->opalum_temperature_values[0],
							tas2552_dev->opalum_temperature_values[1]);

	total = sizeof(tmp);
	if (copy_to_user(buf, tmp, total)){
		dev_err(&tas2552_dev->client->dev,
			"failed to copy to user space\n");
	};

	pr_info("tas2552_dev_read, tmp=%s,total=%d\n",tmp,total);

	return total;
}

static ssize_t tas2552_dev_write(struct file *filp, const char __user *buf,
				   size_t count, loff_t *offset)
{
	struct tas2552 *tas2552_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	char *start = tmp;
	unsigned long data1, data2;
	unsigned long effect;
	size_t buf_size;

	if (count > MAX_BUFFER_SIZE) {
		dev_err(&tas2552_dev->client->dev, "out of memory\n");
		return -ENOMEM;
	}
	buf_size = min(count, (sizeof(tmp)-1));
	tmp[buf_size] = 0;
	if (copy_from_user(tmp, buf, count)) {
		dev_err(&tas2552_dev->client->dev,
			"failed to copy from user space\n");
		return -EFAULT;
	}
	if (!strncmp(tmp, "start", 5)) {
		pr_info("tas2552_dev_write: start\n");
		opalum_afe_set_param_diag(AFE_SET_DIAG_TX);
		opalum_afe_set_param_diag(AFE_SET_DIAG_RX);
	}else if (!strncmp(tmp, "get", 3)) {
		pr_info("tas2552_dev_write: get\n");
		opalum_afe_get_param(AFE_GET_F0);
		opalum_afe_get_param(AFE_GET_TEMPERATUE);
	}else if (!strncmp(tmp, "set", 3)) {
		start+=3;
		while (*start == ' ')
			start++;
		data1 = simple_strtoul(start, &start, 10);
		while (*start == ' ')
			start++;
		if (strict_strtoul(start, 10, &data2))
			printk("strict_strtoul error\n");

		pr_info("tas2552_dev_write: set data1=%lu,data2=%lu\n",data1, data2);
		/*opalum_afe_set_param_calib((int32_t)data1, (int32_t)data2);*/
		temp_data1 = (int32_t)data1;
		temp_data2 = (int32_t)data2;
	}else if (!strncmp(tmp, "effect", 6)) {
		start+=6;
		effect = simple_strtoul(start, &start, 10);
		pr_info("tas2552_dev_write: audio effect=%lu\n",effect);
		if(effect >=AUDIO_EFFECT_0 && effect < AUDIO_EFFECT_MAX)
			audio_effect = effect;
		if(audio_playback_status){
			opalum_afe_set_param_effect(audio_effect);
			msleep(10);
			tas2552_do_calibration();
		}
	}else if (!strncmp(tmp, "voice", 5)) {
		start+=5;
		effect = simple_strtoul(start, &start, 10);
		pr_info("tas2552_dev_write: voice effect=%lu\n",effect);
		if(effect >=AUDIO_EFFECT_0 && effect < AUDIO_EFFECT_MAX)
			voice_effect = effect;
		if(voice_call_status){
			opalum_afe_set_param_effect(voice_effect);
			msleep(10);
			tas2552_do_calibration();
		}
	}

	return count;
}

static long tas2552_dev_unlocked_ioctl(struct file *filp,
					 unsigned int cmd, unsigned long arg)
{
	/*struct tas2552 *tas2552_dev = filp->private_data;*/
	return 0;
}

static int tas2552_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct tas2552 *tas2552_dev = container_of(filp->private_data,
							   struct tas2552,
							   tas2552_device);
	filp->private_data = tas2552_dev;
	dev_info(&tas2552_dev->client->dev,
		 "device node major=%d, minor=%d\n", imajor(inode), iminor(inode));

	return ret;
}

static const struct file_operations tas2552_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.poll = tas2552_dev_poll,
	.read = tas2552_dev_read,
	.write = tas2552_dev_write,
	.open = tas2552_dev_open,
	.unlocked_ioctl = tas2552_dev_unlocked_ioctl
};

static int tas2552_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret;
	unsigned int en_gpio = 1;

	if (client->dev.of_node)
		en_gpio = of_get_named_gpio(client->dev.of_node,
					"qcom,en-gpio", 0);

	if ((!gpio_is_valid(en_gpio)))
		return -EINVAL;

	ret = gpio_request(en_gpio, "tas2552_en_gpio");
	if (ret) {
		pr_err("%s: gpio_request failed for tas2552_en_gpio(ret %d)\n",
			__func__, ret);
		return -EINVAL;
	}
	gpio_direction_output(en_gpio, 0);


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	tas2552_dev = devm_kzalloc(&client->dev,
				sizeof(struct tas2552), GFP_KERNEL);
	if (tas2552_dev == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	tas2552_dev->client = client;
	tas2552_dev->en_gpio = en_gpio;

	i2c_set_clientdata(client, tas2552_dev);

	tas2552_dev->tas2552_device.minor = MISC_DYNAMIC_MINOR;
	tas2552_dev->tas2552_device.name = "tas2552";
	tas2552_dev->tas2552_device.fops = &tas2552_dev_fops;

	init_waitqueue_head(&tas2552_dev->read_wq);
	mutex_init(&tas2552_dev->read_mutex);

	ret = misc_register(&tas2552_dev->tas2552_device);
	if (ret) {
		dev_err(&client->dev, "misc_register failed\n");
		return ret;
	}

	tas2552_hard_enable(1);
	tas2552_init();
	tas2552_soft_enable(0);

	dev_info(&client->dev,
		 "%s, probing tas2552 driver exited successfully\n",
		 __func__);

	return 0;
}

static int tas2552_remove(struct i2c_client *client)
{
	struct tas2552_dev *tas2552_dev;

	tas2552_dev = i2c_get_clientdata(client);
	kfree(tas2552_dev);

	return 0;
}

static struct of_device_id msm_match_table[] = {
	{.compatible = "ti,tas2552-i2c-codec"},
	{}
};

static const struct i2c_device_id tas2552_id[] = {
	{"tas2552-i2c", 0},
	{}
};

static struct i2c_driver tas2552_driver = {
	.id_table = tas2552_id,
	.probe = tas2552_probe,
	.remove = tas2552_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tas2552-i2c",
		.of_match_table = msm_match_table,
	},
};

/*
 * module load/unload record keeping
 */

static int __init tas2552_dev_init(void)
{
	return i2c_add_driver(&tas2552_driver);
}
module_init(tas2552_dev_init);

static void __exit tas2552_dev_exit(void)
{
	i2c_del_driver(&tas2552_driver);
}
module_exit(tas2552_dev_exit);

MODULE_AUTHOR("Hisense");
MODULE_DESCRIPTION("TI tas2552 driver");
MODULE_LICENSE("GPL");
