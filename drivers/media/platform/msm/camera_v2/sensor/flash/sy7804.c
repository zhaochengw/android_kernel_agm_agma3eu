/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/kobject.h>


#include <linux/kernel.h>
#include <linux/platform_device.h>


#include "msm_cci.h"

#include "msm_camera_io_util.h"
#include "msm_flash.h"

#define SY7804_FLASH_NAME "sg,sy7804"

//#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define SY7804_DBG(fmt, args...) pr_err("%s: "fmt, __func__, ##args)
#else
#define SY7804_DBG(fmt, args...)
#endif

#define SY7804_ERR(fmt, args...) pr_err("%s:%d "fmt, __func__, __LINE__, ##args)

#define FLASH_REAR  0
#define FLASH_FRONT 1



static struct msm_flash_ctrl_t sy7804_fctrl;
static struct i2c_driver sy7804_i2c_driver;

static struct device * sy7804_dev = NULL;

enum msm_sy7804_init_id {
	SY7804_ID_ENABLE = 0,
	SY7804_ID_IVFM,
	SY7804_ID_TRT,
	SY7804_ID_FRT,
	SY7804_ID_CURRENT,
};

static const int torch_current_unit = 4687;/*46.87 mA*/
static const int flash_current_unit = 9375;/*93.75 mA*/
static const int flash_timeout_unit = 100; /*100 ms*/
static const int flash_max_current = 1500; /*1.5A*/
static const int torch_max_current = 375; /*375mA*/
static const int flash_max_timeout = 800; /*800ms*/

static int sy7804_power_self = 0;


static struct msm_camera_i2c_reg_array sy7804_init_array[] = {
	{0x0A, 0x80},/* bit[7]IVFM, [6]TXI, [5]Flash_pin, [4]Torch_pin, 
				      [1:0]  00:Shutdown  01: Indicator 10:Torch mode 11: Flash mode*/
	{0x01, 0x8C},/* bit[7]UVLO enable,  [4,3,2] IVM-Down threshold=3.2V*/
	{0x06, 0x00},/* Torch Ramp-up time= 16ms,  Torch Ramp-Down time=16ms */
	{0x08, 0x15},/* Flash Ramp time=1ms, Flash timeout=600ms*/
	{0x09, 0x39},/* Torch cur=187.5mA, Flash cur=937.5mA*/
};

static struct msm_camera_i2c_reg_array sy7804_off_array[] = {
	{0x0A, 0x80},
};

static struct msm_camera_i2c_reg_array sy7804_release_array[] = {
	{0x0A, 0x80},
};

static struct msm_camera_i2c_reg_array sy7804_low_array[] = {
	{0x0A, 0x92},
};

static struct msm_camera_i2c_reg_array sy7804_high_array[] = {
	{0x0A, 0x83},
};


static struct msm_camera_i2c_reg_array sy7804_status_array[] = {
	{0x0B, 0x00},/*Read only*/
};




static void msm_sy7804_power_on(
	struct msm_flash_ctrl_t *fctrl)
{
//	int rc;
	struct msm_camera_gpio_conf *gpio_conf = NULL; 

	SY7804_DBG(" E. \n");

	if(!fctrl){
		return;
	}

	gpio_conf = fctrl->power_info.gpio_conf;
	if(!gpio_conf){
		return;
	}

#if 0	
	rc = msm_camera_request_gpio_table(
		gpio_conf->cam_gpio_req_tbl,
		gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0){		
		SY7804_ERR(" requst gpios fail! \n");
		//return;
	}	
#endif

	if (gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_EN]){
		SY7804_DBG(" gpio=%d set 1\n", 
			gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN]);		
		
		gpio_set_value_cansleep(
			gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN],
			1);
	}

#if 0	

	rc = msm_camera_request_gpio_table(
		gpio_conf->cam_gpio_req_tbl,
		gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0){		
		SY7804_ERR(" requst gpios fail! \n");
		//return;
	}
#endif
	
	SY7804_DBG(" X. \n");
	
}


static void msm_sy7804_power_down(
	struct msm_flash_ctrl_t *fctrl)
{
//	int rc;
	struct msm_camera_gpio_conf *gpio_conf = NULL; 

	SY7804_DBG(" E. \n");

	if(!fctrl){
		return;
	}

	gpio_conf = fctrl->power_info.gpio_conf;
	if(! gpio_conf){
		return;
	}

#if 0	
	rc = msm_camera_request_gpio_table(
		gpio_conf->cam_gpio_req_tbl,
		gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0){		
		SY7804_ERR(" requst gpios fail! \n");
		//return;
	}
#endif

	if (gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_EN]){
		SY7804_DBG(" gpio=%d set 0\n", 
			gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN]);
		
		gpio_set_value_cansleep(
			gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN],
			0);
	}
#if 0
		rc = msm_camera_request_gpio_table(
			gpio_conf->cam_gpio_req_tbl,
			gpio_conf->cam_gpio_req_tbl_size, 0);		
	}
#endif	
	SY7804_DBG(" X. \n");
	
}

#if 0
static struct led_classdev msm_sy7804_flash = {
	.name			= "flash-sy7804",
	.brightness_set	= msm_sy7804_brightness_set,
	.brightness		= LED_OFF,
};
#endif

static int32_t msm_sy7804_set_flash_max_current(struct msm_flash_ctrl_t *fctrl, int cur, int id)
{
	if(!fctrl){
		SY7804_ERR(" fail!\n");
		return -EINVAL;
	}
	
	if(id >= MAX_LED_TRIGGERS){
		SY7804_ERR("id %d >= MAX_LED_TRIGGERS %d fail!\n", id, MAX_LED_TRIGGERS);
		return -EINVAL;
	}

	if(cur > flash_max_current){
		SY7804_ERR("target %d > max %d fail!\n", cur, flash_max_current);
		cur = flash_max_current;
	}

	fctrl->flash_max_current[id] = cur;	
	
	SY7804_DBG("flash_max_current[%d] = %d\n", id, cur);

	return 0;
}



static int32_t msm_sy7804_set_flash_op_current(struct msm_flash_ctrl_t *fctrl, int cur, int id)
{
	uint32_t reg;
	uint16_t data;
	
	if(!fctrl){
		SY7804_ERR(" fail!\n");
		return -EINVAL;
	}
	
	if(id >= MAX_LED_TRIGGERS){
		SY7804_ERR("id %d >= MAX_LED_TRIGGERS %d fail!\n", id, MAX_LED_TRIGGERS);
		return -EINVAL;
	}

	if(cur > fctrl->flash_max_current[id]){
		SY7804_ERR("flash_op_current %d > max %d! -> %d\n", cur, 
			fctrl->flash_max_current[id],
			fctrl->flash_max_current[id]);		
		cur = fctrl->flash_max_current[id];
	}

	fctrl->flash_op_current[id] = cur;	

	cur = cur * 100;
	if(cur < flash_current_unit){
		reg = 0x00;
	}else{
		reg = (cur - flash_current_unit) / flash_current_unit;
		reg  = (reg > 0x0F) ? 0x0F : reg;
	}
	
	data = sy7804_init_array[SY7804_ID_CURRENT].reg_data;
	data = ((data & 0xF0) | reg);
	
	sy7804_init_array[SY7804_ID_CURRENT].reg_data = data;
	
	SY7804_DBG("flash_op_current[%d]=%d->0x%02x reg[0x%02x, 0x%02x]\n",
		id, cur, reg,
		sy7804_init_array[SY7804_ID_CURRENT].reg_addr,
		sy7804_init_array[SY7804_ID_CURRENT].reg_data);

	return 0;

}

static int32_t msm_sy7804_set_flash_max_timeout(struct msm_flash_ctrl_t *fctrl, int time, int id)
{
	if(!fctrl){
		SY7804_ERR(" fail!\n");
		return -EINVAL;
	}
	
	if(id >= MAX_LED_TRIGGERS){
		SY7804_ERR("id %d >= MAX_LED_TRIGGERS %d fail!\n", id, MAX_LED_TRIGGERS);
		return -EINVAL;
	}

	if(time > flash_max_timeout){
		SY7804_ERR("target %d > max %d fail!\n", time, flash_max_timeout);
		time = flash_max_timeout;
	}

	fctrl->flash_max_timeout[id] = time;	
	SY7804_DBG("flash_max_timeout[%d] = %d\n", id, time);

	return 0;
}


static int32_t msm_sy7804_set_flash_op_timeout(struct msm_flash_ctrl_t *fctrl, int time, int id)
{
	uint32_t reg;
	uint16_t data;
	
	if(!fctrl){
		SY7804_ERR(" fail!\n");
		return -EINVAL;
	}
	
	if(id >= MAX_LED_TRIGGERS){
		SY7804_ERR("id %d >= MAX_LED_TRIGGERS %d fail!\n", id, MAX_LED_TRIGGERS);
		return -EINVAL;
	}

	if(time > fctrl->flash_max_timeout[id]){
		SY7804_ERR("flash_op_timeout %d > max %d! -> %d\n", time, 
			fctrl->flash_max_timeout[id],
			fctrl->flash_max_timeout[id]);		
		time = fctrl->flash_max_timeout[id];
	}

	fctrl->flash_op_timeout[id] = time;

	if(time < flash_timeout_unit){
		reg = time;
	}else{
		reg = (time - flash_timeout_unit) / flash_timeout_unit;
		reg  = (reg > 0x07) ? 0x07 : reg;
	}	
	
	data = sy7804_init_array[SY7804_ID_FRT].reg_data;
	data = ((data & 0xf8) | reg);
	
	sy7804_init_array[SY7804_ID_FRT].reg_data = data;
	
	SY7804_DBG("flash_op_timeout[%d]=%d reg[0x%02x, 0x%02x]\n",
		id, time,
		sy7804_init_array[SY7804_ID_FRT].reg_addr,
		sy7804_init_array[SY7804_ID_FRT].reg_data);

	return 0;

}


static int32_t msm_sy7804_set_torch_max_current(struct msm_flash_ctrl_t *fctrl, int cur, int id)
{
	if(!fctrl){
		SY7804_ERR(" fail!\n");
		return -EINVAL;
	}
	
	if(id >= MAX_LED_TRIGGERS){
		SY7804_ERR("id %d >= MAX_LED_TRIGGERS %d fail!\n", id, MAX_LED_TRIGGERS);
		return -EINVAL;
	}

	if(cur > torch_max_current){
		SY7804_ERR("target %d > max %d fail!\n", cur, torch_max_current);
		cur = torch_max_current;
	}

	fctrl->torch_max_current[id] = cur;		
	SY7804_DBG("torch_max_current[%d] = %d\n", id, cur);

	return 0;
}

static int32_t msm_sy7804_set_torch_op_current(struct msm_flash_ctrl_t *fctrl, int cur, int id)
{
	uint32_t reg;
	uint16_t data;
	
	if(!fctrl){
		SY7804_ERR(" fail!\n");
		return -EINVAL;
	}
	
	if(id >= MAX_LED_TRIGGERS){
		SY7804_ERR("id %d >= MAX_LED_TRIGGERS %d fail!\n", id, MAX_LED_TRIGGERS);
		return -EINVAL;
	}

	if(cur > fctrl->torch_max_current[id]){
		SY7804_ERR("torch_op_current %d > max %d! -> %d\n", cur, 
			fctrl->torch_max_current[id],
			fctrl->torch_max_current[id]);		
		cur = fctrl->torch_max_current[id];
	}

	fctrl->torch_op_current[id] = cur;	

	cur = cur * 100;
	if (cur < torch_current_unit){
		reg = 0x00;
	}else{
		reg = (cur - torch_current_unit) / torch_current_unit;	
		reg  = (reg > 0x07) ? 0x07 : reg;
	}
	
	data = sy7804_init_array[SY7804_ID_CURRENT].reg_data;
	data = ((data & 0x8f) | (reg << 4));
	
	sy7804_init_array[SY7804_ID_CURRENT].reg_data = data;
	
	SY7804_DBG("torch_op_current[%d]=%d reg[0x%02x, 0x%02x]\n",	id, cur,
		sy7804_init_array[SY7804_ID_CURRENT].reg_addr,
		sy7804_init_array[SY7804_ID_CURRENT].reg_data);

	return 0;

}

static int32_t msm_sy7804_clear_status(struct msm_flash_ctrl_t *fctrl)
{
	int rc = 0;
	int i = 0;
	struct msm_camera_i2c_client  *client = NULL;
	struct msm_camera_i2c_reg_setting *status = NULL;
	
	if (!fctrl) {
		SY7804_ERR(" fail!\n");
		return -EINVAL;
	}

	client = &fctrl->flash_i2c_client;
	
	if (fctrl->reg_setting && fctrl->reg_setting->clear_status) {
		status = fctrl->reg_setting->clear_status;
		for (i = 0; i < status->size; i++){
			
			SY7804_DBG("clear status[%d][0x%x, 0x%x] master=%d sid=0x%x cid=0x%x freq=%d\n", i, 
				status->reg_setting[i].reg_addr,
				status->reg_setting[i].reg_data,
				client->cci_client->cci_i2c_master,
				client->cci_client->sid,
				client->cci_client->cid,
				client->cci_client->freq);
			
			rc = client->i2c_func_tbl->i2c_read(client,
						status->reg_setting[i].reg_addr, 
						&status->reg_setting[i].reg_data,
						MSM_CAMERA_I2C_BYTE_DATA);		
			if (rc < 0){
				SY7804_ERR(" i2c_read failed rc=%d\n", rc);
				return -EIO;
			}
		}
	}
	
	return rc;	
}



static uint16_t msm_sy7804_i2c_read(struct msm_flash_ctrl_t *fctrl, uint16_t addr)
{
	int rc = 0;
	uint16_t data = 0;
	struct msm_camera_i2c_client  *client = NULL;

	client = &fctrl->flash_i2c_client;
	
	rc = client->i2c_func_tbl->i2c_read(client, 
		addr, &data, MSM_CAMERA_I2C_BYTE_DATA);		
	if (rc < 0){
		SY7804_ERR(" i2c_read failed rc=%d\n", rc);
		return 0xFF;
	}
	SY7804_DBG(" [0x%02x, 0x%02x]\n", addr, data);
	return data;	
}


int msm_flash_sy7804_init(struct msm_flash_ctrl_t *fctrl, 
	struct msm_flash_cfg_data_t *cfg_data)
{
	int rc = 0;	
	int leds_num = 0;
	int i = 0;	
	struct msm_camera_i2c_client  *client = NULL;
	struct msm_camera_i2c_reg_setting * setting = NULL;
	struct msm_camera_gpio_conf *gpio_conf = NULL;
	
	SY7804_DBG(" E.\n");

	if(!fctrl){
		SY7804_ERR(" fail!\n");
		return -EINVAL;
	}
	gpio_conf = fctrl->power_info.gpio_conf;

	fctrl->led_state = MSM_CAMERA_LED_RELEASE;	
	client = &fctrl->flash_i2c_client;
	
	client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	
	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = client->i2c_func_tbl->i2c_util(client, MSM_CCI_INIT);
		if (rc < 0) {
			SY7804_ERR("cci_init failed rc=%d\n", rc);
			return rc;
		}
	}	
	
	leds_num = fctrl->flash_num_sources;

	msm_sy7804_clear_status(fctrl);
	
	for(i = 0; i < leds_num; i++){
		msm_sy7804_set_flash_op_current(fctrl, fctrl->flash_op_current[i], i);
		msm_sy7804_set_flash_op_timeout(fctrl, fctrl->flash_op_timeout[i], i);
		msm_sy7804_set_torch_op_current(fctrl, fctrl->torch_op_current[i], i);
	}
	
	if ((gpio_conf != NULL) && gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_NOW]){
		gpio_request_one(gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
				0, "flash-now");
		SY7804_DBG(" gpio=%d set 0\n",gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW]);
		gpio_set_value_cansleep(
			gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_LOW);
	}

	if (fctrl->reg_setting) {
		setting = fctrl->reg_setting->init_setting;

		for(i = 0; i < setting->size; i++){
			SY7804_DBG("write [%d][0x%02x 0x%02x] ", i, (setting->reg_setting + i)->reg_addr, (setting->reg_setting + i)->reg_data);
			rc = client->i2c_func_tbl->i2c_write(client,
				(setting->reg_setting + i)->reg_addr,
				(setting->reg_setting + i)->reg_data,
				MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0){
				SY7804_ERR(" [%d][0x%02x 0x%02x] write failed rc=%d \n",\
				i, (setting->reg_setting + i)->reg_addr, (setting->reg_setting + i)->reg_data, rc);
			}
		}
	}

	fctrl->led_state = MSM_CAMERA_LED_INIT;
	SY7804_DBG(" X.\n");
	
	return rc;
}

int msm_flash_sy7804_release(struct msm_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_i2c_client  *client = NULL;
	struct msm_camera_gpio_conf *gpio_conf = NULL;

	SY7804_DBG(" E.\n");

	if (!fctrl && (MSM_CAMERA_LED_INIT != fctrl->led_state)) {
		SY7804_ERR("fail.\n");
		return -EINVAL;
	}

	client = &fctrl->flash_i2c_client;

	gpio_conf = fctrl->power_info.gpio_conf;

	if (fctrl->reg_setting) {
		rc = client->i2c_func_tbl->i2c_write_table(client,
			fctrl->reg_setting->release_setting);
		if (rc < 0)
			SY7804_ERR(" failed rc=%d\n", rc);
	}

	msm_sy7804_clear_status(fctrl);

	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = client->i2c_func_tbl->i2c_util(client, MSM_CCI_RELEASE);
		if (rc < 0) {
			SY7804_ERR("cci_release failed rc=%d\n", rc);
		}
	}	
	if ((gpio_conf != NULL) && gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_NOW]){
		SY7804_DBG(" gpio=%d set 0\n",gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW]);
		gpio_set_value_cansleep(
			gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_LOW);
		gpio_free(gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW]);
	}
	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	
	SY7804_DBG(" X.\n");
	return 0;
}

int msm_flash_sy7804_off(struct msm_flash_ctrl_t *fctrl,
	struct msm_flash_cfg_data_t *cfg_data)
{
	int rc = 0;
	struct msm_camera_i2c_client  *client = NULL;
	struct msm_camera_gpio_conf *gpio_conf = NULL;

	SY7804_DBG(" E.\n");
	
	if (!fctrl && (MSM_CAMERA_LED_INIT != fctrl->led_state)) {
		SY7804_ERR("fail.\n");
		return -EINVAL;
	}

	client = &fctrl->flash_i2c_client;
	gpio_conf = fctrl->power_info.gpio_conf;

	if ((gpio_conf != NULL) && gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_NOW]){

		SY7804_DBG(" gpio=%d set 0\n",
			gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW]);
		gpio_set_value_cansleep(
			gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_LOW);
	}

	if (fctrl->reg_setting) {
		rc = client->i2c_func_tbl->i2c_write_table(client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			SY7804_ERR(" failed rc=%d\n", rc);
	}

	msm_sy7804_clear_status(fctrl);
	
	SY7804_DBG(" X.\n");
	return rc;
}

int msm_flash_sy7804_low(struct msm_flash_ctrl_t *fctrl,
	struct msm_flash_cfg_data_t *cfg_data)
{
	int rc = 0;
	struct msm_camera_i2c_client  *client = NULL;
	struct msm_camera_gpio_conf *gpio_conf = NULL;
	SY7804_DBG(" E.\n");
	
	if (!fctrl && (MSM_CAMERA_LED_INIT != fctrl->led_state)) {
		SY7804_ERR("fail.\n");
		return -EINVAL;
	}

	if(!fctrl->enable){
		SY7804_ERR(" disabled!\n");
		return 0;
	}
	gpio_conf = fctrl->power_info.gpio_conf;
	client = &fctrl->flash_i2c_client;
	if ((gpio_conf != NULL) && gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_NOW]){
		SY7804_DBG(" gpio=%d set 1\n",
			gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW]);
		gpio_set_value_cansleep(
			gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_HIGH);
	}
	if (fctrl->reg_setting) {
		rc = client->i2c_func_tbl->i2c_write_table(client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			SY7804_ERR(" failed rc=%d\n", rc);
	}

	msm_sy7804_clear_status(fctrl);
	
	SY7804_DBG(" X.\n");

	return rc;
}

int msm_flash_sy7804_high(struct msm_flash_ctrl_t *fctrl,
	struct msm_flash_cfg_data_t *cfg_data)
{
	int rc = 0;
	struct msm_camera_i2c_client  *client = NULL;

	SY7804_DBG(" E.\n");
	
	if (!fctrl && (MSM_CAMERA_LED_INIT != fctrl->led_state)) {
		SY7804_ERR("fail.\n");
		return -EINVAL;
	}
	
	if(!fctrl->enable){
		SY7804_ERR(" disabled!\n");
		return 0;
	}
	
	client = &fctrl->flash_i2c_client;

	if (fctrl->reg_setting) {
		rc = client->i2c_func_tbl->i2c_write_table(client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			SY7804_ERR(" failed rc=%d\n", rc);
	}
	
	msm_sy7804_clear_status(fctrl);
	
	SY7804_DBG(" X.\n");
	return rc;
}


#if 0
int32_t msm_flash_sy7804_config(struct msm_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	int i = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
	SY7804_DBG("E. cmd = %d\n", cfg->cfgtype);

	if (!fctrl->func_tbl) {
		SY7804_ERR("fail.\n");
		return -EINVAL;
	}
	switch (cfg->cfgtype) {

	case MSM_CAMERA_LED_INIT:
		SY7804_DBG("MSM_CAMERA_LED_INIT:");
		
		if (fctrl->func_tbl->flash_led_init)
			rc = fctrl->func_tbl->flash_led_init(fctrl);
		
		for (i = 0; i < MAX_LED_TRIGGERS; i++) {
			SY7804_DBG("flash_current[%d] = %d\n", i, cfg->flash_current[i]);
			SY7804_DBG("torch_current[%d] = %d\n", i, cfg->torch_current[i]);
			SY7804_DBG("flash_duration[%d] = %d\n", i, cfg->flash_duration[i]);
		}
		break;

	case MSM_CAMERA_LED_RELEASE:
		SY7804_DBG("MSM_CAMERA_LED_RELEASE:");
		if (fctrl->func_tbl->flash_led_release)
			rc = fctrl->func_tbl->flash_led_release(fctrl);
		break;

	case MSM_CAMERA_LED_OFF:		
		SY7804_DBG("MSM_CAMERA_LED_OFF:");
		if (fctrl->func_tbl->flash_led_off)
			rc = fctrl->func_tbl->flash_led_off(fctrl);
		break;

	case MSM_CAMERA_LED_LOW:		
		SY7804_DBG("MSM_CAMERA_LED_LOW:");
		if (fctrl->func_tbl->flash_led_low)
			rc = fctrl->func_tbl->flash_led_low(fctrl);
		break;

	case MSM_CAMERA_LED_HIGH:
		SY7804_DBG("MSM_CAMERA_LED_HIGH:");
		if (fctrl->func_tbl->flash_led_high)
			rc = fctrl->func_tbl->flash_led_high(fctrl);
		break;
		
	default:
		rc = -EFAULT;
		break;
	}
	
	SY7804_DBG(" X. cmd=%d return %d\n", cfg->cfgtype, rc);
	return rc;
}
#endif

static ssize_t attr_get_flash_enable(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;
	
	led_num = fctrl->flash_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->enable);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7804_DBG(" %s", buf);

	return offset + len;		
}

static ssize_t attr_set_flash_enable(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned int enable;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;
	
	if (kstrtouint(buf, 10, &enable))
		return -EINVAL;
	
	fctrl->enable = enable;
	SY7804_DBG(" enable = %d", enable);
	return size;
}

static ssize_t attr_get_flash_max_current(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;
	
	led_num = fctrl->flash_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->flash_max_current[i]);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7804_DBG(" %s", buf);

	return offset + len;		
}


static ssize_t attr_set_flash_max_current(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned int cur;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;

	if (kstrtouint(buf, 10, &cur))
		return -EINVAL;
	
	SY7804_DBG(" cur=%d\n", cur);
		
	msm_sy7804_set_flash_max_current(fctrl, cur, 0);

	return size;
}



static ssize_t attr_get_flash_op_current(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;
	
	led_num = fctrl->flash_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->flash_op_current[i]);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7804_DBG(" %s", buf);

	return offset + len;		
}


static ssize_t attr_set_flash_op_current(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned int cur;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;

	if (kstrtouint(buf, 10, &cur))
		return -EINVAL;
	
	SY7804_DBG(" cur=%d\n", cur);
		
	msm_sy7804_set_flash_op_current(fctrl, cur, 0);

	return size;
}


static ssize_t attr_get_flash_max_timeout(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;
	
	led_num = fctrl->flash_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->flash_max_timeout[i]);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7804_DBG(" %s", buf);

	return offset + len;		
}


static ssize_t attr_set_flash_max_timeout(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int time;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;

	if (kstrtoint(buf, 10, &time))
		return -EINVAL;
	
	SY7804_DBG(" time=%d\n", time);
		
	msm_sy7804_set_flash_max_timeout(fctrl, time, 0);

	return size;
}



static ssize_t attr_get_flash_op_timeout(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;
	
	led_num = fctrl->flash_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->flash_op_timeout[i]);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7804_DBG(" %s", buf);

	return offset + len;		
}


static ssize_t attr_set_flash_op_timeout(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int time;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;

	if (kstrtoint(buf, 10, &time))
		return -EINVAL;
	
	SY7804_DBG(" time=%d\n", time);
		
	msm_sy7804_set_flash_op_timeout(fctrl, time, 0);

	return size;
}


static ssize_t attr_get_torch_max_current(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;
	
	led_num = fctrl->flash_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->torch_max_current[i]);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7804_DBG(" %s", buf);

	return offset + len;		
}


static ssize_t attr_set_torch_max_current(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned int cur;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;

	if (kstrtouint(buf, 10, &cur))
		return -EINVAL;
	
	SY7804_DBG(" cur=%d\n", cur);
		
	msm_sy7804_set_torch_max_current(fctrl, cur, 0);

	return size;
}


static ssize_t attr_get_torch_op_current(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;
	
	led_num = fctrl->flash_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->torch_op_current[i]);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7804_DBG(" %s", buf);

	return offset + len;		
}


static ssize_t attr_set_torch_op_current(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned int cur;
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;

	if (kstrtouint(buf, 10, &cur))
		return -EINVAL;
	
	SY7804_DBG(" cur=%d\n", cur);
		
	msm_sy7804_set_torch_op_current(fctrl, cur, 0);

	return size;
}

static ssize_t attr_set_torch_test(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int rc;
	unsigned int enable;
	
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;

	if (kstrtouint(buf, 10, &enable))
		return size;
	
	SY7804_DBG(" enable=%d\n", enable);

	if(enable){
		
		if(MSM_CAMERA_LED_INIT != fctrl->led_state){

			sy7804_power_self = 1;
			msm_sy7804_power_on(fctrl);
			msleep(10);
			
			rc = msm_flash_sy7804_init(fctrl, NULL);
			if(rc < 0){
				SY7804_ERR(" sy7804_init fail!\n");
				goto clean;
			}
		}
		
		msm_flash_sy7804_low(fctrl, NULL);
		
	}else{
		msm_flash_sy7804_off(fctrl, NULL);
		goto clean;
	}

	return size;
	
clean:
	if(sy7804_power_self){
		msm_flash_sy7804_release(fctrl);			
		msm_sy7804_power_down(fctrl);
		sy7804_power_self = 0;			
	}

	return size;
}



static ssize_t attr_set_flash_test(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int rc;
	unsigned int times;
	
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;

	if (kstrtouint(buf, 10, &times))
		return size;
	
	SY7804_DBG(" times=%d\n", times);

	if(times > 0){
		if(MSM_CAMERA_LED_INIT != fctrl->led_state){
			sy7804_power_self = 1;
			msm_sy7804_power_on(fctrl);
			msleep(10);
			
			rc = msm_flash_sy7804_init(fctrl, NULL);
			if(rc < 0){
				SY7804_ERR(" sy7804_init fail!\n");
				goto clean;
			}
		}
		
		do{
			SY7804_ERR(" flash times left %d\n", times);
			msm_flash_sy7804_high(fctrl, NULL);
			msleep(1500);			
			msm_flash_sy7804_off(fctrl, NULL);
			msleep(500);			
		}while(times--);

	}
	
clean:
	if(sy7804_power_self){
		msm_flash_sy7804_release(fctrl);			
		msm_sy7804_power_down(fctrl);
		sy7804_power_self = 0;			
	}
	return size;
}


static ssize_t attr_dump_regs(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i;
	int offset = 0;
	int len;
	uint16_t data;

	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;
	struct msm_camera_i2c_reg_setting * setting = NULL;


	setting = fctrl->reg_setting->init_setting;
	if(setting){
		for(i = 0; i < setting->size; i++){		
			data = msm_sy7804_i2c_read(fctrl, (setting->reg_setting + i)->reg_addr);			
			len = sprintf(buf + offset, "init[0x%02x 0x%02x][0x%02x]\n", 
				(setting->reg_setting + i)->reg_addr,
				(setting->reg_setting + i)->reg_data, data);
			offset += len;
		}
	}

	setting = fctrl->reg_setting->low_setting;
	if(setting){
		for(i = 0; i < setting->size; i++){			
			data = msm_sy7804_i2c_read(fctrl, (setting->reg_setting + i)->reg_addr);			
			len = sprintf(buf + offset, "torch[0x%02x 0x%02x][0x%02x]\n", 
				(setting->reg_setting + i)->reg_addr,
				(setting->reg_setting + i)->reg_data, data);
			offset += len;
		}
	}

	setting = fctrl->reg_setting->high_setting;
	if(setting){
		for(i = 0; i < setting->size; i++){			
			data = msm_sy7804_i2c_read(fctrl, (setting->reg_setting + i)->reg_addr);			
			len = sprintf(buf + offset, "flash[0x%02x 0x%02x][0x%02x]\n", 
				(setting->reg_setting + i)->reg_addr,
				(setting->reg_setting + i)->reg_data, data);
			offset += len;
		}
	}

	setting = fctrl->reg_setting->off_setting;
	if(setting){
		for(i = 0; i < setting->size; i++){		
			data = msm_sy7804_i2c_read(fctrl, (setting->reg_setting + i)->reg_addr);
			len = sprintf(buf + offset, "off[0x%02x 0x%02x][0x%02x]\n", 
				(setting->reg_setting + i)->reg_addr,
				(setting->reg_setting + i)->reg_data, data);
			offset += len;
		}
	}

	SY7804_DBG(" %s", buf);

	return offset;
}

static struct device_attribute attributes[] = {	
	__ATTR(enable, 0664, attr_get_flash_enable, attr_set_flash_enable),
	__ATTR(flash_max_current, 0664, attr_get_flash_max_current, attr_set_flash_max_current),
	__ATTR(flash_op_current, 0664, attr_get_flash_op_current, attr_set_flash_op_current),
	__ATTR(flash_max_timeout, 0664, attr_get_flash_max_timeout, attr_set_flash_max_timeout),
	__ATTR(flash_op_timeout, 0664, attr_get_flash_op_timeout, attr_set_flash_op_timeout),
	__ATTR(torch_max_current, 0664, attr_get_torch_max_current, attr_set_torch_max_current),
	__ATTR(torch_op_current, 0664, attr_get_torch_op_current, attr_set_torch_op_current),
	__ATTR(test_torch, 0222, NULL, attr_set_torch_test),
	__ATTR(test_flash, 0222, NULL, attr_set_flash_test),	
	__ATTR(dump, 0444, attr_dump_regs, NULL),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;

	sy7804_dev = dev;
	
	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		err = device_create_file(dev, attributes + i);
		if (err)
			goto error;
	}
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return err;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}


static const struct of_device_id sy7804_dt_match[] = {
	{.compatible = "sg,sy7804", .data = &sy7804_fctrl},
	{}
};

static int32_t msm_sy7804_create_classdev(struct device *dev ,
				void *data)
{
	int rc;	
	struct msm_flash_ctrl_t *fctrl = &sy7804_fctrl;
	
	if (!fctrl) {
		pr_err("Invalid fctrl\n");
		return -EINVAL;
	}
    SY7804_DBG("%s E\n",__func__);
	if (fctrl->front_flash_node == false) {
        fctrl->led_cl_dev.name = "flashlight";
        fctrl->led_cl_dev.brightness_set = msm_flash_i2c_rear_brightness_set;
        fctrl->led_cl_dev.position = FLASH_REAR;
        rc = led_classdev_register(dev, &fctrl->led_cl_dev);
        msm_flash_i2c_rear_brightness_set(&fctrl->led_cl_dev,LED_OFF);
    } else {
        fctrl->led_cl_dev.name = "front_flashlight";
        fctrl->led_cl_dev.brightness_set = msm_flash_i2c_front_brightness_set;
        fctrl->led_cl_dev.position = FLASH_FRONT;
        rc = led_classdev_register(dev, &fctrl->led_cl_dev);
        msm_flash_i2c_front_brightness_set(&fctrl->led_cl_dev,LED_OFF);
    }
    
	create_sysfs_interfaces(fctrl->led_cl_dev.dev);
    
    SY7804_DBG("%s X\n",__func__);
	return 0;
};


static int msm_flash_sy7804_platform_probe(struct platform_device *pdev)
{
	int rc = 0 ;	
	const struct of_device_id *match;

	SY7804_DBG("E. \n");
	
	match = of_match_device(sy7804_dt_match, &pdev->dev);
	rc = msm_flash_platform_probe(pdev, match->data);

	SY7804_DBG("X. rc=%d\n", rc);

	if (!rc){		
		msm_sy7804_create_classdev(&pdev->dev, NULL);
	}
	
	
	return rc;
}



MODULE_DEVICE_TABLE(of, sy7804_dt_match);

#if 0

static int32_t msm_flash_sy7804_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_flash_i2c_probe(client, id);
}


static int msm_flash_sy7804_i2c_remove(struct i2c_client *client)
{
	int rc = 0 ;
	SY7804_DBG("%s entry\n", __func__);
	return rc;
}



static const struct i2c_device_id sy7804_i2c_id[] = {
	{SY7804_FLASH_NAME, (kernel_ulong_t)&sy7804_fctrl},
	{ }
};


static struct i2c_driver sy7804_i2c_driver = {
	.id_table = sy7804_i2c_id,
	.probe  = msm_flash_sy7804_i2c_probe,
	.remove = msm_flash_sy7804_i2c_remove,
	.driver = {
		.name = SY7804_FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sy7804_dt_match,
	},
};
static struct msm_camera_i2c_client sy7804_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

#endif




static struct platform_driver sy7804_platform_driver = {
	.driver = {
		.name = SY7804_FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sy7804_dt_match,
	},
	.probe = msm_flash_sy7804_platform_probe,
};

static int __init msm_sy7804_init(void)
{

	int32_t rc;
	SY7804_DBG("E. \n");
	rc = platform_driver_register(&sy7804_platform_driver);
	SY7804_DBG("X. platform_driver_register rc=%d\n", rc);

	if(rc){
		rc = i2c_add_driver(&sy7804_i2c_driver);
		SY7804_DBG("X. i2c_add_driver rc=%d\n", rc);		
	}
	
	return rc;
}

static void __exit msm_sy7804_exit(void)
{
	SY7804_DBG("E. \n");

	if(sy7804_dev){
		remove_sysfs_interfaces(sy7804_dev);
	}
	
	SY7804_DBG("X. \n");	
	return;
}

static struct msm_camera_i2c_reg_setting sy7804_init_setting = {
	.reg_setting = sy7804_init_array,
	.size = ARRAY_SIZE(sy7804_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7804_off_setting = {
	.reg_setting = sy7804_off_array,
	.size = ARRAY_SIZE(sy7804_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7804_release_setting = {
	.reg_setting = sy7804_release_array,
	.size = ARRAY_SIZE(sy7804_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7804_low_setting = {
	.reg_setting = sy7804_low_array,
	.size = ARRAY_SIZE(sy7804_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7804_high_setting = {
	.reg_setting = sy7804_high_array,
	.size = ARRAY_SIZE(sy7804_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7804_clear_status = {
	.reg_setting = sy7804_status_array,
	.size = ARRAY_SIZE(sy7804_status_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};


static struct msm_flash_reg_t sy7804_regs = {
	.init_setting = &sy7804_init_setting,
	.off_setting = &sy7804_off_setting,
	.low_setting = &sy7804_low_setting,
	.high_setting = &sy7804_high_setting,
	.release_setting = &sy7804_release_setting,
	.clear_status = &sy7804_clear_status,
};

static struct msm_flash_func_t sy7804_func_tbl = {
	.camera_flash_init = msm_flash_sy7804_init,
	.camera_flash_release = msm_flash_sy7804_release,
	.camera_flash_off = msm_flash_sy7804_off,
	.camera_flash_low = msm_flash_sy7804_low,
	.camera_flash_high = msm_flash_sy7804_high,
	.camera_flash_clear_status = msm_sy7804_clear_status,
	.camera_flash_power_on = msm_sy7804_power_on,
    .camera_flash_power_down = msm_sy7804_power_down,
};

static struct msm_flash_ctrl_t sy7804_fctrl = {
	.flash_name = SY7804_FLASH_NAME,
	.flash_i2c_client = {
		.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	},
	.reg_setting = &sy7804_regs,
	.func_tbl = &sy7804_func_tbl,
	.enable = 1,
	.flash_driver_type = FLASH_DRIVER_I2C,
};

module_init(msm_sy7804_init);
module_exit(msm_sy7804_exit);
MODULE_DESCRIPTION("sy7804 FLASH");
MODULE_LICENSE("GPL v2");
