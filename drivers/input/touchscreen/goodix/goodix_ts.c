/* drivers/input/touchscreen/gt9xx.c
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
 * Version: 2.2
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2014/01/14
 * Revision record:
 *      V1.0:   
 *          first Release. By Andrew, 2012/08/31 
 *      V1.2:
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F. By Andrew, 2012/10/15
 *      V1.4:
 *          modify gt9xx_update.c. By Andrew, 2012/12/12
 *      V1.6: 
 *          1. new heartbeat/esd_protect mechanism(add external watchdog)
 *          2. doze mode, sliding wakeup 
 *          3. 3 more cfg_group(GT9 Sensor_ID: 0~5) 
 *          3. config length verification
 *          4. names & comments
 *                  By Meta, 2013/03/11
 *      V1.8:
 *          1. pen/stylus identification 
 *          2. read double check & fixed config support
 *          3. new esd & slide wakeup optimization
 *                  By Meta, 2013/06/08
 *      V2.0:
 *          1. compatible with GT9XXF
 *          2. send config after resume
 *                  By Meta, 2013/08/06
 *      V2.2:
 *          1. gt9xx_config for debug
 *          2. gesture wakeup
 *          3. pen separate input device, active-pen button support
 *          4. coordinates & keys optimization
 *                  By Meta, 2014/01/14
 */

#include <linux/irq.h>
#include "goodix_ts.h"
#include <linux/input/mt.h>
//#include <linux/productinfo.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
#include <linux/buffer_head.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/genhd.h>
#include <linux/device.h>
#include <linux/slab.h>
#endif
#include <linux/hisresume.h>

static const char *goodix_ts_name = "goodix-ts";
static struct workqueue_struct *goodix_wq;
struct i2c_client * i2c_connect_client = NULL; 
extern s32 gt9xx_short_test(struct i2c_client * client);
extern s32 gt9xx_open_test(struct i2c_client * client);

#if GTP_DEBUG_ON
    static const int  key_codes[] = {KEY_HOME, KEY_BACK, KEY_MENU, KEY_SEARCH};
    static const char *key_names[] = {"Key_Home", "Key_Back", "Key_Menu", "Key_Search"};
#endif

static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
s32 gtp_send_cfg(struct i2c_client *client);
void gtp_int_sync(struct goodix_ts_data * ts, s32 ms);
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
extern s32 gup_i2c_read(struct i2c_client *client, u8 *buf, s32 len);

static ssize_t gt91xx_config_read_proc(struct file *, char __user *, size_t, loff_t *);
static ssize_t gt91xx_config_write_proc(struct file *, const char __user *,size_t , loff_t *);
static int goodix_ts_pinctrl_init(struct goodix_ts_data *ts);
static int goodix_ts_pinctrl_select(struct goodix_ts_data *ts, bool on);

static struct proc_dir_entry *gt91xx_config_proc = NULL;
static const struct file_operations config_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xx_config_read_proc,
    .write = gt91xx_config_write_proc,
};


/**
 * goodix_power_on - Turn device power ON
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_on(struct goodix_ts_data *ts);
static int goodix_power_off(struct goodix_ts_data *ts);
static int goodix_power_init(struct goodix_ts_data *ts);
static int goodix_power_deinit(struct goodix_ts_data *ts);



#if defined(CONFIG_FB)
static void goodix_ts_resume_work(struct work_struct *work);
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

extern s32 init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);

static struct delayed_work gtp_esd_check_work;
//static struct delayed_work gtp_productinfo_work;

static struct workqueue_struct *gtp_esd_check_workqueue = NULL;
static struct workqueue_struct *gtp_productinfo_workqueue = NULL;

static void gtp_esd_check_func(struct work_struct *);
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
void gtp_esd_switch(struct i2c_client *, s32);

extern int factory_get_fs_fw_version(struct device *dev, char *buf);
extern int factory_check_fw_update_need(struct device *dev);
extern int factory_get_fw_update_progress(struct device *dev);
extern int factory_proc_fw_update(struct device *dev, bool force);
extern int factory_proc_fw_bin_update(struct device * dev, const char *buf);

//*********** For GT9XXF Start **********//
#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
extern s32 i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 gup_clk_calibration(void);
extern s32 gup_fw_download_proc(void *dir, u8 dwn_mode);
extern u8 gup_check_fs_mounted(char *path_name);

void gtp_recovery_reset(struct i2c_client *client);
static s32 gtp_esd_recovery(struct i2c_client *client);
s32 gtp_fw_startup(struct i2c_client *client);
static s32 gtp_main_clk_proc(struct goodix_ts_data *ts);
static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode);
static int diag_write(unsigned int offset, char *buf, unsigned int size);
static struct block_device* diag_partition_open( void );
static void diag_partition_close( struct block_device *bdev);
static int diag_partition_read(struct block_device *bdev, unsigned int where, char *pBuffer, unsigned int size);
static int diag_partition_write(struct block_device *bdev, unsigned int where, char *pBuffer, unsigned int size);
static int gtp_write_clk_to_diag(char *buf, unsigned int size);
static void gtp_recovery_reset_func(struct work_struct *work);


static struct workqueue_struct *gtp_main_clk_workqueue = NULL;
static struct workqueue_struct *gtp_recovery_reset_workqueue = NULL;

static struct delayed_work gtp_main_clk_work;
static struct work_struct  gtp_recovery_reset_work;


#define ABOOT_PART_LENGTH       32*1024
#define ABOOT_READ_LENGTH       1*PAGE_SIZE
static char gtp_main_clock[20];
u8 p_main_clk[6] = {0};
#endif
//********** For GT9XXF End **********//

#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
typedef enum
{
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
}DOZE_T;

static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);
#endif
static int debug_mask = 0;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR);

#undef dev_info
#define dev_info(dev, format, arg...) do { if (debug_mask) { dev_printk(KERN_INFO , dev , format , ## arg); }} while(0)

extern ssize_t gtp_get_rawdatainfo(struct i2c_client* client, char* buf);
extern ssize_t gtp_get_rawdata(struct i2c_client* client, char* buf);


static int factory_get_rawdata(struct device *dev, char *buf)
{

	struct goodix_ts_data *ts = dev_get_drvdata(dev);

	return gtp_get_rawdata(ts->client, buf);

}

static int factory_get_rawdata_info(struct device *dev, char *buf)
{

	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	
	return gtp_get_rawdatainfo(ts->client, buf);

}

static int factory_proc_hibernate_test(struct device *dev)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	s32 ret = -1;
	u8 buf[3] = {0};
	buf[0] = (u8)(GTP_REG_CONFIG_DATA >> 8);
	buf[1] = (u8)(GTP_REG_CONFIG_DATA & 0xFF);
	ts->gtp_reset_mode = 1;

	disable_irq(ts->client->irq);

    ts->gtp_is_suspend = 1;

	gpio_direction_output(ts->pdata->reset_gpio, 0);
	mdelay(10);
	ret = gup_i2c_read(ts->client, buf, GTP_ADDR_LENGTH+1);
	ts->gtp_reset_mode = 0;		//don't chage the place,it will use in reset_guitar 
	if(ret >= 0){
		GTP_ERROR("IC reset pin is low,but i2c work.");
		ret = FAIL;
		goto hibernate_exit;
	}
	mdelay(10); 

	gtp_reset_guitar(ts->client, 10);

	ret = gup_i2c_read(ts->client, buf, GTP_ADDR_LENGTH+1);
	if(ret < 0){
		GTP_ERROR("IC reset pin is high,but i2c don't work.");
		ret = FAIL;
		goto hibernate_exit;
	}
	ret = SUCCESS;
	printk("%s end of reset test.\n",__func__);
hibernate_exit:
	gtp_send_cfg(ts->client);
	mdelay(50);
	enable_irq(ts->client->irq);

    ts->gtp_is_suspend = 0;
	
#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
	queue_work(gtp_recovery_reset_workqueue,&gtp_recovery_reset_work);
#endif
	
	return ret;
}

static int factory_get_ic_fw_version(struct device *dev, char *buf)
{	
	struct goodix_ts_data *data = dev_get_drvdata(dev);
	u8 opr_buf[16] = {0};
	s32 ret = -1;
	ret = gtp_i2c_read_dbl_check(data->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);
        
    if (ret == FAIL)
    {
    	GTP_ERROR("[factory_get_ic_fw_version]get ic_config_version failed,exit");
        return FAIL;		
    }
	data->ic_config_version = opr_buf[0];
	
	ret = gtp_i2c_read_dbl_check(data->client, 0x8140, &opr_buf[GTP_ADDR_LENGTH], 6);
    if (FAIL == ret)
    {
        GTP_ERROR("[factory_get_ic_fw_version]get pid & vid failed,exit");
        return FAIL;
    }
	data->ic_version = opr_buf[GTP_ADDR_LENGTH+4] + (opr_buf[GTP_ADDR_LENGTH+5]<<8);
	return sprintf(buf, "0x%04X-%d\n", data->ic_version,data->ic_config_version);

}

static int factory_get_module_id(struct device *dev, char *buf)
{
	struct goodix_ts_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "0x%02X\n", data->sensor_id);
}

static int factory_get_calibration_ret(struct device *dev)
{
	return 1;
}

#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
static unsigned int asic_to_hex(unsigned char val)
{
	if((val >= '0') && (val <= '9')){
		val -= '0';
		}
	else if((val >= 'a') && (val <= 'z')){
		val = val - 'a' + 10;
		}
	else if((val >= 'A') && (val <= 'Z')){
		val = val - 'A' + 10;
		}
	return (unsigned int)val;
}

static int set_gesture_switch(struct device *dev, const char *buf)
{
	struct goodix_ts_data *data = dev_get_drvdata(dev);
	unsigned char gesture[10], len;
	
	strlcpy(gesture, buf, sizeof(gesture));
	len = strlen(gesture);
	if (len > 0) {
		if((gesture[len-1] == '\n') || (gesture[len-1] == '\0')){
			len--;
		}
	}
	//printk("%s len: %d gtp_state: %d,%d,%d.\n",__func__,len,gesture[0],gesture[1],gesture[2]);
	if(len == 1){
		if (gesture[0] == '1') 
			data->gesture_state = 0xffff;	//cmd =1, it will open all gesture.
		else if (gesture[0] == '0') 
			data->gesture_state = 0x0;		//cmd =0, it will close all gesture.
	} else if(len == 4) {
		data->gesture_state = asic_to_hex(gesture[0])*0x1000 
						+ asic_to_hex(gesture[1]) * 0x100
						+ asic_to_hex(gesture[2]) * 0x10
						+ asic_to_hex(gesture[3]);
	} else {
		GTP_ERROR("[set_gesture_switch]write wrong cmd.");
		return 0;
	}
	if(!data->gesture_state)
		data->gesture_wakeup_enable = false;
	else
		data->gesture_wakeup_enable = true;
	
	printk("GTP %s is %x.\n",__func__,data->gesture_state);
 	return 0;
}

static bool get_gesture_switch(struct device *dev)
{
	struct goodix_ts_data *data = dev_get_drvdata(dev);

	return data->gesture_wakeup_enable;
}
#endif

static int factory_short_test(struct device *dev, char *buf)
{
	struct goodix_ts_data *data = dev_get_drvdata(dev);
    s32 ret = 3;

	ret = gt9xx_short_test(data->client);

	return sprintf(buf,"%d\n",ret);
}

static int factory_ts_func_test_register(struct goodix_ts_data* data)
{
	data->ts_test_dev.dev = &data->client->dev;
	data->ts_test_dev.get_calibration_ret = factory_get_calibration_ret;
	data->ts_test_dev.get_fs_fw_version = factory_get_fs_fw_version;
	data->ts_test_dev.get_ic_fw_version = factory_get_ic_fw_version;
	data->ts_test_dev.get_module_id = factory_get_module_id;
	data->ts_test_dev.get_rawdata = factory_get_rawdata;
	data->ts_test_dev.get_rawdata_info = factory_get_rawdata_info;	
	data->ts_test_dev.proc_hibernate_test = factory_proc_hibernate_test;
	
	
	
#if !defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
	data->ts_test_dev.check_fw_update_need = factory_check_fw_update_need;
	data->ts_test_dev.get_fw_update_progress = factory_get_fw_update_progress;
	data->ts_test_dev.proc_fw_update = factory_proc_fw_update;
	data->ts_test_dev.proc_fw_update_with_given_file = factory_proc_fw_bin_update;
#endif

#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
	data->ts_test_dev.get_gesture_switch = get_gesture_switch;
	data->ts_test_dev.set_gesture_switch = set_gesture_switch;
#endif

	data->ts_test_dev.get_short_test= factory_short_test;

	register_ts_func_test_device(&data->ts_test_dev);
	return 0;
}

#if 0
void gt9xx_ts_register_productinfo(struct goodix_ts_data* ts_data)
{
    // format as flow: version:0x01 Module id:0x57
    char deviceinfo[64];
	u8 opr_buf[16] = {0};
	s32 ret = -1;
	
	ret = gtp_i2c_read_dbl_check(ts_data->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);
	if (FAIL == ret)
	{
        GTP_ERROR("[gt9xx_ts_register_productinfo]get ic_config_version failed,exit");
        return;
	}
	ts_data->ic_config_version = opr_buf[0];
	
	ret = gtp_i2c_read_dbl_check(ts_data->client, 0x8140, &opr_buf[GTP_ADDR_LENGTH], 6);
    if (FAIL == ret)
    {
        GTP_ERROR("[gt9xx_ts_register_productinfo]get pid & vid failed,exit");
        return;
    }
	ts_data->ic_version = opr_buf[GTP_ADDR_LENGTH+4] + (opr_buf[GTP_ADDR_LENGTH+5]<<8);
		
	sprintf(deviceinfo, "FW version:0x%04X-%d Module id:0x%2x",
                                            ts_data->ic_version,ts_data->ic_config_version,ts_data->sensor_id);

   	productinfo_register(PRODUCTINFO_CTP_ID, NULL, deviceinfo);
}

static void goodix_register_productinfo_func(struct work_struct *work)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	gt9xx_ts_register_productinfo(ts);
}
#endif

/*******************************************************
Function:
    Read data from the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   read start address.
    buf[2~len-1]:   read data buffer.
    len:    GTP_ADDR_LENGTH + read bytes count
Output:
    numbers of i2c_msgs to transfer: 
      2: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.
    
    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if((retries >= 5))
    {
    #if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
        struct goodix_ts_data *ts = i2c_get_clientdata(client);
    #endif
	
	#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
        // reset chip would quit doze mode
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    #endif
        GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
	#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
        if (CHIP_TYPE_GT9F == ts->chip_type)
        { 
            gtp_recovery_reset(client);
        }
        else
    #endif
        {
            gtp_reset_guitar(client, 10);  
        }
    }
    return ret;
}



/*******************************************************
Function:
    Write data to the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   write start address.
    buf[2~len-1]:   data buffer
    len:    GTP_ADDR_LENGTH + write bytes count
Output:
    numbers of i2c_msgs to transfer: 
        1: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
    //msg.scl_rate = 300 * 1000;    // for Rockchip, etc

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
    #if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
        struct goodix_ts_data *ts = i2c_get_clientdata(client);
    #endif
	
	#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    #endif
        GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
	 #if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
        if (CHIP_TYPE_GT9F == ts->chip_type)
        { 
            gtp_recovery_reset(client);
        }
        else
    #endif
        {
            gtp_reset_guitar(client, 10);  
        }
    }
    return ret;
}


/*******************************************************
Function:
    i2c read twice, compare the results
Input:
    client:  i2c device
    addr:    operate address
    rxbuf:   read data to store, if compare successful
    len:     bytes to read
Output:
    FAIL:    read failed
    SUCCESS: read successful
*********************************************************/
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    u8 retry = 0;
    
    while (retry++ < 3)
    {
        memset(buf, 0xAA, 16);
        buf[0] = (u8)(addr >> 8);
        buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, buf, len + 2);
        
        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (u8)(addr >> 8);
        confirm_buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, confirm_buf, len + 2);
        
        if (!memcmp(buf, confirm_buf, len+2))
        {
            memcpy(rxbuf, confirm_buf+2, len);
            return SUCCESS;
        }
    }    
    GTP_ERROR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
    return FAIL;
}

/*******************************************************
Function:
    Send config.
Input:
    client: i2c device.
Output:
    result of i2c write operation. 
        1: succeed, otherwise: failed
*********************************************************/

s32 gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 2;
    s32 retry = 0;
    struct goodix_ts_data *ts = i2c_get_clientdata(client);
    if (ts->pnl_init_error)
    {
        GTP_ERROR("Error occured in init_panel, no config sent");
        return 0;
    }
    if (ts->fw_error)
    {
        GTP_ERROR("Error occured in update, no config sent");
        return 0;
    }
	dev_info(&ts->client->dev,"Driver send config.\n");
    for (retry = 0; retry < 5; retry++)
    {        
                
        ret = gtp_i2c_write(client, ts->config_data , ts->gtp_cfg_len);
   
        if (ret > 0)
        {
            break;
        }
    }
    return ret;
}
/*******************************************************
Function:
    Disable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
    unsigned long irqflags;

    GTP_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1; 
        disable_irq_nosync(ts->client->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Enable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
    unsigned long irqflags = 0;

    GTP_DEBUG_FUNC();
    
    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disable) 
    {
        enable_irq(ts->client->irq);
        ts->irq_is_disable = 0; 
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


/*******************************************************
Function:
    Report touch point event 
Input:
    ts: goodix i2c_client private data
    id: trackId
    x:  input x coordinate
    y:  input y coordinate
    w:  input pressure
Output:
    None.
*********************************************************/
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
	if(ts->pdata->enable_slot_report){
	    input_mt_slot(ts->input_dev, id);
	    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	}
	else{
	    input_report_key(ts->input_dev, BTN_TOUCH, 1);
	    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	    input_mt_sync(ts->input_dev);
	}
    GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
    Report touch release event
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
	if(ts->pdata->enable_slot_report)
	{
	    input_mt_slot(ts->input_dev, id);
	    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
	    GTP_DEBUG("Touch id[%2d] release!", id);
	}
	else
    	input_report_key(ts->input_dev, BTN_TOUCH, 0);
}

/*******************************************************
Function:
    Goodix touchscreen work function
Input:
    work: work struct of goodix_workqueue
Output:
    None.
*********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u16 pre_touch = 0;
    static u8 pre_key = 0;
    u8  key_value = 0;
    u8* coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;
	
	
#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
		u8 rqst_buf[3] = {0x80, 0x43};	// for GT9XXF
#endif

#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
    u8 doze_buf[3] = {0x81, 0x4B};
	u8 gesture_data[6] = {0x81, 0x4D};
	u8 gesture_vale = 0;
#endif

    GTP_DEBUG_FUNC();
    ts = container_of(work, struct goodix_ts_data, work);
    if (ts->enter_update)
    {
        return;
    }
#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
    if (DOZE_ENABLED == doze_status)
    {               
        ret = gtp_i2c_read(i2c_connect_client, doze_buf, 3);
		if (ret < 0){
			GTP_ERROR("%s read gesture cmd error",__func__);
			goto gesture_exit;
		}
        
    	gesture_vale = doze_buf[2];
    	doze_status = DOZE_WAKEUP;
		if(doze_buf[2] == 0){//appear after suspend,we ignore it
			goto gesture_exit;
		}
		printk("%s Wakeup by gesture (%d).\n",__func__,doze_buf[2]);
		if (0xCC == gesture_vale)//double clilk
        {
            ret = gtp_i2c_read(i2c_connect_client, gesture_data, 6);
            GTP_DEBUG("gesture_data[2] = 0x%02X, gesture_data[3] = 0x%02X,\n gesture_data[4] = 0x%02X, gesture_data[5] = 0x%02X", gesture_data[2], gesture_data[3], gesture_data[4], gesture_data[5]);	
            if (((gesture_data[2]==1) || (gesture_data[2]==2) || (gesture_data[2]==4))&&(gesture_data[3]==0)&&(gesture_data[4]==0)&&(gesture_data[5]==0))//814d is 1,2,4  814e &814f are 0
			{
	        	GTP_INFO("Double click by key area, ignore light up the screen!");
				goto gesture_exit;
			}
		}
		
    	for (i = 0; i < ts->pdata->gesture_key->nbuttons; i++)
        {
           if (gesture_vale == ts->pdata->gesture_key->map[i])
           		break;
        }
		if(i >= ts->pdata->gesture_key->nbuttons)
			GTP_ERROR("%s: have no gesture to report.",__func__);
		else{
			if((ts->gesture_state) >> (i+1) & 1) {
				resumeinfo_start(S_A_DB_CLICK_ID);
				input_report_key(ts->input_dev, ts->pdata->gesture_map->map[i], 1); 
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, ts->pdata->gesture_map->map[i], 0); 
				input_sync(ts->input_dev);
			}else { 
				GTP_ERROR("%s: not open the gesture switch.",__func__);
			}
        }
gesture_exit:
		// clear 0x814B
		doze_buf[2] = 0x00;
		gtp_i2c_write(i2c_connect_client, doze_buf, 3);
		gtp_enter_doze(ts);
        if (ts->use_irq)
        {
            gtp_irq_enable(ts);
        }
        return;
    }
#endif

    ret = gtp_i2c_read(ts->client, point_data, 12);
    if (ret < 0)
    {
        GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
        if (ts->use_irq)
        {
            gtp_irq_enable(ts);
        }
        return;
    }
    
    finger = point_data[GTP_ADDR_LENGTH];
#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
		// GT9XXF
		if ((finger == 0x00) && (CHIP_TYPE_GT9F == ts->chip_type))	   // request arrived
		{
			ret = gtp_i2c_read(ts->client, rqst_buf, 3);
			if (ret < 0)
			{
			   GTP_ERROR("Read request status error!");
			   goto exit_work_func;
			} 
			
			switch (rqst_buf[2])
			{
			case GTP_RQST_CONFIG:
				dev_info(&ts->client->dev,"Request for config.\n");
				ret = gtp_send_cfg(ts->client);
				if (ret < 0)
				{
					GTP_ERROR("Request for config unresponded!");
				}
				else
				{
					rqst_buf[2] = GTP_RQST_RESPONDED;
					gtp_i2c_write(ts->client, rqst_buf, 3);
					dev_info(&ts->client->dev,"Request for config responded!\n");
				}
				break;
				
			case GTP_RQST_BAK_REF:
				dev_info(&ts->client->dev,"Request for backup reference.\n");
				ts->rqst_processing = 1;
				ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_SEND);
				if (SUCCESS == ret)
				{
					rqst_buf[2] = GTP_RQST_RESPONDED;
					gtp_i2c_write(ts->client, rqst_buf, 3);
					ts->rqst_processing = 0;
					dev_info(&ts->client->dev,"Request for backup reference responded!\n");
				}
				else
				{
					GTP_ERROR("Requeset for backup reference unresponed!");
				}
				break;
				
			case GTP_RQST_RESET:
				dev_info(&ts->client->dev,"Request for reset.\n");
				gtp_recovery_reset(ts->client);
				break;
				
			case GTP_RQST_MAIN_CLOCK:
				dev_info(&ts->client->dev,"Request for main clock.\n");
				ts->rqst_processing = 1;
				ret = gtp_main_clk_proc(ts);
				if (FAIL == ret)
				{
					GTP_ERROR("Request for main clock unresponded!");
				}
				else
				{
					dev_info(&ts->client->dev,"Request for main clock responded!\n");
					rqst_buf[2] = GTP_RQST_RESPONDED;
					gtp_i2c_write(ts->client, rqst_buf, 3);
					ts->rqst_processing = 0;
					ts->clk_chk_fs_times = 0;
				}
				break;
				
			default:
				dev_info(&ts->client->dev,"Undefined request: 0x%02X\n", rqst_buf[2]);
				rqst_buf[2] = GTP_RQST_RESPONDED;  
				gtp_i2c_write(ts->client, rqst_buf, 3);
				break;
			}
		}
#endif

    if (finger == 0x00)
    {
        if (ts->use_irq)
        {
            gtp_irq_enable(ts);
        }
        return;
    }

    if((finger & 0x80) == 0)
    {
        goto exit_work_func;
    }

    touch_num = finger & 0x0f;
    if (touch_num > GTP_MAX_TOUCH)
    {
        goto exit_work_func;
    }

    if (touch_num > 1)
    {
        u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

        ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1)); 
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }

    key_value = point_data[3 + 8 * touch_num];
    
    if(key_value || pre_key)
    {
        if (!pre_touch)
        {
            for (i = 0; i < ts->pdata->button_map->nbuttons; i++)
            {
            #if GTP_DEBUG_ON
                for (ret = 0; ret < ts->pdata->button_map->nbuttons; ++ret)
                {
                    if (key_codes[ret] == ts->pdata->button_map->map[i])
                    {
                        GTP_DEBUG("Key: %s %s", key_names[ret], (key_value & (0x01 << i)) ? "Down" : "Up");
                        break;
                    }
                }
            #endif
                input_report_key(ts->input_dev, ts->pdata->button_map->map[i], key_value & (0x01<<i));   
            }
            touch_num = 0;  // shield fingers
        }
    }

    pre_key = key_value;

	dev_info(&ts->client->dev, "%s:pre_touch:%02x, finger:%02x.\n", __func__,pre_touch, finger);

	if(ts->pdata->enable_slot_report)
	{
	    if (pre_touch || touch_num)
	    {
	        s32 pos = 0;
	        u16 touch_index = 0;
	        u8 report_num = 0;
	        coor_data = &point_data[3];
	        
	        if(touch_num)
	        {
	            id = coor_data[pos] & 0x0F;        
	            touch_index |= (0x01<<id);
	        }
	        
	        GTP_DEBUG("id = %d,touch_index = 0x%x, pre_touch = 0x%x\n",id, touch_index,pre_touch);
	        for (i = 0; i < GTP_MAX_TOUCH; i++)
	        {        
	            if ((touch_index & (0x01<<i)))
	            {
	                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
	                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
	                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);

	                gtp_touch_down(ts, id, input_x, input_y, input_w);
	                pre_touch |= 0x01 << i;
	                
	                report_num++;
	                if (report_num < touch_num)
	                {
	                    pos += 8;
	                    id = coor_data[pos] & 0x0F;
	                    touch_index |= (0x01<<id);
	                }
	            }
	            else
	            {
	                gtp_touch_up(ts, i);
	                pre_touch &= ~(0x01 << i);
	            }
	        }
	    }
	}

	else
	{
	    if (touch_num)
	    {
	        for (i = 0; i < touch_num; i++)
	        {
	            coor_data = &point_data[i * 8 + 3];

	            id = coor_data[0] & 0x0F;
	            input_x  = coor_data[1] | (coor_data[2] << 8);
	            input_y  = coor_data[3] | (coor_data[4] << 8);
	            input_w  = coor_data[5] | (coor_data[6] << 8);
	            {
	                gtp_touch_down(ts, id, input_x, input_y, input_w);
					dev_info(&ts->client->dev, "%s: [x, y]=[%d, %d]\n",__func__, input_x, input_y);
	            }
	        }
	    }
	    else if (pre_touch)
	    {
	        {
	            GTP_DEBUG("Touch Release!");
	            gtp_touch_up(ts, 0);
	        }
	    }

    	pre_touch = touch_num;
	}
    
    input_sync(ts->input_dev);

exit_work_func:
    if(!ts->gtp_rawdiff_mode)
    {
        ret = gtp_i2c_write(ts->client, end_cmd, 3);
        if (ret < 0)
        {
            GTP_INFO("I2C write end_cmd error!");
        }
    }
    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
}

/*******************************************************
Function:
    Timer interrupt service routine for polling mode.
Input:
    timer: timer struct pointer
Output:
    Timer work mode. 
        HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
    struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

    GTP_DEBUG_FUNC();

    queue_work(goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

/*******************************************************
Function:
    External interrupt service routine for interrupt mode.
Input:
    irq:  interrupt number.
    dev_id: private data pointer
Output:
    Handle Result.
        IRQ_HANDLED: interrupt handled successfully
*********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
    struct goodix_ts_data *ts = dev_id;

    GTP_DEBUG_FUNC();

	dev_info(&ts->client->dev,"goodix_ts_irq_handler.\n");
 
    gtp_irq_disable(ts);

    queue_work(goodix_wq, &ts->work);
    
    return IRQ_HANDLED;
}
/*******************************************************
Function:
    Synchronization.
Input:
    ms: synchronization time in millisecond.
Output:
    None.
*******************************************************/
void gtp_int_sync(struct goodix_ts_data * ts, s32 ms)
{
    gpio_direction_output(ts->pdata->irq_gpio, 0);
    msleep(ms);
    gpio_direction_input(ts->pdata->irq_gpio);
}


/*******************************************************
Function:
    Reset chip.
Input:
    ms: reset time in millisecond
Output:
    None.
*******************************************************/
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if(ts->gtp_reset_mode)
	{
		GTP_ERROR("IC in resettest,don't reset.");
		return ;
	}
    
	dev_info(&ts->client->dev,"Guitar reset\n");
    /* This reset sequence will selcet I2C slave address */
    gpio_direction_output(ts->pdata->reset_gpio, 0);
    mdelay(ms);

    if (ts->client->addr == 0x14)
        gpio_direction_output(ts->pdata->irq_gpio, 1);
    else
        gpio_direction_output(ts->pdata->irq_gpio, 0);

    mdelay(RESET_DELAY_T3);
    gpio_direction_output(ts->pdata->reset_gpio, 1);
    mdelay(RESET_DELAY_T4);

    gpio_direction_input(ts->pdata->reset_gpio);
#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        return;
    }
#endif	
    gtp_int_sync(ts, 50);  

	if(ts->pdata->enable_esd)
	{
    	gtp_init_ext_watchdog(client);
	}
}

#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
/*******************************************************
Function:
    Enter doze mode for sliding wakeup.
Input:
    ts: goodix tp private data
Output:
    1: succeed, otherwise failed
*******************************************************/
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

    GTP_DEBUG_FUNC();

    GTP_DEBUG("Entering gesture mode.");
    while(retry++ < 5)
    {
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x46;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret < 0)
        {
            GTP_DEBUG("failed to set doze flag into 0x8046, %d", retry);
            continue;
        }
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x40;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            doze_status = DOZE_ENABLED;
            GTP_DEBUG("Gesture mode enabled.");
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send gesture cmd failed.");
    return ret;
}
#endif

/*******************************************************
Function:
    Enter sleep mode.
Input:
    ts: private data.
Output:
    Executive outcomes.
       1: succeed, otherwise failed.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};
	
#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
    u8 status_buf[3] = {0x80, 0x44};
#endif    

    GTP_DEBUG_FUNC();

#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
		if (CHIP_TYPE_GT9F == ts->chip_type)
		{
			// GT9XXF: host interact with ic
			ret = gtp_i2c_read(ts->client, status_buf, 3);
			if (ret < 0)
			{
				GTP_ERROR("failed to get backup-reference status");
			}
			
			if (status_buf[2] & 0x80)
			{
				ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_STORE);
				if (FAIL == ret)
				{
					GTP_ERROR("failed to store bak_ref");
				}
			}
		}
#endif

    if (ts->pdata->enable_power_off) {
		
		ret = goodix_power_off(ts);
		if (ret) {
			dev_err(&ts->client->dev, "GTP power off failed.\n");
			return ret;
		}
		return 0;
    }
    
    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    msleep(5);
    
    while(retry++ < 5)
    {
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            GTP_INFO("GTP enter sleep!");
            
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send sleep cmd failed.");
    return ret;
}

/*******************************************************
Function:
    Wakeup from sleep.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data * ts)
{

    u8 retry = 0;
    s8 ret = -1;
	
    GTP_DEBUG_FUNC();
#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
		if (CHIP_TYPE_GT9F == ts->chip_type)
		{
			u8 opr_buf[3] = {0x41, 0x80};
			
			GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
			msleep(5);
		
			for (retry = 0; retry < 10; ++retry)
			{
				// hold ss51 & dsp
				opr_buf[2] = 0x0C;
				ret = gtp_i2c_write(ts->client, opr_buf, 3);
				if (FAIL == ret)
				{
					GTP_ERROR("failed to hold ss51 & dsp!");
					continue;
				}
				opr_buf[2] = 0x00;
				ret = gtp_i2c_read(ts->client, opr_buf, 3);
				if (FAIL == ret)
				{
					GTP_ERROR("failed to get ss51 & dsp status!");
					continue;
				}
				if (0x0C != opr_buf[2])
				{
					GTP_DEBUG("ss51 & dsp not been hold, %d", retry+1);
					continue;
				}
				GTP_DEBUG("ss51 & dsp confirmed hold");
				
				ret = gtp_fw_startup(ts->client);
				if (FAIL == ret)
				{
					GTP_ERROR("failed to startup GT9XXF, process recovery");
					gtp_esd_recovery(ts->client);
				}
				break;
			}
			if (retry >= 10)
			{
				GTP_ERROR("failed to wakeup, processing esd recovery");
				gtp_esd_recovery(ts->client);
			}
			else
			{
				GTP_INFO("GT9XXF gtp wakeup success");
			}
			return ret;
		}
#endif

    if (ts->pdata->enable_power_off) {
		ret = goodix_power_on(ts);
		if (ret) {
			dev_err(&ts->client->dev, "GTP power on failed.\n");
			return 0;
		}
    }

    while(retry++ < 3)
    {
#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
		printk("%s gesture_wakeup_enable = %d.\n",__func__,ts->gesture_wakeup_enable);
		if(ts->gesture_wakeup_enable_pre){
	        doze_status = DOZE_DISABLED;
	        gtp_irq_disable(ts);
	        gtp_reset_guitar(ts->client, 10);
	        gtp_irq_enable(ts);
		}
		else
#endif
		{
        	//GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
        	//msleep(5);
        	gtp_irq_disable(ts);
			gtp_reset_guitar(ts->client, 10);//need disable irq,or it appears i2c error
			gtp_irq_enable(ts);
		}
	
        ret = gtp_i2c_test(ts->client);
        if (ret > 0)
        {
            GTP_DEBUG("GTP wakeup sleep.");
            
#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
	    if(ts->gesture_wakeup_enable)
			return ret;
			else
#endif	
        	{
                gtp_int_sync(ts, 25);
            	if(ts->pdata->enable_esd)
                	gtp_init_ext_watchdog(ts->client);
				return ret;
         	}
        }
    }

    GTP_ERROR("GTP wakeup sleep failed.");
    return ret;
}

/*******************************************************
Function:
    Initialize gtp.
Input:
    ts: goodix private data
Output:
    Executive outcomes.
        0: succeed, otherwise: failed
*******************************************************/
s32 gtp_init_panel(struct goodix_ts_data *ts)
{
    s32 ret = -1;
    s32 i = 0;
    u8 check_sum = 0;
    u8 opr_buf[16] = {0};
    u8 sensor_id = 0;
    unsigned char *config;
	//u8 grp_cfg_version = 0;

    u8**send_cfg_buf =  ts->pdata->config_data;
    int  *cfg_info_len = ts->pdata->config_data_len;
    
    GTP_DEBUG_FUNC();
    GTP_DEBUG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d", 
        cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
        cfg_info_len[4], cfg_info_len[5]);

#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        ts->fw_error = 0;
    }
    else
#endif    
    {
        ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
        if (SUCCESS == ret) 
        {
            if (opr_buf[0] != 0xBE)
            {
                ts->fw_error = 1;
                GTP_ERROR("Firmware error, no config sent!");
                return -1;
            }
        }
    }

    if ((!cfg_info_len[1]) && (!cfg_info_len[2]) && 
        (!cfg_info_len[3]) && (!cfg_info_len[4]) && 
        (!cfg_info_len[5]))
    {
        sensor_id = 0; 
    }
    else
    {
    #if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
        msleep(50);
    #endif
        ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID, &sensor_id, 1);
        if (SUCCESS == ret)
        {
            if (sensor_id >= 0x06)
            {
                GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
                ts->pnl_init_error = 1;
                return -1;
            }
        }
        else
        {
            GTP_ERROR("Failed to get sensor_id, No config sent!");
            ts->pnl_init_error = 1;
            return -1;
        }
        GTP_INFO("Sensor_ID: %d", sensor_id);
    }
    ts->sensor_id =  sensor_id;
	if (ts->pdata->config_data[ts->sensor_id] == NULL){
		ts->pnl_init_error = 1;
		return -1;
	}
    ts->config_data = ts->pdata->config_data[sensor_id];
    ts->gtp_cfg_len = ts->pdata->config_data_len[sensor_id]+ GTP_ADDR_LENGTH;
    config = ts->config_data;
	ts->config_version = ts->config_data[2];
     
    GTP_DEBUG("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id , ts->gtp_cfg_len);
    
    if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH)
    {
        GTP_ERROR("Config Group%d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id+1, ts->gtp_cfg_len);
        ts->pnl_init_error = 1;
        return -1;
    }

    {
        ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);
        
        if (ret == SUCCESS)
        {
            GTP_DEBUG("CFG_GROUP%d Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X", sensor_id, 
                        send_cfg_buf[sensor_id][0], send_cfg_buf[sensor_id][0], opr_buf[0], opr_buf[0]);
            ts->ic_config_version = opr_buf[0];
        //grp_cfg_version = send_cfg_buf[sensor_id][GTP_ADDR_LENGTH];       // backup group config version
        //send_cfg_buf[sensor_id][GTP_ADDR_LENGTH] = 0x00;
        }
        else
        {
            GTP_ERROR("Failed to get ic config version!No config sent!");
            return -1;
        }
    }
    
    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len - GTP_ADDR_LENGTH; i++)
    {
        check_sum += config[i];
    }
    config[ts->gtp_cfg_len - GTP_ADDR_LENGTH] = (~check_sum) + 1;

    if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0))
    {
        ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
        ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
        ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03; 
    }
	
#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        u8 sensor_num = 0;
        u8 driver_num = 0;
        u8 have_key = 0;
        
        have_key = (config[GTP_REG_HAVE_KEY - GTP_REG_CONFIG_DATA + 2] & 0x01);
        
        if (1 == ts->is_950)
        {
            driver_num = config[GTP_REG_MATRIX_DRVNUM - GTP_REG_CONFIG_DATA + 2];
            sensor_num = config[GTP_REG_MATRIX_SENNUM - GTP_REG_CONFIG_DATA + 2];
            if (have_key)
            {
                driver_num--;
            }
            ts->bak_ref_len = (driver_num * (sensor_num - 1) + 2) * 2 * 6;
        }
        else
        {
            driver_num = (config[CFG_LOC_DRVA_NUM] & 0x1F) + (config[CFG_LOC_DRVB_NUM]&0x1F);
            if (have_key)
            {
                driver_num--;
            }
            sensor_num = (config[CFG_LOC_SENS_NUM] & 0x0F) + ((config[CFG_LOC_SENS_NUM] >> 4) & 0x0F);
            ts->bak_ref_len = (driver_num * (sensor_num - 2) + 2) * 2;
        }
    
        GTP_INFO("Drv * Sen: %d * %d(key: %d), X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x",
           driver_num, sensor_num, have_key, ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
        return 0;
    }
    else
#endif
    {
        ret = gtp_send_cfg(ts->client);
    	if (ret < 0)
    	{
        	GTP_ERROR("Send config error.");
    	}
		
    // set config version to CTP_CFG_GROUP, for resume to send config
    /*
	    config[GTP_ADDR_LENGTH] = grp_cfg_version;
	    check_sum = 0;
	    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len - GTP_ADDR_LENGTH; i++)
	    {
	        check_sum += config[i];
	    }
	    config[ts->gtp_cfg_len - GTP_ADDR_LENGTH] = (~check_sum) + 1;
	    GTP_DEBUG("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x", ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
    */
    }
    
    msleep(10);
    return 0;
}


static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
    char *ptr = page;
    char temp_data[GTP_CONFIG_MAX_LENGTH + 2] = {0x80, 0x47};
    int i;
    
    if (*ppos)
    {
        return 0;
    }
    ptr += sprintf(ptr, "==== GT9XX config init value====\n");

    for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", ts->config_data[i + 2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }

    ptr += sprintf(ptr, "\n");

    ptr += sprintf(ptr, "==== GT9XX config real value====\n");
    gtp_i2c_read(i2c_connect_client, temp_data, GTP_CONFIG_MAX_LENGTH + 2);
    for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", temp_data[i+2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }
    *ppos += ptr - page;
    return (ptr - page);
}

static ssize_t gt91xx_config_write_proc(struct file *filp, const char __user *buffer, size_t size, loff_t *off)
{
    s32 ret = 0;
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
    int count ;
    count = (int )size;
    GTP_DEBUG("write count %d\n", count);

    if (count > GTP_CONFIG_MAX_LENGTH)
    {
        GTP_ERROR("size not match [%d:%d]\n", GTP_CONFIG_MAX_LENGTH, count);
        return -EFAULT;
    }

    if (copy_from_user(&ts->config_data[2], buffer, count))
    {
        GTP_ERROR("copy from user fail\n");
        return -EFAULT;
    }

    ret = gtp_send_cfg(i2c_connect_client);

    if (ret < 0)
    {
        GTP_ERROR("send config failed.");
    }

    return size;
}
/*******************************************************
Function:
    Read chip version.
Input:
    client:  i2c device
    version: buffer to keep ic firmware version
Output:
    read operation return.
        2: succeed, otherwise: failed
*******************************************************/
s32 gtp_read_version(struct goodix_ts_data *ts, u16* version)
{
    struct i2c_client *client = ts->client;
    u16 ver= 0;
    s32 ret = -1;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed");
        return ret;
    }

    ver = (buf[7] << 8) | buf[6];
    ts->ic_version =  ver;
    
    if (version)
    {
        *version =ver;
    }
    if (buf[5] == 0x00)
    {
        GTP_INFO("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
    }
    else
    {
        GTP_INFO("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }    
    return ret;
}

/*******************************************************
Function:
    I2c test Function.
Input:
    client:i2c client.
Output:
    Executive outcomes.
        2: succeed, otherwise failed.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    u8 retry = 0;
    s8 ret = -1;
  
    GTP_DEBUG_FUNC();
  
    while(retry++ < 5)
    {
        ret = gtp_i2c_read(client, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        GTP_ERROR("GTP i2c test failed time %d.",retry);
        msleep(10);
    }
    return ret;
}

/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >= 0: succeed, < 0: failed
*******************************************************/
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
	struct i2c_client *client = ts->client;
	struct goodix_ts_platform_data *pdata = ts->pdata;
	int ret;

	if (gpio_is_valid(pdata->irq_gpio)) {
		ret = gpio_request(pdata->irq_gpio, "goodix_ts_irq_gpio");
		if (ret) {
			dev_err(&client->dev, "Unable to request irq gpio [%d]\n",
				pdata->irq_gpio);
			goto err_pwr_off;
		}
		ret = gpio_direction_input(pdata->irq_gpio);
		if (ret) {
			dev_err(&client->dev, "Unable to set direction for irq gpio [%d]\n",
				pdata->irq_gpio);
			goto err_free_irq_gpio;
		}
	} else {
		dev_err(&client->dev, "Invalid irq gpio [%d]!\n",
			pdata->irq_gpio);
		ret = -EINVAL;
		goto err_pwr_off;
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		ret = gpio_request(pdata->reset_gpio, "goodix_ts_reset_gpio");
		if (ret) {
			dev_err(&client->dev, "Unable to request reset gpio [%d]\n",
				pdata->reset_gpio);
			goto err_free_irq_gpio;
		}

		ret = gpio_direction_output(pdata->reset_gpio, 0);
		if (ret) {
			dev_err(&client->dev, "Unable to set direction for reset gpio [%d]\n",
				pdata->reset_gpio);
			goto err_free_reset_gpio;
		}
	} else {
		dev_err(&client->dev, "Invalid irq gpio [%d]!\n",
			pdata->reset_gpio);
		ret = -EINVAL;
		goto err_free_irq_gpio;
	}
	/* IRQ GPIO is an input signal, but we are setting it to output
	  * direction and pulling it down, to comply with power up timing
	  * requirements, mentioned in power up timing section of device
	  * datasheet.
	  */
	ret = gpio_direction_output(pdata->irq_gpio, 0);
	if (ret)
		dev_warn(&client->dev,
			"pull down interrupt gpio failed\n");
	ret = gpio_direction_output(pdata->reset_gpio, 0);
	if (ret)
		dev_warn(&client->dev,
			"pull down reset gpio failed\n");

	return ret;

err_free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
err_free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
err_pwr_off:
	return ret;
}

/*******************************************************
Function:
    Request interrupt.
Input:
    ts: private data.
Output:
    Executive outcomes.
        0: succeed, -1: failed.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
    s32 ret = -1;
    const u8 irq_table[] = GTP_IRQ_TAB;

    GTP_DEBUG_FUNC();
    GTP_DEBUG("INT trigger type:%x", ts->int_trigger_type);

    ret  = request_irq(ts->client->irq, 
                       goodix_ts_irq_handler,
                       irq_table[ts->int_trigger_type],
                       ts->client->name,
                       ts);
    if (ret)
    {
        GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
        GTP_GPIO_AS_INPUT(GTP_INT_PORT);
        GTP_GPIO_FREE(GTP_INT_PORT);

        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = goodix_ts_timer_handler;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        return -1;
    }
    else 
    {
        gtp_irq_disable(ts);
        ts->use_irq = 1;
        return 0;
    }
}

/*******************************************************
Function:
    Request input device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
        0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 phys[32];
    u8 index = 0;  
    GTP_DEBUG_FUNC();
  
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
    {
        GTP_ERROR("Failed to allocate input device.");
        return -ENOMEM;
    }

    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;

	if(ts->pdata->enable_slot_report)
	{
    	input_mt_init_slots(ts->input_dev, 16, 0);     // in case of "out of memory"
	}
	else
    {
		ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	}
    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

    for (index = 0; index < ts->pdata->button_map->nbuttons; index++)
    {
        input_set_capability(ts->input_dev, EV_KEY, ts->pdata->button_map->map[index]);
    }

#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
    for (index = 0; index < ts->pdata->gesture_map->nbuttons; index++)
    {
        input_set_capability(ts->input_dev, EV_KEY, ts->pdata->gesture_map->map[index]);  
    }
#endif 
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);

    sprintf(phys, "input/ts");
    ts->input_dev->name = goodix_ts_name;
    ts->input_dev->phys = phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0xDEAD;
    ts->input_dev->id.product = 0xBEEF;
    ts->input_dev->id.version = 10427;
    
    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        GTP_ERROR("Register %s input device failed", ts->input_dev->name);
        return -ENODEV;
    }


 #if defined(CONFIG_FB)
	INIT_WORK(&ts->resume_work, goodix_ts_resume_work);
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		dev_err(&ts->client->dev,
			"Unable to register fb_notifier: %d\n",
			ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

    return 0;
}


static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

/**
 * goodix_power_on - Turn device power ON
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_on(struct goodix_ts_data *ts)
{
	int ret;

	if (ts->power_on) {
		dev_info(&ts->client->dev,
				"Device already power on\n");
		return 0;
	}

	if (!IS_ERR(ts->avdd)) {
		ret = reg_set_optimum_mode_check(ts->avdd,
			GOODIX_VDD_LOAD_MAX_UA);
		if (ret < 0) {
			dev_err(&ts->client->dev,
				"Regulator avdd set_opt failed rc=%d\n", ret);
			goto err_set_opt_avdd;
		}
		ret = regulator_enable(ts->avdd);
		if (ret) {
			dev_err(&ts->client->dev,
				"Regulator avdd enable failed ret=%d\n", ret);
			goto err_enable_avdd;
		}
	}

	if (!IS_ERR(ts->vdd)) {
		ret = regulator_set_voltage(ts->vdd, GOODIX_VTG_MIN_UV,
					   GOODIX_VTG_MAX_UV);
		if (ret) {
			dev_err(&ts->client->dev,
				"Regulator set_vtg failed vdd ret=%d\n", ret);
			goto err_set_vtg_vdd;
		}
		ret = reg_set_optimum_mode_check(ts->vdd,
			GOODIX_VDD_LOAD_MAX_UA);
		if (ret < 0) {
			dev_err(&ts->client->dev,
				"Regulator vdd set_opt failed rc=%d\n", ret);
			goto err_set_opt_vdd;
		}
		ret = regulator_enable(ts->vdd);
		if (ret) {
			dev_err(&ts->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			goto err_enable_vdd;
		}
	}

	if (!IS_ERR(ts->vcc_i2c)) {
                    if(regulator_count_voltages(ts->vcc_i2c) >  0){
	        		ret = regulator_set_voltage(ts->vcc_i2c, GOODIX_I2C_VTG_MIN_UV,
	        					   GOODIX_I2C_VTG_MAX_UV);
	        		if (ret) {
	        			dev_err(&ts->client->dev,
	        				"Regulator set_vtg failed vcc_i2c ret=%d\n",
	        				ret);
	        			goto err_set_vtg_vcc_i2c;
	        		}
	        		ret = reg_set_optimum_mode_check(ts->vcc_i2c,
	        			GOODIX_VIO_LOAD_MAX_UA);
	        		if (ret < 0) {
	        			dev_err(&ts->client->dev,
	        				"Regulator vcc_i2c set_opt failed rc=%d\n",
	        				ret);
	        			goto err_set_opt_vcc_i2c;
	        		}
                    }
		ret = regulator_enable(ts->vcc_i2c);
		if (ret) {
			dev_err(&ts->client->dev,
				"Regulator vcc_i2c enable failed ret=%d\n",
				ret);
			regulator_disable(ts->vdd);
			goto err_enable_vcc_i2c;
		}
	}

	ts->power_on = true;
	return 0;

err_enable_vcc_i2c:
err_set_opt_vcc_i2c:
	if (!IS_ERR(ts->vcc_i2c))
		regulator_set_voltage(ts->vcc_i2c, 0, GOODIX_I2C_VTG_MAX_UV);
err_set_vtg_vcc_i2c:
	if (!IS_ERR(ts->vdd))
		regulator_disable(ts->vdd);
err_enable_vdd:
err_set_opt_vdd:
	if (!IS_ERR(ts->vdd))
		regulator_set_voltage(ts->vdd, 0, GOODIX_VTG_MAX_UV);
err_set_vtg_vdd:
	if (!IS_ERR(ts->avdd))
		regulator_disable(ts->avdd);
err_enable_avdd:
err_set_opt_avdd:
	ts->power_on = false;
	return ret;
}



/**
 * goodix_power_off - Turn device power OFF
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_off(struct goodix_ts_data *ts)
{
	int ret;

	if (!ts->power_on) {
		dev_info(&ts->client->dev,
				"Device already power off\n");
		return 0;
	}

	if (!IS_ERR(ts->vcc_i2c)) {
		ret = regulator_set_voltage(ts->vcc_i2c, 0,
			GOODIX_I2C_VTG_MAX_UV);
		if (ret < 0)
			dev_err(&ts->client->dev,
				"Regulator vcc_i2c set_vtg failed ret=%d\n",
				ret);
		ret = regulator_disable(ts->vcc_i2c);
		if (ret)
			dev_err(&ts->client->dev,
				"Regulator vcc_i2c disable failed ret=%d\n",
				ret);
	}

	if (!IS_ERR(ts->vdd)) {
		ret = regulator_set_voltage(ts->vdd, 0, GOODIX_VTG_MAX_UV);
		if (ret < 0)
			dev_err(&ts->client->dev,
				"Regulator vdd set_vtg failed ret=%d\n", ret);
		ret = regulator_disable(ts->vdd);
		if (ret)
			dev_err(&ts->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
	}

	if (!IS_ERR(ts->avdd)) {
		ret = regulator_disable(ts->avdd);
		if (ret)
			dev_err(&ts->client->dev,
				"Regulator avdd disable failed ret=%d\n", ret);
	}

	ts->power_on = false;
	return 0;
}


/**
 * goodix_power_init - Initialize device power
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_init(struct goodix_ts_data *ts)
{
	int ret;

	ts->avdd = regulator_get(&ts->client->dev, "avdd");
	if (IS_ERR(ts->avdd)) {
		ret = PTR_ERR(ts->avdd);
		dev_info(&ts->client->dev,
			"Regulator get failed avdd ret=%d\n", ret);
	}

	ts->vdd = regulator_get(&ts->client->dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		ret = PTR_ERR(ts->vdd);
		dev_info(&ts->client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
	}

	ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc-i2c");
	if (IS_ERR(ts->vcc_i2c)) {
		ret = PTR_ERR(ts->vcc_i2c);
		dev_info(&ts->client->dev,
			"Regulator get failed vcc_i2c ret=%d\n", ret);
	}

	return 0;
}



/**
 * goodix_power_deinit - Deinitialize device power
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_deinit(struct goodix_ts_data *ts)
{
	regulator_put(ts->vdd);
	regulator_put(ts->vcc_i2c);
	regulator_put(ts->avdd);

	return 0;
}

static int goodix_ts_get_dt_coords(struct device *dev, char *name,
				struct goodix_ts_platform_data *pdata)
{
	struct property *prop;
	struct device_node *np = dev->of_node;
	int rc;
	u32 coords[4];

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	rc = of_property_read_u32_array(np, name, coords,4);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "goodix,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "goodix,display-coords")) {
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

static int goodix_parse_dt(struct device *dev,
			struct goodix_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];
	u32 gesture_map[MAX_GESTURE];
	char prop_name[PROP_NAME_SIZE];
	int i, read_cfg_num;

	rc = goodix_ts_get_dt_coords(dev, "goodix,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = goodix_ts_get_dt_coords(dev, "goodix,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"goodix,i2c-pull-up");

	pdata->enable_power_off = of_property_read_bool(np,
						"goodix,enable-power-off");
	
	pdata->enable_slot_report = of_property_read_bool(np,
						"goodix,enable-slot-report");

	pdata->enable_esd = of_property_read_bool(np,
						"goodix,enable-esd");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "reset-gpios",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "interrupt-gpios",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	rc = of_property_read_string(np, "goodix,product-id",
						&pdata->product_id);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Failed to parse product_id.");
		return -EINVAL;
	}

	prop = of_find_property(np, "goodix,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;
		pdata->button_map= devm_kzalloc(dev,sizeof(*pdata->button_map),GFP_KERNEL);
		if (!pdata->button_map)
			return -ENOMEM;
		pdata->button_map->map = devm_kzalloc(dev,
					sizeof(*pdata->button_map->map) *
					MAX_BUTTONS, GFP_KERNEL);
		if (!pdata->button_map->map)
			return -ENOMEM;
		rc = of_property_read_u32_array(np,
			"goodix,button-map", button_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			pdata->button_map->map[i] = button_map[i];
		pdata->button_map->nbuttons = num_buttons;
	}

	prop = of_find_property(np, "goodix,gesture-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_GESTURE)
			return -EINVAL;
		pdata->gesture_map= devm_kzalloc(dev,sizeof(*pdata->gesture_map),GFP_KERNEL);
		if (!pdata->gesture_map)
			return -ENOMEM;
		pdata->gesture_map->map = devm_kzalloc(dev,
					sizeof(*pdata->gesture_map->map) *
					MAX_GESTURE, GFP_KERNEL);
		if (!pdata->gesture_map->map)
			return -ENOMEM;
		rc = of_property_read_u32_array(np,
			"goodix,gesture-map", gesture_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read gesture codes\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			pdata->gesture_map->map[i] = gesture_map[i];
		pdata->gesture_map->nbuttons = num_buttons;
	}
	prop = of_find_property(np, "goodix,gesture-key", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_GESTURE)
			return -EINVAL;
		pdata->gesture_key= devm_kzalloc(dev,sizeof(*pdata->gesture_key),GFP_KERNEL);
		if (!pdata->gesture_key)
			return -ENOMEM;
		pdata->gesture_key->map = devm_kzalloc(dev,
					sizeof(*pdata->gesture_key->map) *
					MAX_GESTURE, GFP_KERNEL);
		if (!pdata->gesture_key->map)
			return -ENOMEM;
		rc = of_property_read_u32_array(np,
			"goodix,gesture-key", gesture_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read gesture codes\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			pdata->gesture_key->map[i] = gesture_map[i];
		pdata->gesture_key->nbuttons = num_buttons;
	}
	
	read_cfg_num = 0;
	for (i = 0; i < GOODIX_MAX_CFG_GROUP; i++) {
		snprintf(prop_name, sizeof(prop_name), "goodix,cfg-data%d", i);
		prop = of_find_property(np, prop_name,
			&pdata->config_data_len[i]);
		if (!prop || !prop->value) {
			pdata->config_data_len[i] = 0;
			pdata->config_data[i] = NULL;
			continue;
		}
		pdata->config_data[i] = devm_kzalloc(dev,
				GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH,
				GFP_KERNEL);
		if (!pdata->config_data[i]) {
			dev_err(dev,
				"Not enough memory for panel config data %d\n",
				i);
			return -ENOMEM;
		}
		pdata->config_data[i][0] = GTP_REG_CONFIG_DATA >> 8;
		pdata->config_data[i][1] = GTP_REG_CONFIG_DATA & 0xff;
		memcpy(&pdata->config_data[i][GTP_ADDR_LENGTH],
				prop->value, pdata->config_data_len[i]);
		read_cfg_num++;
	}
	dev_dbg(dev, "%d config data read from device tree.\n", read_cfg_num);
	read_cfg_num = 0;
	for (i = 0; i < GOODIX_MAX_CFG_GROUP; i++) {
		snprintf(prop_name, sizeof(prop_name), "goodix,test-data%d", i);
		prop = of_find_property(np, prop_name,
			&pdata->test_data_len[i]);
		if (!prop || !prop->value) {
			pdata->test_data_len[i] = 0;
			pdata->test_data[i] = NULL;
			continue;
		}
		pdata->test_data[i] = devm_kzalloc(dev,
				GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH,
				GFP_KERNEL);
		if (!pdata->test_data[i]) {
			dev_err(dev,
				"Not enough memory for panel test data %d\n",
				i);
			return -ENOMEM;
		}
		pdata->test_data[i][0] = GTP_REG_CONFIG_DATA >> 8;
		pdata->test_data[i][1] = GTP_REG_CONFIG_DATA & 0xff;
		memcpy(&pdata->test_data[i][GTP_ADDR_LENGTH],
				prop->value, pdata->test_data_len[i]);
		read_cfg_num++;
	}
	return 0;
}

//************** For GT9XXF Start *************//
#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)

s32 gtp_fw_startup(struct i2c_client *client)
{
    u8 opr_buf[4];
    s32 ret = 0;
    struct goodix_ts_data *ts;
    
    ts = i2c_get_clientdata(client);
    //init sw WDT
	opr_buf[0] = 0xAA;
	ret = i2c_write_bytes(client, 0x8041, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    
    //release SS51 & DSP
    opr_buf[0] = 0x00;
    ret = i2c_write_bytes(client, 0x4180, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    //int sync
    gtp_int_sync(ts,25);  
    
    //check fw run status
    ret = i2c_read_bytes(client, 0x8041, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    if(0xAA == opr_buf[0])
    {
        GTP_ERROR("IC works abnormally,startup failed.");
        return FAIL;
    }
    else
    {
		dev_info(&ts->client->dev,"IC works normally, Startup success.\n");
        opr_buf[0] = 0xAA;
        i2c_write_bytes(client, 0x8041, opr_buf, 1);
        return SUCCESS;
    }
}

static s32 gtp_esd_recovery(struct i2c_client *client)
{
    s32 retry = 0;
    s32 ret = 0;
    struct goodix_ts_data *ts;
    
    ts = i2c_get_clientdata(client);
    
    gtp_irq_disable(ts);
    
    GTP_INFO("GT9XXF esd recovery mode");
    for (retry = 0; retry < 5; retry++)
    {
        ret = gup_fw_download_proc(NULL, GTP_FL_ESD_RECOVERY); 
        if (FAIL == ret)
        {
            GTP_ERROR("esd recovery failed %d", retry+1);
            continue;
        }
        ret = gtp_fw_startup(ts->client);
        if (FAIL == ret)
        {
            GTP_ERROR("GT9XXF start up failed %d", retry+1);
            continue;
        }
        break;
    }
    gtp_irq_enable(ts);
    
    if (retry >= 5)
    {
        GTP_ERROR("failed to esd recovery");
        return FAIL;
    }
    
    GTP_INFO("Esd recovery successful");
    return SUCCESS;
}

void gtp_recovery_reset(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if(ts->pdata->enable_esd)
    	gtp_esd_switch(client, SWITCH_OFF);

    GTP_DEBUG_FUNC();
    
    gtp_esd_recovery(client); 
    
	if(ts->pdata->enable_esd)
    	gtp_esd_switch(client, SWITCH_ON);
}

static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode)
{
    s32 ret = 0;
    s32 i = 0;
    s32 j = 0;
    u16 ref_sum = 0;
    u16 learn_cnt = 0;
    u16 chksum = 0;
    s32 ref_seg_len = 0;
    s32 ref_grps = 0;
    struct file *ref_filp = NULL;
    u8 *p_bak_ref;
    
    ret = gup_check_fs_mounted("/data");
    if (FAIL == ret)
    {
        ts->ref_chk_fs_times++;
        GTP_DEBUG("Ref check /data times/MAX_TIMES: %d / %d", ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
        if (ts->ref_chk_fs_times < GTP_CHK_FS_MNT_MAX)
        {
            msleep(50);
			dev_info(&ts->client->dev,"/data not mounted.\n");
            return FAIL;
        }
        GTP_INFO("check /data mount timeout...");
    }
    else
    {
		dev_info(&ts->client->dev,"/data mounted!!!(%d/%d)\n", ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
    }
    
    p_bak_ref = (u8 *)kzalloc(ts->bak_ref_len, GFP_KERNEL);
    
    if (NULL == p_bak_ref)
    {
        GTP_ERROR("Allocate memory for p_bak_ref failed!");
        return FAIL;
    }
    
    if (ts->is_950)
    {
        ref_seg_len = ts->bak_ref_len / 6;
        ref_grps = 6;
    }
    else
    {
        ref_seg_len = ts->bak_ref_len;
        ref_grps = 1;
    }
    ref_filp = filp_open(GTP_BAK_REF_PATH, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(ref_filp))
    { 
        GTP_ERROR("Failed to open/create %s.", GTP_BAK_REF_PATH);
        if (GTP_BAK_REF_SEND == mode)
        {
            goto bak_ref_default;
        }
        else
        {
            goto bak_ref_exit;
        }
    }
    
    switch (mode)
    {
    case GTP_BAK_REF_SEND:
		dev_info(&ts->client->dev,"Send backup-reference\n");
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ret = ref_filp->f_op->read(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
        if (ret < 0)
        {
            GTP_ERROR("failed to read bak_ref info from file, sending defualt bak_ref");
            goto bak_ref_default;
        }
        for (j = 0; j < ref_grps; ++j)
        {
            ref_sum = 0;
            for (i = 0; i < (ref_seg_len); i += 2)
            {
                ref_sum += (p_bak_ref[i + j * ref_seg_len] << 8) + p_bak_ref[i+1 + j * ref_seg_len];
            }
            learn_cnt = (p_bak_ref[j * ref_seg_len + ref_seg_len -4] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len -3]);
            chksum = (p_bak_ref[j * ref_seg_len + ref_seg_len -2] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len -1]);
            GTP_DEBUG("learn count = %d", learn_cnt);
            GTP_DEBUG("chksum = %d", chksum);
            GTP_DEBUG("ref_sum = 0x%04X", ref_sum & 0xFFFF);
            // Sum(1~ref_seg_len) == 1
            if (1 != ref_sum)
            {
				dev_info(&ts->client->dev,"wrong chksum for bak_ref, reset to 0x00 bak_ref\n");
                memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
                p_bak_ref[ref_seg_len + j * ref_seg_len - 1] = 0x01;
            }
            else
            {
                if (j == (ref_grps - 1))
                {
					dev_info(&ts->client->dev,"backup-reference data in %s used\n", GTP_BAK_REF_PATH);
                }
            }
        }
        ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
        if (FAIL == ret)
        {
            GTP_ERROR("failed to send bak_ref because of iic comm error");
            goto bak_ref_exit;
        }
        break;
        
    case GTP_BAK_REF_STORE:
		dev_info(&ts->client->dev,"Store backup-reference\n");
        ret = i2c_read_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
        if (ret < 0)
        {
            GTP_ERROR("failed to read bak_ref info, sending default back-reference");
            goto bak_ref_default;
        }
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ref_filp->f_op->write(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
        break;
        
    default:
        GTP_ERROR("invalid backup-reference request");
        break;
    }
    ret = SUCCESS;
    goto bak_ref_exit;

bak_ref_default:
    
    for (j = 0; j < ref_grps; ++j)
    {
        memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
        p_bak_ref[j * ref_seg_len + ref_seg_len - 1] = 0x01;  // checksum = 1     
    }
    ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
    if (!IS_ERR(ref_filp))
    {
		dev_info(&ts->client->dev,"write backup-reference data into %s\n", GTP_BAK_REF_PATH);
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ref_filp->f_op->write(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
    }
    if (ret == FAIL)
    {
        GTP_ERROR("failed to load the default backup reference");
    }
    
bak_ref_exit:
    
    if (p_bak_ref)
    {
        kfree(p_bak_ref);
    }
    if (ref_filp && !IS_ERR(ref_filp))
    {
        filp_close(ref_filp, NULL);
    }
    return ret;
}


static s32 gtp_verify_main_clk(u8 *p_main_clk)
{
    u8 main_clock = p_main_clk[0];
    s32 i = 0;
	u8 clk_chksum = 0;
    
    if (main_clock < 50 || main_clock > 120)    
    {
        return FAIL;
    }
    
    for (i = 0; i < 5; ++i)
    {
        if (main_clock != p_main_clk[i])
        {
            return FAIL;
        }
		clk_chksum += p_main_clk[i];
    }
    clk_chksum += p_main_clk[5];
    if ( (clk_chksum) == 0)
    {
        return SUCCESS;
    }
    else
    {
        return FAIL;
	}
    return SUCCESS;
}


static void gtp_write_clk_delay(struct work_struct *work)
{
	gtp_write_clk_to_diag(p_main_clk, 6);
}

static s32 gtp_main_clk_proc(struct goodix_ts_data *ts)
{
    s32 ret = 0;
    s32 i = 0;
	s32 clk_chksum = 0;

	for (i = 0; i < 5; ++i)
    {
        p_main_clk[i] = gtp_main_clock[i];
        clk_chksum += p_main_clk[i];
    }
    p_main_clk[5] = 0 - clk_chksum;
		
    ret = gtp_verify_main_clk(p_main_clk);
	
	if (FAIL == ret)
	{
		// recalculate main clock & rewrite main clock data to file
		GTP_ERROR("main clock data in %s is wrong, recalculate main clock", GTP_MAIN_CLK_PATH);
	}
	else
	{ 
		GTP_INFO("main clock data in %s used, main clock freq: %d", GTP_MAIN_CLK_PATH,gtp_main_clock[0]);	
    	goto update_main_clk;
	}
	
	if(ts->pdata->enable_esd)
    	gtp_esd_switch(ts->client, SWITCH_OFF);
	ret = gup_clk_calibration();
    gtp_esd_recovery(ts->client);
    
	if(ts->pdata->enable_esd)
    	gtp_esd_switch(ts->client, SWITCH_ON);

    GTP_INFO("calibrate main clock: %d", ret);
    if (ret < 50 || ret > 120)
    {
        GTP_ERROR("wrong main clock: %d", ret);
        return FAIL;
    }
    
    // Sum{0x8020~0x8025} = 0
    for (i = 0; i < 5; ++i)
    {
        p_main_clk[i] = ret;
        clk_chksum += p_main_clk[i];
    }
    p_main_clk[5] = 0 - clk_chksum;

	queue_delayed_work(gtp_main_clk_workqueue,&gtp_main_clk_work,500);
    
update_main_clk:
    ret = i2c_write_bytes(ts->client, GTP_REG_MAIN_CLK, p_main_clk, 6);
    if (FAIL == ret)
    {
        GTP_ERROR("update main clock failed!");
        return FAIL;
    }
    return SUCCESS;
}


s32 gtp_gt9xxf_init(struct i2c_client *client)
{
    s32 ret = 0;
    ret = gup_fw_download_proc(NULL, GTP_FL_FW_BURN); 
    if (FAIL == ret)
    {
        return FAIL;
    }
    
    ret = gtp_fw_startup(client);
    if (FAIL == ret)
    {
        return FAIL;
    }
    return SUCCESS;
}

void gtp_get_chip_type(struct goodix_ts_data *ts)
{
    u8 opr_buf[10] = {0x00};
    s32 ret = 0;
    
    msleep(10);
    
    ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CHIP_TYPE, opr_buf, 10);
    
    if (FAIL == ret)
    {
        GTP_ERROR("Failed to get chip-type, set chip type default: GOODIX_GT9");
        ts->chip_type = CHIP_TYPE_GT9;
        return;
    }
    
    if (!memcmp(opr_buf, "GOODIX_GT9", 10))
    {
        ts->chip_type = CHIP_TYPE_GT9;
    }
    else // GT9XXF
    {
        ts->chip_type = CHIP_TYPE_GT9F;
    }
    GTP_INFO("Chip Type: %s", (ts->chip_type == CHIP_TYPE_GT9) ? "GOODIX_GT9" : "GOODIX_GT9F");
}

static int diag_write(unsigned int offset, char *buf, unsigned int size)
{
    int result = -1;
    char *pBuffer;
    struct block_device *bdev;
    unsigned int start = 0;
    unsigned int align = 0;
    unsigned int len = 0;
    unsigned int copied = 0;

    bdev = diag_partition_open();
    if(!bdev)
    {	
    	printk("open diag partition failed!\n");
    	return (-1);
    }  

    pBuffer = kmalloc(ABOOT_READ_LENGTH, GFP_KERNEL);
    if (NULL == pBuffer)
    {
    	printk(KERN_ERR "kmalloc failed!\n");
    	return -1;
    }

    memset(pBuffer, 0, ABOOT_READ_LENGTH);

    start = offset & (~(PAGE_SIZE - 1));
    align = offset % PAGE_SIZE;

    while (copied < size)
    {
    	result = diag_partition_read(bdev, start, pBuffer, ABOOT_READ_LENGTH);
    	if (result < 0)
    	{
    		printk("read diag failed!\n");
    		kfree(pBuffer);
    		diag_partition_close(bdev);
    		return (-1);
    	}

    	len = ((ABOOT_READ_LENGTH - align) >= (size - copied)) ? 
    		(size - copied) : (ABOOT_READ_LENGTH - align);
    	memcpy(pBuffer + align, buf + copied, len);
    	result = diag_partition_write(bdev, start, pBuffer, ABOOT_READ_LENGTH);
    	if(result < 0)
    	{
    		printk("write diag failed\n");
    	}

    	copied += len;
    	start += ABOOT_READ_LENGTH;
    	align = 0;
    }

    diag_partition_close(bdev);
    kfree(pBuffer);

    return result;
}


static struct block_device* diag_partition_open( void )
{
	struct block_device *bdev;

	bdev = blkdev_get_by_path("/dev/block/platform/7824900.sdhci/by-name/diag", 
		FMODE_WRITE|FMODE_READ, NULL);
	if (IS_ERR(bdev))
	{
        printk(KERN_ERR "open /dev/block/platform/7824900.sdhci/by-name/diag block device failed!\n");
        return NULL;
	}
	
	return bdev;
}

static void diag_partition_close( struct block_device *bdev)
{
    blkdev_put(bdev, FMODE_READ|FMODE_WRITE);
}

static int diag_partition_read(struct block_device *bdev, unsigned int where, char *pBuffer, unsigned int size)
{
    struct buffer_head *bh = NULL;
    unsigned int bytesOfRead = 0;
    unsigned int numOfPages = 0;

    if(size > ABOOT_PART_LENGTH)
    {
        printk("%s(): size is not right!\n", __func__);
        return -1;
    }

    if(where%PAGE_SIZE != 0)
    {
        printk("%s(): where is not right!\n", __func__);
        return -1;
    }
    
    numOfPages = (size/PAGE_SIZE) + ((size%PAGE_SIZE)? 1 : 0);
    if(numOfPages > (ABOOT_PART_LENGTH/PAGE_SIZE))
    {
        printk("%s(): size is not right!\n", __func__);
        return -1;
    }
    
    while(bytesOfRead < size)
    {
        bh = __bread(bdev, (where+bytesOfRead)/PAGE_SIZE, PAGE_SIZE);
        memcpy(pBuffer+bytesOfRead, bh->b_data, PAGE_SIZE);
        bytesOfRead += PAGE_SIZE;
    }
    
    brelse(bh);
    return bytesOfRead;
}

static int diag_partition_write(struct block_device *bdev, unsigned int where, char *pBuffer, unsigned int size)
{
    struct buffer_head *bh;
    unsigned int pagesOfWritten = 0;
    unsigned int numOfPages = 0;
    unsigned int pageOfStart = 0;

    if(size > ABOOT_PART_LENGTH)
    {
        printk("%s(): size is not right!\n", __func__);
        return -1;
    }

    if(where%PAGE_SIZE != 0)
    {
        printk("%s(): where is not right!\n", __func__);
        return -1;
    }

    numOfPages = (size/PAGE_SIZE) + ((size%PAGE_SIZE)? 1 : 0);
    if(numOfPages > (ABOOT_PART_LENGTH/PAGE_SIZE))
    {
        printk("%s(): size is not right!\n", __func__);
        return -1;
    }

    pageOfStart = where/PAGE_SIZE;
    while(pagesOfWritten < numOfPages)
    {
        bh = __getblk(bdev, pageOfStart+pagesOfWritten, PAGE_SIZE);
        set_buffer_uptodate(bh);
        memcpy(bh->b_data, pBuffer+pagesOfWritten*PAGE_SIZE, PAGE_SIZE);
        mark_buffer_dirty(bh);
        ll_rw_block(WRITE, 1, &bh);
        brelse(bh);
        pagesOfWritten++;
    }

    return ((pagesOfWritten+1)*PAGE_SIZE);
}

static int gtp_write_clk_to_diag(char *buf, unsigned int size)
{
    int result = -1;
    struct block_device *bdev;
    int diag_size = 0;
    
    if(diag_size == 0)
    {
        bdev = diag_partition_open();
        if(!bdev)
        {   
            printk("open diag partition failed!\n");
            return (-1);
        }
        diag_size = bdev->bd_part->nr_sects;
        printk("wpc: partition diag size: %d ", diag_size);
        diag_partition_close(bdev);
    }
    result = diag_write((diag_size-1)*512+32, buf, 6);
    
    return result;
}



static void gtp_recovery_reset_func(struct work_struct *work)
{
	gtp_recovery_reset(i2c_connect_client);
}

#endif
//************* For GT9XXF End ************//

/*******************************************************
Function:
    I2c probe.
Input:
    client: i2c device struct.
    id: device id.
Output:
    Executive outcomes. 
        0: succeed.
*******************************************************/
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;
    struct goodix_ts_platform_data *pdata;
    struct goodix_ts_data *ts;
    u16 version_info;
    
    GTP_DEBUG_FUNC();
    //do NOT remove these logs
    GTP_DEBUG("GTP Driver Version: %s", GTP_DRIVER_VERSION);
    GTP_DEBUG("GTP Driver Built@%s, %s", __TIME__, __DATE__);
    GTP_DEBUG("GTP I2C Address: 0x%02x", client->addr);

    if (client->dev.of_node) {
        pdata = devm_kzalloc(&client->dev,sizeof(struct goodix_ts_platform_data), GFP_KERNEL);
        if (!pdata) {
            dev_err(&client->dev,
            	"GTP Failed to allocate memory for pdata\n");
            return -ENOMEM;
        }

        ret = goodix_parse_dt(&client->dev, pdata);
        if (ret)
            return ret;
    } else {
        pdata = client->dev.platform_data;
    }

    if (!pdata) {
        dev_err(&client->dev, "GTP invalid pdata\n");
        return -EINVAL;
    }

    i2c_connect_client = client;
    
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
        GTP_ERROR("I2C check functionality failed.");
        return -ENODEV;
    }
    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
    {
        GTP_ERROR("Alloc GFP_KERNEL memory failed.");
        return -ENOMEM;
    }
    
    memset(ts, 0, sizeof(*ts));
    INIT_WORK(&ts->work, goodix_ts_work_func);
#if 0
	INIT_DELAYED_WORK(&gtp_productinfo_work, goodix_register_productinfo_func);
#endif
	gtp_productinfo_workqueue = create_workqueue("gtp_productinfo");
	
#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
		gtp_recovery_reset_workqueue = create_workqueue("gtp_recovery_reset_workqueue");
		INIT_WORK(&gtp_recovery_reset_work, gtp_recovery_reset_func);
#endif

    ts->client = client;
    ts->pdata = pdata;
    spin_lock_init(&ts->irq_lock);          // 2.6.39 later
    // ts->irq_lock = SPIN_LOCK_UNLOCKED;   // 2.6.39 & before

	if(ts->pdata->enable_esd)
	{
		
		INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
		gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
		if(!gtp_esd_check_workqueue)
		{
			GTP_ERROR("Creat esd_workqueue failed.");
			return -ENOMEM;
		}
	    ts->clk_tick_cnt = 2 * HZ;      // HZ: clock ticks in 1 second generated by system
	    GTP_DEBUG("Clock ticks for an esd cycle: %d", ts->clk_tick_cnt);  
	    spin_lock_init(&ts->esd_lock);
	    // ts->esd_lock = SPIN_LOCK_UNLOCKED;
	}

    i2c_set_clientdata(client, ts);
    
    ts->gtp_rawdiff_mode = 0;
    ts->gtp_reset_mode = 0;
    ts->power_on = false;
	ts->gtp_is_suspend = false;

	ret = goodix_ts_pinctrl_init(ts);
		if (!ret && ts->ts_pinctrl) {
			ret = goodix_ts_pinctrl_select(ts, true);
			if (ret < 0)
				goto exit_free_client_data;
		}

    ret = gtp_request_io_port(ts);
    if (ret) {
    	dev_err(&client->dev, "GTP request IO port failed.\n");    	
		goto exit_free_client_data;
    }

    ret = goodix_power_init(ts);
    if (ret) {
    	dev_err(&client->dev, "GTP power init failed\n");
		goto exit_free_io_port;
    }

    ret = goodix_power_on(ts);
    if (ret) {
    	dev_err(&client->dev, "GTP power on failed\n");
		goto exit_deinit_power;
    }

    gtp_reset_guitar(ts->client, 10);
	
#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
	INIT_DELAYED_WORK(&gtp_main_clk_work, gtp_write_clk_delay);
	gtp_main_clk_workqueue = create_workqueue("gtp_main_clk_to_diag");
	
	gtp_get_chip_type(ts);
		
	if (CHIP_TYPE_GT9F == ts->chip_type)
	{
		ret = gtp_gt9xxf_init(ts->client);
		if (FAIL == ret)
		{
			GTP_INFO("Failed to init GT9XXF.");
		}
	}
#endif

    ret = gtp_i2c_test(client);
    if (ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
		goto exit_power_off;
    }

    ret = gtp_read_version(ts, &version_info);
    if (ret < 0)
    {
        GTP_ERROR("Read version failed.");

    }

    ret = gtp_init_panel(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP init panel failed.");
        ts->abs_x_max = ts->pdata->x_max;
        ts->abs_y_max = ts->pdata->y_max;
        ts->int_trigger_type = GTP_INT_TRIGGER;
    }
    
    // Create proc file system
    gt91xx_config_proc = proc_create(GT91XX_CONFIG_PROC_FILE, 0666, NULL, &config_proc_ops);
    if (gt91xx_config_proc == NULL)
    {
        GTP_ERROR("create_proc_entry %s failed\n", GT91XX_CONFIG_PROC_FILE);
    }
    else
    {
        GTP_DEBUG("create proc entry %s success", GT91XX_CONFIG_PROC_FILE);
    }

    ret = gtp_request_input_dev(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request input dev failed");
    }
    
    ret = gtp_request_irq(ts); 
    if (ret < 0)
    {
        GTP_DEBUG("GTP works in polling mode.");
    }
    else
    {
        GTP_DEBUG("GTP works in interrupt mode.");
    }

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }

    init_wr_node(client);
    
	if(ts->pdata->enable_esd)
	    gtp_esd_switch(client, SWITCH_ON);

#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
	 ts->gesture_wakeup_enable = false;
	ts->gesture_state = 0x00;
#endif

    factory_ts_func_test_register(ts);
	//queue_delayed_work(gtp_productinfo_workqueue,&gtp_productinfo_work,200);
    return 0;
	
exit_power_off:
	goodix_power_off(ts);
exit_deinit_power:
	goodix_power_deinit(ts);
exit_free_io_port:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
	if (ts->ts_pinctrl) {
		//ret = goodix_ts_pinctrl_select(ts, false); //DO NOT CHANGE RET VALUE,IT BREAK DRIVER PROBE LOGIC
		if (goodix_ts_pinctrl_select(ts, false) < 0)
			pr_err("Cannot get idle pinctrl state\n");
	}
	//pinctrl_disable_state(ts->ts_pinctrl);
exit_free_client_data:
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return ret;
}

/*******************************************************
Function:
    Goodix touchscreen driver release function.
Input:
    client: i2c device struct.
Output:
    Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(client);
    int ret;
	
    GTP_DEBUG_FUNC();
    
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
#endif

    uninit_wr_node();
	if(gtp_esd_check_workqueue)
	    destroy_workqueue(gtp_esd_check_workqueue);
    if (ts) 
    {
        if (ts->use_irq)
        {
            GTP_GPIO_AS_INPUT(GTP_INT_PORT);
            GTP_GPIO_FREE(GTP_INT_PORT);
            free_irq(client->irq, ts);
        }
        else
        {
            hrtimer_cancel(&ts->timer);
        }
    }   
    
    GTP_INFO("GTP driver removing...");
    i2c_set_clientdata(client, NULL);
    input_unregister_device(ts->input_dev);
	goodix_power_deinit(ts);
	if (ts->ts_pinctrl) {
		ret = goodix_ts_pinctrl_select(ts, false);
		if (ret < 0)
			pr_err("Cannot get idle pinctrl state\n");
	}
	//pinctrl_disable_state(ts->ts_pinctrl);
    kfree(ts);

	return 0;
}

static int goodix_ts_pinctrl_init(struct goodix_ts_data *ts)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ts->ts_pinctrl = devm_pinctrl_get(&(ts->client->dev));
	if (IS_ERR_OR_NULL(ts->ts_pinctrl)) {
		dev_dbg(&ts->client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(ts->ts_pinctrl);
		ts->ts_pinctrl = NULL;
		return retval;
	}

	ts->gpio_state_active
		= pinctrl_lookup_state(ts->ts_pinctrl,
			"pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->gpio_state_active)) {
		dev_dbg(&ts->client->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(ts->gpio_state_active);
		ts->ts_pinctrl = NULL;
		return retval;
	}

	ts->gpio_state_suspend
		= pinctrl_lookup_state(ts->ts_pinctrl,
			"pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->gpio_state_suspend)) {
		dev_err(&ts->client->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(ts->gpio_state_suspend);
		ts->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int goodix_ts_pinctrl_select(struct goodix_ts_data *ts,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? ts->gpio_state_active
		: ts->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(ts->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&ts->client->dev,
				"can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else {
		dev_err(&ts->client->dev,
			"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return 0;
}


#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)
/*******************************************************
Function:
	Early suspend function.
Input:
	ts: goodix_ts_data struct.
Output:
	None.
*******************************************************/
static void goodix_ts_suspend(struct goodix_ts_data *ts)
{
    s8 ret = -1;    
    
    suspendinfo_start(S_A_TOUCH_ID);
    GTP_DEBUG_FUNC();
    
    if (ts->enter_update)
    {
    	GTP_INFO("Firmware loading in process...");
        return;
    }

	if(ts->gtp_is_suspend)
	{
		GTP_INFO("Already in suspend state.");
		suspendinfo_end(S_A_TOUCH_ID);
		return;
	}

	GTP_INFO("System suspend.");
	
    ts->gtp_is_suspend = 1;
	
	if(ts->pdata->enable_esd)
    	gtp_esd_switch(ts->client, SWITCH_OFF);

#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
	ts->gesture_wakeup_enable_pre = ts->gesture_wakeup_enable;
    if(ts->gesture_wakeup_enable){
    	ret = gtp_enter_doze(ts);
    }else
#endif
	{
	    if (ts->use_irq)
	    {
	        gtp_irq_disable(ts);
	    }
	    else
	    {
	        hrtimer_cancel(&ts->timer);
	    }
	    ret = gtp_enter_sleep(ts);
	}
 
    if (ret < 0)
    {
        GTP_ERROR("GTP early suspend failed.");
    }
    // to avoid waking up while not sleeping
    //  delay 48 + 10ms to ensure reliability    
    msleep(58);   
    suspendinfo_end(S_A_TOUCH_ID);
}



/*******************************************************
Function:
	Late resume function.
Input:
	ts: goodix_ts_data struct.
Output:
	None.
*******************************************************/
static void goodix_ts_resume(struct goodix_ts_data *ts)
{
    
    s8 ret = -1;    

    resumeinfo_start(S_A_TOUCH_ID);
    GTP_DEBUG_FUNC();
	
    if (ts->enter_update)
    {
    	GTP_INFO("Firmware loading in process...");
        return;
    }
	/* if device is not in suspend mode, do NOT resume */
	if(!ts->gtp_is_suspend)
	{
		GTP_INFO("Already in awake state.");
		resumeinfo_end(S_A_TOUCH_ID);
		return;
	}

    GTP_INFO("System resume.");
	
    ret = gtp_wakeup_sleep(ts);

#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
    doze_status = DOZE_DISABLED;
#endif

    if (ret < 0)
    {
        GTP_ERROR("GTP later resume failed.");
    }
#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        // do nothing
    }
    else
#endif
    gtp_send_cfg(ts->client);

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
    else
    {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

    ts->gtp_is_suspend = 0;

	if(ts->pdata->enable_esd)
    	gtp_esd_switch(ts->client, SWITCH_ON);

	resumeinfo_end(S_A_TOUCH_ID);
}
#if defined(CONFIG_FB)

static void goodix_ts_resume_work(struct work_struct *work)
{
    struct goodix_ts_data *ts_data = container_of(work, 
    										   struct goodix_ts_data, resume_work);
    	
    goodix_ts_resume(ts_data);
}


static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct goodix_ts_data *ts =
    	container_of(self, struct goodix_ts_data, fb_notif);

    if (evdata && evdata->data && event == FB_EVENT_BLANK &&
    		ts && ts->client) {
    	blank = evdata->data;
    	if (*blank == FB_BLANK_UNBLANK) {
    		if (!work_pending(&ts->resume_work)){
    			schedule_work(&ts->resume_work);				
    		}			
    	} else if (*blank == FB_BLANK_POWERDOWN) {
    		cancel_work_sync(&ts->resume_work);
    		goodix_ts_suspend(ts);
    	}
    }

    return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_early_suspend(struct early_suspend *h)
{
    struct goodix_ts_data *ts;
   
    ts = container_of(h, struct goodix_ts_data, early_suspend);
    
    GTP_DEBUG_FUNC();
    
   goodix_ts_suspend(ts);
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_late_resume(struct early_suspend *h)
{
    struct goodix_ts_data *ts;

    ts = container_of(h, struct goodix_ts_data, early_suspend);
    
    GTP_DEBUG_FUNC();
    
   goodix_ts_resume(ts);
}
#endif
#endif

s32 gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.
    
    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if ((retries >= 5))
    {    
        GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}

s32 gtp_i2c_write_no_rst(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
    //msg.scl_rate = 300 * 1000;    // for Rockchip, etc

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
        GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}
/*******************************************************
Function:
    switch on & off esd delayed work
Input:
    client:  i2c device
    on:      SWITCH_ON / SWITCH_OFF
Output:
    void
*********************************************************/
void gtp_esd_switch(struct i2c_client *client, s32 on)
{
    struct goodix_ts_data *ts;
    
    ts = i2c_get_clientdata(client);
    spin_lock(&ts->esd_lock);
    
    if (SWITCH_ON == on)     // switch on esd 
    {
        if (!ts->esd_running)
        {
            ts->esd_running = 1;
            spin_unlock(&ts->esd_lock);
            GTP_INFO("Esd started");
            queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
    }
    else    // switch off esd
    {
        if (ts->esd_running)
        {
            ts->esd_running = 0;
            spin_unlock(&ts->esd_lock);
            GTP_INFO("Esd cancelled");
            cancel_delayed_work_sync(&gtp_esd_check_work);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
    }
}

/*******************************************************
Function:
    Initialize external watchdog for esd protect
Input:
    client:  i2c device.
Output:
    result of i2c write operation. 
        1: succeed, otherwise: failed
*********************************************************/
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
    u8 opr_buffer[3] = {0x80, 0x41, 0xAA};
    GTP_DEBUG("[Esd]Init external watchdog");
    return gtp_i2c_write_no_rst(client, opr_buffer, 3);
}

/*******************************************************
Function:
    Esd protect function.
    External watchdog added by meta, 2013/03/07
Input:
    work: delayed work
Output:
    None.
*******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
    s32 i;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;
    u8 esd_buf[5] = {0x80, 0x40};
    
    GTP_DEBUG_FUNC();
   
    ts = i2c_get_clientdata(i2c_connect_client);

    if (ts->gtp_is_suspend)
    {
        GTP_INFO("Esd suspended!");
        return;
    }
    
    for (i = 0; i < 3; i++)
    {
        ret = gtp_i2c_read_no_rst(ts->client, esd_buf, 4);
        
        GTP_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[2], esd_buf[3]);
        if ((ret < 0))
        {
            // IIC communication problem
            continue;
        }
        else
        { 
            if ((esd_buf[2] == 0xAA) || (esd_buf[3] != 0xAA))
            {
                // IC works abnormally..
                u8 chk_buf[4] = {0x80, 0x40};
                
                gtp_i2c_read_no_rst(ts->client, chk_buf, 4);
                
                GTP_DEBUG("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X", chk_buf[2], chk_buf[3]);
                
                if ((chk_buf[2] == 0xAA) || (chk_buf[3] != 0xAA))
                {
                    i = 3;
                    break;
                }
                else
                {
                    continue;
                }
            }
            else 
            {
                // IC works normally, Write 0x8040 0xAA, feed the dog
                esd_buf[2] = 0xAA; 
                gtp_i2c_write_no_rst(ts->client, esd_buf, 3);
                break;
            }
        }
    }
    if (i >= 3)
    {
    #if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
        if (CHIP_TYPE_GT9F == ts->chip_type)
        {        
            if (ts->rqst_processing)
            {
				dev_info(&ts->client->dev,"Request processing, no esd recovery\n");
            }
            else
            {
                GTP_ERROR("IC working abnormally! Process esd recovery.");
                esd_buf[0] = 0x42;
                esd_buf[1] = 0x26;
                esd_buf[2] = 0x01;
                esd_buf[3] = 0x01;
                esd_buf[4] = 0x01;
                gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
                msleep(50);
                gtp_esd_recovery(ts->client);
            }
        }
        else
    #endif
        {
            GTP_ERROR("IC working abnormally! Process reset guitar.");
            esd_buf[0] = 0x42;
            esd_buf[1] = 0x26;
            esd_buf[2] = 0x01;
            esd_buf[3] = 0x01;
            esd_buf[4] = 0x01;
            gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
            msleep(50);
            gtp_reset_guitar(ts->client, 10);
            msleep(50);
            gtp_send_cfg(ts->client);
        }
    }

    if(!ts->gtp_is_suspend)
    {
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
    }
    else
    {
        GTP_INFO("Esd suspended!");
    }
    return;
}

#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
static int goodix_ts_gesture_suspend(struct device *dev)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	if(ts->gesture_wakeup_enable_pre){
		printk("%s. disable irq.\n",__func__);
		disable_irq(ts->client->irq);
		enable_irq_wake(ts->client->irq);
	}
	return 0;
}
static int goodix_ts_gesture_resume(struct device *dev)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	if(ts->gesture_wakeup_enable_pre){
		printk("%s. enable irq.\n",__func__);
		disable_irq_wake(ts->client->irq);
		enable_irq(ts->client->irq);
	}
	return 0;
}

static const struct dev_pm_ops goodix_ts_dev_pm_ops = {
	.suspend = goodix_ts_gesture_suspend,
	.resume  = goodix_ts_gesture_resume,
};
#endif

static const struct i2c_device_id goodix_ts_id[] = {
    { GTP_I2C_NAME, 0 },
    { }
};

static struct of_device_id goodix_match_table[] = {
	{ .compatible = "goodix,gt9xx", },
	{ },
};

static struct i2c_driver goodix_ts_driver = {
    .probe      = goodix_ts_probe,
    .remove     = goodix_ts_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
    .suspend    = goodix_ts_early_suspend,
    .resume     = goodix_ts_late_resume,
#endif
    .id_table   = goodix_ts_id,
    .driver = {
        .name     = GTP_I2C_NAME,
        .owner    = THIS_MODULE,        
        .of_match_table = goodix_match_table,
#if defined(CONFIG_TOUCHSCREEN_GT9XX_GESTURE)
		.pm = &goodix_ts_dev_pm_ops,
#endif
	
    },
};

/*******************************************************    
Function:
    Driver Install function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static int goodix_ts_init(void)
{
    s32 ret;
	GTP_INFO("GTP driver installing...");
    goodix_wq = create_singlethread_workqueue("goodix_wq");
    if (!goodix_wq)
    {
        GTP_ERROR("Creat workqueue failed.");
        return -ENOMEM;
    }
    ret = i2c_add_driver(&goodix_ts_driver);

    return ret; 
}

/*******************************************************    
Function:
    Driver uninstall function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static void goodix_ts_exit(void)
{
    GTP_DEBUG_FUNC();
    GTP_INFO("GTP driver exited.");
    i2c_del_driver(&goodix_ts_driver);
    if (goodix_wq)
    {
        destroy_workqueue(goodix_wq);
    }
}

#if defined(CONFIG_TOUCHSCREEN_GT9XX_COMPATIBLE_MODE)
static int gtp_get_main_clock_init(char *s)
{
	strlcpy(gtp_main_clock, s, sizeof(gtp_main_clock));
	return 1;
}
__setup("androidboot.gt9xx=", gtp_get_main_clock_init);
#endif

module_init(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
