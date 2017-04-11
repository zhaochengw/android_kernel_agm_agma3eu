/* drivers/input/touchscreen/gt1x.c
 *
 * 2010 - 2014 Goodix Technology.
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
 * Version: 1.2
 * Release Date:  2015/04/20
 */

#include <linux/irq.h>
#include "gt1x.h"
#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif
#include <linux/productinfo.h>

static struct input_dev *input_dev;
static struct workqueue_struct *gt1x_wq;
static const char *gt1x_ts_name = "goodix-ts";
static const char *input_dev_phys = "input/ts";
#ifdef GTP_CONFIG_OF
int gt1x_rst_gpio;
int gt1x_int_gpio;
#endif

static int gt1x_register_powermanger(void);
static int gt1x_unregister_powermanger(void);
void gt1x_ts_register_productinfo(struct goodix_ts_data *ts_data);


/**
 * gt1x_i2c_write - i2c write.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_write(u16 addr, u8 *buffer, s32 len)
{
	struct i2c_msg msg = {
		.flags = 0,
		.addr = gt1x_i2c_client->addr,
	};
	return _do_i2c_write(&msg, addr, buffer, len);
}

/**
 * gt1x_i2c_read - i2c read.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_read(u16 addr, u8 *buffer, s32 len)
{
	u8 addr_buf[GTP_ADDR_LENGTH] = { (addr >> 8) & 0xFF, addr & 0xFF };
	struct i2c_msg msgs[2] = {
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = 0,
		 .buf = addr_buf,
		 .len = GTP_ADDR_LENGTH},
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = I2C_M_RD}
	};
	return _do_i2c_read(msgs, addr, buffer, len);
}

static s32 irq_is_disable;

/**
 * gt1x_irq_enable - enable irq function.
 *
 */
void gt1x_irq_enable(void)
{
	unsigned long irqflags = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(gt1x_i2c_client);

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (irq_is_disable) {
		enable_irq(gt1x_i2c_client->irq);
		irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/**
 * gt1x_irq_enable - disable irq function.
 *
 */
void gt1x_irq_disable(void)
{
	unsigned long irqflags;
	struct goodix_ts_data *ts = i2c_get_clientdata(gt1x_i2c_client);

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!irq_is_disable) {
		irq_is_disable = 1;
		disable_irq_nosync(gt1x_i2c_client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

int gt1x_debug_proc(u8 *buf, int count)
{
	return -ENOMEM;
}

/**
 * gt1x_ts_irq_handler - External interrupt service routine for interrupt mode.
 * @irq:  interrupt number.
 * @dev_id: private data pointer.
 * Return: Handle Result.
 * IRQ_HANDLED: interrupt handled successfully
 */
static irqreturn_t gt1x_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(gt1x_i2c_client);

	GTP_DEBUG_FUNC();
	gt1x_irq_disable();
	queue_work(gt1x_wq, &ts->work);

	return IRQ_HANDLED;
}

/**
 * gt1x_touch_down - Report touch point event .
 * @id: trackId
 * @x:  input x coordinate
 * @y:  input y coordinate
 * @w:  input pressure
 * Return: none.
 */
void gt1x_touch_down(s32 x, s32 y, s32 size, s32 id)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

#if GTP_ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_report_abs(input_dev, ABS_MT_PRESSURE, size);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
#else
	input_report_key(input_dev, BTN_TOUCH, 1);
	if ((!size) && (!id)) {
		/* for virtual button */
		input_report_abs(input_dev, ABS_MT_PRESSURE, 100);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 100);
	} else {
		input_report_abs(input_dev, ABS_MT_PRESSURE, size);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);
		input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	}
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_touch_up -  Report touch release event.
 * @id: trackId
 * Return: none.
 */
void gt1x_touch_up(s32 id)
{
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
#else
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_ts_work_func - Goodix touchscreen work function.
 * @iwork: work struct of gt1x_workqueue.
 * Return: none.
 */
static void gt1x_ts_work_func(struct work_struct *work)
{
	u8 end_cmd = 0;
	u8 finger = 0;
	s32 ret = 0;
	u8 point_data[11] = { 0 };

	if (update_info.status) {
		GTP_DEBUG("Ignore interrupts during fw update.");
		return;
	}

#ifdef CONFIG_TOUCHSCREEN_GT1X_GESTURE
	ret = gesture_event_handler(input_dev);
	if (ret >= 0)
		goto exit_work_func;
#endif

	if (gt1x_halt) {
		GTP_DEBUG("Ignore interrupts after suspend...");
		return;
	}

	ret = gt1x_i2c_read(GTP_READ_COOR_ADDR, point_data, sizeof(point_data));
	if (ret < 0) {
		GTP_ERROR("I2C transfer error!");
#if !GTP_ESD_PROTECT
		gt1x_power_reset(true);
#endif
		goto exit_work_func;
	}

	finger = point_data[0];
	if (finger == 0x00)
		gt1x_request_event_handler();

	if ((finger & 0x80) == 0) {
#if HOTKNOT_BLOCK_RW
		if (!hotknot_paired_flag)
#endif
		{
			goto exit_eint;
		}
	}
#if HOTKNOT_BLOCK_RW
	ret = hotknot_event_handler(point_data);
	if (!ret)
		goto exit_work_func;
#endif

#if GTP_PROXIMITY
	ret = gt1x_prox_event_handler(point_data);
	if (ret > 0)
		goto exit_work_func;
#endif

#if GTP_WITH_STYLUS
	ret = gt1x_touch_event_handler(point_data, input_dev, pen_dev);
#else
	ret = gt1x_touch_event_handler(point_data, input_dev, NULL);
#endif

exit_work_func:
	if (!gt1x_rawdiff_mode && (ret >= 0 || ret == ERROR_VALUE)) {
		ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
		if (ret < 0)
			GTP_ERROR("I2C write end_cmd  error!");
	}
exit_eint:
	gt1x_irq_enable();

}

/*
* Devices Tree support,
*/
#ifdef GTP_CONFIG_OF
/**
 * gt1x_parse_dt - parse platform infomation form devices tree.
 */
static int gt1x_parse_dt(struct device *dev,
			struct goodix_ts_platform_data *pdata)
{
	int rc = 0;
	struct device_node *np;
	char prop_name[PROP_NAME_SIZE];
	int i = 0;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 gesture_map[MAX_GESTURE];

	if (!dev)
		return -ENODEV;
	np = dev->of_node;
	gt1x_int_gpio = of_get_named_gpio(np, "interrupt-gpios", 0);
	gt1x_rst_gpio = of_get_named_gpio(np, "reset-gpios", 0);

	if (!gpio_is_valid(gt1x_int_gpio) || !gpio_is_valid(gt1x_rst_gpio)) {
		GTP_ERROR("Invalid GPIO, irq-gpio:%d, rst-gpio:%d",
			gt1x_int_gpio, gt1x_rst_gpio);
		return -EINVAL;
	}

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
				GTP_CONFIG_MAX_LENGTH, GFP_KERNEL);
		if (!pdata->config_data[i]) {
			dev_err(dev,
				"Not enough memory for panel config data %d\n",
				i);
			return -ENOMEM;
		}
		memcpy(&pdata->config_data[i][0],
				prop->value, pdata->config_data_len[i]);
	}

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
				GTP_CONFIG_MAX_LENGTH, GFP_KERNEL);
		if (!pdata->test_data[i]) {
			dev_err(dev,
				"Not enough memory for panel config data %d\n",
				i);
			return -ENOMEM;
		}
		memcpy(&pdata->test_data[i][0],
				prop->value, pdata->test_data_len[i]);
	}

	for (i = 0; i < GOODIX_MAX_CFG_GROUP; i++) {
		snprintf(prop_name, sizeof(prop_name), "goodix,chr-data%d", i);
		prop = of_find_property(np, prop_name,
			&pdata->chr_data_len[i]);
		if (!prop || !prop->value) {
			pdata->chr_data_len[i] = 0;
			pdata->chr_data[i] = NULL;
			continue;
		}
		pdata->chr_data[i] = devm_kzalloc(dev,
				GTP_CONFIG_MAX_LENGTH, GFP_KERNEL);
		if (!pdata->chr_data[i]) {
			dev_err(dev,
				"Not enough memory for panel chr data %d\n",
				i);
			return -ENOMEM;
		}
		memcpy(&pdata->chr_data[i][0],
				prop->value, pdata->chr_data_len[i]);
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

	rc = of_property_read_u32(np, "goodix,touch-area-param",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read touch area param\n");
		return rc;
	} else if (rc) {
		pdata->touch_area_param = 1;
		rc = 0;
	} else {
		pdata->touch_area_param = temp_val;
	}

	return rc;
}

int gt1x_power_on(struct goodix_ts_data *ts, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(ts->vdd);
	if (rc) {
		dev_err(&ts->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(ts->vcc_i2c);
	if (rc) {
		dev_err(&ts->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(ts->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(ts->vdd);
	if (rc) {
		dev_err(&ts->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(ts->vcc_i2c);
	if (rc) {
		dev_err(&ts->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(ts->vdd);
		if (rc) {
			dev_err(&ts->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}

int gt1x_power_init(struct goodix_ts_data *ts, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	ts->vdd = regulator_get(&ts->client->dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		rc = PTR_ERR(ts->vdd);
		dev_err(&ts->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(ts->vdd) > 0) {
		rc = regulator_set_voltage(ts->vdd, GT1X_VTG_MIN_UV,
					GT1X_VTG_MAX_UV);
		if (rc) {
			dev_err(&ts->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc-i2c");
	if (IS_ERR(ts->vcc_i2c)) {
		rc = PTR_ERR(ts->vcc_i2c);
		dev_err(&ts->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(ts->vcc_i2c) > 0) {
		rc = regulator_set_voltage(ts->vcc_i2c, GT1X_I2C_VTG_MIN_UV,
					   GT1X_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&ts->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(ts->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(ts->vdd) > 0)
		regulator_set_voltage(ts->vdd, 0, GT1X_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(ts->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(ts->vdd) > 0)
		regulator_set_voltage(ts->vdd, 0, GT1X_VTG_MAX_UV);

	regulator_put(ts->vdd);

	if (regulator_count_voltages(ts->vcc_i2c) > 0)
		regulator_set_voltage(ts->vcc_i2c, 0, GT1X_I2C_VTG_MAX_UV);

	regulator_put(ts->vcc_i2c);
	return 0;
}

#endif

/**
 * gt1x_request_io_port - Request gpio(INT & RST) ports.
 */
static s32 gt1x_request_io_port(void)
{
	s32 ret = 0;

	GTP_DEBUG_FUNC();
	ret = gpio_request(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",
			(s32) GTP_INT_PORT, ret);
		ret = -ENODEV;
	} else {
		GTP_GPIO_AS_INT(GTP_INT_PORT);
		gt1x_i2c_client->irq = GTP_INT_IRQ;
	}

	ret = gpio_request(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",
			(s32) GTP_RST_PORT, ret);
		ret = -ENODEV;
	}

	GTP_GPIO_AS_INPUT(GTP_RST_PORT);
	if (ret < 0) {
		gpio_free(GTP_RST_PORT);
		gpio_free(GTP_INT_PORT);
	}

	return ret;
}

/**
 * gt1x_request_irq - Request interrupt.
 * Return
 *	  0: succeed, -1: failed.
 */
static s32 gt1x_request_irq(void)
{
	s32 ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG_FUNC();
	GTP_DEBUG("INT trigger type:%x", gt1x_int_type);

	ret = request_irq(gt1x_i2c_client->irq, gt1x_ts_irq_handler,
			irq_table[gt1x_int_type],
				gt1x_i2c_client->name, gt1x_i2c_client);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		GTP_GPIO_AS_INPUT(GTP_INT_PORT);
		gpio_free(GTP_INT_PORT);

		return -ENODEV;
	} else {
		gt1x_irq_disable();
		return 0;
	}
}

/**
 * gt1x_request_input_dev -  Request input device Function.
 * Return
 *	  0: succeed, -1: failed.
 */
static s8 gt1x_request_input_dev(void)
{
	s8 ret = -1;
#if GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif
#ifdef CONFIG_TOUCHSCREEN_GT1X_GESTURE
	int i = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(gt1x_i2c_client);
#endif

	GTP_DEBUG_FUNC();

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}
	input_dev->evbit[0] = BIT_MASK(EV_SYN) |
		BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#if GTP_ICS_SLOT_REPORT
	input_mt_init_slots(input_dev, 16, INPUT_MT_DIRECT);
#else
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++)
		input_set_capability(input_dev,
		EV_KEY, gt1x_touch_key_array[index]);
#endif

#ifdef CONFIG_TOUCHSCREEN_GT1X_GESTURE
    for (i = 0; i < ts->pdata->gesture_map->nbuttons; i++)
        input_set_capability(input_dev, EV_KEY, ts->pdata->gesture_map->map[i]);
#endif

#if GTP_CHANGE_X2Y
	input_set_abs_params(input_dev,
		ABS_MT_POSITION_X, 0, gt1x_abs_y_max, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_POSITION_Y, 0, gt1x_abs_x_max, 0, 0);
#else
	input_set_abs_params(input_dev,
		ABS_MT_POSITION_X, 0, gt1x_abs_x_max, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_POSITION_Y, 0, gt1x_abs_y_max, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	input_dev->name = gt1x_ts_name;
	input_dev->phys = input_dev_phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xDEAD;
	input_dev->id.product = 0xBEEF;
	input_dev->id.version = 10427;

	ret = input_register_device(input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed", input_dev->name);
		return -ENODEV;
	}

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

#ifdef CONFIG_TOUCHSCREEN_GT1X_GESTURE
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

	if (len == 1) {
		if (gesture[0] == '1')
			data->gesture_state = 0xffff;
		else if (gesture[0] == '0')
			data->gesture_state = 0x0;
	} else if(len == 4) {
		data->gesture_state = asic_to_hex(gesture[0])*0x1000
						+ asic_to_hex(gesture[1]) * 0x100
						+ asic_to_hex(gesture[2]) * 0x10
						+ asic_to_hex(gesture[3]);
	} else {
		GTP_ERROR("[set_gesture_switch]write wrong cmd.");
		return 0;
	}
	if (!data->gesture_state)
		data->gesture_enabled = false;
	else
		data->gesture_enabled = true;

	printk("GTP %s is %x.\n",__func__,data->gesture_state);
	return 0;
}

static bool get_gesture_switch(struct device *dev)
{
	struct goodix_ts_data *data = dev_get_drvdata(dev);

	return data->gesture_enabled;
}

extern st_gesture_data gesture_data; /* gesture data buffer */

static int get_gesture_pos(struct device *dev, char *buf)
{
	char temp[20] = {0};
	char gesture_pos[512] = {0};
	u16 gesture_track_x = 0, gesture_track_y = 0;
	int i = 0;

	for (i = 0; i < gesture_data.data[1]; i++) {
		gesture_track_x = gesture_data.data[4 + i * 4]
			| (gesture_data.data[4 + i * 4 + 1] << 8);
		gesture_track_y = gesture_data.data[4 + i * 4 +2]
			| (gesture_data.data[4 + i * 4 + 3] << 8);
		snprintf(temp, PAGE_SIZE, "%u,%u;", gesture_track_x, gesture_track_y);
		strlcat(gesture_pos, temp, 512);
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", gesture_pos);
}

#endif

static int factory_get_ic_fw_version(struct device *dev, char *buf)
{
	struct goodix_ts_data *data = dev_get_drvdata(dev);
	u8 r_buf[12] = {0};
	s32 ret = 0;

	ret = gt1x_i2c_read_dbl_check(GTP_REG_CONFIG_DATA, &r_buf[0], 1);
	if (ret) {
		GTP_ERROR("get ic_config_version failed,exit");
		return snprintf(buf, PAGE_SIZE, "Failed");
	}
	data->ic_config_version = r_buf[0];

	ret = gt1x_i2c_read_dbl_check(GTP_REG_VERSION,
		r_buf, sizeof(r_buf));
	if (ret) {
		GTP_ERROR("get pid & vid failed,exit");
		return snprintf(buf, PAGE_SIZE, "Failed");
	}
	data->ic_fw_version = r_buf[6] + (r_buf[5] << 8);

	return snprintf(buf, PAGE_SIZE, "0x%04X-%d\n", data->ic_fw_version,
		data->ic_config_version);
}

static int gt1x_get_fw_filename(char *buf, int size)
{
	strlcpy(buf, "/etc/firmware/fw_", size);
	strlcat(buf, CONFIG_HIS_PRODUCT_NAME, size);
	strlcat(buf, "_goodix.bin", size);

	return 0;
}

int factory_get_fs_fw_version(struct device *dev, char *buf)
{
	struct goodix_ts_data *data = dev_get_drvdata(dev);
	s32 ret = 0;
	char fw_path[128] = {0};

	gt1x_get_fw_filename(fw_path, sizeof(fw_path));
	ret = gt1x_update_prepare(fw_path);
	if (ret) {
		GTP_ERROR("Prepare failed.");
		return snprintf(buf, PAGE_SIZE, "Failed");
	}
	ret = gt1x_check_firmware();
	if (ret) {
		GTP_ERROR("check firmware failed.");
		return snprintf(buf, PAGE_SIZE, "Failed");
	}

	return snprintf(buf, PAGE_SIZE, "0x%04X-%d\n", data->fw_version, data->config_version);
}

static int factory_get_module_id(struct device *dev, char *buf)
{
	struct goodix_ts_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", data->sensor_id);
}

static int factory_get_calibration_ret(struct device *dev)
{
	return 1;
}

static int factory_proc_hibernate_test(struct device *dev)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	s32 ret = -1;
	u8 buf[1] = {0};

	disable_irq(ts->client->irq);
	gpio_direction_output(gt1x_rst_gpio, 0);
	mdelay(10);
	ret = gt1x_i2c_read(GTP_REG_CONFIG_DATA, buf, 1);
	if (ret >= 0) {
		GTP_ERROR("IC reset pin is low,but i2c work.");
		ret = 0;
		goto hibernate_exit;
	}

	mdelay(10);
	gt1x_reset_guitar();
	ret = gt1x_i2c_read(GTP_REG_CONFIG_DATA, buf, 1);
	if (ret < 0) {
		GTP_ERROR("IC reset pin is high,but i2c don't work.");
		ret = 0;
		goto hibernate_exit;
	}
	ret = 1;

hibernate_exit:
	msleep(50);
	enable_irq(ts->client->irq);

	return ret;
}

static int factory_get_rawdata_info(struct device *dev, char *buf)
{
	return gt1x_get_rawdatainfo(buf);
}

static int factory_get_rawdata(struct device *dev, char *buf)
{
	return gt1x_get_rawdata(buf);
}

static int factory_short_test(struct device *dev, char *buf)
{
	int ret = 0;
	unsigned char *result;

	init_chip_type();
	result = (unsigned char *)kzalloc(300, GFP_KERNEL);
	if (!result) {
		GTP_ERROR("Failed to alloc memory to result.");
		return 0;
	}
	ret = open_short_test(result);
	if (!ret)
		return 1;
	return 0;
}

static int factory_check_fw_update_need(struct device *dev)
{
	int ret = 0;
	bool need_update = false;
	struct goodix_ts_data *data = dev_get_drvdata(dev);
	char fw_path[128] = {0};
	u8 r_buf[12] = {0};

	gt1x_get_fw_filename(fw_path, sizeof(fw_path));

	ret = gt1x_update_prepare(fw_path);
	if (ret) {
		GTP_ERROR("Prepare failed.");
		return false;
	}

	ret = gt1x_check_firmware();
	if (ret) {
		GTP_ERROR("check firmware failed.");
		return false;
	}

	ret = gt1x_i2c_read_dbl_check(GTP_REG_VERSION,
		r_buf, sizeof(r_buf));
	if (ret) {
		GTP_ERROR("get ic firmware version failed, need update");
		need_update = true;
	} else {
		data->ic_fw_version = r_buf[6] + (r_buf[5] << 8);

		if (data->fw_version > data->ic_fw_version)
			need_update = true;
		else
			need_update = false;
	}

	return need_update;
}

static int factory_proc_fw_update(struct device *dev, bool force)
{
	int ret = 0;
	char fw_path[128] = {0};
	struct goodix_ts_data *data = dev_get_drvdata(dev);

	data->fw_updating = 1;
	if (data->is_suspended) {
		data->fw_updating = 0;
		GTP_ERROR("IC enter suspend.abort update");
		return -EPERM;
	}
	gt1x_get_fw_filename(fw_path, sizeof(fw_path));
	ret = gt1x_update_firmware(fw_path);
	if (ret) {
		GTP_ERROR("Update firmware failed");
		data->fw_updating = 0;
		return ret;
	}
	gt1x_ts_register_productinfo(data);
	data->fw_updating = 0;

	return ret;
}

static int factory_get_fw_update_progress(struct device *dev)
{
	struct goodix_ts_data *data = dev_get_drvdata(dev);
	return data->fw_updating ? 2 : 1;
}

static int factory_proc_fw_bin_update(struct device *dev, const char *buf)
{
	s32 ret = 0;
	unsigned int len;
	char fw_path[128];
	struct goodix_ts_data *data = dev_get_drvdata(dev);

	data->fw_updating = 1;
	if (data->is_suspended) {
		data->fw_updating = 0;
		GTP_ERROR("[factory_proc_fw_bin_update]IC enter suspend.");
		return -EPERM;
	}

	/* Get firmware path */
	strlcpy(fw_path, buf, sizeof(fw_path));
	len = strlen(fw_path);
	if (len > 0) {
		if (fw_path[len-1] == '\n')
			fw_path[len-1] = '\0';
	}

	ret = gt1x_update_firmware(fw_path);
	if (ret) {
		GTP_ERROR("Update firmware failed");
		data->fw_updating = 0;
		return ret;
	}

	gt1x_ts_register_productinfo(data);
		data->fw_updating = 0;

		return ret;
}

static int factory_ts_func_test_register(struct goodix_ts_data *data)
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
	data->ts_test_dev.proc_fw_update_with_given_file = factory_proc_fw_bin_update;

#if defined(CONFIG_TOUCHSCREEN_GT1X_GESTURE)
	data->ts_test_dev.get_gesture_switch = get_gesture_switch;
	data->ts_test_dev.set_gesture_switch = set_gesture_switch;
	data->ts_test_dev.get_gesture_pos = get_gesture_pos;
#endif

	data->ts_test_dev.get_short_test = factory_short_test;
	register_ts_func_test_device(&data->ts_test_dev);
	return 0;
}

void gt1x_ts_register_productinfo(struct goodix_ts_data *ts_data)
{
    /* format as flow: version:0x01 Module id:0x57 */
	char deviceinfo[64];
	u8 r_buf[12] = {0};
	s32 ret = 0;

	ret = gt1x_i2c_read_dbl_check(GTP_REG_CONFIG_DATA, &r_buf[0], 1);
	if (ret) {
		GTP_ERROR("get ic_config_version failed,exit");
		return;
	}
	ts_data->ic_config_version = r_buf[0];

	ret = gt1x_i2c_read_dbl_check(GTP_REG_VERSION, r_buf, sizeof(r_buf));
	if (ret) {
		GTP_ERROR("get pid & vid failed,exit");
		return;
	}
	ts_data->ic_fw_version = r_buf[6] + (r_buf[5] << 8);
	ts_data->sensor_id = r_buf[10] & 0x0F;

	snprintf(deviceinfo, PAGE_SIZE, "FW version:0x%04X-%d Module id:0x%2x",
		ts_data->ic_fw_version, ts_data->ic_config_version, ts_data->sensor_id);

}

/**
 * gt1x_ts_probe -   I2c probe.
 * @client: i2c device struct.
 * @id: device id.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	s32 ret = 0;
	struct goodix_ts_platform_data *pdata;
	struct goodix_ts_data *ts;

	GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

	gt1x_i2c_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct goodix_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			GTP_ERROR("GTP Failed to allocate memory for pdata\n");
			return -ENOMEM;
		}

		ret = gt1x_parse_dt(&client->dev, pdata);
		if (ret)
			return ret;
	} else {
		 pdata = client->dev.platform_data;
	}

	if (!pdata) {
		GTP_ERROR("GTP invalid pdata\n");
		return -EINVAL;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		GTP_ERROR("Alloc GFP_KERNEL memory failed.");
		return -ENOMEM;
	}

	memset(ts, 0, sizeof(*ts));
	INIT_WORK(&ts->work, gt1x_ts_work_func);
	ts->client = client;
	ts->pdata = pdata;
	spin_lock_init(&ts->irq_lock);

	i2c_set_clientdata(client, ts);

	ret = goodix_ts_pinctrl_init(ts);
	if (!ret && ts->ts_pinctrl) {
		ret = goodix_ts_pinctrl_select(ts, true);
		if (ret < 0)
			goto err_ts_pinctrl_select;
		}

	ret = gt1x_request_io_port();
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		goto err_request_io_port;
	}

	ret = gt1x_init();
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		goto err_init;
	}

	ret = gt1x_request_input_dev();
	if (ret < 0) {
		GTP_ERROR("GTP request input dev failed");
		goto err_request_input_dev;
	}

	ret = gt1x_request_irq();
	if (ret < 0) {
		GTP_ERROR("GTP request irq failed.");
		goto err_request_irq;
	}

	gt1x_irq_enable();

#if GTP_ESD_PROTECT
	/*  must before auto update */
	gt1x_init_esd_protect();
	gt1x_esd_switch(SWITCH_ON);
#endif

	gt1x_register_powermanger();

	ret = gtp_test_sysfs_init();
	if (ret) {
		GTP_ERROR("Failed to create test sysfs.");
		goto err_test_sysfs_init;
	}

	factory_ts_func_test_register(ts);
	gt1x_ts_register_productinfo(ts);

	return 0;

err_test_sysfs_init:
	gt1x_unregister_powermanger();
	gt1x_irq_disable();
err_request_irq:
	input_unregister_device(input_dev);
err_request_input_dev:
	gt1x_deinit();
err_init:
	if (gpio_is_valid(GTP_RST_PORT))
		gpio_free(GTP_RST_PORT);
	if (gpio_is_valid(GTP_INT_PORT))
		gpio_free(GTP_INT_PORT);
err_request_io_port:
	if (ts->ts_pinctrl)
		if (goodix_ts_pinctrl_select(ts, false))
			pr_err("Cannot get idle pinctrl state\n");
err_ts_pinctrl_select:
	kfree(ts);
	return ret;

}

/**
 * gt1x_ts_remove -  Goodix touchscreen driver release function.
 * @client: i2c device struct.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	int ret = 0;

	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver removing...");

	gtp_test_sysfs_deinit();
	gt1x_unregister_powermanger();

#ifdef CONFIG_TOUCHSCREEN_GT1X_GESTURE
	disable_irq_wake(client->irq);
#endif
	gt1x_deinit();
	input_unregister_device(input_dev);

	if (ts->ts_pinctrl) {
		ret = goodix_ts_pinctrl_select(ts, false);
		if (ret < 0)
			pr_err("Cannot get idle pinctrl state\n");
	}

	return 0;
}

#if defined(CONFIG_FB)
/* frame buffer notifier block control the suspend/resume procedure */
static struct notifier_block gt1x_fb_notifier;

static int gtp_fb_notifier_callback(struct notifier_block *noti,
	unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	int *blank;

	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
		if (*blank == FB_BLANK_UNBLANK) {
			GTP_DEBUG("Resume by fb notifier.");
			gt1x_resume();
		} else if (*blank == FB_BLANK_POWERDOWN) {
			GTP_DEBUG("Suspend by fb notifier.");
			gt1x_suspend();
		}
	}

	return 0;
}

#if defined(CONFIG_TOUCHSCREEN_GT1X_GESTURE)
static int gt1x_ts_gesture_suspend(struct device *dev)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	if(ts->gesture_wakeup_enable_pre){
		printk("%s. disable irq.\n",__func__);
		disable_irq(ts->client->irq);
		enable_irq_wake(ts->client->irq);
	}
	return 0;
}
static int gt1x_ts_gesture_resume(struct device *dev)
{
	struct goodix_ts_data *ts = dev_get_drvdata(dev);
	if(ts->gesture_wakeup_enable_pre){
		printk("%s. enable irq.\n",__func__);
		disable_irq_wake(ts->client->irq);
		enable_irq(ts->client->irq);
	}
	return 0;
}

static const struct dev_pm_ops gt1x_ts_pm_ops = {
	.suspend = gt1x_ts_gesture_suspend,
	.resume  = gt1x_ts_gesture_resume,
};
#endif

#elif defined(CONFIG_PM)
/**
 * gt1x_ts_suspend - i2c suspend callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_pm_suspend(struct device *dev)
{
	return gt1x_suspend();
}

/**
 * gt1x_ts_resume - i2c resume callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_pm_resume(struct device *dev)
{
	return gt1x_resume();
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops gt1x_ts_pm_ops = {
	.suspend = gt1x_pm_suspend,
	.resume = gt1x_pm_resume,
};

#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* earlysuspend module the suspend/resume procedure */
static void gt1x_ts_early_suspend(struct early_suspend *h)
{
	gt1x_suspend();
}

static void gt1x_ts_late_resume(struct early_suspend *h)
{
	gt1x_resume();
}

static struct early_suspend gt1x_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = gt1x_ts_early_suspend,
	.resume = gt1x_ts_late_resume,
};
#endif


static int gt1x_register_powermanger(void)
{
#if defined(CONFIG_FB)
	gt1x_fb_notifier.notifier_call = gtp_fb_notifier_callback;
	fb_register_client(&gt1x_fb_notifier);

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	register_early_suspend(&gt1x_early_suspend);
#endif
	return 0;
}

static int gt1x_unregister_powermanger(void)
{
#if defined(CONFIG_FB)
	fb_unregister_client(&gt1x_fb_notifier);

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&gt1x_early_suspend);
#endif
	return 0;
}

#ifdef GTP_CONFIG_OF
static const struct of_device_id gt1x_match_table[] = {
	{.compatible = "goodix,gt1x",},
	{ },
};
#endif

static const struct i2c_device_id gt1x_ts_id[] = {
	{GTP_I2C_NAME, 0},
	{}
};

static struct i2c_driver gt1x_ts_driver = {
	.probe = gt1x_ts_probe,
	.remove = gt1x_ts_remove,
	.id_table = gt1x_ts_id,
	.driver = {
		   .name = GTP_I2C_NAME,
		   .owner = THIS_MODULE,
#ifdef GTP_CONFIG_OF
		   .of_match_table = gt1x_match_table,
#endif
#if (defined(CONFIG_FB) && defined(CONFIG_TOUCHSCREEN_GT1X_GESTURE)) || (!defined(CONFIG_FB) && defined(CONFIG_PM))
		   .pm = &gt1x_ts_pm_ops,
#endif
		   },
};

/**
 * gt1x_ts_init - Driver Install function.
 * Return   0---succeed.
 */
static int __init gt1x_ts_init(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver installing...");
	gt1x_wq = create_singlethread_workqueue("gt1x_wq");
	if (!gt1x_wq) {
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}

	return i2c_add_driver(&gt1x_ts_driver);
}

/**
 * gt1x_ts_exit - Driver uninstall function.
 * Return   0---succeed.
 */
static void __exit gt1x_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&gt1x_ts_driver);
	if (gt1x_wq)
		destroy_workqueue(gt1x_wq);
}

module_init(gt1x_ts_init);
module_exit(gt1x_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
