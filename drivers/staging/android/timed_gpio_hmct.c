/* drivers/misc/timed_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include "timed_output.h"
#include "timed_gpio.h"


struct timed_gpio_data {
	struct timed_output_dev dev;
	struct hrtimer timer;
	spinlock_t lock;
	unsigned gpio;
	int max_timeout;
	u8 active_low;
};

static enum hrtimer_restart gpio_timer_func(struct hrtimer *timer)
{
	struct timed_gpio_data *data =
		container_of(timer, struct timed_gpio_data, timer);

	gpio_direction_output(data->gpio, data->active_low ? 1 : 0);
	return HRTIMER_NORESTART;
}

static int gpio_get_time(struct timed_output_dev *dev)
{
	struct timed_gpio_data	*data =
		container_of(dev, struct timed_gpio_data, dev);

	if (hrtimer_active(&data->timer)) {
		ktime_t r = hrtimer_get_remaining(&data->timer);
		struct timeval t = ktime_to_timeval(r);

		return t.tv_sec * 1000 + t.tv_usec / 1000;
	} else
		return 0;
}

static void gpio_enable(struct timed_output_dev *dev, int value)
{
	struct timed_gpio_data	*data =
		container_of(dev, struct timed_gpio_data, dev);
	unsigned long	flags;

	spin_lock_irqsave(&data->lock, flags);

	/* cancel previous timer and set GPIO according to value */
	hrtimer_cancel(&data->timer);
	gpio_direction_output(data->gpio, data->active_low ? !value : !!value);

	if (value > 0) {
		if (value > data->max_timeout)
			value = data->max_timeout;

		hrtimer_start(&data->timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&data->lock, flags);
}

static int timed_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct timed_gpio *cur_gpio;
	struct timed_gpio_data *gpio_data, *gpio_dat;
	int ret;

	gpio_data = kzalloc(sizeof(struct timed_gpio_data) , GFP_KERNEL);
	if (!gpio_data)
		return -ENOMEM;

	cur_gpio = kzalloc(sizeof(struct timed_gpio) , GFP_KERNEL);
	if (!cur_gpio)
		return -ENOMEM;

	cur_gpio->gpio = of_get_named_gpio(np, "vib-gpio", 0);
	cur_gpio->name = "vib-gpio";
	cur_gpio->max_timeout = 15000;
	cur_gpio->active_low = 0;

	pr_info("cur_gpio->gpio is %d, cur_gpio->name is %s\n", cur_gpio->gpio, cur_gpio->name);

	gpio_dat = gpio_data;

	hrtimer_init(&gpio_dat->timer, CLOCK_MONOTONIC,
				HRTIMER_MODE_REL);
	gpio_dat->timer.function = gpio_timer_func;
	spin_lock_init(&gpio_dat->lock);

	gpio_dat->dev.name = "vibrator";
	gpio_dat->dev.get_time = gpio_get_time;
	gpio_dat->dev.enable = gpio_enable;
	ret = gpio_request(cur_gpio->gpio, cur_gpio->name);
	if (ret < 0)
		{
		pr_info("gpio_request error, ret is %d\n", ret);
		goto err_out;
		}
	ret = timed_output_dev_register(&gpio_dat->dev);
	if (ret < 0) {
		pr_info("timed_output_dev_register error ret is %d\n", ret);
		gpio_free(cur_gpio->gpio);
		goto err_out;
	}

	gpio_dat->gpio = cur_gpio->gpio;
	gpio_dat->max_timeout = cur_gpio->max_timeout;
	gpio_dat->active_low = cur_gpio->active_low;
	gpio_direction_output(gpio_dat->gpio, gpio_dat->active_low);
	

	platform_set_drvdata(pdev, gpio_data);

	pr_info("timed_gpio_probe done \n");

	return 0;

err_out:
	timed_output_dev_unregister(&gpio_data->dev);
	gpio_free(gpio_data->gpio);
	
	kfree(gpio_data);
	kfree(cur_gpio);

	return ret;
}

static const struct of_device_id of_timed_gpio[] = {
	{ .compatible = "timed-gpio", },
	{},
};

static struct platform_driver timed_gpio_driver = {
	.probe		= timed_gpio_probe,
	.driver		= {
		.name		= TIMED_GPIO_NAME,
		.of_match_table = of_match_ptr(of_timed_gpio),
		.owner		= THIS_MODULE,
	},
};

module_platform_driver(timed_gpio_driver);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("timed gpio driver");
MODULE_LICENSE("GPL");
