/*
 *
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

#define	S_REGU_DEV_NAME	"s_regu"

struct sns_reg_pdata {
	char *name;
	struct regulator *s_regu;
	u32 min_uv;	/* device allow minimum voltage */
	u32 max_uv;	/* device allow max voltage */
};

struct s_regu_data {
	struct sns_reg_pdata *vddio;
};

struct s_regu_data *sr_data = NULL;

static void s_regu_deinit_regulator(struct sns_reg_pdata *pdata)
{
	if (regulator_count_voltages(pdata->s_regu) > 0) {
		regulator_set_voltage(pdata->s_regu, 0, pdata->max_uv);
		regulator_put(pdata->s_regu);
	}
}

static int s_regu_init_regulator(struct platform_device *dev,
	struct sns_reg_pdata *pdata)
{
	int rc = 0;

	pdata->s_regu = regulator_get(&dev->dev, pdata->name);
	if (IS_ERR(pdata->s_regu)) {
		rc = PTR_ERR(pdata->s_regu);
		dev_err(&dev->dev, "%s regulator get failed, rc = %d\n",
			pdata->name, rc);
		pdata->s_regu = NULL;
	} else {
		if (regulator_count_voltages(pdata->s_regu) > 0) {
			rc = regulator_set_voltage(pdata->s_regu,
				pdata->min_uv, pdata->max_uv);
			if (rc) {
				dev_err(&dev->dev, "%s regulator set failed, rc = %d\n",
					pdata->name, rc);
				regulator_put(pdata->s_regu);
			}
		}
	}

	return rc;
}

static int s_regu_set_regulator(struct platform_device *dev,
	struct sns_reg_pdata *pdata, bool on)
{
	int rc = -EFAULT;

	if (!IS_ERR_OR_NULL(pdata->s_regu)) {
		rc = on ?
			regulator_enable(pdata->s_regu) :
			regulator_disable(pdata->s_regu);
		if (rc) {
			dev_err(&dev->dev, "%s regulator %s failed, rc = %d\n",
				on ? "Enable" : "Disable", pdata->name, rc);
		}
	}
	return rc;
}

#ifdef CONFIG_OF
static int s_regu_parse_dt(struct device *dev, struct s_regu_data *data)
{
	u32 tempval;
	int rc;
	struct device_node *np = dev->of_node;

	rc = of_property_read_u32(np, "linux,vio-max-uv", &tempval);
	if (rc) {
		dev_err(dev, "unable to read vio-max-uv\n");
		return -EINVAL;
	}
	data->vddio->max_uv = tempval;

	rc = of_property_read_u32(np, "linux,vio-min-uv", &tempval);
	if (rc) {
		dev_err(dev, "unable to read vio-min-uv\n");
		return -EINVAL;
	}
	data->vddio->min_uv = tempval;
	data->vddio->name = "vio";

	return 0;
}
#endif

static int s_regu_driver_probe(struct platform_device *dev)
{
	int err = 0;

	dev_err(&dev->dev, "s_regu_driver probe\n");

	sr_data = kzalloc(sizeof(struct s_regu_data), GFP_KERNEL);
	if (sr_data == NULL) {
		dev_err(&dev->dev, "failed to allocate memory %d\n", err);
		return -ENOMEM;
	}

	sr_data->vddio = kzalloc(sizeof(struct sns_reg_pdata), GFP_KERNEL);
	if (sr_data->vddio == NULL) {
		dev_err(&dev->dev, "failed to allocate memory %d\n", err);
		kfree(sr_data);
		return -ENOMEM;
	}
	dev_set_drvdata(&dev->dev, sr_data);

	if (dev->dev.of_node) {
		err = s_regu_parse_dt(&dev->dev, sr_data);
		if (err < 0) {
			dev_err(&dev->dev, "Failed to parse device tree\n");
			goto exit;
		}
	} else {
		dev_err(&dev->dev, "No valid platform data.\n");
		err = -ENODEV;
		goto exit;
	}

	err = s_regu_init_regulator(dev, sr_data->vddio);
	if (err < 0) {
		dev_err(&dev->dev, "Configure power vddio failed: %d\n", err);
		goto err_vddio_init;
	}

	err = s_regu_set_regulator(dev, sr_data->vddio, true);
	if (err < 0) {
		dev_err(&dev->dev, "power on vddio failed: %d\n", err);
		goto err_vddio_set;
	}

	return 0;

err_vddio_set:
	s_regu_deinit_regulator(sr_data->vddio);
err_vddio_init:
exit:
	kfree(sr_data);
	return err;
}

static int s_regu_driver_remove(struct platform_device *dev)
{
	struct s_regu_data *data = dev_get_drvdata(&dev->dev);
	s_regu_set_regulator(dev, data->vddio, false);
	s_regu_deinit_regulator(data->vddio);
	kfree(data);

	return 0;
}

static struct platform_device_id s_regu_id[] = {
	{S_REGU_DEV_NAME, 0 },
	{ },
};

#ifdef CONFIG_OF
static struct of_device_id s_regu_match_table[] = {
	{.compatible = "s_regu", },
	{ },
};
#endif

static struct platform_driver s_regu_driver = {
	.driver = {
		.name = S_REGU_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = s_regu_match_table,
	},
	.probe = s_regu_driver_probe,
	.remove = s_regu_driver_remove,
	.id_table = s_regu_id,
};

static int __init s_regu_init(void)
{
	return platform_driver_register(&s_regu_driver);
}

static void __exit s_regu_exit(void)
{
	platform_driver_unregister(&s_regu_driver);
}

module_init(s_regu_init);
module_exit(s_regu_exit);
MODULE_DESCRIPTION("Sensor Regulator driver");
MODULE_LICENSE("GPL v2");

