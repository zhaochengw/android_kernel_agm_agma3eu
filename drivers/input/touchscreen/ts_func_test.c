/*
 *
 * General TouchScreen Function Test
 *
 * Copyright (c) 2013  Hisense Ltd.
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "ts_func_test.h"

#define MOD_TAG "ts_gen_func_test"

#define gen_printk(level, title, format, arg...)	\
	printk(level "%s: " format , title , ## arg)
#ifdef DEBUG
#define gen_dbg(format, arg...)		\
	gen_printk(KERN_DEBUG , MOD_TAG , format , ## arg)
#else
#define gen_dbg(format, arg...) do { (void)(dev); } while (0)
#endif
#define gen_err(format, arg...)		\
	gen_printk (KERN_ERR , MOD_TAG , format , ## arg)
#define gen_info(dev, format, arg...)		\
	gen_printk (KERN_INFO , MOD_TAG , format , ## arg)
#define gen_warn(dev, format, arg...)		\
	gen_printk (KERN_WARNING , MOD_TAG , format , ## arg)
#define gen_notice(dev, format, arg...)		\
	gen_printk (KERN_NOTICE , MOD_TAG , format , ## arg)

static struct ts_func_test_device *ts_func_test_device = NULL;

void register_ts_func_test_device(struct ts_func_test_device *device)
{
	ts_func_test_device = device;
	__module_get(THIS_MODULE);		
}
EXPORT_SYMBOL(register_ts_func_test_device);

void unregister_ts_func_test_device(struct ts_func_test_device *device)
{
	ts_func_test_device = NULL;
	module_put(THIS_MODULE);
}
EXPORT_SYMBOL(unregister_ts_func_test_device);

static ssize_t fw_state_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	int update_need = 0;
	
	if (test_device && test_device->check_fw_update_need) {
		update_need = test_device->check_fw_update_need(test_device->dev);
	}
	return sprintf(buf, "%d\n", update_need);
}

static ssize_t fw_update_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	int update_progress = 0;

	if (test_device && test_device->get_fw_update_progress) {
		update_progress = test_device->get_fw_update_progress(test_device->dev);
	}
	return sprintf(buf, "%d\n", update_progress);
}

static ssize_t fw_update_store(struct kobject  *obj, struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	
	if (strncmp(buf, "UPDATE", 6)) {
		return -EINVAL;
	}

	if (test_device && test_device->proc_fw_update) {
		test_device->proc_fw_update(test_device->dev, false);
	}
	return size;
}

static ssize_t fw_path_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	char fw_path[64] = {0};

	if (test_device && test_device->get_fw_path) {
		test_device->get_fw_path(test_device->dev, fw_path, sizeof(fw_path));
	}
	return sprintf(buf, "%s\n", fw_path);
}

static ssize_t fw_path_store(struct kobject  *obj, struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	
	if (test_device && test_device->set_fw_path) {
		test_device->set_fw_path(test_device->dev, buf);		
	}
	return size;
}

static ssize_t fw_update_with_appointed_file_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	int update_progress = 0;

	if (test_device && test_device->get_fw_update_progress) {
		update_progress = test_device->get_fw_update_progress(test_device->dev);
	}
	return sprintf(buf, "%d\n", update_progress);
}

static ssize_t fw_update_with_appointed_file_store(struct kobject  *obj, struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	
	if (test_device && test_device->set_fw_path) {
		test_device->set_fw_path(test_device->dev, buf);

		if (test_device && test_device->proc_fw_update) {
			test_device->proc_fw_update(test_device->dev, true);
		}
	}
	else if (test_device && test_device->proc_fw_update_with_given_file) {
		test_device->proc_fw_update_with_given_file(test_device->dev, buf);
	}
	return size;
}

static ssize_t rawdata_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	int ret = 0;

	if (test_device && test_device->get_rawdata) {
		ret = test_device->get_rawdata(test_device->dev, buf);
	}	
	return ret;
}

static ssize_t rawdata_info_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	int ret = 0;

	if (test_device && test_device->get_rawdata_info) {
		ret = test_device->get_rawdata_info(test_device->dev, buf);
	}	
	return ret;
}

static ssize_t hibernate_test_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	int ret = 0;

	if (test_device && test_device->proc_hibernate_test) {
		ret = test_device->proc_hibernate_test(test_device->dev);
	}	
	return sprintf(buf, "%d\n", ret);
}

static ssize_t fw_ic_version_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	int ret = 0;

	if (test_device && test_device->get_ic_fw_version) {
		ret = test_device->get_ic_fw_version(test_device->dev, buf);
	}	
	return ret;
}

static ssize_t fw_fs_version_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	int ret = 0;

	if (test_device && test_device->get_fs_fw_version) {
		ret = test_device->get_fs_fw_version(test_device->dev, buf);
	}	
	return ret;
}

static ssize_t module_id_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	int ret = 0;

	if (test_device && test_device->get_module_id) {
		ret = test_device->get_module_id(test_device->dev, buf);
	}	
	return ret;
}

static ssize_t calibration_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	int ret = 0;

	if (test_device && test_device->get_calibration_ret) {
		ret = test_device->get_calibration_ret(test_device->dev);
	}	
	return sprintf(buf, "%d\n", ret);		
}

static ssize_t need_test_config_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	bool enable = false;
	if (test_device && test_device->need_test_config) {
		enable = test_device->need_test_config(test_device->dev);
		return sprintf(buf, "%d\n", enable? 1: 0);
	} else {
		return sprintf(buf, "%s\n", "NOT SUPPORTED");
	}
}

static ssize_t test_config_path_store(struct kobject  *obj, struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct ts_func_test_device * test_device = ts_func_test_device;

	if (test_device && test_device->set_test_config_path)
	{
		test_device->set_test_config_path(test_device->dev, buf);
		return size;
	}
	return -EINVAL;
}

static ssize_t short_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	bool enable = false;
	if (test_device && test_device->get_short_test) {
		enable = test_device->get_short_test(test_device->dev, buf);
		return sprintf(buf, "%d\n", enable? 1: 0);
	}else{
		return sprintf(buf, "%s\n", "NOT SUPPORTED");
	}
}

static ssize_t open_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	bool enable = false;
	if (test_device && test_device->get_open_test) {
		enable = test_device->get_open_test(test_device->dev, buf);
		return sprintf(buf, "%d\n", enable? 1: 0);
	}else{
		return sprintf(buf, "%s\n", "NOT SUPPORTED");
	}
}

static ssize_t gesture_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	bool enable = false;

	if (test_device && test_device->get_gesture_switch)
	{
		enable = test_device->get_gesture_switch(test_device->dev);
		return sprintf(buf, "%d\n", enable? 1: 0);
	}
	return sprintf(buf, "%s\n", "NOT SUPPORTED");
}

static ssize_t gesture_store(struct kobject  *obj, struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct ts_func_test_device * test_device = ts_func_test_device;

	if (test_device && test_device->set_gesture_switch)
	{
		test_device->set_gesture_switch(test_device->dev, buf);
		return size;
	}
	return -EINVAL;
}

static ssize_t tp_enable_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	bool enable = false;

	if (test_device && test_device->get_tp_enable_switch)
	{
		enable = test_device->get_tp_enable_switch(test_device->dev);
		return sprintf(buf, "%d\n", enable? 1: 0);
	}
	return sprintf(buf, "%s\n", "NOT SUPPORTED");
}

static ssize_t tp_enable_store(struct kobject  *obj, struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct ts_func_test_device * test_device = ts_func_test_device;

	if (test_device && test_device->set_tp_enable_switch)
	{
		if (strncmp(buf, "0", 1)) {
			test_device->set_tp_enable_switch(test_device->dev, true);
		} else {
			test_device->set_tp_enable_switch(test_device->dev, false);
		}
		return size;
	}

	return -EINVAL;
}


static ssize_t gesture_pos_show(struct kobject  *obj, struct kobj_attribute *attr, char *buf)
{
	struct ts_func_test_device * test_device = ts_func_test_device;
	int ret = 0;

	if (test_device && test_device->get_gesture_pos) {
		ret = test_device->get_gesture_pos(test_device->dev, buf);
	}
	return ret;
}


static struct kobj_attribute fw_state_attr = {
	.attr = {
		.name = "fwstate",
		.mode = S_IRUGO,
	},
	.show = fw_state_show,
};

static struct kobj_attribute fw_update_attr = {
	.attr = {
		.name = "fwupdate",
		.mode = S_IRUGO|S_IWUSR,
	},
	.show = fw_update_show,
	.store = fw_update_store,
};

static struct kobj_attribute fw_path_attr = {
	.attr = {
		.name = "fwpath",
		.mode = S_IRUGO|S_IWUSR,
	},
	.show = fw_path_show,
	.store = fw_path_store,
};

static struct kobj_attribute fw_update_with_appointed_file_attr = {
	.attr = {
		.name = "fwbinupdate",
		.mode = S_IRUGO|S_IWUSR,
	},
	.show = fw_update_with_appointed_file_show,
	.store = fw_update_with_appointed_file_store,
};

static struct kobj_attribute rawdata_attr = {
	.attr = {
		.name = "rawdatashow",
		.mode = S_IRUGO,
	},
	.show = rawdata_show,
};

static struct kobj_attribute rawdata_info_attr = {
	.attr = {
		.name = "rawdatainfo",
		.mode = S_IRUGO,
	},
	.show = rawdata_info_show,
};

static struct kobj_attribute hibernate_test_attr = {
	.attr = {
		.name = "resettest",
		.mode = S_IRUGO,
	},
	.show = hibernate_test_show,
};

static struct kobj_attribute fw_ic_version_attr = {
	.attr = {
		.name = "fwversion",
		.mode = S_IRUGO,
	},
	.show = fw_ic_version_show,
};

static struct kobj_attribute fw_fs_version_attr = {
	.attr = {
		.name = "fwhostversion",
		.mode = S_IRUGO,
	},
	.show = fw_fs_version_show,
};

static struct kobj_attribute module_id_attr = {
	.attr = {
		.name = "fwmoduleid",
		.mode = S_IRUGO,
	},
	.show = module_id_show,
};

static struct kobj_attribute calibration_attr = {
	.attr = {
		.name = "caltest",
		.mode = S_IRUGO,
	},
	.show = calibration_show,
};

static struct kobj_attribute test_config_attr = {
	.attr = {
		.name = "testconfig",
		.mode = S_IRUGO|S_IWUSR,
	},
	.show = need_test_config_show,
	.store = test_config_path_store,
};

static struct kobj_attribute short_attr = {
	.attr = {
		.name = "shorttest",
		.mode = S_IRUGO,
	},
	.show = short_show,
};

static struct kobj_attribute open_attr = {
	.attr = {
		.name = "opentest",
		.mode = S_IRUGO,
	},
	.show = open_show,
};

static struct kobj_attribute gesture_attr = {
	.attr = {
		.name = "gesture",
		.mode = S_IRUGO|S_IWUSR,
	},
	.show = gesture_show,
	.store = gesture_store,
};

static struct kobj_attribute tp_enable_attr = {
	.attr = {
		.name = "tpenable",
		.mode = S_IRUGO|S_IWUSR,
	},
	.show = tp_enable_show,
	.store = tp_enable_store,
};

static struct kobj_attribute gesture_pos_attr = {
	.attr = {
		.name = "gesturepos",
		.mode = S_IRUGO|S_IWUSR,
	},
	.show = gesture_pos_show,
};

static struct attribute* ts_func_ctp_func_attr[] ={
	&gesture_attr.attr,
	&gesture_pos_attr.attr,
	&tp_enable_attr.attr,
	NULL,
};

static struct attribute *ts_func_ctp_update_attr[] = {
	&fw_state_attr.attr,
	&fw_update_attr.attr,
	&fw_path_attr.attr,
	&fw_update_with_appointed_file_attr.attr,
	NULL
};

static struct attribute* ts_func_ctp_test_attr[] ={
	&rawdata_attr.attr,
	&rawdata_info_attr.attr,
	&hibernate_test_attr.attr,
	&fw_ic_version_attr.attr,
	&fw_fs_version_attr.attr,
	&module_id_attr.attr,
	&calibration_attr.attr,
	&open_attr.attr,
	&short_attr.attr,
	&test_config_attr.attr,
	NULL,
};

static struct attribute_group ts_func_ctp_update_grp = {
	.attrs = ts_func_ctp_update_attr,
};

static struct attribute_group ts_func_ctp_test_grp = {
	.attrs = ts_func_ctp_test_attr,
};


static struct attribute_group ts_func_ctp_func_grp = {
	.attrs = ts_func_ctp_func_attr,
};

static struct kobject *ts_func_test_obj;
static struct kobject *ts_func_ctp_test_obj;
static struct kobject *ts_func_ctp_update_obj;
static struct kobject *ts_func_ctp_func_obj;

static int __init ts_gen_func_test_init(void)
{
	int ret = 0;

	ts_func_test_obj = kobject_create_and_add("ctp", NULL);
	if (!ts_func_test_obj) {
		gen_err("unable to create kobject\n");
		return -ENOMEM;
	}

	ts_func_ctp_test_obj = kobject_create_and_add("ctp_test", ts_func_test_obj);
	if (!ts_func_ctp_test_obj) {
		gen_err("unable to create kobject-ts_func_test_obj\n");
		goto destroy_test_obj;
	}

	ts_func_ctp_update_obj = kobject_create_and_add("ctp_update", ts_func_test_obj);
	if (!ts_func_ctp_update_obj) {
		gen_err("unable to create kobject-ts_func_test_obj\n");
		goto destroy_ctp_test_obj;
	}


	ts_func_ctp_func_obj = kobject_create_and_add("ctp_func", ts_func_test_obj);
	if (!ts_func_ctp_func_obj) {
		gen_err("unable to create kobject-ts_func_ctp_func_obj\n");
		goto destroy_ctp_func_obj;
	}

	ret = sysfs_create_group(ts_func_ctp_test_obj, &ts_func_ctp_test_grp);
	if (ret) {
		gen_err("failed to create attributes- ts_func_ctp_test_grp\n");
		goto destroy_ctp_update_obj;
	}

	ret = sysfs_create_group(ts_func_ctp_update_obj, &ts_func_ctp_update_grp);
	if (ret) {
		gen_err("failed to create attributes- ts_func_ctp_update_grp\n");
		goto remove_ctp_test_grp;
	}


	ret = sysfs_create_group(ts_func_ctp_func_obj, &ts_func_ctp_func_grp);
	if (ret) {
		gen_err("failed to create attributes- ts_func_ctp_func_grp\n");
		goto remove_ctp_func_grp;
	}

	return 0;

remove_ctp_func_grp:
	sysfs_remove_group(ts_func_test_obj, &ts_func_ctp_func_grp);
remove_ctp_test_grp:
	sysfs_remove_group(ts_func_test_obj, &ts_func_ctp_test_grp);
destroy_ctp_update_obj:
	kobject_put(ts_func_ctp_update_obj);
destroy_ctp_func_obj:
	kobject_put(ts_func_ctp_func_obj);
destroy_ctp_test_obj:
	kobject_put(ts_func_ctp_test_obj);
destroy_test_obj:
	kobject_put(ts_func_test_obj);
	return ret;
}
module_init(ts_gen_func_test_init);

static void __exit ts_gen_func_test_exit(void)
{
	sysfs_remove_group(ts_func_test_obj, &ts_func_ctp_func_grp);
	sysfs_remove_group(ts_func_test_obj, &ts_func_ctp_test_grp);
	sysfs_remove_group(ts_func_test_obj, &ts_func_ctp_update_grp);
	kobject_put(ts_func_ctp_func_obj);
	kobject_put(ts_func_ctp_test_obj);
	kobject_put(ts_func_ctp_update_obj);
	kobject_put(ts_func_test_obj);
}
module_exit(ts_gen_func_test_exit);

MODULE_DESCRIPTION("General TouchScreen function test");
MODULE_LICENSE("GPL v2");
