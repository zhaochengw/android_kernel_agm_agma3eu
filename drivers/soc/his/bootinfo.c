/*
 * Copyright (C) 2008-2014 Hisense, Inc.
 *
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <asm/setup.h>

#define POWERON_REASON_LEN          20

int meid_is_null_flag = 1;
EXPORT_SYMBOL_GPL(meid_is_null_flag); 

int driver_log_print_flag = 0;
EXPORT_SYMBOL_GPL(driver_log_print_flag); 

/*Add for sensor ADSP mode*/
uint8_t adsp_mode = 0;
EXPORT_SYMBOL_GPL(adsp_mode); 

char poweron_reason[POWERON_REASON_LEN];
int boot_charger_status;
EXPORT_SYMBOL_GPL(boot_charger_status); 

int boot_ftm_mode;
int boot_recovery_mode;
EXPORT_SYMBOL_GPL(boot_recovery_mode); 
EXPORT_SYMBOL_GPL(boot_ftm_mode); 

#define BOOTINFO_ATTR(_name) \
static struct kobj_attribute _name##_attr = { \
	.attr   = {	                              \
		.name = __stringify(_name),	          \
		.mode = 0444,                         \
	},                                        \
	.show   = _name##_show,	                  \
	.store  = NULL,	                          \
}

static int __init powerup_reason_setup(char *p)
{
	strlcpy(poweron_reason, p, POWERON_REASON_LEN);
	pr_err("%s: poweron_reason = %s\n", __func__, poweron_reason);

	return 0;
}
early_param("powerup_reason", powerup_reason_setup);

static ssize_t powerup_reason_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, POWERON_REASON_LEN, "%s\n", poweron_reason);

	return ret;
}
BOOTINFO_ATTR(powerup_reason);


static int __init meid_status_setup(char *p)
{
	if (!strcmp(p, "0"))
		meid_is_null_flag = 1;
	else
		meid_is_null_flag = 0;

	pr_err("%s: meid_status = %d\n", __func__, meid_is_null_flag);

	return 0;
}
early_param("androidboot.meid", meid_status_setup);

static ssize_t meid_is_null_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	char *meid_is_null_text[] = {
		"is_null",
		"is_not_null",
	};

	if (meid_is_null_flag)
		s += snprintf(s, 15, "%s", meid_is_null_text[0]);
	else
		s += snprintf(s, 15, "%s", meid_is_null_text[1]);

	return s-buf;
}
BOOTINFO_ATTR(meid_is_null);

int __init dirver_log_print_setup(char *s)
{
	if (!strcmp(s, "initlog"))
	{
		driver_log_print_flag = 1;
	}
	printk("driver_init_log_print_flag = %d\n",driver_log_print_flag);
	return 0;
}

__setup("poweron_debug=", dirver_log_print_setup);

int __init boot_charger_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		boot_charger_status = 1;

	if (!strcmp(s, "factory2"))
		boot_ftm_mode = 1;
		
	if (!strcmp(s, "recovery"))
		boot_recovery_mode = 1;

	pr_err("%s: buffer = %s boot_charger_status = %d, boot_ftm_mode = %d , boot_recovery_mode = %d\n",
		__func__, s, boot_charger_status, boot_ftm_mode, boot_recovery_mode);

	return 1;
}
__setup("androidboot.mode=", boot_charger_mode_init);


/*Add for sensor ADSP mode*/
int __init adsp_mode_setup(char *s)
{
	if (!strcmp(s, "1"))
		adsp_mode = 1;
	else
		adsp_mode = 0;

	printk("adsp_mode = %d\n",adsp_mode);
	return 0;
}
__setup("ADSPMode=", adsp_mode_setup);


static struct attribute *bootattr[] = {
	&powerup_reason_attr.attr,
	&meid_is_null_attr.attr,
	NULL,
};

static struct attribute_group bootattr_group = {
	.attrs = bootattr,
};

static int __init bootinfo_init(void)
{
	int ret = -ENOMEM;
	struct kobject *bootinfo_kobj = NULL;

	bootinfo_kobj = kobject_create_and_add("bootinfo", NULL);
	if (bootinfo_kobj == NULL) {
		pr_err("bootinfo_init: subsystem_register failed\n");
		return ret;
	}

	ret = sysfs_create_group(bootinfo_kobj, &bootattr_group);
	if (ret) {
		pr_err("bootinfo_init: subsystem_register failed\n");
		goto sys_fail;
	}

	return ret;

sys_fail:
	kobject_del(bootinfo_kobj);
	return ret;
}
core_initcall(bootinfo_init);

