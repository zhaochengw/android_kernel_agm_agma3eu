
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/compiler.h>
#include <linux/timex.h>
#include <linux/rtc.h>

static struct platform_device *report_dev;

#define REPORT_ATTR(_name) \
	static struct kobj_attribute _name##_attr = {   \
		.attr = {                           \
			.name = __stringify(_name),     \
			.mode = 0644,                   \
		},                                  \
		.show   = _name##_show,             \
		.store  = _name##_store,            \
	}

void subsystem_report(const char *subsys_name, const char *err_log)
{
	static char subsys_log[512];
	char *envp[2];
	struct timex txc;
	struct rtc_time tm;
	char time_str[64] = {0};
	int ret;

	do_gettimeofday(&(txc.time));
	rtc_time_to_tm(txc.time.tv_sec, &tm);
	snprintf(time_str, sizeof(time_str), "%d-%d-%d_%d:%d:%d",
		tm.tm_year + 1900,
		tm.tm_mon+1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec);

	snprintf(subsys_log, sizeof(subsys_log), "ERR_REPORT=\"%s: %s %s\"",
			subsys_name, time_str, err_log);
	pr_err("the subsys_log is %s\n", subsys_log);

	envp[0] = subsys_log;
	envp[1] = NULL;
	ret = kobject_uevent_env(&report_dev->dev.kobj, KOBJ_CHANGE, envp);
	if (ret)
		pr_err("kobject_uevent_env failed: %d\n", ret);
}
EXPORT_SYMBOL(subsystem_report);

static ssize_t test_report_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t test_report_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t n)
{
	static int test_time;
	char test_buf[30] = {0};

	snprintf(test_buf, sizeof(test_buf), "dump time is %d", test_time++);
	subsystem_report("test", (const char *)test_buf);

	return n;
}
REPORT_ATTR(test_report);

static struct attribute *test_attrs[] = {
	&test_report_attr.attr,
	NULL,
};

static struct attribute_group test_attr_group = {
	.attrs = test_attrs,
};

static int subsys_err_report_probe(struct platform_device *pdev)
{
	int ret = 0;

	report_dev = pdev;

	ret = sysfs_create_group(&pdev->dev.kobj, &test_attr_group);
	if (ret)
		pr_err("subsys_err_report register attr failed\n");

	return ret;
}

static int subsys_err_report_remove(struct platform_device *pdev)
{
	int ret = 0;
	sysfs_remove_group(&pdev->dev.kobj, &test_attr_group);
	return ret;
}

static struct platform_device subsys_err_report_dev = {
	.name = "subsys_err_report",
	.id = -1,
};

static struct platform_driver subsys_err_report_driver = {
	.probe = subsys_err_report_probe,
	.remove = subsys_err_report_remove,
	.driver = {
		.name = "subsys_err_report",
	},
};

static int __init subsys_err_report_init(void)
{
	int ret;

	ret = platform_device_register(&subsys_err_report_dev);
	if (ret) {
		pr_err("%s: register device error\n", __func__);
		return ret;
	}

	ret = platform_driver_register(&subsys_err_report_driver);
	if (ret) {
		platform_device_unregister(&subsys_err_report_dev);
		pr_err("%s: register driver error\n", __func__);
		return ret;
	}

	return ret;
}

static void __exit subsys_err_report_exit(void)
{
	platform_driver_unregister(&subsys_err_report_driver);
	platform_device_unregister(&subsys_err_report_dev);
	return;
}

module_init(subsys_err_report_init);
module_exit(subsys_err_report_exit);
