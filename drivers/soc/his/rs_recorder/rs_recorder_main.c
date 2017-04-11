/*
 * rs_recorder (Runtime State Recorder) Module
 *
 * Copyright (C) 2015 Hisense, Inc.
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

#define pr_fmt(fmt) "rs-recorder: " fmt

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include <linux/export.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/productinfo.h>
#include "rs_common.h"

#define ERRINFO_BUF_SIZE         (128*1024)

static int rs_dump_common_init(struct rs_recorder_log *, char *);
static int rs_dump_dev_errinfo(struct rs_recorder_log *, char *);

struct errinfo_recorder {
	u32 w_pos;
	char buf[ERRINFO_BUF_SIZE];
};

static struct errinfo_recorder dev_errinfo;

/* global recorder buf data */
static struct rs_recorder_log s_record_buf;

static struct rs_dump_info rs_dump_info;
static struct rs_common_info g_common_info;
static struct work_struct dump_work;
static struct work_struct save_errinfo_work;
static DEFINE_MUTEX(rs_mutex_lock);
static int rs_recorder_ready = 0;

struct rs_module_ops {
	char *name;
	int level;
	char *filename;
	int (*dump_log)(struct rs_recorder_log *rs_log, char *fname);
};

struct rs_module_ops rs_dump_ops_set[] = {
	/* Name           Lvl    filename           ops */
	{"common",        1,     NULL,              rs_dump_common_init},
	{"blocked_task",  1,     NULL,              rs_print_blocked_task},
	{"meminfo",       1,     "meminfo.txt",     rs_dump_meminfo},
	{"dev_errinfo",   1,     "dev_errinfo.txt", rs_dump_dev_errinfo},
	{"dmesg",         1,     "dmesg.txt",       rs_dump_dmesg_log},
	{"ps_info",       2,     "ps.txt",          rs_dump_allps_info},
	{"interrupts",    2,     "irqs.txt",        rs_dump_interrupts},
	{"slabinfo",      3,     "slabinfo.txt",    rs_dump_all_slabinfo},
#ifdef CONFIG_DEBUG_KMEMLEAK
	{"kmemleak",      3,     "kmemleak.txt",    rs_dump_kmemleak}
#endif
};

static char * rs_get_product_name(void)
{
	return (char *)CONFIG_HIS_PRODUCT_NAME;
}

static void rs_get_kernel_ver(char *buf)
{
	snprintf(buf, PAGE_SIZE, "%s%s%s", utsname()->sysname,
		utsname()->release, utsname()->version);
}

static void rs_get_productinfo(char *buf)
{
	productinfo_dump(buf, 0);
}

static void rs_get_current_time(unsigned long *rem_nsec, u64 *ts)
{
	if (rem_nsec == NULL || ts == NULL)
		BUG();

	*ts = local_clock();

	*rem_nsec = do_div(*ts, 1000000000);
}

void save_dev_errinfo(const char *fmt, ...)
{
	static char log_buf[512];
	int log_len;
	int remain;
	va_list args;
	unsigned long rem_nsec;
	char * buff;
	u64 ts;

	rs_get_current_time(&rem_nsec, &ts);
	log_len = snprintf(log_buf, sizeof(log_buf), "[%5lu.%06lu] ",
			(unsigned long)ts, rem_nsec / 1000);

	va_start(args, fmt);
	log_len += vscnprintf(log_buf+log_len, sizeof(log_buf), fmt, args);
	va_end(args);

	buff = dev_errinfo.buf + dev_errinfo.w_pos;
	memcpy(buff, log_buf, log_len);
	dev_errinfo.w_pos += log_len;

	remain = ERRINFO_BUF_SIZE - dev_errinfo.w_pos;
	if (remain < 512)
		schedule_work(&save_errinfo_work);
}
EXPORT_SYMBOL(save_dev_errinfo);

static void save_dev_errinfo_work(struct work_struct *work)
{
	int ret = 0;
	static int index = 0;
	char fname[32] = {0};

	snprintf(fname, sizeof(fname), "dev_errinfo%d.txt", index++);
	ret = rs_create_dir(RS_DUMP_PATH);
	rs_save_file(RS_DUMP_PATH, fname, dev_errinfo.buf,
			dev_errinfo.w_pos);
	dev_errinfo.w_pos = 0;

	return;
}

void rs_update_buf_header(struct rs_recorder_log *log, char *fname, u32 len)
{
	struct rs_buf_header *pheader;

	mutex_lock(&rs_mutex_lock);
	pheader = (struct rs_buf_header *)log->head_ptr;

	memset(pheader->items[pheader->num].filename, 0, RS_ARCHIVE_FNAME_LEN);
	memcpy(pheader->items[pheader->num].filename, fname, strlen(fname));
	pheader->items[pheader->num].off = log->w_pos;
	pheader->items[pheader->num].orig_len = len;
	pheader->num ++;
	pr_err("the '%s' offset is %d, len is %d\n", fname, log->w_pos, len);

	log->w_pos += len;
	log->left  -= len;
	mutex_unlock(&rs_mutex_lock);
}

static int rs_dump_common_init(struct rs_recorder_log *log, char *fname)
{
	int ret = 0;
	u32 len = 0;
	struct rs_buf_header *pheader;
	char * pbuf = log->log_buf;
	char * vstr = "version.txt";
	char * pinfo = "productinfo.txt";

	pheader = (struct rs_buf_header *)log->head_ptr;
	if (!log->is_panic) {
		ret = rs_create_dir(log->path);

		rs_save_file(log->path, vstr, g_common_info.kernel_ver,
			strlen(g_common_info.kernel_ver));
		rs_save_file(log->path, pinfo, g_common_info.productinfo,
			strlen(g_common_info.productinfo));
	} else {
		len = strlen(g_common_info.kernel_ver);
		memcpy(pbuf + log->w_pos, g_common_info.kernel_ver, len);
		rs_update_buf_header(log, vstr, len);

		len = strlen(g_common_info.productinfo);
		memcpy(pbuf + log->w_pos, g_common_info.productinfo, len);
		rs_update_buf_header(log, pinfo, len);
	}

	return 0;
}

static int rs_dump_dev_errinfo(struct rs_recorder_log *log, char *fname)
{
	char * buff;
	u32 data_len = dev_errinfo.w_pos;

	if (!log->is_panic) {
		rs_save_file(log->path, fname, dev_errinfo.buf, data_len);
	} else {
		buff = log->log_buf + log->w_pos;

		memcpy(buff, dev_errinfo.buf, data_len);
		rs_update_buf_header(log, fname, data_len);
	}

	return 0;
}

void rs_recorder_dump_log(char *reason, int level, int line, int is_panic)
{
	int i = 0;
	int ops_size = 0;
	char time[32] = {0};
	unsigned long rem_nsec;
	char fname[RS_ARCHIVE_FNAME_LEN] = {0};
	struct rs_buf_header *pheader;
	u64 ts;

	mutex_lock(&rs_mutex_lock);
	if (rs_recorder_ready == 0) {
		mutex_unlock(&rs_mutex_lock);
		return;
	}
	rs_recorder_ready = 0;

	s_record_buf.w_pos = 0;
	s_record_buf.left = s_record_buf.buf_size;

	s_record_buf.log_dump_pos = line;
	s_record_buf.is_panic = is_panic;
	mutex_unlock(&rs_mutex_lock);

	pr_info("enter dump log, reason:%s,level:%d\n", reason, level);
	/* init path by reason and time */
	rs_get_current_time(&rem_nsec, &ts);
	snprintf(time, sizeof(time), "%lu.%lu", (unsigned long)ts,
			rem_nsec / 1000);

	snprintf(fname, sizeof(fname), "%s_%s", reason, time);
	snprintf(s_record_buf.path, RS_LOG_PATH_LEN, "%s%s/",
			RS_DUMP_PATH, fname);

	pheader = (struct rs_buf_header *)s_record_buf.head_ptr;
	if (is_panic)
		pheader->magic = RS_LOG_PANIC_MAGIC;
	else
		pheader->magic = RS_LOG_GENERIC_MAGIC;

	memcpy(pheader->path, s_record_buf.path, strlen(s_record_buf.path));
	pheader->num = 0;

	ops_size = sizeof(rs_dump_ops_set)/sizeof(rs_dump_ops_set[0]);
	for (i = 0; i < ops_size; i++) {
		if (rs_dump_ops_set[i].level <= level) {
			char * fname = rs_dump_ops_set[i].filename;
			rs_dump_ops_set[i].dump_log(&s_record_buf, fname);
		}
	}

	if (!is_panic) {
#ifdef CONFIG_RS_RECORDER_ARCHIVE
		create_archive_file(s_record_buf.path, fname);
		rs_remove_whole_dir(s_record_buf.path);
		rs_rm_file_bylimit(RS_DUMP_PATH);
#endif
	}

	rs_recorder_ready = 1;
}

static int rs_check_dump_thread(void * args)
{
	int ret = 0;
	int head_sz = 0;
	struct rs_buf_header *pheader;
	int buf_sz;

	head_sz = sizeof(struct rs_buf_header);
	pheader = (struct rs_buf_header *)s_record_buf.head_ptr;

	pr_info("wait fs ready start\n");
	while (rs_wait_fs_ready() > 0);
	pr_info("wait fs ready end\n");

#ifndef CONFIG_SUPPORT_WARM_RESET
	buf_sz = s_record_buf.buf_size + head_sz;

	ret = rs_read_file(RS_LOG_PATITION, 0, (char *)pheader, buf_sz);
	pr_info("the buf_size %d, read size is %d\n", buf_sz, ret);
	if (ret < 0)
		goto exit;
#endif /* CONFIG_SUPPORT_WARM_RESET */

	pr_info("the header magic is 0x%x\n", pheader->magic);
	ret = -EINVAL;
	if (pheader->magic != RS_LOG_PANIC_MAGIC) {
		pr_info("no panic dump!!\n");
		goto exit;
	}

	ret = rs_save_dump_to_file(pheader);

exit:
	/* delete information after saved */
	memset((char *)pheader, 0, head_sz);
#ifndef CONFIG_SUPPORT_WARM_RESET
	if (ret == 0)
		rs_write_file(RS_LOG_PATITION, 0, (char *)pheader, head_sz);
#endif /* CONFIG_SUPPORT_WARM_RESET */

	rs_recorder_ready = 1;

	return 0;
}

static void dump_kernel_info_work(struct work_struct *work)
{
	rs_recorder_dump_log(rs_dump_info.reason, rs_dump_info.level,
			rs_dump_info.line, rs_dump_info.is_panic);

	memset((char *)&rs_dump_info, 0, sizeof(struct rs_dump_info));
}

void rs_exec_dump_task(char *reason, int level, int line, int is_panic)
{
	memcpy(rs_dump_info.reason, reason, strlen(reason));
	rs_dump_info.level = level;
	rs_dump_info.line = line;
	rs_dump_info.is_panic = is_panic;

	schedule_work(&dump_work);
}

#ifdef CONFIG_DEBUG_FS
static int dev_errinfo_show(struct seq_file *m, void *v)
{
	int size = dev_errinfo.w_pos;
	char *msg_str = dev_errinfo.buf;

	seq_write(m, msg_str, size);

	return 0;
}

static int dev_errinfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, dev_errinfo_show, NULL);
}

static const struct file_operations dev_errinfo_proc_fops = {
	.open       = dev_errinfo_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static int manual_dumplog_show(struct seq_file *m, void *v)
{
	char * show_msg = "use 'echo 1 > manual_dump' to dump.\n";
	seq_write(m, show_msg, strlen(show_msg));

	return 0;
}

static ssize_t manual_dump_write(struct file *file, const char __user *data,
							size_t size, loff_t *offset)
{
	int value;
	char buf[4];

	if (size > sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, data, size))
		return -EINVAL;

	sscanf(buf, "%u\n", &value);
	if (value == 1)
		rs_exec_dump_task("manual_dump", 3, __LINE__, false);
	else if (2 == value)
		rs_rm_file_bylimit(RS_DUMP_PATH);

	return size;
}

static int test_dump_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, manual_dumplog_show, NULL);
}

static const struct file_operations dev_manual_dump_fops = {
	.open       = test_dump_proc_open,
	.read       = seq_read,
	.write      = manual_dump_write,
	.llseek     = seq_lseek,
	.release    = single_release,
};
#endif

static int rs_recorder_parse_dt(struct device *dev, u32 *data)
{
	struct device_node *pnode;
	const u32 *addr;
	u64 size;

	pnode = of_parse_phandle(dev->of_node, "linux,contiguous-region", 0);
	if (pnode == NULL) {
		pr_err("mem reservation for rs-recorder not present\n");
		return -EINVAL;
	}

	addr = of_get_address(pnode, 0, &size, NULL);
	if (!addr) {
		pr_err("failed to parse the reserve memory address\n");
		of_node_put(pnode);
		return -EINVAL;
	}

	data[0] = (u32) of_read_ulong(addr, 2);
	data[1] = (u32) size;

	return 0;
}

static int
rs_task_panic(struct notifier_block *this, unsigned long event, void *ptr)
{
	rs_recorder_dump_log("panic", 2, __LINE__, true);
	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = rs_task_panic,
};

static int rs_recorder_probe(struct platform_device *pdev)
{
	int rc = 0;
	u32 offsets[2];
	void *vaddr;
	u32 buf_size = 0;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_dir;
#endif

	if (!pdev->dev.of_node) {
		pr_err("can not find of_node of device\n");
		return -EINVAL;
	}

	rc = rs_recorder_parse_dt(&pdev->dev, offsets);
	if (rc < 0) {
		pr_err("rs_recorder parse dts failed %d\n", rc);
		return -EINVAL;
	}

	buf_size = offsets[1];

	vaddr = ioremap_wc(offsets[0], buf_size);
	pr_info("the phys_base=0x%x, size=0x%x vaddr=0x%llx\n",
			offsets[0], buf_size, (u64) vaddr);

	/* init rs_recorder buffer */
	s_record_buf.head_ptr = vaddr;
	s_record_buf.log_buf = vaddr + sizeof(struct rs_buf_header);
	memset(s_record_buf.path, 0, RS_LOG_PATH_LEN);
	s_record_buf.w_pos = 0;
	s_record_buf.buf_size = buf_size - sizeof(struct rs_buf_header);
	s_record_buf.left = s_record_buf.buf_size;
	s_record_buf.log_dump_pos = 0;

	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);

	INIT_WORK(&save_errinfo_work, save_dev_errinfo_work);
	INIT_WORK(&dump_work, dump_kernel_info_work);

#ifdef CONFIG_DEBUG_FS
	debugfs_dir = debugfs_create_dir("rs_recorder", NULL);
	debugfs_create_file("dev_errinfo", S_IRUGO, debugfs_dir,
			NULL, &dev_errinfo_proc_fops);
	debugfs_create_file("manual_dump", S_IRUGO|S_IWUGO, debugfs_dir,
			NULL, &dev_manual_dump_fops);
#endif

	if (!kthread_run(rs_check_dump_thread, NULL, "rs_check_dump")) {
		pr_err("create dump thread failed\n");
		rc = -EINVAL;
	}

	return rc;
}

static struct of_device_id rs_recorder_match_table[] = {
	{.compatible = "rs-recorder"},
	{},
};

static struct platform_driver rs_recorder_driver = {
	.probe		= rs_recorder_probe,
	.driver		= {
		.name	= "rs-recorder",
		.owner	= THIS_MODULE,
		.of_match_table = rs_recorder_match_table,
	},
};

static int __init rs_recorder_main_init(void)
{
	struct rs_common_info *ptr = &g_common_info;
	char *pname;

	dev_errinfo.w_pos = 0;

	/* common infomation init */
	pname = rs_get_product_name();
	strncpy((char *)ptr->product_name, pname, strlen(pname));

	rs_get_kernel_ver(ptr->kernel_ver);
	rs_get_productinfo(ptr->productinfo);

	return platform_driver_register(&rs_recorder_driver);
}

static void __exit rs_recorder_main_exit(void)
{
	vfree(s_record_buf.log_buf);
}

module_init(rs_recorder_main_init);
module_exit(rs_recorder_main_exit);

