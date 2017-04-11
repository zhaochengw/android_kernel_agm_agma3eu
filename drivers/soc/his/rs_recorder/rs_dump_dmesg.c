
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/ctype.h>
#include <linux/highmem.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/version.h>
#include <linux/time.h>
#include <linux/types.h>
#include "rs_common.h"

extern int rs_recorder_get_dmesg(char *buf, int size);

int rs_dump_dmesg_log(struct rs_recorder_log *log, char *fname)
{
	u32 get_cnt = 0;
	char *buff;

	if (NULL == log) {
		pr_err("[%s]: line [%d] invalid param!\n", __FILE__, __LINE__);
		return -1;
	}

	buff = log->log_buf + log->w_pos;
	get_cnt = rs_recorder_get_dmesg(buff, log->left);

	if (!log->is_panic)
		rs_save_file(log->path, fname, buff, get_cnt);

	rs_update_buf_header(log, fname, get_cnt);

	return 0;
}

