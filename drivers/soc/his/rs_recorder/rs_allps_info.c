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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/fdtable.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include "rs_common.h"

static const char *s_process_state[] = {
    "R (running)",        /*   0 */
    "S (sleeping)",       /*   1 */
    "D (disk sleep)",     /*   2 */
    "T (stopped)",        /*   4 */
    "t (tracing stop)",   /*   8 */
    "Z (zombie)",         /*  16 */
    "X (dead)",           /*  32 */
    "x (dead)",           /*  64 */
    "K (wakekill)",       /* 128 */
    "W (waking)",         /* 256 */
};

static int rs_open_files_cnt(struct fdtable *fdt)
{
	int size = 0;
	int i;

	if (NULL == fdt) {
		pr_err("File [%s] line [%d] invalid param!\n", __FILE__, __LINE__);
		return 0;
	}

	size = fdt->max_fds;
	/* Find the last open fd */
	for (i = size / BITS_PER_LONG; i > 0; ) {
		if (fdt->open_fds[--i])
			break;
	}

	i = (i + 1) * BITS_PER_LONG;

	return i;
}

static inline const char *rs_get_task_state(struct task_struct *p)
{
	unsigned int state = (p->state & TASK_REPORT) | p->exit_state;
	const char **pstr = &s_process_state[0];

	while (0 != state) {
		pstr++;
		state >>= 1;
	}

	return *pstr;
}

/*
 * display in kilobytes.
 */
#define K(x) ((x) << (PAGE_SHIFT - 10))
static int rs_show_single_taskinfo(struct rs_recorder_log *log,
		struct task_struct *ptask, int size)
{
	int info_len = 0;
	int open_files = 0;
	struct fdtable *old_fdt = NULL;
	char *buff = log->log_buf + log->w_pos + size;

	if (NULL == ptask) {
		pr_err("File: %s line: %d invalid param!\n",
				__FILE__, __LINE__);
		return -1;
	}

	if (NULL != ptask->files) {
		if (spin_trylock(&ptask->files->file_lock)) {
			old_fdt = files_fdtable(ptask->files);
			open_files = rs_open_files_cnt(old_fdt);
			spin_unlock(&ptask->files->file_lock);
		}
	} else {
		open_files = 0;
	}

	/* pid ppid tgid cpuid state vmm rss files name */
	info_len = snprintf(buff, log->left,
			"%5d %6d %6d %d %s %8lu KB %8lu KB %d %s\n",
			ptask->pid,
			ptask->parent->pid,
			ptask->tgid,
			task_cpu(ptask),
			rs_get_task_state(ptask),
			(NULL == ptask->mm) ? (0) : (K(ptask->mm->total_vm)),
			(NULL == ptask->mm) ? (0) : (K(get_mm_rss(ptask->mm))),
			open_files,
			ptask->comm);

	return info_len;
}

static int rs_show_all_taskinfo(struct rs_recorder_log *log)
{
	int size = 0;
	struct task_struct *pcur = NULL;
	char *buff = log->log_buf + log->w_pos;

	size = snprintf(buff, log->left, "%s",
			"pid    ppid    tgid  cpuid  state        vmm         rss     files    name\n");

	if (read_trylock(&tasklist_lock)) {
		for_each_process(pcur) {
			if (spin_trylock(&(pcur->alloc_lock))) {
				size += rs_show_single_taskinfo(log, pcur, size);
				spin_unlock(&(pcur->alloc_lock));
			}
		}

		read_unlock(&tasklist_lock);
	}

	return size;
}

int rs_dump_allps_info(struct rs_recorder_log *log, char *fname)
{
	char *buff;
	u32 info_size = 0;

	buff = log->log_buf + log->w_pos;
	info_size = rs_show_all_taskinfo(log);

	if (!log->is_panic)
		rs_save_file(log->path, fname, buff, info_size);

	rs_update_buf_header(log, fname, info_size);

	return 0;
}

