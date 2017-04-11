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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/fdtable.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include "rs_common.h"

int rs_get_dir_name(char *path, char *pname)
{
	int len;
	char tmp_buf[RS_LOG_PATH_LEN];
	char *p = tmp_buf;

	if (*path == '\0')
		return -1;

	len = strlen(path);
	memcpy(tmp_buf, path, len);
	/* delete the last '/' character */
	tmp_buf[len-1] = '\0';

	while (*p)
		p++;

	while ((*p != '/') && (p != tmp_buf))
		p--;

	if (*p == '/') {
		p++;
		memcpy(pname, p, strlen(p));
		return 0;
	}

	return -1;
}

int rs_save_dump_to_file(struct rs_buf_header *pheader)
{
	int i = 0, ret;
	u32 start = 0;
	u32 data_len = 0;
	char *fname = NULL;
	char *buff = (char *)pheader;
	char archive_name[RS_ARCHIVE_FNAME_LEN];

	buff += sizeof(struct rs_buf_header);
	rs_create_dir(pheader->path);

	for (i = 0; i < pheader->num; i++) {
		char * tmp_buf = NULL;

		fname = pheader->items[i].filename;
		data_len = pheader->items[i].orig_len;
		start = pheader->items[i].off;
		tmp_buf = buff + start;

		rs_save_file(pheader->path, fname, tmp_buf, data_len);
	}

	ret = rs_get_dir_name(pheader->path, archive_name);

#ifdef CONFIG_RS_RECORDER_ARCHIVE
	pr_info("create archive file: %s\n", archive_name);
	if (!ret) {
		create_archive_file(pheader->path, archive_name);
		rs_remove_whole_dir(pheader->path);
		rs_rm_file_bylimit(RS_DUMP_PATH);
	}
#endif

	return 0;
}

