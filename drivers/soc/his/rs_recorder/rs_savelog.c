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
#include <linux/syscalls.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/vmalloc.h>
#include <linux/sort.h>
#include <linux/utsname.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/fs.h>
#include "rs_common.h"

/* TODO: copy it from fs/readdir.c */
struct linux_dirent {
	unsigned long	d_ino;
	unsigned long	d_off;
	unsigned short	d_reclen;
	char		d_name[1];
};

int rs_save_file(const char *dir, const char *name,
			const void *address, u32 length)
{
	int fd;
	long bytes;
	mm_segment_t old_fs;
	char xname[RS_LOG_PATH_LEN] = {0};

	if (NULL == dir || NULL == name) {
		pr_err("file name and dir should not null\n");
		return -1;
	}

	if ((strlen((const char *)dir) + strlen((const char *)name)  + 1)
			>= RS_LOG_PATH_LEN) {
		pr_err("error: dir is too long, exit\n");
		return -1;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	memset(xname, 0, sizeof(xname));
	snprintf(xname, sizeof(xname), "%s/%s", dir, name);

	fd = sys_creat(xname, 0644);
	if (fd < 0) {
		pr_err("create file %s err.\n", xname);
		set_fs(old_fs);
		return -1;
	}

	bytes = sys_write((unsigned int)fd, (const char *)address, length);
	if ((u32)bytes != length) {
		sys_close(fd);
		set_fs(old_fs);
		return -1;
	}

	sys_fsync(fd);
	sys_close(fd);
	set_fs(old_fs);
	pr_info("save file %s success\n", xname);

	return 0;
}

int rs_read_file(const char *path, unsigned int offset,
	void *buf, unsigned int size)
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(path, O_RDONLY, 0644);
	if (IS_ERR(filp)) {
		pr_err(KERN_ERR "Failed to open %s\n", path);
		ret = -1;
		goto read_exit;
	}

	filp->f_op->llseek(filp, offset, SEEK_SET);
	ret = filp->f_op->read(filp, (char*)buf, size, &filp->f_pos);

	filp_close(filp, NULL);

read_exit:
	set_fs(old_fs);

	return ret;
}

int rs_write_file(const char *path, unsigned int offset,
	void *buf, unsigned int size)
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(path, O_RDWR | O_CREAT, 0666);
	if (IS_ERR(filp)) {
		pr_err(KERN_ERR "Failed to open %s\n", path);
		ret = -1;
		goto write_exit;
	}

	filp->f_op->llseek(filp, offset, SEEK_SET);
	ret = filp->f_op->write(filp, (char*)buf, size, &filp->f_pos);

	filp_close(filp, NULL);

write_exit:
	set_fs(old_fs);

	return ret;
}

static int rs_wait4partition(char *path, int timeouts)
{
	struct kstat m_stat;
	mm_segment_t old_fs;
	int timeo = timeouts;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	while (0 != vfs_stat(path, &m_stat)) {
		current->state = TASK_INTERRUPTIBLE;
		/*wait for 1 second*/
		schedule_timeout(HZ);
		if (timeouts-- == 0) {
			pr_err("wait %s %ds fail.skip!\n",
					path, timeo);
			set_fs(old_fs);
			return 1;
		}
	}

	set_fs(old_fs);
	pr_info("%s is ready!\n", path);

	return 0;
}

int rs_wait_fs_ready(void)
{
	int ret = 0;

	ret += rs_wait4partition("/data/lost+found", 20);
	ret += rs_wait4partition("/dev/block", 20);

	return ret;
}

static int __rs_create_dir(char *path)
{
	int fd;

	mm_segment_t old_fs;
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_access(path, 0);
	if (0 != fd) {
		fd  = sys_mkdir(path, 0755);
		if (fd < 0) {
			pr_err("create dir %s failed! ret = %d\n",
					path, fd);
			set_fs(old_fs);
			return fd;
		}

		pr_info("create dir '%s' successed!\n", path);
	}

	set_fs(old_fs);

	return 0;
}

int rs_create_dir(const char *path)
{
	char cur_path[RS_LOG_PATH_LEN];
	int index = 0;

	memset(cur_path, 0, RS_LOG_PATH_LEN);
	if (*path != '/')
		return -1;
	cur_path[index++] = *path++;

	while (*path != '\0') {
		if (*path == '/')
			__rs_create_dir(cur_path);

		cur_path[index] = *path;
		path++;
		index++;
	}

	return 0;
}

static int rs_rm_file(char *filename, u64 arg1, u64 arg2, int size)
{
	int ret;

	if (filename == NULL)
		return 0;

	ret = sys_access(filename, 0);
	if (0 == ret) {
		if (sys_unlink(filename)) {
			pr_err("%s: unlink %s failed\n", __func__, filename);
			return -1;
		}
	}

	return 0;
}

static int rs_rm_dir(char *path)
{
	char *pdst = path;
	int ret = 0;

	while (*pdst)
		pdst++;
	pdst--;
	if (*pdst == '/')
		*pdst = '\0';

	ret = sys_rmdir(path);
	if (ret != 0)
		pr_err("%s(): remove %s failed %d\n", __func__, path, ret);

	return ret;
}

static int rs_zipfile_init(int init)
{
	static int compress_init_flag;
	int ret;

	if (init) {
		if (compress_init_flag == 0) {
			ret = rs_compress_init();
			if (ret == 0) {
				compress_init_flag = 1;
				return 0;
			} else {
				pr_err("rs_recorder: %s() init error\n", __func__);
				return -1;
			}
		} else
			return 0;
	} else {
		if (compress_init_flag == 1) {
			rs_compress_exit();
			compress_init_flag = 0;
		}
	}

	return 0;
}

#define RDRDIRSIZ 1024
static char rs_tmp_buff[512*1024];
typedef void (*rs_funcptr)(unsigned long, u64, u64, int);

int rs_list_dir(char *path, rs_funcptr fn, u64 arg1, u64 arg2, int arg3,
		int *cnt, int type)
{
	int fd = -1, nread, bpos, ret = 0, tmp_cnt = 0;
	char *buf;
	struct linux_dirent *d;
	char d_type;
	mm_segment_t old_fs;
	char fullname[RS_LOG_PATH_LEN];

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(path, O_RDONLY, 0664);
	if (fd < 0) {
		pr_err("%s(),open %s fail,r:%d\n", __func__, path, fd);
		ret = -1;
		goto out;
	}

	buf = vmalloc(RDRDIRSIZ);
	if (buf == NULL) {
		pr_err("%s():vmalloc failed\n", __func__);
		ret = -1;
		goto out;
	}

	for (;;) {
		nread = sys_getdents(fd, (struct linux_dirent *)buf, RDRDIRSIZ);
		if (nread == -1) {
			pr_err("%s():sys_getdents failed\n", __func__);
			ret = -1;
			break;
		}

		if (nread == 0) {
			ret = 0;
			break;
		}

		for (bpos = 0; bpos < nread;) {
			d = (struct linux_dirent *)(buf + bpos);
			d_type = *(buf + bpos + d->d_reclen - 1);
			if ((d_type == type) && (fn != NULL)) {
				snprintf(fullname, sizeof(fullname), "%s%s",
						path, d->d_name);
				pr_debug("fullname:%s", fullname);
				fn((u64)fullname, arg1, arg2, arg3);
			}
			if (d_type == type)
				tmp_cnt++;
			bpos += d->d_reclen;
		}
	}
	if (cnt != (int *)0)
		*cnt = tmp_cnt;
	vfree(buf);
out:
	if (fd >= 0)
		sys_close(fd);
	set_fs(old_fs);
	return ret;
}

int rs_remove_whole_dir(char * path)
{
	int ret;

	rs_list_dir(path, (rs_funcptr)rs_rm_file,
				0, 0, 0, &ret, DT_REG);
	rs_rm_dir(path);

	return 0;
}

static char *rs_get_file_name(char *filename_withpath)
{
	char *p = filename_withpath;

	if (*p == '\0')
		return NULL;

	while (*p)
		p++;

	while ((*p != '/') && (p != filename_withpath))
		p--;

	if (*p == '/') {
		p++;
		return p;
	}

	return NULL;
}

int rs_file2zfile(char *fullname, u64 zipfd, u64 zip_head, int size)
{
	int ret, fd, i, out_cnt, err = 0, in_len;
	struct rs_archive_file *zfile = (struct rs_archive_file *)zip_head;
	struct rs_archive_head *head = &zfile->head;
	char *zip_buf = (char *)(zip_head + size);
	char *name = rs_get_file_name(fullname);

	if (name == NULL) {
		pr_err("%s() name is null...return.\n", __func__);
		return -1;
	}

	fd = sys_open(fullname, O_RDONLY, 0664);
	if (fd < 0) {
		pr_err("%s() open bin file failed\n", __func__);
		return -1;
	}

	for (i = 0; i < zfile->file_num; i++) {
		if (*head[i].filename == '\0')
			break;
	}

	in_len = sys_read(fd, (char *)rs_tmp_buff, sizeof(rs_tmp_buff));
	if (in_len <= 0) {
		pr_debug("read the file to zip end.");
		err = 0;
		goto out;
	}

	out_cnt = rs_recorder_compress(rs_tmp_buff, zip_buf,
			in_len, sizeof(rs_tmp_buff));
	if (out_cnt < 0) {
		pr_err("%s() after compress size %d, inlen:%d.\n",
				__func__, out_cnt, in_len);
		if (in_len < 100) {
			memcpy(zip_buf, (char *)rs_tmp_buff, in_len);
			out_cnt = in_len;
		} else {
			err = -1;
			goto out;
		}
	}
	ret = sys_write(zipfd, zip_buf, out_cnt);
	if (ret < out_cnt) {
		pr_err("%s() sys_write return %d.\n", __func__, ret);
		err = -1;
		goto out;
	}

	if (i == 0)
		head[i].off = size;
	else
		head[i].off = head[i - 1].off + head[i - 1].zip_len;

	strncpy(head[i].filename, name, RS_ARCHIVE_FNAME_LEN - 1);
	head[i].filename[RS_ARCHIVE_FNAME_LEN - 1] = '\0';
	head[i].orig_len = in_len;
	head[i].zip_len = out_cnt;

out:
	sys_close(fd);
	return err;
}

int rs_compress_buffer(char * inbuf, int inlen)
{
	u32 out_len;

	rs_zipfile_init(1);
	out_len = rs_recorder_compress(inbuf, rs_tmp_buff,
				inlen, sizeof(rs_tmp_buff));
	if (out_len < 0) {
		pr_err("%s() after compress size %d.\n",
				__func__, out_len);
		goto error;
	}
	rs_zipfile_init(0);

	memcpy(inbuf, &out_len, sizeof(out_len));
	memcpy(inbuf+sizeof(u32), rs_tmp_buff, out_len);

error:
	pr_info("After compress buffer size is %d\n", out_len);
	return out_len;
}

static int rs_compress_dir(char *path, char *db_name)
{
	struct rs_archive_file *dst;
	int zipfd, ret;
	struct kstat m_stat;
	char fname[RS_LOG_PATH_LEN];

	if (path == NULL) {
		pr_err("The path pointer is NULL\n");
		return -1;
	}

	ret = vfs_stat(path, &m_stat);
	if (ret) {
		pr_err("%s: dir not exist, exit\n", __func__);
		return -1;
	}

	/* check path file count, if count=0, del and return */
	rs_list_dir(path, NULL, 0, 0, 0, &ret, DT_REG);
	pr_info("%s: file cnt is %d\n", __func__, ret);
	if (ret > 0) {
		size_t siz = sizeof(struct rs_archive_file) +
				sizeof(struct rs_archive_head) * (ret - 1);
		size_t siz2 = siz + sizeof(rs_tmp_buff);

		dst = vzalloc(siz2);
		if (dst == NULL) {
			pr_err("%s():vmalloc dst failed\n", __func__);
			return -1;
		}

		dst->file_magic = RS_ARCHIVE_FILE_MAGIC;
		dst->file_num = ret;

		snprintf(fname, sizeof(fname), "%s%s.db", RS_DUMP_PATH, db_name);
		zipfd = sys_open(fname, O_CREAT | O_RDWR, 0664);
		if (zipfd < 0) {
			vfree(dst);
			return -1;
		}

		rs_zipfile_init(1);
		ret = sys_lseek(zipfd, siz, SEEK_SET);
		if (ret < siz) {
			pr_err("%s():lseek to %ld failed\n", __func__, siz);
			sys_close(zipfd);
			vfree(dst);
			rs_zipfile_init(0);
			return -1;
		}

		rs_list_dir(path, (rs_funcptr)rs_file2zfile, (u64)zipfd,
				(u64)dst, siz, &ret, DT_REG);

		rs_zipfile_init(0);

		/* write head to file: */
		ret = sys_lseek(zipfd, 0, SEEK_SET);
		if (ret != 0) {
			pr_err("%s():lseek failed\n", __func__);
			sys_close(zipfd);
			vfree(dst);
			return -1;
		}
		ret = sys_write(zipfd, (char *)dst, siz);
		if (ret < siz) {
			pr_err("%s(): write head failed\n", __func__);
			sys_close(zipfd);
			vfree(dst);
			return -1;
		}

		sys_close(zipfd);
		vfree(dst);
	} else {
		pr_info("delete empty dir %s", path);
		ret = sys_rmdir(path);
		if (ret != 0)
			pr_err("%s():delete dir %s fail\n", __func__, path);
	}

	return 0;
}

void create_archive_file(char *path, char *name)
{
	/* compress log dir */
	rs_compress_dir(path, name);
}

struct file_ctime_info {
	char filename[RS_LOG_PATH_LEN];
	struct timespec ctime;
};

static struct file_ctime_info oldest_file;

static int rs_find_oldest_file(char *filename, u64 arg1, u64 arg2, int size)
{
	int ret = 0;
	struct kstat m_stat;
	static int run_times = 0;

	ret = vfs_stat(filename, &m_stat);
	if (ret) {
		pr_err("%s: file %s not exist!\n", __func__, filename);
		return -1;
	}

	if (run_times == 0) {
		strncpy(oldest_file.filename, filename, strlen(filename));
		oldest_file.ctime = m_stat.ctime;
		run_times ++;
		return 0;
	}

	run_times ++;
	if (run_times >= size) {
		pr_info("run_times is %d, reset run_times\n", run_times);
		run_times = 0;
	}

	/*
	 * lhs < rhs:  return <0
	 * lhs == rhs: return 0
	 * lhs > rhs:  return >0
	 */
	if (timespec_compare(&oldest_file.ctime, &m_stat.ctime) > 0) {
		strncpy(oldest_file.filename, filename, strlen(filename));
		oldest_file.ctime = m_stat.ctime;
	}

	return 0;
}

int rs_rm_file_bylimit(char *path)
{
	int ret = 0;
	int num = 0;
	struct kstat m_stat;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	ret = vfs_stat(path, &m_stat);
	if (ret) {
		pr_err("%s: dir not exist!\n", __func__);
		return -1;
	}

	rs_list_dir(path, NULL, 0, 0, 0, &num, DT_REG);
	pr_info("The current dir exist %d file\n", num);
	if (num > RS_FILE_NUM_LIMIT) {
		/* find the oldest file */
		rs_list_dir(path, (rs_funcptr)rs_find_oldest_file,
				0, 0, num, &ret, DT_REG);

		rs_rm_file(oldest_file.filename, 0, 0, 0);
		memset(&oldest_file, 0, sizeof(struct file_ctime_info));
	}

	set_fs(old_fs);

	return 0;
}

