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

#include <linux/vmalloc.h>
#include <linux/zlib.h>
#include <linux/mutex.h>
#include <linux/errno.h>

#define COMPRESS_LEVEL              3

static DEFINE_MUTEX(compr_mutex);
static struct z_stream_s log_stream;

int rs_recorder_compress(void *in, void *out, size_t inlen, size_t outlen)
{
	int err, ret;

	ret = -EIO;
	mutex_lock(&compr_mutex);
	err = zlib_deflateInit(&log_stream, COMPRESS_LEVEL);
	if (err != Z_OK)
		goto error;

	log_stream.next_in = in;
	log_stream.avail_in = inlen;
	log_stream.total_in = 0;
	log_stream.next_out = out;
	log_stream.avail_out = outlen;
	log_stream.total_out = 0;

	err = zlib_deflate(&log_stream, Z_FINISH);
	if (err != Z_STREAM_END)
		goto error;

	err = zlib_deflateEnd(&log_stream);
	if (err != Z_OK)
		goto error;

	if (log_stream.total_out >= log_stream.total_in)
		goto error;

	ret = log_stream.total_out;
error:
	mutex_unlock(&compr_mutex);
	return ret;
}

int rs_compress_init(void)
{
	size_t size = 0;

	size = max(zlib_deflate_workspacesize(MAX_WBITS, MAX_MEM_LEVEL),
				zlib_inflate_workspacesize());

	log_stream.workspace = vmalloc(size);
	if (!log_stream.workspace)
		return -ENOMEM;

	return 0;
}

void rs_compress_exit(void)
{
	vfree(log_stream.workspace);
}
