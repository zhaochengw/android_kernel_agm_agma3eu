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

#ifndef __RS_RECORDER_H__
#define __RS_RECORDER_H__


extern void rs_recorder_dump_log(char *reason, int level, int line,
		int is_panic);
extern void rs_exec_dump_task(char *reason, int level, int line,
		int is_panic);

#ifdef CONFIG_RS_RECORDER_SUPPORT
void save_dev_errinfo(const char *fmt, ...);
#else  /* CONFIG_RS_RECORDER_SUPPORT */
static inline void save_dev_errinfo(const char *fmt, ...) {}
#endif /* CONFIG_RS_RECORDER_SUPPORT */

#endif /* __RS_RECORDER_H__ */

