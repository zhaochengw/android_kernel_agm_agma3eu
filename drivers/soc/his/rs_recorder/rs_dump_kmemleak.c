
#define pr_fmt(fmt) "rs-recorder: " fmt

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
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/atomic.h>
#include <linux/kasan.h>
#include <linux/kmemcheck.h>
#include <linux/kmemleak.h>
#include <linux/memory_hotplug.h>
#include "rs_common.h"

#define HEX_ROW_SIZE		16
#define HEX_GROUP_SIZE		1
#define HEX_ASCII			1
#define HEX_MAX_LINES		2
#define MAX_TRACE			16
#define OBJECT_ALLOCATED	(1 << 0)
#define OBJECT_REPORTED		(1 << 1)
#define KMEMLEAK_BLACK	-1
extern int rs_logbuff_print(struct rs_recorder_log *log, int sz,
		const char *fmt, ...);

extern int rs_kmemleak_enabled_detect(void);
extern struct mutex *rs_kmemleak_get_scan_mutex(void);
extern struct list_head *rs_kmemleak_get_list_head(void);
extern struct kmem_cache *rs_kmemleak_get_object_cache(void);
extern struct kmem_cache *rs_kmemleak_get_area_cache(void);
extern unsigned long rs_kmemleak_get_min_age(void);
extern unsigned long rs_kmemleak_get_last_scan(void);

struct rs_kmemleak_object {
	spinlock_t lock;
	unsigned long flags;		/* object status flags */
	struct list_head object_list;
	struct list_head gray_list;
	struct rb_node rb_node;
	struct rcu_head rcu;		/* object_list lockless traversal */
	/* object usage count; object freed when use_count == 0 */
	atomic_t use_count;
	unsigned long pointer;
	size_t size;
	/* minimum number of a pointers found before it is considered leak */
	int min_count;
	/* the total number of pointers found pointing to this object */
	int count;
	/* checksum for detecting modified objects */
	u32 checksum;
	/* memory ranges to be scanned inside an object (empty for all) */
	struct hlist_head area_list;
	unsigned long trace[MAX_TRACE];
	unsigned int trace_len;
	unsigned long jiffies;		/* creation timestamp */
	pid_t pid;			/* pid of the current task */
	char comm[TASK_COMM_LEN];	/* executable name */
};

struct rs_kmemleak_scan_area {
	struct hlist_node node;
	unsigned long start;
	size_t size;
};

static int rs_get_object(struct rs_kmemleak_object *object)
{
	return atomic_inc_not_zero(&object->use_count);
}

static void rs_free_object_rcu(struct rcu_head *rcu)
{
	struct hlist_node *tmp;
	struct rs_kmemleak_scan_area *area;
	struct rs_kmemleak_object *object =
		container_of(rcu, struct rs_kmemleak_object, rcu);
	struct kmem_cache *scan_area_cache =
		rs_kmemleak_get_area_cache();
	struct kmem_cache *object_cache =
		rs_kmemleak_get_object_cache();

	/*
	 * Once use_count is 0 (guaranteed by put_object), there is no other
	 * code accessing this object, hence no need for locking.
	 */
	hlist_for_each_entry_safe(area, tmp, &object->area_list, node) {
		hlist_del(&area->node);
		kmem_cache_free(scan_area_cache, area);
	}
	kmem_cache_free(object_cache, object);
}

static void rs_put_object(struct rs_kmemleak_object *object)
{
	if (!atomic_dec_and_test(&object->use_count))
		return;

	/* should only get here after delete_object was called */
	WARN_ON(object->flags & OBJECT_ALLOCATED);

	call_rcu(&object->rcu, rs_free_object_rcu);
}

static int rs_hex_dump_object(struct rs_recorder_log *log,
			    struct rs_kmemleak_object *object, int size)
{
	const u8 *ptr = (const u8 *)object->pointer;
	int i, len, remaining;
	unsigned char linebuf[HEX_ROW_SIZE * 5];
	int hex_sz = size;

	/* limit the number of lines to HEX_MAX_LINES */
	remaining = len =
		min(object->size, (size_t)(HEX_MAX_LINES * HEX_ROW_SIZE));

	hex_sz += rs_logbuff_print(log, hex_sz, "  hex dump (first %d bytes):\n", len);
	for (i = 0; i < len; i += HEX_ROW_SIZE) {
		int linelen = min(remaining, HEX_ROW_SIZE);

		remaining -= HEX_ROW_SIZE;
		hex_dump_to_buffer(ptr + i, linelen, HEX_ROW_SIZE,
				   HEX_GROUP_SIZE, linebuf, sizeof(linebuf),
				   HEX_ASCII);
		hex_sz += rs_logbuff_print(log, hex_sz, "    %s\n", linebuf);
	}

	return hex_sz;
}


static int rs_print_unreferenced(struct rs_recorder_log *log,
			       struct rs_kmemleak_object *object)
{
	int i;
	int size = 0;
	unsigned int msecs_age = jiffies_to_msecs(jiffies - object->jiffies);

	size += rs_logbuff_print(log, size, "unreferenced object 0x%08lx (size %zu):\n",
		   object->pointer, object->size);
	size += rs_logbuff_print(log, size, "comm \"%s\", pid %d, jiffies %lu (age %d.%03ds)\n",
		   object->comm, object->pid, object->jiffies,
		   msecs_age / 1000, msecs_age % 1000);

	size += rs_hex_dump_object(log, object, size);
	size += rs_logbuff_print(log, size, "  backtrace:\n");

	for (i = 0; i < object->trace_len; i++) {
		void *ptr = (void *)object->trace[i];
		size += rs_logbuff_print(log, size, "    [<%p>] %pS\n", ptr, ptr);
	}

	return size;
}

static void *rs_kmemleak_start(struct list_head *klist, loff_t *pos)
{
	struct rs_kmemleak_object *object;
	loff_t n = *pos;

	list_for_each_entry_rcu(object, klist, object_list) {
		if (n-- > 0)
			continue;
		if (rs_get_object(object))
			goto out;
	}
	object = NULL;
out:
	return object;
}

static bool rs_color_white(const struct rs_kmemleak_object *object)
{
	return object->count != KMEMLEAK_BLACK &&
		object->count < object->min_count;
}

static bool rs_unreferenced_object(struct rs_kmemleak_object *object)
{
	unsigned long jiffies_min_age = rs_kmemleak_get_min_age();
	unsigned long jiffies_last_scan = rs_kmemleak_get_last_scan();

	return (rs_color_white(object) && object->flags & OBJECT_ALLOCATED) &&
		time_before_eq(object->jiffies + jiffies_min_age,
			       jiffies_last_scan);
}

static void *rs_kmemleak_next(struct list_head *klist, void *v, loff_t *pos)
{
	struct rs_kmemleak_object *prev_obj = v;
	struct rs_kmemleak_object *next_obj = NULL;
	struct rs_kmemleak_object *obj = prev_obj;

	++(*pos);

	list_for_each_entry_continue_rcu(obj, klist, object_list) {
		if (rs_get_object(obj)) {
			next_obj = obj;
			break;
		}
	}

	rs_put_object(prev_obj);
	return next_obj;
}

static int rs_kmemleak_show(struct rs_recorder_log *log, void *v)
{
	int size;
	struct rs_kmemleak_object *object = v;
	unsigned long flags;

	spin_lock_irqsave(&object->lock, flags);
	if ((object->flags & OBJECT_REPORTED) && rs_unreferenced_object(object))
		size = rs_print_unreferenced(log, object);
	spin_unlock_irqrestore(&object->lock, flags);

	return size;
}

int rs_dump_kmemleak(struct rs_recorder_log *log, char *fname)
{
	char *buff;
	u32 data_size = 0;
	loff_t pos = 0;
	void *ptr = NULL;
	struct mutex *kmem_mutex = NULL;
	struct list_head *kmem_list = NULL;

	if (NULL == log) {
		pr_err("[%s]: line [%d] invalid param!\n", __FILE__, __LINE__);
		return -EINVAL;
	}

	if (!rs_kmemleak_enabled_detect()) {
		pr_err("kmemleak not enabled, skip it\n");
		return 0;
	}

	buff = log->log_buf + log->w_pos;
	kmem_mutex = rs_kmemleak_get_scan_mutex();
	kmem_list = rs_kmemleak_get_list_head();

	if (mutex_trylock(kmem_mutex)) {
		rcu_read_lock();
		ptr = rs_kmemleak_start(kmem_list, &pos);
		while (1) {
			if (!ptr || IS_ERR(ptr))
				break;

			data_size += rs_kmemleak_show(log, ptr);

			ptr = rs_kmemleak_next(kmem_list, ptr, &pos);
		}
		rcu_read_unlock();
		mutex_unlock(kmem_mutex);
	}

	if (!log->is_panic)
		rs_save_file(log->path, fname, buff, data_size);

	rs_update_buf_header(log, fname, data_size);

	return 0;
}

