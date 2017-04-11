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

#include <linux/stddef.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <linux/memcontrol.h>
#include <linux/uaccess.h>
#include "rs_common.h"

#ifdef CONFIG_SLUB
#define OO_SHIFT 16
#define OO_MASK ((1 << OO_SHIFT) - 1)
#endif

/*
* struct array_cache
*
* Purpose:
* - LIFO ordering, to hand out cache-warm objects from _alloc
* - reduce the number of linked list operations
* - reduce spinlock operations
*
* The limit is stored in the per-cpu structure to reduce the data cache
* footprint.
*
*/
struct array_cache {
	unsigned int avail;
	unsigned int limit;
	unsigned int batchcount;
	unsigned int touched;
	spinlock_t lock;

	/*
	* Must have this definition in here for the proper
	* alignment of array_cache. Also simplifies accessing
	* the entries.
	*/
	void *entry[];
};

typedef unsigned int kmem_bufctl_t;

/*
* struct slab_rcu
*
* slab_destroy on a SLAB_DESTROY_BY_RCU cache uses this structure to
* arrange for kmem_freepages to be called via RCU.	This is useful if
* we need to approach a kernel structure obliquely, from its address
* obtained without the usual locking.  We can lock the structure to
* stabilize it and check it's still at the given address, only if we
* can be sure that the memory has not been meanwhile reused for some
* other kind of object (which our subsystem's lock might corrupt).
*
* rcu_read_lock before reading the address, then rcu_read_unlock after
* taking the spinlock within the structure expected at that address.
*/
struct slab_rcu {
	struct rcu_head head;
	struct kmem_cache *cachep;
	void *addr;
};

/*
 * struct slab
 *
 * Manages the objs in a slab. Placed either at the beginning of mem allocated
 * for a slab, or allocated from an general cache.
 * Slabs are chained into three list: fully used, partial, fully free slabs.
 */
struct slab {
	union {
		struct {
			struct list_head list;
			unsigned long colouroff;
			void *s_mem;		/* including colour offset */
			unsigned int inuse;    /* num of objs active in slab */
			kmem_bufctl_t free;
			unsigned short nodeid;
		};

		struct slab_rcu __slab_cover_slab_rcu;
	};
};

/*
* The slab lists for all objects.
*/
struct kmem_list3 {
	struct list_head slabs_partial;    /* partial list first, better asm code */
	struct list_head slabs_full;
	struct list_head slabs_free;
	unsigned long free_objects;
	unsigned int free_limit;
	unsigned int colour_next;	 /* Per-node cache coloring */
	spinlock_t list_lock;
	struct array_cache *shared;    /* shared per node */
	struct array_cache **alien;    /* on other nodes */
	unsigned long next_reap;	/* updated without locking */
	int free_touched;		 /* updated without locking */
};

struct slabinfo {
	unsigned long active_objs;
	unsigned long num_objs;
	unsigned long active_slabs;
	unsigned long num_slabs;
	unsigned long shared_avail;
	unsigned int limit;
	unsigned int batchcount;
	unsigned int shared;
	unsigned int objects_per_slab;
	unsigned int cache_order;
};

/*
 * The slab lists for all objects.
 */
struct kmem_cache_node {
	spinlock_t list_lock;

#ifdef CONFIG_SLAB
	struct list_head slabs_partial;	/* partial list first, better asm code */
	struct list_head slabs_full;
	struct list_head slabs_free;
	unsigned long free_objects;
	unsigned int free_limit;
	unsigned int colour_next;	/* Per-node cache coloring */
	struct array_cache *shared;	/* shared per node */
	struct array_cache **alien;	/* on other nodes */
	unsigned long next_reap;	/* updated without locking */
	int free_touched;		/* updated without locking */
#endif

#ifdef CONFIG_SLUB
	unsigned long nr_partial;
	struct list_head partial;
#ifdef CONFIG_SLUB_DEBUG
	atomic_long_t nr_slabs;
	atomic_long_t total_objects;
	struct list_head full;
#endif
#endif
};

#ifdef CONFIG_SLUB
static inline int rs_oo_order(struct kmem_cache_order_objects x)
{
	return x.x >> OO_SHIFT;
}

static inline int rs_oo_objects(struct kmem_cache_order_objects x)
{
	return x.x & OO_MASK;
}

static int rs_count_free(struct page *page)
{
	if (unlikely(NULL == page))
		return 0;

	return page->objects - page->inuse;/* [false alarm]:there is page protect before  */
}

static unsigned long rs_count_partial(struct kmem_cache_node *n, int (*get_count)(struct page *))
{
	unsigned long flags;
	unsigned long x = 0;
	struct page *page;

	if (unlikely(NULL == n || NULL == get_count))
		return 0;

	if (spin_trylock_irqsave(&n->list_lock, flags)) {
		list_for_each_entry(page, &n->partial, lru) {
			x += get_count(page);
		}

		spin_unlock_irqrestore(&n->list_lock, flags);
	}

	return x;
}

static inline struct kmem_cache_node *rs_get_node(struct kmem_cache *s, int node)
{
	return s->node[node];
}
#endif

#ifdef CONFIG_MEMCG_KMEM
static inline bool rs_is_root_cache(struct kmem_cache *s)
{
	return !s->memcg_params || s->memcg_params->is_root_cache;
}

static inline const char *rs_cache_name(struct kmem_cache *s)
{
	if (!rs_is_root_cache(s))
		return s->memcg_params->root_cache->name;

	return s->name;
}

static inline struct kmem_cache *rs_cache_from_memcg(struct kmem_cache *s, int idx)
{
	if (NULL == s->memcg_params)
		return NULL;

	return s->memcg_params->memcg_caches[idx];
}
#else
static inline bool rs_is_root_cache(struct kmem_cache *s)
{
	return true;
}

static inline const char *rs_cache_name(struct kmem_cache *s)
{
	return s->name;
}

static inline struct kmem_cache *rs_cache_from_memcg(struct kmem_cache *s, int idx)
{
	return NULL;
}
#endif

#ifdef CONFIG_SLUB
static void rs_do_get_slabinfo(struct kmem_cache *s, struct slabinfo *sinfo)
{
	unsigned long nr_partials = 0;
	unsigned long nr_slabs = 0;
	unsigned long nr_objs = 0;
	unsigned long nr_free = 0;
	int node;

	for_each_online_node(node) {
		struct kmem_cache_node *n = rs_get_node(s, node);
		if (NULL == n)
			continue;

		nr_partials += n->nr_partial;
#ifdef CONFIG_SLUB_DEBUG
		nr_slabs += atomic_long_read(&n->nr_slabs);
		nr_objs += atomic_long_read(&n->total_objects);
#endif
		nr_free += rs_count_partial(n, rs_count_free);
	}

	sinfo->active_objs = nr_objs - nr_free;
	sinfo->num_objs = nr_objs;
	sinfo->active_slabs = nr_slabs;
	sinfo->num_slabs = nr_slabs;
	sinfo->objects_per_slab = rs_oo_objects(s->oo);
	sinfo->cache_order = rs_oo_order(s->oo);
}
#endif

static void rs_memcg_accumulate_slabinfo(struct kmem_cache *s, struct slabinfo *info)
{
	struct kmem_cache *c;
	struct slabinfo sinfo;
	int i;

	if (!rs_is_root_cache(s))
		return;

	for_each_memcg_cache_index(i) {
		c = rs_cache_from_memcg(s, i);
		if (NULL == c)
			continue;

		memset(&sinfo, 0, sizeof(sinfo));
		rs_do_get_slabinfo(c, &sinfo);

		info->active_slabs += sinfo.active_slabs;
		info->num_slabs += sinfo.num_slabs;
		info->shared_avail += sinfo.shared_avail;
		info->active_objs += sinfo.active_objs;
		info->num_objs += sinfo.num_objs;
	}
}

static int rs_write_slabinfo_header(struct rs_recorder_log *log)
{
	int size = 0;
	char *buff = log->log_buf + log->w_pos;

	size = snprintf(buff, log->left, "%s",
			"slabinfo - version: 2.1\n"
			"# name            <active_objs> <num_objs> <objsize> <objperslab> <pagesperslab>"
			" : tunables <limit> <batchcount> <sharedfactor>"
			" : slabdata <active_slabs> <num_slabs> <sharedavail>\n");

	return size;
}

static void *rs_slabinfo_start(struct list_head *cache_chain, loff_t *pos)
{
	return seq_list_start(cache_chain, *pos);
}

static void *rs_slabinfo_next(void *p, struct list_head *cache_chain, loff_t *pos)
{
	return seq_list_next(p, cache_chain, pos);
}

static int rs_get_slabinfo(struct rs_recorder_log *log, void *p, int size)
{
	int info_len = 0;
	char *buff = NULL;
	struct slabinfo sinfo = {0};
	struct kmem_cache *s = list_entry(p, struct kmem_cache, list);

	buff = log->log_buf + log->w_pos + size;
	memset(&sinfo, 0, sizeof(sinfo));

	rs_do_get_slabinfo(s, &sinfo);
	rs_memcg_accumulate_slabinfo(s, &sinfo);
	info_len = snprintf(buff, log->left,
		"%-17s %6lu %6lu %6u %4u %4d"
		" : tunables %4u %4u %4u"
		" : slabdata %6lu %6lu %6lu\n",
		rs_cache_name(s), sinfo.active_objs, sinfo.num_objs, s->size, sinfo.objects_per_slab, (1 << sinfo.cache_order),
		sinfo.limit, sinfo.batchcount, sinfo.shared,
		sinfo.active_slabs, sinfo.num_slabs, sinfo.shared_avail);

	return info_len;
}

int rs_dump_all_slabinfo(struct rs_recorder_log *log, char *fname)
{
	loff_t pos = 0;
	void *ptr = NULL;
	char *buff;
	u32 data_size = 0;
	/* The slab cache mutex protects the management structures during changes */
	extern struct mutex slab_mutex;
	/* The list of all slab caches on the system */
	extern struct list_head slab_caches;

	buff = log->log_buf + log->w_pos;
	data_size = rs_write_slabinfo_header(log);

	if (mutex_trylock(&slab_mutex)) {
		ptr = rs_slabinfo_start(&slab_caches, &pos);

		while (1) {
			if (ptr == NULL)
				break;

			data_size += rs_get_slabinfo(log, ptr, data_size);
			ptr = rs_slabinfo_next(ptr, &slab_caches, &pos);
		}

		mutex_unlock(&slab_mutex);
	}

	if (!log->is_panic)
		rs_save_file(log->path, fname, buff, data_size);

	rs_update_buf_header(log, fname, data_size);

	return 0;
}

