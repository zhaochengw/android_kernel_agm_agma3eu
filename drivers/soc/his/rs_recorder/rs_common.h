
#ifndef __RS_RECORDER_COMMON_H__
#define __RS_RECORDER_COMMON_H__

#include <linux/productinfo.h>

#define RS_PRODUCT_NAME_LEN         16
#define RS_KERNEL_VER_LEN           128
#define PRODUCTINFO_DATA_LEN        100

struct rs_common_info {
	char product_name[RS_PRODUCT_NAME_LEN];
	char kernel_ver[RS_KERNEL_VER_LEN];
	char productinfo[PRODUCTINFO_ID_NUM * PRODUCTINFO_DATA_LEN];
};

struct rs_dump_info {
	char reason[20];
	int level;
	int line;
	int is_panic;
};

#define RS_DUMP_PATH             "/data/rs_recorder/"
#define RS_LOG_PATH_LEN          128

struct rs_recorder_log {
	char *head_ptr;
	char *log_buf;
	char path[RS_LOG_PATH_LEN];
	int is_panic;
	u32 w_pos;        /* buffer write position */
	int left;         /* buf left size */
	u32 buf_size;     /* buf real size */
	u32 log_dump_pos; /* use __LINE__ to avoid repeat dump */
};

#define RS_RECORDER_BUF_SIZE   ((1 << CONFIG_LOG_BUF_SHIFT) + 1024)

#define RS_FILE_NUM_LIMIT        20

#define RS_LOG_PANIC_MAGIC       0x53504F4F   /* OOPS */
#define RS_LOG_GENERIC_MAGIC     0x44525352   /* RSRD */

/* rs_recorder archive file header infomation */
#define RS_ARCHIVE_FILE_MAGIC    0x50495A48   /* HZIP */
#define RS_ARCHIVE_FNAME_LEN     32

struct rs_archive_head {
	char filename[RS_ARCHIVE_FNAME_LEN];
	u32 off;
	u32 orig_len;
	u32 zip_len;
};

struct rs_archive_file {
	u32 file_magic;
	u32 file_num;
	struct rs_archive_head head;
};

#define RS_BUF_ITEMS_NUM       16
struct rs_buf_header {
	u32 magic;
	u32 num;
	char path[RS_LOG_PATH_LEN];
	struct rs_archive_head items[RS_BUF_ITEMS_NUM];
};

#define RS_LOG_PATITION    "/dev/block/bootdevice/by-name/grow"

extern void rs_update_buf_header(struct rs_recorder_log *log,
		char *fname, u32 len);

extern int rs_wait_fs_ready(void);
extern int rs_save_file(const char *dir, const char *name,
			const void *address, u32 length);
extern int rs_create_dir(const char *path);
extern int rs_remove_whole_dir(char *path);
extern int rs_rm_file_bylimit(char *path);
extern int rs_read_file(const char *path, unsigned int offset,
		void *buf, unsigned int size);
extern int rs_write_file(const char *path, unsigned int offset,
		void *buf, unsigned int size);

extern void create_archive_file(char *path, char *name);
extern int rs_compress_buffer(char *inbuf, int inlen);

extern int rs_dump_dmesg_log(struct rs_recorder_log *log, char *fname);
extern int rs_print_blocked_task(struct rs_recorder_log *log, char *fname);
extern int rs_dump_allps_info(struct rs_recorder_log *log, char *fname);
extern int rs_dump_all_slabinfo(struct rs_recorder_log *log, char *fname);
extern int rs_dump_meminfo(struct rs_recorder_log *log, char *fname);
extern int rs_dump_interrupts(struct rs_recorder_log *log, char *fname);

extern int rs_save_dump_to_file(struct rs_buf_header *header);

#ifdef CONFIG_DEBUG_KMEMLEAK
extern int rs_dump_kmemleak(struct rs_recorder_log *log, char *fname);
#endif

extern int rs_compress_init(void);
extern void rs_compress_exit(void);
extern int rs_recorder_compress(void *in, void *out, size_t inlen, size_t outlen);
#endif  /* __RS_RECORDER_COMMON_H__ */

