#ifndef __HIS_EMMC_OPS_H__
#define __HIS_EMMC_OPS_H__

#define NO_SD_RAMDUMP_FLAG_PARTITION		"flag"

#define FLAG_UPDATE_GPT_GROW_OFFSET			(0x600)
#define FLAG_FORMAT_GROW_OFFSET				(0x604)
#define FLAG_NO_SD_RAMDUMP_ENABLE_OFFSET	(0x608)
#define FLAG_SHRINK_USERDATA_OFFSET			(0x60C)
#define FLAG_DDR_SIZE_OFFSET				(0x610)
#define FLAG_ENABLE_SERIAL_CONSOLE_OFFSET	(0x700)


int his_emmc_write(const char *name, unsigned int offset, 
	unsigned int size, void *buf);
int his_emmc_read(const char *name, unsigned int offset, 
	void *buf, unsigned int size);


#endif	/* __HIS_EMMC_OPS_H__ */
