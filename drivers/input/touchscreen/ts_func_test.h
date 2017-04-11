#ifndef __TS_FUNC_TEST_H
#define __TS_FUNC_TEST_H

struct ts_func_test_device {
	struct device *dev;

	int (*check_fw_update_need)(struct device *dev);
	int (*get_fw_update_progress)(struct device *dev);
	int (*proc_fw_update)(struct device *dev, bool force);
	int (*get_rawdata)(struct device *dev, char *buf);
	int (*get_rawdata_info)(struct device *dev, char *buf);
	int (*proc_hibernate_test)(struct device *dev);
	int (*get_ic_fw_version)(struct device *dev, char *buf);
	int (*get_fs_fw_version)(struct device *dev, char *buf);
	int (*get_module_id)(struct device *dev, char *buf);
	int (*get_calibration_ret)(struct device *dev);

	int (*get_fw_path)(struct device *dev, char* buf, size_t buf_size);
	int (*set_fw_path)(struct device *dev, const char* buf);
	int (*proc_fw_update_with_given_file)(struct device *dev, const char* buf);
	int (*set_gesture_switch)(struct device *dev, const char* buf);
	bool (*get_gesture_switch)(struct device *dev);
	int (*get_open_test)(struct device *dev, char* buf);
	int (*get_short_test)(struct device *dev, char* buf);
	int (*set_test_config_path)(struct device *dev, const char* buf);
	bool (*need_test_config)(struct device *dev);
	int (*get_gesture_pos)(struct device *dev, char* buf);
	int (*set_tp_enable_switch)(struct device *dev, bool enable);
	bool (*get_tp_enable_switch)(struct device *dev);
};


void register_ts_func_test_device(struct ts_func_test_device *device);
void unregister_ts_func_test_device(struct ts_func_test_device *device);

#endif
