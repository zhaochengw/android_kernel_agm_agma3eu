/*
 * hisresume.h
 *
 * Disp resume info!
 *
 * Copyright (C) HMCT Corporation 2015
 *
 */
#ifndef  _HISRESUME_H_
#define _HISRESUME_H_

enum s_a_id{
S_A_START,
S_A_POWERKEY_ID,
S_A_DB_CLICK_ID,//Ë«»÷»½ÐÑÊÂ¼þ
S_A_EMMC_ID,
S_A_TOUCH_ID,
S_A_LCD_ID,
S_A_TFCARD_ID,
S_A_MAIN_CAM_ID,
S_A_FRONT_CAM_ID,
S_A_SENSOR_HUB_ID,
S_A_SENSOR_ACCELEROMETER_ID,
S_A_SENSOR_COMPASS_ID,
S_A_SENSOR_ALPS_ID,
S_A_BL_ID,
S_A_MAX_ID,
S_A_SIGN = 0x7fffffff,
};
static inline void resumeinfo_start(enum s_a_id id)
{
	return;
}
static inline void resumeinfo_end(enum s_a_id id)
{
	return;
}
static inline void suspendinfo_start(enum s_a_id id)
{
	return;
}
static inline void suspendinfo_end(enum s_a_id id)
{
	return;
}

#endif
