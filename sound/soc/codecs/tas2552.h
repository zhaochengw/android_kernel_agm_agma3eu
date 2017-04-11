/*
 * TAS2552  Audio Codec driver
 *
 * Copyright 2015 Hisense Corporation..
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __TAS2552_H__
#define __TAS2552_H__

struct tas2552 {
	struct i2c_client *client;
	struct miscdevice tas2552_device;
	unsigned int en_gpio;
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	int32_t opalum_f0_values[2];
	int32_t opalum_temperature_values[2];
};

struct tas2552_reg_val_set {
	u16	reg;
	u8	val;
};

enum tas2552_opalum_diag_set {
	AFE_SET_DIAG_TX = 0,
	AFE_SET_DIAG_RX = 1,
};

enum tas2552_opalum_effect_set {
	AUDIO_EFFECT_0 = 0,
	AUDIO_EFFECT_1,
	AUDIO_EFFECT_2,
	AUDIO_EFFECT_3,
	AUDIO_EFFECT_4,
	AUDIO_EFFECT_MAX,
};


enum tas2552_opalum_diag_get {
	AFE_GET_F0 = 0,
	AFE_GET_TEMPERATUE = 1,
};

#define TAS2552_REG_VAL(reg, val)		{reg, val}

#define TAS2552_EN_DELAY 2000
#define TAS2552_EN_DELAY_DELTA 1000

#define TAS2552_MAGIC	0xFA
#define TAS2552_READ_CTL		_IO(TAS2552_MAGIC, 0x01)
#define TAS2552_WRITE_CTL		_IO(TAS2552_MAGIC, 0x02)

extern int tas2552_do_calibration(void);
extern int tas2552_set_audio_effect(void);
extern void tas2552_set_voice_call_status(bool st);
extern void tas2552_set_playback_status( bool st);
extern void tas2552_hard_enable(int enable);
extern void tas2552_soft_enable(int enable);
extern void tas2552_init(void);

extern int opalum_afe_get_param(int command);
extern int opalum_afe_set_param_effect(int command);
extern int opalum_afe_set_param_diag(int command);
extern int opalum_afe_set_param_calib(int32_t data1, int32_t data2);
extern int opalum_afe_get_calib_values( int32_t *f0, int32_t *temp);

#endif
