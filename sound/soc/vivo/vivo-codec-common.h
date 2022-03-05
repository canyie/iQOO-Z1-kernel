/* Add by ChenJinQuan<chenjinquan@vivo.com> for vivo codec. */
#ifndef __VIVO_CODEC_COMMON_H__
#define __VIVO_CODEC_COMMON_H__

#define INVAL_GPIO -1


#define SND_SOC_ADV_MT_FMTS (\
			       SNDRV_PCM_FMTBIT_S16_LE |\
			       SNDRV_PCM_FMTBIT_S16_BE |\
			       SNDRV_PCM_FMTBIT_U16_LE |\
			       SNDRV_PCM_FMTBIT_U16_BE |\
			       SNDRV_PCM_FMTBIT_S24_LE |\
			       SNDRV_PCM_FMTBIT_S24_BE |\
			       SNDRV_PCM_FMTBIT_U24_LE |\
			       SNDRV_PCM_FMTBIT_U24_BE |\
			       SNDRV_PCM_FMTBIT_S32_LE |\
			       SNDRV_PCM_FMTBIT_S32_BE |\
				  SNDRV_PCM_FMTBIT_U32_LE |\
				  SNDRV_PCM_FMTBIT_U32_BE)

#define SND_SOC_STD_MT_FMTS (\
			       SNDRV_PCM_FMTBIT_S16_LE |\
			       SNDRV_PCM_FMTBIT_S16_BE |\
			       SNDRV_PCM_FMTBIT_U16_LE |\
			       SNDRV_PCM_FMTBIT_U16_BE)
			       
enum {
	I2S_IN = 0,
	I2S_OUT,
};

enum {
	I2S_RATE_44_1_KHZ = 0,
	I2S_RATE_48_KHZ,
	I2S_RATE_88_2_KHZ,
	I2S_RATE_96_KHZ,
	I2S_RATE_176_4_KHZ,
	I2S_RATE_192_KHZ,
};

enum {
	I2S_FORMAT_S16_LE = 0,
	I2S_FORMAT_S24_LE,
};

enum {
	HP_PATH_NORMAL = 0,
	HP_PATH_HIFI,
	HP_PATH_KTV,
};

enum {
	MIC_PATH_NORMAL = 0,
	MIC_PATH_KTV,
};

enum {
	SMART_PA_MODE_OFF = 0,
	SMART_PA_MODE_MUSIC,
	SMART_PA_MODE_VOICE,
	SMART_PA_MODE_KTV,
};

enum {
	OFF = 0,
	ON,
};

/*
 * External codec id, defined for power up/down,
 * hw reset and gpio setting.
 */
enum vivo_codec_id {
	VIVO_CODEC_SMART_PA = 0,
	VIVO_CODEC_HIFI_DAC,
	VIVO_CODEC_HIFI_CLK,
	VIVO_CODEC_KTV,
	VIVO_CODEC_HIFI_DAC_RST_DOWN,
};

enum ext_pa_swich_state {
	EXT_PA_SWICH_NONE = 0,
	EXT_PA_SWICH_MUSIC,
	EXT_PA_SWICH_VOICE,
	EXT_PA_SWICH_FM,
};

struct audio_params {
	int rate;
	unsigned int i2s_format;
	int pcm_format;
	unsigned long sys_clk;
	/* Each external codec has their private params */
	void *private_params;
};

struct vivo_codec_function {
	/*
	 * Called by external codecs driver,
	 * such as smart pa, hifi dac, ktv codec etc.
	 */
	int (*power_up)(enum vivo_codec_id id);
	int (*power_down)(enum vivo_codec_id id);
	int (*hw_reset)(enum vivo_codec_id id);
	int (*mclk_enable)(enum vivo_codec_id id, unsigned long rate);
	int (*mclk_disable)(enum vivo_codec_id id);
	int (*mi2s_clk_enable)(struct audio_params *params);
	int (*mi2s_clk_disable)(void);

	/* Called by vivo codec driver */
	int (*hifi_clk_enable)(struct audio_params *params, bool enable);
	int (*hifi_dac_enable)(struct audio_params *params, bool enable);
	int (*hifi_dac_mute)(bool mute);
	int (*smart_pa_enable)(struct audio_params *params, bool enable, int mode);
	int (*smart_pa_set_mode)(int mode);
	int (*smart_pa_mute)(bool mute);
	/*ktv ftm mode*/
	int (*ktv_ftm_enable)(int mode);

		/* for mbhc */
	int (*get_hp_switch_state)(void);
	int (*set_hp_switch_state)(int state);
	int (*get_mic_switch_state)(void);
	int (*set_mic_switch_state)(int state);
	unsigned int (*mbhc_get_hp_impedance)(void);

	/* for ext pa */
	int (*ext_pa_enable)(int state);
};

struct vivo_codec_function *get_vivo_codec_function(void);
void set_vivo_codec_function(struct vivo_codec_function *fun);
void vivo_set_codec_name(const char *codec_name);
char *vivo_get_codec_name(void);
#endif