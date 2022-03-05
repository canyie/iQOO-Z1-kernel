/****************************
	author: vivo
	description: this is vivo codec header file
*****************************/
#ifndef _VIVO_SOC_CODEC__H
#define _VIVO_SOC_CODEC__H

#include <sound/pcm.h>

#define VIVO_SOC_CODEC_NAME  "vivo_soc_codec"
#define HIFI_POWER_EN           GPIO_HIFI_LDO1V8_EN_PIN
#define DAC_RESET_CTRL_PIN	 	GPIO_HIFI_RSET_EN


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
			       

struct vivo_codec_priv {	
	unsigned int hp_path;
	unsigned int hp_mic_path;
	unsigned int hifi_on;
};

extern const char *vivo_codec_name;

#endif
