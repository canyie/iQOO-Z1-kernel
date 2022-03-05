/********************************************************************
	Name: vivo_soc_codec.c
	Author: vivo
	Description:This is a codec driver for vivo audio platform, include hifi, smart pa, and ktv
	Version: 0.1 20150817   -----created by bowen

*********************************************************************/

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kobject.h>
#include <linux/of_gpio.h>
/* #include <linux/earlysuspend.h> */
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
/* #include <sound/sndinfo_vivo.h> */
/* #include <linux/bbk_drivers_info.h> */
#include "vivo-codec-common.h"

#ifdef pr_debug
#undef pr_debug
#define pr_debug pr_err
#endif
#define VIVO_SOC_CODEC_NAME  "vivo_soc_codec"

const char *vivo_codec_name;

struct vivo_codec_prv {
	struct audio_params params;
	struct snd_soc_codec *codec;
	unsigned int spk_en;
	int shdnpin;
	unsigned int hp_path;
	unsigned int hifi_on;
	unsigned int hifi_switch_on;
	unsigned int hifi_mode;
	unsigned int hifi_dac_reset_gpio;
	unsigned int ext_spk_mode;
	struct vivo_codec_function *fun;
	struct pinctrl *pinctrl;
	struct pinctrl_state *select_pin_hi_state;
	struct pinctrl_state *select_pin_lo_state;
};

static struct vivo_codec_prv *vivo_codec;


static const char *const spk_en_text[] = {
	"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"
};
static const char *const spk_switch_text[] = {
	"None", "Spk", "Rcv", "FM"
};


static const struct soc_enum spk_en_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spk_en_text), spk_en_text),
};
static const struct soc_enum spk_switch_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spk_switch_text), spk_switch_text),
};



static int vivo_codec_spk_enable_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;
	struct vivo_codec_prv *vivo_codec = snd_soc_codec_get_drvdata(codec);

	if (!vivo_codec) {
		pr_err("%s:fail to get vivo codec data.\n", __func__);
		return -ENODATA;
	}

	ucontrol->value.integer.value[0] = vivo_codec->spk_en;
	pr_debug("%s: value=%d.\n", __func__, vivo_codec->spk_en);

	return 0;
}
static int vivo_codec_spk_enable_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;
	struct vivo_codec_prv *vivo_codec = snd_soc_codec_get_drvdata(codec);
	int i = 0, ret = 0;

	if (ucontrol->value.integer.value[0] == vivo_codec->spk_en) {
		pr_info("%s: the same value, no need to set.\n", __func__);
		return 0;
	}
	vivo_codec->spk_en = ucontrol->value.integer.value[0];
	pr_info("%s: spk_en=%d.\n", __func__, vivo_codec->spk_en);
	if (vivo_codec->shdnpin > 0) {
		if (vivo_codec->spk_en) {
			for (i = 0; i < vivo_codec->spk_en; i++) {
				ret = gpio_direction_output(vivo_codec->shdnpin, 0);
				if (ret) {
					pr_err("%s: Can not set ext_spkamp_gpio", __func__);
					return ret;
				}
				udelay(2);
				ret = gpio_direction_output(vivo_codec->shdnpin, 1);
				if (ret) {
					pr_err("%s: Can not set ext_spkamp_gpio", __func__);
					return ret;
				}
				udelay(2);
			}
			usleep_range(5*1000, 5*1000);
		} else {
			usleep_range(4*1000, 4*1000);
			ret = gpio_direction_output(vivo_codec->shdnpin, 0);
			if (ret) {
				pr_err("%s: Can not set ext_spkamp_gpio", __func__);
				return ret;
			}
			usleep_range(5*1000, 5*1000);
		}
	}
	return 0;
}

static int vivo_codec_ext_spk_switch_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;
	struct vivo_codec_prv *vivo_codec = snd_soc_codec_get_drvdata(codec);

	if (!vivo_codec) {
		pr_err("%s:fail to get vivo codec data.\n", __func__);
		return -ENODATA;
	}

	ucontrol->value.integer.value[0] = vivo_codec->ext_spk_mode ;
	pr_debug("%s: value=%d.\n", __func__, vivo_codec->ext_spk_mode);

	return 0;
}
static int vivo_codec_ext_spk_switch_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;
	struct vivo_codec_prv *vivo_codec = snd_soc_codec_get_drvdata(codec);
	int ret;

	if (!vivo_codec)
		pr_err("%s: vivo_codec is null\n", __func__);
	if (!vivo_codec->fun)
		pr_err("%s: vivo_codec fun is null\n", __func__);
	if (!vivo_codec->fun->ext_pa_enable)
		pr_err("%s: vivo_codec fun enable is null\n", __func__);
	if (!vivo_codec || !vivo_codec->fun || !vivo_codec->fun->ext_pa_enable) {
		pr_err("%s: fail to get vivo codec pri data.\n", __func__);
		return -ENODATA;
	}

	if (ucontrol->value.integer.value[0] == vivo_codec->ext_spk_mode) {
		pr_info("%s: the same value, no need to set.\n", __func__);
		return 0;
	}

	vivo_codec->ext_spk_mode = ucontrol->value.integer.value[0];
	if ((vivo_codec->ext_spk_mode == EXT_PA_SWICH_VOICE) || (vivo_codec->ext_spk_mode == EXT_PA_SWICH_MUSIC) || (vivo_codec->ext_spk_mode == EXT_PA_SWICH_FM)) {
		ret = vivo_codec->fun->ext_pa_enable(vivo_codec->ext_spk_mode);
		usleep_range(20*1000, 20*1001);
	} else {
		usleep_range(10, 11);
		ret = vivo_codec->fun->ext_pa_enable(vivo_codec->ext_spk_mode);
	}

	return ret;
}

static int vivo_soc_codec_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	int ret = 0;
	pr_debug("%s:enter.\n", __func__);
	return ret;
}

static void vivo_soc_codec_shutdown(struct snd_pcm_substream *substream,
										struct snd_soc_dai *dai)
{
	pr_debug("%s. enter.\n", __func__);
}

static int vivo_soc_codec_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct vivo_codec_prv *vivo_codec = snd_soc_codec_get_drvdata(codec);
	printk("%s: fmt %d.\n", __func__, fmt);
	vivo_codec->params.pcm_format = fmt;
	return 0;
}

static int vivo_soc_codec_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *parms, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct vivo_codec_prv *vivo_codec = snd_soc_codec_get_drvdata(codec);
	struct snd_pcm_runtime *runtime = substream->runtime;

	int ret = 0, fmt;
	fmt = runtime->format;
	vivo_codec->params.rate = params_rate(parms);
	vivo_codec->params.i2s_format |= SND_SOC_DAIFMT_CBS_CFS;
	pr_debug("%s:enter, rate=%d, fmt=%d.\n", __func__, vivo_codec->params.rate, fmt);
	return ret;
}
static int vivo_soc_codec_free(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	pr_debug("%s:enter.\n", __func__);
	return 0;
}

static int vivo_soc_codec_digital_mute(struct snd_soc_dai *dai, int mute)
{
	int ret = 0;
	pr_debug("%s:enter, mute=%d.\n", __func__, mute);
	return ret;
}
static int vivo_soc_codec_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int fmt;
	fmt = runtime->format;
	printk("%s: fmt=%d.", __func__, fmt);

	return 0;
}

static struct snd_soc_dai_ops vivo_codec_ops = {
	.startup = vivo_soc_codec_startup,
	.shutdown = vivo_soc_codec_shutdown,
	.digital_mute = vivo_soc_codec_digital_mute,
	.set_fmt = vivo_soc_codec_set_dai_fmt,
	.hw_params = vivo_soc_codec_hw_params,
	.hw_free = vivo_soc_codec_free,
	.prepare = vivo_soc_codec_prepare,
};

static struct snd_soc_dai_driver vivo_soc_codec_dai[] = {
	{
		.name = "VIVO-Codec-dai",
		.playback = {
			.stream_name = "Vivocodec_Playback",
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SND_SOC_ADV_MT_FMTS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.capture = {
			.stream_name = "VIVO CODEC Capture",
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SND_SOC_ADV_MT_FMTS,
			.rate_max = 48000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &vivo_codec_ops,
	}
};

static const struct snd_kcontrol_new vivo_soc_codec_controls[] = {
	SOC_ENUM_EXT("SpkAmp Enable Switch", spk_en_enum[0],
		vivo_codec_spk_enable_get, vivo_codec_spk_enable_set),
	SOC_ENUM_EXT("vivo_ext_spk_mode", spk_switch_enum[0],
		vivo_codec_ext_spk_switch_get, vivo_codec_ext_spk_switch_set),
};

static int vivo_soc_codec_probe(struct snd_soc_codec *codec)
{
	pr_info("%s:enter\n", __func__);
	snd_soc_codec_set_drvdata(codec, vivo_codec);
	vivo_codec->codec = codec;
	snd_soc_add_codec_controls(codec, vivo_soc_codec_controls,
				   ARRAY_SIZE(vivo_soc_codec_controls));

	return 0;
}

static int vivo_soc_codec_remove(struct snd_soc_codec *codec)
{
	struct vivo_codec_prv *vivo_codec_prv =
				snd_soc_codec_get_drvdata(codec);

	if (vivo_codec_prv && vivo_codec_prv->params.private_params) {
		kfree(vivo_codec_prv->params.private_params);
		vivo_codec_prv->params.private_params = NULL;
	}

	if (vivo_codec_prv) {
		kfree(vivo_codec_prv);
		vivo_codec_prv = NULL;
	}
	return 0;
}

static const struct snd_soc_codec_driver vivo_soc_codec_drv = {
	.probe = vivo_soc_codec_probe,
	.remove = vivo_soc_codec_remove,
};

#if 0
static void vivo_snd_info_require(struct device_node	*of_node)
{
	unsigned int vivo_snd_info = 0;
	unsigned int vivo_fm_info = 0;
	int ret, temp;

	if (of_find_property(of_node, "vivo, fm-support", NULL))
		vivo_snd_info |= 1 << BBK_SND_FM_SUPPORT_SHIFT;
	if (of_find_property(of_node, "vivo, hifi-support", NULL))
		vivo_snd_info |= 1 << BBK_SND_HIFI_SUPPORT_SHIFT;
	if (of_find_property(of_node, "vivo, hifi-always-on", NULL))
		vivo_snd_info |= 1 << BBK_SND_HIFI_ALWAYS_ON_SHIFT;
	if (of_find_property(of_node, "vivo, double-mic-support", NULL))
		vivo_snd_info |= 2 << BBK_SND_BUILTIN_MIC_NUM_SHIFT;
	set_vivo_snd_card_info(BBK_SND_FM_SUPPORT_MASK
		| BBK_SND_HIFI_SUPPORT_MASK | BBK_SND_HIFI_ALWAYS_ON_MASK
		| BBK_SND_BUILTIN_MIC_NUM_MASK, vivo_snd_info);

	ret = of_property_read_u32(of_node, "vivo, spa-driver-type", &temp);
	if (!ret) {
		vivo_snd_info = 0;
		vivo_snd_info |= temp << BBK_SND_PA_DRIVER_TYPE_SHIFT;
		set_vivo_snd_card_info(BBK_SND_PA_DRIVER_TYPE_MASK, vivo_snd_info);
	}

	ret = of_property_read_u32(of_node, "vivo, smartpa-num", &temp);
	if (!ret) {
		vivo_snd_info = 0;
		vivo_snd_info |= temp << BBK_SND_SMARTPA_NUM_SHIFT;
		set_vivo_snd_card_info(BBK_SND_SMARTPA_NUM_MASK, vivo_snd_info);
	}

	ret = of_property_read_u32(of_node, "vivo, pa-manufacturer", &temp);
	if (!ret) {
		vivo_snd_info = 0;
		vivo_snd_info |= temp << BBK_SND_PA_MANUFACTURER_SHIFT;
		set_vivo_snd_card_info(BBK_SND_PA_MANUFACTURER_MASK, vivo_snd_info);
	}

	of_property_read_u32(of_node, "vivo, fm-pcm-hp", &temp);
	if ((temp != 5) && (temp != 42))
		temp = 5;
	vivo_fm_info = 0;
	vivo_fm_info |= temp << BBK_SND_FM_PCM_HP_SHIFT;
	set_vivo_fm_info(BBK_SND_FM_PCM_HP_MASK, vivo_fm_info);

	of_property_read_u32(of_node, "vivo, fm-pcm-spk", &temp);
	if ((temp != 5) && (temp != 42))
		temp = 42;
	vivo_fm_info = 0;
	vivo_fm_info |= temp << BBK_SND_FM_PCM_SPK_SHIFT;
	set_vivo_fm_info(BBK_SND_FM_PCM_SPK_MASK, vivo_fm_info);

}
#endif
static int vivo_codec_parse_dt(struct platform_device *pdev,
					struct vivo_codec_prv *priv)
{
	int ret = 0;

	priv->shdnpin = of_get_named_gpio(pdev->dev.of_node, "shdn-gpios", 0);
	if (!gpio_is_valid(priv->shdnpin)) {
		pr_err("%s fail to get shdn-gpio pin\n", __func__);
		return -1;
	}
	ret = gpio_request(priv->shdnpin, "ext spkamp gpio");
	if (ret)
		pr_err("%s: gpio %d for ext spkamp gpio request failed...\n ", __func__, priv->shdnpin);
	else
		gpio_direction_output(priv->shdnpin, 0);
	return ret;
}

static int vivo_soc_platform_probe(struct platform_device *pdev)
{
	struct vivo_codec_prv *vivo_codec_prv;
	struct vivo_codec_function *vivo_codec_function;
	int ret = 0;

	dev_info(&pdev->dev, "%s: enter.\n", __func__);
	vivo_codec_function = kzalloc(sizeof(struct vivo_codec_function), GFP_KERNEL);
	if (!vivo_codec_function) {
		dev_err(&pdev->dev, "%s:vivo_codec_function malloc failed\n", __func__);
		return -ENOMEM;
	}
	vivo_codec_prv = kzalloc(sizeof(struct vivo_codec_prv), GFP_KERNEL);
	if (!vivo_codec_prv) {
		dev_err(&pdev->dev, "%s:vivo_codec_prv malloc failed\n", __func__);
		ret = -ENOMEM;
		goto err_kmalloc;
	}
	vivo_codec_parse_dt(pdev, vivo_codec_prv);
	vivo_codec_function->get_hp_switch_state = NULL;
	vivo_codec_function->set_hp_switch_state = NULL;
	vivo_codec_function->get_mic_switch_state = NULL;
	vivo_codec_function->set_mic_switch_state = NULL;

	vivo_codec_function->hifi_clk_enable = NULL;
	vivo_codec_function->hifi_dac_enable = NULL;
	vivo_codec_function->hifi_dac_mute = NULL;
	vivo_codec_function->smart_pa_enable = NULL;
	vivo_codec_function->smart_pa_set_mode = NULL;
	vivo_codec_function->smart_pa_mute = NULL;
	vivo_codec_function->mbhc_get_hp_impedance = NULL;
	vivo_codec_function->ext_pa_enable = NULL;

	set_vivo_codec_function(vivo_codec_function);

	dev_info(&pdev->dev, "%s() vivo_codec_fun(%p)%p\n",
		__func__, &vivo_codec_function, vivo_codec_function);


	vivo_codec_prv->hp_path = false;
	vivo_codec_prv->fun = vivo_codec_function;
	vivo_codec = vivo_codec_prv;

	platform_set_drvdata(pdev, vivo_codec_function);


	pr_info("%s: device_name(%s)\n", __func__, dev_name(&pdev->dev));
	//device_rename(&pdev->dev, VIVO_SOC_CODEC_NAME);
	vivo_codec_name = dev_name(&pdev->dev);

	ret = snd_soc_register_codec(&pdev->dev, &vivo_soc_codec_drv,
		vivo_soc_codec_dai, ARRAY_SIZE(vivo_soc_codec_dai));
	if (ret < 0) {
		pr_err("soc register error %s, rc=%d\n", __func__, ret);
		goto err_register_codec;
	}

	pr_info("%s:complete\n", __func__);

	return 0;

err_kmalloc:
err_register_codec:
	if (vivo_codec_prv) {
		kfree(vivo_codec_prv);
		vivo_codec = NULL;
	}

	return ret;

}

static int vivo_soc_platform_remove(struct platform_device *pdev)
{

	if (vivo_codec && vivo_codec->params.private_params) {
		kfree(vivo_codec->params.private_params);
		vivo_codec->params.private_params = NULL;
	}

	if (vivo_codec && vivo_codec->shdnpin > 0)
		gpio_free(vivo_codec->shdnpin);

	if (vivo_codec) {
		kfree(vivo_codec);
		vivo_codec = NULL;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id vivo_codec_of_match[] = {
		{ .compatible = "vivo,vivo-soc-codec",},
		{},
};
#else
#define vivo_codec_of_match 0
#endif


#ifndef CONFIG_OF
static struct platform_device *vivo_codec_platform_device;
#endif

static struct platform_driver vivo_codec_platform_driver = {
	.probe = vivo_soc_platform_probe,
	.remove = vivo_soc_platform_remove,
	.driver = {
		.name = "vivo_soc_codec",
		.owner = THIS_MODULE,
		.of_match_table = vivo_codec_of_match,
	},
};

static int __init vivo_codec_init(void)
{
	printk("%s, enter.\n", __func__);
#ifndef CONFIG_OF
	int ret;
	vivo_codec_platform_device = platform_device_alloc(VIVO_SOC_CODEC_NAME, -1);
	if (!vivo_codec_platform_device)
		return -ENOMEM;

	ret = platform_device_add(vivo_codec_platform_device);
	if (ret  != 0) {
		platform_device_put(vivo_codec_platform_device);
		return ret;
	}
#endif

	return platform_driver_register(&vivo_codec_platform_driver);
}

static void __exit vivo_codec_exit(void)
{
	pr_err("%s, enter.\n", __func__);
	platform_driver_unregister(&vivo_codec_platform_driver);
}

module_init(vivo_codec_init);
module_exit(vivo_codec_exit);
MODULE_DESCRIPTION("Vivo codec module");
MODULE_LICENSE("GPL");
