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
/* #include <linux/earlysuspend.h> */
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
/* #include <sound/sndinfo_vivo.h> */
/* #include <linux/bbk_drivers_info.h> */
#include "vivo-codec-common.h"
#include "ak4376.h"

#ifdef pr_debug
#undef pr_debug
#define pr_debug pr_err
#endif
#define VIVO_SOC_CODEC_NAME  "vivo_soc_codec"



struct vivo_codec_prv {
	struct audio_params params;
	struct snd_soc_codec *codec;
	unsigned int hp_path;
	unsigned int hifi_on;
	unsigned int hifi_switch_on;
	unsigned int hifi_mode;
	unsigned int hifi_dac_reset_gpio;
	struct vivo_codec_function *fun;
	struct pinctrl *pinctrl;
	struct pinctrl_state *select_pin_hi_state;
	struct pinctrl_state *select_pin_lo_state;
};

static struct vivo_codec_prv *vivo_codec;

static int vivo_codec_power_up(enum vivo_codec_id id)
{
	pr_info("%s:id %d\n", __func__, id);

	switch (id) {
	case VIVO_CODEC_HIFI_DAC:
		break;
	case VIVO_CODEC_HIFI_CLK:
		break;
	case VIVO_CODEC_SMART_PA:
		break;
	default:
		pr_err("%s() invalid id %d\n", __func__, id);
	}

	return 0;
}

static int vivo_codec_power_down(enum vivo_codec_id id)
{
	int ret = 0;

	pr_info("%s:id %d\n", __func__, id);

	switch (id) {
	case VIVO_CODEC_HIFI_DAC:
		if (vivo_codec->hifi_dac_reset_gpio > 0) {
			msleep(40);
		}
		break;
	case VIVO_CODEC_HIFI_CLK:
		break;
	case VIVO_CODEC_SMART_PA:
		break;
	default:
		pr_err("%s() invalid id %d\n", __func__, id);
	}
	return ret;
}

static int vivo_codec_hw_reset(enum vivo_codec_id id)
{
	int ret = 0;
	pr_info("%s:id %d\n", __func__, id);

	switch (id) {
	case VIVO_CODEC_HIFI_DAC:
		if (vivo_codec->hifi_dac_reset_gpio > 0) {
			msleep(10);
		}
		break;
	case VIVO_CODEC_HIFI_CLK:
		break;
	case VIVO_CODEC_SMART_PA:
		break;
	default:
		pr_err("%s() invalid id %d\n", __func__, id);
	}

	return ret;
}

static int vivo_codec_mclk_enable(enum vivo_codec_id id, unsigned long rate)
{
	pr_info("%s enter rate %lu\n", __func__, rate);

	switch (id) {
	case VIVO_CODEC_HIFI_DAC:
		break;
	case VIVO_CODEC_HIFI_CLK:
		break;
	case VIVO_CODEC_SMART_PA:
		break;
	default:
		pr_err("%s() invalid id %d\n", __func__, id);
	}

	return 0;
}

static int vivo_codec_mclk_disable(enum vivo_codec_id id)
{
	switch (id) {
	case VIVO_CODEC_HIFI_DAC:
		break;
	case VIVO_CODEC_HIFI_CLK:
		break;
	case VIVO_CODEC_SMART_PA:
		break;
	default:
		pr_err("%s() invalid id %d\n", __func__, id);
	}

	return 0;
}

static int vivo_hifi_dac_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->dapm->component->codec;
	struct vivo_codec_prv *vivo_codec = snd_soc_codec_get_drvdata(codec);
	int ret = 0, rate;

	rate = vivo_codec->params.rate;
	printk("%s: %s,  event: %d", __func__,
		event & (SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU)?"power up":"power down", event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
	case SND_SOC_DAPM_POST_PMU:
			if (vivo_codec && vivo_codec->fun && vivo_codec->fun->hifi_dac_enable)
				ret = vivo_codec->fun->hifi_dac_enable(&vivo_codec->params, true);
			vivo_codec->hifi_on = true;
			break;
	case SND_SOC_DAPM_PRE_PMD:
	case SND_SOC_DAPM_POST_PMD:
			vivo_codec->hifi_on = false;
			if (vivo_codec && vivo_codec->fun && vivo_codec->fun->hifi_dac_enable)
				ret = vivo_codec->fun->hifi_dac_enable(&vivo_codec->params, false);
			break;
	default:
			break;

	}
	return ret;
}

static const char *const hp_text[] = {
	"Normal", "Hifi", "Ktv"
};

static const struct soc_enum hp_path_enum[] = {
	SOC_ENUM_SINGLE_EXT(3, hp_text),
};

static const char *const enable_text[] = {
	"Off", "On"
};
static const char *const hifi_mode_text[] = {
	"Normal", "HiFi"
};
static const struct soc_enum enable_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, enable_text),
};
static const struct soc_enum hifi_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, hifi_mode_text),
};
static int vivo_codec_hifi_mode_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;
	struct vivo_codec_prv *vivo_codec = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = vivo_codec->hifi_mode;
	pr_debug("%s: get mode %d.\n", __func__, vivo_codec->hp_path);
	return 0;
}
static int vivo_codec_hifi_mode_set(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;
	struct vivo_codec_prv  *vivo_codec = snd_soc_codec_get_drvdata(codec);
	struct ak4376_params *hifi_param;

	if (!vivo_codec) {
		pr_err("%s: fail to get vivo codec pri data.\n", __func__);
		return -ENODATA;
	}

	pr_debug("%s: ucontrol-value: %ld.\n", __func__, ucontrol->value.integer.value[0]);

	vivo_codec->hifi_mode = ucontrol->value.integer.value[0];

	hifi_param = (struct ak4376_params *)(vivo_codec->params.private_params);
	if (!hifi_param) {
		pr_err("%s: no hifi private params.\n", __func__);
		return -ENODATA;
	}
	hifi_param->mode = vivo_codec->hifi_mode+1;
	pr_debug("%s: hifi_mode=%d.\n", __func__, hifi_param->mode);
	if (hifi_param->mode == 1)
		hifi_param->vol = 0x11;
	else
		hifi_param->vol = 0x17;
	return 0;
}

static int vivo_codec_hp_path_option_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;
	struct vivo_codec_prv *vivo_codec = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = vivo_codec->hp_path;

	pr_debug("%s: get hp_path %d.\n", __func__, vivo_codec->hp_path);

	return 0;
}
static int vivo_codec_hp_path_option_set(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;
	struct vivo_codec_prv  *vivo_codec = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	pr_debug("%s: ucontrol-value: %ld.\n", __func__, ucontrol->value.integer.value[0]);

	switch (ucontrol->value.integer.value[0]) {
	case 0:
		vivo_codec->hp_path = 0;
		if (vivo_codec->pinctrl) {
			usleep_range(10*1000, 10*1000);
			pinctrl_select_state(vivo_codec->pinctrl,  vivo_codec->select_pin_hi_state);
			usleep_range(90*1000, 90*1000);
		}
		break;
	case 1:
		vivo_codec->hp_path = 1;
		if (vivo_codec->pinctrl) {
			pinctrl_select_state(vivo_codec->pinctrl, vivo_codec->select_pin_lo_state);
			usleep_range(30*1000, 30*1000);
		}
		break;
	case 2:
		break;
	default:
		pr_err("%s: invalid hp path option.\n", __func__);
		break;
	}
	pr_debug("%s: set to %d.\n", __func__, vivo_codec->hp_path);

	return ret;
}

static int vivo_codec_hifi_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;
	struct vivo_codec_prv *vivo_codec = snd_soc_codec_get_drvdata(codec);

	if (!vivo_codec) {
		pr_err("%s:fail to get vivo codec data.\n", __func__);
		return -ENODATA;
	}

	ucontrol->value.integer.value[0] = vivo_codec->hifi_on;
	pr_debug("%s: value=%d.\n", __func__, vivo_codec->hifi_on);

	return 0;
}
static int vivo_codec_hifi_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;
	struct vivo_codec_prv *vivo_codec = snd_soc_codec_get_drvdata(codec);
	int ret;

	if (!vivo_codec || !vivo_codec->fun || !vivo_codec->fun->hifi_dac_enable) {
		pr_err("%s: fail to get vivo codec pri data.\n", __func__);
		return -ENODATA;
	}

	if (ucontrol->value.integer.value[0] == vivo_codec->hifi_on) {
		pr_info("%s: the same value, no need to set.\n", __func__);
		return 0;
	}

	vivo_codec->hifi_on = ucontrol->value.integer.value[0];
	ret = vivo_codec->fun->hifi_dac_enable(&vivo_codec->params, !!vivo_codec->hifi_on);

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

static int vivo_soc_codec_probe(struct snd_soc_codec *codec)
{
	snd_soc_codec_set_drvdata(codec, vivo_codec);
	vivo_codec->codec = codec;
	snd_soc_dapm_ignore_suspend(&codec->component.dapm, "HPOUT");
	/*
	snd_soc_dapm_ignore_suspend(&codec->dapm, "HiFi DAC");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "I2S In");
	*/
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
	if (vivo_codec_prv && vivo_codec_prv->fun) {
		kfree(vivo_codec_prv->fun);
		vivo_codec_prv->fun = NULL;
	}
	if (vivo_codec_prv) {
		kfree(vivo_codec_prv);
		vivo_codec_prv = NULL;
	}
	return 0;
}

static const struct snd_kcontrol_new vivo_soc_codec_controls[] = {
	SOC_ENUM_EXT("Hp_path_Option", hp_path_enum[0],
		vivo_codec_hp_path_option_get, vivo_codec_hp_path_option_set),
	SOC_ENUM_EXT("VIVO_HiFi_Headset", enable_enum[0], vivo_codec_hifi_get, vivo_codec_hifi_set),
	SOC_ENUM_EXT("HiFi Mode Switch", hifi_mode_enum[0], vivo_codec_hifi_mode_get, vivo_codec_hifi_mode_set),
};

static int hifi_dac_get_switch_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = vivo_codec->hifi_switch_on;
	pr_info("%s: hifi dac enable %ld\n", __func__,
		ucontrol->value.integer.value[0]);
	return 0;
}
static int hifi_dac_put_switch_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	pr_info("%s: hifi dac enable %ld\n", __func__,
			ucontrol->value.integer.value[0]);
	if (ucontrol->value.integer.value[0] == vivo_codec->hifi_switch_on) {
		pr_info("%s: no need to set hifi the same value again.\n", __func__);
		return 0;
	}
	vivo_codec->hifi_switch_on = ucontrol->value.integer.value[0];

	if (ucontrol->value.integer.value[0])
		snd_soc_dapm_mixer_update_power(&vivo_codec->codec->component.dapm, kcontrol, 1, NULL);
	else
		snd_soc_dapm_mixer_update_power(&vivo_codec->codec->component.dapm, kcontrol, 0, NULL);

	return 0;
}

static const struct snd_kcontrol_new HiFi_dac_ctl =
	SOC_SINGLE_EXT("Switch", SND_SOC_NOPM, 0, 1, 0,
		hifi_dac_get_switch_mixer, hifi_dac_put_switch_mixer);

static const struct snd_soc_dapm_widget vivo_codec_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("HPOUT"),

	SND_SOC_DAPM_DAC_E("HiFi DAC", NULL, SND_SOC_NOPM, 0, 0,
		vivo_hifi_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SWITCH("HiFi", SND_SOC_NOPM, 0, 1, &HiFi_dac_ctl),

	SND_SOC_DAPM_AIF_IN("I2S In", "VIVO CODEC Playback", 0,
		SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_OUT("I2S Out", "VIVO CODEC Capture", 0,
		SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route vivo_codec_audio_routes[] = {
		{"HPOUT", NULL, "HiFi DAC"},
		{"HiFi DAC", NULL, "HiFi"},
		{"HiFi", "Switch", "I2S In"},
};

static const struct snd_soc_codec_driver vivo_soc_codec_drv = {
	.probe = vivo_soc_codec_probe,
	.remove = vivo_soc_codec_remove,
	.controls = vivo_soc_codec_controls,
	.num_controls = ARRAY_SIZE(vivo_soc_codec_controls),
	.dapm_widgets = vivo_codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(vivo_codec_dapm_widgets),
	.dapm_routes = vivo_codec_audio_routes,
	.num_dapm_routes = ARRAY_SIZE(vivo_codec_audio_routes),
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
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_state;

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		pr_err("Can't find select gpio pinctrl!.\n");
		return ret;
	}
	priv->pinctrl = pinctrl;

	pin_state = pinctrl_lookup_state(pinctrl, "select_out_hi");
	if (IS_ERR_OR_NULL(pin_state)) {
		ret = PTR_ERR(pin_state);
		pr_err("Can't find gpio select_out_hi pinctrl!.\n");
		return ret;
	}
	priv->select_pin_hi_state = pin_state;

	pin_state = pinctrl_lookup_state(pinctrl, "select_out_lo");
	if (IS_ERR_OR_NULL(pin_state)) {
		ret = PTR_ERR(pin_state);
		pr_err("Can't find gpio select_out_lo pinctrl!.\n");
		return ret;
	}
	priv->select_pin_lo_state = pin_state;
	pinctrl_select_state(pinctrl, priv->select_pin_hi_state);
	return ret;
}

static int vivo_soc_platform_probe(struct platform_device *pdev)
{
	struct vivo_codec_prv *vivo_codec_prv;
	struct vivo_codec_function *vivo_codec_function;
	struct ak4376_params *hifi_param = NULL;
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
	vivo_codec_function->power_up = vivo_codec_power_up;
	vivo_codec_function->power_down = vivo_codec_power_down;
	vivo_codec_function->hw_reset = vivo_codec_hw_reset;
	vivo_codec_function->mclk_enable = vivo_codec_mclk_enable;
	vivo_codec_function->mclk_disable = vivo_codec_mclk_disable;
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

	set_vivo_codec_function(vivo_codec_function);

	dev_info(&pdev->dev, "%s() vivo_codec_fun(%p)%p\n",
		__func__, &vivo_codec_function, vivo_codec_function);

	vivo_codec_prv->params.rate = 44100;
	vivo_codec_prv->params.pcm_format = SNDRV_PCM_FORMAT_S24_LE;
	vivo_codec_prv->params.i2s_format = SND_SOC_DAIFMT_CBS_CFS |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
	hifi_param = kmalloc(sizeof(struct ak4376_params), GFP_KERNEL);
	if (!hifi_param) {
		dev_err(&pdev->dev, "%s:hifi_param malloc failed\n", __func__);
		vivo_codec_prv->params.private_params = NULL;
	} else {
		vivo_codec_prv->params.private_params = hifi_param;
		hifi_param->mode = 2;
		hifi_param->vol = 0x17;
	}
	vivo_codec_prv->hifi_switch_on = 0;
	vivo_codec_prv->hifi_mode = 0;
	vivo_codec_prv->hifi_on = false;
	vivo_codec_prv->hp_path = false;
	vivo_codec_prv->fun = vivo_codec_function;
	vivo_codec = vivo_codec_prv;

	platform_set_drvdata(pdev, vivo_codec_function);

	dev_set_name(&pdev->dev, "%s", VIVO_SOC_CODEC_NAME);

	ret = snd_soc_register_codec(&pdev->dev, &vivo_soc_codec_drv,
		vivo_soc_codec_dai, ARRAY_SIZE(vivo_soc_codec_dai));
	if (ret < 0) {
		pr_err("soc register error %s, rc=%d\n", __func__, ret);
		goto err_register_codec;
	}

	pr_info("%s:complete\n", __func__);
	return 0;

err_register_codec:
	if (vivo_codec && vivo_codec->params.private_params) {
		kfree(vivo_codec->params.private_params);
		vivo_codec->params.private_params = NULL;
	}
	if (vivo_codec_prv) {
		kfree(vivo_codec_prv);
		vivo_codec = NULL;
	}
err_kmalloc:
	if (vivo_codec_function) {
		kfree(vivo_codec_function);
		set_vivo_codec_function(NULL);
	}

	return ret;

}

static int vivo_soc_platform_remove(struct platform_device *pdev)
{
	if (vivo_codec && vivo_codec->fun) {
		kfree(vivo_codec->fun);
		vivo_codec->fun = NULL;
	}
	if (vivo_codec && vivo_codec->params.private_params) {
		kfree(vivo_codec->params.private_params);
		vivo_codec->params.private_params = NULL;
	}
	if (vivo_codec) {
		kfree(vivo_codec);
		vivo_codec = NULL;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id vivo_codec_of_match[] = {
		{ .compatible = "vivo,vivo-soc-codec-td1702",},
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
		.name = "vivo_soc_codec_td1702",
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
	pr_info("%s, enter.\n", __func__);
	platform_driver_unregister(&vivo_codec_platform_driver);
}

module_init(vivo_codec_init);
module_exit(vivo_codec_exit);
MODULE_DESCRIPTION("Vivo codec module");
MODULE_LICENSE("GPL");
