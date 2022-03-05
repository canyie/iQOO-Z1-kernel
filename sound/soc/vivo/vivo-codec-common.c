#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "vivo-codec-common.h"

struct vivo_codec_function *vivo_codec_fun;
static char s_codec_name[100];

struct vivo_codec_function *get_vivo_codec_function(void)
{
	return vivo_codec_fun;
}

void set_vivo_codec_function(struct vivo_codec_function *fun)
{
	vivo_codec_fun = fun;
}
void vivo_set_codec_name(const char *codec_name)
{
	strcpy(s_codec_name, codec_name);
	pr_info("%s: extern codec(%s)\n", __func__, s_codec_name);
}
char *vivo_get_codec_name(void)
{
	return s_codec_name;
}