#include <linux/module.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/clk.h>
#include "vivo-codec-common.h"
#ifndef CONFIG_VIVO_CODEC_FOR_MTK
//#include <sound/vivo_auddrv_log.h>
#else
/* #include <sound/vivo_auddrv_log.h> */
#define VIVO_CODEC_HIFI_DAC 1
#endif

#include "ak4376.h"
#define ak4376_DEV_NAME "ak4376"
#define ak4376_PHYS_NAME "ak4376_phys_dev"
static bool ak4376_available;

#ifdef CONFIG_FMEA
extern int fmea_notify(char *name, char *data);
extern int writeDatas(char *modelId, char *filename, char *fmt, ...);
#endif

/*
 * add test_bit judgment
 * for ak4376 mclk@fanyongxiang
 */
enum {
	STATUS_MCLK,
	STATUS_MAX,
};

enum {
	ak4375_VERSION_0,
	ak4375_VERSION_A,
	AK4376_VERSION_0,
	ak4376_VERSION_NONE

} ak4376_VERSION;
int ak4376_version = ak4376_VERSION_NONE;

#ifndef BBK_IQOO_AUDIO_DEBUG
#define BBK_IQOO_AUDIO_DEBUG
#endif


#ifdef BBK_IQOO_AUDIO_DEBUG
static struct dentry *ak4376_debugfs_root;
static struct dentry *ak4376_debugfs_reg;
static struct dentry *ak4376_debugfs_i2c;

static u8 ak4376_regs[] = {
	0X00,
	0X01,
	0X02,
	0X03,
	0X04,
	0X05,
	0X06,
	0X07,
	0X08,
	0X09,
	0X0A,
	0X0B,
	0X0C,
	0X0D,
	0X0E,
	0X0F,
	0X10,
	0X11,
	0X12,
	0X13,
	0X14,
	0X15,
	0X24,
};

#endif

static u8 ak4376_reg_disable[6][2] = {
	{ 0x03, 0x00 },
	{ 0x01, 0x31 },         /* shijianxing: PowerDown CP2--->LDO1P/LDO1N--->CP1 */
	{ 0x02, 0x00 },
	{ 0x01, 0x01 },         /* shijianxing:PowerDown LDO1P/LDO1N */
	{ 0xff, 0xff },
	{ 0x01, 0x00 }          /* shijianxing: PowerDown CP1 */
};

static u8 ak4375_reg_enable[34][2] = {
	{ 0x06, 0x00 },
	{ 0x07, 0x21 },
	{ 0x08, 0x00 },
	{ 0x09, 0x00 },
	{ 0x0A, 0x00 },
	{ 0x0B, 0x19 },
	{ 0x0C, 0x19 },
	{ 0x0D, 0x73 },
	{ 0x0E, 0x01 },
	{ 0x0F, 0x00 },
	{ 0x10, 0x00 },
	{ 0x11, 0x00 },
	{ 0x12, 0x27 },
	{ 0x14, 0x09 },
	{ 0x15, 0x20 },
	{ 0x24, 0x00 },
	{ 0x13, 0x01 },
	{ 0x00, 0x01 },
	{ 0x05, 0x0A },
	{ 0xff, 0xff },
	{ 0x01, 0x01 },
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0x01, 0x31 },
	{ 0xff, 0xff },
	{ 0x02, 0x01 },
	{ 0x01, 0x33 },
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0x03, 0x03 },
	{ 0x04, 0x00 }
};

/* shijianxing add for AK4376 */
static u8 ak4376_reg_enable[36][2] = {
	{ 0x26, 0x20 },         /* DAC adjustment1 init */
	{ 0x2A, 0x05 },         /* DAC adjustment2 init */
	{ 0x06, 0x00 },
	{ 0x07, 0x21 },
	{ 0x08, 0x00 },
	{ 0x09, 0x00 },
	{ 0x0A, 0x00 },
	{ 0x0B, 0x17 },
	{ 0x0C, 0x17 },
	{ 0x0D, 0x0B },
	{ 0x0E, 0x01 },
	{ 0x0F, 0x00 },
	{ 0x10, 0x00 },
	{ 0x11, 0x00 },
	{ 0x12, 0x27 },
	{ 0x14, 0x09 },
	{ 0x15, 0x40 },
	{ 0x24, 0x00 },
	{ 0x13, 0x01 },
	{ 0x00, 0x01 },
	{ 0x05, 0x09 },
	{ 0xff, 0xff },
	{ 0x01, 0x01 },         /* shijianxing M:PowerUp CP1--->LDO1P/LDO1N--->CP2 */
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0x01, 0x31 },         /* shijianxing:PowerUp LDO1P/LDO1N */
	{ 0xff, 0xff },
	{ 0x02, 0x01 },
	{ 0x01, 0x33 },         /* shijianxing:PowerUp CP2 */
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0x03, 0x03 },
	{ 0x04, 0x00 }
};

/* shijianxing add:Low Power Mode for AK4376 */
static u8 ak4376_normal_reg_enable[36][2] = {
	{ 0x26, 0x20 },         /* DAC adjustment1 init */
	{ 0x2A, 0x05 },         /* DAC adjustment2 init */
	{ 0x06, 0x00 },
	{ 0x07, 0x21 },
	{ 0x08, 0x00 },
	{ 0x09, 0x00 },
	{ 0x0A, 0x00 },
	{ 0x0B, 0x17 },
	{ 0x0C, 0x17 },
	{ 0x0D, 0x0B },
	{ 0x0E, 0x01 },
	{ 0x0F, 0x00 },
	{ 0x10, 0x00 },
	{ 0x11, 0x00 },
	{ 0x12, 0x27 },
	{ 0x14, 0x09 },
	{ 0x15, 0x40 },
	{ 0x24, 0x40 },         /* DSMLPM=1 */
	{ 0x13, 0x01 },
	{ 0x00, 0x01 },
	{ 0x05, 0x0a },         /* 48K */
	{ 0xff, 0xff },
	{ 0x01, 0x01 },         /* shijianxing M:PowerUp CP1--->LDO1P/LDO1N--->CP2 */
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0x01, 0x31 },
	{ 0xff, 0xff },
	{ 0x02, 0x11 },         /* LPMode=1 */
	{ 0x01, 0x33 },
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0xff, 0xff },
	{ 0x03, 0x03 },
	{ 0x04, 0x00 }
};

static int reg_addr_tbl[] = {
	0x05,
	0x08,
	0x0e,
	0x10,
	0x12,
	0x14,
	0x15
};
#ifndef CONFIG_VIVO_CODEC_FOR_MTK
static int reg_value_tbl[21][7] = {
	{ 0x0a, 0x0a, 0x01, 0x00, 0x4f, 0x09, 0x01 },   /* ak4376 48k 16bit */
	{ 0x0a, 0x0a, 0x01, 0x00, 0x27, 0x09, 0x00 },   /* ak4376 48k 24bit */
	{ 0x0e, 0x0e, 0x01, 0x00, 0x13, 0x09, 0x00 },   /* ak4376 96k 24bit */
	{ 0x12, 0x12, 0x01, 0x00, 0x09, 0x09, 0x00 },   /* ak4376 192k 24bit */
	{ 0x0a, 0x0a, 0x01, 0x00, 0x4f, 0x09, 0x21 },   /* ak4376a 48k 16bit */
	{ 0x09, 0x09, 0x00, 0x18, 0x92, 0x09, 0x30 },   /* ak4376a 44.1k 24bit */
	{ 0x0a, 0x0a, 0x01, 0x00, 0x27, 0x09, 0x20 },   /* ak4376a 48k 24bit */
	{ 0x0d, 0x0d, 0x00, 0x18, 0x92, 0x04, 0x30 },   /* ak4376a 88.2k 24bit */
	{ 0x0e, 0x0e, 0x01, 0x01, 0x27, 0x04, 0x20 },   /* ak4376a 96k 24bit */
	{ 0x71, 0x71, 0x00, 0x18, 0x92, 0x04, 0x30 },   /* ak4376a 176.4k 24bit */
	{ 0x72, 0x72, 0x01, 0x03, 0x27, 0x04, 0x20 },   /* ak4376a 192k 24bit */
	/* shijianxing add for ak4376 for slave mode */
	{ 0x09, 0x00, 0x00, 0x18, 0x92, 0x09, 0x50 },   /* ak4376 44.1k 24bit */
	{ 0x0a, 0x00, 0x00, 0x09, 0x3f, 0x09, 0x52 },   /* ak4376 48k 24bit */
	{ 0x0d, 0x00, 0x00, 0x18, 0x92, 0x04, 0x50 },   /* ak4376 88.2k 24bit */
	{ 0x0e, 0x00, 0x00, 0x09, 0x3f, 0x04, 0x52 },   /* ak4376 96k 24bit */
	{ 0x71, 0x00, 0x00, 0x18, 0x92, 0x04, 0x50 },   /* ak4376 176.4k 24bit */
	{ 0x72, 0x00, 0x00, 0x09, 0x3f, 0x04, 0x52 },   /* ak4376 192k 24bit */
	/* shijianxing add for ak4376 for master mode */
	{ 0x0a, 0x00, 0x01, 0x00, 0x27, 0x09, 0x40 },   /* ak4376 48k 24bit */
	{ 0x0e, 0x00, 0x01, 0x01, 0x27, 0x04, 0x40 },   /* ak4376 96k 24bit */
	{ 0x72, 0x00, 0x01, 0x03, 0x27, 0x04, 0x40 },   /* ak4376 192k 24bit */
	{ 0x0a, 0x00, 0x01, 0x00, 0x4f, 0x09, 0x41 },   /* ak4376 48k 16bit */
};
#else
static int reg_value_tbl[19][7] = {
	{0x0a, 0x0a, 0x01, 0x00, 0x4f, 0x09, 0x01}, /* ak4376 48k 16bit */
	{0x0a, 0x0a, 0x01, 0x00, 0x27, 0x09, 0x00}, /* ak4376 48k 24bit */
	{0x0e, 0x0e, 0x01, 0x00, 0x13, 0x09, 0x00}, /* ak4376 96k 24bit */
	{0x12, 0x12, 0x01, 0x00, 0x09, 0x09, 0x00}, /* ak4376 192k 24bit */
	{0x0a, 0x0a, 0x01, 0x00, 0x4f, 0x09, 0x21}, /* ak4376a 48k 16bit */
	{0x09, 0x09, 0x00, 0x18, 0x92, 0x09, 0x30}, /* ak4376a 44.1k 24bit */
	{0x0a, 0x0a, 0x01, 0x00, 0x27, 0x09, 0x20}, /* ak4376a 48k 24bit */
	{0x0d, 0x0d, 0x00, 0x18, 0x92, 0x04, 0x30}, /* ak4376a 88.2k 24bit */
	{0x0e, 0x0e, 0x01, 0x01, 0x27, 0x04, 0x20}, /* ak4376a 96k 24bit */
	{0x71, 0x71, 0x00, 0x18, 0x92, 0x04, 0x30}, /* ak4376a 176.4k 24bit */
	{0x72, 0x72, 0x01, 0x03, 0x27, 0x04, 0x20}, /* ak4376a 192k 24bit */
	/*shijianxing add for ak4376 */
	{0x0a, 0x00, 0x01, 0x00, 0x4f, 0x09, 0x41}, /* ak4376 48k 16bit */
	{0x09, 0x00, 0x01, 0x00, 0x27, 0x09, 0x40}, /* ak4376 44.1k 24bit */
	{0x0a, 0x00, 0x01, 0x00, 0x27, 0x09, 0x40}, /* ak4376 48k 24bit */
	{0x0d, 0x00, 0x01, 0x01, 0x27, 0x04, 0x40}, /* ak4376 88.2k 24bit */
	{0x0e, 0x00, 0x01, 0x01, 0x27, 0x04, 0x40}, /* ak4376 96k 24bit */
	{0x71, 0x00, 0x01, 0x04, 0x31, 0x04, 0x40}, /* ak4376 176.4k 24bit */
	{0x72, 0x00, 0x01, 0x03, 0x27, 0x04, 0x40}, /* ak4376 192k 24bit */
	{0x04, 0x00, 0x01, 0x00, 0x77, 0x1D, 0x40}, /* ak4375a 16k 32bit //09  */
};

#endif

struct ak4376_data {
	struct i2c_client *client;
	char *driver_name;
	/* int vdd_en_gpio; */
	int rst_gpio;
	int volume;
	struct regulator *hifi_1v8_regulator;
	struct clk *hifi_mclk;
	struct vivo_codec_function *fun;
	struct mutex lock;
	int reg_size;
	int *reg_table;
	DECLARE_BITMAP(mclk_status_mask, STATUS_MAX);
#ifdef CONFIG_VIVO_CODEC_FOR_MTK
	struct regulator *vd_regulator;
	struct regulator *va_regulator;
	int vd_type;
	int va_type;
	int vd_ctrl_gpio;
	int va_ctrl_gpio;
	struct pinctrl *pinctrl;
	struct pinctrl_state *hifi_vd_hi_state;
	struct pinctrl_state *hifi_vd_lo_state;
	struct pinctrl_state *hifi_rst_hi_state;
	struct pinctrl_state *hifi_rst_lo_state;
#endif
};

static struct ak4376_data *ak4376_data;

static int ak4376_phys_open(struct inode *inode, struct file *filep)
{
	filep->private_data = inode->i_private;
	return 0;
}

int ak4376_phys_release(struct inode *inode, struct file *filep)
{
	int ret = 0;

	return ret;
}

long ak4376_phys_ioctl(struct file *filep, unsigned int cmd, unsigned long args)
{
	int ret = 0;

	pr_info("[HiFi] \n");
	return (long)ret;
}


static struct file_operations ak4376_phys_fops = {
	.open = ak4376_phys_open,
	.unlocked_ioctl = ak4376_phys_ioctl,
	.release = ak4376_phys_release,
};

static struct miscdevice ak4376_phys_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = ak4376_PHYS_NAME,
	.fops = &ak4376_phys_fops,
};

static int ak4376_i2c_read_byte(u8 reg)
{
	int ret = 0;
	u8 buf = 0xff;

	ret = i2c_master_send(ak4376_data->client, &reg, 1);
	if (ret < 0) {
		pr_err("[HiFi] i2c send cmd error reg=%d \n", reg);
		return ret;
	}
	ret = i2c_master_recv(ak4376_data->client, &buf, 1);
	if (ret < 0) {
		pr_err("[HiFi] i2c recv error \n ");
		return ret;
	}
	/* pr_err("%s: reg = 0x%x value = 0x%x\n", __func__, reg, buf); */

	return buf;

}

static int ak4376_i2c_write_byte(u8 reg, u8 data)
{
	int ret = 0;
	u8 cmd[2];
	static int i2c_err = 1;

	cmd[0] = reg;
	cmd[1] = data;

	ret = i2c_master_send(ak4376_data->client, cmd, sizeof(cmd));
	if (ret < 1) {
		pr_err("[HiFi] i2c send error cmd[0]=%d,cmd[1]=%d\n", cmd[0], cmd[1]);
		if (i2c_err) {
#ifdef CONFIG_FMEA
		writeDatas("1800", "1800_1",
				   "audio driver i2c: ak4376 reg 0x%x, val 0x%x, ret %d.\n",
				   cmd[0], cmd[1], ret);
#endif
		i2c_err = 0;
		}
	}
	pr_debug("[HiFi]: reg = 0x%x value = 0x%x\n", reg, data);

	return ret;
}

int ak4376_reset(int reset_pin)
{
#ifndef CONFIG_VIVO_CODEC_FOR_MTK
	gpio_direction_output(reset_pin, 0);
	usleep_range(2000, 3000);
	gpio_direction_output(reset_pin, 1);
	usleep_range(10000, 10000);
#else
	if (ak4376_data->pinctrl && ak4376_data->hifi_rst_lo_state && ak4376_data->hifi_rst_hi_state) {
		pinctrl_select_state(ak4376_data->pinctrl, ak4376_data->hifi_rst_lo_state);
		usleep_range(10*1000, 10*1000);
		pinctrl_select_state(ak4376_data->pinctrl, ak4376_data->hifi_rst_hi_state);
		usleep_range(2000, 3000);
	}
#endif
	return 0;
}

int ak4376_mute(bool mute)
{

	return 0;
}

static int do_reg_write_check(u8 reg, u8 val)
{
	u8 reg_val = 0;
	int retry = 5;
	int ret = 0;
	char logbuf[100];

	reg_val = ak4376_i2c_read_byte(reg);
	while ((val != reg_val) && (retry > 0)) {
		ak4376_i2c_write_byte(reg, val);
		reg_val = ak4376_i2c_read_byte(reg);
		retry--;
	}
	if (retry == 5)
		ret = 0;
	else if ((retry >= 0) && (val == reg_val)) {
		snprintf(logbuf, sizeof(logbuf), "Write 0x%2x to 0x%2x success, retry %d times\n",
			val, reg, 5 - retry);
#ifdef CONFIG_FMEA
		fmea_notify("ak4376", logbuf);
#endif
		ret = 0;
	} else if (!retry) {
		snprintf(logbuf, sizeof(logbuf), "Write 0x%2x to 0x%2x failed, retry 5 times\n",
			val, reg);
#ifdef CONFIG_FMEA
		fmea_notify("ak4376", logbuf);
#endif
		ret = -1;
	}
	return ret;
}

static int ak4376_setting_special_params(struct ak4376_params *params)
{
	int ret = 0;

	pr_info("[HiFi] params->mode %d\n", params->mode);
	if (params->mode < 0) {
		pr_err("[HiFi] no params to set\n");
		return 0;
	}

	ret = params->mode;
	return ret;
}

int ak4376_enable(struct audio_params *params, bool enable)
{
	int i, j;
	int ret = 0;
	int format = params->pcm_format;
	int sample_rate = params->rate;
	int reg_value, reg_addr;
	int mode_sel = 0;
	int hifi_mode_status;
	int *reg_vals;
#ifdef CONFIG_VIVO_CODEC_FOR_MTK
	struct ak4376_params *ak4376_params = NULL;
#endif

	if (ak4376_data->client == NULL) {
		pr_err("[HiFi] client is NULL \n ");
		return -EFAULT;
	}
	if (!ak4376_available) {
		pr_err("[HiFi] ak4376 not avail \n ");
		return 0;
	}

	if (params->private_params) {
		hifi_mode_status = ak4376_setting_special_params((struct ak4376_params *)(params->private_params));
#ifdef CONFIG_VIVO_CODEC_FOR_MTK
		ak4376_params = params->private_params;
#endif
	}

	switch (params->rate) {
	case 44100:
		sample_rate = 0;
		break;
	case 48000:
		sample_rate = 1;
		break;
	case 88200:
		sample_rate = 2;
		break;
	case 96000:
		sample_rate = 3;
		break;
	case 176400:
		sample_rate = 4;
		break;
	case 192000:
		sample_rate = 5;
		break;
#ifdef CONFIG_VIVO_CODEC_FOR_MTK
	case 16000:
		sample_rate = 6;
		break;
#endif
	default:
		sample_rate = 1;
		break;
	}

#ifndef CONFIG_VIVO_CODEC_FOR_MTK
	if (ak4376_version == AK4376_VERSION_0 && ((params->i2s_format &
			SND_SOC_DAIFMT_MASTER_MASK) == SND_SOC_DAIFMT_CBS_CFS)) {
		switch (params->rate) {
		case 48000:
			sample_rate = 6;
			break;
		case 96000:
			sample_rate = 7;
			break;
		case 192000:
			sample_rate = 8;
			break;
		default:
			break;
		}
	}
#endif

	pr_info("[HiFi]:enable %d, mode %d, format = %d, sample_rate = %d params->rate = %d\n",
			(int)enable, hifi_mode_status, format, sample_rate, params->rate);
	mutex_lock(&ak4376_data->lock);
	if (enable) {
		if (hifi_mode_status == 2) { /* shijianxing add: HiFi Mode enable */
			if (ak4376_data->hifi_mclk) {
				set_bit(STATUS_MCLK, ak4376_data->mclk_status_mask);
				clk_prepare_enable(ak4376_data->hifi_mclk);
				pr_info("[HiFi] HiFi MCLK prepare.\n");
			}
			usleep_range(2000, 2000);
			ak4376_reset(ak4376_data->rst_gpio);

			if (ak4376_version == ak4375_VERSION_A) {
				if (format == SNDRV_PCM_FORMAT_S24_LE) {
					if ((sample_rate >= 0) && (sample_rate <= 5))
						mode_sel = sample_rate + 5;
					else
						mode_sel = 6;
				} else
					mode_sel = 4;
			} else if (ak4376_version == AK4376_VERSION_0) {
				if (format == SNDRV_PCM_FORMAT_S24_LE) {
#ifndef CONFIG_VIVO_CODEC_FOR_MTK
					if ((sample_rate >= 0) && (sample_rate < 9))
						mode_sel = sample_rate + 11;    /* shijianxing add */
					else
						mode_sel = 11;                  /* shijianxing add for default 48K */
				} else
					mode_sel = 20;                      /* shijianxing add for 48K 16bits */
#else
				if ((sample_rate >= 0) && (sample_rate <= 5))
					mode_sel = sample_rate + 12; /* shijianxing add ??? */
				else
					mode_sel = 13; /* shijianxing add ??? */
			} else
				mode_sel = 11; /* shijianxing add ??? */
#endif
		} else {
			if (format == SNDRV_PCM_FORMAT_S24_LE) {
				if (sample_rate == 1)
					mode_sel = 1;
				else if (sample_rate == 3)
					mode_sel = 2;
				else if (sample_rate == 5)
					mode_sel = 3;

			} else
				mode_sel = 0;
		}
#ifdef CONFIG_VIVO_CODEC_FOR_MTK
		if (params->rate == 16000) /* for mtk voice call. */
			mode_sel = 18;
#endif
		pr_info("[HiFi]:mode_sel = %d,hifi_version = %d\n", mode_sel, ak4376_version);
		/*
		 * shijianxing add: must be set PMHPL and PMHPR bit to 0 before changing the mode
		 */
		if (ak4376_version == AK4376_VERSION_0) {
			ret = ak4376_i2c_write_byte(0x03, 0x00);
			if (ret < 0) {
				pr_err("[HiFi]:write regsister 0x03 failed\n");
				goto end;
			}
		}

		for (i = 0; i < 36; i++) {
			if (ak4376_version == AK4376_VERSION_0) {
				reg_addr = ak4376_reg_enable[i][0];
				reg_value = ak4376_reg_enable[i][1];
			} else {
				reg_addr = ak4375_reg_enable[i][0];
				reg_value = ak4375_reg_enable[i][1];
			}
			for (j = 0; j < 7; j++) {
				if (reg_addr_tbl[j] == reg_addr) {
					reg_value = reg_value_tbl[mode_sel][j];
					break;
				}
			}
			/* shijianxing add:volume setting by project dtsi,default:0x0B,0dB */
			if (reg_addr == 0x0d && ak4376_data->volume > 0) {
				reg_value = ak4376_data->volume;
					pr_info("[HiFi]:volume = 0x%x\n", reg_value);
			}

			if (reg_addr == 0xff) {
				usleep_range(2000, 2000);
				continue;
			}

			for (j = 0; j < ak4376_data->reg_size; j++) {
				reg_vals = ak4376_data->reg_table;
				if (reg_vals[2 * j] == reg_addr) {
					reg_value = reg_vals[2 * j + 1];
						pr_info("[HiFi]: + new addr 0x%x val 0x%x.\n", reg_addr, reg_value);
					break;
				}
			}

			ret = ak4376_i2c_write_byte(reg_addr, reg_value);
			if (ret < 0) {
					pr_err("[HiFi]:check i2c addr= 0x%x,value= 0x%x\n", reg_addr, reg_value);
				goto end;
			}
			ret = do_reg_write_check(reg_addr, reg_value);
			if (ret < 0) {
					pr_err("[HiFi]:check i2c addr= 0x%x,value= 0x%x\n", reg_addr, reg_value);
				goto end;
			}
		}
		/* msleep(26); */
		/* shijianxing add for HPL/HPR Amplifier power on time,25.9ms@44.1k,23.9ms@other samplerates */
		ret = 0;
	} else if (hifi_mode_status == 1) { /* shijianxing add: Low Power Mode enable */
		/* shijianxing add: Low Power Mode enable */
		/* shijianxing M:Low Power Mode SmapleRate always 48K,MCLK not need */
		if ((params->i2s_format & SND_SOC_DAIFMT_MASTER_MASK) == SND_SOC_DAIFMT_CBM_CFM) {
			if (ak4376_data->hifi_mclk) {
				set_bit(STATUS_MCLK, ak4376_data->mclk_status_mask);
				clk_prepare_enable(ak4376_data->hifi_mclk);
					pr_info("[HiFi] HiFi MCLK prepare.\n");
			}
			usleep_range(2000, 2000);

		}
		ak4376_reset(ak4376_data->rst_gpio);

		if (ak4376_version == ak4375_VERSION_A) {
			if (format == SNDRV_PCM_FORMAT_S24_LE) {
				if ((sample_rate >= 0) && (sample_rate <= 5))
					mode_sel = sample_rate + 5;
				else
					mode_sel = 6;
			} else
				mode_sel = 4;
		} else if (ak4376_version == AK4376_VERSION_0) {
			if (format == SNDRV_PCM_FORMAT_S24_LE) {
#ifndef CONFIG_VIVO_CODEC_FOR_MTK
				if ((sample_rate >= 0) && (sample_rate < 9))
					mode_sel = sample_rate + 11;    /* shijianxing add */
				else
					mode_sel = 11;                  /* shijianxing add for default 48K */
			} else
				mode_sel = 20;                          /* shijianxing add for 48K 16bits */
#else
			if ((sample_rate >= 0) && (sample_rate <= 5))
				mode_sel = sample_rate + 12;/* shijianxing add ??? */
			else
				mode_sel = 13;/* shijianxing add ??? */
		} else
			mode_sel = 11;/* shijianxing add ??? */
#endif
	} else {
		if (format == SNDRV_PCM_FORMAT_S24_LE) {
			if (sample_rate == 1)
				mode_sel = 1;
			else if (sample_rate == 3)
				mode_sel = 2;
			else if (sample_rate == 5)
				mode_sel = 3;
		} else
			mode_sel = 0;
	}
	pr_info("[HiFi]:mode_sel = %d,hifi_version = %d\n", mode_sel, ak4376_version);
	/*
	 * shijianxing add: must be set PMHPL and PMHPR bit to 0 before changing the mode
	 */
	if (ak4376_version == AK4376_VERSION_0) {
		ret = ak4376_i2c_write_byte(0x03, 0x00);
		if (ret < 0) {
			pr_err("[HiFi]:check i2c addr= 0x%x,value= 0x%x\n", reg_addr, reg_value);
			goto end;
		}
	}

	for (i = 0; i < 36; i++) {

		if (ak4376_version == AK4376_VERSION_0) {
			reg_addr = ak4376_normal_reg_enable[i][0];
			reg_value = ak4376_normal_reg_enable[i][1];
		} else    {
			reg_addr = ak4375_reg_enable[i][0];
			reg_value = ak4375_reg_enable[i][1];
		}

		for (j = 0; j < 7; j++) {
			if (reg_addr_tbl[j] == reg_addr) {
				reg_value = reg_value_tbl[mode_sel][j];
				break;
			}
		}
		if (reg_addr == 0xff) {
			usleep_range(2000, 2000);
			continue;
		}

		for (j = 0; j < ak4376_data->reg_size; j++) {
			reg_vals = ak4376_data->reg_table;
			if (reg_vals[2 * j] == reg_addr) {
				reg_value = reg_vals[2 * j + 1];
				pr_info("[HiFi]: + new addr 0x%x val 0x%x.\n", reg_addr, reg_value);
				break;
			}
		}

		ret = ak4376_i2c_write_byte(reg_addr, reg_value);
		if (ret < 0) {
			pr_err("[HiFi]:check i2c addr= 0x%x,value= 0x%x\n", reg_addr, reg_value);
			goto end;
		}
		ret = do_reg_write_check(reg_addr, reg_value);
		if (ret < 0) {
			pr_err("[HiFi]:check i2c addr= 0x%x,value= 0x%x\n", reg_addr, reg_value);
			goto end;
		}
	}
	/* shijianxing add for HPL/HPR Amplifier power on time,25.9ms@44.1k,23.9ms@other samplerates */
	/* msleep(26); */
	ret = 0;
}
#ifdef CONFIG_VIVO_CODEC_FOR_MTK
		if (ak4376_params->vol) {
			ak4376_i2c_write_byte(0x0B, ak4376_params->vol);
			ak4376_i2c_write_byte(0x0C, ak4376_params->vol);
		}

		if (params) {
			reg_value = ak4376_i2c_read_byte(0x15);
			if (params->i2s_format & SND_SOC_DAIFMT_CBS_CFS) {
				reg_value &= 0xEF;
				ak4376_i2c_write_byte(0x15, reg_value); /* set to slave */
			} else {
				reg_value |= 0x10;
				ak4376_i2c_write_byte(0x15, reg_value); /* set to master */
			}
		}
#endif
	} else {

		for (i = 0; i < 6; i++) {
			reg_addr = ak4376_reg_disable[i][0];
			reg_value = ak4376_reg_disable[i][1];
			if (reg_addr == 0xff) {
				usleep_range(2000, 2000);
				continue;
			}
			ret = ak4376_i2c_write_byte(reg_addr, reg_value);
			if (ret < 0) {
				pr_err("[HiFi]:check i2c addr= 0x%x,value= 0x%x\n", reg_addr, reg_value);
				goto end;
			}
			ret = do_reg_write_check(reg_addr, reg_value);
			if (ret < 0) {
				pr_err("[HiFi]:check i2c addr= 0x%x,value= 0x%x\n", reg_addr, reg_value);
				goto end;
			}
		}
#ifndef CONFIG_VIVO_CODEC_FOR_MTK
		gpio_direction_output(ak4376_data->rst_gpio, 0);
		usleep_range(5000, 5000);
		if (test_bit(STATUS_MCLK, ak4376_data->mclk_status_mask)) {
			if (ak4376_data->hifi_mclk) {
				clear_bit(STATUS_MCLK, ak4376_data->mclk_status_mask);
				clk_disable_unprepare(ak4376_data->hifi_mclk);
				pr_info("[HiFi] HiFi MCLK close.\n");
			}
		}
#else
		if (ak4376_data->pinctrl && ak4376_data->hifi_rst_lo_state)
			pinctrl_select_state(ak4376_data->pinctrl, ak4376_data->hifi_rst_lo_state);
#endif
		ret = 0;
	}
end:
	mutex_unlock(&ak4376_data->lock);
	pr_info("[HiFi]:exist\n");
	return ret;
}

#ifdef BBK_IQOO_AUDIO_DEBUG
static int ak4376_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t ak4376_debug_write(struct file *filp,
		   const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret = 0;
	unsigned int kbuf[2];
	char *temp;

	temp = kmalloc(cnt, GFP_KERNEL);
	if (!temp) {
		return -ENOMEM;
	}

	ret = copy_from_user(temp, (void __user *)ubuf, cnt);
	ret = sscanf(temp, "%x %x", &kbuf[0], &kbuf[1]);
	if (!ret) {
		kfree(temp);
		return -EFAULT;
	}

	pr_debug(KERN_INFO "kbuf[0]=%x,kbuf[1]=%x cnt =%d.\n", kbuf[0], kbuf[1], (int)cnt);

	if (kbuf[0] == 0xff) {
		pr_info("gpio =0 disable voltage convert \n");
	} else {

		pr_info("gpio =1 enable voltage convert \n");
	}

	ak4376_i2c_write_byte(kbuf[0], kbuf[1]);
	kfree(temp);

	return cnt;
}

static ssize_t ak4376_debug_read(struct file *file, char __user *buf,
		  size_t count, loff_t *pos)
{
	int i;
	const int size = 512;
	u8 data;
	char buffer[size];
	int n = 0;

	pr_info("[HiFi]:============caught hifi reg start =============\n");

	for (i = 0; i < sizeof(ak4376_regs); i++) {
		data = ak4376_i2c_read_byte(ak4376_regs[i]);
		n += scnprintf(buffer + n, size - n, "reg{0x%x}:0x%x\n", ak4376_regs[i], data);

		pr_info("[HiFi]:catch hifi reg[0x%x]:0x%x\n", ak4376_regs[i], data);
	}

	buffer[n] = 0;

	pr_info("[HiFi]:============caught hifi reg end =============\n");

	return simple_read_from_buffer(buf, count, pos, buffer, n);

}

static struct file_operations ak4376_debugfs_fops = {
	.open	= ak4376_debug_open,
	.read	= ak4376_debug_read,
	.write	= ak4376_debug_write,
};

static ssize_t ak4376_debug_i2c_read(struct file *file, char __user *buf,
		      size_t count, loff_t *pos)
{
	const int size = 512;
	char buffer[size];
	int n = 0;

	pr_info("[HiFi] debug i2c read enter.\n");

	n += scnprintf(buffer+n, size-n, "HiFi-0x%x %s\n", ak4376_data->client->addr,
							ak4376_VERSION_NONE != ak4376_version ? "OK" : "ERROR");
	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations ak4376_i2c_debugfs_fops = {
	.open = ak4376_debug_open,
	.read = ak4376_debug_i2c_read,
};

static void ak4376_debugfs_init(void)
{
	ak4376_debugfs_root = debugfs_create_dir("audio-ak4376", NULL);
	if (!ak4376_debugfs_root) {
		pr_err("[HiFi] debugfs create dir error\n");
	} else if (IS_ERR(ak4376_debugfs_root)) {
		pr_err("[HiFi] Kernel not support debugfs \n");
		ak4376_debugfs_root = NULL;
	}

	ak4376_debugfs_reg = debugfs_create_file("reg", 0644, ak4376_debugfs_root,
						NULL, &ak4376_debugfs_fops);
	if (!ak4376_debugfs_reg) {
		pr_err("[HiFi] debugfs create fail \n");
	}

	ak4376_debugfs_i2c = debugfs_create_file("i2c", 0444, ak4376_debugfs_root,
						NULL, &ak4376_i2c_debugfs_fops);
	if (!ak4376_debugfs_i2c) {
		pr_err("[HiFi] i2c create fail \n");
	}

	return ;
}

static void ak4376_debugfs_deinit(void)
{
	debugfs_remove(ak4376_debugfs_i2c);
	debugfs_remove(ak4376_debugfs_reg);
	debugfs_remove(ak4376_debugfs_root);
	return;
}
#endif

static void ak4376_reg_table(const struct device_node *np, struct ak4376_data *ak4376)
{
	int reg_size, ret;
	int *array;

	if (of_find_property(np, "vivo,reg_table", &reg_size)) {
		reg_size /= sizeof(int);
		if (!(reg_size % 2)) {
			array = kzalloc(sizeof(int) * reg_size, GFP_KERNEL);
			if (!array) {
				ak4376->reg_size = 0;
				pr_err("[HiFi] Out of Memory\n");
				return;
			}
			ret = of_property_read_u32_array(np, "vivo,reg_table", array, reg_size);
			if (!ret) {
				ak4376->reg_size = reg_size / 2;
				ak4376->reg_table = array;
				pr_info("[HiFi] special params \n");
				return;
			}
		}
	}
	ak4376->reg_size = 0;
	pr_info("[HiFi] no special params \n");
	return;
}

#ifdef CONFIG_VIVO_CODEC_FOR_MTK
static int ak4376_get_pinctrl_state(struct device *dev, struct ak4376_data *ak4376_data)
{
	int ret = 0;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_state;

	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		pr_err("Can't find select gpio pinctrl!.\n");
		return 0;
	}

	ak4376_data->pinctrl = pinctrl;
	pin_state = pinctrl_lookup_state(pinctrl, "hifi_vd_enable_hi");
	if (IS_ERR(pin_state)) {
		ret = PTR_ERR(pin_state);
		pr_err("Can't find gpio hifi_vd_enable_hi pinctrl!.\n");
	} else {
		ak4376_data->hifi_vd_hi_state = pin_state;
	}

	pin_state = pinctrl_lookup_state(pinctrl, "hifi_vd_enable_lo");
	if (IS_ERR(pin_state)) {
		ret = PTR_ERR(pin_state);
		pr_err("Can't find gpio hifi_vd_enable_lo pinctrl!.\n");
	} else {
		ak4376_data->hifi_vd_lo_state = pin_state;
	}

	pin_state = pinctrl_lookup_state(pinctrl, "hifi_rst_hi");
	if (IS_ERR(pin_state)) {
		ret = PTR_ERR(pin_state);
		pr_err("Can't find gpio hifi_rst_hi pinctrl!.\n");
	} else {
		ak4376_data->hifi_rst_hi_state = pin_state;
	}

	pin_state = pinctrl_lookup_state(pinctrl, "hifi_rst_lo");
	if (IS_ERR(pin_state)) {
		ret = PTR_ERR(pin_state);
		pr_err("Can't find gpio hifi_rst_lo pinctrl!.\n");
	} else {
		ak4376_data->hifi_rst_lo_state = pin_state;
	}
	/*
	pinctrl_select_state(pinctrl, ak4376_data->hifi_vd_hi_state);
	*/
	pinctrl_select_state(pinctrl, ak4376_data->hifi_rst_lo_state);
	usleep_range(1000, 1001);
	pinctrl_select_state(pinctrl, ak4376_data->hifi_rst_hi_state);

	return 0;
}
#endif

static int ak4376_dev_parse(struct device *dev, struct ak4376_data *ak4376_data)
{
	int ret = 0;

#ifndef CONFIG_VIVO_CODEC_FOR_MTK
	if (dev->of_node) {
		of_property_read_u32(dev->of_node, "bbk,vd-type", &ak4376_data->vd_type);
		of_property_read_u32(dev->of_node, "bbk,va-type", &ak4376_data->va_type);
		of_property_read_u32(dev->of_node, "vivo,hp-volume-control", &ak4376_data->volume);

		printk("[HiFi]:%d,%d,hp volume 0x%2x\n", ak4376_data->vd_type, ak4376_data->va_type, ak4376_data->volume);
		if (ak4376_data->vd_type > 0) {
			ak4376_data->vd_regulator = regulator_get(dev, "ext_dac_vd");
			if (IS_ERR(ak4376_data->vd_regulator)) {
				ret = PTR_ERR(ak4376_data->vd_regulator);
				printk("Regulator get failed vd ret=%d\n", ret);
				return ret;
			}
			if ((regulator_count_voltages(ak4376_data->vd_regulator) > 0)) {
				ret = regulator_set_voltage(ak4376_data->vd_regulator, 1800000,
												1800000);
				if (ret) {
					printk("regulator set_vtg failed rc=%d\n", ret);
					regulator_put(ak4376_data->vd_regulator);
					return ret;
				}
			}
		} else {
			ak4376_data->vd_ctrl_gpio = of_get_named_gpio(dev->of_node, "ext-dac-vd-enable-gpio", 0);
			if (ak4376_data->vd_ctrl_gpio >= 0) /* shijianxing add:if not exist,can't request */
				gpio_request(ak4376_data->vd_ctrl_gpio, "ak4376-vd-ctrl-gpio");
			printk("[HiFi]:vd_ctrl_gpio = %d\n", ak4376_data->vd_ctrl_gpio);
		}

		if (ak4376_data->va_type > 0) {
			ak4376_data->va_regulator = regulator_get(dev, "ext_dac_va");
			if (IS_ERR(ak4376_data->vd_regulator)) {
				ret = PTR_ERR(ak4376_data->vd_regulator);
				printk("Regulator get failed va ret=%d\n", ret);
				return ret;
			}
			if ((regulator_count_voltages(ak4376_data->vd_regulator) > 0)) {
				ret = regulator_set_voltage(ak4376_data->vd_regulator, 1800000,
											1800000);
				if (ret) {
					printk("regulator set_vtg failed ret=%d\n", ret);
					regulator_put(ak4376_data->vd_regulator);
					return ret;
				}
			}
		} else {
			ak4376_data->va_ctrl_gpio = of_get_named_gpio(dev->of_node, "ext-dac-va-enable-gpio", 0);
			if (ak4376_data->va_ctrl_gpio >= 0) /* shijianxing add:if not exist,can't request */
				gpio_request(ak4376_data->va_ctrl_gpio, "ak4376-va-ctrl-gpio");
			printk("[HiFi]:va_ctrl_gpio = %d\n", ak4376_data->va_ctrl_gpio);
		}
		ak4376_data->rst_gpio = of_get_named_gpio(dev->of_node, "ext-dac-rst-gpio", 0);
	}    	else {
		if (dev->platform_data) {
			/* do nothing */
		}
	}
	if (ak4376_data->vd_type > 0)
		ret = regulator_enable(ak4376_data->vd_regulator);
	else if (ak4376_data->pinctrl && ak4376_data->hifi_vd_hi_state)
		pinctrl_select_state(ak4376_data->pinctrl, ak4376_data->hifi_vd_hi_state);

	if (!ak4376_data->fun->hw_reset) { /* shijianxing add : gpio request already request by vivo-codec-pd1602.c */
		ret = gpio_request(ak4376_data->rst_gpio, "ak4376_rst");
		printk("[HiFi]:HiFi_rst = %d\n", ak4376_data->rst_gpio);
		if (ret < 0)
			printk("[HiFi]:(%d) gpio_request already, rc=%d\n",
			       ak4376_data->rst_gpio, ret);
	}
	/* parse regulator */
	ak4376_data->hifi_1v8_regulator = regulator_get(dev, "vivo,hifi-1v8");
	if (!ak4376_data->hifi_1v8_regulator || IS_ERR(ak4376_data->hifi_1v8_regulator))
		pr_err("get 1v8 regulator failed\n");
	else {
		if ((regulator_count_voltages(ak4376_data->hifi_1v8_regulator) > 0)
		    && regulator_can_change_voltage(ak4376_data->hifi_1v8_regulator)) {
			ret = regulator_set_voltage(ak4376_data->hifi_1v8_regulator, 1800000,
										1800000);
			if (ret) {
				pr_err("regulator set_vtg failed rc=%d\n", ret);
				regulator_put(ak4376_data->hifi_1v8_regulator);
			}
		}
	}
	/* this 1.8v supply need to be always on. */
	if (ak4376_data->hifi_1v8_regulator && !IS_ERR(ak4376_data->hifi_1v8_regulator))
		ret = regulator_enable(ak4376_data->hifi_1v8_regulator);
	msleep(5);
	/* parse gpios */
	ak4376_data->rst_gpio = of_get_named_gpio(dev->of_node, "ext-dac-rst-gpio", 0);
	ret = gpio_request(ak4376_data->rst_gpio, "ak4376-reset-gpio");
	if (ret < 0) {
		pr_err("[HiFi]:gpio %d for dac reset gpio request failed.\n",
							ak4376_data->rst_gpio);
	}
	pr_info("[HiFi]:rst_gpio=%d\n", ak4376_data->rst_gpio);

	ak4376_data->hifi_mclk = clk_get(dev, "ref_clk");

	if (IS_ERR(ak4376_data->hifi_mclk)) {
		pr_err("Get main 19.2M clock: %d\n", (int)PTR_ERR(ak4376_data->hifi_mclk));
	} else {
		pr_info("[HiFi]:get 19.2M clock suscess!!\n");
		clk_set_rate(ak4376_data->hifi_mclk, 19200000);
	}
#endif

	return ret;
}

static int ak4376_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct vivo_codec_function *ak4376_function = NULL;

	pr_info("[HiFi] start.\n");

#ifdef BBK_IQOO_AUDIO_DEBUG
	ak4376_debugfs_init();
#endif

	ak4376_function = get_vivo_codec_function();

	if (!ak4376_function) {
		pr_err("[HiFi] codec not ready \n");
		return -EPROBE_DEFER;
	}
	ak4376_data = kzalloc(sizeof(struct ak4376_data), GFP_KERNEL);
	if (!ak4376_data) {
		pr_err("[HiFi] kzalloc failed\n");
		return -ENOMEM;
	}
	ak4376_data->fun = ak4376_function;
	ak4376_data->fun->hifi_dac_enable = ak4376_enable;
	ak4376_data->fun->hifi_dac_mute = ak4376_mute;
	clear_bit(STATUS_MCLK, ak4376_data->mclk_status_mask);
#ifdef CONFIG_VIVO_CODEC_FOR_MTK
	ak4376_get_pinctrl_state(&client->dev, ak4376_data);
	ak4376_data->fun = ak4376_function;
#endif
	ak4376_dev_parse(&client->dev, ak4376_data);

	ak4376_data->client = client;
	ak4376_data->driver_name = ak4376_DEV_NAME;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		pr_err("[HiFi] i2c check funtion error \n");



	ak4376_reset(ak4376_data->rst_gpio);

	ret = ak4376_i2c_read_byte(0x15);
	if (ret < 0) {
		pr_err("[HiFi] probe device not exist\n");
		/* goto err_gpio; */
	}
	ret = ((ret & 0xe0) >> 5);
	pr_info("[HiFi] 0x15 = 0x%x\n", ret);
	switch (ret) {
	case 0:
		ak4376_version = ak4375_VERSION_0;
		pr_info("[HiFi] ak4375_VERSION_0\n");
		break;
	case 1:
		ak4376_version = ak4375_VERSION_A;
		pr_info("[HiFi] ak4375_VERSION_A\n");
		break;
	case 2:
		ak4376_version = AK4376_VERSION_0;
		pr_info("[HiFi] ak4376_VERSION_0\n");
		break;
	default:
		ak4376_version = AK4376_VERSION_0;
		pr_info("[HiFi] Unsupported device revision\n");
		break;
	}
	ak4376_available = true;
	mutex_init(&ak4376_data->lock);

	/* shijianxing add:reset gpio by probe finish */
	/* gpio_direction_output(ak4376_data->rst_gpio, 0); */

	ak4376_reg_table(client->dev.of_node, ak4376_data);

	ret = misc_register(&ak4376_phys_dev);
	if (ret < 0) {
		pr_err("[HiFi] hifi phys misc device register error\n");
		goto err_gpio;
	}
err_gpio:

	return ret;

}

static int ak4376_i2c_remove(struct i2c_client *client)
{
#ifdef BBK_IQOO_AUDIO_DEBUG
	ak4376_debugfs_deinit();
#endif
	return 0;
}

static void ak4376_i2c_shutdown(struct i2c_client *client)
{
#ifdef CONFIG_VIVO_CODEC_FOR_MTK
	ak4376_reset(0);
#else
	gpio_direction_output(ak4376_data->rst_gpio, 0);
	msleep(5);
	regulator_disable(ak4376_data->hifi_1v8_regulator);
#endif
	return;
}
/*
static int ak4376_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int ak4376_i2c_resume(struct i2c_client *client)
{
	return 0;
}
*/

#ifdef CONFIG_OF
static const struct of_device_id device_ak4376_of_match[] = {
	{ .compatible = "ak,ak4376", },
	{},
};
#else
#define device_ak4376_of_match 0
#endif

static const struct i2c_device_id ak4376_i2c_id[] = {
	{ "ak4376", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, ak4376_i2c_id);

static struct i2c_driver ak4376_i2c_driver = {
	.driver			= {
		.name		= "ak4376-codec",
		.owner		= THIS_MODULE,
		.of_match_table = device_ak4376_of_match,
	},
	.probe			= ak4376_i2c_probe,
	.remove			= ak4376_i2c_remove,
	.shutdown		= ak4376_i2c_shutdown,
#if 0
	.suspend		= ak4376_i2c_suspend,
	.resume			= ak4376_i2c_resume,
#endif
	.id_table		= ak4376_i2c_id,
};


module_i2c_driver(ak4376_i2c_driver);

MODULE_DESCRIPTION("ASoC ak4376 codec driver");
MODULE_AUTHOR("Houzheng.Shen <shenhouzheng@iqoo.com>");
MODULE_LICENSE("GPL");


