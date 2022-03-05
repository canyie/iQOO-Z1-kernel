/**************************************************************************
*  AW87319_Audio.c
*
*  Create Date :
*
*  Modify Date :
*
*  Create by   : AWINIC Technology CO., LTD
*
*  Version     : 0.9, 2016/02/15
**************************************************************************/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
//#include <linux/wakelock.h>
#include <linux/debugfs.h>
#include "vivo-codec-common.h"

#ifndef BBK_IQOO_AUDIO_DEBUG
#define BBK_IQOO_AUDIO_DEBUG
#endif


#define AW87319_I2C_NAME	"aw87319_pa"
#define AW87319_I2C_BUS		0
#define AW87319_I2C_ADDR	0x58

enum {
	AW87319_VERSION,
	AW87339_VERSION,
	AW873XX_VERSION_NONE
};
static int aw873xx_version = AW873XX_VERSION_NONE;

unsigned char aw87319_audio_reciver(void);
unsigned char aW87319_audio_speaker(void);
unsigned char aw87319_audio_off(void);

unsigned char aw87319_hw_on(void);
unsigned char aw87319_hw_off(void);
unsigned char aw87319_sw_on(void);
unsigned char aw87319_sw_off(void);

static ssize_t aw87319_get_reg(struct device *cd, struct device_attribute *attr, char *buf);
static ssize_t aw87319_set_reg(struct device *cd, struct device_attribute *attr, const char *buf, size_t len);
static ssize_t aw87319_set_swen(struct device *cd, struct device_attribute *attr, const char *buf, size_t len);
static ssize_t aw87319_get_swen(struct device *cd, struct device_attribute *attr, char *buf);
static ssize_t aw87319_set_hwen(struct device *cd, struct device_attribute *attr, const char *buf, size_t len);
static ssize_t aw87319_get_hwen(struct device *cd, struct device_attribute *attr, char *buf);

static DEVICE_ATTR(reg, 0660, aw87319_get_reg,  aw87319_set_reg);
static DEVICE_ATTR(swen, 0660, aw87319_get_swen,  aw87319_set_swen);
static DEVICE_ATTR(hwen, 0660, aw87319_get_hwen,  aw87319_set_hwen);

struct i2c_client *aw87319_pa_client;
static int aw87319_rst;
static unsigned char aw87319_hwen_flag;
static bool chipid_ok;

#ifdef BBK_IQOO_AUDIO_DEBUG
static struct dentry *aw87319_debugfs_root;
static struct dentry *aw87319_debugfs_reg;
static struct dentry *aw87319_debugfs_i2c;
#endif

/* GPIO Control */

/* static int aw87319_pinctrl_init(struct device *dev)
{
	int ret = 0;
	aw87319ctrl = devm_pinctrl_get(dev);
	if (IS_ERR(aw87319ctrl)) {
		ret = PTR_ERR(aw87319ctrl);
		pr_info("%s devm_pinctrl_get fail!\n", __func__);
	}

	aw87319_rst_high = pinctrl_lookup_state(aw87319ctrl, "aw87319_rst_high");
	if (IS_ERR(aw87319_rst_high)) {
		ret = PTR_ERR(aw87319_rst_high);
		pr_info("%s : pinctrl err, aw87319_rst_high\n", __func__);
	}

	aw87319_rst_low = pinctrl_lookup_state(aw87319ctrl, "aw87319_rst_low");
	if (IS_ERR(aw87319_rst_low)) {
		ret = PTR_ERR(aw87319_rst_low);
		pr_info("%s : pinctrl err, aw87319_rst_low\n", __func__);
	}

	return ret;
} */

static void aw87319_pa_pwron(void)
{
	pr_info("%s: enter.\n", __func__);

	gpio_direction_output(aw87319_rst, 0);
	usleep_range(1000, 1050);
	gpio_direction_output(aw87319_rst, 1);
	usleep_range(10 * 1000, 10 * 1050);
	aw87319_hwen_flag = 1;

	pr_info("%s: leave.\n", __func__);
}

static void aw87319_pa_pwroff(void)
{
	pr_info("%s: enter\n", __func__);

	gpio_direction_output(aw87319_rst, 0);
	usleep_range(1000, 1050);
	aw87319_hwen_flag = 0;

	pr_info("%s: leave.\n", __func__);
}


/*
  i2c write and read
*/
unsigned char I2C_write_reg(unsigned char addr, unsigned char reg_data)
{
	char ret;
	u8 wdbuf[2] = {0};
	struct i2c_msg msgs[1];

	if (!aw87319_pa_client) {
		pr_err("%s: aw87319_pa_client is NULL\n",
			__func__);
		return -EINVAL;
	}

	msgs[0].addr = aw87319_pa_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = wdbuf;

	wdbuf[0] = addr;
	wdbuf[1] = reg_data;

	ret = i2c_transfer(aw87319_pa_client->adapter,
		msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		pr_err("%s: i2c read error: %d\n", __func__, ret);

	return ret;
}

unsigned char I2C_read_reg(unsigned char addr)
{
	unsigned char ret;
	u8 rdbuf;
	struct i2c_msg msgs[2];

	if (!aw87319_pa_client) {
		pr_err("%s: aw87319_pa_client is NULL\n",
			__func__);
		return -EINVAL;
	}

	msgs[0].addr = aw87319_pa_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &addr;

	msgs[1].addr = aw87319_pa_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = &rdbuf;

	ret = i2c_transfer(aw87319_pa_client->adapter,
		msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		pr_err("%s: i2c read error: %d\n",
			__func__, ret);

	return rdbuf;
}

/*
 AW87319 PA
*/
unsigned char aw87319_audio_reciver(void)
{
	pr_info("%s: enter, aw873xx_version: %d\n",
		__func__, aw873xx_version);

	aw87319_hw_on();

	if (AW87319_VERSION == aw873xx_version) {
		I2C_write_reg(0x01, 0x03);		/* Class D Enable; Boost Enable */

		I2C_write_reg(0x02, 0x28);		/* BATSAFE */
		I2C_write_reg(0x03, 0x03);		/* BOV */
		I2C_write_reg(0x04, 0x04);		/* BP */
		I2C_write_reg(0x05, 0x0D);		/* Gain */
		I2C_write_reg(0x06, 0x05);		/* AGC3_Po */
		I2C_write_reg(0x07, 0x52);		/* AGC3 */
		I2C_write_reg(0x08, 0x08);		/* AGC2 */
		I2C_write_reg(0x09, 0x02);		/* AGC1 */

		I2C_write_reg(0x01, 0x07);		/* CHIP Enable; Class D Enable; Boost Enable */
	} else if (AW87339_VERSION == aw873xx_version) {
		I2C_write_reg(0x01, 0x03);		/* Class D Enable; Boost Enable */

		I2C_write_reg(0x02, 0xAf);		/* BATSAFE */
		I2C_write_reg(0x03, 0x08);		/* BOV */
		I2C_write_reg(0x04, 0x05);		/* BP */
		I2C_write_reg(0x05, 0x07);		/* Gain */
		I2C_write_reg(0x06, 0x0f);		/* AGC3_Po */
		I2C_write_reg(0x07, 0x52);		/* AGC3 */
		I2C_write_reg(0x08, 0x09);		/* AGC2 */
		I2C_write_reg(0x09, 0x08);		/* AGC1 */
		I2C_write_reg(0x0A, 0x97);

		I2C_write_reg(0x01, 0x0a);		/* CHIP Enable; Class D Enable; Boost Enable */
	}

	return 0;
}

unsigned char aW87319_audio_speaker(void)
{
	pr_info("%s: enter, aw873xx_version: %d\n",
		__func__, aw873xx_version);

	aw87319_hw_on();

	if (AW87319_VERSION == aw873xx_version) {
		I2C_write_reg(0x01, 0x03);		/* Class D Enable; Boost Enable */

		I2C_write_reg(0x02, 0x28);		/* BATSAFE */
		I2C_write_reg(0x03, 0x05);		/* BOV */
		I2C_write_reg(0x04, 0x04);		/* BP */
		I2C_write_reg(0x05, 0x0D);		/* Gain */
		I2C_write_reg(0x06, 0x06);		/* AGC3_Po */
		I2C_write_reg(0x07, 0x52);		/* AGC3 */
		I2C_write_reg(0x08, 0x68);		/* AGC2 */
		I2C_write_reg(0x09, 0x02);		/* AGC1 */

		I2C_write_reg(0x01, 0x07);		/* CHIP Enable; Class D Enable; Boost Enable */
	} else if (AW87339_VERSION == aw873xx_version) {
		I2C_write_reg(0x01, 0x03);		/* Class D Enable; Boost Enable */

		I2C_write_reg(0x02, 0xA3);		/* BATSAFE */
		I2C_write_reg(0x03, 0x04);		/* BOV */
		I2C_write_reg(0x04, 0x05);		/* BP */
		I2C_write_reg(0x05, 0x0c);		/* Gain */
		I2C_write_reg(0x06, 0x09);		/* AGC3_Po */
		I2C_write_reg(0x07, 0x52);		/* AGC3 */
		I2C_write_reg(0x08, 0x07);		/* AGC2 */
		I2C_write_reg(0x09, 0x08);		/* AGC1 */
		I2C_write_reg(0x0A, 0x96);

		I2C_write_reg(0x01, 0x0E);		/* CHIP Enable; Class D Enable; Boost Enable */
	}

	return 0;
}

unsigned char aw87319_audio_off(void)
{
	pr_info("%s: enter, flag: %d\n",
		__func__, aw87319_hwen_flag);

	if (aw87319_hwen_flag) {
		I2C_write_reg(0x01, 0x00);
	}

	aw87319_hw_off();

	return 0;
}

static int aw87319_pa_enble(int state)
{
	int ret = 0;

	pr_info("%s: enter, state: %d\n",
		__func__, state);

	switch (state) {
	case EXT_PA_SWICH_VOICE:
		ret = aw87319_audio_reciver();
		break;
	case EXT_PA_SWICH_MUSIC:
	case EXT_PA_SWICH_FM:
		ret = aW87319_audio_speaker();
		break;
	case EXT_PA_SWICH_NONE:
		ret = aw87319_audio_off();
		break;
	default:
		ret = aw87319_audio_off();
		break;
	}

	pr_info("%s: leave.\n", __func__);

	return ret;
}

/*
 AW87319 Debug
*/
unsigned char aw87319_sw_on(void)
{
	unsigned char reg;
	reg = I2C_read_reg(0x01);
	reg |= 0x04;
	I2C_write_reg(0x01, reg);		/*  CHIP Enable; Class D Enable; Boost Enable */

	return 0;
}

unsigned char aw87319_sw_off(void)
{
	unsigned char reg;
	reg = I2C_read_reg(0x01);
	reg &= 0xFB;
	I2C_write_reg(0x01, reg);		/*  CHIP Disable */

	return 0;
}

unsigned char aw87319_hw_on(void)
{
	aw87319_pa_pwron();
	I2C_write_reg(0x64, 0x2C);

	return 0;
}

unsigned char aw87319_hw_off(void)
{
	aw87319_pa_pwroff();

	return 0;
}
/*
AW87319 Debug
*/
static ssize_t aw87319_get_reg(struct device *cd, struct device_attribute *attr, char *buf)
{
	unsigned char reg_val;
	ssize_t len = 0;
	u8 i;
	for (i = 0; i < 0x10; i++) {
		reg_val = I2C_read_reg(i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg%2X = 0x%2X, ", i, reg_val);
	}

	return len;
}

static ssize_t aw87319_set_reg(struct device *cd, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[2];
	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		I2C_write_reg(databuf[0], databuf[1]);
	}
	return len;
}

static ssize_t aw87319_get_swen(struct device *cd, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "aw87319_sw_on(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 1 > swen\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "aw87319_sw_off(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 0 > swen\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");

	return len;
}

static ssize_t aw87319_set_swen(struct device *cd, struct device_attribute *attr, const char *buf, size_t len)
{
	int databuf[16];

	sscanf(buf, "%d", &databuf[0]);
	if (databuf[0] == 0) {
		aw87319_sw_off();
	} else {
		aw87319_sw_on();
	}

	return len;
}

static ssize_t aw87319_get_hwen(struct device *cd, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "aw87319_hw_on(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 1 > hwen\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "aw87319_hw_off(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 0 > hwen\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");

	return len;
}

static ssize_t aw87319_set_hwen(struct device *cd, struct device_attribute *attr, const char *buf, size_t len)
{
	int databuf[16];

	sscanf(buf, "%d", &databuf[0]);
	if (databuf[0] == 0) {
		aw87319_hw_off();
	} else {
		aw87319_hw_on();
	}

	return len;
}


static int aw87319_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);
	err = device_create_file(dev, &dev_attr_swen);
	err = device_create_file(dev, &dev_attr_hwen);
	return err;
}

#ifdef BBK_IQOO_AUDIO_DEBUG
static int aw87319_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t aw87319_debug_write(struct file *filp,
					const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret = 0;
	unsigned int kbuf[2];
	char *temp;

	temp = kmalloc(cnt, GFP_KERNEL);
	if (!temp) {
		return -ENOMEM;
	}

	ret = copy_from_user(temp, ubuf, cnt);
	ret = sscanf(temp, "%x %x", &kbuf[0], &kbuf[1]);
	if (!ret) {
		kfree(temp);
		return -EFAULT;
	}

	pr_info("%s: kbuf[0]: %x, kbuf[1]: %x cnt: %d\n",
		__func__, kbuf[0], kbuf[1], (int)cnt);

	I2C_write_reg(kbuf[0], kbuf[1]);
	kfree(temp);

	return cnt;
}
static ssize_t aw87319_debug_read(struct file *file, char __user *buf,
					size_t count, loff_t *pos)
{
	unsigned char reg_val;
	const int size = 512;
	char buffer[size];
	int n = 0;
	u8 i;

	pr_info("%s:============caught aw87319 reg start =============\n", __func__);

	for (i = 0; i < 0x10; i++) {
		reg_val = I2C_read_reg(i);
		n += snprintf(buffer+n, size-n, "reg{%2x} = 0x%2x\n", i, reg_val);
		pr_info("%s:catch aw87319 reg[0x%x]:0x%x\n", __func__, i, reg_val);
	}

	buffer[n] = 0;

	pr_info("%s:============caught aw87319 reg end =============\n", __func__);

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations aw87319_debugfs_fops = {
	.open = aw87319_debug_open,
	.read = aw87319_debug_read,
	.write = aw87319_debug_write,
};

static ssize_t aw87319_debug_i2c_read(struct file *file, char __user *buf,
					size_t count, loff_t *pos)
{
	const int size = 512;
	char buffer[size];
	int n = 0;

	pr_info("%s enter.\n", __func__);

	n += scnprintf(buffer+n, size-n, "SmartPA-0x%x %s\n", aw87319_pa_client->addr,
				   chipid_ok ? "OK" : "ERROR");

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations aw87319_i2c_debugfs_fops = {
	.open = aw87319_debug_open,
	.read = aw87319_debug_i2c_read,
};

static void aw87319_debugfs_init(void)
{
	aw87319_debugfs_root = debugfs_create_dir("audio-aw87319", NULL);
	if (!aw87319_debugfs_root) {
		pr_err("%s debugfs create dir error\n", __func__);
	} else if (IS_ERR(aw87319_debugfs_root)) {
		pr_err("%s Kernel not support debugfs \n", __func__);
		aw87319_debugfs_root = NULL;
	}

	aw87319_debugfs_reg = debugfs_create_file("reg", 0644,
		aw87319_debugfs_root, NULL, &aw87319_debugfs_fops);
	if (!aw87319_debugfs_reg) {
		pr_err("aw87319 debugfs create fail \n");
	}
	aw87319_debugfs_i2c = debugfs_create_file("i2c", 0444,
		aw87319_debugfs_root, NULL, &aw87319_i2c_debugfs_fops);
	if (!aw87319_debugfs_i2c) {
		pr_err("aw87319 i2c create fail \n");
	}

	return ;
}

static void aw87319_debugfs_deinit(void)
{
	debugfs_remove(aw87319_debugfs_i2c);
	debugfs_remove(aw87319_debugfs_reg);
	debugfs_remove(aw87319_debugfs_root);
	return ;
}
#endif

static int aw87319_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct vivo_codec_function *vivo_codec_function = get_vivo_codec_function();
	unsigned char reg_value;
	unsigned char cnt = 5;
	int err = 0;
	int ret = 0;

	pr_info("%s: enter.\n", __func__);

	if (!vivo_codec_function) {
		pr_err("%s:vivo_codec_function malloc failed\n", __func__);
		//return -EPROBE_DEFER;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		//goto exit_check_functionality_failed;
	}

	/*  AW87319 RST Pin */
	aw87319_rst = of_get_named_gpio(client->dev.of_node, "awinic,ext_pa_spk_aw87319_rst", 0);
	if (aw87319_rst < 0) {
		err = -ENODEV;
		//goto exit_gpio_get_failed;
	}
	ret = gpio_request(aw87319_rst, "ext_pa_spk_aw87319_rst");
	if (ret) {
		pr_err("%s: gpio request failed for ext_pa_spk_aw87319_rst\n", __func__);
		err = -ENODEV;
		//goto exit_gpio_request_failed;
	}
	gpio_direction_output(aw87319_rst, 0);
	pr_info("%s: aw87319_rst = %d \n", __func__, aw87319_rst);


	aw87319_pa_client = client;

	aw87319_hw_on();
	msleep(10);

	while (cnt > 0) {
		I2C_write_reg(0x64, 0x2C);
		reg_value = I2C_read_reg(0x00);

		pr_info("AW873XX CHIPID: 0x%2x\n", reg_value);

		switch (reg_value) {
		case 0x9B:
			aw873xx_version = AW87319_VERSION;
			chipid_ok = true;
			pr_info("%s: AW87319_VERSION.\n", __func__);
			break;
		case 0x39:
			aw873xx_version = AW87339_VERSION;
			chipid_ok = true;
			pr_info("%s: AW87339_VERSION.\n", __func__);
			break;
		default:
			aw873xx_version = AW873XX_VERSION_NONE;
			pr_err("%s: Unsupported device revision\n",
				__func__);
			break;
		}

		cnt--;
		msleep(10);

		if (chipid_ok)
			break;
	}

	pr_info("%s: read chip ID remaining cnt: %d.\n",
		__func__, cnt);

	if (!cnt) {
		err = -ENODEV;
		aw87319_hw_off();
		//goto exit_create_singlethread;
	}

	vivo_codec_function->ext_pa_enable = aw87319_pa_enble;

	aw87319_create_sysfs(client);
#ifdef BBK_IQOO_AUDIO_DEBUG
	aw87319_debugfs_init();
#endif

	aw87319_hw_off();

	pr_info("%s: leave.\n", __func__);

	return 0;
/*
exit_create_singlethread:
	aw87319_pa_client = NULL;
exit_gpio_request_failed:
	gpio_free(aw87319_rst);
exit_gpio_get_failed:
exit_check_functionality_failed:
	if (vivo_codec_function) {
		vivo_codec_function->ext_pa_enable = NULL;
	}
*/
	return err;
}

static int aw87319_i2c_remove(struct i2c_client *client)
{
#ifdef BBK_IQOO_AUDIO_DEBUG
	aw87319_debugfs_deinit();
#endif

	aw87319_pa_client = NULL;
	return 0;
}

static const struct i2c_device_id aw87319_i2c_id[] = {
	{ AW87319_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw87319_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id extpa_of_match[] = {
	{.compatible = "awinic,aw87319_pa"},
	{},
};
MODULE_DEVICE_TABLE(of, extpa_of_match);
#endif

static struct i2c_driver aw87319_i2c_driver = {
	.driver = {
		.name	= AW87319_I2C_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = extpa_of_match,
#endif
	},
	.probe		= aw87319_i2c_probe,
	.remove		= aw87319_i2c_remove,
	.id_table	= aw87319_i2c_id,
};

/*
static int __init aw87319_pa_init(void)
{
	int ret;
	pr_info("%s Enter\n", __func__);

	ret = i2c_add_driver(&aw87319_i2c_driver);
	if (ret) {
		pr_err("****[%s] Unable to register driver (%d)\n", __func__, ret);
		return ret;
	}
	return 0;
}

static void __exit aw87319_pa_exit(void)
{
	pr_info("%s Enter\n", __func__);
	i2c_del_driver(&aw87319_i2c_driver);
}

module_init(aw87319_pa_init);
module_exit(aw87319_pa_exit);

*/

module_i2c_driver(aw87319_i2c_driver);

MODULE_AUTHOR("<liweilei@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC AW87319 PA driver");
MODULE_LICENSE("GPL");

