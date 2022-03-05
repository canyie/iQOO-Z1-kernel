
/*
 * Copyright (C) 2020 Samsung System LSI, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

//#define pr_fmt(fmt) "%s(): " fmt, __func__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/regmap.h>


#include <max20328-i2c.h>

#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug pr_info

#ifdef dev_dbg
#undef dev_dbg
#endif
#define dev_dbg dev_info

#define MAX20328B_VERSION	"v0.0"
unsigned int delay_bias = 30;
unsigned int delay_switch = 30;

char test_value[1024];
static struct i2c_client *local_i2c;
static int is_audio_adapter;

int max20328_is_ready;
int mmax_is_unuseirq;

static int regD_val, regE_val;

static struct max20328_priv *usbc_switch_mmax_priv;


static const struct regmap_config max20328_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX20328_REG_MAX,
};
#if IS_ENABLED(CONFIG_CHARGER_MCU_UNUSE)
enum dchg_switch_state {
	DCHG_SWITCH_OFF,
	DCHG_SWITCH_ON,
};
int max20328_get_dchg_switch_state(struct max20328_priv *mmax_priv)
{
	union power_supply_propval dchg_switch = {0,};

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: invalid parameter mmax_priv!\n", __func__);
		return;
	}

	if (IS_ERR_OR_NULL(mmax_priv->mcu_psy))
		mmax_priv->mcu_psy = power_supply_get_by_name("mcu");
	if (IS_ERR_OR_NULL(mmax_priv->mcu_psy)) {
		mmax_priv->dchg_uart_state = -1;
		pr_err("%s: failed\n", __func__);
	} else {
		POWER_SUPPLY_GET_PROPERTY(mmax_priv->mcu_psy,
			POWER_SUPPLY_PROP_USBSEL, &dchg_switch);
		mmax_priv->dchg_uart_state = dchg_switch.intval;
		pr_err("%s: dchg_uart_state:%d\n", __func__, mmax_priv->dchg_uart_state);
	}
	return mmax_priv->dchg_uart_state;
}
#endif

void max20328_usbc_set_switch_mode(int mode)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int data;
	int i;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: invalid parameter mmax_priv!\n", __func__);
		return;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: regmap invalid\n", __func__);
		return;
	}

	pr_info("%s: mode (%d)\n", __func__, mode);
#if IS_ENABLED(CONFIG_CHARGER_MCU_UNUSE)
	max20328_get_dchg_switch_state(mmax_priv);
#endif

	mutex_lock(&mmax_priv->usbc_switch_lock);
	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		switch (mode) {
		case POWER_SUPPLY_TYPEC_NONE: /* USB mode */
			is_audio_adapter = 0;
			if (mmax_priv->is_mostest) {
				regmap_write(mmax_priv->regmap, 0x0E, 0x40); /* DEF register2 set 00 */
				regmap_write(mmax_priv->regmap, 0x0D, 0x47); /* DEF register1 set TOP side closed in data connection, bottom side is open */
			} else {
				regmap_write(mmax_priv->regmap, 0x0E, 0x00); /* DEF register2 set 00 */
			#if IS_ENABLED(CONFIG_CHARGER_MCU_UNUSE)
				if (mmax_priv->dchg_uart_state == DCHG_SWITCH_ON) {
					regmap_write(mmax_priv->regmap, 0x0D, 0x10);
					pr_info("%s: dchg on ,keep (%d)\n", __func__, mode);
				} else
			#endif
					regmap_write(mmax_priv->regmap, 0x0D, 0x40); /* DEF register1 set TOP side closed in data connection, bottom side is open */
			}
			regmap_write(mmax_priv->regmap, 0x07, 0x00); /* CONTROL2 register, switch state NOT Force mode nor follow MODE[0:2] */
			regmap_write(mmax_priv->regmap, 0x08, 0x00); /* CONTROL3 register, force value is not use, anyway default it. */
			regmap_write(mmax_priv->regmap, 0x09, 0x30); /* ADC CONTROL1, ADC is always off on USB MODE */
			regmap_write(mmax_priv->regmap, 0x06, 0x13); /* CONTROL1 register, switch enable, default programmable with registers 0x0D and 0x0E */
			break;
		case POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER:
			is_audio_adapter = 1;
			regmap_write(mmax_priv->regmap, 0x0D, 0x03); /* DEF register */
			regmap_write(mmax_priv->regmap, 0x0E, 0x10); /* DEF register2 */
			regmap_write(mmax_priv->regmap, 0x07, 0x02); /* CONTROL2 register */
			regmap_write(mmax_priv->regmap, 0x08, 0x00); /* CONTROL3 register */
			regmap_write(mmax_priv->regmap, 0x09, 0x00); /* ADC CONTROL1, ADC is always off */
			regmap_write(mmax_priv->regmap, 0x06, 0x13); /* CONTROL1 register, switch enable, single Audio accessory */
			for (i = 0; i < sizeof(max20328_regs); i++) {
				regmap_read(mmax_priv->regmap, max20328_regs[i], &data);
				pr_info("%s: reg[0x%02x]: 0x%02x.\n", __func__, max20328_regs[i], data);
			}

			break;
		default:
			break;
		}
		break;
	case FSA4480:
		break;
	}

	mutex_unlock(&mmax_priv->usbc_switch_lock);
}
EXPORT_SYMBOL(max20328_usbc_set_switch_mode);

/* wangkai add */
// flag = true, charger Plug in, DP_T/DM_T pull down, disconnect DP_AP --> DP_T
// flag = false, charger Plug out, no need to set it
void max20328_switch_DPDM_low(bool flag)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;

	unsigned int val = 0;

	if (!flag)
		return;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return;
	}
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	regmap_read(mmax_priv->regmap, 0x0D, &val);
	printk("%s read 0x0D register is 0x%2x (before)\n", __func__, val);
	val = (val & 0x3f);
	printk("%s write 0x%2x to 0x0D register\n", __func__, val);
	regmap_write(mmax_priv->regmap, 0x0D, val);
	regmap_read(mmax_priv->regmap, 0x0D, &val);
	printk("%s read 0x0D register is 0x%2x (after)\n", __func__, val);
	mutex_unlock(&mmax_priv->usbc_switch_lock);
}
EXPORT_SYMBOL(max20328_switch_DPDM_low);
/* end */

static void max20328_usbc_switch_enable(struct max20328_priv *mmax_priv, bool enable)
{
	unsigned int val = 0, reg_06 = 0;
	int ret;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: regmap invalid\n", __func__);
		return;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	val = enable ? 0x13 : 0x03;
	ret = regmap_write(mmax_priv->regmap, 0x06, val);
	if (ret)
		pr_err("%s: failed %d\n", __func__, ret);
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	usleep_range(5 * 1000, 5 * 1000);

	mutex_lock(&mmax_priv->usbc_switch_lock);
	regmap_read(mmax_priv->regmap, 0x06, &reg_06);
	pr_err("%s: enable (%d) reg_0x06 (0x%x)\n",
			__func__, enable, reg_06);
	mutex_unlock(&mmax_priv->usbc_switch_lock);

}

int max20328_switch_mode_event(enum max_function event)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	int rc = 0 ;

	unsigned int val = 0;
	unsigned int val_0x0D = 0, val_0x0E = 0;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return -EINVAL;
	}
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		switch (event) {
		case MAX_MIC_GND_SWAP:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, 0x0D, &val);
				if ((val & 0x0f) == 0x07) {
					val = 0x03 | (val & 0xf0);
					regmap_write(mmax_priv->regmap, 0x0D, val);
					regmap_write(mmax_priv->regmap, 0x0E, 0x10);
				} else {
					val = 0x07 | (val & 0xf0);
					regmap_write(mmax_priv->regmap, 0x0D, val);
					regmap_write(mmax_priv->regmap, 0x0E, 0x40);
				}
			}
			break;
		case MAX_USBC_AUIDO_HP_ON:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, 0x0D, &val);
				val = 0xa0 | (val & 0x0f);
				regmap_write(mmax_priv->regmap, 0x0D, val);
			}
			regmap_read(mmax_priv->regmap, 0x0D, &regD_val);
			regmap_read(mmax_priv->regmap, 0x0E, &regE_val);
			break;
		case MAX_USBC_AUIDO_HP_OFF:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, 0x0D, &val);
				val = val & 0x0f;
				regmap_write(mmax_priv->regmap, 0x0D, val);
			}
			break;
		case MAX_USBC_ORIENTATION_CC1:
			regmap_write(mmax_priv->regmap, 0x06, 0x14);
			break;
		case MAX_USBC_ORIENTATION_CC2:
			regmap_write(mmax_priv->regmap, 0x06, 0x34);
			break;
		case MAX_USBC_DISPLAYPORT_DISCONNECTED:
			regmap_write(mmax_priv->regmap, 0x06, 0x14);
			break;
		case MAX_USBC_FAST_CHARGE_SELECT:
			if (!is_audio_adapter)
				regmap_write(mmax_priv->regmap, 0x0D, 0x10);
			break;
		case MAX_USBC_FAST_CHARGE_EXIT:
			if (!is_audio_adapter) {
				if (mmax_priv->is_mostest) {
					regmap_write(mmax_priv->regmap, 0x0D, 0x47);
				} else {
					regmap_write(mmax_priv->regmap, 0x0D, 0x40);
				}
			}

			break;
		case MAX_USBC_SWITCH_ENABLE:
			if (!is_audio_adapter)
				max20328_usbc_switch_enable(mmax_priv, true);
			break;
		case MAX_USBC_SWITCH_DISABLE:
			if (!is_audio_adapter)
				max20328_usbc_switch_enable(mmax_priv, false);
			break;
		default:
			break;
		}
		regmap_read(mmax_priv->regmap, 0x0D, &val_0x0D);
		regmap_read(mmax_priv->regmap, 0x0E, &val_0x0E);
		pr_info("%s: max20328: val_0x0D = 0x%x, val_0x0E = 0x%x, is_audio_adapter %d\n",
				__func__, val_0x0D, val_0x0E, is_audio_adapter);
		break;
	case FSA4480:
		pr_info("%s: fsa4480 set event\n", __func__);
		break;
	}

	mutex_unlock(&mmax_priv->usbc_switch_lock);
	return rc;
}
EXPORT_SYMBOL(max20328_switch_mode_event);

int max20328_switch_status_restore(void)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int data;
	int i;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return -EINVAL;
	}
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	regmap_write(mmax_priv->regmap, 0x0D, regD_val); /* DEF register */
	regmap_write(mmax_priv->regmap, 0x0E, regE_val); /* DEF register2 */
	regmap_write(mmax_priv->regmap, 0x07, 0x02); /* CONTROL2 register */
	regmap_write(mmax_priv->regmap, 0x08, 0x00); /* CONTROL3 register */
	regmap_write(mmax_priv->regmap, 0x09, 0x00); /* ADC CONTROL1, ADC is always off */
	regmap_write(mmax_priv->regmap, 0x06, 0x13); /* CONTROL1 register, switch enable, single Audio accessory */
	for (i = 0; i < sizeof(max20328_regs); i++) {
		regmap_read(mmax_priv->regmap, max20328_regs[i], &data);
		pr_info("%s: reg[0x%02x]: 0x%02x.\n", __func__, max20328_regs[i], data);
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);
	return 0;
}
EXPORT_SYMBOL(max20328_switch_status_restore);

int get_usbc_mg_status(void)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int val_0x0D = 0, val_0x0E = 0;
	int ret = -1;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: mmax container invalid\n", __func__);
		return ret;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return ret;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		regmap_read(mmax_priv->regmap, MAX20328_SW_DEFLT1, &val_0x0D);
		regmap_read(mmax_priv->regmap, MAX20328_SW_DEFLT2, &val_0x0E);
		pr_info("%s: max20328: val_0x0D: 0x%x, val_0x0E: 0x%x\n",
				__func__, val_0x0D, val_0x0E);
		if ((val_0x0D & 0xf) == 0x7) {
			ret = 1;
		} else if ((val_0x0D & 0xf) == 0x3) {
			ret = 2;
		}

		pr_err("%s: get_usbc_mg_status %d\n", __func__, ret);
		break;
	case FSA4480:
		pr_info("%s: fsa4480: val_sel\n", __func__);
		break;
	default:
		break;
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return ret;
}
EXPORT_SYMBOL(get_usbc_mg_status);

static void max20328_update_reg_defaults(struct regmap *regmap)
{
	u8 i;
//	unsigned int data;
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return;
	}
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		for (i = 0; i < ARRAY_SIZE(mmax_reg_i2c_defaults); i++)
			regmap_write(regmap, mmax_reg_i2c_defaults[i].reg,
					   mmax_reg_i2c_defaults[i].val);

		/*audio_v add for voice mos test*/
		if (mmax_priv->is_mostest) {
			regmap_write(mmax_priv->regmap, 0x0d, 0x47);
			regmap_write(mmax_priv->regmap, 0x0e, 0x40);
		}
		/*audio_v add for voice mos test end*/
		break;
	case FSA4480:
		/* i2c reset */
		pr_info("%s: FSA4480 update default reg.\n", __func__);
		break;
	default:
		break;
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);
	pr_info("legen-%s: update default reg ic = %s success.\n", __func__, mmax_priv->usbc_ic_mode == MAX20328 ? "MAX20328" : "FSA4480");
}

/*
 * max20328_notify_mos
 * audio_v : archer add for voice mos test
*/
int max20328_notify_mos(int state)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	if (IS_ERR_OR_NULL(mmax_priv))
		return -EINVAL;
	if (IS_ERR_OR_NULL(mmax_priv->regmap))
		return -EINVAL;

	mmax_priv->is_mostest = state;

	max20328_update_reg_defaults(mmax_priv->regmap);

	return 0;
}
EXPORT_SYMBOL(max20328_notify_mos);
static ssize_t max20328_reg_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *ubuf, size_t count)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int kbuf[2];
	char *temp;
	int ret = 0;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	temp = kmalloc(count, GFP_KERNEL);
	if (!temp) {
		return -ENOMEM;
	}

	ret = copy_from_user(temp, ubuf, count);
	ret = sscanf(temp, "%x %x", &kbuf[0], &kbuf[1]);
	if (!ret) {
		kfree(temp);
		return -EFAULT;
	}

	pr_info("%s: kbuf[0]: %x, kbuf[1]: %x cnt: %d\n",
		__func__, kbuf[0], kbuf[1], (int)count);

	if (kbuf[0] <= MAX20328_REG_MAX) {
		mutex_lock(&mmax_priv->usbc_switch_lock);
		regmap_write(mmax_priv->regmap, kbuf[0], kbuf[1]);
		mutex_unlock(&mmax_priv->usbc_switch_lock);
	} else {
		pr_err("%s: reg addr 0x%x out of range.\n", __func__, kbuf[0]);
	}

	kfree(temp);
	return count;
}

static ssize_t max20328_reg_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	const int size = 512;

	int n = 0, i;
	unsigned int data;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	for (i = 0; i < sizeof(max20328_regs); i++) {
		regmap_read(mmax_priv->regmap, max20328_regs[i], &data);
		n += scnprintf(ubuf+n, size-n, "reg[0x%02x]: 0x%02x\n", max20328_regs[i], data);
		pr_info("%s: reg[0x%02x]: 0x%02x.\n", __func__, max20328_regs[i], data);
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	ubuf[n] = 0;

	return n;
}

static ssize_t max20328_i2c_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	struct i2c_client *i2c = NULL;
	const int size = 512;
	int n = 0, ret = 0, reg_01 = 0;

	pr_info("%s: i2c read enter.\n", __func__);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->dev)) {
		pr_err("%s: Invalid client.\n ", __func__);
		return -EFAULT;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	i2c = to_i2c_client(mmax_priv->dev);

	mutex_lock(&mmax_priv->usbc_switch_lock);
	ret = regmap_read(mmax_priv->regmap, MAX20328_DEVICE_ID, &reg_01);
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	n += scnprintf(ubuf+n, size-n, "MAX20328-0x%x %s\n",
		i2c->addr, ((ret < 0) || !(reg_01 & 0x80)) ? "ERROR" : "OK");
	ubuf[n] = 0;
	return n;
}

static struct kobj_attribute dev_attr_reg =
	__ATTR(reg, 0664, max20328_reg_show, max20328_reg_store);
static struct kobj_attribute dev_attr_i2c =
	__ATTR(i2c, 0664, max20328_i2c_show, NULL);
static struct attribute *sys_node_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_i2c.attr,
	NULL
};
static struct attribute_group node_attribute_group = {
	.name = NULL,		/* put in device directory */
	.attrs = sys_node_attributes
};

static int class_attr_create(struct kobject *kobj)
{
	int ret = -1;
	char name[64];

	scnprintf(name, 48, "audio-max20328");

	kobj = kobject_create_and_add(name, kernel_kobj);
	if (!kobj) {
		pr_err("%s: kobject_create_and_add %s faild\n", __func__, name);
		return 0;
	}

	ret = sysfs_create_group(kobj, &node_attribute_group);
	if (ret) {
		kobject_del(kobj);
		kobj = NULL;
		pr_err("%s: sysfs_create_group %s faild\n", __func__, name);
	}

	pr_info("%s: sysfs create name successful\n", __func__, name);
	return ret;
}

static int class_attr_remove(struct kobject *kobj)
{
	if (kobj) {
		sysfs_remove_group(kobj, &node_attribute_group);
		kobject_del(kobj);
		kobj = NULL;
	}
	return 0;
}

static ssize_t reg_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	const int size = 1024;
	char *buffer = buf;
	int n = 0, i;
	unsigned int data;
	u8 *regs = max20328_regs;
	u8 regs_size = sizeof(max20328_regs);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		regs = max20328_regs;
		regs_size = sizeof(max20328_regs);
		break;
	case FSA4480:
		pr_info("%s: reg read FSA4480.\n", __func__);
		break;
	default:
		break;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	for (i = 0; i < regs_size; i++) {
		regmap_read(mmax_priv->regmap, regs[i], &data);
		n += scnprintf(buffer+n, size-n, "0x%02x: 0x%02x\n", regs[i], data);
		pr_info("%s: reg[0x%02x]: 0x%02x.\n", __func__, regs[i], data);
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	buffer[n] = 0;

	return n;
}

static ssize_t reg_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int kbuf[2];
	char *temp;
	int ret = 0;
	int reg_max = sizeof(max20328_regs);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	temp = kmalloc(count, GFP_KERNEL);
	if (!temp) {
		return -ENOMEM;
	}

	//ret = copy_from_user(temp, buf, count);
	memcpy(temp, buf, count);
	ret = sscanf(temp, "%x %x", &kbuf[0], &kbuf[1]);
	if (!ret) {
		kfree(temp);
		return -EFAULT;
	}

	pr_info("%s: kbuf[0]: %x, kbuf[1]: %x count: %d\n",
		__func__, kbuf[0], kbuf[1], (int)count);

	if (kbuf[0] <= reg_max) {
		mutex_lock(&mmax_priv->usbc_switch_lock);
		regmap_write(mmax_priv->regmap, kbuf[0], kbuf[1]);
		mutex_unlock(&mmax_priv->usbc_switch_lock);
	} else {
		pr_err("%s: reg addr 0x%x out of range.\n", __func__, kbuf[0]);
	}

	kfree(temp);
	return count;
}


static ssize_t i2c_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	struct i2c_client *i2c = NULL;
	const int size = 512;
	char *buffer = buf;
	unsigned int reg_01 = 0;
	int n = 0, ret = 0;

	pr_info("%s: i2c read enter.\n", __func__);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->dev)) {
		pr_err("%s: Invalid client.\n ", __func__);
		return -EFAULT;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	i2c = to_i2c_client(mmax_priv->dev);

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		mutex_lock(&mmax_priv->usbc_switch_lock);
		ret = regmap_read(mmax_priv->regmap, MAX20328_DEVICE_ID, &reg_01);
		mutex_unlock(&mmax_priv->usbc_switch_lock);
		n += scnprintf(buffer+n, size-n, "MAX20328-0x%x %s\n",
			i2c->addr, ((ret < 0) || !(reg_01 & 0x80)) ? "ERROR" : "OK");
		break;
	case FSA4480:
		pr_info("%s: i2c read FSA4480.\n", __func__);
		break;
	default:
		break;
	}

	buffer[n] = 0;

	return n;
}

static ssize_t switch_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	struct i2c_client *i2c = NULL;
	const int size = 512;
	char *buffer = buf;
	int n = 0, ret = 0;

	pr_info("%s: i2c read enter.\n", __func__);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->dev)) {
		pr_err("%s: Invalid client.\n ", __func__);
		return -EFAULT;
	}

	i2c = to_i2c_client(mmax_priv->dev);

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		n += scnprintf(buffer+n, size-n, "MAX20328-0x%x %s\n",
			i2c->addr, (ret < 0) ? "ERROR" : "OK");
		break;
	case FSA4480:
		pr_info("%s: switch read FSA4480.\n", __func__);
		break;
	default:
		break;
	}

	buffer[n] = 0;

	return n;

	//return sprintf(buf, "%d\n", foo);
}

static ssize_t mos_show (struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	const int size = 600;
	int n = 0;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	n += scnprintf(buffer+n, size-n, "getprop vendor.audio.vivo.mos.test : %s\n", mmax_priv->is_mostest ? "true":"false");

	buffer[n] = 0;
	return n;
}

static ssize_t mos_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *ubuf, size_t cnt)
{
	  struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	  unsigned int kbuf[2];
	  char *temp;
	  int ret = 0;

	  if (IS_ERR_OR_NULL(mmax_priv)) {
		  pr_err("%s: Invalid data.\n", __func__);
		  return 0;
	  }

	  if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		  pr_err("%s: Invalid regmap.\n ", __func__);
		  return 0;
	  }

	  temp = kmalloc(cnt, GFP_KERNEL);
	  if (!temp) {
		  return 0;
	  }

	  memcpy(temp, ubuf, cnt);
	  ret = sscanf(temp, "%x", &kbuf[0]);
	  if (!ret) {
		  kfree(temp);
		  return 0;
	  }

	  pr_info("%s: kbuf[0]: %x cnt: %d\n",
		  __func__, kbuf[0], (int)cnt);

	  if (kbuf[0] == true) {
		  mmax_priv->is_mostest = true;
	  } else {
		  mmax_priv->is_mostest = false;
	  }
	  max20328_update_reg_defaults(mmax_priv->regmap);

	  kfree(temp);
	  return cnt;
}


static ssize_t max20328_unuseirq_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	const int size = 512;
	int n = 0;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	n += scnprintf(ubuf+n, size-n, "is_unuseirq: %d\n", mmax_priv->is_unuseirq);

	ubuf[n] = 0;
	return n;
}

static ssize_t max20328_unuseirq_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *ubuf, size_t cnt)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int kbuf[2];
	char *temp;
	int ret = 0;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return 0;
	}

	temp = kmalloc(cnt, GFP_KERNEL);
	if (!temp) {
		return 0;
	}

	memcpy(temp, ubuf, cnt);
	ret = sscanf(temp, "%x", &kbuf[0]);
	if (!ret) {
		kfree(temp);
		return 0;
	}

	pr_info("%s: kbuf[0]: %x cnt: %d\n",
		__func__, kbuf[0], (int)cnt);

	mutex_lock(&mmax_priv->usbc_switch_lock);
	if (kbuf[0] == 1) {
		mmax_priv->is_unuseirq = 1;
		mmax_is_unuseirq = 1;
		#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
		if (get_typec_drp())
			vote_typec_drp(ACCDET_TCPC_VOTER, false);
		#endif
	} else {
		mmax_priv->is_unuseirq = 0;
		mmax_is_unuseirq = 0;
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	kfree(temp);
	return cnt;
}

/*
 * This module shows how to create a simple subdirectory in sysfs called
 * /sys/kernel/kobject-example  In that directory, 3 files are created:
 * "reg", "i2c", and "switch".  If an integer is written to these files, it can be
 * later read out of it.
 */

static struct kobj_attribute i2c_attribute =
	__ATTR(i2c, 0664, i2c_show, NULL);
static struct kobj_attribute switch_attribute =
	__ATTR(switch, 0664, switch_show, NULL);
static struct kobj_attribute reg_attribute =
	__ATTR(reg, 0664, reg_show, reg_store);
static struct kobj_attribute dev_attr_mos =
	__ATTR(mos, 0664, mos_show, mos_store);
static struct kobj_attribute dev_attr_unuseirq =
	__ATTR(unuseirq, 0664, max20328_unuseirq_show, max20328_unuseirq_store);

/*
* Create a group of attributes so that we can create and destroy them all
* at once.
*/
static struct attribute *attrs[] = {
	&i2c_attribute.attr,
	&switch_attribute.attr,
	&reg_attribute.attr,
	&dev_attr_mos.attr,
	&dev_attr_unuseirq.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory.  If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
static struct attribute_group attr_group = {
	.attrs = attrs,
};


static const struct i2c_device_id max20328b_i2c_id[] = {
	{ "max20328b", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max20328b_i2c_id);

static struct of_device_id max20328b_dt_match[] = {
	{ .compatible = "maxin,max20328",},
	{ },
};


static int max20328b_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct max20328_priv *mmax_priv;
	struct pinctrl *max_pinctrl;
	struct pinctrl_state *pins_rst;
	int err;
	int ret;

	mmax_priv = devm_kzalloc(&i2c->dev, sizeof(*mmax_priv),
				GFP_KERNEL);
	if (!mmax_priv)
		return -ENOMEM;


	mmax_priv->dev = &i2c->dev;
	mutex_init(&mmax_priv->usbc_switch_lock);

	usbc_switch_mmax_priv = mmax_priv;
	mmax_priv->is_unuseirq = 0;

	/*
	 * Create a simple kobject with the name of "kobject_example",
	 * located under /sys/kernel/
	 *
	 * As this is a simple directory, no uevent will be sent to
	 * userspace.  That is why this function should not be used for
	 * any type of dynamic kobjects, where the name and number are
	 * not known ahead of time.
	 */
	mmax_priv->switch_kobj = kobject_create_and_add("audio-max20328", kernel_kobj);
	if (!mmax_priv->switch_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	err = sysfs_create_group(mmax_priv->switch_kobj, &attr_group);
	if (err)
		kobject_put(mmax_priv->switch_kobj);

	local_i2c = i2c;

	if (of_find_property(mmax_priv->dev->of_node, "max,usbc-ic-mode", NULL)) {
		err = of_property_read_u32(mmax_priv->dev->of_node,
					"max,usbc-ic-mode", &mmax_priv->usbc_ic_mode);
		if (err < 0) {
			pr_info("%s: read property failed %d\n", __func__, err);
		} else {
			pr_info("%s: usbc_ic_mode:%s\n", __func__, mmax_priv->usbc_ic_mode == MAX20328 ? "MAX20328" : "FSA4480");
		}
	}

	max_pinctrl = devm_pinctrl_get(&i2c->dev);
	if (IS_ERR(max_pinctrl)) {
		ret = PTR_ERR(max_pinctrl);
		dev_notice(&i2c->dev, "get max_pinctrl fail.\n");
	} else {
		pins_rst = pinctrl_lookup_state(max_pinctrl, "typec_rst_set_state");
		if (IS_ERR(pins_rst)) {
			ret = PTR_ERR(pins_rst);
			dev_notice(&i2c->dev, "lookup rst pinctrl fail\n");
		} else {
			pinctrl_select_state(max_pinctrl, pins_rst);
			dev_notice(&i2c->dev, "lookup rst pinctrl set to gpio input mode\n");
		}
	}

	mmax_priv->regmap = devm_regmap_init_i2c(i2c, &max20328_regmap_config);
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Failed to initialize regmap:\n", __func__);
		if (!mmax_priv->regmap) {
			err = -EINVAL;
			pr_err("%s: Failed to initialize regmap: %d\n", __func__, err);
		}
		err = PTR_ERR(mmax_priv->regmap);
		pr_err("%s: Failed to initialize regmap ptr err: %d\n", __func__, err);
	}

	max20328_update_reg_defaults(mmax_priv->regmap);

	class_attr_create(mmax_priv->kobj);

	max20328_is_ready = 1;

	pr_info("%s Probe completed successfully! \n", __func__);

	return 0;

}

static int max20328b_i2c_remove(struct i2c_client *i2c)
{
	struct max20328_priv *mmax_priv = (struct max20328_priv *)i2c_get_clientdata(i2c);
	if (!mmax_priv)
		return -EINVAL;
	class_attr_remove(mmax_priv->kobj);

	kobject_put(usbc_switch_mmax_priv->switch_kobj);
	usbc_switch_mmax_priv = NULL;
	max20328_is_ready = 0;
	return 0;
}


static struct i2c_driver max20328b_i2c_driver = {
	.driver = {
		.name = "max20328-driver",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max20328b_dt_match),
	},
	.probe =    max20328b_i2c_probe,
	.remove =   max20328b_i2c_remove,
	.id_table = max20328b_i2c_id,
};

static int __init max20328b_i2c_init(void)
{
	int ret = 0;
	pr_info("MAX20328b driver version %s\n", MAX20328B_VERSION);

	ret = i2c_add_driver(&max20328b_i2c_driver);

	return ret;
}

module_init(max20328b_i2c_init);

static void __exit max20328b_i2c_exit(void)
{
	i2c_del_driver(&max20328b_i2c_driver);
}

module_exit(max20328b_i2c_exit);

MODULE_DESCRIPTION("ASoC MAX20328B driver");
MODULE_LICENSE("GPL");

