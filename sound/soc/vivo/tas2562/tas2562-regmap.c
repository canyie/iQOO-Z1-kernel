/*
 * ALSA SoC Texas Instruments TAS2562 High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2016 Texas Instruments, Inc.
 *
 * Author: saiprasad
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#ifdef CONFIG_TAS2562_REGMAP

//TODO: Only to debug
#define VIVO_PORT_SMARTPA
//#define CONFIG_DEBUG_FS

#define DEBUG 5
#define AT_MODE
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pm.h>

#include "tas2562.h"
#include "tas2562-codec.h"
#include "tas2562-misc.h"
#include "tas2562-calib.h"
#ifdef CONFIG_TAS25XX_ALGO
#include <dsp/smart_amp.h>
#endif /*CONFIG_TAS25XX_ALGO*/
#include <linux/debugfs.h>


static char pICN[] = {0x00, 0x00, 0x95, 0x2c};
static char pICN2[] = {0x00, 0x00, 0x2F, 0x2C};
static char delay_reg[] = {0x00, 0x00, 0x03, 0xc0};
static char delay_reg2[] = {0x00, 0x01, 0x77, 0x00};


#ifdef VIVO_PORT_SMARTPA
#include "smart_amp.h"
#include "smartpa-debug-common.h"

static struct tas2562_priv *tas2562_priv;
int smartpa_init_dbg(char *buffer, int size);
int smartpa_read_freq_dbg(char *buffer, int size);
void smartpa_read_prars_dbg(int temp[5], unsigned char addr);
void smartpa_get_client(struct i2c_client **client, unsigned char addr);
int smartpa_check_calib_dbg(void);
static bool smartpa_check_re(void);
static int smartpa_calib_get(uint32_t *calib_value);
static bool rdc_check_valid(uint32_t rdc, uint8_t iter);

#define STR_SZ_TAS 512

static void smartpa_set_re(uint32_t *calibRe);
#endif

static int tas2562_regmap_write(struct tas2562_priv *p_tas2562,
	unsigned int reg, unsigned int value)
{
	int nResult = 0;
	int retry_count = TAS2562_I2C_RETRY_COUNT;

	if (p_tas2562->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	while (retry_count--) {
		nResult = regmap_write(p_tas2562->regmap, reg,
			value);
		if (nResult >= 0)
			break;
		msleep(20);
	}
	if (retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas2562_regmap_bulk_write(struct tas2562_priv *p_tas2562,
	unsigned int reg, unsigned char *pData, unsigned int nLength)
{
	int nResult = 0;
	int retry_count = TAS2562_I2C_RETRY_COUNT;

	if (p_tas2562->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	while (retry_count--) {
		nResult = regmap_bulk_write(p_tas2562->regmap, reg,
			 pData, nLength);
		if (nResult >= 0)
			break;
		msleep(20);
	}
	if (retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas2562_regmap_read(struct tas2562_priv *p_tas2562,
	unsigned int reg, unsigned int *value)
{
	int nResult = 0;
	int retry_count = TAS2562_I2C_RETRY_COUNT;

	if (p_tas2562->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	while (retry_count--) {
		nResult = regmap_read(p_tas2562->regmap, reg,
			value);
		if (nResult >= 0)
			break;
		msleep(20);
	}
	if (retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas2562_regmap_bulk_read(struct tas2562_priv *p_tas2562,
	unsigned int reg, unsigned char *pData, unsigned int nLength)
{
	int nResult = 0;
	int retry_count = TAS2562_I2C_RETRY_COUNT;

	if (p_tas2562->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	while (retry_count--) {
		nResult = regmap_bulk_read(p_tas2562->regmap, reg,
			 pData, nLength);
		if (nResult >= 0)
			break;
		msleep(20);
	}
	if (retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas2562_regmap_update_bits(struct tas2562_priv *p_tas2562,
	unsigned int reg, unsigned int mask, unsigned int value)
{
	int nResult = 0;
	int retry_count = TAS2562_I2C_RETRY_COUNT;

	if (p_tas2562->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	while (retry_count--) {
		nResult = regmap_update_bits(p_tas2562->regmap, reg,
			mask, value);
		if (nResult >= 0)
			break;
		msleep(20);
	}
	if (retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas2562_change_book_page(struct tas2562_priv *p_tas2562,
	enum channel chn,
	int book, int page)
{
	int n_result = 0;


	if (chn&channel_left) {
		p_tas2562->client->addr = p_tas2562->mn_l_addr;
		if (p_tas2562->mn_l_current_book != book) {
			n_result = tas2562_regmap_write(p_tas2562,
				TAS2562_BOOKCTL_PAGE, 0);
			if (n_result < 0) {
				dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
					__func__, __LINE__, n_result);
				goto end;
			}
			p_tas2562->mn_l_current_page = 0;
			n_result = tas2562_regmap_write(p_tas2562,
				TAS2562_BOOKCTL_REG, book);
			if (n_result < 0) {
				dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
					__func__, __LINE__, n_result);
				goto end;
			}
			p_tas2562->mn_l_current_book = book;
		}

		if (p_tas2562->mn_l_current_page != page) {
			n_result = tas2562_regmap_write(p_tas2562,
				TAS2562_BOOKCTL_PAGE, page);
			if (n_result < 0) {
				dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
					__func__, __LINE__, n_result);
				goto end;
			}
			p_tas2562->mn_l_current_page = page;
		}
	}

	if ((chn&channel_right) && (p_tas2562->mn_channels == 2)) {
		p_tas2562->client->addr = p_tas2562->mn_r_addr;
		if (p_tas2562->mn_r_current_book != book) {
			n_result = tas2562_regmap_write(p_tas2562,
				TAS2562_BOOKCTL_PAGE, 0);
			if (n_result < 0) {
				dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
					__func__, __LINE__, n_result);
				goto end;
			}
			p_tas2562->mn_r_current_page = 0;
			n_result = tas2562_regmap_write(p_tas2562,
				TAS2562_BOOKCTL_REG, book);
			if (n_result < 0) {
				dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
					__func__, __LINE__, n_result);
				goto end;
			}
			p_tas2562->mn_r_current_book = book;
		}

		if (p_tas2562->mn_r_current_page != page) {
			n_result = tas2562_regmap_write(p_tas2562,
				TAS2562_BOOKCTL_PAGE, page);
			if (n_result < 0) {
				dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
					__func__, __LINE__, n_result);
				goto end;
			}
			p_tas2562->mn_r_current_page = page;
		}
	}

end:
	return n_result;
}

static int tas2562_dev_read(struct tas2562_priv *p_tas2562,
	enum channel chn,
	unsigned int reg, unsigned int *pValue)
{
	int n_result = 0;

	mutex_lock(&p_tas2562->dev_lock);

	n_result = tas2562_change_book_page(p_tas2562, chn,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (n_result < 0)
		goto end;

	if (chn == channel_left)
		p_tas2562->client->addr = p_tas2562->mn_l_addr;
	else if (chn == channel_right)
		p_tas2562->client->addr = p_tas2562->mn_r_addr;
	else
		dev_err(p_tas2562->dev, "%s, wrong channel number\n", __func__);

	n_result = tas2562_regmap_read(p_tas2562,
		TAS2562_PAGE_REG(reg), pValue);
	if (n_result < 0)
		dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
			__func__, __LINE__, n_result);
	else
		dev_dbg(p_tas2562->dev,
			"%s: chn:%x:BOOK:PAGE:REG %u:%u:%u,0x%x\n", __func__,
			p_tas2562->client->addr, TAS2562_BOOK_ID(reg),
			TAS2562_PAGE_ID(reg),
			TAS2562_PAGE_REG(reg), *pValue);

end:
	mutex_unlock(&p_tas2562->dev_lock);
	return n_result;
}

static int tas2562_dev_write(struct tas2562_priv *p_tas2562, enum channel chn,
	unsigned int reg, unsigned int value)
{
	int n_result = 0;

	mutex_lock(&p_tas2562->dev_lock);

	n_result = tas2562_change_book_page(p_tas2562, chn,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (n_result < 0)
		goto end;

	if (chn&channel_left) {
		p_tas2562->client->addr = p_tas2562->mn_l_addr;

		n_result = tas2562_regmap_write(p_tas2562,
			TAS2562_PAGE_REG(reg), value);
		if (n_result < 0)
			dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
				__func__, __LINE__, n_result);
		else
			dev_dbg(p_tas2562->dev,
			"%s: chn%x:BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
				__func__, p_tas2562->client->addr,
				TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), value);
	}

	if ((chn&channel_right) && (p_tas2562->mn_channels == 2)) {
		p_tas2562->client->addr = p_tas2562->mn_r_addr;

		n_result = tas2562_regmap_write(p_tas2562,
		TAS2562_PAGE_REG(reg),
			value);
		if (n_result < 0)
			dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
				__func__, __LINE__, n_result);
		else
			dev_dbg(p_tas2562->dev, "%s: chn%x:BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
				__func__, p_tas2562->client->addr,
				TAS2562_BOOK_ID(reg),
				TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), value);
	}

end:
	mutex_unlock(&p_tas2562->dev_lock);
	return n_result;
}

static int tas2562_dev_bulk_write(struct tas2562_priv *p_tas2562,
	enum channel chn,
	unsigned int reg, unsigned char *p_data, unsigned int n_length)
{
	int n_result = 0;

	mutex_lock(&p_tas2562->dev_lock);

	n_result = tas2562_change_book_page(p_tas2562, chn,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (n_result < 0)
		goto end;

	if (chn&channel_left) {
		p_tas2562->client->addr = p_tas2562->mn_l_addr;
		n_result = tas2562_regmap_bulk_write(p_tas2562,
			TAS2562_PAGE_REG(reg), p_data, n_length);
		if (n_result < 0)
			dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
				__func__, __LINE__, n_result);
		else
			dev_dbg(p_tas2562->dev, "%s: chn%x:BOOK:PAGE:REG %u:%u:%u, len: 0x%02x\n",
				__func__, p_tas2562->client->addr,
				TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), n_length);
	}

	if ((chn&channel_right) && (p_tas2562->mn_channels == 2)) {
		p_tas2562->client->addr = p_tas2562->mn_r_addr;
				n_result = tas2562_regmap_bulk_write(p_tas2562,
			TAS2562_PAGE_REG(reg), p_data, n_length);
		if (n_result < 0)
			dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
				__func__, __LINE__, n_result);
		else
			dev_dbg(p_tas2562->dev, "%s: %x:BOOK:PAGE:REG %u:%u:%u, len: 0x%02x\n",
				__func__, p_tas2562->client->addr,
				TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), n_length);
	}

end:
	mutex_unlock(&p_tas2562->dev_lock);
	return n_result;
}

static int tas2562_dev_bulk_read(struct tas2562_priv *p_tas2562,
	enum channel chn,
	unsigned int reg, unsigned char *p_data, unsigned int n_length)
{
	int n_result = 0;

	mutex_lock(&p_tas2562->dev_lock);

	if (chn == channel_left)
		p_tas2562->client->addr = p_tas2562->mn_l_addr;
	else if (chn == channel_right)
		p_tas2562->client->addr = p_tas2562->mn_r_addr;
	else
		dev_err(p_tas2562->dev, "%s, wrong channel number\n", __func__);

	n_result = tas2562_change_book_page(p_tas2562, chn,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (n_result < 0)
		goto end;

	n_result = tas2562_regmap_bulk_read(p_tas2562,
	TAS2562_PAGE_REG(reg), p_data, n_length);
	if (n_result < 0)
		dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
			__func__, __LINE__, n_result);
	else
		dev_dbg(p_tas2562->dev, "%s: chn%x:BOOK:PAGE:REG %u:%u:%u, len: 0x%02x\n",
			__func__, p_tas2562->client->addr,
			TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
			TAS2562_PAGE_REG(reg), n_length);
end:
	mutex_unlock(&p_tas2562->dev_lock);
	return n_result;
}

static int tas2562_dev_update_bits(struct tas2562_priv *p_tas2562,
	enum channel chn,
	unsigned int reg, unsigned int mask, unsigned int value)
{
	int n_result = 0;

	mutex_lock(&p_tas2562->dev_lock);
	n_result = tas2562_change_book_page(p_tas2562, chn,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (n_result < 0)
		goto end;

	if (chn&channel_left) {
		p_tas2562->client->addr = p_tas2562->mn_l_addr;
		n_result = tas2562_regmap_update_bits(p_tas2562,
			TAS2562_PAGE_REG(reg), mask, value);
		if (n_result < 0)
			dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
				__func__, __LINE__, n_result);
		else
			dev_dbg(p_tas2562->dev,
			"%s: chn%x:BOOK:PAGE:REG %u:%u:%u, mask: 0x%x, val: 0x%x\n",
				__func__, p_tas2562->client->addr,
				TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), mask, value);
	}

	if ((chn&channel_right) && (p_tas2562->mn_channels == 2)) {
		p_tas2562->client->addr = p_tas2562->mn_r_addr;
		n_result = tas2562_regmap_update_bits(p_tas2562,
			TAS2562_PAGE_REG(reg), mask, value);
		if (n_result < 0)
			dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
				__func__, __LINE__, n_result);
		else
			dev_dbg(p_tas2562->dev,
				"%s:chn%x:BOOK:PAGE:REG %u:%u:%u,mask: 0x%x, val: 0x%x\n",
				__func__, p_tas2562->client->addr,
				TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), mask, value);
	}

end:
	mutex_unlock(&p_tas2562->dev_lock);
	return n_result;
}

static bool tas2562_volatile(struct device *dev, unsigned int reg)
{
	return true;
}

static bool tas2562_writeable(struct device *dev, unsigned int reg)
{
	return true;
}
static const struct regmap_config tas2562_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas2562_writeable,
	.volatile_reg = tas2562_volatile,
	.cache_type = REGCACHE_NONE,
	.max_register = 1 * 128,
};


static void tas2562_hw_reset(struct tas2562_priv *p_tas2562)
{
	if (gpio_is_valid(p_tas2562->mn_reset_gpio)) {
		gpio_direction_output(p_tas2562->mn_reset_gpio, 0);

		if (p_tas2562->mn_channels != 1) {
			dev_dbg(p_tas2562->dev, "Reset gpio: not mono case, resetting second gpio");
			if (gpio_is_valid(p_tas2562->mn_reset_gpio2))
				gpio_direction_output(p_tas2562->mn_reset_gpio2, 0);
		} else {
			dev_dbg(p_tas2562->dev, "Reset gpio: mono case, not resetting second gpio");
		}
		msleep(20);

		gpio_direction_output(p_tas2562->mn_reset_gpio, 1);

		if (p_tas2562->mn_channels != 1) {
			dev_dbg(p_tas2562->dev, "Reset gpio: not mono case, resetting second gpio");
			if (gpio_is_valid(p_tas2562->mn_reset_gpio2))
				gpio_direction_output(p_tas2562->mn_reset_gpio2, 1);
		} else {
			dev_dbg(p_tas2562->dev, "Reset gpio: mono case, not resetting second gpio");
		}

		msleep(20);
	}
	dev_info(p_tas2562->dev, "reset gpio up !!\n");

	p_tas2562->mn_l_current_book = -1;
	p_tas2562->mn_l_current_page = -1;
	p_tas2562->mn_r_current_book = -1;
	p_tas2562->mn_r_current_page = -1;
}

void tas2562_enable_irq(struct tas2562_priv *p_tas2562, bool enable)
{
	static int irq1_enabled;
	static int irq2_enabled;
	struct irq_desc *desc = NULL;

	if (enable) {
		if (p_tas2562->mb_irq_eable)
			return;

		if (gpio_is_valid(p_tas2562->mn_irq_gpio) && irq1_enabled == 0) {
			desc = irq_to_desc(p_tas2562->mn_irq);
			if (desc && desc->depth > 0) {
				enable_irq(p_tas2562->mn_irq);
			} else {
				dev_info (p_tas2562->dev, "### irq already enabled");
			}
			irq1_enabled = 1;
		}
		if (gpio_is_valid(p_tas2562->mn_irq_gpio2) && irq2_enabled == 0) {
			enable_irq(p_tas2562->mn_irq2);
			irq2_enabled = 1;
		}

		p_tas2562->mb_irq_eable = true;
	} else {
		if (gpio_is_valid(p_tas2562->mn_irq_gpio) && irq1_enabled == 1) {
			disable_irq_nosync(p_tas2562->mn_irq);
			irq1_enabled = 0;
		}
		if (gpio_is_valid(p_tas2562->mn_irq_gpio2) && irq2_enabled == 1) {
			disable_irq_nosync(p_tas2562->mn_irq2);
			irq2_enabled = 0;
		}
		p_tas2562->mb_irq_eable = false;
	}
}

static int get_reg_from_DTS(int book, int page, int reg)
{
	int i;
	printk("%s, find book:page:reg=0x%02x:0x%02x:0x%02x 's value\n", __func__, book, page, reg);
	if (IS_ERR_OR_NULL(tas2562_priv)) {
		printk("%s, tas2562_priv is invalid\n", __func__);
		return 0; // use default value
	}

	if (IS_ERR_OR_NULL(tas2562_priv->reg_table) || tas2562_priv->reg_size <= 0) {
		printk("%s, reg_table or reg_size is invalid\n", __func__);
		return 0; // use default value
	}

	for (i = 0; i < tas2562_priv->reg_size; i++) {
		if (tas2562_priv->reg_table[i * 5] == book &&
			tas2562_priv->reg_table[i * 5 + 1] == page &&
			tas2562_priv->reg_table[i * 5 + 2] == reg) {
				printk("%s, this reg's value(0x%02x) find\n", __func__, tas2562_priv->reg_table[i * 5 + 4]);
				return tas2562_priv->reg_table[i * 5 + 4];
		}
	}

	printk("%s, this reg's value not find!!!\n", __func__);
	return 0;
}

static void irq_work_routine(struct work_struct *work)
{
	struct tas2562_priv *p_tas2562 =
		container_of(work, struct tas2562_priv, irq_work.work);
	unsigned int nDevInt1Status = 0, nDevInt2Status = 0,
		nDevInt3Status = 0, nDevInt4Status = 0;
	int n_counter = 2;
	int n_result = 0;
	int irqreg;
	enum channel chn;

	dev_info(p_tas2562->dev, "%s\n", __func__);
#ifdef CONFIG_TAS2562_CODEC
	mutex_lock(&p_tas2562->codec_lock);
#endif
	tas2562_enable_irq(p_tas2562, false);

	if (p_tas2562->mb_runtime_suspend) {
		dev_info(p_tas2562->dev, "%s, Runtime Suspended\n", __func__);
		goto end;
	}

	if (p_tas2562->mn_power_state == TAS2562_POWER_SHUTDOWN) {
		dev_info(p_tas2562->dev, "%s, device not powered\n", __func__);
		goto end;
	}

	n_result = p_tas2562->write(p_tas2562, channel_both,
				TAS2562_INTERRUPTMASKREG0,
				TAS2562_INTERRUPTMASKREG0_DISABLE);
	n_result = p_tas2562->write(p_tas2562, channel_both,
				TAS2562_INTERRUPTMASKREG1,
				TAS2562_INTERRUPTMASKREG1_DISABLE);

	if (n_result < 0)
		goto reload;

	if ((p_tas2562->spk_l_control == 1)
			&& (p_tas2562->spk_r_control == 1)
			&& (p_tas2562->mn_channels == 2))
		chn = channel_both;
	else if (p_tas2562->spk_l_control == 1)
		chn = channel_left;
	else if ((p_tas2562->spk_r_control == 1)
			&& (p_tas2562->mn_channels == 2))
		chn = channel_right;
	else
		chn = channel_left;

	dev_info(p_tas2562->dev, "%s, chn=%d.\n", __func__, chn);

	if (chn & channel_left)
		n_result = p_tas2562->read(p_tas2562, channel_left,
		TAS2562_LATCHEDINTERRUPTREG0, &nDevInt1Status);
	if (n_result >= 0)
		n_result = p_tas2562->read(p_tas2562, channel_left,
		TAS2562_LATCHEDINTERRUPTREG1, &nDevInt2Status);
	else
		goto reload;

	if (chn & channel_right)
		n_result = p_tas2562->read(p_tas2562, channel_right,
		TAS2562_LATCHEDINTERRUPTREG0, &nDevInt3Status);
	if (n_result >= 0)
		n_result = p_tas2562->read(p_tas2562, channel_right,
		TAS2562_LATCHEDINTERRUPTREG1, &nDevInt4Status);
	else
		goto reload;

	dev_dbg(p_tas2562->dev, "IRQ status : 0x%x, 0x%x, 0x%x, 0x%x\n",
			nDevInt3Status, nDevInt4Status,
			nDevInt3Status, nDevInt4Status);

	if (((nDevInt1Status & 0x7) != 0)
		|| ((nDevInt2Status & 0x0f) != 0) ||
		((nDevInt3Status & 0x7) != 0)
		|| ((nDevInt4Status & 0x0f) != 0)) {
/*		in case of INT_CLK, INT_OC, INT_OT,
 *		INT_OVLT, INT_UVLT, INT_BO
 */

		if ((nDevInt1Status &
	    TAS2562_LATCHEDINTERRUPTREG0_TDMCLOCKERRORSTICKY_INTERRUPT) ||
		(nDevInt3Status &
	    TAS2562_LATCHEDINTERRUPTREG0_TDMCLOCKERRORSTICKY_INTERRUPT)) {
			p_tas2562->mn_err_code |= ERROR_CLOCK;
			dev_err(p_tas2562->dev, "TDM clock error!\n");
	} else
		p_tas2562->mn_err_code &= ~ERROR_OVER_CURRENT;

	if ((nDevInt1Status &
		TAS2562_LATCHEDINTERRUPTREG0_OCEFLAGSTICKY_INTERRUPT) ||
		(nDevInt3Status &
		TAS2562_LATCHEDINTERRUPTREG0_OCEFLAGSTICKY_INTERRUPT)) {
		p_tas2562->mn_err_code |= ERROR_OVER_CURRENT;
		dev_err(p_tas2562->dev, "SPK over current!\n");
	} else
		p_tas2562->mn_err_code &= ~ERROR_OVER_CURRENT;

	if ((nDevInt1Status &
		TAS2562_LATCHEDINTERRUPTREG0_OTEFLAGSTICKY_INTERRUPT) ||
		(nDevInt3Status &
		TAS2562_LATCHEDINTERRUPTREG0_OTEFLAGSTICKY_INTERRUPT)) {
		p_tas2562->mn_err_code |= ERROR_DIE_OVERTEMP;
		dev_err(p_tas2562->dev, "die over temperature!\n");
	} else
		p_tas2562->mn_err_code &= ~ERROR_DIE_OVERTEMP;

	if ((nDevInt2Status &
	TAS2562_LATCHEDINTERRUPTREG1_VBATOVLOSTICKY_INTERRUPT) ||
		(nDevInt4Status &
	TAS2562_LATCHEDINTERRUPTREG1_VBATOVLOSTICKY_INTERRUPT)) {
		p_tas2562->mn_err_code |= ERROR_OVER_VOLTAGE;
		dev_err(p_tas2562->dev, "SPK over voltage!\n");
	} else
		p_tas2562->mn_err_code &= ~ERROR_UNDER_VOLTAGE;

	if ((nDevInt2Status &
	TAS2562_LATCHEDINTERRUPTREG1_VBATUVLOSTICKY_INTERRUPT) ||
		(nDevInt4Status &
	TAS2562_LATCHEDINTERRUPTREG1_VBATUVLOSTICKY_INTERRUPT)) {
		p_tas2562->mn_err_code |= ERROR_UNDER_VOLTAGE;
		dev_err(p_tas2562->dev, "SPK under voltage!\n");
	} else
		p_tas2562->mn_err_code &= ~ERROR_UNDER_VOLTAGE;

	if ((nDevInt2Status &
	TAS2562_LATCHEDINTERRUPTREG1_BROWNOUTFLAGSTICKY_INTERRUPT) ||
		(nDevInt4Status &
	TAS2562_LATCHEDINTERRUPTREG1_BROWNOUTFLAGSTICKY_INTERRUPT)) {
		p_tas2562->mn_err_code |= ERROR_BROWNOUT;
		dev_err(p_tas2562->dev, "brownout!\n");
	} else
		p_tas2562->mn_err_code &= ~ERROR_BROWNOUT;

		goto reload;
	} else {
		n_counter = 2;

	while (n_counter > 0) {
		if (chn & channel_left)
			n_result = p_tas2562->read(p_tas2562, channel_left,
				TAS2562_POWERCONTROL, &nDevInt1Status);
		if (n_result < 0)
			goto reload;
		if (chn & channel_right)
			n_result = p_tas2562->read(p_tas2562, channel_right,
			TAS2562_POWERCONTROL, &nDevInt3Status);
		if (n_result < 0)
			goto reload;

		if ((nDevInt1Status
			& TAS2562_POWERCONTROL_OPERATIONALMODE10_MASK)
			!= TAS2562_POWERCONTROL_OPERATIONALMODE10_SHUTDOWN) {
			/* If only left should be power on */
			if (chn == channel_left)
				break;
			/* If both should be power on */
			if ((nDevInt3Status
				& TAS2562_POWERCONTROL_OPERATIONALMODE10_MASK)
				!=
				TAS2562_POWERCONTROL_OPERATIONALMODE10_SHUTDOWN)
				break;
		}
		/*If only right should be power on */
		else if (chn == channel_right) {
			if ((nDevInt3Status
				& TAS2562_POWERCONTROL_OPERATIONALMODE10_MASK)
				!=
				TAS2562_POWERCONTROL_OPERATIONALMODE10_SHUTDOWN)
				break;
		}

			p_tas2562->read(p_tas2562, channel_left,
				TAS2562_LATCHEDINTERRUPTREG0, &irqreg);
			dev_info(p_tas2562->dev, "IRQ reg is: %s %d, %d\n",
				__func__, irqreg, __LINE__);
			p_tas2562->read(p_tas2562, channel_right,
				TAS2562_LATCHEDINTERRUPTREG0, &irqreg);
			dev_info(p_tas2562->dev, "IRQ reg is: %s %d, %d\n",
				__func__, irqreg, __LINE__);

			n_result = p_tas2562->update_bits(p_tas2562,
				chn, TAS2562_POWERCONTROL,
				TAS2562_POWERCONTROL_OPERATIONALMODE10_MASK,
				TAS2562_POWERCONTROL_OPERATIONALMODE10_ACTIVE);
			if (n_result < 0)
				goto reload;

			dev_info(p_tas2562->dev, "set ICN to -80dB\n");

			// Make sure Power Up is completed.
			msleep(2);
			n_result = p_tas2562->write(p_tas2562, channel_both,
					TAS2562_REG(0x0, 0x01, 0x21), 0x08);

			if (p_tas2562->spk_r_control) {
				n_result = p_tas2562->bulk_write(p_tas2562, chn, TAS2562_DELAY_REG, delay_reg2, 4);
				/* Solve the receiver low noise 0x10 -> 0x30 */
				n_result = p_tas2562->write(p_tas2562, channel_both,
						TAS2562_REG(0x0, 0x0, 0x3e), 0x30);
				n_result = p_tas2562->bulk_write(p_tas2562, chn, TAS2562_ICN_REG, pICN2, 4);
			} else {
				n_result = p_tas2562->bulk_write(p_tas2562, chn, TAS2562_DELAY_REG, delay_reg, 4);
				/* Solve the receiver low noise 0x10 */
				n_result = p_tas2562->write(p_tas2562, channel_both,
						TAS2562_REG(0x0, 0x0, 0x3e), 0x10);
				n_result = p_tas2562->bulk_write(p_tas2562, chn, TAS2562_ICN_REG, pICN, 4);
			}


			p_tas2562->read(p_tas2562, channel_left,
					TAS2562_LATCHEDINTERRUPTREG0, &irqreg);
			dev_info(p_tas2562->dev, "IRQ reg is: %s, %d, %d\n",
					__func__, irqreg, __LINE__);
			p_tas2562->read(p_tas2562, channel_right,
					TAS2562_LATCHEDINTERRUPTREG0, &irqreg);
			dev_info(p_tas2562->dev, "IRQ reg is: %s %d, %d\n",
					__func__, irqreg, __LINE__);

			n_counter--;
			if (n_counter > 0) {
	/* in case check pow status just after power on TAS2562 */
				dev_dbg(p_tas2562->dev, "PowSts B: 0x%x, check again after 10ms\n",
					nDevInt1Status);
				msleep(20);
			}
		}

		if ((((nDevInt1Status
			& TAS2562_POWERCONTROL_OPERATIONALMODE10_MASK)
			== TAS2562_POWERCONTROL_OPERATIONALMODE10_SHUTDOWN)
			&& (chn & channel_left))
			|| (((nDevInt3Status
			& TAS2562_POWERCONTROL_OPERATIONALMODE10_MASK)
			== TAS2562_POWERCONTROL_OPERATIONALMODE10_SHUTDOWN)
			&& (chn & channel_right))) {
			dev_err(p_tas2562->dev, "%s, Critical ERROR REG[0x%x] = 0x%x\n",
				__func__,
				TAS2562_POWERCONTROL,
				nDevInt1Status);
			p_tas2562->mn_err_code |= ERROR_CLASSD_PWR;
			goto reload;
		}
		p_tas2562->mn_err_code &= ~ERROR_CLASSD_PWR;
	}

	n_result = p_tas2562->write(p_tas2562, chn,
		TAS2562_INTERRUPTMASKREG0, 0xf8);
	if (n_result < 0)
		goto reload;

	n_result = p_tas2562->write(p_tas2562, chn,
		TAS2562_INTERRUPTMASKREG1, 0xb1);
	if (n_result < 0)
		goto reload;

	goto end;

reload:
	/* hardware reset and reload */
	tas2562_load_config(p_tas2562);

end:
	tas2562_enable_irq(p_tas2562, true);
#ifdef CONFIG_TAS2562_CODEC
	mutex_unlock(&p_tas2562->codec_lock);
#endif
}

static void init_work_routine(struct work_struct *work)
{
	struct tas2562_priv *p_tas2562 =
		container_of(work, struct tas2562_priv, init_work.work);
	int nResult = 0;
	//int irqreg;
	//dev_info(p_tas2562->dev, "%s\n", __func__);
#ifdef CONFIG_TAS2562_CODEC
	mutex_lock(&p_tas2562->codec_lock);
#endif

	p_tas2562->update_bits(p_tas2562, channel_both, TAS2562_POWERCONTROL,
		TAS2562_POWERCONTROL_OPERATIONALMODE10_MASK,
		TAS2562_POWERCONTROL_OPERATIONALMODE10_ACTIVE);

	nResult = gpio_get_value(p_tas2562->mn_irq_gpio);
	//dev_info(p_tas2562->dev, "%s, irq GPIO state: %d\n", __func__, nResult);

#ifdef CONFIG_TAS2562_CODEC
	mutex_unlock(&p_tas2562->codec_lock);
#endif
}

static irqreturn_t tas2562_irq_handler(int irq, void *dev_id)
{
	struct tas2562_priv *p_tas2562 = (struct tas2562_priv *)dev_id;

	/* get IRQ status after 100 ms */
	schedule_delayed_work(&p_tas2562->irq_work, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

static int tas2562_runtime_suspend(struct tas2562_priv *p_tas2562)
{
	dev_dbg(p_tas2562->dev, "%s\n", __func__);

	p_tas2562->mb_runtime_suspend = true;

	if (delayed_work_pending(&p_tas2562->irq_work)) {
		dev_dbg(p_tas2562->dev, "cancel IRQ work\n");
		cancel_delayed_work_sync(&p_tas2562->irq_work);
	}

	return 0;
}

static int tas2562_runtime_resume(struct tas2562_priv *p_tas2562)
{
	dev_dbg(p_tas2562->dev, "%s\n", __func__);

	p_tas2562->mb_runtime_suspend = false;

	return 0;
}

static int tas2562_pm_suspend(struct device *dev)
{
	struct tas2562_priv *p_tas2562 = dev_get_drvdata(dev);

	if (!p_tas2562) {
		dev_err(p_tas2562->dev, "drvdata is NULL\n");
		return -EINVAL;
	}

	mutex_lock(&p_tas2562->codec_lock);
	tas2562_runtime_suspend(p_tas2562);
	mutex_unlock(&p_tas2562->codec_lock);
	return 0;
}

static int tas2562_pm_resume(struct device *dev)
{
	struct tas2562_priv *p_tas2562 = dev_get_drvdata(dev);

	if (!p_tas2562) {
		dev_err(p_tas2562->dev, "drvdata is NULL\n");
		return -EINVAL;
	}

	mutex_lock(&p_tas2562->codec_lock);
	tas2562_runtime_resume(p_tas2562);
	mutex_unlock(&p_tas2562->codec_lock);
	return 0;
}

#ifdef VIVO_PORT_SMARTPA
#define CALIBRATE_FILE   "/mnt/vendor/persist/audio/smartamp.bin"
#define FREQ_FILE   "/data/engineermode/speakerleak"
#define MAX_CONTROL_NAME        48

static unsigned int tas256x_extra_regs[] = {
	TAS2562_REG(0x0, 0x2, 0x64),
	TAS2562_REG(0x0, 0x2, 0x65),
	TAS2562_REG(0x0, 0x2, 0x66),
	TAS2562_REG(0x0, 0x2, 0x67),
	TAS2562_REG(0x0, 0x2, 0x6c),
	TAS2562_REG(0x0, 0x2, 0x6d),
	TAS2562_REG(0x0, 0x2, 0x6e),
	TAS2562_REG(0x0, 0x2, 0x6f),
	TAS2562_REG(0x0, 0xfd, 0xd),
	TAS2562_REG(0x0, 0xfd, 0x47),
	TAS2562_REG(0x0, 0x1, 0x21)
};

#define REG_NUM 0x80
#define STR_BUF_SIZE 1024

#ifdef CONFIG_DEBUG_FS
static ssize_t smartpa_dbgfs_registers_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	int ret = 0;
	char *str;
	int ret_count = 0;
	uint8_t i = 0, channels = 1;
	int reg_val = 0;
	int str_remain = STR_BUF_SIZE * 2;


	channels = tas2562_priv->mn_channels;

	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: SmartPA_priv is NULL\n", __LINE__);
		return -1;
	}

	str = kmalloc(1024*2, GFP_KERNEL);
	if (!str) {
		pr_info("[SmartPA-%d]debugfs calibre: failed to kmalloc \n", __LINE__);
		ret = -ENOMEM;
		goto end;
	}

	pr_info("[SmartPA-%d]TAS2562 REG DUMP left channel: \n", __LINE__);
	ret = scnprintf(str+ret, str_remain, "reg left: \n");
	str_remain = str_remain - ret;
	for (i = 0; i < REG_NUM; i++) {
		tas2562_dev_read(tas2562_priv, channel_left, i, &reg_val);
		ret += scnprintf(str+ret, str_remain-ret, "%02x:%02x \n", i, reg_val);
	}
	ret += scnprintf(str+ret, str_remain, "extra reg left: \n");
	for (i = 0; i < ARRAY_SIZE(tas256x_extra_regs); i++) {
		tas2562_dev_read(tas2562_priv, channel_left, tas256x_extra_regs[i], &reg_val);
		ret += scnprintf(str+ret, str_remain-ret, "%02x:%02x \n", tas256x_extra_regs[i], reg_val);
	}

	if (channels > 1) {
		pr_info("[SmartPA-%d]TAS2562 REG DUMP right channel: \n", __LINE__);
		ret += scnprintf(str+ret, str_remain-ret, "reg right: \n");
		for (i = 0; i < REG_NUM; i++) {
			tas2562_dev_read(tas2562_priv, channel_right, i, &reg_val);
			ret += scnprintf(str+ret, str_remain-ret, "%02x:%02x \n", i, reg_val);
		}
		ret += scnprintf(str+ret, str_remain, "extra reg right: \n");
		for (i = 0; i < ARRAY_SIZE(tas256x_extra_regs); i++) {
			tas2562_dev_read(tas2562_priv, channel_right, tas256x_extra_regs[i], &reg_val);
			ret += scnprintf(str+ret, str_remain-ret, "%02x:%02x \n", tas256x_extra_regs[i], reg_val);
		}
	}
	ret_count = simple_read_from_buffer(user_buf, count, ppos, str, ret);
	kfree(str);
end:
	if (ret < 0) {
		return ret;
	} else {
		return ret_count;
	}

}
static ssize_t smartpa_dbgfs_registers_write(struct file *file,
		const char __user *user_buf, size_t count,
		loff_t *ppos)
{
	int channel;
	int rpage = 0;
	int reg = 0;
	int reg_val = 0;
	int rc;
	char *kbuf = NULL;

	pr_info("%s\n", __func__);
	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: SmartPA_priv is NULL\n", __LINE__);
		return -1;
	}

	kbuf = kmalloc(count + 1, GFP_KERNEL);
	if (kbuf == NULL)
		return -ENOMEM;

	rc = simple_write_to_buffer(kbuf, count, ppos, user_buf, count);
	if (rc != count) {
		kfree(kbuf);
		return rc >= 0 ? -EIO : rc;
	}

	rc = sscanf(kbuf, "%02x %02x %02x %02x", &channel, &rpage, &reg, &reg_val);
	if (channel == 0) {
		pr_info("left: page=%02x, reg=%02x, val: %02x.\n", rpage, reg, reg_val);
		tas2562_dev_write(tas2562_priv, channel_left, TAS2562_REG(0x0, rpage, reg), reg_val);
	}

	if (channel == 1) {
		pr_info("right: page=%02x, reg=%02x, val: %02x.\n", rpage, reg, reg_val);
		tas2562_dev_write(tas2562_priv, channel_right, TAS2562_REG(0x0, rpage, reg), reg_val);
	}

	kfree(kbuf);

end:
	return rc;
}
static ssize_t smartpa_dbgfs_calibrate_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	int ret = 0;
	uint32_t data = 0;
	uint8_t nSize = sizeof(uint32_t);
	uint32_t paramid = 0;
	uint32_t calib_re[MAX_CHANNELS];
	char *str;
	int ret_count = 0;
	uint8_t iter = 0, channels = 1;

	pr_info("%s\n", __func__);

	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: SmartPA_priv is NULL\n", __LINE__);
		return -1;
	}
	channels = tas2562_priv->mn_channels;

	//calib init
	for (iter = 0; iter < channels; iter++)	{
		data = 1;//Value is ignored
		paramid = ((AFE_SA_CALIB_INIT)|((iter+1)<<24)|(1<<16));
		ret = afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize);
		if (ret < 0)
			goto end;
	}
	pr_info("[SmartPA-%d]dbgfs_calibrate_read: calib init\n", __LINE__);

	msleep(2*1000);

	//get Re
	for (iter = 0; iter < channels; iter++)	{
		paramid = ((AFE_SA_GET_RE)|((iter+1)<<24)|(1<<16));
		ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_GET_PARAM, /*length */ 4);
		if (ret < 0)
			goto deinit;

		calib_re[iter] = data;
		pr_info("[SmartPa-%d]debugfs: calib_re-%d 0x%x\n", __LINE__, (int)iter, (int)calib_re[iter]);

		if ((calib_re[iter] < tas2562_priv->imped_min[iter])
			|| (calib_re[iter] > tas2562_priv->imped_max[iter]))
			calib_re[iter] = CALIB_FAILED;

		tas2562_priv->calibRe[iter] = calib_re[iter];
		pr_info("[SmartPA-%d]debugfs calib_re[%d] is %d\n", __LINE__, iter, calib_re[iter]);
	}

	str = kmalloc(STR_SZ_TAS*channels, GFP_KERNEL);
	if (!str) {
		pr_info("[SmartPA-%d]debugfs calibre: failed to kmalloc \n", __LINE__);
		ret = -ENOMEM;
		goto deinit;
	}
	ret = 0;
	if (channels == 2) {
		if (calib_re[0] == CALIB_FAILED)
			ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = 0x%x; ", calib_re[0]);
		else
			ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = %02d.%02d; ",
				TRANSF_IMPED_TO_USER_I(calib_re[0]), TRANSF_IMPED_TO_USER_M(calib_re[0]));
		if (calib_re[1] == CALIB_FAILED)
			ret += scnprintf(str+ret, STR_SZ_TAS-ret, "Channel[1] = 0x%x;\n", calib_re[1]);
		else
			ret += scnprintf(str+ret, STR_SZ_TAS-ret, "Channel[1] = %02d.%02d;\n",
				TRANSF_IMPED_TO_USER_I(calib_re[1]), TRANSF_IMPED_TO_USER_M(calib_re[1]));
	} else {
		if (calib_re[0] == CALIB_FAILED)
			ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = 0x%x;\n", calib_re[0]);
		else
			ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = %02d.%02d;\n",
				TRANSF_IMPED_TO_USER_I(calib_re[0]), TRANSF_IMPED_TO_USER_M(calib_re[0]));
	}
	ret_count = simple_read_from_buffer(user_buf, count, ppos, str, ret);
	pr_info("[SmartPA-%d]debugfs count %d, ret_count %d, ret %d\n",
			__LINE__, (int)count, (int)ret_count, (int)ret);
	kfree(str);

deinit:
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid = ((AFE_SA_CALIB_DEINIT)|((iter+1)<<24)|(1<<16));
		ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_SET_PARAM, nSize);
	}
	pr_info("[SmartPA-%d]dbgfs_calibrate_read: decalib init\n", __LINE__);

end:
	pr_info("[SmartPA-%d]dbgfs_calibrate_read: end\n", __LINE__);

	if (ret < 0) {
		return ret;
	} else {
		return ret_count;
	}
}

static const struct file_operations smartpa_dbgfs_register_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_registers_read,
	.write = smartpa_dbgfs_registers_write,
	.llseek = default_llseek,
};

static const struct file_operations smartpa_dbgfs_calibrate_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_calibrate_read,
	.llseek = default_llseek,
};
static ssize_t smartpa_dbgfs_impedance_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	return 0;
}

static const struct file_operations smartpa_dbgfs_impedance_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_impedance_read,
	.llseek = default_llseek,
};

static ssize_t smartpa_dbgfs_f0_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	u32 length = 1;
	uint32_t calibRe[MAX_CHANNELS];
	uint32_t F0[MAX_CHANNELS], Q[MAX_CHANNELS];
	int ret = 0, ret_count = 0;
	uint32_t data;
	mm_segment_t fs;
	uint32_t paramid = 0;
	int nSize = sizeof(uint32_t);
	char *str;
	struct file *fp = NULL;
	loff_t pos;
	uint8_t iter = 0, channels = 1;
	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);

	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: SmartPA_priv is NULL\n", __LINE__);
		return -1;
	}
	channels = tas2562_priv->mn_channels;

	//Load Calib
	if (smartpa_check_re()) {
		calibRe[0] = tas2562_priv->calibRe[0];
		if (channels == 2)
			calibRe[1] = tas2562_priv->calibRe[1];
		smartpa_set_re(calibRe);
	}

	for (iter = 0; iter < channels; iter++) {
		data = 1;//Value is ignored
		paramid = ((AFE_SA_F0_TEST_INIT) | (length << 16) | ((iter+1) << 24));
		ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_SET_PARAM, nSize);
		if (ret < 0)
			goto end;
	}
	//wait 5s
	msleep(5000);
	//read F0
	for (iter = 0; iter < channels; iter++)	{
		data = 0;//Resets data to 0
		paramid = (AFE_SA_GET_F0 | (length << 16) | ((iter+1) << 24));
		ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_GET_PARAM, /*length **/ 4);
		if (ret < 0)
			goto end;
		F0[iter] = data;
		pr_err("[SmartPA-%d]: F0[%d] is %d\n", __LINE__, iter, F0[iter]);
	}

	str = kmalloc(STR_SZ_TAS*channels, GFP_KERNEL);
	if (!str) {
		pr_info("[SmartPA-%d]debugfs calibre: failed to kmalloc \n", __LINE__);
		ret = -ENOMEM;
		goto deinit;
	}
	ret = 0;
	if (channels == 2) {
		ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = %d; Channel[1] = %d;\n", (F0[0] >> 19), (F0[1] >> 19));
	} else
		ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = %d;\n", (F0[0] >> 19));

	ret_count = simple_read_from_buffer(user_buf, count, ppos, str, ret);
	kfree(str);

	//read Q
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Reset data to 0
		paramid = (AFE_SA_GET_Q | (length << 16) | ((iter+1) << 24));
		ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_GET_PARAM, /*length **/ 4);
		if (ret < 0)
			goto end;
		Q[iter] = data;
		pr_err("[SmartPA-%d]: Q[%d] is %d\n", __LINE__, iter, Q[iter]);
	}

	//write to file
	fp = filp_open(CALIBRATE_FILE, O_RDWR | O_CREAT, 0644);
	if (fp > 0) {
		fs = get_fs();
		set_fs(KERNEL_DS);
		pos = 0;
		nSize = vfs_write(fp, (char *)&F0[0], sizeof(uint32_t), &pos);
		nSize = vfs_write(fp, (char *)&Q[0], sizeof(uint32_t), &pos);
		pr_info("[SmartPA-%d] write to file channel[0], F0 = %d, Q = %d\n", __LINE__, F0[0], Q[0]);
		if (channels == 2) {
			nSize = vfs_write(fp, (char *)&F0[1], sizeof(uint32_t), &pos);
			nSize = vfs_write(fp, (char *)&Q[1], sizeof(uint32_t), &pos);
			pr_info("[SmartPA-%d] write to file channel[1], F0 = %d, Q = %d\n", __LINE__, F0[1], Q[1]);
		}
		filp_close(fp, NULL);
		set_fs(fs);
	}

deinit:
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid = ((AFE_SA_F0_TEST_DEINIT) | (length << 16) | ((iter+1) << 24));
		ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid, TAS_SET_PARAM, nSize);
		if (ret < 0)
			goto end;
	}
end:
	if (ret < 0) {
		return ret;
	} else {
		return ret_count;
	}
}

static const struct file_operations smartpa_dbgfs_f0_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_f0_read,
	.llseek = default_llseek,
};
static ssize_t smartpa_dbgfs_temperature_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	return 0;
}

static const struct file_operations smartpa_dbgfs_temperature_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_temperature_read,
	.llseek = default_llseek,
};
static ssize_t smartpa_dbgfs_QFactor_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	return 0;
}

static const struct file_operations smartpa_dbgfs_QFactor_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_QFactor_read,
	.llseek = default_llseek,
};

static ssize_t smartpa_dbgfs_i2c_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tas2562_priv *tas2562 = i2c_get_clientdata(i2c);
	const int size = 512;
	char buffer[size];
	int n = 0;

	if (!tas2562)
		return -ENOMEM;

	pr_info("[SmartPA-%d]%s enter.\n", __LINE__, __func__);

	if (tas2562->mn_channels == 2) {
		n += scnprintf(buffer+n, size-n, "SmartPA-stereo %d\n",
				tas2562->smartpa_i2c_check);
	} else {
		n += scnprintf(buffer+n, size-n, "SmartPA-mono %s\n",
				tas2562->smartpa_i2c_check ? "OK" : "ERROR");
	}

	buffer[n] = 0;

	return simple_read_from_buffer(user_buf, count, ppos, buffer, n);
}

static const struct file_operations smartpa_dbgfs_i2c_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_i2c_read,
	.llseek = default_llseek,
};

static void smartpa_debug_init(struct tas2562_priv *tas2562, struct i2c_client *i2c)
{
	char name[60];

	scnprintf(name, MAX_CONTROL_NAME, "audio-%s", i2c->name);
	tas2562->dbg_dir = debugfs_create_dir(name, NULL);
	debugfs_create_file("reg", S_IRUGO|S_IWUGO, tas2562->dbg_dir,
			i2c, &smartpa_dbgfs_register_fops);
	debugfs_create_file("calibrate", S_IRUGO|S_IWUGO, tas2562->dbg_dir,
			i2c, &smartpa_dbgfs_calibrate_fops);
	debugfs_create_file("impedance", S_IRUGO|S_IWUGO, tas2562->dbg_dir,
			i2c, &smartpa_dbgfs_impedance_fops);
	debugfs_create_file("f0detect", S_IRUGO|S_IWUGO, tas2562->dbg_dir,
			i2c, &smartpa_dbgfs_f0_fops);
	debugfs_create_file("QFactor", S_IRUGO|S_IWUGO, tas2562->dbg_dir,
			i2c, &smartpa_dbgfs_QFactor_fops);
	debugfs_create_file("temperature", S_IRUGO|S_IWUGO, tas2562->dbg_dir,
			i2c, &smartpa_dbgfs_temperature_fops);
	debugfs_create_file("i2c", S_IRUGO|S_IWUGO, tas2562->dbg_dir,
			i2c, &smartpa_dbgfs_i2c_fops);
}

static void smartpa_debug_remove(struct tas2562_priv *tas2562)
{
	if (tas2562->dbg_dir)
		debugfs_remove_recursive(tas2562->dbg_dir);
}

#endif

static int smartpa_calib_save(uint32_t *calib_value)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;
	uint8_t channels  = 1;

	if (!tas2562_priv || !calib_value) {
		pr_err("[SmartPA-%d]: SmartPA_priv or calib_value is NULL\n", __LINE__);
		return -1;
	}
	channels = tas2562_priv->mn_channels;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(CALIBRATE_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("[SmartPA-%d]smartpa_calib_save: save calib_value[0]=%d \n", __LINE__, calib_value[0]);
		if (channels == 2)
			pr_info("[SmartPA-%d]smartpa_calib_save: save calib_value[1]=%d \n", __LINE__, calib_value[1]);
		vfs_write(pfile, (char *)calib_value, sizeof(uint32_t)*channels, &pos);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]smartpa_calib_save: %s open failed! \n", __LINE__, CALIBRATE_FILE);
		ret = -1;
	}

	set_fs(old_fs);

	return ret;
}

static void smartpa_set_re(uint32_t *calibRe)
{
	int nSize = sizeof(uint32_t);
	int ret;
	uint8_t iter = 0;
	uint32_t paramid = 0;

	if (!tas2562_priv || !calibRe) {
		pr_err("[SmartPA-%d]: tas2562_priv or calibRe is NULL\n", __LINE__);
		return;
	}
	//if((calibRe != NO_CALIB) && (calibRe != CALIB_FAILED)) {
	for (iter = 0; iter < tas2562_priv->mn_channels; iter++) {
		if (calibRe[iter] != 0) {
			pr_info("[SmartPA-%d]: smartamp : Payload : %d", __LINE__, calibRe[iter]);
			paramid = ((iter+1) << 24) | (nSize << 16) | AFE_SA_SET_RE;
			ret = afe_smartamp_algo_ctrl((uint8_t *)&calibRe[iter], paramid, TAS_SET_PARAM, nSize);
			pr_err("[SmartPA-%d]: set Re[%d]: %d", __LINE__, iter, calibRe[iter]);
		} else
			pr_err("[SmartPA-%d]: Cannot set Re for calib status wrong", __LINE__);
	}
}

int smartpa_init_dbg(char *buffer, int size)
{
	uint32_t calib_re[MAX_CHANNELS] = {0};
	uint32_t paramid = 0;
	int ret = 0, n = 0;
	uint32_t data = 0;
	bool done[MAX_CHANNELS] = {false};
	int nSize = sizeof(uint32_t);
	uint8_t iter = 0, channels = 1;

	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);

	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: tas2562_priv is NULL\n", __LINE__);
		return -1;
	}
	channels = tas2562_priv->mn_channels;
	if (channels == 1)
		done[1] = true;

	if (tas2562_priv->mb_power_up) {
		//calib init
		for (iter = 0; iter < channels; iter++) {
			data = 1;//Value is ignored
			paramid = ((AFE_SA_CALIB_INIT)|((iter+1)<<24)|(1<<16));
			ret = afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize);
			if (ret < 0) {
				done[iter] = false;
				pr_info("[SmartPA-%d]init_dbg:set calib_data error.\n", __LINE__);
				ret = -ENOMEM;
			}
		}
		pr_info("[SmartPA-%d]init_dbg: calib init\n", __LINE__);

		msleep(3 * 1000);

		//get Re
		for (iter = 0; iter < channels; iter++) {
			data = 0;//Reset data to 0
			paramid = ((AFE_SA_GET_RE)|((iter+1)<<24)|(1<<16));
			ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_GET_PARAM, /*length */ 4);
			if (ret < 0) {
				done[iter] = false;
				pr_info("[SmartPA-%d]init_dbg: decalib init\n", __LINE__);
			} else {
				calib_re[iter] = data;

				if ((calib_re[iter] < tas2562_priv->imped_min[iter]) || (calib_re[iter] > tas2562_priv->imped_max[iter]))
					done[iter] = false;
				else
					done[iter] = true;
				pr_info("[SmartPA-%d]init_dbg: calib_re is %d, valid range (%d %d)\n",
						__LINE__, calib_re[iter], tas2562_priv->imped_min[iter], tas2562_priv->imped_max[iter]);
			}
		}
	} else {
		done[0] = false;
		if (channels == 2)
			done[1] = false;
		ret = -EINVAL;
		pr_info("[SmartPA-%d]dbg init: failed to calibrate %d\n", __LINE__, ret);
	}

	n += scnprintf(buffer + n, size - n, "current status:[SmartPA] %s\n", (channels == 1) ? "Mono" : "Stereo");
	for (iter = 0; iter < channels; iter++) {
		n += scnprintf(buffer + n, size - n,
			"Channel[%d]: impedance %02d.%02d ohm, valid range(%02d.%02d ~ %02d.%02d ohm). \n", iter,
			TRANSF_IMPED_TO_USER_I(calib_re[iter]), TRANSF_IMPED_TO_USER_M(calib_re[iter]),
			TRANSF_IMPED_TO_USER_I(tas2562_priv->imped_min[iter]), TRANSF_IMPED_TO_USER_M(tas2562_priv->imped_min[iter]),
			TRANSF_IMPED_TO_USER_I(tas2562_priv->imped_max[iter]), TRANSF_IMPED_TO_USER_M(tas2562_priv->imped_max[iter]));
		pr_info("[SmartPA-%d]init_dbg: calibRe[%d] %d\n", __LINE__, iter, calib_re[iter]);
		if (!done[iter]) {
			calib_re[iter] = CALIB_FAILED;
		}
	}
	n += scnprintf(buffer + n, size - n, "\n Calibrate result: %s\n", (done[0] && done[1]) ? "OKAY(impedance ok)." : "ERROR!");
	buffer[n] = 0;

	pr_info("[SmartPA-%d]init_dbg: write to file\n", __LINE__);

	tas2562_priv->calibRe[0] = calib_re[0];
	tas2562_priv->calibRe[1] = calib_re[1];
	pr_info("[SmartPA-%d]init_dbg: update Re value\n", __LINE__);
	smartpa_calib_save(calib_re);

//deinit:
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid  = ((AFE_SA_CALIB_DEINIT)|((iter+1)<<24)|(1<<16));
		ret = afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize);
		pr_info("[SmartPA-%d]init_dbg: decalib init\n", __LINE__);
	}
//end:
	pr_info("[SmartPA-%d]init_dbg: end\n", __LINE__);

	if (done[0] && done[1])
		return 0;
	else
		return -1;
}

static int smartpa_freq_save(char *buffer, int count)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(FREQ_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("[SmartPA-%d]freq: save count=%d \n", __LINE__, count);
		vfs_write(pfile, buffer, count, &pos);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]freq: %s open failed! \n", __LINE__, FREQ_FILE);
		ret = -1;
	}

	set_fs(old_fs);

	return ret;
}

int smartpa_read_freq_dbg(char *buffer, int size)
{
	u32 length = 1;
	uint32_t calibRe[MAX_CHANNELS] = {0};
	uint32_t F0[MAX_CHANNELS] = {0}, Q[MAX_CHANNELS] = {0};
	int ret = 0, n = 0, i[MAX_CHANNELS] = {0}, j[MAX_CHANNELS] = {0};
	uint32_t data = 0;
	uint32_t paramid = 0;
	int nSize = sizeof(uint32_t);
	uint8_t iter = 0, channels = 1;

	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);

	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: tas2562_priv is NULL\n", __LINE__);
		return -1;
	}
	channels = tas2562_priv->mn_channels;

	//Load Calib
	if (smartpa_check_re()) {

		for (iter = 0; iter < channels; iter++) {
			calibRe[iter] = tas2562_priv->calibRe[iter];
		}
		smartpa_set_re(calibRe);
	}

	for (iter = 0; iter < channels; iter++) {
		data = 1;//Value is ignored
		paramid = ((AFE_SA_F0_TEST_INIT) | (length << 16) | ((iter+1) << 24));
		ret = afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize);
	}
	//wait 5s
	msleep(5000);

	for (iter = 0; iter < channels; iter++) {
		n += scnprintf(buffer+n, size-n, "Ch[%d] ", iter);
		if (calibRe[iter] == CALIB_FAILED)
			n += scnprintf(buffer+n, size-n, "Rdc = %x\n", calibRe[iter]);
		else
			n += scnprintf(buffer+n, size-n, "Rdc = %02d.%02d\n",
				TRANSF_IMPED_TO_USER_I(calibRe[iter]), TRANSF_IMPED_TO_USER_M(calibRe[iter]));
		while ((i[iter]++ < 5) && (j[iter] < 3)) {
			//read F0
			data = 0;//Reset data to 0
			paramid = (AFE_SA_GET_F0 | (length << 16) | ((iter+1) << 24));
			ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid,
					TAS_GET_PARAM, /*length **/ 4);
			F0[iter] = data;

			//read Q
			data = 0;//Reset data to 0
			paramid = (AFE_SA_GET_Q | (length << 16) | ((iter+1) << 24));
			ret = afe_smartamp_algo_ctrl((u8 *)&data, paramid,
					TAS_GET_PARAM, /*length **/ 4);
			Q[iter] = data;

			if (((F0[iter] >> 19) < tas2562_priv->fres_min[iter]) || ((F0[iter] >> 19) > tas2562_priv->fres_max[iter])
				|| (((Q[iter] * 100) >> 19) < tas2562_priv->Qt[iter]))
				j[iter] = 0;
			else
				j[iter]++;

			pr_info("[SmartPA-%d]read freq dbg channel[%d]: f0 = %d Q = %d i = %d j = %d\n",
					__LINE__, iter, F0[iter], Q[iter], i[iter], j[iter]);
			n += scnprintf(buffer+n, size-n, "f0: %d Qt = %d.%d\n",
				(F0[iter] >> 19), TRANSF_IMPED_TO_USER_I(Q[iter]), TRANSF_IMPED_TO_USER_M(Q[iter]));
			msleep(500);
		}
		n += scnprintf(buffer+n, size-n, "f0 (%d~%d)\nQ_Min: %d.%d \n",
					   tas2562_priv->fres_min[iter], tas2562_priv->fres_max[iter],
					   tas2562_priv->Qt[iter] / 100, tas2562_priv->Qt[iter] % 100);
		if (j[iter] == 3)
			n += scnprintf(buffer+n, size-n, "PASS\n");
		else
			n += scnprintf(buffer+n, size-n, "FAIL\n");
	}

	ret = smartpa_freq_save(buffer, n);
	buffer[n] = 0;

//deinit :
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid = ((AFE_SA_F0_TEST_DEINIT) | (length << 16) | ((iter+1) << 24));
		ret = afe_smartamp_algo_ctrl((uint8_t *)&data, paramid, TAS_SET_PARAM, nSize);
	}
//end:
	return 0;
}

void smartpa_read_prars_dbg(int temp[5], unsigned char addr)
{
	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);

	return ;
}

void smartpa_get_client(struct i2c_client **client, unsigned char addr)

{
	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);

	return ;
}

int smartpa_check_calib_dbg(void)
{
	uint32_t impedance[MAX_CHANNELS] = {0};
	uint8_t iter = 0, channels = 0;
	int ret = 0;

	if (!tas2562_priv)
		return 0;

	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);

	smartpa_calib_get(impedance);
	channels = tas2562_priv->mn_channels;
	for (iter = 0; iter < channels; iter++) {
		ret |= rdc_check_valid(impedance[iter], iter) << iter;
	}
	return ret;  /* spk0 bit0, spk1 bit1 */
}

static bool rdc_check_valid(uint32_t rdc, uint8_t iter)
{
	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: tas2562_priv is NULL\n", __LINE__);
		return false;
	}
	if (rdc > tas2562_priv->imped_min[iter] && rdc < tas2562_priv->imped_max[iter]) {
		return true;
	}

	pr_info("[SmartPA-%d]rdc check: rdc=%d invalid, [%d, %d] \n", __LINE__, rdc, tas2562_priv->imped_min[iter], tas2562_priv->imped_max[iter]);
	return false;
}

static int smartpa_calib_get(uint32_t *calib_value)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int found = 0;
	loff_t pos = 0;
	int channels = 1;

	if (!tas2562_priv || !calib_value) {
		pr_err("[SmartPA-%d]: tas2562_priv or calib_value is NULL\n", __LINE__);
		return false;
	}
	channels =  tas2562_priv->mn_channels;

	*calib_value = 0;
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(CALIBRATE_FILE, O_RDONLY, 0);
	if (!IS_ERR_OR_NULL(pfile)) {
		found = 1;
		vfs_read(pfile, (char *)calib_value, sizeof(uint32_t)*channels, &pos);
		pr_info("[SmartPA-%d]calibrate:get calib_value[0] %d  \n", __LINE__, calib_value[0]);
		if (channels == 2)
			pr_info("[SmartPA-%d]calibrate:get calib_value[1] %d  \n", __LINE__, calib_value[1]);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]calibrate: No found\n", __LINE__);
		found = 0;
	}

	set_fs(old_fs);

	return found;
}

static bool smartpa_check_re(void)
{
	int rc = 0;
	uint32_t impedance[MAX_CHANNELS] = {0};
	uint8_t iter = 0, channels = 0;

	pr_info("[SmartPA-%d] smartpa_check_re enter.\n", __LINE__);
	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: tas2562_priv is NULL\n", __LINE__);
		return false;
	}
	channels = tas2562_priv->mn_channels;

	for (iter = 0; iter < channels; iter++) {
		if (rdc_check_valid(tas2562_priv->calibRe[iter], iter) || (tas2562_priv->calibRe[iter] == 0xCACACACA)) {
			pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d ok.\n",
					__LINE__, iter, tas2562_priv->calibRe[iter]);
			rc += 1;
		}
	}
	if (rc == channels)
		return true;

	rc = smartpa_calib_get(impedance);
	if (rc == 0) {
		pr_info("[SmartPA-%d] smartpa_check_re get re failed.\n", __LINE__);
		return false;
	}

	rc = 0;
	for (iter = 0; iter < channels; iter++) {
		tas2562_priv->calibRe[iter] = impedance[iter];
		if (rdc_check_valid(tas2562_priv->calibRe[iter], iter) || (tas2562_priv->calibRe[iter] == 0xCACACACA)) {
			pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d success.\n", __LINE__, iter, impedance[iter]);
			rc += 1;
		} else {
			pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d failed.\n", __LINE__, iter, impedance[iter]);
		}
	}

	if (rc == channels)
		return true;

	return false;
}

static void smartpa_parse_special_reg_table(void)
{
	const char *reg_table = "vivo,reg_table";
	int reg_size = 0, *array = NULL;
	int i = 0;

	if (tas2562_priv == NULL || tas2562_priv->dev == NULL || tas2562_priv->dev->of_node == NULL) {
		pr_err("%s, tas2562_priv is invalid\n", __func__);
		return;
	}

	tas2562_priv->reg_size = 0;
	tas2562_priv->reg_table = NULL;

	if (!of_find_property(tas2562_priv->dev->of_node, reg_table, &reg_size)) {
		dev_err(tas2562_priv->dev, "%s: Missing %s in dt node.\n", __func__, reg_table);
		return;
	}

	reg_size /= sizeof(int);
	if (reg_size % 5) {
		dev_err(tas2562_priv->dev, "%s: Abnormal number of parameters.\n", __func__);
		return;
	}

	array = kzalloc(sizeof(int) * reg_size, GFP_KERNEL);
	if (!array) {
		dev_err(tas2562_priv->dev, "%s: Out of memory.\n", __func__);
		return;
	}

	if (of_property_read_u32_array(tas2562_priv->dev->of_node, reg_table, array, reg_size)) {
		kfree(array);
		array = NULL;
		dev_err(tas2562_priv->dev, "%s: Read property %s node failed.\n", __func__, reg_table);
		return;
	}

	tas2562_priv->reg_size = reg_size / 5;
	tas2562_priv->reg_table = array;
	dev_info(tas2562_priv->dev, "%s: special params, reg_size: %d.\n", __func__, tas2562_priv->reg_size);
	for (i = 0; i < tas2562_priv->reg_size; i++) {
		printk("0x%2x 0x%2x 0x%2x 0x%2x 0x%2x\n",
			tas2562_priv->reg_table[i*5+0],
			tas2562_priv->reg_table[i*5+1],
			tas2562_priv->reg_table[i*5+2],
			tas2562_priv->reg_table[i*5+3],
			tas2562_priv->reg_table[i*5+4]
			);
	}

	return;
}

static int smartpa_parse_dt(struct i2c_client *i2c)
{
	int temp, ret = 0;

	if (!tas2562_priv) {
		pr_err("[SmartPA-%d]: tas2562_priv is NULL\n", __LINE__);
		return -1;
	}

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,impedance-min", &temp);
	tas2562_priv->imped_min[0] = (!ret) ? (int)temp : RDC_MIN_L;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,impedance-max", &temp);
	tas2562_priv->imped_max[0] = (!ret) ? (int)temp : RDC_MAX_L;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,frequency-min", &temp);
	tas2562_priv->fres_min[0] = (!ret) ? (int)temp : 500;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,frequency-max", &temp);
	tas2562_priv->fres_max[0] = (!ret) ? (int)temp : 1100;

	ret = of_property_read_u32(i2c->dev.of_node, "vivo,Qt-min", &temp);
	tas2562_priv->Qt[0] = (!ret) ? (int)temp : 100;

	printk("%s, before smartpa_parse_special_reg_table\n", __func__);
	smartpa_parse_special_reg_table();
	printk("%s, after smartpa_parse_special_reg_table\n", __func__);

	if (tas2562_priv->mn_channels == 2) {
		ret = of_property_read_u32(i2c->dev.of_node, "vivo,impedance-min", &temp);
		tas2562_priv->imped_min[1] = (!ret) ? (int)temp : RDC_MIN_R;

		ret = of_property_read_u32(i2c->dev.of_node, "vivo,impedance-max", &temp);
		tas2562_priv->imped_max[1] = (!ret) ? (int)temp : RDC_MAX_R;

		ret = of_property_read_u32(i2c->dev.of_node, "vivo,frequency-min", &temp);
		tas2562_priv->fres_min[1] = (!ret) ? (int)temp : 500;

		ret = of_property_read_u32(i2c->dev.of_node, "vivo,frequency-max", &temp);
		tas2562_priv->fres_max[1] = (!ret) ? (int)temp : 1100;

		ret = of_property_read_u32(i2c->dev.of_node, "vivo,Qt-min", &temp);
		tas2562_priv->Qt[1] = (!ret) ? (int)temp : 100;
	}

	return 0;
}
#endif
static int tas2562_parse_dt(struct device *dev, struct tas2562_priv *p_tas2562)
{
	struct device_node *np = dev->of_node;
	int rc = 0, ret = 0;

	rc = of_property_read_u32(np, "ti,channels", &p_tas2562->mn_channels);
	if (rc) {
		dev_err(p_tas2562->dev, "Looking up %s property in node %s failed %d\n",
			"ti,channels", np->full_name, rc);
	} else {
		dev_dbg(p_tas2562->dev, "ti,channels=%d",
			p_tas2562->mn_channels);
	}

	rc = of_property_read_u32(np, "ti,left-channel", &p_tas2562->mn_l_addr);
	if (rc) {
		dev_err(p_tas2562->dev, "Looking up %s property in node %s failed %d\n",
			"ti,left-channel", np->full_name, rc);
	} else {
		dev_dbg(p_tas2562->dev, "ti,left-channel=0x%x",
			p_tas2562->mn_l_addr);
	}

	if (p_tas2562->mn_channels != 1) {
		rc = of_property_read_u32(np, "ti,right-channel",
			&p_tas2562->mn_r_addr);
		if (rc) {
			dev_err(p_tas2562->dev, "Looking up %s property in node %s failed %d\n",
				"ti,right-channel", np->full_name, rc);
		} else {
			dev_dbg(p_tas2562->dev, "ti,right-channel=0x%x",
				p_tas2562->mn_r_addr);
		}
	}

	p_tas2562->mn_reset_gpio = of_get_named_gpio(np, "ti,reset-gpio", 0);
	if (!gpio_is_valid(p_tas2562->mn_reset_gpio)) {
		dev_err(p_tas2562->dev, "Looking up %s property in node %s failed %d\n",
			"ti,reset-gpio", np->full_name,
				p_tas2562->mn_reset_gpio);
	} else {
		dev_dbg(p_tas2562->dev, "ti,reset-gpio=%d",
			p_tas2562->mn_reset_gpio);
	}

	if (p_tas2562->mn_channels != 1) {
		p_tas2562->mn_reset_gpio2 = of_get_named_gpio(np, "ti,reset-gpio2", 0);
		if (!gpio_is_valid(p_tas2562->mn_reset_gpio2)) {
			dev_dbg(p_tas2562->dev, "Looking up %s property in node %s failed %d\n",
				"ti,reset-gpio2", np->full_name,
				p_tas2562->mn_reset_gpio2);
		} else {
			dev_dbg(p_tas2562->dev, "ti,reset-gpio2=%d",
				p_tas2562->mn_reset_gpio2);
		}
	}

	p_tas2562->mn_irq_gpio = of_get_named_gpio(np, "ti,irq-gpio", 0);
	if (!gpio_is_valid(p_tas2562->mn_irq_gpio)) {
		dev_err(p_tas2562->dev, "Looking up %s property in node %s failed %d\n",
			"ti,irq-gpio", np->full_name, p_tas2562->mn_irq_gpio);
	} else {
		dev_dbg(p_tas2562->dev, "ti,irq-gpio=%d",
			p_tas2562->mn_irq_gpio);
	}

	if (p_tas2562->mn_channels != 1) {
		p_tas2562->mn_irq_gpio2 = of_get_named_gpio(np, "ti,irq-gpio2", 0);
		if (!gpio_is_valid(p_tas2562->mn_irq_gpio2)) {
			dev_dbg(p_tas2562->dev, "Looking up %s property in node %s failed %d\n",
			"ti,irq-gpio2", np->full_name, p_tas2562->mn_irq_gpio2);
		} else {
			dev_dbg(p_tas2562->dev, "ti,irq-gpio2=%d",
				p_tas2562->mn_irq_gpio2);
		}
	}
#ifdef CONFIG_TAS25XX_ALGO
	tas25xx_parse_algo_dt(np);
#endif /*CONFIG_TAS25XX_ALGO*/
	return ret;
}

static int tas2562_device_check(struct tas2562_priv *p_tas2562,
	unsigned int reg, int value)
{
	int ret = 0;
	int n_result = 0;

	mutex_lock(&p_tas2562->dev_lock);

	if (p_tas2562->mn_channels == 1) {
		goto end;
	}
//check left, if error
	if (p_tas2562->mn_channels == 2) {
		p_tas2562->client->addr = p_tas2562->mn_l_addr;

		n_result = tas2562_regmap_write(p_tas2562,
			TAS2562_PAGE_REG(reg), value);
		if (n_result < 0)
			dev_err(p_tas2562->dev, "%s, ERROR, L=%d, E=%d\n",
				__func__, __LINE__, n_result);
		else {
			dev_dbg(p_tas2562->dev,
			"%s: chn%x:BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
				__func__, p_tas2562->client->addr,
				TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), value);
		}
	}

	if (p_tas2562->mn_channels == 2) {
		p_tas2562->client->addr = p_tas2562->mn_r_addr;

		n_result = tas2562_regmap_write(p_tas2562,
		TAS2562_PAGE_REG(reg),
			value);
		if (n_result < 0) {
			dev_err(p_tas2562->dev, "%s, ERROR, Line=%d, device not exist or device damage.\n",
				__func__, __LINE__);
			p_tas2562->mn_channels = 1;
			p_tas2562->mn_reset_gpio2 = 0;
			p_tas2562->mn_irq_gpio2 = 0;
			//p_tas2562->mn_irq2 = 0;
		} else {
			dev_dbg(p_tas2562->dev, "%s: chn%x:BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
				__func__, p_tas2562->client->addr,
				TAS2562_BOOK_ID(reg),
				TAS2562_PAGE_ID(reg),
				TAS2562_PAGE_REG(reg), value);
		}
	}

end:
	mutex_unlock(&p_tas2562->dev_lock);
	return n_result;
}


#ifdef AT_MODE
static ssize_t smartpa_i2c_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	const int size = 512;
	int devicenum = 1;
	int n = 0;

	if (!tas2562_priv || !tas2562_priv->smartpa_i2c_check)
		return 0;

	if (tas2562_priv->mn_channels == 2) {
		if ((tas2562_priv->smartpa_i2c_check & 3) == 3) {
			n += scnprintf(ubuf+n, size-n, "SmartPA-stereo OK\n");
		} else {
			n += scnprintf(ubuf+n, size-n, "SmartPA-stereo ERROR\n");
		}
	} else if (tas2562_priv->mn_channels == 1) {
		if ((tas2562_priv->smartpa_i2c_check & 1) == 1) {
			n += scnprintf(ubuf+n, size-n, "SmartPA-mono OK\n");
		} else {
			n += scnprintf(ubuf+n, size-n, "SmartPA-mono ERROR\n");
		}
	}

	ubuf[n] = 0;
	return n;
}

static ssize_t smartpa_reg_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	const int size = 4096;
	int n = 0, channels = 1;
	int i = 0, reg_val = 0;

	if (!tas2562_priv) {
		pr_err("%s: SmartPA_priv is NULL\n", __func__);
		return 0;
	}

	channels = tas2562_priv->mn_channels;

	pr_info("%s: TAS2562 REG DUMP left channel: \n", __func__);
	n += scnprintf(ubuf+n, size-n, "reg left: \n");
	for (i = 0; i < REG_NUM; i++) {
		tas2562_dev_read(tas2562_priv, channel_left, i, &reg_val);
		n += scnprintf(ubuf+n, size-n, "0x%02x:0x%02x \n", i, reg_val);
		if (n >= size - 20)
			goto end;
	}

	n += scnprintf(ubuf+n, size-n, "extra reg left: \n");
	for (i = 0; i < ARRAY_SIZE(tas256x_extra_regs); i++) {
		tas2562_dev_read(tas2562_priv, channel_left, tas256x_extra_regs[i], &reg_val);
		n += scnprintf(ubuf+n, size-n, "%02x:%02x \n", tas256x_extra_regs[i], reg_val);
		if (n >= size - 20)
			goto end;
	}

	if (channels > 1) {
		pr_info("%s: TAS2562 REG DUMP right channel: \n", __func__);
		n += scnprintf(ubuf+n, size-n, "reg right: \n");
		for (i = 0; i < REG_NUM; i++) {
			tas2562_dev_read(tas2562_priv, channel_right, i, &reg_val);
			n += scnprintf(ubuf+n, size-n, "%02x:%02x \n", i, reg_val);
			if (n >= size - 20)
				goto end;
		}

		n += scnprintf(ubuf+n, size-n, "extra reg right: \n");
		for (i = 0; i < ARRAY_SIZE(tas256x_extra_regs); i++) {
			tas2562_dev_read(tas2562_priv, channel_right, tas256x_extra_regs[i], &reg_val);
			n += scnprintf(ubuf+n, size-n, "%02x:%02x \n", tas256x_extra_regs[i], reg_val);
			if (n >= size - 20)
				goto end;
		}
	}

end:
	ubuf[n] = 0;
	return n;
}

//just for mono case
static ssize_t smartpa_reg_store(struct kobject *kobj, struct kobj_attribute *attr, const char *ubuf, size_t count)
{
	const int size = 30;
	char *temp;
	unsigned int kbuf[4];
	int ret = 0;
	int i = 0;

	if (IS_ERR_OR_NULL(tas2562_priv)) {
		pr_err("%s: SmartPA_priv is NULL\n", __func__);
		return 0;
	}

	if (count > size) {
		pr_err("%s: argv is too long (%u)\n", __func__, count);
		return 0;
	}

	temp = kmalloc(count, GFP_KERNEL);
	if (IS_ERR_OR_NULL(temp)) {
		pr_err("%s: alloc memory failed\n", __func__);
		return 0;
	}

	memset((void *)kbuf, 0, sizeof(kbuf));
	memcpy(temp, ubuf, count);
	ret = sscanf(temp, "%x %x %x %x", &kbuf[0], &kbuf[1], &kbuf[2], &kbuf[3]);
	if (!ret) {
		pr_err("%s: invalid argv (%s)\n", __func__, ubuf);
		goto end;
	}


	pr_info("%s: TAS2562 REG write left channel: \n", __func__);
	pr_info("%s: ===> book:0x%02x, page:0x%02, reg:0x%02x, val:0x%02x\n", __func__, kbuf[0], kbuf[1], kbuf[2], kbuf[3]);
	pr_info("%s: ===> reg addr:0x%02x, val:0x%02x\n", __func__, TAS2562_REG(kbuf[0], kbuf[1], kbuf[2]), kbuf[3]);

	ret = tas2562_dev_write(tas2562_priv, channel_left, TAS2562_REG(kbuf[0], kbuf[1], kbuf[2]), kbuf[3]);
	if (ret < 0) {
		pr_err("%s: reg write failed (ret = %d)\n", __func__, ret);
	} else {
		pr_info("%s: reg write successful\n", __func__);
	}

end:
	kfree((void *)temp);
	return count;
}

static ssize_t smartpa_quick_reg_store(struct kobject *kobj, struct kobj_attribute *attr, const char *ubuf, size_t count)
{
	const int size = 20;
	char *temp;
	unsigned int kbuf[2];
	int ret = 0;
	int i = 0;

	if (IS_ERR_OR_NULL(tas2562_priv)) {
		pr_err("%s: SmartPA_priv is NULL\n", __func__);
		return 0;
	}

	if (count > size) {
		pr_err("%s: argv is too long (%u)\n", __func__, count);
		return 0;
	}

	temp = kmalloc(count, GFP_KERNEL);
	if (IS_ERR_OR_NULL(temp)) {
		pr_err("%s: alloc memory failed\n", __func__);
		return 0;
	}

	memset((void *)kbuf, 0, sizeof(kbuf));
	memcpy(temp, ubuf, count);
	ret = sscanf(temp, "%x %x", &kbuf[0], &kbuf[1]);
	if (!ret) {
		pr_err("%s: invalid argv (%s)\n", __func__, ubuf);
		goto end;
	}

	pr_info("%s: TAS2562 REG write left channel: \n", __func__);
	pr_info("%s: ===> reg addr:0x%02x, val:0x%02x\n", __func__, kbuf[0], kbuf[1]);
	pr_info("%s: ===> book:0x%02x, page:0x%02, reg:0x%02x, val:0x%02x\n", __func__,
		TAS2562_BOOK_ID(kbuf[0]), TAS2562_PAGE_ID(kbuf[0]), TAS2562_PAGE_REG(kbuf[0]), kbuf[1]);

	ret = tas2562_dev_write(tas2562_priv, channel_left, kbuf[0], kbuf[1]);
	if (ret < 0) {
		pr_err("%s: reg write failed (ret = %d)\n", __func__, ret);
	} else {
		pr_info("%s: reg write successful\n", __func__);
	} 

end:
	kfree((void *)temp);
	return count;
}

static struct kobj_attribute dev_attr_i2c =
	__ATTR(i2c, 0664, smartpa_i2c_show, NULL);

static struct kobj_attribute dev_attr_reg =
	__ATTR(reg, 0664, smartpa_reg_show, smartpa_reg_store);

static struct kobj_attribute dev_attr_quick_reg =
	__ATTR(quick_reg, 0664, smartpa_reg_show, smartpa_quick_reg_store);


static struct attribute *sys_node_attributes[] = {
	&dev_attr_quick_reg.attr,
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
	char name[64] = "audio-tas2562";

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

#endif


static int tas2562_i2c_probe(struct i2c_client *p_client,
			const struct i2c_device_id *id)
{
	struct tas2562_priv *p_tas2562;
	int n_result;
	dev_err(&p_client->dev, "Driver ID: %s\n", TAS2562_DRIVER_ID);
	dev_info(&p_client->dev, "%s enter\n", __func__);
	printk("legen-%s:enter.\n", __func__);
	p_tas2562 = devm_kzalloc(&p_client->dev,
		sizeof(struct tas2562_priv), GFP_KERNEL);
	if (p_tas2562 == NULL) {
		/* dev_err(&p_client->dev, "failed to get i2c device\n"); */
		n_result = -ENOMEM;
		goto err;
	}

	p_tas2562->client = p_client;
	p_tas2562->dev = &p_client->dev;
	i2c_set_clientdata(p_client, p_tas2562);
	dev_set_drvdata(&p_client->dev, p_tas2562);

	/*qinbaoqiang add */
	dev_set_name(&p_client->dev, "%s", "tas2562");


	p_tas2562->regmap = devm_regmap_init_i2c(p_client, &tas2562_i2c_regmap);
	if (IS_ERR(p_tas2562->regmap)) {
		n_result = PTR_ERR(p_tas2562->regmap);
		dev_err(&p_client->dev, "Failed to allocate register map: %d\n",
					n_result);
		goto err;
	}

	if (p_client->dev.of_node)
		tas2562_parse_dt(&p_client->dev, p_tas2562);

	if (gpio_is_valid(p_tas2562->mn_reset_gpio)) {
		n_result = gpio_request(p_tas2562->mn_reset_gpio,
			"TAS2562_RESET");
		if (n_result) {
			dev_err(p_tas2562->dev, "%s: Failed to request gpio %d\n",
				__func__, p_tas2562->mn_reset_gpio);
			n_result = -EINVAL;
			goto err;
		}
		tas2562_hw_reset(p_tas2562);
	}

	if (gpio_is_valid(p_tas2562->mn_reset_gpio2) &&
			(p_tas2562->mn_channels == 2)) {
		n_result = gpio_request(p_tas2562->mn_reset_gpio2,
			"TAS2562_RESET2");
		if (n_result) {
			dev_err(p_tas2562->dev, "%s: Failed to request gpio %d\n",
				__func__, p_tas2562->mn_reset_gpio2);
			n_result = -EINVAL;
			goto err;
		}
		tas2562_hw_reset(p_tas2562);
	}

	p_tas2562->read = tas2562_dev_read;
	p_tas2562->write = tas2562_dev_write;
	p_tas2562->bulk_read = tas2562_dev_bulk_read;
	p_tas2562->bulk_write = tas2562_dev_bulk_write;
	p_tas2562->update_bits = tas2562_dev_update_bits;
	p_tas2562->hw_reset = tas2562_hw_reset;
	p_tas2562->enable_irq = tas2562_enable_irq;
#ifdef CODEC_PM
	p_tas2562->runtime_suspend = tas2562_runtime_suspend;
	p_tas2562->runtime_resume = tas2562_runtime_resume;
	p_tas2562->mn_power_state = TAS2562_POWER_SHUTDOWN;
#endif
	p_tas2562->mn_power_state = TAS2562_POWER_SHUTDOWN;
	p_tas2562->spk_l_control = 1;
	if (p_tas2562->mn_channels == 2)
		p_tas2562->spk_r_control = 1;
#ifdef VIVO_PORT_SMARTPA
	memset(p_tas2562->calibRe, 0, sizeof(uint32_t)*p_tas2562->mn_channels);
	p_tas2562->set_re = smartpa_set_re;
	p_tas2562->check_re = smartpa_check_re;
#endif

	mutex_init(&p_tas2562->dev_lock);

	tas2562_device_check(p_tas2562, TAS2562_SOFTWARERESET, 0x01);
	dev_info(&p_client->dev, "Before SW reset\n");
	/* Reset the chip */
	n_result = tas2562_dev_write(p_tas2562, channel_left, TAS2562_SOFTWARERESET, 0x01);
	if (n_result < 0)
		dev_err(&p_client->dev, "channel_left I2c fail, %d", n_result);
	else
		p_tas2562->smartpa_i2c_check |= 1;
	if (p_tas2562->mn_channels == 2) {
		n_result = tas2562_dev_write(p_tas2562, channel_right, TAS2562_SOFTWARERESET, 0x01);
		if (n_result < 0)
			dev_err(&p_client->dev, "channel_right I2c fail, %d", n_result);
		else
			p_tas2562->smartpa_i2c_check |= 2;
	}
	if (((p_tas2562->mn_channels == 1) && (p_tas2562->smartpa_i2c_check != 1)) ||
		((p_tas2562->mn_channels == 2) && (p_tas2562->smartpa_i2c_check != 3))) {
		dev_err(&p_client->dev, "I2c fail, smartpa_i2c_check %d\n",
			p_tas2562->smartpa_i2c_check);
		goto err;
	}
	dev_info(&p_client->dev, "After SW reset\n");

	if (gpio_is_valid(p_tas2562->mn_irq_gpio)) {
		n_result = gpio_request(p_tas2562->mn_irq_gpio, "TAS2562-IRQ");
		if (n_result < 0) {
			dev_err(p_tas2562->dev, "%s: GPIO %d request error\n",
				__func__, p_tas2562->mn_irq_gpio);
			goto err;
		}
		gpio_direction_input(p_tas2562->mn_irq_gpio);
		tas2562_dev_write(p_tas2562, channel_both,
			TAS2562_MISCCONFIGURATIONREG0, 0xce);

		p_tas2562->mn_irq = gpio_to_irq(p_tas2562->mn_irq_gpio);
		dev_info(p_tas2562->dev, "irq = %d\n", p_tas2562->mn_irq);
		INIT_DELAYED_WORK(&p_tas2562->irq_work, irq_work_routine);
		n_result = request_threaded_irq(p_tas2562->mn_irq,
				tas2562_irq_handler,
				NULL, IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
				p_client->name, p_tas2562);
		if (n_result < 0) {
			dev_err(p_tas2562->dev,
				"request_irq failed, %d\n", n_result);
			goto err;
		}
		disable_irq_nosync(p_tas2562->mn_irq);
	}
	if (gpio_is_valid(p_tas2562->mn_irq_gpio2) &&
			(p_tas2562->mn_channels == 2)) {
		n_result = gpio_request(p_tas2562->mn_irq_gpio2,
				"TAS2562-IRQ2");
		if (n_result < 0) {
			dev_err(p_tas2562->dev, "%s: GPIO %d request error\n",
				__func__, p_tas2562->mn_irq_gpio2);
			goto err;
		}
		gpio_direction_input(p_tas2562->mn_irq_gpio2);
		tas2562_dev_write(p_tas2562, channel_both,
				TAS2562_MISCCONFIGURATIONREG0, 0xce);

		p_tas2562->mn_irq2 = gpio_to_irq(p_tas2562->mn_irq_gpio2);
		dev_info(p_tas2562->dev, "irq = %d\n", p_tas2562->mn_irq2);
		INIT_DELAYED_WORK(&p_tas2562->irq_work, irq_work_routine);
		n_result = request_threaded_irq(p_tas2562->mn_irq2,
				tas2562_irq_handler,
				NULL, IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
				p_client->name, p_tas2562);
		if (n_result < 0) {
			dev_err(p_tas2562->dev,
				"request_irq failed, %d\n", n_result);
			goto err;
		}
		disable_irq_nosync(p_tas2562->mn_irq2);
	}
	tas2562_enable_irq(p_tas2562, true);
	INIT_DELAYED_WORK(&p_tas2562->init_work, init_work_routine);

#ifdef CONFIG_TAS2562_CODEC
	mutex_init(&p_tas2562->codec_lock);
	n_result = tas2562_register_codec(p_tas2562);
	if (n_result < 0) {
		dev_err(p_tas2562->dev,
			"register codec failed, %d\n", n_result);
		goto err;
	}
#endif

#ifdef CONFIG_TAS2562_MISC
	mutex_init(&p_tas2562->file_lock);
	n_result = tas2562_register_misc(p_tas2562);
	if (n_result < 0) {
		dev_err(p_tas2562->dev,
			"register codec failed, %d\n", n_result);
		goto err;
	}
#endif

err:

	printk("%s, before smartpa_parse_dt\n", __func__);
#ifdef VIVO_PORT_SMARTPA
	tas_calib_init();
	tas2562_priv = p_tas2562;
	n_result = smartpa_debug_probe(p_client);
	if (n_result != 0)
		pr_err("[SmartPA-%d] Failed to probe debug interface: %d\n", __LINE__, n_result);
	n_result = smartpa_parse_dt(p_client);
#ifdef CONFIG_DEBUG_FS
	smartpa_debug_init(p_tas2562, p_client);
#endif
#endif

#ifdef AT_MODE
	class_attr_create(p_tas2562->kobj);
#endif
	printk("%s, after smartpa_parse_dt\n", __func__);
	return n_result;
}

static int tas2562_i2c_remove(struct i2c_client *p_client)
{
	struct tas2562_priv *p_tas2562 = i2c_get_clientdata(p_client);

	dev_info(p_tas2562->dev, "%s\n", __func__);

#ifdef AT_MODE
	if (p_tas2562 && p_tas2562->kobj)
		class_attr_remove(p_tas2562->kobj);
#endif

#ifdef CONFIG_TAS2562_CODEC
	tas2562_deregister_codec(p_tas2562);
	mutex_destroy(&p_tas2562->codec_lock);
#endif

#ifdef CONFIG_TAS2562_MISC
	tas2562_deregister_misc(p_tas2562);
	mutex_destroy(&p_tas2562->file_lock);
#endif

	if (p_tas2562->reg_table) {
		kfree((void *)p_tas2562->reg_table);
		p_tas2562->reg_table = NULL;
		p_tas2562->reg_size = 0;
	}

	tas_calib_exit();

	if (gpio_is_valid(p_tas2562->mn_reset_gpio))
		gpio_free(p_tas2562->mn_reset_gpio);
	if (gpio_is_valid(p_tas2562->mn_irq_gpio))
		gpio_free(p_tas2562->mn_irq_gpio);
	if (gpio_is_valid(p_tas2562->mn_reset_gpio2))
		gpio_free(p_tas2562->mn_reset_gpio2);
	if (gpio_is_valid(p_tas2562->mn_irq_gpio2))
		gpio_free(p_tas2562->mn_irq_gpio2);

	return 0;
}


static const struct i2c_device_id tas2562_i2c_id[] = {
	{ "tas2562", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas2562_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas2562_of_match[] = {
	{ .compatible = "ti,tas2562" },
	{},
};
MODULE_DEVICE_TABLE(of, tas2562_of_match);
#endif

static const struct dev_pm_ops tas2562_pm_ops = {
	.suspend = tas2562_pm_suspend,
	.resume = tas2562_pm_resume
};

static struct i2c_driver tas2562_i2c_driver = {
	.driver = {
		.name   = "tas2562",
		.owner  = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(tas2562_of_match),
#endif
		.pm = &tas2562_pm_ops,
	},
	.probe      = tas2562_i2c_probe,
	.remove     = tas2562_i2c_remove,
	.id_table   = tas2562_i2c_id,
};

module_i2c_driver(tas2562_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2562 I2C Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
#endif
