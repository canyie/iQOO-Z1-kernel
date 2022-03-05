#include "tas256x-logic.h"
#include "../tas256x/inc/tas256x-device.h"
#include "../tas256x/inc/tas256x.h"


#ifdef CONFIG_TAS25XX_ALGO
#ifdef CONFIG_PLATFORM_EXYNOS
#include <sound/smart_amp.h>
#else
#ifdef CONFIG_TAS256x_FOR_MTK
#include "../dsp/tas_smart_amp_v2.h"
#else
#include <sound/tas_smart_amp_v2.h>
#endif /* CONFIG_TAS256x_FOR_MTK */
#endif /*CONFIG_PLATFORM_EXYNOS*/
#include "../algo/inc/tas25xx-calib.h"
#endif /*CONFIG_TAS25XX_ALGO*/

static int tas256x_change_book_page(struct tas256x_priv *p_tas256x,
	enum channel chn,
	int book, int page)
{
	int n_result = 0, rc = 0;
	int i = 0;

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (((chn&channel_left) && (i == 0))
			|| ((chn&channel_right) && (i == 1))) {
			if (p_tas256x->devs[i]->mn_current_book != book) {
				n_result = p_tas256x->plat_write(p_tas256x->platform_data,
					p_tas256x->devs[i]->mn_addr, TAS256X_BOOKCTL_PAGE, 0);
				if (n_result < 0) {
					pr_err(
						"%s, ERROR, L=%d, E=%d\n",
						__func__, __LINE__, n_result);
					rc |= n_result;
					continue;
				}
				p_tas256x->devs[i]->mn_current_page = 0;
				n_result = p_tas256x->plat_write(p_tas256x->platform_data,
					p_tas256x->devs[i]->mn_addr, TAS256X_BOOKCTL_REG, book);
				if (n_result < 0) {
					pr_err(
						"%s, ERROR, L=%d, E=%d\n",
						__func__, __LINE__, n_result);
					rc |= n_result;
					continue;
				}
				p_tas256x->devs[i]->mn_current_book = book;
			}

			if (p_tas256x->devs[i]->mn_current_page != page) {
				n_result = p_tas256x->plat_write(p_tas256x->platform_data,
					p_tas256x->devs[i]->mn_addr, TAS256X_BOOKCTL_PAGE, page);
				if (n_result < 0) {
					pr_err(
						"%s, ERROR, L=%d, E=%d\n",
						__func__, __LINE__, n_result);
					rc |= n_result;
					continue;
				}
				p_tas256x->devs[i]->mn_current_page = page;
			}
		}
	}

	if (rc < 0) {
		if (chn&channel_left)
			p_tas256x->mn_err_code |= ERROR_DEVA_I2C_COMM;
		if (chn&channel_right)
			p_tas256x->mn_err_code |= ERROR_DEVB_I2C_COMM;
	} else {
		if (chn&channel_left)
			p_tas256x->mn_err_code &= ~ERROR_DEVA_I2C_COMM;
		if (chn&channel_right)
			p_tas256x->mn_err_code &= ~ERROR_DEVB_I2C_COMM;
	}
	return rc;
}

static int tas256x_dev_read(struct tas256x_priv *p_tas256x,
	enum channel chn,
	unsigned int reg, unsigned int *pValue)
{
	int n_result = 0;
	int i = 0, chnTemp = 0;

	mutex_lock(&p_tas256x->dev_lock);

	if (chn == channel_both) {
		for (i = 0; i < p_tas256x->mn_channels; i++) {
			if (p_tas256x->devs[i]->spk_control == 1)
				chnTemp |= 1<<i;
		}
		chn = (chnTemp == 0)?chn:(enum channel)chnTemp;
	}

	n_result = tas256x_change_book_page(p_tas256x, chn,
		TAS256X_BOOK_ID(reg), TAS256X_PAGE_ID(reg));
	if (n_result < 0)
		goto end;

	/*Force left incase of mono or if both channels*/
	if ((p_tas256x->mn_channels == 1) || (chn == channel_both))
		chn = channel_left;

	n_result = p_tas256x->plat_read(p_tas256x->platform_data,
		p_tas256x->devs[chn>>1]->mn_addr, TAS256X_PAGE_REG(reg), pValue);
	if (n_result < 0) {
		pr_err("%s, ERROR, L=%d, E=%d\n",
			__func__, __LINE__, n_result);
		if (chn&channel_left)
			p_tas256x->mn_err_code |= ERROR_DEVA_I2C_COMM;
		if (chn&channel_right)
			p_tas256x->mn_err_code |= ERROR_DEVB_I2C_COMM;
	} else {
		pr_err(
			"%s: chn:%x:BOOK:PAGE:REG 0x%02x:0x%02x:0x%02x,0x%02x\n",
			__func__,
			p_tas256x->devs[chn>>1]->mn_addr, TAS256X_BOOK_ID(reg),
			TAS256X_PAGE_ID(reg),
			TAS256X_PAGE_REG(reg), *pValue);
		if (chn&channel_left)
			p_tas256x->mn_err_code &= ~ERROR_DEVA_I2C_COMM;
		if (chn&channel_right)
			p_tas256x->mn_err_code &= ~ERROR_DEVB_I2C_COMM;
	}
end:
	mutex_unlock(&p_tas256x->dev_lock);
	return n_result;
}

static int tas256x_dev_write(struct tas256x_priv *p_tas256x, enum channel chn,
	unsigned int reg, unsigned int value)
{
	int n_result = 0, rc = 0;
	int i = 0, chnTemp = 0;

	mutex_lock(&p_tas256x->dev_lock);

	if (chn == channel_both) {
		for (i = 0; i < p_tas256x->mn_channels; i++) {
			if (p_tas256x->devs[i]->spk_control == 1)
				chnTemp |= 1<<i;
		}
		chn = (chnTemp == 0)?chn:(enum channel)chnTemp;
	}

	n_result = tas256x_change_book_page(p_tas256x, chn,
		TAS256X_BOOK_ID(reg), TAS256X_PAGE_ID(reg));
	if (n_result < 0)
		goto end;

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (((chn&channel_left) && (i == 0))
			|| ((chn&channel_right) && (i == 1))) {
			n_result = p_tas256x->plat_write(p_tas256x->platform_data,
				p_tas256x->devs[i]->mn_addr, TAS256X_PAGE_REG(reg), value);
			if (n_result < 0) {
				pr_err(
					"%s, ERROR, L=%u, chn=0x%02x, E=%d\n",
					__func__, __LINE__,
					p_tas256x->devs[i]->mn_addr, n_result);
				rc |= n_result;
				if (chn&channel_left)
					p_tas256x->mn_err_code |= ERROR_DEVA_I2C_COMM;
				if (chn&channel_right)
					p_tas256x->mn_err_code |= ERROR_DEVB_I2C_COMM;
			} else {
				pr_err(
					"%s: %u: chn:0x%02x:BOOK:PAGE:REG 0x%02x:0x%02x:0x%02x, VAL: 0x%02x\n",
					__func__, __LINE__,
					p_tas256x->devs[i]->mn_addr,
					TAS256X_BOOK_ID(reg),
					TAS256X_PAGE_ID(reg),
					TAS256X_PAGE_REG(reg), value);
				if (chn&channel_left)
					p_tas256x->mn_err_code &= ~ERROR_DEVA_I2C_COMM;
				if (chn&channel_right)
					p_tas256x->mn_err_code &= ~ERROR_DEVB_I2C_COMM;
			}
		}
	}
end:
	mutex_unlock(&p_tas256x->dev_lock);
	return rc;
}

static int tas256x_dev_bulk_write(struct tas256x_priv *p_tas256x,
	enum channel chn,
	unsigned int reg, unsigned char *p_data, unsigned int n_length)
{
	int n_result = 0, rc = 0;
	int i = 0, chnTemp = 0;

	mutex_lock(&p_tas256x->dev_lock);

	if (chn == channel_both) {
		for (i = 0; i < p_tas256x->mn_channels; i++) {
			if (p_tas256x->devs[i]->spk_control == 1)
				chnTemp |= 1<<i;
		}
		chn = (chnTemp == 0)?chn:(enum channel)chnTemp;
	}

	n_result = tas256x_change_book_page(p_tas256x, chn,
		TAS256X_BOOK_ID(reg), TAS256X_PAGE_ID(reg));
	if (n_result < 0)
		goto end;

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (((chn&channel_left) && (i == 0))
			|| ((chn&channel_right) && (i == 1))) {
			n_result = p_tas256x->plat_bulk_write(p_tas256x->platform_data,
				p_tas256x->devs[i]->mn_addr, TAS256X_PAGE_REG(reg), p_data, n_length);
			if (n_result < 0) {
				pr_err(
					"%s, ERROR, L=%u, chn=0x%02x: E=%d\n",
					__func__, __LINE__,
					p_tas256x->devs[i]->mn_addr, n_result);
				rc |= n_result;
				if (chn&channel_left)
					p_tas256x->mn_err_code |= ERROR_DEVA_I2C_COMM;
				if (chn&channel_right)
					p_tas256x->mn_err_code |= ERROR_DEVB_I2C_COMM;
			} else {
				pr_err(
					"%s: chn%x:BOOK:PAGE:REG 0x%02x:0x%02x:0x%02x, len: %u\n",
					__func__, p_tas256x->devs[i]->mn_addr,
					TAS256X_BOOK_ID(reg), TAS256X_PAGE_ID(reg),
					TAS256X_PAGE_REG(reg), n_length);
				if (chn&channel_left)
					p_tas256x->mn_err_code &= ~ERROR_DEVA_I2C_COMM;
				if (chn&channel_right)
					p_tas256x->mn_err_code &= ~ERROR_DEVB_I2C_COMM;
			}
		}
	}

end:
	mutex_unlock(&p_tas256x->dev_lock);
	return rc;
}

static int tas256x_dev_bulk_read(struct tas256x_priv *p_tas256x,
	enum channel chn,
	unsigned int reg, unsigned char *p_data, unsigned int n_length)
{
	int n_result = 0;
	int i = 0, chnTemp = 0;

	mutex_lock(&p_tas256x->dev_lock);

	if (chn == channel_both) {
		for (i = 0; i < p_tas256x->mn_channels; i++) {
			if (p_tas256x->devs[i]->spk_control == 1)
				chnTemp |= 1<<i;
		}
		chn = (chnTemp == 0)?chn:(enum channel)chnTemp;
	}

	n_result = tas256x_change_book_page(p_tas256x, chn,
		TAS256X_BOOK_ID(reg), TAS256X_PAGE_ID(reg));
	if (n_result < 0)
		goto end;

	n_result = p_tas256x->plat_bulk_read(p_tas256x->platform_data,
		p_tas256x->devs[chn>>1]->mn_addr, TAS256X_PAGE_REG(reg),
		p_data, n_length);
	if (n_result < 0) {
		pr_err("%s, ERROR, L=%d, E=%d\n",
			__func__, __LINE__, n_result);
		if (chn&channel_left)
			p_tas256x->mn_err_code |= ERROR_DEVA_I2C_COMM;
		if (chn&channel_right)
			p_tas256x->mn_err_code |= ERROR_DEVB_I2C_COMM;
	} else {
		pr_err(
			"%s: chn%x:BOOK:PAGE:REG %u:%u:%u, len: 0x%02x\n",
			__func__, p_tas256x->devs[chn>>1]->mn_addr,
			TAS256X_BOOK_ID(reg), TAS256X_PAGE_ID(reg),
			TAS256X_PAGE_REG(reg), n_length);
		if (chn&channel_left)
			p_tas256x->mn_err_code &= ~ERROR_DEVA_I2C_COMM;
		if (chn&channel_right)
			p_tas256x->mn_err_code &= ~ERROR_DEVB_I2C_COMM;
	}
end:
	mutex_unlock(&p_tas256x->dev_lock);
	return n_result;
}

static int tas256x_dev_update_bits(struct tas256x_priv *p_tas256x,
	enum channel chn,
	unsigned int reg, unsigned int mask, unsigned int value)
{
	int n_result = 0, rc = 0;
	int i = 0, chnTemp = 0;

	mutex_lock(&p_tas256x->dev_lock);

	if (chn == channel_both) {
		for (i = 0; i < p_tas256x->mn_channels; i++) {
			if (p_tas256x->devs[i]->spk_control == 1)
				chnTemp |= 1<<i;
		}
		chn = (chnTemp == 0)?chn:(enum channel)chnTemp;
	}

	n_result = tas256x_change_book_page(p_tas256x, chn,
		TAS256X_BOOK_ID(reg), TAS256X_PAGE_ID(reg));
	if (n_result < 0) {
		rc = n_result;
		goto end;
	}

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (((chn&channel_left) && (i == 0))
			|| ((chn&channel_right) && (i == 1))) {
			n_result = p_tas256x->plat_update_bits(p_tas256x->platform_data,
				p_tas256x->devs[i]->mn_addr, TAS256X_PAGE_REG(reg), mask, value);
			if (n_result < 0) {
				pr_err(
					"%s, ERROR, L=%u, chn=0x%02x: E=%d\n",
					__func__, __LINE__,
					p_tas256x->devs[i]->mn_addr, n_result);
				rc |= n_result;
				p_tas256x->mn_err_code |=
					(chn == channel_left) ? ERROR_DEVA_I2C_COMM : ERROR_DEVB_I2C_COMM;
			} else {
				pr_err(
					"%s: chn%x:BOOK:PAGE:REG 0x%02x:0x%02x:0x%02x, mask: 0x%02x, val: 0x%02x\n",
					__func__, p_tas256x->devs[i]->mn_addr,
					TAS256X_BOOK_ID(reg),
					TAS256X_PAGE_ID(reg),
					TAS256X_PAGE_REG(reg), mask, value);
				p_tas256x->mn_err_code &= 
					(chn == channel_left) ? ~ERROR_DEVA_I2C_COMM : ~ERROR_DEVB_I2C_COMM;
			}
		}
	}

end:
	mutex_unlock(&p_tas256x->dev_lock);
	return rc;
}

static void tas256x_hard_reset(struct tas256x_priv  *p_tas256x)
{
	int i = 0;

	p_tas256x->hw_reset(p_tas256x);

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		p_tas256x->devs[i]->mn_current_book = -1;
		p_tas256x->devs[i]->mn_current_page = -1;
	}

	if (p_tas256x->mn_err_code)
		pr_err("%s: before reset, ErrCode=0x%x\n", __func__, p_tas256x->mn_err_code);
	p_tas256x->mn_err_code = 0;
}

void tas256x_failsafe(struct tas256x_priv  *p_tas256x)
{
	int n_result;

	pr_err("tas256x %s\n", __func__);
	p_tas256x->mn_err_code |= ERROR_FAILSAFE;

	if (p_tas256x->mn_restart < RESTART_MAX) {
		p_tas256x->mn_restart++;
		msleep(100);
		pr_err("I2C COMM error, restart SmartAmp.\n");
		schedule_delayed_work(&p_tas256x->irq_work, msecs_to_jiffies(100));
		return;
	}

	n_result = tas256x_set_power_shutdown(p_tas256x, channel_both);
	p_tas256x->mb_power_up = false;
	p_tas256x->mn_power_state = TAS256X_POWER_SHUTDOWN;
	msleep(20);
	/*Mask interrupt for TDM*/
	n_result = tas256x_interrupt_enable(p_tas256x, 0/*Disable*/,
		channel_both);
	p_tas256x->enable_irq(p_tas256x, false);
	tas256x_hard_reset(p_tas256x);
	p_tas256x->write(p_tas256x, channel_both, TAS256X_SOFTWARERESET, TAS256X_SOFTWARERESET_SOFTWARERESET_RESET);
	udelay(1000);
	/*pTAS256x->write(pTAS256x, channel_both, TAS256X_SPK_CTRL_REG, 0x04);*/
}

int tas256x_load_init(struct tas256x_priv *p_tas256x)
{
	int ret = 0, i;

	pr_err("%s:\n", __func__);

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (p_tas256x->devs[i]->dev_ops.tas_init)
			ret |= (p_tas256x->devs[i]->dev_ops.tas_init)(p_tas256x, i+1);
	}

	ret |= tas256x_set_misc_config(p_tas256x, 0/*Ignored*/, channel_both);
	if (ret < 0)
		goto end;
	ret |= tas256x_set_tx_config(p_tas256x, 0/*Ignored*/, channel_both);
	if (ret < 0)
		goto end;
	ret |= tas256x_set_clock_config(p_tas256x, 0/*Ignored*/, channel_both);
	if (ret < 0)
		goto end;

	/*ICN Improve Performance*/
	ret |= tas256x_icn_config(p_tas256x, 0/*Ignored*/, channel_both);
	if (ret < 0)
		goto end;
	/*Disable the HPF in Forward Path*/
	#ifndef CONFIG_TAS256x_FOR_MTK
	ret |= tas256x_HPF_FF_Bypass(p_tas256x, 0/*Ignored*/, channel_both);
	if (ret < 0)
		goto end;

	/*Disable the HPF in Reverse Path*/
	ret |= tas256x_HPF_FB_Bypass(p_tas256x, 0/*Ignored*/, channel_both);
	if (ret < 0)
		goto end;
	#endif
	ret |= tas256x_set_classH_config(p_tas256x, 0/*Ignored*/, channel_both);

end:
	if (ret < 0) {
		if (p_tas256x->mn_err_code &
			(ERROR_DEVA_I2C_COMM | ERROR_DEVB_I2C_COMM))
			tas256x_failsafe(p_tas256x);
	}
	return ret;
}

void tas256x_load_config(struct tas256x_priv *p_tas256x)
{
	int ret = 0;

	pr_err("%s:\n", __func__);
	tas256x_hard_reset(p_tas256x);
	msleep(20);

	ret |= tas56x_software_reset(p_tas256x, channel_both);
	if (ret < 0)
		goto end;
	ret |= tas256x_load_init(p_tas256x);
	if (ret < 0)
		goto end;
	ret |= tas256x_iv_sense_enable_set(p_tas256x, 1,
		channel_both);
	if (ret < 0)
		goto end;
	ret |= tas256x_set_bitwidth(p_tas256x,
		p_tas256x->mn_rx_width, TAS256X_STREAM_PLAYBACK);
	if (ret < 0)
		goto end;
	ret |= tas256x_set_bitwidth(p_tas256x,
		p_tas256x->mn_rx_width, TAS256X_STREAM_CAPTURE);
	if (ret < 0)
		goto end;
	ret = tas256x_rx_set_edge(p_tas256x, p_tas256x->mn_rx_edge,
		channel_both);
	if (ret < 0)
		goto end;
	ret = tas256x_rx_set_start_slot(p_tas256x,
		p_tas256x->mn_rx_start_slot, channel_both);
	if (ret < 0)
		goto end;
	ret |= tas256x_set_samplerate(p_tas256x, p_tas256x->mn_sampling_rate,
		channel_both);
	if (ret < 0)
		goto end;
	ret |= tas256x_set_power_state(p_tas256x, p_tas256x->mn_power_state);
	if (ret < 0)
		goto end;
end:
/* power up failed, restart later */
	if (ret < 0) {
		if (p_tas256x->mn_err_code &
			(ERROR_DEVA_I2C_COMM | ERROR_DEVB_I2C_COMM))
			tas256x_failsafe(p_tas256x);
	}
}

void tas256x_reload(struct tas256x_priv *p_tas256x, int chn)
{
	int ret = 0;
	/*To be used later*/
	(void)chn;

	pr_err("%s: chn %d\n", __func__, chn);
	p_tas256x->enable_irq(p_tas256x, false);

	ret |= tas56x_software_reset(p_tas256x, channel_both);
	if (ret < 0)
		goto end;
	ret |= tas256x_load_init(p_tas256x);
	if (ret < 0)
		goto end;
	ret |= tas256x_iv_sense_enable_set(p_tas256x, 1,
		channel_both);
	if (ret < 0)
		goto end;
	ret |= tas256x_set_bitwidth(p_tas256x,
		p_tas256x->mn_rx_width, TAS256X_STREAM_PLAYBACK);
	if (ret < 0)
		goto end;
	ret |= tas256x_set_bitwidth(p_tas256x,
		p_tas256x->mn_rx_width, TAS256X_STREAM_CAPTURE);
	if (ret < 0)
		goto end;
	ret = tas256x_rx_set_edge(p_tas256x, p_tas256x->mn_rx_edge,
		channel_both);
	if (ret < 0)
		goto end;
	ret = tas256x_rx_set_start_slot(p_tas256x,
		p_tas256x->mn_rx_start_slot, channel_both);
	if (ret < 0)
		goto end;
	ret |= tas256x_set_samplerate(p_tas256x, p_tas256x->mn_sampling_rate,
		channel_both);
	if (ret < 0)
		goto end;
	ret |= tas256x_set_power_state(p_tas256x, p_tas256x->mn_power_state);
	if (ret < 0)
		goto end;
end:
	p_tas256x->enable_irq(p_tas256x, true);
/* power up failed, restart later */
	if (ret < 0) {
		if (p_tas256x->mn_err_code &
			(ERROR_DEVA_I2C_COMM | ERROR_DEVB_I2C_COMM))
			tas256x_failsafe(p_tas256x);
	}
}

static int tas2558_specific(struct tas256x_priv *p_tas256x, int chn)
{
	int ret = 0;

	pr_err("%s: chn %d\n", __func__, chn);
	ret = tas256x_boost_volt_update(p_tas256x, DEVICE_TAS2558, chn);

	return ret;
}

static int tas2564_specific(struct tas256x_priv *p_tas256x, int chn)
{
	int ret = 0;

	pr_err("%s: chn %d\n", __func__, chn);
	ret = tas256x_boost_volt_update(p_tas256x, DEVICE_TAS2564, chn);

	return ret;
}

static int tas2562_specific(struct tas256x_priv *p_tas256x, int chn)
{
	int ret = 0;

	pr_err("%s: chn %d\n", __func__, chn);
	ret = tas256x_boost_volt_update(p_tas256x, DEVICE_TAS2562, chn);

	return ret;
}

int tas256x_irq_work_func(struct tas256x_priv *p_tas256x)
{
	unsigned int nDevInt1Status = 0, nDevInt2Status = 0,
		nDevInt3Status = 0, nDevInt4Status = 0;
	int n_counter = 2;
	int n_result = 0;
	int irqreg, irqreg2, i, chnTemp = 0;
	enum channel chn = channel_left;

	pr_err("%s: \n", __func__);

	p_tas256x->enable_irq(p_tas256x, false);
	
	if (p_tas256x->mn_err_code & ERROR_FAILSAFE)
		goto reload;

	if (p_tas256x->mn_power_state == TAS256X_POWER_SHUTDOWN) {
		pr_err("%s: device not powered\n", __func__);
		goto end;
	}

	n_result = tas256x_interrupt_enable(p_tas256x, 0/*Disable*/,
			channel_both);
	if (n_result < 0)
		goto reload;

	/*Reset error codes*/
	p_tas256x->mn_err_code = 0;

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (p_tas256x->devs[i]->spk_control == 1)
			chnTemp |= 1<<i;
	}
	chn = (chnTemp == 0) ? chn : (enum channel)chnTemp;

	if (chn & channel_left) {
		n_result = tas256x_interrupt_read(p_tas256x,
			&nDevInt1Status, &nDevInt2Status, channel_left);
		if (n_result < 0)
			goto reload;
		p_tas256x->mn_err_code =
			tas256x_interrupt_determine(p_tas256x, channel_left,
				nDevInt1Status, nDevInt2Status);
	}

	if (chn & channel_right) {
		n_result = tas256x_interrupt_read(p_tas256x,
			&nDevInt3Status, &nDevInt4Status, channel_right);
		if (n_result < 0)
			goto reload;
		p_tas256x->mn_err_code |=
			tas256x_interrupt_determine(p_tas256x, channel_right,
				nDevInt3Status, nDevInt4Status);
	}

	pr_err("%s: IRQ status : 0x%x, 0x%x, 0x%x, 0x%x mn_err_code %d\n",
		__func__,
		nDevInt1Status, nDevInt2Status,
		nDevInt3Status, nDevInt4Status,
		p_tas256x->mn_err_code);

	if (p_tas256x->mn_err_code)
		goto reload;
	else {
		pr_err("%s: Power Up \n", __func__);
		n_counter = 2;
		while (n_counter > 0) {
			if (chn & channel_left)
				n_result = tas256x_power_check(p_tas256x,
						&nDevInt1Status,
						channel_left);
			if (n_result < 0)
				goto reload;
			if (chn & channel_right)
				n_result = tas256x_power_check(p_tas256x,
						&nDevInt3Status,
						channel_right);
			if (n_result < 0)
				goto reload;

			if (nDevInt1Status) {
				/* If only left should be power on */
				if (chn == channel_left)
					break;
				/* If both should be power on */
				if (nDevInt3Status)
					break;
			} else if (chn == channel_right) {
				/*If only right should be power on */
				if (nDevInt3Status)
					break;
			}

			tas256x_interrupt_read(p_tas256x,
				&irqreg, &irqreg2, chn);

			n_result = tas256x_set_power_up(p_tas256x, chn);
			if (n_result < 0)
				goto reload;

			pr_err("%s: set ICN to -80dB\n", __func__);
			n_result = tas256x_icn_data(p_tas256x, chn);

			n_counter--;
			if (n_counter > 0) {
			/* in case check pow status
			 *just after power on TAS256x
			 */
				if (chn & channel_left)
					pr_err("%s: PowSts A: 0x%x, check again after 10ms\n",
						__func__,
						nDevInt1Status);

				if (chn & channel_right)
					pr_err("%s: PowSts B: 0x%x, check again after 10ms\n",
						__func__,
						nDevInt3Status);

				msleep(20);
			}
		}

		if (((!nDevInt1Status) && (chn & channel_left))
			|| ((!nDevInt3Status) && (chn & channel_right))) {
				if (chn & channel_right)
					pr_err("%s, Critical ERROR A REG[POWERCONTROL] = 0x%x\n",
						__func__,
						nDevInt1Status);
				
				if (chn & channel_right)
					pr_err("%s, Critical ERROR B REG[POWERCONTROL] = 0x%x\n",
						__func__,
						nDevInt3Status);
				goto reload;
		}
	}

	n_result = tas256x_interrupt_enable(p_tas256x, 1/*Enable*/,
		chn);
	if (n_result < 0)
		goto reload;

	goto end;

reload:
	/* hardware reset and reload */
	tas256x_load_config(p_tas256x);

end:
	p_tas256x->enable_irq(p_tas256x, true);

	return n_result;
}

int tas256x_init_work_func(struct tas256x_priv *p_tas256x)
{
	int n_result = 0;

	pr_err("%s: \n", __func__);

	n_result = tas256x_set_power_up(p_tas256x, channel_both);
	n_result = tas256x_icn_data(p_tas256x, channel_both);

	return n_result;
}

int tas256x_dc_work_func(struct tas256x_priv *p_tas256x, int ch)
{
	int n_result = 0;

	pr_err("%s: ch %d\n", __func__, ch);
	tas256x_reload(p_tas256x, ch);

	return n_result;
}

int tas256x_register_device(struct tas256x_priv  *p_tas256x)
{
	int n_result;
	int i;

	pr_err("%s:\n", __func__);
	p_tas256x->read = tas256x_dev_read;
	p_tas256x->write = tas256x_dev_write;
	p_tas256x->bulk_read = tas256x_dev_bulk_read;
	p_tas256x->bulk_write = tas256x_dev_bulk_write;
	p_tas256x->update_bits = tas256x_dev_update_bits;

	tas256x_hard_reset(p_tas256x);

	pr_err("Before SW reset\n");
	/* Reset the chip */
	n_result = tas56x_software_reset(p_tas256x, channel_left);
	if (n_result < 0) {
		pr_err("channel left I2c fail, %d\n", n_result);
	} else {
		p_tas256x->smartpa_i2c_check |= 0x01;
	}
	if (p_tas256x->mn_channels == 2) {
		n_result = tas56x_software_reset(p_tas256x, channel_right);
		if (n_result < 0) {
			pr_err("channel right I2c fail, %d\n", n_result);
		} else {
			p_tas256x->smartpa_i2c_check |= 0x02;
		}
	}
	if (p_tas256x->smartpa_i2c_check == 0) {
		pr_err("I2c fail, smartpa_i2c_check %d\n",
			p_tas256x->smartpa_i2c_check);
		goto err;
	}

	pr_err("After SW reset\n");

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		n_result = tas56x_get_chipid(p_tas256x,
			&(p_tas256x->devs[i]->mn_chip_id),
			(i == 0) ? channel_left : channel_right);
		if (n_result < 0)
			goto err;
		switch (p_tas256x->devs[i]->mn_chip_id) {
		case 0x10:
		case 0x20:
			pr_err("TAS2562 chip");
			p_tas256x->devs[i]->device_id = DEVICE_TAS2562;
			p_tas256x->devs[i]->dev_ops.tas_init = 
				tas2562_specific;
			break;
		case 0x00:
			pr_err("TAS2564 chip");
			p_tas256x->devs[i]->device_id = DEVICE_TAS2564;
			p_tas256x->devs[i]->dev_ops.tas_init =
				tas2564_specific;
			break;
		default:
			pr_err("TAS2558 chip");
			p_tas256x->devs[i]->device_id = DEVICE_TAS2558;
			p_tas256x->devs[i]->dev_ops.tas_init =
				tas2558_specific;
			break;
		}
		n_result |= tas256x_set_misc_config(p_tas256x, 0,
				(i == 0) ? channel_left : channel_right);
	}
err:
	return n_result;
}

int tas256x_probe(struct tas256x_priv *p_tas256x)
{
	int ret = 0;

	pr_err("%s:\n", __func__);
	ret = tas256x_load_init(p_tas256x);
	if (ret < 0)
		goto end;
	ret = tas256x_iv_sense_enable_set(p_tas256x, 1, channel_both);
#ifdef CONFIG_TAS25XX_ALGO
#ifndef CONFIG_TAS256x_FOR_MTK
	/*Send IV Vbat format but don't update to algo yet*/
	tas25xx_set_iv_bit_fomat(p_tas256x->mn_iv_width,
		p_tas256x->mn_vbat, 0);
#endif
#endif

end:
	return ret;
}

int tas256x_set_power_state(struct tas256x_priv *p_tas256x,
			int state)
{
	int n_result = 0, i = 0, chnTemp = 0;
	enum channel chn = channel_left;

	pr_err("%s: state %d\n", __func__, state);

	if ((p_tas256x->mb_mute) && (state == TAS256X_POWER_ACTIVE))
		state = TAS256X_POWER_MUTE;

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (p_tas256x->devs[i]->spk_control == 1)
			chnTemp |= 1<<i;
	}
	chn = (chnTemp == 0) ? chn:(enum channel)chnTemp;

	switch (state) {
	case TAS256X_POWER_ACTIVE:
		/* if set format was not called by asoc, then set it default */
		if (p_tas256x->mn_rx_edge == 0) {
			n_result = tas256x_rx_set_edge(p_tas256x, 0, chn);
		}
		if (p_tas256x->mn_rx_start_slot == 0) {
			n_result |= tas256x_rx_set_start_slot(p_tas256x, 1, chn);
		}
		if (n_result < 0)
			return n_result;
		n_result = tas256x_iv_sense_enable_set(p_tas256x, 1, chn);
#ifdef CONFIG_TAS25XX_ALGO
		tas25xx_send_algo_calibration();

		/*Moved to probe*/
		/*tas25xx_set_iv_bit_fomat (p_tas256x->mn_iv_width,
		 *p_tas256x->mn_vbat, 1);
		 */
#endif
		/* Clear latched IRQ before power on */
		tas256x_interrupt_clear(p_tas256x, chn);

		p_tas256x->mb_power_up = true;
		p_tas256x->mn_power_state = TAS256X_POWER_ACTIVE;
		p_tas256x->schedule_init_work(p_tas256x);
		break;

	case TAS256X_POWER_MUTE:
		n_result = tas256x_set_power_mute(p_tas256x, chn);
			p_tas256x->mb_power_up = true;
			p_tas256x->mn_power_state = TAS256X_POWER_MUTE;

		/*Mask interrupt for TDM*/
		n_result = tas256x_interrupt_enable(p_tas256x, 0/*Disable*/,
			chn);
		break;

	case TAS256X_POWER_SHUTDOWN:
		for (i = 0; i < p_tas256x->mn_channels; i++) {
			if (p_tas256x->devs[i]->device_id == DEVICE_TAS2564) {
				if (chn & (i+1)) {
					/*Mask interrupt for TDM*/
					n_result = tas256x_interrupt_enable(p_tas256x, 0/*Disable*/,
							i+1);
					n_result = tas256x_set_power_mute(p_tas256x, i+1);
					n_result = tas256x_iv_sense_enable_set(p_tas256x, 0,
						i+1);
					p_tas256x->mb_power_up = false;
					p_tas256x->mn_power_state = TAS256X_POWER_SHUTDOWN;
					msleep(20);
				}
			} else {
				n_result = tas256x_set_power_shutdown(p_tas256x, i+1);
				n_result = tas256x_iv_sense_enable_set(p_tas256x, 0,
					i+1);
				p_tas256x->mb_power_up = false;
				p_tas256x->mn_power_state = TAS256X_POWER_SHUTDOWN;
				msleep(20);
				/*Mask interrupt for TDM*/
				n_result = tas256x_interrupt_enable(p_tas256x, 0/*Disable*/,
					i+1);
			}
		}
		p_tas256x->enable_irq(p_tas256x, false);
#ifdef CONFIG_TAS25XX_ALGO
		//tas25xx_update_big_data();
#endif

		break;

	default:
		pr_err("wrong power state setting %d\n", state);
	}

	return n_result;
}

int tas256x_iv_vbat_slot_config(struct tas256x_priv *p_tas256x, int mn_slot_width)
{
	int n_result = 0;

	pr_err("%s: mn_slot_width %d\n", __func__, mn_slot_width);
	if (p_tas256x->mn_channels == 2) {
		if (mn_slot_width == 16) {
			n_result |= tas256x_set_iv_slot(p_tas256x, channel_left, TX_SLOT1, TX_SLOT0);
			n_result |= tas256x_set_iv_slot(p_tas256x, channel_right, TX_SLOT3, TX_SLOT2);
		} else { /*16 or 32 bit*/
			if (p_tas256x->mn_iv_width == 8) {
				n_result |= tas256x_set_iv_slot(p_tas256x, channel_left, TX_SLOT1, TX_SLOT0);
				n_result |= tas256x_set_iv_slot(p_tas256x, channel_right, TX_SLOT5, TX_SLOT4);
				if (p_tas256x->mn_vbat == 1) {
					n_result |= tas256x_set_vbat_slot(p_tas256x, channel_left, TX_SLOT2);
					n_result |= tas256x_set_vbat_slot(p_tas256x, channel_right, TX_SLOT6);
				}
			} else { /*p_tas256x->mn_iv_width == 16 & VBat cannot exist in this combination*/
				n_result |= tas256x_set_iv_slot(p_tas256x, channel_left, TX_SLOT2, TX_SLOT0);
				n_result |= tas256x_set_iv_slot(p_tas256x, channel_right, TX_SLOT6, TX_SLOT4);
			}
		}
	} else if ((p_tas256x->mn_channels == 1)
		&& (mn_slot_width == 32)) {
		if (p_tas256x->mn_iv_width == 16) {
			n_result |= tas256x_set_iv_slot(p_tas256x, channel_left, TX_SLOT4, TX_SLOT0);
		} else if (p_tas256x->mn_iv_width == 12) {
			n_result |= tas256x_set_iv_slot(p_tas256x, channel_left, TX_SLOT1, TX_SLOT0);
			if (p_tas256x->mn_vbat == 1)
				n_result |= tas256x_set_vbat_slot(p_tas256x, channel_left, TX_SLOT4);
		}
	} else if ((p_tas256x->mn_channels == 1)
		&& (mn_slot_width == 16)) {
		if (p_tas256x->mn_iv_width == 16) {
			n_result |= tas256x_set_iv_slot(p_tas256x, channel_left, TX_SLOT2, TX_SLOT0);
		} else if (p_tas256x->mn_iv_width == 12) {
			n_result |= tas256x_set_iv_slot(p_tas256x, channel_left, TX_SLOT1, TX_SLOT0);
			if (p_tas256x->mn_vbat == 1)
				n_result |= tas256x_set_vbat_slot(p_tas256x, channel_left, TX_SLOT3);
		}
	} else {
		n_result = -1;
	}

	if (n_result == 0)
		p_tas256x->mn_tx_slot_width = mn_slot_width;

	return n_result;
}

int tas256x_set_bitwidth(struct tas256x_priv *p_tas256x,
	int bitwidth, int stream)
{
	int n_result = 0;
	int slot_width_tmp = 16;

	if (bitwidth == 24)
		slot_width_tmp = 32;
	else  if (bitwidth == 32)
		slot_width_tmp = 32;

	pr_err("%s: bitwidth %d stream %d\n", __func__, bitwidth, stream);

	if (stream == TAS256X_STREAM_PLAYBACK) {
		n_result |= tas256x_rx_set_bitwidth(p_tas256x, bitwidth, channel_both);
		n_result |= tas256x_rx_set_slot(p_tas256x, slot_width_tmp, channel_both);
	} else { /*stream == TAS256X_STREAM_CAPTURE*/
		n_result |= tas256x_iv_bitwidth_config(p_tas256x, p_tas256x->mn_iv_width, channel_both);
		n_result |= tas256x_iv_vbat_slot_config(p_tas256x, slot_width_tmp);
	}

	if (n_result < 0) {
		if (p_tas256x->mn_err_code &
			(ERROR_DEVA_I2C_COMM | ERROR_DEVB_I2C_COMM))
			tas256x_failsafe(p_tas256x);
	}

	return n_result;
}
