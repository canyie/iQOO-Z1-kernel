/*
 * aw87xxx.c   aw87xxx pa module
 *
 * Version: v0.1.0
 *
 * Copyright (c) 2020 AWINIC Technology CO., LTD
 *
 *  Author: Alex <zhangpengbiao@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include "aw87xxx.h"
#include "aw87339.h"
#include "aw87359.h"
#include "aw87519.h"
#include "aw87xxx_mnt.h"
#include <linux/debugfs.h>
#include "vivo-codec-common.h"

/*****************************************************************
* aw87xxx marco
******************************************************************/
#define AW87XXX_I2C_NAME	"aw87xxx_pa"
#define AW87XXX_DRIVER_VERSION	"v0.1.0"

#ifndef BBK_IQOO_AUDIO_DEBUG
#define BBK_IQOO_AUDIO_DEBUG
#endif

/*************************************************************************
 * aw87xxx variable
 ************************************************************************/
#ifdef BBK_IQOO_AUDIO_DEBUG
static struct dentry *aw87xxx_debugfs_root;
static struct dentry *aw87xxx_debugfs_reg;
static struct dentry *aw87xxx_debugfs_i2c;
static struct kobject *aw87xxx_kobj;
#endif
static bool chipid_ok;
struct aw87xxx *g_aw87xxx;

static char aw87xxx_cfg_name[AW87XXX_MODE_MAX][AW87XXX_CFG_NAME_MAX] = {
	{"aw87xxx_pid_xx_off.bin"},
	{"aw87xxx_pid_xx_music.bin"},
	{"aw87xxx_pid_xx_voice.bin"},
	{"aw87xxx_pid_xx_fm.bin"},
	{"aw87xxx_pid_xx_rcv.bin"},
};



/**********************************************************
* i2c write and read
**********************************************************/
static int aw87xxx_i2c_write(struct aw87xxx *aw87xxx,
	unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw87xxx->i2c_client,
			reg_addr, reg_data);
		if (ret < 0) {
			aw_dev_err(aw87xxx->dev, "%s: i2c_write cnt=%d error=%d\n",
				__func__, cnt, ret);
		} else {
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw87xxx_i2c_read(struct aw87xxx *aw87xxx,
	unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw87xxx->i2c_client,
						reg_addr);
		if (ret < 0) {
			aw_dev_err(aw87xxx->dev, "%s: i2c_read cnt=%d error=%d\n",
				__func__, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

/************************************************************************
* aw87xxx hardware control
************************************************************************/
static int aw87xxx_hw_on(struct aw87xxx *aw87xxx)
{
	aw_dev_info(aw87xxx->dev, "%s enter\n", __func__);

	if (gpio_is_valid(aw87xxx->reset_gpio)) {
		gpio_set_value_cansleep(aw87xxx->reset_gpio, 0);
		mdelay(2);
		gpio_set_value_cansleep(aw87xxx->reset_gpio, 1);
		mdelay(2);
	}
	aw87xxx->hwen_flag = 1;

	return 0;
}

static int aw87xxx_hw_off(struct aw87xxx *aw87xxx)
{
	aw_dev_info(aw87xxx->dev, "%s enter\n", __func__);

	if (gpio_is_valid(aw87xxx->reset_gpio)) {
		gpio_set_value_cansleep(aw87xxx->reset_gpio, 0);
		mdelay(2);
	}
	aw87xxx->hwen_flag = 0;

	return 0;
}

/*******************************************************************************
* aw87xxx control interface
******************************************************************************/
static int aw87xxx_audio_off(struct aw87xxx *aw87xxx)
{
	int i;
	struct aw87xxx_scene_param *aw_off =
		&aw87xxx->aw87xxx_scene_ls.aw_off;

	if (!aw_off || !aw_off->scene_cnt) {
		aw_dev_err(aw87xxx->dev, "%s: param or cnt is null. \n", __func__);
		return -EINVAL;
	}
	if (aw_off->scene_cnt->len % 2) {
		aw_dev_err(aw87xxx->dev, "%s: the size of para not in 2 times, return here. \n", __func__);
		return -EINVAL;
	}

	if (aw87xxx->hwen_flag) {
		if (aw_off->cfg_update_flag == AW87XXX_CFG_OK) {
			/*send firmware data*/
			for (i = 0; i < aw_off->scene_cnt->len; i = i+2) {
				aw_dev_dbg(aw87xxx->dev, "%s: reg=0x%02x, val = 0x%02x\n",
					__func__, aw_off->scene_cnt->data[i],
					aw_off->scene_cnt->data[i+1]);

				aw87xxx_i2c_write(aw87xxx,
					aw_off->scene_cnt->data[i],
					aw_off->scene_cnt->data[i+1]);
			}
			aw87xxx->current_mode = aw_off->scene_mode;
		}
	}

	aw87xxx_hw_off(aw87xxx);

	aw87xxx_mnt_stop(&aw87xxx->mnt);

	return 0;
}

static int aw87xxx_reg_cnt_update(struct aw87xxx *aw87xxx,
	struct aw87xxx_scene_param *scene_param)
{
	int i;
	struct aw87xxx_container *load_cnt =
				scene_param->scene_cnt;

	aw_dev_info(aw87xxx->dev, "%s enter, mode:%d\n",
		__func__, scene_param->scene_mode);
	if (!aw87xxx->hwen_flag)
		aw87xxx_hw_on(aw87xxx);

	if (!load_cnt) {
		aw_dev_err(aw87xxx->dev, "%s:cnt is null. \n", __func__);
		return -EINVAL;
	}
	if (load_cnt->len % 2) {
		aw_dev_err(aw87xxx->dev, "%s: the size of para not in 2 times, return here. \n", __func__);
		return -EINVAL;
	}

	if (scene_param->cfg_update_flag == AW87XXX_CFG_OK) {
		/*send firmware data*/
		for (i = 0; i < load_cnt->len; i = i+2) {
			aw_dev_dbg(aw87xxx->dev, "%s: reg=0x%02x, val = 0x%02x\n",
				__func__, load_cnt->data[i],
				load_cnt->data[i+1]);
			aw87xxx_i2c_write(aw87xxx,
					load_cnt->data[i],
					load_cnt->data[i+1]);
		}
		aw87xxx->current_mode = scene_param->scene_mode;
	} else {
		aw_dev_err(aw87xxx->dev, "%s: scene not ready\n",
				__func__);
		return -ENOENT;
	}

	if (aw87xxx->current_mode == AW87XXX_RCV_MODE)
		aw87xxx_mnt_stop(&aw87xxx->mnt);
	else
		aw87xxx_mnt_start(&aw87xxx->mnt);

	return 0;
}

unsigned char aw87xxx_show_current_mode(void)
{
	struct aw87xxx *aw87xxx = g_aw87xxx;

	if (aw87xxx == NULL) {
		aw_dev_err(aw87xxx->dev, "%s struct aw87xxx not ready\n",
			__func__);
		return -EPERM;
	}

	return aw87xxx->current_mode;
}

int aw87xxx_audio_scenne_load(uint8_t mode)
{
	struct aw87xxx_scene_param *scene_param;
	struct aw87xxx *aw87xxx = g_aw87xxx;

	if (aw87xxx == NULL) {
		aw_dev_err(aw87xxx->dev, "%s struct aw87xxx not ready\n",
			__func__);
		return -EPERM;
	}

	aw_dev_info(aw87xxx->dev, "%s enter\n",
		__func__);
	switch (aw87xxx->chipid) {
	case AW87XXX_PID_39:
		aw_dev_info(aw87xxx->dev, "%s 39 on.\n", __func__);
		break;
	case AW87XXX_PID_59:
		aw_dev_info(aw87xxx->dev, "%s 59 on.\n", __func__);
		break;
	case AW87XXX_PID_69:
		aw_dev_info(aw87xxx->dev, "%s 69 on.\n", __func__);
		break;
	default:
		aw_dev_info(aw87xxx->dev, "%s: unsupported device revision (0x%x)\n",
			__func__, aw87xxx->chipid);
		break;
	}

	switch (mode) {
	case AW87XXX_OFF_MODE:
		return aw87xxx_audio_off(aw87xxx);
	case AW87XXX_MUSIC_MODE:
		scene_param = &aw87xxx->aw87xxx_scene_ls.aw_music;
		break;
	case AW87XXX_VOICE_MODE:
		scene_param = &aw87xxx->aw87xxx_scene_ls.aw_voice;
		break;
	case AW87XXX_FM_MODE:
		scene_param = &aw87xxx->aw87xxx_scene_ls.aw_fm;
		break;
	case AW87XXX_RCV_MODE:
		scene_param = &aw87xxx->aw87xxx_scene_ls.aw_rcv;
		break;
	default:
		aw_dev_err(aw87xxx->dev, "%s unsupported mode: %d\n",
			__func__, mode);
		return -EINVAL;
	}

	return aw87xxx_reg_cnt_update(aw87xxx, scene_param);
}

static int aw87xxx_pa_enable(int state)
{
	int ret = 0;

	pr_info("%s: enter, state: %d\n",
		__func__, state);

	switch (state) {
	case EXT_PA_SWICH_VOICE:
		ret = aw87xxx_audio_scenne_load(AW87XXX_VOICE_MODE);
		break;
	case EXT_PA_SWICH_MUSIC:
		ret = aw87xxx_audio_scenne_load(AW87XXX_MUSIC_MODE);
		break;
	case EXT_PA_SWICH_FM:
		ret = aw87xxx_audio_scenne_load(AW87XXX_FM_MODE);
		break;
	case EXT_PA_SWICH_NONE:
		ret = aw87xxx_audio_scenne_load(AW87XXX_OFF_MODE);
		break;
	default:
		ret = aw87xxx_audio_scenne_load(AW87XXX_OFF_MODE);
		break;
	}

	pr_info("%s: leave.\n", __func__);

	return ret;
}

/****************************************************************************
* aw873xx firmware cfg update
***************************************************************************/
static void aw87xxx_parse_chip_name(struct aw87xxx *aw87xxx,
	uint32_t *chip_name, uint32_t len)
{
	int i;
	char chip_name_str[8] = {0};

	memcpy(chip_name_str, (char *)chip_name, len);
	for (i = 0; i < len; i++) {
		if (chip_name_str[i] == 'A')
			break;
	}
	memcpy(aw87xxx->chip_name, chip_name_str + i, len - i);
	aw_dev_info(aw87xxx->dev, "%s: chip name:%s\n",
		__func__, aw87xxx->chip_name);
}

static int aw87xxx_get_parse_fw_way(struct aw87xxx *aw87xxx,
	const struct firmware *cont)
{
	int i = 0;
	uint32_t sum_data = 0;
	uint32_t check_sum = 0;

	aw_dev_info(aw87xxx->dev, "%s enter\n", __func__);

	memcpy(&check_sum, cont->data, sizeof(check_sum));
	/*check bit*/
	for (i = sizeof(check_sum); i < cont->size; i++)
		sum_data += cont->data[i];

	aw_dev_info(aw87xxx->dev, "%s: check_sum:%d sum_data:%d\n",
			__func__, check_sum, sum_data);
	if (check_sum != sum_data) {
		aw_dev_info(aw87xxx->dev, "%s: use old parsing\n",
			__func__);
		return AW87XXX_OLD_FIRMWARE;
	} else {
		aw_dev_info(aw87xxx->dev, "%s: use frame head parsing\n",
			__func__);
		return AW87XXX_FH_FIRMWARE;
	}
}

static int aw87xxx_fh_parse_firmware(struct aw87xxx *aw87xxx,
	struct aw87xxx_scene_param *current_scene,
	const struct firmware *cont)
{
	int i = 0;
	struct ui_frame_header *frame_header;
	struct aw87xxx_container *curr_scene_cnt;

	aw_dev_info(aw87xxx->dev, "%s enter\n", __func__);
	if (current_scene->frame_header != NULL)
		current_scene->frame_header = NULL;

	frame_header = current_scene->frame_header =
		kzalloc(sizeof(struct ui_frame_header), GFP_KERNEL);
	if (!frame_header) {
		aw_dev_err(aw87xxx->dev, "%s: Error allocating memory\n",
			__func__);
		return -ENOMEM;
	}

	memcpy(frame_header, cont->data, sizeof(struct ui_frame_header));

	/*check addr wide, data wide and bin type*/
	if (frame_header->addr_bit_wide != AW87XXX_ADDR_BIT_WIDE ||
		frame_header->data_bit_wide != AW87XXX_DATA_BIT_WIDE ||
		frame_header->bin_data_type != AW87XXX_BIN_TYPE_REG) {
		aw_dev_err(aw87xxx->dev, "%s: %d bin file type mismatch\n",
			__func__, frame_header->bin_data_type);
	}

	aw_dev_dbg(aw87xxx->dev, "%s: bin_fh_ver:0x%08x\n",
		__func__, frame_header->bin_fh_ver);
	aw_dev_dbg(aw87xxx->dev, "%s: ui_ver:0x%08x\n",
		__func__, frame_header->ui_ver);
	aw_dev_dbg(aw87xxx->dev, "%s: bin_data_ver:0x%08x\n",
		__func__, frame_header->bin_data_ver);
	aw_dev_dbg(aw87xxx->dev, "%s: bin_data_type:0x%08x\n",
		__func__, frame_header->bin_data_type);

	aw87xxx_parse_chip_name(aw87xxx, frame_header->chip_name,
		sizeof(frame_header->chip_name));

	if (current_scene->scene_cnt != NULL)
		current_scene->scene_cnt = NULL;

	curr_scene_cnt = current_scene->scene_cnt =
		kzalloc(frame_header->bin_data_len + sizeof(int), GFP_KERNEL);
	if (!curr_scene_cnt) {
		aw_dev_err(aw87xxx->dev, "%s: Error allocating memory\n",
			__func__);
		devm_kfree(aw87xxx->dev, frame_header);
		return -ENOMEM;
	}

	curr_scene_cnt->len = frame_header->bin_data_len;
	memcpy(curr_scene_cnt->data, cont->data + sizeof(struct ui_frame_header),
		frame_header->bin_data_len);
	current_scene->cfg_update_flag = AW87XXX_CFG_OK;
	if (curr_scene_cnt->len % 2) {
		aw_dev_err(aw87xxx->dev, "%s: the size of para not in 2 times, return here. \n", __func__);
		return -EINVAL;
	}
	for (i = 0; i < curr_scene_cnt->len; i = i+2) {
		aw_dev_info(aw87xxx->dev, "%s: addr:0x%02x, data:0x%02x\n",
			__func__, curr_scene_cnt->data[i],
			curr_scene_cnt->data[i+1]);
	}

	aw_dev_info(aw87xxx->dev, "%s: %s update complete\n",
		__func__, aw87xxx->cfg_name[current_scene->scene_mode]);
	return 0;
}

static void aw87xxx_old_parse_firmware(
	struct aw87xxx *aw87xxx,
	struct aw87xxx_scene_param *current_scene,
	const struct firmware *cont)
{
	int i = 0;
	struct aw87xxx_container *scene_cnt;

	aw_dev_info(aw87xxx->dev, "%s enter\n", __func__);
	if (current_scene->scene_cnt != NULL)
		current_scene->scene_cnt = NULL;

	scene_cnt = current_scene->scene_cnt =
		kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!scene_cnt) {
		aw_dev_err(aw87xxx->dev, "%s: Error allocating memory\n",
			__func__);
		return;
	}

	scene_cnt->len = cont->size;
	memcpy(scene_cnt->data, cont->data, cont->size);
	current_scene->cfg_update_flag = AW87XXX_CFG_OK;

	if (cont->size % 2) {
		aw_dev_err(aw87xxx->dev, "%s: the size of para not in 2 times, return here. \n", __func__);
		return;
	}

	for (i = 0; i < scene_cnt->len; i = i+2) {
		aw_dev_info(aw87xxx->dev, "%s: addr:0x%02x, data:0x%02x\n",
			__func__, scene_cnt->data[i],
			scene_cnt->data[i+1]);
	}

	aw_dev_info(aw87xxx->dev, "%s: %s update complete\n",
		__func__, aw87xxx->cfg_name[current_scene->scene_mode]);
}

static void aw87xxx_parse_firmware(struct aw87xxx *aw87xxx,
	struct aw87xxx_scene_param *current_scene,
	const struct firmware *cont)
{
	int ret;

	ret = aw87xxx_get_parse_fw_way(aw87xxx, cont);
	if (ret == AW87XXX_OLD_FIRMWARE)
		aw87xxx_old_parse_firmware(aw87xxx, current_scene, cont);
	else if (ret == AW87XXX_FH_FIRMWARE)
		aw87xxx_fh_parse_firmware(aw87xxx, current_scene, cont);

	release_firmware(cont);
}

static void aw87xxx_scene_cfg_loaded(const struct firmware *cont, void *context)
{
	int ram_timer_val = 2000;
	struct aw87xxx_scene_param *current_scene = context;
	struct aw87xxx *aw87xxx;

	pr_info("%s enter\n", __func__);
	switch (current_scene->scene_mode) {
	case AW87XXX_OFF_MODE:
		aw87xxx = container_of(current_scene,
				struct aw87xxx, aw87xxx_scene_ls.aw_off);
		break;
	case AW87XXX_MUSIC_MODE:
		aw87xxx = container_of(current_scene,
				struct aw87xxx, aw87xxx_scene_ls.aw_music);
		break;
	case AW87XXX_VOICE_MODE:
		aw87xxx = container_of(current_scene,
				struct aw87xxx, aw87xxx_scene_ls.aw_voice);
		break;
	case AW87XXX_FM_MODE:
		aw87xxx = container_of(current_scene,
				struct aw87xxx, aw87xxx_scene_ls.aw_fm);
		break;
	case AW87XXX_RCV_MODE:
		aw87xxx = container_of(current_scene,
				struct aw87xxx, aw87xxx_scene_ls.aw_rcv);
		break;
	default:
		pr_err("%s: unsupported scence:%d\n",
			__func__, current_scene->scene_mode);
		return;
	}

	current_scene->update_num++;
	if (!cont) {
		aw_dev_err(aw87xxx->dev, "%s: failed to read %s\n",
			__func__, aw87xxx->cfg_name[current_scene->scene_mode]);

		release_firmware(cont);

		if (current_scene->update_num < AW87XXX_LOAD_CFG_RETRIES) {
			aw_dev_info(aw87xxx->dev, "%s:restart hrtimer to load firmware\n",
				__func__);
			schedule_delayed_work(&aw87xxx->ram_work,
					msecs_to_jiffies(ram_timer_val));
		}
		return;
	}

	aw_dev_info(aw87xxx->dev, "%s: loaded %s - size: %zu\n",
		__func__, aw87xxx->cfg_name[current_scene->scene_mode],
		cont ? cont->size : 0);

	/* aw87xxx ram update */
	aw87xxx_parse_firmware(aw87xxx, current_scene, cont);
}

static int aw87xxx_scene_update(struct aw87xxx *aw87xxx, uint8_t scence_mode)
{
	struct aw87xxx_scene_param *current_scene = NULL;

	aw_dev_info(aw87xxx->dev, "%s enter\n", __func__);
	switch (scence_mode) {
	case AW87XXX_OFF_MODE:
		current_scene = &aw87xxx->aw87xxx_scene_ls.aw_off;
		break;
	case AW87XXX_MUSIC_MODE:
		current_scene = &aw87xxx->aw87xxx_scene_ls.aw_music;
		break;
	case AW87XXX_VOICE_MODE:
		current_scene = &aw87xxx->aw87xxx_scene_ls.aw_voice;
		break;
	case AW87XXX_FM_MODE:
		current_scene = &aw87xxx->aw87xxx_scene_ls.aw_fm;
		break;
	case AW87XXX_RCV_MODE:
		current_scene = &aw87xxx->aw87xxx_scene_ls.aw_rcv;
		break;
	default:
		aw_dev_err(aw87xxx->dev, "%s: unsupported scence:%d\n",
			__func__, scence_mode);
		return -EINVAL;
	}

	return request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					aw87xxx->cfg_name[scence_mode],
					aw87xxx->dev,
					GFP_KERNEL,
					current_scene,
					aw87xxx_scene_cfg_loaded);
}

static void aw87xxx_vmax_cfg_loaded(
	const struct firmware *cont, void *context)
{
	struct aw87xxx *aw87xxx = context;
	struct vmax_config *vmax_cfg;
	int vmax_cfg_num = 0, i;
	int ram_timer_val = 2000;

	aw87xxx->mnt.update_num++;
	if (!cont) {
		aw_dev_err(aw87xxx->dev, "%s: failed to read %s\n",
			__func__, aw87xxx_vmax_cfg_name);

		release_firmware(cont);

		if (aw87xxx->mnt.update_num < AW87XXX_LOAD_CFG_RETRIES) {
			aw_dev_info(aw87xxx->dev, "%s:restart hrtimer to load firmware\n",
				__func__);
			schedule_delayed_work(&aw87xxx->ram_work,
					msecs_to_jiffies(ram_timer_val));
		}
		return;
	}

	if (aw87xxx->mnt.vmax_cfg != NULL)
		aw87xxx->mnt.vmax_cfg = NULL;

	if (cont->size % 8) {
		aw_dev_err(aw87xxx->dev, "%s: the size of para not in 8 times, return here. \n", __func__);
		return;
	}
	vmax_cfg_num = cont->size / sizeof(struct vmax_single_config);

	vmax_cfg = aw87xxx->mnt.vmax_cfg =
		kzalloc((vmax_cfg_num * sizeof(struct vmax_config) +
			vmax_cfg_num * sizeof(struct vmax_single_config)),
			GFP_KERNEL);
	if (!vmax_cfg)
		return;

	vmax_cfg->vmax_cfg_num = vmax_cfg_num;
	for (i = 0; i < cont->size; i += 8) {
		vmax_cfg->vmax_cfg_total[i / 8].min_thr = (cont->data[i+3]<<24) +
							(cont->data[i+2]<<16) +
							(cont->data[i+1]<<8) +
							(cont->data[i]);

		vmax_cfg->vmax_cfg_total[i / 8].vmax = (cont->data[i+7]<<24) +
							(cont->data[i+6]<<16) +
							(cont->data[i+5]<<8) +
							(cont->data[i+4]);

		aw_dev_info(aw87xxx->dev, "%s: minthr:%d ,vmax:0x%08x\n",
			__func__, vmax_cfg->vmax_cfg_total[i / 8].min_thr,
			vmax_cfg->vmax_cfg_total[i / 8].vmax);
	}

	release_firmware(cont);

	aw87xxx->mnt.cfg_update_flag = AW87XXX_CFG_OK;

	aw_dev_info(aw87xxx->dev, "%s: vbat mnt update complete\n",
		__func__);
}

static int aw87xxx_vbat_mnt_update(struct aw87xxx *aw87xxx)
{
	aw_dev_info(aw87xxx->dev, "%s enter\n", __func__);

	return request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					aw87xxx_vmax_cfg_name,
					aw87xxx->dev,
					GFP_KERNEL,
					aw87xxx,
					aw87xxx_vmax_cfg_loaded);
}

static void aw87xxx_cfg_work_routine(struct work_struct *work)
{
	struct aw87xxx *aw87xxx = container_of(work,
				struct aw87xxx, ram_work.work);
	struct aw87xxx_scene_list *aw_scene_ls = &aw87xxx->aw87xxx_scene_ls;

	aw_dev_info(aw87xxx->dev, "%s enter\n", __func__);
	aw_scene_ls->aw_off.scene_mode = AW87XXX_OFF_MODE;
	aw_scene_ls->aw_music.scene_mode = AW87XXX_MUSIC_MODE;
	aw_scene_ls->aw_voice.scene_mode = AW87XXX_VOICE_MODE;
	aw_scene_ls->aw_fm.scene_mode = AW87XXX_FM_MODE;
	aw_scene_ls->aw_rcv.scene_mode = AW87XXX_RCV_MODE;

	if (aw_scene_ls->aw_off.cfg_update_flag == AW87XXX_CFG_WAIT)
		aw87xxx_scene_update(aw87xxx,
				aw_scene_ls->aw_off.scene_mode);
	if (aw_scene_ls->aw_music.cfg_update_flag == AW87XXX_CFG_WAIT)
		aw87xxx_scene_update(aw87xxx,
				aw_scene_ls->aw_music.scene_mode);
	if (aw_scene_ls->aw_voice.cfg_update_flag == AW87XXX_CFG_WAIT)
		aw87xxx_scene_update(aw87xxx,
				aw_scene_ls->aw_voice.scene_mode);
	if (aw_scene_ls->aw_fm.cfg_update_flag == AW87XXX_CFG_WAIT)
		aw87xxx_scene_update(aw87xxx,
				aw_scene_ls->aw_fm.scene_mode);
	if (aw_scene_ls->aw_rcv.cfg_update_flag == AW87XXX_CFG_WAIT)
		aw87xxx_scene_update(aw87xxx,
				aw_scene_ls->aw_rcv.scene_mode);
	if (aw87xxx->mnt.cfg_update_flag == AW87XXX_CFG_WAIT)
		aw87xxx_vbat_mnt_update(aw87xxx);
}

static int aw87xxx_cfg_init(struct aw87xxx *aw87xxx)
{
	int cfg_timer_val = 8000;

	INIT_DELAYED_WORK(&aw87xxx->ram_work, aw87xxx_cfg_work_routine);
	schedule_delayed_work(&aw87xxx->ram_work,
		msecs_to_jiffies(cfg_timer_val));

	return 0;
}

/****************************************************************************
* aw87xxx compatible
*****************************************************************************/
static int aw87xxx_choice_device(struct aw87xxx *aw87xxx, uint8_t chip_type)
{

	switch (chip_type) {
	case AW87XXX_339:
		aw87xxx->reg_param.reg_max = AW87339_REG_MAX;
		aw87xxx->reg_param.reg_access = aw87339_reg_access;
		break;
	case AW87XXX_359:
	case AW87XXX_369:
		aw87xxx->reg_param.reg_max = AW87359_REG_MAX;
		aw87xxx->reg_param.reg_access = aw87359_reg_access;
		break;
	case AW87XXX_519:
		aw87xxx->reg_param.reg_max = AW87519_REG_MAX;
		aw87xxx->reg_param.reg_access = aw87519_reg_access;
		break;
	default:
		aw_dev_err(aw87xxx->dev, "%s unsupport chip type:%d\n",
		__func__, chip_type);
		return -EINVAL;
	}
	return 0;
}

static void aw87xxx_cfg_free(struct aw87xxx *aw87xxx)
{
	int i;
	struct aw87xxx_scene_param *scene_param =
			&aw87xxx->aw87xxx_scene_ls.aw_off;

	for (i = 0; i < AW87XXX_MODE_MAX; i++) {
		kfree(scene_param->frame_header);
		kfree(scene_param->scene_cnt);

		aw_dev_dbg(aw87xxx->dev, "kfree %d cfg",
			scene_param->scene_mode);
		scene_param++;
	}
	memset(&aw87xxx->aw87xxx_scene_ls, 0, sizeof(struct aw87xxx_scene_list));

	kfree(aw87xxx->mnt.vmax_cfg);
	aw87xxx->mnt.cfg_update_flag = AW87XXX_CFG_WAIT;
	aw87xxx->mnt.update_num = AW87XXX_CFG_NUM_INIT;
}

/****************************************************************************
* aw87xxx attribute
*****************************************************************************/
static ssize_t aw87xxx_get_reg(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);

	for (i = 0; i < aw87xxx->reg_param.reg_max; i++) {
		if (!(aw87xxx->reg_param.reg_access[i] & AW87XXX_REG_RD_ACCESS))
			continue;
		aw87xxx_i2c_read(aw87xxx, i, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}

	return len;
}

static ssize_t aw87xxx_set_reg(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[2] = {0};
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw87xxx_i2c_write(aw87xxx, databuf[0], databuf[1]);

	return len;
}

static ssize_t aw87xxx_get_hwen(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);

	len += snprintf(buf+len, PAGE_SIZE-len, "hwen: %d\n",
			aw87xxx->hwen_flag);

	return len;
}

static ssize_t aw87xxx_set_hwen(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	ssize_t ret;
	unsigned int state;
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 10, &state);
	if (ret) {
		aw_dev_err(aw87xxx->dev, "%s: fail to change str to int\n",
			__func__);
		return ret;
	}
	if (state == 0) {		/*OFF*/
		aw87xxx_hw_off(aw87xxx);
	} else {			/*ON*/
		aw87xxx_hw_on(aw87xxx);
	}

	return len;
}

static ssize_t aw87xxx_set_update(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	ssize_t ret;
	unsigned int state;
	int cfg_timer_val = 10;
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 10, &state);
	if (ret) {
		aw_dev_err(aw87xxx->dev, "%s: fail to change str to int\n",
			__func__);
		return ret;
	}
	if (state) {
		aw87xxx_cfg_free(aw87xxx);

		schedule_delayed_work(&aw87xxx->ram_work,
				msecs_to_jiffies(cfg_timer_val));
	}

	return len;
}

static ssize_t aw87xxx_get_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);

	len += snprintf(buf+len, PAGE_SIZE-len, "0: off mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "1: music mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "2: voice mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "3: fm mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "4: rcv mode\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "current mode:%d\n",
			aw87xxx->current_mode);

	return len;
}

static ssize_t aw87xxx_set_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int ret;
	uint32_t mode;
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 8, &mode);
	if (ret) {
		aw_dev_err(aw87xxx->dev, "%s: fail to change str to int\n",
		__func__);
		return ret;
	}
	ret = aw87xxx_audio_scenne_load((uint8_t)mode);
	if (ret) {
		aw_dev_err(aw87xxx->dev, "%s: load scene:%s faid\n",
			__func__, buf);
	}

	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,
	aw87xxx_get_reg, aw87xxx_set_reg);
static DEVICE_ATTR(hwen, S_IWUSR | S_IRUGO,
	aw87xxx_get_hwen, aw87xxx_set_hwen);
static DEVICE_ATTR(update, S_IWUSR,
	NULL, aw87xxx_set_update);
static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO,
	aw87xxx_get_mode, aw87xxx_set_mode);

static struct attribute *aw87xxx_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_update.attr,
	&dev_attr_mode.attr,
	NULL
};

static struct attribute_group aw87xxx_attribute_group = {
	.attrs = aw87xxx_attributes
};

/****************************************************************************
* aw87xxx parse dts
*****************************************************************************/
static void aw87xxx_parse_gpio_dt(struct aw87xxx *aw87xxx,
	struct device_node *np)
{
	aw87xxx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw87xxx->reset_gpio < 0) {
		dev_err(aw87xxx->dev, "%s: no reset gpio provided, will not HW reset device\n",
			__func__);
		aw87xxx->reset_gpio = -1;
	} else {
		dev_info(aw87xxx->dev, "%s: reset gpio provided ok\n",
			__func__);
	}
}

static void aw87xxx_parse_dt(struct aw87xxx *aw87xxx, struct device_node *np)
{
	aw87xxx_parse_gpio_dt(aw87xxx, np);
	aw87xxx_parse_mnt_dt(&aw87xxx->mnt);
}

/****************************************************************************
* aw87xxx hardware reset
*****************************************************************************/
static int aw87xxx_hw_reset(struct aw87xxx *aw87xxx)
{
	aw_dev_info(aw87xxx->dev, "%s enter\n", __func__);

	if (aw87xxx != NULL) {
		if (gpio_is_valid(aw87xxx->reset_gpio)) {
			aw_dev_info(aw87xxx->dev, "%s aw87xxx's reset_gpio is valid: %d\n", __func__, aw87xxx->reset_gpio);
			gpio_set_value_cansleep(aw87xxx->reset_gpio, 0);
			mdelay(2);
			gpio_set_value_cansleep(aw87xxx->reset_gpio, 1);
			mdelay(2);
			aw87xxx->hwen_flag = 1;
			aw_dev_info(aw87xxx->dev, "%s be careful here, becouse if reset GPIO is not matched, it may occurs I2C ack error\n", __func__);
			return 0;
		} else {
			aw_dev_err(aw87xxx->dev, "%s aw87xxx's reset_gpio is invalid: %d\n", __func__, aw87xxx->reset_gpio);
			aw87xxx->hwen_flag = 0;
			return -EINVAL;
		}
	} else {
		aw87xxx->hwen_flag = 0;
		aw_dev_err(aw87xxx->dev, "%s: aw87xxx is NULL\n", __func__);
		return -EINVAL;
	}
}

/*****************************************************
* check chip id
*****************************************************/
static int aw87xxx_update_cfg_name(struct aw87xxx *aw87xxx)
{
	char aw87xxx_head[] = {"aw87xxx_pid_"};
	char buf[3] = {0};
	int i = 0;
	uint8_t head_index = 0;

	memcpy(aw87xxx->cfg_name, aw87xxx_cfg_name, sizeof(aw87xxx_cfg_name));
	head_index = sizeof(aw87xxx_head) - 1;

	/*add product information */
	snprintf(buf, sizeof(buf)+1, "%02x", aw87xxx->chipid);

	for (i = 0; i < sizeof(aw87xxx->cfg_name) / AW87XXX_CFG_NAME_MAX; i++) {
		memcpy(aw87xxx->cfg_name[i] + head_index, buf,
			sizeof(buf) - 1);
	}

	for (i = 0; i < sizeof(aw87xxx->cfg_name) / AW87XXX_CFG_NAME_MAX; i++) {
		aw_dev_info(aw87xxx->dev, "%s: %s\n",
			__func__, aw87xxx->cfg_name[i]);
	}

	return 0;
}

static int aw87xxx_read_chipid(struct aw87xxx *aw87xxx)
{
	unsigned int cnt = 0;
	int ret = -1;
	unsigned char reg_val = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw87xxx_i2c_read(aw87xxx, AW87XXX_REG_CHIPID, &reg_val);
		aw_dev_info(aw87xxx->dev, "%s: the chip is aw87xxx chipid=0x%x\n",
				__func__, reg_val);
		chipid_ok = true;
		switch (reg_val) {
		case AW87XXX_PID_39:
			aw87xxx->chipid = reg_val;
			aw87xxx_choice_device(aw87xxx, AW87XXX_339);
			aw87xxx_update_cfg_name(aw87xxx);
			return 0;
		case AW87XXX_PID_59:
			aw87xxx->chipid = reg_val;
			if (gpio_is_valid(aw87xxx->reset_gpio))
				aw87xxx_choice_device(aw87xxx, AW87XXX_519);
			else
				aw87xxx_choice_device(aw87xxx, AW87XXX_359);
			aw87xxx_update_cfg_name(aw87xxx);
			return 0;
		case AW87XXX_PID_69:
			aw87xxx->chipid = reg_val;
			aw87xxx_choice_device(aw87xxx, AW87XXX_369);
			aw87xxx_update_cfg_name(aw87xxx);
			return 0;
		default:
			aw_dev_info(aw87xxx->dev, "%s: unsupported device revision (0x%x)\n",
				__func__, reg_val);
			break;
		}
		cnt++;

		mdelay(AW_READ_CHIPID_RETRY_DELAY);
	}
	chipid_ok = false;
	aw_dev_err(aw87xxx->dev, "%s: aw87xxx chipid=0x%x error\n",
		__func__, reg_val);
	return -EINVAL;
}

#ifdef BBK_IQOO_AUDIO_DEBUG
static int aw87xxx_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t aw87xxx_debug_write(struct file *filp,
					const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	struct aw87xxx *aw87xxx = g_aw87xxx;
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
	aw87xxx_i2c_write(aw87xxx,
			kbuf[0],
			kbuf[1]);

	kfree(temp);

	return cnt;
}
static ssize_t aw87xxx_debug_read(struct file *file, char __user *buf,
					size_t count, loff_t *pos)
{
	struct aw87xxx *aw87xxx = g_aw87xxx;
	unsigned char reg_val;
	const int size = 512;
	char buffer[size];
	int len = 0;
	u8 i;

	for (i = 0; i < aw87xxx->reg_param.reg_max; i++) {
		if (!(aw87xxx->reg_param.reg_access[i] & AW87XXX_REG_RD_ACCESS))
			continue;
		aw87xxx_i2c_read(aw87xxx, i, &reg_val);
		len += snprintf(buffer+len, size-len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}

	buffer[len] = 0;
	pr_info("%s:============caught aw87xxx reg end =============\n", __func__);

	return simple_read_from_buffer(buf, count, pos, buffer, len);
}

static struct file_operations aw87xxx_debugfs_fops = {
	.open = aw87xxx_debug_open,
	.read = aw87xxx_debug_read,
	.write = aw87xxx_debug_write,
};

static ssize_t aw87xxx_debug_i2c_read(struct file *file, char __user *buf,
					size_t count, loff_t *pos)
{
	struct aw87xxx *aw87xxx = g_aw87xxx;
	const int size = 512;
	char buffer[size];
	int n = 0;

	pr_info("%s enter.\n", __func__);

	n += scnprintf(buffer+n, size-n, "SmartPA-0x%x %s\n", aw87xxx->i2c_client->addr,
					chipid_ok ? "OK" : "ERROR");

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations aw87xxx_i2c_debugfs_fops = {
	.open = aw87xxx_debug_open,
	.read = aw87xxx_debug_i2c_read,
};

static void aw87xxx_debugfs_init(void)
{
	aw87xxx_debugfs_root = debugfs_create_dir("audio-aw87xxx", NULL);
	if (!aw87xxx_debugfs_root) {
		pr_err("%s debugfs create dir error\n", __func__);
	} else if (IS_ERR(aw87xxx_debugfs_root)) {
		pr_err("%s Kernel not support debugfs \n", __func__);
		aw87xxx_debugfs_root = NULL;
	}

	aw87xxx_debugfs_reg = debugfs_create_file("reg", 0644,
		aw87xxx_debugfs_root, NULL, &aw87xxx_debugfs_fops);
	if (!aw87xxx_debugfs_reg) {
		pr_err("aw87xxx debugfs create fail \n");
	}
	aw87xxx_debugfs_i2c = debugfs_create_file("i2c", 0444,
		aw87xxx_debugfs_root, NULL, &aw87xxx_i2c_debugfs_fops);
	if (!aw87xxx_debugfs_i2c) {
		pr_err("aw87xxx i2c create fail \n");
	}

	return ;
}

static void aw87xxx_debugfs_deinit(void)
{
	debugfs_remove(aw87xxx_debugfs_i2c);
	debugfs_remove(aw87xxx_debugfs_reg);
	debugfs_remove(aw87xxx_debugfs_root);
	return ;
}

static ssize_t aw87xxx_reg_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *ubuf, size_t count)
{
	int ret = 0;
	unsigned int kbuf[2];
	char *temp;
	struct aw87xxx *aw87xxx = g_aw87xxx;

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

	pr_info("%s: kbuf[0]=%x, kbuf[1]=%x cnt =%d\n", __func__, kbuf[0], kbuf[1], (int)count);

	aw87xxx_i2c_write(aw87xxx, kbuf[0], kbuf[1]);
	kfree(temp);

	return count;
}

static ssize_t aw87xxx_reg_show(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
	unsigned char reg_val;
	const int size = 512;
	int len = 0;
	u8 i;
	struct aw87xxx *aw87xxx = g_aw87xxx;

	pr_info("%s:============caught aw87xxx reg start =============\n", __func__);

	for (i = 0; i < aw87xxx->reg_param.reg_max; i++) {
		if (!(aw87xxx->reg_param.reg_access[i] & AW87XXX_REG_RD_ACCESS))
			continue;
		aw87xxx_i2c_read(aw87xxx, i, &reg_val);
		len += snprintf(buffer+len, size-len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}

	buffer[len] = 0;

	pr_info("%s:============caught aw87xxx reg end =============\n", __func__);

	return len;
}

static ssize_t aw87xxx_i2c_show(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
	const int size = 512;
	int n = 0;
	struct aw87xxx *aw87xxx = g_aw87xxx;

	pr_info("%s enter.\n", __func__);

	n += scnprintf(buffer+n, size-n, "SmartPA-0x%x %s\n", aw87xxx->i2c_client->addr,
				   chipid_ok ? "OK" : "ERROR");

	buffer[n] = 0;

	return n;
}

static struct kobj_attribute dev_attr_reg1 =
	__ATTR(reg, 0664, aw87xxx_reg_show, aw87xxx_reg_store);
static struct kobj_attribute dev_attr_i2c =
	__ATTR(i2c, 0664, aw87xxx_i2c_show, NULL);

static struct attribute *sys_node_attributes[] = {
	&dev_attr_reg1.attr,
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

	scnprintf(name, 48, "audio-aw87xxx");

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

/****************************************************************************
* aw87xxx i2c driver
*****************************************************************************/
static int aw87xxx_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct vivo_codec_function *vivo_codec_function = get_vivo_codec_function();
	struct device_node *np = client->dev.of_node;
	struct aw87xxx *aw87xxx;
	int ret = -1;

	aw_dev_info(&client->dev, "%s Enter\n", __func__);
	if (!vivo_codec_function) {
		pr_err("%s:vivo_codec_function malloc failed\n", __func__);
		return -EPROBE_DEFER;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		aw_dev_err(&client->dev, "%s: check_functionality failed\n",
			__func__);
		ret = -ENODEV;
		goto exit_check_functionality_failed;
	}

	g_aw87xxx = aw87xxx = devm_kzalloc(&client->dev,
				sizeof(struct aw87xxx),
				GFP_KERNEL);
	if (aw87xxx == NULL) {
		ret = -ENOMEM;
		goto exit_devm_kzalloc_failed;
	}

	aw87xxx->dev = &client->dev;
	aw87xxx->i2c_client = client;
	i2c_set_clientdata(client, aw87xxx);

	if (np)
		aw87xxx_parse_dt(aw87xxx, np);
	else {
		aw87xxx->reset_gpio = -1;
		aw_dev_err(&client->dev, "%s: reset_gpio is invalid: %d\n",
				__func__, aw87xxx->reset_gpio);
		goto reset_gpio_failed;
	}

	if (gpio_is_valid(aw87xxx->reset_gpio)) {
		ret = devm_gpio_request_one(&client->dev,
					aw87xxx->reset_gpio,
					GPIOF_OUT_INIT_LOW, "aw87xxx_rst");
		if (ret) {
			aw_dev_err(&client->dev,
				"%s: rst request failed\n", __func__);
			goto exit_gpio_request_failed;
		}
	}

	if (client->dev.of_node)
		dev_set_name(&client->dev, "%s", "aw87xxx_smartpa");
	else
		aw_dev_err(&client->dev, "%s failed to set device name\n",
				__func__);

	ret = aw87xxx_hw_reset(aw87xxx);
	if (ret)
		goto exit_hw_reset_failed;

	aw_dev_info(&client->dev, "%s before read chipid\n", __func__);

	/* aw87xxx chip id */
	ret = aw87xxx_read_chipid(aw87xxx);
	if (ret < 0) {
		aw_dev_err(&client->dev, "%s: aw873xx_read_chipid failed ret=%d\n",
			__func__, ret);
		goto exit_i2c_check_id_failed;
	}

	ret = sysfs_create_group(&client->dev.kobj, &aw87xxx_attribute_group);
	if (ret < 0) {
		aw_dev_info(&client->dev, "%s error creating sysfs attr files\n",
			__func__);
	}

	aw87xxx_cfg_init(aw87xxx);

	vivo_codec_function->ext_pa_enable = aw87xxx_pa_enable;
#ifdef BBK_IQOO_AUDIO_DEBUG
	aw87xxx_debugfs_init();
	class_attr_create(aw87xxx_kobj);
#endif
	aw87xxx_hw_off(aw87xxx);

	aw87xxx_mnt_init(&aw87xxx->mnt);
	return 0;

exit_hw_reset_failed:
exit_i2c_check_id_failed:
	if (gpio_is_valid(aw87xxx->reset_gpio))
		devm_gpio_free(&client->dev, aw87xxx->reset_gpio);
exit_gpio_request_failed:
reset_gpio_failed:
	devm_kfree(&client->dev, aw87xxx);
	aw87xxx = NULL;
exit_devm_kzalloc_failed:
exit_check_functionality_failed:
	return ret;
}

static int aw87xxx_i2c_remove(struct i2c_client *client)
{
	struct aw87xxx *aw87xxx = i2c_get_clientdata(client);

#ifdef BBK_IQOO_AUDIO_DEBUG
	aw87xxx_debugfs_deinit();
	if (aw87xxx_kobj)
		class_attr_remove(aw87xxx_kobj);
#endif
	if (gpio_is_valid(aw87xxx->reset_gpio))
		devm_gpio_free(&client->dev, aw87xxx->reset_gpio);

	devm_kfree(&client->dev, aw87xxx);
	aw87xxx = NULL;

	return 0;
}

static const struct i2c_device_id aw87xxx_i2c_id[] = {
	{ AW87XXX_I2C_NAME, 0 },
	{ }
};

static const struct of_device_id extpa_of_match[] = {
	{.compatible = "awinic,aw87xxx_pa"},
	{},
};


static struct i2c_driver aw87xxx_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = AW87XXX_I2C_NAME,
		.of_match_table = extpa_of_match,
	},
	.probe = aw87xxx_i2c_probe,
	.remove = aw87xxx_i2c_remove,
	.id_table = aw87xxx_i2c_id,
};

static int __init aw87xxx_pa_init(void)
{
	int ret;

	pr_info("%s enter\n", __func__);
	pr_info("%s: driver version: %s\n", __func__, AW87XXX_DRIVER_VERSION);

	ret = i2c_add_driver(&aw87xxx_i2c_driver);
	if (ret) {
		pr_info("****[%s] Unable to register driver (%d)\n",
				__func__, ret);
		return ret;
	}
	return 0;
}

static void __exit aw87xxx_pa_exit(void)
{
	pr_info("%s enter\n", __func__);
	i2c_del_driver(&aw87xxx_i2c_driver);
}

module_init(aw87xxx_pa_init);
module_exit(aw87xxx_pa_exit);

MODULE_AUTHOR("<zhangpengbiao@awinic.com>");
MODULE_DESCRIPTION("awinic aw87xxx pa driver");
MODULE_LICENSE("GPL v2");
