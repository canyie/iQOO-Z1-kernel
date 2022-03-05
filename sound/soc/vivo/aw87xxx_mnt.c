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
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/hrtimer.h>
#include "aw87xxx_mnt.h"
#include "aw87xxx.h"


/*****************************************************
 * aw87xxx mnt control
*****************************************************/
int aw87xxx_mnt_stop(struct aw87xxx_mnt *mnt)
{

	return 0;
}

int aw87xxx_mnt_start(struct aw87xxx_mnt *mnt)
{

	return 0;
}


/****************************************************************************
* aw87xxx get battery capacity
*****************************************************************************/
int aw87xxx_get_battery_capacity(struct aw87xxx *aw87xxx,
	uint32_t *vbat_capacity)
{
	char name[] = "battery";
	int ret;
	union power_supply_propval prop = {0};
	struct power_supply *psy;

	aw_dev_info(aw87xxx->dev, "%s:enter\n", __func__);
	psy = power_supply_get_by_name(name);
	if (psy) {
		ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_CAPACITY, &prop);
		if (ret < 0) {
			aw_dev_err(aw87xxx->dev, "%s: get vbat capacity failed\n", __func__);
			return -EINVAL;
		}
		*vbat_capacity = prop.intval;
		aw_dev_info(aw87xxx->dev, "The percentage is %d\n", *vbat_capacity);
	} else {
		aw_dev_err(aw87xxx->dev, "no struct power supply name :%s", name);
		return -EINVAL;
	}
	return 0;
}

/****************************************************************************
* aw87xxx vmax attr
*****************************************************************************/
static ssize_t aw87xxx_get_vmax(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i = 0;
	size_t len = 0;
	uint32_t capacity;
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw87xxx_mnt *mnt = &aw87xxx->mnt;


	aw_dev_info(aw87xxx->dev, "%s:%s\n", __func__, aw87xxx->chip_name);
	if (aw87xxx->mnt.cfg_update_flag == AW87XXX_CFG_WAIT) {
		aw_dev_info(aw87xxx->dev, "%s: vmax bin load failed\n", __func__);
		len += snprintf(buf, PAGE_SIZE-len, "0\n");
		return len;
	}

	aw87xxx_get_battery_capacity(aw87xxx, &capacity);
	aw_dev_info(aw87xxx->dev, "%s: %d\n", __func__, capacity);
	if (capacity >= mnt->vmax_cfg->vmax_cfg_total[0].min_thr) {
		aw_dev_info(aw87xxx->dev, "%s:---- %d\n", __func__,
			mnt->vmax_cfg->vmax_cfg_total[0].min_thr);
		len += snprintf(buf, PAGE_SIZE-len, "%08x\n",
			mnt->vmax_cfg->vmax_cfg_total[0].vmax);
	} else {
		for (i = 0; i < mnt->vmax_cfg->vmax_cfg_num; ++i) {
			if ((capacity <= mnt->vmax_cfg->vmax_cfg_total[i].min_thr)
				&& (capacity >
				mnt->vmax_cfg->vmax_cfg_total[i+1].min_thr)) {
				aw_dev_info(aw87xxx->dev, "%s: vmax 0x%08x\n",
					__func__,
					mnt->vmax_cfg->vmax_cfg_total[i+1].vmax);
				len += snprintf(buf, PAGE_SIZE-len, "%08x\n",
				mnt->vmax_cfg->vmax_cfg_total[i+1].vmax);
			}
		}
	}
	return len;
}

static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO,
	aw87xxx_get_vmax, NULL);

static struct attribute *aw87xxx_mnt_attr[] = {
	&dev_attr_vmax.attr,
	NULL
};

static struct attribute_group aw87xxx_mnt_attr_group = {
	.attrs = aw87xxx_mnt_attr
};

/**********************************************************
 * aw87xxx mnt init
***********************************************************/
void aw87xxx_mnt_init(struct aw87xxx_mnt *mnt)
{
	int ret;
	struct aw87xxx *aw87xxx = container_of(mnt,
				struct aw87xxx, mnt);

	aw_dev_info(aw87xxx->dev, "%s: enter\n", __func__);

	ret = sysfs_create_group(&aw87xxx->dev->kobj,
				&aw87xxx_mnt_attr_group);
	if (ret < 0) {
		aw_dev_err(aw87xxx->dev, "%s error creating mnt sysfs attr files\n",
			__func__);
	}
}

void aw87xxx_parse_mnt_dt(struct aw87xxx_mnt *mnt)
{

	return;
}


