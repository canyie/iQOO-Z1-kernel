#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/rwsem.h>
#include <linux/sensors.h>
#include <linux/string.h>
#include <linux/delay.h>
#include "vts_core.h"

static struct class *vts_class = NULL;
static atomic_t nr_devices = ATOMIC_INIT(0);

#define VTS_FW_ATTR(name, type) \
	static ssize_t vts_##name##_show(struct device *dev, \
				struct device_attribute *attr, char *buf) \
	{ \
		struct vts_device *vtsdev = dev_get_drvdata(dev); \
		int ret; \
	\
		ret = vts_call_func_sync(vtsdev, int, vts_fw_path_get, type, buf, PAGE_SIZE); \
		if (ret) \
			return ret; \
	\
		sprintf(buf + strlen(buf), "\n"); \
		return strlen(buf); \
	} \
	\
	static ssize_t vts_##name##_store(struct device *dev, \
			struct device_attribute *attr, const char *buf, size_t size) \
	{ \
		struct vts_device *vtsdev = dev_get_drvdata(dev); \
		int ret; \
		char *tmp = (char *)buf; \
		char *path; \
		char *update_s; \
		int update; \
	\
		path = strsep(&tmp, ","); \
		if (!path) { \
			vts_dev_err(vtsdev, "invalid args!"); \
			return -EINVAL; \
		} \
	\
		update_s = strsep(&tmp, ","); \
		if (!update_s) { \
			vts_dev_err(vtsdev, "invalid args!"); \
			return -EINVAL; \
		} \
	\
		if(kstrtoint(update_s, 10, &update)) { \
			vts_dev_err(vtsdev, "invalid args!"); \
			return -EINVAL; \
		} \
	\
		ret = vts_call_func_sync(vtsdev, int, vts_fw_path_set, type, buf, update != 0); \
		if (ret) \
			return ret; \
	\
		return size; \
	} \
	static DEVICE_ATTR(name, 0644, vts_##name##_show, vts_##name##_store)

static ssize_t vts_lcmid_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u32 lcmid;
	int ret;

	ret = vts_call_func_sync(vtsdev, int, vts_get_lcmid, &lcmid);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", lcmid);
}

static ssize_t vts_lcmid_compatible_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u32 lcmid[VTS_MODULE_LCMID_NR_MAX];
	int ret;
	ssize_t count = 0;
	size_t sz = ARRAY_SIZE(lcmid);
	int i;

	memset(lcmid, 0, sizeof(lcmid));
	ret = vts_call_func_sync(vtsdev, int, vts_get_lcmid_compatible, lcmid, &sz);
	if (ret)
		return ret;

	if (sz == 0)
		return -ENODEV;

	for (i = 0; i < sz; i++)
		if (i == (sz -1))
			count += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf), "%d", lcmid[i]);
		else
			count += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf), "%d ", lcmid[i]);

	return count;
}

static ssize_t vts_version_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u64 version;
	int ret;
	ret = vts_call_func_sync(vtsdev, int, vts_firmware_version_get, &version);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "0x%llx\n", version);
}

VTS_FW_ATTR(firmware, VTS_FW_TYPE_FW);
VTS_FW_ATTR(firmware_config, VTS_FW_TYPE_CONFIG);
VTS_FW_ATTR(threshold, VTS_FW_TYPE_LIMIT);
VTS_FW_ATTR(firmware_mp, VTS_FW_TYPE_MP);
static DEVICE_ATTR(lcmid, 0644, vts_lcmid_show, NULL);
static DEVICE_ATTR(lcmid_compatible, 0644, vts_lcmid_compatible_show, NULL);
static DEVICE_ATTR(version, 0644, vts_version_show, NULL);

static struct attribute *vts_dev_attrs[] = {
	&dev_attr_version.attr,
	&dev_attr_firmware.attr,
	&dev_attr_firmware_mp.attr,
	&dev_attr_firmware_config.attr,
	&dev_attr_threshold.attr,
	NULL,
};

static struct attribute_group vts_attr_group = {
	.name	= NULL,
	.attrs	= vts_dev_attrs,
};

static struct attribute *vts_property_attrs[] = {
	&dev_attr_lcmid.attr,
	&dev_attr_lcmid_compatible.attr,
	NULL,
};

static struct attribute_group property_attr_group = {
	.name	= "properties",
	.attrs	= vts_property_attrs,
};

const struct attribute_group *vts_class_groups[] = {
	&property_attr_group,
	&vts_attr_group,
	NULL,
};

static int vts_dev_suspend(struct device *dev)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	vts_dev_info(vtsdev, "suspended\n");
	vts_device_lock(vtsdev);
	return 0;
}

static int vts_dev_resume(struct device *dev)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	vts_dev_info(vtsdev, "resumed\n");
	vts_device_unlock(vtsdev);
	return 0;
}

struct dev_pm_ops vts_dev_pm = {
	.suspend = vts_dev_suspend,
	.resume = vts_dev_resume,
};

static int vts_class_init(void)
{
	int ret = 0;

	if(vts_class)
		return 0;

	vts_class = class_create(THIS_MODULE, "vts");
	if (IS_ERR_OR_NULL(vts_class)) {
		VTE("create class vts failed ret = %ld\n", PTR_ERR(vts_class));
		ret = PTR_ERR(vts_class);
		vts_class = NULL;
		return ret;
	}
	vts_class->dev_groups = vts_class_groups;
	vts_class->pm = &vts_dev_pm;
	return 0;
}

static void vts_class_exit(void)
{
	if (!vts_class)
		return ;

	class_destroy(vts_class);
	vts_class = NULL;
}

int vts_classdev_register(struct device *parent,
				struct vts_device *vtsdev)
{
	int ret;
	u32 val;

	ret = vts_property_get(vtsdev, VTS_PROPERTY_PANEL_TYPE, &val);
	if (ret) {
		vts_dev_err(vtsdev, "get panle type error, ret = %d\n", ret);
		return ret;
	}

	if (atomic_inc_return(&nr_devices) == 1) {
		ret = vts_class_init();
		if (ret)
			return ret;
	}

	vtsdev->dev = device_create(vts_class, parent, val,
				      vtsdev, "%s", vts_name(vtsdev));
	if (IS_ERR(vtsdev->dev)) {
		vts_dev_err(vtsdev, "device create failed! ret = %ld\n", PTR_ERR(vtsdev->dev));
		if (atomic_dec_return(&nr_devices) == 0)
			vts_class_exit();
		return PTR_ERR(vtsdev->dev);
	}

	vts_dev_info(vtsdev, "class device registered\n");
	return 0;
}

void vts_classdev_unregister(struct vts_device *vtsdev)
{
	int ret;
	u32 val;

	ret = vts_property_get(vtsdev, VTS_PROPERTY_PANEL_TYPE, &val);
	if (ret) {
		vts_dev_err(vtsdev, "get panle type error, ret = %d\n", ret);
		return ;
	}

	device_destroy(vts_class, val);
	vts_dev_info(vtsdev, "class device unregistered\n");
	if (atomic_dec_return(&nr_devices) == 0)
		vts_class_exit();
}

