#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include "vts_core.h"

struct vts_node {
	/* touch driver manager */
	struct mutex lock;
	struct list_head tps;

	/* vts core code info */
	int code_version;

	/* sys/touchscreen node use */
	struct kobject kobj;
	int imei_read_mode;
	int tune_cmd;
	int cmd_code;
	int at_sensor_test_cmd;
};

struct vts_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count);
};

static ssize_t vts_null_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
		return -EINVAL;
}

struct vts_cmd_handler {
	const char *tag;
	enum vts_state state_id;
};

const static struct vts_cmd_handler cmd_handlers[] = {
	{"vts_lcd", VTS_STA_LCD},
};

static int vts_state_id_lookup(const char *tag, enum vts_state *state_id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cmd_handlers); i++)
		if (!strcmp(cmd_handlers[i].tag, tag)) {
			*state_id =  cmd_handlers[i].state_id;
			return 0;
	}

	return -EINVAL;
}

static int vts_cmd_process(struct vts_device *vtsdev, enum vts_state state_id, int val)
{
	 vts_state_set(vtsdev, state_id, val);
	 return 0;
}

static bool match_name(struct vts_device *vtsdev, void *data)
{
	return strcmp(vts_name(vtsdev), (const char *)data) == 0 ? true : false;
}

static bool match_type(struct vts_device *vtsdev, void *data)
{
	u32 type = 0;

	vts_property_get(vtsdev, VTS_PROPERTY_PANEL_TYPE, &type);
	return type == (enum vts_type)data ? true : false;
}

static struct vts_device *vtsdev_lookup(struct vts_node *vtsc, bool (*match)(struct vts_device *vtsdev, void *data), void *data)
{
	struct vts_device *vtsdev;

	mutex_lock(&vtsc->lock);
	list_for_each_entry(vtsdev, &vtsc->tps, tp_list) {
		if (match(vtsdev, data)) {
			mutex_unlock(&vtsc->lock);
			return vtsdev;
		}
	}

	mutex_unlock(&vtsc->lock);
	return NULL;
}

static struct vts_device *vtsdev_lookup_by_type(struct vts_node *vtsc, enum vts_type type)
{
	return vtsdev_lookup(vtsc, match_type, (void *)type);
}

static struct vts_device *vtsdev_lookup_by_name(struct vts_node *vtsc, char *devname)
{
	return vtsdev_lookup(vtsc, match_name, devname);
}

static int vts_cmd_parse_dispatch(struct vts_node *vtsc, const char *buf)
{
	char *ptr = (char *)buf;
	char *tag;
	char *touch;
	char *state;
	int val;
	int type = 0;
	enum vts_state state_id;
	struct vts_device *vtsdev;

	tag = strsep(&ptr, ":");
	if (!tag) {
		VTE("invalid command format!\n");
		return -EINVAL;
	}


	if (vts_state_id_lookup(tag, &state_id)) {
		VTI("invalid tag! tag:%s\n", tag);
		return -EINVAL;
	}

	touch = strsep(&ptr, ":");
	if (!touch) {
		VTE("invalid command format!\n");
		return -EINVAL;
	}

	state = strsep(&ptr, ":");
	if (!state) {
		VTE("invalid command format!\n");
		return -EINVAL;
	}

	if (kstrtoint(state, 10, &val)) {
		VTE("invalid command format!\n");
		return -EINVAL;
	}

	if (!strcmp(touch, "ALL")) {
		for (type = 0; type <= VTS_TYPE_SECOND; type++) {
			vtsdev = vtsdev_lookup_by_type(vtsc, type);
			if (vtsdev)
				vts_cmd_process(vtsdev, state_id, val);
		}
		return 0;
	}

	vtsdev = vtsdev_lookup_by_name(vtsc, touch);
	if (!vtsdev) {
		VTE("invalid touch name or no related touch was registered! touch name is %s\n", touch);
		return -EINVAL;
	}

	vts_cmd_process(vtsdev, state_id, val);
	return 0;
}

int vts_get_screen_clock_zone(struct vts_screen_clock_cmd *dis, struct vts_screen_clock_zone *src)
{
	return 0;
}

static ssize_t vts_service_cmd_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);

	VTI("buf:%s\n", buf);
	vts_cmd_parse_dispatch(vtsc, buf);
	return count;
}

static struct vts_sysfs_entry vts_app_name = __ATTR(app_name, 0644, vts_null_show, vts_service_cmd_store);

static struct attribute *vts_sys_attrs[] = {
	&vts_app_name.attr,
	NULL
};

static ssize_t vts_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);
	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}
static ssize_t vts_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);
	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}

static void vts_object_release(struct kobject *kobj)
{
	return;
}

static const struct sysfs_ops vts_object_sysfs_ops = {
	.show = vts_object_show,
	.store = vts_object_store,
};

static struct kobj_type vts_object_type = {
	.sysfs_ops	= &vts_object_sysfs_ops,
	.release	= vts_object_release,
	.default_attrs = vts_sys_attrs,
};

static struct vts_node *vts = NULL;
static DEFINE_MUTEX(vts_node_mutex);

int vts_node_reset_fw_download(enum vts_type type)
{
	struct vts_device *vtsdev;
	int ret;

	mutex_lock(&vts_node_mutex);

	if (!vts) {
		VTE("no vts\n");
		mutex_unlock(&vts_node_mutex);
		return -EINVAL;
	}

	vtsdev = vtsdev_lookup_by_type(vts, type);
	if (!vtsdev) {
		VTE("no vts data\n");
		mutex_unlock(&vts_node_mutex);
		return -EINVAL;
	}

	ret = vts_call_func_sync(vtsdev, int, vts_reset);
	if (ret) {
		VTE("reset to download firmware failed!\n");
		mutex_unlock(&vts_node_mutex);
		return ret;
	}

	mutex_unlock(&vts_node_mutex);
	return ret;
}

static int vts_node_init(void)
{
	int ret;

	if (vts)
		return 0;

	vts = kzalloc(sizeof(*vts), GFP_KERNEL);
	if (!vts) {
		VTE("alloc memory for vts core failed!\n");
		return -ENOMEM;
	}

	ret = kobject_init_and_add(&vts->kobj, &vts_object_type, NULL, "touchscreen");
	if (ret) {
		VTE("create vts node faield!\n");
		kfree(vts);
		vts = NULL;
		return ret;
	}

	INIT_LIST_HEAD(&vts->tps);
	mutex_init(&vts->lock);
	vts->code_version = VTS_CODE_VERSION;
	return 0;
}

static void vts_node_exit(void)
{
	mutex_destroy(&vts->lock);
	kobject_del(&vts->kobj);
	kfree(vts);
	vts = NULL;
}

int vts_node_sysfs_add(struct vts_device *vtsdev)
{
	int ret;

	mutex_lock(&vts_node_mutex);
	ret = vts_node_init();
	if (ret) {
		vts_dev_err(vtsdev, "node init failed!\n");
		mutex_unlock(&vts_node_mutex);
		return ret;
	}

	mutex_lock(&vts->lock);
	list_add_tail(&vtsdev->tp_list, &vts->tps);
	mutex_unlock(&vts->lock);
	mutex_unlock(&vts_node_mutex);
	return 0;
}

void vts_node_sysfs_remove(struct vts_device *vtsdev)
{
	mutex_lock(&vts_node_mutex);

	if (!vts) {
		mutex_unlock(&vts_node_mutex);
		return ;
	}

	mutex_lock(&vts->lock);
	list_del(&vtsdev->tp_list);
	mutex_unlock(&vts->lock);

	if (list_empty(&vts->tps))
		vts_node_exit();

	mutex_unlock(&vts_node_mutex);
}
