#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/firmware.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
#include <linux/sched/debug.h>
#endif

#include "vts_core.h"

#define VTS_FACTORY_KEY_DEFAULT "null:null:null:null:null"


#define ENUM_NAME(x) [x] = #x

static const char *name_of_states[] = {
	ENUM_NAME(VTS_STA_LCD),
	ENUM_NAME(VTS_STA_TDDI_LCD),
	ENUM_NAME(VTS_STA_PROX_STATE),
	ENUM_NAME(VTS_STA_VIRTUAL_PROX_STATE),
	ENUM_NAME(VTS_STA_GESTURE),
	ENUM_NAME(VTS_STA_FINGER_HIGHLIGHT),
	ENUM_NAME(VTS_STA_FACE_HIGHLIGHT),
	ENUM_NAME(VTS_STA_USB_CHARGE),
	ENUM_NAME(VTS_STA_ROTATION),
	ENUM_NAME(VTS_STA_SAVE_POWER),
	ENUM_NAME(VTS_STA_FORCE_NORMAL),
	ENUM_NAME(VTS_STA_NOTICE_UP),
	ENUM_NAME(VTS_STA_FORCE_DOUBLE),
	ENUM_NAME(VTS_STA_CALLING),
	ENUM_NAME(VTS_STA_DEFAUTL),
	ENUM_NAME(VTS_STA_OTHER_DEVICE_STATE_CHANGED),
	ENUM_NAME(VTS_STA_FINGER_UNLOCK_OPEN),
	ENUM_NAME(VTS_STA_GAME_MODE),
	ENUM_NAME(VTS_STA_VIRTUAL_KEY),
	ENUM_NAME(VTS_STA_VK_LONGPRESS),
	ENUM_NAME(VTS_STA_VK_ACTIVEMODE),
	ENUM_NAME(VTS_STA_BAND_STATE),
	ENUM_NAME(VTS_STA_RESET),
	ENUM_NAME(VTS_STA_IN_CALL),
	ENUM_NAME(VTS_STA_FACTORY_SWITCH),
	ENUM_NAME(VTS_STA_VIRTUAL_GAMEKEY),
	ENUM_NAME(VTS_STA_SCREEN_CLOCK_OPEN),
	ENUM_NAME(VTS_STA_SCREEN_CLOCK_REPORT_ABS),
	ENUM_NAME(VTS_STA_FORCE_DISABLE),
	ENUM_NAME(VTS_STA_FINGER_MODE),
	ENUM_NAME(VTS_STA_INPUT_METHOD),
	ENUM_NAME(VTS_STA_FINGER_CENTER),
	ENUM_NAME(VTS_STA_FINGER_MODE_SUPPORT),
};

static const char *vts_state_name(enum vts_state state)
{
	if (unlikely(state >= ARRAY_SIZE(name_of_states)))
		return "invalid state";

	return name_of_states[state];
}

static const char *name_of_workmodes[] = {
	ENUM_NAME(VTS_ST_NORMAL),
	ENUM_NAME(VTS_ST_SLEEP),
	ENUM_NAME(VTS_ST_GESTURE),
	ENUM_NAME(VTS_ST_UNKNOWN),
};

static const char *vts_mode_name(enum vts_run_mode mode)
{
	if (unlikely(mode >= ARRAY_SIZE(name_of_workmodes)))
		return "invalid mode";

	return name_of_workmodes[mode];
}

static atomic_t nr_messages = ATOMIC_INIT(0);
static DEFINE_MUTEX(vts_lock);
static LIST_HEAD(vts_devices);

#define CMDLINE_SIZE 4096
char str_cmdline[CMDLINE_SIZE];
unsigned int os_boot_loadfail;
static unsigned long lcm_software_id;

static u8 *__vts_fw_data_get(struct vts_device *vtsdev, enum vts_fw_type type, int *size, struct firmware **firmware);

enum vts_msg_type {
	VTS_MSG_TYPE_IRQ_EVENT,
	VTS_MSG_TYPE_CHANGE_MODE,
	VTS_MSG_TYPE_MODE_CHANGED_STATE_SYNC,
	VTS_MSG_TYPE_EXCEPTION,
	VTS_MSG_TYPE_UPDATE,
	VTS_MSG_TYPE_COUNT
};

struct vts_message {
	long index;
	enum vts_msg_type type;
	int arg1;
	int arg2;
	ktime_t arg3;
	struct list_head list;
};

static int vts_send_msg(struct vts_device *vtsdev, enum vts_msg_type type, int arg1, int arg2, ktime_t arg3, bool add_first);

#ifdef VTS_TEMP_ENABLE
static void boot_fw_update_work(struct work_struct *work);
#endif

struct vts_device *vts_device_alloc(void)
{
	struct vts_device *vtsdev = NULL;
	vtsdev = (struct vts_device *)kzalloc(sizeof(struct vts_device), GFP_KERNEL);
	if (!vtsdev) {
		VTE("fail kmalloc vtsdev");
		return NULL;
	}

	snprintf(vtsdev->msg_km_name, sizeof(vtsdev->msg_km_name), "vts_msg_km_%d", vtsdev->type);
	vtsdev->msg_km = kmem_cache_create(vtsdev->msg_km_name, sizeof(struct vts_message), 0, 0, NULL);
	if (!vtsdev->msg_km) {
		VTE("create mem cache failed!\n");
		kfree(vtsdev);
		return NULL;
	}

	vtsdev->finger3_mode = false;
	vtsdev->large_touches = 0;
	vtsdev->type = VTS_TYPE_MAIN;
	atomic_set(&vtsdev->mobile_state[VTS_STA_LCD], 1);
	atomic_set(&vtsdev->mobile_state[VTS_STA_PROX_STATE], 1);
	atomic_set(&vtsdev->mobile_state[VTS_STA_ROTATION], 1);
	atomic_set(&vtsdev->mobile_state[VTS_STA_TDDI_LCD], 1);
	atomic_set(&vtsdev->firmware_cache, 1);
	sema_init(&vtsdev->sem, 1);
	sema_init(&vtsdev->msg_sem, 0);
	spin_lock_init(&vtsdev->msg_lock);
	spin_lock_init(&vtsdev->mslock);
	INIT_LIST_HEAD(&vtsdev->frames);
	INIT_LIST_HEAD(&vtsdev->messages);
#ifdef VTS_TEMP_ENABLE
	INIT_DELAYED_WORK(&vtsdev->fw_update_work, boot_fw_update_work);
#endif
	return vtsdev;
}
EXPORT_SYMBOL(vts_device_alloc);

int vts_device_free(struct vts_device *vtsdev)
{
	vts_dev_info(vtsdev, "freed");

	if (vtsdev->msg_km != NULL) {
		kmem_cache_destroy(vtsdev->msg_km);
	    vtsdev->msg_km = NULL;
	}
	if(vtsdev->rom_data != NULL){
		kfree(vtsdev->rom_data);
		vtsdev->rom_data = NULL;
	}
	kfree(vtsdev);
	return 0;
}

static int vts_change_to_mode(struct vts_device *vtsdev, enum vts_run_mode old,
	enum vts_run_mode new, enum vts_state reason)
{
	int ret;

	if (old == new)
		return 0;

	ret = vts_call_ic_ops(vtsdev ,change_mode, new);
	if (ret) {
		vts_dev_err(vtsdev, "change to mode %s error, ret = %d\n", vts_mode_name(new), ret);
		return ret;
	}

	return 0;
}

enum vts_run_mode vts_get_run_mode(struct vts_device *vtsdev)
{
	return atomic_read(&vtsdev->run_mode);
}

static void vts_set_run_mode(struct vts_device *vtsdev, enum vts_run_mode mode)
{
	atomic_set(&vtsdev->run_mode, mode);
}

static enum vts_run_mode vts_expected_mode(struct vts_device *vtsdev)
{
	u32 tddi = 0;

	vts_state_dump(vtsdev);
	vts_property_get(vtsdev, VTS_PROPERTY_TDDI, &tddi);

	if (!tddi && vts_state_get(vtsdev, VTS_STA_LCD))
		return VTS_ST_NORMAL;

	if (tddi && vts_state_get(vtsdev, VTS_STA_TDDI_LCD))
		return VTS_ST_NORMAL;

	return VTS_ST_SLEEP;
}

static int vts_suspend_resume(struct vts_device *vtsdev, enum vts_state reason, int val)
{
	int ret = 0;
	enum vts_run_mode new;
	enum vts_run_mode old;
	s64 ktime_start;

	old = vts_get_run_mode(vtsdev);
	new = vts_expected_mode(vtsdev);
	if (old == new) {
		vts_dev_info(vtsdev, "%s:%d:%d, don't need to change mode, current:%s, expected:%s",
			vts_state_name(reason), val, vts_state_get(vtsdev, reason), vts_mode_name(old), vts_mode_name(new));
		return ret;
	}

	ktime_start = ktime_to_ms(ktime_get());
	ret = vts_change_to_mode(vtsdev, old, new, reason);
	if (ret) {
		vts_dev_err(vtsdev, "%s:%d:%d, change mode from %s to %s failed!, set state to VTS_ST_UNKNOWN",
			vts_state_name(reason), val, vts_state_get(vtsdev, reason), vts_mode_name(old), vts_mode_name(new));
		vts_set_run_mode(vtsdev, VTS_ST_UNKNOWN);
		return ret;
	}
	vts_dev_info(vtsdev, "%s:%d:%d, change mode from %s to %s success! take %lld ms",
		vts_state_name(reason),
		val,
		vts_state_get(vtsdev, reason),
		vts_mode_name(old), vts_mode_name(new), ktime_to_ms(ktime_get()) - ktime_start);

	vts_set_run_mode(vtsdev, new);
	vts_report_release(vtsdev);
	return 0;
}

static int __vts_state_set(struct vts_device *vtsdev, enum vts_state state, int val, bool async)
{
	if (state >= VTS_STA_MAX) {
		vts_dev_err(vtsdev, "invalid state %d\n", state);
		return -EINVAL;
	}

	if (vts_state_get(vtsdev, state) == val) {
		vts_dev_info(vtsdev, "same state value, state:%d ,val:%d\n", state, val);
		return 0;
	}

	atomic_set(&vtsdev->mobile_state[state], val);

	if (async)
		return vts_send_msg(vtsdev, VTS_MSG_TYPE_CHANGE_MODE, state, val, ktime_set(0, 0), false);

	return vts_suspend_resume(vtsdev, state, val);
}

int vts_state_get(struct vts_device *vtsdev, enum vts_state state)
{
	if (state >= VTS_STA_MAX) {
		vts_dev_err(vtsdev, "invalid state %d\n", state);
		return -EINVAL;
	}

	return atomic_read(&vtsdev->mobile_state[state]);
}

void vts_state_dump(struct vts_device *vtsdev)
{
	int i;

	for (i = 0; i < VTS_STA_MAX; i++)
		vts_dev_dbg(vtsdev, "%s:%d", vts_state_name(i), vts_state_get(vtsdev, i));
}

void vts_device_lock(struct vts_device *vtsdev)
{
	down(&vtsdev->sem);
}

int vts_device_lock_timeout(struct vts_device *vtsdev, long jiffies)
{
	return down_timeout(&vtsdev->sem, jiffies);
}

void vts_device_unlock(struct vts_device *vtsdev)
{
	up(&vtsdev->sem);
}

static u32 vts_current_module_lcmid(struct vts_panel_module *module)
{
	if (module->properties[VTS_PROPERTY_PANEL_TYPE] == VTS_TYPE_MAIN)
		return vts_get_msdd_report_lcm_id(VTS_TYPE_MAIN);
	else if (module->properties[VTS_PROPERTY_PANEL_TYPE] == VTS_TYPE_SECOND)
		return vts_get_msdd_report_lcm_id(VTS_TYPE_SECOND);
	else {
		VTE("invalid module type %d\n", module->properties[VTS_PROPERTY_PANEL_TYPE]);
		return 0xff;
	}
}

static unsigned long vts_current_module_softid(void)
{
	return lcm_software_id;
}

int vts_get_lcmid(struct vts_device *vtsdev, u32 *lcmid)
{
	if (vtsdev->module->softid_active) {
		*lcmid = vtsdev->module->lcmid[0];
	} else {
		*lcmid = vts_current_module_lcmid(vtsdev->module);
	}
	return 0;
}

int vts_get_lcmid_compatible(struct vts_device *vtsdev, u32 *lcmid, size_t *size)
{
	int i;

	for (i = 0; i < vtsdev->module->lcmid_cnt; i++) {
		if (i >= *size)
			return -EOVERFLOW;

		lcmid[i] = vtsdev->module->lcmid[i];
	}

	*size = vtsdev->module->lcmid_cnt;
	return 0;
}

#define parse_property(np, prop_name, data_type, val, err_return, err_default) do { \
		if(of_property_read_##data_type(np, prop_name, val)) { \
			if (err_return) {\
				VTE("get property "prop_name" failed!!\n"); \
				return -EINVAL; \
			} \
			\
			*val = err_default; \
			VTI("property "prop_name" not configed, set to default value:"#err_default"\n"); \
		} \
	} while (0)

#define parse_property_u32_with_default(np, prop_name, val, err_default) parse_property(np, prop_name, u32, val, false, err_default)
#define parse_property_u32(np, prop_name, val) parse_property(np, prop_name, u32, val, true, 0)
#define parse_property_string_with_default(np, prop_name, val, err_default) parse_property(np, prop_name, string, val, false, err_default)

struct vts_module_id {
	struct list_head entry;
	u32 lcmid;
	u32 softid;
};

static int vts_parse_module_id(struct vts_panel_module *module, struct device_node *np)
{
	int i, j;
	int ret = 0;
	u32 length = 0;
	u32 size = 0;
	u32 *arr_32 = NULL;
	const u32 *arr;
	struct vts_module_id *module_id;
	struct device_node *node;
#if (defined(CONFIG_ARCH_QCOM) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	struct drm_panel *panel;
#endif
	int count;

	if (!module->softid_active)
		return 0;

	count = of_count_phandle_with_args(np, "vts-lcm-module", NULL);
	if (count <= 0) {
		VTE("vts-lcm-module count invalid\n");
		return -EINVAL;
	}

	INIT_LIST_HEAD(&module->module_list);
	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "vts-lcm-module", i);
#if (defined(CONFIG_ARCH_QCOM) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
		if (module->properties[VTS_PROPERTY_TDDI]) {
			panel = of_drm_find_panel(node);
			if (!IS_ERR(panel)) {
				module->active_panel_v2 = panel;
				VTI("find vts-incell-panel success");
			}
		}
#endif
		arr = of_get_property(node, "qcom,mdss-dsi-lcmid-sequence", &length);
		if (!arr) {
			VTE("dsi_panel_parse_lcmid not found\n");
			of_node_put(node);
			continue;
		}

		if (length & 0x1) {
			VTE("syntax error for dsi_panel_parse_lcmid\n");
			of_node_put(node);
			continue;
		}

		length = length / sizeof(u32);
		size   = length * sizeof(u32);
		arr_32 = kzalloc(size, GFP_KERNEL);
		if (!arr_32) {
			VTE("kzalloc fail\n");
			of_node_put(node);
			continue;
		}

		ret = of_property_read_u32_array(node, "qcom,mdss-dsi-lcmid-sequence", arr_32, length);
		if (ret) {
			VTE("cannot read qcom,mdss-dsi-lcmid-sequence\n");
			of_node_put(node);
			kfree(arr_32);
			continue;
		}

		for (j = 0; j < length; j += 2) {
			module_id = kzalloc(sizeof(*module_id), GFP_KERNEL);
			if (!module_id) {
				VTE("kmalloc module_id fail!\n");
				continue;
			}

			module_id->lcmid  = arr_32[j];
			module_id->softid = arr_32[j + 1];
			INIT_LIST_HEAD(&module_id->entry);
			list_add_tail(&module_id->entry, &module->module_list);
		}

		of_node_put(node);
		kfree(arr_32);
	}

	return ret;
}

static int vts_parse_module_property(struct vts_panel_module *module, struct device_node *np)
{
	int ret;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 5, 0)
	int i;
	module->lcmid_cnt = of_property_count_u32_elems(np, "vts-lcmid");
	if(module->lcmid_cnt <= 0)
		VTE("get property vts-lcmid failed!ret = %d\n", module->lcmid_cnt);

	for(i = 0; i < module->lcmid_cnt; i++)
		of_property_read_u32_index(np, "vts-lcmid", i, &module->lcmid[i]);
#else
	module->lcmid_cnt = of_property_read_variable_u32_array(np, "vts-lcmid", module->lcmid, 1, ARRAY_SIZE(module->lcmid));
#endif
	VTI("lcmid_cnt:%d\n", module->lcmid_cnt);

	if (module->lcmid_cnt <= 0)
		module->softid_active = true;

	/* need to configure in dtsi file start */
	parse_property_u32(np, "vts-ic-number", &module->properties[VTS_PROPERTY_IC_NUMBER]);
	parse_property_u32(np, "vts-dimention-x", &module->properties[VTS_PROPERTY_DIMENTION_X]);
	parse_property_u32(np, "vts-dimention-y", &module->properties[VTS_PROPERTY_DIMENTION_Y]);
	parse_property_u32(np, "vts-tx-sensors", &module->properties[VTS_PROPERTY_SENSOR_TX_NUM]);
	parse_property_u32(np, "vts-rx-sensors", &module->properties[VTS_PROPERTY_SENSOR_RX_NUM]);
	/* need to configure in dtsi file end */
	parse_property_u32_with_default(np, "vts-type", &module->properties[VTS_PROPERTY_PANEL_TYPE], VTS_TYPE_MAIN);
	parse_property_u32_with_default(np, "vts-need-caliberation", &module->properties[VTS_PROPERTY_NEED_CALI], 0);
	parse_property_u32_with_default(np, "vts-incell", &module->properties[VTS_PROPERTY_TDDI], 0);
	parse_property_u32_with_default(np, "vts-virtual-proximinity", &module->properties[VTS_PROPERTY_VIRTUAL_PROXIMINITY], 0);
	parse_property_u32_with_default(np, "vts-long-press", &module->properties[VTS_PROPERTY_LONG_PRESS], 0);
	parse_property_u32_with_default(np, "vts-no-flash", &module->properties[VTS_PROPERTY_NO_FLASH], 0);
	parse_property_u32_with_default(np, "vts-policy", &module->properties[VTS_PROPERTY_POLICY], 0);
	parse_property_u32_with_default(np, "vts-module-vendor", &module->properties[VTS_PROPERTY_VENDOR], 0);
	parse_property_u32_with_default(np, "vts-game-mode", &module->properties[VTS_PROPERTY_GAME_MODE], 0);
	parse_property_u32_with_default(np, "vts-band", &module->properties[VTS_PROPERTY_BAND], 0);
	parse_property_u32_with_default(np, "vts-rotation", &module->properties[VTS_PROPERTY_ROTATION], 1);
	parse_property_u32_with_default(np, "vts-charge", &module->properties[VTS_PROPERTY_CHARGE], 1);
	parse_property_u32_with_default(np, "vts-gesture", &module->properties[VTS_PROPERTY_GESTURE], 0);
	parse_property_u32_with_default(np, "vts-virtual-key", &module->properties[VTS_PROPERTY_VIRTUAL_KEY], 0);
	parse_property_u32_with_default(np, "vts-landscape-gamemode", &module->properties[VTS_PROPERTY_LANDSCAPE_GAMEMODE], 0);
	parse_property_u32_with_default(np, "vts-report-timestamp", &module->properties[VTS_PROPERTY_REPORT_TIMESTAMP], 0);
	parse_property_u32_with_default(np, "vts-virtual-gamekey", &module->properties[VTS_PROPERYT_VIRTUAL_GAMEKEY], 0);
	parse_property_u32_with_default(np, "vts-screen-clock", &module->properties[VTS_PROPERTY_SCREEN_CLOCK], 0);
	parse_property_u32_with_default(np, "vts-broken-disable", &module->properties[VTS_PROPERTY_BROKEN_DISABLE], 0);
	parse_property_u32_with_default(np, "vts-game-high_rate", &module->properties[VTS_PROPERTY_GAME_HIGH_RATE], 0);
	parse_property_u32_with_default(np, "vts-game-idle-time", &module->properties[VTS_PROPERTY_GAME_IDLE_TIME], 0);
	parse_property_u32_with_default(np, "vts-finger-mode", &module->properties[VTS_PROPERTY_FINGER_MODE], 0);
	parse_property_u32_with_default(np, "vts-fp-feedback", &module->properties[VTS_PROPERTY_FD_FEED_BACK], 0);
	parse_property_u32_with_default(np, "vts-display-x", &module->properties[VTS_PROPERTY_DISPLAY_X], 0);
	parse_property_u32_with_default(np, "vts-display-y", &module->properties[VTS_PROPERTY_DISPLAY_Y], 0);
	parse_property_u32_with_default(np, "vts-resolution-adjust", &module->properties[VTS_PROPERTY_RESOLUTION_ADJUST], 0);
	parse_property_u32_with_default(np, "vts-input-method", &module->properties[VTS_PROPERTY_INPUT_METHOD], 0);
	parse_property_u32_with_default(np, "vts-channel_comp_collect", &module->properties[VTS_PROPERTY_TP_CHANNEL_COMP], 0);
	parse_property_u32_with_default(np, "vts-bus-state-set", &module->properties[VTS_PROPERTY_SET_BUS_STATE], 0);
	parse_property_u32_with_default(np, "vts-get-i2c-event", &module->properties[VTS_PROPERTY_I2C_EVENT], 0);
	parse_property_u32_with_default(np, "vts-boost", &module->properties[VTS_PROPERTY_BOOST], 0);
	parse_property_u32_with_default(np, "vts-finger-center", &module->properties[VTS_PROPERTY_FINGER_CENTER], 0);
	parse_property_u32_with_default(np, "vts-finger-center-x", &module->properties[VTS_PROPERTY_FINGER_CENTER_X], 0);
	parse_property_u32_with_default(np, "vts-finger-center-y", &module->properties[VTS_PROPERTY_FINGER_CENTER_Y], 0);
	parse_property_u32_with_default(np, "vts-calibration-twice", &module->properties[VTS_PROPERTY_CALIBRATION_TWICE], 0);
	parse_property_u32_with_default(np, "vts-clock-report-abs", &module->properties[VTS_PROPERTY_SCREEN_CLOCK_REPORT_ABS], 0);
	parse_property_u32_with_default(np, "vts-virtual-lcmid", &module->properties[VTS_PROPERTY_VIRTUAL_LCMID], 0);
	parse_property_u32_with_default(np, "vts-lcmid-state-share", &module->properties[VTS_PROPERTY_SHARE_LCMID], 0);
	parse_property_u32_with_default(np, "vts-def-lcmid", &module->properties[VTS_PROPERTY_DEF_LCMID], 0xFF);
	parse_property_string_with_default(np, "vts-policy-name", &module->policy_name, "default");
	parse_property_u32_with_default(np, "vts-sleep-time", &module->properties[VTS_PROPERTY_SLEEP_TIME], 0);
	
	
	if (module->properties[VTS_PROPERTY_CALIBRATION_TWICE] == 1) {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 5, 0)
		module->highestCali_cnt = of_property_count_u32_elems(np, "vts-highest-calibration");
		if(module->highestCali_cnt <= 0) {
			VTE("get property vts-highest-calibration!ret = %d\n", module->highestCali_cnt);
			return -EINVAL;
		}
		for(i = 0; i < module->highestCali_cnt; i++)
			of_property_read_u32_index(np, "vts-highest-calibration", i, &module->highest_calibration[i]);

		module->otherCali_cnt = of_property_count_u32_elems(np, "vts-other-calibration");
		if(module->otherCali_cnt <= 0) {
			VTE("get property vts-other-calibration!ret = %d\n", module->otherCali_cnt);
			return -EINVAL;
		}
		for(i = 0; i < module->otherCali_cnt; i++)
			of_property_read_u32_index(np, "vts-other-calibration", i, &module->other_calibration[i]);		

#else
		module->highestCali_cnt = of_property_read_variable_u32_array(np, "vts-highest-calibration", module->highest_calibration, 1, ARRAY_SIZE(module->highest_calibration));
		module->otherCali_cnt = of_property_read_variable_u32_array(np, "vts-other-calibration", module->other_calibration, 1, ARRAY_SIZE(module->other_calibration));
#endif
		VTI("highestCali_cnt:%d\n", module->highestCali_cnt);
		VTI("otherCali_cnt:%d\n", module->otherCali_cnt);
	}

	ret = vts_parse_module_id(module, np);
	if (ret) {
		VTI("parse module id fail\n");
		return -EINVAL;
	}

	return 0;
}

#if (defined(CONFIG_ARCH_QCOM) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
int vts_parse_incell_panel(struct vts_device *vtsdev, struct device_node *np)
{
	int i;
	int count;
	struct device_node *node;
	struct drm_panel *panel;

	vtsdev->active_panel_v2 = NULL;

	VTI("vts_parse_incell_panel in !!!");
	count = of_count_phandle_with_args(np, "vts-incell-panel", NULL);
	if (count <= 0){
		return 0;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "vts-incell-panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			vtsdev->active_panel_v2 = panel;
			VTI("find vts-incell-panel success");
			return 0;
		}
	}
	VTI("find vts-incell-panel FAIL");

	return -ENODEV;
}
#endif

int vts_parse_dt_property(struct vts_device *vtsdev, struct device_node *np)
{
	parse_property_u32(np, "vts-ic-number", &vtsdev->ic_number);
	parse_property_u32_with_default(np, "vts-type", &vtsdev->type, VTS_TYPE_MAIN);
	parse_property_string_with_default(np, "sensor-test-key", &vtsdev->activity_path[VTS_TEST_APK_TYPE_SENSOR_TEST], VTS_FACTORY_KEY_DEFAULT);
	parse_property_string_with_default(np, "lcm-noise-test-key", &vtsdev->activity_path[VTS_TEST_APK_TYPE_LCM_NOISE_TEST], VTS_FACTORY_KEY_DEFAULT);
	parse_property_string_with_default(np, "bsp-lcm-noise-test-key", &vtsdev->activity_path[VTS_TEST_APK_TYPE_BSP_LCM_NOISE_TEST], VTS_FACTORY_KEY_DEFAULT);
	parse_property_string_with_default(np, "rawdata-test-key", &vtsdev->activity_path[VTS_TEST_APK_TYPE_RAWDATA_TEST], VTS_FACTORY_KEY_DEFAULT);
	parse_property_string_with_default(np, "rf-noise-test-key", &vtsdev->activity_path[VTS_TEST_APK_TYPE_RF_TEST], VTS_FACTORY_KEY_DEFAULT);
	#if (defined(CONFIG_ARCH_QCOM) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	vts_parse_incell_panel(vtsdev,np);
	#endif
	return 0;
}

static void vts_module_dump(struct vts_panel_module *module)
{
	int i;
	u8 lcmid_buf[128];
	u8 moduleid_buf[256];
	struct vts_module_id *module_id;

	memset(lcmid_buf, 0, sizeof(lcmid_buf));
	memset(moduleid_buf, 0, sizeof(moduleid_buf));
	for (i = 0; i < module->lcmid_cnt; i++)
		snprintf(lcmid_buf + strlen(lcmid_buf), sizeof(lcmid_buf) - strlen(lcmid_buf), "%d ", module->lcmid[i]);

	if (module->softid_active) {
		list_for_each_entry(module_id, &module->module_list, entry) {
			snprintf(moduleid_buf + strlen(moduleid_buf), sizeof(moduleid_buf) - strlen(moduleid_buf), "<%d,0x%x> ", module_id->lcmid, module_id->softid);
		}
	}

	VTI("ic number:%d\n", module->properties[VTS_PROPERTY_IC_NUMBER]);
	VTI("type:%d\n", module->properties[VTS_PROPERTY_PANEL_TYPE]);
	VTI("lcmid:%s\n", lcmid_buf);
	VTI("lcmid-softid:%s\n", moduleid_buf);
	VTI("no_flash:%d\n", module->properties[VTS_PROPERTY_NO_FLASH]);
	VTI("incell:%d\n", module->properties[VTS_PROPERTY_TDDI]);
	VTI("dimension_x:%d\n", module->properties[VTS_PROPERTY_DIMENTION_X]);
	VTI("dimension_y:%d\n", module->properties[VTS_PROPERTY_DIMENTION_Y]);
	VTI("display_x:%d\n", module->properties[VTS_PROPERTY_DISPLAY_X]);
	VTI("display_y:%d\n", module->properties[VTS_PROPERTY_DISPLAY_Y]);
	VTI("tx_sensors:%d\n", module->properties[VTS_PROPERTY_SENSOR_TX_NUM]);
	VTI("rx_sensors:%d\n", module->properties[VTS_PROPERTY_SENSOR_RX_NUM]);
}

static bool vts_module_contain_lcmid(struct vts_panel_module *module, u32 lcmid)
{
	int i;

	for (i = 0; i < module->lcmid_cnt; i++)
		if (module->lcmid[i] == lcmid)
			return true;
	return false;
}

static bool vts_module_contain_softid(struct vts_panel_module *module, u32 softid)
{
	bool match = false;
	u32 virtual_lcmid = 0;
	u32 share_state = 0;
	u32 def_lcmid = 0;
	struct vts_module_id *module_id;

	virtual_lcmid = module->properties[VTS_PROPERTY_VIRTUAL_LCMID];
	share_state = module->properties[VTS_PROPERTY_SHARE_LCMID];
	def_lcmid = module->properties[VTS_PROPERTY_DEF_LCMID];
	list_for_each_entry(module_id, &module->module_list, entry) {
		if (module_id->softid == softid) {
			if (share_state == 0)
				module->lcmid[0] = module_id->lcmid;
			else
				module->lcmid[0] = virtual_lcmid;
			module->lcmid_cnt = 1;
			match = true;
			break;
		}
	}

	if (!match && (def_lcmid != 0xFF)) {
		module->lcmid[0] = def_lcmid;
		module->lcmid_cnt = 1;
		match = true;
		VTI("def_lcmid:%d\n", def_lcmid);
	}

	return match;
}


static LIST_HEAD(vts_panel_modules);
static DEFINE_MUTEX(modules_mutex);

static void vts_parse_module_in_dts(void)
{
	static bool parsed = false;
	struct device_node *node = NULL;
	struct vts_panel_module *module;
	int index = 0;

	if (parsed)
		return ;

	mutex_lock(&modules_mutex);

	while((node = of_find_compatible_node(node ,NULL, "vivo,touch-panel-module")) != NULL) {
		module = kzalloc(sizeof(*module), GFP_KERNEL);
		if (!module) {
			VTE("no mmeory!\n");
			continue;
		}

		if (vts_parse_module_property(module, node)) {
			VTE("parse module error!\n");
			kfree(module);
			continue;
		}

		INIT_LIST_HEAD(&module->list);
		list_add_tail(&module->list, &vts_panel_modules);
	}

	if(list_empty(&vts_panel_modules))
		VTE("error!!!, no module in DTS\n");

	list_for_each_entry(module, &vts_panel_modules, list) {
		VTI("*****module %d information start*****\n", ++index);
		vts_module_dump(module);
		VTI("*****module %d information end*****\n", index);
	}

	mutex_unlock(&modules_mutex);
	parsed = true;
}

static int vts_module_attach(struct vts_device *vtsdev, int ic_number, enum vts_type type)
{
	struct vts_panel_module *module;

	vts_parse_module_in_dts();

	mutex_lock(&modules_mutex);
	list_for_each_entry(module, &vts_panel_modules, list) {
		if (module->softid_active) {
			if (!vts_module_contain_softid(module, vts_current_module_softid()))
				continue;
		} else {
			if (!vts_module_contain_lcmid(module, vts_current_module_lcmid(module)))
				continue;
		}

		if (module->properties[VTS_PROPERTY_IC_NUMBER] != ic_number || module->properties[VTS_PROPERTY_PANEL_TYPE] != type)
			continue;

		vts_dev_info(vtsdev, "module attched!, icnumber:%d, lcmid:%d\n", module->properties[VTS_PROPERTY_IC_NUMBER], vts_current_module_lcmid(module));
		vtsdev->module = module;
		mutex_unlock(&modules_mutex);
		return 0;
	}
	mutex_unlock(&modules_mutex);
	vts_dev_err(vtsdev, "not found touch panel module, icnumber:%d, type:%d\n", ic_number, type);
	return -ENODEV;
}

static void vts_module_deattach(struct vts_device *vtsdev)
{
	vtsdev->module = NULL;
	vts_dev_info(vtsdev, "module deattched!\n");
}

static int __vts_firmware_update(struct vts_device *vtsdev, enum vts_fw_type type)
{
	int ret;
	int fw_size = 0;
	unsigned char *fw_data = NULL;
	struct firmware *firmware;
	u64 version;
	//fw_data = __vts_fw_data_get(vtsdev, type, &fw_size, &firmware);
	firmware = vtsdev->firmwares[type].firmware;
	if (IS_ERR_OR_NULL(firmware)) {
		vts_dev_err(vtsdev, "get fw fail\n");
		return -EIO;
	}
	fw_data = (u8*)vtsdev->firmwares[type].firmware->data;
	fw_size = vtsdev->firmwares[type].firmware->size;
	if (IS_ERR_OR_NULL(fw_data) || !fw_size) {
		vts_dev_err(vtsdev, "get fw fail\n");
		return -EIO;
	}

	ret = vts_call_ic_ops(vtsdev, update_firmware ,firmware);
	if (ret) {
		vts_dev_err(vtsdev, "update firmwarefailed ret %d\n", ret);
		return ret;
	}

	vts_firmware_version_get(vtsdev,&version);

	vts_dev_info(vtsdev, "update firmware suceess!\n");
	return 0;
}

static int vts_firmware_update(struct vts_device *vtsdev, enum vts_fw_type type)
{
	int ret;
	enum vts_run_mode mode = vts_get_run_mode(vtsdev);

	if (vts_change_to_mode(vtsdev, mode, VTS_ST_NORMAL, VTS_STA_DEFAUTL)) {
		vts_dev_err(vtsdev, "change to normal mode error!\n");
		return -EIO;
	}

	vts_report_release(vtsdev);
	ret = __vts_firmware_update(vtsdev, type);
	vts_report_release(vtsdev);

	if (vts_change_to_mode(vtsdev, VTS_ST_NORMAL, mode, VTS_STA_DEFAUTL))
		vts_dev_err(vtsdev, "change to mode %s error!\n", vts_mode_name(mode));

	return ret;
}

static int vts_send_msg(struct vts_device *vtsdev, enum vts_msg_type type, int arg1, int arg2, ktime_t arg3, bool add_first)
{
	struct vts_message *msg;
	unsigned long flags = 0;

	msg = kmem_cache_zalloc(vtsdev->msg_km, GFP_ATOMIC);
	if (!msg) {
		vts_dev_err(vtsdev, "alloc message buffer failed");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&msg->list);
	msg->index = atomic_inc_return(&nr_messages);
	msg->type = type;
	msg->arg1 = arg1;
	msg->arg2 = arg2;
	msg->arg3 = arg3;
	VTD("msg type:%d, msg index:%ld\n", msg->type, msg->index);

	if (in_interrupt())
		spin_lock(&vtsdev->msg_lock);
	else
		spin_lock_irqsave(&vtsdev->msg_lock, flags);

	if (unlikely(add_first))
		list_add(&msg->list, &vtsdev->messages);
	else
		list_add_tail(&msg->list, &vtsdev->messages);

	if (in_interrupt())
		spin_unlock(&vtsdev->msg_lock);
	else
		spin_unlock_irqrestore(&vtsdev->msg_lock, flags);

	up(&vtsdev->msg_sem);
	return 0;
}


static int vts_recv_msg(struct vts_device *vtsdev, enum vts_msg_type *type, int *arg1, int *arg2, ktime_t *arg3)
{
	struct vts_message *msg = NULL;
	unsigned long flags = 0;

	down_interruptible(&vtsdev->msg_sem);
	spin_lock_irqsave(&vtsdev->msg_lock, flags);
	if (list_empty(&vtsdev->messages)) {
		spin_unlock_irqrestore(&vtsdev->msg_lock, flags);
		return -EAGAIN;
	}

	msg = list_first_entry(&vtsdev->messages, struct vts_message, list);
	*type = msg->type;
	*arg1 = msg->arg1;
	*arg2 = msg->arg2;
	*arg3 = msg->arg3;
	list_del(&msg->list);
	spin_unlock_irqrestore(&vtsdev->msg_lock, flags);
	VTD("msg type:%d, msg index:%ld\n", msg->type, msg->index);
	kmem_cache_free(vtsdev->msg_km, msg);
	return 0;
}

static int vts_interrupt_handler(struct vts_device *vtsdev, int arg1, int arg2, ktime_t arg3)
{
	if (vts_get_run_mode(vtsdev) != VTS_ST_SLEEP) {
		vtsdev->irq_thread_fn(vtsdev->irq, vtsdev->irqdata, arg3);
	}
	__pm_relax(vtsdev->irq_wakelock);
	enable_irq(vtsdev->irq);
	return 0;
}

static int vts_state_change_handler(struct vts_device *vtsdev, int arg1, int arg2, ktime_t arg3)
{
	return vts_suspend_resume(vtsdev, (enum vts_state)arg1, arg2);
}

static int vts_exception_handler(struct vts_device *vtsdev, int arg1, int arg2, ktime_t arg3)
{
	vts_dev_err(vtsdev, "exception:%d, val:%d\n\n", arg1, arg2);

	if (vtsdev->exception_handler)
		return vtsdev->exception_handler(vtsdev, (enum vts_ic_exception)arg1, arg2);

	return 0;
}

static int vts_update_handler(struct vts_device *vtsdev, int arg1, int arg2, ktime_t arg3)
{
	if (NULL == vtsdev->firmwares[arg1].path) {
#ifdef VTS_TEMP_ENABLE
		goto update;
#endif
		vts_dev_err(vtsdev, "undefined firmware path\n");
		return -EINVAL;
	}

#ifdef VTS_TEMP_ENABLE
update:
#endif
	return vts_firmware_update(vtsdev, arg1);
}

int vts_property_get(struct vts_device *vtsdev, enum vts_property prop, u32 *val)
{
	if (unlikely(!vtsdev->module)) {
		vts_dev_err(vtsdev, "didn't attach module!!\n");
		return -EIO;
	}

	if (unlikely(prop >= ARRAY_SIZE(vtsdev->module->properties))) {
		vts_dev_err(vtsdev, "invalid prop index %d\n", prop);
		return -EINVAL;
	}

	*val = vtsdev->module->properties[prop];
	return 0;
}

int vts_state_set(struct vts_device *vtsdev, enum vts_state id, int val)
{
	return __vts_state_set(vtsdev, id, val, true);
}

int vts_state_set_sync(struct vts_device *vtsdev, enum vts_state id, int val)
{
	return __vts_state_set(vtsdev, id, val, false);
}

int vts_state_broadcast(struct vts_device *vtsdev, enum vts_state id, int val)
{
	struct vts_device *tmp;
	int ret = 0;

	mutex_lock(&vts_lock);
	list_for_each_entry(tmp, &vts_devices, entry) {
		if (tmp != vtsdev) {
			vts_send_msg(tmp, VTS_MSG_TYPE_CHANGE_MODE, VTS_STA_OTHER_DEVICE_STATE_CHANGED, id, ktime_set(0, 0), false);
		}
	}
	mutex_unlock(&vts_lock);
	return ret;
}

bool vts_state_all_equals(enum vts_state state, int val)
{
	struct vts_device *tmp;

	mutex_lock(&vts_lock);
	list_for_each_entry(tmp, &vts_devices, entry) {
		if (vts_state_get(tmp, state) != val) {
			mutex_unlock(&vts_lock);
			return false;
		}
	}
	mutex_unlock(&vts_lock);
	return true;
}

int vts_device_count_get(void)
{
	int count = 0;
	struct vts_device *tmp;

	mutex_lock(&vts_lock);
	list_for_each_entry(tmp, &vts_devices, entry) {
		count++;
	}
	mutex_unlock(&vts_lock);

	return count;
}

static int vts_interrupt_msg_send(struct vts_device *vtsdev)
{
	return vts_send_msg(vtsdev, VTS_MSG_TYPE_IRQ_EVENT, 0, 0, ktime_get(), false);
}

int vts_notify_ic_exception(struct vts_device *vtsdev, enum vts_ic_exception exception, int arg)
{
	if(vtsdev == NULL) {
		VTI("vtsdev is null!");
		return 0;
	}
	vtsdev->exceptions[exception]++;
	return vts_send_msg(vtsdev, VTS_MSG_TYPE_EXCEPTION, exception, arg, ktime_set(0, 0), false);
}

typedef int (*vts_handler)(struct vts_device *vtsdev, int arg1, int arg2, ktime_t arg3);

static const vts_handler vts_handlers[VTS_MSG_TYPE_COUNT] = {
	[VTS_MSG_TYPE_IRQ_EVENT] = vts_interrupt_handler,
	[VTS_MSG_TYPE_CHANGE_MODE] = vts_state_change_handler,
	[VTS_MSG_TYPE_EXCEPTION] = vts_exception_handler,
	[VTS_MSG_TYPE_UPDATE] = vts_update_handler,
};

int vts_reset(struct vts_device *vtsdev)
{
	if ((vts_get_run_mode(vtsdev) != VTS_ST_NORMAL) && (vts_get_run_mode(vtsdev) != VTS_ST_GESTURE)) {
		vts_dev_err(vtsdev, "not in normal/gesture mode!\n");
		return -EBUSY;
	}

	vts_call_ic_ops(vtsdev, reset);
	vts_send_msg(vtsdev, VTS_MSG_TYPE_MODE_CHANGED_STATE_SYNC, VTS_STA_RESET, 0, ktime_set(0, 0), false);

	return 0;
}

#ifdef VTS_TEMP_ENABLE
static void boot_fw_update_work(struct work_struct *work)
{
	struct delayed_work *p_delayed_work = 
		container_of(work, struct delayed_work, work);
	struct vts_device *vtsdev = 
		container_of(p_delayed_work, struct vts_device, fw_update_work);
	vts_dev_info(vtsdev, "send fw update msg\n");
	vts_send_msg(vtsdev, VTS_MSG_TYPE_UPDATE, VTS_FW_TYPE_FW, 0, ktime_set(0, 0), false);
	vts_send_msg(vtsdev, VTS_MSG_TYPE_UPDATE, VTS_FW_TYPE_CONFIG, 0, ktime_set(0, 0), false);
}
#endif

static int vts_kthread(void *dev)
{
	struct vts_device *vtsdev = (struct vts_device *)dev;
	enum vts_msg_type type;
	int ret, arg1, arg2;
	ktime_t arg3;
	u32 sleep_time;

	static struct sched_param para = {
		.sched_priority = 40,
	};

	sched_setscheduler(current, SCHED_FIFO, &para);

	vts_property_get(vtsdev, VTS_PROPERTY_SLEEP_TIME, &sleep_time);
	if (sleep_time) {
		VTI("sleep %d s in vts kthread!!!", sleep_time);
		msleep(sleep_time * 1000);
	}
	vts_call_ic_ops_sync(vtsdev, init);
	while (likely(!kthread_should_stop())) {

		ret = vts_recv_msg(vtsdev, &type, &arg1, &arg2, &arg3);
		if (ret) {
			vts_dev_err(vtsdev, "recv msg error!, ret = %d\n", ret);
			continue;
		}

		if (vts_device_lock_timeout(vtsdev, HZ/10)) {
			vts_send_msg(vtsdev, type, arg1, arg2, arg3, true);
			vts_dev_err(vtsdev, "device is locked!!, send message %d again!!\n", type);
			continue;
		}

		__pm_stay_awake(vtsdev->k_wakelock);
		ret = vts_handlers[type](vtsdev, arg1, arg2, arg3);
		__pm_relax(vtsdev->k_wakelock);
		vts_device_unlock(vtsdev);
		if (ret)
			vts_dev_err(vtsdev, "process msg %d error!\n, ret = %d\n", type, ret);
	}
	vts_call_ic_ops_sync(vtsdev, exit);
	vts_wakelock_unregister(vtsdev->k_wakelock);
	vtsdev->k_wakelock = NULL;

	return 0;
}

static irqreturn_t vts_interrupt_func(int irq, void *data)
{
	struct vts_device *vtsdev = (struct vts_device *)data;

	disable_irq_nosync(irq);
	__pm_stay_awake(vtsdev->irq_wakelock);
	vts_interrupt_msg_send(vtsdev);
	return IRQ_HANDLED;
}

struct wakeup_source *vts_wakelock_register(struct vts_device *vtsdev, const char *wakelock_name)
{
	struct wakeup_source *wakelock;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0) || LINUX_VERSION_CODE == KERNEL_VERSION(4, 19, 110))
	wakelock = wakeup_source_register(NULL, wakelock_name);
	if (!wakelock) {
		vts_dev_err(vtsdev, "fail register wakelock");
		return NULL;
	}
#else
	wakelock = (struct wakeup_source *)kzalloc(sizeof(struct wakeup_source), GFP_KERNEL);
	if (!wakelock) {
		vts_dev_err(vtsdev, "fail kmalloc wakelock");
		return NULL;
	}
	wakeup_source_init(wakelock, wakelock_name);
#endif

	return wakelock;
}

void vts_wakelock_unregister(struct wakeup_source *wakelock)
{
	if (!wakelock)
		return;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0) || LINUX_VERSION_CODE == KERNEL_VERSION(4, 19, 110))
	wakeup_source_unregister(wakelock);
#else
	wakeup_source_trash(wakelock);
	kfree(wakelock);
#endif
}

int vts_wakelock_init(struct vts_device *vtsdev)
{
	snprintf(vtsdev->wakelock_name, sizeof(vtsdev->wakelock_name), "vts_kthread_%d", vtsdev->type);

	vtsdev->k_wakelock = vts_wakelock_register(vtsdev, vtsdev->wakelock_name);
	if (!vtsdev->k_wakelock) {
		vts_dev_err(vtsdev, "kthread_wakelock init fail\n");
		return -ENOMEM;
	}

	vtsdev->irq_wakelock = vts_wakelock_register(vtsdev, vts_name(vtsdev));
	if (!vtsdev->irq_wakelock) {
		vts_dev_err(vtsdev, "irq_wakelock init fail!\n");
		return -ENOMEM;
	}

	return 0;
}

void vts_wakelock_deinit(struct vts_device *vtsdev)
{
	vts_wakelock_unregister(vtsdev->irq_wakelock);
	vts_wakelock_unregister(vtsdev->k_wakelock);
	vtsdev->irq_wakelock = NULL;
	vtsdev->k_wakelock = NULL;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
struct vts_sem_waiter {
	struct list_head list;
	struct task_struct *task;
	bool up;
};

static noinline void __sched _vts_kthread_stop(struct semaphore *sem, struct task_struct *task)
{
	struct vts_sem_waiter *waiter;
	unsigned long flags;

	raw_spin_lock_irqsave(&sem->lock, flags);
	if (likely(list_empty(&sem->wait_list))) {
		sem->count++;
	} else {
		waiter = list_first_entry(&sem->wait_list, struct vts_sem_waiter, list);
		list_del(&waiter->list);
		waiter->up = true;
	}
	raw_spin_unlock_irqrestore(&sem->lock, flags);

	if (!IS_ERR(task))
		kthread_stop(task);
}
#endif

void vts_kthread_stop(struct semaphore *sem, struct task_struct *task)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	_vts_kthread_stop(sem, task);
#else
	if (!IS_ERR(task))
		kthread_stop(task);
#endif
}


int vts_register_driver(struct vts_device *vtsdev)
{
	int ret;

	if (vts_module_attach(vtsdev, vtsdev->ic_number, vtsdev->type)) {
		vts_dev_err(vtsdev, "no matched module, ic number = %d, type = %d\n", vtsdev->ic_number, vtsdev->type);
		return -ENODEV;
	}

	ret = vts_wakelock_init(vtsdev);
	if (ret) {
		vts_dev_err(vtsdev, "vts wakelock init fail\n");
		goto errcode1;
	}

	vtsdev->t = kthread_run(vts_kthread, vtsdev, "vtsthread%d", vtsdev->type);
	if (IS_ERR(vtsdev->t)) {
		vts_dev_err(vtsdev, "create thread error!");
		ret = PTR_ERR(vtsdev->t);
		goto errcode2;
	}

	ret = vts_report_init(vtsdev);
	if (ret) {
		vts_dev_err(vtsdev, "report init error!, ret = %d", ret);
		goto errcode3;
	}

	ret = vts_classdev_register(NULL, vtsdev);
	if (ret) {
		vts_dev_err(vtsdev, "proxminity init error!, ret = %d", ret);
		goto errcode4;
	}

	ret = vts_incell_init(vtsdev);
	if (ret) {
		vts_dev_err(vtsdev, "proxminity init error!, ret = %d", ret);
		goto errcode5;
	}

	ret = vts_node_sysfs_add(vtsdev);
	if (ret) {
		vts_dev_err(vtsdev, "proxminity init error!, ret = %d", ret);
		goto errcode6;
	}

#ifdef VTS_TEMP_ENABLE
	schedule_delayed_work(&vtsdev->fw_update_work, 5 * HZ);
#endif

	vts_long_press_timer_init(vtsdev);

	mutex_lock(&vts_lock);
	list_add_tail(&vtsdev->entry, &vts_devices);
	mutex_unlock(&vts_lock);
	vts_dev_info(vtsdev, "register vtsdev success");
	return 0;

errcode6:
	vts_incell_deinit(vtsdev);
errcode5:
	vts_classdev_unregister(vtsdev);
errcode4:
	vts_report_deinit(vtsdev);
errcode3:
	vts_kthread_stop(&vtsdev->msg_sem, vtsdev->t);
errcode2:
	vts_wakelock_deinit(vtsdev);
errcode1:
	vts_module_deattach(vtsdev);


	return ret;
}

int vts_unregister_driver(struct vts_device *vtsdev)
{
	mutex_lock(&vts_lock);
	list_del(&vtsdev->entry);
	mutex_unlock(&vts_lock);
	vts_node_sysfs_remove(vtsdev);
	vts_incell_deinit(vtsdev);
	vts_classdev_unregister(vtsdev);
	vts_report_deinit(vtsdev);
	vts_module_deattach(vtsdev);
	vts_wakelock_deinit(vtsdev);
	vts_long_press_timer_deinit(vtsdev);
	vts_kthread_stop(&vtsdev->msg_sem, vtsdev->t);

	return 0;
}

int vts_firmware_version_get(struct vts_device *vtsdev, u64 *version)
{
	int ret;
	enum vts_run_mode mode = vts_get_run_mode(vtsdev);

	if (vts_change_to_mode(vtsdev, mode, VTS_ST_NORMAL, VTS_STA_DEFAUTL)) {
		vts_dev_err(vtsdev, "change to normal mode error!\n");
		return -EIO;
	}

	ret = vts_call_ic_ops(vtsdev, get_fw_version, version);

	if (vts_change_to_mode(vtsdev, VTS_ST_NORMAL, mode, VTS_STA_DEFAUTL))
		vts_dev_err(vtsdev, "change to mode %s error!\n", vts_mode_name(mode));

	return ret;
}

int vts_touch_ic_mode_get(struct vts_device *vtsdev)
{
	int ret;
	ret = vts_call_ic_ops(vtsdev, get_ic_mode);
	return ret;
}

int vts_get_calibration_status(struct vts_device *vtsdev)
{
	int ret;
	ret = vts_call_ic_ops(vtsdev, get_calibration_status);
	return ret;
}

#ifdef VTS_TEMP_ENABLE
#include "vts_fw_collection.h"
#endif
static u8 *__vts_fw_data_get(struct vts_device *vtsdev, enum vts_fw_type type, int *size, struct firmware **firmware)
{
	int ret;
#ifdef VTS_TEMP_ENABLE
	u32 lcm_id;
	int i;
#endif

	if (type >= VTS_FW_TYPE_COUNT) {
		vts_dev_err(vtsdev, "invalid args\n");
		return ERR_PTR(-EINVAL);
	}

	if (strlen(vtsdev->firmwares[type].path) == 0) {
#ifdef VTS_TEMP_ENABLE
		goto get_fw_from_array;
#endif
		vts_dev_err(vtsdev, "no data for type %d!!\n", type);
		return NULL;
	}

	if (!atomic_read(&vtsdev->firmware_cache))
		vts_fw_data_put(vtsdev, type);

	if (vtsdev->firmwares[type].firmware) {
		if (firmware)
			*firmware = vtsdev->firmwares[type].firmware;
		*size = (int)vtsdev->firmwares[type].firmware->size;
		return (u8 *)vtsdev->firmwares[type].firmware->data;
	}

	ret = request_firmware((const struct firmware **)&vtsdev->firmwares[type].firmware, vtsdev->firmwares[type].path, vtsdev->dev);
	if (ret) {
#ifdef VTS_TEMP_ENABLE
		goto get_fw_from_array;
#endif
		vts_dev_err(vtsdev, "request firmware %s failed!\n", vtsdev->firmwares[type].path);
		return ERR_PTR(ret);
	}

	if (firmware)
		*firmware = vtsdev->firmwares[type].firmware;
	*size = (int)vtsdev->firmwares[type].firmware->size;
	return (u8 *)vtsdev->firmwares[type].firmware->data;

#ifdef VTS_TEMP_ENABLE
get_fw_from_array:
	if (NULL != vtsdev->firmwares[type].firmware) {
		if (firmware)
			*firmware = vtsdev->firmwares[type].firmware;
		*size = (int)vtsdev->firmwares[type].firmware->size;
		return (u8 *)vtsdev->firmwares[type].firmware->data;
	}
	vts_get_lcmid(vtsdev, &lcm_id);
	vts_dev_err(vtsdev, "Get lcm_id[%d]\n", lcm_id);
	for (i = 0; i < (sizeof(vts_fw_array)/ sizeof(vts_fw_array[0])); i++) {
		if (vts_fw_array[i].lcm_id == lcm_id &&
					vts_fw_array[i].fw_type == type) {
			VTI("Get fw data, lcm_id[%d] fw_type[%d]", lcm_id, type);
			vtsdev->firmwares[type].firmware = kzalloc(sizeof(struct firmware), GFP_KERNEL);
			if (NULL == vtsdev->firmwares[type].firmware) {
				vts_dev_err(vtsdev, "alloc mem for firmware fail!\n");
				ret = -EINVAL;
				return ERR_PTR(ret);
			}
			vtsdev->firmwares[type].firmware->data = (u8 *)vts_fw_array[i].fw_data;
			vtsdev->firmwares[type].firmware->size = (size_t)vts_fw_array[i].fw_size;
			if (firmware) {
				*firmware = vtsdev->firmwares[type].firmware;
			}
			*size = vts_fw_array[i].fw_size;
			return (u8 *)vts_fw_array[i].fw_data;
		}
	}
	return NULL;
#endif
}

u8 *vts_fw_data_get(struct vts_device *vtsdev, enum vts_fw_type type, int *size)
{
	return __vts_fw_data_get(vtsdev, type, size, NULL);
}

void vts_fw_data_put(struct vts_device *vtsdev, enum vts_fw_type type)
{
	if (type >= VTS_FW_TYPE_COUNT) {
		vts_dev_err(vtsdev, "invalid args\n");
		return ;
	}

	if (vtsdev->firmwares[type].firmware) {
		release_firmware(vtsdev->firmwares[type].firmware);
		vtsdev->firmwares[type].firmware = NULL;
	}
}

int vts_fw_path_set(struct vts_device *vtsdev, enum vts_fw_type type, const char *path, bool update)
{
	int fw_size;
	if (type >= VTS_FW_TYPE_COUNT) {
		vts_dev_err(vtsdev, "invalid args\n");
		return -EINVAL;
	}

	vts_dev_info(vtsdev, "type:%d, path:%s, update:%d\n", type, path, update);
	memset(vtsdev->firmwares[type].path, 0, sizeof(vtsdev->firmwares[type].path));
	strncpy(vtsdev->firmwares[type].path, path, sizeof(vtsdev->firmwares[type].path));

	__vts_fw_data_get(vtsdev, type, &fw_size, NULL);
	if (fw_size == 0) {
		vts_dev_err(vtsdev, "request firmware %s failed!\n", vtsdev->firmwares[type].path);
	}

	if (update)
		vts_send_msg(vtsdev, VTS_MSG_TYPE_UPDATE, type, 0, ktime_set(0, 0), false);

	return 0;
}

int vts_fw_path_get(struct vts_device *vtsdev, enum vts_fw_type type, char *path, size_t size)
{
	if (type >= VTS_FW_TYPE_COUNT) {
		vts_dev_err(vtsdev, "invalid args\n");
		return -EINVAL;
	}

	strncpy(path, vtsdev->firmwares[type].path, size);
	return 0;
}

struct vts_frame *vts_frame_data_get(struct vts_device *vtsdev, enum vts_frame_type type)
{
	int size;
	struct vts_frame *frame;
	int ret;
	enum vts_run_mode mode = vts_get_run_mode(vtsdev);
	int tx;
	int rx;
	u8 *outbuf;
	u32 outbuf_len;
	u32 nr_tx;
	u32 nr_rx;

	vts_property_get(vtsdev, VTS_PROPERTY_SENSOR_TX_NUM, &nr_tx);
	vts_property_get(vtsdev, VTS_PROPERTY_SENSOR_RX_NUM, &nr_rx);

	if (type == VTS_FRAME_MUTUAL_RAW || type == VTS_FRAME_MUTUAL_DELTA)
		size = nr_tx* nr_rx;
	else
		size = nr_tx + nr_rx;

	frame = kzalloc(sizeof(*frame) + size * sizeof(short), GFP_KERNEL);
	if (!frame) {
		vts_dev_err(vtsdev, "alloc memory for frame failed\n");
		return NULL;
	}

	frame->type = type;
	frame->data = (short *)(((u8 *)frame) + sizeof(*frame));
	frame->size = size;
	INIT_LIST_HEAD(&frame->node);

	if (vts_change_to_mode(vtsdev, mode, VTS_ST_NORMAL, VTS_STA_DEFAUTL)) {
		vts_dev_err(vtsdev, "change to normal mode error!\n");
		kfree(frame);
		return NULL;
	}

	ret = vts_call_ic_ops(vtsdev, get_frame, type, frame->data, frame->size);
	vts_report_release(vtsdev);
	if (vts_change_to_mode(vtsdev, VTS_ST_NORMAL, mode, VTS_STA_DEFAUTL))
		vts_dev_err(vtsdev, "change to mode %s error!\n", vts_mode_name(mode));
	
	if (ret) {
		vts_dev_err(vtsdev, "get frame data error!, type = %d\n", type);
		kfree(frame);
		return NULL;
	}

	list_add_tail(&frame->node, &vtsdev->frames);

	outbuf_len = nr_rx * 16;
	outbuf = kzalloc(outbuf_len, GFP_KERNEL);
	if (!outbuf)
		return frame;

	vts_dev_info(vtsdev, "frame type:%d\n", type);
	if (type == VTS_FRAME_MUTUAL_RAW || type == VTS_FRAME_MUTUAL_DELTA) {
		for (tx = 0; tx < nr_tx; tx++) {
			memset(outbuf, 0, outbuf_len);
			for (rx = 0; rx < nr_rx ; rx++)
				snprintf(outbuf + strlen(outbuf), outbuf_len - strlen(outbuf), "%5d ", frame->data[tx * nr_rx  + rx]);
			vts_dev_info(vtsdev, "%s\n", outbuf);
		}
	} else {
		vts_dev_info(vtsdev, "      ");
		memset(outbuf, 0, outbuf_len);
		for (rx = 0; rx < nr_rx ; rx++)
			snprintf(outbuf + strlen(outbuf), outbuf_len - strlen(outbuf), "%5d ", frame->data[rx + nr_tx]);
		vts_dev_info(vtsdev, "%s\n", outbuf);
		for (tx = 0; tx < nr_tx; tx++)
			vts_dev_info(vtsdev, "%5d\n", frame->data[tx]);
	}
	kfree(outbuf);
	return frame;
}

int vts_frame_data_put(struct vts_device *vtsdev, struct vts_frame *frame)
{
	if (frame) {
		list_del(&frame->node);
		kfree(frame);
	}
	return 0;
}

struct vts_rom_zone {
	const char *name;
	enum vts_zone zone;
	u32 start;
	u32 end;
};

#define VTS_ROM_ZONE_DEF(n, z, s, e) {.name = n, .zone = z, .start = s, .end = e }

static const struct vts_rom_zone vts_rom_map[] = {
	VTS_ROM_ZONE_DEF("imei", VTS_ROM_ZONE_IMEI, 0, 14),
	VTS_ROM_ZONE_DEF("lcm_cali", VTS_ROM_ZONE_LCM, 15, 18)
};

ssize_t vts_rom_zone_read(struct vts_device *vtsdev, enum vts_zone zone, u8 *buf, size_t size)
{
	return 0;
}

ssize_t vts_rom_zone_write(struct vts_device *vtsdev, enum vts_zone zone, const u8 *buf, size_t size)
{
	return 0;
}

ssize_t vts_rom_zone_size(struct vts_device *vtsdev, enum vts_zone zone)
{
	return 0;
}

int vts_debug_interface(struct vts_device *vtsdev)
{
	VTI("vts debug interface");
	return 0;
}

int vts_interrupt_register(struct vts_device *vtsdev, unsigned int irq, vts_irq_handler_t irq_thread_fn, unsigned long flags, void *data)
{
	int ret;

	vtsdev->irq= irq;
	vtsdev->irq_thread_fn = irq_thread_fn;
	vtsdev->flags = flags;
	vtsdev->irqdata = data;
	ret = request_irq(irq, vts_interrupt_func, flags, vts_name(vtsdev), vtsdev);
	if (ret) {
		vts_dev_err(vtsdev, "request irq error!, ret = %d", ret);
		return ret;
	}

	return 0;
}

void vts_interrupt_unregister(struct vts_device *vtsdev)
{
	free_irq(vtsdev->irq, vtsdev);
}

static bool vts_ic_number_match(int ic_number, const int *ic_numbers, size_t size)
{
	int i;

	for (i = 0; i < size; i++) {
		if (ic_number == ic_numbers[i])
			return true;
	}
	return false;
}

static void vts_parse_cmdline(void)
{
	static unsigned long softid = 0;
	static bool match = false;
	struct device_node *node;
	char *pos = NULL;
	const char *cmdline = NULL;
	char str1[] = ":lcm_software_id=0xFF";
	char str2[] = "boot_puresys=1";
	unsigned int boot_puresys = 0;

	if (match)
		return;

	node = of_find_node_opts_by_path("/chosen", NULL);
	if (!node) {
		VTE("/chosen cannot be found\n");
		return;
	}

	if (of_property_read_string(node, "bootargs", &cmdline)) {
		VTE("bootargs read fail\n");
		return;
	}

	if (!cmdline) {
		VTE("cmdline is null\n");
		return;
	}

	strlcpy(str_cmdline, cmdline, sizeof(str_cmdline));

	pos = strnstr(str_cmdline, ":lcm_software_id=0x", strlen(str_cmdline));
	if (pos) {
		strlcpy(str1, pos, sizeof(str1));
		if (kstrtol(str1 + strlen(":lcm_software_id=0x"), 16, (unsigned long *)&softid)) {
			VTE("invalid softid: %s.\n", str_cmdline);
		} else {
			match = true;
			lcm_software_id = softid;
			VTI("found match lcm_software_id = <0x%x>!!!\n", (unsigned int)lcm_software_id);
		}
	}

	pos = strnstr(str_cmdline, "boot_puresys=", strlen(str_cmdline));
	if (pos) {
		strlcpy(str2, pos, sizeof(str2));
		if (kstrtol(str2 + strlen("boot_puresys="), 16, (unsigned long *)&boot_puresys)) {
			VTE("invalid boot_puresys: %s.\n", str_cmdline);
		} else {
			os_boot_loadfail = boot_puresys;
			VTI("found boot_puresys = <%d>!!!\n", boot_puresys);
		}
	}

	VTI("lcm_software_id:0x%x, os_boot_loadfail:%d", (unsigned int)lcm_software_id, os_boot_loadfail);
}

bool vts_is_load_driver(char *driver_name, const int *ic_numbers, size_t size)
{
	struct vts_panel_module *module;
	bool match = false;
	u32 lcm_id;
	enum vts_boot_mode boot_mode;

	vts_parse_cmdline();
	boot_mode = vts_get_boot_mode();
	lcm_id = vts_get_msdd_report_lcm_id(VTS_TYPE_MAIN);
	VTI("lcm_id from dispanel is %d", lcm_id);

	vts_parse_module_in_dts();

	mutex_lock(&modules_mutex);
	list_for_each_entry(module, &vts_panel_modules, list) {
		if (module->softid_active) {
			if (vts_module_contain_softid(module, vts_current_module_softid()) &&
				vts_ic_number_match(module->properties[VTS_PROPERTY_IC_NUMBER], ic_numbers, size)) {
				match = true;
				break;
			}
		} else {
			if (vts_module_contain_lcmid(module, vts_current_module_lcmid(module)) &&
				vts_ic_number_match(module->properties[VTS_PROPERTY_IC_NUMBER], ic_numbers, size)) {
				match = true;
				break;
			}
		}
	}
	mutex_unlock(&modules_mutex);

	if (!match) {
		VTE("no match module!!!\n");
		return false;
	}

	VTI("found match module\n");

	if (boot_mode == VTS_BOOT_MODE_POWER_OFF_CHARGING) {
		VTI("power of charging mode! don't load driver\n");
		return false;
	}

	if (boot_mode == VTS_BOOT_MODE_AT && !module->properties[VTS_PROPERTY_NO_FLASH]) {
		VTI("at mode and flash module, don't load driver\n");
		return false;
	}

	if (boot_mode == VTS_BOOT_MODE_OS_BOOT_LOADFAIL) {
		VTI("os_boot_puresys module, don't load driver\n");
		return false;
	}

	VTI("boot mode:%d, load %s driver\n", boot_mode, driver_name);
	return true;
}
