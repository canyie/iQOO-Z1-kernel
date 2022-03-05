#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/fb.h>
#include <linux/list.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include "vts_core.h"

/*
*	TP report point callback
*/

struct tp_point_callback {
	struct list_head list;
	struct drivers_callback_handler *handler;
} point_callbacks;

int tp_report_point_register_callback(struct drivers_callback_handler *handler) {	

	struct tp_point_callback *new_callback = NULL;
	
	new_callback = kzalloc(sizeof(struct tp_point_callback), GFP_KERNEL);
	if (!new_callback) {
		pr_err("%s: Failed at allocate callback struct\n", __func__);
		return -ENOMEM;
	}

	new_callback->handler = handler;
	INIT_LIST_HEAD(&new_callback->list);
	list_add_tail(&new_callback->list, &point_callbacks.list);

	return 0;
}
EXPORT_SYMBOL_GPL(tp_report_point_register_callback);

void tp_report_point_unregister_callback(char *callback_name) {
	struct tp_point_callback *entry;
	
	if (!list_empty(&point_callbacks.list)) {
		list_for_each_entry(entry, &point_callbacks.list, list)
			if (!strcmp(entry->handler->name, callback_name)) {
				list_del(&entry->list);
				kfree(entry);
				return;
			}
	}
}
EXPORT_SYMBOL_GPL(tp_report_point_unregister_callback);


void tp_point_switch_do_callback(int touchId, int x, int y, int is_down)
{
	struct tp_point_callback *entry;
	
	if (!list_empty(&point_callbacks.list)) {
		list_for_each_entry(entry, &point_callbacks.list, list) {			
			entry->handler->point_report(touchId, x, y, is_down);
		}
	}
}

#ifdef CONFIG_INPUT_TIMESTAMP
static void vts_input_event(struct input_dev *dev,
		 unsigned int type, unsigned int code, int value, ktime_t timestamp)
{
	struct vts_device *vtsdev = input_get_drvdata(dev);

	if (vtsdev->report.flags & FLAGS_REPORT_TIMESTAMP)
		input_event_timestamp(dev, type, code, value, timestamp);
	else
		input_event(dev, type, code, value);
}

static void vts_input_mt_report_slot_state(struct input_dev *dev,
				unsigned int tool_type, bool active, ktime_t timestamp)
{
	struct vts_device *vtsdev = input_get_drvdata(dev);

	if (vtsdev->report.flags & FLAGS_REPORT_TIMESTAMP)
		input_mt_report_slot_state_timestamp(dev, tool_type, active, timestamp);
	else
		input_mt_report_slot_state(dev, tool_type, active);
}

#else
static void vts_input_event(struct input_dev *dev,
		 unsigned int type, unsigned int code, int value, ktime_t timestamp)
{
	input_event(dev, type, code, value);
}

static void vts_input_mt_report_slot_state(struct input_dev *dev,
				unsigned int tool_type, bool active, ktime_t timestamp)
{
	input_mt_report_slot_state(dev, tool_type, active);
}
#endif

static void vts_input_report_key(struct input_dev *dev, unsigned int code, int value, ktime_t kt)
{
	vts_input_event(dev, EV_KEY, code, !!value, kt);
}

static void vts_input_sync(struct input_dev *dev, ktime_t kt)
{
	vts_input_event(dev, EV_SYN, SYN_REPORT, 0, kt);
}

static void vts_input_report_abs(struct input_dev *dev, unsigned int code, int value, ktime_t kt)
{
	vts_input_event(dev, EV_ABS, code, value, kt);
}

static void vts_input_mt_slot(struct input_dev *dev, int slot, ktime_t kt)
{
	vts_input_event(dev, EV_ABS, ABS_MT_SLOT, slot, kt);
}

struct vts_report_event {
	struct list_head list;
	int touch_id;
	int nr_touches;
	int x;
	int y;
	int wx;
	int wy;
	int keycode;
	ktime_t kt;
};

static void inline vts_report_init_event(struct vts_report_event *event, int touch_id,
	int nr_touches, int x, int y, int wx, int wy, int keycode, ktime_t kt)
{
	event->touch_id = touch_id;
	event->nr_touches = nr_touches;
	event->x = x;
	event->y = y;
	event->wx = wx;
	event->wy = wy;
	event->keycode = keycode;
	event->kt = kt;
	INIT_LIST_HEAD(&event->list);
}

static int vts_report_add_event(struct vts_report *report, int touch_id, int nr_touches,
	int x, int y, int wx, int wy, int keycode, ktime_t kt, struct list_head *head)
{
	struct vts_report_event *event;
	struct vts_device *vtsdev = container_of(report, struct vts_device, report);

	event = kmem_cache_zalloc(report->km, GFP_ATOMIC);
	if (!event) {
		vts_dev_err(vtsdev, "alloc point mem failed!\n");
		return -ENOMEM;
	}

	vts_report_init_event(event, touch_id, nr_touches, x, y, wx, wy, keycode, kt);
	list_add_tail(&event->list, head);
	return 0;
}

static void vts_report_remove_event(struct vts_report *report, struct vts_report_event *event)
{
	list_del(&event->list);
	kmem_cache_free(report->km, event);
}

static int vts_report_add_point_down(struct vts_report *report, int touch_id, int nr_touches, int x,int y,int wx,int wy, ktime_t kt)
{
	return vts_report_add_event(report, touch_id, nr_touches, x, y, wx, wy, 0, kt, &report->down_points);
}

static int vts_report_add_point_up(struct vts_report *report, int touch_id, int nr_touches, int x,int y,int wx,int wy, ktime_t kt)
{
	return vts_report_add_event(report, touch_id, nr_touches, x, y, wx, wy, 0, kt, &report->up_points);
}

/* if set 15s lcd off,while press ts not move,maybe lcd will off. */
static void vts_long_press_work(struct work_struct *work)
{
	struct vts_long_press_report *long_press_report = container_of(work, struct vts_long_press_report, long_press_work);
	struct vts_device *vtsdev = container_of(long_press_report, struct vts_device, long_press_report);
	struct vts_report *report = &vtsdev->report;
	ktime_t kt = ktime_get();

	mutex_lock(&report->lock);
	vts_input_mt_slot(vtsdev->idev, long_press_report->slot, kt);
	vts_input_report_abs(vtsdev->idev, ABS_MT_TOUCH_MAJOR, long_press_report->major, kt);
	if (list_empty(&report->down_points))
		vts_input_sync(vtsdev->idev, kt);
	mutex_unlock(&report->lock);
	long_press_report->work_status = 0;
}

static enum hrtimer_restart vts_long_press_report(struct hrtimer *timer)
{
	struct vts_long_press_report *long_press_report = container_of(timer, struct vts_long_press_report, long_press_timer);

	long_press_report->major = long_press_report->major > long_press_report->index ? long_press_report->major - 1 : long_press_report->major + 1;
	schedule_work(&long_press_report->long_press_work);
	long_press_report->work_status = 1;
	hrtimer_forward_now(&long_press_report->long_press_timer, ms_to_ktime(10000));
	return HRTIMER_RESTART;
}

void vts_long_press_timer_init(struct vts_device *vtsdev)
{
	struct vts_long_press_report *report = &vtsdev->long_press_report;

	/* long press report major */
	hrtimer_init(&report->long_press_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	report->long_press_timer.function = vts_long_press_report;
	INIT_WORK(&vtsdev->long_press_report.long_press_work, vts_long_press_work);
}

void vts_long_press_timer_deinit(struct vts_device *vtsdev)
{
	if (vtsdev->long_press_report.work_status == 1) {
		cancel_work_sync(&vtsdev->long_press_report.long_press_work);
		vtsdev->long_press_report.work_status = 0;
	}

	hrtimer_cancel(&vtsdev->long_press_report.long_press_timer);
}

int vts_report_coordinates_set(struct vts_device *vtsdev, u16 *x, u16 *y, int nr_points)
{
	struct vts_report *report = &vtsdev->report;

	if (nr_points == 0 ||
		nr_points > ARRAY_SIZE(report->coordinates_x) ||
		nr_points > ARRAY_SIZE(report->coordinates_y))
		return -EINVAL;

	mutex_lock(&report->lock);
	report->nr_coordinates = 32;
	memset(report->coordinates_x, 0xff, ARRAY_SIZE(report->coordinates_x) * sizeof(*x));
	memset(report->coordinates_y, 0xff, ARRAY_SIZE(report->coordinates_y) * sizeof(*y));
	memcpy(report->coordinates_x, x, nr_points * sizeof(*x));
	memcpy(report->coordinates_y, y, nr_points * sizeof(*y));
	mutex_unlock(&report->lock);
	return 0;
}

int vts_report_coordinates_get(struct vts_device *vtsdev, u16 *x, u16 *y, size_t size, int *nr_points)
{
	struct vts_report *report = &vtsdev->report;

	if (report->nr_coordinates > size)
		return -EINVAL;

	if (report->nr_coordinates == 0)
		return -EBUSY;

	mutex_lock(&report->lock);
	*nr_points = report->nr_coordinates;
	memcpy(x, report->coordinates_x, *nr_points * sizeof(*x));
	memcpy(y, report->coordinates_y, *nr_points * sizeof(*y));
	report->nr_coordinates = 0;
	mutex_unlock(&report->lock);

	return 0;
}

int vts_update_dclick_point(struct vts_device *vtsdev,int x ,int y)
{
 	vtsdev->screen_clock_point.realX = x ;
	vtsdev->screen_clock_point.realY = y ;
	return 0;
}
int vts_report_event_down(struct vts_device *vtsdev, enum vts_event event)
{
	return 0;
}

int vts_report_event_up(struct vts_device *vtsdev, enum vts_event event)
{
	return 0;
}

int vts_report_point_down(struct vts_device *vtsdev, int touch_id, int nr_touches,
	int x, int y, int wx, int wy, bool large_press, u8 *custom_data, size_t custom_size, ktime_t kt)
{
	struct vts_report *report = &vtsdev->report;
	struct vts_point point_info;
	int ret = 0;
	u8 buf[32];
	int i;
	int high_x = 0;
	int high_y = 0;	
	int display_x, display_y, dimention_x, dimention_y;
	vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &display_x);
	vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &display_y);
	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &dimention_x);
	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &dimention_y);

	point_info.x = x;
	point_info.y = y;
	point_info.wx = wx;
	point_info.touch_id = touch_id;
	/* if long_press_report timer is running, cancel it when new point down */
	if (vtsdev->long_press_report.work_status == 1) {
		cancel_work_sync(&vtsdev->long_press_report.long_press_work);
		vtsdev->long_press_report.work_status = 0;
	}
	hrtimer_cancel(&vtsdev->long_press_report.long_press_timer);

	if (display_x && display_y) {
		if (vtsdev->fw_x == display_x && vtsdev->fw_y == display_y) {
			x = x * dimention_x / vtsdev->fw_x;
			y = y * dimention_y / vtsdev->fw_y;
		}
	
		high_x = x * display_x / dimention_x;
		high_y = y * display_y / dimention_y;
	}

	memset(buf, 0, sizeof(buf));
	for (i = 0; (i < custom_size) && custom_data; i++)
		snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "[0x%x]", custom_data[i]);

	mutex_lock(&report->lock);
	ret = vts_report_add_point_down(&vtsdev->report, touch_id, nr_touches, x, y, wx, wy, kt);
	tp_point_switch_do_callback(touch_id, x, y, 1);
	set_bit(touch_id, &report->touchbit);
	mutex_unlock(&report->lock);
	return ret;
}

int vts_report_point_up(struct vts_device *vtsdev, int touch_id, int nr_touches, int x, int y, int wx, int wy, bool large_press, ktime_t kt)
{
	struct vts_report *report = &vtsdev->report;
	struct vts_point point_info;
	int ret = 0;
	int high_x = 0;
	int high_y = 0;
	int display_x, display_y, dimention_x, dimention_y;

	point_info.x = x;
	point_info.y = y;
	point_info.wx = wx;
	point_info.touch_id = touch_id;

	vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &display_x);
	vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &display_y);
	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &dimention_x);
	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &dimention_y);

	if (display_x && display_y) {
		if (vtsdev->fw_x == display_x && vtsdev->fw_y == display_y) {
			x = x * dimention_x / vtsdev->fw_x;
			y = y * dimention_y / vtsdev->fw_y;
		}
		high_x = x * display_x / dimention_x;
		high_y = y * display_y / dimention_y;
	}

	mutex_lock(&report->lock);
	ret = vts_report_add_point_up(&vtsdev->report, touch_id, nr_touches, x, y, wx, wy, kt);
	tp_point_switch_do_callback(touch_id, x, y, 0);
	clear_bit(touch_id, &report->touchbit);
	mutex_unlock(&report->lock);

	return ret;
}

int vts_report_release(struct vts_device *vtsdev)
{
	struct vts_report *report = &vtsdev->report;
	int i;
	ktime_t kt = ktime_get();

	/* if long_press_report timer is running, cancel it when point release */
	if (vtsdev->long_press_report.work_status == 1) {
		cancel_work_sync(&vtsdev->long_press_report.long_press_work);
		vtsdev->long_press_report.work_status = 0;
	}
	hrtimer_cancel(&vtsdev->long_press_report.long_press_timer);

	if (!report->slotbit)
		return 0;

	mutex_lock(&report->lock);
	for (i = 0 ; i < sizeof(report->slotbit) * 8; i++) {
		if (test_bit(i, &report->slotbit)) {
			vts_input_mt_slot(vtsdev->idev, i, kt);
			vts_input_mt_report_slot_state(vtsdev->idev, MT_TOOL_FINGER, 0, kt);
			clear_bit(i, &report->slotbit);
		}
	}

	report->touchbit = 0;
	input_report_key(vtsdev->idev, BTN_TOOL_FINGER, 0);
	input_report_key(vtsdev->idev, BTN_TOUCH, 0);
	input_sync(vtsdev->idev);
	mutex_unlock(&report->lock);
	return 0;
}

int vts_report_ic_status(struct vts_device *vtsdev, int status)
{
	return 0;
}

int vts_report_set_flags(struct vts_device *vtsdev, unsigned long flags)
{
#ifndef CONFIG_INPUT_TIMESTAMP
	if(flags & FLAGS_REPORT_TIMESTAMP) {
		vts_dev_info(vtsdev, "not support input event with timestamp\n");
		return -EPERM;
	}
#endif
	vtsdev->report.flags = flags;
	return 0;
}

unsigned long vts_report_flags(struct vts_device *vtsdev)
{
	return vtsdev->report.flags;
}

int vts_report_ic_exception(struct vts_device *vtsdev, enum vts_ic_exception exception, int arg)
{
	return vts_notify_ic_exception(vtsdev, exception, arg);
}

int vts_report_point_sync(struct vts_device *vtsdev)
{
	struct vts_report *report = &vtsdev->report;
	struct vts_report_event *point;
	ktime_t kt;
	int last_major, last_slot;

	mutex_lock(&report->lock);

	while(!list_empty(&report->down_points)) {
		point = list_first_entry(&report->down_points,struct vts_report_event, list);
		kt = point->kt;

		if (report->slotbit == 0) {
			vts_input_report_key(vtsdev->idev, BTN_TOUCH, 1, kt);
			vts_input_report_key(vtsdev->idev, BTN_TOOL_FINGER, 1, kt);
		}

		set_bit(point->touch_id, &report->slotbit);
		vts_input_mt_slot(vtsdev->idev, point->touch_id, kt);
		vts_input_mt_report_slot_state(vtsdev->idev, MT_TOOL_FINGER, 1, kt);
		vts_input_report_abs(vtsdev->idev, ABS_MT_POSITION_X, point->x, kt);
		vts_input_report_abs(vtsdev->idev, ABS_MT_POSITION_Y, point->y, kt);
		vts_input_report_abs(vtsdev->idev, ABS_MT_TOUCH_MAJOR, max(point->wx, point->wy), kt);
		vts_input_report_abs(vtsdev->idev, ABS_MT_TOUCH_MINOR, min(point->wx, point->wy), kt);
		last_slot = point->touch_id;
		last_major = max(point->wx, point->wy);
		vts_report_remove_event(report, point);


		if (list_empty(&report->down_points)) {
			vts_input_sync(vtsdev->idev, kt);
			vtsdev->long_press_report.slot = last_slot;
			vtsdev->long_press_report.index = last_major;
			vtsdev->long_press_report.major = last_major;
			hrtimer_start(&vtsdev->long_press_report.long_press_timer, ms_to_ktime(10000), HRTIMER_MODE_REL);
		}
	}

	while (!list_empty(&report->up_points)) {
		point = list_first_entry(&report->up_points,struct vts_report_event, list);
		kt = point->kt;

		clear_bit(point->touch_id, &report->slotbit);
		vts_input_mt_slot(vtsdev->idev, point->touch_id, kt);
		vts_input_mt_report_slot_state(vtsdev->idev, MT_TOOL_FINGER, 0, kt);
		vts_report_remove_event(report, point);

		if (report->slotbit == 0) {
			vts_input_report_key(vtsdev->idev, BTN_TOOL_FINGER, 0, kt);
			vts_input_report_key(vtsdev->idev, BTN_TOUCH, 0, kt);
		}

		if (list_empty(&report->up_points)) {
			vts_input_sync(vtsdev->idev, kt);
			/* if long_press_report timer is running, cancel it when point up */
			if (vtsdev->long_press_report.work_status == 1) {
				cancel_work_sync(&vtsdev->long_press_report.long_press_work);
				vtsdev->long_press_report.work_status = 0;
			}
			hrtimer_cancel(&vtsdev->long_press_report.long_press_timer);
		}
	}

	mutex_unlock(&report->lock);
	return 0;
}

static int vts_report_input_init(struct vts_device *vtsdev)
{
	int ret;
	u32 x_max = 0;
	u32 y_max = 0;

	vtsdev->idev = input_allocate_device();
	if (!vtsdev->idev) {
		vts_dev_err(vtsdev, "no memory for alloc input device\n");
		return -ENOMEM;
	}

	vtsdev->idev->name = vts_name(vtsdev);
	vtsdev->idev->id.bustype = vtsdev->busType;;
	vtsdev->idev->dev.parent = NULL;
	vtsdev->idev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_mt_init_slots(vtsdev->idev, VIVO_TS_MAX_TOUCH_NUM, INPUT_MT_DIRECT);
	vtsdev->idev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	set_bit(BTN_TOOL_FINGER, vtsdev->idev->keybit);

	input_set_capability(vtsdev->idev, EV_KEY, KEY_TS_LARGE_SUPPRESSION);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_WAKEUP);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_WAKEUP_SWIPE);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_LEFT);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_RIGHT);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_UP);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_O);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_W);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_E);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_M);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_C);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_F);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_A);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_V);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_H);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_MENU);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_HOMEPAGE);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_BACK);
	input_set_capability(vtsdev->idev, EV_KEY, VTS_KEY_FINGER_GESTURE);
	input_set_capability(vtsdev->idev, EV_KEY, VTS_KEY_FACE_GESTURE);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_CAMERA);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_LONGPRESS_DOWN);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_LONGPRESS_UP);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_VIRTUAL_WAKEUP);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_SCREEN_CLOCK_WAKE_UP);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_INSIDE_SLIDE_LEFT);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_INSIDE_SLIDE_RIGHT);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_QUIT_ACTIVE_MODE);

	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &x_max);
	input_set_abs_params(vtsdev->idev, ABS_MT_POSITION_X, 0, x_max - 1, 0, 0);
	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &y_max);
	input_set_abs_params(vtsdev->idev, ABS_MT_POSITION_Y, 0, y_max - 1, 0, 0);
	input_set_abs_params(vtsdev->idev, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0);
	input_set_abs_params(vtsdev->idev, ABS_MT_TOUCH_MINOR, 0, 31, 0, 0);
	input_mt_init_slots(vtsdev->idev, VIVO_TS_MAX_TOUCH_NUM, 0);
	input_set_drvdata(vtsdev->idev, vtsdev);

	ret = input_register_device(vtsdev->idev);
	if (ret) {
		vts_dev_err(vtsdev, "register input device failed. ret = %d\n", ret);
		input_free_device(vtsdev->idev);
		return ret;
	}

	return 0;
}

static void vts_report_input_deinit(struct vts_device *vtsdev)
{
	input_unregister_device(vtsdev->idev);
	vtsdev->idev = NULL;
	return ;
}

int vts_report_init(struct vts_device *vtsdev)
{
	struct vts_report *report = &vtsdev->report;
	int ret;
	u32 report_timestamp = 0;

	snprintf(report->km_name, sizeof(report->km_name), "vts_report_km%d", vtsdev->type);
	report->km = kmem_cache_create(report->km_name, sizeof(struct vts_report_event), 0, 0, NULL);
	if (!report->km) {
		vts_dev_err(vtsdev, "create mem cache failed!\n");
		return -ENOMEM;
	}

	ret = vts_report_input_init(vtsdev);
	if (ret) {
		vts_dev_err(vtsdev, "report input init error, ret = %d\n", ret);
		kmem_cache_destroy(report->km);
		report->km = NULL;
		return ret;
	}

	mutex_init(&report->lock);
	report->slotbit = 0;
	vts_property_get(vtsdev, VTS_PROPERTY_REPORT_TIMESTAMP, &report_timestamp);
#ifdef CONFIG_INPUT_TIMESTAMP
	if (report_timestamp)
		report->flags |= FLAGS_REPORT_TIMESTAMP;
#else
	report->flags &= ~FLAGS_REPORT_TIMESTAMP;
#endif
	INIT_LIST_HEAD(&report->down_points);
	INIT_LIST_HEAD(&report->up_points);
	INIT_LIST_HEAD(&report->keys);
	INIT_LIST_HEAD(&point_callbacks.list);
	return 0;
}

int vts_report_deinit(struct vts_device *vtsdev)
{
	struct vts_report *report = &vtsdev->report;

	mutex_destroy(&report->lock);
	vts_report_input_deinit(vtsdev);
	return 0;
}
