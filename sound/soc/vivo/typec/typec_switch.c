/******************************************
******************************************
       file:typec_switch.c
       function: for typec headset common function
       author: qinbaoqiang
       version: v00  2020-11-16
*********************************************
********************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/sched/clock.h>
#include <linux/timer.h>
#include <linux/irq.h>

#ifdef CONFIG_MTK_ACCDET
#include "accdet.h"
#endif

#include "typec_switch.h"

#ifdef pr_debug
#undef pr_debug
#define pr_debug pr_info
#endif

#define EINT_PIN_PLUG_IN        (1)
#define EINT_PIN_PLUG_OUT       (0)

static u32 cur_eint_state = EINT_PIN_PLUG_OUT;
static u32 typec_eint_type = IRQ_TYPE_LEVEL_LOW;

const struct of_device_id typec_of_match[];
static struct work_struct eint_work;
static struct workqueue_struct *eint_workqueue;
static struct pinctrl *typec_switch_pinctrl;
static struct pinctrl_state *pins_eint;
static u32 gpiopin, gpio_typec_deb;
static u32 typec_switch_irq;

static struct notifier_block typec_switch_nb;
static struct tcpc_device *typec_switch_tcpc_dev;

extern void max14743_en(bool en);
static int typec_switch_tcp_notifier_call(struct notifier_block *nb, unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	pr_info("event=%ld,vbus_state[type:mv:ma]=[%d:%d:%d]\n",
			event, noti->vbus_state.type, noti->vbus_state.mv, noti->vbus_state.ma);
	switch (event) {
	case TCP_NOTIFY_TYPEC_STATE:
		if (noti->typec_state.new_state == TYPEC_ATTACHED_SNK ||
				noti->typec_state.new_state == TYPEC_ATTACHED_CUSTOM_SRC) {
			pr_debug("USB Plug in, pol = %d\n", noti->typec_state.polarity);
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SNK ||
				noti->typec_state.old_state == TYPEC_ATTACHED_CUSTOM_SRC) &&
				noti->typec_state.new_state == TYPEC_UNATTACHED) {
			pr_debug("USB Plug out\n");
		}
		if (noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			pr_debug("legen-OTG Plug in\n");
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_SRC &&
				noti->typec_state.new_state == TYPEC_UNATTACHED) {
			pr_debug("legen-OTG Plug out\n");
		}
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_AUDIO) {
			pr_debug("%s-audio Plug in.\n", __func__);
			/* audio plugin logic here */
			if (max20328_is_ready)
				max20328_usbc_set_switch_mode(POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER);
			accdet_enable_power(true);
			max14743_en(true);
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_AUDIO &&
			noti->typec_state.new_state == TYPEC_UNATTACHED) {
			pr_debug("%s-audio Plug out.\n", __func__);
			/* audio plugout logic here */
			if (max20328_is_ready)
				max20328_usbc_set_switch_mode(POWER_SUPPLY_TYPEC_NONE);
			accdet_enable_power(false);
			max14743_en(false);

		}
		break;
	default:
		break;
	
	}
	return NOTIFY_OK;
}
void accdet_register_typec_notifier_call(void)
{
	int ret = 0;

	if (typec_switch_tcpc_dev) {
		pr_info("%s dev already exist\n", __func__);
		return;
	}

	if (!typec_switch_tcpc_dev)
		typec_switch_tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	if (!typec_switch_tcpc_dev) {
		pr_err("%s: get type_c_port0 fail\n", __func__);

		return;
	}

	typec_switch_nb.notifier_call = typec_switch_tcp_notifier_call;
	ret = register_tcp_dev_notifier(typec_switch_tcpc_dev, &typec_switch_nb,
		TCP_NOTIFY_TYPE_VBUS | TCP_NOTIFY_TYPEC_STATE |
		TCP_NOTIFY_TYPE_MISC);
	if (ret < 0) {
		pr_err("%s:register OTG <%p> fail\n", __func__, typec_switch_tcpc_dev);
		return;
	}
	pr_info("%s register success!\n", __func__);

}

static void eint_work_callback(struct work_struct *work)
{
	unsigned int retry = 25;
	int pin_val = 0;

	if (gpio_is_valid(gpiopin))
		pin_val = gpio_get_value(gpiopin);
	pr_info("%s: pin_val=%d, mmax_is_unuseirq=%d\n", __func__, pin_val, mmax_is_unuseirq);

	if (mmax_is_unuseirq)
		goto leave;

	if (!pin_val) {
		#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
		while(!get_typec_drp() && retry) {
			vote_typec_drp(ACCDET_TCPC_VOTER, true);
			if (!get_typec_drp())
				msleep(200);
			--retry;
		}
		#endif
	} else {
		pr_info("accdet cur:plug-out, cur_eint_state = %d\n",
			cur_eint_state);
		#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
		if (get_typec_drp())
			vote_typec_drp(ACCDET_TCPC_VOTER, false);
		#endif
		if (max20328_is_ready)
			max20328_usbc_set_switch_mode(POWER_SUPPLY_TYPEC_NONE);
		accdet_enable_power(false);

	}

leave:
	enable_irq(typec_switch_irq);
	pr_info("[typec]typec_switch_irq retry=%d  !!!!!!\n", retry);

}

static irqreturn_t typec_ex_eint_handler(int irq, void *data)
{
	int ret = 0;

	if (cur_eint_state == EINT_PIN_PLUG_IN) {
		/* To trigger EINT when the headset was plugged in
		 * We set the polarity back as we initialed.
		 */
		if (typec_eint_type == IRQ_TYPE_LEVEL_HIGH)
			irq_set_irq_type(typec_switch_irq, IRQ_TYPE_LEVEL_HIGH);
		else
			irq_set_irq_type(typec_switch_irq, IRQ_TYPE_LEVEL_LOW);
		gpio_set_debounce(gpiopin, gpio_typec_deb);
		cur_eint_state = EINT_PIN_PLUG_OUT;
	} else {
		/* To trigger EINT when the headset was plugged out
		 * We set the opposite polarity to what we initialed.
		 */
		if (typec_eint_type == IRQ_TYPE_LEVEL_HIGH)
			irq_set_irq_type(typec_switch_irq, IRQ_TYPE_LEVEL_LOW);
		else
			irq_set_irq_type(typec_switch_irq, IRQ_TYPE_LEVEL_HIGH);
		gpio_set_debounce(gpiopin, 16 * 1000);
		cur_eint_state = EINT_PIN_PLUG_IN;
	}

	disable_irq_nosync(typec_switch_irq);
	pr_info("accdet %s(), cur_eint_state=%d. \n", __func__, cur_eint_state);
	ret = queue_work(eint_workqueue, &eint_work);
	return IRQ_HANDLED;
}


static inline int typec_ext_eint_setup(struct platform_device *platform_device)
{
	int ret = 0;
	u32 ints[4] = { 0 };
	struct device_node *node = NULL;
	struct pinctrl_state *pins_default = NULL;

	pr_info("accdet %s()\n", __func__);
	typec_switch_pinctrl = devm_pinctrl_get(&platform_device->dev);
	if (IS_ERR(typec_switch_pinctrl)) {
		ret = PTR_ERR(typec_switch_pinctrl);
		dev_notice(&platform_device->dev, "get typec_switch_pinctrl fail.\n");
		return ret;
	}

	pins_default = pinctrl_lookup_state(typec_switch_pinctrl, "default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		dev_notice(&platform_device->dev,
			"deflt pinctrl not found, skip it\n");
	}

	pins_eint = pinctrl_lookup_state(typec_switch_pinctrl, "typec_eint_as_int");
	if (IS_ERR(pins_eint)) {
		ret = PTR_ERR(pins_eint);
		dev_notice(&platform_device->dev, "lookup eint pinctrl fail\n");
		return ret;
	}
	pinctrl_select_state(typec_switch_pinctrl, pins_eint);

	node = of_find_matching_node(node, typec_of_match);
	if (!node) {
		pr_notice("accdet %s can't find compatible node\n", __func__);
		return -1;
	}

	gpiopin = of_get_named_gpio(node, "deb-gpios", 0);
	ret = of_property_read_u32(node, "debounce", &gpio_typec_deb);
	if (ret < 0) {
		pr_notice("accdet %s gpiodebounce not found,ret:%d\n",
			__func__, ret);
		return ret;
	}

	gpio_set_debounce(gpiopin, gpio_typec_deb);

	typec_switch_irq = irq_of_parse_and_map(node, 0);
	ret = of_property_read_u32_array(node, "interrupts", ints,
			ARRAY_SIZE(ints));
	if (ret) {
		pr_notice("accdet %s interrupts not found,ret:%d\n",
			__func__, ret);
		return ret;
	}
	typec_eint_type = ints[1];
	pr_info("accdet set gpio EINT, typec_switch_irq=%d, gpiopin=%d, typec_eint_type=%d\n",
			typec_switch_irq, ints[0], typec_eint_type);
	ret = request_threaded_irq(typec_switch_irq, NULL, typec_ex_eint_handler,
		IRQF_TRIGGER_NONE | IRQF_ONESHOT, "typec-switch-eint", NULL);
	if (ret) {
		pr_notice("accdet %s request_irq fail, ret:%d.\n", __func__,
			ret);
		return ret;
	}

	pr_info("typec switch set gpio EINT finished, irq=%d, gpio_typec_deb=%d\n",
			typec_switch_irq, gpio_typec_deb);

	return 0;
}

static int typec_switch_probe(struct platform_device *dev)
{
	int ret;

	pr_info("%s: enter\n", __func__);
	eint_workqueue = create_singlethread_workqueue("typec_switch_eint");
	INIT_WORK(&eint_work, eint_work_callback);
	if (!eint_workqueue) {
		ret = -1;
		pr_notice("%s create eint workqueue fail.\n", __func__);
		return ret;
	}
	accdet_register_typec_notifier_call();
	msleep(20);
	ret = typec_ext_eint_setup(dev);
	if (ret) {
		pr_notice("%s ap eint setup fail.ret:%d\n", __func__, ret);
		goto err_create_workqueue;
	}

	return 0;

err_create_workqueue:
	destroy_workqueue(eint_workqueue);
	return ret;
}

static int typec_switch_remove(struct platform_device *dev)
{
	destroy_workqueue(eint_workqueue);
	return 0;
}

const struct of_device_id typec_of_match[] = {
	{ .compatible = "vivo,typec-switch", },
	{},
};

static struct platform_driver typec_switch_driver = {
	.probe = typec_switch_probe,
	.remove = typec_switch_remove,
	.driver = {
		.name = "typec witch driver",
		.of_match_table = typec_of_match,
	},
};


static int typec_switch_mod_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&typec_switch_driver);
	if (ret)
		pr_notice("Typec switch platform_driver_register error:(%d)\n", ret);

	pr_info("%s() done!\n", __func__);
	return ret;
}

static void typec_switch_mod_exit(void)
{
	pr_info("%s()\n", __func__);
	platform_driver_unregister(&typec_switch_driver);
}

module_init(typec_switch_mod_init);
module_exit(typec_switch_mod_exit);


MODULE_DESCRIPTION("VIVO TYPEC SWITCH driver");
MODULE_AUTHOR("qinbaoqiang <qinbaoqiang@vivo.com>");
MODULE_LICENSE("GPL");

