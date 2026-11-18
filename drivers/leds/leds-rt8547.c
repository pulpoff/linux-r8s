// SPDX-License-Identifier: GPL-2.0
//
// Richtek RT8547 flash LED driver, torch mode only.
//
// The RT8547 takes its settings over a one-wire interface on CTRL: a write
// is the chip address, a 3-bit register and a data byte, MSB first, each bit
// a low/high pulse pair whose long half is the value (0: long low, 1: long
// high). EN then lights the LED in the mode the current register selects.
// This drives only the torch ("movie") mode at up to 225 mA, the Galaxy
// S20 FE's continuous level; the strobe (up to 1.5 A) is left alone.

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spinlock.h>

#define RT8547_ADDR			0x99

#define RT8547_REG_LVP			0x1	/* low-voltage protection */
#define RT8547_LVP_3V			0x0
#define RT8547_REG_FLASH		0x2	/* [7:5] timer min. current */
#define RT8547_FLASH_TIMEOUT_400MA	(0x7 << 5)
#define RT8547_REG_CURRENT		0x3
#define RT8547_CURRENT_TORCH		BIT(4)	/* clear: strobe */
#define RT8547_TORCH_STEP_UA		25000	/* level n is (n + 1) x 25 mA */

/* the one-wire timing, in us */
#define RT8547_T_SHORT			4
#define RT8547_T_LONG			60
#define RT8547_T_SOD			10	/* start of data */
#define RT8547_T_EOD			10	/* end of data */

#define RT8547_MAX_UA			225000

struct rt8547 {
	struct led_classdev cdev;
	struct gpio_desc *en;
	struct gpio_desc *ctrl;
	struct mutex lock;		/* serialises writes and EN */
};

static void rt8547_send_bit(struct rt8547 *rt, bool one)
{
	gpiod_set_value(rt->ctrl, 0);
	udelay(one ? RT8547_T_SHORT : RT8547_T_LONG);
	gpiod_set_value(rt->ctrl, 1);
	udelay(one ? RT8547_T_LONG : RT8547_T_SHORT);
}

/* ~1.3 ms with interrupts off: a stretched pulse would be read as a bit */
static void rt8547_write(struct rt8547 *rt, u8 reg, u8 val)
{
	u32 word = (RT8547_ADDR << 11) | ((reg & 0x7) << 8) | val;
	unsigned long flags;
	int i;

	local_irq_save(flags);
	gpiod_set_value(rt->ctrl, 1);
	udelay(RT8547_T_SOD);
	for (i = 18; i >= 0; i--)
		rt8547_send_bit(rt, word & BIT(i));
	gpiod_set_value(rt->ctrl, 0);
	udelay(RT8547_T_SHORT);
	gpiod_set_value(rt->ctrl, 1);
	udelay(RT8547_T_EOD);
	local_irq_restore(flags);
}

static int rt8547_brightness_set(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	struct rt8547 *rt = container_of(cdev, struct rt8547, cdev);

	mutex_lock(&rt->lock);
	if (brightness == LED_OFF) {
		gpiod_set_value_cansleep(rt->en, 0);
		gpiod_set_value(rt->ctrl, 0);
	} else {
		rt8547_write(rt, RT8547_REG_LVP, RT8547_LVP_3V);
		rt8547_write(rt, RT8547_REG_CURRENT,
			     RT8547_CURRENT_TORCH | (brightness - 1));
		rt8547_write(rt, RT8547_REG_FLASH, RT8547_FLASH_TIMEOUT_400MA);
		gpiod_set_value_cansleep(rt->en, 1);
	}
	mutex_unlock(&rt->lock);
	return 0;
}

static int rt8547_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct led_init_data init_data = {};
	struct fwnode_handle *child;
	struct rt8547 *rt;
	u32 max_ua = RT8547_MAX_UA;
	int ret;

	rt = devm_kzalloc(dev, sizeof(*rt), GFP_KERNEL);
	if (!rt)
		return -ENOMEM;
	platform_set_drvdata(pdev, rt);

	rt->en = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(rt->en))
		return dev_err_probe(dev, PTR_ERR(rt->en), "no enable GPIO\n");
	rt->ctrl = devm_gpiod_get(dev, "ctrl", GPIOD_OUT_LOW);
	if (IS_ERR(rt->ctrl))
		return dev_err_probe(dev, PTR_ERR(rt->ctrl), "no ctrl GPIO\n");
	if (gpiod_cansleep(rt->ctrl))
		return dev_err_probe(dev, -EINVAL,
				     "ctrl must be a GPIO that cannot sleep\n");

	child = device_get_next_child_node(dev, NULL);
	if (!child)
		return dev_err_probe(dev, -EINVAL, "no LED node\n");
	fwnode_property_read_u32(child, "led-max-microamp", &max_ua);
	max_ua = clamp(max_ua, RT8547_TORCH_STEP_UA, RT8547_MAX_UA);

	ret = devm_mutex_init(dev, &rt->lock);
	if (ret)
		goto out;

	rt->cdev.max_brightness = max_ua / RT8547_TORCH_STEP_UA;
	rt->cdev.brightness_set_blocking = rt8547_brightness_set;
	rt->cdev.flags = LED_CORE_SUSPENDRESUME;
	init_data.fwnode = child;
	init_data.devicename = NULL;
	init_data.default_label = "flash";
	ret = devm_led_classdev_register_ext(dev, &rt->cdev, &init_data);
	if (ret)
		dev_err_probe(dev, ret, "cannot register the LED\n");
out:
	fwnode_handle_put(child);
	return ret;
}

static void rt8547_shutdown(struct platform_device *pdev)
{
	struct rt8547 *rt = platform_get_drvdata(pdev);

	if (rt)
		rt8547_brightness_set(&rt->cdev, LED_OFF);
}

static const struct of_device_id rt8547_of_match[] = {
	{ .compatible = "richtek,rt8547" },
	{ }
};
MODULE_DEVICE_TABLE(of, rt8547_of_match);

static struct platform_driver rt8547_driver = {
	.driver = {
		.name = "leds-rt8547",
		.of_match_table = rt8547_of_match,
	},
	.probe = rt8547_probe,
	.shutdown = rt8547_shutdown,
};
module_platform_driver(rt8547_driver);

MODULE_DESCRIPTION("Richtek RT8547 flash LED (torch mode)");
MODULE_LICENSE("GPL");
