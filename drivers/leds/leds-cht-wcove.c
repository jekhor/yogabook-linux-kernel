// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for LEDs connected to the Intel Cherry Trail Whiskey Cove PMIC
 *
 * Copyright 2019 Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * Based on Lenovo Yoga Book Android kernel sources
 */
#include <linux/kernel.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/leds.h>


#define CHT_WC_LED1_CTRL		0x5e1f
#define CHT_WC_LED1_FSM			0x5e20
#define CHT_WC_LED1_PWM			0x5e21

#define CHT_WC_LED2_CTRL		0x4fdf
#define CHT_WC_LED2_FSM			0x4fe0
#define CHT_WC_LED2_PWM			0x4fe1

/* HW or SW control of charging led */
#define CHT_WC_LED1_SWCTL		BIT(0)
#define CHT_WC_LED1_ON			BIT(1)

#define CHT_WC_LED2_ON			BIT(0)
#define CHT_WC_LED_I_MA2_5		(2<<2)
/* LED current limit */
#define CHT_WC_LED_I_MASK		GENMASK(3, 2)

#define CHT_WC_LED_F_1_4_HZ		(0<<4)
#define CHT_WC_LED_F_1_2_HZ		(1<<4)
#define CHT_WC_LED_F_1_HZ		(2<<4)
#define CHT_WC_LED_F_2_HZ		(3<<4)
#define CHT_WC_LED_F_MASK		0x30

#define CHT_WC_LED_EFF_ON		(1<<1)     //always on
#define CHT_WC_LED_EFF_BLINKING		(2<<1)
#define CHT_WC_LED_EFF_BREATHING	(3<<1)
#define CHT_WC_LED_EFF_MASK		0x06


struct cht_wc_led {
	struct led_classdev cdev;
	struct intel_soc_pmic *pmic;
	const char *name;
	uint16_t ctrl_reg;
	uint8_t enable_mask;
	uint16_t fsm_reg;
	uint16_t pwm_reg;
};

static struct cht_wc_led cht_wc_leds[] = {
	{
		.name = "pmic::charge",
		.ctrl_reg = CHT_WC_LED1_CTRL,
		.fsm_reg = CHT_WC_LED1_FSM,
		.pwm_reg = CHT_WC_LED1_PWM,
		.enable_mask = CHT_WC_LED1_ON,
	},
	{
		.name = "pmic::gpled",
		.ctrl_reg = CHT_WC_LED2_CTRL,
		.fsm_reg = CHT_WC_LED2_FSM,
		.pwm_reg = CHT_WC_LED2_PWM,
		.enable_mask = CHT_WC_LED2_ON,
	},
};

static int cht_wc_leds_brightness_set(struct led_classdev *cdev,
		enum led_brightness value)
{
	struct cht_wc_led *led = container_of(cdev, struct cht_wc_led, cdev);
	int ret;

	if (!value) {
		ret = regmap_update_bits(led->pmic->regmap, led->ctrl_reg,
				led->enable_mask, 0);
		if (ret)
			dev_err(cdev->dev, "Failed to turn off: %d\n", ret);

		ret = regmap_update_bits(led->pmic->regmap, led->fsm_reg,
				CHT_WC_LED_EFF_MASK, CHT_WC_LED_EFF_ON);
		if (ret < 0)
			dev_err(cdev->dev, "Failed to update LED FSM reg: %d\n", ret);
	} else {
		ret = regmap_write(led->pmic->regmap, led->pwm_reg, value);
		if (ret)
			dev_err(cdev->dev, "Failed to set brightness: %d\n", ret);

		ret = regmap_update_bits(led->pmic->regmap, led->ctrl_reg,
				led->enable_mask, led->enable_mask);
		if (ret)
			dev_err(cdev->dev, "Failed to turn on: %d\n", ret);
	}
	return ret;
}

enum led_brightness cht_wc_leds_brightness_get(struct led_classdev *cdev)
{
	struct cht_wc_led *led = container_of(cdev, struct cht_wc_led, cdev);
	int ret;
	unsigned int val;

	ret = regmap_read(led->pmic->regmap, led->ctrl_reg, &val);
	if (ret < 0) {
		dev_err(cdev->dev, "Failed to read LED CTRL reg: %d\n", ret);
		return LED_OFF;
	}

	val &= led->enable_mask;

	if (!val)
		return LED_OFF;

	ret = regmap_read(led->pmic->regmap, led->pwm_reg, &val);
	if (ret < 0) {
		dev_err(cdev->dev, "Failed to read LED PWM reg: %d\n", ret);
		return LED_ON;
	}

	return val;
}

/* Return blinking period for given CTRL reg value */
static unsigned long cht_wc_leds_get_period(int ctrl)
{
	ctrl &= CHT_WC_LED_F_MASK;

	switch (ctrl) {
	case CHT_WC_LED_F_1_4_HZ:
		return 1000 * 4;
	case CHT_WC_LED_F_1_2_HZ:
		return 1000 * 2;
	case CHT_WC_LED_F_1_HZ:
		return 1000;
	case CHT_WC_LED_F_2_HZ:
		return 1000 / 2;
	};

	return 0;
}

/*
 * Find suitable hardware blink mode for given period.
 * period < 750 ms - select 2 HZ
 * 750 ms <= period < 1500 ms - select 1 HZ
 * 1500 ms <= period < 3000 ms - select 1/2 HZ
 * 3000 ms <= period < 5000 ms - select 1/4 HZ
 * 5000 ms <= period – return -1
 */
static int cht_wc_leds_find_freq(unsigned long period)
{

	if (period < 750)
		return CHT_WC_LED_F_2_HZ;
	else if (period < 1500)
		return CHT_WC_LED_F_1_HZ;
	else if (period < 3000)
		return CHT_WC_LED_F_1_2_HZ;
	else if (period < 5000)
		return CHT_WC_LED_F_1_4_HZ;
	else
		return -1;
}

static int cht_wc_leds_blink_set(struct led_classdev *cdev,
				  unsigned long *delay_on,
				  unsigned long *delay_off)
{
	struct cht_wc_led *led = container_of(cdev, struct cht_wc_led, cdev);
	unsigned int ctrl;
	int ret;

	if (!*delay_on && !*delay_off) {
		/* Return current settings */
		ret = regmap_read(led->pmic->regmap, led->ctrl_reg, &ctrl);

		if (ret < 0) {
			dev_err(cdev->dev, "Failed to read LED CTRL reg: %d\n", ret);
			return ret;
		}

		*delay_off = *delay_on = cht_wc_leds_get_period(ctrl) / 2;

		return 0;
	}

	ctrl = cht_wc_leds_find_freq(*delay_on + *delay_off);
	if (ctrl < 0) {
		/* Disable HW blinking */
		ret = regmap_update_bits(led->pmic->regmap, led->fsm_reg,
				CHT_WC_LED_EFF_MASK, CHT_WC_LED_EFF_ON);
		if (ret < 0)
			dev_err(cdev->dev, "Failed to update LED FSM reg: %d\n", ret);

		/* Fallback to software timer */
		*delay_on = *delay_off = 0;
		return -EINVAL;
	} else {
		ret = regmap_update_bits(led->pmic->regmap, led->fsm_reg,
				CHT_WC_LED_EFF_MASK, CHT_WC_LED_EFF_BLINKING);
		if (ret < 0)
			dev_err(cdev->dev, "Failed to update LED FSM reg: %d\n", ret);

		ret = regmap_update_bits(led->pmic->regmap, led->ctrl_reg,
					 CHT_WC_LED_F_MASK, ctrl);
		if (ret < 0)
			dev_err(cdev->dev, "Failed to update LED CTRL reg: %d\n", ret);

		*delay_off = *delay_on = cht_wc_leds_get_period(ctrl) / 2;
	}

	return 0;
}


static int cht_wc_leds_probe(struct platform_device *pdev)
{
	struct intel_soc_pmic *pmic = dev_get_drvdata(pdev->dev.parent);
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(cht_wc_leds); i++) {
		cht_wc_leds[i].pmic = pmic;
		cht_wc_leds[i].cdev.name = cht_wc_leds[i].name;
		cht_wc_leds[i].cdev.brightness_set_blocking = cht_wc_leds_brightness_set;
		cht_wc_leds[i].cdev.brightness_get = cht_wc_leds_brightness_get;
		cht_wc_leds[i].cdev.blink_set = cht_wc_leds_blink_set;
		cht_wc_leds[i].cdev.max_brightness = 255;

		ret = devm_led_classdev_register(&pdev->dev, &cht_wc_leds[i].cdev);
		if (ret < 0)
			return ret;
	}

	ret = regmap_update_bits(pmic->regmap, CHT_WC_LED1_CTRL,
				 CHT_WC_LED1_SWCTL, 1);

	if (ret)
		dev_err(&pdev->dev, "Failed to set SW control bit for charger LED: %d\n", ret);

	platform_set_drvdata(pdev, cht_wc_leds);

	return 0;
}

static const struct platform_device_id cht_wc_leds_table[] = {
	{ .name = "cht_wcove_leds" },
	{},
};
MODULE_DEVICE_TABLE(platform, cht_wc_leds_table);

static struct platform_driver cht_wc_leds_driver = {
	.probe = cht_wc_leds_probe,
	.id_table = cht_wc_leds_table,
	.driver = {
		.name = "cht_wcove_leds",
	},
};
module_platform_driver(cht_wc_leds_driver);

MODULE_DESCRIPTION("Intel Cherrytrail Whiskey Cove PMIC LEDs driver");
MODULE_AUTHOR("Yauhen Kharuzhy <jekhor@gmail.com>");
MODULE_LICENSE("GPL");

