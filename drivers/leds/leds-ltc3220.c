// SPDX-License-Identifier: GPL-2.0+
/*
 * LTC3220 18-Channel LED Driver
 *
 * Copyright 2025 Analog Devices Inc.
 *
 * Author: Edelweise Escala <edelweise.escala@analog.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/platform_data/leds-ltc3220.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

/* LTC3220 Registers */
#define LTC3220_COMMAND     0x00
#define LTC3220_ULED01      0x01
#define LTC3220_ULED02      0x02
#define LTC3220_ULED03      0x03
#define LTC3220_ULED04      0x04
#define LTC3220_ULED05      0x05
#define LTC3220_ULED06      0x06
#define LTC3220_ULED07      0x07
#define LTC3220_ULED08      0x08
#define LTC3220_ULED09      0x09
#define LTC3220_ULED10      0x0A
#define LTC3220_ULED11      0x0B
#define LTC3220_ULED12      0x0C
#define LTC3220_ULED13      0x0D
#define LTC3220_ULED14      0x0E
#define LTC3220_ULED15      0x0F
#define LTC3220_ULED16      0x10
#define LTC3220_ULED17      0x11
#define LTC3220_ULED18      0x12
#define LTC3220_GRAD_BLINK  0x13

static int ltc3220_write(struct i2c_client *client, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int ltc3220_set_command(struct ltc3220_platform_data *platform_data,
	bool is_shutdown, bool force2x, bool force1p5, bool quickWrite)
{
	u8 reg_val;
	int ret;
	struct i2c_client *client = platform_data->client;

	reg_val = (is_shutdown << 3) | (force2x << 2)
	| (force1p5 << 1) | quickWrite;
	ret = ltc3220_write(client, LTC3220_COMMAND, reg_val);
	if (ret < 0) {
		dev_err(&client->dev, "Set command fail\n");
		return ret;
	}

	return 0;
}

static ssize_t ltc3220_shutdown_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ltc3220_platform_data *platform_data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", platform_data->command_cfg.is_shutdown);
}

static ssize_t ltc3220_shutdown_store(struct device *dev,
					struct device_attribute *devAttr,
					const char *buf, size_t size)
{
	unsigned int state;
	int ret;
	struct ltc3220_platform_data *platform_data = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 0, &state);
	if (ret)
		return ret;

	ret = ltc3220_set_command(platform_data, state,
		platform_data->command_cfg.is_force_cpo_2x,
		platform_data->command_cfg.is_force_cpo_1p5x,
		platform_data->command_cfg.is_quick_write);
	if (ret < 0)
		return ret;

	platform_data->command_cfg.is_shutdown = state;

	return size;
}

static ssize_t ltc3220_force_cpo_2x_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ltc3220_platform_data *platform_data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", platform_data->command_cfg.is_force_cpo_2x);
}

static ssize_t ltc3220_force_cpo_2x_store(struct device *dev,
						struct device_attribute *devAttr,
						const char *buf, size_t size)
{
	unsigned int state;
	int ret;
	struct ltc3220_platform_data *platform_data = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 0, &state);
	if (ret)
		return ret;

	ret = ltc3220_set_command(platform_data,
		platform_data->command_cfg.is_shutdown,
		state,
		platform_data->command_cfg.is_force_cpo_1p5x,
		platform_data->command_cfg.is_quick_write);
	if (ret < 0)
		return ret;

	platform_data->command_cfg.is_force_cpo_2x = state;

	return size;
}

static ssize_t ltc3220_force_cpo_1p5x_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ltc3220_platform_data *platform_data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", platform_data->command_cfg.is_force_cpo_1p5x);
}

static ssize_t ltc3220_force_cpo_1p5x_store(struct device *dev,
							struct device_attribute *devAttr,
							const char *buf, size_t size)
{
	unsigned int state;
	int ret;
	struct ltc3220_platform_data *platform_data = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 0, &state);
	if (ret)
		return ret;

	ret = ltc3220_set_command(platform_data,
		platform_data->command_cfg.is_shutdown,
		platform_data->command_cfg.is_force_cpo_2x,
		state,
		platform_data->command_cfg.is_quick_write);
	if (ret < 0)
		return ret;

	platform_data->command_cfg.is_force_cpo_1p5x = state;

	return size;
}

static ssize_t ltc3220_quick_write_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ltc3220_platform_data *platform_data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", platform_data->command_cfg.is_quick_write);
}

static ssize_t ltc3220_quick_write_store(struct device *dev,
					struct device_attribute *devAttr,
					const char *buf, size_t size)
{
	unsigned int state;
	int ret;
	struct ltc3220_platform_data *platform_data = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 0, &state);
	if (ret)
		return ret;

	ret = ltc3220_set_command(platform_data,
		platform_data->command_cfg.is_shutdown,
		platform_data->command_cfg.is_force_cpo_2x,
		platform_data->command_cfg.is_force_cpo_1p5x,
		state);
	if (ret < 0)
		return ret;

	platform_data->command_cfg.is_quick_write = state;

	return size;
}

static DEVICE_ATTR_RW(ltc3220_shutdown);
static DEVICE_ATTR_RW(ltc3220_force_cpo_2x);
static DEVICE_ATTR_RW(ltc3220_force_cpo_1p5x);
static DEVICE_ATTR_RW(ltc3220_quick_write);

static struct attribute *mode_attrs[] = {
	&dev_attr_ltc3220_shutdown.attr,
	&dev_attr_ltc3220_force_cpo_2x.attr,
	&dev_attr_ltc3220_force_cpo_1p5x.attr,
	&dev_attr_ltc3220_quick_write.attr,
	NULL
};

static int ltc3220_set_led(struct ltc3220_uled_cfg *uled, u8 led_idx,
					u8 mode, u8 current_level)
{
	int ret;
	struct i2c_client *client = uled->client;
	u8 reg_val = (mode << 6) | current_level;

	ret = ltc3220_write(client, LTC3220_ULED01 + led_idx, reg_val);
	if (ret < 0) {
		dev_err(&client->dev, "Set led fail\n");
		return ret;
	}

	return 0;
}

static int ltc3220_set_led_mode(struct led_classdev *led_cdev,
							int mode)
{
	int ret;
	int i = 0;
	struct ltc3220_platform_data *platform_data;
	struct ltc3220_uled_cfg *uled_cfg = container_of(led_cdev, struct ltc3220_uled_cfg, led_cdev);

	if (mode > 3) {
		dev_err(&uled_cfg->client->dev, "Mode set to high\n");
		return -EINVAL;
	}

	ret = ltc3220_set_led(uled_cfg, uled_cfg->led_index, mode, uled_cfg->current_level);
	if (ret < 0)
		return ret;

	platform_data = i2c_get_clientdata(uled_cfg->client);
	if (platform_data->command_cfg.is_quick_write && uled_cfg->led_index == 0) {
		for (i = 0; i < NUM_LEDS; i++)
			platform_data->uled_cfg[i].mode = mode;
	} else {
		uled_cfg->mode = mode;
	}

	return 0;
}

static int ltc3220_set_led_current_level(struct led_classdev *led_cdev,
					   enum led_brightness brightness)
{
	int ret;
	int current_level;
	int i = 0;
	struct ltc3220_platform_data *platform_data;
	struct ltc3220_uled_cfg *uled_cfg;

	current_level = (int)brightness;
	if (current_level > MAX_CURRENT)
		current_level = MAX_CURRENT;

	uled_cfg = container_of(led_cdev, struct ltc3220_uled_cfg, led_cdev);
	ret = ltc3220_set_led(uled_cfg, uled_cfg->led_index, uled_cfg->mode, current_level);
	if (ret < 0)
		return ret;

	platform_data = i2c_get_clientdata(uled_cfg->client);
	if (platform_data->command_cfg.is_quick_write && uled_cfg->led_index == 0) {
		for (i = 0; i < NUM_LEDS; i++)
			platform_data->uled_cfg[i].current_level = current_level;
	} else {
		uled_cfg->current_level = current_level;
	}
	return 0;
}

static enum led_brightness ltc3220_get_led_current_level(struct led_classdev *led_cdev)
{
	struct ltc3220_uled_cfg *uled_cfg = container_of(led_cdev, struct ltc3220_uled_cfg, led_cdev);

	return uled_cfg->current_level;
}

static ssize_t ltc3220_led_mode_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ltc3220_uled_cfg *uled_cfg = container_of(led_cdev, struct ltc3220_uled_cfg, led_cdev);

	return sysfs_emit(buf, "%d\n", uled_cfg->mode);
}

static ssize_t ltc3220_led_mode_store(struct device *dev,
					  struct device_attribute *devAttr,
					  const char *buf, size_t size)
{
	unsigned int state;
	ssize_t ret;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		return ret;

	ltc3220_set_led_mode(led_cdev, state);
	return size;
}

static DEVICE_ATTR_RW(ltc3220_led_mode);

static struct attribute *ltc3220_led_attrs[] = {
	&dev_attr_ltc3220_led_mode.attr,
	NULL
};
ATTRIBUTE_GROUPS(ltc3220_led);

static int ltc3220_set_blinking_and_gradation(struct ltc3220_platform_data *platform_data,
	bool is_blink_long_period, bool is_blink_fast_on, bool gradation_d2,
	bool gradation_d1, bool is_increasing)
{
	int ret;
	struct i2c_client *client = platform_data->client;
	u8 reg_val = (is_blink_long_period << 4) | (is_blink_fast_on << 3) |
	(gradation_d2 << 2) | (gradation_d1 << 1) | is_increasing;

	ret = ltc3220_write(client, LTC3220_GRAD_BLINK, reg_val);
	if (ret < 0) {
		dev_err(&client->dev, "Set blinking and gradation fail\n");
		return ret;
	}

	return 0;
}

static int ltc3220_set_blink(struct led_classdev *led_cdev,
					 unsigned long *delay_on,
					 unsigned long *delay_off)
{
	bool is_fast;
	bool is_long_period;
	int ret;
	struct ltc3220_platform_data *platform_data;

	platform_data = container_of(led_cdev, struct ltc3220_platform_data, led_cdev);

	if (*delay_on >= 1) {
		*delay_on = 1;
		is_long_period = true;
	} else {
		is_long_period = false;
	}

	if (*delay_off >= 1) {
		*delay_off = 1;
		is_fast = true;
	} else {
		is_fast = false;
	}

	ret = ltc3220_set_blinking_and_gradation(platform_data,
		is_long_period, is_fast,
		platform_data->grad_cfg.gradation_d2,
		platform_data->grad_cfg.gradation_d1,
		platform_data->grad_cfg.is_increasing);
	if (ret < 0)
		return ret;

	platform_data->blink_cfg.is_blink_long_period = is_long_period;
	platform_data->blink_cfg.is_blink_fast_on = is_fast;

	return 0;
}

static struct attribute_group mode_group = {
	.attrs = mode_attrs,
};

static ssize_t ltc3220_gradation_d2_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ltc3220_platform_data *platform_data = container_of(led_cdev, struct ltc3220_platform_data, led_cdev);

	return sysfs_emit(buf, "%d\n", platform_data->grad_cfg.gradation_d2);
}

static ssize_t ltc3220_gradation_d2_store(struct device *dev,
					  struct device_attribute *devAttr,
					  const char *buf, size_t size)
{
	unsigned int state;
	int ret;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ltc3220_platform_data *platform_data;

	platform_data = container_of(led_cdev, struct ltc3220_platform_data, led_cdev);
	ret = kstrtouint(buf, 0, &state);
	if (ret)
		return ret;

	ret = ltc3220_set_blinking_and_gradation(platform_data,
		platform_data->blink_cfg.is_blink_long_period,
		platform_data->blink_cfg.is_blink_fast_on,
		state,
		platform_data->grad_cfg.gradation_d1,
		platform_data->grad_cfg.is_increasing);
	if (ret < 0)
		return ret;

	platform_data->grad_cfg.gradation_d2 = state;
	return size;
}

static ssize_t ltc3220_gradation_d1_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ltc3220_platform_data *platform_data;

	platform_data = container_of(led_cdev, struct ltc3220_platform_data, led_cdev);
	return sysfs_emit(buf, "%d\n", platform_data->grad_cfg.gradation_d1);
}

static ssize_t ltc3220_gradation_d1_store(struct device *dev,
					  struct device_attribute *devAttr,
					  const char *buf, size_t size)
{
	unsigned int state;
	int ret;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ltc3220_platform_data *platform_data;

	platform_data = container_of(led_cdev, struct ltc3220_platform_data, led_cdev);
	ret = kstrtouint(buf, 0, &state);
	if (ret)
		return ret;

	ret = ltc3220_set_blinking_and_gradation(platform_data,
		platform_data->blink_cfg.is_blink_long_period,
		platform_data->blink_cfg.is_blink_fast_on,
		platform_data->grad_cfg.gradation_d2,
		state,
		platform_data->grad_cfg.is_increasing);
	if (ret < 0)
		return ret;

	platform_data->grad_cfg.gradation_d1 = state;

	return size;
}

static ssize_t ltc3220_gradation_is_increasing_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ltc3220_platform_data *platform_data;

	platform_data = container_of(led_cdev, struct ltc3220_platform_data, led_cdev);
	return sysfs_emit(buf, "%d\n", platform_data->grad_cfg.is_increasing);
}

static ssize_t ltc3220_gradation_is_increasing_store(struct device *dev,
					  struct device_attribute *devAttr,
					  const char *buf, size_t size)
{
	unsigned int state;
	int ret;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ltc3220_platform_data *platform_data;

	platform_data = container_of(led_cdev, struct ltc3220_platform_data, led_cdev);
	ret = kstrtouint(buf, 0, &state);
	if (ret)
		return ret;

	ret = ltc3220_set_blinking_and_gradation(platform_data,
		platform_data->blink_cfg.is_blink_long_period,
		platform_data->blink_cfg.is_blink_fast_on,
		platform_data->grad_cfg.gradation_d2,
		platform_data->grad_cfg.gradation_d1,
		state);
	if (ret < 0)
		return ret;

	platform_data->grad_cfg.is_increasing = state;

	return size;
}

static DEVICE_ATTR_RW(ltc3220_gradation_d2);
static DEVICE_ATTR_RW(ltc3220_gradation_d1);
static DEVICE_ATTR_RW(ltc3220_gradation_is_increasing);
static struct attribute *ltc3220_config_attrs[] = {
	&dev_attr_ltc3220_gradation_d2.attr,
	&dev_attr_ltc3220_gradation_d1.attr,
	&dev_attr_ltc3220_gradation_is_increasing.attr,
	NULL
};
ATTRIBUTE_GROUPS(ltc3220_config);

static int ltc3220_reset(struct ltc3220_platform_data *platform_data)
{
	if (platform_data->reset_gpio) {
		gpiod_set_value_cansleep(platform_data->reset_gpio, 0);
		msleep(20);
		gpiod_set_value_cansleep(platform_data->reset_gpio, 1);
	} else {
		ltc3220_set_command(platform_data, 0, 0, 0, 1);
		ltc3220_set_led_mode(&platform_data->uled_cfg[0].led_cdev, 0);
		ltc3220_set_led_current_level(&platform_data->uled_cfg[0].led_cdev, 0);
		ltc3220_set_command(platform_data, 0, 0, 0, 0);
		ltc3220_set_blinking_and_gradation(platform_data, 0, 0, 0, 0, 0);
	}

	return 0;
}

static void ltc3220_remove(struct i2c_client *client)
{
	struct ltc3220_platform_data *platform_data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &mode_group);

	if (platform_data->reset_gpio)
		gpiod_set_value(platform_data->reset_gpio, 0);
}

static int ltc3220_probe(struct i2c_client *client)
{
	u8 i = 0;
	int ret;
	struct device_node *np, *child;
	struct ltc3220_platform_data *platform_data;
	struct led_init_data init_data_config = {};

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	platform_data = devm_kzalloc(&client->dev, sizeof(*platform_data), GFP_KERNEL);
	if (!platform_data)
		return -ENODEV;

	np = dev_of_node(&client->dev);
	if (!np) {
		dev_err(&client->dev, "Device Tree Node not found\n");
		return -ENODEV;
	}

	platform_data->client = client;

	i2c_set_clientdata(client, platform_data);

	platform_data->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(platform_data->reset_gpio)) {
		dev_err(&client->dev, "Fail to set reset GPIO\n");
		return -ENOMEM;
	}

	init_data_config.fwnode = of_fwnode_handle(np);
	init_data_config.devicename = "ltc3220";
	init_data_config.default_label = ":led_config";

	platform_data->dev = &client->dev;
	platform_data->led_cdev.blink_set = ltc3220_set_blink;
	platform_data->led_cdev.default_trigger = "timer";
	platform_data->led_cdev.groups = ltc3220_config_groups;
	ret = devm_led_classdev_register_ext(&client->dev,
							&platform_data->led_cdev,
							&init_data_config);

	if (ret) {
		dev_err(&client->dev, "Fail LED class configuration register\n");
		return -ENOMEM;
	}

	for_each_available_child_of_node(np, child) {
		u32 source;
		struct led_init_data init_data = {};
		char label_buffer[32];

		ret = of_property_read_u32(child, "reg", &source);
		if (ret != 0 || !source || source > NUM_LEDS) {
			of_node_put(child);
			dev_err(&client->dev, "Couldn't read LED address: %d\n",
				ret);
			return -ENOMEM;
		}

		i = source - 1;
		init_data.fwnode = of_fwnode_handle(child);
		init_data.devicename = "ltc3220";
		snprintf(label_buffer, sizeof(label_buffer), ":led%d", i);
		init_data.default_label = label_buffer;

		platform_data->uled_cfg[i].led_index = i;
		platform_data->uled_cfg[i].client = client;
		platform_data->uled_cfg[i].current_level = 0;
		platform_data->uled_cfg[i].mode = LTC3220_LED_NORMAL;
		platform_data->uled_cfg[i].led_cdev.max_brightness = MAX_CURRENT;
		platform_data->uled_cfg[i].led_cdev.brightness_set_blocking = ltc3220_set_led_current_level;
		platform_data->uled_cfg[i].led_cdev.brightness_get = ltc3220_get_led_current_level;
		platform_data->uled_cfg[i].led_cdev.groups = ltc3220_led_groups;

		ret = devm_led_classdev_register_ext(&client->dev,
							   &platform_data->uled_cfg[i].led_cdev,
							   &init_data);
		if (ret) {
			of_node_put(child);
			dev_err(&client->dev, "Fail LED class register\n");
			return -ENOMEM;
		}
	}

	ret = sysfs_create_group(&client->dev.kobj, &mode_group);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to create mode_test sysfs group\n");
		return -ENOMEM;
	}

	ltc3220_reset(platform_data);
	return 0;
}

static const struct of_device_id ltc3220_of_match[] = {
	{ .compatible = "lltc,ltc3220", },
	{ .compatible = "lltc,ltc3220-1", },
	{ },
};
MODULE_DEVICE_TABLE(of, ltc3220_of_match);

static struct i2c_driver ltc3220_led_driver = {
	.driver		= {
		.name	= "ltc3220",
		.of_match_table = ltc3220_of_match,
	},
	.probe		= ltc3220_probe,
	.remove		= ltc3220_remove,
};

module_i2c_driver(ltc3220_led_driver);

MODULE_AUTHOR("Edelweise Escala <edelweise.escala@analog.com>");
MODULE_DESCRIPTION("LED driver for LTC3220 controllers");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-ltc3220");
