/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * LTC3220 18-Channel LED Driver Data Header
 *
 * Copyright 2025 Analog Devices Inc.
 *
 * Author: Edelweise Escala <edelweise.escala@analog.com>
 */

#ifndef __LINUX_I2C_LTC3220_H
#define __LINUX_I2C_LTC3220_H

#define NUM_LEDS 18

#define MAX_CURRENT 64

/*
 * LEDs subdevice platform data
 */
enum ltc3220_uled_mode {
	LTC3220_LED_NORMAL = 0,
	LTC3220_LED_BLINK = 1,
	LTC3220_LED_GRADATION = 2,
	LTC3220_LED_GPO	 = 3
};

struct ltc3220_command_cfg {
	bool is_quick_write;
	bool is_force_cpo_1p5x;
	bool is_force_cpo_2x;
	bool is_shutdown;
};

struct ltc3220_uled_cfg {
	enum ltc3220_uled_mode mode;
	u8 current_level;
	u8 led_index;
	struct i2c_client	*client;
	struct led_classdev led_cdev;
};

struct ltc3220_blink_cfg {
	bool is_blink_long_period;
	bool is_blink_fast_on;
};

struct ltc3220_grad_cfg {
	bool gradation_d2;
	bool gradation_d1;
	bool is_increasing;
};

struct ltc3220_platform_data {
	struct i2c_client	*client;
	struct ltc3220_command_cfg command_cfg;
	struct ltc3220_uled_cfg uled_cfg[NUM_LEDS];
	struct ltc3220_blink_cfg blink_cfg;
	struct ltc3220_grad_cfg grad_cfg;
	struct led_classdev led_cdev;
	struct device *dev;
	struct gpio_desc *reset_gpio;
};

#endif /* __LINUX_I2C_LTC3220_H */
