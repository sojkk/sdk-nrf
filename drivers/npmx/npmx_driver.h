/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_NPMX_NPMX_DRIVER_H__
#define ZEPHYR_DRIVERS_NPMX_NPMX_DRIVER_H__

#include <npmx_core.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

struct npmx_data {
	const struct device *dev;
	npmx_instance_t npmx_instance;
	struct k_work work;
	struct gpio_callback gpio_cb;
};

struct npmx_config {
	const struct i2c_dt_spec i2c;
	const struct gpio_dt_spec int_gpio;
};

#endif /* ZEPHYR_DRIVERS_NPMX_NPMX_DRIVER_H__ */
