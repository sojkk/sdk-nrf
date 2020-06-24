/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>


#define LEDPORT	DT_LABEL(DT_NODELABEL(gpio0))
#define LED1	DT_GPIO_PIN(DT_ALIAS(led0), gpios)
#define LED2	DT_GPIO_PIN(DT_ALIAS(led1), gpios)
#define LED3	DT_GPIO_PIN(DT_ALIAS(led2), gpios)
#define LED4	DT_GPIO_PIN(DT_ALIAS(led3), gpios)

void main(void)
{
	struct device *led_port;
	bool led_is_on = true;
	int ret;

	led_port = device_get_binding(LEDPORT);
	if (led_port == NULL) {
		return;
	}

	ret = gpio_pin_configure(led_port, LED1, GPIO_OUTPUT);
	if (ret < 0) {
		return;
	}
	
	ret = gpio_pin_configure(led_port, LED2, GPIO_OUTPUT);
	if (ret < 0) {
		return;
	}
	
	ret = gpio_pin_configure(led_port, LED3, GPIO_OUTPUT);
	if (ret < 0) {
		return;
	}
	
	ret = gpio_pin_configure(led_port, LED4, GPIO_OUTPUT);
	if (ret < 0) {
		return;
	}

	while (1) {
		gpio_pin_set(led_port, LED1, (int)led_is_on);
		gpio_pin_set(led_port, LED2, (int)led_is_on);
		gpio_pin_set(led_port, LED3, (int)led_is_on);
		gpio_pin_set(led_port, LED4, (int)led_is_on);
		led_is_on = !led_is_on;
		k_sleep(K_SECONDS(5));
	}
}
