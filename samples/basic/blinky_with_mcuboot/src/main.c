/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <logging/log.h>

/* 500 msec = 0.5 sec */
#define SLEEP_TIME_MS   500


LOG_MODULE_REGISTER(blinky_with_mcuboot, CONFIG_APP_LOG_LEVEL);


static const uint8_t  led_pins[] = {DT_GPIO_PIN(DT_ALIAS(led0), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led1), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led2), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led3), gpios)};


static const struct device *led_port;

static int gpios_init(void)
{
	int err;
	
	led_port = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	if (!led_port) {
		LOG_ERR("Could not bind to LED port %s",
			DT_LABEL(DT_NODELABEL(gpio0)));
		return -EIO;
	}

	for (size_t i = 0; i < ARRAY_SIZE(led_pins); i++) {
			err = gpio_pin_configure(led_port, led_pins[i],
					GPIO_OUTPUT);
			if (err) {
					LOG_ERR("Unable to configure LED%u, err %d", i, err);
					led_port = NULL;
					return err;
				}
                      		
			gpio_pin_set(led_port, led_pins[i], 1); 
	}
	
	return 0;
	
}


void main(void)
{

	bool led_is_off = false;
	int ret;
	uint8_t i =0;

	ret = gpios_init();
	
	while (1) {

		
		gpio_pin_set(led_port, led_pins[i], (int)led_is_off);
		led_is_off = !led_is_off;
		if(!led_is_off) {
			i = (i+1)% (ARRAY_SIZE(led_pins));
		}
		
/*	
		gpio_pin_set(led_port ,led_pins[i], (int)led_is_off);
		led_is_off = !led_is_off;
*/
		k_msleep(SLEEP_TIME_MS);

	}
}
