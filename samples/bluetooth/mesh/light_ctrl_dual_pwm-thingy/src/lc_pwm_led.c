/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <bluetooth/mesh/models.h>
#include "model_handler.h"
#include <drivers/gpio.h>
#include <drivers/gpio/gpio_sx1509b.h>


#define NUMBER_OF_LEDS 3
#define RED_LED DT_GPIO_PIN(DT_NODELABEL(led0), gpios)
#define GREEN_LED DT_GPIO_PIN(DT_NODELABEL(led1), gpios)
#define BLUE_LED DT_GPIO_PIN(DT_NODELABEL(led2), gpios)

enum sx1509b_color { sx1509b_red = 0, sx1509b_green, sx1509b_blue };


const struct device *sx1509b_dev;


static const gpio_pin_t rgb_pins[] = {
	RED_LED,
	GREEN_LED,
	BLUE_LED,
};



void lc_pwm_led_init(void)
{	
	int err;


	sx1509b_dev = device_get_binding(DT_PROP(DT_NODELABEL(sx1509b), label));
	
	if (sx1509b_dev == NULL) {
		printk("Error binding SX1509B device\n");

		return;
	}

	for (int i = 0; i < NUMBER_OF_LEDS; i++) {
		err = sx1509b_led_intensity_pin_configure(sx1509b_dev,
							  rgb_pins[i]);

		if (err) {
			printk("Error configuring pin for LED intensity\n");
		}
	}

	
}

void lc_pwm_led_set(uint8_t dev_num, uint16_t desired_lvl)
{
	/* desired_lvl: 0 -BT_MESH_LIGHTNESS_MAX
		scaled_lvl: 0 -255 */

	uint32_t scaled_lvl =
		(255 * desired_lvl) /
		BT_MESH_LIGHTNESS_MAX;

   switch (dev_num)
   {
	   
		case 0:   

			sx1509b_led_intensity_pin_set(sx1509b_dev, RED_LED, scaled_lvl);
			
		break;
		
		case 1:   

			sx1509b_led_intensity_pin_set(sx1509b_dev, GREEN_LED, scaled_lvl);
			
		break;

   }
}
