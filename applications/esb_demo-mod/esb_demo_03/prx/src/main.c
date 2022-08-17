/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <drivers/gpio.h>
#include <irq.h>
#include <logging/log.h>
#include <nrf.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include "radio.h"
#include "radio_config.h"

LOG_MODULE_REGISTER(esb_prx, CONFIG_APP_LOG_LEVEL);

#define LED_ON  0
#define LED_OFF 1

#define DT_DRV_COMPAT nordic_nrf_clock

static const struct device *led_port;

#define PERIPH_NUM   1//1..2

#define	PAYLOAD_SIZE	64  //bytes

static radio_data_t data_send;

uint8_t data_packet[] = {	1,  0,  3,  4,  5,  6,  7,  8,
							9, 10, 11, 12, 13, 14, 15, 16,
							17, 18, 19, 20, 21, 22, 23, 24,
							25, 26, 27, 28, 29, 30, 31, 32,
							33, 34, 35, 36, 37, 38, 39, 40,
							41, 42, 43, 44, 45, 46, 47, 48,
							49, 50, 51, 52, 53, 54, 55, 56,
							57, 58, 59, 60, 61, 62, 63, 64, 
							65, 66, 67, 68, 69, 70, 71, 72,
						   };


static int leds_init(void)
{
	led_port = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
	if (!led_port) {
		LOG_ERR("Could not bind to LED port %s",
			DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
		return -EIO;
	}

	const uint8_t pins[] = {DT_GPIO_PIN(DT_ALIAS(led0), gpios),
			     DT_GPIO_PIN(DT_ALIAS(led1), gpios),
			     DT_GPIO_PIN(DT_ALIAS(led2), gpios),
			     DT_GPIO_PIN(DT_ALIAS(led3), gpios)};

	for (size_t i = 0; i < ARRAY_SIZE(pins); i++) {
		int err = gpio_pin_configure(led_port, pins[i], GPIO_OUTPUT);

		if (err) {
			LOG_ERR("Unable to configure LED%u, err %d.", i, err);
			led_port = NULL;
			return err;
		}

                gpio_pin_set(led_port, pins[i], 1);
	}

	return 0;
}

static void leds_update(uint8_t value)
{
	bool led0_status = !(value % 8 > 0 && value % 8 <= 4);
	bool led1_status = !(value % 8 > 1 && value % 8 <= 5);
	bool led2_status = !(value % 8 > 2 && value % 8 <= 6);
	bool led3_status = !(value % 8 > 3);

	gpio_port_pins_t mask =
		1 << DT_GPIO_PIN(DT_ALIAS(led0), gpios) |
		1 << DT_GPIO_PIN(DT_ALIAS(led1), gpios) |
		1 << DT_GPIO_PIN(DT_ALIAS(led2), gpios) |
		1 << DT_GPIO_PIN(DT_ALIAS(led3), gpios);

	gpio_port_value_t val =
		led0_status << DT_GPIO_PIN(DT_ALIAS(led0), gpios) |
		led1_status << DT_GPIO_PIN(DT_ALIAS(led1), gpios) |
		led2_status << DT_GPIO_PIN(DT_ALIAS(led2), gpios) |
		led3_status << DT_GPIO_PIN(DT_ALIAS(led3), gpios);

	if (led_port != NULL) {
		gpio_port_set_masked_raw(led_port, mask, val);
	}
}



void lf_clock_start(void)
{

#if defined(CONFIG_SOC_SERIES_NRF53X)
	NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_LFRC;
#else
	NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC;
#endif

  //Start LFCLK
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);

}


void radio_evt_cb(uint8_t radio_event)
{
	if(radio_event == RADIO_PERIPH_DATA_SENT)
	{
		
              // Set LEDs identical to the ones on the PTX.
              leds_update(data_send.data[1]);

              data_send.data[1]++;
		
              radio_put_packet(&data_send);
		
	}

}


void data_init(void)
{
	data_send.length = PAYLOAD_SIZE;
	data_send.periph_num = PERIPH_NUM;
	memcpy(data_send.data, data_packet, PAYLOAD_SIZE);	
}


void main(void)
{

	LOG_INF("Enhanced ShockBurst prx sample");

	lf_clock_start();

	leds_init();     

        data_init();

        radio_setup(false, RADIO_TX_POWER_0DBM, radio_evt_cb, PERIPH_NUM);

       // radio_put_packet(&data_send);
	
        radio_scan_for_poll();

	
	while (1) {
		/* do nothing */
	}
}
