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
	radio_data_t rx_payload;
	
	if(radio_event == RADIO_PERIPH_BCT_RECEIVED)
	{
		//For test purpose only
		//gpio_pin_set(led_port,  DT_GPIO_PIN(DT_ALIAS(led1), gpios), 1);
		
		radio_fetch_packet(&rx_payload);

        leds_update(rx_payload.data[1]);     

		LOG_INF("data = %d, %d, %d", rx_payload.data[0], rx_payload.data[1], rx_payload.data[2]);
		
		LOG_INF("length = %d, periph_num = %d",  rx_payload.length, rx_payload.periph_num);
		
		//For test purpose only
		//gpio_pin_set(led_port,  DT_GPIO_PIN(DT_ALIAS(led1), gpios), 0);
	}

}




void main(void)
{
	
	LOG_INF("Enhanced ShockBurst brx sample");

	lf_clock_start();

	leds_init();     

       

    radio_setup(false, RADIO_TX_POWER_0DBM, radio_evt_cb, BCT_PERIPH_NUM);
	
    radio_scan_for_poll();

	
	while (1) {
		/* do nothing */
	}
}
