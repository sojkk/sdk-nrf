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

LOG_MODULE_REGISTER(esb_ptx, CONFIG_APP_LOG_LEVEL);

#define LED_ON    0
#define LED_OFF   1

#define DT_DRV_COMPAT nordic_nrf_clock

#define	BROADCAST_SIZE	94  //bytes

static const uint8_t  led_pins[] = {DT_GPIO_PIN(DT_ALIAS(led0), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led1), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led2), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led3), gpios)};


static const struct device *led_port;

static radio_data_t bct_send;

uint8_t broadcast_packet[] = { 
							  100, 99, 98, 97, 96, 95, 94, 93, 92, 91, 
							   90, 89, 88, 87, 86, 85, 84, 83, 82, 81,
							   80, 79, 78, 77, 76, 75, 74, 73, 72, 71,
							   70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 
                               60, 59, 58, 57, 56, 55, 54, 53, 52, 51,
                               50, 49, 48, 47, 46, 45, 44, 43, 42, 41,
                               40, 39, 38, 37, 36, 35, 34, 33, 32, 31,
                               30, 29, 28, 27, 26, 25, 24, 23, 22, 21,
                               20, 19, 18, 17, 16, 15, 14, 13, 12, 11,
                               10,  9,  8,  7,  6,  5,  4,  3,  2,  1 };

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


static void leds_update(uint8_t value)
{
	bool led0_status = !(value % 8 > 0 && value % 8 <= 4);
	bool led1_status = !(value % 8 > 1 && value % 8 <= 5);
	bool led2_status = !(value % 8 > 2 && value % 8 <= 6);
	bool led3_status = !(value % 8 > 3);

	gpio_port_pins_t mask =
		1 << led_pins[0] |
		1 << led_pins[1] |
		1 << led_pins[2] |
		1 << led_pins[3];

	gpio_port_value_t val =
		led0_status << led_pins[0] |
		led1_status << led_pins[1] |
		led2_status << led_pins[2] |
		led3_status << led_pins[3];

	if (led_port != NULL) {
		(void)gpio_port_set_masked_raw(led_port, mask, val);
	}
}


void radio_evt_cb(uint8_t radio_event)
{
	radio_data_t rx_payload;

	if(radio_event==RADIO_CENTRAL_DATA_RECEIVED)
	{

            radio_fetch_packet(&rx_payload);

            leds_update(rx_payload.data[1]);

	}
	else if (radio_event== RADIO_CENTRAL_BCT_SENT)
	{
		bct_send.data[1]++;
		bct_send.periph_num = 2;
		radio_put_bct_packet(&bct_send);
	}
}

void data_init(void)
{
	bct_send.length = BROADCAST_SIZE;
	memcpy(bct_send.data, broadcast_packet, BROADCAST_SIZE);	
}

void main(void)
{

	LOG_INF("Enhanced ShockBurst ptx sample");

	lf_clock_start();
	
	gpios_init();
	
	data_init();
	
      //radio_setup(true, RADIO_TX_POWER_4DBM, radio_evt_cb, 0);

        radio_setup(true, RADIO_TX_POWER_0DBM, radio_evt_cb, 0);
		
		radio_put_bct_packet(&bct_send);
          
        radio_poll_timer_start(POLL_TICKS);

        LOG_INF("Central PTX started...");

	while (1) {
		
              /* do nothing */
	}
}
