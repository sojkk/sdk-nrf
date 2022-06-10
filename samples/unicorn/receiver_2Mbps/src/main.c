/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
*
* @defgroup nrf_dev_radio_rx_example_main main.c
* @{
* @ingroup nrf_dev_radio_rx_example
* @brief Radio Receiver example Application main file.
*
* This file contains the source code for a sample application using the NRF_RADIO peripheral.
*
*/
#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <drivers/gpio.h>
#include <hal/nrf_gpio.h>
#include <irq.h>
#include <logging/log.h>
#include <nrf.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include "radio.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);


static radio_modes_t           mode = MODE_2_MBIT;
static radio_power_t  tx_power = RADIO_TX_POWER_0DBM;
static uint8_t             data_packet[32];              /**< Packet to receive */

static radio_init_t       radio_init;	

static const uint8_t  led_pins[] = {DT_GPIO_PIN(DT_ALIAS(led0), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led1), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led2), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led3), gpios)};


static const struct device *led_port;

void radio_evt_cb(radio_evt_t const * p_event)
{
	
		if(p_event->evt_id ==RADIO_EVENT_POLL_RCV )
		{
			radio_get_packet();

			// Set LEDs identical to the ones on the PTX.
			gpio_pin_set(led_port, led_pins[0], !(data_packet[0]%8>0 && data_packet[0]%8<=4));
			gpio_pin_set(led_port, led_pins[1], !(data_packet[0]%8>1 && data_packet[0]%8<=5));
			gpio_pin_set(led_port, led_pins[2], !(data_packet[0]%8>2 && data_packet[0]%8<=6));
			gpio_pin_set(led_port, led_pins[3], !(data_packet[0]%8>3));
		
		}
}

int gpio_init( void )
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
	
    nrf_gpio_range_cfg_output(28,31);
	
	return 0;
}

/*
void power_manage(void)
{
    __WFE();
    __SEV();
    __WFE();
}
*/
/**
 * @brief Function for application main entry.
 * @return 0. int return type required by ANSI/ISO standard.
 */
void main(void)
{

    gpio_init();


	radio_init.is_transmitter 	= false;
	radio_init.mode				= mode;
	radio_init.event_callback	= radio_evt_cb;
	radio_init.tx_power			= tx_power;
	radio_init.rx_buf			= data_packet;
    radio_setup(radio_init);
    radio_receive_packet();
    
	 LOG_INF("Radio receiver example started.");

}

/**
 *@}
 **/
