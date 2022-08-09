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
* @defgroup nrf_dev_button_radio_tx_example_main main.c
* @{
* @ingroup nrf_dev_button_radio_tx_example
*
* @brief Radio Transceiver Example Application main file.
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
#include "radio_config.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static const uint8_t  led_pins[] = {DT_GPIO_PIN(DT_ALIAS(led0), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led1), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led2), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led3), gpios)};

static const struct device *led_port;

static radio_modes_t mode = MODE_2_MBIT;

#if defined(CONFIG_NRF5340)
static radio_power_t tx_power = RADIO_TX_POWER_3DBM;
#else
static radio_power_t tx_power = RADIO_TX_POWER_4DBM;
#endif

//32 bytes packet
static uint8_t      	tx_packet[]   = { 1,  2,  3,  4,  5,  6,  7,  8, 
                                          9, 10, 11, 12, 13, 14, 15, 16,
                                          17, 18, 19, 20, 21, 22, 23, 24,
                                          25, 26, 27, 28, 29, 30, 31, 32} ;                    /**< Packet to transmit. */


static uint8_t			rx_packet[NUM_OF_PERIPHS][PERIPH_PKT_SIZE];			


static radio_init_t     radio_init;																									 
																					 


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
		err = gpio_pin_configure(led_port, led_pins[i], GPIO_OUTPUT);
		if (err){
					LOG_ERR("Unable to configure LED%u, err %d", i, err);
					led_port = NULL;
					return err;
				}
                      		
		gpio_pin_set(led_port, led_pins[i], 1); 
	}

    #if defined(CONFIG_NRF5340)
	nrf_gpio_cfg_output(7);
	nrf_gpio_cfg_output(25);
	nrf_gpio_cfg_output(26);
	#else
    nrf_gpio_range_cfg_output(28,31);
	#endif
	
	return 0;

}

void radio_evt_cb(radio_evt_t const * p_event)
{
   if(p_event->evt_id == RADIO_EVENT_CENTRAL_POLL_EXP)
	 {	

		uint8_t cnt = p_event->chan_cnt;
		 
		cnt = cnt %4; 

		if (cnt == 1)
		{
			// Toggle one of the LEDs.
			gpio_pin_set(led_port, led_pins[0], !(rx_packet[0][0]%8>0 && rx_packet[0][0]%8<=4));
			gpio_pin_set(led_port, led_pins[1], !(rx_packet[1][0]%8>1 && rx_packet[1][0]%8<=5));
			//gpio_pin_set(led_port, led_pins[2], !(rx_packet[2][0]%8>2 && rx_packet[2][0]%8<=6));
			//gpio_pin_set(led_port, led_pins[3], !(rx_packet[3][0]%8>3));
			tx_packet[0]++;
		}
		
			
	}
}


/**
 * @brief Function for application main entry.
 * 
 */
void  main(void)
{

	//+3dBm for nRF5340
#if defined(CONFIG_NRF5340)
	if( tx_power == RADIO_TX_POWER_3DBM)
	{
		NRF_VREQCTRL->VREGRADIO.VREQH=1;
		while (NRF_VREQCTRL->VREGRADIO.VREQHREADY==false)
	  		;
	}	
#endif

    gpio_init();


	radio_init.is_central		= true;
	radio_init.mode				= mode;
	radio_init.event_callback	= radio_evt_cb;
	radio_init.tx_power			= tx_power;
	radio_init.tx_buf			= tx_packet;
	radio_init.rx_buf			= &rx_packet[0][0];
	radio_init.tx_length		= sizeof(tx_packet);
 
    radio_setup(radio_init);
	
	radio_start_poll();
   
    LOG_INF("Radio transmitter example started.");

}

