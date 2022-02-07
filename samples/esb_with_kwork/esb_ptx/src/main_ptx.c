/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <drivers/gpio.h>
#include <irq.h>
#include <logging/log.h>
#include <nrf.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <hal/nrf_gpio.h>

#include <fast_esb.h>



#define DATA_SENDING_P0   3
#define DATA_SENDING_P1   4
#define DATA_SENDING_P2  28
#define DATA_SENDING_P3	 29
#define DATA_SENDING_P4	 30
#define DATA_SENDING_P5	 31

//LOG_MODULE_REGISTER(main_ptx, CONFIG_APP_LOG_LEVEL);

static struct k_work 			pkt_tx_work;

static struct esb_payload        tx_payload; 

uint8_t data_packet[] = {   1,  0,  3,  4,  5,  6,  7,  8,
							9, 10, 11, 12, 13, 14, 15, 16,
							17, 18, 19, 20, 21, 22, 23, 24,
							25, 26, 27, 28, 29, 30, 31, 32,
							33, 34, 35, 36, 37, 38, 39, 40,
							41, 42, 43, 44, 45, 46, 47, 48,
							49, 50, 51, 52, 53, 54, 55, 56,
							57, 58, 59, 60, 61, 62, 63, 64 };


static struct esb_payload        rx_payload;


static const struct device *led_port;

static const uint8_t  led_pins[] = {DT_GPIO_PIN(DT_ALIAS(led0), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led1), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led2), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led3), gpios)};
													 
													 
static void pkt_tx_handler(struct k_work *item)
{

	if  (esb_write_payload(&tx_payload) == 0) 
		{
			
			nrf_gpio_pin_set(DATA_SENDING_P0);
		
			// Toggle one of the LEDs.
			gpio_pin_set(led_port, led_pins[0], !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
			gpio_pin_set(led_port, led_pins[1], !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
			gpio_pin_set(led_port, led_pins[2], !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
			gpio_pin_set(led_port, led_pins[3], !(tx_payload.data[1]%8>3));
			tx_payload.data[1]++;
		
			//nrf_gpio_pin_clear(DATA_SENDING_P0);	
						
		}

	
}	
													 
													 

void radio_event_handler(struct esb_evt const * p_event)
{

    switch (p_event->evt_id)
    {
			
		case ESB_EVENT_TX_FAILED:
             
			//LOG_DBG("TX FAILED EVENT");
						
            (void)esb_flush_tx();
   
        case ESB_EVENT_TX_SUCCESS:
		
			nrf_gpio_pin_clear(DATA_SENDING_P0);	
            
			//LOG_DBG("TX SUCCESS EVENT");
			
			k_work_submit(&pkt_tx_work);
			
			nrf_gpio_pin_toggle(DATA_SENDING_P1);

            break;
			
        case ESB_EVENT_RX_RECEIVED:
		
            //LOG_DBG("RX RECEIVED EVENT");
            while (esb_read_rx_payload(&rx_payload) == 0)
            {
                if (rx_payload.length > 0)
                {
                    //LOG_DBG("RX RECEIVED PAYLOAD");
                }
            }
            break;
    }
}


void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


int gpio_init( void )
{
	
	int err;
	
	led_port = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
	if (!led_port) {
		//LOG_ERR("Could not bind to LED port %s",
		return -EIO;
	}

	for (size_t i = 0; i < ARRAY_SIZE(led_pins); i++) {
		err = gpio_pin_configure(led_port, led_pins[i], GPIO_OUTPUT);
		if (err) {
				//LOG_ERR("Unable to configure LED%u, err %d", i, err);
				led_port = NULL;
				return err;
			}
                      		
		gpio_pin_set(led_port, led_pins[i], 1); 
	}
	

	nrf_gpio_cfg_output(DATA_SENDING_P0);
	nrf_gpio_cfg_output(DATA_SENDING_P1);
	nrf_gpio_cfg_output(DATA_SENDING_P2);
	nrf_gpio_cfg_output(DATA_SENDING_P3);
	nrf_gpio_cfg_output(DATA_SENDING_P4);
	nrf_gpio_cfg_output(DATA_SENDING_P5);
	nrf_gpio_pin_clear(DATA_SENDING_P0);
	nrf_gpio_pin_clear(DATA_SENDING_P1);
	nrf_gpio_pin_clear(DATA_SENDING_P2);
	nrf_gpio_pin_clear(DATA_SENDING_P3);
	nrf_gpio_pin_clear(DATA_SENDING_P4);
	nrf_gpio_pin_clear(DATA_SENDING_P5);	
	
	return 0;
}


uint32_t radio_init( void )
{
    int err;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

	struct esb_config config = ESB_DEFAULT_CONFIG;
   
	config.protocol                 = ESB_PROTOCOL_ESB_DPL;
    config.bitrate                  = ESB_BITRATE_2MBPS;
    config.event_handler            = radio_event_handler;
    config.mode                     = ESB_MODE_PTX;
    config.selective_auto_ack       = false;
	config.retransmit_delay         = 250;
	config.payload_length			= 64;

    err = esb_init(&config);
	if (err) {
		return err;
	}
   
    err = esb_set_base_address_0(base_addr_0);
    if (err) {
		return err;
	}

    err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

    err = esb_set_prefixes(addr_prefix, sizeof(addr_prefix));
	if (err) {
		return err;
	}

    return 0;
}



int main(void)
{
   int err;

	k_work_init(&pkt_tx_work, pkt_tx_handler);

    err = gpio_init();
/*
	if(err)
	{
		LOG_ERR("Error on gpio_init!!");
	}
*/

    clocks_start();
	


    err = radio_init();
/*	
	if(err)
	{
		LOG_ERR("Error on radio_init!!");
	}
*/

	memcpy(tx_payload.data, data_packet, sizeof(data_packet));
	tx_payload.noack = false;
	tx_payload.length=64;
	
		
	k_work_submit(&pkt_tx_work);
	
    //LOG_DBG("Enhanced ShockBurst Transmitter Example started.");

    while (true)
    {
       k_sleep(K_MSEC(100)); 

    }
}
