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

#include "esb_k.h"
#include "jetstr.h"

LOG_MODULE_REGISTER(esb_ptx, CONFIG_APP_LOG_LEVEL);


/*****************************************************************************/
/** @name Define */
/*****************************************************************************/
#define SCHED_MAX_EVENT_DATA_SIZE       32                                         /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                10                                         /**< Maximum number of events in the scheduler queue. */
#define DATA_SENDING_P0   3
#define DATA_SENDING_P1   4
#define DATA_SENDING_P2  28
#define DATA_SENDING_P3	 29
#define DATA_SENDING_P4	 30
#define DATA_SENDING_P5	 31

static struct k_work pkt_tx_work;

static struct esb_payload        tx_payload; 

uint8_t data_packet[] = {	1,  0,  3,  4,  5,  6,  7,  8,
							9, 10, 11, 12, 13, 14, 15, 16,
							17, 18, 19, 20, 21, 22, 23, 24,
							25, 26, 27, 28, 29, 30, 31, 32,
							33, 34, 35, 36, 37, 38, 39, 40,
							41, 42, 43, 44, 45, 46, 47, 48,
							49, 50, 51, 52, 53, 54, 55, 56,
							57, 58, 59, 60, 61, 62, 63, 64,
							1,  2,  3,  4,  5,  6,  7,  8,
							9, 10, 11, 12, 13, 14, 15, 16,
							17, 18, 19, 20, 21, 22, 23, 24,
							25, 26, 27, 28, 29, 30, 31, 32,
							33, 34, 35, 36, 37, 38, 39, 40,
							41, 42, 43, 44, 45, 46, 47, 48,
							49, 50, 51, 52, 53, 54, 55, 56,
							57, 58, 59, 60, 61, 62, 63, 64, 65
						};

static uint8_t s_channel_tab[] = JETSTR_CHANNEL_TAB;

static bool ready = true;

static const uint8_t  led_pins[] = {DT_GPIO_PIN(DT_ALIAS(led0), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led1), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led2), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led3), gpios)};


static const struct device *led_port;



int clocks_start(void)
{	
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}



static int leds_init(void)
{
	int err;
	
	led_port = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
	if (!led_port) {
		LOG_ERR("Could not bind to LED port %s",
			DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
		return -EIO;
	}

	for (size_t i = 0; i < ARRAY_SIZE(led_pins); i++) {
		err = gpio_pin_configure(led_port, led_pins[i], GPIO_OUTPUT);
		if (err) {
				LOG_ERR("Unable to configure LED%u, err %d", i, err);
				led_port = NULL;
				return err;
			}
                      		
		gpio_pin_set(led_port, led_pins[i], 1); 
	}

	return 0;
}


static void pkt_tx_handler(void * p_event_data, uint16_t event_size)
{

	if  (esb_write_payload(&tx_payload) == 0) 
	{
#ifdef DBG_SIG_ENABLE	
		nrf_gpio_pin_set(DBG_SIG_POLL_EXP);  
#endif							
								

	// Toggle one of the LEDs.
		gpio_pin_set(led_port, led_pins[0], !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
		gpio_pin_set(led_port, led_pins[1], !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
		gpio_pin_set(led_port, led_pins[2], !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
		gpio_pin_set(led_port, led_pins[3], !(tx_payload.data[1]%8>3));
		tx_payload.data[1]++;

#ifdef DBG_SIG_ENABLE	
		nrf_gpio_pin_clear(DBG_SIG_POLL_EXP);  
#endif										
								
										
	}

	
}

void jetstr_event_handler(struct jetstr_evt const * p_event_data)
{  
	//uint16_t success_rate;
	
	ret_code_t err_code;
	
	jetstr_evt_t * evt;
	
	
	evt = (jetstr_evt_t *) p_event_data;
	
	
	switch (evt->type)
	{
		case jetstr_evt_none:
			
			break;
		
		case jetstr_evt_tx_failed:

			
		case jetstr_evt_tx_success:

			k_work_submit(&pkt_tx_work);
		
		break;
			
		
		case jetstr_evt_rx_received:
			
		
		break;
		
		
	}	
	
	
}



void main(void)
{
	int err;
	
	jetstr_cfg_t jetstr_config;
	
	jetstr_cfg_params_t jetstr_cfg_params;

	jetstr_cfg_params.jetstr_channel_tab            = s_channel_tab;
	jetstr_cfg_params.jetstr_channel_tab_size       = JETSTR_CHANNEL_TAB_SIZE;
	jetstr_cfg_params.jetstr_rx_period              = JETSTR_RX_PERIOD;
	jetstr_cfg_params.jetstr_rx_delay               = JETSTR_RX_DELAY;
	jetstr_cfg_params.jetstr_retran_cnt_in_sync     = JETSTR_RETRAN_CNT_IN_SYNC;   
	jetstr_cfg_params.jetstr_retran_cnt_out_of_sync = JETSTR_RETRAN_CNT_OUT_OF_SYNC;
	jetstr_cfg_params.jetsr_rx_retran               = JETSTR_RX_RETRAN;
	jetstr_cfg_params.jetstr_retran_cnt_chan_sw		= JETSTR_RETRAN_CNT_CHAN_SW;
	

	LOG_INF("Enhanced ShockBurst ptx sample");

	err = clocks_start();
	if (err) {
		return;
	}

	leds_init();

	k_work_init(&pkt_tx_work, pkt_tx_handler);
	
	jetstr_config.mode            = NRF_ESB_MODE_PTX;
	jetstr_config.event_callback  = jetstr_event_handler;
	jetstr_init(&jetstr_config, &jetstr_cfg_params);
	
	tx_payload.noack = false;
	tx_payload.length= PKT_SIZE+1;
	tx_payload.pipe =  NRFR_DATA_PIPE;
	
	k_work_submit(&pkt_tx_work);
	
	
	while (1) {

		k_sleep(K_MSEC(100));
	}
}
