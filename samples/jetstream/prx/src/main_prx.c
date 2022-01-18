/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
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
#include "nrf_esb.h"

#include <stdbool.h>
#include <stdint.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "boards.h"

#include "app_scheduler.h"
#include "app_timer.h"
#include "jetstr.h"

#include "nrf_drv_clock.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**** log count  ****/
#define PACKET_SIZE		PKT_SIZE*8
#define DATA_RATE_LOG_INTERVAL		APP_TIMER_TICKS(1000)   
APP_TIMER_DEF(m_data_rate_id);

uint16_t m_success_cnt =0;



/*****************************************************************************/
/** @name Define */
/*****************************************************************************/
#define SCHED_MAX_EVENT_DATA_SIZE       32                                         /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                10                                         /**< Maximum number of events in the scheduler queue. */

static uint8_t s_channel_tab[] = JETSTR_CHANNEL_TAB;

uint8_t led_nr;

nrf_esb_payload_t rx_payload;

/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_DEBUG("TX SUCCESS EVENT");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT");
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            NRF_LOG_DEBUG("RX RECEIVED EVENT");
            if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                // Set LEDs identical to the ones on the PTX.
                nrf_gpio_pin_write(LED_1, !(rx_payload.data[1]%8>0 && rx_payload.data[1]%8<=4));
                nrf_gpio_pin_write(LED_2, !(rx_payload.data[1]%8>1 && rx_payload.data[1]%8<=5));
                nrf_gpio_pin_write(LED_3, !(rx_payload.data[1]%8>2 && rx_payload.data[1]%8<=6));
                nrf_gpio_pin_write(LED_4, !(rx_payload.data[1]%8>3));

                NRF_LOG_DEBUG("Receiving packet: %02x", rx_payload.data[1]);
            }
            break;
    }
}





void gpio_init( void )
{
    bsp_board_init(BSP_INIT_LEDS);
	
    	
}




/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

void jetstr_event_handler(void * p_event_data, uint16_t event_size)
{ 
     jetstr_evt_t * evt;
	
     ASSERT(event_size == sizeof(jetstr_evt_t));
	
     evt = (jetstr_evt_t *) p_event_data;

     switch (evt->type)
     {
	case jetstr_evt_none:
	case jetstr_evt_tx_success:
	case jetstr_evt_tx_failed:
         break;
		
	case jetstr_evt_rx_received:
		
				m_success_cnt++;

				// Set LEDs identical to the ones on the PTX.
				nrf_gpio_pin_write(LED_1, !(evt->rcv_data[1]%8>0 && evt->rcv_data[1]%8<=4));
				nrf_gpio_pin_write(LED_2, !(evt->rcv_data[1]%8>1 && evt->rcv_data[1]%8<=5));
				nrf_gpio_pin_write(LED_3, !(evt->rcv_data[1]%8>2 && evt->rcv_data[1]%8<=6));
				nrf_gpio_pin_write(LED_4, !(evt->rcv_data[1]%8>3));
	
        break;                
     }	


}

void radio_init(void)
{

    jetstr_cfg_t jetstr_config;
	
		jetstr_cfg_params_t jetstr_cfg_params;
   
		jetstr_cfg_params.jetstr_channel_tab            = s_channel_tab;
		jetstr_cfg_params.jetstr_channel_tab_size       = JETSTR_CHANNEL_TAB_SIZE;
		jetstr_cfg_params.jetstr_rx_period              = JETSTR_RX_PERIOD;
		jetstr_cfg_params.jetstr_rx_delay               = JETSTR_RX_DELAY;
		jetstr_cfg_params.jetstr_retran_cnt_in_sync     = JETSTR_RETRAN_CNT_IN_SYNC;    
		jetstr_cfg_params.jetstr_retran_cnt_out_of_sync = JETSTR_RETRAN_CNT_OUT_OF_SYNC;
		jetstr_cfg_params.jetsr_rx_retran               = JETSTR_RX_RETRAN;	
	
    jetstr_config.mode            = NRF_ESB_MODE_PRX;
    jetstr_config.event_callback  = jetstr_event_handler;
  
    jetstr_init(&jetstr_config, &jetstr_cfg_params);  
}	


static void data_rate_timeout_handler(void * p_context)
{
	 uint16_t data_rate;
	
    UNUSED_PARAMETER(p_context);
	
		data_rate = m_success_cnt * PACKET_SIZE  / 1000;
	
		NRF_LOG_INFO("Success packet cnt %d, data rate = %d kbps", m_success_cnt, data_rate);

		m_success_cnt =0;
	
}	


void timers_init(void)
{
	app_timer_init();
	
	app_timer_create(&m_data_rate_id, APP_TIMER_MODE_REPEATED, data_rate_timeout_handler);
	
}


static void clocks_start( void )
{
  uint32_t err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);
  nrf_drv_clock_lfclk_request(NULL);
}

int main(void)
{
    uint32_t err_code;

    gpio_init();
	
		clocks_start();
	
		err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
	
		timers_init();
	
		scheduler_init();

		radio_init();
	
		nrf_esb_set_tx_power(NRF_ESB_TX_POWER_0DBM);  //JS MOdify: 2019/8/2 USB3.0 testing

    NRF_LOG_INFO("Enhanced ShockBurst Receiver Example running.\r\n");

    jetstr_rx_start(JETSTR_RX_PERIOD);
		
		app_timer_start(m_data_rate_id, DATA_RATE_LOG_INTERVAL, NULL);
	
    while (true)
    {
				 app_sched_execute(); 
			
        if (NRF_LOG_PROCESS() == false)
        {
            __WFE();
        }
    }
}
/*lint -restore */
