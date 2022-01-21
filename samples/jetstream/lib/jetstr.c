/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
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

#include "esb_k.h"
#include "jetstr.h"
#include "jetstr_config.h"
#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <drivers/gpio.h>
#include <irq.h>
#include <logging/log.h>
#include <string.h>
#include <nrf.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <hal/nrf_gpio.h>



static uint8_t pipe0_addr[] = SYSTEM_ADDRESS;

static bool first_rnd_sw = false;
static uint8_t chan_sw_cnt = 0;

static uint8_t channel_cnt;

// Debug helper variables
static volatile uint32_t push_ok;

static jetstr_cfg_params_t m_jetstr_cfg_params;

static uint8_t  rf_channel_tab[5];

static uint8_t max_channel_attempts_before_discard; 

#ifdef DBG_SIG_ENABLE
uint8_t dbg_sig_rf_channel[] = DBG_SIG_RF_CHANNEL_OUT;
#endif

static uint8_t channel_attempts = 0;

static uint8_t jetstr_channel_cnt = 0;

static jetstr_evt_callback_t    jetstr_event_callback;


static void nrf_esb_ptx_event_handler(nrf_esb_evt_t const * p_event)
{
    jetstr_evt_t evt;
	 
    evt.type = jetstr_evt_none;
	 
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:					 
             evt.type = jetstr_evt_tx_success;
             nrf_esb_set_retransmit_count(m_jetstr_cfg_params.jetstr_retran_cnt_in_sync);
#ifdef DBG_SIG_ENABLE						
            nrf_gpio_pin_set(DBG_SIG_TX_SUCCESS);
#endif				
//	 (void) nrf_esb_flush_tx();    //JS Modify: 1/3/18,  Remove 
	    channel_attempts = 0;
#ifdef DBG_SIG_ENABLE					
            nrf_gpio_pin_clear(DBG_SIG_TX_SUCCESS);		
#endif				
//     if (!nrf_gpio_pin_read(BSP_LED_1))  nrf_gpio_pin_set(BSP_LED_1);
				
            break;
        case NRF_ESB_EVENT_TX_FAILED:					
					  
           channel_cnt= (channel_cnt+1)% m_jetstr_cfg_params.jetstr_channel_tab_size;		
           channel_attempts++;
           if (channel_attempts < max_channel_attempts_before_discard)    // retransamit at next radio channels
	   {	
							
#ifdef DBG_SIG_ENABLE								
            nrf_gpio_pin_set(DBG_SIG_RETX);
              
#endif	    
            (void) nrf_esb_set_retransmit_count(m_jetstr_cfg_params.jetstr_retran_cnt_chan_sw);										
            (void) nrf_esb_set_rf_channel(rf_channel_tab[channel_cnt]);
            (void) nrf_esb_start_tx();
							
#ifdef DBG_SIG_ENABLE								
             nrf_gpio_pin_clear(DBG_SIG_RETX);
#endif			
              return;							
           }
           else // declare fail!!
           {
							
               evt.type = jetstr_evt_tx_failed;
							
#ifdef DBG_SIG_ENABLE								
            nrf_gpio_pin_set(DBG_SIG_TX_FAIL);
#endif
							
              //Discard transmission 
              (void) nrf_esb_set_retransmit_count(m_jetstr_cfg_params.jetstr_retran_cnt_out_of_sync);								
              (void) nrf_esb_flush_tx();
			
						 //channel_attempts = 0;  //JS Modify: 1/7/2022
        // nrf_gpio_pin_clear(BSP_LED_1);
				
#ifdef DBG_SIG_ENABLE								
           nrf_gpio_pin_clear(DBG_SIG_TX_FAIL);
#endif							
           }							
            break;  
             
        case NRF_ESB_EVENT_RX_RECEIVED:
            evt.type = jetstr_evt_rx_received;				 
            break;					
    }
		
    jetstr_event_callback(&evt);
}	

volatile  static uint16_t jetstr_tx_period;
volatile  static uint16_t jetstr_modify_rx_period;	

static void nrf_esb_prx_event_handler(nrf_esb_evt_t const * p_event)
{
   uint8_t retx_num;

   nrf_esb_payload_t rx_buf;
   jetstr_evt_t evt;
	
    evt.type = jetstr_evt_none;
	 	
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
					 
             break;
        case NRF_ESB_EVENT_TX_FAILED:
					
             break;
        case NRF_ESB_EVENT_RX_RECEIVED:

             first_rnd_sw  = true;
             chan_sw_cnt = 0;      // JS Modify: 10/22/2018 <-- reset channel sw cnt

             evt.type = jetstr_evt_rx_received;
#ifdef DBG_SIG_ENABLE				 
             nrf_gpio_pin_set(DBG_SIG_RF_RCV);
#endif				
             JETSTR_SYS_TIMER->TASKS_STOP =1;
             JETSTR_SYS_TIMER->TASKS_CLEAR =1;         //clear timer 
             nrf_esb_read_rx_payload(&rx_buf);			
            
							jetstr_modify_rx_period = (m_jetstr_cfg_params.jetstr_rx_period - m_jetstr_cfg_params.jetstr_rx_delay); 					 
            		
         			evt.rcv_length = rx_buf.length;
							memcpy(evt.rcv_data, rx_buf.data, evt.rcv_length) ;
							jetstr_event_callback(&evt);
      								
							retx_num = rx_buf.data[0]; 				

							JETSTR_SYS_TIMER->TASKS_STOP  =1;	
							JETSTR_SYS_TIMER->TASKS_CLEAR =1;
										
							if(retx_num <=2)
							{			               							 
									JETSTR_SYS_TIMER->CC[0]     = jetstr_modify_rx_period - (retx_num* m_jetstr_cfg_params.jetsr_rx_retran);		                					 
							} 
				
							JETSTR_SYS_TIMER->CC[1]    =  jetstr_tx_period - 100; 
				
             JETSTR_SYS_TIMER->TASKS_START =1;	
				 

#ifdef DBG_SIG_ENABLE				 
            nrf_gpio_pin_clear(DBG_SIG_RF_RCV);
#endif				
				  
    }
	
}



static int jetstr_esb_init(enum esb_mode mode)
{
    int err;
    uint8_t base_addr_0[4] ;
    uint8_t addr_prefix[] = {1}; 
		
	memcpy(base_addr_0, pipe0_addr	, 4);
		
    channel_cnt =0;
	
    struct esb_config	config			= NRF_ESB_DEFAULT_CONFIG;
    config.protocol	                    = NRF_ESB_PROTOCOL_ESB_DPL;
		
#ifdef NRF52840_XXAA		
    config.tx_output_power				= NRF_ESB_TX_POWER_8DBM;  
#else		

	config.tx_output_power				= NRF_ESB_TX_POWER_4DBM; 
#endif	
	
//	config.tx_output_power              = NRF_ESB_TX_POWER_NEG20DBM;   //<-- for testing only	
		
	config.mode                     		= mode;
	if (mode == ESB_MODE_PTX)	
	{	
	config.event_handler            = esb_ptx_event_handler;
	config.selective_auto_ack       = false;
	config.retransmit_count         = m_jetstr_cfg_params.jetstr_retran_cnt_out_of_sync;
	}	
	else if (mode == ESB_MODE_PRX)
	{	
		config.event_handler            = esb_prx_event_handler; 
	}

	err = esb_init(&config);
	if (err) {
		return err;
	}
		
	err = esb_set_rf_channel(rf_channel_tab[channel_cnt]);
	if (err) {
		return err;
	}
	

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
			return err;
		}
	

	err = esb_set_prefixes(addr_prefix, sizeof(addr_prefix));
	if (err) {
			return err;
		}
	
	if (mode == NRF_ESB_MODE_PTX)
	{	
#ifdef DBG_SIG_ENABLE	 	
     nrf_gpio_cfg_output(DBG_SIG_POLL_EXP);
     nrf_gpio_cfg_output(DBG_SIG_SEND_PKT);
     nrf_gpio_cfg_output(DBG_SIG_TX_SUCCESS);
     nrf_gpio_cfg_output(DBG_SIG_RETX);
     nrf_gpio_cfg_output(DBG_SIG_TX_FAIL);
#endif	
	}
   else if (mode == NRF_ESB_MODE_PRX)	
   {
		 
     NVIC_ClearPendingIRQ(JETSTR_IRQn);
     NVIC_SetPriority(JETSTR_IRQn,0x00);
     NVIC_EnableIRQ(JETSTR_IRQn);
     JETSTR_SYS_TIMER->INTENSET =   TIMER_INTENSET_COMPARE0_Msk ;
   //JETSTR_SYS_TIMER->INTENSET = ( TIMER_INTENSET_COMPARE0_Msk | TIMER_INTENSET_COMPARE1_Msk);
#ifdef DBG_SIG_ENABLE		
     for (uint8_t i=0; i<5; i++)
     {
       nrf_gpio_cfg_output(dbg_sig_rf_channel[i]);		
     }
     nrf_gpio_cfg_output(DBG_SIG_RF_RCV);
     nrf_gpio_cfg_output(DBG_SIG_TIM_IRQ);
#endif			 
   }
		
   return 0;
}

void jetstr_hfclk_start(void)
{
    //Start HFCLK
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
 
}	


void jetstr_hfclk_stop(void)
{
   //Stop HFCLK
   NRF_CLOCK->TASKS_HFCLKSTOP = 1;
}	


void jetstr_init(const jetstr_cfg_t *  jetstr_cfg, const jetstr_cfg_params_t * jetstr_cfg_params)
{
	
	memcpy(&m_jetstr_cfg_params, jetstr_cfg_params, sizeof(m_jetstr_cfg_params));

	memcpy(rf_channel_tab, m_jetstr_cfg_params.jetstr_channel_tab, sizeof(rf_channel_tab));

	max_channel_attempts_before_discard = m_jetstr_cfg_params.jetstr_channel_tab_size +1;

    jetstr_channel_cnt = 0;
	
    jetstr_event_callback = jetstr_cfg->event_callback;
	
    jetstr_esb_init(jetstr_cfg->mode);
	
}
	

void jetstr_rx_start(uint16_t rx_period)
{
  uint32_t err_code;  
#ifdef DBG_SIG_ENABLE		
	nrf_gpio_pin_set(dbg_sig_rf_channel[0]);
#endif	
	
	jetstr_hfclk_start();
	
  //nrf_esb_write_payload(&sys_addr_buf);
	
  err_code = nrf_esb_start_rx();
	
// Configure the system timer with a 1 MHz base frequency
  JETSTR_SYS_TIMER->PRESCALER = 4;
  JETSTR_SYS_TIMER->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
  JETSTR_SYS_TIMER->SHORTS    = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
  JETSTR_SYS_TIMER->CC[0]     = m_jetstr_cfg_params.jetstr_rx_period;
  JETSTR_SYS_TIMER->TASKS_CLEAR =1;
  JETSTR_SYS_TIMER->TASKS_START =1;
  
  APP_ERROR_CHECK(err_code);
	

	
}	


void JETSTR_IRQHandler(void)
{
	if (JETSTR_SYS_TIMER->EVENTS_COMPARE[0] == 1)
	{	
           JETSTR_SYS_TIMER->EVENTS_COMPARE[0] =0;         //clear timer compare event
	   JETSTR_SYS_TIMER->CC[0]             = m_jetstr_cfg_params.jetstr_rx_period; 	
#ifdef DBG_SIG_ENABLE	 
	  nrf_gpio_pin_set(DBG_SIG_TIM_IRQ);
#endif	
	
          nrf_esb_stop_rx();
		 		  
		 
#ifdef DBG_SIG_ENABLE			 
	  nrf_gpio_pin_clear( dbg_sig_rf_channel[jetstr_channel_cnt]);
#endif
      if ( (!first_rnd_sw) || (chan_sw_cnt%2 == 0) )
      {
              jetstr_channel_cnt= (jetstr_channel_cnt + 1) % JETSTR_CHANNEL_TAB_SIZE;

      }

                 
      if  ( first_rnd_sw )
      {
         if  (chan_sw_cnt >= JETSTR_CHANNEL_TAB_SIZE * 2 )  
         {                         
             first_rnd_sw = false;
             chan_sw_cnt = 0;
         }   

         else
         {
           chan_sw_cnt++;
         }
         
      }
      
    
                
#ifdef DBG_SIG_ENABLE				 
		  nrf_gpio_pin_set( dbg_sig_rf_channel[jetstr_channel_cnt]);
#endif
		 
		  nrf_esb_set_rf_channel(rf_channel_tab[jetstr_channel_cnt]);	 
	    nrf_esb_start_rx();
		 

#ifdef DBG_SIG_ENABLE	 
	  nrf_gpio_pin_clear(DBG_SIG_TIM_IRQ);
#endif		 
	 }
	

}	

