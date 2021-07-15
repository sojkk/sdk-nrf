/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#include "esb.h"
#include "radio.h"
#include "radio_config.h"
#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <drivers/gpio.h>
#include <irq.h>
#include <logging/log.h>
#include <string.h>
#include <nrf.h>
#include <zephyr.h>
#include <zephyr/types.h>


LOG_MODULE_REGISTER(radio, CONFIG_APP_LOG_LEVEL);

#define RSSI_SAMPLE_TIMES	6

#define MAX_RTC_TASKS_DELAY     47                                          /**< Maximum delay until an RTC task is executed. */

//log
//static uint16_t m_log_total_cnt[] =  {0, 0};
//static uint16_t m_log_success_cnt[]= {0, 0};

static bool m_rx_ack_wait = false;

static bool m_rx_received = false;

static uint8_t chan_cnt = 0;

static uint8_t m_radio_chan_tab[] = RADIO_CHAN_TAB;

static uint8_t rx_loss_cnt = 0;

static uint8_t periph_cnt = 0;
														 
static struct esb_payload	dum_payload  = ESB_CREATE_PAYLOAD(DATA_PIPE, 0xBE);

static struct esb_payload    data_payload;

// These function pointers are changed dynamically, depending on protocol configuration and state.
static void (*on_radio_rtc_interrupt)(void) = 0;

static event_callback_t m_event_callback;

static const uint8_t m_system_address[]= SYSTEM_ADDRESS ;

static const uint8_t dbg_pins[] = {DATA_SENDING_P0, DATA_SENDING_P1, DATA_SENDING_P2, DATA_SENDING_P3 ,DATA_SENDING_P4, DATA_SENDING_P5 };

static radio_states_t rx_state;

static bool is_rx_on = false;

static const struct device *dbg_port;

static void RADIO_RTC_IRQHandler(void);

static bool bct_rcv_flag = false;


/**@brief Function for init the RTC1 timer.
 */
static void radio_rtc_init(void)
{
    RADIO_RTC->INTENSET = RTC_INTENSET_COMPARE0_Msk;

    IRQ_DIRECT_CONNECT(RADIO_RTC_IRQn, 6 ,
                       RADIO_RTC_IRQHandler, 0);

    irq_enable(RADIO_RTC_IRQn);

    NVIC_ClearPendingIRQ(RADIO_RTC_IRQn);
    NVIC_EnableIRQ(RADIO_RTC_IRQn);
}	
 
 
 
 
/**@brief Function for starting the RTC1 timer.
 */ 
static void radio_rtc_start(void)
{
    
    RADIO_RTC->TASKS_START = 1;
    k_sleep(K_USEC(MAX_RTC_TASKS_DELAY));

}


/**@brief Function for stopping the RTC1 timer.
 */
static void radio_rtc_stop(void)
{
    NVIC_DisableIRQ(RADIO_RTC_IRQn);

    RADIO_RTC->TASKS_STOP = 1;
    k_sleep(K_USEC(MAX_RTC_TASKS_DELAY));

    RADIO_RTC->TASKS_CLEAR = 1;
    k_sleep(K_USEC(MAX_RTC_TASKS_DELAY));

}

/**@brief Function for setting the RTC Capture Compare register 0, and enabling the corresponding
 *        event.
 *
 * @param[in] value   New value of Capture Compare register 0.
 */
static __INLINE void radio_rtc_compare0_set(uint32_t value)
{
    RADIO_RTC->CC[0] = value;
}


static __INLINE void radio_rtc_clear_count(void)
{
	 RADIO_RTC->TASKS_CLEAR =1;
}	



static void rtc_tx_event_handler(void)
{
        int err;
	
	esb_flush_tx();  //JS Modify: 4/23/2020  <--
	
	radio_rtc_clear_count();
	
	//server periphs 
	esb_update_prefix(DATA_PIPE, periph_cnt);	

	
	if(periph_cnt == BCT_PERIPH_NUM) //broadcast
	{
		
		LOG_DBG("data payload = %d, %d, %d", data_payload.data[0], data_payload.data[1], data_payload.data[2]);
		LOG_DBG("data length =%d, pipe = %d", data_payload.length, data_payload.pipe);
		
		data_payload.noack = true;
		err = esb_write_payload(&data_payload);
		if(err)
		{
			LOG_ERR("Broadcasting packet failed");	
		}
        else
		{
			gpio_pin_set(dbg_port, dbg_pins[periph_cnt], 1); //Set debug pin for poll the periph
		}
		
		m_event_callback(RADIO_CENTRAL_BCT_SENT);
	}
	else
	{
		dum_payload.noack = false;
		err = esb_write_payload(&dum_payload);
	}
	
	if(err)
    {
		LOG_ERR("Sending packet failed");	
	}
        else
	{
		gpio_pin_set(dbg_port, dbg_pins[periph_cnt], 1); //Set debug pin for poll the periph
	}
	

}

static void rtc_rx_event_handler(void)
{

	
	if(rx_state ==  RADIO_PERIPH_SEARCH)
	{
		 radio_rtc_clear_count();
		
			//For power saving, turn off scan for RX_SEARCH_PERIOD
			if(is_rx_on)
			{
				is_rx_on = false;
				esb_stop_rx();
				gpio_pin_set(dbg_port, dbg_pins[4], 0);
				
			}
			else
			{
				is_rx_on = true;
				
				gpio_pin_set(dbg_port, dbg_pins[4], 1);

				chan_cnt = (chan_cnt +1) % RADIO_CHAN_TAB_SIZE;
				esb_set_rf_channel(m_radio_chan_tab[chan_cnt]);
				esb_start_rx();
				
			}
	}
	else if ( rx_state == RADIO_PERIPH_WAIT_FOR_ACK_WR)
	{

		
		esb_stop_rx();	
		gpio_pin_set(dbg_port, dbg_pins[4], 0);							
		rx_state = RADIO_PERIPH_OPERATE;
		radio_rtc_clear_count();		
		radio_rtc_compare0_set(RX_OPERATE_PERIOD_W_WAIT);
	
				
		m_rx_received = true;
		m_rx_ack_wait = true;

	}

	
	else //rx_state == RADIO_PERIPH_OPERATE
	{
	
	
		
		if (rx_loss_cnt == RX_LOSS_THRESHOLD	)
		{
			rx_state = RADIO_PERIPH_SEARCH;
			radio_rtc_compare0_set(RX_SEARCH_PERIOD);
			 m_rx_received = false;
		}
		else
		{		
			if(m_rx_received)
			{	
				radio_rtc_clear_count();  //1/6/2021
				//JS Modify: 10/29/2020, switch to next rf chan for next round
				radio_rtc_compare0_set(RX_OPERATE_PERIOD);
				if(!m_rx_ack_wait)
				{
					esb_stop_rx();
				}
				esb_set_rf_channel(m_radio_chan_tab[chan_cnt]);	
						chan_cnt = (chan_cnt +1) % RADIO_CHAN_TAB_SIZE;	
				
					m_rx_ack_wait = false;
			}
			else //1/6/2021 modify
			{
				rx_loss_cnt++;
			}
			
			
                        gpio_pin_set(dbg_port, dbg_pins[4], 1);
			
		}
		
		esb_start_rx();
		

	}
}



static void nrf_esb_ptx_event_handler(struct esb_evt const * p_event)
{	
	//uint16_t success_rate;
	
	switch (p_event->evt_id)
    {
        case ESB_EVENT_TX_SUCCESS:
		{
				LOG_DBG("TX SUCCESS EVENT");	                                                          
				__NOP();
								
									
				if(periph_cnt == BCT_PERIPH_NUM)
				{	
					gpio_pin_set(dbg_port, dbg_pins[periph_cnt], 0);
					
					periph_cnt = (periph_cnt +1) % NUM_OF_PERIPH;
					
					
				}
			
		}						
				break;
        case ESB_EVENT_TX_FAILED:
				LOG_DBG("TX FAILED EVENT");
				(void) esb_flush_tx();			
				break;
        case ESB_EVENT_RX_RECEIVED:										 
				LOG_DBG("RX RECEIVED EVENT");
				
				//m_log_success_cnt[periph_cnt]++; //log
				

				gpio_pin_set(dbg_port, dbg_pins[periph_cnt],0); //clear the debug pin if received data 
                             
									
				m_event_callback(RADIO_CENTRAL_DATA_RECEIVED);		   
				
				
				break;
			
			
		default:
		
				break;
	}	


        //gpio_pin_set(dbg_port, dbg_pins[periph_cnt],0); //clear the debug pin for testing timer only
		
		if (p_event->evt_id != ESB_EVENT_TX_SUCCESS)   // TX_SUCCESS deplicates with DATA_RECEIVED
		{	
			periph_cnt = (periph_cnt +1) % NUM_OF_PERIPH; // periph_cnt = [0..(NUM_OF_PERIPH-1)]

			//JS Modify: 10/29/2020, switch to next rf chan for next round
			if(periph_cnt==0)
			{	
				chan_cnt = (chan_cnt +1) % RADIO_CHAN_TAB_SIZE;		
				esb_set_rf_channel(m_radio_chan_tab[chan_cnt]);
			}
			
			//m_log_total_cnt[periph_cnt]++; //log
		}
		
//log		
		//if  (m_log_total_cnt[periph_cnt]== LOG_CNT)		
		//{		
			//success_rate = 	m_log_success_cnt[periph_cnt] *100 /m_log_total_cnt[periph_cnt] ;
			//LOG_INF("Success rate for periph %d  = %d", periph_cnt+1, m_log_success_cnt[periph_cnt]);
			//m_log_total_cnt[periph_cnt]=0;
			//m_log_success_cnt[periph_cnt]=0;
		//}
}


static void nrf_esb_prx_event_handler(struct esb_evt const *p_event)
{
    int err;

	switch (p_event->evt_id)
    {
        case ESB_EVENT_TX_SUCCESS:
				LOG_DBG("TX SUCCESS EVENT");							
				break;
        case ESB_EVENT_TX_FAILED:
				LOG_DBG("TX FAILED EVENT");
				break;
        case ESB_EVENT_RX_RECEIVED:
				LOG_DBG("RX RECEIVED EVENT");
	
		if (p_event->tx_attempts <= RETRAN_CNT)
						m_rx_received = true;
	
		if(bct_rcv_flag)
		{
			           										
			m_event_callback(RADIO_PERIPH_DATA_RECEIVED);	
			
		
		
		}
		else
		{
	
			err = esb_read_rx_payload(&dum_payload);
			
			if (!err)             
			{								
	
				esb_write_payload(&data_payload);
			
				m_event_callback(RADIO_PERIPH_DATA_SENT);
			}		
										
				
		}
		
		rx_state = RADIO_PERIPH_WAIT_FOR_ACK_WR;
		radio_rtc_clear_count();
		radio_rtc_compare0_set(RX_WAIT_FOR_ACK_WR_PERIOD);
		rx_loss_cnt = 0;
							
		default: 
			
			break;
	}
}





static void radio_hfclk_start( void )
{
  //Start HFCLK
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}	



static void dbg_pins_init(void)
{
        int err;
        
        dbg_port = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	if (!dbg_port) {
		LOG_ERR("Could not bind to radio debug port");
         }

         for (size_t i = 0; i < ARRAY_SIZE(dbg_pins); i++) {
			err = gpio_pin_configure(dbg_port, dbg_pins[i],
					GPIO_OUTPUT);
			if (err) {
					LOG_ERR("Unable to configure radio debug port");
					dbg_port = NULL;
				}
                      		
                        gpio_pin_set(dbg_port, dbg_pins[i], 0); 
         }
}




int radio_setup(bool is_central, radio_tx_power_t tx_power,  event_callback_t event_callback, uint8_t periph_num)	
{
	int err;
	
	uint8_t addr_prefix[]= {0, 1};
	
	m_event_callback = event_callback;
		
	struct esb_config config        = ESB_DEFAULT_CONFIG;
	
	config.protocol                 = ESB_PROTOCOL_ESB_DPL;
	config.bitrate                  = ESB_BITRATE_1MBPS;
	config.event_handler            = (is_central)?nrf_esb_ptx_event_handler:nrf_esb_prx_event_handler;
	config.mode                     = (is_central)?ESB_MODE_PTX:ESB_MODE_PRX;
	config.selective_auto_ack       = true; 
	config.tx_output_power          = tx_power;
	config.retransmit_count         = RETRAN_CNT;
	config.retransmit_delay         = 600;
	config.payload_length           = 64;

	bct_rcv_flag = false;



/********** nRF3540 radio power with additional 3dB *******/	

#if defined(CONFIG_SOC_SERIES_NRF53X)

	NRF_VREQCTRL->VREGRADIO.VREQH = 1;
	while(!(NRF_VREQCTRL->VREGRADIO.VREQHREADY ==1) )
		;
#endif

/**********Receive Broadcast ***************************/	
	
	if (periph_num == BCT_PERIPH_NUM)
	{
		bct_rcv_flag = true;
	}
	
/*******************************************************/	
	err = esb_init(&config);
	if (err) {
		return err;
	}
	
	err = esb_set_base_address_0(m_system_address);
	if (err) {
		return err;
	}
	
	err = esb_set_rf_channel(m_radio_chan_tab[0]);
	if (err) {
		return err;
	}
	
	if(!is_central)
	{
		addr_prefix[0] = periph_num;
	}
	
	err = esb_set_prefixes(addr_prefix, sizeof(addr_prefix));
	if (err) {
		return err;
	}
	
	err = esb_enable_pipes(1 << DATA_PIPE );  //only enabled data pipe
	if (err) {
		return err;
	}	
	
	on_radio_rtc_interrupt = (is_central)? rtc_tx_event_handler:rtc_rx_event_handler;

	radio_rtc_init();

	radio_hfclk_start();

	dbg_pins_init();
		
	return 0;
}





void radio_poll_timer_start(uint32_t cnt)
{
	
	radio_rtc_compare0_set(cnt);
	
	radio_rtc_start();	
	
}



void radio_fetch_packet(radio_data_t * rcv_data)
{
	struct esb_payload  rx_payload;
	
	esb_read_rx_payload(&rx_payload);
	
	rcv_data->length = rx_payload.length;
	rcv_data->periph_num = esb_get_addr_prefix(DATA_PIPE); 
	memcpy(rcv_data->data, rx_payload.data, rx_payload.length);	
	
	LOG_DBG("rcv data = %d, %d, %d", rcv_data->data[0], rcv_data->data[1], rcv_data->data[2]);
	LOG_DBG("rcv length = %d", rcv_data->length);
	
}	


void radio_scan_for_poll(void)
{
      int err;

      rx_state = RADIO_PERIPH_SEARCH;

      radio_rtc_compare0_set(RX_SEARCH_PERIOD);

      radio_rtc_start();
      
      err = esb_start_rx();

      gpio_pin_set(dbg_port, dbg_pins[4] ,1);

      is_rx_on = true;
	
}


void radio_put_packet(radio_data_t * tx_data)
{	
	data_payload.pipe = DATA_PIPE;
	data_payload.length = tx_data->length;
	memcpy(data_payload.data, tx_data->data, tx_data->length);
	
}

		
static void RADIO_RTC_IRQHandler(void)
{
	if (RADIO_RTC->EVENTS_COMPARE[0] == 1)	
	{
     RADIO_RTC->EVENTS_COMPARE[0] =0;         //clear rtc compare event
	}
	
	   if(on_radio_rtc_interrupt)
       on_radio_rtc_interrupt();

	
}
