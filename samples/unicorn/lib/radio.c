#include <zephyr.h>
#include <zephyr/types.h>
#include <irq.h>
#include <nrf.h>
#include <string.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <hal/nrf_gpio.h>
#include <logging/log.h>
#include "radio.h"
#include "radio_config.h"

LOG_MODULE_REGISTER(radio, CONFIG_APP_LOG_LEVEL);


#define MAX_PACKET_LENGTH 128


static event_callback_t     m_event_callback;

static  radio_evt_t         m_radio_event;

static rx_states_t rx_state;

static uint8_t rf_chan_idx=0;
static uint8_t dma_buf[MAX_PACKET_LENGTH+1];
static uint8_t* m_tx_buf;
static uint8_t* m_rx_buf;
static uint8_t  m_tx_length;


static volatile int8_t rssi = 0;
static volatile bool packet_received = false;
static volatile bool crc_ok = false;
static bool m_is_transmitter = false;

static uint8_t loss_cnt = 0;


// These function pointers are changed dynamically, depending on protocol configuration and state.
static void (*on_radio_timer_interrupt)(void) = 0;

int8_t get_rssi()
{
    return rssi;
}

bool get_crc()
{
    return crc_ok;
}

bool is_packet_received()
{
    bool ret_val = packet_received;
    if(packet_received)
    {
        packet_received = false;
    }
    return ret_val;
}

void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

#if 0
void radio_set_mode(radio_modes_t mode)
{
    NRF_RADIO->POWER = 0;
    radio_setup(m_is_transmitter, m_tx_power, mode);
}
#endif

void radio_set_tx_power(int8_t power)
{
    NRF_RADIO->TXPOWER = power;

}


#if 0
static void reload_timer(nrf_drv_timer_t const * const p_instance, uint32_t reload_value)
{
  uint32_t time_ticks;

    if ( nrf_drv_timer_is_enabled(p_instance) )
    {
      nrf_drv_timer_pause(p_instance);
      nrf_drv_timer_clear(p_instance); 
    }
    time_ticks = nrf_drv_timer_ms_to_ticks(p_instance, reload_value);  //reload comp value
    nrf_drv_timer_extended_compare(p_instance, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    if ( nrf_drv_timer_is_enabled(p_instance) )
    {
        nrf_drv_timer_resume(p_instance);
    }
    else
    {
        nrf_drv_timer_enable(p_instance);
    }
}
#endif



/**
 * @brief Handler for timer events.
 */

void RADIO_TIMER_IRQHandler(void)
{
   if (RADIO_TIMER->EVENTS_COMPARE[0] == 1)
   {
     RADIO_TIMER->EVENTS_COMPARE[0] =0;         //clear timer compare event

     nrf_gpio_pin_toggle(PIN_CHANNEL_HOP);
	  
     
     if(on_radio_timer_interrupt)
       on_radio_timer_interrupt();

   }
}



static void timer_tx_event_handler(void)
{
	//Hop frequnecy
	rf_chan_idx =(rf_chan_idx+1)% (RF_CHAN_TAB_SIZE);  
	NRF_RADIO->FREQUENCY    = RF_CHANNEL_TAB[rf_chan_idx];
	//Application callback
	m_radio_event.evt_id = RADIO_EVENT_POLL_EXP;
	m_radio_event.chan_cnt = rf_chan_idx;				
	m_event_callback(&m_radio_event);	
	//Copy m_tx_buf to dma_buf
	dma_buf[0]= m_tx_length;
	memcpy(&dma_buf[1], m_tx_buf, m_tx_length);
	//Start TX
	NRF_RADIO->TASKS_TXEN = 1;
      
}


static void timer_rx_event_handler(void)
{         
	rf_chan_idx =(rf_chan_idx+1)% (RF_CHAN_TAB_SIZE);
 
	NRF_RADIO->FREQUENCY    = RF_CHANNEL_TAB[rf_chan_idx];
   
	NRF_RADIO->TASKS_RXEN  = 1;

   if (rx_state== RF_OPERATE)
   {

	  if ( (loss_cnt ==  RF_CHAN_TAB_SIZE ) )
	  {

		rx_state = RF_SEARCH;
	   //Reload  RF_RX_OPERATE_CNT to radio timer
	   RADIO_TIMER->TASKS_STOP=1;
	   RADIO_TIMER->TASKS_CLEAR =1;
	   RADIO_TIMER->CC[0] = RF_RX_SEARCH_PERIOD;
	   RADIO_TIMER->TASKS_START =1;

	  }
	  else
					{
	   RADIO_TIMER->TASKS_STOP=1 ;
	   RADIO_TIMER->TASKS_CLEAR=1;
	   RADIO_TIMER->CC[0] = RF_RX_OPERATE_PERIOD;
	   RADIO_TIMER->TASKS_START =1;
		loss_cnt++;
		
	  }  
   }

}

void RADIO_IRQHandler(void)
{

    if (NRF_RADIO->EVENTS_ADDRESS)
    {
        NRF_RADIO->EVENTS_ADDRESS = 0;      
    }

    if (NRF_RADIO->EVENTS_DISABLED && (NRF_RADIO->INTENSET & RADIO_INTENSET_DISABLED_Msk))
    {
        NRF_RADIO->EVENTS_DISABLED = 0;

        if(NRF_RADIO->CRCSTATUS & (RADIO_CRCSTATUS_CRCSTATUS_CRCOk << RADIO_CRCSTATUS_CRCSTATUS_Pos))
        {
           
            rssi = -NRF_RADIO->RSSISAMPLE;
            packet_received = true;

           if(!m_is_transmitter)
           {

              nrf_gpio_pin_set(PIN_DATA_RECEIVED);

              loss_cnt = 0;             
             

              if(!TUNE_MODE)
              {
                // Change to RF_OPERATE
                if (rx_state == RF_SEARCH )
                {
										rx_state = RF_OPERATE;
								}

                //Reload  RF_RX_OPERATE_CNT to radio timer
                RADIO_TIMER->TASKS_STOP =1 ;
                RADIO_TIMER->TASKS_CLEAR =1 ;
                RADIO_TIMER->CC[0] = RF_RX_OPERATE_CNT;
            
                RADIO_TIMER->TASKS_START =1;
              }

              nrf_gpio_pin_clear(PIN_DATA_RECEIVED);
							
							m_radio_event.evt_id = RADIO_EVENT_POLL_RCV;
							m_radio_event.chan_cnt =  rf_chan_idx;
							m_event_callback(&m_radio_event);
      
           }
					 
					 
        }
        else
        {
           packet_received = false;
        }
        
 
    }


}



void radio_setup(radio_init_t init )
{

    m_event_callback = init.event_callback;
    m_is_transmitter = init.is_transmitter;
	m_tx_buf = init.tx_buf;
	m_rx_buf = init.rx_buf;
	m_tx_length = init.tx_length;

    clocks_start();
    

     on_radio_timer_interrupt = (m_is_transmitter)? timer_tx_event_handler:timer_rx_event_handler;

    NRF_RADIO->MODECNF0     |= true;
    
    NRF_RADIO->POWER        = 1;
	
#ifdef NRF52840_XXAA	   
    uint32_t preamble_mask = 0;
	
    if  ( (mode == MODE_125_KBIT) || (mode == MODE_500_KBIT) )
    {
        preamble_mask  = ((RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos) & (3 << RADIO_PCNF0_PLEN_Pos));
        preamble_mask |= ((2                          << RADIO_PCNF0_CILEN_Pos) & RADIO_PCNF0_CILEN_Msk);
        preamble_mask |= ((3                          << RADIO_PCNF0_TERMLEN_Pos) & RADIO_PCNF0_TERMLEN_Msk); 
    }
#endif
    
    NRF_RADIO->PCNF0        =   (((0UL) << RADIO_PCNF0_S0LEN_Pos                               ) & RADIO_PCNF0_S0LEN_Msk)
                              | (((0UL) << RADIO_PCNF0_S1LEN_Pos                               ) & RADIO_PCNF0_S1LEN_Msk)
                              | (((8UL) << RADIO_PCNF0_LFLEN_Pos                               ) & RADIO_PCNF0_LFLEN_Msk);
                              
#ifdef NRF52840_XXAA	
    NRF_RADIO->PCNF0       |= preamble_mask;	
#endif		
		
    NRF_RADIO->PCNF1        =   (((RADIO_PCNF1_ENDIAN_Big )      << RADIO_PCNF1_ENDIAN_Pos    ) & RADIO_PCNF1_ENDIAN_Msk)
                              | (((3UL)                           << RADIO_PCNF1_BALEN_Pos     ) & RADIO_PCNF1_BALEN_Msk)
                              | (((0UL)                           << RADIO_PCNF1_STATLEN_Pos   ) & RADIO_PCNF1_STATLEN_Msk)
                              | ((((uint32_t) MAX_PACKET_LENGTH)                 << RADIO_PCNF1_MAXLEN_Pos    ) & RADIO_PCNF1_MAXLEN_Msk)
                              | ((RADIO_PCNF1_WHITEEN_Disabled     << RADIO_PCNF1_WHITEEN_Pos   ) & RADIO_PCNF1_WHITEEN_Msk);
    NRF_RADIO->CRCCNF       =   (((RADIO_CRCCNF_SKIPADDR_Skip)    << RADIO_CRCCNF_SKIPADDR_Pos ) & RADIO_CRCCNF_SKIPADDR_Msk)
                            | (((RADIO_CRCCNF_LEN_Three)        << RADIO_CRCCNF_LEN_Pos      ) & RADIO_CRCCNF_LEN_Msk);
    NRF_RADIO->CRCPOLY      = 0x0000065b;
    NRF_RADIO->RXADDRESSES  = ((RADIO_RXADDRESSES_ADDR0_Enabled)  << RADIO_RXADDRESSES_ADDR0_Pos);
    NRF_RADIO->SHORTS       = ((1 << RADIO_SHORTS_READY_START_Pos) | (1 << RADIO_SHORTS_END_DISABLE_Pos)
                            | RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_DISABLED_RSSISTOP_Msk);
    
    switch(init.mode)
    {
#ifdef NRF52840_XXAA			
        case MODE_125_KBIT:
             NRF_RADIO->MODE         = ((5)        << RADIO_MODE_MODE_Pos       ) & RADIO_MODE_MODE_Msk;  
            break;
        case MODE_500_KBIT:
            NRF_RADIO->MODE          = ((6)        << RADIO_MODE_MODE_Pos       ) & RADIO_MODE_MODE_Msk;  
            break;
#endif				
        case MODE_1_MBIT:
            NRF_RADIO->MODE         = (RADIO_MODE_MODE_Ble_1Mbit        << RADIO_MODE_MODE_Pos       ) & RADIO_MODE_MODE_Msk;
            break;
        case MODE_2_MBIT:
            NRF_RADIO->MODE         = (RADIO_MODE_MODE_Ble_2Mbit        << RADIO_MODE_MODE_Pos       ) & RADIO_MODE_MODE_Msk;
            break;
        default:
            break;
    }
    
    //NRF_RADIO->TIFS         = 150;
    
   

    if(!m_is_transmitter)
    {
		
		IRQ_DIRECT_CONNECT(RADIO_IRQn, 6 ,
						   RADIO_IRQHandler, 0);
		irq_enable(RADIO_IRQn);										
		NVIC_ClearPendingIRQ(RADIO_IRQn);
		NVIC_EnableIRQ(RADIO_IRQn);
		
		NRF_RADIO->INTENSET     = (1 << RADIO_INTENSET_DISABLED_Pos); // | (1 << RADIO_INTENSET_ADDRESS_Pos); //| (1 << RADIO_INTENSET_CRCERROR_Pos);     
    }
    
//    NRF_RADIO->PREFIX0      = 0x0000008e; //access_addr[3]
//    NRF_RADIO->BASE0        = 0x89bed600; //access_addr[0:3]
//    NRF_RADIO->CRCINIT      = 0x00555555;
		
		// Configure radio address registers according to ESB default values
    NRF_RADIO->BASE0   = 0xE7E7E7E7;
    NRF_RADIO->BASE1   = 0x43434343;
    NRF_RADIO->PREFIX0 = 0x23C343E7;
    NRF_RADIO->PREFIX1 = 0x13E363A3;
		
    NRF_RADIO->PACKETPTR    = (uint32_t) dma_buf;
    

	// Radio Timer IRQ settings 
	IRQ_DIRECT_CONNECT(RADIO_TIMER_IRQn, 6 ,
					   RADIO_TIMER_IRQHandler, 0);
	irq_enable(RADIO_TIMER_IRQn);
	NVIC_ClearPendingIRQ(RADIO_TIMER_IRQn);
	NVIC_EnableIRQ(RADIO_TIMER_IRQn);
	
	RADIO_TIMER->INTENSET =   TIMER_INTENSET_COMPARE0_Msk ;
 
   
	// Configure the radio timer with a 1 MHz base frequency
	RADIO_TIMER->PRESCALER = 4;
	RADIO_TIMER->BITMODE   = TIMER_BITMODE_BITMODE_32Bit;
	RADIO_TIMER->SHORTS    = TIMER_SHORTS_COMPARE0_CLEAR_Msk;


	NRF_RADIO->FREQUENCY    = RF_CHANNEL_TAB[rf_chan_idx];

    if(m_is_transmitter)
    {
        radio_set_tx_power(init.tx_power);
          
          RADIO_TIMER->CC[0]        = EVENT_US;
          RADIO_TIMER->TASKS_CLEAR =1;
          RADIO_TIMER->TASKS_START =1;
       
    }

    
    memset(dma_buf, 0, sizeof(dma_buf));

  
  
}


void radio_receive_packet()
{	
  // 
     rx_state = RF_SEARCH;
 
     RADIO_TIMER->CC[0]        = RF_RX_SEARCH_PERIOD;
     RADIO_TIMER->TASKS_CLEAR =1;
     RADIO_TIMER->TASKS_START =1;
    
    NRF_RADIO->TASKS_RXEN = 1;
}


uint8_t radio_get_packet(void)
{
    memcpy(m_rx_buf, &dma_buf[1], dma_buf[0]);
    return dma_buf[0];
}



