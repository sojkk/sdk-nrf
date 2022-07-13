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

static radio_evt_t			m_radio_event;
static radio_states_t		m_radio_state = IDLE_STATE;
static event_callback_t     m_event_callback;
// Address parameters
__ALIGN(4) static radio_address_t	m_radio_addr = RADIO_ADDR_DEFAULT;

static rx_states_t rx_state;

static uint8_t m_rf_chan_idx=0;
static uint8_t dma_buf[MAX_PACKET_LENGTH+1];  //Byte 0 being length feild, rest of them contains the payload
static uint8_t* m_tx_buf;
static uint8_t* m_rx_buf;
static uint8_t  m_tx_length;

static volatile int8_t rssi = 0;
static volatile bool crc_ok = false;
static bool m_is_central = false;

static bool m_periph_is_poll_rcv = false;

static uint8_t loss_cnt = 0;

// Function to do bytewise bit-swap on an unsigned 32-bit value
static uint32_t bytewise_bit_swap(uint8_t const * p_inp)
{
#if __CORTEX_M == (0x04U)
    uint32_t inp = (*(uint32_t*)p_inp);
    return __REV((uint32_t)__RBIT(inp)); //lint -esym(628, __rev) -esym(526, __rev) -esym(628, __rbit) -esym(526, __rbit) */
#else
    uint32_t inp = (p_inp[3] << 24) | (p_inp[2] << 16) | (p_inp[1] << 8) | (p_inp[0]);
    inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
    inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
    inp = (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
    return inp;
#endif
}

// Convert a base address from nRF24L format to nRF5 format
static uint32_t addr_conv(uint8_t const* p_addr)
{
    return __REV(bytewise_bit_swap(p_addr)); //lint -esym(628, __rev) -esym(526, __rev) */
}

// These function pointers are changed dynamically, depending on protocol configuration and state.
static void (*on_radio_timer_interrupt)(void) = 0;
static void (*on_radio_disabled_interrupt)(void) = 0;

int8_t get_rssi()
{
    return rssi;
}

bool get_crc()
{
    return crc_ok;
}


void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


static void radio_set_tx_power(radio_power_t power)
{
    NRF_RADIO->TXPOWER = power;

}


__INLINE static void on_delay_timer_expired(void)
{
	
	if(m_radio_state == PERIPH_RX_STATE)
	{
		//Set dma_buf to m_tx_buf
		memcpy(&dma_buf[1], m_tx_buf, m_tx_length);
		dma_buf[0] = m_tx_length;
 
		//Start TX
		m_radio_state = PERIPH_TX_STATE;
		NRF_RADIO->TASKS_TXEN = 1;
		nrf_gpio_pin_set(PIN_DATA_TX);
	}

}	


static void timer_tx_event_handler(void)
{
	if(m_radio_state!=IDLE_STATE)
		NRF_RADIO->TASKS_DISABLE =1;
	
	NRF_RADIO->SHORTS       = ((1 << RADIO_SHORTS_READY_START_Pos) | (1 << RADIO_SHORTS_END_DISABLE_Pos)
				| RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_DISABLED_RSSISTOP_Msk);

	NRF_RADIO->INTENSET     = (1 << RADIO_INTENSET_DISABLED_Pos);     
				
				
	m_rf_chan_idx =(m_rf_chan_idx+1)% (RF_CHAN_TAB_SIZE);  
	NRF_RADIO->FREQUENCY    = RF_CHANNEL_TAB[m_rf_chan_idx];
	m_radio_event.evt_id 		= RADIO_EVENT_CENTRAL_POLL_EXP;
	m_radio_event.chan_cnt	= m_rf_chan_idx;
	
	//Callback to application
	m_event_callback(&m_radio_event);

	//Copy m_tx_buf to dma_buf
	memcpy(&dma_buf[1], m_tx_buf, m_tx_length);
	dma_buf[0] = m_tx_length;			

	//Start poll pavket TX
	m_radio_state = CENTRAL_TX_STATE;
	NRF_RADIO->TASKS_TXEN = 1;
      
}


static void timer_rx_event_handler(void)
{	
	m_rf_chan_idx =(m_rf_chan_idx+1)% (RF_CHAN_TAB_SIZE);
 
	NRF_RADIO->FREQUENCY    = RF_CHANNEL_TAB[m_rf_chan_idx];
   
	NRF_RADIO->TASKS_RXEN  = 1;

	m_radio_state = PERIPH_RX_STATE;

	if (rx_state== RX_OPERATE)
	{

		if ( loss_cnt ==  RF_CHAN_TAB_SIZE )
		{

		rx_state = RX_SEARCH;
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
						
		if(m_periph_is_poll_rcv)
		{	
				m_periph_is_poll_rcv = false;
				
		}
		else
		{
				m_radio_event.evt_id = RADIO_EVENT_PERIPH_POLL_NOT_RCV;
				m_radio_event.chan_cnt =  m_rf_chan_idx;
				m_event_callback(&m_radio_event);
		}
						
   }

}


static void on_central_disabled(void)
{
 
		if(m_radio_state == CENTRAL_TX_STATE)
		{
				//modify radio shorts without end -> disable
				 NRF_RADIO->SHORTS       =  ( (1 << RADIO_SHORTS_READY_START_Pos) | 
																			 RADIO_SHORTS_ADDRESS_RSSISTART_Msk | 
																			 RADIO_SHORTS_DISABLED_RSSISTOP_Msk );

				//NRF_RADIO->EVENTS_END		= 0;
				NRF_RADIO->INTENSET     = (1 << RADIO_INTENSET_END_Pos);
			
				//go to CENTRAL_RX_STATE
				NRF_RADIO->TASKS_RXEN = 1;
			
				m_radio_state = CENTRAL_RX_STATE;
			 
		}
		else if (m_radio_state == CENTRAL_RX_STATE)
		{

		}
							

}	


static void on_periph_disabled(void)
{
  if(m_radio_state == PERIPH_RX_STATE)
	{	
				
		if(NRF_RADIO->CRCSTATUS & (RADIO_CRCSTATUS_CRCSTATUS_CRCOk << RADIO_CRCSTATUS_CRCSTATUS_Pos))
		{
       
			//poll packet received	
			m_periph_is_poll_rcv = true;

			//Compare 1 interrupt enable 
			RADIO_TIMER->INTENSET  |=   TIMER_INTENSET_COMPARE1_Msk ;
			 
			RADIO_TIMER->TASKS_CAPTURE[1] =1;
			//Set compare 1 value
			RADIO_TIMER->CC[1] -= PERIPH_TX_DELAY_OFFSET ;
			RADIO_TIMER->CC[1] += PERIPH_TX_DELAY_PERIOD * (PERIPH_NUM-1) ;
			 
            rssi = -NRF_RADIO->RSSISAMPLE;
 

            nrf_gpio_pin_set(PIN_DATA_RECEIVED);
			 
            loss_cnt = 0;             
             
            if(!TUNE_MODE)
            {
                // Change to RX_OPERATE
                if (rx_state == RX_SEARCH )
                {
					rx_state = RX_OPERATE;
				}

                //Reload  RF_RX_OPERATE_CNT to radio timer
                RADIO_TIMER->TASKS_STOP =1 ;
                RADIO_TIMER->TASKS_CLEAR =1 ;
                RADIO_TIMER->CC[0] = RF_RX_OPERATE_CNT;
            
                RADIO_TIMER->TASKS_START =1;
            }

			nrf_gpio_pin_clear(PIN_DATA_RECEIVED);
							
			m_radio_event.evt_id = RADIO_EVENT_PERIPH_POLL_RCV;
			m_radio_event.chan_cnt =  m_rf_chan_idx;
			m_event_callback(&m_radio_event);
      
		}
 
	}
	else if (m_radio_state == PERIPH_TX_STATE)
	{
			nrf_gpio_pin_clear(PIN_DATA_TX);
			m_radio_state = PERIPH_RX_STATE;
	}	
	
}	

static void on_central_end(void)
{
	uint8_t pipe_no;
	
	 if (m_radio_state == CENTRAL_RX_STATE)
	 {
		 
		if(NRF_RADIO->CRCSTATUS & (RADIO_CRCSTATUS_CRCSTATUS_CRCOk << RADIO_CRCSTATUS_CRCSTATUS_Pos))
		{
			 
			 pipe_no = (uint8_t) NRF_RADIO->RXMATCH;
			 
			 //load receive payload to rx_buf
			 memcpy(m_rx_buf+(pipe_no-1)*dma_buf[0],  &dma_buf[1] , dma_buf[0]);
			 
			  if(pipe_no!=NUM_OF_PERIPHS)  
						 NRF_RADIO->TASKS_START =1;  //restart rx
		 
			 if(pipe_no==NUM_OF_PERIPHS)  //disable radio if received last one, esle restart receiver
			 { 
				 NRF_RADIO->TASKS_DISABLE=1;
				 m_radio_state = IDLE_STATE;
			 }
			 else
			 {	 
				 NRF_RADIO->TASKS_START =1;  //restart rx
				 m_radio_state = CENTRAL_RX_STATE;
			 }
		}
	 }
}



void radio_start_poll(void)
{

	RADIO_TIMER->CC[0]        = EVENT_US;
	RADIO_TIMER->TASKS_CLEAR =1;
	RADIO_TIMER->TASKS_START =1;
	
}



void radio_start_receive(void)
{	
	RADIO_TIMER->CC[0]        = RF_RX_SEARCH_PERIOD;
	RADIO_TIMER->TASKS_CLEAR =1;
	RADIO_TIMER->TASKS_START =1;
	   
    NRF_RADIO->TASKS_RXEN = 1;
	
	rx_state = RX_SEARCH;
	m_radio_state =  PERIPH_RX_STATE;
}


uint8_t radio_get_poll_packet(void)
{
    memcpy(m_rx_buf, &dma_buf[1],dma_buf[0]);
    return dma_buf[0];
}


/**
 * @brief Handler for radio timer events.
 */
void radio_timer_irq_handler(void)
{
   if (RADIO_TIMER->EVENTS_COMPARE[0] == 1)
   {
		RADIO_TIMER->EVENTS_COMPARE[0] =0;         //clear timer compare event

		nrf_gpio_pin_toggle(PIN_CHANNEL_HOP);
	  
     
		if(on_radio_timer_interrupt)
			on_radio_timer_interrupt();

   }
	
	if (RADIO_TIMER->EVENTS_COMPARE[1] == 1)
	{
		RADIO_TIMER->EVENTS_COMPARE[1] =0;         //clear timer compare event
			
		RADIO_TIMER->CC[1] = 0;	
		RADIO_TIMER->INTENSET =   TIMER_INTENSET_COMPARE0_Msk ;
		 
		on_delay_timer_expired();
		 
	}
}

ISR_DIRECT_DECLARE(RADIO_TIMER_IRQHandler)
{
	radio_timer_irq_handler();
	return 1;
}



/**
 * @brief Handler for radio timer events.
 */
void  radio_irq_handler(void)
{

    if (NRF_RADIO->EVENTS_END && (NRF_RADIO->INTENSET & RADIO_INTENSET_END_Msk))
    {
        NRF_RADIO->EVENTS_END = 0;      
			
		if(m_is_central)
		{					
			on_central_end();				
		}
    }

    if (NRF_RADIO->EVENTS_DISABLED && (NRF_RADIO->INTENSET & RADIO_INTENSET_DISABLED_Msk))
    {
			
        NRF_RADIO->EVENTS_DISABLED = 0;
			
		if(on_radio_disabled_interrupt)
			on_radio_disabled_interrupt();
 
    }

}


ISR_DIRECT_DECLARE(RADIO_IRQHandler)
{
	radio_irq_handler();
	return 1;
}


void radio_setup(radio_init_t init)
{

    m_event_callback = init.event_callback;
    m_is_central = init.is_central;
	m_tx_buf = init.tx_buf;
	m_rx_buf = init.rx_buf;
	m_tx_length = init.tx_length;

	
    clocks_start();
    
	on_radio_timer_interrupt = (m_is_central)? timer_tx_event_handler:timer_rx_event_handler;
	
	on_radio_disabled_interrupt = (m_is_central)? on_central_disabled:on_periph_disabled;

    NRF_RADIO->MODECNF0     |= 1;
    
    NRF_RADIO->POWER        = 1;
	
    
    NRF_RADIO->PCNF0        =   (((0UL) << RADIO_PCNF0_S0LEN_Pos                               ) & RADIO_PCNF0_S0LEN_Msk)
                              | (((0UL) << RADIO_PCNF0_S1LEN_Pos                               ) & RADIO_PCNF0_S1LEN_Msk)
                              | (((8UL) << RADIO_PCNF0_LFLEN_Pos                               ) & RADIO_PCNF0_LFLEN_Msk);

    NRF_RADIO->PCNF1        =   (((RADIO_PCNF1_ENDIAN_Big )      << RADIO_PCNF1_ENDIAN_Pos    ) & RADIO_PCNF1_ENDIAN_Msk)
                              | (((3UL)                           << RADIO_PCNF1_BALEN_Pos     ) & RADIO_PCNF1_BALEN_Msk)
                              | (((0UL)                           << RADIO_PCNF1_STATLEN_Pos   ) & RADIO_PCNF1_STATLEN_Msk)
                              | ((((uint32_t) MAX_PACKET_LENGTH)                 << RADIO_PCNF1_MAXLEN_Pos    ) & RADIO_PCNF1_MAXLEN_Msk)
                              | ((RADIO_PCNF1_WHITEEN_Disabled     << RADIO_PCNF1_WHITEEN_Pos   ) & RADIO_PCNF1_WHITEEN_Msk);
   
	NRF_RADIO->CRCCNF       =   (((RADIO_CRCCNF_SKIPADDR_Include)    << RADIO_CRCCNF_SKIPADDR_Pos ) & RADIO_CRCCNF_SKIPADDR_Msk)
								| (((RADIO_CRCCNF_LEN_Two)        		 << RADIO_CRCCNF_LEN_Pos      ) & RADIO_CRCCNF_LEN_Msk);
	NRF_RADIO->CRCINIT 			= 0xFFFFUL;
	NRF_RADIO->CRCPOLY      = 0x11021UL;
	NRF_RADIO->SHORTS       = ((1 << RADIO_SHORTS_READY_START_Pos) | (1 << RADIO_SHORTS_END_DISABLE_Pos)
                            | RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_DISABLED_RSSISTOP_Msk);
    
    switch(init.mode)
    {	
        case MODE_1_MBIT:
            NRF_RADIO->MODE         = (RADIO_MODE_MODE_Ble_1Mbit        << RADIO_MODE_MODE_Pos       ) & RADIO_MODE_MODE_Msk;
            break;
        case MODE_2_MBIT:
            NRF_RADIO->MODE         = (RADIO_MODE_MODE_Ble_2Mbit        << RADIO_MODE_MODE_Pos       ) & RADIO_MODE_MODE_Msk;
            break;
        default:
            break;
    }

	//Radio interrupt settings
	IRQ_DIRECT_CONNECT( RADIO_IRQn, 6 ,
						RADIO_IRQHandler, 0);
	irq_enable(RADIO_IRQn);					
	NVIC_ClearPendingIRQ(RADIO_IRQn);
	NVIC_SetPriority(RADIO_IRQn, 5);
	NVIC_EnableIRQ(RADIO_IRQn);     
	NRF_RADIO->INTENSET     = (1 << RADIO_INTENSET_DISABLED_Pos);     
       
	 // Configure radio address registers according to default values
    NRF_RADIO->BASE0   = addr_conv(m_radio_addr.base_addr_p0);
    NRF_RADIO->BASE1   = addr_conv(m_radio_addr.base_addr_p1);
    NRF_RADIO->PREFIX0 = bytewise_bit_swap(&m_radio_addr.pipe_prefixes[0]);
    NRF_RADIO->PREFIX1 = bytewise_bit_swap(&m_radio_addr.pipe_prefixes[4]);
		
	// Configure radio address registers according to ESB default values
//    NRF_RADIO->BASE0   = 0xE7E7E7E7;
//    NRF_RADIO->BASE1   = 0x43434343;
//    NRF_RADIO->PREFIX0 = 0x23C343E7;
//    NRF_RADIO->PREFIX1 = 0x13E363A3;
		
    NRF_RADIO->PACKETPTR    = (uint32_t) dma_buf;
    

	// Radio Timer IRQ settings
	IRQ_DIRECT_CONNECT( RADIO_TIMER_IRQn, 6 ,
						RADIO_TIMER_IRQHandler, 0);
	irq_enable(RADIO_TIMER_IRQn);			
	NVIC_ClearPendingIRQ(RADIO_TIMER_IRQn);
	NVIC_SetPriority(RADIO_TIMER_IRQn,6);
	NVIC_EnableIRQ(RADIO_TIMER_IRQn);
	RADIO_TIMER->INTENSET =   TIMER_INTENSET_COMPARE0_Msk ;
 
   
   // Configure the radio timer with a 1 MHz base frequency
	RADIO_TIMER->PRESCALER = 4;
	RADIO_TIMER->BITMODE   = TIMER_BITMODE_BITMODE_32Bit;
	RADIO_TIMER->SHORTS    = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
	


	radio_set_tx_power(init.tx_power);

	if (m_is_central)
	{	
		NRF_RADIO->TXADDRESS = 0;
		NRF_RADIO->RXADDRESSES = 0xFF;   //turn on all pipes
		
	}
	
	else
	{			
			NRF_RADIO->TXADDRESS =  PERIPH_NUM;		
			NRF_RADIO->RXADDRESSES = 1;     //turn on pipe 0 only
	}


    NRF_RADIO->FREQUENCY    = RF_CHANNEL_TAB[m_rf_chan_idx];
    
    memset(dma_buf, 0, sizeof(dma_buf));

  
}
