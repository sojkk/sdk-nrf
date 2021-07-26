#include "esb_timeslot.h"
#include "nrf_esb.h"

#include <kernel.h>
#include <sys/printk.h>
#include <zephyr/types.h>
#include <irq.h>

#include <mpsl_timeslot.h>
#include <mpsl.h>
#include <hal/nrf_timer.h>


/**@brief The NRF RADIO multitimer. */
#define NRF_RADIO_MULTITIMER          NRF_TIMER0
#define NRF_RADIO_MULTITIMER_IRQn     TIMER0_IRQn

#define TIMESLOT_BEGIN_IRQn         COMP_LPCOMP_IRQn
#define TIMESLOT_BEGIN_IRQHandler   COMP_LPCOMP_IRQHandler
#define TIMESLOT_BEGIN_IRQPriority  5 //3
#define TIMESLOT_END_IRQn           QDEC_IRQn
#define TIMESLOT_END_IRQHandler     QDEC_IRQHandler
#define TIMESLOT_END_IRQPriority    5 //3



#define TS_LEN_US                   (5000UL)
#define TX_LEN_EXTENSION_US         (5000UL)
#define TS_SAFETY_MARGIN_US         (700UL)    /**< The timeslot activity should be finished with this much to spare. */
#define TS_EXTEND_MARGIN_US         (2000UL) 



/**@brief Debug pin configuration. */
#define ESB_TIMESLOT_DEBUG_ENABLE   0
#if ESB_TIMESLOT_DEBUG_ENABLE
#define ESB_TIMESLOT_DBG_PIN_RADIO_TIMESLOT     2   /**< Pin which indicates radio time-slot being active. */
#define ESB_TIMESLOT_DBG_PIN_RADIO_IRQHANDLER   3   /**< Pin which indicates the activity of RADIO_IRQHandler. */
#define ESB_TIMESLOT_DBG_PIN_DISABLE            4   /**< Pin which indicates ESB radio being disabled. */

#define ESB_TIMESLOT_DEBUG_PIN_SET(x)           nrf_gpio_pin_set(x)
#define ESB_TIMESLOT_DEBUG_PIN_CLEAR(x)         nrf_gpio_pin_clear(x)
#else
#define ESB_TIMESLOT_DEBUG_PIN_SET(x)
#define ESB_TIMESLOT_DEBUG_PIN_CLEAR(x)
#endif


/*****************************************************************************/
/** @name Configuration */
/*****************************************************************************/
#if BUTTONS_NUMBER < 1
#error "Not enough buttons on board"
#endif

#if LEDS_NUMBER < 1
#error "Not enough LEDs on board"
#endif

#define DEBUG_SIG_ENABLE

// Define pip
#define DATA_PIPE  0

// Define addresses
#define BASE_ADDR_0      {3, 6, 9, 12}
#define BASE_ADDR_1      {6, 8, 11, 20}
#define PREFIX_ADDR_0    0x01
#define PREFIX_ADDR_1    0x02
#define PREFIX_ADDR_2    0x03

#define RF_CHANNEL                  3     //2403MHz


#define DBG_SIG_TIMSLOTS                    3   
#define DBG_SIG_SEND_PKT                    4
#define DBG_SIG_TX_SUCCESS                  28     
#define DBG_SIG_RETX                        29    
#define DBG_SIG_TX_FAIL                     30
 

static volatile enum
{
    STATE_IDLE,                  /**< Default state. */
    STATE_RX                     /**< Waiting for packets. */
} m_state = STATE_IDLE;



static nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;

static nrf_esb_payload_t rx_buf;	

static uint8_t base_addr_0[4] =  BASE_ADDR_0;
static uint8_t base_addr_1[4] =  BASE_ADDR_1;
static uint8_t addr_prefix[3] =  {PREFIX_ADDR_0, PREFIX_ADDR_1, PREFIX_ADDR_2};


static callback_t callback;


static void error(void)
{
	printk("ERROR!\n");
	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}



/**Constants for timeslot API
*/
static nrf_radio_request_t  m_timeslot_request;

static nrf_radio_signal_callback_return_param_t signal_callback_return_param;
static uint32_t m_total_timeslot_length = 0;
void RADIO_IRQHandler(void);

static void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
     switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:

            break;
        case NRF_ESB_EVENT_TX_FAILED:
					
            break;   
        case NRF_ESB_EVENT_RX_RECEIVED:
					
				NRF_LOG_DEBUG("ESB Event : RX_RECEIVED\r\n");
				APP_ERROR_CHECK ((ret_code_t) nrf_esb_read_rx_payload(&rx_buf) );
        callback(rx_buf.data);
			
            break;					
    }
}

void esb_config( void )
{
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
    nrf_esb_config.selective_auto_ack       = false;
	  nrf_esb_config.retransmit_count         = 0;
	
	  rx_buf.pipe = DATA_PIPE;
}

uint32_t esb_init(void)
{
    uint32_t err_code;
	
    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);
		
		err_code = nrf_esb_set_rf_channel(RF_CHANNEL);
		VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 3);
    VERIFY_SUCCESS(err_code);
		
	

	  return err_code;

}	


/* Timeslot requests */
static mpsl_timeslot_request_t timeslot_request_earliest = {
	.request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST,
	.params.earliest.hfclk = MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE,
	.params.earliest.priority = MPSL_TIMESLOT_PRIORITY_NORMAL,
	.params.earliest.length_us = TS_LENGTH_US,
	.params.earliest.timeout_us = 1000000
};

static mpsl_timeslot_request_t timeslot_config_earliest = {
	.request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST,
	.params.normal.hfclk = MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE,
	.params.normal.priority = MPSL_TIMESLOT_PRIORITY_NORMAL,
	.params.normal.length_us = TS_LENGTH_US
	.params.earliest.timeout_us = 1000000
};

static mpsl_timeslot_signal_return_param_t signal_callback_return_param;


/**@brief Timeslot event handler
 */
static mpsl_timeslot_signal_return_param_t *mpsl_timeslot_callback(mpsl_timeslot_session_id_t session_id, uint32_t signal_type)
{
	(void) session_id; /* unused parameter */

	mpsl_timeslot_signal_return_param_t *p_ret_val = NULL;
	
	int err;

	switch (signal_type) {

	case MPSL_TIMESLOT_SIGNAL_START:
		//Start of the timeslot - set up timer interrupt					
        signal_callback_return_param.params.request.p_next = NULL;
		signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
		p_ret_val = &signal_callback_return_param;

		/* Setup timer to trigger an interrupt (and thus the TIMER0
		 * signal) before timeslot end. 
		 */
		NRF_TIMER0->TASKS_STOP          = 1;
		NRF_TIMER0->TASKS_CLEAR         = 1;
		NRF_TIMER0->MODE                = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
		NRF_TIMER0->EVENTS_COMPARE[0]   = 0;
		NRF_TIMER0->EVENTS_COMPARE[1]   = 0;
		NRF_TIMER0->INTENSET            = TIMER_INTENSET_COMPARE0_Msk | TIMER_INTENSET_COMPARE1_Msk ;
		NRF_TIMER0->CC[0]               = TS_LEN_US - TS_SAFETY_MARGIN_US;
		NRF_TIMER0->CC[1]               = TS_LEN_US - TS_EXTEND_MARGIN_US;
		NRF_TIMER0->BITMODE             = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);
		NRF_TIMER0->TASKS_START         = 1;
		NRF_RADIO->POWER                = (RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos);

		NVIC_EnableIRQ(TIMER0_IRQn); 
		NVIC_SetPendingIRQ(TIMESLOT_BEGIN_IRQn);
		
		//nrf_gpio_pin_set(DBG_SIG_TIMSLOTS);	

		break;
		
	case MPSL_TIMESLOT_SIGNAL_RADIO:
	
		signal_callback_return_param.params.request.p_next = NULL;
        signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
		p_ret_val = &signal_callback_return_param;
		
        RADIO_IRQHandler();

		break;
		
	case MPSL_TIMESLOT_SIGNAL_TIMER0:
	
		//nrf_gpio_pin_clear(DBG_SIG_TIMSLOTS);

		if (NRF_TIMER0->EVENTS_COMPARE[0] &&
			(NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENCLR_COMPARE0_Pos)))
		{
				NRF_TIMER0->TASKS_STOP  = 1;
				NRF_TIMER0->EVENTS_COMPARE[0] = 0;
				// This is the "timeslot is about to end" timeout
				if (!nrf_esb_is_idle())
				{
					NRF_RADIO->INTENCLR      = 0xFFFFFFFF;
					NRF_RADIO->TASKS_DISABLE = 1;
				}
				// Schedule next timeslot
				configure_next_event_earliest();
				signal_callback_return_param.params.request.p_next = &m_timeslot_request;
				signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
									
		}

		if (NRF_TIMER0->EVENTS_COMPARE[1] &&
			 (NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENCLR_COMPARE1_Pos)))
		{
			NRF_TIMER0->EVENTS_COMPARE[1] = 0;
					
			if (m_total_timeslot_length < (128000000UL - 1UL - TX_LEN_EXTENSION_US))
			{
					// Request timeslot extension if total length does not exceed 128 seconds
				signal_callback_return_param.params.extend.length_us = TX_LEN_EXTENSION_US;
				signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
			}
			else
			{
				// Return with no action request
				signal_callback_return_param.params.request.p_next = NULL;
				signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
			}
		}
	
		p_ret_val = &signal_callback_return_param;

		break;


	case MPSL_TIMESLOT_SIGNAL_EXTEND_SUCCEEDED:
		 // NRF_LOG("  NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED\r\n");
            NRF_TIMER0->TASKS_STOP          = 1;
            NRF_TIMER0->EVENTS_COMPARE[0]   = 0;
            NRF_TIMER0->EVENTS_COMPARE[1]   = 0;
            NRF_TIMER0->CC[0]               += (TX_LEN_EXTENSION_US - 25);
            NRF_TIMER0->CC[1]               += (TX_LEN_EXTENSION_US - 25);
            NRF_TIMER0->TASKS_START         = 1;
            m_total_timeslot_length += TX_LEN_EXTENSION_US;
            NVIC_SetPendingIRQ(TIMESLOT_BEGIN_IRQn);
				
		//nrf_gpio_pin_set(DBG_SIG_TIMSLOTS);
	
	
		break;
				
	
	case MPSL_TIMESLOT_SIGNAL_EXTEND_FAILED:
	
		//Try scheduling a new timeslot
		// Disabling UESB is done in a lower interrupt priority 
		NVIC_SetPendingIRQ(TIMESLOT_END_IRQn);

		signal_callback_return_param.params.request.p_next = NULL;
		signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
	
	
	
		break;

		
	case MPSL_TIMESLOT_SIGNAL_SESSION_IDLE:
	
	
	
		break;
		

	case MPSL_TIMESLOT_SIGNAL_BLOCKED:
	
		//Fall through
		
	case MPSL_TIMESLOT_SIGNAL_CANCELLED:

		err = mpsl_timeslot_request(session_id, &timeslot_request_earliest);
		if (err) {
			error();
		}
		
		break;
		
	case MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED:
		break;
		
	default:
		printk("unexpected signal: %u", signal_type);
		error();
		break;
	}

	return p_ret_val;
}



/**@brief Function for initializing the timeslot API.
 */
void  timeslot_sd_start(void)
{
    int err;


	err = mpsl_timeslot_session_open(
					mpsl_timeslot_callback,
					&session_id);
	if (err) {
		error();
	}


	err = mpsl_timeslot_request(
					session_id,
					&timeslot_request_earliest);
	if (err) {
		error();
	}
}

void timeslot_sd_stop(void)
{
	int err;
	
   /* Initialize to invalid session id */
	mpsl_timeslot_session_id_t session_id = 0xFFu;
	
	err = mpsl_timeslot_session_close(session_id);
	
	if (err) {
		error();
	}
	
}

/**@brief IRQHandler used for execution context management. 
  *        Any available handler can be used as we're not using the associated hardware.
  *        This handler is used to stop and disable UESB
  */
void TIMESLOT_END_IRQHandler(void)
{
    uint32_t err_code;
    ESB_TIMESLOT_DEBUG_PIN_SET(ESB_TIMESLOT_DBG_PIN_DISABLE);

    // Timeslot is about to end: stop UESB
	 if(m_state == STATE_RX)
	 {	 
    err_code= nrf_esb_stop_rx();
	  APP_ERROR_CHECK(err_code);
		 
		err_code= nrf_esb_flush_rx();
    APP_ERROR_CHECK(err_code); 
		 
    err_code= nrf_esb_disable();
    APP_ERROR_CHECK(err_code);

    m_total_timeslot_length = 0; 
		 
		m_state = STATE_IDLE; 
	 }
	
	  
	
    m_total_timeslot_length = 0;
    
    ESB_TIMESLOT_DEBUG_PIN_CLEAR(ESB_TIMESLOT_DBG_PIN_DISABLE);
    ESB_TIMESLOT_DEBUG_PIN_CLEAR(ESB_TIMESLOT_DBG_PIN_RADIO_TIMESLOT);
}

/**@brief IRQHandler used for execution context management. 
  *        Any available handler can be used as we're not using the associated hardware.
  *        This handler is used to initiate UESB RX/TX
  */
void TIMESLOT_BEGIN_IRQHandler(void)
{
   
	ESB_TIMESLOT_DEBUG_PIN_SET(ESB_TIMESLOT_DBG_PIN_RADIO_TIMESLOT);
	
	 CRITICAL_REGION_ENTER();
   {
	
	     if (m_state == STATE_IDLE)
       {
				 
			    (void) esb_init();
			
	     }
		
		  
		
		  uint32_t err_code = nrf_esb_start_rx();
      APP_ERROR_CHECK(err_code);
      
			m_state = STATE_RX;
	
   
      
  }
  CRITICAL_REGION_EXIT();
}


void timeslot_sd_init(callback_t process_function)
{
    esb_config();
        
    // Using three avilable interrupt handlers for interrupt level management
    // These can be any available IRQ as we're not using any of the hardware,
    // simply triggering them through software
    NVIC_ClearPendingIRQ(TIMESLOT_END_IRQn);
    NVIC_SetPriority(TIMESLOT_END_IRQn, TIMESLOT_END_IRQPriority);
    NVIC_EnableIRQ(TIMESLOT_END_IRQn);

    NVIC_ClearPendingIRQ(TIMESLOT_BEGIN_IRQn);
    NVIC_SetPriority(TIMESLOT_BEGIN_IRQn, TIMESLOT_BEGIN_IRQPriority);
    NVIC_EnableIRQ(TIMESLOT_BEGIN_IRQn);
	
	
	
	  //for test only
	  nrf_gpio_cfg_output(DBG_SIG_TIMSLOTS);
	  nrf_gpio_pin_clear(DBG_SIG_TIMSLOTS);
	
	  callback = process_function;

    return NRF_SUCCESS;
}


