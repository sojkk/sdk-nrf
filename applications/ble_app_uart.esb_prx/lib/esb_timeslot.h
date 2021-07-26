#ifndef TIMESLOT_H__
#define TIMESLOT_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
//#include "softdevice_handler.h"
#include "nrf_soc.h"
#include "boards.h"


#include "nrf_esb.h"


typedef void (*esb_timeslot_data_handler_t)(void * p_data, uint16_t length);

typedef void (*callback_t)(uint8_t *);    //callback function for esb handler


/**@brief Radio event handler
*/
void RADIO_timeslot_IRQHandler(void);



/**@brief Configure next timeslot event in normal configuration
 */
void configure_next_event_normal(void);


/**@brief Timeslot signal handler
 */
void nrf_evt_signal_handler(uint32_t evt_id);


/**@brief Timeslot event handler
 */
nrf_radio_signal_callback_return_param_t * radio_callback(uint8_t signal_type);


/**@brief Function for initializing and start the timeslot API.
 */
void timeslot_sd_start(void);

/**@brief Function for stopping the timeslot API.
 */
void timeslot_sd_stop(void);

uint32_t timeslot_sd_init(callback_t  process_function);

#endif
