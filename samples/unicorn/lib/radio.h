#ifndef RADIO_H
#define RADIO_H

#include <stdbool.h>
#include <stdint.h>
#ifdef NRF52840_XXAA
#include "nrf52840_bitfields.h"
#endif
#ifdef NRF52811_XXAA
#include "nrf52811_bitfields.h"
#endif
#include "nrfx_timer.h"


#define RADIO_TIMER              NRF_TIMER0
#define RADIO_TIMER_IRQn         TIMER0_IRQn
#define RADIO_TIMER_IRQHandler   TIMER0_IRQHandler


typedef enum
{
    MODE_2_MBIT,
    MODE_1_MBIT,
#ifdef NRF52840_XXAA		
    MODE_500_KBIT,
    MODE_125_KBIT,
#endif		    
} radio_modes_t;


/**@brief Enhanced ShockBurst radio transmission power modes. */
typedef enum {
#ifdef NRF52840_XXAA	
    RADIO_POWER_8DBM     = RADIO_TXPOWER_TXPOWER_Pos8dBm,  /**< 8 dBm radio transmit power.   */
#endif	
    RADIO_TX_POWER_4DBM     = RADIO_TXPOWER_TXPOWER_Pos4dBm,  /**< 4 dBm radio transmit power.   */
    RADIO_TX_POWER_3DBM     = RADIO_TXPOWER_TXPOWER_Pos3dBm,  /**< 3 dBm radio transmit power.   */
    RADIO_TX_POWER_0DBM     = RADIO_TXPOWER_TXPOWER_0dBm,     /**< 0 dBm radio transmit power.   */
    RADIO_TX_POWER_NEG4DBM  = RADIO_TXPOWER_TXPOWER_Neg4dBm,  /**< -4 dBm radio transmit power.  */
    RADIO_TX_POWER_NEG8DBM  = RADIO_TXPOWER_TXPOWER_Neg8dBm,  /**< -8 dBm radio transmit power.  */
    RADIO_TX_POWER_NEG12DBM = RADIO_TXPOWER_TXPOWER_Neg12dBm, /**< -12 dBm radio transmit power. */
    RADIO_TX_POWER_NEG16DBM = RADIO_TXPOWER_TXPOWER_Neg16dBm, /**< -16 dBm radio transmit power. */
    RADIO_TX_POWER_NEG20DBM = RADIO_TXPOWER_TXPOWER_Neg20dBm, /**< -20 dBm radio transmit power. */
    RADIO_TX_POWER_NEG30DBM = RADIO_TXPOWER_TXPOWER_Neg30dBm, /**< -30 dBm radio transmit power. */
    RADIO_TX_POWER_NEG40DBM = RADIO_TXPOWER_TXPOWER_Neg40dBm  /**< -40 dBm radio transmit power. */
} radio_power_t;

typedef enum
{
    RADIO_EVENT_POLL_EXP,   /**< Event triggered on Central poll timer expire    */
    RADIO_EVENT_POLL_RCV     /**< Event triggered on Peripheral recieved poll packet.     */

} radio_evt_id_t;


typedef enum
{
  RF_SEARCH,
  RF_OPERATE
}
rx_states_t;


typedef struct
{
    radio_evt_id_t    evt_id;                     //!< Enhanced ShockBurst event ID.
    uint8_t        		chan_cnt;                  //!< channel count
} radio_evt_t;


typedef void (*event_callback_t ) (radio_evt_t const * p_event);


typedef struct
{
	bool 							is_transmitter;
	radio_power_t 		tx_power;
	radio_modes_t			mode;
	event_callback_t 	event_callback;
	uint8_t * 				tx_buf;
	uint8_t *					rx_buf;
	uint8_t						tx_length;
	
} radio_init_t;

void radio_set_mode(radio_modes_t mode);
void radio_set_tx_power(int8_t power);


void radio_setup(radio_init_t radio_init);
void radio_receive_packet(void);
uint8_t radio_get_packet(void);

int8_t get_rssi(void);
bool get_crc(void);
bool is_packet_received(void);

#endif //RADIO_H
