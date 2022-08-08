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


#define MAX_PACKET_LENGTH 128

#define RADIO_TIMER              NRF_TIMER0
#define RADIO_TIMER_IRQn         TIMER0_IRQn
#define RADIO_TIMER_IRQHandler   TIMER0_IRQHandler


typedef enum
{
	
 IDLE_STATE=0,
 CENTRAL_TX_STATE,
 CENTRAL_RX_STATE,
 PERIPH_TX_STATE,
 PERIPH_RX_STATE	

}radio_states_t;


typedef enum
{
    MODE_2_MBIT,
    MODE_1_MBIT,    
} radio_modes_t;


/**@brief Enhanced ShockBurst radio transmission power modes. */
typedef enum {
#if defined(CONFIG_NRF52840)	
    RADIO_POWER_8DBM     = RADIO_TXPOWER_TXPOWER_Pos8dBm,  /**< 8 dBm radio transmit power.   */
#endif	
#if defined(CONFIG_NRF5340)
    RADIO_TX_POWER_3DBM     = RADIO_TXPOWER_TXPOWER_0dBm,  /**< 3 dBm radio transmit power.   */
#else
    RADIO_TX_POWER_4DBM     = RADIO_TXPOWER_TXPOWER_Pos4dBm,  /**< 4 dBm radio transmit power.   */
    RADIO_TX_POWER_3DBM     = RADIO_TXPOWER_TXPOWER_Pos3dBm,  /**< 3 dBm radio transmit power.   */
#endif
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
    RADIO_EVENT_CENTRAL_POLL_EXP,   /**< Event triggered on Central poll timer expire    */
    RADIO_EVENT_PERIPH_POLL_RCV,    /**< Event triggered on Peripheral recieved poll packet.  */
		RADIO_EVENT_PERIPH_POLL_NOT_RCV

} radio_evt_id_t;


typedef enum
{
  RX_SEARCH,
  RX_OPERATE
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
	bool				is_central;
	radio_power_t 		tx_power;
	radio_modes_t		mode;
	event_callback_t 	event_callback;
	uint8_t * 			tx_buf;
	uint8_t *			rx_buf;
	uint8_t				tx_length;
	
} radio_init_t;



typedef struct
{
    uint8_t base_addr_p0[4];        /**< Base address for pipe 0 encoded in big endian. */
    uint8_t base_addr_p1[4];        /**< Base address for pipe 1-7 encoded in big endian. */
    uint8_t pipe_prefixes[8];       /**< Address prefix for pipe 0 to 7. */
    uint8_t num_pipes;              /**< Number of pipes available. */
} radio_address_t;


typedef struct
{
    uint8_t length;                                 //!< Length of the packet (maximum value is @ref NRF_ESB_MAX_PAYLOAD_LENGTH).
    uint8_t pipe;                                   //!< Pipe used for this payload.
    int8_t  rssi;                                   //!< RSSI for the received packet.
    uint8_t data[MAX_PACKET_LENGTH];       //!< The payload data.
} radio_payload_t;



void radio_setup(radio_init_t init);
void radio_start_poll(void);
void radio_start_receive(void);
uint8_t radio_get_poll_packet(void);

int8_t get_rssi(void);
bool get_crc(void);
//bool is_packet_received(void);

#endif //RADIO_H
