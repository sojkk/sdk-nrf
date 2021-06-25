#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifndef ESB_IPC_H
#define ESB_IPC_H


#define INVALID_MODE	0xFF
#define	CONFIG_CENTRAL		1
#define	CONFIG_PERIPH		2

typedef enum
{
	APP_IDLE,
	APP_CFG,
	APP_OPT
	
	
}app_state_t;

typedef enum
{
	RADIO_INITIALIZE = 1,
	RADIO_START_TX_POLL,
	RADIO_START_RX_SCAN,
	RADIO_FETCH_PACKET,
	RADIO_CENTRAL_DATA_RCV,
	RADIO_PERIPH_DATA_SND

}cmd_evt_t;

typedef struct
{
    cmd_evt_t 	data_hdr;
    uint8_t 	data_len;
    uint8_t 	data[65];
    
}ipc_msg_t;

#endif