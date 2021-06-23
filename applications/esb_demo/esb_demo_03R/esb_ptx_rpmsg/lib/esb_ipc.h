#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifndef ESB_IPC_H
#define ESB_IPC_H


#define INVALID_MODE	0xFF
#define	CONFIG_PTX		1
#define	CONFIG_PRX		2

typedef enum
{
	APP_IDLE,
	APP_CFG,
	
	
}app_state_t;

typedef enum
{
	ESB_INITIALIZE = 1,
	WRITE_TX_PAYLOAD,
	READ_RX_PAYLOAD,
	START_TX,
	START_RX,
	STOP_RX,
	TX_SUCCESS,
	TX_FAILED,
	RX_RECEIVED,
	
}cmd_evt_t;

typedef struct
{
    cmd_evt_t 	data_hdr;
    uint8_t 	data_len;
    uint8_t 	data[32];
    
}ipc_msg_t;

#endif