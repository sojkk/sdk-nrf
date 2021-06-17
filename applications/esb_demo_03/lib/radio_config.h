#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

#define DATA_SENDING_P0             3
#define DATA_SENDING_P1             4
#define DATA_SENDING_P2             28
#define DATA_SENDING_P3             29
#define DATA_SENDING_P4             30
#define DATA_SENDING_P5             31

#define NUM_OF_PERIPH               2

#define	PAIRING_PIPE                0
#define	DATA_PIPE                   1

#define PAIRING_ADDRESS             {0xE7, 0xE7, 0xE7, 0xE7}
#define SYSTEM_ADDRESS              {0xC2, 0xC2, 0xC2, 0xC2}


#define LOG_CNT                     1000

#define RADIO_CHAN_TAB              {6, 58,  78 }
#define	RADIO_CHAN_TAB_SIZE         3

#define	RETRAN_CNT                  0

#define ADJ                         25
#define POLL_TICKS                  30 //30.5 us  PER TICK  


#define RX_SEARCH_PERIOD            POLL_TICKS * (NUM_OF_PERIPH+2)
#define RX_OPERATE_PERIOD_W_WAIT    POLL_TICKS * (NUM_OF_PERIPH) - 2*ADJ 
#define	RX_WAIT_FOR_ACK_WR_PERIOD			ADJ
#define RX_OPERATE_PERIOD           POLL_TICKS * (NUM_OF_PERIPH)
	
#define RX_LOSS_THRESHOLD           RADIO_CHAN_TAB_SIZE * 2

#endif
