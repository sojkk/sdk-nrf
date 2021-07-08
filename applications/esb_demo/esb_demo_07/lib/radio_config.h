#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

#if defined(CONFIG_SOC_SERIES_NRF53X)
#define DATA_SENDING_P0             4
#define DATA_SENDING_P1             5
#define DATA_SENDING_P2             6
#define DATA_SENDING_P3             7
#define DATA_SENDING_P4             25
#define DATA_SENDING_P5             26
#else
#define DATA_SENDING_P0             3
#define DATA_SENDING_P1             4
#define DATA_SENDING_P2             28
#define DATA_SENDING_P3             29
#define DATA_SENDING_P4             30
#define DATA_SENDING_P5             31
#endif

#define NUM_OF_PERIPH               3

#define	DATA_PIPE                   0


#define SYSTEM_ADDRESS              {0xC2, 0xE7, 0xC2, 0xE7}


#define PERIPH_ON_SYNC_CNT			100

#define LOG_CNT                     1000


#define RADIO_CHAN_TAB 				{2, 42, 72, 12, 22, 32, 52, 62, 78}		
#define TOTAL_CHAN_TAB_SIZE			9
#define	RADIO_CHAN_TAB_SIZE		 	3
#define	RADIO_UNUSED_CHAN_TAB_SIZE	 (TOTAL_CHAN_TAB_SIZE - RADIO_CHAN_TAB_SIZE)
#define CENTRAL_SWITCH_CHAN_CNT		5//3

#define	RETRAN_CNT                  0

#define ADJ                         25
#define POLL_TICKS                  30 //30.5 us  PER TICK  


#define RX_SEARCH_PERIOD            POLL_TICKS * (NUM_OF_PERIPH+2)
#define RX_OPERATE_PERIOD_W_WAIT    POLL_TICKS * (NUM_OF_PERIPH) - 2*ADJ 
#define	RX_WAIT_FOR_ACK_WR_PERIOD			ADJ
#define RX_OPERATE_PERIOD           POLL_TICKS * (NUM_OF_PERIPH)
	
#define RX_LOSS_THRESHOLD           RADIO_CHAN_TAB_SIZE *NUM_OF_PERIPH* 2

//aFH
#define CENTRAL_LOSS_THRESHOLD  6//4

#define BCT_PKT_LENGTH			4

#endif
