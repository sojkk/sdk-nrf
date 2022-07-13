/**********************************
 *
 * Radio Config for 1 to 1 gaming 
 *
 **********************************/

#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "nrf.h"

#define NUM_OF_PERIPHS	1

#define TUNE_MODE   false


/**
   Debug Pins
*/
/********************************************************************************************/
#define PIN_CHANNEL_HOP       31
#define PIN_DATA_RECEIVED     30
#define PIN_DATA_TX           29

/**
   RADIO parameters
*/
/********************************************************************************************/

#define LATENCY                   70
#define T2_ADJ                    246-LATENCY  
#define EVENT_US                  500         // 500us
#define RF_CHAN_TAB_SIZE          15
#define RF_RX_OPERATE_PERIOD      EVENT_US
#define RF_RX_SEARCH_PERIOD       RF_RX_OPERATE_PERIOD* (RF_CHAN_TAB_SIZE +1)
#define RF_RX_LOSS_PERIOD         RF_RX_SEARCH_PERIOD
#define RF_RX_OPERATE_CNT         EVENT_US - T2_ADJ

#define PERIPH_NUM					   1
#define PERIPH_TX_DELAY_OFFSET		280
#define PERIPH_TX_DELAY_PERIOD		140


#define	PERIPH_PKT_SIZE						16

static const uint8_t  RF_CHANNEL_TAB[] =  {  6, 28, 52, 76, 24, \
                                            48, 72, 20, 44, 68, \
                                            16, 40, 64, 12, 36    };


/*
static const uint8_t  RF_CHANNEL_TAB[] =  {  6, 6, 6, 6, 6, \
                                             6, 6, 6, 6, 6, \
                                             6, 6, 6, 6, 6 };
 */                                         


#define     RADIO_PIPE_COUNT                8  																						
																						
#define	RADIO_ADDR_DEFAULT															\
{																					\
	 .base_addr_p0       = { 0xE7, 0xE7, 0xE7, 0xE7 },								\
	 .base_addr_p1       = { 0xC2, 0xC2, 0xC2, 0xC2 },								\
	 .pipe_prefixes      = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 },   	\
	 .num_pipes          = RADIO_PIPE_COUNT                                   		\
}



#endif //RADIO_H
