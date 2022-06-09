#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

#include <stdint.h>

#define TUNE_MODE   false


/**
   Debug Pins
*/
/********************************************************************************************/

#define PIN_DATA_RECEIVED   30
#define PIN_CHANNEL_HOP     31


/**
   RADIO parameters
*/
/********************************************************************************************/
#define FAST_RAMP									40
#define LATENCY                   12//80//
#define T2_ADJ                    233- LATENCY -FAST_RAMP // 246-LATENCY // for 32 bytes payload in fast mode
#define EVENT_US                  1000         // 1ms
#define RF_CHAN_TAB_SIZE          15
#define RF_RX_OPERATE_PERIOD      EVENT_US
#define RF_RX_SEARCH_PERIOD       RF_RX_OPERATE_PERIOD* (RF_CHAN_TAB_SIZE +1)
#define RF_RX_LOSS_PERIOD         RF_RX_SEARCH_PERIOD
#define RF_RX_OPERATE_CNT         EVENT_US - T2_ADJ




static const uint8_t  RF_CHANNEL_TAB[] =  {  6, 28, 52, 76, 24, \
                                            48, 72, 20, 44, 68, \
                                            16, 40, 64, 12, 36    };


/*
static const uint8_t  RF_CHANNEL_TAB[] =  {  6, 6, 6, 6, 6, \
                                             6, 6, 6, 6, 6, \
                                             6, 6, 6, 6, 6 };
 */                                         
	



#endif //RADIO_H
