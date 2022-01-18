/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */


// Define pipe
#define NRFR_DATA_PIPE   0


//Define addresses
#define SYSTEM_ADDRESS     {3, 6, 9, 12}


//define Radio channels
#define     JETSTR_CHANNEL_TAB                {3, 23, 40, 61, 75}    
#define     JETSTR_CHANNEL_TAB_SIZE            5


/**
USB report ID: Mouse motion.
*/
#define NRFR_DATA_HDR               0x01

#define NRFR_CTRL_HDR               0x10


  #define     JETSTR_RX_PERIOD                  1400 //128 bytes payload
//#define     JETSTR_RX_PERIOD                  800 //64 bytes payload
//#define     JETSTR_RX_PERIOD 									750 //25 bytes payload		 
//#define     JETSTR_RX_PERIOD 									615 //10 bytes or less             
#define     JETSTR_RX_DELAY                    160
#define     JETSTR_RETRAN_CNT_IN_SYNC          2
#define     JETSTR_RETRAN_CNT_CHAN_SW          3
#define     JETSTR_RETRAN_CNT_OUT_OF_SYNC      8
#define     JETSTR_RX_RETRAN                   JETSTR_RX_PERIOD/(JETSTR_RETRAN_CNT_IN_SYNC +1)

