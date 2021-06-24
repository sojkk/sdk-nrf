/*
 * Copyright (c) 2018-2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <zephyr/types.h>

#include <drivers/ipm.h>
#include <sys/printk.h>
#include <device.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nrf.h>
#include "esb.h"
#include "radio.h"

#include <ipc/rpmsg_service.h>

#include "radio_ipc.h"

#define APP_TASK_STACK_SIZE (1024)
K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;

static ipc_msg_t tx_event;

static ipc_msg_t rx_cmd;

static K_SEM_DEFINE(data_rx_sem, 0, 1);

static int ep_id;


static radio_data_t rx_buf;


static int send_message(void);

static void hfclk_start( void )
{
  //Start HFCLK
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}



void radio_evt_cb(uint8_t radio_event)
{
	
	int status = 0;

	if(radio_event==RADIO_CENTRAL_DATA_RECEIVED)
	{
		
		radio_fetch_packet(&rx_buf);
		

		tx_event.data_hdr = RADIO_CENTRAL_DATA_RECEIVED;
		tx_event.data_len = rx_buf.length;
		tx_event.data[0]  = rx_buf.periph_num;
		memcpy(&tx_event.data[1], rx_buf.data, rx_buf.length);
						
		status = send_message();
				
        if (status < 0) {

            printk("esb_event_hadler send_message failed with status %d\n", status);

        }

	}
}



int endpoint_cb(struct rpmsg_endpoint *ept, void *data,
		size_t len, uint32_t src, void *priv)
{
	memcpy (&rx_cmd, data, sizeof(rx_cmd));

	k_sem_give(&data_rx_sem);

	return RPMSG_SUCCESS;
}




static void receive_message(void)
{
	int err =0 ;
	struct esb_payload payload_buffer;
	
	payload_buffer.pipe		= 0;
	payload_buffer.noack	= false;

	k_sem_take(&data_rx_sem, K_FOREVER);

	switch (rx_cmd.data_hdr)		
	{
		
		case RADIO_INITIALIZE:
					
			tx_event.data_hdr	= RADIO_INITIALIZE;
			tx_event.data_len	= 1;
			
			if(rx_cmd.data[0] == CONFIG_CENTRAL)
			{	
				err = radio_setup(true, RADIO_TX_POWER_0DBM, radio_evt_cb, 0);
				tx_event.data[0]	= (uint8_t) (256-err);
			}
			else if (rx_cmd.data[0] == CONFIG_PERIPH)
			{
				err = radio_setup(false, RADIO_TX_POWER_0DBM, radio_evt_cb, 0);
				tx_event.data[0]	= (uint8_t) (256-err);
			}
			else
			{
				tx_event.data[0]	= INVALID_MODE;
				
			}
		
			if (err!=0) {
				printk("CPUNET: Radio initialization failed, err %d\n", (uint8_t) err);
				return;
			}
			
			
			break;
		
		case RADIO_START_TX_POLL:			
				
			tx_event.data_hdr	= RADIO_START_TX_POLL;
			tx_event.data_len	= 1;
			
			radio_poll_timer_start(rx_cmd.data[0]);
			 
			tx_event.data[0]=0;
		
			break;
				
		
		default :
		
		
			break;
		
			
			
		}

}


static int send_message(void)
{
	return rpmsg_service_send(ep_id, &tx_event, sizeof(tx_event));
}



void app_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	
	int status = 0;

     
	printk("\r\nRPMsg Service [remote] started\r\n");


	while(1) {
		
		
		receive_message();
		
		
		status = send_message();
				
		if (status < 0) {
			printk("send_message failed with status %d\n",
			       status);
			break;
		}

		
	}


}







void main(void)
{

	hfclk_start();	
	
	memset(&tx_event, 0, sizeof(tx_event));
	memset(&rx_cmd, 0, sizeof(rx_cmd));
	
	printk("Starting application thread!\n");
	k_thread_create(&thread_data, thread_stack, APP_TASK_STACK_SIZE,
			(k_thread_entry_t)app_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);


}

/* Make sure we register endpoint before RPMsg Service is initialized. */
int register_endpoint(const struct device *arg)
{
	int status;

	status = rpmsg_service_register_endpoint("demo", endpoint_cb);

	if (status < 0) {
		printk("rpmsg_create_ept failed %d\n", status);
		return status;
	}

	ep_id = status;

	return 0;
}

SYS_INIT(register_endpoint, POST_KERNEL, CONFIG_RPMSG_SERVICE_EP_REG_PRIORITY);
