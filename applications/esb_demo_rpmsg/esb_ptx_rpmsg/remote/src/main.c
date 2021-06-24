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
#include <esb.h>

#include <ipc/rpmsg_service.h>

#include "esb_ipc.h"

#define APP_TASK_STACK_SIZE (1024)
K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;

static ipc_msg_t tx_event;

static ipc_msg_t rx_cmd;

static K_SEM_DEFINE(data_rx_sem, 0, 1);

static int ep_id;

static int send_message(void);

static void hfclk_start( void )
{
  //Start HFCLK
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}



void radio_event_handler(struct esb_evt const *event)
{
	
	int status = 0;

	switch (event->evt_id) {
		case ESB_EVENT_TX_SUCCESS:
		
			tx_event.data_hdr = TX_SUCCESS;
			
			break;
		case ESB_EVENT_TX_FAILED:
			
			esb_flush_tx();  //Flush TX_FIFO
			tx_event.data_hdr = TX_FAILED;
			
			break;
		case ESB_EVENT_RX_RECEIVED:
		
		
			tx_event.data_hdr = RX_RECEIVED;
					
			break;
			
		default:
		
			break;
	}
	
	status = send_message();
				
        if (status < 0) {

            printk("esb_event_hadler send_message failed with status %d\n", status);

        }

	
}

static int esb_initialize(bool is_ptx)
{
	int err;
	/* These are arbitrary default addresses. In end user products
	 * different addresses should be used for each set of devices.
	 */
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.retransmit_delay = 600;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.event_handler = radio_event_handler;
	config.mode = (is_ptx)? ESB_MODE_PTX:ESB_MODE_PRX;
	config.selective_auto_ack = true;

	err = esb_init(&config);

	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}

	return 0;
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
		
		case ESB_INITIALIZE:
					
			tx_event.data_hdr	= ESB_INITIALIZE;
			tx_event.data_len	= 0;
			
			if(rx_cmd.data[0] == CONFIG_PTX)
			{	
				err = esb_initialize(true);
				tx_event.data[0]	= (uint8_t) (256-err);
			}
			else if (rx_cmd.data[0] == CONFIG_PRX)
			{
				err = esb_initialize(false);
				tx_event.data[0]	= (uint8_t) (256-err);
			}
			else
			{
				tx_event.data[0]	= INVALID_MODE;
				
			}
		
			if (err!=0) {
				printk("ESB initialization failed, err %d\n", (uint8_t) err);
				return;
			}
			
			
			break;
		
		case WRITE_TX_PAYLOAD:			
				
			payload_buffer.length	= rx_cmd.data_len;
			memcpy(&payload_buffer.data[0], &rx_cmd.data[0], payload_buffer.length);
			
			err = esb_write_payload(&payload_buffer);
			tx_event.data_hdr	= WRITE_TX_PAYLOAD;
			tx_event.data_len	= 0;
			tx_event.data[0]	= (uint8_t) (256-err);
				
		
			break;
			
		case READ_RX_PAYLOAD:
			
			err = esb_read_rx_payload(&payload_buffer);
			tx_event.data_hdr	= READ_RX_PAYLOAD;
			
			if (err==0)
			{				
				tx_event.data_len	= payload_buffer.length;
				memcpy(&tx_event.data[0], &payload_buffer.data[0], payload_buffer.length);
			} 
			else
			{
				tx_event.data_len	= 0;
				tx_event.data[0]	= (uint8_t) (256-err);			
			}
			
			break;
		
		case START_RX:
		
			err = esb_start_rx();
			
			tx_event.data_hdr	= START_RX;
			tx_event.data_len	= 0;
			tx_event.data[0]	= (uint8_t) (256-err);
		
			break;
		
		
		case STOP_RX:
		
			err = esb_stop_rx();
			
			tx_event.data_hdr	= START_RX;
			tx_event.data_len	= 0;
			tx_event.data[0]	= (uint8_t) (256-err);
			
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
