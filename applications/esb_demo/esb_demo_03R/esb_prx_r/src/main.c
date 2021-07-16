/*
 * Copyright (c) 2018-2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/ipm.h>
#include <sys/printk.h>
#include <device.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <init.h>
#include <drivers/gpio.h>

#include <ipc/rpmsg_service.h>

#include "radio_ipc.h"
#include "radio_config.h"

#define APP_TASK_STACK_SIZE (1024)

#define TX_PERIOD		K_MSEC(200)

#define LED_ON    0
#define LED_OFF   1


#define PERIPH_NUM   2//1..2

#define	PAYLOAD_SIZE	64  //bytes

uint8_t data_packet[] = {   1,  0,  3,  4,  5,  6,  7,  8,
                            9, 10, 11, 12, 13, 14, 15, 16,
                           17, 18, 19, 20, 21, 22, 23, 24,
                           25, 26, 27, 28, 29, 30, 31, 32,
                           33, 34, 35, 36, 37, 38, 39, 40,
                           41, 42, 43, 44, 45, 46, 47, 48,
                           49, 50, 51, 52, 53, 54, 55, 56,
                           57, 58, 59, 60, 61, 62, 63, 64 };



K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;


static const uint8_t  led_pins[] = {DT_GPIO_PIN(DT_ALIAS(led0), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led1), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led2), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led3), gpios)};


static const struct device *led_port;

static ipc_cmd_msg_t tx_cmd;

static ipc_evt_msg_t rx_evt;

static K_SEM_DEFINE(data_rx_sem, 0, 1);

static app_state_t application_state = APP_IDLE;

static int ep_id;

struct rpmsg_endpoint my_ept;

struct rpmsg_endpoint *ep = &my_ept;

static uint8_t data_buf[65];



static int gpios_init(void)
{
	int err;
	
	led_port = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	if (!led_port) {
		printk("Could not bind to LED port %s",
			DT_LABEL(DT_NODELABEL(gpio0)));
		return -EIO;
	}

	for (size_t i = 0; i < ARRAY_SIZE(led_pins); i++) {
                err = gpio_pin_configure(led_port, led_pins[i],
                                GPIO_OUTPUT);
                if (err) {
                                printk("Unable to configure LED%u, err %d", i, err);
                                led_port = NULL;
                                return err;
                        }
                        
                gpio_pin_set(led_port, led_pins[i], 1); 
	}
	
	return 0;
	
}


static void leds_update(uint8_t value)
{
	bool led0_status = !(value % 8 > 0 && value % 8 <= 4);
	bool led1_status = !(value % 8 > 1 && value % 8 <= 5);
	bool led2_status = !(value % 8 > 2 && value % 8 <= 6);
	bool led3_status = !(value % 8 > 3);

	gpio_port_pins_t mask =
		1 << led_pins[0] |
		1 << led_pins[1] |
		1 << led_pins[2] |
		1 << led_pins[3];

	gpio_port_value_t val =
		led0_status << led_pins[0] |
		led1_status << led_pins[1] |
		led2_status << led_pins[2] |
		led3_status << led_pins[3];

	if (led_port != NULL) {
		(void)gpio_port_set_masked_raw(led_port, mask, val);
	}
}



int rpmsg_endpoint_cb(struct rpmsg_endpoint *ept, void *data,
		size_t len, uint32_t src, void *priv)
{
	memcpy (&rx_evt, data, sizeof(rx_evt));
	
	k_sem_give(&data_rx_sem);

	return RPMSG_SUCCESS;
}



static int send_message(void)
{
	if (application_state == APP_IDLE)
	{
		tx_cmd.data_hdr		= RADIO_INITIALIZE_CMD;
		tx_cmd.data_len		= 2;
		tx_cmd.data[0]		= CONFIG_PERIPH;
		tx_cmd.data[1]		= PERIPH_NUM;
	}
	else if (application_state == APP_CFG)	
	{
		tx_cmd.data_hdr		= RADIO_PUT_PACKET_CMD;
		tx_cmd.data_len		= sizeof(data_buf);
		memcpy(tx_cmd.data, data_buf, sizeof(data_buf));
	}
	else if (application_state == APP_CFG2)	
	{
		tx_cmd.data_hdr		= RADIO_START_RX_SCAN_CMD;
		tx_cmd.data_len		= 1;
		tx_cmd.data[0]		= 0;
	}	
	else if (application_state == APP_OPT)
	{
		// Set LEDs identical to the ones on the PTX.
        leds_update(data_buf[2]);

        data_buf[2]++;
		
		tx_cmd.data_hdr		= RADIO_PUT_PACKET_CMD;
		tx_cmd.data_len		= sizeof(data_buf);
		memcpy(tx_cmd.data , data_buf, sizeof(data_buf));
		
	}
	
	return rpmsg_service_send(ep_id, &tx_cmd, sizeof(tx_cmd));
}




static void receive_message(void)
{
	int err ;
	
	k_sem_take(&data_rx_sem, K_FOREVER);
	
	switch (rx_evt.data_hdr)
	{
		case RADIO_INITIALIZE_EVT:
		
			if ( (err= rx_evt.data[0]) !=0)	//error occurs
			{
				printk("CPUAPP: radio initialize error %d\n", err);
			}
			else
			{
				application_state = APP_CFG;  //radio configured
				send_message();
				printk("CPUAPP: radio initialized\n");
			}
	
			break;
			
			
		case RADIO_PUT_PACKET_EVT:
		
		if(application_state == APP_CFG)
		{
			if ( (err= rx_evt.data[0]) !=0)	//error occurs
			{
				printk("CPUAPP: radio put packet error %d\n", err);
			}
			else
			{
				application_state = APP_CFG2;  //radio configured
				send_message();
				printk("CPUAPP: radio put packet success\n");
			}
		}
		
		break;	
			
			
		case RADIO_START_RX_SCAN_EVT:
		
			if ( (err= rx_evt.data[0]) !=0)	//error occurs
			{
				printk("CPUAPP: radio start poll timer error %d\n", err);
			}
			else
			{
				printk("CPUAPP: radio poll started\n");
				
			}
						
			break;
		
		
		case RADIO_PERIPH_DATA_SND_EVT:
			if ( (err= rx_evt.data[0]) !=0)	//error occurs
			{
				printk("CPUAPP: radio peripheral data sent error %d\n", err);
			}
			else
			{
				application_state = APP_OPT; // ready for data sent
				send_message();
				printk("CPUAPP: Ready for data sent\n");
			}
			
			break;
		
		default:
		
			break;

				
		
	}
	
	
}




void app_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	

	printk("\r\nRPMsg Service ptx-app started\r\n");

	/* Since we are using name service, we need to wait for a response
	 * from NS setup and than we need to process it
	 */
	while (!rpmsg_service_endpoint_is_bound(ep_id)) {
		k_sleep(K_MSEC(1));
	}
		
	send_message();
	
	while (1) {

		receive_message();			
	}
		
		
}
	

void data_init(void)
{
	data_buf[0] = PERIPH_NUM;
        memcpy( &data_buf[1],  data_packet, sizeof(data_packet) );
}	
	

void main(void)
{
	gpios_init();

	data_init();

	printk("Starting application thread!\n");
	k_thread_create(&thread_data, thread_stack, APP_TASK_STACK_SIZE,
			(k_thread_entry_t)app_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	memset(&tx_cmd, 0, sizeof(tx_cmd));
	memset(&rx_evt, 0, sizeof(rx_evt));
}

/* Make sure we register endpoint before RPMsg Service is initialized. */
int register_endpoint(const struct device *arg)
{
	int status;

	status = rpmsg_service_register_endpoint("demo", rpmsg_endpoint_cb);

	if (status < 0) {
		printk("rpmsg_create_ept failed %d\n", status);
		return status;
	}

	ep_id = status;

	return 0;
}

SYS_INIT(register_endpoint, POST_KERNEL, CONFIG_RPMSG_SERVICE_EP_REG_PRIORITY);
