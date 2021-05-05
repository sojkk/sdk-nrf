/*
 * Copyright (c) 2018, NXP
 * Copyright (c) 2018-2019, Linaro Limited
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

#include <ipc/rpmsg_service.h>

#define APP_TASK_STACK_SIZE (1024)
K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;

uint8_t const test_data[]= {0xDE, 0xAD, 0xBE, 0xEF};


typedef struct
{
    uint8_t interrupt_event;
    uint8_t command;
    uint8_t data[32];
    
}msg_t;


static msg_t message;

static msg_t received_message;

static volatile uint8_t received_data;

static K_SEM_DEFINE(data_rx_sem, 0, 1);

int endpoint_cb(struct rpmsg_endpoint *ept, void *data,
		size_t len, uint32_t src, void *priv)
{
	memcpy (&received_message, data, sizeof(received_message));


    received_data = received_message.command;

	k_sem_give(&data_rx_sem);

	return RPMSG_SUCCESS;
}

static int ep_id;
struct rpmsg_endpoint my_ept;
struct rpmsg_endpoint *ep = &my_ept;

static unsigned int receive_message(void)
{
	k_sem_take(&data_rx_sem, K_FOREVER);
	return received_data;
}

//static int send_message(unsigned int message)

static int send_message(void)
{
	return rpmsg_service_send(ep_id, &message, sizeof(message));
}

void app_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	int status = 0;
	//unsigned int message = 0U;


        message.command = 0;

	printk("\r\nRPMsg Service [master] demo started\r\n");

	/* Since we are using name service, we need to wait for a response
	 * from NS setup and than we need to process it
	 */
	while (!rpmsg_service_endpoint_is_bound(ep_id)) {
		k_sleep(K_MSEC(1));
	}


	while (message.command < 100) {
		status = send_message();
		if (status < 0) {
			printk("send_message(%d) failed with status %d\n",
			       message.command, status);		   		
			break;
		}

		message.command = receive_message();
		printk("Master core received a message: command = %d\n", received_message.command);
		printk("Master core received a message: interrupt_event = %d\n", received_message.interrupt_event);
		printk("Master core received a message: data = %x %x %x %x\n", received_message.data[0], received_message.data[1], received_message.data[2], received_message.data[3] );

		message.command++;
	}


	printk("RPMsg Service demo ended.\n");
}

void main(void)
{
	printk("Starting application thread!\n");
	k_thread_create(&thread_data, thread_stack, APP_TASK_STACK_SIZE,
			(k_thread_entry_t)app_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

        memset(&message, 0, sizeof(message));
		
		message.interrupt_event = 0x01;
		memcpy(&message.data[0], test_data, sizeof(test_data));

#if defined(CONFIG_SOC_MPS2_AN521) || \
	defined(CONFIG_SOC_V2M_MUSCA_A) || \
	defined(CONFIG_SOC_V2M_MUSCA_B1)
	wakeup_cpu1();
	k_msleep(500);
#endif /* #if defined(CONFIG_SOC_MPS2_AN521) */
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
