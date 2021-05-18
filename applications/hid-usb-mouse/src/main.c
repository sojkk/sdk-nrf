/*
 * Copyright (c) 2018 qianfan Zhao
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>

#include <usb/usb_device.h>
#include <usb/class/usb_hid.h>

#include <drivers/ipm.h>
#include <device.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <init.h>

#include <ipc/rpmsg_service.h>

#include "esb_ipc.h"

#define STACK_SIZE (1024)

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(main);

#define SW0_NODE DT_ALIAS(sw0)

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
#define PORT0		DT_GPIO_LABEL(SW0_NODE, gpios)
#define PIN0		DT_GPIO_PIN(SW0_NODE, gpios)
#define PIN0_FLAGS	DT_GPIO_FLAGS(SW0_NODE, gpios)
#else
#error "Unsupported board: sw0 devicetree alias is not defined"
#define PORT0		""
#define PIN0		0
#define PIN0_FLAGS	0
#endif

#define SW1_NODE DT_ALIAS(sw1)

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
#define PORT1		DT_GPIO_LABEL(SW1_NODE, gpios)
#define PIN1		DT_GPIO_PIN(SW1_NODE, gpios)
#define PIN1_FLAGS	DT_GPIO_FLAGS(SW1_NODE, gpios)
#endif

#define SW2_NODE DT_ALIAS(sw2)

#if DT_NODE_HAS_STATUS(SW2_NODE, okay)
#define PORT2		DT_GPIO_LABEL(SW2_NODE, gpios)
#define PIN2		DT_GPIO_PIN(SW2_NODE, gpios)
#define PIN2_FLAGS	DT_GPIO_FLAGS(SW2_NODE, gpios)
#endif

#define SW3_NODE DT_ALIAS(sw3)

#if DT_NODE_HAS_STATUS(SW3_NODE, okay)
#define PORT3		DT_GPIO_LABEL(SW3_NODE, gpios)
#define PIN3		DT_GPIO_PIN(SW3_NODE, gpios)
#define PIN3_FLAGS	DT_GPIO_FLAGS(SW3_NODE, gpios)
#endif

#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED_PORT	DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED		DT_GPIO_PIN(LED0_NODE, gpios)
#define LED_FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED_PORT	""
#define LED		0
#define LED_FLAGS	0
#endif

static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(2);

static uint8_t def_val[4];
static volatile uint8_t status[4];
static K_SEM_DEFINE(sem, 0, 1);	/* starts off "not available" */
static K_SEM_DEFINE(usb_init_ok, 0, 1);
static struct gpio_callback callback[4];
static enum usb_dc_status_code usb_status;

#define MOUSE_BTN_REPORT_POS	0
#define MOUSE_X_REPORT_POS	1
#define MOUSE_Y_REPORT_POS	2

#define MOUSE_BTN_LEFT		BIT(0)
#define MOUSE_BTN_RIGHT		BIT(1)
#define MOUSE_BTN_MIDDLE	BIT(2)

#define TX_PERIOD		K_MSEC(200)


static const uint8_t  led_pins[] = {DT_GPIO_PIN(DT_ALIAS(led0), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led1), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led2), gpios),
                                    DT_GPIO_PIN(DT_ALIAS(led3), gpios)};

static const struct device *led_dev;

const struct device *hid_dev;

static int ep_id;

static struct k_timer tx_timer_id;

static uint8_t message = 0;

static ipc_msg_t tx_cmd;

static ipc_msg_t rx_evt;

static K_SEM_DEFINE(data_rx_sem, 0, 1);

static app_state_t application_state = APP_IDLE;


static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	usb_status = status;
}

static void left_button(const struct device *gpio, struct gpio_callback *cb,
			uint32_t pins)
{
	int ret;
	uint8_t state = status[MOUSE_BTN_REPORT_POS];

	if (IS_ENABLED(CONFIG_USB_DEVICE_REMOTE_WAKEUP)) {
		if (usb_status == USB_DC_SUSPEND) {
			usb_wakeup_request();
			return;
		}
	}

	ret = gpio_pin_get(gpio, PIN0);
	if (ret < 0) {
		LOG_ERR("Failed to get the state of pin %u, error: %d",
			PIN0, ret);
		return;
	}

	if (def_val[0] != (uint8_t)ret) {
		state |= MOUSE_BTN_LEFT;
	} else {
		state &= ~MOUSE_BTN_LEFT;
	}

	if (status[MOUSE_BTN_REPORT_POS] != state) {
		status[MOUSE_BTN_REPORT_POS] = state;
		k_sem_give(&sem);
	}
}

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
static void right_button(const struct device *gpio, struct gpio_callback *cb,
			 uint32_t pins)
{
	int ret;
	uint8_t state = status[MOUSE_BTN_REPORT_POS];

	if (IS_ENABLED(CONFIG_USB_DEVICE_REMOTE_WAKEUP)) {
		if (usb_status == USB_DC_SUSPEND) {
			usb_wakeup_request();
			return;
		}
	}

	ret = gpio_pin_get(gpio, PIN1);
	if (ret < 0) {
		LOG_ERR("Failed to get the state of pin %u, error: %d",
			PIN1, ret);
		return;
	}

	if (def_val[1] != (uint8_t)ret) {
		state |= MOUSE_BTN_RIGHT;
	} else {
		state &= ~MOUSE_BTN_RIGHT;
	}

	if (status[MOUSE_BTN_REPORT_POS] != state) {
		status[MOUSE_BTN_REPORT_POS] = state;
		k_sem_give(&sem);
	}
}
#endif

#if DT_NODE_HAS_STATUS(SW2_NODE, okay)
static void x_move(const struct device *gpio, struct gpio_callback *cb,
		   uint32_t pins)
{
	int ret;
	uint8_t state = status[MOUSE_X_REPORT_POS];

	ret = gpio_pin_get(gpio, PIN2);
	if (ret < 0) {
		LOG_ERR("Failed to get the state of pin %u, error: %d",
			PIN2, ret);
		return;
	}

	if (def_val[2] != (uint8_t)ret) {
		state += 10U;
	}

	if (status[MOUSE_X_REPORT_POS] != state) {
		status[MOUSE_X_REPORT_POS] = state;
		k_sem_give(&sem);
	}
}
#endif

#if DT_NODE_HAS_STATUS(SW3_NODE, okay)
static void y_move(const struct device *gpio, struct gpio_callback *cb,
		   uint32_t pins)
{
	int ret;
	uint8_t state = status[MOUSE_Y_REPORT_POS];

	ret = gpio_pin_get(gpio, PIN3);
	if (ret < 0) {
		LOG_ERR("Failed to get the state of pin %u, error: %d",
			PIN3, ret);
		return;
	}

	if (def_val[3] != (uint8_t)ret) {
		state += 10U;
	}

	if (status[MOUSE_Y_REPORT_POS] != state) {
		status[MOUSE_Y_REPORT_POS] = state;
		k_sem_give(&sem);
	}
}
#endif

int callbacks_configure(const struct device *gpio, uint32_t pin, int flags,
			gpio_callback_handler_t handler,
			struct gpio_callback *callback, uint8_t *val)
{
	int ret;

	if (!gpio) {
		LOG_ERR("Could not find PORT");
		return -ENXIO;
	}

	ret = gpio_pin_configure(gpio, pin, GPIO_INPUT | flags);
	if (ret < 0) {
		LOG_ERR("Failed to configure pin %u, error: %d",
			pin, ret);
		return ret;
	}

	ret = gpio_pin_get(gpio, pin);
	if (ret < 0) {
		LOG_ERR("Failed to get the state of pin %u, error: %d",
			pin, ret);
		return ret;
	}

	*val = (uint8_t)ret;

	gpio_init_callback(callback, handler, BIT(pin));
	ret = gpio_add_callback(gpio, callback);
	if (ret < 0) {
		LOG_ERR("Failed to add the callback for pin %u, error: %d",
			pin, ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure(gpio, pin, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("Failed to configure interrupt for pin %u, error: %d",
			pin, ret);
		return ret;
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

	if (led_dev != NULL) {
		(void)gpio_port_set_masked_raw(led_dev, mask, val);
	}
}

int endpoint_cb(struct rpmsg_endpoint *ept, void *data,
		size_t len, uint32_t src, void *priv)
{
	memcpy (&rx_evt, data, sizeof(rx_evt));
	
	k_sem_give(&data_rx_sem);

	return RPMSG_SUCCESS;
}


static int send_message(uint8_t send_data)
{
	if (application_state == APP_IDLE)
	{
		tx_cmd.data_hdr		= ESB_INITIALIZE;
		tx_cmd.data_len		= 1;
		tx_cmd.data[0]		= CONFIG_PTX;
	}
	else if (application_state == APP_CFG)	
	{
		tx_cmd.data_hdr		= WRITE_TX_PAYLOAD;
		tx_cmd.data_len		= 2;
		tx_cmd.data[0]		= 0;
		tx_cmd.data[1]		= send_data;
	}
		
	
	return rpmsg_service_send(ep_id, &tx_cmd, sizeof(tx_cmd));
}


static void receive_message(void)
{
	int err ;
	
	k_sem_take(&data_rx_sem, K_FOREVER);
	
	switch (rx_evt.data_hdr)
	{
		case ESB_INITIALIZE:
		
			if ( (err= rx_evt.data[0]) !=0)	//error occurs
			{
				LOG_INF("esb initialize error %d\n", err);
				k_timer_stop(&tx_timer_id);
			}
			else
			{
				application_state = APP_CFG;  //radio configured
				LOG_INF("esb initialized");
			}
	
			break;
		
		case WRITE_TX_PAYLOAD:
		
			if ( (err= rx_evt.data[0]) !=0)	//error occurs
			{
				LOG_INF("esb write payload error %d", err);
			}
			
			break;

		case TX_SUCCESS:
		
			leds_update(message);
			message++;
			LOG_INF("esb tx success");	
			
			break;
			
		case TX_FAILED:

			LOG_INF("esb tx failed");
		
			break;
			
		
	}
}


void tx_timer_handler(struct k_timer *dummy)
{
	
	int status =0;

	status = send_message(message);
	if (status < 0) {
		LOG_ERR("send_message failed with status %d\n", status);		   		
	}
	
}


void  esb_send_thread(void)
{

	LOG_INF("Starting esb send thread!");
	
	/* Since we are using name service, we need to wait for a response
	 * from NS setup and than we need to process it
	 */
	
	while (!rpmsg_service_endpoint_is_bound(ep_id)) {
		k_sleep(K_MSEC(1));
	}
	
	k_timer_start(&tx_timer_id, TX_PERIOD, TX_PERIOD);
	LOG_INF("k_timer started!");
	
	while (1) {
		
			receive_message();
			
			k_sleep(K_MSEC(10));
			
		}
	
}	


void main(void)
{
	int ret;

	led_dev = device_get_binding(LED_PORT);
	if (led_dev == NULL) {
		LOG_ERR("Cannot get LED");
		return;
	}

	hid_dev = device_get_binding("HID_0");
	if (hid_dev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return;
	}

	ret = gpio_pin_configure(led_dev, LED, GPIO_OUTPUT | LED_FLAGS);
	if (ret < 0) {
		LOG_ERR("Failed to configure the LED pin, error: %d", ret);
		return;
	}

	if (callbacks_configure(device_get_binding(PORT0), PIN0, PIN0_FLAGS,
				&left_button, &callback[0], &def_val[0])) {
		LOG_ERR("Failed configuring left button callback.");
		return;
	}

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
	if (callbacks_configure(device_get_binding(PORT1), PIN1, PIN1_FLAGS,
				&right_button, &callback[1], &def_val[1])) {
		LOG_ERR("Failed configuring right button callback.");
		return;
	}
#endif

#if DT_NODE_HAS_STATUS(SW2_NODE, okay)
	if (callbacks_configure(device_get_binding(PORT2), PIN2, PIN2_FLAGS,
				&x_move, &callback[2], &def_val[2])) {
		LOG_ERR("Failed configuring X axis movement callback.");
		return;
	}
#endif

#if DT_NODE_HAS_STATUS(SW3_NODE, okay)
	if (callbacks_configure(device_get_binding(PORT3), PIN3, PIN3_FLAGS,
				&y_move, &callback[3], &def_val[3])) {
		LOG_ERR("Failed configuring Y axis movement callback.");
		return;
	}
#endif

	usb_hid_register_device(hid_dev,
				hid_report_desc, sizeof(hid_report_desc),
				NULL);

	usb_hid_init(hid_dev);

	ret = usb_enable(status_cb);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}
	
	LOG_INF("USB initialized");

	k_sem_give(&usb_init_ok);

	
	k_timer_init(&tx_timer_id, tx_timer_handler, NULL);
	
	
}


void usb_report_thread(void)
{
	int ret;
	uint8_t report[4] = { 0x00 };
		
	k_sem_take(&usb_init_ok, K_FOREVER);	
		
	LOG_INF("Starting usb report thread!");
	
	while (true) {
		k_sem_take(&sem, K_FOREVER);

		report[MOUSE_BTN_REPORT_POS] = status[MOUSE_BTN_REPORT_POS];
		report[MOUSE_X_REPORT_POS] = status[MOUSE_X_REPORT_POS];
		status[MOUSE_X_REPORT_POS] = 0U;
		report[MOUSE_Y_REPORT_POS] = status[MOUSE_Y_REPORT_POS];
		status[MOUSE_Y_REPORT_POS] = 0U;
		ret = hid_int_ep_write(hid_dev, report, sizeof(report), NULL);
		if (ret) {
			LOG_ERR("HID write error, %d", ret);
		}

		/* Toggle LED on sent report */
		ret = gpio_pin_toggle(led_dev, LED);
		if (ret < 0) {
			LOG_ERR("Failed to toggle the LED pin, error: %d", ret);
		}
		
		k_sleep(K_MSEC(100));
	}
	
}	

/* Make sure we register endpoint before RPMsg Service is initialized. */

int register_endpoint(const struct device *arg)
{
	int ep_status;

	ep_status = rpmsg_service_register_endpoint("demo", endpoint_cb);

	if (ep_status < 0) {
		LOG_ERR("rpmsg_create_ept failed %d", ep_status);
		return ep_status;
	}

	ep_id = ep_status;

	return 0;
}

SYS_INIT(register_endpoint, POST_KERNEL, CONFIG_RPMSG_SERVICE_EP_REG_PRIORITY);

K_THREAD_DEFINE(usb_report_thread_id, STACK_SIZE, usb_report_thread, NULL, NULL,
		NULL, 7, 0, 50);

K_THREAD_DEFINE(esb_send_thread_id, STACK_SIZE, esb_send_thread, NULL, NULL,
		NULL, 6, 0, 100);
