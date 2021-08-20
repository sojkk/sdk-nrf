/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
 
 /* File: motion_circle_notimer.c */

#include <zephyr.h>
#include <drivers/gpio.h>

#include "circle_test.h"
#include "event_manager.h"
#include "button_event.h"
#include "motion_event.h"
#include "hid_event.h"

#define MODULE motion
#include "module_state_event.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_DESKTOP_MOTION_LOG_LEVEL);


#define LED_ON	0
#define LED_OFF	1

#define MOTION_SEND			29
#define STATE_PEND_CONN		30
#define DRAW_HANDLE			31


enum state {
	STATE_IDLE,
	STATE_CONNECTED,
	STATE_PENDING
};


enum dir {
	DIR_START,
	DIR_STOP,

	DIR_COUNT
};





static const struct device *led_port;
	
static int8_t circle_data[2];

static enum state state;


static bool is_motion_active = false;


static void motion_event_send(int16_t dx, int16_t dy)
{
	struct motion_event *event = new_motion_event();

	event->dx = dx;
	event->dy = dy;

	EVENT_SUBMIT(event);
}

static enum dir key_to_dir(uint16_t key_id)
{
	enum dir dir = DIR_COUNT;
	
		
	switch (key_id) {
	case CONFIG_DESKTOP_MOTION_START_ID:
		dir = DIR_START;
		break;

	case CONFIG_DESKTOP_MOTION_STOP_ID:
		dir = DIR_STOP;
		break;	

	default:
		break;
	}

	return dir;
}


static void send_motion(void)
{
	gpio_pin_set(led_port, MOTION_SEND, 1);

	motion_event_send((int16_t)circle_data[0], (int16_t)circle_data[1]);
	
	gpio_pin_set(led_port, MOTION_SEND, 0);
}


static int leds_init(void)
{
	led_port = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
		
	if (!led_port) {
		LOG_ERR("Could not bind to LED port %s",
		DT_LABEL(DT_NODELABEL(gpio0))	);
		return -EIO;
	}
	
			
	int err = gpio_pin_configure(led_port, DT_GPIO_PIN(DT_ALIAS(led2), gpios) , GPIO_OUTPUT);
	
	if (err) {
	LOG_ERR("Unable to configure Start/Stop LED, err %d", err);
	led_port = NULL;
	return err;
	}
	
	if(led_port != NULL) {
		(void)gpio_pin_set(led_port, DT_GPIO_PIN(DT_ALIAS(led2), gpios)  , LED_OFF); 
	}
 /**  For Testing purpose only 
  */		
	err = gpio_pin_configure(led_port, MOTION_SEND, GPIO_OUTPUT);
	if (err) {
	LOG_ERR("Unable to config pin31, err %d", err);
	led_port = NULL;
	return err;
	}
	
	(void)gpio_pin_configure(led_port, STATE_PEND_CONN , GPIO_OUTPUT);
	(void)gpio_pin_configure(led_port, DRAW_HANDLE, GPIO_OUTPUT);
 
	return 0;
 
 
}	

void draw_circle_handler(void)
{
	
	(void)gpio_pin_set(led_port, DRAW_HANDLE, 1);


	circle_test_get(circle_data);


   	if (state == STATE_CONNECTED) {	
		
		send_motion();
		LOG_INF("Send motion");
		state = STATE_PENDING;
		gpio_pin_set(led_port, STATE_PEND_CONN , 0);
	}
	(void)gpio_pin_set(led_port,DRAW_HANDLE, 0);

}


static bool handle_button_event(const struct button_event *event)
{
	enum dir dir = key_to_dir(event->key_id);

	
	if (dir == DIR_COUNT) {
				
		return false;
	}

	
	if(dir == DIR_START)
	{
		

		is_motion_active = true;
		draw_circle_handler();
		/* Lit LED3 */
		if(led_port != NULL) {
			(void)gpio_pin_set(led_port, DT_GPIO_PIN(DT_ALIAS(led2), gpios) , LED_ON); 
		}
	    LOG_INF("Button event: Start button pressed");
	}
			
	else if(dir == DIR_STOP)
	{

		is_motion_active = false;
		if(state == STATE_PENDING)  
			state = STATE_CONNECTED;
		
		/* Off LED3 */
			if(led_port != NULL) {
			(void)gpio_pin_set(led_port, DT_GPIO_PIN(DT_ALIAS(led2), gpios) , LED_OFF); 
		}
		LOG_INF("Button event: Stop button pressed");
	}
			
	return true;
}

static bool handle_module_state_event(const struct module_state_event *event)
{
	(void)leds_init();
	
	/* Replicate the state of buttons module */
	if (event->module_id == MODULE_ID(buttons)) {
		module_set_state(event->state);
	}

	return false;
}


static bool handle_hid_report_sent_event(const struct hid_report_sent_event *event)
{
	if (event->report_id == REPORT_ID_MOUSE) {
		
		
		if (state == STATE_PENDING) {
			if (is_motion_active) {
			
				//LOG_INF("HID_REPORT_SEND EVENT: STATE CONNECTED");
				state = STATE_CONNECTED;
				gpio_pin_set(led_port, STATE_PEND_CONN, 1);
				draw_circle_handler();
			
			}
		
		}
	}

	return false;
}

static bool handle_hid_report_subscription_event(const struct hid_report_subscription_event *event)
{
	if (event->report_id == REPORT_ID_MOUSE) {
		static uint8_t peer_count;

		if (event->enabled) {
			__ASSERT_NO_MSG(peer_count < UCHAR_MAX);
			peer_count++;
		} else {
			__ASSERT_NO_MSG(peer_count > 0);
			peer_count--;
		}

		bool is_connected = (peer_count != 0);

		if ((state == STATE_IDLE) && is_connected) {
			if (is_motion_active) {
				send_motion();
				state = STATE_PENDING;
				gpio_pin_set(led_port, STATE_PEND_CONN, 0);
			} else {
				//LOG_INF("SUBSCRIPTION EVENT: STATE CONNECTED");
				state = STATE_CONNECTED;
				gpio_pin_set(led_port, STATE_PEND_CONN, 1);
			}
			return false;
		}
	
		if ((state != STATE_IDLE) && !is_connected) {
			state = STATE_IDLE;
			return false;
		}
	}

	return false;
}

static bool event_handler(const struct event_header *eh)
{
	
	
	if (is_hid_report_sent_event(eh)) {
		return handle_hid_report_sent_event(
				cast_hid_report_sent_event(eh));
	}

	if (is_button_event(eh)) {
		return handle_button_event(cast_button_event(eh));
	}

	if (is_module_state_event(eh)) {
		return handle_module_state_event(cast_module_state_event(eh));
	}

	if (is_hid_report_subscription_event(eh)) {
		return handle_hid_report_subscription_event(
				cast_hid_report_subscription_event(eh));
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}
EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE_EARLY(MODULE, button_event);
EVENT_SUBSCRIBE(MODULE, module_state_event);
EVENT_SUBSCRIBE(MODULE, hid_report_sent_event);
EVENT_SUBSCRIBE(MODULE, hid_report_subscription_event);
