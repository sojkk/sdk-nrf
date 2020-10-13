/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/* File: voice_dummy.c */

#include <zephyr.h>
#include <drivers/gpio.h>


#include "event_manager.h"
#include "button_event.h"
#include "voice_event.h"
#include "hid_event.h"

#define MODULE voice
#include "module_state_event.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_DESKTOP_MOTION_LOG_LEVEL);


#define LED_ON	0
#define LED_OFF	1

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


static u8_t vdata_ptr=0;

static struct device *led_port;

//40 bytes compressed voice data	
static u8_t voice_data[] =
{
184,125,234,110,91,246,57,31,72,137,\
60,224,93,236,44,217,98,245,78,13,\
37,245,223,137,70,149,8,232,133,250,\
78,213,161,52,229,200,149,212,34,157,\
};


static enum state state;




static void voice_event_send(void)
{
	struct voice_event *event = new_voice_event();

        u8_t data_buffer[REPORT_SIZE_VOICE];

        if (vdata_ptr <=  (sizeof(voice_data) - REPORT_SIZE_VOICE) )
        {
          memcpy (data_buffer, &voice_data[vdata_ptr], REPORT_SIZE_VOICE);
        }
        else 
        {
          memcpy (data_buffer, &voice_data[vdata_ptr], (sizeof(voice_data)- vdata_ptr ));
	
          memcpy (&data_buffer[sizeof(voice_data)-vdata_ptr ], voice_data, (REPORT_SIZE_VOICE-sizeof(voice_data)+vdata_ptr ));
        }

        memcpy(event->data , data_buffer , REPORT_SIZE_VOICE);  //pointer to the dummy voice data

        vdata_ptr =  (vdata_ptr + REPORT_SIZE_VOICE) % (sizeof(voice_data));

	EVENT_SUBMIT(event);
}

static enum dir key_to_dir(u16_t key_id)
{
	enum dir dir = DIR_COUNT;
	
		
	switch (key_id) {
	case CONFIG_DESKTOP_VOICE_START_ID:
		dir = DIR_START;
		break;

	case CONFIG_DESKTOP_VOICE_STOP_ID:
		dir = DIR_STOP;
		break;	

	default:
		break;
	}

	return dir;
}

static int leds_init(void)
{
	led_port = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
		
	if (!led_port) {
		LOG_ERR("Could not bind to LED port %s",
			DT_LABEL(DT_NODELABEL(gpio0)) );
		return -EIO;
	}
	
	int err = gpio_pin_configure(led_port, DT_GPIO_PIN(DT_ALIAS(led3), gpios), GPIO_OUTPUT);
	
	if (err) {
	LOG_ERR("Unable to configure Start/Stop LED, err %d", err);
	led_port = NULL;
	return err;
	}
	
	if(led_port != NULL) {
		(void)gpio_pin_set(led_port, DT_GPIO_PIN(DT_ALIAS(led3), gpios), LED_OFF); 
	}

	return 0;
 
 
}	

void voice_timer_handler(struct k_timer *dummy)
{
	
	//circle_test_get(circle_data);

	//LOG_INF("s_idx = %i",s_idx);

   	if (state == STATE_CONNECTED) {	
		voice_event_send();
		state = STATE_PENDING;
		//gpio_pin_set(led_port, 30, 0);
	}
	//void)gpio_pin_set(led_port,31, 0);
}

K_TIMER_DEFINE(voice_timer, voice_timer_handler, NULL);


static bool handle_button_event(const struct button_event *event)
{
	enum dir dir = key_to_dir(event->key_id);

	
	if (dir == DIR_COUNT) {
				
		return false;
	}

	
	if(dir == DIR_START)
	{
		
		/* Start spp timer , repeated per 8ms*/
		k_timer_start(&voice_timer,K_MSEC(8), K_MSEC(8));
		/* Lit LED3 */
		if(led_port != NULL) {
			(void)gpio_pin_set(led_port, DT_GPIO_PIN(DT_ALIAS(led3), gpios), LED_ON); 
		}
	    LOG_INF("Button event: Start button pressed");
	}
			
	else if(dir == DIR_STOP)
	{
		/* Stop app timer */
		k_timer_stop(&voice_timer);
		/* Off LED3 */
			if(led_port != NULL) {
			(void)gpio_pin_set(led_port, DT_GPIO_PIN(DT_ALIAS(led3), gpios), LED_OFF); 
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
	if (event->report_id == REPORT_ID_VOICE) {
		if (state == STATE_PENDING) {
			state = STATE_CONNECTED;
		}
	}

	return false;
}

static bool handle_hid_report_subscription_event(const struct hid_report_subscription_event *event)
{
	if (event->report_id == REPORT_ID_VOICE) {
		static u8_t peer_count;

		if (event->enabled) {
			__ASSERT_NO_MSG(peer_count < UCHAR_MAX);
			peer_count++;
		} else {
			__ASSERT_NO_MSG(peer_count > 0);
			peer_count--;
		}

		bool is_connected = (peer_count != 0);

		if ((state == STATE_IDLE) && is_connected) {
			state = STATE_CONNECTED;
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
