/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"
#include <drivers/pwm.h>

#define PWM_LED0_NODE	DT_ALIAS(pwm_led0)
#define PWM_LED1_NODE	DT_ALIAS(pwm_led1)

/*
 * Devicetree helper macro which gets the 'flags' cell from the node's
 * pwms property, or returns 0 if the property has no 'flags' cell.
 */

#if DT_PHA_HAS_CELL(PWM_LED0_NODE, pwms, flags)
#define PWM_FLAGS0 DT_PWMS_FLAGS(PWM_LED0_NODE)
#else
#define PWM_FLAGS0 0
#endif

#if DT_PHA_HAS_CELL(PWM_LED1_NODE, pwms, flags)
#define PWM_FLAGS1 DT_PWMS_FLAGS(PWM_LED1_NODE)
#else
#define PWM_FLAGS1 0
#endif

#if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
#define PWM_CTLR0	DT_PWMS_CTLR(PWM_LED0_NODE)
#define PWM_CHANNEL0	DT_PWMS_CHANNEL(PWM_LED0_NODE)
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR0	DT_INVALID_NODE
#define PWM_CHANNEL0	0
#endif

#if DT_NODE_HAS_STATUS(PWM_LED1_NODE, okay)
#define PWM_CTLR1	DT_PWMS_CTLR(PWM_LED1_NODE)
#define PWM_CHANNEL1	DT_PWMS_CHANNEL(PWM_LED1_NODE)
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR1	DT_INVALID_NODE
#define PWM_CHANNEL1	0
#endif

static const struct device *pwm0, *pwm1;
#define PWM_PERIOD 1024

void lc_pwm_led_init(void)
{
	pwm0 = DEVICE_DT_GET(PWM_CTLR0);
	if (!device_is_ready(pwm0)) {
		printk("Error: PWM device %s is not ready\n", pwm0->name);
		return;
	}
	
	pwm1 = DEVICE_DT_GET(PWM_CTLR1);
	if (!device_is_ready(pwm1)) {
		printk("Error: PWM device %s is not ready\n", pwm1->name);
		return;
	}
	
}

void lc_pwm_led_set(uint8_t dev_num, uint16_t desired_lvl)
{
	uint32_t scaled_lvl =
		(PWM_PERIOD * desired_lvl) /
		BT_MESH_LIGHTNESS_MAX;

   switch (dev_num)
   {
	   
		case 0:   

			pwm_pin_set_usec(pwm0,
				PWM_CHANNEL0,
				PWM_PERIOD,
				scaled_lvl,
				PWM_FLAGS0);
			
		break;
		
		case 1:   

			pwm_pin_set_usec(pwm1,
				PWM_CHANNEL1,
				PWM_PERIOD,
				scaled_lvl,
				PWM_FLAGS1);
			
		break;

   }
}
