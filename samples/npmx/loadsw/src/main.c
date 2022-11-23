/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*************************************************************/
/* API is not stable and maturity level of SW is evaluation. */
/*************************************************************/

#include <errno.h>
#include <device.h>
#include <zephyr.h>
#include <zephyr/logging/log.h>
#include <npmx_driver.h>
#include <npmx_gpio.h>
#include <npmx_core.h>
#include <npmx_ldsw.h>

#define LOG_MODULE_NAME pmic_loadsw
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* Possible LDSW testcases. */
#define TESTCASE_SW_ENABLE 1
#define TESTCASE_GPIO_ENABLE 2

/* Select one testcase. */
#define SELECTED_TESTCASE TESTCASE_SW_ENABLE

/**
 * @brief Function for testing enabling and disabling LDSWs with software.
 * In an infinite loop, switches are enabled and disabled alternately.
 * 
 * @param[in] p_pm The pointer to the instance of nPM device.
 */
static void test_sw_enable(npmx_instance_t *p_pm)
{
	LOG_INF("Test enable LDSW with software");

	while (1) {
		/* Enable LDSW1 and disable LDSW2. */
		if (npmx_ldsw_status_set(p_pm, NPMX_LDSW_INSTANCE_1, true) == false) {
			LOG_ERR("Unable to enable LDSW1");
		}

		if (npmx_ldsw_status_set(p_pm, NPMX_LDSW_INSTANCE_2, false) == false) {
			LOG_ERR("Unable to disable LDSW2");
		}

		k_msleep(5000);

		/* Disable LDSW1 and enable LDSW2. */
		if (npmx_ldsw_status_set(p_pm, NPMX_LDSW_INSTANCE_1, false) == false) {
			LOG_ERR("Unable to disable LDSW1");
		}

		if (npmx_ldsw_status_set(p_pm, NPMX_LDSW_INSTANCE_2, true) == false) {
			LOG_ERR("Unable to enable LDSW2");
		}

		k_msleep(5000);
	}
}

/**
 * @brief Function for testing enabling and disabling LDSWs using external pins.
 * When GPIO1 is in HIGH state, LDSW1 is enabled, otherwise disabled.
 * When GPIO2 is in HIGH state, LDSW2 is enabled, otherwise disabled.
 * 
 * @param[in] p_pm The pointer to the instance of nPM device.
 */
static void test_gpio_enable(npmx_instance_t *p_pm)
{
	LOG_INF("Test enable LDSW via connected pin");

	/* Switch GPIO1 and GPIO2 to input mode. */
	if (npmx_gpio_mode_set(p_pm, NPMX_GPIO_INSTANCE_1, NPMX_GPIO_MODE_INPUT) == false) {
		LOG_ERR("Unable to switch GPIO1 to input mode");
	}

	if (npmx_gpio_mode_set(p_pm, NPMX_GPIO_INSTANCE_2, NPMX_GPIO_MODE_INPUT) == false) {
		LOG_ERR("Unable to switch GPIO2 to input mode");
	}

	/* Connect GPIO1 to enable LDSW1. */
	if (npmx_ldsw_enable_gpio_select(p_pm, NPMX_LDSW_INSTANCE_1, NPMX_LDSW_GPIO_1) == false) {
		LOG_ERR("Unable to connect GPIO1 to LDSW1");
	}

	/* Connect GPIO2 to enable LDSW2. */
	if (npmx_ldsw_enable_gpio_select(p_pm, NPMX_LDSW_INSTANCE_2, NPMX_LDSW_GPIO_2) == false) {
		LOG_ERR("Unable to connect GPIO2 to LDSW2");
	}
}

void main(void)
{
	const struct device *npmx_dev = DEVICE_DT_GET(DT_NODELABEL(npmx));

	if (!device_is_ready(npmx_dev)) {
		LOG_INF("NPMX device is not ready");
		return;
	} else {
		LOG_INF("NPMX device ok");
	}

	/* Get the pointer to npmx device. */
	npmx_instance_t *npmx_instance = &((struct npmx_data *)npmx_dev->data)->npmx_instance;

	/* Enable active discharge, to see changes on output pins. */
	npmx_ldsw_active_discharge_enable(npmx_instance, NPMX_LDSW_INSTANCE_1, true);
	npmx_ldsw_active_discharge_enable(npmx_instance, NPMX_LDSW_INSTANCE_2, true);

#if SELECTED_TESTCASE == TESTCASE_SW_ENABLE
	test_sw_enable(npmx_instance);
#elif SELECTED_TESTCASE == TESTCASE_GPIO_ENABLE
	test_gpio_enable(npmx_instance);
#endif

	while (1) {
		k_sleep(K_FOREVER);
	}
}
