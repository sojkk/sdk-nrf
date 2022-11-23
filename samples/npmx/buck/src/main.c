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
#include <npmx_buck.h>

#define LOG_MODULE_NAME pmic_buck
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* Possible BUCK testcases. */
#define TESTCASE_SET_BUCK_VOLTAGE 1
#define TESTCASE_OUTPUT_VOLTAGE 2
#define TESTCASE_RETENTION_VOLTAGE 3
#define TESTCASE_ENABLE_VIA_PINS 4

/* Select one testcase. */
#define SELECTED_TESTCASE TESTCASE_SET_BUCK_VOLTAGE

/**
 * @brief Function for returning Buck instance name.
 * 
 * @param[in] buck_instance Specified buck instance.
 * @return The pointer to the buck instance name.
 */
static const char *buck_instance_to_str(npmx_buck_instance_t buck_instance)
{
	__ASSERT_NO_MSG(buck_instance < NPMX_BUCK_INSTANCE_COUNT);
	const static char *instances[NPMX_BUCK_INSTANCE_COUNT] = {
		[NPMX_BUCK_INSTANCE_1] = "BUCK1", [NPMX_BUCK_INSTANCE_2] = "BUCK2"
	};
	return instances[buck_instance];
}

/**
 * @brief Function for setting the buck output voltage for the specified buck instance.
 * 
 * @param[in] p_pm The pointer to the instance of nPM device.
 * @param[in] buck_instance The buck instance to set the voltage to.
 * @param[in] voltage The selected voltage.
 */
static void set_buck_voltage(npmx_instance_t *p_pm, npmx_buck_instance_t buck_instance,
			     npmx_buck_voltage_t voltage)
{
	/* Set the output voltage. */
	if (npmx_buck_normal_voltage_set(p_pm, buck_instance, voltage) == false) {
		LOG_ERR("Unable to set normal voltage");
	}

	/* Have to be called each time to change output voltage. */
	if (npmx_buck_vout_select(p_pm, buck_instance, NPMX_BUCK_VOUT_SELECT_SOFTWARE) == false) {
		LOG_ERR("Unable to select vout reference");
	}
}

/**
 * @brief Function for reading and logging selected voltage from the specified buck instance.
 * This function does not read the actual output voltage of the buck converter.
 * 
 * @param[in] p_pm The pointer to the instance of nPM device.
 * @param[in] buck_instance The buck instance to read and log the voltage from.
 */
static void get_and_log_voltage(npmx_instance_t *p_pm, npmx_buck_instance_t buck_instance)
{
	npmx_buck_voltage_t buck_voltage;
	/* Get the output voltage, and LOG voltage value. */
	if (npmx_buck_status_get(p_pm, buck_instance, &buck_voltage) == false) {
		LOG_ERR("Unable to get voltage status");
	}
	/* Get voltage from enum index value. */
	/* Magic scalar, when buck_voltage is 0 (NPMX_BUCK_VOLTAGE_1V0) it means 1.0 V. */
	uint8_t temp = buck_voltage + 10;
	uint8_t voltage = temp / 10; /* Get the unity digit of the voltage. */
	uint8_t voltage_frac = temp % 10; /* Get fractional digit of the voltage. */
	LOG_INF("Voltage %s: %d.%d V", buck_instance_to_str(buck_instance), voltage, voltage_frac);
}

/**
 * @brief Function for testing buck voltage range, from start_voltage to NPMX_BUCK_VOLTAGE_MAX with step every 200 ms.
 * 
 * @param[in] p_pm The pointer to the instance of nPM device.
 * @param[in] buck_instance The buck instance to test for.
 * @param[in] start_voltage The start test voltage level.
 */
static void test_buck_voltage(npmx_instance_t *p_pm, npmx_buck_instance_t buck_instance,
			      npmx_buck_voltage_t start_voltage)
{
	for (uint8_t i = start_voltage; i < NPMX_BUCK_VOLTAGE_MAX; i++) {
		/* Set the expected output voltage. */
		set_buck_voltage(p_pm, buck_instance, (npmx_buck_voltage_t)i);

		/* Wait some time to stabilize and measure the output voltage. */
		k_msleep(200);

		/* Check in nPM device which voltage is selected. */
		get_and_log_voltage(p_pm, buck_instance);
	}
}

/**
 * @brief Function for testing the output voltage of both BUCKs.
 * 
 * @param[in] p_pm The pointer to the instance of nPM device.
 */
static void test_output_voltage(npmx_instance_t *p_pm) {
	LOG_INF("Test output voltage");

	/* Voltage output testing for BUCK1. */
	test_buck_voltage(p_pm, NPMX_BUCK_INSTANCE_1, NPMX_BUCK_VOLTAGE_1V0);

	/* Voltage output testing for BUCK2. */
	test_buck_voltage(p_pm, NPMX_BUCK_INSTANCE_2, NPMX_BUCK_VOLTAGE_2V0);

	/* Go back to nominal voltage based on connected resistors. */
	/* Note: without the load on output pins, the voltage can drop down a little slow. */
	npmx_buck_vout_select(p_pm, NPMX_BUCK_INSTANCE_1, NPMX_BUCK_VOUT_SELECT_VSET_PIN);
	npmx_buck_vout_select(p_pm, NPMX_BUCK_INSTANCE_2, NPMX_BUCK_VOUT_SELECT_VSET_PIN);

	/* With GoBoard and connected resistors, output voltage should be:
		- 1.0 V for BUCK1
		- 2.0 V for BUCK2 
	*/
	get_and_log_voltage(p_pm, NPMX_BUCK_INSTANCE_1);
	get_and_log_voltage(p_pm, NPMX_BUCK_INSTANCE_2);
}

/**
 * @brief Function for testing the retention voltage with the selected external pin.
 * 
 * @param[in] p_pm The pointer to the instance of nPM device.
 */
static void test_retention_voltage(npmx_instance_t *p_pm) {
		/* GPIO1 as BUCK1 retention select. */
	LOG_INF("Test retention voltage");

	/* Switch GPIO1 to input mode. */
	if (npmx_gpio_mode_set(p_pm, NPMX_GPIO_INSTANCE_1, NPMX_GPIO_MODE_INPUT) == false) {
		LOG_ERR("Unable to switch GPIO1 to input mode");
	}

	/* Select voltages: in normal mode should be 1.6 V, in retention mode should be 3.3 V. */
	if (npmx_buck_normal_voltage_set(p_pm, NPMX_BUCK_INSTANCE_1, NPMX_BUCK_VOLTAGE_1V6) == false) {
		LOG_ERR("Unable to set normal voltage");
	}

	if (npmx_buck_retention_voltage_set(p_pm, NPMX_BUCK_INSTANCE_1,
					    NPMX_BUCK_VOLTAGE_3V3) == false) {
		LOG_ERR("Unable to set retention voltage");
	}

	/* Apply voltages. */
	if (npmx_buck_vout_select(p_pm, NPMX_BUCK_INSTANCE_1,
				  NPMX_BUCK_VOUT_SELECT_SOFTWARE) == false) {
		LOG_ERR("Unable to select vout reference");
	}

	/* Select GPIO in buck instance. If inversion is false, retention is active when GPIO1 is in HIGH state. */
	if (npmx_buck_retention_gpio_select(p_pm, NPMX_BUCK_INSTANCE_1, NPMX_BUCK_GPIO_1,
					    false) == false) {
		LOG_ERR("Unable to select retention gpio");
	}
}

/**
 * @brief Function for testing enable mode with the selected external pin.
 * 
 * @param[in] p_pm The pointer to the instance of nPM device.
 */
static void test_enable_bucks(npmx_instance_t *p_pm) {
	/* Enable and disable BUCK converter using selected GPIO pin. */
	LOG_INF("Test enable BUCK via connected pin");

	/* Disable BUCK instance for test. */
	if (npmx_buck_status_set(p_pm, NPMX_BUCK_INSTANCE_1, false) == false) {
		LOG_ERR("Unable to disable buck");
	}

	/* Switch GPIO3 to input mode. */
	if (npmx_gpio_mode_set(p_pm, NPMX_GPIO_INSTANCE_3, NPMX_GPIO_MODE_INPUT) == false) {
		LOG_ERR("Unable to switch GPIO3 to input mode");
	}

	/* Select output voltage to be 3.3 V. */
	set_buck_voltage(p_pm, NPMX_BUCK_INSTANCE_1, NPMX_BUCK_VOLTAGE_3V3);

	/* When GPIO3 changes to HIGH state, BUCK1 will start working. */
	if (npmx_buck_enable_gpio_select(p_pm, NPMX_BUCK_INSTANCE_1, NPMX_BUCK_GPIO_3,
					 false) == false) {
		LOG_ERR("Unable to connect GPIO3 to BUCK1");
	}

	/* For faster capacitor discharge, when there is no load connected to GoBoard. */
	if (npmx_buck_active_discharge_enable(p_pm, NPMX_BUCK_INSTANCE_1, true) == false) {
		LOG_ERR("Unable to activate auto discharge mode");
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

	/* After reset, BUCK converters are enabled by default, but to be sure during testing enable both at the beginning. */
	npmx_buck_status_set(npmx_instance, NPMX_BUCK_INSTANCE_1, true);
	npmx_buck_status_set(npmx_instance, NPMX_BUCK_INSTANCE_2, true);

#if SELECTED_TESTCASE == TESTCASE_SET_BUCK_VOLTAGE
	/* Set output voltages for BUCK1 and BUCK2. */
	set_buck_voltage(npmx_instance, NPMX_BUCK_INSTANCE_1, NPMX_BUCK_VOLTAGE_1V5);
	set_buck_voltage(npmx_instance, NPMX_BUCK_INSTANCE_2, NPMX_BUCK_VOLTAGE_2V0);
#elif SELECTED_TESTCASE == TESTCASE_OUTPUT_VOLTAGE
	test_output_voltage(npmx_instance);
#elif SELECTED_TESTCASE == TESTCASE_RETENTION_VOLTAGE
	test_retention_voltage(npmx_instance);
#elif SELECTED_TESTCASE == TESTCASE_ENABLE_VIA_PINS
	test_enable_bucks(npmx_instance);
#endif

	while (1) {
		k_sleep(K_FOREVER);
	}
}
