/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <npmx_core.h>
#include <npmx_trigger.h>
#include <npmx_driver.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(NPMX, CONFIG_NPMX_LOG_LEVEL);

#define DT_DRV_COMPAT nordic_npm1300

static void generic_callback(npmx_instance_t *pm, npmx_callback_type_t type, uint32_t arg)
{
	LOG_DBG("%s:", npmx_callback_to_str(type));
	for (uint8_t i = 0; i < 8; i++) {
		if ((1U << i) & arg) {
			LOG_DBG("\t%s", npmx_callback_bit_to_str(type, i));
		}
	}
}

static int npmx_driver_init(const struct device *dev)
{
	struct npmx_data *data = dev->data;
	const struct npmx_config *config = dev->config;
	const struct device *bus = config->i2c.bus;
	npmx_backend_instance_t *const backend = &data->npmx_instance.backend_inst;

	if (!device_is_ready(bus)) {
		LOG_ERR("%s: bus device %s is not ready", dev->name, bus->name);
		return -ENODEV;
	}

	npmx_backend_init(backend, (void *)bus, config->i2c.addr);

	data->dev = dev;

	data->npmx_instance.generic_cb = generic_callback;

	if (npmx_core_init(&data->npmx_instance) == false) {
		LOG_ERR("Unable to init npmx device");
		return -EIO;
	}

	/* Clear all events before enabling interrupts. */
	for (uint32_t i = 0; i < NPMX_EVENT_GROUP_COUNT; i++) {
		if (npmx_core_event_interrupt_disable(&data->npmx_instance, (npmx_event_group_t)i,
						      NPMX_EVENT_GROUP_ALL_EVENTS_MASK) == false) {
			LOG_ERR("Failed to disable interrupts");
			return -EIO;
		}
	}

	if (npmx_trigger_gpio_interrupt_init(dev) < 0) {
		LOG_ERR("Failed to initialize interrupt");
		return -EIO;
	}

	return 0;
};

#define NPMX_DEFINE(inst)                                                                          \
	static struct npmx_data npmx_data_##inst;                                                  \
	static const struct npmx_config npmx_config_##inst = {                                     \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.int_gpio = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, npmx_driver_init, NULL, &npmx_data_##inst,                     \
			      &npmx_config_##inst, POST_KERNEL, CONFIG_NPMX_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(NPMX_DEFINE)
