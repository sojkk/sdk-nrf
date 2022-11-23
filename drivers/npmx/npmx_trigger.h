/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_NPMX_NPMX_TRIGGER_H__
#define ZEPHYR_DRIVERS_NPMX_NPMX_TRIGGER_H__

#include <zephyr/device.h>

/**
 * @brief Initialize gpio interrupt to handle events from nPM device
 * 
 * @param[in] dev The pointer to nPM Zephyr device
 * @return int Return error code, 0 when succeed
 */
int npmx_trigger_gpio_interrupt_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_NPMX_NPMX_TRIGGER_H__ */
