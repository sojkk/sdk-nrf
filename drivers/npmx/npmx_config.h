/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_NPMX_NPMX_CONFIG_H__
#define ZEPHYR_DRIVERS_NPMX_NPMX_CONFIG_H__

#if defined(NPM1300_FP1)
    #include <npmx_config_npm1300_fp1.h>
#else
    #error "Unknown device."
#endif

#endif // ZEPHYR_DRIVERS_NPMX_NPMX_CONFIG_H__
