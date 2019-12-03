/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>

/*
 * Zephyr supports the following log levels:
 * 0 OFF - do not log
 * 1 ERROR - only LOG_LEVEL_ERR
 * 2 WARNING - LOG_LEVEL_WRN or higher
 * 3 INFO - LOG_LEVEL_INFO or higher
 * 4 DEBUG - LOG_LEVEL_DBG or higher
 */

#include <logging/log.h>
#define MODULE_LOG_LEVEL 4
LOG_MODULE_REGISTER(our_sample, MODULE_LOG_LEVEL);

void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);
	LOG_ERR("Error");
	LOG_WRN("Warning");
	LOG_INF("Info");
	LOG_DBG("Debug");
}
