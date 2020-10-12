/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef _VOICE_EVENT_H_
#define _VOICE_EVENT_H_

/**
 * @brief Motion Event
 * @defgroup motion_event Motion Event
 * @{
 */

#include "event_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

struct voice_event {
	struct event_header header;

	u8_t data[16]; /**<Voice data array */
};

EVENT_TYPE_DECLARE(voice_event);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* _MOTION_EVENT_H_ */
