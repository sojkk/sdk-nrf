/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <stdio.h>

#include "voice_event.h"

EVENT_TYPE_DEFINE(voice_event,
		  IS_ENABLED(CONFIG_DESKTOP_INIT_LOG_VOICE_EVENT),
		  NULL,
		  NULL);


/*
static int log_voice_event(const struct event_header *eh, char *buf,
				size_t buf_len)
{
	const struct voice_event *event = cast_voice_event(eh);

	
	return snprintf(buf, buf_len, "1st data=%d", event->data[0]);
}

static void profile_voice_event(struct log_event_buf *buf, 
				    const struct event_header *eh)
{
	const struct voice_event *event = cast_voice_event(eh);

	profiler_log_encode_u32(buf, (u32_t) event->data);
}


EVENT_INFO_DEFINE(voice_event,
		  ENCODE(PROFILER_ARG_U8),
		  ENCODE("data"),
		  profile_voice_event);


EVENT_TYPE_DEFINE(voice_event,
		  IS_ENABLED(CONFIG_DESKTOP_INIT_LOG_VOICE_EVENT),
		  log_voice_event,
		  &voice_event_info);

*/