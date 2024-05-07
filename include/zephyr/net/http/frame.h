/*
 * Copyright (c) 2023, Emna Rekik
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_NET_HTTP_SERVER_FRAME_H_
#define ZEPHYR_INCLUDE_NET_HTTP_SERVER_FRAME_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum http_frame_type {
	HTTP_SERVER_DATA_FRAME = 0x00,
	HTTP_SERVER_HEADERS_FRAME = 0x01,
	HTTP_SERVER_PRIORITY_FRAME = 0x02,
	HTTP_SERVER_RST_STREAM_FRAME = 0x03,
	HTTP_SERVER_SETTINGS_FRAME = 0x04,
	HTTP_SERVER_PUSH_PROMISE_FRAME = 0x05,
	HTTP_SERVER_PING_FRAME = 0x06,
	HTTP_SERVER_GOAWAY_FRAME = 0x07,
	HTTP_SERVER_WINDOW_UPDATE_FRAME = 0x08,
	HTTP_SERVER_CONTINUATION_FRAME = 0x09
};

#define HTTP_SERVER_HPACK_METHOD 0
#define HTTP_SERVER_HPACK_PATH   1

#define HTTP_SERVER_FLAG_SETTINGS_ACK 0x1
#define HTTP_SERVER_FLAG_END_HEADERS  0x4
#define HTTP_SERVER_FLAG_END_STREAM   0x1

#define HTTP_SERVER_FRAME_HEADER_SIZE      9
#define HTTP_SERVER_FRAME_LENGTH_OFFSET    0
#define HTTP_SERVER_FRAME_TYPE_OFFSET      3
#define HTTP_SERVER_FRAME_FLAGS_OFFSET     4
#define HTTP_SERVER_FRAME_STREAM_ID_OFFSET 5

struct http_settings_field {
	uint16_t id;
	uint32_t value;
} __packed;

enum http_settings {
	HTTP_SETTINGS_HEADER_TABLE_SIZE = 1,
	HTTP_SETTINGS_ENABLE_PUSH = 2,
	HTTP_SETTINGS_MAX_CONCURRENT_STREAMS = 3,
	HTTP_SETTINGS_INITIAL_WINDOW_SIZE = 4,
	HTTP_SETTINGS_MAX_FRAME_SIZE = 5,
	HTTP_SETTINGS_MAX_HEADER_LIST_SIZE = 6,
};

#ifdef __cplusplus
}
#endif

#endif
