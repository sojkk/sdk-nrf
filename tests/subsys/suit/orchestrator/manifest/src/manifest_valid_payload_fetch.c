/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#if defined(CONFIG_SOC_NRF52840) || defined(CONFIG_BOARD_NATIVE_POSIX)
#include <stdint.h>
#include <stddef.h>

/** @brief Valid SUIT envelope, based on ../manifest/sample_valid_root_payload_fetch.yaml
 *
 */
const uint8_t manifest_valid_payload_fetch_buf[] = {
	0xD8, 0x6B, 0xA3, 0x02, 0x58, 0x7A, 0x82, 0x58, 0x24, 0x82, 0x2F, 0x58, 0x20, 0xFD, 0xF8,
	0x9A, 0x91, 0xAB, 0xF9, 0x01, 0x4B, 0xB1, 0x33, 0x32, 0x49, 0xFC, 0x71, 0xE1, 0x7E, 0x7E,
	0xEC, 0x57, 0xDE, 0xD2, 0xBB, 0x1E, 0x2D, 0x30, 0xB5, 0xA9, 0xA0, 0xE0, 0x9A, 0x32, 0x27,
	0x58, 0x51, 0xD2, 0x84, 0x4A, 0xA2, 0x01, 0x26, 0x04, 0x45, 0x1A, 0x40, 0x00, 0xAA, 0x00,
	0xA0, 0xF6, 0x58, 0x40, 0xD7, 0x8F, 0x47, 0xBA, 0x24, 0xED, 0x01, 0x8E, 0x5A, 0x31, 0x57,
	0x83, 0xB6, 0x2F, 0x55, 0x12, 0x0D, 0x4B, 0x89, 0x0C, 0xF5, 0x72, 0xAB, 0x39, 0xA8, 0x2F,
	0x67, 0x9E, 0x3C, 0x7F, 0xA7, 0x12, 0x44, 0x7D, 0x13, 0xD0, 0x43, 0xDE, 0xC1, 0x1F, 0x9B,
	0xF1, 0x36, 0xB9, 0x06, 0x6A, 0x54, 0xA2, 0x75, 0x35, 0x5C, 0x9D, 0x43, 0x6A, 0xA1, 0xEB,
	0x27, 0xAC, 0xAB, 0xAB, 0x5D, 0xDB, 0x91, 0xE2, 0x03, 0x59, 0x01, 0x1A, 0xAA, 0x01, 0x01,
	0x02, 0x01, 0x03, 0x58, 0x78, 0xA3, 0x02, 0x83, 0x82, 0x4A, 0x69, 0x43, 0x41, 0x4E, 0x44,
	0x5F, 0x4D, 0x46, 0x53, 0x54, 0x41, 0x00, 0x82, 0x4C, 0x6B, 0x49, 0x4E, 0x53, 0x54, 0x4C,
	0x44, 0x5F, 0x4D, 0x46, 0x53, 0x54, 0x50, 0x08, 0xC1, 0xB5, 0x99, 0x55, 0xE8, 0x5F, 0xBC,
	0x9E, 0x76, 0x7B, 0xC2, 0x9C, 0xE1, 0xB0, 0x4D, 0x82, 0x4B, 0x6A, 0x43, 0x41, 0x43, 0x48,
	0x45, 0x5F, 0x50, 0x4F, 0x4F, 0x4C, 0x41, 0x00, 0x04, 0x58, 0x30, 0x8A, 0x0C, 0x01, 0x14,
	0xA2, 0x01, 0x50, 0x76, 0x17, 0xDA, 0xA5, 0x71, 0xFD, 0x5A, 0x85, 0x8F, 0x94, 0xE2, 0x8D,
	0x73, 0x5C, 0xE9, 0xF4, 0x02, 0x50, 0x3F, 0x6A, 0x3A, 0x4D, 0xCD, 0xFA, 0x58, 0xC5, 0xAC,
	0xCE, 0xF9, 0xF5, 0x84, 0xC4, 0x11, 0x24, 0x0C, 0x81, 0x01, 0x01, 0x0F, 0x02, 0x0F, 0x01,
	0xA2, 0x00, 0xA0, 0x01, 0xA0, 0x07, 0x48, 0x86, 0x0C, 0x81, 0x01, 0x07, 0x0F, 0x0B, 0x0F,
	0x09, 0x48, 0x86, 0x0C, 0x81, 0x01, 0x07, 0x0F, 0x0B, 0x0F, 0x10, 0x55, 0x8A, 0x0C, 0x00,
	0x14, 0xA1, 0x15, 0x68, 0x61, 0x70, 0x70, 0x2E, 0x73, 0x75, 0x69, 0x74, 0x15, 0x02, 0x07,
	0x0F, 0x0B, 0x0F, 0x11, 0x55, 0x8A, 0x0C, 0x00, 0x14, 0xA1, 0x15, 0x68, 0x61, 0x70, 0x70,
	0x2E, 0x73, 0x75, 0x69, 0x74, 0x15, 0x02, 0x07, 0x0F, 0x0B, 0x0F, 0x17, 0x82, 0x2F, 0x58,
	0x20, 0xB8, 0x36, 0xCD, 0xE4, 0x01, 0x13, 0xBD, 0x77, 0x5D, 0xE0, 0xEC, 0x07, 0xFC, 0x3E,
	0xBC, 0x5F, 0x36, 0x8F, 0x8C, 0xA5, 0x05, 0xC2, 0x78, 0x7D, 0x1A, 0xED, 0x46, 0xAB, 0x8F,
	0x6B, 0x05, 0x89, 0x0F, 0x51, 0x86, 0x0C, 0x02, 0x14, 0xA1, 0x15, 0x68, 0x61, 0x70, 0x70,
	0x2E, 0x73, 0x75, 0x69, 0x74, 0x15, 0x02, 0x05, 0x82, 0x4C, 0x6B, 0x49, 0x4E, 0x53, 0x54,
	0x4C, 0x44, 0x5F, 0x4D, 0x46, 0x53, 0x54, 0x50, 0x3F, 0x6A, 0x3A, 0x4D, 0xCD, 0xFA, 0x58,
	0xC5, 0xAC, 0xCE, 0xF9, 0xF5, 0x84, 0xC4, 0x11, 0x24, 0x17, 0x58, 0x84, 0xA1, 0x62, 0x65,
	0x6E, 0xA1, 0x82, 0x4A, 0x69, 0x43, 0x41, 0x4E, 0x44, 0x5F, 0x4D, 0x46, 0x53, 0x54, 0x41,
	0x00, 0xA6, 0x01, 0x78, 0x18, 0x4E, 0x6F, 0x72, 0x64, 0x69, 0x63, 0x20, 0x53, 0x65, 0x6D,
	0x69, 0x63, 0x6F, 0x6E, 0x64, 0x75, 0x63, 0x74, 0x6F, 0x72, 0x20, 0x41, 0x53, 0x41, 0x02,
	0x68, 0x6E, 0x52, 0x46, 0x35, 0x34, 0x48, 0x32, 0x30, 0x03, 0x6E, 0x6E, 0x6F, 0x72, 0x64,
	0x69, 0x63, 0x73, 0x65, 0x6D, 0x69, 0x2E, 0x63, 0x6F, 0x6D, 0x04, 0x78, 0x1A, 0x54, 0x68,
	0x65, 0x20, 0x6E, 0x52, 0x46, 0x35, 0x34, 0x48, 0x32, 0x30, 0x20, 0x72, 0x6F, 0x6F, 0x74,
	0x20, 0x6D, 0x61, 0x6E, 0x69, 0x66, 0x65, 0x73, 0x74, 0x05, 0x74, 0x53, 0x61, 0x6D, 0x70,
	0x6C, 0x65, 0x20, 0x72, 0x6F, 0x6F, 0x74, 0x20, 0x6D, 0x61, 0x6E, 0x69, 0x66, 0x65, 0x73,
	0x74, 0x06, 0x66, 0x76, 0x31, 0x2E, 0x30, 0x2E, 0x30};

const size_t manifest_valid_payload_fetch_len = sizeof(manifest_valid_payload_fetch_buf);

#endif /* CONFIG_SOC_NRF52840 || CONFIG_BOARD_NATIVE_POSIX */