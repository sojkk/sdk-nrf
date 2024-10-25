/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdint.h>
#include <stddef.h>

/** @brief Valid SUIT envelope, based on ../recovery_validate_load_invoke.yaml
 *
 */
const uint8_t manifest_recovery_validate_load_invoke_buf[] = {
	0xD8, 0x6B, 0xA2, 0x02, 0x58, 0x7A, 0x82, 0x58, 0x24, 0x82, 0x2F, 0x58, 0x20, 0x01, 0x91,
	0xDA, 0x1A, 0x63, 0xEB, 0x7C, 0x1A, 0x32, 0x6B, 0xA4, 0xCE, 0x48, 0xDF, 0xAE, 0x51, 0x3F,
	0x86, 0x8C, 0xE4, 0xFE, 0xA8, 0x99, 0xE5, 0x82, 0xE3, 0xB9, 0xBB, 0x8E, 0xE1, 0x49, 0x13,
	0x58, 0x51, 0xD2, 0x84, 0x4A, 0xA2, 0x01, 0x26, 0x04, 0x45, 0x1A, 0x40, 0x00, 0x00, 0x00,
	0xA0, 0xF6, 0x58, 0x40, 0xC1, 0x6A, 0x22, 0xBC, 0x91, 0xDA, 0x46, 0xD4, 0x35, 0x61, 0xA9,
	0x18, 0x67, 0x3D, 0xC1, 0xFB, 0xB8, 0x47, 0x78, 0x65, 0x49, 0x9E, 0xF3, 0x70, 0x7D, 0xC6,
	0xE4, 0xEF, 0xAD, 0xF8, 0x38, 0x55, 0x79, 0xE9, 0xF0, 0x91, 0x8F, 0xC9, 0xFB, 0xCE, 0x06,
	0x7D, 0xF1, 0xAF, 0xE5, 0xE0, 0xCA, 0x4E, 0x49, 0x9B, 0xFF, 0xDC, 0xF5, 0x1E, 0xD6, 0xC2,
	0x09, 0xB1, 0x60, 0x40, 0x78, 0x86, 0x3F, 0xE7, 0x03, 0x58, 0x76, 0xA7, 0x01, 0x01, 0x02,
	0x01, 0x03, 0x58, 0x3F, 0xA3, 0x02, 0x81, 0x82, 0x4A, 0x69, 0x43, 0x41, 0x4E, 0x44, 0x5F,
	0x4D, 0x46, 0x53, 0x54, 0x41, 0x00, 0x04, 0x58, 0x27, 0x82, 0x14, 0xA2, 0x01, 0x50, 0x76,
	0x17, 0xDA, 0xA5, 0x71, 0xFD, 0x5A, 0x85, 0x8F, 0x94, 0xE2, 0x8D, 0x73, 0x5C, 0xE9, 0xF4,
	0x02, 0x50, 0x74, 0xA0, 0xC6, 0xE7, 0xA9, 0x2A, 0x56, 0x00, 0x9C, 0x5D, 0x30, 0xEE, 0x87,
	0x8B, 0x06, 0xBA, 0x01, 0xA1, 0x00, 0xA0, 0x07, 0x43, 0x82, 0x0C, 0x00, 0x08, 0x43, 0x82,
	0x0C, 0x00, 0x09, 0x43, 0x82, 0x0C, 0x00, 0x05, 0x82, 0x4C, 0x6B, 0x49, 0x4E, 0x53, 0x54,
	0x4C, 0x44, 0x5F, 0x4D, 0x46, 0x53, 0x54, 0x50, 0x74, 0xA0, 0xC6, 0xE7, 0xA9, 0x2A, 0x56,
	0x00, 0x9C, 0x5D, 0x30, 0xEE, 0x87, 0x8B, 0x06, 0xBA};

const size_t manifest_recovery_validate_load_invoke_len =
	sizeof(manifest_recovery_validate_load_invoke_buf);