/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>

#include "hid_report_desc.h"

const uint8_t hid_report_desc_0[] = {

0x05, 0x01,                        //     Usage Page (Generic Desktop)
0x09, 0x02,                        //     Usage (Mouse)

0xA1, 0x01,                        //     Collection (Application)

// Report ID 1: Mouse buttons + scroll/pan
0x85, 0x01,                        //     Report Id 1 
0x09, 0x01,                        //     Usage (Pointer)
0xA1, 0x00,                        //     Collection (Physical)
0x95, 0x05,                        //     Report Count (3)
0x75, 0x01,                        //     Report Size (1)
0x05, 0x09,                        //     Usage Page (Buttons)
0x19, 0x01,                        //     Usage Minimum (01)
0x29, 0x05,                        //     Usage Maximum (05)
0x15, 0x00,                        //     Logical Minimum (0)
0x25, 0x01,                        //     Logical Maximum (1)
0x81, 0x02,                        //     Input (Data, Variable, Absolute)
0x95, 0x01,                        //     Report Count (1)
0x75, 0x03,                        //     Report Size (3)
0x81, 0x01,                        //     Input (Constant) for padding
0x75, 0x08,                        //     Report Size (8)
0x95, 0x01,                        //     Report Count (1)
0x05, 0x01,                        //     Usage Page (Generic Desktop)
0x09, 0x38,                        //     USAGE (Wheel)
0x15, 0x81,                        //     Logical Minimum (-127)
0x25, 0x7F,                        //     Logical Maximum (127)
0x81, 0x06,                        //     Input (Data, Variable, Relative) 
0x05, 0x0C,                        //     Usage Page (Consumer)
0x0A, 0x38, 0x02,                  //     Usage (AC Pan) 
0x95, 0x01,                        //     Report Count (1)
0x81, 0x06,                        //     Input (Data,Value,Relative,Bit Field)
0xC0,                              //     End Collection (Physical)

// Report ID 2: Mouse motion
0x85, 0x02,                        //     Report Id 2 
0x09, 0x01,                        //     Usage (Pointer)
0xA1, 0x00,                        //     Collection (Physical)
0x75, 0x0C,                        //     Report Size (12)
0x95, 0x02,                        //     Report Count (2)
0x05, 0x01,                        //     Usage Page (Generic Desktop)
0x09, 0x30,                        //     Usage (X)                                                  
0x09, 0x31,                        //     Usage (Y)
0x16, 0x01, 0xF8,                  //     Logical maximum (2047) 
0x26, 0xFF, 0x07,                  //     Logical minimum (-2047) 
0x81, 0x06,                        //     Input (Data, Variable, Relative) 
0xC0,                              //   End Collection (Physical) 
0xC0,                              // End Collection (Application)

// Report ID 3: Advanced buttons
0x05, 0x0C,                         // Usage Page (Consumer)
0x09, 0x01,                         // Usage (Consumer Control) 
0xA1, 0x01,                         // Collection (Application)
0x85, 0x03,                         // Report Id (3) 
0x15, 0x00,                         // Logical minimum (0)
0x25, 0x01,                         // Logical maximum (1) 
0x75, 0x01,                         // Report Size (1)
0x95, 0x01,                         // Report Count (1)

0x09, 0xCD,                         // Usage (Play/Pause) 
0x81, 0x06,                         // Input (Data,Value,Relative,Bit Field)
0x0A, 0x83, 0x01,                   // Usage (AL Consumer Control Configuration) 
0x81, 0x06,                         // Input (Data,Value,Relative,Bit Field)
0x09, 0xB5,                         // Usage (Scan Next Track)
0x81, 0x06,                         // Input (Data,Value,Relative,Bit Field)   
0x09, 0xB6,                         // Usage (Scan Previous Track)
0x81, 0x06,                         // Input (Data,Value,Relative,Bit Field) 

0x09, 0xEA,                         //Usage (Volume Down)
0x81, 0x06,                         // Input (Data,Value,Relative,Bit Field) 
0x09, 0xE9,                         //Usage (Volume Up) 
0x81, 0x06,                         // Input (Data,Value,Relative,Bit Field) 
0x0A, 0x25, 0x02,                   // Usage (AC Forward) 
0x81, 0x06,                         // Input (Data,Value,Relative,Bit Field) 
0x0A, 0x24, 0x02,                   // Usage (AC Back) 
0x81, 0x06,                         // Input (Data,Value,Relative,Bit Field) 
0xC0                                // End Collection

};

const size_t hid_report_desc_size_0 = sizeof(hid_report_desc_0);


const uint8_t hid_report_desc_1[] = {
	
0x05, 0x01,                         // Usage Page (Generic Desktop)
0x09, 0x06,                         // Usage (Keyboard)
0xA1, 0x01,                         // Collection (Application)
0x05, 0x07,                         //     Usage Page (Key Codes)
0x19, 0xe0,                         //     Usage Minimum (224)
0x29, 0xe7,                         //     Usage Maximum (231)
0x15, 0x00,                         //     Logical Minimum (0)
0x25, 0x01,                         //     Logical Maximum (1)
0x75, 0x01,                         //     Report Size (1)
0x95, 0x08,                         //     Report Count (8)
0x81, 0x02,                         //     Input (Data, Variable, Absolute)

0x95, 0x01,                         //     Report Count (1)
0x75, 0x08,                         //     Report Size (8)
0x81, 0x01,                         //     Input (Constant) reserved byte(1)

0x95, 0x05,                         //     Report Count (5)
0x75, 0x01,                         //     Report Size (1)
0x05, 0x08,                         //     Usage Page (Page# for LEDs)
0x19, 0x01,                         //     Usage Minimum (1)
0x29, 0x05,                         //     Usage Maximum (5)
0x91, 0x02,                         //     Output (Data, Variable, Absolute), Led report
0x95, 0x01,                         //     Report Count (1)
0x75, 0x03,                         //     Report Size (3)
0x91, 0x01,                         //     Output (Data, Variable, Absolute), Led report padding

0x95, 0x06,                         //     Report Count (6)
0x75, 0x08,                         //     Report Size (8)
0x15, 0x00,                         //     Logical Minimum (0)
0x25, 0x65,                         //     Logical Maximum (101)
0x05, 0x07,                         //     Usage Page (Key codes)
0x19, 0x00,                         //     Usage Minimum (0)
0x29, 0x65,                         //     Usage Maximum (101)
0x81, 0x00,                         //     Input (Data, Array) Key array(6 bytes)	
	
};

const size_t hid_report_desc_size_1 = sizeof(hid_report_desc_1);
