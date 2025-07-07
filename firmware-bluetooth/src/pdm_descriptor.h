#pragma once

#include <stdint.h>

// Custom HID Report Descriptor for PDM audio data (audio level, frequency, volume)
static const uint8_t pdm_hid_report_desc[] = {
    0x05, 0x20,        // Usage Page (Sensor)
    0x09, 0x8B,        // Usage (Motion: Audio)
    0xA1, 0x01,        // Collection (Application)
    
    // Audio Level (RMS)
    0x05, 0x20,        //   Usage Page (Sensor)
    0x09, 0x90,        //   Usage (Audio: Level)
    0x16, 0x00, 0x80,  //   Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,  //   Logical Maximum (+32767)
    0x75, 0x10,        //   Report Size (16 bits)
    0x95, 0x01,        //   Report Count (1)
    0x55, 0x00,        //   Unit Exponent (0)
    0x65, 0x00,        //   Unit (None)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Frequency
    0x09, 0x91,        //   Usage (Audio: Frequency)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0xFF,  //   Logical Maximum (65535)
    0x75, 0x10,        //   Report Size (16 bits)
    0x95, 0x01,        //   Report Count (1)
    0x55, 0x00,        //   Unit Exponent (0)
    0x65, 0x00,        //   Unit (None)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Volume (Peak)
    0x09, 0x92,        //   Usage (Audio: Volume)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8 bits)
    0x95, 0x01,        //   Report Count (1)
    0x55, 0x00,        //   Unit Exponent (0)
    0x65, 0x00,        //   Unit (None)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    0xC0,              // End Collection
};

#define PDM_HID_REPORT_DESC_SIZE sizeof(pdm_hid_report_desc)

// Audio report structure
typedef struct {
    int16_t audio_level;  // RMS audio level (-32768 to +32767)
    uint16_t frequency;   // Estimated frequency (0 to 65535 Hz)
    uint8_t volume;       // Peak volume (0 to 255)
} __attribute__((packed)) pdm_report_t; 