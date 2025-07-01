#ifndef _IMU_DESCRIPTOR_H_
#define _IMU_DESCRIPTOR_H_

#include <stdint.h>

// Custom HID Report Descriptor for orientation data (yaw, pitch, roll) and acceleration magnitude
static const uint8_t imu_hid_report_desc[] = {
    0x05, 0x20,        // Usage Page (Sensor)
    0x09, 0x8A,        // Usage (Motion: Orientation)
    0xA1, 0x01,        // Collection (Application)
    
    // Yaw (rotation around Z-axis)
    0x05, 0x20,        //   Usage Page (Sensor)
    0x09, 0x8D,        //   Usage (Orientation: Yaw)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8 bits)
    0x95, 0x01,        //   Report Count (1)
    0x55, 0x00,        //   Unit Exponent (0)
    0x65, 0x14,        //   Unit (Degrees)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Pitch (rotation around X-axis)
    0x09, 0x8E,        //   Usage (Orientation: Pitch)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Roll (rotation around Y-axis)
    0x09, 0x8F,        //   Usage (Orientation: Roll)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Acceleration Magnitude
    0x05, 0x20,        //   Usage Page (Sensor)
    0x09, 0x73,        //   Usage (Motion: Acceleration)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8 bits)
    0x95, 0x01,        //   Report Count (1)
    0x55, 0x00,        //   Unit Exponent (0)
    0x66, 0x14, 0xF0,  //   Unit (m/s²)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    0xC0,              // End Collection
};

#define IMU_HID_REPORT_DESC_SIZE sizeof(imu_hid_report_desc)

// Orientation and magnitude report structure
typedef struct {
    uint8_t yaw;        // Yaw angle (0-255 representing 0-360 degrees)
    uint8_t pitch;      // Pitch angle (0-255 representing -90 to +90 degrees)
    uint8_t roll;       // Roll angle (0-255 representing -180 to +180 degrees)
    uint8_t magnitude;  // High-pass filtered acceleration magnitude (0-255 representing 0-25.0 m/s² dynamic motion)
} __attribute__((packed)) imu_report_t;

#endif // _IMU_DESCRIPTOR_H_ 