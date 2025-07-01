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
    0x27, 0xFF, 0xFF, 0x00, 0x00,  //   Logical Maximum (65535)
    0x75, 0x10,        //   Report Size (16 bits)
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
    0x27, 0xFF, 0xFF, 0x00, 0x00,  //   Logical Maximum (65535)
    0x75, 0x10,        //   Report Size (16 bits)
    0x95, 0x01,        //   Report Count (1)
    0x55, 0x00,        //   Unit Exponent (0)
    0x66, 0x14, 0xF0,  //   Unit (m/s²)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Raw Accelerometer Data (X, Y, Z)
    0x05, 0x20,        //   Usage Page (Sensor)
    0x0A, 0x50, 0x04,  //   Usage (Acceleration X)
    0x16, 0x00, 0x80,  //   Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,  //   Logical Maximum (32767)
    0x75, 0x10,        //   Report Size (16 bits)
    0x95, 0x01,        //   Report Count (1)
    0x55, 0x0E,        //   Unit Exponent (-2, for 0.01 units)
    0x66, 0x14, 0xF0,  //   Unit (m/s²)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    0x0A, 0x51, 0x04,  //   Usage (Acceleration Y)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    0x0A, 0x52, 0x04,  //   Usage (Acceleration Z)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Raw Gyroscope Data (X, Y, Z)
    0x0A, 0x53, 0x04,  //   Usage (Angular Velocity X)
    0x66, 0x14, 0x00,  //   Unit (degrees/s)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    0x0A, 0x54, 0x04,  //   Usage (Angular Velocity Y)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    0x0A, 0x55, 0x04,  //   Usage (Angular Velocity Z)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    0xC0,              // End Collection
};

#define IMU_HID_REPORT_DESC_SIZE sizeof(imu_hid_report_desc)

// Complete IMU report structure with processed orientation + raw sensor data
typedef struct {
    // Processed orientation data (for easy joystick use)
    uint16_t yaw;       // Yaw angle (0-65535 representing 0 to 360 degrees)
    uint16_t pitch;     // Pitch angle (0-65535 representing -90 to +90 degrees)
    uint16_t roll;      // Roll angle (0-65535 representing -180 to +180 degrees)
    uint16_t magnitude; // High-pass filtered acceleration magnitude (0-65535)
    
    // Raw sensor data (DualShock 4 compatible)
    int16_t accel_x;    // Raw accelerometer X (-32768 to +32767, ±4g, 8192 LSB/g)
    int16_t accel_y;    // Raw accelerometer Y (-32768 to +32767, ±4g, 8192 LSB/g)
    int16_t accel_z;    // Raw accelerometer Z (-32768 to +32767, ±4g, 8192 LSB/g)
    int16_t gyro_x;     // Raw gyroscope X (-32768 to +32767, ±2000°/s, 16.384 LSB/°/s)
    int16_t gyro_y;     // Raw gyroscope Y (-32768 to +32767, ±2000°/s, 16.384 LSB/°/s) 
    int16_t gyro_z;     // Raw gyroscope Z (-32768 to +32767, ±2000°/s, 16.384 LSB/°/s)
} __attribute__((packed)) imu_report_t;

#endif // _IMU_DESCRIPTOR_H_ 