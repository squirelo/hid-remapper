#ifndef _IMU_DESCRIPTOR_H_
#define _IMU_DESCRIPTOR_H_

#include <stdint.h>

// Custom HID Report Descriptor for 6-axis IMU data (accelerometer + gyroscope)
static const uint8_t imu_hid_report_desc[] = {
    0x05, 0x20,        // Usage Page (Sensor)
    0x09, 0x73,        // Usage (Motion: Accelerometer + Gyroscope 3D)
    0xA1, 0x01,        // Collection (Application)
    
    // Accelerometer data
    0x05, 0x20,        //   Usage Page (Sensor)
    0x09, 0x53,        //   Usage (Acceleration Axis X)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8 bits)
    0x95, 0x01,        //   Report Count (1)
    0x55, 0x0E,        //   Unit Exponent (-2)
    0x65, 0x11,        //   Unit (SI Linear: Meter/Second²)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Y-axis acceleration  
    0x09, 0x54,        //   Usage (Acceleration Axis Y)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Z-axis acceleration
    0x09, 0x55,        //   Usage (Acceleration Axis Z)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Gyroscope data
    0x09, 0x56,        //   Usage (Angular Velocity Axis X)
    0x55, 0x0F,        //   Unit Exponent (-1)
    0x65, 0x12,        //   Unit (SI Rotation: Radians/Second)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Y-axis angular velocity
    0x09, 0x57,        //   Usage (Angular Velocity Axis Y)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Z-axis angular velocity
    0x09, 0x58,        //   Usage (Angular Velocity Axis Z)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    0xC0,              // End Collection
};

#define IMU_HID_REPORT_DESC_SIZE sizeof(imu_hid_report_desc)

// 6-axis IMU report structure (accelerometer + gyroscope)
typedef struct {
    uint8_t accel_x;  // X-axis acceleration
    uint8_t accel_y;  // Y-axis acceleration  
    uint8_t accel_z;  // Z-axis acceleration
    uint8_t gyro_x;   // X-axis angular velocity
    uint8_t gyro_y;   // Y-axis angular velocity
    uint8_t gyro_z;   // Z-axis angular velocity
} __attribute__((packed)) imu_report_t;

#endif // _IMU_DESCRIPTOR_H_ 